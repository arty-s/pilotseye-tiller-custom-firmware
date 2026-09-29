import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location("worker", Path(__file__).parents[1] / "worker/worker.py")
w = importlib.util.module_from_spec(spec)
spec.loader.exec_module(w)


class FakeEsp:
    CHIP_NAME = "ESP32-S3"
    secure_download_mode = False
    def __init__(self):
        self.closed = False
        self._port = self
        self.reads = 0
        self.fail_read = False
        self.switch_mac = False
        self.mac_calls = 0
    def close(self): self.closed = True
    def get_secure_boot_enabled(self): return False
    def get_flash_encryption_enabled(self): return False
    def read_mac(self):
        self.mac_calls += 1
        return bytes.fromhex("aabbccddeeff" if not self.switch_mac or self.mac_calls == 1 else "112233445566")
    def run_stub(self): return self
    def flash_set_parameters(self, size): assert size == w.SIZE
    def change_baud(self, baud): pass
    def flash_md5sum(self, offset, size): return hashlib.md5(bytes(size)).hexdigest()
    def read_flash(self, offset, size, progress):
        if self.fail_read and self.reads == 1: raise TimeoutError("cable removed")
        self.reads += 1
        progress(size, size, offset)
        return bytes(size)


class FakeApi:
    def __init__(self):
        self.esp = FakeEsp()
        self.connections = 0
        self.writes = 0
        self.verifies = 0
        self.fail_verify = False
    def detect_chip(self, **kwargs):
        self.connections += 1
        # Reproduces the serial handoff failure: a second process loses the port.
        if self.connections > 1: raise OSError("COM disappeared between processes")
        return self.esp
    def attach_flash(self, esp): pass
    def detect_flash_size(self, esp): return "16MB"
    def write_flash(self, esp, images, **kwargs):
        assert not esp.closed
        assert esp.WRITE_FLASH_ATTEMPTS == 1
        self.writes += 1
    def verify_flash(self, esp, images, **kwargs):
        self.verifies += 1
        if self.fail_verify: raise RuntimeError("MD5 mismatch")


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "firmware").mkdir()
        files = []
        for _, name in w.IMAGES:
            data = b"image" + name.encode()
            (self.root / "firmware" / name).write_bytes(data)
            files.append({"path": "firmware/" + name, "sha256": w.sha(data)})
        (self.root / "manifest.json").write_text(json.dumps({"files": files}))
        self.config = dict(action="install", package=str(self.root), port="COM17", baud=460800,
                           backups=str(self.root / "backups"))
        self.api = FakeApi()
        self.events = []
    def run_worker(self, consent=True):
        w.run(self.config, self.api, lambda kind, **data: self.events.append((kind, data)), lambda: consent,
              lambda port: port)
    def test_old_handoff_is_red_capable(self):
        self.api.detect_chip()
        with self.assertRaisesRegex(OSError, "COM disappeared"):
            self.api.detect_chip()
    def test_new_session_never_reopens_and_checks_backup_before_consent(self):
        self.run_worker()
        self.assertEqual(self.api.connections, 1)
        self.assertEqual(self.api.writes, 1)
        self.assertEqual(self.api.verifies, 1)
        types = [x[0] for x in self.events]
        self.assertLess(types.index("backup"), types.index("ready"))
        self.assertLess(types.index("ready"), types.index("writing"))
        self.assertEqual(types[-1], "done")
        self.assertTrue(self.api.esp.closed)
        self.assertEqual(len(list((self.root / "backups").rglob("backup.json"))), 1)
    def test_disconnect_never_marks_partial_backup_or_writes(self):
        self.api.esp.fail_read = True
        with self.assertRaises(TimeoutError): self.run_worker()
        self.assertEqual(self.api.writes, 0)
        self.assertFalse(list(self.root.rglob("backup.json")))
        self.assertTrue(self.api.esp.closed)
    def test_cancel_preserves_backup_without_write(self):
        self.run_worker(False)
        self.assertEqual(self.api.writes, 0)
        self.assertEqual(self.events[-1][0], "cancelled")
        self.assertTrue(list(self.root.rglob("backup.json")))
    def test_install_can_skip_backup(self):
        self.config["make_backup"] = False
        self.run_worker()
        self.assertEqual(self.api.writes, 1)
        self.assertFalse(list(self.root.rglob("backup.json")))
        self.assertNotIn("backup", [event[0] for event in self.events])
    def test_external_app_image_replaces_only_app(self):
        custom = self.root / "custom.bin"
        custom.write_bytes(b"\xe9custom-app")
        images = w.package_images(self.root, custom)
        self.assertEqual(images[-1], (0x10000, b"\xe9custom-app"))
        self.assertEqual([name for name, _ in images[:-1]], [0, 0x8000, 0xe000])
    def test_invalid_external_app_is_rejected(self):
        custom = self.root / "custom.bin"
        custom.write_bytes(b"not-an-esp-image")
        with self.assertRaises(w.SafetyError):
            w.package_images(self.root, custom)
    def test_resume_skips_verified_chunk_after_disconnect(self):
        self.api.esp.fail_read = True
        with self.assertRaises(TimeoutError): self.run_worker()
        self.api = FakeApi()
        self.run_worker()
        self.assertEqual(self.api.esp.reads, w.SIZE // w.CHUNK - 1)
        self.assertEqual(len(list(self.root.rglob("backup.json"))), 1)
        self.assertTrue(any(k == "resume" and d["current"] == w.CHUNK for k, d in self.events))
    def test_resume_rereads_corrupt_local_chunk(self):
        self.api.esp.fail_read = True
        with self.assertRaises(TimeoutError): self.run_worker()
        next(self.root.rglob("000.bin")).write_bytes(b"corrupt")
        self.api = FakeApi()
        self.run_worker()
        self.assertEqual(self.api.esp.reads, w.SIZE // w.CHUNK)
    def test_changed_flash_never_mixes_old_blocks(self):
        self.api.esp.fail_read = True
        with self.assertRaises(TimeoutError): self.run_worker()
        self.api = FakeApi()
        self.api.esp.flash_md5sum = lambda offset, size: "a" * 32
        with self.assertRaises(w.SafetyError): self.run_worker()
        self.assertEqual(self.api.esp.reads, w.SIZE // w.CHUNK)
        self.assertEqual(self.api.writes, 0)
        self.assertFalse(list(self.root.rglob("backup.json")))
    def test_changed_device_is_blocked_before_write(self):
        self.api.esp.switch_mac = True
        with self.assertRaises(w.SafetyError): self.run_worker()
        self.assertEqual(self.api.writes, 0)
    def test_retry_with_wrong_mac_is_blocked_before_backup(self):
        self.config["expected_mac"] = "11:22:33:44:55:66"
        with self.assertRaises(w.SafetyError): self.run_worker()
        self.assertEqual(self.api.esp.reads, 0)
    def test_bad_package_never_opens_port(self):
        (self.root / "firmware/app.bin").write_bytes(b"bad")
        with self.assertRaises(w.SafetyError): self.run_worker()
        self.assertEqual(self.api.connections, 0)
    def test_verification_error_never_reports_success_or_retries_write(self):
        self.api.fail_verify = True
        with self.assertRaises(RuntimeError): self.run_worker()
        self.assertEqual(self.api.writes, 1)
        self.assertNotIn("done", [e[0] for e in self.events])
    def test_restore_wrong_mac_or_hash_rejected(self):
        self.config["action"] = "restore"
        meta = self.root / "backup.json"
        meta.write_text(json.dumps(dict(mac="11:22:33:44:55:66", bytes=w.SIZE, sha256="0"*64)))
        self.config["restore"] = str(meta)
        with self.assertRaises(w.SafetyError): self.run_worker()
        self.assertEqual(self.api.writes, 0)
        with self.assertRaises(w.SafetyError):
            meta.write_text(json.dumps(dict(mac="aa:bb:cc:dd:ee:ff", bytes=w.SIZE, sha256="0"*64)))
            (self.root / "before-flash.bin").write_bytes(bytes(w.SIZE))
            w.restore_image(meta, "aa:bb:cc:dd:ee:ff")
    def test_restore_creates_rescue_backup_and_uses_single_session(self):
        self.config["action"] = "backup"
        self.run_worker()
        self.api = FakeApi()
        self.config["action"] = "restore"
        self.config["restore"] = str(next(self.root.rglob("backup.json")))
        self.run_worker()
        self.assertEqual(self.api.writes, 1)
        self.assertEqual(len(list(self.root.rglob("backup.json"))), 2)
    def test_check_never_touches_device(self):
        self.config["action"] = "check"
        self.run_worker()
        self.assertEqual(self.api.connections, 0)
    def test_network_ports_not_allowed(self):
        self.config["port"] = "rfc2217://example.com:1234"
        with self.assertRaises(w.SafetyError): self.run_worker()
        self.assertEqual(self.api.connections, 0)
    def test_discovery_follows_tiller_when_com_number_changes(self):
        devices = [SimpleNamespace(device="COM14", vid=0x031F, pid=0x9001)]
        self.assertEqual(w.choose_connected_port(devices, "COM6").device, "COM14")
        loader = [SimpleNamespace(device="COM18", vid=0x303A, pid=0x1001)]
        self.assertEqual(w.choose_connected_port(loader, "COM6").device, "COM18")


if __name__ == "__main__": unittest.main(verbosity=2)
