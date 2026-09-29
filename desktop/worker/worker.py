# SPDX-License-Identifier: GPL-2.0-or-later
"""One serial session per operation. stdout is a versioned JSON-lines protocol.

No shell, CLI-output parsing, port guessing, or firmware writes before consent.
"""
import contextlib
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import time
import uuid

SIZE = 16 * 1024 * 1024
CHUNK = 256 * 1024
IMAGES = [(0, "bootloader.bin"), (0x8000, "partitions.bin"),
          (0xe000, "boot_app0.bin"), (0x10000, "app.bin")]


class SafetyError(Exception):
    pass


def choose_connected_port(devices, requested):
    devices = list(devices)
    current = next((p for p in devices if p.device.upper() == requested.upper()
                    and ((p.vid == 0x031F and p.pid == 0x9001) or p.vid == 0x303A)), None)
    if current is not None:
        return current
    running = [p for p in devices if p.vid == 0x031F and p.pid == 0x9001]
    if len(running) == 1:
        return running[0]
    loaders = [p for p in devices if p.vid == 0x303A]
    if not running and len(loaders) == 1:
        return loaders[0]
    if len(running) > 1 or len(loaders) > 1:
        raise SafetyError("Найдено несколько подходящих устройств. Оставьте подключённым один тиллер.")
    raise SafetyError("Тиллер не найден. Переподключите USB.")


def resolve_loader_port(requested):
    """Put the Tiller into ROM mode; esptool will perform the actual port discovery."""
    import serial
    from serial.tools import list_ports

    def ports():
        return list(list_ports.comports())

    current = choose_connected_port(ports(), requested)
    if current.vid == 0x303A:
        return None
    if current.vid != 0x031F or current.pid != 0x9001:
        raise SafetyError("Выбранный COM-порт не принадлежит тиллеру.")
    previous_loaders = {p.device.upper() for p in ports() if p.vid == 0x303A}
    try:
        connection = serial.Serial(current.device, 1200)
        connection.setDTR(False)
        time.sleep(0.25)
        with contextlib.suppress(Exception):
            connection.setRTS(True)
            time.sleep(0.05)
            connection.setRTS(False)
        connection.close()
    except Exception as error:
        raise SafetyError("Не удалось перевести тиллер в режим обновления: " + str(error)) from error
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        candidates = [p.device for p in ports() if p.vid == 0x303A]
        appeared = [device for device in candidates if device.upper() not in previous_loaders]
        if len(appeared) == 1:
            return None
        if candidates and not previous_loaders:
            return None
        time.sleep(0.25)
    raise SafetyError("Загрузчик не появился. Выполните BOOT + RST и повторите обновление.")


def emit(kind, **data):
    sys.__stdout__.write(json.dumps({"v": 1, "type": kind, **data}, ensure_ascii=True) + "\n")
    sys.__stdout__.flush()


def sha(data):
    return hashlib.sha256(data).hexdigest()


def package_images(root, firmware_file=""):
    root = Path(root)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8-sig"))
    result = []
    for address, name in IMAGES:
        key = "firmware/" + name
        entries = [x for x in manifest["files"] if x["path"] == key]
        data = (root / key).read_bytes()
        if len(entries) != 1 or sha(data) != entries[0]["sha256"].lower():
            raise SafetyError("Повреждён комплект прошивки: " + name)
        if not data or address + len(data) > SIZE:
            raise SafetyError("Недопустимый размер образа: " + name)
        result.append((address, data))
    if firmware_file:
        override = Path(firmware_file)
        if override.suffix.lower() != ".bin" or not override.is_file():
            raise SafetyError("Выбранный файл прошивки должен быть существующим .bin-файлом.")
        data = override.read_bytes()
        if not data or data[0] != 0xE9 or 0x10000 + len(data) > SIZE:
            raise SafetyError("Выбранный файл не похож на допустимый образ приложения ESP32-S3.")
        result[-1] = (0x10000, data)
    for (address, data), (next_address, _) in zip(result, result[1:]):
        if address + len(data) > next_address:
            raise SafetyError("Образы прошивки перекрываются.")
    return result


def restore_image(meta_path, mac):
    path = Path(meta_path)
    meta = json.loads(path.read_text(encoding="utf-8-sig"))
    if meta.get("mac", "").lower() != mac or meta.get("bytes") != SIZE:
        raise SafetyError("Этот бэкап относится к другому устройству или имеет неверный размер.")
    data = (path.parent / "before-flash.bin").read_bytes()
    if len(data) != SIZE or sha(data) != str(meta.get("sha256", "")).lower():
        raise SafetyError("Бэкап повреждён: размер или SHA-256 не совпадает.")
    return [(0, data)]


def check_identity(esp, api, expected=""):
    if esp.CHIP_NAME != "ESP32-S3":
        raise SafetyError("Нужен ESP32-S3; обнаружен " + esp.CHIP_NAME)
    if esp.secure_download_mode or esp.get_secure_boot_enabled() or esp.get_flash_encryption_enabled():
        raise SafetyError("Защищённый чип: запись этой прошивки запрещена.")
    api.attach_flash(esp)
    if api.detect_flash_size(esp) != "16MB":
        raise SafetyError("Нужна flash-память 16 МБ.")
    mac = ":".join(f"{x:02x}" for x in esp.read_mac())
    if expected and mac != expected.lower():
        raise SafetyError("Подключён другой тиллер. Запись заблокирована.")
    return mac


def atomic_json(path, value):
    temp = path.with_suffix(".json.tmp")
    with temp.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temp.replace(path)


def flash_fingerprint(esp):
    value = esp.flash_md5sum(0, SIZE)
    if not isinstance(value, str) or not re.fullmatch(r"[a-fA-F0-9]{32}", value):
        raise SafetyError("Не удалось получить контрольную сумму flash.")
    return value.lower()


def backup(esp, base, mac, baud, send):
    base = Path(base)
    base.mkdir(parents=True, exist_ok=True)
    send("stage", phase="fingerprint", message="Проверяю содержимое flash для безопасной докачки…")
    fingerprint = flash_fingerprint(esp)
    folder, state = None, None
    for candidate in sorted(base.iterdir(), reverse=True):
        if not candidate.is_dir() or (candidate / "backup.json").exists():
            continue
        try:
            saved = json.loads((candidate / "resume.json").read_text(encoding="utf-8"))
            if (saved.get("mac") == mac and saved.get("fingerprint") == fingerprint
                    and saved.get("chunk_size") == CHUNK and saved.get("bytes") == SIZE
                    and isinstance(saved.get("chunks"), dict)):
                folder, state = candidate, saved
                break
        except (OSError, ValueError):
            continue
    if folder is None:
        folder = base / (time.strftime("%Y%m%d-%H%M%S-") + mac.replace(":", "") + "-" + uuid.uuid4().hex[:8])
        folder.mkdir(exist_ok=False)
        state = dict(mac=mac, fingerprint=fingerprint, chunk_size=CHUNK, bytes=SIZE, chunks={})
        atomic_json(folder / "resume.json", state)
    chunks = folder / "chunks"
    chunks.mkdir(exist_ok=True)
    valid = set()
    for index in range(SIZE // CHUNK):
        block = chunks / f"{index:03}.bin"
        if str(index) in state["chunks"] and block.is_file():
            data = block.read_bytes()
            if len(data) == CHUNK and sha(data) == state["chunks"][str(index)]:
                valid.add(index)
    completed = len(valid) * CHUNK
    send("resume", current=completed, total=SIZE, path=str(folder))
    send("stage", phase="backup", message="Читаю недостающие блоки бэкапа…")
    send("progress", phase="backup", current=completed, total=SIZE)
    for index in range(SIZE // CHUNK):
        if index in valid:
            continue
        last_update = 0.0
        def progress(count, length, start):
            nonlocal last_update
            now = time.monotonic()
            if now - last_update >= 0.1 or count == length:
                send("progress", phase="backup", current=completed + count, total=SIZE)
                last_update = now
        # Stub checks each received chunk's MD5. Only fsynced blocks enter resume.json.
        data = esp.read_flash(index * CHUNK, CHUNK, progress)
        if len(data) != CHUNK:
            raise SafetyError("Получен неполный блок бэкапа.")
        block = chunks / f"{index:03}.bin"
        temp = block.with_suffix(".partial")
        with temp.open("wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        temp.replace(block)
        state["chunks"][str(index)] = sha(data)
        atomic_json(folder / "resume.json", state)
        completed += CHUNK
    partial = folder / "before-flash.partial"
    digest, md5 = hashlib.sha256(), hashlib.md5(usedforsecurity=False)
    with partial.open("wb") as stream:
        for index in range(SIZE // CHUNK):
            data = (chunks / f"{index:03}.bin").read_bytes()
            if len(data) != CHUNK or sha(data) != state["chunks"][str(index)]:
                raise SafetyError("Сохранённый блок бэкапа повреждён.")
            stream.write(data)
            digest.update(data)
            md5.update(data)
        stream.flush()
        os.fsync(stream.fileno())
    if md5.hexdigest() != fingerprint or flash_fingerprint(esp) != fingerprint:
        raise SafetyError("Flash изменилась во время бэкапа; использовать его нельзя.")
    if partial.stat().st_size != SIZE or sha(partial.read_bytes()) != digest.hexdigest():
        raise SafetyError("Не удалось проверить бэкап на диске.")
    image = folder / "before-flash.bin"
    partial.replace(image)
    meta = {"mac": mac, "bytes": SIZE, "sha256": digest.hexdigest(), "baud": baud,
            "saved": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "note": "Firmware before this operation; not necessarily factory firmware."}
    atomic_json(folder / "backup.json", meta)
    send("backup", path=str(folder / "backup.json"))
    return folder / "backup.json"


def run(config, api, send=emit, consent=None, port_resolver=resolve_loader_port):
    action = config.get("action")
    if action not in ("check", "install", "restore", "backup"):
        raise SafetyError("Неизвестная операция.")
    images = package_images(config["package"], config.get("firmware_file", ""))
    if action == "check":
        send("done", message="Комплект проверен. Подключения к устройству не было.")
        return
    port = config.get("port", "")
    baud = config.get("baud", 460800)
    if not re.fullmatch(r"COM[1-9][0-9]{0,3}", port) or baud not in (115200, 460800, 921600, 1500000, 2000000):
        raise SafetyError("Недопустимый COM-порт или скорость.")
    send("stage", phase="connect", message="Перевожу тиллер в режим обновления…")
    esp = None
    try:
        port = port_resolver(port)
        if port is None:
            # Official esptool discovery follows the re-enumerated ROM port instead of
            # trusting the stale COM number from the running CDC firmware.
            esp = api.connect_esp(port=None, chip="esp32s3", initial_baud=115200,
                                  port_filter=["vid=0x303A"], before="no-reset",
                                  connect_attempts=3, open_port_attempts=2)
        else:
            # Injectable path used by deterministic transport tests.
            esp = api.detect_chip(port=port, baud=115200, connect_mode="no-reset", connect_attempts=3)
        mac = check_identity(esp, api, config.get("expected_mac", ""))
        send("device", mac=mac, port=port)
        if action == "restore":
            images = restore_image(config["restore"], mac)
        esp = esp.run_stub()
        api.attach_flash(esp)
        esp.flash_set_parameters(SIZE)
        if baud != 115200:
            esp.change_baud(baud)
        make_backup = action in ("backup", "restore") or config.get("make_backup", True)
        if make_backup:
            send("stage", phase="backup", message="Сохраняю текущую прошивку · 16 МиБ")
            backup(esp, config["backups"], mac, baud, send)
        if action == "backup":
            send("done", message="Бэкап сохранён и проверен. Flash-память не изменялась.")
            return
        send("ready", mac=mac, message="Можно начинать запись." if not make_backup else "Бэкап проверен. Можно начинать запись.")
        if not consent or not consent():
            send("cancelled", message="Запись отменена. Бэкап сохранён.")
            return
        # Recheck the physical device on the SAME open session before any erase/write.
        if ":".join(f"{x:02x}" for x in esp.read_mac()) != mac:
            raise SafetyError("Устройство изменилось перед записью.")
        # Images are immutable bytes validated before connecting (no TOCTOU on files).
        # esptool otherwise reconnects and retries writes internally after SerialException.
        # A new connection needs fresh identity checks and explicit user action.
        esp.WRITE_FLASH_ATTEMPTS = 1
        send("writing", message="Запись началась. Не отключайте USB.")
        api.write_flash(esp, images, flash_mode="keep", flash_freq="keep", flash_size="keep", compress=True)
        send("stage", phase="verify", message="Сверяю записанные образы…")
        api.verify_flash(esp, images, flash_mode="keep", flash_freq="keep", flash_size="keep")
        send("done", message="Прошивка записана и проверена. Отключите USB, закройте корпус. Подключите БП 12 В, затем USB; оставьте ручке свободный ход.")
    finally:
        if esp is not None:
            with contextlib.suppress(Exception):
                esp._port.close()


def main():
    # Library chatter goes to stderr; only our events use stdout.
    sys.stdout = sys.stderr
    import esptool
    from esptool.logger import TemplateLogger, log

    class Logger(TemplateLogger):
        def print(self, *args, **kwargs):
            print(*args, file=sys.stderr, flush=True)
        def note(self, message): self.print(message)
        def warning(self, message): self.print(message)
        def error(self, message): self.print(message)
        def stage(self, finish=False): pass
        def set_verbosity(self, verbosity): pass
        def progress_bar(self, cur_iter, total_iters, prefix="", suffix="", bar_length=30):
            emit("progress", phase="transfer", current=cur_iter, total=total_iters)

    log.set_logger(Logger())
    writing = False
    def send(kind, **data):
        nonlocal writing
        if kind == "writing": writing = True
        emit(kind, **data)
    try:
        line = sys.stdin.readline(65537)
        if len(line) > 65536:
            raise SafetyError("Слишком большой запрос.")
        config = json.loads(line)
        run(config, esptool, send, lambda: sys.stdin.readline(64).strip() == "WRITE")
        return 0
    except Exception as error:
        emit("error", message=str(error), writing=writing,
             retryable=not writing and not isinstance(error, (SafetyError, OSError, ValueError, KeyError)))
        import traceback
        traceback.print_exc(file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
