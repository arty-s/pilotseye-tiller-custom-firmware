"""Reboot the running CDC firmware into ROM download mode via the 1200-baud touch.
No buttons needed. Exit 0 = touched; 1 = no CDC port (use manual BOOT+RST); 2 = error."""
import sys, time
try:
    import serial
    from serial.tools import list_ports
except Exception as e:
    print("pyserial missing:", e); sys.exit(2)

port = None
# explicit port override: touch_to_bootloader.py COM6
if len(sys.argv) > 1 and sys.argv[1].upper().startswith("COM"):
    port = sys.argv[1]
else:
    for p in list_ports.comports():
        if p.vid == 0x031F and p.pid == 0x9001:
            port = p.device; break
if not port:
    print("No 031F:9001 CDC port found (device not on CDC firmware / already in bootloader).")
    sys.exit(1)

print("Touching %s @1200 baud -> download mode ..." % port)
try:
    s = serial.Serial(port, 1200)
    s.setDTR(False); time.sleep(0.25)
    try: s.setRTS(True); time.sleep(0.05); s.setRTS(False)
    except Exception: pass
    s.close()
except Exception as e:
    print("touch error:", e); sys.exit(2)
print("done")
sys.exit(0)
