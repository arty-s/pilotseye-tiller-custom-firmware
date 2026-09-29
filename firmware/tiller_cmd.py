#!/usr/bin/env python
"""Send a Commander command to the PilotsEYE tiller over its USB serial (CDC) and print the reply.

Usage (needs pyserial):
    python tiller_cmd.py S        # status
    ... tiller_cmd.py "F3"     # set force 3V     ... tiller_cmd.py A   # auto-calibrate
    ... tiller_cmd.py I        # AS5600 magnet diag           ... tiller_cmd.py K   # raw knob ADC

Finds the tiller by USB VID:PID 031F:9001 (its CDC interface). 115200 baud.
Any serial terminal (PuTTY / Arduino Serial Monitor) at 115200 on the "PilotsEYE-" COM port works too.
"""
import sys, time
import serial
from serial.tools import list_ports

cmd = sys.argv[1] if len(sys.argv) > 1 else "S"
port = None
for p in list_ports.comports():
    if p.vid == 0x031F and p.pid == 0x9001:
        port = p.device
if not port:
    print("PilotsEYE tiller CDC port (031F:9001) not found - plugged in and on CDC firmware?")
    sys.exit(1)

s = serial.Serial(port, 115200, timeout=1)
time.sleep(0.3)
s.reset_input_buffer()
s.write((cmd + "\n").encode())
time.sleep(0.45)
print(s.read(3000).decode("utf-8", "replace").strip() or "(no response)")
s.close()
