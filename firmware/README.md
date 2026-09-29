# Firmware

Arduino sketch for the ESP32-S3 inside the tiller: SimpleFOC voltage-torque
control of the centering motor, a USB HID game controller (one axis, one
button) and a USB serial console.

## Build

Toolchain: [Arduino CLI](https://arduino.github.io/arduino-cli/) 1.5,
Arduino-ESP32 core **3.3.12**, library **SimpleFOC 2.4.0**
(SimpleFOC 2.3.x fails in `initFOC()` on ESP-IDF 5.5).

```bash
arduino-cli core install esp32:esp32@3.3.12
arduino-cli lib install "Simple FOC@2.4.0"
arduino-cli compile --output-dir build --fqbn "esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,FlashMode=dio,FlashSize=16M,PSRAM=opi,PartitionScheme=huge_app,CPUFreq=240" firmware/piloteye_tiller
```

All code is in `piloteye_tiller.cpp`; the `.ino` is intentionally empty (it
avoids Arduino's prototype generation). Keep `CDCOnBoot=default`: with
`CDCOnBoot=cdc` USB starts before `setup()` and the device enumerates with the
Espressif VID instead of `031F:9001`.

Run the tests (Python 3, no dependencies):

```bash
python -m unittest discover -s firmware/tests
```

## Flash

The easiest way is the Windows updater from the releases page: it also makes a
full backup first. By hand, with esptool, only the application at `0x10000` is
needed (the release's `bootloader.bin` / `partitions.bin` / `boot_app0.bin` go
to `0x0` / `0x8000` / `0xe000` for a first install over the stock firmware):

```bash
python firmware/touch_to_bootloader.py
python -m esptool --chip esp32s3 --before no-reset --after watchdog-reset write-flash -z 0x10000 app.bin
```

`touch_to_bootloader.py` opens the tiller's serial port at 1200 baud, which
restarts it into the ROM bootloader (a new COM port with VID `303A` appears).

## Serial console

Any terminal at 115200 on the `PilotsEYE-` COM port, or
`python firmware/tiller_cmd.py <command>` (needs `pyserial`).

| Command | Action |
|---|---|
| `S` | status: angle, axis, voltages, knobs, calibration, sensor health, sleep state |
| `X` | diagnostics: task heartbeats, HID/USB counters, the last saved fault |
| `A` | recalibrate: drive to both end stops, center = middle (also done at every power-up) |
| `C` | manual travel calibration (sweep the handle by hand for 6 s) |
| `O<n>` | control mode: `O2` factory law (default), `O0` spring + damper with an observer, `O1` spring + damper with a filtered derivative |
| `Os<s>` | switch the motor off after `s` seconds untouched (default 60, `Os0` = never) |
| `L` | toggle live knobs / manual values from the console |
| `F<V>` `D<x>` `P<x>` | voltage cap, damping, spring stiffness (modes 0/1, with live knobs off) |
| `N` `W` `Y` | damper velocity dead zone, observer bandwidth, complementary split (modes 0/1) |
| `R` | record 1500 control-loop samples (time, angle, velocity, voltage) and print them |
| `J[rad]` | step test: shift the center by `rad` for 0.4 s and record the response |
| `K` | raw ADC of the knob pins |
| `I` / `Q` | AS5600 magnet diagnostics / sensor filter registers |
| `U` | test USB-only recovery (reconnect without restarting the motor loop) |
| `H` | toggle the status stream |
| `M…` | SimpleFOC's own motor commands |

Console settings are not saved: a restart returns the defaults. Only the
travel calibration and the last fault snapshot are kept in flash.

## Hardware

| Function | Pins |
|---|---|
| Motor driver DRV8313 | PWM A/B/C = GPIO 4/5/6, EN = GPIO 7; 14 pole pairs |
| AS5600 angle sensor | I²C 0x36, SDA = GPIO 42, SCL = GPIO 41, 400 kHz |
| Handle button | GPIO 38, active low |
| Red / blue knob | ADC GPIO 1 / GPIO 2 |

USB: VID `031F`, PID `9001`, "PilotsEYE-Studios" / "PilotsEYE-", HID report ID 3,
axis 0..65535, plus a CDC serial interface.

## Robustness

- The AS5600 is read with an I²C timeout and a 9-clock bus recovery; on an
  error the last good angle is used, so the loop never blocks.
- The motor and HID tasks run under the task watchdog. A supervisor saves a
  diagnostic snapshot (`X`) and restarts only when a task stops for 5 s.
- If HID reports stop getting through while everything else runs, only USB is
  disconnected and reconnected; the motor loop and the calibration continue.
