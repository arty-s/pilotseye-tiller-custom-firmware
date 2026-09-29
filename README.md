# Custom firmware for the PilotsEYE / Simplayer USB tiller

**Unofficial.** Not affiliated with or endorsed by PilotsEYE or Simplayer.
Flashing is at your own risk. [Русская версия](README.ru.md)

Open-source replacement firmware and a one-click Windows updater for the
motorized nosewheel-steering tiller sold as the **Simplayer A320/B737 USB tiller**.
On USB it shows up as `PilotsEYE-Studios` / `PilotsEYE-` (VID:PID `031F:9001`).

## The device

A tiller for flight simulators (MSFS, X-Plane and others) with a motor that
returns the handle to center. Inside:

- ESP32-S3 module (16 MB flash, 8 MB PSRAM);
- a brushless gimbal motor with a DRV8313 driver, powered by an external supply;
- an AS5600 magnetic angle sensor;
- two knobs (red and blue) to adjust the centering feel, and a **DISC** (pedal
  disconnect) button on the handle.

To the PC it is a USB game controller with one axis and one button. It ships
with a 12 V / 2 A power supply and is sold, for example, in the
[Simplayer official store on AliExpress](https://aliexpress.ru/item/1005010218144117.html).

## What the stock firmware gets wrong, and what this firmware fixes

| Stock firmware | This firmware |
|---|---|
| Randomly stops responding in the sim and stays dead until it is reset. | Fixed at the cause. Sensor reads have timeouts and I²C bus recovery, so the control loop can no longer hang. The USB stack was moved to current Arduino-ESP32 3.3 / ESP-IDF 5.5 / TinyUSB 0.21. Watchdogs cover every task. If USB ever stalls, only USB reconnects: the motor keeps centering and the calibration is kept. |
| The knobs are read only at power-up; turning them does nothing until a restart. | The knobs work live. |
| Allows up to 12 V on the motor, more than a 12 V supply can deliver cleanly, so strong returns can feel notchy. | The voltage is capped at the clean maximum (6.9 V at 12 V): the same strength, smooth. |
| No way to see what the device is doing. | A USB serial console with status, diagnostics and tuning. |

The centering feel is the factory one: the same control law and the same knob
ranges, reconstructed from the stock behavior. The red knob sets the return
speed and holding force; the blue knob sets the stiffness and damping.

Also:

- after a minute untouched the motor switches off completely (silent, no heat);
  moving the handle wakes it, and the spring fades in over 0.2 s, so you don't feel it;
- the travel is calibrated automatically at every power-up (the motor gently
  drives to both end stops and takes the middle as center);
  holding DISC for 1.5 s recalibrates;
- the USB identity (VID/PID, names, HID descriptor) is identical to the stock
  one, so the simulator sees the same device and your bindings stay.

## Installing

The updater's interface is in Russian; the button names are given below.

### First install (over the stock firmware)

The stock firmware has no way to be switched into the bootloader over USB, so
the first time you need the buttons on the ESP32 module inside:

1. Unplug the motor power supply and USB, remove the bottom cover.
2. Download `Tiller-Updater-…zip` from [Releases](../../releases/latest), unpack
   it and run `Tiller.exe`.
3. Connect the tiller with USB (the port you normally use for the sim).
4. On the ESP32 module hold **BOOT**, press and release **RST**, then release
   **BOOT**. The updater shows the green `<COM port> найден` ("found").
5. Keep the backup option on: the updater first saves the full 16 MB flash image
   to `%LOCALAPPDATA%\Tiller\backups\…\before-flash.bin`, which lets you return
   to the stock firmware.
6. Press `Обновить` ("Update") and don't unplug USB until `Готово` ("Done").
7. Put the cover back, plug the motor supply in, then unplug and replug USB.
   The tiller starts with the new firmware and calibrates its travel (it can't
   without motor power at start-up).

### Later updates

Just run the new updater with the tiller connected and press `Обновить`: this
firmware switches itself into the bootloader over USB, no need to open the case.
If an update is ever interrupted, use **BOOT** + **RST** as above and run it again.

**Requirements:** Windows 10/11 x64 and a **12 V** motor supply. The firmware
is tuned for 12 V; the device accepts up to 24 V, but at 24 V the motor would
get twice the intended voltage.

### Back to stock

Write the backup made by the updater back with esptool (in bootloader mode):

```bash
python -m esptool --chip esp32s3 write-flash 0x0 before-flash.bin
```

## Repository

| Path | Contents |
|---|---|
| `firmware/piloteye_tiller/` | Firmware source (Arduino / SimpleFOC). Build: [`firmware/README.md`](firmware/README.md) |
| `firmware/tests/` | Unit tests and closed-loop model tests (`python -m unittest discover -s firmware/tests`) |
| `firmware/diag-recordings/` | Motor and loop simulator used by the tests, fitted to a hardware recording |
| `firmware/tiller_cmd.py` | Send a console command and print the reply |
| `desktop/` | Windows updater: .NET 10 / Avalonia UI plus an esptool-based worker |

Tested on the author's unit. If the knobs on your unit act differently, please
open an issue.

## License

MIT, see [`LICENSE`](LICENSE). Third-party components and their licenses:
[`THIRD-PARTY.md`](THIRD-PARTY.md).
