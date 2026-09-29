# Third-party components

The code in this repository is MIT-licensed (see `LICENSE`). The binaries built
from it include the following third-party components under their own licenses.

## Firmware (`firmware/`, `app.bin`)

| Component | Version | License |
|---|---|---|
| [Arduino-ESP32 core](https://github.com/espressif/arduino-esp32/tree/3.3.12) | 3.3.12 | LGPL-2.1 |
| [ESP-IDF](https://github.com/espressif/esp-idf/tree/v5.5.5) | 5.5.5 | Apache-2.0 |
| [TinyUSB](https://github.com/hathach/tinyusb) (bundled with the core) | 0.21 | MIT |
| [SimpleFOC / Arduino-FOC](https://github.com/simplefoc/Arduino-FOC/tree/v2.4.0) | 2.4.0 | MIT |

`bootloader.bin` and `partitions.bin` come from an Arduino-ESP32 2.0.9 build
(ESP-IDF 4.4.7, Apache-2.0); `boot_app0.bin` is the stock file from that core.

The USB identity (VID/PID, manufacturer and product strings, HID report
descriptor) matches the original device so that simulators and existing
bindings keep working. It was read from the device over USB for
interoperability. No code or data from the vendor's firmware is included in
this repository or in the release files.

## Windows updater (`desktop/`)

| Component | Version | License |
|---|---|---|
| [.NET](https://github.com/dotnet/runtime) | 10 | MIT (+ [third-party notices](https://github.com/dotnet/runtime/blob/main/THIRD-PARTY-NOTICES.TXT)) |
| [Avalonia](https://github.com/AvaloniaUI/Avalonia/blob/12.1.3/licence.md) | 12.1.3 | MIT |
| [SkiaSharp](https://github.com/mono/SkiaSharp/blob/main/LICENSE.md) / Skia / HarfBuzz | bundled | MIT / BSD-3-Clause / MIT |
| [esptool](https://github.com/espressif/esptool) | 5.4.0 | GPL-2.0-or-later |
| Python runtime (via PyInstaller) | 3.13 | PSF-2.0 |
| [PyInstaller](https://github.com/pyinstaller/pyinstaller) bootloader | 6.20.0 | GPL-2.0 with bootloader exception |

The flashing worker (`desktop/worker/worker.py`) imports esptool, so the bundled
worker is distributed under GPL-2.0-or-later (`desktop/COPYING-worker.txt`).
The updater archive includes the worker source and the exact esptool source
archive.
