# PlatformIO commands (ESP-IDF)

Run these from the project root that contains `platformio.ini`. Aimed at projects using `framework = espidf`.

## Commands

| Command | Description |
|---------|-------------|
| `pio run` | Build the firmware for the active environment. |
| `pio run -t upload` | Build (if needed) and upload firmware to the device. |
| `pio run -t clean` | Remove build artifacts for the active environment. |
| `pio run -t uploadfs` | Build and upload the filesystem image (SPIFFS/LittleFS). |
| `pio run -t erase` | Erase the entire flash on the connected device. |
| `pio device monitor` | Open the serial monitor. |
| `pio run -t menuconfig` | Open the ESP-IDF Kconfig menu (`menuconfig`). |

### Examples

```bash
pio run
pio run -t upload
pio run -t clean
pio run -t uploadfs
pio run -t erase
pio device monitor
pio run -t menuconfig
```

## Common flags

| Flag | Description |
|------|-------------|
| `-e <env>` / `--environment <env>` | Select a `platformio.ini` environment (for example a board profile). |
| `-t <target>` / `--target <target>` | Run a named target (`upload`, `clean`, `uploadfs`, `erase`, `menuconfig`, …). |
| `--upload-port <port>` | Serial/USB port used for flash upload (for example `/dev/ttyUSB0` or `COM3`). |
| `-p <port>` / `--port <port>` | Serial port for `pio device monitor` and related device commands. |
| `-b <baud>` / `--baud <baud>` | Monitor baud rate (overrides `monitor_speed` in `platformio.ini` when passed). |
| `-d <path>` / `--project-dir <path>` | Project root if you are not running from the directory that contains `platformio.ini`. |
| `-v` / `--verbose` | Verbose build/upload output (useful when diagnosing ESP-IDF or toolchain issues). |

### Examples with flags

```bash
pio run -e esp32dev
pio run -e esp32dev -t upload --upload-port /dev/ttyUSB0
pio device monitor -p /dev/ttyUSB0 -b 115200
pio run -d /path/to/project -t menuconfig -v
```
