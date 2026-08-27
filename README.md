# proto_esp32_s3

Breadboard prototype firmware (screen, buttons, IMU-ready) for ESP32-S3.

## Dependencies

| Package | How | Role |
|---------|-----|------|
| `ESP32-S3_Libraries` | `lib_deps` → `../ESP32-S3_Libraries` | `gpio_api`, `spi_api` |
| `esp32_s3_display` | `lib_deps` → `../ESP32-S3_Libraries/display` | `display_api` + layout |
| `st7789` | `lib/st7789` (vendored LibDriver) | Third-party ST7789 driver |

App entry is [`src/main.cpp`](src/main.cpp) (pins from [`docs/pinout_gpio.json`](docs/pinout_gpio.json)). ESP32-S3_Libraries provides drivers only.

## Build / upload

```bash
pio run
pio run -t upload
pio device monitor
```

Close any serial monitor before upload if the port is busy.
