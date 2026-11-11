# Changelog

All notable changes to **proto_esp32_s3** are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

- The `config` command accepts `rate_hz` values 200 and 1500.
- The same command accepts `range_ga` values 2, 8, 12, and 30.
- The config reply still reports `odr_hz` and `gain_ga`.
- Card lines sit in an internal DRAM buffer so the SDSPI DMA channel can move them.
- `tool/wifi_smoke.py` checks `range_ga` and both sample rates.

### Changed

- Above 200 Hz the TCP task sends about 50 raw frames each second.
- The TCP task still sends every FFT frame.

## [2026-11-10]

### Added

- The logger reads the QMC5883P and writes samples to the microSD card.
- The board opens a SoftAP named ProtoMag and a TCP server on port 3333.
- Each TCP frame is one CBOR map with a big-endian length prefix.

## [2026-10-28]

### Changed

- The pin map moved to the magnetometer and microSD layout.
- The NimBLE example stays in the tree.

## [2026-08-26]

### Added

- The thermal camera image path reached its first version.
- Magnetometer plots support time and frequency views.

## [2026-05-09]

### Added

- Initial ESP32-S3 breadboard firmware.
