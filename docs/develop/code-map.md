# Code map

Paths are relative to the repository root. Most modules are a header in
`include/` paired with an implementation in `src/`.

## Core

| Module | Responsibility |
|---|---|
| `src/main.cpp` | `setup()` boot sequence, `loop()` scheduler, the GNSS fix cycle, telemetry batch and daily-upload triggers, battery backoff, watchdog. See [GNSS fix cycle](../how-it-works/scheduler.md). |
| `include/pins.h` | **All** GPIO and UART assignments and baud rates |
| `device_config` | Field settings: struct, compiled-in defaults, NVS load/save |
| `include/secrets.h.example` | Template for `secrets.h` (git-ignored), the first-boot connection seeds |
| `field_service` | Service-pin trigger, text menu over USB CDC |

## Sensors

| Module | Responsibility |
|---|---|
| `gnss` | ZED-F9P: UART polling, NMEA parsing (TinyGPS++), GST accuracy, UBX config, raw UBX/RTCM capture to SD, EXTINT wake, PMREQ sleep, GNSS-anchored UTC clock (`gnssEpochMillisNow()`) |
| `include/ubx.h` | UBX checksum and frame builder (unit-tested) |
| `include/utc_civil.h` | Epoch ↔ civil date conversions |
| `geophone` | RS3D UART task: drains, parses, feeds MiniSEED |
| `geophone_parse` | Pure JSON packet parser for the digitizer stream (unit-tested) |
| `include/geophone_timing.h` | `GeoClock`: digitizer MSEC to UTC mapping, per-channel continuity (header-only, unit-tested). See [Timing](../how-it-works/timing.md). |
| `battery` | External and internal battery voltage via ADC (divider ratios here) |
| `reference_position` | Commissioning average, persisted reference, N/E/U displacement |

## Storage and data

| Module | Responsibility |
|---|---|
| `sd_storage` | SPI mount, card-detect and eject, async stream writer task, free space, USB mass storage. See [Storage](../how-it-works/storage.md). |
| `miniseed_writer` | Per-channel buffering, daily file rotation, low-space pause |
| `miniseed_record` | Pure SEED 2.4 record builder, Steim2 and int32 encoders (unit-tested) |
| `position_log` | Daily positions CSV; reads unsent rows into the MQTT JSON batch |

## Communications

| Module | Responsibility |
|---|---|
| `modem` | WalterModem wrapper: init, registration, sleep, async TCP socket, worker task and job queue, MQTT batch, HTTP POST, TLS/CA setup |
| `ntrip_client` | Caster connection with retries, RTCM forwarding to F9P, GGA relay |
| `mqtt_client` | Thin layer over the modem's async batch publish |
| `upload_client` | Chunked HTTPS upload job of a daily file |
| `include/isrg_root_x1.h` | Let's Encrypt root CA (MQTT + upload TLS) |
| `include/spi_flash_mmap.h` | Shim for an ESP-IDF 5 header that WalterModem expects but IDF 4.4 lacks |
| `lib/WalterModem/` | Vendored, patched modem library; see [WalterModem library](walter-modem.md) |

## Tests and other files

| Path | Contents |
|---|---|
| `test/test_native/test_main.cpp`, `test_mseed.inc` | Unity tests run on the PC; see [Build and test](building-and-testing.md) |
| `test/obspy_check.py` | Cross-checks the Steim2 encoder output with ObsPy |
| `platformio.ini` | `walter` (device) and `native` (tests) environments |
| `cao/` | CAD and 3D-print files; see [Enclosure parts](../build/enclosure.md) |
| `docs/`, `mkdocs.yml` | This documentation site |

## Where to change common things

| Change | File |
|---|---|
| Move a signal to another pin | `include/pins.h` |
| Add a menu setting | `include/device_config.h` and `src/device_config.cpp` (struct, default, NVS key), `src/field_service.cpp` (menu, `showConfig()`), then the [menu reference](../field/configuration.md) |
| Add a telemetry field | `src/position_log.cpp` (CSV header, row format, JSON batch), then [Positions CSV](../data/positions-csv.md) and [MQTT](../data/mqtt.md) |
| Calibrate the battery divider | `src/battery.cpp` |
| Burst timing constants | top of `src/main.cpp` |
