# Open Permafrost Monitor — RTK GNSS + seismic field station

Walter ESP32-S3 (Sequans GM02SP LTE-M/NB-IoT modem) driving a u-blox ZED-F9P
RTK GNSS receiver and a Raspberry Shake RS3D geophone, for unattended field
deployment with no WiFi/router dependency. Runs continuously (no deep sleep,
since the geophone streams 24/7): periodic RTK fixes via NTRIP, batched
MQTT telemetry, and a daily push of the day's raw files to S3-compatible
storage over the cellular link.

## Hardware

- Walter ESP32-S3 board (Sequans GM02SP LTE-M/NB-IoT modem, no WiFi used)
- u-blox ZED-F9P RTK GNSS (gnss.store ELT0128), UART1 on the F9P, SEL jumper
  unsoldered, powered from the board's 5V rail
- Raspberry Shake RS3D geophone digitizer (protocol reverse-engineered in
  [raspberry_shake.md](raspberry_shake.md)), UART, receive-only, fixed
  230400 baud
- microSD card on the native SDMMC peripheral (1-bit mode, not SPI) —
  **D3 needs a pull-up resistor even in 1-bit mode**, or the card falls back
  to SPI mode and mounting fails
- A 4-wire field-service cable (ground + the console/SD-access trigger pin)

All pin assignments live in one place: [include/pins.h](include/pins.h).

| Subsystem       | Pins                                             |
|-----------------|---------------------------------------------------|
| GNSS (UART2)    | RX 16, TX 17, EXTINT 18                            |
| Geophone (UART1)| RX 15 (receive-only)                               |
| SD (SDMMC)      | CLK 12, CMD 11, D0 13, D3 38, card-detect 39      |
| SD power        | 3V3_EN 0 (active LOW, held enabled continuously)   |
| Field service   | select 40 (grounded by the service cable)          |
| Battery ADC     | external 2, internal 1                             |

## Code layout

| File | Responsibility |
|------|-----------------|
| [include/pins.h](include/pins.h) | All GPIO/UART pin assignments |
| [include/device_config.h](include/device_config.h) / [src/device_config.cpp](src/device_config.cpp) | Field-configurable settings, persisted in NVS flash |
| [include/field_service.h](include/field_service.h) / [src/field_service.cpp](src/field_service.cpp) | Text config menu + SD file download over native USB CDC |
| [include/modem.h](include/modem.h) / [src/modem.cpp](src/modem.cpp) | Thin wrapper around WalterModem: registration, async TCP socket (NTRIP), MQTT, HTTP |
| [include/gnss.h](include/gnss.h) / [src/gnss.cpp](src/gnss.cpp) | ZED-F9P UART, NMEA parsing, fix quality/accuracy, EXTINT sleep/wake |
| [include/ntrip_client.h](include/ntrip_client.h) / [src/ntrip_client.cpp](src/ntrip_client.cpp) | NTRIP caster connection (with retry), RTCM forwarding, GGA relay |
| [include/geophone.h](include/geophone.h) / [src/geophone.cpp](src/geophone.cpp) | RS3D UART reader, continuous JSON sample stream parser |
| [include/miniseed_writer.h](include/miniseed_writer.h) / [src/miniseed_writer.cpp](src/miniseed_writer.cpp) | Minimal MiniSEED writer, GNSS-timestamped |
| [include/sd_storage.h](include/sd_storage.h) / [src/sd_storage.cpp](src/sd_storage.cpp) | SD_MMC mount, daily-rotated file helpers shared by every logger |
| [include/position_log.h](include/position_log.h) / [src/position_log.cpp](src/position_log.cpp) | Daily position CSV — also the MQTT batch buffer |
| [include/battery.h](include/battery.h) / [src/battery.cpp](src/battery.cpp) | Internal/external battery voltage via ADC |
| [include/mqtt_client.h](include/mqtt_client.h) / [src/mqtt_client.cpp](src/mqtt_client.cpp) | Batched MQTTS telemetry publish |
| [include/upload_client.h](include/upload_client.h) / [src/upload_client.cpp](src/upload_client.cpp) | Daily chunked HTTP POST of the day's files to a user-run backend |
| [include/isrg_root_x1.h](include/isrg_root_x1.h) | Let's Encrypt root CA for TLS validation (MQTT + upload) |
| [src/main.cpp](src/main.cpp) | Persistent scheduler tying everything together |
| [include/secrets.h](include/secrets.h) (git-ignored, copy from `secrets.h.example`) | Compiled-in **seed defaults** loaded into NVS on first boot only |
| [lib/WalterModem/](lib/WalterModem/) | Vendored copy of `quickspot/WalterModem` 1.5.0 with two crash-fix patches — see [lib/WalterModem/README-PATCH.md](lib/WalterModem/README-PATCH.md) |

## Why no deep sleep

The geophone streams continuously at 230400 baud with no way to pause it, so
the CPU can never sleep. `loop()` runs forever, servicing the geophone and
GNSS UARTs every single iteration, with everything else (GNSS fix cycles,
telemetry batches, daily uploads) scheduled off its own `millis()` timer.
Low power now comes from keeping the modem and GNSS idle as much as
possible while the CPU stays up for the geophone, not from CPU sleep.

## Field service mode

Grounding the field-service select pin (cable's 4th wire) — either at boot,
or at any point during normal operation — reboots the device straight into
a text menu over native USB serial (115200 8N1). No physical reset needed;
unplugging the cable (or `0) Save and exit`) reboots back into normal
operation. From there a technician can:

- Adjust the GNSS measurement interval, fix timeout, and good-accuracy
  threshold
- Set MQTT / NTRIP / upload-endpoint credentials (stored in NVS, survives
  reflashing)
- List and download files off the SD card, streamed over the same serial
  connection (`BEGIN <name> <size>` … raw bytes … `END`) — true USB mass
  storage wasn't possible without forking the SD_MMC library, so this is
  the alternative

Kept deliberately minimal (per the "as few parameters as possible"
preference) — most tuning happens through `secrets.h` at flash time.

## GNSS fix cycle

On the configured `measurement_interval_s`:

1. Pulse the F9P's EXTINT pin to wake it from `UBX-RXM-PMREQ` backup mode.
2. Connect to the NTRIP caster (retries on failure — 5s for a 401, which
   this caster's history shows is often transient, 10s otherwise) and start
   forwarding RTCM3 corrections into the F9P.
3. Relay the rover's own GGA position back to the caster periodically —
   required for network RTK/VRS casters (like this deployment's) to center
   corrections on the rover's actual location, not just nice-to-have.
4. Poll until horizontal accuracy (from the receiver's own GST error
   estimate) meets the configured threshold, or the fix timeout elapses.
5. Append a row to the day's position CSV (time, lat/lon/alt, accuracy,
   fix quality, battery voltages).
6. Disconnect NTRIP, send the F9P back into backup mode.

GNSS sleep/wake uses the same EXTINT + `UBX-RXM-PMREQ` mechanism as before;
UART RX activity is also configured as a wake source as a bring-up safety
net in case EXTINT wiring/polarity ever needs adjustment.

## Telemetry and daily upload

- **MQTT**: batched, not per-fix — connects once per hour (not
  user-configurable, to keep the modem idle more) and publishes every
  position_log row appended since the last batch as one JSON array message.
- **Daily upload**: on GNSS-date rollover, POSTs the previous day's four
  rotated files (geophone MiniSEED, GNSS raw log, NTRIP raw log, position
  CSV) in 32KB chunks to a backend the user runs themselves, authenticated
  with a single API key (HTTP Basic auth). That backend — not part of this
  firmware — is responsible for forwarding to actual S3-compatible storage;
  the device never holds cloud-storage credentials, only one rotatable key.

## MQTT payload

```json
{
  "time": "2026-07-14T12:34:56Z",
  "latitude": 78.2232012,
  "longitude": 15.6512034,
  "altitude": 12.34,
  "fix_quality": 4,
  "satellites": 22,
  "horizontal_accuracy": 0.031,
  "vertical_accuracy": 0.048,
  "quality_ok": true
}
```

Sent as a JSON array of rows per batch. `fix_quality` follows the standard
NMEA GGA fix indicator (0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed,
5=RTK float). `quality_ok` reflects whether `horizontal_accuracy` met the
configured threshold before the wait timed out.

## Build & flash

```
pio run -e walter -t upload
pio device monitor -e walter
```

(or open this folder in the PlatformIO VS Code extension and use the
Upload / Monitor buttons)

## Notes / known issues

- The Arduino loop-task stack is raised to 16KB (`SET_LOOP_TASK_STACK_SIZE`)
  and arduino-esp32's UART event task stack to 8KB
  (`ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE`) — both default sizes are too
  small for WalterModem's TLS/AT-parsing workload and crash with a stack
  overflow otherwise.
- A task watchdog (60s) reboots the device if the main loop ever genuinely
  stalls — a last-resort safety net for unattended operation, not expected
  to fire in normal use.
- WalterModem 1.5.0 has a real crash bug (unguarded null dereference in its
  response parser, hit reproducibly under bursty NTRIP traffic) — vendored
  locally with a fix rather than pulled from the registry; see
  [lib/WalterModem/README-PATCH.md](lib/WalterModem/README-PATCH.md).
- SD card mount currently fails (`0x107`) on the physical carrier board —
  suspected missing/insufficient pull-up on the D3 line (see the comment in
  `pins.h`); needs hardware inspection, not a firmware fix.
- `secrets.h` is git-ignored; only `secrets.h.example` (with placeholder
  values) is committed.
