# Open Permafrost Monitor — RTK GNSS + seismic field station

Walter ESP32-S3 (Sequans GM02SP LTE-M/NB-IoT modem) driving a u-blox ZED-F9P
RTK GNSS receiver and a Raspberry Shake RS3D geophone, for unattended field
deployment with no WiFi/router dependency. Runs continuously (no deep sleep,
since the geophone streams 24/7): periodic RTK fixes via NTRIP, batched
MQTT telemetry, and a daily push of the day's raw files to S3-compatible
storage over the cellular link.

**Building/wiring a station for the first time?** Start with
[GETTING_STARTED.md](GETTING_STARTED.md) — a step-by-step wiring, flashing,
and first-boot walkthrough. This document covers the *why* behind the
system's behavior; that one covers the *what do I connect and type*.

## Hardware

- Walter ESP32-S3 board (Sequans GM02SP LTE-M/NB-IoT modem, no WiFi used)
- u-blox ZED-F9P RTK GNSS (gnss.store ELT0128), UART1 on the F9P, SEL jumper
  unsoldered, powered from the board's 5V rail
- Raspberry Shake RS3D geophone digitizer (protocol reverse-engineered in
  [raspberry_shake.md](raspberry_shake.md)), UART, receive-only, fixed
  230400 baud
- microSD card over SPI (not native SDMMC — SDMMC's tighter signal-integrity
  requirements kept hitting command timeouts on this carrier board's
  provisional wiring; SPI's looser timing tolerates it reliably)
- A momentary eject button, and a 4-wire field-service cable (ground + the
  console/SD-access trigger pin)

All pin assignments live in one place: [include/pins.h](include/pins.h).

| Subsystem       | Pins                                             |
|-----------------|---------------------------------------------------|
| GNSS (UART2)    | RX 16, TX 17, EXTINT 18                            |
| Geophone (UART1)| RX 15 (receive-only)                               |
| SD (SPI)        | SCLK 12, MOSI 11, MISO 13, CS 38, card-detect 39   |
| SD eject button | 41 (momentary, to GND)                             |
| SD power        | 3V3_EN 0 (active LOW, held enabled continuously)   |
| Field service   | select 40 (grounded by the service cable)          |
| Battery ADC     | external 2, internal 1                             |

## Code layout

| File | Responsibility |
|------|-----------------|
| [include/pins.h](include/pins.h) | All GPIO/UART pin assignments |
| [include/device_config.h](include/device_config.h) / [src/device_config.cpp](src/device_config.cpp) | Field-configurable settings, persisted in NVS flash |
| [include/field_service.h](include/field_service.h) / [src/field_service.cpp](src/field_service.cpp) | Text config menu over USB CDC, composite with the SD-as-USB-drive mode (sd_storage.h) on the same port |
| [include/modem.h](include/modem.h) / [src/modem.cpp](src/modem.cpp) | Thin wrapper around WalterModem: registration, async TCP socket (NTRIP), MQTT, HTTP |
| [include/gnss.h](include/gnss.h) / [src/gnss.cpp](src/gnss.cpp) | ZED-F9P UART, NMEA parsing, fix quality/accuracy, EXTINT sleep/wake |
| [include/ntrip_client.h](include/ntrip_client.h) / [src/ntrip_client.cpp](src/ntrip_client.cpp) | NTRIP caster connection (with retry), RTCM forwarding, GGA relay |
| [include/geophone.h](include/geophone.h) / [src/geophone.cpp](src/geophone.cpp) | RS3D UART reader, continuous JSON sample stream parser |
| [include/miniseed_writer.h](include/miniseed_writer.h) / [src/miniseed_writer.cpp](src/miniseed_writer.cpp) | Minimal MiniSEED writer, sample-clock timed and mapped to GNSS UTC |
| [include/sd_storage.h](include/sd_storage.h) / [src/sd_storage.cpp](src/sd_storage.cpp) | SD-over-SPI mount, daily-rotated file helpers shared by every logger, eject button, USB mass-storage exposure for field service mode |
| [include/position_log.h](include/position_log.h) / [src/position_log.cpp](src/position_log.cpp) | Daily position CSV — also the MQTT batch buffer |
| [include/battery.h](include/battery.h) / [src/battery.cpp](src/battery.cpp) | Internal/external battery voltage via ADC |
| [include/mqtt_client.h](include/mqtt_client.h) / [src/mqtt_client.cpp](src/mqtt_client.cpp) | Batched MQTTS telemetry publish |
| [include/upload_client.h](include/upload_client.h) / [src/upload_client.cpp](src/upload_client.cpp) | Daily chunked HTTP POST of the day's files to a user-run backend |
| [include/isrg_root_x1.h](include/isrg_root_x1.h) | Let's Encrypt root CA for TLS validation (MQTT + upload) |
| [src/main.cpp](src/main.cpp) | Persistent scheduler tying everything together |
| [include/secrets.h](include/secrets.h) (git-ignored, copy from `secrets.h.example`) | Compiled-in **seed defaults** loaded into NVS on first boot only |
| [lib/WalterModem/](lib/WalterModem/) | Vendored copy of `quickspot/WalterModem` 1.5.0 with two crash-fix patches — see [lib/WalterModem/README-PATCH.md](lib/WalterModem/README-PATCH.md) |

## Power management

The geophone streams continuously at 230400 baud with no way to pause it, so
the CPU can never deep-sleep. The geophone runs on its own FreeRTOS task
(UART drain, packet parsing, MiniSEED encoding), so nothing blocking on
`loop()` can make it lose samples. `loop()` runs forever, servicing the GNSS
UART, with everything else (GNSS fix cycles, telemetry batches, daily
uploads) scheduled off its own `millis()` timer. Instead, the two genuinely power-hungry peripherals are put to sleep
between measurement cycles:

- **GNSS**: the F9P goes into `UBX-RXM-PMREQ` backup mode (`gnssSleep()`)
  right after each fix cycle, woken by a pulse on its EXTINT pin
  (`gnssWakeup()`) at the start of the next one. The wake-source mask is
  EXTINT-only — an earlier revision also armed UART RX as a wake source as
  a bring-up safety net, but that meant any electrical noise on the shared
  UART line (not just a real wake pulse) kept the module from actually
  sleeping, confirmed on live hardware.
- **Modem**: dropped to `NO_RF` opstate (`modemSleep()`) once a cycle's
  NTRIP/MQTT work is done, brought back to `FULL` at the start of the next
  cycle.
- **Low-battery backoff**: below `low_battery_cutoff_v` (external battery
  ADC, averaged over 16 reads), new GNSS/modem bursts are skipped entirely
  until the pack recovers to 0.2 V above the cutoff (hysteresis, so a pack
  sagging under burst load doesn't flip on and off) — geophone/SD logging
  (comparatively low draw) keeps running regardless.

Time itself has the same "frozen while asleep" problem as the RF
peripherals: GNSS's own UTC fields don't update while the F9P is asleep, so
anything that needs continuously-advancing time across a sleep window
extrapolates from the last fix via `millis()` instead of reading the GNSS
fields directly — see `gnssEpochMillisNow()`/`gnssCivilNow()` in
[include/gnss.h](include/gnss.h).

MiniSEED record times come from the digitizer's own sample clock: its
`MSEC` cycle marker is mapped onto that GNSS-extrapolated UTC, and each
channel advances by exactly one sample interval per sample, so records are
contiguous. The clock is only re-synced (and a line logged) on a real
discontinuity — see [include/geophone_timing.h](include/geophone_timing.h).
The constant latency between sampling and the marker reaching the ESP32 is
not calibrated out.

## Field service mode

Grounding the field-service select pin (cable's 4th wire) — either at boot,
or at any point during normal operation — reboots the device straight into
a text config menu over USB. No physical reset needed; unplugging the cable
(or `0) Save and exit`) reboots back into normal operation. From there a
technician can adjust the GNSS measurement interval, fix timeout, and
good-accuracy threshold; set MQTT/NTRIP/upload-endpoint credentials (stored
in NVS, survives reflashing); and start reference-position commissioning.

The same USB connection also exposes the SD card as an ordinary Windows/
Mac/Linux drive, at the same time as the console (composite CDC+MSC —
see `ARDUINO_USB_MODE=0` in `platformio.ini`), so a technician can just
drag files off the card directly instead of the old serial download
protocol. The drive only becomes usable once field service mode is
entered: normal operation still owns the card via FatFs, and having a host
do raw sector I/O at the same time would corrupt it, so
`sdStorageMscEnable()` closes every open FatFs stream and only then marks
the drive present (`sd_storage.cpp`). The MSC interface itself is always
present in the USB descriptor (interfaces can't be added after
enumeration), it just reports "no media" outside field service mode.

Independent of field service mode, the physical eject button
(`SD_EJECT_BUTTON_PIN`) flushes and closes every open stream, unmounts the
card, and cuts power to its rail — for physically swapping the card without
a hot-unplug mid-write. Press again (or just reinsert a card) to remount.

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
   fix quality, battery voltages, reference-position displacement if
   commissioned).
6. Publish any unsent position rows as one MQTT batch (piggybacking on the
   modem session already open for NTRIP - see Telemetry below).
7. Disconnect NTRIP, send the F9P back into backup mode, drop the modem to
   `NO_RF` - see Power management above.

## Telemetry and daily upload

- **MQTT**: batched, not per-fix, but published at the end of every fix
  cycle rather than on a separate timer — the modem's already up for NTRIP
  every cycle anyway (`measurement_interval_s`, usually ~10 min), so
  waiting longer wouldn't save a wake, only delay data reaching the
  backend. Every position_log row appended since the last successful batch
  goes out as one message; async (doesn't block the fix cycle), and rows
  that fail to send (modem down, batch already in flight) stay unsent and
  retry next cycle.
- **Daily upload**: once a UTC day is over, POSTs that day's positions CSV
  in 32KB chunks to a backend the user runs themselves, authenticated with
  a single API key (HTTP Basic auth). That backend — not part of this
  firmware — is responsible for forwarding to actual S3-compatible storage;
  the device never holds cloud-storage credentials, only one rotatable key.
  The upload runs at the end of a fix cycle on the modem session NTRIP
  already opened (so it's battery-gated like any burst and adds no wake of
  its own), on the modem worker task alongside the MQTT batch. The next day
  to upload is kept in NVS and only advanced after a successful upload, so
  failures retry next cycle and a reboot doesn't skip a day. Leaving
  `upload_host` empty disables it.
  MiniSEED (~230 MB/day uncompressed) and the per-burst GNSS raw files are
  **not** uploaded — far too much for LTE-M airtime and battery — and stay
  on the SD card for physical retrieval (field-service USB drive).

## MQTT payload

ThingsBoard-style `{"ts": ..., "values": {...}}` per row (a flat object
gets stamped with the server's ingestion time instead of the actual
reading time, which matters whenever a batch is delayed) - sent as a JSON
array of one such object per unsent position:

```json
[
  {
    "ts": 1786790096000,
    "values": {
      "latitude": 78.2232012,
      "longitude": 15.6512034,
      "altitude": 12.34,
      "horizontal_accuracy": 0.031,
      "vertical_accuracy": 0.048,
      "fix_quality": 4,
      "quality_ok": true,
      "battery_external_v": 12.61,
      "battery_internal_v": 4.11,
      "has_displacement": true,
      "north_m": 0.012,
      "east_m": -0.004,
      "up_m": -0.031,
      "horizontal_displacement_m": 0.0127
    }
  }
]
```

`fix_quality` follows the standard NMEA GGA fix indicator (0=no fix,
1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float). `quality_ok` reflects
whether `horizontal_accuracy` met the configured threshold before the wait
timed out. A burst without a fix still sends a row (battery voltages,
`fix_quality` 0, `quality_ok` false) so a blind station stays visible; such
rows omit `latitude`/`longitude`/`altitude`/accuracy entirely.
`has_displacement`/`north_m`/`east_m`/`up_m` are only meaningful
once a reference position has been commissioned (field service menu);
`horizontal_displacement_m` is derived (not stored), `sqrt(north² + east²)`.

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
- The USB console can miss the first few lines printed right after a fresh
  connection (e.g. reopening `pio device monitor` right after a flash) -
  `setup()` waits for `Serial` to report connected before printing
  anything, but that only means the host's DTR line is asserted, not that
  its terminal application has actually started reading bytes yet. Cosmetic
  only (nothing depends on those lines being seen); if it matters for a
  specific debugging session, reconnect once more or add a short fixed
  delay before the first print.
- `secrets.h` is git-ignored; only `secrets.h.example` (with placeholder
  values) is committed.
