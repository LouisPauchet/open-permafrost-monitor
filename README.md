# Open Permafrost Monitor — RTK GNSS field logger

ESP32 + u-blox ZED-F9P (gnss.store ELT0128) RTK rover. Wakes on a configurable interval, connects to WiFi, pulls RTCM corrections from an NTRIP caster, waits for a fix meeting a configurable accuracy threshold, publishes the position over MQTTS, then deep-sleeps until the next cycle.

## Hardware

- ESP32 DevKit V1
- gnss.store ELT0128 (u-blox ZED-F9P breakout)
- U.FL/IPEX GNSS antenna with clear sky view

## Wiring (UART1, SEL jumper left unsoldered)

| ZED-F9P    | ESP32       |
|------------|-------------|
| UART1 TXO  | GPIO16 (RX2)|
| UART1 RXI  | GPIO17 (TX2)|
| VCC (3.3V) | 3.3V        |
| GND        | GND         |

Both boards are 3.3V logic, so no level shifter is needed. Do not power the F9P from 5V. All pin assignments live in one place: [include/pins.h](include/pins.h).

## Code layout

| File | Responsibility |
|------|-----------------|
| [include/pins.h](include/pins.h) | All GPIO/UART pin assignments |
| [include/device_config.h](include/device_config.h) / [src/device_config.cpp](src/device_config.cpp) | Field-configurable settings, persisted in NVS flash |
| [include/config_portal.h](include/config_portal.h) / [src/config_portal.cpp](src/config_portal.cpp) | WiFi AP + captive web portal for field setup |
| [include/gnss.h](include/gnss.h) / [src/gnss.cpp](src/gnss.cpp) | ZED-F9P UART, NMEA parsing, fix quality/accuracy |
| [include/ntrip_client.h](include/ntrip_client.h) / [src/ntrip_client.cpp](src/ntrip_client.cpp) | NTRIP caster connection, RTCM forwarding |
| [include/mqtt_client.h](include/mqtt_client.h) / [src/mqtt_client.cpp](src/mqtt_client.cpp) | MQTTS publish |
| [include/isrg_root_x1.h](include/isrg_root_x1.h) | Let's Encrypt root CA, for validating the MQTT broker's TLS cert |
| [include/sd_logger.h](include/sd_logger.h) / [src/sd_logger.cpp](src/sd_logger.cpp) | Raw GNSS logging to SD for later PPK/PPP post-processing (pending SD reader hardware) |
| [src/main.cpp](src/main.cpp) | Orchestrates one wake→measure→publish→sleep cycle |
| [include/secrets.h](include/secrets.h) (git-ignored, copy from `secrets.h.example`) | Compiled-in **seed defaults** loaded into NVS on first boot only |

## Field setup (no reflashing needed)

On every **power-on or reset** (not on a timer wake from deep sleep), the device opens a WiFi hotspot for 5 minutes, named **"PermafrostMonitor-XXXXXX"** where `XXXXXX` is the last 3 bytes of that unit's ESP32 MAC address — so each physical device in the field has a distinguishable network name when several are deployed at once.

1. Connect your phone/laptop to that WiFi network.
2. Browse to `192.168.4.1`.
3. Set the WiFi network the device should use in the field, plus:
   - MQTT host/port/client ID/username(token)/password/topic
   - NTRIP caster host/port/mountpoint/username/password
   - **Measurement interval** — how often to wake, measure, and publish (seconds)
   - **Max wait for a good fix** — give up and publish best-effort after this long (seconds)
   - **Good fix horizontal accuracy** — threshold in meters (e.g. `0.05` for 5 cm) to consider a fix "good enough"
4. Submit. Settings are saved to flash (NVS) and used from then on, surviving reboots and reflashes.

If nobody connects within 5 minutes, the device proceeds with whatever was last saved (or the compiled-in defaults from `secrets.h` on a brand-new device).

## Low-power measurement cycle

Each wake:

1. Connect WiFi (using credentials saved via the portal).
2. Connect to the NTRIP caster and start forwarding RTCM3 corrections into the F9P.
3. Poll the GNSS until horizontal accuracy (from the receiver's own GST error estimate) is at or below the configured threshold, or until the configured timeout elapses — whichever comes first.
4. Connect to the MQTT broker over TLS (cert validated against Let's Encrypt's root) and publish the position.
5. Disconnect everything and deep-sleep for the configured measurement interval. On wake, execution restarts from the top (no state is kept in RAM) — the portal is skipped because the wake reason is a timer, not a power-on/reset.

## MQTT payload

Published as JSON to the configured topic. Position fields follow CF (Climate and Forecast) convention naming:

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

`fix_quality` follows the standard NMEA GGA fix indicator (0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float). `quality_ok` reflects whether `horizontal_accuracy` met the configured threshold before the wait timed out.

## Post-processing (PPK/PPP) — SD card logging

Not yet wired up (SD reader hardware pending), but scaffolded so it activates without further code changes once it arrives:

1. Uncomment and set `SD_CS_PIN` in [include/pins.h](include/pins.h) to match your reader's wiring (standard SPI: MOSI/MISO/SCK are the ESP32's default VSPI pins 23/19/18, only CS varies by board).
2. On the next boot, `initSDLogger()` will detect the card, open a new log file (`/gnss_NNNNNN.ubx`, an incrementing counter stored in RTC memory), and `gnss.cpp` will both enable `RXM-RAWX`/`RXM-SFRBX` raw measurement output on the F9P and write every raw byte from its UART (NMEA + UBX) straight to that file.
3. Those `.ubx` log files can be fed into RTKLIB (`convbin`) or u-center to convert to RINEX and post-process against a base station's raw data (or precise products) for accuracy beyond what a live RTK float/fixed solution reached — useful for cycles where `quality_ok` came back `false`.

If `SD_CS_PIN` is left undefined (the current state), all of this is a complete no-op — no behavior change, no wasted cycles.

## Build & flash

```
pio run -t upload
pio device monitor
```

(or open this folder in the PlatformIO VS Code extension and use the Upload / Monitor buttons)

## Notes

- `WiFi.setSleep(false)` is set on every connect to avoid ESP32's modem-sleep power-save mode, which otherwise drops the WiFi association roughly every 30-60s.
- The Arduino loop-task stack is raised to 16KB (`SET_LOOP_TASK_STACK_SIZE`) because the default 8KB is too small for a TLS handshake and crashes with a stack overflow.
- `secrets.h` is git-ignored; only `secrets.h.example` (with placeholder values) is committed.
