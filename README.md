# Open Permafrost Monitor — GNSS bring-up

Step 1: confirm the ESP32 can read NMEA data from the ZED-F9P (gnss.store ELT0128) and get a basic fix, before adding RTK corrections.

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

Both boards are 3.3V logic, so no level shifter is needed. Do not power the F9P from 5V.

## Build & flash

```
pio run -t upload
pio device monitor
```

(or open this folder in the PlatformIO VS Code extension and use the Upload / Monitor buttons)

## What to expect

- Raw NMEA sentences ($GNGGA, $GNRMC, etc.) printed as they arrive.
- A "Fix summary" block once per second: fix validity, satellite count, HDOP, lat/lng, altitude.
- Outdoors with sky view, a basic (non-RTK) fix typically appears within 30-60s cold start.

If nothing appears in the monitor after ~15s, the firmware prints a warning — try changing `GPS_BAUD` in [src/main.cpp](src/main.cpp) to 9600 or 115200 (some F9P boards arrive pre-configured to a different rate than the factory default 38400).

## Next step

Once a basic (standalone) fix is confirmed, the next step is adding an NTRIP client to push RTCM correction data into the F9P's UART for RTK fix mode.
