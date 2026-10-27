# Build and test

## Environments

| Env | Target | Builds |
|---|---|---|
| `walter` | ESP32-S3 (as `esp32-s3-devkitc-1`, 16 MB flash, QIO/OPI PSRAM) | The firmware |
| `native` | Your PC | Only the hardware-free logic: `geophone_parse.cpp`, `miniseed_record.cpp`, plus header-only `geophone_timing.h` and `ubx.h` |

Always pass `-e` explicitly. Plain `pio run` builds both.

```sh
pio run -e walter                      # compile firmware
pio run -e walter -t upload            # flash
pio device monitor -e walter           # console with exception decoder
pio test -e native                     # host unit tests
```

### Pinned versions (don't bump casually)

- `platform = espressif32 @ 6.13.0` (Arduino core 2.0.17, ESP-IDF 4.4).
  The unpinned platform resolves to 8.x, which breaks the vendored
  WalterModem 1.5.0 (`SPI_FLASH_SEC_SIZE` errors). The firmware also uses
  the IDF 4.4 `esp_task_wdt_init(timeout, panic)` signature.
- `TinyGPSPlus 1.1.0`, `ArduinoJson 7.4.3`.

### Build flags that matter

| Flag | Why |
|---|---|
| `-std=gnu++17` | Required by WalterModem |
| `ARDUINO_USB_MODE=0`, `ARDUINO_USB_CDC_ON_BOOT=1` | TinyUSB composite CDC + MSC, so the SD card can be a USB drive on the console port |
| `CORE_DEBUG_LEVEL=2` | Compiles out debug logs. At level 4, WalterModem hex-dumps every RTCM chunk, which saturates USB-CDC and trips the watchdog. Raise it only for short debugging sessions. |
| `ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE=8192` | WalterModem's UART receive callback overflowed the 2 KB default |
| `SET_LOOP_TASK_STACK_SIZE(16 KB)` (in `main.cpp`) | TLS handshakes overflow the default 8 KB loop stack |

## Unit tests (`native`)

`pio test -e native` runs the Unity tests in `test/test_native/`:

| Area | Tests |
|---|---|
| Digitizer packet parser | channel packets, compact/lower-case keys, 50-sample packets, markers, malformed input, too many samples |
| Sample timing | jitter tracking without stepping, stepping on big jumps, MSEC wraparound, channel continuity and resync |
| UBX | checksum against known frames, CFG-MSG builder |
| MiniSEED | Steim2 round-trips (quiet, mixed, extreme values), int32 record header, ObsPy fixture |

The last test writes `steim2_fixture.mseed` and `steim2_fixture.txt` to
the project root (git-ignored). Cross-check them with an independent
decoder:

```sh
pip install obspy
python test/obspy_check.py
# ObsPy OK: XX.PERM1..EH1 ... 200.0 Hz, lossless
```

The native environment needs a host C++ compiler: `g++` on Linux, Xcode
command-line tools on macOS, MinGW or WSL on Windows.

## Container build (no local toolchain)

```sh
podman run --rm -v "$PWD:/src:ro" -v pio-cache:/root/.platformio python:3.11-slim \
  bash -c 'apt-get update -qq && apt-get install -y -qq g++ >/dev/null &&
           pip install -q platformio &&
           mkdir /work && tar -C /src --exclude=.pio --exclude=cao -cf - . | tar -C /work -xf - &&
           cd /work && cp include/secrets.h.example include/secrets.h &&
           pio run -e walter && pio test -e native'
```

`docker` works the same way. The named volume caches the PlatformIO
toolchain between runs.

## Testing on hardware

There is no hardware-in-the-loop test suite. To validate a change:

1. Flash, then run the [first boot](../build/first-boot.md) bench checklist.
2. Watch at least two full bursts with NTRIP. Check that `SDdrop=0B`
   throughout, and that no watchdog reset or `Guru Meditation` occurs.
3. Enter and leave service mode, and press eject and reinsert the card,
   both during a burst. Both should react immediately.
4. For timing changes, load a day of MiniSEED in ObsPy. Check that
   `st.merge()` gives one trace per channel and inspect `st.get_gaps()`.
