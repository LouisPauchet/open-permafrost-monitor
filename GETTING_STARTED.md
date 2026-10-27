# Getting started

A step-by-step guide to wiring, flashing, and configuring this station from
scratch — written for someone building/setting up a unit for the first
time, not necessarily someone who's touched embedded firmware before. For
*why* things work the way they do, see [README.md](README.md) (system
behavior/architecture) and [raspberry_shake.md](raspberry_shake.md)
(geophone protocol deep-dive). This doc is just: what do I connect, what do
I type, what should I see.

## 1. What you'll need

**Hardware:**

- A [Walter](https://www.quickspot.io/) ESP32-S3 board (Sequans GM02SP
  LTE-M/NB-IoT modem on board)
- A u-blox ZED-F9P RTK GNSS receiver breakout (this project uses the
  gnss.store ELT0128) + its antenna
- A Raspberry Shake RS3D geophone digitizer board (the sensor+digitizer
  assembly, not a full Raspberry Shake unit — this project replaces the
  Raspberry Pi with the ESP32, see [raspberry_shake.md](raspberry_shake.md)
  §1 for why that's possible)
- A microSD card (any capacity that fits your retention needs — see
  README.md's storage-vs-decimation discussion for how much a day of data
  actually costs) + a breakout wired for **SPI** mode (CS/MOSI/MISO/SCK, not
  a 1-bit SDMMC-only module)
- Two momentary push-buttons (SD eject) and a way to ground one GPIO on
  demand (field-service cable) — a simple 2-pin header + jumper works fine
  for bench use
- A LTE-M/NB-IoT SIM card with a data plan (matching Walter's modem bands)
- A 5V/3.3V-logic power source your battery/solar setup can regulate down to
  (see README.md's "Power management" section for what draws current when)

**Software:**

- [PlatformIO](https://platformio.org/) — either the CLI or the VS Code
  extension (recommended if you're new to this: install the "PlatformIO
  IDE" extension in VS Code, then open this repo's folder — it picks up
  `platformio.ini` automatically)
- Git
- A serial terminal is built into PlatformIO (`pio device monitor` or the
  VS Code extension's monitor button) — you don't need a separate tool
- Optional but recommended for real deployment: an NTRIP caster account
  (for RTK corrections) and an MQTT broker (e.g.
  [ThingsBoard](https://thingsboard.io/)) for telemetry. The station still
  runs and logs everything locally without either — you just won't get
  live corrections or remote telemetry.

## 2. Wiring

All pin numbers below are ESP32-S3 GPIO numbers and match
[include/pins.h](include/pins.h) exactly — if you ever need to move a
signal to a different pin, that's the one file to change. Every connection
here is 3.3V logic; none of these modules are 5V-tolerant on their signal
pins even where they're powered from 5V.

### GNSS (ZED-F9P), UART2, 5V power rail

| Walter (ESP32-S3) | ZED-F9P breakout |
|---|---|
| GPIO 16 (RX) | UART1 TXO (F9P → ESP32) |
| GPIO 17 (TX) | UART1 RXI (ESP32 → F9P) |
| GPIO 18 | EXTINT |
| 5V | VCC |
| GND | GND |

**Before powering it on**: the ZED-F9P breakout's `SEL` jumper must be
**unsoldered/open** — this selects UART1 mode. If it's left in its default
(soldered/closed) state the module may come up in I2C mode instead and
nothing will show up on the UART at all. Baud rate is 38400 (set in
firmware, not something you configure on the module).

### Geophone (Raspberry Shake RS3D digitizer), UART1, receive-only

The digitizer board uses a Raspberry Pi 40-pin GPIO header layout even
though there's no Pi here — see
[raspberry_shake.md](raspberry_shake.md) §1 for the full pinout table.
The only two connections you actually need:

| Walter (ESP32-S3) | RS3D digitizer (Pi header pin) |
|---|---|
| GPIO 15 (RX) | Pin 10 (Pi RXD — carries the sensor data stream) |
| GND | Pin 6/9/14/20/25 (any ground pin) |
| 5V | Pin 2/4 (5V power) |

This is receive-only — the ESP32 never transmits to the digitizer, so
there's no wire to the digitizer's pin 8 (Pi TXD). Fixed baud rate, 230400,
not configurable (see raspberry_shake.md for why — it's baked into the
digitizer's own firmware).

### SD card, SPI mode

| Walter (ESP32-S3) | SD breakout |
|---|---|
| GPIO 12 | SCK |
| GPIO 11 | MOSI (DI on some breakouts) |
| GPIO 13 | MISO (DO on some breakouts) |
| GPIO 38 | CS |
| GPIO 39 | Card-detect switch (if your breakout has one — see below) |
| GPIO 0 | Switched 3.3V power enable — see caution below |

**Card-detect**: if your SD breakout doesn't expose a card-detect switch,
tie GPIO 39 to a fixed HIGH (the firmware treats HIGH as "card present" —
confirmed polarity on the reference hardware, see `sd_storage.cpp`) so the
firmware doesn't think the card is permanently missing.

**Power-enable caution**: GPIO 0 is one of the ESP32's boot-strapping
pins — externally pulling it low *during reset* forces the chip into its
USB download/bootloader mode instead of booting normally. The firmware
only drives it low in software after boot has already completed, which is
safe on the reference hardware, but if you're wiring your own power-switch
circuit (a MOSFET or load switch feeding the SD card's 3.3V rail), make
sure nothing in that circuit can pull GPIO 0 low during an external reset
or power-up before the firmware takes control of the pin.

**Eject button**: a momentary push-button between GPIO 41 and GND (uses
the internal pull-up — no external resistor needed).

### Field service cable

A momentary connection between GPIO 40 and GND — grounding it (a jumper,
switch, or a 4th wire on a service cable) reboots the station into the
config menu. No external resistor needed (internal pull-up).

### Battery monitoring

| Walter (ESP32-S3) | Signal |
|---|---|
| GPIO 2 | External battery voltage, via a resistor divider |
| GPIO 1 | Internal (Walter board) battery voltage, via a resistor divider |

The firmware assumes specific divider ratios (`EXTERNAL_DIVIDER_RATIO`/
`INTERNAL_DIVIDER_RATIO` in `src/battery.cpp`) — if your divider resistors
differ from the reference hardware's, measure your actual battery voltage
with a multimeter, compare it to what the field-service menu's `1) Show
current configuration` reports, and adjust those constants to match before
trusting the low-battery cutoff feature.

## 3. First build and flash

```sh
git clone <this repo's URL>
cd open_permafrost_monitor
cp include/secrets.h.example include/secrets.h
```

Edit `include/secrets.h` — fill in your NTRIP caster and MQTT broker
details (or leave the placeholders if you don't have either yet; the
station runs fine without them, see §1). These are only *seed defaults*
loaded into flash once on first boot — you can change all of them later
from the field-service menu without reflashing, so don't worry about
getting them perfect right now.

Then, with the board connected over USB:

```sh
pio run -t upload
pio device monitor
```

(Or in VS Code: the PlatformIO extension adds Upload/Monitor buttons to
the status bar at the bottom of the window.)

## 4. What to expect on first boot

Within the first few seconds you should see something like:

```
Walter <-> ZED-F9P + RS3D permafrost/seismic field station
SD: mounted, <size> MB
GNSS: listening for raw UART bytes (wiring/baud check)...
GNSS: <N> chars seen in 3s smoke test (0 = check wiring/baud/SEL jumper, not sleep/wake).
```

If the GNSS smoke test reports 0 chars, double-check the SEL jumper and
the RX/TX wiring (it's easy to swap TX/RX by mistake — the F9P's TXO goes
to the ESP32's RX, not its TX).

If you see `SD: mount failed.` or `SD: no card detected.`, check the card
is fully seated — a card that looks inserted but isn't fully seated
produces exactly this symptom and is easy to miss on visual inspection
alone (this bit us once on the reference hardware).

After the smoke test, the station starts its first GNSS fix cycle — this
can take up to two minutes if it's converging to an RTK fix via NTRIP
(longer if there's no caster configured yet, since it'll fall back to
autonomous/DGPS quality). You'll see periodic `[STATUS]` lines tracking fix
quality and accuracy as it converges.

## 5. Configuring the station

Ground the field-service pin (GPIO 40) to enter the config menu — either
hold it low while powering on, or just ground it any time during normal
operation (it reboots into the menu automatically, no need to power-cycle
by hand). You'll see:

```
=== Field service menu ===
1) Show current configuration
2) Set GNSS measurement interval (s)
3) Set GNSS fix timeout (s)
4) Set good-fix horizontal accuracy (m)
5) Set MQTT broker
6) Set NTRIP caster
7) Set upload endpoint
A) Set PPK minimum recording duration (s)
B) Set reference-position averaging window (s)
C) Start commissioning (establish reference position)
D) Set low-battery cutoff (V) - new bursts skipped below this
0) Save and exit
```

Type a single character and press enter to select an option. Numeric/text
options prompt you for a new value (press enter with nothing typed to keep
the current one, shown in brackets). `0) Save and exit` persists everything
to flash and reboots back into normal operation — unplugging the service
cable does the same thing without needing to explicitly select `0`.

While you're in here, the SD card also shows up as an ordinary USB drive on
your computer (same cable, at the same time as this console) — drag files
off it directly instead of anything more complicated. It only works while
you're actually in this menu; normal operation keeps the card to itself.

## 6. Commissioning the reference position (one-time, do this last)

**Do this only once the antenna is mounted at its final, permanent
location** — option `C` in the menu above averages GNSS fixes over a real
survey window (`reference_window_s`, default long enough to average out
multipath/atmospheric noise) and locks the result in permanently as the
station's ground-movement benchmark. Every later fix gets reported as a
local North/East/Up displacement from this point, which is the actual
measurement a permafrost station cares about — not absolute position, but
how far the ground has moved since commissioning.

If you commission before final mounting, or move the antenna afterward,
there's no "undo" — you'd need to run `C` again to establish a new
benchmark, which resets the displacement history back to zero.

## 7. Getting data off the station

Two ways, both covered above: the physical eject button (GPIO 41 — safely
unmounts and cuts power to the card so you can pull it and read it in a
laptop's own card reader), or field-service mode's USB drive access (leave
the card in place, just plug in the service cable and browse it directly).
If MQTT/upload credentials are configured, position telemetry and the
daily positions-CSV upload happen automatically over the cellular link too — see
README.md's "Telemetry and daily upload" section for the formats.

## 8. Troubleshooting quick reference

| Symptom | Likely cause |
|---|---|
| `GNSS: 0 chars seen in 3s smoke test` | SEL jumper still soldered, or RX/TX swapped |
| `SD: mount failed.` / `SD: no card detected.` | Card not fully seated — reseat and reboot |
| Fix quality stuck at `no fix`/`autonomous` | No NTRIP caster configured, or caster unreachable — check `secrets.h`/field-service menu option 6 |
| `NTRIP caster returned 401 Unauthorized` a few times, then connects | Normal — the firmware retries with increasing delays; this specific caster's sessions sometimes take a few seconds to clear server-side |
| Nothing on the serial monitor right after reconnecting | Cosmetic timing quirk, not a crash — see README.md's "Notes / known issues"; just reconnect once more |
| Station reboots on its own after ~60s of no output | The task watchdog firing — a real stall, not expected in normal operation; worth investigating what was blocking |

For anything not covered here, the inline comments throughout `src/` are
extensive and explain the *why* behind almost every non-obvious decision —
that's usually the fastest way to understand a specific behavior.
