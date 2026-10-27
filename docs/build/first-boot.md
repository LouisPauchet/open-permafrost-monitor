# First boot

The station has **no status LED**. The USB serial console (115200 baud) is
the only indicator. Do this first boot on the bench, with the GNSS antenna
outside or by a window.

## Expected boot log

```text
Walter <-> ZED-F9P + RS3D permafrost/seismic field station
No saved config found, using compiled-in defaults.     ← first boot only
Reference position: not commissioned yet.
SD: mounted, 30436 MB
SD: 30410 MB free
GNSS: listening for raw UART bytes (wiring/baud check)...
GNSS: 4211 chars seen in 3s smoke test (0 = check wiring/baud/SEL jumper, not sleep/wake).
Waiting up to 120 s for an initial fix before starting NTRIP...
```

On later boots, the second line reads `Loaded device config from NVS.`

| Line | What it checks | If it's wrong |
|---|---|---|
| `Loaded device config…` / `No saved config…` | Settings source; see [Configuration model](../how-it-works/configuration-model.md) | — |
| `Reference position: …` | Commissioning state | Normal on a new station |
| `SD: mounted, N MB` | SD wiring and card | `SD: mount failed.` or `SD: no card detected.`: reseat the card, then check wiring and card-detect |
| `GNSS: N chars seen…` | F9P UART wiring | **0 chars**: SEL jumper closed, TX/RX swapped, or no 5 V to the F9P |
| `Modem: begin() failed - rebooting to retry (n/m).` | Modem start-up | The station reboots a few times to retry, then runs without the modem until the next power cycle (`Modem: begin() keeps failing…`). Logging continues. |
| `Modem: registration timed out.` | Cellular registration | Check the SIM, data plan, coverage and antenna. Retried every burst. |

## The first GNSS burst

The first burst starts right after boot:

```text
Waiting up to 120 s for an initial fix before starting NTRIP...
Modem: registered.
NTRIP stream connected, forwarding RTCM to F9P.
Waiting up to 120 s more for a fix with horizontal accuracy <= 0.050 m (recording at least 120 s for PPK)...
[STATUS] t=10s fix=RTK FLOAT hAcc=0.214m RTCM=8121B SDdrop=0B heap=163420B
[STATUS] t=20s fix=RTK FIXED hAcc=0.018m RTCM=17344B SDdrop=0B heap=163112B
...
Fix quality: RTK FIXED
Horizontal accuracy: 0.014 m (good=yes)
RTCM bytes forwarded: 98213
```

!!! info "Numbers are examples"
    Sizes, accuracies and timings above are illustrative. Fix labels and
    message text match the firmware.

`[STATUS]` appears every 10 s during a burst:

- `fix`: fix quality
- `hAcc`: horizontal accuracy estimate in metres
- `RTCM`: correction bytes received
- `SDdrop`: bytes lost because the SD card couldn't keep up. **Should
  stay 0.**
- `heap`: free memory

A burst takes **at least** the PPK minimum (120 s) after the first fix. In
the worst case it takes up to two fix timeouts (2 × 120 s) plus NTRIP retry
time (up to about 95 s). After it, the GNSS and modem go to sleep until the
next burst, 600 s after this one started.

Without an NTRIP caster configured, the fix stays at `autonomous` or `DGPS`
and the burst ends on the PPK minimum. That is expected.

## Bench checklist

- [ ] Boot log shows SD mounted and a non-zero GNSS char count
- [ ] A burst reaches a fix (RTK if NTRIP is configured)
- [ ] `SDdrop=0B` throughout
- [ ] Ground GPIO 40: the station reboots into the [service menu](../field/configuration.md); the SD card appears as a USB drive
- [ ] The drive contains `data/geophones/geophone_<today>.mseed` and it grows between checks
- [ ] Option `1` in the menu shows a sensible battery voltage
- [ ] MQTT (if configured): one telemetry message arrives per burst
- [ ] Press the eject button: the station logs the eject. Reinsert the card: it remounts.
