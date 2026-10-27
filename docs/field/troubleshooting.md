# Troubleshooting

The serial console (115200 baud over USB) is the only diagnostic output.
There is no LED and no log file. Telemetry gives a first remote view:
`fix_quality`, `quality_ok`, battery voltages and `sd_free_mb` all arrive
over MQTT.

## Remote symptoms (from telemetry)

| Symptom | Likely cause | Action |
|---|---|---|
| No MQTT messages at all | Battery below cutoff; no cellular coverage; broker or TLS problem; station down | Check the last `battery_external_v`. If it was near the cutoff, wait for recovery (cutoff + 0.2 V). |
| Messages arrive with `fix_quality: 0` and no position | Antenna covered (snow, ice) or disconnected | Station is "alive but blind": visit, or wait for melt |
| `quality_ok: false` with `fix_quality` 1 or 2 | No RTK: NTRIP down or credentials expired | Check the caster account; check `6` in the menu |
| Messages only every Nth burst | Option `E` > 1 | Expected |
| `sd_free_mb` below 1000 | Card almost full: **geophone recording paused** | Swap or empty the card |
| MiniSEED missing since a reboot, and no fixes since | Geophone samples are dropped until the first GNSS fix after boot | Restore the antenna's sky view; see [Timing](../how-it-works/timing.md) |
| Gaps in daily uploads | Backend down, or the station was offline | The station catches up one day per burst, up to 14 days back |

## Console messages

| Message | Meaning | Action |
|---|---|---|
| `GNSS: 0 chars seen in 3s smoke test` | No data from the F9P | SEL jumper closed, RX/TX swapped, no 5 V; see [Wiring](../build/wiring.md#gnss-zed-f9p) |
| `WARNING: no GNSS data received this cycle` | F9P silent during a burst | Wiring, or the F9P did not wake (EXTINT) |
| `SD: mount failed.` / `SD: no card detected.` | Card not seated, bad card, or card-detect wiring | Reseat (a half-seated card looks inserted); without a detect switch, tie GPIO 39 HIGH |
| `SD writer: stream N write failed (...)` | Card write error | Replace the card |
| `SD: card present but unusable - recovery reboot n/2.` then `SD: card still unusable after recovery reboots...` | Card fails to mount or write | Station reboots twice to recover, then runs without SD; replace the card |
| `SDdrop` > 0 in `[STATUS]` | Card too slow, so data is being lost | Use a faster, high-endurance card |
| `MiniSEED: SD card nearly full - geophone recording paused.` | Under 1 GB free | Swap or empty the card |
| `No initial fix within timeout - ending this burst` | No fix in `fix_timeout_s` | Sky view and antenna cable. After a long power-off, the first fix is slower. |
| `NTRIP connect attempt n/5 failed - retrying in Xs...` | Caster refused or unreachable | Some 401s are normal (the caster still holds an old session): retries wait 5, 15, 30, then 45 s. Persistent: check the credentials and mountpoint. |
| `NTRIP connect: giving up for this fix cycle.` | All attempts failed | Burst continues without RTK; retried next burst |
| `NTRIP: no RTCM for 15s - reconnecting (n/2).` | Correction stream stalled | Automatic. Frequent: caster or coverage issues. |
| `Modem: registration timed out.` | No cellular network | SIM, data plan, bands, antenna, coverage |
| `Modem: begin() keeps failing - running without the modem until power cycle.` | Modem chip did not start | Power cycle. If it repeats, suspect hardware. |
| `Telemetry: modem not connected, skipping this batch.` | No modem this burst | Rows stay unsent and go out with the next batch |
| `MQTT: connect failed.` / `MQTT: broker rejected the connection.` / `MQTT: publish failed.` | Broker refused or unreachable | Host, port, token, topic; broker certificate must chain to Let's Encrypt || `Daily upload: <date> failed - retrying next cycle.` | Upload backend error | Check the backend; it retries automatically |
| `Battery: X V below Y V cutoff - skipping new GNSS/modem bursts` | Low battery | Expected protection; recovers at cutoff + 0.2 V |
| Reboot after about 60 s with no output | Task watchdog: something hung | Note what was printed last and report it ([Contributing](../develop/contributing.md)) |
| Nothing printed right after connecting | The console missed the first lines | Harmless; press RESET or reconnect |

## The station keeps rebooting

Copy the console output, starting before the reboot, and look at the last
lines:

- `Guru Meditation Error` / `Backtrace:` is a crash. The monitor's
  exception decoder (`pio device monitor -e walter`) turns the backtrace
  into source lines. Report it.
- `Service cable detected - rebooting into field service mode.` means
  GPIO 40 is grounded, or floating because of a damaged wire.
- Reboots right after `SD: …` lines are SD recovery reboots. The station
  stops after two and runs without SD.
