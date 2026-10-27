# GNSS raw data and PPK

Every burst records the receiver's **raw observations**. You can
post-process them later (PPK) for positions that are independent of, and
often better than, the real-time RTK result. This also works for bursts
that had no NTRIP.

## Files

Each burst writes a pair of files to `data/gnss/`. Both share a stamp,
`YYYYMMDD_HHMMSS_NNN`, made from the UTC start time and a burst counter.

| File | Content |
|---|---|
| `<stamp>.ubx` | Every checksum-valid UBX frame from the F9P, including **RXM-RAWX** (pseudorange, carrier phase, Doppler) and **RXM-SFRBX** (broadcast navigation data). NMEA is not stored. |
| `<stamp>.rtcm` | The raw RTCM3 correction stream received from the NTRIP caster during the burst. Only written for bursts that used NTRIP. |

## How long each burst records

After the first fix, raw data is recorded for at least the **PPK minimum
duration** (menu `A`, default 120 s). The burst continues past that while
RTK is still converging, up to the fix timeout. It ends when **both** are
done:

- RTK: good accuracy reached, timeout, or no NTRIP this burst.
- PPK: the minimum duration since the first fix has passed.

A burst with **no first fix** within the fix timeout ends at once, and
records almost no raw data.

## Bursts without NTRIP (option E)

With **NTRIP every Nth burst** set to N > 1, the bursts in between record
only the `.ubx` file. They don't wake the modem, so they produce no `.rtcm`
file and no MQTT telemetry. Those bursts still appear as rows in the
positions CSV, at autonomous quality.

To post-process them you need base-station observations from elsewhere:
for example, RINEX from a permanent reference station, or from your
correction provider for the same period.

## Post-processing (example)

With [RTKLIB](https://www.rtklib.com/) (`convbin` / `rnx2rtkp`), or its
GUI tools RTKCONV and RTKPOST:

```sh
convbin -r ubx -od -os 20261027_101502_041.ubx            # → .obs / .nav RINEX
rnx2rtkp -k ppk.conf 20261027_101502_041.obs base.obs 20261027_101502_041.nav
```

`ppk.conf` is your own RTKLIB processing configuration, and `base.obs` is
base-station RINEX. The `.rtcm` file can also be converted with `convbin -r
rtcm3` and used as base data, when the caster mount is a single physical
base station. A network-RTK/VRS mount's virtual base is centred on the
rover's position at connection time.
