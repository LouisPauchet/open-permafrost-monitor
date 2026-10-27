# MiniSEED (geophone)

| Property | Value |
|---|---|
| File | `data/geophones/geophone_YYYYMMDD.mseed`, one per UTC day |
| Format | SEED 2.4 data records, 512 bytes, blockette 1000, quality `D` |
| Encoding | **Steim2** (default) or uncompressed int32; menu option `G` |
| Channels | `EH1`, `EH2`, `EH3` (names as sent by the digitizer) |
| Sample rate | **200 Hz**: the raw digitizer rate, not decimated |
| Units | Raw ADC **counts** (no instrument response applied) |
| Codes | Network / station / location from menu option `F` (default `XX` / `PERM1` / blank) |

Steim2 is lossless. The firmware's encoder is checked against ObsPy in the
test suite.

## Reading the data

```python
from obspy import read

st = read("geophone_20261027.mseed")
st.merge()          # join records into continuous traces
print(st)           # XX.PERM1..EH1 | 2026-10-27T00:00:00.002 - ... | 200.0 Hz, ...
```

SeisComP, libmseed and other standard tools read the files directly.

## Converting counts to ground velocity

Divide the counts by the digitizer sensitivity. For the RS3D board
revision used here (V11, in the spec sheet's "V7+" bracket):

**velocity [m/s] = counts / 3.9965 × 10⁸**

Other board revisions are listed in
[Raspberry Shake protocol](../how-it-works/raspberry-shake.md#converting-counts-to-velocity).
The value is a flat sensitivity. For full instrument correction below about
1 Hz, use the Raspberry Shake response files for the RS3D.

```python
for tr in st:
    tr.data = tr.data / 3.9965e8   # m/s
```

!!! note "Channel orientation"
    The digitizer's names `EH1`, `EH2` and `EH3` are kept as they are.
    Which channel is vertical, and the azimuth of the horizontals, depends
    on how the geophones are mounted. Record it at installation.

## Timing accuracy

Sample times come from the digitizer's own clock, mapped to GNSS UTC. See
[Timing](../how-it-works/timing.md) for the full mechanism.

- Records are **contiguous**: each sample is exactly 5 ms after the
  previous one. There is no jitter from the ESP32.
- The UTC mapping follows slow drift, and only jumps (a "resync") on a real
  discontinuity of more than 20 ms, such as a lost packet.
- **Known constant offset, not calibrated:** the delay between the
  digitizer sampling and the marker reaching the ESP32 (UART +
  buffering). All channels share it, so it cancels in inter-channel work,
  but it matters for absolute event timing.
- Between GNSS bursts (about 10 min), UTC is extrapolated from the ESP32
  clock, so long-term accuracy depends on its drift until the next fix.

!!! tip "Gaps"
    Gaps happen:

    - during service-menu visits and card swaps;
    - after every reboot, **until the first GNSS fix** (samples can't be
      timestamped before it, so they are dropped);
    - while the card is below 1 GB free.

    `st.get_gaps()` in ObsPy lists them.
