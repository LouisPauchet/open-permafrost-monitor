# Positions CSV

One file per UTC day, `data/positions_YYYYMMDD.csv`. It gets **one row per
GNSS burst**, including bursts that got no fix. The same rows are sent over
[MQTT](mqtt.md) and [uploaded daily](upload-api.md).

```text
time,latitude,longitude,altitude,horizontal_accuracy,vertical_accuracy,fix_quality,quality_ok,battery_external_v,battery_internal_v,has_displacement,north_m,east_m,up_m,epoch_ms,sd_free_mb
2026-10-27T10:17:41Z,78.22320120,15.65120340,12.34,0.014,0.022,4,true,12.61,4.11,true,0.0120,-0.0040,-0.0310,1793096261000,27310
2026-10-27T10:27:03Z,0.00000000,0.00000000,0.00,-1.000,-1.000,0,false,12.58,4.10,false,0.0000,0.0000,0.0000,1793096823000,27305
```

## Columns

| Column | Unit | Description |
|---|---|---|
| `time` | ISO 8601 UTC | End-of-burst time, second resolution |
| `latitude`, `longitude` | degrees (WGS84) | 8 decimals. **0 when `fix_quality` = 0.** |
| `altitude` | m | Above mean sea level, as reported by the receiver |
| `horizontal_accuracy` | m | √(σ_lat² + σ_lon²) from the receiver's own 1-σ error estimates (NMEA GST). `-1` = no estimate. |
| `vertical_accuracy` | m | 1-σ altitude error (NMEA GST) |
| `fix_quality` | code | NMEA GGA indicator: 0 no fix, 1 autonomous, 2 DGPS, **4 RTK fixed**, 5 RTK float |
| `quality_ok` | bool | `true` if `horizontal_accuracy` reached the good-fix threshold (option `4`, default 0.05 m) during the burst |
| `battery_external_v` | V | 12 V pack (calibrated divider) |
| `battery_internal_v` | V | Walter battery (**uncalibrated** divider) |
| `has_displacement` | bool | `true` only when this row's fix was `quality_ok` **and** a reference position is locked |
| `north_m`, `east_m`, `up_m` | m | Displacement from the [reference position](../field/commissioning.md), local tangent plane. 0 when `has_displacement` is false. |
| `epoch_ms` | ms since 1970 UTC | Same instant as `time`, machine-friendly |
| `sd_free_mb` | MB | SD free space; `-1` = unknown. Below 1024, MiniSEED is paused. |

!!! warning "Filter before plotting"
    - Ignore positions where `fix_quality` = 0 (they are placeholders).
    - For movement analysis, use only rows with `has_displacement = true`.
      Other fixes are logged but not accurate enough.

## Reading it

```python
import pandas as pd

df = pd.read_csv("positions_20261027.csv", parse_dates=["time"])
good = df[df.has_displacement]
good["horizontal_m"] = (good.north_m**2 + good.east_m**2) ** 0.5
```
