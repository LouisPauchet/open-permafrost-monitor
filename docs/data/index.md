# Data

The station produces four data products. Each one has its own path off the
station.

```mermaid
flowchart LR
  subgraph Sensors
    GEO["RS3D geophone<br/>3 ch × 200 sps, continuous"]
    GNSS["ZED-F9P<br/>bursts every 10 min"]
  end
  subgraph SD["SD card (everything)"]
    MS["geophones/geophone_YYYYMMDD.mseed"]
    UBX["gnss/&lt;stamp&gt;.ubx + .rtcm"]
    CSV["positions_YYYYMMDD.csv"]
  end
  GEO --> MS
  GNSS -- raw observations --> UBX
  GNSS -- "fix, accuracy, displacement<br/>+ battery + SD free" --> CSV
  CSV -- "every burst (JSON batch)" --> MQTT[("MQTT broker")]
  CSV -- "once per finished UTC day" --> BE[("Upload backend → S3")]
  MS -. "physical collection only" .-> YOU(["You"])
  UBX -. "physical collection only" .-> YOU
```

| Product | Where | Latency | Page |
|---|---|---|---|
| Positions: fix, accuracy, displacement, battery, SD free | SD + MQTT + daily upload | About 10 min (MQTT) | [Positions CSV](positions-csv.md), [MQTT](mqtt.md) |
| Daily positions file | Your backend | About 1 day | [Upload backend API](upload-api.md) |
| Seismic waveforms | SD only | Until the next visit | [MiniSEED](miniseed.md) |
| Raw GNSS observations + corrections | SD only | Until the next visit | [GNSS raw / PPK](gnss-raw.md) |

All times are **UTC**, derived from GNSS. See [Timing](../how-it-works/timing.md)
for how accurate they are.
