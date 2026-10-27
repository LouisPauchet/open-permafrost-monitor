# SD card layout

```text
/data/
├── positions_20261027.csv            one per UTC day, one row per GNSS burst
├── positions_20261028.csv
├── geophones/
│   ├── geophone_20261027.mseed       one per UTC day, continuous
│   └── geophone_20261028.mseed
└── gnss/
    ├── 20261027_101502_041.ubx       one pair per GNSS burst
    ├── 20261027_101502_041.rtcm
    └── ...
```

The folders are created at every boot if they are missing. There are no
log files: the console is the only log.

| File | Name pattern | Rotation | Format |
|---|---|---|---|
| Positions | `data/positions_YYYYMMDD.csv` | At UTC midnight | CSV with header; see [Positions CSV](positions-csv.md) |
| Seismic | `data/geophones/geophone_YYYYMMDD.mseed` | At UTC midnight (by sample time) | SEED 2.4 MiniSEED, 512-byte records; see [MiniSEED](miniseed.md) |
| GNSS raw | `data/gnss/YYYYMMDD_HHMMSS_NNN.ubx` | One per burst | Every valid UBX frame from the F9P (incl. RXM-RAWX, RXM-SFRBX) |
| Corrections | `data/gnss/YYYYMMDD_HHMMSS_NNN.rtcm` | One per burst that used NTRIP | Raw RTCM3 stream exactly as received from the caster |

In the GNSS names, `HHMMSS` is the UTC time of the burst and `NNN` is a
counter (mod 1000) that keeps names unique. A burst that starts before
GNSS knows the date, which is rare, is named `boot_<millis>_NNN` instead.

## Writes and power loss

- All writing goes through one background writer task with a ring buffer
  per file. Files are flushed every 5 s. The positions CSV is flushed after
  every row.
- On a sudden power cut you can lose up to about 5 s of MiniSEED and raw
  GNSS data, but not completed position rows.
- Files are closed cleanly before every reboot the firmware itself
  triggers: entering the menu, eject, SD recovery.
- After a same-day reboot, writing continues in the same daily file
  (append). A MiniSEED file can then contain a short gap.

## Low space

Free space is checked every minute and reported as `sd_free_mb` in every
position row. Below **1 GB free**, MiniSEED writing pauses. The geophone is
still read and timed, so recording resumes cleanly after a card swap.
Positions and GNSS raw keep being written. Telemetry shows this as
`sd_free_mb < 1024`.
