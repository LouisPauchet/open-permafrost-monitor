# Architecture

## Hardware

```mermaid
flowchart TB
  subgraph Walter["Walter board"]
    ESP["ESP32-S3<br/>dual core, 16 MB flash, 2 MB PSRAM"]
    MOD["Sequans GM02SP<br/>LTE-M / NB-IoT modem"]
    ESP <-- "UART, AT commands<br/>(WalterModem library)" --> MOD
  end
  F9P["u-blox ZED-F9P"] <-- "UART2 · 38400<br/>NMEA+UBX out / RTCM+UBX cfg in" --> ESP
  ESP -- "EXTINT pulse (wake)" --> F9P
  RS3D["RS3D digitizer"] -- "UART1 · 230400 · JSON" --> ESP
  ESP <-- "SPI · 4 MHz" --> SD[("microSD")]
  ESP <-- "native USB<br/>CDC console + MSC drive" --> PC["Laptop<br/>(service only)"]
  BTN["Eject btn · Service pin"] --> ESP
  BAT["Battery dividers"] --> ESP
  MOD <-- LTE --> Cloud["NTRIP · MQTT · HTTPS"]
```

## Firmware tasks

The firmware runs four tasks of its own, plus the modem library's internal
ones. The split exists so that nothing slow can make the geophone lose
samples, and nothing blocking can trip the watchdog.

```mermaid
flowchart LR
  subgraph core1["Core 1"]
    LOOP["loopTask (16 KB)<br/>scheduler · GNSS UART · NTRIP<br/>buttons · battery · watchdog"]
  end
  subgraph core0["Core 0"]
    GEO["geophoneTask (prio 2)<br/>UART1 drain → parse → MiniSEED"]
    SDW["sdWriterTask (prio 1)<br/>ring buffers → FatFs files"]
    MW["modemWorker (prio 1)<br/>job queue: MQTT batch, HTTP upload"]
  end
  RS3D[RS3D] --> GEO
  F9P[F9P] <--> LOOP
  GEO -- "512-byte records" --> SDW
  LOOP -- "UBX, RTCM, CSV rows" --> SDW
  LOOP -- "queue job" --> MW
  MW -- "result flags" --> LOOP
  LOOP -- "NTRIP socket" --> MODEM[(WalterModem)]
  MW --> MODEM
  SDW --> SD[(SD card)]
  MW -. "reads CSV for upload" .-> SD
```

| Task | Stack / priority / core | Job | Source |
|---|---|---|---|
| `loopTask` | 16 KB, Arduino default, core 1 | Scheduler: GNSS bursts, NTRIP relay, battery, buttons, field-service trigger. Fed to the 60 s watchdog. | `src/main.cpp` |
| `geophoneTask` | 6 KB, prio 2, core 0 | Drains the digitizer UART, parses JSON packets, timestamps samples, builds MiniSEED records | `src/geophone.cpp`, `src/miniseed_writer.cpp` |
| `sdWriterTask` | 10 KB, prio 1, core 0 | Only task that writes files. Producers just `memcpy` into per-stream ring buffers. | `src/sd_storage.cpp` |
| `modemWorker` | 8 KB, prio 1, core 0 | Runs blocking AT sequences (MQTT connect/publish, HTTP POST) one at a time | `src/modem.cpp` |

### Rules that keep this safe

- **Producers never block on SD.** `sdStreamWrite()` copies into a ring
  buffer. If the card stalls, data is dropped and counted (`SDdrop` in
  `[STATUS]`) rather than freezing the producer.
- **One modem user at a time.** The NTRIP socket is driven from `loopTask`
  during a burst. MQTT and upload jobs run on `modemWorker` *after* NTRIP
  disconnects. A new burst waits up to 60 s for unfinished jobs before
  touching the modem.
- **The modem sleeps only when its jobs are done.** `loopTask` sets
  `modemSleepPending`, and calls `modemSleep()` once `modemJobsBusy()` is
  false.
- **Service mode never runs the geophone task.** The geophone task starts
  only after the field-service check, so it can't write while the card is
  exposed over USB.

## Module map

```mermaid
flowchart TD
  main[main.cpp] --> gnss[gnss]
  main --> ntrip[ntrip_client]
  main --> mqtt[mqtt_client]
  main --> upload[upload_client]
  main --> poslog[position_log]
  main --> ref[reference_position]
  main --> bat[battery]
  main --> fs[field_service]
  main --> geo[geophone]
  geo --> gparse[geophone_parse]
  geo --> msw[miniseed_writer]
  msw --> gtime[geophone_timing.h]
  msw --> msrec[miniseed_record]
  ntrip --> modem[modem]
  mqtt --> modem
  upload --> modem
  modem --> WM[("lib/WalterModem")]
  gnss --> ubx[ubx.h]
  gnss --> sd[sd_storage]
  msw --> sd
  poslog --> sd
  upload --> sd
  fs --> cfg[device_config]
  fs --> sd
  main --> cfg
```

File-by-file details: [Code map](../develop/code-map.md).

## Boot sequence

```mermaid
flowchart TD
  A[USB up, console] --> B["Load config (NVS)<br/>reference position, upload state"]
  B --> C[Mount SD, start writer task,<br/>prepare USB drive]
  C --> D[Wake F9P, configure,<br/>3 s UART smoke test, sleep F9P]
  D --> E{GPIO 40 grounded?}
  E -- yes --> F["Field service menu<br/>(SD exposed over USB)"] --> R([reboot])
  E -- no --> G[Start geophone task]
  G --> H[Arm 60 s watchdog]
  H --> I[Init modem]
  I --> J(["loop(): first GNSS burst immediately"])
```
