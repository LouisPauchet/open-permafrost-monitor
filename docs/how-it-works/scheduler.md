# GNSS fix cycle

`loop()` runs forever. It polls the GNSS UART, the buttons and the
field-service pin, checks the battery every 30 s, and starts a **burst**
when one is due:

- **First burst:** immediately after boot.
- **Commissioning:** back-to-back.
- **Otherwise:** `measurement_interval_s` (default 600 s) after the start
  of the previous burst.

No burst starts while the battery is below the cutoff; see
[Power](power.md#low-battery-backoff).

## One burst, step by step

```mermaid
flowchart TD
  W["Wake F9P (EXTINT)<br/>configure outputs"] --> IF{"Initial fix within<br/>fix_timeout_s?"}
  IF -- no --> NF["Log no-fix row"]
  IF -- yes --> NT{"NTRIP this burst?<br/>(every Nth, or commissioning)"}
  NT -- no --> PPK["Record raw UBX until<br/>ppk_min_duration_s since fix"]
  NT -- yes --> MC["Wake modem, register<br/>connect NTRIP, send GGA"]
  MC --> RTK["Forward RTCM → F9P<br/>GGA every 10 s<br/>until accuracy ≤ good OR timeout<br/>AND ppk floor met"]
  PPK --> LOG
  RTK --> LOG["Append position row<br/>(+ displacement or commissioning sample)"]
  NF --> LOG2["Append no-fix row"]
  LOG --> FIN
  LOG2 --> FIN["Disconnect NTRIP<br/>queue MQTT batch + daily upload<br/>F9P → backup mode"]
  FIN --> SL["Modem → NO_RF<br/>once its jobs finish"]
```

If the burst is not an NTRIP burst and there is no initial fix, it ends at
once. The modem is not woken.

## Sequence with the outside world

```mermaid
sequenceDiagram
  participant L as loopTask
  participant F as ZED-F9P
  participant M as Modem
  participant C as NTRIP caster
  participant W as modemWorker
  participant B as MQTT broker
  participant U as Upload backend

  L->>F: EXTINT pulse (wake), enable NMEA GST, RAWX, SFRBX
  F-->>L: NMEA + UBX (UBX saved to SD)
  Note over L,F: wait for first fix (≤ fix_timeout_s)
  L->>M: setOpState FULL, wait for registration
  L->>C: TCP connect + NTRIP request (401 → retry 5/15/30/45 s)
  L->>C: GGA (immediately, then every 10 s)
  loop until RTK done AND PPK floor met
    C-->>L: RTCM3
    L->>F: RTCM3 (also saved to .rtcm)
    F-->>L: fix quality, GST accuracy
    Note over L: no RTCM for 15 s → reconnect (≤ 2 per burst)
  end
  L->>L: append CSV row
  L->>C: disconnect
  L->>W: queue MQTT batch job, queue upload job (if a day is due)
  L->>F: UBX-RXM-PMREQ backup (sleep)
  W->>B: TLS connect, publish QoS 1, disconnect
  W->>U: HTTPS POST /upload/positions_YYYYMMDD.csv
  W-->>L: results (mark rows sent / advance upload date)
  L->>M: setOpState NO_RF (when jobs are done)
```

## Timers and limits

| Constant | Value | Where | Purpose |
|---|---|---|---|
| `measurement_interval_s` | 600 s (menu `2`) | NVS | Burst period, start to start |
| `fix_timeout_s` | 120 s (menu `3`) | NVS | Applied twice: wait for first fix, then wait for good accuracy |
| `good_accuracy_m` | 0.05 m (menu `4`) | NVS | Ends RTK convergence; sets `quality_ok` |
| `ppk_min_duration_s` | 120 s (menu `A`) | NVS | Raw-recording floor after first fix |
| `ntrip_every_n_bursts` | 1 (menu `E`) | NVS | NTRIP/modem on every Nth burst |
| `GGA_RESEND_INTERVAL_MS` | 10 s | `main.cpp` | Network RTK/VRS mounts need the rover position |
| `GGA_FAILURE_RETRY_MS` | 1.5 s | `main.cpp` | Faster retry after a failed GGA send |
| `NTRIP_STALL_TIMEOUT_MS` | 15 s | `main.cpp` | No RTCM: reconnect |
| `NTRIP_MAX_RECONNECTS_PER_BURST` | 2 | `main.cpp` | |
| NTRIP connect attempts | 5; 401 waits 5/15/30/45 s, others 10 s | `ntrip_client.cpp` | Caster-side sessions take time to clear |
| `MODEM_JOB_WAIT_MS` | 60 s | `main.cpp` | Wait for previous MQTT/upload jobs before NTRIP |
| `GNSS_CONFIG_REFRESH_INTERVAL_MS` | 5 s | `main.cpp` | Re-send F9P output config (GST was seen to stop) |
| `UPLOAD_MAX_BACKLOG_DAYS` | 14 | `main.cpp` | Upload catch-up limit |

### Why wait for a fix before NTRIP?

Network-RTK (VRS) casters build the correction stream around the position
in the first GGA they receive. Connecting before the receiver has any fix
would waste the session on an empty GGA. The modem has nothing else to do
during that wait.

### Why at least `ppk_min_duration_s`?

RTK can converge in about 20 s. Stopping then would leave too little raw
data for reliable post-processing. The burst runs until **both** the RTK
side is done and the PPK floor is met.

## Telemetry and the daily upload

Both run **at the end of a burst**, on the modem session NTRIP already
opened. They add no extra wake-ups and are battery-gated like any burst:

- **MQTT:** all unsent rows from today's CSV, as one message; see
  [MQTT](../data/mqtt.md).
- **Upload:** the oldest finished UTC day not yet uploaded, one per burst;
  see [Upload API](../data/upload-api.md). The "next day to upload" is
  kept in NVS (namespace `upload`, key `next_date`).
