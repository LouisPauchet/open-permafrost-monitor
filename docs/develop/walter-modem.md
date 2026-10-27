# WalterModem library

`lib/WalterModem/` is a **vendored** copy of `quickspot/WalterModem` 1.5.0,
with crash-fix patches. PlatformIO prefers a project `lib/` folder over a
registry package of the same name. So the library is deliberately **not**
in `lib_deps`; adding it there would not override the patched copy, but
would confuse readers.

## The patch

A stray `+CME ERROR` that arrives while no AT command is outstanding made
`_finishModemCMD(NULL, …)` dereference a null pointer (`LoadProhibited`
crash). This happened reliably during NTRIP sessions, around the caster's
periodic RTCM 1007/1008/1033 burst. The fix: return early when `cmd` is
`NULL`.

Full analysis:
[`lib/WalterModem/README-PATCH.md`](https://github.com/LouisPauchet/open-permafrost-monitor/blob/main/lib/WalterModem/README-PATCH.md).
Library changelog: `lib/WalterModem/CHANGELOG.md`.

## How the firmware uses it

| Use | Mode | Runs on |
|---|---|---|
| NTRIP | TCP socket in **command mode** (`AT+SQNSSEND`/`AT+SQNSRECV`), async receive | `loopTask`, during a burst |
| MQTT telemetry | Modem's built-in MQTT client, TLS profile, QoS 1 | `modemWorker` job |
| Daily upload | Modem's built-in HTTP client, TLS 1.2, Basic auth | `modemWorker` job |
| Sleep | `setOpState(NO_RF)` / `FULL` | `loopTask` |

The socket's transparent ("online") mode was tried to avoid the AT
framing. The caster never received data that way on this modem firmware,
so command mode is used.

The CA certificate is written to the modem only when it has changed, and
rewritten once if a TLS job fails.

## Known constraints

- Only **one** user of the modem at a time. The NTRIP socket and the
  worker's jobs never overlap; see [Architecture](../how-it-works/architecture.md#rules-that-keep-this-safe).
- `httpSend` takes a `uint16_t` length. Uploads are chunked at 32 KB.
- WalterModem 1.5.0 can't set a custom `Authorization` header, hence HTTP
  Basic auth with the API key as the username.
- A future migration away from this library is under consideration. Until
  then, keep the pin to espressif32 6.13.0.
