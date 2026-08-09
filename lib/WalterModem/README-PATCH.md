# Vendored copy - local patch applied

This is `quickspot/WalterModem` 1.5.0, vendored here (instead of pulled via
`platformio.ini`'s `lib_deps`) because of a crash bug confirmed on real
hardware that isn't fixed in the current release.

## The bug

`WalterModem::_processModemRSP()` decides whether to finish a command with:

```cpp
if((cmd != NULL && cmd->atRsp != NULL && memcmp(...) == 0) || result != WALTER_MODEM_STATE_OK) {
  _finishModemCMD(cmd, result);
```

The `result != WALTER_MODEM_STATE_OK` branch isn't gated on `cmd != NULL`.
Any error response (e.g. `+CME ERROR: 4`) that arrives while no command is
actually outstanding (`_curCmd == NULL` - a stray/unsolicited error, not a
reply to anything the app sent) reaches `_finishModemCMD(NULL, result)`,
which immediately dereferences `cmd->rsp` with no null check:

```cpp
void WalterModem::_finishModemCMD(WalterModemCmd* cmd, WalterModemState result)
{
  cmd->rsp->result = result;   // <-- crash: LoadProhibited, cmd == NULL
```

Reproduced repeatedly on a Walter ESP32-S3 board pulling RTCM corrections
over NTRIP: during the caster's periodic station-metadata burst
(RTCM 1007/1008/1033, roughly every 10s), a stray `+CME ERROR: 4` reliably
lands with nothing outstanding, crashing the device
(`Guru Meditation Error ... LoadProhibited`, always at the same
`_finishModemCMD` line and fault address). This happened identically
whether socket receive/send were blocking or async, and across several
chunk sizes and pacing strategies - none of that affects the raw byte
stream the modem sends, which is what triggers this.

## The fix

`_finishModemCMD()` now returns immediately if `cmd == NULL` - there's
nothing to finish/notify for a command that doesn't exist, so this is
correct behavior, not just a crash workaround. See the `PATCH` comment at
the top of `_finishModemCMD()` in `src/WalterModem.cpp`.

## Removing this

If a future WalterModem release fixes this upstream, delete this directory,
restore `quickspot/WalterModem @ ^x.y.z` to `platformio.ini`'s `lib_deps`,
and delete `include/spi_flash_mmap.h` if it's no longer needed either.
