#include "modem.h"
#include "isrg_root_x1.h"
#include "geophone.h"
#include "sd_storage.h"
#include <WalterModem.h>
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <atomic>
#include <string.h>

static WalterModem modem;

static const int MODEM_PDP_CTX = 1;
static const int MODEM_SOCKET_ID = 1;
// Slots 0-9 are reserved for Sequans/BlueCherry-internal use (confirmed both
// in tlsWriteCredential()'s own doc comment - "recommended to save
// credentials in index 10-19 to avoid overwriting preinstalled certificates"
// - and empirically: this was the root cause of every "AT+SQNSMQTTCFG ...
// ERROR" seen tonight referencing a TLS profile. Slot 0 (reserved) let
// tlsWriteCredential/tlsConfigProfile both report success, but referencing
// that profile from mqttConfig failed outright; open_renewable_lab (same
// board/library) uses slot 12 and works. Matching that here.
static const uint8_t MODEM_CA_SLOT = 12; // shared Let's Encrypt root, both TLS profiles below
static const uint8_t MODEM_TLS_PROFILE_MQTT = 1;
static const uint8_t MODEM_TLS_PROFILE_HTTP = 2;
static const int MODEM_HTTP_PROFILE = 1;

// socketReceive is delivered via the WALTER_MODEM_SOCKET_EVENT_RING event,
// not a polled available()/read() pair like WiFiClient - this buffer bridges
// the two so ntrip_client.cpp can keep a simple polling-style read loop.
// Touched from two different tasks now (see the async receive design
// below), so access is guarded by socketRxMutex rather than the old
// single-task assumption.
static uint8_t socketRxBuf[4096];
static size_t socketRxLen = 0;
static SemaphoreHandle_t socketRxMutex = nullptr; // created in modemInit(), after the scheduler is up

// Set by the RING event - just a flag, no AT command issued here.
static volatile bool socketRingPending = false;

static void socketEventHandler(WMSocketEventType event, const WMSocketEventData *data, void *args) {
  if (event == WALTER_MODEM_SOCKET_EVENT_RING) {
    socketRingPending = true;
  }
}

// --- Async socket receive ---
//
// Went through three prior designs, each fixed by live hardware testing:
// (1) call the blocking modem.socketReceive() straight from
//     socketEventHandler() - crashed, because that handler runs on
//     WalterModem's _eventProcessingTask while the main loop concurrently
//     issues its own blocking commands (e.g. GGA sends), and the library's
//     command pipeline isn't safe against two tasks submitting blocking
//     commands at once.
// (2) move the blocking call onto the main loop task only, with small
//     chunks and a pacing gap between calls - fixed the crash, but under a
//     burst (the caster's periodic antenna/receiver metadata resend, RTCM
//     1007/1008/1033) the AT channel couldn't drain fast enough and
//     WalterModem's fixed 8-slot buffer pool ("No free buffers") and
//     command pool both got exhausted.
// (3) drain aggressively instead (max chunk size, loop while reads come
//     back full) - this genuinely helped and is still how a normal burst
//     gets consumed, but occasionally a single AT+SQNSRECV command still
//     gets "lost" inside the library: no "Command time-out" warning ever
//     appears (confirmed on live hardware, the entire gap between the
//     stall and our own watchdog firing), meaning the library's own 180s
//     timeout never even sees it. A *blocking* call has no recourse then
//     except our watchdog - which reboots the device and throws away the
//     whole in-progress fix cycle, exactly when NTRIP corrections matter
//     most.
//
// This design accepts that a command can be lost and stops blocking on it
// at all: kick off one receive with a callback and return immediately.
// GNSS/geophone/everything else keeps running untouched whether or not
// that specific callback ever fires. A request left outstanding too long
// is abandoned (asyncGeneration bumped) so a fresh one can be issued; a
// stale callback firing later is recognized via the generation tag and
// ignored, so it can never write into a buffer a newer request now owns.
//
// The async switch fixed the "lost command" hang, but live hardware then
// hit a *separate*, more severe bug: a hard crash inside WalterModem's own
// _cmdProcessingTask (_finishModemCMD dereferencing a stale command
// pointer - same crash signature, same address, as the very first crash
// fixed weeks ago), independent of blocking vs. async. The AT log showed
// "No free buffers" errors interrupting the hex dump of a 1500-byte
// SQNSRECV response *while it was still being received* - fresh
// +SQNSRING notifications were arriving mid-transfer and corrupting the
// parser's own buffer bookkeeping. That only ever happened after a full
// 1500-byte read, never after the small ones, so a max-size request keeps
// the UART busy long enough for a collision to become likely. Shrinking
// the request size (with the aggressive-loop-while-full behavior in
// drainSocketRing() still doing the catch-up work) narrows that window
// without reintroducing the separate buffer-pool exhaustion an earlier,
// paced version of this function caused - that was from an artificial
// delay between requests, not from a smaller size.
static uint8_t asyncChunk[256];
static WalterModemRsp asyncRsp;
static volatile bool asyncReceiveInFlight = false;
static unsigned long asyncReceiveStartMs = 0;
static volatile uint32_t asyncGeneration = 0;
static const unsigned long ASYNC_RECEIVE_ABANDON_MS = 5000;

static void onSocketReceiveComplete(const WalterModemRsp *rsp, void *args) {
  uint32_t completedGeneration = (uint32_t)(uintptr_t)args;
  if (completedGeneration != asyncGeneration) {
    return; // abandoned already - a newer request may now own asyncChunk
  }
  asyncReceiveInFlight = false;

  if (rsp->result != WALTER_MODEM_STATE_OK) {
    // A stray/unrelated error (e.g. "+CME ERROR: 4" during a burst) can
    // fail this specific SQNSRECV attempt without the modem having
    // actually discarded the data it was reporting via RING - retry
    // rather than silently losing that chunk. Live hardware showed a
    // 120s fix cycle forwarding only ~5.6KB of RTCM (far too little for
    // continuous corrections) even after the crash fix, because failed
    // reads during a burst were dropped instead of retried.
    socketRingPending = true;
    return;
  }

  // data_len on the RING event isn't reliable here: this firmware's
  // +SQNSRING URC carries only the socket id, no length (confirmed via AT
  // log - plain "+SQNSRING: 1", not "+SQNSRING: 1,<n>"), so it comes
  // through as 0 - requesting 0 bytes ("AT+SQNSRECV=1,0") gets
  // "+CME ERROR: 50". Always ask for the max instead (see drainSocketRing)
  // and use the response's own bytesReceived to know how much landed.
  uint16_t n = rsp->data.socketResponse.bytesReceived;
  if (n == 0) {
    return;
  }

  xSemaphoreTake(socketRxMutex, portMAX_DELAY);
  if (socketRxLen + n <= sizeof(socketRxBuf)) {
    memcpy(socketRxBuf + socketRxLen, asyncChunk, n);
    socketRxLen += n;
  } // else drop - the consumer isn't keeping up
  xSemaphoreGive(socketRxMutex);

  if (n == sizeof(asyncChunk)) {
    // Got a full chunk - more is very likely still queued at the modem.
    socketRingPending = true;
  }
}

// Set by modemSuppressReceiveFor() right before a priority send (GGA) so
// it gets one guaranteed-clear shot at the AT channel instead of racing
// the receive side. Checking modemReceiveBusy() alone wasn't enough - live
// hardware showed only ~29% of GGA sends succeeding even with that check,
// because RTCM now drains almost continuously (every RING re-triggers
// another receive immediately), so the "not busy" window it was checking
// for is usually only a few ms wide. A receive that's already in flight
// when the suppression starts is left alone (its own retry/abandon logic
// still applies) - this only holds off *new* kickoffs.
// Start + duration rather than an absolute deadline, so the comparison
// survives millis() wrapping (every ~49.7 days of uptime).
static unsigned long receiveSuppressStartMs = 0;
static unsigned long receiveSuppressDurationMs = 0;

void modemSuppressReceiveFor(unsigned long ms) {
  receiveSuppressStartMs = millis();
  receiveSuppressDurationMs = ms;
}

// Called from the main loop only (modemSocketAvailable()/Read()) - kicks
// off at most one async receive per call, never blocks.
static void drainSocketRing() {
  if (asyncReceiveInFlight) {
    if (millis() - asyncReceiveStartMs > ASYNC_RECEIVE_ABANDON_MS) {
      asyncReceiveInFlight = false;
      asyncGeneration++;
      // Temporary diagnostic: previously silent. Live hardware has been
      // hitting a task-watchdog reset ~7-90s into heavy RTCM traffic with
      // no root cause pinned down yet - if abandons cluster right before a
      // crash, that's evidence commands are getting lost at an increasing
      // rate under sustained load, not a one-off.
      Serial.println(F("Modem: [diag] abandoning a stuck async receive."));
    } else {
      return; // still within the normal grace period, keep waiting
    }
  }

  if (millis() - receiveSuppressStartMs < receiveSuppressDurationMs) {
    return; // holding off for a priority send - see modemSuppressReceiveFor()
  }

  if (!socketRingPending) {
    return;
  }
  socketRingPending = false;

  asyncReceiveInFlight = true;
  asyncReceiveStartMs = millis();
  uint32_t myGeneration = asyncGeneration;
  if (!modem.socketReceive(MODEM_SOCKET_ID, asyncChunk, sizeof(asyncChunk), &asyncRsp,
                            onSocketReceiveComplete, (void *)(uintptr_t)myGeneration)) {
    asyncReceiveInFlight = false; // couldn't even queue the command - try again next call
    // Temporary diagnostic: previously silent - see the abandon log above.
    Serial.println(F("Modem: [diag] socketReceive() couldn't even queue (pool/queue full?)."));
  }
}

static bool isRegistered(WalterModemNetworkRegState state) {
  switch (state) {
    case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
    case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
      return true;
    default:
      return false;
  }
}

// True once modemInit() has completed. Every AT command path checks it:
// WalterModem's command queue doesn't even exist until begin() succeeds,
// and a failed init used to be permanent until the next reboot - nothing
// ever called modemInit() again, and no socket event handler was ever
// registered, so NTRIP could never receive anything.
static bool modemReady = false;

// WalterModem::begin() must only ever run once per boot: it starts its
// tasks on *static* stacks before the steps that can fail (modem reset,
// CME/CEREG config), so calling it again after a failure would start a
// second task on the same stack memory. Everything after begin() in
// modemInit() is safe to retry (begin() returns early once it succeeded).
// A failed begin() is instead recovered with a clean reboot, capped so a
// genuinely dead modem doesn't reboot-loop the station - the geophone and
// SD logging keep running without it. RTC_NOINIT survives a software
// reset but not a power cycle (hence the magic check).
static bool beginAttempted = false;
static bool beginOk = false;
static const uint32_t BEGIN_FAIL_MAGIC = 0x4D4F4445; // "MODE"
static const uint32_t MAX_BEGIN_FAIL_REBOOTS = 3;
RTC_NOINIT_ATTR static uint32_t beginFailMagic;
RTC_NOINIT_ATTR static uint32_t beginFailReboots;

static void rebootForFailedBeginIfAllowed() {
  if (beginFailMagic != BEGIN_FAIL_MAGIC) {
    beginFailMagic = BEGIN_FAIL_MAGIC;
    beginFailReboots = 0;
  }
  if (beginFailReboots >= MAX_BEGIN_FAIL_REBOOTS) {
    Serial.println(F("Modem: begin() keeps failing - running without the modem until power cycle."));
    return;
  }
  beginFailReboots++;
  Serial.printf("Modem: begin() failed - rebooting to retry (%lu/%lu).\n",
                (unsigned long)beginFailReboots, (unsigned long)MAX_BEGIN_FAIL_REBOOTS);
  sdStorageCloseAllForReboot();
  Serial.flush();
  delay(100);
  ESP.restart();
}

bool modemInit() {
  if (modemReady) {
    return true;
  }
  if (beginAttempted && !beginOk) {
    return false; // see beginAttempted - modemConnect() handles recovery
  }
  if (!socketRxMutex) {
    socketRxMutex = xSemaphoreCreateMutex();
  }

  // The Sequans modem chip needs real boot time after power-on before it
  // reliably answers AT commands - the reference WalterModem examples all
  // wait 2s before the first call. Without this, setOpState() calls can
  // silently succeed on a not-yet-ready modem while definePDPContext()
  // (which needs the SIM to have settled) fails right after.
  if (!beginAttempted) {
    geophoneAwareDelay(2000);
    beginAttempted = true;
    if (!modem.begin(&Serial2)) {
      Serial.println(F("Modem: begin() failed."));
      return false;
    }
    beginOk = true;
    beginFailMagic = BEGIN_FAIL_MAGIC;
    beginFailReboots = 0;
  }

  if (!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println(F("Modem: setOpState(NO_RF) failed."));
    return false;
  }

  // AT+CGDCONT? confirmed context 1 is already defined in the modem's own
  // non-volatile memory as type "IP" ("+CGDCONT: 1,\"IP\",\"\",,,,0,...") -
  // this board carries state from prior testing (e.g. open_renewable_lab).
  // Every attempt to redefine it got "+CME ERROR: 4" (operation not
  // supported) - including re-sending the *exact same* "IP" value already
  // stored, which rules out the specific type/value as the cause. This
  // firmware rejects touching an already-defined context at all, not any
  // particular redefinition. Since it's already there and (per the SIM
  // provider) usable as-is, there's nothing to fix by resetting the modem
  // or retrying - just log the query result and move on to registration
  // without treating the (expected) redefinition failure as fatal.
  WalterModemRsp queryRsp;
  modem.sendCmd("AT+CGDCONT?", "OK", &queryRsp);

  char cgdcontCmd[64];
  snprintf(cgdcontCmd, sizeof(cgdcontCmd), "AT+CGDCONT=%d,\"IP\"", MODEM_PDP_CTX);

  WalterModemRsp pdpRsp;
  if (!modem.sendCmd(cgdcontCmd, "OK", &pdpRsp)) {
    Serial.printf("Modem: AT+CGDCONT redefinition rejected (result=%d cmeError=%d) - "
                  "expected if a context is already provisioned; continuing.\n",
                  pdpRsp.result, pdpRsp.data.cmeError);
  }

  // One shared CA credential (Let's Encrypt root) covers both the MQTT
  // broker and the upload backend, assuming both present publicly-trusted
  // certs. Written to MODEM_CA_SLOT (10-19 range - see its definition for
  // why 0-9 doesn't work).
  if (!modem.tlsWriteCredential(false, MODEM_CA_SLOT, ISRG_ROOT_X1_CA)) {
    Serial.println(F("Modem: tlsWriteCredential failed."));
  }
  WalterModemRsp tlsRsp;
  if (!modem.tlsConfigProfile(MODEM_TLS_PROFILE_MQTT, WALTER_MODEM_TLS_VALIDATION_CA,
                               WALTER_MODEM_TLS_VERSION_12, MODEM_CA_SLOT, 0xff, 0xff, &tlsRsp)) {
    Serial.printf("Modem: tlsConfigProfile(MQTT) failed (result=%d).\n", tlsRsp.result);
  }
  if (!modem.tlsConfigProfile(MODEM_TLS_PROFILE_HTTP, WALTER_MODEM_TLS_VALIDATION_CA,
                               WALTER_MODEM_TLS_VERSION_12, MODEM_CA_SLOT, 0xff, 0xff, &tlsRsp)) {
    Serial.printf("Modem: tlsConfigProfile(HTTP) failed (result=%d).\n", tlsRsp.result);
  }

  if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println(F("Modem: setOpState(FULL) failed."));
    return false;
  }

  if (!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println(F("Modem: setNetworkSelectionMode failed."));
    return false;
  }

  modem.setSocketEventHandler(socketEventHandler);

  modemReady = true;
  return true;
}

bool modemConnect() {
  if (!modemReady) {
    if (beginAttempted && !beginOk) {
      rebootForFailedBeginIfAllowed(); // only returns if the cap is reached
      return false;
    }
    // One retry per call - callers invoke this once per measurement cycle.
    Serial.println(F("Modem: not initialized yet - retrying init."));
    if (!modemInit()) {
      Serial.println(F("Modem: init retry failed."));
      return false;
    }
  }

  if (isRegistered(modem.getNetworkRegState())) {
    return true;
  }

  // Coming back from modemSleep() (NO_RF between measurement cycles) lands
  // here too, not just a genuinely fresh boot - radio has to be back on
  // before registration can even begin.
  modem.setOpState(WALTER_MODEM_OPSTATE_FULL);

  Serial.println(F("Modem: waiting for network registration..."));
  unsigned long start = millis();
  while (!isRegistered(modem.getNetworkRegState()) && millis() - start < 60000) {
    geophoneAwareDelay(500);
  }

  bool ok = isRegistered(modem.getNetworkRegState());
  Serial.println(ok ? F("Modem: registered.") : F("Modem: registration timed out."));
  return ok;
}

// Between measurement cycles the modem has nothing to do - no subscriptions,
// no expected downlink, just telemetry it initiates itself - so there's no
// reason to stay registered on the LTE-M network for the ~80% of each
// measurement_interval_s cycle that isn't spent on an active NTRIP/MQTT
// burst. Mirrors gnss.cpp's gnssSleep()/gnssWakeup() pattern (same
// reasoning: don't let an idle peripheral dominate the power budget).
// Must only be called once the async MQTT batch (see the dedicated task
// above) has actually finished, not just been kicked off - see main.cpp's
// modemSleepPending gating.
void modemSleep() {
  if (!modemReady) {
    return;
  }
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println(F("Modem: setOpState(NO_RF) failed (sleep)."));
  }
}

bool modemIsConnected() {
  return modemReady && isRegistered(modem.getNetworkRegState());
}

bool modemSocketOpen(const char *host, uint16_t port) {
  if (!modemReady) {
    return false;
  }
  xSemaphoreTake(socketRxMutex, portMAX_DELAY);
  socketRxLen = 0;
  xSemaphoreGive(socketRxMutex);
  socketRingPending = false;
  // Abandon (don't wait for) any receive still outstanding from a previous
  // socket session - its callback, if it ever fires, must not write into
  // asyncChunk on behalf of this new one.
  asyncReceiveInFlight = false;
  asyncGeneration++;

  if (!modem.socketConfig(MODEM_SOCKET_ID, MODEM_PDP_CTX)) {
    Serial.println(F("Modem: socketConfig failed."));
    return false;
  }
  modem.socketConfigSecure(MODEM_SOCKET_ID, false); // NTRIP is plaintext HTTP-over-TCP

  if (!modem.socketDial(MODEM_SOCKET_ID, WALTER_MODEM_SOCKET_PROTO_TCP, port, host)) {
    Serial.println(F("Modem: socketDial failed."));
    return false;
  }
  return true;
}

// Async for the same reason the receive path is: live hardware crashes
// (_finishModemCMD, _cmdProcessingTask - see drainSocketRing() above)
// consistently landed within a few hundred ms of the periodic GGA resend,
// every time, across every log collected. ntripSendGGA() fires roughly
// every 10s from the main loop - the same cadence a caster's periodic
// station-metadata burst arrives on - so a *blocking* send here could
// submit a new command to WalterModem's queue at the exact moment an
// async receive is already in flight for that burst. Two commands
// converging on the queue that close together, right as the library is
// already under the most pressure, lines up with every crash seen so
// far. Making the send async too - fire-and-forget, no result the caller
// needs synchronously anyway - keeps app-side command submission from
// ever blocking during exactly that window.
// Set here (on WalterModem's own task, when the async result actually
// comes back) and consumed by modemLastSendFailed() (on the main loop
// task). The command is successfully *queued* well before we know whether
// it actually succeeded - a caller that only checks the queue result (as
// ntripSendGGA()/main.cpp did before) has no way to react to a failure
// that shows up later, so a real failure was waiting a full resend
// interval (7s) to retry instead of the ~1.5s this now allows. GGA is
// required for this NTRIP mount (network RTK/VRS - confirmed with the
// user), not just nice-to-have, so a fix quality stuck on autonomous
// despite RTCM flowing traces back to this gap.
static volatile bool lastSendFailed = false;

// sendBuf (below) is handed to WalterModem by pointer and only read when
// the modem's "> " prompt arrives - a second modemSocketSend() before that
// would overwrite the bytes still waiting to go out. Set while a send is
// outstanding; abandoned after SEND_ABANDON_MS in case its callback never
// fires (the same "lost command" failure the receive path guards against).
static volatile bool sendInFlight = false;
static unsigned long sendStartMs = 0;
static const unsigned long SEND_ABANDON_MS = 10000;
// Same generation-tag idea as the receive path: a late callback from an
// abandoned send must not clear sendInFlight for the newer one.
static volatile uint32_t sendGeneration = 0;

static void onSocketSendComplete(const WalterModemRsp *rsp, void *args) {
  if ((uint32_t)(uintptr_t)args == sendGeneration) {
    sendInFlight = false;
  }
  if (rsp->result != WALTER_MODEM_STATE_OK) {
    lastSendFailed = true;
    Serial.println(F("Modem: async send failed."));
  }
}

// WalterModem's socketSend() (WalterSocket.cpp) queues AT+SQNSSENDEXT and
// stores the caller's raw pointer as cmd->payload - it does NOT copy it. The
// actual UART write of that payload only happens later, asynchronously,
// when the "> " data prompt line is parsed (WalterModem.cpp,
// _uart->write(cmd->payload, cmd->payloadSize)), which can be a full
// AT-command round-trip after this function returns. A caller passing a
// stack-local buffer (ntripSendGGA()'s `line`) or a temporary String's
// internal buffer (ntripConnectOnce()'s `request`) has every reason to
// expect the bytes are consumed synchronously, but they're read back out
// later - by which point that memory can already be reused by whatever else
// ran in the gap. Confirmed on live hardware: the bytes actually
// transmitted for a GGA send were garbage ("$G" followed by what looks like
// unrelated float/double data), not the NMEA sentence built moments
// earlier - which fully explains the caster accepting every connection but
// never streaming RTCM, since this VRS mount needs a real GGA to start.
// Copying into storage that outlives the async round-trip fixes it for
// every caller at once, the same way sendRsp below already has to.
static uint8_t sendBuf[512];

bool modemSocketSend(const uint8_t *data, size_t len) {
  if (!modemReady) {
    return false;
  }
  if (len > sizeof(sendBuf)) {
    Serial.println(F("Modem: modemSocketSend() payload exceeds sendBuf - dropped."));
    return false;
  }
  if (sendInFlight) {
    if (millis() - sendStartMs < SEND_ABANDON_MS) {
      return false; // previous payload may still be unsent - caller retries shortly
    }
    // The callback never came. Whatever WalterModem still holds for that
    // command may yet read sendBuf, but waiting longer won't make that
    // safer, and a stuck flag would block every later send.
    Serial.println(F("Modem: [diag] abandoning a stuck async send."));
  }
  memcpy(sendBuf, data, len);

  static WalterModemRsp sendRsp; // must outlive the async call
  sendInFlight = true;
  sendStartMs = millis();
  uint32_t myGeneration = ++sendGeneration;
  if (!modem.socketSend(MODEM_SOCKET_ID, sendBuf, (uint16_t)len, WALTER_MODEM_RAI_NO_INFO,
                        &sendRsp, onSocketSendComplete, (void *)(uintptr_t)myGeneration)) {
    sendInFlight = false;
    return false;
  }
  return true;
}

bool modemLastSendFailed() {
  bool failed = lastSendFailed;
  lastSendFailed = false; // consume - each check reflects only what's new since the last one
  return failed;
}

bool modemReceiveBusy() {
  return asyncReceiveInFlight;
}

size_t modemSocketAvailable() {
  drainSocketRing();
  return socketRxLen; // single size_t read - a momentarily stale value just means "check again next call"
}

size_t modemSocketRead(uint8_t *buf, size_t maxLen) {
  // Don't rely on callers to call modemSocketAvailable() first - ntripPoll()
  // (the main RTCM consumer) calls this directly. Draining here too means
  // the ring gets serviced regardless of which function a caller reaches
  // for first.
  drainSocketRing();

  xSemaphoreTake(socketRxMutex, portMAX_DELAY);
  size_t n = maxLen < socketRxLen ? maxLen : socketRxLen;
  memcpy(buf, socketRxBuf, n);
  if (n < socketRxLen) {
    memmove(socketRxBuf, socketRxBuf + n, socketRxLen - n);
  }
  socketRxLen -= n;
  xSemaphoreGive(socketRxMutex);
  return n;
}

// Async for the same reason send/receive are (see the design notes above):
// live hardware hit a task-watchdog abort landing right in this call, called
// from ntripDisconnect() at the end of every fix cycle - right where the log
// showed "abandoning a stuck async receive" firing twice in a row moments
// earlier, i.e. the command channel was already under the exact contention
// that's previously made a *blocking* AT command get lost with no internal
// timeout ever firing. There's nothing the caller needs synchronously here -
// socketRxLen/socketRingPending/asyncReceiveInFlight are reset immediately
// below regardless of whether the close AT command itself ever completes.
static void onSocketCloseComplete(const WalterModemRsp *rsp, void *args) {
  if (rsp->result != WALTER_MODEM_STATE_OK) {
    Serial.println(F("Modem: [diag] async socketClose failed."));
  }
}

void modemSocketClose() {
  if (!modemReady) {
    return;
  }
  static WalterModemRsp closeRsp; // must outlive the async call
  modem.socketClose(MODEM_SOCKET_ID, &closeRsp, onSocketCloseComplete, nullptr);
  xSemaphoreTake(socketRxMutex, portMAX_DELAY);
  socketRxLen = 0;
  xSemaphoreGive(socketRxMutex);
  socketRingPending = false;
  asyncReceiveInFlight = false;
  asyncGeneration++;
}

// --- MQTT publish (batched telemetry), off a dedicated task ---
//
// This originally chained mqttConfig->mqttConnect->mqttPublish->
// mqttDisconnect via WalterModem's own async callback mechanism, each step
// calling straight into the next from inside the previous step's callback.
// That crashed live hardware: "***ERROR*** A stack overflow in task
// queueProcessing has been detected." Every WalterModem callback runs ON
// the library's own internal queueProcessing task, and chaining four AT
// commands this way stacks all four calls' frames on top of each other
// there instead of unwinding between steps - nothing in the library
// suggests this is a supported usage pattern, and open_renewable_lab (same
// board/library, confirmed working) does the opposite: plain sequential
// *blocking* mqttConfig/mqttConnect/mqttPublish calls (no cb at all) from
// its own caller task.
//
// Matching that here, but off loopTask (the only task esp_task_wdt watches,
// see WATCHDOG_TIMEOUT_S in main.cpp) rather than calling these blocking
// functions directly from it: a dedicated task - mirrors sd_storage.cpp's
// writer task exactly, same reasoning - picks up a pending batch and runs
// the whole sequence as ordinary blocking calls, so nothing stacks and nothing
// can trip the watchdog even if the network stalls. loopTask only stages
// the batch and queues it.
//
// That task is now a general modem worker (see modemRunJob() below): the
// daily HTTP upload used to run its blocking AT sequence straight on
// loopTask, possibly at the same moment this task was mid-MQTT - two tasks
// issuing blocking WalterModem commands at once, which design note (1) at
// the top of this file already established is unsafe - and could block
// loopTask past the watchdog on a slow network. Every blocking AT
// sequence now goes through the one worker, one job at a time.
static volatile bool mqttBatchInFlight = false;
static volatile bool mqttBatchOkPending = false;

struct ModemJob {
  ModemJobFn fn;
  void *ctx;
};
static const UBaseType_t MODEM_JOB_QUEUE_LEN = 4;
static QueueHandle_t modemJobQueue = nullptr;
static TaskHandle_t modemWorkerHandle = nullptr;
// Queued + running. Incremented by modemRunJob() before queueing,
// decremented by the worker only after a job has fully returned, so
// modemJobsBusy() can never read false while AT work is still pending.
static std::atomic<int> modemJobsOutstanding{0};

static void modemWorkerFn(void *) {
  ModemJob job;
  for (;;) {
    if (xQueueReceive(modemJobQueue, &job, portMAX_DELAY) == pdTRUE) {
      job.fn(job.ctx);
      modemJobsOutstanding--;
    }
  }
}

bool modemRunJob(ModemJobFn fn, void *ctx) {
  if (!modemReady) {
    return false;
  }
  if (!modemWorkerHandle) {
    modemJobQueue = xQueueCreate(MODEM_JOB_QUEUE_LEN, sizeof(ModemJob));
    // Core 0 alongside the SD writer and geophone tasks - core 1 already
    // carries loopTask plus WalterModem's own queueProcessing/
    // eventProcessing tasks. Stack sized well above what a single AT
    // command round trip needs (the crash the original MQTT task replaced
    // was exactly a stack-size problem) plus the upload job's SD reads.
    xTaskCreatePinnedToCore(modemWorkerFn, "modemWorker", 8192, NULL, 1, &modemWorkerHandle, 0);
  }
  ModemJob job = {fn, ctx};
  modemJobsOutstanding++;
  if (xQueueSend(modemJobQueue, &job, 0) != pdTRUE) {
    modemJobsOutstanding--;
    Serial.println(F("Modem: job queue full - job dropped."));
    return false;
  }
  return true;
}

bool modemJobsBusy() {
  return modemJobsOutstanding.load() > 0;
}

static char mqttHost[64];
static uint16_t mqttPort;
static char mqttClientId[32];
static char mqttUser[64];
static char mqttPass[64];
static uint8_t mqttTlsProfile;
static char mqttTopic[64];
static uint8_t mqttPayload[2048];
static uint16_t mqttPayloadLen;

// mqttConnect() returning true only means AT+SQNSMQTTCONNECT was accepted -
// the actual broker handshake result arrives later via the
// WALTER_MODEM_MQTT_EVENT_CONNECTED URC. Publishing right after mqttConnect()
// returns raced that: it worked when incidental delay elsewhere happened to
// give the handshake enough time, and failed outright (same "plain ERROR,
// no CME code" shape as every other bug tonight) when it didn't - confirmed
// intermittently on live hardware. open_renewable_lab hits this exact issue
// (its own comment: "fails every time" without waiting) and blocks on a
// semaphore signaled from this URC before publishing; mirrored here.
static SemaphoreHandle_t mqttConnectDoneSem = nullptr;
static volatile bool mqttConnectedOk = false;

static void mqttEventHandler(WMMQTTEventType event, const WMMQTTEventData *data, void *args) {
  if (event == WALTER_MODEM_MQTT_EVENT_CONNECTED) {
    mqttConnectedOk = data && data->rc == 0;
    if (mqttConnectDoneSem) {
      xSemaphoreGive(mqttConnectDoneSem);
    }
  }
}

static void mqttBatchJob(void *) {
  // WalterModem::mqttConfig only emits username/password when *both*
  // pointers are non-NULL - it won't send a lone username. ThingsBoard's
  // convention (and this deployment's secrets.h) is username-as-token
  // with a blank password, so an empty password must still be passed as
  // "" here, not NULL, or the real username silently never reaches the
  // AT command.
  const char *userArg = mqttUser[0] ? mqttUser : NULL;
  const char *passArg = userArg ? mqttPass : NULL;

  bool ok = false;
  if (!modem.mqttConfig(mqttClientId, userArg, passArg, mqttTlsProfile)) {
    Serial.println(F("MQTT: config failed."));
  } else {
    mqttConnectedOk = false;
    xSemaphoreTake(mqttConnectDoneSem, 0); // drain any stale pending signal
    if (!modem.mqttConnect(mqttHost, mqttPort, 60)) {
      Serial.println(F("MQTT: connect failed."));
    } else if (xSemaphoreTake(mqttConnectDoneSem, pdMS_TO_TICKS(10000)) != pdTRUE) {
      Serial.println(F("MQTT: connect timed out waiting for broker confirmation."));
    } else if (!mqttConnectedOk) {
      Serial.println(F("MQTT: broker rejected the connection."));
    } else if (!modem.mqttPublish(mqttTopic, mqttPayload, mqttPayloadLen, 1)) {
      Serial.println(F("MQTT: publish failed."));
    } else {
      ok = true;
      Serial.println(F("MQTT: publish succeeded."));
    }
  }
  // Always, win or lose - leaving a half-open session behind is exactly
  // what open_renewable_lab's be_connect() comment describes as wedging
  // the *next* mqttConfig() call outright.
  modem.mqttDisconnect();

  mqttBatchOkPending = ok;
  mqttBatchInFlight = false; // batch is over either way
}

bool modemMqttPublishBatchAsync(const char *host, uint16_t port, const char *clientId,
                                 const char *user, const char *pass, const char *caCert,
                                 const char *topic, const uint8_t *payload, size_t len) {
  if (mqttBatchInFlight) {
    Serial.println(F("MQTT: previous batch still in flight, skipping this one."));
    return false;
  }
  if (len > sizeof(mqttPayload)) {
    Serial.println(F("MQTT: payload exceeds buffer - dropped."));
    return false;
  }

  if (!modemReady) {
    return false;
  }
  if (!mqttConnectDoneSem) {
    mqttConnectDoneSem = xSemaphoreCreateBinary();
    modem.setMQTTEventHandler(mqttEventHandler, nullptr);
  }

  strncpy(mqttHost, host, sizeof(mqttHost) - 1);
  mqttHost[sizeof(mqttHost) - 1] = '\0';
  mqttPort = port;
  if (clientId && clientId[0]) {
    strncpy(mqttClientId, clientId, sizeof(mqttClientId) - 1);
    mqttClientId[sizeof(mqttClientId) - 1] = '\0';
  } else {
    // AT+SQNSMQTTCFG rejects an empty client-id string outright, and
    // brokers like ThingsBoard don't care what it's set to (auth is by
    // username token) - secrets.h intentionally leaves MQTT_CLIENT_ID
    // blank for that reason, so fall back to a MAC-derived id that's
    // still unique per device instead of requiring it to be filled in.
    uint64_t mac = ESP.getEfuseMac();
    snprintf(mqttClientId, sizeof(mqttClientId), "walter-%04X%08X", (uint16_t)(mac >> 32),
             (uint32_t)mac);
  }
  strncpy(mqttUser, (user && user[0]) ? user : "", sizeof(mqttUser) - 1);
  mqttUser[sizeof(mqttUser) - 1] = '\0';
  strncpy(mqttPass, (pass && pass[0]) ? pass : "", sizeof(mqttPass) - 1);
  mqttPass[sizeof(mqttPass) - 1] = '\0';
  mqttTlsProfile = (caCert != nullptr && caCert[0] != '\0') ? MODEM_TLS_PROFILE_MQTT : 0;
  strncpy(mqttTopic, topic, sizeof(mqttTopic) - 1);
  mqttTopic[sizeof(mqttTopic) - 1] = '\0';
  memcpy(mqttPayload, payload, len);
  mqttPayloadLen = (uint16_t)len;

  mqttBatchInFlight = true;
  if (!modemRunJob(mqttBatchJob, nullptr)) {
    mqttBatchInFlight = false;
    return false;
  }
  return true;
}

bool modemMqttBatchBusy() {
  return mqttBatchInFlight;
}

// Consumes: true only once, right when a batch completes successfully.
bool modemMqttLastBatchOk() {
  bool ok = mqttBatchOkPending;
  mqttBatchOkPending = false;
  return ok;
}

// NOTE: httpSend's buffer parameter is uint16_t - capped at 65535 bytes per
// call. Fine for small payloads (this is used directly for the position
// CSV); upload_client.cpp chunks anything larger into repeated calls.
bool modemHttpPost(const char *host, uint16_t port, const char *path,
                    const char *apiKey, const uint8_t *body, size_t bodyLen) {
  if (!modemReady) {
    return false;
  }
  if (bodyLen > 65535) {
    Serial.println(F("Modem: modemHttpPost body exceeds 65535 bytes - caller must chunk."));
    return false;
  }

  if (!modem.httpConfigProfile(MODEM_HTTP_PROFILE, host, port, MODEM_TLS_PROFILE_HTTP,
                                true, apiKey, "")) {
    Serial.println(F("Modem: httpConfigProfile failed."));
    return false;
  }

  if (!modem.httpSend(MODEM_HTTP_PROFILE, path, (uint8_t *)body, (uint16_t)bodyLen,
                       WALTER_MODEM_HTTP_SEND_CMD_POST)) {
    Serial.println(F("Modem: httpSend failed."));
    return false;
  }
  return true;
}
