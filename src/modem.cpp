#include "modem.h"
#include "isrg_root_x1.h"
#include "geophone.h"
#include <WalterModem.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>

static WalterModem modem;

static const int MODEM_PDP_CTX = 1;
static const int MODEM_SOCKET_ID = 1;
static const uint8_t MODEM_CA_SLOT = 0; // shared Let's Encrypt root, both TLS profiles below
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
static unsigned long receiveSuppressedUntilMs = 0;

void modemSuppressReceiveFor(unsigned long ms) {
  receiveSuppressedUntilMs = millis() + ms;
}

// Called from the main loop only (modemSocketAvailable()/Read()) - kicks
// off at most one async receive per call, never blocks.
static void drainSocketRing() {
  if (asyncReceiveInFlight) {
    if (millis() - asyncReceiveStartMs > ASYNC_RECEIVE_ABANDON_MS) {
      asyncReceiveInFlight = false;
      asyncGeneration++;
    } else {
      return; // still within the normal grace period, keep waiting
    }
  }

  if (millis() < receiveSuppressedUntilMs) {
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

bool modemInit() {
  socketRxMutex = xSemaphoreCreateMutex();

  // The Sequans modem chip needs real boot time after power-on before it
  // reliably answers AT commands - the reference WalterModem examples all
  // wait 2s before the first call. Without this, setOpState() calls can
  // silently succeed on a not-yet-ready modem while definePDPContext()
  // (which needs the SIM to have settled) fails right after.
  geophoneAwareDelay(2000);

  if (!modem.begin(&Serial2)) {
    Serial.println(F("Modem: begin() failed."));
    return false;
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

  // One shared CA credential (Let's Encrypt root, reused from the MQTT
  // work) covers both the MQTT broker and the upload backend, assuming
  // both present publicly-trusted certs.
  modem.tlsWriteCredential(false, MODEM_CA_SLOT, ISRG_ROOT_X1_CA);
  modem.tlsConfigProfile(MODEM_TLS_PROFILE_MQTT, WALTER_MODEM_TLS_VALIDATION_CA,
                          WALTER_MODEM_TLS_VERSION_12, MODEM_CA_SLOT);
  modem.tlsConfigProfile(MODEM_TLS_PROFILE_HTTP, WALTER_MODEM_TLS_VALIDATION_CA,
                          WALTER_MODEM_TLS_VERSION_12, MODEM_CA_SLOT);

  if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println(F("Modem: setOpState(FULL) failed."));
    return false;
  }

  if (!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println(F("Modem: setNetworkSelectionMode failed."));
    return false;
  }

  modem.setSocketEventHandler(socketEventHandler);

  return true;
}

bool modemConnect() {
  if (isRegistered(modem.getNetworkRegState())) {
    return true;
  }

  Serial.println(F("Modem: waiting for network registration..."));
  unsigned long start = millis();
  while (!isRegistered(modem.getNetworkRegState()) && millis() - start < 60000) {
    geophoneAwareDelay(500);
  }

  bool ok = isRegistered(modem.getNetworkRegState());
  Serial.println(ok ? F("Modem: registered.") : F("Modem: registration timed out."));
  return ok;
}

bool modemIsConnected() {
  return isRegistered(modem.getNetworkRegState());
}

bool modemSocketOpen(const char *host, uint16_t port) {
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

static void onSocketSendComplete(const WalterModemRsp *rsp, void *args) {
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
  if (len > sizeof(sendBuf)) {
    Serial.println(F("Modem: modemSocketSend() payload exceeds sendBuf - dropped."));
    return false;
  }
  memcpy(sendBuf, data, len);

  static WalterModemRsp sendRsp; // must outlive the async call
  return modem.socketSend(MODEM_SOCKET_ID, sendBuf, (uint16_t)len, WALTER_MODEM_RAI_NO_INFO,
                           &sendRsp, onSocketSendComplete, nullptr);
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

void modemSocketClose() {
  modem.socketClose(MODEM_SOCKET_ID);
  xSemaphoreTake(socketRxMutex, portMAX_DELAY);
  socketRxLen = 0;
  xSemaphoreGive(socketRxMutex);
  socketRingPending = false;
  asyncReceiveInFlight = false;
  asyncGeneration++;
}

bool modemMqttConnect(const char *host, uint16_t port, const char *clientId,
                       const char *user, const char *pass, const char *caCert) {
  uint8_t tlsProfile = (caCert != nullptr && caCert[0] != '\0') ? MODEM_TLS_PROFILE_MQTT : 0;

  if (!modem.mqttConfig(clientId, (user && user[0]) ? user : NULL,
                         (pass && pass[0]) ? pass : NULL, tlsProfile)) {
    Serial.println(F("Modem: mqttConfig failed."));
    return false;
  }

  if (!modem.mqttConnect(host, port)) {
    Serial.println(F("Modem: mqttConnect failed."));
    return false;
  }
  return true;
}

bool modemMqttPublish(const char *topic, const uint8_t *payload, size_t len) {
  return modem.mqttPublish(topic, (uint8_t *)payload, (uint16_t)len);
}

void modemMqttDisconnect() {
  modem.mqttDisconnect();
}

// NOTE: httpSend's buffer parameter is uint16_t - capped at 65535 bytes per
// call. Fine for small payloads (this is used directly for the position
// CSV); upload_client.cpp chunks anything larger into repeated calls.
bool modemHttpPost(const char *host, uint16_t port, const char *path,
                    const char *apiKey, const uint8_t *body, size_t bodyLen) {
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
