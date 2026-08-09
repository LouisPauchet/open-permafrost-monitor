#include "modem.h"
#include "isrg_root_x1.h"
#include "geophone.h"
#include <WalterModem.h>
#include <esp_task_wdt.h>
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
// Not thread-safe against true concurrent access; fine here since nothing
// else touches it between loop() iterations.
static uint8_t socketRxBuf[4096];
static size_t socketRxLen = 0;

// Set by the RING event, cleared by drainSocketRing() - just a flag, no AT
// command issued here. WalterModem's command pipeline (_cmdProcessingTask /
// _curCmd) isn't safe against two tasks submitting blocking commands at
// once: this handler runs on the library's own _eventProcessingTask, while
// the main loop concurrently issues its own blocking commands (e.g. GGA
// sends). Calling modem.socketReceive() - itself a blocking AT command -
// directly from here raced against that and crashed the library
// (_finishModemCMD dereferencing a stale/freed command pointer after a
// stray response landed with no matching command outstanding). Deferring
// the actual receive to drainSocketRing(), only ever called from
// modemSocketAvailable() on the main Arduino task, keeps every blocking AT
// command on one task.
static volatile bool socketRingPending = false;

static void socketEventHandler(WMSocketEventType event, const WMSocketEventData *data, void *args) {
  if (event == WALTER_MODEM_SOCKET_EVENT_RING) {
    socketRingPending = true;
  }
}

// data_len on the RING event isn't reliable here: this firmware's
// +SQNSRING URC carries only the socket id, no length (confirmed via AT
// log - plain "+SQNSRING: 1", not "+SQNSRING: 1,<n>"), so it comes through
// as 0 - requesting 0 bytes ("AT+SQNSRECV=1,0") gets "+CME ERROR: 50"
// (incorrect parameters). Always ask for the max instead, and use the
// response's own bytesReceived to know how much of the buffer is actually
// valid.
//
// A previous version of this function added a chunk-size cut and a 50ms
// pacing gap between drains, guessing that hammering the AT channel less
// hard would help. Live hardware showed the opposite: after a ~10s network
// lull, several +SQNSRING notifications arrived within milliseconds of
// each other, and WalterModem started logging "No free buffers"
// (WalterModem.h: WALTER_MODEM_BUFFER_POOL_SIZE = 8, a fixed pool - every
// queued line, including plain URCs, holds one slot until
// _cmdProcessingTask processes it). Draining slowly just let that backlog
// build past 8 before we could catch up. This drains everything pending in
// one go instead - request the real max (1500) and keep going as long as
// a read comes back full (strong signal more is still buffered), so a
// burst gets consumed as fast as the AT channel allows rather than
// trickling out one small read per outer-loop pass. Still bounded (maxLoops)
// and still polls the geophone/watchdog between reads, since a large catch
// -up can span several real round-trips.
static void drainSocketRing() {
  const int maxLoops = 16; // safety cap, not expected to be hit in practice
  for (int i = 0; socketRingPending && i < maxLoops; i++) {
    socketRingPending = false;

    uint8_t chunk[1500];
    WalterModemRsp rsp;
    if (!modem.socketReceive(MODEM_SOCKET_ID, chunk, sizeof(chunk), &rsp)) {
      return;
    }

    uint16_t n = rsp.data.socketResponse.bytesReceived;
    if (n == 0) {
      return; // nothing received
    }
    if (socketRxLen + n > sizeof(socketRxBuf)) {
      return; // drop if the consumer isn't keeping up
    }

    memcpy(socketRxBuf + socketRxLen, chunk, n);
    socketRxLen += n;

    pollGeophone();
    esp_task_wdt_reset();

    if (n < sizeof(chunk)) {
      return; // got less than the max - modem's buffer is empty for now
    }
    // Got a full chunk back - more is very likely still queued at the
    // modem, loop again immediately even if socketRingPending hasn't been
    // re-set yet by a fresh RING.
    socketRingPending = true;
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
  socketRxLen = 0;
  socketRingPending = false;

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

bool modemSocketSend(const uint8_t *data, size_t len) {
  return modem.socketSend(MODEM_SOCKET_ID, (uint8_t *)data, (uint16_t)len);
}

size_t modemSocketAvailable() {
  drainSocketRing();
  return socketRxLen;
}

size_t modemSocketRead(uint8_t *buf, size_t maxLen) {
  // Don't rely on callers to call modemSocketAvailable() first - ntripPoll()
  // (the main RTCM consumer) calls this directly. Draining here too means
  // the ring gets serviced regardless of which function a caller reaches
  // for first.
  drainSocketRing();

  size_t n = maxLen < socketRxLen ? maxLen : socketRxLen;
  memcpy(buf, socketRxBuf, n);
  if (n < socketRxLen) {
    memmove(socketRxBuf, socketRxBuf + n, socketRxLen - n);
  }
  socketRxLen -= n;
  return n;
}

void modemSocketClose() {
  modem.socketClose(MODEM_SOCKET_ID);
  socketRxLen = 0;
  socketRingPending = false;
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
