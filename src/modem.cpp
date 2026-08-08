#include "modem.h"
#include "isrg_root_x1.h"
#include <WalterModem.h>
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

static void socketEventHandler(WMSocketEventType event, const WMSocketEventData *data, void *args) {
  if (event != WALTER_MODEM_SOCKET_EVENT_RING) {
    return;
  }
  if (socketRxLen + data->data_len > sizeof(socketRxBuf)) {
    return; // drop if the consumer isn't keeping up; avoids overflow
  }
  uint8_t chunk[1500];
  uint16_t toRead = data->data_len > sizeof(chunk) ? sizeof(chunk) : data->data_len;
  if (modem.socketReceive(MODEM_SOCKET_ID, chunk, toRead)) {
    memcpy(socketRxBuf + socketRxLen, chunk, toRead);
    socketRxLen += toRead;
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
  if (!modem.begin(&Serial2)) {
    Serial.println(F("Modem: begin() failed."));
    return false;
  }

  modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM);

  if (!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println(F("Modem: setOpState(NO_RF) failed."));
    return false;
  }

  if (!modem.definePDPContext(MODEM_PDP_CTX)) {
    Serial.println(F("Modem: definePDPContext failed."));
    return false;
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
    delay(500);
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
  return socketRxLen;
}

size_t modemSocketRead(uint8_t *buf, size_t maxLen) {
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
