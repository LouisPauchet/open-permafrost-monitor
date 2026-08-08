#include "ntrip_client.h"
#include "device_config.h"
#include "gnss.h"
#include "modem.h"
#include <string.h>

static bool connected = false;
static uint32_t bytesForwarded = 0;

static String base64Encode(const String &input) {
  static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String out;
  int len = input.length();

  for (int i = 0; i < len; i += 3) {
    uint32_t octet_a = (uint8_t)input[i];
    uint32_t octet_b = i + 1 < len ? (uint8_t)input[i + 1] : 0;
    uint32_t octet_c = i + 2 < len ? (uint8_t)input[i + 2] : 0;
    uint32_t triple = (octet_a << 16) + (octet_b << 8) + octet_c;

    out += table[(triple >> 18) & 0x3F];
    out += table[(triple >> 12) & 0x3F];
    out += (i + 1 < len) ? table[(triple >> 6) & 0x3F] : '=';
    out += (i + 2 < len) ? table[triple & 0x3F] : '=';
  }
  return out;
}

// Reads one line (up to and including '\n') from the modem socket, blocking
// briefly. Used only during the header-parsing part of the NTRIP handshake -
// the streaming RTCM body afterward goes through ntripPoll() instead.
static String socketReadLine(unsigned long timeoutMs) {
  String line;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (modemSocketAvailable() > 0) {
      uint8_t b;
      modemSocketRead(&b, 1);
      line += (char)b;
      if (b == '\n') {
        return line;
      }
    } else {
      delay(10);
    }
  }
  return line;
}

bool ntripConnect() {
  Serial.printf("Connecting to NTRIP caster %s:%d ...\n", deviceConfig.ntrip_host, deviceConfig.ntrip_port);

  if (!modemSocketOpen(deviceConfig.ntrip_host, deviceConfig.ntrip_port)) {
    Serial.println(F("NTRIP TCP connect failed."));
    return false;
  }

  String auth = base64Encode(String(deviceConfig.ntrip_user) + ":" + String(deviceConfig.ntrip_password));

  String request = String("GET /") + deviceConfig.ntrip_mountpoint + " HTTP/1.0\r\n" +
                    "Host: " + deviceConfig.ntrip_host + "\r\n" +
                    "User-Agent: NTRIP WalterClient/1.0\r\n" +
                    "Authorization: Basic " + auth + "\r\n" +
                    "\r\n";
  modemSocketSend((const uint8_t *)request.c_str(), request.length());

  String statusLine = socketReadLine(5000);
  if (statusLine.length() == 0) {
    Serial.println(F("NTRIP caster did not respond."));
    modemSocketClose();
    return false;
  }

  Serial.print(F("Caster response: "));
  Serial.println(statusLine);

  if (statusLine.indexOf("200") < 0) {
    Serial.println(F("NTRIP caster rejected request (check mountpoint/credentials)."));
    modemSocketClose();
    return false;
  }

  // Drain the rest of the HTTP header block up to the blank line.
  while (true) {
    String headerLine = socketReadLine(2000);
    if (headerLine == "\r\n" || headerLine.length() == 0) {
      break;
    }
  }

  Serial.println(F("NTRIP stream connected, forwarding RTCM to F9P."));
  connected = true;
  return true;
}

bool ntripIsConnected() {
  return connected;
}

void ntripPoll() {
  uint8_t buf[256];
  size_t n;
  while ((n = modemSocketRead(buf, sizeof(buf))) > 0) {
    for (size_t i = 0; i < n; i++) {
      gnssWriteRTCM(buf[i]);
    }
    bytesForwarded += n;
  }
}

void ntripSendGGA(const char *ggaLine) {
  if (!connected || ggaLine[0] == '\0') {
    return;
  }
  String line = String(ggaLine) + "\r\n";
  modemSocketSend((const uint8_t *)line.c_str(), line.length());
}

uint32_t ntripBytesForwarded() {
  return bytesForwarded;
}

void ntripDisconnect() {
  modemSocketClose();
  connected = false;
}
