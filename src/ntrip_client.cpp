#include "ntrip_client.h"
#include "device_config.h"
#include "gnss.h"
#include <WiFiClient.h>

static WiFiClient ntripClient;
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

bool ntripConnect() {
  Serial.printf("Connecting to NTRIP caster %s:%d ...\n", deviceConfig.ntrip_host, deviceConfig.ntrip_port);

  if (!ntripClient.connect(deviceConfig.ntrip_host, deviceConfig.ntrip_port)) {
    Serial.println(F("NTRIP TCP connect failed."));
    return false;
  }

  String auth = base64Encode(String(deviceConfig.ntrip_user) + ":" + String(deviceConfig.ntrip_password));

  ntripClient.print(String("GET /") + deviceConfig.ntrip_mountpoint + " HTTP/1.0\r\n");
  ntripClient.print(String("Host: ") + deviceConfig.ntrip_host + "\r\n");
  ntripClient.print("User-Agent: NTRIP ESP32Client/1.0\r\n");
  ntripClient.print(String("Authorization: Basic ") + auth + "\r\n");
  ntripClient.print("\r\n");

  unsigned long start = millis();
  while (!ntripClient.available() && millis() - start < 5000) {
    delay(50);
  }

  if (!ntripClient.available()) {
    Serial.println(F("NTRIP caster did not respond."));
    ntripClient.stop();
    return false;
  }

  String statusLine = ntripClient.readStringUntil('\n');
  Serial.print(F("Caster response: "));
  Serial.println(statusLine);

  if (statusLine.indexOf("200") < 0) {
    Serial.println(F("NTRIP caster rejected request (check mountpoint/credentials)."));
    ntripClient.stop();
    return false;
  }

  // Drain the rest of the HTTP header block up to the blank line.
  while (ntripClient.connected()) {
    String headerLine = ntripClient.readStringUntil('\n');
    if (headerLine == "\r" || headerLine.length() == 0) {
      break;
    }
  }

  Serial.println(F("NTRIP stream connected, forwarding RTCM to F9P."));
  return true;
}

bool ntripIsConnected() {
  return ntripClient.connected();
}

void ntripPoll() {
  while (ntripClient.available() > 0) {
    gnssWriteRTCM((uint8_t)ntripClient.read());
    bytesForwarded++;
  }
}

void ntripSendGGA(const char *ggaLine) {
  if (!ntripClient.connected() || ggaLine[0] == '\0') {
    return;
  }
  ntripClient.print(ggaLine);
  ntripClient.print("\r\n");
}

uint32_t ntripBytesForwarded() {
  return bytesForwarded;
}

void ntripDisconnect() {
  ntripClient.stop();
}
