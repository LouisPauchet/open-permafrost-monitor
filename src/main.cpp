#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include "secrets.h"
#include "isrg_root_x1.h"

// The default 8KB Arduino loop-task stack is too small for a TLS handshake
// (WiFiClientSecure/mbedTLS) and crashes with a stack overflow at connect time.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// ZED-F9P (gnss.store ELT0128) wired to ESP32 UART2.
// SEL jumper left unsoldered -> module's UART1 is the active serial interface.
//   F9P UART1 TXO -> ESP32 GPIO16 (RX2)
//   F9P UART1 RXI -> ESP32 GPIO17 (TX2)
//   F9P VCC       -> 3.3V
//   F9P GND       -> GND
//
// ZED-F9P ships from factory at 38400 baud on UART1. If nothing shows up,
// try 9600 or 115200 in GPS_BAUD below (some boards get pre-configured).

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 38400

// Some NTRIP casters (especially VRS/network-RTK ones) need the rover's
// approximate position to pick or synthesize the right correction stream.
// Fixed single-base mountpoints usually ignore it. Leave true unless your
// caster provider tells you otherwise.
#define SEND_GGA_TO_CASTER true
const unsigned long GGA_SEND_INTERVAL_MS = 10000;

const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;
const unsigned long NTRIP_RETRY_INTERVAL_MS = 5000;
const unsigned long MQTT_RETRY_INTERVAL_MS = 5000;

// How often to publish a position (with a fix) to the MQTT broker.
const unsigned long MQTT_PUBLISH_INTERVAL_MS = 5000;

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;
WiFiClient ntripClient;
WiFiClientSecure mqttNetClient;
PubSubClient mqttClient(mqttNetClient);

// GST (GNSS Pseudorange error statistics) carries the receiver's own 1-sigma
// position error estimate in meters - a real accuracy figure, not just HDOP.
// It's off by default on the F9P, so we enable it with a UBX-CFG-MSG command
// in setup(). Field numbers are 1-based, counted after the talker+sentence ID:
// $GNGST,time,rangeRms,stdMajor,stdMinor,orient,stdLat,stdLon,stdAlt*cs
TinyGPSCustom gstStdLat(gps, "GNGST", 6);
TinyGPSCustom gstStdLon(gps, "GNGST", 7);
TinyGPSCustom gstStdAlt(gps, "GNGST", 8);

// GGA fix quality: 0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float.
// $GNGGA,time,lat,NS,lon,EW,quality,numSV,hdop,alt,M,geoidSep,M,diffAge,diffStation*cs
TinyGPSCustom ggaFixQuality(gps, "GNGGA", 6);

// UBX-CFG-MSG: enable NMEA GST (class 0xF0, id 0x07) on UART1 only, rate 1.
// This setting lives in RAM only, so it's resent every boot - nothing to undo.
const uint8_t UBX_ENABLE_GST_UART1[] = {
  0xB5, 0x62,             // sync chars
  0x06, 0x01,             // class/id: CFG-MSG
  0x08, 0x00,             // payload length = 8
  0xF0, 0x07,             // msgClass=NMEA, msgID=GST
  0x00, 0x01, 0x00, 0x00, 0x00, 0x00, // rates: DDC, UART1, UART2, USB, SPI, reserved
  0x07, 0x59              // checksum (CK_A, CK_B)
};

unsigned long lastSummaryMs = 0;
const unsigned long SUMMARY_INTERVAL_MS = 1000;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastNtripAttemptMs = 0;
unsigned long lastGgaSentMs = 0;
unsigned long lastMqttAttemptMs = 0;
unsigned long lastMqttPublishMs = 0;
unsigned long rtcmBytesForwarded = 0;

// Raw GGA line last seen from the F9P itself, forwarded to the caster as-is
// (correct format/precision/checksum guaranteed since the module produced it).
char lastGGA[100] = "";
bool haveGGA = false;
char lineBuf[100];
uint8_t lineLen = 0;

const char *fixQualityLabel(int quality) {
  switch (quality) {
    case 0: return "no fix";
    case 1: return "autonomous";
    case 2: return "DGPS";
    case 3: return "PPS";
    case 4: return "RTK FIXED";
    case 5: return "RTK FLOAT";
    case 6: return "estimated/dead-reckoning";
    default: return "unknown";
  }
}

String base64Encode(const String &input) {
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

void connectWiFi() {
  Serial.printf("Connecting to WiFi \"%s\"...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  // ESP32 WiFi power-save (modem sleep) periodically drops the association
  // (typically every ~30-60s) to save power - disable it for a stable link.
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("WiFi connected, IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi connect failed, will retry."));
  }
}

bool connectNtrip() {
  Serial.printf("Connecting to NTRIP caster %s:%d ...\n", NTRIP_HOST, NTRIP_PORT);

  if (!ntripClient.connect(NTRIP_HOST, NTRIP_PORT)) {
    Serial.println(F("NTRIP TCP connect failed."));
    return false;
  }

  String auth = base64Encode(String(NTRIP_USER) + ":" + String(NTRIP_PASSWORD));

  ntripClient.print(String("GET /") + NTRIP_MOUNTPOINT + " HTTP/1.0\r\n");
  ntripClient.print(String("Host: ") + NTRIP_HOST + "\r\n");
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

void sendGgaToCaster() {
  if (!haveGGA || !ntripClient.connected()) {
    return;
  }
  ntripClient.print(lastGGA);
  ntripClient.print("\r\n");
}

bool connectMqtt() {
  Serial.printf("Connecting to MQTT broker %s:%d ...\n", MQTT_HOST, MQTT_PORT);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  bool ok;
  if (strlen(MQTT_USER) > 0) {
    ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
  } else {
    ok = mqttClient.connect(MQTT_CLIENT_ID);
  }

  if (!ok) {
    Serial.printf("MQTT connect failed, state=%d\n", mqttClient.state());
    char tlsErr[128];
    mqttNetClient.lastError(tlsErr, sizeof(tlsErr));
    if (tlsErr[0] != '\0') {
      Serial.print(F("TLS error: "));
      Serial.println(tlsErr);
    }
    return false;
  }

  Serial.println(F("MQTT connected."));
  return true;
}

void publishFix() {
  if (!mqttClient.connected() || !gps.location.isValid()) {
    return;
  }

  JsonDocument doc;

  if (gps.date.isValid() && gps.time.isValid()) {
    char timestamp[25];
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    doc["timestamp"] = timestamp;
  }

  if (ggaFixQuality.isValid()) {
    doc["fix_quality"] = atoi(ggaFixQuality.value());
  }
  doc["satellites"] = gps.satellites.isValid() ? gps.satellites.value() : 0;
  doc["lat"] = gps.location.lat();
  doc["lon"] = gps.location.lng();
  if (gps.altitude.isValid()) {
    doc["alt_m"] = gps.altitude.meters();
  }

  if (gstStdLat.isValid()) {
    float latErr = atof(gstStdLat.value());
    float lonErr = atof(gstStdLon.value());
    float altErr = atof(gstStdAlt.value());
    doc["horizontal_error_m"] = sqrt(latErr * latErr + lonErr * lonErr);
    doc["vertical_error_m"] = altErr;
  }

  char payload[512];
  size_t len = serializeJson(doc, payload);

  if (mqttClient.publish(MQTT_TOPIC, (const uint8_t *)payload, len)) {
    Serial.println(F("Published position to MQTT."));
  } else {
    Serial.println(F("MQTT publish failed."));
  }
}

void printFixSummary() {
  Serial.println(F("---- Fix summary ----"));

  Serial.print(F("Fix: "));
  Serial.println(gps.location.isValid() ? F("YES") : F("no"));

  if (ggaFixQuality.isValid()) {
    int quality = atoi(ggaFixQuality.value());
    Serial.printf("Fix quality: %d (%s)\n", quality, fixQualityLabel(quality));
  }

  Serial.print(F("Satellites: "));
  Serial.println(gps.satellites.isValid() ? gps.satellites.value() : 0);

  Serial.print(F("HDOP: "));
  Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : -1.0, 2);

  if (gps.location.isValid()) {
    Serial.print(F("Lat: "));
    Serial.println(gps.location.lat(), 8);
    Serial.print(F("Lng: "));
    Serial.println(gps.location.lng(), 8);
  }

  if (gps.altitude.isValid()) {
    Serial.print(F("Alt (m): "));
    Serial.println(gps.altitude.meters(), 2);
  }

  if (gstStdLat.isUpdated() || gstStdLat.isValid()) {
    float latErr = atof(gstStdLat.value());
    float lonErr = atof(gstStdLon.value());
    float altErr = atof(gstStdAlt.value());
    float horizErr = sqrt(latErr * latErr + lonErr * lonErr);

    Serial.print(F("Position error - horizontal (m): "));
    Serial.println(horizErr, 3);
    Serial.print(F("Position error - alt (m): "));
    Serial.println(altErr, 3);
  } else {
    Serial.println(F("Position error: waiting for GST sentence..."));
  }

  if (gps.time.isValid()) {
    Serial.printf("UTC time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  Serial.print(F("WiFi: "));
  Serial.println(WiFi.status() == WL_CONNECTED ? F("connected") : F("disconnected"));
  Serial.print(F("NTRIP: "));
  Serial.println(ntripClient.connected() ? F("connected") : F("disconnected"));
  Serial.print(F("RTCM bytes forwarded: "));
  Serial.println(rtcmBytesForwarded);
  Serial.print(F("MQTT: "));
  Serial.println(mqttClient.connected() ? F("connected") : F("disconnected"));

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for USB serial on boards that need it
  }
  delay(200);

  Serial.println(F("ESP32 <-> ZED-F9P RTK test"));
  Serial.printf("Opening GPS UART2 at %d baud (RX=%d, TX=%d)\n", GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);

  GPSSerial.setRxBufferSize(1024);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  delay(100);
  Serial.println(F("Enabling GST (position error) sentence on the F9P..."));
  GPSSerial.write(UBX_ENABLE_GST_UART1, sizeof(UBX_ENABLE_GST_UART1));

  mqttNetClient.setCACert(ISRG_ROOT_X1_CA);

  connectWiFi();
}

void loop() {
  // GPS -> parser, plus capture the raw GGA line to relay to the caster.
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen >= 6 && strncmp(lineBuf + 3, "GGA", 3) == 0) {
        strncpy(lastGGA, lineBuf, sizeof(lastGGA) - 1);
        lastGGA[sizeof(lastGGA) - 1] = '\0';
        haveGGA = true;
      }
      lineLen = 0;
    } else if (c != '\r' && lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  // Caster -> F9P: forward RTCM3 correction bytes straight into UART1.
  while (ntripClient.available() > 0) {
    uint8_t rtcmByte = ntripClient.read();
    GPSSerial.write(rtcmByte);
    rtcmBytesForwarded++;
  }

  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED && now - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    lastWifiAttemptMs = now;
    connectWiFi();
  }

  if (WiFi.status() == WL_CONNECTED && !ntripClient.connected() &&
      now - lastNtripAttemptMs >= NTRIP_RETRY_INTERVAL_MS) {
    lastNtripAttemptMs = now;
    connectNtrip();
  }

#if SEND_GGA_TO_CASTER
  if (now - lastGgaSentMs >= GGA_SEND_INTERVAL_MS) {
    lastGgaSentMs = now;
    sendGgaToCaster();
  }
#endif

  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected() &&
      now - lastMqttAttemptMs >= MQTT_RETRY_INTERVAL_MS) {
    lastMqttAttemptMs = now;
    connectMqtt();
  }
  mqttClient.loop();

  if (now - lastMqttPublishMs >= MQTT_PUBLISH_INTERVAL_MS) {
    lastMqttPublishMs = now;
    publishFix();
  }

  if (now - lastSummaryMs >= SUMMARY_INTERVAL_MS) {
    lastSummaryMs = now;
    printFixSummary();

    if (now > 15000 && gps.charsProcessed() < 10) {
      Serial.println(F("WARNING: no GPS data received - check wiring/baud rate (try 9600 or 115200)."));
    }
  }
}
