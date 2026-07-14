#include "mqtt_client.h"
#include "device_config.h"
#include "gnss.h"
#include "isrg_root_x1.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>

static WiFiClientSecure netClient;
static PubSubClient client(netClient);
static bool caLoaded = false;

bool mqttConnectBroker() {
  if (!caLoaded) {
    netClient.setCACert(ISRG_ROOT_X1_CA);
    caLoaded = true;
  }

  client.setServer(deviceConfig.mqtt_host, deviceConfig.mqtt_port);

  Serial.printf("Connecting to MQTT broker %s:%d ...\n", deviceConfig.mqtt_host, deviceConfig.mqtt_port);

  bool ok;
  if (strlen(deviceConfig.mqtt_user) > 0) {
    ok = client.connect(deviceConfig.mqtt_client_id, deviceConfig.mqtt_user, deviceConfig.mqtt_password);
  } else {
    ok = client.connect(deviceConfig.mqtt_client_id);
  }

  if (!ok) {
    Serial.printf("MQTT connect failed, state=%d\n", client.state());
    char tlsErr[128];
    netClient.lastError(tlsErr, sizeof(tlsErr));
    if (tlsErr[0] != '\0') {
      Serial.print(F("TLS error: "));
      Serial.println(tlsErr);
    }
    return false;
  }

  Serial.println(F("MQTT connected."));
  return true;
}

bool mqttIsConnected() {
  return client.connected();
}

// Field names follow CF (Climate and Forecast) conventions for the
// geospatial coordinates: latitude, longitude, altitude.
void mqttPublishPosition(bool qualityOk) {
  if (!client.connected() || !gnssHasFix()) {
    Serial.println(F("Skipping MQTT publish: no MQTT connection or no GNSS fix."));
    return;
  }

  JsonDocument doc;

  if (gnssHasDateTime()) {
    char ts[25];
    gnssTimestamp(ts, sizeof(ts));
    doc["time"] = ts;
  }

  doc["latitude"] = gnssLatitude();
  doc["longitude"] = gnssLongitude();
  doc["altitude"] = gnssAltitude();
  doc["fix_quality"] = gnssFixQuality();
  doc["satellites"] = gnssSatellites();

  if (gnssHasAccuracyEstimate()) {
    doc["horizontal_accuracy"] = gnssHorizontalAccuracy();
    doc["vertical_accuracy"] = gnssVerticalAccuracy();
  }

  doc["quality_ok"] = qualityOk;

  char payload[512];
  size_t len = serializeJson(doc, payload);

  if (client.publish(deviceConfig.mqtt_topic, (const uint8_t *)payload, len)) {
    Serial.print(F("Published: "));
    Serial.println(payload);
  } else {
    Serial.println(F("MQTT publish failed."));
  }
}

void mqttDisconnectBroker() {
  client.disconnect();
}
