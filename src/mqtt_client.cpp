#include "mqtt_client.h"
#include "device_config.h"
#include "gnss.h"
#include "isrg_root_x1.h"
#include "modem.h"
#include <ArduinoJson.h>
#include <string.h>

static bool connected = false;

bool mqttConnectBroker() {
  Serial.printf("Connecting to MQTT broker %s:%d ...\n", deviceConfig.mqtt_host, deviceConfig.mqtt_port);

  connected = modemMqttConnect(deviceConfig.mqtt_host, deviceConfig.mqtt_port,
                                deviceConfig.mqtt_client_id, deviceConfig.mqtt_user,
                                deviceConfig.mqtt_password, ISRG_ROOT_X1_CA);

  Serial.println(connected ? F("MQTT connected.") : F("MQTT connect failed."));
  return connected;
}

bool mqttIsConnected() {
  return connected;
}

// Field names follow CF (Climate and Forecast) conventions for the
// geospatial coordinates: latitude, longitude, altitude.
void mqttPublishPosition(bool qualityOk) {
  if (!connected || !gnssHasFix()) {
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

  if (modemMqttPublish(deviceConfig.mqtt_topic, (const uint8_t *)payload, len)) {
    Serial.print(F("Published: "));
    Serial.println(payload);
  } else {
    Serial.println(F("MQTT publish failed."));
  }
}

void mqttDisconnectBroker() {
  modemMqttDisconnect();
  connected = false;
}
