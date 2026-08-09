#include "mqtt_client.h"
#include "device_config.h"
#include "isrg_root_x1.h"
#include "modem.h"

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

// Position fields inside each row follow CF (Climate and Forecast)
// convention naming (latitude/longitude/altitude) - see position_log.cpp
// for how each row is built.
bool mqttPublishBatch(const char *jsonArrayPayload, size_t rowCount) {
  if (!connected) {
    Serial.println(F("Skipping MQTT publish: not connected."));
    return false;
  }

  size_t len = strlen(jsonArrayPayload);
  if (modemMqttPublish(deviceConfig.mqtt_topic, (const uint8_t *)jsonArrayPayload, len)) {
    Serial.printf("Published %u position(s): %s\n", (unsigned)rowCount, jsonArrayPayload);
    return true;
  }

  Serial.println(F("MQTT publish failed."));
  return false;
}

void mqttDisconnectBroker() {
  modemMqttDisconnect();
  connected = false;
}
