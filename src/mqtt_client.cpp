#include "mqtt_client.h"
#include "device_config.h"
#include "isrg_root_x1.h"
#include "modem.h"
#include <string.h>

// Position fields inside each row follow CF (Climate and Forecast)
// convention naming (latitude/longitude/altitude) - see position_log.cpp
// for how each row is built.
bool mqttPublishBatchAsync(const char *jsonArrayPayload, size_t rowCount) {
  Serial.printf("MQTT: publishing %u position(s)...\n", (unsigned)rowCount);
  return modemMqttPublishBatchAsync(deviceConfig.mqtt_host, deviceConfig.mqtt_port,
                                    deviceConfig.mqtt_client_id, deviceConfig.mqtt_user,
                                    deviceConfig.mqtt_password, ISRG_ROOT_X1_CA,
                                    deviceConfig.mqtt_topic, (const uint8_t *)jsonArrayPayload,
                                    strlen(jsonArrayPayload));
}

bool mqttBatchBusy() {
  return modemMqttBatchBusy();
}

bool mqttLastBatchOk() {
  return modemMqttLastBatchOk();
}
