#pragma once
#include <Arduino.h>

bool mqttConnectBroker();
bool mqttIsConnected();

// Publishes a pre-built JSON array of position_log rows (see
// position_log.h) as one MQTT message, so the modem only needs one
// connect/publish/disconnect cycle per batch rather than one per GNSS fix.
bool mqttPublishBatch(const char *jsonArrayPayload, size_t rowCount);

void mqttDisconnectBroker();
