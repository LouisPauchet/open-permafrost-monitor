#pragma once
#include <Arduino.h>

bool mqttConnectBroker();
bool mqttIsConnected();
void mqttPublishPosition(bool qualityOk);
void mqttDisconnectBroker();
