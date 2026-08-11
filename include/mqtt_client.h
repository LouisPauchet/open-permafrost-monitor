#pragma once
#include <Arduino.h>

// Kicks off an async publish of a pre-built JSON array of position_log rows
// as one MQTT message - returns immediately (true only means the
// config->connect->publish->disconnect chain was successfully kicked off,
// not that the publish itself succeeded). Check mqttBatchBusy()/
// mqttLastBatchOk() on a later loop() iteration to find out the actual
// result, since the whole chain now runs asynchronously (see modem.cpp)
// instead of blocking here the way it used to.
bool mqttPublishBatchAsync(const char *jsonArrayPayload, size_t rowCount);

bool mqttBatchBusy();
bool mqttLastBatchOk(); // consumes: true only once, right when a batch succeeds
