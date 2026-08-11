#pragma once
#include <Arduino.h>

// Daily-rotated CSV of GNSS fixes on SD. Also doubles as the MQTT batching
// buffer (see mqtt_client.h) - telemetry publishes read back whatever rows
// were appended since the last publish, rather than keeping a separate
// in-RAM structure.

void initPositionLog();

// Appends one row for the current GNSS fix. Call once per measurement
// cycle. hasDisplacement is false (and northM/eastM/upM ignored) until
// reference_position.h has a locked reference to measure from.
void positionLogAppend(bool qualityOk, float batteryExternalV, float batteryInternalV,
                       bool hasDisplacement, float northM, float eastM, float upM);

// Reads back rows appended since the last call to positionLogMarkSent(),
// formatting them as a JSON array into buf. Returns the number of rows
// included (0 if none, in which case buf is untouched).
size_t positionLogReadUnsent(char *buf, size_t bufSize);
void positionLogMarkSent();
