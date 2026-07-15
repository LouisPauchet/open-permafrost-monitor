#pragma once
#include <Arduino.h>

void initGNSS();
void pollGNSS(); // call frequently; feeds available UART bytes into the parser
void gnssWriteRTCM(uint8_t b); // forward one RTCM correction byte from the NTRIP caster

// Software sleep/wake via UBX-RXM-PMREQ backup mode + the EXTINT pin, so the
// F9P (100+ mA active) doesn't dominate the power budget between measurement
// cycles the way it would if left powered/active during the ESP32's own deep
// sleep. Call gnssSleep() right before the ESP32 goes to sleep, and
// gnssWakeup() right after it wakes, before talking to the module again.
void gnssSleep();
void gnssWakeup();

bool gnssHasFix();
int gnssFixQuality(); // 0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float
const char *gnssFixQualityLabel(int quality);

bool gnssHasAccuracyEstimate();
float gnssHorizontalAccuracy(); // meters, 1-sigma
float gnssVerticalAccuracy();   // meters, 1-sigma
// Re-sends the GST-enable command; initGNSS() already calls this once, but
// it's exposed so callers can retry if the first attempt lands before the
// F9P (esp. right after an EXTINT wake from backup mode) is ready for it.
void gnssEnableGST();

double gnssLatitude();
double gnssLongitude();
double gnssAltitude();
uint32_t gnssSatellites();

bool gnssHasDateTime();
void gnssTimestamp(char *buf, size_t bufSize); // ISO8601 UTC

const char *gnssLastGGA(); // raw $..GGA line as last seen from the module, "" if none yet
uint32_t gnssCharsProcessed();
