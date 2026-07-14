#pragma once
#include <Arduino.h>

void initGNSS();
void pollGNSS(); // call frequently; feeds available UART bytes into the parser
void gnssWriteRTCM(uint8_t b); // forward one RTCM correction byte from the NTRIP caster

bool gnssHasFix();
int gnssFixQuality(); // 0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float
const char *gnssFixQualityLabel(int quality);

bool gnssHasAccuracyEstimate();
float gnssHorizontalAccuracy(); // meters, 1-sigma
float gnssVerticalAccuracy();   // meters, 1-sigma

double gnssLatitude();
double gnssLongitude();
double gnssAltitude();
uint32_t gnssSatellites();

bool gnssHasDateTime();
void gnssTimestamp(char *buf, size_t bufSize); // ISO8601 UTC

const char *gnssLastGGA(); // raw $..GGA line as last seen from the module, "" if none yet
uint32_t gnssCharsProcessed();
