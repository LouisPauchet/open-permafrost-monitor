#pragma once
#include <Arduino.h>

void initGNSS();
void pollGNSS(); // call frequently; feeds available UART bytes into the parser

// Commits the current burst's /data/gnss/*.ubx and *.rtcm files' buffered
// writes to the SD card without closing them - call periodically. Both
// files are already opened/closed once per burst (see
// gnssWakeup()/gnssSleep()), so this only matters for a crash/watchdog
// reset mid-burst, not for getting the files to appear at all.
void gnssFlushCaptureFiles();
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

// Temporary diagnostic: re-sends the RXM-RTCM enable command, for the same
// "first attempt may land before the F9P is ready" reason as gnssEnableGST()
// above. Prints "[RTCM-IN] type=... station=... ok/CRC-FAIL" for every
// incoming RTCM message the F9P reports processing - ground truth on
// whether corrections are actually usable, not just "bytes were sent".
void gnssEnableRxmRtcmDiag();

// Explicitly enables RTCM3 as an *input* protocol on UART1 (UBX-CFG-VALSET,
// CFG-UART1INPROT-RTCM3X). Suspected real root cause behind RTK never
// engaging despite RTCM flowing and GGA reaching the caster: if this
// wasn't already enabled, the F9P would discard every RTCM byte before
// ever attempting to parse it.
void gnssEnableRtcm3Input();

double gnssLatitude();
double gnssLongitude();
double gnssAltitude();
uint32_t gnssSatellites();

bool gnssHasDateTime();
void gnssTimestamp(char *buf, size_t bufSize);   // ISO8601 UTC
void gnssDateString(char *buf, size_t bufSize);  // "YYYYMMDD" UTC, for daily filenames
uint16_t gnssYear();
uint8_t gnssMonth();
uint8_t gnssDay();
uint8_t gnssHour();
uint8_t gnssMinute();
uint8_t gnssSecond();
uint16_t gnssMillisecond(); // sub-second, from TinyGPS++'s centisecond field

const char *gnssLastGGA(); // raw $..GGA line as last seen from the module, "" if none yet
uint32_t gnssCharsProcessed();
