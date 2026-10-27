#pragma once
#include <Arduino.h>

void initGNSS();
void pollGNSS(); // call frequently; feeds available UART bytes into the parser

// Forwards a chunk of RTCM correction bytes from the NTRIP caster to the
// F9P, and captures the same bytes as the base-station side of the PPK
// pair (see gnss.cpp) - takes a chunk rather than one byte at a time so the
// capture doesn't need its own byte-at-a-time buffering on top of the
// SD_STREAM_RTCM ring buffer it already writes into (see sd_storage.h).
void gnssWriteRTCM(const uint8_t *buf, size_t len);

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
// Unix epoch milliseconds (UTC) - for ThingsBoard's {"ts":...,"values":{...}}
// telemetry format, which needs the actual reading time (not the MQTT
// publish time, which can lag well behind it if a batch was delayed - see
// position_log.cpp). Computed directly from the UTC y/m/d/h/m/s fields
// above via a standalone civil-calendar algorithm rather than mktime(),
// which assumes the C library's configured local timezone unless a caller
// remembers to force UTC first - this way there's no such dependency to
// get wrong.
uint64_t gnssEpochMillis();

// gnssEpochMillis()/gnssYear()/etc. above snapshot TinyGPS++'s last-received
// fields, which never expire on their own (isValid() is sticky, not an age
// check) - they stay frozen at whatever the wall clock was at the *last*
// fix for as long as the F9P goes without reporting, including all through
// backup-mode sleep between measurement cycles. Anything that needs "the
// actual current time" spanning an idle period - not just a snapshot from
// whenever GNSS last talked - must use these instead: they extrapolate via
// millis() off the last genuinely fresh fix, so time keeps advancing
// correctly even while the module is asleep. miniseed_writer.cpp needs this
// for continuous per-record timestamps; using the frozen versions there
// caused every record during an idle window to collide at the same instant
// (confirmed on live hardware once GNSS sleep itself started actually
// working - see gnssSleep()'s own fix).
bool gnssHasEpochAnchor(); // false until the first-ever fix, same as gnssHasDateTime() was used for
uint64_t gnssEpochMillisNow();
void gnssCivilNow(uint16_t *year, uint8_t *month, uint8_t *day, uint8_t *hour, uint8_t *minute,
                   uint8_t *second, uint16_t *milli);
// Same broken-down UTC fields for an arbitrary epoch-ms value (MiniSEED
// record start times, which come from the sample clock rather than "now").
void gnssCivilFromEpochMillis(uint64_t ms, uint16_t *year, uint8_t *month, uint8_t *day,
                              uint8_t *hour, uint8_t *minute, uint8_t *second, uint16_t *milli);

// "YYYYMMDD" date arithmetic (UTC calendar) - e.g. stepping the daily
// upload through days it missed. false if the input isn't a valid date.
bool gnssDateAddDays(const char *yyyymmdd, int days, char *out, size_t outSize);

const char *gnssLastGGA(); // raw $..GGA line as last seen from the module, "" if none yet
uint32_t gnssCharsProcessed();
