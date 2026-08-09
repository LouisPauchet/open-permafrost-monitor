#include "gnss.h"
#include "pins.h"
#include "sd_storage.h"
#include <TinyGPSPlus.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Mirror every raw byte from the F9P to the USB serial monitor - useful for
// debugging (e.g. confirming GST/NMEA output timing around EXTINT wake).
// Flip to false to go back to summary-only output.
static const bool DEBUG_PRINT_RAW_GNSS = true;

static HardwareSerial GPSSerial(2);
static TinyGPSPlus gps;

// GST (GNSS Pseudorange error statistics) carries the receiver's own 1-sigma
// position error estimate in meters - a real accuracy figure, not just HDOP.
// It's off by default on the F9P, enabled below with a UBX-CFG-MSG command.
// Field numbers are 1-based, counted after the talker+sentence ID:
// $GNGST,time,rangeRms,stdMajor,stdMinor,orient,stdLat,stdLon,stdAlt*cs
static TinyGPSCustom gstStdLat(gps, "GNGST", 6);
static TinyGPSCustom gstStdLon(gps, "GNGST", 7);
static TinyGPSCustom gstStdAlt(gps, "GNGST", 8);

// GGA fix quality: 0=no fix, 1=autonomous, 2=DGPS, 4=RTK fixed, 5=RTK float.
// $GNGGA,time,lat,NS,lon,EW,quality,numSV,hdop,alt,M,geoidSep,M,diffAge,diffStation*cs
static TinyGPSCustom ggaFixQuality(gps, "GNGGA", 6);

// UBX-CFG-MSG: enable NMEA GST (class 0xF0, id 0x07) on UART1 only, rate 1.
// This setting lives in RAM only, so it's resent every boot - nothing to undo.
static const uint8_t UBX_ENABLE_GST_UART1[] = {
  0xB5, 0x62,             // sync chars
  0x06, 0x01,             // class/id: CFG-MSG
  0x08, 0x00,             // payload length = 8
  0xF0, 0x07,             // msgClass=NMEA, msgID=GST
  0x00, 0x01, 0x00, 0x00, 0x00, 0x00, // rates: DDC, UART1, UART2, USB, SPI, reserved
  0x07, 0x59              // checksum (CK_A, CK_B)
};

// UBX-CFG-MSG: enable RXM-RAWX (class 0x02, id 0x15) and RXM-SFRBX
// (class 0x02, id 0x13) on UART1 - raw pseudorange/carrier-phase and
// broadcast navigation data, needed for PPK/PPP post-processing. Only sent
// when SD logging is actually available, so nothing changes on the F9P
// until the SD reader is wired up.
static const uint8_t UBX_ENABLE_RAWX_UART1[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x02, 0x15, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x27, 0x4B
};
static const uint8_t UBX_ENABLE_SFRBX_UART1[] = {
  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
  0x02, 0x13, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x25, 0x3D
};

// UBX-RXM-PMREQ (class 0x02, id 0x41), 16-byte payload version, requesting
// indefinite backup mode (duration=0), force+backup flags (0x06 - enters
// backup even with USB attached, so bring-up over USB serial behaves the
// same as field/battery operation), waking on EXTINT0 or UART RX activity
// (whichever fires first - UART RX is a bring-up safety net in case EXTINT
// wiring/polarity needs adjustment). Byte layout and wakeupSources bit
// values (EXTINT0=0x20, UARTRX=0x08) verified against SparkFun's u-blox
// GNSS Arduino library (src/SparkFun_u-blox_GNSS_Arduino_Library.cpp,
// powerOffWithInterrupt()), checksum computed by hand.
static const uint8_t UBX_PMREQ_BACKUP_EXTINT_UARTRX[] = {
  0xB5, 0x62, 0x02, 0x41, 0x10, 0x00,
  0x00, 0x00, 0x00, 0x00,             // version, reserved
  0x00, 0x00, 0x00, 0x00,             // duration = 0 (indefinite)
  0x06, 0x00, 0x00, 0x00,             // flags: force | backup
  0x28, 0x00, 0x00, 0x00,             // wakeupSources: EXTINT0 | UARTRX
  0x81, 0xEB                          // checksum (CK_A, CK_B)
};

// Raw GGA line last seen from the F9P itself, relayed to the NTRIP caster
// as-is (correct format/precision/checksum guaranteed since the module made it).
static char lastGGA[100] = "";
static bool haveGGA = false;
static char lineBuf[100];
static uint8_t lineLen = 0;

// Daily-rotated raw NMEA+UBX capture (for PPK/PPP post-processing), only
// meaningful once GNSS time is known (used for the filename date).
static File rawLogFile;
static char rawLogDate[9] = "";

static void rawLogRotateIfNeeded() {
  if (!sdStorageAvailable() || !gnssHasDateTime()) {
    return;
  }
  char today[9];
  gnssDateString(today, sizeof(today));
  if (strcmp(today, rawLogDate) != 0) {
    if (rawLogFile) {
      rawLogFile.close();
    }
    char path[48];
    sdBuildDailyPath(path, sizeof(path), "gnss_raw", today, "log");
    rawLogFile = sdOpenForAppend(path);
    strncpy(rawLogDate, today, sizeof(rawLogDate) - 1);
  }
}

void gnssEnableGST() {
  GPSSerial.write(UBX_ENABLE_GST_UART1, sizeof(UBX_ENABLE_GST_UART1));
}

// Safe to call once per boot only - HardwareSerial::setRxBufferSize()
// throws an error if the UART is already running. The persistent main
// loop calls initGNSS() once at startup; each subsequent wake cycle only
// needs the message-enable resend below (backup mode can reset it), not a
// fresh UART begin(), so that part is split into gnssReenableMessages().
static bool uartStarted = false;

void gnssReenableMessages() {
  gnssEnableGST();

  if (sdStorageAvailable()) {
    GPSSerial.write(UBX_ENABLE_RAWX_UART1, sizeof(UBX_ENABLE_RAWX_UART1));
    GPSSerial.write(UBX_ENABLE_SFRBX_UART1, sizeof(UBX_ENABLE_SFRBX_UART1));
  }
}

void initGNSS() {
  if (!uartStarted) {
    GPSSerial.setRxBufferSize(1024);
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    uartStarted = true;
    delay(100);
  }

  gnssReenableMessages();
}

void gnssSleep() {
  GPSSerial.write(UBX_PMREQ_BACKUP_EXTINT_UARTRX, sizeof(UBX_PMREQ_BACKUP_EXTINT_UARTRX));
  GPSSerial.flush();
}

void gnssWakeup() {
  // Wake trigger is an edge on EXTINT (u-blox docs/reference drivers note
  // it can be a falling or rising edge) - drive both to not depend on which.
  pinMode(GPS_EXTINT_PIN, OUTPUT);
  digitalWrite(GPS_EXTINT_PIN, LOW);
  delay(10);
  digitalWrite(GPS_EXTINT_PIN, HIGH);
  delay(10);
  digitalWrite(GPS_EXTINT_PIN, LOW);
}

void pollGNSS() {
  rawLogRotateIfNeeded();

  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    if (DEBUG_PRINT_RAW_GNSS) {
      Serial.write(c);
    }
    if (rawLogFile) {
      rawLogFile.write((uint8_t)c);
    }
    gps.encode(c);

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen >= 6 && strncmp(lineBuf + 3, "GGA", 3) == 0) {
        strncpy(lastGGA, lineBuf, sizeof(lastGGA) - 1);
        lastGGA[sizeof(lastGGA) - 1] = '\0';
        haveGGA = true;
      }
      lineLen = 0;
    } else if (c != '\r' && lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

void gnssWriteRTCM(uint8_t b) {
  GPSSerial.write(b);
}

bool gnssHasFix() {
  return gps.location.isValid();
}

int gnssFixQuality() {
  return ggaFixQuality.isValid() ? atoi(ggaFixQuality.value()) : 0;
}

const char *gnssFixQualityLabel(int quality) {
  switch (quality) {
    case 0: return "no fix";
    case 1: return "autonomous";
    case 2: return "DGPS";
    case 3: return "PPS";
    case 4: return "RTK FIXED";
    case 5: return "RTK FLOAT";
    case 6: return "estimated/dead-reckoning";
    default: return "unknown";
  }
}

bool gnssHasAccuracyEstimate() {
  return gstStdLat.isValid();
}

float gnssHorizontalAccuracy() {
  float latErr = atof(gstStdLat.value());
  float lonErr = atof(gstStdLon.value());
  return sqrt(latErr * latErr + lonErr * lonErr);
}

float gnssVerticalAccuracy() {
  return atof(gstStdAlt.value());
}

double gnssLatitude() { return gps.location.lat(); }
double gnssLongitude() { return gps.location.lng(); }
double gnssAltitude() { return gps.altitude.meters(); }

uint32_t gnssSatellites() {
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

bool gnssHasDateTime() {
  return gps.date.isValid() && gps.time.isValid();
}

void gnssTimestamp(char *buf, size_t bufSize) {
  snprintf(buf, bufSize, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());
}

void gnssDateString(char *buf, size_t bufSize) {
  snprintf(buf, bufSize, "%04d%02d%02d", gps.date.year(), gps.date.month(), gps.date.day());
}

uint16_t gnssYear() { return gps.date.year(); }
uint8_t gnssMonth() { return gps.date.month(); }
uint8_t gnssDay() { return gps.date.day(); }
uint8_t gnssHour() { return gps.time.hour(); }
uint8_t gnssMinute() { return gps.time.minute(); }
uint8_t gnssSecond() { return gps.time.second(); }
uint16_t gnssMillisecond() { return gps.time.centisecond() * 10; }

const char *gnssLastGGA() {
  return haveGGA ? lastGGA : "";
}

uint32_t gnssCharsProcessed() {
  return gps.charsProcessed();
}
