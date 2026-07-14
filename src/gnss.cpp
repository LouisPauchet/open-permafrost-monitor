#include "gnss.h"
#include "pins.h"
#include <TinyGPSPlus.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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

// Raw GGA line last seen from the F9P itself, relayed to the NTRIP caster
// as-is (correct format/precision/checksum guaranteed since the module made it).
static char lastGGA[100] = "";
static bool haveGGA = false;
static char lineBuf[100];
static uint8_t lineLen = 0;

void initGNSS() {
  GPSSerial.setRxBufferSize(1024);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  delay(100);
  GPSSerial.write(UBX_ENABLE_GST_UART1, sizeof(UBX_ENABLE_GST_UART1));
}

void pollGNSS() {
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
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

const char *gnssLastGGA() {
  return haveGGA ? lastGGA : "";
}

uint32_t gnssCharsProcessed() {
  return gps.charsProcessed();
}
