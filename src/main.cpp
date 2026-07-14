#include <Arduino.h>
#include <TinyGPSPlus.h>

// ZED-F9P (gnss.store ELT0128) wired to ESP32 UART2.
// SEL jumper left unsoldered -> module's UART1 is the active serial interface.
//   F9P UART1 TXO -> ESP32 GPIO16 (RX2)
//   F9P UART1 RXI -> ESP32 GPIO17 (TX2)
//   F9P VCC       -> 3.3V
//   F9P GND       -> GND
//
// ZED-F9P ships from factory at 38400 baud on UART1. If nothing shows up,
// try 9600 or 115200 in GPS_BAUD below (some boards get pre-configured).

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 38400

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

unsigned long lastSummaryMs = 0;
const unsigned long SUMMARY_INTERVAL_MS = 1000;

void printFixSummary() {
  Serial.println(F("---- Fix summary ----"));

  Serial.print(F("Fix: "));
  Serial.println(gps.location.isValid() ? F("YES") : F("no"));

  Serial.print(F("Satellites: "));
  Serial.println(gps.satellites.isValid() ? gps.satellites.value() : 0);

  Serial.print(F("HDOP: "));
  Serial.println(gps.hdop.isValid() ? gps.hdop.hdop() : -1.0, 2);

  if (gps.location.isValid()) {
    Serial.print(F("Lat: "));
    Serial.println(gps.location.lat(), 8);
    Serial.print(F("Lng: "));
    Serial.println(gps.location.lng(), 8);
  }

  if (gps.altitude.isValid()) {
    Serial.print(F("Alt (m): "));
    Serial.println(gps.altitude.meters(), 2);
  }

  if (gps.time.isValid()) {
    Serial.printf("UTC time: %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  Serial.print(F("Sentences with fix: "));
  Serial.println(gps.sentencesWithFix());
  Serial.print(F("Failed checksum: "));
  Serial.println(gps.failedChecksum());
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for USB serial on boards that need it
  }
  delay(200);

  Serial.println(F("ESP32 <-> ZED-F9P basic fix test"));
  Serial.printf("Opening GPS UART2 at %d baud (RX=%d, TX=%d)\n", GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void loop() {
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    Serial.write(c);      // raw NMEA passthrough so you can see exactly what the module sends
    gps.encode(c);        // feed the parser
  }

  if (millis() - lastSummaryMs >= SUMMARY_INTERVAL_MS) {
    lastSummaryMs = millis();
    printFixSummary();

    if (millis() > 15000 && gps.charsProcessed() < 10) {
      Serial.println(F("WARNING: no GPS data received - check wiring/baud rate (try 9600 or 115200)."));
    }
  }
}
