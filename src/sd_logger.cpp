#include "sd_logger.h"
#include "pins.h"

#ifdef SD_CS_PIN
#include <SPI.h>
#include <SD.h>

static bool available = false;
static File logFile;
RTC_DATA_ATTR static uint32_t logFileCounter = 0;

bool initSDLogger() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD logging: card not detected, skipping."));
    return false;
  }

  logFileCounter++;
  char filename[32];
  snprintf(filename, sizeof(filename), "/gnss_%06lu.ubx", (unsigned long)logFileCounter);

  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.println(F("SD logging: failed to open log file."));
    return false;
  }

  Serial.print(F("SD logging: writing raw GNSS stream to "));
  Serial.println(filename);
  available = true;
  return true;
}

bool sdLoggerAvailable() {
  return available;
}

void sdLogRawByte(uint8_t b) {
  if (available) {
    logFile.write(b);
  }
}

void closeSDLogger() {
  if (available) {
    logFile.close();
    available = false;
  }
}

#else // SD_CS_PIN not defined - reader not wired up yet

bool initSDLogger() {
  return false;
}

bool sdLoggerAvailable() {
  return false;
}

void sdLogRawByte(uint8_t b) {
  (void)b;
}

void closeSDLogger() {
}

#endif
