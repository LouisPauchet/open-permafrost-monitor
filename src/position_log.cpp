#include "position_log.h"
#include "sd_storage.h"
#include "gnss.h"
#include <ArduinoJson.h>
#include <SD_MMC.h>
#include <string.h>
#include <stdlib.h>

static char currentDate[9] = "";
static char currentPath[48] = "";
static size_t sentOffset = 0;   // committed high-water mark
static size_t pendingOffset = 0; // file size as of the last positionLogReadUnsent() call

static const char *CSV_HEADER = "time,latitude,longitude,altitude,horizontal_accuracy,"
                                 "vertical_accuracy,fix_quality,quality_ok,battery_external_v,"
                                 "battery_internal_v";

static void rotateIfNeeded() {
  if (!gnssHasDateTime()) {
    return;
  }
  char today[9];
  gnssDateString(today, sizeof(today));
  if (strcmp(today, currentDate) == 0) {
    return;
  }

  strncpy(currentDate, today, sizeof(currentDate) - 1);
  sdBuildDailyPath(currentPath, sizeof(currentPath), "data/positions", today, "csv");
  sentOffset = 0;
  pendingOffset = 0;

  // Open (and, if the file turns out to be genuinely new, header) happens
  // on the SD writer task - see sd_storage.h. No more SD_MMC.exists() check
  // here: that was a blocking call in its own right, and headerIfEmpty
  // already handles "already has a header from before a same-day reboot"
  // by checking the file's size instead.
  sdStreamOpen(SD_STREAM_POSITIONS, currentPath, CSV_HEADER);
}

void initPositionLog() {
  currentDate[0] = '\0';
  currentPath[0] = '\0';
  sentOffset = 0;
  pendingOffset = 0;
}

void positionLogAppend(bool qualityOk, float batteryExternalV, float batteryInternalV) {
  rotateIfNeeded();
  if (!sdStorageAvailable() || currentPath[0] == '\0') {
    return;
  }

  char ts[25] = "";
  if (gnssHasDateTime()) {
    gnssTimestamp(ts, sizeof(ts));
  }

  char row[200];
  int len = snprintf(row, sizeof(row), "%s,%.8f,%.8f,%.2f,%.3f,%.3f,%d,%s,%.2f,%.2f\n", ts,
                      gnssLatitude(), gnssLongitude(), gnssAltitude(),
                      gnssHasAccuracyEstimate() ? gnssHorizontalAccuracy() : -1.0f,
                      gnssHasAccuracyEstimate() ? gnssVerticalAccuracy() : -1.0f, gnssFixQuality(),
                      qualityOk ? "true" : "false", batteryExternalV, batteryInternalV);
  if (len <= 0) {
    return;
  }
  if ((size_t)len >= sizeof(row)) {
    len = sizeof(row) - 1; // truncated - still write what fits rather than nothing
  }
  sdStreamWrite(SD_STREAM_POSITIONS, (const uint8_t *)row, (size_t)len);
}

// Two known, currently-accepted gaps left by this still being a direct
// blocking read while positionLogAppend() writes via the SD writer task
// (see sd_storage.h): (1) this read isn't itself protected against a
// wedged card the way the write side now is - a hang here would still
// reach the watchdog: fine while telemetry runs on its own well-spaced
// timer, worth revisiting before calling this from somewhere more
// frequent/latency-sensitive. (2) a row appended moments ago may not be
// visible yet - it can still be sitting in the writer task's ring buffer
// or its 50ms drain cycle hasn't run - but it's never lost, just picked up
// on the next call via sentOffset, so this only affects freshness.
size_t positionLogReadUnsent(char *buf, size_t bufSize) {
  if (!sdStorageAvailable() || currentPath[0] == '\0') {
    return 0;
  }

  File f = SD_MMC.open(currentPath, FILE_READ);
  if (!f) {
    return 0;
  }

  size_t fileSize = f.size();
  if (fileSize <= sentOffset) {
    f.close();
    return 0; // nothing new since the last publish
  }
  pendingOffset = fileSize;
  f.seek(sentOffset);

  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();

  size_t count = 0;
  const size_t MAX_ROWS = 100; // bounds worst-case memory if a publish was skipped for a while
  while (f.available() && count < MAX_ROWS) {
    String line = f.readStringUntil('\n');
    if (line.length() == 0) {
      continue;
    }

    char lineBuf[160];
    strncpy(lineBuf, line.c_str(), sizeof(lineBuf) - 1);
    lineBuf[sizeof(lineBuf) - 1] = '\0';

    char *field[10];
    int n = 0;
    char *tok = strtok(lineBuf, ",");
    while (tok != nullptr && n < 10) {
      field[n++] = tok;
      tok = strtok(nullptr, ",");
    }
    if (n < 10) {
      continue; // malformed/short line (e.g. the header row) - skip
    }

    JsonObject row = arr.add<JsonObject>();
    row["time"] = field[0];
    row["latitude"] = atof(field[1]);
    row["longitude"] = atof(field[2]);
    row["altitude"] = atof(field[3]);
    row["horizontal_accuracy"] = atof(field[4]);
    row["vertical_accuracy"] = atof(field[5]);
    row["fix_quality"] = atoi(field[6]);
    row["quality_ok"] = (strcmp(field[7], "true") == 0);
    row["battery_external_v"] = atof(field[8]);
    row["battery_internal_v"] = atof(field[9]);

    count++;
  }

  f.close();

  if (count > 0) {
    serializeJson(arr, buf, bufSize);
  }

  return count;
}

void positionLogMarkSent() {
  sentOffset = pendingOffset;
}
