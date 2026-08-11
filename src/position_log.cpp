#include "position_log.h"
#include "sd_storage.h"
#include "gnss.h"
#include <ArduinoJson.h>
#include <SD_MMC.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static char currentDate[9] = "";
static char currentPath[48] = "";
static size_t sentOffset = 0;   // committed high-water mark
static size_t pendingOffset = 0; // file size as of the last positionLogReadUnsent() call

static const char *CSV_HEADER = "time,latitude,longitude,altitude,horizontal_accuracy,"
                                 "vertical_accuracy,fix_quality,quality_ok,battery_external_v,"
                                 "battery_internal_v,ref_locked,north_m,east_m,up_m";

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

void positionLogAppend(bool qualityOk, float batteryExternalV, float batteryInternalV,
                       bool hasDisplacement, float northM, float eastM, float upM) {
  rotateIfNeeded();
  if (!sdStorageAvailable() || currentPath[0] == '\0') {
    return;
  }

  char ts[25] = "";
  if (gnssHasDateTime()) {
    gnssTimestamp(ts, sizeof(ts));
  }

  char row[240];
  int len = snprintf(row, sizeof(row), "%s,%.8f,%.8f,%.2f,%.3f,%.3f,%d,%s,%.2f,%.2f,%s,%.4f,%.4f,%.4f\n",
                      ts, gnssLatitude(), gnssLongitude(), gnssAltitude(),
                      gnssHasAccuracyEstimate() ? gnssHorizontalAccuracy() : -1.0f,
                      gnssHasAccuracyEstimate() ? gnssVerticalAccuracy() : -1.0f, gnssFixQuality(),
                      qualityOk ? "true" : "false", batteryExternalV, batteryInternalV,
                      hasDisplacement ? "true" : "false", hasDisplacement ? northM : 0.0f,
                      hasDisplacement ? eastM : 0.0f, hasDisplacement ? upM : 0.0f);
  if (len <= 0) {
    return;
  }
  if ((size_t)len >= sizeof(row)) {
    len = sizeof(row) - 1; // truncated - still write what fits rather than nothing
  }
  sdStreamWrite(SD_STREAM_POSITIONS, (const uint8_t *)row, (size_t)len);
  // One row per measurement cycle (minutes apart), not a high-rate stream
  // like UBX/RTCM/MiniSEED - flushing immediately bounds this file's
  // exposure to sudden power loss to "this row" instead of the shared 5s
  // periodic sweep, worth the extra SD_MMC round trip at this rate.
  sdStreamFlush(SD_STREAM_POSITIONS);
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
  f.seek(sentOffset);

  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();

  size_t count = 0;
  // Tracks the file position through the last row actually accepted into
  // arr below - NOT fileSize. positionLogMarkSent() commits sentOffset up
  // to this value, so a row that gets left out (MAX_ROWS, or doesn't fit in
  // the caller's buffer - see below) stays pending and is retried on a
  // later call instead of being silently marked sent and lost.
  size_t lastGoodPos = sentOffset;
  const size_t MAX_ROWS = 100; // bounds worst-case memory if a publish was skipped for a while
  while (f.available() && count < MAX_ROWS) {
    String line = f.readStringUntil('\n');
    size_t posAfterLine = f.position();
    if (line.length() == 0) {
      lastGoodPos = posAfterLine;
      continue;
    }

    char lineBuf[240];
    strncpy(lineBuf, line.c_str(), sizeof(lineBuf) - 1);
    lineBuf[sizeof(lineBuf) - 1] = '\0';

    char *field[14];
    int n = 0;
    char *tok = strtok(lineBuf, ",");
    while (tok != nullptr && n < 14) {
      field[n++] = tok;
      tok = strtok(nullptr, ",");
    }
    if (n < 14) {
      lastGoodPos = posAfterLine; // malformed/short line (e.g. the header row) - skip for good
      continue;
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
    bool refLocked = (strcmp(field[10], "true") == 0);
    row["ref_locked"] = refLocked;
    float northM = atof(field[11]);
    float eastM = atof(field[12]);
    row["north_m"] = northM;
    row["east_m"] = eastM;
    row["up_m"] = atof(field[13]);
    // Derived, not stored separately on disk - dashboards generally want a
    // single at-a-glance "how far has it moved" figure alongside the
    // directional components.
    row["horizontal_displacement_m"] = refLocked ? sqrtf(northM * northM + eastM * eastM) : 0.0f;

    // Stop before this row would overflow the caller's buffer - publish the
    // valid, complete batch built so far now rather than a truncated one;
    // this row (and anything after it) is retried on the next call since
    // lastGoodPos isn't advanced past it.
    if (measureJson(arr) + 1 > bufSize) {
      arr.remove(arr.size() - 1);
      break;
    }

    count++;
    lastGoodPos = posAfterLine;
  }

  f.close();
  pendingOffset = lastGoodPos;

  if (count > 0) {
    size_t written = serializeJson(arr, buf, bufSize);
    buf[written < bufSize ? written : bufSize - 1] = '\0';
  }

  return count;
}

void positionLogMarkSent() {
  sentOffset = pendingOffset;
}
