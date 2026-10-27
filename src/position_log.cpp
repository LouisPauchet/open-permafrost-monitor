#include "position_log.h"
#include "sd_storage.h"
#include "gnss.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static char currentDate[9] = "";
static char currentPath[48] = "";
static size_t sentOffset = 0;   // committed high-water mark
static size_t pendingOffset = 0; // file size as of the last positionLogReadUnsent() call

static const char *CSV_HEADER = "time,latitude,longitude,altitude,horizontal_accuracy,"
                                 "vertical_accuracy,fix_quality,quality_ok,battery_external_v,"
                                 "battery_internal_v,has_displacement,north_m,east_m,up_m,epoch_ms";

// Row time and the daily file's date both come from gnssEpochMillisNow()
// (extrapolated from the last fresh fix), not TinyGPS++'s date/time fields:
// those never expire, so a burst in which the F9P never reported a time
// would have logged - and published to ThingsBoard, which keys rows by ts -
// the previous burst's timestamp again, overwriting that earlier reading.
static bool currentCivilTime(char *date, size_t dateSize, char *iso, size_t isoSize,
                             uint64_t *epochMs) {
  if (!gnssHasEpochAnchor()) {
    return false;
  }
  *epochMs = gnssEpochMillisNow();
  uint16_t year, milli;
  uint8_t month, day, hour, minute, second;
  gnssCivilFromEpochMillis(*epochMs, &year, &month, &day, &hour, &minute, &second, &milli);
  if (date) {
    snprintf(date, dateSize, "%04u%02u%02u", year, month, day);
  }
  if (iso) {
    snprintf(iso, isoSize, "%04u-%02u-%02uT%02u:%02u:%02uZ", year, month, day, hour, minute, second);
  }
  return true;
}

static void rotateIfNeeded() {
  char today[9];
  uint64_t epochMs;
  if (!currentCivilTime(today, sizeof(today), nullptr, 0, &epochMs)) {
    return;
  }
  if (strcmp(today, currentDate) == 0) {
    // Already rotated to today - but if the open never actually landed
    // (e.g. a transient SD hiccup right at rotation, seen on live hardware
    // racing the tail end of a GNSS burst's own SD writes), retry here
    // rather than leaving position logging silently stuck until the next
    // reboot. Harmless to check every call: sdStreamIsOpen() is just a
    // local flag read, and sdStreamOpen() is a no-op-cost request once the
    // file is actually open (this branch is skipped entirely then).
    if (currentPath[0] != '\0' && !sdStreamIsOpen(SD_STREAM_POSITIONS)) {
      sdStreamOpen(SD_STREAM_POSITIONS, currentPath, CSV_HEADER);
    }
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
  uint64_t epochMs = 0;
  currentCivilTime(nullptr, 0, ts, sizeof(ts), &epochMs);

  char row[256];
  int len = snprintf(
      row, sizeof(row), "%s,%.8f,%.8f,%.2f,%.3f,%.3f,%d,%s,%.2f,%.2f,%s,%.4f,%.4f,%.4f,%llu\n", ts,
      gnssLatitude(), gnssLongitude(), gnssAltitude(),
      gnssHasAccuracyEstimate() ? gnssHorizontalAccuracy() : -1.0f,
      gnssHasAccuracyEstimate() ? gnssVerticalAccuracy() : -1.0f, gnssFixQuality(),
      qualityOk ? "true" : "false", batteryExternalV, batteryInternalV,
      hasDisplacement ? "true" : "false", hasDisplacement ? northM : 0.0f,
      hasDisplacement ? eastM : 0.0f, hasDisplacement ? upM : 0.0f,
      (unsigned long long)epochMs);
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
// sentOffset lives in RAM only, so after a reboot the day's rows are
// published again - harmless, since ThingsBoard keys each row by its ts.
size_t positionLogReadUnsent(char *buf, size_t bufSize) {
  if (!sdStorageAvailable() || currentPath[0] == '\0') {
    return 0;
  }

  File f = SD.open(currentPath, FILE_READ);
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

    char lineBuf[256];
    strncpy(lineBuf, line.c_str(), sizeof(lineBuf) - 1);
    lineBuf[sizeof(lineBuf) - 1] = '\0';

    char *field[15];
    int n = 0;
    char *tok = strtok(lineBuf, ",");
    while (tok != nullptr && n < 15) {
      field[n++] = tok;
      tok = strtok(nullptr, ",");
    }
    if (n < 15) {
      lastGoodPos = posAfterLine; // malformed/short line (e.g. the header row) - skip for good
      continue;
    }

    // ThingsBoard's telemetry API only treats a message as timestamped by
    // the actual reading time if it's shaped {"ts":<epoch ms>,"values":
    // {...}} - a plain flat object gets stamped with the server's ingestion
    // time instead, which is wrong here whenever a batch is delayed (e.g.
    // picked up several cycles late after an SD-triggered reboot - see
    // main.cpp/sd_storage.cpp). epoch_ms (field[14]) is written by
    // positionLogAppend() from the GNSS UTC clock, not the publish time.
    JsonObject row = arr.add<JsonObject>();
    row["ts"] = strtoull(field[14], nullptr, 10);
    JsonObject values = row["values"].to<JsonObject>();
    values["latitude"] = atof(field[1]);
    values["longitude"] = atof(field[2]);
    values["altitude"] = atof(field[3]);
    values["horizontal_accuracy"] = atof(field[4]);
    values["vertical_accuracy"] = atof(field[5]);
    values["fix_quality"] = atoi(field[6]);
    values["quality_ok"] = (strcmp(field[7], "true") == 0);
    values["battery_external_v"] = atof(field[8]);
    values["battery_internal_v"] = atof(field[9]);
    // Per-row: was a valid displacement actually computed for this reading
    // (gated on quality_ok, see positionLogAppend()) - NOT whether the
    // station's reference position is locked, which is a one-time,
    // effectively permanent state after commissioning completes. The old
    // field name "ref_locked" read as the latter and could easily be
    // misread on a dashboard as the station repeatedly losing its
    // reference lock, when a false value here just means this particular
    // reading's quality wasn't good enough to trust for displacement.
    bool hasDisplacement = (strcmp(field[10], "true") == 0);
    values["has_displacement"] = hasDisplacement;
    float northM = atof(field[11]);
    float eastM = atof(field[12]);
    values["north_m"] = northM;
    values["east_m"] = eastM;
    values["up_m"] = atof(field[13]);
    // Derived, not stored separately on disk - dashboards generally want a
    // single at-a-glance "how far has it moved" figure alongside the
    // directional components.
    values["horizontal_displacement_m"] =
        hasDisplacement ? sqrtf(northM * northM + eastM * eastM) : 0.0f;

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
