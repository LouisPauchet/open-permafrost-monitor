#pragma once
#include <Arduino.h>
#include <FS.h>

// SD_MMC-based storage (1-bit mode - see pins.h), mounted once at boot and
// held powered continuously, since the geophone writes to it 24/7. Fixed
// layout: /data for CSV-style logs, /data/gnss for per-burst GNSS UBX/RTCM
// captures, /data/geophones for the continuous MiniSEED stream (see
// sdStorageInit()). Shared "daily file" path primitive used by every
// single-file-per-day writer (MiniSEED, position CSV) - the per-burst GNSS
// files use their own naming, see gnss.cpp.

bool sdStorageInit();
bool sdStorageAvailable();

// Polls SD_CARD_DETECT_PIN (debounced) - reboots to cleanly mount a newly
// (re)inserted card, or marks storage unavailable the moment one is pulled
// rather than waiting for the next write to fail. Call every loop().
void sdCheckCardPresence();

// Builds "/<prefix>_<dateYYYYMMDD>.<ext>" into buf.
void sdBuildDailyPath(char *buf, size_t bufSize, const char *prefix,
                       const char *dateYYYYMMDD, const char *ext);

// Opens (creating if needed) a file for appending. Returned File's
// operator bool() is false if SD isn't available or the open failed.
File sdOpenForAppend(const char *path);

// --- Async stream writer ---
//
// Arduino's SD_MMC File API has no non-blocking mode of its own. Live
// hardware hit a task-watchdog abort with the exact same *shape* as the
// modem crashes fixed earlier (a blocking call landing on loopTask, no
// timeout, past the watchdog) but from SD_MMC.write() instead: a burst of
// "sdmmc_send_cmd returned 0x107" (ESP_ERR_TIMEOUT) errors under heavy
// GNSS burst write traffic, then total silence for ~60s before the abort -
// a degrading/wedged card, not the earlier RTCM-clock-speed issue (that
// one made mount/write fail fast and visibly; this one just stopped
// returning).
//
// A fixed, small set of named streams (not a general multi-file system -
// just enough for this firmware's actually-continuous writers) backed by a
// dedicated task: the caller only ever does a fast, bounded memcpy into a
// ring buffer (silently dropping data if the writer task can't keep up -
// same tradeoff already accepted for the modem's own socketRxBuf), so a
// wedged card can only ever block the writer task, never loopTask.
enum SdStream : uint8_t {
  SD_STREAM_UBX = 0,
  SD_STREAM_RTCM = 1,
  SD_STREAM_MINISEED = 2,
  SD_STREAM_POSITIONS = 3,
  SD_STREAM_COUNT,
};

// Starts the writer task - call once from setup(), after sdStorageInit().
void sdWriterInit();

// Closes any file currently open for this stream, then opens path (created
// if needed) for the NEXT write - both happen on the writer task, not here.
// headerIfEmpty, if non-NULL, is written (with a trailing newline) right
// after opening, but only if the file turned out to be empty - lets a
// caller like position_log.cpp ensure a header row without its own
// blocking SD_MMC.exists() check: opening the same still-populated file
// again after a same-day reboot (currentDate resets, but the file and its
// header don't) just skips it.
void sdStreamOpen(SdStream stream, const char *path, const char *headerIfEmpty = nullptr);

// Appends data to the stream's ring buffer. Fast and non-blocking - the
// actual SD write happens later, on the writer task.
void sdStreamWrite(SdStream stream, const uint8_t *data, size_t len);

// Closes the stream's current file (flushing it in the process) once the
// writer task has drained whatever's still in the ring buffer.
void sdStreamClose(SdStream stream);
