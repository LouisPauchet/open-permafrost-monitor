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
