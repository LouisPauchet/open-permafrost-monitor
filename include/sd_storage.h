#pragma once
#include <Arduino.h>
#include <FS.h>

// SD_MMC-based storage (1-bit mode - see pins.h), mounted once at boot and
// held powered continuously, since the geophone writes to it 24/7. Shared
// "daily file" primitives used by every writer (MiniSEED, GNSS raw log,
// NTRIP log, position CSV).

bool sdStorageInit();
bool sdStorageAvailable();

// Builds "/<prefix>_<dateYYYYMMDD>.<ext>" into buf.
void sdBuildDailyPath(char *buf, size_t bufSize, const char *prefix,
                       const char *dateYYYYMMDD, const char *ext);

// Opens (creating if needed) a file for appending. Returned File's
// operator bool() is false if SD isn't available or the open failed.
File sdOpenForAppend(const char *path);
