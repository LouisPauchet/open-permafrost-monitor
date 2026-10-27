#include "miniseed_writer.h"
#include "sd_storage.h"
#include "gnss.h"
#include "geophone_timing.h"
#include <string.h>
#include <stdio.h>

// SEED identifiers - "XX" is the reserved placeholder network code for
// unregistered/temporary networks; customize both for a real deployment
// (each physical station should have a unique station code).
static const char *STATION_NETWORK = "XX";
static const char *STATION_CODE = "PERM1";
static const char *LOCATION_CODE = "";

static const uint32_t DEFAULT_SAMPLE_INTERVAL_US = 5000; // 200 Hz raw digitizer rate - see raspberry_shake.md
static const int RECORD_LEN = 512;
// A channel's sample clock is only snapped to the digitizer round's start
// when they disagree by more than this - 4 samples at 200 Hz. Arrival
// jitter is already smoothed out by GeoClock, so crossing it means a real
// discontinuity (lost packet) or accumulated clock drift.
static const int64_t RESYNC_THRESHOLD_US = 20000;
static const int SAMPLES_PER_RECORD = 114; // (512 - 48 header - 8 blockette1000) / 4 bytes

struct ChannelBuffer {
  char name[4];
  int32_t samples[SAMPLES_PER_RECORD];
  int count;
  int64_t startUs;      // UTC epoch us of samples[0]
  uint32_t intervalUs;  // sample interval this record was built at
  int64_t nextUs;       // where the next packet's first sample is expected
  bool haveNext;
  int64_t lastRoundMs;  // digitizer round (marker MSEC) this channel last stamped from
  uint32_t seq;
};

static ChannelBuffer channels[3];
static char dailyDate[9] = "";
static int64_t dailyDayNumber = -1;

static int dayOfYear(int year, int month, int day) {
  static const int cum[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  int doy = cum[month - 1] + day;
  bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  if (leap && month > 2) {
    doy++;
  }
  return doy;
}

static void putU16BE(uint8_t *p, uint16_t v) {
  p[0] = (v >> 8) & 0xFF;
  p[1] = v & 0xFF;
}

static void putU32BE(uint8_t *p, uint32_t v) {
  p[0] = (v >> 24) & 0xFF;
  p[1] = (v >> 16) & 0xFF;
  p[2] = (v >> 8) & 0xFF;
  p[3] = v & 0xFF;
}

static ChannelBuffer *findChannel(const char *name) {
  for (int i = 0; i < 3; i++) {
    if (strncmp(channels[i].name, name, 3) == 0) {
      return &channels[i];
    }
  }
  return nullptr;
}

static void rotateIfNeeded(int64_t epochUs) {
  if (!sdStorageAvailable() || epochUs < 0) {
    return;
  }
  // Cheap integer compare first - this runs once per packet (12/s), and the
  // civil-date formatting below only once per UTC day.
  int64_t dayNumber = epochUs / 86400000000LL;
  if (dayNumber == dailyDayNumber) {
    return;
  }
  uint16_t year, milli;
  uint8_t month, day, hour, minute, second;
  gnssCivilFromEpochMillis((uint64_t)(epochUs / 1000), &year, &month, &day, &hour, &minute, &second,
                           &milli);
  char today[9];
  snprintf(today, sizeof(today), "%04u%02u%02u", year, month, day);
  char path[48];
  sdBuildDailyPath(path, sizeof(path), "data/geophones/geophone", today, "mseed");
  sdStreamOpen(SD_STREAM_MINISEED, path); // actual open happens on the SD writer task, see sd_storage.h
  strncpy(dailyDate, today, sizeof(dailyDate) - 1);
  dailyDayNumber = dayNumber;
}

// Writes one fixed 512-byte MiniSEED data record (SEED v2.4 fixed header +
// blockette 1000 + uncompressed 32-bit int samples, big-endian throughout).
static void writeRecord(ChannelBuffer *ch) {
  if (ch->count == 0) {
    return;
  }

  uint8_t record[RECORD_LEN];
  memset(record, 0, sizeof(record));

  char seqStr[7];
  snprintf(seqStr, sizeof(seqStr), "%06lu", (unsigned long)(ch->seq % 1000000));
  memcpy(record, seqStr, 6);
  record[6] = 'D'; // data header/quality indicator
  record[7] = ' '; // reserved

  memset(record + 8, ' ', 5);
  memcpy(record + 8, STATION_CODE, strlen(STATION_CODE) < 5 ? strlen(STATION_CODE) : 5);

  memset(record + 13, ' ', 2);
  size_t locLen = strlen(LOCATION_CODE);
  memcpy(record + 13, LOCATION_CODE, locLen < 2 ? locLen : 2);

  memcpy(record + 15, ch->name, 3);

  memset(record + 18, ' ', 2);
  size_t netLen = strlen(STATION_NETWORK);
  memcpy(record + 18, STATION_NETWORK, netLen < 2 ? netLen : 2);

  // Start time (BTIME, 10 bytes) at offset 20.
  uint16_t year, milli;
  uint8_t month, day, hour, minute, second;
  gnssCivilFromEpochMillis((uint64_t)(ch->startUs / 1000), &year, &month, &day, &hour, &minute,
                           &second, &milli);
  putU16BE(record + 20, year);
  putU16BE(record + 22, (uint16_t)dayOfYear(year, month, day));
  record[24] = hour;
  record[25] = minute;
  record[26] = second;
  record[27] = 0; // unused
  putU16BE(record + 28, (uint16_t)((ch->startUs / 100) % 10000)); // 0.0001s units

  // Sample rate as factor/multiplier: factor = Hz with multiplier 1 when
  // that's a whole number (200 Hz in practice); otherwise factor 1000 with
  // a negative multiplier (= divide) of the interval in ms, i.e.
  // 1000/ms Hz - exact for any whole-ms interval.
  putU16BE(record + 30, (uint16_t)ch->count); // number of samples
  if (ch->intervalUs > 0 && 1000000UL % ch->intervalUs == 0) {
    putU16BE(record + 32, (uint16_t)(1000000UL / ch->intervalUs));
    putU16BE(record + 34, 1);
  } else {
    uint32_t intervalMs = ch->intervalUs / 1000 ? ch->intervalUs / 1000 : 1;
    putU16BE(record + 32, 1000);
    putU16BE(record + 34, (uint16_t)(-(int16_t)intervalMs));
  }

  record[36] = 0; // activity flags
  record[37] = 0; // I/O and clock flags
  record[38] = 0; // data quality flags
  record[39] = 1; // number of blockettes that follow

  putU32BE(record + 40, 0); // time correction

  putU16BE(record + 44, 56); // beginning of data (byte offset in record)
  putU16BE(record + 46, 48); // first blockette offset

  // Blockette 1000 (Data Only SEED Blockette, 8 bytes) at offset 48.
  putU16BE(record + 48, 1000); // blockette type
  putU16BE(record + 50, 0);    // next blockette offset (none)
  record[52] = 3; // encoding format: 32-bit integers
  record[53] = 1; // word order: big-endian
  record[54] = 9; // record length = 2^9 = 512
  record[55] = 0; // reserved

  for (int i = 0; i < ch->count; i++) {
    putU32BE(record + 56 + i * 4, (uint32_t)ch->samples[i]);
  }

  sdStreamWrite(SD_STREAM_MINISEED, record, RECORD_LEN);

  ch->seq++;
  ch->count = 0;
}

void initMiniseedWriter() {
  strncpy(channels[0].name, "EH1", 4);
  strncpy(channels[1].name, "EH2", 4);
  strncpy(channels[2].name, "EH3", 4);
  for (int i = 0; i < 3; i++) {
    channels[i].count = 0;
    channels[i].haveNext = false;
    channels[i].intervalUs = DEFAULT_SAMPLE_INTERVAL_US;
    channels[i].seq = 1;
  }
}

void miniseedAddPacket(const char *channel, const int32_t *samples, size_t n,
                       uint32_t sampleIntervalUs, int64_t roundMs, int64_t roundStartUs) {
  ChannelBuffer *ch = findChannel(channel);
  if (!ch || n == 0) {
    return;
  }
  // A second packet for this channel within the same round means the
  // marker in between was lost (corrupt object dropped) - roundStartUs is
  // then the *previous* round's start, 250 ms behind. Trust the channel's
  // own continuous clock instead of snapping back to it.
  if (ch->haveNext && roundMs == ch->lastRoundMs) {
    roundStartUs = ch->nextUs;
  }
  ch->lastRoundMs = roundMs;
  uint32_t intervalUs = sampleIntervalUs ? sampleIntervalUs : DEFAULT_SAMPLE_INTERVAL_US;

  bool hadClock = ch->haveNext;
  int64_t expectedUs = ch->nextUs;
  bool resynced = false;
  int64_t t = geoChannelPacketStartUs(&ch->nextUs, &ch->haveNext, roundStartUs,
                                      RESYNC_THRESHOLD_US, &resynced);
  // A record's samples must be contiguous at one rate - close the partial
  // one rather than let a time step or rate change land inside it.
  if (ch->count > 0 && (resynced || intervalUs != ch->intervalUs)) {
    writeRecord(ch);
  }
  if (resynced && hadClock) {
    // Expected to be rare (a lost packet, or slow digitizer-vs-GNSS clock
    // drift crossing RESYNC_THRESHOLD_US) - frequent lines here mean
    // something upstream is dropping data.
    Serial.printf("MiniSEED: %s time re-synced by %lld us\n", ch->name,
                  (long long)(roundStartUs - expectedUs));
  }
  rotateIfNeeded(t);

  for (size_t i = 0; i < n; i++) {
    if (ch->count == 0) {
      ch->startUs = t;
      ch->intervalUs = intervalUs;
    }
    ch->samples[ch->count++] = samples[i];
    t += intervalUs;
    if (ch->count >= SAMPLES_PER_RECORD) {
      writeRecord(ch);
    }
  }
  ch->nextUs = t;
}

// Moves whatever's currently buffered in RAM (a partial, not-yet-114-sample
// record) out into the stream, rather than waiting for a full record - so a
// quiet channel's data still reaches the card within geophone.cpp's MINISEED_FLUSH_INTERVAL_MS
// instead of waiting indefinitely for enough samples to accumulate. Actually
// reaching the physical card is the SD writer task's own job now (periodic
// flush, see sd_storage.cpp) - this function no longer needs to trigger
// that itself.
void miniseedFlush() {
  for (int i = 0; i < 3; i++) {
    if (channels[i].count > 0) {
      writeRecord(&channels[i]);
    }
  }
}
