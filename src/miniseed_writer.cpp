#include "miniseed_writer.h"
#include "sd_storage.h"
#include "gnss.h"
#include <string.h>
#include <stdio.h>

// SEED identifiers - "XX" is the reserved placeholder network code for
// unregistered/temporary networks; customize both for a real deployment
// (each physical station should have a unique station code).
static const char *STATION_NETWORK = "XX";
static const char *STATION_CODE = "PERM1";
static const char *LOCATION_CODE = "";

static const int SAMPLE_RATE_HZ = 200; // raw digitizer rate - see raspberry_shake.md
static const int RECORD_LEN = 512;
static const int SAMPLES_PER_RECORD = 114; // (512 - 48 header - 8 blockette1000) / 4 bytes

struct ChannelBuffer {
  char name[4];
  int32_t samples[SAMPLES_PER_RECORD];
  int count;
  bool haveStart;
  uint16_t startYear;
  uint8_t startMonth, startDay, startHour, startMinute, startSecond;
  uint16_t startMs;
  uint32_t seq;
};

static ChannelBuffer channels[3];
static char dailyDate[9] = "";

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

static void rotateIfNeeded() {
  if (!sdStorageAvailable() || !gnssHasDateTime()) {
    return;
  }
  char today[9];
  gnssDateString(today, sizeof(today));
  if (strcmp(today, dailyDate) != 0) {
    char path[48];
    sdBuildDailyPath(path, sizeof(path), "data/geophones/geophone", today, "mseed");
    sdStreamOpen(SD_STREAM_MINISEED, path); // actual open happens on the SD writer task, see sd_storage.h
    strncpy(dailyDate, today, sizeof(dailyDate) - 1);
  }
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
  putU16BE(record + 20, ch->startYear);
  putU16BE(record + 22, (uint16_t)dayOfYear(ch->startYear, ch->startMonth, ch->startDay));
  record[24] = ch->startHour;
  record[25] = ch->startMinute;
  record[26] = ch->startSecond;
  record[27] = 0; // unused
  putU16BE(record + 28, (uint16_t)(ch->startMs * 10)); // ms -> 0.0001s units

  putU16BE(record + 30, (uint16_t)ch->count);      // number of samples
  putU16BE(record + 32, (uint16_t)SAMPLE_RATE_HZ); // sample rate factor (positive = Hz)
  putU16BE(record + 34, 1);                        // sample rate multiplier

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
  ch->haveStart = false;
}

void initMiniseedWriter() {
  strncpy(channels[0].name, "EH1", 4);
  strncpy(channels[1].name, "EH2", 4);
  strncpy(channels[2].name, "EH3", 4);
  for (int i = 0; i < 3; i++) {
    channels[i].count = 0;
    channels[i].haveStart = false;
    channels[i].seq = 1;
  }
}

void miniseedAddSample(const char *channel, int32_t sample) {
  rotateIfNeeded();

  ChannelBuffer *ch = findChannel(channel);
  if (!ch) {
    return;
  }

  if (!ch->haveStart) {
    if (!gnssHasDateTime()) {
      return; // no absolute time yet (e.g. before the first GNSS fix) - drop until we have one
    }
    ch->startYear = gnssYear();
    ch->startMonth = gnssMonth();
    ch->startDay = gnssDay();
    ch->startHour = gnssHour();
    ch->startMinute = gnssMinute();
    ch->startSecond = gnssSecond();
    ch->startMs = gnssMillisecond();
    ch->haveStart = true;
  }

  ch->samples[ch->count++] = sample;

  if (ch->count >= SAMPLES_PER_RECORD) {
    writeRecord(ch);
  }
}

// Moves whatever's currently buffered in RAM (a partial, not-yet-114-sample
// record) out into the stream, rather than waiting for a full record - so a
// quiet channel's data still reaches the card within MINISEED_FLUSH_INTERVAL_MS
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
