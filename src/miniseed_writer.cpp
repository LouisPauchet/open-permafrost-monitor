#include "miniseed_writer.h"
#include "sd_storage.h"
#include "gnss.h"
#include "device_config.h"
#include "geophone_timing.h"
#include "miniseed_record.h"
#include <string.h>
#include <stdio.h>

// SEED network/station/location codes and the Steim2 switch come from
// deviceConfig (field-service menu) - only changed in field service mode,
// before the geophone task starts, so reading them here without locking is
// safe.

static const uint32_t DEFAULT_SAMPLE_INTERVAL_US = 5000; // 200 Hz raw digitizer rate - see raspberry_shake.md
// A channel's sample clock is only snapped to the digitizer round's start
// when they disagree by more than this - 4 samples at 200 Hz. Arrival
// jitter is already smoothed out by GeoClock, so crossing it means a real
// discontinuity (lost packet) or accumulated clock drift.
static const int64_t RESYNC_THRESHOLD_US = 20000;

// Samples are buffered until they fill a record (a Steim2 record holds a
// variable number, depending on how well they compress), then records are
// cut from the front. Room for one maximal Steim2 record plus a few
// packets beyond it.
static const size_t CHANNEL_BUFFER_SAMPLES = MSEED_STEIM2_MAX_SAMPLES_PER_RECORD + 300;
// Always leave room for one more packet (50 samples in practice) - past
// this, a not-yet-full record is written anyway.
static const size_t GEOPHONE_PACKET_HEADROOM = 128;

struct ChannelBuffer {
  char name[4];
  int32_t samples[CHANNEL_BUFFER_SAMPLES];
  size_t count;
  int64_t startUs;      // UTC epoch us of samples[0]
  uint32_t intervalUs;  // sample interval the buffered samples were taken at
  int32_t prevSample;   // last sample already written (Steim2's first difference)
  int64_t nextUs;       // where the next packet's first sample is expected
  bool haveNext;
  int64_t lastRoundMs;  // digitizer round (marker MSEC) this channel last stamped from
  uint32_t seq;
};

static ChannelBuffer channels[3];
static int64_t dailyDayNumber = -1;
static bool lowSpaceReported = false;

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
  dailyDayNumber = dayNumber;
}

// Cuts one record from the front of the channel's buffer and queues it for
// the SD writer task. With onlyIfFull, a record that would consume
// everything buffered (i.e. isn't full yet) is discarded instead and the
// samples wait for more - so normal operation only ever writes full
// records, and partial ones only come from miniseedFlush() or a time step.
// Returns whether a record was written.
static bool emitRecord(ChannelBuffer *ch, bool onlyIfFull) {
  if (ch->count == 0) {
    return false;
  }
  uint8_t record[MSEED_RECORD_LEN];
  MseedRecordInfo info = {deviceConfig.seed_network, deviceConfig.seed_station,
                          deviceConfig.seed_location, ch->name,
                          ch->seq, ch->startUs, ch->intervalUs};
  size_t used = mseedBuildRecord(record, info, ch->samples, ch->count, deviceConfig.miniseed_steim2,
                                 ch->prevSample);
  if (onlyIfFull && used == ch->count && ch->count < CHANNEL_BUFFER_SAMPLES - GEOPHONE_PACKET_HEADROOM) {
    return false;
  }

  // Below the free-space floor the geophone stops being written (the
  // positions CSV and GNSS raw files are tiny by comparison and keep going)
  // - the samples are still consumed so timing carries on seamlessly if
  // space is freed by swapping the card (which reboots anyway).
  if (sdStorageLowSpace()) {
    if (!lowSpaceReported) {
      lowSpaceReported = true;
      Serial.println(F("MiniSEED: SD card nearly full - geophone recording paused."));
    }
  } else {
    lowSpaceReported = false;
    sdStreamWrite(SD_STREAM_MINISEED, record, MSEED_RECORD_LEN);
  }

  ch->prevSample = ch->samples[used - 1];
  ch->seq++;
  ch->startUs += (int64_t)used * ch->intervalUs;
  ch->count -= used;
  memmove(ch->samples, ch->samples + used, ch->count * sizeof(ch->samples[0]));
  return true;
}

static void emitAll(ChannelBuffer *ch) {
  while (emitRecord(ch, false)) {
  }
}

void initMiniseedWriter() {
  strncpy(channels[0].name, "EH1", 4);
  strncpy(channels[1].name, "EH2", 4);
  strncpy(channels[2].name, "EH3", 4);
  for (int i = 0; i < 3; i++) {
    channels[i].count = 0;
    channels[i].haveNext = false;
    channels[i].intervalUs = DEFAULT_SAMPLE_INTERVAL_US;
    channels[i].prevSample = 0;
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
  // A record's samples must be contiguous at one rate - write out what's
  // buffered rather than let a time step or rate change land inside it.
  if (ch->count > 0 && (resynced || intervalUs != ch->intervalUs)) {
    emitAll(ch);
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
    if (ch->count >= CHANNEL_BUFFER_SAMPLES) {
      emitRecord(ch, false); // can't normally happen (see GEOPHONE_PACKET_HEADROOM) - never overflow
    }
  }
  // Speculative encode each packet: cheap (a few hundred cycles per
  // sample, 600 samples/s), and keeps every routine record full.
  while (emitRecord(ch, true)) {
  }
  ch->nextUs = t;
}

// Moves whatever's currently buffered in RAM (partial records) out into
// the stream, rather than waiting for full ones - so data reaches the card
// within geophone.cpp's MINISEED_FLUSH_INTERVAL_MS. Actually reaching the
// physical card is the SD writer task's own job (periodic flush, see
// sd_storage.cpp).
void miniseedFlush() {
  for (int i = 0; i < 3; i++) {
    emitAll(&channels[i]);
  }
}
