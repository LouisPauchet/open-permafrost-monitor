#include "miniseed_record.h"
#include "utc_civil.h"
#include <stdio.h>
#include <string.h>

static const size_t HEADER_LEN = 48;
static const size_t INT32_DATA_OFFSET = 56;  // header + blockette 1000
static const size_t STEIM_DATA_OFFSET = 64;  // Steim frames must start on a 64-byte boundary
static const size_t STEIM_FRAMES = (MSEED_RECORD_LEN - STEIM_DATA_OFFSET) / 64;

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

static void putPadded(uint8_t *dst, const char *src, size_t width) {
  memset(dst, ' ', width);
  size_t len = src ? strlen(src) : 0;
  memcpy(dst, src, len < width ? len : width);
}

static void writeHeader(uint8_t *rec, const MseedRecordInfo &info, size_t count, bool steim2) {
  char seqStr[7];
  snprintf(seqStr, sizeof(seqStr), "%06lu", (unsigned long)(info.sequence % 1000000));
  memcpy(rec, seqStr, 6);
  rec[6] = 'D'; // data header/quality indicator
  rec[7] = ' '; // reserved
  putPadded(rec + 8, info.station, 5);
  putPadded(rec + 13, info.location, 2);
  putPadded(rec + 15, info.channel, 3);
  putPadded(rec + 18, info.network, 2);

  // Start time (BTIME, 10 bytes) at offset 20.
  int64_t totalMs = info.startUs / 1000;
  int64_t days = totalMs / 86400000LL;
  uint32_t msOfDay = (uint32_t)(totalMs % 86400000LL);
  int y;
  unsigned m, d;
  civilFromDaysUTC(days, &y, &m, &d);
  putU16BE(rec + 20, (uint16_t)y);
  putU16BE(rec + 22, (uint16_t)(days - daysFromCivilUTC(y, 1, 1) + 1)); // day of year
  rec[24] = (uint8_t)(msOfDay / 3600000);
  rec[25] = (uint8_t)((msOfDay / 60000) % 60);
  rec[26] = (uint8_t)((msOfDay / 1000) % 60);
  rec[27] = 0;                                                     // unused
  putU16BE(rec + 28, (uint16_t)((info.startUs / 100) % 10000)); // 0.0001 s units

  putU16BE(rec + 30, (uint16_t)count); // number of samples
  // Sample rate as factor/multiplier: factor = Hz with multiplier 1 when
  // that's a whole number (200 Hz in practice); otherwise factor 1000 with
  // a negative multiplier (= divide) of the interval in ms, i.e.
  // 1000/ms Hz - exact for any whole-ms interval.
  if (info.intervalUs > 0 && 1000000UL % info.intervalUs == 0) {
    putU16BE(rec + 32, (uint16_t)(1000000UL / info.intervalUs));
    putU16BE(rec + 34, 1);
  } else {
    uint32_t intervalMs = info.intervalUs / 1000 ? info.intervalUs / 1000 : 1;
    putU16BE(rec + 32, 1000);
    putU16BE(rec + 34, (uint16_t)(-(int16_t)intervalMs));
  }

  rec[36] = 0; // activity flags
  rec[37] = 0; // I/O and clock flags
  rec[38] = 0; // data quality flags
  rec[39] = 1; // number of blockettes that follow
  putU32BE(rec + 40, 0); // time correction
  putU16BE(rec + 44, (uint16_t)(steim2 ? STEIM_DATA_OFFSET : INT32_DATA_OFFSET)); // beginning of data
  putU16BE(rec + 46, HEADER_LEN);                                                // first blockette

  // Blockette 1000 (Data Only SEED Blockette, 8 bytes) at offset 48.
  putU16BE(rec + 48, 1000); // blockette type
  putU16BE(rec + 50, 0);    // next blockette offset (none)
  rec[52] = steim2 ? 11 : 3; // encoding format: Steim2 / 32-bit integers
  rec[53] = 1;               // word order: big-endian
  rec[54] = 9;               // record length = 2^9 = 512
  rec[55] = 0;               // reserved
}

// --- Steim2 ---
//
// Each 32-bit data word holds 1-7 differences, chosen by the 2-bit code
// for that word in the frame's first ("nibble") word, plus a 2-bit dnib in
// the word itself for codes 2 and 3 (SEED manual, appendix B):
//   code 1:            4 x 8-bit
//   code 2, dnib 1:    1 x 30-bit     code 3, dnib 0: 5 x 6-bit
//   code 2, dnib 2:    2 x 15-bit     code 3, dnib 1: 6 x 5-bit
//   code 2, dnib 3:    3 x 10-bit     code 3, dnib 2: 7 x 4-bit
struct SteimPacking {
  uint8_t count, bits, code, dnib;
};
// Densest first - the encoder takes the first one the next diffs fit.
static const SteimPacking PACKINGS[] = {
    {7, 4, 3, 2}, {6, 5, 3, 1}, {5, 6, 3, 0}, {4, 8, 1, 0},
    {3, 10, 2, 3}, {2, 15, 2, 2}, {1, 30, 2, 1},
};

static bool fitsBits(int64_t v, unsigned bits) {
  int64_t lim = (int64_t)1 << (bits - 1);
  return v >= -lim && v < lim;
}

static size_t encodeSteim2(uint8_t *data, const int32_t *samples, size_t n, int32_t prevSample) {
  // A difference outside 30 bits can't be stored: end the record before
  // that sample (its own record then starts with it as X0, where the
  // first difference doesn't matter).
  size_t limit = n < MSEED_STEIM2_MAX_SAMPLES_PER_RECORD ? n : MSEED_STEIM2_MAX_SAMPLES_PER_RECORD;
  for (size_t i = 1; i < limit; i++) {
    if (!fitsBits((int64_t)samples[i] - samples[i - 1], 30)) {
      limit = i;
      break;
    }
  }
  int64_t d0 = (int64_t)samples[0] - prevSample;
  if (!fitsBits(d0, 30)) {
    d0 = 0; // not used by decoders (X0 is explicit) - just keep it encodable
  }
  auto diff = [&](size_t i) -> int64_t { return i == 0 ? d0 : (int64_t)samples[i] - samples[i - 1]; };

  memset(data, 0, STEIM_FRAMES * 64);
  size_t idx = 0;
  for (size_t f = 0; f < STEIM_FRAMES && idx < limit; f++) {
    uint8_t *frame = data + f * 64;
    uint32_t nibbles = 0;
    for (size_t w = (f == 0 ? 3 : 1); w < 16 && idx < limit; w++) {
      size_t remaining = limit - idx;
      const SteimPacking *pk = &PACKINGS[sizeof(PACKINGS) / sizeof(PACKINGS[0]) - 1];
      for (const SteimPacking &cand : PACKINGS) {
        if (cand.count > remaining) {
          continue;
        }
        bool ok = true;
        for (size_t k = 0; k < cand.count && ok; k++) {
          ok = fitsBits(diff(idx + k), cand.bits);
        }
        if (ok) {
          pk = &cand;
          break;
        }
      }
      uint32_t word = 0;
      if (pk->code == 1) {
        for (size_t k = 0; k < 4; k++) {
          word |= ((uint32_t)diff(idx + k) & 0xFF) << (24 - 8 * k);
        }
      } else {
        uint32_t mask = (pk->bits == 32) ? 0xFFFFFFFFu : ((1u << pk->bits) - 1);
        word = (uint32_t)pk->dnib << 30;
        for (size_t k = 0; k < pk->count; k++) {
          unsigned shift = pk->bits * (pk->count - 1 - k);
          word |= ((uint32_t)diff(idx + k) & mask) << shift;
        }
      }
      putU32BE(frame + 4 * w, word);
      nibbles |= (uint32_t)pk->code << (30 - 2 * w);
      idx += pk->count;
    }
    putU32BE(frame, nibbles);
  }
  // Integration constants: first and last sample of the record.
  putU32BE(data + 4, (uint32_t)samples[0]);
  putU32BE(data + 8, (uint32_t)samples[idx - 1]);
  return idx;
}

size_t mseedBuildRecord(uint8_t *rec, const MseedRecordInfo &info, const int32_t *samples, size_t n,
                        bool steim2, int32_t prevSample) {
  if (n == 0) {
    return 0;
  }
  memset(rec, 0, MSEED_RECORD_LEN);
  size_t count;
  if (steim2) {
    count = encodeSteim2(rec + STEIM_DATA_OFFSET, samples, n, prevSample);
  } else {
    count = n < MSEED_INT32_SAMPLES_PER_RECORD ? n : MSEED_INT32_SAMPLES_PER_RECORD;
    for (size_t i = 0; i < count; i++) {
      putU32BE(rec + INT32_DATA_OFFSET + 4 * i, (uint32_t)samples[i]);
    }
  }
  writeHeader(rec, info, count, steim2);
  return count;
}
