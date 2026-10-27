// MiniSEED record builder tests - #included by test_main.cpp.
#include "miniseed_record.h"
#include "utc_civil.h"
#include <math.h>
#include <stdlib.h>

static uint16_t getU16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static uint32_t getU32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static int32_t signExtend(uint32_t v, unsigned bits) {
  uint32_t m = 1u << (bits - 1);
  v &= (bits == 32) ? 0xFFFFFFFFu : ((1u << bits) - 1);
  return (int32_t)((v ^ m) - m);
}

// Independent Steim2 decoder, written from the SEED manual rather than
// from the encoder. Returns the number of samples decoded, -1 on any
// inconsistency (including the Xn integrity check).
static int steim2Decode(const uint8_t *rec, int32_t *out, size_t maxOut) {
  size_t nsamp = getU16(rec + 30);
  size_t dataOff = getU16(rec + 44);
  if (rec[52] != 11 || nsamp > maxOut || dataOff % 64 != 0) {
    return -1;
  }
  static int32_t diffs[MSEED_STEIM2_MAX_SAMPLES_PER_RECORD + 8];
  size_t nd = 0;
  int32_t x0 = 0, xn = 0;
  for (size_t f = 0; dataOff + 64 * (f + 1) <= MSEED_RECORD_LEN; f++) {
    const uint8_t *fr = rec + dataOff + 64 * f;
    uint32_t nib = getU32(fr);
    for (size_t w = 1; w < 16; w++) {
      uint32_t word = getU32(fr + 4 * w);
      if (f == 0 && w == 1) { x0 = (int32_t)word; continue; }
      if (f == 0 && w == 2) { xn = (int32_t)word; continue; }
      unsigned code = (nib >> (30 - 2 * w)) & 3;
      unsigned dnib = word >> 30;
      int cnt = 0, bits = 0;
      if (code == 0) continue;
      if (code == 1) {
        for (int k = 0; k < 4; k++) diffs[nd++] = (int8_t)(word >> (24 - 8 * k));
        continue;
      }
      if (code == 2) {
        if (dnib == 1) { cnt = 1; bits = 30; }
        else if (dnib == 2) { cnt = 2; bits = 15; }
        else if (dnib == 3) { cnt = 3; bits = 10; }
        else return -1;
      } else {
        if (dnib == 0) { cnt = 5; bits = 6; }
        else if (dnib == 1) { cnt = 6; bits = 5; }
        else if (dnib == 2) { cnt = 7; bits = 4; }
        else return -1;
      }
      for (int k = 0; k < cnt; k++) {
        diffs[nd++] = signExtend(word >> (bits * (cnt - 1 - k)), bits);
      }
    }
  }
  if (nd < nsamp || nsamp == 0) {
    return -1;
  }
  out[0] = x0;
  for (size_t i = 1; i < nsamp; i++) {
    out[i] = (int32_t)((uint32_t)out[i - 1] + (uint32_t)diffs[i]);
  }
  return out[nsamp - 1] == xn ? (int)nsamp : -1;
}

static MseedRecordInfo testInfo(int64_t startUs) {
  MseedRecordInfo info = {"XX", "PERM1", "", "EH1", 1, startUs, 5000};
  return info;
}

static uint32_t rngState = 12345;
static int32_t rng(int32_t span) { // deterministic LCG in [-span, span]
  rngState = rngState * 1103515245u + 12345u;
  return (int32_t)((rngState >> 8) % (uint32_t)(2 * span + 1)) - span;
}

// Encodes the whole buffer record by record and checks every record
// decodes back to exactly the input.
static void roundTrip(const int32_t *s, size_t n, size_t *recordsOut) {
  static uint8_t rec[MSEED_RECORD_LEN];
  static int32_t dec[MSEED_STEIM2_MAX_SAMPLES_PER_RECORD];
  size_t pos = 0, records = 0;
  int32_t prev = s[0];
  while (pos < n) {
    size_t used = mseedBuildRecord(rec, testInfo(0), s + pos, n - pos, true, prev);
    TEST_ASSERT_TRUE(used >= 1);
    int got = steim2Decode(rec, dec, MSEED_STEIM2_MAX_SAMPLES_PER_RECORD);
    TEST_ASSERT_EQUAL_INT((int)used, got);
    TEST_ASSERT_EQUAL_INT32_ARRAY(s + pos, dec, used);
    prev = s[pos + used - 1];
    pos += used;
    records++;
  }
  *recordsOut = records;
}

static int32_t bigBuf[20000];

static void test_steim2_roundtrip_quiet_signal() {
  int32_t v = 1000;
  for (size_t i = 0; i < 5000; i++) { v += rng(5); bigBuf[i] = v; } // fits 4-bit diffs
  size_t records;
  roundTrip(bigBuf, 5000, &records);
  // 4-bit packing: ~7 samples/word over 103 words -> well over 500/record.
  TEST_ASSERT_TRUE(records <= 5000 / 500 + 1);
}

static void test_steim2_roundtrip_mixed_amplitudes() {
  int32_t v = 0;
  for (size_t i = 0; i < 20000; i++) {
    int32_t span = (i / 500) % 5 == 0 ? 5 : (i / 500) % 5 == 1 ? 100 : (i / 500) % 5 == 2 ? 3000
                 : (i / 500) % 5 == 3 ? 200000 : 50000000;
    v += rng(span);
    bigBuf[i] = v;
  }
  size_t records;
  roundTrip(bigBuf, 20000, &records);
}

static void test_steim2_extreme_values_and_jumps() {
  const int32_t s[] = {0, 2147483647, -2147483647 - 1, 5, 5, 536870911, -536870912, 0, 1, -1};
  size_t records;
  roundTrip(s, sizeof(s) / sizeof(s[0]), &records); // jumps > 30 bits split records
  TEST_ASSERT_TRUE(records >= 3);
  const int32_t one[] = {-123456};
  roundTrip(one, 1, &records);
  TEST_ASSERT_EQUAL(1, records);
}

static void test_int32_record_and_header() {
  static uint8_t rec[MSEED_RECORD_LEN];
  int32_t s[200];
  for (int i = 0; i < 200; i++) s[i] = i * 1000 - 50000;
  // 2026-02-03T04:05:06.7891 UTC
  int64_t startUs = (daysFromCivilUTC(2026, 2, 3) * 86400LL + 4 * 3600 + 5 * 60 + 6) * 1000000LL + 789100;
  MseedRecordInfo info = {"AB", "STA12", "00", "EH2", 42, startUs, 5000};
  size_t used = mseedBuildRecord(rec, info, s, 200, false, 0);
  TEST_ASSERT_EQUAL(MSEED_INT32_SAMPLES_PER_RECORD, used);
  TEST_ASSERT_EQUAL_MEMORY("000042D STA1200EH2AB", rec, 20);
  TEST_ASSERT_EQUAL_UINT16(2026, getU16(rec + 20));
  TEST_ASSERT_EQUAL_UINT16(34, getU16(rec + 22)); // day of year
  TEST_ASSERT_EQUAL_UINT8(4, rec[24]);
  TEST_ASSERT_EQUAL_UINT8(5, rec[25]);
  TEST_ASSERT_EQUAL_UINT8(6, rec[26]);
  TEST_ASSERT_EQUAL_UINT16(7891, getU16(rec + 28));
  TEST_ASSERT_EQUAL_UINT16(114, getU16(rec + 30));
  TEST_ASSERT_EQUAL_UINT16(200, getU16(rec + 32));
  TEST_ASSERT_EQUAL_UINT16(56, getU16(rec + 44));
  TEST_ASSERT_EQUAL_UINT8(3, rec[52]);
  TEST_ASSERT_EQUAL_INT32(s[113], (int32_t)getU32(rec + 56 + 4 * 113));
}

// Writes a realistic multi-record Steim2 file plus the expected samples,
// for test/obspy_check.py to read back with ObsPy (an independent decoder).
static void test_steim2_write_obspy_fixture() {
  static int32_t s[12000];
  for (size_t i = 0; i < 12000; i++) {
    double t = i / 200.0;
    s[i] = (int32_t)(20000 * sin(2 * M_PI * 1.3 * t) + 300 * sin(2 * M_PI * 17 * t)) + rng(40) +
           (i > 6000 && i < 6400 ? (int32_t)(800000 * sin(2 * M_PI * 4 * t)) : 0);
  }
  FILE *f = fopen("steim2_fixture.mseed", "wb");
  FILE *txt = fopen("steim2_fixture.txt", "w");
  TEST_ASSERT_NOT_NULL(f);
  TEST_ASSERT_NOT_NULL(txt);
  static uint8_t rec[MSEED_RECORD_LEN];
  int64_t startUs = (daysFromCivilUTC(2026, 10, 27) * 86400LL + 12 * 3600) * 1000000LL;
  size_t pos = 0;
  uint32_t seq = 1;
  int32_t prev = s[0];
  while (pos < 12000) {
    MseedRecordInfo info = {"XX", "PERM1", "", "EHZ", seq++, startUs + (int64_t)pos * 5000, 5000};
    size_t used = mseedBuildRecord(rec, info, s + pos, 12000 - pos, true, prev);
    fwrite(rec, 1, MSEED_RECORD_LEN, f);
    prev = s[pos + used - 1];
    pos += used;
  }
  for (size_t i = 0; i < 12000; i++) fprintf(txt, "%ld\n", (long)s[i]);
  fclose(f);
  fclose(txt);
  printf("steim2 fixture: 12000 samples in %lu records (%.1fx vs int32)\n", (unsigned long)(seq - 1),
         (12000.0 / MSEED_INT32_SAMPLES_PER_RECORD) / (seq - 1));
}
