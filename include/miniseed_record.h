#pragma once
#include <stddef.h>
#include <stdint.h>

// Builds one fixed 512-byte MiniSEED (SEED v2.4) data record: fixed header
// + blockette 1000 + samples, big-endian throughout. Pure, no Arduino
// dependency - unit-tested on the host (test/test_native), including an
// ObsPy read-back of the encoder's output (test/obspy_check.py).
//
// Two encodings:
// - uncompressed 32-bit integers (format 3): always exactly
//   MSEED_INT32_SAMPLES_PER_RECORD samples (fewer only for the last one);
// - Steim2 (format 11): lossless difference compression, typically 2-4x
//   denser for geophone data, so the number of samples per record varies
//   with how much the signal changes.

static const size_t MSEED_RECORD_LEN = 512;
static const size_t MSEED_INT32_SAMPLES_PER_RECORD = 114; // (512 - 48 header - 8 blockette 1000) / 4
// 7 Steim frames of 16 words; frame 0 spends 3 words on the nibble word,
// X0 and Xn, the others 1 on the nibble word. At most 7 differences per
// word.
static const size_t MSEED_STEIM2_MAX_SAMPLES_PER_RECORD = (13 + 6 * 15) * 7;

struct MseedRecordInfo {
  const char *network;  // up to 2 chars
  const char *station;  // up to 5 chars
  const char *location; // up to 2 chars, may be ""
  const char *channel;  // 3 chars
  uint32_t sequence;    // record sequence number (mod 1e6)
  int64_t startUs;      // UTC epoch us of samples[0]
  uint32_t intervalUs;  // sample interval
};

// Encodes as many of samples[0..n) as fit into rec (MSEED_RECORD_LEN bytes)
// and returns how many were used: at least 1 whenever n >= 1. prevSample is
// the sample immediately before samples[0] (Steim2's first difference
// refers to it; decoders don't rely on it, since X0 is stored explicitly).
// A Steim2 record also ends early before any sample whose difference from
// its predecessor doesn't fit Steim2's 30-bit range, so such a jump simply
// starts the next record.
size_t mseedBuildRecord(uint8_t *rec, const MseedRecordInfo &info, const int32_t *samples, size_t n,
                        bool steim2, int32_t prevSample);
