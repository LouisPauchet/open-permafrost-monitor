#pragma once
#include <stddef.h>
#include <stdint.h>

// Allocation-free parser for one complete RS3D JSON object (see
// raspberry_shake.md SS2) - replaces a per-packet ArduinoJson JsonDocument,
// which heap-allocated ~12 times a second, forever, on a station meant to
// run unattended for months. Pure C++ with no Arduino dependency so it can
// be unit-tested on the host (see test/test_native).
//
// Two object shapes are recognized:
//   channel data:  {"MA":..., "CN":"EH1", ..., "SI":5000, "DS":["3F62",...]}
//   cycle marker:  {"MSEC":64500, "LS":0}

static const size_t GEOPHONE_MAX_SAMPLES = 128; // every real packet has exactly 50

struct GeophonePacket {
  bool isChannelData;  // CN + a complete DS array were found
  char channel[4];     // "EH1"/"EH2"/"EH3", NUL-terminated
  uint32_t sampleIntervalUs; // SI, 0 if absent
  size_t count;
  int32_t samples[GEOPHONE_MAX_SAMPLES];

  bool isMarker;       // MSEC was found (and no CN)
  uint32_t msec;
};

// Returns false for anything malformed or truncated (e.g. a UART glitch
// mid-object) - the caller drops it, same as before.
bool geophoneParsePacket(const char *json, size_t len, GeophonePacket *out);
