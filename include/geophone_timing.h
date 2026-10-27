#pragma once
#include <stdint.h>

// Sample timing for the MiniSEED stream - pure, header-only, unit-tested on
// the host (test/test_native).
//
// Records used to be stamped with "now" at whichever moment the first
// sample of a record happened to be processed. Samples arrive in 50-sample
// (250 ms) packets, so that put every record start anywhere from 0 to
// 250 ms off, plus however long loopTask had been stalled - and adjacent
// records overlapped or gapped at random.
//
// Instead, time is derived from the digitizer's own clock: the cycle
// marker {"MSEC":...} that closes every 3-channel round advances by exactly
// 250 per round, and the round that follows marker M covers digitizer time
// [M, M+250) ms. GeoClock maps that counter onto UTC (GNSS-extrapolated,
// see gnssEpochMillisNow()) with an offset that slowly tracks
// arrival-time jitter and the drift between the two clocks, and is
// hard-reset only on a real discontinuity. All three channels share one
// mapping, so they stay aligned with each other.
//
// Known, uncalibrated constant: the latency between the digitizer sampling
// and the marker reaching us (UART + digitizer buffering) is folded into
// the offset - same systematic error the old arrival-time stamping had,
// minus the jitter.

struct GeoClock {
  bool haveMarker = false;
  uint32_t lastMsec = 0;
  int64_t extMs = 0;      // MSEC unwrapped to 64 bits
  bool haveOffset = false;
  int64_t offsetUs = 0;   // UTC epoch us = extMs * 1000 + offsetUs
};

// Feeds one cycle marker. arrivalEpochUs is the UTC time it was received.
// Returns true if the offset was hard-reset (first marker, or the estimate
// jumped by more than stepThresholdUs) - worth logging, should be rare.
inline bool geoClockOnMarker(GeoClock *c, uint32_t msec, int64_t arrivalEpochUs,
                             int64_t stepThresholdUs) {
  if (c->haveMarker) {
    c->extMs += (int32_t)(msec - c->lastMsec); // wrap-safe unwrap of the 32-bit counter
  } else {
    c->extMs = msec;
    c->haveMarker = true;
  }
  c->lastMsec = msec;

  int64_t measured = arrivalEpochUs - c->extMs * 1000;
  int64_t diff = measured - c->offsetUs;
  if (!c->haveOffset || diff > stepThresholdUs || diff < -stepThresholdUs) {
    c->offsetUs = measured;
    c->haveOffset = true;
    return true;
  }
  c->offsetUs += diff / 64; // smooth out poll/arrival jitter, follow slow drift
  return false;
}

inline bool geoClockReady(const GeoClock *c) {
  return c->haveMarker && c->haveOffset;
}

// UTC epoch us of the first sample of the round following the last marker.
inline int64_t geoClockRoundStartUs(const GeoClock *c) {
  return c->extMs * 1000 + c->offsetUs;
}

// Per-channel continuity: keeps a channel's sample clock running at exactly
// the nominal interval from packet to packet, and only snaps it to the
// round's start when the two disagree by more than thresholdUs (a lost
// packet, or accumulated drift). Returns the time of this packet's first
// sample; *resynced reports a snap.
inline int64_t geoChannelPacketStartUs(int64_t *nextUs, bool *haveNext, int64_t roundStartUs,
                                       int64_t thresholdUs, bool *resynced) {
  int64_t diff = roundStartUs - *nextUs;
  *resynced = !*haveNext || diff > thresholdUs || diff < -thresholdUs;
  if (*resynced) {
    *nextUs = roundStartUs;
    *haveNext = true;
  }
  return *nextUs;
}
