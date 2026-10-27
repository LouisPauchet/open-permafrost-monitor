#pragma once
#include <Arduino.h>

// Minimal, from-scratch MiniSEED writer: fixed 512-byte SEED data records,
// uncompressed 32-bit integer encoding (format code 3) - not Steim1/2. One
// combined daily file holds interleaved records from all three geophone
// channels. Record start times come from the digitizer's own sample clock,
// mapped onto GNSS UTC - see geophone_timing.h. Called only from the
// geophone task (geophone.cpp).

void initMiniseedWriter();

// Called once per decoded channel packet. channel is "EH1"/"EH2"/"EH3" (the
// digitizer's CN field), sampleIntervalUs its SI field (0 = assume 200 Hz),
// roundMs the digitizer round it belongs to (GeoClock::extMs) and
// roundStartUs the UTC epoch us that round started at
// (geoClockRoundStartUs()).
void miniseedAddPacket(const char *channel, const int32_t *samples, size_t n,
                       uint32_t sampleIntervalUs, int64_t roundMs, int64_t roundStartUs);

// Flushes any partially-filled per-channel record buffers to disk. Call
// periodically so data isn't held in RAM indefinitely waiting for a
// buffer to fill (each channel only gets one sample every 5ms at 200Hz,
// so a 114-sample buffer can take over half a second to fill).
void miniseedFlush();
