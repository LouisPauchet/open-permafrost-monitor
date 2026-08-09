#pragma once
#include <Arduino.h>

// Minimal, from-scratch MiniSEED writer: fixed 512-byte SEED data records,
// uncompressed 32-bit integer encoding (format code 3) - not Steim1/2. One
// combined daily file holds interleaved records from all three geophone
// channels. Every record's start time comes from GNSS UTC (gnss.h) - the
// digitizer itself has no absolute clock (see raspberry_shake.md SS4).

void initMiniseedWriter();

// Called once per decoded sample. channel is "EH1"/"EH2"/"EH3" (the
// digitizer's CN field, see geophone.h).
void miniseedAddSample(const char *channel, int32_t sample);

// Flushes any partially-filled per-channel record buffers to disk. Call
// periodically so data isn't held in RAM indefinitely waiting for a
// buffer to fill (each channel only gets one sample every 5ms at 200Hz,
// so a 114-sample buffer can take over half a second to fill).
void miniseedFlush();
