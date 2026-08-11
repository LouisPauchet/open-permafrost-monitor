#pragma once
#include <Arduino.h>

// Establishes this station's fixed reference (benchmark) position once, by
// averaging GNSS fixes over a commissioning window (device_config.h:
// reference_window_s), then reports every later fix as a local East/North/
// Up displacement from that benchmark - the actual measurement a permafrost
// station cares about (ground movement), not absolute position on its own.
//
// Averaging over a real window (not just taking the first fix) cancels out
// multipath/atmospheric error the same way a real static-GNSS survey does -
// a single fix, even RTK FIXED, still carries several mm-cm of scatter that
// would otherwise get baked permanently into the benchmark.

void referencePositionInit(); // load persisted state - call once from setup()

// Starts (or restarts) a commissioning run: clears any previous in-progress
// average and begins accumulating fixes toward a fresh reference position.
// Does NOT reboot itself - the field-service menu that calls this does, so
// the main loop's commissioning-aware scheduling (see main.cpp) takes over
// cleanly from a known state.
void referencePositionStartCommissioning();

bool referencePositionIsCommissioning();
bool referencePositionIsLocked();

// Feeds one fix into the reference average (only while commissioning is
// active; a no-op once locked or if commissioning was never started).
// Pass only fixes you'd trust as a survey observation - callers should
// gate this on qualityOk, not just gnssHasFix().
void referencePositionUpdate(double lat, double lon, double alt);

// Local tangent-plane displacement (WGS84 ellipsoid, meters) of (lat, lon,
// alt) from the locked reference position. Returns false (outputs
// untouched) if there isn't one yet.
bool referencePositionDisplacement(double lat, double lon, double alt, float *northM,
                                    float *eastM, float *upM);

// For status prints (main.cpp) and the field-service menu.
uint32_t referencePositionElapsedS();
uint32_t referencePositionSampleCount();
bool referencePositionGet(double *lat, double *lon, double *alt); // the locked reference itself
