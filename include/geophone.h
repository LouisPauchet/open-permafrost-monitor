#pragma once
#include <Arduino.h>

// Raspberry Shake RS3D digitizer interface - see raspberry_shake.md for the
// reverse-engineered protocol. Receive-only UART1 at 230400 baud, streaming
// back-to-back JSON objects (no newline delimiters) at a continuous ~200
// samples/sec per channel.
//
// initGeophone() starts a dedicated task that drains the UART, parses
// packets and feeds the MiniSEED writer (including its periodic flush) -
// nothing else needs to poll it. Call after initMiniseedWriter(), and only
// once normal operation starts (not in field service mode, where the SD
// card is exposed over USB and must not be written).
void initGeophone();

// Watchdog-feeding delay for any other module's blocking waits
// (NTRIP/MQTT/modem connect, fix-wait loops) - keeps the *waiting loop
// itself* from being mistaken for a hang by the task watchdog (see
// main.cpp). The geophone no longer depends on it: its own task keeps
// running regardless. It does service the GNSS UART (pollGNSS()), so a long
// wait mid-burst doesn't overflow it and lose PPK data. loopTask only -
// pollGNSS() isn't safe to call from two tasks. Name kept from when it
// polled the geophone.
void geophoneAwareDelay(unsigned long ms);
