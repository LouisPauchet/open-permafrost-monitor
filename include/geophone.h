#pragma once
#include <Arduino.h>

// Raspberry Shake RS3D digitizer interface - see raspberry_shake.md for the
// reverse-engineered protocol. Receive-only UART1 at 230400 baud, streaming
// back-to-back JSON objects (no newline delimiters) at a continuous ~200
// samples/sec per channel. Must be polled every loop() iteration - the
// digitizer never pauses, so falling behind risks a UART RX overrun.

void initGeophone();
void pollGeophone();

// delay() that keeps servicing the geophone UART throughout, for use in any
// other module's blocking waits (NTRIP/MQTT/modem connect, fix-wait loops)
// instead of a raw delay() that would starve pollGeophone() for the whole
// wait. Doesn't help with delays *inside* WalterModem's own blocking AT
// command calls (mqttConnect/socketDial/httpSend etc.) - those can still
// cause a brief geophone data gap; the RX buffer is sized with headroom for
// that, and it's an accepted tradeoff against a full async rewrite.
//
// Also feeds the ESP32 task watchdog (see main.cpp) on every call, since
// it's already the shared choke point for the app's longer blocking waits
// (modem registration, NTRIP connect) - a single AT command stall inside
// one of those can still exceed the watchdog timeout on its own, but this
// keeps the *waiting loop itself* from being mistaken for a hang.
void geophoneAwareDelay(unsigned long ms);
