#pragma once
#include <Arduino.h>

// Retries internally on failure (5s for a 401 - often transient on this
// caster, 10s otherwise), up to maxAttempts times (max 5), before giving up
// and returning false. A mid-burst reconnect (see main.cpp) uses 1.
bool ntripConnect(int maxAttempts = 5);
bool ntripIsConnected();
void ntripPoll(); // forwards available RTCM bytes from the caster into the GNSS UART

// Relays rover position (VRS/network-RTK casters need this). Returns false
// (without sending) if a receive or the previous send is still in flight
// rather than risk colliding with it - the caller should not treat that as
// "sent, wait for the next interval" but retry again soon.
bool ntripSendGGA(const char *ggaLine);

uint32_t ntripBytesForwarded();
void ntripDisconnect();
