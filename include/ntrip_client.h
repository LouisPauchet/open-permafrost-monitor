#pragma once
#include <Arduino.h>

// Retries internally on failure (5s for a 401 - often transient on this
// caster, 10s otherwise), a bounded number of times, before giving up and
// returning false for this fix cycle.
bool ntripConnect();
bool ntripIsConnected();
void ntripPoll(); // forwards available RTCM bytes from the caster into the GNSS UART

// Relays rover position (VRS/network-RTK casters need this). Returns false
// (without sending) if a receive is currently in flight rather than risk
// colliding with it - the caller should not treat that as "sent, wait for
// the next interval" but retry again soon.
bool ntripSendGGA(const char *ggaLine);

uint32_t ntripBytesForwarded();
void ntripDisconnect();
