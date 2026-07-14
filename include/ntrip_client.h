#pragma once
#include <Arduino.h>

bool ntripConnect();
bool ntripIsConnected();
void ntripPoll(); // forwards available RTCM bytes from the caster into the GNSS UART
void ntripSendGGA(const char *ggaLine); // relay rover position (VRS/network-RTK casters need this)
uint32_t ntripBytesForwarded();
void ntripDisconnect();
