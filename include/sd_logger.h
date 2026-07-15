#pragma once
#include <Arduino.h>

// Raw GNSS byte logging to SD card, for later PPK/PPP post-processing to
// reach higher accuracy than live RTK (useful when only RTK FLOAT was
// achieved in the field). Degrades to a harmless no-op if SD_CS_PIN isn't
// defined in pins.h yet or no card is present - safe to call unconditionally.

bool initSDLogger();      // returns false if no card/pins configured; never blocks
bool sdLoggerAvailable();
void sdLogRawByte(uint8_t b); // called once per raw byte from the F9P UART
void closeSDLogger();
