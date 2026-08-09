#include "geophone.h"
#include "pins.h"
#include "miniseed_writer.h"
#include <ArduinoJson.h>
#include <stdlib.h>

static HardwareSerial GeophoneSerial(1);

// Accumulates one JSON object at a time (brace-depth tracked, not
// newline-delimited - see raspberry_shake.md SS2). A channel-data packet
// with 50 hex samples is a few hundred bytes; 1024 leaves headroom.
static char objBuf[1024];
static size_t objLen = 0;
static int braceDepth = 0;

static void handleCompleteObject() {
  JsonDocument doc;
  if (deserializeJson(doc, objBuf, objLen) != DeserializationError::Ok) {
    return; // corrupt/truncated fragment (e.g. a UART glitch) - drop it
  }

  const char *channel = doc["CN"] | (const char *)nullptr;
  if (channel == nullptr) {
    return; // not a channel-data packet (e.g. the {"MSEC":...,"LS":0} cycle marker)
  }

  // DS: array of signed 32-bit samples, hex-encoded with no zero-padding.
  // Must parse with strtoul + reinterpret as int32_t (not a signed parser),
  // or negative values (which print all 8 hex digits) saturate incorrectly.
  for (JsonVariantConst v : doc["DS"].as<JsonArrayConst>()) {
    const char *hex = v.as<const char *>();
    if (hex == nullptr) {
      continue;
    }
    int32_t sample = (int32_t)strtoul(hex, nullptr, 16);
    miniseedAddSample(channel, sample);
  }
}

void initGeophone() {
  // Continuous 230400 baud stream, but actual average throughput is only
  // ~6-8 KB/s (raspberry_shake.md: ~1.5-2KB per 250ms cycle across all 3
  // channels), well under the line rate - an 8KB RX buffer gives real
  // headroom against the brief stalls that happen elsewhere in loop()
  // (network operations) without dropping bytes.
  GeophoneSerial.setRxBufferSize(8192);
  GeophoneSerial.begin(GEOPHONE_BAUD, SERIAL_8N1, GEOPHONE_RX_PIN, -1); // receive-only
}

void geophoneAwareDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    pollGeophone();
    delay(10);
  } while (millis() - start < ms);
}

void pollGeophone() {
  while (GeophoneSerial.available() > 0) {
    char c = GeophoneSerial.read();

    if (c == '{') {
      braceDepth++;
      if (braceDepth == 1) {
        objLen = 0; // start of a new top-level object
      }
    }

    if (braceDepth > 0 && objLen < sizeof(objBuf) - 1) {
      objBuf[objLen++] = c;
    }

    if (c == '}' && braceDepth > 0) {
      braceDepth--;
      if (braceDepth == 0) {
        objBuf[objLen] = '\0';
        handleCompleteObject();
      }
    }
  }
}
