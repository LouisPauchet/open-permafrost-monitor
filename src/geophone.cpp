#include "geophone.h"
#include "pins.h"
#include "gnss.h"
#include "miniseed_writer.h"
#include "geophone_parse.h"
#include "geophone_timing.h"
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static HardwareSerial GeophoneSerial(1);

// Accumulates one JSON object at a time (brace-depth tracked, not
// newline-delimited - see raspberry_shake.md SS2). A channel-data packet
// with 50 hex samples is a few hundred bytes; 1024 leaves headroom.
static char objBuf[1024];
static size_t objLen = 0;
static int braceDepth = 0;

static GeophonePacket packet; // ~550 bytes - static rather than on the task stack
static GeoClock geoClock;

// The marker's arrival time is only ever jittered by how promptly this
// task drains the UART (<= GEOPHONE_POLL_MS) - a jump larger than this is a
// real discontinuity (GNSS re-anchor, lost marker), not jitter.
static const int64_t CLOCK_STEP_THRESHOLD_US = 50000;

static void handleCompleteObject() {
  if (!geophoneParsePacket(objBuf, objLen, &packet)) {
    return; // corrupt/truncated fragment (e.g. a UART glitch) - drop it
  }

  if (packet.isMarker) {
    if (!gnssHasEpochAnchor()) {
      return; // no absolute time yet (before the first-ever GNSS fix)
    }
    int64_t arrivalUs = (int64_t)gnssEpochMillisNow() * 1000;
    if (geoClockOnMarker(&geoClock, packet.msec, arrivalUs, CLOCK_STEP_THRESHOLD_US)) {
      Serial.printf("Geophone: clock (re)anchored at MSEC=%lu\n", (unsigned long)packet.msec);
    }
    return;
  }

  if (!geoClockReady(&geoClock)) {
    return; // no time base yet - drop until the first marker after a GNSS fix
  }
  miniseedAddPacket(packet.channel, packet.samples, packet.count, packet.sampleIntervalUs,
                    geoClock.extMs, geoClockRoundStartUs(&geoClock));
}

static void pollGeophone() {
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

// --- Dedicated task ---
//
// pollGeophone() used to run from loopTask only, so every blocking call
// there (WalterModem's socketDial/setOpState/httpSend, registration waits)
// stopped draining the UART. The 8KB RX buffer only covers ~1s of the
// digitizer's ~6-8 KB/s, so any longer stall silently lost samples. On its
// own task nothing loopTask does can starve it.
static const uint32_t GEOPHONE_POLL_MS = 10;
// Full MiniSEED records are queued the moment they fill (every ~0.6 s
// uncompressed, a few seconds with Steim2) - this only bounds how long a
// *partial* record can sit in RAM. Each flush cuts a partially filled
// record per channel, which costs Steim2 density, so it's not shorter.
static const uint32_t MINISEED_FLUSH_INTERVAL_MS = 30000;

static void geophoneTaskFn(void *) {
  uint32_t lastFlushMs = millis();
  for (;;) {
    pollGeophone();
    if (millis() - lastFlushMs >= MINISEED_FLUSH_INTERVAL_MS) {
      lastFlushMs = millis();
      miniseedFlush();
    }
    vTaskDelay(pdMS_TO_TICKS(GEOPHONE_POLL_MS));
  }
}

void initGeophone() {
  // Continuous 230400 baud stream, but actual average throughput is only
  // ~6-8 KB/s (raspberry_shake.md: ~1.5-2KB per 250ms cycle across all 3
  // channels), well under the line rate - 8KB of RX buffer is far more than
  // one GEOPHONE_POLL_MS gap needs, and still covers the odd longer stall
  // (e.g. SD writer or modem task holding core 0).
  GeophoneSerial.setRxBufferSize(8192);
  GeophoneSerial.begin(GEOPHONE_BAUD, SERIAL_8N1, GEOPHONE_RX_PIN, -1); // receive-only

  // Core 0 with the SD writer (priority 1) and modem worker (priority 1) -
  // above both so a long SD write or AT exchange can't delay draining.
  // Core 1 carries loopTask and WalterModem's own tasks.
  xTaskCreatePinnedToCore(geophoneTaskFn, "geophoneTask", 6144, NULL, 2, NULL, 0);
}

void geophoneAwareDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    esp_task_wdt_reset();
    pollGNSS(); // keeps PPK UBX flowing to SD through NTRIP/registration waits
    delay(10);
  } while (millis() - start < ms);
}
