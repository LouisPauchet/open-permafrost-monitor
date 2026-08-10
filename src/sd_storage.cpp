#include "sd_storage.h"
#include "pins.h"
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

static bool available = false;

bool sdStorageInit() {
  pinMode(SD_POWER_EN_PIN, OUTPUT);
  digitalWrite(SD_POWER_EN_PIN, LOW); // active LOW enable; held on continuously -
                                       // geophone logging never stops, so unlike
                                       // the GNSS there's no per-cycle power gating.
  delay(50); // let the rail stabilize before the card sees clock/commands

  pinMode(SD_CARD_DETECT_PIN, INPUT); // sdCheckCardPresence() reads this every loop()

  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);

  // Arduino's SD_MMC.begin() defaults to SDMMC_FREQ_HIGHSPEED (40MHz,
  // BOARD_MAX_SDMMC_FREQ in SD_MMC.h) unless told otherwise. This carrier
  // board's SD wiring is explicitly provisional (see platformio.ini) and
  // couldn't reliably mount at all at 40MHz, nor sustain real writes at
  // 20MHz (confirmed on live hardware: mount succeeded but the first write
  // failed with "sdmmc_send_cmd returned 0x109/0x107", CRC/timeout errors
  // typical of marginal signal integrity) - 10MHz has run clean since.
  static const int SD_CLOCK_FREQ_KHZ = 10000;
  if (!SD_MMC.begin("/sdcard", true /* 1-bit mode */, false /* format_if_mount_failed */,
                     SD_CLOCK_FREQ_KHZ)) {
    Serial.println(F("SD: mount failed."));
    available = false;
    return false;
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println(F("SD: no card detected."));
    available = false;
    return false;
  }

  Serial.printf("SD: mounted, %llu MB\n", SD_MMC.cardSize() / (1024ULL * 1024ULL));
  available = true;

  // Fixed layout: /data for CSV-style logs, /data/gnss for one UBX file per
  // GNSS burst, /data/geophones for the continuous MiniSEED stream. mkdir()
  // only creates one level and isn't recursive, so /data has to exist
  // before its children; each call is idempotent (false on an
  // already-existing dir isn't an error, only matters on a fresh card).
  SD_MMC.mkdir("/data");
  SD_MMC.mkdir("/data/gnss");
  SD_MMC.mkdir("/data/geophones");

  return true;
}

bool sdStorageAvailable() {
  return available;
}

// Card-detect polarity confirmed on live hardware: HIGH while a card is
// seated, LOW once it's pulled. Mirrors checkFieldModeTrigger()'s
// debounce-then-restart shape in field_service.cpp - a clean reboot re-runs
// sdStorageInit() (and every other subsystem that only initializes once at
// boot) rather than trying to hand-roll an in-place re-mount that would
// also need every open-file/rotation-state owner (gnss.cpp,
// miniseed_writer.cpp, position_log.cpp) reset correctly too.
// restartAttempted caps this to one restart per detected insertion: if the
// card still won't mount after that (a real fault, not just a hot-plug
// timing thing), the station keeps running SD-less instead of
// reboot-looping forever.
void sdCheckCardPresence() {
  static unsigned long presentSinceMs = 0;
  static unsigned long absentSinceMs = 0;
  static bool restartAttempted = false;
  const unsigned long DEBOUNCE_MS = 200;

  bool present = digitalRead(SD_CARD_DETECT_PIN) == HIGH;

  if (present) {
    absentSinceMs = 0;
    if (!available && !restartAttempted) {
      if (presentSinceMs == 0) {
        presentSinceMs = millis();
      } else if (millis() - presentSinceMs >= DEBOUNCE_MS) {
        restartAttempted = true;
        Serial.println(F("SD: card detected - rebooting to mount it cleanly."));
        Serial.flush();
        delay(100);
        ESP.restart();
      }
    }
  } else {
    presentSinceMs = 0;
    if (available) {
      if (absentSinceMs == 0) {
        absentSinceMs = millis();
      } else if (millis() - absentSinceMs >= DEBOUNCE_MS) {
        Serial.println(F("SD: card removed - marking storage unavailable."));
        available = false;
        restartAttempted = false; // a freshly (re)inserted card gets its own attempt
      }
    }
  }
}

void sdBuildDailyPath(char *buf, size_t bufSize, const char *prefix,
                       const char *dateYYYYMMDD, const char *ext) {
  snprintf(buf, bufSize, "/%s_%s.%s", prefix, dateYYYYMMDD, ext);
}

File sdOpenForAppend(const char *path) {
  if (!available) {
    return File();
  }
  File f = SD_MMC.open(path, FILE_APPEND);
  if (!f) {
    Serial.printf("SD: failed to open %s for append.\n", path);
    // A card that failed to mount at all is caught by sdStorageInit()'s own
    // check, but one that mounted fine and then degraded mid-session (loose
    // connection, marginal signal integrity - see the clock-speed comment
    // in sdStorageInit()) previously kept getting retried every single
    // caller, every cycle, forever - each
    // retry against an already-wedged card is exactly the kind of unbounded
    // SD_MMC I/O that's landed a task-watchdog abort on live hardware
    // (confirmed: the boot immediately after one such abort failed to mount
    // at all, "sdmmc_host_wait_for_event returned 0x107"). Latching false
    // here after the first real failure stops hammering a card that's
    // already in a bad state - the tradeoff is losing the rest of this boot's
    // logging if it was a one-off glitch rather than genuine degradation,
    // preferable to repeatedly risking another stall.
    available = false;
  }
  return f;
}

// --- Async stream writer (see sd_storage.h for why) ---

namespace {

struct StreamRing {
  uint8_t buf[4096]; // comfortably covers a worst-case ~2200-byte UBX RAWX
                      // frame plus margin; RTCM only ever needs a fraction
                      // of this between 50ms drains, sized the same for
                      // simplicity, not because it needs to be this big.
  size_t len = 0;
  SemaphoreHandle_t mutex = nullptr;
};

struct StreamState {
  File file;
  char pendingPath[64] = "";
  char pendingHeader[128] = ""; // empty = no header requested for this open
  volatile bool openPending = false;
  volatile bool closePending = false;
};

StreamRing g_ring[SD_STREAM_COUNT];
StreamState g_stream[SD_STREAM_COUNT];
TaskHandle_t g_writerTask = nullptr;

// Runs on its own task - every blocking SD_MMC call (open/write/flush/
// close) happens here and only here, so a wedged card blocks this task,
// not whichever one called sdStreamWrite()/Open()/Close().
void sdWriterTaskFn(void *) {
  const TickType_t drainPeriod = pdMS_TO_TICKS(50);
  unsigned long lastFlushMs = 0;
  uint8_t local[sizeof(StreamRing::buf)]; // task-owned scratch, not on the stack per-iteration

  for (;;) {
    for (int s = 0; s < SD_STREAM_COUNT; s++) {
      StreamState &st = g_stream[s];

      if (st.openPending) {
        st.openPending = false;
        if (st.file) {
          st.file.close();
        }
        st.file = sdOpenForAppend(st.pendingPath);
        Serial.printf("SD writer: stream %d open %s -> %s\n", s, st.pendingPath,
                      st.file ? "ok" : "FAILED");
        if (st.file && st.pendingHeader[0] != '\0' && st.file.size() == 0) {
          st.file.printf("%s\n", st.pendingHeader);
        }
      }

      StreamRing &ring = g_ring[s];
      xSemaphoreTake(ring.mutex, portMAX_DELAY);
      size_t n = ring.len;
      if (n > 0) {
        memcpy(local, ring.buf, n);
        ring.len = 0;
      }
      xSemaphoreGive(ring.mutex);

      if (n > 0 && st.file) {
        st.file.write(local, n);
      }

      if (st.closePending) {
        st.closePending = false;
        if (st.file) {
          st.file.close();
        }
      }
    }

    // Periodic flush (not per-stream-close, which already flushes via
    // close()) so a crash mid-burst loses at most this interval's worth of
    // data rather than everything since the file was opened - same reason
    // main.cpp already does this for MiniSEED, just relocated off the loop
    // task along with everything else here.
    if (millis() - lastFlushMs >= 5000) {
      lastFlushMs = millis();
      for (int s = 0; s < SD_STREAM_COUNT; s++) {
        if (g_stream[s].file) {
          g_stream[s].file.flush();
        }
      }
    }

    vTaskDelay(drainPeriod);
  }
}

} // namespace

void sdWriterInit() {
  for (int s = 0; s < SD_STREAM_COUNT; s++) {
    g_ring[s].mutex = xSemaphoreCreateMutex();
  }
  // Core 0, deliberately: loopTask and WalterModem's own queueProcessing/
  // eventProcessing tasks are all already on core 1 (see modem.cpp) - no
  // reason to add a fourth task competing there when core 0 is otherwise
  // idle. Stack sized for the 4KB scratch buffer above plus SD_MMC/FatFs's
  // own internal call depth with real margin.
  xTaskCreatePinnedToCore(sdWriterTaskFn, "sdWriterTask", 10240, NULL, 1, &g_writerTask, 0);
}

void sdStreamOpen(SdStream stream, const char *path, const char *headerIfEmpty) {
  StreamState &st = g_stream[stream];
  strncpy(st.pendingPath, path, sizeof(st.pendingPath) - 1);
  st.pendingPath[sizeof(st.pendingPath) - 1] = '\0';
  if (headerIfEmpty) {
    strncpy(st.pendingHeader, headerIfEmpty, sizeof(st.pendingHeader) - 1);
    st.pendingHeader[sizeof(st.pendingHeader) - 1] = '\0';
  } else {
    st.pendingHeader[0] = '\0';
  }
  st.openPending = true;
}

void sdStreamWrite(SdStream stream, const uint8_t *data, size_t len) {
  StreamRing &ring = g_ring[stream];
  xSemaphoreTake(ring.mutex, portMAX_DELAY); // just a bounded memcpy - never blocks on SD I/O
  size_t space = sizeof(ring.buf) - ring.len;
  size_t n = len < space ? len : space; // drop the tail if the writer task can't keep up
  if (n > 0) {
    memcpy(ring.buf + ring.len, data, n);
    ring.len += n;
  }
  xSemaphoreGive(ring.mutex);
}

void sdStreamClose(SdStream stream) {
  g_stream[stream].closePending = true;
}
