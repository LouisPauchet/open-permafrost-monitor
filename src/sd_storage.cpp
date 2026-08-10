#include "sd_storage.h"
#include "pins.h"
#include <SD_MMC.h>

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
