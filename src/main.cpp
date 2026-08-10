#include <Arduino.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <SD_MMC.h> // for enumerating /data/gnss/*.ubx and *.rtcm in runDailyUploadIfNeeded()
#include <string.h>

#include "device_config.h"
#include "field_service.h"
#include "gnss.h"
#include "geophone.h"
#include "miniseed_writer.h"
#include "sd_storage.h"
#include "battery.h"
#include "position_log.h"
#include "modem.h"
#include "ntrip_client.h"
#include "mqtt_client.h"
#include "upload_client.h"

// The default 8KB Arduino loop-task stack is too small for a TLS handshake
// (WalterModem/mbedTLS) and crashes with a stack overflow at connect time.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// Continuous 24/7 geophone capture means the CPU can never deep-sleep (see
// the plan) - loop() now runs forever, with each subsystem scheduled off
// its own millis() timer instead of one wake/measure/sleep cycle per boot.

// CPOS documents a GGA resend interval of 10s - this is a network-RTK/VRS
// mount (confirmed with the user), which genuinely needs GGA to know where
// to center corrections, unlike a single physical base station that would
// send the same stream regardless.
static const unsigned long GGA_RESEND_INTERVAL_MS = 10000;
// This NTRIP mount is network RTK/VRS (confirmed with the user) - it
// genuinely needs GGA to know where to center corrections, unlike a
// single physical base station that would send the same stream
// regardless. A send that fails (discovered asynchronously, after
// modemSocketSend() already reported "queued OK") gets retried this soon
// instead of waiting a full GGA_RESEND_INTERVAL_MS - see modemLastSendFailed().
static const unsigned long GGA_FAILURE_RETRY_MS = 1500;
static const unsigned long GST_ENABLE_RETRY_INTERVAL_MS = 5000;
static const unsigned long MINISEED_FLUSH_INTERVAL_MS = 5000;
// Not user-configurable (kept out of the config menu per the "as few
// parameters as possible" preference) - how often to wake the modem for a
// batched MQTT publish, independent of the (usually shorter) GNSS fix
// interval, so the modem doesn't need a fresh MQTT session every fix.
static const unsigned long TELEMETRY_BATCH_INTERVAL_MS = 3600000UL; // 1 hour

// Safety net for unattended field operation: if loop() (or anything it
// blocks in) doesn't feed this for WATCHDOG_TIMEOUT_S, the ESP32 reboots
// itself rather than sitting stuck until someone physically power-cycles
// it.
//
// Originally set to 210s - above WalterModem's own internal AT command
// timeout (180s, WALTER_MODEM_CMD_TIMEOUT_MS) - on the assumption that a
// wedged command would eventually resolve itself via that timeout, with
// our watchdog only as a last-resort backstop. Live hardware disproved
// that: a socket receive got stuck with zero "Command time-out" warnings
// (which the library logs when its own 180s timeout fires) for the entire
// 215s until our watchdog reset the device - i.e. the stuck command never
// got attributed to _curCmd for the library's own timeout logic to see, so
// nothing internal was ever going to recover it. There's no benefit to
// waiting anywhere near 180s for a recovery path that doesn't apply to
// this failure mode; shortened so a stall costs far less station uptime.
static const uint32_t WATCHDOG_TIMEOUT_S = 60;

static unsigned long lastFixCycleMs = 0;
static unsigned long lastTelemetryMs = 0;
static unsigned long lastMiniseedFlushMs = 0;
static char lastUploadedDate[9] = "";
static bool firstFixCycleDone = false;

// CPOS (the network RTK service behind this NTRIP mount) documents that a
// client must already have a regular fix - autonomous is fine - before
// opening an NTRIP session: the correction stream is centered on whatever
// position the first GGA carries, so connecting earlier just spends the
// window sending a quality=0/empty GGA the caster has nothing to center
// corrections on. The modem has nothing else to do during this wait, so
// deferring the connect costs nothing.
static void waitForInitialFix() {
  Serial.printf("Waiting up to %u s for an initial fix before starting NTRIP...\n",
                deviceConfig.fix_timeout_s);
  unsigned long start = millis();
  unsigned long lastGstEnableRetry = 0;
  while (!gnssHasFix() && millis() - start < deviceConfig.fix_timeout_s * 1000UL) {
    esp_task_wdt_reset();
    pollGeophone();
    pollGNSS();
    if (millis() - lastGstEnableRetry >= GST_ENABLE_RETRY_INTERVAL_MS) {
      lastGstEnableRetry = millis();
      gnssEnableGST();
    }
    delay(20);
  }
}

static void runGnssFixCycle() {
  gnssWakeup();
  geophoneAwareDelay(500);
  initGNSS();

  waitForInitialFix();

  // PPK's raw-observation floor (see below) is anchored to this moment, not
  // to gnssWakeup() - it's specifically "record for X more seconds once we
  // have a fix", not "keep the module awake X seconds total".
  bool hadInitialFix = gnssHasFix();
  unsigned long initialFixMs = millis();

  bool ntripUp = false;
  if (hadInitialFix) {
    if (modemConnect()) {
      ntripUp = ntripConnect();
    }
  } else {
    Serial.println(F("No initial fix within timeout - skipping NTRIP this cycle."));
  }

  // Per CPOS: first GGA immediately on connect, then every
  // GGA_RESEND_INTERVAL_MS - not delayed by a full interval like a resend.
  unsigned long nextGgaAttemptMs = 0;
  if (ntripUp) {
    if (ntripSendGGA(gnssLastGGA())) {
      nextGgaAttemptMs = millis() + GGA_RESEND_INTERVAL_MS;
    }
  }

  Serial.printf("Waiting up to %u s more for a fix with horizontal accuracy <= %.3f m"
                " (recording at least %u s for PPK)...\n",
                deviceConfig.fix_timeout_s, deviceConfig.good_accuracy_m,
                (unsigned)deviceConfig.ppk_min_duration_s);

  unsigned long fixWaitStart = millis();
  unsigned long lastGstEnableRetry = 0;
  unsigned long lastRxmRtcmDiagResend = 0;
  unsigned long lastStatusPrint = 0;
  bool qualityOk = false;

  // Runs until BOTH: the RTK/NTRIP side is done (good accuracy reached, or
  // fix_timeout_s gave up) AND the PPK raw-observation floor since the
  // initial fix has been met - whichever of the two takes longer. A fast RTK
  // convergence (seen as early as ~20s on live hardware) would otherwise
  // stop recording well short of ppk_min_duration_s; a slow/failed
  // convergence already usually exceeds it, but isn't guaranteed to
  // (fix_timeout_s and ppk_min_duration_s are independently configurable).
  while (millis() - fixWaitStart < deviceConfig.fix_timeout_s * 1000UL ||
         (hadInitialFix && millis() - initialFixMs < deviceConfig.ppk_min_duration_s * 1000UL)) {
    esp_task_wdt_reset();
    pollGeophone();
    pollGNSS();

    if (ntripUp) {
      ntripPoll();

      unsigned long now = millis();

      // A prior send can fail *after* modemSocketSend() already reported
      // "queued OK" - the actual result only arrives later, asynchronously.
      // Checked every iteration so a real failure gets noticed quickly and
      // pulls the next attempt in, rather than waiting out the full
      // interval as if nothing had gone wrong.
      if (modemLastSendFailed()) {
        nextGgaAttemptMs = now + GGA_FAILURE_RETRY_MS;
      }

      if (now >= nextGgaAttemptMs) {
        if (ntripSendGGA(gnssLastGGA())) {
          nextGgaAttemptMs = now + GGA_RESEND_INTERVAL_MS;
        } else {
          // Deferred (a receive was in flight) rather than sent at all -
          // retry next iteration, not after a fresh full interval.
          nextGgaAttemptMs = now;
        }
      }
    }

    // The GST-enable command can land before the F9P is ready to process
    // it, especially right after an EXTINT wake from backup mode - keep
    // retrying until GST sentences are actually flowing.
    if (!gnssHasAccuracyEstimate() && millis() - lastGstEnableRetry >= GST_ENABLE_RETRY_INTERVAL_MS) {
      lastGstEnableRetry = millis();
      gnssEnableGST();
    }

    // Temporary diagnostic (RXM-RTCM): same "may land before the F9P is
    // ready" risk as GST above, but with no isValid()-style flag to gate
    // on, so just resent unconditionally on the same cadence for the whole
    // cycle - negligible cost, guarantees it isn't silently lost to timing.
    if (millis() - lastRxmRtcmDiagResend >= GST_ENABLE_RETRY_INTERVAL_MS) {
      lastRxmRtcmDiagResend = millis();
      gnssEnableRxmRtcmDiag();
      // Same timing risk as the above two - resent on the same cadence.
      // Suspected real fix for RTK never engaging: see gnss.h.
      gnssEnableRtcm3Input();
    }

    if (!qualityOk && gnssHasFix() && gnssHasAccuracyEstimate() &&
        gnssHorizontalAccuracy() <= deviceConfig.good_accuracy_m) {
      qualityOk = true;
    }

    // Both sides of the loop condition satisfied - stop as soon as that's
    // true rather than waiting out whichever bound happens to be longer a
    // second time through the condition check above.
    if (qualityOk && (!hadInitialFix || millis() - initialFixMs >= deviceConfig.ppk_min_duration_s * 1000UL)) {
      break;
    }

    // Temporary diagnostic: mid-cycle RTK progress, independent of whether
    // the cycle ever reaches its own end-of-cycle summary print. Live
    // hardware has been hitting a task-watchdog reset ~85-97s into heavy
    // RTCM traffic (root cause not yet found - the async command paths
    // that could plausibly explain a hang were traced and all fail
    // gracefully, not blocking), cutting every cycle short before its
    // normal "Fix quality: ..." line. The F9P is a separate chip that very
    // likely keeps its own RTK engine state across an ESP32 reboot (only
    // the correction stream itself is what actually stops, briefly, during
    // the ~10-20s reboot/reinit) - this print is what lets convergence
    // progress actually be observed despite the instability, rather than
    // only ever seeing "cycle cut short, no data."
    if (millis() - lastStatusPrint >= 10000) {
      lastStatusPrint = millis();
      Serial.printf("[STATUS] t=%lus fix=%s hAcc=%.3fm RTCM=%uB\n", (millis() - fixWaitStart) / 1000,
                    gnssFixQualityLabel(gnssFixQuality()),
                    gnssHasAccuracyEstimate() ? gnssHorizontalAccuracy() : -1.0f, ntripBytesForwarded());
    }

    delay(20);
  }

  if (gnssCharsProcessed() < 10) {
    Serial.println(F("WARNING: no GNSS data received this cycle - check wiring, or EXTINT wake polarity/timing."));
  }

  Serial.print(F("Fix quality: "));
  Serial.println(gnssFixQualityLabel(gnssFixQuality()));
  if (gnssHasAccuracyEstimate()) {
    Serial.printf("Horizontal accuracy: %.3f m (good=%s)\n", gnssHorizontalAccuracy(), qualityOk ? "yes" : "no");
  }
  Serial.printf("RTCM bytes forwarded: %u\n", ntripBytesForwarded());

  positionLogAppend(qualityOk, batteryExternalVoltage(), batteryInternalVoltage());
  ntripDisconnect();
  gnssSleep();
}

static void runTelemetryBatch() {
  static char payload[2048];
  size_t rows = positionLogReadUnsent(payload, sizeof(payload));
  if (rows == 0) {
    return;
  }

  if (!modemConnect()) {
    Serial.println(F("Telemetry: modem not connected, skipping this batch."));
    return;
  }

  if (mqttConnectBroker()) {
    if (mqttPublishBatch(payload, rows)) {
      positionLogMarkSent();
    }
    mqttDisconnectBroker();
  }
}

// /data/gnss/ holds many per-burst files a day (see gnss.cpp), not one
// daily file like the other data types, so it can't use the same
// sdBuildDailyPath() single-file pattern - upload every *.ubx/*.rtcm whose
// name starts with the rolled-over date instead. Filenames without a real
// date prefix (the rare "boot_<millis>..." fallback, see
// gnssEnsureBurstStamp()) won't match anything and are never uploaded by
// this - a known, accepted gap for that already-rare case.
static void uploadGnssBurstFiles(const char *datePrefix) {
  File dir = SD_MMC.open("/data/gnss");
  if (!dir) {
    return;
  }
  size_t prefixLen = strlen(datePrefix);
  File f = dir.openNextFile();
  while (f) {
    if (!f.isDirectory() && strncmp(f.name(), datePrefix, prefixLen) == 0) {
      uploadFile(f.path(), f.name());
    }
    f = dir.openNextFile();
  }
  dir.close();
}

static void runDailyUploadIfNeeded() {
  if (!gnssHasDateTime()) {
    return;
  }

  char today[9];
  gnssDateString(today, sizeof(today));

  if (lastUploadedDate[0] == '\0') {
    // First time we've known the date this boot - nothing "yesterday" to
    // upload yet, just remember today so we upload once it rolls over.
    strncpy(lastUploadedDate, today, sizeof(lastUploadedDate) - 1);
    return;
  }

  if (strcmp(today, lastUploadedDate) == 0) {
    return; // still the same day
  }

  Serial.printf("Date rolled over (%s -> %s) - uploading yesterday's files.\n", lastUploadedDate, today);

  if (!modemConnect()) {
    Serial.println(F("Daily upload: modem not connected, will retry next loop."));
    return; // don't advance lastUploadedDate - retry until it actually uploads
  }

  // Single daily file per type - local path carries the /data subfolder,
  // remote name deliberately doesn't (the upload backend has no reason to
  // care about this device's local folder layout).
  static const char *localPrefixes[] = {"data/geophones/geophone", "data/positions"};
  static const char *remoteBasenames[] = {"geophone", "positions"};
  static const char *exts[] = {"mseed", "csv"};
  for (int i = 0; i < 2; i++) {
    char localPath[64];
    sdBuildDailyPath(localPath, sizeof(localPath), localPrefixes[i], lastUploadedDate, exts[i]);
    char remoteName[64];
    snprintf(remoteName, sizeof(remoteName), "%s_%s.%s", remoteBasenames[i], lastUploadedDate, exts[i]);
    uploadFile(localPath, remoteName);
  }

  uploadGnssBurstFiles(lastUploadedDate);

  strncpy(lastUploadedDate, today, sizeof(lastUploadedDate) - 1);
}

void setup() {
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) {
    delay(10); // give the USB CDC host a moment to enumerate; don't hang forever if unplugged
  }
  delay(200);

  Serial.println(F("Walter <-> ZED-F9P + RS3D permafrost/seismic field station"));

  // WalterModem's AT command TX/RX trace (ESP_LOGD("WalterModem", ...)) is
  // useful for short debugging sessions but hex-dumps every RTCM chunk at
  // DEBUG level - live hardware crashed (task watchdog on the idle task,
  // starved by how long the main task spent blocked in Serial writes) once
  // RTCM was flowing heavily enough for that plus the rest of this
  // project's own logging to saturate the USB-CDC link. WARN keeps actual
  // problems visible without the per-chunk firehose; raise back to
  // ESP_LOG_DEBUG only for targeted debugging.
  esp_log_level_set("WalterModem", ESP_LOG_WARN);

  loadDeviceConfig();

  sdStorageInit();
  initBattery();
  initPositionLog();

  initGeophone();
  initMiniseedWriter();

  gnssWakeup();
  delay(500);
  initGNSS();

  // One-time wiring/baud smoke test, run before the module is ever put to
  // sleep: isolates "is the raw UART link alive at all" from "does the
  // EXTINT/PMREQ sleep-wake cycle work" - the two have identical symptoms
  // (zero bytes received) from inside the main loop, since polling+printing
  // only happens after gnssSleep() has already run once.
  Serial.println(F("GNSS: listening for raw UART bytes (wiring/baud check)..."));
  unsigned long gnssCheckStart = millis();
  while (millis() - gnssCheckStart < 3000) {
    pollGNSS();
  }
  Serial.printf("GNSS: %lu chars seen in 3s smoke test (0 = check wiring/baud/SEL jumper, not sleep/wake).\n",
                (unsigned long)gnssCharsProcessed());

  gnssSleep(); // idle until the first fix cycle in loop()

  if (fieldModeRequested()) {
    runFieldService();
  }

  // Subscribed only for normal operation, after the field-service check
  // above: that console can legitimately sit idle for minutes waiting on a
  // technician's keystrokes, which would otherwise look like a hang and
  // trigger an unwanted reboot mid-configuration.
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  if (!modemInit()) {
    Serial.println(F("Modem init failed - modemConnect() will keep retrying each cycle."));
  }
}

void loop() {
  esp_task_wdt_reset();

  // Highest priority: the geophone digitizer streams continuously and
  // never pauses, so this must run every iteration.
  pollGeophone();
  pollGNSS();

  // Lets a technician trigger field service mode just by plugging in the
  // service cable, no physical reset needed - see field_service.h.
  checkFieldModeTrigger();

  // Lets a technician swap the SD card without a manual power cycle - see
  // sd_storage.h.
  sdCheckCardPresence();

  unsigned long now = millis();

  // Also covers the current burst's UBX/RTCM capture files
  // (gnssFlushCaptureFiles()) - same underlying problem (an open file only
  // reaches the card on close), same fix, no reason for a second timer.
  if (now - lastMiniseedFlushMs >= MINISEED_FLUSH_INTERVAL_MS) {
    lastMiniseedFlushMs = now;
    miniseedFlush();
    gnssFlushCaptureFiles();
  }

  if (!firstFixCycleDone || now - lastFixCycleMs >= deviceConfig.measurement_interval_s * 1000UL) {
    firstFixCycleDone = true;
    lastFixCycleMs = now;
    runGnssFixCycle();
  }

  if (now - lastTelemetryMs >= TELEMETRY_BATCH_INTERVAL_MS) {
    lastTelemetryMs = now;
    runTelemetryBatch();
  }

  runDailyUploadIfNeeded();
}
