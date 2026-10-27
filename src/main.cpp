#include <Arduino.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <SD.h> // for enumerating /data/gnss/*.ubx and *.rtcm in runDailyUploadIfNeeded()
#include <USB.h> // composite CDC+MSC (see sd_storage.h's sdStorageMsc* functions)
#include <string.h>

#include "device_config.h"
#include "field_service.h"
#include "gnss.h"
#include "geophone.h"
#include "miniseed_writer.h"
#include "sd_storage.h"
#include "battery.h"
#include "position_log.h"
#include "reference_position.h"
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
// RTCM normally flows continuously once GGA has reached the caster - this
// long with nothing new means the caster dropped the session or the
// receive path wedged, and the rest of the burst would otherwise pass with
// no corrections at all. Comfortably above GGA_RESEND_INTERVAL_MS, since
// a VRS mount only starts streaming after the first GGA lands.
static const unsigned long NTRIP_STALL_TIMEOUT_MS = 15000;
static const int NTRIP_MAX_RECONNECTS_PER_BURST = 2;

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
static unsigned long lastBatteryCheckMs = 0;
static char lastUploadedDate[9] = "";
static bool firstFixCycleDone = false;
static bool batteryOkForBurst = true;
static bool modemSleepPending = false; // see runGnssFixCycle()'s end and loop() below

// Read no more than once every 30s - analogReadMilliVolts() is cheap, but
// there's no reason to poll it every loop() iteration when the whole point
// is a slow-moving pack voltage, not a fast signal.
static const unsigned long BATTERY_CHECK_INTERVAL_MS = 30000;

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
    pollGNSS();
    // A burst can run 100+s - checkFieldModeTrigger()/sdCheckEjectButton()/
    // sdCheckCardPresence() only being polled once per *outer* loop()
    // iteration (as they still are for the idle-period case) meant a
    // technician pressing either button mid-burst went unnoticed until the
    // whole cycle finished - confirmed on live hardware. Polled here too so
    // both react immediately regardless of what the cycle is doing.
    checkFieldModeTrigger();
    sdCheckEjectButton();
    sdCheckCardPresence();
    if (millis() - lastGstEnableRetry >= GST_ENABLE_RETRY_INTERVAL_MS) {
      lastGstEnableRetry = millis();
      gnssEnableGST();
    }
    delay(20);
  }
}

// Runs at the end of every fix cycle now, not on its own hourly timer - the
// modem's already up for NTRIP every cycle anyway (measurement_interval_s,
// usually ~10 min), so batching hourly to "avoid an extra wake" no longer
// saves anything, and telemetry reaches the backend far sooner this way.
// Async (see modem.cpp/mqtt_client.h): runs right after ntripDisconnect()'s
// own async modemSocketClose(), so a blocking call here would land in
// exactly the command-channel contention that's already cost a lost AT
// command once before.
static void runTelemetryBatch() {
  static char payload[2048]; // must outlive the async publish - see modemMqttPublishBatchAsync()
  size_t rows = positionLogReadUnsent(payload, sizeof(payload));
  if (rows == 0) {
    return;
  }

  if (mqttBatchBusy()) {
    // Shouldn't normally happen (a whole burst separates one attempt from
    // the next), but don't stomp the in-flight payload buffer if it does -
    // these rows stay unsent and get picked up next cycle instead.
    Serial.println(F("Telemetry: previous MQTT batch still in flight, skipping this cycle."));
    return;
  }

  if (!modemIsConnected()) {
    Serial.println(F("Telemetry: modem not connected, skipping this batch."));
    return;
  }

  mqttPublishBatchAsync(payload, rows);
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
  // Timer kept as start + delay (not an absolute deadline) so it survives
  // millis() wrapping.
  unsigned long ggaTimerStartMs = millis();
  unsigned long ggaDelayMs = 0;
  if (ntripUp) {
    if (ntripSendGGA(gnssLastGGA())) {
      ggaTimerStartMs = millis();
      ggaDelayMs = GGA_RESEND_INTERVAL_MS;
    }
  }
  uint32_t lastRtcmBytes = ntripBytesForwarded();
  unsigned long lastRtcmProgressMs = millis();
  int ntripReconnects = 0;

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
    pollGNSS();
    // See waitForInitialFix()'s identical comment - this is the longer-
    // running of the two burst loops (typically the ~110s+ one), so it's
    // even more important these aren't only checked back in loop().
    checkFieldModeTrigger();
    sdCheckEjectButton();
    sdCheckCardPresence();

    if (ntripUp) {
      ntripPoll();

      unsigned long now = millis();

      // A prior send can fail *after* modemSocketSend() already reported
      // "queued OK" - the actual result only arrives later, asynchronously.
      // Checked every iteration so a real failure gets noticed quickly and
      // pulls the next attempt in, rather than waiting out the full
      // interval as if nothing had gone wrong.
      if (modemLastSendFailed()) {
        ggaTimerStartMs = now;
        ggaDelayMs = GGA_FAILURE_RETRY_MS;
      }

      if (now - ggaTimerStartMs >= ggaDelayMs) {
        if (ntripSendGGA(gnssLastGGA())) {
          ggaTimerStartMs = now;
          ggaDelayMs = GGA_RESEND_INTERVAL_MS;
        } else {
          // Deferred (a receive or the previous send was in flight) rather
          // than sent at all - retry next iteration, not after a fresh
          // full interval.
          ggaDelayMs = 0;
        }
      }

      uint32_t rtcmBytes = ntripBytesForwarded();
      if (rtcmBytes != lastRtcmBytes) {
        lastRtcmBytes = rtcmBytes;
        lastRtcmProgressMs = now;
      } else if (now - lastRtcmProgressMs >= NTRIP_STALL_TIMEOUT_MS &&
                 ntripReconnects < NTRIP_MAX_RECONNECTS_PER_BURST) {
        ntripReconnects++;
        Serial.printf("NTRIP: no RTCM for %lus - reconnecting (%d/%d).\n",
                      (now - lastRtcmProgressMs) / 1000, ntripReconnects,
                      NTRIP_MAX_RECONNECTS_PER_BURST);
        ntripDisconnect();
        ntripUp = ntripConnect(1);
        lastRtcmProgressMs = millis();
        ggaTimerStartMs = millis();
        ggaDelayMs = 0; // first GGA right away on the new session
      }
    }

    // The GST-enable command can land before the F9P is ready to process
    // it, especially right after an EXTINT wake from backup mode - originally
    // retried only until gnssHasAccuracyEstimate() went true, but live
    // hardware showed hAcc then freezing at that first value for the rest
    // of the cycle (qualityOk permanently false: "hAcc=X.XXXm ... good=no"
    // never changing across 100+s of STATUS lines) - GST sentences
    // apparently stop actually flowing at some point mid-cycle with no
    // self-healing retry once isValid() had already gone true once.
    // Resending unconditionally on the same cadence as RXM-RTCM/RTCM3-input
    // below (an already-proven fix for the identical "silently stops,
    // nothing notices" risk class) costs nothing and guarantees this can't
    // wedge for an entire cycle again.
    if (millis() - lastGstEnableRetry >= GST_ENABLE_RETRY_INTERVAL_MS) {
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

  // Only a fix good enough to be logged as "quality_ok" is trusted as a
  // commissioning survey observation or a real displacement measurement -
  // the same accuracy bar used everywhere else in this cycle, not a
  // separate one.
  float northM = 0, eastM = 0, upM = 0;
  bool hasDisplacement = false;
  if (qualityOk) {
    if (referencePositionIsCommissioning()) {
      referencePositionUpdate(gnssLatitude(), gnssLongitude(), gnssAltitude());
      Serial.printf("Reference position: commissioning %u/%u s, %u samples.\n",
                    (unsigned)referencePositionElapsedS(), (unsigned)deviceConfig.reference_window_s,
                    (unsigned)referencePositionSampleCount());
    } else if (referencePositionIsLocked()) {
      hasDisplacement = referencePositionDisplacement(gnssLatitude(), gnssLongitude(), gnssAltitude(),
                                                       &northM, &eastM, &upM);
      Serial.printf("Displacement from reference: north=%.3fm east=%.3fm up=%.3fm\n", northM, eastM,
                    upM);
    }
  }

  positionLogAppend(qualityOk, batteryExternalVoltage(), batteryInternalVoltage(), hasDisplacement,
                    northM, eastM, upM);
  ntripDisconnect();
  runTelemetryBatch(); // piggyback on the modem session this cycle already opened for NTRIP
  // Can't call modemSleep() here directly - runTelemetryBatch() only kicked
  // the MQTT batch off, it's still running on its own task (see modem.cpp)
  // and cutting radio out from under it would break the AT command
  // in-flight. loop() below actually sleeps once mqttBatchBusy() goes false.
  modemSleepPending = true;
  gnssSleep();
}

// /data/gnss/ holds many per-burst files a day (see gnss.cpp), not one
// daily file like the other data types, so it can't use the same
// sdBuildDailyPath() single-file pattern - upload every *.ubx/*.rtcm whose
// name starts with the rolled-over date instead. Filenames without a real
// date prefix (the rare "boot_<millis>..." fallback, see
// gnssEnsureBurstStamp()) won't match anything and are never uploaded by
// this - a known, accepted gap for that already-rare case.
static void uploadGnssBurstFiles(const char *datePrefix) {
  File dir = SD.open("/data/gnss");
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
  // Composite CDC+MSC (see sd_storage.h) - must run before the wait below,
  // since with ARDUINO_USB_MODE=0 nothing enumerates on the USB bus at all
  // (and Serial never becomes true) until USB.begin() actually brings the
  // peripheral up. The MSC LUN itself isn't configured yet (that needs the
  // card mounted first, see sdStorageMscBegin() below) - TinyUSB just reads
  // its capacity/media-present state live on each host request, so it's
  // safe to finalize the descriptor now and fill in real values later.
  USB.begin();
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
  referencePositionInit();

  sdStorageInit();
  sdStorageMscBegin();
  sdWriterInit();
  initBattery();
  initPositionLog();

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
    sdStorageMscEnable();
    runFieldService();
  }

  // Only now, past the field-service check: the geophone task writes
  // MiniSEED continuously, which must never happen while the card is
  // exposed to a USB host (runFieldService() reboots on exit, so normal
  // operation always comes through here).
  initGeophone();

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

  // The geophone digitizer has its own task now (see geophone.h) - nothing
  // in this loop can starve it any more.
  pollGNSS();

  // Lets a technician trigger field service mode just by plugging in the
  // service cable, no physical reset needed - see field_service.h.
  checkFieldModeTrigger();

  // Lets a technician swap the SD card without a manual power cycle - see
  // sd_storage.h.
  sdCheckCardPresence();
  sdCheckEjectButton();

  unsigned long now = millis();

  if (now - lastBatteryCheckMs >= BATTERY_CHECK_INTERVAL_MS) {
    lastBatteryCheckMs = now;
    bool wasOk = batteryOkForBurst;
    batteryOkForBurst = batteryExternalVoltage() >= deviceConfig.low_battery_cutoff_v;
    if (wasOk && !batteryOkForBurst) {
      Serial.printf("Battery: %.2fV below %.2fV cutoff - skipping new GNSS/modem bursts until "
                    "it recovers (geophone/SD logging continues).\n",
                    batteryExternalVoltage(), deviceConfig.low_battery_cutoff_v);
    } else if (!wasOk && batteryOkForBurst) {
      Serial.println(F("Battery: recovered above cutoff - resuming GNSS/modem bursts."));
    }
  }

  // Commissioning wants back-to-back bursts (each one a real RTK-quality
  // observation) rather than waiting out the normal measurement_interval_s
  // between them - reaching reference_window_s of accumulated quality fixes
  // this way instead of stretching over 10-minute-spaced cycles for days.
  bool commissioning = referencePositionIsCommissioning();
  if ((!firstFixCycleDone || commissioning ||
       now - lastFixCycleMs >= deviceConfig.measurement_interval_s * 1000UL) &&
      batteryOkForBurst) {
    firstFixCycleDone = true;
    lastFixCycleMs = now;
    runGnssFixCycle();
  }

  // Cheap consuming check every iteration - true only once, right when an
  // async batch kicked off by runTelemetryBatch() (see above) actually
  // completes, which can be several loop() iterations after that call
  // returned (see modem.cpp).
  if (mqttLastBatchOk()) {
    positionLogMarkSent();
  }

  // Deferred from runGnssFixCycle()'s end - only safe to cut radio once the
  // async MQTT batch that cycle kicked off (if any) has actually finished,
  // not merely been started. A cycle with nothing to publish clears this on
  // the very next iteration, since mqttBatchBusy() is already false then.
  if (modemSleepPending && !mqttBatchBusy()) {
    modemSleepPending = false;
    modemSleep();
  }

  runDailyUploadIfNeeded();
}
