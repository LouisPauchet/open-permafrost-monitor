#include <Arduino.h>
#include <esp_system.h>
#include <esp_log.h>
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

static const unsigned long GGA_RESEND_INTERVAL_MS = 10000;
static const unsigned long GST_ENABLE_RETRY_INTERVAL_MS = 5000;
static const unsigned long MINISEED_FLUSH_INTERVAL_MS = 5000;
// Not user-configurable (kept out of the config menu per the "as few
// parameters as possible" preference) - how often to wake the modem for a
// batched MQTT publish, independent of the (usually shorter) GNSS fix
// interval, so the modem doesn't need a fresh MQTT session every fix.
static const unsigned long TELEMETRY_BATCH_INTERVAL_MS = 3600000UL; // 1 hour

static unsigned long lastFixCycleMs = 0;
static unsigned long lastTelemetryMs = 0;
static unsigned long lastMiniseedFlushMs = 0;
static char lastUploadedDate[9] = "";
static bool firstFixCycleDone = false;

static void runGnssFixCycle() {
  gnssWakeup();
  geophoneAwareDelay(500);
  initGNSS();

  bool ntripUp = false;
  if (modemConnect()) {
    ntripUp = ntripConnect();
  }

  Serial.printf("Waiting up to %u s for a fix with horizontal accuracy <= %.3f m...\n",
                deviceConfig.fix_timeout_s, deviceConfig.good_accuracy_m);

  unsigned long fixWaitStart = millis();
  unsigned long lastGgaSent = 0;
  unsigned long lastGstEnableRetry = 0;
  bool qualityOk = false;

  while (millis() - fixWaitStart < deviceConfig.fix_timeout_s * 1000UL) {
    pollGeophone();
    pollGNSS();

    if (ntripUp) {
      ntripPoll();

      unsigned long now = millis();
      if (now - lastGgaSent >= GGA_RESEND_INTERVAL_MS) {
        lastGgaSent = now;
        ntripSendGGA(gnssLastGGA());
      }
    }

    // The GST-enable command can land before the F9P is ready to process
    // it, especially right after an EXTINT wake from backup mode - keep
    // retrying until GST sentences are actually flowing.
    if (!gnssHasAccuracyEstimate() && millis() - lastGstEnableRetry >= GST_ENABLE_RETRY_INTERVAL_MS) {
      lastGstEnableRetry = millis();
      gnssEnableGST();
    }

    if (gnssHasFix() && gnssHasAccuracyEstimate() && gnssHorizontalAccuracy() <= deviceConfig.good_accuracy_m) {
      qualityOk = true;
      break;
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

  static const char *prefixes[] = {"geophone", "gnss_raw", "ntrip", "positions"};
  static const char *exts[] = {"mseed", "log", "log", "csv"};
  for (int i = 0; i < 4; i++) {
    char localPath[48];
    sdBuildDailyPath(localPath, sizeof(localPath), prefixes[i], lastUploadedDate, exts[i]);
    char remoteName[64];
    snprintf(remoteName, sizeof(remoteName), "%s_%s.%s", prefixes[i], lastUploadedDate, exts[i]);
    uploadFile(localPath, remoteName);
  }

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

  // Belt-and-suspenders alongside the CORE_DEBUG_LEVEL build flag: makes
  // WalterModem's AT command TX/RX trace (ESP_LOGD("WalterModem", ...))
  // visible regardless of whatever default the Arduino core startup set.
  esp_log_level_set("WalterModem", ESP_LOG_DEBUG);

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

  if (!modemInit()) {
    Serial.println(F("Modem init failed - modemConnect() will keep retrying each cycle."));
  }
}

void loop() {
  // Highest priority: the geophone digitizer streams continuously and
  // never pauses, so this must run every iteration.
  pollGeophone();
  pollGNSS();

  // Lets a technician trigger field service mode just by plugging in the
  // service cable, no physical reset needed - see field_service.h.
  checkFieldModeTrigger();

  unsigned long now = millis();

  if (now - lastMiniseedFlushMs >= MINISEED_FLUSH_INTERVAL_MS) {
    lastMiniseedFlushMs = now;
    miniseedFlush();
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
