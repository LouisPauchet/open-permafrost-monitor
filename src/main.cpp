#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include "device_config.h"
#include "gnss.h"
#include "modem.h"
#include "ntrip_client.h"
#include "mqtt_client.h"
#include "sd_logger.h"

// TODO(next stage): main.cpp is still the old deep-sleep-per-cycle shape
// here - this is just the WiFi->cellular transport swap (stage a of the
// field-deployment rewrite). It gets replaced by a persistent loop() once
// the geophone module lands, since that needs the CPU continuously awake.

static const unsigned long GGA_RESEND_INTERVAL_MS = 10000;
static const unsigned long GST_ENABLE_RETRY_INTERVAL_MS = 5000;

static void goToSleep() {
  Serial.printf("Sleeping for %u s...\n", deviceConfig.measurement_interval_s);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)deviceConfig.measurement_interval_s * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("Walter <-> ZED-F9P RTK permafrost monitor"));

  loadDeviceConfig();
  initSDLogger(); // no-op today (no reader wired up yet); ready once it is

  // Wake the F9P from UBX-RXM-PMREQ backup mode (harmless if it wasn't
  // asleep, e.g. the very first boot ever) and give it a moment to come
  // back up before talking to it over UART.
  gnssWakeup();
  delay(500);
  initGNSS();

  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
  bool wokeFromTimer = (wakeReason == ESP_SLEEP_WAKEUP_TIMER);
  Serial.printf("Wake cause: %d (ESP_SLEEP_WAKEUP_TIMER=%d), reset reason: %d\n",
                wakeReason, ESP_SLEEP_WAKEUP_TIMER, esp_reset_reason());

  if (!modemInit()) {
    Serial.println(F("Modem init failed - sleeping and retrying next cycle."));
    gnssSleep();
    goToSleep();
    return;
  }

  if (!modemConnect()) {
    gnssSleep();
    goToSleep();
    return;
  }

  bool ntripOk = ntripConnect();

  Serial.printf("Waiting up to %u s for a fix with horizontal accuracy <= %.3f m...\n",
                deviceConfig.fix_timeout_s, deviceConfig.good_accuracy_m);

  unsigned long fixWaitStart = millis();
  unsigned long lastGgaSent = 0;
  unsigned long lastGstEnableRetry = 0;
  bool qualityOk = false;

  while (millis() - fixWaitStart < deviceConfig.fix_timeout_s * 1000UL) {
    pollGNSS();

    if (ntripOk) {
      ntripPoll();

      unsigned long now = millis();
      if (now - lastGgaSent >= GGA_RESEND_INTERVAL_MS) {
        lastGgaSent = now;
        ntripSendGGA(gnssLastGGA());
      }
    }

    // The GST-enable command sent in initGNSS() can land before the F9P is
    // ready to process it, especially right after an EXTINT wake from
    // backup mode - keep retrying until GST sentences are actually flowing.
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

  if (mqttConnectBroker()) {
    mqttPublishPosition(qualityOk);
    mqttDisconnectBroker();
  }

  ntripDisconnect();
  closeSDLogger();
  gnssSleep();

  goToSleep();
}

void loop() {
  // Never reached: setup() always ends in deep sleep, which restarts
  // execution from setup() on wake.
}
