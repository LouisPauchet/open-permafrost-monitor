#include <Arduino.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include "device_config.h"
#include "config_portal.h"
#include "gnss.h"
#include "ntrip_client.h"
#include "mqtt_client.h"

// The default 8KB Arduino loop-task stack is too small for a TLS handshake
// (WiFiClientSecure/mbedTLS) and crashes with a stack overflow at connect time.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;
static const unsigned long GGA_RESEND_INTERVAL_MS = 10000;

static bool connectWiFiSTA() {
  // The config portal (if it ran this boot) already connects WiFi - calling
  // WiFi.begin() again here would restart the association and race with
  // DHCP, briefly leaving WiFi.status() == WL_CONNECTED with IP 0.0.0.0.
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("WiFi already connected, IP: "));
    Serial.println(WiFi.localIP());
    return true;
  }

  WiFi.mode(WIFI_STA);
  // ESP32 WiFi power-save (modem sleep) periodically drops the association
  // (typically every ~30-60s) to save power - disable it for a stable link.
  WiFi.setSleep(false);
  WiFi.begin(); // reconnect with credentials saved by the config portal

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("WiFi connected, IP: "));
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println(F("WiFi connect failed."));
  return false;
}

static void goToSleep() {
  Serial.printf("Sleeping for %u s...\n", deviceConfig.measurement_interval_s);
  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)deviceConfig.measurement_interval_s * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("ESP32 <-> ZED-F9P RTK permafrost monitor"));

  loadDeviceConfig();
  initGNSS();

  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
  bool wokeFromTimer = (wakeReason == ESP_SLEEP_WAKEUP_TIMER);
  Serial.printf("Wake cause: %d (ESP_SLEEP_WAKEUP_TIMER=%d), reset reason: %d\n",
                wakeReason, ESP_SLEEP_WAKEUP_TIMER, esp_reset_reason());

  if (!wokeFromTimer) {
    Serial.println(F("Power-on/reset boot - opening the setup portal."));
    runConfigPortal();
  }

  if (!connectWiFiSTA()) {
    goToSleep();
    return;
  }

  bool ntripOk = ntripConnect();

  Serial.printf("Waiting up to %u s for a fix with horizontal accuracy <= %.3f m...\n",
                deviceConfig.fix_timeout_s, deviceConfig.good_accuracy_m);

  unsigned long fixWaitStart = millis();
  unsigned long lastGgaSent = 0;
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

    if (gnssHasFix() && gnssHasAccuracyEstimate() && gnssHorizontalAccuracy() <= deviceConfig.good_accuracy_m) {
      qualityOk = true;
      break;
    }

    delay(20);
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
  WiFi.disconnect(true);

  goToSleep();
}

void loop() {
  // Never reached: setup() always ends in deep sleep, which restarts
  // execution from setup() on wake.
}
