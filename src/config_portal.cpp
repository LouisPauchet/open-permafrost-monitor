#include "config_portal.h"
#include "device_config.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <stdlib.h>
#include <string.h>

static const uint16_t CONFIG_PORTAL_TIMEOUT_S = 300; // 5 minutes

static bool configSaved = false;
static void onSaveConfig() {
  configSaved = true;
}

static void copyField(char *dst, size_t dstSize, const char *src) {
  strncpy(dst, src, dstSize - 1);
  dst[dstSize - 1] = '\0';
}

// "PermafrostMonitor-AABBCC" using the last 3 bytes of the ESP32's factory
// MAC, so each physical unit gets a distinguishable hotspot name in the field.
static String buildApName() {
  WiFi.mode(WIFI_STA); // ensures a valid MAC is readable
  String mac = WiFi.macAddress(); // "AA:BB:CC:DD:EE:FF"
  mac.replace(":", "");
  String suffix = mac.substring(mac.length() - 6);
  return String("PermafrostMonitor-") + suffix;
}

void runConfigPortal() {
  configSaved = false;
  String apName = buildApName();
  WiFiManager wm;

  char port_buf[8];
  char ntrip_port_buf[8];
  char interval_buf[10];
  char timeout_buf[10];
  char accuracy_buf[10];
  snprintf(port_buf, sizeof(port_buf), "%u", deviceConfig.mqtt_port);
  snprintf(ntrip_port_buf, sizeof(ntrip_port_buf), "%u", deviceConfig.ntrip_port);
  snprintf(interval_buf, sizeof(interval_buf), "%u", deviceConfig.measurement_interval_s);
  snprintf(timeout_buf, sizeof(timeout_buf), "%u", deviceConfig.fix_timeout_s);
  snprintf(accuracy_buf, sizeof(accuracy_buf), "%.3f", deviceConfig.good_accuracy_m);

  WiFiManagerParameter p_mqtt_host("mqtt_host", "MQTT host", deviceConfig.mqtt_host, sizeof(deviceConfig.mqtt_host) - 1);
  WiFiManagerParameter p_mqtt_port("mqtt_port", "MQTT port", port_buf, sizeof(port_buf) - 1);
  WiFiManagerParameter p_mqtt_cid("mqtt_cid", "MQTT client ID", deviceConfig.mqtt_client_id, sizeof(deviceConfig.mqtt_client_id) - 1);
  WiFiManagerParameter p_mqtt_user("mqtt_user", "MQTT username / token", deviceConfig.mqtt_user, sizeof(deviceConfig.mqtt_user) - 1);
  WiFiManagerParameter p_mqtt_pass("mqtt_pass", "MQTT password", deviceConfig.mqtt_password, sizeof(deviceConfig.mqtt_password) - 1);
  WiFiManagerParameter p_mqtt_topic("mqtt_topic", "MQTT topic", deviceConfig.mqtt_topic, sizeof(deviceConfig.mqtt_topic) - 1);

  WiFiManagerParameter p_ntrip_host("ntrip_host", "NTRIP caster host", deviceConfig.ntrip_host, sizeof(deviceConfig.ntrip_host) - 1);
  WiFiManagerParameter p_ntrip_port("ntrip_port", "NTRIP caster port", ntrip_port_buf, sizeof(ntrip_port_buf) - 1);
  WiFiManagerParameter p_ntrip_mnt("ntrip_mnt", "NTRIP mountpoint", deviceConfig.ntrip_mountpoint, sizeof(deviceConfig.ntrip_mountpoint) - 1);
  WiFiManagerParameter p_ntrip_user("ntrip_user", "NTRIP username", deviceConfig.ntrip_user, sizeof(deviceConfig.ntrip_user) - 1);
  WiFiManagerParameter p_ntrip_pass("ntrip_pass", "NTRIP password", deviceConfig.ntrip_password, sizeof(deviceConfig.ntrip_password) - 1);

  WiFiManagerParameter p_interval("interval_s", "Measurement interval (seconds)", interval_buf, sizeof(interval_buf) - 1);
  WiFiManagerParameter p_timeout("timeout_s", "Max wait for a good fix (seconds)", timeout_buf, sizeof(timeout_buf) - 1);
  WiFiManagerParameter p_accuracy("accuracy_m", "Good fix horizontal accuracy (meters)", accuracy_buf, sizeof(accuracy_buf) - 1);

  wm.addParameter(&p_mqtt_host);
  wm.addParameter(&p_mqtt_port);
  wm.addParameter(&p_mqtt_cid);
  wm.addParameter(&p_mqtt_user);
  wm.addParameter(&p_mqtt_pass);
  wm.addParameter(&p_mqtt_topic);
  wm.addParameter(&p_ntrip_host);
  wm.addParameter(&p_ntrip_port);
  wm.addParameter(&p_ntrip_mnt);
  wm.addParameter(&p_ntrip_user);
  wm.addParameter(&p_ntrip_pass);
  wm.addParameter(&p_interval);
  wm.addParameter(&p_timeout);
  wm.addParameter(&p_accuracy);

  wm.setSaveConfigCallback(onSaveConfig);
  wm.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT_S);

  Serial.printf("Setup portal open for %u s - connect to WiFi \"%s\" and browse to 192.168.4.1\n",
                CONFIG_PORTAL_TIMEOUT_S, apName.c_str());
  wm.startConfigPortal(apName.c_str());

  if (configSaved) {
    // Guard against persisting an obviously-blank submission (e.g. a phone's
    // automatic captive-portal-detection probe hitting the save endpoint, or
    // a page load before fields were pre-filled) - that would otherwise
    // silently wipe a previously-working configuration with empty/zero
    // values. Require the two hostnames and the numeric fields to be
    // non-empty/non-zero before accepting the submission.
    bool looksValid = strlen(p_mqtt_host.getValue()) > 0 &&
                       strlen(p_ntrip_host.getValue()) > 0 &&
                       atoi(p_mqtt_port.getValue()) > 0 &&
                       atoi(p_ntrip_port.getValue()) > 0 &&
                       strtoul(p_interval.getValue(), nullptr, 10) > 0 &&
                       strtoul(p_timeout.getValue(), nullptr, 10) > 0;

    if (!looksValid) {
      Serial.println(F("Setup portal submission looked blank/invalid (host or numeric field empty/zero) - ignoring it, keeping existing config."));
      return;
    }

    copyField(deviceConfig.mqtt_host, sizeof(deviceConfig.mqtt_host), p_mqtt_host.getValue());
    deviceConfig.mqtt_port = (uint16_t)atoi(p_mqtt_port.getValue());
    copyField(deviceConfig.mqtt_client_id, sizeof(deviceConfig.mqtt_client_id), p_mqtt_cid.getValue());
    copyField(deviceConfig.mqtt_user, sizeof(deviceConfig.mqtt_user), p_mqtt_user.getValue());
    copyField(deviceConfig.mqtt_password, sizeof(deviceConfig.mqtt_password), p_mqtt_pass.getValue());
    copyField(deviceConfig.mqtt_topic, sizeof(deviceConfig.mqtt_topic), p_mqtt_topic.getValue());

    copyField(deviceConfig.ntrip_host, sizeof(deviceConfig.ntrip_host), p_ntrip_host.getValue());
    deviceConfig.ntrip_port = (uint16_t)atoi(p_ntrip_port.getValue());
    copyField(deviceConfig.ntrip_mountpoint, sizeof(deviceConfig.ntrip_mountpoint), p_ntrip_mnt.getValue());
    copyField(deviceConfig.ntrip_user, sizeof(deviceConfig.ntrip_user), p_ntrip_user.getValue());
    copyField(deviceConfig.ntrip_password, sizeof(deviceConfig.ntrip_password), p_ntrip_pass.getValue());

    deviceConfig.measurement_interval_s = strtoul(p_interval.getValue(), nullptr, 10);
    deviceConfig.fix_timeout_s = strtoul(p_timeout.getValue(), nullptr, 10);
    deviceConfig.good_accuracy_m = atof(p_accuracy.getValue());

    saveDeviceConfig();
    Serial.println(F("Configuration saved."));
  } else {
    Serial.println(F("Setup portal closed without saving (timeout or no submission)."));
  }
}
