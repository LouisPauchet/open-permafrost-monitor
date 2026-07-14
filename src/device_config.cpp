#include "device_config.h"
#include <Preferences.h>
#include "secrets.h"

DeviceConfig deviceConfig;
static Preferences prefs;
static const char *NVS_NAMESPACE = "device";

static void copyField(char *dst, size_t dstSize, const char *src) {
  strncpy(dst, src, dstSize - 1);
  dst[dstSize - 1] = '\0';
}

static void applyDefaults() {
  copyField(deviceConfig.mqtt_host, sizeof(deviceConfig.mqtt_host), MQTT_HOST);
  deviceConfig.mqtt_port = MQTT_PORT;
  copyField(deviceConfig.mqtt_client_id, sizeof(deviceConfig.mqtt_client_id), MQTT_CLIENT_ID);
  copyField(deviceConfig.mqtt_user, sizeof(deviceConfig.mqtt_user), MQTT_USER);
  copyField(deviceConfig.mqtt_password, sizeof(deviceConfig.mqtt_password), MQTT_PASSWORD);
  copyField(deviceConfig.mqtt_topic, sizeof(deviceConfig.mqtt_topic), MQTT_TOPIC);

  copyField(deviceConfig.ntrip_host, sizeof(deviceConfig.ntrip_host), NTRIP_HOST);
  deviceConfig.ntrip_port = NTRIP_PORT;
  copyField(deviceConfig.ntrip_mountpoint, sizeof(deviceConfig.ntrip_mountpoint), NTRIP_MOUNTPOINT);
  copyField(deviceConfig.ntrip_user, sizeof(deviceConfig.ntrip_user), NTRIP_USER);
  copyField(deviceConfig.ntrip_password, sizeof(deviceConfig.ntrip_password), NTRIP_PASSWORD);

  deviceConfig.measurement_interval_s = 600; // 10 minutes
  deviceConfig.fix_timeout_s = 120;          // give up after 2 minutes if not "good"
  deviceConfig.good_accuracy_m = 0.05f;      // 5 cm
}

void loadDeviceConfig() {
  applyDefaults();

  prefs.begin(NVS_NAMESPACE, true);
  bool hasSavedConfig = prefs.isKey("mqtt_host");

  if (hasSavedConfig) {
    prefs.getString("mqtt_host", deviceConfig.mqtt_host, sizeof(deviceConfig.mqtt_host));
    deviceConfig.mqtt_port = prefs.getUShort("mqtt_port", deviceConfig.mqtt_port);
    prefs.getString("mqtt_cid", deviceConfig.mqtt_client_id, sizeof(deviceConfig.mqtt_client_id));
    prefs.getString("mqtt_user", deviceConfig.mqtt_user, sizeof(deviceConfig.mqtt_user));
    prefs.getString("mqtt_pass", deviceConfig.mqtt_password, sizeof(deviceConfig.mqtt_password));
    prefs.getString("mqtt_topic", deviceConfig.mqtt_topic, sizeof(deviceConfig.mqtt_topic));

    prefs.getString("ntrip_host", deviceConfig.ntrip_host, sizeof(deviceConfig.ntrip_host));
    deviceConfig.ntrip_port = prefs.getUShort("ntrip_port", deviceConfig.ntrip_port);
    prefs.getString("ntrip_mnt", deviceConfig.ntrip_mountpoint, sizeof(deviceConfig.ntrip_mountpoint));
    prefs.getString("ntrip_user", deviceConfig.ntrip_user, sizeof(deviceConfig.ntrip_user));
    prefs.getString("ntrip_pass", deviceConfig.ntrip_password, sizeof(deviceConfig.ntrip_password));

    deviceConfig.measurement_interval_s = prefs.getUInt("interval_s", deviceConfig.measurement_interval_s);
    deviceConfig.fix_timeout_s = prefs.getUInt("timeout_s", deviceConfig.fix_timeout_s);
    deviceConfig.good_accuracy_m = prefs.getFloat("accuracy_m", deviceConfig.good_accuracy_m);
  }
  prefs.end();

  Serial.println(hasSavedConfig ? F("Loaded device config from NVS.") : F("No saved config found, using compiled-in defaults."));
}

void saveDeviceConfig() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putString("mqtt_host", deviceConfig.mqtt_host);
  prefs.putUShort("mqtt_port", deviceConfig.mqtt_port);
  prefs.putString("mqtt_cid", deviceConfig.mqtt_client_id);
  prefs.putString("mqtt_user", deviceConfig.mqtt_user);
  prefs.putString("mqtt_pass", deviceConfig.mqtt_password);
  prefs.putString("mqtt_topic", deviceConfig.mqtt_topic);

  prefs.putString("ntrip_host", deviceConfig.ntrip_host);
  prefs.putUShort("ntrip_port", deviceConfig.ntrip_port);
  prefs.putString("ntrip_mnt", deviceConfig.ntrip_mountpoint);
  prefs.putString("ntrip_user", deviceConfig.ntrip_user);
  prefs.putString("ntrip_pass", deviceConfig.ntrip_password);

  prefs.putUInt("interval_s", deviceConfig.measurement_interval_s);
  prefs.putUInt("timeout_s", deviceConfig.fix_timeout_s);
  prefs.putFloat("accuracy_m", deviceConfig.good_accuracy_m);
  prefs.end();
}
