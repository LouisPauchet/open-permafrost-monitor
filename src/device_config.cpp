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
  deviceConfig.ppk_min_duration_s = 120;     // 2 minutes of raw observations per cycle
  deviceConfig.ntrip_every_n_bursts = 1;     // NTRIP on every burst
  deviceConfig.reference_window_s = 7200;    // 2 hours of commissioning averaging
  deviceConfig.low_battery_cutoff_v = 11.8f; // 12V lead-acid, see device_config.h
  copyField(deviceConfig.seed_network, sizeof(deviceConfig.seed_network), "XX");
  copyField(deviceConfig.seed_station, sizeof(deviceConfig.seed_station), "PERM1");
  copyField(deviceConfig.seed_location, sizeof(deviceConfig.seed_location), "");
  deviceConfig.miniseed_steim2 = true;

  copyField(deviceConfig.upload_host, sizeof(deviceConfig.upload_host), UPLOAD_HOST);
  deviceConfig.upload_port = UPLOAD_PORT;
  copyField(deviceConfig.upload_api_key, sizeof(deviceConfig.upload_api_key), UPLOAD_API_KEY);
}

void loadDeviceConfig() {
  applyDefaults();

  prefs.begin(NVS_NAMESPACE, true);
  bool hasSavedConfig = prefs.isKey("mqtt_host");

  if (hasSavedConfig) {
    DeviceConfig loaded = deviceConfig; // start from the defaults already applied above

    prefs.getString("mqtt_host", loaded.mqtt_host, sizeof(loaded.mqtt_host));
    loaded.mqtt_port = prefs.getUShort("mqtt_port", loaded.mqtt_port);
    prefs.getString("mqtt_cid", loaded.mqtt_client_id, sizeof(loaded.mqtt_client_id));
    prefs.getString("mqtt_user", loaded.mqtt_user, sizeof(loaded.mqtt_user));
    prefs.getString("mqtt_pass", loaded.mqtt_password, sizeof(loaded.mqtt_password));
    prefs.getString("mqtt_topic", loaded.mqtt_topic, sizeof(loaded.mqtt_topic));

    prefs.getString("ntrip_host", loaded.ntrip_host, sizeof(loaded.ntrip_host));
    loaded.ntrip_port = prefs.getUShort("ntrip_port", loaded.ntrip_port);
    prefs.getString("ntrip_mnt", loaded.ntrip_mountpoint, sizeof(loaded.ntrip_mountpoint));
    prefs.getString("ntrip_user", loaded.ntrip_user, sizeof(loaded.ntrip_user));
    prefs.getString("ntrip_pass", loaded.ntrip_password, sizeof(loaded.ntrip_password));

    loaded.measurement_interval_s = prefs.getUInt("interval_s", loaded.measurement_interval_s);
    loaded.fix_timeout_s = prefs.getUInt("timeout_s", loaded.fix_timeout_s);
    loaded.good_accuracy_m = prefs.getFloat("accuracy_m", loaded.good_accuracy_m);
    loaded.ppk_min_duration_s = prefs.getUInt("ppk_s", loaded.ppk_min_duration_s);
    loaded.ntrip_every_n_bursts = prefs.getUInt("ntrip_n", loaded.ntrip_every_n_bursts);
    if (loaded.ntrip_every_n_bursts == 0) {
      loaded.ntrip_every_n_bursts = 1; // 0 would mean "never" - not a setting this offers
    }
    loaded.reference_window_s = prefs.getUInt("ref_win_s", loaded.reference_window_s);
    loaded.low_battery_cutoff_v = prefs.getFloat("batt_cutoff_v", loaded.low_battery_cutoff_v);
    if (prefs.isKey("seed_sta")) { // absent on configs saved by older firmware
      prefs.getString("seed_net", loaded.seed_network, sizeof(loaded.seed_network));
      prefs.getString("seed_sta", loaded.seed_station, sizeof(loaded.seed_station));
      prefs.getString("seed_loc", loaded.seed_location, sizeof(loaded.seed_location));
    }
    loaded.miniseed_steim2 = prefs.getBool("mseed_steim2", loaded.miniseed_steim2);

    prefs.getString("upload_host", loaded.upload_host, sizeof(loaded.upload_host));
    loaded.upload_port = prefs.getUShort("upload_port", loaded.upload_port);
    prefs.getString("upload_key", loaded.upload_api_key, sizeof(loaded.upload_api_key));

    // Guard against a previously-saved blank/corrupt entry (see
    // field_service.cpp for how this is now prevented going forward) -
    // fall back to the compiled-in defaults instead of running with
    // empty hosts / a zero measurement interval.
    bool looksValid = strlen(loaded.mqtt_host) > 0 && strlen(loaded.ntrip_host) > 0 &&
                       loaded.mqtt_port > 0 && loaded.ntrip_port > 0 &&
                       loaded.measurement_interval_s > 0 && loaded.fix_timeout_s > 0;

    if (looksValid) {
      deviceConfig = loaded;
    } else {
      hasSavedConfig = false;
      Serial.println(F("Saved config in NVS looked blank/corrupt (empty host or zero interval) - ignoring it, using compiled-in defaults instead."));
    }
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
  prefs.putUInt("ppk_s", deviceConfig.ppk_min_duration_s);
  prefs.putUInt("ntrip_n", deviceConfig.ntrip_every_n_bursts);
  prefs.putUInt("ref_win_s", deviceConfig.reference_window_s);
  prefs.putFloat("batt_cutoff_v", deviceConfig.low_battery_cutoff_v);
  prefs.putString("seed_net", deviceConfig.seed_network);
  prefs.putString("seed_sta", deviceConfig.seed_station);
  prefs.putString("seed_loc", deviceConfig.seed_location);
  prefs.putBool("mseed_steim2", deviceConfig.miniseed_steim2);

  prefs.putString("upload_host", deviceConfig.upload_host);
  prefs.putUShort("upload_port", deviceConfig.upload_port);
  prefs.putString("upload_key", deviceConfig.upload_api_key);
  prefs.end();
}
