#pragma once
#include <Arduino.h>

// Field-configurable settings, persisted in NVS flash. Set via the
// field-service config menu (see field_service.h) or seeded from
// include/secrets.h defaults on first boot.
struct DeviceConfig {
  char mqtt_host[64];
  uint16_t mqtt_port;
  char mqtt_client_id[32];
  char mqtt_user[64];
  char mqtt_password[64];
  char mqtt_topic[64];

  char ntrip_host[64];
  uint16_t ntrip_port;
  char ntrip_mountpoint[32];
  char ntrip_user[32];
  char ntrip_password[32];

  uint32_t measurement_interval_s;  // how often to take a GNSS fix, publish, sleep
  uint32_t fix_timeout_s;           // max time to wait for a "good" fix before giving up
  float good_accuracy_m;            // horizontal accuracy threshold for a "good" fix
  // Minimum raw-observation (RAWX/SFRBX) recording time for PPK post-
  // processing, measured from the initial (autonomous) fix - independent of
  // fix_timeout_s/good_accuracy_m, which govern the real-time RTK/NTRIP
  // side. A fast RTK convergence would otherwise end the cycle (and put the
  // GNSS module to sleep) well before enough raw data has accumulated to
  // post-process.
  uint32_t ppk_min_duration_s;

  // NTRIP (and with it the modem, MQTT telemetry and the daily upload) only
  // runs on every Nth GNSS burst - 1 = every burst. The bursts in between
  // still record UBX raw observations for ppk_min_duration_s and then end
  // (no RTK accuracy to wait for without corrections), with no modem wake
  // at all; their PPK base data has to come from elsewhere (e.g. a public
  // CORS station) since no RTCM is recorded. Commissioning bursts always
  // use NTRIP. Queued telemetry rows go out on the next NTRIP burst.
  uint32_t ntrip_every_n_bursts;

  // How long a commissioning run (field-service menu) averages GNSS fixes
  // for before locking in the station's reference position - see
  // reference_position.h. Default matches a real static-GNSS survey
  // duration long enough to average out multipath/atmospheric error;
  // shortened only for bench testing, never for a real deployment.
  uint32_t reference_window_s;

  // Daily-file upload endpoint. One plain HTTPS POST per file (large files
  // chunked, see upload_client.h), authenticated with HTTP Basic auth using
  // upload_api_key as the username - a small backend the user runs
  // themselves forwards the file to S3-compatible storage; see the plan.
  char upload_host[64];
  uint16_t upload_port;
  char upload_api_key[64];

  // Below this (measured on BATTERY_EXTERNAL_ADC_PIN - see battery.h), new
  // GNSS/modem bursts are skipped entirely - geophone/SD logging keeps
  // running (comparatively low power), but the LTE modem and F9P (the two
  // real power draws) stay off until solar charging recovers the pack.
  // Default assumes a 12V lead-acid pack: ~11.8V is roughly 10-20%
  // remaining, comfortably above the ~11.5V deep-discharge zone that
  // shortens lead-acid lifespan, but under-load sag during an active burst
  // may warrant a site-specific value once real discharge behavior is
  // observed in the field.
  float low_battery_cutoff_v;
};

extern DeviceConfig deviceConfig;

void loadDeviceConfig();
void saveDeviceConfig();
