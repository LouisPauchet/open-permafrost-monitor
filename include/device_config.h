#pragma once
#include <Arduino.h>

// Field-configurable settings, persisted in NVS flash (survives deep sleep
// and reflashing). Set via the setup portal (see config_portal.h) or seeded
// from include/secrets.h defaults on first boot.
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

  uint32_t measurement_interval_s;  // how often to wake, measure, publish, sleep
  uint32_t fix_timeout_s;           // max time to wait for a "good" fix before giving up
  float good_accuracy_m;            // horizontal accuracy threshold for a "good" fix
};

extern DeviceConfig deviceConfig;

void loadDeviceConfig();
void saveDeviceConfig();
