#pragma once

// Opens a WiFi AP + captive web portal (blocking) for up to CONFIG_PORTAL_TIMEOUT_S
// seconds, letting a technician set WiFi, MQTT, NTRIP and measurement parameters
// without reflashing. Saves to NVS via device_config.h if the form is submitted.
// Only call this on a cold boot (not on a deep-sleep timer wake) - see main.cpp.
void runConfigPortal();
