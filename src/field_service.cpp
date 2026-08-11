#include "field_service.h"
#include "device_config.h"
#include "reference_position.h"
#include "battery.h"
#include "pins.h"
#include <SD_MMC.h>
#include <stdlib.h>

bool fieldModeRequested() {
  pinMode(FIELD_MODE_SELECT_PIN, INPUT_PULLUP);
  delay(10);
  return digitalRead(FIELD_MODE_SELECT_PIN) == LOW;
}

// Returns "" if the cable is pulled while waiting - callers already treat
// an empty line as "no change" (the prompt* helpers) or harmlessly fall
// through to "Unknown choice" (the main menu), and the caller's own loop
// condition ends the session on the next check either way.
static String readLine() {
  while (!Serial.available()) {
    if (digitalRead(FIELD_MODE_SELECT_PIN) == HIGH) {
      return String();
    }
    delay(10);
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  return line;
}

static void promptString(const char *label, char *dst, size_t dstSize) {
  Serial.printf("%s [%s]: ", label, dst);
  String line = readLine();
  if (line.length() > 0) {
    strncpy(dst, line.c_str(), dstSize - 1);
    dst[dstSize - 1] = '\0';
  }
}

static void promptUInt(const char *label, uint32_t *dst) {
  Serial.printf("%s [%u]: ", label, (unsigned)*dst);
  String line = readLine();
  if (line.length() > 0) {
    *dst = (uint32_t)strtoul(line.c_str(), nullptr, 10);
  }
}

static void promptFloat(const char *label, float *dst) {
  Serial.printf("%s [%.3f]: ", label, *dst);
  String line = readLine();
  if (line.length() > 0) {
    *dst = atof(line.c_str());
  }
}

static void printMenu() {
  Serial.println();
  Serial.println(F("=== Field service menu ==="));
  Serial.println(F("1) Show current configuration"));
  Serial.println(F("2) Set GNSS measurement interval (s)"));
  Serial.println(F("3) Set GNSS fix timeout (s)"));
  Serial.println(F("4) Set good-fix horizontal accuracy (m)"));
  Serial.println(F("5) Set MQTT broker"));
  Serial.println(F("6) Set NTRIP caster"));
  Serial.println(F("7) Set upload endpoint"));
  Serial.println(F("8) List SD card files"));
  Serial.println(F("9) Download a file"));
  Serial.println(F("A) Set PPK minimum recording duration (s)"));
  Serial.println(F("B) Set reference-position averaging window (s)"));
  Serial.println(F("C) Start commissioning (establish reference position)"));
  Serial.println(F("D) Set low-battery cutoff (V) - new bursts skipped below this"));
  Serial.println(F("0) Save and exit"));
  Serial.print(F("> "));
}

static void showConfig() {
  Serial.println(F("--- Current configuration ---"));
  Serial.printf("Measurement interval: %u s\n", (unsigned)deviceConfig.measurement_interval_s);
  Serial.printf("Fix timeout: %u s\n", (unsigned)deviceConfig.fix_timeout_s);
  Serial.printf("Good accuracy: %.3f m\n", deviceConfig.good_accuracy_m);
  Serial.printf("PPK minimum recording duration: %u s\n", (unsigned)deviceConfig.ppk_min_duration_s);
  Serial.printf("Reference-position averaging window: %u s\n", (unsigned)deviceConfig.reference_window_s);
  if (referencePositionIsLocked()) {
    double lat, lon, alt;
    referencePositionGet(&lat, &lon, &alt);
    Serial.printf("Reference position: LOCKED at %.8f, %.8f, %.2fm\n", lat, lon, alt);
  } else if (referencePositionIsCommissioning()) {
    Serial.printf("Reference position: commissioning in progress (%u/%u s, %u samples)\n",
                  (unsigned)referencePositionElapsedS(), (unsigned)deviceConfig.reference_window_s,
                  (unsigned)referencePositionSampleCount());
  } else {
    Serial.println(F("Reference position: not commissioned yet."));
  }
  Serial.printf("MQTT: %s:%u (client %s)\n", deviceConfig.mqtt_host, deviceConfig.mqtt_port,
                deviceConfig.mqtt_client_id);
  Serial.printf("NTRIP: %s:%u/%s\n", deviceConfig.ntrip_host, deviceConfig.ntrip_port,
                deviceConfig.ntrip_mountpoint);
  Serial.printf("Upload: %s:%u\n", deviceConfig.upload_host, deviceConfig.upload_port);
  Serial.printf("Low-battery cutoff: %.2f V (currently reading %.2f V)\n",
                deviceConfig.low_battery_cutoff_v, batteryExternalVoltage());
}

static void setMqtt() {
  promptString("MQTT host", deviceConfig.mqtt_host, sizeof(deviceConfig.mqtt_host));
  uint32_t port = deviceConfig.mqtt_port;
  promptUInt("MQTT port", &port);
  deviceConfig.mqtt_port = (uint16_t)port;
  promptString("MQTT client ID", deviceConfig.mqtt_client_id, sizeof(deviceConfig.mqtt_client_id));
  promptString("MQTT username/token", deviceConfig.mqtt_user, sizeof(deviceConfig.mqtt_user));
  promptString("MQTT password", deviceConfig.mqtt_password, sizeof(deviceConfig.mqtt_password));
  promptString("MQTT topic", deviceConfig.mqtt_topic, sizeof(deviceConfig.mqtt_topic));
}

static void setNtrip() {
  promptString("NTRIP host", deviceConfig.ntrip_host, sizeof(deviceConfig.ntrip_host));
  uint32_t port = deviceConfig.ntrip_port;
  promptUInt("NTRIP port", &port);
  deviceConfig.ntrip_port = (uint16_t)port;
  promptString("NTRIP mountpoint", deviceConfig.ntrip_mountpoint, sizeof(deviceConfig.ntrip_mountpoint));
  promptString("NTRIP username", deviceConfig.ntrip_user, sizeof(deviceConfig.ntrip_user));
  promptString("NTRIP password", deviceConfig.ntrip_password, sizeof(deviceConfig.ntrip_password));
}

static void setUpload() {
  promptString("Upload host", deviceConfig.upload_host, sizeof(deviceConfig.upload_host));
  uint32_t port = deviceConfig.upload_port;
  promptUInt("Upload port", &port);
  deviceConfig.upload_port = (uint16_t)port;
  promptString("Upload API key", deviceConfig.upload_api_key, sizeof(deviceConfig.upload_api_key));
}

// Recurses one level deep, which is all the current /data/<subfolder>/file
// layout (see sd_storage.cpp/gnss.cpp) needs - a flat listing of just "/"
// would show nothing but an empty "data" entry now that every real file
// lives under it.
static bool listFilesIn(const char *dirPath) {
  File dir = SD_MMC.open(dirPath);
  if (!dir) {
    return false;
  }
  File f = dir.openNextFile();
  while (f) {
    if (f.isDirectory()) {
      listFilesIn(f.path());
    } else {
      Serial.printf("%-40s %10lu bytes\n", f.path(), (unsigned long)f.size());
    }
    f = dir.openNextFile();
  }
  dir.close();
  return true;
}

static void listFiles() {
  if (!listFilesIn("/")) {
    Serial.println(F("SD card not available."));
  }
}

static void downloadFile() {
  Serial.print(F("Filename to download: "));
  String name = readLine();
  if (name.length() == 0) {
    return;
  }
  String path = name.startsWith("/") ? name : "/" + name;

  File f = SD_MMC.open(path, FILE_READ);
  if (!f) {
    Serial.println(F("File not found."));
    return;
  }

  Serial.printf("BEGIN %s %lu\n", name.c_str(), (unsigned long)f.size());
  uint8_t buf[512];
  size_t n;
  while ((n = f.read(buf, sizeof(buf))) > 0) {
    Serial.write(buf, n);
  }
  f.close();
  Serial.println();
  Serial.println(F("END"));
}

void runFieldService() {
  Serial.println(F("Field service mode - config menu + file download over this USB serial connection."));

  while (digitalRead(FIELD_MODE_SELECT_PIN) == LOW) {
    printMenu();
    String choice = readLine();

    if (choice == "1") {
      showConfig();
    } else if (choice == "2") {
      promptUInt("Measurement interval (s)", &deviceConfig.measurement_interval_s);
    } else if (choice == "3") {
      promptUInt("Fix timeout (s)", &deviceConfig.fix_timeout_s);
    } else if (choice == "4") {
      promptFloat("Good accuracy (m)", &deviceConfig.good_accuracy_m);
    } else if (choice == "5") {
      setMqtt();
    } else if (choice == "6") {
      setNtrip();
    } else if (choice == "7") {
      setUpload();
    } else if (choice == "8") {
      listFiles();
    } else if (choice == "9") {
      downloadFile();
    } else if (choice == "A" || choice == "a") {
      promptUInt("PPK minimum recording duration (s)", &deviceConfig.ppk_min_duration_s);
    } else if (choice == "B" || choice == "b") {
      promptUInt("Reference-position averaging window (s)", &deviceConfig.reference_window_s);
    } else if (choice == "D" || choice == "d") {
      promptFloat("Low-battery cutoff (V)", &deviceConfig.low_battery_cutoff_v);
    } else if (choice == "C" || choice == "c") {
      // Antenna needs to already be at its final, permanent mounting
      // position before this is confirmed - the average this starts
      // collecting becomes the permanent ground-movement benchmark for the
      // life of the station, see reference_position.h.
      Serial.printf("This will average GNSS fixes for %u s (back-to-back bursts, not the normal "
                    "%u s cycle interval) to establish the permanent reference position. Confirm "
                    "the antenna is at its final mounting location. Proceed? [y/N]: ",
                    (unsigned)deviceConfig.reference_window_s,
                    (unsigned)deviceConfig.measurement_interval_s);
      String confirm = readLine();
      confirm.toLowerCase();
      if (confirm == "y" || confirm == "yes") {
        saveDeviceConfig(); // persist reference_window_s in case it was just changed above
        referencePositionStartCommissioning();
        Serial.println(F("Commissioning started - rebooting into normal operation."));
        Serial.flush();
        delay(100);
        ESP.restart();
      } else {
        Serial.println(F("Cancelled."));
      }
    } else if (choice == "0") {
      saveDeviceConfig();
      Serial.println(F("Configuration saved."));
      break;
    } else {
      Serial.println(F("Unknown choice."));
    }
  }

  Serial.println(F("Exiting field service mode - rebooting into normal operation."));
  Serial.flush();
  delay(100);
  ESP.restart();
}

void checkFieldModeTrigger() {
  static unsigned long lowSince = 0;
  const unsigned long DEBOUNCE_MS = 200;

  if (digitalRead(FIELD_MODE_SELECT_PIN) == LOW) {
    if (lowSince == 0) {
      lowSince = millis();
    } else if (millis() - lowSince >= DEBOUNCE_MS) {
      Serial.println(F("Service cable detected - rebooting into field service mode."));
      Serial.flush();
      delay(100);
      ESP.restart();
    }
  } else {
    lowSince = 0;
  }
}
