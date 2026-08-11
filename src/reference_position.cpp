#include "reference_position.h"
#include "device_config.h"
#include <Preferences.h>
#include <math.h>

static Preferences prefs;
static const char *NVS_NAMESPACE = "refpos";

// WGS84 ellipsoid constants (meters, dimensionless) - used for the local
// tangent-plane (East/North/Up) displacement calc below.
static const double WGS84_A = 6378137.0;
static const double WGS84_E2 = 0.00669437999014;

static bool locked = false;
static double refLat = 0, refLon = 0, refAlt = 0;

static bool commissioning = false;
static uint32_t sampleCount = 0;
static double meanLat = 0, meanLon = 0, meanAlt = 0;
// Accumulated wall-clock seconds spent commissioning, persisted so a reboot
// mid-commissioning (this project's own history says these happen) doesn't
// reset the clock to zero and never finish - only this boot's own session
// delta gets added each time, not re-counted from scratch.
static uint32_t elapsedS = 0;
static unsigned long lastUpdateMs = 0;

static void persist() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putBool("locked", locked);
  if (locked) {
    prefs.putDouble("ref_lat", refLat);
    prefs.putDouble("ref_lon", refLon);
    prefs.putDouble("ref_alt", refAlt);
  }
  prefs.putBool("commissioning", commissioning);
  prefs.putUInt("count", sampleCount);
  prefs.putDouble("mean_lat", meanLat);
  prefs.putDouble("mean_lon", meanLon);
  prefs.putDouble("mean_alt", meanAlt);
  prefs.putUInt("elapsed_s", elapsedS);
  prefs.end();
}

void referencePositionInit() {
  prefs.begin(NVS_NAMESPACE, true);
  locked = prefs.getBool("locked", false);
  if (locked) {
    refLat = prefs.getDouble("ref_lat", 0);
    refLon = prefs.getDouble("ref_lon", 0);
    refAlt = prefs.getDouble("ref_alt", 0);
  }
  commissioning = prefs.getBool("commissioning", false);
  sampleCount = prefs.getUInt("count", 0);
  meanLat = prefs.getDouble("mean_lat", 0);
  meanLon = prefs.getDouble("mean_lon", 0);
  meanAlt = prefs.getDouble("mean_alt", 0);
  elapsedS = prefs.getUInt("elapsed_s", 0);
  prefs.end();

  if (locked) {
    Serial.printf("Reference position: locked at %.8f, %.8f, %.2fm\n", refLat, refLon, refAlt);
  } else if (commissioning) {
    Serial.printf("Reference position: commissioning resumed (%u samples, %lu/%lu s elapsed).\n",
                  (unsigned)sampleCount, (unsigned long)elapsedS,
                  (unsigned long)deviceConfig.reference_window_s);
  } else {
    Serial.println(F("Reference position: not commissioned yet."));
  }
}

void referencePositionStartCommissioning() {
  locked = false;
  commissioning = true;
  sampleCount = 0;
  meanLat = meanLon = meanAlt = 0;
  elapsedS = 0;
  lastUpdateMs = 0;
  persist();
}

bool referencePositionIsCommissioning() {
  return commissioning;
}

bool referencePositionIsLocked() {
  return locked;
}

void referencePositionUpdate(double lat, double lon, double alt) {
  if (!commissioning || locked) {
    return;
  }

  unsigned long now = millis();
  if (lastUpdateMs != 0) {
    elapsedS += (now - lastUpdateMs) / 1000UL;
  }
  lastUpdateMs = now;

  // Incremental (Welford-style) mean - numerically stable, and avoids
  // needing to store every sample just to average them.
  sampleCount++;
  meanLat += (lat - meanLat) / sampleCount;
  meanLon += (lon - meanLon) / sampleCount;
  meanAlt += (alt - meanAlt) / sampleCount;

  if (elapsedS >= deviceConfig.reference_window_s) {
    locked = true;
    commissioning = false;
    refLat = meanLat;
    refLon = meanLon;
    refAlt = meanAlt;
    Serial.printf("Reference position: LOCKED after %u samples over %lu s -> %.8f, %.8f, %.2fm\n",
                  (unsigned)sampleCount, (unsigned long)elapsedS, refLat, refLon, refAlt);
  }

  persist();
}

bool referencePositionDisplacement(double lat, double lon, double alt, float *northM, float *eastM,
                                    float *upM) {
  if (!locked) {
    return false;
  }

  double latRad = refLat * DEG_TO_RAD;
  double sinLat = sin(latRad);
  double denom = sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
  double N = WGS84_A / denom;                                    // prime vertical radius
  double M = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom); // meridional radius

  double dLatRad = (lat - refLat) * DEG_TO_RAD;
  double dLonRad = (lon - refLon) * DEG_TO_RAD;

  *northM = (float)(dLatRad * M);
  *eastM = (float)(dLonRad * N * cos(latRad));
  *upM = (float)(alt - refAlt);
  return true;
}

uint32_t referencePositionElapsedS() {
  return elapsedS;
}

uint32_t referencePositionSampleCount() {
  return sampleCount;
}

bool referencePositionGet(double *lat, double *lon, double *alt) {
  if (!locked) {
    return false;
  }
  *lat = refLat;
  *lon = refLon;
  *alt = refAlt;
  return true;
}
