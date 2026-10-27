#include "battery.h"
#include "pins.h"

// EXTERNAL_DIVIDER_RATIO is the real, measured carrier-board divider ratio
// (pin 2) - actual battery voltage = ADC millivolts x this. INTERNAL is
// still an uncalibrated placeholder (2.0 = simple 1:1 divider assumption);
// calibrate once that divider's real resistor values are known, the same
// way this one just was.
static const float EXTERNAL_DIVIDER_RATIO = 6.4545454545454f;
static const float INTERNAL_DIVIDER_RATIO = 2.0f;

void initBattery() {
  analogReadResolution(12);
}

// Averaged: single ESP32 ADC reads are noisy by tens of mV at the pin,
// which the external divider multiplies by ~6.5 - enough to make a pack
// sitting near low_battery_cutoff_v look like it's crossing it.
static const int ADC_SAMPLES = 16;

static float readVoltage(int pin, float dividerRatio) {
  uint32_t sumMv = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sumMv += analogReadMilliVolts(pin); // chip-calibrated (eFuse) millivolts at the pin
  }
  return (sumMv / (float)ADC_SAMPLES / 1000.0f) * dividerRatio;
}

float batteryExternalVoltage() {
  return readVoltage(BATTERY_EXTERNAL_ADC_PIN, EXTERNAL_DIVIDER_RATIO);
}

float batteryInternalVoltage() {
  return readVoltage(BATTERY_INTERNAL_ADC_PIN, INTERNAL_DIVIDER_RATIO);
}
