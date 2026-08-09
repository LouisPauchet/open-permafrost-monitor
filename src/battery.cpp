#include "battery.h"
#include "pins.h"

// Placeholder divider ratios - calibrate once the real hardware's voltage
// divider resistor values are known. A ratio of 2.0 assumes a simple 1:1
// divider (actual voltage = 2x what the ADC pin reads).
static const float EXTERNAL_DIVIDER_RATIO = 2.0f;
static const float INTERNAL_DIVIDER_RATIO = 2.0f;

void initBattery() {
  analogReadResolution(12);
}

static float readVoltage(int pin, float dividerRatio) {
  uint32_t mv = analogReadMilliVolts(pin); // chip-calibrated (eFuse) millivolts at the pin
  return (mv / 1000.0f) * dividerRatio;
}

float batteryExternalVoltage() {
  return readVoltage(BATTERY_EXTERNAL_ADC_PIN, EXTERNAL_DIVIDER_RATIO);
}

float batteryInternalVoltage() {
  return readVoltage(BATTERY_INTERNAL_ADC_PIN, INTERNAL_DIVIDER_RATIO);
}
