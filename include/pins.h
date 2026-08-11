#pragma once

// Centralized pin/hardware configuration for the Walter ESP32-S3 field
// station (ZED-F9P RTK GNSS + Raspberry Shake RS3D geophone + SD storage).
// Numbers below match the carrier board's pin table exactly.

// --- GNSS (ZED-F9P), UART2, 5V rail, SEL jumper unsoldered (UART1 on F9P) ---
#define GPS_RX_PIN 16   // F9P UART1 TXO -> ESP32 UART2 RXD
#define GPS_TX_PIN 17   // F9P UART1 RXI <- ESP32 UART2 TXD
#define GPS_BAUD 38400
#define GPS_EXTINT_PIN 18 // wake from UBX-RXM-PMREQ backup mode (gnss.h)

// --- Geophone (Raspberry Shake RS3D digitizer), UART1, receive-only ---
#define GEOPHONE_RX_PIN 15
#define GEOPHONE_BAUD 230400 // see raspberry_shake.md - fixed, not configurable

// --- SD card, native SDMMC 1-bit mode (not SPI) ---
#define SD_CLK_PIN 12
#define SD_CMD_PIN 11
#define SD_D0_PIN 13
// D3 must stay pulled up even in 1-bit mode, or the card silently falls
// back to SPI mode - this pin is wired, not bit-banged by firmware.
#define SD_D3_PIN 38
#define SD_CARD_DETECT_PIN 39 // wired; not currently read by firmware
// 3V3_EN: switched 3.3V rail that powers the SD card. Active LOW (drive
// low to enable). Held enabled continuously - geophone logging never
// stops, so there's no per-cycle power-gating to do here.
#define SD_POWER_EN_PIN 0

// --- Field service cable ---
// Grounded by the setup cable's 4th wire to enter config-menu + SD-access
// mode (see field_service.h). Console runs over native USB (CDC), not the
// exposed-but-unused UART0 (IO44/IO43).
#define FIELD_MODE_SELECT_PIN 40

// Momentary button to GND - flushes/closes every SD stream and cuts power
// to the card's rail (SD_POWER_EN_PIN) so a technician can pull the card
// without a hot-unplug mid-transaction (see sdCheckEjectButton() in
// sd_storage.cpp). Re-inserting a card is picked up automatically by the
// existing sdCheckCardPresence() remount path - no separate "resume" step.
#define SD_EJECT_BUTTON_PIN 41

// --- Battery monitoring ---
#define BATTERY_EXTERNAL_ADC_PIN 2
#define BATTERY_INTERNAL_ADC_PIN 1
