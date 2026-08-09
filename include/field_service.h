#pragma once
#include <Arduino.h>

// Field-maintenance mode: entered whenever FIELD_MODE_SELECT_PIN reads
// grounded (the service cable's 4th wire) - checked once at boot
// (fieldModeRequested()) and continuously during normal operation
// (checkFieldModeTrigger()), so plugging the cable in is enough on its
// own; no physical reset needed. Runs a blocking text menu over Serial
// (native USB CDC) for device configuration and for downloading files off
// the SD card. Unplugging the cable (or choosing "0) Save and exit")
// reboots straight back into normal operation.
//
// NOTE: true USB mass-storage (MSC) passthrough to the SD card was
// considered (per the original plan) but dropped - the installed Arduino
// SD_MMC library keeps its underlying sdmmc_card_t handle `protected`, with
// no public API to bridge it to USBMSC's raw sector read/write callbacks
// without forking the library. File download over the same CDC serial
// connection is simpler and avoids two things (firmware's FATFS mount and
// a host OS) writing the card at once. See runFieldService()'s "download"
// command: it prints "BEGIN <name> <size>", streams raw bytes, then "END".

bool fieldModeRequested(); // reads FIELD_MODE_SELECT_PIN; call once at boot

// Never returns to its caller: this is a hard boundary, not a mode you fall
// out of mid-boot. Either the operator saves/exits or the cable is pulled,
// and either way this ends in ESP.restart() - the next boot's
// fieldModeRequested() check is what actually leaves field mode.
void runFieldService();

// Call every loop() iteration during normal operation. Debounced; reboots
// (via runFieldService(), see above) the moment the service cable grounds
// the pin, so a technician never needs to find/press a physical reset.
void checkFieldModeTrigger();
