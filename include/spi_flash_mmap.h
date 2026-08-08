#pragma once

// Compatibility shim: WalterModem 1.5.0 unconditionally includes
// <spi_flash_mmap.h> (an ESP-IDF 5.x header, split out of esp_spi_flash.h)
// even though this platform's bundled ESP-IDF headers predate that split.
// See https://github.com/QuickSpot/walter-arduino/issues/142 - this file
// shadows the missing header (PlatformIO puts include/ on the search path
// ahead of the toolchain's own headers) until upstream adds proper
// ESP_IDF_VERSION guards.

#include <esp_partition.h>
#include <esp_vfs_fat.h>

#ifndef esp_vfs_fat_spiflash_mount_rw_wl
#define esp_vfs_fat_spiflash_mount_rw_wl esp_vfs_fat_spiflash_mount
#endif
