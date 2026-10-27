# Hardware

## Bill of materials

| Part | Reference used | Notes |
|---|---|---|
| Main board | [Walter](https://www.quickspot.io/) ESP32-S3 board | Has the Sequans GM02SP LTE-M/NB-IoT modem on board. WiFi is not used. |
| RTK GNSS receiver | u-blox ZED-F9P breakout (gnss.store **ELT0128**) | Needs a multi-band GNSS antenna. The `SEL` jumper must be **open**; see [Wiring](wiring.md#gnss-zed-f9p). |
| Geophone + digitizer | Raspberry Shake **RS3D** digitizer board with its 3-axis geophones | Only the sensor and digitizer HAT. The ESP32 replaces the Raspberry Pi; see [Raspberry Shake protocol](../how-it-works/raspberry-shake.md). |
| Storage | microSD card + breakout wired for **SPI** | A card-detect switch is optional. See [card sizing](#sd-card-size) below. |
| SD eject button | 1 × momentary push-button | Lets you remove the card safely. |
| Field-service connector | 2-pin header + jumper, or a 4th wire on the service cable | Grounding it opens the configuration menu. |
| SIM card | LTE-M or NB-IoT data SIM | Must work on the bands the GM02SP modem supports, in the deployment country. |
| Battery | 12 V pack (default settings assume **lead-acid**) | The low-battery cutoff defaults to 11.8 V. Change it for other chemistries; see [menu option D](../field/configuration.md#option-reference). |
| Regulator | Pololu **5 V, 5 A** step-down regulator | Makes the 5 V rail for the Walter board, the ZED-F9P and the RS3D. See [Wiring → Power](wiring.md#power). |
| Fuse | Inline fuse + holder on the battery + lead | Size it for your wire gauge and the regulator's input current |
| Battery sense | Resistor divider from the battery to GPIO 2 | The firmware's ratio is 6.4545 (calibrated on the reference board). See [Wiring](wiring.md#battery-monitoring). |
| Enclosure parts | 3D-printed parts from `cao/` | See [Enclosure parts](enclosure.md). |
| USB-C cable | Data-capable | For flashing, the console and service mode. |

!!! info "Power supply"
    The 12 V battery feeds a **fuse**, then a **Pololu 5 V / 5 A
    step-down regulator**. Its 5 V output powers the Walter board, the
    ZED-F9P and the RS3D. The SD card runs from the Walter's switched 3.3 V
    output. Full diagram: [Wiring](wiring.md#wiring-diagram).

## Optional external services

The station records everything locally without any of these. Each one adds
a capability:

| Service | Adds | Without it |
|---|---|---|
| **NTRIP caster** account (a network-RTK/VRS mount) | RTK corrections, so fixes reach centimetre accuracy within one burst | Fixes stay at autonomous/DGPS quality. Raw UBX is still recorded for later post-processing (PPK). |
| **MQTT broker** with TLS (e.g. [ThingsBoard](https://thingsboard.io/)) | Live position, battery and SD telemetry after every burst | No remote visibility |
| **Upload backend** you run yourself | Daily copy of the positions CSV to S3-compatible storage | The CSV stays on the SD card |

!!! warning "TLS certificates"
    The firmware trusts only one root certificate: **ISRG Root X1** (Let's
    Encrypt). The MQTT broker's and the upload backend's certificates must
    chain to it.

## SD card size

| Data | Approximate size per day |
|---|---|
| MiniSEED, Steim2 (default) | about 80–120 MB (a third to a half of the uncompressed size) |
| MiniSEED, uncompressed int32 | about 230 MB |
| GNSS raw UBX + RTCM | roughly 0.3–0.5 MB per burst, so about 50–70 MB/day at a 10-minute interval (an estimate; it depends on satellites in view and burst length) |
| Positions CSV | a few tens of kB |

A **32 GB** card lasts roughly 4 months with uncompressed MiniSEED, and
longer with Steim2. When free space drops below **1 GB**, the firmware
stops writing MiniSEED, so the positions CSV and GNSS files can keep going.
See [SD card layout](../data/sd-layout.md).
