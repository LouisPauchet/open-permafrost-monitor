# Deployment checklist

Print this page and take it to the field.

## Before leaving (lab)

- [ ] Firmware flashed; [first boot](../build/first-boot.md) bench checklist passed
- [ ] SIM active; `Modem: registered.` seen on the bench
- [ ] NTRIP (`6`), MQTT (`5`), upload (`7`) set and tested: a bench burst reaches `RTK FIXED`, MQTT message received
- [ ] **Station code set** (`F`), unique for this site
- [ ] Interval (`2`), NTRIP every N (`E`), battery cutoff (`D`) set for this site's power budget
- [ ] External battery reading in option `1` matches a multimeter (otherwise [calibrate](../build/wiring.md#battery-monitoring))
- [ ] SD card has enough space for the visit interval (see [card sizing](../build/hardware.md#sd-card-size)); spare card packed
- [ ] Service jumper/cable, USB-C cable, laptop with serial terminal packed
- [ ] Site log prepared: station code, SIM ID, card ID, settings

## On site

- [ ] Mount the GNSS antenna in its **final** position with a clear sky view
- [ ] Place the geophone box on firm ground, levelled, oriented as your survey requires (note the orientation)
- [ ] Connect power; check the boot log: SD mounted, GNSS chars > 0
- [ ] Watch one full burst: fix reached, `SDdrop=0B`
- [ ] Start [commissioning](commissioning.md) (`C`); note the time
- [ ] Remove the service jumper **after** choosing `0` or `C`
- [ ] Confirm telemetry arrives at the broker (phone or remote colleague)
- [ ] Close the enclosure; photograph the installation

## Service visit

- [ ] Before touching anything, record the station state: telemetry, `sd_free_mb`, battery
- [ ] Enter service mode; copy `data/` off the card (or swap the card; see [Getting data off](data-retrieval.md))
- [ ] Check settings with `1`; reference position still `LOCKED`
- [ ] Exit with `0`; watch the boot log and one burst
- [ ] Log the visit time (the menu period is a recording gap)
