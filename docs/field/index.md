# Field use

This section is for whoever configures, deploys and services a station.
You need a laptop with a serial terminal and a USB-C cable. A PlatformIO
install works, and so does any terminal at 115200 baud: PuTTY, `screen`, or
the Arduino serial monitor.

| Task | Page |
|---|---|
| Change settings: intervals, credentials, SEED codes, battery cutoff | [Configuration menu](configuration.md) |
| Set the ground-movement benchmark after installation | [Commissioning](commissioning.md) |
| Prepare a trip, install, and leave a station running | [Deployment checklist](deployment-checklist.md) |
| Collect the seismic and GNSS data | [Getting data off](data-retrieval.md) |
| Fix a station that misbehaves | [Troubleshooting](troubleshooting.md) |

## What the station does on its own

Once powered, the station needs no attention:

- **Continuously:** records the geophone to the SD card.
- **Every 10 minutes (default):** wakes the GNSS and modem, gets a fix,
  logs it, sends telemetry, then puts both back to sleep.
- **Once a day:** uploads the previous day's positions CSV, if an upload
  backend is configured.
- **Low battery:** below the cutoff it skips GNSS and modem bursts, but
  keeps recording the geophone.
- **SD almost full** (under 1 GB free): stops recording the geophone and
  keeps logging positions.
- **Stall:** if the firmware hangs for 60 s, a watchdog reboots it.
