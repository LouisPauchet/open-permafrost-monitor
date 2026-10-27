# How it works

These pages explain the firmware's mechanisms with diagrams: what runs
when, why, and what the trade-offs are. Read them before changing the
firmware, or when you need to interpret the data precisely.

| Page | Question it answers |
|---|---|
| [Architecture](architecture.md) | What are the parts, tasks and data flows? |
| [GNSS fix cycle](scheduler.md) | What exactly happens during a measurement burst? |
| [Power management](power.md) | What sleeps when, and what does low battery do? |
| [Timing](timing.md) | How do MiniSEED samples get UTC timestamps? |
| [Storage](storage.md) | How is the SD card shared between loggers, eject and USB? |
| [Configuration model](configuration-model.md) | Where does each setting come from, and which value wins? |
| [Raspberry Shake protocol](raspberry-shake.md) | How does the geophone digitizer talk, and how was that found? |

## Design principles

1. **The geophone never stops.** It streams 24/7 with no way to pause, so
   the CPU never deep-sleeps. Instead, only the power-hungry parts (GNSS
   receiver, modem) are slept between bursts.
2. **Nothing blocking on the main loop.** Seismic capture, SD writes and
   modem jobs each run on their own task. A slow card or a stuck AT command
   can't starve the others.
3. **Survive unattended.** A watchdog reboots the board after 60 s of
   stall. Files are closed before every self-triggered reboot. Progress
   (commissioning, uploads) is kept in flash so a reboot doesn't lose it.
4. **Few settings.** Everything a technician may need is in one menu.
   Everything else is a constant in the code, with a comment explaining the
   value.
