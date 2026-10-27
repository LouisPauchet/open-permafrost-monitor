# Enclosure parts

Mechanical parts are in
[`cao/`](https://github.com/LouisPauchet/open-permafrost-monitor/tree/main/cao)
in three forms:

- **CATIA source:** `.CATPart`, `.CATProduct`
- **Mesh for printing:** `.stl`
- **Slicer projects:** `.3mf`, plus one ready-to-print `.bgcode`

When a part has several versions, print the **highest-numbered** one. Older
versions are kept for history.

| Part | Print this | Source | Purpose |
|---|---|---|---|
| Antenna support | `cao/AntennaSupport/AntennaSupport_v2.stl` (`.3mf`) | `AntennaSupport.CATPart` | Mounts the GNSS antenna |
| Geophone holder | `cao/RaspberryShake/GeophonesHolder_v3.4.stl` (`.3mf`) | `GeophonesHolder_v3.CATPart` | Holds the three geophones in the box |
| GPS board mount A | `cao/RaspberryShake/GPSBoardMountA.stl` | `GPSBoardMountA.CATPart` | Mounts the ZED-F9P breakout |
| GPS board mount B | `cao/RaspberryShake/GPSBoardMountB.stl` | `GPSBoardMountB.CATPart` | Second half of the F9P mount (both halves: `GPSBoardMountA&B.3mf`) |
| Box bottom | (CATIA source only) | `Box_bottom.CATPart` | Reference geometry |
| Full assembly | — | `RaspberryShake.CATProduct` | How the parts fit together |

!!! tip "Ready-to-print file"
    `GeophonesHolder_v3.4_0.4n_0.2mm_PLA_COREONE_2h34m.bgcode` is pre-sliced
    for a **Prusa Core One** with a 0.4 mm nozzle, 0.2 mm layers and PLA
    (about 2 h 34 min). Use the `.3mf` or `.stl` for any other printer or
    material.

!!! note "Outdoor material"
    PLA softens in sun and becomes brittle in deep cold. For parts exposed
    outdoors, consider PETG or ASA.
