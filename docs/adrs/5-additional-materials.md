# ADR-id: buy additional materials for device assembly

## Status

Accepted

## Date

2026/01/25

## Context

I've so far coded classes for the LED, Button, and MPU-6050 sensor to be used in the device. However, testing them has been especially difficult since the microcontroller and sensor aren't making addecuate contact with their headers due not being soldered, and that in turn produces hardware bugs. Initially I didn't solder them because I thought I wouldn't use headers in the final design, but this may not be the case. After making sure the circuit works, I can move from a breadboard to a protoboard/perfboard, where using headers seems acceptable.
I've also inspected my current tooling and materials:

- Soldering iron: very old, gets cold quickly, needs non-trivial restoration to work properly (watch: <https://www.youtube.com/watch?v=5zMtSMqqk0c>), no stand (potentially unsafe), no tip replacement.
- Solder: opaque, probably leaded (toxic), no flux core.
- No reworking or desoldering tools.
- New multimeter: good enough for my needs, but wires got cut with little use.

## Decision

Buy or find the following items:

- Solid copper wire (NOT stranded) 22-26 AWG (diameter: 0.643mm -  0.405mm)
- Small protoboard / perfboard
- New pair of wires for multimeter
- Solder 60/40 tin-lead or lead-free, with flux-core, around 1mm
- Solder iron, with adjustable temperature if possible, 25W-40W
- Solder sucker and desoldering braid
- Safety lenses with lateral protection
- N95/KF94/FFP2/P100 mask for soldering fumes

Optional:

- JST 2.0 PH connector for LiPo battery
- Nitrile gloves

For now solder headers to my microcontroller and MPU-6050 sensor to facilitate my work. When I finish implementing and testing everything (LED, Button, MPU6050, RF antenna), migrate to protoboard/perfboard as well as start to think about the case/enclosure.

## Consequences

### Positive

I have better tooling to work effectively on the project.

### Negative

The project gets more expensive (about 30-35 USD more).
