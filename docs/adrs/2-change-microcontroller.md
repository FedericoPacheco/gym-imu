# ADR-2: change microcontroller for device

## Status

Accepted

## Date

2026/01/07

## Context

I've found another variant of the ESP32 C3: XIAO by seeedstudio, which includes the same processor, but features a battery charging circuit (ETA4054S2F) and pins for connecting a LiPo battery directly. The cost of the microcontroller is higher, but it roughly matches the cost of the MT3608 + TP4056 + ESP32 C3 supermini.

## Decision

Switch microcontrollers to the XIAO ESP32 C3 variant by seeedstudio and drop MT3608 + TP4056 + ESP32 C3 supermini.

The schematic is available at: [xiao-esp32-c3 schematic](../schematics/xiao-esp32-c3.puml).

## Consequences

### Positive

- Simplifies the design: fewer components to solder and connect.
- Better RF performance: the XIAO ESP32 C3 external antenna should perform better than the ESP32 C3 supermini's PCB antenna.

### Negative

- The antenna requires to be exposed outside the case, requiring the surface to at least match its size.
