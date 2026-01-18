# ADR-3: Choose PlatformIO + ESP IDF for Development Framework

## Status

Accepted

## Date

2026/01/18

## Context

Building a portable IMU device for gym exercise tracking requires balancing ease of development with low-level control for BLE transmission, sensor fusion, and power efficiency. Arduino is too abstracted and it's IDE lacks advanced features like dependency management, debugging or tests. ESP IDF seems unnecessarily complex for quick prototyping. PlatformIO sits in the middle and offers support for both Arduino and ESP IDF frameworks, integrated into VS Code.

## Decision

Use PlatformIO to manage the project + ESP IDF framework to code close enough to the metal.

## Consequences

### Positive

- Faster setup and library management via PlatformIO.
- Full ESP IDF access for advanced features like FreeRTOS, BLE, and IMU algorithms.
- Better debugging and testing support in VS Code.
- Scalable for future project expansions.

### Negative

- Slight learning curve for PlatformIO coming from the Arduino IDE.
- Potential overhead from PlatformIO's abstractions, though minimal.
