# Velocity-Based Training (VBT) System

A system to capture and analyze velocity data during strength training exercises, enabling athletes and coaches to perform:

- Force-velocity curve analysis

- One-repetition maximum (1RM) estimation (maximum intensity an athlete can lift for one repetition) through regression analysis

- Pre-workout fatigue detection by comparison against historical data

- Velocity zones prescriptions for training sessions  

The development of the project was inspired by the chapter "Velocity in the weight room" from the book ["Science and Practice of Strength Training (3rd edition)" by Zatsiorsky, Kraemer and Fry](<https://us.humankinetics.com/products/science-and-practice-of-strength-training-3rd-edition-epub?_pos=2&_sid=e4ce0c3cd&_ss=r>).

## Table of contents

- [Overview](#overview)
- [Device](#device)
  - [Hardware](#hardware)
  - [Firmware](#firmware)
  - [Testing](#testing)
- [Signal Analysis and Processing](#signal-analysis-and-processing)
- [Mobile App](#mobile-app)

## Overview

The system consists of:

- A wearable inertial measurement unit (IMU) device that captures and processes motion data during strength training exercise
- A companion mobile app that receives the results wirelessly, computes summary metrics and displays the results.

See the [C4 Context Diagram](docs/diagrams/c4/context.svg) and [C4 Container Diagram](docs/diagrams/c4/container.svg) for architecture overviews.

Main technologies used:

- Device Hardware: [XIAO ESP32 C3](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/), [MPU6050](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)
- Device Firmware: PlatformIO with [Espressif IoT Development Framework (ESP-IDF)](https://www.espressif.com/en/products/sdks/esp-idf) framework, [FreeRTOS](https://www.freertos.org/Why-FreeRTOS/What-is-FreeRTOS), NimBLE (Bluetooth Low Energy stack), C++20
- Signal analysis and processing: TBD (probably Python for data analysis and visualization)
- Mobile App: TBD (probably TS + React Native)

Key decisions and their rationale are documented in [Architecture Decision Records (ADRs)](docs/adrs/).

## Device

### Hardware

My device will be made primarily by hand using off-the-shelf components, with a custom 3D-printed enclosure. See the [device's schematic](docs/diagrams/schematics/xiao-esp32-c3/schematic.svg), [BOM](docs/diagrams/schematics/xiao-esp32-c3/bom.md) (TBD) and [tools](docs/diagrams/schematics/xiao-esp32-c3/tools.md) (WIP) to build your own device.

### Firmware

The architecture follows a pipes and filter style, as documented in [ADR-7](docs/adrs/7-architecture-style.md) and the [C4 Component Diagram](docs/diagrams/c4/component.svg).

See [setup.md](docs/setup.md) for instructions on how to set up the development environment and flash the device.

### Testing

TBD

## Signal Analysis and Processing

TBD

## Mobile App

TBD
