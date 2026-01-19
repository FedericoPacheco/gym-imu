# ADR-4: choose esp32-mpu-driver library for IMU communication

## Status

Accepted

## Date

2026/01/19

## Context

I need a library to communicate with the MPU6050 sensor to avoid writing low-level I2C code. There a multiple options available:

- Adafruit MPU6050: popular and beginner-friendly. Also requires the Adafruit Unified Sensor library. Lacks DMP ("digital motion processor", a coprocessor inside the MPU6050) support.

- MPU6050_light: lightweight, fast, easy to use and purposely minimalistic. Lacks DMP support. Requires external math.

- Electronic cats MPU6050: feature-rich (DMP, complex math) and actively maintained.

- esp32-mpu-driver: specifically designed for ESP32, supports DMP and advanced math. Less popular and has a steeper learning curve.

Adafruit's, light and Electronic cats libraries are Arduino-based. It can be used on esp IDF, but it requires the Arduino core for ESP32, which adds overhead to the project and may run into compatibility issues if I use both frameworks.

## Decision

Use the esp32-mpu-driver library for IMU communication. It is specifically designed for ESP32, supports DMP and advanced math, and is compatible with ESP-IDF without needing the Arduino core. Implement the Adapter pattern to facilitate future library changes if needed, initially wrapping only esential functionalities to understand the signal.

## Consequences

### Positive

- Full compatibility with ESP IDF.
- Built-in DMP and advanced math support.
- Less risk of needing to switch libraries later.

### Negative

- Stepeer learning curve.
- Less educational value by not implementing the advanced features myself.
