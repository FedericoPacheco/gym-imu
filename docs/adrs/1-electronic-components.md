# ADR-id: choose electronic components for device

## Status

Accepted

## Date

2026/01/03

## Context

I want to build my own wearable IMU (inertial measurement unit) device to measure velocity from bodyweight or barbell exercises. I need to choose the electronic components for the device, balancing cost, availability, physical size, performance and power consumption. Below is a list of components I am considering, available as of January 2026 on Mercado Libre (1 USD = 1500ARS):

- Microcontroller:
  - ESP32 c3: about 5 USD, BLE (bluetooth low energy), single core RISC V (160MHz), 400kb ram, low power consumption, with FPU, very small size
  - ESP32 (classic): about 7 USD, Bluetooth Classic + BLE, dual core, 512kb ram, dual core Xtensa LX6 (240MHz), 520kb ram, higher power consumption, no FPU, more pins, larger size
  - Arduino Nano 33 BLE: about 65 USD, all in one (microcontroller + bluetooth + IMU)
  - Classic Arduino Uno/Nano: very slow/inferior compared to ESP32 (16 mhz, 8bit, 2kb ram), low storage (32kb flash), no wireless connectivity

- Accelerometer:
  - MPU-6050: about 3 USD, 6 axis (linear + gyro), no magnetometer (drift with time), older
  - MPU-9250: about 10-15 USD, 9 axis (linear + gyro + magnetometer)
  - BNO055: about 40 USD, outputs orientation directly, plug and play
  - ADXL345: about 4 USD, 3 axis only

Note: measuring velocity directly is not possible for a wearable device. It needs different techniques: encoders, drift in radio waves, lasers, gps; and: fixed references, certain environmental conditions, more power, etc.

- Battery: small to medium LiPo (300-1000 mAh, 3.7V, about 8-16USD) + TP4056 (about 2 USD, safe charging, prevent fires) + either step up (3.7V to 5V) or step down (3.7V to 3.3V) voltage regulator (e.g. MT3608, XL6009, AMS1117, NOT powerbanks such as 134N3P, LPO or buck/boost, about 2-4 USD) + 2 pin switch (about 1-2 USD).

## Decision

Buy:

- ESP32 c3 supermini microcontroller: low cost, very small size, good enough performance for signal processing and ODEs and low power consumption.

- MPU-6050 accelerometer: very low cost, 6 axis seems enough for velocity measurement, magnetometer may through wrong measurements due to nearby metalic objects (drift will need to be corrected).

- Small-ish LiPo battery (600 mAh) + TP4056 charging circuit + step down voltage regulator (use 5V's ESP32 pin) + 2 pin switch: small size, good enough capacity, stable system operation (not guaranteed when feeding directly from battery), prevents backfeed to battery when powered from ESP32 USB.

## Consequences

### Positive

- Low cost: total cost of components will be about 25-30 USD.
- Small size: should fit in a small wearable device.
- Good enough for an initial prototype.

### Negative

- May need to change accelerometer later if the sensor is not accurate enough.
- May need to optimize code for performance/power consumption on ESP32 c3.
