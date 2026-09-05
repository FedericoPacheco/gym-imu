# ADR-9: refine signal transformation steps with simple methods

## Status

Superseded by ADR-10

## Date

2026/07/18

## Context

I implemented a first iteration of the signal processing pipeline outlined in [ADR-8](8-signal-transformer-steps.md), except for the low pass step. I've observed that the velocity estimation is very unreliable (e.g. >10m/s in less than 20 seconds).

## Decision

Use simple methods to try to correct the behavior before considering more complex methods and major changes to the pipeline:

- Online bias estimation for both the gyroscope and and gravity-free acceleration (after gravity removal): when stationality is detected, update the bias estimate with a low-pass filter and subtract it from the current sample.

- Improve the affine acceleration calibration matrix by using captures at additional orientations: gravity present on two of the IMU's axes at a time.

- Add a small low-pass filter for both the gyroscope and acceleration to benefit the downstream processing steps.

- Try out trapezoidal integration instead of Euler in the complementary filter for orientation estimation.

- Explore better zero-velocity update strategies to stabilize the estimation.

## Consequences

### Positive

I attempt to maintain the pipeline simple, easy to understand and fast, and understand the problem better before considering more complex methods and major changes to the pipeline.

### Negative

I waste more time in case the simple methods don't work.
