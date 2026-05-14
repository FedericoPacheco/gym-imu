# ADR-8: choose initial signal transformation steps

## Status

Accepted

## Date

2026/05/16

## Context

I'm close to finishing a book on DSP, *The Scientist and Engineer's Guide to Digital Signal Processing* by Smith, as well as scattered material on the internet, and I'm kind of lost about where to start or what to do.

## Decision

I'll begin with a minimal and as simple as possible processing pipeline that allows me to validate the overall pipeline and iterate on it to address errors and limitations.

Approach: work offline on each step on a different Jupyter notebook, evaluating it first on fake data and later on real captures. Later, implement all steps at once on one notebook and validate the complete pipeline. Finally, implement each step on the device using the Strategy pattern repeatedly and call them sequentially from the IMUSignalProcessor class. Evalute performance between steps. Perform a final capture with the device and validate the complete pipeline on it.

Steps:

Input: raw single precision acceleration and gyroscope data on each axis + sequence numbers.

1. Calibrate accelerometer with affine transformation and gyroscope by subtracting stationary bias (already done).

2. Mild low-pass both signals to remove noise with a fast, simple filter (e.g. moving average, single-pole IIR).

3. Compute orientation for roll and pitch with a complementary filter. Use yaw directly from the gyroscope, since it can't be estimated with the accelerometer.  

4. Rotate the signals using the computed orientation to later subtract gravity from the acceleration.

“Integrate acceleration with respect to time using the device sampling period and a fast method (e.g. trapezoidal rule) to get velocity. Apply a drift control strategy (e.g. detrending, zero velocity update on stationary periods). Leave angular velocity as-is.

Output: processed single precision acceleration and gyroscope data on each axis + sequence numbers.

## Consequences

### Positive

I avoid overengineering, too long development times and overwhelming myself with complexity.

### Negative

I might have to iterate more on the processing pipeline, but that's probably expected and I'm not time-constrained.
