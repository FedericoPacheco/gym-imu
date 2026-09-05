# ADR-10: Switch to the Mahony orientation filter

## Status

Accepted

## Date

2026/09/04

## Context

I implemented the improvements outlined in the ADR-9, used the Euler kinematical equations on the complementary filter, and identified a flaw on the previously judged as successful pipeline (incorrect axis-wise ZUPTs). Correcting this mistake, the performance of the signal processing isn't enough: velocity estimation still diverges.

The main source of error is the residual acceleration after gravity removal (on the order of $10^{-1}\frac{m}{s^2}$), which is later integrated to obtain velocity. A new orientation filter that improves accuracy (ideally, by an order of magnitude) but still has relatively low computational cost is needed.

There are two main options:

*Mahony*:

Core idea: proportional-Integral (PI) feedback controller on the gravity-direction error.

Pros:

* Explicitly stated on its original paper that is a low-cost filter for embedded use.
* Employs quaternions to represent orientation, avoiding the hard-to-handle singularities I found with Euler angles.
* Incorporates online gyro bias correction naturally.
* Leverages classic control theory principles I've studied at university.
* Provides accurate and stable estimates of orientation even in the presence of external disturbances.
* Integral feedback can be gated with stationary detection to prevent drift during dynamic motions.

Cons:

* Requires tuning $K_{i}$ and $K_{p}$ gains carefully.
* May not shield a 10x improvement in residual acceleration.

*Madgwick*:

Core idea: gradient descent minimizing the mismatch between estimated and measured gravity direction

Pros:

* Original paper states that performance is "comparable to a Kalman-based algorithm", and works well at low sampling rates.
* Relatively low computational cost (higher than Mahony due to Jacobian computation), suitable for embedded systems.
* Also employs quaternions.
* Very good convergence from poor initial orientation estimates.

Cons:

* Requires tuning the $\beta$ parameter carefully, and it may make less intuitive sense from a control theory perspective.
* Original version doesn't provide online gyro bias correction as the Mahony filter.
* Filter may catch fast dynamic motions less effectively than the Mahony filter.

Kalman filter variants (Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), etc.) offer higher accuracy at the cost of increased computational complexity and implementation difficulty, and they will not be considered for now.

Sources:

* GPT-5.6 Terra
* [A Comparative Analysis of Sensor Fusion Algorithms for Miniature IMU Measurements](../misc/A%20Comparative%20Analysis%20of%20Sensor%20Fusion%20Algorithms%20for%20Miniature%20IMU%20Measurements%20-%20Cocoli,%20Badia.pdf)
* [Nonlinear Complementary Filters on the Special Orthogonal Group](../misc/Nonlinear%20Complementary%20Filters%20on%20the%20Special%20Orthogonal%20Group%20-%20Robert%20Mahony.pdf)
* [An efficient orientation filter for inertial and inertial/magnetic sensor arrays](../misc/An%20efficient%20orientation%20filter%20for%20inertial%20and%20inertial-magnetic%20sensor%20arrays%20-%20Sebastian%20Madgwick.pdf)
* [AHRS: Attitude and Heading Reference Systems](https://ahrs.readthedocs.io/en/latest/index.html)
* [AHRS: Madgwick Filter Documentation](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#orientation-from-imu)
* [AHRS: Mahony Filter Documentation](https://ahrs.readthedocs.io/en/latest/filters/mahony.html)

## Decision

Choose the Mahony filter for orientation estimation since it's lower cost, and fits my dynamic motion requirements and existing pipeline better. Read the original paper as well as supporting documentation to understand its implementation and tuning requirements.

## Consequences

### Positive

I increase complexity gradually matching my current understanding of the field.

### Negative

I may waste time on a new filter and not jump to Kalman filter variants immediately.
