#pragma once
#include <Constants.hpp>
#include <IMUEulerOrientationFinder.hpp>
#include <StationaryChecker.hpp>
#include <numbers>

/*
For details, refer to: signal/3-orientation

Misc docs:
* https://en.cppreference.com/cpp/numeric/math/atan2
* https://en.cppreference.com/cpp/numeric/math/copysign
* https://en.cppreference.com/cpp/numeric/math/fmod

*/
class MPU6050ComplementaryOrientationFinder : public IMUEulerOrientationFinder {
private:
  static constexpr float ALPHA_STATIONARY = 0.75f;
  static constexpr float ALPHA_MOVING = 0.95f;
  static constexpr float dt = 1.0f / SAMPLING_FREQUENCY_HZ;
  static constexpr float ATAN2_A_TOL = 0.15;

  IMUSample prevSample;
  EulerOrientationSample prevOrientation;

  void transformHWToSWFrame(IMUSample &sample) {
    IMUSample hwSample = sample;
    sample.a.x = hwSample.a.z;
    sample.a.y = -hwSample.a.x;
    sample.a.z = -hwSample.a.y;
    sample.w.roll = hwSample.w.yaw;
    sample.w.pitch = -hwSample.w.roll;
    sample.w.yaw = -hwSample.w.pitch;
  }

  std::tuple<EulerOrientationSample, EulerOrientationSample>
  computeAccelAngleCandidates(AccelerationSample a) {
    EulerOrientationSample mainAngles, alternativeAngles;

    mainAngles.roll = 180.0f / std::numbers::pi * modAtan2(a.y, a.z);
    mainAngles.pitch = 180.0f / std::numbers::pi *
                       modAtan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
    mainAngles.yaw = 0.0f; // Placeholder

    alternativeAngles.roll = wrapInDegrees(mainAngles.roll + 180.0f);
    alternativeAngles.pitch = wrapInDegrees(
        std::copysign(180.0f - std::abs(mainAngles.pitch), mainAngles.pitch));
    alternativeAngles.yaw = 0.0f; // Placeholder

    return {mainAngles, alternativeAngles};
  }

  float wrapInDegrees(float angle) {
    return mod((angle + 180.0f), 360.0f) - 180.0f;
  }
  // Return result with the sign of the divisor instead of the denominator's, as
  // it happens with fmod(x, y) = x - (y * trunc(x/y))
  float mod(float x, float y) { return x - (y * std::floor(x / y)); }

  float modAtan2(float y, float x) {
    if (std::abs(y) > ATAN2_A_TOL) {
      return std::atan2(y, x);
    } else {
      return std::atan2(0, x);
    }
  }

  float angleDistanceInDegrees(float angle1, float angle2) {
    return std::abs(wrapInDegrees(angle1 - angle2));
  }

  EulerOrientationSample integrateTrapezoidGyro(AngularVelocitySample w) {
    EulerOrientationSample predAngleGyro;
    predAngleGyro.roll =
        prevOrientation.roll + 0.5f * (prevSample.w.roll + w.roll) * dt;
    predAngleGyro.pitch =
        prevOrientation.pitch + 0.5f * (prevSample.w.pitch + w.pitch) * dt;
    predAngleGyro.yaw =
        prevOrientation.yaw + 0.5f * (prevSample.w.yaw + w.yaw) * dt;
    return predAngleGyro;
  }

public:
  EulerOrientationSample find(IMUSample &sample) override {
    transformHWToSWFrame(sample);
    EulerOrientationSample predAngleAccel, predAngleGyro, predAngleCompl;

    if (sample.seq > 1) {
      predAngleGyro = integrateTrapezoidGyro(sample.w);

      auto [mainPredAngleAccel, altPredAngleAccel] =
          computeAccelAngleCandidates(sample.a);
      float mainError =
          angleDistanceInDegrees(mainPredAngleAccel.roll, predAngleGyro.roll) +
          angleDistanceInDegrees(mainPredAngleAccel.pitch, predAngleGyro.pitch);
      float altError =
          angleDistanceInDegrees(altPredAngleAccel.roll, predAngleGyro.roll) +
          angleDistanceInDegrees(altPredAngleAccel.pitch, predAngleGyro.pitch);
      if (mainError <= altError) {
        predAngleAccel = mainPredAngleAccel;
      } else {
        predAngleAccel = altPredAngleAccel;
      }

      // Make continuous
      predAngleAccel.roll =
          predAngleGyro.roll +
          wrapInDegrees(predAngleAccel.roll - predAngleGyro.roll);
      predAngleAccel.pitch =
          predAngleGyro.pitch +
          wrapInDegrees(predAngleAccel.pitch - predAngleGyro.pitch);

      if (StationaryChecker::isStationaryPreOrientation(sample)) {
        predAngleCompl.roll = ALPHA_STATIONARY * predAngleGyro.roll +
                              (1.0f - ALPHA_STATIONARY) * predAngleAccel.roll;
        predAngleCompl.pitch = ALPHA_STATIONARY * predAngleGyro.pitch +
                               (1.0f - ALPHA_STATIONARY) * predAngleAccel.pitch;
      } else {
        predAngleCompl.roll = ALPHA_MOVING * predAngleGyro.roll +
                              (1.0f - ALPHA_MOVING) * predAngleAccel.roll;
        predAngleCompl.pitch = ALPHA_MOVING * predAngleGyro.pitch +
                               (1.0f - ALPHA_MOVING) * predAngleAccel.pitch;
      }
      predAngleCompl.yaw = predAngleGyro.yaw;

      prevOrientation = predAngleCompl;
      prevSample = sample;

    } else {
      auto [mainAngles, alternativeAngles] =
          computeAccelAngleCandidates(sample.a);
      prevSample = sample;
      prevOrientation = mainAngles;
      predAngleCompl = mainAngles;
    }

    return predAngleCompl;
  }
};
