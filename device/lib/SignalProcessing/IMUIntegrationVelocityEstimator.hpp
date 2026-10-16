#pragma once
#include <IMUSensorPort.hpp>
#include <IMUVelocityEstimator.hpp>
#include <IMUWindowedStationaryDetector.hpp>
#include <Quaternion.hpp>

typedef LinearAxes VelocitySample; // m/s

class IMUIntegrationVelocityEstimator : public IMUVelocityEstimator {
public:
  IMUIntegrationVelocityEstimator(float dt) : IMUVelocityEstimator(dt) {
    this->reset();
  }

  VelocitySample estimate(IMUSample &sample, Quaternion &orientation) override {
    if (isFirst) {
      isFirst = false;
      auto aB = std::array<float, 3>{sample.a.x - A_STATIONARY_BIAS[0],
                                     sample.a.y - A_STATIONARY_BIAS[1],
                                     sample.a.z - A_STATIONARY_BIAS[2]};
      auto aW = orientation.rotate(aB);
      sample.a.x = aW[0];
      sample.a.y = aW[1];
      sample.a.z = aW[2];
      prevAW = aW;
      return VelocitySample{0.0f, 0.0f, 0.0f};
    }

    IMUSample originalSample = sample;
    bool isStationary =
        stationaryDetector.isStationaryPreVelocity(originalSample);
    this->calibrateAccelOnline(sample, isStationary);
    auto aW = orientation.rotate(
        std::array<float, 3>{sample.a.x, sample.a.y, sample.a.z});
    auto vW =
        std::array<float, 3>{prevVW[0] + 0.5f * (aW[0] + prevAW[0]) * this->dt,
                             prevVW[1] + 0.5f * (aW[1] + prevAW[1]) * this->dt,
                             prevVW[2] + 0.5f * (aW[2] + prevAW[2]) * this->dt};

    if (isStationary) {
      vW[0] = ALPHA_V * vW[0];
      vW[1] = ALPHA_V * vW[1];
      vW[2] = ALPHA_V * vW[2];
    } else if (stationaryDetector.isGyroOnStationaryBandPreVelocity(
                   originalSample)) {
      if (std::abs(originalSample.a.x - A_STATIONARY_BIAS[0]) <=
          A_STATIONARY_TOL[0])
        vW[0] = prevVW[0];
      if (std::abs(originalSample.a.y - A_STATIONARY_BIAS[1]) <=
          A_STATIONARY_TOL[1])
        vW[1] = prevVW[1];
      if (std::abs(originalSample.a.z - A_STATIONARY_BIAS[2]) <=
          A_STATIONARY_TOL[2])
        vW[2] = prevVW[2];
    }

    prevVW = vW;
    prevAW = aW;
    sample.a.x = aW[0];
    sample.a.y = aW[1];
    sample.a.z = aW[2];
    return VelocitySample{vW[0], vW[1], vW[2]};
  }

  void reset() override {
    prevVW = {0.0f, 0.0f, 0.0f};
    prevAW = {0.0f, 0.0f, 0.0f};
    aBias = A_STATIONARY_BIAS;
    stationaryDetector.reset();
    isFirst = true;
  }

private:
  bool isFirst;
  static constexpr float ALPHA_A = 0.967216f;
  static constexpr float ALPHA_V = 0.806013f;
  static constexpr std::array<float, 3> A_STATIONARY_BIAS = {
      0.000370f, 0.018512f, 0.000310f};
  static constexpr std::array<float, 3> A_STATIONARY_TOL = {
      0.030170f, 0.098853f, 0.032412f};

  std::array<float, 3> aBias;
  IMUWindowedStationaryDetector stationaryDetector;
  std::array<float, 3> prevVW;
  std::array<float, 3> prevAW;

  void calibrateAccelOnline(IMUSample &sample, bool isStationary) {
    if (isStationary) {
      aBias[0] = ALPHA_A * aBias[0] + (1 - ALPHA_A) * sample.a.x;
      aBias[1] = ALPHA_A * aBias[1] + (1 - ALPHA_A) * sample.a.y;
      aBias[2] = ALPHA_A * aBias[2] + (1 - ALPHA_A) * sample.a.z;
    }
    sample.a.x -= aBias[0];
    sample.a.y -= aBias[1];
    sample.a.z -= aBias[2];
  }
};
