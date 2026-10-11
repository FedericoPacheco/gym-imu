#pragma once
#include <Constants.hpp>
#include <IMUGravityRemover.hpp>
#include <numbers>

/*
Misc docs:
* https://en.cppreference.com/cpp/numeric/math/sin
*/

class IMUEulerGravityRemover : public IMUGravityRemover<EulerOrientationSample> {
public:
  void remove(IMUSample &sample, EulerOrientationSample &orientation) override {
    float rollRad = orientation.roll * std::numbers::pi_v<float> / 180.0f;
    float pitchRad = orientation.pitch * std::numbers::pi_v<float> / 180.0f;

    sample.a.x = sample.a.x + g * std::sin(pitchRad);
    sample.a.y = sample.a.y - g * std::sin(rollRad) * std::cos(pitchRad);
    sample.a.z = sample.a.z - g * std::cos(rollRad) * std::cos(pitchRad);
  }
};