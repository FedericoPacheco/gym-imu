#pragma once

class StationaryChecker {
  static constexpr float PRE_CALIBRATION_A_STATIONARY_MEAN = 9.907132f;
  static constexpr float PRE_CALIBRATION_A_STATIONARY_TOL = 0.691502f;
  static constexpr float PRE_CALIBRATION_W_STATIONARY_MEAN = 6.291752f;
  static constexpr float PRE_CALIBRATION_W_STATIONARY_TOL = 4.630163f;

public:
  static bool isStationaryPreCalibration(const IMUSample &sample) {
    return isStationary(sample, PRE_CALIBRATION_A_STATIONARY_MEAN,
                        PRE_CALIBRATION_A_STATIONARY_TOL,
                        PRE_CALIBRATION_W_STATIONARY_MEAN,
                        PRE_CALIBRATION_W_STATIONARY_TOL);
  }

private:
  static bool isStationary(const IMUSample &sample, const float aCenter,
                           const float aRadius, const float wCenter,
                           const float wRadius) {
    const float aNorm =
        std::sqrt(sample.a.x * sample.a.x + sample.a.y * sample.a.y +
                  sample.a.z * sample.a.z);
    const float wNorm = std::sqrt(sample.w.roll * sample.w.roll +
                                  sample.w.pitch * sample.w.pitch +
                                  sample.w.yaw * sample.w.yaw);
    return std::abs(aNorm - aCenter) <= aRadius &&
           std::abs(wNorm - wCenter) <= wRadius;
  }
};
