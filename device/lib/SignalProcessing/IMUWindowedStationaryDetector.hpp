#pragma once
class IMUWindowedStationaryDetector {

public:
  bool isStationaryPreOrientation(const IMUSample &sample) {
    return isStationary(sample, PRE_ORIENTATION_A_STATIONARY_MEDIAN,
                        PRE_ORIENTATION_A_STATIONARY_TOL,
                        PRE_ORIENTATION_W_STATIONARY_MEDIAN,
                        PRE_ORIENTATION_W_STATIONARY_TOL);
  }

  void reset() { this->stationaryCount = 0; }

private:
  static constexpr float PRE_ORIENTATION_A_STATIONARY_MEDIAN = 9.814409f;
  static constexpr float PRE_ORIENTATION_A_STATIONARY_TOL = 0.260682f;
  static constexpr float PRE_ORIENTATION_W_STATIONARY_MEDIAN = 6.232008f;
  static constexpr float PRE_ORIENTATION_W_STATIONARY_TOL = 1.433158f;

  static constexpr uint32_t WINDOW_SIZE = 7; // Samples

  uint32_t stationaryCount = 0;

  bool isStationary(const IMUSample &sample, const float aCenter,
                    const float aRadius, const float wCenter,
                    const float wRadius) {
    const float aNorm =
        std::sqrt(sample.a.x * sample.a.x + sample.a.y * sample.a.y +
                  sample.a.z * sample.a.z);
    const float wNorm = std::sqrt(sample.w.roll * sample.w.roll +
                                  sample.w.pitch * sample.w.pitch +
                                  sample.w.yaw * sample.w.yaw);

    const bool isAccelStationary = std::abs(aNorm - aCenter) <= aRadius;
    const bool isGyroStationary = std::abs(wNorm - wCenter) <= wRadius;

    if (isAccelStationary && isGyroStationary) {
      this->stationaryCount++;
      if (this->stationaryCount >= WINDOW_SIZE)
        return true;
    } else
      this->stationaryCount = 0;
    return false;
  }
};
