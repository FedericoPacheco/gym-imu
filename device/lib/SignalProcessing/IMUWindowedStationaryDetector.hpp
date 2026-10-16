#pragma once
class IMUWindowedStationaryDetector {

public:
  bool isStationaryPreOrientation(const IMUSample &sample) {
    return this->isStationary(sample, PRE_ORIENTATION_F_STATIONARY_MEDIAN,
                              PRE_ORIENTATION_F_STATIONARY_TOL,
                              PRE_ORIENTATION_W_STATIONARY_MEDIAN,
                              PRE_ORIENTATION_W_STATIONARY_TOL);
  }
  bool isGyroOnStationaryBandPreOrientation(const IMUSample &sample) {
    return this->isGyroOnStationaryBand(sample,
                                        PRE_ORIENTATION_W_STATIONARY_MEDIAN,
                                        PRE_ORIENTATION_W_STATIONARY_TOL);
  }
  bool isAccelOnStationaryBandPreOrientation(const IMUSample &sample) {
    return this->isAccelOnStationaryBand(sample,
                                         PRE_ORIENTATION_F_STATIONARY_MEDIAN,
                                         PRE_ORIENTATION_F_STATIONARY_TOL);
  }

  bool isStationaryPreVelocity(const IMUSample &sample) {
    return this->isStationary(
        sample, PRE_VELOCITY_A_STATIONARY_MEDIAN, PRE_VELOCITY_A_STATIONARY_TOL,
        PRE_VELOCITY_W_STATIONARY_MEDIAN, PRE_VELOCITY_W_STATIONARY_TOL);
  }
  bool isGyroOnStationaryBandPreVelocity(const IMUSample &sample) {
    return this->isGyroOnStationaryBand(sample,
                                        PRE_VELOCITY_W_STATIONARY_MEDIAN,
                                        PRE_VELOCITY_W_STATIONARY_TOL);
  }
  bool isAccelOnStationaryBandPreVelocity(const IMUSample &sample) {
    return this->isAccelOnStationaryBand(sample,
                                         PRE_VELOCITY_A_STATIONARY_MEDIAN,
                                         PRE_VELOCITY_A_STATIONARY_TOL);
  }

  void reset() { this->stationaryCount = 0; }

private:
  static constexpr float PRE_ORIENTATION_F_STATIONARY_MEDIAN = 9.814409f;
  static constexpr float PRE_ORIENTATION_F_STATIONARY_TOL = 0.260682f;
  static constexpr float PRE_ORIENTATION_W_STATIONARY_MEDIAN = 6.232008f;
  static constexpr float PRE_ORIENTATION_W_STATIONARY_TOL = 1.433158f;

  static constexpr float PRE_VELOCITY_A_STATIONARY_MEDIAN = 0.047670f;
  static constexpr float PRE_VELOCITY_A_STATIONARY_TOL = 0.168150f;
  static constexpr float PRE_VELOCITY_W_STATIONARY_MEDIAN = 0.091010f;
  static constexpr float PRE_VELOCITY_W_STATIONARY_TOL = 0.136889f;

  static constexpr uint32_t WINDOW_SIZE = 7; // Samples

  uint32_t stationaryCount = 0;

  bool isStationary(const IMUSample &sample, const float aCenter,
                    const float aRadius, const float wCenter,
                    const float wRadius) {

    const bool isAccelStationary =
        this->isAccelOnStationaryBand(sample, aCenter, aRadius);
    const bool isGyroStationary =
        this->isGyroOnStationaryBand(sample, wCenter, wRadius);

    if (isAccelStationary && isGyroStationary) {
      this->stationaryCount++;
      if (this->stationaryCount >= WINDOW_SIZE)
        return true;
    } else
      this->stationaryCount = 0;
    return false;
  }

  bool isGyroOnStationaryBand(const IMUSample &sample, const float wCenter,
                              const float wRadius) {
    const float wNorm = std::sqrt(sample.w.roll * sample.w.roll +
                                  sample.w.pitch * sample.w.pitch +
                                  sample.w.yaw * sample.w.yaw);
    return std::abs(wNorm - wCenter) <= wRadius;
  }

  bool isAccelOnStationaryBand(const IMUSample &sample, const float aCenter,
                               const float aRadius) {
    const float aNorm =
        std::sqrt(sample.a.x * sample.a.x + sample.a.y * sample.a.y +
                  sample.a.z * sample.a.z);
    return std::abs(aNorm - aCenter) <= aRadius;
  }
};
