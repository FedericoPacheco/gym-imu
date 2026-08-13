#pragma once
#include <IMUCalibrator.hpp>
#include <StationaryChecker.hpp>

// For details, refer to: signal/1-calibration
class MPU6050AffineCalibrator : public IMUCalibrator {
private:
  static constexpr float A_AFFINE_TRANSF[4][3] = {
      {1.00081528f, -0.01812657f, 0.01910186f},
      {0.01423018f, 1.00642545f, -0.00695651f},
      {-0.01369805f, 0.00239254f, 0.97203501f},
      {-0.30637323f, -0.19065541f, -0.10564389f}};

  static constexpr float W_STATIONARY_BIAS[3] = {-5.180648f, 3.161055f,
                                                 -1.4527919f};
  static constexpr float W_ALPHA = 0.967216f;

  float wBias[3] = {W_STATIONARY_BIAS[0], W_STATIONARY_BIAS[1],
                    W_STATIONARY_BIAS[2]};

public:
  MPU6050AffineCalibrator() = default;
  ~MPU6050AffineCalibrator() = default;

  void calibrate(IMUSample &sample) override {
    if (StationaryChecker::isStationaryPreCalibration(sample) &&
        sample.seq > 1) {
      this->wBias[0] =
          MPU6050AffineCalibrator::W_ALPHA * this->wBias[0] +
          (1.0f - MPU6050AffineCalibrator::W_ALPHA) * sample.w.roll;
      this->wBias[1] =
          MPU6050AffineCalibrator::W_ALPHA * this->wBias[1] +
          (1.0f - MPU6050AffineCalibrator::W_ALPHA) * sample.w.pitch;
      this->wBias[2] = MPU6050AffineCalibrator::W_ALPHA * this->wBias[2] +
                       (1.0f - MPU6050AffineCalibrator::W_ALPHA) * sample.w.yaw;
    }
    sample.w.roll -= this->wBias[0];
    sample.w.pitch -= this->wBias[1];
    sample.w.yaw -= this->wBias[2];

    // Emulate matrix multiplication to avoid importing a linear algebra library
    // just for this, saving space.
    const float axRaw = sample.a.x;
    const float ayRaw = sample.a.y;
    const float azRaw = sample.a.z;
    sample.a.x = axRaw * A_AFFINE_TRANSF[0][0] + ayRaw * A_AFFINE_TRANSF[1][0] +
                 azRaw * A_AFFINE_TRANSF[2][0] + A_AFFINE_TRANSF[3][0];
    sample.a.y = axRaw * A_AFFINE_TRANSF[0][1] + ayRaw * A_AFFINE_TRANSF[1][1] +
                 azRaw * A_AFFINE_TRANSF[2][1] + A_AFFINE_TRANSF[3][1];
    sample.a.z = axRaw * A_AFFINE_TRANSF[0][2] + ayRaw * A_AFFINE_TRANSF[1][2] +
                 azRaw * A_AFFINE_TRANSF[2][2] + A_AFFINE_TRANSF[3][2];
  }
};
