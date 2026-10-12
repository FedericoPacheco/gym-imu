#pragma once
#include <IMUCalibrator.hpp>

// For details, refer to: signal/1-calibration
class MPU6050AccelAffineCalibrator : public IMUCalibrator {
private:
  static constexpr float A_AFFINE_TRANSF[4][3] = {
      {1.00081528f, -0.01812657f, 0.01910186f},
      {0.01423018f, 1.00642545f, -0.00695651f},
      {-0.01369805f, 0.00239254f, 0.97203501f},
      {-0.30637323f, -0.19065541f, -0.10564389f}};

public:
  MPU6050AccelAffineCalibrator() = default;
  ~MPU6050AccelAffineCalibrator() = default;

  void initialize(IMUSample &sample) override {}

  void calibrate(IMUSample &sample) override {
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

  void reset() override {}
};
