#pragma once
#include <IMUCalibrator.hpp>

// For details on offline acceleration transformation matrix and gyroscope
// biases computation, refer to: signal/calibration/3-calibration.ipynb
class MPU6050AffineCalibrator : public IMUCalibrator {
private:
  static constexpr float aTransf[4][3] = {
      {0.99878374f, -0.00933288f, 0.02040961f},
      {0.01210606f, 0.99696514f, -0.01862394f},
      {-0.01313606f, -0.00642989f, 0.98043959f},
      {-0.32646267f, -0.16230203f, -0.1406934f}};

  static constexpr float wBiases[3] = {-5.133701801300049f, 3.226569890975952f,
                                       -1.397046685218811f};

public:
  MPU6050AffineCalibrator() = default;
  ~MPU6050AffineCalibrator() = default;

  void calibrate(IMUSample &sample) override {
    // Emulate matrix multiplication to avoid importing a linear algebra library
    // just for this, saving space.
    sample.a.x = sample.a.x * aTransf[0][0] + sample.a.y * aTransf[1][0] +
                 sample.a.z * aTransf[2][0] + 1 * aTransf[3][0];
    sample.a.y = sample.a.x * aTransf[0][1] + sample.a.y * aTransf[1][1] +
                 sample.a.z * aTransf[2][1] + 1 * aTransf[3][1];
    sample.a.z = sample.a.x * aTransf[0][2] + sample.a.y * aTransf[1][2] +
                 sample.a.z * aTransf[2][2] + 1 * aTransf[3][2];

    sample.w.roll = sample.w.roll - wBiases[0];
    sample.w.pitch = sample.w.pitch - wBiases[1];
    sample.w.yaw = sample.w.yaw - wBiases[2];
  }
};
