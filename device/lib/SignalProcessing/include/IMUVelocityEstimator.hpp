#pragma once
#include <Auxiliary/Quaternion.hpp>
#include <IMUSensorPort.hpp>

typedef LinearAxes VelocitySample; // m/s

class IMUVelocityEstimator {
protected:
  float dt;
  IMUVelocityEstimator(float dt) : dt(dt) {}

public:
  virtual ~IMUVelocityEstimator() = default;

  virtual VelocitySample estimate(IMUSample &sample,
                                  Quaternion &orientation) = 0;
  virtual void reset() = 0;
};
