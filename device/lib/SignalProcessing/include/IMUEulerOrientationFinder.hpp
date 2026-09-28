#pragma once
#include <IMUSensorPort.hpp>

typedef AngleAxes EulerOrientationSample; // deg

// Strategy interface for orientation using Euler angles
class IMUEulerOrientationFinder {
protected:
  IMUEulerOrientationFinder() = default;

public:
  virtual ~IMUEulerOrientationFinder() = default;

  virtual EulerOrientationSample find(IMUSample &sample) = 0;
};
