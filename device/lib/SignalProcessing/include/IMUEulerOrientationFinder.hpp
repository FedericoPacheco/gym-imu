#pragma once
#include <IMUSensorPort.hpp>
#include <IMUSignalProcessor.hpp>

// Strategy interface for orientation using Euler angles
class IMUEulerOrientationFinder {
protected:
  IMUEulerOrientationFinder() = default;

public:
  virtual ~IMUEulerOrientationFinder() = default;

  virtual EulerOrientationSample find(IMUSample &sample) = 0;
};
