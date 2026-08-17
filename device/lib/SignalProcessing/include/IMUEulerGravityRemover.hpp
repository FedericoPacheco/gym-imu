#pragma once
#include <IMUEulerOrientationFinder.hpp>
#include <IMUSensorPort.hpp>

// Strategy interface for gravity removal employing previously obtained Euler
// angles
class IMUEulerGravityRemover {
protected:
  IMUEulerGravityRemover() = default;

public:
  virtual ~IMUEulerGravityRemover() = default;

  virtual void remove(IMUSample &sample,
                      EulerOrientationSample &orientation) = 0;
};
