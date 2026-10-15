#pragma once
#include <IMUSensorPort.hpp>

typedef AngleAxes EulerOrientationSample; // deg

/*
Both Euler angles and Quaternions could be made to implement a common interface
with a rotate() method (so that, for instance, IMUEulerGravityRemover could be
"simplified", at some extra computational cost, an EulerOrientationSample gets
transformed into a class instead of being a simple struct), but this seems
unnecessary since I'll migrate away from Euler angles.
*/

template <typename Orientation> class IMUOrientationFinder {
protected:
  float dt;
  IMUOrientationFinder(float dt) : dt(dt) {}

public:
  virtual ~IMUOrientationFinder() = default;

  virtual void initialize(IMUSample &sample) = 0;
  virtual Orientation find(IMUSample &sample) = 0;
  virtual void reset() = 0;
};
