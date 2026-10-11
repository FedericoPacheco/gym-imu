#pragma once
#include <IMUSensorPort.hpp>

typedef AngleAxes EulerOrientationSample; // deg

template <typename Orientation>
class IMUOrientationFinder {
protected:
  IMUOrientationFinder() = default;

public:
  virtual ~IMUOrientationFinder() = default;
  virtual Orientation find(IMUSample &sample) = 0;
};

