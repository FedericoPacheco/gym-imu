#pragma once
#include <IMUOrientationFinder.hpp>
#include <IMUSensorPort.hpp>

template <typename Orientation> class IMUGravityRemover {
protected:
  IMUGravityRemover() = default;

public:
  virtual ~IMUGravityRemover() = default;

  // Edit sample in place to reduce memory usage
  virtual void remove(IMUSample &sample, Orientation &orientation) = 0;
};
