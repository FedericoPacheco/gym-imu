#pragma once
#include <IMUOrientationFinder.hpp>
#include <IMUSensorPort.hpp>

template <typename Orientation>
class IMUGravityRemover {
protected:
  IMUGravityRemover() = default;

public:
  virtual ~IMUGravityRemover() = default;

  virtual void remove(IMUSample &sample,
                      Orientation &orientation) = 0;
};
