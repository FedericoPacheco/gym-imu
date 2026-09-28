#pragma once
#include <IMUSensorPort.hpp>

// Strategy interface for low-passing/noise reduction
class IMUNoiseReducer {
protected:
  IMUNoiseReducer() = default;

public:
  virtual ~IMUNoiseReducer() = default;

  // Edit sample in place to reduce memory usage
  virtual void filter(IMUSample &sample) = 0;
};