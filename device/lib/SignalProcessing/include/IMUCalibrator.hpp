#pragma once
#include <IMUSensorPort.hpp>

// Strategy interface for calibration
class IMUCalibrator {
protected:
  IMUCalibrator() = default;

public:
  virtual ~IMUCalibrator() = default;

  // Edit sample in place to reduce memory usage
  virtual void calibrate(IMUSample &sample) = 0;
};
