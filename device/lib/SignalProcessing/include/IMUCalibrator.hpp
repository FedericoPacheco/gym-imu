#pragma once
#include <IMUSensorPort.hpp>

// Strategy interface for calibration
class IMUCalibrator {
protected:
  IMUCalibrator() = default;

public:
  virtual ~IMUCalibrator() = default;

  virtual void initialize(IMUSample &sample) = 0;
  // Edit sample in place to reduce memory usage
  virtual void calibrate(IMUSample &sample) = 0;
  virtual void reset() = 0;
};
