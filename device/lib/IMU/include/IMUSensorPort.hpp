#pragma once
#include <optional>
#include <stdint.h>

struct LinearAxes {
  float x;
  float y;
  float z;
};
struct AngleAxes {
  float roll;
  float pitch;
  float yaw;
};
typedef LinearAxes AccelerationSample;   // m/s2
typedef AngleAxes AngularVelocitySample; // deg/s
typedef uint32_t SequenceNumber;
struct IMUSample {
  AccelerationSample a;
  AngularVelocitySample w;
  SequenceNumber seq;
};

class IMUSensorPort {
protected:
  IMUSensorPort() = default;

public:
  virtual ~IMUSensorPort() = default;

  virtual std::optional<IMUSample> readSync() = 0;
  virtual std::optional<IMUSample> readAsync() = 0;
  virtual void beginAsync() = 0;
  virtual void stopAsync() = 0;
};