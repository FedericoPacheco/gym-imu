#pragma once
#include <optional>
#include <stdint.h>

// g (m/s2)
struct AccelerationSample {
  float x;
  float y;
  float z;
};
// deg/s
struct AngularVelocitySample {
  float roll;
  float pitch;
  float yaw;
};
// us (microseconds)
typedef int64_t Timestamp;
struct IMUSample {
  AccelerationSample a;
  AngularVelocitySample w;
  Timestamp t;
};

class IMUSensor {
protected:
  IMUSensor() = default;

public:
  virtual ~IMUSensor() = default;

  virtual std::optional<IMUSample> readSync() = 0;
  virtual std::optional<IMUSample> readAsync() = 0;
  virtual void beginAsync() = 0;
  virtual void stopAsync() = 0;
};