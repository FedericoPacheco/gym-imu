#pragma once
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

typedef LinearAxes AccelerationSample;    // m/s2
typedef LinearAxes VelocitySample;        // m/s
typedef AngleAxes AngularVelocitySample;  // deg/s
typedef AngleAxes EulerOrientationSample; // deg
typedef uint32_t SequenceNumber;

struct IMUSample {
  AccelerationSample a;
  AngularVelocitySample w;
  SequenceNumber seq;
};
