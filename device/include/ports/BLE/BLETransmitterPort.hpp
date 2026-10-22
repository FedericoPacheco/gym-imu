#pragma once

#include <cstdint>

class BLETransmitterPort {
public:
  static constexpr uint32_t STOPPED_MASK = 1u << 2;

  virtual ~BLETransmitterPort() = default;

  virtual bool isConnected() = 0;
  virtual void beginTransmission() = 0;
  virtual void requestStopTransmission() = 0;
};
