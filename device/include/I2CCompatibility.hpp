#pragma once

#include <ESPCompatibility.hpp>

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)

typedef int i2c_port_t;
static constexpr i2c_port_t I2C_NUM_0 = 0;
struct I2C_t {
  int placeholder;
};

#else

#include "I2Cbus.hpp"

#endif