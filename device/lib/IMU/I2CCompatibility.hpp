#pragma once

#include <ESPCompatibility.hpp>
#include <cstdint>

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)

typedef int gpio_num_t;
typedef int gpio_pullup_t;
typedef int i2c_port_t;

static constexpr gpio_num_t GPIO_NUM_5 = 5;
static constexpr gpio_num_t GPIO_NUM_6 = 6;
static constexpr gpio_num_t GPIO_NUM_7 = 7;

static constexpr gpio_pullup_t GPIO_PULLUP_DISABLE = 0;
static constexpr gpio_pullup_t GPIO_PULLDOWN_ENABLE = 1;

static constexpr i2c_port_t I2C_NUM_0 = 0;

struct I2C_t {
  int placeholder;
};

#else

#include "I2Cbus.hpp"

#endif