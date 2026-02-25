#pragma once

#include <I2CCompatibility.hpp>

class I2CPort {
public:
  virtual ~I2CPort() = default;

  virtual esp_err_t begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num,
                          gpio_pullup_t sda_pullup_en,
                          gpio_pullup_t scl_pullup_en, uint32_t clk_speed) = 0;
  virtual void scanner() = 0;
  virtual I2C_t &native() = 0;
};
