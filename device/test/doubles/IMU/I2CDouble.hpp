#pragma once

#include <I2CPort.hpp>
#include <gmock/gmock.h>

class I2CDouble : public I2CPort {
public:
  MOCK_METHOD(esp_err_t, begin,
              (gpio_num_t sda_io_num, gpio_num_t scl_io_num,
               gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en,
               uint32_t clk_speed),
              (override));
  MOCK_METHOD(void, scanner, (), (override));
  MOCK_METHOD(I2C_t &, native, (), (override));
};
