#pragma once

#include <I2CPort.hpp>

class I2CReal : public I2CPort {
public:
  explicit I2CReal(i2c_port_t port = I2C_NUM_0);
  ~I2CReal() override = default;

  esp_err_t begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num,
                  gpio_pullup_t sda_pullup_en, gpio_pullup_t scl_pullup_en,
                  uint32_t clk_speed) override;
  void scanner() override;
  I2C_t &native() override;

private:
  I2C_t bus;
};
