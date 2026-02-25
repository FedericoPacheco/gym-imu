#include "I2CReal.hpp"

I2CReal::I2CReal(i2c_port_t port) : bus(port) {}

esp_err_t I2CReal::begin(gpio_num_t sda_io_num, gpio_num_t scl_io_num,
                         gpio_pullup_t sda_pullup_en,
                         gpio_pullup_t scl_pullup_en, uint32_t clk_speed) {
  return this->bus.begin(sda_io_num, scl_io_num, sda_pullup_en, scl_pullup_en,
                         clk_speed);
}

void I2CReal::scanner() { this->bus.scanner(); }

I2C_t &I2CReal::native() { return this->bus; }
