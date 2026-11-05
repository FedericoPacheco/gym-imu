#pragma once

#include "driver/i2c.h"
#include <LoggerPort.hpp>
#include <memory>

// TODO: delete this class once the correct ICM42688P sensor is received
// (also remove from CMakeLists file)

class BMI323I2CProbe {
public:
  static std::unique_ptr<BMI323I2CProbe>
  create(LoggerPort *logger, gpio_num_t sdaPin, gpio_num_t sclPin);
  ~BMI323I2CProbe();

private:
  static constexpr i2c_port_t i2cPort = I2C_NUM_0;
  static constexpr uint8_t CHIP_ID_REGISTER = 0x00;
  static constexpr uint8_t CHIP_ID_VALUE = 0x43;
  static constexpr size_t CHIP_ID_READ_LENGTH = 4;

  BMI323I2CProbe(LoggerPort *logger, gpio_num_t sdaPin, gpio_num_t sclPin);

  LoggerPort *logger;
  gpio_num_t sdaPin;
  gpio_num_t sclPin;
  bool driverInstalled = false;

  bool initializeI2CBus();
  bool testConnection();
  esp_err_t readChipId(uint8_t address, uint8_t *chipId);
};
