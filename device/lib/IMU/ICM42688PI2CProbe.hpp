#pragma once

#include "driver/i2c.h"
#include <LoggerPort.hpp>
#include <memory>

// TODO: delete this class once the correct ICM42688P sensor is received
// (also remove from CMakeLists file)

class ICM42688PI2CProbe {
public:
  static std::unique_ptr<ICM42688PI2CProbe>
  create(LoggerPort *logger, gpio_num_t sdaPin, gpio_num_t sclPin);
  ~ICM42688PI2CProbe();

private:
  static constexpr i2c_port_t i2cPort = I2C_NUM_0;
  static constexpr uint8_t DEVICE_CONFIG_REGISTER = 0x11;
  static constexpr uint8_t SOFT_RESET_VALUE = 0x01;
  static constexpr uint8_t WHO_AM_I_REGISTER = 0x75;
  static constexpr uint8_t WHO_AM_I_VALUE = 0x47;
  static constexpr uint8_t DRIVE_CONFIG_REGISTER = 0x13;
  static constexpr uint8_t DRIVE_CONFIG_RESET_VALUE = 0x05;
  static constexpr uint8_t INTF_CONFIG0_REGISTER = 0x4C;
  static constexpr uint8_t INTF_CONFIG0_RESET_VALUE = 0x30;

  ICM42688PI2CProbe(LoggerPort *logger, gpio_num_t sdaPin, gpio_num_t sclPin);

  LoggerPort *logger;
  gpio_num_t sdaPin;
  gpio_num_t sclPin;
  bool driverInstalled = false;

  bool initializeI2CBus();
  bool testConnection();
  esp_err_t readRegister(uint8_t address, uint8_t registerAddress,
                         uint8_t *value);
  esp_err_t writeRegister(uint8_t address, uint8_t registerAddress,
                          uint8_t value);
};
