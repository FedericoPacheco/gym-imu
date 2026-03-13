#pragma once

#include <ports/I2C/I2CPort.hpp>
#include <ports/MPU/MPUCompatibility.hpp>

class MPUPort {
public:
  virtual ~MPUPort() = default;

  virtual void setBus(I2CPort &bus) = 0;
  virtual void setAddr(mpud::mpu_addr_handle_t addr) = 0;
  virtual esp_err_t lastError() = 0;

  virtual esp_err_t initialize() = 0;
  virtual esp_err_t reset() = 0;
  virtual esp_err_t testConnection() = 0;
  virtual esp_err_t selfTest(mpud::selftest_t *result) = 0;
  virtual uint8_t whoAmI() = 0;
  virtual esp_err_t registerDump(uint8_t start, uint8_t end) = 0;

  virtual esp_err_t setSampleRate(uint16_t rate) = 0;
  virtual esp_err_t setAccelFullScale(mpud::accel_fs_t fsr) = 0;
  virtual esp_err_t setGyroFullScale(mpud::gyro_fs_t fsr) = 0;

  virtual esp_err_t setInterruptConfig(mpud::int_config_t config) = 0;
  virtual esp_err_t setInterruptEnabled(mpud::int_en_t mask) = 0;

  virtual esp_err_t setFIFOConfig(mpud::fifo_config_t config) = 0;
  virtual esp_err_t setFIFOEnabled(bool enable) = 0;
  virtual esp_err_t resetFIFO() = 0;
  virtual uint16_t getFIFOCount() = 0;
  virtual esp_err_t readFIFO(size_t length, uint8_t *data) = 0;

  virtual esp_err_t acceleration(mpud::raw_axes_t *accel) = 0;
  virtual esp_err_t rotation(mpud::raw_axes_t *gyro) = 0;
};
