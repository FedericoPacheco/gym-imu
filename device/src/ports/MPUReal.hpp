#pragma once

#include "I2CReal.hpp"
#include <MPUPort.hpp>

class MPUReal : public MPUPort {
public:
  MPUReal() = default;
  ~MPUReal() override = default;

  void setBus(I2CPort &bus) override;
  void setAddr(mpud::mpu_addr_handle_t addr) override;
  esp_err_t lastError() override;

  esp_err_t initialize() override;
  esp_err_t reset() override;
  esp_err_t testConnection() override;
  esp_err_t selfTest(mpud::selftest_t *result) override;
  uint8_t whoAmI() override;
  esp_err_t registerDump(uint8_t start, uint8_t end) override;

  esp_err_t setSampleRate(uint16_t rate) override;
  esp_err_t setAccelFullScale(mpud::accel_fs_t fsr) override;
  esp_err_t setGyroFullScale(mpud::gyro_fs_t fsr) override;

  esp_err_t setInterruptConfig(mpud::int_config_t config) override;
  esp_err_t setInterruptEnabled(mpud::int_en_t mask) override;

  esp_err_t setFIFOConfig(mpud::fifo_config_t config) override;
  esp_err_t setFIFOEnabled(bool enable) override;
  esp_err_t resetFIFO() override;
  uint16_t getFIFOCount() override;
  esp_err_t readFIFO(size_t length, uint8_t *data) override;

  esp_err_t acceleration(mpud::raw_axes_t *accel) override;
  esp_err_t rotation(mpud::raw_axes_t *gyro) override;

private:
  MPU_t sensor;
};
