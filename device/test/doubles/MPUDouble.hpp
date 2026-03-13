#pragma once

#include <gmock/gmock.h>
#include <ports/MPU/MPUPort.hpp>

class MPUDouble : public MPUPort {
public:
  MOCK_METHOD(void, setBus, (I2CPort & bus), (override));
  MOCK_METHOD(void, setAddr, (mpud::mpu_addr_handle_t addr), (override));
  MOCK_METHOD(esp_err_t, lastError, (), (override));

  MOCK_METHOD(esp_err_t, initialize, (), (override));
  MOCK_METHOD(esp_err_t, reset, (), (override));
  MOCK_METHOD(esp_err_t, testConnection, (), (override));
  MOCK_METHOD(esp_err_t, selfTest, (mpud::selftest_t * result), (override));
  MOCK_METHOD(uint8_t, whoAmI, (), (override));
  MOCK_METHOD(esp_err_t, registerDump, (uint8_t start, uint8_t end),
              (override));

  MOCK_METHOD(esp_err_t, setSampleRate, (uint16_t rate), (override));
  MOCK_METHOD(esp_err_t, setAccelFullScale, (mpud::accel_fs_t fsr), (override));
  MOCK_METHOD(esp_err_t, setGyroFullScale, (mpud::gyro_fs_t fsr), (override));

  MOCK_METHOD(esp_err_t, setInterruptConfig, (mpud::int_config_t config),
              (override));
  MOCK_METHOD(esp_err_t, setInterruptEnabled, (mpud::int_en_t mask),
              (override));

  MOCK_METHOD(esp_err_t, setFIFOConfig, (mpud::fifo_config_t config),
              (override));
  MOCK_METHOD(esp_err_t, setFIFOEnabled, (bool enable), (override));
  MOCK_METHOD(esp_err_t, resetFIFO, (), (override));
  MOCK_METHOD(uint16_t, getFIFOCount, (), (override));
  MOCK_METHOD(esp_err_t, readFIFO, (size_t length, uint8_t *data), (override));

  MOCK_METHOD(esp_err_t, acceleration, (mpud::raw_axes_t * accel), (override));
  MOCK_METHOD(esp_err_t, rotation, (mpud::raw_axes_t * gyro), (override));
};
