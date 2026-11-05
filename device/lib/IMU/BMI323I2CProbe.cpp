#include "BMI323I2CProbe.hpp"

#include "freertos/FreeRTOS.h"
#include <new>

BMI323I2CProbe::BMI323I2CProbe(LoggerPort *logger, gpio_num_t sdaPin,
                               gpio_num_t sclPin)
    : logger(logger), sdaPin(sdaPin), sclPin(sclPin) {}

BMI323I2CProbe::~BMI323I2CProbe() {
  if (this->driverInstalled) {
    const esp_err_t err = i2c_driver_delete(BMI323I2CProbe::i2cPort);
    if (err != ESP_OK) {
      this->logger->error("Failed to uninstall I2C driver: %s",
                          esp_err_to_name(err));
    }
  }
}

std::unique_ptr<BMI323I2CProbe>
BMI323I2CProbe::create(LoggerPort *logger, gpio_num_t sdaPin,
                       gpio_num_t sclPin) {
  if (!logger)
    return nullptr;

  std::unique_ptr<BMI323I2CProbe> probe(
      new (std::nothrow) BMI323I2CProbe(logger, sdaPin, sclPin));
  if (!probe) {
    logger->error("Failed to allocate BMI323 I2C probe.");
    return nullptr;
  }

  if (!probe->initializeI2CBus() || !probe->testConnection())
    return nullptr;

  return probe;
}

bool BMI323I2CProbe::initializeI2CBus() {
  i2c_config_t config{};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = this->sdaPin;
  config.scl_io_num = this->sclPin;
  config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  config.master.clk_speed = 100000;
  config.clk_flags = 0;

  esp_err_t err = i2c_param_config(BMI323I2CProbe::i2cPort, &config);
  if (err != ESP_OK) {
    this->logger->error("Failed to configure I2C bus: %s",
                        esp_err_to_name(err));
    return false;
  }

  err = i2c_driver_install(BMI323I2CProbe::i2cPort, config.mode, 0, 0, 0);
  if (err != ESP_OK) {
    this->logger->error("Failed to install I2C driver: %s",
                        esp_err_to_name(err));
    return false;
  }

  this->driverInstalled = true;
  this->logger->info("I2C bus initialized at 100 kHz.");
  return true;
}

bool BMI323I2CProbe::testConnection() {
  constexpr uint8_t addresses[] = {0x68, 0x69};
  bool foundSensor = false;

  for (const uint8_t address : addresses) {
    uint8_t chipId = 0;
    const esp_err_t err = this->readChipId(address, &chipId);
    if (err != ESP_OK) {
      this->logger->debug("BMI323 CHIP_ID read at 0x%02X failed: %s", address,
                          esp_err_to_name(err));
      continue;
    }

    this->logger->info("BMI323 CHIP_ID at address 0x%02X: 0x%02X (expected "
                       "0x%02X)",
                       address, chipId, BMI323I2CProbe::CHIP_ID_VALUE);
    if (chipId == BMI323I2CProbe::CHIP_ID_VALUE)
      foundSensor = true;
  }

  if (!foundSensor)
    this->logger->error("BMI323 not detected at I2C address 0x68 or 0x69.");
  return foundSensor;
}

esp_err_t BMI323I2CProbe::readChipId(uint8_t address, uint8_t *chipId) {
  if (!chipId)
    return ESP_ERR_INVALID_ARG;

  const uint8_t registerAddress = BMI323I2CProbe::CHIP_ID_REGISTER;
  uint8_t response[BMI323I2CProbe::CHIP_ID_READ_LENGTH]{};
  const esp_err_t err = i2c_master_write_read_device(
      BMI323I2CProbe::i2cPort, address, &registerAddress, 1, response,
      sizeof(response), pdMS_TO_TICKS(100));
  if (err != ESP_OK)
    return err;

  // BMI323 I2C reads return two dummy bytes before the 16-bit register word.
  *chipId = response[2];
  return ESP_OK;
}
