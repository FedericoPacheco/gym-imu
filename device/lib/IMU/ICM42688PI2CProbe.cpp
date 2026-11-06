#include "ICM42688PI2CProbe.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <new>

ICM42688PI2CProbe::ICM42688PI2CProbe(LoggerPort *logger, gpio_num_t sdaPin,
                                     gpio_num_t sclPin)
    : logger(logger), sdaPin(sdaPin), sclPin(sclPin) {}

ICM42688PI2CProbe::~ICM42688PI2CProbe() {
  if (this->driverInstalled) {
    const esp_err_t err = i2c_driver_delete(ICM42688PI2CProbe::i2cPort);
    if (err != ESP_OK) {
      this->logger->error("Failed to uninstall I2C driver: %s",
                          esp_err_to_name(err));
    }
  }
}

std::unique_ptr<ICM42688PI2CProbe>
ICM42688PI2CProbe::create(LoggerPort *logger, gpio_num_t sdaPin,
                          gpio_num_t sclPin) {
  if (!logger)
    return nullptr;

  std::unique_ptr<ICM42688PI2CProbe> probe(
      new (std::nothrow) ICM42688PI2CProbe(logger, sdaPin, sclPin));
  if (!probe) {
    logger->error("Failed to allocate ICM42688P I2C probe.");
    return nullptr;
  }

  if (!probe->initializeI2CBus() || !probe->testConnection())
    return nullptr;

  return probe;
}

bool ICM42688PI2CProbe::initializeI2CBus() {
  i2c_config_t config{};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = this->sdaPin;
  config.scl_io_num = this->sclPin;
  config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  config.master.clk_speed = 100000;
  config.clk_flags = 0;

  esp_err_t err = i2c_param_config(ICM42688PI2CProbe::i2cPort, &config);
  if (err != ESP_OK) {
    this->logger->error("Failed to configure I2C bus: %s",
                        esp_err_to_name(err));
    return false;
  }

  err = i2c_driver_install(ICM42688PI2CProbe::i2cPort, config.mode, 0, 0, 0);
  if (err != ESP_OK) {
    this->logger->error("Failed to install I2C driver: %s",
                        esp_err_to_name(err));
    return false;
  }

  this->driverInstalled = true;
  this->logger->info("I2C bus initialized at 100 kHz.");
  return true;
}

bool ICM42688PI2CProbe::testConnection() {
  constexpr uint8_t addresses[] = {0x68, 0x69};
  bool foundSensor = false;

  for (const uint8_t address : addresses) {
    uint8_t whoAmI = 0;
    const esp_err_t err = this->readRegister(
        address, ICM42688PI2CProbe::WHO_AM_I_REGISTER, &whoAmI);
    if (err != ESP_OK) {
      this->logger->debug("I2C read of WHO_AM_I at address 0x%02X failed: %s",
                          address, esp_err_to_name(err));
      continue;
    }

    // this->logger->info(
    //     "I2C address 0x%02X responded; WHO_AM_I before reset: 0x%02X",
    //     address, whoAmI);

    // const esp_err_t resetErr =
    //     this->writeRegister(address,
    //     ICM42688PI2CProbe::DEVICE_CONFIG_REGISTER,
    //                         ICM42688PI2CProbe::SOFT_RESET_VALUE);
    // if (resetErr != ESP_OK) {
    //   this->logger->error("Soft reset write at I2C address 0x%02X failed:
    //   %s",
    //                       address, esp_err_to_name(resetErr));
    //   continue;
    // }

    // this->logger->info("Soft reset sent to I2C address 0x%02X; waiting 10
    // ms.",
    //                    address);
    // vTaskDelay(pdMS_TO_TICKS(10));

    // whoAmI = 0;
    // const esp_err_t whoAmIErr =
    //     this->readRegister(address, ICM42688PI2CProbe::WHO_AM_I_REGISTER,
    //                        &whoAmI);
    // if (whoAmIErr != ESP_OK) {
    //   this->logger->error("I2C read of WHO_AM_I after reset at 0x%02X failed:
    //   "
    //                       "%s",
    //                       address, esp_err_to_name(whoAmIErr));
    //   continue;
    // }

    // if (whoAmI == ICM42688PI2CProbe::WHO_AM_I_VALUE) {
    //   foundSensor = true;
    //   this->logger->info("WHO_AM_I after reset at I2C address 0x%02X:
    //   0x%02X",
    //                      address, whoAmI);
    // } else {
    //   this->logger->warn(
    //       "I2C address 0x%02X responded, but WHO_AM_I after reset was 0x%02X
    //       "
    //       "(expected 0x%02X).",
    //       address, whoAmI, ICM42688PI2CProbe::WHO_AM_I_VALUE);
    // }

    // uint8_t driveConfig = 0;
    // const esp_err_t driveErr =
    //     this->readRegister(address, ICM42688PI2CProbe::DRIVE_CONFIG_REGISTER,
    //                        &driveConfig);
    // if (driveErr == ESP_OK) {
    //   this->logger->info(
    //       "DRIVE_CONFIG (0x%02X) at 0x%02X: 0x%02X (reset value: 0x%02X)",
    //       ICM42688PI2CProbe::DRIVE_CONFIG_REGISTER, address, driveConfig,
    //       ICM42688PI2CProbe::DRIVE_CONFIG_RESET_VALUE);
    // } else {
    //   this->logger->warn("I2C read of DRIVE_CONFIG at 0x%02X failed: %s",
    //                      address, esp_err_to_name(driveErr));
    // }

    // uint8_t intfConfig0 = 0;
    // const esp_err_t intfErr =
    //     this->readRegister(address, ICM42688PI2CProbe::INTF_CONFIG0_REGISTER,
    //                        &intfConfig0);
    // if (intfErr == ESP_OK) {
    //   this->logger->info(
    //       "INTF_CONFIG0 (0x%02X) at 0x%02X: 0x%02X (reset value: 0x%02X)",
    //       ICM42688PI2CProbe::INTF_CONFIG0_REGISTER, address, intfConfig0,
    //       ICM42688PI2CProbe::INTF_CONFIG0_RESET_VALUE);
    // } else {
    //   this->logger->warn("I2C read of INTF_CONFIG0 at 0x%02X failed: %s",
    //                      address, esp_err_to_name(intfErr));
    // }
  }

  if (!foundSensor) {
    this->logger->error(
        "ICM-42688-P was not found at I2C address 0x68 or 0x69.");
  }
  return foundSensor;
}

esp_err_t ICM42688PI2CProbe::readRegister(uint8_t address,
                                          uint8_t registerAddress,
                                          uint8_t *value) {
  if (!value)
    return ESP_ERR_INVALID_ARG;

  return i2c_master_write_read_device(ICM42688PI2CProbe::i2cPort, address,
                                      &registerAddress, 1, value, 1,
                                      pdMS_TO_TICKS(100));
}

esp_err_t ICM42688PI2CProbe::writeRegister(uint8_t address,
                                           uint8_t registerAddress,
                                           uint8_t value) {
  const uint8_t command[] = {registerAddress, value};
  return i2c_master_write_to_device(ICM42688PI2CProbe::i2cPort, address,
                                    command, sizeof(command),
                                    pdMS_TO_TICKS(100));
}
