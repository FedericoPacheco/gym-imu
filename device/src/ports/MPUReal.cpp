#include "MPUReal.hpp"

void MPUReal::setBus(I2CPort &bus) {
  auto &realBus = static_cast<I2CReal &>(bus);
  this->sensor.setBus(realBus.native());
}

void MPUReal::setAddr(mpud::mpu_addr_handle_t addr) {
  this->sensor.setAddr(addr);
}

esp_err_t MPUReal::lastError() { return this->sensor.lastError(); }

esp_err_t MPUReal::initialize() { return this->sensor.initialize(); }

esp_err_t MPUReal::reset() { return this->sensor.reset(); }

esp_err_t MPUReal::testConnection() { return this->sensor.testConnection(); }

esp_err_t MPUReal::selfTest(mpud::selftest_t *result) {
  return this->sensor.selfTest(result);
}

uint8_t MPUReal::whoAmI() { return this->sensor.whoAmI(); }

esp_err_t MPUReal::registerDump(uint8_t start, uint8_t end) {
  return this->sensor.registerDump(start, end);
}

esp_err_t MPUReal::setSampleRate(uint16_t rate) {
  return this->sensor.setSampleRate(rate);
}

esp_err_t MPUReal::setAccelFullScale(mpud::accel_fs_t fsr) {
  return this->sensor.setAccelFullScale(fsr);
}

esp_err_t MPUReal::setGyroFullScale(mpud::gyro_fs_t fsr) {
  return this->sensor.setGyroFullScale(fsr);
}

esp_err_t MPUReal::setInterruptConfig(mpud::int_config_t config) {
  return this->sensor.setInterruptConfig(config);
}

esp_err_t MPUReal::setInterruptEnabled(mpud::int_en_t mask) {
  return this->sensor.setInterruptEnabled(mask);
}

esp_err_t MPUReal::setFIFOConfig(mpud::fifo_config_t config) {
  return this->sensor.setFIFOConfig(config);
}

esp_err_t MPUReal::setFIFOEnabled(bool enable) {
  return this->sensor.setFIFOEnabled(enable);
}

esp_err_t MPUReal::resetFIFO() { return this->sensor.resetFIFO(); }

uint16_t MPUReal::getFIFOCount() { return this->sensor.getFIFOCount(); }

esp_err_t MPUReal::readFIFO(size_t length, uint8_t *data) {
  return this->sensor.readFIFO(length, data);
}

esp_err_t MPUReal::acceleration(mpud::raw_axes_t *accel) {
  return this->sensor.acceleration(accel);
}

esp_err_t MPUReal::rotation(mpud::raw_axes_t *gyro) {
  return this->sensor.rotation(gyro);
}
