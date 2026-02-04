#include "Logger.hpp"
#include "MPU6050Sensor.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <memory>

extern "C" void app_main() {
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  vTaskDelay(pdMS_TO_TICKS(5000));
  Logger logger((LogLevel::INFO));
  std::unique_ptr<IMUSensor> imu = MPU6050Sensor::create(&logger);

  logger.debug("DEBUG Sanity Check");
  logger.info("INFO Sanity Check");
  logger.warn("WARN Sanity Check");
  logger.error("ERROR Sanity Check");

  if (imu == nullptr) {
    logger.error("Failed to initialize MPU6050 sensor");
    return;
  }

  imu->beginAsync();
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    imu->readAsync();
  }
  imu->stopAsync();
}