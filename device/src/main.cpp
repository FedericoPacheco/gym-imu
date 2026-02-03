#include "Logger.hpp"
#include "MPU6050Sensor.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <memory>

extern "C" void app_main() {
  // gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  vTaskDelay(pdMS_TO_TICKS(5000));
  Logger logger((LogLevel::DEBUG));
  std::unique_ptr<IMUSensor> imu = MPU6050Sensor::create(&logger);

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    imu->readSync();
  }
}