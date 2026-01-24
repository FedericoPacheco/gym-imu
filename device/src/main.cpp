#include "Logger.hpp"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <iostream>

extern "C" void app_main() {
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  Logger logger((LogLevel::DEBUG));
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000 + std::rand() % 2000));
    logger.debug("Debug log message");

    vTaskDelay(pdMS_TO_TICKS(1000 + std::rand() % 2000));
    logger.info("Info log message");

    vTaskDelay(pdMS_TO_TICKS(1000 + std::rand() % 2000));
    logger.warn("Warning log message");

    vTaskDelay(pdMS_TO_TICKS(1000 + std::rand() % 2000));
    logger.error("Error log message");

    logger.setLevel(LogLevel::ERROR);
  }
}