#include "Button.h"
#include "LED.h"
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
  logger.debug("DEBUG Sanity Check");
  logger.info("INFO Sanity Check");
  logger.warn("WARN Sanity Check");
  logger.error("ERROR Sanity Check");

  std::unique_ptr<LED> led = LED::create(&logger);
  if (led == nullptr) {
    logger.error("Failed to initialize LED");
    return;
  }

  std::unique_ptr<Button> button = Button::create(&logger);
  if (button == nullptr) {
    logger.error("Failed to initialize Button");
    return;
  }

  std::unique_ptr<IMUSensor> imu = MPU6050Sensor::create(&logger);
  if (imu == nullptr) {
    logger.error("Failed to initialize MPU6050 sensor");
    return;
  }

  bool doSample = false;
  button->enableAsync();
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (button->wasPressedAsync()) {
      doSample = !doSample;
      led->toggle();
      if (doSample) {
        imu->beginAsync();
      } else {
        imu->stopAsync();
      }
    }

    if (doSample) {
      imu->readAsync();
    }
  }
  button->disableAsync();
}