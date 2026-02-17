#include "Button.h"
#include "LED.h"
#include "Logger.hpp"
#include "MPU6050Sensor.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <BLE.hpp>
#include <Constants.hpp>
#include <QueuePipe.hpp>
#include <memory>

// TODO: address why button.wasPressedAsync() doesn't work now
// TODO: extract control logic to separate class
// TODO: document overall architecture with C4 model

// extern "C" void app_main() {

//   vTaskDelay(pdMS_TO_TICKS(5000));

//   Logger logger((LogLevel::DEBUG));
//   BLE *ble = BLE::getInstance(&logger);
//   IMUSample sample = {.a = {.x = 1.0f, .y = 2.0f, .z = 3.0f},
//                       .w = {.roll = 4.0f, .pitch = 5.0f, .yaw = 6.0f},
//                       .t = 0};

//   while (true) {
//     ble->send(sample);
//     sample.a.x += 1.0f;
//     sample.a.y += 1.0f;
//     sample.a.z += 1.0f;
//     sample.w.roll += 1.0f;
//     sample.w.pitch += 1.0f;
//     sample.w.yaw += 1.0f;
//     sample.t += 1000;
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }
// }

extern "C" void app_main() {
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  Logger logger((LogLevel::DEBUG));

  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> samplingPipe =
      QueuePipe<IMUSample, SAMPLING_PIPE_SIZE>::create(&logger);
  if (!samplingPipe) {
    logger.error("Failed to create pipe");
    return;
  }

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

  std::unique_ptr<IMUSensor> imu = MPU6050Sensor::create(&logger, samplingPipe);
  if (imu == nullptr) {
    logger.error("Failed to initialize MPU6050 sensor");
    return;
  }

  BLE *ble = BLE::getInstance(&logger /* , &samplingPipe */);
  if (ble == nullptr) {
    logger.error("Failed to initialize BLE");
    return;
  }

  bool doSample = false;
  std::optional<IMUSample> sample;
  button->enableAsync();
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(2500));

    if (button->isPressedSync()) {
      doSample = !doSample;
      led->toggle();
      if (doSample) {
        imu->beginAsync();
      } else {
        imu->stopAsync();
      }
    }

    if (doSample) {
      sample = imu->readAsync();
      if (sample.has_value()) {
        ble->send(sample.value());
      }
    }
    button->disableAsync();
  }
}