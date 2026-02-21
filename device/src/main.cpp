#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <BLE.hpp>
#include <Button.hpp>
#include <Constants.hpp>
#include <LED.hpp>
#include <Logger.hpp>
#include <MPU6050Sensor.hpp>
#include <QueuePipe.hpp>
#include <memory>

extern "C" void app_main() {
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

  Logger samplingPipeLogger("SamplingPipe", LogLevel::WARN);
  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> samplingPipe =
      QueuePipe<IMUSample, SAMPLING_PIPE_SIZE>::create(&samplingPipeLogger);
  if (!samplingPipe) {
    samplingPipeLogger.error("Failed to create pipe");
    return;
  }

  Logger ledLogger("LED", LogLevel::WARN);
  std::unique_ptr<LED> led = LED::create(&ledLogger);
  if (led == nullptr) {
    ledLogger.error("Failed to initialize LED");
    return;
  }

  Logger buttonLogger("Button", LogLevel::WARN);
  std::unique_ptr<Button> button = Button::create(&buttonLogger);
  if (button == nullptr) {
    buttonLogger.error("Failed to initialize Button");
    return;
  }

  Logger imuLogger("IMU", LogLevel::WARN);
  IMUSensor *imu = MPU6050Sensor::getInstance(&imuLogger, samplingPipe);
  if (imu == nullptr) {
    imuLogger.error("Failed to initialize MPU6050 sensor");
    return;
  }

  Logger bleLogger("BLE", LogLevel::WARN);
  BLE *ble = BLE::getInstance(&bleLogger, samplingPipe);
  if (ble == nullptr) {
    bleLogger.error("Failed to initialize BLE");
    return;
  }

  bool doSample = false;
  button->enableAsync();
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (button->wasPressedAsync() && ble->isConnected()) {
      doSample = !doSample;
      led->toggle();
      if (doSample) {
        imu->beginAsync();
        ble->beginTransmission();
      } else {
        imu->stopAsync();
        ble->stopTransmission();
      }
    }
  }
  button->disableAsync();
}