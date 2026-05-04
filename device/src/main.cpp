#include "adapters/I2CReal.hpp"
#include "adapters/MPUReal.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <BLE.hpp>
#include <Button.hpp>
#include <Constants.hpp>
#include <FreeRTOSLoopRunner.hpp>
#include <FreeRTOSNotificationRunner.hpp>
#include <IMUSignalProcessor.hpp>
#include <LED.hpp>
#include <Logger.hpp>
#include <MPU6050AffineCalibrator.hpp>
#include <MPU6050Sensor.hpp>
#include <QueuePipe.hpp>
#include <memory>

extern "C" void app_main() {
  /* Note:
   Do NOT use the ESP_INTR_FLAG_IRAM here. Depending on timing, it produces the
   watchdog to panic and restart the system indefinitively. The flag enables
   ISRs to run when flash operations (write/erase) are in progress, but it also
   prevents the use of any code that lives in flash from within the ISR, which
   effectively prevents me from using the Runner abstractions I've built (not
   IRAM-safe). Tradeoff: slight delays or misses in rare cases that may make me
   lose some samples (acceptable due to soft real-time requirements and mission
   criticality) vs. more complex, less flexible, code.
  */
  gpio_install_isr_service(0);

  UARTLogger ledLogger("LED", LogLevel::WARN);
  std::unique_ptr<LED> led = LED::create(&ledLogger);
  if (led == nullptr) {
    ledLogger.error("Failed to initialize LED");
    return;
  }

  UARTLogger buttonLogger("Button", LogLevel::WARN);
  std::unique_ptr<Button> button = Button::create(&buttonLogger);
  if (button == nullptr) {
    buttonLogger.error("Failed to initialize Button");
    return;
  }

  UARTLogger samplingPipeLogger("SamplingPipe", LogLevel::WARN);
  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> samplingPipe =
      QueuePipe<IMUSample, SAMPLING_PIPE_SIZE>::create(&samplingPipeLogger);
  if (!samplingPipe) {
    samplingPipeLogger.error("Failed to create sampling pipe");
    return;
  }
  UARTLogger transmissionPipeLogger("TransmissionPipe", LogLevel::WARN);
  std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> transmissionPipe =
      QueuePipe<IMUSample, TRANSMISSION_PIPE_SIZE>::create(
          &transmissionPipeLogger);
  if (!transmissionPipe) {
    transmissionPipeLogger.error("Failed to create transmission pipe");
    return;
  }

  UARTLogger imuLogger("IMU", LogLevel::DEBUG);
  auto imuMPUPort = std::make_unique<MPUReal>();
  auto imuI2CPort = std::make_unique<I2CReal>();
  auto imuRunner = std::make_unique<FreeRTOSNotificationRunner>(
      "readTask", IMU_READ_TASK_STACK_SIZE, IMU_READ_TASK_PRIORITY);
  IMUSensorPort *imu = MPU6050Sensor::getInstance(
      &imuLogger, samplingPipe, std::move(imuMPUPort), std::move(imuI2CPort),
      std::move(imuRunner));
  if (imu == nullptr) {
    imuLogger.error("Failed to initialize MPU6050 sensor");
    return;
  }

  UARTLogger processorLogger("Processor", LogLevel::DEBUG);
  auto processorRunner = std::make_unique<FreeRTOSLoopRunner>(
      "processTask", PROCESS_TASK_STACK_SIZE, PROCESS_TASK_PRIORITY,
      pdMS_TO_TICKS(25));
  std::unique_ptr<IMUCalibrator> calibrator =
      std::make_unique<MPU6050AffineCalibrator>();
  IMUSignalProcessor processor(samplingPipe, transmissionPipe,
                               std::move(processorRunner), &processorLogger,
                               std::move(calibrator));

  UARTLogger bleLogger("BLE", LogLevel::DEBUG);
  auto bleLoopRunner = std::make_unique<FreeRTOSLoopRunner>(
      "transmitTask", TRANSMIT_TASK_STACK_SIZE, TRANSMIT_TASK_PRIORITY,
      pdMS_TO_TICKS(100));
  BLE *ble =
      BLE::getInstance(&bleLogger, transmissionPipe, std::move(bleLoopRunner));
  if (ble == nullptr) {
    bleLogger.error("Failed to initialize BLE");
    return;
  }

  bool doSample = false;
  button->enableAsync();
  processor.beginProcessing();
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
  processor.stopProcessing();
  button->disableAsync();
}