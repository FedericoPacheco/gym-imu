#include "adapters/I2CReal.hpp"
#include "adapters/MPUReal.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <1-Calibration/MPU6050AccelAffineCalibrator.hpp>
#include <2-NoiseReduction/MPU6050NoiseReducer.hpp>
#include <3-Orientation/IMUMahonyOrientationFinder.hpp>
#include <4-GravityRemoval/IMUQuaternionGravityRemover.hpp>
#include <5-Velocity/IMUIntegrationVelocityEstimator.hpp>
#include <BLE.hpp>
#include <Button.hpp>
#include <Constants.hpp>
#include <FreeRTOSLoopRunner.hpp>
#include <FreeRTOSNotificationRunner.hpp>
#include <IMUSignalProcessor.hpp>
#include <LED.hpp>
#include <Logger.hpp>
#include <MPU6050Sensor.hpp>
#include <QueuePipe.hpp>
#include <Types.hpp>
#include <esp_system.h>
#include <memory>

/*
Configure firmware settings: pio run -t menuconfig

Advanced debugging guides:
https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/advanced-development/debugging/panic-exception.html

Watchdog:
If the watchdog panics, enable CONFIG_ESP_SYSTEM_USE_FRAME_POINTER (Component
Config > ESP System Settings > Backtrace Behavior) in the menuconfig. Then use:
riscv32-esp-elf-addr2line -pfiaC -e .pio/build/device/firmware.elf \
  <hexAddresses>
This maps addresses in flash to their corresponding source code lines and
functions. This only works well for the same build that produced the error.
Otherwise it may be misleading.

Ticks conversion:
configTICK_RATE_HZ defines the number of ticks per second. It
can be changed in the menuconfig: Component config > FreeRTOS > Kernel.
pdMS_TO_TICKS(x) can effectively output zero for short wall time intervals if
the tick rate is low (with x < 10 for configTICK_RATE_HZ = 100). BE CAREFUL
WITH THIS!
Tradeoff: increasing the tick rate improves resolution but also tick interrupts,
CPU load and potentially timing.
*/

#ifdef PROCESS_SIGNAL
using TransmittedSample = ProcessedSample;
#else
using TransmittedSample = IMUSample;
#endif

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

  TaskHandle_t coordinator = xTaskGetCurrentTaskHandle();

  auto ledRunner = std::make_unique<FreeRTOSLoopRunner>(
      "ledTask", LED_TASK_STACK_SIZE, LED_TASK_PRIORITY, pdMS_TO_TICKS(100));
  UARTLogger ledLogger("LED", LogLevel::WARN);
  std::unique_ptr<LED> led = LED::create(std::move(ledRunner), &ledLogger);
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

  UARTLogger samplingPipeLogger("SamplingPipe", LogLevel::INFO);
  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> samplingPipe =
      QueuePipe<IMUSample, SAMPLING_PIPE_SIZE>::create(&samplingPipeLogger);
  if (!samplingPipe) {
    samplingPipeLogger.error("Failed to create sampling pipe");
    return;
  }

  UARTLogger imuLogger("IMU", LogLevel::DEBUG);
  auto imuMPUPort = std::make_unique<MPUReal>();
  auto imuI2CPort = std::make_unique<I2CReal>();
  auto imuRunner = std::make_unique<FreeRTOSNotificationRunner>(
      "readTask", IMU_READ_TASK_STACK_SIZE, IMU_READ_TASK_PRIORITY);
#ifdef PROCESS_SIGNAL
  const bool useHWLowPassFilter = true;
#else
  const bool useHWLowPassFilter = false;
#endif
  IMUSensorPort *imu = MPU6050Sensor::getInstance(
      &imuLogger, samplingPipe, coordinator, SAMPLING_FREQUENCY_HZ,
      useHWLowPassFilter, std::move(imuMPUPort), std::move(imuI2CPort),
      std::move(imuRunner));
  if (imu == nullptr) {
    imuLogger.error("Failed to initialize MPU6050 sensor");
    return;
  }

#ifdef PROCESS_SIGNAL
  UARTLogger transmissionPipeLogger("TransmissionPipe", LogLevel::INFO);
  std::shared_ptr<Pipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>>
      transmissionPipe =
          QueuePipe<ProcessedSample, TRANSMISSION_PIPE_SIZE>::create(
              &transmissionPipeLogger);
  if (!transmissionPipe) {
    transmissionPipeLogger.error("Failed to create transmission pipe");
    return;
  }

  UARTLogger processorLogger("Processor", LogLevel::DEBUG);
  auto processorRunner = std::make_unique<FreeRTOSLoopRunner>(
      "processTask", PROCESS_TASK_STACK_SIZE, PROCESS_TASK_PRIORITY,
      pdMS_TO_TICKS(10)); // 5));
  std::unique_ptr<IMUCalibrator> calibrator =
      std::make_unique<MPU6050AccelAffineCalibrator>();
  std::unique_ptr<IMUNoiseReducer> noiseReducer =
      std::make_unique<MPU6050NoiseReducer>();
  std::unique_ptr<IMUOrientationFinder<Quaternion>> orientationFinder =
      std::make_unique<IMUMahonyOrientationFinder>(dt);
  std::unique_ptr<IMUGravityRemover<Quaternion>> gravityRemover =
      std::make_unique<IMUQuaternionGravityRemover>();
  std::unique_ptr<IMUVelocityEstimator> velocityEstimator =
      std::make_unique<IMUIntegrationVelocityEstimator>(dt);
  IMUSignalProcessor *processor = IMUSignalProcessor::getInstance(
      samplingPipe, transmissionPipe, std::move(processorRunner),
      &processorLogger, std::move(calibrator), std::move(noiseReducer),
      std::move(orientationFinder), std::move(gravityRemover),
      std::move(velocityEstimator), INITIALIZATION_SAMPLES_COUNT, coordinator);
  if (processor == nullptr) {
    processorLogger.error("Failed to initialize IMU signal processor");
    return;
  }

  UARTLogger bleLogger("BLE", LogLevel::DEBUG);
  auto bleLoopRunner = std::make_unique<FreeRTOSLoopRunner>(
      "transmitTask", TRANSMIT_TASK_STACK_SIZE, TRANSMIT_TASK_PRIORITY,
      pdMS_TO_TICKS(50));
  BLE<ProcessedSample> *ble = BLE<ProcessedSample>::getInstance(
      &bleLogger, transmissionPipe, std::move(bleLoopRunner), coordinator);
  if (ble == nullptr) {
    bleLogger.error("Failed to initialize BLE");
    return;
  }
#else
  UARTLogger bleLogger("BLE", LogLevel::DEBUG);
  auto bleLoopRunner = std::make_unique<FreeRTOSLoopRunner>(
      "transmitTask", TRANSMIT_TASK_STACK_SIZE, TRANSMIT_TASK_PRIORITY,
      pdMS_TO_TICKS(50));
  BLE<TransmittedSample> *ble = BLE<TransmittedSample>::getInstance(
      &bleLogger, samplingPipe, std::move(bleLoopRunner), coordinator);
  if (ble == nullptr) {
    bleLogger.error("Failed to initialize BLE");
    return;
  }
#endif

  // -------------------------------------------------------------------------

  auto stopPipeline = [&] -> void {
    imu->requestStopAsync();
    uint32_t maskValue = 0;
    BaseType_t wasNotified =
        xTaskNotifyWait(0, IMUSensorPort::STOPPED_MASK, &maskValue,
                        MAX_BLOCKING_TIME_AFTER_STOP_TICKS);
    if (wasNotified != pdTRUE ||
        (maskValue & IMUSensorPort::STOPPED_MASK) == 0) {
      imuLogger.error("Failed to stop IMU properly. Rebooting...");
      vTaskDelay(pdMS_TO_TICKS(100));
      esp_restart(); // Simplest recovery strategy: restart the device
    }

#ifdef PROCESS_SIGNAL
    processor->requestStopProcessing();
    maskValue = 0;
    wasNotified =
        xTaskNotifyWait(0, IMUSignalProcessor::STOPPED_MASK, &maskValue,
                        MAX_BLOCKING_TIME_AFTER_STOP_TICKS);
    if (wasNotified != pdTRUE ||
        (maskValue & IMUSignalProcessor::STOPPED_MASK) == 0) {
      processorLogger.error(
          "Failed to stop SignalProcessor properly. Rebooting...");
      vTaskDelay(pdMS_TO_TICKS(100));
      esp_restart();
    }
#endif

    ble->requestStopTransmission();
    maskValue = 0;
    wasNotified =
        xTaskNotifyWait(0, BLE<TransmittedSample>::STOPPED_MASK, &maskValue,
                        MAX_BLOCKING_TIME_AFTER_STOP_TICKS);
    if (wasNotified != pdTRUE ||
        (maskValue & BLE<TransmittedSample>::STOPPED_MASK) == 0) {
      bleLogger.error("Failed to stop BLE properly. Rebooting...");
      vTaskDelay(pdMS_TO_TICKS(100));
      esp_restart();
    }
  };

  bool doSample = false;
  button->enableAsync();
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(250));

    // Toggle sampling when user presses button, but only if a client is
    // connected
    // TODO: improve isConnected()
    if (button->wasPressedAsync() && ble->isConnected()) {
      doSample = !doSample;
      if (doSample) {
        ble->beginTransmission();
#ifdef PROCESS_SIGNAL
        processor->beginProcessing();
#endif
        imu->beginAsync();
      } else {
        stopPipeline();

        PipeMetrics samplingMetrics = samplingPipe->getMetrics();
        samplingPipeLogger.info(
            "Sampling pipe metrics: drops=%u, maxDepth=%u, currentDepth=%u",
            samplingMetrics.drops, samplingMetrics.maxDepth,
            samplingMetrics.currentDepth);
#ifdef PROCESS_SIGNAL
        PipeMetrics transmissionMetrics = transmissionPipe->getMetrics();
        transmissionPipeLogger.info("Transmission pipe metrics: drops=%u, "
                                    "maxDepth=%u, currentDepth=%u",
                                    transmissionMetrics.drops,
                                    transmissionMetrics.maxDepth,
                                    transmissionMetrics.currentDepth);
#endif
      }
    }

    // Stop sampling if BLE gets disconnected
    if (doSample && !ble->isConnected()) {
      doSample = false;
      stopPipeline();
    }

// Provide visual feedback to the user
#ifdef PROCESS_SIGNAL
    // Callbacks or events weren't used for simplicity
    switch (processor->getState()) {
    case IMUSignalProcessor::State::IDLE:
      led->turnOff();
      break;
    case IMUSignalProcessor::State::INITIALIZING:
      led->blink();
      break;
    case IMUSignalProcessor::State::PROCESSING:
    case IMUSignalProcessor::State::DRAINING:
      led->turnOn();
      break;
    }
#else
    if (doSample) {
      led->turnOn();
    } else {
      led->turnOff();
    }
#endif
  }
  button->disableAsync();
}
