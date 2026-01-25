#pragma once
#include "IMUSensor.hpp"
#include "Logger.hpp"
#include "MPU.hpp"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "mpu/types.hpp"
#include "soc/gpio_num.h"
#include <atomic>
#include <tuple>

/*
  A class to interface with the MPU6050 IMU sensor, providing synchronous and
  asynchronous sample reading capabilities.

  How it works:
  - Initializes and configures the MPU6050 sensor over I2C, as well as its FIFO
  buffer and interrupt configuration.
  - On syncronous read, it fetches the latest accelerometer and gyroscope data
  from the sensor and converts them into physical units.
  - On asynchronous read, it leverages FreeRTOS tasks, notifications and queues
  to handle data reading in the background. An interrupt service routine (ISR)
  is triggered when new data is available, notifying the read task to fetch and
  process the data, which is then stored in a queue for later retrieval. This
  process can be turned on and off with a dedicated flag.

  How to use:
  Sync:
    std::unique_ptr<IMUSensor> imu = std::make_unique<MPU6050Sensor>(logger);
    while (true) {
      auto sampleOpt = imu->readSync();
      if (sampleOpt) {
        // Process sample...
      }
    }
  Async:
    std::unique_ptr<IMUSensor> imu = std::make_unique<MPU6050Sensor>(logger);
    imu->beginAsync();
    while (true) {
      auto sampleOpt = imu->readAsync();
      if (sampleOpt) {
        // Process sample...
      }
    }
    imu->stopAsync();
*/

class MPU6050Sensor : public IMUSensor {
private:
  static constexpr mpud::types::accel_fs_t ACCELEROMETER_SCALE =
      mpud::ACCEL_FS_2G;
  static constexpr mpud::types::gyro_fs_t GYROSCOPE_SCALE =
      mpud::GYRO_FS_250DPS;
  static constexpr int BUS_FREQUENCY_HZ = 400000; // 400kHz

  static constexpr int SAMPLE_QUEUE_SIZE = 128;
  static constexpr int FIFO_PACKET_SIZE = 12;
  static constexpr int READ_TASK_STACK_SIZE = 4096;
  static constexpr int READ_TASK_PRIORITY = 10;

  static constexpr float g = 9.80665f; // m/s²

  gpio_num_t INTPin, SDAPin, SCLPin;
  int samplingFrequencyHz;

  Logger *logger;

  // Prevent races between main and read tasks
  std::atomic<bool> doRead;

  uint8_t sampleQueueBuffer[SAMPLE_QUEUE_SIZE * sizeof(IMUSample)];
  StaticQueue_t sampleQueueControlBlock;
  QueueHandle_t sampleQueue;
  TaskHandle_t readTaskHandle;

  mpud::MPU sensor;

  static IRAM_ATTR void isrHandler(void *arg);
  static void readTask(void *arg);

  void setDoRead(bool value);
  bool getDoRead();

  std::tuple<mpud::raw_axes_t, mpud::raw_axes_t>
  parseSensorData(const uint8_t *data);
  IMUSample convertToSample(mpud::raw_axes_t aRaw, mpud::raw_axes_t wRaw);

public:
  MPU6050Sensor(Logger *logger, gpio_num_t INTPin = GPIO_NUM_5,
                gpio_num_t SDAPin = GPIO_NUM_6, gpio_num_t SCLPin = GPIO_NUM_7,
                int samplingFrequencyHz = 100);
  ~MPU6050Sensor() override;

  std::optional<IMUSample> readSync() override;
  std::optional<IMUSample> readAsync() override;
  void beginAsync() override;
  void stopAsync() override;
};