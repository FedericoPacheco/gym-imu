#pragma once
#include "I2Cbus.hpp"
#include "IMUSensor.hpp"
#include "Logger.hpp"
#include "MPU.hpp"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "hal/i2c_types.h"
#include "mpu/types.hpp"
#include "soc/gpio_num.h"
#include <atomic>
#include <cstdint>
#include <memory>
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
    Logger logger((LogLevel::DEBUG));
    std::unique_ptr<IMUSensor> imu = MPU6050Sensor::create(&logger);
    while (true) {
      auto sampleOpt = imu->readSync();
      if (sampleOpt) {
        // Process sample...
      }
    }
  Async:
    Logger logger((LogLevel::DEBUG));
    std::unique_ptr<IMUSensor> imu = MPU6050Sensor::create(&logger);
    imu->beginAsync();
    while (true) {
      auto sampleOpt = imu->readAsync();
      if (sampleOpt) {
        // Process sample...
      }
    }
    imu->stopAsync();

  Notes:
  Requires to edit the MPU.testConnection() method to accept other WHO_AM_I
  values due to the sensor being fake/clone/counterfeit (see setup.md for
  details). This may affect the quality and reliability of the sensor data, but
  it's accepted due to the difficulty and cost of getting better hardware here
  in Argentina.
  */

class MPU6050Sensor : public IMUSensor {
private:
  static constexpr mpud::types::accel_fs_t ACCELEROMETER_SCALE =
      mpud::ACCEL_FS_2G;
  static constexpr mpud::types::gyro_fs_t GYROSCOPE_SCALE =
      mpud::GYRO_FS_250DPS;
  static constexpr int BUS_FREQUENCY_HZ = 400000; // 400kHz, short wires (<10cm)
  // static constexpr int BUS_FREQUENCY_HZ = 100000; // 100kHz, long wires

  static constexpr int SAMPLE_QUEUE_SIZE = 128;
  static constexpr int FIFO_PACKET_SIZE = 12;

  static constexpr int READ_TASK_STACK_SIZE = 4096;
  static constexpr int READ_TASK_MAX_BATCH = 12;
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

  MPU_t sensor;
  I2C_t bus;

  static IRAM_ATTR void isrHandler(void *arg);
  static void readTask(void *arg);

  void setDoRead(bool value);
  bool getDoRead();

  MPU6050Sensor(Logger *logger, gpio_num_t INTPin, gpio_num_t SDAPin,
                gpio_num_t SCLPin, int samplingFrequencyHz);

  void batchReadDMPQueue(uint16_t initialFifoCount);
  bool dropOldestSampleIfFull();
  std::tuple<mpud::raw_axes_t, mpud::raw_axes_t>
  parseSensorData(const uint8_t *data);
  IMUSample toIMUSample(mpud::raw_axes_t aRaw, mpud::raw_axes_t wRaw);

  // create() helper methods
  static bool initializeI2CBus(MPU6050Sensor *imu);
  static bool resetSensor(MPU6050Sensor *imu);
  static void performDiagnostics(MPU6050Sensor *imu);
  static bool testConnection(MPU6050Sensor *imu);
  static bool initializeSensor(MPU6050Sensor *imu);
  static bool configureSettings(MPU6050Sensor *imu);
  static bool setupDMPQueue(MPU6050Sensor *imu);
  static bool configureInterrupts(MPU6050Sensor *imu);
  static bool setupAsyncComponents(MPU6050Sensor *imu);

public:
  // Factory method, returns null on failure
  static std::unique_ptr<IMUSensor> create(Logger *logger,
                                           gpio_num_t INTPin = GPIO_NUM_5,
                                           gpio_num_t SDAPin = GPIO_NUM_6,
                                           gpio_num_t SCLPin = GPIO_NUM_7,
                                           int samplingFrequencyHz = 64);
  ~MPU6050Sensor() override;

  std::optional<IMUSample> readSync() override;
  std::optional<IMUSample> readAsync() override;
  void beginAsync() override;
  void stopAsync() override;
};