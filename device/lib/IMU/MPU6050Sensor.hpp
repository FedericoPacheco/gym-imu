#pragma once
#include <Constants.hpp>
#include <ErrorMacros.hpp>
#include <IMUSensorPort.hpp>
#include <LoggerPort.hpp>
#include <Pipe.hpp>
#include <atomic>
#include <cstdint>
#include <memory>
#include <ports/ESP-IDF/GPIOPort.hpp>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>
#include <ports/MPU/MPUPort.hpp>
#include <tuple>

/*
Overview:
A class to interface with the MPU6050 IMU sensor, providing synchronous and
asynchronous sample reading capabilities.

How it works:
- Initializes and configures the MPU6050 sensor over I2C, as well as its FIFO
buffer and interrupt configuration.
- On syncronous read, it fetches the latest accelerometer and gyroscope data
from the sensor and converts them into physical units.
- On asynchronous read, it leverages FreeRTOS tasks, notifications and a pipe
(queue under the hood) to handle data reading in the background. An interrupt
service routine (ISR) is triggered when new data is available, notifying the
read task to fetch and process the data, which is then stored in a pipe for
later retrieval. This process can be turned on and off with a dedicated flag.

How to use:
Sync:
...
IMUSensorPort *imu = MPU6050Sensor::getInstance(&logger, &pipe, &sensorPort,
&i2cPort); while (true) { auto sampleOpt = imu->readSync(); if (sampleOpt) {
    // Process sample...
  }
}
Async:
gpio_install_isr_service(0); // Install ISR service once globally
IMUSensorPort *imu = MPU6050Sensor::getInstance(&logger, &pipe, &sensorPort,
&i2cPort);
imu->beginAsync();
while (true) {
  auto sampleOpt = imu->readAsync();
  if (sampleOpt) {
    // Process sample...
  }
}
imu->stopAsync();

Notes:
Requires to edit the MPU.testConnection() method to accept other
WHO_AM_I values due to the sensor being fake/clone/counterfeit (see setup.md
for details). This may affect the quality and reliability of the sensor data,
but it's accepted due to the difficulty and cost of getting better hardware
here in Argentina.
*/

class MPU6050Sensor : public IMUSensorPort {
public:
  static constexpr int READ_TASK_STACK_SIZE = 4096;
  static constexpr int READ_TASK_MAX_BATCH = 12;
  static constexpr int READ_TASK_PRIORITY = 5;

  static constexpr float g = 9.80665f; // m/s²

  static MPU6050Sensor *
  getInstance(LoggerPort *logger,
              std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
              MPUPort *sensor = nullptr, I2CPort *i2c = nullptr,
              gpio_num_t INTPin = GPIO_NUM_5, gpio_num_t SDAPin = GPIO_NUM_6,
              gpio_num_t SCLPin = GPIO_NUM_7, int samplingFrequencyHz = 30);

  ~MPU6050Sensor() override;

  std::optional<IMUSample> readSync() override;
  std::optional<IMUSample> readAsync() override;
  void beginAsync() override;
  void stopAsync() override;

private:
  static constexpr mpud::types::accel_fs_t ACCELEROMETER_SCALE =
      mpud::ACCEL_FS_2G;
  static constexpr mpud::types::gyro_fs_t GYROSCOPE_SCALE =
      mpud::GYRO_FS_250DPS;
  static constexpr int BUS_FREQUENCY_HZ = 400000; // 400kHz, short wires (<10cm)
  // static constexpr int BUS_FREQUENCY_HZ = 100000; // 100kHz, long wires

  static constexpr int FIFO_PACKET_SIZE = 12;

  LoggerPort *logger;
  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe;
  MPUPort *sensor;
  I2CPort *bus;
  gpio_num_t INTPin, SDAPin, SCLPin;
  int samplingFrequencyHz;

  struct InstanceState {
    std::unique_ptr<MPU6050Sensor> instance;
    SemaphoreHandle_t semaphoreHandle;
    StaticSemaphore_t semaphoreControlBlock;
    portMUX_TYPE mux;
  } static instanceState;

  // Prevent races between main and read tasks
  std::atomic<bool> doRead;
  TaskHandle_t readTaskHandle;

  static IRAM_ATTR void isrHandler(void *arg);
  static void readTask(void *arg);
  void setDoRead(bool value);
  bool getDoRead();

  MPU6050Sensor(LoggerPort *logger,
                std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
                MPUPort *sensor, I2CPort *i2c, gpio_num_t INTPin,
                gpio_num_t SDAPin, gpio_num_t SCLPin, int samplingFrequencyHz);

  static std::unique_ptr<MPU6050Sensor>
  create(LoggerPort *logger,
         std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
         MPUPort *sensor = nullptr, I2CPort *i2c = nullptr,
         gpio_num_t INTPin = GPIO_NUM_5, gpio_num_t SDAPin = GPIO_NUM_6,
         gpio_num_t SCLPin = GPIO_NUM_7, int samplingFrequencyHz = 30);
  bool initializeI2CBus();
  bool resetSensor();
  void performDiagnostics();
  bool testConnection();
  bool initializeSensor();
  bool configureSettings();
  bool setupDMPQueue();
  bool configureInterrupts();
  bool setupTask();

  void batchReadDMPQueue(uint16_t initialFifoCount);
  std::tuple<mpud::raw_axes_t, mpud::raw_axes_t>
  parseSensorData(const uint8_t *data);
  IMUSample toIMUSample(mpud::raw_axes_t aRaw, mpud::raw_axes_t wRaw);
};