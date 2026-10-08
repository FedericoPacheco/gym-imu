#pragma once
#include <Constants.hpp>
#include <ErrorMacros.hpp>
#include <IMUSensorPort.hpp>
#include <LoggerPort.hpp>
#include <NotificationRunner.hpp>
#include <Pipe.hpp>
#include <atomic>
#include <cstdint>
#include <memory>
#include <ports/ESP-IDF/ESPIDFPort.hpp>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>
#include <ports/MPU/MPUPort.hpp>
#include <tuple>
#include <utility>

/*
Overview:
A class to interface with the MPU6050 IMU sensor that stores samples in a pipe
for later consumption by other tasks. Also provides sync/async reading
capabilities, but they are not currently use directly in the project.

How it works:
- Initializes and configures the MPU6050 sensor over I2C, as well as its FIFO
buffer and interrupt configuration.
- It leverages FreeRTOS tasks, notifications and a pipe (queue under the hood)
to handle data reading in the background. An interrupt service routine (ISR)
 is triggered when new data is available, notifying the read task to fetch
and process the data, which is finally stored in a pipe for later retrieval.
This process can be turned on and off with a dedicated flag.
- On synchronous read, it fetches the latest accelerometer and gyroscope data
directly from the sensor's registers.
- On asynchronous read, it pops one sample from the pipe, if available.

How to use:
Refer to main.cpp

Notes:
The underlying driver's MPU.testConnection() method has been modified to accept
other WHO_AM_I values due to the sensor being fake/clone/counterfeit. This is
a workaround for the limitations of the available hardware here in Argentina.
*/

class MPU6050Sensor : public IMUSensorPort {
public:
  static MPU6050Sensor *
  getInstance(LoggerPort *logger,
              std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
              std::unique_ptr<MPUPort> sensor = nullptr,
              std::unique_ptr<I2CPort> i2c = nullptr,
              std::unique_ptr<NotificationRunner> runner = nullptr,
              gpio_num_t INTPin = GPIO_NUM_2, gpio_num_t SDAPin = GPIO_NUM_6,
              gpio_num_t SCLPin = GPIO_NUM_7,
              int samplingFrequencyHz = SAMPLING_FREQUENCY_HZ);

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)
  static void resetInstanceForTests();
#endif

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
  static constexpr mpud::dlpf_t DLPF_CONFIG = mpud::DLPF_42HZ;
  static constexpr int BUS_FREQUENCY_HZ = 400000; // 400kHz, short wires (<10cm)
  // static constexpr int BUS_FREQUENCY_HZ = 100000; // 100kHz, long wires

  static constexpr int FIFO_PACKET_SIZE = 12;

  LoggerPort *logger;
  std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe;
  std::unique_ptr<MPUPort> sensor;
  std::unique_ptr<I2CPort> bus;
  std::unique_ptr<NotificationRunner> runner;
  gpio_num_t INTPin, SDAPin, SCLPin;
  int samplingFrequencyHz;

  struct InstanceState {
    std::unique_ptr<MPU6050Sensor> instance;
    SemaphoreHandle_t semaphoreHandle;
    StaticSemaphore_t semaphoreControlBlock;
    portMUX_TYPE mux;
  } static instanceState;

  std::atomic<bool> doRead{false};
  std::atomic<uint32_t> nextSeq{0};

  MPU6050Sensor(LoggerPort *logger,
                std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
                std::unique_ptr<MPUPort> sensor, std::unique_ptr<I2CPort> i2c,
                std::unique_ptr<NotificationRunner> runner, gpio_num_t INTPin,
                gpio_num_t SDAPin, gpio_num_t SCLPin, int samplingFrequencyHz);

  static std::unique_ptr<MPU6050Sensor>
  create(LoggerPort *logger,
         std::shared_ptr<Pipe<IMUSample, SAMPLING_PIPE_SIZE>> pipe,
         std::unique_ptr<MPUPort> sensor = nullptr,
         std::unique_ptr<I2CPort> i2c = nullptr,
         std::unique_ptr<NotificationRunner> runner = nullptr,
         gpio_num_t INTPin = GPIO_NUM_2, gpio_num_t SDAPin = GPIO_NUM_6,
         gpio_num_t SCLPin = GPIO_NUM_7,
         int samplingFrequencyHz = SAMPLING_FREQUENCY_HZ);
  bool initializeI2CBus();
  bool resetSensor();
  void performDiagnostics();
  bool testConnection();
  bool initializeSensor();
  bool configureSettings();
  bool setupDMPQueue();
  bool configureInterrupts();
  bool setupTask();

  static IRAM_ATTR void isrHandler(void *arg);
  static void onReadTaskNotification(void *arg, uint32_t notificationValue);
  void setDoRead(bool value);
  bool getDoRead();
  void batchReadDMPQueue(uint16_t initialFifoCount);
  std::tuple<mpud::raw_axes_t, mpud::raw_axes_t>
  parseSensorData(const uint8_t *data);
  IMUSample toIMUSample(mpud::raw_axes_t aRaw, mpud::raw_axes_t wRaw);
  void setNextSeq();
  uint32_t getNextSeq();
};