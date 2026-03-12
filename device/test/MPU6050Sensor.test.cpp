#include <gmock/gmock.h>
#include <gtest/gtest.h>

extern "C" {
#include <fff.h>
}
DEFINE_FFF_GLOBALS;

#include "doubles/ESPIDFDouble.cpp"
#include "doubles/ESPIDFDouble.hpp"
#include "doubles/FreeRTOSDouble.cpp"
#include "doubles/FreeRTOSDouble.hpp"
#include "doubles/I2CDouble.hpp"
#include "doubles/LoggerDouble.hpp"
#include "doubles/MPUDouble.hpp"
#include "doubles/PipeDouble.hpp"
#include <DeterministicNotificationRunner.hpp>
#include <MPU6050Sensor.hpp>

LoggerDouble gLogger;

namespace {

using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::WithArg;

constexpr int FIFO_PACKET_SIZE = 12;

// ----------------------------------------------------------------------------------------------------
// Helpers for setting up tests
class StartFailRunner final : public DeterministicNotificationRunner {
public:
  bool start(std::function<void(void *, uint32_t)>, void *) override {
    return false;
  }
};
struct MPU6050SensorDependencies {
  std::shared_ptr<testing::NiceMock<PipeDouble>> pipe;
  std::unique_ptr<testing::NiceMock<MPUDouble>> sensor;
  std::unique_ptr<testing::NiceMock<I2CDouble>> i2c;
  std::unique_ptr<NotificationRunner> runner;
};
MPU6050SensorDependencies buildDefaultDependencies() {
  MPU6050Sensor::resetInstanceForTests();

  // Initialization
  resetFreeRTOSPortFakes();
  rtosCreateMutexStatic_fake.return_val =
      reinterpret_cast<SemaphoreHandle_t>(0x1);
  rtosSemaphoreTake_fake.return_val = pdTRUE;
  rtosSemaphoreGive_fake.return_val = pdTRUE;

  resetESPIDFPortFakes();
  gpioSetConfig_fake.return_val = ESP_OK;
  gpioAddISRHandler_fake.return_val = ESP_OK;

  MPU6050SensorDependencies deps;
  deps.pipe = std::make_shared<testing::NiceMock<PipeDouble>>();
  deps.sensor = std::make_unique<testing::NiceMock<MPUDouble>>();
  deps.i2c = std::make_unique<testing::NiceMock<I2CDouble>>();
  deps.runner = std::make_unique<DeterministicNotificationRunner>();

  ON_CALL(*deps.i2c, begin(_, _, _, _, _)).WillByDefault(Return(ESP_OK));

  ON_CALL(*deps.sensor, lastError()).WillByDefault(Return(ESP_OK));
  ON_CALL(*deps.sensor, reset()).WillByDefault(Return(ESP_OK));
  ON_CALL(*deps.sensor, testConnection()).WillByDefault(Return(ESP_OK));
  ON_CALL(*deps.sensor, initialize()).WillByDefault(Return(ESP_OK));

  ON_CALL(*deps.sensor, setSampleRate(_)).WillByDefault(Return(ESP_OK));
  ON_CALL(*deps.sensor, setAccelFullScale(_)).WillByDefault(Return(ESP_OK));
  ON_CALL(*deps.sensor, setGyroFullScale(_)).WillByDefault(Return(ESP_OK));

  ON_CALL(*deps.sensor, setFIFOConfig(_)).WillByDefault(Return(ESP_OK));
  ON_CALL(*deps.sensor, setFIFOEnabled(_)).WillByDefault(Return(ESP_OK));

  ON_CALL(*deps.sensor, setInterruptConfig(_)).WillByDefault(Return(ESP_OK));
  ON_CALL(*deps.sensor, setInterruptEnabled(_)).WillByDefault(Return(ESP_OK));

  // Sensor sampling
  getTimeUs_fake.return_val = 123456;

  ON_CALL(*deps.sensor, resetFIFO()).WillByDefault(Return(ESP_OK));
  ON_CALL(*deps.sensor, getFIFOCount()).WillByDefault(Return(0));
  ON_CALL(*deps.sensor, readFIFO(_, _)).WillByDefault(Return(ESP_OK));

  ON_CALL(*deps.pipe, pop(_)).WillByDefault(Return(std::nullopt));
  ON_CALL(*deps.pipe, push(_)).WillByDefault(Return(true));

  return deps;
}
MPU6050Sensor *getInstanceWith(MPU6050SensorDependencies &&deps) {
  return MPU6050Sensor::getInstance(&gLogger, deps.pipe, std::move(deps.sensor),
                                    std::move(deps.i2c),
                                    std::move(deps.runner));
}

esp_err_t fillDeterministicFIFOPacket(size_t length, uint8_t *data) {
  if (length != static_cast<size_t>(FIFO_PACKET_SIZE) || data == nullptr)
    return ESP_FAIL;

  static constexpr uint8_t packet[FIFO_PACKET_SIZE] = {
      0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x0F,
  };

  for (size_t i = 0; i < static_cast<size_t>(FIFO_PACKET_SIZE); ++i)
    data[i] = packet[i];

  return ESP_OK;
}

// ----------------------------------------------------------------------------------------------------
// Initialization tests

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenI2CFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.i2c, begin(_, _, _, _, _)).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenSettingBusToSensorFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, lastError()).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenSettingAddressToSensorFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, lastError())
      .WillOnce(Return(ESP_OK))
      .WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenResetSensorFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, reset()).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance,
     ReturnsNullWhenConnectionTestFailsAfterMaxRetries) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, testConnection())
      .Times(6)
      .WillRepeatedly(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenSensorInitializationFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, initialize()).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenSettingSampleRateFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, setSampleRate(_)).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenSettingAccelerationScaleFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, setAccelFullScale(_)).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenSettingGyroscopeScaleFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, setGyroFullScale(_)).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenConfiguringFIFOFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, setFIFOConfig(_)).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenEnablingFIFOFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, setFIFOEnabled(_)).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenConfiguringGPIOFails) {
  auto deps = buildDefaultDependencies();
  gpioSetConfig_fake.return_val = ESP_FAIL;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenAddingISRHandlerFails) {
  auto deps = buildDefaultDependencies();
  gpioAddISRHandler_fake.return_val = ESP_FAIL;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenConfiguringInterruptFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, setInterruptConfig(_)).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenEnablingInterruptFails) {
  auto deps = buildDefaultDependencies();
  EXPECT_CALL(*deps.sensor, setInterruptEnabled(_)).WillOnce(Return(ESP_FAIL));

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, ReturnsNullWhenRunnerStartFails) {
  auto deps = buildDefaultDependencies();
  deps.runner = std::make_unique<StartFailRunner>();

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, InitializesSuccessfully) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicNotificationRunner *>(deps.runner.get());

  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_NE(instance, nullptr);
  EXPECT_GT(runnerRaw->getStartCallCount(), 0);
}

// ----------------------------------------------------------------------------------------------------
// Sensor sampling tests

TEST(MPU6050Sensor_onReadTaskNotification,
     SendsMultipleSamplesToPipeWhenFifoCountIsValid) {
  auto deps = buildDefaultDependencies();
  auto *runner =
      static_cast<DeterministicNotificationRunner *>(deps.runner.get());
  auto *sensor = deps.sensor.get();
  auto *pipe = deps.pipe.get();
  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_CALL(*sensor, resetFIFO()).Times(1); // beginAsync() call
  EXPECT_CALL(*sensor, getFIFOCount()).WillOnce(Return(3 * FIFO_PACKET_SIZE));
  EXPECT_CALL(*sensor, lastError()).WillOnce(Return(ESP_OK));
  EXPECT_CALL(*sensor, readFIFO(FIFO_PACKET_SIZE, _))
      .Times(3)
      .WillRepeatedly(Invoke(fillDeterministicFIFOPacket));
  EXPECT_CALL(*pipe, push(_)).Times(3).WillRepeatedly(Return(true));

  instance->beginAsync();
  runner->notify();
  runner->runOneStep();
  instance->stopAsync();
}

TEST(MPU6050Sensor_onReadTaskNotification,
     ConvertsRawBytesAndPushesSampleWithGravityScaling) {
  auto deps = buildDefaultDependencies();
  auto *runner =
      static_cast<DeterministicNotificationRunner *>(deps.runner.get());
  auto *sensor = deps.sensor.get();
  auto *pipe = deps.pipe.get();
  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_CALL(*sensor, resetFIFO()).Times(1);
  EXPECT_CALL(*sensor, getFIFOCount()).WillOnce(Return(FIFO_PACKET_SIZE));
  EXPECT_CALL(*sensor, lastError()).WillOnce(Return(ESP_OK));
  EXPECT_CALL(*sensor, readFIFO(FIFO_PACKET_SIZE, _))
      .Times(1)
      .WillOnce(Invoke(fillDeterministicFIFOPacket));
  EXPECT_CALL(*pipe, push(_))
      .Times(1)
      .WillOnce(DoAll(WithArg<0>([](const IMUSample &sample) {
                        EXPECT_FLOAT_EQ(sample.a.x, 1.0f * MPU6050Sensor::g);
                        EXPECT_FLOAT_EQ(sample.a.y, 2.0f * MPU6050Sensor::g);
                        EXPECT_FLOAT_EQ(sample.a.z, 3.0f * MPU6050Sensor::g);
                        EXPECT_FLOAT_EQ(sample.w.roll, 5.0f);
                        EXPECT_FLOAT_EQ(sample.w.pitch, 10.0f);
                        EXPECT_FLOAT_EQ(sample.w.yaw, 15.0f);
                        EXPECT_EQ(sample.t, 123456);
                      }),
                      Return(true)));

  instance->beginAsync();
  runner->notify();
  runner->runOneStep();
  instance->stopAsync();
}

TEST(MPU6050Sensor_onReadTaskNotification,
     ResetsFIFOWhenFifoCountReadReturnsError) {
  auto deps = buildDefaultDependencies();
  auto *runner =
      static_cast<DeterministicNotificationRunner *>(deps.runner.get());
  auto *sensor = deps.sensor.get();
  auto *pipe = deps.pipe.get();
  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_CALL(*sensor, resetFIFO()).Times(2);
  EXPECT_CALL(*sensor, getFIFOCount()).WillOnce(Return(FIFO_PACKET_SIZE));
  EXPECT_CALL(*sensor, lastError()).WillOnce(Return(ESP_FAIL));
  EXPECT_CALL(*sensor, readFIFO(_, _)).Times(0);
  EXPECT_CALL(*pipe, push(_)).Times(0);

  instance->beginAsync();
  runner->notify();
  runner->runOneStep();
  instance->stopAsync();
}

TEST(MPU6050Sensor_onReadTaskNotification,
     ResetsFIFOWhenFifoByteCountIsMisaligned) {
  auto deps = buildDefaultDependencies();
  auto *runner =
      static_cast<DeterministicNotificationRunner *>(deps.runner.get());
  auto *sensor = deps.sensor.get();
  auto *pipe = deps.pipe.get();
  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_CALL(*sensor, resetFIFO()).Times(2);
  EXPECT_CALL(*sensor, getFIFOCount()).WillOnce(Return(FIFO_PACKET_SIZE + 1));
  EXPECT_CALL(*sensor, lastError()).WillOnce(Return(ESP_OK));
  EXPECT_CALL(*sensor, readFIFO(_, _)).Times(0);
  EXPECT_CALL(*pipe, push(_)).Times(0);

  instance->beginAsync();
  runner->notify();
  runner->runOneStep();
  instance->stopAsync();
}

TEST(MPU6050Sensor_onReadTaskNotification,
     ResetsFIFOWhenReadFIFOFailsDuringBatchRead) {
  auto deps = buildDefaultDependencies();
  auto *runner =
      static_cast<DeterministicNotificationRunner *>(deps.runner.get());
  auto *sensor = deps.sensor.get();
  auto *pipe = deps.pipe.get();
  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_CALL(*sensor, resetFIFO()).Times(2);
  EXPECT_CALL(*sensor, getFIFOCount()).WillOnce(Return(2 * FIFO_PACKET_SIZE));
  EXPECT_CALL(*sensor, lastError()).WillOnce(Return(ESP_OK));
  EXPECT_CALL(*sensor, readFIFO(FIFO_PACKET_SIZE, _))
      .WillOnce(Invoke(fillDeterministicFIFOPacket))
      .WillOnce(Return(ESP_FAIL));
  EXPECT_CALL(*pipe, push(_)).Times(1).WillOnce(Return(true));

  instance->beginAsync();
  runner->notify();
  runner->runOneStep();
  instance->stopAsync();
}

TEST(MPU6050Sensor_onReadTaskNotification,
     ResetsFIFOWhenBacklogRemainsAfterMaxBatchProcessing) {
  auto deps = buildDefaultDependencies();
  auto *runner =
      static_cast<DeterministicNotificationRunner *>(deps.runner.get());
  auto *sensor = deps.sensor.get();
  auto *pipe = deps.pipe.get();
  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_CALL(*sensor, resetFIFO()).Times(2);
  EXPECT_CALL(*sensor, getFIFOCount())
      .WillOnce(Return((IMU_READ_TASK_MAX_BATCH + 2) * FIFO_PACKET_SIZE));
  EXPECT_CALL(*sensor, lastError()).WillOnce(Return(ESP_OK));
  EXPECT_CALL(*sensor, readFIFO(FIFO_PACKET_SIZE, _))
      .Times(IMU_READ_TASK_MAX_BATCH)
      .WillRepeatedly(Invoke(fillDeterministicFIFOPacket));
  EXPECT_CALL(*pipe, push(_))
      .Times(IMU_READ_TASK_MAX_BATCH)
      .WillRepeatedly(Return(true));

  instance->beginAsync();
  runner->notify();
  runner->runOneStep();
  instance->stopAsync();
}

TEST(MPU6050Sensor_onReadTaskNotification, ReturnsEarlyWhenDoReadFlagIsFalse) {
  auto deps = buildDefaultDependencies();
  auto *runner =
      static_cast<DeterministicNotificationRunner *>(deps.runner.get());
  auto *sensor = deps.sensor.get();
  auto *pipe = deps.pipe.get();
  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_CALL(*sensor, getFIFOCount()).Times(0);
  EXPECT_CALL(*sensor, lastError()).Times(0);
  EXPECT_CALL(*sensor, readFIFO(_, _)).Times(0);
  EXPECT_CALL(*sensor, resetFIFO()).Times(0);
  EXPECT_CALL(*pipe, push(_)).Times(0);

  instance->stopAsync();
  runner->notify();
  runner->runOneStep();
}

} // namespace