#include <gmock/gmock.h>
#include <gtest/gtest.h>

extern "C" {
#include <fff.h>
}

DEFINE_FFF_GLOBALS;

#include "doubles/FreeRTOSDouble.cpp"
#include "doubles/FreeRTOSDouble.hpp"
#include "doubles/GPIODouble.cpp"
#include "doubles/GPIODouble.hpp"
#include "doubles/I2CDouble.hpp"
#include "doubles/LoggerDouble.hpp"
#include "doubles/MPUDouble.hpp"
#include "doubles/PipeDouble.hpp"
#include <DeterministicRunner.hpp>
#include <MPU6050Sensor.hpp>

LoggerDouble gLogger;

namespace {

using ::testing::_;
using ::testing::Return;

class StartFailRunner final : public DeterministicRunner {
public:
  bool start(std::function<void(void *, uint32_t)>, void *) override {
    return false;
  }
};
struct MPU6050SensorDependencies {
  std::shared_ptr<testing::NiceMock<PipeDouble>> pipe;
  std::unique_ptr<testing::NiceMock<MPUDouble>> sensor;
  std::unique_ptr<testing::NiceMock<I2CDouble>> i2c;
  std::unique_ptr<Runner> runner;
};
MPU6050SensorDependencies
buildDefaultDependencies(std::unique_ptr<Runner> runner) {
  resetFreeRTOSPortFakes();
  rtosCreateMutexStatic_fake.return_val =
      reinterpret_cast<SemaphoreHandle_t>(0x1);
  rtosSemaphoreTake_fake.return_val = pdTRUE;
  rtosSemaphoreGive_fake.return_val = pdTRUE;

  resetGPIOPortFakes();
  gpioSetConfig_fake.return_val = ESP_OK;
  gpioAddISRHandler_fake.return_val = ESP_OK;

  MPU6050SensorDependencies deps;
  deps.pipe = std::make_shared<testing::NiceMock<PipeDouble>>();
  deps.sensor = std::make_unique<testing::NiceMock<MPUDouble>>();
  deps.i2c = std::make_unique<testing::NiceMock<I2CDouble>>();
  deps.runner = std::move(runner);

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

  return deps;
}
MPU6050SensorDependencies buildDefaultDependencies() {
  return buildDefaultDependencies(std::make_unique<DeterministicRunner>());
}
MPU6050Sensor *getInstanceWith(MPU6050SensorDependencies &&deps) {
  return MPU6050Sensor::getInstance(&gLogger, deps.pipe, std::move(deps.sensor),
                                    std::move(deps.i2c),
                                    std::move(deps.runner));
}

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
  auto deps = buildDefaultDependencies(std::make_unique<StartFailRunner>());

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(MPU6050Sensor_getInstance, InitializesSuccessfully) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw = static_cast<DeterministicRunner *>(deps.runner.get());

  MPU6050Sensor *instance = getInstanceWith(std::move(deps));

  EXPECT_NE(instance, nullptr);
  EXPECT_GT(runnerRaw->getStartCallCount(), 0);
  EXPECT_GT(gpioSetConfig_fake.call_count, 0);
  EXPECT_GT(gpioAddISRHandler_fake.call_count, 0);
}

} // namespace