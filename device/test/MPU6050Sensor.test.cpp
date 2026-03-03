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

TEST(MPU6050Sensor, GetInstanceInitializesSuccessfullyAndCallsAllDrivers) {
  using testing::AtLeast;
  using testing::Return;

  resetFreeRTOSPortFakes();
  resetGPIOPortFakes();

  rtosCreateMutexStatic_fake.return_val =
      reinterpret_cast<SemaphoreHandle_t>(0x1);
  rtosSemaphoreTake_fake.return_val = pdTRUE;
  rtosSemaphoreGive_fake.return_val = pdTRUE;

  gpioSetConfig_fake.return_val = ESP_OK;
  gpioAddISRHandler_fake.return_val = ESP_OK;

  auto pipe = std::make_shared<testing::NiceMock<PipeDouble>>();
  auto sensor = std::make_unique<testing::NiceMock<MPUDouble>>();
  auto i2c = std::make_unique<testing::NiceMock<I2CDouble>>();
  auto runner = std::make_unique<DeterministicRunner>();
  auto *runnerRaw = runner.get();

  EXPECT_CALL(*i2c,
              begin(testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));

  EXPECT_CALL(*sensor, setBus(testing::_)).Times(AtLeast(1));
  EXPECT_CALL(*sensor, setAddr(testing::_)).Times(AtLeast(1));
  EXPECT_CALL(*sensor, lastError())
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));

  EXPECT_CALL(*sensor, reset())
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));
  EXPECT_CALL(*sensor, testConnection())
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));
  EXPECT_CALL(*sensor, initialize())
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));

  EXPECT_CALL(*sensor, setSampleRate(testing::_))
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));
  EXPECT_CALL(*sensor, setAccelFullScale(testing::_))
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));
  EXPECT_CALL(*sensor, setGyroFullScale(testing::_))
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));

  EXPECT_CALL(*sensor, setFIFOConfig(testing::_))
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));
  EXPECT_CALL(*sensor, setFIFOEnabled(testing::_))
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));

  EXPECT_CALL(*sensor, setInterruptConfig(testing::_))
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));
  EXPECT_CALL(*sensor, setInterruptEnabled(testing::_))
      .Times(AtLeast(1))
      .WillRepeatedly(Return(ESP_OK));

  MPU6050Sensor *instance = MPU6050Sensor::getInstance(
      &gLogger, pipe, std::move(sensor), std::move(i2c), std::move(runner),
      GPIO_NUM_5, GPIO_NUM_6, GPIO_NUM_7, 30);

  EXPECT_NE(instance, nullptr);
  EXPECT_GT(rtosTaskDelay_fake.call_count, 0);

  EXPECT_GT(runnerRaw->getStartCallCount(), 0);

  EXPECT_GT(gpioSetConfig_fake.call_count, 0);
  EXPECT_GT(gpioAddISRHandler_fake.call_count, 0);

  EXPECT_GT(rtosTaskEnterCritical_fake.call_count, 0);
  EXPECT_GT(rtosTaskExitCritical_fake.call_count, 0);
  EXPECT_GT(rtosCreateMutexStatic_fake.call_count, 0);
  EXPECT_GT(rtosSemaphoreTake_fake.call_count, 0);
  EXPECT_GT(rtosSemaphoreGive_fake.call_count, 0);
}
