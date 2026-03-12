#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "doubles/ESPIDFDouble.hpp"
#include "doubles/FreeRTOSDouble.hpp"
#include "doubles/LoggerDouble.hpp"
#include "doubles/NimBLEDouble.cpp"
#include "doubles/NimBLEDouble.hpp"
#include "doubles/PipeDouble.hpp"
#include <BLE.hpp>
#include <DeterministicLoopRunner.hpp>

namespace {

using ::testing::NiceMock;

LoggerDouble gLogger;

class StartFailLoopRunner final : public DeterministicLoopRunner {
public:
  bool start(std::function<void(void *)>, void *) override { return false; }
};

struct BLEDependencies {
  LoggerPort *logger;
  std::shared_ptr<NiceMock<PipeDouble>> pipe;
  std::unique_ptr<LoopRunner> transmitRunner;
};

BLEDependencies buildDefaultDependencies() {
  BLE::resetInstanceForTests();

  resetFreeRTOSPortFakes();
  rtosCreateMutexStatic_fake.return_val =
      reinterpret_cast<SemaphoreHandle_t>(0x1);
  rtosSemaphoreTake_fake.return_val = pdTRUE;
  rtosSemaphoreGive_fake.return_val = pdTRUE;
  rtosTaskCreate_fake.return_val = pdPASS;

  resetESPIDFPortFakes();
  nvsFlashInit_fake.return_val = ESP_OK;
  nvsFlashErase_fake.return_val = ESP_OK;

  resetNimBLEPortFakes();
  nimblePortInit_fake.return_val = 0;
  nimbleGapDeviceNameSet_fake.return_val = 0;
  nimbleGapDeviceAppearanceSet_fake.return_val = 0;
  nimbleGattServerCountConfig_fake.return_val = 0;
  nimbleGattServerAddServices_fake.return_val = 0;

  BLEDependencies deps;
  deps.logger = &gLogger;
  deps.pipe = std::make_shared<NiceMock<PipeDouble>>();
  deps.transmitRunner = std::make_unique<DeterministicLoopRunner>();

  return deps;
}

BLE *getInstanceWith(BLEDependencies &&deps) {
  return BLE::getInstance(deps.logger, deps.pipe,
                          std::move(deps.transmitRunner));
}

TEST(BLE_getInstance, ReturnsNullWhenLoggerIsNull) {
  auto deps = buildDefaultDependencies();
  deps.logger = nullptr;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenPipeIsNull) {
  auto deps = buildDefaultDependencies();
  deps.pipe = nullptr;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenTransmitRunnerIsNull) {
  auto deps = buildDefaultDependencies();
  deps.transmitRunner = nullptr;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenCreatingMutexFails) {
  auto deps = buildDefaultDependencies();
  rtosCreateMutexStatic_fake.return_val = nullptr;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenNVSFlashInitFails) {
  auto deps = buildDefaultDependencies();
  nvsFlashInit_fake.return_val = ESP_FAIL;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenNimBLEPortInitFails) {
  auto deps = buildDefaultDependencies();
  nimblePortInit_fake.return_val = -1;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenSettingDeviceNameFails) {
  auto deps = buildDefaultDependencies();
  nimbleGapDeviceNameSet_fake.return_val = -1;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenSettingDeviceAppearanceFails) {
  auto deps = buildDefaultDependencies();
  nimbleGapDeviceAppearanceSet_fake.return_val = -1;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenCountingGATTConfigFails) {
  auto deps = buildDefaultDependencies();
  nimbleGattServerCountConfig_fake.return_val = -1;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenAddingGATTServicesFails) {
  auto deps = buildDefaultDependencies();
  nimbleGattServerAddServices_fake.return_val = -1;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenTransmitRunnerStartFails) {
  auto deps = buildDefaultDependencies();
  deps.transmitRunner = std::make_unique<StartFailLoopRunner>();

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, ReturnsNullWhenCreatingBLETaskFails) {
  auto deps = buildDefaultDependencies();
  rtosTaskCreate_fake.return_val = pdFALSE;

  EXPECT_EQ(getInstanceWith(std::move(deps)), nullptr);
}

TEST(BLE_getInstance, InitializesSuccessfully) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());

  BLE *instance = getInstanceWith(std::move(deps));

  EXPECT_NE(instance, nullptr);
  EXPECT_GT(runnerRaw->getStartCallCount(), 0);
}

} // namespace
