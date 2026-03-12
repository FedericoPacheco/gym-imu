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
using ::testing::Return;

constexpr uint16_t VALID_IMU_CHARACTERISTIC_HANDLE = 0x0042;
constexpr uint16_t TEST_CONNECTION_HANDLE = 0x0123;
constexpr uint16_t TEST_BATCH_SIZE = 2;

LoggerDouble logger;
os_mbuf nimbleBuffer = {0};

// ----------------------------------------------------------------------------------------------------
// Helpers

class StartFailLoopRunner final : public DeterministicLoopRunner {
public:
  bool start(std::function<void(void *)>, void *) override { return false; }
};

int assignCharacteristicHandleOnAddServices(struct ble_gatt_svc_def *services) {
  if (services == nullptr || services[0].characteristics == nullptr ||
      services[0].characteristics[0].val_handle == nullptr)
    return -1;

  *(services[0].characteristics[0].val_handle) =
      VALID_IMU_CHARACTERISTIC_HANDLE;

  return 0;
}

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
  nimbleGattServerAddServices_fake.custom_fake =
      assignCharacteristicHandleOnAddServices;
  nimbleInferAutoAddressType_fake.return_val = 0;
  nimbleCopyAddress_fake.return_val = 0;
  nimbleGapAdvertisingSetFields_fake.return_val = 0;
  nimbleGapAdvertisingResponseSetFields_fake.return_val = 0;
  nimbleGapAdvertisingStart_fake.return_val = 0;
  nimbleGattClientExchangeMtu_fake.return_val = 0;
  nimbleGapUpdateParams_fake.return_val = 0;
  nimbleMbufFromFlat_fake.return_val = &nimbleBuffer;
  nimbleGattServerNotifyCustom_fake.return_val = 0;

  BLEDependencies deps;
  deps.logger = &logger;
  deps.pipe = std::make_shared<NiceMock<PipeDouble>>();
  deps.transmitRunner = std::make_unique<DeterministicLoopRunner>();

  return deps;
}

BLE *getInstanceWith(BLEDependencies &&deps) {
  return BLE::getInstance(deps.logger, deps.pipe,
                          std::move(deps.transmitRunner));
}

// Drive BLE internal behavior through events

void runStackSyncCallback() {
  ASSERT_NE(ble_hs_cfg.sync_cb, nullptr);
  ble_hs_cfg.sync_cb();
}

void dispatchGAPEvent(const ble_gap_event &event) {
  ASSERT_GT(nimbleGapAdvertisingStart_fake.call_count, 0);
  ASSERT_NE(nimbleGapAdvertisingStart_fake.arg2_val, nullptr);

  ble_gap_event mutableEvent = event;
  nimbleGapAdvertisingStart_fake.arg2_val(
      &mutableEvent, nimbleGapAdvertisingStart_fake.arg3_val);
}

void connectClient() {
  ble_gap_event event = {};
  event.type = BLE_GAP_EVENT_CONNECT;
  event.connect.status = 0;
  event.connect.conn_handle = TEST_CONNECTION_HANDLE;
  dispatchGAPEvent(event);
}

void subscribeClientToImuCharacteristic() {
  ble_gap_event event = {};
  event.type = BLE_GAP_EVENT_SUBSCRIBE;
  event.subscribe.attr_handle = VALID_IMU_CHARACTERISTIC_HANDLE;
  event.subscribe.cur_notify = 1;
  dispatchGAPEvent(event);
}

void setBatchSizeToTestValue() {
  ble_gap_event event = {};
  event.type = BLE_GAP_EVENT_MTU;
  event.mtu.value =
      static_cast<uint16_t>(sizeof(IMUSample) * TEST_BATCH_SIZE + 3);
  dispatchGAPEvent(event);
}

// ----------------------------------------------------------------------------------------------------
// Initialization tests

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
  nimbleGattServerAddServices_fake.custom_fake = nullptr;
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

// ----------------------------------------------------------------------------------------------------
// Transmission tests

TEST(BLE_transmitLoopFunction,
     SendsNoNotificationWhenCharacteristicHandleIsInvalid) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());
  nimbleGattServerAddServices_fake.custom_fake = nullptr;
  nimbleGattServerAddServices_fake.return_val = 0;

  BLE *instance = getInstanceWith(std::move(deps));
  instance->beginTransmission();
  runnerRaw->runOneStep();
  instance->stopTransmission();

  EXPECT_EQ(nimbleGattServerNotifyCustom_fake.call_count, 0u);
}

TEST(BLE_transmitLoopFunction, SendsNoNotificationWhenClientIsNotConnected) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());

  BLE *instance = getInstanceWith(std::move(deps));
  runStackSyncCallback();
  subscribeClientToImuCharacteristic();
  instance->beginTransmission();
  runnerRaw->runOneStep();
  instance->stopTransmission();

  EXPECT_EQ(nimbleGattServerNotifyCustom_fake.call_count, 0u);
}

TEST(BLE_transmitLoopFunction, SendsNoNotificationWhenClientIsNotSubscribed) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());

  BLE *instance = getInstanceWith(std::move(deps));
  runStackSyncCallback();
  connectClient();
  instance->beginTransmission();
  runnerRaw->runOneStep();
  instance->stopTransmission();

  EXPECT_EQ(nimbleGattServerNotifyCustom_fake.call_count, 0u);
}

TEST(BLE_transmitLoopFunction,
     SendsNoNotificationWhenItemsAreBatchSizeMinusOne) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());
  auto *pipeRaw = deps.pipe.get();

  EXPECT_CALL(*pipeRaw, itemsFilled())
      .WillOnce(Return(static_cast<uint32_t>(TEST_BATCH_SIZE - 1)));

  BLE *instance = getInstanceWith(std::move(deps));
  runStackSyncCallback();
  connectClient();
  subscribeClientToImuCharacteristic();
  setBatchSizeToTestValue();
  instance->beginTransmission();
  runnerRaw->runOneStep();
  instance->stopTransmission();

  EXPECT_EQ(nimbleGattServerNotifyCustom_fake.call_count, 0u);
}

TEST(BLE_transmitLoopFunction, SendsOneNotificationWhenItemsEqualBatchSize) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());
  auto *pipeRaw = deps.pipe.get();

  EXPECT_CALL(*pipeRaw, itemsFilled())
      .WillOnce(Return(static_cast<uint32_t>(TEST_BATCH_SIZE)))
      .WillOnce(Return(0));
  EXPECT_CALL(*pipeRaw, pop(false))
      .WillRepeatedly(Return(std::optional<IMUSample>(IMUSample{})));

  BLE *instance = getInstanceWith(std::move(deps));
  runStackSyncCallback();
  connectClient();
  subscribeClientToImuCharacteristic();
  setBatchSizeToTestValue();
  instance->beginTransmission();
  runnerRaw->runOneStep();
  instance->stopTransmission();

  EXPECT_EQ(nimbleGattServerNotifyCustom_fake.call_count, 1u);
}

TEST(BLE_transmitLoopFunction,
     SendsOneNotificationWhenItemsAreBatchSizePlusOne) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());
  auto *pipeRaw = deps.pipe.get();

  EXPECT_CALL(*pipeRaw, itemsFilled())
      .WillOnce(Return(static_cast<uint32_t>(TEST_BATCH_SIZE + 1)))
      .WillOnce(Return(static_cast<uint32_t>(TEST_BATCH_SIZE - 1)));
  EXPECT_CALL(*pipeRaw, pop(false))
      .WillRepeatedly(Return(std::optional<IMUSample>(IMUSample{})));

  BLE *instance = getInstanceWith(std::move(deps));
  runStackSyncCallback();
  connectClient();
  subscribeClientToImuCharacteristic();
  setBatchSizeToTestValue();
  instance->beginTransmission();
  runnerRaw->runOneStep();
  instance->stopTransmission();

  EXPECT_EQ(nimbleGattServerNotifyCustom_fake.call_count, 1u);
}

TEST(BLE_transmitLoopFunction, SendsNoNotificationWhenBufferCreationFails) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());
  auto *pipeRaw = deps.pipe.get();
  nimbleMbufFromFlat_fake.return_val = nullptr;

  EXPECT_CALL(*pipeRaw, itemsFilled())
      .WillOnce(Return(static_cast<uint32_t>(TEST_BATCH_SIZE)));
  EXPECT_CALL(*pipeRaw, pop(false))
      .WillRepeatedly(Return(std::optional<IMUSample>(IMUSample{})));

  BLE *instance = getInstanceWith(std::move(deps));
  runStackSyncCallback();
  connectClient();
  subscribeClientToImuCharacteristic();
  setBatchSizeToTestValue();
  instance->beginTransmission();
  runnerRaw->runOneStep();
  instance->stopTransmission();

  EXPECT_EQ(nimbleGattServerNotifyCustom_fake.call_count, 0u);
}

TEST(BLE_transmitLoopFunction, SendsOneNotificationWhenNotificationFails) {
  auto deps = buildDefaultDependencies();
  auto *runnerRaw =
      static_cast<DeterministicLoopRunner *>(deps.transmitRunner.get());
  auto *pipeRaw = deps.pipe.get();
  nimbleGattServerNotifyCustom_fake.return_val = -1;

  EXPECT_CALL(*pipeRaw, itemsFilled())
      .WillOnce(Return(static_cast<uint32_t>(TEST_BATCH_SIZE)));
  EXPECT_CALL(*pipeRaw, pop(false))
      .WillRepeatedly(Return(std::optional<IMUSample>(IMUSample{})));

  BLE *instance = getInstanceWith(std::move(deps));
  runStackSyncCallback();
  connectClient();
  subscribeClientToImuCharacteristic();
  setBatchSizeToTestValue();
  instance->beginTransmission();
  runnerRaw->runOneStep();
  instance->stopTransmission();

  EXPECT_EQ(nimbleGattServerNotifyCustom_fake.call_count, 1u);
}

} // namespace
