#include <BLE.hpp>

// InitializingInstace: temporary pointer used while a BLE instance is being
// created and the static `instance` unique_ptr has not yet been assigned. Some
// NimBLE callbacks (e.g. sync) may be invoked during initialization. Use this
// to route those early callbacks to the creating object to avoid null crashes.
template <typename TransmittedSample>
typename BLE<TransmittedSample>::InstanceState
    BLE<TransmittedSample>::instanceState = {};

template <typename TransmittedSample>
BLE<TransmittedSample>::BLE(
    LoggerPort *logger,
    std::shared_ptr<Pipe<TransmittedSample, TRANSMISSION_PIPE_SIZE>> pipe,
    std::unique_ptr<LoopRunner> transmitRunner, TaskHandle_t coordinator)
    : communicationState{}, logger(logger), imuCharacteristics{}, services{},
      bleTaskHandle(nullptr), transmitRunner(std::move(transmitRunner)),
      coordinator(coordinator), pipe(pipe), primaryAdvertisingPacket{},
      scanResponsePacket{}, advertisingConfig{} {

  this->communicationState.mux = portMUX_INITIALIZER_UNLOCKED;
  this->communicationState.connectionHandle = BLE_HS_CONN_HANDLE_NONE;
  this->communicationState.mtu = BLE<TransmittedSample>::PREFERRED_MTU;
  this->communicationState.currentBatchSize =
      BLE<TransmittedSample>::PREFERRED_BATCH_SEND_SIZE;
  this->communicationState.isSubscribedToTransmittedSampleCharacteristic =
      false;
  this->communicationState.transmittedSampleCharacteristicHandle = 0;
}

template <typename TransmittedSample>
std::unique_ptr<BLE<TransmittedSample>> BLE<TransmittedSample>::create(
    LoggerPort *logger,
    std::shared_ptr<Pipe<TransmittedSample, TRANSMISSION_PIPE_SIZE>> pipe,
    std::unique_ptr<LoopRunner> transmitRunner, TaskHandle_t coordinator) {

  if (!logger) {
    return nullptr;
  }

  if (!pipe) {
    logger->error("BLE", "Invalid pipe provided to BLE instance");
    return nullptr;
  }

  if (!transmitRunner) {
    logger->error("BLE", "Invalid loop runner provided to BLE instance");
    return nullptr;
  }

  std::unique_ptr<BLE<TransmittedSample>> ble(
      new (std::nothrow) BLE<TransmittedSample>(
          logger, pipe, std::move(transmitRunner), coordinator));

  if (!ble) {
    logger->error("BLE", "Failed to allocate memory for BLE instance");
    return nullptr;
  }

  // Make early callbacks target this object while we initialize the host
  // stack. Clear on all exit paths to avoid leaving a dangling pointer.
  BLE<TransmittedSample>::instanceState.initializingInstance = ble.get();

  if (!ble->initializeFlash()) {
    BLE<TransmittedSample>::instanceState.initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeControllerAndHost()) {
    BLE<TransmittedSample>::instanceState.initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeGAP()) {
    BLE<TransmittedSample>::instanceState.initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeGATT()) {
    BLE<TransmittedSample>::instanceState.initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeTasks()) {
    BLE<TransmittedSample>::instanceState.initializingInstance = nullptr;
    return nullptr;
  }

  // clear the temporary pointer when finished
  BLE<TransmittedSample>::instanceState.initializingInstance = nullptr;

  return ble;
}

template <typename TransmittedSample>
BLE<TransmittedSample> *BLE<TransmittedSample>::getInstance(
    LoggerPort *logger,
    std::shared_ptr<Pipe<TransmittedSample, TRANSMISSION_PIPE_SIZE>> pipe,
    std::unique_ptr<LoopRunner> transmitRunner, TaskHandle_t coordinator) {

  // Protect instance creation with a mutex to ensure thread safety
  if (BLE<TransmittedSample>::instanceState.semaphoreHandle == nullptr) {
    rtosTaskEnterCritical(&BLE<TransmittedSample>::instanceState.mux);
    BLE<TransmittedSample>::instanceState.semaphoreHandle =
        rtosCreateMutexStatic(
            &BLE<TransmittedSample>::instanceState.semaphoreControlBlock);
    if (BLE<TransmittedSample>::instanceState.semaphoreHandle == nullptr) {
      logger->error("Failed to create mutex for BLE instance");
      rtosTaskExitCritical(&BLE<TransmittedSample>::instanceState.mux);
      return nullptr;
    }
    rtosTaskExitCritical(&BLE<TransmittedSample>::instanceState.mux);
  }

  if (rtosSemaphoreTake(BLE<TransmittedSample>::instanceState.semaphoreHandle,
                        portMAX_DELAY) == pdTRUE) {
    if (!BLE<TransmittedSample>::instanceState.instance)
      BLE<TransmittedSample>::instanceState.instance =
          BLE<TransmittedSample>::create(
              logger, pipe, std::move(transmitRunner), coordinator);
    rtosSemaphoreGive(BLE<TransmittedSample>::instanceState.semaphoreHandle);
  }
  return BLE<TransmittedSample>::instanceState.instance.get();
}

template <typename TransmittedSample> BLE<TransmittedSample>::~BLE() {
  this->setState(BLE<TransmittedSample>::State::IDLE);

  if (this->transmitRunner)
    this->transmitRunner->stop();

  nimblePortStop();

  if (this->bleTaskHandle)
    rtosTaskDelete(this->bleTaskHandle);
}

template <typename TransmittedSample>
bool BLE<TransmittedSample>::initializeFlash() {
  this->logger->debug("Initializing NVS flash");

  // NVS: non-volatile storage
  esp_err_t error = nvsFlashInit();
  if (error == ESP_OK)
    return true;

  if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
      error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvsFlashErase());
    RETURN_FALSE_ON_ESP_ERROR(nvsFlashInit(), this->logger,
                              "Failed to initialize NVS flash");
    return true;
  }

  this->logger->error("Failed to initialize NVS flash");
  return false;
}

template <typename TransmittedSample>
bool BLE<TransmittedSample>::initializeControllerAndHost() {
  this->logger->debug("Initializing controller and NimBLE host stack");

  // Initialize both at once
  RETURN_FALSE_ON_NIMBLE_ERROR(nimblePortInit(), this->logger,
                               "Failed to initialize NimBLE port");

  // Set callbacks for application logic
  ble_hs_cfg.reset_cb = BLE<TransmittedSample>::onStackReset;
  ble_hs_cfg.sync_cb = BLE<TransmittedSample>::onStackSync;
  ble_hs_cfg.gatts_register_cb = BLE<TransmittedSample>::onGATTRegister;
  ble_hs_cfg.gatts_register_arg = this;

  // Persist bond information and keys in NVS flash with these callbacks
  // Note: ble_store_config_init() doesn't compile
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  ble_hs_cfg.store_read_cb = ble_store_config_read;
  ble_hs_cfg.store_write_cb = ble_store_config_write;
  ble_hs_cfg.store_delete_cb = ble_store_config_delete;

  // Set larger MTU for batch sending IMU samples
  nimbleSetPreferredMtu(BLE<TransmittedSample>::PREFERRED_MTU);

  return true;
}

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)
template <typename TransmittedSample>
void BLE<TransmittedSample>::resetInstanceForTests() {
  BLE<TransmittedSample>::instanceState.instance.reset();
  BLE<TransmittedSample>::instanceState.initializingInstance = nullptr;
  BLE<TransmittedSample>::instanceState.semaphoreHandle = nullptr;
  BLE<TransmittedSample>::instanceState.semaphoreControlBlock = {};
  BLE<TransmittedSample>::instanceState.mux = {};
}
#endif

template <typename TransmittedSample>
void BLE<TransmittedSample>::onStackReset(int reason) {
  BLE<TransmittedSample> *self = nullptr;
  if (BLE<TransmittedSample>::instanceState.instance)
    self = BLE<TransmittedSample>::instanceState.instance.get();
  else if (BLE<TransmittedSample>::instanceState.initializingInstance)
    self = BLE<TransmittedSample>::instanceState.initializingInstance;

  if (self == nullptr)
    return;

  self->logger->info("NimBLE stack reset, reason: %d", reason);
}
template <typename TransmittedSample>
void BLE<TransmittedSample>::onStackSync() {
  BLE<TransmittedSample> *self = nullptr;
  if (BLE<TransmittedSample>::instanceState.instance)
    self = BLE<TransmittedSample>::instanceState.instance.get();
  else if (BLE<TransmittedSample>::instanceState.initializingInstance)
    self = BLE<TransmittedSample>::instanceState.initializingInstance;

  if (self == nullptr)
    return;

  if (self->initializeAdvertising())
    self->startAdvertising();
}
template <typename TransmittedSample>
void BLE<TransmittedSample>::onGATTRegister(
    struct ble_gatt_register_ctxt *context, void *arg) {
  BLE<TransmittedSample> *self = static_cast<BLE<TransmittedSample> *>(arg);
  char buffer[128];

  switch (context->op) {
  // Service register event
  case BLE_GATT_REGISTER_OP_SVC:
    self->logger->debug("Registered service %s with handle: %d",
                        nimbleUuidToStr(context->svc.svc_def->uuid, buffer),
                        context->svc.handle);
    break;
  // Characteristic register event
  case BLE_GATT_REGISTER_OP_CHR:
    self->logger->debug(
        "Registered characteristic %s with def_handle: %d val_handle: %d",
        nimbleUuidToStr(context->chr.chr_def->uuid, buffer),
        context->chr.def_handle, context->chr.val_handle);
    break;

  // Descriptor register event
  case BLE_GATT_REGISTER_OP_DSC:
    break;

  // Unknown event
  default:
    self->logger->error("Unknown GATT register event: %d", context->op);
    break;
  }
}

template <typename TransmittedSample>
bool BLE<TransmittedSample>::initializeGAP() {
  this->logger->debug("Initializing GAP service");

  nimbleGapServiceInit();

  RETURN_FALSE_ON_NIMBLE_ERROR(
      nimbleGapDeviceNameSet(BLE<TransmittedSample>::DEVICE_NAME), this->logger,
      "Failed to set device name");

  // Set how the device advertises itself to other devices
  RETURN_FALSE_ON_NIMBLE_ERROR(
      nimbleGapDeviceAppearanceSet(BLE<TransmittedSample>::APPEARANCE),
      this->logger, "Failed to set device appearance");

  return true;
}
// portENTER_CRITICAL() / portEXIT_CRITICAL():
// disable/enable interrupts to ensure atomic execution of critical sections.
// Used here to protect access to communicationState that are written here and
// read on transmitLoopFunction(). Reference:
// https://freertos.org/Documentation/02-Kernel/04-API-references/04-RTOS-kernel-control/01-taskENTER_CRITICAL_taskEXIT_CRITICAL
template <typename TransmittedSample>
int BLE<TransmittedSample>::handleGAPEvent(ble_gap_event *event, void *arg) {
  BLE<TransmittedSample> *self = static_cast<BLE<TransmittedSample> *>(arg);

  self->logger->debug("Received GAP event: %d", event->type);

  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT: {
    if (event->connect.status == 0) {
      // Allow only one connection at a time (e.g. phone or laptop)
      rtosPortEnterCritical(&self->communicationState.mux);
      self->communicationState.connectionHandle = event->connect.conn_handle;
      rtosPortExitCritical(&self->communicationState.mux);
      self->logger->info("Connection established with handle: %d",
                         self->communicationState.connectionHandle);

      RETURN_FALSE_ON_NIMBLE_ERROR(
          nimbleGattClientExchangeMtu(
              self->communicationState.connectionHandle),
          self->logger,
          "Failed to negotiate MTU with client, using default (23 bytes)");

      struct ble_gap_upd_params connectionParameters = {
          .itvl_min = static_cast<uint16_t>(
              BLE<TransmittedSample>::CONNECTION_INTERVAL_MIN_MS / 1.25),
          .itvl_max = static_cast<uint16_t>(
              BLE<TransmittedSample>::CONNECTION_INTERVAL_MAX_MS / 1.25),
          .latency = BLE<TransmittedSample>::CONNECTION_PERIPHERAL_LATENCY,
          .supervision_timeout =
              BLE<TransmittedSample>::CONNECTION_SUPERVISION_TIMEOUT_MS / 10,
          //  Duration of connection events
          .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
          .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN};

      self->logger->debug("Requesting connection parameters update: "
                          "itvl_min=%u, itvl_max=%u, "
                          "latency=%u, supervision_timeout=%u",
                          connectionParameters.itvl_min,
                          connectionParameters.itvl_max,
                          connectionParameters.latency,
                          connectionParameters.supervision_timeout);

      int connectionParametersError = nimbleGapUpdateParams(
          self->communicationState.connectionHandle, &connectionParameters);
      if (connectionParametersError != 0) {
        self->logger->warn(
            "Failed to request connection parameters update, status: %d",
            connectionParametersError);
      }

    } else {
      self->logger->error("Connection failed with error code: %d",
                          event->connect.status);
      self->startAdvertising();
    }
    break;
  }
  case BLE_GAP_EVENT_SUBSCRIBE: {
    if (event->subscribe.attr_handle ==
        self->communicationState.transmittedSampleCharacteristicHandle) {
      if (event->subscribe.cur_notify) {
        rtosPortEnterCritical(&self->communicationState.mux);
        self->communicationState.isSubscribedToTransmittedSampleCharacteristic =
            true;
        rtosPortExitCritical(&self->communicationState.mux);
        self->logger->info("Client subscribed to IMU Sample Characteristic");
      } else {
        rtosPortEnterCritical(&self->communicationState.mux);
        self->communicationState.isSubscribedToTransmittedSampleCharacteristic =
            false;
        rtosPortExitCritical(&self->communicationState.mux);
        self->logger->info(
            "Client unsubscribed from IMU Sample Characteristic");
      }
    }
    break;
  }
  case BLE_GAP_EVENT_DISCONNECT: {
    self->logger->info("Connection terminated, reason: %d",
                       event->disconnect.reason);
    rtosPortEnterCritical(&self->communicationState.mux);
    self->communicationState.connectionHandle = BLE_HS_CONN_HANDLE_NONE;
    self->communicationState.isSubscribedToTransmittedSampleCharacteristic =
        false;
    rtosPortExitCritical(&self->communicationState.mux);
    self->startAdvertising();
    break;
  }
  case BLE_GAP_EVENT_NOTIFY_TX: {
    if ((event->notify_tx.status != 0) &&
        (event->notify_tx.status != BLE_HS_EDONE)) {
      self->logger->error("Notification transmission failed, status: %d",
                          event->notify_tx.status);
    }
    break;
  }
  case BLE_GAP_EVENT_MTU: {
    rtosPortEnterCritical(&self->communicationState.mux);
    self->communicationState.mtu = event->mtu.value;
    self->communicationState.currentBatchSize = std::min(
        BLE<TransmittedSample>::PREFERRED_BATCH_SEND_SIZE,
        std::max(1, static_cast<int>((self->communicationState.mtu - 3) /
                                     sizeof(TransmittedSample))));
    rtosPortExitCritical(&self->communicationState.mux);
    self->logger->info(
        "Negotiated MTU: %d bytes. Current batch size: %d packets",
        self->communicationState.mtu,
        self->communicationState.currentBatchSize);
    break;
  }
  }

  return 0;
}

// GATT: Generic Attribute Profile
template <typename TransmittedSample>
bool BLE<TransmittedSample>::initializeGATT() {
  this->logger->debug("Initializing GATT service");

  nimbleGattServiceInit();

  imuCharacteristics[0].uuid =
      &BLE<TransmittedSample>::IMU_SAMPLE_CHARACTERISTIC_UUID.u;
  imuCharacteristics[0].access_cb =
      BLE<TransmittedSample>::accessTransmittedSampleCharacteristic;
  imuCharacteristics[0].arg = this;
  imuCharacteristics[0].flags = BLE_GATT_CHR_F_NOTIFY;
  imuCharacteristics[0].val_handle =
      &this->communicationState.transmittedSampleCharacteristicHandle;
  imuCharacteristics[0].min_key_size = 0; // No encryption required

  services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
  services[0].uuid = &BLE<TransmittedSample>::IMU_SERVICE_UUID.u;
  services[0].characteristics = this->imuCharacteristics;

  RETURN_FALSE_ON_NIMBLE_ERROR(nimbleGattServerCountConfig(this->services),
                               this->logger,
                               "Failed to count GATT configuration");

  RETURN_FALSE_ON_NIMBLE_ERROR(nimbleGattServerAddServices(this->services),
                               this->logger, "Failed to add GATT services");

  return true;
}
// Clients will only listen for notifications, not pull data from the server
template <typename TransmittedSample>
int BLE<TransmittedSample>::accessTransmittedSampleCharacteristic(
    uint16_t connectionHandle, uint16_t attributeHandle,
    struct ble_gatt_access_ctxt *context, void *arg) {
  return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

template <typename TransmittedSample>
bool BLE<TransmittedSample>::initializeTasks() {

  this->logger->debug("Initializing BLE tasks");

  if (!this->transmitRunner->start(BLE<TransmittedSample>::transmitLoopFunction,
                                   static_cast<void *>(this))) {
    this->logger->error("Failed to create transmit task");
    return false;
  }

  BaseType_t bleTaskResult = rtosTaskCreate(
      BLE<TransmittedSample>::bleTask, "bleTask", BLE_TASK_STACK_SIZE, this,
      BLE_TASK_PRIORITY, &this->bleTaskHandle);
  if (bleTaskResult != pdPASS) {
    this->logger->error("Failed to create BLE task");
    this->transmitRunner->stop();
    return false;
  }

  return true;
}

template <typename TransmittedSample>
void BLE<TransmittedSample>::bleTask(void *arg) {
  BLE<TransmittedSample> *self = static_cast<BLE<TransmittedSample> *>(arg);

  self->logger->debug("BLE task started");

  nimblePortRun();
}

template <typename TransmittedSample>
void BLE<TransmittedSample>::transmitLoopFunction(void *arg) {
  BLE<TransmittedSample> *self = static_cast<BLE<TransmittedSample> *>(arg);
  if (!self)
    return;

  if (self->getState() == BLE<TransmittedSample>::State::IDLE)
    return;

  TransmittedSample
      batchSamples[BLE<TransmittedSample>::PREFERRED_BATCH_SEND_SIZE];
  std::optional<TransmittedSample> optionalBatchSample;
  uint8_t batchCount;

  uint16_t characteristicHandle, connectionHandle, currentBatchSize;
  bool isSubscribed;

  // Sincronize access to shared variables
  rtosPortEnterCritical(&self->communicationState.mux);
  characteristicHandle =
      self->communicationState.transmittedSampleCharacteristicHandle;
  connectionHandle = self->communicationState.connectionHandle;
  isSubscribed =
      self->communicationState.isSubscribedToTransmittedSampleCharacteristic;
  currentBatchSize = self->communicationState.currentBatchSize;
  rtosPortExitCritical(&self->communicationState.mux);

  bool isValidCharacteristic = characteristicHandle != 0;
  if (!isValidCharacteristic)
    self->logger->warn(
        "Can't send data: IMU Sample Characteristic handle is invalid");
  bool isValidConnection = connectionHandle != BLE_HS_CONN_HANDLE_NONE;
  if (!isValidConnection)
    self->logger->warn("Can't send data: no connected client");
  if (!isSubscribed)
    self->logger->warn("Can't send data: client not subscribed to IMU sample "
                       "Characteristic");

  bool canTransmit = isValidCharacteristic && isValidConnection && isSubscribed;
  if (canTransmit) {
    while ((self->getState() == BLE<TransmittedSample>::State::TRANSMITTING &&
            self->pipe->itemsFilled() >= currentBatchSize) ||
           (self->getState() == BLE<TransmittedSample>::State::DRAINING &&
            self->pipe->itemsFilled() > 0)) {
      batchCount = 0;
      while (batchCount < currentBatchSize) {
        optionalBatchSample = self->pipe->pop(false);
        if (!optionalBatchSample.has_value())
          break;
        batchSamples[batchCount] = optionalBatchSample.value();
        batchCount++;
      }

      struct os_mbuf *nimbleBuffer = nimbleMbufFromFlat(
          batchSamples, batchCount * sizeof(TransmittedSample));

      if (nimbleBuffer == NULL) {
        self->logger->error("Can't send data: no memory available for nimble "
                            "buffer");
        return;
      }
      // Broadcast notification with data to all connected clients. The
      // buffer's memory is freed by nimble (no os_mbuf_free_chain() needed).
      // Note: ble_gatts_notify() only notifies the client to later pull the
      // data from the server, which I think is less efficient and more
      // complicated than pushing directly
      if (nimbleGattServerNotifyCustom(connectionHandle, characteristicHandle,
                                       nimbleBuffer) != 0) {
        self->logger->error("Failed to send IMU samples notification");
        return;
      }

      self->logger->debug("Sent notification with %d IMU samples", batchCount);
    }
  }

  // Notify that transmission of remaining samples has finished (previous loop
  // should make self->pipe->itemsFilled() == 0 with a valid connection),
  // provided that processing/sampling tasks were stopped by the coordinator. If
  // the connection is lost and there are still items in the pipe, simply
  // discard them.
  if (self->getState() == BLE<TransmittedSample>::State::DRAINING) {
    self->logger->debug("Notifying coordinator that transmission has finished");
    if (!canTransmit && self->pipe->itemsFilled() > 0) {
      self->logger->warn("Connection was lost, discarding %d remaining samples",
                         self->pipe->itemsFilled());
      self->pipe->empty();
    }
    self->setState(BLE<TransmittedSample>::State::IDLE);
    rtosTaskNotify(self->coordinator, BLE::STOPPED_MASK, eSetBits);
  }
}

template <typename TransmittedSample>
bool BLE<TransmittedSample>::initializeAdvertising() {
  this->logger->debug("Initializing advertising");

  // Infer address type: public/private, static/random
  RETURN_FALSE_ON_NIMBLE_ERROR(
      nimbleInferAutoAddressType(&this->communicationState.address.type),
      this->logger, "Failed to determine address type");

  RETURN_FALSE_ON_NIMBLE_ERROR(
      nimbleCopyAddress(this->communicationState.address.type,
                        this->communicationState.address.value, nullptr),
      this->logger, "Failed to read device address");

  sprintf(this->communicationState.address.readableValue,
          "0x%02X:%02X:%02X:%02X:%02X:%02X",
          this->communicationState.address.value[0],
          this->communicationState.address.value[1],
          this->communicationState.address.value[2],
          this->communicationState.address.value[3],
          this->communicationState.address.value[4],
          this->communicationState.address.value[5]);
  this->logger->info("Device BLE address: %s",
                     this->communicationState.address.readableValue);

  // Primary advertising packet (31 bytes max):
  // Flags (3) + Name (10) + 128-bit UUID (18) = 31 bytes exactly

  // General discoverable, LE-only
  this->primaryAdvertisingPacket.flags =
      BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  this->primaryAdvertisingPacket.name =
      (uint8_t *)BLE<TransmittedSample>::DEVICE_NAME;
  this->primaryAdvertisingPacket.name_len =
      strlen(BLE<TransmittedSample>::DEVICE_NAME);
  this->primaryAdvertisingPacket.name_is_complete = 1;

  // Advertise the IMU service UUID so apps can filter/discover by service
  this->primaryAdvertisingPacket.uuids128 =
      &BLE<TransmittedSample>::IMU_SERVICE_UUID;
  this->primaryAdvertisingPacket.num_uuids128 = 1;
  this->primaryAdvertisingPacket.uuids128_is_complete = 1;

  // Scan response packet (31 bytes max):
  // TX power (3) + Appearance (4) + Address (8) + Adv interval (4) = 19 bytes

  // Antenna transmission power
  this->scanResponsePacket.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  this->scanResponsePacket.tx_pwr_lvl_is_present = 1;

  // Appearance
  this->scanResponsePacket.appearance = BLE<TransmittedSample>::APPEARANCE;
  this->scanResponsePacket.appearance_is_present = 1;

  // Address
  this->scanResponsePacket.device_addr = this->communicationState.address.value;
  this->scanResponsePacket.device_addr_type =
      this->communicationState.address.type;
  this->scanResponsePacket.device_addr_is_present = 1;

  // Advertising interval
  this->scanResponsePacket.adv_itvl =
      BLE_GAP_ADV_ITVL_MS(BLE<TransmittedSample>::ADVERTISING_INTERVAL_MS);
  this->scanResponsePacket.adv_itvl_is_present = 1;
  this->advertisingConfig.itvl_min =
      BLE_GAP_ADV_ITVL_MS(BLE<TransmittedSample>::ADVERTISING_INTERVAL_MS);
  this->advertisingConfig.itvl_max =
      BLE_GAP_ADV_ITVL_MS(BLE<TransmittedSample>::ADVERTISING_INTERVAL_MS);

  // Behavior:
  // Connection mode: undirected connectable (accept connections from any
  // device)
  // Discovery mode: general discoverable
  this->advertisingConfig.conn_mode = BLE_GAP_CONN_MODE_UND;
  this->advertisingConfig.disc_mode = BLE_GAP_DISC_MODE_GEN;

  RETURN_FALSE_ON_NIMBLE_ERROR(
      nimbleGapAdvertisingSetFields(&this->primaryAdvertisingPacket),
      this->logger, "Failed to set primary advertising packet fields");

  RETURN_FALSE_ON_NIMBLE_ERROR(
      nimbleGapAdvertisingResponseSetFields(&this->scanResponsePacket),
      this->logger, "Failed to set scan response packet fields");

  return true;
}
template <typename TransmittedSample>
bool BLE<TransmittedSample>::startAdvertising() {
  RETURN_FALSE_ON_NIMBLE_ERROR(
      nimbleGapAdvertisingStart(this->communicationState.address.type,
                                &this->advertisingConfig,
                                BLE<TransmittedSample>::handleGAPEvent, this),
      this->logger, "Failed to start advertising");

  return true;
}

template <typename TransmittedSample>
bool BLE<TransmittedSample>::isConnected() {
  uint16_t connectionHandle;
  rtosPortEnterCritical(&this->communicationState.mux);
  connectionHandle = this->communicationState.connectionHandle;
  rtosPortExitCritical(&this->communicationState.mux);
  return connectionHandle != BLE_HS_CONN_HANDLE_NONE;
}

template <typename TransmittedSample>
void BLE<TransmittedSample>::beginTransmission() {
  this->logger->info("Starting BLE transmission");
  if (this->getState() == BLE<TransmittedSample>::State::IDLE)
    this->setState(BLE<TransmittedSample>::State::TRANSMITTING);
}
template <typename TransmittedSample>
void BLE<TransmittedSample>::requestStopTransmission() {
  this->logger->info("Stopping BLE transmission");
  if (this->getState() == BLE<TransmittedSample>::State::TRANSMITTING)
    this->setState(BLE<TransmittedSample>::State::DRAINING);
}

template <typename TransmittedSample>
void BLE<TransmittedSample>::setState(BLE<TransmittedSample>::State value) {
  this->state.store(value, std::memory_order_relaxed);
}
template <typename TransmittedSample>
BLE<TransmittedSample>::State BLE<TransmittedSample>::getState() {
  return this->state.load(std::memory_order_relaxed);
}

template class BLE<IMUSample>;
template class BLE<ProcessedSample>;