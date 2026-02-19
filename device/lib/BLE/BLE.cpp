#include <BLE.hpp>
#include <cstdint>

std::unique_ptr<BLE> BLE::instance = nullptr;
// Temporary pointer used while a BLE instance is being created and the static
// `instance` unique_ptr has not yet been assigned. Some NimBLE callbacks
// (e.g. sync) may be invoked during initialization. Use this to route those
// early callbacks to the creating object to avoid null crashes.
BLE *BLE::initializingInstance = nullptr;

BLE::BLE(Logger *logger,
         std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> pipe)
    : communicationState{}, logger(logger), imuCharacteristics{}, services{},
      bleTaskHandle(nullptr), transmitTaskHandle(nullptr), doTransmit(true),
      pipe(pipe), primaryAdvertisingPacket{}, scanResponsePacket{},
      advertisingConfig{} {

  this->communicationState.mux = portMUX_INITIALIZER_UNLOCKED;
  this->communicationState.connectionHandle = BLE_HS_CONN_HANDLE_NONE;
  this->communicationState.mtu = BLE::PREFERRED_MTU;
  this->communicationState.currentBatchSize = BLE::PREFERRED_BATCH_SEND_SIZE;
  this->communicationState.isSubscribedToImuSampleCharacteristic = false;
  this->communicationState.imuSampleCharacteristicHandle = 0;
}

std::unique_ptr<BLE>
BLE::create(Logger *logger,
            std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> pipe) {
  std::unique_ptr<BLE> ble(new (std::nothrow) BLE(logger, pipe));

  if (!ble) {
    if (logger)
      logger->error("BLE", "Failed to allocate memory for BLE instance");
    return nullptr;
  }

  if (!pipe) {
    if (logger)
      logger->error("BLE", "Invalid pipe provided to BLE instance");
    return nullptr;
  }

  // Make early callbacks target this object while we initialize the host
  // stack. Clear on all exit paths to avoid leaving a dangling pointer.
  BLE::initializingInstance = ble.get();

  if (!ble->initializeFlash()) {
    BLE::initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeControllerAndHost()) {
    BLE::initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeGAP()) {
    BLE::initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeGATT()) {
    BLE::initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeTasks()) {
    BLE::initializingInstance = nullptr;
    return nullptr;
  }

  // clear the temporary pointer when finished
  BLE::initializingInstance = nullptr;

  return ble;
}
BLE *BLE::getInstance(
    Logger *logger,
    std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> pipe) {
  if (!BLE::instance)
    BLE::instance = BLE::create(logger, pipe);
  return BLE::instance.get();
}

BLE::~BLE() {
  // Transmit remaining samples before shutting down
  if (this->transmitTaskHandle) {
    this->doTransmit = false;
    xTaskNotifyGive(this->transmitTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(BLE::TRANSMIT_TASK_SHUTDOWN_DELAY_MS));
    vTaskDelete(this->transmitTaskHandle);
  }

  nimble_port_stop();

  if (this->bleTaskHandle)
    vTaskDelete(this->bleTaskHandle);
}

bool BLE::initializeFlash() {
  this->logger->debug("Initializing NVS flash");

  // NVS: non-volatile storage
  esp_err_t error = nvs_flash_init();
  if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
      error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    RETURN_FALSE_ON_ERROR(nvs_flash_init(), this->logger,
                          "Failed to initialize NVS flash");
  }

  return true;
}

bool BLE::initializeControllerAndHost() {
  this->logger->debug("Initializing controller and NimBLE host stack");

  // Initialize both at once
  RETURN_FALSE_ON_NIMBLE_ERROR(nimble_port_init(), this->logger,
                               "Failed to initialize NimBLE port");

  // Set callbacks for application logic
  ble_hs_cfg.reset_cb = BLE::onStackReset;
  ble_hs_cfg.sync_cb = BLE::onStackSync;
  ble_hs_cfg.gatts_register_cb = BLE::onGATTRegister;
  ble_hs_cfg.gatts_register_arg = this;

  // Persist bond information and keys in NVS flash with these callbacks
  // Note: ble_store_config_init() doesn't compile
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  ble_hs_cfg.store_read_cb = ble_store_config_read;
  ble_hs_cfg.store_write_cb = ble_store_config_write;
  ble_hs_cfg.store_delete_cb = ble_store_config_delete;

  // Set larger MTU for batch sending IMU samples
  ble_att_set_preferred_mtu(BLE::PREFERRED_MTU);

  return true;
}
void BLE::onStackReset(int reason) {
  instance->logger->info("NimBLE stack reset, reason: %d", reason);
}
void BLE::onStackSync() {
  BLE *self = nullptr;
  if (BLE::instance)
    self = BLE::instance.get();
  else if (BLE::initializingInstance)
    self = BLE::initializingInstance;

  if (self == nullptr)
    return;

  if (self->initializeAdvertising())
    self->startAdvertising();
}
void BLE::onGATTRegister(struct ble_gatt_register_ctxt *context, void *arg) {
  BLE *self = static_cast<BLE *>(arg);
  char buffer[128];

  switch (context->op) {
  // Service register event
  case BLE_GATT_REGISTER_OP_SVC:
    self->logger->debug("Registered service %s with handle: %d",
                        ble_uuid_to_str(context->svc.svc_def->uuid, buffer),
                        context->svc.handle);
    break;
  // Characteristic register event
  case BLE_GATT_REGISTER_OP_CHR:
    self->logger->debug(
        "Registered characteristic %s with def_handle: %d val_handle: %d",
        ble_uuid_to_str(context->chr.chr_def->uuid, buffer),
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

bool BLE::initializeGAP() {
  this->logger->debug("Initializing GAP service");

  ble_svc_gap_init();

  RETURN_FALSE_ON_NIMBLE_ERROR(ble_svc_gap_device_name_set(BLE::DEVICE_NAME),
                               this->logger, "Failed to set device name");

  // Set how the device advertises itself to other devices
  RETURN_FALSE_ON_NIMBLE_ERROR(
      ble_svc_gap_device_appearance_set(BLE::APPEARANCE), this->logger,
      "Failed to set device appearance");

  return true;
}
// portENTER_CRITICAL() / portEXIT_CRITICAL():
// disable/enable interrupts to ensure atomic execution of critical sections.
// Used here to protect access to communicationState that are written here and
// read on transmitTask() and send() Reference:
// https://freertos.org/Documentation/02-Kernel/04-API-references/04-RTOS-kernel-control/01-taskENTER_CRITICAL_taskEXIT_CRITICAL
int BLE::handleGAPEvent(ble_gap_event *event, void *arg) {
  BLE *self = static_cast<BLE *>(arg);

  self->logger->debug("Received GAP event: %d", event->type);

  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT: {
    if (event->connect.status == 0) {
      // Allow only one connection at a time (e.g. phone or laptop)
      portENTER_CRITICAL(&self->communicationState.mux);
      self->communicationState.connectionHandle = event->connect.conn_handle;
      portEXIT_CRITICAL(&self->communicationState.mux);
      self->logger->info("Connection established");

      RETURN_FALSE_ON_NIMBLE_ERROR(
          ble_gattc_exchange_mtu(self->communicationState.connectionHandle,
                                 NULL, NULL),
          self->logger,
          "Failed to negotiate MTU with client, using default (23 bytes)");

      struct ble_gap_upd_params connectionParameters = {
          .itvl_min = BLE::CONNECTION_INTERVAL_MIN_MS,
          .itvl_max = BLE::CONNECTION_INTERVAL_MAX_MS,
          .latency = BLE::CONNECTION_PERIPHERAL_LATENCY,
          .supervision_timeout = BLE::CONNECTION_SUPERVISION_TIMEOUT_MS,
          //  Duration of connection events
          .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
          .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN};

      self->logger->debug(
          "Requesting connection parameters update: itvl_min=%u, itvl_max=%u, "
          "latency=%u, supervision_timeout=%u",
          connectionParameters.itvl_min, connectionParameters.itvl_max,
          connectionParameters.latency,
          connectionParameters.supervision_timeout);

      int connectionParametersError = ble_gap_update_params(
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
        self->communicationState.imuSampleCharacteristicHandle) {
      if (event->subscribe.cur_notify) {
        portENTER_CRITICAL(&self->communicationState.mux);
        self->communicationState.isSubscribedToImuSampleCharacteristic = true;
        portEXIT_CRITICAL(&self->communicationState.mux);
        self->logger->info("Client subscribed to IMU Sample Characteristic");
      } else {
        portENTER_CRITICAL(&self->communicationState.mux);
        self->communicationState.isSubscribedToImuSampleCharacteristic = false;
        portEXIT_CRITICAL(&self->communicationState.mux);
        self->logger->info(
            "Client unsubscribed from IMU Sample Characteristic");
      }
    }
    break;
  }
  case BLE_GAP_EVENT_DISCONNECT: {
    self->logger->info("Connection terminated, reason: %d",
                       event->disconnect.reason);
    portENTER_CRITICAL(&self->communicationState.mux);
    self->communicationState.connectionHandle = BLE_HS_CONN_HANDLE_NONE;
    self->communicationState.isSubscribedToImuSampleCharacteristic = false;
    portEXIT_CRITICAL(&self->communicationState.mux);
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
    portENTER_CRITICAL(&self->communicationState.mux);
    self->communicationState.mtu = event->mtu.value;
    self->communicationState.currentBatchSize =
        MIN(BLE::PREFERRED_BATCH_SEND_SIZE,
            MAX(1, (self->communicationState.mtu - 3) / sizeof(IMUSample)));
    portEXIT_CRITICAL(&self->communicationState.mux);
    self->logger->info("Negotiated MTU: %d. Current batch size: %d",
                       self->communicationState.mtu,
                       self->communicationState.currentBatchSize);
    break;
  }
  }

  return 0;
}

// GATT: Generic Attribute Profile
bool BLE::initializeGATT() {
  this->logger->debug("Initializing GATT service");

  ble_svc_gatt_init();

  imuCharacteristics[0].uuid = &BLE::IMU_SAMPLE_CHARACTERISTIC_UUID.u;
  imuCharacteristics[0].access_cb = BLE::accessImuSampleCharacteristic;
  imuCharacteristics[0].arg = this;
  imuCharacteristics[0].flags = BLE_GATT_CHR_F_NOTIFY;
  imuCharacteristics[0].val_handle =
      &this->communicationState.imuSampleCharacteristicHandle;
  imuCharacteristics[0].min_key_size = 0; // No encryption required

  services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
  services[0].uuid = &BLE::IMU_SERVICE_UUID.u;
  services[0].characteristics = this->imuCharacteristics;

  RETURN_FALSE_ON_NIMBLE_ERROR(ble_gatts_count_cfg(this->services),
                               this->logger,
                               "Failed to count GATT configuration");

  RETURN_FALSE_ON_NIMBLE_ERROR(ble_gatts_add_svcs(this->services), this->logger,
                               "Failed to add GATT services");

  return true;
}
// Clients will only listen for notifications, not pull data from the server
int BLE::accessImuSampleCharacteristic(uint16_t connectionHandle,
                                       uint16_t attributeHandle,
                                       struct ble_gatt_access_ctxt *context,
                                       void *arg) {
  return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

bool BLE::initializeTasks() {
  BaseType_t transmitTaskResult = xTaskCreate(
      BLE::transmitTask, "transmitTask", BLE::TRANSMIT_TASK_STACK_SIZE, this,
      BLE::TRANSMIT_TASK_PRIORITY, &this->transmitTaskHandle);
  if (transmitTaskResult != pdPASS) {
    this->logger->error("Failed to create transmit task");
    return false;
  }

  BaseType_t bleTaskResult =
      xTaskCreate(BLE::bleTask, "bleTask", BLE::BLE_TASK_STACK_SIZE, this,
                  BLE::BLE_TASK_PRIORITY, &this->bleTaskHandle);
  if (bleTaskResult != pdPASS) {
    this->logger->error("Failed to create BLE task");
    vTaskDelete(this->transmitTaskHandle);
    return false;
  }

  return true;
}

void BLE::bleTask(void *arg) {
  BLE *self = static_cast<BLE *>(arg);

  self->logger->debug("BLE task started");

  nimble_port_run();
}

void BLE::transmitTask(void *arg) {
  BLE *self = static_cast<BLE *>(arg);

  self->logger->debug("Transmit task started");

  IMUSample batchSamples[BLE::PREFERRED_BATCH_SEND_SIZE];
  std::optional<IMUSample> optionalBatchSample;
  uint8_t batchCount;

  uint16_t characteristicHandle, connectionHandle, currentBatchSize;
  bool isSubscribed;

  while (self->doTransmit) {
    // Block until enough samples are available in the pipe to send
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Sincronize access to shared variables
    portENTER_CRITICAL(&self->communicationState.mux);
    characteristicHandle =
        self->communicationState.imuSampleCharacteristicHandle;
    connectionHandle = self->communicationState.connectionHandle;
    isSubscribed =
        self->communicationState.isSubscribedToImuSampleCharacteristic;
    currentBatchSize = self->communicationState.currentBatchSize;
    portEXIT_CRITICAL(&self->communicationState.mux);

    if (characteristicHandle == 0) {
      self->logger->warn(
          "Can't send data: IMU Sample Characteristic handle is invalid");
      continue;
    }
    if (connectionHandle == BLE_HS_CONN_HANDLE_NONE) {
      self->logger->warn("Can't send data: no connected client");
      continue;
    }
    if (!isSubscribed) {
      self->logger->warn("Can't send data: client not subscribed to IMU "
                         "sample Characteristic");
      continue;
    }

    while (self->pipe->itemsFilled() >= currentBatchSize) {
      batchCount = 0;
      optionalBatchSample = self->pipe->pop(false);
      while (batchCount < currentBatchSize &&
             (optionalBatchSample != std::nullopt)) {
        batchSamples[batchCount] = optionalBatchSample.value();
        optionalBatchSample = self->pipe->pop(false);
        batchCount++;
      }

      struct os_mbuf *nimbleBuffer =
          ble_hs_mbuf_from_flat(batchSamples, batchCount * sizeof(IMUSample));

      if (nimbleBuffer == NULL) {
        self->logger->error(
            "Can't send data: no memory available for nimble buffer");
        break;
      }
      // Broadcast notification with data to all connected clients. The
      // buffer's memory is freed by nimble (no os_mbuf_free_chain() needed).
      // Note: ble_gatts_notify() only notifies the client to later pull the
      // data from the server, which I think is less efficient and more
      // complicated than pushing directly
      if (ble_gatts_notify_custom(connectionHandle, characteristicHandle,
                                  nimbleBuffer) != 0) {
        self->logger->error("Failed to send IMU samples notification");
      }
    }
  }
}

bool BLE::initializeAdvertising() {
  this->logger->debug("Initializing advertising");

  // Infer address type: public/private, static/random
  RETURN_FALSE_ON_NIMBLE_ERROR(
      ble_hs_id_infer_auto(0, &this->communicationState.address.type),
      this->logger, "Failed to determine address type");

  RETURN_FALSE_ON_NIMBLE_ERROR(
      ble_hs_id_copy_addr(this->communicationState.address.type,
                          this->communicationState.address.value, NULL),
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

  this->primaryAdvertisingPacket.name = (uint8_t *)BLE::DEVICE_NAME;
  this->primaryAdvertisingPacket.name_len = strlen(BLE::DEVICE_NAME);
  this->primaryAdvertisingPacket.name_is_complete = 1;

  // Advertise the IMU service UUID so apps can filter/discover by service
  this->primaryAdvertisingPacket.uuids128 = &BLE::IMU_SERVICE_UUID;
  this->primaryAdvertisingPacket.num_uuids128 = 1;
  this->primaryAdvertisingPacket.uuids128_is_complete = 1;

  // Scan response packet (31 bytes max):
  // TX power (3) + Appearance (4) + Address (8) + Adv interval (4) = 19 bytes

  // Antenna transmission power
  this->scanResponsePacket.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  this->scanResponsePacket.tx_pwr_lvl_is_present = 1;

  // Appearance
  this->scanResponsePacket.appearance = BLE::APPEARANCE;
  this->scanResponsePacket.appearance_is_present = 1;

  // Address
  this->scanResponsePacket.device_addr = this->communicationState.address.value;
  this->scanResponsePacket.device_addr_type =
      this->communicationState.address.type;
  this->scanResponsePacket.device_addr_is_present = 1;

  // Advertising interval
  this->scanResponsePacket.adv_itvl =
      BLE_GAP_ADV_ITVL_MS(BLE::ADVERTISING_INTERVAL_MS);
  this->scanResponsePacket.adv_itvl_is_present = 1;
  this->advertisingConfig.itvl_min =
      BLE_GAP_ADV_ITVL_MS(BLE::ADVERTISING_INTERVAL_MS);
  this->advertisingConfig.itvl_max =
      BLE_GAP_ADV_ITVL_MS(BLE::ADVERTISING_INTERVAL_MS);

  // Behavior:
  // Connection mode: undirected connectable (accept connections from any
  // device)
  // Discovery mode: general discoverable
  this->advertisingConfig.conn_mode = BLE_GAP_CONN_MODE_UND;
  this->advertisingConfig.disc_mode = BLE_GAP_DISC_MODE_GEN;

  RETURN_FALSE_ON_NIMBLE_ERROR(
      ble_gap_adv_set_fields(&this->primaryAdvertisingPacket), this->logger,
      "Failed to set primary advertising packet fields");

  RETURN_FALSE_ON_NIMBLE_ERROR(
      ble_gap_adv_rsp_set_fields(&this->scanResponsePacket), this->logger,
      "Failed to set scan response packet fields");

  return true;
}
bool BLE::startAdvertising() {
  RETURN_FALSE_ON_NIMBLE_ERROR(
      ble_gap_adv_start(this->communicationState.address.type, NULL,
                        BLE_HS_FOREVER, &this->advertisingConfig,
                        BLE::handleGAPEvent, this),
      this->logger, "Failed to start advertising");

  return true;
}

// TODO: use pipe class methods
void BLE::send(const IMUSample &sample) {
  uint16_t currentBatchSize;
  portENTER_CRITICAL(&this->communicationState.mux);
  currentBatchSize = this->communicationState.currentBatchSize;
  portEXIT_CRITICAL(&this->communicationState.mux);

  // Buffer samples regardless of connection/subscription status
  this->pipe->push(sample);
  if (this->pipe->itemsFilled() >= currentBatchSize) {
    xTaskNotifyGive(this->transmitTaskHandle);
  }
}