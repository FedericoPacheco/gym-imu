#include "BLE.hpp"

std::unique_ptr<BLE> BLE::instance = nullptr;
// Temporary pointer used while a BLE instance is being created and the static
// `instance` unique_ptr has not yet been assigned. Some NimBLE callbacks
// (e.g. sync) may be invoked during initialization. Use this to route those
// early callbacks to the creating object to avoid null crashes.
BLE *BLE::initializingInstance = nullptr;

const ble_uuid128_t BLE::IMU_SERVICE_UUID =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

const ble_uuid128_t BLE::IMU_SAMPLE_CHARACTERISTIC_UUID =
    BLE_UUID128_INIT(0x3a, 0x7c, 0xd6, 0x9e, 0x6f, 0x71, 0x20, 0xab, 0xa2, 0x4d,
                     0xf2, 0x31, 0x0b, 0x34, 0x1c, 0xc2);

BLE::BLE(Logger *logger)
    : logger(logger), mtu(BLE::DEFAULT_MTU), imuSampleCharacteristicHandle(0),
      address{}, primaryAdvertisingPacket{}, scanResponsePacket{},
      advertisingConfig{}, bleTaskHandle(nullptr), doTransmit(true),
      currentBatchSize(PREFERRED_BATCH_SEND_SIZE), transmitTaskHandle(nullptr),
      sampleQueueHandle(nullptr), mux(portMUX_INITIALIZER_UNLOCKED),
      connectionHandle(BLE_HS_CONN_HANDLE_NONE),
      isSubscribedToImuSampleCharacteristic(false) {

  // Default initialize to 0 to avoid garbage values and the apply custom
  // settings
  memset(imuSampleCharacteristic, 0, sizeof(imuSampleCharacteristic));
  imuSampleCharacteristic[0].uuid = &IMU_SAMPLE_CHARACTERISTIC_UUID.u;
  imuSampleCharacteristic[0].access_cb = BLE::accessImuSampleCharacteristic;
  imuSampleCharacteristic[0].arg = this;
  imuSampleCharacteristic[0].flags = BLE_GATT_CHR_F_NOTIFY;
  imuSampleCharacteristic[0].val_handle = &imuSampleCharacteristicHandle;

  memset(imuService, 0, sizeof(imuService));
  imuService[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
  imuService[0].uuid = &IMU_SERVICE_UUID.u;
  imuService[0].characteristics = imuSampleCharacteristic;
}

std::unique_ptr<BLE> BLE::create(Logger *logger) {
  std::unique_ptr<BLE> ble(new (std::nothrow) BLE(logger));

  if (!ble) {
    if (logger)
      logger->error("BLE", "Failed to allocate memory for BLE instance");
    return nullptr;
  }

  // Make early callbacks target this object while we initialize the host
  // stack. Clear on all exit paths to avoid leaving a dangling pointer.
  BLE::initializingInstance = ble.get();

  if (!ble->initializeFlash()) {
    BLE::initializingInstance = nullptr;
    return nullptr;
  }
  if (!ble->initializeControllerAndStack()) {
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
BLE *BLE::getInstance(Logger *logger) {
  if (!BLE::instance)
    BLE::instance = BLE::create(logger);
  return BLE::instance.get();
}

BLE::~BLE() {
  // Transmit remaining samples before shutting down
  if (this->transmitTaskHandle) {
    this->doTransmit = false;
    xTaskNotifyGive(this->transmitTaskHandle);
    vTaskDelay(pdMS_TO_TICKS(250));
    vTaskDelete(this->transmitTaskHandle);
  }
  if (this->sampleQueueHandle)
    vQueueDelete(this->sampleQueueHandle);

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

/*
- Controller: low-level hardware that manages communication through radio
(sending/receiving signals, packets, error handling, encoding/decoding, channel
hopping, etc.). Runs separate from the CPU and accepts HCI (Host Controller
Interface) commands from the host stack.
- Stack: software that implements the Bluetooth protocol, built on top of the
controller, exposing a high-level API to the application. Handles GAP, GATT,
security, etc. Runs on the CPU and listens for events from the controller.
*/
bool BLE::initializeControllerAndStack() {
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

// GAP: Generic Access Profile
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
// Reference:
// https://freertos.org/Documentation/02-Kernel/04-API-references/04-RTOS-kernel-control/01-taskENTER_CRITICAL_taskEXIT_CRITICAL
// Used here to protect access to shared variables (connectionHandle,
// isSubscribed..., imuSampleCharacteristicHandle, batchSize) that are written
// here and read on transmitTask() and send()
int BLE::handleGAPEvent(ble_gap_event *event, void *arg) {
  BLE *self = static_cast<BLE *>(arg);

  self->logger->debug("Received GAP event: %d", event->type);

  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT: {
    if (event->connect.status == 0) {
      // Allow only one connection at a time (e.g. phone or laptop)
      portENTER_CRITICAL(&self->mux);
      self->connectionHandle = event->connect.conn_handle;
      portEXIT_CRITICAL(&self->mux);
      self->logger->info("Connection established");

      // struct ble_gap_conn_desc descriptor;
      // RETURN_FALSE_ON_NIMBLE_ERROR(
      //     ble_gap_conn_find(self->connectionHandle, &descriptor),
      //     self->logger, "Failed to find connection by handle");

      RETURN_FALSE_ON_NIMBLE_ERROR(
          ble_gattc_exchange_mtu(self->connectionHandle, NULL, NULL),
          self->logger,
          "Failed to negotiate MTU with client, using default (23 bytes)");

    } else {
      self->logger->error("Connection failed with error code: %d",
                          event->connect.status);
      self->startAdvertising();
    }
    break;
  }
  case BLE_GAP_EVENT_SUBSCRIBE: {
    if (event->subscribe.attr_handle == self->imuSampleCharacteristicHandle) {
      if (event->subscribe.cur_notify) {
        portENTER_CRITICAL(&self->mux);
        self->isSubscribedToImuSampleCharacteristic = true;
        portEXIT_CRITICAL(&self->mux);
        self->logger->info("Client subscribed to IMU Sample Characteristic");
      } else {
        portENTER_CRITICAL(&self->mux);
        self->isSubscribedToImuSampleCharacteristic = false;
        portEXIT_CRITICAL(&self->mux);
        self->logger->info(
            "Client unsubscribed from IMU Sample Characteristic");
      }
    }
    break;
  }
  case BLE_GAP_EVENT_DISCONNECT: {
    self->logger->info("Connection terminated, reason: %d",
                       event->disconnect.reason);
    portENTER_CRITICAL(&self->mux);
    self->connectionHandle = BLE_HS_CONN_HANDLE_NONE;
    self->isSubscribedToImuSampleCharacteristic = false;
    portEXIT_CRITICAL(&self->mux);
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
    portENTER_CRITICAL(&self->mux);
    self->mtu = event->mtu.value;
    self->currentBatchSize = MIN(BLE::PREFERRED_BATCH_SEND_SIZE,
                                 MAX(1, (self->mtu - 3) / sizeof(IMUSample)));
    portEXIT_CRITICAL(&self->mux);
    self->logger->info("Negotiated MTU: %d. Current batch size: %d", self->mtu,
                       self->currentBatchSize);
    break;
  }
  }

  return 0;
}

// GATT: Generic Attribute Profile
bool BLE::initializeGATT() {
  this->logger->debug("Initializing GATT service");

  ble_svc_gatt_init();

  RETURN_FALSE_ON_NIMBLE_ERROR(ble_gatts_count_cfg(this->imuService),
                               this->logger,
                               "Failed to count GATT configuration");

  RETURN_FALSE_ON_NIMBLE_ERROR(ble_gatts_add_svcs(this->imuService),
                               this->logger, "Failed to add GATT services");

  return true;
}
// Clients will only listen for notifications, not pull data from the server
int BLE::accessImuSampleCharacteristic(uint16_t connectionHandle,
                                       uint16_t attributeHandle,
                                       struct ble_gatt_access_ctxt *context,
                                       void *arg) {
  return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

bool BLE::initializeAdvertising() {
  this->logger->debug("Initializing advertising");

  // Infer address type: public or random
  RETURN_FALSE_ON_NIMBLE_ERROR(ble_hs_id_infer_auto(0, &this->address.type),
                               this->logger,
                               "Failed to determine address type");

  RETURN_FALSE_ON_NIMBLE_ERROR(
      ble_hs_id_copy_addr(this->address.type, this->address.value, NULL),
      this->logger, "Failed to read device address");

  sprintf(this->address.readableValue, "0x%02X:%02X:%02X:%02X:%02X:%02X",
          this->address.value[0], this->address.value[1],
          this->address.value[2], this->address.value[3],
          this->address.value[4], this->address.value[5]);
  this->logger->info("Device BLE address: %s", this->address.readableValue);

  // Packet fields:
  // General discoverable, LE-only
  this->primaryAdvertisingPacket.flags =
      BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  this->primaryAdvertisingPacket.name = (uint8_t *)BLE::DEVICE_NAME;
  this->primaryAdvertisingPacket.name_len = strlen(BLE::DEVICE_NAME);
  this->primaryAdvertisingPacket.name_is_complete = 1;

  // Antenna transmission power
  this->primaryAdvertisingPacket.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  this->primaryAdvertisingPacket.tx_pwr_lvl_is_present = 1;

  this->primaryAdvertisingPacket.appearance = BLE::APPEARANCE;
  this->primaryAdvertisingPacket.appearance_is_present = 1;

  // Device role: peripheral only (not central, so it can't connect to other
  // devices, only accept connections).
  // Disclaimer: present in ESP-IDF example, but it DOESN'T COMPILE.
  // this->primaryAdvertisingPacket.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
  // this->primaryAdvertisingPacket.le_role_is_present = 1;

  this->scanResponsePacket.device_addr = this->address.value;
  this->scanResponsePacket.device_addr_type = this->address.type;
  this->scanResponsePacket.device_addr_is_present = 1;

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

bool BLE::initializeTasks() {
  this->sampleQueueHandle =
      xQueueCreate(BLE::SAMPLE_QUEUE_SIZE, sizeof(IMUSample));
  if (this->sampleQueueHandle == NULL) {
    this->logger->error("Failed to create sample queue");
    return false;
  }

  BaseType_t transmitTaskResult = xTaskCreate(
      BLE::transmitTask, "transmitTask", BLE::TRANSMIT_TASK_STACK_SIZE, this,
      BLE::TRANSMIT_TASK_PRIORITY, &this->transmitTaskHandle);
  if (transmitTaskResult != pdPASS) {
    this->logger->error("Failed to create transmit task");
    vQueueDelete(this->sampleQueueHandle);
    return false;
  }

  BaseType_t bleTaskResult =
      xTaskCreate(BLE::bleTask, "bleTask", BLE::BLE_TASK_STACK_SIZE, this,
                  BLE::BLE_TASK_PRIORITY, &this->bleTaskHandle);
  if (bleTaskResult != pdPASS) {
    this->logger->error("Failed to create BLE task");
    vQueueDelete(this->sampleQueueHandle);
    vTaskDelete(this->transmitTaskHandle);
    return false;
  }

  return true;
}

bool BLE::startAdvertising() {
  RETURN_FALSE_ON_NIMBLE_ERROR(
      ble_gap_adv_start(this->address.type, NULL, BLE_HS_FOREVER,
                        &this->advertisingConfig, BLE::handleGAPEvent, this),
      this->logger, "Failed to start advertising");

  return true;
}

void BLE::bleTask(void *arg) {
  BLE *self = static_cast<BLE *>(arg);

  self->logger->debug("BLE task started");

  nimble_port_run();
}

void BLE::send(const IMUSample &sample) {

  uint16_t currentBatchSize;
  portENTER_CRITICAL(&this->mux);
  currentBatchSize = this->currentBatchSize;
  portEXIT_CRITICAL(&this->mux);

  // Buffer samples regardless of connection/subscription status
  if (xQueueSend(this->sampleQueueHandle, &sample, 0) == pdPASS) {
    UBaseType_t queueSize = uxQueueMessagesWaiting(this->sampleQueueHandle);
    if (queueSize >= currentBatchSize) {
      xTaskNotifyGive(this->transmitTaskHandle);
    }
  } else {
    this->logger->warn("BLE sample queue is full, dropping oldest sample to "
                       "make room for new one");
    if (xQueueReceive(this->sampleQueueHandle, NULL, 0) != pdPASS) {
      this->logger->error("Failed to remove oldest sample from queue");
      return;
    }
    xQueueSend(this->sampleQueueHandle, &sample, 0);
  }
}

void BLE::transmitTask(void *arg) {
  BLE *self = static_cast<BLE *>(arg);

  self->logger->debug("Transmit task started");

  IMUSample batchSamples[BLE::PREFERRED_BATCH_SEND_SIZE];
  UBaseType_t currentQueueSize;
  uint8_t batchCount;

  uint16_t characteristicHandle, connectionHandle, currentBatchSize;
  bool isSubscribed;

  while (self->doTransmit) {
    // Block until enough samples are available in the queue to send
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Sincronize access to shared variables
    portENTER_CRITICAL(&self->mux);
    characteristicHandle = self->imuSampleCharacteristicHandle;
    connectionHandle = self->connectionHandle;
    isSubscribed = self->isSubscribedToImuSampleCharacteristic;
    currentBatchSize = self->currentBatchSize;
    portEXIT_CRITICAL(&self->mux);

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

    currentQueueSize = uxQueueMessagesWaiting(self->sampleQueueHandle);
    while (currentQueueSize >= currentBatchSize) {

      batchCount = 0;
      while (batchCount < currentBatchSize &&
             xQueueReceive(self->sampleQueueHandle, &batchSamples[batchCount],
                           0) == pdPASS)
        batchCount++;

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

      currentQueueSize = uxQueueMessagesWaiting(self->sampleQueueHandle);
    }
  }
}
