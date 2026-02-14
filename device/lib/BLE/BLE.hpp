#include "ErrorMacros.hpp"
#include "IMUSensor.hpp"
#include "Logger.hpp"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"
#include <cstddef>
#include <cstdint>
#include <freertos/task.h>
#include <memory>
#include <sys/param.h>

// TODO: document overall behavior
// TODO: perform refactors when possible (sliding, extract func, etc.)

struct BLEAddress {
  uint8_t type;
  uint8_t value[6];
  char readableValue[20];
};

class BLE {
public:
  static constexpr const char *DEVICE_NAME = "Gym-IMU";
  // Motion sensor. See BLE specification .pdf, section 2.6.3.:
  // https://www.bluetooth.com/specifications/assigned-numbers/
  static constexpr int APPEARANCE = 0x0541;

  // Big endian (human readable, most significant byte first):
  // 12345678-1234-5678-1234-56789abcdef0
  // Little endian (common in BLE): 0xf0debc9a785634127856341278563412
  static constexpr ble_uuid128_t IMU_SERVICE_UUID =
      BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x78,
                       0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
  // Big endian: c21c340b-f231-4da2-ab20-716f9ed67c3a
  // Little endian: 0x3a7cd69e6f7120aba24df2310b341cc2
  static constexpr ble_uuid128_t IMU_SAMPLE_CHARACTERISTIC_UUID =
      BLE_UUID128_INIT(0x3a, 0x7c, 0xd6, 0x9e, 0x6f, 0x71, 0x20, 0xab, 0xa2,
                       0x4d, 0xf2, 0x31, 0x0b, 0x34, 0x1c, 0xc2);

  static constexpr int BLE_TASK_PRIORITY = 4;
  static constexpr int BLE_TASK_STACK_SIZE = 4096;
  static constexpr int TRANSMIT_TASK_PRIORITY = 4;
  static constexpr int TRANSMIT_TASK_STACK_SIZE = 4096;
  static constexpr int SAMPLE_QUEUE_SIZE = 128;
  static constexpr int PREFERRED_BATCH_SEND_SIZE = 1; // 6;

  static constexpr int DEFAULT_MTU = 23;
  // Effetive payload = MTU (maximum transmission unit) - 3 bytes for ATT header
  static constexpr int PREFERRED_MTU =
      MIN(sizeof(IMUSample) * BLE::PREFERRED_BATCH_SEND_SIZE + 3, 512);

  static BLE *getInstance(Logger *logger);
  void send(const IMUSample &sample);
  ~BLE();

private:
  Logger *logger;
  static std::unique_ptr<BLE> instance;
  static BLE *initializingInstance;

  ble_gatt_chr_def imuSampleCharacteristic[2];
  uint16_t imuSampleCharacteristicHandle;
  bool isSubscribedToImuSampleCharacteristic;
  ble_gatt_svc_def imuService[2];
  uint16_t connectionHandle;

  TaskHandle_t bleTaskHandle, transmitTaskHandle;
  bool doTransmit;
  portMUX_TYPE mux;
  uint16_t mtu;
  uint16_t currentBatchSize;
  QueueHandle_t sampleQueueHandle;

  BLEAddress address;
  ble_hs_adv_fields primaryAdvertisingPacket;
  ble_hs_adv_fields scanResponsePacket;
  ble_gap_adv_params advertisingConfig;

  BLE(Logger *logger);
  static std::unique_ptr<BLE> create(Logger *logger);

  inline bool initializeFlash();

  inline bool initializeControllerAndStack();
  static void onStackReset(int reason);
  static void onStackSync();
  static void onGATTRegister(struct ble_gatt_register_ctxt *context, void *arg);

  inline bool initializeGAP();
  static int handleGAPEvent(ble_gap_event *event, void *arg);

  inline bool initializeGATT();
  static int accessImuSampleCharacteristic(uint16_t connectionHandle,
                                           uint16_t attributeHandle,
                                           struct ble_gatt_access_ctxt *context,
                                           void *arg);

  inline bool initializeTasks();
  static void bleTask(void *arg);
  static void transmitTask(void *arg);

  inline bool initializeAdvertising();
  bool startAdvertising();
};