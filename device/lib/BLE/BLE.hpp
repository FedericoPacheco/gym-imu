#pragma once
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
#include <Constants.hpp>
#include <ErrorMacros.hpp>
#include <IMUSensor.hpp>
#include <Logger.hpp>
#include <Pipe.hpp>
#include <cstddef>
#include <cstdint>
#include <freertos/task.h>
#include <memory>
#include <optional>
#include <sys/param.h>

// TODO: add security

/*
Overview:
An application layer class to manage Bluetooth Low Energy (BLE) communication,
including initialization, advertising to other devices, and data transmission.

How it works:
The first time getInstance() is called, it creates a singleton instance that:
1. Initializes NVS flash for storing BLE bonding information and keys.
2. Initializes the NimBLE host stack and controller.
3. Sets up GAP (Generic Access Profile) for advertising and connection
management.
4. Sets up GATT (Generic Attribute Profile) for defining services and
characteristics. Currently there's one service and one characteristic for
sending IMU samples.
5. Creates FreeRTOS tasks for handling BLE events and transmitting data.
6. Starts advertising to allow other devices to discover and connect, passing
information such as device name, appearance, supported services (currently
notifications, a stream of data sent from the device with no ACK from the
client), address, connection/discovery mode, etc. Allows only one connection.
When send() is called with an IMUSample, it adds the sample to a pipe.
The transmit task waits until there are enough samples in the pipe (based on
negotiated MTU) and then sends them as a notification to connected
clients.

How to use:
unique_ptr<BLE> ble = BLE::getInstance(logger, pipe);
...
ble->send(sample);

Data can be received on the phone with the app nRF Connect.

Notes:
Notifications, not allowing reads from the client, and only allowing a single
connection were chosen to reduce radio strain, reduce latency, and maximize data
streaming capabilities.

Definitions:
- Controller: low-level hardware that manages communication through radio
(sending/receiving signals, packets, error handling, encoding/decoding, channel
hopping, etc.). Runs separate from the CPU and accepts HCI (Host Controller
Interface) commands from the host stack. Subdivided in Link Layer (LL) and
Physical Layer (PHY).
- Host stack: software that implements the Bluetooth protocol, built on top of
the controller, exposing a high-level API to the application. Handles GAP, GATT,
security, etc. Runs on the CPU and listens for events from the controller.
*/

class BLE {
public:
  static constexpr const char *DEVICE_NAME = "Gym-IMU";
  // Motion sensor. See BLE specification .pdf, section 2.6.3.:
  // https://www.bluetooth.com/specifications/assigned-numbers/
  static constexpr int APPEARANCE = 0x0541;

  static constexpr int PREFERRED_BATCH_SEND_SIZE = 6;

  // MTU: maximum transmission unit, the largest payload size that can be sent
  // in a single BLE packet
  static constexpr int DEFAULT_MTU = 23;
  // Effetive payload = MTU (maximum transmission unit) - 3 bytes for ATT header
  static constexpr int PREFERRED_MTU =
      MIN(sizeof(IMUSample) * PREFERRED_BATCH_SEND_SIZE + 3, 512);
  static constexpr int TRANSMIT_TASK_SHUTDOWN_DELAY_MS = 250;
  // Time between data exchanges when connected
  static constexpr int CONNECTION_INTERVAL_MIN_MS = 15;
  static constexpr int CONNECTION_INTERVAL_MAX_MS = 30;
  // Amount of connection events that can be skipped to save power
  static constexpr int CONNECTION_PERIPHERAL_LATENCY = 0;
  // Time after which the connection is considered lost if no
  // packets are received
  static constexpr int CONNECTION_SUPERVISION_TIMEOUT_MS = 5000;

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

  // Time between advertising packets when not connected
  static constexpr int ADVERTISING_INTERVAL_MS = 100;

  static BLE *
  getInstance(Logger *logger,
              std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> pipe);
  void send(const IMUSample &sample);
  ~BLE();

private:
  static std::unique_ptr<BLE> instance;
  static BLE *initializingInstance;
  struct BLEAddress {
    uint8_t type;
    uint8_t value[6];
    char readableValue[20];
  };

  struct BLECommunicationState {
    portMUX_TYPE mux;
    uint16_t connectionHandle;
    uint16_t mtu;
    uint16_t currentBatchSize;
    bool isSubscribedToImuSampleCharacteristic;
    uint16_t imuSampleCharacteristicHandle;
    BLEAddress address;
  } communicationState;

  Logger *logger;

  ble_gatt_chr_def imuCharacteristics[2];
  ble_gatt_svc_def services[2];

  TaskHandle_t bleTaskHandle, transmitTaskHandle;
  bool doTransmit;
  std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> pipe;

  ble_hs_adv_fields primaryAdvertisingPacket;
  ble_hs_adv_fields scanResponsePacket;
  ble_gap_adv_params advertisingConfig;

  BLE(Logger *logger,
      std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> pipe);
  static std::unique_ptr<BLE>
  create(Logger *logger,
         std::shared_ptr<Pipe<IMUSample, TRANSMISSION_PIPE_SIZE>> pipe);

  inline bool initializeFlash();

  inline bool initializeControllerAndHost();
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
