#pragma once

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)

#include <cstdint>

struct ble_uuid_t {
  uint8_t type;
};

struct ble_uuid128_t {
  ble_uuid_t u;
  uint8_t value[16];
};

#define BLE_UUID128_INIT(...)                                                  \
  {                                                                            \
    {0}, { __VA_ARGS__ }                                                       \
  }

struct ble_gatt_access_ctxt {};
struct os_mbuf {
  uint8_t storage;
};

struct ble_gatt_chr_def {
  const ble_uuid_t *uuid;
  int (*access_cb)(uint16_t connectionHandle, uint16_t attributeHandle,
                   ble_gatt_access_ctxt *context, void *arg);
  void *arg;
  uint16_t flags;
  uint16_t *val_handle;
  uint8_t min_key_size;
};

struct ble_gatt_svc_def {
  uint8_t type;
  const ble_uuid_t *uuid;
  ble_gatt_chr_def *characteristics;
};

struct ble_hs_adv_fields {
  uint8_t flags;
  uint8_t *name;
  uint8_t name_len;
  uint8_t name_is_complete;
  const ble_uuid128_t *uuids128;
  uint8_t num_uuids128;
  uint8_t uuids128_is_complete;
  int8_t tx_pwr_lvl;
  uint8_t tx_pwr_lvl_is_present;
  uint16_t appearance;
  uint8_t appearance_is_present;
  uint8_t *device_addr;
  uint8_t device_addr_type;
  uint8_t device_addr_is_present;
  uint16_t adv_itvl;
  uint8_t adv_itvl_is_present;
};

struct ble_gap_adv_params {
  uint16_t itvl_min;
  uint16_t itvl_max;
  uint8_t conn_mode;
  uint8_t disc_mode;
};

struct ble_gap_upd_params {
  uint16_t itvl_min;
  uint16_t itvl_max;
  uint16_t latency;
  uint16_t supervision_timeout;
  uint16_t min_ce_len;
  uint16_t max_ce_len;
};

struct ble_gap_event {
  uint8_t type;
  struct {
    int status;
    uint16_t conn_handle;
  } connect;
  struct {
    uint16_t attr_handle;
    uint8_t cur_notify;
  } subscribe;
  struct {
    int reason;
  } disconnect;
  struct {
    int status;
  } notify_tx;
  struct {
    uint16_t value;
  } mtu;
};

typedef int ble_gap_event_fn(struct ble_gap_event *event, void *arg);

// Host tests provide no-op implementations for NimBLE persistent store hooks.
void ble_store_util_status_rr(void);
void ble_store_config_read(void);
void ble_store_config_write(void);
void ble_store_config_delete(void);

using ble_store_callback_t = void (*)(void);

struct ble_gatt_register_ctxt {
  uint8_t op;
  struct {
    const ble_gatt_svc_def *svc_def;
    uint16_t handle;
  } svc;
  struct {
    const ble_gatt_chr_def *chr_def;
    uint16_t def_handle;
    uint16_t val_handle;
  } chr;
};

struct ble_hs_cfg_t {
  void (*reset_cb)(int reason);
  void (*sync_cb)(void);
  void (*gatts_register_cb)(struct ble_gatt_register_ctxt *context, void *arg);
  void *gatts_register_arg;
  ble_store_callback_t store_status_cb;
  ble_store_callback_t store_read_cb;
  ble_store_callback_t store_write_cb;
  ble_store_callback_t store_delete_cb;
};

extern ble_hs_cfg_t ble_hs_cfg;

static constexpr uint16_t BLE_HS_CONN_HANDLE_NONE = 0xFFFFu;
static constexpr int BLE_HS_EDONE = 1;
static constexpr int BLE_HS_FOREVER = -1;

static constexpr uint8_t BLE_GATT_REGISTER_OP_SVC = 0;
static constexpr uint8_t BLE_GATT_REGISTER_OP_CHR = 1;
static constexpr uint8_t BLE_GATT_REGISTER_OP_DSC = 2;

static constexpr uint8_t BLE_GAP_EVENT_CONNECT = 0;
static constexpr uint8_t BLE_GAP_EVENT_SUBSCRIBE = 1;
static constexpr uint8_t BLE_GAP_EVENT_DISCONNECT = 2;
static constexpr uint8_t BLE_GAP_EVENT_NOTIFY_TX = 3;
static constexpr uint8_t BLE_GAP_EVENT_MTU = 4;

static constexpr uint16_t BLE_ATT_ERR_REQ_NOT_SUPPORTED = 0x06;

static constexpr uint16_t BLE_GATT_CHR_F_NOTIFY = 0x0010;
static constexpr uint8_t BLE_GATT_SVC_TYPE_PRIMARY = 0x01;

static constexpr uint8_t BLE_HS_ADV_F_DISC_GEN = 0x02;
static constexpr uint8_t BLE_HS_ADV_F_BREDR_UNSUP = 0x04;
static constexpr int8_t BLE_HS_ADV_TX_PWR_LVL_AUTO = 0;

static constexpr uint8_t BLE_GAP_CONN_MODE_UND = 0;
static constexpr uint8_t BLE_GAP_DISC_MODE_GEN = 0;

static constexpr uint16_t BLE_GAP_INITIAL_CONN_MIN_CE_LEN = 0;
static constexpr uint16_t BLE_GAP_INITIAL_CONN_MAX_CE_LEN = 0;

#ifndef BLE_GAP_ADV_ITVL_MS
#define BLE_GAP_ADV_ITVL_MS(ms) (ms)
#endif

#else

#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#endif