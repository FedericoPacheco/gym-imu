#pragma once

extern "C" {
#include <fff.h>
}

#include <ports/NimBLE/NimBLEPort.hpp>

DECLARE_FAKE_VALUE_FUNC(int, nimblePortInit);
DECLARE_FAKE_VALUE_FUNC(int, nimblePortStop);
DECLARE_FAKE_VOID_FUNC(nimblePortRun);

DECLARE_FAKE_VOID_FUNC(nimbleSetPreferredMtu, uint16_t);

DECLARE_FAKE_VALUE_FUNC(const char *, nimbleUuidToStr, const ble_uuid_t *,
                        char *);

DECLARE_FAKE_VOID_FUNC(nimbleGapServiceInit);
DECLARE_FAKE_VALUE_FUNC(int, nimbleGapDeviceNameSet, const char *);
DECLARE_FAKE_VALUE_FUNC(int, nimbleGapDeviceAppearanceSet, uint16_t);

DECLARE_FAKE_VALUE_FUNC(int, nimbleGattClientExchangeMtu, uint16_t);
DECLARE_FAKE_VALUE_FUNC(int, nimbleGapUpdateParams, uint16_t,
                        struct ble_gap_upd_params *);

DECLARE_FAKE_VOID_FUNC(nimbleGattServiceInit);
DECLARE_FAKE_VALUE_FUNC(int, nimbleGattServerCountConfig,
                        struct ble_gatt_svc_def *);
DECLARE_FAKE_VALUE_FUNC(int, nimbleGattServerAddServices,
                        struct ble_gatt_svc_def *);

DECLARE_FAKE_VALUE_FUNC(struct os_mbuf *, nimbleMbufFromFlat, const void *,
                        uint16_t);
DECLARE_FAKE_VALUE_FUNC(int, nimbleGattServerNotifyCustom, uint16_t, uint16_t,
                        struct os_mbuf *);

DECLARE_FAKE_VALUE_FUNC(int, nimbleInferAutoAddressType, uint8_t *);
DECLARE_FAKE_VALUE_FUNC(int, nimbleCopyAddress, uint8_t, uint8_t *, int *);

DECLARE_FAKE_VALUE_FUNC(int, nimbleGapAdvertisingSetFields,
                        struct ble_hs_adv_fields *);
DECLARE_FAKE_VALUE_FUNC(int, nimbleGapAdvertisingResponseSetFields,
                        struct ble_hs_adv_fields *);
DECLARE_FAKE_VALUE_FUNC(int, nimbleGapAdvertisingStart, uint8_t,
                        struct ble_gap_adv_params *, ble_gap_event_fn *,
                        void *);

inline void resetNimBLEPortFakes() {
  RESET_FAKE(nimblePortInit);
  RESET_FAKE(nimblePortStop);
  RESET_FAKE(nimblePortRun);

  RESET_FAKE(nimbleSetPreferredMtu);

  RESET_FAKE(nimbleUuidToStr);

  RESET_FAKE(nimbleGapServiceInit);
  RESET_FAKE(nimbleGapDeviceNameSet);
  RESET_FAKE(nimbleGapDeviceAppearanceSet);

  RESET_FAKE(nimbleGattClientExchangeMtu);
  RESET_FAKE(nimbleGapUpdateParams);

  RESET_FAKE(nimbleGattServiceInit);
  RESET_FAKE(nimbleGattServerCountConfig);
  RESET_FAKE(nimbleGattServerAddServices);

  RESET_FAKE(nimbleMbufFromFlat);
  RESET_FAKE(nimbleGattServerNotifyCustom);

  RESET_FAKE(nimbleInferAutoAddressType);
  RESET_FAKE(nimbleCopyAddress);

  RESET_FAKE(nimbleGapAdvertisingSetFields);
  RESET_FAKE(nimbleGapAdvertisingResponseSetFields);
  RESET_FAKE(nimbleGapAdvertisingStart);

  FFF_RESET_HISTORY();
}