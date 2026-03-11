extern "C" {
#include <fff.h>
}

#include "doubles/NimBLEDouble.hpp"

DEFINE_FAKE_VALUE_FUNC(int, nimblePortInit);
DEFINE_FAKE_VALUE_FUNC(int, nimblePortStop);
DEFINE_FAKE_VOID_FUNC(nimblePortRun);

DEFINE_FAKE_VOID_FUNC(nimbleSetPreferredMtu, uint16_t);

DEFINE_FAKE_VALUE_FUNC(const char *, nimbleUuidToStr, const ble_uuid_t *,
                       char *);

DEFINE_FAKE_VOID_FUNC(nimbleGapServiceInit);
DEFINE_FAKE_VALUE_FUNC(int, nimbleGapDeviceNameSet, const char *);
DEFINE_FAKE_VALUE_FUNC(int, nimbleGapDeviceAppearanceSet, uint16_t);

DEFINE_FAKE_VALUE_FUNC(int, nimbleGattClientExchangeMtu, uint16_t);
DEFINE_FAKE_VALUE_FUNC(int, nimbleGapUpdateParams, uint16_t,
                       struct ble_gap_upd_params *);

DEFINE_FAKE_VOID_FUNC(nimbleGattServiceInit);
DEFINE_FAKE_VALUE_FUNC(int, nimbleGattServerCountConfig,
                       struct ble_gatt_svc_def *);
DEFINE_FAKE_VALUE_FUNC(int, nimbleGattServerAddServices,
                       struct ble_gatt_svc_def *);

DEFINE_FAKE_VALUE_FUNC(struct os_mbuf *, nimbleMbufFromFlat, const void *,
                       uint16_t);
DEFINE_FAKE_VALUE_FUNC(int, nimbleGattServerNotifyCustom, uint16_t, uint16_t,
                       struct os_mbuf *);

DEFINE_FAKE_VALUE_FUNC(int, nimbleInferAutoAddressType, uint8_t *);
DEFINE_FAKE_VALUE_FUNC(int, nimbleCopyAddress, uint8_t, uint8_t *, int *);

DEFINE_FAKE_VALUE_FUNC(int, nimbleGapAdvertisingSetFields,
                       struct ble_hs_adv_fields *);
DEFINE_FAKE_VALUE_FUNC(int, nimbleGapAdvertisingResponseSetFields,
                       struct ble_hs_adv_fields *);
DEFINE_FAKE_VALUE_FUNC(int, nimbleGapAdvertisingStart, uint8_t,
                       struct ble_gap_adv_params *, ble_gap_event_fn *, void *);