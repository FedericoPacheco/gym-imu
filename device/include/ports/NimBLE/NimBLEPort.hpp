#pragma once

#include <ports/NimBLE/NimBLECompatibility.hpp>

int nimblePortInit();
int nimblePortStop();
void nimblePortRun();

void nimbleSetPreferredMtu(uint16_t mtu);
const char *nimbleUuidToStr(const ble_uuid_t *uuid, char *buffer);

void nimbleGapServiceInit();
int nimbleGapDeviceNameSet(const char *deviceName);
int nimbleGapDeviceAppearanceSet(uint16_t appearance);

int nimbleGattClientExchangeMtu(uint16_t connectionHandle);
int nimbleGapUpdateParams(uint16_t connectionHandle,
                          struct ble_gap_upd_params *params);

void nimbleGattServiceInit();
int nimbleGattServerCountConfig(struct ble_gatt_svc_def *services);
int nimbleGattServerAddServices(struct ble_gatt_svc_def *services);

struct os_mbuf *nimbleMbufFromFlat(const void *data, uint16_t length);
int nimbleGattServerNotifyCustom(uint16_t connectionHandle,
                                 uint16_t characteristicHandle,
                                 struct os_mbuf *buffer);

int nimbleInferAutoAddressType(uint8_t *addressType);
int nimbleCopyAddress(uint8_t addressType, uint8_t *address,
                      int *identityAddress);

int nimbleGapAdvertisingSetFields(struct ble_hs_adv_fields *fields);
int nimbleGapAdvertisingResponseSetFields(struct ble_hs_adv_fields *fields);
int nimbleGapAdvertisingStart(uint8_t addressType,
                              struct ble_gap_adv_params *advertisingParams,
                              ble_gap_event_fn *eventHandler, void *eventArg);