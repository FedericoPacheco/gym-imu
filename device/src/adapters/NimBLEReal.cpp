#include <ports/NimBLE/NimBLEPort.hpp>

int nimblePortInit() { return nimble_port_init(); }

int nimblePortStop() {
  nimble_port_stop();
  return 0;
}

void nimblePortRun() { nimble_port_run(); }

void nimbleSetPreferredMtu(uint16_t mtu) { ble_att_set_preferred_mtu(mtu); }

const char *nimbleUuidToStr(const ble_uuid_t *uuid, char *buffer) {
  return ble_uuid_to_str(uuid, buffer);
}

void nimbleGapServiceInit() { ble_svc_gap_init(); }

int nimbleGapDeviceNameSet(const char *deviceName) {
  return ble_svc_gap_device_name_set(deviceName);
}

int nimbleGapDeviceAppearanceSet(uint16_t appearance) {
  return ble_svc_gap_device_appearance_set(appearance);
}

int nimbleGattClientExchangeMtu(uint16_t connectionHandle) {
  return ble_gattc_exchange_mtu(connectionHandle, nullptr, nullptr);
}

int nimbleGapUpdateParams(uint16_t connectionHandle,
                          struct ble_gap_upd_params *params) {
  return ble_gap_update_params(connectionHandle, params);
}

void nimbleGattServiceInit() { ble_svc_gatt_init(); }

int nimbleGattServerCountConfig(struct ble_gatt_svc_def *services) {
  return ble_gatts_count_cfg(services);
}

int nimbleGattServerAddServices(struct ble_gatt_svc_def *services) {
  return ble_gatts_add_svcs(services);
}

struct os_mbuf *nimbleMbufFromFlat(const void *data, uint16_t length) {
  return ble_hs_mbuf_from_flat(data, length);
}

int nimbleGattServerNotifyCustom(uint16_t connectionHandle,
                                 uint16_t characteristicHandle,
                                 struct os_mbuf *buffer) {
  return ble_gatts_notify_custom(connectionHandle, characteristicHandle,
                                 buffer);
}

int nimbleInferAutoAddressType(uint8_t *addressType) {
  return ble_hs_id_infer_auto(0, addressType);
}

int nimbleCopyAddress(uint8_t addressType, uint8_t *address,
                      int *identityAddress) {
  return ble_hs_id_copy_addr(addressType, address, identityAddress);
}

int nimbleGapAdvertisingSetFields(struct ble_hs_adv_fields *fields) {
  return ble_gap_adv_set_fields(fields);
}

int nimbleGapAdvertisingResponseSetFields(struct ble_hs_adv_fields *fields) {
  return ble_gap_adv_rsp_set_fields(fields);
}

int nimbleGapAdvertisingStart(uint8_t addressType,
                              struct ble_gap_adv_params *advertisingParams,
                              ble_gap_event_fn *eventHandler, void *eventArg) {
  return ble_gap_adv_start(addressType, nullptr, BLE_HS_FOREVER,
                           advertisingParams, eventHandler, eventArg);
}