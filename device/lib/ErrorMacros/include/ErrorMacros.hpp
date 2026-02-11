#ifndef ERROR_MACROS_HPP
#define ERROR_MACROS_HPP

#include "Logger.hpp"
#include "esp_err.h"
#include "host/ble_hs.h"
#include <cstdio>

#define RETURN_NULL_ON_ERROR(result, logger, msg)                              \
  if (!checkError((result), (logger), (msg)))                                  \
  return nullptr
#define RETURN_FALSE_ON_ERROR(result, logger, msg)                             \
  if (!checkError((result), (logger), (msg)))                                  \
  return false

inline bool checkError(esp_err_t err, Logger *logger, const char *msg) {
  if (err != ESP_OK) {
    if (logger != nullptr)
      logger->error("%s: %s", msg, esp_err_to_name(err));
    return false;
  }
  return true;
}

inline const char *nimble_err_to_name(esp_err_t err);
#define RETURN_NULL_ON_NIMBLE_ERROR(result, logger, msg)                       \
  if (!checkNimbleError((result), (logger), (msg)))                            \
  return nullptr
#define RETURN_FALSE_ON_NIMBLE_ERROR(result, logger, msg)                      \
  if (!checkNimbleError((result), (logger), (msg)))                            \
  return false

inline bool checkNimbleError(int errorCode, Logger *logger, const char *msg) {
  if (errorCode != 0) {
    if (logger != nullptr)
      logger->error("%s: %s (%d)", msg, nimble_err_to_name(errorCode),
                    errorCode);
    return false;
  }
  return true;
}

inline const char *nimble_err_to_name(int errorCode) {
  static char messageBuffer[64];

  switch (errorCode) {
  case 0:
    return "OK";
  case BLE_HS_EAGAIN:
    return "EAGAIN";
  case BLE_HS_EALREADY:
    return "EALREADY";
  case BLE_HS_EINVAL:
    return "EINVAL";
  case BLE_HS_EMSGSIZE:
    return "EMSGSIZE";
  case BLE_HS_ENOENT:
    return "ENOENT";
  case BLE_HS_ENOMEM:
    return "ENOMEM";
  case BLE_HS_ENOTCONN:
    return "ENOTCONN";
  case BLE_HS_ENOTSUP:
    return "ENOTSUP";
  case BLE_HS_EAPP:
    return "EAPP";
  case BLE_HS_EBADDATA:
    return "EBADDATA";
  case BLE_HS_EOS:
    return "EOS";
  case BLE_HS_ECONTROLLER:
    return "ECONTROLLER";
  case BLE_HS_ETIMEOUT:
    return "ETIMEOUT";
  case BLE_HS_EDONE:
    return "EDONE";
  case BLE_HS_EBUSY:
    return "EBUSY";
  case BLE_HS_EREJECT:
    return "EREJECT";
  case BLE_HS_EUNKNOWN:
    return "EUNKNOWN";
  case BLE_HS_EROLE:
    return "EROLE";
  case BLE_HS_ETIMEOUT_HCI:
    return "ETIMEOUT_HCI";
  case BLE_HS_ENOMEM_EVT:
    return "ENOMEM_EVT";
  case BLE_HS_ENOADDR:
    return "ENOADDR";
  case BLE_HS_ENOTSYNCED:
    return "ENOTSYNCED";
  case BLE_HS_EAUTHEN:
    return "EAUTHEN";
  case BLE_HS_EAUTHOR:
    return "EAUTHOR";
  case BLE_HS_EENCRYPT:
    return "EENCRYPT";
  case BLE_HS_EENCRYPT_KEY_SZ:
    return "EENCRYPT_KEY_SZ";
  case BLE_HS_ESTORE_CAP:
    return "ESTORE_CAP";
  case BLE_HS_ESTORE_FAIL:
    return "ESTORE_FAIL";
  case BLE_HS_EPREEMPTED:
    return "EPREEMPTED";
  case BLE_HS_EDISABLED:
    return "EDISABLED";
  case BLE_HS_ESTALLED:
    return "ESTALLED";
  default: {
    if (errorCode >= BLE_HS_ERR_ATT_BASE && errorCode < BLE_HS_ERR_HCI_BASE) {
      std::snprintf(messageBuffer, sizeof(messageBuffer), "ATT error 0x%02x",
                    errorCode - BLE_HS_ERR_ATT_BASE);
      return messageBuffer;
    } else if (errorCode >= BLE_HS_ERR_HCI_BASE &&
               errorCode < BLE_HS_ERR_L2C_BASE) {
      std::snprintf(messageBuffer, sizeof(messageBuffer), "HCI error 0x%02x",
                    errorCode - BLE_HS_ERR_HCI_BASE);
      return messageBuffer;
    } else if (errorCode >= BLE_HS_ERR_L2C_BASE &&
               errorCode < BLE_HS_ERR_SM_US_BASE) {
      std::snprintf(messageBuffer, sizeof(messageBuffer), "L2CAP error 0x%02x",
                    errorCode - BLE_HS_ERR_L2C_BASE);
      return messageBuffer;
    }
    return "Unknown host error";
  }
  }
}

#endif
