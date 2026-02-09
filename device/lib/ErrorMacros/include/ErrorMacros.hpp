#ifndef ERROR_MACROS_HPP
#define ERROR_MACROS_HPP

#include "esp_err.h"

class Logger;

inline bool checkError(esp_err_t err, Logger *logger, const char *msg) {
  if (err != ESP_OK) {
    if (logger != nullptr)
      logger->error("%s: %s", msg, esp_err_to_name(err));
    return false;
  }
  return true;
}

#define RETURN_NULL_ON_ERROR(result, logger, msg)                              \
  if (!checkError((result), (logger), (msg)))                                  \
  return nullptr

#define RETURN_FALSE_ON_ERROR(result, logger, msg)                             \
  if (!checkError((result), (logger), (msg)))                                  \
  return false

#endif // ERROR_MACROS_HPP
