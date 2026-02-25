#pragma once

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)

using esp_err_t = int;
static constexpr esp_err_t ESP_OK = 0;
static constexpr esp_err_t ESP_FAIL = -1;

inline const char *esp_err_to_name(esp_err_t err) {
  if (err == ESP_OK)
    return "ESP_OK";
  if (err == ESP_FAIL)
    return "ESP_FAIL";
  return "ESP_ERR_UNKNOWN";
}

#else

#include "esp_err.h"

#endif