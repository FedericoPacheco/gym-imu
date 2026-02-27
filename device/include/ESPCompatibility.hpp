#pragma once

#include <cstddef>
#include <cstdint>

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

typedef int gpio_num_t;
typedef int gpio_pullup_t;
typedef int gpio_pulldown_t;
typedef int gpio_mode_t;
typedef int gpio_int_type_t;
typedef void (*gpio_isr_t)(void *);

struct gpio_config_t {
  uint64_t pin_bit_mask;
  gpio_mode_t mode;
  gpio_pullup_t pull_up_en;
  gpio_pulldown_t pull_down_en;
  gpio_int_type_t intr_type;
};

static constexpr gpio_num_t GPIO_NUM_5 = 5;
static constexpr gpio_num_t GPIO_NUM_6 = 6;
static constexpr gpio_num_t GPIO_NUM_7 = 7;
static constexpr gpio_pullup_t GPIO_PULLUP_DISABLE = 0;
static constexpr gpio_pullup_t GPIO_PULLUP_ENABLE = 1;
static constexpr gpio_pulldown_t GPIO_PULLDOWN_DISABLE = 0;
static constexpr gpio_pulldown_t GPIO_PULLDOWN_ENABLE = 1;
static constexpr gpio_mode_t GPIO_MODE_DISABLE = 0;
static constexpr gpio_mode_t GPIO_MODE_INPUT = 1;
static constexpr gpio_int_type_t GPIO_INTR_DISABLE = 0;
static constexpr gpio_int_type_t GPIO_INTR_POSEDGE = 1;

#else

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"

#endif