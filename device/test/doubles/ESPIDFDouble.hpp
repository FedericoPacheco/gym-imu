#pragma once

extern "C" {
#include <fff.h>
}

#include <ports/ESP-IDF/ESPIDFPort.hpp>

DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpioSetConfig, const gpio_config_t *);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpioAddISRHandler, gpio_num_t, gpio_isr_t,
                        void *);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpioRemoveISRHandler, gpio_num_t);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, nvsFlashInit);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, nvsFlashErase);
DECLARE_FAKE_VALUE_FUNC(int64_t, getTimeUs);

inline void resetESPIDFPortFakes() {
  RESET_FAKE(gpioSetConfig);
  RESET_FAKE(gpioAddISRHandler);
  RESET_FAKE(gpioRemoveISRHandler);
  RESET_FAKE(nvsFlashInit);
  RESET_FAKE(nvsFlashErase);
  RESET_FAKE(getTimeUs);
  FFF_RESET_HISTORY();
}
