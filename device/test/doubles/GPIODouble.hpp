#pragma once

extern "C" {
#include <fff.h>
}

#include <ports/ESP-IDF/GPIOPort.hpp>

DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpioConfigureInterrupt,
                        const gpio_config_t *);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpioAddISRHandler, gpio_num_t, gpio_isr_t,
                        void *);
DECLARE_FAKE_VALUE_FUNC(esp_err_t, gpioRemoveISRHandler, gpio_num_t);
DECLARE_FAKE_VALUE_FUNC(int64_t, gpioGetTimeUs);

inline void resetGPIOPortFakes() {
  RESET_FAKE(gpioConfigureInterrupt);
  RESET_FAKE(gpioAddISRHandler);
  RESET_FAKE(gpioRemoveISRHandler);
  RESET_FAKE(gpioGetTimeUs);
  FFF_RESET_HISTORY();
}
