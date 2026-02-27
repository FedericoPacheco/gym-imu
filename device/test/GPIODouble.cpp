extern "C" {
#include <fff.h>
}

#include "doubles/IMU/GPIODouble.hpp"

DEFINE_FFF_GLOBALS;

DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpioConfigureInterrupt,
                       const gpio_config_t *);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpioAddISRHandler, gpio_num_t, gpio_isr_t,
                       void *);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpioRemoveISRHandler, gpio_num_t);
DEFINE_FAKE_VALUE_FUNC(int64_t, gpioGetTimeUs);
