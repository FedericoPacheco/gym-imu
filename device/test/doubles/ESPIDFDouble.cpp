extern "C" {
#include <fff.h>
}

#include "doubles/ESPIDFDouble.hpp"

// FFF globals are intentionally defined in the test translation unit
// (e.g. MPU6050Sensor.test.cpp) to avoid multiple-definition linker errors
// when several *Double.cpp files are included together.

DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpioSetConfig, const gpio_config_t *);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpioAddISRHandler, gpio_num_t, gpio_isr_t,
                       void *);
DEFINE_FAKE_VALUE_FUNC(esp_err_t, gpioRemoveISRHandler, gpio_num_t);
DEFINE_FAKE_VALUE_FUNC(int64_t, getTimeUs);
