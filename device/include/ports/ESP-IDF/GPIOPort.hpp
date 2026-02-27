#pragma once

#include <ports/ESP-IDF/ESPCompatibility.hpp>

esp_err_t gpioConfigureInterrupt(const gpio_config_t *config);
esp_err_t gpioAddISRHandler(gpio_num_t pin, gpio_isr_t handler, void *arg);
esp_err_t gpioRemoveISRHandler(gpio_num_t pin);
int64_t gpioGetTimeUs();
