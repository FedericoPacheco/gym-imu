#pragma once

#include <ports/ESP-IDF/ESPIDFCompatibility.hpp>

esp_err_t gpioSetConfig(const gpio_config_t *config);
esp_err_t gpioAddISRHandler(gpio_num_t pin, gpio_isr_t handler, void *arg);
esp_err_t gpioRemoveISRHandler(gpio_num_t pin);
esp_err_t nvsFlashInit();
esp_err_t nvsFlashErase();
int64_t getTimeUs();
