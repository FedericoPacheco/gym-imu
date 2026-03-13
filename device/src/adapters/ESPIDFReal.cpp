#include "ports/ESP-IDF/ESPIDFPort.hpp"

esp_err_t gpioSetConfig(const gpio_config_t *config) {
  return gpio_config(config);
}

esp_err_t gpioAddISRHandler(gpio_num_t pin, gpio_isr_t handler, void *arg) {
  return gpio_isr_handler_add(pin, handler, arg);
}

esp_err_t gpioRemoveISRHandler(gpio_num_t pin) {
  return gpio_isr_handler_remove(pin);
}

esp_err_t nvsFlashInit() { return nvs_flash_init(); }

esp_err_t nvsFlashErase() { return nvs_flash_erase(); }

int64_t getTimeUs() { return esp_timer_get_time(); }
