#include "GPIOPort.hpp"

esp_err_t gpioConfigureInterrupt(const gpio_config_t *config) {
  return gpio_config(config);
}

esp_err_t gpioAddISRHandler(gpio_num_t pin, gpio_isr_t handler, void *arg) {
  return gpio_isr_handler_add(pin, handler, arg);
}

esp_err_t gpioRemoveISRHandler(gpio_num_t pin) {
  return gpio_isr_handler_remove(pin);
}

int64_t gpioGetTimeUs() { return esp_timer_get_time(); }
