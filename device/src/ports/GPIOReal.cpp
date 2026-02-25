#include "GPIOReal.hpp"

#include "esp_timer.h"

esp_err_t GPIOReal::configureInterrupt(const GPIOInputInterruptConfig &config) {
  const gpio_mode_t mode =
      config.mode == GPIOPortMode::Input ? GPIO_MODE_INPUT : GPIO_MODE_DISABLE;
  const gpio_pulldown_t pullDown = config.pullDown == GPIOPortPullDown::Enable
                                       ? GPIO_PULLDOWN_ENABLE
                                       : GPIO_PULLDOWN_DISABLE;
  const gpio_int_type_t intrType =
      config.interruptType == GPIOPortInterruptType::Posedge
          ? GPIO_INTR_POSEDGE
          : GPIO_INTR_DISABLE;

  const gpio_config_t pinConfig{.pin_bit_mask = config.pinBitMask,
                                .mode = mode,
                                .pull_up_en = config.pullUp,
                                .pull_down_en = pullDown,
                                .intr_type = intrType};
  return gpio_config(&pinConfig);
}

esp_err_t GPIOReal::addISRHandler(gpio_num_t pin, void (*handler)(void *),
                                  void *arg) {
  return gpio_isr_handler_add(pin, handler, arg);
}

esp_err_t GPIOReal::removeISRHandler(gpio_num_t pin) {
  return gpio_isr_handler_remove(pin);
}

int64_t GPIOReal::getTimeUs() { return esp_timer_get_time(); }
