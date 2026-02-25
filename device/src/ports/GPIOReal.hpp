#pragma once

#include <GPIOPort.hpp>

#include "driver/gpio.h"

class GPIOReal : public GPIOPort {
public:
  ~GPIOReal() override = default;

  esp_err_t configureInterrupt(const GPIOInputInterruptConfig &config) override;
  esp_err_t addISRHandler(gpio_num_t pin, void (*handler)(void *),
                          void *arg) override;
  esp_err_t removeISRHandler(gpio_num_t pin) override;
  int64_t getTimeUs() override;
};
