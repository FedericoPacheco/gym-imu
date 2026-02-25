#pragma once

#include <GPIOPort.hpp>
#include <gmock/gmock.h>

class GPIODouble : public GPIOPort {
public:
  MOCK_METHOD(esp_err_t, configureInterrupt,
              (const GPIOInputInterruptConfig &config), (override));
  MOCK_METHOD(esp_err_t, addISRHandler,
              (gpio_num_t pin, void (*handler)(void *), void *arg), (override));
  MOCK_METHOD(esp_err_t, removeISRHandler, (gpio_num_t pin), (override));
  MOCK_METHOD(int64_t, getTimeUs, (), (override));
};
