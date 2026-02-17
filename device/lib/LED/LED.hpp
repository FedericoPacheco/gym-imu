#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "soc/gpio_num.h"
#include <ErrorMacros.hpp>
#include <Logger.hpp>
#include <memory>

class LED {
public:
  static std::unique_ptr<LED> create(Logger *logger,
                                     gpio_num_t pin = GPIO_NUM_4); // D2

  bool toggle();
  bool turnOn();
  bool turnOff();

private:
  LED(Logger *logger, gpio_num_t pin);
  void checkState();

  Logger *logger;
  gpio_num_t pin;
  bool softwareState; // true = ON, false = OFF
};