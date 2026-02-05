#pragma once
#include "Logger.hpp"
#include "driver/gpio.h"
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