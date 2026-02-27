#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include <ErrorMacros.hpp>
#include <LoggerPort.hpp>
#include <memory>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>

class LED {
public:
  static std::unique_ptr<LED> create(LoggerPort *logger,
                                     gpio_num_t pin = GPIO_NUM_4); // D2

  bool toggle();
  bool turnOn();
  bool turnOff();

private:
  LED(LoggerPort *logger, gpio_num_t pin);
  void checkState();

  LoggerPort *logger;
  gpio_num_t pin;
  bool softwareState; // true = ON, false = OFF
};