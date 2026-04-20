#pragma once
#include <ErrorMacros.hpp>
#include <LoggerPort.hpp>
#include <memory>
#include <ports/ESP-IDF/ESPIDFPort.hpp>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>
class LED {
public:
  static std::unique_ptr<LED> create(LoggerPort *logger,
                                     gpio_num_t pin = GPIO_NUM_9);

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