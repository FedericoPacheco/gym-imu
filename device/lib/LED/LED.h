#pragma once
#include "driver/gpio.h"

class LED {
public:
  LED(gpio_num_t pin = GPIO_NUM_4); // D2
  bool toggle();
  bool turnOn();
  bool turnOff();

private:
  gpio_num_t pin;
  bool state; // true = ON, false = OFF
};