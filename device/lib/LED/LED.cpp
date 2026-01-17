#include "LED.h"
#include "freertos/FreeRTOS.h"
#include <iostream>

LED::LED(gpio_num_t pin) {
  this->pin = pin;
  this->state = false;

  gpio_reset_pin(this->pin);
  gpio_set_direction(this->pin, GPIO_MODE_OUTPUT);
  gpio_set_level(this->pin, this->state);
}

bool LED::toggle() {
  this->state = !this->state;
  gpio_set_level(this->pin, this->state);
  return this->state;
}

bool LED::getState() { return this->state; }