#include "LED.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <iostream>

extern "C" void app_main() {
  LED led;
  while (true) {
    led.toggle();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}