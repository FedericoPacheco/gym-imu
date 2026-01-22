#include "Button.h"
#include "LED.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <iostream>

extern "C" void app_main() {
  LED led;

  Button button;
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  button.enableAsync();

  while (true) {
    if (button.wasPressedAsync()) {
      std::cout << "Button pressed!" << std::endl;
      led.turnOff();
    } else {
      led.turnOn();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}