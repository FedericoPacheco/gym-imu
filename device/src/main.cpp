#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <iostream>

class Greeter {
public:
  void greet() { std::cout << "Hello world from XIAO ESP32 C3!" << std::endl; }
};

extern "C" void app_main() {
  Greeter *greeter = new Greeter();
  while (true) {
    greeter->greet();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  delete greeter;
}