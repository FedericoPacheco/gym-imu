#include "Greeter.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" void app_main() {
  Greeter *greeter = new Greeter();
  while (true) {
    greeter->greet();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  delete greeter;
}