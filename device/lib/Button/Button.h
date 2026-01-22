#pragma once
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

/*
A class for handling button presses with debouncing and asynchronous event
detection.

How it works:
Sync: reads pin state directly.
Async: sets up an ISR (interrupt service routine) which, after a timer expires,
loads an event into the button queue. If there are elements in the queue, the
button was indeed pressed.

How to use:
Sync:
  Button button;
  while (true) {
    if (button.isPressedSync()) {
      // Handle press
    }
  }
Async:
  Button button;
  gpio_install_isr_service(0); // Install ISR service once globally
  button.enableAsync();        // Enable async detection for this button
  while (true) {
    if (button.wasPressedAsync()) {
      // Handle press
    }
  }
  button.disableAsync(); // Optionally disable async detection when done
*/

class Button {
public:
  Button(gpio_num_t pin = GPIO_NUM_3 /* D1 */, int asyncQueueSize = 10,
         int debouncingDelayMs = 50);
  ~Button();

  bool isPressedSync();

  // Non-blocking press detection
  void enableAsync();
  void disableAsync();
  bool wasPressedAsync();

private:
  gpio_num_t pin;

  QueueHandle_t eventQueue;
  TimerHandle_t debounceTimer;
  static void IRAM_ATTR isrHandler(void *arg);
  static void debounceTimerCallback(TimerHandle_t xTimer);
};