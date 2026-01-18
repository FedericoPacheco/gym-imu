#include "Button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Button::Button(gpio_num_t pin, int asyncQueueSize, int debouncingDelayMs) {
  this->pin = pin;

  gpio_reset_pin(this->pin);
  gpio_set_direction(this->pin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(this->pin, GPIO_PULLDOWN_ONLY); // See schematic

  this->eventQueue = xQueueCreate(asyncQueueSize, sizeof(bool));
  this->debounceTimer =
      xTimerCreate("debounceTimer", pdMS_TO_TICKS(debouncingDelayMs),
                   pdFALSE, // One-shot timer, doesn't auto-restart
                   this,    // Timer id is pointer to this Button instance
                   debounceTimerCallback);
}

Button::~Button() {
  this->disableAsync();
  vQueueDelete(eventQueue);
  // portMAX_DELAY: block until timer is deleted
  xTimerDelete(debounceTimer, portMAX_DELAY);
}

bool Button::isPressedSync() { return gpio_get_level(this->pin) == 1; }

void Button::enableAsync() {
  gpio_isr_handler_add(this->pin, isrHandler, this);
  gpio_set_intr_type(this->pin, GPIO_INTR_POSEDGE); // Trigger on rising edge
}
void IRAM_ATTR Button::isrHandler(void *arg) {
  Button *btn = (Button *)arg;
  xTimerStartFromISR(btn->debounceTimer, NULL);
}
void Button::debounceTimerCallback(TimerHandle_t xTimer) {
  Button *btn = (Button *)pvTimerGetTimerID(xTimer); // Retrieve Button instance
  bool event = true;
  xQueueSend(btn->eventQueue, &event,
             0); // Send from task context. 0: ignore higher priority tasks
}

void Button::disableAsync() {
  gpio_isr_handler_remove(this->pin);
  gpio_set_intr_type(this->pin, GPIO_INTR_DISABLE);
}

bool Button::wasPressedAsync() {
  bool event;
  return xQueueReceive(eventQueue, &event, 0) == pdTRUE; // Non-blocking check
}
