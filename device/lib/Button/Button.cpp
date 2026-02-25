#include <Button.hpp>

Button::Button(LoggerPort *logger, gpio_num_t pin) : logger(logger) {
  this->pin = pin;
}

std::unique_ptr<Button> Button::create(LoggerPort *logger, gpio_num_t pin,
                                       int asyncQueueSize,
                                       int debouncingDelayMs) {

  if (logger)
    logger->info("Initializing Button on pin %d", pin);

  std::unique_ptr<Button> button(new (std::nothrow) Button(logger, pin));
  if (!button) {
    if (logger)
      logger->error("Button: Failed to create Button instance");
    return nullptr;
  }

  RETURN_NULL_ON_ERROR(gpio_reset_pin(button->pin), logger,
                       "GPIO reset failed");
  RETURN_NULL_ON_ERROR(gpio_set_direction(button->pin, GPIO_MODE_INPUT), logger,
                       "GPIO set direction failed");
  RETURN_NULL_ON_ERROR(gpio_set_pull_mode(button->pin, GPIO_PULLDOWN_ONLY),
                       logger,
                       "GPIO set pull mode failed"); // See schematic

  button->eventQueue = xQueueCreate(asyncQueueSize, sizeof(bool));
  if (button->eventQueue == NULL) {
    logger->error("Button: Failed to create event queue");
    return nullptr;
  }

  button->debounceTimer =
      xTimerCreate("debounceTimer", pdMS_TO_TICKS(debouncingDelayMs),
                   pdFALSE,      // One-shot timer, doesn't auto-restart
                   button.get(), // Timer id is pointer to this Button instance
                   debounceTimerCallback);
  if (button->debounceTimer == NULL) {
    logger->error("Button: Failed to create debounce timer");
    vQueueDelete(button->eventQueue);
    return nullptr;
  }

  logger->info("Button initialized successfully on pin: %d", pin);

  return button;
}

Button::~Button() {
  this->disableAsync();
  vQueueDelete(this->eventQueue);
  // portMAX_DELAY: block until timer is deleted
  if (xTimerDelete(this->debounceTimer, portMAX_DELAY) != pdPASS) {
    this->logger->error("Button: Failed to delete debounce timer");
  }
}

bool Button::isPressedSync() { return gpio_get_level(this->pin) == 1; }

bool Button::enableAsync() {
  RETURN_FALSE_ON_ERROR(gpio_isr_handler_add(this->pin, isrHandler, this),
                        this->logger, "Button GPIO ISR handler add failed");
  RETURN_FALSE_ON_ERROR(
      gpio_set_intr_type(this->pin, GPIO_INTR_POSEDGE), this->logger,
      "Button GPIO set interrupt type failed"); // Trigger on rising edge
  return true;
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

bool Button::disableAsync() {
  RETURN_FALSE_ON_ERROR(gpio_isr_handler_remove(this->pin), this->logger,
                        "Button GPIO ISR handler remove failed");
  RETURN_FALSE_ON_ERROR(gpio_set_intr_type(this->pin, GPIO_INTR_DISABLE),
                        this->logger, "Button GPIO disable interrupt failed");
  return true;
}

bool Button::wasPressedAsync() {
  bool event;
  return xQueueReceive(eventQueue, &event, 0) == pdTRUE; // Non-blocking check
}
