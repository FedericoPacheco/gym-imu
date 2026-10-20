#include <LED.hpp>

LED::LED(std::unique_ptr<LoopRunner> runner, LoggerPort *logger, gpio_num_t pin)
    : runner(std::move(runner)), logger(logger), pin(pin) {}

std::unique_ptr<LED> LED::create(std::unique_ptr<LoopRunner> runner,
                                 LoggerPort *logger, gpio_num_t pin) {

  if (!logger) {
    return nullptr;
  }

  logger->info("Initializing LED on pin %d", pin);
  if (!runner) {
    logger->error("Invalid runner");
    return nullptr;
  }

  // Create instance
  logger->debug("Creating LED instance");
  std::unique_ptr<LED> led(new (std::nothrow)
                               LED(std::move(runner), logger, pin));
  if (!led) {
    logger->error("Failed to allocate LED");
    return nullptr;
  }

  // Configure GPIO with output mode and disabled pulls
  logger->debug("Configuring LED GPIO");
  const gpio_config_t config = {
      .pin_bit_mask = (1ULL << pin),
      .mode = GPIO_MODE_INPUT_OUTPUT, // INPUT: allow reading for state checks
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE};

  RETURN_NULL_ON_ESP_ERROR(gpio_config(&config), logger,
                           "LED GPIO config failed");

  RETURN_NULL_ON_ESP_ERROR(gpio_set_level(led->pin, 0), logger,
                           "LED GPIO initial level set failed");

  logger->debug("Starting LED task for instance %p", led.get());
  if (!led->runner->start(LED::loopFunction, static_cast<void *>(led.get()))) {
    logger->error("Failed to start LED task");
    return nullptr;
  }

  logger->info("LED initialized successfully");

  return led;
}

LED::~LED() {
  gpio_set_level(this->pin, 0);
  this->runner->stop();
}

void LED::loopFunction(void *arg) {
  LED *self = static_cast<LED *>(arg);

  if (!self) {
    return;
  }

  int oldState = gpio_get_level(self->pin);
  State newState = self->state.load();
  switch (newState) {
  case State::OFF:
    if (oldState == 1)
      gpio_set_level(self->pin, 0);
    break;
  case State::ON:
    if (oldState == 0)
      gpio_set_level(self->pin, 1);
    break;
  case State::BLINKING:
    if (oldState == 0) {
      gpio_set_level(self->pin, 1);
      rtosTaskDelay(pdMS_TO_TICKS(BLINKING_TURN_OFF_TIME_MS));
    }
    gpio_set_level(self->pin, 0);
    rtosTaskDelay(pdMS_TO_TICKS(BLINKING_TURN_ON_TIME_MS));
    break;
  }
}

void LED::turnOn() { this->state.store(State::ON); }
void LED::turnOff() { this->state.store(State::OFF); }
void LED::blink() { this->state.store(State::BLINKING); }
