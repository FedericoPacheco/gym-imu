#include <LED.hpp>

// -------------------------------------------------------------------------

LED::LED(LoggerPort *logger, gpio_num_t pin) : logger(logger) {
  this->pin = pin;
  this->softwareState = false;
}

std::unique_ptr<LED> LED::create(LoggerPort *logger, gpio_num_t pin) {
  logger->info("Initializing LED on pin %d", pin);

  std::unique_ptr<LED> led(new (std::nothrow) LED(logger, pin));

  if (!led) {
    if (logger) {
      logger->error("Failed to allocate LED");
    }
    return nullptr;
  }

  // Configure GPIO with output mode and disabled pulls
  const gpio_config_t config = {
      .pin_bit_mask = (1ULL << pin),
      .mode = GPIO_MODE_INPUT_OUTPUT, // INPUT: allow reading for state checks
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE};

  RETURN_NULL_ON_ERROR(gpio_config(&config), logger, "LED GPIO config failed");

  RETURN_NULL_ON_ERROR(gpio_set_level(led->pin, led->softwareState), logger,
                       "LED GPIO initial level set failed");

  logger->info("LED initialized successfully");

  return led;
}

bool LED::toggle() {
  this->softwareState = !this->softwareState;
  gpio_set_level(this->pin, this->softwareState);
  this->checkState();
  return this->softwareState;
}

bool LED::turnOn() {
  this->softwareState = true;
  gpio_set_level(this->pin, this->softwareState);
  this->checkState();
  return this->softwareState;
}

bool LED::turnOff() {
  this->softwareState = false;
  gpio_set_level(this->pin, this->softwareState);
  this->checkState();
  return this->softwareState;
}

void LED::checkState() {
  int hardwareState = static_cast<bool>(gpio_get_level(this->pin));
  if (hardwareState != this->softwareState) {
    this->logger->warn("LED state mismatch: software=%d, hardware=%d",
                       this->softwareState, hardwareState);
  }
}