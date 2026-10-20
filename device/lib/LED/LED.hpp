#pragma once
#include "LoopRunner.hpp"
#include <ErrorMacros.hpp>
#include <LoggerPort.hpp>
#include <LoopRunner.hpp>
#include <atomic>
#include <memory>
#include <ports/ESP-IDF/ESPIDFPort.hpp>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>

class LED {
public:
  enum class State { OFF, ON, BLINKING };

  static std::unique_ptr<LED> create(std::unique_ptr<LoopRunner> runner,
                                     LoggerPort *logger,
                                     gpio_num_t pin = GPIO_NUM_9);
  ~LED();

  void turnOn();
  void turnOff();
  void blink();

private:
  static constexpr int BLINKING_TURN_OFF_TIME_MS = 250;
  static constexpr int BLINKING_TURN_ON_TIME_MS = 250;

  std::unique_ptr<LoopRunner> runner;
  LoggerPort *logger;
  gpio_num_t pin;
  std::atomic<State> state{State::OFF};

  LED(std::unique_ptr<LoopRunner> runner, LoggerPort *logger, gpio_num_t pin);
  static void loopFunction(void *arg);
};