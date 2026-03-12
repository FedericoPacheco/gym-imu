#pragma once

#include <cstdint>
#include <functional>

class NotificationRunner {
public:
  virtual ~NotificationRunner() = default;

  // Call from inside a class with a private static method and this
  virtual bool start(std::function<void(void *, uint32_t)> loopFunction,
                     void *arg) = 0;
  virtual void runOneStep() = 0;
  virtual void notifyFromISR() = 0;
  virtual void notify() = 0;
  virtual void stop() = 0;
};