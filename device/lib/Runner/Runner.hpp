#pragma once

#include <cstdint>
#include <functional>

class Runner {
public:
  virtual ~Runner() = default;

  // Call from inside the class with lambda wrapping private static method:
  // runner->start([](void *arg, uint32_t notificationValue) { loopFunction(arg,
  // notificationValue); }, this);
  virtual bool start(std::function<void(void *, uint32_t)> loopFunction,
                     void *arg) = 0;
  virtual void runOneStep() = 0;
  virtual void notifyFromISR() = 0;
  virtual void notify() = 0;
  virtual void stop() = 0;
};