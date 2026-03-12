#pragma once

#include <functional>

class LoopRunner {
public:
  virtual ~LoopRunner() = default;

  // Call from inside a class with a private static method and this
  virtual bool start(std::function<void(void *)> loopFunction, void *arg) = 0;
  virtual void runOneStep() = 0;
  virtual void stop() = 0;
};
