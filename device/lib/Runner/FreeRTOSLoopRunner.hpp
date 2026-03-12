#pragma once

#include <LoopRunner.hpp>
#include <atomic>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>
#include <string.h>

class FreeRTOSLoopRunner : public LoopRunner {
public:
  FreeRTOSLoopRunner(const char *taskName, configSTACK_DEPTH_TYPE stackDepth,
                     UBaseType_t taskPriority, TickType_t waitTicks);
  ~FreeRTOSLoopRunner() override;

  bool start(std::function<void(void *)> loopFunction,
             void *argForCallback) override;
  void runOneStep() override;
  void stop() override;

private:
  char *taskName;
  configSTACK_DEPTH_TYPE stackDepth;
  UBaseType_t taskPriority;
  TickType_t waitTicks;
  TaskHandle_t taskHandle;
  std::atomic<bool> isRunning;

  std::function<void(void *)> storedLoopFunction;
  void *storedArgForCallback;

  static void taskEntry(void *param);
  void taskLoop();
};
