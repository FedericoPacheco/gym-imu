#pragma once

#include <Runner.hpp>
#include <ports/FreeRTOS/FreeRTOSPort.hpp>
#include <string.h>

class FreeRTOSRunner : public Runner {
public:
  FreeRTOSRunner(const char *taskName, configSTACK_DEPTH_TYPE stackDepth,
                 UBaseType_t taskPriority);
  ~FreeRTOSRunner() override;

  bool start(std::function<void(void *, uint32_t)> loopFunction,
             void *argForCallback) override;
  void runOneStep() override;
  void notifyFromISR() override;
  void notify() override;
  void stop() override;

private:
  char *taskName;
  configSTACK_DEPTH_TYPE stackDepth;
  UBaseType_t taskPriority;
  TaskHandle_t taskHandle;

  std::function<void(void *, uint32_t)> storedLoopFunction;
  void *storedArgForCallback;

  static void taskEntry(void *param);
  void taskLoop();
};