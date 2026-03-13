#include <FreeRTOSLoopRunner.hpp>
#include <utility>

FreeRTOSLoopRunner::FreeRTOSLoopRunner(const char *taskName,
                                       configSTACK_DEPTH_TYPE stackDepth,
                                       UBaseType_t taskPriority,
                                       TickType_t waitTicks)
    : taskName(strdup(taskName)), stackDepth(stackDepth),
      taskPriority(taskPriority), waitTicks(waitTicks), taskHandle(nullptr),
      isRunning(false), storedLoopFunction(nullptr),
      storedArgForCallback(nullptr) {}

FreeRTOSLoopRunner::~FreeRTOSLoopRunner() {
  this->stop();

  if (this->taskName)
    free(this->taskName);
}

bool FreeRTOSLoopRunner::start(std::function<void(void *)> loopFunction,
                               void *argForCallback) {
  if (this->taskHandle)
    return true; // Task already running

  // Indirection: store callback and argument in the instance to avoid raw C
  // function pointer casts from TaskFunction_t.
  this->storedLoopFunction = std::move(loopFunction);
  this->storedArgForCallback = argForCallback;
  this->isRunning.store(true, std::memory_order_relaxed);

  BaseType_t result = rtosTaskCreate(&FreeRTOSLoopRunner::taskEntry,
                                     this->taskName, this->stackDepth, this,
                                     this->taskPriority, &this->taskHandle);
  if (result != pdPASS) {
    this->taskHandle = nullptr;
    this->isRunning.store(false, std::memory_order_relaxed);
    return false;
  }
  return true;
}

void FreeRTOSLoopRunner::taskEntry(void *param) {
  auto runner = static_cast<FreeRTOSLoopRunner *>(param);
  runner->taskLoop();
}

void FreeRTOSLoopRunner::taskLoop() {
  while (this->isRunning.load(std::memory_order_relaxed)) {
    if (this->storedLoopFunction)
      this->storedLoopFunction(this->storedArgForCallback);

    // Avoid watchdog's false positives. Block with a delay to allow other tasks
    // to be scheduled
    if (this->waitTicks > 0)
      rtosTaskDelay(this->waitTicks);
  }
}

void FreeRTOSLoopRunner::runOneStep() {
  // Not applicable
}

void FreeRTOSLoopRunner::stop() {
  if (this->taskHandle) {
    this->isRunning.store(false, std::memory_order_relaxed);
    rtosTaskDelay(pdMS_TO_TICKS(100));
    rtosTaskDelete(this->taskHandle);
    this->taskHandle = nullptr;
  }

  this->storedLoopFunction = nullptr;
  this->storedArgForCallback = nullptr;
}
