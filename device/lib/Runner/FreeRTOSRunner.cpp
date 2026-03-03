#include <FreeRTOSRunner.hpp>

FreeRTOSRunner::FreeRTOSRunner(const char *taskName,
                               configSTACK_DEPTH_TYPE stackDepth,
                               UBaseType_t taskPriority)
    : taskName(strdup(taskName)), stackDepth(stackDepth),
      taskPriority(taskPriority), taskHandle(nullptr) {}

FreeRTOSRunner::~FreeRTOSRunner() {
  if (taskName)
    free(taskName);
}

bool FreeRTOSRunner::start(std::function<void(void *, uint32_t)> loopFunction,
                           void *argForCallback) {
  if (this->taskHandle)
    return true; // Task already running

  // Indirection: store the callback and its argument on the instance and then
  // later call taskLoop() to avoid TaskFunction_t (raw C function pointer)
  // casting issues
  this->storedLoopFunction = std::move(loopFunction);
  this->storedArgForCallback = argForCallback;

  BaseType_t result = rtosTaskCreate(&FreeRTOSRunner::taskEntry, this->taskName,
                                     this->stackDepth, this, this->taskPriority,
                                     &this->taskHandle);
  if (result != pdPASS) {
    this->taskHandle = nullptr; // Task creation failed
    return false;
  }
  return true;
}

void FreeRTOSRunner::taskEntry(void *param) {
  auto runner = static_cast<FreeRTOSRunner *>(param);
  runner->taskLoop();
}

void FreeRTOSRunner::taskLoop() {
  while (true) {
    uint32_t notificationValue = rtosTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (this->storedLoopFunction)
      this->storedLoopFunction(this->storedArgForCallback, notificationValue);
  }
}

void FreeRTOSRunner::runOneStep() {
  // Not applicable
}

void FreeRTOSRunner::notifyFromISR() {
  if (this->taskHandle) {
    BaseType_t highPriorityTaskWoken = pdFALSE;
    rtosTaskNotifyGiveFromISR(this->taskHandle, &highPriorityTaskWoken);
    if (highPriorityTaskWoken == pdTRUE)
      rtosYieldFromISR(); // Context switch if needed
  }
}

void FreeRTOSRunner::notify() {
  if (this->taskHandle) {
    rtosTaskNotifyGive(this->taskHandle);
  }
}

void FreeRTOSRunner::stop() {
  // Notify task to unblock and finish cleanly
  if (this->taskHandle) {
    this->notify();
    rtosTaskDelay(pdMS_TO_TICKS(100)); // Give time for task to finish
    rtosTaskDelete(this->taskHandle);
    this->taskHandle = nullptr;
  }
}
