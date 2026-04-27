#include <FreeRTOSNotificationRunner.hpp>

FreeRTOSNotificationRunner::FreeRTOSNotificationRunner(
    const char *taskName, configSTACK_DEPTH_TYPE stackDepth,
    UBaseType_t taskPriority)
    : taskName(strdup(taskName)), stackDepth(stackDepth),
      taskPriority(taskPriority), taskHandle(nullptr) {}

FreeRTOSNotificationRunner::~FreeRTOSNotificationRunner() {
  if (taskName)
    free(taskName);
}

bool FreeRTOSNotificationRunner::start(
    std::function<void(void *, uint32_t)> loopFunction, void *argForCallback) {
  if (this->taskHandle)
    return true; // Task already running

  // Indirection: store the callback and its argument on the instance and then
  // later call taskLoop() to avoid TaskFunction_t (raw C function pointer)
  // casting issues
  this->storedLoopFunction = std::move(loopFunction);
  this->storedArgForCallback = argForCallback;

  BaseType_t result = rtosTaskCreate(&FreeRTOSNotificationRunner::taskEntry,
                                     this->taskName, this->stackDepth, this,
                                     this->taskPriority, &this->taskHandle);
  if (result != pdPASS) {
    this->taskHandle = nullptr; // Task creation failed
    return false;
  }
  return true;
}

void FreeRTOSNotificationRunner::taskEntry(void *param) {
  auto runner = static_cast<FreeRTOSNotificationRunner *>(param);
  runner->taskLoop();
}

void FreeRTOSNotificationRunner::taskLoop() {
  while (true) {
    uint32_t notificationValue = rtosTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (this->storedLoopFunction)
      this->storedLoopFunction(this->storedArgForCallback, notificationValue);
  }
}

void FreeRTOSNotificationRunner::runOneStep() {
  // Not applicable
}

void IRAM_ATTR FreeRTOSNotificationRunner::notifyFromISR() {
  if (this->taskHandle) {
    BaseType_t highPriorityTaskWoken = pdFALSE;
    rtosTaskNotifyGiveFromISR(this->taskHandle, &highPriorityTaskWoken);
    if (highPriorityTaskWoken == pdTRUE)
      rtosYieldFromISR(); // Context switch if needed
  }
}

void FreeRTOSNotificationRunner::notify() {
  if (this->taskHandle) {
    rtosTaskNotifyGive(this->taskHandle);
  }
}

void FreeRTOSNotificationRunner::stop() {
  // Notify task to unblock and finish cleanly
  if (this->taskHandle) {
    this->notify();
    rtosTaskDelay(pdMS_TO_TICKS(100)); // Give time for task to finish
    rtosTaskDelete(this->taskHandle);
    this->taskHandle = nullptr;
  }
}
