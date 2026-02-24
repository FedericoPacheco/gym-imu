#include <FreeRTOSPort.hpp>

SemaphoreHandle_t rtosCreateMutexStatic(StaticSemaphore_t *controlBlock) {
  return xSemaphoreCreateMutexStatic(controlBlock);
}

BaseType_t rtosSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks) {
  return xSemaphoreTake(semaphore, ticks);
}

BaseType_t rtosSemaphoreGive(SemaphoreHandle_t semaphore) {
  return xSemaphoreGive(semaphore);
}

void rtosTaskEnterCritical(portMUX_TYPE *mux) { taskENTER_CRITICAL(mux); }

void rtosTaskExitCritical(portMUX_TYPE *mux) { taskEXIT_CRITICAL(mux); }

void rtosPortEnterCritical(portMUX_TYPE *mux) { portENTER_CRITICAL(mux); }

void rtosPortExitCritical(portMUX_TYPE *mux) { portEXIT_CRITICAL(mux); }

BaseType_t rtosTaskCreate(TaskFunction_t taskFn, const char *taskName,
                          configSTACK_DEPTH_TYPE stackDepth, void *taskArg,
                          UBaseType_t priority, TaskHandle_t *taskHandle) {
  return xTaskCreate(taskFn, taskName, stackDepth, taskArg, priority,
                     taskHandle);
}

uint32_t rtosTaskNotifyTake(BaseType_t clearCountOnExit,
                            TickType_t ticksToWait) {
  return ulTaskNotifyTake(clearCountOnExit, ticksToWait);
}

void rtosTaskNotifyGive(TaskHandle_t taskHandle) {
  xTaskNotifyGive(taskHandle);
}

void rtosTaskNotifyGiveFromISR(TaskHandle_t taskHandle,
                               BaseType_t *highPriorityTaskWoken) {
  vTaskNotifyGiveFromISR(taskHandle, highPriorityTaskWoken);
}

void rtosTaskDelay(TickType_t ticks) { vTaskDelay(ticks); }

void rtosTaskDelete(TaskHandle_t taskHandle) { vTaskDelete(taskHandle); }

void rtosYieldFromISR() { portYIELD_FROM_ISR(); }