#pragma once

#include <ports/FreeRTOS/FreeRTOSCompatibility.hpp>

SemaphoreHandle_t rtosCreateMutexStatic(StaticSemaphore_t *controlBlock);
BaseType_t rtosSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks);
BaseType_t rtosSemaphoreGive(SemaphoreHandle_t semaphore);

void rtosTaskEnterCritical(portMUX_TYPE *mux);
void rtosTaskExitCritical(portMUX_TYPE *mux);
void rtosPortEnterCritical(portMUX_TYPE *mux);
void rtosPortExitCritical(portMUX_TYPE *mux);

BaseType_t rtosTaskCreate(TaskFunction_t taskFn, const char *taskName,
                          configSTACK_DEPTH_TYPE stackDepth, void *taskArg,
                          UBaseType_t priority, TaskHandle_t *taskHandle);
uint32_t rtosTaskNotifyTake(BaseType_t clearCountOnExit,
                            TickType_t ticksToWait);
void rtosTaskNotifyGive(TaskHandle_t taskHandle);
void rtosTaskNotifyGiveFromISR(TaskHandle_t taskHandle,
                               BaseType_t *highPriorityTaskWoken);

void rtosTaskDelay(TickType_t ticks);
void rtosTaskDelete(TaskHandle_t taskHandle);
void rtosYieldFromISR();