#pragma once

#if defined(UNIT_TEST) && !defined(ESP_PLATFORM)

#include <cstdint>

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
typedef uint32_t configSTACK_DEPTH_TYPE;
typedef void (*TaskFunction_t)(void *);

typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;

struct StaticSemaphore_t {
  uint8_t storage;
};

struct portMUX_TYPE {
  uint8_t storage;
};

static constexpr BaseType_t pdTRUE = 1;
static constexpr BaseType_t pdFALSE = 0;
static constexpr BaseType_t pdPASS = 1;
static constexpr TickType_t portMAX_DELAY = 0xFFFFFFFFu;

#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(x) (x)
#endif

#else

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/portmacro.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#endif

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