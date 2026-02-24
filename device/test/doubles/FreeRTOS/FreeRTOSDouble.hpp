#pragma once

extern "C" {
#include <fff.h>
}

#include <FreeRTOSPort.hpp>

DECLARE_FAKE_VALUE_FUNC(SemaphoreHandle_t, rtosCreateMutexStatic,
                        StaticSemaphore_t *);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, rtosSemaphoreTake, SemaphoreHandle_t,
                        TickType_t);
DECLARE_FAKE_VALUE_FUNC(BaseType_t, rtosSemaphoreGive, SemaphoreHandle_t);

DECLARE_FAKE_VOID_FUNC(rtosTaskEnterCritical, portMUX_TYPE *);
DECLARE_FAKE_VOID_FUNC(rtosTaskExitCritical, portMUX_TYPE *);
DECLARE_FAKE_VOID_FUNC(rtosPortEnterCritical, portMUX_TYPE *);
DECLARE_FAKE_VOID_FUNC(rtosPortExitCritical, portMUX_TYPE *);

DECLARE_FAKE_VALUE_FUNC(BaseType_t, rtosTaskCreate, TaskFunction_t,
                        const char *, configSTACK_DEPTH_TYPE, void *,
                        UBaseType_t, TaskHandle_t *);
DECLARE_FAKE_VALUE_FUNC(uint32_t, rtosTaskNotifyTake, BaseType_t, TickType_t);
DECLARE_FAKE_VOID_FUNC(rtosTaskNotifyGive, TaskHandle_t);
DECLARE_FAKE_VOID_FUNC(rtosTaskNotifyGiveFromISR, TaskHandle_t, BaseType_t *);

DECLARE_FAKE_VOID_FUNC(rtosTaskDelay, TickType_t);
DECLARE_FAKE_VOID_FUNC(rtosTaskDelete, TaskHandle_t);
DECLARE_FAKE_VOID_FUNC(rtosYieldFromISR);

inline void resetFreeRTOSPortFakes() {
  RESET_FAKE(rtosCreateMutexStatic);
  RESET_FAKE(rtosSemaphoreTake);
  RESET_FAKE(rtosSemaphoreGive);
  RESET_FAKE(rtosTaskEnterCritical);
  RESET_FAKE(rtosTaskExitCritical);
  RESET_FAKE(rtosPortEnterCritical);
  RESET_FAKE(rtosPortExitCritical);
  RESET_FAKE(rtosTaskCreate);
  RESET_FAKE(rtosTaskNotifyTake);
  RESET_FAKE(rtosTaskNotifyGive);
  RESET_FAKE(rtosTaskNotifyGiveFromISR);
  RESET_FAKE(rtosTaskDelay);
  RESET_FAKE(rtosTaskDelete);
  RESET_FAKE(rtosYieldFromISR);
  FFF_RESET_HISTORY();
}