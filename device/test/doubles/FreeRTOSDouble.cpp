extern "C" {
#include <fff.h>
}

#include <ports/FreeRTOS/FreeRTOSPort.hpp>

DEFINE_FFF_GLOBALS;

DEFINE_FAKE_VALUE_FUNC(SemaphoreHandle_t, rtosCreateMutexStatic,
                       StaticSemaphore_t *);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, rtosSemaphoreTake, SemaphoreHandle_t,
                       TickType_t);
DEFINE_FAKE_VALUE_FUNC(BaseType_t, rtosSemaphoreGive, SemaphoreHandle_t);

DEFINE_FAKE_VOID_FUNC(rtosTaskEnterCritical, portMUX_TYPE *);
DEFINE_FAKE_VOID_FUNC(rtosTaskExitCritical, portMUX_TYPE *);
DEFINE_FAKE_VOID_FUNC(rtosPortEnterCritical, portMUX_TYPE *);
DEFINE_FAKE_VOID_FUNC(rtosPortExitCritical, portMUX_TYPE *);

DEFINE_FAKE_VALUE_FUNC(BaseType_t, rtosTaskCreate, TaskFunction_t, const char *,
                       configSTACK_DEPTH_TYPE, void *, UBaseType_t,
                       TaskHandle_t *);
DEFINE_FAKE_VALUE_FUNC(uint32_t, rtosTaskNotifyTake, BaseType_t, TickType_t);
DEFINE_FAKE_VOID_FUNC(rtosTaskNotifyGive, TaskHandle_t);
DEFINE_FAKE_VOID_FUNC(rtosTaskNotifyGiveFromISR, TaskHandle_t, BaseType_t *);

DEFINE_FAKE_VOID_FUNC(rtosTaskDelay, TickType_t);
DEFINE_FAKE_VOID_FUNC(rtosTaskDelete, TaskHandle_t);
DEFINE_FAKE_VOID_FUNC(rtosYieldFromISR);