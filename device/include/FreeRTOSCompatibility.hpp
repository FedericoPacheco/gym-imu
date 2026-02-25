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