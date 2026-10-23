#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef uint8_t StackType_t;   /* ESP-IDF: stack depths are in bytes */
typedef struct { uint8_t opaque[64]; } StaticTask_t;
#define pdPASS 1
#define portTICK_PERIOD_MS 1
void vTaskDelay(TickType_t t);
BaseType_t xTaskCreatePinnedToCore(void (*fn)(void *), const char *name, uint32_t stack, void *arg, int pri, TaskHandle_t *h, int core);
TaskHandle_t xTaskCreateStaticPinnedToCore(void (*fn)(void *), const char *name, uint32_t stack, void *arg, int pri,
                                           StackType_t *stackBuf, StaticTask_t *tcb, int core);
static inline uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t h) { (void)h; return 4000; }
