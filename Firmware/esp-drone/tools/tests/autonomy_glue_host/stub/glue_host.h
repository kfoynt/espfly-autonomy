/*
 * Host stubs for the autonomy glue tests (tools/tests/test_autonomy_glue_host.c,
 * tools/tests/test_cam_vz_glue_host.c): the ESP-IDF, FreeRTOS and drone
 * declarations autonomy.c and cam_vz_glue.c include that are not host-clean.
 * Every stub header in this directory is a one-line #include of this file;
 * the host-clean drone headers (stabilizer.h, camera_flow.h, sensfusion6.h,
 * motor_guard.h, flight_rtc_log.h, cam_vz_glue.h, autonomy.h, ...) are the
 * real ones. The declarations copy the real headers' signatures; the test
 * files define them.
 */
#ifndef GLUE_HOST_H_
#define GLUE_HOST_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* ---- FreeRTOS ---- */
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
void glueHostLock(portMUX_TYPE *m);
void glueHostUnlock(portMUX_TYPE *m);
#define portENTER_CRITICAL(m) glueHostLock(m)
#define portEXIT_CRITICAL(m) glueHostUnlock(m)
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef uint8_t StackType_t;
typedef struct { int dummy; } StaticTask_t;
typedef void *TaskHandle_t;
#define pdTRUE 1
#define portTICK_PERIOD_MS 1
TickType_t xTaskGetTickCount(void);
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t t);
void vTaskDelay(TickType_t t);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait);
void xTaskNotifyGive(TaskHandle_t t);
TaskHandle_t xTaskCreateStaticPinnedToCore(void (*fn)(void *), const char *name, uint32_t depth, void *arg,
                                           int pri, StackType_t *stack, StaticTask_t *tcb, int core);

/* ---- stm32_legacy.h ---- */
#define M2T(X) ((unsigned int)(X) / portTICK_PERIOD_MS)
#define T2M(X) ((unsigned int)(X) * portTICK_PERIOD_MS)

/* ---- ESP-IDF ---- */
#define IRAM_ATTR
#define RTC_NOINIT_ATTR
int64_t esp_timer_get_time(void);
uint32_t esp_random(void);
#define MALLOC_CAP_8BIT     (1u << 2)
#define MALLOC_CAP_SPIRAM   (1u << 10)
#define MALLOC_CAP_INTERNAL (1u << 11)
size_t heap_caps_get_free_size(uint32_t caps);
void *heap_caps_malloc(size_t size, uint32_t caps);
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NVS_NOT_FOUND 0x1102
#define ESP_ERR_NVS_TYPE_MISMATCH 0x1103
const char *esp_err_to_name(esp_err_t e);
typedef enum {
  ESP_RST_UNKNOWN, ESP_RST_POWERON, ESP_RST_EXT, ESP_RST_SW, ESP_RST_PANIC,
} esp_reset_reason_t;
esp_reset_reason_t esp_reset_reason(void);

/* ---- debug_cf.h ---- */
extern bool glueHostQuiet;
#define DEBUG_PRINTI(...) do { if (!glueHostQuiet) { printf("  [I] " __VA_ARGS__); printf("\n"); } } while (0)
#define DEBUG_PRINTW(...) do { if (!glueHostQuiet) { printf("  [W] " __VA_ARGS__); printf("\n"); } } while (0)

/* ---- ledseq.h ---- */
typedef struct ledseqContext_s {
  int state;
} ledseqContext_t;
extern ledseqContext_t seq_autoPrespin;
bool ledseqRun(ledseqContext_t *context);
bool ledseqStop(ledseqContext_t *context);

/* ---- motors.h ---- */
void motorsEmergencyStopNoWait(void);
bool motorsAnyDutyNonZero(void);
bool motorsReadyNoLock(void);
bool motorsIsInhibited(void);

/* ---- pm_esplane.h ---- */
float pmGetBatteryVoltage(void);

/* ---- crtp_commander.h ---- */
void crtpCommanderRpytLockThrust(void);

/* ---- imu_level_cal_store.h (the autonomy settings store) ---- */
esp_err_t autoSettingStoreLoad(const char *key, uint8_t nvsType, int32_t *value);
esp_err_t autoSettingStoreSave(const char *key, uint8_t nvsType, int32_t value);
esp_err_t autoSettingStoreErase(const char *key);

#endif /* GLUE_HOST_H_ */
