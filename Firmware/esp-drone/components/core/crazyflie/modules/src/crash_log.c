/*
 * ESP-Drone Firmware
 *
 * crash_log.c - Post-mortem record that survives a panic / software reset
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_debug_helpers.h"
#include "esp_cpu_utils.h"
#include "esp_rom_sys.h"
#include "esp_private/panic_internal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/xtensa_context.h"

#include "crash_log.h"
#include "motor_guard.h"

#define CRASH_MAGIC 0xC4A5B10Eu   /* bump when crashStore_t changes */

typedef struct {
  uint32_t magic;
  crashInfo_t info;
  uint32_t ringHead;               /* total bytes ever written */
  char ring[CRASH_LOG_RING_LEN];
  uint32_t uptimeS;                /* updated ~1 Hz by crashLogTick() */
  uint32_t bootCount;              /* boots since the record was created */
  crashBoot_t boots[CRASH_LOG_BOOT_HISTORY];  /* ring, index bootCount-1 is newest */
} crashStore_t;

static RTC_NOINIT_ATTR crashStore_t store;
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t prevVprintf;

extern bool g_panic_abort;
extern void __real_esp_panic_handler(panic_info_t *info);
extern void __real_esp_system_abort(const char *details) __attribute__((noreturn));

/* Runs inside the panic handler: IRAM only, no locks, no flash access. The
 * motor pins go low first: the LEDC keeps the last duty while the handler
 * prints and reboots (exceptions, asserts, the interrupt watchdog). */
void IRAM_ATTR __wrap_esp_panic_handler(panic_info_t *info)
{
  motorGuardPanicPinsLow();
  if (store.magic == CRASH_MAGIC) {
    crashInfo_t *c = &store.info;
    const XtExcFrame *f = (const XtExcFrame *)info->frame;

    c->valid = 1;
    c->bootsSince = 0;
    c->core = info->core;
    c->exception = info->exception;
    c->pc = (uint32_t)info->addr;
    c->exccause = f ? f->exccause : 0;
    c->excvaddr = f ? f->excvaddr : 0;
    if (!g_panic_abort) {
      c->details[0] = '\0';
    }
    c->depth = 0;
    if (f) {
      esp_backtrace_frame_t fr = {.pc = f->pc, .sp = f->a1, .next_pc = f->a0, .exc_frame = f};
      c->backtrace[c->depth++] = esp_cpu_process_stack_pc(fr.pc);
      while (c->depth < CRASH_LOG_BACKTRACE_DEPTH && fr.next_pc != 0) {
        if (!esp_backtrace_get_next_frame(&fr)) {
          break;
        }
        c->backtrace[c->depth++] = esp_cpu_process_stack_pc(fr.pc);
      }
    }
  }
  __real_esp_panic_handler(info);
}

/* abort() and failed asserts arrive here with their message first. */
void IRAM_ATTR __attribute__((noreturn)) __wrap_esp_system_abort(const char *details)
{
  motorGuardPanicPinsLow();
  if (store.magic == CRASH_MAGIC) {
    int i = 0;
    if (details) {
      for (; i < CRASH_LOG_DETAILS_LEN - 1 && details[i]; i++) {
        store.info.details[i] = details[i];
      }
    }
    store.info.details[i] = '\0';
  }
  __real_esp_system_abort(details);
}

/* ESP_LOG hook: keep warning/error lines, then pass everything on. */
static int logHook(const char *fmt, va_list ap)
{
  const char *p = fmt;
  if (p[0] == '\033') {
    const char *m = strchr(p, 'm');
    if (m) {
      p = m + 1;
    }
  }
  if ((p[0] == 'E' || p[0] == 'W') && p[1] == ' ') {
    char line[112];
    va_list cp;
    va_copy(cp, ap);
    int n = vsnprintf(line, sizeof(line), fmt, cp);
    va_end(cp);
    if (n > (int)sizeof(line) - 1) {
      n = sizeof(line) - 1;
    }
    if (n > 0) {
      portENTER_CRITICAL(&ringMux);
      for (int i = 0; i < n; i++) {
        if (line[i] == '\033') {            /* drop colour codes */
          while (i < n && line[i] != 'm') {
            i++;
          }
          continue;
        }
        if (line[i] == '\r' || line[i] == '\n') {
          continue;
        }
        store.ring[store.ringHead++ % CRASH_LOG_RING_LEN] = line[i];
      }
      store.ring[store.ringHead++ % CRASH_LOG_RING_LEN] = '\n';
      portEXIT_CRITICAL(&ringMux);
    }
  }
  return prevVprintf ? prevVprintf(fmt, ap) : vprintf(fmt, ap);
}

void crashLogNote(const char *text)
{
  if (store.magic != CRASH_MAGIC || text == NULL) {
    return;
  }
  portENTER_CRITICAL(&ringMux);
  for (int i = 0; i < CRASH_LOG_NOTE_MAX && text[i]; i++) {
    if (text[i] != '\r' && text[i] != '\n') {
      store.ring[store.ringHead++ % CRASH_LOG_RING_LEN] = text[i];
    }
  }
  store.ring[store.ringHead++ % CRASH_LOG_RING_LEN] = '\n';
  portEXIT_CRITICAL(&ringMux);
}

void crashLogInit(void)
{
  static bool isInit;
  if (isInit) {
    return;
  }
  uint32_t prevUptime = 0;
  if (store.magic != CRASH_MAGIC || store.info.depth > CRASH_LOG_BACKTRACE_DEPTH) {
    memset(&store, 0, sizeof(store));   /* power-on: RTC no-init memory is garbage */
    store.magic = CRASH_MAGIC;
  } else {
    prevUptime = store.uptimeS;
    if (store.info.valid) {
      store.info.bootsSince++;
    }
  }
  crashBoot_t *b = &store.boots[store.bootCount % CRASH_LOG_BOOT_HISTORY];
  b->resetReason = esp_reset_reason();
  b->prevUptimeS = prevUptime;
  b->rawReason[0] = (uint8_t)esp_rom_get_reset_reason(0);
  b->rawReason[1] = (uint8_t)esp_rom_get_reset_reason(1);
  store.bootCount++;
  store.uptimeS = 0;
  store.info.details[CRASH_LOG_DETAILS_LEN - 1] = '\0';
  prevVprintf = esp_log_set_vprintf(logHook);
  isInit = true;
}

void crashLogTick(uint32_t uptimeS)
{
  store.uptimeS = uptimeS;
}

int crashLogBoots(crashBoot_t *out, int max, uint32_t *totalBoots)
{
  *totalBoots = store.bootCount;
  int n = 0;
  for (uint32_t i = 0; i < CRASH_LOG_BOOT_HISTORY && i < store.bootCount && n < max; i++) {
    out[n++] = store.boots[(store.bootCount - 1 - i) % CRASH_LOG_BOOT_HISTORY];
  }
  return n;
}

bool crashLogGet(crashInfo_t *out)
{
  if (store.magic != CRASH_MAGIC) {
    return false;
  }
  *out = store.info;
  return store.info.valid != 0;
}

void crashLogClear(void)
{
  portENTER_CRITICAL(&ringMux);
  memset(&store.info, 0, sizeof(store.info));
  store.ringHead = 0;
  portEXIT_CRITICAL(&ringMux);
}

void crashLogGetLines(char *buf, int len)
{
  if (len <= 0) {
    return;
  }
  int k = 0;
  portENTER_CRITICAL(&ringMux);
  uint32_t head = store.ringHead;
  uint32_t n = head < CRASH_LOG_RING_LEN ? head : CRASH_LOG_RING_LEN;
  for (uint32_t i = head - n; i < head && k < len - 1; i++) {
    buf[k++] = store.ring[i % CRASH_LOG_RING_LEN];
  }
  portEXIT_CRITICAL(&ringMux);
  buf[k] = '\0';
}
