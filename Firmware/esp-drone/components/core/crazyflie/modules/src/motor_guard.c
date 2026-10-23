/*
 * ESP-Drone Firmware
 *
 * motor_guard.c - Independent motor watchdog (motor_guard.h,
 * docs/autonomy-a2.md section 7): an esp_timer outside the stabilizer stops
 * the motors when the stabilizer makes no progress while they are driven.
 *
 * The callback does only non-blocking work: a spinlock around the pure
 * decision (motor_guard_core.c), the driver's no-wait STOP, one line in the
 * RTC crash-log ring and a no-wait LED request. It never prints (an ESP_LOG
 * can spin up to 50 ms on a full USB-Serial-JTAG FIFO, and the esp_timer
 * stack is 3.5 KB); the trip shows in 'mguard', 'status' and 'crash' on the
 * USB console, and to the pilot as the red restart-lock blink (same remedy:
 * replug the battery).
 */
#include "motor_guard.h"

#include "freertos/FreeRTOS.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "motors.h"
#include "crash_log.h"
#include "ledseq.h"
#define DEBUG_MODULE "MGUARD"
#include "debug_cf.h"

static motorGuardCore_t core;
static portMUX_TYPE coreMux = portMUX_INITIALIZER_UNLOCKED;
static esp_timer_handle_t timer;
static volatile bool running;
static volatile bool enabledSwitch = true;
/* Stabilizer loops; its only writer is the stabilizer task, so a plain
 * increment of an aligned word is enough for the reader on the other core. */
static volatile uint32_t aliveCount;
static uint32_t callbacks;
static volatile uint32_t tripUptimeMs;

static char *appendText(char *p, const char *end, const char *s)
{
  while (*s && p < end) {
    *p++ = *s++;
  }
  return p;
}

/* Decimal without printf: the callback runs on the esp_timer task's stack. */
static char *appendU32(char *p, const char *end, uint32_t v)
{
  char digits[10];
  int n = 0;
  do {
    digits[n++] = (char)('0' + v % 10);
    v /= 10;
  } while (v && n < (int)sizeof(digits));
  while (n > 0 && p < end) {
    *p++ = digits[--n];
  }
  return p;
}

static void noteTrip(uint32_t uptimeMs, uint32_t stallUs, uint32_t alive)
{
  /* Same shape as an ESP_LOG warning, so 'crash' lists it with the others. */
  char line[CRASH_LOG_NOTE_MAX + 1];
  char *p = line;
  const char *end = line + CRASH_LOG_NOTE_MAX;
  p = appendText(p, end, "W (");
  p = appendU32(p, end, uptimeMs);
  p = appendText(p, end, ") MGUARD: stabilizer frozen ");
  p = appendU32(p, end, stallUs / 1000);
  p = appendText(p, end, " ms with motors on (loop ");
  p = appendU32(p, end, alive);
  p = appendText(p, end, "): motors off until reboot");
  *p = '\0';
  crashLogNote(line);
}

static void guardCallback(void *arg)
{
  (void)arg;
  const uint64_t nowUs = (uint64_t)esp_timer_get_time();
  const bool motorsOn = motorsAnyDutyNonZero();
  portENTER_CRITICAL(&coreMux);
  const uint32_t alive = aliveCount;
  const motorGuardVerdict_t v = motorGuardCoreStep(&core, nowUs, alive, motorsOn, enabledSwitch);
  const uint32_t stallUs = core.stallUs;
  callbacks++;
  portEXIT_CRITICAL(&coreMux);
  if (v != MG_TRIP) {
    return;
  }
  motorsEmergencyStopNoWait();
  const uint32_t ms = (uint32_t)(nowUs / 1000);
  tripUptimeMs = ms ? ms : 1;
  noteTrip(ms, stallUs, alive);
  /* Queue send with no wait. A trip needs a motor duty > 0, which only the
   * stabilizer loop writes after systemStart(), so ledseq is initialised. */
  (void)ledseqRun(&seq_restartLock);
}

void motorGuardInit(bool enabled)
{
  if (timer) {
    return;
  }
  enabledSwitch = enabled;
  motorGuardCoreInit(&core);
  const esp_timer_create_args_t args = {
    .callback = guardCallback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "mguard",
    /* After a stall of the whole system run once, not a burst of catch-up
     * calls; the late call's gap is not counted either way. */
    .skip_unhandled_events = true,
  };
  esp_err_t err = esp_timer_create(&args, &timer);
  if (err == ESP_OK) {
    err = esp_timer_start_periodic(timer, MOTOR_GUARD_PERIOD_US);
    if (err != ESP_OK) {
      esp_timer_delete(timer);
      timer = NULL;
    }
  }
  if (err != ESP_OK) {
    /* Manual flight goes on without it; autonomy ARM refuses (not enabled). */
    DEBUG_PRINTW("motor guard not running (%s): no cut on a frozen stabilizer", esp_err_to_name(err));
    return;
  }
  running = true;
  DEBUG_PRINTI("motor guard %s: cut after %u ms without stabilizer progress with motors on",
               enabled ? "on" : "off", (unsigned)(MOTOR_GUARD_FREEZE_US / 1000));
}

void motorGuardStabilizerAlive(void)
{
  aliveCount = aliveCount + 1;
}

void motorGuardSetEnabled(bool on)
{
  enabledSwitch = on;
}

bool IRAM_ATTR motorGuardEnabled(void)
{
  return running && enabledSwitch;
}

bool IRAM_ATTR motorGuardTripped(void)
{
  return core.tripped;
}

void motorGuardGetStats(motorGuardStats_t *out)
{
  portENTER_CRITICAL(&coreMux);
  out->enabled = running && enabledSwitch;
  out->tripped = core.tripped;
  out->trips = core.tripped ? 1 : 0;
  out->aliveCount = aliveCount;
  out->maxStallUs = core.maxStallUs;
  out->lateCalls = core.lateCalls;
  out->maxCallGapUs = core.maxCallGapUs;
  out->callbacks = callbacks;
  portEXIT_CRITICAL(&coreMux);
  out->tripUptimeMs = tripUptimeMs;
}

void IRAM_ATTR motorGuardPanicPinsLow(void)
{
  motorsPinsLowNoLock();
}
