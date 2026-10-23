/*
 * ESP-Drone Firmware
 *
 * motor_guard_core.c - The motor watchdog's decision (motor_guard.h): has the
 * stabilizer stopped while the motors are driven? Pure code, host-tested by
 * tools/tests/test_motor_guard.c.
 */
#include "motor_guard.h"

static uint32_t saturateUs(uint64_t us)
{
  return us > UINT32_MAX ? UINT32_MAX : (uint32_t)us;
}

static void restartWindow(motorGuardCore_t *c, uint64_t nowUs, uint32_t alive)
{
  c->windowStartUs = nowUs;
  c->lastAlive = alive;
  c->stallUs = 0;
}

void motorGuardCoreInit(motorGuardCore_t *c)
{
  c->lastCallUs = 0;
  c->windowStartUs = 0;
  c->lastAlive = 0;
  c->stallUs = 0;
  c->maxStallUs = 0;
  c->lateCalls = 0;
  c->maxCallGapUs = 0;
  c->tripped = false;
}

motorGuardVerdict_t motorGuardCoreStep(motorGuardCore_t *c, uint64_t nowUs, uint32_t alive, bool motorsOn,
                                       bool enabled)
{
  /* The callback timing is tracked on every call, so maxCallGapUs shows the
   * guard's own health whatever it decides. A clock that went backwards
   * counts as late (it cannot on the drone; the window restarts anyway). */
  const bool first = c->lastCallUs == 0;
  const bool backwards = !first && nowUs < c->lastCallUs;
  const uint32_t gapUs = (first || backwards) ? 0 : saturateUs(nowUs - c->lastCallUs);
  c->lastCallUs = nowUs;
  if (gapUs > c->maxCallGapUs) {
    c->maxCallGapUs = gapUs;
  }

  if (c->tripped) {
    return MG_TRIPPED;
  }
  if (!enabled) {
    restartWindow(c, nowUs, alive);
    return MG_DISABLED;
  }
  if (first || backwards) {
    if (backwards) {
      c->lateCalls++;
    }
    restartWindow(c, nowUs, alive);
    return MG_LATE;
  }
  const bool late = gapUs > MOTOR_GUARD_LATE_US;
  if (late) {
    c->lateCalls++;
  }
  if (alive != c->lastAlive || !motorsOn) {
    restartWindow(c, nowUs, alive);
    return late ? MG_LATE : MG_OK;
  }
  if (late) {
    /* The guard itself was held up (flash erase with the cache off, a
     * priority >= 23 task or a long critical section on core 0): this gap
     * was not observed, so none of it counts. What was observed before it
     * stays counted: restarting here would let a guard that is late every
     * few tens of ms never trip on a real freeze. A system-wide stall cannot
     * cut either way, the stabilizer resumes within 1-2 ms of it. */
    return MG_LATE;
  }
  const uint64_t stall = (uint64_t)c->stallUs + gapUs;
  c->stallUs = saturateUs(stall);
  if (c->stallUs > c->maxStallUs) {
    c->maxStallUs = c->stallUs;
  }
  if (c->stallUs >= MOTOR_GUARD_FREEZE_US) {
    c->tripped = true;
    return MG_TRIP;
  }
  return MG_STALLING;
}
