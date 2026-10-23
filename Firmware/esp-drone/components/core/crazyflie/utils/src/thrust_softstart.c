/*
 * ESP-Drone Firmware
 *
 * thrust_softstart.c - Gentle take-off from a ground stop (thrust slew limit)
 */
#include "thrust_softstart.h"

#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#else
#define IRAM_ATTR
#endif

void thrustSoftStartInit(thrustSoftStart_t *s)
{
  s->restLoops = 0;
  s->fallLoops = 0;
  s->level = 0;
  s->active = true;   /* boot: on the ground */
}

uint16_t thrustSoftStartApply(thrustSoftStart_t *s, uint16_t request, bool resting, bool falling, bool enabled)
{
  if (falling) {
    if (s->fallLoops < UINT16_MAX) {
      s->fallLoops++;
    }
    if (s->fallLoops >= THRUST_SOFT_FALL_LOOPS) {
      s->active = false;   /* in the air: never limit thrust */
    }
  } else {
    s->fallLoops = 0;
  }
  if (request == 0) {
    if (resting) {
      if (s->restLoops < UINT16_MAX) {
        s->restLoops++;
      }
    } else {
      s->restLoops = 0;
    }
    if (s->restLoops >= THRUST_SOFT_ARM_LOOPS && !falling) {
      s->active = true;
      s->level = 0;
    } else if (s->active) {
      s->level = s->level > THRUST_SOFT_STEP ? (uint16_t)(s->level - THRUST_SOFT_STEP) : 0;   /* spinning down */
    }
    return 0;
  }
  s->restLoops = 0;
  if (!enabled) {
    /* No ramp, but keep 'active' meaning "on the ground since the last
     * rest": the take-off window (stabilizer) is keyed on it. */
    s->level = request;
    if (request >= THRUST_SOFT_END) {
      s->active = false;
    }
    return request;
  }
  if (!s->active) {
    return request;
  }
  const uint32_t allowed = (uint32_t)s->level + THRUST_SOFT_STEP;
  const uint16_t out = request <= allowed ? request : (uint16_t)allowed;
  if (out >= s->level) {
    s->level = out;
  } else {
    /* Lower request: the motors slow down, but not instantly. */
    s->level = (s->level - out > THRUST_SOFT_STEP) ? (uint16_t)(s->level - THRUST_SOFT_STEP) : out;
  }
  if (out == request && request >= THRUST_SOFT_END) {
    s->active = false;   /* caught up and spinning: pass through until the next ground stop */
  }
  return out;
}

bool IRAM_ATTR thrustSoftStartActive(const thrustSoftStart_t *s)
{
  return s->active;
}
