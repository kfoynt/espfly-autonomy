/*
 * ESP-Drone Firmware
 *
 * cam_gnd.c - When the flow camera sleeps on the ground (cam_gnd.h; pure,
 * host-tested by tools/tests/test_cam_gnd.c). Flash code: the camera task and
 * the link task call it a few times a second.
 */
#include <string.h>

#include "cam_gnd.h"

/* t lies in (now, now + span]: a deadline still ahead, wrap-safe. */
static bool ahead(uint32_t t, uint32_t now, uint32_t span)
{
  return (uint32_t)(t - now) - 1u < span;
}

void camGndInit(camGnd_t *g, uint32_t nowMs)
{
  memset(g, 0, sizeof(*g));
  g->holdUntilMs = nowMs;
  g->retryAtMs = nowMs;
  g->wokeAtMs = nowMs;
}

void camGndRestarted(camGnd_t *g, uint32_t nowMs)
{
  if (g->asleep) {
    g->asleepMs += nowMs - g->sleptAtMs;
  }
  g->asleep = false;
  g->wokeAtMs = nowMs;
}

void camGndWake(camGnd_t *g, uint32_t nowMs, uint32_t holdMs)
{
  if (holdMs > CAM_GND_WAKE_MAX_MS) {
    holdMs = CAM_GND_WAKE_MAX_MS;
  }
  const uint32_t until = nowMs + holdMs;
  /* keep a longer hold already running */
  if (!(ahead(g->holdUntilMs, nowMs, CAM_GND_WAKE_MAX_MS) && (int32_t)(g->holdUntilMs - until) > 0)) {
    g->holdUntilMs = until;
  }
}

void camGndRelease(camGnd_t *g, uint32_t nowMs)
{
  g->holdUntilMs = nowMs;
}

bool camGndHoldActive(const camGnd_t *g, uint32_t nowMs)
{
  return ahead(g->holdUntilMs, nowMs, CAM_GND_WAKE_MAX_MS);
}

camGndAction_t camGndAwakeStep(const camGnd_t *g, const camGndIn_t *in)
{
  if (g->asleep || !in->enabled || !in->supported || !in->running || in->autonomyActive || !in->onGround ||
      in->takeoffWindow || in->snapPending || in->benchHold || in->clockPending || camGndHoldActive(g, in->nowMs) ||
      in->nowMs - in->startMs < CAM_GND_START_MS || ahead(g->retryAtMs, in->nowMs, CAM_GND_RETRY_MS)) {
    return CAM_GND_STAY;
  }
  return CAM_GND_SLEEP;
}

void camGndSleepResult(camGnd_t *g, uint32_t nowMs, bool ok)
{
  if (ok) {
    g->asleep = true;
    g->slept = true;
    g->sleptAtMs = nowMs;
    g->count++;
  } else {
    g->fails++;
    g->retryAtMs = nowMs + CAM_GND_RETRY_MS;
  }
}

camGndAction_t camGndAsleepStep(const camGnd_t *g, const camGndIn_t *in)
{
  if (!g->asleep) {
    return CAM_GND_STAY;
  }
  /* A ground start first: it also clears onGround in the same stabilizer loop. */
  if (in->groundStart) {
    return in->takeoffStandby ? CAM_GND_HANDOVER : CAM_GND_WAKE;
  }
  if (camGndHoldActive(g, in->nowMs) || in->autonomyActive || !in->onGround || !in->enabled || in->benchHold) {
    return CAM_GND_WAKE;
  }
  return CAM_GND_STAY;
}

void camGndWoke(camGnd_t *g, uint32_t nowMs, bool handover)
{
  if (g->asleep) {
    g->asleepMs += nowMs - g->sleptAtMs;
  }
  g->asleep = false;
  if (!handover) {
    g->wokeAtMs = nowMs;
  }
}

void camGndWakeFailed(camGnd_t *g)
{
  g->fails++;
}

bool camGndFpsVisible(const camGnd_t *g, uint32_t nowMs, uint32_t leadMs, bool autonomyActive)
{
  return autonomyActive || leadMs == 0u || (!g->asleep && nowMs - g->wokeAtMs >= leadMs);
}

uint32_t camGndAwakeMs(const camGnd_t *g, uint32_t nowMs)
{
  if (!g->slept) {
    return UINT32_MAX;
  }
  return g->asleep ? 0u : nowMs - g->wokeAtMs;
}

uint32_t camGndAsleepTotalMs(const camGnd_t *g, uint32_t nowMs)
{
  return g->asleepMs + (g->asleep ? nowMs - g->sleptAtMs : 0u);
}
