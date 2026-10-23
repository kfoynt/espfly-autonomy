/*
 * ESP-Drone Firmware
 *
 * ground_cool.c - Ground prop-wash cooling spin (pure helper)
 */
#include <math.h>
#include <string.h>

#include "ground_cool.h"

void groundCoolInit(groundCool_t *gc)
{
  memset(gc, 0, sizeof(*gc));
}

groundCoolRefusal_t groundCoolStart(groundCool_t *gc, const groundCoolIn_t *in, uint32_t pwm, uint32_t seconds,
                                    uint32_t nowMs)
{
  if (pwm < GROUND_COOL_PWM_MIN || pwm > GROUND_COOL_PWM_MAX || seconds < 1 || seconds > GROUND_COOL_S_MAX) {
    return GC_E_RANGE;
  }
  if (gc->running) {
    return GC_E_RUNNING;
  }
  if (in->autonomyActive) {
    return GC_E_AUTO;
  }
  if (!in->dutyZero || !in->thrustZero) {
    return GC_E_THRUST;
  }
  const bool resting = fabsf(in->gyroDps[0]) < GROUND_COOL_REST_GYRO_DPS &&
                       fabsf(in->gyroDps[1]) < GROUND_COOL_REST_GYRO_DPS &&
                       fabsf(in->gyroDps[2]) < GROUND_COOL_REST_GYRO_DPS &&
                       fabsf(in->accNormG - 1.0f) < GROUND_COOL_ACC_TOL_G;
  if (!in->onGround || !resting || !(in->tiltDeg < GROUND_COOL_TILT_MAX_DEG)) {
    return GC_E_GROUND;
  }
  if (!in->motorsReady) {
    return GC_E_LOCKED;
  }
  if (!(in->vbat >= in->vminV)) {
    return GC_E_BATT;
  }
  gc->running = true;
  gc->pwm = (uint16_t)pwm;
  gc->seconds = (uint16_t)seconds;
  gc->startMs = nowMs;
  gc->killSeen = in->killCount;
  gc->end = GC_R_NONE;
  gc->stopReq = false;
  gc->killReq = false;
  gc->tStartC = in->imuTempC;
  gc->tEndC = in->imuTempC;
  gc->tiltMaxDeg = 0.0f;
  gc->gyroMaxDps = 0.0f;
  gc->elapsedMs = 0;
  gc->runs++;
  return GC_OK;
}

static void endRun(groundCool_t *gc, groundCoolEnd_t why, uint32_t nowMs)
{
  gc->running = false;
  gc->end = (uint8_t)why;
  gc->elapsedMs = nowMs - gc->startMs;
}

float groundCoolStep(groundCool_t *gc, const groundCoolIn_t *in, uint32_t nowMs)
{
  if (!gc->running) {
    return 0.0f;
  }
  const uint32_t elapsed = nowMs - gc->startMs;
  gc->tEndC = in->imuTempC;
  const float gyro = fmaxf(fmaxf(fabsf(in->gyroDps[0]), fabsf(in->gyroDps[1])), fabsf(in->gyroDps[2]));
  if (in->tiltDeg > gc->tiltMaxDeg) {
    gc->tiltMaxDeg = in->tiltDeg;
  }
  if (gyro > gc->gyroMaxDps) {
    gc->gyroMaxDps = gyro;
  }
  groundCoolEnd_t why = GC_R_NONE;
  if (gc->killReq || in->killCount != gc->killSeen) {
    why = GC_R_KILL;
  } else if (gc->stopReq) {
    why = GC_R_STOP;
  } else if (in->autonomyActive) {
    why = GC_R_AUTO;
  } else if (!in->motorsReady) {
    why = GC_R_LOCKED;
  } else if (!(in->tiltDeg <= GROUND_COOL_TILT_MAX_DEG)) {   /* a NaN tilt aborts too */
    why = GC_R_TILT;
  } else if (!(gyro <= GROUND_COOL_GYRO_MAX_DPS)) {
    why = GC_R_GYRO;
  } else if (elapsed >= GROUND_COOL_SPINUP_MS && !(fabsf(in->accNormG - 1.0f) <= GROUND_COOL_ACC_TOL_G)) {
    why = GC_R_ACC;
  } else if (elapsed >= (uint32_t)gc->seconds * 1000u) {
    why = GC_R_DONE;
  }
  if (why != GC_R_NONE) {
    endRun(gc, why, nowMs);
    return 0.0f;
  }
  return (float)gc->pwm;
}

void groundCoolStop(groundCool_t *gc)
{
  gc->stopReq = true;
}

void groundCoolKill(groundCool_t *gc)
{
  gc->killReq = true;
}

const char *groundCoolRefusalName(groundCoolRefusal_t r)
{
  switch (r) {
    case GC_OK: return "ok";
    case GC_E_RANGE: return "range";
    case GC_E_RUNNING: return "running";
    case GC_E_AUTO: return "autonomy active";
    case GC_E_THRUST: return "thrust requested or a motor output nonzero";
    case GC_E_GROUND: return "not resting on the ground";
    case GC_E_LOCKED: return "motor driver locked (STOP latch or restart lock)";
    case GC_E_BATT: return "battery below vmin";
  }
  return "?";
}

const char *groundCoolEndName(groundCoolEnd_t e)
{
  switch (e) {
    case GC_R_NONE: return "none";
    case GC_R_DONE: return "DONE";
    case GC_R_STOP: return "STOP";
    case GC_R_KILL: return "KILL";
    case GC_R_TILT: return "TILT";
    case GC_R_GYRO: return "GYRO";
    case GC_R_ACC: return "ACC";
    case GC_R_AUTO: return "AUTO";
    case GC_R_LOCKED: return "LOCKED";
  }
  return "?";
}
