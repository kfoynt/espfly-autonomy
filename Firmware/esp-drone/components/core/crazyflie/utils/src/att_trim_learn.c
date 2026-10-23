/*
 * ESP-Drone Firmware
 *
 * att_trim_learn.c - Learn the attitude trim from the camera drift hold
 */
#include "att_trim_learn.h"

#include <math.h>

#include "auto_fsm.h"   /* autoState_t, autoReason_t values (attLearnEndOk) */

void attLearnStart(attLearnFlight_t *f, uint32_t sampleRateHz)
{
  f->sumRoll = 0.0f;
  f->sumPitch = 0.0f;
  f->n = 0;
  f->settle = 0;
  f->settleSamples = (uint32_t)(ATT_LEARN_SETTLE_S * (float)sampleRateHz);
}

void attLearnAdd(attLearnFlight_t *f, const attLearnSample_t *s)
{
  const bool finite = isfinite(s->corrRollDeg) && isfinite(s->corrPitchDeg) && isfinite(s->rollDeg) &&
                      isfinite(s->pitchDeg) && isfinite(s->gyroDps);
  const bool steady = finite && fabsf(s->rollDeg) <= ATT_LEARN_MAX_TILT_DEG &&
                      fabsf(s->pitchDeg) <= ATT_LEARN_MAX_TILT_DEG && s->gyroDps <= ATT_LEARN_MAX_GYRO_DPS;
  /* A motion's feedforward lean and references are in the correction: like a stick input. */
  if (!s->holdApplied || !s->sticksCentred || !s->airborne || !steady || s->motionActive) {
    f->settle = 0;   /* whatever follows starts with the hold braking: settle again */
    return;
  }
  if (f->settle < f->settleSamples) {
    f->settle++;
    return;
  }
  if (!s->holdOnView) {
    return;          /* coasting on a lost view: skip, but stay settled */
  }
  if (fabsf(s->corrRollDeg) >= ATT_LEARN_SAT_DEG || fabsf(s->corrPitchDeg) >= ATT_LEARN_SAT_DEG) {
    return;          /* the hold at its limit chases something it cannot hold: skip, stay settled */
  }
  f->sumRoll += s->corrRollDeg;
  f->sumPitch += s->corrPitchDeg;
  f->n++;
}

bool attLearnEndOk(bool autonomySeen, uint8_t stateEnd, uint8_t reasonEnd, uint16_t events)
{
  if (!autonomySeen) {
    return true;     /* a manual flight */
  }
  if (stateEnd != AUTO_LAND || events != 0) {
    return false;    /* (an event: what used to end the sortie, now held and reported) */
  }
  switch (reasonEnd) {
    case AUTO_RS_LAND_CMD:
    case AUTO_RS_SORTIE_DONE:
    case AUTO_RS_HOVER_TIMEOUT:
    case AUTO_RS_BATT_LOW:
      return true;
    default:
      return false;
  }
}

static float clampf(float v, float lim)
{
  return v > lim ? lim : (v < -lim ? -lim : v);
}

/* One axis: move GAIN of the mean into the trim, bounded per step and in
 * total; a hand-set trim beyond the limit is never pushed further out. */
static float learnAxis(float trim, float mean)
{
  float next = trim + clampf(ATT_LEARN_GAIN * mean, ATT_LEARN_MAX_STEP_DEG);
  if (fabsf(next) > ATT_LEARN_LIMIT_DEG) {
    if (fabsf(trim) > ATT_LEARN_LIMIT_DEG) {
      return fabsf(next) < fabsf(trim) ? next : trim;
    }
    return clampf(next, ATT_LEARN_LIMIT_DEG);
  }
  return next;
}

bool attLearnFinish(const attLearnFlight_t *f, uint32_t sampleRateHz, float *trimRoll, float *trimPitch,
                    float *meanRoll, float *meanPitch)
{
  if (sampleRateHz == 0 || (float)f->n < ATT_LEARN_MIN_S * (float)sampleRateHz) {
    return false;
  }
  const float mr = f->sumRoll / (float)f->n;
  const float mp = f->sumPitch / (float)f->n;
  if (!isfinite(mr) || !isfinite(mp) || !isfinite(*trimRoll) || !isfinite(*trimPitch)) {
    return false;
  }
  *meanRoll = mr;
  *meanPitch = mp;
  /* A mean beyond what trim error explains (a draft, a one-off): skip the axis. */
  if (fabsf(mr) <= ATT_LEARN_MAX_MEAN_DEG) {
    *trimRoll = learnAxis(*trimRoll, mr);
  }
  if (fabsf(mp) <= ATT_LEARN_MAX_MEAN_DEG) {
    *trimPitch = learnAxis(*trimPitch, mp);
  }
  return true;
}
