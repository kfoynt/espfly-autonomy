/*
 * ESP-Drone Firmware
 *
 * camera_hold.c - Camera drift hold control law (pure)
 */
#include <math.h>
#include <string.h>

#include "camera_hold.h"
#include "camera_cadence.h"
#include "auto_fsm.h"   /* autoState_t (camHoldGainTarget) */

#define RATE_REF_ZERO 1e-4f   /* 1/s: a decayed rate reference below this is exactly 0 again */

static float clampf(float v, float lim)
{
  return fminf(fmaxf(v, -lim), lim);
}

/* Move 'cur' toward 'target' by at most 'step'. */
static float slew(float cur, float target, float step)
{
  float d = target - cur;
  if (d > step) {
    d = step;
  } else if (d < -step) {
    d = -step;
  }
  return cur + d;
}

void camHoldReset(camHoldState_t *st)
{
  memset(st, 0, sizeof(*st));
}

/* Positions of a new reference epoch start from 0 (the integrals stay). */
static void epochStart(camHoldState_t *st, uint32_t epoch)
{
  st->epoch = epoch;
  st->pFwd = st->pLat = 0.0f;
  st->rFwd = st->rLat = 0.0f;
  st->lostFwd = st->lostLat = 0.0f;
  st->rateFwdF = st->rateLatF = 0.0f;
}

/* Takes a reference change off one integral, clamped as the measurement is. */
static void refTake(float *i, float *lost, float dr, float lim)
{
  const float pre = *i - dr;
  *i = clampf(pre, lim);
  *lost += pre - *i;
}

/* The gain scale as applied: 0, NaN, inf or <= 1 are 1; at most CAM_HOLD_GAIN_SCALE_MAX. */
static float gainScaleOf(float s)
{
  return (s > 1.0f && isfinite(s)) ? fminf(s, CAM_HOLD_GAIN_SCALE_MAX) : 1.0f;
}

/* The MOVE lateral position gain as applied (camHoldInput_t latGain): 0, NaN, inf or <= 1 are 1; at most
 * CAM_HOLD_LAT_GAIN_MAX. */
static float latGainOf(float g)
{
  return (g > 1.0f && isfinite(g)) ? fminf(g, CAM_HOLD_LAT_GAIN_MAX) : 1.0f;
}

/* A gain scale change: the integral times f = s_old / s_new keeps the learned lean KI s i; the bookkeeping takes
 * the same factor (lost' = f lost + (1 - f) prog, prog = p - r, plus what the clamp takes), so that
 * i + lost - prog, the epoch start's integral a snap restores, is scaled by f too (its lean kept as well). */
static void gainRescale(float *i, float *lost, float prog, float f, float lim)
{
  const float pre = *i * f;
  *i = clampf(pre, lim);
  *lost = *lost * f + (1.0f - f) * prog + (pre - *i);
}

/* camHoldInput_t ffBrakeScale: a braking feedforward ff times the measured over the reference rate (the same filter),
 * clamped to 0..CAM_HOLD_FF_BRAKE_MAX; unscaled when not braking, the reference rate is under CAM_HOLD_FF_BRAKE_WMIN, or
 * the camera has not seen the move go (its measured progress p under CAM_HOLD_FF_BRAKE_TRACK of the reference taken r:
 * a frozen view reads no speed while the drone flies, and must keep its brake). */
static float brakeScaled(float ff, bool braking, float meas, float ref, float p, float r)
{
  if (!braking || !(fabsf(ref) >= CAM_HOLD_FF_BRAKE_WMIN) || !isfinite(meas) ||
      !(p * r > 0.0f && fabsf(p) >= CAM_HOLD_FF_BRAKE_TRACK * fabsf(r))) {
    return ff;
  }
  return ff * fminf(fmaxf(meas / ref, 0.0f), CAM_HOLD_FF_BRAKE_MAX);
}

bool camHoldStep(camHoldState_t *st, const camHoldInput_t *in)
{
  float dt = in->dt;
  bool dropout = !(dt > 0.0f) || dt > CAM_HOLD_MAX_DT_S || !isfinite(in->wFwd) || !isfinite(in->wLat);
  if (!(dt > 0.0f) || !isfinite(dt)) {
    dt = 0.0f;
  } else if (dt > CAM_HOLD_MAX_DT_S) {
    dt = CAM_HOLD_MAX_DT_S;
  }

  /* The gain scale (docs/autonomy-a2b.md 20): s 1 computes exactly what the hold did before it. A change first
   * rescales the integrals so that the learned lean (KI s i) is continuous. */
  const float s = gainScaleOf(in->gainScale);
  const float sOld = gainScaleOf(st->gainScale);
  const float kp = CAM_HOLD_KP * s, ki = CAM_HOLD_KI * s, iLim = CAM_HOLD_I_LIMIT_DEG / ki;
  const float ls = fminf(gainScaleOf(in->latGainScale), CAM_HOLD_STARTUP_LAT_MAX);
  /* the MOVE lateral position gain (camera_hold.h latGain; at 1 exactly ki x ls: the lateral axis as before); the
   * state keeps the product ls x lg (at lg 1 the startup scale itself, as before) */
  const float lg = latGainOf(in->latGain);
  const float lsLg = ls * lg;
  const float lsLgOld = (st->latGainScale > 1.0f && isfinite(st->latGainScale)) ? st->latGainScale : 1.0f;
  const float latKp = kp * ls, latKi = ki * lsLg, latILim = CAM_HOLD_I_LIMIT_DEG / latKi;
  if (s != sOld) {
    const float f = sOld / s;
    gainRescale(&st->iFwd, &st->lostFwd, st->pFwd - st->rFwd, f, iLim);
  }
  if (s != sOld || lsLg != lsLgOld) {
    gainRescale(&st->iLat, &st->lostLat, st->pLat - st->rLat, (sOld * lsLgOld) / (s * lsLg), latILim);
  }
  st->gainScale = s;
  st->latGainScale = lsLg;

  /* A new reference epoch (only ever with a motion). A snap first puts each
   * integral back where it was when the ending epoch began: integral plus
   * what its clamp took, minus (measured progress - reference taken). */
  if (in->refEpoch != st->epoch) {
    if ((int32_t)(in->refSnapEpoch - st->epoch) > 0) {
      st->iFwd = clampf(st->iFwd + st->lostFwd - (st->pFwd - st->rFwd), iLim);
      st->iLat = clampf(st->iLat + st->lostLat - (st->pLat - st->rLat), latILim);
    }
    epochStart(st, in->refEpoch);
  }

  st->airborneTime = in->airborne ? st->airborneTime + dt : 0.0f;

  const float settle = in->earlyTakeoffLat && in->fwdOff ? CAM_HOLD_TAKEOFF_LAT_SETTLE_S : CAM_HOLD_SETTLE_S;
  bool engage = in->enabled && in->sticksCentred && in->airborne && in->viewOk && !dropout &&
                st->airborneTime >= settle;
  bool coast = !engage && in->enabled && in->sticksCentred && in->airborne &&
               st->airborneTime >= settle;

  if (engage) {
    if (!st->engaged) {
      /* Start the filters from the current measurement. */
      st->fwdF = in->fwdOff ? 0.0f : in->wFwd;
      st->latF = in->wLat;
      st->rateFwdF = in->rateFwd;
      st->rateLatF = in->rateLat;
    }
    const float a = dt / (CAM_HOLD_FILTER_TAU_S + dt);
    if (in->fwdOff) {
      st->fwdF = 0.0f;   /* the forward axis is off: nothing filtered, nothing integrated */
    } else {
      st->fwdF += a * (in->wFwd - st->fwdF);
      const float iFwd0 = st->iFwd;
      st->iFwd = clampf(st->iFwd + in->wFwd * dt, iLim);
      /* bookkeeping for the reference epochs (never feeds the outputs) */
      st->lostFwd += (iFwd0 + in->wFwd * dt) - st->iFwd;
      st->pFwd += in->wFwd * dt;
    }
    st->latF += a * (in->wLat - st->latF);
    const float iLat0 = st->iLat;
    st->iLat = clampf(st->iLat + in->wLat * dt, latILim);
    st->lostLat += (iLat0 + in->wLat * dt) - st->iLat;
    st->pLat += in->wLat * dt;
    /* the moving reference: what it moved since the last engaged frame */
    const float drFwd = in->fwdOff ? 0.0f : in->refFwd - st->rFwd, drLat = in->refLat - st->rLat;
    if (drFwd != 0.0f && isfinite(drFwd)) {
      refTake(&st->iFwd, &st->lostFwd, drFwd, iLim);
      st->rFwd = in->refFwd;
    }
    if (drLat != 0.0f && isfinite(drLat)) {
      refTake(&st->iLat, &st->lostLat, drLat, latILim);
      st->rLat = in->refLat;
    }
    if (in->rateFwd != 0.0f || st->rateFwdF != 0.0f) {
      st->rateFwdF += a * (in->rateFwd - st->rateFwdF);
      if (in->rateFwd == 0.0f && fabsf(st->rateFwdF) < RATE_REF_ZERO) {
        st->rateFwdF = 0.0f;
      }
    }
    if (in->rateLat != 0.0f || st->rateLatF != 0.0f) {
      st->rateLatF += a * (in->rateLat - st->rateLatF);
      if (in->rateLat == 0.0f && fabsf(st->rateLatF) < RATE_REF_ZERO) {
        st->rateLatF = 0.0f;
      }
    }
    st->idleTime = 0.0f;
  } else if (coast) {
    st->idleTime = 0.0f;     /* keep the learned integrals */
  } else {
    st->idleTime += dt;
    if (st->idleTime >= CAM_HOLD_RESET_S || !in->airborne) {
      st->iFwd = 0.0f;
      st->iLat = 0.0f;
      epochStart(st, st->epoch);   /* the reference positions go with the integrals */
    }
  }
  st->engaged = engage;
  st->coasting = coast;

  float targetPitch = 0.0f, targetRoll = 0.0f;
  const bool refActive = st->rateFwdF != 0.0f || st->rateLatF != 0.0f || in->ffPitch != 0.0f || in->ffRoll != 0.0f;
  const bool fast = in->moveLimDeg > 0.0f && refActive;   /* mvspeed > 100 (camera_hold.h): 0 never takes it */
  /* the feedforward; a braking one scaled by the measured speed (ffBrakeScale, engaged frames only) */
  float ffPs = isfinite(in->ffPitch) ? in->ffPitch : 0.0f;
  float ffRs = isfinite(in->ffRoll) ? in->ffRoll : 0.0f;
  if (in->ffBrakeScale && engage) {
    ffPs = brakeScaled(ffPs, ffPs * in->rateFwd > 0.0f, st->fwdF, st->rateFwdF, st->pFwd, st->rFwd);   /* nose up
                                                                                                    against forward */
    ffRs = brakeScaled(ffRs, ffRs * in->rateLat < 0.0f, st->latF, st->rateLatF, st->pLat, st->rLat);   /* left down
                                                                                                    against right */
  }
  if (fast && (engage || coast) && !(engage && in->fwdOff)) {
    /* the camera feedback (the learned trim included) clamped as before, the feedforward on top, the sum at the move
     * limit; coasting applies the feedforward too (a stopped move's brake while the view is lost) */
    const float ffP = ffPs;
    const float ffR = ffRs;
    const float fbP = engage ? kp * (st->fwdF - st->rateFwdF) + ki * st->iFwd : ki * st->iFwd;
    const float fbR = engage ? -(latKp * (st->latF - st->rateLatF) + latKi * st->iLat) : -latKi * st->iLat;
    targetPitch = clampf(clampf(fbP, CAM_HOLD_LIMIT_DEG) + ffP, in->moveLimDeg);
    targetRoll = clampf(clampf(fbR, CAM_HOLD_LIMIT_DEG) + ffR, in->moveLimDeg);
  } else if (engage && refActive) {
    const float ffP = ffPs;
    const float ffR = ffRs;
    targetPitch = clampf(kp * (st->fwdF - st->rateFwdF) + ki * st->iFwd + ffP, CAM_HOLD_LIMIT_DEG);
    targetRoll = clampf(-(latKp * (st->latF - st->rateLatF) + latKi * st->iLat) + ffR, CAM_HOLD_LIMIT_DEG);
  } else if (engage) {
    targetPitch = clampf(kp * st->fwdF + ki * st->iFwd, CAM_HOLD_LIMIT_DEG);
    targetRoll = clampf(-(latKp * st->latF + latKi * st->iLat), CAM_HOLD_LIMIT_DEG);
  } else if (coast) {
    targetPitch = clampf(ki * st->iFwd, CAM_HOLD_LIMIT_DEG);
    targetRoll = clampf(-latKi * st->iLat, CAM_HOLD_LIMIT_DEG);
  }
  if (engage && in->fwdOff) {
    /* the forward axis off: what it had learned, plus the feedforward */
    targetPitch = clampf(ki * st->iFwd + (isfinite(in->ffPitch) ? in->ffPitch : 0.0f), CAM_HOLD_LIMIT_DEG);
  }
  const float step = (fast && in->moveSlewDegS > 0.0f ? in->moveSlewDegS : CAM_HOLD_SLEW_DEG_S) * dt;
  st->pitch = slew(st->pitch, targetPitch, step);
  st->roll = slew(st->roll, targetRoll, step);
  return engage || coast;
}

float camHoldStartupLatTarget(uint8_t autoState, uint32_t refEpoch, uint8_t cvFlags, uint16_t cvGainDpm, float hoverS)
{
  return (autoState == AUTO_TAKEOFF || autoState == AUTO_HOVER) && refEpoch == 0 &&
         (autoState != AUTO_HOVER || (hoverS >= 0.0f && hoverS < CAM_HOLD_STARTUP_HOVER_S)) &&
         (cvFlags & 0x07u) == 0x07u && /* airborne, healthy, tracked */
         cvGainDpm >= CAM_HOLD_GAIN_DPM_MIN && cvGainDpm <= 500u ? CAM_HOLD_STARTUP_LAT_MAX : 1.0f;
}

bool camHoldMoving(camHoldState_t *st, float dt)
{
  if (!st->engaged) {
    st->stillS = 0.0f;
    return false;
  }
  if (fabsf(st->fwdF) < CAM_HOLD_STILL_RATE && fabsf(st->latF) < CAM_HOLD_STILL_RATE) {
    st->stillS += (dt > 0.0f && isfinite(dt)) ? fminf(dt, CAM_HOLD_MAX_DT_S) : 0.0f;
  } else {
    st->stillS = 0.0f;
  }
  return st->stillS < CAM_HOLD_STILL_S;
}

void camHoldMoveLimits(uint16_t pct, float *limDeg, float *slewDegS)
{
  if (pct <= 100u) {
    *limDeg = 0.0f;
    *slewDegS = 0.0f;
    return;
  }
  const float s = (float)(pct > 300u ? 300u : pct) * 0.01f;
  *limDeg = fminf(CAM_HOLD_LIMIT_DEG + CAM_HOLD_MOVE_LIM_PER_S * (s - 1.0f), CAM_HOLD_MOVE_LIM_MAX_DEG);
  *slewDegS = fminf(CAM_HOLD_SLEW_DEG_S * s, CAM_HOLD_MOVE_SLEW_MAX_DEG_S);
}

bool camHoldFastActive(const camHoldState_t *st, const camHoldInput_t *in)
{
  return in->moveLimDeg > 0.0f &&
         (st->rateFwdF != 0.0f || st->rateLatF != 0.0f || in->ffPitch != 0.0f || in->ffRoll != 0.0f);
}

float camHoldSlewOf(const camHoldState_t *st, const camHoldInput_t *in)
{
  return camHoldFastActive(st, in) && in->moveSlewDegS > 0.0f ? in->moveSlewDegS : CAM_HOLD_SLEW_DEG_S;
}

void camHoldSceneReset(camHoldScene_t *g)
{
  memset(g, 0, sizeof(*g));
  g->maxResidual = CAM_HOLD_SCENE_RES_DEFAULT;
}

void camHoldSceneSetMaxResidual(camHoldScene_t *g, float maxResidual)
{
  g->maxResidual = maxResidual > 0.0f ? maxResidual : 0.0f;   /* (a NaN or negative value: off) */
}

bool camHoldSceneOk(camHoldScene_t *g, bool hover, float dt, float residual, float dxResPx, float scale)
{
  /* the observer's expansion EMA (cam_vz_obs.c frameUpdate), on every frame */
  if (dt > 0.0f && isfinite(scale)) {
    const float rate = scale / dt;
    if (isfinite(rate)) {
#ifdef ESPFLY_NATIVE_VGA
      g->expEma += camCadenceExpAlpha(dt) * (rate - g->expEma);
#else
      g->expEma += CAM_HOLD_SCENE_EXP_ALPHA * (rate - g->expEma);
#endif
    }
  }
  g->hoverS = (hover && g->inHover) ? g->hoverS + (dt > 0.0f ? dt : 0.0f) : 0.0f;
  g->inHover = hover;
  g->lastRejected = false;
  if (!(hover && g->hoverS >= CAM_HOLD_SCENE_ARM_S)) {
    return true;
  }
  if (g->maxResidual > 0.0f && !(residual < g->maxResidual)) {   /* (0: no residual test; NVS holdres) */
    g->rejRes++;
  } else if (!(fabsf(dxResPx) <=
#ifdef ESPFLY_NATIVE_VGA
               camCadenceDxLimit(CAM_HOLD_SCENE_MAX_DX_PX, dt)
#else
               CAM_HOLD_SCENE_MAX_DX_PX
#endif
               )) {
    g->rejDx++;
  } else if (fabsf(g->expEma) > CAM_HOLD_SCENE_MAX_EXP_RATE) {
    g->rejExp++;
  } else {
    return true;
  }
  g->lastRejected = true;
  return false;
}

bool camHoldYawOk(float yawAngleRad, float dt, bool turn)
{
  return dt > 0.0f && fabsf(yawAngleRad) < (turn ? AUTO_MV_TURN_GATE_RPS : CAM_HOLD_MAX_YAW_RATE) * dt;
}

bool camHoldTurnActive(uint8_t mvState)
{
  const unsigned st = mvState & 0x0Fu, kind = (unsigned)mvState >> 4;
  return kind == AUTO_MVK_YAW && (st == AUTO_MV_RUN || st == AUTO_MV_PAUSED || st == AUTO_MV_SETTLE);
}

bool camHoldMotionActive(uint8_t mvState)
{
  const unsigned st = mvState & 0x0Fu;
  return st == AUTO_MV_RUN || st == AUTO_MV_PAUSED || st == AUTO_MV_SETTLE;
}

bool camHoldMoveActive(uint8_t mvState)
{
  return ((unsigned)mvState >> 4) == AUTO_MVK_MOVE && camHoldMotionActive(mvState);
}

void camHoldGainReset(camHoldGainLatch_t *l)
{
  memset(l, 0, sizeof(*l));
}

float camHoldGainTarget(camHoldGainLatch_t *l, uint8_t autoState, uint8_t mvState, uint8_t cvFlags,
                        uint16_t cvGainDpm, float kmax)
{
  if (autoState != AUTO_TAKEOFF && autoState != AUTO_HOVER && autoState != AUTO_LAND) {
    camHoldGainReset(l);   /* IDLE, ARMED (a new sortie), KILLED: nothing latched */
    return 1.0f;
  }
  if (((unsigned)mvState >> 4) == AUTO_MVK_MOVE) {
    l->translated = true;   /* for the rest of the sortie */
  }
  if (camHoldTurnActive(mvState)) {
    l->g = 0.0f;            /* a new view: wait for a fresh tracked G */
  } else if ((cvFlags & CAM_HOLD_GAIN_CVF_LATCH) == CAM_HOLD_GAIN_CVF_LATCH && cvGainDpm >= CAM_HOLD_GAIN_DPM_MIN &&
             cvGainDpm <= CAM_HOLD_GAIN_DPM_MAX) {
    l->g = (float)cvGainDpm * 0.1f;
  }
  /* LAND keeps the latch but flies s 1: the descent's floor parallax reads as forward motion, which a larger gain
   * turns into a larger creep (the verifier's log replay through touchdown: +0.10..+0.26 m forward at s 1.4-2.5) */
  if (!(kmax > 1.0f) || l->translated || !(l->g > 0.0f) || autoState == AUTO_LAND) {
    return 1.0f;
  }
  return fminf(fmaxf(CAM_HOLD_GAIN_G_REF / l->g, 1.0f), fminf(kmax, CAM_HOLD_GAIN_SCALE_MAX));
}

float camHoldGainMove(camHoldGainLatch_t *l, float target, float sNow, uint8_t autoState, uint8_t mvState,
                      float dLowM, float refFwd, float progFwd)
{
  const unsigned st = mvState & 0x0Fu, kind = (unsigned)mvState >> 4;
  const bool active = kind == AUTO_MVK_MOVE && (st == AUTO_MV_RUN || st == AUTO_MV_PAUSED || st == AUTO_MV_SETTLE);
  if (active && !l->moveActive) {
    l->sMove = isfinite(sNow) && sNow > 1.0f ? sNow : 1.0f;   /* this move's start */
    l->confirmed = false;
  }
  l->moveActive = active;
  l->untracked = false;
  if (!active || autoState != AUTO_HOVER || !(l->g > 0.0f) || !(dLowM > 0.0f) || !isfinite(refFwd) ||
      !(l->sMove > 1.0f)) {
    return target;
  }
  if (refFwd > CAM_HOLD_GAIN_TRACK_MIN && !(l->moveS < l->graceS)) {
    l->confirmed = progFwd >= CAM_HOLD_GAIN_TRACK_FRAC * refFwd;
    l->untracked = !l->confirmed;   /* the camera does not see the move (a frozen view): no gain on its push */
  }
  if (l->fromMeasured) {
    /* mvfix GAIN: the distance the drone is at by its own measured progress (k at its low: the lower bound) */
    if (l->untracked) {
      return target;
    }
    const float p = isfinite(progFwd) && progFwd > 0.0f ? progFwd : 0.0f;
    const float dMeas = dLowM * expf(-p * (1.0f / AUTO_MV_K_LOW));
    return fminf(fmaxf(dMeas * (CAM_HOLD_GAIN_G_REF / AUTO_MV_FY_PX), 1.0f), l->sMove);
  }
  if (!l->confirmed) {
    return target;
  }
  const float dNow = dLowM * expf(-refFwd * (1.0f / AUTO_MV_K_LOW));
  return fminf(fmaxf(dNow * (CAM_HOLD_GAIN_G_REF / AUTO_MV_FY_PX), 1.0f), l->sMove);
}

void camHoldGainMoveFix(camHoldGainLatch_t *l, bool measured, float graceS, float dt)
{
  l->fromMeasured = measured;
  l->graceS = isfinite(graceS) && graceS > 0.0f ? graceS : 0.0f;
  const float d = (dt > 0.0f && isfinite(dt)) ? fminf(dt, CAM_HOLD_MAX_DT_S) : 0.0f;
  l->moveS = l->moveActive ? l->moveS + d : 0.0f;
}

float camHoldGainRate(const camHoldGainLatch_t *l, uint8_t autoState)
{
  return (l->translated && autoState == AUTO_HOVER && !l->untracked) ? CAM_HOLD_GAIN_SLEW_MOVE_PER_S
                                                                       : CAM_HOLD_GAIN_SLEW_PER_S;
}

float camHoldGainSlew(float s, float target, float dt, float ratePerS)
{
  if (!isfinite(s)) {
    s = 1.0f;
  }
  if (!isfinite(target)) {
    target = 1.0f;
  }
  if (!(ratePerS > 0.0f) || !isfinite(ratePerS)) {
    ratePerS = CAM_HOLD_GAIN_SLEW_PER_S;
  }
  const float d = (dt > 0.0f) ? ratePerS * fminf(dt, CAM_HOLD_MAX_DT_S) : 0.0f;
  return slew(s, target, d);
}

/* The hold at its limit (camera_hold.h camHoldLimitStep). A run's sign cannot flip at >= 1.3 deg (the output slew,
 * 6 deg/s, crosses the band in >= 0.4 s of frames): the sign is the latest frame's. */
uint8_t camHoldLimitStep(camHoldLimit_t *s, float roll, float pitch, bool counting, uint32_t nowMs)
{
  const float v[2] = {roll, pitch};
  static const uint8_t bit[2] = {CAM_HOLD_LIM_ROLL, CAM_HOLD_LIM_PITCH};
  static const uint8_t neg[2] = {CAM_HOLD_LIM_ROLL_NEG, CAM_HOLD_LIM_PITCH_NEG};
  for (int i = 0; i < 2; i++) {
    /* a run opens at CAM_HOLD_LIMIT_WARN_DEG and stays open down to CAM_HOLD_LIMIT_KEEP_DEG (2026-10-16 review: one
     * frame just under 1.3 deg no longer restarts the 8 s) */
    const float thr = (s->run & bit[i]) != 0u ? CAM_HOLD_LIMIT_KEEP_DEG : CAM_HOLD_LIMIT_WARN_DEG;
    const bool at = counting && fabsf(v[i]) >= thr;   /* (NaN: not at it) */
    if (!at) {
      s->run &= (uint8_t)~bit[i];
      s->bits &= (uint8_t)~(bit[i] | neg[i]);
      continue;
    }
    if ((s->run & bit[i]) == 0u) {
      s->run |= bit[i];
      s->sinceMs[i] = nowMs;
    }
    if (nowMs - s->sinceMs[i] >= CAM_HOLD_LIMIT_WARN_MS) {
      s->bits = (uint8_t)((s->bits & (uint8_t)~neg[i]) | bit[i] | (v[i] < 0.0f ? neg[i] : 0u));
    }
  }
  return s->bits;
}
