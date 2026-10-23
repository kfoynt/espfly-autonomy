/*
 * ESP-Drone Firmware
 *
 * cam_vz_shadow.c - The camera height observer in shadow mode (cam_vz_shadow.h)
 */
#include <math.h>
#include <string.h>

#include "cam_vz_shadow.h"

#define LP20_TAU_S (1.0f / (2.0f * 3.14159265f * 20.0f))

static CAM_VZ_HOT int16_t sat16(float v)
{
  if (!(v == v)) {
    return 0;
  }
  if (v > 32767.0f) {
    return 32767;
  }
  if (v < -32767.0f) {
    return -32767;   /* -32768 is FSUM_CVZ_NONE */
  }
  return (int16_t)lrintf(v);
}

static CAM_VZ_HOT uint16_t satu16(float v)
{
  if (!(v > 0.0f)) {
    return 0;
  }
  return v >= 65535.0f ? 65535 : (uint16_t)lrintf(v);
}

static uint16_t satu16u(uint32_t v)
{
  return v > 0xFFFFu ? 0xFFFFu : (uint16_t)v;
}

static uint8_t satu8u(uint32_t v)
{
  return v > 0xFFu ? 0xFFu : (uint8_t)v;
}

/* flash: the flight summary's T2 rates only */
static int8_t sat8(float v)
{
  if (!(v == v)) {
    return 0;
  }
  return v > 127.0f ? 127 : (v < -127.0f ? -127 : (int8_t)lrintf(v));
}

/* ---- ground baseline ring ------------------------------------------------- */

static void g0Clear(camVzShadow_t *s)
{
  s->g0Head = 0;
  s->g0N = 0;
  s->g0Sum = 0;
  s->g0Has = false;
  s->g0AccN = 0;
  s->g0AccSum = 0.0f;
}

static void g0Push(camVzShadow_t *s, float a)
{
  float d = (a - 1.0f) * 1e4f;
  d = fminf(fmaxf(d, -32767.0f), 32767.0f);
  const int16_t v = (int16_t)lrintf(d);
  if (s->g0N == CVS_G0_RING) {
    s->g0Sum -= s->g0Ring[s->g0Head];
  } else {
    s->g0N++;
  }
  s->g0Ring[s->g0Head] = v;
  s->g0Sum += v;
  s->g0Head = (uint8_t)((s->g0Head + 1) % CVS_G0_RING);
}

/* One baseline sample of a_z (g). The samples of each CVS_G0_PERIOD_US time
 * bucket are averaged and the mean enters the ring when the next bucket
 * starts (1 kHz: 10 samples per entry; the study's 100 Hz logs: 1). A gap of
 * more than CVS_G0_GAP_US between samples (handled, flying, or the take-off
 * sat out) restarts the ring. */
static void g0Add(camVzShadow_t *s, uint64_t timeUs, float a)
{
  if (!isfinite(a)) {
    return;
  }
  if (s->g0Has) {
    if (timeUs <= s->g0LastUs) {
      return;
    }
    if (timeUs - s->g0LastUs > CVS_G0_GAP_US) {
      g0Clear(s);
    }
  }
  const uint64_t bucket = timeUs / CVS_G0_PERIOD_US;
  if (s->g0AccN > 0 && bucket != s->g0Bucket) {
    g0Push(s, s->g0AccSum / (float)s->g0AccN);
    s->g0AccN = 0;
    s->g0AccSum = 0.0f;
  }
  if (s->g0AccN == 0) {
    s->g0Bucket = bucket;
  }
  s->g0AccSum += a;
  s->g0AccN++;
  s->g0Has = true;
  s->g0LastUs = timeUs;
}

static float g0Mean(const camVzShadow_t *s)
{
  return s->g0N ? 1.0f + (float)s->g0Sum / (float)s->g0N * 1e-4f : 1.0f;
}

/* k-th smallest of v[0..n-1] (reorders v). */
static int16_t selectK(int16_t *v, int n, int k)
{
  int lo = 0, hi = n - 1;
  while (lo < hi) {
    const int16_t pivot = v[(lo + hi) / 2];
    int i = lo, j = hi;
    while (i <= j) {
      while (v[i] < pivot) {
        i++;
      }
      while (v[j] > pivot) {
        j--;
      }
      if (i <= j) {
        const int16_t t = v[i];
        v[i] = v[j];
        v[j] = t;
        i++;
        j--;
      }
    }
    if (k <= j) {
      hi = j;
    } else if (k >= i) {
      lo = i;
    } else {
      break;
    }
  }
  return v[k];
}

/* Median of the ring without its newest CVS_G0_SKIP_LAST samples (all of them
 * if that would leave fewer than CVS_G0_PRIOR_MIN), g; false if too few. */
static bool g0Median(camVzShadow_t *s, float *g0)
{
  int n = s->g0N;
  if (n < CVS_G0_PRIOR_MIN) {
    return false;
  }
  if (n - CVS_G0_SKIP_LAST >= CVS_G0_PRIOR_MIN) {
    n -= CVS_G0_SKIP_LAST;
  }
  /* oldest first: the ring's oldest entry is at head when full, else at 0 */
  const int first = s->g0N == CVS_G0_RING ? s->g0Head : 0;
  for (int i = 0; i < n; i++) {
    s->g0Scratch[i] = s->g0Ring[(first + i) % CVS_G0_RING];
  }
  float m = (float)selectK(s->g0Scratch, n, n / 2);
  if (n % 2 == 0) {
    /* after the selection every entry below n / 2 is <= it: the lower middle is their maximum */
    int16_t lower = s->g0Scratch[0];
    for (int i = 1; i < n / 2; i++) {
      if (s->g0Scratch[i] > lower) {
        lower = s->g0Scratch[i];
      }
    }
    m = 0.5f * (m + (float)lower);
  }
  *g0 = 1.0f + m * 1e-4f;
  return true;
}

/* ---- sortie ----------------------------------------------------------------- */

static void detectorReset(camVzShadow_t *s)
{
  s->lp20 = 0.0f;
  s->cusum = 0.0f;
  s->liftHighUs = 0;
}

static void sortieStart(camVzShadow_t *s, uint64_t timeUs)
{
  camVzObsReset(&s->obs);
  s->sortie++;
  s->sortieUs = timeUs;
  s->phase = CVS_GROUND;
  s->g0 = 1.0f;
  detectorReset(s);
  s->liftBy = CVS_LIFT_NONE;
  s->lifted = false;
  s->tLift = 0;
  s->tWake = 0;
  s->hasWake = false;
  s->prevH = s->prevV = 0.0f;
  s->prevBiasG = 0.0f;
  s->hasImpact = false;
  s->tImpact = 0;
  s->impH = s->impV = s->impBiasG = 0.0f;
  s->touchdown = false;
  s->tdImpact = false;
  s->tdH = s->tdV = s->tdBiasG = 0.0f;
  s->airUs = s->camAirUs = s->okUs = s->gapMaxUs = s->firstOkUs = 0;
  s->hasFirstOk = false;
  s->hasBias1 = s->hasBias2 = false;
  s->bias1G = s->bias2G = s->sigBias2G = 0.0f;
  s->hMax = 0.0f;
  s->gainLast = 0.0f;
  s->gainEver = false;
  s->fault = false;
  s->priorSet = false;
  s->gyroBias[0] = s->gyroBias[1] = s->gyroBias[2] = 0.0f;
  s->gyroBiasSeeded = false;
}

static CAM_VZ_HOT uint32_t sinceSortie(const camVzShadow_t *s, uint64_t timeUs)
{
  if (timeUs <= s->sortieUs) {
    return 0;
  }
  const uint64_t d = timeUs - s->sortieUs;
  return d > UINT32_MAX ? UINT32_MAX : (uint32_t)d;
}

/* Accelerometer lift-off rule (vert_ctrl's detector against the ring mean). */
static bool accLiftoff(camVzShadow_t *s, float a, uint32_t dtUs)
{
  if (s->g0N < CVS_G0_MIN_SAMPLES || !isfinite(a) || dtUs == 0) {
    return false;
  }
  const float dt = (float)dtUs * 1e-6f;
  const float aRel = a - g0Mean(s);
  s->lp20 += dt / (dt + LP20_TAU_S) * (aRel - s->lp20);
  s->cusum = fmaxf(0.0f, s->cusum + CAM_VZ_G * (aRel - CVS_LIFT_CUSUM_K_G) * dt);
  s->liftHighUs = s->lp20 > CVS_LIFT_ACC_G ? s->liftHighUs + dtUs : 0;
  return s->liftHighUs >= CVS_LIFT_HOLD_US || s->cusum >= CVS_LIFT_CUSUM_V;
}

static void liftoff(camVzShadow_t *s, camVzShadowLift_t by, uint32_t tRel)
{
  if (!s->lifted) {
    float g0;
    if (g0Median(s, &g0) &&
        camVzObsSetBiasPrior(&s->obs, g0 - 1.0f + CAM_VZ_PRIOR_OFFSET_G, CAM_VZ_PRIOR_SIGMA_G)) {
      s->g0 = g0;
      s->priorSet = true;
    }
    s->lifted = true;
    s->tLift = tRel;
    s->liftBy = by;
  }
  detectorReset(s);
  s->hasImpact = false;
  s->phase = CVS_AIR;
}

/* In-flight gyro bias: low-pass of minus the attitude estimator's correction
 * (cam_vz_shadow.h, CVS_GYRO_BIAS_TAU_S). */
static CAM_VZ_HOT void gyroBiasStep(camVzShadow_t *s, const camVzShadowIn_t *in, uint32_t dtUs)
{
  if (dtUs == 0) {
    return;
  }
  const float dt = (float)dtUs * 1e-6f;
  /* the first block starts from the correction itself: it has settled at minus the bias while the drone
   * rested before the ground start (on the logged take-offs: rms error at wake 0.12 deg/s, from 0: 0.20) */
  const float w = s->gyroBiasSeeded ? dt / (CVS_GYRO_BIAS_TAU_S + dt) : 1.0f;
  s->gyroBiasSeeded = true;
  for (int k = 0; k < 3; k++) {
    const float x = -in->rateCorr[k];
    if (isfinite(x)) {
      const float b = s->gyroBias[k] + w * (x - s->gyroBias[k]);
      s->gyroBias[k] = fminf(fmaxf(b, -CVS_GYRO_BIAS_MAX_RADS), CVS_GYRO_BIAS_MAX_RADS);
    }
  }
}

/* ---- public ------------------------------------------------------------------ */

bool camVzShadowInit(camVzShadow_t *s, const camVzObsConfig_t *cfg)
{
  memset(s, 0, sizeof(*s));
  const bool ok = camVzObsInit(&s->obs, cfg);
  s->phase = CVS_IDLE;
  s->g0 = 1.0f;
  s->lastDecision = CAM_VZ_F_DUPLICATE;
  camVzObsGet(&s->obs, &s->out);
  return ok;
}

/* The phases on the ground (IDLE, GROUND, LANDED) and the sortie start: in
 * flash, the airborne step stays in IRAM (CAM_VZ_HOT). */
static void __attribute__((noinline)) stepGround(camVzShadow_t *s, const camVzShadowIn_t *in, float a, bool motors,
                                                 uint32_t dtUs, uint32_t tRel)
{
  switch (s->phase) {
    case CVS_IDLE:
      if (in->restingOnGround && in->still) {
        g0Add(s, in->timeUs, a);
      }
      if (s->out.fault) {
        camVzObsReset(&s->obs);   /* e.g. CAM_VZ_MAX_RUN_US on the bench; the next sortie resets anyway */
      }
      break;
    case CVS_GROUND:
      if (motors) {
        g0Add(s, in->timeUs, a);
        const bool acc = accLiftoff(s, a, dtUs);
        if (in->thrust >= CVS_LIFT_THRUST) {
          liftoff(s, CVS_LIFT_BY_THRUST, tRel);
        } else if (acc) {
          liftoff(s, CVS_LIFT_BY_ACC, tRel);
        } else if (in->liftoffHint) {
          liftoff(s, CVS_LIFT_BY_HINT, tRel);
        }
      } else {
        detectorReset(s);
        if (in->restingOnGround) {
          s->phase = CVS_IDLE;   /* take-off abandoned on the ground */
        }
      }
      break;
    case CVS_LANDED:
      if (motors) {
        const bool acc = accLiftoff(s, a, dtUs);
        if (in->thrust >= CVS_LIFT_THRUST) {
          liftoff(s, CVS_LIFT_BY_THRUST, tRel);
        } else if (acc) {
          liftoff(s, CVS_LIFT_BY_ACC, tRel);
        } else if (in->liftoffHint) {
          liftoff(s, CVS_LIFT_BY_HINT, tRel);
        }
      } else {
        detectorReset(s);
        if (in->restingOnGround) {
          s->phase = CVS_IDLE;
        }
      }
      break;
    case CVS_AIR:
      break;
  }
}

static void __attribute__((noinline)) sortieStartCold(camVzShadow_t *s, uint64_t timeUs)
{
  sortieStart(s, timeUs);
}

CAM_VZ_HOT void camVzShadowStep(camVzShadow_t *s, const camVzShadowIn_t *in)
{
  uint32_t dtUs = 0;
  if (s->hasT && in->timeUs > s->lastUs) {
    const uint64_t d = in->timeUs - s->lastUs;
    dtUs = d > CVS_DT_MAX_US ? CVS_DT_MAX_US : (uint32_t)d;
  }
  if (!s->hasT || in->timeUs > s->lastUs) {
    s->lastUs = in->timeUs;
    s->hasT = true;
  }
  const float a = in->accUpG;
  const bool motors = isfinite(in->thrust) && in->thrust > 0.0f;

  /* A take-off without a ground stop (the soft-start disarmed by a fall while
   * idle) starts a sortie too. */
  if (in->groundStart || (s->phase == CVS_IDLE && motors)) {
    sortieStartCold(s, in->timeUs);
  }
  const uint32_t tRel = sinceSortie(s, in->timeUs);

  switch (s->phase) {
    case CVS_IDLE:
    case CVS_GROUND:
    case CVS_LANDED:
      stepGround(s, in, a, motors, dtUs, tRel);
      break;
    case CVS_AIR:
      if (!motors) {
        /* touchdown: the values before the first impact of the cluster that reaches into the last 0.5 s, or
         * the last airborne ones */
        s->touchdown = true;
        if (s->hasImpact && tRel - s->tImpact <= CVS_IMPACT_WINDOW_US) {
          s->tdImpact = true;
          s->tdH = s->impH;
          s->tdV = s->impV;
          s->tdBiasG = s->impBiasG;
        } else {
          s->tdImpact = false;
          s->tdH = s->prevH;
          s->tdV = s->prevV;
          s->tdBiasG = s->prevBiasG;
        }
        s->phase = CVS_LANDED;
      } else if (tRel - s->tLift >= CVS_IMPACT_MIN_AIR_US &&
                 ((isfinite(a) && fmaxf(a, isfinite(in->accMaxG) ? in->accMaxG : a) > CVS_IMPACT_G) ||
                  in->gyroMaxDps > CVS_IMPACT_DPS)) {
        /* a new cluster after a 0.5 s gap; within one, its first impact's values stay */
        if (!s->hasImpact || tRel - s->tImpact > CVS_IMPACT_WINDOW_US) {
          s->hasImpact = true;
          s->impH = s->prevH;
          s->impV = s->prevV;
          s->impBiasG = s->prevBiasG;
        }
        s->tImpact = tRel;   /* the cluster's latest */
      }
      break;
  }

  /* in-flight gyro bias: fed while the motors run in a sortie (on the ground they spin up first) */
  if (s->sortie > 0 && motors) {
    gyroBiasStep(s, in, dtUs);
  }

  /* The filter on 10 ms means (CVS_OBS_BLOCK_US): its integrals hold each
   * sample until the next one, as with the 100 Hz recorder data it was tuned
   * on. A non-finite sample goes through at once (the filter faults on it).
   * Called once per ~10 ms glue block (cam_vz_glue.c) every call must step:
   * the tolerance absorbs blocks a few us short (CVS_OBS_BLOCK_TOL_US). */
  if (!s->blkHasT) {
    s->blkHasT = true;
    s->blkT0 = in->timeUs;
  }
  if (isfinite(a)) {
    s->blkSum += a;
    s->blkN++;
  }
  if (!isfinite(a) || (in->timeUs > s->blkT0 && in->timeUs - s->blkT0 + CVS_OBS_BLOCK_TOL_US >= CVS_OBS_BLOCK_US)) {
    const float mean = (isfinite(a) && s->blkN > 0) ? s->blkSum / (float)s->blkN : a;
    camVzObsImu(&s->obs, in->timeUs, mean, s->phase != CVS_AIR, isfinite(in->thrust) ? in->thrust : 0.0f);
    camVzObsGet(&s->obs, &s->out);
    s->blkSum = 0.0f;
    s->blkN = 0;
    s->blkT0 = in->timeUs;
  }

  if (s->phase == CVS_AIR) {
    const camVzObsOut_t *o = &s->out;
    s->airUs += dtUs;
    if (o->h > s->hMax) {
      s->hMax = o->h;
    }
    if (o->gainTracked) {
      s->gainLast = o->gain;
      s->gainEver = true;
    }
    if (s->hasWake && tRel >= s->tWake) {
      const uint32_t since = tRel - s->tWake;
      s->camAirUs += dtUs;
      if (o->healthy) {
        s->okUs += dtUs;
        if (!s->hasFirstOk) {
          s->hasFirstOk = true;
          s->firstOkUs = since;
        }
      }
      if (!s->hasBias1 && since >= CVS_BIAS_AT1_US) {
        s->hasBias1 = true;
        s->bias1G = o->biasG;
      }
      if (!s->hasBias2 && since >= CVS_BIAS_AT2_US) {
        s->hasBias2 = true;
        s->bias2G = o->biasG;
        s->sigBias2G = o->sigmaBiasG;
      }
    }
    if (o->sinceAcceptUs != UINT32_MAX && o->sinceAcceptUs > s->gapMaxUs) {
      s->gapMaxUs = o->sinceAcceptUs;
    }
    s->prevH = o->h;
    s->prevV = o->v;
    s->prevBiasG = o->biasG;
  }
  if (s->out.fault && s->sortie > 0) {
    s->fault = true;
  }
}

camVzObsDecision_t camVzShadowFrame(camVzShadow_t *s, const camVzObsFrame_t *f, uint64_t nowUs)
{
  /* the gyro angles without the in-flight gyro bias (frames on the ground are skipped anyway) */
  camVzObsFrame_t fc = *f;
  const float dt = (float)f->dtUs * 1e-6f;
  for (int k = 0; k < 3; k++) {
    fc.gyroAngle[k] -= s->gyroBias[k] * dt;
  }
  const camVzObsDecision_t d = camVzObsFrame(&s->obs, &fc, nowUs);
  if (d != CAM_VZ_F_DUPLICATE) {
    s->lastDecision = d;
  }
  if (d != CAM_VZ_F_DUPLICATE && d != CAM_VZ_F_SKIPPED && s->sortie > 0 && !s->hasWake) {
    s->hasWake = true;
    s->tWake = sinceSortie(s, s->lastUs);
  }
  camVzObsGet(&s->obs, &s->out);
  if (s->out.fault && s->sortie > 0) {
    s->fault = true;
  }
  return d;
}

void camVzShadowVzInput(const camVzShadow_t *s, float vzG0, vzInput_t *in)
{
  const camVzObsOut_t *o = &s->out;
  const bool valid = !o->fault && isfinite(o->v) && isfinite(o->biasHeldG) && isfinite(vzG0);
  in->camVzValid = valid;
  /* The filter's outputs are up to one 10 ms block old: the sortie phase
   * (updated every step) decides on the ground. */
  in->camHealthy = valid && o->healthy && o->airborne && s->phase == CVS_AIR;
  in->camVz = valid ? o->v : 0.0f;
  /* the held offset: biasG itself unless the late offset noise is past its step (cam_vz_obs.h) */
  in->camBias = valid ? o->biasHeldG + 1.0f - vzG0 : 0.0f;
}

CAM_VZ_HOT void camVzShadowRecord(const camVzShadow_t *s, camVzShadowRec_t *r)
{
  const camVzObsOut_t *o = &s->out;
  r->hMm = sat16(o->h * 1000.0f);
  r->vMms = sat16(o->v * 1000.0f);
  r->biasCmg = sat16(o->biasG * 1e5f);
  r->gainDpm = satu16(o->gain * 10.0f);
  r->flags = (uint8_t)((o->airborne ? CVS_REC_AIRBORNE : 0) | (o->healthy ? CVS_REC_HEALTHY : 0) |
                       (o->gainTracked ? CVS_REC_GAIN : 0) | (o->fault ? CVS_REC_FAULT : 0) |
                       (((unsigned)s->lastDecision & 0x0Fu) << 4));
  const float sv = o->sigmaV * 1000.0f;
  r->sigVMms = !(sv > 0.0f) ? 0 : (sv >= 255.0f ? 255 : (uint8_t)lrintf(sv));
}

void camVzShadowFlightStats(const camVzShadow_t *s, fsumCamVz_t *out)
{
  memset(out, 0, sizeof(*out));
  if (s->sortie == 0) {
    return;
  }
  const bool airNow = s->phase == CVS_AIR;
  out->flags = (uint8_t)(FSUM_CVZ_RAN | (s->lifted ? FSUM_CVZ_LIFT : 0) |
                         (s->liftBy == CVS_LIFT_BY_ACC ? FSUM_CVZ_LIFT_ACC : 0) |
                         (s->touchdown ? FSUM_CVZ_TD : 0) | (s->touchdown && s->tdImpact ? FSUM_CVZ_IMPACT : 0) |
                         (s->gainEver ? FSUM_CVZ_GAIN : 0) | (s->priorSet ? FSUM_CVZ_PRIOR : 0) |
                         (s->fault ? FSUM_CVZ_FAULT : 0));
  out->firstOkDs = s->hasFirstOk ? (uint8_t)(s->firstOkUs / 100000u > 254u ? 254u : s->firstOkUs / 100000u) : 255;
  out->g0Dmg = s->priorSet ? sat16((s->g0 - 1.0f) * 1e4f) : FSUM_CVZ_NONE;
  out->bias3Cmg = s->hasBias1 ? sat16(s->bias1G * 1e5f) : FSUM_CVZ_NONE;
  out->bias5Cmg = s->hasBias2 ? sat16(s->bias2G * 1e5f) : FSUM_CVZ_NONE;
  if (s->touchdown && !airNow) {
    out->biasTdCmg = sat16(s->tdBiasG * 1e5f);
  } else if (s->lifted) {
    out->biasTdCmg = sat16((airNow ? s->out.biasG : s->prevBiasG) * 1e5f);
  } else {
    out->biasTdCmg = FSUM_CVZ_NONE;
  }
  out->sigB5Cmg = s->hasBias2 ? satu16(s->sigBias2G * 1e5f) : 0;
  out->gainDpm = s->gainEver ? satu16(s->gainLast * 10.0f) : 0;
  const bool td = s->touchdown && !airNow;
  out->hTdMm = td ? sat16(s->tdH * 1000.0f) : FSUM_CVZ_NONE;
  out->vTdMms = td ? sat16(s->tdV * 1000.0f) : FSUM_CVZ_NONE;
  out->hMaxMm = s->lifted ? sat16(s->hMax * 1000.0f) : FSUM_CVZ_NONE;
  out->airCs = satu16u(s->airUs / 10000u);
  out->camAirCs = satu16u(s->camAirUs / 10000u);
  out->okCs = satu16u(s->okUs / 10000u);
  out->gapMaxCs = satu16u(s->gapMaxUs / 10000u);
  out->accepted = satu16u(s->obs.cnt.accepted);
  out->rejected = satu16u(s->obs.cnt.rejected);
  out->alarms = satu8u(s->obs.cnt.alarms);
  out->yawResets = satu8u(s->obs.cnt.yawResets);
}

void camVzShadowLateStats(const camVzShadow_t *s, fsumCvLate_t *out)
{
  memset(out, 0, sizeof(*out));
  if (s->sortie == 0) {
    return;
  }
  const camVzObs_t *o = &s->obs;
  out->flags = (uint8_t)(FSUM_CVL_RAN | (o->late ? FSUM_CVL_STEP : 0) | (s->lateCfgBad ? FSUM_CVL_CFG_BAD : 0));
  out->qLate = (uint8_t)lrintf(o->cfg.qLateMult);   /* 1..10 (camVzObsConfigValid) */
  out->qLateS = (uint8_t)lrintf(o->cfg.qLateS);     /* 20..120 */
  out->guards = satu8u(o->lateGuards);
  out->lateDs = satu16u(o->lateUs / 100000u);
  /* the offset drift rate T2 (cam_vz_obs.h "Offset drift rate") */
  camVzObsT2_t t2;
  camVzObsT2Get(o, &t2);
  out->flags |= (uint8_t)((t2.on ? FSUM_CVL_T2_ON : 0) | (s->t2CfgBad ? FSUM_CVL_T2_CFG_BAD : 0));
  out->t2K = (uint8_t)lrintf(o->cfg.t2kR * 100.0f);           /* 0..200 (camVzObsConfigValid) */
  out->t2BrEnd = sat8(t2.t2RateMgS * 100.0f);
  out->t2BrMin = sat8(t2.minMgS * 100.0f);
  out->t2BrMax = sat8(t2.maxMgS * 100.0f);
  out->t2OnDs = satu16u(t2.onUs / 100000u);
  out->t2PriorUgs = sat16(o->cfg.t2PriorMgS * 1000.0f);       /* |.| <= 500 */
  /* the healthy offset speed-up (cam_vz_obs.h "Healthy offset speed-up"): which version flew, two bits */
  out->flags |= (uint8_t)((o->cfg.hbMult > 0.0f ? FSUM_CVL_HB_ON : 0) | (s->hbCfgBad ? FSUM_CVL_HB_CFG_BAD : 0));
}

void camVzShadowBenchIn(camVzShadowBench_t *b, camVzShadowIn_t *in)
{
  if (b->mode == CVS_BENCH_OFF) {
    /* The bench sortie ends with real thrust (the caller ended the mode on
     * it): that block starts the real sortie even without a ground start (a
     * hand launch, soft-start off), so no bench air time, drift or prior
     * carries into the flight. */
    if (b->started && isfinite(in->thrust) && in->thrust > 0.0f) {
      in->groundStart = true;
    }
    b->started = false;
    return;
  }
  in->groundStart = !b->started;
  b->started = true;
  if (!(in->thrust >= CVS_LIFT_THRUST)) {
    in->thrust = CVS_LIFT_THRUST;
  }
  in->restingOnGround = false;
  in->still = false;
  in->liftoffHint = false;
}

void camVzShadowBenchFrame(const camVzShadowBench_t *b, camVzObsFrame_t *f)
{
  if (b->mode != CVS_BENCH_FULL && b->mode != CVS_BENCH_SPIN) {
    return;
  }
  f->ok = true;
  if (!(f->texture >= 20.0f)) {
    f->texture = 20.0f;
  }
  if (!(f->residual <= 1.0f)) {
    f->residual = 1.0f;
  }
  if (!isfinite(f->dx)) {
    f->dx = 0.0f;
  }
  if (!isfinite(f->dy)) {
    f->dy = 0.0f;
  }
  if (!isfinite(f->scale)) {
    f->scale = 0.0f;
  }
  for (int k = 0; k < 2; k++) {
    if (!isfinite(f->gyroAngle[k])) {
      f->gyroAngle[k] = 0.0f;
    }
  }
  f->gyroAngle[2] = 0.0f;
}

const char *camVzShadowPhaseName(camVzShadowPhase_t p)
{
  switch (p) {
    case CVS_IDLE: return "idle";
    case CVS_GROUND: return "ground";
    case CVS_AIR: return "air";
    case CVS_LANDED: return "landed";
    default: return "?";
  }
}
