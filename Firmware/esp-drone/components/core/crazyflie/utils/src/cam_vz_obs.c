/*
 * ESP-Drone Firmware
 *
 * cam_vz_obs.c - Camera + accelerometer vertical observer (pure), see
 * cam_vz_obs.h. Comments naming observer.py point at the reference
 * implementation of the camera height study.
 */
#include <math.h>
#include <string.h>

#include "cam_vz_obs.h"
#include "camera_cadence.h"
#include "camera_profile.h"

#define P_FLOOR 1e-8f   /* variance of h and v on the ground (observer.py) */

void camVzObsDefaultConfig(camVzObsConfig_t *c)
{
  memset(c, 0, sizeof(*c));
  c->sigAcc = 0.25f;
  c->qBiasG = 0.0003f;
  c->sigPx = 0.25f;
  c->sigGrel = 0.20f;
  c->sigCamV = 0.02f;
  c->gateK = 3.5f;
  c->relockS = 1.5f;
  c->relockSigV = 0.10f;
  c->hpTauS = 1.0f;
  c->rlsTauS = 3.0f;
  c->gMin = 15.0f;
  c->gMax = 500.0f;
  c->rMax = -0.5f;
  c->minWRms = 0.03f;
  c->minN = 12.0f;
  c->gapResetS = 0.4f;
  c->warmN = 4;
  c->useCamera = true;
  c->minTexture = 8.0f;
  c->maxResidual = 8.0f;
  c->maxYawRate = 0.35f;
  c->dyOffsetPx = 0.0f;
  c->maxExpRate = 0.3f;
  c->expAlpha = 0.2f;
  c->maxDxPx = 1.5f;
  c->cusumK = 1.0f;
  c->cusumH = 5.0f;
  c->cusumHoldS = 3.0f;
  c->yawResetRad = 0.35f;
  c->gDefault = 80.0f;
  c->sigGrelDefault = 0.6f;
  c->gainReanchorRel = 0.3f;
  c->kyGx = 1.2836f;
  c->kyGy = 99.790f;
  c->kyGz = -2.7643f;
  c->kyC = 0.0294f;
  c->kxGx = -1.0f;
  c->kxGy = -4.9f;
  c->kxGz = 102.4f;
  c->gyroOffsetUs = 15000;
  c->wakeUs = 1500000;
  c->predictPeriodUs = 10000;
  c->maxFrameAgeUs = 300000;
  c->healthyWindowUs = 2000000;
  c->qLateMult = 1.0f;
  c->qLateS = 40.0f;
  c->t2kR = 0.0f;
  c->t2StartS = CAM_VZ_T2_START_S;
  c->t2PriorMgS = CAM_VZ_T2_PRIOR_MGS;
  c->t2BoundMgS = CAM_VZ_T2_BOUND_MGS;
  c->hbMult = 0.0f;
#ifdef ESPFLY_NATIVE_VGA
  const float ratio = CAM_CADENCE_REF_S / (CAM_CADENCE_VGA_PERIOD_US * 1e-6f);
  c->cadenceAware = true;
  c->minN *= ratio;   /* same effective evidence duration, not 12 low-rate frames */
  c->warmN = (int)ceilf((4.0f / 15.0f) / (CAM_CADENCE_VGA_PERIOD_US * 1e-6f)); /* same physical warm-up */
  c->minGainSamples = 8; /* retain an independent-observation floor */
#if CAMERA_NATIVE_WIDTH == 320
  c->maxResidual = 12.0f;
#endif
  c->sigGrel /= sqrtf(ratio); /* fewer observations: more uncertainty, not more trust */
  c->gyroOffsetUs = 0; /* VGA timestamps already include their physical centre offset */
  /* Startup scale is not measured yet. Feeding the arbitrary 80 px/m
   * fallback into the Kalman state before G is learned can bias velocity
   * and accelerometer offset, then raise CUSUM before the tracked scale
   * arrives. Keep learning G from the independent inertial window, but do
   * not fuse camera velocity until there is a measured scale. No alarm,
   * quality gate or healthy criterion is relaxed. A previously measured
   * G can still become a guess through a commanded yaw as before. */
  c->gDefault = 0.0f;
#endif
}

void camVzObsPlainConfig(camVzObsConfig_t *c)
{
  camVzObsDefaultConfig(c);
  c->maxExpRate = 0.0f;
  c->maxDxPx = 0.0f;
  c->cusumK = -1.0f;
  c->cusumH = 6.0f;
  c->yawResetRad = 0.0f;
  c->gDefault = 0.0f;
  c->gainReanchorRel = 0.0f;
}

bool camVzObsConfigValid(const camVzObsConfig_t *c)
{
  const float f[] = {
    c->sigAcc, c->qBiasG, c->sigPx, c->sigGrel, c->sigCamV, c->gateK, c->relockS, c->relockSigV,
    c->hpTauS, c->rlsTauS, c->gMin, c->gMax, c->rMax, c->minWRms, c->minN, c->gapResetS,
    c->minTexture, c->maxResidual, c->maxYawRate, c->dyOffsetPx, c->maxExpRate, c->expAlpha, c->maxDxPx,
    c->cusumK, c->cusumH, c->cusumHoldS, c->yawResetRad, c->gDefault, c->sigGrelDefault, c->gainReanchorRel,
    c->kyGx, c->kyGy, c->kyGz, c->kyC, c->kxGx, c->kxGy, c->kxGz, c->qLateMult, c->qLateS,
    c->t2kR, c->t2StartS, c->t2PriorMgS, c->t2BoundMgS, c->hbMult,
  };
  for (unsigned i = 0; i < sizeof(f) / sizeof(f[0]); i++) {
    if (!isfinite(f[i])) {
      return false;
    }
  }
  return c->sigAcc > 0.0f && c->qBiasG >= 0.0f && c->sigPx > 0.0f && c->sigGrel >= 0.0f &&
         c->sigCamV >= 0.0f && c->gateK > 0.0f && c->relockS > 0.0f && c->relockS <= 60.0f &&
         c->relockSigV >= 0.0f && c->hpTauS > 0.0f && c->rlsTauS > 0.0f && c->gMin > 0.0f &&
         c->gMax > c->gMin && c->rMax >= -1.0f && c->rMax < 0.0f && c->minWRms >= 0.0f && c->minN >= 3.0f &&
         c->minGainSamples <= 100 && (!c->cadenceAware || c->minGainSamples >= 6) &&
         c->gapResetS > 0.0f && c->gapResetS <= 60.0f && c->warmN <= 100 && c->minTexture >= 0.0f &&
         c->maxResidual > 0.0f && c->maxYawRate > 0.0f && c->expAlpha > 0.0f && c->expAlpha <= 1.0f &&
         c->cusumH > 0.0f && c->cusumHoldS > 0.0f && c->cusumHoldS <= 60.0f && c->sigGrelDefault >= 0.0f &&
         c->gainReanchorRel >= 0.0f && c->gainReanchorRel <= 1.0f &&
         c->kyGy > 0.0f && c->kxGz > 0.0f && c->gyroOffsetUs <= 100000 && c->wakeUs <= 10000000 &&
         c->predictPeriodUs >= 5000 && c->predictPeriodUs <= 20000 &&
         c->maxFrameAgeUs >= 50000 && c->maxFrameAgeUs <= 1000000 &&
         c->healthyWindowUs >= 100000 && c->healthyWindowUs <= 10000000 &&
         c->qLateMult >= 1.0f && c->qLateMult <= CAM_VZ_QLATE_MULT_MAX && c->qLateS >= CAM_VZ_QLATE_S_MIN &&
         c->qLateS <= CAM_VZ_QLATE_S_MAX &&
         c->t2kR >= 0.0f && c->t2kR <= CAM_VZ_T2_KR_MAX && c->t2StartS >= CAM_VZ_T2_S_MIN &&
         c->t2StartS <= CAM_VZ_T2_S_MAX && c->t2BoundMgS > 0.0f && c->t2BoundMgS <= CAM_VZ_T2_BOUND_MAX_MGS &&
         fabsf(c->t2PriorMgS) <= c->t2BoundMgS &&
         /* R1 and T2 together: never tested, refused loudly rather than flown */
         !(c->qLateMult > 1.0f && c->t2kR > 0.0f) &&
         /* the healthy offset speed-up: off, or in its range and alone (with R1 or T2 untested too) */
         (c->hbMult == 0.0f || (c->hbMult >= CAM_VZ_HB_MULT_MIN && c->hbMult <= CAM_VZ_HB_MULT_MAX &&
                                c->qLateMult == 1.0f && c->t2kR == 0.0f));
}

bool camVzObsInit(camVzObs_t *o, const camVzObsConfig_t *cfg)
{
  memset(o, 0, sizeof(*o));
  bool ok = true;
  if (cfg != NULL && camVzObsConfigValid(cfg)) {
    o->cfg = *cfg;
  } else {
    ok = cfg == NULL;
    camVzObsDefaultConfig(&o->cfg);
  }
  camVzObsReset(o);
  return ok;
}

void camVzObsReset(camVzObs_t *o)
{
  const camVzObsConfig_t c = o->cfg;
  memset(o, 0, sizeof(*o));
  o->cfg = c;
  o->P[0][0] = P_FLOOR;
  o->P[1][1] = P_FLOOR;
  const float s = CAM_VZ_PRIOR_SIGMA_VAGUE_G * CAM_VZ_G;
  o->P[2][2] = s * s;
}

bool camVzObsSetBiasPrior(camVzObs_t *o, float b0G, float sigmaG)
{
  if (!isfinite(b0G) || !isfinite(sigmaG) || sigmaG < 0.0f || o->lifted) {
    return false;
  }
  const float b = b0G * CAM_VZ_G;
  const float s = sigmaG * CAM_VZ_G;
  o->x[2] = b;
  o->b0 = b;
  o->P[2][2] = s * s;
  o->P[0][2] = o->P[2][0] = 0.0f;
  o->P[1][2] = o->P[2][1] = 0.0f;
  return true;
}

bool camVzObsSetLiftoffTime(camVzObs_t *o, uint64_t timeUs)
{
  if (!o->started || !o->lifted) {
    return false;
  }
  int64_t rel = (int64_t)timeUs - (int64_t)o->tStartUs;
  if (rel < 0) {
    rel = 0;
  } else if (rel > (int64_t)o->tIn) {
    rel = o->tIn;
  }
  if (o->hasFirstLift && o->tFirstLift == o->tLift) {
    o->tFirstLift = (uint32_t)rel;   /* the first lift-off itself (the late offset noise's clock) */
  }
  o->tLift = (uint32_t)rel;
  return true;
}

static CAM_VZ_HOT uint32_t usOf(float s)
{
  return (uint32_t)(s * 1e6f + 0.5f);
}

static CAM_VZ_HOT bool stateFinite(const camVzObs_t *o)
{
  for (int i = 0; i < 3; i++) {
    if (!isfinite(o->x[i])) {
      return false;
    }
    for (int j = 0; j < 3; j++) {
      if (!isfinite(o->P[i][j])) {
        return false;
      }
    }
  }
  return true;
}

/* ---- raw-integral ring ---------------------------------------------------- */

static CAM_VZ_HOT uint16_t ringIndex(const camVzObs_t *o, uint16_t k)
{
  /* k-th entry counted from the oldest */
  return (uint16_t)((o->ringHead + CAM_VZ_RING_LEN - o->ringCount + k) % CAM_VZ_RING_LEN);
}

static CAM_VZ_HOT camVzObsRing_t *ringNewest(camVzObs_t *o)
{
  return &o->ring[(o->ringHead + CAM_VZ_RING_LEN - 1) % CAM_VZ_RING_LEN];
}

static CAM_VZ_HOT void ringPush(camVzObs_t *o, uint32_t t, float a)
{
  camVzObsRing_t *e = &o->ring[o->ringHead];
  e->tUs = t;
  e->va = o->va;
  e->za = o->za;
  e->a = a;
  o->ringHead = (uint16_t)((o->ringHead + 1) % CAM_VZ_RING_LEN);
  if (o->ringCount < CAM_VZ_RING_LEN) {
    o->ringCount++;
  }
}

/* The double integral only enters as differences inside the ring: shift it
 * (and every entry) back to zero to keep float resolution. */
static CAM_VZ_HOT void ringRebase(camVzObs_t *o)
{
  const float z = o->za;
  for (uint16_t k = 0; k < o->ringCount; k++) {
    o->ring[ringIndex(o, k)].za -= z;
  }
  o->za = 0.0f;
}

/* Raw double integral at s (us since the start) with the zero-order-hold
 * acceleration of the entry before it (observer.py _Za_at). */
static bool zaAt(const camVzObs_t *o, int64_t s, float *za)
{
  if (o->ringCount == 0) {
    return false;
  }
  const camVzObsRing_t *oldest = &o->ring[ringIndex(o, 0)];
  const camVzObsRing_t *newest = &o->ring[ringIndex(o, (uint16_t)(o->ringCount - 1))];
  if (s < (int64_t)oldest->tUs || s > (int64_t)newest->tUs) {
    return false;
  }
  uint16_t lo = 0;
  uint16_t hi = (uint16_t)(o->ringCount - 1);
  while (lo < hi) {
    const uint16_t mid = (uint16_t)((lo + hi + 1) / 2);
    if ((int64_t)o->ring[ringIndex(o, mid)].tUs <= s) {
      lo = mid;
    } else {
      hi = (uint16_t)(mid - 1);
    }
  }
  const camVzObsRing_t *e = &o->ring[ringIndex(o, lo)];
  const float tau = (float)(uint32_t)(s - (int64_t)e->tUs) * 1e-6f;   /* >= 0, < the ring span */
  *za = e->za + e->va * tau + 0.5f * e->a * tau * tau;
  return true;
}

/* ---- Kalman filter --------------------------------------------------------- */

static CAM_VZ_HOT void mat3Mul(const float A[3][3], const float B[3][3], float C[3][3])
{
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
    }
  }
}

/* C = A B A^T, symmetrised */
static CAM_VZ_HOT void mat3Sandwich(const float A[3][3], const float B[3][3], float C[3][3])
{
  float AB[3][3];
  mat3Mul(A, B, AB);
  for (int i = 0; i < 3; i++) {
    for (int j = i; j < 3; j++) {
      const float v = AB[i][0] * A[j][0] + AB[i][1] * A[j][1] + AB[i][2] * A[j][2];
      C[i][j] = v;
      C[j][i] = v;
    }
  }
}

/* Propagates the filter over d seconds in which the raw specific force minus
 * 1 g integrated to dV (m/s) and dZ (m, from rest at the block start); the
 * offset's random walk is qBiasG, x qLateMult with late (the late offset
 * noise, header), x hbMult with hb (the healthy offset speed-up, header;
 * never both: camVzObsConfigValid). */
static CAM_VZ_HOT void predict(camVzObs_t *o, float d, float dV, float dZ, bool late, bool hb)
{
  const camVzObsConfig_t *c = &o->cfg;
  float *x = o->x;
  x[0] += x[1] * d + dZ - 0.5f * x[2] * d * d;
  x[1] += dV - x[2] * d;
  const float F[3][3] = {{1.0f, d, -0.5f * d * d}, {0.0f, 1.0f, -d}, {0.0f, 0.0f, 1.0f}};
  float N[3][3];
  mat3Sandwich(F, o->P, N);
  const float gq[2] = {0.5f * d * d, d};
  const float s2 = c->sigAcc * c->sigAcc;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      N[i][j] += s2 * gq[i] * gq[j];
    }
  }
  float qb = c->qBiasG * CAM_VZ_G;
  if (late) {
    qb *= c->qLateMult;   /* the late offset noise (header) */
  } else if (hb) {
    qb *= c->hbMult;      /* the healthy offset speed-up (header) */
  }
  N[2][2] += qb * qb * d;
  memcpy(o->P, N, sizeof(N));
}

/* On the floor h = v = 0 exactly (observer.py imu, on_ground). */
static CAM_VZ_HOT void groundConstraint(camVzObs_t *o)
{
  o->x[0] = 0.0f;
  o->x[1] = 0.0f;
  for (int k = 0; k < 3; k++) {
    o->P[0][k] = o->P[k][0] = 0.0f;
    o->P[1][k] = o->P[k][1] = 0.0f;
  }
  o->P[0][0] = P_FLOOR;
  o->P[1][1] = P_FLOOR;
}

static CAM_VZ_HOT void closeBlock(camVzObs_t *o)
{
  const float d = (float)(o->tIn - o->tState) * 1e-6f;
  /* The newest entry's acceleration over this block: the single sample's
   * (already stored at its push) or the one matching the block's double integral. */
  if (o->bN > 1 && d > 0.0f) {
    ringNewest(o)->a = 2.0f * o->bdZ / (d * d);
  }
  /* the offset drift rate (T2, header): set at a frame (t2Pre), applied here only in the air with an accepted update
   * this recent (a gap or the ground freezes the offset as today's filter would); with t2kR 0 one flag test */
  if (o->t2Init && !o->onGround && o->hasAccept && o->tIn - o->lastAccept < CAM_VZ_T2_ACC_US) {
    o->x[2] += o->bRate * d;
    o->t2Us += o->tIn - o->tState;
  }
  /* the healthy offset speed-up: the frame's flag (hbPost()), in the air, with an accepted update this recent (the
   * healthy window: a camera gap stops it as it stops the controller's use of the camera); with hbMult 0 one flag
   * test */
  const bool hb = o->hbOn && !o->onGround && o->tIn - o->lastAccept < o->cfg.healthyWindowUs;
  if (hb) {
    o->hbUs += o->tIn - o->tState;
  } else if (o->onGround) {
    o->hbOn = false;   /* the ground (a bounce too): off until a frame after the re-lift sets it again */
  }
  /* the late offset noise: decided at each frame (latePre(), latePost()), never on the ground */
  predict(o, d, o->bdV, o->bdZ, o->lateOn && !o->onGround, hb);
  if (o->onGround) {
    groundConstraint(o);
  }
  o->tState = o->tIn;
  if (fabsf(o->za) > CAM_VZ_ZA_REBASE_M) {
    ringRebase(o);
  }
  ringPush(o, o->tState, o->aPrev);
  o->bdV = 0.0f;
  o->bdZ = 0.0f;
  o->bN = 0;
  if (!stateFinite(o)) {
    o->fault = true;
  }
}

static CAM_VZ_HOT void sampleFlags(camVzObs_t *o, uint32_t rel, bool onGround, float thrust)
{
  if (thrust > 0.0f && !o->throttled) {
    o->throttled = true;
    o->tThrottle = rel;
  }
  if (!onGround && (!o->lifted || o->onGround)) {
    o->lifted = true;
    o->tLift = rel;
  }
  o->onGround = onGround;
}

CAM_VZ_HOT void camVzObsImu(camVzObs_t *o, uint64_t timeUs, float accUpG, bool onGround, float thrust)
{
  if (o->fault) {
    return;
  }
  if (!isfinite(accUpG) || !isfinite(thrust)) {
    o->fault = true;
    return;
  }
  const float a = (accUpG - 1.0f) * CAM_VZ_G;
  if (!o->started) {
    o->started = true;
    o->tStartUs = timeUs;
    o->tIn = 0;
    o->tState = 0;
    o->aPrev = a;
    o->va = 0.0f;
    o->za = 0.0f;
    ringPush(o, 0, a);
    sampleFlags(o, 0, onGround, thrust);
    return;
  }
  if (timeUs <= o->tStartUs + o->tIn) {
    o->cnt.imuIgnored++;
    return;
  }
  const uint64_t rel64 = timeUs - o->tStartUs;
  if (rel64 > CAM_VZ_MAX_RUN_US) {
    o->fault = true;
    return;
  }
  const uint32_t rel = (uint32_t)rel64;
  const uint32_t dtUs = rel - o->tIn;
  const float dt = (float)dtUs * 1e-6f;
  const float ap = o->aPrev;             /* zero-order hold of the previous sample */
  o->za += o->va * dt + 0.5f * ap * dt * dt;
  o->va += ap * dt;
  o->bdZ += o->bdV * dt + 0.5f * ap * dt * dt;
  o->bdV += ap * dt;
  o->bN++;
  o->tIn = rel;
  o->aPrev = a;
  sampleFlags(o, rel, onGround, thrust);
  /* Close the block at the sample nearest to predictPeriodUs after its start
   * (every sample at 100 Hz, every 10th at 1 kHz). */
  if ((rel - o->tState) + dtUs / 2 >= o->cfg.predictPeriodUs) {
    closeBlock(o);
  }
}

/* ---- gain tracker (observer.py GainTracker) -------------------------------- */

static void gainUpdate(camVzObs_t *o, uint32_t tSum, float u, float w, float fwdLn)
{
  const camVzObsConfig_t *c = &o->cfg;
  camVzObsGain_t *g = &o->gt;
  const uint32_t gapSum = 2u * usOf(c->gapResetS);
  if (!g->hasT || tSum <= g->tSumLast || tSum - g->tSumLast > gapSum) {
    if (g->hasT && tSum > g->tSumLast) {
      const float lam = expf(-(float)(tSum - g->tSumLast) * 0.5e-6f / c->rlsTauS);
      g->sxx *= lam;
      g->sxy *= lam;
      g->syy *= lam;
      g->sw *= lam;
    }
    g->uPrev = u;
    g->wPrev = w;
    g->u1 = g->w1 = g->u2 = g->w2 = 0.0f;
    g->nSince = 0;
    g->samples = 0;
    g->hasT = true;
    g->tSumLast = tSum;
    g->validNow = false;
    return;
  }
  const float dt = (float)(tSum - g->tSumLast) * 0.5e-6f;
  g->tSumLast = tSum;
  const float a = c->hpTauS / (c->hpTauS + dt);
  const float u1 = a * (g->u1 + u - g->uPrev);
  const float w1 = a * (g->w1 + w - g->wPrev);
  const float u2 = a * (g->u2 + u1 - g->u1);
  const float w2 = a * (g->w2 + w1 - g->w1);
  g->uPrev = u;
  g->wPrev = w;
  g->u1 = u1;
  g->w1 = w1;
  g->u2 = u2;
  g->w2 = w2;
  g->nSince++;
  const float lam = expf(-dt / c->rlsTauS);
  g->sxx *= lam;
  g->sxy *= lam;
  g->syy *= lam;
  g->sw *= lam;
  if (g->nSince >= c->warmN) {
    g->sxx += w2 * w2;
    g->sxy += w2 * u2;
    g->syy += u2 * u2;
    g->sw += 1.0f;
    if (g->samples < UINT32_MAX) g->samples++;
  }
  g->validNow = false;
  if (g->sw >= c->minN && g->samples >= c->minGainSamples && g->sxx > 0.0f && g->syy > 0.0f) {
    g->r = g->sxy / sqrtf(g->sxx * g->syy);
    const float gg = -g->sxy / g->sxx;
    if (g->r <= c->rMax && sqrtf(g->sxx / g->sw) >= c->minWRms && gg >= c->gMin && gg <= c->gMax) {
      g->g = gg;
      g->hasG = true;
      g->validNow = true;
      o->fwdLnAtG = fwdLn;   /* measured at the distance now: the forward scale starts again from here */
      if (!g->hasFirst) {
        g->hasFirst = true;
        g->tSumFirst = tSum;
      }
    }
  }
}

/* The gain an update uses now and its relative error. */
static CAM_VZ_HOT bool selectGain(const camVzObs_t *o, float *G, float *sg)
{
  const camVzObsConfig_t *c = &o->cfg;
  if (o->gt.hasG) {
    *G = o->gt.g;
    *sg = c->sigGrel;
  } else if (o->hasGuess) {
    *G = o->gGuess;
    *sg = c->sigGrelDefault;
  } else if (c->gDefault > 0.0f) {
    *G = c->gDefault;
    *sg = c->sigGrelDefault;
  } else {
    return false;
  }
  return true;
}

/* The factor the gain in use follows the forward progress since it was taken by (header, "Gain through a forward
 * move"): exactly 1 while the frames' fwdLn stays where it was. */
static float fwdScale(const camVzObs_t *o, float fwdLn)
{
  const float d = fwdLn - o->fwdLnAtG;
  if (d == 0.0f || !isfinite(d)) {
    return 1.0f;
  }
  return expf(fminf(fmaxf(d, -CAM_VZ_FWD_LN_MAX), CAM_VZ_FWD_LN_MAX));
}

static void inflateSigmaV(camVzObs_t *o)
{
  const float s = sqrtf(fmaxf(o->P[1][1], 0.0f)) + o->cfg.relockSigV;
  o->P[1][1] = s * s;
}

/* The take-off climb as the speed estimate sees it (gain re-anchor window):
 * seen once v > CAM_VZ_REANCHOR_CLIMB_V, ended at the first frame after that
 * with v < CAM_VZ_REANCHOR_STOP_V. Once per reset: a later re-lift (a bounce)
 * keeps the first climb's end. */
static void climbTrack(camVzObs_t *o)
{
  if (o->raClimbEnd) {
    return;
  }
  if (o->x[1] > CAM_VZ_REANCHOR_CLIMB_V) {
    o->raClimb = true;
  } else if (o->raClimb && o->x[1] < CAM_VZ_REANCHOR_STOP_V) {
    o->raClimbEnd = true;
    /* rounded up: < 36000 in the 30 min run */
    o->raClimbEnd50ms = (uint16_t)((o->tState + (CAM_VZ_REANCHOR_TICK_US - 1u)) / CAM_VZ_REANCHOR_TICK_US);
  }
}

/* Re-anchors are allowed only in the take-off climb and the tracker's lag
 * behind it: up to CAM_VZ_REANCHOR_AFTER_S after the climb ended and never
 * later than CAM_VZ_REANCHOR_MAX_S after the lift-off. */
static bool reanchorWindow(const camVzObs_t *o)
{
  if (o->tState > o->tLift && o->tState - o->tLift > usOf(CAM_VZ_REANCHOR_MAX_S)) {
    return false;
  }
  if (!o->raClimbEnd) {
    return true;
  }
  const uint32_t end = (uint32_t)o->raClimbEnd50ms * CAM_VZ_REANCHOR_TICK_US;
  return o->tState <= end || o->tState - end <= usOf(CAM_VZ_REANCHOR_AFTER_S);
}

/* ---- camera frames (observer.py VertObserver.frame) ------------------------ */

static camVzObsDecision_t frameUpdate(camVzObs_t *o, const camVzObsFrame_t *f, uint64_t nowUs,
                                      int64_t t0, int64_t t1, bool seqOk)
{
  const camVzObsConfig_t *c = &o->cfg;
  const float dt = (float)f->dtUs * 1e-6f;
  const float *ga = f->gyroAngle;

  /* view change: the accumulated yaw restarts the tracker; its G becomes a guess */
  if (c->yawResetRad > 0.0f && isfinite(ga[2])) {
    o->yawAcc += ga[2];
    if (fabsf(o->yawAcc) > c->yawResetRad) {
      o->yawAcc = 0.0f;
      /* the guess: the G in use, with the forward progress since it was taken (header) */
      const float sc = fwdScale(o, f->fwdLn);
      if (o->gt.hasG) {
        o->hasGuess = true;
        o->gGuess = o->gt.g * sc;
      } else if (o->hasGuess) {
        o->gGuess *= sc;
      }
      o->fwdLnAtG = f->fwdLn;
      memset(&o->gt, 0, sizeof(o->gt));
      o->gAnchorSet = false;   /* the next tracked G (a new view) is the new anchor */
      o->csPos = 0.0f;
      o->csNeg = 0.0f;
      o->alarm = false;
      o->cnt.yawResets++;
    }
  }
  if (f->dtUs > 0 && isfinite(f->scale)) {
    const float alpha = c->cadenceAware ? camCadenceExpAlpha(dt) : c->expAlpha;
    o->expEma += alpha * (f->scale / dt - o->expEma);
  }
  climbTrack(o);

  const uint64_t age = nowUs > f->frameTimeUs ? nowUs - f->frameTimeUs : 0;
  const bool finiteIn = isfinite(f->dx) && isfinite(f->dy) && isfinite(ga[0]) && isfinite(ga[1]) &&
                        isfinite(ga[2]) && isfinite(f->texture) && isfinite(f->residual);
  /* A native QVGA fit may retain high-frequency image disagreement absent
   * from the old 160 reference image. Only admit the extra residual budget
   * on strongly textured frames with bounded residual/gradient; account for
   * the larger measurement noise below. Frame age, yaw, exposure, scale
   * learning, innovation tests and health expiry are unchanged. */
  const bool noiseAware = c->cadenceAware && c->maxResidual == 12.0f;
  const bool extraFit = !noiseAware || f->residual <= 8.0f ||
                        (f->texture >= 64.0f && f->residual * f->residual <= 1.21f * f->texture);
  const bool good = extraFit && f->ok && finiteIn && age < c->maxFrameAgeUs && seqOk && f->dtUs > 0 &&
                    f->texture >= c->minTexture && f->residual < c->maxResidual &&
                    fabsf(ga[2]) < (o->turn ? CAM_VZ_TURN_MAX_YAW_RATE : c->maxYawRate) * fmaxf(dt, 1e-6f);
  if (!good) {
    o->cnt.quality++;
    return CAM_VZ_F_QUALITY;
  }
  if (c->maxExpRate > 0.0f && fabsf(o->expEma) > c->maxExpRate) {
    o->cnt.expRejects++;
    return CAM_VZ_F_EXPANSION;
  }
  const float dxRes = f->dx - (c->kxGx * ga[0] + c->kxGy * ga[1] + c->kxGz * ga[2]);
  const float dxLimit = c->cadenceAware ? camCadenceDxLimit(c->maxDxPx, dt) : c->maxDxPx;
  if (c->maxDxPx > 0.0f && fabsf(dxRes) > dxLimit) {
    o->cnt.latRejects++;
    return CAM_VZ_F_LATERAL;
  }

  /* mean speed over the window predicted from the current state, and the
   * accelerometer-only window speed for the tracker (prior offset ramp removed) */
  float z0, z1;
  if (!zaAt(o, t0, &z0) || !zaAt(o, t1, &z1)) {
    o->cnt.noWindow++;
    return CAM_VZ_F_NO_WINDOW;
  }
  const float wMean = (z1 - z0) / dt;
  /* now - tMid, s (both window ends are inside the ring: >= 0 and < 2 ring spans) */
  const float tau = (float)(uint32_t)(2 * (int64_t)o->tState - t0 - t1) * 0.5e-6f;
  const float vpred = o->x[1] - ringNewest(o)->va + wMean + o->x[2] * tau;
  const float wAcc = wMean - o->b0 * ((float)(uint32_t)(t0 + t1) * 0.5e-6f);   /* t0 + t1 < 2 x 30 min */

  const float dyRes = f->dy - (c->kyGx * ga[0] + c->kyGy * ga[1] + c->kyGz * ga[2] + c->kyC);
  const float u = (dyRes - c->dyOffsetPx) / dt;   /* px/s */
  gainUpdate(o, (uint32_t)(t0 + t1), u, wAcc, f->fwdLn);

  float G, sg;
  if (!c->useCamera || !selectGain(o, &G, &sg)) {
    o->cnt.noGain++;
    return CAM_VZ_F_NO_GAIN;
  }
  /* the update's gain: G at the scene distance now (a forward move since G was taken; the re-anchor below compares
   * the tracker's own G) */
  const float Gm = G * fwdScale(o, f->fwdLn);
  const float y = -u / Gm;
  const float fitNoise = noiseAware ? fmaxf(1.0f, f->residual / 8.0f) : 1.0f;
  const float spx = c->sigPx * fitNoise / dt;
  const float R = (spx * spx + (sg * u) * (sg * u)) / (Gm * Gm) + c->sigCamV * c->sigCamV;
  float PH[3];
  for (int i = 0; i < 3; i++) {
    PH[i] = o->P[i][1] + tau * o->P[i][2];
  }
  float S = PH[1] + tau * PH[2] + R;
  const float nu = y - vpred;
  /* Gain re-anchor: a tracked G that moved more than gainReanchorRel from
   * its anchor re-inflates sigma_v before this frame's CUSUM, gate and
   * update, so this frame already corrects the speed with the new G. Only
   * in the take-off climb window, and only on a frame the gate would take
   * without it: a G moved by a dragging object or by a dead camera during
   * real vertical motion in the hover would otherwise let the bad speed in
   * and hide it from the CUSUM. Off on a clean take-off. */
  if (c->gainReanchorRel > 0.0f && o->gt.hasG) {
    if (!o->gAnchorSet) {
      o->gAnchor = G;
      o->gAnchorSet = true;
    } else if (fabsf(G - o->gAnchor) > c->gainReanchorRel * o->gAnchor && reanchorWindow(o) &&
               nu * nu <= c->gateK * c->gateK * S) {
      inflateSigmaV(o);
      o->gAnchor = G;
      o->cnt.reanchors++;
      for (int i = 0; i < 3; i++) {
        PH[i] = o->P[i][1] + tau * o->P[i][2];
      }
      S = PH[1] + tau * PH[2] + R;
    }
  }
  o->lastNu = nu;
  o->lastS = S;

  if (c->cusumK >= 0.0f) {
    const float zn = nu / sqrtf(S);
    /* Preserve the alarm's elapsed-time response at the lower frame rate. */
    if (c->cadenceAware) {
      const float cw = fminf(dt / CAM_CADENCE_REF_S, 4.5f);
      o->csPos = fmaxf(0.0f, o->csPos + (zn - c->cusumK) * cw);
      o->csNeg = fmaxf(0.0f, o->csNeg + (-zn - c->cusumK) * cw);
    } else {
      o->csPos = fmaxf(0.0f, o->csPos + zn - c->cusumK);
      o->csNeg = fmaxf(0.0f, o->csNeg - zn - c->cusumK);
    }
    if (!o->alarm && fmaxf(o->csPos, o->csNeg) > c->cusumH) {
      o->alarm = true;
      o->alarmT = o->tState;
      o->cnt.alarms++;
    }
    if (o->alarm) {
      if (o->tState - o->alarmT < usOf(c->cusumHoldS)) {
        o->cnt.rejected++;
        return CAM_VZ_F_CUSUM;
      }
      o->alarm = false;
      o->csPos = 0.0f;
      o->csNeg = 0.0f;
      inflateSigmaV(o);
    }
  }
  if (nu * nu > c->gateK * c->gateK * S) {
    o->cnt.rejected++;
    if (!o->rejecting) {
      o->rejecting = true;
      o->rejSince = o->tState;
    } else if (o->tState - o->rejSince > usOf(c->relockS)) {
      inflateSigmaV(o);
      o->cnt.relocks++;
      o->rejSince = o->tState;
    }
    return CAM_VZ_F_GATE;
  }
  o->rejecting = false;

  /* Kalman update, Joseph form */
  const float H[3] = {0.0f, 1.0f, tau};
  float K[3];
  for (int i = 0; i < 3; i++) {
    K[i] = PH[i] / S;
    o->x[i] += K[i] * nu;
  }
  float A[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      A[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
    }
  }
  float N[3][3];
  mat3Sandwich(A, o->P, N);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      o->P[i][j] = N[i][j] + R * K[i] * K[j];
    }
  }
  o->cnt.accepted++;
  o->hasAccept = true;
  o->lastAccept = o->tState;
  if (!stateFinite(o)) {
    o->fault = true;
  }
  return CAM_VZ_F_ACCEPTED;
}

/* ---- guard events (R1's late offset noise and T2's drift rate; header) ---- */

/* Notes a guard event now (tState). */
static void guardEvent(camVzObs_t *o)
{
  o->hasGuard = true;
  o->tGuard = o->tState;
}

/* Before a frame's update, with qLateMult > 1 or t2kR > 0: the clock of the
 * step (the lift-off of the first frame the filter takes) and the block-level
 * guard events (a commanded turn or motion, no tracked G, no accepted update
 * for CAM_VZ_QLATE_NOACC_US). Sets only the guard's bookkeeping. */
static void guardPre(camVzObs_t *o)
{
  const uint32_t now = o->tState;
  if (!o->hasFirstLift) {
    /* the clock of the step: the lift-off of the first frame the filter takes (the camera wakes >= 1.5 s after the
     * throttle-up), never restarted by a re-lift (tLift is); a bounce before that frame only delays the step */
    o->hasFirstLift = true;
    o->tFirstLift = o->tLift;
  }
  if (o->turn || o->moving || !o->gt.hasG || !o->hasAccept || now - o->lastAccept > CAM_VZ_QLATE_NOACC_US) {
    guardEvent(o);
  }
}

/* After a frame's update: its guard events (a frame refused for what it
 * shows: EXPANSION, LATERAL, NO_GAIN, the CUSUM hold, the innovation gate; a
 * CUSUM sum building up or its alarm; a re-lock, re-anchor or yaw restart,
 * n0 their counts before the frame; past R1's step or T2's start, the offset
 * moved more than CAM_VZ_QLATE_RATE_G since the history's newest entry at
 * least 1 s old) and the offset history (one entry per CAM_VZ_QLATE_HIST_US,
 * from the step / the start on). Not a QUALITY or NO_WINDOW refusal: such a
 * frame carries nothing into the filter, single ones are common in a good
 * hover (3 % of the 10-15 p1 hover's frames, every few seconds), and a run
 * of them is the no-update event after 0.5 s. DUPLICATE and SKIPPED frames
 * never reach the guard (camVzObsFrame returns before it). */
static void guardPost(camVzObs_t *o, camVzObsDecision_t d, uint32_t n0)
{
  const camVzObsConfig_t *c = &o->cfg;
  const uint32_t now = o->tState;
  const uint32_t n = o->cnt.yawResets + o->cnt.relocks + o->cnt.reanchors + o->cnt.alarms;
  const bool refused = d != CAM_VZ_F_ACCEPTED && d != CAM_VZ_F_QUALITY && d != CAM_VZ_F_NO_WINDOW;
  if (refused || n != n0 || o->alarm ||
      (c->cusumK >= 0.0f && fmaxf(o->csPos, o->csNeg) > CAM_VZ_QLATE_CS_REL * c->cusumH)) {
    guardEvent(o);
  }
  if (!o->late && !o->t2Init) {
    return;
  }
  /* the offset faster than the accelerometer drifts (<= 0.25 mg/s on the logged flights): the camera moved it */
  for (uint8_t k = 1; k <= o->bHistN; k++) {
    const uint8_t i = (uint8_t)((o->bHistHead + CAM_VZ_QLATE_HIST - k) % CAM_VZ_QLATE_HIST);
    if (now - o->bHistT[i] >= CAM_VZ_QLATE_RATE_US) {
      if (fabsf(o->x[2] - o->bHist[i]) > CAM_VZ_QLATE_RATE_G * CAM_VZ_G) {
        guardEvent(o);
      }
      break;
    }
  }
  const uint8_t newest = (uint8_t)((o->bHistHead + CAM_VZ_QLATE_HIST - 1) % CAM_VZ_QLATE_HIST);
  if (o->bHistN == 0 || now - o->bHistT[newest] >= CAM_VZ_QLATE_HIST_US) {
    o->bHist[o->bHistHead] = o->x[2];
    o->bHistT[o->bHistHead] = now;
    o->bHistHead = (uint8_t)((o->bHistHead + 1) % CAM_VZ_QLATE_HIST);
    if (o->bHistN < CAM_VZ_QLATE_HIST) {
      o->bHistN++;
    }
  }
}

/* ---- late offset noise (header "Late offset noise"; with qLateMult > 1) ---- */

/* Before a frame's update, after guardPre(): the step (p22Norm and bHeld from
 * here) and, while the guard holds, the offset's variance back to its size at
 * the step plus qBiasG's own growth since the previous frame, before this
 * frame can use it (the boosted walk is never let loose on a doubtful camera,
 * and a gap regrows it as today's filter would; the covariances scaled with
 * its sigma keep P positive semi-definite). Between frames the multiplier
 * grows P[2][2] (and through it P[1][1], P[0][0], left as they are): what the
 * next update may take is decided here, so the frame rate is fine enough. */
static void latePre(camVzObs_t *o)
{
  const camVzObsConfig_t *c = &o->cfg;
  const uint32_t now = o->tState;
  if (!o->late) {
    if (now - o->tFirstLift < usOf(c->qLateS)) {
      return;   /* before the step: the filter of before, bit for bit */
    }
    o->late = true;
    o->p22Norm = o->P[2][2];
    o->bHeldG = o->x[2] / CAM_VZ_G;
    o->tLatePrev = now;
  }
  /* the bound: the variance at the step plus what qBiasG alone adds since the previous frame (a gap or the ground
   * regrows it as today's filter would; only the multiplier's share is taken back) */
  const float qb = c->qBiasG * CAM_VZ_G;
  const float bound = o->p22Norm + qb * qb * (float)(now - o->tLatePrev) * 1e-6f;
  if (o->hasGuard && now - o->tGuard < CAM_VZ_QLATE_QUIET_US && o->P[2][2] > bound) {
    const float s = sqrtf(bound / o->P[2][2]);
    o->P[0][2] *= s;
    o->P[2][0] = o->P[0][2];
    o->P[1][2] *= s;
    o->P[2][1] = o->P[1][2];
    o->P[2][2] = bound;
  }
}

/* After a frame's update and guardPost(): the multiplier for the blocks up to
 * the next frame and, unless the guard holds (its run's first
 * CAM_VZ_QLATE_FREEZE_US), bHeldG = the mean of the history's entries
 * CAM_VZ_QLATE_HOLD_US .. + CAM_VZ_QLATE_AVG_US old. */
static void latePost(camVzObs_t *o)
{
  if (!o->late) {
    return;
  }
  const uint32_t now = o->tState;
  const bool guard = o->hasGuard && now - o->tGuard < CAM_VZ_QLATE_QUIET_US;
  if (o->lateOn) {
    const uint32_t dt = now - o->tLatePrev;
    o->lateUs += dt < CAM_VZ_QLATE_NOACC_US ? dt : CAM_VZ_QLATE_NOACC_US;
  }
  o->tLatePrev = now;
  if (o->lateOn && guard) {
    o->lateGuards++;
  }
  o->lateOn = !guard;
  if (!guard) {
    o->freezing = false;
  } else if (!o->freezing) {
    o->freezing = true;
    o->tFreeze = now;
  }
  if (guard && now - o->tFreeze < CAM_VZ_QLATE_FREEZE_US) {
    return;   /* frozen: the offset from before the event, for the guard run's first CAM_VZ_QLATE_FREEZE_US */
  }
  /* else (no guard, or a guard run longer than that: a long motion, a drift that keeps the rate guard tripping) the
   * delayed mean, of an offset that moved at qBiasG alone while the guard held (biasG itself there tested worse: a
   * glitch then a loss, docs/autonomy-a2b.md 25.9) */
  float sum = 0.0f;
  int m = 0;
  for (uint8_t k = 1; k <= o->bHistN; k++) {
    const uint8_t i = (uint8_t)((o->bHistHead + CAM_VZ_QLATE_HIST - k) % CAM_VZ_QLATE_HIST);
    const uint32_t age = now - o->bHistT[i];
    if (age >= CAM_VZ_QLATE_HOLD_US && age <= CAM_VZ_QLATE_HOLD_US + CAM_VZ_QLATE_AVG_US) {
      sum += o->bHist[i];
      m++;
    }
  }
  if (m > 0) {
    o->bHeldG = sum / (float)m / CAM_VZ_G;
  }
}

/* ---- offset drift rate (header "Offset drift rate"; with t2kR > 0) ---------- */

/* Before a frame's update, after guardPre(): the start, t2StartS after the
 * sortie's first lift-off (bRate = the prior from here). */
static void t2Pre(camVzObs_t *o)
{
  if (!o->t2Init && o->tState - o->tFirstLift >= usOf(o->cfg.t2StartS)) {
    o->t2Init = true;
    o->bRate = o->cfg.t2PriorMgS * 1e-3f * CAM_VZ_G;
    o->bRateMin = o->bRate;
    o->bRateMax = o->bRate;
  }
}

/* After an ACCEPTED frame and guardPost(), outside the guard hold: bRate takes
 * t2kR x the offset correction this frame made (x2Before: x[2] before its
 * update), clamped to +-t2BoundMgS. */
static void t2Post(camVzObs_t *o, camVzObsDecision_t d, float x2Before)
{
  if (!o->t2Init || d != CAM_VZ_F_ACCEPTED || (o->hasGuard && o->tState - o->tGuard < CAM_VZ_QLATE_QUIET_US)) {
    return;
  }
  const float bm = o->cfg.t2BoundMgS * 1e-3f * CAM_VZ_G;
  const float r = fminf(fmaxf(o->bRate + o->cfg.t2kR * (o->x[2] - x2Before), -bm), bm);
  o->bRate = r;
  if (r < o->bRateMin) {
    o->bRateMin = r;
  }
  if (r > o->bRateMax) {
    o->bRateMax = r;
  }
}

void camVzObsT2Get(const camVzObs_t *o, camVzObsT2_t *t)
{
  memset(t, 0, sizeof(*t));
  if (!o->t2Init) {
    return;
  }
  const float k = 1e3f / CAM_VZ_G;
  t->on = true;
  t->drifting = !o->onGround && o->hasAccept && o->tIn - o->lastAccept < CAM_VZ_T2_ACC_US;
  t->guard = o->hasGuard && o->tState - o->tGuard < CAM_VZ_QLATE_QUIET_US;
  t->t2RateMgS = o->bRate * k;
  t->minMgS = o->bRateMin * k;
  t->maxMgS = o->bRateMax * k;
  t->onUs = o->t2Us;
}

/* ---- healthy offset speed-up (header "Healthy offset speed-up"; with hbMult > 0) ---- */

/* After a frame the filter took (any decision): the flag the block path
 * reads. Everything in it changes at frames only (the tracker's G and the
 * CUSUM in frameUpdate, the first accepted update); the block adds the ground
 * and the accepted-update window, which run between frames. */
static void hbPost(camVzObs_t *o)
{
  o->hbOn = !o->fault && o->gt.hasG && !o->alarm && o->hasAccept;
}

uint8_t camVzObsLateState(const camVzObs_t *o)
{
  if (!o->late) {
    return CAM_VZ_LATE_OFF;
  }
  /* frames that stopped are a guard event at the next one (latePre): until then the multiplier only grows the
   * offset's variance, which that frame's latePre bounds before any update takes it */
  const bool noAccept = !o->hasAccept || o->tIn - o->lastAccept > CAM_VZ_QLATE_NOACC_US;
  return o->lateOn && !o->onGround && !noAccept ? CAM_VZ_LATE_ON : CAM_VZ_LATE_GUARD;
}

camVzObsDecision_t camVzObsFrame(camVzObs_t *o, const camVzObsFrame_t *f, uint64_t nowUs)
{
  if (o->hasSeq && f->seq == o->lastSeq) {
    return CAM_VZ_F_DUPLICATE;
  }
  const bool seqOk = o->hasSeq && f->seq == o->lastSeq + 1u;
  if (o->hasSeq && !seqOk) {
    o->cnt.seqGaps++;
  }
  o->hasSeq = true;
  o->lastSeq = f->seq;
  o->cnt.frames++;

  if (!o->started || o->fault || o->onGround || !o->lifted || !o->throttled) {
    o->cnt.skipped++;
    if (o->onGround && o->lateOn) {
      o->lateOn = false;   /* the late offset noise: the ground stops it until a frame after the re-lift (lateUs) */
      o->lateGuards++;
    }
    return CAM_VZ_F_SKIPPED;
  }
  /* motion window [t0, t1], us since the start */
  const int64_t t1 = (int64_t)f->frameTimeUs - (int64_t)o->tStartUs + (int64_t)o->cfg.gyroOffsetUs;
  const int64_t t0 = t1 - (int64_t)f->dtUs;
  int64_t camStart = (int64_t)o->tThrottle + (int64_t)o->cfg.wakeUs;
  if ((int64_t)o->tLift > camStart) {
    camStart = o->tLift;
  }
  if (t0 < camStart) {
    o->cnt.skipped++;
    return CAM_VZ_F_SKIPPED;
  }
  /* the guard path (R1 or T2; camVzObsConfigValid refuses both together): shared guard events, then each one's own */
  const bool r1 = o->cfg.qLateMult > 1.0f;
  camVzObsDecision_t d;
  if (r1 || o->cfg.t2kR > 0.0f) {
    const uint32_t n0 = o->cnt.yawResets + o->cnt.relocks + o->cnt.reanchors + o->cnt.alarms;
    guardPre(o);
    if (r1) {
      latePre(o);
    } else {
      t2Pre(o);
    }
    const float x2Before = o->x[2];
    d = frameUpdate(o, f, nowUs, t0, t1, seqOk);
    guardPost(o, d, n0);
    if (r1) {
      latePost(o);
    } else {
      t2Post(o, d, x2Before);
    }
  } else {
    d = frameUpdate(o, f, nowUs, t0, t1, seqOk);
  }
  /* the healthy offset speed-up (alone: refused with R1 or T2): the block path's flag */
  if (o->cfg.hbMult > 0.0f) {
    hbPost(o);
  }
  return d;
}

CAM_VZ_HOT void camVzObsGet(const camVzObs_t *o, camVzObsOut_t *out)
{
  memset(out, 0, sizeof(*out));
  const bool airborne = o->started && o->lifted && !o->onGround;
  if (airborne) {
    /* extrapolated over the open block */
    const float d = (float)(o->tIn - o->tState) * 1e-6f;
    out->h = o->x[0] + o->x[1] * d + o->bdZ - 0.5f * o->x[2] * d * d;
    out->v = o->x[1] + o->bdV - o->x[2] * d;
  }
  out->biasG = o->x[2] / CAM_VZ_G;
  out->sigmaH = sqrtf(fmaxf(o->P[0][0], 0.0f));
  out->sigmaV = sqrtf(fmaxf(o->P[1][1], 0.0f));
  out->sigmaBiasG = sqrtf(fmaxf(o->P[2][2], 0.0f)) / CAM_VZ_G;
  float G, sg;
  out->gain = selectGain(o, &G, &sg) ? G : 0.0f;
  out->gainTracked = o->gt.hasG;
  out->airborne = airborne;
  out->fault = o->fault;
  out->sinceAcceptUs = o->hasAccept ? o->tIn - o->lastAccept : UINT32_MAX;
  /* While a CUSUM alarm holds, the speed the frames before the alarm pulled
   * in keeps integrating into h with no camera correction: not healthy. */
  out->healthy = !o->fault && o->cfg.useCamera && o->hasAccept && out->sinceAcceptUs < o->cfg.healthyWindowUs &&
                 o->gt.hasG && !o->alarm;
  out->healthyGuess = !o->fault && o->cfg.useCamera && o->hasAccept && out->sinceAcceptUs < o->cfg.healthyWindowUs &&
                      (o->gt.hasG || o->hasGuess) && !o->alarm;
  out->lastInnov = o->lastNu;
  out->lastInnovSigma = sqrtf(fmaxf(o->lastS, 0.0f));
  out->biasHeldG = o->late ? o->bHeldG : out->biasG;
  /* the healthy offset speed-up on the current block: the frame's flag, airborne and healthy (closeBlock's test) */
  out->hbOn = o->hbOn && airborne && out->healthy;
}

bool camVzObsFirstGainTime(const camVzObs_t *o, float *tS)
{
  if (!o->gt.hasFirst) {
    return false;
  }
  *tS = (float)o->gt.tSumFirst * 0.5e-6f;
  return true;
}

const char *camVzObsDecisionName(camVzObsDecision_t d)
{
  switch (d) {
    case CAM_VZ_F_DUPLICATE: return "DUPLICATE";
    case CAM_VZ_F_SKIPPED: return "SKIPPED";
    case CAM_VZ_F_QUALITY: return "QUALITY";
    case CAM_VZ_F_EXPANSION: return "EXPANSION";
    case CAM_VZ_F_LATERAL: return "LATERAL";
    case CAM_VZ_F_NO_WINDOW: return "NO_WINDOW";
    case CAM_VZ_F_NO_GAIN: return "NO_GAIN";
    case CAM_VZ_F_CUSUM: return "CUSUM";
    case CAM_VZ_F_GATE: return "GATE";
    case CAM_VZ_F_ACCEPTED: return "ACCEPTED";
  }
  return "?";
}
