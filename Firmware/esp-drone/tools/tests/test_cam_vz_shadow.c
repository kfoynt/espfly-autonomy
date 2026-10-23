/*
 * Host tests for components/core/crazyflie/utils/src/cam_vz_shadow.c (the
 * camera height observer in shadow mode) and an optional replay of the logged
 * flights through it.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      -Icomponents/core/crazyflie/modules/interface \
 *      tools/tests/test_cam_vz_shadow.c components/core/crazyflie/utils/src/cam_vz_shadow.c \
 *      components/core/crazyflie/utils/src/cam_vz_obs.c -lm -o /tmp/test_cam_vz_shadow && \
 *   /tmp/test_cam_vz_shadow [CSV_DIR [BIAS_TRUTH_CSV]] [-v] [-g[=P_US]]
 *
 * Synthetic flights (1 kHz steps, a point mass with a known trajectory, an
 * accelerometer with a ground baseline g0 and an in-flight offset, and a
 * forward camera whose vertical image motion is -G times the height change
 * over each frame's motion window, delivered 150 ms after capture) check the
 * ground / lift-off / touchdown latch, the g0 prior, the per-flight
 * statistics, the recorder encoding and vert_ctrl's external-estimate fields.
 *
 * Replay (CSV_DIR: the 100 Hz recorder files flight*.csv, e.g.
 * ../../firmware-backups/cal; skipped without it): every take-off of the 13
 * camera files goes through the module as the drone would run it (each row
 * held for ten 1 ms steps, the world-up specific force from the logged
 * attitude, a ground start at the first row of a take-off, frames on a new
 * flowSeq) and prints the flight-summary fields. With BIAS_TRUTH_CSV (the
 * camera height study's bias_truth.csv) the logged offset at wake + 3 s and
 * + 5 s is compared with the study's post-hoc truth and the on-board lift-off
 * time with the study's offline one. Every take-off is replayed twice: without
 * the attitude estimator's rate correction (rateCorr 0: no gyro-bias
 * correction) and with it, reconstructed from the log as the drone's
 * sensfusion6 would have given it (the gyro angle the estimator did not
 * follow: inside each frame's motion window from the firmware's own gyro
 * integral flowG* and the logged attitude, elsewhere from 10-row chunks of the
 * logged gyro). Files 6-15 (no ground re-zero yet: an in-flight pitch-gyro
 * bias of +0.2..+0.8 deg/s) must drift less, files 16-21 (re-zeroed, ~0) the same.
 *
 * Every take-off is also replayed with the rate correction of the attitude
 * filter itself (modules/src/sensfusion6.c, included) run over the logged
 * 100 Hz gyro and accelerometer, with the in-flight bias learning off
 * (twoKiAir 0, the filter of 14bcfc6) and on (0.040, docs/autonomy-a2b.md
 * 19.2): the learning must not make the drift, the offsets or the counters
 * worse.
 *
 * Replay with -g[=P_US] feeds each take-off as the drone does (cam_vz_glue.c):
 * one camVzShadowStep() per 10-sample block, the IMU period P_US (default
 * 999.9 us, an MPU clock a hair fast: 9999 us blocks), and checks that the
 * filter steps on every block (~100 Hz, the rate it was tuned at).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cam_vz_shadow.h"

/* The attitude filter, for the replays' rate correction (as test_sensfusion6_ratecorr.c includes it). */
#define __LOG_H__
#define __PARAM_H__
#define LOG_GROUP_START(x)
#define LOG_GROUP_STOP(x)
#define LOG_ADD(t, n, p)
#define PARAM_GROUP_START(x)
#define PARAM_GROUP_STOP(x)
#define PARAM_ADD(t, n, p)
#define long int32_t
#include "../../components/core/crazyflie/modules/src/sensfusion6.c"
#undef long

static int failures = 0;
static int checks = 0;
static int verbose = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define G 9.80665

static camVzShadow_t sh;

/* ---- deterministic noise ------------------------------------------------------ */

static uint32_t rngState = 1;

static double urand(void)
{
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (rngState + 0.5) / 4294967296.0;
}

static double grand(void)
{
  const double u1 = urand(), u2 = urand();
  return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* ---- synthetic flight -------------------------------------------------------- */

#define SIM_MAX_MS 30000

typedef struct {
  double g0Off;          /* g: ground baseline - 1 */
  double airOff;         /* g: extra in-flight offset (not seen by g0) */
  double accNoise;       /* g rms per 1 kHz sample */
  double camC;           /* m/s: camera speed offset (positive: reads a climb that is not there) */
  double camNoisePx;     /* px rms per frame */
  double camG;           /* px/m */
  bool camera;
  float thrustMax;       /* 45000: lift-off by the thrust level; 28000: by the accelerometer */
  double hoverS;         /* hover with a +-3 cm oscillation (gain excitation) */
  bool implicitStart;    /* no ground-start flag (soft-start disarmed) */
  bool hop;              /* thrust back 0.3 s after the stop, a short second flight */
  int nanAtMs;           /* > 0: the accelerometer reads NaN at this step */
  double vzG0;           /* vert_ctrl's g0 for the camBias mapping check */
  double gyroBiasDps;    /* the pitch gyro reads this rate the drone does not have (frames' gyroAngle[1]) */
  bool feedRateCorr;     /* give the shadow the settled estimator correction (-bias) */
} simCfg_t;

typedef struct {
  /* truth */
  double tLift, tTouch, tStop;   /* s */
  double hTrueMax;
  /* observed */
  double tPhase[4];              /* first time each phase was entered (s, -1: never) */
  int liftBy;
  fsumCamVz_t stats;             /* at the end */
  fsumCamVz_t statsAtStop;       /* right after the motors stopped (what the RTC log copies) */
  bool healthySeen, camHealthyInput, camHealthyOnGround;
  double camBiasErrMax;          /* |camBias - (biasG + 1 - vzG0)| */
  bool vzInputFinite;
  double hErrMaxHealthy;         /* |h - h_true| while healthy (camC = 0) */
  int decisions[10];
  camVzShadowRec_t recAir;       /* a record in the hover */
  float biasAtRec;
  float g0;
} simRes_t;

static void defaultSim(simCfg_t *c)
{
  memset(c, 0, sizeof(*c));
  c->g0Off = 0.012;
  c->airOff = 0.0014;
  c->accNoise = 0.02;
  c->camC = 0.0;
  c->camNoisePx = 0.25;
  c->camG = 100.0;
  c->camera = true;
  c->thrustMax = 45000.0f;
  c->hoverS = 8.0;
  c->vzG0 = 1.003;
}

static double hTrue[SIM_MAX_MS + 1];

static double hAt(double t)
{
  double ms = t * 1000.0;
  if (ms < 0.0) {
    return 0.0;
  }
  int i = (int)ms;
  if (i >= SIM_MAX_MS) {
    return hTrue[SIM_MAX_MS];
  }
  const double f = ms - i;
  return hTrue[i] * (1.0 - f) + hTrue[i + 1] * f;
}

typedef struct {
  camVzObsFrame_t f;
  double w0;             /* motion window start, s */
  double deliverT;
} pending_t;

#define QLEN 16

static void runSim(const simCfg_t *c, simRes_t *r, uint32_t seed)
{
  memset(r, 0, sizeof(*r));
  for (int k = 0; k < 4; k++) {
    r->tPhase[k] = -1.0;
  }
  r->tTouch = r->tStop = -1.0;
  r->vzInputFinite = true;
  rngState = seed ? seed : 1;
  camVzShadowInit(&sh, NULL);

  const double tRest = 1.0;              /* resting before the ground start */
  const double t0 = tRest;               /* ground start */
  const double tl = t0 + 0.6;            /* true lift-off */
  const double climbT = 1.5, climbH = 0.35;
  const double climbA = 2.0 * M_PI * climbH / (climbT * climbT);
  const double oscA = 0.03, oscW = 2.0 * M_PI * 0.5;
  const double tDesc = tl + climbT + c->hoverS;
  const double camWake = t0 + 1.5;       /* the camera's take-off standby */
  const double frameDt = 0.059, latency = 0.150, gyroOff = 0.015;

  double h = 0.0, v = 0.0;
  bool air = false, landed = false, stopped = false;
  double tStop = -1.0, impactUntil = -1.0;
  bool hopDone = false, hopOn = false;
  double nextFrame = camWake;
  uint32_t seq = 0;
  pending_t q[QLEN];
  int qHead = 0, qTail = 0;   /* FIFO: frames arrive in capture order */
  bool recTaken = false;

  for (int ms = 0; ms <= SIM_MAX_MS; ms++) {
    const double t = ms * 1e-3;
    /* true acceleration */
    double a = 0.0;
    if (air) {
      const double tau = t - tl;
      if (tau < climbT) {
        a = climbA * sin(2.0 * M_PI * tau / climbT);
      } else if (t < tDesc) {
        a = -oscA * oscW * oscW * cos(oscW * (tau - climbT));
      } else if (t < tDesc + 0.5) {
        a = -0.5;
      } else {
        a = 0.0;
      }
      v += a * 1e-3;
      h += v * 1e-3;
      if (h <= 0.0 && t > tl + 0.5) {
        /* touchdown: a 10 ms stop */
        air = false;
        landed = true;
        r->tTouch = t;
        impactUntil = t + 0.010;
        a = -v / 0.010;
        h = 0.0;
        v = 0.0;
      }
    } else if (!landed && t >= tl) {
      air = true;
      r->tLift = t;
    } else if (landed && t < impactUntil) {
      a = 25.0;   /* the stop's deceleration, felt as specific force */
    }
    if (h > r->hTrueMax) {
      r->hTrueMax = h;
    }
    hTrue[ms] = h;

    /* thrust */
    float thrust = 0.0f;
    if (t >= t0 && !stopped) {
      thrust = (float)fmin(100.0 + 60000.0 * (t - t0), c->thrustMax);
    }
    if (landed && !stopped && t >= r->tTouch + 0.3) {
      stopped = true;
      tStop = t;
      r->tStop = t;
      thrust = 0.0f;
    }
    if (c->hop && stopped && !hopDone && t >= tStop + 0.3) {
      hopOn = true;
    }
    if (hopOn) {
      thrust = 45000.0f;
      if (t >= tStop + 0.8) {
        hopOn = false;
        hopDone = true;
      }
    }
    if (stopped && !hopOn) {
      thrust = 0.0f;
    }

    /* accelerometer: baseline on the ground, plus the in-flight offset in the air */
    double acc = 1.0 + c->g0Off + a / G + c->accNoise * grand();
    if (air) {
      acc += c->airOff;
    }
    if (c->nanAtMs > 0 && ms == c->nanAtMs) {
      acc = NAN;
    }
    const bool restingNow = thrust == 0.0f && (t < t0 || (stopped && t >= tStop + 0.5));
    camVzShadowIn_t in = {
      .timeUs = 1000000ull + (uint64_t)ms * 1000ull,
      .accUpG = (float)acc,
      .thrust = thrust,
      .gyroMaxDps = 2.0f,
      .groundStart = !c->implicitStart && ms == (int)lround(t0 * 1000.0),
      .restingOnGround = restingNow,
      .still = thrust == 0.0f && !(landed && t < impactUntil + 0.05),
      .liftoffHint = false,
      .rateCorr = { 0.0f, c->feedRateCorr ? (float)(-c->gyroBiasDps * M_PI / 180.0) : 0.0f, 0.0f },
    };
    const camVzShadowPhase_t before = sh.phase;
    camVzShadowStep(&sh, &in);
    if (sh.phase != before && r->tPhase[sh.phase] < 0.0) {
      r->tPhase[sh.phase] = t;
    }
    if (stopped && ms == (int)lround(tStop * 1000.0)) {
      camVzShadowFlightStats(&sh, &r->statsAtStop);
    }

    /* camera: capture, then delivery after the latency */
    if (c->camera && t >= nextFrame && (tStop < 0.0 || t < tStop + 0.5) && qTail - qHead < QLEN) {
      pending_t *p = &q[qTail++ % QLEN];
      memset(p, 0, sizeof(*p));
      p->f.seq = ++seq;
      p->f.frameTimeUs = in.timeUs;
      p->f.dtUs = (uint32_t)lround(frameDt * 1e6);
      p->f.ok = true;
      p->f.texture = 20.0f;
      p->f.residual = 3.0f;
      p->f.gyroAngle[1] = (float)(c->gyroBiasDps * M_PI / 180.0 * frameDt);
      p->w0 = t + gyroOff - frameDt;   /* the window ends gyroOff after the capture: dy is made at delivery */
      p->deliverT = t + latency;
      nextFrame += frameDt;
    }
    while (qHead < qTail && t >= q[qHead % QLEN].deliverT) {
      pending_t *p = &q[qHead++ % QLEN];
      p->f.dy = (float)(-c->camG * (hAt(p->w0 + frameDt) - hAt(p->w0) + c->camC * frameDt) +
                        c->camNoisePx * grand());
      const camVzObsDecision_t d = camVzShadowFrame(&sh, &p->f, in.timeUs);
      r->decisions[d]++;
    }

    /* outputs */
    const camVzObsOut_t *o = &sh.out;
    vzInput_t vin;
    memset(&vin, 0, sizeof(vin));
    camVzShadowVzInput(&sh, (float)c->vzG0, &vin);
    if (!isfinite(vin.camVz) || !isfinite(vin.camBias)) {
      r->vzInputFinite = false;
    }
    if (vin.camHealthy) {
      r->camHealthyInput = true;
      if (sh.phase != CVS_AIR) {
        r->camHealthyOnGround = true;
      }
    }
    if (vin.camVzValid) {
      const double e = fabs(vin.camBias - (o->biasG + 1.0 - c->vzG0));
      if (e > r->camBiasErrMax) {
        r->camBiasErrMax = e;
      }
    }
    if (o->healthy && sh.phase == CVS_AIR) {
      r->healthySeen = true;
      const double e = fabs(o->h - hAt(t));
      if (e > r->hErrMaxHealthy) {
        r->hErrMaxHealthy = e;
      }
    }
    if (!recTaken && air && t > tl + climbT + 4.0) {
      camVzShadowRecord(&sh, &r->recAir);
      r->biasAtRec = o->biasG;
      recTaken = true;
    }
    if (stopped && t > tStop + 1.5 && !c->hop) {
      break;
    }
    if (c->hop && hopDone && t > tStop + 2.5) {
      break;
    }
  }
  camVzShadowFlightStats(&sh, &r->stats);
  r->liftBy = sh.liftBy;
  r->g0 = sh.g0;
}

static double cmg(int16_t v)
{
  return v / 100.0;
}

static void testNominal(void)
{
  simCfg_t c;
  defaultSim(&c);
  simRes_t r;
  runSim(&c, &r, 7);
  const fsumCamVz_t *s = &r.stats;
  if (verbose) {
    printf("nominal: lift %.3f touch %.3f stop %.3f | phases ground %.3f air %.3f landed %.3f idle %.3f\n",
           r.tLift, r.tTouch, r.tStop, r.tPhase[CVS_GROUND], r.tPhase[CVS_AIR], r.tPhase[CVS_LANDED],
           r.tPhase[CVS_IDLE]);
    printf("  flags 0x%02x g0 %.1f mg b3 %.2f b5 %.2f btd %.2f sig5 %.2f mg G %.1f hTd %d mm vTd %d hMax %d air %u "
           "camAir %u ok %u gap %u acc %u rej %u firstOk %u hErr %.3f\n",
           s->flags, s->g0Dmg / 10.0, cmg(s->bias3Cmg), cmg(s->bias5Cmg), cmg(s->biasTdCmg), s->sigB5Cmg / 100.0,
           s->gainDpm / 10.0, s->hTdMm, s->vTdMms, s->hMaxMm, s->airCs, s->camAirCs, s->okCs, s->gapMaxCs,
           s->accepted, s->rejected, s->firstOkDs, r.hErrMaxHealthy);
  }
  /* latch */
  CHECK(fabs(r.tPhase[CVS_GROUND] - 1.0) < 0.0015, "ground start at 1.000 s, got %.4f", r.tPhase[CVS_GROUND]);
  CHECK(fabs(r.tPhase[CVS_AIR] - 1.499) < 0.0015, "lift-off by thrust 30000 at 1.499 s, got %.4f", r.tPhase[CVS_AIR]);
  CHECK(r.liftBy == CVS_LIFT_BY_THRUST, "lift-off by the thrust level, got %d", r.liftBy);
  CHECK(fabs(r.tPhase[CVS_LANDED] - r.tStop) < 0.0015, "LANDED at the motor stop %.3f, got %.4f", r.tStop,
        r.tPhase[CVS_LANDED]);
  CHECK(r.tPhase[CVS_IDLE] > r.tStop + 0.49 && r.tPhase[CVS_IDLE] < r.tStop + 0.51, "IDLE after the rest, got %.4f",
        r.tPhase[CVS_IDLE]);
  /* prior and offset */
  /* 128 means of 10 white 20 mg samples: sigma of the median ~0.7 mg */
  CHECK(fabs(r.g0 - (1.0 + c.g0Off)) < 0.0015, "g0 %.5f vs %.5f", r.g0, 1.0 + c.g0Off);
  CHECK(s->flags & FSUM_CVZ_PRIOR, "prior from g0");
  CHECK(abs(s->g0Dmg - (int)lround((r.g0 - 1.0) * 1e4)) <= 1, "g0Dmg %d vs g0 %.5f", s->g0Dmg, r.g0);
  const double bTrue = (c.g0Off + c.airOff) * 1e3;
  CHECK(s->bias3Cmg != FSUM_CVZ_NONE && fabs(cmg(s->bias3Cmg) - bTrue) < 1.0, "bias +3 s %.2f vs %.2f mg",
        cmg(s->bias3Cmg), bTrue);
  CHECK(s->bias5Cmg != FSUM_CVZ_NONE && fabs(cmg(s->bias5Cmg) - bTrue) < 1.0, "bias +5 s %.2f vs %.2f mg",
        cmg(s->bias5Cmg), bTrue);
  CHECK(fabs(cmg(s->biasTdCmg) - bTrue) < 1.0, "bias at touchdown %.2f vs %.2f mg", cmg(s->biasTdCmg), bTrue);
  CHECK(s->sigB5Cmg > 0 && s->sigB5Cmg < 300, "sigma at +5 s %u (0.01 mg)", s->sigB5Cmg);
  /* camera */
  CHECK(r.healthySeen, "healthy in the air");
  CHECK(s->flags & FSUM_CVZ_GAIN, "gain tracked");
  CHECK(fabs(s->gainDpm / 10.0 - c.camG) < 0.25 * c.camG, "G %.1f vs %.1f px/m", s->gainDpm / 10.0, c.camG);
  CHECK(s->okCs > s->camAirCs / 2, "healthy %u of %u cs", s->okCs, s->camAirCs);
  CHECK(s->firstOkDs < 40, "first healthy %u ds after the wake", s->firstOkDs);
  CHECK(s->gapMaxCs < 50, "longest gap %u cs", s->gapMaxCs);
  CHECK(s->accepted > 100, "accepted %u", s->accepted);
  CHECK(r.hErrMaxHealthy < 0.06, "|h - h_true| while healthy %.3f m", r.hErrMaxHealthy);
  /* touchdown */
  CHECK((s->flags & (FSUM_CVZ_RAN | FSUM_CVZ_LIFT | FSUM_CVZ_TD | FSUM_CVZ_IMPACT)) ==
        (FSUM_CVZ_RAN | FSUM_CVZ_LIFT | FSUM_CVZ_TD | FSUM_CVZ_IMPACT), "flags 0x%02x", s->flags);
  CHECK(!(s->flags & (FSUM_CVZ_LIFT_ACC | FSUM_CVZ_FAULT)), "flags 0x%02x", s->flags);
  CHECK(abs(s->hTdMm) < 50, "h at touchdown %d mm", s->hTdMm);
  CHECK(abs(s->vTdMms + 250) < 80, "v at touchdown %d mm/s (-250)", s->vTdMms);
  CHECK(abs(s->hMaxMm - (int)lround(r.hTrueMax * 1000.0)) < 60, "hMax %d mm vs %.0f", s->hMaxMm, r.hTrueMax * 1e3);
  const double airTrue = r.tStop - r.tPhase[CVS_AIR];
  CHECK(fabs(s->airCs / 100.0 - airTrue) < 0.02, "air %u cs vs %.2f s", s->airCs, airTrue);
  CHECK(memcmp(&r.statsAtStop, &r.stats, sizeof(fsumCamVz_t)) == 0, "stats frozen after the stop");
  /* vert_ctrl's fields */
  CHECK(r.camHealthyInput, "camHealthy given to vert_ctrl in the air");
  CHECK(!r.camHealthyOnGround, "camHealthy never on the ground");
  CHECK(r.camBiasErrMax < 1e-6, "camBias = biasG + 1 - g0 (err %.2e)", r.camBiasErrMax);
  CHECK(r.vzInputFinite, "vzInput fields finite");
  /* recorder */
  CHECK((r.recAir.flags & 0x0F) == (CVS_REC_AIRBORNE | CVS_REC_HEALTHY | CVS_REC_GAIN), "record flags 0x%02x",
        r.recAir.flags);
  CHECK((r.recAir.flags >> 4) <= CAM_VZ_F_ACCEPTED, "record decision %u", r.recAir.flags >> 4);
  CHECK(abs(r.recAir.biasCmg - (int)lrintf(r.biasAtRec * 1e5f)) <= 1, "record bias %d", r.recAir.biasCmg);
  CHECK(r.recAir.gainDpm > 500 && r.recAir.gainDpm < 1500, "record gain %u", r.recAir.gainDpm);
  CHECK(r.recAir.sigVMms > 0 && r.recAir.sigVMms < 100, "record sigma v %u mm/s", r.recAir.sigVMms);
  CHECK(r.decisions[CAM_VZ_F_ACCEPTED] > 100 && r.decisions[CAM_VZ_F_SKIPPED] > 0, "decisions A %d skipped %d",
        r.decisions[CAM_VZ_F_ACCEPTED], r.decisions[CAM_VZ_F_SKIPPED]);
}

/* Touchdown values with several impacts (2026-10-13 flight 1: five in-air
 * contacts over 0.83 s, motors off 0.86 s after the first). A scripted flight
 * without the camera: a ground start, lift-off by the thrust level, then a
 * steady 0.3 m/s^2 climb so h and v differ at every impact; impacts are
 * gyro spikes (300 dps for 3 ms), which leave the filter's input alone.
 * Offsets are seconds from the first impact, 2.0 s after the lift-off. The
 * touchdown values must be those just before the first impact of the cluster
 * that reaches into the last 0.5 s before the stop. */
static void impactRun(const double *impacts, int n, double stopAfter, fsumCamVz_t *st, float *expH, float *expV,
                      int expIdx)
{
  camVzShadowInit(&sh, NULL);
  const double t0 = 1.0, tFirst = 3.5;   /* ground start; first impact (lift-off at ~1.5 s) */
  float hBefore[8] = { 0 }, vBefore[8] = { 0 };
  int next = 0;
  double impactUntil = -1.0, tl = -1.0;
  for (int ms = 0; ms <= 6000; ms++) {
    const double t = ms * 1e-3;
    const float thrust = t < t0 || t >= tFirst + stopAfter ? 0.0f : (float)fmin(100.0 + 60000.0 * (t - t0), 45000.0);
    if (tl < 0.0 && sh.phase == CVS_AIR) {
      tl = t;
    }
    const double a = tl >= 0.0 && thrust > 0.0f ? 0.3 : 0.0;
    float gyro = 2.0f;
    if (next < n && t >= tFirst + impacts[next] - 1e-9) {
      hBefore[next] = sh.prevH;   /* what the step stores as the impact's values */
      vBefore[next] = sh.prevV;
      impactUntil = t + 0.003;
      next++;
    }
    if (t < impactUntil) {
      gyro = 300.0f;
    }
    camVzShadowIn_t in = {
      .timeUs = 1000000ull + (uint64_t)ms * 1000ull, .accUpG = (float)(1.004 + a / G), .thrust = thrust,
      .gyroMaxDps = gyro, .groundStart = ms == 1000, .restingOnGround = thrust == 0.0f && t < t0,
      .still = thrust == 0.0f,
    };
    camVzShadowStep(&sh, &in);
    if (sh.phase == CVS_LANDED) {
      break;
    }
  }
  camVzShadowFlightStats(&sh, st);
  *expH = expIdx >= 0 ? hBefore[expIdx] : sh.prevH;
  *expV = expIdx >= 0 ? vBefore[expIdx] : sh.prevV;
}

static void testImpactCluster(void)
{
  fsumCamVz_t st;
  float eh, ev;
  /* five contacts over 0.83 s, motors off at 0.86 s: the values before the first */
  const double five[] = { 0.0, 0.25, 0.37, 0.66, 0.83 };
  impactRun(five, 5, 0.86, &st, &eh, &ev, 0);
  float h4, v4;
  fsumCamVz_t st4;
  impactRun(five, 5, 0.86, &st4, &h4, &v4, 3);   /* the 0.66 s impact: what the first-impact-only rule gave */
  printf("impact cluster 0/0.25/0.37/0.66/0.83 s, stop 0.86 s: touchdown h %d mm v %d mm/s (before the first impact "
         "%.0f / %.0f; before the 0.66 s one, the old rule's, %.0f / %.0f)\n", st.hTdMm, st.vTdMms, eh * 1e3, ev * 1e3,
         h4 * 1e3, v4 * 1e3);
  CHECK((st.flags & FSUM_CVZ_TD) && (st.flags & FSUM_CVZ_IMPACT) && abs(st.hTdMm - (int)lrintf(eh * 1000.0f)) <= 1 &&
        abs(st.vTdMms - (int)lrintf(ev * 1000.0f)) <= 1, "five contacts: touchdown values from before the first "
        "(h %d vs %.0f mm, v %d vs %.0f mm/s)", st.hTdMm, eh * 1e3, st.vTdMms, ev * 1e3);
  CHECK(fabsf(ev - v4) > 0.15f, "the scenario separates the impacts (v %.3f vs %.3f m/s)", ev, v4);
  /* a single impact 0.3 s before the stop: its values */
  const double one[] = { 0.0 };
  impactRun(one, 1, 0.3, &st, &eh, &ev, 0);
  CHECK((st.flags & FSUM_CVZ_IMPACT) && abs(st.hTdMm - (int)lrintf(eh * 1000.0f)) <= 1 &&
        abs(st.vTdMms - (int)lrintf(ev * 1000.0f)) <= 1, "single impact 0.3 s before the stop: its values");
  /* a bounce, then the landing 1.2 s later: the later cluster */
  const double two[] = { 0.0, 1.2 };
  impactRun(two, 2, 1.3, &st, &eh, &ev, 1);
  CHECK((st.flags & FSUM_CVZ_IMPACT) && abs(st.hTdMm - (int)lrintf(eh * 1000.0f)) <= 1 &&
        abs(st.vTdMms - (int)lrintf(ev * 1000.0f)) <= 1, "impacts 1.2 s apart: the later one's values");
  /* the last impact more than 0.5 s before the stop: the last airborne values, no impact flag */
  impactRun(two, 2, 1.8, &st, &eh, &ev, -1);
  CHECK(!(st.flags & FSUM_CVZ_IMPACT) && abs(st.hTdMm - (int)lrintf(eh * 1000.0f)) <= 1,
        "the last impact 0.6 s before the stop: the values at the stop, no impact flag");
}

/* The camera's slow speed offset c shows up as the height at touchdown: the
 * measurement the shadow mode is for. */
static void testCameraOffset(void)
{
  double hTd[2];
  double air = 0.0;
  for (int k = 0; k < 2; k++) {
    simCfg_t c;
    defaultSim(&c);
    c.camC = k == 0 ? 0.03 : -0.03;
    simRes_t r;
    runSim(&c, &r, 11);
    hTd[k] = r.stats.hTdMm / 1000.0;
    air = r.stats.camAirCs / 100.0;
    if (verbose) {
      printf("camera offset %+.2f m/s: hTd %+.3f m over %.1f s with the camera (%.3f m/s), b5 %.2f mg\n", c.camC, hTd[k],
             air, hTd[k] / air, cmg(r.stats.bias5Cmg));
    }
  }
  CHECK(hTd[0] > 0.08 && hTd[1] < -0.08, "hTd with c = +-0.03 m/s: %+.3f / %+.3f m", hTd[0], hTd[1]);
  CHECK(fabs(hTd[0] / air - 0.03) < 0.015 && fabs(hTd[1] / air + 0.03) < 0.015,
        "hTd / camera time follows c: %+.4f / %+.4f m/s", hTd[0] / air, hTd[1] / air);
}

/* An in-flight pitch-gyro bias b leaks through the rotation removal as a
 * speed offset kyGy * b / G; the attitude estimator's correction (-b once
 * settled) removes it. */
static void testGyroBias(void)
{
  const double bDps = 0.8;
  const double leak = 99.790 * bDps * M_PI / 180.0 / 100.0;   /* kyGy b / G, m/s */
  double hTd[3], air = 0.0;
  for (int k = 0; k < 3; k++) {   /* no bias (the seed's own drift), bias uncorrected, bias corrected */
    simCfg_t c;
    defaultSim(&c);
    c.gyroBiasDps = k == 0 ? 0.0 : bDps;
    c.feedRateCorr = k == 2;
    simRes_t r;
    runSim(&c, &r, 11);
    hTd[k] = r.stats.hTdMm / 1000.0;
    air = r.stats.camAirCs / 100.0;
    if (verbose) {
      printf("gyro bias %.1f deg/s, rateCorr %s: hTd %+.3f m over %.1f s with the camera, estimate %.3f deg/s, "
             "acc %u rej %u\n", c.gyroBiasDps, c.feedRateCorr ? "fed" : "off", hTd[k], air,
             sh.gyroBias[1] * 180.0 / M_PI, r.stats.accepted, r.stats.rejected);
    }
    CHECK(r.stats.accepted > 100 && (r.stats.flags & FSUM_CVZ_TD), "gyro bias %d: accepted %u flags 0x%02x", k,
          r.stats.accepted, r.stats.flags);
    if (k == 2) {
      CHECK(fabs(sh.gyroBias[1] * 180.0 / M_PI - bDps) < 0.02 * bDps, "bias estimate %.3f vs %.3f deg/s",
            sh.gyroBias[1] * 180.0 / M_PI, bDps);
      CHECK(sh.gyroBias[0] == 0.0f && sh.gyroBias[2] == 0.0f, "no roll/yaw bias from a zero correction");
    } else {
      CHECK(sh.gyroBias[1] == 0.0f, "rateCorr 0: no estimate");
    }
  }
  const double dOff = (hTd[1] - hTd[0]) / air, dFed = (hTd[2] - hTd[0]) / air;
  if (verbose) {
    printf("  extra drift from the bias: %+.4f m/s uncorrected, %+.4f corrected (leak kyGy b / G %+.4f)\n", dOff, dFed,
           leak);
  }
  CHECK(fabs(dOff - leak) < 0.25 * leak, "uncorrected: extra drift %+.4f m/s vs the leak %+.4f", dOff, leak);
  CHECK(fabs(dFed) < 0.15 * leak, "corrected: extra drift %+.4f m/s (leak %+.4f)", dFed, leak);

  /* the estimate: fed only with the motors on, clamped, deaf to non-finite input, reset at each ground start */
  camVzShadowInit(&sh, NULL);
  uint64_t us = 1000000;
  for (int ms = 0; ms < 3000; ms++, us += 1000) {
    camVzShadowIn_t in = { .timeUs = us, .accUpG = 1.0f, .thrust = ms >= 1000 ? 20000.0f : 0.0f,
                           .groundStart = ms == 1000, .restingOnGround = ms < 1000, .still = ms < 1000,
                           .rateCorr = { ms == 2000 ? NAN : 0.5f, -1.0f, 0.0f } };
    camVzShadowStep(&sh, &in);
    if (ms == 999) {
      CHECK(sh.gyroBias[0] == 0.0f && sh.gyroBias[1] == 0.0f, "not fed before the ground start (motors off)");
    }
  }
  CHECK(fabsf(sh.gyroBias[0] + CVS_GYRO_BIAS_MAX_RADS) < 1e-6f && fabsf(sh.gyroBias[1] - CVS_GYRO_BIAS_MAX_RADS) < 1e-6f,
        "clamped to +-%.3f rad/s: %.4f %.4f", CVS_GYRO_BIAS_MAX_RADS, sh.gyroBias[0], sh.gyroBias[1]);
  camVzShadowIn_t in = { .timeUs = us, .accUpG = 1.0f, .thrust = 0.0f, .rateCorr = { 1.0f, 1.0f, 1.0f } };
  for (int k = 0; k < 100; k++, us += 1000) {
    in.timeUs = us;
    camVzShadowStep(&sh, &in);
  }
  CHECK(fabsf(sh.gyroBias[1] - CVS_GYRO_BIAS_MAX_RADS) < 1e-6f, "held with the motors off: %.4f", sh.gyroBias[1]);
  in.timeUs = us + 1000000;
  in.groundStart = true;
  in.thrust = 10000.0f;
  in.rateCorr[0] = in.rateCorr[1] = in.rateCorr[2] = 0.0f;
  camVzShadowStep(&sh, &in);
  CHECK(sh.gyroBias[0] == 0.0f && sh.gyroBias[1] == 0.0f && sh.gyroBias[2] == 0.0f, "reset at the ground start");
}

static void testAccLiftoff(void)
{
  simCfg_t c;
  defaultSim(&c);
  c.thrustMax = 28000.0f;
  simRes_t r;
  runSim(&c, &r, 3);
  if (verbose) {
    printf("acc lift-off: true %.3f detected %.3f by %d\n", r.tLift, r.tPhase[CVS_AIR], r.liftBy);
  }
  CHECK(r.liftBy == CVS_LIFT_BY_ACC, "lift-off by the accelerometer, got %d", r.liftBy);
  /* a gentle lift-off (0.1 g peak): the CUSUM needs ~0.12 s, when the drone climbs at ~0.03 m/s */
  CHECK(r.tPhase[CVS_AIR] > r.tLift && r.tPhase[CVS_AIR] - r.tLift < 0.15, "detected %.3f s after the lift-off",
        r.tPhase[CVS_AIR] - r.tLift);
  CHECK(r.stats.flags & FSUM_CVZ_LIFT_ACC, "flag LIFT_ACC");
  CHECK(abs(r.stats.hTdMm) < 80, "h at touchdown %d mm", r.stats.hTdMm);

  /* no false lift-off: 3 s on the ground at 28000 with noise and no camera */
  defaultSim(&c);
  c.thrustMax = 28000.0f;
  c.camera = false;
  c.accNoise = 0.03;
  camVzShadowInit(&sh, NULL);
  rngState = 99;
  int lifts = 0;
  for (int ms = 0; ms < 4000; ms++) {
    const double t = ms * 1e-3;
    const float thrust = t >= 1.0 ? (float)fmin(60000.0 * (t - 1.0), 28000.0) : 0.0f;
    camVzShadowIn_t in = {
      .timeUs = 5000000ull + (uint64_t)ms * 1000ull, .accUpG = (float)(1.0 + c.g0Off + c.accNoise * grand()),
      .thrust = thrust, .gyroMaxDps = 3.0f, .groundStart = ms == 1000, .restingOnGround = t < 1.0,
      .still = t < 1.0, .liftoffHint = false,
    };
    camVzShadowStep(&sh, &in);
    lifts += sh.phase == CVS_AIR;
  }
  CHECK(lifts == 0, "no lift-off on the ground at 28000 with 30 mg noise (%d air steps)", lifts);
}

static void testAborted(void)
{
  camVzShadowInit(&sh, NULL);
  fsumCamVz_t s;
  camVzShadowFlightStats(&sh, &s);
  CHECK(s.flags == 0 && s.bias3Cmg == 0 && s.hTdMm == 0, "all 0 before the first sortie");
  for (int ms = 0; ms < 4000; ms++) {
    const double t = ms * 1e-3;
    const float thrust = (t >= 1.0 && t < 2.0) ? 15000.0f : 0.0f;
    camVzShadowIn_t in = {
      .timeUs = 1000ull * (uint64_t)ms + 7, .accUpG = 1.004f, .thrust = thrust, .gyroMaxDps = 1.0f,
      .groundStart = ms == 1000, .restingOnGround = t < 1.0 || t >= 2.5, .still = thrust == 0.0f,
      .liftoffHint = false,
    };
    camVzShadowStep(&sh, &in);
    if (ms == 1500) {
      CHECK(sh.phase == CVS_GROUND, "GROUND during the spin, got %d", sh.phase);
    }
    if (ms == 2200) {
      CHECK(sh.phase == CVS_GROUND, "GROUND until rested, got %d", sh.phase);
    }
  }
  CHECK(sh.phase == CVS_IDLE, "IDLE after an abandoned take-off, got %d", sh.phase);
  camVzShadowFlightStats(&sh, &s);
  CHECK(s.flags == FSUM_CVZ_RAN, "only RAN, got 0x%02x", s.flags);
  CHECK(s.g0Dmg == FSUM_CVZ_NONE && s.bias3Cmg == FSUM_CVZ_NONE && s.bias5Cmg == FSUM_CVZ_NONE &&
        s.biasTdCmg == FSUM_CVZ_NONE && s.hTdMm == FSUM_CVZ_NONE && s.vTdMms == FSUM_CVZ_NONE &&
        s.hMaxMm == FSUM_CVZ_NONE, "NONE fields without a lift-off");
  CHECK(s.airCs == 0 && s.firstOkDs == 255 && s.gainDpm == 0, "no air time");
  CHECK(sh.sortie == 1, "one sortie, got %u", (unsigned)sh.sortie);
}

static void testHopAndImplicit(void)
{
  simCfg_t c;
  defaultSim(&c);
  c.hop = true;
  c.hoverS = 2.0;
  simRes_t r;
  runSim(&c, &r, 5);
  CHECK(sh.sortie == 1, "a hop is the same sortie, got %u", (unsigned)sh.sortie);
  CHECK(r.statsAtStop.flags & FSUM_CVZ_TD, "first touchdown recorded");
  /* during the hop the drone sits on the floor at 45000 (the sim keeps it down): h restarts at 0 */
  CHECK(r.stats.airCs > r.statsAtStop.airCs + 30, "hop air time added: %u -> %u", r.statsAtStop.airCs,
        r.stats.airCs);
  CHECK(r.stats.flags & FSUM_CVZ_TD, "touchdown after the hop");
  CHECK(r.liftBy == CVS_LIFT_BY_THRUST, "first lift-off kept");

  defaultSim(&c);
  c.implicitStart = true;
  c.hoverS = 2.0;
  runSim(&c, &r, 6);
  CHECK(sh.sortie == 1 && (r.stats.flags & FSUM_CVZ_TD), "a take-off without the ground-start flag starts a sortie");
  CHECK(fabs(r.tPhase[CVS_GROUND] - 1.0) < 0.0005, "implicit start at the first thrust, got %.4f",
        r.tPhase[CVS_GROUND]);
}

static void testNoCameraAndFault(void)
{
  simCfg_t c;
  defaultSim(&c);
  c.camera = false;
  c.hoverS = 3.0;
  simRes_t r;
  runSim(&c, &r, 8);
  CHECK(!r.healthySeen && !r.camHealthyInput, "never healthy without the camera");
  CHECK(r.stats.bias3Cmg == FSUM_CVZ_NONE && r.stats.camAirCs == 0 && r.stats.firstOkDs == 255,
        "no wake without frames");
  CHECK(r.stats.flags & FSUM_CVZ_TD, "touchdown still recorded");
  CHECK(r.stats.accepted == 0 && r.stats.gainDpm == 0, "no camera statistics");

  defaultSim(&c);
  c.hoverS = 3.0;
  c.nanAtMs = 3000;
  runSim(&c, &r, 9);
  CHECK(r.stats.flags & FSUM_CVZ_FAULT, "fault flag after a NaN, got 0x%02x", r.stats.flags);
  CHECK(r.vzInputFinite, "vzInput finite after the fault");

  /* while faulted: the unusable vzInput values */
  camVzShadowInit(&sh, NULL);
  for (int ms = 0; ms < 1500; ms++) {
    camVzShadowIn_t in = { .timeUs = 2000000ull + 1000ull * (uint64_t)ms, .accUpG = ms == 1400 ? NAN : 1.01f,
                           .thrust = ms >= 1000 ? 40000.0f : 0.0f, .gyroMaxDps = 1.0f, .groundStart = ms == 1000,
                           .restingOnGround = ms < 1000, .still = ms < 1000 };
    camVzShadowStep(&sh, &in);
  }
  CHECK(sh.phase == CVS_AIR && sh.out.fault, "faulted in the air");
  vzInput_t vin = { .camHealthy = true, .camVzValid = true, .camVz = 1.0f, .camBias = 1.0f };
  camVzShadowVzInput(&sh, 1.0f, &vin);
  CHECK(!vin.camHealthy && !vin.camVzValid && vin.camVz == 0.0f && vin.camBias == 0.0f,
        "faulted observer: unusable vzInput");
  fsumCamVz_t s;
  camVzShadowFlightStats(&sh, &s);
  CHECK(s.flags & FSUM_CVZ_FAULT, "fault flag in the air");
  /* the next ground start clears it */
  camVzShadowIn_t in = { .timeUs = 99000000ull, .accUpG = 1.0f, .thrust = 10000.0f, .groundStart = true };
  camVzShadowStep(&sh, &in);
  camVzShadowFlightStats(&sh, &s);
  CHECK(!(s.flags & FSUM_CVZ_FAULT) && !sh.out.fault, "fault cleared by the next ground start");

  /* a NaN g0 from vert_ctrl never leaks out */
  defaultSim(&c);
  c.hoverS = 2.0;
  runSim(&c, &r, 10);
  vin.camVzValid = true;
  camVzShadowVzInput(&sh, NAN, &vin);
  CHECK(!vin.camVzValid && !vin.camHealthy && vin.camBias == 0.0f, "non-finite vzG0: unusable");
}

/* The bench load test ('camvz air', cam_vz_glue.c): a drone resting on the
 * bench, fed as the glue feeds it (one step per 10-sample block, blocks
 * 9990 us apart, a camera frame every 50 ms delivered 150 ms after capture),
 * looks airborne while the mode is on, and every frame after the camera wake
 * takes the update path; switching it off lands the bench sortie and the next
 * real ground start begins a clean one. */
typedef struct {
  int frames, skipped, quality, update;   /* update: CUSUM, GATE or ACCEPTED */
} benchCount_t;

static void benchRun(camVzShadowBench_t *b, uint64_t *tUs, uint32_t *seq, double seconds, bool goodFrames,
                     benchCount_t *n)
{
  const uint64_t blockUs = 9990;
  const int blocks = (int)(seconds * 1e6 / (double)blockUs);
  for (int k = 0; k < blocks; k++) {
    *tUs += blockUs;
    camVzShadowIn_t in = { .timeUs = *tUs, .accUpG = (float)(1.004 + 0.002 * grand()), .thrust = 0.0f,
                           .gyroMaxDps = 0.5f, .restingOnGround = true, .still = true };
    in.accMaxG = in.accUpG + 0.003f;
    camVzShadowBenchIn(b, &in);
    camVzShadowStep(&sh, &in);
    /* a frame every 5 blocks, captured 150 ms before this block */
    if (k % 5 == 4) {
      camVzObsFrame_t f;
      memset(&f, 0, sizeof(f));
      f.seq = ++*seq;
      f.frameTimeUs = *tUs - 150000u;
      f.dtUs = 5u * (uint32_t)blockUs;
      f.dy = (float)(0.1 * grand());
      f.dx = (float)(0.1 * grand());
      f.texture = goodFrames ? 30.0f : 2.0f;
      f.residual = goodFrames ? 2.0f : 20.0f;
      f.ok = goodFrames;
      camVzShadowBenchFrame(b, &f);
      const camVzObsDecision_t d = camVzShadowFrame(&sh, &f, *tUs);
      n->frames++;
      n->skipped += d == CAM_VZ_F_SKIPPED;
      n->quality += d == CAM_VZ_F_QUALITY;
      n->update += d == CAM_VZ_F_CUSUM || d == CAM_VZ_F_GATE || d == CAM_VZ_F_ACCEPTED;
    }
  }
}

static void testBenchAir(void)
{
  rngState = 77;
  camVzShadowInit(&sh, NULL);
  camVzShadowBench_t b = { .mode = CVS_BENCH_OFF };
  uint64_t t = 5000000;
  uint32_t seq = 0;
  benchCount_t n0 = { 0 };
  benchRun(&b, &t, &seq, 3.0, false, &n0);
  CHECK(sh.phase == CVS_IDLE && sh.sortie == 0 && n0.skipped == n0.frames, "off: idle, frames skipped (%d/%d)",
        n0.skipped, n0.frames);

  /* full: airborne at the first block, every frame updates after the 1.5 s wake (bad frames forced good) */
  b.mode = CVS_BENCH_FULL;
  benchCount_t n1 = { 0 };
  benchRun(&b, &t, &seq, 0.0105, false, &n1);
  CHECK(sh.sortie == 1 && sh.phase == CVS_AIR && sh.liftBy == CVS_LIFT_BY_THRUST && sh.priorSet,
        "full: sortie 1 in the air at once (phase %d, lift %d, prior %d)", sh.phase, sh.liftBy, sh.priorSet);
  benchRun(&b, &t, &seq, 6.0, false, &n1);
  CHECK(sh.phase == CVS_AIR && sh.sortie == 1, "still one bench sortie in the air");
  CHECK(n1.quality == 0, "full: no quality rejects, got %d", n1.quality);
  CHECK(n1.skipped >= 28 && n1.skipped <= 40, "full: frames skipped until the wake + window, got %d", n1.skipped);
  CHECK(n1.update == n1.frames - n1.skipped && n1.update >= 75, "full: every later frame updates (%d of %d)",
        n1.update, n1.frames);
  CHECK(sh.obs.cnt.accepted >= 70, "full: Kalman updates accepted, got %u", (unsigned)sh.obs.cnt.accepted);
  CHECK(fabsf(sh.out.v) < 0.1f && fabsf(sh.out.h) < 0.3f && !sh.out.fault, "bench estimate bounded: h %.3f v %.3f",
        sh.out.h, sh.out.v);

  /* air (not full): the real quality gate stays */
  b.mode = CVS_BENCH_AIR;
  benchCount_t n2 = { 0 };
  benchRun(&b, &t, &seq, 1.0, false, &n2);
  CHECK(n2.quality == n2.frames && sh.phase == CVS_AIR, "air: bad frames still fail the quality gate (%d/%d)",
        n2.quality, n2.frames);

  /* off: the real inputs land the bench sortie, then idle */
  b.mode = CVS_BENCH_OFF;
  benchCount_t n3 = { 0 };
  benchRun(&b, &t, &seq, 0.0105, true, &n3);
  CHECK(sh.phase == CVS_LANDED && sh.touchdown && !b.started, "off: bench sortie landed (phase %d)", sh.phase);
  benchRun(&b, &t, &seq, 1.0, true, &n3);
  CHECK(sh.phase == CVS_IDLE, "off: idle again, phase %d", sh.phase);

  /* the next real take-off is a clean sortie */
  t += 9990;
  camVzShadowIn_t in = { .timeUs = t, .accUpG = 1.0f, .accMaxG = 1.0f, .thrust = 12000.0f, .groundStart = true };
  camVzShadowStep(&sh, &in);
  CHECK(sh.sortie == 2 && sh.phase == CVS_GROUND && sh.obs.cnt.accepted == 0 && !sh.touchdown,
        "real ground start after the bench: sortie 2 from scratch");

  /* spin (props-off motor test under gcap): real thrust below the lift-off level still reads airborne,
   * and bad frames are forced good as in full */
  b.mode = CVS_BENCH_SPIN;
  b.started = false;
  camVzShadowIn_t sp = { .timeUs = t + 9990, .accUpG = 1.0f, .thrust = 15000.0f, .restingOnGround = true };
  camVzShadowBenchIn(&b, &sp);
  CHECK(sp.groundStart && sp.thrust == CVS_LIFT_THRUST && !sp.restingOnGround && b.started,
        "spin: ground start once, thrust raised to the lift-off level");
  camVzObsFrame_t bad;
  memset(&bad, 0, sizeof(bad));
  bad.texture = NAN;
  bad.residual = 50.0f;
  bad.gyroAngle[2] = 0.3f;
  camVzShadowBenchFrame(&b, &bad);
  CHECK(bad.ok && bad.texture >= 20.0f && bad.residual <= 1.0f && bad.gyroAngle[2] == 0.0f, "spin: frame forced good");

  /* the helper leaves real inputs alone when off */
  camVzShadowIn_t a = { .timeUs = 1, .accUpG = 1.0f, .thrust = 0.0f, .restingOnGround = true, .still = true };
  camVzShadowIn_t a0 = a;
  b.mode = CVS_BENCH_OFF;
  b.started = true;
  camVzShadowBenchIn(&b, &a);
  CHECK(memcmp(&a, &a0, sizeof(a)) == 0 && !b.started, "off: inputs untouched");

  /* Left on into a real take-off without a ground start (hand launch, soft-start off): the glue ends the
   * mode at the first block with real thrust, and that block starts a fresh sortie, so no bench air time,
   * drift or prior carries into the flight. */
  camVzShadowInit(&sh, NULL);
  b.mode = CVS_BENCH_FULL;
  b.started = false;
  benchCount_t n4 = { 0 };
  benchRun(&b, &t, &seq, 3.0, false, &n4);
  const uint32_t benchSortie = sh.sortie;
  CHECK(benchSortie == 1 && sh.phase == CVS_AIR && sh.obs.cnt.accepted > 0, "bench sortie running (sortie %u)",
        (unsigned)benchSortie);
  b.mode = CVS_BENCH_OFF;
  t += 9990;
  camVzShadowIn_t real = { .timeUs = t, .accUpG = 1.0f, .accMaxG = 1.0f, .thrust = 35000.0f };
  camVzShadowBenchIn(&b, &real);
  CHECK(real.groundStart && !b.started, "off with real thrust after a bench sortie: a ground start");
  camVzShadowStep(&sh, &real);
  CHECK(sh.sortie == benchSortie + 1 && sh.obs.cnt.accepted == 0 && !sh.touchdown,
        "the real take-off is sortie %u from scratch (got %u, accepted %u)", (unsigned)(benchSortie + 1),
        (unsigned)sh.sortie, (unsigned)sh.obs.cnt.accepted);
}

/* g0: the median of the ring without its newest 5 samples, against a sort. */
static int cmpi16(const void *a, const void *b)
{
  return *(const int16_t *)a - *(const int16_t *)b;
}

static void testG0Median(void)
{
  rngState = 1234;
  for (int trial = 0; trial < 200; trial++) {
    camVzShadowInit(&sh, NULL);
    const int n = 1 + (int)(urand() * 160.0);   /* rest samples pushed */
    int16_t vals[200];
    uint64_t us = 1000000;
    for (int k = 0; k < n; k++) {
      int16_t v = (int16_t)((int)(urand() * 400.0) - 200);   /* 0.1 mg: +-20 mg */
      if (trial % 5 == 0) {
        v = (int16_t)(v / 50);   /* many ties */
      }
      vals[k] = v;
      camVzShadowIn_t in = { .timeUs = us, .accUpG = 1.0f + v * 1e-4f, .thrust = 0.0f, .gyroMaxDps = 0.0f,
                             .restingOnGround = true, .still = true };
      camVzShadowStep(&sh, &in);
      us += 10000;
    }
    /* ground start and an immediate lift-off by thrust (the step's own sample joins the ring) */
    const int16_t vLast = 7;
    camVzShadowIn_t in = { .timeUs = us, .accUpG = 1.0f + vLast * 1e-4f, .thrust = 35000.0f, .gyroMaxDps = 0.0f,
                           .groundStart = true };
    camVzShadowStep(&sh, &in);
    (void)vLast;   /* its bucket is still open: not in the ring */
    const int total = n;
    const int kept = total > CVS_G0_RING ? CVS_G0_RING : total;
    int16_t w[200];
    int m = kept;
    if (m - CVS_G0_SKIP_LAST >= CVS_G0_PRIOR_MIN) {
      m -= CVS_G0_SKIP_LAST;
    }
    for (int k = 0; k < m; k++) {
      w[k] = vals[total - kept + k];
    }
    if (kept < CVS_G0_PRIOR_MIN) {
      CHECK(!sh.priorSet && sh.g0 == 1.0f, "n=%d: no prior with %d samples", n, kept);
      continue;
    }
    qsort(w, (size_t)m, sizeof(int16_t), cmpi16);
    const double med = m % 2 ? w[m / 2] : 0.5 * (w[m / 2 - 1] + w[m / 2]);
    CHECK(sh.priorSet && fabs((sh.g0 - 1.0) * 1e4 - med) < 0.02, "n=%d: g0 %.3f vs median %.2f (0.1 mg)", n,
          (sh.g0 - 1.0) * 1e4, med);
  }

  /* a gap of more than 50 ms between baseline samples restarts the ring */
  camVzShadowInit(&sh, NULL);
  uint64_t us = 1000000;
  for (int k = 0; k < 50; k++, us += 10000) {
    camVzShadowIn_t in = { .timeUs = us, .accUpG = 1.050f, .restingOnGround = true, .still = true };
    camVzShadowStep(&sh, &in);
  }
  us += 200000;   /* picked up: not still */
  for (int k = 0; k < 30; k++, us += 10000) {
    camVzShadowIn_t in = { .timeUs = us, .accUpG = 1.002f, .restingOnGround = true, .still = true };
    camVzShadowStep(&sh, &in);
  }
  camVzShadowIn_t in = { .timeUs = us, .accUpG = 1.002f, .thrust = 31000.0f, .groundStart = true };
  camVzShadowStep(&sh, &in);
  CHECK(fabs(sh.g0 - 1.002f) < 1e-5f, "old rest samples dropped after a gap: g0 %.4f", sh.g0);

  /* 1 kHz steps feed the ring at 100 Hz */
  camVzShadowInit(&sh, NULL);
  us = 1000000;
  for (int k = 0; k < 1000; k++, us += 1000) {
    camVzShadowIn_t in2 = { .timeUs = us, .accUpG = 1.0f, .restingOnGround = true, .still = true };
    camVzShadowStep(&sh, &in2);
  }
  CHECK(sh.g0N == 99, "1 s of 1 kHz rest = 100 buckets, 99 closed, got %u", sh.g0N);
  CHECK(sh.g0AccN == 10, "the open bucket has 10 samples, got %u", sh.g0AccN);
}

/* Filter steps actually taken: camVzObsImu() moves tState on every step
 * (the first one only starts the observer). */
static uint32_t obsSteps;
static uint32_t obsLastState;
static bool obsLastStarted;

static void countObsStep(void)
{
  if (sh.obs.started && (!obsLastStarted || sh.obs.tState != obsLastState)) {
    obsSteps++;
  }
  obsLastStarted = sh.obs.started;
  obsLastState = sh.obs.tState;
}

/* The drone calls camVzShadowStep() once per cam_vz_glue.c block (10 IMU
 * samples, closed at the first sample >= 8500 us after the block's first).
 * The filter must step on every block whatever the MPU's clock (a fraction of
 * a percent fast or slow), the stabilizer's missed samples (~4 % in flight on
 * a426962) and the interrupt timestamp jitter. With a strict 10000 us gate a
 * 999.9 us IMU period gave 9999 us blocks and the filter stepped on every
 * other one (~50 Hz). A 1 kHz per-sample caller keeps 10-sample blocks. */
static void testGlueBlockRate(void)
{
  const double periods[] = { 960.0, 995.0, 999.0, 999.9, 1000.0, 1000.1, 1001.0, 1005.0, 1050.0 };
  for (size_t pi = 0; pi < sizeof(periods) / sizeof(periods[0]); pi++) {
    for (int skipPct = 0; skipPct <= 5; skipPct += 5) {
      camVzShadowInit(&sh, NULL);
      obsSteps = 0;
      obsLastStarted = false;
      rngState = 7u + (uint32_t)pi;
      double tImu = 1e6;
      uint64_t start = 0;
      bool open = false;
      double sum = 0.0;
      int n = 0, blocks = 0;
      for (int k = 0; k < 3000; k++) {
        do {
          tImu += periods[pi];
        } while (urand() * 100.0 < skipPct);
        const uint64_t t = (uint64_t)llround(tImu + 6.0 * (urand() - 0.5));   /* +-3 us jitter */
        if (!open) {
          open = true;
          start = t;
          sum = 0.0;
          n = 0;
        }
        sum += 1.0;
        n++;
        if (t > start && t - start >= 8500u) {
          camVzShadowIn_t in = { .timeUs = t, .accUpG = (float)(sum / n), .accMaxG = 1.0f, .thrust = 0.0f,
                                 .gyroMaxDps = 0.0f, .restingOnGround = true, .still = true };
          camVzShadowStep(&sh, &in);
          countObsStep();
          blocks++;
          open = false;
        }
      }
      CHECK(obsSteps >= (uint32_t)blocks - 1u && obsSteps <= (uint32_t)blocks,
            "IMU period %.1f us, %d %% missed: the filter stepped on %u of %d glue blocks", periods[pi], skipPct,
            obsSteps, blocks);
    }
  }
  /* per-sample callers (the replay, the synthetic flights): 10 samples per step */
  const double perSample[] = { 999.0, 1000.0, 1001.0 };
  for (size_t pi = 0; pi < sizeof(perSample) / sizeof(perSample[0]); pi++) {
    camVzShadowInit(&sh, NULL);
    obsSteps = 0;
    obsLastStarted = false;
    for (int k = 0; k < 2001; k++) {
      camVzShadowIn_t in = { .timeUs = 1000000ull + (uint64_t)llround(k * perSample[pi]), .accUpG = 1.0f,
                             .restingOnGround = true, .still = true };
      camVzShadowStep(&sh, &in);
      countObsStep();
    }
    CHECK(obsSteps == 200 || obsSteps == 201, "1 kHz samples (%.0f us): %u filter steps in 2 s, want 200",
          perSample[pi], obsSteps);
  }
}

/* ---- replay of the logged flights ------------------------------------------------ */

#define MAX_ROWS 20000   /* a 180 s recorder dump (docs/autonomy-a2b.md 19.3) */
enum { C_TICK, C_FID, C_THRUST, C_ROLL, C_PITCH, C_GX, C_GY, C_GZ, C_AX, C_AY, C_AZ, C_SEQ, C_OK, C_TEX, C_RES,
       C_DT, C_AGE, C_DX, C_DY, C_SC, C_ROT, C_FGX, C_FGY, C_FGZ, C_YAW, C_N };
static const char *colNames[C_N] = {
  "tickMs", "flightId", "thrust", "roll_cdeg", "pitch_cdeg", "gx_ddps", "gy_ddps", "gz_ddps", "ax_mg", "ay_mg",
  "az_mg", "flowSeq", "flowOk", "flowTexture", "flowResidual_dg", "flowDt_100us", "flowAge_ms", "flowDx_cpx",
  "flowDy_cpx", "flowScale_e4", "flowRot_e4", "flowGx_e4", "flowGy_e4", "flowGz_e4", "yaw_cdeg",
};
static long rows[MAX_ROWS][C_N];
static int nRows;

static bool loadCsv(const char *path)
{
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    return false;
  }
  static char line[4096];
  int idx[C_N];
  for (int c = 0; c < C_N; c++) {
    idx[c] = -1;
  }
  if (fgets(line, sizeof(line), f) == NULL) {
    fclose(f);
    return false;
  }
  int col = 0;
  for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
    for (int c = 0; c < C_N; c++) {
      if (strcmp(tok, colNames[c]) == 0) {
        idx[c] = col;
      }
    }
  }
  for (int c = 0; c < C_N; c++) {
    if (idx[c] < 0) {
      fclose(f);
      return false;
    }
  }
  nRows = 0;
  while (nRows < MAX_ROWS && fgets(line, sizeof(line), f) != NULL) {
    col = 0;
    for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
      for (int c = 0; c < C_N; c++) {
        if (idx[c] == col) {
          rows[nRows][c] = strtol(tok, NULL, 10);
        }
      }
    }
    nRows++;
  }
  fclose(f);
  return true;
}

typedef struct {
  int file, fid;
  double bAbs, sigmaB, tLiftoff, tTouch;
  char grade;
  bool ok;
} truth_t;
static truth_t truth[200];
static int nTruth;

static void loadTruth(const char *path)
{
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    printf("note: no bias truth at %s\n", path);
    return;
  }
  static char line[8192];
  int cFile = -1, cFid = -1, cStatus = -1, cGrade = -1, cB = -1, cS = -1, cTl = -1, cTt = -1;
  if (fgets(line, sizeof(line), f) == NULL) {
    fclose(f);
    return;
  }
  int col = 0;
  for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
    if (!strcmp(tok, "file")) cFile = col;
    else if (!strcmp(tok, "fid")) cFid = col;
    else if (!strcmp(tok, "status")) cStatus = col;
    else if (!strcmp(tok, "grade")) cGrade = col;
    else if (!strcmp(tok, "b_abs")) cB = col;
    else if (!strcmp(tok, "sigma_b")) cS = col;
    else if (!strcmp(tok, "t_liftoff")) cTl = col;
    else if (!strcmp(tok, "t_touch")) cTt = col;
  }
  while (nTruth < 200 && fgets(line, sizeof(line), f) != NULL) {
    truth_t *t = &truth[nTruth];
    memset(t, 0, sizeof(*t));
    t->bAbs = t->sigmaB = t->tLiftoff = t->tTouch = NAN;
    /* split on commas keeping empty fields (no quoted commas before the needed columns) */
    char *p = line;
    for (col = 0; p != NULL; col++) {
      char *comma = strchr(p, ',');
      if (comma) {
        *comma = '\0';
      }
      if (col == cFile) t->file = atoi(p);
      else if (col == cFid) t->fid = atoi(p);
      else if (col == cStatus) t->ok = strcmp(p, "ok") == 0;
      else if (col == cGrade) t->grade = p[0];
      else if (col == cB && *p) t->bAbs = atof(p);
      else if (col == cS && *p) t->sigmaB = atof(p);
      else if (col == cTl && *p) t->tLiftoff = atof(p);
      else if (col == cTt && *p) t->tTouch = atof(p);
      p = comma ? comma + 1 : NULL;
    }
    nTruth++;
  }
  fclose(f);
}

static const truth_t *findTruth(int file, int fid)
{
  for (int i = 0; i < nTruth; i++) {
    if (truth[i].file == file && truth[i].fid == fid) {
      return &truth[i];
    }
  }
  return NULL;
}

static const int camFiles[] = { 6, 8, 9, 10, 11, 12, 15, 16, 17, 18, 19, 20, 21 };
static double glueP;   /* -g: IMU period (us) of the glue-block replay; 0: per-sample replay */

typedef struct {
  int n, n3, ok3, n5, ok5;
  double err3[100], err5[100];
  int nLift;
  double dLift[100];
  int takeoffs, lifted, touched, healthy;
  uint32_t blocks, steps;
  double okFrac[100];
  double hTd[100], airS[100], camS[100];
  int tdFile[100];
  int nTd;
  unsigned acc, rej;
} replayStats_t;

static int cmpd(const void *a, const void *b)
{
  const double x = *(const double *)a, y = *(const double *)b;
  return x < y ? -1 : (x > y ? 1 : 0);
}

static double pct(double *v, int n, double p)
{
  if (n == 0) {
    return NAN;
  }
  qsort(v, (size_t)n, sizeof(double), cmpd);
  return v[(int)fmin(n - 1, floor(p * n))];
}

/* The attitude estimator's rate correction the drone would have fed (rad/s, body x and y; the yaw part is
 * ~0 on a level drone and not rebuilt), from rows [i0, i1) of one take-off (tickMs: ms on these logs): minus
 * the gyro rotation the logged attitude did not follow (droll = +int gx, dpitch = -int gy). Inside each
 * frame's motion window from the frame's 1 kHz gyro integral flowG*, elsewhere from 10-row chunks of the
 * logged 100 Hz gyro. */
static float rateCorrRow[MAX_ROWS][2];

static double attRad(int i, int ax)
{
  return (double)rows[i][ax == 0 ? C_ROLL : C_PITCH] / 100.0 * M_PI / 180.0;
}

static bool attAt(int i0, int i1, double tMs, int ax, double *a)
{
  if (tMs < (double)rows[i0][C_TICK] || tMs > (double)rows[i1 - 1][C_TICK]) {
    return false;
  }
  int k = i0;
  while (k + 1 < i1 && (double)rows[k + 1][C_TICK] < tMs) {
    k++;
  }
  if (k + 1 >= i1) {
    *a = attRad(k, ax);
    return true;
  }
  const double t0 = (double)rows[k][C_TICK], t1 = (double)rows[k + 1][C_TICK];
  const double f = t1 > t0 ? (tMs - t0) / (t1 - t0) : 0.0;
  *a = attRad(k, ax) + f * (attRad(k + 1, ax) - attRad(k, ax));
  return true;
}

static void rebuildRateCorr(int i0, int i1)
{
  const double S[2] = { 1.0, -1.0 };
  for (int i = i0; i < i1; i++) {
    rateCorrRow[i][0] = rateCorrRow[i][1] = 0.0f;
  }
  for (int c0 = i0; c0 + 1 < i1; c0 += 10) {
    const int c1 = c0 + 10 < i1 - 1 ? c0 + 10 : i1 - 1;
    const double T = (double)(rows[c1][C_TICK] - rows[c0][C_TICK]) * 1e-3;
    if (!(T > 0.0)) {
      continue;
    }
    for (int ax = 0; ax < 2; ax++) {
      double g = 0.0;
      for (int r = c0; r < c1; r++) {
        g += (double)rows[r][ax == 0 ? C_GX : C_GY] / 10.0 * M_PI / 180.0 * (double)(rows[r + 1][C_TICK] - rows[r][C_TICK]) * 1e-3;
      }
      const double x = (g - S[ax] * (attRad(c1, ax) - attRad(c0, ax))) / T;
      for (int r = c0; r < c1; r++) {
        rateCorrRow[r][ax] = (float)-x;
      }
    }
  }
  for (int k = i0 + 1; k < i1; k++) {
    if (rows[k][C_SEQ] == 0 || rows[k][C_SEQ] == rows[k - 1][C_SEQ] || rows[k][C_DT] <= 0 ||
        (rows[k][C_FGX] == 0 && rows[k][C_FGY] == 0 && rows[k][C_FGZ] == 0)) {
      continue;
    }
    const double dt = (double)rows[k][C_DT] * 1e-4;
    const double t1 = (double)(rows[k][C_TICK] - rows[k][C_AGE]) + 15.0, t0 = t1 - dt * 1000.0;
    for (int ax = 0; ax < 2; ax++) {
      double a0, a1;
      if (!attAt(i0, i1, t0, ax, &a0) || !attAt(i0, i1, t1, ax, &a1)) {
        break;
      }
      const double e = (double)rows[k][ax == 0 ? C_FGX : C_FGY] / 1e4 - S[ax] * (a1 - a0);
      for (int r = i0; r < i1; r++) {
        if ((double)rows[r][C_TICK] >= t0 && (double)rows[r][C_TICK] < t1) {
          rateCorrRow[r][ax] = (float)(-e / dt);
        }
      }
    }
  }
}

/* The filter's own rate correction over rows [i0, i1) (twoKiAir kiAir), from the logged attitude of row i0, the
 * logged gyro and accelerometer at the recorder's time (one update per row), airborne above 30000 thrust. */
static void replayFilterRateCorr(int i0, int i1, float kiAir)
{
  sensfusion6SetKiAir(kiAir);
  const double r0 = rows[i0][C_ROLL] / 100.0 * M_PI / 180.0, p0 = rows[i0][C_PITCH] / 100.0 * M_PI / 180.0;
  const double cr = cos(r0 / 2), sr = sin(r0 / 2), cp = cos(-p0 / 2), sp = sin(-p0 / 2);
  qw = (float)(cr * cp);
  qx = (float)(sr * cp);
  qy = (float)(cr * sp);
  qz = (float)(-sr * sp);
  integralFBx = integralFBy = integralFBz = 0.0f;
  for (int i = i0; i < i1; i++) {
    const double dtr = i > i0 ? (double)(rows[i][C_TICK] - rows[i - 1][C_TICK]) * 1e-3 : 0.01;
    sensfusion6SetAirborne(rows[i][C_THRUST] > 30000);
    sensfusion6UpdateQ((float)rows[i][C_GX] / 10.0f, (float)rows[i][C_GY] / 10.0f, (float)rows[i][C_GZ] / 10.0f,
                       (float)rows[i][C_AX] / 1000.0f, (float)rows[i][C_AY] / 1000.0f, (float)rows[i][C_AZ] / 1000.0f,
                       dtr > 0.0 && dtr < 0.1 ? (float)dtr : 0.01f);
    rateCorrRow[i][0] = sensfusion6RateCorr[0];
    rateCorrRow[i][1] = sensfusion6RateCorr[1];
  }
  sensfusion6SetAirborne(false);
  sensfusion6SetKiAir(0.040f);
}

static fsumCamVz_t replayLast;   /* the latest take-off's flight-summary fields (at the motor stop) */
static long replayAtTick = -1;   /* > 0: the shadow's h and v just before the row of this tick */
static float replayAtH, replayAtV;

static void replayTakeoff(int file, int i0, int i1, replayStats_t *rs, bool useRateCorr)
{
  camVzShadowInit(&sh, NULL);
  obsSteps = 0;
  obsLastStarted = false;
  /* frame seq unwrapped from the logged seq % 255 + 1 */
  uint32_t seq = 0;
  long lastLogged = -1;
  const long tick0 = rows[i0][C_TICK];
  double tAirDetect = -1.0;
  fsumCamVz_t atStop;
  bool haveStop = false;
  int zeroRun = 0;
  for (int i = i0; i < i1; i++) {
    const long *r = rows[i];
    const float rr = (float)r[C_ROLL] / 100.0f * 0.0174533f, pp = (float)r[C_PITCH] / 100.0f * 0.0174533f;
    const float aUp = ((float)r[C_AX] * sinf(pp) + (float)r[C_AY] * cosf(pp) * sinf(rr) +
                       (float)r[C_AZ] * cosf(pp) * cosf(rr)) / 1000.0f;
    const float an = sqrtf((float)(r[C_AX] * r[C_AX] + r[C_AY] * r[C_AY] + r[C_AZ] * r[C_AZ])) / 1000.0f;
    const bool still = labs(r[C_GX]) < 100 && labs(r[C_GY]) < 100 && labs(r[C_GZ]) < 100 && fabsf(an - 1.0f) < 0.15f;
    const float gmax = fmaxf(fabsf((float)r[C_GX]), fabsf((float)r[C_GY])) / 10.0f;
    zeroRun = r[C_THRUST] == 0 ? zeroRun + 1 : 0;
    if (r[C_TICK] == replayAtTick) {
      replayAtH = sh.prevH;
      replayAtV = sh.prevV;
    }
    const bool newFrame = r[C_SEQ] != 0 && r[C_SEQ] != lastLogged;
    /* -g: the row's ten samples as one glue block ending at its tenth, the frame right after it */
    const int kFirst = glueP > 0.0 ? 9 : 0;
    for (int k = kFirst; k < 10; k++) {
      const uint64_t us = glueP > 0.0 ? (uint64_t)((int64_t)tick0 * 1000 + llround((double)(r[C_TICK] - tick0 + k) * glueP))
                                      : (uint64_t)r[C_TICK] * 1000ull + (uint64_t)k * 1000ull;
      camVzShadowIn_t in = {
        .timeUs = us, .accUpG = aUp, .thrust = (float)r[C_THRUST], .gyroMaxDps = gmax,
        .groundStart = i == i0 && k == kFirst, .restingOnGround = zeroRun >= 50 && still, .still = still,
        .liftoffHint = false,
        .rateCorr = { useRateCorr ? rateCorrRow[i][0] : 0.0f, useRateCorr ? rateCorrRow[i][1] : 0.0f, 0.0f },
      };
      const camVzShadowPhase_t before = sh.phase;
      camVzShadowStep(&sh, &in);
      countObsStep();
      rs->blocks++;
      if (before != CVS_AIR && sh.phase == CVS_AIR && tAirDetect < 0.0) {
        tAirDetect = (double)(us - (uint64_t)tick0 * 1000ull) * 1e-6;
      }
      if (before == CVS_AIR && sh.phase == CVS_LANDED && !haveStop) {
        camVzShadowFlightStats(&sh, &atStop);
        haveStop = true;
      }
      if (k == kFirst && newFrame) {
        if (lastLogged >= 0) {
          seq += (uint32_t)((r[C_SEQ] - lastLogged + 255) % 255);
        } else {
          seq = 1000;
        }
        lastLogged = r[C_SEQ];
        camVzObsFrame_t f = {
          .seq = seq,
          .frameTimeUs = glueP > 0.0 ? (uint64_t)((int64_t)tick0 * 1000 +
                                                  llround((double)(r[C_TICK] - r[C_AGE] + 1 - tick0) * glueP))
                                     : (uint64_t)(r[C_TICK] - r[C_AGE] + 1) * 1000ull,
          .dtUs = (uint32_t)r[C_DT] * 100u,
          .dx = (float)r[C_DX] / 100.0f, .dy = (float)r[C_DY] / 100.0f,
          .scale = (float)r[C_SC] / 1e4f, .rot = (float)r[C_ROT] / 1e4f,
          .gyroAngle = { (float)r[C_FGX] / 1e4f, (float)r[C_FGY] / 1e4f, (float)r[C_FGZ] / 1e4f },
          .texture = (float)r[C_TEX], .residual = (float)r[C_RES] / 10.0f, .ok = r[C_OK] != 0,
        };
        camVzShadowFrame(&sh, &f, us);
      }
    }
  }
  rs->steps += obsSteps;
  fsumCamVz_t s;
  if (haveStop) {
    s = atStop;
  } else {
    camVzShadowFlightStats(&sh, &s);
  }
  replayLast = s;
  const int fid = (int)rows[i0][C_FID];
  const truth_t *t = findTruth(file, fid);
  rs->takeoffs++;
  rs->lifted += !!(s.flags & FSUM_CVZ_LIFT);
  rs->touched += !!(s.flags & FSUM_CVZ_TD);
  rs->healthy += s.okCs > 0;
  if (s.camAirCs > 0 && rs->n < 100) {
    rs->okFrac[rs->n++] = (double)s.okCs / s.camAirCs;
  }
  if ((s.flags & FSUM_CVZ_TD) && rs->nTd < 100) {
    rs->hTd[rs->nTd] = s.hTdMm / 1000.0;
    rs->airS[rs->nTd] = s.airCs / 100.0;
    rs->camS[rs->nTd] = s.camAirCs / 100.0;
    rs->tdFile[rs->nTd] = file;
    rs->nTd++;
  }
  rs->acc += s.accepted;
  rs->rej += s.rejected;
  const bool graded = t && t->ok && (t->grade == 'A' || t->grade == 'B') && isfinite(t->bAbs);
  char e3[16] = "     -", e5[16] = "     -";
  if (graded && s.bias3Cmg != FSUM_CVZ_NONE && rs->n3 < 100) {
    const double e = cmg(s.bias3Cmg) - t->bAbs;
    rs->err3[rs->n3++] = fabs(e);
    rs->ok3 += fabs(e) <= 3.0;
    snprintf(e3, sizeof(e3), "%+6.2f", e);
  }
  if (graded && s.bias5Cmg != FSUM_CVZ_NONE && rs->n5 < 100) {
    const double e = cmg(s.bias5Cmg) - t->bAbs;
    rs->err5[rs->n5++] = fabs(e);
    rs->ok5 += fabs(e) <= 3.0;
    snprintf(e5, sizeof(e5), "%+6.2f", e);
  }
  char dl[16] = "     -";
  if (t && t->ok && isfinite(t->tLiftoff) && tAirDetect >= 0.0 && rs->nLift < 100) {
    rs->dLift[rs->nLift++] = tAirDetect - t->tLiftoff;
    snprintf(dl, sizeof(dl), "%+6.2f", tAirDetect - t->tLiftoff);
  }
  if (verbose) {
    printf("  %s f%-2d fid %3d %c | flags 0x%02x g0 %+6.1f b3 %+7.2f b5 %+7.2f btd %+7.2f mg | G %5.1f | hTd %+5.2f "
           "m vTd %+5.2f hMax %5.2f | air %5.2f s ok %3.0f%% first %4.1f s gap %4.1f s | acc %4u rej %3u al %u yaw %u"
           " | err3 %s err5 %s mg | dLift %s s | gyro bias %+.2f %+.2f deg/s\n",
           useRateCorr ? "gb" : "--", file, fid, t ? t->grade : '?', s.flags, s.g0Dmg == FSUM_CVZ_NONE ? NAN : s.g0Dmg / 10.0,
           s.bias3Cmg == FSUM_CVZ_NONE ? NAN : cmg(s.bias3Cmg), s.bias5Cmg == FSUM_CVZ_NONE ? NAN : cmg(s.bias5Cmg),
           s.biasTdCmg == FSUM_CVZ_NONE ? NAN : cmg(s.biasTdCmg), s.gainDpm / 10.0,
           s.hTdMm == FSUM_CVZ_NONE ? NAN : s.hTdMm / 1000.0, s.vTdMms == FSUM_CVZ_NONE ? NAN : s.vTdMms / 1000.0,
           s.hMaxMm == FSUM_CVZ_NONE ? NAN : s.hMaxMm / 1000.0, s.airCs / 100.0,
           s.camAirCs ? 100.0 * s.okCs / s.camAirCs : 0.0, s.firstOkDs == 255 ? NAN : s.firstOkDs / 10.0,
           s.gapMaxCs / 100.0, s.accepted, s.rejected, s.alarms, s.yawResets, e3, e5, dl, sh.gyroBias[0] * 180.0 / M_PI,
           sh.gyroBias[1] * 180.0 / M_PI);
  }
}

/* Mean touchdown height per second with the camera over the take-offs of files [lo, hi]. */
static double driftRate(const replayStats_t *rs, int lo, int hi, int *n)
{
  double sum = 0.0;
  *n = 0;
  for (int k = 0; k < rs->nTd; k++) {
    if (rs->tdFile[k] >= lo && rs->tdFile[k] <= hi && rs->camS[k] > 3.0) {
      sum += rs->hTd[k] / rs->camS[k];
      (*n)++;
    }
  }
  return *n ? sum / *n : NAN;
}

static void replay(const char *dir)
{
  replayStats_t rs, rs0;   /* with the gyro-bias correction (the firmware), without it */
  memset(&rs, 0, sizeof(rs));
  memset(&rs0, 0, sizeof(rs0));
  static replayStats_t rsF0, rsF1;   /* the replayed filter's rate correction: twoKiAir 0 / 0.040 */
  memset(&rsF0, 0, sizeof(rsF0));
  memset(&rsF1, 0, sizeof(rsF1));
  int files = 0;
  for (size_t fi = 0; fi < sizeof(camFiles) / sizeof(camFiles[0]); fi++) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/flight%d.csv", dir, camFiles[fi]);
    if (!loadCsv(path)) {
      continue;
    }
    files++;
    /* take-offs: a flightId change or a recorder gap > 200 ms (biastruth.py), starting at thrust > 0 */
    int i = 0;
    while (i < nRows) {
      int j = i + 1;
      while (j < nRows && rows[j][C_FID] == rows[i][C_FID] && rows[j][C_TICK] - rows[j - 1][C_TICK] <= 200) {
        j++;
      }
      int s0 = i;
      while (s0 < j && rows[s0][C_THRUST] == 0) {
        s0++;
      }
      if (j - s0 >= 50) {
        rebuildRateCorr(s0, j);
        replayTakeoff(camFiles[fi], s0, j, &rs0, false);
        replayTakeoff(camFiles[fi], s0, j, &rs, true);
        replayFilterRateCorr(s0, j, 0.0f);
        replayTakeoff(camFiles[fi], s0, j, &rsF0, true);
        replayFilterRateCorr(s0, j, 0.040f);
        replayTakeoff(camFiles[fi], s0, j, &rsF1, true);
      }
      i = j;
    }
  }
  if (files == 0) {
    printf("replay skipped: no camera flight*.csv in %s\n", dir);
    return;
  }
  printf("replay: %d files, %d take-offs, %d lifted off, %d touched down, %d ever healthy\n", files, rs.takeoffs,
         rs.lifted, rs.touched, rs.healthy);
  printf("  %s: %u filter steps for %u shadow steps\n", glueP > 0.0 ? "glue blocks" : "1 kHz samples", rs.steps,
         rs.blocks);
  if (glueP > 0.0) {
    CHECK(rs.steps + (uint32_t)rs.takeoffs >= rs.blocks, "glue replay (IMU period %.1f us): the filter stepped on "
          "%u of %u blocks", glueP, rs.steps, rs.blocks);
  }
  const double okMed = pct(rs.okFrac, rs.n, 0.5);
  printf("  healthy fraction of airborne-after-wake time: median %.0f %% (n=%d)\n", 100.0 * okMed, rs.n);
  if (rs.nTd) {
    double rate[100];
    for (int k = 0; k < rs.nTd; k++) {
      rate[k] = rs.airS[k] > 0.5 ? fabs(rs.hTd[k]) / rs.airS[k] : 0.0;
    }
    double ah[100];
    for (int k = 0; k < rs.nTd; k++) {
      ah[k] = fabs(rs.hTd[k]);
    }
    printf("  |h at touchdown|: median %.2f m, p80 %.2f m, max %.2f m; |h| / airborne time median %.3f m/s (n=%d)\n",
           pct(ah, rs.nTd, 0.5), pct(ah, rs.nTd, 0.8), pct(ah, rs.nTd, 1.0), pct(rate, rs.nTd, 0.5), rs.nTd);
  }
  CHECK(rs.takeoffs >= 30, "replayed %d take-offs", rs.takeoffs);
  CHECK(rs.lifted >= rs.takeoffs - 4, "lift-off in %d of %d", rs.lifted, rs.takeoffs);
  if (nTruth) {
    if (rs.n3) {
      printf("  offset at wake + 3 s vs study truth (A/B): n=%d within +-3 mg %.1f %%, |err| median %.2f p80 %.2f max %.2f mg\n",
             rs.n3, 100.0 * rs.ok3 / rs.n3, pct(rs.err3, rs.n3, 0.5), pct(rs.err3, rs.n3, 0.8),
             pct(rs.err3, rs.n3, 1.0));
    }
    if (rs.n5) {
      printf("  offset at wake + 5 s vs study truth (A/B): n=%d within +-3 mg %.1f %%, |err| median %.2f p80 %.2f max %.2f mg\n",
             rs.n5, 100.0 * rs.ok5 / rs.n5, pct(rs.err5, rs.n5, 0.5), pct(rs.err5, rs.n5, 0.8),
             pct(rs.err5, rs.n5, 1.0));
    }
    if (rs.nLift) {
      double d[100];
      memcpy(d, rs.dLift, sizeof(double) * (size_t)rs.nLift);
      printf("  on-board lift-off minus the study's offline lift-off: median %+.2f s, min %+.2f, max %+.2f (n=%d)\n",
             pct(d, rs.nLift, 0.5), pct(d, rs.nLift, 0.0), pct(d, rs.nLift, 1.0), rs.nLift);
      /* The study's lift-off is back-dated on purpose ("err early"); flight 8 take-off 2 rocks on the
       * ground at 24-29k thrust (gyro 100-400 dps) and climbs cleanly only ~0.6 s after it. */
      const double med = pct(d, rs.nLift, 0.5), mx = pct(d, rs.nLift, 1.0);
      CHECK(fabs(med) < 0.1 && mx < 0.3, "on-board lift-off vs the study's: median %+.2f s, max %+.2f s", med, mx);
    }
    CHECK(rs.n3 >= 20 && rs.ok3 >= (int)(0.75 * rs.n3), "offset at +3 s within 3 mg in %d of %d", rs.ok3, rs.n3);
    CHECK(rs.n5 >= 20 && rs.ok5 >= (int)(0.80 * rs.n5), "offset at +5 s within 3 mg in %d of %d", rs.ok5, rs.n5);
    printf("  without the gyro-bias correction: +3 s within 3 mg %d of %d, +5 s %d of %d\n", rs0.ok3, rs0.n3, rs0.ok5,
           rs0.n5);
    CHECK(rs.ok3 >= rs0.ok3 - 1 && rs.ok5 >= rs0.ok5 - 1, "gyro-bias correction keeps the offset accuracy: +3 s %d vs %d, "
          "+5 s %d vs %d", rs.ok3, rs0.ok3, rs.ok5, rs0.ok5);
  }
  int nPre, nPost, nPre0, nPost0;
  const double pre = driftRate(&rs, 6, 15, &nPre), post = driftRate(&rs, 16, 21, &nPost);
  const double pre0 = driftRate(&rs0, 6, 15, &nPre0), post0 = driftRate(&rs0, 16, 21, &nPost0);
  printf("  touchdown height / camera time, mean: files 6-15 %+.4f m/s (uncorrected %+.4f, n=%d), files 16-21 %+.4f "
         "(uncorrected %+.4f, n=%d); accepted/rejected %u/%u (uncorrected %u/%u)\n", pre, pre0, nPre, post, post0, nPost,
         rs.acc, rs.rej, rs0.acc, rs0.rej);
  CHECK(nPre == nPre0 && nPost == nPost0 && nPre >= 8 && nPost >= 15, "touchdowns %d/%d vs %d/%d", nPre, nPost, nPre0,
        nPost0);
  CHECK(pre < pre0 - 0.005, "files 6-15 (in-flight gyro bias): drift %+.4f vs %+.4f m/s uncorrected", pre, pre0);
  CHECK(fabs(post - post0) < 0.003, "files 16-21 (no bias): drift %+.4f vs %+.4f m/s uncorrected", post, post0);
  CHECK(abs((int)rs.acc - (int)rs0.acc) <= (int)rs0.acc / 50 && abs((int)rs.rej - (int)rs0.rej) <= 5 + (int)rs0.rej / 10,
        "counters: accepted %u vs %u, rejected %u vs %u", rs.acc, rs0.acc, rs.rej, rs0.rej);

  /* the replayed filter's own rate correction, the in-flight bias learning off and on */
  int nF0a, nF0b, nF1a, nF1b;
  const double f0pre = driftRate(&rsF0, 6, 15, &nF0a), f0post = driftRate(&rsF0, 16, 21, &nF0b);
  const double f1pre = driftRate(&rsF1, 6, 15, &nF1a), f1post = driftRate(&rsF1, 16, 21, &nF1b);
  printf("  replayed attitude filter's rate correction, twoKiAir 0 -> 0.040: drift files 6-15 %+.4f -> %+.4f m/s, "
         "16-21 %+.4f -> %+.4f (the rebuilt one %+.4f / %+.4f); offset within 3 mg +3 s %d -> %d of %d, +5 s %d -> %d "
         "of %d; |err| median +5 s %.2f -> %.2f mg; accepted/rejected %u/%u -> %u/%u\n", f0pre, f1pre, f0post, f1post,
         pre, post, rsF0.ok3, rsF1.ok3, rsF1.n3, rsF0.ok5, rsF1.ok5, rsF1.n5, pct(rsF0.err5, rsF0.n5, 0.5),
         pct(rsF1.err5, rsF1.n5, 0.5), rsF0.acc, rsF0.rej, rsF1.acc, rsF1.rej);
  CHECK(nF0a == nF1a && nF0b == nF1b && nF1a == nPre && nF1b == nPost, "touchdowns %d/%d vs %d/%d", nF1a, nF1b, nF0a,
        nF0b);
  CHECK(fabs(f1pre) <= fabs(f0pre) + 0.003 && fabs(f1post) <= fabs(f0post) + 0.003, "the bias learning makes the "
        "drift worse: %+.4f vs %+.4f (6-15), %+.4f vs %+.4f (16-21) m/s", f1pre, f0pre, f1post, f0post);
  if (nTruth) {
    CHECK(rsF1.ok3 >= rsF0.ok3 - 1 && rsF1.ok5 >= rsF0.ok5 - 1, "the bias learning keeps the offset accuracy: +3 s "
          "%d vs %d, +5 s %d vs %d", rsF1.ok3, rsF0.ok3, rsF1.ok5, rsF0.ok5);
  }
  CHECK(abs((int)rsF1.acc - (int)rsF0.acc) <= (int)rsF0.acc / 50 &&
        abs((int)rsF1.rej - (int)rsF0.rej) <= 5 + (int)rsF0.rej / 10, "the bias learning's counters: accepted %u vs "
        "%u, rejected %u vs %u", rsF1.acc, rsF0.acc, rsF1.rej, rsF0.rej);
}

/* 2026-10-13 flight 1 (CSV mode): five in-air contacts from 34.45 s after
 * HOVER entry (tick 79020), motors off at 35.31 s. The touchdown values come
 * from before the first contact; before the cluster rule they came from the
 * third (on board: h +1.15 m, v +0.45 m/s, after two impulses integrated as a
 * climb). */
static void replayContact(const char *dir)
{
  char path[1024];
  snprintf(path, sizeof(path), "%s/flight-2026-10-13-interactive.csv", dir);
  if (!loadCsv(path)) {
    printf("contact replay skipped: no %s\n", path);
    return;
  }
  int i0 = -1, i1 = -1;
  for (int i = 0; i < nRows; i++) {
    if (rows[i][C_FID] == 1) {
      if (i0 < 0 && rows[i][C_THRUST] > 0) {
        i0 = i;
      }
      i1 = i + 1;
    }
  }
  CHECK(i0 >= 0 && i1 - i0 > 3000, "2026-10-13 flight 1 rows %d..%d", i0, i1);
  if (i0 < 0) {
    return;
  }
  replayStats_t rs;
  memset(&rs, 0, sizeof(rs));
  rebuildRateCorr(i0, i1);
  replayAtTick = 79020;   /* the first contact's row */
  replayTakeoff(0, i0, i1, &rs, true);
  replayAtTick = -1;
  const fsumCamVz_t *f = &replayLast;
  printf("2026-10-13 flight 1 (in-air contacts from tick 79020, 34.45 s from HOVER entry): touchdown h %+.3f m v "
         "%+.3f m/s, impact %d; the shadow before the first contact: h %+.3f m v %+.3f m/s\n", f->hTdMm / 1000.0,
         f->vTdMms / 1000.0, !!(f->flags & FSUM_CVZ_IMPACT), replayAtH, replayAtV);
  CHECK((f->flags & FSUM_CVZ_TD) && (f->flags & FSUM_CVZ_IMPACT) &&
        abs(f->hTdMm - (int)lrintf(replayAtH * 1000.0f)) <= 30 && abs(f->vTdMms - (int)lrintf(replayAtV * 1000.0f)) <= 30 &&
        abs(f->vTdMms) <= 100, "flight 1: touchdown values from before the first contact (h %d vs %.0f mm, v %d vs "
        "%.0f mm/s), |v| <= 100 mm/s", f->hTdMm, replayAtH * 1e3, f->vTdMms, replayAtV * 1e3);
}

int main(int argc, char **argv)
{
  const char *dir = NULL, *truthPath = NULL;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-v") == 0) {
      verbose = 1;
    } else if (strncmp(argv[i], "-g", 2) == 0) {
      glueP = argv[i][2] == '=' ? atof(argv[i] + 3) : 999.9;
    } else if (dir == NULL) {
      dir = argv[i];
    } else {
      truthPath = argv[i];
    }
  }
  printf("sizeof(camVzShadow_t) = %u bytes, sizeof(fsumCamVz_t) = %u, sizeof(flightSummary_t) = %u\n",
         (unsigned)sizeof(camVzShadow_t), (unsigned)sizeof(fsumCamVz_t), (unsigned)sizeof(flightSummary_t));
  CHECK(sizeof(fsumCamVz_t) == 34, "fsumCamVz_t is 34 bytes (RTC budget), got %u", (unsigned)sizeof(fsumCamVz_t));
  testNominal();
  testImpactCluster();
  testCameraOffset();
  testGyroBias();
  testAccLiftoff();
  testAborted();
  testHopAndImplicit();
  testNoCameraAndFault();
  testG0Median();
  testGlueBlockRate();
  testBenchAir();
  if (dir) {
    if (truthPath) {
      loadTruth(truthPath);
    }
    replay(dir);
    replayContact(dir);
  } else {
    printf("replay skipped: pass the directory with flight*.csv (e.g. ../../firmware-backups/cal) [and bias_truth.csv]\n");
  }
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
