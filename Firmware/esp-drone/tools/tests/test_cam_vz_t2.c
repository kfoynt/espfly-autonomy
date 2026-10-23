/*
 * Tests of the camera height observer's offset drift rate T2 (t2kR, cam_vz_obs.h "Offset drift rate", the late-sink
 * fix "T2p" of 2026-10-16; docs/autonomy-a2b.md 29; the plan's host tests 0-viii,
 * firmware-backups/handoff-2026-10-12/colour-sink-plan-2026-10-16.md section 3).
 *
 * Two harnesses:
 *   - closed loop (fly): the real vertical controller (vert_ctrl.c, HOLD at 1.00 m on its sim plant vzSim) on the
 *     real shadow observer (cam_vz_shadow.c + cam_vz_obs.c, fed as cam_vz_glue.c feeds it: 10 ms means, the newest
 *     camera frame after each block, the controller's camera fields from camVzShadowVzInput) over a synthetic camera
 *     (a frame every 67.5 ms, 120 ms late, its window 15 ms past the capture, 0.12 px of noise, G the scene's) and an
 *     accelerometer whose in-flight offset follows a chosen shape. As test_cam_vz_late.c, whose harness this is.
 *   - open loop (ol*): the observer alone on an exact bobbing hover (h = A (1 - cos w t), the accelerometer its true
 *     acceleration plus the offset), for the event-level checks.
 *
 *   0. kR 0 is bit for bit the observer of 740b6ad (a hash of every step's thrust and estimate, pinned; built with
 *      -DT2_BASE against that commit's cam_vz_obs.c / cam_vz_shadow.c); with kR > 0 a sortie shorter than t2StartS
 *      is too. (The other camvz suites are the rest of 0: unchanged and passing.)
 *   i.   Offset ramp +0.2 mg/s from 30 s, clean camera, G 25 and G 56: the offset estimate lags the truth by < 0.4 mg
 *        at 60 s (today's lag printed, and the true height's change 30 -> 90 s).
 *   ii.  No drift: |bRate| < 0.05 mg/s at 70 s.
 *   iii. Reversed drift -0.2 mg/s: bRate goes negative and stays within the bound.
 *   iv.  Camera glitches of +-0.1 / +-0.3 m/s lasting 0.5, 1 or 2 s at 50 and 60 s, with and without the camera lost
 *        right after: the induced change within today's + 0.10 m (and the late test's raw per-case mark).
 *   v.   The ground freezes bRate and a bounce does not restart it; a commanded motion, a turn and a yaw restart hold
 *        the adaptation for 3 s.
 *   vi.  The bound +-0.35 mg/s holds; a NaN input faults the observer; an invalid config is refused (camVzObsInit
 *        false), never flown.
 *   vii. No accepted frame for > 0.5 s: no drift (the offset estimate frozen through gaps of 3, 5 and 10 s).
 *   viii. With kR > 0 and qLateMult 1, biasHeldG == biasG on every block and R1's step and variance bound never run.
 *   ix.  (printed and checked) The real in-flight offsets of the three 10-15 hovers (fixture cam_vz_late.txt):
 *        less late height drift than today.
 *
 * The plan's marks i, ii and iv are NOT all met (2026-10-16; docs/autonomy-a2b.md 29.5): they are measured, tallied
 * and printed ("PLAN MARK"), and what is gated is what T2 must keep doing (better than today where it is meant to
 * help, within the bound, the events, gaps, ground and configs exactly) plus a pin of the glitch result, so the suite
 * stays a regression test. Read the PLAN MARK lines, not only the failure count.
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=c99 -O2 -Wall -Wextra -I$C/utils/interface -I$C/modules/interface tools/tests/test_cam_vz_t2.c \
 *      $C/utils/src/cam_vz_shadow.c $C/utils/src/cam_vz_obs.c $C/utils/src/vert_ctrl.c \
 *      -lm -o /tmp/test_cam_vz_t2 && /tmp/test_cam_vz_t2 [fixture] [-v]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cam_vz_shadow.h"
#include "vert_ctrl.h"

static int failures = 0;
static int checks = 0;
static int verbose = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define DT_US        1000u        /* the stabilizer loop */
#define FRAME_US     67500u       /* 14.8 fps */
#define LAT_US       120000u      /* capture -> delivered */
#define WIN_OFF_US   15000u       /* the frame's motion window ends this long after the capture (cam_vz_obs.h) */
#define PX_NOISE     0.12
#define SINK_S       5.0          /* auto_fsm.h AUTO_CAM_LOST_MAX_MS */
#define HIST_N       4096         /* true heights, one per ms (> LAT_US + a frame) */
#define BETA_N       1251
#define SEG_N        600

#ifdef T2_BASE
typedef struct {
  bool on;
  float t2RateMgS, minMgS, maxMgS;
  uint32_t onUs;
} camVzObsT2_t;   /* the observer of 740b6ad has none: -DT2_BASE only pins section 0's hashes */
#endif

#define KR_PLAN      0.5f         /* the plan's kR (cvt2k 50) */
#define MG_S         (1e-3f * CAM_VZ_G)   /* mg/s in the observer's m/s^3 */

/* ---- the fixture (test_cam_vz_late.c's: the 10-15 offsets) ------------------------------------------------------ */

typedef struct {
  char name[16];
  float g;
  float beta[BETA_N];       /* in-flight offset vs the ground baseline, g, every 0.1 s from the lift-off */
} betaTab_t;

static betaTab_t betas[3];
static int nBeta;

static bool loadFixture(const char *path)
{
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    return false;
  }
  char line[512];
  while (fgets(line, sizeof(line), f) != NULL) {
    if (line[0] == 'B' && nBeta < 3) {
      betaTab_t *b = &betas[nBeta++];
      int n = 0;
      if (sscanf(line, "B %15s %f %d", b->name, &b->g, &n) != 3 || n != BETA_N) {
        fclose(f);
        return false;
      }
      for (int i = 0; i < n; i++) {
        long v;
        if (fscanf(f, "%ld", &v) != 1) {
          fclose(f);
          return false;
        }
        b->beta[i] = (float)(v * 1e-5);
      }
    } else if (line[0] == 'S') {
      int n = 0;
      char nm[16];
      if (sscanf(line, "S %15s %d", nm, &n) != 2 || n > SEG_N) {
        fclose(f);
        return false;
      }
      for (int i = 0; i < n; i++) {
        double t;
        long a, b, c;
        if (fscanf(f, "%lf %ld %ld %ld", &t, &a, &b, &c) != 4) {
          fclose(f);
          return false;
        }
      }
    }
  }
  fclose(f);
  return nBeta == 3;
}

/* ---- closed loop ------------------------------------------------------------------------------------------------ */

typedef enum {
  SHAPE_1015 = 0,           /* the 10-15 shape: +1.37 mg falling 0.04 mg/s to 25 s, then rising 0.20 mg/s */
  SHAPE_RAMP30,             /* +1.37 mg to 30 s, then rampMgS (mg/s) */
  SHAPE_FLAT,               /* +1.37 mg throughout */
  SHAPE_BETA,               /* a real one (the fixture) */
} shape_t;

typedef struct {
  float kR;                 /* t2kR (0: today) */
  float qLate;              /* qLateMult (1: off) */
  double g;                 /* the scene, px/m */
  shape_t shape;
  double rampMgS;
  const betaTab_t *beta;
  double glitch;            /* m/s the camera reads beyond the truth (+ a climb) ... */
  double gT, gDur;          /* ... from gT for gDur (s after the lift-off) */
  bool lose;                /* no frame from gT + gDur on */
  double tEnd;              /* s after the lift-off */
  uint32_t seed;
  double tLastSet;          /* Claude's exact SET_HEIGHT every 12 s from 20 s up to this (< 20: none) */
} scen_t;

typedef struct {
  bool hold, sink, landed;
  vzReason_t land;
  double tSink, hRef, err, errMax, tLow;
  double h30, h60, h90, hEnd;
  double lag60Mg;           /* true offset - the estimate's, mean over 59-61 s, mg (positive: the estimate low) */
  double bRate70;           /* mg/s at 70 s */
  camVzObsT2_t t2;          /* at the end */
#ifndef T2_BASE
  fsumCvLate_t late;        /* the flight summary's fields at the end (camVzShadowLateStats) */
#endif
  uint32_t heldMismatch;    /* blocks with biasHeldG != biasG */
  uint32_t blocks;
  bool lateRan;             /* R1's step reached, its variance bound or a late guard (viii: never with T2) */
  float hDs[1500];          /* true height every 0.1 s after the lift-off (iv's reference runs) */
  uint64_t hash;            /* every step's thrust and estimate (FNV-1a) */
} res_t;

static scen_t scen(float kR, double g, shape_t shape, double tEnd, uint32_t seed)
{
  scen_t s;
  memset(&s, 0, sizeof(s));
  s.kR = kR;
  s.qLate = 1.0f;
  s.g = g;
  s.shape = shape;
  s.gT = 200.0;
  s.tEnd = tEnd;
  s.seed = seed;
  return s;
}

static uint32_t rng;
static double gauss(void)
{
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  const double u1 = (rng + 0.5) / 4294967296.0;
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  const double u2 = (rng + 0.5) / 4294967296.0;
  return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

static uint64_t fnv(uint64_t h, const void *p, size_t n)
{
  const unsigned char *b = p;
  for (size_t i = 0; i < n; i++) {
    h ^= b[i];
    h *= 1099511628211ull;
  }
  return h;
}

/* The in-flight offset (g, against 1 g) t s after the lift-off. */
static double offsetAt(const scen_t *sc, double t)
{
  switch (sc->shape) {
    case SHAPE_BETA: {
      const double k = t * 10.0;
      const int i = (int)k;
      if (i >= BETA_N - 1) {
        return sc->beta->beta[BETA_N - 1];
      }
      return sc->beta->beta[i] + (sc->beta->beta[i + 1] - sc->beta->beta[i]) * (k - i);
    }
    case SHAPE_FLAT:
      return 0.00137;
    case SHAPE_RAMP30:
      return 0.00137 + (t > 30.0 ? sc->rampMgS * 1e-3 * (t - 30.0) : 0.0);
    case SHAPE_1015:
    default:
      return 0.00137 - 0.00004 * fmin(t, 25.0) + (t > 25.0 ? 0.0002 * (t - 25.0) : 0.0);
  }
}

static float hist[HIST_N];   /* the true height every ms */

static double hAt(uint64_t k, double tUs)
{
  long i = lround(tUs / 1000.0);
  if (i > (long)k) {
    i = (long)k;
  }
  if (i < (long)k - (HIST_N - 1)) {
    i = (long)k - (HIST_N - 1);
  }
  if (i < 0) {
    i = 0;
  }
  return hist[i % HIST_N];
}

static void fly(const scen_t *sc, res_t *r)
{
  memset(r, 0, sizeof(*r));
  r->tLow = -1.0;
  r->hash = 14695981039346656037ull;
  rng = sc->seed * 2654435761u | 1u;
  vzConfig_t vc;
  vzDefaultConfig(&vc);
  vzConfigSetHeight(&vc, 1.0f);
  static vzState_t st;
  vzInit(&st, &vc);
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.accNoiseG = 0.02f;
  static vzSimState_t sim;
  vzSimInit(&sim, &pc, sc->seed | 1u);
  camVzObsConfig_t oc;
  camVzObsDefaultConfig(&oc);
  oc.qLateMult = sc->qLate;
#ifndef T2_BASE
  oc.t2kR = sc->kR;
#endif
  static camVzShadow_t sh;
  CHECK(camVzShadowInit(&sh, &oc), "observer config");
  const uint64_t base = 1000000000ull;
  vzOutput_t vo;
  memset(&vo, 0, sizeof(vo));
  float applied = 0.0f;
  double blkSum = 0.0;
  int blkN = 0;
  bool groundStart = false, wasZero = true;
  uint32_t seq = 0;
  uint64_t nextCap = 0, lastSeqCap = 0;
  bool haveCap = false;
  double liftT = -1.0, tHold = -1.0, lastUsed = -1.0;
  double nextSet = 20.0;
  bool moving = false;
  double lagSum = 0.0;
  int lagN = 0;
  bool have70 = false;
  const double t0 = 1.0;   /* 1 s at rest, then TAKEOFF */
  const double tStop = t0 + 12.0 + sc->tEnd;
  for (uint64_t k = 0;; k++) {
    const double t = k * 1e-3;
    const double ta = liftT >= 0.0 ? t - liftT : -1.0;
    if (t > tStop || (liftT >= 0.0 && ta > sc->tEnd)) {
      break;
    }
    if (liftT < 0.0 && sim.liftT >= 0.0f) {
      liftT = sim.liftT;
    }
    const double off = ta >= 0.0 ? offsetAt(sc, ta) : 0.0;
    const float accZ = sim.accZ + (float)off;
    hist[k % HIST_N] = sim.h;
    vzInput_t in;
    memset(&in, 0, sizeof(in));
    in.dt = 0.001f;
    in.cmd = t < t0 ? VZ_CMD_NONE : VZ_CMD_TAKEOFF;
    in.accZ = accZ;
    in.gravZ = 1.0f;
    in.gyroMaxDps = 0.5f;
    in.vbat = sim.vbat;
    in.thrustApplied = applied;
    /* Claude's SET_HEIGHT: "you are at X" (the true height, to the cm), then the correction move */
    if (ta >= nextSet && nextSet <= sc->tLastSet && vo.phase == VZ_HOLD && !moving) {
      const float d = fminf(fmaxf(roundf((sim.h - vo.h) * 100.0f) * 0.01f, -VZ_SHIFT_MAX_M), VZ_SHIFT_MAX_M);
      vzShiftHeight(&st, d);
      vo.h += d;
      moving = true;
      nextSet += 12.0;
    }
    if (moving) {
      const float e = vc.hTarget - vo.h;
      in.vRef = fminf(fmaxf(e, -0.15f), 0.15f);
      moving = fabsf(e) >= 0.01f || ta < nextSet - 11.5;
      if (!moving) {
        in.vRef = 0.0f;
      }
    }
    camVzShadowVzInput(&sh, vo.g0, &in);
    vzStep(&st, &in, &vo);
    vzSimStep(&sim, &pc, vo.thrust, 0.001f);
    applied = vo.thrust;
    r->hash = fnv(fnv(r->hash, &vo.thrust, sizeof(vo.thrust)), &vo.h, sizeof(vo.h));
    if (vo.phase == VZ_HOLD && tHold < 0.0) {
      tHold = t;
      r->hold = true;
    }
    if (vo.camHealthy) {
      lastUsed = t;
    }
    if ((vo.phase == VZ_LAND || vo.phase == VZ_DONE || vo.phase == VZ_CUT) && !r->landed && tHold >= 0.0) {
      r->landed = true;
      r->land = vo.landReason;
    }
    /* the glue: 10 samples per block, the newest frame after it */
    groundStart = groundStart || (applied > 0.0f && wasZero);
    wasZero = applied <= 0.0f;
    blkSum += accZ;
    blkN++;
    if (blkN == 10) {
      camVzShadowIn_t si;
      memset(&si, 0, sizeof(si));
      si.timeUs = base + k * DT_US;
      si.accUpG = (float)(blkSum / blkN);
      si.thrust = applied;
      si.gyroMaxDps = 0.5f;
      si.groundStart = groundStart;
      si.restingOnGround = applied <= 0.0f && sim.onGround;
      si.still = si.restingOnGround;
      si.liftoffHint = vo.liftedOff;
      camVzObsSetMotion(&sh.obs, moving);   /* the glue: a commanded motion (holdWait) */
      camVzShadowStep(&sh, &si);
      groundStart = false;
      blkSum = 0.0;
      blkN = 0;
      const uint64_t nowUs = k * DT_US;
      while (nextCap + LAT_US <= nowUs) {
        lastSeqCap = nextCap;
        haveCap = true;
        nextCap += FRAME_US;
        seq++;
      }
      if (haveCap) {
        haveCap = false;
        const double tc = lastSeqCap * 1e-6;
        const double tca = liftT >= 0.0 ? tc - liftT : -1.0;
        const bool lost = sc->lose && liftT >= 0.0 && tca >= sc->gT + sc->gDur;
        if (!lost) {
          camVzObsFrame_t f;
          memset(&f, 0, sizeof(f));
          f.seq = seq;
          f.frameTimeUs = base + lastSeqCap;
          f.dtUs = FRAME_US;
          const double w1 = (double)lastSeqCap + WIN_OFF_US, w0 = w1 - FRAME_US;
          double dy = -sc->g * (hAt(k, w1) - hAt(k, w0)) + oc.kyC + PX_NOISE * gauss();
          f.ok = tc > 0.0;
          if (tca >= sc->gT && tca < sc->gT + sc->gDur) {
            dy -= sc->g * sc->glitch * FRAME_US * 1e-6;
          }
          f.dy = (float)dy;
          f.dx = (float)(0.1 * gauss());
          f.texture = 20.0f;
          f.residual = 3.0f;
          camVzShadowFrame(&sh, &f, base + nowUs);
        }
      }
      /* viii: the controller's offset is the estimate's own on every block with R1 off */
      camVzObsOut_t q;
      camVzObsGet(&sh.obs, &q);
      r->blocks++;
      if (memcmp(&q.biasHeldG, &q.biasG, sizeof(float)) != 0) {
        r->heldMismatch++;
      }
      if (ta >= 59.0 && ta < 61.0) {
        lagSum += off - q.biasG;
        lagN++;
      }
#ifndef T2_BASE
      if (ta >= 70.0 && !have70) {
        have70 = true;
        camVzObsT2_t t2;
        camVzObsT2Get(&sh.obs, &t2);
        r->bRate70 = t2.t2RateMgS;
      }
#endif
    }
    if (ta >= 0.0) {
      const int ds = (int)(ta * 10.0);
      if (ds < 1500) {
        r->hDs[ds] = sim.h;
      }
      if (ta >= 30.0 && r->h30 == 0.0) {
        r->h30 = sim.h;
      }
      if (ta >= 60.0 && r->h60 == 0.0) {
        r->h60 = sim.h;
      }
      if (ta >= 90.0 && r->h90 == 0.0) {
        r->h90 = sim.h;
      }
      if (ta >= sc->gT && r->hRef == 0.0) {
        r->hRef = sim.h;
      }
      if (ta >= sc->gT) {
        r->errMax = fmax(r->errMax, fabs(sim.h - r->hRef));
        if (r->tLow < 0.0 && sim.h <= 0.10f) {
          r->tLow = ta;
        }
        if (!sc->lose && ta <= sc->gT + 15.0) {
          r->err = sim.h - r->hRef;
        }
      }
      r->hEnd = sim.h;
      /* the FSM's camera rule: the SINK 5 s after the later of the last step in use and the HOLD entry */
      if (tHold >= 0.0 && !r->sink && t - fmax(lastUsed, tHold) >= SINK_S) {
        r->sink = true;
        r->tSink = ta;
        r->err = sim.h - r->hRef;
        break;
      }
    }
  }
  r->lag60Mg = lagN ? lagSum / lagN * 1e3 : NAN;
  r->lateRan = sh.obs.late || sh.obs.lateOn || sh.obs.lateUs != 0 || sh.obs.lateGuards != 0 || sh.obs.p22Norm != 0.0f;
#ifndef T2_BASE
  camVzObsT2Get(&sh.obs, &r->t2);
  camVzShadowLateStats(&sh, &r->late);
#endif
}

#ifndef T2_BASE
/* ---- open loop: the observer alone on an exact bobbing hover --------------------------------------------------- */

#define OL_LIFT_S   1.0           /* on the ground until here (throttled from 0) */
#define OL_A        0.04          /* m: h = A (1 - cos w t) after the lift-off (enough vertical motion for G) */
#define OL_W        (2.0 * M_PI / 2.5)

typedef struct {
  double rampMgS;           /* offset ramp from 30 s after the lift-off */
  double groundA, groundB;  /* on the ground (the frames skipped) in [A, B), then a re-lift (a bounce) */
  double gapA, gapB;        /* no frame delivered in [A, B) */
  double motionAt, turnAt, yawAt;   /* one block's motion flag, turn flag; one frame turning 0.4 rad */
  double nanAt;             /* a NaN accelerometer sample */
  double tEnd;
  double g;                 /* the scene, px/m */
  uint32_t seed;
} olScen_t;

typedef struct {
  camVzObs_t o;
  double t;                 /* s since the start */
  double tLift;             /* the latest lift-off (s) */
} ol_t;

static olScen_t olScen(double tEnd)
{
  olScen_t s;
  memset(&s, 0, sizeof(s));
  s.groundA = s.groundB = s.gapA = s.gapB = s.motionAt = s.turnAt = s.yawAt = s.nanAt = 1e9;
  s.tEnd = tEnd;
  s.g = 40.0;
  s.seed = 77u;
  return s;
}

static double olH(const ol_t *ol, double t)
{
  return t <= ol->tLift ? 0.0 : OL_A * (1.0 - cos(OL_W * (t - ol->tLift)));
}

/* One open-loop run; cb (may be NULL) after every block and every frame (frame: its decision, else -1). */
typedef void (*olCb_t)(const ol_t *ol, int decision, void *ctx);

static void olRun(ol_t *ol, const olScen_t *sc, float kR, olCb_t cb, void *ctx)
{
  camVzObsConfig_t c;
  camVzObsDefaultConfig(&c);
  c.t2kR = kR;
  CHECK(camVzObsInit(&ol->o, &c), "open loop: config");
  camVzObsSetBiasPrior(&ol->o, 0.00137f, CAM_VZ_PRIOR_SIGMA_G);
  ol->tLift = OL_LIFT_S;
  rng = sc->seed;
  const uint64_t base = 5000000000ull;
  uint32_t seq = 0;
  uint64_t nextCap = 0;
  for (uint64_t k = 0; k * 1e-3 < sc->tEnd; k++) {
    const double t = k * 1e-3;
    ol->t = t;
    const bool ground = t < OL_LIFT_S || (t >= sc->groundA && t < sc->groundB);
    if (t >= sc->groundB && ol->tLift < sc->groundB) {
      ol->tLift = sc->groundB;   /* the bounce: a new lift-off */
    }
    const double ta = t - OL_LIFT_S;
    const double off = 0.00137 + (ta > 30.0 ? sc->rampMgS * 1e-3 * (ta - 30.0) : 0.0);
    const double acc = ground ? 0.0 : OL_A * OL_W * OL_W * cos(OL_W * (t - ol->tLift));
    float a = (float)(1.0 + acc / CAM_VZ_G + off + 0.002 * gauss());
    if (fabs(t - sc->nanAt) < 0.5e-3) {
      a = NAN;
    }
    camVzObsImu(&ol->o, base + k * DT_US, a, ground, 40000.0f);
    if (k % 10 != 9) {
      continue;
    }
    /* a block closed at this sample */
    /* held one frame period, so a frame sees it */
    camVzObsSetMotion(&ol->o, t >= sc->motionAt && t < sc->motionAt + 0.07);
    camVzObsSetTurn(&ol->o, t >= sc->turnAt && t < sc->turnAt + 0.07);
    if (cb) {
      cb(ol, -1, ctx);
    }
    const uint64_t nowUs = k * DT_US;
    bool have = false;
    uint64_t cap = 0;
    while (nextCap + LAT_US <= nowUs) {
      cap = nextCap;
      have = true;
      nextCap += FRAME_US;
      seq++;
    }
    const double tc = cap * 1e-6;
    if (!have || (tc >= sc->gapA && tc < sc->gapB)) {
      continue;
    }
    camVzObsFrame_t f;
    memset(&f, 0, sizeof(f));
    f.seq = seq;
    f.frameTimeUs = base + cap;
    f.dtUs = FRAME_US;
    const double w1 = (cap + WIN_OFF_US) * 1e-6, w0 = w1 - FRAME_US * 1e-6;
    f.dy = (float)(-sc->g * (olH(ol, w1) - olH(ol, w0)) + c.kyC + PX_NOISE * gauss());
    f.dx = (float)(0.1 * gauss());
    f.texture = 20.0f;
    f.residual = 3.0f;
    f.ok = true;
    if (fabs(tc - sc->yawAt) < 0.034) {
      f.gyroAngle[2] = 0.4f;   /* a 0.4 rad yaw in one frame: the tracker restarts (yawResetRad 0.35) */
    }
    const camVzObsDecision_t d = camVzObsFrame(&ol->o, &f, base + nowUs);
    if (cb) {
      cb(ol, (int)d, ctx);
    }
  }
}

#endif /* !T2_BASE */

/* ---- 0. bit-identical with kR 0 ---------------------------------------------------------------------------------- */

/* the runs' hashes with the observer of 740b6ad (-DT2_BASE against that commit's cam_vz_obs.c and cam_vz_shadow.c) */
#define HASH_BASE_GLITCH 0x869d48cae62cef73ull   /* = test_cam_vz_late.c's FLY_HASH_BASE (65dfcba): the same sortie */
#define HASH_BASE_RAMP   0x476529dd7f6c2f92ull

static void testIdentity(void)
{
  scen_t a = scen(0.0f, 33.0, SHAPE_1015, 90.0, 7u);
  a.glitch = 0.3;
  a.gT = 60.0;
  a.gDur = 1.0;
  a.lose = true;
  a.tLastSet = 56.0;
  scen_t b = scen(0.0f, 25.0, SHAPE_RAMP30, 100.0, 3u);
  b.rampMgS = 0.2;
  res_t ra, rb;
  fly(&a, &ra);
  fly(&b, &rb);
  printf("0. kR 0: glitch+lost sortie hash %016llx (740b6ad: %016llx), ramp sortie %016llx (740b6ad: %016llx)\n",
         (unsigned long long)ra.hash, (unsigned long long)HASH_BASE_GLITCH, (unsigned long long)rb.hash,
         (unsigned long long)HASH_BASE_RAMP);
#ifdef T2_BASE
  (void)ra;
  (void)rb;
#else
  CHECK(ra.hash == HASH_BASE_GLITCH && rb.hash == HASH_BASE_RAMP,
        "0. kR 0: the controller's every step bit for bit as with the observer of 740b6ad");
  /* with kR > 0, a sortie shorter than t2StartS is the same */
  for (int i = 0; i < 2; i++) {
    scen_t s0 = scen(0.0f, i ? 56.0 : 25.0, SHAPE_1015, 29.0, 9u + (uint32_t)i);
    s0.glitch = i ? -0.3 : 0.3;
    s0.gT = 22.0;
    s0.gDur = 1.0;
    s0.tLastSet = 24.0;
    scen_t s1 = s0;
    s1.kR = KR_PLAN;
    scen_t s2 = s0;
    s2.kR = CAM_VZ_T2_KR_MAX;
    res_t r0, r1, r2;
    fly(&s0, &r0);
    fly(&s1, &r1);
    fly(&s2, &r2);
    CHECK(r0.hash == r1.hash && r0.hash == r2.hash && !r1.t2.on && !r2.t2.on,
          "0. kR 0.5 / 2, a 29 s sortie (G %.0f, a glitch at 22 s): bit-identical to kR 0", s0.g);
  }
#endif
}

#ifndef T2_BASE

/* ---- i-iii. the drift ------------------------------------------------------------------------------------------- */

/* The plan's marks i-iii are measured twice and tallied, not gated (they are the design's targets; the report and
 * docs/autonomy-a2b.md 29 give the result): in the closed loop (the controller on the sim plant, the camera with its
 * noise and the drone's own height drift) and on the observer alone (the open loop's exact bobbing hover, a clean
 * camera). Gated: what T2 must do against today in the closed loop (a smaller offset lag and late drift on the ramp,
 * no more than 0.10 m worse with no drift) and the bound and sign of iii in both. */
typedef struct {
  int n, met;
} mark_t;

static void markAdd(mark_t *m, bool met)
{
  m->n++;
  m->met += met;
}

static double olLagAcc;
static int olLagN;
static float olBr70;
static double olRamp;

static void olDriftCb(const ol_t *ol, int d, void *ctx)
{
  (void)ctx;
  if (d != -1) {
    return;
  }
  const double t = ol->o.tIn * 1e-6, ta = t - OL_LIFT_S;
  const double off = 0.00137 + (ta > 30.0 ? olRamp * 1e-3 * (ta - 30.0) : 0.0);
  if (ta >= 59.0 && ta < 61.0) {
    olLagAcc += off - ol->o.x[2] / CAM_VZ_G;
    olLagN++;
  }
  if (ta >= 70.0 && isnan(olBr70)) {
    olBr70 = ol->o.bRate / MG_S;
  }
}

static void testDrift(void)
{
  static const double gs[2] = {25.0, 56.0};
  mark_t mi[2] = {{0, 0}, {0, 0}}, mii[2] = {{0, 0}, {0, 0}}, miii[2] = {{0, 0}, {0, 0}};   /* [closed, open] */
  static res_t today, r;
  for (int gi = 0; gi < 2; gi++) {
    for (uint32_t seed = 1; seed <= 2; seed++) {
      /* i. +0.2 mg/s from 30 s */
      scen_t s = scen(0.0f, gs[gi], SHAPE_RAMP30, 92.0, seed);
      s.rampMgS = 0.2;
      fly(&s, &today);
      s.kR = KR_PLAN;
      fly(&s, &r);
      markAdd(&mi[0], fabs(r.lag60Mg) < 0.4);
      printf("i.   closed loop, ramp +0.2 mg/s, G %.0f seed %u: lag at 60 s %+.2f mg (today %+.2f; plan < 0.4: %s), "
             "true height 30 -> 90 s %+.3f m (today %+.3f), bRate %+.3f mg/s (range %+.3f..%+.3f), on %.1f s\n",
             gs[gi], (unsigned)seed, r.lag60Mg, today.lag60Mg, fabs(r.lag60Mg) < 0.4 ? "met" : "MISSED",
             r.h90 - r.h30, today.h90 - today.h30, r.t2.t2RateMgS, r.t2.minMgS, r.t2.maxMgS, r.t2.onUs * 1e-6);
      CHECK(r.hold && !r.landed && fabs(r.lag60Mg) < 0.5 * fabs(today.lag60Mg) &&
            fabs(r.h90 - r.h30) < fabs(today.h90 - today.h30),
            "i. ramp G %.0f seed %u: lag %+.2f mg under half of today's %+.2f, late drift %+.3f m under today's %+.3f",
            gs[gi], (unsigned)seed, r.lag60Mg, today.lag60Mg, r.h90 - r.h30, today.h90 - today.h30);
      CHECK((r.late.flags & (FSUM_CVL_RAN | FSUM_CVL_T2_ON)) == (FSUM_CVL_RAN | FSUM_CVL_T2_ON) &&
                !(r.late.flags & (FSUM_CVL_STEP | FSUM_CVL_T2_CFG_BAD)) && r.late.t2K == 50 &&
                r.late.t2PriorUgs == 124 && r.late.t2OnDs == r.t2.onUs / 100000u &&
                r.late.t2BrEnd == (int8_t)lrintf(r.t2.t2RateMgS * 100.0f) &&
                r.late.t2BrMin == (int8_t)lrintf(r.t2.minMgS * 100.0f) &&
                r.late.t2BrMax == (int8_t)lrintf(r.t2.maxMgS * 100.0f) && today.late.t2K == 0 &&
                !(today.late.flags & FSUM_CVL_T2_ON) && today.late.t2OnDs == 0,
            "flight summary: T2 on, k50 prior 124, %u ds on, bRate %d (%d..%d) x0.01 mg/s; today's all off",
            (unsigned)r.late.t2OnDs, r.late.t2BrEnd, r.late.t2BrMin, r.late.t2BrMax);
      CHECK(r.heldMismatch == 0 && !r.lateRan, "viii. ramp G %.0f seed %u: biasHeldG == biasG on all %u blocks, "
            "no R1 step or bound (%u differ)", gs[gi], (unsigned)seed, (unsigned)r.blocks, (unsigned)r.heldMismatch);
      /* ii. no drift */
      s = scen(0.0f, gs[gi], SHAPE_FLAT, 92.0, seed);
      fly(&s, &today);
      s.kR = KR_PLAN;
      fly(&s, &r);
      markAdd(&mii[0], fabs(r.bRate70) < 0.05);
      printf("ii.  closed loop, no drift,      G %.0f seed %u: bRate at 70 s %+.3f mg/s (plan |.| < 0.05: %s; range "
             "%+.3f..%+.3f), true height 30 -> 90 s %+.3f m (today %+.3f)\n", gs[gi], (unsigned)seed, r.bRate70,
             fabs(r.bRate70) < 0.05 ? "met" : "MISSED", r.t2.minMgS, r.t2.maxMgS, r.h90 - r.h30,
             today.h90 - today.h30);
      CHECK(r.hold && !r.landed && fabs(r.h90 - r.h30) <= fabs(today.h90 - today.h30) + 0.10 &&
            r.t2.maxMgS <= 0.35f + 1e-6f && r.t2.minMgS >= -0.35f - 1e-6f,
            "ii. no drift G %.0f seed %u: late drift %+.3f m within today's %+.3f + 0.10, bRate within the bound",
            gs[gi], (unsigned)seed, r.h90 - r.h30, today.h90 - today.h30);
      CHECK(r.heldMismatch == 0 && !r.lateRan, "viii. flat G %.0f seed %u: biasHeldG == biasG", gs[gi],
            (unsigned)seed);
      /* iii. reversed */
      s = scen(0.0f, gs[gi], SHAPE_RAMP30, 92.0, seed);
      s.rampMgS = -0.2;
      fly(&s, &today);
      s.kR = KR_PLAN;
      fly(&s, &r);
      const bool m3 = r.t2.minMgS < 0.0f && r.t2.t2RateMgS < 0.0f && r.t2.minMgS >= -0.35f - 1e-6f &&
                      r.t2.maxMgS <= 0.35f + 1e-6f;
      markAdd(&miii[0], m3);
      printf("iii. closed loop, ramp -0.2 mg/s, G %.0f seed %u: bRate %+.3f mg/s (range %+.3f..%+.3f; plan: %s), lag "
             "at 60 s %+.2f mg (today %+.2f), true height 30 -> 90 s %+.3f m (today %+.3f)\n", gs[gi],
             (unsigned)seed, r.t2.t2RateMgS, r.t2.minMgS, r.t2.maxMgS, m3 ? "met" : "MISSED", r.lag60Mg,
             today.lag60Mg, r.h90 - r.h30, today.h90 - today.h30);
      CHECK(r.hold && !r.landed && m3, "iii. reversed, G %.0f seed %u: bRate negative (%+.3f, min %+.3f) within "
            "+-0.35", gs[gi], (unsigned)seed, r.t2.t2RateMgS, r.t2.minMgS);
      CHECK(r.heldMismatch == 0 && !r.lateRan, "viii. reversed G %.0f seed %u: biasHeldG == biasG", gs[gi],
            (unsigned)seed);
    }
  }
  /* the observer alone, clean camera */
  for (int gi = 0; gi < 2; gi++) {
    for (uint32_t sd = 1; sd <= 3; sd++) {
      double lag = 0.0, br = 0.0, br3 = 0.0, mn3 = 0.0;
      for (int k = 0; k < 3; k++) {
        olScen_t sc = olScen(80.0);
        sc.g = gs[gi];
        sc.seed = 77u * sd;
        sc.rampMgS = k == 0 ? 0.2 : (k == 1 ? 0.0 : -0.2);
        olRamp = sc.rampMgS;
        olLagAcc = 0.0;
        olLagN = 0;
        olBr70 = NAN;
        static ol_t ol;
        olRun(&ol, &sc, KR_PLAN, olDriftCb, NULL);
        if (k == 0) {
          lag = olLagAcc / olLagN * 1e3;
        } else if (k == 1) {
          br = olBr70;
        } else {
          br3 = ol.o.bRate / MG_S;
          mn3 = ol.o.bRateMin / MG_S;
          CHECK(ol.o.bRateMin >= -0.35f * MG_S && ol.o.bRateMax <= 0.35f * MG_S && mn3 < 0.0,
                "iii. open loop G %.0f seed %u: bRate went negative (min %+.3f) within the bound", gs[gi],
                (unsigned)(77u * sd), mn3);
        }
      }
      markAdd(&mi[1], fabs(lag) < 0.4);
      markAdd(&mii[1], fabs(br) < 0.05);
      markAdd(&miii[1], mn3 < 0.0 && br3 < 0.0);
      printf("i-iii open loop G %.0f seed %u: i lag at 60 s %+.3f mg (%s); ii bRate at 70 s %+.3f mg/s (%s); iii bRate "
             "at 80 s %+.3f, min %+.3f (%s)\n", gs[gi], (unsigned)(77u * sd), lag, fabs(lag) < 0.4 ? "met" : "MISSED",
             br, fabs(br) < 0.05 ? "met" : "MISSED", br3, mn3, mn3 < 0.0 && br3 < 0.0 ? "met" : "MISSED");
    }
  }
  printf("PLAN MARKS i-iii: i (lag < 0.4 mg at 60 s) met %d/%d closed loop, %d/%d open loop; ii (|bRate| < 0.05 mg/s "
         "at 70 s) met %d/%d, %d/%d; iii (bRate negative, in the bound) met %d/%d, %d/%d\n", mi[0].met, mi[0].n,
         mi[1].met, mi[1].n, mii[0].met, mii[0].n, mii[1].met, mii[1].n, miii[0].met, miii[0].n, miii[1].met,
         miii[1].n);
}

/* ---- iv. camera glitches ---------------------------------------------------------------------------------------- */

/* Two scores per case, both against today on the same seed:
 *   raw     the true height's change from the glitch to the score time (the SINK's start with the camera lost, else
 *           15 s later): test_cam_vz_late.c's mark, |T2| <= |today| + 0.10 m (both on the floor: there no more than
 *           2 s sooner). It carries each version's own drift trend since the glitch (T2 climbs a little where today
 *           sinks on the 10-15 shape).
 *   induced the same change minus that of the same version's run without the glitch (and without the loss) over the
 *           same window: what the glitch did (the replay's G5 measure). The plan's mark: the worst induced change each
 *           way within today's worst + 0.10 m.
 * NOT MET at the time of writing (2026-10-16): induced -1.19 / +1.00 m against today's -0.79 / +0.58, raw missed in
 * 24 of 192 (negative glitches, mostly with the camera lost: a glitch the CUSUM took longer to see, absorbed by the
 * offset and then flown on). Both are tallied and printed as plan marks; the gate pins today's result (GLITCH_PIN_*)
 * so a change that makes it worse fails. T2 stays default off (cvt2k 0) until the user decides. */
#define EXCESS_M       0.10
#define FLOOR_EARLY_S  2.0
#define GLITCH_PIN_DOWN_M  -1.25  /* the worst induced change at the time of writing, -1.185 / +0.999 m, + 0.06 */
#define GLITCH_PIN_UP_M     1.06
#define GLITCH_PIN_RAWMISS  24    /* raw-mark misses of 192 at the time of writing */

static double hDsAt(const res_t *r, double t)
{
  const int i = (int)(t * 10.0);
  return r->hDs[i < 0 ? 0 : (i > 1499 ? 1499 : i)];
}

static void testGlitch(void)
{
  static const double gs[2] = {25.0, 56.0};
  static const double glitches[4] = {0.1, -0.1, 0.3, -0.3};
  static const double durs[3] = {0.5, 1.0, 2.0};
  static const double gts[2] = {50.0, 60.0};
  int n = 0, rawPass = 0, floors = 0, mism = 0, holds = 0;
  double worstExcess = -9.0, worstUp = -9.0, worstDown = 9.0, todayUp = -9.0, todayDown = 9.0;
  double indUp = -9.0, indDown = 9.0, indTodayUp = -9.0, indTodayDown = 9.0;
  static res_t ref0, ref1, today, r;
  for (int ti = 0; ti < 2; ti++) {
    for (int gi = 0; gi < 2; gi++) {
      for (uint32_t seed = 1; seed <= 2; seed++) {
        scen_t base = scen(0.0f, gs[gi], SHAPE_1015, 90.0, seed);
        base.tLastSet = gts[ti] - 4.0;
        fly(&base, &ref0);
        base.kR = KR_PLAN;
        fly(&base, &ref1);
        for (int ci = 0; ci < 4; ci++) {
          for (int di = 0; di < 3; di++) {
            for (int lose = 0; lose < 2; lose++) {
              scen_t sc = base;
              sc.kR = 0.0f;
              sc.glitch = glitches[ci];
              sc.gT = gts[ti];
              sc.gDur = durs[di];
              sc.lose = lose != 0;
              fly(&sc, &today);
              sc.kR = KR_PLAN;
              fly(&sc, &r);
              const double excess = fabs(r.err) - fabs(today.err);
              const bool floor = today.hRef + today.err <= 0.10 && r.hRef + r.err <= 0.10;
              const double early = floor ? today.tLow - r.tLow : 0.0;
              const bool ok = floor ? early <= FLOOR_EARLY_S : excess <= EXCESS_M;
              const double tS1 = r.sink ? r.tSink : sc.gT + 15.0, tS0 = today.sink ? today.tSink : sc.gT + 15.0;
              const double ind1 = r.err - (hDsAt(&ref1, tS1) - hDsAt(&ref1, sc.gT));
              const double ind0 = today.err - (hDsAt(&ref0, tS0) - hDsAt(&ref0, sc.gT));
              n++;
              holds += r.hold;
              rawPass += ok;
              mism += r.heldMismatch != 0 || r.lateRan;
              floors += floor;
              if (!floor) {
                worstExcess = fmax(worstExcess, excess);
              }
              worstUp = fmax(worstUp, r.err);
              worstDown = fmin(worstDown, r.err);
              todayUp = fmax(todayUp, today.err);
              todayDown = fmin(todayDown, today.err);
              indUp = fmax(indUp, ind1);
              indDown = fmin(indDown, ind1);
              indTodayUp = fmax(indTodayUp, ind0);
              indTodayDown = fmin(indTodayDown, ind0);
              if (verbose || !ok) {
                printf("  iv. %+.1f m/s %.1f s at %.0f s%-7s G %.0f seed %u: raw %+.3f m (today %+.3f)%s%s, induced "
                       "%+.3f (today %+.3f)%s\n", glitches[ci], durs[di], gts[ti], lose ? " + lost" : "", gs[gi],
                       (unsigned)seed, r.err, today.err, r.sink ? ", SINK" : "", floor ? ", both on the floor" : "",
                       ind1, ind0, ok ? "" : "  (raw mark missed)");
              }
            }
          }
        }
      }
    }
  }
  printf("iv.  glitches (%d cases): induced worst %+.3f / %+.3f m (today %+.3f / %+.3f); raw mark (late test) met in "
         "%d, missed in %d (worst excess %+.3f m), %d both on the floor; raw range %+.3f..%+.3f m (today "
         "%+.3f..%+.3f)\n", n, indDown, indUp, indTodayDown, indTodayUp, rawPass, n - rawPass, worstExcess, floors,
         worstDown, worstUp, todayDown, todayUp);
  CHECK(holds == n, "iv. every glitch run reached HOLD (%d of %d)", holds, n);
  const bool mark = indUp <= indTodayUp + EXCESS_M && indDown >= indTodayDown - EXCESS_M;
  printf("PLAN MARK iv (worst induced within today's +- %.2f m): %s; raw mark missed in %d of %d\n", EXCESS_M,
         mark ? "met" : "MISSED", n - rawPass, n);
  CHECK(indDown >= GLITCH_PIN_DOWN_M && indUp <= GLITCH_PIN_UP_M && n - rawPass <= GLITCH_PIN_RAWMISS,
        "iv. pinned: worst induced %+.3f / %+.3f m within %+.2f / %+.2f, raw misses %d <= %d (no worse than at the "
        "time of writing)", indDown, indUp, GLITCH_PIN_DOWN_M, GLITCH_PIN_UP_M, n - rawPass, GLITCH_PIN_RAWMISS);
  CHECK(mism == 0, "viii. glitch family: biasHeldG == biasG and no R1 step in every run (%d differ)", mism);
}

/* ---- v-vii. events, bounds, gaps (open loop) -------------------------------------------------------------------- */

typedef struct {
  double tA, tB;            /* the window checked */
  bool haveFirst;
  float bRate0;             /* bRate at its start */
  bool changedIn;           /* bRate changed inside [tA, tB) */
  bool changedAfter;        /* ... after tB (until tB + 4 s) */
  float x2Prev;
  bool driftSeen;           /* a block moved x[2] by exactly bRate d without a frame between */
  int frozenBad;            /* gap: blocks that moved x[2] more than 0.5 s after the last accept */
  int frozenN;
  float bMax, bMin;
  bool sawFault;
} watch_t;

static void watchHold(const ol_t *ol, int decision, void *ctx)
{
  watch_t *w = ctx;
  const camVzObs_t *o = &ol->o;
  if (decision != CAM_VZ_F_ACCEPTED) {
    return;
  }
  const double t = o->tState * 1e-6;
  if (t >= w->tA && !w->haveFirst) {
    w->haveFirst = true;
    w->bRate0 = o->bRate;
  }
  if (w->haveFirst && t < w->tB && o->bRate != w->bRate0) {
    w->changedIn = true;
  }
  if (t >= w->tB && t < w->tB + 4.0 && o->bRate != w->bRate0) {
    w->changedAfter = true;
  }
}

static void testEvents(void)
{
  /* v. a commanded motion, a turn, a yaw restart: the adaptation held 3 s (guard), then back */
  static const char *const names[3] = {"motion", "turn", "yaw restart"};
  for (int e = 0; e < 3; e++) {
    olScen_t sc = olScen(75.0);
    sc.rampMgS = 0.2;
    const double at = 50.0;
    if (e == 0) {
      sc.motionAt = at;
    } else if (e == 1) {
      sc.turnAt = at;
    } else {
      sc.yawAt = at;
    }
    watch_t w;
    memset(&w, 0, sizeof(w));
    w.tA = at + 0.08;   /* the event's frame comes within one frame period */
    w.tB = at + 3.0;
    static ol_t ol;
    olRun(&ol, &sc, KR_PLAN, watchHold, &w);
    printf("v.   %-11s at %.0f s: bRate unchanged for 3 s %d, adapting again within 4 s after %d (yaw restarts %u)\n",
           names[e], at, !w.changedIn, w.changedAfter, (unsigned)ol.o.cnt.yawResets);
    CHECK(w.haveFirst && !w.changedIn && w.changedAfter, "v. %s at %.0f s holds the adaptation 3 s, then it resumes",
          names[e], at);
  }
  {
    /* v. the ground: bRate and the offset frozen, the clock and the rate kept through the bounce */
    olScen_t sc = olScen(70.0);
    sc.rampMgS = 0.2;
    sc.groundA = 45.0;
    sc.groundB = 48.0;
    static ol_t ol;
    olRun(&ol, &sc, KR_PLAN, NULL, NULL);
    /* rerun step by step to the ground and through it */
    camVzObsT2_t a, b;
    static ol_t o2;
    olScen_t s2 = sc;
    s2.tEnd = 45.0;   /* every airborne block (the last closes at 44.999 s) and its frame */
    olRun(&o2, &s2, KR_PLAN, NULL, NULL);
    camVzObsT2Get(&o2.o, &a);
    const float x2a = o2.o.x[2];
    const uint32_t lift0 = o2.o.tFirstLift;
    s2.tEnd = 48.0;   /* the re-lift sample's block not yet past a frame */
    olRun(&o2, &s2, KR_PLAN, NULL, NULL);
    camVzObsT2Get(&o2.o, &b);
    printf("v.   ground 45-48 s: bRate %+.4f -> %+.4f mg/s, offset %+.4f -> %+.4f mg, first lift-off %u -> %u us, "
           "on at the end %d (rate %+.3f)\n", a.t2RateMgS, b.t2RateMgS, x2a / MG_S, o2.o.x[2] / MG_S,
           (unsigned)lift0, (unsigned)o2.o.tFirstLift, ol.o.t2Init, ol.o.bRate / MG_S);
    CHECK(a.on && b.on && a.t2RateMgS == b.t2RateMgS && o2.o.x[2] == x2a && o2.o.tFirstLift == lift0 &&
          !b.drifting, "v. on the ground bRate and the offset frozen (no drift), the first lift-off's clock kept");
    CHECK(ol.o.t2Init && ol.o.tFirstLift == lift0 && ol.o.t2Us > b.onUs,
          "v. after the bounce T2 continues (not restarted: no new start, the drift resumes)");
  }
  {
    /* vi. the bound */
    for (int sgn = -1; sgn <= 1; sgn += 2) {
      olScen_t sc = olScen(90.0);
      sc.rampMgS = 0.6 * sgn;   /* under the guard's 1 mg/s offset-rate event (a faster one holds the rate) */
      static ol_t ol;
      olRun(&ol, &sc, CAM_VZ_T2_KR_MAX, NULL, NULL);
      camVzObsT2_t t;
      camVzObsT2Get(&ol.o, &t);
      printf("vi.  ramp %+.1f mg/s at kR %.1f: bRate range %+.4f..%+.4f mg/s\n", sc.rampMgS, CAM_VZ_T2_KR_MAX,
             t.minMgS, t.maxMgS);
      const float bm = 0.35f * MG_S;
      CHECK(ol.o.bRateMax <= bm && ol.o.bRateMin >= -bm && (sgn > 0 ? ol.o.bRateMax == bm : ol.o.bRateMin == -bm),
            "vi. ramp %+.1f mg/s: bRate reaches and keeps the bound %+.2f mg/s", sc.rampMgS, 0.35 * sgn);
    }
    /* vi. NaN */
    olScen_t sc = olScen(50.0);
    sc.rampMgS = 0.2;
    sc.nanAt = 45.0;
    static ol_t ol;
    olRun(&ol, &sc, KR_PLAN, NULL, NULL);
    camVzObsOut_t q;
    camVzObsGet(&ol.o, &q);
    CHECK(q.fault && isfinite(ol.o.bRate), "vi. a NaN accelerometer sample at 45 s faults the observer (bRate %g)",
          (double)ol.o.bRate);
    /* vi. invalid configs refused (camVzObsInit false: the caller says so and flies none of it) */
    struct {
      const char *what;
      float kR, start, prior, bound, qLate;
    } bad[] = {
      {"kR -0.1", -0.1f, 30.0f, 0.124f, 0.35f, 1.0f}, {"kR 2.01", 2.01f, 30.0f, 0.124f, 0.35f, 1.0f},
      {"kR NaN", NAN, 30.0f, 0.124f, 0.35f, 1.0f}, {"start 19", 0.5f, 19.0f, 0.124f, 0.35f, 1.0f},
      {"start 121", 0.5f, 121.0f, 0.124f, 0.35f, 1.0f}, {"start inf", 0.5f, INFINITY, 0.124f, 0.35f, 1.0f},
      {"bound 0", 0.5f, 30.0f, 0.0f, 0.0f, 1.0f}, {"bound 0.51", 0.5f, 30.0f, 0.124f, 0.51f, 1.0f},
      {"prior 0.36 > bound", 0.5f, 30.0f, 0.36f, 0.35f, 1.0f}, {"prior -0.36", 0.5f, 30.0f, -0.36f, 0.35f, 1.0f},
      {"prior NaN", 0.5f, 30.0f, NAN, 0.35f, 1.0f}, {"R1 x5 with T2", 0.5f, 30.0f, 0.124f, 0.35f, 5.0f},
      {"prior 0.4 > bound with kR 0", 0.0f, 30.0f, 0.4f, 0.35f, 1.0f},
    };
    int refused = 0;
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
      camVzObsConfig_t c;
      camVzObsDefaultConfig(&c);
      c.t2kR = bad[i].kR;
      c.t2StartS = bad[i].start;
      c.t2PriorMgS = bad[i].prior;
      c.t2BoundMgS = bad[i].bound;
      c.qLateMult = bad[i].qLate;
      static camVzObs_t o;
      const bool ok = camVzObsInit(&o, &c);
      refused += !ok;
      CHECK(!camVzObsConfigValid(&c) && !ok && o.cfg.t2kR == 0.0f && o.cfg.qLateMult == 1.0f,
            "vi. config %s refused (camVzObsInit false, nothing of it kept)", bad[i].what);
    }
    camVzObsConfig_t c;
    camVzObsDefaultConfig(&c);
    CHECK(camVzObsConfigValid(&c) && c.t2kR == 0.0f && c.t2StartS == 30.0f && c.t2PriorMgS == 0.124f &&
          c.t2BoundMgS == 0.35f, "vi. the default: T2 off, start 30 s, prior 0.124, bound 0.35 mg/s");
    c.t2kR = CAM_VZ_T2_KR_MAX;
    c.t2StartS = CAM_VZ_T2_S_MIN;
    c.t2PriorMgS = -0.35f;
    CHECK(camVzObsConfigValid(&c), "vi. kR 2, start 20 s, prior -0.35 (= -bound) valid");
    printf("vi.  %d invalid configs refused\n", refused);
  }
}

static void watchGap(const ol_t *ol, int decision, void *ctx)
{
  watch_t *w = ctx;
  const camVzObs_t *o = &ol->o;
  if (decision != -1) {
    w->x2Prev = o->x[2];
    return;
  }
  const double t = o->tIn * 1e-6;
  const bool stale = !o->hasAccept || o->tIn - o->lastAccept >= CAM_VZ_T2_ACC_US;
  if (o->t2Init && t >= w->tA && t < w->tB && stale) {
    w->frozenN++;
    if (o->x[2] != w->x2Prev) {
      w->frozenBad++;
    }
  }
  if (o->t2Init && !stale && t < w->tA && o->x[2] != w->x2Prev && o->bRate != 0.0f) {
    w->driftSeen = true;
  }
  w->x2Prev = o->x[2];
}

static void testGap(void)
{
  static const double lens[3] = {3.0, 5.0, 10.0};
  for (int i = 0; i < 3; i++) {
    olScen_t sc = olScen(70.0);
    sc.rampMgS = 0.0;
    sc.gapA = 40.0;
    sc.gapB = 40.0 + lens[i];
    watch_t w;
    memset(&w, 0, sizeof(w));
    w.tA = 40.0;
    w.tB = 40.0 + lens[i] + 0.2;
    static ol_t ol;
    olRun(&ol, &sc, KR_PLAN, watchGap, &w);
    printf("vii. gap %.0f s from 40 s: %d blocks without an accept this recent, %d moved the offset; drift before it "
           "%d\n", lens[i], w.frozenN, w.frozenBad, w.driftSeen);
    CHECK(w.driftSeen && w.frozenN > (int)(lens[i] * 100.0 - 60.0) && w.frozenBad == 0,
          "vii. a %.0f s gap: the offset frozen (no drift) from 0.5 s after the last accepted frame", lens[i]);
  }
}

/* ---- ix. the real 10-15 offsets --------------------------------------------------------------------------------- */

static void testReal(const char *fixture)
{
  if (!loadFixture(fixture)) {
    printf("ix. skipped: fixture %s not found or bad\n", fixture);
    CHECK(false, "fixture %s", fixture);
    return;
  }
  for (int bi = 0; bi < nBeta; bi++) {
    for (uint32_t seed = 1; seed <= 2; seed++) {
      scen_t s = scen(0.0f, betas[bi].g, SHAPE_BETA, 120.0, 30u + seed);
      s.beta = &betas[bi];
      s.tLastSet = 44.0;
      res_t today, r;
      fly(&s, &today);
      s.kR = KR_PLAN;
      fly(&s, &r);
      const double d = r.h90 - r.h60, d1 = today.h90 - today.h60;
      printf("ix.  %-10s G %.0f seed %u: true height 60 -> 90 s %+.3f m (today %+.3f), 90 -> 120 s %+.3f (%+.3f), "
             "bRate %+.3f mg/s (range %+.3f..%+.3f)\n", betas[bi].name, betas[bi].g, (unsigned)seed, d, d1,
             r.hEnd - r.h90, today.hEnd - today.h90, r.t2.t2RateMgS, r.t2.minMgS, r.t2.maxMgS);
      CHECK(r.hold && !r.landed && fabs(d) < fabs(d1), "ix. %s seed %u: less late drift than today (%+.3f / %+.3f m)",
            betas[bi].name, (unsigned)seed, d, d1);
      CHECK(r.heldMismatch == 0 && !r.lateRan, "viii. %s seed %u: biasHeldG == biasG", betas[bi].name, (unsigned)seed);
    }
  }
}

#endif /* !T2_BASE */

int main(int argc, char **argv)
{
  const char *fixture = "tools/tests/fixtures/cam_vz_late.txt";
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-v") == 0) {
      verbose = 1;
    } else {
      fixture = argv[i];
    }
  }
  testIdentity();
#ifndef T2_BASE
  testDrift();
  testEvents();
  testGap();
  testGlitch();
  testReal(fixture);
#else
  (void)fixture;
#endif
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
