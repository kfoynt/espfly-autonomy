/*
 * Closed-loop fault tests of the camera height observer's late offset noise
 * (qLateMult, cam_vz_obs.h "Late offset noise", R1 of the height study
 * 2026-10-15, its section 6.4; docs/autonomy-a2b.md 25): the real vertical
 * controller (vert_ctrl.c, HOLD at 1.00 m on its sim plant vzSim) flies on the
 * real shadow observer (cam_vz_shadow.c + cam_vz_obs.c, fed as cam_vz_glue.c
 * feeds it: 10 ms means, the newest camera frame after each block, the
 * controller's camera fields from camVzShadowVzInput), over a synthetic camera
 * (a frame every 67.5 ms, 120 ms late, its window 15 ms past the capture,
 * 0.12 px of noise, G the scene's) and an accelerometer whose in-flight offset
 * drifts. The FSM's camera rule is emulated: the SINK starts 5 s after the
 * last step the controller used the camera (AUTO_CAM_LOST_MAX_MS). Claude's
 * SET_HEIGHT is emulated before the fault (an exact photo every 12 s from
 * 20 s, the last 4 s before it: vzShiftHeight() and the state machine's
 * correction move, vRef = clamp(1 /s x the error, 0.15 m/s), the observer's
 * motion flag set through it), so that both versions meet the fault near
 * 1.00 m and a camera lost on top of a drifting offset does not end on the
 * floor (today's drone sinks ~2 cm/s by 60 s at G 25 without them).
 *
 * The error is the true height's change from the fault's start: at the SINK's
 * start with a camera lost, else 15 s after the fault. PASS (the study's): the
 * late offset noise's error at most today's (x1, same seed) + 0.10 m; when
 * both are on the floor then (true height <= 0.10 m: the error is only the
 * height each met the fault at, so it says nothing), the late offset noise
 * there no more than 2 s sooner than today (scored and printed apart; until
 * 2026-10-15's review such cases passed whatever they did, and hid the frozen
 * held offset of a long motion).
 *
 *   1. Synthetic 90 s hovers, the offset like the 10-15 flights' (falling
 *      0.04 mg/s to 25 s, then rising 0.20 mg/s): no fault (the multiplier
 *      must help), and a camera glitch of +-0.1 / +-0.3 m/s lasting 0.5, 1 or
 *      2 s at 60 s, each with and without the camera lost right after it, at
 *      G 25 and 56 px/m, two seeds, x3 and x5 (the most, cvqlate 1..5) from
 *      40 s.
 *   2. The real camera of two 2026-10-13 interactive sorties spliced into the
 *      real in-flight offsets of the three 10-15 hovers from 50 s (fixture
 *      tools/tests/fixtures/cam_vz_late.txt, gen_cam_vz_late_fixture.py):
 *      F1 (the person, 33 s) and F2 (the bright window, 9.5 s), with and
 *      without the camera lost at the segment's end.
 *   3. With the multiplier 1 the controller's run is bit for bit the one
 *      with the observer of 65dfcba (a hash of every step's thrust and
 *      estimate, pinned; built with -DCVL_BASE against that commit's
 *      cam_vz_obs.c and cam_vz_shadow.c).
 *   4. Resting on the floor: K_RANGE's time after the arrival, printed.
 *   5. Claude's MOVEs / YAWs past the step, one held 45-90 s and 6 s on /
 *      4 s off: no worse than today (+0.05 m, or on the floor no more than
 *      2 s sooner).
 *   6. Slow camera errors (2-6 cm/s for 5-20 s, a step or a ramp, with and
 *      without a loss, on the drifting and on a flat offset): the known
 *      residual risk of the faster offset, pinned (at most 10 % of the cases
 *      over the mark, none more than 0.45 m; docs/autonomy-a2b.md 25.8).
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=c99 -O2 -Wall -Wextra -I$C/utils/interface -I$C/modules/interface tools/tests/test_cam_vz_late.c \
 *      $C/utils/src/cam_vz_shadow.c $C/utils/src/cam_vz_obs.c $C/utils/src/vert_ctrl.c \
 *      -lm -o /tmp/test_cam_vz_late && /tmp/test_cam_vz_late [fixture] [-v]
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

/* ---- the fixture ---------------------------------------------------------------------------------------------- */

typedef struct {
  char name[16];
  float g;                  /* the scene's G, px/m */
  float beta[BETA_N];       /* in-flight offset vs the ground baseline, g, every 0.1 s from the lift-off */
} betaTab_t;

typedef struct {
  char name[16];
  int n;
  float t[SEG_N];           /* s from the segment's first frame */
  float err[SEG_N];         /* px */
  bool ok[SEG_N];
} seg_t;

static betaTab_t betas[3];
static seg_t segs[2];
static int nBeta, nSeg;

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
    } else if (line[0] == 'S' && nSeg < 2) {
      seg_t *s = &segs[nSeg++];
      if (sscanf(line, "S %15s %d", s->name, &s->n) != 2 || s->n > SEG_N) {
        fclose(f);
        return false;
      }
      for (int i = 0; i < s->n; i++) {
        double t;
        long dt, ok, e;
        if (fscanf(f, "%lf %ld %ld %ld", &t, &dt, &ok, &e) != 4) {
          fclose(f);
          return false;
        }
        s->t[i] = (float)t;
        s->ok[i] = ok != 0;
        s->err[i] = (float)(e * 0.01);
      }
    }
  }
  fclose(f);
  return nBeta == 3 && nSeg == 2;
}

/* ---- one sortie ------------------------------------------------------------------------------------------------ */

typedef struct {
  float mult;               /* qLateMult (1: today) */
  float stepS;              /* qLateS */
  double g;                 /* the scene, px/m */
  double glitch;            /* m/s the camera reads beyond the truth (+ a climb) ... */
  double gT, gDur;          /* ... from gT for gDur (s after the lift-off) */
  bool lose;                /* no frame from gT + gDur on */
  const betaTab_t *beta;    /* NULL: the synthetic offset */
  const seg_t *seg;         /* the camera's error from gT: this segment's (once) */
  double tEnd;              /* s after the lift-off */
  uint32_t seed;
  double tLastSet;          /* the last SET_HEIGHT (from 20 s every 12 s up to this; < 20: none) */
  double restT;             /* > 0: from this time SET_HEIGHTs 6 s apart fly the drone down onto the floor (each at most
                             * VZ_SHIFT_MAX_M, the last to 2 cm under it), where it then rests (section 4) */
  double moveT, moveDur;    /* > 0: Claude's MOVEs / YAWs (no vertical part: the observer's motion flag only) from moveT
                             * for moveDur s, ... */
  double moveOn, moveOff;   /* ... moveOn s on, moveOff s off (moveOff 0: one motion held throughout; section 5) */
  bool flat;                /* the synthetic offset flat at the prior's mean (no drift; section 6) */
  bool ramp;                /* the glitch grows from 0 to its size over gDur (a slow camera error; section 6) */
} scen_t;

typedef struct {
  bool hold;                /* reached HOLD */
  bool sink;                /* the SINK started (the camera out of use 5 s) */
  double tSink;             /* s after the lift-off */
  double hRef;              /* true height at gT */
  double err;               /* true height - hRef: at the SINK's start, else at gT + 15 s (or the end) */
  double errMax;            /* max |true - hRef| from gT to the SINK or the end */
  double h40, h60, hEnd;    /* true heights (h40: at 38 s) */
  bool landed;              /* the controller started a LAND of its own ... */
  vzReason_t land;
  double tLand;             /* ... at this time after the lift-off */
  double tFloor;            /* the first time the true height was <= 0.02 m after restT (-1: never) */
  double tLow;              /* the first time the true height was <= 0.10 m from gT on (s after the lift-off; -1:
                             * never): the outcome when both versions end on the floor */
  double ctlErrMg;          /* at the SINK's start: the controller's latched offset minus the true one, mg */
  float lateS;              /* the observer's time with the multiplier on */
  uint32_t guards;
  uint64_t hash;            /* every step's thrust and estimate (FNV-1a) */
} res_t;

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

/* The in-flight offset (g, against the ground baseline) t s after the lift-off. */
static double offsetAt(const scen_t *sc, double t)
{
  if (sc->beta != NULL) {
    const double k = t * 10.0;
    const int i = (int)k;
    if (i >= BETA_N - 1) {
      return sc->beta->beta[BETA_N - 1];
    }
    return sc->beta->beta[i] + (sc->beta->beta[i + 1] - sc->beta->beta[i]) * (k - i);
  }
  if (sc->flat) {
    return 0.00137;
  }
  /* the 10-15 shape: +1.37 mg (the prior's mean) falling 0.04 mg/s to 25 s, then rising 0.20 mg/s */
  return 0.00137 - 0.00004 * fmin(t, 25.0) + (t > 25.0 ? 0.0002 * (t - 25.0) : 0.0);
}

static float hist[HIST_N];   /* the true height every ms */

static double hAt(uint64_t k, double tUs)
{
  /* tUs: us since the start; k: the latest ms index written */
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

/* the fields after restT of a scenario without motions, a flat offset or a ramp */
#define SC_PLAIN 0.0, 0.0, 0.0, 0.0, false, false

/* Claude's MOVE / YAW at ta (section 5's pattern) */
static bool motionAt(const scen_t *sc, double ta)
{
  if (sc->moveDur <= 0.0 || ta < sc->moveT || ta >= sc->moveT + sc->moveDur) {
    return false;
  }
  return sc->moveOff <= 0.0 || fmod(ta - sc->moveT, sc->moveOn + sc->moveOff) < sc->moveOn;
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
#ifndef CVL_BASE
  oc.qLateMult = sc->mult;
  oc.qLateS = sc->stepS;
#endif
  static camVzShadow_t sh;
  CHECK(camVzShadowInit(&sh, &oc), "observer config");
  const uint64_t base = 1000000000ull;
  vzOutput_t vo;
  memset(&vo, 0, sizeof(vo));
  float applied = 0.0f;
  /* the glue's block */
  double blkSum = 0.0;
  int blkN = 0;
  bool groundStart = false, wasZero = true;
  uint32_t seq = 0;
  uint64_t nextCap = 0, lastSeqCap = 0;
  bool haveCap = false;
  double liftT = -1.0, tHold = -1.0, lastUsed = -1.0;
  int segI = 0;
  double nextSet = 20.0;
  bool moving = false;
  int rests = 0;
  const double t0 = 1.0;   /* 1 s at rest, then TAKEOFF */
  const double tStop = t0 + 12.0 + sc->tEnd;
  for (uint64_t k = 0;; k++) {
    const double t = k * 1e-3;
    const double ta = liftT >= 0.0 ? t - liftT : -1.0;
    if (t > tStop || (liftT >= 0.0 && ta > sc->tEnd)) {
      break;
    }
    /* the plant's accelerometer plus the in-flight offset */
    if (liftT < 0.0 && sim.liftT >= 0.0f) {
      liftT = sim.liftT;
    }
    const double off = ta >= 0.0 ? offsetAt(sc, ta) : 0.0;
    const float accZ = sim.accZ + (float)off;
    hist[k % HIST_N] = sim.h;
    /* the controller, on the shadow's published estimate */
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
    if (sc->restT > 0.0 && ta >= sc->restT + 6.0 * rests && vo.phase == VZ_HOLD && sim.h > 0.02f && !moving) {
      /* "you are this much higher": the correction flies down to 2 cm under the floor at most, so the drone arrives
       * slowly at the move's end and then rests with no descent commanded */
      const float d = fminf(sim.h + 0.02f, VZ_SHIFT_MAX_M);
      vzShiftHeight(&st, d);
      vo.h += d;                              /* the move below reads this step's estimate */
      moving = true;
      nextSet = ta + 11.5;   /* the move's own clock below */
      rests++;
    }
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
      r->tLand = ta;
      if (sc->restT > 0.0) {
        break;
      }
    }
    if (sc->restT > 0.0 && ta >= sc->restT && r->tFloor == 0.0 && sim.h <= 0.02f) {
      r->tFloor = ta;
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
#ifndef CVL_BASE
      camVzObsSetMotion(&sh.obs, moving || motionAt(sc, ta));   /* the glue: a commanded motion (holdWait) */
#endif
      camVzShadowStep(&sh, &si);
      groundStart = false;
      blkSum = 0.0;
      blkN = 0;
      /* captures up to now - LAT_US: the newest is given (one per block at most at 67.5 ms) */
      const uint64_t nowUs = k * DT_US;
      while (nextCap + LAT_US <= nowUs) {
        lastSeqCap = nextCap;
        haveCap = true;
        nextCap += FRAME_US;
        seq++;
      }
      if (haveCap) {
        haveCap = false;
        const double tc = lastSeqCap * 1e-6;          /* capture, s since the start */
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
            dy -= sc->g * sc->glitch * (sc->ramp ? (tca - sc->gT) / sc->gDur : 1.0) * FRAME_US * 1e-6;
          }
          if (sc->seg != NULL && tca >= sc->gT) {
            while (segI + 1 < sc->seg->n && sc->seg->t[segI + 1] <= tca - sc->gT) {
              segI++;
            }
            if (tca - sc->gT <= sc->seg->t[sc->seg->n - 1] + 0.1) {
              dy += sc->seg->err[segI];
              f.ok = f.ok && sc->seg->ok[segI];
            }
          }
          f.dy = (float)dy;
          f.dx = (float)(0.1 * gauss());
          f.texture = 20.0f;
          f.residual = 3.0f;
          camVzShadowFrame(&sh, &f, base + nowUs);
        }
      }
    }
    if (ta >= 0.0) {
      if (ta >= 38.0 && r->h40 == 0.0) {   /* the observer's lift-off (by the thrust) can come before the plant's */
        r->h40 = sim.h;
      }
      if (ta >= 60.0 && r->h60 == 0.0) {
        r->h60 = sim.h;
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
        r->ctlErrMg = (st.lastBias - (1.0 + off - vo.g0)) * 1e3;   /* camBias = biasHeldG + 1 - g0 (cam_vz_glue.c) */
        break;
      }
    }
  }
#ifndef CVL_BASE
  r->lateS = sh.obs.lateUs * 1e-6f;
  r->guards = sh.obs.lateGuards;
#endif
  r->tLand = r->landed ? r->tLand : -1.0;
}

/* ---- 1. synthetic faults ---------------------------------------------------------------------------------------- */

typedef struct {
  int n, pass;
  int over;                 /* not both on the floor, R1's error more than today's + 0.10 m */
  int floor, floorPass;     /* both on the floor, R1 there no sooner than today - FLOOR_EARLY_S */
  double worstExcess;       /* max |err R1| - |err today| (not both on the floor) */
  double worstEarly;        /* both on the floor: max (today's time to it - R1's), s */
} tally_t;

#define EXCESS_M       0.10   /* the study's mark */
#define FLOOR_EARLY_S  2.0    /* both on the floor: R1 may get there this much sooner (the heights say nothing; at the
                               * 4-5 cm/s these runs sink at near the floor, EXCESS_M) */

/* Scores R1 against today (same seed): neither on the floor, R1's error at most today's + EXCESS_M; both on the
 * floor (true height <= 0.10 m when scored: the error is the height each met the fault at), R1 there no more
 * than FLOOR_EARLY_S sooner. gate: CHECK each case (false: only tallied, the family's own mark below). */
static bool compare(tally_t *ty, const char *what, const res_t *today, const res_t *r1, float mult, bool gate)
{
  const double excess = fabs(r1->err) - fabs(today->err);
  const bool floor = today->hRef + today->err <= 0.10 && r1->hRef + r1->err <= 0.10;
  const double early = floor ? today->tLow - r1->tLow : 0.0;
  const bool ok = floor ? early <= FLOOR_EARLY_S : excess <= EXCESS_M;
  ty->n++;
  ty->pass += ok;
  if (floor) {
    ty->floor++;
    ty->floorPass += ok;
    ty->worstEarly = fmax(ty->worstEarly, early);
  } else {
    ty->over += !ok;
    ty->worstExcess = fmax(ty->worstExcess, excess);
  }
  if (verbose || !ok) {
    printf("  %-48s x%.0f err %+.3f m (today %+.3f), max %.3f (%.3f)%s%s, ctl offset %+.2f mg (%+.2f), on %.1f s, "
           "%u guards", what, mult, r1->err, today->err, r1->errMax, today->errMax, r1->sink ? ", SINK" : "",
           today->sink ? " (today SINK)" : "", r1->ctlErrMg, today->ctlErrMg, r1->lateS, (unsigned)r1->guards);
    if (floor) {
      printf(", both on the floor: at %.1f s (today %.1f s)", r1->tLow, today->tLow);
    }
    printf("%s\n", ok ? "" : (gate ? "  <-- FAIL" : "  <-- over"));
  }
  if (gate) {
    CHECK(ok, "%s x%.0f: %s", what, mult, floor ? "both on the floor, R1 there no more than 2 s sooner" :
          "error at most today's + 0.10 m");
  }
  return ok;
}

static void tallyPrint(const char *name, const tally_t *ty)
{
  printf("%s: %d of %d pass; %d of %d not on the floor within today's error + %.2f m (worst excess %+.3f m); %d of "
         "%d with both on the floor there no more than %.1f s sooner (worst %+.1f s)\n", name, ty->pass, ty->n,
         ty->n - ty->floor - ty->over, ty->n - ty->floor, EXCESS_M, ty->worstExcess, ty->floorPass, ty->floor,
         FLOOR_EARLY_S, ty->worstEarly);
}

static void testSynthetic(void)
{
  static const double gs[2] = {25.0, 56.0};
  static const double glitches[4] = {0.1, -0.1, 0.3, -0.3};
  static const double durs[3] = {0.5, 1.0, 2.0};
  static const float mults[2] = {3.0f, 5.0f};   /* cvqlate 1..5 (x10 was cut, docs/autonomy-a2b.md 25.8) */
  tally_t ty = {0, 0, 0, 0, 0, -9.0, -99.0};
  /* no fault: the late sink the multiplier is for */
  for (int gi = 0; gi < 2; gi++) {
    for (uint32_t seed = 1; seed <= 2; seed++) {
      scen_t sc = {1.0f, 40.0f, gs[gi], 0.0, 60.0, 0.0, false, NULL, NULL, 90.0, seed, 56.0, 0.0, SC_PLAIN};
      res_t today;
      fly(&sc, &today);
      CHECK(today.hold && !today.landed && !today.sink, "G %.0f seed %u: today hovers to 90 s (%s)", gs[gi],
            (unsigned)seed, vzReasonName(today.land));
      for (int mi = 0; mi < 2; mi++) {
        sc.mult = mults[mi];
        res_t r;
        fly(&sc, &r);
        const bool same40 = fabs(r.h40 - today.h40) < 1e-9;
        printf("no fault, G %.0f seed %u x%.0f: true height change 60 -> 90 s %+.3f m (today %+.3f), 38 s bit-identical "
               "%d, on %.1f s, %u guards\n", gs[gi], (unsigned)seed, mults[mi], r.hEnd - r.h60, today.hEnd - today.h60,
               same40, r.lateS, (unsigned)r.guards);
        CHECK(r.hold && !r.landed && !r.sink && same40 && fabs(r.hEnd - r.h60) < fabs(today.hEnd - today.h60),
              "no fault, G %.0f seed %u x%.0f: same as today to 38 s, less late drift (%+.3f / %+.3f m)", gs[gi],
              (unsigned)seed, mults[mi], r.hEnd - r.h60, today.hEnd - today.h60);
      }
    }
  }
  for (int gi = 0; gi < 2; gi++) {
    for (int ci = 0; ci < 4; ci++) {
      for (int di = 0; di < 3; di++) {
        for (int lose = 0; lose < 2; lose++) {
          for (uint32_t seed = 1; seed <= 2; seed++) {
            scen_t sc = {1.0f, 40.0f, gs[gi], glitches[ci], 60.0, durs[di], lose != 0, NULL, NULL, 90.0, seed, 56.0,
                         0.0, SC_PLAIN};
            res_t today;
            fly(&sc, &today);
            char what[96];
            snprintf(what, sizeof(what), "G %.0f %+.1f m/s %.1f s%s seed %u", gs[gi], glitches[ci], durs[di],
                     lose ? " + lost" : "", (unsigned)seed);
            CHECK(today.hold && (!lose || today.sink), "%s: today in HOLD%s", what, lose ? ", SINK" : "");
            for (int mi = 0; mi < 2; mi++) {
              sc.mult = mults[mi];
              res_t r;
              fly(&sc, &r);
              compare(&ty, what, &today, &r, mults[mi], true);
            }
          }
        }
      }
    }
  }
  tallyPrint("synthetic faults", &ty);
}

/* ---- 2. the real camera spliced into the real offsets ------------------------------------------------------------- */

static void testSplice(const char *fixture)
{
  if (!loadFixture(fixture)) {
    printf("splice skipped: fixture %s not found or bad\n", fixture);
    CHECK(false, "fixture %s", fixture);
    return;
  }
  tally_t ty = {0, 0, 0, 0, 0, -9.0, -99.0};
  for (int bi = 0; bi < nBeta; bi++) {
    for (int si = 0; si < nSeg; si++) {
      for (int lose = 0; lose < 2; lose++) {
        const seg_t *sg = &segs[si];
        scen_t sc = {1.0f, 40.0f, betas[bi].g, 0.0, 50.0, sg->t[sg->n - 1] + 0.1, lose != 0, &betas[bi], sg, 90.0,
                     11u + (uint32_t)bi, 44.0, 0.0, SC_PLAIN};
        res_t today;
        fly(&sc, &today);
        char what[96];
        snprintf(what, sizeof(what), "%s offsets, %s from 50 s%s", betas[bi].name, sg->name,
                 lose ? " + lost at its end" : "");
        for (int mi = 0; mi < 2; mi++) {
          sc.mult = mi == 0 ? 3.0f : 5.0f;
          res_t r;
          fly(&sc, &r);
          printf("splice %-40s x%.0f: err %+.3f m (today %+.3f), max %.3f (%.3f)%s%s, on %.1f s, %u guards\n", what,
                 sc.mult, r.err, today.err, r.errMax, today.errMax, r.sink ? ", SINK" : "",
                 today.sink ? " (today SINK)" : "", r.lateS, (unsigned)r.guards);
          compare(&ty, what, &today, &r, sc.mult, true);
        }
      }
    }
  }
  tallyPrint("splices", &ty);
}

/* ---- 5. long and repeated commanded motion past the step ------------------------------------------------------ */

/* Claude's MOVEs and YAWs (the observer's motion flag, a guard event) from 45 to 90 s on the synthetic drifting offset,
 * one held throughout and a 6 s on / 4 s off pattern: the guard holds the whole time or comes back every 10 s, so
 * the held offset would stay frozen for as long without CAM_VZ_QLATE_FREEZE_US (the review of 2026-10-15: x5 then
 * sank 0.27-0.35 m more than today from 60 to 90 s at G 56 and reached the floor at G 25). PASS: the true height
 * change from 60 to 90 s no larger than today's + MOTION_TOL_M. */
#define MOTION_TOL_M 0.05

static void testMotion(void)
{
  static const double gs[2] = {25.0, 56.0};
  static const float mults[2] = {3.0f, 5.0f};
  for (int pat = 0; pat < 2; pat++) {
    for (int gi = 0; gi < 2; gi++) {
      for (uint32_t seed = 1; seed <= 2; seed++) {
        scen_t sc = {1.0f, 40.0f, gs[gi], 0.0, 60.0, 0.0, false, NULL, NULL, 90.0, seed, 56.0, 0.0, SC_PLAIN};
        sc.moveT = 45.0;
        sc.moveDur = 45.0;
        sc.moveOn = pat == 0 ? 45.0 : 6.0;
        sc.moveOff = pat == 0 ? 0.0 : 4.0;
        res_t today;
        fly(&sc, &today);
        for (int mi = 0; mi < 2; mi++) {
          sc.mult = mults[mi];
          res_t r;
          fly(&sc, &r);
          const double d = r.hEnd - r.h60, d1 = today.hEnd - today.h60;
          /* G 25 sinks onto the floor in both by 90 s (today ~0.9 m from 60 s): then the time it got there */
          const bool floor = r.tLow >= 0.0 && today.tLow >= 0.0;
          const bool ok = floor ? today.tLow - r.tLow <= FLOOR_EARLY_S
                                : r.tLow < 0.0 && fabs(d) <= fabs(d1) + MOTION_TOL_M;
          printf("motion %s 45-90 s, G %.0f seed %u x%.0f: true height change 60 -> 90 s %+.3f m (today %+.3f)",
                 pat == 0 ? "held     " : "6 s / 4 s", gs[gi], (unsigned)seed, mults[mi], d, d1);
          if (r.tLow >= 0.0 || today.tLow >= 0.0) {
            printf(", at 0.10 m at %.1f s (today %.1f s)", r.tLow, today.tLow);
          }
          printf(", on %.1f s, %u guards\n", r.lateS, (unsigned)r.guards);
          CHECK(r.hold && ok, "motion %s, G %.0f seed %u x%.0f: %+.3f m against today's %+.3f m (+%.2f allowed; on "
                "the floor at %.1f s against %.1f s)", pat == 0 ? "held" : "6/4 s", gs[gi], (unsigned)seed, mults[mi],
                d, d1, MOTION_TOL_M, r.tLow, today.tLow);
        }
      }
    }
  }
}

/* ---- 6. slow camera errors -------------------------------------------------------------------------------------- */

/* A camera that reads a climb or a sink of 2, 4 or 6 cm/s it does not make, for 5, 10 or 20 s from 60 s (a step, or
 * a ramp from 0), under every per-frame gate, with and without the camera lost at its end, on the synthetic drifting
 * offset and on a flat one, G 25 and 56. The size of the late camera error the kyC check found on the 10-15 flights
 * (0.24-0.67 px/s, 1-3 cm/s, cause unknown). The faster offset takes such an error sooner than today's: the drone
 * follows a false sink up further, and after a loss keeps climbing on the offset it moved (the review of 2026-10-15:
 * x10 up to +0.62 m more than today, which cut the strength to x5). No guard tells such an error from the
 * accelerometer's own drift (a bound on the offset's move over 4.5-5 s, at 1.25-2.0 mg, took 0.06-0.17 m of the
 * benefit on the p1 and fastcheck replays and left most of these cases as they were). So this family is the known
 * residual risk, pinned, not a proof (docs/autonomy-a2b.md 25.8): at each strength at most SLOW_OVER_FRAC of the
 * cases more than today's error + 0.10 m (or, both on the floor, there more than 1 s sooner), and none more than
 * SLOW_WORST_M. */
#define SLOW_OVER_FRAC 0.10
#define SLOW_WORST_M   0.45

static void testSlowError(void)
{
  static const double gs[2] = {25.0, 56.0};
  static const double amps[6] = {0.02, -0.02, 0.04, -0.04, 0.06, -0.06};
  static const double durs[3] = {5.0, 10.0, 20.0};
  static const float mults[2] = {3.0f, 5.0f};
  tally_t ty[2] = {{0, 0, 0, 0, 0, -9.0, -99.0}, {0, 0, 0, 0, 0, -9.0, -99.0}};
  for (int flat = 0; flat < 2; flat++) {
    for (int gi = 0; gi < 2; gi++) {
      for (int ai = 0; ai < 6; ai++) {
        for (int di = 0; di < 3; di++) {
          for (int ramp = 0; ramp <= (flat ? 0 : 1); ramp++) {
            for (int lose = 0; lose < 2; lose++) {
              scen_t sc = {1.0f, 40.0f, gs[gi], amps[ai], 60.0, durs[di], lose != 0, NULL, NULL, 100.0, 3u, 56.0, 0.0,
                           SC_PLAIN};
              sc.flat = flat != 0;
              sc.ramp = ramp != 0;
              res_t today;
              fly(&sc, &today);
              char what[96];
              snprintf(what, sizeof(what), "%s G %.0f %+.2f m/s %.0f s%s%s", flat ? "flat" : "drift", gs[gi], amps[ai],
                       durs[di], ramp ? " ramp" : "", lose ? " + lost" : "");
              for (int mi = 0; mi < 2; mi++) {
                sc.mult = mults[mi];
                res_t r;
                fly(&sc, &r);
                compare(&ty[mi], what, &today, &r, mults[mi], false);
              }
            }
          }
        }
      }
    }
  }
  for (int mi = 0; mi < 2; mi++) {
    char name[48];
    snprintf(name, sizeof(name), "slow camera errors x%.0f", mults[mi]);
    tallyPrint(name, &ty[mi]);
    const int bad = ty[mi].n - ty[mi].pass;
    CHECK(bad <= SLOW_OVER_FRAC * ty[mi].n && ty[mi].worstExcess <= SLOW_WORST_M,
          "%s: %d of %d over the mark (%.0f %% allowed), worst excess %+.3f m (%.2f allowed): the residual risk "
          "pinned (a2b 25.8)", name, bad, ty[mi].n, SLOW_OVER_FRAC * 100.0, ty[mi].worstExcess, SLOW_WORST_M);
  }
}

/* ---- 4. resting on a surface ------------------------------------------------------------------------------------ */

/* The 10-15 evening flights ended on a surface: the sinking drone arrived, sat at near-hover thrust, and the
 * controller's K_RANGE (Khat at its HOLD floor 0.3 s) landed it 3-8 s later, because the lagging offset read a climb
 * the loop kept answering down (the study, section 4.1 point 4). Here the drone is flown down onto the floor from
 * 60 s by SET_HEIGHTs whose last move ends 2 cm under it (a slow arrival, then no descent commanded), on the
 * synthetic offset and the three real ones; printed: the controller's own LAND, today against the late offset noise
 * (no pass mark: the study expected R1 to make it rarer; the user keeps K_RANGE as it is). */
static void testResting(void)
{
  for (int bi = -2; bi < (nBeta == 3 ? 3 : 0); bi++) {
    const double g = bi == -2 ? 25.0 : (bi == -1 ? 56.0 : betas[bi].g);
    char name[48];
    snprintf(name, sizeof(name), "%s G %.0f", bi < 0 ? "synthetic" : betas[bi].name, g);
    char line[256];
    int k = snprintf(line, sizeof(line), "resting from ~60 s, %-14s:", name);
    for (int mi = 0; mi < 3; mi++) {
      const float mult = mi == 0 ? 1.0f : (mi == 1 ? 3.0f : 5.0f);
      scen_t sc = {mult, 40.0f, g, 0.0, 200.0, 0.0, false, bi >= 0 ? &betas[bi] : NULL, NULL, 100.0, 21u, 56.0, 60.0,
                   SC_PLAIN};
      res_t r;
      fly(&sc, &r);
      k += snprintf(line + k, sizeof(line) - (size_t)k, " x%.0f %s", mult,
                    r.landed ? vzReasonName(r.land) : "no LAND");
      if (r.landed) {
        k += snprintf(line + k, sizeof(line) - (size_t)k, " %.1f s after the floor", r.tLand - r.tFloor);
      }
      k += snprintf(line + k, sizeof(line) - (size_t)k, "%s", r.tFloor > 0.0 ? ";" : " (never on the floor);");
      CHECK(r.hold && r.tFloor > 0.0, "%s x%.0f: flown down onto the floor (%.1f s)", name, mult, r.tFloor);
    }
    printf("%s\n", line);
  }
}

/* ---- 3. bit-identical with the multiplier 1 ------------------------------------------------------------------- */

/* the run's hash with the observer of 65dfcba (-DCVL_BASE) */
#define FLY_HASH_BASE 0x869d48cae62cef73ull

static void testIdentity(void)
{
  scen_t sc = {1.0f, 40.0f, 33.0, 0.3, 60.0, 1.0, true, NULL, NULL, 90.0, 7u, 56.0, 0.0, SC_PLAIN};
  res_t r;
  fly(&sc, &r);
  printf("x1 sortie (G 33, +0.3 m/s for 1 s at 60 s, then lost): hash %016llx (the observer of 65dfcba: %016llx)\n",
         (unsigned long long)r.hash, (unsigned long long)FLY_HASH_BASE);
  CHECK(r.hash == FLY_HASH_BASE, "x1: the controller's every step bit for bit as before");
#ifndef CVL_BASE
  /* with the multiplier on, a sortie shorter than its step is the same */
  scen_t s5 = {5.0f, 40.0f, 33.0, 0.3, 30.0, 1.0, true, NULL, NULL, 38.0, 7u, 25.0, 0.0, SC_PLAIN};
  scen_t s1 = s5;
  s1.mult = 1.0f;
  res_t a, b;
  fly(&s1, &a);
  fly(&s5, &b);
  CHECK(a.hash == b.hash && b.lateS == 0.0f, "x5, a sortie shorter than the step (38 s, the camera lost at 21 s): "
        "bit-identical to x1");
#endif
}

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
#ifndef CVL_BASE
  testSynthetic();
  testSplice(fixture);
  testResting();
  testMotion();
  testSlowError();
#endif
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
