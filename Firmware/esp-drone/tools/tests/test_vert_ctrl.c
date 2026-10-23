/*
 * Host unit tests for components/core/crazyflie/utils/src/vert_ctrl.c
 * (docs/autonomy-a2.md section 9.1)
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_vert_ctrl.c components/core/crazyflie/utils/src/vert_ctrl.c \
 *      -lm -o /tmp/tvc && /tmp/tvc
 *
 * Every sortie runs the controller on the sim plant at 1 kHz, with a small
 * emulation of the state machine's LAND rules that a 1-D sim can trigger
 * (doc 3.2: the air clock from the first RAMP step with its pre-wait and
 * 18 s limits, STABLE 1 s + photos, the hover wait, NOT_STABLE, the camera
 * height rule, free fall), and checks the ceilings on every step. DRY
 * timing waives the camera, as the state machine does in DRY. FLIGHT timing
 * makes STABLE wait for the camera and NOT_STABLE for its 1.5 s view, lands
 * a hover that has no healthy camera height estimate after 2 s, and tells
 * the controller its inputs are real (HOLD_NO_CAM applies). The camera is
 * the sim plant's observer model. tools/tests/test_auto_sortie.c runs the
 * real state machine on the same plant. Lines starting with "info:" are the
 * measured numbers (peak heights, landing times, latencies); they are not
 * checks.
 */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vert_ctrl.h"
#include "vert_ctrl_identity.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define DT 0.001f
#define AUTO_COLLIDE_MS_T     3       /* auto_fsm.h AUTO_COLLIDE_MS: the FSM lands this long after a contact began */
#define TOL_T 0.5f            /* thrust units, float rounding in the checks */

/* The state machine's numbers the emulation uses (auto_fsm.h). */
#define FSM_AIR_MAX_S       6.0f    /* AUTO_AIR_MAX_DEFAULT_MS: air clock until the hover wait */
#define FSM_AIR_HI_S        8.0f    /* AUTO_AIR_MAX_HI_MS: the highest NVS autoair */
#define FSM_AIR_HARD_S      18.0f   /* AUTO_AIR_HARD_MS */
#define FSM_WAIT_S          5.0f    /* AUTO_HOVER_WAIT_DEFAULT_MS */
#define FSM_WAIT_MAX_S      12.0f   /* AUTO_HOVER_WAIT_MAX_MS */
#define FSM_ACCEL_ONLY_S    2.0f    /* AUTO_HOVER_ACCEL_DEFAULT_MS */
#define FSM_VIEW_S          1.5f    /* AUTO_NOT_STABLE_CAM_MS */
/* The evening envelope's top (2026-10-12, 0.60 m: the FSM_* timings above
 * are that evening's); the interactive hover's 1.20 m has its own test
 * (testHigh120) with its own timings. */
#define H_EVENING_MAX       0.60f

/* ---- test RNG (offsets, K errors) ---------------------------------------- */

static uint32_t rngState = 12345u;

static float urand(void)
{
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (float)(rngState >> 8) * (1.0f / 16777216.0f);
}

static float nrand(void)
{
  float s = 0.0f;
  for (int i = 0; i < 12; i++) {
    s += urand();
  }
  return s - 6.0f;
}

/* ---- sortie harness ------------------------------------------------------ */

typedef struct {
  bool fsmLand;          /* emulate the state machine's LAND rules (doc 3.2) */
  bool flight;           /* FLIGHT timing (see the top); else DRY timing (camera waived) */
  float airMaxS;         /* pre-wait limit on the air clock (from the first RAMP step) */
  float airHardS;        /* the air clock never more */
  float stableS;         /* STABLE needed before the capture */
  float photoS;          /* capture of 2 photos 150 ms apart */
  float waitS;           /* hover wait after the photos, then LAND HOVER_TIMEOUT; 0: LAND at once (SORTIE_DONE) */
  float notStableS;      /* NOT_STABLE after HOLD + this (FLIGHT: once the camera has had a 1.5 s view) */
  float accelOnlyS;      /* FLIGHT: HOVER past this without vz camHealthy -> LAND */
  float landAtHoldS;     /* >= 0: LAND command this long after HOLD instead */
  float landAtS;         /* >= 0: LAND command at this sim time */
  bool freefallTrip;     /* FSM: |acc| < 0.4 g for 100 ms -> controller CUT */
  float airAtLandG;      /* finite: sim accAirG becomes this at LAND entry (test 6) */
  uint32_t jitterSeed;   /* 0: dt 1 ms; else 0.6..1.4 ms */
  float maxS;
} harness_t;

/* DRY timing, LAND right after the photos (hover wait 0). */
static harness_t harnessDefault(void)
{
  harness_t h = {
    .fsmLand = true, .flight = false, .airMaxS = FSM_AIR_MAX_S, .airHardS = FSM_AIR_HARD_S, .stableS = 1.0f,
    .photoS = 0.21f, .waitS = 0.0f, .notStableS = 2.0f, .accelOnlyS = FSM_ACCEL_ONLY_S, .landAtHoldS = -1.0f,
    .landAtS = -1.0f, .freefallTrip = true, .airAtLandG = NAN, .jitterSeed = 0, .maxS = 25.0f,
  };
  return h;
}

/* FLIGHT timing with the default hover wait. */
static harness_t harnessFlight(void)
{
  harness_t h = harnessDefault();
  h.flight = true;
  h.waitS = FSM_WAIT_S;
  return h;
}

typedef enum {
  FSM_NONE = 0, FSM_VZ, FSM_AIR_TIME, FSM_SORTIE_DONE, FSM_NOT_STABLE, FSM_CMD, FSM_HOVER_TIMEOUT, FSM_NO_CAM, FSM_N
} fsmWhy_t;
static const char *const fsmWhyName[FSM_N] = { "none", "VZ", "AIR_TIME", "SORTIE_DONE", "NOT_STABLE", "CMD",
                                               "HOVER_TIMEOUT", "HOVER_NO_CAM" };

typedef struct {
  vzOutput_t last;
  fsmWhy_t why;                  /* why the emulated state machine sent LAND */
  float hMax, impactV;
  float hAtEnd;                  /* true height when the controller reached DONE or CUT */
  float liftTrue, liftDet;       /* sim time of the true lift-off / of the detection, -1 if none */
  float hAtDet;                  /* true height at the detection */
  float cmdAtTrueLift, tLo;
  float rampT;                   /* air clock start (first RAMP step), -1 if none */
  float holdT, holdH, holdHMax, holdHMin;  /* HOLD entry time, true height then, extremes in HOLD */
  float hHold1;                  /* true height 1 s after HOLD entry */
  float steadyAfterHold;         /* first holdSteady, s after HOLD entry */
  float waitT;                   /* hover wait start, -1 if none */
  float landCmdT, airAtLandCmd, trueAirAtLandCmd, hAtLandCmd;  /* when the FSM sent LAND, -1 if never */
  float landT0, landS, landEntryH;
  float tdOkAfter;               /* first tdThrustOk, s after LAND entry */
  float maxOverLo;               /* max thrust / T_lo in CLIMB, / (T_lo max(V_lo, Vf) / Vf) in HOLD */
  float maxOverTrueLift;         /* max thrust / commanded thrust at the true lift-off, CLIMB and HOLD */
  float maxRampOverPlateau;      /* max RAMP thrust / (rampTopRel kPrior / Vf) */
  float tLoOverBound;            /* T_lo / (tLoMaxRel kPrior / Vf) at the detection */
  float maxThrust;
  float landAboveGround;         /* s of LAND with the sim above 5 cm and tdThrustOk set (detector armed) */
  float camHoldS;                /* s of HOLD with the external estimate in use */
  float airS;                    /* true airborne time (sim off the ground) */
  float tdTrueH;                 /* true height when touchdown was declared, -1 if none */
  float landMaxRise;             /* largest step-to-step thrust rise in LAND (below the ceiling) */
  float abortMaxDrop;            /* fastest thrust fall in the abort ramp, per s */
  bool contactInLand;            /* the sim touched the ground during LAND */
  float contactT;                /* sim time of the first ground contact in LAND, -1 if none */
  float doneAfterContact;        /* s from that contact to DONE or CUT, -1 if either is missing */
  bool landSlow;                 /* LAND started at landTop (from HOLD with the camera estimate in use) */
  bool freefall;                 /* the emulated FSM free-fall trip fired */
  int violations;                /* broken invariants (first one printed) */
  int tdHigh;                    /* touchdowns declared with the sim above 5 cm */
} run_t;

static void violation(run_t *r, uint32_t seed, float t, const char *what, float a, float b)
{
  if (r->violations++ == 0) {
    printf("  invariant broken (seed %u, t %.3f): %s (%.2f vs %.2f)\n", (unsigned)seed, t, what, a, b);
  }
}

static void runSortie(const vzConfig_t *cfg, const vzSimConfig_t *simCfg, uint32_t seed, const harness_t *hp,
                      run_t *r)
{
  vzSimConfig_t sc = *simCfg;
  vzState_t st;
  vzSimState_t sim;
  vzOutput_t out;
  memset(&out, 0, sizeof(out));
  memset(r, 0, sizeof(*r));
  r->liftTrue = r->liftDet = r->holdT = r->landT0 = r->tdOkAfter = r->steadyAfterHold = r->tdTrueH = -1.0f;
  r->rampT = r->waitT = r->landCmdT = r->hHold1 = -1.0f;
  r->contactT = r->doneAfterContact = -1.0f;
  r->holdHMin = 9.0f;
  vzInit(&st, cfg);
  vzSimInit(&sim, &sc, seed);

  float t = 0.0f, applied = 0.0f, vbat = sim.vbat, vbatNext = 0.0f, vfT = -1.0f, vLoT = 0.0f, view = 0.0f;
  float stable = 0.0f, captureT = -1.0f, ff = 0.0f, endT = -1.0f, prevCeil = 0.0f, prevThrust = 0.0f;
  bool landCmd = false, cut = false, inLand = false, ended = false;
  uint32_t jr = hp->jitterSeed;

  while (t < hp->maxS) {
    float dt = DT;
    if (jr != 0) {
      jr ^= jr << 13;
      jr ^= jr >> 17;
      jr ^= jr << 5;
      dt = DT * (0.6f + 0.8f * (float)(jr >> 8) / 16777216.0f);
    }
    if (t >= vbatNext) {   /* the battery is read at 10 Hz */
      vbat = sim.vbat;
      vbatNext += 0.1f;
    }
    const float hTrue = sim.h;
    const bool tdBefore = out.touchdown;
    const vzPhase_t phaseBefore = out.phase;
    vzInput_t in = {
      .dt = dt, .cmd = cut ? VZ_CMD_CUT : (landCmd ? VZ_CMD_LAND : VZ_CMD_TAKEOFF),
      .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = 0.0f, .gyroMaxDps = 0.5f, .vbat = vbat,
      .thrustApplied = applied, .fromSim = !hp->flight,
      .camHealthy = sim.camHealthy, .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias,
      .landFastOk = true,   /* the Mac's LAND (the state machine's allow list) */
    };
    vzStep(&st, &in, &out);
    vfT = vfT < 0.0f ? fminf(fmaxf(vbat, 2.0f), 5.0f) : vfT + (vbat - vfT) * dt / (cfg->vfTauS + dt);
    const float tLoBound = cfg->tLoMaxRel * cfg->kPrior / vfT;
    const float rampTop = cfg->rampTopRel * cfg->kPrior / vfT;
    if (out.liftoffNow) {
      vLoT = vfT;   /* the controller's V_lo: its Vf at the lift-off */
    }

    /* Invariants on every step. */
    if (!(out.thrust >= 0.0f && out.thrust <= out.ceiling + 1e-3f && out.ceiling <= VZ_THRUST_ABS_MAX)) {
      violation(r, seed, t, "0 <= thrust <= ceiling <= VZ_THRUST_ABS_MAX (58000)", out.thrust, out.ceiling);
    }
    if (out.phase == VZ_RAMP) {
      r->maxRampOverPlateau = fmaxf(r->maxRampOverPlateau, out.thrust / rampTop);
      if (out.thrust > rampTop * 1.002f + TOL_T) {
        violation(r, seed, t, "RAMP thrust <= rampTopRel x kPrior / Vf", out.thrust, rampTop);
      }
    }
    if (out.phase == VZ_CLIMB || out.phase == VZ_HOLD) {
      /* CLIMB: capLo x T_lo; HOLD: capLo x T_lo x max(V_lo, Vf) / Vf (battery-scaled, 2026-10-14) */
      const bool hold = out.phase == VZ_HOLD;
      const float loScale = hold ? fmaxf(vLoT / vfT, 1.0f) : 1.0f;
      if (out.thrust > cfg->capLo * out.tLo * loScale * (hold ? 1.002f : 1.0f) + TOL_T) {
        violation(r, seed, t,
                  hold ? "HOLD thrust <= capLo x T_lo x max(V_lo, Vf) / Vf" : "CLIMB thrust <= capLo x T_lo",
                  out.thrust, cfg->capLo * out.tLo * loScale);
      }
      if (out.thrust > cfg->capKhat * out.khat / vfT * 1.002f + TOL_T) {
        violation(r, seed, t, "thrust <= capKhat x Khat / Vf", out.thrust, cfg->capKhat * out.khat / vfT);
      }
      if (out.tLo > 0.0f) {
        r->maxOverLo = fmaxf(r->maxOverLo, out.thrust / (out.tLo * loScale));
      }
      if (r->cmdAtTrueLift > 0.0f) {
        r->maxOverTrueLift = fmaxf(r->maxOverTrueLift, out.thrust / r->cmdAtTrueLift);
      }
    }
    if ((out.phase == VZ_IDLE || out.phase == VZ_DONE || out.phase == VZ_CUT) && out.thrust != 0.0f) {
      violation(r, seed, t, "thrust 0 in IDLE/DONE/CUT", out.thrust, 0.0f);
    }
    if (out.camHealthy && !out.liftedOff) {
      violation(r, seed, t, "the external estimate is not used before lift-off", 1.0f, 0.0f);
    }
    r->maxThrust = fmaxf(r->maxThrust, out.thrust);

    /* Events. */
    if (r->rampT < 0.0f && (out.phase == VZ_RAMP || out.phase == VZ_CLIMB || out.phase == VZ_HOLD)) {
      r->rampT = t;
    }
    if (out.liftoffNow) {
      r->liftDet = t;
      r->tLo = out.tLo;
      r->hAtDet = hTrue;
      r->tLoOverBound = out.tLo / tLoBound;
      if (out.tLo > tLoBound * 1.002f + TOL_T) {
        violation(r, seed, t, "T_lo <= tLoMaxRel x kPrior / Vf", out.tLo, tLoBound);
      }
    }
    if (out.phase == VZ_HOLD && r->holdT < 0.0f) {
      r->holdT = t;
      r->holdH = hTrue;
    }
    if (out.phase == VZ_HOLD) {
      r->holdHMax = fmaxf(r->holdHMax, hTrue);
      r->holdHMin = fminf(r->holdHMin, hTrue);
      if (out.holdSteady && r->steadyAfterHold < 0.0f) {
        r->steadyAfterHold = t - r->holdT;
      }
      if (r->hHold1 < 0.0f && t - r->holdT >= 1.0f) {
        r->hHold1 = hTrue;
      }
      if (out.camHealthy) {
        r->camHoldS += dt;
      }
    }
    if (out.phase == VZ_LAND && !out.liftedOff && phaseBefore == VZ_LAND) {
      r->abortMaxDrop = fmaxf(r->abortMaxDrop, (prevThrust - out.thrust) / dt);
    }
    if (out.phase == VZ_LAND && out.liftedOff) {
      if (!inLand) {
        inLand = true;
        r->landT0 = t;
        r->landEntryH = hTrue;
        prevCeil = out.ceiling;
        if (isfinite(hp->airAtLandG)) {
          sc.accAirG = hp->airAtLandG;
        }
      }
      const float tl = out.tPhase;
      if (tl <= 0.0015f) {
        r->landSlow = out.ceiling > out.tRef * (cfg->landStart + 1e-3f) + TOL_T;   /* from HOLD with the camera: landTop */
      }
      const float top = r->landSlow ? cfg->landTop : cfg->landStart;
      const float flat = r->landSlow ? cfg->landFlatS : 0.0f;
      const float ramp = out.tRef * fmaxf(cfg->landFloor, top - cfg->landRate * fmaxf(0.0f, tl - flat));
      if (out.ceiling > prevCeil + 1e-3f) {
        violation(r, seed, t, "LAND ceiling never rises", out.ceiling, prevCeil);
      }
      if (out.thrust > ramp + TOL_T) {
        violation(r, seed, t, "LAND thrust <= T_ref max(floor, start - rate t)", out.thrust, ramp);
      }
      if (out.thrust > out.tRef * fmaxf(cfg->landStart, cfg->landTop) + TOL_T) {
        violation(r, seed, t, "LAND thrust <= max(landStart, landTop) x T_ref", out.thrust, out.tRef * top);
      }
      if (phaseBefore == VZ_LAND) {
        r->landMaxRise = fmaxf(r->landMaxRise, out.thrust - prevThrust);
      }
      prevCeil = out.ceiling;
      if (out.tdThrustOk && r->tdOkAfter < 0.0f) {
        r->tdOkAfter = t - r->landT0;
      }
      if (out.tdThrustOk && hTrue > 0.05f) {
        r->landAboveGround += dt;
      }
      if (sim.onGround) {
        if (!r->contactInLand) {
          r->contactT = t;
        }
        r->contactInLand = true;
      }
    }
    if (out.touchdown && !tdBefore) {
      r->tdTrueH = hTrue;
      if (hTrue > 0.05f) {
        r->tdHigh++;
      }
    }
    if ((out.phase == VZ_DONE || out.phase == VZ_CUT) && !ended) {
      ended = true;
      endT = t;
      r->hAtEnd = hTrue;
      if (inLand) {
        r->landS = t - r->landT0;
      }
    }
    prevThrust = out.thrust;

    /* Plant. */
    const bool groundBefore = sim.liftT < 0.0f;
    vzSimStep(&sim, &sc, out.thrust, dt);
    if (groundBefore && sim.liftT >= 0.0f) {
      r->liftTrue = sim.liftT;
      r->cmdAtTrueLift = out.thrust;
    }
    if (!sim.onGround) {
      r->airS += dt;
    }
    applied = out.thrust;
    t += dt;

    /* Emulated state machine (doc 3.2, rows in its order). */
    view = sim.camAwake ? view + dt : 0.0f;
    if (hp->freefallTrip && out.liftedOff && !ended && fabsf(sim.accZ) < 0.4f) {
      ff += dt;
      if (ff >= 0.1f && !cut) {
        cut = true;   /* KILLED (in LAND a free fall is never a touchdown): both send CUT */
        r->freefall = true;
      }
    } else {
      ff = 0.0f;
    }
    if (!landCmd && !cut) {
      fsmWhy_t why = FSM_NONE;
      const float air = r->rampT >= 0.0f ? t - r->rampT : 0.0f;
      if (out.phase == VZ_LAND || out.phase == VZ_DONE || out.phase == VZ_CUT) {
        why = FSM_VZ;   /* the controller landed on its own (row 9) */
      } else if (hp->landAtS >= 0.0f && t >= hp->landAtS) {
        why = FSM_CMD;
      } else if (hp->fsmLand && r->rampT >= 0.0f && (air >= hp->airHardS || (r->waitT < 0.0f && air >= hp->airMaxS))) {
        why = FSM_AIR_TIME;
      } else if (r->holdT >= 0.0f) {
        const float inHold = t - r->holdT;
        if (hp->landAtHoldS >= 0.0f) {
          why = inHold >= hp->landAtHoldS ? FSM_CMD : FSM_NONE;
        } else if (hp->fsmLand) {
          if (captureT < 0.0f) {
            const bool s = out.holdSteady && (!hp->flight || sim.camAwake);
            stable = s ? stable + dt : 0.0f;
          }
          if (hp->flight && inHold >= hp->accelOnlyS && !out.camHealthy) {
            why = FSM_NO_CAM;
          } else if (r->waitT >= 0.0f && t - r->waitT >= hp->waitS) {
            why = FSM_HOVER_TIMEOUT;
          } else if (captureT < 0.0f && inHold >= hp->notStableS && (!hp->flight || view >= FSM_VIEW_S)) {
            why = FSM_NOT_STABLE;
          } else if (captureT >= 0.0f && r->waitT < 0.0f && t - captureT >= hp->photoS) {
            if (hp->waitS > 0.0f) {
              r->waitT = t;
            } else {
              why = FSM_SORTIE_DONE;
            }
          } else if (captureT < 0.0f && stable >= hp->stableS) {
            captureT = t;
          }
        }
      }
      if (why != FSM_NONE) {
        landCmd = true;
        r->why = why;
        r->landCmdT = t;
        r->airAtLandCmd = air;
        r->trueAirAtLandCmd = r->liftTrue >= 0.0f ? t - r->liftTrue : 0.0f;
        r->hAtLandCmd = sim.h;
      }
    }
    if (ended && t - endT > 0.5f) {
      break;
    }
  }
  r->last = out;
  r->hMax = sim.hMax;
  r->impactV = sim.impactV;
  if (r->contactT >= 0.0f && endT >= 0.0f) {
    r->doneAfterContact = endT - r->contactT;
  }
}

static bool endedOk(const run_t *r)
{
  return (r->last.phase == VZ_DONE && (r->last.touchdown || r->last.reason == VZ_R_ABORTED)) ||
         (r->last.phase == VZ_CUT && r->last.reason == VZ_R_LAND_TIMEOUT);
}

static int cmpf(const void *a, const void *b)
{
  const float x = *(const float *)a, y = *(const float *)b;
  return x < y ? -1 : (x > y ? 1 : 0);
}

static float pct(float *v, int n, float p)
{
  qsort(v, (size_t)n, sizeof(float), cmpf);
  int i = (int)(p * (float)(n - 1) + 0.5f);
  return v[i < 0 ? 0 : (i >= n ? n - 1 : i)];
}

/* ---- Monte Carlo --------------------------------------------------------- */

typedef struct {
  int n, ended, cut, aborted, held, waited, freefall, violations, tdHigh;
  int why[FSM_N];
  float worstCutH;
  int landReasons[32];
  float worstH, worstImpact, worstLandS, worstAirS, worstOverLo, worstOverTrueLift;
  float worstBias, worstKerr, worstCamC, landMaxRise;
  float worstAirClock, worstTrueAirAtCmd, worstTLoOverBound, worstRampOverPlateau;
  float p50H, p95H, p50Imp, p95Imp, p50Land, p95Land;
  float armedAirS, camHoldS;
  /* The corner pass, in its own maxima and endings (ended, cut, violations,
   * tdHigh, free-fall trips, the LAND time, the ceilings and the air clock
   * above include it). */
  int cornerN, cornerWaited, cornerAborted;
  int cornerWhy[FSM_N];
  float cornerH, cornerImpact, cornerBias, cornerKerr, cornerCamC;
} mc_t;

/* One draw: residual in-flight offset after the controller's bAir (g), thrust
 * gain error, the camera's velocity offset c (m/s) and its offset error (g). */
typedef struct {
  float bias, kerr, camC, camB;
} draw_t;
typedef void (*mcDraw_t)(int i, int n, draw_t *d);
/* Corner j (0..7) of a range, noise seed k (0..MC_CORNER_SEEDS-1). */
typedef void (*mcCorner_t)(int j, int k, draw_t *d);

#define MC_CORNER_SEEDS 50

/* One sortie into the row's numbers: a corner into the corner maxima and
 * endings, a uniform draw into the others; the invariants take both. */
static void mcAdd(mc_t *m, const run_t *r, const draw_t *d, bool corner)
{
  m->ended += endedOk(r);
  m->cut += r->last.phase == VZ_CUT;
  m->freefall += r->freefall;
  if (r->last.phase == VZ_CUT) {
    m->worstCutH = fmaxf(m->worstCutH, r->hAtEnd);
  }
  m->violations += r->violations;
  m->tdHigh += r->tdHigh;
  m->worstAirS = fmaxf(m->worstAirS, r->airS);
  m->worstOverLo = fmaxf(m->worstOverLo, r->maxOverLo);
  m->worstOverTrueLift = fmaxf(m->worstOverTrueLift, r->maxOverTrueLift);
  m->worstTLoOverBound = fmaxf(m->worstTLoOverBound, r->tLoOverBound);
  m->worstRampOverPlateau = fmaxf(m->worstRampOverPlateau, r->maxRampOverPlateau);
  m->landMaxRise = fmaxf(m->landMaxRise, r->landMaxRise);
  if (r->landCmdT >= 0.0f) {
    m->worstAirClock = fmaxf(m->worstAirClock, r->airAtLandCmd);
    m->worstTrueAirAtCmd = fmaxf(m->worstTrueAirAtCmd, r->trueAirAtLandCmd);
  }
  if (r->landT0 >= 0.0f) {
    m->worstLandS = fmaxf(m->worstLandS, r->landS);
  }
  if (corner) {
    m->cornerN++;
    m->cornerWaited += r->waitT >= 0.0f;
    m->cornerAborted += r->last.reason == VZ_R_ABORTED;
    m->cornerWhy[r->why]++;
    if (r->hMax > m->cornerH) {
      m->cornerH = r->hMax;
      m->cornerBias = d->bias;
      m->cornerKerr = d->kerr;
      m->cornerCamC = d->camC;
    }
    m->cornerImpact = fmaxf(m->cornerImpact, r->impactV);
    return;
  }
  m->aborted += r->last.reason == VZ_R_ABORTED;
  m->held += r->holdT >= 0.0f;
  m->waited += r->waitT >= 0.0f;
  m->why[r->why]++;
  m->armedAirS += r->landAboveGround;
  m->camHoldS += r->camHoldS;
  m->landReasons[r->last.landReason & 31]++;
  if (r->hMax > m->worstH) {
    m->worstH = r->hMax;
    m->worstBias = d->bias;
    m->worstKerr = d->kerr;
    m->worstCamC = d->camC;
  }
  m->worstImpact = fmaxf(m->worstImpact, r->impactV);
}

static void mcSortie(const vzConfig_t *cfg, const harness_t *hp, const vzSimConfig_t *base, const draw_t *d,
                     uint32_t seed, run_t *r)
{
  vzSimConfig_t sc = *base;
  sc.accAirG = cfg->bAir + d->bias;      /* residual after the controller's bAir is 'bias' */
  sc.kTrue = cfg->kPrior * (1.0f + d->kerr);
  sc.camVzOffset = d->camC;
  sc.camBiasErrG = d->camB;
  runSortie(cfg, &sc, seed, hp, r);
}

/* n uniform draws (the percentiles and the uniform maxima), then, with a
 * corner function, the range's 8 corners with MC_CORNER_SEEDS seeds each
 * (seeds seed0 + 1000 + 100 j + k). */
static void runMc(const char *label, const vzConfig_t *cfg, const harness_t *hp, const vzSimConfig_t *base, int n,
                  mcDraw_t draw, mcCorner_t corner, uint32_t seed0, mc_t *m)
{
  float *hs = malloc(sizeof(float) * (size_t)n), *im = malloc(sizeof(float) * (size_t)n);
  float *ls = malloc(sizeof(float) * (size_t)n);
  int nl = 0;
  memset(m, 0, sizeof(*m));
  m->n = n;
  for (int i = 0; i < n; i++) {
    draw_t d;
    draw(i, n, &d);
    run_t r;
    mcSortie(cfg, hp, base, &d, seed0 + (uint32_t)i, &r);
    mcAdd(m, &r, &d, false);
    hs[i] = r.hMax;
    im[i] = r.impactV;
    if (r.landT0 >= 0.0f) {
      ls[nl++] = r.landS;
    }
  }
  for (int j = 0; corner != NULL && j < 8; j++) {
    for (int k = 0; k < MC_CORNER_SEEDS; k++) {
      draw_t d;
      corner(j, k, &d);
      run_t r;
      mcSortie(cfg, hp, base, &d, seed0 + 1000u + 100u * (uint32_t)j + (uint32_t)k, &r);
      mcAdd(m, &r, &d, true);
    }
  }
  if (corner != NULL) {
    CHECK(m->cornerN == 8 * MC_CORNER_SEEDS, "the corner pass ran (%d of %d)", m->cornerN, 8 * MC_CORNER_SEEDS);
  }
  m->p50H = pct(hs, n, 0.5f);
  m->p95H = pct(hs, n, 0.95f);
  m->p50Imp = pct(im, n, 0.5f);
  m->p95Imp = pct(im, n, 0.95f);
  m->p50Land = nl > 0 ? pct(ls, nl, 0.5f) : 0.0f;
  m->p95Land = nl > 0 ? pct(ls, nl, 0.95f) : 0.0f;
  free(hs);
  free(im);
  free(ls);

  printf("info: %s: n %d, aborted %d, HOLD %d, hover wait %d; %sended %d/%d (6 s cut %d at true h <= %.2f m), free-fall trips %d\n",
         label, n, m->aborted, m->held, m->waited, m->cornerN > 0 ? "with the corners: " : "", m->ended,
         m->n + m->cornerN, m->cut, m->worstCutH, m->freefall);
  printf("info:   the FSM landed by:");
  for (int k = 1; k < FSM_N; k++) {
    if (m->why[k] > 0) {
      printf(" %s %d", fsmWhyName[k], m->why[k]);
    }
  }
  printf("\n");
  printf("info:   true peak h p50 %.2f p95 %.2f max %.2f m (bias %+.1f mg, K %+.0f %%, c %+.3f m/s); impact p50 %.2f p95 %.2f max %.2f m/s\n",
         m->p50H, m->p95H, m->worstH, m->worstBias * 1000.0f, m->worstKerr * 100.0f, m->worstCamC, m->p50Imp,
         m->p95Imp, m->worstImpact);
  printf("info:   LAND (to touchdown) p50 %.2f p95 %.2f max %.2f s; at the FSM's LAND: air clock max %.2f s, true airborne max %.2f s; true airborne in all max %.2f s\n",
         m->p50Land, m->p95Land, m->worstLandS, m->worstAirClock, m->worstTrueAirAtCmd, m->worstAirS);
  printf("info:   climb/hold thrust max %.3f x T_lo (HOLD: x max(V_lo, Vf) / Vf), %.3f x thrust at true lift-off; T_lo max %.3f x its bound; RAMP max %.3f x its plateau; armed LAND above 5 cm %.1f s; HOLD on the camera %.1f s\n",
         m->worstOverLo, m->worstOverTrueLift, m->worstTLoOverBound, m->worstRampOverPlateau, m->armedAirS,
         m->camHoldS);
  printf("info:   controller land reasons:");
  for (int k = 0; k < 32; k++) {
    if (m->landReasons[k] > 0) {
      printf(" %s %d", vzReasonName((vzReason_t)k), m->landReasons[k]);
    }
  }
  printf("\n");
  if (m->cornerN > 0) {
    printf("info:   corners (8 x %d seeds): true peak max %.2f m (bias %+.1f mg, K %+.0f %%, c %+.3f m/s), impact max %.2f m/s; hover wait %d, aborted %d; the FSM landed by:",
           MC_CORNER_SEEDS, m->cornerH, m->cornerBias * 1000.0f, m->cornerKerr * 100.0f, m->cornerCamC,
           m->cornerImpact, m->cornerWaited, m->cornerAborted);
    for (int k = 1; k < FSM_N; k++) {
      if (m->cornerWhy[k] > 0) {
        printf(" %s %d", fsmWhyName[k], m->cornerWhy[k]);
      }
    }
    printf("\n");
  }
}

/* Every row's invariants: ceilings, endings, touchdown, LAND time, T_lo. */
static void checkMc(const char *what, const mc_t *m, float capLo)
{
  CHECK(m->violations == 0, "%s: every ceiling held on every step (%d)", what, m->violations);
  CHECK(m->ended == m->n + m->cornerN, "%s: LAND always ends in touchdown, abort or the 6 s cut (%d/%d)", what,
        m->ended, m->n + m->cornerN);
  CHECK(m->tdHigh == 0, "%s: no touchdown declared above 0.05 m (%d)", what, m->tdHigh);
  CHECK(m->worstLandS <= VZ_LAND_MAX_S + 0.002f, "%s: LAND never longer than 6 s (%.2f)", what, m->worstLandS);
  CHECK(m->worstOverLo <= capLo + 1e-3f, "%s: climb thrust <= capLo x T_lo, hold <= capLo x T_lo x max(V_lo, Vf) / Vf "
        "(%.4f x)", what, m->worstOverLo);
  CHECK(m->worstAirClock <= FSM_AIR_HARD_S + 0.002f, "%s: the FSM's LAND by the 18 s air clock (%.2f s)", what,
        m->worstAirClock);
}

/* Uniform in the user range's box (the corners have their own pass). */
static void drawUserRange(int i, int n, draw_t *d)
{
  (void)i;
  (void)n;
  d->bias = -0.020f + 0.040f * urand();
  d->kerr = -0.12f + 0.24f * urand();
  d->camC = -0.05f + 0.10f * urand();
  d->camB = -0.002f + 0.004f * urand();
}

/* Corner j of the user range: bit 0 the offset (+/-20 mg), bit 1 the K error
 * (+/-12 %), bit 2 c (+/-0.05 m/s); the camera's offset error +/-2 mg
 * alternates with the seed. */
static void cornerUserRange(int j, int k, draw_t *d)
{
  d->bias = (j & 1) ? 0.020f : -0.020f;
  d->kerr = (j & 2) ? 0.12f : -0.12f;
  d->camC = (j & 4) ? 0.05f : -0.05f;
  d->camB = (k & 1) ? 0.002f : -0.002f;
}

/* The same with the camera offset c doubled (+/-0.10 m/s, the study's
 * older flights). */
static void drawUserRangeC10(int i, int n, draw_t *d)
{
  drawUserRange(i, n, d);
  d->camC *= 2.0f;
}

static void cornerUserRangeC10(int j, int k, draw_t *d)
{
  cornerUserRange(j, k, d);
  d->camC *= 2.0f;
}

static void drawSpec5(int i, int n, draw_t *d)
{
  (void)i;
  (void)n;
  d->bias = 0.010f * nrand();
  d->kerr = -0.08f + 0.16f * urand();
  d->camC = 0.0f;
  d->camB = 0.0f;
}

/* ---- tests --------------------------------------------------------------- */

/* "No offset" as in the spec's prototype: the sim reads the true specific
 * force in the air and the controller subtracts no in-flight offset. */
static void noOffset(vzConfig_t *c, vzSimConfig_t *sc)
{
  c->bAir = 0.0f;
  sc->accAirG = 0.0f;
}

/* The camera observer as the drone's warm camera gives it: first frame 1.5 s
 * into the take-off window, healthy 0.3 s after the later of that and the
 * lift-off. */
static void warmCamera(vzSimConfig_t *sc)
{
  sc->camWakeS = 1.5f;
}

static void testDefaults(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  CHECK(vzConfigValid(&c), "the default config is valid");
  CHECK(c.capLo == 1.65f && c.capKhat == 1.15f && c.tLoMaxRel == 0.90f && c.rampTopRel == 1.00f &&
        c.landStart == 0.95f &&
        c.landRate == 0.10f && c.landFloor == 0.65f && c.tdThrust == 0.75f && c.tdForce == 1.3f &&
        c.tdForceS == 0.2f, "defaults carry the lead's caps and the spec's LAND/touchdown rules");
  CHECK(c.holdMaxS == 599.0f && VZ_HOLD_MAX_LIMIT_S == 600.0f && c.holdNoCamS == 7.0f && VZ_HOLD_NO_CAM_MAX_S == 8.0f &&
        c.camTauS == 0.2f,
        "hold backstops 599 s (no air clock since 2026-10-15, docs/autonomy-a2b.md 22: no battery reaches it; 119 under "
        "the 120 s air clock before) and 7 s since the camera estimate was last used (2 s behind the FSM's SINK; 3 s "
        "of HOLD before), camera time constant 0.2 s");
  CHECK(c.hMax == 1.0f && c.hMin == -0.3f && c.climbMaxS == 2.0f && c.landFlatS == 1.5f && c.hTarget == 0.30f,
        "estimate guard hMax 1.0 m, climbMaxS 2.0 s and landFlatS 1.5 s up to 0.40 m (vzConfigSetHeight raises them "
        "above), default target 0.30 m");
  CHECK(c.kPrior == 165000.0f && c.rampRate == 13250.0f && c.landClimbRefRel == 1.00f && c.abortSlowRel == 0.60f,
        "A2b: kPrior 165000, rampRate 13250/s (12000 x 165000 / 149500), landClimbRefRel 1.00, abortSlowRel 0.60");
  CHECK(c.landVfast == -0.50f && c.landFastH == 0.60f && c.landFastBand == 0.25f && c.landFastMinH == 1.00f &&
        c.landBrakeTop == 1.03f && c.landBrakeH == 0.40f && c.landBrakeMaxS == 0.5f && c.vClimbMax == 0.40f &&
        c.aClimb == 1.0f && VZ_LAND_FAST_TREF_MAX == 53400.0f && VZ_LAND_FAST_VMIN == 3.15f,
        "the fast LAND (2026-10-14): -0.50 m/s above 0.85 m, back to -0.30 by 0.60 m, from 1.00 m, a 1.03 brake in the "
        "band 0.85-0.40 m for 0.5 s, T_ref <= 53400, Vf >= 3.15 V; the climb unchanged at 0.40 m/s");
  vzState_t st;
  vzInit(&st, &c);
  vzOutput_t o;
  vzInput_t in = { .dt = DT, .cmd = VZ_CMD_NONE, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.9f };
  vzStep(&st, &in, &o);
  CHECK(o.phase == VZ_IDLE && o.thrust == 0.0f && o.g0 == 1.0f, "IDLE with NONE: thrust 0, g0 1.0 until captured");
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  CHECK(sc.camWakeS < 0.0f, "the sim plant has no camera unless asked");
}

/* 9.1 #1: a 1.10 cap (the user's first choice) cannot leave ground effect
 * (open issue 1); it stays a valid setting. */
static void testCapLo110(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  c.capLo = 1.10f;
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  harness_t hp = harnessDefault();
  int capped = 0, touchdown = 0, low = 0, cutN = 0, viol = 0;
  float worst = 0.0f;
  const int n = 50;
  for (int i = 0; i < n; i++) {
    run_t r;
    runSortie(&c, &sc, 100u + (uint32_t)i, &hp, &r);
    capped += r.last.landReason == VZ_R_CLIMB_CAPPED;
    touchdown += r.last.phase == VZ_DONE && r.last.touchdown;
    low += r.hMax <= 0.08f;
    cutN += r.last.phase == VZ_CUT;
    viol += r.violations;
    worst = fmaxf(worst, r.hMax);
  }
  CHECK(capped == n, "capLo 1.10, no offset, K exact: LAND CLIMB_CAPPED in every seed (%d/%d)", capped, n);
  CHECK(touchdown == n && cutN == 0, "and DONE by touchdown, never CUT (%d/%d, cut %d)", touchdown, n, cutN);
  CHECK(low == n, "true max height <= 0.08 m (%d/%d, worst %.3f m)", low, n, worst);
  CHECK(viol == 0, "no ceiling broken (%d)", viol);
  printf("info: capLo 1.10 nominal: %d/%d CLIMB_CAPPED, worst true height %.3f m\n", capped, n, worst);
}

/* 9.1 #2: with the default climb cap (1.65), which can leave ground effect. */
static void testCapLoDefault(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  harness_t hp = harnessDefault();
  const int n = 50;
  int holdOk = 0, entryOk = 0, wideOk = 0, steadyOk = 0, landCmd = 0, tdOk = 0, impactOk = 0, viol = 0;
  float worstLand = 0.0f, worstImpact = 0.0f, hLo = 9.0f, hHi = 0.0f, worstSteady = 0.0f;
  for (int i = 0; i < n; i++) {
    run_t r;
    runSortie(&c, &sc, 200u + (uint32_t)i, &hp, &r);
    holdOk += r.holdT >= 0.0f && fabsf(r.holdH - 0.30f) <= 0.10f && fabsf(r.holdHMax - 0.30f) <= 0.10f;
    entryOk += r.holdT >= 0.0f && fabsf(r.holdH - 0.30f) <= 0.10f;
    wideOk += r.holdT >= 0.0f && fabsf(r.holdH - 0.30f) <= 0.15f && fabsf(r.holdHMax - 0.30f) <= 0.15f;
    steadyOk += r.steadyAfterHold >= 0.0f && r.steadyAfterHold <= 1.5f;
    landCmd += r.last.landReason == VZ_R_CMD_LAND;
    tdOk += r.last.touchdown && r.landS <= 3.0f &&
            (r.last.reason == VZ_R_TOUCHDOWN_FORCE || r.last.reason == VZ_R_TOUCHDOWN_JOLT);
    impactOk += r.impactV <= 0.8f;
    viol += r.violations;
    worstLand = fmaxf(worstLand, r.landS);
    worstImpact = fmaxf(worstImpact, r.impactV);
    worstSteady = fmaxf(worstSteady, r.steadyAfterHold);
    hLo = fminf(hLo, r.holdH);
    hHi = fmaxf(hHi, r.holdHMax);
  }
  /* The spec's +/-0.10 m is for one run. Over 50 seeds the g0 noise alone
   * (30 mg per sample over the 0.5 s window: 1.3 mg rms, ~5 cm per mg over
   * the sortie) puts about one seed in 50 just outside it. */
  CHECK(entryOk >= n - n / 20, "capLo 1.65: reaches HOLD with true h within 0.30 +/- 0.10 m in >= 95 %% (%d/%d)",
        entryOk, n);
  CHECK(holdOk >= n - n / 20, "and stays within it through HOLD in >= 95 %% (%d/%d)", holdOk, n);
  CHECK(wideOk == n, "every seed within 0.30 +/- 0.15 m from HOLD entry to LAND (%d/%d, %.3f..%.3f)", wideOk, n, hLo,
        hHi);
  CHECK(steadyOk == n, "holdSteady within 1.5 s of HOLD (%d/%d, worst %.2f s)", steadyOk, n, worstSteady);
  CHECK(landCmd == n, "LAND on command (%d/%d)", landCmd, n);
  CHECK(tdOk == n, "touchdown by force or jolt within 3.0 s (%d/%d, worst %.2f s)", tdOk, n, worstLand);
  CHECK(impactOk == n, "impact <= 0.8 m/s (%d/%d, worst %.2f)", impactOk, n, worstImpact);
  CHECK(viol == 0, "no ceiling broken (%d)", viol);
  printf("info: capLo 1.65 nominal: HOLD true h %.3f..%.3f m, steady after %.2f s max, LAND max %.2f s, impact max %.2f m/s\n",
         hLo, hHi, worstSteady, worstLand, worstImpact);
}

/* The user's target range, 0.20 m, the old maximum 0.40 m and the
 * maximum 0.60 m (user decision 2026-10-12, evening), each set as the glue
 * does it (vzConfigSetHeight): hold height, photos within the pre-wait
 * airborne limit, landing. DRY timing, no camera: the accelerometer alone,
 * whose height error and estimate-independent LAND grow with the height
 * (0.60 m gets its own, measured bounds); then FLIGHT timing with the warm
 * camera and the 10 s wait the user flies at 0.60 m. */
static void testHeightRange(void)
{
  const float hts[] = { VZ_H_TARGET_MIN, VZ_H_TIMING_REF, H_EVENING_MAX };
  const float band[] = { 0.10f, 0.10f, 0.15f };     /* HOLD true h within hTarget +/- this in >= 95 % */
  const float impLim[] = { 1.0f, 1.0f, 1.25f };
  for (int j = 0; j < 3; j++) {
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, hts[j]);
    vzSimConfig_t sc;
    vzSimDefaultConfig(&sc);
    noOffset(&c, &sc);
    harness_t hp = harnessDefault();
    const int n = 50;
    int inBand = 0, td = 0, photos = 0, viol = 0;
    float lo = 9.0f, hi = 0.0f, imp = 0.0f;
    for (int i = 0; i < n; i++) {
      run_t r;
      runSortie(&c, &sc, 1200u + (uint32_t)i, &hp, &r);
      inBand += r.holdT >= 0.0f && fabsf(r.holdH - hts[j]) <= band[j] && fabsf(r.holdHMax - hts[j]) <= band[j];
      td += r.last.touchdown;
      photos += r.why == FSM_SORTIE_DONE;
      viol += r.violations;
      lo = fminf(lo, r.holdH);
      hi = fmaxf(hi, r.holdHMax);
      imp = fmaxf(imp, r.impactV);
    }
    CHECK(inBand >= n - n / 20 && td == n && viol == 0,
          "hTarget %.2f: HOLD within +/-%.2f m in >= 95 %% (%d/%d), touchdown every time (%d), ceilings held (%d)",
          hts[j], band[j], inBand, n, td, viol);
    CHECK(photos >= n - n / 10 && imp <= impLim[j],
          "hTarget %.2f: photos within the 6.0 s air clock in >= 90 %% (%d/%d), impact <= %.2f m/s (%.2f)", hts[j],
          photos, n, impLim[j], imp);
    printf("info: hTarget %.2f nominal, accelerometer only: HOLD true h %.3f..%.3f m, photos %d/%d, impact max %.2f "
           "m/s\n", hts[j], lo, hi, photos, n, imp);
  }

  /* 0.40 and 0.60 m in FLIGHT timing, warm camera, the 10 s hover wait,
   * nominal (no offset, K exact, c 0): the hover at the height (HOLD has no
   * height feedback, so over 10 s the camera's velocity noise walks it by
   * up to ~0.2 m), the wait reached and timed out inside the 18 s air clock,
   * the slow LAND from HOLD as slow at contact as from 0.40 m. With
   * landFlatS left at 1.5 s the 0.60 m LAND touched down at p50 0.39 m/s
   * (0.40 m: 0.25); with vzConfigSetHeight's 2.0 s, 0.27. */
  float impP50[2] = { 0.0f, 0.0f }, impMax[2] = { 0.0f, 0.0f };
  for (int j = 0; j < 2; j++) {
    const float h = j == 0 ? VZ_H_TIMING_REF : H_EVENING_MAX;
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, h);
    vzSimConfig_t sc;
    vzSimDefaultConfig(&sc);
    noOffset(&c, &sc);
    warmCamera(&sc);
    harness_t hp = harnessFlight();
    hp.waitS = 10.0f;
    const int n = 50;
    int inBand = 0, timeout = 0, slow = 0, viol = 0, td = 0;
    float lo = 9.0f, hi = 0.0f, imp = 0.0f, waitAir = 0.0f, landAir = 0.0f, landS = 0.0f, air = 0.0f, im[50];
    for (int i = 0; i < n; i++) {
      run_t r;
      runSortie(&c, &sc, 1250u + (uint32_t)i, &hp, &r);
      im[i] = r.impactV;
      inBand += r.holdT >= 0.0f && fabsf(r.holdH - h) <= 0.10f && fabsf(r.holdHMax - h) <= 0.10f;
      timeout += r.why == FSM_HOVER_TIMEOUT;
      slow += r.landSlow;
      td += r.last.touchdown;
      viol += r.violations;
      lo = fminf(lo, r.holdHMin);
      hi = fmaxf(hi, r.holdHMax);
      imp = fmaxf(imp, r.impactV);
      waitAir = fmaxf(waitAir, r.waitT >= 0.0f ? r.waitT - r.rampT : 99.0f);
      landAir = fmaxf(landAir, r.airAtLandCmd);
      landS = fmaxf(landS, r.landS);
      air = fmaxf(air, r.airS);
    }
    impP50[j] = pct(im, n, 0.5f);
    impMax[j] = imp;
    CHECK(inBand >= n - n / 20 && lo >= h - 0.20f && timeout == n && slow == n && td == n && viol == 0,
          "hTarget %.2f, warm camera, wait 10 s: HOLD entry and peak within +/-0.10 m in >= 95 %% (%d/%d), never "
          "below by 0.20 m (%.3f), HOVER_TIMEOUT (%d), slow LAND from HOLD (%d), touchdown (%d), ceilings held (%d)",
          h, inBand, n, lo, timeout, slow, td, viol);
    CHECK(waitAir <= FSM_AIR_MAX_S - 1.0f && landAir <= FSM_AIR_HARD_S - 2.0f,
          "hTarget %.2f, warm camera, wait 10 s: the wait starts >= 1 s inside the pre-wait limit (air clock %.2f s "
          "of %.1f), LAND >= 2 s inside the hard limit (%.2f s of %.1f)", h, waitAir, FSM_AIR_MAX_S, landAir,
          FSM_AIR_HARD_S);
    printf("info: hTarget %.2f nominal, warm camera, wait 10 s: HOLD true h %.3f..%.3f m; air clock at the wait max "
           "%.2f s, at LAND max %.2f s; LAND max %.2f s, impact p50 %.2f max %.2f m/s; true airborne max %.2f s\n", h,
           lo, hi, waitAir, landAir, landS, impP50[j], imp, air);
  }
  CHECK(impP50[1] <= impP50[0] + 0.05f && impMax[1] <= 0.45f,
        "warm camera, wait 10 s: the slow LAND from 0.60 m is as slow at contact as from 0.40 m (p50 %.2f vs %.2f "
        "m/s, max %.2f <= 0.45)", impP50[1], impP50[0], impMax[1]);
}

/* The EST_RANGE guard as settings (vert_ctrl.h estRangeOn / estGraceS; NVS vzest / vzestms, 2026-10-22). A sortie on
 * the sim with a warm camera to HOLD; then the estimate pushed 2 m out of its range (white-box, as the interactive
 * sim's scenario 10) on every step from pushAtS after HOLD, broken for one step at breakAtS when >= 0. The default
 * (on, 0 s) lands EST_RANGE on the first out-of-range step, as before (testIdentity keeps the hash); off never lands
 * EST_RANGE (the event is still reported); a grace delays the LAND by that long and restarts after a break. In CLIMB
 * (pushAtS < 0: at climbAtS after lift-off, the camera estimate in use, pushed on from there) the default lands at
 * once; deferred, the climb loop's own arrival rule ends the climb in HOLD_REACHED on that step (the estimate reads
 * above the target), and the grace then runs in HOLD (holdEnter restarts it). */
typedef struct {
  bool on;
  float graceS, pushAtS, breakAtS, climbAtS, runS;
} erScen_t;

typedef struct {
  bool held;
  float holdT, landT, pushT, evtT;
  vzReason_t landReason;
  vzPhase_t endPhase;
  uint8_t evt;
} erRun_t;

static void erRun(const erScen_t *sc, erRun_t *r)
{
  memset(r, 0, sizeof(*r));
  r->holdT = r->landT = r->pushT = r->evtT = -1.0f;
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, VZ_H_TIMING_REF);
  c.estRangeOn = sc->on;
  c.estGraceS = sc->graceS;
  CHECK(vzConfigValid(&c), "estRangeOn %d, estGraceS %.2f: a valid config", sc->on, sc->graceS);
  vzState_t st;
  vzInit(&st, &c);
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  noOffset(&c, &pc);
  warmCamera(&pc);
  vzSimState_t sim;
  vzSimInit(&sim, &pc, 4242u);
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  float applied = 0.0f, vbat = sim.vbat, liftT = -1.0f;
  uint8_t evtN = 0;
  int pushK = -1;
  const int kMax = (int)(sc->runS * 1000.0f);
  for (int k = 0; k < kMax; k++) {
    const float t = (float)k * DT;
    if (k % 100 == 0) {
      vbat = sim.vbat;
    }
    const float th = r->holdT >= 0.0f ? t - r->holdT : -1.0f;
    const float tl = liftT >= 0.0f ? t - liftT : -1.0f;
    const bool pushNow = sc->pushAtS >= 0.0f ? th >= sc->pushAtS
                                            : (pushK >= 0 || (tl >= sc->climbAtS && o.phase == VZ_CLIMB));
    const bool breakNow = sc->breakAtS >= 0.0f && pushK >= 0 && k == pushK + (int)(sc->breakAtS * 1000.0f + 0.5f);
    if (pushNow && !breakNow) {
      st.h += 2.0f;   /* white-box: out of hMin..hMax + hOff for this step (the step reads st.h) */
      if (pushK < 0) {
        pushK = k;
        r->pushT = t;
      }
    }
    vzInput_t in = {
      .dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = 0.5f, .gyroMaxDps = 0.3f,
      .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy, .camVzValid = sim.camVzValid,
      .camVz = sim.camVz, .camBias = sim.camBias, .landFastOk = true,
    };
    vzStep(&st, &in, &o);
    if (pushNow && !breakNow && o.phase != VZ_LAND) {
      st.h -= 2.0f;   /* the estimate itself carries on (the push is the step's view of it) */
    }
    if (liftT < 0.0f && (o.phase == VZ_CLIMB || o.phase == VZ_HOLD)) {
      liftT = t;
    }
    if (o.phase == VZ_HOLD && r->holdT < 0.0f) {
      r->holdT = t;
      r->held = true;
    }
    if (o.evtN != evtN) {
      evtN = o.evtN;
      if (o.evt == VZ_R_EST_RANGE && r->evtT < 0.0f) {
        r->evtT = t;
      }
    }
    if (o.phase == VZ_LAND && r->landT < 0.0f) {
      r->landT = t;
      r->landReason = o.landReason;
    }
    vzSimStep(&sim, &pc, o.thrust, DT);
    applied = o.thrust;
    if (o.phase == VZ_DONE || o.phase == VZ_CUT || (o.phase == VZ_LAND && t - r->landT > 1.0f)) {
      break;
    }
  }
  r->endPhase = o.phase;
  printf("info: EST_RANGE on %d grace %.1f push %s: lift-off, HOLD %.3f s, push %.3f s, event %.3f s, LAND %.3f s (%s), "
         "end %s\n", sc->on, sc->graceS, sc->pushAtS >= 0.0f ? "in HOLD" : "in CLIMB", r->holdT, r->pushT, r->evtT,
         r->landT, r->landT >= 0.0f ? vzReasonName(r->landReason) : "none", vzPhaseName(r->endPhase));
}

static void testEstRange(void)
{
  erRun_t r;
  /* HOLD, the default: LAND EST_RANGE on the first out-of-range step */
  erRun(&(erScen_t){.on = true, .graceS = 0.0f, .pushAtS = 2.0f, .breakAtS = -1.0f, .runS = 30.0f}, &r);
  CHECK(r.held && r.landT >= 0.0f && r.landReason == VZ_R_EST_RANGE && r.landT - r.pushT < DT * 0.5f,
        "default (vzest 1, vzestms 0): HOLD, the estimate pushed 2 m out: LAND EST_RANGE on that step (%.3f s after)",
        r.landT - r.pushT);
  /* off: never that LAND; the event is reported */
  erRun(&(erScen_t){.on = false, .graceS = 0.0f, .pushAtS = 2.0f, .breakAtS = -1.0f, .runS = 30.0f}, &r);
  CHECK(r.held && r.pushT >= 0.0f && (r.landT < 0.0f || r.landReason != VZ_R_EST_RANGE) && r.evtT >= 0.0f &&
            r.evtT - r.pushT < DT * 0.5f,
        "vzest 0: the estimate out of range for %.0f s of HOLD: no EST_RANGE LAND (end %s, LAND %s), the event reported",
        30.0f - r.pushT, vzPhaseName(r.endPhase), r.landT >= 0.0f ? vzReasonName(r.landReason) : "none");
  /* a grace of 0.5 s: the LAND 0.5 s after the push */
  erRun(&(erScen_t){.on = true, .graceS = 0.5f, .pushAtS = 2.0f, .breakAtS = -1.0f, .runS = 30.0f}, &r);
  CHECK(r.held && r.landT >= 0.0f && r.landReason == VZ_R_EST_RANGE && fabsf(r.landT - r.pushT - 0.5f) < 0.0025f &&
            r.evtT >= 0.0f && r.evtT - r.pushT < DT * 0.5f,
        "vzestms 500: LAND EST_RANGE %.3f s after the push (the event at once)", r.landT - r.pushT);
  /* a grace of 3.0 s (the most): 3.0 s after */
  erRun(&(erScen_t){.on = true, .graceS = 3.0f, .pushAtS = 2.0f, .breakAtS = -1.0f, .runS = 30.0f}, &r);
  CHECK(r.held && r.landT >= 0.0f && r.landReason == VZ_R_EST_RANGE && fabsf(r.landT - r.pushT - 3.0f) < 0.0025f,
        "vzestms 3000: LAND EST_RANGE %.3f s after the push", r.landT - r.pushT);
  /* the grace restarts after one in-range step: broken at 0.3 s, the LAND at 0.3 + 0.5 s */
  erRun(&(erScen_t){.on = true, .graceS = 0.5f, .pushAtS = 2.0f, .breakAtS = 0.3f, .runS = 30.0f}, &r);
  CHECK(r.held && r.landT >= 0.0f && r.landReason == VZ_R_EST_RANGE && fabsf(r.landT - r.pushT - 0.8f) < 0.0035f,
        "vzestms 500, the condition broken for one step at 0.3 s: LAND EST_RANGE %.3f s after the push (0.3 + 0.5)",
        r.landT - r.pushT);
  /* CLIMB: the default lands at once (the height out of range: LAND, not the speed glitch's HOLD) */
  erRun(&(erScen_t){.on = true, .graceS = 0.0f, .pushAtS = -1.0f, .breakAtS = -1.0f, .climbAtS = 0.3f, .runS = 30.0f},
        &r);
  CHECK(!r.held && r.pushT >= 0.0f && r.landT >= 0.0f && r.landReason == VZ_R_EST_RANGE && r.landT - r.pushT < DT * 0.5f,
        "CLIMB, default: pushed 0.3 s after lift-off: LAND EST_RANGE on that step");
  erRun(&(erScen_t){.on = false, .graceS = 0.0f, .pushAtS = -1.0f, .breakAtS = -1.0f, .climbAtS = 0.3f, .runS = 30.0f},
        &r);
  CHECK(r.pushT >= 0.0f && r.held && (r.landT < 0.0f || r.landReason != VZ_R_EST_RANGE) && r.evtT >= 0.0f,
        "CLIMB, vzest 0: HOLD_REACHED, then no EST_RANGE LAND for the run (end %s, LAND %s), the event reported",
        vzPhaseName(r.endPhase), r.landT >= 0.0f ? vzReasonName(r.landReason) : "none");
  /* (a short grace: an estimate 2 m high makes the climb loop descend hard, a longer one ends the climb otherwise) */
  erRun(&(erScen_t){.on = true, .graceS = 0.1f, .pushAtS = -1.0f, .breakAtS = -1.0f, .climbAtS = 0.3f, .runS = 30.0f},
        &r);
  CHECK(r.pushT >= 0.0f && r.held && r.holdT - r.pushT < DT * 0.5f && r.landT >= 0.0f &&
            r.landReason == VZ_R_EST_RANGE && fabsf(r.landT - r.pushT - 0.1f) < 0.0025f,
        "CLIMB, vzestms 100: HOLD_REACHED on the push's step (the estimate above the target), LAND EST_RANGE %.3f s "
        "after the push", r.landT - r.pushT);
  /* the defaults as vzDefaultConfig sets them */
  vzConfig_t c;
  vzDefaultConfig(&c);
  CHECK(c.estRangeOn && c.estGraceS == 0.0f, "vzDefaultConfig: estRangeOn, estGraceS 0 (today's guard)");
}

/* vzConfigSetHeight(): the glue's per-sortie target. Up to 0.40 m (and for
 * NaN) hMax, climbMaxS and landFlatS keep their values; above it hMax grows
 * by the extra height (0.6 m over the target) and the timings by the extra
 * height at vClimbMax and at |landVref|, landFlatS at most 2.0 s. The
 * nominal climb to 0.60 m takes about 0.45 s longer than to 0.40 m, which
 * the old 2.0 s timeout would have covered by only ~0.15-0.3 s. */
static void testSetHeight(void)
{
  static const struct { float h, hMax, climb, flat; } want[] = {
    { 0.20f, 1.0f, 2.0f, 1.5f }, { 0.30f, 1.0f, 2.0f, 1.5f }, { 0.40f, 1.0f, 2.0f, 1.5f },
    { 0.50f, 1.1f, 2.25f, 1.5f + 0.1f / 0.3f }, { 0.60f, 1.2f, 2.5f, 2.0f },
  };
  for (size_t i = 0; i < sizeof(want) / sizeof(want[0]); i++) {
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, want[i].h);
    CHECK(c.hTarget == want[i].h && fabsf(c.hMax - want[i].hMax) <= 1e-5f &&
          fabsf(c.climbMaxS - want[i].climb) <= 1e-5f && fabsf(c.landFlatS - want[i].flat) <= 1e-5f &&
          vzConfigValid(&c),
          "vzConfigSetHeight(%.2f): hMax %.4f (want %.4f), climbMaxS %.4f (want %.4f), landFlatS %.4f (want %.4f), "
          "valid", want[i].h, c.hMax, want[i].hMax, c.climbMaxS, want[i].climb, c.landFlatS, want[i].flat);
  }
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, NAN);
  CHECK(c.hMax == 1.0f && c.climbMaxS == 2.0f && c.landFlatS == 1.5f && !vzConfigValid(&c),
        "vzConfigSetHeight(NaN): guard and timings unchanged, config invalid (CUT at vzInit)");
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, 1.21f);
  CHECK(!vzConfigValid(&c), "vzConfigSetHeight(1.21): above VZ_H_TAKEOFF_MAX, invalid");
  /* 2.00 m (2026-10-15) is the hover cap, reached by moves up (hOff): a take-off there is invalid (a LAND in its climb
   * falls from landStart at once: 1.8-2.05 m/s, docs/autonomy-a2b.md 23.9); its LAND timings come from hTarget + hOff */
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, 2.00f);
  CHECK(!vzConfigValid(&c), "vzConfigSetHeight(2.00): a take-off above VZ_H_TAKEOFF_MAX, invalid");
  /* the timing limits follow the take-off cap: what vzConfigSetHeight gives there + 0.5 s, the flown 4.5 and 4.5 s;
   * |hOff| holds 0.20 m moved up to the 2.00 m hover cap plus SET_HEIGHT's 0.50 m */
  CHECK(VZ_H_TAKEOFF_MAX == 1.20f && VZ_H_TARGET_MAX == 2.00f && fabsf(VZ_CLIMB_MAX_LIMIT_S - 4.5f) < 1e-5f &&
        fabsf(VZ_LAND_FLAT_LIMIT_S - 4.5f) < 1e-5f && fabsf(VZ_HOFF_MAX - 2.30f) < 1e-6f,
        "the take-off cap 1.20 m: climbMaxS <= %.2f s, landFlatS <= %.3f s; the hover cap 2.00 m: |hOff| <= %.2f m",
        (double)VZ_CLIMB_MAX_LIMIT_S, (double)VZ_LAND_FLAT_LIMIT_S, (double)VZ_HOFF_MAX);
  {
    vzConfig_t d;
    vzDefaultConfig(&d);
    CHECK(d.vClimbMax == VZ_CLIMB_V_DEFAULT && d.landVref == -VZ_LAND_V_DEFAULT && d.climbMaxS == VZ_CLIMB_BASE_S,
          "the limits are sized on the defaults: vClimbMax %.2f, landVref %.2f, climbMaxS %.1f", d.vClimbMax,
          d.landVref, d.climbMaxS);
  }
  /* the interactive hover's heights: above the knee the LAND flat part grows by the height at |landVref| */
  static const struct { float h, hMax, climb, flat; } hi[] = {
    { 0.80f, 1.4f, 3.0f, 2.0f + 0.2f / 0.3f }, { 1.00f, 1.6f, 3.5f, 2.0f + 0.4f / 0.3f }, { 1.20f, 1.8f, 4.0f, 4.0f },
  };
  for (size_t i = 0; i < sizeof(hi) / sizeof(hi[0]); i++) {
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, hi[i].h);
    CHECK(fabsf(c.hMax - hi[i].hMax) <= 1e-5f && fabsf(c.climbMaxS - hi[i].climb) <= 1e-5f &&
          fabsf(c.landFlatS - hi[i].flat) <= 1e-4f && vzConfigValid(&c),
          "vzConfigSetHeight(%.2f): hMax %.3f, climbMaxS %.3f, landFlatS %.3f, valid", hi[i].h, c.hMax, c.climbMaxS,
          c.landFlatS);
  }

  /* The nominal climb to 0.40, 0.60 and 1.20 m (no offset, K exact, warm camera):
   * the time in CLIMB against the timeout. */
  static const float climbH[3] = { VZ_H_TIMING_REF, H_EVENING_MAX, 1.20f };
  float climb[3] = { 0.0f, 0.0f, 0.0f }, climbMax[3] = { 0.0f, 0.0f, 0.0f };
  int timeout = 0, timeoutHi = 0;
  for (int j = 0; j < 3; j++) {
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, climbH[j]);
    climbMax[j] = c.climbMaxS;
    vzSimConfig_t sc;
    vzSimDefaultConfig(&sc);
    noOffset(&c, &sc);
    warmCamera(&sc);
    harness_t hp = harnessFlight();
    hp.landAtHoldS = 0.5f;
    /* the state machine's pre-wait limit grows by 25 ms per cm above 0.60 m (auto_fsm.h AUTO_AIR_PER_CM_ABOVE_60_MS) */
    hp.airMaxS += fmaxf(0.0f, climbH[j] - 0.60f) * 2.5f;
    for (int i = 0; i < 20; i++) {
      run_t r;
      runSortie(&c, &sc, 1300u + (uint32_t)i, &hp, &r);
      climb[j] = fmaxf(climb[j], r.holdT >= 0.0f ? r.holdT - r.liftDet : 99.0f);
      if (j < 2) {
        timeout += r.last.landReason == VZ_R_CLIMB_TIMEOUT;
      } else {
        timeoutHi += r.last.landReason == VZ_R_CLIMB_TIMEOUT;
      }
    }
  }
  CHECK(timeout == 0 && climb[1] <= climb[0] + 0.6f && 2.5f - climb[1] >= 2.0f - climb[0] - 0.05f,
        "nominal climb: 0.40 m in %.2f s (timeout 2.0 s), 0.60 m in %.2f s (timeout 2.5 s): the same margin, no "
        "CLIMB_TIMEOUT (%d)", climb[0], climb[1], timeout);
  CHECK(timeoutHi == 0 && climbMax[2] - climb[2] >= climbMax[0] - climb[0] - 0.05f,
        "nominal climb: 1.20 m in %.2f s (timeout %.2f s): at least the 0.40 m's margin (%.2f s), no CLIMB_TIMEOUT (%d)",
        climb[2], climbMax[2], climbMax[0] - climb[0], timeoutHi);
  printf("info: nominal climb (detected lift-off to HOLD): 0.40 m %.2f s, 0.60 m %.2f s, 1.20 m %.2f s\n", climb[0],
         climb[1], climb[2]);
}

/* n RAMPs on the ground (lift-off would need 1.02 x the RAMP plateau, so the
 * thrust is held just below it): returns the false lift-offs; RAMP seconds
 * and runs that left the ground in *rampS and *airborne. */
static int groundRamps(const vzConfig_t *c, float noise, int n, uint32_t seed0, float *rampS, int *airborne)
{
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  sc.kTrue = c->kPrior * c->rampTopRel * (1.0f + sc.geGain) * 1.02f;
  sc.accNoiseG = noise;
  int falseLift = 0;
  *rampS = 0.0f;
  *airborne = 0;
  for (int i = 0; i < n; i++) {
    vzState_t st;
    vzSimState_t sim;
    vzOutput_t o;
    vzInit(&st, c);
    vzSimInit(&sim, &sc, seed0 + (uint32_t)i);
    float applied = 0.0f;
    for (int k = 0; k < 5000; k++) {
      vzInput_t in = { .dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .gyroMaxDps = 0.5f,
                       .vbat = sim.vbat, .thrustApplied = applied, .fromSim = true };
      vzStep(&st, &in, &o);
      if (o.phase == VZ_RAMP) {
        *rampS += DT;
      }
      if (o.liftoffNow) {
        falseLift++;
        break;
      }
      if (o.phase == VZ_DONE) {
        break;
      }
      vzSimStep(&sim, &sc, o.thrust, DT);
      applied = o.thrust;
    }
    *airborne += sim.liftT >= 0.0f;
  }
  return falseLift;
}

/* 9.1 #3 and the 60 ms question: lift-off latency, T_lo and its bound. */
static void testLiftoff(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  harness_t hp = harnessDefault();
  hp.maxS = 3.5f;   /* lift-off happens at ~1.8 s */
  const int n = 100;
  float lat[100], hmm[100], err[100];
  int within250 = 0, early = 0, tloOk = 0, detected = 0, within60 = 0, below1cm = 0, viol = 0;
  for (int i = 0; i < n; i++) {
    run_t r;
    runSortie(&c, &sc, 300u + (uint32_t)i, &hp, &r);
    const bool d = r.liftDet >= 0.0f && r.liftTrue >= 0.0f;
    detected += d;
    lat[i] = d ? r.liftDet - r.liftTrue : 9.0f;
    hmm[i] = d ? r.hAtDet * 1000.0f : 999.0f;
    err[i] = d ? r.tLo / r.cmdAtTrueLift - 1.0f : 9.0f;
    within250 += d && lat[i] <= 0.250f;
    within60 += d && lat[i] <= 0.060f;
    early += d && lat[i] < 0.0f;
    below1cm += d && r.hAtDet < 0.01f;
    tloOk += d && err[i] >= -0.05f && err[i] <= 0.15f;
    viol += r.violations;
  }
  CHECK(detected == n, "lift-off detected in every seed (%d/%d)", detected, n);
  CHECK(within250 >= 95, "detected within 250 ms of the true lift-off in >= 95 seeds (%d)", within250);
  CHECK(early == 0, "never before the true lift-off (%d)", early);
  CHECK(below1cm == n, "detected before the drone is 1 cm up (%d/%d)", below1cm, n);
  CHECK(tloOk == n, "T_lo within -5..+15 %% of the thrust commanded at the true lift-off (%d/%d)", tloOk, n);
  CHECK(viol == 0, "RAMP under its plateau, T_lo under its bound (%d)", viol);
  const float l50 = pct(lat, n, 0.5f), l95 = pct(lat, n, 0.95f), lmax = pct(lat, n, 1.0f);
  const float h50 = pct(hmm, n, 0.5f), hmax = pct(hmm, n, 1.0f);
  const float e50 = pct(err, n, 0.5f), emax = pct(err, n, 1.0f);
  printf("info: lift-off latency vs the sim's true lift-off p50 %.0f p95 %.0f max %.0f ms (%d/100 within 60 ms); true height at detection p50 %.1f max %.1f mm; T_lo error p50 %+.1f max %+.1f %%\n",
         l50 * 1000.0f, l95 * 1000.0f, lmax * 1000.0f, within60, h50, hmax, e50 * 100.0f, emax * 100.0f);

  /* Default bAir (-2.3 mg) and a sim whose in-flight reading matches it,
   * then residual offsets the controller cannot know (unsafe sign). A late
   * detection used to keep the ramp going with the drone up (T_lo +44 %,
   * runaways to 1.9 m); now the ramp stops at its plateau, the drone floats
   * in ground effect and RAMP ends NO_LIFTOFF_TIME, or T_lo is bounded. */
  const float resid[] = { 0.0f, -0.005f, -0.010f, -0.020f };
  for (int j = 0; j < 4; j++) {
    vzDefaultConfig(&c);
    vzSimDefaultConfig(&sc);
    sc.accAirG = c.bAir + resid[j];
    int in250 = 0, late1s = 0, never = 0, viol2 = 0, never20 = 0;
    float errMax = -1.0f, hNever = 0.0f;
    for (int i = 0; i < n; i++) {
      run_t r;
      runSortie(&c, &sc, 300u + (uint32_t)i, &hp, &r);
      const float l = r.liftDet >= 0.0f ? r.liftDet - r.liftTrue : 9.0f;
      lat[i] = l;
      in250 += l >= 0.0f && l <= 0.250f;
      late1s += l > 1.0f && l < 9.0f;
      never += r.liftDet < 0.0f;
      viol2 += r.violations;
      if (r.liftDet >= 0.0f) {
        errMax = fmaxf(errMax, r.tLo / r.cmdAtTrueLift - 1.0f);
      } else {
        hNever = fmaxf(hNever, r.hMax);
        never20 += r.hMax <= 0.20f;
      }
    }
    if (j == 0) {
      CHECK(in250 >= 95, "default bAir with a matching in-flight reading: within 250 ms in >= 95 seeds (%d)", in250);
    }
    if (j == 2) {
      /* A2b: rampRate scaled with the prior (13250/s at 165k). An absolute
       * 12000/s at 165k gave 36/100 (A2a at 149.5k: 85/100). */
      CHECK(in250 >= 75, "residual -10 mg: within 250 ms in >= 75 seeds with the scaled ramp (%d)", in250);
    }
    CHECK(viol2 == 0, "residual %+.0f mg: RAMP under its plateau, T_lo under its bound (%d)", resid[j] * 1000.0f,
          viol2);
    CHECK(never20 == never, "residual %+.0f mg: an undetected lift-off floats below 0.20 m (%d/%d, highest %.2f m)",
          resid[j] * 1000.0f, never20, never, hNever);
    printf("info: lift-off with residual in-flight offset %+.0f mg: p50 %.0f p95 %.0f ms, %d/100 within 250 ms, %d later than 1 s, %d never (RAMP ended, true h <= %.2f m), T_lo up to %+.0f %% of the thrust at the true lift-off\n",
           resid[j] * 1000.0f, pct(lat, n, 0.5f) * 1000.0f, pct(lat, n, 0.95f) * 1000.0f, in250, late1s, never,
           hNever, errMax * 100.0f);
  }

  /* A stored negative bAir with the sim's in-flight reading matching it:
   * the detector does not subtract bAir (on the ground the offset is
   * absent), so nothing fires before the drone moves. */
  const float bairs[] = { -0.005f, -0.010f, -0.015f };
  for (int j = 0; j < 3; j++) {
    vzDefaultConfig(&c);
    c.bAir = bairs[j];
    vzSimDefaultConfig(&sc);
    sc.accAirG = c.bAir;
    int earlyB = 0, det = 0, tlo = 0, viol3 = 0;
    for (int i = 0; i < n; i++) {
      run_t r;
      runSortie(&c, &sc, 300u + (uint32_t)i, &hp, &r);
      const bool d = r.liftDet >= 0.0f && r.liftTrue >= 0.0f;
      det += d;
      earlyB += (r.liftDet >= 0.0f && r.liftTrue < 0.0f) || (d && r.liftDet < r.liftTrue);
      tlo += d && r.tLo / r.cmdAtTrueLift - 1.0f >= -0.05f && r.tLo / r.cmdAtTrueLift - 1.0f <= 0.15f;
      viol3 += r.violations;
    }
    CHECK(earlyB == 0 && viol3 == 0, "stored bAir %.0f mg: no detection before the true lift-off (%d), bounds held (%d)",
          bairs[j] * 1000.0f, earlyB, viol3);
    if (j <= 1) {
      CHECK(det == n && tlo == n, "stored bAir %.0f mg: all detected (%d), T_lo within -5..+15 %% (%d)",
            bairs[j] * 1000.0f, det, tlo);
    }
    printf("info: stored bAir %.0f mg (matching in-flight offset): detected %d/100, T_lo within -5..+15 %% in %d\n",
           bairs[j] * 1000.0f, det, tlo);
  }

  /* No detection on the ground: thrust held just below lift-off at the
   * RAMP plateau, noise only. */
  vzDefaultConfig(&c);
  float rampS = 0.0f;
  int airborne = 0;
  const int falseLift = groundRamps(&c, 0.03f, 6, 400u, &rampS, &airborne);
  CHECK(airborne == 0, "the sim stayed on the ground (%d)", airborne);
  CHECK(falseLift == 0 && rampS >= 10.0f, "no lift-off detected in %.1f s of RAMP on the ground at 0.03 g noise (%d)",
        rampS, falseLift);
  /* The rate over many ground RAMPs, and at twice the noise (SPIN's noise
   * gate opened so the RAMP runs), against the design's 10 mg / 0.04 m/s. */
  vzConfig_t d = c;
  d.liftCusumK = 0.010f;
  d.liftCusumV = 0.04f;
  int rate[2][2];
  for (int k = 0; k < 2; k++) {
    vzConfig_t *cc = k == 0 ? &c : &d;
    rate[k][0] = groundRamps(cc, 0.03f, 1000, 5000u, &rampS, &airborne);
    cc->spinStdMax = 0.10f;
    rate[k][1] = groundRamps(cc, 0.06f, 300, 5000u, &rampS, &airborne);
  }
  CHECK(rate[0][0] <= 3, "at 0.03 g at most 3 false lift-offs in 1000 ground RAMPs of 2.5 s (%d)", rate[0][0]);
  printf("info: false lift-offs on the ground: 0.03 g noise %d/1000 RAMPs (design detector %d), 0.06 g %d/300 (design %d)\n",
         rate[0][0], rate[1][0], rate[0][1], rate[1][1]);
}

/* 9.1 #4: no lift-off within the RAMP plateau. */
static void testNoLiftoff(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  sc.kTrue = 1.8f * c.kPrior;
  harness_t hp = harnessDefault();
  run_t r;
  runSortie(&c, &sc, 7u, &hp, &r);
  CHECK(r.last.phase == VZ_DONE && r.last.reason == VZ_R_ABORTED, "no lift-off: DONE ABORTED (%s %s)",
        vzPhaseName(r.last.phase), vzReasonName(r.last.reason));
  CHECK(r.last.landReason == VZ_R_NO_LIFTOFF_CEIL || r.last.landReason == VZ_R_NO_LIFTOFF_TIME,
        "abort reason NO_LIFTOFF_* (%s)", vzReasonName(r.last.landReason));
  CHECK(fabsf(r.abortMaxDrop - c.abortRate) < 1.0f, "thrust falls at 60000/s (%.0f)", r.abortMaxDrop);
  CHECK(!r.last.liftedOff && r.liftTrue < 0.0f && r.violations == 0, "never left the ground, no ceiling broken");
  CHECK(r.maxRampOverPlateau > 0.999f && r.maxRampOverPlateau < 1.002f, "the RAMP held its plateau (%.4f x)",
        r.maxRampOverPlateau);
}

/* One RAMP abort for testRampAbort. ABORT_CMD: LAND on the step after the
 * RAMP thrust reaches 'from' (above) or the last step below it (!above).
 * ABORT_TILT / ABORT_GYRO: from the step the RAMP thrust reaches 'from', a
 * leg lifts: the controller reads a 0.2 g rise with a 20 deg tilt (gravZ
 * cos 20 deg) or a 300 dps rate, until the lift-off detector fires with
 * that attitude (LIFTOFF_ATTITUDE); the tilt or rate stays. */
typedef enum { ABORT_CMD = 0, ABORT_TILT, ABORT_GYRO } abortHow_t;

typedef struct {
  float ref;          /* the last RAMP thrust before the abort */
  float firstDrop;    /* thrust fall on the abort's first step */
  float maxDev;       /* while ref (landStart - landRate t) > landFloor ref: largest |thrust - that| */
  float slowS;        /* s of the abort in that window */
  float doneS;        /* s to DONE */
  float maxFastDev;   /* largest |thrust - max(0, ref - abortRate t)| over the abort */
  vzReason_t landReason;
  bool aborted;       /* DONE ABORTED, never lifted off (the sim and the controller) */
} rampAbort_t;

static void rampAbortRun(const vzConfig_t *c, const vzSimConfig_t *sc, float from, bool above, abortHow_t how,
                         rampAbort_t *r)
{
  vzState_t st;
  vzSimState_t sim;
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  memset(r, 0, sizeof(*r));
  vzInit(&st, c);
  vzSimInit(&sim, sc, 11u);
  const float stepUp = c->rampRate * DT;
  const float tiltDeg = 20.0f, gyroDps = 300.0f;
  float applied = 0.0f, lastRamp = 0.0f;
  bool land = false, leg = false, inLand = false;
  for (int k = 0; k < 10000; k++) {
    const float before = o.thrust;
    vzInput_t in = { .dt = DT, .cmd = land ? VZ_CMD_LAND : VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f,
                     .gyroMaxDps = 0.5f, .vbat = sim.vbat, .thrustApplied = applied, .fromSim = true };
    if (leg) {
      if (o.phase == VZ_RAMP) {
        in.accZ += 0.2f;   /* the rise the detector sees while a leg lifts */
      }
      if (how == ABORT_TILT) {
        in.tiltDeg = tiltDeg;
        in.gravZ = cosf(tiltDeg * 3.14159265f / 180.0f);
      } else {
        in.gyroMaxDps = gyroDps;
      }
    }
    vzStep(&st, &in, &o);
    if (o.phase == VZ_LAND || (inLand && o.phase != VZ_RAMP)) {
      if (!inLand) {
        inLand = true;
        r->ref = lastRamp;
        r->firstDrop = before - o.thrust;
        r->landReason = o.landReason;
      }
      if (o.phase != VZ_LAND) {
        r->doneS += DT;
        r->aborted = o.phase == VZ_DONE && o.reason == VZ_R_ABORTED && !o.liftedOff && sim.liftT < 0.0f;
        return;
      }
      r->doneS += DT;
      r->maxFastDev = fmaxf(r->maxFastDev, fabsf(o.thrust - fmaxf(0.0f, r->ref - c->abortRate * r->doneS)));
      const float ceil = r->ref * (c->landStart - c->landRate * o.tPhase);
      if (ceil > c->landFloor * r->ref) {
        r->maxDev = fmaxf(r->maxDev, fabsf(o.thrust - ceil));
        r->slowS = r->doneS;
      }
    } else if (o.phase == VZ_RAMP) {
      lastRamp = o.thrust;
      if (above ? o.thrust >= from : (o.thrust < from && o.thrust + stepUp >= from)) {
        land = land || how == ABORT_CMD;
        leg = leg || how != ABORT_CMD;
      }
    }
    vzSimStep(&sim, sc, o.thrust, DT);
    applied = o.thrust;
  }
}

/* The RAMP abort (A2b "always slow land", critique R13): from a RAMP thrust
 * at or above abortSlowRel x kPrior / Vf, where a lift-off hidden by a
 * negative offset may already have happened, the abort first follows LAND's
 * ceiling with T_ref = that thrust (0.95 down to 0.65 x it at 0.10 x it per
 * s), then falls at abortRate; one step below it the abort falls at
 * abortRate at once (on the ground: no reason to keep the motors turning).
 * A lift-off signature with a tilt or rate over the lift-off limits is one
 * leg up (LIFTOFF_ATTITUDE): it falls at abortRate at once from any RAMP
 * thrust, the slow rule's range included, so a tipped drone is not held at
 * 0.95 x its thrust for 3 s. A drone 1.8 x heavier than its prior never
 * leaves the ground; its battery does not sag, so Vf is the resting voltage
 * exactly. */
static void testRampAbort(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  sc.kTrue = 1.8f * c.kPrior;
  sc.vSag = 0.0f;
  const float from = c.abortSlowRel * c.kPrior / sc.vRest;
  const float plateau = c.rampTopRel * c.kPrior / sc.vRest;
  rampAbort_t a, b;
  rampAbortRun(&c, &sc, from, true, ABORT_CMD, &a);
  rampAbortRun(&c, &sc, from, false, ABORT_CMD, &b);
  const float slowS = (c.landStart - c.landFloor) / c.landRate;
  CHECK(a.ref >= from && a.ref < from + c.rampRate * DT + TOL_T && a.ref < 0.7f * plateau && a.aborted &&
        a.landReason == VZ_R_CMD_LAND && a.maxDev <= TOL_T && fabsf(a.slowS - slowS) <= 0.002f &&
        fabsf(a.doneS - (slowS + c.landFloor * a.ref / c.abortRate)) <= 0.005f,
        "abort from a RAMP thrust >= abortSlowRel kPrior/Vf (%.0f, %.3f x, below the plateau): LAND's ceiling for %.3f s (max deviation %.2f), then abortRate; DONE ABORTED after %.3f s",
        a.ref, a.ref * sc.vRest / c.kPrior, a.slowS, a.maxDev, a.doneS);
  CHECK(b.ref < from && b.ref > from - c.rampRate * DT - TOL_T && b.aborted && b.landReason == VZ_R_CMD_LAND &&
        fabsf(b.firstDrop - c.abortRate * DT) <= TOL_T && fabsf(b.doneS - b.ref / c.abortRate) <= 0.002f,
        "abort from one step below it (%.0f): abortRate from the first step (%.1f per step), DONE ABORTED after %.3f s",
        b.ref, b.firstDrop, b.doneS);
  printf("info: RAMP abort at %.0f (%.3f x kPrior/Vf): slow for %.3f s, DONE after %.3f s; at %.0f: DONE after %.3f s\n",
         a.ref, a.ref * sc.vRest / c.kPrior, a.slowS, a.doneS, b.ref, b.doneS);

  /* LIFTOFF_ATTITUDE inside the slow rule's range (WP4-1): a tilt or a rate. */
  const char *const attName[2] = { "tilt 20 deg", "rate 300 dps" };
  for (int k = 0; k < 2; k++) {
    rampAbort_t t;
    rampAbortRun(&c, &sc, from, true, k == 0 ? ABORT_TILT : ABORT_GYRO, &t);
    CHECK(t.landReason == VZ_R_LIFTOFF_ATTITUDE && t.aborted && t.ref >= from && t.ref < 0.7f * plateau &&
          fabsf(t.firstDrop - c.abortRate * DT) <= TOL_T && t.maxFastDev <= TOL_T &&
          fabsf(t.doneS - t.ref / c.abortRate) <= 0.002f,
          "%s at lift-off from a RAMP thrust >= abortSlowRel kPrior/Vf (%.0f, %.3f x): %s, abortRate from the first step (%.1f per step, max deviation %.2f), DONE ABORTED after %.3f s (not the slow abort's %.3f s)",
          attName[k], t.ref, t.ref * sc.vRest / c.kPrior, vzReasonName(t.landReason), t.firstDrop, t.maxFastDev,
          t.doneS, slowS + c.landFloor * t.ref / c.abortRate);
    printf("info: RAMP %s lift-off at %.0f (%.3f x kPrior/Vf): %s, %.1f per step, DONE after %.3f s\n", attName[k],
           t.ref, t.ref * sc.vRest / c.kPrior, vzReasonName(t.landReason), t.firstDrop, t.doneS);
  }
}

/* 9.1 #5: LAND properties over offsets and K errors. */
static void testLandMc(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  harness_t hp = harnessDefault();
  vzSimConfig_t base;
  vzSimDefaultConfig(&base);
  mc_t m;
  rngState = 777u;
  runMc("spec 9.1 #5: DRY timing, residual N(0, 10 mg), K +/-8 %, 300 seeds", &c, &hp, &base, 300, drawSpec5, NULL,
        5000u, &m);
  CHECK(m.violations == 0, "LAND thrust under the falling ceiling, ceiling never rising, every ceiling held (%d)",
        m.violations);
  CHECK(m.ended == m.n, "every sortie ends: touchdown, abort or the 6 s cut (%d/%d)", m.ended, m.n);
  CHECK(m.tdHigh == 0, "no touchdown declared with the sim above 0.05 m (%d)", m.tdHigh);
}

/* The user's range: residual offset -20..+20 mg, K error +/-12 %, the
 * camera's velocity offset +/-0.05 m/s and offset error +/-2 mg, uniform
 * (300 per row) and at its 8 corners (50 seeds each). DRY timing
 * (the camera waived) as before, and FLIGHT timing, where STABLE and
 * NOT_STABLE wait for the camera: without a camera the hover lands at
 * HOVER + 2 s (or the 6 s air clock), with a camera the sortie reaches
 * the hover wait and lands at its end or at the 18 s air clock (reached
 * with the longest take-off photo set, 3 x 1 s, and NVS autoair 8 s). */
static void testUserRangeMc(void)
{
  vzConfig_t c, c110;
  vzDefaultConfig(&c);
  c110 = c;
  c110.capLo = 1.10f;
  vzSimConfig_t noCam, warm, cold, lost;
  vzSimDefaultConfig(&noCam);
  warm = noCam;
  warmCamera(&warm);
  cold = noCam;
  cold.camWakeS = 3.0f;
  lost = warm;
  lost.camLossS = 3.0f;      /* gone for good 3 s after the lift-off: mid-wait */
  lost.camLossLenS = 100.0f;
  const harness_t dry = harnessDefault();
  const harness_t fl = harnessFlight();
  harness_t fl12 = fl;
  fl12.waitS = FSM_WAIT_MAX_S;
  harness_t flHard = fl12;
  flHard.photoS = 3.0f;          /* 3 photos 1 s apart */
  flHard.airMaxS = FSM_AIR_HI_S;  /* NVS autoair at its highest */
  const int n = 300;
  mc_t m[10];
  const mcDraw_t u = drawUserRange;
  const mcCorner_t uc = cornerUserRange;
  rngState = 4242u;
  runMc("user range, DRY timing, capLo 1.65, no wait", &c, &dry, &noCam, n, u, uc, 9000u, &m[0]);
  rngState = 4242u;
  runMc("user range, DRY timing, capLo 1.10", &c110, &dry, &noCam, n, u, uc, 9000u, &m[1]);
  rngState = 4242u;
  runMc("user range, FLIGHT, no camera (STABLE unreachable)", &c, &fl, &noCam, n, u, uc, 9000u, &m[2]);
  rngState = 4242u;
  runMc("user range, FLIGHT, capLo 1.10, no camera", &c110, &fl, &noCam, n, u, uc, 9000u, &m[3]);
  rngState = 4242u;
  runMc("user range, FLIGHT, cold camera (first frame 3.0 s into the take-off)", &c, &fl, &cold, n, u, uc, 9000u,
        &m[4]);
  rngState = 4242u;
  runMc("user range, FLIGHT, warm camera, wait 5 s", &c, &fl, &warm, n, u, uc, 9000u, &m[5]);
  rngState = 4242u;
  runMc("user range, FLIGHT, warm camera, wait 12 s (max)", &c, &fl12, &warm, n, u, uc, 9000u, &m[6]);
  rngState = 4242u;
  runMc("user range, FLIGHT, warm camera lost 3 s after lift-off", &c, &fl, &lost, n, u, uc, 9000u, &m[7]);
  rngState = 4242u;
  runMc("user range, FLIGHT, warm camera, c +/-0.10 m/s, wait 5 s", &c, &fl, &warm, n, drawUserRangeC10,
        cornerUserRangeC10, 9000u, &m[8]);
  rngState = 4242u;
  runMc("user range, FLIGHT, warm camera, wait 12 s, photos 3 x 1 s, autoair 8 s", &c, &flHard, &warm, n, u, uc,
        9000u, &m[9]);

  const char *const what[10] = { "DRY 1.65", "DRY 1.10", "FLIGHT no camera", "FLIGHT 1.10 no camera", "FLIGHT cold",
                                 "FLIGHT warm", "FLIGHT warm 12 s", "FLIGHT camera lost", "FLIGHT c 0.10",
                                 "FLIGHT warm 12 s, long photos" };
  for (int k = 0; k < 10; k++) {
    checkMc(what[k], &m[k], (k == 1 || k == 3) ? 1.10f : 1.65f);
  }
  /* The airborne limits are really reached now (they never were with the
   * DRY timing alone). */
  CHECK(m[2].why[FSM_NO_CAM] > 0 && m[2].why[FSM_HOVER_TIMEOUT] == 0 && m[2].waited == 0 && m[2].cornerWaited == 0,
        "no camera: the hover lands HOVER_NO_CAM (%d), never waits (corners included)", m[2].why[FSM_NO_CAM]);
  CHECK(m[5].why[FSM_HOVER_TIMEOUT] >= n / 2, "warm camera: most sorties wait and land HOVER_TIMEOUT (%d/%d)",
        m[5].why[FSM_HOVER_TIMEOUT], n);
  /* The hover wait at its maximum: the 6 s pre-wait limit and the 12 s wait
   * end inside the 18 s air clock, so the wait ends HOVER_TIMEOUT; the
   * longest take-off photo set (3 x 1 s) under NVS autoair 8 s starts the
   * wait late enough that the 18 s clock lands first. */
  CHECK(m[6].why[FSM_HOVER_TIMEOUT] >= n / 2 && m[6].why[FSM_AIR_TIME] == 0 &&
        m[6].worstAirClock <= FSM_AIR_HARD_S + 0.002f,
        "wait 12 s: HOVER_TIMEOUT (%d/%d), never the air clock (%d), air clock at LAND %.3f s at most",
        m[6].why[FSM_HOVER_TIMEOUT], n, m[6].why[FSM_AIR_TIME], m[6].worstAirClock);
  CHECK(m[9].why[FSM_AIR_TIME] >= n / 2 && m[9].worstAirClock <= FSM_AIR_HARD_S + 0.002f &&
        m[9].worstAirClock >= FSM_AIR_HARD_S - 0.002f,
        "wait 12 s after 3 x 1 s photos: the 18 s air clock lands (%d/%d; HOVER_TIMEOUT %d), at %.3f s",
        m[9].why[FSM_AIR_TIME], n, m[9].why[FSM_HOVER_TIMEOUT], m[9].worstAirClock);
  CHECK(m[7].why[FSM_NO_CAM] > 0, "camera lost mid-wait: HOVER_NO_CAM (%d)", m[7].why[FSM_NO_CAM]);
  /* Peak heights and impacts over the uniform draws and the corners: the
   * measured maxima (docs/autonomy-a2.md section 4.9) plus about 10 %. A
   * 1.10 cap keeps every sortie low now that a late lift-off detection
   * cannot raise it (finding: 1.91 m before; the rest is a strong drone
   * rising at the ramp plateau). The bound for the others is time: the 6 s pre-wait
   * air clock (or HOVER + 2 s) without a camera, the camera's velocity offset
   * over the wait with it (a 12 s wait: 1.03 m, the 8 s wait of A2b 0.74 m).
   * The accelerometer-only worst case is the corner -20 mg, K -12 % (2.08 m,
   * 2.20 m/s), outside the uniform draw's 1.73 m.
   * 2026-10-15 (docs/autonomy-a2b.md 22): a camera HOLD no longer lands K_RANGE at khatMinRel (its floor is
   * VZ_KHAT_HOLD_LO_REL), so the corners K -12 % with the camera's offset c -0.05 m/s hover on to the emulated wait's
   * end, climbing at c (the camera cannot see its own offset): cold corners 0.84 -> 1.12 m, warm 0.58 -> 0.87, warm 12 s
   * 1.03 -> 1.28, c 0.10 1.24, 12 s after long photos 1.41 (impact 1.30); the uniform draws barely move (0.89, 0.72,
   * 1.03, 1.02, 1.14). The limits are those + ~10 %. Without the air clock the drift lasts until Claude's LAND or the
   * battery (risk 3 of 22: c x the flight; the logged estimate drifted -0.011..+0.031 m/s). */
  static const float hLim[10] = { 2.25f, 0.8f, 2.25f, 0.8f, 1.25f, 0.95f, 1.40f, 0.70f, 1.35f, 1.55f };
  static const float impLim[10] = { 2.35f, 1.35f, 2.35f, 1.35f, 1.5f, 1.2f, 1.3f, 1.2f, 1.5f, 1.4f };
  for (int k = 0; k < 10; k++) {
    const float h = fmaxf(m[k].worstH, m[k].cornerH), imp = fmaxf(m[k].worstImpact, m[k].cornerImpact);
    CHECK(h <= hLim[k] && imp <= impLim[k],
          "%s: true peak <= %.2f m (uniform %.2f, corners %.2f), impact <= %.2f m/s (uniform %.2f, corners %.2f)",
          what[k], hLim[k], m[k].worstH, m[k].cornerH, impLim[k], m[k].worstImpact, m[k].cornerImpact);
  }
}

/* The absolute cap: a high-K drone on a sagging battery would need more
 * than VZ_THRUST_ABS_MAX to climb; the ceiling is VZ_THRUST_ABS_MAX on
 * every step and the climb lands CLIMB_CAPPED or K_RANGE instead of
 * saturating the motors. It still binds at 58000 (2026-10-14): 3.55 V at
 * rest, 14e-6 V of sag per count, the hover needs about 64000. */
static void testAbsCap(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  c.capLo = 2.0f;
  c.kPrior = 170000.0f;
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  sc.kTrue = 175000.0f;
  sc.vRest = 3.55f;
  sc.vSag = 14e-6f;   /* ~2.74 V loaded at 58000: the hover needs ~64000 */
  harness_t hp = harnessDefault();
  int atAbs = 0, viol = 0, ended = 0;
  float maxT = 0.0f;
  for (int i = 0; i < 10; i++) {
    run_t r;
    runSortie(&c, &sc, 1100u + (uint32_t)i, &hp, &r);
    maxT = fmaxf(maxT, r.maxThrust);
    atAbs += r.maxThrust >= VZ_THRUST_ABS_MAX - 1.0f;
    viol += r.violations;
    ended += endedOk(&r);
  }
  CHECK(maxT <= VZ_THRUST_ABS_MAX && atAbs > 0, "%.0f binds and is never exceeded (max %.0f, %d runs at it)",
        VZ_THRUST_ABS_MAX, maxT, atAbs);
  CHECK(viol == 0 && ended == 10, "every ceiling held and every run ended (%d, %d/10)", viol, ended);
}

/* The capLo ceiling (2026-10-14, docs/autonomy-a2b.md "End of battery"):
 * CLIMB keeps capLo x T_lo in thrust units (the climb's runaway bound); HOLD
 * is battery-scaled, capLo x T_lo x max(V_lo, Vf) / Vf (capLo x K_lo in K
 * units), never tighter than before. One controller step from a set state,
 * checked on the exact ceiling it reports. The controller computes both
 * bounds in K units with one division, so capLo x T_lo comes out as
 * capLo x T_lo x Vf / Vf: exact for these values, at most 1 ulp off in
 * general. */
static void capLoStep(vzPhase_t ph, float tLo, float vLo, float vf, vzConfig_t *c, vzOutput_t *o)
{
  vzDefaultConfig(c);
  vzState_t st;
  vzInit(&st, c);
  st.phase = ph;
  st.liftedOff = st.started = true;
  st.holdReached = ph == VZ_HOLD;
  st.vf = vf;
  st.vLo = vLo;
  st.tLo = tLo;
  st.khat = st.kfrz = c->kPrior;
  st.h = ph == VZ_HOLD ? c->hTarget : 0.05f;
  vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = vf, .thrustApplied = tLo};
  vzStep(&st, &in, o);
}

static void testHoldCapLoK(void)
{
  vzConfig_t c;
  vzOutput_t o;
  /* (1) T_lo 33000 at V_lo 3.70, HOLD at Vf 3.00: capLo x K_lo is 67.2k; the ceiling is min(that, capKhat x Khat /
   * Vf, 58000) and capLo x T_lo (54450) no longer binds */
  capLoStep(VZ_HOLD, 33000.0f, 3.70f, 3.00f, &c, &o);
  float want = fminf(fminf(c.capLo * 33000.0f * 3.70f / 3.00f, c.capKhat * o.khat / 3.00f), VZ_THRUST_ABS_MAX);
  CHECK(o.phase == VZ_HOLD && fabsf(o.ceiling - want) <= 0.05f && o.ceiling > c.capLo * 33000.0f + 1.0f &&
            c.capLo * 33000.0f * 3.70f / 3.00f > 67000.0f,
        "HOLD, T_lo 33000 at 3.70 V, Vf 3.00 V: ceiling %.1f = min(%.0f, capKhat %.0f, %.0f), over capLo x T_lo %.0f",
        o.ceiling, c.capLo * 33000.0f * 3.70f / 3.00f, c.capKhat * o.khat / 3.00f, VZ_THRUST_ABS_MAX,
        c.capLo * 33000.0f);
  /* ... and with T_lo 28000 the scaled bound itself binds: capLo x T_lo x 3.70 / 3.00 */
  capLoStep(VZ_HOLD, 28000.0f, 3.70f, 3.00f, &c, &o);
  want = c.capLo * 28000.0f * 3.70f / 3.00f;
  CHECK(o.phase == VZ_HOLD && fabsf(o.ceiling - want) <= 0.05f && want < c.capKhat * o.khat / 3.00f &&
            want < VZ_THRUST_ABS_MAX,
        "HOLD, T_lo 28000 at 3.70 V, Vf 3.00 V: ceiling %.1f = capLo x T_lo x V_lo / Vf %.1f", o.ceiling, want);
  /* (2) the same in CLIMB: exactly capLo x T_lo (thrust units, as before) */
  capLoStep(VZ_CLIMB, 33000.0f, 3.70f, 3.00f, &c, &o);
  CHECK(o.phase == VZ_CLIMB && o.ceiling == c.capLo * 33000.0f,
        "CLIMB, T_lo 33000 at 3.70 V, Vf 3.00 V: ceiling exactly capLo x T_lo (%.2f, %.2f)", o.ceiling,
        c.capLo * 33000.0f);
  capLoStep(VZ_CLIMB, 28000.0f, 3.70f, 3.00f, &c, &o);
  CHECK(o.phase == VZ_CLIMB && o.ceiling == c.capLo * 28000.0f,
        "CLIMB, T_lo 28000 at 3.70 V, Vf 3.00 V: ceiling exactly capLo x T_lo (%.2f)", o.ceiling);
  /* (3) HOLD with Vf above V_lo: exactly capLo x T_lo (never tighter than before) */
  capLoStep(VZ_HOLD, 28000.0f, 3.70f, 3.90f, &c, &o);
  CHECK(o.phase == VZ_HOLD && o.ceiling == c.capLo * 28000.0f && c.capLo * 28000.0f < c.capKhat * o.khat / 3.90f,
        "HOLD, T_lo 28000 at 3.70 V, Vf 3.90 V: ceiling exactly capLo x T_lo (%.2f)", o.ceiling);
  /* V_lo unset (0): the factor is 1 */
  capLoStep(VZ_HOLD, 28000.0f, 0.0f, 3.00f, &c, &o);
  CHECK(o.phase == VZ_HOLD && o.ceiling == c.capLo * 28000.0f, "HOLD with V_lo 0: capLo x T_lo (%.2f)", o.ceiling);
}

/* 9.1 #6: the LAND loop fooled into wanting more thrust. */
static void testFooledLoop(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  harness_t hp = harnessDefault();
  hp.airAtLandG = c.bAir - 0.030f;
  const int n = 30;
  int ok = 0, td = 0, viol = 0;
  float worstTdOk = 0.0f, worstLand = 0.0f;
  for (int i = 0; i < n; i++) {
    run_t r;
    runSortie(&c, &sc, 600u + (uint32_t)i, &hp, &r);
    ok += r.tdOkAfter >= 0.0f && r.tdOkAfter <= 2.3f;
    td += r.last.touchdown && r.landS < VZ_LAND_MAX_S;
    viol += r.violations;
    worstTdOk = fmaxf(worstTdOk, r.tdOkAfter);
    worstLand = fmaxf(worstLand, r.landS);
  }
  /* The spec says 2.1 s; the ceiling reaches 0.75 T_ref at 2.0 s, but
   * T_hov = K_ref / Vf falls ~3 % as the battery recovers during the descent,
   * so tdThrustOk comes at ~2.2 s. */
  CHECK(ok == n, "fooled loop: tdThrustOk within 2.3 s of LAND entry (%d/%d, worst %.2f s)", ok, n, worstTdOk);
  CHECK(td == n, "and touchdown before 6 s (%d/%d, worst LAND %.2f s)", td, n, worstLand);
  CHECK(viol == 0, "thrust followed the ceiling, never above it (%d)", viol);
  printf("info: fooled LAND loop (-30 mg at LAND entry): tdThrustOk after %.2f s worst, LAND %.2f s worst\n",
         worstTdOk, worstLand);
}

/* 9.1 #7: floating in strong ground effect. */
static void testFloating(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  sc.geGain = 0.6f;
  harness_t hp = harnessDefault();
  const int n = 30;
  int high = 0, cutContact = 0, cuts = 0, viol = 0;
  float worstTdH = 0.0f;
  for (int i = 0; i < n; i++) {
    run_t r;
    runSortie(&c, &sc, 700u + (uint32_t)i, &hp, &r);
    high += r.tdHigh;
    worstTdH = fmaxf(worstTdH, r.tdTrueH);
    cuts += r.last.phase == VZ_CUT;
    cutContact += r.last.phase == VZ_CUT && r.contactInLand;
    viol += r.violations;
  }
  CHECK(high == 0, "geGain 0.6: no touchdown declared above 0.05 m (%d, highest %.3f m)", high, worstTdH);
  CHECK(cutContact == 0, "the 6 s cut fires only without contact (%d of %d cuts)", cutContact, cuts);
  CHECK(viol == 0, "no ceiling broken (%d)", viol);
  printf("info: floating (geGain 0.6): touchdown declared at true h <= %.3f m, %d cuts\n", worstTdH, cuts);
}

/* One LAND from CLIMB for testClimbLandRef. The test keeps its own copy of
 * the CLIMB average (the applied thrust, tau tHoldTauS, seeded with T_lo at
 * the lift-off) and of the CLIMB time since the last step at a ceiling, from
 * the outputs alone, and sends LAND on the first CLIMB step with tAir >=
 * airS, or (afterCapS > 0) on the first one free of the ceiling for afterCapS
 * after a capped step. The sim's battery does not sag: Vf is vRest exactly. */
typedef struct {
  bool ok;            /* LAND CMD_LAND from CLIMB happened */
  bool capSeen;       /* a CLIMB step at a ceiling before the LAND */
  float tAir, freeS;  /* at the LAND: s since the lift-off, CLIMB s since the last capped step */
  float tLo, avg, bound, tRef;
} climbRef_t;

static void climbLandRef(const vzConfig_t *c, const vzSimConfig_t *sc, uint32_t seed, float airS, float afterCapS,
                         climbRef_t *r)
{
  vzState_t st;
  vzSimState_t sim;
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  memset(r, 0, sizeof(*r));
  vzInit(&st, c);
  vzSimInit(&sim, sc, seed);
  r->bound = c->landClimbRefRel * c->kPrior / sc->vRest;
  float applied = 0.0f;
  bool land = false;
  for (int k = 0; k < 10000; k++) {
    vzInput_t in = { .dt = DT, .cmd = land ? VZ_CMD_LAND : VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f,
                     .gyroMaxDps = 0.5f, .vbat = sim.vbat, .thrustApplied = applied, .fromSim = true };
    vzStep(&st, &in, &o);
    if (land) {
      r->ok = o.phase == VZ_LAND && o.landReason == VZ_R_CMD_LAND && o.liftedOff && !o.holdReached;
      r->tRef = o.tRef;
      return;
    }
    if (o.phase == VZ_CLIMB) {
      if (o.liftoffNow) {
        r->tLo = r->avg = o.tLo;
        r->freeS = 0.0f;
      }
      r->avg += (applied - r->avg) * DT / (c->tHoldTauS + DT);
      r->freeS = o.atCap ? 0.0f : r->freeS + DT;
      r->capSeen = r->capSeen || o.atCap;
      r->tAir = o.tAir;
      land = (airS >= 0.0f && o.tAir >= airS) || (afterCapS > 0.0f && r->capSeen && r->freeS >= afterCapS);
    } else if (o.phase != VZ_SPIN && o.phase != VZ_RAMP) {
      return;   /* HOLD, or LAND by the controller itself: not this case */
    }
    vzSimStep(&sim, sc, o.thrust, DT);
    applied = o.thrust;
  }
}

/* T_ref of a LAND from CLIMB (A2b "always slow land", docs/autonomy-a2.md
 * section 4.2) as a unit: T_lo within tHoldTauS of the lift-off or of a
 * capped CLIMB step, else the CLIMB average within [T_lo, landClimbRefRel x
 * kPrior / Vf]. Swept over the LAND time, K, the residual offset, capLo
 * (1.30 climbs at its ceiling for a while, then free of it) and
 * landClimbRefRel; every branch of the rule must be reached. */
static void testClimbLandRef(void)
{
  const float caps[] = { 1.30f, 1.65f }, rels[] = { 1.00f, 0.80f }, ks[] = { 0.88f, 1.00f, 1.12f };
  const float bs[] = { -0.010f, 0.0f, 0.010f };
  int n = 0, match = 0, byTime = 0, byTimeExact = 0, byCap = 0, byCapExact = 0, inner = 0, atBound = 0, atLo = 0;
  float worst = 0.0f;
  for (int a = 0; a < 2; a++) {
    for (int q = 0; q < 2; q++) {
      for (int k = 0; k < 3; k++) {
        for (int b = 0; b < 3; b++) {
          vzConfig_t c;
          vzDefaultConfig(&c);
          c.capLo = caps[a];
          c.landClimbRefRel = rels[q];
          vzSimConfig_t sc;
          vzSimDefaultConfig(&sc);
          sc.vSag = 0.0f;
          sc.kTrue = c.kPrior * ks[k];
          sc.accAirG = c.bAir + bs[b];
          for (int j = 0; j < 10; j++) {
            const uint32_t seed = 3000u + (uint32_t)(((a * 2 + q) * 3 + k) * 3 + b);
            climbRef_t r;
            /* 7 LAND times 0.05-0.65 s after the lift-off, then 50, 300 and 150 ms after a capped step */
            climbLandRef(&c, &sc, seed, j < 7 ? 0.05f + 0.10f * (float)j : -1.0f,
                         j == 7 ? 0.05f : (j == 8 ? 0.30f : (j == 9 ? 0.15f : 0.0f)), &r);
            if (!r.ok) {
              continue;
            }
            n++;
            const bool lo = r.freeS < c.tHoldTauS;
            const float want = lo ? r.tLo : fmaxf(r.tLo, fminf(r.avg, r.bound));
            match += fabsf(r.tRef - want) <= TOL_T;
            worst = fmaxf(worst, fabsf(r.tRef - want));
            if (lo && !r.capSeen) {
              byTime++;
              byTimeExact += r.tRef == r.tLo;
            } else if (lo && r.tAir >= c.tHoldTauS) {
              byCap++;
              byCapExact += r.tRef == r.tLo;
            } else if (!lo) {
              inner += r.avg > r.tLo && r.avg < r.bound;
              atBound += r.avg >= r.bound;
              atLo += r.avg <= r.tLo;
            }
          }
        }
      }
    }
  }
  CHECK(n >= 250 && match == n, "LAND from CLIMB: T_ref follows the rule in every LAND (%d/%d, worst %.2f)", match, n,
        worst);
  CHECK(byTime > 0 && byTimeExact == byTime, "within tHoldTauS of the lift-off: T_ref = T_lo (%d/%d)", byTimeExact,
        byTime);
  CHECK(byCap > 0 && byCapExact == byCap, "within tHoldTauS of a capped CLIMB step, later than that after the lift-off: T_ref = T_lo (%d/%d)",
        byCapExact, byCap);
  CHECK(inner > 0 && atBound > 0, "otherwise the CLIMB average: inside [T_lo, landClimbRefRel kPrior/Vf] %d, at the bound %d",
        inner, atBound);
  printf("info: LAND from CLIMB T_ref: %d LANDs, T_lo by time %d, by a capped step %d, CLIMB average %d inside, %d at the bound, %d at T_lo\n",
         n, byTime, byCap, inner, atBound, atLo);
}

/* No false touchdown in the air. HOLD cannot declare touchdown at all, so
 * what counts is LAND with the detector armed (tdThrustOk) and the drone
 * above 5 cm. The riskiest case is LAND from CLIMB with T_ref = T_lo
 * (K_ref = kPrior: tdThrustOk holds from the first step): A2b keeps it inside
 * tHoldTauS of the lift-off and after a capped climb step. The phone's stick
 * (slow LAND) or a Mac LAND mid-climb gives it: swept over the LAND time after
 * lift-off, the K error, the residual offset, the cap and the target. The
 * same sweep bounds the impact and the rise after a LAND from CLIMB (A2b
 * section 11.2), with no camera (the checks) and with the camera awake from
 * the first command (info). */
typedef struct {
  int runs, fromClimb, tdHigh, viol, ended;
  float armed, worstTdH, worstImpact, worstImpactCap, worstRise;
  float imp50[2], imp95[2], impMax[2], rise50[2], rise95[2], riseMax[2], inner95[2], innerMax[2];
  int nA[2], nN[2];
} climbSweep_t;

static const float sweepLow[3] = { 0.20f, 0.30f, 0.40f };
static const float sweepHigh[3] = { 0.50f, 0.55f, 0.60f };   /* above VZ_H_TIMING_REF (user decision 2026-10-12) */

static void climbLandSweep(const float *hts, float camWakeS, float camConvS, climbSweep_t *o)
{
  const float caps[] = { 1.10f, 1.65f };
  static float impA[2][2000], riseA[2][2000], impN[2][2000];
  memset(o, 0, sizeof(*o));
  for (int a = 0; a < 2; a++) {
    for (int h = 0; h < 3; h++) {
      for (int k = 0; k < 5; k++) {
        for (int b = 0; b < 5; b++) {
          vzConfig_t c;
          vzDefaultConfig(&c);
          c.capLo = caps[a];
          vzConfigSetHeight(&c, hts[h]);
          vzSimConfig_t sc;
          vzSimDefaultConfig(&sc);
          sc.kTrue = c.kPrior * (0.88f + 0.06f * (float)k);
          sc.accAirG = c.bAir - 0.020f + 0.010f * (float)b;
          sc.accNoiseG = 0.02f + 0.0075f * (float)((k + b) % 3);   /* up to 0.035 g: SPIN's 0.040 g gate passes */
          sc.camWakeS = camWakeS;
          if (camConvS > 0.0f) {
            sc.camConvS = camConvS;
          }
          const uint32_t seed = 900u + (uint32_t)(((a * 3 + h) * 5 + k) * 5 + b);
          harness_t hp = harnessDefault();
          hp.fsmLand = false;
          hp.maxS = 4.5f;
          run_t r0;
          runSortie(&c, &sc, seed, &hp, &r0);
          if (r0.liftDet < 0.0f) {
            continue;   /* RAMP ended without a lift-off (offset -20 mg): nothing to land from */
          }
          /* 12 LAND times 0.05-1.70 s after the detected lift-off (the A2a
           * sweep; its impact and rise statistics), then 8 more 0.03-0.24 s
           * after it (A2b: inside tHoldTauS T_ref is still T_lo and the
           * detector is armed from the first LAND step, in the air). */
          for (int j = 0; j < 20; j++) {
            hp.landAtS = r0.liftDet + (j < 12 ? 0.05f + 0.15f * (float)j : 0.03f + 0.03f * (float)(j - 12));
            hp.maxS = 25.0f;
            run_t r;
            runSortie(&c, &sc, seed, &hp, &r);
            o->runs++;
            o->armed += r.landAboveGround;
            o->tdHigh += r.tdHigh;
            o->viol += r.violations;
            o->ended += endedOk(&r);
            const bool climb = r.holdT < 0.0f && r.landT0 >= 0.0f && r.last.landReason == VZ_R_CMD_LAND;
            if (climb) {
              o->fromClimb++;
              o->worstImpact = fmaxf(o->worstImpact, r.impactV);
              o->worstRise = fmaxf(o->worstRise, r.hMax - r.hAtLandCmd);
              if (j < 12 && o->nA[a] < 2000) {
                impA[a][o->nA[a]] = r.impactV;
                riseA[a][o->nA[a]] = r.hMax - r.hAtLandCmd;
                o->nA[a]++;
                if (k >= 1 && k <= 3 && b >= 1 && b <= 3) {
                  impN[a][o->nN[a]++] = r.impactV;
                }
              }
            }
            if (caps[a] > 1.5f) {
              o->worstImpactCap = fmaxf(o->worstImpactCap, r.impactV);
            }
            o->worstTdH = fmaxf(o->worstTdH, r.tdTrueH);
          }
        }
      }
    }
  }
  for (int a = 0; a < 2; a++) {
    const int n = o->nA[a], m = o->nN[a];
    o->imp50[a] = pct(impA[a], n, 0.5f);
    o->imp95[a] = pct(impA[a], n, 0.95f);
    o->impMax[a] = pct(impA[a], n, 1.0f);
    o->rise50[a] = pct(riseA[a], n, 0.5f);
    o->rise95[a] = pct(riseA[a], n, 0.95f);
    o->riseMax[a] = pct(riseA[a], n, 1.0f);
    o->inner95[a] = m > 0 ? pct(impN[a], m, 0.95f) : 0.0f;
    o->innerMax[a] = m > 0 ? pct(impN[a], m, 1.0f) : 0.0f;
  }
}

static void printClimbSweep(const char *label, const climbSweep_t *o)
{
  printf("info: LAND from CLIMB, %s: %d LANDs (%d from CLIMB), %.1f s of armed LAND above 5 cm, touchdowns above 5 cm %d (highest declared at %.3f m)\n",
         label, o->runs, o->fromClimb, o->armed, o->tdHigh, o->worstTdH);
  for (int a = 0; a < 2; a++) {
    printf("info:   capLo %.2f, LAND 0.05-1.70 s: from CLIMB n %d impact p50 %.2f p95 %.2f max %.2f m/s; rise after LAND p50 %.3f p95 %.3f max %.3f m; inner (K 0.94-1.06, -10..+10 mg) n %d impact p95 %.2f max %.2f\n",
           a == 0 ? 1.10f : 1.65f, o->nA[a], o->imp50[a], o->imp95[a], o->impMax[a], o->rise50[a], o->rise95[a],
           o->riseMax[a], o->nN[a], o->inner95[a], o->innerMax[a]);
  }
}

static void testNoFalseTouchdown(void)
{
  static climbSweep_t o;
  climbLandSweep(sweepLow, -1.0f, 0.0f, &o);
  CHECK(o.tdHigh == 0, "LAND mid-climb, detector armed above 5 cm: no touchdown declared (%d in %d sorties)", o.tdHigh,
        o.runs);
  /* A2a had 92 s (T_ref = T_lo everywhere). A2b arms the detector near the
   * ground except inside tHoldTauS and after a capped step: the 0.03-0.24 s
   * rows keep the proof's exposure. */
  CHECK(o.armed >= 60.0f, "armed LAND above 5 cm exposure %.1f s (>= 60 s)", o.armed);
  CHECK(o.fromClimb > 0 && o.viol == 0 && o.ended == o.runs,
        "LAND from CLIMB covered (%d), ceilings held (%d), all ended (%d/%d)", o.fromClimb, o.viol, o.ended, o.runs);
  /* Non-regression against A2a (T_ref = T_lo; 1.45 m/s, 0.30 m at 149.5k):
   * the CLIMB average may not make the worst corner worse than A2a's impact,
   * and its rise is bounded (0.48 m at -20 mg / K -12 %, docs/autonomy-a2.md 11.2). */
  CHECK(o.worstImpact <= 1.45f, "LAND from CLIMB, every row: impact max %.2f m/s (<= 1.45, A2a 1.45)", o.worstImpact);
  CHECK(o.worstRise <= 0.50f, "LAND from CLIMB, every row: rise after LAND max %.3f m (<= 0.50)", o.worstRise);
  CHECK(o.inner95[1] <= 0.95f, "LAND from CLIMB, capLo 1.65, K 0.94-1.06, -10..+10 mg: impact p95 %.2f m/s (<= 0.95)",
        o.inner95[1]);
  printClimbSweep("no camera", &o);
  static climbSweep_t oc;
  climbLandSweep(sweepLow, 0.0f, 0.0f, &oc);   /* camera awake from the first command, observer healthy 0.3 s after lift-off */
  printClimbSweep("camera awake, observer healthy 0.3 s after lift-off", &oc);
  climbLandSweep(sweepLow, 0.0f, 1.3f, &oc);
  printClimbSweep("camera awake, observer healthy 1.3 s after lift-off", &oc);

  /* The same sweep at the targets above 0.40 m (0.50/0.55/0.60): the climb
   * lasts up to ~1.7 s, so the LAND times reach a drone 0.4-0.6 m up and
   * still climbing. No false touchdown either; a LAND from that climb
   * falls harder (the estimate-independent ceiling from 0.95 T_ref, falling
   * 10 %/s): measured without a camera impact max 1.71 m/s, capLo 1.65
   * inner p95 1.15 (0.20-0.40 m: 1.39, 0.84), rise max 0.36 m; with the
   * camera healthy 0.3 s after the lift-off max 1.37, inner p95 1.01. */
  static climbSweep_t oh;
  climbLandSweep(sweepHigh, -1.0f, 0.0f, &oh);
  CHECK(oh.tdHigh == 0 && oh.armed >= 60.0f,
        "targets 0.50-0.60 m, LAND mid-climb, detector armed above 5 cm (%.1f s): no touchdown declared (%d in %d)",
        oh.armed, oh.tdHigh, oh.runs);
  CHECK(oh.fromClimb > 0 && oh.viol == 0 && oh.ended == oh.runs,
        "targets 0.50-0.60 m: LAND from CLIMB covered (%d), ceilings held (%d), all ended (%d/%d)", oh.fromClimb,
        oh.viol, oh.ended, oh.runs);
  CHECK(oh.worstImpact <= 1.90f && oh.worstRise <= 0.40f && oh.inner95[1] <= 1.25f,
        "targets 0.50-0.60 m, LAND from CLIMB: impact max %.2f m/s (<= 1.90), rise max %.3f m (<= 0.40), capLo 1.65 "
        "inner p95 %.2f (<= 1.25)", oh.worstImpact, oh.worstRise, oh.inner95[1]);
  printClimbSweep("targets 0.50-0.60 m, no camera", &oh);
  climbLandSweep(sweepHigh, 0.0f, 0.0f, &oh);
  CHECK(oh.tdHigh == 0, "targets 0.50-0.60 m, camera awake: no touchdown declared above 5 cm (%d in %d)", oh.tdHigh,
        oh.runs);
  printClimbSweep("targets 0.50-0.60 m, camera awake, observer healthy 0.3 s after lift-off", &oh);
}

static bool reachPhase(vzPhase_t target, vzState_t *st, vzSimState_t *sim, const vzSimConfig_t *sc,
                       const vzConfig_t *c, float *applied, vzOutput_t *o);

/* The external estimate (section 4.10). */
static void testCamera(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  warmCamera(&sc);

  /* Offsets the accelerometer cannot see are gone once the camera is
   * healthy: a 6 s HOLD at -10 / +20 mg stays near the target (at -20 mg
   * the lift-off is not detected: RAMP ends at its plateau). */
  const float resid[] = { -0.010f, 0.020f };
  for (int j = 0; j < 2; j++) {
    sc.accAirG = c.bAir + resid[j];
    sc.camVzOffset = 0.0f;
    int ok = 0, onCam = 0, viol = 0, held = 0;
    float lo = 9.0f, hi = 0.0f;
    for (int i = 0; i < 20; i++) {
      harness_t hp = harnessFlight();
      hp.fsmLand = false;
      hp.landAtHoldS = 6.0f;
      run_t r;
      runSortie(&c, &sc, 2000u + (uint32_t)i, &hp, &r);
      viol += r.violations;
      if (r.holdT < 0.0f) {
        continue;
      }
      held++;
      ok += r.holdHMax <= 0.45f && r.holdHMin >= 0.15f && r.last.touchdown;
      onCam += r.camHoldS >= 5.9f;
      lo = fminf(lo, r.holdHMin);
      hi = fmaxf(hi, r.holdHMax);
    }
    CHECK(held >= 15 && ok == held && onCam == held && viol == 0,
          "camera, residual %+.0f mg: a 6 s HOLD on the camera stays within 0.15..0.45 m (%d/%d, %.2f..%.2f m), on the camera throughout (%d), ceilings held (%d)",
          resid[j] * 1000.0f, ok, held, lo, hi, onCam, viol);
    printf("info: camera, residual %+.0f mg, 6 s HOLD: true h %.2f..%.2f m (%d/20 reached HOLD)\n", resid[j] * 1000.0f,
           lo, hi, held);
  }

  /* The loop holds the camera's velocity at 0, so a camera offset c moves
   * the drone at -c: here c = -0.05 m/s (the camera reads a descent). */
  sc.accAirG = c.bAir;
  sc.camVzOffset = -0.05f;
  {
    int ok = 0;
    float rMin = 9.0f, rMax = -9.0f;
    for (int i = 0; i < 10; i++) {
      harness_t hp = harnessFlight();
      hp.fsmLand = false;
      hp.landAtHoldS = 6.0f;
      run_t r;
      runSortie(&c, &sc, 2100u + (uint32_t)i, &hp, &r);
      const float rate = (r.hAtLandCmd - r.hHold1) / 5.0f;
      ok += fabsf(rate - 0.05f) <= 0.02f;
      rMin = fminf(rMin, rate);
      rMax = fmaxf(rMax, rate);
    }
    CHECK(ok == 10, "camera offset c -0.05 m/s: the drone climbs at 0.05 +/- 0.02 m/s in HOLD (%d/10, %.3f..%.3f)", ok,
          rMin, rMax);
  }
  sc.camVzOffset = 0.0f;

  /* Without the camera on real inputs: HOLD_NO_CAM at holdNoCamS (7 s since the later of the HOLD entry and the last
   * step with the camera estimate in use, 2026-10-15); on the sim (fromSim) the rule is off and the holdMaxS backstop
   * lands (7.5 s here, so the accelerometer-only estimate stays in range). */
  {
    vzConfig_t c0 = c;
    vzSimConfig_t nc;
    vzSimDefaultConfig(&nc);
    noOffset(&c0, &nc);
    c0.holdMaxS = 7.5f;
    harness_t hp = harnessFlight();
    hp.fsmLand = false;
    run_t r;
    runSortie(&c0, &nc, 2200u, &hp, &r);
    CHECK(r.last.landReason == VZ_R_HOLD_NO_CAM && fabsf(r.landT0 - r.holdT - c.holdNoCamS) <= 0.002f,
          "no camera, real inputs: LAND HOLD_NO_CAM %.3f s after HOLD (%s)", r.landT0 - r.holdT,
          vzReasonName(r.last.landReason));
    hp.flight = false;
    runSortie(&c0, &nc, 2200u, &hp, &r);
    CHECK(r.last.landReason == VZ_R_HOLD_TIMEOUT && fabsf(r.landT0 - r.holdT - c0.holdMaxS) <= 0.002f,
          "no camera, sim inputs: no camera rule, HOLD_TIMEOUT backstop %.3f s after HOLD (%s)", r.landT0 - r.holdT,
          vzReasonName(r.last.landReason));
    /* camera lost for good 4 s after the lift-off (HOLD + ~3.2 s): holdNoCamS after its last use (3 s of HOLD
     * until 2026-10-15) */
    vzSimConfig_t ls = sc;
    ls.camLossS = 4.0f;
    ls.camLossLenS = 100.0f;
    hp = harnessFlight();
    hp.fsmLand = false;
    runSortie(&c, &ls, 2201u, &hp, &r);
    const float want = fmaxf(r.liftTrue + 4.0f, r.holdT) + c.holdNoCamS;
    CHECK(r.last.landReason == VZ_R_HOLD_NO_CAM && fabsf(r.landT0 - want) <= 0.006f,
          "camera lost 4 s after lift-off: HOLD_NO_CAM holdNoCamS after its last use (%.3f s after lift-off, HOLD %.3f "
          "s, %s)", r.landT0 - r.liftTrue, r.landT0 - r.holdT, vzReasonName(r.last.landReason));
  }

  /* Bad values: never a CUT, never used. */
  {
    vzState_t st;
    vzSimState_t sim;
    vzOutput_t o;
    float applied;
    vzSimConfig_t nc;
    vzSimDefaultConfig(&nc);
    const struct { bool healthy, valid; float vz, bias; bool use; const char *what; } cs[] = {
      { true, true, 0.0f, 0.0f, true, "valid" },
      { true, true, NAN, 0.0f, false, "camVz NaN" },
      { true, true, 0.0f, INFINITY, false, "camBias Inf" },
      { true, true, 5.1f, 0.0f, false, "camVz 5.1 m/s" },
      { true, true, 0.0f, -0.051f, false, "camBias -51 mg" },
      { false, true, 0.0f, 0.0f, false, "not healthy" },
      { true, false, 0.0f, 0.0f, false, "camVz not valid" },
    };
    for (size_t i = 0; i < sizeof(cs) / sizeof(cs[0]); i++) {
      reachPhase(VZ_HOLD, &st, &sim, &nc, &c, &applied, &o);
      bool used = false, cut = false;
      for (int k = 0; k < 50; k++) {
        vzInput_t in = { .dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .gyroMaxDps = 0.5f,
                         .vbat = sim.vbat, .thrustApplied = applied, .fromSim = true, .camHealthy = cs[i].healthy,
                         .camVzValid = cs[i].valid, .camVz = cs[i].vz, .camBias = cs[i].bias };
        vzStep(&st, &in, &o);
        vzSimStep(&sim, &nc, o.thrust, DT);
        applied = o.thrust;
        used = used || o.camHealthy;
        cut = cut || o.phase == VZ_CUT;
      }
      CHECK(!cut && used == cs[i].use && isfinite(o.v) && isfinite(o.thrust),
            "HOLD with %s: no CUT, %s (%s)", cs[i].what, cs[i].use ? "used" : "not used", vzPhaseName(o.phase));
    }
    /* before lift-off a healthy input is not used */
    reachPhase(VZ_RAMP, &st, &sim, &nc, &c, &applied, &o);
    vzInput_t in = { .dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .gyroMaxDps = 0.5f,
                     .vbat = sim.vbat, .thrustApplied = applied, .fromSim = true, .camHealthy = true,
                     .camVzValid = true, .camVz = 0.3f, .camBias = 0.0f };
    vzStep(&st, &in, &o);
    CHECK(o.phase == VZ_RAMP && !o.camHealthy && o.v == 0.0f, "RAMP: a healthy camera is not used before lift-off");
  }
}

/* Drives one sortie to the first step of 'target' and returns the state. */
/* LAND from HOLD on the camera (A2b hops, 2026-10-12: 0.8-1.0 m/s at the
 * take-off height, then powered bounces while the touchdown thrust condition
 * waited ~2 s for the ceiling): the ceiling holds landTop for landFlatS, so
 * the loop flies the slow descent, and the first contact ends LAND. */
static void testLandAfterHold(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  warmCamera(&sc);
  sc.accAirG = c.bAir;
  const int n = 40;
  int slow = 0, td = 0, high = 0, viol = 0, ended = 0;
  float worstImpact = 0.0f, worstAfter = 0.0f, worstLand = 0.0f;
  for (int i = 0; i < n; i++) {
    harness_t hp = harnessFlight();
    hp.fsmLand = false;
    hp.landAtHoldS = 2.0f;
    run_t r;
    runSortie(&c, &sc, 3100u + (uint32_t)i, &hp, &r);
    slow += r.landSlow;
    td += r.last.touchdown;
    high += r.tdHigh;
    viol += r.violations;
    ended += r.doneAfterContact >= 0.0f;
    worstImpact = fmaxf(worstImpact, r.impactV);
    worstAfter = fmaxf(worstAfter, r.doneAfterContact);
    worstLand = fmaxf(worstLand, r.landS);
  }
  CHECK(slow == n, "LAND from HOLD on the camera starts at landTop (%d/%d)", slow, n);
  CHECK(td == n && ended == n && high == 0 && viol == 0,
        "every LAND ends in a touchdown after contact (%d, %d/%d), none declared above 5 cm (%d), ceilings held (%d)",
        td, ended, n, high, viol);
  CHECK(worstImpact <= 0.45f, "LAND from HOLD: impact max %.2f m/s (<= 0.45)", worstImpact);
  /* The sim's jolt (0.5 g per m/s) stays under tdNearJoltG at these speeds,
   * so here the stall rule ends LAND; the hops' contacts read 2-4 g per m/s. */
  CHECK(worstAfter <= 0.60f, "LAND from HOLD: DONE at most %.3f s after the first contact (<= 0.60)", worstAfter);
  printf("info: LAND from HOLD on the camera: impact <= %.2f m/s, DONE <= %.3f s after contact, LAND <= %.2f s\n",
         worstImpact, worstAfter, worstLand);
}

/* The near-ground bump rule, open loop on a real LAND: a 0.8 g step on accZ
 * below tdNearH ends LAND at any thrust; the same step higher up only sets
 * the contact latch. */
static void testNearJolt(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  for (int low = 0; low < 2; low++) {
    vzState_t st;
    vzSimState_t sim;
    vzOutput_t o;
    float applied;
    if (!reachPhase(VZ_HOLD, &st, &sim, &sc, &c, &applied, &o)) {
      CHECK(false, "reached HOLD");
      return;
    }
    int spike = -1, doneAt = -1;
    bool highThrust = false, contactSeen = false;
    float hAt = 0.0f;
    for (int k = 0; k < 6000 && o.phase != VZ_DONE && o.phase != VZ_CUT; k++) {
      const bool arm = o.phase == VZ_LAND && !sim.onGround &&
                       (low ? (o.h < c.tdNearH - 0.03f) : (o.tPhase > 0.02f && o.h > c.tdNearH + 0.05f));
      if (spike < 0 && arm) {
        spike = k;
        highThrust = !o.tdThrustOk;
        hAt = o.h;
      }
      const bool inSpike = spike >= 0 && k - spike < 30;
      vzInput_t in = { .dt = DT, .cmd = VZ_CMD_LAND, .accZ = sim.accZ + (inSpike ? 0.8f : 0.0f), .gravZ = 1.0f,
                       .gyroMaxDps = 0.5f, .vbat = sim.vbat, .thrustApplied = applied, .fromSim = true };
      vzStep(&st, &in, &o);
      contactSeen = contactSeen || (inSpike && o.contact);
      if (o.phase == VZ_DONE && doneAt < 0) {
        doneAt = k;
      }
      vzSimStep(&sim, &sc, o.thrust, DT);
      applied = o.thrust;
      if (spike >= 0 && k - spike >= 30 && !low) {
        break;
      }
    }
    if (low) {
      CHECK(spike >= 0 && highThrust && doneAt >= spike && doneAt - spike < 30 && o.reason == VZ_R_TOUCHDOWN_JOLT,
            "bump at est. h %.3f m with the thrust above tdThrust: TOUCHDOWN_JOLT %d ms after it (reason %s)", hAt,
            doneAt - spike, vzReasonName(o.reason));
    } else {
      CHECK(spike >= 0 && doneAt < 0 && o.phase == VZ_LAND && contactSeen,
            "bump at est. h %.3f m (above tdNearH): still in LAND, contact latched (%d)", hAt, contactSeen);
    }
  }
}

static bool reachPhase(vzPhase_t target, vzState_t *st, vzSimState_t *sim, const vzSimConfig_t *sc,
                       const vzConfig_t *c, float *applied, vzOutput_t *o)
{
  vzInit(st, c);
  vzSimInit(sim, sc, 42u);
  *applied = 0.0f;
  memset(o, 0, sizeof(*o));
  if (target == VZ_IDLE) {
    return true;
  }
  for (int k = 0; k < 20000; k++) {
    vzCmd_t cmd = VZ_CMD_TAKEOFF;
    if (target == VZ_LAND && o->phase == VZ_HOLD && o->tPhase > 0.6f) {
      cmd = VZ_CMD_LAND;
    }
    if (target == VZ_DONE && (o->phase == VZ_HOLD || o->phase == VZ_LAND)) {
      cmd = VZ_CMD_LAND;
    }
    vzInput_t in = { .dt = DT, .cmd = cmd, .accZ = sim->accZ, .gravZ = 1.0f, .gyroMaxDps = 0.5f,
                     .vbat = sim->vbat, .thrustApplied = *applied, .fromSim = true };
    vzStep(st, &in, o);
    vzSimStep(sim, sc, o->thrust, DT);
    *applied = o->thrust;
    if (o->phase == target && (target != VZ_LAND || o->tPhase > 0.5f) && (target != VZ_HOLD || o->tPhase > 0.6f) &&
        (target != VZ_RAMP || o->tPhase > 0.2f) && (target != VZ_SPIN || o->tPhase > 0.4f) &&
        (target != VZ_CLIMB || o->tPhase > 0.1f)) {
      return true;
    }
  }
  return false;
}
static const vzPhase_t allPhases[] = { VZ_IDLE, VZ_SPIN, VZ_RAMP, VZ_CLIMB, VZ_HOLD, VZ_LAND, VZ_DONE };

/* 9.1 #8: NaN or Inf in each input field, in each phase. */
static void testNonFinite(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  const float bad[] = { NAN, INFINITY, -INFINITY };
  int cases = 0, ok = 0, latched = 0;
  for (size_t p = 0; p < sizeof(allPhases) / sizeof(allPhases[0]); p++) {
    for (int field = 0; field < 7; field++) {
      for (int b = 0; b < 3; b++) {
        vzState_t st;
        vzSimState_t sim;
        vzOutput_t o;
        float applied;
        if (!reachPhase(allPhases[p], &st, &sim, &sc, &c, &applied, &o)) {
          CHECK(false, "reached phase %s", vzPhaseName(allPhases[p]));
          continue;
        }
        vzInput_t in = { .dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .gyroMaxDps = 0.5f,
                         .vbat = sim.vbat, .thrustApplied = applied, .fromSim = true };
        float *f[] = { &in.dt, &in.accZ, &in.gravZ, &in.tiltDeg, &in.gyroMaxDps, &in.vbat, &in.thrustApplied };
        *f[field] = bad[b];
        vzStep(&st, &in, &o);
        cases++;
        ok += o.phase == VZ_CUT && o.thrust == 0.0f && o.reason == VZ_R_NAN_INPUT && isfinite(o.ceiling);
        in = (vzInput_t){ .dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.8f,
                          .thrustApplied = 0.0f };
        for (int k = 0; k < 100; k++) {
          vzStep(&st, &in, &o);
        }
        latched += o.phase == VZ_CUT && o.thrust == 0.0f;
      }
    }
  }
  CHECK(ok == cases, "non-finite input in each field and phase: CUT, thrust 0, NAN_INPUT in that step (%d/%d)", ok,
        cases);
  CHECK(latched == cases, "and CUT stays latched (%d/%d)", latched, cases);

  /* Huge finite inputs never turn into NaN. */
  vzState_t st;
  vzSimState_t sim;
  vzOutput_t o;
  float applied;
  reachPhase(VZ_HOLD, &st, &sim, &sc, &c, &applied, &o);
  bool finiteOut = true;
  for (int k = 0; k < 3000; k++) {
    vzInput_t in = { .dt = DT, .cmd = k < 1000 ? VZ_CMD_TAKEOFF : VZ_CMD_LAND, .accZ = (k % 2) ? 1e30f : -1e30f,
                     .gravZ = -1e30f, .tiltDeg = 1e30f, .gyroMaxDps = -1e30f, .vbat = 1e30f, .thrustApplied = 1e30f };
    vzStep(&st, &in, &o);
    finiteOut = finiteOut && isfinite(o.thrust) && isfinite(o.h) && isfinite(o.v) && o.thrust <= o.ceiling &&
                o.ceiling <= VZ_THRUST_ABS_MAX;
  }
  CHECK(finiteOut, "huge finite inputs: outputs stay finite and under the ceiling (%s)", vzPhaseName(o.phase));
}

/* 9.1 #9 (kill): CUT in each phase gives thrust 0 in that step. */
static void testCut(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  int ok = 0, latched = 0, reinit = 0, n = 0;
  for (size_t p = 0; p < sizeof(allPhases) / sizeof(allPhases[0]); p++) {
    vzState_t st;
    vzSimState_t sim;
    vzOutput_t o;
    float applied;
    reachPhase(allPhases[p], &st, &sim, &sc, &c, &applied, &o);
    const float before = o.thrust;
    vzInput_t in = { .dt = DT, .cmd = VZ_CMD_CUT, .accZ = sim.accZ, .gravZ = 1.0f, .vbat = sim.vbat,
                     .thrustApplied = applied };
    vzStep(&st, &in, &o);
    n++;
    ok += o.phase == VZ_CUT && o.thrust == 0.0f && o.reason == VZ_R_CMD_CUT;
    in.cmd = VZ_CMD_TAKEOFF;
    for (int k = 0; k < 500; k++) {
      vzStep(&st, &in, &o);
    }
    latched += o.phase == VZ_CUT && o.thrust == 0.0f;
    vzInit(&st, &c);
    vzStep(&st, &in, &o);
    reinit += o.phase == VZ_SPIN;
    if (allPhases[p] == VZ_HOLD) {
      CHECK(before > 30000.0f, "the HOLD case was flying (%.0f)", before);
    }
  }
  CHECK(ok == n, "VZ_CMD_CUT in each phase: thrust 0 in that step (%d/%d)", ok, n);
  CHECK(latched == n, "CUT latched until vzInit (%d/%d)", latched, n);
  CHECK(reinit == n, "vzInit clears it (%d/%d)", reinit, n);
}

/* 9.1 #10: NONE while flying lands; LAND in IDLE is DONE. */
static void testCmdMissing(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  const vzPhase_t ph[] = { VZ_SPIN, VZ_RAMP, VZ_CLIMB, VZ_HOLD };
  for (int p = 0; p < 4; p++) {
    vzState_t st;
    vzSimState_t sim;
    vzOutput_t o;
    float applied;
    reachPhase(ph[p], &st, &sim, &sc, &c, &applied, &o);
    const float before = o.thrust;
    vzInput_t in = { .dt = DT, .cmd = VZ_CMD_NONE, .accZ = sim.accZ, .gravZ = 1.0f, .vbat = sim.vbat,
                     .thrustApplied = applied };
    vzStep(&st, &in, &o);
    CHECK(o.phase == VZ_LAND && o.landReason == VZ_R_CMD_MISSING && o.reason == VZ_R_CMD_MISSING,
          "NONE in %s: LAND/abort with CMD_MISSING (%s %s)", vzPhaseName(ph[p]), vzPhaseName(o.phase),
          vzReasonName(o.landReason));
    CHECK(o.thrust <= before, "and the thrust does not rise (%.0f -> %.0f)", before, o.thrust);
    in.cmd = VZ_CMD_TAKEOFF;   /* a TAKEOFF later never leaves LAND */
    vzStep(&st, &in, &o);
    CHECK(o.phase == VZ_LAND, "TAKEOFF after it stays in LAND (%s)", vzPhaseName(o.phase));
  }
  vzState_t st;
  vzOutput_t o;
  vzInit(&st, &c);
  vzInput_t in = { .dt = DT, .cmd = VZ_CMD_LAND, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.9f };
  vzStep(&st, &in, &o);
  CHECK(o.phase == VZ_DONE && o.landReason == VZ_R_CMD_LAND && o.thrust == 0.0f, "LAND in IDLE: DONE CMD_LAND");
  in.cmd = (vzCmd_t)77;   /* garbage command: treated as NONE */
  vzInit(&st, &c);
  vzStep(&st, &in, &o);
  CHECK(o.phase == VZ_IDLE && o.thrust == 0.0f, "an unknown command in IDLE does nothing");
}

/* 9.1 #11: dt of 0, -1 ms and 100 ms. */
static void testDt(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzState_t st;
  vzOutput_t o;
  vzInit(&st, &c);
  vzInput_t in = { .dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.9f };
  vzStep(&st, &in, &o);
  const float dts[] = { 0.0f, -0.001f, 0.100f };
  const float want[] = { VZ_DT_MIN_S, VZ_DT_MIN_S, VZ_DT_MAX_S };
  for (int i = 0; i < 3; i++) {
    const float t0 = o.tPhase;
    in.dt = dts[i];
    vzStep(&st, &in, &o);
    CHECK(isfinite(o.thrust) && o.phase == VZ_SPIN && fabsf((o.tPhase - t0) - want[i]) < 1e-6f,
          "dt %.3f s is used as %.4f s (%.6f)", dts[i], want[i], o.tPhase - t0);
  }
  /* A whole sortie at 100 ms per call stays finite and within its ceilings. */
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  vzSimState_t sim;
  vzInit(&st, &c);
  vzSimInit(&sim, &sc, 3u);
  float applied = 0.0f;
  bool good = true;
  for (int k = 0; k < 4000; k++) {
    in = (vzInput_t){ .dt = (k % 3 == 0) ? 0.1f : ((k % 3 == 1) ? 0.0f : -0.001f), .cmd = VZ_CMD_TAKEOFF,
                      .accZ = sim.accZ, .gravZ = 1.0f, .vbat = sim.vbat, .thrustApplied = applied };
    vzStep(&st, &in, &o);
    good = good && isfinite(o.thrust) && isfinite(o.h) && o.thrust <= o.ceiling && o.phase != VZ_CUT;
    vzSimStep(&sim, &sc, o.thrust, (k % 3 == 0) ? 0.005f : 0.0002f);
    applied = o.thrust;
  }
  CHECK(good, "odd dt values for a whole sortie: finite, within the ceilings, no CUT");
}

/* SPIN_GYRO needs the gyro over spinGyroMax for spinGyroHoldS (docs/autonomy-a2b.md 19.7): one 1 ms 40 dps
 * spike (a vibration sample) does not abort; 20 ms at 16 dps does; 19 ms does not; a spike's sample stays out of g0;
 * spinGyroHoldS 0 is the old one-sample rule. Steps at 1 ms over a SPIN in its g0 window. */
static int spinRun(float holdS, int spikeAtMs, int spikeMs, float dps, int spike2AtMs, vzOutput_t *o, vzState_t *st)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  c.spinGyroHoldS = holdS;
  vzInit(st, &c);
  memset(o, 0, sizeof(*o));
  int abortAt = -1;
  for (int k = 0; k < 1000; k++) {
    const bool spike = (k >= spikeAtMs && k < spikeAtMs + spikeMs) || (spike2AtMs >= 0 && k == spike2AtMs);
    vzInput_t in = { .dt = 0.001f, .cmd = VZ_CMD_TAKEOFF, .accZ = spike ? 1.5f : 1.0f, .gravZ = 1.0f,
                     .tiltDeg = 0.5f, .gyroMaxDps = spike ? dps : 3.0f, .vbat = 3.9f };
    vzStep(st, &in, o);
    if (abortAt < 0 && o->reason == VZ_R_SPIN_GYRO) {
      abortAt = k;
    }
  }
  return abortAt;
}

static void testSpinGyroHold(void)
{
  vzOutput_t o;
  vzState_t st;
  vzConfig_t d;
  vzDefaultConfig(&d);
  CHECK(d.spinGyroHoldS == 0.020f && d.spinGyroMax == 15.0f, "defaults 15 dps for 20 ms");
  /* the g0 window is the SPIN's last 0.5 s (0.3..0.8 s) */
  int a = spinRun(d.spinGyroHoldS, 500, 1, 40.0f, -1, &o, &st);
  CHECK(a < 0 && st.liftedOff == false && o.phase != VZ_LAND && o.phase != VZ_DONE,
        "one 1 ms 40 dps spike in the window: no abort (phase %d)", (int)o.phase);
  CHECK(fabsf(st.g0 - 1.0f) < 1e-4f, "the spike's 1.5 g sample is not in g0 (%.5f)", st.g0);
  a = spinRun(d.spinGyroHoldS, 500, 19, 16.0f, -1, &o, &st);
  CHECK(a < 0, "19 ms at 16 dps: no abort");
  a = spinRun(d.spinGyroHoldS, 500, 20, 16.0f, -1, &o, &st);
  CHECK(a == 519, "20 ms at 16 dps: SPIN_GYRO on its 20th step (%d)", a);
  a = spinRun(d.spinGyroHoldS, 500, 15, 16.0f, 520, &o, &st);
  CHECK(a < 0, "15 ms, a gap, then one sample: the count restarts, no abort");
  a = spinRun(d.spinGyroHoldS, 200, 30, 40.0f, -1, &o, &st);
  CHECK(a < 0, "30 ms at 40 dps before the window (0.2 s): no abort, as before");
  /* spinGyroHoldS 0: the old rule, one sample */
  a = spinRun(0.0f, 500, 1, 40.0f, -1, &o, &st);
  CHECK(a == 500, "spinGyroHoldS 0: one sample aborts, as before (%d)", a);
  vzConfig_t c = d;
  c.spinGyroHoldS = -0.001f;
  CHECK(!vzConfigValid(&c), "spinGyroHoldS < 0 refused");
  c.spinGyroHoldS = 0.21f;
  CHECK(!vzConfigValid(&c), "spinGyroHoldS > 0.2 refused");
  c.spinGyroHoldS = NAN;
  CHECK(!vzConfigValid(&c), "NaN refused");
  c.spinGyroHoldS = 0.0f;
  CHECK(vzConfigValid(&c), "0 accepted");
}

/* 9.1 #12: hard limits. */
static void testConfigValid(void)
{
  typedef struct { size_t off; float bad; } lim_t;
#define L(f, v) { offsetof(vzConfig_t, f), (v) }
  const lim_t lims[] = {
    L(hTarget, 0.19f), L(hTarget, 1.21f), L(kPrior, 119000.0f), L(kPrior, 181000.0f), L(bAir, 0.031f),
    L(bAir, -0.031f), L(capKhat, 0.99f), L(capKhat, 1.26f), L(capLo, 0.99f), L(capLo, 2.01f),
    L(tiltDivMin, 0.89f), L(tiltDivMin, 1.01f), L(spinThrustMax, 25001.0f), L(spinThrustMax, 0.0f),
    L(rampRate, 999.0f), L(rampRate, 30001.0f), L(rampMaxS, 0.0f), L(rampMaxS, 3.01f), L(climbMaxS, 4.51f),
    L(climbMaxS, -1.0f), L(holdMaxS, 600.5f), L(holdMaxS, 600.01f), L(holdMaxS, 0.0f), L(landStart, 0.79f), L(landStart, 1.01f),
    L(landRate, 0.019f), L(landRate, 0.31f), L(landFloor, 0.39f), L(landFloor, 0.96f), L(tdThrust, 0.49f),
    L(tdThrust, 0.91f), L(tdForce, 1.09f), L(tdForce, 2.01f), L(hMin, 0.0f), L(landVref, 0.0f),
    L(spinS, 0.0f), L(liftCusumK, -0.01f), L(kpRel, 0.0f), L(vfTauS, 0.0f), L(tdForceS, 0.0f),
    L(tdJoltG, -1.0f), L(hMax, 0.0f), L(vAbsMax, NAN), L(abortRate, INFINITY), L(khatMaxRel, 0.99f),
    L(khatMinRel, 1.01f), L(liftLatencyS, 0.0f), L(tLoMaxRel, 0.59f), L(tLoMaxRel, 1.01f),
    L(holdNoCamS, 0.0f), L(holdNoCamS, 8.01f), L(camTauS, 0.0f), L(rampTopRel, 0.79f), L(rampTopRel, 1.11f),
    L(landClimbRefRel, 0.79f), L(landClimbRefRel, 1.01f), L(landClimbRefRel, NAN), L(abortSlowRel, 0.39f),
    L(abortSlowRel, 1.01f), L(abortSlowRel, NAN), L(landFlatS, 4.51f), L(landFlatS, -0.1f),
    /* the fast LAND (2026-10-14) */
    L(landVfast, -0.61f), L(landVfast, -0.29f), L(landVfast, NAN), L(landFastH, 0.29f), L(landFastH, 1.01f),
    L(landFastBand, 0.04f), L(landFastBand, 0.51f), L(landFastMinH, 0.64f), L(landFastMinH, 1.51f),
    L(landBrakeTop, 0.99f), L(landBrakeTop, 1.051f), L(landBrakeH, 0.19f), L(landBrakeH, 0.61f),
    L(landBrakeMaxS, -0.01f), L(landBrakeMaxS, 1.01f), L(landBrakeMaxS, INFINITY),
    /* the EST_RANGE guard's grace (2026-10-22) */
    L(estGraceS, -0.01f), L(estGraceS, 3.01f), L(estGraceS, NAN),
  };
#undef L
  int rejected = 0, cut = 0;
  const int n = (int)(sizeof(lims) / sizeof(lims[0]));
  for (int i = 0; i < n; i++) {
    vzConfig_t c;
    vzDefaultConfig(&c);
    memcpy((char *)&c + lims[i].off, &lims[i].bad, sizeof(float));
    const bool v = vzConfigValid(&c);
    rejected += !v;
    if (v) {
      printf("  accepted a bad config, field at offset %u = %g\n", (unsigned)lims[i].off, lims[i].bad);
    }
    vzState_t st;
    vzOutput_t o;
    vzInit(&st, &c);
    vzInput_t in = { .dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.9f };
    vzStep(&st, &in, &o);
    cut += o.phase == VZ_CUT && o.reason == VZ_R_BAD_CONFIG && o.thrust == 0.0f;
  }
  CHECK(rejected == n, "vzConfigValid rejects every hard limit (%d/%d)", rejected, n);
  CHECK(cut == n, "vzInit with a bad config: CUT BAD_CONFIG, thrust 0 even on TAKEOFF (%d/%d)", cut, n);
  CHECK(!vzConfigValid(NULL), "NULL config rejected");
  vzConfig_t c;
  vzDefaultConfig(&c);
  c.hTarget = 0.20f;
  c.capLo = 2.0f;
  c.landFloor = c.landStart;
  c.holdMaxS = VZ_HOLD_MAX_LIMIT_S;
  c.holdNoCamS = VZ_HOLD_NO_CAM_MAX_S;
  c.tLoMaxRel = 1.0f;
  c.landClimbRefRel = 0.80f;
  c.abortSlowRel = 0.40f;
  CHECK(vzConfigValid(&c), "values on the limits are accepted");
  c.hTarget = 1.20f;
  c.climbMaxS = 4.5f;
  c.landFlatS = 4.5f;
  CHECK(vzConfigValid(&c), "hTarget 1.20, climbMaxS 4.5 and landFlatS 4.5 are accepted (the take-off cap's limits)");
  c.hTarget = 2.00f;
  CHECK(!vzConfigValid(&c), "hTarget 2.00 (the hover cap, reached by moves) is not a take-off");
  c.hTarget = 0.20f;
  c.tLoMaxRel = 0.60f;
  c.landClimbRefRel = 1.00f;
  c.abortSlowRel = 1.00f;
  CHECK(vzConfigValid(&c), "tLoMaxRel 0.60, landClimbRefRel 1.00 and abortSlowRel 1.00 are accepted");
  vzDefaultConfig(&c);
  c.landVfast = -0.60f;
  c.landFastH = 0.30f;
  c.landFastBand = 0.05f;
  c.landFastMinH = c.landFastH + c.landFastBand;
  c.landBrakeTop = 1.05f;
  c.landBrakeH = c.landSlowH;
  c.landBrakeMaxS = 1.0f;
  CHECK(vzConfigValid(&c), "the fast LAND's limits are accepted");
  vzDefaultConfig(&c);
  c.landVfast = c.landVref;
  c.landBrakeTop = 1.00f;
  c.landBrakeMaxS = 0.0f;
  c.landFastMinH = 1.50f;
  c.landFastH = 1.00f;
  c.landFastBand = 0.50f;
  c.landBrakeH = 1.00f;
  CHECK(vzConfigValid(&c), "landVfast = landVref (the fast LAND off) and the upper limits are accepted");
}

/* One sortie for the learning tests, on the sim with fromSim off as if it
 * had been flown: LAND landS after HOLD entry. Returns Khat on the first HOLD
 * step at holdLoopS (0 if never) in *khatAtLoop and the last output in *o. */
static void learnSortie(const vzConfig_t *c, const vzSimConfig_t *sc, uint32_t seed, float landS, vzState_t *st,
                        vzOutput_t *o, float *khatAtLoop)
{
  vzSimState_t sim;
  memset(o, 0, sizeof(*o));
  vzInit(st, c);
  vzSimInit(&sim, sc, seed);
  *khatAtLoop = 0.0f;
  float applied = 0.0f, t = 0.0f, holdT = -1.0f;
  for (int k = 0; k < 20000 && o->phase != VZ_DONE && o->phase != VZ_CUT; k++, t += DT) {
    vzCmd_t cmd = (holdT >= 0.0f && t - holdT > landS) ? VZ_CMD_LAND : VZ_CMD_TAKEOFF;
    vzInput_t in = { .dt = DT, .cmd = cmd, .accZ = sim.accZ, .gravZ = 1.0f, .vbat = sim.vbat,
                     .thrustApplied = applied, .fromSim = false, .camHealthy = sim.camHealthy,
                     .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias,
                     .landFastOk = true };
    vzStep(st, &in, o);
    if (o->phase == VZ_HOLD && holdT < 0.0f) {
      holdT = t;
    }
    if (o->phase == VZ_HOLD && o->tPhase >= c->holdLoopS && *khatAtLoop == 0.0f) {
      *khatAtLoop = o->khat;
    }
    vzSimStep(&sim, sc, o->thrust, DT);
    applied = o->thrust;
  }
}

/* 9.1 #13: learning the hover gain. */
static void testLearnK(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  c.kPrior = 145000.0f;
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  sc.kTrue = 150000.0f;
  vzState_t st;
  vzOutput_t o;
  float khatAtLoop;
  learnSortie(&c, &sc, 5u, 1.5f, &st, &o, &khatAtLoop);
  CHECK(o.phase == VZ_DONE && o.touchdown && o.tAir >= 1.5f && o.landReason == VZ_R_CMD_LAND,
        "learning sortie ended by touchdown after a commanded LAND (%s %s, %.2f s)", vzReasonName(o.landReason),
        vzReasonName(o.reason), o.tAir);
  CHECK(khatAtLoop > 0.0f && st.kLearn == khatAtLoop,
        "the sample is Khat on the first HOLD step at holdLoopS (%.0f, %.0f)", st.kLearn, khatAtLoop);
  float k = c.kPrior;
  CHECK(vzLearnK(&st, &k), "a qualifying sortie is learned");
  const float want = c.kPrior + 0.3f * (st.kLearn - c.kPrior);
  CHECK(fabsf(k - want) < 1.0f && st.kLearn > c.kPrior,
        "kPrior moves 0.3 toward the sample, not toward Kfrz (%.0f -> %.0f, sample %.0f, Kfrz %.0f)", c.kPrior, k,
        st.kLearn, o.kfrz);
  k = 120000.0f;
  vzLearnK(&st, &k);
  CHECK(fabsf(k - 126000.0f) < 1.0f, "at most 5 %% per sortie (%.0f)", k);
  k = 179000.0f;
  vzState_t s2 = st;
  s2.kLearn = 250000.0f;
  vzLearnK(&s2, &k);
  CHECK(k <= VZ_K_MAX, "clamped to VZ_K_MAX (%.0f)", k);

  /* A camera hold with an offset that sinks the drone (c +0.05 m/s, the
   * study's positive cases): Kfrz follows Khat down in ground effect; the
   * sample stays at the hold height. */
  vzSimConfig_t cam = sc;
  warmCamera(&cam);
  cam.camVzOffset = 0.05f;
  vzState_t s3;
  learnSortie(&c, &cam, 6u, 6.0f, &s3, &o, &khatAtLoop);
  CHECK(s3.kLearn == khatAtLoop && s3.kLearn > 0.0f && o.kfrz < (1.0f - c.kfrzBias) * s3.kLearn - 1000.0f,
        "camera hold, c +0.05 m/s: the sample stays Khat at holdLoopS (%.0f) while Kfrz falls to %.0f (%s)",
        s3.kLearn, o.kfrz, vzReasonName(o.landReason));
  k = c.kPrior;
  const bool learned = vzLearnK(&s3, &k);
  CHECK(learned == (o.landReason == VZ_R_CMD_LAND || o.landReason == VZ_R_HOLD_NO_CAM) &&
        (!learned || fabsf(k - (c.kPrior + 0.3f * (s3.kLearn - c.kPrior))) < 1.0f),
        "and it learns the sample, or refuses (%s, %.0f -> %.0f)", vzReasonName(o.landReason), c.kPrior, k);
  printf("info: learning (prior 145000, true 150000): sample %.0f, Kfrz %.0f; camera hold at c +0.05 m/s: sample %.0f, Kfrz at LAND %.0f, LAND %s, %s\n",
         st.kLearn, st.kfrz, s3.kLearn, o.kfrz, vzReasonName(o.landReason), learned ? "learned" : "refused");

  /* How LAND started decides: a commanded LAND or a camera dropout teaches;
   * the model or the estimate out of its range, or a LAND the state machine
   * did not command, does not. */
  static const vzReason_t refused[] = { VZ_R_K_RANGE, VZ_R_HOLD_CAPPED, VZ_R_EST_RANGE, VZ_R_HOLD_TIMEOUT,
                                        VZ_R_CMD_MISSING, VZ_R_CLIMB_CAPPED, VZ_R_CLIMB_TIMEOUT, VZ_R_NONE };
  int refusedOk = 0;
  const int nRefused = (int)(sizeof(refused) / sizeof(refused[0]));
  for (int i = 0; i < nRefused; i++) {
    s2 = st;
    s2.landReason = refused[i];
    k = 140000.0f;
    refusedOk += !vzLearnK(&s2, &k) && k == 140000.0f;
  }
  CHECK(refusedOk == nRefused,
        "LAND by K_RANGE, HOLD_CAPPED, EST_RANGE, HOLD_TIMEOUT, CMD_MISSING, ... is refused (%d/%d)", refusedOk,
        nRefused);
  s2 = st;
  s2.landReason = VZ_R_HOLD_NO_CAM;
  k = 140000.0f;
  CHECK(vzLearnK(&s2, &k) && k != 140000.0f, "LAND by HOLD_NO_CAM (camera dropout after the sample) is learned");
  s2 = st;
  s2.kLearn = 0.0f;
  k = 140000.0f;
  CHECK(!vzLearnK(&s2, &k), "a sortie without the sample is refused");

  s2 = st;
  s2.fromSimSeen = true;
  k = 140000.0f;
  CHECK(!vzLearnK(&s2, &k) && k == 140000.0f, "a sortie with fromSim is refused");
  s2 = st;
  s2.kFrozen = false;
  CHECK(!vzLearnK(&s2, &k), "a sortie without the HOLD gain freeze is refused");
  s2 = st;
  s2.tAir = 1.4f;
  CHECK(!vzLearnK(&s2, &k), "a short sortie is refused");
  s2 = st;
  s2.touchdown = false;
  s2.reason = VZ_R_ABORTED;
  CHECK(!vzLearnK(&s2, &k), "an abort is refused");
  s2 = st;
  s2.phase = VZ_CUT;
  CHECK(!vzLearnK(&s2, &k), "a CUT is refused");
  CHECK(!vzLearnK(&st, NULL), "NULL is refused");
}

/* 9.1 #14: the same seed gives the same run. */
static void testDeterministic(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  sc.accAirG = c.bAir - 0.005f;
  harness_t hp = harnessDefault();
  run_t a, b;
  runSortie(&c, &sc, 99u, &hp, &a);
  runSortie(&c, &sc, 99u, &hp, &b);
  CHECK(a.hMax == b.hMax && a.landS == b.landS && a.last.thrust == b.last.thrust && a.last.h == b.last.h &&
        a.liftDet == b.liftDet, "same seed, same run");
  runSortie(&c, &sc, 98u, &hp, &b);
  CHECK(a.hMax != b.hMax, "another seed, another run");
}

/* Timing jitter of the real loop (0.6..1.4 ms) changes nothing important. */
static void testJitter(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzSimConfig_t sc;
  vzSimDefaultConfig(&sc);
  noOffset(&c, &sc);
  harness_t hp = harnessDefault();
  int ok = 0;
  const int n = 20;
  for (int i = 0; i < n; i++) {
    hp.jitterSeed = 31u + (uint32_t)i;
    run_t r;
    runSortie(&c, &sc, 1000u + (uint32_t)i, &hp, &r);
    ok += r.violations == 0 && r.holdT >= 0.0f && fabsf(r.holdH - 0.30f) <= 0.15f && r.last.touchdown &&
          r.hMax < 0.45f && r.impactV < 0.8f;
  }
  CHECK(ok == n, "with 0.6..1.4 ms loop jitter every nominal sortie holds, lands and keeps its ceilings (%d/%d)", ok,
        n);
}


/* ---- the interactive hover: vertical moves, 1.20 m ------------------------- */

/* vRef and hOff 0: bit-identical to 8f51285 over whole sorties at every
 * height of that envelope (vert_ctrl_identity.h), up to 6742d91. The end of
 * battery (2026-10-14: VZ_THRUST_ABS_MAX 58000, the HOLD ceiling capLo x T_lo
 * battery-scaled) changes the reported ceiling in 23 of these 30 sorties and
 * the thrust in none (step by step against 6742d91: a ceiling that does not
 * bind changes nothing else), so the hash moves to VZ_IDENTITY_HASH_EOB. The
 * thrust changes only where a ceiling binds. */
#define VZ_IDENTITY_HASH_8F51285 0x169CEE4B871A74D9ull
#define VZ_IDENTITY_HASH_EOB     0x6E54AEE6FC4E3868ull
/* Hold and report (2026-10-15, docs/autonomy-a2b.md 22) moves it again, in 10 of the 30 sorties, every other one
 * bit-identical (checked sortie by sortie against 56bd936): the 5 "no camera" sorties (HOLD_NO_CAM 7 s after the
 * HOLD entry instead of 3 s, holdNoCamS) and the 5 "camera loss" sorties (the 2 s loss: the open loop on the
 * observer's last offset instead of bAir, its thrust under capQ). */
#define VZ_IDENTITY_HASH_HR      0x8738D2591A4F2D0Eull

static void testIdentity(void)
{
  const uint64_t h = vertCtrlIdentityHash();
  CHECK(h == VZ_IDENTITY_HASH_HR, "vRef and hOff 0: outputs bit-identical to 8f51285 but for the ceilings of the "
        "end of battery and, without the camera, hold and report (hash 0x%016llX)", (unsigned long long)h);
}

/* A sortie on the sim with a warm camera, TAKEOFF to hTarget, then a
 * script: vRef(t) / hOff(t) in HOLD, LAND at landAtS after HOLD (camera lost
 * from camLossAtS after HOLD when >= 0). */
typedef struct {
  float hTarget, kErr, accAirG, camC;
  float moveAtS, moveUpM, moveV;   /* a vertical move: from moveAtS after HOLD at moveV (m/s) for moveUpM */
  float landAtS, camLossAtS;
  uint32_t seed;
  float maxS;               /* the run's length (0: 160 s) */
} vmScen_t;

typedef struct {
  bool held;
  float hHold, hBeforeMove, hAfterMove, hMaxTrue, impactV, landS, landMaxS, peakV;
  float holdDurS;           /* HOLD's first step to the LAND's */
  vzReason_t landReason, endReason;
  vzPhase_t endPhase;
  bool landSlow, estRange;
} vmRun_t;

static void vmRun(const vmScen_t *sc, vmRun_t *r)
{
  memset(r, 0, sizeof(*r));
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, sc->hTarget);
  vzState_t st;
  vzInit(&st, &c);
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.kTrue = c.kPrior * (1.0f + sc->kErr);
  pc.accAirG = c.bAir + sc->accAirG;
  warmCamera(&pc);
  pc.camVzOffset = sc->camC;
  vzSimState_t sim;
  vzSimInit(&sim, &pc, sc->seed);
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  float applied = 0.0f, vbat = sim.vbat, holdT = -1.0f, landT = -1.0f, hOff = 0.0f;
  const int kMax = sc->maxS > 0.0f ? (int)(sc->maxS * 1000.0f) : 160000;
  for (int k = 0; k < kMax; k++) {
    const float t = (float)k * DT;
    if (k % 100 == 0) {
      vbat = sim.vbat;
    }
    const float th = holdT >= 0.0f ? t - holdT : -1.0f;
    float vRef = 0.0f;
    if (sc->moveUpM != 0.0f && th >= sc->moveAtS) {
      const float dur = fabsf(sc->moveUpM) / sc->moveV;
      if (th < sc->moveAtS + dur) {
        vRef = sc->moveUpM > 0.0f ? sc->moveV : -sc->moveV;
        hOff = vRef * (th - sc->moveAtS);
      } else {
        hOff = sc->moveUpM;
      }
    }
    if (sc->camLossAtS >= 0.0f && th >= sc->camLossAtS) {
      pc.camLossS = 0.0f;           /* the observer lost for good from now */
      pc.camLossLenS = 1000.0f;
    }
    const bool land = sc->landAtS >= 0.0f && th >= sc->landAtS;
    vzInput_t in = {
      .dt = DT, .cmd = land ? VZ_CMD_LAND : VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = 0.5f,
      .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy,
      .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias, .vRef = vRef, .hOff = hOff,
      .landFastOk = true,   /* the Mac's LAND */
    };
    vzStep(&st, &in, &o);
    if (o.phase == VZ_HOLD && holdT < 0.0f) {
      holdT = t;
      r->held = true;
      r->hHold = sim.h;
    }
    if (sc->moveUpM != 0.0f && th >= 0.0f && th < sc->moveAtS) {
      r->hBeforeMove = sim.h;
    }
    if (sc->moveUpM != 0.0f && th >= sc->moveAtS + fabsf(sc->moveUpM) / sc->moveV + 1.5f && !land) {
      r->hAfterMove = sim.h;
    }
    if (o.phase == VZ_LAND && landT < 0.0f) {
      landT = t;
      r->landReason = o.landReason;
      r->landMaxS = o.landMaxS;
      r->landSlow = o.ceiling >= 0.97f * o.tRef;
      r->holdDurS = holdT >= 0.0f ? t - holdT : 0.0f;
    }
    if (o.phase == VZ_LAND) {
      r->peakV = fmaxf(r->peakV, -sim.v);
    }
    r->estRange = r->estRange || o.landReason == VZ_R_EST_RANGE;
    vzSimStep(&sim, &pc, o.thrust, DT);
    applied = o.thrust;
    if (o.phase == VZ_DONE || o.phase == VZ_CUT) {
      r->landS = landT >= 0.0f ? t - landT : 0.0f;
      break;
    }
  }
  r->endPhase = o.phase;
  r->endReason = o.reason;
  r->hMaxTrue = sim.hMax;
  r->impactV = sim.impactV;
}

/* A vertical move: vRef 0.10 m/s for 0.20 m in HOLD on the camera: the true
 * height changes by about that; hOff keeps the estimate guard over the new
 * reference (no EST_RANGE). */
static void testVerticalMove(void)
{
  int ok = 0, n = 20;
  float lo = 9.0f, hi = -9.0f;
  for (int i = 0; i < n; i++) {
    for (int up = 0; up < 2; up++) {
      vmScen_t sc = {.hTarget = 0.80f, .kErr = 0.02f * (float)((i % 5) - 2), .accAirG = 0.001f * (float)((i % 7) - 3),
                     .moveAtS = 2.0f, .moveUpM = up ? 0.30f : -0.30f, .moveV = 0.10f, .landAtS = 9.0f,
                     .camLossAtS = -1.0f, .seed = 4100u + (uint32_t)i};
      vmRun_t r;
      vmRun(&sc, &r);
      const float d = r.hAfterMove - r.hBeforeMove;
      const float want = sc.moveUpM;
      lo = fminf(lo, d / want);
      hi = fmaxf(hi, d / want);
      ok += r.held && fabsf(d - want) <= 0.25f * fabsf(want) && !r.estRange && r.endPhase == VZ_DONE &&
            r.landReason == VZ_R_CMD_LAND && r.impactV <= 0.5f;
    }
  }
  printf("info: vertical move +/-0.30 m at 0.10 m/s from 0.80 m (K +/-4 %%, -3..+3 mg): true change %.2f..%.2f x\n",
         lo, hi);
  CHECK(ok == 2 * n, "vertical moves of +/-0.30 m: within 25 %% of the command, no EST_RANGE, slow LAND (%d/%d)",
        ok, 2 * n);
  /* hMax follows hOff: the estimate up at 1.80 m - 0.10 + 0.30 does not trip */
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, 1.20f);
  vzState_t st;
  vzInit(&st, &c);
  st.phase = VZ_HOLD;
  st.liftedOff = st.holdReached = st.started = true;
  st.vf = 3.6f;
  st.khat = st.kfrz = c.kPrior;
  st.tLo = 35000.0f;
  st.h = 2.0f;
  vzOutput_t o;
  vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.6f, .thrustApplied = 45000.0f,
                  .hOff = 0.30f};
  vzStep(&st, &in, &o);
  CHECK(o.phase == VZ_HOLD, "estimate 2.00 m, hMax 1.80 + hOff 0.30: still HOLD (%s)", vzPhaseName(o.phase));
  in.hOff = 0.0f;
  vzStep(&st, &in, &o);
  CHECK(o.phase == VZ_LAND && o.landReason == VZ_R_EST_RANGE, "hOff 0: LAND EST_RANGE");
  /* inputs sanitized */
  vzInit(&st, &c);
  in = (vzInput_t){.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.9f, .vRef = NAN,
                   .hOff = INFINITY};
  vzStep(&st, &in, &o);
  CHECK(o.phase == VZ_SPIN && st.hOff == 0.0f, "a NaN vRef or infinite hOff is 0, never a CUT");
}

/* A low take-off moved up to 1.20 m (0.20 + 0.50 + 0.50 m, the motion
 * module's limits): the controller takes the whole offset (VZ_HOFF_MAX 1.00 =
 * AUTO_MV_HOFF_MAX; 0.60 in 591f176 sized this LAND for 0.80 m), so the LAND
 * is as slow as from a 1.20 m take-off, with or without the camera, and its
 * timeout keeps 4 s after its longer flat part. */
static void testHoffHigh(void)
{
  static const struct { float h, up, landAt, camLoss; const char *name; } cs[] = {
    {0.20f, 1.00f, 16.0f, -1.0f, "0.20 + 1.00 m, LAND (camera)"},
    /* lost 10 s into HOLD (12 s until 2026-10-15): HOLD_NO_CAM 7 s later, inside the 18 s without a drift allowance
     * (the FSM's SINK comes 2 s before it on the drone) */
    {0.20f, 1.00f, -1.0f, 10.0f, "0.20 + 1.00 m, camera lost (HOLD_NO_CAM)"},
    {0.40f, 0.80f, -1.0f, 10.0f, "0.40 + 0.80 m, camera lost (HOLD_NO_CAM)"},
  };
  const int n = 20;
  for (int k = 0; k < 3; k++) {
    int ok = 0;
    float imp = 0.0f, landS = 0.0f, landMax = 0.0f, margin = 9.0f;
    for (int i = 0; i < n; i++) {
      vmScen_t sc = {.hTarget = cs[k].h, .kErr = 0.03f * (float)((i % 5) - 2), .accAirG = 0.001f * (float)((i % 7) - 3),
                     .moveAtS = 2.0f, .moveUpM = cs[k].up, .moveV = 0.15f, .landAtS = cs[k].landAt,
                     .camLossAtS = cs[k].camLoss, .seed = 9100u + 50u * (uint32_t)k + (uint32_t)i};
      vmRun_t r;
      vmRun(&sc, &r);
      imp = fmaxf(imp, r.impactV);
      landS = fmaxf(landS, r.landS);
      landMax = fmaxf(landMax, r.landMaxS);
      margin = fminf(margin, r.landMaxS - r.landS);
      ok += r.held && r.endPhase == VZ_DONE && r.landSlow && !r.estRange && r.landS < r.landMaxS;
    }
    const float flat = (cs[k].h <= VZ_H_TIMING_REF ? 1.5f : 0.0f) + cs[k].up / 0.3f;
    printf("info: %s: LAND max %.2f s (timeout %.2f s, margin %.2f s), impact max %.2f m/s\n", cs[k].name, landS,
           landMax, margin, imp);
    /* camera lost: the 7 s open loop under capQ before this backstop (the FSM's SINK comes at 5 s) can only sink
     * (2026-10-15; 0.45 before, after 3 s of HOLD without the camera) */
    const float impLim = cs[k].camLoss >= 0.0f ? 0.50f : 0.45f;
    CHECK(ok == n && imp <= impLim && fabsf(landMax - (flat + VZ_LAND_MAX_S - VZ_LAND_FLAT_MAX_S)) < 0.02f &&
              margin >= 0.8f,
          "%s: slow LAND to the ground in every sortie (%d/%d), impact %.2f <= %.2f m/s, timeout %.2f s = the flat "
          "part + 4 s, >= 0.8 s to spare (%.2f)", cs[k].name, ok, n, imp, impLim, landMax, margin);
  }
}

/* A take-off above the knee moved down to it or below, LANDed within VZ_LAND_DRIFT_AFTER_S of HOLD (not blind):
 * startLand()'s plain branch keeps the take-off height's flat part (4.0 s from the 1.20 m cap) and, as landTimes(),
 * its timeout at least 4 s after it (2026-10-15 review: 6.0 s there, the take-off cap's 2.00 m then had a 6.67 s flat
 * part and its LANDs ended LAND_TIMEOUT up to 0.67 m in the air; from 1.20 m the 2 s after the flat part ended them
 * LAND_TIMEOUT on the floor, a CUT, with the estimate reading high: camera speed offsets -0.03..-0.04 m/s, beyond the
 * logged -0.016). Every LAND a touchdown (DONE), its timeout 8.0 s. */
static void testLandAfterMoveDown(void)
{
  static const float cs[3] = {0.0f, -0.03f, -0.04f}, downs[2] = {-0.60f, -0.70f};
  int n = 0, done = 0, timeoutOk = 0;
  float imp[3] = {0.0f, 0.0f, 0.0f}, landMax = 0.0f;
  for (int ic = 0; ic < 3; ic++) {
    for (int id = 0; id < 2; id++) {
      for (int il = 0; il < 3; il++) {
        for (uint32_t sd = 0; sd < 6; sd++) {
          vmScen_t sc = {.hTarget = VZ_H_TAKEOFF_MAX, .camC = cs[ic], .moveAtS = 0.5f, .moveUpM = downs[id],
                         .moveV = 0.15f, .landAtS = 15.5f + (float)il, .camLossAtS = -1.0f, .seed = 7100u + sd,
                         .maxS = 60.0f};
          vmRun_t r;
          vmRun(&sc, &r);
          n++;
          done += r.endPhase == VZ_DONE && r.landReason == VZ_R_CMD_LAND && r.landSlow;
          timeoutOk += fabsf(r.landMaxS - (4.0f + VZ_LAND_MAX_S - VZ_LAND_FLAT_MAX_S)) < 1e-4f;
          imp[ic] = fmaxf(imp[ic], r.impactV);
          landMax = fmaxf(landMax, r.landS);
        }
      }
    }
  }
  printf("info: 1.20 m moved down to 0.60 / 0.50 m, LAND 15.5-17.5 s into HOLD: touchdown max %.2f (c 0), %.2f (c "
         "-0.03), %.2f m/s (c -0.04), LAND max %.2f s\n", imp[0], imp[1], imp[2], landMax);
  CHECK(done == n && timeoutOk == n && imp[0] <= 0.35f && imp[1] <= 0.6f,
        "a take-off above the knee moved down to it, LAND: a touchdown in every sortie (%d/%d, no LAND_TIMEOUT), the "
        "timeout the flat part + 4 s (%d/%d), touchdown %.2f (c 0) / %.2f m/s (c -0.03)", done, n, timeoutOk, n,
        imp[0], imp[1]);
}

/* 1.20 m: the climb, the hold, the slow LAND with the camera and without it
 * (the camera lost in HOLD: HOLD_NO_CAM, the slow LAND above the knee), and
 * a LAND after a +0.30 m vertical move: longer LAND times, never cut in the
 * air, soft at contact. */
static void testHigh120(void)
{
  const int n = 30;
  int ok[4] = {0, 0, 0, 0};
  float imp[4] = {0, 0, 0, 0}, landS[4] = {0, 0, 0, 0}, peak[4] = {0, 0, 0, 0}, peakLo[4] = {9, 9, 9, 9};
  float landMax[4] = {0, 0, 0, 0};
  for (int k = 0; k < 4; k++) {
    for (int i = 0; i < n; i++) {
      vmScen_t sc = {.hTarget = k == 3 ? 0.90f : 1.20f, .kErr = 0.03f * (float)((i % 5) - 2),
                     .accAirG = 0.001f * (float)((i % 7) - 3), .camC = k == 1 ? 0.01f * (float)((i % 3) - 1) : 0.0f,
                     .moveAtS = 2.0f, .moveUpM = k == 3 ? 0.30f : 0.0f, .moveV = 0.10f,
                     .landAtS = k == 2 ? -1.0f : 12.0f, .camLossAtS = k == 2 ? 4.0f : -1.0f,
                     .seed = 4200u + 50u * (uint32_t)k + (uint32_t)i};
      vmRun_t r;
      vmRun(&sc, &r);
      imp[k] = fmaxf(imp[k], r.impactV);
      landS[k] = fmaxf(landS[k], r.landS);
      peak[k] = fmaxf(peak[k], r.hMaxTrue);
      peakLo[k] = fminf(peakLo[k], r.hMaxTrue);
      landMax[k] = fmaxf(landMax[k], r.landMaxS);
      const vzReason_t want = k == 2 ? VZ_R_HOLD_NO_CAM : VZ_R_CMD_LAND;
      ok[k] += r.held && r.endPhase == VZ_DONE && r.landReason == want && r.landSlow && !r.estRange &&
               r.landS < r.landMaxS;
    }
  }
  /* 60 s of HOLD at 1.00 m with a camera speed offset of +/-0.010 m/s (the
   * truth drifts +/-0.6 m from the estimate): the drift allowance keeps the
   * LAND slow down to the ground. */
  int okLong = 0;
  float impLong = 0.0f, landLong = 0.0f, maxLong = 0.0f, peakLong = 0.0f;
  for (int i = 0; i < n; i++) {
    vmScen_t sc = {.hTarget = 1.00f, .kErr = 0.02f * (float)((i % 5) - 2), .camC = i % 2 ? 0.010f : -0.010f,
                   .landAtS = 60.0f, .camLossAtS = -1.0f, .seed = 4400u + (uint32_t)i};
    vmRun_t r;
    vmRun(&sc, &r);
    impLong = fmaxf(impLong, r.impactV);
    landLong = fmaxf(landLong, r.landS);
    maxLong = fmaxf(maxLong, r.landMaxS);
    peakLong = fmaxf(peakLong, r.hMaxTrue);
    okLong += r.held && r.endPhase == VZ_DONE && r.landReason == VZ_R_CMD_LAND && r.landSlow && r.landS < r.landMaxS;
  }
  printf("info: 1.00 m, 60 s HOLD, camera offset +/-0.010 m/s: true peak %.2f m, LAND max %.2f s (timeout %.2f s), "
         "impact max %.2f m/s\n", peakLong, landLong, maxLong, impLong);
  CHECK(okLong == n && impLong <= 0.6f &&
            fabsf(maxLong - (VZ_LAND_MAX_S + (1.0f - VZ_H_KNEE) / 0.3f +
                             fminf(VZ_LAND_DRIFT_MAX_M, VZ_LAND_DRIFT_MPS * 60.0f) / 0.20f)) < 0.02f,
        "1.00 m, 60 s HOLD, drifting estimate: slow LAND to the ground before the timeout (%.2f s) in every sortie "
        "(%d/%d), impact %.2f <= 0.6", maxLong, okLong, n, impLong);
  static const char *const name[4] = {"1.20 m, camera, LAND", "1.20 m, camera offset +/-0.01 m/s, LAND",
                                      "1.20 m, camera lost in HOLD (HOLD_NO_CAM)", "0.90 m + 0.30 m move, LAND"};
  for (int k = 0; k < 4; k++) {
    printf("info: %s: true peak %.2f..%.2f m, LAND max %.2f s (timeout %.2f s), impact max %.2f m/s\n", name[k],
           peakLo[k], peak[k], landS[k], landMax[k], imp[k]);
  }
  CHECK(ok[0] == n && imp[0] <= 0.45f && peak[0] <= 1.45f && peakLo[0] >= 1.0f && fabsf(landMax[0] - 8.0f) < 1e-3f,
        "%s: HOLD, slow LAND, touchdown before the 8.0 s timeout in every sortie (%d/%d), impact <= 0.45 (%.2f)",
        name[0], ok[0], n, imp[0]);
  CHECK(ok[1] == n && imp[1] <= 0.65f, "%s: every sortie (%d/%d), impact %.2f", name[1], ok[1], n, imp[1]);
  CHECK(ok[2] == n && imp[2] <= 0.6f, "%s: the slow LAND above the knee, touchdown (%d/%d), impact %.2f <= 0.6",
        name[2], ok[2], n, imp[2]);
  CHECK(ok[3] == n && imp[3] <= 0.45f && fabsf(landMax[3] - 8.0f) < 1e-3f,
        "%s: the LAND times of 1.20 m (timeout %.2f), every sortie (%d/%d), impact %.2f", name[3], landMax[3], ok[3], n,
        imp[3]);
}


/* ---- LAND COLLISION: the descent after an in-air contact ------------------
 * (user decision 2026-10-13: "try a slow landing" instead of the cut;
 * docs/autonomy-a2.md section 12.8). Hover at 1.00 or 1.20 m on the warm
 * camera, then a contact: the accelerometer reads a pulse that puts an error
 * e on the controller's speed estimate (10 ms integrating to e; 2026-10-13
 * flight 1 read +0.42, then +0.85 m/s of climb that was not there) and the
 * observer's camVz carries e too, decaying with tauE (the observer
 * recovering), persistent, or the observer unhealthy from 50 ms after the
 * contact (the controller then keeps e from its own integral). A contact is
 * a +3 g, 3 ms spike on the accelerometer (|acc| ~4 g, flight 1's first);
 * the train adds flight 1's later ones at 0.25 / 0.37 / 0.66 / 0.83 s. The
 * plant itself is not pushed. LAND starts 3 ms after the contact (the FSM's
 * AUTO_COLLIDE_MS) and is blind for blindS (vzInput_t.blind): 0 is (a) the
 * normal LAND, the whole LAND (b) the ceiling alone, 0.5 / 1.0 s (c) the
 * normal LAND with the loop off for its first part. */
typedef struct {
  float hTarget, kErr, accAirG, hoverS;
  float e, tauE;            /* the estimate's error at the contact (m/s, + reads a climb); its decay (<= 0: none) */
  bool camLost;             /* the observer unhealthy from 50 ms after the contact on */
  bool train;               /* flight 1's five contacts, not one */
  float blindS;             /* LAND blind this long from its start */
  float damp, geGain;       /* plant (0: default) */
  bool inClimb;             /* the contact in CLIMB at true height climbAtH (no hoverS) */
  float climbAtH;
  int vmove;                /* a vertical motion hoverS after HOLD entry (ctMotion: 1 MOVE up, 2 MOVE down, 3 the
                               re-track bob), the contact contactS after its start (not at hoverS) */
  float moveM, contactS;
  uint32_t seed;
} ctScen_t;

/* The vertical motions of auto_motion.c as the controller sees them (vRef, hOff), at time tm from the start:
 * a MOVE up/down of moveM (trapezoid at AUTO_MV_A_UP 0.20 m/s^2 and AUTO_MV_V_UP_MAX 0.15 m/s, plus the
 * position correction AUTO_MV_UP_KP 1/s on the height estimate's progress, all within AUTO_MV_UP_VREF_MAX
 * 0.15 m/s; then the settle on the correction alone until within 0.04 m + 10 % of the move, at most 3 s; then
 * DONE: vRef 0, hOff the move), or the re-track bob (legs of +/-AUTO_MV_BOB_A_M 0.04 m at 0.20 m/s^2, 4
 * cycles, hOff unchanged). Returns vRef; *settled latches the settle's end. */
static float ctMotion(int kind, float moveM, float tm, float prog, float *hOff, bool *settled)
{
  const float a = 0.20f, vmax = 0.15f;
  float z = 0.0f, vz = 0.0f, zEnd = 0.0f, T = 0.0f;
  if (tm < 0.0f || kind == 0) {
    return 0.0f;
  }
  if (kind == 3) {
    const float A = 0.04f, leg = 2.0f * sqrtf(A / a);
    const int n = (int)(tm / leg);
    if (n >= 8) {
      return 0.0f;
    }
    const float tl = tm - (float)n * leg, th = 0.5f * leg;
    const float s = tl < th ? 0.5f * a * tl * tl : A - 0.5f * a * (leg - tl) * (leg - tl);
    const float v = tl < th ? a * tl : a * (leg - tl);
    z = n % 2 ? A - s : s;
    vz = n % 2 ? -v : v;
    return fmaxf(-vmax, fminf(vmax, vz + (z - prog)));
  }
  const float D = fabsf(moveM), sg = kind == 1 ? 1.0f : -1.0f;
  float ta = vmax / a, vp = vmax;
  if (a * ta * ta > D) {
    ta = sqrtf(D / a);
    vp = a * ta;
  }
  const float tc = (D - a * ta * ta) / vp;
  T = 2.0f * ta + tc;
  float s;
  if (tm < ta) {
    s = 0.5f * a * tm * tm;
    vz = a * tm;
  } else if (tm < ta + tc) {
    s = 0.5f * a * ta * ta + vp * (tm - ta);
    vz = vp;
  } else if (tm < T) {
    s = D - 0.5f * a * (T - tm) * (T - tm);
    vz = a * (T - tm);
  } else {
    s = D;
    vz = 0.0f;
  }
  z = sg * s;
  vz *= sg;
  zEnd = sg * D;
  if (tm < T) {
    *hOff = z;
    return fmaxf(-vmax, fminf(vmax, vz + (z - prog)));
  }
  *hOff = zEnd;
  *settled = *settled || fabsf(prog - zEnd) <= 0.04f + 0.10f * D || tm >= T + 3.0f;
  return *settled ? 0.0f : fmaxf(-vmax, fminf(vmax, zEnd - prog));
}

typedef struct {
  float impactV;            /* fastest touchdown, m/s */
  float climb;              /* highest true height after the contact minus the height at it */
  float landS, landMaxS, hAt;
  float floorS;             /* from the first true height below 0.10 m after the contact to DONE (props near the floor) */
  float groundS;            /* from the last floor contact (the touchdown that stayed) to DONE (-1: DONE off the floor) */
  bool airDone;             /* DONE (thrust 0) with the drone still above 3 cm */
  vzPhase_t end;
  vzReason_t endReason;
} ctRun_t;

static void ctRun(const ctScen_t *sc, ctRun_t *r)
{
  memset(r, 0, sizeof(*r));
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, sc->hTarget);
  vzState_t st;
  vzInit(&st, &c);
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.kTrue = c.kPrior * (1.0f + sc->kErr);
  pc.accAirG = c.bAir + sc->accAirG;
  if (sc->damp > 0.0f) {
    pc.damp = sc->damp;
  }
  if (sc->geGain > 0.0f) {
    pc.geGain = sc->geGain;
  }
  warmCamera(&pc);
  vzSimState_t sim;
  vzSimInit(&sim, &pc, sc->seed);
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  static const float trainS[5] = {0.0f, 0.25f, 0.37f, 0.66f, 0.83f};
  float applied = 0.0f, vbat = sim.vbat, holdT = -1.0f, landT = -1.0f, hMaxAfter = 0.0f, floorT = -1.0f;
  float hOff = 0.0f, h0 = 0.0f, groundT = -1.0f;
  bool wasGround = false;
  bool settled = false;
  int kc = -1;   /* the contact's step */
  for (int k = 0; k < 200000; k++) {
    const float t = (float)k * DT;
    if (k % 100 == 0) {
      vbat = sim.vbat;
    }
    if (o.phase == VZ_HOLD && holdT < 0.0f) {
      holdT = t;
    }
    /* the vertical motion (its progress on the controller's height estimate, as auto_motion.c) */
    const float tm = holdT >= 0.0f && sc->vmove ? t - holdT - sc->hoverS : -1.0f;
    if (tm < 0.0f) {
      h0 = o.h;
    }
    const float vRef = ctMotion(sc->vmove, sc->moveM, tm, o.h - h0, &hOff, &settled);
    if (kc < 0 && ((!sc->inClimb && !sc->vmove && holdT >= 0.0f && t - holdT >= sc->hoverS) ||
                   (sc->vmove && tm >= sc->contactS) ||
                   (sc->inClimb && o.phase == VZ_CLIMB && sim.h >= sc->climbAtH))) {
      kc = k;
      r->hAt = sim.h;
      hMaxAfter = sim.h;
    }
    float accAdd = 0.0f;
    const int ks = kc >= 0 ? k - kc : -1;   /* ms since the contact */
    if (ks >= 0) {
      if (ks < 10) {
        accAdd += sc->e / (VZ_G * 0.010f);
      }
      for (int i = 0; i < (sc->train ? 5 : 1); i++) {
        const int k0 = (int)(trainS[i] * 1000.0f + 0.5f);
        if (ks >= k0 && ks < k0 + 3) {
          accAdd += 3.0f;
        }
      }
      pc.camVzOffset = sc->tauE > 0.0f ? sc->e * expf(-(float)ks * 0.001f / sc->tauE) : sc->e;
      if (sc->camLost && ks >= 50) {
        pc.camLossS = 0.0f;   /* unhealthy from now on */
        pc.camLossLenS = 1000.0f;
      }
    }
    const bool land = ks >= AUTO_COLLIDE_MS_T;
    if (land && landT < 0.0f) {
      landT = t;
      if ((sc->vmove == 1 || sc->vmove == 2) && !settled) {
        hOff = o.h - h0;   /* autoMotionStop: a stopped move keeps its measured progress (commitUp) */
      }
    }
    vzInput_t in = {
      .dt = DT, .cmd = land ? VZ_CMD_LAND : VZ_CMD_TAKEOFF, .accZ = sim.accZ + accAdd, .gravZ = 1.0f, .tiltDeg = 0.5f,
      .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy,
      .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias,
      .vRef = land ? 0.0f : vRef, .hOff = hOff,   /* the FSM's LAND stops the motion on the contact's step */
      .blind = land && t - landT < sc->blindS,
      .landFastOk = true,   /* (a): a LAND for a reason on the allow list, the worst case of an unseen contact */
    };
    vzStep(&st, &in, &o);
    vzSimStep(&sim, &pc, o.thrust, DT);
    applied = o.thrust;
    if (kc >= 0) {
      hMaxAfter = fmaxf(hMaxAfter, sim.h);
      if (floorT < 0.0f && sim.h < 0.10f) {
        floorT = t;
      }
      if (sim.onGround && !wasGround) {
        groundT = t;   /* the latest arrival on the floor */
      }
      wasGround = sim.onGround;
    }
    if (o.phase == VZ_LAND) {
      r->landMaxS = o.landMaxS;
    }
    if (o.phase == VZ_DONE || o.phase == VZ_CUT) {
      r->landS = landT >= 0.0f ? t - landT : -1.0f;
      r->floorS = floorT >= 0.0f ? t - floorT : 0.0f;
      r->groundS = groundT >= 0.0f && sim.onGround ? t - groundT : -1.0f;
      r->airDone = sim.h > 0.03f;
      for (int j = 0; j < 3000 && !sim.onGround; j++) {
        vzSimStep(&sim, &pc, 0.0f, DT);   /* thrust 0: the fall, if any */
      }
      break;
    }
  }
  r->end = o.phase;
  r->endReason = o.reason;
  r->impactV = sim.impactV;
  r->climb = hMaxAfter - r->hAt;
}

typedef struct {
  float impact, climb, landS, eAt, tauAt, floorS;
  int n, airDone, notDone, camLostAt, trainAt;
  float all[256];           /* every touchdown speed (percentiles) */
} ctAgg_t;

static void ctAdd(ctAgg_t *a, const ctScen_t *sc, const ctRun_t *r)
{
  if (a->n < 256) {
    a->all[a->n] = r->impactV;
  }
  a->n++;
  if (r->impactV > a->impact) {
    a->impact = r->impactV;
    a->eAt = sc->e;
    a->tauAt = sc->camLost ? -2.0f : sc->tauE;
    a->trainAt = sc->train;
  }
  a->climb = fmaxf(a->climb, r->climb);
  a->landS = fmaxf(a->landS, r->landS);
  a->floorS = fmaxf(a->floorS, r->floorS);
  a->airDone += r->airDone;
  a->notDone += r->end != VZ_DONE || r->landS < 0.0f || r->landS >= r->landMaxS;
}

/* The sweep at one height for one policy: e -1..+1 m/s, the observer
 * recovering (tau 0.3 / 1.0 s), persistent, or lost; one contact or the
 * train; three drones (K -3/0/+3 %, in-flight offset -2..+2 mg), 5 or 20 s
 * of hover before the contact: 240 sorties. */
static ctAgg_t ctSweep(float h, float blindS)
{
  static const float es[5] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
  static const float taus[4] = {0.3f, 1.0f, -1.0f, -1.0f};
  ctAgg_t a;
  memset(&a, 0, sizeof(a));
  for (int ie = 0; ie < 5; ie++) {
    for (int obs = 0; obs < 4; obs++) {
      for (int tr = 0; tr < 2; tr++) {
        for (int d = 0; d < 6; d++) {
          ctScen_t sc = {.hTarget = h, .kErr = 0.03f * (float)((d % 3) - 1), .accAirG = 0.002f * (float)((d % 5) - 2),
                         .hoverS = d < 3 ? 5.0f : 20.0f, .e = es[ie], .tauE = taus[obs], .camLost = obs == 3,
                         .train = tr != 0, .blindS = blindS,
                         .seed = 6100u + (uint32_t)(d + 7 * (ie + 5 * (obs + 4 * tr)))};
          ctRun_t r;
          ctRun(&sc, &r);
          ctAdd(&a, &sc, &r);
        }
      }
    }
  }
  return a;
}

static void testContactLand(void)
{
  /* the KILL of 98cd4ae from the hover: thrust 0 at the contact; the free fall from the true height (the sim's
   * rotor damping, 0.07 g per m/s, slows its fall a little: the cut rotors spin down) */
  float killV[2] = {0.0f, 0.0f}, killSim[2] = {0.0f, 0.0f}, killH[2] = {0.0f, 0.0f};
  for (int ih = 0; ih < 2; ih++) {
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, ih ? 1.20f : 1.00f);
    vzSimConfig_t sc;
    vzSimDefaultConfig(&sc);
    warmCamera(&sc);
    vzState_t st;
    vzSimState_t sim;
    vzOutput_t o;
    float applied;
    vzInit(&st, &c);
    vzSimInit(&sim, &sc, 77u);
    applied = 0.0f;
    memset(&o, 0, sizeof(o));
    float holdT = -1.0f;
    for (int k = 0; k < 20000; k++) {
      vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .gyroMaxDps = 0.3f,
                      .vbat = sim.vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy,
                      .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias};
      vzStep(&st, &in, &o);
      if (o.phase == VZ_HOLD && holdT < 0.0f) {
        holdT = (float)k * DT;
      }
      const bool cut = holdT >= 0.0f && (float)k * DT - holdT >= 3.0f;
      if (cut && killH[ih] == 0.0f) {
        killH[ih] = sim.h;
      }
      vzSimStep(&sim, &sc, cut ? 0.0f : o.thrust, DT);
      applied = o.thrust;
      if (cut && sim.onGround) {
        break;
      }
    }
    killSim[ih] = sim.impactV;
    killV[ih] = sqrtf(2.0f * VZ_G * killH[ih]);
  }
  printf("info: KILLED (98cd4ae) from the hover at %.2f / %.2f m (true): the floor at %.2f / %.2f m/s free fall (sim "
         "plant with rotor damping %.2f / %.2f)\n", killH[0], killH[1], killV[0], killV[1], killSim[0], killSim[1]);

  static const struct { float blindS; const char *name; } pol[4] = {
    {0.0f, "(a) normal LAND"}, {1e9f, "(b) ceiling alone (blind)"}, {0.5f, "(c) loop off 0.5 s"},
    {1.0f, "(c) loop off 1.0 s"},
  };
  ctAgg_t agg[2][4];
  for (int ih = 0; ih < 2; ih++) {
    const float h = ih ? 1.20f : 1.00f;
    for (int p = 0; p < 4; p++) {
      agg[ih][p] = ctSweep(h, pol[p].blindS);
      ctAgg_t *a = &agg[ih][p];
      const int nn = a->n < 256 ? a->n : 256;
      const char *obs = a->tauAt == -2.0f ? "lost" : (a->tauAt < 0.0f ? "persistent" :
                                                       (a->tauAt < 0.5f ? "tau 0.3 s" : "tau 1.0 s"));
      printf("info: contact at %.2f m, %-26s %d sorties: touchdown p50 %.2f p95 %.2f max %.2f m/s (e %+.1f, observer "
             "%s, %s), true climb max %.3f m, LAND max %.2f s (below 0.10 m before DONE max %.1f s), DONE in the air "
             "%d, not DONE in time %d\n", h, pol[p].name, a->n, pct(a->all, nn, 0.50f), pct(a->all, nn, 0.95f),
             a->impact, a->eAt, obs, a->trainAt ? "train" : "one contact", a->climb, a->landS, a->floorS, a->airDone,
             a->notDone);
    }
    /* (a), the normal LAND after a contact the FSM did not see (an estimate corrupted but still "healthy"; the
     * real FSM lands such a contact blind, LAND COLLISION, never fast): the fast LAND's brake (1.03 x T_ref for
     * at most 0.5 s while the estimate reads above 0.40 m) lifts the drone up to ~0.1 m (0.008 m without it). */
    CHECK(agg[ih][0].climb <= 0.10f, "contact at %.2f m, (a) normal LAND: true climb after the contact <= 0.10 m "
          "(%.3f; the fast LAND's brake, docs/autonomy-a2b.md 17.4)", h, agg[ih][0].climb);
    const ctAgg_t *b = &agg[ih][1];
    CHECK(b->impact <= 1.1f && b->climb <= 0.02f && b->airDone == 0 && b->notDone == 0,
          "contact at %.2f m, blind LAND: touchdown <= 1.1 m/s (%.2f) across e -1..+1 m/s, no climb (%.3f m), never "
          "DONE in the air (%d), DONE before the timeout in every sortie (%d not)", h, b->impact, b->climb,
          b->airDone, b->notDone);
    CHECK(b->impact < 0.5f * killV[ih], "contact at %.2f m: the blind LAND (%.2f m/s) under half the KILL's %.2f m/s",
          h, b->impact, killV[ih]);
    CHECK(b->impact + 0.5f < agg[ih][0].impact && b->impact + 0.5f < agg[ih][2].impact &&
          b->impact + 0.5f < agg[ih][3].impact,
          "contact at %.2f m: the blind LAND's worst case (%.2f) beats (a) %.2f and (c) %.2f / %.2f m/s by > 0.5", h,
          b->impact, agg[ih][0].impact, agg[ih][2].impact, agg[ih][3].impact);
  }

  /* The blind LAND does not depend on e: per error, persistent, the train */
  for (int ih = 0; ih < 2; ih++) {
    float lo = 9.0f, hi = 0.0f;
    for (int ie = 0; ie < 5; ie++) {
      float w = 0.0f;
      for (int d = 0; d < 3; d++) {
        ctScen_t sc = {.hTarget = ih ? 1.20f : 1.00f, .kErr = 0.03f * (float)(d - 1), .hoverS = 5.0f,
                       .e = -1.0f + 0.5f * (float)ie, .tauE = -1.0f, .train = true, .blindS = 1e9f,
                       .seed = 6900u + (uint32_t)d};
        ctRun_t r;
        ctRun(&sc, &r);
        w = fmaxf(w, r.impactV);
      }
      lo = fminf(lo, w);
      hi = fmaxf(hi, w);
    }
    CHECK(hi - lo <= 0.10f, "%.2f m, blind: the touchdown speed the same for every e (%.2f..%.2f m/s)",
          ih ? 1.20f : 1.00f, lo, hi);
  }

  /* After a long hover (60 s: the LAND's flat part and timeout grow by the drift allowance, 5 s): the blind
   * LAND floats near the floor at landTop x T_hold until the flat part ends, then lands before the timeout. */
  for (int ih = 0; ih < 2; ih++) {
    float w = 0.0f, fl = 0.0f, margin = 99.0f;
    int ok = 0, n = 0;
    for (int ie = 0; ie < 3; ie++) {
      for (int d = 0; d < 2; d++) {
        ctScen_t sc = {.hTarget = ih ? 1.20f : 1.00f, .kErr = 0.03f * (float)(2 * d - 1), .hoverS = 60.0f,
                       .e = -1.0f + (float)ie, .tauE = -1.0f, .train = d != 0, .blindS = 1e9f,
                       .seed = 7500u + (uint32_t)(ie + 3 * d)};
        ctRun_t r;
        ctRun(&sc, &r);
        n++;
        w = fmaxf(w, r.impactV);
        fl = fmaxf(fl, r.floorS);
        margin = fminf(margin, r.landMaxS - r.landS);
        ok += r.end == VZ_DONE && !r.airDone && r.landS > 0.0f && r.landS < r.landMaxS && r.climb <= 0.02f;
      }
    }
    printf("info: blind LAND after 60 s of hover at %.2f m: touchdown max %.2f m/s, below 0.10 m before DONE max %.1f "
           "s, LAND timeout margin min %.2f s\n", ih ? 1.20f : 1.00f, w, fl, margin);
    CHECK(ok == n && w <= 1.1f && margin >= 0.5f, "%.2f m, 60 s hover, blind: DONE on the ground before the timeout "
          "in every sortie (%d/%d, margin %.2f s), touchdown %.2f m/s", ih ? 1.20f : 1.00f, ok, n, margin, w);
  }

  /* Sensitivity of the blind LAND to the plant's rotor damping and ground effect (no feedback: the descent is
   * the ceiling's thrust deficit against the damping), informative. */
  for (int ih = 0; ih < 2; ih++) {
    static const float damps[3] = {0.035f, 0.07f, 0.14f};
    static const float ges[3] = {0.15f, 0.33f, 0.50f};
    float w = 0.0f, cl = 0.0f;
    int air = 0, notDone = 0;
    char worst[64] = "";
    for (int id = 0; id < 3; id++) {
      for (int ig = 0; ig < 3; ig++) {
        for (int k = 0; k < 10; k++) {
          ctScen_t sc = {.hTarget = ih ? 1.20f : 1.00f, .kErr = 0.03f * (float)((k % 3) - 1),
                         .accAirG = 0.002f * (float)((k % 5) - 2), .hoverS = 5.0f, .e = -1.0f + 0.5f * (float)(k % 5),
                         .tauE = -1.0f, .train = k % 2, .blindS = 1e9f, .damp = damps[id], .geGain = ges[ig],
                         .seed = 7000u + (uint32_t)k};
          ctRun_t r;
          ctRun(&sc, &r);
          if (r.impactV > w) {
            w = r.impactV;
            snprintf(worst, sizeof(worst), "damping %.3f g/(m/s), ground effect %.2f", damps[id], ges[ig]);
          }
          cl = fmaxf(cl, r.climb);
          air += r.airDone;
          notDone += r.end != VZ_DONE;
        }
      }
    }
    printf("info: blind LAND from %.2f m, plant damping 0.035..0.14 g per m/s, ground effect 0.15..0.50: touchdown max "
           "%.2f m/s (%s), climb max %.3f m, DONE in the air %d, not DONE %d\n", ih ? 1.20f : 1.00f, w, worst, cl, air,
           notDone);
    CHECK(air == 0 && notDone == 0 && cl <= 0.02f && w < 0.5f * killV[ih],
          "%.2f m, blind, other plants: DONE on the ground in every sortie, no climb, under half the KILL's speed "
          "(%.2f m/s)", ih ? 1.20f : 1.00f, w);
  }

  /* A contact in CLIMB (true height 0.60 m on the way to 1.20 m, climbing at up to 0.4 m/s): LAND from CLIMB,
   * landStart x the CLIMB reference falling at once (no flat part: the CLIMB thrust is above the hover), so the
   * drone rises ~0.1-0.2 m on its own speed first; informative. */
  float climbW[2] = {0.0f, 0.0f};
  int climbAir[2] = {0, 0};
  for (int p = 0; p < 2; p++) {
    float w = 0.0f, cl = 0.0f;
    int air = 0, n = 0;
    for (int ie = 0; ie < 5; ie++) {
      for (int k = 0; k < 6; k++) {
        ctScen_t sc = {.hTarget = 1.20f, .kErr = 0.03f * (float)((k % 3) - 1), .accAirG = 0.002f * (float)((k % 5) - 2),
                       .e = -1.0f + 0.5f * (float)ie, .tauE = -1.0f, .train = k % 2, .blindS = p ? 1e9f : 0.0f,
                       .inClimb = true, .climbAtH = 0.60f, .seed = 7300u + (uint32_t)k};
        ctRun_t r;
        ctRun(&sc, &r);
        w = fmaxf(w, r.impactV);
        cl = fmaxf(cl, r.climb);
        air += r.airDone;
        n++;
      }
    }
    printf("info: contact in CLIMB at 0.60 m, %s, %d sorties: touchdown max %.2f m/s, climb after the contact max "
           "%.2f m, DONE in the air %d\n", p ? "blind" : "normal LAND", n, w, cl, air);
    climbW[p] = w;
    climbAir[p] = air;
  }
  CHECK(climbAir[1] == 0 && climbW[1] <= 1.5f && climbW[1] < climbW[0],
        "contact in CLIMB, blind: never DONE in the air (%d; normal LAND %d), touchdown %.2f <= 1.5 m/s (normal %.2f)",
        climbAir[1], climbAir[0], climbW[1], climbW[0]);

  /* Contacts during the hover's vertical motions (the verification of 1c2a43f): T_hold, a 0.25 s average of the
   * applied thrust, carries a motion's accelerations (a MOVE up's start and climb, a MOVE down's braking, the
   * bob's legs: up to ~4 % over the hover), and 1c2a43f's blind LAND held 0.98 x that for its 3.3-5 s flat part
   * with no feedback: here it climbed up to 1.02 m (MOVE up), 0.36 m (the end of a MOVE down), 1.78 m (the bob)
   * and touched down at up to 2.93 m/s. The blind reference is now T_hold from before the motion (vzState_t.tHoldQ:
   * none in the VZ_QUIET_S before): what still climbs is the drone's own upward speed at the contact. A contact
   * every 0.1 s through the motion (and its settle), plant damping 0.035 / 0.07 / 0.14 g per m/s, one contact
   * or the train. */
  static const struct {
    int kind;
    float h, moveM, t0, t1, climbMax, impMax;
    const char *name;
  } vm[3] = {
    {1, 1.00f, 0.50f, 0.0f, 6.0f, 0.15f, 1.4f, "MOVE up 0.50 m from 1.00 m"},
    {2, 1.20f, 0.50f, 2.5f, 6.5f, 0.03f, 1.1f, "the end of a MOVE down 0.50 m from 1.20 m"},
    {3, 1.00f, 0.00f, 0.0f, 7.2f, 0.10f, 1.2f, "the re-track bob at 1.00 m"},
  };
  for (int m = 0; m < 3; m++) {
    static const float damps[3] = {0.035f, 0.07f, 0.14f};
    float w = 0.0f, cl = 0.0f, clAt = 0.0f;
    int air = 0, notDone = 0, n = 0;
    for (int id = 0; id < 3; id++) {
      for (int it = 0; vm[m].t0 + 0.1f * (float)it <= vm[m].t1 + 1e-3f; it++) {
        ctScen_t sc = {.hTarget = vm[m].h, .kErr = 0.03f * (float)((it % 3) - 1), .hoverS = 5.0f, .tauE = -1.0f,
                       .train = it % 2, .blindS = 1e9f, .damp = damps[id], .vmove = vm[m].kind, .moveM = vm[m].moveM,
                       .contactS = vm[m].t0 + 0.1f * (float)it, .seed = 7700u + (uint32_t)(it + 100 * id)};
        ctRun_t r;
        ctRun(&sc, &r);
        n++;
        w = fmaxf(w, r.impactV);
        if (r.climb > cl) {
          cl = r.climb;
          clAt = sc.contactS;
        }
        air += r.airDone;
        notDone += r.end != VZ_DONE || r.landS < 0.0f || r.landS >= r.landMaxS;
      }
    }
    printf("info: blind LAND, contact during %s (%d sorties, damping 0.035..0.14): true climb max %.3f m (contact %.1f s "
           "into it), touchdown max %.2f m/s, DONE in the air %d, not DONE in time %d\n", vm[m].name, n, cl, clAt, w, air,
           notDone);
    CHECK(air == 0 && notDone == 0 && cl <= vm[m].climbMax && w <= vm[m].impMax,
          "contact during %s, blind: climb %.3f <= %.2f m, touchdown %.2f <= %.1f m/s, never DONE in the air (%d), DONE "
          "before the timeout (%d not)", vm[m].name, cl, vm[m].climbMax, w, vm[m].impMax, air, notDone);
  }

  /* A take-off above the knee moved down to 0.50 m, a contact in the still hover after it: the reference is at
   * or below the knee, so the timeout is VZ_LAND_MAX_S (6.0 s), and the blind ceiling touches down ~2.9 s after
   * its flat part (no loop, no stall rule). 1c2a43f kept the take-off height's flat part (3.3 / 4.0 s) and ended
   * these LANDs in LAND_TIMEOUT on the floor; the blind flat part is now at most the knee's (2.0 s). (1.20 m to
   * 0.50 m is two MOVEs in the firmware; the controller sees only vRef and hOff.) */
  for (int ih = 0; ih < 2; ih++) {
    static const float damps[3] = {0.035f, 0.07f, 0.14f};
    const float h = ih ? 1.20f : 1.00f, mm = h - 0.50f;
    const float T = 1.5f + (mm - 0.1125f) / 0.15f;   /* the profile (ctMotion) */
    float w = 0.0f, margin = 99.0f, landMax = 0.0f;
    int ok = 0, n = 0;
    for (int id = 0; id < 3; id++) {
      for (int k = 0; k < 8; k++) {
        ctScen_t sc = {.hTarget = h, .kErr = 0.03f * (float)((k % 3) - 1), .hoverS = 3.0f, .tauE = -1.0f,
                       .train = k % 2, .blindS = 1e9f, .damp = damps[id], .vmove = 2, .moveM = mm,
                       .contactS = T + 4.5f + 0.4f * (float)k, .seed = 7800u + (uint32_t)(k + 10 * id)};
        ctRun_t r;
        ctRun(&sc, &r);
        n++;
        w = fmaxf(w, r.impactV);
        landMax = fmaxf(landMax, r.landS);
        margin = fminf(margin, r.landMaxS - r.landS);
        ok += r.end == VZ_DONE && (r.endReason == VZ_R_TOUCHDOWN_FORCE || r.endReason == VZ_R_TOUCHDOWN_JOLT) &&
              !r.airDone && r.climb <= 0.02f;
      }
    }
    printf("info: blind LAND after %.2f m moved down to 0.50 m (%d sorties): LAND max %.2f s, timeout margin min %.2f s, "
           "touchdown max %.2f m/s\n", h, n, landMax, margin, w);
    CHECK(ok == n && margin >= 1.0f && w <= 0.8f, "%.2f m moved down to 0.50 m, contact, blind: a touchdown, never "
          "LAND_TIMEOUT (%d/%d), >= 1.0 s before the timeout (%.2f), touchdown %.2f <= 0.8 m/s", h, ok, n, margin, w);
  }
}

/* vzRampAtTop() (the end of the glue's take-off integrator hold for a lift-off
 * never latched; user decision 2026-10-13, evening, bounded after the
 * verification of e4f7686): false until the RAMP reaches its plateau, then
 * latched until vzInit(); a detected lift-off always comes before it on the
 * plant (K x 1.00-1.12, no offset); an undetected one (a residual in-flight
 * offset of -10..-30 mg) is airborne before the plateau for at most ~0.9 s,
 * against up to 3.75 s through the float and the abort LAND (the hold
 * bounded by !liftedOff alone). */
/* ---- the fast LAND (2026-10-14, docs/autonomy-a2b.md 17.4) ---------------------
 * A sortie on the sim with a warm camera (kPrior 160400, the drone's NVS vzK), TAKEOFF to hTarget, LAND
 * landAtS after HOLD entry; the estimate reads e high from the LAND on (st.h += e, as the designer's sweep:
 * e > 0, the drone is lower than it thinks); the camera lost from camLossS into the LAND. fast = false runs
 * the same sortie with landVfast = landVref (the LAND as before). */
typedef struct {
  float hTarget, damp, kErr, e, camLossS, vRest, vSag, landAtS, hOff;
  float kPrior;             /* > 0: the controller's prior (else 160400, the drone's NVS vzK) */
  bool fast, blind, cmdNone;
  bool slowReason;          /* the state machine's LAND reason is not on the fast LAND's allow list (VBAT_DIP...) */
  int estRangeAtS;          /* >= 0: the height estimate jumps 2 m this long after HOLD (EST_RANGE) */
  float holdMaxS;           /* > 0: cfg.holdMaxS (HOLD_TIMEOUT) */
  float landInClimbH;       /* > 0: LAND in CLIMB at this true height */
  float moveUpM;            /* != 0: a vertical move of this much from 1.0 s after HOLD at 0.15 m/s (vRef, hOff ramp;
                               the take-off at most VZ_H_TAKEOFF_MAX, 2.00 m only by moves up) */
  uint32_t seed;
} flScen_t;

typedef struct {
  bool held, landFast;
  vzReason_t landReason;
  vzPhase_t endPhase;
  float impact, landS, maxOverRef, brakeS, overLowH, overHighH, climb, tRef, vfAtLand;
  uint64_t hash;            /* every output from the start (vert_ctrl_identity.h) */
} flRun_t;

static void flRun(const flScen_t *sc, flRun_t *r)
{
  memset(r, 0, sizeof(*r));
  vzConfig_t c;
  vzDefaultConfig(&c);
  c.kPrior = sc->kPrior > 0.0f ? sc->kPrior : 160400.0f;
  vzConfigSetHeight(&c, sc->hTarget);
  if (!sc->fast) {
    c.landVfast = c.landVref;
  }
  if (sc->holdMaxS > 0.0f) {
    c.holdMaxS = sc->holdMaxS;
  }
  vzState_t st;
  vzInit(&st, &c);
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.kTrue = c.kPrior * (1.0f + sc->kErr);
  pc.damp = sc->damp > 0.0f ? sc->damp : pc.damp;
  pc.vRest = sc->vRest > 0.0f ? sc->vRest : 3.85f;
  pc.vSag = sc->vSag > 0.0f ? sc->vSag : 1.15e-5f;
  pc.accAirG = c.bAir;
  warmCamera(&pc);
  vzSimState_t sim;
  vzSimInit(&sim, &pc, sc->seed);
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  uint64_t h = 0xCBF29CE484222325ull;
  float applied = 0.0f, vbat = sim.vbat, holdT = -1.0f, landT = -1.0f, hAtLand = 0.0f;
  bool landCmd = false;
  for (int k = 0; k < 160000; k++) {
    const float t = (float)k * DT;
    if (k % 100 == 0) {
      vbat = sim.vbat;
    }
    const float th = holdT >= 0.0f ? t - holdT : -1.0f;
    if (!landCmd) {
      landCmd = sc->landInClimbH > 0.0f ? (o.phase == VZ_CLIMB && sim.h >= sc->landInClimbH)
                                         : (sc->landAtS >= 0.0f && th >= sc->landAtS);
      if (landCmd && sc->e != 0.0f) {
        st.h += sc->e;
      }
    }
    if (sc->estRangeAtS >= 0 && th >= (float)sc->estRangeAtS && th < (float)sc->estRangeAtS + 0.0015f) {
      st.h += 2.0f;
    }
    const bool lost = (sc->camLossS >= 0.0f && landT >= 0.0f && t - landT >= sc->camLossS) ||
                      (sc->camLossS < -1.5f && th >= -sc->camLossS);   /* -2: from 2 s after HOLD entry */
    float hOff = sc->hOff, vRef = 0.0f;
    if (sc->moveUpM != 0.0f && th >= 1.0f) {
      const float dur = fabsf(sc->moveUpM) / 0.15f;
      if (th < 1.0f + dur) {
        vRef = sc->moveUpM > 0.0f ? 0.15f : -0.15f;
        hOff += vRef * (th - 1.0f);
      } else {
        hOff += sc->moveUpM;
      }
    }
    vzInput_t in = {
      .dt = DT, .cmd = landCmd ? (sc->cmdNone ? VZ_CMD_NONE : VZ_CMD_LAND) : VZ_CMD_TAKEOFF, .accZ = sim.accZ,
      .gravZ = 1.0f, .tiltDeg = 0.5f, .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied,
      .camHealthy = sim.camHealthy && !lost, .camVzValid = sim.camVzValid && !lost, .camVz = sim.camVz,
      .camBias = sim.camBias, .vRef = landCmd ? 0.0f : vRef, .hOff = hOff, .blind = sc->blind && landCmd,
      .landFastOk = !sc->slowReason,   /* the state machine's allow list (auto_fsm.h autoFsmLandFastReason()) */
    };
    vzStep(&st, &in, &o);
    h = vzIdOut(h, &o);
    if (o.phase == VZ_HOLD && holdT < 0.0f) {
      holdT = t;
      r->held = true;
    }
    if (o.phase == VZ_LAND && o.liftedOff && landT < 0.0f) {
      landT = t;
      hAtLand = sim.h;
      r->landReason = o.landReason;
      r->landFast = st.landFast;
      r->tRef = o.tRef;
      r->vfAtLand = st.vf;
    }
    if (o.phase == VZ_LAND && landT >= 0.0f && o.tRef > 0.0f) {
      const float over = o.thrust / o.tRef;
      r->maxOverRef = fmaxf(r->maxOverRef, over);
      if (o.thrust > o.ceiling + TOL_T) {
        r->brakeS += DT;
        if (o.h <= c.landBrakeH) {
          r->overLowH += DT;    /* the brake below landBrakeH (estimated): must stay 0 */
        }
        if (o.h > c.landFastH + c.landFastBand + 0.002f) {
          r->overHighH += DT;   /* the brake above the slow-down band (the cruise): must stay 0 */
        }
      }
      r->climb = fmaxf(r->climb, sim.h - hAtLand);
    }
    vzSimStep(&sim, &pc, o.thrust, DT);
    applied = o.thrust;
    if (o.phase == VZ_DONE || o.phase == VZ_CUT) {
      r->landS = landT >= 0.0f ? t - landT : 0.0f;
      break;
    }
  }
  r->endPhase = o.phase;
  r->impact = sim.impactV;
  r->hash = h;
}

/* A scenario with the defaults: LAND 3 s after HOLD entry, the camera kept, no estimate jump. */
static flScen_t flDefault(float hTarget, uint32_t seed)
{
  flScen_t sc;
  memset(&sc, 0, sizeof(sc));
  sc.hTarget = hTarget;
  sc.landAtS = 3.0f;
  sc.camLossS = -1.0f;
  sc.estRangeAtS = -1;
  sc.seed = seed;
  return sc;
}


/* The 2.00 m cap (2026-10-15, docs/autonomy-a2b.md 23.9): the hover goes to 2.00 m by moves up from the 1.20 m
 * take-off cap (VZ_H_TAKEOFF_MAX: a LAND in the climb falls from landStart at once). The sorties of testHigh120 at
 * 1.50 and 2.00 m reached so (the camera's LAND, a camera offset, the camera lost in HOLD, a 0.60 m take-off moved up
 * to the height, 60 s of HOLD with a drifting estimate); the LAND timeout grows by the reference height above the knee
 * at 0.30 m/s (9.0 s from 1.50 m, 10.67 s from 2.00 m, as a take-off there would have had). Every LAND ends on the
 * ground, slow, before its timeout, never cut in the air. */
static void testHigh200(void)
{
  const int n = 20;
  static const float hs[2] = {1.50f, 2.00f};
  for (int ih = 0; ih < 2; ih++) {
    const float H = hs[ih], timeout = VZ_LAND_MAX_S + (H - VZ_H_KNEE) / 0.30f;
    int ok[5] = {0, 0, 0, 0, 0};
    float imp[5] = {0, 0, 0, 0, 0}, landS[5] = {0, 0, 0, 0, 0}, peak[5] = {0, 0, 0, 0, 0};
    float peakLo[5] = {9, 9, 9, 9, 9}, landMax[5] = {0, 0, 0, 0, 0};
    for (int k = 0; k < 5; k++) {
      /* the take-off cap (k 3: 0.60 m) moved up to H at 0.15 m/s from 2 s into HOLD */
      const float to = k == 3 ? 0.60f : VZ_H_TAKEOFF_MAX, up = H - to, moveEnd = 2.0f + up / 0.15f;
      for (int i = 0; i < n; i++) {
        vmScen_t sc = {.hTarget = to, .kErr = 0.03f * (float)((i % 5) - 2), .accAirG = 0.001f * (float)((i % 7) - 3),
                       .camC = k == 1 ? 0.01f * (float)((i % 3) - 1) : (k == 4 ? (i % 2 ? 0.010f : -0.010f) : 0.0f),
                       .moveAtS = 2.0f, .moveUpM = up, .moveV = 0.15f,
                       .landAtS = k == 2 ? -1.0f : (k == 4 ? 60.0f : moveEnd + 4.0f),
                       .camLossAtS = k == 2 ? moveEnd + 6.0f : -1.0f,   /* in the settled hover at H */
                       .seed = 5200u + 50u * (uint32_t)k + 400u * (uint32_t)ih + (uint32_t)i};
        vmRun_t r;
        vmRun(&sc, &r);
        imp[k] = fmaxf(imp[k], r.impactV);
        landS[k] = fmaxf(landS[k], r.landS);
        peak[k] = fmaxf(peak[k], r.hMaxTrue);
        peakLo[k] = fminf(peakLo[k], r.hMaxTrue);
        landMax[k] = fmaxf(landMax[k], r.landMaxS);
        const vzReason_t want = k == 2 ? VZ_R_HOLD_NO_CAM : VZ_R_CMD_LAND;
        ok[k] += r.held && r.endPhase == VZ_DONE && r.landReason == want && r.landSlow && !r.estRange &&
                 r.landS < r.landMaxS;
      }
    }
    static const char *const name[5] = {"1.20 m moved up, LAND", "1.20 m moved up, camera offset +/-0.01 m/s, LAND",
                                         "1.20 m moved up, camera lost in HOLD (HOLD_NO_CAM)",
                                         "0.60 m moved up, LAND",
                                         "1.20 m moved up, 60 s HOLD, camera offset +/-0.010 m/s, LAND"};
    for (int k = 0; k < 5; k++) {
      printf("info: %.2f m, %s: true peak %.2f..%.2f m, LAND max %.2f s (timeout %.2f s), impact max %.2f m/s\n", H,
             name[k], peakLo[k], peak[k], landS[k], landMax[k], imp[k]);
    }
    CHECK(ok[0] == n && imp[0] <= 0.45f && peak[0] <= H + 0.25f && peakLo[0] >= H - 0.20f &&
          fabsf(landMax[0] - timeout) < 1e-3f,
          "%.2f m, %s: HOLD, slow LAND, touchdown before the %.2f s timeout in every sortie (%d/%d), impact <= 0.45 "
          "(%.2f), true peak %.2f..%.2f", H, name[0], timeout, ok[0], n, imp[0], peakLo[0], peak[0]);
    CHECK(ok[1] == n && imp[1] <= 0.65f, "%.2f m, %s: every sortie (%d/%d), impact %.2f", H, name[1], ok[1], n, imp[1]);
    CHECK(ok[2] == n && imp[2] <= 0.6f, "%.2f m, %s: the slow LAND above the knee, touchdown (%d/%d), impact %.2f <= "
          "0.6", H, name[2], ok[2], n, imp[2]);
    CHECK(ok[3] == n && imp[3] <= 0.45f && fabsf(landMax[3] - timeout) < 1e-3f,
          "%.2f m, %s: the LAND times of %.2f m (timeout %.2f), every sortie (%d/%d), impact %.2f", H, name[3], H,
          landMax[3], ok[3], n, imp[3]);
    const float longMax = timeout + fminf(VZ_LAND_DRIFT_MAX_M, VZ_LAND_DRIFT_MPS * 60.0f) / 0.20f;
    CHECK(ok[4] == n && imp[4] <= 0.6f && fabsf(landMax[4] - longMax) < 0.02f,
          "%.2f m, %s: slow LAND to the ground before the timeout (%.2f s, want %.2f) in every sortie (%d/%d), impact "
          "%.2f <= 0.6", H, name[4], landMax[4], longMax, ok[4], n, imp[4]);
  }

  /* the blind LAND (every LAND after a contact) at 1.50 and 2.00 m (the 1.20 m take-off moved up), after 5 and 60 s
   * of hover there: the ceiling alone for the longer flat part, then down; e -1..+1 m/s, K -3..+3 %, one contact or
   * the train */
  for (int ih = 0; ih < 2; ih++) {
    const float H = hs[ih], up = H - VZ_H_TAKEOFF_MAX;
    /* ctMotion's MOVE: the trapezoid at 0.20 m/s^2 and 0.15 m/s, then at most 3 s of settle */
    const float tMove = 2.0f * 0.75f + (up - 0.20f * 0.75f * 0.75f) / 0.15f + 3.0f;
    float w = 0.0f, cl = 0.0f, margin = 99.0f, fl = 0.0f;
    int okB = 0, nB = 0, air = 0;
    for (int ie = 0; ie < 3; ie++) {
      for (int d = 0; d < 6; d++) {
        ctScen_t sc = {.hTarget = VZ_H_TAKEOFF_MAX, .kErr = 0.03f * (float)((d % 3) - 1),
                       .accAirG = 0.002f * (float)((d % 5) - 2), .hoverS = 1.0f, .e = -1.0f + (float)ie, .tauE = -1.0f,
                       .train = d % 2, .blindS = 1e9f, .vmove = 1, .moveM = up,
                       .contactS = tMove + (d < 3 ? 5.0f : 60.0f), .seed = 8100u + (uint32_t)(d + 7 * ie + 50 * ih)};
        ctRun_t r;
        ctRun(&sc, &r);
        nB++;
        w = fmaxf(w, r.impactV);
        cl = fmaxf(cl, r.climb);
        fl = fmaxf(fl, r.floorS);
        margin = fminf(margin, r.landMaxS - r.landS);
        air += r.airDone;
        okB += r.end == VZ_DONE && !r.airDone && r.landS > 0.0f && r.landS < r.landMaxS && r.climb <= 0.02f;
      }
    }
    printf("info: blind LAND from %.2f m (1.20 m moved up; 5 / 60 s of hover, e -1..+1 m/s): touchdown max %.2f m/s, "
           "climb max %.3f m, below 0.10 m before DONE max %.1f s, LAND timeout margin min %.2f s, DONE in the air "
           "%d\n", H, w, cl, fl, margin, air);
    CHECK(okB == nB && w <= 1.1f && margin >= 0.5f, "%.2f m, blind LAND: DONE on the ground before the timeout in "
          "every sortie (%d/%d, margin %.2f s), touchdown %.2f m/s <= 1.1, no climb (%.3f)", H, okB, nB, margin, w, cl);
  }

  /* the fast LAND (the Mac's LAND) at 1.50 and 2.00 m (the 1.20 m take-off moved up): plants damping 0.07 / 0.035 /
   * 0.02, e -0.2..+0.4 m */
  for (int ih = 0; ih < 2; ih++) {
    const float H = hs[ih];
    static const float damps[3] = {0.07f, 0.035f, 0.02f}, es[4] = {-0.2f, 0.0f, 0.2f, 0.4f};
    int okF = 0, nF = 0, fast = 0;
    float w = 0.0f, over = 0.0f, brake = 0.0f, cl = 0.0f, tl = 0.0f;
    for (int id = 0; id < 3; id++) {
      for (int ie = 0; ie < 4; ie++) {
        for (uint32_t sd = 0; sd < 4; sd++) {
          flScen_t sc = flDefault(VZ_H_TAKEOFF_MAX, 9100u + 64u * (uint32_t)(id + 3 * ie + 12 * ih) + sd);
          sc.moveUpM = H - VZ_H_TAKEOFF_MAX;
          sc.landAtS = 1.0f + sc.moveUpM / 0.15f + 3.0f;
          sc.damp = damps[id];
          sc.kErr = 0.03f * (float)((int)(sd % 3) - 1);
          sc.e = es[ie];
          sc.fast = true;
          flRun_t r;
          flRun(&sc, &r);
          nF++;
          fast += r.landFast;
          w = fmaxf(w, r.impact);
          over = fmaxf(over, r.maxOverRef);
          brake = fmaxf(brake, r.brakeS);
          tl = fmaxf(tl, r.landS);
          if (es[ie] == 0.0f) {
            cl = fmaxf(cl, r.climb);
          }
          okF += r.endPhase == VZ_DONE && r.impact <= 0.55f && r.maxOverRef <= 1.03f + 1e-4f &&
                 r.overLowH == 0.0f && r.overHighH == 0.0f && r.brakeS <= 0.5f + 1.5f * DT;
        }
      }
    }
    printf("info: fast LAND from %.2f m (1.20 m moved up; %d sorties, fast %d): touchdown max %.2f m/s, LAND max %.2f "
           "s, brake max %.2f s at <= %.3f x T_ref, climb (e 0) max %.3f m\n", H, nF, fast, w, tl, brake, over, cl);
    CHECK(okF == nF && fast == nF && cl <= 0.02f, "%.2f m, fast LAND: DONE, touchdown <= 0.55 m/s, the brake within "
          "its rules in every sortie (%d/%d, fast %d), touchdown max %.2f", H, okF, nF, fast, w);
  }

  /* A known limit, as flown (reported, bounded against a regression; docs/autonomy-a2b.md 23.9): the camera lost for good
   * during a vertical move or within ~3 s after its settle. The open loop flies at most capQ, the latest quiet window
   * (from before the move, or the first one after it, which carries its settle), so the drone may climb or sink
   * slowly while the accelerometer estimate drifts its own way; the controller's backstop LAND (HOLD_NO_CAM, 7 s
   * later; on the drone the state machine's SINK comes 2 s earlier) starts with an estimate off by up to ~0.7 m and
   * touches down harder. The motion module's MOVE (ctMotion: profile, correction, settle; a loss stops it with the
   * measured progress), the loss every 0.5 s from the move's start to 4 s after its end, 10 drones. */
  {
    static const float tos[2] = {0.60f, VZ_H_TAKEOFF_MAX}, ups[2] = {0.60f, 0.80f};
    float worst[2] = {0.0f, 0.0f};
    int hard[2] = {0, 0}, nl[2] = {0, 0}, done[2] = {0, 0};
    for (int m = 0; m < 2; m++) {
      const float dur = 2.0f * 0.75f + (ups[m] - 0.20f * 0.75f * 0.75f) / 0.15f;
      for (float loss = 0.5f; loss <= dur + 7.0f; loss += 0.5f) {
        for (int i = 0; i < 10; i++) {
          vzConfig_t c;
          vzDefaultConfig(&c);
          vzConfigSetHeight(&c, tos[m]);
          vzState_t st;
          vzInit(&st, &c);
          vzSimConfig_t pc;
          vzSimDefaultConfig(&pc);
          pc.kTrue = c.kPrior * (1.0f + 0.03f * (float)((i % 5) - 2));
          pc.accAirG = c.bAir + 0.001f * (float)((i % 7) - 3);
          warmCamera(&pc);
          vzSimState_t sim;
          vzSimInit(&sim, &pc, 5700u + (uint32_t)i);
          vzOutput_t o;
          memset(&o, 0, sizeof(o));
          float applied = 0.0f, vbat = sim.vbat, holdT = -1.0f, hOff = 0.0f, h0 = 0.0f;
          bool settled = false, lost = false;
          for (int k = 0; k < 90000; k++) {
            const float t = (float)k * DT;
            if (k % 100 == 0) {
              vbat = sim.vbat;
            }
            const float tm = holdT >= 0.0f ? t - holdT - 2.0f : -1.0f;   /* the move from 2 s into HOLD */
            if (tm < 0.0f) {
              h0 = o.h;
            }
            const float vRef = lost ? 0.0f : ctMotion(1, ups[m], tm, o.h - h0, &hOff, &settled);
            if (!lost && tm >= loss) {
              lost = true;
              pc.camLossS = 0.0f;
              pc.camLossLenS = 1000.0f;
              if (!settled) {
                hOff = o.h - h0;   /* the state machine stops the motion: the measured progress */
              }
            }
            vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = 0.5f,
                            .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy,
                            .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias,
                            .vRef = lost ? 0.0f : vRef, .hOff = hOff};
            vzStep(&st, &in, &o);
            if (o.phase == VZ_HOLD && holdT < 0.0f) {
              holdT = t;
            }
            vzSimStep(&sim, &pc, o.thrust, DT);
            applied = o.thrust;
            if (o.phase == VZ_DONE || o.phase == VZ_CUT) {
              break;
            }
          }
          nl[m]++;
          done[m] += o.phase == VZ_DONE && o.landReason == VZ_R_HOLD_NO_CAM;
          worst[m] = fmaxf(worst[m], sim.impactV);
          hard[m] += sim.impactV > 0.6f;
        }
      }
    }
    printf("info: the camera lost for good in or <= 4 s after a move up (HOLD_NO_CAM): 0.60 + 0.60 = 1.20 m (as flown) "
           "touchdown max %.2f m/s, > 0.6 in %d of %d; 1.20 + 0.80 = 2.00 m max %.2f, > 0.6 in %d of %d\n", worst[0],
           hard[0], nl[0], worst[1], hard[1], nl[1]);
    CHECK(done[0] == nl[0] && done[1] == nl[1] && worst[0] <= 1.1f && worst[1] <= 1.35f,
          "the camera lost for good in or after a move up: DONE HOLD_NO_CAM in every sortie (%d/%d, %d/%d), touchdown "
          "%.2f m/s at 1.20 m (bound 1.1), %.2f at 2.00 m (bound 1.35): a known limit, bounded", done[0], nl[0],
          done[1], nl[1], worst[0], worst[1]);
  }

  /* a take-off to 2.00 m is invalid (BAD_CONFIG: CUT before any thrust), whatever asks for it */
  {
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, 2.00f);
    vzState_t st;
    vzInit(&st, &c);
    CHECK(st.phase == VZ_CUT && st.reason == VZ_R_BAD_CONFIG, "a 2.00 m take-off: vzInit CUT BAD_CONFIG (%s %s)",
          vzPhaseName(st.phase), vzReasonName(st.reason));
  }
}


static void testFastLand(void)
{
  /* The gates: only HOLD + the camera + CMD_LAND for a reason on the state machine's allow list + hRef >= 1.00 +
   * T_ref <= 53400 + Vf >= 3.15; every other LAND bit-identical to the controller without it (landVfast =
   * landVref). */
  typedef struct { const char *what; flScen_t sc; bool fast; vzReason_t reason; } gate_t;
  gate_t gates[16];
  int ng = 0;
  gates[ng++] = (gate_t){"1.00 m CMD_LAND (hRef 1.00: fast)", flDefault(1.00f, 11), true, VZ_R_CMD_LAND};
  gates[ng++] = (gate_t){"1.20 m CMD_LAND (fast)", flDefault(1.20f, 12), true, VZ_R_CMD_LAND};
  gates[ng++] = (gate_t){"0.99 m CMD_LAND", flDefault(0.99f, 13), false, VZ_R_CMD_LAND};
  gates[ng++] = (gate_t){"0.70 m CMD_LAND", flDefault(0.70f, 14), false, VZ_R_CMD_LAND};
  gates[ng] = (gate_t){"1.00 m CMD_LAND for VBAT_DIP (a reason off the allow list, Vf >= 3.15 V)", flDefault(1.00f, 23),
                       false, VZ_R_CMD_LAND};
  gates[ng++].sc.slowReason = true;
  gates[ng] = (gate_t){"1.20 m CMD_LAND for LOOP_GAP / HB_LOST (off the allow list)", flDefault(1.20f, 24), false,
                       VZ_R_CMD_LAND};
  gates[ng++].sc.slowReason = true;
  gates[ng] = (gate_t){"1.00 m CMD_MISSING", flDefault(1.00f, 15), false, VZ_R_CMD_MISSING};
  gates[ng++].sc.cmdNone = true;
  gates[ng] = (gate_t){"1.00 m blind (COLLISION)", flDefault(1.00f, 16), false, VZ_R_CMD_LAND};
  gates[ng++].sc.blind = true;
  gates[ng] = (gate_t){"1.00 m HOLD_TIMEOUT", flDefault(1.00f, 17), false, VZ_R_HOLD_TIMEOUT};
  gates[ng].sc.landAtS = -1.0f;
  gates[ng++].sc.holdMaxS = 4.0f;
  gates[ng] = (gate_t){"1.00 m EST_RANGE", flDefault(1.00f, 18), false, VZ_R_EST_RANGE};
  gates[ng].sc.landAtS = -1.0f;
  gates[ng++].sc.estRangeAtS = 2;
  gates[ng] = (gate_t){"1.00 m, LAND in CLIMB at 0.80 m", flDefault(1.00f, 19), false, VZ_R_CMD_LAND};
  gates[ng++].sc.landInClimbH = 0.80f;
  gates[ng] = (gate_t){"1.00 m, the camera lost in HOLD (HOLD_NO_CAM)", flDefault(1.00f, 22), false,
                       VZ_R_HOLD_NO_CAM};
  gates[ng].sc.landAtS = -1.0f;
  gates[ng++].sc.camLossS = -2.0f;   /* flRun: lost from 2 s after HOLD entry */
  gates[ng] = (gate_t){"1.00 m, T_ref over 53400 (a heavy drone, vzK 168000, Vf ~3.2 V)", flDefault(1.00f, 20),
                       false, VZ_R_CMD_LAND};
  gates[ng].sc.kPrior = 168000.0f;
  gates[ng].sc.kErr = 0.02f;
  gates[ng++].sc.vRest = 3.82f;
  gates[ng] = (gate_t){"1.00 m, Vf under 3.15 V (the end of the battery)", flDefault(1.00f, 21), false,
                       VZ_R_CMD_LAND};
  gates[ng].sc.kErr = -0.07f;
  gates[ng++].sc.vRest = 3.62f;
  for (int i = 0; i < ng; i++) {
    flScen_t a = gates[i].sc, b = gates[i].sc;
    a.fast = true;
    b.fast = false;
    flRun_t ra, rb;
    flRun(&a, &ra);
    flRun(&b, &rb);
    CHECK(ra.landFast == gates[i].fast && ra.landReason == gates[i].reason && ra.endPhase == VZ_DONE,
          "%s: fast %d (want %d), LAND %s (want %s), ends %s (T_ref %.0f, Vf %.2f V)", gates[i].what, ra.landFast,
          gates[i].fast, vzReasonName(ra.landReason), vzReasonName(gates[i].reason), vzPhaseName(ra.endPhase),
          (double)ra.tRef, (double)ra.vfAtLand);
    if (gates[i].fast) {
      CHECK(ra.hash != rb.hash && ra.landS < rb.landS, "%s: faster than the LAND before (%.2f < %.2f s)",
            gates[i].what, (double)ra.landS, (double)rb.landS);
    } else {
      CHECK(ra.hash == rb.hash, "%s: bit-identical to the LAND before", gates[i].what);
    }
    if (i == ng - 2) {
      CHECK(ra.tRef > VZ_LAND_FAST_TREF_MAX && ra.vfAtLand >= VZ_LAND_FAST_VMIN,
            "the heavy drone: T_ref %.0f over 53400 at Vf %.2f V", (double)ra.tRef, (double)ra.vfAtLand);
    } else if (i == ng - 1) {
      CHECK(ra.tRef <= VZ_LAND_FAST_TREF_MAX && ra.vfAtLand < VZ_LAND_FAST_VMIN,
            "the weak cell: T_ref %.0f, Vf %.2f V under 3.15", (double)ra.tRef, (double)ra.vfAtLand);
    }
  }

  /* Monte Carlo against the LAND before on the same seeds (verifier 2026-10-14: per cell, means and shares, every
   * plant, the estimate reading high): 0.8 / 1.0 / 1.2 m x damping 0.07 (the sim's default) / 0.035 / 0.02 (the
   * real accelerating LANDs) x e -0.2 / 0 / +0.2 / +0.4 m (the estimate reading high from the LAND on; 2 of 11
   * logged contacts were >= +0.27 m) x K -3 / 0 / +3 % x FL_MC_SEEDS seeds. 0.8 m is under landFastMinH: identical.
   * The sim's touchdown is bimodal on the low- and mid-drag plants (the ground effect catches the drone, which
   * then floats ~4 cm and drops at ~0.4 m/s at landFloor, or it touches at the approach speed), so one seed
   * scatters 0.0-0.5 m/s and a cell's mean moves by ~0.01 m/s between seed sets; the before-vs-before control on
   * other seeds (batch/fixer landx null=1) moved the cell means by up to +0.018 m/s (never 2 SE) and the shares
   * over 0.40 m/s by up to 14 of 180. Per cell: the mean touchdown <= the LAND before's + 0.015 m/s, the share
   * over 0.40 m/s not higher by more than 2 SE (two proportions) + 1, the hardest <= the LAND before's + 0.03 m/s;
   * LAND-to-DONE faster on average. */
#define FL_MC_SEEDS 20
  const float hs[3] = {0.80f, 1.00f, 1.20f}, damps[3] = {0.07f, 0.035f, 0.02f}, kErrs[3] = {-0.03f, 0.0f, 0.03f};
  const float es[4] = {-0.2f, 0.0f, 0.2f, 0.4f};
  int n = 0, nCell = 0, okMean = 0, okShare = 0, okMax = 0, okCellT = 0, okAbs = 0, okTop = 0, okLow = 0, okHigh = 0;
  int okBrake = 0, okClimb = 0, nClimb = 0, nFast = 0, nFastWant = 0, nSame = 0, nSameWant = 0;
  float worstMean = -9.0f, worstMax = -9.0f, worstShare = -9.0f, worstCellT = -9.0f, maxTd = 0.0f, maxOver = 0.0f;
  float maxBrake = 0.0f, maxClimb = 0.0f, bestMean = 9.0f;
  for (int ie = 0; ie < 4; ie++) {
    for (int id = 0; id < 3; id++) {
      printf("info: fast LAND, e %+.1f m, damping %.3f, from 0.8 / 1.0 / 1.2 m: touchdown mean fast/before (m/s), "
             "over 0.40, LAND-to-DONE mean (s):", (double)es[ie], (double)damps[id]);
      for (int ih = 0; ih < 3; ih++) {
        float mf = 0.0f, mb = 0.0f, xf = 0.0f, xb = 0.0f, fT = 0.0f, bT = 0.0f, sdd = 0.0f;
        int cell = 0, f40 = 0, b40 = 0;
        for (int ik = 0; ik < 3; ik++) {
          for (uint32_t sd = 0; sd < FL_MC_SEEDS; sd++) {
            flScen_t sc = flDefault(hs[ih], 900u + 37u * (uint32_t)(ih * 41 + id * 13 + ik * 5 + ie * 101) * 64u + sd);
            sc.damp = damps[id];
            sc.kErr = kErrs[ik];
            sc.e = es[ie];
            flRun_t rf, rb;
            sc.fast = true;
            flRun(&sc, &rf);
            sc.fast = false;
            flRun(&sc, &rb);
            n++;
            cell++;
            const bool want = hs[ih] >= 1.0f;
            nFastWant += want;
            nFast += want && rf.landFast;
            nSameWant += !want;
            nSame += !want && !rf.landFast && rf.hash == rb.hash;
            mf += rf.impact;
            mb += rb.impact;
            sdd += (rf.impact - rb.impact) * (rf.impact - rb.impact);
            xf = fmaxf(xf, rf.impact);
            xb = fmaxf(xb, rb.impact);
            f40 += rf.impact > 0.40f;
            b40 += rb.impact > 0.40f;
            fT += rf.landS;
            bT += rb.landS;
            maxTd = fmaxf(maxTd, rf.impact);
            okAbs += rf.endPhase == VZ_DONE && rf.impact <= 0.55f;
            okTop += rf.maxOverRef <= 1.03f + 1e-4f;
            okLow += rf.overLowH == 0.0f;
            okHigh += rf.overHighH == 0.0f;
            okBrake += rf.brakeS <= 0.5f + 1.5f * DT;
            maxOver = fmaxf(maxOver, rf.maxOverRef);
            maxBrake = fmaxf(maxBrake, rf.brakeS);
            if (es[ie] == 0.0f) {
              nClimb++;
              okClimb += rf.climb <= 0.02f;
              maxClimb = fmaxf(maxClimb, rf.climb);
            }
          }
        }
        nCell++;
        const float d = (mf - mb) / (float)cell;
        const float pp = (float)(f40 + b40) / (2.0f * (float)cell);
        const float shareTol = 2.0f * sqrtf(2.0f * pp * (1.0f - pp) * (float)cell) + 1.0f;   /* in runs */
        okMean += d <= 0.015f;
        okShare += (float)(f40 - b40) <= shareTol;
        okMax += xf <= xb + 0.03f;
        okCellT += hs[ih] < 1.0f || fT < bT;
        worstMean = fmaxf(worstMean, d);
        bestMean = fminf(bestMean, d);
        worstShare = fmaxf(worstShare, (float)(f40 - b40) - shareTol);
        worstMax = fmaxf(worstMax, xf - xb);
        if (hs[ih] >= 1.0f) {
          worstCellT = fmaxf(worstCellT, (fT - bT) / (float)cell);
        }
        printf(" %.3f/%.3f (%+.3f +-%.3f), %d/%d, %.2f/%.2f%s", (double)(mf / (float)cell), (double)(mb / (float)cell),
               (double)d, (double)sqrtf(fmaxf(0.0f, sdd / (float)cell - d * d) / (float)(cell - 1)), f40, b40,
               (double)(fT / (float)cell), (double)(bT / (float)cell), ih < 2 ? ";" : "\n");
      }
    }
  }
  CHECK(nFast == nFastWant && nSame == nSameWant, "every Monte Carlo LAND from 1.0-1.2 m is fast (%d/%d), every one "
        "from 0.8 m bit-identical to the LAND before (%d/%d)", nFast, nFastWant, nSame, nSameWant);
  CHECK(okMean == nCell, "touchdown: the mean per (h, damping, e) cell <= the LAND before's + 0.015 m/s (%d/%d, worst "
        "%+.3f, best %+.3f)", okMean, nCell, (double)worstMean, (double)bestMean);
  CHECK(okShare == nCell, "touchdown: the share over 0.40 m/s per cell not higher than the LAND before's by more than "
        "2 SE + 1 run (%d/%d, worst %+.1f runs over that)", okShare, nCell, (double)worstShare);
  CHECK(okMax == nCell, "touchdown: the hardest per cell <= the LAND before's hardest + 0.03 m/s (%d/%d, worst %+.3f)",
        okMax, nCell, (double)worstMax);
  CHECK(okAbs == n, "touchdown <= 0.55 m/s and DONE in every run (%d/%d, max %.2f)", okAbs, n, (double)maxTd);
  CHECK(okCellT == nCell, "LAND-to-DONE: faster on average in every cell from 1.0-1.2 m (%d/%d, the least gain "
        "%+.2f s)", okCellT, nCell, (double)worstCellT);
  CHECK(okTop == n && okLow == n && okHigh == n && okBrake == n, "thrust <= 1.03 x T_ref (%d/%d, max %.4f), above the "
        "0.98 ceiling only while the estimate reads 0.40-0.85 m (below %d/%d, above %d/%d) and for <= 0.5 s in all "
        "(%d/%d, max %.3f s)", okTop, n, (double)maxOver, okLow, n, okHigh, n, okBrake, n, (double)maxBrake);
  CHECK(okClimb == nClimb, "no climb after the LAND start (e = 0): <= 0.02 m (%d/%d, max %.3f m)", okClimb, nClimb,
        (double)maxClimb);
#undef FL_MC_SEEDS

  /* the camera lost 0.3..1.5 s into the LAND: the fast vref reverts at once, the brake keeps its 0.5 s (in the
   * slow-down band, on the propagated estimate). Per (h, damping): 4 loss times x K -3 / 0 / +3 % x 5 seeds. The
   * mid-drag plant's bimodal touchdown (above) makes these cells ~+0.03 m/s harder on average; bounded here. */
  const float losses[4] = {0.3f, 0.6f, 1.0f, 1.5f};
  int nl = 0, okl = 0, nlc = 0, oklc = 0;
  float worstL = -9.0f, maxL = 0.0f, worstLS = -9.0f;
  for (int ih = 1; ih < 3; ih++) {
    for (int id = 0; id < 3; id++) {
      float mf = 0.0f, mb = 0.0f;
      int f40 = 0, b40 = 0, cell = 0;
      for (int il = 0; il < 4; il++) {
        for (uint32_t sd = 0; sd < 15; sd++) {
          flScen_t sc = flDefault(hs[ih], 1300u + 977u * (uint32_t)(ih * 12 + id * 4 + il) + sd);
          sc.damp = damps[id];
          sc.kErr = kErrs[sd % 3];
          sc.camLossS = losses[il];
          flRun_t rf, rb;
          sc.fast = true;
          flRun(&sc, &rf);
          sc.fast = false;
          flRun(&sc, &rb);
          nl++;
          cell++;
          okl += rf.endPhase == VZ_DONE && rf.impact <= 0.55f;
          mf += rf.impact;
          mb += rb.impact;
          f40 += rf.impact > 0.40f;
          b40 += rb.impact > 0.40f;
          maxL = fmaxf(maxL, rf.impact);
        }
      }
      const float pp = (float)(f40 + b40) / (2.0f * (float)cell);
      const float shareTol = 2.0f * sqrtf(2.0f * pp * (1.0f - pp) * (float)cell) + 1.0f;
      nlc++;
      oklc += (mf - mb) / (float)cell <= 0.04f && (float)(f40 - b40) <= shareTol;
      worstL = fmaxf(worstL, (mf - mb) / (float)cell);
      worstLS = fmaxf(worstLS, (float)(f40 - b40) - shareTol);
    }
  }
  CHECK(okl == nl && oklc == nlc, "the camera lost 0.3-1.5 s into a fast LAND: DONE and touchdown <= 0.55 m/s (%d/%d, "
        "max %.2f), the mean per (h, damping) <= the LAND before's + 0.04 m/s and its share over 0.40 not higher by "
        "more than 2 SE + 1 (%d/%d, worst mean %+.3f, share %+.1f runs over)", okl, nl, (double)maxL, oklc, nlc,
        (double)worstL, (double)worstLS);
  printf("info: fast LAND, the camera lost 0.3-1.5 s into it (%d runs): touchdown max %.2f m/s, the worst mean per "
         "(h, damping) %+.3f m/s against the LAND before\n", nl, (double)maxL, (double)worstL);
}

static void testRampAtTop(void)
{
  CHECK(!vzRampAtTop(NULL), "NULL: false");
  const float offs[] = {0.0f, 0.0f, 0.0f, -0.010f, -0.015f, -0.020f, -0.030f};
  const float kf[] = {1.00f, 1.06f, 1.12f, 1.00f, 1.00f, 1.00f, 1.00f};
  for (int k = 0; k < 7; k++) {
    int topBeforeLift = 0, never = 0, topAfterInit = 0, topBeforeRamp = 0;
    float airHeldMax = 0.0f, airAllMax = 0.0f, hHeldMax = 0.0f, topT = -1.0f;
    for (uint32_t seed = 1; seed <= 40; seed++) {
      vzConfig_t c;
      vzDefaultConfig(&c);
      vzSimConfig_t sc;
      vzSimDefaultConfig(&sc);
      sc.kTrue = c.kPrior * kf[k];
      sc.accAirG = c.bAir + offs[k];
      vzState_t st;
      vzSimState_t sim;
      vzOutput_t out;
      memset(&out, 0, sizeof(out));
      vzInit(&st, &c);
      topAfterInit += vzRampAtTop(&st);
      vzSimInit(&sim, &sc, seed * 7919u);
      float applied = 0.0f, airHeld = 0.0f, airAll = 0.0f;
      bool land = false;
      for (int i = 0; i < 20000; i++) {
        vzInput_t in = {.dt = 0.001f, .cmd = land ? VZ_CMD_LAND : VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f,
                        .vbat = sim.vbat, .thrustApplied = applied, .gyroMaxDps = 0.5f};
        vzStep(&st, &in, &out);
        const bool top = vzRampAtTop(&st);
        topBeforeRamp += top && (out.phase == VZ_IDLE || out.phase == VZ_SPIN);
        if (top && topT < 0.0f && out.phase == VZ_RAMP) {
          topT = out.tPhase;
        }
        topBeforeLift += top && out.liftoffNow;
        const bool held = !out.liftedOff && out.phase <= VZ_RAMP && !top;   /* the glue's rule */
        vzSimStep(&sim, &sc, out.thrust, 0.001f);
        applied = out.thrust;
        land = land || out.phase == VZ_HOLD || out.phase == VZ_LAND;
        if (!sim.onGround && !out.liftedOff) {
          airAll += 0.001f;
          if (held) {
            airHeld += 0.001f;
            hHeldMax = fmaxf(hHeldMax, sim.h);
          }
        }
        if (out.phase == VZ_DONE || out.phase == VZ_CUT) {
          break;
        }
      }
      never += !out.liftedOff;
      airHeldMax = fmaxf(airHeldMax, airHeld);
      airAllMax = fmaxf(airAllMax, airAll);
    }
    CHECK(topAfterInit == 0 && topBeforeRamp == 0, "K x %.2f, %+.0f mg: vzRampAtTop false after vzInit and before RAMP",
          (double)kf[k], (double)(offs[k] * 1000.0f));
    CHECK(topBeforeLift == 0, "K x %.2f, %+.0f mg: no latched lift-off after the plateau (%d)", (double)kf[k],
          (double)(offs[k] * 1000.0f), topBeforeLift);
    CHECK(airHeldMax < 0.95f && hHeldMax < 0.07f, "K x %.2f, %+.0f mg: airborne without a latched lift-off before the "
          "plateau at most %.2f s (%.3f m)", (double)kf[k], (double)(offs[k] * 1000.0f), (double)airHeldMax,
          (double)hHeldMax);
    printf("info: ramp top: K x %.2f, %+.0f mg: never latched %d/40; airborne unlatched %.2f s in all, %.2f s before "
           "the plateau (h <= %.3f m); plateau at RAMP %.2f s\n", (double)kf[k], (double)(offs[k] * 1000.0f), never,
           (double)airAllMax, (double)airHeldMax, (double)hHeldMax, (double)topT);
  }
}

/* ---- hold and report (user decisions 2026-10-14 ~20:30-20:40, docs/autonomy-a2b.md 22) --------------------------
 * hrRun: a sortie on the sim with a warm camera to HOLD at hTarget, then a script: a hand holding the drone (its
 * height forced along a path, the accelerometer reading 1 g, the camera the hand's speed), the camera estimate lost
 * for a while, a contact's hush (vzInput_t.hush) in HOLD or in CLIMB at a true height, with accelerometer spikes, an
 * emulated FSM SINK (CMD_LAND: at once without capQ once the camera is out, sinkS after the camera was last in use
 * with it), a CMD_LAND at a time, vzInput_t.resume. Records every event (evtN changes), the LAND, a resume, the open
 * loop's thrust against capQ and the hush's freeze. */
typedef struct {
  float hTarget, kErr, camBiasErrG, accAirErrG, camC;
  float camLossAtS, camLossLenS;   /* the camera estimate unhealthy from this long after HOLD (< 0: never) for this */
  bool sink;                /* emulate the FSM's SINK (5 s with capQ, at once without, 0.5 s after a climb contact) */
  float landAtS;            /* CMD_LAND this long after HOLD (< 0: never) */
  float resumeAtS;          /* >= 0: vzInput_t.resume (and VZ_CMD_TAKEOFF) from this long after HOLD on */
  float hushAtS;            /* >= 0: a 0.5 s hush this long after HOLD */
  float hushClimbH;         /* > 0: the hush in CLIMB, at this true height */
  float camLossAtHushS;     /* > 0: the camera estimate unhealthy this long from the hush */
  float spikeG;             /* the hush's accelerometer spikes: this many g for 3 ms at its start and 0.25 s in */
  float grabAtS, pullV, pullS, heldV;   /* grabAtS >= 0: a hand from this long after HOLD: pullV for pullS, then heldV */
  float vGlitchAtS;         /* >= 0: a 1.5 m/s glitch in the estimate's speed (a 0.15 g, 1 s accelerometer step) */
  float maxS;               /* sim time (0: 40 s) */
  uint32_t seed;
} hrScen_t;

typedef struct {
  vzOutput_t last;
  float holdT, landT, hushT, lossT, resumeT, grabT;   /* sim times (-1: none) */
  vzReason_t landReason, holdReason;
  int nEvt;
  vzReason_t evt[16];
  float evtT[16];           /* s after HOLD */
  float openOverQ;          /* max (thrust - capQ) on open-loop HOLD steps with capQ set */
  int openSteps;
  bool hushFrozen;          /* h, v, vHp and Khat unchanged on every hush step but its first */
  float hushThr, hushHover; /* the hush's thrust, and the hover estimate's at its start */
  float vEdge, vEdgeWant;   /* v on the hush's falling-edge step, and what it should be re-seeded to */
  float vPreErr;            /* |the controller's vPre - v on the step before the contact| */
  float vBefore;
  float postCamMax;         /* the open window after the hush: |v change - the accelerometer's| (the camera unused) */
  float hAtEvt, hMaxAfter5, hMaxAfter, hAtLand, impactV;   /* the true height at the event (hush, loss, grab), the
                                                             most over it within 5 s and in all */
  int resumes, sinks;
  float hiMax;
} hrRun_t;

static void hrRun(const hrScen_t *sc, hrRun_t *r)
{
  memset(r, 0, sizeof(*r));
  r->holdT = r->landT = r->hushT = r->lossT = r->resumeT = r->grabT = -1.0f;
  r->openOverQ = -1e9f;
  r->hushFrozen = true;
  r->hAtEvt = -1.0f;
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, sc->hTarget);
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.kTrue = c.kPrior * (1.0f + sc->kErr);
  pc.accAirG = c.bAir + sc->accAirErrG;
  pc.camBiasErrG = sc->camBiasErrG;
  pc.camVzOffset = sc->camC;
  warmCamera(&pc);
  vzState_t st;
  vzInit(&st, &c);
  vzSimState_t sim;
  vzSimInit(&sim, &pc, sc->seed);
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  float applied = 0.0f, vbat = sim.vbat, prevH = 0.0f, prevV = 0.0f, prevVhp = 0.0f, prevK = 0.0f, camLast = -1.0f;
  float hHand = 0.0f, camBackT = -1.0f, contactT = -1.0f, openT = -1.0f;
  bool contactBlind = false;   /* the FSM's: from a contact until the camera estimate has been in use 0.5 s after it */
  uint8_t evtSeen = 0;
  bool hushPrev = false, landCmd = false;
  const int n = (int)((sc->maxS > 0.0f ? sc->maxS : 40.0f) * 1000.0f);
  for (int k = 0; k < n; k++) {
    const float t = (float)k * DT;
    if (k % 100 == 0) {
      vbat = sim.vbat;
    }
    const float th = r->holdT >= 0.0f ? t - r->holdT : -1.0f;
    /* the contact: its |acc| from contactT; the FSM's trip holds it AUTO_COLLIDE_MS, so the hush starts 3 ms later
     * (those 3 steps are integrated: vert_ctrl.c takes them out of vPre) */
    if (contactT < 0.0f && ((sc->hushClimbH > 0.0f && o.phase == VZ_CLIMB && sim.h >= sc->hushClimbH) ||
                            (sc->hushAtS >= 0.0f && th >= sc->hushAtS - 0.003f))) {
      contactT = t;
      r->hAtEvt = sim.h;
      r->vBefore = o.v;
    }
    if (contactT >= 0.0f && r->hushT < 0.0f && t >= contactT + 0.003f - 1e-6f) {
      r->hushT = t;
    }
    const bool hush = r->hushT >= 0.0f && t - r->hushT < 0.5f - 1e-6f;
    float spike = 0.0f;
    if (contactT >= 0.0f && sc->spikeG != 0.0f) {
      const float tc = t - contactT;
      spike = (tc < 0.004f - 1e-6f || (tc >= 0.253f && tc < 0.256f - 1e-6f)) ? sc->spikeG : 0.0f;
    }
    if (sc->vGlitchAtS >= 0.0f && th >= sc->vGlitchAtS && th < sc->vGlitchAtS + 1.0f) {
      spike += 0.15f;
    }
    bool lost = sc->camLossAtS >= 0.0f && th >= sc->camLossAtS &&
                (sc->camLossLenS < 0.0f || th < sc->camLossAtS + sc->camLossLenS);
    lost = lost || (sc->camLossAtHushS > 0.0f && r->hushT >= 0.0f && t - r->hushT < sc->camLossAtHushS);
    if (lost && r->lossT < 0.0f) {
      r->lossT = t;
      if (r->hAtEvt < 0.0f) {
        r->hAtEvt = sim.h;
      }
    }
    /* the FSM's SINK, emulated */
    if (sc->sink && !landCmd && o.phase == VZ_HOLD) {
      if (o.camHealthy) {
        camLast = t;
      }
      const float outFor = camLast >= 0.0f ? t - camLast : th;
      if (o.camOpen) {
        openT = t;
      }
      /* (camOpen: the controller's window after a contact, not a loss; the FSM also passes its last step) */
      if (!o.camHealthy && !o.camOpen && !(openT >= 0.0f && t - openT <= 1.5f * DT) &&
          ((o.capQ > 0.0f && outFor >= 5.0f) || (!(o.capQ > 0.0f) && th >= 2.0f) ||
           (!(o.capQ > 0.0f) && r->hushT >= 0.0f && t - r->hushT >= 0.5f))) {
        landCmd = true;
        r->sinks++;
      }
    }
    if (sc->landAtS >= 0.0f && th >= sc->landAtS && r->resumes == 0) {
      landCmd = true;
    }
    if (hush) {
      contactBlind = true;
      camBackT = -1.0f;
    } else if (contactBlind && o.camHealthy) {
      if (camBackT < 0.0f) {
        camBackT = t;
      } else if (t - camBackT >= 0.5f) {
        contactBlind = false;
      }
    } else {
      camBackT = -1.0f;
    }
    const bool resume = sc->resumeAtS >= 0.0f && th >= sc->resumeAtS;
    if (resume) {
      landCmd = false;
    }
    vzInput_t in = {
      .dt = DT, .cmd = landCmd ? VZ_CMD_LAND : VZ_CMD_TAKEOFF, .accZ = sim.accZ + spike, .gravZ = 1.0f,
      .tiltDeg = 0.5f, .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy && !lost,
      .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias, .hush = hush, .resume = resume,
      .blind = landCmd && contactBlind,
    };
    const vzPhase_t ph0 = o.phase;
    const float hoverAtHush = st.tQc > 0.0f ? st.tQc : 0.0f;
    vzStep(&st, &in, &o);
    if (o.phase == VZ_HOLD && r->holdT < 0.0f) {
      r->holdT = t;
      r->holdReason = o.reason;
    }
    if (o.evtN != evtSeen) {
      evtSeen = o.evtN;
      if (r->nEvt < 16) {
        r->evt[r->nEvt] = (vzReason_t)o.evt;
        r->evtT[r->nEvt] = r->holdT >= 0.0f ? t - r->holdT : -1.0f;
      }
      r->nEvt++;
    }
    if (ph0 == VZ_LAND && o.phase == VZ_HOLD) {
      r->resumes++;
      r->resumeT = t;
    }
    if (o.phase == VZ_LAND && r->landT < 0.0f) {
      r->landT = t;
      r->landReason = o.landReason;
      r->hAtLand = sim.h;
    }
    r->hiMax = fmaxf(r->hiMax, o.hiS);
    if (hush && !hushPrev) {
      r->hushThr = o.thrust;
      r->hushHover = hoverAtHush;
      r->vPreErr = fabsf(st.vPre - r->vBefore);
    }
    if (hush && hushPrev && o.phase == VZ_HOLD) {
      /* frozen; the thrust at most the first step's (a rise the camera sees lowers it, to 0.8 x at most) */
      r->hushFrozen = r->hushFrozen && o.h == prevH && o.v == prevV && o.vHp == prevVhp && o.khat == prevK &&
                      o.thrust <= fminf(st.hushT, o.ceiling) + TOL_T &&
                      o.thrust >= 0.80f * fminf(st.hushT, o.ceiling) - TOL_T;
    }
    if (!hush && hushPrev) {
      r->vEdge = o.v;
      /* the speed from before the contact, toward the camera's by at most VZ_HUSH_DV_MAX (this step then
       * integrates one sample) */
      r->vEdgeWant = in.camHealthy ? st.vPre + fmaxf(-VZ_HUSH_DV_MAX, fminf(VZ_HUSH_DV_MAX, in.camVz - st.vPre))
                                   : st.vPre;
    }
    if (!hush && r->hushT >= 0.0f && t - r->hushT >= 0.52f && t - r->hushT < 0.5f + VZ_HUSH_OPEN_S - 0.01f &&
        o.phase == VZ_HOLD) {
      /* the open window: v moves by the accelerometer alone (no camera blend) */
      const float dvAcc = VZ_G * (in.accZ - st.g0 - st.lastBias) * DT;
      r->postCamMax = fmaxf(r->postCamMax, fabsf((o.v - prevV) - dvAcc));
    }
    hushPrev = hush;
    prevH = o.h;
    prevV = o.v;
    prevVhp = o.vHp;
    prevK = o.khat;
    if (o.phase == VZ_HOLD && !o.camHealthy && o.capQ > 0.0f && st.tPhase >= st.loopEnd && !hush) {
      r->openOverQ = fmaxf(r->openOverQ, o.thrust - o.capQ);
      r->openSteps++;
    }
    vzSimStep(&sim, &pc, o.thrust, DT);
    applied = o.thrust;
    if (sc->grabAtS >= 0.0f && th >= sc->grabAtS && o.phase == VZ_HOLD) {
      if (r->grabT < 0.0f) {
        r->grabT = t;
        hHand = sim.h;
        r->hAtEvt = sim.h;
      }
      const float v = t - r->grabT < sc->pullS ? sc->pullV : sc->heldV;
      sim.camVz += v - sim.v;
      hHand += v * DT;
      sim.h = hHand;
      sim.v = v;
      sim.accZ = 1.0f;   /* held at a constant speed: 1 g */
    }
    const float tEvt = r->hushT >= 0.0f ? r->hushT : (r->lossT >= 0.0f ? r->lossT : r->grabT);
    if (tEvt >= 0.0f && r->hAtEvt >= 0.0f) {
      r->hMaxAfter = fmaxf(r->hMaxAfter, sim.h - r->hAtEvt);
      if (t - tEvt <= 5.0f) {
        r->hMaxAfter5 = fmaxf(r->hMaxAfter5, sim.h - r->hAtEvt);
      }
    }
    if (o.phase == VZ_DONE || o.phase == VZ_CUT) {
      break;
    }
  }
  r->last = o;
  r->impactV = sim.impactV;
}

static bool hrHasEvt(const hrRun_t *r, vzReason_t e, float *atS)
{
  for (int i = 0; i < r->nEvt && i < 16; i++) {
    if (r->evt[i] == e) {
      if (atS != NULL) {
        *atS = r->evtT[i];
      }
      return true;
    }
  }
  return false;
}

/* The contact's hush in HOLD (vzInput_t.hush): nothing integrates or adapts and the thrust is the hover estimate less
 * kfrzBias; the falling edge re-seeds v from the camera, or from the v before it without the camera; no climb. */
static void testHushHold(void)
{
  int ok = 0, n = 0;
  float climbMax = -9.0f, thrRelMax = 0.0f;
  for (int i = 0; i < 12; i++) {
    for (int cam = 0; cam < 2; cam++) {
      hrScen_t sc = {.hTarget = i % 2 ? 1.20f : 1.00f, .kErr = 0.02f * (float)((i % 5) - 2), .camLossAtS = -1.0f,
                     .landAtS = -1.0f, .resumeAtS = -1.0f, .hushAtS = 4.0f, .camLossAtHushS = cam ? 0.0f : 3.0f,
                     .spikeG = i % 3 == 0 ? 0.0f : (i % 3 == 1 ? 3.0f : -2.0f), .grabAtS = -1.0f, .vGlitchAtS = -1.0f,
                     .maxS = 14.0f, .seed = 7300u + (uint32_t)(2 * i + cam)};
      hrRun_t r;
      hrRun(&sc, &r);
      n++;
      /* (that step integrates one sample after the re-seed: within 0.005 m/s); vPre the v before the contact (its
       * 3 integrated steps of +3 / -2 g, 0.09 / 0.06 m/s, taken out); the window: the camera unused */
      const bool edge = fabsf(r.vEdge - r.vEdgeWant) <= 0.005f && r.vPreErr <= 0.006f && r.postCamMax <= 1e-4f;
      const bool hover = r.hushHover > 0.0f && r.hushThr <= r.hushHover * (1.0f - 0.005f) + TOL_T;
      climbMax = fmaxf(climbMax, r.hMaxAfter5);
      thrRelMax = fmaxf(thrRelMax, r.hushHover > 0.0f ? r.hushThr / r.hushHover : 9.0f);
      const bool good = r.holdT >= 0.0f && r.hushFrozen && edge && hover && r.landT < 0.0f &&
                        r.last.phase == VZ_HOLD && r.hMaxAfter5 <= 0.10f;
      ok += good;
      if (!good) {
        printf("info: hush in HOLD %d cam %d: frozen %d edge %.3f/%.3f vPre err %.4f window %.5f thr %.0f hover %.0f "
               "land %s climb %+.3f phase %s hold %.2f\n", i, cam, r.hushFrozen, r.vEdge, r.vEdgeWant, r.vPreErr,
               r.postCamMax, r.hushThr, r.hushHover,
               r.landT >= 0.0f ? vzReasonName(r.landReason) : "-", r.hMaxAfter5, vzPhaseName(r.last.phase), r.holdT);
      }
    }
  }
  printf("info: hush in HOLD (%d, spikes 0 / +3 / -2 g, camera kept or lost 3 s): hush thrust <= %.4f x the "
         "camera-measured hover, true height within 5 s at most %+.3f m over the contact's\n", n, thrRelMax, climbMax);
  CHECK(ok == n, "hush in HOLD: h, v, vHp, Khat frozen, the thrust the hover less kfrzBias (lower on a rise the "
        "camera sees), v re-seeded on the falling edge (the v before, toward camVz by <= 0.3 m/s), the camera unused "
        "for VZ_HUSH_OPEN_S, no LAND, no climb over 0.10 m in 5 s (%d/%d)", ok, n);
}

/* A contact in the climb with the camera estimate in use (the FSM lands blind without it): HOLD where it is
 * (CONTACT_HOLD, its event), the hush braking the climb; the camera kept: holding; the camera lost after it: the
 * FSM's SINK at once (no capQ yet). */
static void testHushClimb(void)
{
  static const float hs[3] = {0.30f, 0.60f, 0.90f};
  static const float loss[3] = {0.0f, 3.0f, 100.0f};
  int ok = 0, n = 0;
  float climbMax = 0.0f, climbKept = 0.0f;
  for (int il = 0; il < 3; il++) {
    for (int ih = 0; ih < 3; ih++) {
      for (int sd = 0; sd < 4; sd++) {
        hrScen_t sc = {.hTarget = 1.00f, .kErr = 0.02f * (float)(sd - 1), .accAirErrG = -0.005f * (float)(sd % 3),
                       .camLossAtS = -1.0f, .sink = true, .landAtS = -1.0f, .resumeAtS = -1.0f, .hushAtS = -1.0f,
                       .hushClimbH = hs[ih], .camLossAtHushS = loss[il], .spikeG = sd % 2 ? 3.0f : 0.0f,
                       .grabAtS = -1.0f, .vGlitchAtS = -1.0f, .maxS = 20.0f,
                       .seed = 7400u + 16u * (uint32_t)il + 4u * (uint32_t)ih + (uint32_t)sd};
        hrRun_t r;
        hrRun(&sc, &r);
        n++;
        float at;
        const bool held = r.holdReason == VZ_R_CONTACT_HOLD && hrHasEvt(&r, VZ_R_CONTACT_HOLD, &at);
        /* lost: the SINK's LAND is blind (the FSM's contactBlind) and from CLIMB (the HOLD's hover not measured):
         * today's COLLISION LAND in the climb (LAND from CLIMB, testClimbLandRef: impact max 1.39 m/s) */
        const bool end = il == 0 ? (r.landT < 0.0f && r.last.phase == VZ_HOLD)
                                 : (r.sinks == 1 && r.landT >= 0.0f && r.landT - r.hushT <= 0.55f &&
                                    r.last.phase == VZ_DONE && r.impactV <= 1.40f);
        climbMax = fmaxf(climbMax, r.hMaxAfter);
        if (il == 0) {
          climbKept = fmaxf(climbKept, r.hMaxAfter);
        }
        const bool good = held && end && r.hMaxAfter <= 0.25f;
        ok += good;
        if (!good) {
          printf("info: climb contact at %.1f m, loss %.0f s, seed %d: hold %s evt %d end %s land %s at %+.2f s after "
                 "the contact, impact %.2f, climb %+.3f\n", hs[ih], loss[il], sd, vzReasonName(r.holdReason), held,
                 vzPhaseName(r.last.phase), r.landT >= 0.0f ? vzReasonName(r.landReason) : "-", r.landT - r.hushT,
                 r.impactV, r.hMaxAfter);
        }
      }
    }
  }
  printf("info: contact in the climb at 0.3 / 0.6 / 0.9 m (36: camera kept, lost 3 s, lost for good; 0 / -5 / -10 mg "
         "in-flight offset error): the true height at most %+.3f m over the contact's (camera kept %+.3f: the climb's "
         "own speed braked by the hush, then the loop's transient)\n", climbMax, climbKept);
  CHECK(ok == n, "contact in the climb with the camera: HOLD CONTACT_HOLD and its event; kept: holding; lost: the "
        "blind SINK 0.5 s after it, touchdown <= 1.40 m/s (LAND from CLIMB); never over +0.25 m (%d/%d)", ok, n);
}

/* The climb faults with the camera estimate in use end the climb in HOLD with an event; without it they LAND as
 * before. K_RANGE: a drone 14 % heavier than its prior (the 2026-10-14 f6 case); CLIMB_TIMEOUT: a weak battery
 * (climbMaxS 1.0 s here). */
static void testClimbFaults(void)
{
  int ok = 0, n = 0;
  for (int cam = 0; cam < 2; cam++) {
    for (int f = 0; f < 2; f++) {
      for (int sd = 0; sd < 4; sd++) {
        vzConfig_t c;
        vzDefaultConfig(&c);
        vzConfigSetHeight(&c, 1.00f);
        if (f == 1) {
          c.climbMaxS = 1.0f;
        }
        vzSimConfig_t pc;
        vzSimDefaultConfig(&pc);
        pc.kTrue = c.kPrior * (f == 0 ? 1.14f : 1.0f);
        if (cam) {
          pc.camWakeS = 0.0f;
        }
        vzState_t st;
        vzInit(&st, &c);
        vzSimState_t sim;
        vzSimInit(&sim, &pc, 7500u + 8u * (uint32_t)cam + 4u * (uint32_t)f + (uint32_t)sd);
        vzOutput_t o;
        memset(&o, 0, sizeof(o));
        float applied = 0.0f, vbat = sim.vbat;
        vzReason_t holdWhy = VZ_R_NONE;
        bool evt = false;
        for (int k = 0; k < 8000; k++) {
          if (k % 100 == 0) {
            vbat = sim.vbat;
          }
          vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = 0.5f,
                          .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy,
                          .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias};
          const vzPhase_t ph0 = o.phase;
          vzStep(&st, &in, &o);
          if (ph0 == VZ_CLIMB && o.phase == VZ_HOLD) {
            holdWhy = o.reason;
            evt = o.evtN > 0 && o.evt == (uint8_t)o.reason;
          }
          vzSimStep(&sim, &pc, o.thrust, DT);
          applied = o.thrust;
          if (ph0 == VZ_CLIMB && o.phase != VZ_CLIMB) {
            break;
          }
        }
        const vzReason_t want = f == 0 ? VZ_R_K_RANGE : VZ_R_CLIMB_TIMEOUT;
        const bool good = cam ? (o.phase == VZ_HOLD && holdWhy == want && evt)
                              : (o.phase == VZ_LAND && o.landReason == want && o.evtN == 0);
        n++;
        ok += good;
        if (!good) {
          printf("info: climb fault %s camera %d seed %d: phase %s reason %s land %s evt %d\n", vzReasonName(want), cam,
                 sd, vzPhaseName(o.phase), vzReasonName(o.reason), vzReasonName(o.landReason), (int)o.evtN);
        }
      }
    }
  }
  CHECK(ok == n, "climb faults K_RANGE (K +14 %%) and CLIMB_TIMEOUT: with the camera estimate HOLD there with its "
        "event, without it LAND as before (%d/%d)", ok, n);
  /* EST_RANGE in CLIMB, its speed alone (1.5 m/s): with the camera HOLD, v re-seeded from camVz; without it, or the
   * height out of range, LAND as before */
  {
    vzConfig_t c;
    vzDefaultConfig(&c);
    bool ok3[3] = {false, false, false};
    for (int q = 0; q < 3; q++) {
      vzState_t st;
      vzInit(&st, &c);
      st.phase = VZ_CLIMB;
      st.liftedOff = st.started = true;
      st.vf = 3.7f;
      st.khat = c.kPrior;
      st.tLo = 35000.0f;
      st.tClimb = 44000.0f;
      st.h = q == 2 ? c.hMax + 0.05f : 0.20f;
      st.v = 1.5f;
      vzOutput_t o;
      vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.7f,
                      .thrustApplied = 44000.0f, .camHealthy = q != 1, .camVzValid = true, .camVz = 0.30f};
      vzStep(&st, &in, &o);
      ok3[q] = q == 0 ? (o.phase == VZ_HOLD && o.reason == VZ_R_EST_RANGE && fabsf(o.v - 0.30f) < 0.02f &&
                         o.evt == (uint8_t)VZ_R_EST_RANGE && o.thrust > 30000.0f)
                      : (o.phase == VZ_LAND && o.landReason == VZ_R_EST_RANGE);
    }
    CHECK(ok3[0] && ok3[1] && ok3[2], "EST_RANGE in CLIMB: a speed glitch with the camera -> HOLD, v re-seeded, an "
          "event (%d); without the camera LAND (%d); the height out of range LAND (%d)", ok3[0], ok3[1], ok3[2]);
  }
}

/* The upper thrust bound (VZ_HI_LAND_S, leaky): a drone pinned by a hand that keeps pulling it down (Khat at its
 * upper clamp) lands within ~5-7 s of the grab; a free drone 13.5 % heavier than its prior (Khat at the clamp,
 * sinking slowly on the P term) lands K_RANGE 5 s after its event. A drone held still after a pull is at no bound:
 * the loop has no speed error, Khat stays where the pull left it, and the drone pushes into the hand at that thrust
 * until Claude's LAND or the user's k (docs/autonomy-a2b.md 22: the runbook lands on a hand in the photo). */
static void testHiBound(void)
{
  int ok = 0, n = 0;
  float worst = 0.0f;
  for (int p = 0; p < 2; p++) {
    for (int sd = 0; sd < 4; sd++) {
      hrScen_t sc = {.hTarget = 1.00f, .camLossAtS = -1.0f, .landAtS = -1.0f, .resumeAtS = -1.0f, .hushAtS = -1.0f,
                     .grabAtS = 6.0f, .pullV = p ? -0.10f : -0.20f, .pullS = p ? 1.0f : 0.5f, .heldV = -0.05f,
                     .vGlitchAtS = -1.0f, .maxS = 40.0f, .seed = 7600u + 4u * (uint32_t)p + (uint32_t)sd};
      hrRun_t r;
      hrRun(&sc, &r);
      n++;
      const float after = r.landT - r.grabT;
      worst = fmaxf(worst, after);
      const bool good = r.landT >= 0.0f && after <= 8.0f &&
                        (r.landReason == VZ_R_K_RANGE || r.landReason == VZ_R_HOLD_CAPPED);
      ok += good;
      if (!good) {
        printf("info: pinned %d seed %d: LAND %s %.2f s after the grab, hiS max %.2f\n", p, sd,
               r.landT >= 0.0f ? vzReasonName(r.landReason) : "-", after, r.hiMax);
      }
    }
  }
  printf("info: pinned by a hand pulling down (8): LAND K_RANGE / HOLD_CAPPED at most %.2f s after the grab\n", worst);
  CHECK(ok == n, "pinned by a hand: the thrust-bound LAND within 8 s of the grab (%d/%d)", ok, n);
  int still = 0;
  float thrRel = 0.0f, hiMax = 0.0f;
  for (int sd = 0; sd < 4; sd++) {
    hrScen_t sc = {.hTarget = 1.00f, .camLossAtS = -1.0f, .landAtS = -1.0f, .resumeAtS = -1.0f, .hushAtS = -1.0f,
                   .grabAtS = 6.0f, .pullV = -0.20f, .pullS = 0.5f, .heldV = 0.0f, .vGlitchAtS = -1.0f, .maxS = 30.0f,
                   .seed = 7610u + (uint32_t)sd};
    hrRun_t r;
    hrRun(&sc, &r);
    still += r.landT < 0.0f;
    thrRel = fmaxf(thrRel, r.last.thrust / (r.last.capQ / 0.995f));
    hiMax = fmaxf(hiMax, r.hiMax);
  }
  printf("info: held still after a pull (4): %d without a LAND in 18 s, thrust up to %.3f x the measured hover, hiS "
         "up to %.2f s\n", still, thrRel, hiMax);
  int okH = 0;
  float landAt = 0.0f;
  for (int sd = 0; sd < 4; sd++) {
    hrScen_t sc = {.hTarget = 1.00f, .kErr = 0.135f, .camLossAtS = -1.0f, .landAtS = -1.0f, .resumeAtS = -1.0f,
                   .hushAtS = -1.0f, .grabAtS = -1.0f, .vGlitchAtS = -1.0f, .maxS = 40.0f, .seed = 7620u + (uint32_t)sd};
    hrRun_t r;
    hrRun(&sc, &r);
    float at;
    const bool evt = hrHasEvt(&r, VZ_R_K_RANGE, &at) || hrHasEvt(&r, VZ_R_HOLD_CAPPED, &at);
    landAt = fmaxf(landAt, r.landT - r.holdT);
    okH += r.landT >= 0.0f && r.landReason == VZ_R_K_RANGE && evt && r.landT - r.holdT >= 5.0f &&
           r.impactV <= 0.6f;
  }
  CHECK(okH == 4, "K +13.5 %% (Khat at its upper clamp): the K_RANGE event, then the LAND >= 5 s later (%.2f s after "
        "HOLD at most), touchdown <= 0.6 m/s (%d/4)", landAt, okH);
}

/* Khat's HOLD floor (VZ_KHAT_HOLD_LO_REL) with the camera estimate: a drone 14-23 % lighter than its prior holds
 * (the K_RANGE event under khatMinRel) instead of climbing on the P term alone (fixer 2026-10-15: +1.2 m at 0.86, +2.9
 * m at 0.83 in 20 s with the floor at khatMinRel); 26 % lighter is past the floor: LAND K_RANGE. */
static void testKFloor(void)
{
  static const float kr[4] = {-0.14f, -0.17f, -0.20f, -0.23f};
  int ok = 0, n = 0;
  float worst = 0.0f;
  for (int i = 0; i < 4; i++) {
    for (int sd = 0; sd < 3; sd++) {
      hrScen_t sc = {.hTarget = 1.00f, .kErr = kr[i], .camLossAtS = -1.0f, .landAtS = -1.0f, .resumeAtS = -1.0f,
                     .hushAtS = -1.0f, .grabAtS = -1.0f, .vGlitchAtS = -1.0f, .maxS = 35.0f,
                     .seed = 7700u + 4u * (uint32_t)i + (uint32_t)sd};
      vzConfig_t c;
      vzDefaultConfig(&c);
      hrRun_t r;
      hrRun(&sc, &r);
      n++;
      /* the true height over the last 30 s against HOLD's: hMaxAfter is not armed (no event); use the peak */
      const float rise = r.last.h;   /* (the estimate holds; the truth below) */
      (void)rise;
      float at;
      const bool evt = hrHasEvt(&r, VZ_R_K_RANGE, &at);
      const bool good = r.landT < 0.0f && r.last.phase == VZ_HOLD && evt && r.last.khat < c.khatMinRel * c.kPrior;
      ok += good;
      worst = fmaxf(worst, r.last.khat / c.kPrior);
      if (!good) {
        printf("info: K %+.2f seed %d: phase %s land %s evt %d khat %.3f\n", kr[i], sd, vzPhaseName(r.last.phase),
               r.landT >= 0.0f ? vzReasonName(r.landReason) : "-", evt, r.last.khat / c.kPrior);
      }
    }
  }
  CHECK(ok == n, "K -14..-23 %% with the camera: HOLD on Khat below khatMinRel, the K_RANGE event, no LAND in 35 s "
        "(%d/%d)", ok, n);
  /* the true height: no steady climb (the vzSim peak over the run against the HOLD height) */
  int okR = 0;
  float riseMax = 0.0f;
  for (int i = 0; i < 4; i++) {
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, 1.00f);
    vzSimConfig_t pc;
    vzSimDefaultConfig(&pc);
    pc.kTrue = c.kPrior * (1.0f + kr[i]);
    warmCamera(&pc);
    vzState_t st;
    vzInit(&st, &c);
    vzSimState_t sim;
    vzSimInit(&sim, &pc, 7720u + (uint32_t)i);
    vzOutput_t o;
    memset(&o, 0, sizeof(o));
    float applied = 0.0f, vbat = sim.vbat, h5 = -1.0f, hiAfter = 0.0f, holdT = -1.0f;
    for (int k = 0; k < 35000; k++) {
      const float t = (float)k * DT;
      if (k % 100 == 0) {
        vbat = sim.vbat;
      }
      vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = 0.5f,
                      .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy,
                      .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias};
      vzStep(&st, &in, &o);
      if (o.phase == VZ_HOLD && holdT < 0.0f) {
        holdT = t;
      }
      if (holdT >= 0.0f && t - holdT >= 5.0f) {
        if (h5 < 0.0f) {
          h5 = sim.h;
        }
        hiAfter = fmaxf(hiAfter, sim.h - h5);
      }
      vzSimStep(&sim, &pc, o.thrust, DT);
      applied = o.thrust;
    }
    riseMax = fmaxf(riseMax, hiAfter);
    okR += o.phase == VZ_HOLD && hiAfter <= 0.15f;
  }
  printf("info: K -14..-23 %%, 30 s of HOLD on the camera: the true height rose at most %.3f m\n", riseMax);
  CHECK(okR == 4, "K -14..-23 %%: no steady climb in 30 s of HOLD (rise <= 0.15 m: %.3f; %d/4)", riseMax, okR);
  int okL = 0;
  for (int sd = 0; sd < 3; sd++) {
    hrScen_t sc = {.hTarget = 1.00f, .kErr = -0.26f, .camLossAtS = -1.0f, .landAtS = -1.0f, .resumeAtS = -1.0f,
                   .hushAtS = -1.0f, .grabAtS = -1.0f, .vGlitchAtS = -1.0f, .maxS = 25.0f, .seed = 7740u + (uint32_t)sd};
    hrRun_t r;
    hrRun(&sc, &r);
    okL += r.landT >= 0.0f && r.landReason == VZ_R_K_RANGE && r.last.phase == VZ_DONE;
  }
  CHECK(okL == 3, "K -26 %%: past Khat's HOLD floor, LAND K_RANGE (%d/3)", okL);
}

/* The camera estimate lost in a settled hold: the open loop under capQ (never above it on any step), on the observer's
 * last offset, sinks or holds, never climbs over 0.05 m in the 5 s before the FSM's SINK (fixer 2026-10-15; the
 * verifier's seed 7 climbed 0.215 m on a single 0.25 s sample); back after 2 s: closed again, no LAND. */
static void testCamLostCap(void)
{
  static const float be[3] = {0.0f, 0.005f, -0.005f};
  int ok = 0, n = 0;
  float climb = -9.0f, over = -9e9f;
  for (int ib = 0; ib < 3; ib++) {
    for (int sd = 0; sd < 24; sd++) {
      hrScen_t sc = {.hTarget = sd % 2 ? 1.20f : 1.00f, .kErr = 0.01f * (float)((sd % 5) - 2), .camBiasErrG = be[ib],
                     .accAirErrG = 0.002f * (float)((sd % 3) - 1), .camLossAtS = 8.0f, .camLossLenS = 8.0f,
                     .sink = true, .landAtS = -1.0f, .resumeAtS = -1.0f, .hushAtS = -1.0f, .grabAtS = -1.0f,
                     .vGlitchAtS = -1.0f, .maxS = 30.0f, .seed = 7800u + 32u * (uint32_t)ib + (uint32_t)sd};
      hrRun_t r;
      hrRun(&sc, &r);
      n++;
      climb = fmaxf(climb, r.hMaxAfter5);
      over = fmaxf(over, r.openOverQ);
      const bool good = r.openSteps > 1000 && r.openOverQ <= TOL_T && r.hMaxAfter5 <= 0.05f && r.sinks == 1 &&
                        r.landT - r.lossT >= 4.99f && r.landT - r.lossT <= 5.01f && r.last.phase == VZ_DONE &&
                        r.impactV <= 0.65f;
      ok += good;
      if (!good) {
        printf("info: camera lost, berr %+.3f seed %d: open %d over %.1f climb %+.3f sinks %d land %.2f s after, end %s "
               "impact %.2f\n", be[ib], sd, r.openSteps, r.openOverQ, r.hMaxAfter5, r.sinks, r.landT - r.lossT,
               vzPhaseName(r.last.phase), r.impactV);
      }
    }
  }
  printf("info: camera height lost 8 s in a settled hold (72: offset error 0 / +-5 mg, K +-2 %%): open-loop thrust - "
         "capQ at most %+.1f, the true height within 5 s at most %+.3f m over the loss's, SINK at 5 s\n", over, climb);
  CHECK(ok == n, "camera lost in HOLD: the open loop under capQ, no climb over 0.05 m in 5 s, the SINK 5 s after, "
        "touchdown <= 0.65 m/s (%d/%d)", ok, n);
  /* back after 2 s: no LAND */
  int okB = 0;
  for (int sd = 0; sd < 6; sd++) {
    hrScen_t sc = {.hTarget = 1.00f, .camLossAtS = 8.0f, .camLossLenS = 2.0f, .sink = true, .landAtS = -1.0f,
                   .resumeAtS = -1.0f, .hushAtS = -1.0f, .grabAtS = -1.0f, .vGlitchAtS = -1.0f, .maxS = 25.0f,
                   .seed = 7900u + (uint32_t)sd};
    hrRun_t r;
    hrRun(&sc, &r);
    okB += r.landT < 0.0f && r.last.phase == VZ_HOLD && r.last.camHealthy && r.hMaxAfter5 <= 0.05f;
  }
  CHECK(okB == 6, "camera lost 2 s in HOLD: closed again, no LAND (%d/6)", okB);
  /* no camera-measured hover yet (lost 0.6 s into HOLD, before a 1 s quiet window): capQ 0 */
  {
    hrScen_t sc = {.hTarget = 1.00f, .camLossAtS = 0.6f, .camLossLenS = -1.0f, .landAtS = -1.0f, .resumeAtS = -1.0f,
                   .hushAtS = -1.0f, .grabAtS = -1.0f, .vGlitchAtS = -1.0f, .maxS = 3.0f, .seed = 7910u};
    hrRun_t r;
    hrRun(&sc, &r);
    CHECK(!(r.last.capQ > 0.0f) && r.openSteps == 0, "camera lost before a quiet second of HOLD: no capQ (%.0f)",
          r.last.capQ);
  }
}

/* The offset in the open loop: the observer's last (vzInput_t camBias on its last used step), not the stored bAir. A
 * stored bAir 8 mg off: with it the open loop drifted 0.5 x 9.81 x 0.008 x t^2 (0.98 m in 5 s on the estimate); with
 * the last offset the estimate's speed stays within the true one's drift. */
static void testLastBias(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, 1.00f);
  c.bAir += 0.008f;
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.accAirG = c.bAir - 0.008f;   /* the true in-flight offset: 8 mg below the stored one */
  pc.camLossS = 10.0f;
  pc.camLossLenS = 100.0f;
  warmCamera(&pc);
  vzState_t st;
  vzInit(&st, &c);
  vzSimState_t sim;
  vzSimInit(&sim, &pc, 7950u);
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  float applied = 0.0f, vbat = sim.vbat, biasAtLast = 0.0f, errMax = 0.0f;
  bool camWas = false;
  float tLoss = -1.0f;
  for (int k = 0; k < 16000; k++) {
    const float t = (float)k * DT;
    if (k % 100 == 0) {
      vbat = sim.vbat;
    }
    vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = 0.5f,
                    .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy,
                    .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias};
    vzStep(&st, &in, &o);
    if (o.camHealthy) {
      biasAtLast = in.camBias;
    }
    if (camWas && !o.camHealthy && tLoss < 0.0f) {
      tLoss = t;
    }
    camWas = o.camHealthy;
    if (tLoss >= 0.0f && t - tLoss <= 5.0f && o.phase == VZ_HOLD) {
      errMax = fmaxf(errMax, fabsf(o.v - sim.v));
    }
    vzSimStep(&sim, &pc, o.thrust, DT);
    applied = o.thrust;
    if (o.phase != VZ_HOLD && tLoss >= 0.0f) {
      break;
    }
  }
  CHECK(tLoss >= 0.0f && st.lastBias == biasAtLast && errMax <= 0.10f,
        "camera lost: the open loop uses the observer's last offset (%.4f g, bAir %.4f), the speed estimate within "
        "%.3f m/s of the truth over 5 s (bAir alone: ~0.39)", st.lastBias, c.bAir, errMax);
}

/* EST_RANGE in HOLD: a 1.5 m/s glitch in the speed estimate re-seeds it (from camVz with the camera), an event, no
 * LAND; the height out of range with the camera in use is the runaway guard: LAND. */
static void testEstHold(void)
{
  /* a 1.5 m/s speed glitch on a camera HOLD (direct state): re-seeded from camVz, the event, still HOLD; without the
   * camera clamped to vAbsMax */
  {
    vzConfig_t c;
    vzDefaultConfig(&c);
    bool good[2] = {false, false};
    for (int q = 0; q < 2; q++) {
      vzState_t st;
      vzInit(&st, &c);
      st.phase = VZ_HOLD;
      st.liftedOff = st.holdReached = st.started = st.kFrozen = true;
      st.vf = 3.7f;
      st.khat = st.kfrz = c.kPrior;
      st.tLo = 35000.0f;
      st.tHold = st.tHoldQ = 44000.0f;
      st.tQc = 44000.0f;
      st.capQ = 44000.0f * (1.0f - c.kfrzBias);
      st.h = 0.40f;
      st.tPhase = 5.0f;
      st.v = st.vHp = 1.5f;
      vzOutput_t o;
      vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.7f,
                      .thrustApplied = 44000.0f, .camHealthy = q == 0, .camVzValid = true, .camVz = 0.20f};
      vzStep(&st, &in, &o);
      good[q] = o.phase == VZ_HOLD && o.evt == (uint8_t)VZ_R_EST_RANGE && o.evtN == 1 &&
                (q == 0 ? fabsf(o.v - 0.20f) < 0.01f : fabsf(o.v - c.vAbsMax) < 0.01f);
    }
    CHECK(good[0] && good[1], "a 1.5 m/s speed glitch in HOLD: re-seeded from camVz (%d) or clamped to vAbsMax "
          "without the camera (%d), the EST_RANGE event, no LAND", good[0], good[1]);
  }
  /* the height guard on a camera HOLD: hOff (a move's offset) put hMax + hOff under the estimate 1 s into HOLD */
  {
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, 0.40f);
    vzSimConfig_t pc;
    vzSimDefaultConfig(&pc);
    pc.camWakeS = 0.0f;
    vzState_t st;
    vzInit(&st, &c);
    vzSimState_t sim;
    vzSimInit(&sim, &pc, 8010u);
    vzOutput_t o;
    memset(&o, 0, sizeof(o));
    float applied = 0.0f, vbat = sim.vbat, holdT = -1.0f;
    for (int k = 0; k < 20000 && o.phase != VZ_LAND; k++) {
      const float t = (float)k * DT;
      if (k % 100 == 0) {
        vbat = sim.vbat;
      }
      vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = 0.5f,
                      .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .camHealthy = sim.camHealthy,
                      .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias,
                      .hOff = holdT >= 0.0f && t - holdT >= 1.0f ? -0.70f : 0.0f};
      vzStep(&st, &in, &o);
      if (o.phase == VZ_HOLD && holdT < 0.0f) {
        holdT = t;
      }
      vzSimStep(&sim, &pc, o.thrust, DT);
      applied = o.thrust;
    }
    CHECK(o.phase == VZ_LAND && o.landReason == VZ_R_EST_RANGE && o.camHealthy,
          "the height estimate over hMax + hOff on a camera HOLD: LAND EST_RANGE, the runaway guard (%s %s)",
          vzPhaseName(o.phase), vzReasonName(o.landReason));
  }
}

/* SINK and resume: the FSM's CMD_LAND from HOLD, then vzInput_t.resume with the camera estimate back: HOLD again
 * (RESUME, its event); refused with a contact latch, blind, or a LAND the controller started itself. */
static void testResume(void)
{
  int ok = 0;
  for (int sd = 0; sd < 4; sd++) {
    hrScen_t sc = {.hTarget = 1.00f, .camLossAtS = 6.0f, .camLossLenS = 2.0f, .landAtS = 7.0f, .resumeAtS = 8.6f,
                   .hushAtS = -1.0f, .grabAtS = -1.0f, .vGlitchAtS = -1.0f, .maxS = 16.0f, .seed = 8100u + (uint32_t)sd};
    hrRun_t r;
    hrRun(&sc, &r);
    float at;
    ok += r.resumes == 1 && r.last.phase == VZ_HOLD && hrHasEvt(&r, VZ_R_RESUME, &at) && r.resumeT - r.holdT >= 8.6f &&
          r.resumeT - r.holdT < 8.7f;
  }
  CHECK(ok == 4, "SINK (CMD_LAND), the camera back, resume: HOLD RESUME with its event (%d/4)", ok);
  /* no resume input: the LAND goes on to the ground */
  {
    hrScen_t sc = {.hTarget = 1.00f, .camLossAtS = 6.0f, .camLossLenS = 2.0f, .landAtS = 7.0f, .resumeAtS = -1.0f,
                   .hushAtS = -1.0f, .grabAtS = -1.0f, .vGlitchAtS = -1.0f, .maxS = 30.0f, .seed = 8120u};
    hrRun_t r;
    hrRun(&sc, &r);
    CHECK(r.resumes == 0 && r.last.phase == VZ_DONE, "without vzInput_t.resume a SINK lands (%s)",
          vzPhaseName(r.last.phase));
  }
  /* a LAND the controller started (EST_RANGE here: not CMD_LAND) never resumes, nor one with the contact latch, nor
   * on VZ_CMD_LAND; a blind one resumes as any since 2026-10-15 (user decision (B): every LAND after a contact is
   * blind, a SINK's descent with it; the state machine does not ask in a contact's window: test_auto_fsm.c) */
  {
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzState_t st;
    vzInit(&st, &c);
    st.phase = VZ_LAND;
    st.liftedOff = st.holdReached = st.started = true;
    st.vf = 3.7f;
    st.khat = c.kPrior;
    st.tHold = 45000.0f;
    st.landReason = VZ_R_EST_RANGE;
    vzOutput_t o;
    vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.7f,
                    .thrustApplied = 45000.0f, .camHealthy = true, .camVzValid = true, .resume = true};
    vzStep(&st, &in, &o);
    const bool notCmd = o.phase == VZ_LAND;
    st.landReason = VZ_R_CMD_LAND;
    st.contact = true;
    vzStep(&st, &in, &o);
    const bool notContact = o.phase == VZ_LAND;
    st.contact = false;
    in.cmd = VZ_CMD_LAND;
    vzStep(&st, &in, &o);
    const bool notLandCmd = o.phase == VZ_LAND;
    in.cmd = VZ_CMD_TAKEOFF;
    vzState_t blindSt = st;
    vzStep(&st, &in, &o);
    CHECK(notCmd && notContact && notLandCmd && o.phase == VZ_HOLD && o.reason == VZ_R_RESUME,
          "resume only from a CMD_LAND LAND, no contact latch, with VZ_CMD_TAKEOFF (%d %d %d, then %s)",
          notCmd, notContact, notLandCmd, vzPhaseName(o.phase));
    /* blind: in its flat part (the SINK's slow LAND from HOLD) and descending; past the flat part, or not descending
     * for stallS (the floor in ground effect: no stall rule touched it down first), never (docs/autonomy-a2b.md
     * 22.10) */
    in.blind = true;
    blindSt.landSlow = true;
    blindSt.landFlatEff = 3.0f;
    blindSt.tPhase = 1.0f;
    vzState_t pastFlat = blindSt, atRest = blindSt;
    vzStep(&blindSt, &in, &o);
    CHECK(o.phase == VZ_HOLD && o.reason == VZ_R_RESUME && blindSt.resumed,
          "a blind SINK (a contact earlier in the sortie) resumes too, in its flat part: %s %s", vzPhaseName(o.phase),
          vzReasonName(o.reason));
    pastFlat.tPhase = 3.0f;
    vzStep(&pastFlat, &in, &o);
    const bool pastOk = o.phase == VZ_LAND && !pastFlat.resumed;
    atRest.stallT = atRest.cfg.stallS;
    atRest.landFlatBase = 2.5f;   /* the floor rule has not ended the flat part: the refusal is the rest's own */
    atRest.h = 0.5f;              /* ... nor did the stall rule latch contact (the estimate above tdNearH) */
    vzStep(&atRest, &in, &o);
    const bool restOk = o.phase == VZ_LAND && !atRest.resumed;
    CHECK(pastOk && restOk && !atRest.contact, "a blind SINK past its flat part (%d) or not descending for stallS (%d, "
          "no contact latch %d) does not resume", pastOk, restOk, !atRest.contact);
  }
}

/* Events: at most one of the same reason per VZ_EVT_GAP_S; another reason at once. vzLearnK refuses a sortie with an
 * event or a resume. */
static void testEvents(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzState_t st;
  vzInit(&st, &c);
  st.phase = VZ_HOLD;
  st.liftedOff = st.holdReached = st.started = st.kFrozen = true;
  st.vf = 3.7f;
  st.khat = st.kfrz = c.kPrior;
  st.tLo = 35000.0f;
  st.tHold = st.tHoldQ = 44000.0f;
  st.h = 0.40f;
  st.tPhase = 5.0f;
  st.loopEnd = c.holdLoopS;
  vzOutput_t o;
  vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.7f, .thrustApplied = 44000.0f,
                  .camHealthy = true, .camVzValid = true};
  int glitches = 0;
  for (int k = 0; k < 1500; k++) {
    if (k % 100 == 0) {
      st.v = 1.5f;   /* a speed glitch every 0.1 s */
      glitches++;
    }
    vzStep(&st, &in, &o);
  }
  const uint8_t n1 = o.evtN;
  CHECK(n1 == 2 && o.evt == (uint8_t)VZ_R_EST_RANGE,
        "the same event reason at most once per %.1f s: %d glitches in 1.5 s give %d events", (double)VZ_EVT_GAP_S,
        glitches, (int)n1);
  float k = c.kPrior;
  st.phase = VZ_DONE;
  st.touchdown = true;
  st.tAir = 10.0f;
  st.kLearn = c.kPrior * 1.02f;
  st.landReason = VZ_R_CMD_LAND;
  const bool withEvt = vzLearnK(&st, &k);
  st.evtAny = false;
  st.resumed = true;
  const bool withResume = vzLearnK(&st, &k);
  st.resumed = false;
  const bool clean = vzLearnK(&st, &k);
  CHECK(!withEvt && !withResume && clean, "vzLearnK refuses a sortie with an event (%d) or a resume (%d), not a clean "
        "one (%d)", withEvt, withResume, clean);
}

static void testHoldReport(void)
{
  testHushHold();
  testHushClimb();
  testClimbFaults();
  testHiBound();
  testKFloor();
  testCamLostCap();
  testLastBias();
  testEstHold();
  testResume();
  testEvents();
}

/* Every LAND after a contact is the blind one (user decision (B) 2026-10-15, docs/autonomy-a2b.md 22.9), also long after
 * it, so a blind LAND can begin in a closed-loop hold that pushes (braking a descent, climbing on a wrong camera): its
 * hover reference from HOLD is the lowest of T_hold, tHoldQ and the camera-measured quiet hover tQc scaled to the
 * battery now (x Vf then / Vf now); not blind, T_hold as before. In a blind LAND the stall rule (the ceiling at stallRate,
 * the contact latch) acts only with the estimate below tdNearH (a float at the floor; never a stall read in the air). */
static void blindRefLand(vzState_t *st, vzOutput_t *o, float tHold, float tHoldQ, float tQc, float vfQc, float vf,
                         bool blind, bool estRange)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, 1.0f);
  vzInit(st, &c);
  st->phase = VZ_HOLD;
  st->liftedOff = st->holdReached = st->started = st->kFrozen = st->hoverKnown = true;
  st->vf = vf;
  st->khat = st->kfrz = c.kPrior;
  st->tLo = 35000.0f;
  st->tHold = tHold;
  st->tHoldQ = tHoldQ;
  st->tQc = tQc;
  st->vfQc = vfQc;
  st->capQ = st->capLim = tQc * (1.0f - c.kfrzBias);
  st->h = estRange ? 3.0f : 1.0f;   /* above hMax: the controller's own LAND EST_RANGE (the camera estimate in use) */
  st->tPhase = 5.0f;
  st->loopEnd = 0.0f;
  vzInput_t in = {.dt = DT, .cmd = estRange ? VZ_CMD_TAKEOFF : VZ_CMD_LAND, .accZ = 1.0f, .gravZ = 1.0f, .vbat = vf,
                  .thrustApplied = tHold, .camHealthy = true, .camVzValid = true, .blind = blind};
  vzStep(st, &in, o);
}

static void testBlindEveryLand(void)
{
  vzState_t st;
  vzOutput_t o;
  const float top = 0.98f, start = 0.95f;   /* vzDefaultConfig landTop, landStart */
  /* a hold pushing 4 % over the quiet hover (braking a descent): blind, the quiet hover; not blind, T_hold as before */
  blindRefLand(&st, &o, 47000.0f, 47000.0f, 45000.0f, 3.7f, 3.7f, true, false);
  CHECK(o.phase == VZ_LAND && fabsf(o.tRef - 45000.0f) < 1.0f && fabsf(st.ceiling - top * 45000.0f) < 2.0f &&
            st.landSlow,
        "blind LAND from a hold at T_hold 47000, tQc 45000: T_ref %.0f (tQc), the flat part at %.0f", o.tRef, st.ceiling);
  blindRefLand(&st, &o, 47000.0f, 47000.0f, 45000.0f, 3.7f, 3.7f, false, false);
  CHECK(o.phase == VZ_LAND && fabsf(o.tRef - 47000.0f) < 1.0f, "not blind: T_ref %.0f (T_hold, as before)", o.tRef);
  /* T_hold below the quiet hover (braking a climb): T_hold */
  blindRefLand(&st, &o, 44000.0f, 44000.0f, 45000.0f, 3.7f, 3.7f, true, false);
  CHECK(fabsf(o.tRef - 44000.0f) < 1.0f, "blind, T_hold 44000 under tQc 45000: T_ref %.0f (T_hold)", o.tRef);
  /* the battery sagged since the window (3.90 -> 3.60 V): tQc scaled up to 48750, T_hold 49000 above it */
  blindRefLand(&st, &o, 49000.0f, 49000.0f, 45000.0f, 3.9f, 3.6f, true, false);
  CHECK(fabsf(o.tRef - 45000.0f * 3.9f / 3.6f) < 2.0f, "blind, tQc 45000 at 3.90 V, now 3.60 V, T_hold 49000: T_ref %.0f "
        "(tQc x 3.90 / 3.60 = %.0f)", o.tRef, 45000.0f * 3.9f / 3.6f);
  /* no quiet window yet (tQc 0): the lower of T_hold and tHoldQ, as before 2026-10-15 */
  blindRefLand(&st, &o, 47000.0f, 46000.0f, 0.0f, 0.0f, 3.7f, true, false);
  CHECK(fabsf(o.tRef - 46000.0f) < 1.0f, "blind without tQc: T_ref %.0f (tHoldQ)", o.tRef);
  /* the controller's own LAND (EST_RANGE) begins blind: the bounded reference and, since every LAND after a contact is
   * the slow blind one (docs/autonomy-a2b.md 22.10), its flat part at landTop; not blind, landStart falling at once */
  blindRefLand(&st, &o, 47000.0f, 47000.0f, 45000.0f, 3.7f, 3.7f, true, true);
  CHECK(o.phase == VZ_LAND && o.landReason == VZ_R_EST_RANGE && fabsf(o.tRef - 45000.0f) < 1.0f &&
            fabsf(st.ceiling - top * 45000.0f) < 2.0f && st.landSlow,
        "the controller's EST_RANGE, blind: T_ref %.0f (tQc), ceiling %.0f (landTop, the slow blind LAND) (%s)", o.tRef,
        st.ceiling, vzReasonName(o.landReason));
  blindRefLand(&st, &o, 47000.0f, 47000.0f, 45000.0f, 3.7f, 3.7f, false, true);
  CHECK(o.phase == VZ_LAND && o.landReason == VZ_R_EST_RANGE && fabsf(o.tRef - 47000.0f) < 1.0f &&
            fabsf(st.ceiling - start * 47000.0f) < 2.0f && !st.landSlow,
        "the controller's EST_RANGE, not blind: T_ref %.0f (T_hold), ceiling %.0f (landStart, falling at once, as "
        "before)", o.tRef, st.ceiling);

  /* the stall rule in a blind LAND: the estimate at 0.50 m and not descending: nothing (the flat part holds); at 0.05 m:
   * the ceiling at stallRate and the contact latch (the hold's lean blocked by the glue), from VZ_BLIND_STALL_ARM_S on
   * (not before: the open loop starts the descent slowly) */
  for (int k = 0; k < 3; k++) {
    blindRefLand(&st, &o, 45000.0f, 45000.0f, 45000.0f, 3.7f, 3.7f, k != 2, false);
    vzInput_t in = {.dt = DT, .cmd = VZ_CMD_LAND, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.7f, .thrustApplied = 44100.0f,
                    .camHealthy = true, .camVzValid = true, .blind = k != 2};
    const float c0 = st.ceiling;
    float cArm = 0.0f;
    bool conArm = false;
    for (int i = 0; i < 1500; i++) {   /* 1.5 s: v stays 0 (camVz 0, no acceleration) */
      st.h = k == 1 ? 0.05f : 0.50f;
      vzStep(&st, &in, &o);
      if (o.phase == VZ_LAND && st.tPhase < VZ_BLIND_STALL_ARM_S - 0.002f) {
        cArm = st.ceiling;
        conArm = o.contact;
      }
    }
    if (k == 0) {
      CHECK(fabsf(st.ceiling - c0) < 1.0f && !o.contact && o.phase == VZ_LAND,
            "blind LAND, the estimate 0.50 m and not descending for 1.5 s: the flat part holds (ceiling %.0f -> %.0f), "
            "no contact latch", c0, st.ceiling);
    } else if (k == 1) {
      CHECK(fabsf(cArm - c0) < 1.0f && !conArm && st.ceiling < c0 - 0.3f * 45000.0f && o.contact,
            "blind LAND, the estimate 0.05 m and not descending: nothing before VZ_BLIND_STALL_ARM_S (ceiling %.0f, "
            "contact %d), then the stall rule (ceiling %.0f -> %.0f), the contact latch %d", cArm, conArm, c0,
            st.ceiling, o.contact);
    } else {
      CHECK(st.ceiling < c0 - 0.3f * 45000.0f && !o.contact, "not blind, the estimate 0.50 m and not descending: the stall "
            "rule as before (ceiling %.0f -> %.0f), no contact latch above tdNearH", c0, st.ceiling);
    }
  }
}

/* A blind LAND's floor (docs/autonomy-a2b.md 22.10): after a 60 s HOLD at 1.00 m its flat part is the reference
 * height's 3.33 s plus the drift allowance 5.1 s. The estimate reads 0.50 m throughout (drifted: the stall rule never
 * acts). Kinds: 0 not descending from the start: the flat part ends at its base (3.33 s), then the ceiling falls at
 * landRate (not stallRate), no contact latch; 1 descending at 0.30 m/s: the allowance runs in full; 2 descending, then
 * at rest from base + 1 s: it ends VZ_BLIND_FLOOR_S later; 3 at rest, checked just before the base: nothing ends it
 * before; 4 descending with the contact latch from 1 s: it ends at the base; 5 not blind, at rest: landFlatEff as it
 * was (the stall rule acts there and touches down within 2 s). */
static void testBlindFloor(void)
{
  const float tRef = 45000.0f, top = 0.98f, rate = 0.10f;   /* vzDefaultConfig landTop, landRate */
  for (int k = 0; k < 6; k++) {
    vzConfig_t c;
    vzDefaultConfig(&c);
    vzConfigSetHeight(&c, 1.0f);
    vzState_t st;
    vzInit(&st, &c);
    st.phase = VZ_HOLD;
    st.liftedOff = st.holdReached = st.started = st.kFrozen = st.hoverKnown = true;
    st.vf = 3.7f;
    st.khat = st.kfrz = c.kPrior;
    st.tLo = 35000.0f;
    st.tHold = st.tHoldQ = st.tQc = tRef;
    st.vfQc = 3.7f;
    st.capQ = st.capLim = tRef * (1.0f - c.kfrzBias);
    st.h = 0.50f;
    st.tPhase = 60.0f;   /* the HOLD's length: the drift allowance min(2.0, 0.017 x 60) / 0.20 = 5.1 s */
    st.loopEnd = 0.0f;
    const bool blind = k != 5;
    vzInput_t in = {.dt = DT, .cmd = VZ_CMD_LAND, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.7f, .thrustApplied = tRef,
                    .camHealthy = true, .camVzValid = true, .blind = blind};
    vzOutput_t o;
    vzStep(&st, &in, &o);   /* LAND CMD_LAND from the 60 s HOLD */
    const float base = st.landFlatBase, flat0 = st.landFlatEff;
    const bool startOk = o.phase == VZ_LAND && st.landSlow && fabsf(flat0 - (2.0f + 0.4f / 0.3f + 5.1f)) < 0.01f &&
                         (!blind || fabsf(base - (2.0f + 0.4f / 0.3f)) < 0.01f);
    float endAt = -1.0f, ceilAt = 0.0f, ceilBefore = 0.0f;
    const float tEnd = k == 1 ? base + 3.0f : k == 2 ? base + 2.0f : k == 3 ? base - 0.01f : k == 5 ? 2.0f : base + 1.0f;
    while (st.tPhase < tEnd && o.phase == VZ_LAND) {
      const bool rest = k == 0 || k == 3 || k == 5 || (k == 2 && st.tPhase >= base + 1.0f);
      st.v = rest ? 0.0f : -0.30f;
      st.h = 0.50f;
      in.camVz = st.v;
      if (k == 4 && st.tPhase >= 1.0f) {
        st.contact = true;
      }
      in.thrustApplied = st.ceiling;
      if (st.tPhase < base - 0.05f) {
        ceilBefore = st.ceiling;
      }
      vzStep(&st, &in, &o);
      if (endAt < 0.0f && st.landFlatEff < flat0) {
        endAt = st.landFlatEff;
      }
    }
    ceilAt = st.ceiling;
    bool ok = startOk && (o.phase == VZ_LAND || k == 5);
    if (k == 0) {
      ok = ok && fabsf(endAt - base) < 0.01f && fabsf(ceilAt - tRef * (top - rate * 1.0f)) < 0.005f * tRef &&
           !o.contact;
    } else if (k == 1) {
      ok = ok && endAt < 0.0f && fabsf(ceilAt - top * tRef) < 1.0f;
    } else if (k == 2) {
      ok = ok && fabsf(endAt - (base + 1.0f + VZ_BLIND_FLOOR_S)) < 0.01f;
    } else if (k == 3) {
      ok = ok && endAt < 0.0f && fabsf(ceilAt - top * tRef) < 1.0f && fabsf(ceilBefore - top * tRef) < 1.0f;
    } else if (k == 4) {
      ok = ok && fabsf(endAt - base) < 0.01f;
    } else {
      ok = ok && endAt < 0.0f && st.landFlatEff == flat0 && o.phase == VZ_DONE;   /* the stall rule touched it down */
    }
    CHECK(ok, "blind LAND's floor after a 60 s HOLD, kind %d: flat part %.2f s (base %.2f), ended at %.3f s (-1: not), "
          "ceiling %.0f at %.2f s (x T_ref %.3f), contact %d, %s", k, flat0, base, endAt, ceilAt, st.tPhase,
          ceilAt / tRef, o.contact, vzPhaseName(o.phase));
  }
}

/* No air clock (2026-10-15, docs/autonomy-a2b.md 22; 120 s from 2026-10-14, 18.1): the controller's HOLD backstop is
 * holdMaxS 599 s under VZ_HOLD_MAX_LIMIT_S 600 s (600.5 is a BAD_CONFIG), which no battery reaches (~65 s a flight),
 * and the LAND after a long HOLD keeps its flat part and timeout longer by the drift allowance, capped at
 * VZ_LAND_DRIFT_MAX_M for every HOLD length. landTimesAt: the flat part and the timeout of a LAND from HOLD at hTarget
 * after holdS of HOLD (not blind / blind). */
static void landTimesAt(float hTarget, float holdS, bool blind, float *flat, float *landMax)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, hTarget);
  vzState_t st;
  vzInit(&st, &c);
  st.phase = VZ_HOLD;
  st.liftedOff = st.holdReached = st.started = st.kFrozen = true;
  st.vf = 3.6f;
  st.khat = st.kfrz = c.kPrior;
  st.tLo = 35000.0f;
  st.tHold = st.tHoldQ = 45000.0f;
  st.h = hTarget;
  st.tPhase = holdS - DT;   /* this step adds DT: the HOLD is holdS old at the LAND */
  vzOutput_t o;
  vzInput_t in = {.dt = DT, .cmd = VZ_CMD_LAND, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.6f, .thrustApplied = 45000.0f,
                  .camHealthy = true, .camVzValid = true, .blind = blind};
  vzStep(&st, &in, &o);
  *flat = o.phase == VZ_LAND ? st.landFlatEff : -1.0f;
  *landMax = o.phase == VZ_LAND ? o.landMaxS : -1.0f;
}

static void testHoldBackstop600(void)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  CHECK(vzConfigValid(&c) && c.holdMaxS == 599.0f, "the default config is valid with holdMaxS 599 s");
  c.holdMaxS = VZ_HOLD_MAX_LIMIT_S;
  CHECK(vzConfigValid(&c) && VZ_HOLD_MAX_LIMIT_S == 600.0f, "holdMaxS 600 s (the limit) is valid");
  c.holdMaxS = 600.5f;
  vzState_t st;
  vzOutput_t o;
  vzInit(&st, &c);
  vzInput_t in = {.dt = DT, .cmd = VZ_CMD_TAKEOFF, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.9f};
  vzStep(&st, &in, &o);
  CHECK(!vzConfigValid(&c) && o.phase == VZ_CUT && o.reason == VZ_R_BAD_CONFIG && o.thrust == 0.0f,
        "holdMaxS 600.5 s: BAD_CONFIG, CUT (%s)", vzReasonName(o.reason));

  /* the drift allowance: from 1.20 m (flat 4.0 s, timeout 8.0 s without it) and 1.05 m (3.5 s, 7.5 s) */
  static const struct { float h, holdS, extra; } lt[] = {
    {1.20f, 17.9f, 0.0f}, {1.20f, 58.8f, 5.0f}, {1.20f, 80.0f, 6.8f}, {1.20f, 118.0f, 10.0f}, {1.20f, 119.0f, 10.0f},
    {1.20f, 300.0f, 10.0f}, {1.20f, 599.0f, 10.0f},
    {1.05f, 58.8f, 5.0f}, {1.05f, 80.0f, 6.8f}, {1.05f, 118.0f, 10.0f}, {1.05f, 599.0f, 10.0f},
  };
  int ok = 0;
  for (size_t i = 0; i < sizeof(lt) / sizeof(lt[0]); i++) {
    const float baseFlat = lt[i].h > 1.1f ? 4.0f : 3.5f, baseMax = lt[i].h > 1.1f ? 8.0f : 7.5f;
    for (int b = 0; b < 2; b++) {
      float flat, landMax;
      landTimesAt(lt[i].h, lt[i].holdS, b == 1, &flat, &landMax);
      const bool good = fabsf(flat - (baseFlat + lt[i].extra)) < 0.01f && fabsf(landMax - (baseMax + lt[i].extra)) < 0.01f;
      ok += good;
      if (!good || b == 0) {
        printf("info: LAND from %.2f m after a %.1f s HOLD%s: flat part %.2f s, timeout %.2f s (+%.2f s of drift "
               "allowance)\n", lt[i].h, lt[i].holdS, b ? " (blind)" : "", flat, landMax, lt[i].extra);
      }
    }
  }
  CHECK(ok == 2 * (int)(sizeof(lt) / sizeof(lt[0])), "the drift allowance: +5.0 s at 58.8 s, +6.8 s at 80 s, +10.0 s "
        "from 117.6 s to 599 s (VZ_LAND_DRIFT_MAX_M %.1f m), flat part and timeout, blind or not (%d/%d)",
        (double)VZ_LAND_DRIFT_MAX_M, ok, 2 * (int)(sizeof(lt) / sizeof(lt[0])));

  /* a 599 s camera HOLD on the sim, never commanded to land: HOLD_TIMEOUT at 599 s (598.4-599.5 s: tPhase sums 1 ms
   * steps in float, which reads ~0.1 % early over 599 s), then the slow LAND with its +10 s flat part down to the
   * ground. The sim's battery does not drain; on the drone BATT_LOW ends every flight long before (~65 s). The camera
   * offset c -0.003 m/s cancels the sim's own slow sink over 300 s (its true height fell ~3 mm/s against the estimate:
   * at c 0 two of four reached the floor's ground effect and ended K_RANGE there, at Khat's HOLD floor). */
  int okH = 0;
  float durLo = 999.0f, durHi = 0.0f, imp = 0.0f, landS = 0.0f;
  for (int i = 0; i < 4; i++) {
    vmScen_t sc = {.hTarget = i < 2 ? 1.00f : 1.20f, .kErr = 0.02f * (float)(i - 1), .camC = -0.003f,
                   .landAtS = -1.0f, .camLossAtS = -1.0f, .seed = 4700u + (uint32_t)i, .maxS = 640.0f};
    vmRun_t r;
    vmRun(&sc, &r);
    durLo = fminf(durLo, r.holdDurS);
    durHi = fmaxf(durHi, r.holdDurS);
    imp = fmaxf(imp, r.impactV);
    landS = fmaxf(landS, r.landS);
    const bool good = r.held && r.landReason == VZ_R_HOLD_TIMEOUT && r.holdDurS >= 598.4f && r.holdDurS <= 599.5f &&
                      r.endPhase == VZ_DONE && r.landSlow && r.landS < r.landMaxS && r.impactV <= 0.5f;
    okH += good;
    if (!good) {
      printf("info: 599 s HOLD %d: LAND %s after %.3f s of HOLD, end %s, slow %d, %.2f of %.2f s, impact %.2f\n", i,
             vzReasonName(r.landReason), r.holdDurS, vzPhaseName(r.endPhase), r.landSlow, r.landS, r.landMaxS,
             r.impactV);
    }
  }
  printf("info: 599 s camera HOLD (sim): HOLD_TIMEOUT after %.3f..%.3f s, LAND to touchdown max %.2f s, impact max "
         "%.2f m/s\n", durLo, durHi, landS, imp);
  CHECK(okH == 4, "a HOLD never commanded to land: HOLD_TIMEOUT at 598.4-599.5 s, a slow LAND to a touchdown <= "
        "0.5 m/s (%d/4)", okH);
}

/* ---- SET_HEIGHT: vzShiftHeight() (2026-10-15, auto_fsm.h AUTO_SH_*, docs/autonomy-a2b.md 23) ---------------------- */

/* One controller step on the sim's inputs, the camera observer model's included. */
static void shStep(vzState_t *st, const vzSimState_t *sim, vzCmd_t cmd, float hOff, float applied, vzOutput_t *o)
{
  vzInput_t in = { .dt = DT, .cmd = cmd, .accZ = sim->accZ, .gravZ = 1.0f, .gyroMaxDps = 0.5f, .vbat = sim->vbat,
                   .thrustApplied = applied, .fromSim = true, .camHealthy = sim->camHealthy,
                   .camVzValid = sim->camVzValid, .camVz = sim->camVz, .camBias = sim->camBias, .hOff = hOff };
  vzStep(st, &in, o);
}

/* A camera HOLD at hTarget, holdS into it (the twin tests start from it). */
static bool shHold(float hTarget, float holdS, vzState_t *st, vzSimState_t *sim, vzSimConfig_t *sc, float *applied,
                   vzOutput_t *o)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, hTarget);
  vzSimDefaultConfig(sc);
  warmCamera(sc);
  vzInit(st, &c);
  vzSimInit(sim, sc, 777u);
  *applied = 0.0f;
  for (int k = 0; k < 40000; k++) {
    shStep(st, sim, VZ_CMD_TAKEOFF, 0.0f, *applied, o);
    vzSimStep(sim, sc, o->thrust, DT);
    *applied = o->thrust;
    if (o->phase == VZ_HOLD && o->tPhase >= holdS) {
      return o->camHealthy;
    }
    if (o->phase != VZ_SPIN && o->phase != VZ_RAMP && o->phase != VZ_CLIMB && o->phase != VZ_HOLD) {
      return false;
    }
  }
  return false;
}

static void testShiftHeight(void)
{
  vzState_t a, b, tmp;
  vzSimState_t sim;
  vzSimConfig_t sc;
  float applied;
  vzOutput_t oa, ob;
  for (int k = 0; k < 4; k++) {
    const float d = k == 0 ? 0.50f : k == 1 ? -0.50f : k == 2 ? 0.23f : -0.07f;
    CHECK(shHold(1.00f, 4.0f, &a, &sim, &sc, &applied, &oa), "a camera HOLD at 1.00 m, 4 s");
    memcpy(&b, &a, sizeof(b));
    CHECK(vzShiftHeight(&b, d), "HOLD, d %+.2f: shifted", d);
    memcpy(&tmp, &b, sizeof(tmp));
    tmp.h = a.h;
    CHECK(memcmp(&tmp, &a, sizeof(a)) == 0 && b.h == a.h + d,
          "d %+.2f: only h changed (%.4f -> %.4f); v, vHp, Khat, Kfrz, T_hold, tHoldQ, tQc/capQ, kLearn, the LAND "
          "fields as they were", d, a.h, b.h);
    /* the next 2000 steps: the thrust bit-identical to the unshifted twin, the estimates d apart (the hold is a
     * speed loop: the height itself flies nothing) */
    int same = 0;
    float dErr = 0.0f;
    for (int i = 0; i < 2000; i++) {
      shStep(&a, &sim, VZ_CMD_TAKEOFF, 0.0f, applied, &oa);
      shStep(&b, &sim, VZ_CMD_TAKEOFF, 0.0f, applied, &ob);
      uint32_t ta, tb;
      memcpy(&ta, &oa.thrust, 4);
      memcpy(&tb, &ob.thrust, 4);
      same += ta == tb && oa.phase == VZ_HOLD && ob.phase == VZ_HOLD;
      dErr = fmaxf(dErr, fabsf((ob.h - oa.h) - d));
      vzSimStep(&sim, &sc, oa.thrust, DT);
      applied = oa.thrust;
    }
    CHECK(same == 2000 && dErr < 1e-4f, "d %+.2f: 2000 steps bit-identical thrust (%d), both in HOLD, the estimates "
          "d apart (error %.2e)", d, same, dErr);
  }
  /* the EST_RANGE margin: with the reference moved by d (the state machine's hOff + d) exactly as before; without
   * it, a shift toward hMax lands EST_RANGE (the reason the offset moves with the estimate) */
  CHECK(shHold(1.00f, 3.0f, &a, &sim, &sc, &applied, &oa), "a camera HOLD at 1.00 m");
  a.h = a.cfg.hMax - 0.10f;   /* 0.10 m inside the guard */
  vzState_t c2;
  memcpy(&b, &a, sizeof(b));
  memcpy(&c2, &a, sizeof(c2));
  CHECK(vzShiftHeight(&b, 0.30f) && vzShiftHeight(&c2, 0.30f), "both shifted +0.30");
  shStep(&a, &sim, VZ_CMD_TAKEOFF, 0.0f, applied, &oa);
  shStep(&b, &sim, VZ_CMD_TAKEOFF, 0.30f, applied, &ob);
  vzOutput_t oc;
  shStep(&c2, &sim, VZ_CMD_TAKEOFF, 0.0f, applied, &oc);
  CHECK(oa.phase == VZ_HOLD && ob.phase == VZ_HOLD && oa.thrust == ob.thrust && oc.phase == VZ_LAND &&
        oc.landReason == VZ_R_EST_RANGE,
        "0.10 m inside the guard: shifted +0.30 with hOff +0.30 HOLD on (thrust %.0f = %.0f); without the offset LAND "
        "%s", oa.thrust, ob.thrust, vzReasonName(oc.landReason));
  /* refused (nothing changed) outside HOLD, for a non-finite d and beyond VZ_SHIFT_MAX_M */
  static const vzPhase_t ph[] = { VZ_IDLE, VZ_CLIMB, VZ_LAND };
  vzConfig_t cfg;
  vzDefaultConfig(&cfg);
  vzSimDefaultConfig(&sc);
  for (int k = 0; k < 3; k++) {
    CHECK(reachPhase(ph[k], &a, &sim, &sc, &cfg, &applied, &oa), "reach %s", vzPhaseName(ph[k]));
    memcpy(&b, &a, sizeof(b));
    CHECK(!vzShiftHeight(&a, 0.2f) && memcmp(&a, &b, sizeof(a)) == 0, "%s: refused, nothing changed",
          vzPhaseName(ph[k]));
  }
  CHECK(reachPhase(VZ_HOLD, &a, &sim, &sc, &cfg, &applied, &oa), "reach HOLD");
  vzInput_t cut = { .dt = DT, .cmd = VZ_CMD_CUT, .accZ = 1.0f, .gravZ = 1.0f, .vbat = 3.8f };
  memcpy(&tmp, &a, sizeof(tmp));
  vzStep(&tmp, &cut, &oa);
  memcpy(&b, &tmp, sizeof(b));
  CHECK(oa.phase == VZ_CUT && !vzShiftHeight(&tmp, 0.2f) && memcmp(&tmp, &b, sizeof(b)) == 0, "CUT: refused");
  memcpy(&b, &a, sizeof(b));
  CHECK(!vzShiftHeight(&a, NAN) && !vzShiftHeight(&a, INFINITY) && !vzShiftHeight(&a, 0.52f) &&
        !vzShiftHeight(&a, -0.52f) && memcmp(&a, &b, sizeof(a)) == 0, "HOLD: NaN, inf, |d| 0.52: refused, unchanged");
  CHECK(vzShiftHeight(&a, VZ_SHIFT_MAX_M) && vzShiftHeight(&a, -VZ_SHIFT_MAX_M) && !vzShiftHeight(NULL, 0.1f),
        "|d| %.2f accepted (AUTO_SH_MAX_M + rounding); NULL refused", (double)VZ_SHIFT_MAX_M);
}

int main(void)
{
  testDefaults();
  testCapLo110();
  testCapLoDefault();
  testHeightRange();
  testSetHeight();
  testLiftoff();
  testNoLiftoff();
  testRampAbort();
  testLandMc();
  testUserRangeMc();
  testAbsCap();
  testHoldCapLoK();
  testFooledLoop();
  testFloating();
  testClimbLandRef();
  testNoFalseTouchdown();
  testCamera();
  testLandAfterHold();
  testNearJolt();
  testNonFinite();
  testCut();
  testCmdMissing();
  testDt();
  testSpinGyroHold();
  testConfigValid();
  testLearnK();
  testDeterministic();
  testJitter();
  testIdentity();
  testVerticalMove();
  testHigh120();
  testHigh200();
  testHoffHigh();
  testLandAfterMoveDown();
  testContactLand();
  testFastLand();
  testRampAtTop();
  testHoldBackstop600();
  testHoldReport();
  testBlindEveryLand();
  testBlindFloor();
  testShiftHeight();
  testEstRange();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
