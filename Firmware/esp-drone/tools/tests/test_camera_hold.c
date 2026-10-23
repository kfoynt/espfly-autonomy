/*
 * Host unit tests for components/core/crazyflie/utils/src/camera_hold.c
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_camera_hold.c components/core/crazyflie/utils/src/camera_hold.c \
 *      -lm -o /tmp/test_camera_hold && /tmp/test_camera_hold
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "camera_hold.h"
#include "camera_hold_identity.h"
#include "auto_fsm.h"   /* autoState_t (testGainRate) */

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static const float DT = 0.08f;  /* ~12.5 fps, as in flight */

static camHoldInput_t base(void)
{
  camHoldInput_t in = {.enabled = true, .sticksCentred = true, .airborne = true, .viewOk = true,
                       .wFwd = 0.0f, .wLat = 0.0f, .dt = DT};
  return in;
}

static void run(camHoldState_t *st, camHoldInput_t in, float seconds)
{
  for (float t = 0; t < seconds; t += in.dt) {
    camHoldStep(st, &in);
  }
}

static void testDisabledAndSettle(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  in.enabled = false;
  in.wFwd = 0.2f;
  run(&st, in, 3.0f);
  CHECK(!st.engaged && st.pitch == 0.0f && st.roll == 0.0f, "disabled: no correction");

  camHoldReset(&st);
  in = base();
  in.wFwd = 0.2f;
  run(&st, in, 0.8f);
  CHECK(!st.engaged && st.pitch == 0.0f, "not engaged before the settle time (pitch %g)", st.pitch);
  run(&st, in, 0.5f);
  CHECK(st.engaged, "engaged after the settle time");
}

static void testDirections(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);              /* settle */
  in.wFwd = 0.15f;                 /* drifting toward the scene */
  run(&st, in, 1.0f);
  CHECK(st.pitch > 0.5f, "forward drift -> nose-up correction (pitch %g)", st.pitch);
  CHECK(fabsf(st.roll) < 1e-6f, "no roll from forward drift (roll %g)", st.roll);

  camHoldReset(&st);
  in = base();
  run(&st, in, 1.2f);
  in.wLat = 0.1f;                  /* drifting right */
  run(&st, in, 1.0f);
  CHECK(st.roll < -0.3f, "rightward drift -> left-side-down correction (roll %g)", st.roll);

  camHoldReset(&st);
  in = base();
  run(&st, in, 1.2f);
  in.wFwd = -0.15f;                /* backing away from the scene */
  run(&st, in, 1.0f);
  CHECK(st.pitch < -0.5f, "backward drift -> nose-down correction (pitch %g)", st.pitch);
}

static void testLimitsAndSlew(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.wFwd = 5.0f;
  in.wLat = -5.0f;
  float prevP = st.pitch, prevR = st.roll, maxStep = 0.0f;
  for (int i = 0; i < 100; i++) {
    camHoldStep(&st, &in);
    maxStep = fmaxf(maxStep, fmaxf(fabsf(st.pitch - prevP), fabsf(st.roll - prevR)));
    prevP = st.pitch;
    prevR = st.roll;
  }
  CHECK(fabsf(st.pitch) <= CAM_HOLD_LIMIT_DEG + 1e-5f && fabsf(st.roll) <= CAM_HOLD_LIMIT_DEG + 1e-5f,
        "corrections limited (%g, %g)", st.pitch, st.roll);
  CHECK(fabsf(st.pitch - CAM_HOLD_LIMIT_DEG) < 1e-4f, "saturates at the limit (%g)", st.pitch);
  CHECK(maxStep <= CAM_HOLD_SLEW_DEG_S * DT + 1e-5f, "slew limited (max step %g)", maxStep);
}

static void testIntegralPullAndClamp(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.wFwd = 0.1f;
  run(&st, in, 2.0f);
  in.wFwd = 0.0f;                  /* drift stopped, but the drone has moved forward */
  run(&st, in, 2.0f);
  CHECK(st.pitch > 0.1f, "integral keeps a pull back after moving (pitch %g)", st.pitch);
  CHECK(st.iFwd * CAM_HOLD_KI <= CAM_HOLD_I_LIMIT_DEG + 1e-5f, "integral contribution clamped");
  in.wFwd = 1.0f;
  run(&st, in, 20.0f);
  CHECK(st.iFwd * CAM_HOLD_KI <= CAM_HOLD_I_LIMIT_DEG + 1e-5f, "integral clamped under long drift (%g)",
        st.iFwd * CAM_HOLD_KI);
}

static void testOverrideAndLanding(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.wFwd = 0.3f;
  run(&st, in, 1.5f);
  CHECK(st.pitch > 1.0f, "correction built up (%g)", st.pitch);

  in.sticksCentred = false;        /* pilot takes over */
  camHoldStep(&st, &in);
  CHECK(!st.engaged, "stick input disengages immediately");
  run(&st, in, 1.5f);
  CHECK(st.pitch == 0.0f, "correction decays to zero after override (%g)", st.pitch);
  CHECK(st.iFwd == 0.0f, "integral reset after %g s disengaged", (double)CAM_HOLD_RESET_S);

  in = base();
  run(&st, in, 1.2f);
  in.wFwd = 0.3f;
  run(&st, in, 1.0f);
  in.airborne = false;             /* landed */
  camHoldStep(&st, &in);
  CHECK(!st.engaged && st.iFwd == 0.0f && st.airborneTime == 0.0f, "landing resets integral and settle timer");
}

static void testBadInput(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.wFwd = NAN;
  camHoldStep(&st, &in);
  CHECK(!st.engaged && isfinite(st.pitch), "NaN input does not engage or poison the state");
  in.wFwd = 0.1f;
  in.dt = 1.0f;                    /* frame gap */
  camHoldStep(&st, &in);
  CHECK(!st.engaged, "long frame gap treated as a dropout");
  in.dt = DT;
  in.viewOk = false;
  camHoldStep(&st, &in);
  CHECK(!st.engaged, "unusable view does not engage");
}

static void testCoastOnViewLoss(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.wFwd = -0.1f;                 /* steady backward drift: the integral learns a nose-down lean */
  run(&st, in, 3.0f);
  const float learned = st.iFwd;
  CHECK(learned < 0.0f, "integral learned the drift (%g)", learned);

  in.viewOk = false;               /* texture lost, sticks still centred */
  bool active = camHoldStep(&st, &in);
  CHECK(!st.engaged && st.coasting && active, "coasts when the view is lost");
  run(&st, in, 5.0f);
  CHECK(st.iFwd == learned, "integral kept while coasting (%g vs %g)", st.iFwd, learned);
  CHECK(fabsf(st.pitch - CAM_HOLD_KI * learned) < 1e-4f, "applies only the integral term (%g vs %g)",
        st.pitch, CAM_HOLD_KI * learned);

  in.viewOk = true;                /* view back: measuring again from the learned integral */
  camHoldStep(&st, &in);
  CHECK(st.engaged && !st.coasting, "re-engages when the view returns");

  in.viewOk = false;
  in.sticksCentred = false;        /* pilot input: no coasting */
  active = camHoldStep(&st, &in);
  CHECK(!st.coasting && !active, "stick input stops coasting");
  run(&st, in, 1.5f);
  CHECK(st.pitch == 0.0f && st.iFwd == 0.0f, "decays and resets after override (%g, %g)", st.pitch, st.iFwd);

  camHoldReset(&st);
  in = base();
  in.viewOk = false;
  run(&st, in, 2.0f);
  CHECK(st.coasting && st.pitch == 0.0f, "coasting with nothing learned applies nothing (%g)", st.pitch);
}

/* ---- the moving reference (interactive hover) ------------------------------ */

/* No reference: bit-identical to the hold before the moving reference
 * (8f51285), over 200000 frames of every regime (camera_hold_identity.h). */
#define HOLD_IDENTITY_HASH_8F51285 0x42DA004C5D79DB66ull

static void testIdentity(void)
{
  const uint64_t h = holdIdentityHash();
  CHECK(h == HOLD_IDENTITY_HASH_8F51285, "no reference: outputs bit-identical to 8f51285 (hash 0x%016llX)",
        (unsigned long long)h);
}

/* The old outputs of a state: the fields the hold had before the moving reference. */
static bool sameOld(const camHoldState_t *a, const camHoldState_t *b)
{
  return memcmp(&a->fwdF, &b->fwdF, sizeof(float)) == 0 && memcmp(&a->latF, &b->latF, sizeof(float)) == 0 &&
         memcmp(&a->iFwd, &b->iFwd, sizeof(float)) == 0 && memcmp(&a->iLat, &b->iLat, sizeof(float)) == 0 &&
         memcmp(&a->roll, &b->roll, sizeof(float)) == 0 && memcmp(&a->pitch, &b->pitch, sizeof(float)) == 0 &&
         a->engaged == b->engaged && a->coasting == b->coasting;
}

/* After a motion: once its rate references have decayed, the hold is the old
 * hold again, step for step, from the state the motion left. */
static void testIdentityAfterMotion(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.refEpoch = 1;
  for (int k = 0; k < 60; k++) {   /* 4.8 s: a reference to 0.3, rates and a feedforward */
    in.refLat = 0.005f * (float)k;
    in.rateLat = 0.06f;
    in.ffRoll = k < 10 ? 0.4f : 0.0f;
    in.wLat = 0.05f;
    camHoldStep(&st, &in);
  }
  in.rateLat = 0.0f;
  in.ffRoll = 0.0f;
  in.wLat = 0.0f;
  run(&st, in, 3.0f);   /* the rate filter decays to exactly 0 */
  CHECK(st.rateLatF == 0.0f && st.rateFwdF == 0.0f, "rate references decayed to exactly 0 (%g)", st.rateLatF);
  /* a copy with only the old fields, every reference field 0 */
  camHoldState_t old;
  camHoldReset(&old);
  old.fwdF = st.fwdF;
  old.latF = st.latF;
  old.iFwd = st.iFwd;
  old.iLat = st.iLat;
  old.airborneTime = st.airborneTime;
  old.idleTime = st.idleTime;
  old.roll = st.roll;
  old.pitch = st.pitch;
  old.engaged = st.engaged;
  old.coasting = st.coasting;
  camHoldInput_t inOld = base();
  holdIdRng = 99u;
  bool same = true;
  for (int k = 0; k < 2000; k++) {
    in.wFwd = inOld.wFwd = 0.2f * (holdIdUni() - 0.5f);
    in.wLat = inOld.wLat = 0.2f * (holdIdUni() - 0.5f);
    in.viewOk = inOld.viewOk = holdIdUni() > 0.1f;
    camHoldStep(&st, &in);
    camHoldStep(&old, &inOld);
    same = same && sameOld(&st, &old);
  }
  CHECK(same, "after the motion (reference held at its end): the same outputs as the hold without references");
}

/* A simple plant for the closed loop: point mass, lean -> acceleration, a
 * frame of latency; the scene D ahead (lateral: constant). */
typedef struct {
  float x, v, lean;      /* m, m/s, deg (applied roll) */
} plant1_t;

static void plantStep(plant1_t *p, float rollCmd, float dt)
{
  p->lean += (rollCmd - p->lean) * fminf(1.0f, dt / 0.1f);
  const float a = 9.81f * tanf(p->lean * 0.0174533f) - 0.35f * p->v;
  p->v += a * dt;
  p->x += p->v * dt;
}

/* Lateral move of 0.5 m at D 1.5 m: a trapezoid reference (x / D), its rate
 * and feedforward; the drone ends within 10 % of it and the hold's measured
 * progress pLat tracks x / D. */
static void testMovingRefLateral(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  plant1_t p = {0};
  const float D = 1.5f, R = 0.5f, vmax = 0.15f, acc = 0.1f;
  const float tA = vmax / acc, tC = (R - vmax * tA) / vmax, T = 2.0f * tA + tC;
  float prevW = 0.0f;
  float t = 0.0f;
  camHoldInput_t in = base();
  for (int k = 0; k < (int)(20.0f / DT); k++, t += DT) {
    float s, v, a;
    const float tm = t - 2.0f;   /* 2 s of hover first */
    if (tm <= 0.0f) {
      s = v = a = 0.0f;
    } else if (tm < tA) {
      s = 0.5f * acc * tm * tm; v = acc * tm; a = acc;
    } else if (tm < tA + tC) {
      s = 0.5f * acc * tA * tA + vmax * (tm - tA); v = vmax; a = 0.0f;
    } else if (tm < T) {
      const float td = T - tm;
      s = R - 0.5f * acc * td * td; v = acc * td; a = -acc;
    } else {
      s = R; v = 0.0f; a = 0.0f;
    }
    in.refEpoch = tm > 0.0f ? 1u : 0u;
    in.refLat = s / D;
    in.rateLat = v / D;
    in.ffRoll = (a + 0.3f * v) / 9.81f * 57.2958f;
    in.wLat = prevW;   /* one frame old */
    prevW = p.v / D;
    camHoldStep(&st, &in);
    for (int j = 0; j < 80; j++) {
      plantStep(&p, st.roll, DT / 80.0f);
    }
  }
  CHECK(fabsf(p.x - R) <= 0.1f * R, "lateral 0.50 m at D 1.5 m: ends at %.3f m (within 10 %%)", p.x);
  CHECK(fabsf(st.pLat * D - p.x) < 0.02f, "progress pLat x D %.3f m tracks the drone (%.3f m)", st.pLat * D, p.x);
  CHECK(st.rLat == in.refLat, "the whole reference taken (%.4f)", st.rLat);
}

/* A snap puts the integral back to its value at the start of the epoch it ends. */
static void testSnap(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  in.wLat = -0.05f;   /* a lean bias the integral learns */
  run(&st, in, 3.0f);
  in.wLat = 0.0f;
  run(&st, in, 1.0f);
  const float i0 = st.iLat;
  in.refEpoch = 1;
  for (int k = 0; k < 40; k++) {   /* the reference moves, the drone does not (blocked) */
    in.refLat = 0.01f * (float)k;
    in.rateLat = 0.1f;
    camHoldStep(&st, &in);
  }
  CHECK(fabsf(st.iLat - i0) > 0.2f, "the reference moved the integral (%.3f from %.3f)", st.iLat, i0);
  /* STOP: a new epoch that begins with a snap */
  in.refEpoch = 2;
  in.refSnapEpoch = 2;
  in.refLat = 0.0f;
  in.rateLat = 0.0f;
  camHoldStep(&st, &in);
  CHECK(fabsf(st.iLat - i0) < 1e-5f && st.epoch == 2 && st.pLat == 0.0f && st.rLat == 0.0f,
        "snap: the integral back to %.4f (%.4f), epoch 2, positions 0", i0, st.iLat);
  /* through the clamp: a reference far beyond the integral limit */
  in.refEpoch = 3;
  for (int k = 0; k < 40; k++) {
    in.refLat = 0.05f * (float)k;
    camHoldStep(&st, &in);
  }
  CHECK(fabsf(st.iLat) >= CAM_HOLD_I_LIMIT_DEG / CAM_HOLD_KI - 1e-6f && st.lostLat != 0.0f,
        "the clamp took %.3f", st.lostLat);
  in.refEpoch = 5;   /* epoch 4 never seen: a snap in between still counts */
  in.refSnapEpoch = 4;
  in.refLat = 0.0f;
  camHoldStep(&st, &in);
  CHECK(fabsf(st.iLat - i0) < 1e-5f && st.epoch == 5, "snap across a skipped epoch, through the clamp: %.4f",
        st.iLat);
  /* a new epoch without a snap keeps the integral */
  const float i1 = st.iLat;
  in.refEpoch = 6;
  camHoldStep(&st, &in);
  CHECK(st.iLat == i1 && st.epoch == 6, "a new epoch without a snap: integral kept");
}

/* While coasting the reference is not taken; the next engaged frame takes all of it. */
static void testRefWhileCoasting(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.refEpoch = 1;
  in.viewOk = false;
  for (int k = 1; k <= 10; k++) {
    in.refLat = 0.01f * (float)k;
    camHoldStep(&st, &in);
  }
  CHECK(st.coasting && st.rLat == 0.0f && st.iLat == 0.0f, "coasting: nothing taken");
  in.viewOk = true;
  camHoldStep(&st, &in);
  CHECK(st.engaged && fabsf(st.rLat - 0.1f) < 1e-6f && fabsf(st.iLat + 0.1f) < 1e-6f,
        "engaged again: the whole 0.1 taken at once (%.4f)", st.iLat);
}

/* The feedforward adds before the limit; the rate reference enters the P term. */
static void testFeedforward(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.ffRoll = 0.8f;
  in.ffPitch = -3.5f;
  run(&st, in, 1.0f);
  CHECK(fabsf(st.roll - 0.8f) < 1e-4f && fabsf(st.pitch + CAM_HOLD_LIMIT_DEG) < 1e-4f,
        "feedforward roll 0.8, pitch -3.5 limited to -2.5 (%.3f, %.3f)", st.roll, st.pitch);
  camHoldReset(&st);
  in = base();
  run(&st, in, 1.2f);
  in.refEpoch = 1;
  in.rateFwd = 0.1f;
  in.wFwd = 0.1f;   /* moving exactly at the reference rate */
  for (int k = 0; k < 30; k++) {
    in.refFwd = 0.1f * DT * (float)(k + 1);
    camHoldStep(&st, &in);
  }
  CHECK(fabsf(st.pitch) < 0.05f && fabsf(st.iFwd) < 1e-3f, "moving at the reference rate: no correction "
        "(pitch %.3f, iFwd %.4f)", st.pitch, st.iFwd);
}

/* ---- the HOVER scene gate (camHoldSceneOk, 2026-10-13 flight 1) ------------- */

#define GDT 0.0625f   /* 16 fps, exact in binary: 32 frames are 2.0 s */

/* One frame through the gate with clean values except the ones given. */
static bool gate(camHoldScene_t *g, bool hover, float residual, float dxRes, float scale)
{
  return camHoldSceneOk(g, hover, GDT, residual, dxRes, scale);
}

/* Armed: 2 s of clean HOVER frames after the first. */
static void armGate(camHoldScene_t *g)
{
  camHoldSceneReset(g);
  for (int k = 0; k < 33; k++) {
    gate(g, true, 1.0f, 0.0f, 0.0f);
  }
}

static void testSceneGateArming(void)
{
  camHoldScene_t g;
  camHoldSceneReset(&g);
  /* outside HOVER (manual flight, TAKEOFF, LAND): never refuses */
  bool all = true;
  for (int k = 0; k < 160; k++) {
    all = gate(&g, false, 12.0f, 3.0f, 0.1f) && !g.lastRejected && all;
  }
  CHECK(all && g.rejRes + g.rejDx + g.rejExp == 0 && g.hoverS == 0.0f, "not in HOVER: every frame passes (10 s of "
        "residual 12, |dxRes| 3 px, scale/dt 1.6/s)");
  /* in HOVER: not before 2.0 s, counted from its first frame (its interval began before the entry) */
  int firstRefused = -1;
  for (int k = 1; k <= 40 && firstRefused < 0; k++) {
    if (!gate(&g, true, 12.0f, 0.0f, 0.0f)) {
      firstRefused = k;
    }
  }
  CHECK(firstRefused == 33 && g.lastRejected && g.rejRes == 1, "HOVER: the first frame refused is the one 2.0 s "
        "after the first HOVER frame (frame %d, want 33)", firstRefused);
  /* leaving HOVER resets the clock; back in HOVER it waits 2 s again */
  CHECK(gate(&g, false, 12.0f, 0.0f, 0.0f) && g.hoverS == 0.0f && !g.lastRejected, "leaving HOVER: passes, clock 0");
  int passed = 0;
  for (int k = 1; k <= 32; k++) {
    passed += gate(&g, true, 12.0f, 0.0f, 0.0f);
  }
  CHECK(passed == 32 && !gate(&g, true, 12.0f, 0.0f, 0.0f), "back in HOVER: 32 frames pass, the 33rd is refused");
  /* the expansion EMA runs on every frame, armed or not (as the observer's) */
  camHoldSceneReset(&g);
  for (int k = 0; k < 3; k++) {
    gate(&g, false, 1.0f, 0.0f, 0.5f * GDT);
  }
  const float want = 0.5f * (1.0f - 0.8f * 0.8f * 0.8f);
  CHECK(fabsf(g.expEma - want) < 1e-6f, "EMA fed while unarmed: %.4f (want %.4f)", g.expEma, want);
  /* no frame (the take-off standby: NaN values) changes nothing but the clock */
  camHoldSceneReset(&g);
  CHECK(camHoldSceneOk(&g, false, GDT, NAN, NAN, NAN) && g.expEma == 0.0f, "standby call: passes, EMA untouched");
}

static void testSceneGateThresholds(void)
{
  camHoldScene_t g;
  armGate(&g);
  CHECK(gate(&g, true, 7.9f, 0.0f, 0.0f), "armed: residual 7.9 passes");
  CHECK(!gate(&g, true, 8.0f, 0.0f, 0.0f) && g.lastRejected && g.rejRes == 1, "armed: residual 8.0 refused");
  CHECK(!gate(&g, true, NAN, 0.0f, 0.0f) && g.rejRes == 2, "armed: residual NaN refused");
  CHECK(gate(&g, true, 1.0f, 1.5f, 0.0f) && gate(&g, true, 1.0f, -1.5f, 0.0f) && !g.lastRejected,
        "armed: |dxRes| 1.5 px passes");
  CHECK(!gate(&g, true, 1.0f, 1.51f, 0.0f) && !gate(&g, true, 1.0f, -1.51f, 0.0f) && g.rejDx == 2,
        "armed: |dxRes| 1.51 px refused (both signs)");
  CHECK(!gate(&g, true, 1.0f, NAN, 0.0f) && g.rejDx == 3, "armed: dxRes NaN refused");
  /* expansion: scale / dt 0.5 /s every frame, EMA 0.1 0.18 0.244 0.295 0.336: refused from the 5th frame */
  armGate(&g);
  int first = -1, refused = 0;
  for (int k = 1; k <= 10; k++) {
    if (!gate(&g, true, 1.0f, 0.0f, 0.5f * GDT)) {
      refused++;
      if (first < 0) {
        first = k;
      }
    }
  }
  CHECK(first == 5 && refused == 6 && g.rejExp == 6, "armed: scale/dt 0.5 /s refused from the 5th frame (%d), "
        "%d of 10", first, refused);
  /* recovery: the EMA decays (x0.8 a frame) below 0.3 /s */
  int back = -1;
  for (int k = 1; k <= 10 && back < 0; k++) {
    if (gate(&g, true, 1.0f, 0.0f, 0.0f)) {
      back = k;
    }
  }
  CHECK(back >= 1 && back <= 3, "expansion over: passes again after %d frames (EMA %.3f)", back, g.expEma);
  /* reasons in order: residual first */
  const uint32_t r0 = g.rejRes, d0 = g.rejDx;
  CHECK(!gate(&g, true, 9.0f, 2.0f, 0.0f) && g.rejRes == r0 + 1 && g.rejDx == d0, "residual counted first");
}

/* The residual limit as a setting (camHoldSceneSetMaxResidual, NVS holdres; 2026-10-22): the profile's default after
 * a reset (8.0 on this legacy build, 16.0 native), any other limit, 0 off (the lateral and expansion tests stay). */
static void testSceneGateResidualLimit(void)
{
  camHoldScene_t g;
  armGate(&g);
  CHECK(g.maxResidual == CAM_HOLD_SCENE_RES_DEFAULT && CAM_HOLD_SCENE_RES_X10_DEFAULT / 10.0f == CAM_HOLD_SCENE_RES_DEFAULT
#ifdef ESPFLY_NATIVE_VGA
            && g.maxResidual == 16.0f
#else
            && g.maxResidual == 8.0f && CAM_HOLD_SCENE_RES_DEFAULT == CAM_HOLD_SCENE_MAX_RESIDUAL
#endif
            , "after a reset: the profile's limit %.1f", g.maxResidual);
  camHoldSceneSetMaxResidual(&g, 16.0f);
  CHECK(gate(&g, true, 13.4f, 0.0f, 0.0f) && gate(&g, true, 15.9f, 0.0f, 0.0f) && !g.lastRejected && g.rejRes == 0,
        "limit 16.0: residual 13.4 (the table flight's largest) and 15.9 pass");
  CHECK(!gate(&g, true, 16.0f, 0.0f, 0.0f) && g.rejRes == 1, "limit 16.0: residual 16.0 refused");
  camHoldSceneSetMaxResidual(&g, 0.0f);
  CHECK(gate(&g, true, 16.0f, 0.0f, 0.0f) && gate(&g, true, 99.0f, 0.0f, 0.0f) && gate(&g, true, NAN, 0.0f, 0.0f) &&
            g.rejRes == 1,
        "limit 0 (off): residual 16, 99 and NaN pass the residual test");
  CHECK(!gate(&g, true, 1.0f, 1.6f, 0.0f) && g.rejDx == 1, "limit 0: the lateral test still refuses |dxRes| 1.6 px");
  int refused = 0;
  for (int k = 1; k <= 10; k++) {
    refused += !gate(&g, true, 1.0f, 0.0f, 0.5f * GDT);
  }
  CHECK(refused == 6 && g.rejExp == 6, "limit 0: the expansion test still refuses (6 of 10 at 0.5 /s)");
  camHoldSceneSetMaxResidual(&g, -3.0f);
  const bool negOff = g.maxResidual == 0.0f;
  camHoldSceneSetMaxResidual(&g, NAN);
  CHECK(negOff && g.maxResidual == 0.0f, "a negative or NaN limit: off");
  camHoldSceneReset(&g);
  CHECK(g.maxResidual == CAM_HOLD_SCENE_RES_DEFAULT, "a reset restores the profile's limit");
}

/* A gated frame is a lost view: the hold coasts on its integral. wFwd -0.5
 * /s (the person receding: the image shrinks) at residual 12 for 2 s in armed
 * HOVER: the hold's own test (residual < 15) passes it and it leans the drone
 * to the pitch limit; through the gate the pitch stays where it was. */
static void testSceneGateHold(void)
{
  for (int gated = 0; gated < 2; gated++) {
    camHoldState_t st;
    camHoldReset(&st);
    camHoldScene_t g;
    camHoldSceneReset(&g);
    camHoldInput_t in = base();
    in.dt = GDT;
    for (int k = 0; k < 48; k++) {   /* 3 s of clean HOVER: settled, engaged, the gate armed */
      in.viewOk = gate(&g, true, 2.0f, 0.0f, 0.0f);
      camHoldStep(&st, &in);
    }
    const float pitch0 = st.pitch;
    float pMin = pitch0, pMax = pitch0;
    int coast = 0;
    for (int k = 0; k < 32; k++) {   /* 2 s of the person */
      in.wFwd = -0.5f;
      const bool sceneOk = gate(&g, true, 12.0f, 0.0f, in.wFwd * GDT);
      in.viewOk = 12.0f < 15.0f && (!gated || sceneOk);
      camHoldStep(&st, &in);
      coast += st.coasting;
      pMin = fminf(pMin, st.pitch);
      pMax = fmaxf(pMax, st.pitch);
    }
    if (gated) {
      CHECK(st.engaged == false && coast == 32 && pMin == pitch0 && pMax == pitch0, "gated: the hold coasts every "
            "frame (%d of 32) at a constant pitch (%.3f..%.3f, was %.3f)", coast, pMin, pMax, pitch0);
    } else {
      CHECK(coast == 0 && pMin <= -CAM_HOLD_LIMIT_DEG + 1e-4f, "ungated: the hold leans to the pitch limit "
            "(min %.2f deg)", pMin);
    }
  }
}

/* The yaw part of the view test (camera_flow.c; user decision 2026-10-13,
 * evening): 20 deg/s, 45 deg/s while a commanded turn runs (the recorder's
 * mvState: a YAW in RUN, PAUSED or SETTLE). */
static void testYawGate(void)
{
  const float d2r = 0.0174533f, dt = 0.067f;
  CHECK(camHoldYawOk(19.9f * d2r * dt, dt, false) && !camHoldYawOk(20.1f * d2r * dt, dt, false) &&
        !camHoldYawOk(-20.1f * d2r * dt, dt, false), "no turn: 20 deg/s (0.35 rad/s) as before");
  CHECK(camHoldYawOk(30.0f * d2r * dt, dt, true) && camHoldYawOk(-44.9f * d2r * dt, dt, true) &&
        !camHoldYawOk(45.1f * d2r * dt, dt, true), "a commanded turn: up to 45 deg/s");
  CHECK(!camHoldYawOk(0.0f, 0.0f, true) && !camHoldYawOk(NAN, dt, true) && !camHoldYawOk(0.0f, -dt, false),
        "dt <= 0 or a NaN angle: no view");
  const uint8_t yaw = (uint8_t)(AUTO_MVK_YAW << 4), mv = (uint8_t)(AUTO_MVK_MOVE << 4);
  CHECK(camHoldTurnActive(yaw | AUTO_MV_RUN) && camHoldTurnActive(yaw | AUTO_MV_PAUSED) &&
        camHoldTurnActive(yaw | AUTO_MV_SETTLE), "a YAW running, paused or settling (dwell, bob): a turn");
  CHECK(!camHoldTurnActive(yaw | AUTO_MV_DONE) && !camHoldTurnActive(yaw | AUTO_MV_STOPPED) &&
        !camHoldTurnActive(mv | AUTO_MV_RUN) && !camHoldTurnActive(0), "a YAW ended, a MOVE, manual flight: none");
}

/* No forward hold (autonomy TAKEOFF, docs/autonomy-a2b.md 19.5): the forward axis neither filters nor integrates,
 * its output is KI iFwd + ffPitch, the lateral axis is the same as with it on; when it ends the forward axis starts
 * from fwdF 0 and the integral it had. */
static void testFwdOff(void)
{
  camHoldState_t on, off, off2;
  camHoldReset(&on);
  camHoldReset(&off);
  camHoldReset(&off2);
  uint32_t r = 12345u;
  bool rollSame = true, pitchZero = true, fwdUntouched = true;
  for (int k = 0; k < 400; k++) {
    r = r * 1103515245u + 12345u;
    const float nz = (float)((r >> 8) & 0xFFFF) / 65536.0f - 0.5f;
    camHoldInput_t in = {
      .enabled = true, .sticksCentred = true, .airborne = true, .viewOk = (k % 37) != 5, .dt = 0.0675f,
      .wFwd = 0.20f + 0.1f * nz, .wLat = -0.05f + 0.08f * nz,
    };
    camHoldStep(&on, &in);
    in.fwdOff = true;
    camHoldStep(&off, &in);
    in.wFwd = -0.4f;   /* a different forward measurement: no effect while off */
    camHoldStep(&off2, &in);
    rollSame = rollSame && memcmp(&on.roll, &off.roll, sizeof(float)) == 0 && off.latF == on.latF &&
               off.iLat == on.iLat && off.engaged == on.engaged && off.coasting == on.coasting &&
               memcmp(&off.roll, &off2.roll, sizeof(float)) == 0;
    pitchZero = pitchZero && off.pitch == 0.0f && off2.pitch == 0.0f && off.fwdF == 0.0f;
    fwdUntouched = fwdUntouched && off.iFwd == 0.0f && off.pFwd == 0.0f && off.lostFwd == 0.0f;
  }
  CHECK(rollSame, "fwdOff: the lateral axis, engaged and coasting identical");
  CHECK(pitchZero && fwdUntouched, "fwdOff from the start: pitch 0, fwdF 0, iFwd / pFwd untouched");
  CHECK(on.pitch > 0.5f, "with the forward axis on the same climb leans %.2f deg nose-up", on.pitch);

  /* with a learned integral and a feedforward: the output is KI iFwd + ffPitch, slewed */
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = { .enabled = true, .sticksCentred = true, .airborne = true, .viewOk = true, .dt = 0.0675f,
                        .wFwd = 0.1f };
  for (int k = 0; k < 40; k++) {
    camHoldStep(&st, &in);
  }
  const float i0 = st.iFwd;
  in.fwdOff = true;
  in.ffPitch = 0.3f;
  for (int k = 0; k < 80; k++) {
    camHoldStep(&st, &in);
  }
  CHECK(st.iFwd == i0 && fabsf(st.pitch - (CAM_HOLD_KI * i0 + 0.3f)) < 1e-5f,
        "fwdOff: pitch %.4f = KI iFwd + ff %.4f, the integral kept", st.pitch, CAM_HOLD_KI * i0 + 0.3f);
  /* back on: from fwdF 0 */
  in.fwdOff = false;
  in.ffPitch = 0.0f;
  in.wFwd = 0.0f;
  camHoldStep(&st, &in);
  CHECK(st.fwdF == 0.0f && st.iFwd == i0, "back on with no motion: fwdF 0, the integral %.4f", st.iFwd);
}


/* ---- the gain scale with the scene distance (docs/autonomy-a2b.md 20) ----------- */

/* A long deterministic sequence with every input the hold has (references, snaps, feedforward, fwdOff, dropouts,
 * landings) at one gain scale: the full state after every step. */
static void scaleRun(float scale, camHoldState_t *trace, int n)
{
  camHoldState_t st;
  camHoldReset(&st);
  holdIdRng = 0x2468ACEu;
  uint32_t epoch = 0, snap = 0;
  float ref = 0.0f;
  for (int k = 0; k < n; k++) {
    camHoldInput_t in = base();
    in.dt = 0.06f + 0.02f * holdIdUni();
    in.wFwd = 0.3f * (holdIdUni() - 0.5f);
    in.wLat = 0.3f * (holdIdUni() - 0.5f);
    in.viewOk = holdIdUni() > 0.1f;
    in.airborne = holdIdUni() > 0.003f;
    in.sticksCentred = holdIdUni() > 0.003f;
    in.fwdOff = (k / 700) % 3 == 1;
    if (holdIdUni() < 0.01f) {
      epoch++;
      ref = 0.0f;
      if (holdIdUni() < 0.5f) {
        snap = epoch;
      }
    }
    ref += 0.004f;
    in.refEpoch = epoch;
    in.refSnapEpoch = snap;
    in.refLat = epoch ? ref : 0.0f;
    in.refFwd = epoch ? -0.5f * ref : 0.0f;
    in.rateLat = epoch && (k % 50) < 30 ? 0.05f : 0.0f;
    in.ffRoll = epoch && (k % 50) < 5 ? 0.3f : 0.0f;
    in.ffPitch = epoch && (k % 70) < 5 ? -4.0f : 0.0f;
    if (holdIdUni() < 0.01f) {
      in.dt = 0.5f;
    }
    in.gainScale = scale;
    camHoldStep(&st, &in);
    trace[k] = st;
  }
}

/* The outputs and every state field but the applied scale itself. */
static bool sameState(const camHoldState_t *a, const camHoldState_t *b)
{
  camHoldState_t x = *a, y = *b;
  x.gainScale = y.gainScale = 0.0f;
  return memcmp(&x, &y, sizeof(x)) == 0;
}

#define SCALE_STEPS 20000
static camHoldState_t scaleTraceA[SCALE_STEPS], scaleTraceB[SCALE_STEPS];

/* Scale 0 (the default), 1, NaN, -1, +-inf and 0.5 all fly the hold before it: bit-identical states over every
 * regime with references; the identity hash (no references) is the 8f51285 one (testIdentity). */
static void testGainScaleIdentity(void)
{
  scaleRun(0.0f, scaleTraceA, SCALE_STEPS);
  static const float scales[] = {1.0f, NAN, -1.0f, INFINITY, -INFINITY, 0.5f, 1.0f - 1e-7f};
  for (unsigned j = 0; j < sizeof(scales) / sizeof(scales[0]); j++) {
    scaleRun(scales[j], scaleTraceB, SCALE_STEPS);
    int diff = 0;
    for (int k = 0; k < SCALE_STEPS; k++) {
      diff += !sameState(&scaleTraceA[k], &scaleTraceB[k]);
    }
    CHECK(diff == 0, "gain scale %g: bit-identical to scale 0 (%d of %d steps differ)", scales[j], diff, SCALE_STEPS);
  }
  /* above the limit: the same as CAM_HOLD_GAIN_SCALE_MAX */
  scaleRun(CAM_HOLD_GAIN_SCALE_MAX, scaleTraceA, SCALE_STEPS);
  scaleRun(7.0f, scaleTraceB, SCALE_STEPS);
  int diff = 0;
  for (int k = 0; k < SCALE_STEPS; k++) {
    diff += !sameState(&scaleTraceA[k], &scaleTraceB[k]);
  }
  CHECK(diff == 0 && scaleTraceB[SCALE_STEPS - 1].gainScale == CAM_HOLD_GAIN_SCALE_MAX,
        "gain scale 7 is clamped to %.1f (%d steps differ)", CAM_HOLD_GAIN_SCALE_MAX, diff);
}

/* Scale 2.5: P and I x 2.5 below the limits; the integral's contribution at most 1.5 deg; the output at most
 * 2.5 deg; the slew 6 deg/s. */
static void testGainScaleGains(void)
{
  camHoldState_t a, b;
  camHoldReset(&a);
  camHoldReset(&b);
  camHoldInput_t in = base();
  run(&a, in, 1.2f);
  run(&b, in, 1.2f);
  float worst = 0.0f;
  for (int k = 0; k < 40; k++) {   /* a small drift: below every limit and the slew */
    in.wLat = 0.02f + 0.01f * sinf(0.3f * (float)k);
    in.wFwd = -0.015f;
    in.gainScale = 1.0f;
    camHoldStep(&a, &in);
    in.gainScale = 2.5f;
    camHoldStep(&b, &in);
    worst = fmaxf(worst, fmaxf(fabsf(b.roll - 2.5f * a.roll), fabsf(b.pitch - 2.5f * a.pitch)));
  }
  CHECK(worst < 1e-5f && fabsf(a.roll) > 0.1f, "scale 2.5: the output 2.5 x scale 1's below the limits (worst "
        "%.2g, roll %.3f vs %.3f)", worst, b.roll, a.roll);
  CHECK(b.iLat == a.iLat && b.latF == a.latF, "scale 2.5: the same integral and filter (the gains scale the output)");

  /* a drift the integral saturates on: its contribution 1.5 deg, the output limit 2.5 */
  camHoldReset(&b);
  in = base();
  in.gainScale = 2.5f;
  run(&b, in, 1.2f);
  in.wLat = 0.5f;
  run(&b, in, 5.0f);
  in.wLat = 0.0f;
  run(&b, in, 10.0f);   /* latF decays: the output is the integral's part */
  CHECK(fabsf(b.iLat - CAM_HOLD_I_LIMIT_DEG / (CAM_HOLD_KI * 2.5f)) < 1e-6f &&
        fabsf(b.roll + CAM_HOLD_I_LIMIT_DEG) < 1e-4f, "scale 2.5: the integral clamped at %.3f, its contribution "
        "%.3f deg (1.5)", b.iLat, -b.roll);
  camHoldReset(&b);
  in = base();
  in.gainScale = 2.5f;
  run(&b, in, 1.2f);
  in.wLat = 2.0f;
  in.wFwd = -2.0f;
  float maxStep = 0.0f, maxOut = 0.0f, prevR = b.roll, prevP = b.pitch;
  for (int k = 0; k < 60; k++) {
    camHoldStep(&b, &in);
    maxStep = fmaxf(maxStep, fmaxf(fabsf(b.roll - prevR), fabsf(b.pitch - prevP)));
    maxOut = fmaxf(maxOut, fmaxf(fabsf(b.roll), fabsf(b.pitch)));
    prevR = b.roll;
    prevP = b.pitch;
  }
  CHECK(maxOut <= CAM_HOLD_LIMIT_DEG + 1e-6f && fabsf(maxOut - CAM_HOLD_LIMIT_DEG) < 1e-6f,
        "scale 2.5: the output limit %.2f deg (%.4f)", CAM_HOLD_LIMIT_DEG, maxOut);
  CHECK(maxStep <= CAM_HOLD_SLEW_DEG_S * DT + 1e-5f, "scale 2.5: the slew %.2f deg per frame (%.4f)",
        CAM_HOLD_SLEW_DEG_S * DT, maxStep);
}

/* A scale change in a hover keeps the learned lean KI s i continuous (latF 0), engaged and coasting, both ways;
 * a snap after the scale changed in its epoch restores the lean of the epoch start. */
static void testGainScaleContinuity(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.wLat = -0.05f;
  in.wFwd = 0.04f;
  run(&st, in, 3.0f);
  in.wLat = 0.0f;
  in.wFwd = 0.0f;
  run(&st, in, 12.0f);   /* the filters decay: latF, fwdF ~0, the output is the integrals' lean */
  CHECK(fabsf(st.latF) < 1e-12f && fabsf(st.roll) > 0.2f, "learned lean %.3f deg, latF %g", st.roll, st.latF);
  static const float seq[] = {2.5f, 1.0f, 1.7f, 3.0f, 1.2f};
  float worst = 0.0f;
  for (unsigned j = 0; j < sizeof(seq) / sizeof(seq[0]); j++) {
    for (int coast = 0; coast < 2; coast++) {
      in.viewOk = coast == 0;
      const float r0 = st.roll, p0 = st.pitch;
      in.gainScale = seq[j];
      camHoldStep(&st, &in);
      worst = fmaxf(worst, fmaxf(fabsf(st.roll - r0), fabsf(st.pitch - p0)));
      run(&st, in, 0.5f);
    }
  }
  in.viewOk = true;
  CHECK(worst <= 1e-6f, "scale changes 1 -> 2.5 -> 1 -> 1.7 -> 3 -> 1.2, engaged and coasting: |output change| "
        "%.2g deg (<= 1e-6)", worst);

  /* a snap: the epoch begins at scale 2.5 with a learned lean, the scale goes to 1 during it (a YAW forgets the
   * latched G), the snap at its end restores the lean the epoch began with */
  camHoldReset(&st);
  in = base();
  in.gainScale = 2.5f;
  run(&st, in, 1.2f);
  in.wLat = -0.04f;
  run(&st, in, 3.0f);
  in.wLat = 0.0f;
  run(&st, in, 12.0f);
  const float lean0 = -CAM_HOLD_KI * 2.5f * st.iLat;
  in.refEpoch = 1;
  float sc = 2.5f;
  for (int k = 0; k < 40; k++) {   /* the reference moves and the drone with it; the scale slews to 1 */
    in.refLat = 0.004f * (float)k;
    in.wLat = 0.004f / DT * ((k % 3) ? 1.2f : 0.6f);
    sc = fmaxf(1.0f, sc - 2.0f * DT);
    in.gainScale = sc;
    camHoldStep(&st, &in);
  }
  in.wLat = 0.0f;
  in.refEpoch = 2;
  in.refSnapEpoch = 2;
  in.refLat = 0.0f;
  camHoldStep(&st, &in);
  const float lean1 = -CAM_HOLD_KI * 1.0f * st.iLat;
  CHECK(fabsf(lean1 - lean0) < 1e-5f && sc == 1.0f, "snap after the scale went 2.5 -> 1 in its epoch: the lean "
        "%.4f deg restored (%.4f at the epoch start)", lean1, lean0);
}

/* The gain scale's target from the recorder fields (camHoldGainTarget) and its slew. */
static void testGainTarget(void)
{
  const uint8_t trk = 0x07, untrk = 0x03;   /* airborne, healthy, tracked / not */
  const uint8_t moveRun = (uint8_t)(AUTO_MV_RUN | (AUTO_MVK_MOVE << 4)), moveDone = (uint8_t)(AUTO_MV_DONE |
                          (AUTO_MVK_MOVE << 4)), yawRun = (uint8_t)(AUTO_MV_RUN | (AUTO_MVK_YAW << 4)),
                yawSettle = (uint8_t)(AUTO_MV_SETTLE | (AUTO_MVK_YAW << 4)), yawDone = (uint8_t)(AUTO_MV_DONE |
                          (AUTO_MVK_YAW << 4));
  const float km = CAM_HOLD_GAIN_KMAX_DEFAULT;
  camHoldGainLatch_t l;
  camHoldGainReset(&l);
  CHECK(camHoldGainTarget(&l, 0, 0, trk, 320, km) == 1.0f && l.g == 0.0f, "IDLE (a manual flight): 1, nothing "
        "latched");
  CHECK(camHoldGainTarget(&l, 1, 0, trk, 320, km) == 1.0f && l.g == 0.0f, "ARMED: 1");
  CHECK(camHoldGainTarget(&l, 2, 0, untrk, 800, km) == 1.0f && l.g == 0.0f, "TAKEOFF, G not tracked (the default "
        "80): 1");
  CHECK(camHoldGainTarget(&l, 2, 0, 0x14, 185, km) == 1.0f && l.g == 0.0f, "TAKEOFF on the ground: the previous "
        "flight's tracked G 18.5 (cvFlags 0x14, not airborne) ignored");
  CHECK(camHoldGainTarget(&l, 2, 0, trk, 320, km) == 2.5f && l.g == 32.0f, "TAKEOFF, tracked G 32: 2.5");
  CHECK(fabsf(camHoldGainTarget(&l, 3, 0, trk, 400, km) - 2.0f) < 1e-6f, "HOVER, tracked G 40: 2.0");
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 1000, km) == 1.0f, "HOVER, tracked G 100 (D 1.0 m): 1");
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 100, km) == 1.0f && l.g == 100.0f, "G 10 px/m (out of range) ignored: "
        "the latched G 100 stays");
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 5001, km) == 1.0f && l.g == 100.0f, "G 500.1 ignored");
  CHECK(fabsf(camHoldGainTarget(&l, 3, 0, trk, 500, km) - 1.6f) < 1e-6f, "G 50: 1.6");
  CHECK(fabsf(camHoldGainTarget(&l, 3, 0, untrk, 300, km) - 1.6f) < 1e-6f, "untracked: the latched G 50 kept");
  CHECK(camHoldGainTarget(&l, 4, 0, untrk, 0, km) == 1.0f && l.g == 50.0f && !l.translated, "LAND: 1 (the "
        "descent's floor parallax), the latched G 50 kept");
  CHECK(camHoldGainTarget(&l, 4, 0, trk, 320, km) == 1.0f && l.g == 32.0f, "LAND, tracked G 32: latched, still 1");
  CHECK(camHoldGainTarget(&l, 4, 0, trk, 320, 3.0f) == 1.0f, "LAND at kmax 3: 1");
  CHECK(camHoldGainTarget(&l, 5, 0, trk, 500, km) == 1.0f && l.g == 0.0f, "KILLED: cleared");

  /* a MOVE: 1 until the next ARM, whatever is tracked after it, a YAW included */
  camHoldGainReset(&l);
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 320, km) == 2.5f, "HOVER G 32: 2.5");
  CHECK(camHoldGainTarget(&l, 3, moveRun, trk, 320, km) == 1.0f && l.translated, "a MOVE running: 1");
  CHECK(camHoldGainTarget(&l, 3, moveDone, trk, 320, km) == 1.0f, "the MOVE done: still 1");
  CHECK(camHoldGainTarget(&l, 3, yawDone, trk, 320, km) == 1.0f, "a later YAW done, G tracked: still 1");
  CHECK(camHoldGainTarget(&l, 4, 0, trk, 320, km) == 1.0f, "LAND after it: 1");
  CHECK(camHoldGainTarget(&l, 1, 0, trk, 320, km) == 1.0f && !l.translated, "ARM: cleared");
  CHECK(camHoldGainTarget(&l, 2, 0, trk, 320, km) == 2.5f, "the next sortie's TAKEOFF: 2.5 again");

  /* a YAW: 1 from its start until a fresh tracked G after it */
  CHECK(camHoldGainTarget(&l, 3, yawRun, trk, 320, km) == 1.0f && l.g == 0.0f, "a YAW running: 1, G forgotten");
  CHECK(camHoldGainTarget(&l, 3, yawSettle, trk, 320, km) == 1.0f && l.g == 0.0f, "its settle (the re-track bob): 1");
  CHECK(camHoldGainTarget(&l, 3, yawDone, untrk, 320, km) == 1.0f, "done, the tracker not yet back: 1");
  CHECK(fabsf(camHoldGainTarget(&l, 3, yawDone, trk, 400, km) - 2.0f) < 1e-6f, "a fresh tracked G 40: 2.0");

  /* kmax */
  camHoldGainReset(&l);
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 200, 1.0f) == 1.0f && l.g == 20.0f, "kmax 1 (off): 1");
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 200, NAN) == 1.0f, "kmax NaN: 1");
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 200, 0.5f) == 1.0f, "kmax 0.5: 1");
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 200, 2.5f) == 2.5f, "G 20 (D 5 m), kmax 2.5: 2.5");
  CHECK(camHoldGainTarget(&l, 3, 0, trk, 200, 9.0f) == CAM_HOLD_GAIN_SCALE_MAX, "kmax 9: clamped to %.1f",
        CAM_HOLD_GAIN_SCALE_MAX);

  /* the slew */
  const float r2 = CAM_HOLD_GAIN_SLEW_PER_S;
  float s = 1.0f;
  int n = 0;
  while (s < 2.5f && n < 100) {
    s = camHoldGainSlew(s, 2.5f, 0.0675f, r2);
    n++;
  }
  CHECK(n == 12 && s == 2.5f, "slew 1 -> 2.5 at 2 /s over 67.5 ms frames: %d frames (0.81 s)", n);
  CHECK(camHoldGainSlew(2.5f, 1.0f, 1.0f, r2) == 1.9f, "a frame gap counts at most %.1f s", CAM_HOLD_MAX_DT_S);
  CHECK(camHoldGainSlew(2.0f, 1.0f, NAN, r2) == 2.0f && camHoldGainSlew(2.0f, 1.0f, -0.1f, r2) == 2.0f &&
        camHoldGainSlew(2.0f, 1.0f, 0.0f, r2) == 2.0f, "dt NaN, < 0, 0: no move");
  CHECK(camHoldGainSlew(NAN, 2.5f, 0.05f, r2) == 1.1f && camHoldGainSlew(1.5f, NAN, 0.05f, r2) == 1.4f,
        "a NaN scale restarts from 1, a NaN target is 1");
  CHECK(camHoldGainSlew(2.5f, 1.0f, 0.1f, NAN) == 2.3f && camHoldGainSlew(2.5f, 1.0f, 0.1f, 0.0f) == 2.3f &&
        camHoldGainSlew(2.5f, 1.0f, 0.1f, -1.0f) == 2.3f && camHoldGainSlew(2.5f, 1.0f, 0.1f, INFINITY) == 2.3f,
        "a rate NaN, 0, < 0 or inf: 2 /s");
}

/* The rate s moves at (docs/autonomy-a2b.md 21.3): 0.75 /s once a MOVE ran, in HOVER; 2 /s before it (unchanged),
 * in TAKEOFF and in LAND (to 1, the 20.8 creep fix). Frames of 67.5 ms through camHoldGainTarget, camHoldGainRate
 * and camHoldGainSlew in camera_flow.c's order. */
static float gainRun(camHoldGainLatch_t *l, float s, uint8_t autoState, uint8_t mvState, uint16_t g, int frames,
                     int *toOne)
{
  *toOne = -1;
  for (int i = 0; i < frames; i++) {
    const float tgt = camHoldGainTarget(l, autoState, mvState, CAM_HOLD_GAIN_CVF_LATCH, g, 2.5f);
    s = camHoldGainSlew(s, tgt, 0.0675f, camHoldGainRate(l, autoState));
    if (*toOne < 0 && s == 1.0f) {
      *toOne = i + 1;
    }
  }
  return s;
}

static void testGainRate(void)
{
  const uint8_t moveRun = (uint8_t)(AUTO_MV_RUN | AUTO_MVK_MOVE << 4), moveDone = (uint8_t)(AUTO_MV_DONE |
                                                                                            AUTO_MVK_MOVE << 4);
  camHoldGainLatch_t l;
  camHoldGainReset(&l);
  CHECK(CAM_HOLD_GAIN_SLEW_MOVE_PER_S == 0.75f && CAM_HOLD_GAIN_SLEW_PER_S == 2.0f, "0.75 /s in a move, 2 /s else");
  CHECK(camHoldGainRate(&l, AUTO_IDLE) == 2.0f && camHoldGainRate(&l, AUTO_TAKEOFF) == 2.0f &&
        camHoldGainRate(&l, AUTO_HOVER) == 2.0f && camHoldGainRate(&l, AUTO_LAND) == 2.0f, "nothing translated: 2 /s");
  /* before any move: the hover's rise to 2.5 as before (12 frames) */
  int one;
  float s = gainRun(&l, 1.0f, AUTO_HOVER, 0, 320, 12, &one);
  CHECK(s == 2.5f && !l.translated, "HOVER before a MOVE: 1 -> 2.5 in 12 frames (2 /s, unchanged)");
  /* a MOVE starts in HOVER: s falls at 0.75 /s, 2.5 -> 1 in 2.0 s (30 frames of 67.5 ms: 2.025 s) */
  s = gainRun(&l, s, AUTO_HOVER, moveRun, 320, 40, &one);
  CHECK(l.translated && camHoldGainRate(&l, AUTO_HOVER) == 0.75f && s == 1.0f && one >= 29 && one <= 31,
        "a MOVE in HOVER: 2.5 -> 1 at 0.75 /s in %d frames (%.3f s; 2.0 s +-1 frame)", one, one * 0.0675f);
  camHoldGainLatch_t l2;
  camHoldGainReset(&l2);
  float s2 = gainRun(&l2, 1.0f, AUTO_HOVER, 0, 320, 12, &one);
  s2 = gainRun(&l2, s2, AUTO_HOVER, moveRun, 320, 10, &one);
  CHECK(fabsf(s2 - (2.5f - 0.75f * 0.675f)) < 1e-4f, "10 frames into the MOVE: s %.4f (2.5 - 0.75 x 0.675 s)", s2);
  /* LAND after it: 2 /s to 1 (the latch translated, LAND is not HOVER) */
  CHECK(camHoldGainRate(&l2, AUTO_LAND) == 2.0f, "translated, LAND: 2 /s");
  s2 = gainRun(&l2, s2, AUTO_LAND, moveDone, 320, 12, &one);
  CHECK(s2 == 1.0f && one == 8, "LAND entered at s %.2f: 1 in %d frames at 2 /s (0.99 / 0.135)", 2.5f - 0.75f * 0.675f, one);
  /* a LAND without a MOVE: 2.5 -> 1 at 2 /s (12 frames), as before */
  camHoldGainReset(&l2);
  s2 = gainRun(&l2, 1.0f, AUTO_HOVER, 0, 320, 12, &one);
  s2 = gainRun(&l2, s2, AUTO_LAND, 0, 320, 20, &one);
  CHECK(s2 == 1.0f && one == 12, "LAND without a MOVE: 2.5 -> 1 in %d frames (2 /s)", one);
  /* ARM clears the latch: the next sortie rises at 2 /s again */
  (void)camHoldGainTarget(&l, AUTO_ARMED, 0, 0, 0, 2.5f);
  CHECK(!l.translated && camHoldGainRate(&l, AUTO_HOVER) == 2.0f, "ARM: cleared, 2 /s");
}

/* During a MOVE toward the scene (docs/autonomy-a2b.md 21.3, camHoldGainMove): once the camera sees it go, the gain
 * the reference's scene distance needs (D_low e^(-refFwd / 0.6) / 1.25 m), at most the move's starting scale; the
 * plain target otherwise (not yet confirmed, a frozen view, sideways / back moves, LAND, kmax 1, no G). */
static void testGainMove(void)
{
  const uint8_t run = (uint8_t)(AUTO_MV_RUN | AUTO_MVK_MOVE << 4), settle = (uint8_t)(AUTO_MV_SETTLE | AUTO_MVK_MOVE << 4),
                done = (uint8_t)(AUTO_MV_DONE | AUTO_MVK_MOVE << 4), yawRun = (uint8_t)(AUTO_MV_RUN | AUTO_MVK_YAW << 4);
  const uint8_t trk = CAM_HOLD_GAIN_CVF_LATCH;
  camHoldGainLatch_t l;
  camHoldGainReset(&l);
  CHECK(camHoldGainTarget(&l, AUTO_HOVER, 0, trk, 200, 2.5f) == 2.5f, "HOVER, G 20 (D 5 m): 2.5");
  /* the move starts at s 2.5, D_low 4.7 m: not along yet (refFwd 0.005): the plain target 1 */
  float t = camHoldGainTarget(&l, AUTO_HOVER, run, trk, 200, 2.5f);
  t = camHoldGainMove(&l, t, 2.5f, AUTO_HOVER, run, 4.7f, 0.005f, 0.0f);
  CHECK(t == 1.0f && l.sMove == 2.5f && !l.confirmed && !l.untracked && camHoldGainRate(&l, AUTO_HOVER) == 0.75f,
        "a MOVE starts at s 2.5: the ceiling latched, not along yet: the plain 1 at 0.75 /s");
  /* along, the camera sees it (measured 0.04 of 0.05): D_ref 4.7 e^(-0.05 / 0.6) = 4.32 m, 3.5 > 2.5: 2.5 */
  t = camHoldGainMove(&l, 1.0f, 2.45f, AUTO_HOVER, run, 4.7f, 0.05f, 0.04f);
  CHECK(t == 2.5f && l.confirmed && !l.untracked, "confirmed: the starting 2.5 (the scene would allow %.2f)",
        4.7f * expf(-0.05f / 0.6f) * 80.0f / 99.79f);
  /* nearer: refFwd 0.6 ln(4.7 / 2.0): D_ref 2.0 m: 1.60 */
  const float r2 = 0.6f * logf(4.7f / 2.0f);
  t = camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, run, 4.7f, r2, r2);
  CHECK(fabsf(t - 2.0f * 80.0f / 99.79f) < 1e-4f, "D_ref 2.0 m: %.3f (2.0 / 1.25)", t);
  t = camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, settle, 4.7f, 0.6f * logf(4.7f / 0.9f), 0.9f);
  CHECK(t == 1.0f, "settling 0.9 m from the scene: 1 (never under 1)");
  /* a frozen view: measured 0.01 of 0.2 (< 0.3 x): the plain target, 2 /s */
  t = camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, run, 4.7f, 0.2f, 0.01f);
  CHECK(t == 1.0f && !l.confirmed && l.untracked && camHoldGainRate(&l, AUTO_HOVER) == 2.0f,
        "the camera does not see it: the plain target at 2 /s");
  t = camHoldGainMove(&l, 1.0f, 2.0f, AUTO_HOVER, run, 4.7f, 0.25f, 0.2f);
  CHECK(fabsf(t - 4.7f * expf(-0.25f / 0.6f) * 80.0f / 99.79f) < 1e-4f && l.confirmed && !l.untracked &&
        camHoldGainRate(&l, AUTO_HOVER) == 0.75f, "seen again: back on, the scene's %.3f (D_ref 3.10 m)", t);
  /* the move done: the plain target; LAND: unchanged; a turn: unchanged */
  t = camHoldGainMove(&l, 1.0f, 2.0f, AUTO_HOVER, done, 4.7f, 0.3f, 0.3f);
  CHECK(t == 1.0f && !l.moveActive, "DONE: the plain target");
  t = camHoldGainMove(&l, 1.0f, 2.0f, AUTO_LAND, run, 4.7f, 0.3f, 0.3f);
  CHECK(t == 1.0f, "LAND: the plain target (1)");
  t = camHoldGainMove(&l, 1.0f, 2.0f, AUTO_HOVER, yawRun, 4.7f, 0.3f, 0.3f);
  CHECK(t == 1.0f, "a YAW: the plain target");
  /* a second move starts at s 1.4: its ceiling 1.4 */
  (void)camHoldGainMove(&l, 1.0f, 1.0f, AUTO_HOVER, done, 4.7f, 0.0f, 0.0f);
  t = camHoldGainMove(&l, 1.0f, 1.4f, AUTO_HOVER, run, 4.7f, 0.0f, 0.0f);
  t = camHoldGainMove(&l, 1.0f, 1.3f, AUTO_HOVER, run, 4.7f, 0.05f, 0.05f);
  CHECK(l.sMove == 1.4f && t == 1.4f, "the next move: its own start's 1.4 as the ceiling (%.2f)", t);
  /* sideways and back moves (dLowM 0): the plain target */
  (void)camHoldGainMove(&l, 1.0f, 1.0f, AUTO_HOVER, done, 0.0f, 0.0f, 0.0f);
  t = camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, run, 0.0f, 0.0f, 0.0f);
  CHECK(t == 1.0f, "a sideways or back move: the plain target");
  /* kmax 1 (s 1 at the start), no G latched, NaN */
  camHoldGainReset(&l);
  (void)camHoldGainTarget(&l, AUTO_HOVER, 0, trk, 200, 1.0f);
  t = camHoldGainMove(&l, 1.0f, 1.0f, AUTO_HOVER, run, 4.7f, 0.0f, 0.0f);
  t = camHoldGainMove(&l, 1.0f, 1.0f, AUTO_HOVER, run, 4.7f, 0.1f, 0.1f);
  CHECK(t == 1.0f, "kmax 1: 1");
  camHoldGainReset(&l);
  t = camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, run, 4.7f, 0.0f, 0.0f);
  t = camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, run, 4.7f, 0.1f, 0.1f);
  CHECK(t == 1.0f, "no G latched: the plain target");
  camHoldGainReset(&l);
  (void)camHoldGainTarget(&l, AUTO_HOVER, 0, trk, 200, 2.5f);
  t = camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, run, 4.7f, 0.0f, 0.0f);
  CHECK(camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, run, 4.7f, NAN, 0.1f) == 1.0f &&
        camHoldGainMove(&l, 1.0f, 2.5f, AUTO_HOVER, run, NAN, 0.1f, 0.1f) == 1.0f, "NaN reference or D: the plain "
        "target");
  /* the stability bound: approaching at 0.20 m/s from 4.7 m lowers D_ref / 1.25 by <= 0.16 /s < 0.75 /s */
  camHoldGainReset(&l);
  (void)camHoldGainTarget(&l, AUTO_HOVER, 0, trk, 200, 2.5f);
  float sNow = 2.5f, worstRate = 0.0f, prev = -1.0f;
  for (int i = 0; i <= 200; i++) {
    const float dRef = 4.7f - 0.20f * 0.1f * (float)i;   /* 0.1 s steps at 0.20 m/s to 0.7 m */
    const float ref = 0.6f * logf(4.7f / dRef);           /* the reference at k 0.6 (D_ref = dRef) */
    float tg = camHoldGainTarget(&l, AUTO_HOVER, run, trk, 200, 2.5f);
    tg = camHoldGainMove(&l, tg, sNow, AUTO_HOVER, run, 4.7f, ref, ref);
    sNow = camHoldGainSlew(sNow, tg, 0.1f, camHoldGainRate(&l, AUTO_HOVER));
    if (prev >= 0.0f) {
      worstRate = fmaxf(worstRate, (prev - tg) / 0.1f);
    }
    prev = tg;
  }
  CHECK(worstRate <= 0.16f + 1e-3f && sNow == 1.0f, "approaching at 0.20 m/s: the target falls <= %.3f /s (< 0.75): s "
        "follows it to 1", worstRate);
}

/* The hold at its limit (camera_hold.h camHoldLimitStep, docs/autonomy-a2b.md 27.4). */
static void testHoldLimit(void)
{
  camHoldLimit_t s;
  memset(&s, 0, sizeof(s));
  uint32_t t = 0xFFFFF000u;   /* across the u32 wrap */
  uint8_t b = 0;
  /* 15 Hz frames at +1.35 roll: the bit after 8 s, not before */
  for (int i = 0; i < 120; i++, t += 67u) {
    b = camHoldLimitStep(&s, 1.35f, 0.2f, true, t);
  }
  CHECK(b == 0, "+1.35 deg roll for %u ms: no bit (0x%02x)", 119u * 67u, b);
  b = camHoldLimitStep(&s, 1.35f, 0.2f, true, t);
  CHECK(b == CAM_HOLD_LIM_ROLL, "... at %u ms: ROLL (0x%02x)", 120u * 67u, b);
  t += 67u;
  b = camHoldLimitStep(&s, 1.21f, 0.2f, true, t);
  CHECK(b == CAM_HOLD_LIM_ROLL && s.run == CAM_HOLD_LIM_ROLL, "a frame at 1.21 (under 1.30, over the keep 1.20): the "
        "run stays open (0x%02x)", b);
  t += 67u;
  b = camHoldLimitStep(&s, 1.19f, 0.2f, true, t);
  CHECK(b == 0 && s.run == 0, "one frame under 1.20: the run breaks, the bit clears (0x%02x)", b);
  t += 67u;
  b = camHoldLimitStep(&s, 1.25f, 0.2f, true, t);
  CHECK(b == 0 && s.run == 0, "1.25 opens no run (it opens at 1.30) (0x%02x)", b);
  /* a run hovering just under 1.30 with frames over it: one run, the bit after 8 s from its start */
  memset(&s, 0, sizeof(s));
  for (int i = 0; i < 121; i++, t += 67u) {
    b = camHoldLimitStep(&s, (i % 3 == 0) ? 1.31f : 1.27f, 0.0f, true, t);
  }
  CHECK(b == CAM_HOLD_LIM_ROLL, "1.27-1.31 deg after one frame at 1.31: the bit after 8 s (0x%02x)", b);
  /* negative pitch, with a roll run that does not count while a motion runs */
  memset(&s, 0, sizeof(s));
  for (int i = 0; i < 125; i++, t += 67u) {
    b = camHoldLimitStep(&s, 1.6f, -1.45f, true, t);
  }
  CHECK(b == (CAM_HOLD_LIM_ROLL | CAM_HOLD_LIM_PITCH | CAM_HOLD_LIM_PITCH_NEG), "roll +1.6 and pitch -1.45 for 8.3 s: "
        "both, pitch negative (0x%02x)", b);
  b = camHoldLimitStep(&s, 1.6f, -1.45f, false, t);
  CHECK(b == 0 && s.run == 0, "not counting (a motion, LAND, the view lost): everything clears (0x%02x)", b);
  b = camHoldLimitStep(&s, NAN, -1.45f, true, t + 67u);
  CHECK(b == 0 && s.run == CAM_HOLD_LIM_PITCH, "NaN roll: not at the limit; pitch starts a new run (0x%02x)", b);
  CHECK(CAM_HOLD_LIMIT_WARN_DEG == 1.3f && CAM_HOLD_LIMIT_WARN_MS == 8000u && CAM_HOLD_LIMIT_WARN_DEG < CAM_HOLD_I_LIMIT_DEG &&
        CAM_HOLD_LIMIT_KEEP_DEG == 1.2f,
        "1.3 deg of the 1.5 deg integral range, 8 s, kept down to 1.2");
  /* the recorder's mvState: any motion running, paused or settling */
  CHECK(camHoldMotionActive((uint8_t)(AUTO_MV_RUN | (AUTO_MVK_MOVE << 4))) &&
        camHoldMotionActive((uint8_t)(AUTO_MV_SETTLE | (AUTO_MVK_YAW << 4))) &&
        camHoldMotionActive((uint8_t)(AUTO_MV_PAUSED | (AUTO_MVK_MOVE << 4))) &&
        !camHoldMotionActive((uint8_t)(AUTO_MV_DONE | (AUTO_MVK_MOVE << 4))) &&
        !camHoldMotionActive((uint8_t)(AUTO_MV_STOPPED | (AUTO_MVK_YAW << 4))) && !camHoldMotionActive(0),
        "camHoldMotionActive: RUN, PAUSED, SETTLE of any kind");
}

/* Fast moves (mvspeed > 100, 2026-10-16, docs/autonomy-a2b.md 31): the move's lean limit and slew. */
static void testFastMove(void)
{
  float lim, slewS;
  camHoldMoveLimits(100, &lim, &slewS);
  CHECK(lim == 0.0f && slewS == 0.0f, "camHoldMoveLimits(100): 0, 0 (the hold as before)");
  camHoldMoveLimits(150, &lim, &slewS);
  CHECK(fabsf(lim - 3.0f) < 1e-5f && fabsf(slewS - 9.0f) < 1e-5f, "150: 3.0 deg, 9 deg/s (%g %g)", lim, slewS);
  camHoldMoveLimits(200, &lim, &slewS);
  CHECK(fabsf(lim - 3.5f) < 1e-5f && fabsf(slewS - 12.0f) < 1e-5f, "200: 3.5 deg, 12 deg/s (%g %g)", lim, slewS);
  camHoldMoveLimits(300, &lim, &slewS);
  CHECK(fabsf(lim - 4.5f) < 1e-5f && fabsf(slewS - 12.0f) < 1e-5f, "300: 4.5 deg, 12 deg/s (%g %g)", lim, slewS);
  camHoldMoveLimits(999, &lim, &slewS);
  CHECK(fabsf(lim - 4.5f) < 1e-5f && fabsf(slewS - 12.0f) < 1e-5f, "above 300: as 300 (%g %g)", lim, slewS);

  /* moveLim 0: the same outputs as without the fields, a feedforward and a reference running (every frame) */
  {
    camHoldState_t a, b;
    camHoldReset(&a);
    camHoldReset(&b);
    camHoldInput_t in = base();
    bool same = true;
    for (int k = 0; k < 400; k++) {
      in.wFwd = 0.3f * sinf(0.05f * (float)k);
      in.wLat = -0.2f * cosf(0.03f * (float)k);
      in.viewOk = (k / 37) % 4 != 3;
      in.refEpoch = 1u + (uint32_t)(k / 120);
      in.refFwd = 0.002f * (float)(k % 120);
      in.rateFwd = 0.03f;
      in.ffPitch = (k % 120) < 30 ? -1.2f : 0.4f;
      in.ffRoll = 0.6f;
      camHoldInput_t z = in;
      z.moveLimDeg = 0.0f;
      z.moveSlewDegS = 0.0f;
      camHoldStep(&a, &in);
      camHoldStep(&b, &z);
      same = same && memcmp(&a, &b, sizeof(a)) == 0 && camHoldSlewOf(&b, &z) == CAM_HOLD_SLEW_DEG_S;
    }
    CHECK(same, "moveLimDeg 0 / moveSlewDegS 0: the state bit-identical, the slew 6 deg/s");
  }
  /* engaged, the feedback saturated (a forward drift), a feedforward of +1.0 deg: today 2.5, fast 3.5 */
  for (int fast = 0; fast < 2; fast++) {
    camHoldState_t st;
    camHoldReset(&st);
    camHoldInput_t in = base();
    run(&st, in, 1.2f);
    in.wFwd = 1.0f;     /* kp x 1.0 = 6 deg: the feedback saturates */
    in.refEpoch = 1u;
    in.ffPitch = 1.0f;
    if (fast) {
      in.moveLimDeg = 3.5f;
      in.moveSlewDegS = 12.0f;
    }
    run(&st, in, 2.0f);
    CHECK(fabsf(st.pitch - (fast ? 3.5f : 2.5f)) < 1e-4f, "%s: feedback at its 2.5 deg + 1.0 feedforward: pitch %.3f",
          fast ? "fast (lim 3.5)" : "today", st.pitch);
    /* the feedback alone stays clamped at 2.5: a feedforward of +0.4 gives 2.9 (not 3.5) */
    in.ffPitch = 0.4f;
    run(&st, in, 1.0f);
    CHECK(fabsf(st.pitch - (fast ? 2.9f : 2.5f)) < 1e-4f, "%s: the feedback clamped at 2.5 before the feedforward: "
          "pitch %.3f", fast ? "fast" : "today", st.pitch);
  }
  /* coasting with a feedforward (a stopped move's brake while the view is lost): fast applies it, today does not */
  for (int fast = 0; fast < 2; fast++) {
    camHoldState_t st;
    camHoldReset(&st);
    camHoldInput_t in = base();
    run(&st, in, 1.2f);
    in.wFwd = 0.1f;
    run(&st, in, 3.0f);   /* a learned forward integral */
    const float learned = CAM_HOLD_KI * st.iFwd;
    in.viewOk = false;
    in.wFwd = 0.0f;
    in.refEpoch = 2u;
    in.refSnapEpoch = 2u;
    in.ffPitch = 1.2f;
    if (fast) {
      in.moveLimDeg = 3.5f;
      in.moveSlewDegS = 12.0f;
    }
    camHoldStep(&st, &in);   /* the snap */
    const float learned2 = CAM_HOLD_KI * st.iFwd;
    run(&st, in, 2.0f);
    const float want = fast ? fminf(learned2 + 1.2f, 3.5f) : learned2;
    CHECK(st.coasting && fabsf(st.pitch - want) < 1e-4f, "%s coasting: pitch %.3f (the learned %.3f%s)",
          fast ? "fast" : "today", st.pitch, learned2, fast ? " + the 1.2 brake" : ", no feedforward");
    (void)learned;
  }
  /* the slew: 12 deg/s moves 0.96 deg per 0.08 s frame (6: 0.48) */
  {
    camHoldState_t st;
    camHoldReset(&st);
    camHoldInput_t in = base();
    run(&st, in, 1.2f);
    in.refEpoch = 1u;
    in.ffPitch = 3.0f;
    in.moveLimDeg = 3.5f;
    in.moveSlewDegS = 12.0f;
    const float p0 = st.pitch;
    camHoldStep(&st, &in);
    CHECK(fabsf(st.pitch - p0 - 12.0f * DT) < 1e-4f && camHoldSlewOf(&st, &in) == 12.0f,
          "fast slew: %.3f deg in one frame (12 deg/s), camHoldSlewOf 12", st.pitch - p0);
    in.ffPitch = 0.0f;
    in.rateFwd = 0.0f;
    camHoldInput_t q = in;
    CHECK(!camHoldFastActive(&st, &q) || st.rateFwdF != 0.0f, "no feedforward and no rate: not the fast path");
    in.moveLimDeg = 0.0f;
    CHECK(camHoldSlewOf(&st, &in) == CAM_HOLD_SLEW_DEG_S, "moveLimDeg 0: the slew 6 deg/s");
  }
}

/* ---- the move fixes (mvfix BRAKE, STILL, GAIN; the 300 % forensics of 2026-10-17, docs/autonomy-a2b.md 33) --------- */

/* A forward reference at rate w (its position integrated) with the measured rate m and a feedforward ffP / ffR, n
 * frames; scaled: ffBrakeScale. */
static void runRef(camHoldState_t *st, float w, float m, float wl, float ml, float ffP, float ffR, bool scaled, int n)
{
  camHoldInput_t in = base();
  float rf = st->epoch == 1 ? st->rFwd : 0.0f, rl = st->epoch == 1 ? st->rLat : 0.0f;
  in.refEpoch = 1;
  in.rateFwd = w;
  in.wFwd = m;
  in.rateLat = wl;
  in.wLat = ml;
  in.ffPitch = ffP;
  in.ffRoll = ffR;
  in.ffBrakeScale = scaled;
  for (int k = 0; k < n; k++) {
    rf += w * DT;
    rl += wl * DT;
    in.refFwd = rf;
    in.refLat = rl;
    camHoldStep(st, &in);
  }
}

static void testBrakeScale(void)
{
  /* the same frames with and without the scaling: the outputs differ by the braking lean's scaled part only */
  /* (30 frames tracking the reference, then 15 at the measured rate m: the progress stays >= 0.5 x the reference but
   * where the camera never saw the move, the frozen view) */
  const struct { float w, m, ffP, wl, ml, ffR; float wantP, wantR; bool frozen; const char *what; } c[] = {
    {0.10f, 0.05f, 1.2f, 0.0f, 0.0f, 0.0f, 0.6f, 0.0f, false, "forward at half the reference: the brake 1.2 -> 0.6 deg"},
    {0.10f, 0.20f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, false, "forward at twice the reference: the brake unscaled (down "
                                                              "only)"},
    {0.10f, -0.02f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, "sliding back: no brake"},
    {0.10f, 0.05f, -1.2f, 0.0f, 0.0f, 0.0f, -1.2f, 0.0f, false, "a speed-up lean (nose down forward): unscaled"},
    {-0.10f, -0.05f, -1.2f, 0.0f, 0.0f, 0.0f, -0.6f, 0.0f, false, "a back move's brake (nose down) at half: -0.6"},
    {0.0f, 0.0f, 0.0f, 0.08f, 0.02f, -1.0f, 0.0f, -0.25f, false, "right at a quarter: the brake (left down) -1.0 -> "
                                                                 "-0.25"},
    {0.0f, 0.0f, 0.0f, 0.08f, 0.02f, 1.0f, 0.0f, 1.0f, false, "a speed-up roll: unscaled"},
    {0.005f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, false, "a reference under 0.01 /s: unscaled"},
    {0.10f, 0.0f, 1.2f, 0.0f, 0.0f, 0.0f, 1.2f, 0.0f, true, "a frozen view (no progress measured): the full brake"},
  };
  for (unsigned i = 0; i < sizeof c / sizeof c[0]; i++) {
    camHoldState_t a, b;
    camHoldReset(&a);
    camHoldReset(&b);
    run(&a, base(), 1.2f);
    run(&b, base(), 1.2f);
    /* settle the filters at the rates with the feedforward off, then one frame with it on (the slew aside) */
    const float mt = c[i].frozen ? 0.0f : c[i].w, mlt = c[i].frozen ? 0.0f : c[i].wl;
    runRef(&a, c[i].w, mt, c[i].wl, mlt, 0.0f, 0.0f, false, 30);
    runRef(&b, c[i].w, mt, c[i].wl, mlt, 0.0f, 0.0f, true, 30);
    runRef(&a, c[i].w, c[i].m, c[i].wl, c[i].ml, 0.0f, 0.0f, false, 15);
    runRef(&b, c[i].w, c[i].m, c[i].wl, c[i].ml, 0.0f, 0.0f, true, 15);
    const float p0 = a.pitch, r0 = a.roll;
    camHoldState_t a1 = a, b1 = b;
    camHoldInput_t in = base();
    in.refEpoch = 1;
    in.rateFwd = c[i].w;
    in.wFwd = c[i].m;
    in.rateLat = c[i].wl;
    in.wLat = c[i].ml;
    in.refFwd = a.rFwd + c[i].w * DT;
    in.refLat = a.rLat + c[i].wl * DT;
    in.moveLimDeg = 4.5f;   /* (the fast path: the feedforward on top of the feedback, a 12 deg/s slew) */
    in.moveSlewDegS = 100.0f;
    in.ffPitch = c[i].ffP;
    in.ffRoll = c[i].ffR;
    camHoldStep(&a1, &in);
    in.ffBrakeScale = true;
    camHoldStep(&b1, &in);
    const float dP = b1.pitch - a1.pitch, dR = b1.roll - a1.roll;
    const float wantDP = c[i].wantP - c[i].ffP, wantDR = c[i].wantR - c[i].ffR;
    CHECK(fabsf(dP - wantDP) < 0.02f && fabsf(dR - wantDR) < 0.02f && a.pitch == b.pitch && a.roll == b.roll,
          "%s (pitch %+.3f -> %+.3f, roll %+.3f -> %+.3f; before it %+.3f / %+.3f)", c[i].what, a1.pitch, b1.pitch,
          a1.roll, b1.roll, p0, r0);
  }
  /* coasting: the brake unscaled (no measurement) */
  camHoldState_t a, b;
  camHoldReset(&a);
  run(&a, base(), 1.2f);
  runRef(&a, 0.10f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, false, 30);
  runRef(&a, 0.10f, 0.05f, 0.0f, 0.0f, 0.0f, 0.0f, false, 15);
  b = a;
  camHoldInput_t in = base();
  in.viewOk = false;
  in.refEpoch = 1;
  in.rateFwd = 0.10f;
  in.ffPitch = 1.2f;
  in.moveLimDeg = 4.5f;
  in.moveSlewDegS = 100.0f;
  camHoldStep(&a, &in);
  in.ffBrakeScale = true;
  camHoldStep(&b, &in);
  CHECK(a.coasting && a.pitch == b.pitch, "coasting: the brake as planned (%.3f, %.3f)", a.pitch, b.pitch);
}

static void testStill(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  CHECK(!camHoldMoving(&st, DT), "not engaged: not moving (no measurement)");
  camHoldInput_t in = base();
  run(&st, in, 1.2f);
  in.wFwd = 0.08f;   /* drifting at 0.08 /s (0.16 m/s at 2 m: flight 6's drift at move 2) */
  int movingN = 0, n = 0;
  for (int k = 0; k < 20; k++) {
    camHoldStep(&st, &in);
    movingN += camHoldMoving(&st, DT);
    n++;
  }
  CHECK(movingN == n, "a drift of 0.08 /s: moving on every frame (%d/%d)", movingN, n);
  in.wFwd = 0.0f;   /* stopped: the filtered rate decays under 0.05 /s in ~0.1 s, then 0.3 s more */
  float tStill = -1.0f;
  for (int k = 0; k < 40; k++) {
    camHoldStep(&st, &in);
    if (!camHoldMoving(&st, DT) && tStill < 0.0f) {
      tStill = (float)(k + 1) * DT;
    }
  }
  CHECK(tStill > 0.30f && tStill < 0.60f, "stopped: still after %.2f s (the filter, then 0.3 s)", tStill);
  in.wLat = -0.03f;
  for (int k = 0; k < 10; k++) {
    camHoldStep(&st, &in);
  }
  CHECK(!camHoldMoving(&st, DT), "a sideways wander of 0.03 /s (the plain hover's): still");
  in.wLat = -0.07f;
  for (int k = 0; k < 8; k++) {   /* (its filtered rate past 0.05 /s after ~5 frames) */
    camHoldStep(&st, &in);
  }
  CHECK(camHoldMoving(&st, DT), "a sideways drift of 0.07 /s: moving again");
  in.viewOk = false;
  camHoldStep(&st, &in);
  CHECK(st.coasting && !camHoldMoving(&st, DT) && st.stillS == 0.0f, "coasting: not moving (no measurement), the run "
        "restarts");
}

static void testGainMoveFix(void)
{
  const uint8_t run = (uint8_t)(AUTO_MV_RUN | AUTO_MVK_MOVE << 4), done = (uint8_t)(AUTO_MV_DONE | AUTO_MVK_MOVE << 4);
  const uint8_t trk = CAM_HOLD_GAIN_CVF_LATCH;
  camHoldGainLatch_t l;
  camHoldGainReset(&l);
  CHECK(!l.fromMeasured && l.graceS == 0.0f && l.moveS == 0.0f, "reset: as camHoldGainMove before (no fix)");
  (void)camHoldGainTarget(&l, AUTO_HOVER, 0, trk, 200, 2.5f);
  /* measured, 1 s of grace: the move starts at s 1.84 (flight 6: D_low 2.30 m) */
  camHoldGainMoveFix(&l, true, 1.0f, 0.07f);
  float t = camHoldGainTarget(&l, AUTO_HOVER, run, trk, 200, 2.5f);
  t = camHoldGainMove(&l, t, 1.84f, AUTO_HOVER, run, 2.30f, 0.0f, 0.0f);
  CHECK(fabsf(t - 1.84f) < 1e-5f && l.moveS == 0.0f, "the move's first frame: the measured D 2.30 m caps at its start's "
        "1.84 (%.3f), not the plain 1", t);
  /* 0.55 s in: the reference 0.0123, measured 0.0038 (< 0.3 x): in the grace, no untracked cut */
  for (int k = 0; k < 8; k++) {
    camHoldGainMoveFix(&l, true, 1.0f, 0.07f);
    t = camHoldGainMove(&l, 1.0f, 1.84f, AUTO_HOVER, run, 2.30f, 0.0123f, 0.0038f);
  }
  CHECK(!l.untracked && t > 1.80f && fabsf(l.moveS - 0.56f) < 1e-4f && camHoldGainRate(&l, AUTO_HOVER) == 0.75f,
        "0.56 s into the speed-up, measured 0.31 x the reference: no cut in the grace (%.3f, 0.75 /s)", t);
  for (int k = 0; k < 7; k++) {
    camHoldGainMoveFix(&l, true, 1.0f, 0.07f);
    t = camHoldGainMove(&l, 1.0f, 1.84f, AUTO_HOVER, run, 2.30f, 0.05f, 0.01f);
  }
  CHECK(l.untracked && t == 1.0f && camHoldGainRate(&l, AUTO_HOVER) == 2.0f, "after the grace a frozen view is cut as "
        "before (the plain 1 at 2 /s)");
  /* the drone lags its reference (flight 6, 15.9 s: reference 0.37, measured 0.28): the measured D */
  camHoldGainMoveFix(&l, true, 1.0f, 0.07f);
  t = camHoldGainMove(&l, 1.0f, 1.84f, AUTO_HOVER, run, 2.30f, 0.37f, 0.28f);
  const float want = 2.30f * expf(-0.28f / 0.6f) * 80.0f / 99.79f;
  CHECK(!l.untracked && fabsf(t - want) < 1e-4f, "lagging (0.28 of 0.37): the measured D's %.3f (the reference's would "
        "be %.3f)", t, fmaxf(2.30f * expf(-0.37f / 0.6f) * 80.0f / 99.79f, 1.0f));
  /* without the fix: as before */
  camHoldGainLatch_t o;
  camHoldGainReset(&o);
  (void)camHoldGainTarget(&o, AUTO_HOVER, 0, trk, 200, 2.5f);
  camHoldGainMoveFix(&o, false, 0.0f, 0.07f);
  float u = camHoldGainTarget(&o, AUTO_HOVER, run, trk, 200, 2.5f);
  u = camHoldGainMove(&o, u, 1.84f, AUTO_HOVER, run, 2.30f, 0.0f, 0.0f);
  CHECK(u == 1.0f, "without GAIN: the plain 1 before the move is seen, as before");
  camHoldGainMoveFix(&o, false, 0.0f, 0.07f);
  u = camHoldGainMove(&o, 1.0f, 1.84f, AUTO_HOVER, run, 2.30f, 0.0123f, 0.0030f);
  CHECK(o.untracked && u == 1.0f, "without GAIN: an early speed-up under 0.3 x cut as before");
  /* the move's clock restarts with the next move */
  camHoldGainMoveFix(&l, true, 1.0f, 0.07f);
  (void)camHoldGainMove(&l, 1.0f, 1.0f, AUTO_HOVER, done, 2.30f, 0.0f, 0.0f);
  camHoldGainMoveFix(&l, true, 1.0f, 0.07f);
  CHECK(l.moveS == 0.0f, "a new move: its clock from 0");
}

/* The lateral position gain during a MOVE (camHoldInput_t latGain, 2026-10-22 "the left slide", a port of a2b-int
 * cc4d7b5): 1 (0, NaN, 0.5) is the hold bit for bit; a steady lateral pull during a forward move (the plant pushed at a
 * constant acceleration, the reference still: the 10-18/22 moves slid 0.3-0.8 m left against a ~0.45 deg pull) leaves
 * an offset of pull / (KI x latGain): at 3 a third of it; a change of latGain never jumps the output (engaged or
 * coasting); the recorder's mvState picks a MOVE's frames (camHoldMoveActive). */
static void testLatGain(void)
{
  /* identity: a scripted sequence (a reference, a lost view, a snap) with latGain 0 / 1 / 0.5 / NaN against itself */
  camHoldState_t ref;
  float out[4][3];
  for (int q = 0; q < 4; q++) {
    camHoldState_t st;
    camHoldReset(&st);
    camHoldInput_t in = base();
    in.gainScale = 2.0f;
    in.latGain = q == 0 ? 0.0f : (q == 1 ? 1.0f : (q == 2 ? 0.5f : NAN));
    run(&st, in, 1.2f);
    in.refEpoch = 1;
    for (int k = 0; k < 60; k++) {
      in.refLat = 0.003f * (float)k;
      in.rateLat = 0.003f / DT;
      in.wLat = 0.003f / DT * ((k % 4) ? 0.7f : 1.4f) - 0.01f;
      in.wFwd = 0.02f * ((k % 5) ? 1.0f : -1.0f);
      in.viewOk = (k % 17) != 0;
      in.gainScale = 2.0f - 0.01f * (float)k;
      camHoldStep(&st, &in);
    }
    in.refEpoch = 2;
    in.refSnapEpoch = 2;
    in.refLat = 0.0f;
    in.rateLat = 0.0f;
    in.viewOk = true;
    run(&st, in, 2.0f);
    out[q][0] = st.roll;
    out[q][1] = st.pitch;
    out[q][2] = st.iLat;
    if (q == 1) {
      ref = st;
    }
  }
  CHECK(memcmp(out[0], out[1], sizeof(out[0])) == 0 && memcmp(out[2], out[1], sizeof(out[0])) == 0 &&
            memcmp(out[3], out[1], sizeof(out[0])) == 0 && ref.latGainScale == 1.0f,
        "latGain 0, 0.5 and NaN: bit-identical to 1 (roll %.4f pitch %.4f iLat %.5f; the lateral scale 1)",
        out[1][0], out[1][1], out[1][2]);

  /* a steady pull during a forward move: the plant gets -0.10 m/s^2 (0.58 deg of lean) from t 2 s for 7 s, the
   * lateral reference stays 0 (a pure forward move), the measurement is the plant's x / D one frame old */
  const float D = 3.3f, pull = -0.10f;
  float off[2], peak[2];
  for (int q = 0; q < 2; q++) {
    camHoldState_t st;
    camHoldReset(&st);
    plant1_t p = {0};
    camHoldInput_t in = base();
    in.gainScale = 1.0f;
    float prevW = 0.0f, t = 0.0f, worst = 0.0f;
    for (int k = 0; k < (int)(14.0f / DT); k++, t += DT) {
      const bool moving = t >= 2.0f && t < 9.0f;
      in.refEpoch = t >= 2.0f ? 1u : 0u;
      in.rateFwd = moving ? 0.05f : 0.0f;   /* a forward reference: the fast path's refActive */
      in.refFwd = in.refFwd + (moving ? 0.05f * DT : 0.0f);
      in.wFwd = in.rateFwd;                 /* the forward axis tracks exactly */
      in.latGain = (q == 1 && (moving || t < 11.5f)) ? 3.0f : 1.0f;   /* mvlatk 300 on the MOVE's frames */
      in.wLat = prevW;
      prevW = p.v / D;
      camHoldStep(&st, &in);
      for (int j = 0; j < 80; j++) {
        /* the pull as an extra lateral acceleration on the plant (a left push: -x) */
        p.lean += (st.roll - p.lean) * fminf(1.0f, DT / 80.0f / 0.1f);
        const float a = 9.81f * tanf(p.lean * 0.0174533f) - 0.35f * p.v + (moving ? pull : 0.0f);
        p.v += a * DT / 80.0f;
        p.x += p.v * DT / 80.0f;
      }
      if (moving) {
        worst = fmaxf(worst, fabsf(p.x));
      }
      if (t >= 9.0f - DT && t < 9.0f) {
        off[q] = p.x;
      }
    }
    peak[q] = worst;
  }
  CHECK(off[0] < -0.60f && off[0] > -1.20f, "latGain 1: a 0.10 m/s^2 pull for 7 s slides the drone %.2f m at D 3.3 m "
        "(the 10-18 slide: KI 2 deg per unit needs ~0.29 units = 0.95 m for 0.58 deg)", off[0]);
  CHECK(off[1] > -0.45f && off[1] < -0.20f && fabsf(off[1]) <= 0.6f * fabsf(off[0]) && peak[1] < peak[0],
        "latGain 3: the same pull slides it %.2f m (its steady offset 0.32 m, a third of latGain 1's 0.96, which the 7 s "
        "do not reach; <= 0.6 x), the worst excursion %.2f -> %.2f m", off[1], peak[0], peak[1]);

  /* continuity: latGain changes with a learned lean, engaged and coasting, never jump the output; with the startup
   * lateral scale (latGainScale) in force too */
  camHoldState_t st;
  camHoldReset(&st);
  camHoldInput_t in = base();
  in.gainScale = 1.8f;
  in.latGainScale = 1.5f;
  run(&st, in, 1.2f);
  in.wLat = -0.05f;
  in.wFwd = 0.03f;
  run(&st, in, 3.0f);
  in.wLat = 0.0f;
  in.wFwd = 0.0f;
  run(&st, in, 12.0f);
  CHECK(fabsf(st.roll) > 0.2f, "learned lean %.3f deg", st.roll);
  static const float seq[] = {3.0f, 1.0f, 2.2f, 4.0f, 1.0f, 9.0f};
  float worstJump = 0.0f;
  for (unsigned j = 0; j < sizeof(seq) / sizeof(seq[0]); j++) {
    for (int coast = 0; coast < 2; coast++) {
      in.viewOk = coast == 0;
      const float r0 = st.roll, p0 = st.pitch;
      in.latGain = seq[j];
      camHoldStep(&st, &in);
      worstJump = fmaxf(worstJump, fmaxf(fabsf(st.roll - r0), fabsf(st.pitch - p0)));
      run(&st, in, 0.5f);
    }
  }
  CHECK(worstJump <= 1e-6f && st.latGainScale == 1.5f * CAM_HOLD_LAT_GAIN_MAX,
        "latGain 1 -> 3 -> 1 -> 2.2 -> 4 -> 1 -> 9 (clamped 4), engaged and coasting, startup scale 1.5: |output change| "
        "%.2g deg (<= 1e-6), the lateral scale %.2f (1.5 x 4)", worstJump, st.latGainScale);

  /* the recorder's mvState: a MOVE running, paused or settling; not a YAW, not a finished or refused MOVE */
  CHECK(camHoldMoveActive((uint8_t)(AUTO_MV_RUN | (AUTO_MVK_MOVE << 4))) &&
            camHoldMoveActive((uint8_t)(AUTO_MV_PAUSED | (AUTO_MVK_MOVE << 4))) &&
            camHoldMoveActive((uint8_t)(AUTO_MV_SETTLE | (AUTO_MVK_MOVE << 4))) &&
            !camHoldMoveActive((uint8_t)(AUTO_MV_RUN | (AUTO_MVK_YAW << 4))) &&
            !camHoldMoveActive((uint8_t)(AUTO_MV_DONE | (AUTO_MVK_MOVE << 4))) && !camHoldMoveActive(0),
        "camHoldMoveActive: a MOVE's RUN / PAUSED / SETTLE only");
}

int main(void)
{
  testFastMove();
  testFwdOff();
  testHoldLimit();
  testYawGate();
  testDisabledAndSettle();
  testDirections();
  testLimitsAndSlew();
  testIntegralPullAndClamp();
  testOverrideAndLanding();
  testBadInput();
  testCoastOnViewLoss();
  testIdentity();
  testIdentityAfterMotion();
  testMovingRefLateral();
  testSnap();
  testRefWhileCoasting();
  testFeedforward();
  testSceneGateArming();
  testSceneGateThresholds();
  testSceneGateResidualLimit();
  testSceneGateHold();
  testGainScaleIdentity();
  testGainScaleGains();
  testGainScaleContinuity();
  testGainTarget();
  testGainRate();
  testGainMove();
  testBrakeScale();
  testStill();
  testGainMoveFix();
  testLatGain();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
