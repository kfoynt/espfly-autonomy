/*
 * Host unit tests for components/core/crazyflie/utils/src/auto_motion.c (the
 * interactive hover's motions: MOVE, YAW, STOP; auto_motion.h).
 *
 * Covered: the logarithm without libm; trapezoid profiles (area = the
 * distance, the speed and acceleration limits, triangles); the conversions of
 * a move into the hold's units (lateral R / D, forward ln(D0 / (D0 - F)),
 * a straight line for combined axes) and the rate caps that keep the
 * camera's view; the lagged reference and the feedforward; refusals; pause
 * and resume; the progress guard; the settle; STOP and the snap epochs; the
 * yaw ramp, its re-capture and the re-track bob; the D after a forward move
 * and after a turn; exactly 0 outputs without a motion. After the
 * verification of 591f176: a finished MOVE keeps its end in the idle outputs
 * (a LAND must not fly it back); which D converts what (forward dTrack,
 * lateral the larger, the floor's lower bound at k = AUTO_MV_K_LOW); TOO_WIDE;
 * the re-view bob after a lateral shift; the height floor with the
 * estimate's drift; the tighter progress guard and the settle that saw no
 * motion; a STOP never ends a re-track or re-view bob. User decision
 * 2026-10-13, evening: the guard in the settle (the RUN margins around the
 * target, the lateral one at least 0.25 m: an under-shoot within the margin
 * or 0.24 m of sideways wander, no stop; past the target or back from it by
 * the margin, 0.26 m sideways, 0.16 m toward or away from the scene in a
 * sideways move: PROGRESS), the SHORT end (short along the move, or off its
 * line by more than 0.15 m), the travel kept in the status.
 *
 * Fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31): at 100 the identity script (auto_motion_identity.h)
 * hashes to 7ab74b9's value with and without autoMotionSetSpeed(100); the limits of each step (100/150/200/250/300,
 * clamped outside 100..300); the forward image rate's true peak along every profile of a grid (<= W_FWD, reached where
 * the rule binds); the profile, turn, bob, settle and dwell times; VIEW_FAST (a fast reference stopped with its brake,
 * a slow one paused as before, 100 as before, a turn paused); the margins and the frozen-view reference; the profiles
 * of tools/tests/mvspeed_golden.json (the Python mirror's), rewritten with --write-golden PATH.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_auto_motion.c components/core/crazyflie/utils/src/auto_motion.c \
 *      -lm -o /tmp/test_auto_motion && /tmp/test_auto_motion
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "auto_motion.h"
#include "auto_motion_identity.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static autoMvIn_t baseIn(uint32_t now)
{
  autoMvIn_t in;
  memset(&in, 0, sizeof(in));
  in.nowMs = now;
  in.flight = true;
  in.sceneDM = 2.0f;
  in.sceneTracked = true;
  in.holdApplied = true;
  in.camFrameAgeMs = 120;
  in.tiltDeg = 1.0f;
  in.camHeight = true;
  in.vzH = 0.60f;
  in.hTargetM = 0.60f;
  in.airMs = 10000;
  return in;
}

/* Steps 1 ms at a time; the camera applies the epoch at once and measures
 * the lagged reference exactly (ideal tracking) unless frozen. */
typedef struct {
  autoMotion_t m;
  autoMvIn_t in;
  autoMvOut_t out;
  bool camFrozen;
  autoMvLand_t land;
} rig_t;

static void rigInit(rig_t *r)
{
  memset(r, 0, sizeof(*r));
  autoMotionInit(&r->m);
  r->in = baseIn(1000);
}

static void step(rig_t *r)
{
  r->in.nowMs++;
  r->land = autoMotionStep(&r->m, &r->in, &r->out);
  if (!r->camFrozen) {
    r->in.holdEpochSeen = r->out.epoch;
    r->in.progLat = r->out.refLat;
    r->in.progFwd = r->out.refFwd;
  } else if (r->in.holdEpochSeen != r->out.epoch) {
    r->in.holdEpochSeen = r->out.epoch;
    r->in.progLat = r->in.progFwd = 0.0f;
  }
  r->in.vzH += r->out.vRef * 0.001f;   /* the vertical controller follows */
}

static void steps(rig_t *r, int n)
{
  for (int i = 0; i < n; i++) {
    step(r);
  }
}

static bool active(const rig_t *r)
{
  return autoMotionActive(&r->m);
}

static int untilEnd(rig_t *r, int maxMs)
{
  for (int i = 1; i <= maxMs; i++) {
    step(r);
    if (!active(r)) {
      return i;
    }
  }
  return -1;
}

/* ---- helpers --------------------------------------------------------------- */

static void testLn(void)
{
  float worst = 0.0f;
  for (int i = 0; i < 200000; i++) {
    const float x = expf(-30.0f + 60.0f * (float)i / 200000.0f);
    const float a = autoMvLn(x), b = logf(x);
    const float e = fabsf(a - b) / fmaxf(1.0f, fabsf(b));
    worst = fmaxf(worst, e);
  }
  CHECK(worst < 3e-7f, "autoMvLn against logf over e^-30..e^30: worst %.2e", worst);
  CHECK(autoMvLn(1.0f) == 0.0f && fabsf(autoMvLn(2.0f) - 0.6931472f) < 1e-7f && autoMvLn(0.0f) < -1e29f &&
        autoMvLn(-1.0f) < -1e29f && autoMvLn(NAN) < -1e29f && fabsf(autoMvLn(1e-40f) - logf(1e-40f)) < 1e-4f,
        "ln 1 = 0, ln 2, ln of 0 / negative / NaN = -1e30, subnormal");
}

static void testProfile(void)
{
  const struct { float d, v, a; bool tri; } cs[] = {
    {0.50f, 0.20f, 0.10f, false}, {0.30f, 0.20f, 0.10f, true}, {1.50f, 0.12f, 0.10f, false}, {0.01f, 0.2f, 0.1f, true},
    {90.0f, 15.0f, 30.0f, false}, {0.05f, 0.10f, 0.20f, true},
  };
  for (size_t i = 0; i < sizeof(cs) / sizeof(cs[0]); i++) {
    autoMvProf_t p;
    autoMvProfPlan(&p, cs[i].d, cs[i].v, cs[i].a);
    float s = 0.0f, area = 0.0f, vmax = 0.0f, amax = 0.0f, prevV = 0.0f, prevS = 0.0f;
    bool mono = true;
    const float dt = 1e-4f;
    for (float t = 0.0f; t <= p.T + 0.01f; t += dt) {
      float v, a;
      autoMvProfEval(&p, t, &s, &v, &a);
      area += v * dt;
      vmax = fmaxf(vmax, v);
      amax = fmaxf(amax, fabsf(v - prevV) / dt);
      mono = mono && s >= prevS - 1e-6f;
      prevV = v;
      prevS = s;
    }
    CHECK(fabsf(s - cs[i].d) < 1e-5f * fmaxf(1.0f, cs[i].d) && fabsf(area - cs[i].d) < 2e-3f * fmaxf(1.0f, cs[i].d) &&
          vmax <= cs[i].v + 1e-5f && amax <= cs[i].a * 1.01f + 1e-3f && mono && (p.tC == 0.0f) == cs[i].tri,
          "trapezoid %.2f at %.2f/%.2f: ends at %.4f, area %.4f, speed max %.4f, accel max %.4f, %s (T %.2f s)",
          cs[i].d, cs[i].v, cs[i].a, s, area, vmax, amax, cs[i].tri ? "triangle" : "trapezoid", p.T);
  }
  autoMvProf_t p;
  autoMvProfPlan(&p, 0.0f, 0.2f, 0.1f);
  float s, v, a;
  autoMvProfEval(&p, 1.0f, &s, &v, &a);
  CHECK(p.T == 0.0f && s == 0.0f && v == 0.0f, "zero distance: nothing");
  autoMvProfPlan(&p, NAN, 0.2f, 0.1f);
  CHECK(p.T == 0.0f, "NaN distance: nothing");
}

/* ---- MOVE: conversions, rates, lag, feedforward ---------------------------- */

static void testMoveConversions(void)
{
  rig_t r;
  rigInit(&r);
  CHECK(autoMotionMove(&r.m, &r.in, 60, 0, 0) == AUTO_MVW_NONE, "MOVE right 60 accepted");
  float maxW = 0.0f, maxFf = 0.0f;
  int t = 0;
  while (active(&r) && t < 30000) {
    step(&r);
    maxW = fmaxf(maxW, r.out.rateLat);
    maxFf = fmaxf(maxFf, r.out.ffRollDeg);
    t++;
  }
  CHECK(r.m.state == AUTO_MV_DONE && fabsf(r.out.refLat - 0.30f) < 1e-6f && r.out.refFwd == 0.0f &&
        fabsf(r.out.progRightM - 0.60f) < 2e-3f, "lateral 0.60 m at D 2.0 m: reference 0.30 (x / D), DONE");
  CHECK(maxW <= AUTO_MV_W_LAT_MAX + 1e-5f && maxW * 2.0f <= AUTO_MV_V_H_MAX + 1e-5f,
        "lateral rate %.4f /s (<= %.2f /s and %.2f m/s at D 2.0)", maxW, (double)AUTO_MV_W_LAT_MAX,
        (double)AUTO_MV_V_H_MAX);
  CHECK(maxFf > 0.5f && maxFf < 1.2f, "feedforward lean up to %.2f deg (accel 0.10 m/s^2 + drag)", maxFf);
  /* forward: ln(D0 / (D0 - F)), the rate capped where D is smallest */
  rigInit(&r);
  r.in.sceneDM = 1.5f;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 90, 0) == AUTO_MVW_TOO_CLOSE, "MOVE forward 90 at D 1.5 m: TOO_CLOSE at "
        "k 0.6 (ends 1.5 (0.6 / 1.5)^(1/0.6) = 0.33 m)");
  CHECK(autoMotionMove(&r.m, &r.in, 0, 60, 0) == AUTO_MVW_NONE, "MOVE forward 60 at D 1.5 m (0.64 m at k 0.6)");
  float maxWf = 0.0f, minFf = 0.0f;
  while (active(&r)) {
    step(&r);
    maxWf = fmaxf(maxWf, r.out.rateFwd);
    minFf = fminf(minFf, r.out.ffPitchDeg);
  }
  CHECK(fabsf(r.out.refFwd - logf(1.5f / 0.9f)) < 1e-5f && fabsf(r.out.progFwdM - 0.60f) < 2e-3f &&
        maxWf <= AUTO_MV_W_FWD_MAX + 1e-4f && minFf < -0.3f,
        "forward 0.60 m from 1.5 m: ln(1.5 / 0.9) = %.4f, expansion rate max %.4f /s, nose-down feedforward %.2f",
        r.out.refFwd, maxWf, minFf);
  CHECK(fabsf(r.out.sceneDM - 0.90f) < 2e-3f && (r.out.flags & AUTO_MVF_D_TRACK),
        "the next D: 0.90 m, from the move (the observer still says 1.5)");
  /* back: negative, ln(D0 / (D0 + B)) */
  rigInit(&r);
  CHECK(autoMotionMove(&r.m, &r.in, 0, -100, 0) == AUTO_MVW_NONE, "MOVE back 100");
  untilEnd(&r, 30000);
  CHECK(fabsf(r.out.refFwd - logf(2.0f / 3.0f)) < 1e-5f && fabsf(r.out.progFwdM + 1.0f) < 2e-3f &&
        fabsf(r.out.sceneDM - 3.0f) < 2e-3f && (r.out.flags & AUTO_MVF_D_TRACK),
        "back 1.00 m: ln(2 / 3), progress -1.00 m, the forward D now the move's 3.0 (the observer's G still 2.0)");
  /* combined: one straight line, the horizontal speed <= AUTO_MV_V_H_MAX (0.30 m/s since 2026-10-14) in all */
  rigInit(&r);
  r.in.sceneDM = 4.0f;
  CHECK(autoMotionMove(&r.m, &r.in, 120, -90, 0) == AUTO_MVW_NONE, "MOVE right 120 back 90 at D 4.0");
  float maxV = 0.0f, prevLat = 0.0f, prevFwdM = 0.0f;
  float ratioMin = 9.0f, ratioMax = -9.0f;
  while (active(&r)) {
    step(&r);
    const float latM = r.out.refLat * 4.0f, fwdM = 4.0f * (1.0f - expf(-r.out.refFwd));
    const float v = sqrtf((latM - prevLat) * (latM - prevLat) + (fwdM - prevFwdM) * (fwdM - prevFwdM)) / 0.001f;
    if (r.m.state == AUTO_MV_RUN) {   /* (entering SETTLE drops the 0.12 s lag: a < 1 mm step) */
      maxV = fmaxf(maxV, v);
    }
    if (latM > 0.05f) {
      ratioMin = fminf(ratioMin, fwdM / latM);
      ratioMax = fmaxf(ratioMax, fwdM / latM);
    }
    prevLat = latM;
    prevFwdM = fwdM;
  }
  CHECK(maxV <= AUTO_MV_V_H_MAX * 1.01f && ratioMin > -0.76f && ratioMax < -0.74f,
        "straight line (back / right %.3f..%.3f = -0.75), speed max %.3f m/s", ratioMin, ratioMax, maxV);
}

/* The hold's reference is the profile AUTO_MV_CAM_LAG_MS ago (frames are that old). */
static void testLag(void)
{
  rig_t r;
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 100, 0, 0);
  steps(&r, 3000);
  autoMvProf_t p = r.m.ph;
  float s, v, a;
  autoMvProfEval(&p, 3.0f - (float)AUTO_MV_CAM_LAG_MS * 0.001f, &s, &v, &a);
  CHECK(fabsf(r.out.refLat - s / 2.0f) < 1e-5f && fabsf(r.out.rateLat - v / 2.0f) < 1e-5f,
        "3.0 s in: reference at 2.88 s (%.5f vs %.5f)", r.out.refLat, s / 2.0f);
  CHECK(fabsf(r.out.progRightM - s) < 1e-3f, "progress: the camera's measurement in metres (one step old)");
}

/* ---- refusals ----------------------------------------------------------------- */

static void testRefuse(void)
{
  rig_t r;
  rigInit(&r);
  const struct { int16_t r, f, u; autoMvWhy_t w; } cs[] = {
    {151, 0, 0, AUTO_MVW_RANGE}, {-151, 0, 0, AUTO_MVW_RANGE}, {0, 401, 0, AUTO_MVW_RANGE},
    {0, -151, 0, AUTO_MVW_RANGE}, {0, 0, 51, AUTO_MVW_RANGE}, {0, 0, -51, AUTO_MVW_RANGE}, {0, 0, 0, AUTO_MVW_RANGE},
    {0, 0, -36, AUTO_MVW_HEIGHT},   /* 0.60 - 0.36 = 0.24 m < 0.25 + 10 s x 0.01 m/s */
    {0, 0, -26, AUTO_MVW_HEIGHT},   /* 0.34 m < 0.35 m (the floor after 10 s of air) */
    {0, 0, -25, AUTO_MVW_NONE},
    {0, 151, 0, AUTO_MVW_TOO_CLOSE},   /* in range since 2026-10-14 (<= 400 cm); the scene at D 2.0 refuses it */
    {0, 113, 0, AUTO_MVW_TOO_CLOSE},   /* D 2.0, k 0.6: 2.0 (0.87 / 2.0)^(1/0.6) = 0.499 m */
    {0, 112, 0, AUTO_MVW_NONE},        /* 0.509 m */
    {101, 0, 0, AUTO_MVW_TOO_WIDE},    /* > 0.5 x D 2.0 = 1.00 m */
    {-100, 0, 0, AUTO_MVW_NONE},
  };
  for (size_t i = 0; i < sizeof(cs) / sizeof(cs[0]); i++) {
    autoMotionInit(&r.m);
    const autoMvWhy_t w = autoMotionMove(&r.m, &r.in, cs[i].r, cs[i].f, cs[i].u);
    CHECK(w == cs[i].w && r.m.why == w && (w == AUTO_MVW_NONE) == active(&r), "MOVE %d %d %d: %s (%s)", cs[i].r,
          cs[i].f, cs[i].u, autoMvWhyName(cs[i].w), autoMvWhyName(w));
  }
  autoMotionInit(&r.m);
  r.in.sceneDM = 1.99f;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 150, 0) == AUTO_MVW_TOO_CLOSE, "D 1.99, forward 150: TOO_CLOSE (49 cm)");
  CHECK(autoMotionMove(&r.m, &r.in, 100, 0, 0) == AUTO_MVW_TOO_WIDE && autoMotionMove(&r.m, &r.in, -100, 0, 0) ==
        AUTO_MVW_TOO_WIDE && autoMotionMove(&r.m, &r.in, -99, 0, 0) == AUTO_MVW_NONE,
        "D 1.99: right / left 1.00 m: TOO_WIDE (> 0.5 x 1.99 = 0.995 m), left 0.99 m accepted");
  autoMotionInit(&r.m);
  CHECK(autoMotionMove(&r.m, &r.in, 0, -150, 0) == AUTO_MVW_NONE, "back 150 is never too close");
  autoMotionInit(&r.m);
  r.in.sceneDM = 2.0f;
  /* the cap AUTO_MV_H_MAX_M, 2.00 m since 2026-10-15 (1.20 before) */
  r.in.hTargetM = 1.8f;
  r.in.vzH = 1.75f;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 21) == AUTO_MVW_HEIGHT, "1.80 + 0.21 = 2.01 m (commanded): HEIGHT");
  r.in.hTargetM = 1.7f;
  r.in.vzH = 1.8f;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 21) == AUTO_MVW_HEIGHT, "estimate 1.80 + 0.21: HEIGHT (either counts)");
  r.in.vzH = 2.02f;
  CHECK(autoMotionMove(&r.m, &r.in, 30, 0, 0) == AUTO_MVW_NONE && (autoMotionInit(&r.m), true),
        "a horizontal move with the estimate at 2.02 m: not refused (the estimate's bound is a vertical move's)");
  r.in.vzH = 1.8f;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 20) == AUTO_MVW_NONE, "1.80 + 0.20: OK (2.00 m)");
  CHECK(autoMotionMove(&r.m, &r.in, 10, 0, 0) == AUTO_MVW_BUSY && active(&r), "while running: BUSY");
  CHECK(autoMotionYaw(&r.m, &r.in, 30) == AUTO_MVW_BUSY, "YAW while running: BUSY");
  /* no D, no view, no camera height, battery, air */
  rigInit(&r);
  r.in.sceneTracked = false;
  CHECK(autoMotionMove(&r.m, &r.in, 10, 0, 0) == AUTO_MVW_NO_DIST, "not tracked: NO_DIST");
  r.in.sceneTracked = true;
  r.in.sceneDM = 0.29f;
  CHECK(autoMotionMove(&r.m, &r.in, 10, 0, 0) == AUTO_MVW_NO_DIST, "D 0.29 m: NO_DIST");
  r.in.sceneDM = NAN;
  CHECK(autoMotionMove(&r.m, &r.in, 10, 0, 0) == AUTO_MVW_NO_DIST, "D NaN: NO_DIST");
  r.in.sceneDM = 2.0f;
  r.in.holdCoasting = true;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 10) == AUTO_MVW_NO_VIEW, "coasting: NO_VIEW (vertical too)");
  r.in.holdCoasting = false;
  r.in.camHeight = false;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 10) == AUTO_MVW_NO_CAM_HEIGHT, "no camera height: a vertical move refused");
  CHECK(autoMotionMove(&r.m, &r.in, 10, 0, 0) == AUTO_MVW_NONE, "... a lateral one not");
  autoMotionInit(&r.m);
  r.in.camHeight = true;
  r.in.battLow = true;
  CHECK(autoMotionMove(&r.m, &r.in, 10, 0, 0) == AUTO_MVW_BATT && autoMotionYaw(&r.m, &r.in, 10) == AUTO_MVW_BATT,
        "battery low: BATT");
  /* a vertical MOVE down too (2026-10-15 review: a 0.5 m descent accepted at T_hold 54600-56000 ended HOLD_CAPPED in
   * the drain sim, test_interactive_sim.c testHigh200Sim) */
  r.in.vzH = 1.0f;
  r.in.hTargetM = 1.0f;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 10) == AUTO_MVW_BATT && autoMotionMove(&r.m, &r.in, 0, 0, -10) ==
        AUTO_MVW_BATT && r.m.state == AUTO_MV_IDLE, "battery low: a vertical MOVE up or down BATT");
  r.in.battLow = false;
  r.in.airMs = 200000;   /* no air clock (2026-10-15): AIR is never produced */
  CHECK(autoMotionMove(&r.m, &r.in, 0, -150, 0) == AUTO_MVW_NONE, "200 s of air: a 1.5 m move accepted (no AIR)");
  autoMotionInit(&r.m);
  CHECK(autoMotionYaw(&r.m, &r.in, 180) == AUTO_MVW_NONE, "200 s of air: YAW 180 accepted (no AIR)");
  autoMotionInit(&r.m);
  r.in.airMs = 10000;
  /* DRY / sim: no camera rules, the nominal D */
  autoMvIn_t dry = baseIn(1000);
  dry.flight = false;
  dry.simPlant = true;
  dry.sceneTracked = false;
  dry.holdApplied = false;
  autoMotionInit(&r.m);
  CHECK(autoMotionMove(&r.m, &dry, 50, 50, 0) == AUTO_MVW_NONE && fabsf(r.m.d0 - AUTO_MV_SIM_D_M) < 1e-6f,
        "DRY: the sim's D %.1f m, no view needed", (double)AUTO_MV_SIM_D_M);
}

/* ---- guards ------------------------------------------------------------------- */

static void testPauseAndGuards(void)
{
  rig_t r;
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 80, 0, 0);
  steps(&r, 2000);
  r.in.camFrameAgeMs = 301;
  step(&r);
  const float held = r.out.refLat;
  steps(&r, 1500);
  CHECK(r.m.state == AUTO_MV_PAUSED && r.out.refLat == held && r.out.rateLat == 0.0f && r.out.ffRollDeg == 0.0f &&
        r.land == AUTO_MVL_NONE, "frame 301 ms old: PAUSED, reference held, no feedforward");
  r.in.camFrameAgeMs = 100;
  step(&r);
  CHECK(r.m.state == AUTO_MV_RUN, "fresh again: RUN");
  const int t = untilEnd(&r, 30000);
  CHECK(t > 0 && r.m.state == AUTO_MV_DONE && fabsf(r.out.refLat - 0.40f) < 1e-6f, "and DONE (the pause cost "
        "nothing of the move)");
  /* 2 s: LAND */
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 80, 0, 0);
  steps(&r, 500);
  r.in.holdApplied = false;
  int n = 0;
  while (r.land == AUTO_MVL_NONE && n < 5000) {
    step(&r);
    n++;
  }
  CHECK(r.land == AUTO_MVL_NO_CAM && n == AUTO_MV_NO_CAM_LAND_MS + 1, "the hold not applied 2 s: the stop request "
        "NO_CAM (%d ms; a LAND until 2026-10-15)", n);
  /* tilt */
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 90);
  steps(&r, 100);
  r.in.tiltDeg = 8.1f;
  n = 0;
  while (r.land == AUTO_MVL_NONE && n < 1000) {
    step(&r);
    n++;
  }
  CHECK(r.land == AUTO_MVL_TILT && n == AUTO_MV_TILT_LAND_MS + 1, "tilt 8.1 deg 100 ms: the stop request TILT (%d)",
        n);
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 90);
  r.in.tiltDeg = 8.1f;
  steps(&r, 99);
  r.in.tiltDeg = 7.9f;
  steps(&r, 5);
  r.in.tiltDeg = 8.1f;
  steps(&r, 99);
  CHECK(r.land == AUTO_MVL_NONE, "tilt broken by a step at 7.9: nothing");
  /* progress far behind: STOP, snap */
  rigInit(&r);
  r.camFrozen = true;
  autoMotionMove(&r.m, &r.in, 100, 0, 0);
  const uint32_t e0 = r.m.epoch;
  n = untilEnd(&r, 20000);
  CHECK(n > 0 && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS && r.m.epoch == e0 + 1 && r.m.snap &&
        r.out.refLat == 0.0f && r.out.snapEpoch == r.m.epoch, "no progress: STOPPED PROGRESS after %d ms, snap epoch",
        n);
  /* ahead: a camera reading twice the motion */
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 100, 0, 0);
  bool stopped = false;
  for (int i = 0; i < 20000 && active(&r); i++) {
    step(&r);
    r.in.progLat = r.out.refLat * 2.0f;
    stopped = r.m.end == AUTO_MVE_PROGRESS;
  }
  CHECK(stopped, "measured twice the reference: STOPPED PROGRESS");
  /* in DRY none of this: progress ideal, no camera rules */
  rigInit(&r);
  r.in.flight = false;
  r.in.simPlant = true;
  r.in.holdApplied = false;
  r.camFrozen = true;
  autoMotionMove(&r.m, &r.in, 100, 0, 0);
  CHECK(untilEnd(&r, 30000) > 0 && r.m.state == AUTO_MV_DONE && fabsf(r.out.progRightM - 1.0f) < 1e-4f,
        "DRY: DONE, progress the reference (%.3f)", r.out.progRightM);
}

/* ---- STOP, epochs ---------------------------------------------------------------- */

/* The top speeds far from the scene (D 5.0 m, where the image-rate caps do not bind): 0.30 m/s sideways and
 * backward, the component toward the scene 0.20 m/s (AUTO_MV_V_FWD_MAX: person approaches as before 2026-10-14). */
static void testTopSpeeds(void)
{
  const struct { int16_t r, f; float want; const char *what; } t[] = {
    {100, 0, AUTO_MV_V_H_MAX, "right 1.00 m"},
    {-100, 0, AUTO_MV_V_H_MAX, "left 1.00 m"},
    {0, -100, AUTO_MV_V_H_MAX, "back 1.00 m"},
    {0, 100, AUTO_MV_V_FWD_MAX, "forward 1.00 m (toward the scene)"},
    {0, 150, AUTO_MV_V_FWD_MAX, "forward 1.50 m"},
    {100, 100, AUTO_MV_V_FWD_MAX * 1.41421356f, "right 1.00 forward 1.00 m (the forward component 0.20)"},
    {100, -100, AUTO_MV_V_H_MAX, "right 1.00 back 1.00 m"},
  };
  CHECK(AUTO_MV_V_FWD_MAX == 0.20f && AUTO_MV_V_H_MAX == 0.30f, "0.30 m/s, toward the scene 0.20 m/s");
  for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
    rig_t r;
    rigInit(&r);
    r.in.sceneDM = 5.0f;
    const autoMvWhy_t why = autoMotionMove(&r.m, &r.in, t[i].r, t[i].f, 0);
    CHECK(why == AUTO_MVW_NONE && fabsf(r.m.ph.vPk - t[i].want) < 1e-4f, "D 5.0 m, %s: peak %.3f m/s (want %.3f)",
          t[i].what, (double)r.m.ph.vPk, (double)t[i].want);
  }
}

static void testStopEpochs(void)
{
  rig_t r;
  rigInit(&r);
  autoMvOut_t o;
  autoMotionStep(&r.m, &r.in, &o);
  CHECK(o.epoch == 0 && o.snapEpoch == 0 && o.refLat == 0.0f && o.refFwd == 0.0f && o.rateLat == 0.0f &&
        o.rateFwd == 0.0f && o.ffRollDeg == 0.0f && o.ffPitchDeg == 0.0f && o.vRef == 0.0f && o.hOff == 0.0f &&
        o.yawOffDeg == 0.0f && !o.camGuessOk && !o.holdWait && o.state == AUTO_MV_IDLE,
        "no motion: every output exactly 0");
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  CHECK(r.m.state == AUTO_MV_IDLE && r.m.epoch == 0, "STOP with nothing: no change");
  autoMotionMove(&r.m, &r.in, 50, 50, 20);
  CHECK(r.m.epoch == 1 && !r.m.snap, "MOVE: epoch 1, no snap");
  steps(&r, 2000);
  const float vzH = r.in.vzH;
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  step(&r);
  CHECK(r.m.state == AUTO_MV_STOPPED && r.m.epoch == 2 && r.m.snap && r.out.snapEpoch == 2 && r.out.refLat == 0.0f &&
        r.out.vRef == 0.0f && fabsf(r.out.hOff - (vzH - 0.60f)) < 1e-4f,
        "STOP: epoch 2 with a snap, references 0, the height offset as reached (%.3f)", r.out.hOff);
  /* the brake: a lean against the speed the move had, for v / AUTO_MV_BRAKE_ACC */
  rig_t b;
  rigInit(&b);
  autoMotionMove(&b.m, &b.in, 100, 100, 0);
  steps(&b, 4000);
  autoMotionStop(&b.m, &b.in, AUTO_MVE_STOP_CMD);
  step(&b);
  const float ffR0 = b.out.ffRollDeg, ffP0 = b.out.ffPitchDeg;
  int brakeMs = 0;
  while (b.out.ffRollDeg != 0.0f && brakeMs < 5000) {
    step(&b);
    brakeMs++;
  }
  /* the planned peak speed: min(V_H_MAX, W_LAT x (D - F) x len / |R|, W_FWD x (D - F) x len / |F|, V_FWD x len / F)
   * = 0.170 m/s for (100, 100) at D 2.0: the sideways image-rate cap at the forward end (1.0 m away; 2026-10-14,
   * docs/autonomy-a2b.md 21.2; at the start D it was 0.339 and the forward cap's 0.212 bound) */
  const float lenM = sqrtf(2.0f);
  const float vPlan = fminf(fminf(AUTO_MV_V_H_MAX, AUTO_MV_V_FWD_MAX * lenM / 1.0f),
                            fminf(AUTO_MV_W_LAT_MAX * (2.0f - 1.0f) * lenM / 1.0f,
                                  AUTO_MV_W_FWD_MAX * (2.0f - 1.0f) * lenM / 1.0f));
  const float wantMs = 1000.0f * vPlan / AUTO_MV_BRAKE_ACC;
  CHECK(ffR0 < -0.4f && ffP0 > 0.4f && fabsf(ffR0 + ffP0) < 1e-4f && fabsf((float)brakeMs - wantMs) <= 5.0f &&
        fabsf(b.m.ph.vPk - vPlan) < 1e-4f,
        "STOP at %.3f m/s right-forward (the planned peak %.3f): brake left %.2f deg, nose up %.2f deg, for %d ms "
        "(v / a = %.0f)", vPlan, b.m.ph.vPk, ffR0, ffP0, brakeMs, wantMs);
  autoMotionMove(&r.m, &r.in, -20, 0, 0);
  CHECK(r.m.epoch == 3 && !r.m.snap && r.m.snapEpoch == 2, "the next MOVE: epoch 3, the snap epoch kept 2");
  autoMotionStop(&r.m, &r.in, AUTO_MVE_LEFT_HOVER);
  CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_LEFT_HOVER && r.m.snapEpoch == 4, "leaving HOVER: a snap");
  autoMvOut_t idle;
  autoMotionIdleOut(&r.m, &idle);
  CHECK(idle.epoch == 4 && idle.snapEpoch == 4 && idle.refLat == 0.0f && idle.vRef == 0.0f && !idle.holdWait,
        "outside HOVER: the epoch kept, no references");
}

/* ---- vertical ------------------------------------------------------------------- */

static void testVertical(void)
{
  rig_t r;
  rigInit(&r);
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 30) == AUTO_MVW_NONE, "MOVE up 30");
  float vmax = 0.0f;
  bool offOk = true;
  while (active(&r)) {
    step(&r);
    vmax = fmaxf(vmax, r.out.vRef);
    offOk = offOk && r.out.hOff >= -1e-6f && r.out.hOff <= 0.30f + 1e-6f;
  }
  CHECK(offOk, "the height offset stays within 0..0.30 m while moving");
  CHECK(r.m.state == AUTO_MV_DONE && fabsf(r.in.vzH - 0.90f) < 0.01f && fabsf(r.out.hOff - 0.30f) < 1e-6f &&
        vmax <= AUTO_MV_UP_VREF_MAX + 1e-6f && r.out.vRef == 0.0f, "up 0.30: height 0.90 (%.3f), offset 0.30, "
        "vRef max %.3f, then exactly 0", r.in.vzH, vmax);
  /* the vertical controller lags: the position correction closes the gap */
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 0, 0, -20);
  for (int i = 0; i < 20000 && active(&r); i++) {
    r.in.nowMs++;
    r.land = autoMotionStep(&r.m, &r.in, &r.out);
    r.in.vzH += 0.7f * r.out.vRef * 0.001f;   /* follows at 70 % */
  }
  CHECK(r.m.state == AUTO_MV_DONE && fabsf(r.in.vzH - 0.40f) < 0.03f, "a lagging controller still ends at 0.40 m "
        "(%.3f)", r.in.vzH);
}

/* ---- yaw ---------------------------------------------------------------------- */

static void testYaw(void)
{
  rig_t r;
  rigInit(&r);
  CHECK(autoMotionYaw(&r.m, &r.in, -120) == AUTO_MVW_NONE && r.m.epoch == 1 && !r.m.snap, "YAW -120: epoch 1");
  float maxRate = 0.0f, prev = 0.0f;
  bool turnOk = true;
  while (r.m.state == AUTO_MV_RUN) {
    step(&r);
    maxRate = fmaxf(maxRate, fabsf(r.out.yawOffDeg - prev) * 1000.0f);
    prev = r.out.yawOffDeg;
    turnOk = turnOk && r.out.camGuessOk && r.out.refLat == 0.0f && r.out.vRef == 0.0f;
  }
  CHECK(turnOk, "turning: the camera guess allowed, no hold reference, no vertical reference");
  CHECK(fabsf(r.out.yawOffDeg + 120.0f) < 1e-3f && maxRate <= AUTO_MV_YAW_RATE_DPS + 1e-2f && r.m.epoch == 2 &&
        r.m.snap, "-120 deg at <= %.0f deg/s (%.2f), the hold re-captured", (double)AUTO_MV_YAW_RATE_DPS, maxRate);
  /* tracked: no bob */
  steps(&r, AUTO_MV_YAW_DWELL_MS + 2);
  CHECK(r.m.state == AUTO_MV_DONE && !r.m.bob && !r.out.camGuessOk && !r.m.dStale, "tracked after the dwell: DONE");
  /* the offset wraps: -120 - 120 = +120 */
  autoMotionYaw(&r.m, &r.in, -120);
  untilEnd(&r, 20000);
  CHECK(fabsf(r.out.yawOffDeg - 120.0f) < 1e-3f, "-240 is +120 (%.2f)", r.out.yawOffDeg);
  /* a turn that loses the tracker: the bob, whole cycles, back to the start height */
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 45);
  r.in.sceneTracked = false;
  float zmin = 9.0f, zmax = -9.0f;
  int n = 0;
  while (active(&r) && n < 40000) {
    step(&r);
    if (r.m.bob) {
      zmin = fminf(zmin, r.in.vzH - 0.60f);
      zmax = fmaxf(zmax, r.in.vzH - 0.60f);
      if (r.m.bobT > 2.2f * 2.0f * 1.0f) {
        r.in.sceneTracked = true;   /* the tracker has a G again during the third cycle */
      }
    }
    n++;
  }
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE && zmax > 0.03f && zmax < 0.06f && zmin > -0.01f &&
        fabsf(r.in.vzH - 0.60f) < 0.005f && !r.m.dStale,
        "the bob: up to +%.3f m, ends back at the start (%.4f) at a cycle end, D valid", zmax, r.in.vzH - 0.60f);
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 45);
  r.in.sceneTracked = false;
  const uint16_t seq45 = r.m.seq;
  bool undoSeen = false, sameSeq = true;
  n = 0;
  while (active(&r) && n < 90000) {
    step(&r);
    undoSeen = undoSeen || (r.out.flags & AUTO_MVF_UNDO) != 0;
    sameSeq = sameSeq && r.out.seq == seq45;
    n++;
  }
  CHECK(!active(&r) && undoSeen && sameSeq && r.m.end == AUTO_MVE_RETRACK && fabsf(r.out.yawOffDeg) < 1e-3f &&
            r.m.dStale && !r.out.camGuessOk && r.out.sceneDM == 0.0f,
        "never tracked: the bob without a G turns it back (UNDO, the same seq, heading %.3f), then DONE RETRACK after "
        "%d ms, D stale, no guess any more", r.out.yawOffDeg, n);
  r.in.sceneTracked = true;
  step(&r);
  CHECK(!r.m.dStale && r.out.sceneDM > 0.0f, "the tracker back later: D valid again");
  /* the D of the last forward move belongs to the old view */
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 0, 50, 0);
  untilEnd(&r, 20000);
  CHECK(r.m.dTrack > 1.4f && r.m.dTrack < 1.6f, "forward 0.5 m: D 1.5 m tracked");
  autoMotionYaw(&r.m, &r.in, 10);
  CHECK(r.m.dTrack == 0.0f, "a turn forgets it");
}

/* ---- fixes after the verification of 591f176 ------------------------------------ */

/* A finished MOVE keeps its end as the hold's reference outside HOVER too: a
 * LAND after it must not take -latTgt off the hold's integral (the drone flew
 * back toward the move's start while descending). */
static void testIdleKeepsDoneMove(void)
{
  const int16_t mv[][2] = {{80, 0}, {0, -50}, {-60, 40}};
  for (int k = 0; k < 3; k++) {
    rig_t r;
    rigInit(&r);
    CHECK(autoMotionMove(&r.m, &r.in, mv[k][0], mv[k][1], 0) == AUTO_MVW_NONE, "MOVE %d %d", mv[k][0], mv[k][1]);
    untilEnd(&r, 60000);
    step(&r);
    autoMvOut_t idle;
    autoMotionIdleOut(&r.m, &idle);
    CHECK(r.m.state == AUTO_MV_DONE && !r.m.snap && (mv[k][0] == 0 || r.out.refLat != 0.0f) &&
          idle.epoch == r.out.epoch && idle.snapEpoch == r.out.snapEpoch && idle.refLat == r.out.refLat &&
          idle.refFwd == r.out.refFwd && idle.refLat == r.m.latTgt && idle.refFwd == r.m.fwdTgt &&
          idle.rateLat == 0.0f && idle.rateFwd == 0.0f && idle.ffRollDeg == 0.0f && idle.vRef == 0.0f,
          "MOVE %d %d DONE: the idle outputs keep its end (lat %.4f fwd %.4f, epoch %u) and nothing moves",
          mv[k][0], mv[k][1], idle.refLat, idle.refFwd, (unsigned)idle.epoch);
  }
  /* after a STOP (a snap epoch) and after a turn: 0, as before */
  rig_t r;
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 60, 0, 0);
  steps(&r, 2000);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  autoMvOut_t idle;
  autoMotionIdleOut(&r.m, &idle);
  CHECK(idle.refLat == 0.0f && idle.refFwd == 0.0f && idle.snapEpoch == idle.epoch, "STOPPED: references 0 (snap)");
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 30);
  untilEnd(&r, 30000);
  autoMotionIdleOut(&r.m, &idle);
  CHECK(r.m.state == AUTO_MV_DONE && idle.refLat == 0.0f && idle.refFwd == 0.0f, "YAW DONE: references 0");
}

/* Which D converts what: forward the last forward / back move's dTrack,
 * lateral the larger, the TOO_CLOSE floor the lower bound at k 0.6. */
static void testWhichD(void)
{
  rig_t r;
  rigInit(&r);   /* D 2.0 */
  CHECK(autoMotionMove(&r.m, &r.in, 0, 50, 0) == AUTO_MVW_NONE, "forward 0.50 at D 2.0");
  untilEnd(&r, 30000);
  const float low = 2.0f * powf(0.75f, 1.0f / AUTO_MV_K_LOW);   /* 1.24 m */
  CHECK(fabsf(r.m.dTrack - 1.5f) < 2e-3f && fabsf(r.m.dTrackLow - low) < 3e-3f && fabsf(r.out.sceneDM - 1.5f) < 2e-3f,
        "after it: forward D 1.50 (dTrack), lower bound %.3f (k 0.6), reported %.3f", r.m.dTrackLow, r.out.sceneDM);
  /* forward floor on the lower bound: 1.24 ((1.5 - F) / 1.5)^(1/0.6) >= 0.5: F <= 0.63 */
  CHECK(autoMotionMove(&r.m, &r.in, 0, 66, 0) == AUTO_MVW_TOO_CLOSE, "forward 0.66 then: TOO_CLOSE (lower bound)");
  CHECK(autoMotionMove(&r.m, &r.in, 0, 60, 0) == AUTO_MVW_NONE && fabsf(r.m.d0 - 1.5f) < 2e-3f &&
        fabsf(r.m.fwdTgt - logf(1.5f / 0.9f)) < 1e-3f, "forward 0.60: converted with D 1.50");
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  steps(&r, 10);
  /* lateral: the larger D (2.0, the observer's), so the move errs short */
  CHECK(autoMotionMove(&r.m, &r.in, 40, 0, 0) == AUTO_MVW_NONE && fabsf(r.m.dLat0 - 2.0f) < 1e-6f &&
        fabsf(r.m.latTgt - 0.20f) < 1e-6f, "lateral 0.40: converted with max(2.0, dTrack) = 2.0 (%.3f)", r.m.dLat0);
  /* back: dTrack farther than G; the lateral D follows it, the floor's bound stays G's */
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 0, -100, 0);
  untilEnd(&r, 30000);
  CHECK(fabsf(r.m.dTrack - 3.0f) < 3e-3f && fabsf(r.m.dTrackLow - 2.0f) < 1e-6f, "back 1.0 from 2.0: dTrack 3.0, "
        "bound 2.0");
  CHECK(autoMotionMove(&r.m, &r.in, 60, 0, 0) == AUTO_MVW_NONE && fabsf(r.m.dLat0 - 3.0f) < 3e-3f,
        "lateral after it: D 3.0");
  /* DRY: the ideal D after a forward move, with the same floor */
  rigInit(&r);
  r.in.flight = false;
  r.in.simPlant = true;
  autoMotionMove(&r.m, &r.in, 0, 80, 0);
  untilEnd(&r, 30000);
  CHECK(fabsf(r.out.sceneDM - 1.2f) < 2e-3f && (r.out.flags & AUTO_MVF_D_SIM), "DRY: forward 0.8 from the sim's 2.0: "
        "D 1.20 (%.3f)", r.out.sceneDM);
}

/* The forward floor at the expansion scale's low: the true end of an
 * accepted move stays >= 0.50 m for every k >= 0.6 (the hold drives the
 * measured k ln(D0 / D) to the target). */
static void testKFloor(void)
{
  float worst = 9.0f;
  int acc = 0;
  for (int di = 0; di < 30; di++) {
    const float d = 0.6f + 0.15f * (float)di;   /* 0.6 .. 4.95 m */
    for (int f = 5; f <= 150; f += 5) {
      rig_t r;
      rigInit(&r);
      r.in.sceneDM = d;
      if (autoMotionMove(&r.m, &r.in, 0, (int16_t)f, 0) != AUTO_MVW_NONE) {
        continue;
      }
      acc++;
      for (float k = 0.6f; k <= 2.5f; k += 0.1f) {
        worst = fminf(worst, d * expf(-r.m.fwdTgt / k));
      }
    }
  }
  CHECK(acc > 100 && worst >= 0.4999f, "%d forward moves accepted over D 0.6..5 m: the true end >= %.3f m for every "
        "k in 0.6..2.5", acc, worst);
}

/* The re-view bob: a lateral shift of 0.35 D since the last fresh view ends
 * the move with 2 whole bob cycles, D stale meanwhile; STOP does not end it. */
static void testReview(void)
{
  rig_t r;
  rigInit(&r);
  CHECK(autoMotionMove(&r.m, &r.in, 60, 0, 0) == AUTO_MVW_NONE && !r.m.review, "right 0.60 at D 2.0 (0.30 D): no "
        "re-view");
  untilEnd(&r, 30000);
  CHECK(r.m.state == AUTO_MV_DONE && fabsf(r.m.viewShift - 0.30f) < 1e-4f && (r.out.flags & AUTO_MVF_D_VALID),
        "DONE, the view shifted 0.30 D");
  CHECK(autoMotionMove(&r.m, &r.in, -20, 0, 0) == AUTO_MVW_NONE && r.m.review, "left 0.20 more (0.40 D in all): "
        "it ends with a re-view");
  int n = 0;
  while (active(&r) && !r.m.bob && n < 30000) {
    step(&r);
    n++;
  }
  const float h0 = r.in.vzH;
  CHECK(r.m.bob && r.m.state == AUTO_MV_SETTLE && (r.out.flags & AUTO_MVF_D_STALE) && r.out.sceneDM == 0.0f &&
        r.out.holdWait, "the move done: the bob runs, D stale, the wait held");
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  CHECK(r.m.bob && active(&r), "a STOP during the re-view bob: it runs on");
  float zmax = 0.0f, refLat = 0.0f;
  bool refOk = true;
  int t = 0;
  while (active(&r) && t < 20000) {
    step(&r);
    zmax = fmaxf(zmax, r.in.vzH - h0);
    refOk = refOk && fabsf(r.out.refLat - (-0.10f)) < 1e-6f;
    refLat = r.out.refLat;
    t++;
  }
  const float cycle = 2.0f * r.m.pb.T;
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE && refOk && refLat == -0.10f &&
        fabsf((float)t * 0.001f - (float)AUTO_MV_REVIEW_CYCLES * cycle) < 0.05f && zmax > 0.03f &&
        fabsf(r.in.vzH - h0) < 0.005f && r.m.viewShift == 0.0f && (r.out.flags & AUTO_MVF_D_VALID) &&
        r.out.sceneDM == 2.0f, "the bob: %d ms (%d cycles of %.2f s), up to +%.3f m, back at the start, the hold "
        "kept the move's end, D valid again (%.2f)", t, AUTO_MV_REVIEW_CYCLES, cycle, zmax, r.out.sceneDM);
  /* a forward move's dTrack belongs to the old view: forgotten by the re-view */
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 0, 40, 0);
  untilEnd(&r, 30000);
  CHECK(r.m.dTrack > 0.0f, "forward 0.40: dTrack %.2f", r.m.dTrack);
  CHECK(autoMotionMove(&r.m, &r.in, 80, 0, 0) == AUTO_MVW_NONE && r.m.review, "right 0.80 then (0.40 D): re-view");
  untilEnd(&r, 40000);
  CHECK(r.m.dTrack == 0.0f && r.m.dTrackLow == 0.0f && r.out.sceneDM == 2.0f, "after the re-view: the observer's D");
  /* LEFT_HOVER ends it */
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 80, 0, 0);
  while (active(&r) && !r.m.bob) {
    step(&r);
  }
  autoMotionStop(&r.m, &r.in, AUTO_MVE_LEFT_HOVER);
  CHECK(!active(&r) && !r.m.bob && r.m.end == AUTO_MVE_LEFT_HOVER && r.m.snap, "leaving HOVER ends the re-view");
  /* DRY and BENCH: no bob */
  rigInit(&r);
  r.in.flight = false;
  r.in.simPlant = true;
  autoMotionMove(&r.m, &r.in, 100, 0, 0);
  CHECK(!r.m.review && untilEnd(&r, 30000) > 0 && !r.m.bob, "DRY: no re-view");
  /* a small turn whose tracker kept its G counts as a view shift */
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 15);
  untilEnd(&r, 20000);
  CHECK(fabsf(r.m.viewShift - 15.0f * 0.01745329f) < 1e-4f, "a 15 deg turn, tracker kept: view shift %.3f",
        r.m.viewShift);
  CHECK(autoMotionMove(&r.m, &r.in, 20, 0, 0) == AUTO_MVW_NONE && r.m.review, "then right 0.20 (0.36 D): re-view");
}

/* A move down's floor grows with the air time (the estimate drifts, reading high). */
static void testDriftFloor(void)
{
  rig_t r;
  rigInit(&r);
  r.in.airMs = 30000;   /* floor 0.25 + 0.30 = 0.55 m */
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, -6) == AUTO_MVW_HEIGHT, "30 s of air at 0.60 m: down 0.06 refused");
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, -4) == AUTO_MVW_NONE, "down 0.04 (0.56 m) accepted");
  rigInit(&r);
  r.in.airMs = 40000;   /* 0.65 m until 2026-10-15; the drift term capped at AUTO_MV_H_DRIFT_CAP_M: 0.55 m */
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, -6) == AUTO_MVW_HEIGHT, "40 s of air at 0.60 m: down 0.06 refused");
  autoMotionInit(&r.m);
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, -4) == AUTO_MVW_NONE, "... down 0.04 (0.56 m) accepted (capped floor)");
  autoMotionInit(&r.m);
  r.in.airMs = 200000;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, -4) == AUTO_MVW_NONE, "200 s of air: the floor still 0.55 m");
  autoMotionInit(&r.m);
  CHECK(autoMotionMove(&r.m, &r.in, 30, 0, 0) == AUTO_MVW_NONE, "a horizontal move: not refused");
  autoMotionInit(&r.m);
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 10) == AUTO_MVW_NONE, "up 0.10: accepted");
  rigInit(&r);
  r.in.airMs = 0;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, -35) == AUTO_MVW_NONE, "no air time: down to 0.25 m");
}

/* The tighter progress guard and the settle that saw no motion (a frozen view). */
static void testFrozen(void)
{
  const int16_t mv[] = {50, 15, -30};
  for (int k = 0; k < 3; k++) {
    rig_t r;
    rigInit(&r);
    r.camFrozen = true;
    autoMotionMove(&r.m, &r.in, mv[k], 0, 0);
    const int n = untilEnd(&r, 30000);
    autoMvOut_t idle;
    autoMotionIdleOut(&r.m, &idle);
    CHECK(n > 0 && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS && r.m.snap && idle.refLat == 0.0f,
          "frozen view, right %d cm: STOPPED PROGRESS after %d ms, the target dropped (snap)", mv[k], n);
  }
  /* a camera that sees 60 % of the motion: the settle times out, DONE (no stop) */
  rig_t r;
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 50, 0, 0);
  bool stopped = false;
  for (int i = 0; i < 30000 && active(&r); i++) {
    step(&r);
    r.in.progLat = r.out.refLat * 0.6f;
    stopped = stopped || r.m.end == AUTO_MVE_PROGRESS;
  }
  CHECK(!stopped && r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_SHORT && fabsf(r.m.latTgt - 0.15f) < 1e-5f &&
          fabsf(r.out.refLat - 0.15f) < 1e-5f && fabsf(r.out.progRightM - 0.30f) < 1e-3f,
        "60 %% of the motion seen: SHORT after the settle timeout, the measured end (0.30 m) the hold's reference "
        "(%.4f), progress %.3f m", r.out.refLat, r.out.progRightM);
}

/* The progress guard once the profile has ended (user decision 2026-10-13,
 * evening), and the SHORT end. The camera follows the reference ideally
 * through RUN; in SETTLE the test sets what it measures. */
static void settleRig(rig_t *r, int16_t right, int16_t fwd, float d)
{
  rigInit(r);
  r->in.sceneDM = d;
  CHECK(autoMotionMove(&r->m, &r->in, right, fwd, 0) == AUTO_MVW_NONE, "MOVE %d %d at D %.2f", right, fwd, d);
  /* the profile with an ideal camera, its last steps on a frame of the epoch before (so the step entering
   * SETTLE neither guards nor settles): SETTLE then starts on what the test measures */
  while (r->m.state == AUTO_MV_RUN && r->m.tRun + 0.005f < fmaxf(r->m.ph.T, r->m.pv.T)) {
    step(r);
  }
  while (r->m.state == AUTO_MV_RUN) {
    r->in.holdEpochSeen = r->m.epoch - 1u;
    r->in.nowMs++;
    r->land = autoMotionStep(&r->m, &r->in, &r->out);
  }
  r->in.holdEpochSeen = r->m.epoch;
}

/* Steps n ms with the camera measuring (lat m, fwd m) from the move's start; true if it ended. */
static bool settleHold(rig_t *r, float latM, float fwdM, int n)
{
  for (int i = 0; i < n && active(r); i++) {
    r->in.progLat = latM / r->m.dLat0;
    r->in.progFwd = logf(r->m.d0 / (r->m.d0 - fwdM));
    step(r);
  }
  return !active(r);
}

static void testSettleGuard(void)
{
  rig_t r;
  /* tiger2 (2026-10-13): forward 1.00 m at D 2.51; 15 cm sideways in the settle stopped it */
  settleRig(&r, 0, 100, 2.5135f);
  CHECK(fabsf(r.m.margLatM - 0.25f) < 1e-6f, "forward 1.00: the sideways margin 0.25 m (metres at the distance now; "
        "RUN 0.15 m and the settle 0.25 m at the start distance before 2026-10-14): %.3f", r.m.margLatM);
  /* sideways in the rig's metres at the start distance; at the settle's distance now (1.51 m) x 1.51 / 2.51 */
  const float toNow = 1.5135f / 2.5135f;
  CHECK(!settleHold(&r, -0.20f / toNow, 0.86f, 2900) && r.m.state == AUTO_MV_SETTLE,
        "settle: 0.86 m forward, 0.20 m sideways at the distance now: no stop (0.15 m before)");
  settleHold(&r, -0.20f / toNow, 0.86f, 200);
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_SHORT && fabsf(r.out.refLat + 0.20f / 1.5135f) < 1e-5f &&
        fabsf(r.out.progFwdM - 0.86f) < 2e-3f && fabsf(r.out.progRightM + 0.20f / toNow) < 2e-3f,
        "then SHORT at the settle timeout: 0.20 m off its line at the distance now (> 0.15), held there (reference "
        "%.4f): travel %.3f fwd %.3f right (the status: metres at the start distance)", r.out.refLat,
        r.out.progFwdM, r.out.progRightM);
  settleRig(&r, 0, 100, 2.5135f);
  settleHold(&r, -0.20f, 0.92f, 3100);
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE, "0.92 m forward, 0.20 m sideways at the start "
        "distance (0.12 m at the distance now): DONE at the timeout (SHORT before 2026-10-14: the end's 0.06 m of a "
        "long move read 0.17 m)");
  settleRig(&r, 0, 100, 2.5135f);
  CHECK(!settleHold(&r, -0.10f, 0.92f, 2900), "0.92 m forward, 0.10 m sideways: no stop");
  settleHold(&r, -0.10f, 0.92f, 200);
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE && r.out.refLat == 0.0f &&
        fabsf(r.out.refFwd - r.m.fwdTgt) < 1e-6f, "then DONE at the timeout (within 0.04 + 10 %% along, 0.10 m off its "
        "line): the target kept");
  settleRig(&r, 0, 100, 2.5135f);
  CHECK(!settleHold(&r, 0.24f, 0.95f, 2900), "0.24 m sideways in the settle: no stop");
  /* the sideways guard in metres at the distance now (2026-10-14): the settle's D_now is 2.51 - 1.00 = 1.51 m. The
   * rig's 0.26 m at the start distance (progLat 0.26 / 2.51) is 0.157 m there: no stop (the radian guard stopped it) */
  settleRig(&r, 0, 100, 2.5135f);
  CHECK(!settleHold(&r, -0.26f, 0.95f, 2900), "0.26 m sideways at the start distance (0.157 m at 1.51 m): no stop");
  settleRig(&r, 0, 100, 2.5135f);
  CHECK(!settleHold(&r, -0.24f * 2.5135f / 1.5135f, 0.95f, 2900), "0.24 m sideways at the distance now: no stop");
  settleRig(&r, 0, 100, 2.5135f);
  CHECK(settleHold(&r, -0.26f * 2.5135f / 1.5135f, 0.95f, 500) && r.m.state == AUTO_MV_STOPPED &&
        r.m.end == AUTO_MVE_PROGRESS, "0.26 m sideways at the distance now (> 0.25 m): STOP PROGRESS");
  /* an under-shoot within the margin: SHORT, the measured end the move's end */
  settleRig(&r, 0, 100, 2.5135f);
  CHECK(!settleHold(&r, 0.0f, 0.70f, 2900), "settle at 0.70 of 1.00 m forward: no stop");
  settleHold(&r, 0.0f, 0.70f, 200);
  autoMvOut_t idle;
  autoMotionIdleOut(&r.m, &idle);
  const float f70 = logf(2.5135f / (2.5135f - 0.70f));
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_SHORT && !r.m.snap && fabsf(r.out.refFwd - f70) < 1e-5f &&
        fabsf(idle.refFwd - f70) < 1e-5f && fabsf(r.out.progFwdM - 0.70f) < 2e-3f &&
        fabsf(r.m.dTrack - (2.5135f - 0.70f)) < 3e-3f,
        "timed out short: SHORT (state DONE), the hold's reference the measured end %.4f (idle too), travel %.3f m, "
        "dTrack %.3f", r.out.refFwd, r.out.progFwdM, r.m.dTrack);
  /* a fall back toward the start past the target's margin: PROGRESS at once (a receding person or scene: the hold
   * would push the drone on toward it; the verification of e4f7686) */
  settleRig(&r, 0, 100, 2.5135f);
  const float mF = r.m.margFwd;
  const float loM = 2.5135f * (1.0f - expf(-(r.m.fwdTgt - mF)));   /* target - margin in metres: 0.567 */
  r.in.progFwd = r.m.fwdTgt - mF - 0.002f;
  step(&r);
  CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS, "back past the target by the margin (%.3f m "
        "measured): PROGRESS", loM);
  settleRig(&r, 0, 100, 2.5135f);
  r.in.progFwd = r.m.fwdTgt - mF + 0.002f;
  step(&r);
  CHECK(active(&r), "short of the target within the margin: settling on");
  settleRig(&r, 0, 100, 2.5135f);
  settleHold(&r, 0.0f, 0.75f, 300);
  float fStop = -1.0f;
  int msStop = -1;
  for (int i = 0; i < 1000 && active(&r); i++) {
    const float f = 0.75f - 0.75f * (float)i / 1000.0f;   /* 0.75 m falling to 0 in 1 s */
    settleHold(&r, 0.0f, f, 1);
    if (!active(&r)) {
      fStop = f;
      msStop = 300 + i;
    }
  }
  CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS && fStop < loM && fStop > loM - 0.01f,
        "a reading falling from 0.75 m in the settle: PROGRESS as it passes %.3f m (%.3f m, settle +%d ms)", loM,
        fStop, msStop);
  /* the frozen-view rule at the timeout: below half of the move inside the margin (a lateral 0.60 m: 0.27 m) */
  settleRig(&r, -60, 0, 2.0f);
  CHECK(!settleHold(&r, -0.28f, 0.0f, 2900), "left 0.60: 0.28 m measured (its margin 0.33 m): no stop");
  CHECK(settleHold(&r, -0.28f, 0.0f, 200) && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS && r.m.snap,
        "the timeout below half of it: STOP PROGRESS");
  /* past the target by the margin: PROGRESS at once */
  settleRig(&r, 0, 100, 2.5135f);
  r.in.progFwd = r.m.fwdTgt + mF + 0.002f;
  step(&r);
  CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS, "past the target by the margin: PROGRESS");
  settleRig(&r, 0, 100, 2.5135f);
  r.in.progFwd = r.m.fwdTgt + mF - 0.002f;
  step(&r);
  CHECK(active(&r), "past the target within the margin: settling on");
  /* lateral moves: the lateral rules on the lateral axis; the forward axis (toward the scene) keeps the RUN margin,
   * 0.15 m / D, both ways (a false backward reading makes the hold push toward the scene) */
  settleRig(&r, -60, 0, 2.0f);
  CHECK(!r.m.review && fabsf(r.m.margLatM - 0.33f) < 1e-5f && fabsf(r.m.margFwd * 2.0f - 0.15f) < 1e-5f,
        "left 0.60 at D 2.0: its lateral margin 0.33 m (> 0.25), forward 0.15 m, as in RUN");
  CHECK(!settleHold(&r, -0.40f, 0.10f, 2900), "left 0.60: 0.40 m measured, 0.10 m forward drift: no stop");
  settleHold(&r, -0.40f, 0.10f, 200);
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_SHORT && fabsf(r.out.refLat + 0.20f) < 1e-4f &&
        fabsf(r.out.progRightM + 0.40f) < 2e-3f, "SHORT at -0.40 m (reference %.4f)", r.out.refLat);
  settleRig(&r, -60, 0, 2.0f);
  CHECK(settleHold(&r, -0.55f, 0.16f, 500) && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS,
        "left 0.60: 0.16 m forward (toward the scene) in the settle: PROGRESS");
  settleRig(&r, -60, 0, 2.0f);
  CHECK(settleHold(&r, -0.55f, -0.16f, 500) && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS,
        "left 0.60: a 0.16 m backward reading in the settle: PROGRESS");
  settleRig(&r, 36, 0, 1.2f);
  CHECK(settleHold(&r, 0.30f, 0.18f, 500) && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS,
        "right 0.36 at D 1.2: 0.18 m forward in the settle: PROGRESS");
  settleRig(&r, -60, 0, 2.0f);
  r.in.progLat = (-0.60f - 0.15f - 0.30f * 0.60f - 0.01f) / 2.0f;
  step(&r);
  CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS, "left: past the target by the margin: PROGRESS");
  /* a small lateral part: the settle's lateral margin is still 0.25 m (RUN: 0.15 m + 30 % of it) */
  settleRig(&r, 10, 100, 2.5135f);
  CHECK(fabsf(r.m.margLatM - 0.25f) < 1e-6f, "right 0.10 + forward 1.00: the sideways margin 0.25 m (RUN 0.18 m "
        "and the settle 0.25 m before 2026-10-14)");
  CHECK(!settleHold(&r, -0.12f, 0.95f, 2900), "0.12 m left of the start (0.22 m off its lateral target): no stop");
  {
    /* off its lateral target in metres at the settle's distance now (2.51 - 1.00 m): 0.24 m no stop, 0.26 m PROGRESS */
    const float dNow = 2.5135f - 1.0f, latT = 0.10f;
    settleRig(&r, 10, 100, 2.5135f);
    CHECK(!settleHold(&r, (latT - 0.24f / dNow * 2.5135f), 0.95f, 2900), "0.24 m off its lateral target at the "
          "distance now: no stop");
    settleRig(&r, 10, 100, 2.5135f);
    CHECK(settleHold(&r, (latT - 0.26f / dNow * 2.5135f), 0.95f, 500) && r.m.state == AUTO_MV_STOPPED &&
          r.m.end == AUTO_MVE_PROGRESS, "0.26 m off its lateral target at the distance now: PROGRESS");
  }
  /* RUN: the same guard, in metres at the lagged reference's distance (3 s into a 1.00 m forward move from 2.51 m:
   * s 0.376 m at 2.88 s, D_now 2.14 m): 0.24 m sideways no stop, 0.26 m PROGRESS */
  bool stopped = false;
  for (int k = 0; k < 2; k++) {
    rigInit(&r);
    r.in.sceneDM = 2.5135f;
    autoMotionMove(&r.m, &r.in, 0, 100, 0);
    steps(&r, 3000);
    const float dNow = 2.5135f - 0.376f;
    stopped = false;
    for (int i = 0; i < 100 && !stopped; i++) {
      r.in.progLat = -(k ? 0.26f : 0.24f) / dNow;
      step(&r);
      stopped = r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS;
    }
    CHECK(stopped == (k == 1), "RUN: %.2f m sideways at the distance now: %s", k ? 0.26 : 0.24,
          k ? "PROGRESS" : "no stop");
  }
  /* a SHORT move that ends with a re-view bob: the bob keeps the measured end, the end SHORT */
  settleRig(&r, 80, 0, 2.0f);
  CHECK(r.m.review, "right 0.80 at D 2.0 (0.40 D): re-view");
  settleHold(&r, 0.56f, 0.0f, 3100);
  CHECK(r.m.bob && active(&r) && fabsf(r.out.refLat - 0.28f) < 1e-4f, "SHORT at 0.56 m: the bob runs on the measured "
        "end (%.4f)", r.out.refLat);
  untilEnd(&r, 20000);
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_SHORT && fabsf(r.out.progRightM - 0.56f) < 2e-3f,
        "the bob over: SHORT, travel %.3f m", r.out.progRightM);
}

/* The measured travel stays in the status once a MOVE has ended, whatever
 * ended it (its progress used to restart with the snap epoch: the tiger
 * flights' 'fwd -0.01 m' after ~0.8 m flown). */
static void testTravel(void)
{
  rig_t r;
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 100, 0, 0);
  steps(&r, 4000);
  const float at = r.out.progRightM;
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  for (int i = 0; i < 2000; i++) {
    r.in.holdEpochSeen = r.m.epoch;
    r.in.progLat = 0.03f;   /* the hold's wander in the snap epoch */
    r.land = autoMotionStep(&r.m, &r.in, &r.out);
    r.in.nowMs++;
  }
  CHECK(r.m.state == AUTO_MV_STOPPED && at > 0.30f && fabsf(r.out.progRightM - at) < 1e-6f,
        "STOP after 4 s: travel %.3f m kept (not the snap epoch's 0.06 m)", r.out.progRightM);
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 0, -60, 0);
  untilEnd(&r, 30000);
  for (int i = 0; i < 500; i++) {
    r.in.progFwd = -0.5f;
    r.land = autoMotionStep(&r.m, &r.in, &r.out);
    r.in.nowMs++;
  }
  CHECK(r.m.state == AUTO_MV_DONE && fabsf(r.out.progFwdM + 0.60f) < 2e-3f, "DONE: travel %.3f m kept after it",
        r.out.progFwdM);
  CHECK(autoMotionMove(&r.m, &r.in, 30, 0, 0) == AUTO_MVW_NONE && r.m.progLat == 0.0f && r.m.progFwd == 0.0f,
        "the next MOVE starts from 0");
}

/* A STOP during a turn's dwell or re-track bob does not end it (it would
 * take the camera height estimate's guess away and land HOVER_NO_CAM). */
static void testStopDuringRetrack(void)
{
  rig_t r;
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 60);
  r.in.sceneTracked = false;
  while (r.m.state == AUTO_MV_RUN) {
    step(&r);
  }
  steps(&r, 100);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  step(&r);
  CHECK(active(&r) && r.out.camGuessOk && r.m.state == AUTO_MV_SETTLE, "STOP in the dwell: it runs on, the guess "
        "allowed");
  while (!r.m.bob && active(&r)) {
    step(&r);
  }
  steps(&r, 700);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  step(&r);
  CHECK(active(&r) && r.m.bob && r.out.camGuessOk, "STOP in the bob: it runs on");
  r.in.sceneTracked = true;
  const int n = untilEnd(&r, 20000);
  CHECK(n > 0 && r.m.end == AUTO_MVE_DONE && !r.m.dStale, "then DONE with a G (%d ms)", n);
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 60);
  steps(&r, 500);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  step(&r);
  CHECK(r.m.turnDone && active(&r) && fabsf(r.out.yawOffDeg) < 60.0f, "STOP in the turn: the turn ends at %.1f deg, "
        "the dwell follows", r.out.yawOffDeg);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_LEFT_HOVER);
  CHECK(!active(&r) && r.m.end == AUTO_MVE_LEFT_HOVER, "leaving HOVER ends the dwell");
}

/* The committed height offset reaches +1.80 m (a 0.20 m take-off to the 2.00 m cap in 0.50 m moves and a last
 * 0.30; vert_ctrl's VZ_HOFF_MAX is the same plus a SET_HEIGHT's 0.50 m; +1.00 m to 1.20 m until 2026-10-15). */
static void testHoffRange(void)
{
  rig_t r;
  rigInit(&r);
  r.in.hTargetM = 0.20f;
  r.in.vzH = 0.20f;
  r.in.airMs = 0;
  static const int ups[] = { 50, 50, 50, 30 };
  float h = 0.20f;
  for (int i = 0; i < 4; i++) {
    CHECK(autoMotionMove(&r.m, &r.in, 0, 0, ups[i]) == AUTO_MVW_NONE, "up %.2f (%d)", ups[i] * 0.01, i);
    untilEnd(&r, 30000);
    h += (float)ups[i] * 0.01f;
    r.in.vzH = h;   /* the estimate where it was sent (the rig ends within 1 mm) */
  }
  CHECK(fabsf(r.out.hOff - 1.8f) < 1e-5f && autoMotionMove(&r.m, &r.in, 0, 0, 1) == AUTO_MVW_HEIGHT &&
        AUTO_MV_HOFF_MAX >= 1.8f + 0.5f - 1e-6f,
        "0.20 + 0.50 + 0.50 + 0.50 + 0.30: offset %.3f, no higher (AUTO_MV_H_MAX_M); AUTO_MV_HOFF_MAX %.2f holds it + "
        "a SET_HEIGHT's 0.50 m", r.out.hOff, (double)AUTO_MV_HOFF_MAX);
}


/* ---- SET_HEIGHT's correction (autoMotionSetHeight; auto_fsm.h AUTO_SH_*, docs/autonomy-a2b.md 23) ------------------ */

/* A correction of d from an estimate h (the drone believed h, it is at h + d): the committed offset moves by d, a
 * vertical MOVE of upM from h + d (the estimate the glue shifts); the rig shifts the estimate as the glue does. */
static autoMvWhy_t shStart(rig_t *r, float h, float d, float upM)
{
  const autoMvWhy_t w = autoMotionSetHeight(&r->m, &r->in, d, upM);
  if (w == AUTO_MVW_NONE) {
    r->in.vzH = h + d;
  }
  return w;
}

static void testSetHeight(void)
{
  static const float ds[] = { 0.50f, -0.50f, 0.30f, -0.30f, 0.10f, -0.12f };
  for (size_t k = 0; k < sizeof(ds) / sizeof(ds[0]); k++) {
    const float d = ds[k];
    rig_t r;
    rigInit(&r);
    r.in.hTargetM = 1.00f;
    r.in.vzH = 1.00f;
    r.m.hOff = 0.10f;   /* a committed move up before */
    const float hOff0 = r.m.hOff;
    const uint16_t seq0 = r.m.seq;
    CHECK(shStart(&r, 1.00f, d, -d) == AUTO_MVW_NONE && fabsf(r.m.hStart - (1.00f + d)) < 1e-6f &&
          r.m.hOff == hOff0 + d && r.m.upM == -d && r.m.state == AUTO_MV_RUN && r.m.kind == AUTO_MVK_MOVE &&
          r.m.lenM == 0.0f && r.m.seq == seq0 + 1 && r.m.yawFlags == AUTO_MVF_SET_HEIGHT,
          "d %+.2f: accepted, hStart = vzH + d (%.3f), hOff = old + d (%.3f), upM = -d, a vertical-only MOVE, seq + 1, "
          "the SET_HEIGHT flag", d, r.m.hStart, r.m.hOff);
    step(&r);
    float sp, vp, ap;
    autoMvProfEval(&r.m.pv, r.m.tRun, &sp, &vp, &ap);
    const float vWant = (d > 0.0f ? -1.0f : 1.0f) * vp + AUTO_MV_UP_KP * ((d > 0.0f ? -sp : sp) - 0.0f);
    CHECK(r.m.state == AUTO_MV_RUN && fabsf(r.out.vRef - vWant) < 1e-6f && fabsf(r.out.vRef) < 0.001f &&
          (r.out.flags & AUTO_MVF_SET_HEIGHT) && r.out.kind == AUTO_MVK_MOVE && fabsf(r.out.hOff - (hOff0 + d)) < 1e-6f,
          "d %+.2f, the first step: RUN (no PROGRESS stop), vRef %.5f = the profile's (no jump), mvFlags 0x40, the "
          "reference at old + d", d, r.out.vRef);
    float vmax = 0.0f, offLo = 9.0f, offHi = -9.0f;
    int n = 0;
    while (active(&r) && n < 20000) {
      step(&r);
      n++;
      vmax = fmaxf(vmax, fabsf(r.out.vRef));
      if (active(&r)) {
        offLo = fminf(offLo, r.out.hOff);
        offHi = fmaxf(offHi, r.out.hOff);
      }
    }
    const float tWant = r.m.pv.T;
    CHECK(r.m.pv.vPk <= AUTO_MV_V_UP_MAX + 1e-6f && r.m.pv.a <= AUTO_MV_A_UP + 1e-6f && vmax <= AUTO_MV_UP_VREF_MAX + 1e-6f,
          "d %+.2f: the profile at <= %.2f m/s and %.2f m/s^2, |vRef| <= %.3f (max %.4f)", d,
          (double)AUTO_MV_V_UP_MAX, (double)AUTO_MV_A_UP, (double)AUTO_MV_UP_VREF_MAX, vmax);
    CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE && r.m.hOff == hOff0 && r.out.hOff == hOff0 &&
          fabsf(r.in.vzH - 1.00f) < 0.01f && (float)n * 0.001f <= tWant + 3.0f + 0.01f && offLo >= hOff0 - 1e-5f - fmaxf(0.0f, -d) &&
          offHi <= hOff0 + 1e-5f + fmaxf(0.0f, d),
          "d %+.2f: DONE in %.2f s (profile %.2f + settle <= 3 s), the offset back to the old one exactly (%.6f), the "
          "estimate back at 1.00 m (%.3f): the drone where it believed it was", d, (double)n * 0.001, tWant, r.m.hOff,
          r.in.vzH);
  }
  /* the durations: 0.1 m about 1.4 s, 0.3 m 2.75 s, 0.5 m 4.1 s */
  autoMvProf_t p1, p3, p5;
  autoMvProfPlan(&p1, 0.10f, AUTO_MV_V_UP_MAX, AUTO_MV_A_UP);
  autoMvProfPlan(&p3, 0.30f, AUTO_MV_V_UP_MAX, AUTO_MV_A_UP);
  autoMvProfPlan(&p5, 0.50f, AUTO_MV_V_UP_MAX, AUTO_MV_A_UP);
  printf("info: SET_HEIGHT corrections: 0.10 m %.2f s, 0.30 m %.2f s, 0.50 m %.2f s (+ the settle, <= 3 s)\n", p1.T, p3.T,
         p5.T);
  CHECK(fabsf(p1.T - 1.41f) < 0.02f && fabsf(p3.T - 2.75f) < 0.02f && fabsf(p5.T - 4.08f) < 0.02f,
        "the corrections' profiles: 1.41 / 2.75 / 4.08 s");

  /* STOP, a contact and leaving HOVER mid-way: the offset is old + d + the measured travel (the reference where the
   * drone stopped) */
  static const autoMvEnd_t stops[] = { AUTO_MVE_STOP_CMD, AUTO_MVE_CONTACT, AUTO_MVE_LEFT_HOVER };
  for (int k = 0; k < 3; k++) {
    rig_t r;
    rigInit(&r);
    r.in.hTargetM = 1.00f;
    r.in.vzH = 1.00f;
    const float d = k == 1 ? -0.40f : 0.40f;
    shStart(&r, 1.00f, d, -d);
    steps(&r, 1500);
    const float prog = r.in.vzH - (1.00f + d);
    autoMotionStop(&r.m, &r.in, stops[k]);
    CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == stops[k] && fabsf(prog) > 0.05f &&
          fabsf(r.m.hOff - (0.0f + d + prog)) < 1e-5f && !r.m.backOff,
          "%s mid-way (travel %+.3f of %+.2f): STOPPED, offset old + d + travel = %.3f (%.3f); no back-off anchor",
          autoMvEndName(stops[k]), prog, -d, d + prog, r.m.hOff);
  }

  /* a drone that does not move (the controller stuck): PROGRESS at the vertical margin 0.15 + 0.5 |up| */
  {
    rig_t r;
    rigInit(&r);
    r.in.hTargetM = 1.00f;
    r.in.vzH = 1.00f;
    shStart(&r, 1.00f, 0.40f, -0.40f);
    const float h0 = r.in.vzH;
    int n = 0;
    for (; n < 20000 && active(&r); n++) {
      r.in.nowMs++;
      r.land = autoMotionStep(&r.m, &r.in, &r.out);   /* vzH stays: progUp 0 */
    }
    float sp, vp, ap;
    autoMvProfEval(&r.m.pv, (float)n * 0.001f, &sp, &vp, &ap);
    CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS && r.in.vzH == h0 &&
          fabsf(sp - (AUTO_MV_UP_MARGIN_M + 0.5f * 0.40f)) < 0.01f && fabsf(r.m.hOff - 0.40f) < 1e-5f,
          "the estimate never moves: PROGRESS once the reference is %.3f m off (0.15 + 0.5 x 0.40); offset old + d",
          sp);
  }

  /* the refusals: BUSY (a MOVE, a YAW, a correction running), NO_VIEW, NO_CAM_HEIGHT, BATT only when climbing */
  {
    rig_t r;
    rigInit(&r);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 10) == AUTO_MVW_NONE, "MOVE up 10");
    CHECK(autoMotionSetHeight(&r.m, &r.in, 0.2f, -0.2f) == AUTO_MVW_BUSY && r.m.why == AUTO_MVW_BUSY &&
          r.m.yawFlags == 0, "a MOVE running: BUSY");
    rigInit(&r);
    CHECK(autoMotionYaw(&r.m, &r.in, 30) == AUTO_MVW_NONE, "YAW 30");
    CHECK(autoMotionSetHeight(&r.m, &r.in, 0.2f, -0.2f) == AUTO_MVW_BUSY, "a YAW running: BUSY");
    rigInit(&r);
    shStart(&r, 0.60f, 0.2f, -0.2f);
    const uint16_t s1 = r.m.seq;
    CHECK(autoMotionSetHeight(&r.m, &r.in, -0.1f, 0.1f) == AUTO_MVW_BUSY && r.m.seq == s1, "a correction running: BUSY");
    CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 10) == AUTO_MVW_BUSY, "... and a MOVE BUSY while it runs");
    rigInit(&r);
    r.in.holdApplied = false;
    CHECK(autoMotionSetHeight(&r.m, &r.in, 0.2f, -0.2f) == AUTO_MVW_NO_VIEW && r.m.state == AUTO_MV_IDLE, "NO_VIEW");
    rigInit(&r);
    r.in.camHeight = false;
    CHECK(autoMotionSetHeight(&r.m, &r.in, 0.2f, -0.2f) == AUTO_MVW_NO_CAM_HEIGHT, "NO_CAM_HEIGHT");
    r.in.flight = false;
    CHECK(autoMotionSetHeight(&r.m, &r.in, 0.2f, -0.2f) == AUTO_MVW_NO_CAM_HEIGHT,
          "NO_CAM_HEIGHT in DRY too (open loop ignores vRef)");
    rigInit(&r);
    r.in.battLow = true;
    CHECK(autoMotionSetHeight(&r.m, &r.in, -0.2f, 0.2f) == AUTO_MVW_BATT, "battery low: a climbing correction BATT");
    CHECK(autoMotionSetHeight(&r.m, &r.in, 0.2f, -0.2f) == AUTO_MVW_BATT && r.m.state == AUTO_MV_IDLE,
          "battery low: a descending correction BATT too (its braking may end HOLD_CAPPED, 2026-10-15 review)");
    rigInit(&r);
    CHECK(autoMotionSetHeight(&r.m, &r.in, NAN, 0.1f) == AUTO_MVW_RANGE && r.m.state == AUTO_MV_IDLE, "NaN: RANGE");
  }

  /* a later Claude MOVE clears the flag; the correction's own end flag stays with it until then */
  {
    rig_t r;
    rigInit(&r);
    shStart(&r, 0.60f, 0.1f, -0.1f);
    untilEnd(&r, 20000);
    step(&r);
    CHECK(r.m.state == AUTO_MV_DONE && (r.out.flags & AUTO_MVF_SET_HEIGHT), "after it: mvFlags still 0x40 (the latest "
          "motion was a correction)");
    CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 10) == AUTO_MVW_NONE && r.m.yawFlags == 0, "a MOVE: the flag cleared");
    step(&r);
    CHECK(!(r.out.flags & AUTO_MVF_SET_HEIGHT), "... and in the outputs");
  }
}


/* ---- long moves and the guards for them (2026-10-14, docs/autonomy-a2b.md 21) ---------------------------------- */

/* The scene distances: the observer's d (valid to AUTO_MV_D_MAX_M); a D beyond it through the D a back move measured
 * (dTrack, its lower bound the observer's), as auto_motion.c takes it. */
static void sceneAt(rig_t *r, float d)
{
  if (d <= AUTO_MV_D_MAX_M) {
    r->in.sceneDM = d;
    r->m.dTrack = r->m.dTrackLow = 0.0f;
  } else {
    r->in.sceneDM = AUTO_MV_D_MAX_M;
    r->m.dTrack = r->m.dTrackLow = d;
  }
}

static void testLongRange(void)
{
  /* the limits themselves */
  const struct { int32_t r, f, u; bool ok; } lim[] = {
    {0, 400, 0, true}, {0, 401, 0, false}, {150, 0, 0, true}, {151, 0, 0, false}, {-151, 0, 0, false},
    {0, -150, 0, true}, {0, -151, 0, false}, {150, 370, 0, true}, {150, 371, 0, false}, {-150, 370, 0, true},
    {-150, 371, 0, false}, {0, 150, 10, true}, {0, 151, 10, false}, {0, 151, -10, false}, {0, -150, 50, true},
    {0, 0, 50, true}, {0, 0, 51, false}, {0, 0, 0, false}, {100, 300, 0, true}, {150, 150, 50, true},
  };
  int ok = 0;
  for (unsigned i = 0; i < sizeof lim / sizeof lim[0]; i++) {
    const bool got = autoMvRangeOk(lim[i].r, lim[i].f, lim[i].u);
    ok += got == lim[i].ok;
    if (got != lim[i].ok) {
      printf("  autoMvRangeOk(%d, %d, %d) = %d\n", (int)lim[i].r, (int)lim[i].f, (int)lim[i].u, got);
    }
  }
  CHECK(ok == (int)(sizeof lim / sizeof lim[0]), "autoMvRangeOk: forward <= 400, sideways and back <= 150, the vector "
        "<= 400 (150 / 371: 401.2 cm), up <= 50 and then forward <= 150 (%d of %d)", ok,
        (int)(sizeof lim / sizeof lim[0]));
  CHECK(AUTO_MV_FWD_MAX_CM == 400 && AUTO_MV_LEN_MAX_CM == 400 && AUTO_MV_HORIZ_MAX_CM == 150 &&
        AUTO_MV_UP_MAX_CM == 50, "the limits 400 / 400 / 150 / 50 cm");
  /* MOVEs: accepted where the scene allows them, RANGE beyond the limits whatever the scene */
  const struct { int16_t r, f, u; float d; autoMvWhy_t w; const char *what; } cs[] = {
    {0, 400, 0, 8.0f, AUTO_MVW_NONE, "forward 400 at D 8 (dTrack; the observer's lower bound 6)"},
    {0, 400, 0, 6.0f, AUTO_MVW_NONE, "forward 400 at D 6.0"},
    {150, 370, 0, 6.0f, AUTO_MVW_NONE, "right 150 forward 370 at D 6.0 (398.6 cm)"},
    {0, 401, 0, 8.0f, AUTO_MVW_RANGE, "forward 401"},
    {151, 0, 0, 8.0f, AUTO_MVW_RANGE, "right 151"},
    {0, -151, 0, 8.0f, AUTO_MVW_RANGE, "back 151"},
    {150, 371, 0, 8.0f, AUTO_MVW_RANGE, "right 150 forward 371 (401.2 cm)"},
    {0, 151, 10, 8.0f, AUTO_MVW_RANGE, "up 10 with forward 151"},
    {0, 150, 10, 8.0f, AUTO_MVW_NONE, "up 10 with forward 150"},
    {0, 400, 0, 4.66f, AUTO_MVW_TOO_CLOSE, "forward 400 at D 4.66: the scene (TOO_CLOSE)"},
    {0, 330, 0, 4.66f, AUTO_MVW_NONE, "forward 330 at D 4.66 (the Mac's largest)"},
  };
  for (unsigned i = 0; i < sizeof cs / sizeof cs[0]; i++) {
    rig_t r;
    rigInit(&r);
    r.in.hTargetM = 1.0f;
    r.in.vzH = 1.0f;
    sceneAt(&r, cs[i].d);
    const autoMvWhy_t w = autoMotionMove(&r.m, &r.in, cs[i].r, cs[i].f, cs[i].u);
    CHECK(w == cs[i].w, "%s: %s (%s)", cs[i].what, autoMvWhyName(cs[i].w), autoMvWhyName(w));
  }
}

/* Every accepted forward move ends >= 0.50 m from the scene for an expansion scale k of 0.6..2.5: the hold drives
 * k ln(D0 / D) to ln(D / (D - F)) (the observer's D, here the true one), so the true end is D ((D - F) / D)^(1/k). */
static void testFloorSweep(void)
{
  static const float ks[] = {0.6f, 0.7f, 0.8f, 1.0f, 1.3f, 1.6f, 2.0f, 2.5f};
  int accepted = 0, bad = 0;
  float worst = 9.0f, worstD = 0.0f, worstF = 0.0f;
  for (int di = 0; di <= 75; di++) {
    const float d = 0.5f + 0.1f * (float)di;
    for (int fc = 5; fc <= 400; fc += 5) {
      rig_t r;
      rigInit(&r);
      sceneAt(&r, d);
      if (autoMotionMove(&r.m, &r.in, 0, (int16_t)fc, 0) != AUTO_MVW_NONE) {
        continue;
      }
      accepted++;
      for (unsigned k = 0; k < sizeof ks / sizeof ks[0]; k++) {
        const float end = d * powf((d - 0.01f * (float)fc) / d, 1.0f / ks[k]);
        if (end < worst) {
          worst = end;
          worstD = d;
          worstF = 0.01f * (float)fc;
        }
        bad += !(end >= 0.50f - 1e-4f);
      }
    }
  }
  printf("info: floor sweep: %d forward moves accepted, the nearest true end %.3f m (D %.1f m, F %.2f m, k 0.6)\n",
         accepted, worst, worstD, worstF);
  CHECK(accepted > 500 && bad == 0, "floor sweep, forward 5..400 cm x D 0.5..8 m x k 0.6..2.5: %d accepted, every "
        "true end >= 0.50 m (worst %.3f m: D %.1f, F %.2f, k 0.6)", accepted, worst, worstD, worstF);
}

/* The profiles toward the scene (0.20 m/s, 0.10 m/s^2) and the air rule (profile + 3 s settle + 8.5 s LAND). */
static void testLongProfiles(void)
{
  static const struct { int16_t f; float t; } pt[] = {
    {150, 9.5f}, {200, 12.0f}, {250, 14.5f}, {300, 17.0f}, {350, 19.5f}, {400, 22.0f},
  };
  int ok = 0;
  for (unsigned i = 0; i < sizeof pt / sizeof pt[0]; i++) {
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 6.0f);
    const autoMvWhy_t w = autoMotionMove(&r.m, &r.in, 0, pt[i].f, 0);
    ok += w == AUTO_MVW_NONE && fabsf(r.m.ph.T - pt[i].t) <= 0.05f;
    printf("info: forward %.2f m at D 6.0: profile %.2f s (%s)\n", pt[i].f * 0.01f, (double)r.m.ph.T,
           autoMvWhyName(w));
  }
  CHECK(ok == 6, "forward 1.5 / 2 / 2.5 / 3 / 3.5 / 4 m take 9.5 / 12 / 14.5 / 17 / 19.5 / 22 s (+-0.05; %d of 6)", ok);
  rig_t r;
  rigInit(&r);
  sceneAt(&r, 6.0f);
  r.in.airMs = 110000;   /* 110 s: the air clock refused it until 2026-10-15 */
  CHECK(autoMotionMove(&r.m, &r.in, 0, 400, 0) == AUTO_MVW_NONE, "a 4 m move at 110 s of air: accepted (no AIR)");
}

/* The sideways guard in metres at the reference's distance now (auto_motion.h), on a 3 m forward move from D 4.66
 * (the gym): at the start (D_now 4.66 m) and at the end (the settle: D_now 1.66 m), 0.25 m of sideways wander. The
 * camera measures wander / D_now (the integral of v_right / D_now). */
static void testLatGuardMetres(void)
{
  const float d0 = 4.66f;
  for (int k = 0; k < 2; k++) {
    rig_t r;
    rigInit(&r);
    sceneAt(&r, d0);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 300, 0) == AUTO_MVW_NONE && fabsf(r.m.margLatM - 0.25f) < 1e-6f,
          "forward 3.00 m at D 4.66: accepted, the sideways margin 0.25 m");
    steps(&r, 300);   /* s 0.0016 m at the lagged 0.18 s */
    bool stopped = false;
    const float w = k ? 0.26f : 0.24f;
    for (int i = 0; i < 200 && !stopped; i++) {
      r.in.progLat = w / d0;
      r.in.nowMs++;
      r.land = autoMotionStep(&r.m, &r.in, &r.out);
      r.in.holdEpochSeen = r.out.epoch;
      r.in.progFwd = r.out.refFwd;
      stopped = r.m.state == AUTO_MV_STOPPED;
    }
    CHECK(stopped == (k == 1) && (!stopped || r.m.end == AUTO_MVE_PROGRESS), "at the start: %.2f m sideways %s", w,
          k ? "gives PROGRESS" : "passes");
  }
  for (int k = 0; k < 2; k++) {
    rig_t r;
    settleRig(&r, 0, 300, d0);
    const float w = k ? 0.30f : 0.20f, dNow = d0 - 3.0f;
    bool stopped = false;
    for (int i = 0; i < 2900 && !stopped; i++) {
      r.in.progLat = w / dNow;
      r.in.progFwd = r.m.fwdTgt;
      step(&r);
      r.in.progLat = w / dNow;
      stopped = r.m.state == AUTO_MV_STOPPED;
    }
    CHECK(stopped == (k == 1) && (!stopped || r.m.end == AUTO_MVE_PROGRESS), "at the end (the settle, D_now 1.66 m): "
          "%.2f m sideways %s (the radian guard of before: 0.15 m x 1.66 / 4.66 = 0.05 m)", w,
          k ? "gives PROGRESS" : "passes");
  }
}

/* The live front guard: STOP (end FRONT) once the measured forward progress passes progFwdMax = 0.6 ln(D_low /
 * 0.40); never on an ideally tracked accepted move (its target <= 0.6 ln(D_low / 0.50): 0.134 below); never on a
 * sideways or back move. */
static void testFrontGuard(void)
{
  const float d0 = 4.66f;
  /* the largest forward move the drone takes at D 4.66: the TOO_CLOSE boundary */
  int16_t fMax = 400;
  for (;; fMax--) {
    rig_t t;
    rigInit(&t);
    sceneAt(&t, d0);
    if (autoMotionMove(&t.m, &t.in, 0, fMax, 0) == AUTO_MVW_NONE) {
      break;
    }
  }
  rig_t r;
  rigInit(&r);
  sceneAt(&r, d0);
  autoMotionMove(&r.m, &r.in, 0, fMax, 0);
  const float want = AUTO_MV_K_LOW * logf(d0 / 0.40f);   /* (the margins of before: mvfstop 40) */
  const float gap = r.m.progFwdMax - r.m.fwdTgt;
  CHECK(fabsf(r.m.progFwdMax - want) < 1e-4f && gap >= 0.134f - 0.005f && gap < 0.15f,
        "forward %d cm (the TOO_CLOSE boundary at D 4.66): progFwdMax %.4f = 0.6 ln(4.66 / 0.40), its target %.4f "
        "below it by %.3f (0.134 = 0.6 ln(0.50 / 0.40))", fMax, r.m.progFwdMax, r.m.fwdTgt, gap);
  const int n = untilEnd(&r, 60000);
  CHECK(n > 0 && r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE, "ideally tracked: DONE after %d ms", n);
  /* the guard alone (inside the forward margin) in the settle and in RUN */
  settleRig(&r, 0, fMax, d0);
  CHECK(r.m.progFwdMax + 0.01f - r.m.fwdTgt < r.m.margFwd, "progFwdMax + 0.01 is inside the forward margin (%.3f)",
        r.m.margFwd);
  r.in.progFwd = r.m.progFwdMax + 0.01f;
  step(&r);
  CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_FRONT && r.m.snap, "settle: progFwdMax + 0.01: STOPPED, "
        "end FRONT (%s)", autoMvEndName(r.m.end));
  settleRig(&r, 0, fMax, d0);
  bool on = true;
  for (int i = 0; i < 500 && on; i++) {
    r.in.progLat = 0.0f;
    r.in.progFwd = r.m.progFwdMax - 0.01f;
    step(&r);
    on = autoMotionActive(&r.m);
  }
  CHECK(r.m.end != AUTO_MVE_FRONT && (on || (r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE)),
        "settle: progFwdMax - 0.01: no stop (it settles: %s %s; 0.15 m past the target, within the settle's "
        "tolerance of a 3.4 m move)", autoMvStateName(r.m.state), autoMvEndName(r.m.end));
  rigInit(&r);
  sceneAt(&r, d0);
  autoMotionMove(&r.m, &r.in, 0, 300, 0);
  steps(&r, 5000);
  r.in.progFwd = r.m.progFwdMax + 0.01f;
  step(&r);
  CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_FRONT && r.out.ffPitchDeg > 0.5f,
        "RUN (forward 3.00, 5 s in): past progFwdMax: STOPPED FRONT, braking nose up %.2f deg", r.out.ffPitchDeg);
  /* in metres: at k 0.6 the camera reads 0.6 ln(D0 / D_now), so the stop is at D_now = 0.40 m for D_low = D0 */
  const float dStop = d0 * expf(-r.m.progFwdMax / AUTO_MV_K_LOW);
  CHECK(fabsf(dStop - 0.40f) < 1e-4f, "at k 0.6 it trips with the scene %.3f m ahead", dStop);
  /* sideways and back moves: no front guard (progFwdMax 1e30); a large forward reading is the PROGRESS margin's */
  static const int16_t mv[3][2] = {{100, 0}, {-150, 0}, {0, -100}};
  for (int k = 0; k < 3; k++) {
    rigInit(&r);
    sceneAt(&r, d0);
    autoMotionMove(&r.m, &r.in, mv[k][0], mv[k][1], 0);
    CHECK(r.m.progFwdMax > 1e29f, "MOVE %d %d: no front guard", mv[k][0], mv[k][1]);
    steps(&r, 1000);
    r.in.progFwd = 5.0f;
    step(&r);
    CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS, "MOVE %d %d, a forward reading of 5: "
          "PROGRESS, never FRONT (%s)", mv[k][0], mv[k][1], autoMvEndName(r.m.end));
  }
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 30);
  CHECK(r.m.progFwdMax > 1e29f, "YAW: no front guard");
  /* DRY: no camera guards */
  rigInit(&r);
  r.in.flight = false;
  r.in.simPlant = true;
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  steps(&r, 2000);
  r.in.progFwd = 9.0f;
  step(&r);
  CHECK(autoMotionActive(&r.m), "DRY: no front guard (the progress is the reference)");
}

/* The sideways image rate v_lat / D_now at its largest over the profile (D_now = d0 - s uy at the forward end). */
static float latRateMax(const autoMotion_t *m)
{
  float worst = 0.0f;
  const float ux = m->latM / m->lenM, uy = m->fwdM / m->lenM;
  for (int i = 0; i <= 4000; i++) {
    float s, v, a;
    autoMvProfEval(&m->ph, m->ph.T * (float)i / 4000.0f, &s, &v, &a);
    worst = fmaxf(worst, fabsf(v * ux) / (m->d0 - s * uy));
  }
  return worst;
}

static void testDiagonalRate(void)
{
  rig_t r;
  rigInit(&r);
  sceneAt(&r, 4.66f);
  CHECK(autoMotionMove(&r.m, &r.in, 150, 300, 0) == AUTO_MVW_NONE, "right 1.50 forward 3.00 at D 4.66: accepted");
  const float w1 = latRateMax(&r.m);
  printf("info: right 1.50 forward 3.00 at D 4.66: peak %.3f m/s, the largest sideways image rate %.4f /s\n",
         (double)r.m.ph.vPk, w1);
  CHECK(w1 <= AUTO_MV_W_LAT_MAX + 1e-4f, "its sideways image rate <= %.2f /s to its end (max %.4f, peak %.3f m/s)",
        (double)AUTO_MV_W_LAT_MAX, w1, (double)r.m.ph.vPk);
  /* where the cap at the forward end binds: right 1.50 forward 1.50 at D 3.0 (the end 1.5 m away) */
  rigInit(&r);
  sceneAt(&r, 3.0f);
  CHECK(autoMotionMove(&r.m, &r.in, 150, 150, 0) == AUTO_MVW_NONE, "right 1.50 forward 1.50 at D 3.0: accepted");
  const float w2 = latRateMax(&r.m);
  printf("info: right 1.50 forward 1.50 at D 3.0: peak %.3f m/s, the largest sideways image rate %.4f /s\n",
         (double)r.m.ph.vPk, w2);
  const float len = sqrtf(2.0f) * 1.5f;
  const float vOld = fminf(fminf(AUTO_MV_V_H_MAX, AUTO_MV_V_FWD_MAX * len / 1.5f),
                           fminf(AUTO_MV_W_LAT_MAX * 3.0f * len / 1.5f, AUTO_MV_W_FWD_MAX * 1.5f * len / 1.5f));
  autoMotion_t old = r.m;   /* the same move at the speed the cap at the start D allowed */
  autoMvProfPlan(&old.ph, len, vOld, AUTO_MV_A_H);
  const float wOld = latRateMax(&old);
  printf("info: ... with the sideways cap at the start D (before 2026-10-14): %.3f m/s, the rate up to %.4f /s (%.2f x "
         "the cap)\n", vOld, wOld, wOld / AUTO_MV_W_LAT_MAX);
  CHECK(w2 <= AUTO_MV_W_LAT_MAX + 1e-4f && r.m.ph.vPk < vOld - 0.01f,
        "its sideways image rate <= %.2f /s (max %.4f) at %.3f m/s; the cap at the start D allowed %.3f m/s, %.4f /s",
        (double)AUTO_MV_W_LAT_MAX, w2, (double)r.m.ph.vPk, vOld, wOld);
  /* every accepted diagonal toward the scene on a grid (D 1.5..6 m, right 0.1..1.5 m, forward 0.1..4 m): the
   * largest sideways image rate with the cap at the forward end, and what the cap at the start D allowed */
  float worstNew = 0.0f, worstOld = 0.0f, wd = 0.0f;
  int16_t wr = 0, wf = 0, nAcc = 0;
  for (int di = 0; di <= 18; di++) {
    const float d = 1.5f + 0.25f * (float)di;
    for (int16_t rc = 10; rc <= 150; rc += 20) {
      for (int16_t fc = 10; fc <= 400; fc += 15) {
        rigInit(&r);
        sceneAt(&r, d);
        if (autoMotionMove(&r.m, &r.in, rc, fc, 0) != AUTO_MVW_NONE) {
          continue;
        }
        nAcc++;
        worstNew = fmaxf(worstNew, latRateMax(&r.m));
        const float l = r.m.lenM, rm = rc * 0.01f, fm = fc * 0.01f;
        const float v0 = fminf(fminf(AUTO_MV_V_H_MAX, AUTO_MV_V_FWD_MAX * l / fm),
                               fminf(AUTO_MV_W_LAT_MAX * d * l / rm, AUTO_MV_W_FWD_MAX * fmaxf(d - fm, 0.05f) * l / fm));
        autoMotion_t o = r.m;
        autoMvProfPlan(&o.ph, l, v0, AUTO_MV_A_H);
        const float wo = latRateMax(&o);
        if (wo > worstOld) {
          worstOld = wo;
          wd = d;
          wr = rc;
          wf = fc;
        }
      }
    }
  }
  printf("info: %d accepted diagonals toward the scene: the largest sideways image rate %.4f /s (the cap %.2f); with "
         "the cap at the start D %.4f /s (%.2f x; right %d forward %d cm at D %.2f m)\n", nAcc, worstNew,
         (double)AUTO_MV_W_LAT_MAX, worstOld, worstOld / AUTO_MV_W_LAT_MAX, wr, wf, wd);
  CHECK(nAcc > 100 && worstNew <= AUTO_MV_W_LAT_MAX + 1e-4f, "every accepted diagonal: the sideways image rate <= "
        "%.2f /s to its end (max %.4f)", (double)AUTO_MV_W_LAT_MAX, worstNew);
}

/* The camera height observer's gain input (2026-10-15 review, docs/autonomy-a2b.md 21.9): autoMotionFwdLn is the
 * measured forward progress ln(D0 / D) of every MOVE since ARM, the running one's so far; a turn, a sideways move and
 * a hover keep it; a back move lowers it. */
static void testFwdLn(void)
{
  rig_t r;
  rigInit(&r);
  r.in.hTargetM = 1.0f;
  r.in.vzH = 1.0f;
  sceneAt(&r, 4.0f);
  CHECK(autoMotionFwdLn(&r.m) == 0.0f, "no move: 0");
  CHECK(autoMotionMove(&r.m, &r.in, 0, 200, 0) == AUTO_MVW_NONE, "forward 200 at D 4.0");
  steps(&r, 5000);
  const float mid = autoMotionFwdLn(&r.m);
  CHECK(mid > 0.05f && mid < logf(2.0f) && mid == r.m.progFwd, "during it: its measured progress so far (%.4f)", mid);
  CHECK(untilEnd(&r, 30000) > 0 && r.m.end == AUTO_MVE_DONE, "DONE");
  const float one = autoMotionFwdLn(&r.m);
  CHECK(fabsf(one - logf(4.0f / 2.0f)) < 0.002f, "after it: ln(4 / 2) = 0.6931 (%.4f)", one);
  steps(&r, 3000);
  CHECK(autoMotionFwdLn(&r.m) == one, "the hover after it keeps it");
  /* a sideways move: the forward progress stays (its own forward part is ~0) */
  sceneAt(&r, 2.0f);
  CHECK(autoMotionMove(&r.m, &r.in, 50, 0, 0) == AUTO_MVW_NONE && untilEnd(&r, 30000) > 0, "right 50: DONE");
  CHECK(fabsf(autoMotionFwdLn(&r.m) - one) < 1e-6f, "a sideways move keeps it (%.4f)", autoMotionFwdLn(&r.m));
  /* the next forward move adds its own */
  CHECK(autoMotionMove(&r.m, &r.in, 0, 50, 0) == AUTO_MVW_NONE && untilEnd(&r, 30000) > 0, "forward 50 at D 2.0: DONE");
  const float two = autoMotionFwdLn(&r.m);
  CHECK(fabsf(two - one - logf(2.0f / 1.5f)) < 0.002f, "a second forward move adds ln(2 / 1.5) (%.4f)", two - one);
  /* a back move lowers it; a turn keeps it */
  CHECK(autoMotionMove(&r.m, &r.in, 0, -50, 0) == AUTO_MVW_NONE && untilEnd(&r, 30000) > 0, "back 50: DONE");
  const float three = autoMotionFwdLn(&r.m);
  CHECK(three < two - 0.15f, "a back move lowers it (%.4f -> %.4f)", two, three);
  r.in.sceneTracked = true;
  CHECK(autoMotionYaw(&r.m, &r.in, 20) == AUTO_MVW_NONE, "YAW 20");
  steps(&r, 200);
  CHECK(autoMotionFwdLn(&r.m) == three, "a turn keeps it (%.4f)", autoMotionFwdLn(&r.m));
  /* a STOPped move keeps its travel */
  rigInit(&r);
  sceneAt(&r, 4.0f);
  CHECK(autoMotionMove(&r.m, &r.in, 0, 200, 0) == AUTO_MVW_NONE, "forward 200 again");
  steps(&r, 4000);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  const float stopped = autoMotionFwdLn(&r.m);
  steps(&r, 2000);
  CHECK(stopped > 0.02f && autoMotionFwdLn(&r.m) == stopped && r.m.state == AUTO_MV_STOPPED,
        "a STOPped move keeps its travel (%.4f)", stopped);
}

/* The TAKEOFF's planned move flies with the Mac's floor (AUTO_PLAN_MIN_FRONT_CM 60, 2026-10-15 review): the Mac
 * cannot check it before the take-off. Its largest forward move per D is the Mac's (espfly_auto.py fwd_max at
 * 0.60 m), 1 cm more is TOO_CLOSE; a Mac MOVE keeps the drone's 0.50 m. */
static void testPlanFloor(void)
{
  static const struct { float d; int16_t macMax, droneMax; } t[] = {
    {2.0f, 102, 112}, {2.5f, 143, 154}, {3.0f, 185, 197}, {3.5f, 228, 241}, {4.0f, 271, 285}, {4.66f, 329, 343},
    {5.0f, 359, 374},
  };
  int ok = 0;
  for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
    autoMvWhy_t w[4];
    const int16_t fs[4] = {t[i].macMax, (int16_t)(t[i].macMax + 1), t[i].droneMax, (int16_t)(t[i].droneMax + 1)};
    for (int j = 0; j < 4; j++) {
      rig_t r;
      rigInit(&r);
      r.in.hTargetM = 1.0f;
      r.in.vzH = 1.0f;
      sceneAt(&r, t[i].d);
      w[j] = j < 2 ? autoMotionMovePlan(&r.m, &r.in, 0, fs[j], 0) : autoMotionMove(&r.m, &r.in, 0, fs[j], 0);
    }
    const bool good = w[0] == AUTO_MVW_NONE && w[1] == AUTO_MVW_TOO_CLOSE && w[2] == AUTO_MVW_NONE &&
                      w[3] == AUTO_MVW_TOO_CLOSE;
    ok += good;
    if (!good) {
      printf("  D %.2f: plan %d %s, %d %s; move %d %s, %d %s\n", t[i].d, fs[0], autoMvWhyName(w[0]), fs[1],
             autoMvWhyName(w[1]), fs[2], autoMvWhyName(w[2]), fs[3], autoMvWhyName(w[3]));
    }
  }
  CHECK(ok == (int)(sizeof t / sizeof t[0]), "the plan's largest forward move is the Mac's (0.60 m at k 0.6: 1.02 m at "
        "D 2.0 .. 3.59 m at 5.0), a Mac MOVE the drone's (0.50 m: 1.12 .. 3.74 m); 1 cm more TOO_CLOSE (%d of %d D)",
        ok, (int)(sizeof t / sizeof t[0]));
}

/* ---- "Claude decides" (2026-10-15, docs/autonomy-a2b.md 22) ---------------------------------------------------- */

/* The height checks are one-sided: a move down against the floor only (an estimate above the top may move down), a
 * move up against the top only (from below the floor it may move up). */
static void testHeightOneSided(void)
{
  rig_t r;
  rigInit(&r);
  r.in.hTargetM = AUTO_MV_H_MAX_M;
  r.in.vzH = AUTO_MV_H_MAX_M + 0.20f;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, -20) == AUTO_MVW_NONE, "estimate %.2f m (the cap + 0.20): down 0.20 "
        "accepted", (double)r.in.vzH);
  rigInit(&r);
  r.in.hTargetM = AUTO_MV_H_MAX_M;
  r.in.vzH = AUTO_MV_H_MAX_M + 0.20f;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 1) == AUTO_MVW_HEIGHT, "and up 0.01 from there: HEIGHT");
  rigInit(&r);
  r.in.hTargetM = 0.20f;
  r.in.vzH = 0.20f;
  r.in.airMs = 60000;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 0, 20) == AUTO_MVW_NONE, "estimate 0.20 m at 60 s of air: up 0.20 accepted");
}

/* A YAW without a view: up to AUTO_MV_BLIND_YAW_MAX_DEG accepted (flag BLIND_TURN), its turn not paused by the view
 * guard; larger refused NO_VIEW; with a view nothing changes. */
static void testBlindTurn(void)
{
  rig_t r;
  rigInit(&r);
  r.in.holdApplied = false;
  CHECK(autoMotionYaw(&r.m, &r.in, 91) == AUTO_MVW_NO_VIEW && autoMotionYaw(&r.m, &r.in, -91) == AUTO_MVW_NO_VIEW,
        "no view: YAW 91 refused NO_VIEW");
  CHECK(autoMotionYaw(&r.m, &r.in, -90) == AUTO_MVW_NONE && r.m.blind, "no view: YAW -90 accepted (blind)");
  bool flag = true, paused = false;
  int n = 0;
  while (!r.m.turnDone && n < 10000) {
    step(&r);
    flag = flag && (r.out.flags & AUTO_MVF_BLIND_TURN) != 0;
    paused = paused || r.m.state == AUTO_MV_PAUSED;
    n++;
  }
  CHECK(r.m.turnDone && flag && !paused && fabsf(r.out.yawOffDeg + 90.0f) < 1e-3f && r.land == AUTO_MVL_NONE,
        "the blind turn: never paused, flag 0x40, -90 deg in %d ms, no stop request", n);
  r.in.holdApplied = true;   /* the view regained after the turn */
  n = untilEnd(&r, 20000);
  CHECK(n > 0 && r.m.end == AUTO_MVE_DONE, "the view back: the dwell, DONE (%d ms)", n);
  rigInit(&r);
  CHECK(autoMotionYaw(&r.m, &r.in, 120) == AUTO_MVW_NONE && !r.m.blind && !(r.out.flags & AUTO_MVF_BLIND_TURN),
        "with a view: YAW 120 a normal turn");
  /* a blind turn whose dwell finds no view (nor a tracked scene): its bob cannot run, the view guard (2 s) turns it
   * back, then it ends NO_VIEW */
  rigInit(&r);
  r.in.holdApplied = false;
  r.in.sceneTracked = false;
  autoMotionYaw(&r.m, &r.in, 60);
  n = 0;
  int requests = 0;
  while (active(&r) && n < 30000) {
    step(&r);
    if (r.land == AUTO_MVL_NO_CAM) {
      requests++;
      autoMotionStop(&r.m, &r.in, AUTO_MVE_NO_VIEW);   /* as the state machine */
    }
    n++;
  }
  CHECK(!active(&r) && requests == 2 && r.m.end == AUTO_MVE_NO_VIEW && fabsf(r.out.yawOffDeg) < 1e-3f,
        "no view after a blind turn: turned back on the first request, STOPPED NO_VIEW on the second (%d requests, "
        "heading %.2f)", requests, r.out.yawOffDeg);
}

/* The state machine's stops: CONTACT on a MOVE (snap, brake), on a YAW (back to its start heading, the same seq, end
 * UNDONE once re-tracked); TILT and LOOP_GAP on a YAW end its turn where it is and keep the dwell and bob; a snap
 * without a motion. */
static void testStopKinds(void)
{
  rig_t r;
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 0, 80, 0);
  steps(&r, 3000);
  const uint32_t e0 = r.m.epoch;
  autoMotionStop(&r.m, &r.in, AUTO_MVE_CONTACT);
  CHECK(!active(&r) && r.m.end == AUTO_MVE_CONTACT && r.m.state == AUTO_MV_STOPPED && r.m.epoch == e0 + 1 &&
            r.m.snap && r.m.brakeS > 0.0f, "CONTACT on a MOVE: STOPPED CONTACT, a snap epoch and a brake");
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, 90);
  steps(&r, 2000);
  const uint16_t seq = r.m.seq;
  const float mid = r.out.yawOffDeg;
  autoMotionStop(&r.m, &r.in, AUTO_MVE_CONTACT);
  step(&r);
  CHECK(active(&r) && r.m.undo && (r.out.flags & AUTO_MVF_UNDO) && r.m.seq == seq && mid > 10.0f,
        "CONTACT in a YAW at %.1f deg: turning back, the same seq", mid);
  int n = untilEnd(&r, 30000);
  CHECK(n > 0 && r.m.end == AUTO_MVE_UNDONE && r.m.state == AUTO_MV_DONE && fabsf(r.out.yawOffDeg) < 1e-3f,
        "... back at 0 deg and re-tracked: DONE UNDONE (%d ms)", n);
  for (int k = 0; k < 2; k++) {
    rigInit(&r);
    autoMotionYaw(&r.m, &r.in, 90);
    steps(&r, 1500);
    autoMotionStop(&r.m, &r.in, k ? AUTO_MVE_LOOP_GAP : AUTO_MVE_TILT);
    step(&r);
    const bool kept = active(&r) && r.m.turnDone && r.out.camGuessOk && r.m.state == AUTO_MV_SETTLE;
    autoMotionStop(&r.m, &r.in, k ? AUTO_MVE_LOOP_GAP : AUTO_MVE_TILT);   /* again, in the dwell */
    step(&r);
    CHECK(kept && active(&r) && r.out.camGuessOk && !r.m.undo, "%s in a YAW: its turn ends where it is, the dwell "
          "and the camera guess kept, also on a second one", k ? "LOOP_GAP" : "TILT");
  }
  /* the sim (DRY / BENCH): no turn back */
  rigInit(&r);
  r.in.flight = false;
  r.in.simPlant = true;
  autoMotionYaw(&r.m, &r.in, 90);
  steps(&r, 1500);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_CONTACT);
  CHECK(!active(&r) && r.m.end == AUTO_MVE_CONTACT, "on the sim a CONTACT ends the YAW (STOPPED)");
  /* a snap without a motion: a new snap epoch, references 0 */
  rigInit(&r);
  autoMotionMove(&r.m, &r.in, 40, 0, 0);
  untilEnd(&r, 20000);
  step(&r);
  const bool keptEnd = r.out.refLat != 0.0f;
  const uint32_t e1 = r.m.epoch;
  autoMotionSnap(&r.m);
  step(&r);
  CHECK(keptEnd && r.m.epoch == e1 + 1 && r.out.snap && r.out.snapEpoch == r.m.epoch && r.out.refLat == 0.0f &&
            !active(&r), "autoMotionSnap: a snap epoch, the finished MOVE's end no longer the reference");
}

/* A MOVE stopped by a contact (2026-10-15 verification): the hold's anchor AUTO_MV_CONTACT_BACK_M back along the move
 * (the way it came), in the stop's snap epoch, the reference until the next motion or snap; not for a STOP, a pure
 * vertical move or a YAW; a back move's anchor keeps AUTO_MOVE_MIN_FRONT_CM to the scene. */
static void testContactBackOff(void)
{
  static const struct { int16_t r, f, u; bool contact; const char *what; } c[] = {
    {0, 80, 0, true, "forward 80"}, {60, 0, 0, true, "right 60"}, {-60, 60, 0, true, "left 60 forward 60"},
    {0, -60, 0, true, "back 60"}, {0, 0, 20, true, "up 20"}, {0, 80, 0, false, "forward 80, a STOP"},
    {0, 30, 0, true, "forward 30, hit 0.6 s in"},
  };
  for (unsigned k = 0; k < sizeof c / sizeof c[0]; k++) {
    rig_t r;
    rigInit(&r);
    CHECK(autoMotionMove(&r.m, &r.in, c[k].r, c[k].f, c[k].u) == AUTO_MVW_NONE, "%s accepted", c[k].what);
    steps(&r, (c[k].u != 0 || c[k].f == 30) ? 600 : 2500);
    CHECK(active(&r), "%s: running", c[k].what);
    const float dNow = r.m.d0 * expf(-r.m.progFwd), dLat = r.m.dLat0 * expf(-r.m.progFwd);
    const float ux = r.m.lenM > 0.0f ? r.m.latM / r.m.lenM : 0.0f, uy = r.m.lenM > 0.0f ? r.m.fwdM / r.m.lenM : 0.0f;
    const float travel = ux * r.m.progLat * r.m.dLat0 + uy * r.m.d0 * (1.0f - expf(-r.m.progFwd));
    const float back = AUTO_MV_CONTACT_BACK_M;   /* (not capped at the travel: behind the start by up to it) */
    const uint32_t e0 = r.m.epoch;
    autoMotionStop(&r.m, &r.in, c[k].contact ? AUTO_MVE_CONTACT : AUTO_MVE_STOP_CMD);
    step(&r);
    const bool horiz = c[k].r != 0 || c[k].f != 0;
    if (!c[k].contact || !horiz) {
      CHECK(!r.m.backOff && r.out.refLat == 0.0f && r.out.refFwd == 0.0f && r.m.epoch == e0 + 1,
            "%s: the hold anchored where it stopped (no back-off)", c[k].what);
      continue;
    }
    /* the anchor in metres from the stop point: right, forward */
    const float aR = r.out.refLat * dLat, aF = dNow - dNow / expf(r.out.refFwd);   /* D_anchor = D_now / e^ref */
    const float wantF = uy > 0.0f ? -back * uy : fminf(-back * uy, fmaxf(0.0f, dNow - 0.50f));
    CHECK(r.m.backOff && r.out.snap && r.m.epoch == e0 + 1 && fabsf(aR + back * ux) < 0.005f &&
              fabsf(aF - wantF) < 0.005f && (c[k].f != 30 || travel < 0.10f),
          "%s CONTACT: the anchor %.3f m right, %.3f m forward of the stop (want %.3f, %.3f: %.3f m back along the "
          "move, its travel %.3f m)", c[k].what, aR, aF, -back * ux, wantF, back, travel);
    const float rl = r.out.refLat, rf = r.out.refFwd;
    steps(&r, 5000);
    autoMvOut_t idle;
    autoMotionIdleOut(&r.m, &idle);
    CHECK(r.out.refLat == rl && r.out.refFwd == rf && idle.refLat == rl && idle.refFwd == rf && !active(&r),
          "%s: the anchor stays the reference 5 s on, and outside HOVER (LAND)", c[k].what);
    if (k == 0) {
      CHECK(autoMotionYaw(&r.m, &r.in, 30) == AUTO_MVW_NONE && !r.m.backOff, "a new motion ends the back-off");
    } else {
      autoMotionSnap(&r.m);
      step(&r);
      CHECK(!r.m.backOff && r.out.refLat == 0.0f && r.out.refFwd == 0.0f, "a snap (a later contact) ends it");
    }
  }
}

/* ---- fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31) ------------------------------------------------------ */

/* auto_motion_identity.h's script hashed on 7ab74b9's auto_motion.c (this file's build line: -O2, Apple clang; the
 * float contraction makes -O0 differ), the motions before the speed step. */
#define AUTO_MV_ID_HASH_7AB74B9 0x47E10B48658F1BE0ull

/* The guard margins of before 2026-10-17 (auto_motion.h "guard margins"): global, set before autoMotionInit's copy. */
static void setOldMargins(autoMotion_t *m)
{
  (void)m;
  autoMvMargins_t g;
  autoMvMarginsOld(&g);
  autoMotionSetMargins(&g);
}

static void setSpeed100Old(autoMotion_t *m)
{
  setOldMargins(m);
  autoMotionSetSpeed(m, 100);
}

static void testSpeedIdentity(void)
{
  /* the margins of before (autoMvMarginsOld) reproduce 7ab74b9 bit for bit: the margins are the only change of
   * 2026-10-17 at 100; the defaults' hash is information (the bolder margins change TOO_CLOSE and the guards) */
  autoMotionSetMargins(NULL);
  const uint64_t hNew = autoMvIdentityHash(NULL);
  const uint64_t h0 = autoMvIdentityHash(setOldMargins), h1 = autoMvIdentityHash(setSpeed100Old);
  autoMotionSetMargins(NULL);
  printf("info: the motions' identity script: 0x%016llX (Init, the margins of before), 0x%016llX (SetSpeed 100), "
         "7ab74b9 0x%016llX; the default margins of 2026-10-17 0x%016llX\n", (unsigned long long)h0,
         (unsigned long long)h1, (unsigned long long)AUTO_MV_ID_HASH_7AB74B9, (unsigned long long)hNew);
  CHECK(h0 == AUTO_MV_ID_HASH_7AB74B9 && h1 == AUTO_MV_ID_HASH_7AB74B9 && hNew != AUTO_MV_ID_HASH_7AB74B9,
        "mvspeed 100 with the margins of before: every output, answer and the motion memory bit-identical to 7ab74b9 "
        "over the identity script (autoMotionInit alone and with autoMotionSetSpeed(100)); the defaults differ");
  autoMotion_t m;
  autoMotionInit(&m);
  CHECK(m.speedPct == 100 && m.settleMaxMs == AUTO_MV_SETTLE_MAX_MS && m.dwellMs == AUTO_MV_YAW_DWELL_MS &&
        m.brakeAcc == AUTO_MV_BRAKE_ACC && m.frozenRef == AUTO_MV_FROZEN_REF, "autoMotionInit: the header's limits");
}

/* The feedforward roll per forward speed (mvffr, autoMotionSetFfRoll; 2026-10-22 "the left slide"): a forward move with
 * 20 (2.0 deg per m/s) leans right by 2.0 x the reference forward speed on top of the plain feedforward in RUN, nothing
 * in SETTLE; a back move leans left; a pure sideways move is untouched; every other output is the same; 0 restores the
 * identity hash; the setting is clamped. */
static void testFfRoll(void)
{
  static const int16_t mv[3][2] = {{0, 150}, {0, -100}, {80, 0}};
  static const char *const what[3] = {"forward 1.50 at D 4.0", "back 1.00", "right 0.80"};
  autoMotionSetFfRoll(20);
  CHECK(autoMotionGetFfRoll() == 20, "mvffr 20 set (%d)", autoMotionGetFfRoll());
  for (int k = 0; k < 3; k++) {
    rig_t a, b;
    rigInit(&a);
    rigInit(&b);
    a.in.sceneDM = b.in.sceneDM = 4.0f;
    autoMotionSetFfRoll(0);
    const autoMvWhy_t wa = autoMotionMove(&a.m, &a.in, mv[k][0], mv[k][1], 0);
    autoMotionSetFfRoll(20);
    const autoMvWhy_t wb = autoMotionMove(&b.m, &b.in, mv[k][0], mv[k][1], 0);
    CHECK(wa == AUTO_MVW_NONE && wb == AUTO_MVW_NONE, "%s accepted with and without", what[k]);
    float dMax = 0.0f, dMin = 0.0f, dSettle = 0.0f;
    bool same = true;
    int n = 0;
    while (active(&a) && active(&b) && n < 60000) {
      autoMotionSetFfRoll(0);    /* (read at every step, not at the acceptance) */
      step(&a);
      autoMotionSetFfRoll(20);
      step(&b);
      n++;
      const float d = b.out.ffRollDeg - a.out.ffRollDeg;
      if (b.m.state == AUTO_MV_RUN) {
        dMax = fmaxf(dMax, d);
        dMin = fminf(dMin, d);
      } else {
        dSettle = fmaxf(dSettle, fabsf(d));
      }
      same = same && a.out.refLat == b.out.refLat && a.out.refFwd == b.out.refFwd && a.out.rateFwd == b.out.rateFwd &&
             a.out.rateLat == b.out.rateLat && a.out.ffPitchDeg == b.out.ffPitchDeg && a.m.state == b.m.state;
    }
    CHECK(same && !active(&a) && !active(&b) && a.m.state == AUTO_MV_DONE && b.m.state == AUTO_MV_DONE,
          "%s: the references, rates, pitch feedforward and states the same with mvffr 20 and 0 (DONE)", what[k]);
    if (k == 0) {
      /* the forward reference speed peaks at 0.20 m/s here (the 100 % profile), at most AUTO_MV_V_H_MAX -> +0.40 deg */
      CHECK(dMax > 2.0f * 0.15f && dMax <= 2.0f * AUTO_MV_V_H_MAX + 1e-4f && dMin >= 0.0f && dSettle == 0.0f,
            "forward: the roll feedforward +%.3f deg more at the top speed (2.0 deg per m/s x the reference speed, at "
            "most %.2f m/s), never less, 0 in SETTLE", dMax, (double)AUTO_MV_V_H_MAX);
    } else if (k == 1) {
      CHECK(dMin < -2.0f * 0.15f && dMax <= 0.0f && dSettle == 0.0f, "back: leans left, %.3f deg at most, 0 in SETTLE",
            dMin);
    } else {
      CHECK(dMax == 0.0f && dMin == 0.0f && dSettle == 0.0f, "sideways: the roll feedforward untouched");
    }
  }
  autoMotionSetFfRoll(99);
  CHECK(autoMotionGetFfRoll() == AUTO_MV_FFR_MAX_DDEG, "99 clamped to %d", AUTO_MV_FFR_MAX_DDEG);
  autoMotionSetFfRoll(-99);
  CHECK(autoMotionGetFfRoll() == AUTO_MV_FFR_MIN_DDEG, "-99 clamped to %d", AUTO_MV_FFR_MIN_DDEG);
  autoMotionSetFfRoll(0);
  CHECK(autoMotionGetFfRoll() == 0 && autoMvIdentityHash(setOldMargins) == AUTO_MV_ID_HASH_7AB74B9,
        "mvffr 0 again: the identity hash of 7ab74b9 stands");
  autoMotionSetMargins(NULL);
}

static bool near(float a, float b)
{
  return fabsf(a - b) <= 1e-5f * fmaxf(1.0f, fabsf(b));
}

static void testSpeedCfg(void)
{
  static const struct {
    uint16_t pct;
    float vH, vBack, vFwd, aH, wLat, wFwd, brake;
    uint16_t settle, dwell;
    float yawRate, yawAcc, bob, frozen;
  } t[] = {
    {100, 0.30f, 0.30f, 0.20f, 0.10f, 0.12f, 0.15f, 0.15f, 3000, 500, 30.0f, 60.0f, 0.20f, 0.10f},
    /* (after the review of cbccec5: V_H cap 0.60 -> 0.40, W_FWD 0.20 -> 0.17, yaw 40 / 120 -> 35 / 70, bob 0.40 ->
     * 0.20) */
    /* (after the 300 % forensics of 2026-10-17: A_H cap 0.30 -> 0.20, the settle at least 2500 ms, 1000 before) */
    {150, 0.40f, 0.40f, 0.30f, 0.15f, 0.14f, 0.17f, 0.225f, 2500, 333, 35.0f, 70.0f, 0.20f, 0.168f},
    {200, 0.40f, 0.40f, 0.40f, 0.20f, 0.14f, 0.17f, 0.30f, 2500, 300, 35.0f, 70.0f, 0.20f, 0.168f},
    {250, 0.40f, 0.40f, 0.50f, 0.20f, 0.14f, 0.17f, 0.375f, 2500, 300, 35.0f, 70.0f, 0.20f, 0.168f},
    {300, 0.40f, 0.40f, 0.50f, 0.20f, 0.14f, 0.17f, 0.40f, 2500, 300, 35.0f, 70.0f, 0.20f, 0.168f},
  };
  for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
    autoMvSpeed_t c;
    autoMvSpeedCfg(t[i].pct, &c);
    CHECK(c.pct == t[i].pct && c.fast == (t[i].pct > 100) && near(c.vH, t[i].vH) && near(c.vBack, t[i].vBack) &&
              near(c.vFwd, t[i].vFwd) && near(c.aH, t[i].aH) && near(c.wLat, t[i].wLat) && near(c.wFwd, t[i].wFwd) &&
              near(c.brake, t[i].brake) && c.settleMaxMs == t[i].settle && c.dwellMs == t[i].dwell &&
              near(c.yawRate, t[i].yawRate) && near(c.yawAcc, t[i].yawAcc) && near(c.bobAcc, t[i].bob) &&
              near(c.frozenRef, t[i].frozen) && c.tTrack == (t[i].pct > 100 ? AUTO_MV_FAST_T_TRACK_S : 0.0f),
          "mvspeed %u: V_H %.3f V_BACK %.3f V_FWD %.3f A_H %.3f W_LAT %.3f W_FWD %.3f BRAKE %.3f settle %u dwell %u yaw "
          "%.0f/%.0f bob %.2f frozen %.3f", t[i].pct, c.vH, c.vBack, c.vFwd, c.aH, c.wLat, c.wFwd, c.brake,
          c.settleMaxMs, c.dwellMs, c.yawRate, c.yawAcc, c.bobAcc, c.frozenRef);
  }
  autoMvSpeed_t a, b;
  autoMvSpeedCfg(100, &a);
  CHECK(a.vH == AUTO_MV_V_H_MAX && a.vFwd == AUTO_MV_V_FWD_MAX && a.aH == AUTO_MV_A_H && a.wLat == AUTO_MV_W_LAT_MAX &&
        a.wFwd == AUTO_MV_W_FWD_MAX && a.brake == AUTO_MV_BRAKE_ACC && a.yawRate == AUTO_MV_YAW_RATE_DPS &&
        a.yawAcc == AUTO_MV_YAW_ACC_DPS2 && a.bobAcc == AUTO_MV_BOB_ACC && a.frozenRef == AUTO_MV_FROZEN_REF,
        "mvspeed 100: the header constants themselves");
  const uint16_t lows[] = {0, 1, 50, 99};
  for (int i = 0; i < 4; i++) {
    autoMvSpeedCfg(lows[i], &b);
    CHECK(memcmp(&a, &b, sizeof(a)) == 0, "mvspeed %u: as 100", lows[i]);
  }
  autoMvSpeedCfg(300, &a);
  const uint16_t highs[] = {301, 400, 65535};
  for (int i = 0; i < 3; i++) {
    autoMvSpeedCfg(highs[i], &b);
    CHECK(memcmp(&a, &b, sizeof(a)) == 0, "mvspeed %u: as 300", highs[i]);
  }
  autoMotion_t m;
  autoMotionInit(&m);
  autoMotionSetSpeed(&m, 200);
  CHECK(m.speedPct == 200 && m.settleMaxMs == 2500 && m.dwellMs == 300 && near(m.brakeAcc, 0.30f) &&
        near(m.frozenRef, 0.168f) && !m.viewFastArm, "autoMotionSetSpeed(200): the step's fields");
  autoMotionSetSpeed(&m, 1000);
  CHECK(m.speedPct == 300 && m.settleMaxMs == 2500, "autoMotionSetSpeed(1000): clamped to 300");
}

/* A rig at the speed step pct, the scene d ahead. */
static void rigSpeed(rig_t *r, uint16_t pct, float d)
{
  rigInit(r);
  autoMotionSetSpeed(&r->m, pct);
  r->in.sceneDM = d;
}

/* The largest forward image rate v uy / (D - s uy) along a MOVE's profile (D the planning distance). */
static float fwdRateMax(const autoMotion_t *m, float d)
{
  const float uy = m->fwdM / m->lenM;
  float w = 0.0f;
  for (float t = 0.0f; t <= m->ph.T + 1e-3f; t += 0.001f) {
    float s, v, a;
    autoMvProfEval(&m->ph, t, &s, &v, &a);
    w = fmaxf(w, v * uy / (d - s * uy));
  }
  return w;
}

static void testPeakRule(void)
{
  static const float Ls[] = {0.2f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 4.0f};
  static const float Ds[] = {1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
  static const uint16_t ps[] = {101, 120, 150, 200, 250, 300};
  static const int16_t lats[] = {0, 40};
  int n = 0, okMax = 0, bind = 0, okBind = 0;
  float worst = 0.0f;
  for (unsigned ip = 0; ip < sizeof(ps) / sizeof(ps[0]); ip++) {
    autoMvSpeed_t c;
    autoMvSpeedCfg(ps[ip], &c);
    for (unsigned il = 0; il < 7; il++) {
      for (unsigned id = 0; id < 7; id++) {
        for (unsigned ia = 0; ia < 2; ia++) {
          rig_t r;
          rigSpeed(&r, ps[ip], Ds[id]);
          if (autoMotionMove(&r.m, &r.in, lats[ia], (int16_t)lroundf(Ls[il] * 100.0f), 0) != AUTO_MVW_NONE) {
            continue;
          }
          n++;
          const float w = fwdRateMax(&r.m, Ds[id]);
          worst = fmaxf(worst, w - c.wFwd);
          okMax += w <= c.wFwd + 1e-4f;
          /* binding: the peak is under every other cap (speed, the triangle, the sideways image rate) */
          const float uy = r.m.fwdM / r.m.lenM, ux = r.m.latM / r.m.lenM;
          const float other = fminf(fminf(c.vH, c.vFwd / uy), sqrtf(c.aH * r.m.lenM));
          const float latCap = ux != 0.0f ? c.wLat * (Ds[id] - r.m.fwdM) / fabsf(ux) : 1e9f;
          if (r.m.ph.vPk < fminf(other, latCap) - 1e-3f) {
            bind++;
            okBind += w >= 0.99f * c.wFwd;
          }
        }
      }
    }
  }
  printf("info: the forward peak rule: %d accepted moves (L 0.2-4 m, D 1-6 m, pct 101-300, straight and 40 cm "
         "sideways): the profile's largest v / (D - s) at most %+.2e over W_FWD; it binds on %d\n", n, worst, bind);
  CHECK(n > 300 && okMax == n, "the forward image rate <= W_FWD + 1e-4 along every profile (%d/%d)", okMax, n);
  CHECK(bind > 20 && okBind == bind, "where the peak rule binds, the profile reaches W_FWD within 1 %% (%d/%d)", okBind,
        bind);
  /* the closed form: no cap when sqrt(a / (2 De)) <= w */
  CHECK(autoMvFwdPeakCap(3.0f, 1.0f, 0.10f, 0.20f) > 1e29f && autoMvFwdPeakCap(1.5f, 1.0f, 0.30f, 0.20f) < 0.5f,
        "autoMvFwdPeakCap: none far from the scene, a cap near it");
}

static void testSpeedTimes(void)
{
  static const struct {
    uint16_t pct;
    float fwd1, side, yaw90, yaw180, bob;
  } t[] = {
    {100, 7.00f, 4.899f, 3.50f, 6.50f, 1.789f},
    /* (after the review of cbccec5: W_FWD 0.17, yaw 35 deg/s and 70 deg/s^2, the bob as at 100) */
    {150, 5.333f, 4.00f, 3.071f, 5.643f, 1.789f},
    {200, 4.50f, 3.464f, 3.071f, 5.643f, 1.789f},
    /* (after the 300 % forensics of 2026-10-17: A_H at most 0.20, 250 and 300 as 200; 4.118 / 3.114 and 3.894 / 2.881
     * s before) */
    {250, 4.50f, 3.464f, 3.071f, 5.643f, 1.789f},
    {300, 4.50f, 3.464f, 3.071f, 5.643f, 1.789f},
  };
  for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
    rig_t r;
    rigSpeed(&r, t[i].pct, 3.0f);
    autoMotionMove(&r.m, &r.in, 0, 100, 0);
    const float f1 = r.m.ph.T, bob = 2.0f * r.m.pb.T;   /* a cycle: two legs */
    rigSpeed(&r, t[i].pct, 2.5f);
    autoMotionMove(&r.m, &r.in, 60, 0, 0);
    const float side = r.m.ph.T;
    rigSpeed(&r, t[i].pct, 2.5f);
    autoMotionYaw(&r.m, &r.in, 90);
    const float y90 = r.m.py.T;
    rigSpeed(&r, t[i].pct, 2.5f);
    autoMotionYaw(&r.m, &r.in, -180);
    const float y180 = r.m.py.T;
    CHECK(fabsf(f1 - t[i].fwd1) < 0.01f && fabsf(side - t[i].side) < 0.01f && fabsf(y90 - t[i].yaw90) < 0.01f &&
              fabsf(y180 - t[i].yaw180) < 0.01f && fabsf(bob - t[i].bob) < 0.01f,
          "mvspeed %u: 1 m forward at D 3 %.3f s, 0.6 m sideways at D 2.5 %.3f s, turn 90 %.3f s, 180 %.3f s, bob "
          "cycle %.3f s", t[i].pct, f1, side, y90, y180, bob);
  }
  /* the settle and the dwell end at the step's values (a camera that never settles: the timeout) */
  for (int k = 0; k < 2; k++) {
    rig_t r;
    rigSpeed(&r, k ? 200 : 100, 2.5f);
    autoMotionMove(&r.m, &r.in, 40, 0, 0);
    /* the camera sees 70 % of the motion: short of the settle tolerance, the settle times out */
    for (int i = 0; i < 20000 && r.m.state != AUTO_MV_SETTLE; i++) {
      step(&r);
      r.in.progLat = r.out.refLat * 0.7f;
    }
    const uint32_t t0 = r.in.nowMs;
    for (int i = 0; i < 20000 && active(&r); i++) {
      step(&r);
      r.in.progLat = r.m.latTgt * 0.7f;
    }
    const uint32_t settleMs = r.in.nowMs - t0;
    CHECK(settleMs + 1u >= (k ? 2500u : 3000u) && settleMs <= (k ? 2500u : 3000u) + 1u && r.m.end == AUTO_MVE_SHORT,
          "mvspeed %d: the settle times out after %u ms (SHORT)", k ? 200 : 100, (unsigned)settleMs);
    rigSpeed(&r, k ? 200 : 100, 2.5f);
    r.in.flight = false;   /* the tracker keeps its G: the dwell, then DONE */
    autoMotionYaw(&r.m, &r.in, 10);
    for (int i = 0; i < 20000 && r.m.state != AUTO_MV_SETTLE; i++) {
      step(&r);
    }
    const uint32_t t1 = r.in.nowMs;
    untilEnd(&r, 5000);
    const uint32_t dwellMs = r.in.nowMs - t1;
    CHECK(dwellMs + 1u >= (k ? 300u : 500u) && dwellMs <= (k ? 300u : 500u) + 1u && r.m.state == AUTO_MV_DONE,
          "mvspeed %d: the yaw dwell %u ms", k ? 200 : 100, (unsigned)dwellMs);
  }
}

/* The VIEW_FAST timer as a setting (NVS mvview, auto_motion.h "fast moves", 2026-10-22): the profile's as initialised
 * and at 0, never shorter than it (1..profile - 1 clamp up), at most 1000; a MOVE's pause runs to the value in force
 * (recovers inside it, stopped VIEW_FAST one step past it). First in main: it checks the value as initialised. */
static void testViewDeb(void)
{
  CHECK(autoMotionGetViewDeb() == AUTO_MV_FAST_VIEW_DEB_MS, "mvview: the profile's %u ms as initialised (%u)",
        (unsigned)AUTO_MV_FAST_VIEW_DEB_MS, autoMotionGetViewDeb());
  autoMotionSetViewDeb(1);
  const uint16_t a = autoMotionGetViewDeb();
  autoMotionSetViewDeb((int32_t)AUTO_MV_FAST_VIEW_DEB_MS - 1);
  const uint16_t b = autoMotionGetViewDeb();
  autoMotionSetViewDeb(-5);
  const uint16_t c = autoMotionGetViewDeb();
  autoMotionSetViewDeb(1001);
  const uint16_t d = autoMotionGetViewDeb();
  autoMotionSetViewDeb(1000);
  const uint16_t e = autoMotionGetViewDeb();
  autoMotionSetViewDeb(0);
  CHECK(a == AUTO_MV_FAST_VIEW_DEB_MS && b == AUTO_MV_FAST_VIEW_DEB_MS && c == AUTO_MV_FAST_VIEW_DEB_MS && d == 1000 &&
            e == 1000 && autoMotionGetViewDeb() == AUTO_MV_FAST_VIEW_DEB_MS,
        "mvview 1, profile - 1 and -5: the profile's (never shorter); 1001: 1000; 0: the profile's again");
  /* 200, mvview 400: a 1.5 m approach's view lost at 2.5 s stays PAUSED past the profile's timer, recovers at 300 ms
   * and ends DONE; lost for good: stopped VIEW_FAST at 401 ms, its brake as at the profile's timer */
  autoMotionSetViewDeb(400);
  rig_t r;
  rigSpeed(&r, 200, 4.0f);
  CHECK(autoMotionMove(&r.m, &r.in, 0, 150, 0) == AUTO_MVW_NONE, "mvview 400: forward 1.5 m at D 4 accepted");
  steps(&r, 2500);
  r.in.holdApplied = false;
  steps(&r, 300);
  const bool paused = r.m.state == AUTO_MV_PAUSED && r.land == AUTO_MVL_NONE;
  r.in.holdApplied = true;
  step(&r);
  CHECK(paused && r.m.state == AUTO_MV_RUN && r.land == AUTO_MVL_NONE,
        "mvview 400: the view lost 300 ms (past the profile's %u): PAUSED, then RUN again", (unsigned)AUTO_MV_FAST_VIEW_DEB_MS);
  untilEnd(&r, 20000);
  CHECK(r.m.state == AUTO_MV_DONE, "... and the move ends DONE (%s)", autoMvEndName(r.m.end));
  rigSpeed(&r, 200, 4.0f);
  autoMotionMove(&r.m, &r.in, 0, 150, 0);
  steps(&r, 2500);
  float s, v, acc;
  autoMvProfEval(&r.m.ph, r.m.tRun, &s, &v, &acc);
  r.in.holdApplied = false;
  int deb = 0;
  while (r.land == AUTO_MVL_NONE && deb < 1000) {
    step(&r);
    deb++;
  }
  CHECK(r.land == AUTO_MVL_FAST && deb == 401 && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_VIEW_FAST &&
            fabsf(r.m.brakeS - v / 0.30f) < 0.01f,
        "mvview 400: lost for good: STOPPED VIEW_FAST after %d ms, brake %.2f s (v / 0.30)", deb, r.m.brakeS);
  autoMotionSetViewDeb(0);
  CHECK(autoMotionGetViewDeb() == AUTO_MV_FAST_VIEW_DEB_MS, "mvview 0 again: the profile's");
}

static void testViewFast(void)
{
  /* 200: the view lost 2.5 s into a 1.5 m approach (the reference at 0.40 m/s): PAUSED as at 100; still lost
   * AUTO_MV_FAST_VIEW_DEB_MS later: stopped, VIEW_FAST, braking (review of cbccec5: no longer on the first lost step) */
  rig_t r;
  rigSpeed(&r, 200, 4.0f);
  CHECK(autoMotionMove(&r.m, &r.in, 0, 150, 0) == AUTO_MVW_NONE, "200: forward 1.5 m at D 4 accepted");
  steps(&r, 2500);
  float s, v, a;
  autoMvProfEval(&r.m.ph, r.m.tRun, &s, &v, &a);
  r.in.holdApplied = false;
  step(&r);
  CHECK(r.land == AUTO_MVL_NONE && r.m.state == AUTO_MV_PAUSED && r.m.viewFastArm,
        "200: the view lost at %.2f m/s: PAUSED first, the VIEW_FAST check armed", v);
  int deb = 1;
  while (r.land == AUTO_MVL_NONE && deb < 1000) {
    step(&r);
    deb++;
  }
  CHECK(r.land == AUTO_MVL_FAST && deb == AUTO_MV_FAST_VIEW_DEB_MS + 1 && r.m.state == AUTO_MV_STOPPED &&
            r.m.end == AUTO_MVE_VIEW_FAST && r.m.snap && fabsf(r.m.brakeS - v / 0.30f) < 0.01f &&
            v > AUTO_MV_FAST_PAUSE_V && !r.m.viewFastArm,
        "... still lost %d ms later: AUTO_MVL_FAST, STOPPED VIEW_FAST, a snap, brake %.2f s (v / 0.30)", deb - 1,
        r.m.brakeS);
  CHECK(r.out.ffPitchDeg > 1.7f && r.out.ffPitchDeg < 1.8f && r.out.ffRollDeg == 0.0f && !r.out.holdWait,
        "... the brake lean on the same step: nose up %.2f deg (0.30 m/s^2)", r.out.ffPitchDeg);
  int n = 0;
  while (r.out.ffPitchDeg > 0.0f && n < 5000) {
    step(&r);
    n++;
  }
  CHECK(fabsf((float)n * 0.001f - v / 0.30f) < 0.01f, "... for %.2f s", (float)n * 0.001f);
  CHECK(strcmp(autoMvEndName(AUTO_MVE_VIEW_FAST), "VIEW_FAST") == 0, "autoMvEndName(VIEW_FAST)");
  /* 200: a loss shorter than AUTO_MV_FAST_VIEW_DEB_MS (a refused frame or two) pauses, then runs on */
  rigSpeed(&r, 200, 4.0f);
  autoMotionMove(&r.m, &r.in, 0, 150, 0);
  steps(&r, 2500);
  r.in.holdApplied = false;
  steps(&r, AUTO_MV_FAST_VIEW_DEB_MS - 10);
  const bool pausedShort = r.m.state == AUTO_MV_PAUSED && r.land == AUTO_MVL_NONE;
  r.in.holdApplied = true;
  step(&r);
  CHECK(pausedShort && r.m.state == AUTO_MV_RUN && r.land == AUTO_MVL_NONE,
        "200: the view lost %d ms in the cruise: PAUSED, then RUN again (no VIEW_FAST)", AUTO_MV_FAST_VIEW_DEB_MS - 10);
  untilEnd(&r, 20000);
  CHECK(r.m.state == AUTO_MV_DONE, "... and the move ends DONE (%s)", autoMvEndName(r.m.end));
  /* 300: the view lost on the profile's slowing part, the reference under 0.20 m/s but over it 0.4 s before: VIEW_FAST,
   * braking from that earlier speed */
  rigSpeed(&r, 300, 5.5f);
  autoMotionMove(&r.m, &r.in, 0, 130, 0);
  {
    autoMvSpeed_t c;
    autoMvSpeedCfg(300, &c);
    const float tAt = r.m.ph.T - 0.17f / c.aH;   /* the reference at 0.17 m/s, slowing */
    steps(&r, (int)lroundf(tAt * 1000.0f));
    float v1, v0;
    autoMvProfEval(&r.m.ph, r.m.tRun, &s, &v1, &a);
    autoMvProfEval(&r.m.ph, r.m.tRun - AUTO_MV_FAST_T_TRACK_S, &s, &v0, &a);
    r.in.holdApplied = false;
    deb = 0;
    while (r.land == AUTO_MVL_NONE && deb < 3000) {
      step(&r);
      deb++;
    }
    CHECK(v1 < AUTO_MV_FAST_PAUSE_V && v0 > AUTO_MV_FAST_PAUSE_V && r.land == AUTO_MVL_FAST &&
              r.m.end == AUTO_MVE_VIEW_FAST && fabsf(r.m.brakeS - v0 / c.brake) < 0.01f,
          "300: the view lost on the slowing part (the reference %.2f m/s, %.2f m/s 0.4 s before): VIEW_FAST after %d ms, "
          "brake %.2f s from the earlier speed", v1, v0, deb, r.m.brakeS);
  }
  /* 200, the view lost 0.5 s in (0.10 m/s): the pause of before, then NO_CAM after 2 s */
  rigSpeed(&r, 200, 4.0f);
  autoMotionMove(&r.m, &r.in, 0, 150, 0);
  steps(&r, 500);
  r.in.holdApplied = false;
  step(&r);
  CHECK(r.land == AUTO_MVL_NONE && r.m.state == AUTO_MV_PAUSED, "200: the view lost at 0.10 m/s: PAUSED");
  int k = 0;
  while (r.land == AUTO_MVL_NONE && k < 3000) {
    step(&r);
    k++;
  }
  CHECK(r.land == AUTO_MVL_NO_CAM && k >= 1999 && k <= 2001, "... NO_CAM after %d ms", k);
  /* 100: the same loss at 0.20 m/s cruise: the pause of before */
  rigSpeed(&r, 100, 4.0f);
  autoMotionMove(&r.m, &r.in, 0, 150, 0);
  steps(&r, 2500);
  r.in.holdApplied = false;
  step(&r);
  CHECK(r.land == AUTO_MVL_NONE && r.m.state == AUTO_MV_PAUSED, "100: the view lost in the cruise: PAUSED (as before)");
  /* 300: a turn losing its view pauses (no VIEW_FAST for a YAW) */
  rigSpeed(&r, 300, 3.0f);
  autoMotionYaw(&r.m, &r.in, 90);
  steps(&r, 800);
  r.in.holdApplied = false;
  step(&r);
  CHECK(r.land == AUTO_MVL_NONE && r.m.state == AUTO_MV_PAUSED, "300: a turn's view lost: PAUSED");
  /* 200, a tilt and the view lost on one step: the tilt's stop request (the state machine's), no fast stop */
  rigSpeed(&r, 200, 4.0f);
  autoMotionMove(&r.m, &r.in, 0, 150, 0);
  steps(&r, 2500);
  r.in.tiltDeg = 9.0f;
  steps(&r, 100);
  r.in.holdApplied = false;
  step(&r);
  CHECK(r.land == AUTO_MVL_TILT && r.m.end != AUTO_MVE_VIEW_FAST, "200: a tilt stop request on the step: no fast stop");
}

static void testFastMargins(void)
{
  rig_t a, b;
  rigSpeed(&a, 100, 3.0f);
  rigSpeed(&b, 200, 3.0f);
  autoMotionMove(&a.m, &a.in, 40, 100, 0);
  autoMotionMove(&b.m, &b.in, 40, 100, 0);
  const float len = sqrtf(0.4f * 0.4f + 1.0f), lead = 0.40f * b.m.ph.vPk;
  CHECK(fabsf(b.m.margLatM - (a.m.margLatM + lead * 0.4f / len)) < 1e-5f &&
            fabsf(b.m.margFwd - (a.m.margFwd + lead * (1.0f / len) / 2.0f)) < 1e-5f,
        "200: the margins grow by 0.4 s x the peak (%.3f m/s): sideways %.3f -> %.3f m, forward %.4f -> %.4f",
        b.m.ph.vPk, a.m.margLatM, b.m.margLatM, a.m.margFwd, b.m.margFwd);
  CHECK(a.m.frozenRef == AUTO_MV_FROZEN_REF && fabsf(b.m.frozenRef - 0.168f) < 1e-6f, "frozenRef 0.10 / 0.168");
  /* a frozen view at 200: no stop while the reference is under 0.18 ln, PROGRESS once past it */
  rig_t r;
  rigSpeed(&r, 200, 4.0f);
  r.camFrozen = true;
  autoMotionMove(&r.m, &r.in, 0, 200, 0);
  float refAtStop = 0.0f;
  for (int i = 0; i < 20000 && active(&r); i++) {
    const float ref = r.out.refFwd;
    step(&r);
    if (!active(&r)) {
      refAtStop = ref;
    }
  }
  CHECK(r.m.end == AUTO_MVE_PROGRESS && refAtStop > 0.167f && refAtStop < 0.19f, "200, a frozen 2 m approach from D 4: "
        "PROGRESS with the reference at %.3f ln (0.168)", refAtStop);
}

/* The Python mirror's golden (tools/tests/mvspeed_golden.json): the profile time (max of the horizontal and vertical
 * trapezoids) and the horizontal peak of a grid of MOVEs at each speed step, from autoMotionMove itself. Checked
 * against the file (1e-5) when it is there; written with --write-golden. */
/* ---- guard margins (user decisions 2026-10-17, docs/autonomy-a2b.md 32) ----------------------------------------------- */

static void useMargins(int32_t floorCm, int32_t stopCm, int32_t progCm, int32_t progPct, int32_t latCm)
{
  autoMvMargins_t g;
  autoMvMarginsFrom(floorCm, stopCm, progCm, progPct, latCm, AUTO_MV_NEAR_SCENE_CM, 0, &g);   /* (no move fixes) */
  autoMotionSetMargins(&g);
}

static bool sameMargins(const autoMvMargins_t *a, int fl, int pl, int st, int pm, int pp, int la)
{
  return a->floorCm == fl && a->planCm == pl && a->stopCm == st && a->progCm == pm && a->progPct == pp &&
         a->latCm == la;
}

/* The largest forward cm the drone takes at D (plan: the TAKEOFF's plan), from 400 down. */
static int16_t largestFwd(float d, bool plan)
{
  for (int16_t fc = 400; fc > 0; fc--) {
    rig_t t;
    rigInit(&t);
    sceneAt(&t, d);
    if ((plan ? autoMotionMovePlan(&t.m, &t.in, 0, fc, 0) : autoMotionMove(&t.m, &t.in, 0, fc, 0)) == AUTO_MVW_NONE) {
      return fc;
    }
  }
  return 0;
}

/* Runs a MOVE's RUN with the camera reading (lat m at the start distance, the lagged reference forward + extra ln)
 * until it stops or ends; true if it stopped. */
static bool runWith(rig_t *r, float latM, float extraFwd, int n)
{
  for (int i = 0; i < n && active(r); i++) {
    r->in.nowMs++;
    r->land = autoMotionStep(&r->m, &r->in, &r->out);
    r->in.holdEpochSeen = r->out.epoch;
    r->in.progLat = r->out.refLat + latM / r->m.dLat0;
    r->in.progFwd = r->out.refFwd + extraFwd;
  }
  return r->m.state == AUTO_MV_STOPPED;
}

/* The settings, their clamps and defaults; the floor and the plan's; the live front stop; the progress and sideways
 * margins (a back move's forward margin kept); the SHORT end's off-the-line limit; a contact's back-move anchor; the
 * settings of before reproducing the margins of before. */
static void testGuardMargins(void)
{
  autoMvMargins_t g;
  autoMotionSetMargins(NULL);
  autoMotionGetMargins(&g);
  CHECK(sameMargins(&g, 30, 35, 25, 20, 40, 40) && g.nearCm == 80 && g.fix == AUTO_MVFIX_DEFAULT,
        "the default margins: floor 30, plan 35, front stop 25, progress 20 cm + 40 %%, sideways 40 cm, near 80 cm (60 "
        "until the review of 2026-10-17), mvfix 59 (%u %u %u %u %u %u %u)", g.floorCm, g.planCm, g.stopCm,
        g.progCm, g.progPct, g.latCm, g.nearCm);
  autoMvMarginsDefault(&g);
  CHECK(sameMargins(&g, AUTO_MOVE_MIN_FRONT_CM, AUTO_PLAN_MIN_FRONT_CM, AUTO_MV_FRONT_STOP_CM, AUTO_MV_PROG_MARGIN_CM,
                    AUTO_MV_PROG_MARGIN_PCT, AUTO_MV_LAT_MARGIN_CM), "autoMvMarginsDefault: the header's");
  autoMvMarginsOld(&g);
  CHECK(sameMargins(&g, 50, 60, 40, 15, 30, 25) && g.nearCm == 0, "autoMvMarginsOld: the margins before 2026-10-17");
  /* the NVS values into the margins: ranges, the plan = floor + 5, the stop 5 below the floor */
  autoMvMarginsFrom(30, 25, 20, 40, 40, 80, 59, &g);
  CHECK(sameMargins(&g, 30, 35, 25, 20, 40, 40) && g.nearCm == 80 && g.fix == AUTO_MVFIX_DEFAULT &&
            AUTO_MVFIX_DEFAULT == 0x3Bu && AUTO_MVFIX_MAX == 0x3Fu,
        "the default settings give the default margins (mvfix 59: 27 and FRONT)");
  autoMvMarginsFrom(50, 40, 15, 30, 25, 0, 0, &g);
  CHECK(sameMargins(&g, 50, 55, 40, 15, 30, 25), "the settings of before: the plan 55 (the Mac's floor was 0.60, 10 cm "
        "over the drone's, until 2026-10-17)");
  autoMvMarginsFrom(5, 5, 1, 1, 1, -5, -3, &g);
  CHECK(sameMargins(&g, 20, 25, 15, 10, 10, 15) && g.nearCm == 0 && g.fix == 0, "below the ranges: clamped up (%u %u %u %u %u %u)", g.floorCm,
        g.planCm, g.stopCm, g.progCm, g.progPct, g.latCm);
  autoMvMarginsFrom(1000, 1000, 1000, 1000, 1000, 1000, 1000, &g);
  CHECK(sameMargins(&g, 100, 105, 80, 60, 100, 100) && g.nearCm == 150 && g.fix == AUTO_MVFIX_MAX, "above the ranges: clamped down (%u %u %u %u %u %u)", g.floorCm,
        g.planCm, g.stopCm, g.progCm, g.progPct, g.latCm);
  autoMvMarginsFrom(30, 40, 20, 40, 40, 60, 31, &g);
  CHECK(g.stopCm == 25, "a front stop at or over the floor - 5 cm: clamped to it (%u)", g.stopCm);
  autoMvMarginsFrom(-7, 200, 20, 40, 40, 60, 31, &g);
  CHECK(g.floorCm == 20 && g.stopCm == 15, "floor 20: the stop at most 15 (%u %u)", g.floorCm, g.stopCm);
  g.floorCm = 30;
  g.planCm = 10;
  g.stopCm = 60;
  g.progCm = 1;   /* (0 is off since 2026-10-18: testGuardsOff) */
  g.progPct = 255;
  g.latCm = 250;
  g.nearCm = 200;
  autoMotionSetMargins(&g);
  autoMotionGetMargins(&g);
  CHECK(sameMargins(&g, 30, 30, 25, 10, 100, 100) && g.nearCm == 150, "autoMotionSetMargins clamps too (the plan at "
        "least the floor)");
  autoMotionSetMargins(NULL);

  /* the floor (default 0.30 m at k 0.6) and the plan's (0.35 m) at the TOO_CLOSE boundary */
  static const float ds[] = {1.5f, 2.0f, 3.0f, 4.66f};
  int okF = 0;
  for (unsigned i = 0; i < sizeof ds / sizeof ds[0]; i++) {
    const float d = ds[i];
    const int16_t fm = largestFwd(d, false), fp = largestFwd(d, true);
    const float endM = d * powf((d - 0.01f * fm) / d, 1.0f / AUTO_MV_K_LOW);
    const float endM1 = d * powf((d - 0.01f * (fm + 1)) / d, 1.0f / AUTO_MV_K_LOW);
    const float endP = d * powf((d - 0.01f * fp) / d, 1.0f / AUTO_MV_K_LOW);
    const float endP1 = d * powf((d - 0.01f * (fp + 1)) / d, 1.0f / AUTO_MV_K_LOW);
    const bool ok = endM >= 0.30f - 1e-4f && endM1 < 0.30f + 1e-4f && endP >= 0.35f - 1e-4f && endP1 < 0.35f + 1e-4f;
    okF += ok;
    printf("info: D %.2f m: the largest MOVE forward %.2f m (k 0.6 end %.3f m; 0.50 m before: %.2f m), the plan's "
           "%.2f m (end %.3f m)\n", d, fm * 0.01f, endM, d - d * powf(0.50f / d, AUTO_MV_K_LOW), fp * 0.01f, endP);
  }
  CHECK(okF == 4, "the largest forward MOVE ends >= 0.30 m and the plan's >= 0.35 m from the scene at k 0.6, 1 cm "
        "more under them (%d of 4 D)", okF);
  /* the floor sweep at the defaults: every accepted move's true end >= 0.30 m for k 0.6..2.5 */
  {
    static const float ks[] = {0.6f, 0.7f, 0.8f, 1.0f, 1.3f, 1.6f, 2.0f, 2.5f};
    int accepted = 0, bad = 0;
    float worst = 9.0f;
    for (int di = 0; di <= 75; di++) {
      const float d = 0.5f + 0.1f * (float)di;
      for (int fc = 5; fc <= 400; fc += 5) {
        rig_t r;
        rigInit(&r);
        sceneAt(&r, d);
        if (autoMotionMove(&r.m, &r.in, 0, (int16_t)fc, 0) != AUTO_MVW_NONE) {
          continue;
        }
        accepted++;
        for (unsigned k = 0; k < sizeof ks / sizeof ks[0]; k++) {
          const float end = d * powf((d - 0.01f * (float)fc) / d, 1.0f / ks[k]);
          worst = fminf(worst, end);
          bad += !(end >= 0.30f - 1e-4f);
        }
      }
    }
    CHECK(accepted > 500 && bad == 0, "floor sweep at the defaults, forward 5..400 cm x D 0.5..8 m x k 0.6..2.5: %d "
          "accepted, every true end >= 0.30 m (worst %.3f m)", accepted, worst);
  }
  /* a user's mvfloor 45: the floor follows (and the plan 50) */
  useMargins(45, 25, 20, 40, 40);
  {
    const int16_t fm = largestFwd(3.0f, false), fp = largestFwd(3.0f, true);
    const float endM = 3.0f * powf((3.0f - 0.01f * fm) / 3.0f, 1.0f / AUTO_MV_K_LOW);
    const float endP = 3.0f * powf((3.0f - 0.01f * fp) / 3.0f, 1.0f / AUTO_MV_K_LOW);
    CHECK(endM >= 0.45f - 1e-4f && endM < 0.47f && endP >= 0.50f - 1e-4f && endP < 0.52f, "mvfloor 45 at D 3: the "
          "largest MOVE ends %.3f m, the plan's %.3f m from the scene at k 0.6", endM, endP);
  }
  autoMotionSetMargins(NULL);

  /* the live front stop: progFwdMax = 0.6 ln(D_low / 0.25), the largest move's target 0.6 ln(0.30 / 0.25) = 0.109
   * below it; at k 0.6 it trips with the scene 0.25 m ahead; ideally tracked never */
  {
    const float d0 = 4.66f;
    const int16_t fMax = largestFwd(d0, false);
    rig_t r;
    rigInit(&r);
    sceneAt(&r, d0);
    autoMotionMove(&r.m, &r.in, 0, fMax, 0);
    const float want = AUTO_MV_K_LOW * logf(d0 / 0.25f), gap = r.m.progFwdMax - r.m.fwdTgt;
    CHECK(fabsf(r.m.progFwdMax - want) < 1e-4f && gap >= 0.109f - 0.004f && gap < 0.125f, "forward %d cm at D 4.66: "
          "progFwdMax %.4f = 0.6 ln(4.66 / 0.25), the target %.3f below it (0.109 = 0.6 ln(0.30 / 0.25))", fMax,
          r.m.progFwdMax, gap);
    const float dStop = d0 * expf(-r.m.progFwdMax / AUTO_MV_K_LOW);
    CHECK(fabsf(dStop - 0.25f) < 1e-4f, "at k 0.6 it trips with the scene %.3f m ahead", dStop);
    const int n = untilEnd(&r, 60000);
    CHECK(n > 0 && r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE, "ideally tracked: DONE (%s)",
          autoMvEndName(r.m.end));
    rigInit(&r);
    sceneAt(&r, d0);
    autoMotionMove(&r.m, &r.in, 0, fMax, 0);
    steps(&r, 5000);
    r.in.progFwd = r.m.progFwdMax + 0.01f;
    step(&r);
    CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_FRONT && r.out.ffPitchDeg > 0.0f, "RUN past progFwdMax: "
          "STOPPED FRONT, braking nose up %.2f deg", r.out.ffPitchDeg);
    useMargins(50, 40, 20, 40, 40);
    rigInit(&r);
    sceneAt(&r, d0);
    autoMotionMove(&r.m, &r.in, 0, 300, 0);
    CHECK(fabsf(r.m.progFwdMax - AUTO_MV_K_LOW * logf(d0 / 0.40f)) < 1e-4f, "mvfstop 40: the stop of before");
    autoMotionSetMargins(NULL);
  }

  /* the progress margins at the defaults */
  {
    rig_t r;
    settleRig(&r, 0, 100, 2.5135f);
    CHECK(fabsf(r.m.margLatM - 0.40f) < 1e-6f, "forward 1.00: the sideways margin 0.40 m (0.25 before): %.3f",
          r.m.margLatM);
    const float toNow = 1.5135f / 2.5135f;
    CHECK(!settleHold(&r, -0.39f / toNow, 0.95f, 2900), "0.39 m sideways at the distance now in the settle: no stop");
    settleRig(&r, 0, 100, 2.5135f);
    CHECK(settleHold(&r, -0.41f / toNow, 0.95f, 500) && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS,
          "0.41 m sideways at the distance now: STOP PROGRESS");
    /* RUN: the window steps of 2026-10-17 (forward 0.9 at D 2.85, 0.27 m sideways; forward 1.0 + right 0.35 at D 2.91,
     * 0.36 m right early) no longer stop */
    rigInit(&r);
    sceneAt(&r, 2.85f);
    autoMotionMove(&r.m, &r.in, 0, 90, 0);
    steps(&r, 1500);
    CHECK(!runWith(&r, 0.27f, 0.0f, 400) && active(&r), "forward 0.90 at D 2.85, 0.27 m sideways in RUN: no stop "
          "(0.25 m before)");
    rigInit(&r);
    sceneAt(&r, 2.91f);
    autoMotionMove(&r.m, &r.in, 35, 100, 0);
    CHECK(fabsf(r.m.margLatM - 0.40f) < 1e-6f, "right 0.35 + forward 1.00: the sideways margin max(0.40, 0.20 + 40 %% "
          "of 0.35) = 0.40 m (%.3f; 0.255 before)", r.m.margLatM);
    steps(&r, 600);
    CHECK(!runWith(&r, 0.33f, 0.0f, 300), "... 0.33 m off its lateral reference early in RUN: no stop");
    rigInit(&r);
    sceneAt(&r, 4.0f);
    autoMotionMove(&r.m, &r.in, 150, 0, 0);
    CHECK(fabsf(r.m.margLatM - 0.80f) < 1e-6f, "right 1.50 at D 4: the sideways margin 0.20 + 40 %% = 0.80 m (%.3f; "
          "0.60 before)", r.m.margLatM);
    /* forward: 0.20 m at the end distance + 40 % of the ln target */
    rigInit(&r);
    sceneAt(&r, 3.0f);
    autoMotionMove(&r.m, &r.in, 0, 100, 0);
    const float tgt = logf(3.0f / 2.0f);
    CHECK(fabsf(r.m.margFwd - (0.20f / 2.0f + 0.40f * tgt)) < 1e-5f, "forward 1.00 at D 3: the forward margin 0.20 / "
          "2.0 + 0.40 x %.4f = %.4f (%.4f)", tgt, 0.20f / 2.0f + 0.40f * tgt, r.m.margFwd);
    /* a back move keeps the margins of before forward (its overshoot is blind), the looser sideways one; with mvfix LONG
     * (the default since the 300 % forensics) its 0.15 m at its end distance D0 + 1.00 */
    rigInit(&r);
    sceneAt(&r, 2.0f);
    autoMotionMove(&r.m, &r.in, 0, -100, 0);
    const float tb = fabsf(logf(2.0f / 3.0f));
    CHECK(fabsf(r.m.margFwd - (0.15f / 3.0f + 0.30f * tb)) < 1e-5f && fabsf(r.m.margLatM - 0.40f) < 1e-6f,
          "back 1.00 at D 2: the forward margin of before at the end distance 0.15 / 3.0 + 0.30 x %.4f = %.4f (%.4f), "
          "sideways 0.40 m", tb, 0.15f / 3.0f + 0.30f * tb, r.m.margFwd);
    useMargins(30, 25, 10, 20, 40);   /* (no move fixes: the start distance as before) */
    rigInit(&r);
    sceneAt(&r, 2.0f);
    autoMotionMove(&r.m, &r.in, 0, -100, 0);
    CHECK(fabsf(r.m.margFwd - (0.10f / 2.0f + 0.20f * tb)) < 1e-5f, "back: a tighter setting applies (%.4f)",
          r.m.margFwd);
    /* the settings of before give the margins of before */
    useMargins(50, 40, 15, 30, 25);
    rigInit(&r);
    sceneAt(&r, 2.91f);
    autoMotionMove(&r.m, &r.in, 35, 100, 0);
    CHECK(fabsf(r.m.margLatM - (0.15f + 0.30f * 0.35f)) < 1e-6f, "mvprogm 15, mvprogr 30, mvlatm 25: right 0.35 + "
          "forward 1.00: 0.255 m as before (%.4f)", r.m.margLatM);
    autoMotionSetMargins(NULL);
  }

  /* the SHORT end's off-the-line limit is mvprogm: 0.18 m off DONE-or-SHORT-by-the-settle, 0.22 m SHORT */
  {
    rig_t r;
    const float toNow = 1.5135f / 2.5135f;
    settleRig(&r, 0, 100, 2.5135f);
    settleHold(&r, -0.18f / toNow, 0.96f, 3100);
    CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE, "0.96 m forward, 0.18 m off its line at the timeout: "
          "DONE, the target kept (SHORT over 0.15 m before) (%s)", autoMvEndName(r.m.end));
    settleRig(&r, 0, 100, 2.5135f);
    settleHold(&r, -0.22f / toNow, 0.96f, 3100);
    CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_SHORT, "0.22 m off its line: SHORT (%s)",
          autoMvEndName(r.m.end));
  }

  /* the near-scene braked view loss (mvnear 60): a forward MOVE whose measured progress puts the scene within 0.60 m at
   * k 0.6 brakes FRONT once its view stays lost AUTO_MV_FAST_VIEW_DEB_MS, at 100 too and whatever its reference's speed;
   * farther, a back move, or mvnear 0: the pause of before */
  {
    static const struct {
      uint16_t pct;
      int16_t fwd;
      float lostAt;   /* the scene ahead (k 0.6) when the view is lost, m */
      int near;       /* mvnear */
      bool front;
    } nc[] = {
      {100, 200, 0.50f, 60, true}, {100, 200, 0.70f, 60, false}, {100, 200, 0.50f, 0, false},
      {200, 200, 0.55f, 60, true}, {300, 200, 0.50f, 60, true}, {100, -100, 0.0f, 60, false},
    };
    for (unsigned i = 0; i < sizeof nc / sizeof nc[0]; i++) {
      useMargins(30, 25, 20, 40, 40);
      autoMvMargins_t mg;
      autoMotionGetMargins(&mg);
      mg.nearCm = (uint8_t)nc[i].near;
      autoMotionSetMargins(&mg);
      rig_t r;
      rigSpeed(&r, nc[i].pct, 3.0f);
      CHECK(autoMotionMove(&r.m, &r.in, 0, nc[i].fwd, 0) == AUTO_MVW_NONE, "near %u: MOVE %d at D 3 accepted", i,
            nc[i].fwd);
      /* run until the measured progress (the ideal camera) puts the scene at lostAt at k 0.6 (D0 = D_low = 3) */
      const float pAt = nc[i].fwd > 0 ? AUTO_MV_K_LOW * logf(3.0f / nc[i].lostAt) : 0.0f;
      int guard = 0;
      while (nc[i].fwd > 0 && r.m.progFwd < pAt && active(&r) && guard++ < 30000) {
        step(&r);
      }
      if (nc[i].fwd < 0) {
        steps(&r, 1500);
      }
      float s, v, a;
      autoMvProfEval(&r.m.ph, r.m.tRun, &s, &v, &a);
      r.in.holdApplied = false;
      int n = 0;
      while (r.land == AUTO_MVL_NONE && n < AUTO_MV_NO_CAM_LAND_MS - 100) {
        step(&r);
        n++;
      }
      if (nc[i].front) {
        CHECK(active(&r) == false && r.land == AUTO_MVL_FAST && r.m.state == AUTO_MV_STOPPED &&
                  r.m.end == AUTO_MVE_FRONT && n == AUTO_MV_FRONT_VIEW_DEB_MS + 1 && r.out.ffPitchDeg > 0.0f &&
                  r.m.brakeS > 0.0f && r.m.brakeS >= AUTO_MV_FAST_PAUSE_V / r.m.brakeAcc - 0.01f,
              "mvspeed %u, forward %d, the view lost with the scene %.2f m ahead (k 0.6; reference %.2f m/s): STOPPED "
              "FRONT %d ms later, braking nose up %.2f deg for %.2f s", nc[i].pct, nc[i].fwd, nc[i].lostAt, v, n - 1,
              r.out.ffPitchDeg, r.m.brakeS);
      } else {
        CHECK(active(&r) && r.m.state == AUTO_MV_PAUSED && r.land == AUTO_MVL_NONE, "mvspeed %u, MOVE %d, the view "
              "lost (scene %.2f m, mvnear %d): PAUSED as before, %d ms on", nc[i].pct, nc[i].fwd, nc[i].lostAt,
              nc[i].near, n);
      }
    }
    autoMotionSetMargins(NULL);
  }

  /* mvfix FRONT (the review of 2026-10-17, docs/autonomy-a2b.md 34.1): the live front stop on every horizontal MOVE
   * (a sideways or back move from near the floor; a vertical one holds as a hover does), and the near brake on any
   * horizontal MOVE whose view is lost near the scene, at 100 too; without the bit: as before */
  for (int k = 0; k < 2; k++) {
    autoMvMargins_t mg;
    autoMvMarginsDefault(&mg);
    if (k == 0) {
      mg.fix = (uint8_t)(mg.fix & ~AUTO_MVFIX_FRONT);
    }
    const char *nm = k ? "mvfix FRONT" : "without FRONT";
    static const int16_t mvs[3][3] = {{19, 0, 0}, {-19, -40, 0}, {0, 0, 20}};
    for (int j = 0; j < 3; j++) {
      autoMotionSetMargins(&mg);
      rig_t r;
      rigInit(&r);
      sceneAt(&r, 0.40f);
      CHECK(autoMotionMove(&r.m, &r.in, mvs[j][0], mvs[j][1], mvs[j][2]) == AUTO_MVW_NONE, "%s: MOVE %d %d %d at D "
            "0.40 accepted", nm, mvs[j][0], mvs[j][1], mvs[j][2]);
      const float lim = AUTO_MV_K_LOW * logf(r.m.dLow0 / 0.25f);
      const bool horiz = j < 2;
      CHECK(k && horiz ? fabsf(r.m.progFwdMax - lim) < 1e-5f : r.m.progFwdMax > 1e29f,
            "%s, %s: the live front stop %s (%.4f ln: the scene 0.25 m ahead at k 0.6)", nm,
            j == 0 ? "sideways" : (j == 1 ? "back-diagonal" : "vertical"), k && horiz ? "on" : "off",
            k && horiz ? r.m.progFwdMax : lim);
      if (horiz) {
        /* pushed 0.30 ln toward the scene (under the forward PROGRESS margin's 0.20 / 0.40 = 0.5 ln) */
        const bool stopped = runWith(&r, 0.0f, lim + 0.02f, 400);
        CHECK(k ? stopped && r.m.end == AUTO_MVE_FRONT : !(stopped && r.m.end == AUTO_MVE_FRONT),
              "%s, MOVE %d %d pushed past the stop toward the scene: %s (%s)", nm, mvs[j][0], mvs[j][1],
              k ? "STOPPED FRONT" : "no FRONT", autoMvEndName(r.m.end));
      }
    }
    /* the near brake: a sideways move from 0.60 m, its view lost (100 %) */
    autoMotionSetMargins(&mg);
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 0.60f);
    CHECK(autoMotionMove(&r.m, &r.in, 29, 0, 0) == AUTO_MVW_NONE, "%s: right 0.29 at D 0.60 accepted", nm);
    steps(&r, 800);
    r.in.holdApplied = false;
    int n = 0;
    while (r.land == AUTO_MVL_NONE && n < 1000) {
      step(&r);
      n++;
    }
    if (k) {
      CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_FRONT && r.land == AUTO_MVL_FAST &&
                n == AUTO_MV_FRONT_VIEW_DEB_MS + 1 && r.m.brakeLat > 0.0f && r.m.brakeFwd == 0.0f && r.m.brakeS > 0.0f,
            "%s: its view lost 0.60 m from the scene at 100 %%: STOPPED FRONT %d ms later, braking sideways (%.2f m/s "
            "for %.2f s)", nm, n - 1, r.m.brakeLat, r.m.brakeS);
    } else {
      CHECK(r.m.state == AUTO_MV_PAUSED && r.land == AUTO_MVL_NONE, "%s: its view lost at 0.60 m: PAUSED as before",
            nm);
    }
  }
  autoMotionSetMargins(NULL);

  /* a back move stopped by a contact: its anchor toward the scene keeps the move floor */
  {
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 0.40f);
    CHECK(autoMotionMove(&r.m, &r.in, 0, -60, 0) == AUTO_MVW_NONE, "back 0.60 at D 0.40: accepted");
    steps(&r, 300);
    autoMotionStop(&r.m, &r.in, AUTO_MVE_CONTACT);
    const float dF = r.m.d0 * expf(-r.m.progFwd);
    const float ahead = dF - dF * expf(-r.m.fwdTgt);
    CHECK(r.m.backOff && ahead > 0.0f && ahead <= dF - 0.30f + 1e-4f && dF - ahead >= 0.30f - 1e-4f,
          "its contact anchor %.3f m toward the scene, %.3f m from it (>= the 0.30 m floor; 0.50 before)", ahead,
          dF - ahead);
  }
}

static void testSpeedGolden(const char *writePath)
{
  static const int16_t mv[][3] = {{0, 100, 0}, {0, 300, 0}, {0, 200, 0}, {60, 0, 0}, {-60, 0, 0}, {0, -100, 0},
                                  {-150, 0, 0}, {70, 70, 0}, {-30, 150, 0}, {40, -80, 0}, {0, 50, 20}, {0, 0, -30},
                                  {120, 40, 0}, {0, 400, 0}, {10, 30, 0}};
  static const float ds[] = {1.3f, 2.0f, 2.5f, 3.0f, 3.24f, 4.0f, 5.0f, 6.0f};
  static const uint16_t ps[] = {100, 150, 200, 250, 300, 175};
  FILE *w = writePath ? fopen(writePath, "w") : NULL;
  FILE *g = writePath ? NULL : fopen("tools/tests/mvspeed_golden.json", "r");
  if (w) {
    fprintf(w, "{\n \"note\": \"auto_motion.c profiles (tools/tests/test_auto_motion.c --write-golden): [pct, right, "
               "fwd, up, D, T_s, vpk_mps] of accepted MOVEs; espfly_auto.py move_profile_s mirrors them\",\n"
               " \"rows\": [\n");
  }
  int n = 0, cmp = 0, okCmp = 0;
  bool first = true;
  for (unsigned ip = 0; ip < sizeof(ps) / sizeof(ps[0]); ip++) {
    for (unsigned id = 0; id < sizeof(ds) / sizeof(ds[0]); id++) {
      for (unsigned k = 0; k < sizeof(mv) / sizeof(mv[0]); k++) {
        rig_t r;
        rigSpeed(&r, ps[ip], ds[id]);
        if (autoMotionMove(&r.m, &r.in, mv[k][0], mv[k][1], mv[k][2]) != AUTO_MVW_NONE) {
          continue;
        }
        const float T = fmaxf(r.m.ph.T, r.m.pv.T), vpk = r.m.ph.vPk;
        n++;
        if (w) {
          fprintf(w, "%s  [%u, %.2f, %.2f, %.2f, %.2f, %.6f, %.6f]", first ? "" : ",\n", ps[ip], mv[k][0] * 0.01,
                  mv[k][1] * 0.01, mv[k][2] * 0.01, (double)ds[id], (double)T, (double)vpk);
          first = false;
        }
        if (g) {
          char line[256];
          unsigned gp;
          float gr, gf, gu, gd, gT, gv;
          while (fgets(line, sizeof(line), g)) {
            if (sscanf(line, " [%u, %f, %f, %f, %f, %f, %f]", &gp, &gr, &gf, &gu, &gd, &gT, &gv) == 7) {
              cmp++;
              okCmp += gp == ps[ip] && fabsf(gr - mv[k][0] * 0.01f) < 1e-4f && fabsf(gf - mv[k][1] * 0.01f) < 1e-4f &&
                       fabsf(gu - mv[k][2] * 0.01f) < 1e-4f && fabsf(gd - ds[id]) < 1e-4f && fabsf(gT - T) < 1e-5f &&
                       fabsf(gv - vpk) < 1e-5f;
              break;
            }
          }
        }
      }
    }
  }
  if (w) {
    fprintf(w, "\n ]\n}\n");
    fclose(w);
    printf("info: wrote %s (%d rows)\n", writePath, n);
  }
  if (g) {
    fclose(g);
    CHECK(cmp == n && okCmp == n, "tools/tests/mvspeed_golden.json: every row reproduced (%d/%d, %d rows here)", okCmp,
          cmp, n);
  } else if (!w) {
    printf("info: tools/tests/mvspeed_golden.json not found (run from Firmware/esp-drone): not compared\n");
  }
}

/* ---- the fast steps' acceleration and settle as settings (2026-10-17 "no, I want real 300%", auto_motion.h, NVS
 * mvacc / mvsettle, docs/autonomy-a2b.md 35) ----------------------------------------------------------------------- */

/* The identity script (auto_motion_identity.h) at each fast step with the default margins and mvfix, hashed on d3bedee's
 * auto_motion.c (this file's build line, -O2): with its caps (A_H 0.20 m/s^2, the settle floor 2500 ms: the defaults
 * must reproduce them) and with 4dea542's (0.30, 1000: d3bedee built with -DAUTO_MV_HOST_VH_OVERRIDE and its host-only
 * autoMvFastCapsOld, which mvacc 30 / mvsettle 1000 replace). 100 for both: the caps are not read there. */
static const struct {
  uint16_t pct;
  uint64_t d3bedee, caps4dea542;
} fastIdHash[] = {
  {100, 0xFA765C09948F061Cull, 0xFA765C09948F061Cull},
  {150, 0x663B4B9687241D7Full, 0x6FDBFBA2DF28199Eull},
  {200, 0xA63BF6CBDA097D3Dull, 0x98EC01475D1FC49Cull},
  {250, 0x4EC5199DE5D6C23Full, 0x6DF26AD25690360Full},
  {300, 0x7B6490612920859Bull, 0xBC29E98B6758DEF5ull},
};
static uint16_t fastIdPct;

static void setSpeedFastId(autoMotion_t *m)
{
  autoMotionSetSpeed(m, fastIdPct);
}

/* The margins with the guards-off settings (auto_motion.h "guards off"): mvfloor, mvfstop, mvprogm, mvprogr (0 off),
 * mvklow and mvlat; mvlatm, mvnear and mvfix at their defaults. */
static void guardsSet(int32_t floorCm, int32_t stopCm, int32_t progCm, int32_t progPct, int32_t kLowPct, int32_t latPct)
{
  autoMvMargins_t g;
  autoMvMarginsFrom(floorCm, stopCm, progCm, progPct, AUTO_MV_LAT_MARGIN_CM, AUTO_MV_NEAR_SCENE_CM, AUTO_MVFIX_DEFAULT,
                    &g);
  autoMvMarginsExtra(&g, kLowPct, latPct);
  autoMotionSetMargins(&g);
}

/* The guards down to off (user decision 2026-10-18 "remove the guards or make them absolutely minimum"; auto_motion.h
 * "guards off", docs/autonomy-a2b.md 38): the clamps keep 0 as off; mvklow 100 plans at k 1; mvfloor 0 plans up to the
 * scene; mvfstop 0 never FRONT; mvprogm 0 or mvprogr 0 never PROGRESS from the margins or the frozen view; mvlat 0 never
 * TOO_WIDE; mvwmax the expansion rate cap; with everything off the tilt and view stops stay. */
static void testGuardsOff(void)
{
  autoMvMargins_t g;
  autoMotionSetMargins(NULL);
  autoMotionGetMargins(&g);
  CHECK(g.kLowPct == 60 && g.latPct == 50 && g.kLowPct == AUTO_MV_K_LOW_PCT && g.latPct == AUTO_MV_LAT_D_REL_PCT,
        "the defaults: k 0.60, the sideways cap 0.5 x D (%u %u)", g.kLowPct, g.latPct);
  autoMvMarginsOld(&g);
  CHECK(g.kLowPct == 60 && g.latPct == 50, "the margins of before: the same k and cap (constants until 2026-10-18)");
  /* the clamps: 0 stays 0 (the plan with the floor), 1..min - 1 up to min as before, negative up to min */
  autoMvMarginsFrom(0, 0, 0, 0, 40, 80, 59, &g);
  CHECK(g.floorCm == 0 && g.planCm == 0 && g.stopCm == 0 && g.progCm == 0 && g.progPct == 0 && g.latCm == 40 &&
            g.nearCm == 80 && g.fix == 59 && g.kLowPct == 60 && g.latPct == 50,
        "autoMvMarginsFrom with 0: the floor, the plan, the stop, the progress base and part off; the rest as set "
        "(%u %u %u %u %u)", g.floorCm, g.planCm, g.stopCm, g.progCm, g.progPct);
  autoMvMarginsFrom(0, 80, 20, 40, 40, 80, 59, &g);
  CHECK(g.floorCm == 0 && g.stopCm == 80, "no floor: the stop keeps its own range (80, not clamped under a floor)");
  autoMvMarginsFrom(30, 0, 20, 40, 40, 80, 59, &g);
  CHECK(g.floorCm == 30 && g.planCm == 35 && g.stopCm == 0, "a floor with the stop off (%u %u %u)", g.floorCm,
        g.planCm, g.stopCm);
  autoMvMarginsFrom(1, 1, 1, 1, 40, 80, 59, &g);
  CHECK(g.floorCm == 20 && g.planCm == 25 && g.stopCm == 15 && g.progCm == 10 && g.progPct == 10,
        "1: up to the ranges' lower ends as before (%u %u %u %u %u)", g.floorCm, g.planCm, g.stopCm, g.progCm,
        g.progPct);
  autoMvMarginsExtra(&g, 100, 0);
  CHECK(g.kLowPct == 100 && g.latPct == 0, "autoMvMarginsExtra: k 100, the cap off");
  autoMvMarginsExtra(&g, 59, 201);
  CHECK(g.kLowPct == 60 && g.latPct == 200, "... clamped to 60..100 and 0..200 (%u %u)", g.kLowPct, g.latPct);
  autoMvMarginsExtra(&g, 1000, -5);
  CHECK(g.kLowPct == 100 && g.latPct == 0, "... (%u %u)", g.kLowPct, g.latPct);
  g.floorCm = g.planCm = g.stopCm = g.progCm = g.progPct = 0;
  g.kLowPct = 100;
  g.latPct = 0;
  autoMotionSetMargins(&g);
  autoMotionGetMargins(&g);
  CHECK(g.floorCm == 0 && g.planCm == 0 && g.stopCm == 0 && g.progCm == 0 && g.progPct == 0 && g.kLowPct == 100 &&
            g.latPct == 0,
        "autoMotionSetMargins keeps every off (%u %u %u %u %u %u %u)", g.floorCm, g.planCm, g.stopCm, g.progCm,
        g.progPct, g.kLowPct, g.latPct);

  /* mvklow 100: the floor at k 1, so the largest forward MOVE at D 2.0 ends at 0.30 m: 1.70 m (1.35 at k 0.6); the
   * front stop ln(D_low / 0.25); the measured lower bound after the move is the measured D itself */
  guardsSet(30, 25, 20, 40, 100, 50);
  {
    const int16_t fm = largestFwd(2.0f, false), fp = largestFwd(2.0f, true);
    CHECK(fm == 169 && fp == 165, "mvklow 100 at D 2.0: the largest MOVE 1.69 m (1.35 at k 0.6; 1.70 ends 0.29999998 m "
          "in float), the plan's 1.65 m (%d %d)", fm, fp);
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 2.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 169, 0) == AUTO_MVW_NONE, "forward 1.69 at D 2.0 accepted");
    CHECK(fabsf(r.m.progFwdMax - logf(2.0f / 0.25f)) < 1e-4f &&
              fabsf(r.m.progFwdMax - r.m.fwdTgt - logf(0.31f / 0.25f)) < 2e-3f,
          "the front stop ln(2.0 / 0.25) = %.4f, the target %.3f below it (ln(0.31 / 0.25) = 0.215)", r.m.progFwdMax,
          r.m.progFwdMax - r.m.fwdTgt);
    const int n = untilEnd(&r, 60000);
    CHECK(n > 0 && r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE, "ideally tracked: DONE (%s)",
          autoMvEndName(r.m.end));
    CHECK(fabsf(r.m.dTrack - 0.31f) < 2e-3f && fabsf(r.m.dTrackLow - r.m.dTrack) < 1e-5f,
          "after it D 0.31 m measured, its lower bound the same at k 1 (%.3f %.3f)", r.m.dTrack, r.m.dTrackLow);
  }
  autoMotionSetMargins(NULL);
  {
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 2.0f);
    autoMotionMove(&r.m, &r.in, 0, 100, 0);
    untilEnd(&r, 60000);
    CHECK(fabsf(r.m.dTrack - 1.0f) < 2e-3f && r.m.dTrackLow < 0.65f && r.m.dTrackLow > 0.55f,
          "at the default k 0.6 the lower bound is below the measured D (%.3f %.3f: 2.0 x 0.5^(1/0.6) = 0.63)",
          r.m.dTrack, r.m.dTrackLow);
  }

  /* mvfloor 0: a MOVE (and the plan) up to the scene: D - F >= 0.05 m is the only limit; the live stop still runs */
  guardsSet(0, 25, 20, 40, 60, 50);
  {
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 2.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 194, 0) == AUTO_MVW_NONE, "mvfloor 0: forward 1.94 at D 2.0 accepted (D - F "
          "0.06 m; 1.95 leaves 0.0499999 m in float)");
    CHECK(r.m.progFwdMax < 1e29f && fabsf(r.m.progFwdMax - AUTO_MV_K_LOW * logf(2.0f / 0.25f)) < 1e-4f,
          "... the live front stop stays at 0.25 m (%.3f)", r.m.progFwdMax);
    /* (the expansion rate cap at its end distance, 0.06 m, makes this move crawl at 0.009 m/s: mvwmax is what
     * speeds it up; the stop trips at 0.57 m, 1.43 m in, ~160 s) */
    const int n = untilEnd(&r, 300000);
    CHECK(fabsf(r.m.ph.vPk - 0.009f) < 1e-4f, "... planned at %.4f m/s (0.15 x 0.06 m)", r.m.ph.vPk);
    CHECK(n > 0 && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_FRONT, "... and ends the ideally tracked move "
          "FRONT (%s %s)", autoMvStateName(r.m.state), autoMvEndName(r.m.end));
    rigInit(&r);
    sceneAt(&r, 2.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 195, 0) == AUTO_MVW_TOO_CLOSE, "forward 1.95 at D 2.0: TOO_CLOSE (D - F under "
          "0.05 m)");
    rigInit(&r);
    sceneAt(&r, 2.0f);
    CHECK(autoMotionMovePlan(&r.m, &r.in, 0, 194, 0) == AUTO_MVW_NONE, "the plan's floor off with it");
    CHECK(largestFwd(1.5f, false) == 144 && largestFwd(1.5f, true) == 144, "the largest MOVE and plan at D 1.5: 1.44 m "
          "(0.92 / 0.87 at the defaults)");
  }

  /* mvfstop 0: never FRONT (a wild forward reading is the progress margin's) */
  guardsSet(30, 0, 20, 40, 60, 50);
  {
    static const int16_t mv[3][2] = {{0, 300}, {100, 0}, {0, -100}};
    for (int k = 0; k < 3; k++) {
      rig_t r;
      rigInit(&r);
      sceneAt(&r, 4.66f);
      CHECK(autoMotionMove(&r.m, &r.in, mv[k][0], mv[k][1], 0) == AUTO_MVW_NONE && r.m.progFwdMax > 1e29f,
            "mvfstop 0: MOVE %d %d: no live front stop (progFwdMax %.3g)", mv[k][0], mv[k][1], r.m.progFwdMax);
      steps(&r, 3000);
      r.in.progFwd = 5.0f;
      step(&r);
      CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS, "... a forward reading of 5: PROGRESS, "
            "never FRONT (%s)", autoMvEndName(r.m.end));
    }
  }

  /* mvprogm 0 or mvprogr 0: the progress guard off: margins and the frozen reference 1e30; wild readings never stop
   * the move, a frozen camera runs the profile and the settle out (then the settle's own STOP PROGRESS for less than
   * half of the move measured, as documented); the front stop is its own */
  for (int which = 0; which < 2; which++) {
    guardsSet(30, 25, which == 0 ? 0 : 20, which == 0 ? 40 : 0, 60, 50);
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 4.66f);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 200, 0) == AUTO_MVW_NONE && r.m.margFwd > 1e29f && r.m.margLatM > 1e29f &&
              r.m.frozenRef > 1e29f,
          "%s 0: forward 2.00 at D 4.66: the margins and the frozen reference 1e30 (%.3g %.3g %.3g)",
          which == 0 ? "mvprogm" : "mvprogr", r.m.margFwd, r.m.margLatM, r.m.frozenRef);
    steps(&r, 3000);
    r.in.progLat = 5.0f;
    step(&r);
    r.in.progLat = r.out.refLat;
    r.in.progFwd = -5.0f;
    step(&r);
    CHECK(autoMotionActive(&r.m) && r.m.state == AUTO_MV_RUN, "... readings 5 sideways and -5 forward: no stop (%s)",
          autoMvStateName(r.m.state));
    r.in.progFwd = r.m.progFwdMax + 0.01f;
    step(&r);
    CHECK(r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_FRONT, "... past the front stop: FRONT still (%s)",
          autoMvEndName(r.m.end));
    /* a frozen camera: at the defaults the frozen-view rule stops it early in RUN; with the guard off it runs the
     * whole profile and the settle, then STOP PROGRESS at the settle's end (less than half measured) */
    rigInit(&r);
    sceneAt(&r, 4.66f);
    r.camFrozen = true;
    autoMotionMove(&r.m, &r.in, 0, 200, 0);
    const int runMs = (int)(r.m.ph.T * 1000.0f);
    const int n = untilEnd(&r, 60000);
    CHECK(n >= runMs + AUTO_MV_SETTLE_MAX_MS - 5 && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS,
          "... a frozen camera: the profile (%d ms) and the settle run out, then the settle's STOP PROGRESS (%d ms, %s "
          "%s)", runMs, n, autoMvStateName(r.m.state), autoMvEndName(r.m.end));
    autoMotionSetMargins(NULL);
    rigInit(&r);
    sceneAt(&r, 4.66f);
    r.camFrozen = true;
    autoMotionMove(&r.m, &r.in, 0, 200, 0);
    const int n0 = untilEnd(&r, 60000);
    CHECK(n0 < runMs && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_PROGRESS,
          "(at the defaults the frozen view stops it in RUN: %d ms)", n0);
    /* the frozen reference back to the speed step's once the guard is on again */
    rigInit(&r);
    sceneAt(&r, 4.66f);
    guardsSet(30, 25, 0, 40, 60, 50);
    autoMotionMove(&r.m, &r.in, 0, 100, 0);
    CHECK(r.m.frozenRef > 1e29f, "the guard off: frozenRef 1e30 on this move");
    untilEnd(&r, 60000);
    autoMotionSetMargins(NULL);
    autoMotionMove(&r.m, &r.in, 0, 50, 0);
    CHECK(r.m.frozenRef == AUTO_MV_FROZEN_REF, "the guard on again: the next move has the speed step's frozen "
          "reference (%.3f)", r.m.frozenRef);
  }
  /* mvprogm 0: no SHORT for an end off the line (the settle timed out 0.5 m off the line with the move measured) */
  {
    rig_t r;
    autoMotionSetMargins(NULL);
    settleRig(&r, 0, 100, 3.0f);
    settleHold(&r, 0.5f, 1.0f, 4000);
    CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_SHORT, "defaults: a settle 0.5 m off the line ends SHORT "
          "(%s %s)", autoMvStateName(r.m.state), autoMvEndName(r.m.end));
    guardsSet(30, 25, 0, 40, 60, 50);
    settleRig(&r, 0, 100, 3.0f);
    settleHold(&r, 0.5f, 1.0f, 4000);
    CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE, "mvprogm 0: the same ends DONE (%s %s)",
          autoMvStateName(r.m.state), autoMvEndName(r.m.end));
  }

  /* mvlat: the sideways cap x D (0 off) */
  {
    rig_t r;
    autoMotionSetMargins(NULL);
    rigInit(&r);
    sceneAt(&r, 1.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 51, 0, 0) == AUTO_MVW_TOO_WIDE, "default: right 0.51 at D 1.0: TOO_WIDE");
    guardsSet(30, 25, 20, 40, 60, 100);
    rigInit(&r);
    sceneAt(&r, 1.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 100, 0, 0) == AUTO_MVW_NONE, "mvlat 100: right 1.00 at D 1.0 accepted");
    rigInit(&r);
    sceneAt(&r, 1.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 101, 0, 0) == AUTO_MVW_TOO_WIDE, "... 1.01 TOO_WIDE");
    guardsSet(30, 25, 20, 40, 60, 0);
    rigInit(&r);
    sceneAt(&r, 1.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 150, 0, 0) == AUTO_MVW_NONE, "mvlat 0: right 1.50 at D 1.0 accepted (the "
          "command limit is all that is left)");
  }

  /* mvwmax: the expansion rate cap, at 100 % and above */
  {
    autoMvSpeed_t c;
    CHECK(autoMotionGetFwdRate() == AUTO_MV_W_FWD_PCT && autoMotionGetFwdRate() == 15, "mvwmax in force before any "
          "set: 15");
    rig_t r;
    autoMotionSetMargins(NULL);
    rigInit(&r);
    sceneAt(&r, 2.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 135, 0) == AUTO_MVW_NONE, "forward 1.35 at D 2.0 (the largest at the defaults)");
    const float vPk15 = r.m.ph.vPk;
    autoMotionSetFwdRate(30);
    autoMvSpeedCfg(100, &c);
    CHECK(autoMotionGetFwdRate() == 30 && c.wFwd == 0.30f, "mvwmax 30: W_FWD 0.30 at 100 (%.3f)", c.wFwd);
    autoMvSpeedCfg(300, &c);
    CHECK(c.wFwd == 0.30f, "... at 300 the cap is itself (0.30, not 0.17: %.3f)", c.wFwd);
    rigInit(&r);
    sceneAt(&r, 2.0f);
    autoMotionMove(&r.m, &r.in, 0, 135, 0);
    CHECK(fabsf(vPk15 - 0.0975f) < 1e-4f && fabsf(r.m.ph.vPk - 0.195f) < 1e-4f, "forward 1.35 at D 2.0 (end 0.65 m): "
          "the peak 0.0975 m/s at 15, 0.195 at 30 (%.3f %.3f)", vPk15, r.m.ph.vPk);
    autoMotionSetFwdRate(9);
    CHECK(autoMotionGetFwdRate() == 10, "clamped up to 10");
    autoMotionSetFwdRate(41);
    CHECK(autoMotionGetFwdRate() == 40, "clamped down to 40");
    autoMotionSetFwdRate(AUTO_MV_W_FWD_PCT);
    autoMvSpeedCfg(150, &c);
    CHECK(c.wFwd == AUTO_MV_FAST_W_FWD_CAP, "back at 15: 0.17 at 150 as before");
  }

  /* everything off: the largest move plans to D - 0.05 m and, ideally tracked, ends DONE; a wild reading never stops
   * it; the command limits, the tilt and the view stops stay */
  guardsSet(0, 0, 0, 0, 100, 0);
  {
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 2.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 195, 10) == AUTO_MVW_RANGE, "everything off: the command limits stay (up with "
          "forward over 1.50 m: RANGE)");
    rigInit(&r);
    sceneAt(&r, 2.0f);
    CHECK(autoMotionMove(&r.m, &r.in, 0, 194, 0) == AUTO_MVW_NONE && r.m.progFwdMax > 1e29f && r.m.margFwd > 1e29f,
          "forward 1.94 at D 2.0 accepted, no front stop, no margins");
    steps(&r, 2000);
    r.in.progFwd = 9.0f;
    r.in.progLat = -9.0f;
    step(&r);
    CHECK(autoMotionActive(&r.m), "readings 9 / -9: no stop");
    r.in.progFwd = r.out.refFwd;
    r.in.progLat = r.out.refLat;
    const int n = untilEnd(&r, 300000);   /* (0.009 m/s at mvwmax 15: ~215 s) */
    CHECK(n > 0 && r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE, "ideally tracked: DONE (%s %s)",
          autoMvStateName(r.m.state), autoMvEndName(r.m.end));
    rigInit(&r);
    sceneAt(&r, 2.0f);
    autoMotionMove(&r.m, &r.in, 0, 194, 0);
    steps(&r, 1000);
    r.in.tiltDeg = 8.1f;
    int k = 0;
    while (r.land == AUTO_MVL_NONE && k < 1000) {
      step(&r);
      k++;
    }
    CHECK(r.land == AUTO_MVL_TILT, "tilt 8.1 deg: the stop request TILT stays (%d ms)", k);
    rigInit(&r);
    sceneAt(&r, 2.0f);
    autoMotionMove(&r.m, &r.in, 0, 194, 0);
    steps(&r, 1000);
    r.in.holdApplied = false;
    k = 0;
    while (r.land == AUTO_MVL_NONE && k < 5000) {
      step(&r);
      k++;
    }
    CHECK(r.land == AUTO_MVL_NO_CAM && r.m.state == AUTO_MV_PAUSED, "the view lost 2 s: the stop request NO_CAM stays "
          "(%d ms, %s)", k, autoMvStateName(r.m.state));
  }
  autoMotionSetMargins(NULL);
}

static void testFastCapsIdentity(void)
{
  uint8_t acc = 0;
  uint16_t settle = 0;
  autoMotionGetFastCaps(&acc, &settle);
  CHECK(acc == AUTO_MV_ACC_DEFAULT_CM && settle == AUTO_MV_FAST_SETTLE_MIN_MS && acc == 20 && settle == 2500,
        "the pair in force before any set: mvacc %u, mvsettle %u (the defaults)", acc, settle);
  autoMotionSetMargins(NULL);
  int okDef = 0, okSet = 0, okOld = 0;
  const int n = (int)(sizeof(fastIdHash) / sizeof(fastIdHash[0]));
  for (int i = 0; i < n; i++) {
    fastIdPct = fastIdHash[i].pct;
    const uint64_t h0 = autoMvIdentityHash(setSpeedFastId);   /* the pair as initialised */
    autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
    const uint64_t h1 = autoMvIdentityHash(setSpeedFastId);   /* set to the defaults (the glue at ARM) */
    autoMotionSetFastCaps(30, 1000);
    const uint64_t h2 = autoMvIdentityHash(setSpeedFastId);   /* 'real 300 %' */
    autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
    printf("info: identity script at %u %%: 0x%016llX (defaults), 0x%016llX (mvacc 30, mvsettle 1000)\n",
           fastIdHash[i].pct, (unsigned long long)h0, (unsigned long long)h2);
    okDef += h0 == fastIdHash[i].d3bedee;
    okSet += h1 == fastIdHash[i].d3bedee;
    okOld += h2 == fastIdHash[i].caps4dea542;
  }
  CHECK(okDef == n && okSet == n, "the defaults (as initialised %d/%d, set at ARM %d/%d): every output, answer and the "
        "motion memory bit-identical to d3bedee over the identity script at 100/150/200/250/300", okDef, n, okSet, n);
  CHECK(okOld == n, "mvacc 30 + mvsettle 1000: d3bedee's motions with 4dea542's caps bit for bit at every step (%d/%d)",
        okOld, n);
  autoMotionSetMargins(NULL);
}

/* Every row of 4dea542's golden (tools/tests/mvspeed_golden_4dea542.json: the profiles the 300 % flights of 2026-10-17
 * morning were planned with, at every step) planned again at mvacc 30 / mvsettle 1000: the profile time and the peak
 * within 1e-5. Returns the rows compared (0: the file is not there). */
static int fastGolden4dea542(int *ok, int *ok300)
{
  FILE *g = fopen("tools/tests/mvspeed_golden_4dea542.json", "r");
  if (g == NULL) {
    return 0;
  }
  char line[256];
  int n = 0;
  *ok = *ok300 = 0;
  while (fgets(line, sizeof(line), g)) {
    unsigned gp;
    float gr, gf, gu, gd, gT, gv;
    if (sscanf(line, " [%u, %f, %f, %f, %f, %f, %f]", &gp, &gr, &gf, &gu, &gd, &gT, &gv) != 7) {
      continue;
    }
    n++;
    rig_t r;
    rigSpeed(&r, (uint16_t)gp, gd);
    const autoMvWhy_t why = autoMotionMove(&r.m, &r.in, (int16_t)lroundf(gr * 100.0f), (int16_t)lroundf(gf * 100.0f),
                                           (int16_t)lroundf(gu * 100.0f));
    const float T = fmaxf(r.m.ph.T, r.m.pv.T);
    const bool same = why == AUTO_MVW_NONE && fabsf(T - gT) < 1e-5f && fabsf(r.m.ph.vPk - gv) < 1e-5f;
    *ok += same;
    *ok300 += same && gp == 300;
  }
  fclose(g);
  return n;
}

static void testFastCaps(void)
{
  autoMotionSetMargins(NULL);
  /* the limits of each step at mvacc 30 / mvsettle 1000: 4dea542's table (A_H 0.15 / 0.20 / 0.25 / 0.30, the settle
   * 2000 / 1500 / 1200 / 1000 ms); every other limit as at the defaults */
  static const struct {
    uint16_t pct;
    float aH;
    uint16_t settle;
  } t[] = {{150, 0.15f, 2000}, {200, 0.20f, 1500}, {250, 0.25f, 1200}, {300, 0.30f, 1000}};
  int ok = 0;
  for (unsigned i = 0; i < 4; i++) {
    autoMvSpeed_t d, c;
    autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
    autoMvSpeedCfg(t[i].pct, &d);
    autoMotionSetFastCaps(30, 1000);
    autoMvSpeedCfg(t[i].pct, &c);
    const bool limits = near(c.aH, t[i].aH) && c.settleMaxMs == t[i].settle;
    d.aH = c.aH;
    d.settleMaxMs = c.settleMaxMs;
    ok += limits && memcmp(&c, &d, sizeof(c)) == 0;
    CHECK(limits, "mvacc 30, mvsettle 1000 at %u: A_H %.3f, the settle %u ms (4dea542's)", t[i].pct, c.aH,
          c.settleMaxMs);
  }
  CHECK(ok == 4, "... and every other limit as at the defaults (%d/4)", ok);
  /* 100 never reads them; the clamps; above 30 nothing changes (A_H s is 0.30 at 300) */
  {
    autoMvSpeed_t a, b, c;
    autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
    autoMvSpeedCfg(100, &a);
    autoMotionSetFastCaps(10, 1000);
    autoMvSpeedCfg(100, &b);
    autoMotionSetFastCaps(40, 3000);
    autoMvSpeedCfg(100, &c);
    CHECK(memcmp(&a, &b, sizeof(a)) == 0 && memcmp(&a, &c, sizeof(a)) == 0, "mvspeed 100: mvacc and mvsettle not read");
    autoMvSpeedCfg(300, &c);
    autoMotionSetFastCaps(30, 3000);
    autoMvSpeedCfg(300, &b);
    CHECK(memcmp(&b, &c, sizeof(b)) == 0 && c.aH == 0.10f * 3.0f && c.settleMaxMs == 3000,
          "mvacc 40 at 300: as 30 (A_H s = %.2f); mvsettle 3000: the settle 3000 ms (as at 100)", c.aH);
    autoMotionSetFastCaps(10, 1000);
    autoMvSpeedCfg(300, &c);
    CHECK(c.aH == 0.10f && c.settleMaxMs == 1000, "mvacc 10 at 300: A_H 0.10 (as at 100)");
    uint8_t acc;
    uint16_t settle;
    static const int32_t bad[][4] = {{9, 999, 10, 1000}, {0, 0, 10, 1000}, {-5, -1, 10, 1000}, {41, 3001, 40, 3000},
                                     {255, 65535, 40, 3000}, {100000, 100000, 40, 3000}, {35, 1750, 35, 1750}};
    int okC = 0;
    for (int i = 0; i < 7; i++) {
      autoMotionSetFastCaps(bad[i][0], bad[i][1]);
      autoMotionGetFastCaps(&acc, &settle);
      okC += acc == bad[i][2] && settle == bad[i][3];
    }
    CHECK(okC == 7, "autoMotionSetFastCaps clamps to 10..40 cm/s^2, 1000..3000 ms (%d/7)", okC);
  }
  /* the profiles: 4dea542's golden at mvacc 30 / mvsettle 1000, every row (its 300 % ones the flights'); at the
   * defaults its 250 / 300 rows differ (they plan as 200) */
  autoMotionSetFastCaps(30, 1000);
  int okG = 0, okG300 = 0;
  const int nG = fastGolden4dea542(&okG, &okG300);
  autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
  int okD = 0, okD300 = 0;
  fastGolden4dea542(&okD, &okD300);
  if (nG > 0) {
    CHECK(nG > 400 && okG == nG && okG300 > 70, "mvacc 30, mvsettle 1000: every profile of 4dea542's golden "
          "reproduced (%d/%d rows, %d at 300 %%)", okG, nG, okG300);
    CHECK(okD < nG && okD300 < okG300 / 4, "... the defaults do not (%d/%d, %d at 300 %%)", okD, nG, okD300);
  } else {
    printf("info: tools/tests/mvspeed_golden_4dea542.json not found (run from Firmware/esp-drone): not compared\n");
  }
  /* the design table (docs/autonomy-a2b.md 31) at mvacc 30: 4dea542's 250 / 300 times (1 m forward at D 3, 0.6 m
   * sideways at D 2.5); the settle times out at 1.0 s at 300 (the camera seeing 70 % of the move: SHORT) */
  static const struct {
    uint16_t pct;
    float fwd1, side;
  } tt[] = {{200, 4.50f, 3.464f}, {250, 4.118f, 3.114f}, {300, 3.894f, 2.881f}};
  autoMotionSetFastCaps(30, 1000);
  for (unsigned i = 0; i < 3; i++) {
    rig_t r;
    rigSpeed(&r, tt[i].pct, 3.0f);
    autoMotionMove(&r.m, &r.in, 0, 100, 0);
    const float f1 = r.m.ph.T;
    rigSpeed(&r, tt[i].pct, 2.5f);
    autoMotionMove(&r.m, &r.in, 60, 0, 0);
    CHECK(fabsf(f1 - tt[i].fwd1) < 0.01f && fabsf(r.m.ph.T - tt[i].side) < 0.01f,
          "mvacc 30 at %u: 1 m forward at D 3 %.3f s, 0.6 m sideways at D 2.5 %.3f s (4dea542's)", tt[i].pct, f1,
          r.m.ph.T);
  }
  for (int k = 0; k < 2; k++) {
    rig_t r;
    autoMotionSetFastCaps(k ? 30 : AUTO_MV_ACC_DEFAULT_CM, k ? 1000 : AUTO_MV_FAST_SETTLE_MIN_MS);
    rigSpeed(&r, 300, 2.5f);
    autoMotionMove(&r.m, &r.in, 40, 0, 0);
    for (int i = 0; i < 20000 && r.m.state != AUTO_MV_SETTLE; i++) {
      step(&r);
      r.in.progLat = r.out.refLat * 0.7f;
    }
    const uint32_t t0 = r.in.nowMs;
    for (int i = 0; i < 20000 && active(&r); i++) {
      step(&r);
      r.in.progLat = r.m.latTgt * 0.7f;
    }
    const uint32_t settleMs = r.in.nowMs - t0, want = k ? 1000u : 2500u;
    CHECK(settleMs + 1u >= want && settleMs <= want + 1u && r.m.end == AUTO_MVE_SHORT,
          "300 at mvsettle %u: the settle times out after %u ms (SHORT)", (unsigned)want, (unsigned)settleMs);
  }
  /* a RUNNING motion keeps the settle it was set with at ARM (autoMotionSetSpeed copied it): a set later changes only
   * what the next ARM's speed step copies, and the next command's A_H */
  {
    rig_t r;
    autoMotionSetFastCaps(30, 1000);
    rigSpeed(&r, 300, 2.5f);
    autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
    CHECK(r.m.settleMaxMs == 1000, "the motion memory keeps the latched settle (%u ms)", r.m.settleMaxMs);
  }
  autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
  autoMotionSetMargins(NULL);
}

/* ---- move fixes (the 300 % forensics of 2026-10-17, auto_motion.h "move fixes", docs/autonomy-a2b.md 33) ---------- */

/* The fixes on (fix) or off (0) with the default margins. */
static void useFix(uint8_t fix)
{
  autoMvMargins_t g;
  autoMvMarginsDefault(&g);
  g.fix = fix;
  autoMotionSetMargins(&g);
}

/* Runs a MOVE to its end, the camera's measured forward progress the lagged reference times over (1: ideal). */
static int runSettleOver(rig_t *r, float over, int maxMs)
{
  for (int i = 1; i <= maxMs; i++) {
    step(r);
    if (!active(r)) {
      return i;
    }
    r->in.progFwd = r->out.refFwd * over;
  }
  return -1;
}

/* DRIFT: dTrack follows the hold's measured forward progress since the move's end (DONE: the same epoch; STOPPED: the
 * snap epoch from 0); a command plans with it; the 300 % flight's move 2 (D 1.67 m planned, 2.02 m by the camera);
 * nothing in DRY or with the bit off. */
static void testFixDrift(void)
{
  useFix(AUTO_MVFIX_DEFAULT);
  rig_t r;
  rigInit(&r);
  sceneAt(&r, 2.3f);
  CHECK(autoMotionMove(&r.m, &r.in, 0, 100, 0) == AUTO_MVW_NONE && untilEnd(&r, 20000) > 0 && r.m.end == AUTO_MVE_DONE,
        "forward 1.00 at D 2.3: DONE");
  const float dEnd = r.m.dTrack, lowEnd = r.m.dTrackLow, pEnd = r.m.progFwd;
  CHECK(fabsf(dEnd - 1.3f) < 2e-3f && r.m.dTrackRef == pEnd && fabsf(dEnd - 2.3f * expf(-pEnd)) < 1e-5f,
        "dTrack 1.30 m at the end (%.4f), its reference the move's measured progress %.4f", dEnd, r.m.dTrackRef);
  /* the drone drifts 0.10 ln back (away from the scene) after the end, in the move's epoch */
  r.in.progFwd = pEnd - 0.10f;
  r.in.nowMs += 3000;
  CHECK(autoMotionMove(&r.m, &r.in, 0, 50, 0) == AUTO_MVW_NONE && fabsf(r.m.d0 - dEnd * expf(0.10f)) < 1e-4f &&
            fabsf(r.m.dLow0 - fminf(lowEnd * expf(0.10f), r.m.d0)) < 1e-4f && r.m.dTrackRef == 0.0f &&
            fabsf(r.m.dTrack - r.m.d0) < 1e-6f,
        "0.10 ln of drift back since the end: the next MOVE plans at %.4f m (%.4f x e^0.10), its lower bound with it, "
        "dTrack rebased to the new epoch", r.m.d0, dEnd);
  /* toward the scene in a STOP's snap epoch: the lower bound at k 0.6 */
  rigInit(&r);
  sceneAt(&r, 3.0f);
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  steps(&r, 4000);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  const float dStop = r.m.dTrack, lowStop = r.m.dTrackLow;
  steps(&r, 10);   /* the camera on the snap epoch: progress from 0 */
  r.in.progFwd = 0.05f;
  autoMotionMove(&r.m, &r.in, 0, 20, 0);
  CHECK(fabsf(r.m.d0 - dStop * expf(-0.05f)) < 1e-4f && fabsf(r.m.dLow0 - lowStop * expf(-0.05f / 0.6f)) < 1e-4f,
        "a STOP, then 0.05 ln toward the scene: D %.4f (x e^-0.05), its lower bound x e^(-0.05 / 0.6) %.4f", r.m.d0,
        r.m.dLow0);
  /* the 300 % flight (flight 6 of 2026-10-17): move 1 forward 1.00 at D 2.2993 ended SHORT at 0.3205 ln (dTrack
   * 1.6688 m); at move 2's acceptance the camera's count was 0.1296 ln (2.02 m): back 1.00 is planned there */
  rigInit(&r);
  sceneAt(&r, 2.2993f);
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  CHECK(runSettleOver(&r, 0.3205f / logf(2.2993f / 1.2993f), 20000) > 0 && r.m.end == AUTO_MVE_SHORT &&
            fabsf(r.m.dTrack - 1.6688f) < 2e-3f, "flight 6's move 1: SHORT at 0.3205 ln, dTrack %.4f (the log 1.6688)",
        r.m.dTrack);
  r.in.progFwd = 0.1296f;
  r.in.nowMs += 8500;
  autoMotionMove(&r.m, &r.in, 0, -100, 0);
  CHECK(fabsf(r.m.d0 - 2.2993f * expf(-0.1296f)) < 2e-3f && fabsf(r.m.fwdTgt - logf(r.m.d0 / (r.m.d0 + 1.0f))) < 1e-5f &&
            r.m.fwdTgt > -0.41f,
        "flight 6's move 2: planned at %.3f m (4dea542: 1.669), its target %.4f ln (4dea542: -0.4695, 1.21 m at 2.02 m)",
        r.m.d0, r.m.fwdTgt);
  /* the bit off, DRY: dTrack as it was */
  for (int k = 0; k < 2; k++) {
    useFix(k ? AUTO_MVFIX_DEFAULT : (uint8_t)(AUTO_MVFIX_DEFAULT & ~AUTO_MVFIX_DRIFT));
    rigInit(&r);
    sceneAt(&r, 2.3f);
    r.in.flight = k == 0;
    autoMotionMove(&r.m, &r.in, 0, 100, 0);
    untilEnd(&r, 20000);
    const float d1 = r.m.dTrack;
    r.in.progFwd += 0.10f;
    r.in.nowMs += 1000;
    autoMotionMove(&r.m, &r.in, 0, 20, 0);
    CHECK(r.m.d0 == d1, "%s: the next MOVE at dTrack %.4f as before (%.4f)", k ? "DRY" : "mvfix without DRIFT", d1,
          r.m.d0);
  }
  /* a sideways move keeps dTrack, moved by its own forward drift */
  useFix(AUTO_MVFIX_DEFAULT);
  rigInit(&r);
  sceneAt(&r, 2.3f);
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  untilEnd(&r, 20000);
  autoMotionMove(&r.m, &r.in, 40, 0, 0);
  const float dA = r.m.dTrack;   /* (rebased at the command: the camera's end of the move before) */
  for (int i = 0; i < 30000 && active(&r); i++) {
    step(&r);
    r.in.progFwd = 0.03f;   /* it drifts 0.03 ln forward while it flies sideways */
  }
  CHECK(r.m.end == AUTO_MVE_DONE && fabsf(r.m.dTrack - dA * expf(-0.03f)) < 1e-4f, "a sideways move with 0.03 ln of "
        "forward drift: dTrack %.4f -> %.4f", dA, r.m.dTrack);
}

/* LONG: a settle timed out past its target beyond the tolerance ends LONG (DONE), its measured end the hold's
 * reference; within the tolerance DONE; without the bit DONE as before; SHORT unchanged. */
static void testFixLong(void)
{
  for (int k = 0; k < 2; k++) {
    useFix(k ? (uint8_t)(AUTO_MVFIX_DEFAULT & ~AUTO_MVFIX_LONG) : AUTO_MVFIX_DEFAULT);
    rig_t r;
    rigInit(&r);
    sceneAt(&r, 2.0f);
    autoMotionMove(&r.m, &r.in, 0, -100, 0);   /* the 300 % move 2: back 1.00 */
    const float tgt = r.m.fwdTgt;
    CHECK(runSettleOver(&r, 1.3f, 20000) > 0, "back 1.00 at D 2, the camera 30 %% past the target in the settle: ended");
    const float trav = r.out.progFwdM;
    if (k == 0) {
      CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_LONG && r.m.fwdTgt == r.m.progFwd &&
                fabsf(r.m.fwdTgt - 1.3f * tgt) < 1e-5f && r.out.refFwd == r.m.fwdTgt && trav < -1.35f,
            "mvfix LONG: DONE LONG, the measured end (%.4f ln, %.3f m) the hold's reference (%.4f)", r.m.fwdTgt, trav,
            r.out.refFwd);
      autoMvOut_t idle;
      autoMotionIdleOut(&r.m, &idle);
      CHECK(idle.refFwd == r.m.fwdTgt && strcmp(autoMvEndName(AUTO_MVE_LONG), "LONG") == 0,
            "outside HOVER the measured end too; its name LONG");
    } else {
      CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_DONE && r.m.fwdTgt == tgt,
            "without LONG: DONE, the target kept (the hold pulls back to it)");
    }
  }
  useFix(AUTO_MVFIX_DEFAULT);
  rig_t r;
  rigInit(&r);
  sceneAt(&r, 3.0f);
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  CHECK(runSettleOver(&r, 1.10f, 20000) > 0 && r.m.end == AUTO_MVE_DONE, "forward 1.00, 10 %% past (within 0.04 m + "
        "10 %%): settles DONE");
  rigInit(&r);
  sceneAt(&r, 3.0f);
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  const float tgtF = r.m.fwdTgt;
  CHECK(runSettleOver(&r, 1.25f, 20000) > 0 && r.m.end == AUTO_MVE_LONG && r.out.progFwdM > 1.15f &&
            r.m.fwdTgt == tgtF && r.out.refFwd == tgtF && r.m.progFwd > 1.2f * tgtF &&
            fabsf(r.m.dTrack - 3.0f * expf(-r.m.progFwd)) < 1e-4f,
        "forward 1.00, 25 %% past in ln (%.2f m): LONG reported, the target kept (the review of 2026-10-17: the hold "
        "walks it back from the scene), dTrack at the measured end (%.3f m; DRIFT follows it back)", r.out.progFwdM,
        r.m.dTrack);
  {
    /* ... and the next command's D follows the walk back (DRIFT) */
    r.in.progFwd = tgtF;
    r.in.nowMs += 3000;
    CHECK(autoMotionMove(&r.m, &r.in, 0, 20, 0) == AUTO_MVW_NONE && fabsf(r.m.d0 - 3.0f * expf(-tgtF)) < 2e-3f,
          "back at the target: the next MOVE plans at %.3f m (2.00)", r.m.d0);
  }
  rigInit(&r);
  sceneAt(&r, 2.0f);
  autoMotionMove(&r.m, &r.in, 60, 0, 0);
  for (int i = 0; i < 20000 && active(&r); i++) {
    step(&r);
    r.in.progLat = r.out.refLat * 1.3f;
  }
  CHECK(r.m.end == AUTO_MVE_LONG && r.m.latTgt == r.m.progLat, "right 0.60, 30 %% past: LONG, held where it ended (a "
        "move with no part toward the scene re-anchors)");
  rigInit(&r);
  sceneAt(&r, 3.0f);
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  CHECK(runSettleOver(&r, 0.75f, 20000) > 0 && r.m.end == AUTO_MVE_SHORT, "forward 1.00, 25 %% short: SHORT as before");
  /* a re-view bob after a LONG end ends LONG */
  rigInit(&r);
  sceneAt(&r, 2.0f);
  r.m.viewShift = 0.30f;
  autoMotionMove(&r.m, &r.in, 30, -60, 0);
  CHECK(r.m.review, "a move that ends with a re-view bob");
  for (int i = 0; i < 20000 && active(&r) && !r.m.bob; i++) {
    step(&r);
    r.in.progLat = r.out.refLat * 1.3f;
    r.in.progFwd = r.out.refFwd * 1.3f;
  }
  CHECK(r.m.bob && r.m.doneEnd == AUTO_MVE_LONG, "past its target: the bob runs, its end LONG");
  r.in.sceneTracked = true;
  untilEnd(&r, 20000);
  CHECK(r.m.state == AUTO_MV_DONE && r.m.end == AUTO_MVE_LONG, "the bob's end: DONE LONG (%s)",
        autoMvEndName(r.m.end));
}

/* STILL: a horizontal MOVE in FLIGHT holds its profile at its start while the hold reports moving, at most
 * AUTO_MV_STILL_CAP_MS; DRY, a vertical move, the bit off: no wait. */
static void testFixStill(void)
{
  const uint8_t on = (uint8_t)(AUTO_MVFIX_DEFAULT | AUTO_MVFIX_STILL);
  CHECK((AUTO_MVFIX_DEFAULT & AUTO_MVFIX_STILL) == 0u, "STILL is off by default (the replay of the 300 %% flights)");
  rig_t r;
  rigInit(&r);
  sceneAt(&r, 3.0f);
  r.in.holdMoving = true;
  useFix(AUTO_MVFIX_DEFAULT);
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  CHECK(!r.m.stillWait, "the defaults: no wait");
  useFix(on);
  rigInit(&r);
  sceneAt(&r, 3.0f);
  r.in.holdMoving = true;
  autoMotionMove(&r.m, &r.in, 0, 100, 0);
  CHECK(r.m.stillWait && r.m.state == AUTO_MV_RUN, "STILL on, accepted while the drone moves: RUN, waiting for it");
  float ffMax = 0.0f, refMax = 0.0f;
  for (int i = 0; i < 400; i++) {
    step(&r);
    ffMax = fmaxf(ffMax, fabsf(r.out.ffPitchDeg));
    refMax = fmaxf(refMax, fabsf(r.out.refFwd));
  }
  CHECK(r.m.tRun == 0.0f && ffMax == 0.0f && refMax == 0.0f && r.out.state == AUTO_MV_RUN,
        "0.4 s of a moving drone: the profile at its start (no reference, no lean)");
  r.in.holdMoving = false;
  step(&r);
  CHECK(!r.m.stillWait && r.m.tRun == 0.0f, "still: the wait ends");
  steps(&r, 100);
  CHECK(fabsf(r.m.tRun - 0.100f) < 1e-4f, "then the profile runs (%.3f s)", r.m.tRun);
  /* the cap */
  rigInit(&r);
  sceneAt(&r, 3.0f);
  r.in.holdMoving = true;
  autoMotionMove(&r.m, &r.in, 40, 0, 0);
  const uint32_t t0 = r.in.nowMs;
  int waited = 0;
  for (int i = 0; i < 3000 && r.m.stillWait; i++) {
    step(&r);
    waited = (int)(r.in.nowMs - t0);
  }
  CHECK(!r.m.stillWait && waited >= (int)AUTO_MV_STILL_CAP_MS && waited <= (int)AUTO_MV_STILL_CAP_MS + 1,
        "a drone that never comes still: the profile starts after %d ms (cap %u)", waited, AUTO_MV_STILL_CAP_MS);
  CHECK(untilEnd(&r, 20000) > 0 && r.m.end == AUTO_MVE_DONE, "... and ends DONE");
  /* the cap counts from the acceptance: a view flicker (PAUSED -> RUN) does not restart it (the review of 2026-10-17) */
  rigInit(&r);
  sceneAt(&r, 3.0f);
  r.in.holdMoving = true;
  autoMotionMove(&r.m, &r.in, 40, 0, 0);
  const uint32_t t1 = r.in.nowMs;
  waited = 0;
  for (int i = 0; i < 4000 && r.m.stillWait; i++) {
    r.in.holdApplied = (i % 500) >= 100;   /* 0.1 s lost every 0.5 s */
    step(&r);
    waited = (int)(r.in.nowMs - t1);
  }
  r.in.holdApplied = true;
  CHECK(!r.m.stillWait && waited <= (int)AUTO_MV_STILL_CAP_MS + 1, "a view lost 0.1 s in every 0.5 s while it waits: "
        "the profile starts %d ms after the acceptance (cap %u)", waited, AUTO_MV_STILL_CAP_MS);
  /* DRY, a vertical move, the bit off */
  for (int k = 0; k < 3; k++) {
    useFix(k == 2 ? (uint8_t)(on & ~AUTO_MVFIX_STILL) : on);
    rigInit(&r);
    sceneAt(&r, 3.0f);
    r.in.holdMoving = true;
    r.in.flight = k != 0;
    autoMotionMove(&r.m, &r.in, 0, k == 1 ? 0 : 100, k == 1 ? 20 : 0);
    step(&r);
    step(&r);
    CHECK(!r.m.stillWait && r.m.tRun > 0.0f, "%s: no wait", k == 0 ? "DRY" : (k == 1 ? "a vertical move" :
                                                                            "mvfix without STILL"));
  }
  useFix(AUTO_MVFIX_DEFAULT);
}

static void testMoveFixes(void)
{
  testFixDrift();
  testFixLong();
  testFixStill();
  autoMotionSetMargins(NULL);
}

static void testLateYawScaleReset(void)
{
  rig_t r;
  rigInit(&r);
  autoMotionSetSpeed(&r.m, 300);
  for (int i = 0; i < 3; i++) {
    CHECK(autoMotionYaw(&r.m, &r.in, -6) == AUTO_MVW_NONE, "small turn accepted");
    CHECK(untilEnd(&r, 5000) > 0 && r.m.end == AUTO_MVE_DONE, "small turn DONE");
  }
  steps(&r, 3000); /* the recorded reset followed DONE rather than its dwell */
  const float heading = r.out.yawOffDeg, h0 = r.in.vzH;
  const uint16_t seq = r.m.seq;
  r.in.sceneTracked = false;
  r.in.sceneResets++;
  r.in.camHeight = false;
  step(&r);
  CHECK(active(&r) && r.m.bob && r.out.camGuessOk, "late reset starts bounded re-track");
  CHECK(r.m.seq == seq && r.out.yawOffDeg == heading, "no extra command or heading turn");
  r.in.camHeight = true; /* only the existing healthy measured-G guess is allowed */
  float maxDz = 0.0f, maxV = 0.0f;
  int ended = -1;
  for (int i = 0; i < 12000; i++) {
    if (i == 1200) { r.in.sceneTracked = true; r.in.sceneDM = 1.5f; }
    step(&r);
    maxDz = fmaxf(maxDz, fabsf(r.in.vzH - h0));
    maxV = fmaxf(maxV, fabsf(r.out.vRef));
    if (!active(&r)) { ended = i; break; }
  }
  CHECK(ended >= 1200 && ended < 10000 && r.m.end == AUTO_MVE_DONE, "re-track completes after new G");
  CHECK(maxDz < 0.10f && maxV <= AUTO_MV_UP_VREF_MAX + 1e-5f, "existing bounded bob, dz %.3f v %.3f", maxDz, maxV);
  steps(&r, 1000);
  CHECK(!active(&r), "same reset cannot restart recovery forever");
  for (int condition = 0; condition < 5; condition++) {
    rigInit(&r);
    autoMotionYaw(&r.m, &r.in, -6);
    untilEnd(&r, 5000);
    r.in.sceneTracked = false;
    r.in.sceneResets++;
    if (condition == 0) r.in.holdApplied = false;
    if (condition == 1) r.in.battLow = true;
    if (condition == 2) r.in.tiltDeg = 12;
    if (condition == 3) r.in.simPlant = true;
    if (condition == 4) r.m.state = AUTO_MV_STOPPED;
    step(&r);
    CHECK(!active(&r), "no recovery with invalid view/battery/tilt/sim/stopped (%d)", condition);
  }
  rigInit(&r);
  autoMotionYaw(&r.m, &r.in, -6);
  untilEnd(&r, 5000);
  r.in.sceneTracked = false; r.in.sceneResets++;
  step(&r);
  int timeout = untilEnd(&r, 25000); /* no new G: same finite undo/re-track path */
  CHECK(timeout > 0, "no-scale recovery remains bounded (%d)", timeout);
}

int main(int argc, char **argv)
{
  if (argc == 3 && strcmp(argv[1], "--write-golden") == 0) {
    testSpeedGolden(argv[2]);
    return 0;
  }
  /* the margins of 2026-10-17 (the defaults): their own test and the golden profiles (the accepted moves) */
  autoMotionSetMargins(NULL);
  testGuardMargins();
  testSpeedGolden(NULL);
  testFastCapsIdentity();   /* (first: it checks the pair as initialised) */
  testGuardsOff();          /* (before any autoMotionSetFwdRate: it checks the rate as initialised) */
  testViewDeb();            /* (before any autoMotionSetViewDeb: it checks the VIEW_FAST timer as initialised) */
  testFastCaps();
  testSpeedIdentity();
  testFfRoll();
  testMoveFixes();   /* (the move fixes of 2026-10-17's 300 % forensics: on by default) */
  /* The tests written before 2026-10-17 check the margins of before (0.50 / 0.40 m, 0.15 m + 30 %, 0.25 m sideways,
   * the plan 0.60 m) by their numbers: they run with them (autoMvMarginsOld; the settings of before but the plan's
   * 55), the defaults are testGuardMargins' */
  setOldMargins(NULL);
  testSpeedCfg();
  testPeakRule();
  testSpeedTimes();
  testViewFast();
  testFastMargins();
  testLn();
  testProfile();
  testMoveConversions();
  testLag();
  testRefuse();
  testPauseAndGuards();
  testTopSpeeds();
  testStopEpochs();
  testVertical();
  testYaw();
  testIdleKeepsDoneMove();
  testWhichD();
  testKFloor();
  testReview();
  testDriftFloor();
  testFrozen();
  testSettleGuard();
  testTravel();
  testStopDuringRetrack();
  testHoffRange();
  testLongRange();
  testFloorSweep();
  testLongProfiles();
  testLatGuardMetres();
  testFrontGuard();
  testDiagonalRate();
  testFwdLn();
  testPlanFloor();
  testHeightOneSided();
  testBlindTurn();
  testStopKinds();
  testContactBackOff();
  testSetHeight();
  testLateYawScaleReset();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
