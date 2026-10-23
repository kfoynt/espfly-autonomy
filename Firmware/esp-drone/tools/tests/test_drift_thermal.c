/*
 * Closed-loop sim of the thermal gyro drift in a long camera-held hover (docs/autonomy-a2b.md section 19.1-19.2):
 * the real attitude filter (modules/src/sensfusion6.c, 250 Hz) and the real camera drift hold (utils/src/
 * camera_hold.c camHoldStep, 14.8 fps), with the gyro temperature compensation (gyro_tempco.h) modelled as a
 * correction of the bias and the in-flight bias learning (twoKiAir) on or off.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/modules/interface \
 *      -Icomponents/core/crazyflie/utils/interface tools/tests/test_drift_thermal.c \
 *      components/core/crazyflie/utils/src/camera_hold.c -lm -o /tmp/test_drift_thermal && /tmp/test_drift_thermal
 *
 * Model (2026-10-14 flight 4, 43 -> 33 C in 48.5 s):
 *   gyro      = true rate + b(t) - corr(t), b_y(t) = 0.86 (1 - e^(-t/31)) deg/s, b_x = -0.3 b_y
 *               (nose-up rate = -gy, roll rate (right side down) = +gx, the firmware's axes)
 *   corr      = 0 (no compensation) or (1 + e) b(t) (A: e = 0 exact, +-0.3 a 30 % coefficient error)
 *   acc       = the true specific force in the body frame (tilt, the thrust's horizontal acceleration, drag
 *               mu 0.5 /s): ax = -mu v_fwd cos(theta) / g, ay = mu v_right cos(phi) / g, az ~ 1 / (cos theta cos phi)
 *   control   = the estimate tracks the setpoint (the trim, 0 here, plus the hold) with a 0.08 s lag
 *   plant     = v_fwd' = -g tan(theta) - mu v_fwd, v_right' = g tan(phi) - mu v_right
 *   camera    = wFwd = v_fwd / D + N(0, 0.036 /s), wLat = v_right / D + N(0, 0.036 /s), D 2.65 m, the mean over each
 *               67.5 ms frame, its hold output applied 60 ms after the frame (the flow loop's processing)
 * Checks: the baseline (14bcfc6: no compensation, twoKiAir 0) reproduces flight 4 (1.5-2.1 m backward, hold pitch
 * -1.1..-1.7 deg at 45 s); A (exact and +-30 %) with B: |drift| < 0.5 m on both axes and the hold's mean pitch and
 * roll within +-0.4 deg over 120 s. A alone and B alone are reported.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

#include "camera_hold.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define G_MS2      9.80665
#define MU         0.5
#define D_M        2.65
#define FRAME_S    0.0675
#define PROC_S     0.060
#define CTRL_TAU_S 0.08
#define NOISE_W    0.036

static uint32_t rng = 0x9E3779B9u;

static double uni(void)
{
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  return ((rng >> 8) + 0.5) * (1.0 / 16777216.0);
}

static double gauss(void)
{
  return sqrt(-2.0 * log(uni())) * cos(2.0 * M_PI * uni());
}

typedef struct {
  const char *name;
  bool compA;        /* the temperature compensation */
  double aErr;       /* its coefficient error (0.3: 30 % too large) */
  float twoKiAir;    /* B: 0 = 14bcfc6 */
  double secs;
} simCfg_t;

typedef struct {
  double xFwd45, xFwdEnd, xRightEnd, maxAbsFwd, maxAbsRight;
  double holdPitch45, holdRoll45;
  double holdPitchMean, holdRollMean;   /* over the run after the 1 s settle */
  double estErrPitch45;                 /* estimate - true at 45 s, deg */
  float learnedEnd[3];
} simRes_t;

static double biasY(double t)
{
  return 0.86 * (1.0 - exp(-t / 31.0));
}

static void sim(const simCfg_t *c, simRes_t *r)
{
  memset(r, 0, sizeof(*r));
  rng = 0x9E3779B9u;
  qw = 1.0f;
  qx = qy = qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  sensfusion6SetKiAir(c->twoKiAir);
  sensfusion6SetAirborne(true);
  camHoldState_t hold;
  camHoldReset(&hold);
  const double dt = 0.001;
  double theta = 0.0, phi = 0.0;              /* true pitch (nose up) and roll (right side down), deg */
  double vF = 0.0, vR = 0.0, xF = 0.0, xR = 0.0;
  double thetaRate = 0.0, phiRate = 0.0;      /* deg/s */
  double estP = 0.0, estR = 0.0;              /* the filter's latest estimate, deg */
  double holdP = 0.0, holdR = 0.0;            /* applied hold correction, deg */
  double pendP = 0.0, pendR = 0.0, pendAt = -1.0;
  double frameStart = 0.0, sumVF = 0.0, sumVR = 0.0;
  int frameN = 0;
  double sumHP = 0.0, sumHR = 0.0;
  long nH = 0;
  const long steps = (long)(c->secs / dt);
  for (long i = 0; i < steps; i++) {
    const double t = i * dt;
    /* the attitude loop: the estimate follows the setpoint (trim 0 + hold) */
    thetaRate = (holdP - estP) / CTRL_TAU_S;
    phiRate = (holdR - estR) / CTRL_TAU_S;
    theta += thetaRate * dt;
    phi += phiRate * dt;
    const double th = theta * M_PI / 180.0, ph = phi * M_PI / 180.0;
    vF += (-G_MS2 * tan(th) - MU * vF) * dt;
    vR += (G_MS2 * tan(ph) - MU * vR) * dt;
    xF += vF * dt;
    xR += vR * dt;
    /* the attitude filter at 250 Hz */
    if (i % 4 == 0) {
      const double by = biasY(t), bx = -0.3 * by;
      const double corrY = c->compA ? (1.0 + c->aErr) * by : 0.0, corrX = c->compA ? (1.0 + c->aErr) * bx : 0.0;
      const float gx = (float)(phiRate + bx - corrX);
      const float gy = (float)(-thetaRate + by - corrY);
      const float ax = (float)(-MU * vF * cos(th) / G_MS2);
      const float ay = (float)(MU * vR * cos(ph) / G_MS2);
      const float az = (float)(1.0 / (cos(th) * cos(ph)));
      sensfusion6UpdateQ(gx, gy, 0.0f, ax, ay, az, 0.004f);
      float er, ep, ey;
      sensfusion6GetEulerRPY(&er, &ep, &ey);
      estP = ep;
      estR = er;
    }
    /* the camera: the frame's mean velocity, its hold update applied PROC_S later */
    sumVF += vF;
    sumVR += vR;
    frameN++;
    if (t - frameStart >= FRAME_S) {
      camHoldInput_t in = {
        .enabled = true, .sticksCentred = true, .airborne = true, .viewOk = true,
        .wFwd = (float)(sumVF / frameN / D_M + NOISE_W * gauss()),
        .wLat = (float)(sumVR / frameN / D_M + NOISE_W * gauss()),
        .dt = (float)FRAME_S,
      };
      camHoldStep(&hold, &in);
      pendP = hold.pitch;
      pendR = hold.roll;
      pendAt = t + PROC_S;
      frameStart = t;
      sumVF = sumVR = 0.0;
      frameN = 0;
    }
    if (pendAt >= 0.0 && t >= pendAt) {
      holdP = pendP;
      holdR = pendR;
      pendAt = -1.0;
    }
    if (t >= 1.0) {
      sumHP += holdP;
      sumHR += holdR;
      nH++;
    }
    r->maxAbsFwd = fmax(r->maxAbsFwd, fabs(xF));
    r->maxAbsRight = fmax(r->maxAbsRight, fabs(xR));
    if (i == (long)(45.0 / dt)) {
      r->xFwd45 = xF;
      r->holdPitch45 = holdP;
      r->holdRoll45 = holdR;
      r->estErrPitch45 = estP - theta;
    }
  }
  r->xFwdEnd = xF;
  r->xRightEnd = xR;
  r->holdPitchMean = sumHP / nH;
  r->holdRollMean = sumHR / nH;
  sensfusion6GetLearnedBiasDps(r->learnedEnd);
  sensfusion6SetAirborne(false);
}

static void print(const simCfg_t *c, const simRes_t *r)
{
  printf("%-28s 45 s: fwd %+.2f m, hold pitch %+.2f roll %+.2f deg, est err pitch %+.2f | %.0f s: fwd %+.2f right %+.2f m "
         "(max |fwd| %.2f |right| %.2f), hold mean pitch %+.2f roll %+.2f deg, learned %+.2f %+.2f deg/s\n",
         c->name, r->xFwd45, r->holdPitch45, r->holdRoll45, r->estErrPitch45, c->secs, r->xFwdEnd, r->xRightEnd,
         r->maxAbsFwd, r->maxAbsRight, r->holdPitchMean, r->holdRollMean, r->learnedEnd[0], r->learnedEnd[1]);
}

int main(void)
{
  simRes_t r;

  /* the baseline: 14bcfc6 (no compensation, ground Ki only) */
  const simCfg_t base = { "14bcfc6", false, 0.0, 0.0f, 120.0 };
  sim(&base, &r);
  print(&base, &r);
  CHECK(r.xFwd45 <= -1.5 && r.xFwd45 >= -2.1, "baseline: %+.2f m backward at 45 s (flight 4: 1.5-2.1)", r.xFwd45);
  CHECK(r.holdPitch45 <= -1.1 && r.holdPitch45 >= -1.7, "baseline: hold pitch %+.2f deg at 45 s (flight 4 ~-1.4)",
        r.holdPitch45);
  CHECK(r.xRightEnd > 0.3, "baseline: drifts right too (%+.2f m)", r.xRightEnd);

  /* A alone, B alone (reported) */
  const simCfg_t aOnly = { "A exact, no B", true, 0.0, 0.0f, 120.0 };
  sim(&aOnly, &r);
  print(&aOnly, &r);
  CHECK(r.maxAbsFwd < 0.5 && r.maxAbsRight < 0.5, "A exact: |drift| %.2f / %.2f m", r.maxAbsFwd, r.maxAbsRight);
  const simCfg_t bOnly = { "B alone (twoKiAir 0.040)", false, 0.0, 0.040f, 120.0 };
  sim(&bOnly, &r);
  print(&bOnly, &r);
  simRes_t r0;
  sim(&base, &r0);
  CHECK(fabs(r.xFwd45) < 0.6 * fabs(r0.xFwd45), "B alone halves the drift at 45 s at least: %+.2f vs %+.2f m",
        r.xFwd45, r0.xFwd45);

  /* A + B, the build: exact and +-30 % on the coefficients */
  const simCfg_t ab[3] = {
    { "A exact + B", true, 0.0, 0.040f, 120.0 },
    { "A +30 % + B", true, 0.3, 0.040f, 120.0 },
    { "A -30 % + B", true, -0.3, 0.040f, 120.0 },
  };
  for (int k = 0; k < 3; k++) {
    sim(&ab[k], &r);
    print(&ab[k], &r);
    CHECK(r.maxAbsFwd < 0.5 && r.maxAbsRight < 0.5, "%s: |drift| %.2f fwd / %.2f right m over 120 s", ab[k].name,
          r.maxAbsFwd, r.maxAbsRight);
    CHECK(fabs(r.holdPitchMean) <= 0.4 && fabs(r.holdRollMean) <= 0.4, "%s: hold mean pitch %+.2f roll %+.2f deg",
          ab[k].name, r.holdPitchMean, r.holdRollMean);
  }
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
