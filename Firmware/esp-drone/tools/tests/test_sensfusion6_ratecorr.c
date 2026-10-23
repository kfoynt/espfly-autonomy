/*
 * Host test: sensfusion6's exported rate correction (sensfusion6RateCorr) and
 * its path into the camera height observer's gyro-bias estimate
 * (cam_vz_shadow.c gyroBias): the axes and the sign the firmware relies on.
 * And the in-flight bias learning (twoKiAir, docs/autonomy-a2b.md 19.2): a
 * 0.6 deg/s bias learned in the air (error < 0.1 deg after 60 s), the ground
 * unchanged, a 0.015 deg/s/s ramp (error <= 0.8 deg), the |a| and rate
 * gates, the 1.5 deg/s clamp, the zeroing at a new ground bias, and twoKiAir 0
 * bit-identical to the filter of 14bcfc6 (a hash of every update over a
 * synthetic sequence that crosses every gate, pinned from 14bcfc6's
 * sensfusion6.c: build with -DSF6_OLD -DSF6_SRC='"<path of the old file>"'
 * to print it); twoKiAir 0.040 bit-identical to 0ed68c9's learning (its hash,
 * pinned: the loop-rate fix's inline setters and clamp, a2b 19.11).
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Icomponents/core/crazyflie/modules/interface \
 *      -Icomponents/core/crazyflie/utils/interface tools/tests/test_sensfusion6_ratecorr.c \
 *      components/core/crazyflie/utils/src/cam_vz_shadow.c components/core/crazyflie/utils/src/cam_vz_obs.c \
 *      -lm -o /tmp/test_sf6 && /tmp/test_sf6
 *
 * sensfusion6.c is included directly: its log/param tables compile away, and
 * its invSqrt() type-puns a float through `long`, 32 bits on the ESP32 but 64
 * on a 64-bit host, so `long` is int32_t inside it.
 */
#include <math.h>
#include <stdbool.h>
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
#ifndef SF6_SRC
#define SF6_SRC "../../components/core/crazyflie/modules/src/sensfusion6.c"
#endif
#include SF6_SRC
#undef long

#include "cam_vz_shadow.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static camVzShadow_t sh;

/* the true attitude, integrated like sensfusion6 without feedback */
static double tw, tx, ty, tz;

static void trueStep(double wx, double wy, double dt)
{
  wx *= 0.5 * dt;
  wy *= 0.5 * dt;
  const double a = tw, b = tx, c = ty;
  tw += -b * wx - c * wy;
  tx += a * wx - tz * wy;
  ty += a * wy + tz * wx;
  tz += b * wy - c * wx;
  const double n = sqrt(tw * tw + tx * tx + ty * ty + tz * tz);
  tw /= n;
  tx /= n;
  ty /= n;
  tz /= n;
}

/*
 * gyro biases (deg/s) on x and y; flight: the accelerometer sees only the
 * thrust axis (a multirotor in flight) while the drone rocks in roll and
 * pitch, else it rests and sees gravity. loopHz: the stabilizer loop rate
 * (the estimator always integrates its nominal 4 ms). The glue's work is
 * mirrored: every 1 kHz sample adds the latest correction to a block that is
 * handed to camVzShadowStep after >= 8.5 ms (cam_vz_glue.c). Returns the
 * shadow's estimate (deg/s) averaged over the last half.
 */
static void run(double bxDps, double byDps, bool flight, double loopHz, double secs, double est[2], double corr[2])
{
  qw = 1.0f;
  qx = qy = qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  sensfusion6RateCorr[0] = sensfusion6RateCorr[1] = sensfusion6RateCorr[2] = 0.0f;
  tw = 1.0;
  tx = ty = tz = 0.0;
  camVzShadowInit(&sh, NULL);
  const double dt = 1.0 / loopHz;
  double sum[2] = { 0.0, 0.0 }, sumE[2] = { 0.0, 0.0 };
  int n = 0, nE = 0;
  uint64_t blk0 = 1000000;
  const long steps = (long)(secs * loopHz);
  for (long i = 0; i < steps; i++) {
    const double t = i * dt;
    const double wx = flight ? 0.3 * sin(2.0 * M_PI * 0.7 * t) : 0.0;
    const double wy = flight ? 0.4 * sin(2.0 * M_PI * 0.5 * t + 1.0) : 0.0;
    trueStep(wx, wy, dt);
    float ax = 0.0f, ay = 0.0f, az = 1.0f;
    if (!flight) {
      ax = (float)(2.0 * (tx * tz - tw * ty));
      ay = (float)(2.0 * (tw * tx + ty * tz));
      az = (float)(tw * tw - tx * tx - ty * ty + tz * tz);
    }
    if (i % 4 == 0) {
      sensfusion6UpdateQ((float)(wx * 180.0 / M_PI + bxDps), (float)(wy * 180.0 / M_PI + byDps), 0.0f, ax, ay, az,
                         0.004f);
    }
    sum[0] += sensfusion6RateCorr[0];
    sum[1] += sensfusion6RateCorr[1];
    n++;
    const uint64_t us = 1000000 + (uint64_t)llround(t * 1e6);
    if (us - blk0 >= 8500) {
      const camVzShadowIn_t in = { .timeUs = us, .accUpG = 1.0f, .thrust = 20000.0f, .groundStart = i < 20,
                                   .rateCorr = { (float)(sum[0] / n), (float)(sum[1] / n), 0.0f } };
      camVzShadowStep(&sh, &in);
      sum[0] = sum[1] = 0.0;
      n = 0;
      blk0 = us;
      if (t > secs / 2) {
        sumE[0] += sh.gyroBias[0];
        sumE[1] += sh.gyroBias[1];
        nE++;
      }
    }
  }
  for (int k = 0; k < 2; k++) {
    est[k] = sumE[k] / nE * 180.0 / M_PI;
    corr[k] = sensfusion6RateCorr[k] * 180.0 / M_PI;
  }
}

/* ---- the in-flight bias learning ------------------------------------------------------------------------- */

static void sf6Reset(void)
{
  qw = 1.0f;
  qx = qy = qz = 0.0f;
  integralFBx = integralFBy = integralFBz = 0.0f;
  sensfusion6RateCorr[0] = sensfusion6RateCorr[1] = sensfusion6RateCorr[2] = 0.0f;
}

static double degOf(double r)
{
  return r * 180.0 / M_PI;
}

static __attribute__((unused)) void trueEuler(double *roll, double *pitch)
{
  *pitch = degOf(asin(2.0 * (tx * tz - tw * ty)));
  *roll = degOf(atan2(2.0 * (tw * tx + ty * tz), tw * tw - tx * tx - ty * ty + tz * tz));
}

static uint64_t hashMix(uint64_t h, const void *p, size_t n)
{
  const uint8_t *b = (const uint8_t *)p;
  for (size_t i = 0; i < n; i++) {
    h ^= b[i];
    h *= 0x100000001B3ull;   /* FNV-1a 64 */
  }
  return h;
}

/* A deterministic 250 Hz sequence crossing every gate: rest, rocking, fast square-wave rates, |a| off 1 g,
 * biases and ramps; the airborne flag toggles (new filter only). Every update's quaternion and rate correction
 * are hashed. */
static uint64_t identityHash(void)
{
  sf6Reset();
  tw = 1.0;
  tx = ty = tz = 0.0;
  uint64_t h = 0xCBF29CE484222325ull;
  const int steps = 250 * 200;
  for (int i = 0; i < steps; i++) {
    const double t = i * 0.004;
    const int phase = (i / 2500) % 8;
    double wx = 0.25 * sin(2.0 * M_PI * 0.7 * t), wy = 0.35 * sin(2.0 * M_PI * 0.45 * t + 1.0);
    if (phase == 3) {
      wx = ((i / 25) % 2) ? 0.9 : -0.9;   /* 52 deg/s square wave */
    }
    trueStep(wx, wy, 0.004);
    double scale = 1.0;
    if (phase == 5) {
      scale = 1.2;
    } else if (phase == 6) {
      scale = 0.85 + 0.1 * sin(t);
    }
    const float ax = (float)(scale * 2.0 * (tx * tz - tw * ty));
    const float ay = (float)(scale * 2.0 * (tw * tx + ty * tz));
    const float az = (float)(scale * (tw * tw - tx * tx - ty * ty + tz * tz));
    const double by = 0.6 + 0.01 * t, bx = -0.2;
#ifndef SF6_OLD
    sensfusion6SetAirborne(phase != 0 && phase != 7);
#endif
    sensfusion6UpdateQ((float)degOf(wx) + (float)bx, (float)degOf(wy) + (float)by, 0.3f, ax, ay, az, 0.004f);
    const float v[7] = { qw, qx, qy, qz, sensfusion6RateCorr[0], sensfusion6RateCorr[1], sensfusion6RateCorr[2] };
    h = hashMix(h, v, sizeof(v));
  }
  return h;
}

/* The hash of identityHash() through 14bcfc6's sensfusion6.c (on this host's compiler). */
#define SF6_IDENTITY_HASH_14BCFC6 0x34A74C6E637A9DA3ull
/* ... and through 0ed68c9's at twoKiAir 0.040 (the learning before the loop-rate fix, docs/autonomy-a2b.md 19.11:
 * the inline airborne / epoch setters and the inline clamp must not change a bit). */
#define SF6_KIAIR_HASH_0ED68C9 0x44EA30DC66EFEB2Full

#ifndef SF6_OLD
typedef struct {
  double bx, by;        /* deg/s */
  double rampY;         /* deg/s per s added to by */
  bool air;
  double accScale;      /* the accelerometer's magnitude */
  bool fastRates;       /* a +-52 deg/s square wave on x */
  double secs;
} bScen_t;

/* Mean estimate minus true attitude (deg) over the last 5 s; the learned bias at the end. */
static void bScen(const bScen_t *s, double *errRoll, double *errPitch, float learned[3])
{
  sf6Reset();
  tw = 1.0;
  tx = ty = tz = 0.0;
  sensfusion6SetAirborne(s->air);
  const int steps = (int)(s->secs * 250.0);
  double sr = 0.0, sp = 0.0;
  int n = 0;
  for (int i = 0; i < steps; i++) {
    const double t = i * 0.004;
    double wx = 0.2 * sin(2.0 * M_PI * 0.7 * t), wy = 0.3 * sin(2.0 * M_PI * 0.5 * t + 1.0);
    if (s->fastRates) {
      wx = ((i / 25) % 2) ? 0.9 : -0.9;
    }
    trueStep(wx, wy, 0.004);
    const float ax = (float)(s->accScale * 2.0 * (tx * tz - tw * ty));
    const float ay = (float)(s->accScale * 2.0 * (tw * tx + ty * tz));
    const float az = (float)(s->accScale * (tw * tw - tx * tx - ty * ty + tz * tz));
    const double by = s->by + s->rampY * t;
    sensfusion6UpdateQ((float)(degOf(wx) + s->bx), (float)(degOf(wy) + by), 0.0f, ax, ay, az, 0.004f);
    if (t >= s->secs - 5.0) {
      float r, p, y;
      sensfusion6GetEulerRPY(&r, &p, &y);
      double tr, tp;
      trueEuler(&tr, &tp);
      sr += r - tr;
      sp += p - tp;
      n++;
    }
  }
  *errRoll = sr / n;
  *errPitch = sp / n;
  sensfusion6GetLearnedBiasDps(learned);
}

static void quatOf(float q[4])
{
  q[0] = qw;
  q[1] = qx;
  q[2] = qy;
  q[3] = qz;
}

static void testKiAir(void)
{
  double er, ep;
  float lb[3];
  CHECK(sensfusion6GetKiAir() == 0.040f, "default twoKiAir 0.040, got %.4f", sensfusion6GetKiAir());

  /* 0.6 deg/s in the air: learned, error < 0.1 deg after 60 s */
  bScen_t s = { .bx = -0.2, .by = 0.6, .air = true, .accScale = 1.0, .secs = 60.0 };
  bScen(&s, &er, &ep, lb);
  printf("air, bias (-0.20, +0.60) deg/s, 60 s: error roll %+.3f pitch %+.3f deg, learned %+.3f %+.3f deg/s\n", er, ep,
         lb[0], lb[1]);
  CHECK(fabs(ep) < 0.1 && fabs(er) < 0.1, "air 60 s: error %+.3f %+.3f deg", er, ep);
  CHECK(fabs(lb[1] - 0.6) < 0.05 && fabs(lb[0] + 0.2) < 0.05, "learned %+.3f %+.3f", lb[0], lb[1]);
  sensfusion6SetKiAir(0.0f);
  bScen(&s, &er, &ep, lb);
  printf("  the same without it (twoKiAir 0): error pitch %+.3f deg (bias / Kp %.2f)\n", ep, 0.6 / 0.4);
  CHECK(ep < -1.2, "without it the error stays ~ bias / Kp: %+.3f", ep);
  sensfusion6SetKiAir(0.040f);

  /* on the ground: bit-identical to twoKiAir 0 */
  {
    bScen_t g = s;
    g.air = false;
    float q1[4], q0[4];
    bScen(&g, &er, &ep, lb);
    quatOf(q1);
    sensfusion6SetKiAir(0.0f);
    bScen(&g, &er, &ep, lb);
    quatOf(q0);
    sensfusion6SetKiAir(0.040f);
    CHECK(memcmp(q1, q0, sizeof(q0)) == 0, "ground: bit-identical to twoKiAir 0");
  }

  /* a ramp of 0.015 deg/s/s (flight 4's whole cooling) for 50 s: error <= 0.8 deg (r / Ki = 0.75) */
  {
    bScen_t r = { .by = 0.0, .rampY = 0.015, .air = true, .accScale = 1.0, .secs = 50.0 };
    bScen(&r, &er, &ep, lb);
    printf("air, ramp 0.015 deg/s/s, 50 s: error pitch %+.3f deg (bias then %.2f deg/s)\n", ep, 0.015 * 50.0);
    CHECK(fabs(ep) <= 0.8, "ramp error %+.3f deg", ep);
    sensfusion6SetKiAir(0.0f);
    double ep0;
    bScen(&r, &er, &ep0, lb);
    sensfusion6SetKiAir(0.040f);
    printf("  without it: %+.3f deg\n", ep0);
    CHECK(fabs(ep) < 0.6 * fabs(ep0), "the ramp error shrinks: %+.3f vs %+.3f", ep, ep0);
  }

  /* the gates: |a| 1.2 g, or 52 deg/s on x: the slow Ki, bit-identical to twoKiAir 0 */
  for (int k = 0; k < 2; k++) {
    bScen_t gte = s;
    if (k == 0) {
      gte.accScale = 1.2;
    } else {
      gte.fastRates = true;
    }
    float q1[4], q0[4];
    bScen(&gte, &er, &ep, lb);
    quatOf(q1);
    sensfusion6SetKiAir(0.0f);
    bScen(&gte, &er, &ep, lb);
    quatOf(q0);
    sensfusion6SetKiAir(0.040f);
    CHECK(memcmp(q1, q0, sizeof(q0)) == 0, "gate %s: the slow Ki", k == 0 ? "|a| 1.2 g" : "52 deg/s");
  }
  {
    /* inside the band (|a| 1.05 g) it learns */
    bScen_t in = s;
    in.accScale = 1.05;
    bScen(&in, &er, &ep, lb);
    CHECK(fabs(ep) < 0.1, "|a| 1.05 g learns: %+.3f", ep);
  }

  /* the clamp: 3 deg/s in the air for 120 s, the integral stops at 1.5 deg/s */
  {
    bScen_t c = { .by = 3.0, .air = true, .accScale = 1.0, .secs = 120.0 };
    bScen(&c, &er, &ep, lb);
    printf("air, 3 deg/s: learned %+.4f deg/s (clamp 1.5), error pitch %+.2f deg\n", lb[1], ep);
    CHECK(lb[1] <= 1.5012f && lb[1] >= 1.49f, "clamped at 1.5 deg/s: %+.4f", lb[1]);
  }

  /* zeroing at a new ground bias */
  bScen(&s, &er, &ep, lb);
  sensfusion6NoteGyroBiasEpoch(7);
  sensfusion6GetLearnedBiasDps(lb);
  CHECK(fabsf(lb[1] - 0.6f) < 0.05f, "the first epoch note only sets it");
  sensfusion6NoteGyroBiasEpoch(7);
  sensfusion6GetLearnedBiasDps(lb);
  CHECK(fabsf(lb[1] - 0.6f) < 0.05f, "the same epoch: kept");
  sensfusion6NoteGyroBiasEpoch(8);
  sensfusion6GetLearnedBiasDps(lb);
  CHECK(lb[0] == 0.0f && lb[1] == 0.0f && lb[2] == 0.0f, "a re-zero zeroes it");
  bScen(&s, &er, &ep, lb);
  sensfusion6SetKiAir(0.0f);
  sensfusion6NoteGyroBiasEpoch(9);
  sensfusion6GetLearnedBiasDps(lb);
  CHECK(lb[1] != 0.0f, "twoKiAir 0: a re-zero leaves the integral (as 14bcfc6)");
  sensfusion6SetKiAir(0.040f);

  /* the setter's range */
  CHECK(!sensfusion6SetKiAir(0.21f) && !sensfusion6SetKiAir(-0.01f) && !sensfusion6SetKiAir(NAN) &&
        sensfusion6GetKiAir() == 0.040f, "range 0..0.2");
  sensfusion6SetAirborne(false);
}
#endif

int main(void)
{
  double e[2], c[2];

  /* at rest: the correction settles at minus the bias, axis by axis, and the estimate follows */
  run(-0.5, 0.8, false, 1000.0, 20.0, e, c);
  printf("rest, bias (-0.50, +0.80) deg/s: rateCorr (%+.3f, %+.3f), estimate (%+.3f, %+.3f)\n", c[0], c[1], e[0], e[1]);
  CHECK(fabs(c[0] - 0.5) < 0.01 && fabs(c[1] + 0.8) < 0.01, "rateCorr settles at minus the bias: %+.3f %+.3f", c[0],
        c[1]);
  CHECK(fabs(e[0] + 0.5) < 0.03 && fabs(e[1] - 0.8) < 0.03, "estimate follows the bias: %+.3f %+.3f", e[0], e[1]);

  /* in flight (thrust-axis accelerometer, rocking +-7 deg), at the drone's 957 Hz loop */
  run(-0.5, 0.8, true, 957.0, 40.0, e, c);
  printf("flight, 957 Hz loop: estimate (%+.3f, %+.3f)\n", e[0], e[1]);
  CHECK(fabs(e[0] + 0.5) < 0.06 && fabs(e[1] - 0.8) < 0.06, "flight estimate %+.3f %+.3f", e[0], e[1]);

  /* no bias: no estimate */
  run(0.0, 0.0, true, 1000.0, 20.0, e, c);
  CHECK(fabs(e[0]) < 0.03 && fabs(e[1]) < 0.03, "no bias: %+.3f %+.3f", e[0], e[1]);

  /* an update without a usable accelerometer leaves no correction */
  sensfusion6UpdateQ(1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.004f);
  CHECK(sensfusion6RateCorr[0] == 0.0f && sensfusion6RateCorr[1] == 0.0f && sensfusion6RateCorr[2] == 0.0f,
        "zero accelerometer: no correction");

#ifdef SF6_OLD
  printf("identity hash (this filter): 0x%016llX\n", (unsigned long long)identityHash());
#else
  testKiAir();
  /* twoKiAir 0: the filter of 14bcfc6, bit for bit */
  sensfusion6SetKiAir(0.0f);
  const uint64_t h0 = identityHash();
  sensfusion6SetKiAir(0.040f);
  const uint64_t h1 = identityHash();
  sensfusion6SetAirborne(false);
  printf("identity hash: twoKiAir 0 0x%016llX (14bcfc6 0x%016llX), 0.040 0x%016llX (0ed68c9 0x%016llX)\n",
         (unsigned long long)h0, (unsigned long long)SF6_IDENTITY_HASH_14BCFC6, (unsigned long long)h1,
         (unsigned long long)SF6_KIAIR_HASH_0ED68C9);
  CHECK(h0 == SF6_IDENTITY_HASH_14BCFC6, "twoKiAir 0 is bit-identical to 14bcfc6");
  CHECK(h1 != h0, "twoKiAir 0.040 changes the filter (the sequence is airborne)");
  CHECK(h1 == SF6_KIAIR_HASH_0ED68C9, "twoKiAir 0.040 is bit-identical to 0ed68c9's learning");
#endif

  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
