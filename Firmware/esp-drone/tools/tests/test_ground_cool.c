/*
 * Host test: the ground prop-wash cooling spin (components/core/crazyflie/utils/src/ground_cool.c,
 * docs/autonomy-a2b.md 36.3).
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface tools/tests/test_ground_cool.c \
 *      components/core/crazyflie/utils/src/ground_cool.c -lm -o /tmp/test_ground_cool && /tmp/test_ground_cool
 *
 * The start guards one by one (range, running, autonomy, thrust / duty, ground / resting / tilt, the motor lock,
 * the battery), a full run (the PWM for exactly the asked seconds, then 0, DONE, the temperatures and maxima
 * recorded), and every abort path: a stop, a kill (request and counter), autonomy leaving IDLE, the motor driver
 * locking, tilt, gyro, the accelerometer norm (not during the spin-up), a NaN tilt; the clock wrapping.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ground_cool.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static groundCoolIn_t resting(void)
{
  groundCoolIn_t in;
  memset(&in, 0, sizeof(in));
  in.motorsReady = true;
  in.dutyZero = true;
  in.thrustZero = true;
  in.onGround = true;
  in.vbat = 4.05f;
  in.vminV = 3.80f;
  in.tiltDeg = 1.2f;
  in.gyroDps[0] = 0.5f;
  in.gyroDps[1] = -0.3f;
  in.gyroDps[2] = 0.1f;
  in.accNormG = 1.002f;
  in.imuTempC = 49.5f;
  return in;
}

/* Run the loop at 1 kHz from t0 until the spin ends or maxMs pass; returns the ms the thrust was nonzero. */
static uint32_t run(groundCool_t *gc, groundCoolIn_t *in, uint32_t t0, uint32_t maxMs, void (*tweak)(groundCoolIn_t *, uint32_t))
{
  uint32_t on = 0;
  for (uint32_t i = 0; i < maxMs; i++) {
    if (tweak) {
      tweak(in, i);
    }
    const float thr = groundCoolStep(gc, in, t0 + i);
    if (thr > 0.0f) {
      on++;
    }
    if (!gc->running) {
      break;
    }
  }
  return on;
}

static void coolDown(groundCoolIn_t *in, uint32_t ms)
{
  in->imuTempC = 49.5f - 0.3f * ms / 1000.0f;   /* 0.3 C/s under the wash */
}

static void guards(void)
{
  groundCool_t gc;
  groundCoolInit(&gc);
  groundCoolIn_t in = resting();
  CHECK(groundCoolStart(&gc, &in, 999, 20, 1000) == GC_E_RANGE, "pwm below the minimum");
  CHECK(groundCoolStart(&gc, &in, 20001, 20, 1000) == GC_E_RANGE, "pwm above the maximum");
  CHECK(groundCoolStart(&gc, &in, 12000, 0, 1000) == GC_E_RANGE, "0 s");
  CHECK(groundCoolStart(&gc, &in, 12000, 41, 1000) == GC_E_RANGE, "41 s");
  in.autonomyActive = true;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_AUTO, "autonomy active");
  in = resting();
  in.dutyZero = false;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_THRUST, "a motor duty nonzero");
  in = resting();
  in.thrustZero = false;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_THRUST, "thrust requested");
  in = resting();
  in.onGround = false;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_GROUND, "not on the ground");
  in = resting();
  in.gyroDps[1] = 12.0f;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_GROUND, "not resting (gyro)");
  in = resting();
  in.accNormG = 0.80f;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_GROUND, "not resting (acc)");
  in = resting();
  in.tiltDeg = 9.0f;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_GROUND, "tilted");
  in = resting();
  in.motorsReady = false;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_LOCKED, "motor driver locked");
  in = resting();
  in.vbat = 3.79f;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_BATT, "battery below vmin");
  in.vbat = 3.80f;
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_OK, "battery at vmin starts");
  CHECK(gc.running && gc.runs == 1 && gc.pwm == 12000 && gc.seconds == 20 && gc.tStartC == 49.5f, "started");
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 1000) == GC_E_RUNNING, "already running");
  CHECK(!gc.running == false && groundCoolStep(&gc, &in, 1000) == 12000.0f, "the first step asks the PWM");
  CHECK(strcmp(groundCoolRefusalName(GC_E_BATT), "battery below vmin") == 0 && strcmp(groundCoolEndName(GC_R_DONE), "DONE") == 0,
        "names");
}

static void fullRun(void)
{
  groundCool_t gc;
  groundCoolInit(&gc);
  groundCoolIn_t in = resting();
  CHECK(groundCoolStart(&gc, &in, GROUND_COOL_PWM_DEFAULT, 20, 5000) == GC_OK, "start");
  in.gyroDps[0] = 4.0f;   /* the spinning drone vibrates a little */
  in.tiltDeg = 2.5f;
  const uint32_t on = run(&gc, &in, 5000, 60000, coolDown);
  CHECK(!gc.running && gc.end == GC_R_DONE, "ended DONE (%s)", groundCoolEndName((groundCoolEnd_t)gc.end));
  CHECK(on == 20000, "the PWM for exactly 20 s (%u ms)", (unsigned)on);
  CHECK(gc.elapsedMs == 20000, "elapsed %u", (unsigned)gc.elapsedMs);
  CHECK(fabsf(gc.tEndC - (49.5f - 6.0f)) < 0.01f && gc.tStartC == 49.5f, "temperatures %.2f -> %.2f", gc.tStartC, gc.tEndC);
  CHECK(gc.tiltMaxDeg == 2.5f && gc.gyroMaxDps == 4.0f, "maxima %.1f %.1f", gc.tiltMaxDeg, gc.gyroMaxDps);
  CHECK(groundCoolStep(&gc, &in, 30000) == 0.0f, "0 once ended");
  /* a second run, the clock about to wrap */
  in = resting();
  CHECK(groundCoolStart(&gc, &in, 8000, 3, 0xFFFFFC00u) == GC_OK, "start before the wrap");
  const uint32_t on2 = run(&gc, &in, 0xFFFFFC00u, 10000, NULL);
  CHECK(!gc.running && gc.end == GC_R_DONE && on2 == 3000 && gc.elapsedMs == 3000 && gc.runs == 2,
        "3 s across the wrap (%u, %s)", (unsigned)on2, groundCoolEndName((groundCoolEnd_t)gc.end));
}

static void tiltAt(groundCoolIn_t *in, uint32_t ms) { if (ms == 4000) in->tiltDeg = 8.5f; }
static void gyroAt(groundCoolIn_t *in, uint32_t ms) { if (ms == 2500) in->gyroDps[2] = -31.0f; }
static void accEarly(groundCoolIn_t *in, uint32_t ms) { in->accNormG = ms < 200 ? 0.7f : 1.0f; }
static void accLate(groundCoolIn_t *in, uint32_t ms) { if (ms == 6000) in->accNormG = 1.2f; }
static void autoAt(groundCoolIn_t *in, uint32_t ms) { if (ms == 1000) in->autonomyActive = true; }
static void lockAt(groundCoolIn_t *in, uint32_t ms) { if (ms == 1500) in->motorsReady = false; }
static void killAt(groundCoolIn_t *in, uint32_t ms) { if (ms == 700) in->killCount++; }
static void nanAt(groundCoolIn_t *in, uint32_t ms) { if (ms == 300) in->tiltDeg = NAN; }

static void abortCase(const char *name, void (*tweak)(groundCoolIn_t *, uint32_t), groundCoolEnd_t want, uint32_t wantMs)
{
  groundCool_t gc;
  groundCoolInit(&gc);
  groundCoolIn_t in = resting();
  CHECK(groundCoolStart(&gc, &in, 12000, 20, 100) == GC_OK, "%s: start", name);
  const uint32_t on = run(&gc, &in, 100, 30000, tweak);
  CHECK(!gc.running && gc.end == want, "%s: ended %s (want %s)", name, groundCoolEndName((groundCoolEnd_t)gc.end),
        groundCoolEndName(want));
  CHECK(on == wantMs, "%s: the PWM for %u ms (want %u)", name, (unsigned)on, (unsigned)wantMs);
  CHECK(groundCoolStep(&gc, &in, 50000) == 0.0f, "%s: 0 after", name);
}

static void aborts(void)
{
  abortCase("tilt", tiltAt, GC_R_TILT, 4000);
  abortCase("gyro", gyroAt, GC_R_GYRO, 2500);
  abortCase("acc late", accLate, GC_R_ACC, 6000);
  abortCase("autonomy", autoAt, GC_R_AUTO, 1000);
  abortCase("lock", lockAt, GC_R_LOCKED, 1500);
  abortCase("kill counter", killAt, GC_R_KILL, 700);
  abortCase("nan tilt", nanAt, GC_R_TILT, 300);
  /* the accelerometer norm during the spin-up is not an abort */
  {
    groundCool_t gc;
    groundCoolInit(&gc);
    groundCoolIn_t in = resting();
    CHECK(groundCoolStart(&gc, &in, 12000, 2, 100) == GC_OK, "spin-up: start");
    const uint32_t on = run(&gc, &in, 100, 5000, accEarly);
    CHECK(gc.end == GC_R_DONE && on == 2000, "spin-up: the norm dip in the first 200 ms is ignored (%s, %u)",
          groundCoolEndName((groundCoolEnd_t)gc.end), (unsigned)on);
  }
  /* stop and kill requests */
  {
    groundCool_t gc;
    groundCoolInit(&gc);
    groundCoolIn_t in = resting();
    CHECK(groundCoolStart(&gc, &in, 12000, 20, 100) == GC_OK, "stop: start");
    for (uint32_t i = 0; i < 1200; i++) {
      groundCoolStep(&gc, &in, 100 + i);
    }
    groundCoolStop(&gc);
    CHECK(gc.running, "stop: served at the next step, not before");
    CHECK(groundCoolStep(&gc, &in, 1300) == 0.0f && !gc.running && gc.end == GC_R_STOP && gc.elapsedMs == 1200,
          "stop: %s after %u ms", groundCoolEndName((groundCoolEnd_t)gc.end), (unsigned)gc.elapsedMs);
    groundCoolInit(&gc);
    CHECK(groundCoolStart(&gc, &in, 12000, 20, 100) == GC_OK, "kill: start");
    groundCoolStep(&gc, &in, 100);
    groundCoolKill(&gc);
    CHECK(groundCoolStep(&gc, &in, 101) == 0.0f && gc.end == GC_R_KILL, "kill request: %s",
          groundCoolEndName((groundCoolEnd_t)gc.end));
    /* a kill request before a start does not stop the new run (the start clears it) */
    groundCoolKill(&gc);
    CHECK(groundCoolStart(&gc, &in, 12000, 1, 200) == GC_OK, "restart after a kill");
    CHECK(groundCoolStep(&gc, &in, 200) == 12000.0f, "the stale request is cleared by the start");
  }
}

int main(void)
{
  guards();
  fullRun();
  aborts();
  printf("test_ground_cool: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
