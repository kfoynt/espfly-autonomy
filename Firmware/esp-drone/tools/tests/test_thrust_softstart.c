/*
 * Host unit tests for components/core/crazyflie/utils/src/thrust_softstart.c
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_thrust_softstart.c components/core/crazyflie/utils/src/thrust_softstart.c \
 *      -o /tmp/test_thrust_softstart && /tmp/test_thrust_softstart
 */
#include <stdio.h>
#include <stdlib.h>

#include "thrust_softstart.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static uint16_t run(thrustSoftStart_t *s, uint16_t req, bool resting, int loops)
{
  uint16_t out = 0;
  for (int i = 0; i < loops; i++) {
    out = thrustSoftStartApply(s, req, resting, false, true);
  }
  return out;
}

static void testSlamFromBoot(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  uint16_t prev = 0, out = 0;
  int loops, mono = 1;
  for (loops = 1; loops < 2000; loops++) {
    out = thrustSoftStartApply(&s, 52428, false, false, true);
    if (out < prev) mono = 0;
    prev = out;
    if (out == 52428) break;
  }
  CHECK(mono, "monotonic");
  CHECK(loops > 440 && loops < 470, "slam reaches 52428 in %d loops (~0.45 s)", loops);
  CHECK(!s.active, "ramp ended");
  CHECK(thrustSoftStartApply(&s, 60000, false, false, true) == 60000, "passes through after the ramp");
  thrustSoftStartInit(&s);
  CHECK(thrustSoftStartApply(&s, 52428, true, false, true) == THRUST_SOFT_STEP, "first loop is one step");
}

static void testSlowPushNotLimited(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  int limited = 0;
  for (int i = 0; i <= 1000; i++) {
    uint16_t req = (uint16_t)(i * 50);   /* 0 -> 50000 in 1 s, slower than the slope */
    if (thrustSoftStartApply(&s, req, false, false, true) != req) limited++;
  }
  CHECK(limited == 0, "a push slower than the slope is never limited (%d)", limited);
}

static void testZeroBlipDuringRampDecays(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  uint16_t before = run(&s, 50000, false, 100);          /* ~11600 */
  run(&s, 0, true, 50);                                  /* thumb lifted 50 ms, at rest */
  uint16_t after = thrustSoftStartApply(&s, 50000, false, false, true);
  CHECK(after < before, "zero blip lowers the ramp (%u -> %u)", before, after);
  CHECK(after > 0, "but does not reset it fully");
}

static void testInFlightReleaseNeverArms(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  run(&s, 50000, false, 1000);                            /* flying */
  run(&s, 0, false, 3000);                                /* 3 s stick release, falling: not at rest */
  CHECK(thrustSoftStartApply(&s, 50000, false, false, true) == 50000, "a falling drone gets full thrust back");
  run(&s, 0, true, THRUST_SOFT_ARM_LOOPS - 1);            /* almost long enough at rest */
  CHECK(thrustSoftStartApply(&s, 50000, false, false, true) == 50000, "rest shorter than the arm time does not arm");
}

static void testGroundStopArms(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  run(&s, 50000, false, 1000);
  run(&s, 0, true, THRUST_SOFT_ARM_LOOPS);                /* landed and resting */
  CHECK(thrustSoftStartApply(&s, 50000, false, false, true) == THRUST_SOFT_STEP, "next take-off is ramped");
}

static void testDipAboveEndDoesNotRestart(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  run(&s, 52428, false, 300);                             /* ramp at ~34800, still below the request */
  uint16_t level = s.level;
  CHECK(level > THRUST_SOFT_END && s.active, "mid-ramp above END (%u)", level);
  CHECK(run(&s, 15000, false, 80) == 15000, "dip is honoured");
  uint16_t back = thrustSoftStartApply(&s, 52428, false, false, true);
  CHECK(back + 80 * THRUST_SOFT_STEP >= level, "short dip lowers the level by at most a step per loop (%u vs %u)",
        back, level);
}

static void testLowHoldThenPush(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  run(&s, 3000, false, 5000);                             /* barely spinning for 5 s */
  CHECK(s.active, "low hold does not end the ramp");
  CHECK(thrustSoftStartApply(&s, 50000, false, false, true) <= 3000 + THRUST_SOFT_STEP, "push from a low hold is ramped");
}

static void testNeverRaisesAndDisabled(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  int ok = 1;
  srand(7);
  for (int i = 0; i < 20000; i++) {
    uint16_t req = (uint16_t)(rand() % 3 == 0 ? 0 : rand() % 65536);
    uint16_t out = thrustSoftStartApply(&s, req, rand() % 2, rand() % 7 == 0, true);
    if (out > req) ok = 0;
  }
  CHECK(ok, "output never exceeds the request");
  thrustSoftStartInit(&s);
  CHECK(thrustSoftStartApply(&s, 52428, true, false, false) == 52428, "disabled passes through");
}

static void testLongLowHoldAboveEndRamps(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  run(&s, 52428, false, 300);                             /* ramp mid-way above END, still on the ground */
  run(&s, 1000, false, 1000);                             /* stick almost down for 1 s: motors stopped */
  uint16_t out = thrustSoftStartApply(&s, 52428, false, false, true);
  CHECK(out <= 1000 + 2 * THRUST_SOFT_STEP, "push after a long low hold is ramped again (%u)", out);
}

static void testDisabledStillTracksGround(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  for (int i = 0; i < 1000; i++) {
    thrustSoftStartApply(&s, 50000, false, false, false);   /* flying with the ramp off */
  }
  for (int i = 0; i < 30; i++) {
    thrustSoftStartApply(&s, 0, false, false, false);       /* thumb slip in the air, not falling yet */
  }
  CHECK(!thrustSoftStartActive(&s), "ramp off: a zero blip in flight is not a ground start");
  for (int i = 0; i < THRUST_SOFT_ARM_LOOPS; i++) {
    thrustSoftStartApply(&s, 0, true, false, false);        /* landed and resting */
  }
  CHECK(thrustSoftStartActive(&s), "ramp off: a rest re-arms the ground start");
  CHECK(thrustSoftStartApply(&s, 52428, false, false, false) == 52428, "ramp off: still passes through");
}

static void testFreeFallEndsRamp(void)
{
  thrustSoftStart_t s;
  thrustSoftStartInit(&s);
  run(&s, 0, true, THRUST_SOFT_ARM_LOOPS);                /* held still in the hand, armed */
  for (int i = 0; i < THRUST_SOFT_FALL_LOOPS; i++) {
    thrustSoftStartApply(&s, 0, false, true, true);       /* let go: free fall */
  }
  CHECK(thrustSoftStartApply(&s, 50000, false, true, true) == 50000, "full thrust at once after free fall");
  thrustSoftStartInit(&s);
  for (int i = 0; i < THRUST_SOFT_FALL_LOOPS - 1; i++) {
    thrustSoftStartApply(&s, 0, false, true, true);       /* a bump shorter than the fall time */
  }
  CHECK(thrustSoftStartApply(&s, 50000, false, false, true) == THRUST_SOFT_STEP, "short bump does not end the ramp");
}

int main(void)
{
  testLongLowHoldAboveEndRamps();
  testFreeFallEndsRamp();
  testSlamFromBoot();
  testSlowPushNotLimited();
  testZeroBlipDuringRampDecays();
  testInFlightReleaseNeverArms();
  testGroundStopArms();
  testDipAboveEndDoesNotRestart();
  testLowHoldThenPush();
  testNeverRaisesAndDisabled();
  testDisabledStillTracksGround();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
