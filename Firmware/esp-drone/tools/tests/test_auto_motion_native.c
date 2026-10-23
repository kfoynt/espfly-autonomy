/* The motion controller under the native camera profile (ESPFLY_NATIVE_VGA, the build the drone runs since
 * 3cd4951-vga25): the VIEW_FAST timer derived from the profile's frame period (camera_cadence.h, 2026-10-22) and the
 * MOVEs of the 2026-10-22 table flight replayed on it. The legacy test_auto_motion.c is included for its rig (its own
 * main is renamed; its identity hashes and 300 ms view age are the legacy profile's and are not run here).
 *
 *   cc -std=c99 -O2 -Wall -Wextra -DESPFLY_NATIVE_VGA=1 -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_auto_motion_native.c components/core/crazyflie/utils/src/auto_motion.c -lm \
 *      -o /tmp/test_auto_motion_native && /tmp/test_auto_motion_native
 */
#ifndef ESPFLY_NATIVE_VGA
#error "build with -DESPFLY_NATIVE_VGA=1: the native camera profile"
#endif
#define main legacy_motion_tests_main
#include "test_auto_motion.c"
#undef main

/* A 300 % forward MOVE at D (no scene guard bites), its view lost (the hold coasting: holdApplied false) at lostMs
 * after the start for each of the gaps, in turn (gapsMs, relative to the move's start; the step clock is 1 ms):
 * the state after the last gap and the end. */
typedef struct { int atMs, lenMs; } gap_t;

static void replayMove(rig_t *r, int fwdCm, float d, const gap_t *gaps, int n, int *stoppedAt)
{
  rigSpeed(r, 300, d);
  CHECK(autoMotionMove(&r->m, &r->in, 0, (int16_t)fwdCm, 0) == AUTO_MVW_NONE, "replay: forward %d cm at D %.1f accepted",
        fwdCm, d);
  const uint32_t t0 = r->in.nowMs;
  *stoppedAt = -1;
  for (int k = 0; k < n; k++) {
    while ((int)(r->in.nowMs - t0) < gaps[k].atMs && r->land == AUTO_MVL_NONE) {
      step(r);
    }
    r->in.holdApplied = false;
    while ((int)(r->in.nowMs - t0) < gaps[k].atMs + gaps[k].lenMs && r->land == AUTO_MVL_NONE) {
      step(r);
    }
    r->in.holdApplied = true;
    if (r->land != AUTO_MVL_NONE && *stoppedAt < 0) {
      *stoppedAt = (int)(r->in.nowMs - t0) - gaps[k].atMs;   /* ms into this gap */
    }
    step(r);
  }
}

static void testNativeTimer(void)
{
  CHECK(AUTO_MV_FAST_VIEW_DEB_MS == 323 && AUTO_MV_FRONT_VIEW_DEB_MS == 150 && AUTO_MV_VIEW_MAX_AGE_MS == 450 &&
            CAM_CADENCE_VGA_PERIOD_MS == 91 && CAM_CADENCE_VGA_FAST_VIEW_RAW_MS == 323 &&
            CAM_CADENCE_VGA_FAST_VIEW_MS >= CAM_CADENCE_VGA31_FAST_VIEW_MS,
        "native QVGA: VIEW_FAST 3 x 91 + 50 = 323 ms (never below VGA31's 231), FRONT 150, the view age 450");
  CHECK(autoMotionGetViewDeb() == 323, "mvview 0: the timer in force 323 ms (%u)", autoMotionGetViewDeb());
  /* a 300 % approach's view lost in the cruise for 150, 163, 200, 250, 275, 300 and 322 ms: PAUSED, then RUN again
   * (163 the longest frame age of the table flight, 275 its first fatal coast; 231 was the timer before) */
  const int gaps[] = {150, 163, 200, 231, 250, 275, 300, 322};
  for (unsigned k = 0; k < sizeof gaps / sizeof gaps[0]; k++) {
    rig_t r;
    rigSpeed(&r, 300, 5.5f);
    autoMotionMove(&r.m, &r.in, 0, 200, 0);
    steps(&r, 2500);
    r.in.holdApplied = false;
    steps(&r, gaps[k]);
    const bool paused = r.m.state == AUTO_MV_PAUSED && r.land == AUTO_MVL_NONE;
    r.in.holdApplied = true;
    step(&r);
    CHECK(paused && r.m.state == AUTO_MV_RUN && r.land == AUTO_MVL_NONE,
          "300: the view lost %d ms in the cruise: PAUSED, then RUN again (no VIEW_FAST)", gaps[k]);
    untilEnd(&r, 30000);
    CHECK(r.m.state == AUTO_MV_DONE, "... and the move ends DONE (%s)", autoMvEndName(r.m.end));
  }
  /* lost for good: stopped VIEW_FAST at 324 ms with its brake */
  rig_t r;
  rigSpeed(&r, 300, 5.5f);
  autoMotionMove(&r.m, &r.in, 0, 200, 0);
  steps(&r, 2500);
  float s, v, a;
  autoMvProfEval(&r.m.ph, r.m.tRun, &s, &v, &a);
  r.in.holdApplied = false;
  int deb = 0;
  while (r.land == AUTO_MVL_NONE && deb < 2000) {
    step(&r);
    deb++;
  }
  CHECK(r.land == AUTO_MVL_FAST && deb == 324 && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_VIEW_FAST &&
            v > AUTO_MV_FAST_PAUSE_V && fabsf(r.m.brakeS - v / 0.40f) < 0.01f,
        "300: lost for good: STOPPED VIEW_FAST after %d ms (323 + 1), brake %.2f s from %.2f m/s", deb, r.m.brakeS, v);
}

/* The 2026-10-22 table flight (firmware-backups/cal/flight-2026-10-22-table.csv, flight 1; 300 %, every scene guard
 * off): the hold coasted (holdFlags bit 4) during the forward MOVEs at these times after each RUN began, for this long
 * (the flow's own frame refusal, then the HOVER scene gate's residual refusals in a row), and the 231 ms timer of the
 * installed build stopped both moves VIEW_FAST:
 *   move 1 (2.0 m): coasts at 4369 ms for 90 ms (one refused frame), at 4550 ms for 275 ms (three): stopped at 232 ms
 *   move 2 (0.85 m): a coast at 1801 ms for 544 ms (six refused frames): stopped at 231 ms
 * On this build (323 ms) move 1 runs on through both coasts; move 2 is still stopped, 92 ms later than before, and runs
 * on with mvview 600 (the user's knob). The rig has no scene: the profile times are the rig's, the coasts the log's. */
static void testTableFlightReplay(void)
{
  int stoppedAt;
  rig_t r;
  const gap_t m1[] = {{4369, 90}, {4550, 275}};
  replayMove(&r, 200, 5.5f, m1, 2, &stoppedAt);
  CHECK(stoppedAt < 0 && r.m.state == AUTO_MV_RUN && r.land == AUTO_MVL_NONE,
        "move 1 replayed at 323 ms: the 90 and 275 ms coasts pause the move, no VIEW_FAST (state %s)",
        autoMvStateName(r.m.state));
  untilEnd(&r, 30000);
  CHECK(r.m.state == AUTO_MV_DONE, "... the 2.0 m move ends DONE (%s)", autoMvEndName(r.m.end));
  const gap_t m2[] = {{1801, 544}};
  replayMove(&r, 85, 5.5f, m2, 1, &stoppedAt);
  CHECK(stoppedAt == 324 && r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_VIEW_FAST,
        "move 2 replayed at 323 ms: the 544 ms coast still stops it VIEW_FAST, at %d ms into the coast (231 before)",
        stoppedAt);
  /* the same coasts at the old 231 ms timer (mvview cannot set it: below the profile's; the macro's value is checked
   * through the setting's floor) and at mvview 600 */
  autoMotionSetViewDeb(231);
  CHECK(autoMotionGetViewDeb() == 323, "mvview 231: below the profile's, the timer stays 323 (never shorter)");
  autoMotionSetViewDeb(600);
  CHECK(autoMotionGetViewDeb() == 600, "mvview 600 in force");
  replayMove(&r, 85, 5.5f, m2, 1, &stoppedAt);
  CHECK(stoppedAt < 0 && r.m.state == AUTO_MV_RUN && r.land == AUTO_MVL_NONE,
        "move 2 replayed at mvview 600: the 544 ms coast pauses the move, no VIEW_FAST");
  untilEnd(&r, 30000);
  CHECK(r.m.state == AUTO_MV_DONE, "... the 0.85 m move ends DONE (%s)", autoMvEndName(r.m.end));
  replayMove(&r, 200, 5.5f, m1, 2, &stoppedAt);
  CHECK(stoppedAt < 0, "move 1 replayed at mvview 600: runs on");
  untilEnd(&r, 30000);
  CHECK(r.m.state == AUTO_MV_DONE, "... DONE");
  /* lost for good at mvview 600: stopped at 601 ms */
  rigSpeed(&r, 300, 5.5f);
  autoMotionMove(&r.m, &r.in, 0, 200, 0);
  steps(&r, 2500);
  r.in.holdApplied = false;
  int deb = 0;
  while (r.land == AUTO_MVL_NONE && deb < 2000) {
    step(&r);
    deb++;
  }
  CHECK(r.land == AUTO_MVL_FAST && deb == 601 && r.m.end == AUTO_MVE_VIEW_FAST,
        "mvview 600, lost for good: STOPPED VIEW_FAST after %d ms", deb);
  autoMotionSetViewDeb(0);
  CHECK(autoMotionGetViewDeb() == 323, "mvview 0: 323 again");
}

int main(void)
{
  autoMotionSetMargins(NULL);
  testNativeTimer();
  testTableFlightReplay();
  testViewDeb();        /* the setting's clamps and a 400 ms run, on the native profile */
  testViewFast();       /* sustained loss, the brake, tilt priority and the yaw exclusions at 323 ms */
  testGuardMargins();   /* the near-scene FRONT brake stays 150 ms */
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
