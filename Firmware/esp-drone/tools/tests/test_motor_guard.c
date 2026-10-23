/*
 * Host unit tests for components/core/crazyflie/utils/src/motor_guard_core.c
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/modules/interface \
 *      tools/tests/test_motor_guard.c components/core/crazyflie/utils/src/motor_guard_core.c \
 *      -o /tmp/test_motor_guard && /tmp/test_motor_guard
 */
#include <stdio.h>
#include <stdlib.h>

#include "motor_guard.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define T0_US 1000000ULL   /* boot plus 1 s: esp_timer time is never 0 */
#define P     MOTOR_GUARD_PERIOD_US

/* A 1 kHz stabilizer (alive += 1 per ms while running) and the guard every
 * gapUs; the stabilizer stops at freezeUs. Returns the time of MG_TRIP, or
 * 0 if it never tripped before endUs. */
typedef struct {
  motorGuardCore_t c;
  uint64_t now;
  uint32_t alive;
  int trips;
  int tripped;      /* MG_TRIPPED verdicts after the trip */
  int late;
  int stalling;
} sim_t;

static void simInit(sim_t *s)
{
  motorGuardCoreInit(&s->c);
  s->now = T0_US;
  s->alive = 0;
  s->trips = s->tripped = s->late = s->stalling = 0;
}

static motorGuardVerdict_t simCall(sim_t *s, bool motorsOn, bool enabled)
{
  motorGuardVerdict_t v = motorGuardCoreStep(&s->c, s->now, s->alive, motorsOn, enabled);
  if (v == MG_TRIP) s->trips++;
  if (v == MG_TRIPPED) s->tripped++;
  if (v == MG_LATE) s->late++;
  if (v == MG_STALLING) s->stalling++;
  return v;
}

/* Advance by dtUs; the stabilizer runs (one alive tick per ms boundary) if running. */
static void simAdvance(sim_t *s, uint64_t dtUs, bool running)
{
  const uint64_t end = s->now + dtUs;
  if (running) {
    s->alive += (uint32_t)(end / 1000 - s->now / 1000);
  }
  s->now = end;
}

/* Run with the guard every gapUs until endUs; the stabilizer runs until
 * freezeUs (absolute). Returns the trip time or 0. */
static uint64_t simRun(sim_t *s, uint64_t gapUs, uint64_t freezeUs, uint64_t endUs, bool motorsOn, bool enabled)
{
  uint64_t tripAt = 0;
  while (s->now < endUs) {
    simAdvance(s, gapUs, s->now < freezeUs);
    if (simCall(s, motorsOn, enabled) == MG_TRIP) {
      tripAt = s->now;
    }
  }
  return tripAt;
}

static void testFirstCallIsLateNotCounted(void)
{
  sim_t s;
  simInit(&s);
  CHECK(simCall(&s, true, true) == MG_LATE, "first call restarts the window");
  CHECK(s.c.lateCalls == 0, "the first call is not counted late (%u)", (unsigned)s.c.lateCalls);
  CHECK(s.c.maxCallGapUs == 0, "no gap yet (%u)", (unsigned)s.c.maxCallGapUs);
}

static void testSteadyProgressNeverTrips(void)
{
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  int ok = 0;
  for (int i = 0; i < 20000; i++) {          /* 100 s */
    simAdvance(&s, P, true);
    if (simCall(&s, true, true) == MG_OK) ok++;
  }
  CHECK(s.trips == 0 && !s.c.tripped, "steady progress never trips");
  CHECK(ok == 20000, "every call OK (%d)", ok);
  CHECK(s.c.maxStallUs == 0, "no stall seen (%u)", (unsigned)s.c.maxStallUs);
  CHECK(s.c.maxCallGapUs == P, "max gap = period (%u)", (unsigned)s.c.maxCallGapUs);
  CHECK(s.c.lateCalls == 0, "no late calls");
}

static void testFrozenMotorsOnTripsOnce(void)
{
  /* Freeze at every phase of the 1 ms stabilizer and 5 ms guard grids. */
  int good = 0, cases = 0;
  uint64_t minLat = UINT64_MAX, maxLat = 0;
  for (uint64_t phase = 0; phase < P; phase += 250) {
    sim_t s;
    simInit(&s);
    simCall(&s, true, true);
    simRun(&s, P, UINT64_MAX, T0_US + 1000000, true, true);        /* 1 s flying */
    const uint64_t freezeAt = s.now + phase;
    const uint64_t tripAt = simRun(&s, P, freezeAt, freezeAt + 1000000, true, true);
    const uint64_t lastTickUs = (freezeAt / 1000) * 1000;            /* last alive tick */
    const uint64_t lat = tripAt - lastTickUs;
    cases++;
    if (tripAt != 0 && s.trips == 1 && lat >= MOTOR_GUARD_FREEZE_US && lat <= MOTOR_GUARD_FREEZE_US + P) good++;
    if (lat < minLat) minLat = lat;
    if (lat > maxLat) maxLat = lat;
    CHECK(s.tripped > 100, "MG_TRIPPED after the trip (%d)", s.tripped);
  }
  CHECK(good == cases, "trips once, 80..85 ms after the last tick, at every phase (%d/%d)", good, cases);
  printf("  freeze -> trip: %.1f..%.1f ms after the last stabilizer tick\n", minLat / 1000.0, maxLat / 1000.0);
}

static void testTripTimingExact(void)
{
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  simAdvance(&s, P, true);
  CHECK(simCall(&s, true, true) == MG_OK, "progress");
  const uint64_t windowStart = s.now;
  for (int i = 1; i < MOTOR_GUARD_FREEZE_US / P; i++) {
    simAdvance(&s, P, false);
    CHECK(simCall(&s, true, true) == MG_STALLING, "stalling at %d ms", i * P / 1000);
  }
  CHECK(s.c.stallUs == MOTOR_GUARD_FREEZE_US - P, "stallUs one period short (%u)", (unsigned)s.c.stallUs);
  simAdvance(&s, P, false);
  CHECK(simCall(&s, true, true) == MG_TRIP, "trips when the stall reaches the threshold");
  CHECK(s.now - windowStart == MOTOR_GUARD_FREEZE_US, "exactly at 80 ms of observation");
  CHECK(s.c.maxStallUs == MOTOR_GUARD_FREEZE_US, "maxStallUs = threshold (%u)", (unsigned)s.c.maxStallUs);
  /* Latched: nothing clears it. */
  simAdvance(&s, P, true);
  CHECK(simCall(&s, true, true) == MG_TRIPPED, "progress does not clear it");
  simAdvance(&s, P, true);
  CHECK(simCall(&s, false, true) == MG_TRIPPED, "motors off does not clear it");
  simAdvance(&s, P, true);
  CHECK(simCall(&s, true, false) == MG_TRIPPED, "disabling does not clear it");
  simAdvance(&s, 100000, true);
  CHECK(simCall(&s, true, true) == MG_TRIPPED, "a late call does not clear it");
  CHECK(s.trips == 1, "MG_TRIP returned once (%d)", s.trips);
}

static void testFrozenMotorsOffNeverTrips(void)
{
  sim_t s;
  simInit(&s);
  simCall(&s, false, true);
  simRun(&s, P, s.now + 100000, s.now + 10000000, false, true);     /* frozen 10 s, motors off */
  CHECK(s.trips == 0 && s.stalling == 0, "motors off never trips or stalls");
  CHECK(s.c.maxStallUs == 0, "no stall counted (%u)", (unsigned)s.c.maxStallUs);
}

static void testMotorsOffBrieflyRestarts(void)
{
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  for (int i = 0; i < 15; i++) {             /* 75 ms frozen, motors on */
    simAdvance(&s, P, false);
    simCall(&s, true, true);
  }
  simAdvance(&s, P, false);
  CHECK(simCall(&s, false, true) == MG_OK, "one callback with the motors off restarts");
  for (int i = 0; i < 15; i++) {
    simAdvance(&s, P, false);
    simCall(&s, true, true);
  }
  CHECK(s.trips == 0, "two 75 ms stalls split by a motors-off reading do not trip");
  CHECK(s.c.maxStallUs == 75000, "maxStallUs 75 ms (%u)", (unsigned)s.c.maxStallUs);
}

static void testNearMissThenProgress(void)
{
  /* A legitimate stabilizer-only stall: a console print spinning on a full
   * USB-Serial-JTAG FIFO (<= 50 ms) plus a failed I2C read (<= ~11 ms). */
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  simRun(&s, P, UINT64_MAX, s.now + 500000, true, true);
  const uint64_t f = s.now + 1300;
  simRun(&s, P, f, f + 61000, true, true);
  simRun(&s, P, UINT64_MAX, s.now + 500000, true, true);
  CHECK(s.trips == 0, "a 61 ms stall does not trip");
  CHECK(s.c.maxStallUs >= 55000 && s.c.maxStallUs <= 65000, "maxStallUs ~61 ms (%u)", (unsigned)s.c.maxStallUs);
}

static void testSystemStallNotCountedButKeepsObserved(void)
{
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  simAdvance(&s, P, true);
  simCall(&s, true, true);
  for (int i = 0; i < 8; i++) {              /* 40 ms frozen */
    simAdvance(&s, P, false);
    simCall(&s, true, true);
  }
  CHECK(s.c.stallUs == 40000, "40 ms counted (%u)", (unsigned)s.c.stallUs);
  simAdvance(&s, 60000, false);              /* flash erase: the guard stops too */
  CHECK(simCall(&s, true, true) == MG_LATE, "a 60 ms callback gap is late");
  CHECK(s.c.stallUs == 40000, "the late gap adds nothing, the observed 40 ms stay (%u)", (unsigned)s.c.stallUs);
  CHECK(s.c.lateCalls == 1, "lateCalls 1 (%u)", (unsigned)s.c.lateCalls);
  CHECK(s.c.maxCallGapUs == 60000, "maxCallGapUs 60 ms (%u)", (unsigned)s.c.maxCallGapUs);
  const uint64_t late = s.now;
  uint64_t tripAt = simRun(&s, P, 0, s.now + 200000, true, true);   /* still frozen */
  CHECK(tripAt == late + MOTOR_GUARD_FREEZE_US - 40000, "trips after 40 ms more of observation (%lld ms)",
        (long long)(tripAt - late) / 1000);
  CHECK(s.c.maxStallUs == MOTOR_GUARD_FREEZE_US, "maxStallUs (%u)", (unsigned)s.c.maxStallUs);
}

static void testLateCallWithProgressRestarts(void)
{
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  for (int i = 0; i < 8; i++) {              /* 40 ms frozen */
    simAdvance(&s, P, false);
    simCall(&s, true, true);
  }
  simAdvance(&s, 30000, true);               /* late, but the stabilizer ran meanwhile */
  CHECK(simCall(&s, true, true) == MG_LATE, "late call with progress is MG_LATE");
  CHECK(s.c.stallUs == 0 && s.c.lateCalls == 1, "progress restarts the window (%u)", (unsigned)s.c.stallUs);
  for (int i = 0; i < 8; i++) {              /* 40 ms frozen again */
    simAdvance(&s, P, false);
    simCall(&s, true, true);
  }
  simAdvance(&s, 30000, false);              /* late, motors read off */
  CHECK(simCall(&s, false, true) == MG_LATE, "late call with motors off is MG_LATE");
  CHECK(s.c.stallUs == 0, "motors off restarts the window (%u)", (unsigned)s.c.stallUs);
  simRun(&s, P, 0, s.now + 75000, true, true);
  CHECK(s.trips == 0, "no trip: never 80 ms observed in one window");
}

static void testOftenLateGuardStillTrips(void)
{
  /* A real freeze while the guard is held up > 15 ms every 40 ms (cam_task
   * at priority 23 printing, Wi-Fi bursts): under a restart-on-late rule it
   * would never trip. Only on-time gaps count, so it trips after 80 ms of
   * them: 4 cycles of 4 x 5 ms on time + one 20 ms late gap. */
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  simRun(&s, P, UINT64_MAX, s.now + 500000, true, true);
  const uint64_t freezeAt = s.now;
  uint64_t tripAt = 0;
  for (int cycle = 0; cycle < 50 && !tripAt; cycle++) {
    for (int k = 0; k < 4 && !tripAt; k++) {
      simAdvance(&s, P, false);
      if (simCall(&s, true, true) == MG_TRIP) tripAt = s.now;
    }
    if (!tripAt) {
      simAdvance(&s, 20000, false);
      if (simCall(&s, true, true) == MG_TRIP) tripAt = s.now;
    }
  }
  CHECK(tripAt != 0 && s.trips == 1, "a frozen loop trips though the guard is late every 40 ms");
  CHECK(s.c.maxStallUs == MOTOR_GUARD_FREEZE_US, "exactly 80 ms of on-time observation (%u)",
        (unsigned)s.c.maxStallUs);
  CHECK(tripAt - freezeAt <= 160000, "within 160 ms of wall time (%lld ms)", (long long)(tripAt - freezeAt) / 1000);
  CHECK(s.c.lateCalls >= 3, "late gaps seen (%u)", (unsigned)s.c.lateCalls);
}

static void testLegitStallAcrossFlashEraseNoTrip(void)
{
  /* The worst legitimate stabilizer stall (61 ms: a 50 ms console spin plus
   * an 11 ms failed IMU read) with a 45 ms flash erase inside it, and the
   * stabilizer back only after one more guard period: 65 ms counted. */
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  simRun(&s, P, UINT64_MAX, s.now + 500000, true, true);
  for (int i = 0; i < 6; i++) {              /* 30 ms observed */
    simAdvance(&s, P, false);
    simCall(&s, true, true);
  }
  simAdvance(&s, 45000, false);              /* erase: guard late */
  CHECK(simCall(&s, true, true) == MG_LATE, "erase gap late");
  for (int i = 0; i < 7; i++) {              /* 31 ms more observed, then 5 ms with no tick yet */
    simAdvance(&s, P, false);
    simCall(&s, true, true);
  }
  CHECK(s.c.stallUs == 65000, "65 ms counted (%u)", (unsigned)s.c.stallUs);
  simRun(&s, P, UINT64_MAX, s.now + 500000, true, true);
  CHECK(s.trips == 0, "no trip");
  CHECK(s.c.maxStallUs == 65000, "maxStallUs 65 ms (%u)", (unsigned)s.c.maxStallUs);
}

static void testFlashEraseNoTrip(void)
{
  /* A 45 ms sector erase stops both: guard late, stabilizer back within 2 ms. */
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  simRun(&s, P, UINT64_MAX, s.now + 200000, true, true);
  for (int k = 0; k < 50; k++) {
    simAdvance(&s, 2000, true);
    simAdvance(&s, 45000, false);
    simCall(&s, true, true);                 /* late */
    simAdvance(&s, 2000, false);             /* stabilizer still waiting for the IMU */
    simCall(&s, true, true);
    simRun(&s, P, UINT64_MAX, s.now + 20000, true, true);
  }
  CHECK(s.trips == 0, "50 flash erases with the motors on do not trip");
  CHECK(s.c.lateCalls == 50, "each erase counted late (%u)", (unsigned)s.c.lateCalls);
  CHECK(s.c.maxStallUs <= 3000, "only the post-erase wait is counted (%u)", (unsigned)s.c.maxStallUs);
}

static void testRepeatedLateGapsNeverTrip(void)
{
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  simRun(&s, 20000, s.now, s.now + 10000000, true, true);           /* frozen 10 s, guard every 20 ms */
  CHECK(s.trips == 0, "repeated 20 ms gaps never trip");
  CHECK(s.late == 501, "every call late, the first included (%d)", s.late);
  CHECK(s.c.lateCalls == 500, "lateCalls 500 (%u)", (unsigned)s.c.lateCalls);
  CHECK(s.c.maxCallGapUs == 20000, "maxCallGapUs 20 ms (%u)", (unsigned)s.c.maxCallGapUs);
  CHECK(s.c.maxStallUs == 0, "nothing counted (%u)", (unsigned)s.c.maxStallUs);
}

static void testLateThresholdEdge(void)
{
  /* 15 ms gaps (= MOTOR_GUARD_LATE_US) are on time and count; 15.001 ms are late. */
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  uint64_t tripAt = simRun(&s, MOTOR_GUARD_LATE_US, s.now, s.now + 1000000, true, true);
  CHECK(tripAt != 0 && s.c.lateCalls == 0, "15 ms gaps still count and trip");
  CHECK(s.c.maxStallUs >= MOTOR_GUARD_FREEZE_US && s.c.maxStallUs < MOTOR_GUARD_FREEZE_US + MOTOR_GUARD_LATE_US,
        "trip within one gap of the threshold (%u)", (unsigned)s.c.maxStallUs);
  simInit(&s);
  simCall(&s, true, true);
  tripAt = simRun(&s, MOTOR_GUARD_LATE_US + 1, s.now, s.now + 1000000, true, true);
  CHECK(tripAt == 0 && s.c.lateCalls > 0, "15.001 ms gaps are late");
}

static void testJitterStillTrips(void)
{
  /* Callback gaps of 3..9 ms (esp_timer task delayed by Wi-Fi / cam_task). */
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  srand(12345);
  uint64_t tripAt = 0, freezeAt = 0;
  for (int i = 0; i < 4000 && !tripAt; i++) {
    const uint64_t gap = 3000 + (uint64_t)(rand() % 6001);
    if (i == 2000) freezeAt = s.now;
    simAdvance(&s, gap, freezeAt == 0);
    if (simCall(&s, true, true) == MG_TRIP) tripAt = s.now;
  }
  CHECK(freezeAt != 0 && tripAt != 0, "a frozen loop trips under jitter");
  const uint64_t lat = tripAt - freezeAt;
  CHECK(lat >= MOTOR_GUARD_FREEZE_US && lat <= MOTOR_GUARD_FREEZE_US + 2 * 9000, "latency %.1f ms", lat / 1000.0);
  CHECK(s.c.lateCalls == 0, "jitter below 15 ms is not late (%u)", (unsigned)s.c.lateCalls);
  CHECK(s.c.maxCallGapUs <= 9000 && s.c.maxCallGapUs >= 8000, "maxCallGapUs (%u)", (unsigned)s.c.maxCallGapUs);
}

static void testDisabledKeepsNoWindow(void)
{
  sim_t s;
  simInit(&s);
  simCall(&s, true, true);
  int disabled = 0;
  for (int i = 0; i < 400; i++) {            /* frozen 2 s, motors on, guard off */
    simAdvance(&s, P, false);
    if (simCall(&s, true, false) == MG_DISABLED) disabled++;
    CHECK(s.c.stallUs == 0, "no window while disabled");
  }
  CHECK(disabled == 400 && s.trips == 0, "disabled never trips (%d)", disabled);
  CHECK(s.c.maxStallUs == 0, "nothing counted while disabled (%u)", (unsigned)s.c.maxStallUs);
  const uint64_t on = s.now;
  uint64_t tripAt = simRun(&s, P, 0, s.now + 500000, true, true);
  CHECK(tripAt == on + MOTOR_GUARD_FREEZE_US, "after enabling, 80 ms of new observation (%lld ms)",
        (long long)(tripAt - on) / 1000);
}

static void testBackwardsClockIsLate(void)
{
  motorGuardCore_t c;
  motorGuardCoreInit(&c);
  motorGuardCoreStep(&c, T0_US, 5, true, true);
  motorGuardCoreStep(&c, T0_US + P, 5, true, true);
  CHECK(motorGuardCoreStep(&c, T0_US, 5, true, true) == MG_LATE, "time going backwards restarts");
  CHECK(c.lateCalls == 1 && c.stallUs == 0, "counted late, window restarted");
}

static void testAliveWrapAround(void)
{
  motorGuardCore_t c;
  motorGuardCoreInit(&c);
  motorGuardCoreStep(&c, T0_US, UINT32_MAX - 1, true, true);
  CHECK(motorGuardCoreStep(&c, T0_US + P, UINT32_MAX, true, true) == MG_OK, "progress near the wrap");
  CHECK(motorGuardCoreStep(&c, T0_US + 2 * P, 3, true, true) == MG_OK, "progress across the wrap");
}

int main(void)
{
  testFirstCallIsLateNotCounted();
  testSteadyProgressNeverTrips();
  testFrozenMotorsOnTripsOnce();
  testTripTimingExact();
  testFrozenMotorsOffNeverTrips();
  testMotorsOffBrieflyRestarts();
  testNearMissThenProgress();
  testSystemStallNotCountedButKeepsObserved();
  testLateCallWithProgressRestarts();
  testOftenLateGuardStillTrips();
  testLegitStallAcrossFlashEraseNoTrip();
  testFlashEraseNoTrip();
  testRepeatedLateGapsNeverTrip();
  testLateThresholdEdge();
  testJitterStillTrips();
  testDisabledKeepsNoWindow();
  testBackwardsClockIsLate();
  testAliveWrapAround();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
