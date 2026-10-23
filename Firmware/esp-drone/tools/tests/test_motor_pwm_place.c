/*
 * Host unit tests for components/core/crazyflie/utils/src/motor_pwm_place.c
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_motor_pwm_place.c components/core/crazyflie/utils/src/motor_pwm_place.c \
 *      -lm -o /tmp/test_motor_pwm_place && /tmp/test_motor_pwm_place
 *
 * The LEDC model: an 8-bit counter runs 0..255 each period; a channel goes
 * high when the counter equals hpoint and low when it equals hpoint + duty,
 * so it is on for the counts hpoint .. hpoint + duty - 1 (hpoint + duty <=
 * 255: no wrap). The ripple metric is the variance, over one period, of the
 * number of motors on (the battery current in motor currents).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_pwm_place.h"

#define TOP MOTOR_PWM_PLACE_TOP
#define COUNTS (TOP + 1u)
#define N MOTOR_PWM_PLACE_MOTORS

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

/* The placement before this change (motors.c staggerHpoint, 2026-10-11). */
static uint32_t oldStagger(uint32_t id, uint32_t duty)
{
  if (duty == 0 || duty >= TOP) return 0;
  return id * (TOP - duty) / (N - 1);
}

/* On counts of one channel in one period; -1 if the pulse would wrap. */
static int onCounts(uint32_t hpoint, uint32_t duty, unsigned char on[COUNTS])
{
  for (uint32_t t = 0; t < COUNTS; t++) on[t] = 0;
  if (duty == 0) return 0;
  if (hpoint + duty > TOP) return -1;
  for (uint32_t t = hpoint; t < hpoint + duty; t++) on[t] = 1;
  return (int)duty;
}

/* nmin/nmax leave out the last count (255): every pulse is low there, since
 * a duty of at most 255 ends at count 255 at the latest. */
typedef struct { double mean, var; int nmin, nmax, nLast; } ripple_t;

static ripple_t ripple(const uint32_t hp[N], const uint32_t duty[N])
{
  unsigned char on[N][COUNTS];
  for (uint32_t i = 0; i < N; i++) (void)onCounts(hp[i], duty[i], on[i]);
  double s = 0, s2 = 0;
  ripple_t r = { 0, 0, 99, -1, 0 };
  for (uint32_t t = 0; t < COUNTS; t++) {
    int n = 0;
    for (uint32_t i = 0; i < N; i++) n += on[i][t];
    s += n; s2 += (double)n * n;
    if (t == TOP) { r.nLast = n; continue; }
    if (n < r.nmin) r.nmin = n;
    if (n > r.nmax) r.nmax = n;
  }
  r.mean = s / COUNTS;
  r.var = s2 / COUNTS - r.mean * r.mean;
  return r;
}

static ripple_t rippleOld(const uint32_t duty[N])
{
  uint32_t hp[N];
  for (uint32_t i = 0; i < N; i++) hp[i] = oldStagger(i, duty[i]);
  return ripple(hp, duty);
}

static ripple_t rippleNew(motorPwmPlace_t *p, const uint32_t duty[N])
{
  uint32_t hp[N];
  for (uint32_t i = 0; i < N; i++) hp[i] = motorPwmPlaceHpoint(p, i, duty[i]);
  return ripple(hp, duty);
}

/* Three histories: boot, after a full-duty loop (aligned), after a stop. */
static void history(motorPwmPlace_t *p, int which)
{
  motorPwmPlaceInit(p);
  for (uint32_t i = 0; i < N; i++) {
    if (which == 1) (void)motorPwmPlaceHpoint(p, i, TOP);
    if (which == 2) { (void)motorPwmPlaceHpoint(p, i, TOP); (void)motorPwmPlaceHpoint(p, i, 0); }
  }
}

static void testLimits(void)
{
  int bad = 0, wraps = 0, fullOn = 0, wrongLen = 0, zeroBad = 0, topBad = 0;
  for (int h = 0; h < 3; h++) {
    for (uint32_t id = 0; id < N; id++) {
      for (uint32_t duty = 0; duty <= 300; duty++) {
        motorPwmPlace_t p;
        history(&p, h);
        const uint32_t hp = motorPwmPlaceHpoint(&p, id, duty);
        const uint32_t d = duty > TOP ? TOP : duty;
        if (hp > TOP) bad++;
        if (hp + d > TOP) wraps++;
        unsigned char on[COUNTS];
        const int len = onCounts(hp, d, on);
        if (len != (int)d) wrongLen++;
        int low = 0;
        for (uint32_t t = 0; t < COUNTS; t++) low += !on[t];
        if (low < 1) fullOn++;
        if (duty == 0 && hp != 0) zeroBad++;
        if (duty >= TOP && hp != 0) topBad++;
      }
    }
  }
  CHECK(bad == 0, "hpoint within the counter (%d)", bad);
  CHECK(wraps == 0, "hpoint + duty <= 255 for every id, duty 0..300 and history (%d)", wraps);
  CHECK(wrongLen == 0, "each period is on for exactly the commanded duty (%d)", wrongLen);
  CHECK(fullOn == 0, "never full-on: at least one count low per period (%d)", fullOn);
  CHECK(zeroBad == 0, "duty 0 is hpoint 0 (%d)", zeroBad);
  CHECK(topBad == 0, "duty 255 (and above, clamped) is hpoint 0 (%d)", topBad);

  motorPwmPlace_t p;
  motorPwmPlaceInit(&p);
  (void)motorPwmPlaceHpoint(&p, 1, 200);
  const uint8_t before = p.aligned;
  CHECK(motorPwmPlaceHpoint(&p, 4, 200) == 0 && motorPwmPlaceHpoint(&p, 99, 10) == 0, "id out of range gives 0");
  CHECK(p.aligned == before, "id out of range changes nothing");
  CHECK(!motorPwmPlaceAligned(&p, 4), "id out of range is not aligned");
}

static void testSpreadAtOrBelowHalf(void)
{
  int diff = 0;
  for (int h = 0; h < 3; h++) {
    for (uint32_t id = 0; id < N; id++) {
      for (uint32_t duty = 0; duty <= MOTOR_PWM_PLACE_ALIGN_OFF; duty++) {
        motorPwmPlace_t p;
        history(&p, h);
        if (motorPwmPlaceHpoint(&p, id, duty) != oldStagger(id, duty)) diff++;
        if (motorPwmPlaceAligned(&p, id)) diff++;
      }
    }
  }
  CHECK(diff == 0, "at or below 50 %% (duty <= 128) the placement is the old spread, whatever came before (%d)", diff);
  CHECK(MOTOR_PWM_PLACE_ALIGN_OFF * 2 == COUNTS, "the spread threshold is half the period");
}

static void testAlignedAboveOn(void)
{
  int diff = 0;
  for (int h = 0; h < 3; h++) {
    for (uint32_t duty = MOTOR_PWM_PLACE_ALIGN_ON; duty <= TOP; duty++) {
      motorPwmPlace_t p;
      history(&p, h);
      const uint32_t h0 = motorPwmPlaceHpoint(&p, 0, duty), h1 = motorPwmPlaceHpoint(&p, 1, duty);
      const uint32_t h2 = motorPwmPlaceHpoint(&p, 2, duty), h3 = motorPwmPlaceHpoint(&p, 3, duty);
      if (h0 != 0 || h2 != 0) diff++;                       /* M1, M3 start the period */
      if (h1 != TOP - duty || h3 != TOP - duty) diff++;     /* M2, M4 end it */
      for (uint32_t i = 0; i < N; i++) diff += !motorPwmPlaceAligned(&p, i);
    }
  }
  CHECK(diff == 0, "duty >= 141: M1+M3 at hpoint 0, M2+M4 at 255 - duty (%d)", diff);
  CHECK(MOTOR_PWM_PLACE_ALIGN_ON > MOTOR_PWM_PLACE_ALIGN_OFF, "a hysteresis band");
}

static void testM1M4NeverMove(void)
{
  int diff = 0;
  for (int h = 0; h < 3; h++) {
    for (uint32_t duty = 0; duty <= TOP; duty++) {
      motorPwmPlace_t p;
      history(&p, h);
      if (motorPwmPlaceHpoint(&p, 0, duty) != oldStagger(0, duty)) diff++;
      if (motorPwmPlaceHpoint(&p, 3, duty) != oldStagger(3, duty)) diff++;
    }
  }
  CHECK(diff == 0, "M1 and M4 sit where they did before in both layouts (%d)", diff);
}

/* Layout changes of motor 'id' along a duty sequence. */
static int toggles(motorPwmPlace_t *p, uint32_t id, const uint32_t *seq, int n, int *firstAt)
{
  int count = 0;
  bool was = motorPwmPlaceAligned(p, id);
  *firstAt = -1;
  for (int k = 0; k < n; k++) {
    (void)motorPwmPlaceHpoint(p, id, seq[k]);
    const bool now = motorPwmPlaceAligned(p, id);
    if (now != was) { count++; if (*firstAt < 0) *firstAt = k; }
    was = now;
  }
  return count;
}

static void testHysteresis(void)
{
  uint32_t up[TOP + 1], down[TOP + 1];
  for (uint32_t k = 0; k <= TOP; k++) { up[k] = k; down[k] = TOP - k; }
  for (uint32_t id = 0; id < N; id++) {
    motorPwmPlace_t p;
    motorPwmPlaceInit(&p);
    int at;
    int c = toggles(&p, id, up, TOP + 1, &at);
    CHECK(c == 1 && up[at] == MOTOR_PWM_PLACE_ALIGN_ON, "id %u: one change on the way up, at 141 (%d at %d)", (unsigned)id, c, at);
    c = toggles(&p, id, down, TOP + 1, &at);
    CHECK(c == 1 && down[at] == MOTOR_PWM_PLACE_ALIGN_OFF, "id %u: one change on the way down, at 128 (%d at %d)", (unsigned)id, c, at);
  }

  /* Jitter inside the band (129..140) never changes the layout. */
  uint32_t band[2000];
  srand(7);
  for (int k = 0; k < 2000; k++) band[k] = 129u + (uint32_t)(rand() % 12);
  for (int start = 0; start < 2; start++) {
    motorPwmPlace_t p;
    motorPwmPlaceInit(&p);
    if (start) (void)motorPwmPlaceHpoint(&p, 1, 200);
    int at;
    const int c = toggles(&p, 1, band, 2000, &at);
    CHECK(c == 0 && motorPwmPlaceAligned(&p, 1) == (start != 0), "jitter 129..140 keeps the %s layout (%d)", start ? "aligned" : "spread", c);
  }

  /* +-3 counts of loop noise around either threshold: one change, then none. */
  uint32_t nearOff[1000], nearOn[1000];
  for (int k = 0; k < 1000; k++) {
    nearOff[k] = 125u + (uint32_t)(rand() % 7);   /* 125..131 */
    nearOn[k] = 138u + (uint32_t)(rand() % 7);    /* 138..144 */
  }
  motorPwmPlace_t p;
  motorPwmPlaceInit(&p);
  (void)motorPwmPlaceHpoint(&p, 2, 200);
  int at;
  int c = toggles(&p, 2, nearOff, 1000, &at);
  CHECK(c == 1 && !motorPwmPlaceAligned(&p, 2), "noise around 50 %% from aligned: one change to the spread (%d)", c);
  motorPwmPlaceInit(&p);
  c = toggles(&p, 2, nearOn, 1000, &at);
  CHECK(c == 1 && motorPwmPlaceAligned(&p, 2), "noise around 55 %% from the spread: one change to aligned (%d)", c);

  /* Each motor keeps its own layout. */
  motorPwmPlaceInit(&p);
  (void)motorPwmPlaceHpoint(&p, 1, 150);
  (void)motorPwmPlaceHpoint(&p, 2, 135);
  CHECK(motorPwmPlaceAligned(&p, 1) && !motorPwmPlaceAligned(&p, 2) && p.aligned == 0x2, "per-motor layouts (0x%x)", p.aligned);
}

/* A take-off: the thrust ramps through 50 % with loop noise and mixer
 * differences, hovers, then lands. Each motor changes layout once each way. */
static void testHalfCrossing(void)
{
  motorPwmPlace_t p;
  motorPwmPlaceInit(&p);
  const int offs[N] = { -6, 4, 9, -3 };   /* mixer: BL works harder */
  int changes[N] = { 0 }, maxHpJump = 0, worstWrap = 0;
  bool was[N] = { false };
  uint32_t lastHp[N] = { 0 };
  srand(11);
  for (int k = 0; k < 3000; k++) {
    int base;
    if (k < 600) base = k * 180 / 600;              /* ramp 0 -> 70 % in 0.6 s */
    else if (k < 2400) base = 180;                  /* hover */
    else base = 180 - (k - 2400) * 180 / 600;       /* land */
    for (uint32_t i = 0; i < N; i++) {
      int d = base + (base > 20 ? offs[i] : 0) + (rand() % 7) - 3;
      if (d < 0) d = 0;
      if (d > (int)TOP) d = TOP;
      const uint32_t hp = motorPwmPlaceHpoint(&p, i, (uint32_t)d);
      if (hp + (uint32_t)d > TOP) worstWrap++;
      const bool now = motorPwmPlaceAligned(&p, i);
      if (now != was[i]) {
        changes[i]++;
        const int jump = abs((int)hp - (int)lastHp[i]);
        if (jump > maxHpJump) maxHpJump = jump;
      }
      was[i] = now;
      lastHp[i] = hp;
    }
  }
  for (uint32_t i = 0; i < N; i++) {
    CHECK(changes[i] == 2, "motor %u: aligned once on the climb through 50-55 %%, spread once on the landing (%d)", (unsigned)i, changes[i]);
  }
  CHECK(worstWrap == 0, "no wrap through the crossing (%d)", worstWrap);
  printf("  crossing: largest hpoint jump at a layout change %d counts (one period's pulse moves; its length is unchanged)\n", maxHpJump);
}

static void testRippleAtHover(void)
{
  const uint32_t duties[] = { 166, 191, 217 };   /* 65, 75, 85 % */
  for (unsigned k = 0; k < 3; k++) {
    const uint32_t d[N] = { duties[k], duties[k], duties[k], duties[k] };
    motorPwmPlace_t p;
    motorPwmPlaceInit(&p);
    const ripple_t o = rippleOld(d), n = rippleNew(&p, d);
    printf("  duty %3u (%.0f %%): var %.3f -> %.3f (%+.0f %%), motors on %d..%d -> %d..%d, mean %.3f\n",
           (unsigned)duties[k], 100.0 * duties[k] / COUNTS, o.var, n.var, 100.0 * (n.var / o.var - 1.0),
           o.nmin, o.nmax, n.nmin, n.nmax, n.mean);
    CHECK(fabs(o.mean - n.mean) < 1e-12, "same mean current at %u", (unsigned)duties[k]);
    CHECK(n.var < 0.85 * o.var, "variance at least 15 %% lower at %u (%.3f vs %.3f)", (unsigned)duties[k], n.var, o.var);
    CHECK(n.nmin == 2 && n.nmax == 4 && o.nmin == 1 && o.nmax == 4, "swing 1..4 -> 2..4 motors at %u", (unsigned)duties[k]);
    CHECK(n.nLast == 0 && o.nLast == 0, "count 255 off in both layouts at %u", (unsigned)duties[k]);
  }

  /* Every equal duty from the aligned threshold up: never worse. */
  int worse = 0;
  for (uint32_t dd = MOTOR_PWM_PLACE_ALIGN_ON; dd <= TOP; dd++) {
    const uint32_t d[N] = { dd, dd, dd, dd };
    motorPwmPlace_t p;
    motorPwmPlaceInit(&p);
    if (rippleNew(&p, d).var > rippleOld(d).var + 1e-12) worse++;
  }
  CHECK(worse == 0, "equal duties 141..255: the aligned variance is never higher (%d)", worse);

  /* Hover-like unequal duties (each motor within +-12 % of the others, 60-90 %). */
  srand(3);
  int trials = 20000, better = 0;
  double sumGain = 0, worstGain = 1e9;
  for (int t = 0; t < trials; t++) {
    const int base = 154 + rand() % 60;
    uint32_t d[N];
    for (uint32_t i = 0; i < N; i++) {
      int v = base + rand() % 31 - 15;
      if (v > (int)TOP) v = TOP;
      if (v < (int)MOTOR_PWM_PLACE_ALIGN_ON) v = MOTOR_PWM_PLACE_ALIGN_ON;
      d[i] = (uint32_t)v;
    }
    motorPwmPlace_t p;
    motorPwmPlaceInit(&p);
    const double o = rippleOld(d).var, n = rippleNew(&p, d).var;
    if (n < o) better++;
    const double g = 1.0 - n / o;
    sumGain += g;
    if (g < worstGain) worstGain = g;
  }
  printf("  unequal hover duties: lower in %d of %d, mean %.0f %% lower, least %.0f %%\n",
         better, trials, 100.0 * sumGain / trials, 100.0 * worstGain);
  CHECK(better == trials, "unequal hover duties: the aligned variance is lower in every case (%d of %d)", better, trials);

  /* The BL motor 5-9 points above the rest (logged hovers, 2026-10-13). */
  const uint32_t bl[N] = { 180, 182, 199, 178 };
  motorPwmPlace_t p;
  motorPwmPlaceInit(&p);
  const ripple_t o = rippleOld(bl), n = rippleNew(&p, bl);
  printf("  duties 180,182,199,178: var %.3f -> %.3f\n", o.var, n.var);
  CHECK(n.var < 0.85 * o.var, "BL-heavy hover: variance at least 15 %% lower");
}

/* Why motors.c latches hpoint and duty with ONE parameter update: IDF
 * 5.0.7's ledc_set_duty_with_hpoint() + ledc_update_duty() latch the new
 * hpoint one update before the new duty. Count the duty steps where that
 * intermediate period (new hpoint, old duty) would wrap. Informative. */
static void testMixedUpdateHazard(void)
{
  long oldWraps = 0, newWraps = 0, steps = 0;
  for (uint32_t a = 1; a <= TOP; a++) {
    for (uint32_t b = 1; b <= TOP; b++) {
      if (abs((int)a - (int)b) > 20) continue;   /* per-loop duty steps */
      steps++;
      if (oldStagger(3, b) + a > TOP) oldWraps++;
      motorPwmPlace_t p;
      motorPwmPlaceInit(&p);
      (void)motorPwmPlaceHpoint(&p, 1, a);
      if (motorPwmPlaceHpoint(&p, 1, b) + a > TOP) newWraps++;
    }
  }
  printf("  two-step LEDC update: (new hpoint, old duty) wraps in %ld (M4, before) and %ld (M2, now) of %ld duty steps\n",
         oldWraps, newWraps, steps);
  CHECK(oldWraps > 0 && newWraps > 0, "a right-aligned pulse needs the single-latch update");
}

int main(void)
{
  testLimits();
  testSpreadAtOrBelowHalf();
  testAlignedAboveOn();
  testM1M4NeverMove();
  testHysteresis();
  testHalfCrossing();
  testRippleAtHover();
  testMixedUpdateHazard();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
