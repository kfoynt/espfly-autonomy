/*
 * Host unit tests for components/core/crazyflie/utils/src/att_trim_learn.c
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_att_trim_learn.c components/core/crazyflie/utils/src/att_trim_learn.c \
 *      -lm -o /tmp/test_att_trim_learn && /tmp/test_att_trim_learn [CSV_DIR]
 *
 * With CSV_DIR (firmware-backups/cal) the two 2026-10-13 interactive-hover
 * sorties are replayed through the learner as att_trim_auto.c feeds it
 * (the recorder's 100 Hz rows; the take-off window is not in the log and is
 * left out): the guards leave the trim alone.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "att_trim_learn.h"
#include "auto_fsm.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define RATE 100

static attLearnSample_t steady(float corrR, float corrP)
{
  attLearnSample_t s = {
    .holdApplied = true, .holdOnView = true, .sticksCentred = true, .airborne = true,
    .corrRollDeg = corrR, .corrPitchDeg = corrP, .rollDeg = 1.0f, .pitchDeg = -1.0f, .gyroDps = 5.0f,
  };
  return s;
}

/* One flight: 'seconds' of steady hover in which the hold applies the lean
 * still missing from the trim (needed - trim), plus a per-flight offset. */
static bool fly(float *trimR, float *trimP, float needR, float needP, float offR, float offP, float seconds)
{
  attLearnFlight_t f;
  attLearnStart(&f, RATE);
  for (int i = 0; i < (int)(seconds * RATE); i++) {
    attLearnSample_t s = steady(needR - *trimR + offR, needP - *trimP + offP);
    attLearnAdd(&f, &s);
  }
  float mr, mp;
  return attLearnFinish(&f, RATE, trimR, trimP, &mr, &mp);
}

static void testConverges(void)
{
  float r = 0.0f, p = 0.0f;
  for (int i = 0; i < 12; i++) {
    CHECK(fly(&r, &p, 0.15f, 0.35f, 0.0f, 0.0f, 10.0f), "steady flight %d qualifies", i);
  }
  CHECK(fabsf(r - 0.15f) < 0.01f && fabsf(p - 0.35f) < 0.01f, "trim converges to the needed lean (%.3f, %.3f)", r, p);
}

static void testShortFlightIgnored(void)
{
  float r = 0.1f, p = 0.2f;
  CHECK(!fly(&r, &p, 0.5f, 0.5f, 0.0f, 0.0f, ATT_LEARN_SETTLE_S + ATT_LEARN_MIN_S - 0.1f),
        "a flight under settle + minimum is ignored");
  CHECK(r == 0.1f && p == 0.2f, "and leaves the trim unchanged");
}

static void testUnsteadySamplesExcluded(void)
{
  attLearnFlight_t f;
  attLearnStart(&f, RATE);
  for (int i = 0; i < 20 * RATE; i++) {
    attLearnSample_t s = steady(2.0f, 2.0f);
    switch (i % 5) {
      case 0: s.holdApplied = false; break;         /* hold off or stale */
      case 1: s.sticksCentred = false; break;       /* pilot input */
      case 2: s.pitchDeg = 15.0f; break;            /* manoeuvring */
      case 3: s.gyroDps = 200.0f; break;            /* bump, flip, landing */
      default: s.airborne = false; break;           /* on the ground */
    }
    attLearnAdd(&f, &s);
  }
  CHECK(f.n == 0, "no unsteady sample is counted (%u)", (unsigned)f.n);
  attLearnSample_t nan = steady(NAN, 0.0f);
  attLearnAdd(&f, &nan);
  CHECK(f.n == 0, "a non-finite sample is ignored");
}

static void testStepAndLimit(void)
{
  float r = 0.0f, p = 0.0f;
  fly(&r, &p, 0.95f, -0.95f, 0.0f, 0.0f, 10.0f);
  CHECK(fabsf(r - ATT_LEARN_MAX_STEP_DEG) < 1e-5f && fabsf(p + ATT_LEARN_MAX_STEP_DEG) < 1e-5f,
        "one flight moves the trim at most one step (%.3f, %.3f)", r, p);
  for (int i = 0; i < 50; i++) {
    fly(&r, &p, 0.95f, -0.95f, 0.0f, 0.0f, 10.0f);
  }
  CHECK(fabsf(r - ATT_LEARN_LIMIT_DEG) < 1e-5f && fabsf(p + ATT_LEARN_LIMIT_DEG) < 1e-5f,
        "the learned trim stops at the limit (%.3f, %.3f)", r, p);
}

static void testHandSetTrimNotPushedOut(void)
{
  float r = 1.2f, p = 1.2f;
  fly(&r, &p, 1.9f, 0.6f, 0.0f, 0.0f, 10.0f);
  CHECK(r == 1.2f, "a hand-set trim beyond the limit is not pushed further out (%.3f)", r);
  CHECK(p < 1.2f && p > 1.2f - ATT_LEARN_MAX_STEP_DEG - 1e-5f, "but moves toward zero when the hold asks (%.3f)", p);
}

/* Pilot input, then release: the hold brakes the drift the pilot left (a
 * decaying correction), then holds a drone that needs no lean at all. */
static void testBrakingAfterReleaseNotLearned(void)
{
  attLearnFlight_t f;
  attLearnStart(&f, RATE);
  for (int i = 0; i < RATE; i++) {
    attLearnSample_t s = steady(0.0f, 0.0f);
    s.sticksCentred = false;
    attLearnAdd(&f, &s);
  }
  for (int i = 0; i < 12 * RATE; i++) {
    const float brake = 2.0f * expf(-(float)i / (0.6f * RATE));   /* ~2 s of braking */
    attLearnSample_t s = steady(0.0f, brake);
    attLearnAdd(&f, &s);
  }
  float r = 0.0f, p = 0.0f, mr = 0.0f, mp = 0.0f;
  CHECK(attLearnFinish(&f, RATE, &r, &p, &mr, &mp), "the steady part of the flight qualifies");
  CHECK(fabsf(mp) < 0.02f, "braking after a stick release is not learned as trim (mean %.3f)", mp);

  /* A stick blip mid-hover restarts the settle time. */
  attLearnStart(&f, RATE);
  for (int i = 0; i < 4 * RATE; i++) {
    attLearnSample_t s = steady(0.0f, 0.0f);
    attLearnAdd(&f, &s);
  }
  const uint32_t before = f.n;
  attLearnSample_t blip = steady(0.0f, 0.0f);
  blip.sticksCentred = false;
  attLearnAdd(&f, &blip);
  for (int i = 0; i < 2 * RATE; i++) {
    attLearnSample_t s = steady(0.0f, 1.0f);
    attLearnAdd(&f, &s);
  }
  CHECK(f.n == before, "a stick blip restarts the settle time (%u counted after it)", (unsigned)(f.n - before));
}

static void testCoastingKeepsSettled(void)
{
  attLearnFlight_t f;
  attLearnStart(&f, RATE);
  for (int i = 0; i < 4 * RATE; i++) {
    attLearnSample_t s = steady(0.1f, 0.3f);
    if (i >= 3 * RATE + 20 && i < 3 * RATE + 40) {
      s.holdOnView = false;   /* view lost for 0.2 s: coasting */
    }
    attLearnAdd(&f, &s);
  }
  CHECK(f.n == RATE - 20, "coasting skips samples without restarting the settle (%u)", (unsigned)f.n);
}

static void testDraftAxisSkipped(void)
{
  float r = 0.0f, p = 0.0f;
  fly(&r, &p, 0.3f, 1.4f, 0.0f, 0.0f, 10.0f);
  CHECK(p == 0.0f, "a pitch mean beyond what trim explains is not learned (%.3f)", p);
  CHECK(r > 0.0f, "the other axis still learns (%.3f)", r);
}

static void testScatterAveragesOut(void)
{
  float r = 0.0f, p = 0.0f;
  srand(11);
  float worst = 0.0f, sq = 0.0f;
  int n = 0;
  for (int i = 0; i < 60; i++) {
    /* flight-to-flight scatter ~0.22 deg, as measured */
    float oR = ((rand() % 2001) - 1000) / 1000.0f * 0.38f;
    float oP = ((rand() % 2001) - 1000) / 1000.0f * 0.38f;
    fly(&r, &p, 0.15f, 0.35f, oR, oP, 10.0f);
    if (i >= 20) {
      worst = fmaxf(worst, fmaxf(fabsf(r - 0.15f), fabsf(p - 0.35f)));
      sq += (r - 0.15f) * (r - 0.15f) + (p - 0.35f) * (p - 0.35f);
      n += 2;
    }
  }
  const float rms = sqrtf(sq / n);
  /* gain 0.3 shrinks the 0.22 deg scatter to ~0.09 deg rms */
  CHECK(rms < 0.12f, "with flight-to-flight scatter the trim averages it out (rms %.3f)", rms);
  CHECK(worst < 0.3f, "and never strays far (worst %.3f)", worst);
}

/* ---- guards (2026-10-13) ------------------------------------------------- */

static void testMotionRestartsSettle(void)
{
  attLearnFlight_t f;
  attLearnStart(&f, RATE);
  for (int i = 0; i < 4 * RATE; i++) {          /* settled, 1 s counted */
    attLearnSample_t s = steady(0.1f, 0.3f);
    attLearnAdd(&f, &s);
  }
  const uint32_t n0 = f.n;
  for (int i = 0; i < 2 * RATE; i++) {          /* a MOVE: its feedforward lean in the correction */
    attLearnSample_t s = steady(1.5f, -2.0f);
    s.motionActive = true;
    attLearnAdd(&f, &s);
  }
  CHECK(n0 == RATE && f.n == n0 && f.settle == 0, "samples during a motion are not counted and restart the settle "
        "(%u counted, settle %u)", (unsigned)(f.n - n0), (unsigned)f.settle);
  for (int i = 0; i < 2 * RATE; i++) {          /* braking after it: inside the new settle time */
    attLearnSample_t s = steady(0.1f, 1.0f);
    attLearnAdd(&f, &s);
  }
  CHECK(f.n == n0, "after the motion the settle time runs again (%u counted)", (unsigned)(f.n - n0));
}

static void testSaturatedSkipped(void)
{
  attLearnFlight_t f;
  attLearnStart(&f, RATE);
  for (int i = 0; i < 4 * RATE; i++) {
    attLearnAdd(&f, &(attLearnSample_t){ 0 });   /* nothing: settle stays 0 */
  }
  for (int i = 0; i < 6 * RATE; i++) {
    attLearnSample_t s = steady(0.1f, i >= 4 * RATE && i < 5 * RATE ? -2.45f : 0.3f);
    attLearnAdd(&f, &s);
  }
  CHECK(f.n == 2 * RATE, "a correction at the hold's limit (-2.45 deg) is not counted, the settle kept (%u)",
        (unsigned)f.n);
  attLearnStart(&f, RATE);
  for (int i = 0; i < 4 * RATE; i++) {
    attLearnSample_t s = steady(i >= 3 * RATE ? 2.4f : 0.1f, 0.3f);
    attLearnAdd(&f, &s);
  }
  CHECK(f.n == 0, "roll 2.4 deg is skipped too (%u)", (unsigned)f.n);
  attLearnStart(&f, RATE);
  for (int i = 0; i < 4 * RATE; i++) {
    attLearnSample_t s = steady(2.39f, -2.39f);
    attLearnAdd(&f, &s);
  }
  CHECK(f.n == RATE, "2.39 deg is counted (%u)", (unsigned)f.n);
}

static void testEndOk(void)
{
  CHECK(attLearnEndOk(false, AUTO_IDLE, AUTO_RS_NONE, 0), "a manual flight: learned");
  CHECK(attLearnEndOk(false, AUTO_KILLED, AUTO_RS_TILT, 3), "no autonomy in the flight: the end fields do not matter");
  const autoReason_t ok[] = { AUTO_RS_HOVER_TIMEOUT, AUTO_RS_LAND_CMD, AUTO_RS_SORTIE_DONE, AUTO_RS_BATT_LOW };
  for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) {
    CHECK(attLearnEndOk(true, AUTO_LAND, ok[i], 0), "LAND reason %d: learned", (int)ok[i]);
    /* "Claude decides" (2026-10-15): a contact, the camera height lost, NOT_STABLE ... no longer end the sortie */
    CHECK(!attLearnEndOk(true, AUTO_LAND, ok[i], 1), "LAND reason %d after a reported event: not learned", (int)ok[i]);
  }
  CHECK(!attLearnEndOk(true, AUTO_LAND, AUTO_RS_SINK, 1), "LAND SINK: not learned");
  const struct { autoState_t st; autoReason_t why; } bad[] = {
    { AUTO_LAND, AUTO_RS_VZ }, { AUTO_LAND, AUTO_RS_HOVER_NO_CAM }, { AUTO_LAND, AUTO_RS_NOT_STABLE },
    { AUTO_LAND, AUTO_RS_APP_INPUT }, { AUTO_LAND, AUTO_RS_HB_LOST }, { AUTO_LAND, AUTO_RS_AIR_TIME },
    { AUTO_LAND, AUTO_RS_MOVE_NO_CAM }, { AUTO_LAND, AUTO_RS_MOVE_TILT }, { AUTO_LAND, AUTO_RS_CAPTURE_FAIL },
    { AUTO_KILLED, AUTO_RS_COLLISION }, { AUTO_LAND, AUTO_RS_COLLISION },   /* KILLED until eb9264d, LAND since */
    { AUTO_KILLED, AUTO_RS_TILT }, { AUTO_KILLED, AUTO_RS_KILL_CMD },
    { AUTO_HOVER, AUTO_RS_HOLD }, { AUTO_IDLE, AUTO_RS_TOUCHDOWN },
  };
  for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
    CHECK(!attLearnEndOk(true, (uint8_t)bad[i].st, (uint8_t)bad[i].why, 0), "state %d reason %d: not learned",
          (int)bad[i].st, (int)bad[i].why);
  }
}

/* ---- replay of the 2026-10-13 sorties ---------------------------------------- */

#define MAX_ROWS 20000   /* a 180 s recorder dump (docs/autonomy-a2b.md 19.3) */
enum { C_FID, C_THRUST, C_RDES, C_PDES, C_ROLL, C_PITCH, C_GX, C_GY, C_OK, C_HF, C_HR, C_HP, C_AUTO, C_WHY, C_MV, C_N };
static const char *colNames[C_N] = {
  "flightId", "thrust", "rollDes_cdeg", "pitchDes_cdeg", "roll_cdeg", "pitch_cdeg", "gx_ddps", "gy_ddps", "flowOk",
  "holdFlags", "holdRoll_cdeg", "holdPitch_cdeg", "autoState", "autoReason", "mvState",
};
static long rows[MAX_ROWS][C_N];
static int nRows;

static bool loadCsv(const char *path)
{
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    return false;
  }
  static char line[8192];
  int idx[C_N];
  for (int c = 0; c < C_N; c++) {
    idx[c] = -1;
  }
  if (fgets(line, sizeof(line), f) == NULL) {
    fclose(f);
    return false;
  }
  int col = 0;
  for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
    for (int c = 0; c < C_N; c++) {
      if (strcmp(tok, colNames[c]) == 0) {
        idx[c] = col;
      }
    }
  }
  for (int c = 0; c < C_N; c++) {
    if (idx[c] < 0) {
      fclose(f);
      return false;
    }
  }
  nRows = 0;
  while (nRows < MAX_ROWS && fgets(line, sizeof(line), f) != NULL) {
    col = 0;
    for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
      for (int c = 0; c < C_N; c++) {
        if (idx[c] == col) {
          rows[nRows][c] = strtol(tok, NULL, 10);
        }
      }
    }
    nRows++;
  }
  fclose(f);
  return true;
}

/* att_trim_auto.c over one flight's rows; guarded: the end and motion guards on. Returns whether the
 * trim was learned, and the trim after it. */
static bool replayFlight(int fid, bool guarded, float *trimR, float *trimP, float *mr, float *mp, int *endState,
                         int *endReason)
{
  attLearnFlight_t f;
  attLearnStart(&f, RATE);
  bool open = false, autoSeen = false;
  int st = 0, why = 0;
  for (int i = 0; i < nRows; i++) {
    const long *r = rows[i];
    if (r[C_FID] != fid) {
      continue;
    }
    if (r[C_THRUST] > 0) {
      open = true;
      autoSeen = autoSeen || r[C_AUTO] != 0;
      st = (int)r[C_AUTO];
      why = (int)r[C_WHY];
      const bool applied = (r[C_HF] & 2) != 0, coasting = (r[C_HF] & 4) != 0;
      const float gx = fabsf((float)r[C_GX] / 10.0f), gy = fabsf((float)r[C_GY] / 10.0f);
      const int mv = (int)(r[C_MV] & 0x0F);
      attLearnSample_t s = {
        .holdApplied = applied && (coasting || r[C_OK]), .holdOnView = applied && !coasting && r[C_OK],
        .sticksCentred = r[C_RDES] == 0 && r[C_PDES] == 0, .airborne = r[C_THRUST] > 30000,
        .corrRollDeg = (float)r[C_HR] / 100.0f, .corrPitchDeg = (float)r[C_HP] / 100.0f,
        .rollDeg = (float)r[C_ROLL] / 100.0f, .pitchDeg = (float)r[C_PITCH] / 100.0f, .gyroDps = gx > gy ? gx : gy,
        .motionActive = guarded && (mv == AUTO_MV_RUN || mv == AUTO_MV_PAUSED || mv == AUTO_MV_SETTLE),
      };
      if (!guarded && (fabsf(s.corrRollDeg) >= ATT_LEARN_SAT_DEG || fabsf(s.corrPitchDeg) >= ATT_LEARN_SAT_DEG)) {
        s.corrRollDeg = fminf(fmaxf(s.corrRollDeg, -2.39f), 2.39f);   /* the old learner counted them: */
        s.corrPitchDeg = fminf(fmaxf(s.corrPitchDeg, -2.39f), 2.39f); /* an approximation (<= 0.11 deg off) */
      }
      attLearnAdd(&f, &s);
    } else if (open) {
      autoSeen = autoSeen || r[C_AUTO] != 0;   /* the record that closes the flight */
      st = (int)r[C_AUTO];
      why = (int)r[C_WHY];
      break;
    }
  }
  *endState = st;
  *endReason = why;
  *mr = *mp = NAN;
  if (guarded && !attLearnEndOk(autoSeen, (uint8_t)st, (uint8_t)why, 0)) {   /* (the logged sorties: before events) */
    return false;
  }
  return attLearnFinish(&f, RATE, trimR, trimP, mr, mp);
}

static void replay(const char *dir)
{
  char path[1024];
  snprintf(path, sizeof(path), "%s/flight-2026-10-13-interactive.csv", dir);
  if (!loadCsv(path)) {
    CHECK(false, "no %s", path);
    return;
  }
  /* the trims in force at each take-off (flights table: roll/pitch) */
  const struct { int fid; float r, p; } fl[] = { { 1, 0.24f, 0.78f }, { 2, 0.26f, 0.63f } };
  for (unsigned k = 0; k < 2; k++) {
    float r0 = fl[k].r, p0 = fl[k].p, mr, mp;
    int st, why;
    const bool learnedOld = replayFlight(fl[k].fid, false, &r0, &p0, &mr, &mp, &st, &why);
    float r1 = fl[k].r, p1 = fl[k].p, mr1, mp1;
    const bool learned = replayFlight(fl[k].fid, true, &r1, &p1, &mr1, &mp1, &st, &why);
    printf("2026-10-13 flight %d (ended state %d reason %d, %s): without the guards %s (hold mean %+.3f/%+.3f -> "
           "trim %.2f/%.2f roll/pitch from %.2f/%.2f); with them %s, trim %.2f/%.2f\n", fl[k].fid, st, why,
           st == AUTO_LAND && why == AUTO_RS_VZ ? "LAND VZ" : st == AUTO_LAND && why == AUTO_RS_HOVER_NO_CAM ?
           "LAND HOVER_NO_CAM" : "?", learnedOld ? "learned" : "not learned", mr, mp, r0, p0, fl[k].r, fl[k].p,
           learned ? "learned" : "not learned", r1, p1);
    CHECK(!learned && r1 == fl[k].r && p1 == fl[k].p, "2026-10-13 flight %d: no trim change with the guards",
          fl[k].fid);
  }
}

int main(int argc, char **argv)
{
  testConverges();
  testShortFlightIgnored();
  testUnsteadySamplesExcluded();
  testStepAndLimit();
  testHandSetTrimNotPushedOut();
  testScatterAveragesOut();
  testBrakingAfterReleaseNotLearned();
  testCoastingKeepsSettled();
  testDraftAxisSkipped();
  testMotionRestartsSettle();
  testSaturatedSkipped();
  testEndOk();
  if (argc > 1) {
    replay(argv[1]);
  } else {
    printf("replay skipped: pass the directory with flight-2026-10-13-interactive.csv\n");
  }
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
