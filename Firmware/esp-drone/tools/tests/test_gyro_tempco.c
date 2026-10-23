/*
 * Host test: the gyro temperature compensation (components/core/crazyflie/utils/src/gyro_tempco.c,
 * docs/autonomy-a2b.md section 19.1) and a replay of its model against the logged hovers.
 *
 * Build and run from Firmware/esp-drone (the replay is skipped without the CSV directory):
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface tools/tests/test_gyro_tempco.c \
 *      components/core/crazyflie/utils/src/gyro_tempco.c components/core/crazyflie/utils/src/imu_level_cal.c \
 *      -lm -o /tmp/test_gyro_tempco && /tmp/test_gyro_tempco ../../firmware-backups/cal
 *
 * Unit checks: sign and magnitude (43 -> 33 C: corr y +0.600, x -0.180 dps; the corrected gyro of a drone whose
 * zero followed the model reads 0), the knee (47 -> 44 C: 0; 47 -> 38 C: y +0.360), the clamp, the latch and the
 * low-pass, NaN, 'off' (exactly 0, so the driver's output is bit-identical), the NVS record. The re-zero log: the
 * first re-zero after a flight stays readable through 600 quiet re-zeros (once a second on a still drone) and
 * through ground spins that wrap the ring (its own 'air' slot); a hold's and a large step's re-zero are logged.
 * The driver's block rate (docs/autonomy-a2b.md 19.11): gyroTcBlock with one sample is the per-sample step bit for
 * bit; the per-block pipeline (GYRO_TC_BLOCK samples, 20 Hz) against the per-sample one of b40a827 on a flight
 * cooling with re-zeros, the bench cooling test and a 1 C step; off subtracts +0.0f.
 *
 * Replay: every flight of cal/flights-2026-10-1[2-4]*.txt with an 'IMU C start->end' entry whose recorder rows
 * are in the matching flight-*.csv (2026-10-14 morning: flight 4 only; flight 1's rows were overwritten). The die
 * temperature is modelled as T(t) = Tend + (Tstart - Tend) e^(-t / tau) from lift-off (recorder time, the
 * summary's air time at the last airborne row), Tref = Tstart, and the model's bias (the defaults, knee 44 C)
 * is compared with the gyro zero seen in the steady hover windows (HOVER without a motion running, 3-5 s
 * chunks): the mean gx / gy minus the window's own attitude change (the hold's corrections are real rotations:
 * nose-up rate = -gy, roll rate = +gx), tau fitted (10..40 s) on the y axis.
 */
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gyro_tempco.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static bool near(float a, float b, float tol)
{
  return fabsf(a - b) <= tol;
}

/* A state at a steady temperature T, latched at Tref (the low-pass settled at each). */
static void stateAt(gyroTcState_t *st, float tRef, float t)
{
  gyroTcInit(st);
  for (int i = 0; i < 10000; i++) {
    gyroTcStep(st, tRef, 0.001f);
  }
  gyroTcLatch(st);
  for (int i = 0; i < 10000; i++) {
    gyroTcStep(st, t, 0.001f);
  }
}

static void unitTests(void)
{
  gyroTcConfig_t c;
  gyroTcDefaultConfig(&c);
  CHECK(c.on && c.k[0] == 0.018f && c.k[1] == -0.060f && c.k[2] == 0.0f && c.kneeC == 44.0f && c.clampDps == 1.0f,
        "defaults");
  CHECK(gyroTcConfigValid(&c), "defaults valid");

  gyroTcState_t st;
  float corr[3];

  /* sign and magnitude: flight 4, 43 -> 33 C */
  stateAt(&st, 43.0f, 33.0f);
  gyroTcCorr(&c, &st, corr);
  printf("43 -> 33 C: corr %+.4f %+.4f %+.4f dps\n", corr[0], corr[1], corr[2]);
  CHECK(near(corr[1], 0.600f, 0.002f) && near(corr[0], -0.180f, 0.001f) && corr[2] == 0.0f,
        "43->33: y +0.600 x -0.180, got %+.4f %+.4f", corr[1], corr[0]);
  {
    /* a pitch gyro whose zero followed b_y = -0.060 (T - Tref): the corrected reading is 0 */
    const float b[3] = { 0.018f * (33.0f - 43.0f), -0.060f * (33.0f - 43.0f), 0.0f };
    CHECK(near(b[1] - corr[1], 0.0f, 0.002f) && near(b[0] - corr[0], 0.0f, 0.002f), "corrected gyro ~0: %+.4f %+.4f",
          b[0] - corr[0], b[1] - corr[1]);
    /* the wrong sign would double it */
    CHECK(fabsf(b[1] + corr[1]) > 1.1f, "a sign error doubles the drift");
  }

  /* the knee */
  stateAt(&st, 47.0f, 44.0f);
  gyroTcCorr(&c, &st, corr);
  CHECK(corr[0] == 0.0f && corr[1] == 0.0f, "47->44 C: 0, got %+.4f %+.4f", corr[0], corr[1]);
  stateAt(&st, 47.0f, 38.0f);
  gyroTcCorr(&c, &st, corr);
  printf("47 -> 38 C: corr y %+.4f (flight 1)\n", corr[1]);
  CHECK(near(corr[1], 0.360f, 0.002f), "47->38 C: y +0.360, got %+.4f", corr[1]);
  stateAt(&st, 40.0f, 46.0f);
  gyroTcCorr(&c, &st, corr);
  CHECK(near(corr[1], -0.240f, 0.002f), "40->46 C (warming past the knee): y -0.240, got %+.4f", corr[1]);
  CHECK(gyroTcEffDelta(&c, 33.0f, 47.0f) == -11.0f, "effective delta 47->33 with the knee: -11");

  /* the clamp */
  stateAt(&st, 43.0f, 20.0f);
  gyroTcCorr(&c, &st, corr);
  CHECK(corr[1] == 1.0f && near(corr[0], -0.414f, 0.002f), "clamp 1.0 dps: y %+.4f x %+.4f", corr[1], corr[0]);
  {
    gyroTcConfig_t k = c;
    k.k[1] = 0.15f;
    gyroTcCorr(&k, &st, corr);
    CHECK(corr[1] == -1.0f, "clamp both signs: %+.4f", corr[1]);
  }

  /* latch semantics */
  gyroTcInit(&st);
  gyroTcCorr(&c, &st, corr);
  CHECK(corr[0] == 0.0f && corr[1] == 0.0f && corr[2] == 0.0f, "no sample: 0");
  gyroTcLatch(&st);
  CHECK(!st.refSet, "a latch before the first sample is ignored");
  gyroTcStep(&st, 43.0f, 0.001f);
  CHECK(st.seeded && st.tLp == 43.0f, "seeded by the first sample");
  for (int i = 0; i < 5000; i++) {
    gyroTcStep(&st, 33.0f, 0.001f);
  }
  gyroTcCorr(&c, &st, corr);
  CHECK(corr[1] == 0.0f, "no latch: 0 (got %+.4f)", corr[1]);
  gyroTcLatch(&st);
  gyroTcCorr(&c, &st, corr);
  CHECK(near(corr[1], 0.0f, 1e-5f), "right after a latch: ~0 (got %+.6f)", corr[1]);
  /* the low-pass: tau 0.5 s */
  stateAt(&st, 43.0f, 43.0f);
  for (int i = 0; i < 500; i++) {
    gyroTcStep(&st, 33.0f, 0.001f);
  }
  CHECK(near(st.tLp, 43.0f - 10.0f * (1.0f - expf(-1.0f)), 0.05f), "tau 0.5 s: %.3f after 0.5 s", st.tLp);
  /* a re-zero during cooling: the reference moves to the temperature now */
  stateAt(&st, 43.0f, 38.0f);
  gyroTcLatch(&st);
  for (int i = 0; i < 10000; i++) {
    gyroTcStep(&st, 33.0f, 0.001f);
  }
  gyroTcCorr(&c, &st, corr);
  CHECK(near(corr[1], 0.300f, 0.002f), "relative to the latest latch (38 -> 33): +0.300, got %+.4f", corr[1]);

  /* NaN: skipped, and a NaN state gives 0 */
  stateAt(&st, 43.0f, 33.0f);
  const float before = st.tLp;
  gyroTcStep(&st, NAN, 0.001f);
  gyroTcStep(&st, INFINITY, 0.001f);
  CHECK(st.tLp == before, "NaN / inf samples skipped");
  st.tLp = NAN;
  gyroTcCorr(&c, &st, corr);
  CHECK(corr[0] == 0.0f && corr[1] == 0.0f && corr[2] == 0.0f, "NaN state: 0");

  /* off: exactly 0, and the model still computes (for the re-zero log) */
  {
    gyroTcConfig_t off = c;
    off.on = false;
    stateAt(&st, 43.0f, 33.0f);
    gyroTcCorr(&off, &st, corr);
    CHECK(corr[0] == 0.0f && corr[1] == 0.0f && corr[2] == 0.0f && !signbit(corr[1]), "off: +0.0");
    float model[3];
    gyroTcModel(&off, &st, model);
    CHECK(near(model[1], 0.600f, 0.002f), "off: the model still reads %+.4f", model[1]);
    /* the driver skips the subtraction when off; were it done, x - 0.0f is x bit for bit anyway */
    bool same = true;
    for (int i = -20000; i <= 20000; i++) {
      const float x = (float)i * 0.0137f;
      const float y = x - corr[1];
      same = same && memcmp(&x, &y, sizeof(x)) == 0;
    }
    CHECK(same, "x - 0.0f bit-identical");
  }

  /* config validity */
  {
    gyroTcConfig_t k = c;
    k.k[1] = -0.16f;
    CHECK(!gyroTcConfigValid(&k), "|k| > 0.15 refused");
    k = c;
    k.kneeC = 29.0f;
    CHECK(!gyroTcConfigValid(&k), "knee < 30 refused");
    k.kneeC = 61.0f;
    CHECK(!gyroTcConfigValid(&k), "knee > 60 refused");
    k.kneeC = NAN;
    CHECK(!gyroTcConfigValid(&k), "NaN knee refused");
    k = c;
    k.k[0] = NAN;
    CHECK(!gyroTcConfigValid(&k), "NaN k refused");
  }

  /* the NVS record */
  {
    gyroTcConfig_t k = c;
    k.k[1] = -0.071f;
    k.kneeC = 45.5f;
    k.on = false;
    gyroTcRecord_t rec;
    gyroTcRecordInit(&rec, &k);
    CHECK(gyroTcRecordValid(&rec, sizeof(rec)), "record valid");
    CHECK(!gyroTcRecordValid(&rec, sizeof(rec) - 4), "wrong stored size refused");
    gyroTcConfig_t back;
    gyroTcDefaultConfig(&back);
    gyroTcRecordApply(&rec, &back);
    CHECK(back.k[1] == -0.071f && back.kneeC == 45.5f && !back.on && back.clampDps == GYRO_TC_CLAMP_DEFAULT,
          "record round trip");
    gyroTcRecord_t bad = rec;
    bad.k[0] += 0.001f;
    CHECK(!gyroTcRecordValid(&bad, sizeof(bad)), "crc refuses a changed value");
    bad = rec;
    bad.magic ^= 1u;
    CHECK(!gyroTcRecordValid(&bad, sizeof(bad)), "magic");
    bad = rec;
    bad.version = 2;
    CHECK(!gyroTcRecordValid(&bad, sizeof(bad)), "version");
    k.k[2] = 0.2f;   /* out of range, crc right */
    gyroTcRecordInit(&bad, &k);
    CHECK(!gyroTcRecordValid(&bad, sizeof(bad)), "an out-of-range value with a right crc refused");
  }
}

/* ---- replay ------------------------------------------------------------------------------------------------ */

#define MAX_ROWS 20000
enum { C_TICK, C_FID, C_THRUST, C_ROLL, C_PITCH, C_GX, C_GY, C_AUTO, C_MV, C_N };
static const char *colNames[C_N] = { "tickMs", "flightId", "thrust", "roll_cdeg", "pitch_cdeg", "gx_ddps",
                                     "gy_ddps", "autoState", "mvState" };
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

typedef struct {
  int boot, id;
  float airS, t0, t1;
} sumFlight_t;

/* The flights tables: the first one has "<boot> <id> <time>s <air>s"; the second "<boot> <id> ... A->B". */
static int loadSummary(const char *path, sumFlight_t *out, int max)
{
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    return 0;
  }
  static char line[1024];
  int n = 0;
  bool second = false;
  while (fgets(line, sizeof(line), f) != NULL) {
    if (strstr(line, "IMU C start->end") != NULL) {
      second = true;
      continue;
    }
    if (strstr(line, "camera height observer") != NULL) {
      second = false;
    }
    int boot, id;
    float tS, airS;
    if (!second && sscanf(line, " %d %d %fs %fs", &boot, &id, &tS, &airS) == 4 && n < max) {
      out[n].boot = boot;
      out[n].id = id;
      out[n].airS = airS;
      out[n].t0 = out[n].t1 = NAN;
      n++;
      continue;
    }
    if (second && sscanf(line, " %d %d", &boot, &id) == 2) {
      char *arrow = strstr(line, "->");
      if (arrow == NULL) {
        continue;
      }
      char *p = arrow;
      while (p > line && (isdigit((unsigned char)p[-1]) || p[-1] == '.' || p[-1] == '-')) {
        p--;
      }
      const float a = strtof(p, NULL), b = strtof(arrow + 2, NULL);
      for (int i = 0; i < n; i++) {
        if (out[i].boot == boot && out[i].id == id) {
          out[i].t0 = a;
          out[i].t1 = b;
        }
      }
    }
  }
  fclose(f);
  return n;
}

typedef struct {
  float tMid;      /* s from lift-off */
  float bx, by;    /* the gyro zero in the window (dps) */
  float gx, gy;    /* the plain means */
} win_t;

static bool steadyRow(int i, int fid)
{
  const long *r = rows[i];
  const int st = (int)(r[C_MV] & 15);
  return r[C_FID] == fid && r[C_AUTO] == 3 && r[C_THRUST] > 30000 && st != 1 && st != 2 && st != 3;
}

static int windows(int fid, float airS, win_t *w, int max)
{
  int last = -1;
  for (int i = 0; i < nRows; i++) {
    if (rows[i][C_FID] == fid && rows[i][C_THRUST] > 30000) {
      last = i;
    }
  }
  if (last < 0) {
    return 0;
  }
  const long tickEnd = rows[last][C_TICK];
  int n = 0;
  for (int i = 0; i < nRows && n < max;) {
    if (!steadyRow(i, fid)) {
      i++;
      continue;
    }
    int j = i;
    while (j < nRows && steadyRow(j, fid)) {
      j++;
    }
    /* [i, j): one steady segment; skip 0.5 s at its start and 0.3 s at its end */
    const int a = i + 50, b = j - 30;
    if (b - a >= 300) {
      int chunks = (int)lroundf((float)(b - a) / 450.0f);
      if (chunks < 1) {
        chunks = 1;
      }
      while (chunks > 1 && (b - a) / chunks < 300) {
        chunks--;
      }
      for (int c = 0; c < chunks && n < max; c++) {
        const int k0 = a + (b - a) * c / chunks, k1 = a + (b - a) * (c + 1) / chunks;
        double sx = 0.0, sy = 0.0;
        for (int k = k0; k < k1; k++) {
          sx += rows[k][C_GX] / 10.0;
          sy += rows[k][C_GY] / 10.0;
        }
        const double T = (rows[k1 - 1][C_TICK] - rows[k0][C_TICK]) * 1e-3;
        const double mx = sx / (k1 - k0), my = sy / (k1 - k0);
        const double dRoll = (rows[k1 - 1][C_ROLL] - rows[k0][C_ROLL]) / 100.0;
        const double dPitch = (rows[k1 - 1][C_PITCH] - rows[k0][C_PITCH]) / 100.0;
        w[n].gx = (float)mx;
        w[n].gy = (float)my;
        w[n].bx = (float)(mx - dRoll / T);
        w[n].by = (float)(my + dPitch / T);
        w[n].tMid = airS - (float)(tickEnd - (rows[k0][C_TICK] + rows[k1 - 1][C_TICK]) / 2) * 1e-3f;
        n++;
      }
    }
    i = j;
  }
  return n;
}

static float modelBias(const gyroTcConfig_t *c, int axis, float t, float tStart, float tEnd, float tau)
{
  const float T = tEnd + (tStart - tEnd) * expf(-t / tau);
  const float v = c->k[axis] * gyroTcEffDelta(c, T, tStart);
  return fminf(fmaxf(v, -c->clampDps), c->clampDps);
}

/* The driver's re-zero bookkeeping around gyroTcLogRezero (sensors_mpu6050_hm5883L_ms5611.c: the flags since the
 * latest latch, set by sensorsMpu6050SetThrust() at 1 kHz and by 'gtc hold', cleared at every latch). */
typedef struct {
  gyroTcLog_t log;
  bool thrust, air, hold;
  uint32_t airFirst, airLast;
} drvModel_t;

static void drvThrust(drvModel_t *d, uint32_t ms, float thrust)
{
  if (thrust > 0.0f) {
    d->thrust = true;
    if (thrust > 30000.0f) {
      if (!d->air) {
        d->airFirst = ms;
        d->air = true;
      }
      d->airLast = ms;
    }
  }
}

static bool drvRezero(drvModel_t *d, uint32_t ms, float tRef, float tNow, float stepX, float stepY, float corrX,
                      float corrY)
{
  const float step[3] = {stepX, stepY, 0.0f}, corr[3] = {corrX, corrY, 0.0f};
  const uint8_t flags = (d->thrust ? GYRO_TC_RZ_FLIGHT : 0) | GYRO_TC_RZ_ON | (d->air ? GYRO_TC_RZ_AIR : 0) |
                        (d->hold ? GYRO_TC_RZ_HOLD : 0);
  gyroTcRezero_t e;
  gyroTcRezeroFill(&e, tRef, tNow, step, corr, flags);
  const bool logged = gyroTcLogRezero(&d->log, &e, ms, d->airFirst, d->airLast);
  d->thrust = d->air = d->hold = false;
  return logged;
}

/* docs/autonomy-a2b.md 19.1: the first re-zero after a flight must still be readable when USB is plugged in,
 * minutes after landing, with a quiet re-zero once a second in between. */
static void rezeroLogTests(void)
{
  drvModel_t d;
  memset(&d, 0, sizeof(d));
  gyroTcLogInit(&d.log);
  CHECK(gyroTcLogCount(&d.log) == 0 && d.log.airSeq == 0, "empty log");

  /* fill / residual / saturation */
  {
    gyroTcRezero_t e;
    const float step[3] = {-0.150f, 0.650f, 0.004f}, corr[3] = {-0.180f, 0.600f, 0.0f};
    gyroTcRezeroFill(&e, 43.004f, 33.2f, step, corr, GYRO_TC_RZ_AIR);
    CHECK(e.tRefOldCc == 4300 && e.tNowCc == 3320 && e.stepMdps[1] == 650 && e.corrMdps[0] == -180 &&
          e.flags == GYRO_TC_RZ_AIR, "fill: %d %d %d %d", e.tRefOldCc, e.tNowCc, e.stepMdps[1], e.corrMdps[0]);
    CHECK(near(gyroTcRezeroResidual(&e, 1), 0.050f, 1e-6f) && near(gyroTcRezeroResidual(&e, 0), 0.030f, 1e-6f),
          "r = step - corr: %+.3f %+.3f", gyroTcRezeroResidual(&e, 0), gyroTcRezeroResidual(&e, 1));
    const float big[3] = {40.0f, -40.0f, NAN};
    gyroTcRezeroFill(&e, NAN, 400.0f, big, big, 0);
    CHECK(e.stepMdps[0] == 32767 && e.stepMdps[1] == -32767 && e.stepMdps[2] == 0 && e.tRefOldCc == 0 &&
          e.tNowCc == 32767, "saturation and NaN");
    CHECK(near(gyroTcRezeroResidual(&e, 0), 0.0f, 1e-6f) && near(gyroTcRezeroResidual(&e, 1), 0.0f, 1e-6f),
          "r of saturated values in int32");
  }

  /* 20 s on the ground warming: quiet re-zeros once a second are not logged */
  uint32_t ms = 10000;
  int logged = 0;
  for (int i = 0; i < 20; i++, ms += 1000) {
    logged += drvRezero(&d, ms, 42.0f + 0.05f * i, 42.05f + 0.05f * i, 0.001f, -0.003f, -0.001f, 0.003f);
  }
  CHECK(logged == 0 && gyroTcLogCount(&d.log) == 0 && d.log.n == 0, "quiet ground re-zeros: none logged (%d)",
        logged);

  /* flight 4: 48 s above lift-off (thrust 0 -> 45000 over 1 s), IMU 43 -> 33 C; the first re-zero 2.5 s after the
   * motors stop */
  const uint32_t t0 = ms;
  for (uint32_t t = 0; t < 49000; t++) {
    drvThrust(&d, t0 + t, t < 1000 ? 45.0f * t : 45000.0f);
  }
  ms = t0 + 49000 + 2500;
  CHECK(drvRezero(&d, ms, 43.0f, 33.0f, -0.150f, 0.650f, -0.180f, 0.600f), "the post-flight re-zero is logged");
  /* above 30000 from t = 667 ms (the ramp) to the last sample at 48999 ms */
  CHECK(d.log.airSeq == 1 && d.log.airLandMs == 2501 && d.log.airSpanMs == 48332,
        "air slot: seq %u land %u ms span %u ms", (unsigned)d.log.airSeq, (unsigned)d.log.airLandMs,
        (unsigned)d.log.airSpanMs);
  const gyroTcRezero_t flightEntry = d.log.air;
  CHECK((flightEntry.flags & (GYRO_TC_RZ_AIR | GYRO_TC_RZ_FLIGHT)) == (GYRO_TC_RZ_AIR | GYRO_TC_RZ_FLIGHT),
        "flight and air flags");

  /* 10 min still on the bench afterwards (600 quiet re-zeros, ~10 s of them is what pushed it out before) */
  for (int i = 0; i < 600; i++) {
    ms += 1000;
    drvRezero(&d, ms, 33.0f + 0.01f * i, 33.01f + 0.01f * i, 0.0005f, -0.004f, -0.0002f, 0.0006f);
  }
  CHECK(gyroTcLogCount(&d.log) == 1 && d.log.n == 1, "still one entry after 600 quiet re-zeros (%u)",
        (unsigned)d.log.n);
  const gyroTcRezero_t *z = gyroTcLogEntry(&d.log, 0);
  CHECK(memcmp(z, &flightEntry, sizeof(*z)) == 0 && near(gyroTcRezeroResidual(z, 1), 0.050f, 1e-6f),
        "the flight's entry is the ring's newest: r_y %+.3f", gyroTcRezeroResidual(z, 1));
  CHECK(memcmp(&d.log.air, &flightEntry, sizeof(flightEntry)) == 0 && d.log.airSeq == 1, "air slot unchanged");
  CHECK(ms - d.log.airAtMs == 600000u, "its age: 600 s");

  /* a large step on a still drone (a handling-free zero jump) is news; 49 mdps is not */
  CHECK(!drvRezero(&d, ms += 1000, 39.0f, 39.0f, 0.0f, 0.049f, 0.0f, 0.0f), "a 49 mdps step: not logged");
  CHECK(drvRezero(&d, ms += 1000, 39.0f, 39.0f, 0.0f, -0.050f, 0.0f, 0.0f), "a 50 mdps step: logged");
  CHECK(drvRezero(&d, ms += 1000, 39.0f, 39.0f, 0.060f, 0.0f, 0.0f, 0.0f), "a 60 mdps step on x: logged");

  /* the bench cooling test: the first re-zero after a 'gtc hold' is logged with hold=1 */
  d.hold = true;
  CHECK(drvRezero(&d, ms += 240000, 44.0f, 37.0f, 0.0f, 0.400f, 0.0f, 0.420f), "after a hold: logged");
  CHECK(gyroTcLogEntry(&d.log, gyroTcLogCount(&d.log) - 1)->flags & GYRO_TC_RZ_HOLD, "hold flag");
  CHECK(!drvRezero(&d, ms += 1000, 37.0f, 37.0f, 0.0f, 0.002f, 0.0f, 0.0f), "the next one is quiet again");

  /* nine ground spins (thrust below lift-off: SPIN aborts, BENCH): logged (flight=1, air=0), the ring wraps, the
   * air slot keeps the flight */
  for (int i = 0; i < 9; i++) {
    for (uint32_t t = 0; t < 1500; t++) {
      drvThrust(&d, ms + t, 12000.0f);
    }
    ms += 1500 + 2500;
    CHECK(drvRezero(&d, ms, 37.0f, 37.1f, 0.0f, 0.003f, 0.0f, 0.0f), "ground spin %d logged", i);
  }
  CHECK(d.log.n == 13 && gyroTcLogCount(&d.log) == GYRO_TC_LOG_RING, "ring full: n %u", (unsigned)d.log.n);
  for (uint32_t i = 0; i < GYRO_TC_LOG_RING; i++) {
    const gyroTcRezero_t *g = gyroTcLogEntry(&d.log, i);
    CHECK((g->flags & (GYRO_TC_RZ_FLIGHT | GYRO_TC_RZ_AIR)) == GYRO_TC_RZ_FLIGHT, "ring entry %u: a ground spin",
          (unsigned)i);
  }
  CHECK(memcmp(&d.log.air, &flightEntry, sizeof(flightEntry)) == 0 && d.log.airSeq == 1,
        "the air slot still holds the flight after the ring wrapped");

  /* the next flight replaces it; the uptime clock wrapping in between is harmless */
  ms = UINT32_MAX - 3000u;
  for (uint32_t t = 0; t < 6000; t++) {
    drvThrust(&d, ms + t, 40000.0f);
  }
  ms += 6000 + 2200;
  CHECK(drvRezero(&d, ms, 40.0f, 38.0f, 0.0f, 0.130f, 0.0f, 0.120f), "the next flight's re-zero");
  CHECK(d.log.airSeq == 2 && d.log.airLandMs == 2201 && d.log.airSpanMs == 5999 && d.log.air.stepMdps[1] == 130,
        "the next flight replaces the air slot across the clock wrap: land %u span %u", (unsigned)d.log.airLandMs,
        (unsigned)d.log.airSpanMs);
  printf("re-zero log: the flight's r_y +0.050 dps readable after 600 quiet re-zeros and 9 ground spins\n");
}

/* The driver's update rate (docs/autonomy-a2b.md 19.11). Both pipelines of sensors_mpu6050_hm5883L_ms5611.c on the
 * same 1 kHz raw TEMP_OUT stream, the ground re-zeros (latches) at the same samples:
 *   per sample (b40a827..0ed68c9): a latch first (gyroRezeroCheck), then gyroTcStep(raw / 340 + 36.53, 1 ms) and
 *     gyroTcCorr; subtracted when on and latched;
 *   per block (this build): a latch recomputes the correction from the new reference; the raw value is added to
 *     the block's sum and every GYRO_TC_BLOCK samples gyroTcBlock runs on their mean (dt = the block); the cached
 *     correction is subtracted from every sample.
 * The difference of what each subtracts, sample by sample, and of the model each logs at a re-zero. */
typedef struct {
  float tempC;       /* the die temperature (truth) */
  bool latch;        /* an accepted ground re-zero at this sample */
} tcSample_t;

typedef struct {
  float maxDiff[3];       /* max |subtracted (per block) - subtracted (per sample)|, dps */
  float maxLogDiff[3];    /* the model logged at a re-zero */
  float maxRefDiff;       /* |Tref (per block) - Tref (per sample)| at the latches, C */
  float maxAbsCorr;       /* max |subtracted (per sample)| on y: the size of the correction itself */
  bool offPlusZero;       /* off: every block-pipeline correction is +0.0f */
  int latches;
} tcCompare_t;

typedef float (*tcTempFn)(long ms, bool *latch);

static uint32_t lcg = 12345u;
static int noiseLsb(void)   /* deterministic, -3..+3 LSB (the MPU-6050's TEMP_OUT noise is ~1-2 LSB) */
{
  lcg = lcg * 1664525u + 1013904223u;
  return (int)((lcg >> 16) % 7u) - 3;
}

static int16_t rawOf(float tempC)
{
  return (int16_t)lrintf((tempC - 36.53f) * 340.0f) + (int16_t)noiseLsb();
}

static void tcCompare(const gyroTcConfig_t *c, tcTempFn fn, long samples, tcCompare_t *out)
{
  gyroTcState_t a, b;   /* a: per sample; b: per block */
  gyroTcInit(&a);
  gyroTcInit(&b);
  float corrB[3] = {0.0f, 0.0f, 0.0f};
  int32_t sum = 0;
  int n = 0;
  memset(out, 0, sizeof(*out));
  out->offPlusZero = true;
  lcg = 12345u;
  for (long i = 0; i < samples; i++) {
    bool latch = false;
    const float tempC = fn(i, &latch);
    const int16_t raw = rawOf(tempC);
    if (latch) {
      float ma[3], mb[3];
      gyroTcModel(c, &a, ma);
      gyroTcModel(c, &b, mb);
      if (a.refSet && b.refSet) {
        for (int k = 0; k < 3; k++) {
          out->maxLogDiff[k] = fmaxf(out->maxLogDiff[k], fabsf(mb[k] - ma[k]));
        }
      }
      gyroTcLatch(&a);
      gyroTcLatch(&b);
      gyroTcCorr(c, &b, corrB);
      if (a.refSet && b.refSet) {
        out->maxRefDiff = fmaxf(out->maxRefDiff, fabsf(b.tRef - a.tRef));
        out->latches++;
      }
    }
    /* per sample */
    float corrA[3];
    gyroTcStep(&a, (float)raw / 340.0f + 36.53f, 0.001f);
    gyroTcCorr(c, &a, corrA);
    if (!(c->on && a.refSet)) {
      corrA[0] = corrA[1] = corrA[2] = 0.0f;   /* nothing subtracted */
    }
    /* per block */
    sum += raw;
    if (++n >= GYRO_TC_BLOCK) {
      gyroTcBlock(&b, c, sum, n, corrB);
      sum = 0;
      n = 0;
    }
    for (int k = 0; k < 3; k++) {
      out->maxDiff[k] = fmaxf(out->maxDiff[k], fabsf(corrB[k] - corrA[k]));
      if (!c->on && (corrB[k] != 0.0f || signbit(corrB[k]))) {
        out->offPlusZero = false;
      }
    }
    out->maxAbsCorr = fmaxf(out->maxAbsCorr, fabsf(corrA[1]));
  }
}

/* Flight 4 (2026-10-14): 60 s on the ground at 43 C (startup bias at 1.2 s, a re-zero every second from 3 s), 50 s
 * of flight cooling toward 33 C (tau 18 s, no re-zero), then the ground: the first re-zero 2.5 s after landing and
 * one a second for 60 s while it keeps cooling slowly. */
static float tempFlight(long ms, bool *latch)
{
  const float t = ms / 1000.0f;
  if (t < 60.0f) {
    *latch = ms == 1200 || (ms >= 3000 && ms % 1000 == 0);
    return 43.0f;
  }
  if (t < 110.0f) {
    return 33.0f + 10.0f * expf(-(t - 60.0f) / 18.0f);
  }
  const float tLand = 33.0f + 10.0f * expf(-50.0f / 18.0f);
  *latch = ms >= 112500 && (ms - 112500) % 1000 == 0;
  return tLand - 0.02f * (t - 110.0f);
}

/* The bench cooling test (autonomy-bench.md I2): 'gtc hold 240' at 45 C, the camera standby cools it ~0.04 C/s,
 * the first re-zero after the hold. */
static float tempHold(long ms, bool *latch)
{
  *latch = ms == 1500 || ms == 245000;
  return ms < 5000 ? 45.0f : 45.0f - 0.04f * (ms - 5000) / 1000.0f;
}

/* A 1 C step 3 s after the latch: not physical (the die warms or cools at <= ~0.2 C/s), the bound on any change. */
static float tempStep(long ms, bool *latch)
{
  *latch = ms == 1500;
  return ms < 3000 ? 40.0f : 39.0f;
}

static void blockRateTests(void)
{
  gyroTcConfig_t c;
  gyroTcDefaultConfig(&c);

  /* the decode and one sample: n = 1 is the per-sample step bit for bit */
  {
    bool same = true;
    for (int raw = -32768; raw <= 32767; raw += 7) {
      const float t1 = gyroTcTempFromRawSum(raw, 1), t0 = (float)raw / 340.0f + 36.53f;
      same = same && memcmp(&t1, &t0, sizeof(t1)) == 0;
    }
    CHECK(same, "gyroTcTempFromRawSum(raw, 1) == raw / 340 + 36.53 bit for bit");
    gyroTcState_t a, b;
    stateAt(&a, 43.0f, 38.0f);
    b = a;
    float ca[3], cb[3];
    bool sameStep = true;
    for (int i = 0; i < 5000; i++) {
      const int16_t raw = (int16_t)(-1500 + (i % 97) * 3);
      gyroTcStep(&a, (float)raw / 340.0f + 36.53f, 0.001f);
      gyroTcCorr(&c, &a, ca);
      gyroTcBlock(&b, &c, raw, 1, cb);
      sameStep = sameStep && memcmp(&a, &b, sizeof(a)) == 0 && memcmp(ca, cb, sizeof(ca)) == 0;
    }
    CHECK(sameStep, "gyroTcBlock with n = 1 is gyroTcStep + gyroTcCorr bit for bit");
    CHECK(near(gyroTcTempFromRawSum(10 * -2000, 10), -2000.0f / 340.0f + 36.53f, 1e-5f), "a block's mean");
    gyroTcBlock(&b, &c, 0, 0, cb);
    CHECK(memcmp(&a, &b, sizeof(a)) == 0, "an empty block does not step");
  }

  tcCompare_t r;
  tcCompare(&c, tempFlight, 172000, &r);
  printf("flight 4 profile, per block (%d) vs per sample: subtracted max |diff| x %.6f y %.6f z %.6f dps "
         "(correction up to %.3f), logged model %.6f %.6f, Tref %.4f C over %d latches\n", GYRO_TC_BLOCK,
         r.maxDiff[0], r.maxDiff[1], r.maxDiff[2], r.maxAbsCorr, r.maxLogDiff[0], r.maxLogDiff[1], r.maxRefDiff,
         r.latches);
  /* ~0.044 mdps per sample of block on flight 4's fastest cooling (the correction held for a block, the block's mean
   * centred half a block late); never more than 5 mdps (1/12 of a gyro LSB, 1/24 of the model's +-20 %) */
  const float blockTol = 0.00005f * GYRO_TC_BLOCK;
  CHECK(r.maxDiff[0] < blockTol && r.maxDiff[1] < blockTol && r.maxDiff[2] == 0.0f && r.maxDiff[1] < 0.005f,
        "flight: the block rate changes the correction by < %.2f mdps (%.6f %.6f)", 1000.0f * blockTol, r.maxDiff[0],
        r.maxDiff[1]);
  CHECK(r.maxLogDiff[0] < 0.0005f && r.maxLogDiff[1] < 0.0005f, "flight: the re-zero log's model within 0.5 mdps");
  CHECK(r.maxRefDiff < 0.01f && r.latches > 100, "flight: Tref within 0.01 C (%.4f)", r.maxRefDiff);
  CHECK(r.maxAbsCorr > 0.5f, "flight: the correction itself is ~0.6 dps (%.3f)", r.maxAbsCorr);

  tcCompare(&c, tempHold, 250000, &r);
  printf("bench cooling (hold 240 s): max |diff| x %.6f y %.6f dps, logged model %.6f %.6f, correction up to %.3f\n",
         r.maxDiff[0], r.maxDiff[1], r.maxLogDiff[0], r.maxLogDiff[1], r.maxAbsCorr);
  CHECK(r.maxDiff[0] < 0.000005f * GYRO_TC_BLOCK && r.maxDiff[1] < 0.000005f * GYRO_TC_BLOCK &&
        r.maxLogDiff[1] < 0.0005f, "the bench cooling test: within %.2f mdps", 0.005 * GYRO_TC_BLOCK);

  tcCompare(&c, tempStep, 8000, &r);
  printf("a 1 C step (bound): max |diff| x %.6f y %.6f dps\n", r.maxDiff[0], r.maxDiff[1]);
  CHECK(r.maxDiff[1] < 0.00012f * GYRO_TC_BLOCK && r.maxDiff[0] < 0.00004f * GYRO_TC_BLOCK,
        "a 1 C step: within %.1f mdps (%.6f)", 0.12 * GYRO_TC_BLOCK, r.maxDiff[1]);

  c.on = false;
  tcCompare(&c, tempFlight, 172000, &r);
  CHECK(r.offPlusZero && r.maxDiff[0] == 0.0f && r.maxDiff[1] == 0.0f,
        "off: the block pipeline subtracts +0.0f on every sample (x - (+0) is x bit for bit)");
  printf("block rate: %d samples per update (%d Hz)\n", GYRO_TC_BLOCK, 1000 / GYRO_TC_BLOCK);
}

static void replay(const char *dir)
{
  DIR *d = opendir(dir);
  if (d == NULL) {
    printf("replay skipped: pass the directory with flight*.csv and flights*.txt (../../firmware-backups/cal)\n");
    return;
  }
  gyroTcConfig_t c;
  gyroTcDefaultConfig(&c);
  struct dirent *e;
  int flights = 0;
  while ((e = readdir(d)) != NULL) {
    const char *nm = e->d_name;
    if (strncmp(nm, "flights-2026-10-1", 17) != 0 || (nm[17] < '2' || nm[17] > '4') ||
        strstr(nm, ".txt") == NULL) {
      continue;
    }
    char sp[1024], cp[1024];
    snprintf(sp, sizeof(sp), "%s/%s", dir, nm);
    snprintf(cp, sizeof(cp), "%s/flight-%s", dir, nm + strlen("flights-"));
    char *dot = strstr(cp, ".txt");
    if (dot == NULL) {
      continue;
    }
    strcpy(dot, ".csv");
    sumFlight_t sf[64];
    const int ns = loadSummary(sp, sf, 64);
    int maxBoot = -1;
    for (int i = 0; i < ns; i++) {
      maxBoot = sf[i].boot > maxBoot ? sf[i].boot : maxBoot;
    }
    bool loaded = false;
    for (int i = 0; i < ns; i++) {
      if (!isfinite(sf[i].t0) || sf[i].boot != maxBoot || sf[i].t0 - sf[i].t1 < 3.0f || sf[i].airS < 20.0f) {
        continue;   /* the recorder holds the latest boot only; a flight that cooled by less says nothing */
      }
      if (!loaded && !(loaded = loadCsv(cp))) {
        break;
      }
      win_t w[64];
      const int nw = windows(sf[i].id, sf[i].airS, w, 64);
      if (nw < 3) {
        printf("%s flight %d (%.1f -> %.1f C): %d steady windows in the recorder, not replayed\n", nm, sf[i].id,
               sf[i].t0, sf[i].t1, nw);
        continue;
      }
      flights++;
      /* tau on the y axis */
      float bestTau = 10.0f, bestRms = INFINITY;
      for (float tau = 10.0f; tau <= 40.0f; tau += 1.0f) {
        double s = 0.0;
        for (int k = 0; k < nw; k++) {
          const float r = w[k].by - modelBias(&c, 1, w[k].tMid, sf[i].t0, sf[i].t1, tau);
          s += r * r;
        }
        if (sqrt(s / nw) < bestRms) {
          bestRms = (float)sqrt(s / nw);
          bestTau = tau;
        }
      }
      printf("%s flight %d: %.1f -> %.1f C over %.1f s of air, tau %.0f s (fit on y), defaults k %+.3f/%+.3f "
             "knee %.0f:\n", nm, sf[i].id, sf[i].t0, sf[i].t1, sf[i].airS, bestTau, c.k[0], c.k[1], c.kneeC);
      double sx = 0.0, sy = 0.0, maxY = 0.0, maxX = 0.0;
      for (int k = 0; k < nw; k++) {
        const float my = modelBias(&c, 1, w[k].tMid, sf[i].t0, sf[i].t1, bestTau);
        const float mx = modelBias(&c, 0, w[k].tMid, sf[i].t0, sf[i].t1, bestTau);
        printf("  t %5.1f s: gy %+.2f (rate-corrected %+.2f) model %+.2f r %+.2f | gx %+.2f (%+.2f) model %+.2f r %+.2f\n",
               w[k].tMid, w[k].gy, w[k].by, my, w[k].by - my, w[k].gx, w[k].bx, mx, w[k].bx - mx);
        sy += (w[k].by - my) * (w[k].by - my);
        sx += (w[k].bx - mx) * (w[k].bx - mx);
        maxY = fmax(maxY, fabs(w[k].by - my));
        maxX = fmax(maxX, fabs(w[k].bx - mx));
      }
      const double rmsY = sqrt(sy / nw), rmsX = sqrt(sx / nw);
      printf("  residual rms y %.3f (max %.3f), x %.3f (max %.3f) dps\n", rmsY, maxY, rmsX, maxX);
      CHECK(bestTau >= 12.0f && bestTau <= 30.0f, "tau %.0f s outside 12..30 (the forensic's ~20 s)", bestTau);
      CHECK(rmsY <= 0.15 && rmsX <= 0.15, "model vs the logged gyro zero: rms y %.3f x %.3f > 0.15 dps", rmsY, rmsX);
      CHECK(maxY <= 0.20 && maxX <= 0.20, "the worst window: y %.3f x %.3f > 0.20 dps", maxY, maxX);
      /* without the compensation the zero reaches > 0.45 dps: the model explains most of it */
      double s0 = 0.0;
      for (int k = 0; k < nw; k++) {
        s0 += w[k].by * w[k].by;
      }
      printf("  uncompensated rms y %.3f dps -> %.3f with the model\n", sqrt(s0 / nw), rmsY);
      CHECK(rmsY < 0.5 * sqrt(s0 / nw), "the model removes at least half of the y zero");
    }
  }
  closedir(d);
  printf("replayed %d flight(s)\n", flights);
  CHECK(flights >= 1, "at least the 2026-10-14 morning flight 4 replayed");
}

int main(int argc, char **argv)
{
  unitTests();
  rezeroLogTests();
  blockRateTests();
  if (argc >= 2) {
    replay(argv[1]);
  } else {
    printf("replay skipped: pass the directory with flight*.csv and flights*.txt (../../firmware-backups/cal)\n");
  }
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
