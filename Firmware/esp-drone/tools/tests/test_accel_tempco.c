/*
 * Host test: the accelerometer temperature compensation (components/core/crazyflie/utils/src/accel_tempco.c,
 * docs/autonomy-a2b.md section 36) and a replay of its check against the logged flights' ground rows.
 *
 * Build and run from Firmware/esp-drone (the replay is skipped without the CSV directory):
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface tools/tests/test_accel_tempco.c \
 *      components/core/crazyflie/utils/src/accel_tempco.c components/core/crazyflie/utils/src/imu_level_cal.c \
 *      -lm -o /tmp/test_accel_tempco && /tmp/test_accel_tempco ../../firmware-backups/cal
 *
 * Unit checks: the defaults (OFF, k_z +0.20 mg/C, no knee, clamp 15 mg), sign and magnitude (47.7 -> 31.0 C, the
 * 2026-10-17 15:26 flight: z -3.34 mg when on, exactly 0 when off so the driver's output is bit-identical), the
 * knee, the clamp, the latch and the low-pass, NaN, the block step bit for bit with the per-sample one, the window
 * of block means (full after ACC_TC_WIN_BLOCKS blocks, the mean of the raw sums in mg), the log (an airborne
 * entry stays through quiet re-zeros and ground spins; a cooling spin's entry is kept; a quiet one is not), the
 * NVS record (round trip, a flipped bit, a wrong size, a value out of range).
 *
 * Replay: the still ground rows before the first thrust and after the last thrust of the flights that rest on the
 * same spot without an attitude change (2026-10-15 p1-height flight 4, 2026-10-17 table flight 8: 12-16 C of
 * cooling): the step a still accelerometer took, as the driver's check measures it (1 s means, raw), is within
 * 10 mg on z, and the default model's residual r = step - corr too.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "accel_tempco.h"

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
static void stateAt(accTcState_t *st, float tRef, float t)
{
  accTcInit(st);
  for (int i = 0; i < 10000; i++) {
    accTcStep(st, tRef, 0.001f);
  }
  accTcLatch(st);
  for (int i = 0; i < 10000; i++) {
    accTcStep(st, t, 0.001f);
  }
}

static void unitTests(void)
{
  accTcConfig_t c;
  accTcDefaultConfig(&c);
  CHECK(!c.on && c.k[0] == 0.0f && c.k[1] == 0.0f && c.k[2] == 0.20f && c.kneeC == 60.0f && c.clampMg == 15.0f,
        "defaults");
  CHECK(accTcConfigValid(&c), "defaults valid");

  accTcState_t st;
  float corr[3], model[3];

  /* off: exactly 0 (the driver subtracts +0.0f: bit-identical), the model still runs */
  stateAt(&st, 47.7f, 31.0f);
  accTcCorr(&c, &st, corr);
  CHECK(corr[0] == 0.0f && corr[1] == 0.0f && corr[2] == 0.0f && !signbit(corr[2]), "off: +0.0");
  accTcModel(&c, &st, model);
  printf("47.7 -> 31.0 C (the 15:26 flight): model z %+.3f mg\n", model[2]);
  CHECK(near(model[2], -3.34f, 0.01f) && model[0] == 0.0f && model[1] == 0.0f, "model z -3.34 mg, got %+.3f",
        model[2]);

  /* on: the sign. A zero that followed b_z = +0.20 (T - Tref) reads -3.34 mg below its reference after the
   * cooling; corrected, 0. The vertical controller would have read the uncorrected one as sinking. */
  c.on = true;
  accTcCorr(&c, &st, corr);
  CHECK(near(corr[2], -3.34f, 0.01f), "on: corr z %+.3f", corr[2]);
  {
    const float b = 0.20f * (31.0f - 47.7f);
    CHECK(near(b - corr[2], 0.0f, 0.01f), "corrected accelerometer ~0: %+.4f", b - corr[2]);
    CHECK(fabsf(b + corr[2]) > 6.0f, "a sign error doubles the drift");
  }

  /* the knee */
  c.kneeC = 44.0f;
  stateAt(&st, 47.0f, 44.0f);
  accTcCorr(&c, &st, corr);
  CHECK(corr[2] == 0.0f, "47->44 C with the knee at 44: 0, got %+.4f", corr[2]);
  stateAt(&st, 47.0f, 34.0f);
  accTcCorr(&c, &st, corr);
  CHECK(near(corr[2], -2.0f, 0.01f), "47->34 C with the knee at 44: z -2.0, got %+.4f", corr[2]);
  CHECK(accTcEffDelta(&c, 34.0f, 47.0f) == -10.0f, "effective delta");
  c.kneeC = 60.0f;

  /* the clamp */
  c.k[2] = 2.0f;
  stateAt(&st, 50.0f, 30.0f);
  accTcCorr(&c, &st, corr);
  CHECK(corr[2] == -15.0f, "clamp 15 mg: %+.4f", corr[2]);
  stateAt(&st, 30.0f, 50.0f);
  accTcCorr(&c, &st, corr);
  CHECK(corr[2] == 15.0f, "clamp both signs: %+.4f", corr[2]);
  c.k[2] = 0.20f;

  /* the latch and the low-pass */
  accTcInit(&st);
  accTcCorr(&c, &st, corr);
  CHECK(corr[2] == 0.0f, "unseeded: 0");
  accTcLatch(&st);
  CHECK(!st.refSet, "a latch before the first sample is a no-op");
  accTcStep(&st, 45.0f, 0.001f);
  CHECK(st.seeded && st.tLp == 45.0f, "seeded by the first sample");
  accTcStep(&st, NAN, 0.001f);
  CHECK(st.tLp == 45.0f, "NaN skipped");
  accTcCorr(&c, &st, corr);
  CHECK(corr[2] == 0.0f, "seeded but not latched: 0");
  accTcLatch(&st);
  CHECK(st.refSet && st.tRef == 45.0f, "latched at 45");
  for (int i = 0; i < 500; i++) {
    accTcStep(&st, 35.0f, 0.001f);
  }
  CHECK(near(st.tLp, 45.0f - 10.0f * (1.0f - expf(-1.0f)), 0.1f), "tau 0.5 s: after 0.5 s tLp %.3f", st.tLp);
  accTcStep(&st, 35.0f, 0.0f);
  accTcStep(&st, 35.0f, -1.0f);
  CHECK(near(st.tLp, 45.0f - 10.0f * (1.0f - expf(-1.0f)), 0.1f), "dt <= 0 skipped");

  /* the block step: one sample a block is the per-sample step bit for bit */
  {
    accTcState_t a, b;
    accTcInit(&a);
    accTcInit(&b);
    float ca[3], cb[3];
    for (int i = 0; i < 3000; i++) {
      const int32_t raw = (int32_t)((40.0f - 10.0f * i / 3000.0f - 36.53f) * 340.0f);
      accTcStep(&a, (float)raw / 340.0f + 36.53f, 0.001f);
      if (i == 100) {
        accTcLatch(&a);
        accTcLatch(&b);
      }
      accTcBlock(&b, &c, raw, 1, cb);
      if (i == 100) {
        accTcLatch(&b);   /* the latch after the block's step, as the driver orders it */
        accTcLatch(&a);
      }
    }
    accTcCorr(&c, &a, ca);
    CHECK(a.tLp == b.tLp && ca[2] == cb[2], "block of 1 == per sample: %.6f %.6f", a.tLp, b.tLp);
    /* 50-sample blocks against the per-sample path on the fastest logged cooling (0.6 C/s, 3 s): within 0.01 mg
     * (the block's mean lags the per-sample filter by half a block, 25 ms x 0.6 C/s x 0.2 mg/C = 0.003 mg) */
    accTcState_t a6, d;
    accTcInit(&a6);
    accTcInit(&d);
    int32_t sum = 0;
    int n = 0;
    float ca6[3], cd[3] = {0, 0, 0};
    for (int i = 0; i < 3000; i++) {
      const int32_t raw = (int32_t)((40.0f - 1.8f * i / 3000.0f - 36.53f) * 340.0f);
      accTcStep(&a6, (float)raw / 340.0f + 36.53f, 0.001f);
      sum += raw;
      if (++n == ACC_TC_BLOCK) {
        accTcBlock(&d, &c, sum, n, cd);
        sum = 0;
        n = 0;
      }
      if (i == 99) {
        accTcLatch(&d);
        accTcLatch(&a6);
      }
    }
    accTcCorr(&c, &a6, ca6);
    CHECK(near(cd[2], ca6[2], 0.01f), "50-sample blocks within 0.01 mg of per-sample: %+.4f vs %+.4f", cd[2], ca6[2]);
    /* 1.8 C of cooling less the low-pass lag (tau x rate = 0.3 C), from the latch at 0.1 s: -1.47 C x 0.2 */
    CHECK(near(cd[2], -0.29f, 0.02f), "1.8 C of cooling: z -0.29 mg (%+.4f)", cd[2]);
  }

  /* the window of block means */
  {
    accTcWindow_t w;
    accTcWindowInit(&w);
    float m[3];
    CHECK(!accTcWindowMean(&w, m) && m[2] == 0.0f, "empty window: no mean");
    const float mgPerLsb = 1000.0f / 2048.0f;   /* +-16 g full scale */
    for (int b = 0; b < ACC_TC_WIN_BLOCKS - 1; b++) {
      const int32_t s[3] = {0, 0, 2048 * 50};
      accTcWindowPush(&w, s, 50, mgPerLsb);
    }
    CHECK(!accTcWindowMean(&w, m), "19 blocks: not full");
    const int32_t s[3] = {50 * 20, -50 * 20, 2048 * 50 + 50 * 41};
    accTcWindowPush(&w, s, 50, mgPerLsb);
    CHECK(accTcWindowMean(&w, m), "20 blocks: full");
    CHECK(near(m[2], 1000.0f + 41.0f * mgPerLsb / 20.0f, 0.01f) && near(m[0], 20.0f * mgPerLsb / 20.0f, 0.001f) &&
          near(m[1], -20.0f * mgPerLsb / 20.0f, 0.001f), "window mean %.3f %.3f %.3f", m[0], m[1], m[2]);
    for (int b = 0; b < ACC_TC_WIN_BLOCKS; b++) {
      const int32_t t[3] = {0, 0, 2048 * 50};
      accTcWindowPush(&w, t, 50, mgPerLsb);
    }
    accTcWindowMean(&w, m);
    CHECK(near(m[2], 1000.0f, 0.001f), "the ring wraps: the old blocks are gone (%.3f)", m[2]);
    const int32_t z[3] = {0, 0, 0};
    accTcWindowPush(&w, z, 0, mgPerLsb);
    accTcWindowMean(&w, m);
    CHECK(near(m[2], 1000.0f, 0.001f), "an empty block is ignored");
  }

  /* the log */
  {
    accTcLog_t log;
    accTcLogInit(&log);
    accTcRezero_t e;
    const float step[3] = {1.0f, -2.0f, -5.5f}, corrM[3] = {0.0f, 0.0f, -3.3f};
    accTcRezeroFill(&e, 47.7f, 31.0f, step, corrM, ACC_TC_RZ_FLIGHT | ACC_TC_RZ_AIR | ACC_TC_RZ_ON);
    CHECK(e.tRefCc == 4770 && e.tNowCc == 3100 && e.stepCmg[2] == -550 && e.corrCmg[2] == -330,
          "fill: %d %d %d %d", e.tRefCc, e.tNowCc, e.stepCmg[2], e.corrCmg[2]);
    CHECK(near(accTcRezeroResidual(&e, 2), -2.2f, 0.001f), "r = step - corr: %+.3f", accTcRezeroResidual(&e, 2));
    CHECK(accTcLogRezero(&log, &e, 100000, 97500), "the airborne entry is logged");
    CHECK(log.airSeq == 1 && log.airLandMs == 2500 && log.air.stepCmg[2] == -550, "air slot");
    accTcRezero_t q;
    const float zero[3] = {0, 0, 0};
    accTcRezeroFill(&q, 31.0f, 31.2f, zero, zero, 0);
    for (int i = 0; i < 600; i++) {
      CHECK(!accTcLogRezero(&log, &q, 101000 + i * 1000, 0), "quiet re-zeros are not logged");
    }
    CHECK(accTcLogCount(&log) == 1 && accTcLogEntry(&log, 0)->stepCmg[2] == -550, "the flight's entry stays");
    accTcRezero_t spin;
    accTcRezeroFill(&spin, 32.0f, 31.0f, zero, zero, ACC_TC_RZ_FLIGHT);
    for (int i = 0; i < 9; i++) {
      accTcLogRezero(&log, &spin, 800000 + i * 1000, 0);
    }
    CHECK(accTcLogCount(&log) == ACC_TC_LOG_RING && log.n == 10, "ground spins wrap the ring (%u)", (unsigned)log.n);
    CHECK(log.airSeq == 1 && log.air.stepCmg[2] == -550, "the air slot is untouched by them");
    accTcRezero_t cool;
    const float cstep[3] = {0.5f, 0.2f, -4.0f};
    accTcRezeroFill(&cool, 50.0f, 40.0f, cstep, zero, ACC_TC_RZ_COOL);
    CHECK(accTcLogRezero(&log, &cool, 900000, 0), "a cooling spin's entry is logged");
    CHECK(accTcLogEntry(&log, ACC_TC_LOG_RING - 1)->flags == ACC_TC_RZ_COOL, "newest last");
    const float big[3] = {0, 0, 40000.0f};
    accTcRezeroFill(&cool, 50.0f, 40.0f, big, zero, 0);
    CHECK(cool.stepCmg[2] == 32767, "saturated to int16");
    const float nan3[3] = {NAN, NAN, NAN};
    accTcRezeroFill(&cool, NAN, 40.0f, nan3, zero, 0);
    CHECK(cool.stepCmg[2] == 0 && cool.tRefCc == 0, "NaN reads 0");
  }

  /* the NVS record */
  {
    accTcConfig_t k;
    accTcDefaultConfig(&k);
    k.k[2] = 0.35f;
    k.on = true;
    accTcRecord_t rec;
    accTcRecordInit(&rec, &k);
    CHECK(rec.size == sizeof(rec) && rec.magic == ACC_TC_MAGIC && rec.version == ACC_TC_VERSION && rec.on == 1,
          "record header");
    CHECK(accTcRecordValid(&rec, sizeof(rec)), "record valid");
    accTcConfig_t back;
    accTcDefaultConfig(&back);
    back.clampMg = 1.0f;
    accTcRecordApply(&rec, &back);
    CHECK(back.on && back.k[2] == 0.35f && back.kneeC == 60.0f && back.clampMg == ACC_TC_CLAMP_DEFAULT,
          "round trip (the clamp stays the default)");
    CHECK(!accTcRecordValid(&rec, sizeof(rec) - 1), "wrong stored size");
    accTcRecord_t bad = rec;
    bad.k[2] = 0.36f;
    CHECK(!accTcRecordValid(&bad, sizeof(bad)), "a changed byte fails the crc");
    bad = rec;
    bad.on = 2;
    CHECK(!accTcRecordValid(&bad, sizeof(bad)), "on > 1");
    k.k[2] = 2.5f;
    accTcRecordInit(&rec, &k);
    CHECK(!accTcRecordValid(&rec, sizeof(rec)), "k out of range is rejected even with a good crc");
    k.k[2] = 0.35f;
    k.kneeC = 20.0f;
    CHECK(!accTcConfigValid(&k), "knee below 30 invalid");
    k.kneeC = 60.0f;
    k.clampMg = 60.0f;
    CHECK(!accTcConfigValid(&k), "clamp above 50 invalid");
  }
}

/* ---- replay: the check on the logged ground rows -------------------------------------------------------- */

typedef struct {
  double t, thrust, ax, ay, az, T, gx, gy, gz, m;
  int flightId;
} row_t;

static int colIndex(char *header, const char *name)
{
  int i = 0;
  for (char *tok = strtok(header, ",\r\n"); tok; tok = strtok(NULL, ",\r\n"), i++) {
    if (strcmp(tok, name) == 0) {
      return i;
    }
  }
  return -1;
}

static bool readRows(const char *path, int flightId, row_t **out, int *n)
{
  FILE *f = fopen(path, "r");
  if (!f) {
    return false;
  }
  static char line[4096];
  if (!fgets(line, sizeof(line), f)) {
    fclose(f);
    return false;
  }
  char hdr[4096];
  const char *names[] = {"tickMs", "flightId", "thrust", "ax_mg", "ay_mg", "az_mg", "imuTemp_cC", "gx_ddps",
                         "gy_ddps", "gz_ddps", "m1", "m2", "m3", "m4"};
  int idx[14];
  for (int i = 0; i < 14; i++) {
    strcpy(hdr, line);
    idx[i] = colIndex(hdr, names[i]);
    if (idx[i] < 0) {
      fclose(f);
      return false;
    }
  }
  int cap = 20000;
  row_t *rows = malloc(sizeof(row_t) * cap);
  *n = 0;
  while (fgets(line, sizeof(line), f)) {
    double v[14] = {0};
    int col = 0, k = 0;
    for (char *tok = strtok(line, ",\r\n"); tok; tok = strtok(NULL, ",\r\n"), col++) {
      for (int i = 0; i < 14; i++) {
        if (idx[i] == col) {
          v[i] = atof(tok);
          k++;
        }
      }
    }
    if (k < 14 || (int)v[1] != flightId) {
      continue;
    }
    if (*n == cap) {
      cap *= 2;
      rows = realloc(rows, sizeof(row_t) * cap);
    }
    row_t *r = &rows[(*n)++];
    r->t = v[0] / 1000.0;
    r->flightId = (int)v[1];
    r->thrust = v[2];
    r->ax = v[3];
    r->ay = v[4];
    r->az = v[5];
    r->T = v[6] / 100.0;
    r->gx = v[7] / 10.0;
    r->gy = v[8] / 10.0;
    r->gz = v[9] / 10.0;
    r->m = v[10] + v[11] + v[12] + v[13];
  }
  fclose(f);
  *out = rows;
  return *n > 0;
}

static bool stillRow(const row_t *r)
{
  return r->thrust == 0 && r->m == 0 && fabs(r->gx) < 10 && fabs(r->gy) < 10 && fabs(r->gz) < 10;
}

/* The mean over the last 1 s of still rows ending at index end (exclusive), as the driver's window would hold it. */
static bool stillMean(const row_t *rows, int from, int end, double out[4])
{
  int cnt = 0;
  out[0] = out[1] = out[2] = out[3] = 0;
  for (int i = end - 1; i >= from && cnt < 100; i--) {
    if (!stillRow(&rows[i])) {
      return false;
    }
    out[0] += rows[i].ax;
    out[1] += rows[i].ay;
    out[2] += rows[i].az;
    out[3] += rows[i].T;
    cnt++;
  }
  if (cnt < 50) {
    return false;
  }
  for (int i = 0; i < 4; i++) {
    out[i] /= cnt;
  }
  return true;
}

static void replayPair(const char *dir, const char *file, int flightId)
{
  char path[1024];
  snprintf(path, sizeof(path), "%s/%s", dir, file);
  row_t *rows;
  int n;
  if (!readRows(path, flightId, &rows, &n)) {
    printf("replay: %s flight %d not found, skipped\n", file, flightId);
    return;
  }
  int first = -1, last = -1;
  for (int i = 0; i < n; i++) {
    if (rows[i].thrust > 0) {
      if (first < 0) {
        first = i;
      }
      last = i;
    }
  }
  double pre[4], post[4];
  bool ok = first > 0 && stillMean(rows, 0, first, pre);
  /* after the motors stop the driver waits >= 2 s before a re-zero; the logs hold ~2 s of ground rows */
  int postEnd = n;
  while (postEnd > last + 1 && !stillRow(&rows[postEnd - 1])) {
    postEnd--;
  }
  ok = ok && stillMean(rows, last + 1 + 100, postEnd, post);
  if (!ok) {
    printf("replay: %s flight %d has no still 1 s before / after, skipped\n", file, flightId);
    free(rows);
    return;
  }
  const double dT = post[3] - pre[3];
  const double step[3] = {post[0] - pre[0], post[1] - pre[1], post[2] - pre[2]};
  accTcConfig_t c;
  accTcDefaultConfig(&c);
  const float corr = c.k[2] * accTcEffDelta(&c, (float)post[3], (float)pre[3]);
  printf("replay %s flight %d: T %.1f -> %.1f C (dT %+.1f), still step x/y/z %+.2f/%+.2f/%+.2f mg, model z %+.2f, "
         "r_z %+.2f mg, k_z measured %+.3f mg/C\n", file, flightId, pre[3], post[3], dT, step[0], step[1], step[2],
         corr, step[2] - corr, step[2] / dT);
  CHECK(dT < -10.0, "the flight cooled the IMU by more than 10 C (%+.1f)", dT);
  CHECK(fabs(step[2]) < 10.0, "still z step within 10 mg over the flight: %+.2f", step[2]);
  CHECK(fabs(step[2] - corr) < 10.0, "the default model leaves |r_z| < 10 mg: %+.2f", step[2] - corr);
  CHECK(fabs(step[0]) < 20.0 && fabs(step[1]) < 20.0, "no attitude change (x/y within 20 mg): %+.2f %+.2f", step[0],
        step[1]);
  free(rows);
}

int main(int argc, char **argv)
{
  unitTests();
  if (argc >= 2) {
    replayPair(argv[1], "flight-2026-10-15-p1-height.csv", 4);
    replayPair(argv[1], "flight-2026-10-17-table.csv", 8);
  } else {
    printf("replay skipped (no CSV directory)\n");
  }
  printf("test_accel_tempco: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
