/*
 * Replay of the autonomy sorties through the camera height observer
 * (components/core/crazyflie/utils/src/cam_vz_obs.c): the gain re-anchor
 * (gainReanchorRel) against the 2026-10-13 interactive-hover failures
 * (docs/autonomy-a2.md section 12).
 *
 * Build and run from Firmware/esp-drone (skipped without the CSV directory):
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_cam_vz_replay_ih.c components/core/crazyflie/utils/src/cam_vz_obs.c \
 *      -lm -o /tmp/test_cam_vz_replay_ih && /tmp/test_cam_vz_replay_ih ../../firmware-backups/cal [-v]
 *
 * Input: the 100 Hz recorder files of the autonomy sorties
 * (flight-2026-10-1x-*.csv) and a flightId. Every row is one IMU sample of
 * the bare observer (as the camera height study replayed its flights), the
 * frames come on a new flowSeq, exactly as the investigators' replay of
 * 2026-10-13 (fix-ih/flight2/prepall.py + replay.c, which this file ports):
 *   - time: the recorder tick runs slow against the camera clock; each flight's
 *     factor K = sum of the logged frame intervals / sum of the tick
 *     differences of consecutive frames (both 20-200 ms); t = tick x K;
 *   - world-up specific force from the logged attitude and accelerometer,
 *     rounded to 1e-6 g; the ground flag is the drone's own (cvFlags bit 0);
 *   - g0: the median world-up specific force of the recorder's ground rows
 *     (lift - 128 .. lift - 5), never the drone's logged g0 (the recorder's
 *     a_up and the drone's differ by 3-5 mg, docs/autonomy-a2.md 4.11); the
 *     offset prior g0 - 1 + 1.37 mg, sigma 2.2 mg (CAM_VZ_PRIOR_*).
 * With these the re-anchor off reproduces the drone's decisions on flight 2
 * of 2026-10-13 exactly (81 accepted / 36 quality / 44 CUSUM / 1 alarm).
 *
 * Checks:
 *   (a) 2026-10-13 flight 2, re-anchor off: the drone's counts;
 *   (b) flight 2 with the re-anchor: no alarm and no CUSUM refusal, >= 120
 *       accepted, healthy at HOVER + 2.0 s and on every row from the first
 *       healthy one to the LAND entry, healthy >= 80 % of the airborne time,
 *       |h| at the last airborne row <= 0.65 m (1.44 off; 0.53 with a third
 *       re-anchor 4.3 s into LAND, which the take-off window now refuses),
 *       h at HOVER + 2 s in 0.9-1.3 m; no alarm for offset priors -6..+10 mg
 *       against g0;
 *   (c) 2026-10-13 flight 1 (the person, then the in-air contact at tick
 *       79020): decision counts unchanged (492 / 119 / 0 / 0) with the
 *       re-anchor, <= 3 re-anchors, |v| <= 0.10 m/s from HOVER + 2 s to the
 *       contact;
 *   (d) the eight clean autonomy sorties of 2026-10-12: no re-anchor, and
 *       every frame decision and every row's h, v and offset identical with
 *       the re-anchor on and off;
 *   (e) a dead camera during real vertical motion in the hover (2026-10-13
 *       review): the frames from HOVER + tf - 1 s carry no motion (dx = dy =
 *       scale = 0) and a true climb (0.1-0.4 m/s after a 0.5 s ramp) or a
 *       1 s up / 1 s down excursion (0.2-0.6 m/s) is added to the
 *       accelerometer from HOVER + tf, on the 7 sorties x tf of the review.
 *       The re-anchor must not hide it: mission60 flight 1, tf 4 s, 0.3 m/s
 *       unhealthy within 1.5 s (0.94 s off); no re-anchor beyond the
 *       flight's own; unhealthy no later than with the re-anchor off (+0.05
 *       s) except in <= 2 cases (flight 1's own climb re-anchors change the
 *       later state). Faults inside the take-off window (HOVER + 0..1 s on
 *       the short climbs) are printed.
 *   (f) the exposure-step proxy (docs/autonomy-a2b.md 19.4): frames whose
 *       logged mean brightness moved more than 8 % refused, as camera_flow.c
 *       refuses an exposure step it cannot correct; on the good flights
 *       (2026-10-14 flight 4, tiger f1/f2, tiger2 f1, mission60 f1, night f6,
 *       pwmA1 f7) < 1 % of the HOVER frames refused (the TAKEOFF climb's
 *       floor-to-room frames step more, at residuals the observer refuses
 *       anyway), no new unhealthy stretch in HOVER (+0.5 s at
 *       most, none reaching 2 s), the healthy share within 2 points; the
 *       20 MHz bright-window failures (interactive f2, preAB f3) printed.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cam_vz_obs.h"

static int failures = 0;
static int checks = 0;
static int verbose = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define MAX_ROWS 20000   /* a 180 s recorder dump (docs/autonomy-a2b.md 19.3) */
#define MAX_FRAMES 3000

enum { C_TICK, C_FID, C_THRUST, C_ROLL, C_PITCH, C_AX, C_AY, C_AZ, C_SEQ, C_OK, C_TEX, C_RES, C_DT, C_AGE, C_DX, C_DY,
       C_SC, C_FGX, C_FGY, C_FGZ, C_CVF, C_AUTO, C_LUM, C_N };
static const char *colNames[C_N] = {
  "tickMs", "flightId", "thrust", "roll_cdeg", "pitch_cdeg", "ax_mg", "ay_mg", "az_mg", "flowSeq", "flowOk",
  "flowTexture", "flowResidual_dg", "flowDt_100us", "flowAge_ms", "flowDx_cpx", "flowDy_cpx", "flowScale_e4",
  "flowGx_e4", "flowGy_e4", "flowGz_e4", "cvFlags", "autoState", "flowLum",
};
/* The exposure-step proxy (docs/autonomy-a2b.md 19.4; no image sequences were logged): prepare() refuses a frame
 * whose mean brightness moved more than 8 % from the previous frame's, as camera_flow.c does without a measured
 * gain. lumGated counts them. */
static bool lumGate;
static int lumGated, lumGatedHover;
static long csv[MAX_ROWS][C_N];
static int csvRows;

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
  csvRows = 0;
  while (csvRows < MAX_ROWS && fgets(line, sizeof(line), f) != NULL) {
    col = 0;
    for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
      for (int c = 0; c < C_N; c++) {
        if (idx[c] == col) {
          csv[csvRows][c] = strtol(tok, NULL, 10);
        }
      }
    }
    csvRows++;
  }
  fclose(f);
  return true;
}

/* One flight, ready for the observer. */
typedef struct {
  int n;                     /* rows */
  uint64_t t[MAX_ROWS];      /* us */
  double aUp[MAX_ROWS];      /* g, rounded to 1e-6 */
  bool ground[MAX_ROWS];
  float thrust[MAX_ROWS];
  long tick[MAX_ROWS];
  int autoState[MAX_ROWS];
  int frameAt[MAX_ROWS];     /* index into fr[] of the frame after this row, -1 none */
  camVzObsFrame_t fr[MAX_FRAMES];
  int nf;
  double g0, K;
  int tHover, tLand;         /* first HOVER / LAND rows, -1 none */
} flight_t;

static flight_t fl;

static double rnd(double x, const char *fmt)
{
  char b[64];
  snprintf(b, sizeof(b), fmt, x);
  return atof(b);
}

static int cmpd(const void *a, const void *b)
{
  const double x = *(const double *)a, y = *(const double *)b;
  return (x > y) - (x < y);
}

/* The rows of flightId fid of the loaded file (fix-ih/flight2/prepall.py). */
static bool prepare(int fid)
{
  int i0 = -1, i1 = -1;
  for (int i = 0; i < csvRows; i++) {
    if (csv[i][C_FID] == fid) {
      if (i0 < 0) {
        i0 = i;
      }
      i1 = i + 1;
    }
  }
  if (i0 < 0) {
    return false;
  }
  memset(&fl, 0, sizeof(fl));
  fl.n = i1 - i0;
  const long (*r)[C_N] = &csv[i0];
  /* K from consecutive frames */
  static int nw[MAX_FRAMES];
  int nn = 0;
  for (int i = 1; i < fl.n && nn < MAX_FRAMES; i++) {
    if (r[i][C_SEQ] != r[i - 1][C_SEQ] && r[i][C_SEQ] > 0) {
      nw[nn++] = i;
    }
  }
  double sr = 0.0, st = 0.0;
  for (int k = 1; k < nn; k++) {
    const long ds = (((r[nw[k]][C_SEQ] - r[nw[k - 1]][C_SEQ]) % 255) + 255) % 255;
    if (ds != 1) {
      continue;
    }
    const double dtr = (double)r[nw[k]][C_DT] / 1e4;
    const double dtt = (double)((r[nw[k]][C_TICK] - r[nw[k]][C_AGE]) - (r[nw[k - 1]][C_TICK] - r[nw[k - 1]][C_AGE])) / 1000.0;
    if (dtr > 0.02 && dtr < 0.2 && dtt > 0.01 && dtt < 0.3) {
      sr += dtr;
      st += dtt;
    }
  }
  fl.K = st > 0.0 ? sr / st : 1.0;
  static double a[MAX_ROWS];
  int lift = -1;
  fl.tHover = fl.tLand = -1;
  for (int i = 0; i < fl.n; i++) {
    const double rr = (double)r[i][C_ROLL] / 100.0 * M_PI / 180.0, pp = (double)r[i][C_PITCH] / 100.0 * M_PI / 180.0;
    a[i] = (double)r[i][C_AX] / 1000.0 * sin(pp) + (double)r[i][C_AY] / 1000.0 * cos(pp) * sin(rr) +
           (double)r[i][C_AZ] / 1000.0 * cos(pp) * cos(rr);
    fl.aUp[i] = rnd(a[i], "%.6f");
    fl.t[i] = (uint64_t)((double)(r[i][C_TICK] - r[0][C_TICK]) * fl.K * 1000.0 + 1e6);
    fl.ground[i] = (r[i][C_CVF] & 1) == 0;
    fl.thrust[i] = (float)r[i][C_THRUST];
    fl.tick[i] = r[i][C_TICK];
    fl.autoState[i] = (int)r[i][C_AUTO];
    fl.frameAt[i] = -1;
    if (lift < 0 && !fl.ground[i]) {
      lift = i;
    }
    if (fl.tHover < 0 && fl.autoState[i] == 3) {
      fl.tHover = i;
    }
    if (fl.tLand < 0 && fl.autoState[i] == 4) {
      fl.tLand = i;
    }
  }
  /* frames */
  uint32_t seq = 0;
  long prev = -1;
  long prevLum = -1;
  lumGated = lumGatedHover = 0;
  for (int i = 1; i < fl.n && fl.nf < MAX_FRAMES; i++) {
    if (!(r[i][C_SEQ] != r[i - 1][C_SEQ] && r[i][C_SEQ] > 0)) {
      continue;
    }
    if (prev < 0) {
      seq = (uint32_t)r[i][C_SEQ];
    } else {
      seq += (uint32_t)((((r[i][C_SEQ] - prev - 1) % 255) + 255) % 255) + 1u;
    }
    prev = r[i][C_SEQ];
    const double tRow = (double)(r[i][C_TICK] - r[0][C_TICK]) * fl.K * 1000.0 + 1e6;
    camVzObsFrame_t *f = &fl.fr[fl.nf];
    memset(f, 0, sizeof(*f));
    f->seq = seq;
    f->frameTimeUs = (uint64_t)(tRow - (double)r[i][C_AGE] * 1000.0);
    f->dtUs = (uint32_t)r[i][C_DT] * 100u;
    f->dx = (float)rnd((double)r[i][C_DX] / 100.0, "%.2f");
    f->dy = (float)rnd((double)r[i][C_DY] / 100.0, "%.2f");
    f->scale = (float)rnd((double)r[i][C_SC] / 1e4, "%.4f");
    f->gyroAngle[0] = (float)rnd((double)r[i][C_FGX] / 1e4, "%.4f");
    f->gyroAngle[1] = (float)rnd((double)r[i][C_FGY] / 1e4, "%.4f");
    f->gyroAngle[2] = (float)rnd((double)r[i][C_FGZ] / 1e4, "%.4f");
    f->texture = (float)r[i][C_TEX];
    f->residual = (float)rnd((double)r[i][C_RES] / 10.0, "%.1f");
    f->ok = r[i][C_OK] != 0;
    if (prevLum > 0 && labs(r[i][C_LUM] - prevLum) * 100 > 8 * prevLum) {
      lumGated += !fl.ground[i];
      lumGatedHover += !fl.ground[i] && fl.autoState[i] == 3;
      f->ok = f->ok && !lumGate;
    }
    prevLum = r[i][C_LUM];
    fl.frameAt[i] = fl.nf++;
  }
  /* g0: the recorder's own ground rows before the lift-off */
  int g0i = 0, g1i = 1;
  if (lift > 10) {
    g0i = lift - 128 > 0 ? lift - 128 : 0;
    g1i = lift - 5;
  } else {
    g1i = lift > 1 ? lift : 1;
  }
  static double gs[MAX_ROWS];
  int ng = 0;
  for (int i = g0i; i < g1i; i++) {
    gs[ng++] = a[i];
  }
  qsort(gs, (size_t)ng, sizeof(double), cmpd);
  fl.g0 = ng == 0 ? 1.0 : (ng % 2 ? gs[ng / 2] : 0.5 * (gs[ng / 2 - 1] + gs[ng / 2]));
  return true;
}

/* One replay. Per row (after its IMU sample): h, v, offset, healthy; per frame: the decision. */
typedef struct {
  float h[MAX_ROWS], v[MAX_ROWS], b[MAX_ROWS];
  bool healthy[MAX_ROWS];
  uint8_t dec[MAX_FRAMES];
  int n[10];
  camVzObsCounters_t cnt;
  double hTd, vTd;          /* last airborne row */
  double okPct;             /* healthy share of the airborne time */
} run_t;

static run_t runA, runB;

static void replay(float rel, double priorMg, run_t *R)
{
  camVzObsConfig_t c;
  camVzObsDefaultConfig(&c);
  c.gainReanchorRel = rel;
  static camVzObs_t o;
  CHECK(camVzObsInit(&o, &c), "config");
  camVzObsReset(&o);
  camVzObsSetBiasPrior(&o, (float)(fl.g0 - 1.0 + priorMg * 1e-3), 2.2e-3f);
  memset(R, 0, sizeof(*R));
  double airUs = 0.0, healthyUs = 0.0;
  uint64_t tPrev = 0;
  for (int i = 0; i < fl.n; i++) {
    camVzObsImu(&o, fl.t[i], (float)fl.aUp[i], fl.ground[i], fl.thrust[i]);
    camVzObsOut_t q;
    camVzObsGet(&o, &q);
    R->h[i] = q.h;
    R->v[i] = q.v;
    R->b[i] = q.biasG;
    R->healthy[i] = q.healthy;
    if (!fl.ground[i]) {
      R->hTd = q.h;
      R->vTd = q.v;
      airUs += (double)(fl.t[i] - tPrev);
      if (q.healthy) {
        healthyUs += (double)(fl.t[i] - tPrev);
      }
    }
    tPrev = fl.t[i];
    if (fl.frameAt[i] >= 0) {
      const camVzObsDecision_t d = camVzObsFrame(&o, &fl.fr[fl.frameAt[i]], fl.t[i]);
      R->dec[fl.frameAt[i]] = (uint8_t)d;
      R->n[d]++;
    }
  }
  R->cnt = o.cnt;
  R->okPct = airUs > 0.0 ? 100.0 * healthyUs / airUs : 0.0;
}

/* First row at or after HOVER entry + s seconds (camera clock). */
static int hoverPlus(double s)
{
  if (fl.tHover < 0) {
    return -1;
  }
  for (int i = fl.tHover; i < fl.n; i++) {
    if ((double)(fl.t[i] - fl.t[fl.tHover]) >= s * 1e6) {
      return i;
    }
  }
  return -1;
}

static void printRun(const char *name, float rel, const run_t *R)
{
  printf("  %-26s rel %.1f: acc %3d q %3d exp %d lat %d cusum %2d gate %d | alarms %u re-anchors %u | ok %3.0f %% | "
         "h at the last airborne row %+.2f m, v %+.2f m/s\n", name, (double)rel, R->n[CAM_VZ_F_ACCEPTED],
         R->n[CAM_VZ_F_QUALITY], R->n[CAM_VZ_F_EXPANSION], R->n[CAM_VZ_F_LATERAL], R->n[CAM_VZ_F_CUSUM],
         R->n[CAM_VZ_F_GATE], (unsigned)R->cnt.alarms, (unsigned)R->cnt.reanchors, R->okPct, R->hTd, R->vTd);
}

static void flight2(void)
{
  replay(0.0f, 1.37, &runA);
  replay(0.3f, 1.37, &runB);
  printf("2026-10-13 flight 2 (HOVER_NO_CAM, window): K %.5f, g0 - 1 %+.2f mg\n", fl.K, (fl.g0 - 1.0) * 1e3);
  printRun("re-anchor off", 0.0f, &runA);
  printRun("re-anchor on", 0.3f, &runB);
  /* (a) the drone's decisions */
  CHECK(runA.n[CAM_VZ_F_ACCEPTED] == 81 && runA.n[CAM_VZ_F_QUALITY] == 36 && runA.n[CAM_VZ_F_CUSUM] == 44 &&
        runA.cnt.alarms == 1, "flight 2, re-anchor off: %d / %d / %d / %u, the drone's 81 / 36 / 44 / 1",
        runA.n[CAM_VZ_F_ACCEPTED], runA.n[CAM_VZ_F_QUALITY], runA.n[CAM_VZ_F_CUSUM], (unsigned)runA.cnt.alarms);
  const int i2 = hoverPlus(2.0);
  CHECK(i2 > 0 && !runA.healthy[i2], "flight 2, re-anchor off: unhealthy at HOVER + 2 s (the LAND HOVER_NO_CAM)");
  /* (b) the fix */
  int first = -1, gaps = 0;
  for (int i = 0; i < fl.n && (fl.tLand < 0 || i < fl.tLand); i++) {
    if (first < 0 && runB.healthy[i]) {
      first = i;
    }
    if (first >= 0 && !runB.healthy[i]) {
      gaps++;
    }
  }
  printf("  re-anchor on: healthy at HOVER + 2 s %d (h %.2f m); first healthy %.2f s after HOVER entry, unhealthy rows "
         "after it before LAND %d\n", i2 > 0 && runB.healthy[i2], i2 > 0 ? runB.h[i2] : NAN,
         first >= 0 ? ((double)fl.t[first] - (double)fl.t[fl.tHover]) * 1e-6 : NAN, gaps);
  CHECK(runB.cnt.alarms == 0 && runB.n[CAM_VZ_F_CUSUM] == 0 && runB.n[CAM_VZ_F_ACCEPTED] >= 120,
        "flight 2 fixed: %u alarms, %d CUSUM refusals, %d accepted", (unsigned)runB.cnt.alarms,
        runB.n[CAM_VZ_F_CUSUM], runB.n[CAM_VZ_F_ACCEPTED]);
  CHECK(i2 > 0 && runB.healthy[i2] && first >= 0 && gaps == 0, "flight 2 fixed: healthy at HOVER + 2.0 s and on "
        "every row from the first healthy one to LAND (%d gaps)", gaps);
  CHECK(runB.okPct >= 80.0 && fabs(runB.hTd) <= 0.65 && i2 > 0 && runB.h[i2] >= 0.9f && runB.h[i2] <= 1.3f,
        "flight 2 fixed: ok %.0f %% (>= 80), |h| at the last airborne row %.3f m (<= 0.65), h at HOVER + 2 s %.2f m "
        "(0.9-1.3)", runB.okPct, fabs(runB.hTd), i2 > 0 ? runB.h[i2] : NAN);
  CHECK(runB.cnt.reanchors >= 1 && runB.cnt.reanchors <= 4, "flight 2 fixed: %u re-anchors",
        (unsigned)runB.cnt.reanchors);
  const double priors[] = { -6.0, -3.0, -1.0, 0.0, 3.0, 5.0, 8.0, 10.0 };
  char buf[256] = "";
  int bad = 0;
  for (unsigned k = 0; k < sizeof(priors) / sizeof(priors[0]); k++) {
    replay(0.3f, priors[k], &runB);
    snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " %+.0f:%u", priors[k], (unsigned)runB.cnt.alarms);
    bad += runB.cnt.alarms != 0;
  }
  printf("  re-anchor on, offset prior against g0 (mg): alarms%s\n", buf);
  CHECK(bad == 0, "flight 2 fixed: no alarm for priors -6..+10 mg (%d with)", bad);
}

static void flight1(void)
{
  replay(0.0f, 1.37, &runA);
  replay(0.3f, 1.37, &runB);
  printf("2026-10-13 flight 1 (the person, contact at tick 79020): K %.5f, g0 - 1 %+.2f mg\n", fl.K,
         (fl.g0 - 1.0) * 1e3);
  printRun("re-anchor off", 0.0f, &runA);
  printRun("re-anchor on", 0.3f, &runB);
  bool same = true;
  for (int i = 0; i < fl.nf; i++) {
    same = same && runA.dec[i] == runB.dec[i];
  }
  CHECK(same && runB.n[CAM_VZ_F_ACCEPTED] == 492 && runB.n[CAM_VZ_F_QUALITY] == 119 && runB.n[CAM_VZ_F_CUSUM] == 0 &&
        runB.cnt.alarms == 0, "flight 1: decisions unchanged (%d) and the drone's 492 / 119 / 0 / 0: %d / %d / %d / %u",
        same, runB.n[CAM_VZ_F_ACCEPTED], runB.n[CAM_VZ_F_QUALITY], runB.n[CAM_VZ_F_CUSUM], (unsigned)runB.cnt.alarms);
  CHECK(runB.cnt.reanchors <= 3, "flight 1: %u re-anchors (<= 3)", (unsigned)runB.cnt.reanchors);
  const int i2 = hoverPlus(2.0);
  double vMaxA = 0.0, vMaxB = 0.0;
  int iMax = -1;
  for (int i = i2; i2 >= 0 && i < fl.n && fl.tick[i] < 79020; i++) {
    vMaxA = fmax(vMaxA, fabs(runA.v[i]));
    if (fabs(runB.v[i]) > vMaxB) {
      vMaxB = fabs(runB.v[i]);
      iMax = i;
    }
  }
  printf("  max |v| from HOVER + 2 s to the contact: %.3f m/s (re-anchor off %.3f), at %.2f s after HOVER entry\n",
         vMaxB, vMaxA, iMax >= 0 ? ((double)fl.t[iMax] - (double)fl.t[fl.tHover]) * 1e-6 : NAN);
  CHECK(i2 > 0 && vMaxB <= 0.10, "flight 1: |v| <= 0.10 m/s from HOVER + 2 s to the contact (%.3f)", vMaxB);
}

/* (d) the clean sorties: identical with the re-anchor on and off. */
static void cleanSortie(const char *name)
{
  replay(0.0f, 1.37, &runA);
  replay(0.3f, 1.37, &runB);
  int decDiff = 0, rowDiff = 0;
  for (int i = 0; i < fl.nf; i++) {
    decDiff += runA.dec[i] != runB.dec[i];
  }
  for (int i = 0; i < fl.n; i++) {
    rowDiff += runA.h[i] != runB.h[i] || runA.v[i] != runB.v[i] || runA.b[i] != runB.b[i] ||
               runA.healthy[i] != runB.healthy[i];
  }
  if (verbose) {
    printRun(name, 0.3f, &runB);
  }
  CHECK(runB.cnt.reanchors == 0 && decDiff == 0 && rowDiff == 0 && runB.n[CAM_VZ_F_ACCEPTED] > 0,
        "%s: %u re-anchors, %d frame decisions and %d rows differ from the re-anchor off (%d accepted)", name,
        (unsigned)runB.cnt.reanchors, decDiff, rowDiff, runB.n[CAM_VZ_F_ACCEPTED]);
}

/* (e) a dead camera during real vertical motion in the hover */
static flight_t clean;

/* The frames from HOVER + tf - 1 s without motion, and vc m/s added to the
 * accelerometer from HOVER + tf: a climb (0.5 s ramp) or a 1 s up / 1 s down
 * excursion. */
static void injectFrozen(double tf, double vc, bool sustained)
{
  memcpy(&fl, &clean, sizeof(fl));
  const uint64_t t0 = fl.t[fl.tHover] + (uint64_t)(tf * 1e6);
  for (int i = 0; i < fl.n; i++) {
    const double tt = ((double)fl.t[i] - (double)t0) * 1e-6;
    double acc;
    if (sustained) {
      acc = tt >= 0.0 && tt < 0.5 ? vc / 0.5 : 0.0;
    } else {
      acc = tt >= 0.0 && tt < 1.0 ? vc : (tt >= 1.0 && tt < 2.0 ? -vc : 0.0);
    }
    fl.aUp[i] += acc / CAM_VZ_G;
  }
  for (int j = 0; j < fl.nf; j++) {
    camVzObsFrame_t *f = &fl.fr[j];
    if (((double)f->frameTimeUs - (double)t0) * 1e-6 > -1.0) {
      f->dx = 0.0f;
      f->dy = 0.0f;
      f->scale = 0.0f;
    }
  }
}

/* s from HOVER + tf to the first unhealthy row (6 s at most; the excursion
 * up to LAND), NAN if none. */
static double unhealthyAfter(const run_t *R, double tf, bool sustained)
{
  const uint64_t t0 = fl.t[fl.tHover] + (uint64_t)(tf * 1e6);
  for (int i = 0; i < fl.n; i++) {
    const double tt = ((double)fl.t[i] - (double)t0) * 1e-6;
    if (tt < 0.0) {
      continue;
    }
    if (tt > 6.0 || (!sustained && fl.tLand > 0 && i >= fl.tLand)) {
      break;
    }
    if (!R->healthy[i]) {
      return tt;
    }
  }
  return NAN;
}

static void frozenCamera(const char *dir)
{
  static const struct { const char *file; int fid; double tf; bool review; } cases[] = {
    { "flight-2026-10-13-interactive.csv", 1, 6.0, true }, { "flight-2026-10-13-interactive.csv", 1, 15.0, true },
    { "flight-2026-10-12-a2b-hops.csv", 2, 2.0, true },     { "flight-2026-10-12-calib1.csv", 1, 2.0, true },
    { "flight-2026-10-12-mission5s.csv", 1, 3.0, true },    { "flight-2026-10-12-mission60.csv", 1, 4.0, true },
    { "flight-2026-10-12-mission60.csv", 1, 8.0, true },
    /* inside the take-off window (the frames dead from HOVER + 0..0.5 s) */
    { "flight-2026-10-12-mission5s.csv", 1, 1.5, false },   { "flight-2026-10-12-mission60.csv", 1, 1.0, false },
    { "flight-2026-10-13-interactive.csv", 1, 1.5, false },
  };
  char path[1024];
  int n = 0, later = 0, earlier = 0, extraRa = 0, nWin = 0, laterWin = 0, extraRaWin = 0;
  double regOn = NAN, regOff = NAN;
  for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
    snprintf(path, sizeof(path), "%s/%s", dir, cases[k].file);
    const bool ok = loadCsv(path) && prepare(cases[k].fid) && fl.tHover >= 0;
    CHECK(ok, "%s flight %d", cases[k].file, cases[k].fid);
    if (!ok) {
      continue;
    }
    memcpy(&clean, &fl, sizeof(clean));
    replay(0.3f, 1.37, &runB);
    const unsigned ra0 = runB.cnt.reanchors;
    for (int sus = 1; sus >= 0; sus--) {
      for (int m = 1; m <= (sus ? 4 : 3); m++) {
        const double vc = sus ? 0.1 * m : 0.2 * m;
        injectFrozen(cases[k].tf, vc, sus);
        replay(0.0f, 1.37, &runA);
        replay(0.3f, 1.37, &runB);
        const double tOff = unhealthyAfter(&runA, cases[k].tf, sus), tOn = unhealthyAfter(&runB, cases[k].tf, sus);
        const bool isLater = !isnan(tOff) && (isnan(tOn) || tOn > tOff + 0.05);
        const bool isEarlier = !isnan(tOn) && (isnan(tOff) || tOn < tOff - 0.05);
        const bool ra = runB.cnt.reanchors != ra0;
        if (cases[k].review) {
          n++;
          later += isLater;
          earlier += isEarlier;
          extraRa += ra;
        } else {
          nWin++;
          laterWin += isLater;
          extraRaWin += ra;
        }
        if (strcmp(cases[k].file, "flight-2026-10-12-mission60.csv") == 0 && cases[k].tf == 4.0 && sus && m == 3) {
          regOn = tOn;
          regOff = tOff;
        }
        if (verbose || isLater || isEarlier || (ra && cases[k].review)) {
          printf("  dead camera, %.20s f%d HOVER + %4.1f s, %s %.1f m/s: unhealthy after %5.2f s (off %5.2f), "
                 "re-anchors %u (the flight's own %u)%s\n", cases[k].file + 12, cases[k].fid, cases[k].tf,
                 sus ? "climb    " : "excursion", vc, tOn, tOff, (unsigned)runB.cnt.reanchors, ra0,
                 isLater ? "  LATER" : (isEarlier ? "  earlier" : ""));
        }
      }
    }
  }
  printf("dead camera in the hover, %d review cases: unhealthy later than with the re-anchor off in %d, earlier in "
         "%d, a re-anchor in the fault in %d; mission60 f1 tf 4 s 0.3 m/s climb: unhealthy after %.2f s (off %.2f); "
         "inside the take-off window (%d cases, printed): later in %d, a re-anchor in the fault in %d\n", n, later,
         earlier, extraRa, regOn, regOff, nWin, laterWin, extraRaWin);
  CHECK(!isnan(regOn) && regOn <= 1.5, "mission60 f1, dead camera from HOVER + 3 s, 0.3 m/s climb from HOVER + 4 s: "
        "unhealthy after %.2f s (<= 1.5; off %.2f)", regOn, regOff);
  CHECK(n == 49 && extraRa == 0 && later <= 2, "dead camera, %d review cases: %d with a re-anchor in the fault, %d "
        "unhealthy later than with the re-anchor off (<= 2)", n, extraRa, later);
}

/* The longest stretch (s, camera clock) without a healthy estimate from HOVER + 2 s to the LAND entry (the FSM's
 * HOVER_NO_CAM: 2 s at most; the MOVE's view: 2 s). */
static double longestUnhealthy(const run_t *R)
{
  const int i0 = hoverPlus(2.0);
  const int i1 = fl.tLand >= 0 ? fl.tLand : fl.n;
  double best = 0.0;
  int start = -1;
  for (int i = i0 < 0 ? fl.n : i0; i < i1; i++) {
    if (!R->healthy[i]) {
      if (start < 0) {
        start = i;
      }
      best = fmax(best, (double)(fl.t[i] - fl.t[start]) * 1e-6);
    } else {
      start = -1;
    }
  }
  return best;
}

/* (f) the exposure-step proxy on the good flights and the 20 MHz bright-window failures. */
static void exposureProxy(const char *dir)
{
  static const struct { const char *file; int fid; bool good; } cases[] = {
    { "flight-2026-10-14-morning.csv", 4, true },    { "flight-2026-10-13-tiger.csv", 1, true },
    { "flight-2026-10-13-tiger.csv", 2, true },      { "flight-2026-10-13-tiger2.csv", 1, true },
    { "flight-2026-10-12-mission60.csv", 1, true },  { "flight-2026-10-13-night.csv", 6, true },
    { "flight-2026-10-13-pwmA1.csv", 7, true },
    { "flight-2026-10-13-interactive.csv", 2, false }, { "flight-2026-10-13-preAB.csv", 3, false },
  };
  camVzObsConfig_t c;
  camVzObsDefaultConfig(&c);
  char path[1024];
  for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
    snprintf(path, sizeof(path), "%s/%s", dir, cases[k].file);
    lumGate = false;
    const bool ok = loadCsv(path) && prepare(cases[k].fid);
    CHECK(ok, "%s flight %d", cases[k].file, cases[k].fid);
    if (!ok) {
      continue;
    }
    replay(c.gainReanchorRel, 1.37, &runA);
    int airFrames = 0, hoverFrames = 0;
    for (int i = 0; i < fl.n; i++) {
      airFrames += fl.frameAt[i] >= 0 && !fl.ground[i];
      hoverFrames += fl.frameAt[i] >= 0 && !fl.ground[i] && fl.autoState[i] == 3;
    }
    const double uA = longestUnhealthy(&runA);
    lumGate = true;
    prepare(cases[k].fid);
    lumGate = false;
    replay(c.gainReanchorRel, 1.37, &runB);
    const double uB = longestUnhealthy(&runB);
    const double pct = hoverFrames ? 100.0 * lumGatedHover / hoverFrames : 0.0;
    printf("  exposure proxy %-30s f%d: refused %2d of %4d airborne frames, %2d of %4d in HOVER (%.2f %%); healthy "
           "%3.0f -> %3.0f %% of the air; longest unhealthy stretch in HOVER %.2f -> %.2f s; accepted %d -> %d%s\n",
           cases[k].file + 7, cases[k].fid, lumGated, airFrames, lumGatedHover, hoverFrames, pct, runA.okPct,
           runB.okPct, uA, uB, runA.n[CAM_VZ_F_ACCEPTED], runB.n[CAM_VZ_F_ACCEPTED],
           cases[k].good ? "" : " (20 MHz bright-window failure)");
    if (cases[k].good) {
      CHECK(pct < 1.0, "%s f%d: %.2f %% of the HOVER frames refused (< 1 %%)", cases[k].file, cases[k].fid, pct);
      CHECK(uB <= uA + 0.5 && (uA >= 2.0 || uB < 2.0), "%s f%d: a new unhealthy stretch (%.2f -> %.2f s)",
            cases[k].file, cases[k].fid, uA, uB);
      CHECK(runB.okPct >= runA.okPct - 2.0, "%s f%d: healthy %.0f -> %.0f %%", cases[k].file, cases[k].fid,
            runA.okPct, runB.okPct);
    }
  }
}

int main(int argc, char **argv)
{
  const char *dir = NULL;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-v") == 0) {
      verbose = 1;
    } else {
      dir = argv[i];
    }
  }
  if (dir == NULL) {
    printf("replay skipped: pass the directory with the flight-2026-10-1x-*.csv files (firmware-backups/cal)\n");
    printf("%d checks, %d failures\n", checks, failures);
    return 0;
  }
  char path[1024];
  snprintf(path, sizeof(path), "%s/flight-2026-10-13-interactive.csv", dir);
  CHECK(loadCsv(path), "%s", path);
  if (csvRows > 0) {
    CHECK(prepare(2), "flight 2");
    flight2();
    CHECK(prepare(1), "flight 1");
    flight1();
  }
  static const struct { const char *file; int fid; } clean[] = {
    { "flight-2026-10-12-a2b-hops.csv", 1 }, { "flight-2026-10-12-a2b-hops.csv", 2 },
    { "flight-2026-10-12-calib1.csv", 1 },   { "flight-2026-10-12-calib2.csv", 1 },
    { "flight-2026-10-12-evening.csv", 4 },  { "flight-2026-10-12-hop5s.csv", 3 },
    { "flight-2026-10-12-mission5s.csv", 1 }, { "flight-2026-10-12-mission60.csv", 1 },
  };
  int n = 0;
  for (unsigned k = 0; k < sizeof(clean) / sizeof(clean[0]); k++) {
    snprintf(path, sizeof(path), "%s/%s", dir, clean[k].file);
    const bool ok = loadCsv(path) && prepare(clean[k].fid);
    CHECK(ok, "%s flight %d", clean[k].file, clean[k].fid);
    if (ok) {
      char name[96];
      snprintf(name, sizeof(name), "%.22s f%d", clean[k].file + 12, clean[k].fid);
      cleanSortie(name);
      n++;
    }
  }
  printf("clean 2026-10-12 autonomy sorties: %d replayed, re-anchor on vs off identical\n", n);
  frozenCamera(dir);
  exposureProxy(dir);
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
