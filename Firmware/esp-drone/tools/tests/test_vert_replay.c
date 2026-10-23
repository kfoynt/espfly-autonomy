/*
 * Replay of logged take-offs and landings through the vertical controller
 * (components/core/crazyflie/utils/src/vert_ctrl.c, docs/autonomy-a2.md 9.1)
 *
 * Build and run from Firmware/esp-drone (skipped without the CSV directory):
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_vert_replay.c components/core/crazyflie/utils/src/vert_ctrl.c \
 *      -lm -o /tmp/test_vert_replay && /tmp/test_vert_replay ../../firmware-backups/cal
 *
 * Input: the 100 Hz flight recorder CSVs (flight*.csv, header names the
 * columns; only files with vbat_mv are used). Each 100 Hz sample is held for
 * ten 1 ms steps.
 *
 * Lift-off: the real vzStep() runs SPIN and RAMP on the logged world-up
 * specific force, attitude, gyro, battery and applied thrust. The logs have
 * no idle spin, so SPIN is fed the take-off's own ground baseline (median of
 * its first samples below thrust 20000, as an idle-spin stand-in). The
 * controller's own RAMP command is irrelevant here (the pilot's thrust is
 * what flew), so the replay config ramps at the minimum rate for the maximum
 * time (3 s) to keep its ceiling and timeout out of the way. Reference
 * lift-off: the first sample with the specific force > 25 mg above that
 * baseline for 3 samples at thrust >= 20000 (the analysis behind
 * docs/autonomy-a2.md open issue 1).
 *
 * Touchdown: a manual landing is flown by the pilot's thrust, not by the
 * LAND ramp, so the controller's own LAND cannot be replayed. Instead the
 * touchdown RULE (critique B4, section 4.2) is evaluated on the logged
 * thrust: tdThrustOk = thrust <= 0.75 T_hov with T_hov = K_ref / V from the
 * flight's own hover, force = 20 Hz (accZ - g0) + 1 > 1.3 x thrust V / K_ref
 * for 200 ms, jolt = 20 Hz (accZ - g0) > 0.6 g with tdThrustOk. Reported:
 * whether it fires before the motors stop at each landing, and whether it
 * fires anywhere in the airborne part of the flight (false touchdown).
 *
 * (The support probe's replay on the logged contacts of 2026-10-13 was removed with the probe on 2026-10-15,
 * docs/autonomy-a2b.md 22: the palm catch is gone.)
 *
 * The idle spins (spinReplay): the 26 logged SPINs of 2026-10-12..14 through the real SPIN with the SPIN_GYRO
 * persistence (docs/autonomy-a2b.md 19.7) and with the one-sample rule: no abort, the same g0.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vert_ctrl.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define MAX_ROWS 20000   /* a 180 s recorder dump (docs/autonomy-a2b.md 19.3) */
#define STEPS_PER_SAMPLE 10
#define DT 0.001f

enum { C_FID, C_THRUST, C_ROLL, C_PITCH, C_GX, C_GY, C_GZ, C_AX, C_AY, C_AZ, C_VBAT, C_N };
static const char *colNames[C_N] = {
  "flightId", "thrust", "roll_cdeg", "pitch_cdeg", "gx_ddps", "gy_ddps", "gz_ddps", "ax_mg", "ay_mg", "az_mg", "vbat_mv",
};

typedef struct {
  int n;
  int fid[MAX_ROWS], thrust[MAX_ROWS], vbat[MAX_ROWS];
  float aUp[MAX_ROWS], gravZ[MAX_ROWS], tilt[MAX_ROWS], gyro[MAX_ROWS];
} log_t;

static log_t lg;

static bool loadCsv(const char *path, log_t *l)
{
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    return false;
  }
  static char line[4096];
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
      return false;   /* old format without vbat */
    }
  }
  l->n = 0;
  while (l->n < MAX_ROWS && fgets(line, sizeof(line), f) != NULL) {
    long v[C_N] = { 0 };
    col = 0;
    for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
      for (int c = 0; c < C_N; c++) {
        if (idx[c] == col) {
          v[c] = strtol(tok, NULL, 10);
        }
      }
    }
    const int i = l->n++;
    const float r = (float)v[C_ROLL] / 100.0f * 0.0174533f, p = (float)v[C_PITCH] / 100.0f * 0.0174533f;
    /* world-up specific force from the body accelerometer and the attitude */
    l->aUp[i] = ((float)v[C_AX] * sinf(p) + (float)v[C_AY] * cosf(p) * sinf(r) + (float)v[C_AZ] * cosf(p) * cosf(r)) /
                1000.0f;
    l->gravZ[i] = cosf(r) * cosf(p);
    l->tilt[i] = acosf(fminf(1.0f, l->gravZ[i])) * 57.2958f;
    l->gyro[i] = fmaxf(fabsf((float)v[C_GX]), fmaxf(fabsf((float)v[C_GY]), fabsf((float)v[C_GZ]))) / 10.0f;
    l->fid[i] = (int)v[C_FID];
    l->thrust[i] = (int)v[C_THRUST];
    l->vbat[i] = (int)v[C_VBAT];
  }
  fclose(f);
  return true;
}

static int cmpf(const void *a, const void *b)
{
  const float x = *(const float *)a, y = *(const float *)b;
  return x < y ? -1 : (x > y ? 1 : 0);
}

static float median(float *v, int n)
{
  qsort(v, (size_t)n, sizeof(float), cmpf);
  return n % 2 ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

static void step(vzState_t *st, vzOutput_t *o, vzCmd_t cmd, float acc, float gravZ, float tilt, float gyro, float vbat,
                 float thrust)
{
  vzInput_t in = { .dt = DT, .cmd = cmd, .accZ = acc, .gravZ = gravZ, .tiltDeg = tilt, .gyroMaxDps = gyro,
                   .vbat = vbat, .thrustApplied = thrust, .fromSim = false };
  vzStep(st, &in, o);
}

typedef struct {
  int takeoffs, detected, refFound, both, early, earlyFalse, late100, late250, tloIn15;
  float latSum, latMax, tloErrMin, tloErrMax;
  int spinAborts, attitudeAborts, beyondWindow, tiltedStart;
  int landings, tdFired, tdFalse, tdJolt;
  float scanS, armedS;
  float tdBeforeStopMin, tdBeforeStopMax;
  bool f16id3, f21id3;
} stats_t;

/* One take-off starting at sample i0 (first non-zero thrust). */
static void replayTakeoff(const char *name, const log_t *l, int i0, int i1, stats_t *s)
{
  /* Ground baseline from the take-off's own low-thrust samples. */
  float g[40];
  int ng = 0;
  for (int k = i0; k < i1 && ng < 40; k++) {
    if (l->thrust[k] < 20000) {
      g[ng++] = l->aUp[k];
    }
  }
  if (ng < 3) {
    return;
  }
  const float base = median(g, ng);
  /* Reference lift-off. */
  int ref = -1;
  for (int k = i0; k < i1 - 3; k++) {
    if (l->thrust[k] >= 20000 && l->aUp[k] - base > 0.025f && l->aUp[k + 1] - base > 0.025f &&
        l->aUp[k + 2] - base > 0.025f) {
      ref = k;
      break;
    }
  }
  s->takeoffs++;
  s->refFound += ref >= 0;

  vzConfig_t c;
  vzDefaultConfig(&c);
  c.rampRate = 1000.0f;   /* the pilot's thrust flew; keep our own ramp's ceiling out of the way */
  c.rampMaxS = 3.0f;
  vzState_t st;
  vzOutput_t o;
  vzInit(&st, &c);
  const float v0 = (float)l->vbat[i0] / 1000.0f;
  /* The resting attitude would trip SPIN_TILT (and the ARM guard G2) on
   * the real drone; counted, then clamped so the detector still runs. */
  s->tiltedStart += l->tilt[i0] > c.spinTiltMax;
  /* SPIN on the baseline (0.8 s), attitude as logged at the start. */
  memset(&o, 0, sizeof(o));
  for (int k = 0; k < 2000 && (k == 0 || o.phase == VZ_SPIN); k++) {
    step(&st, &o, VZ_CMD_TAKEOFF, base, l->gravZ[i0], fminf(l->tilt[i0], c.spinTiltMax), 0.0f, v0, 0.0f);
  }
  if (o.phase != VZ_RAMP) {
    s->spinAborts++;
    printf("  %s fid %d: SPIN ended %s\n", name, l->fid[i0], vzReasonName(o.reason));
    return;
  }
  int det = -1;
  float tLo = 0.0f;
  for (int k = i0; k < i1 && k < i0 + 300 && det < 0 && o.phase == VZ_RAMP; k++) {
    for (int j = 0; j < STEPS_PER_SAMPLE && det < 0; j++) {
      step(&st, &o, VZ_CMD_TAKEOFF, l->aUp[k], l->gravZ[k], l->tilt[k], l->gyro[k], (float)l->vbat[k] / 1000.0f,
           (float)l->thrust[k]);
      if (o.liftoffNow) {
        det = k;
        tLo = o.tLo;
      }
    }
  }
  if (o.phase == VZ_LAND && o.reason == VZ_R_LIFTOFF_ATTITUDE) {
    s->attitudeAborts++;
  }
  if (det < 0 && ref >= 0 && ref >= i0 + 300) {
    s->beyondWindow++;
  }
  s->detected += det >= 0;
  const float refT = ref >= 0 ? (float)(ref - i0) / 100.0f : -1.0f;
  const float detT = det >= 0 ? (float)(det - i0) / 100.0f : -1.0f;
  const int refThrust = ref >= 0 ? l->thrust[ref - 3 >= i0 ? ref - 3 : i0] : 0;
  if (det >= 0 && ref >= 0) {
    s->both++;
    const float lat = detT - refT;
    const float err = tLo / (float)refThrust - 1.0f;
    /* Earlier than the reference signature is fine near the lift-off thrust
     * (a slow ramp unloads the legs and floats in ground effect before the
     * 25 mg signature); far below it would be a ground false alarm. */
    s->early += lat < -0.0001f;
    s->earlyFalse += lat < -0.0001f && (lat < -0.30f || err < -0.15f);
    s->late100 += lat > 0.100f;
    s->late250 += lat > 0.250f;
    s->latSum += lat;
    s->latMax = fmaxf(s->latMax, lat);
    s->tloErrMin = fminf(s->tloErrMin, err);
    s->tloErrMax = fmaxf(s->tloErrMax, err);
    s->tloIn15 += fabsf(err) <= 0.15f;
    if (strcmp(name, "flight16.csv") == 0 && l->fid[i0] == 3) {
      s->f16id3 = fabsf(lat) <= 0.1f && fabsf(err) <= 0.15f;
    }
    if (strcmp(name, "flight21.csv") == 0 && l->fid[i0] == 3) {
      s->f21id3 = fabsf(lat) <= 0.1f && fabsf(err) <= 0.15f;
    }
  }
  printf("  %-13s fid %2d  ramp %6.0f/s  ref %5.2f s @%5d  det %5.2f s  T_lo %5.0f (%+5.1f %%)  %s\n", name,
         l->fid[i0], ref >= 0 && ref - 20 >= i0 ? (float)(l->thrust[ref] - l->thrust[ref - 20]) / 0.2f : 0.0f, refT,
         refThrust, detT, tLo, refThrust > 0 && det >= 0 ? (tLo / (float)refThrust - 1.0f) * 100.0f : 0.0f,
         det < 0 ? vzReasonName(o.reason) : "");
}

/* Touchdown rule on one flight segment [i0, i1): i1 is the first zero-thrust sample. */
static void replayLanding(const char *name, const log_t *l, int i0, int i1, float base, int ref, stats_t *s)
{
  /* Hover gain from the middle of the flight: median thrust x V over the samples above 30000. */
  static float kv[MAX_ROWS];
  int nk = 0;
  for (int k = i0 + (i1 - i0) / 4; k < i0 + 3 * (i1 - i0) / 4; k++) {
    if (l->thrust[k] > 30000) {
      kv[nk++] = (float)l->thrust[k] * (float)l->vbat[k] / 1000.0f;
    }
  }
  if (nk < 100) {
    return;
  }
  const float kRef = median(kv, nk);
  s->landings++;
  /* Scan from 1 s after the logged lift-off to the motor stop. */
  float lp = 0.0f, tdT = 0.0f;
  const float alpha = 0.01f / (0.01f + 1.0f / (2.0f * 3.14159265f * 20.0f));
  int firstFire = -1, firstJolt = -1;
  for (int k = i0; k < i1; k++) {
    lp += alpha * ((l->aUp[k] - base) - lp);
    const float v = (float)l->vbat[k] / 1000.0f;
    const float tHov = kRef / v;
    const float thr = (float)l->thrust[k];
    const bool ok = thr <= 0.75f * tHov;
    const float fPred = thr * v / kRef;
    tdT = (ok && lp + 1.0f > 1.3f * fPred) ? tdT + 0.01f : 0.0f;
    const bool jolt = ok && lp > 0.6f;
    if (k > ref + 100) {
      s->scanS += 0.01f;
      s->armedS += ok ? 0.01f : 0.0f;
    }
    if ((tdT >= 0.2f || jolt) && firstFire < 0 && k > ref + 100) {
      firstFire = k;
      if (jolt) {
        firstJolt = k;
      }
    }
  }
  if (firstFire >= 0) {
    const float before = (float)(i1 - firstFire) / 100.0f;
    if (before > 1.0f) {
      s->tdFalse++;   /* more than 1 s before the motors stopped: the drone was still flying */
      printf("  %-13s fid %2d: touchdown rule fired %.2f s before the motors stopped (thrust %d, K_ref %.0f)\n", name,
             l->fid[i0], before, l->thrust[firstFire], kRef);
    } else {
      s->tdFired++;
      s->tdJolt += firstJolt >= 0;
      s->tdBeforeStopMin = fminf(s->tdBeforeStopMin, before);
      s->tdBeforeStopMax = fmaxf(s->tdBeforeStopMax, before);
    }
  }
}

/* The logged idle spins (vzPhase SPIN in the autonomy files of 2026-10-12..14): the real vzStep() in SPIN fed the
 * logged world-up specific force, tilt and max(|gx|, |gy|) (each 100 Hz row held for ten steps of 1.07 ms: the
 * recorder samples every tenth loop of the ~935 Hz stabilizer, and the glue's dt is the real time), with the
 * SPIN_GYRO persistence (spinGyroHoldS 0.020 s, docs/autonomy-a2b.md 19.7) and with the one-sample rule (0): no abort
 * from these spins, and the same g0. A single 1 ms spike cannot be seen at 100 Hz; the logged peaks are printed. */
static void spinReplay(const char *dir)
{
  static const char *files[] = {
    "flight-2026-10-12-a2b-hops.csv", "flight-2026-10-12-calib1.csv", "flight-2026-10-12-calib2.csv",
    "flight-2026-10-12-evening.csv", "flight-2026-10-12-hop5s.csv", "flight-2026-10-12-mission5s.csv",
    "flight-2026-10-12-mission60.csv", "flight-2026-10-13-battfwd.csv", "flight-2026-10-13-battfwd2.csv",
    "flight-2026-10-13-interactive.csv", "flight-2026-10-13-night.csv", "flight-2026-10-13-preAB.csv",
    "flight-2026-10-13-propswap.csv", "flight-2026-10-13-pwmA1.csv", "flight-2026-10-13-pwmB60.csv",
    "flight-2026-10-13-tiger.csv", "flight-2026-10-13-tiger2.csv", "flight-2026-10-14-morning.csv",
  };
  enum { S_PH, S_ROLL, S_PITCH, S_GX, S_GY, S_AX, S_AY, S_AZ, S_VBAT, S_N };
  static const char *names[S_N] = { "vzPhase", "roll_cdeg", "pitch_cdeg", "gx_ddps", "gy_ddps", "ax_mg", "ay_mg",
                                    "az_mg", "vbat_mv" };
  static long rows[20000][S_N];
  int spins = 0, abortsNew = 0, abortsOld = 0, sameG0 = 0, ended = 0;
  float peakMax = 0.0f;
  for (unsigned fi = 0; fi < sizeof(files) / sizeof(files[0]); fi++) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, files[fi]);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
      continue;
    }
    static char line[8192];
    int idx[S_N];
    for (int c = 0; c < S_N; c++) {
      idx[c] = -1;
    }
    if (fgets(line, sizeof(line), f) == NULL) {
      fclose(f);
      continue;
    }
    int col = 0;
    for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
      for (int c = 0; c < S_N; c++) {
        if (strcmp(tok, names[c]) == 0) {
          idx[c] = col;
        }
      }
    }
    bool okCols = true;
    for (int c = 0; c < S_N; c++) {
      okCols = okCols && idx[c] >= 0;
    }
    int n = 0;
    while (okCols && n < 20000 && fgets(line, sizeof(line), f) != NULL) {
      col = 0;
      for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
        for (int c = 0; c < S_N; c++) {
          if (idx[c] == col) {
            rows[n][c] = strtol(tok, NULL, 10);
          }
        }
      }
      n++;
    }
    fclose(f);
    for (int i = 1; i < n; i++) {
      if (!(rows[i][S_PH] == 1 && rows[i - 1][S_PH] != 1)) {
        continue;
      }
      int j = i;
      while (j < n && rows[j][S_PH] == 1) {
        j++;
      }
      spins++;
      float peak = 0.0f, g0[2] = { 0.0f, 0.0f };
      vzReason_t endWhy = VZ_R_NONE;
      bool aborted[2] = { false, false }, done[2] = { false, false };
      for (int m = 0; m < 2; m++) {
        vzConfig_t c;
        vzDefaultConfig(&c);
        c.spinGyroHoldS = m ? 0.0f : 0.020f;
        vzState_t st;
        vzInit(&st, &c);
        vzOutput_t o;
        memset(&o, 0, sizeof(o));
        for (int k = i; k < j + 2 && k < n; k++) {
          const float r = (float)rows[k][S_ROLL] / 100.0f * 0.0174533f, p = (float)rows[k][S_PITCH] / 100.0f * 0.0174533f;
          const float aUp = ((float)rows[k][S_AX] * sinf(p) + (float)rows[k][S_AY] * cosf(p) * sinf(r) +
                             (float)rows[k][S_AZ] * cosf(p) * cosf(r)) / 1000.0f;
          const float gyro = fmaxf(fabsf((float)rows[k][S_GX]), fabsf((float)rows[k][S_GY])) / 10.0f;
          const float tilt = acosf(fminf(1.0f, cosf(r) * cosf(p))) * 57.29578f;
          if (m == 0 && k < j) {
            peak = fmaxf(peak, gyro);
          }
          for (int q = 0; q < 10; q++) {
            vzInput_t in = { .dt = 0.00107f, .cmd = VZ_CMD_TAKEOFF, .accZ = aUp, .gravZ = cosf(r) * cosf(p),
                             .tiltDeg = tilt, .gyroMaxDps = gyro, .vbat = (float)rows[k][S_VBAT] / 1000.0f };
            vzStep(&st, &in, &o);
            aborted[m] = aborted[m] || o.reason == VZ_R_SPIN_GYRO;
            if (!done[m] && o.phase != VZ_SPIN) {
              done[m] = true;
              g0[m] = st.g0;
              if (m == 0) {
                endWhy = o.reason;
              }
            }
          }
        }
      }
      peakMax = fmaxf(peakMax, peak);
      abortsNew += aborted[0];
      abortsOld += aborted[1];
      ended += done[0] && done[1];
      sameG0 += done[0] && done[1] && g0[0] == g0[1];
      if (getenv("SPIN_V")) {
        printf("  spin %s row %d (%d rows): end %s, peak %.1f\n", files[fi] + 7, i, j - i, vzReasonName(endWhy), peak);
      }
      if (peak > 12.0f || aborted[0] || aborted[1]) {
        printf("  spin %s row %d: peak %.1f dps (100 Hz), SPIN_GYRO %d (one-sample rule %d)\n", files[fi] + 7, i, peak,
               aborted[0], aborted[1]);
      }
    }
  }
  printf("logged idle spins: %d replayed, single-sample gyro peaks up to %.1f dps; SPIN_GYRO with the 20 ms rule %d, "
         "with the one-sample rule %d; the same g0 in %d of %d that ended the SPIN\n", spins, peakMax, abortsNew,
         abortsOld, sameG0, ended);
  CHECK(spins >= 20 && abortsNew == 0 && sameG0 == ended, "idle spins: %d replayed, %d SPIN_GYRO, g0 the same in %d of "
        "%d", spins, abortsNew, sameG0, ended);
}

int main(int argc, char **argv)
{
  if (argc < 2) {
    printf("skipped: pass the directory with flight*.csv (e.g. ../../firmware-backups/cal)\n");
    return 0;
  }
  stats_t s;
  memset(&s, 0, sizeof(s));
  s.tloErrMin = 9.0f;
  s.tloErrMax = -9.0f;
  s.tdBeforeStopMin = 99.0f;
  int files = 0;
  printf("lift-off (ref = logged signature, det = vzStep):\n");
  for (int fi = 1; fi < 100; fi++) {
    char path[1024], name[32];
    snprintf(name, sizeof(name), "flight%d.csv", fi);
    snprintf(path, sizeof(path), "%s/%s", argv[1], name);
    if (!loadCsv(path, &lg)) {
      continue;
    }
    files++;
    for (int i = 1; i < lg.n; i++) {
      if (lg.thrust[i] > 0 && lg.thrust[i - 1] == 0) {
        int j = i;
        while (j < lg.n && lg.thrust[j] > 0) {
          j++;
        }
        if (j - i >= 500) {   /* flights of at least 5 s, as in the hover analysis */
          replayTakeoff(name, &lg, i, j, &s);
          float g[40];
          int ng = 0;
          for (int k = i; k < j && ng < 40; k++) {
            if (lg.thrust[k] < 20000) {
              g[ng++] = lg.aUp[k];
            }
          }
          const float base = ng >= 3 ? median(g, ng) : 1.0f;
          int ref = -1;
          for (int k = i; k < j - 3 && ref < 0; k++) {
            if (lg.thrust[k] >= 20000 && lg.aUp[k] - base > 0.025f && lg.aUp[k + 1] - base > 0.025f &&
                lg.aUp[k + 2] - base > 0.025f) {
              ref = k;
            }
          }
          if (ng >= 3 && ref >= 0 && j < lg.n) {
            replayLanding(name, &lg, i, j, base, ref, &s);
          }
        }
        i = j;
      }
    }
  }
  if (files == 0) {
    printf("skipped: no flight*.csv with vbat_mv in %s\n", argv[1]);
    return 0;
  }
  printf("lift-off: %d take-offs in %d files; reference found %d, detected %d (both %d); SPIN aborts %d, LIFTOFF_ATTITUDE %d, past the 3 s window %d; resting tilt > 8 deg (SPIN_TILT on the drone) %d\n",
         s.takeoffs, files, s.refFound, s.detected, s.both, s.spinAborts, s.attitudeAborts, s.beyondWindow,
         s.tiltedStart);
  if (s.both > 0) {
    printf("lift-off: detection - reference mean %+.0f ms, max %+.0f ms; earlier than the reference %d, > 100 ms later %d, > 250 ms later %d; T_lo vs reference %+.1f..%+.1f %% (within 15 %%: %d)\n",
           s.latSum / (float)s.both * 1000.0f, s.latMax * 1000.0f, s.early, s.late100, s.late250,
           s.tloErrMin * 100.0f, s.tloErrMax * 100.0f, s.tloIn15);
  }
  printf("touchdown rule on manual landings: %d landings, %.0f s scanned from 1 s after lift-off to the motor stop, thrust <= 0.75 T_hov (rule armed) for %.1f s; fired in the last 1 s before the stop %d (jolt %d), fired earlier while flying %d\n",
         s.landings, s.scanS, s.armedS, s.tdFired, s.tdJolt, s.tdFalse);

  CHECK(s.detected == s.takeoffs - s.spinAborts - s.beyondWindow,
        "lift-off detected for every take-off inside the 3 s RAMP window (%d/%d)", s.detected,
        s.takeoffs - s.spinAborts - s.beyondWindow);
  CHECK(s.earlyFalse == 0, "never a ground false alarm: no detection > 300 ms or 15 %% thrust before the logged signature (%d of %d early)",
        s.earlyFalse, s.early);
  CHECK(s.f16id3, "flight16 id3 (the vertical design's example, ~35k): detected near the logged lift-off and thrust");
  CHECK(s.f21id3, "flight21 id3 (the vertical design's example, 40-45k): detected near the logged lift-off and thrust");
  CHECK(s.late250 == 0, "never more than 250 ms after the logged lift-off (%d)", s.late250);
  CHECK(s.tdFalse == 0, "the touchdown rule never fires while the drone is flying (%d)", s.tdFalse);
  spinReplay(argv[1]);
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
