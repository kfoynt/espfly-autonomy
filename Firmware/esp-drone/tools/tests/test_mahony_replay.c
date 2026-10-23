/*
 * Replay of every logged flight through the attitude filter (modules/src/sensfusion6.c, included) with and
 * without the in-flight bias learning (twoKiAir 0, the filter of 14bcfc6, against 0.040; docs/autonomy-a2b.md
 * section 19.2), on the recorder's 100 Hz gyro and accelerometer (the firmware's own integration time: recorder
 * ticks, 1 ms per stabilizer loop).
 *
 * Build and run from Firmware/esp-drone (skipped without the CSV directory):
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/modules/interface \
 *      -Icomponents/core/crazyflie/utils/interface tools/tests/test_mahony_replay.c -lm \
 *      -o /tmp/test_mahony_replay && /tmp/test_mahony_replay ../../firmware-backups/cal [-v]
 *
 * Files: flight-2026-10-1[0-4]*.csv, every flight with >= 2 s above lift-off thrust. Checks:
 *   (0) the 14bcfc6 filter replayed tracks the logged attitude (mean |error| airborne < 0.2 deg: 0.01-0.18);
 *   (a) hovers >= 10 s: the innovation (accelerometer tilt minus the estimate) over the hover's last 3 s drops
 *       wherever it was over 0.3 deg; 2026-10-14 flight 4 pitch (1.18 over its last 3 s, up to 1.53 in the
 *       forensic's windows) -> <= 0.8 deg;
 *   (b) no 1 s window of TAKEOFF, a MOVE or YAW, LAND or the 1 s after a contact gets worse against the
 *       accelerometer tilt by more than 0.2 deg, over its unaccelerated rows (| |a| - 1 | <= 0.03 g);
 *   (c) |learned bias| <= 1.0 deg/s on every autonomy flight that ended in a normal LAND (no KILLED, not
 *       COLLISION or PALM);
 *   (i) twoKiAir 0 replays bit-identically to 14bcfc6's filter: the hash of every row's quaternion over
 *       2026-10-14 morning flight 4, pinned from 14bcfc6's sensfusion6.c (build with -DSF6_OLD
 *       -DSF6_SRC='"<path of the old file>"' to print it).
 */
#include <dirent.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define __LOG_H__
#define __PARAM_H__
#define LOG_GROUP_START(x)
#define LOG_GROUP_STOP(x)
#define LOG_ADD(t, n, p)
#define PARAM_GROUP_START(x)
#define PARAM_GROUP_STOP(x)
#define PARAM_ADD(t, n, p)
#define long int32_t
#ifndef SF6_SRC
#define SF6_SRC "../../components/core/crazyflie/modules/src/sensfusion6.c"
#endif
#include SF6_SRC
#undef long

static int failures = 0;
static int checks = 0;
static int verbose = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define MAX_ROWS 20000
enum { C_TICK, C_FID, C_THRUST, C_ROLL, C_PITCH, C_GX, C_GY, C_GZ, C_AX, C_AY, C_AZ, C_AUTO, C_REASON, C_MV, C_N };
static const char *colNames[C_N] = { "tickMs", "flightId", "thrust", "roll_cdeg", "pitch_cdeg", "gx_ddps", "gy_ddps",
                                     "gz_ddps", "ax_mg", "ay_mg", "az_mg", "autoState", "autoReason", "mvState" };
static int32_t rows[MAX_ROWS][C_N];
static bool hasCol[C_N];
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
    hasCol[c] = idx[c] >= 0;
    if (!hasCol[c] && c < C_AUTO) {
      fclose(f);
      return false;
    }
  }
  nRows = 0;
  while (nRows < MAX_ROWS && fgets(line, sizeof(line), f) != NULL) {
    memset(rows[nRows], 0, sizeof(rows[nRows]));
    col = 0;
    for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
      for (int c = 0; c < C_N; c++) {
        if (idx[c] == col) {
          rows[nRows][c] = (int32_t)strtol(tok, NULL, 10);
        }
      }
    }
    nRows++;
  }
  fclose(f);
  return true;
}

/* One replay's per-row outputs. */
static float estR[2][MAX_ROWS], estP[2][MAX_ROWS], learnedMax[2];
static float learnedEnd[2][3];

static double accPitch(int i)
{
  const double ax = rows[i][C_AX], ay = rows[i][C_AY], az = rows[i][C_AZ];
  const double n = sqrt(ax * ax + ay * ay + az * az);
  return n > 0.0 ? asin(fmax(-1.0, fmin(1.0, ax / n))) * 180.0 / M_PI : 0.0;
}

static double accRoll(int i)
{
  return atan2((double)rows[i][C_AY], (double)rows[i][C_AZ]) * 180.0 / M_PI;
}

static double accNorm(int i)
{
  const double ax = rows[i][C_AX], ay = rows[i][C_AY], az = rows[i][C_AZ];
  return sqrt(ax * ax + ay * ay + az * az) / 1000.0;
}

static bool airRow(int i)
{
  return rows[i][C_THRUST] > 30000;
}

/* The filter over rows [i0, i1) from the logged attitude of row i0; k: 0 = twoKiAir 0, 1 = 0.040. Returns the
 * hash of the quaternion of every row. */
static uint64_t runFilter(int i0, int i1, int k)
{
#ifndef SF6_OLD
  sensfusion6SetKiAir(k ? 0.040f : 0.0f);
#endif
  const double r0 = rows[i0][C_ROLL] / 100.0 * M_PI / 180.0, p0 = rows[i0][C_PITCH] / 100.0 * M_PI / 180.0;
  /* pitch = asin(gravX) = asin(2 (qx qz - qw qy)): a nose-up pitch is a negative rotation about y */
  const double cr = cos(r0 / 2), sr = sin(r0 / 2), cp = cos(-p0 / 2), sp = sin(-p0 / 2);
  qw = (float)(cr * cp);
  qx = (float)(sr * cp);
  qy = (float)(cr * sp);
  qz = (float)(-sr * sp);
  integralFBx = integralFBy = integralFBz = 0.0f;
  learnedMax[k] = 0.0f;
  uint64_t h = 0xCBF29CE484222325ull;
  for (int i = i0; i < i1; i++) {
    const float dt = i > i0 ? (float)(rows[i][C_TICK] - rows[i - 1][C_TICK]) * 0.001f : 0.01f;
#ifndef SF6_OLD
    sensfusion6SetAirborne(airRow(i));
#endif
    sensfusion6UpdateQ(rows[i][C_GX] / 10.0f, rows[i][C_GY] / 10.0f, rows[i][C_GZ] / 10.0f, rows[i][C_AX] / 1000.0f,
                       rows[i][C_AY] / 1000.0f, rows[i][C_AZ] / 1000.0f, dt > 0.0f && dt < 0.1f ? dt : 0.01f);
    float r, p, y;
    sensfusion6GetEulerRPY(&r, &p, &y);
    estR[k][i] = r;
    estP[k][i] = p;
    const float q[4] = { qw, qx, qy, qz };
    const uint8_t *b = (const uint8_t *)q;
    for (size_t j = 0; j < sizeof(q); j++) {
      h ^= b[j];
      h *= 0x100000001B3ull;
    }
#ifndef SF6_OLD
    float lb[3];
    sensfusion6GetLearnedBiasDps(lb);
    learnedMax[k] = fmaxf(learnedMax[k], fmaxf(fabsf(lb[0]), fabsf(lb[1])));
    memcpy(learnedEnd[k], lb, sizeof(lb));
#endif
  }
#ifndef SF6_OLD
  sensfusion6SetAirborne(false);
#endif
  return h;
}

#define SF6_REPLAY_HASH_14BCFC6 0xA09BF594E8D71487ull   /* 2026-10-14 morning flight 4 through 14bcfc6's filter */

typedef struct {
  int flights, hovers, hoversHigh, hoversDropped, windows, normalLands;
  double worstWindow;
  char worstName[160];
  double worstLearned;
} stats_t;

static stats_t st;

static void flight(const char *name, int i0, int i1)
{
  int nAir = 0;
  for (int i = i0; i < i1; i++) {
    nAir += airRow(i);
  }
  if (nAir < 200) {
    return;
  }
  const int fid = rows[i0][C_FID];
  const uint64_t h0 = runFilter(i0, i1, 0);
#ifdef SF6_OLD
  if (strstr(name, "2026-10-14-morning") && fid == 4) {
    printf("replay hash (this filter) %s flight %d: 0x%016llX\n", name, fid, (unsigned long long)h0);
  }
  return;
#else
  if (strstr(name, "2026-10-14-morning") && fid == 4) {
    printf("replay hash %s flight %d, twoKiAir 0: 0x%016llX (14bcfc6 0x%016llX)\n", name, fid, (unsigned long long)h0,
           (unsigned long long)SF6_REPLAY_HASH_14BCFC6);
    CHECK(h0 == SF6_REPLAY_HASH_14BCFC6, "twoKiAir 0 replays 14bcfc6's filter bit for bit");
  }
  runFilter(i0, i1, 1);
  st.flights++;

  /* (0) the old filter tracks the logged attitude */
  double e0 = 0.0;
  for (int i = i0; i < i1; i++) {
    if (airRow(i)) {
      e0 += fabs(estP[0][i] - rows[i][C_PITCH] / 100.0) + fabs(estR[0][i] - rows[i][C_ROLL] / 100.0);
    }
  }
  e0 /= 2.0 * nAir;
  if (verbose || e0 >= 0.2) {
    printf("%s flight %d: replayed 14bcfc6 filter vs the logged attitude: mean |error| %.3f deg\n", name, fid, e0);
  }
  CHECK(e0 < 0.2, "%s flight %d: the replay does not track the log (%.3f deg)", name, fid, e0);

  /* (a) hovers >= 10 s */
  for (int i = i0; i < i1;) {
    if (!(hasCol[C_AUTO] && rows[i][C_AUTO] == 3 && airRow(i))) {
      i++;
      continue;
    }
    int j = i;
    while (j < i1 && rows[j][C_AUTO] == 3 && airRow(j)) {
      j++;
    }
    if (rows[j - 1][C_TICK] - rows[i][C_TICK] >= 10000) {
      st.hovers++;
      double inP[2] = { 0, 0 }, inR[2] = { 0, 0 };
      int n = 0;
      for (int k = j - 1; k >= i && rows[j - 1][C_TICK] - rows[k][C_TICK] < 3000; k--) {
        for (int m = 0; m < 2; m++) {
          inP[m] += accPitch(k) - estP[m][k];
          inR[m] += accRoll(k) - estR[m][k];
        }
        n++;
      }
      for (int m = 0; m < 2; m++) {
        inP[m] /= n;
        inR[m] /= n;
      }
      const double dur = (rows[j - 1][C_TICK] - rows[i][C_TICK]) / 1000.0;
      printf("%s flight %d: HOVER %.1f s, innovation at its end pitch %+.2f -> %+.2f, roll %+.2f -> %+.2f deg "
             "(learned at the flight's end %+.2f %+.2f deg/s)\n", name, fid, dur, inP[0], inP[1], inR[0], inR[1],
             learnedEnd[1][0], learnedEnd[1][1]);
      for (int ax = 0; ax < 2; ax++) {
        const double o = fabs(ax ? inR[0] : inP[0]), nw = fabs(ax ? inR[1] : inP[1]);
        if (o > 0.3) {
          st.hoversHigh++;
          st.hoversDropped += nw < o;
          CHECK(nw < o, "%s flight %d: the %s innovation does not drop: %.2f -> %.2f", name, fid, ax ? "roll" : "pitch",
                o, nw);
        }
      }
      if (strstr(name, "2026-10-14-morning") && fid == 4) {
        CHECK(fabs(inP[0]) > 1.0 && fabs(inP[1]) <= 0.8, "flight 4 pitch innovation %.2f -> %.2f (<= 0.8)", inP[0],
              inP[1]);
      }
    }
    i = j;
  }

  /* (b) 1 s windows of TAKEOFF, MOVE / YAW, LAND and after contacts, over the unaccelerated rows */
  for (int i = i0; i + 1 < i1;) {
    int cls = 0;   /* 1 TAKEOFF, 2 motion, 3 LAND, 4 contact */
    bool contact = false;
    for (int k = i; k < i1 && rows[k][C_TICK] - rows[i][C_TICK] < 1000; k++) {
      contact = contact || accNorm(k) > 1.6 || abs(rows[k][C_GX]) > 2500 || abs(rows[k][C_GY]) > 2500;
    }
    if (hasCol[C_AUTO]) {
      const int a = rows[i][C_AUTO], mv = rows[i][C_MV] & 15;
      cls = a == 2 ? 1 : (a == 3 && mv >= 1 && mv <= 3) ? 2 : a == 4 ? 3 : 0;
    }
    if (contact) {
      cls = 4;
    }
    int j = i;
    while (j < i1 && rows[j][C_TICK] - rows[i][C_TICK] < 1000) {
      j++;
    }
    if (cls != 0) {
      double d[2] = { 0, 0 };
      int n = 0;
      for (int k = i; k < j; k++) {
        if (fabs(accNorm(k) - 1.0) <= 0.03 && airRow(k)) {
          for (int m = 0; m < 2; m++) {
            d[m] += fabs(accPitch(k) - estP[m][k]) + fabs(accRoll(k) - estR[m][k]);
          }
          n++;
        }
      }
      if (n >= 20) {
        st.windows++;
        const double worse = (d[1] - d[0]) / n;
        if (worse > st.worstWindow) {
          static const char *cn[] = { "", "TAKEOFF", "MOVE/YAW", "LAND", "contact" };
          st.worstWindow = worse;
          snprintf(st.worstName, sizeof(st.worstName), "%s flight %d %s at tick %d", name, fid, cn[cls],
                   (int)rows[i][C_TICK]);
        }
        CHECK(worse <= 0.2, "%s flight %d tick %d (class %d): %.2f deg worse against the accelerometer", name, fid,
              (int)rows[i][C_TICK], cls, worse);
      }
    }
    i = j;
  }

  /* (c) the learned bias on flights that ended in a normal LAND */
  if (hasCol[C_AUTO]) {
    bool killed = false, landed = false;
    int reason = 0;
    for (int i = i0; i < i1; i++) {
      killed = killed || rows[i][C_AUTO] == 5;
      if (!landed && rows[i][C_AUTO] == 4) {
        landed = true;
        reason = rows[i][C_REASON];
      }
    }
    if (landed && !killed && reason != 42 && reason != 43) {
      st.normalLands++;
      st.worstLearned = fmax(st.worstLearned, learnedMax[1]);
      if (verbose) {
        printf("%s flight %d: normal LAND (reason %d), |learned| max %.2f deg/s\n", name, fid, reason, learnedMax[1]);
      }
      CHECK(learnedMax[1] <= 1.0f, "%s flight %d: learned bias %.2f deg/s > 1.0", name, fid, learnedMax[1]);
    }
  }
#endif
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
  DIR *d = dir ? opendir(dir) : NULL;
  if (d == NULL) {
    printf("replay skipped: pass the directory with flight-2026-10-1*.csv (../../firmware-backups/cal)\n");
    printf("%d checks, %d failures\n", checks, failures);
    return 0;
  }
  char names[64][256];
  int nf = 0;
  struct dirent *e;
  while ((e = readdir(d)) != NULL && nf < 64) {
    if (strncmp(e->d_name, "flight-2026-10-1", 16) == 0 && e->d_name[16] >= '0' && e->d_name[16] <= '4' &&
        strstr(e->d_name, ".csv") != NULL) {
      snprintf(names[nf++], sizeof(names[0]), "%s", e->d_name);
    }
  }
  closedir(d);
  qsort(names, (size_t)nf, sizeof(names[0]), (int (*)(const void *, const void *))strcmp);
  int files = 0;
  for (int f = 0; f < nf; f++) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, names[f]);
    if (!loadCsv(path)) {
      continue;
    }
    files++;
    for (int i = 0; i < nRows;) {
      int j = i + 1;
      while (j < nRows && rows[j][C_FID] == rows[i][C_FID] && rows[j][C_TICK] - rows[j - 1][C_TICK] <= 200) {
        j++;
      }
      flight(names[f], i, j);
      i = j;
    }
  }
#ifndef SF6_OLD
  printf("replayed %d files, %d flights: %d hovers >= 10 s, %d of %d innovations over 0.3 deg dropped; %d windows of "
         "TAKEOFF / MOVE / YAW / LAND / contacts, worst %+.3f deg (%s); %d normal LANDs, |learned| max %.2f deg/s\n",
         files, st.flights, st.hovers, st.hoversDropped, st.hoversHigh, st.windows, st.worstWindow, st.worstName,
         st.normalLands, st.worstLearned);
  CHECK(st.flights >= 20 && st.hovers >= 5 && st.windows >= 50, "coverage: %d flights, %d hovers, %d windows",
        st.flights, st.hovers, st.windows);
#endif
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
