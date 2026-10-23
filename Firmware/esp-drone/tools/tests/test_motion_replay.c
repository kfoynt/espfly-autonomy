/*
 * Replay of a logged interactive-hover MOVE through the real motion module
 * (components/core/crazyflie/utils/src/auto_motion.c): the tiger2 flight of
 * 2026-10-13 (firmware-backups/cal/flight-2026-10-13-tiger2.csv), MOVE
 * forward 1.00 m, which d3b3566 ended STOPPED PROGRESS in its settle after
 * ~0.85 m measured forward.
 *
 * The camera hold's measured progress is rebuilt from the recorder's flow
 * columns exactly as camera_flow.c / camera_hold.c compute it: every frame
 * the hold is engaged on (holdFlags 2 set, 4 clear) adds wFwd dt = scale
 * and wLat dt = -dxRes / 102.4, dxRes = dx - (-1.0 gx - 4.9 gy + 102.4 gz)
 * (the frame's gyro angles), from the first frame processed after the
 * MOVE's acceptance; a new hold epoch (a snap) restarts it at its next
 * frame. The scene distance D0 is the one the logged reference implies
 * (ln(D0 / (D0 - 1.00)) = the logged mvRefFwd in the settle), and the
 * recorder's clock (~7-9 % slow against the state machine's) is calibrated
 * by the logged RUN, which lasts the profile's T.
 *
 * After the logged end the log shows the drone's answer to d3b3566's stop
 * (the snap dropped the move's lean; it drifted back ~0.2 m), so from there
 * the camera keeps what it measured at that record and only the clock runs.
 *
 * Checks: the replay reproduces the logged reference; at the logged stop the
 * rebuilt sideways progress had just passed d3b3566's settle margin there
 * (0.15 m / D, the move has no sideways part) while the forward one was
 * well inside its margin (the cause); the motion module (user decision
 * 2026-10-13, evening) does not stop it, ends it SHORT or DONE at the
 * settle's end, and its status keeps the measured travel afterwards. On
 * e46bf90's auto_motion.c the last two fail (PROGRESS at the logged time;
 * the travel restarts with the snap epoch).
 *
 * The flight was flown at AUTO_MV_V_H_MAX 0.20 m/s (0.30 since 2026-10-14,
 * docs/autonomy-a2b.md 17.5): the replay rebuilds its reference from that
 * profile, so it pins the old top speed (autoMvVhMaxOverride; at 0.30 the
 * reference check fails, 0.0112 > 0.0097).
 *
 * The gym moves of 2026-10-14 (flight-2026-10-14-gym.csv, b6a54da: boot 12
 * f3's two, f4, f5; docs/autonomy-a2b.md 21.6): each stopped in its RUN
 * (D 4.2-4.7 m) while the hold's gain scale fell 2.5 -> 1. The replay fits
 * the scene distance and the recorder's clock to the logged reference
 * (forward 1.50 m, or f4's right 0.62 forward 1.13), rebuilds the camera's
 * progress as above, finds where b6a54da's guards (the radian sideways margin
 * 0.15 m + 30 % at the start distance, the forward one) trip against the
 * logged stop, and runs this build's motion module (the sideways margin in
 * metres at the distance now, the front guard, the frozen-view rule) on the
 * same progress up to the logged stop: after it the log is the drone's
 * answer to that stop. The gain scale's own change (s kept during a move,
 * 0.75 /s) cannot be replayed open loop: the closed-loop sims cover it.
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=c99 -O2 -Wall -Wextra -DAUTO_MV_HOST_VH_OVERRIDE -I$C/utils/interface tools/tests/test_motion_replay.c \
 *      $C/utils/src/auto_motion.c -lm -o /tmp/test_motion_replay && \
 *   /tmp/test_motion_replay <firmware-backups/cal>
 * Lines starting with "info:" are measured numbers, not checks.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_motion.h"

#ifndef AUTO_MV_HOST_VH_OVERRIDE
#error "build with -DAUTO_MV_HOST_VH_OVERRIDE: the tiger2 log was flown at 0.20 m/s, the gym's at 0.30"
#endif
extern float autoMvVhMaxOverride;

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define MAX_ROWS 20000   /* a 180 s recorder dump (docs/autonomy-a2b.md 19.3) */
#define MVE_SHORT 6   /* AUTO_MVE_SHORT (auto_motion.h; absent before the user decision of 2026-10-13, evening) */

enum { C_TICK, C_ROLL, C_PITCH, C_SEQ, C_DT, C_DX, C_SCALE, C_GX, C_GY, C_GZ, C_HOLD, C_AGE, C_VZH, C_MV, C_REFFWD,
       C_REFLAT, C_FID, C_N };
static const char *colName[C_N] = {"tickMs", "roll_cdeg", "pitch_cdeg", "flowSeq", "flowDt_100us", "flowDx_cpx",
                                   "flowScale_e4", "flowGx_e4", "flowGy_e4", "flowGz_e4", "holdFlags", "flowAge_ms",
                                   "vzH_mm", "mvState", "mvRefFwd_e4", "mvRefLat_e4", "flightId"};
static long rows[MAX_ROWS][C_N];
static int nRows;

static int loadCsv(const char *path)
{
  FILE *f = fopen(path, "r");
  if (!f) {
    return -1;
  }
  static char line[8192];
  int idx[C_N];
  for (int c = 0; c < C_N; c++) {
    idx[c] = -1;
  }
  if (!fgets(line, sizeof(line), f)) {
    fclose(f);
    return -1;
  }
  int col = 0;
  for (char *tok = strtok(line, ",\r\n"); tok; tok = strtok(NULL, ",\r\n"), col++) {
    for (int c = 0; c < C_N; c++) {
      if (strcmp(tok, colName[c]) == 0) {
        idx[c] = col;
      }
    }
  }
  for (int c = 0; c < C_N; c++) {
    if (idx[c] < 0) {
      fclose(f);
      return -2;
    }
  }
  while (nRows < MAX_ROWS && fgets(line, sizeof(line), f)) {
    long v[128];
    int n = 0;
    for (char *tok = strtok(line, ",\r\n"); tok && n < 128; tok = strtok(NULL, ",\r\n")) {
      v[n++] = strtol(tok, NULL, 10);
    }
    for (int c = 0; c < C_N; c++) {
      rows[nRows][c] = idx[c] < n ? v[idx[c]] : 0;
    }
    nRows++;
  }
  fclose(f);
  return nRows;
}

/* ---- the gym moves of 2026-10-14 ----------------------------------------------------------------------------------- */

/* The lagged forward reference (ln) of a MOVE (r, f cm) from D0 at profile time tau (s), as auto_motion.c plans it. */
static float refFwdAt(const autoMotion_t *m, float tau)
{
  float s, v, a;
  autoMvProfEval(&m->ph, tau - (float)AUTO_MV_CAM_LAG_MS * 0.001f, &s, &v, &a);
  const float uy = m->fwdM / m->lenM;
  return logf(m->d0 / (m->d0 - s * uy));
}

static void gymMove(int a0, int e0, int16_t rc, int16_t fc, int *nOld, int *nNewStop, int *nNewEarlier)
{
  autoMvVhMaxOverride = 0.30f;   /* b6a54da's top speed */
  /* fit D0 and the recorder's clock (ms of state machine per record) to the logged reference in the RUN */
  float bestD = 0.0f, bestK = 0.0f, bestE = 1e9f;
  for (float d = 2.5f; d <= 6.0f; d += 0.01f) {
    autoMotion_t m;
    autoMotionInit(&m);
    autoMvIn_t in;
    memset(&in, 0, sizeof(in));
    in.nowMs = 1000;
    in.flight = true;
    in.sceneDM = d;
    in.sceneTracked = true;
    in.holdApplied = true;
    in.camFrameAgeMs = 120;
    in.camHeight = true;
    in.vzH = 1.0f;
    in.hTargetM = 1.0f;
    in.airMs = 20000;
    if (autoMotionMove(&m, &in, rc, fc, 0) != AUTO_MVW_NONE) {
      continue;
    }
    for (float k = 9.5f; k <= 11.5f; k += 0.05f) {
      float e = 0.0f;
      int n = 0;
      for (int i = a0; i < e0; i++) {
        const float tau = (float)(i - a0) * k * 0.001f;
        const float dl = refFwdAt(&m, tau) - (float)rows[i][C_REFFWD] * 1e-4f;
        e += dl * dl;
        n++;
      }
      e = sqrtf(e / (float)(n ? n : 1));
      if (e < bestE) {
        bestE = e;
        bestD = d;
        bestK = k;
      }
    }
  }
  /* replay: the camera's progress from the logged frames, this build's module, and b6a54da's guards emulated */
  autoMotion_t m;
  autoMotionInit(&m);
  autoMvIn_t in;
  memset(&in, 0, sizeof(in));
  in.nowMs = 100000;
  in.flight = true;
  in.sceneDM = bestD;
  in.sceneTracked = true;
  in.holdApplied = true;
  in.camFrameAgeMs = 120;
  in.camHeight = true;
  in.vzH = (float)rows[a0][C_VZH] * 1e-3f;
  in.hTargetM = in.vzH;
  in.airMs = 20000;
  autoMvOut_t out;
  memset(&out, 0, sizeof(out));
  autoMotionStep(&m, &in, &out);
  in.holdEpochSeen = out.epoch;
  (void)autoMotionMove(&m, &in, rc, fc, 0);
  const float latM = rc * 0.01f, dl = m.dLat0;
  const float margLatOld = (AUTO_MV_PROG_MARGIN_M + AUTO_MV_PROG_MARGIN_REL * fabsf(latM)) / dl;
  uint32_t camEpoch = out.epoch;
  const uint32_t epoch0 = m.epoch;   /* the move's epoch (the camera's progress counts from its first frame) */
  float pLat = 0.0f, pFwd = 0.0f, acc = 0.0f, oldTau = 0.0f;
  long lastSeq = rows[a0][C_SEQ];
  int oldRec = -1, newRec = -1;
  float newLat = 0.0f, newFwd = 0.0f, newRefLat = 0.0f, newRefFwd = 0.0f, lastRefFwd = 0.0f, lastRefLat = 0.0f;
  const char *oldWhy = "-";
  float latAtOld = 0.0f, fwdAtOld = 0.0f;
  for (int i = a0; i <= e0; i++) {
    const long *r = rows[i];
    if (r[C_SEQ] != lastSeq) {
      lastSeq = r[C_SEQ];
      if (out.epoch != camEpoch && camEpoch != epoch0) {
        camEpoch = out.epoch;
        pLat = pFwd = 0.0f;
      } else if (out.epoch != camEpoch && oldRec >= 0) {
        camEpoch = out.epoch;   /* (both guards done: the new epoch's progress from 0) */
        pLat = pFwd = 0.0f;
      }
      const bool engaged = (r[C_HOLD] & 2) && !(r[C_HOLD] & 4);
      if (engaged && r[C_DT] > 0) {
        const float dx = (float)r[C_DX] * 0.01f;
        const float gx = (float)r[C_GX] * 1e-4f, gy = (float)r[C_GY] * 1e-4f, gz = (float)r[C_GZ] * 1e-4f;
        const float dxRes = dx - (-1.0f * gx - 4.9f * gy + 102.4f * gz);
        pFwd += (float)r[C_SCALE] * 1e-4f;
        pLat += -dxRes / 102.4f;
      }
      in.holdEpochSeen = camEpoch;
      in.progLat = pLat;
      in.progFwd = pFwd;
    }
    in.holdApplied = (r[C_HOLD] & 2) != 0;
    in.holdCoasting = (r[C_HOLD] & 4) != 0;
    in.camFrameAgeMs = (uint32_t)r[C_AGE];
    in.tiltDeg = sqrtf((float)(r[C_ROLL] * r[C_ROLL] + r[C_PITCH] * r[C_PITCH])) * 0.01f;
    in.vzH = (float)r[C_VZH] * 1e-3f;
    acc += bestK;
    while (acc >= 1.0f) {
      acc -= 1.0f;
      in.nowMs++;
      in.airMs++;
      const bool was = autoMotionActive(&m);
      autoMotionStep(&m, &in, &out);
      if (was && !autoMotionActive(&m) && newRec < 0) {
        newRec = i;
        newLat = pLat;
        newFwd = pFwd;
        newRefLat = out.refLat;
        newRefFwd = m.fwdTgt;
      }
      if (autoMotionActive(&m)) {
        lastRefFwd = out.refFwd;
        lastRefLat = out.refLat;
      }
      /* b6a54da's guard in RUN on the same inputs, against the move's own reference (whatever this build did): the
       * radian sideways margin, the forward one (unchanged) */
      oldTau += 0.001f;
      if (oldRec < 0 && oldTau < m.ph.T && camEpoch == epoch0) {
        float sR, vR, aR;
        autoMvProfEval(&m.ph, oldTau - (float)AUTO_MV_CAM_LAG_MS * 0.001f, &sR, &vR, &aR);
        const float rLatOld = sR * (m.latM / m.lenM) / m.dLat0, rFwdOld = refFwdAt(&m, oldTau);
        if (fabsf(pLat - rLatOld) > margLatOld) {
          oldRec = i;
          oldWhy = "sideways";
        } else if (fabsf(pFwd - rFwdOld) > m.margFwd) {
          oldRec = i;
          oldWhy = pFwd < rFwdOld ? "forward, behind" : "forward, ahead";
        }
        if (oldRec >= 0) {
          latAtOld = pLat;
          fwdAtOld = pFwd;
        }
      }
    }
  }
  const float dNow = m.dLat0 - bestD * (1.0f - expf(-fwdAtOld));
  printf("info: gym f%ld MOVE right %d forward %d: D0 %.2f m, the recorder at %.2f ms per record (reference rms %.4f); "
         "logged stop at record %d; b6a54da's guard trips at %d (%s: measured sideways %+.4f = %.3f m at the start "
         "distance, %.3f m at the distance now; forward %.4f of the reference %.4f); this build's: %s at %d (%s)\n",
         rows[a0][C_FID], rc, fc, bestD, bestK, bestE, e0, oldRec, oldWhy, latAtOld, latAtOld * dl, latAtOld * dNow,
         fwdAtOld, refFwdAt(&m, 0.0f), newRec < 0 ? "no stop up to the logged one" : autoMvEndName(m.end), newRec,
         autoMvStateName(m.state));
  if (newRec >= 0) {
    printf("info:   this build's stop (%s): measured sideways %+.4f, forward %.4f; the reference then sideways %+.4f, "
           "forward %.4f; the forward margin %.4f (unchanged), progFwdMax %.4f\n", autoMvEndName(m.end), newLat, newFwd,
           lastRefLat, lastRefFwd, m.margFwd, m.progFwdMax);
  } else {
    printf("info:   at the logged stop: measured sideways %+.4f (%.3f m at the distance now), forward %.4f against the "
           "reference %+.4f / %.4f\n", pLat, pLat * (m.dLat0 - bestD * (1.0f - expf(-pFwd))), pFwd, lastRefLat,
           lastRefFwd);
  }
  (void)newRefLat;
  (void)newRefFwd;
  const bool sideways = oldRec >= 0 && strcmp(oldWhy, "sideways") == 0 && oldRec >= e0 - 25 && oldRec <= e0 + 2;
  *nOld += sideways;
  *nNewStop += sideways && newRec >= 0 && newRec <= e0;   /* this build stopping one b6a54da stopped sideways */
  *nNewEarlier += newRec >= 0 && (oldRec < 0 || newRec < oldRec - 2);
}

static void testGym(const char *dir)
{
  char path[1024];
  snprintf(path, sizeof(path), "%s/flight-2026-10-14-gym.csv", dir);
  nRows = 0;
  if (loadCsv(path) < 100) {
    printf("FAIL: cannot read %s\n", path);
    failures++;
    return;
  }
  /* the MOVEs: kind MOVE first seen after none, its STOPPED / DONE record */
  int nMoves = 0, nOld = 0, nNew = 0, nEarlier = 0;
  int a0 = -1;
  for (int i = 1; i < nRows; i++) {
    const int st = (int)(rows[i][C_MV] & 15), kind = (int)(rows[i][C_MV] >> 4);
    const int pst = (int)(rows[i - 1][C_MV] & 15), pkind = (int)(rows[i - 1][C_MV] >> 4);
    if (kind == AUTO_MVK_MOVE && st == AUTO_MV_RUN && !(pkind == AUTO_MVK_MOVE && pst >= 1 && pst <= 3)) {
      a0 = i;
    }
    if (a0 >= 0 && kind == AUTO_MVK_MOVE && (st == AUTO_MV_STOPPED || st == AUTO_MV_DONE)) {
      /* f4 is the diagonal (the logged sideways reference), the others forward 1.50 m */
      bool lat = false;
      for (int j = a0; j < i; j++) {
        lat = lat || rows[j][C_REFLAT] != 0;
      }
      gymMove(a0, i, lat ? 62 : 0, lat ? 113 : 150, &nOld, &nNew, &nEarlier);
      nMoves++;
      a0 = -1;
    }
  }
  printf("info: gym moves: %d replayed; b6a54da's sideways margin at the logged stop %d of them; this build stopping "
         "those %d; stopping one earlier than b6a54da's guards would %d\n", nMoves, nOld, nNew, nEarlier);
  CHECK(nMoves == 4 && nOld >= 2, "gym: the logged stops of f3's first move and f5 reproduced by b6a54da's sideways "
        "margin (0.15 m at the start distance) on the rebuilt camera progress (%d of %d)", nOld, nMoves);
  CHECK(nNew == 0, "gym: this build's guards (0.25 m at the distance now) do not stop them (%d)", nNew);
  CHECK(nEarlier == 0, "gym: this build never stops a move earlier than b6a54da's guards would (%d)", nEarlier);
  autoMvVhMaxOverride = 0.20f;
}

int main(int argc, char **argv)
{
  if (argc < 2) {
    printf("usage: %s <firmware-backups/cal>\n", argv[0]);
    return 2;
  }
  autoMvVhMaxOverride = 0.20f;   /* tiger2: flown at 0.20 m/s */
  char path[1024];
  snprintf(path, sizeof(path), "%s/flight-2026-10-13-tiger2.csv", argv[1]);
  if (loadCsv(path) < 100) {
    printf("FAIL: cannot read %s\n", path);
    return 1;
  }
  /* the MOVE: acceptance (kind MOVE first seen), the settle, the logged end */
  int a0 = -1, s0 = -1, e0 = -1;
  for (int i = 0; i < nRows; i++) {
    const int st = (int)(rows[i][C_MV] & 15), kind = (int)(rows[i][C_MV] >> 4);
    if (a0 < 0 && kind == AUTO_MVK_MOVE) {
      a0 = i;
    }
    if (a0 >= 0 && s0 < 0 && kind == AUTO_MVK_MOVE && st == AUTO_MV_SETTLE) {
      s0 = i;
    }
    if (s0 >= 0 && e0 < 0 && kind == AUTO_MVK_MOVE && (st == AUTO_MV_STOPPED || st == AUTO_MV_DONE)) {
      e0 = i;
    }
  }
  CHECK(a0 > 0 && s0 > a0 && e0 > s0, "the logged MOVE: accepted at record %d, SETTLE %d, ended %d", a0, s0, e0);
  if (!(a0 > 0 && s0 > a0 && e0 > s0)) {
    printf("%d checks, %d failures\n", checks, failures);
    return 1;
  }
  const float fwdTgtLog = (float)rows[s0][C_REFFWD] * 1e-4f;
  const float d0 = 1.00f / (1.0f - expf(-fwdTgtLog));
  printf("info: logged: accepted at %.2f s, SETTLE at %.2f s, %s at %.2f s (recorder clock); the reference "
         "ln-zoom %.4f: D0 %.3f m\n", (rows[a0][C_TICK] - rows[0][C_TICK]) * 1e-3,
         (rows[s0][C_TICK] - rows[0][C_TICK]) * 1e-3, (rows[e0][C_MV] & 15) == AUTO_MV_STOPPED ? "STOPPED" : "DONE",
         (rows[e0][C_TICK] - rows[0][C_TICK]) * 1e-3, fwdTgtLog, d0);

  autoMotion_t m;
  autoMotionInit(&m);
  autoMvIn_t in;
  memset(&in, 0, sizeof(in));
  in.nowMs = 100000;
  in.flight = true;
  in.sceneDM = d0;
  in.sceneTracked = true;
  in.holdApplied = true;
  in.camFrameAgeMs = 120;
  in.camHeight = true;
  in.vzH = (float)rows[a0][C_VZH] * 1e-3f;
  in.hTargetM = in.vzH;
  in.airMs = 9000;
  autoMvOut_t out;
  memset(&out, 0, sizeof(out));
  autoMotionStep(&m, &in, &out);   /* the HOVER step before the command */
  in.holdEpochSeen = out.epoch;
  const autoMvWhy_t why = autoMotionMove(&m, &in, 0, 100, 0);
  CHECK(why == AUTO_MVW_NONE && fabsf(m.fwdTgt - fwdTgtLog) < 2e-4f, "MOVE forward 1.00 at D0 %.3f: accepted, target "
        "%.4f (logged %.4f)", d0, m.fwdTgt, fwdTgtLog);
  const float msPerRec = m.ph.T * 1000.0f / (float)(s0 - a0);
  printf("info: profile T %.2f s over %d records: %.2f ms per record (recorder clock %.1f %% slow)\n", m.ph.T, s0 - a0,
         msPerRec, (msPerRec / 10.0f - 1.0f) * 100.0f);
  const float margLatOld = AUTO_MV_PROG_MARGIN_M / m.dLat0;   /* d3b3566's sideways margin of this move */
  const float tgt0 = m.fwdTgt, margF = m.margFwd;

  /* the camera: progress since the epoch it has applied */
  uint32_t camEpoch = out.epoch;
  float pLat = 0.0f, pFwd = 0.0f;
  long lastSeq = rows[a0][C_SEQ];
  float acc = 0.0f, refErrMax = 0.0f, pLatAtEnd = 0.0f, pFwdAtEnd = 0.0f;
  int endRec = -1, progressRec = -1;
  autoMvEnd_t end = AUTO_MVE_NONE;
  float travelR = 0.0f, travelF = 0.0f;
  bool travelKept = true;
  int afterEnd = 0;
  for (int i = a0; i < nRows && afterEnd < 2000; i++) {
    /* After the logged end the log shows the drone's answer to d3b3566's stop (its snap dropped the move's
     * lean and the drone drifted back), not to this code: from there the camera keeps what it measured at the
     * logged end's record (the frame that stopped it; the drone holding where it was) and only the clock runs. */
    const long *r = rows[i < e0 ? i : e0];
    if (i <= e0 && r[C_SEQ] != lastSeq) {   /* a frame processed since the previous record */
      lastSeq = r[C_SEQ];
      if (out.epoch != camEpoch) {   /* the hold takes a new epoch at its next frame: positions from 0 */
        camEpoch = out.epoch;
        pLat = pFwd = 0.0f;
      }
      const bool engaged = (r[C_HOLD] & 2) && !(r[C_HOLD] & 4);
      if (engaged && r[C_DT] > 0) {
        const float dx = (float)r[C_DX] * 0.01f;
        const float gx = (float)r[C_GX] * 1e-4f, gy = (float)r[C_GY] * 1e-4f, gz = (float)r[C_GZ] * 1e-4f;
        const float dxRes = dx - (-1.0f * gx - 4.9f * gy + 102.4f * gz);
        pFwd += (float)r[C_SCALE] * 1e-4f;
        pLat += -dxRes / 102.4f;
      }
      in.holdEpochSeen = camEpoch;
      in.progLat = pLat;
      in.progFwd = pFwd;
    }
    in.holdApplied = (r[C_HOLD] & 2) != 0;
    in.holdCoasting = (r[C_HOLD] & 4) != 0;
    in.camFrameAgeMs = (uint32_t)r[C_AGE];
    in.tiltDeg = sqrtf((float)(r[C_ROLL] * r[C_ROLL] + r[C_PITCH] * r[C_PITCH])) * 0.01f;
    in.vzH = (float)r[C_VZH] * 1e-3f;
    acc += msPerRec;
    const bool wasActive = autoMotionActive(&m);
    while (acc >= 1.0f) {
      acc -= 1.0f;
      in.nowMs++;
      in.airMs++;
      autoMotionStep(&m, &in, &out);
    }
    if (i < e0 && (int)(r[C_MV] & 15) != AUTO_MV_STOPPED) {
      refErrMax = fmaxf(refErrMax, fabsf(out.refFwd - (float)r[C_REFFWD] * 1e-4f));
    }
    if (i == e0) {
      pLatAtEnd = pLat;
      pFwdAtEnd = pFwd;
    }
    if (wasActive && !autoMotionActive(&m)) {
      endRec = i;
      end = m.end;
      travelR = out.progRightM;
      travelF = out.progFwdM;
      if (m.end == AUTO_MVE_PROGRESS) {
        progressRec = i;
      }
    } else if (endRec >= 0) {
      afterEnd++;
      travelKept = travelKept && out.progRightM == travelR && out.progFwdM == travelF;
    }
  }
  CHECK(refErrMax < 3e-3f, "the replayed reference follows the logged one within %.4f (ln-zoom)", refErrMax);
  printf("info: at the logged stop (record %d): rebuilt progress sideways %+.4f (%.3f m), forward %.4f (%.3f m): "
         "the old sideways margin %.4f (0.15 m / D), the forward margin %.4f from the target %.4f\n", e0, pLatAtEnd,
         pLatAtEnd * m.dLat0, pFwdAtEnd, d0 * (1.0f - expf(-pFwdAtEnd)), margLatOld, margF, tgt0);
  CHECK(fabsf(pLatAtEnd) > margLatOld && fabsf(pLatAtEnd) < 1.03f * margLatOld &&
        fabsf(pFwdAtEnd - tgt0) < 0.5f * margF,
        "the cause: at the logged stop the sideways progress had just passed 0.15 m / D (%.4f of %.4f) while the "
        "forward one was well within its margin (%.4f of %.4f)", fabsf(pLatAtEnd), margLatOld,
        fabsf(pFwdAtEnd - tgt0), margF);
  printf("info: replayed end: %s at record %d (%.2f s recorder clock), travel right %+.3f m, forward %.3f m\n",
         autoMvEndName(end), endRec, endRec >= 0 ? (rows[endRec][C_TICK] - rows[0][C_TICK]) * 1e-3 : -1.0, travelR,
         travelF);
  if (progressRec >= 0) {
    printf("info: PROGRESS at record %d, the log's at %d\n", progressRec, e0);
  }
  CHECK(end != AUTO_MVE_PROGRESS && (end == AUTO_MVE_DONE || (int)end == MVE_SHORT) && m.state == AUTO_MV_DONE,
        "the settle does not stop it: it ends %s (state %s) at the settle's end", autoMvEndName(end),
        autoMvStateName(m.state));
  CHECK(travelF > 0.80f && travelF < 0.92f && travelR < -0.10f && travelR > -0.25f && travelKept && afterEnd > 100,
        "the status keeps the measured travel after the end: forward %.3f m, right %+.3f m (%d steps after)",
        travelF, travelR, afterEnd);
  testGym(argv[1]);
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
