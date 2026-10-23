/* The 2026-10-22 table flight replayed through the hold's HOVER scene gate (camera_hold.h camHoldSceneOk) and the
 * motion controller, native profile: the gate's residual limit as a setting (NVS holdres, 2026-10-22). Every frame of
 * the recorder log (firmware-backups/cal/flight-2026-10-22-table.csv) goes through the gate in order, hover = the
 * log's autoState HOVER; a frame the flow itself refused (flowOk 0) or the gate refused is a lost view until the next
 * accepted frame; the lost intervals of each forward MOVE then drive the motion rig (test_auto_motion.c, included for
 * its rig) at this build's 323 ms VIEW_FAST timer. dxRes is the log's raw dx (the rotation model's share is not in the
 * log; the move frames' |dx| is at most 0.91 px against the 2.0 px limit at 90 ms).
 *
 *   cc -std=c99 -O2 -Wall -Wextra -DESPFLY_NATIVE_VGA=1 -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_scene_gate_replay.c components/core/crazyflie/utils/src/auto_motion.c \
 *      components/core/crazyflie/utils/src/camera_hold.c -lm -o /tmp/t && /tmp/t <firmware-backups/cal dir>
 */
#ifndef ESPFLY_NATIVE_VGA
#error "build with -DESPFLY_NATIVE_VGA=1: the native camera profile"
#endif
#define main legacy_motion_tests_main
#include "test_auto_motion.c"
#undef main
#include <stdlib.h>
#include "camera_hold.h"

#define MAX_ROWS 9000
#define MAX_FRAMES 1200
enum { C_TICK, C_SEQ, C_OK, C_RES, C_DT, C_DX, C_SCALE, C_AUTO, C_MV, C_HOLD, C_N };
static const char *const colNames[C_N] = {"tickMs", "flowSeq", "flowOk", "flowResidual_dg", "flowDt_100us",
                                          "flowDx_cpx", "flowScale_e4", "autoState", "mvState", "holdFlags"};
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
  return csvRows > 0;
}

/* one frame as the recorder first showed it (the row where flowSeq changes) */
typedef struct {
  long tick, seq, hold;
  bool ok, hover;
  float res, dt, dx, scale;
} frame_t;
static frame_t frames[MAX_FRAMES];
static int nFrames;

static void collectFrames(void)
{
  nFrames = 0;
  long prevSeq = -1;
  for (int i = 0; i < csvRows && nFrames < MAX_FRAMES; i++) {
    if (csv[i][C_SEQ] == prevSeq) {
      continue;
    }
    prevSeq = csv[i][C_SEQ];
    frame_t *f = &frames[nFrames++];
    f->tick = csv[i][C_TICK];
    f->seq = csv[i][C_SEQ];
    f->hold = csv[i][C_HOLD];
    f->ok = csv[i][C_OK] != 0;
    f->hover = csv[i][C_AUTO] == 3;   /* AUTO_HOVER */
    f->res = (float)csv[i][C_RES] * 0.1f;
    f->dt = (float)csv[i][C_DT] * 1e-4f;
    f->dx = (float)csv[i][C_DX] * 0.01f;
    f->scale = (float)csv[i][C_SCALE] * 1e-4f;
  }
}

typedef struct { int atMs, lenMs; } gap_t;
#define MAX_GAPS 32

/* the lost-view intervals of the move starting at startTick (RUN), relative to it, under the gate at limit (0 off):
 * every frame of the flight through the gate in order; a refused frame loses the view until the next accepted one */
static int lostIntervals(float limit, long startTick, long endTick, gap_t *gaps, int *refusedRes, int *refusedExp,
                         int *refusedFlow)
{
  camHoldScene_t g;
  camHoldSceneReset(&g);
  camHoldSceneSetMaxResidual(&g, limit);
  int n = 0;
  long lostAt = -1;
  *refusedRes = *refusedExp = *refusedFlow = 0;
  for (int i = 0; i < nFrames; i++) {
    const frame_t *f = &frames[i];
    const uint32_t r0 = g.rejRes, e0 = g.rejExp;
    const bool gateOk = camHoldSceneOk(&g, f->hover, f->dt, f->res, f->dx, f->scale);
    const bool accepted = f->ok && gateOk;
    const bool inMove = f->tick >= startTick && f->tick < endTick;
    if (inMove) {
      *refusedRes += g.rejRes != r0;
      *refusedExp += g.rejExp != e0;
      *refusedFlow += !f->ok;
    }
    if (!accepted && lostAt < 0 && inMove) {
      lostAt = f->tick;
    } else if (accepted && lostAt >= 0) {
      if (n < MAX_GAPS) {
        gaps[n].atMs = (int)(lostAt - startTick);
        gaps[n].lenMs = (int)(f->tick - lostAt);
        n++;
      }
      lostAt = -1;
    }
  }
  return n;
}

static void replayMove(rig_t *r, int fwdCm, const gap_t *gaps, int n, int *stoppedAt)
{
  rigSpeed(r, 300, 5.5f);
  CHECK(autoMotionMove(&r->m, &r->in, 0, (int16_t)fwdCm, 0) == AUTO_MVW_NONE, "replay: forward %d cm accepted", fwdCm);
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
      *stoppedAt = (int)(r->in.nowMs - t0) - gaps[k].atMs;
    }
    step(r);
  }
  untilEnd(r, 40000);
}

static const char *gapsStr(const gap_t *gaps, int n, char *buf, size_t len)
{
  size_t k = 0;
  buf[0] = 0;
  for (int i = 0; i < n && k + 1 < len; i++) {
    k += (size_t)snprintf(buf + k, len - k, "%s%d ms for %d", i ? ", " : "", gaps[i].atMs, gaps[i].lenMs);
  }
  return buf;
}

typedef struct { const char *name; int fwdCm; long startTick, endTick; } move_t;

int main(int argc, char **argv)
{
  const char *dir = argc > 1 ? argv[1] : "../../firmware-backups/cal";
  char path[1024];
  snprintf(path, sizeof(path), "%s/flight-2026-10-22-table.csv", dir);
  if (!loadCsv(path)) {
    printf("no log at %s\n0 checks, 0 failures\n", path);
    return 0;
  }
  collectFrames();
  autoMotionSetMargins(NULL);
  CHECK(CAM_HOLD_SCENE_RES_DEFAULT == 16.0f && CAM_HOLD_SCENE_RES_X10_DEFAULT == 160 && AUTO_MV_FAST_VIEW_DEB_MS == 323,
        "native profile: the scene gate's residual limit 16.0 by default, the VIEW_FAST timer 323 ms");
  /* the two forward MOVEs (mvState 17 RUN from these ticks; the log's own stops at 147348 and 164369) */
  const move_t moves[] = {{"move 1 (2.0 m)", 200, 142566, 160000}, {"move 2 (0.85 m)", 85, 162337, 167000}};
  /* the log's own coasting (holdFlags bit 4 at a frame's first row) for the replay's validation */
  for (int m = 0; m < 2; m++) {
    int logged = 0;
    for (int i = 0; i < nFrames; i++) {
      logged += frames[i].tick >= moves[m].startTick && frames[i].tick < moves[m].endTick && (frames[i].hold & 4) != 0;
    }
    printf("info: %s: %d frames logged as coasting (holdFlags bit 4)\n", moves[m].name, logged);
  }
  const struct { float limit; const char *name; } limits[] = {{8.0f, "8.0 (the installed build's)"},
                                                              {16.0f, "16.0 (this build's native default)"},
                                                              {0.0f, "0 (off)"}};
  for (unsigned L = 0; L < 3; L++) {
    for (int m = 0; m < 2; m++) {
      gap_t gaps[MAX_GAPS];
      int rr, re, rf;
      const int n = lostIntervals(limits[L].limit, moves[m].startTick, moves[m].endTick, gaps, &rr, &re, &rf);
      char buf[512];
      int longest = 0;
      for (int i = 0; i < n; i++) {
        longest = gaps[i].lenMs > longest ? gaps[i].lenMs : longest;
      }
      rig_t r;
      int stoppedAt;
      replayMove(&r, moves[m].fwdCm, gaps, n, &stoppedAt);
      printf("info: limit %s, %s: refused by residual %d, expansion %d, the flow itself %d; lost %d x [%s]; longest %d "
             "ms; the move ends %s%s\n", limits[L].name, moves[m].name, rr, re, rf, n, gapsStr(gaps, n, buf, sizeof buf),
             longest, autoMvStateName(r.m.state), r.m.state == AUTO_MV_STOPPED ? autoMvEndName(r.m.end) : "");
      if (L == 0) {
        /* the installed limit reproduces the log: residual refusals three in a row (move 1; two more in a later
         * 174 ms coast past the log's own stop) and six (move 2), the coasts 275 / 544 ms; at 323 ms move 1 runs on */
        CHECK(m == 0 ? (rr >= 3 && longest >= 270 && longest <= 280) : (rr == 6 && longest >= 540 && longest <= 550),
              "limit 8.0 reproduces the log's coasts: %s refused by residual %d, longest %d ms", moves[m].name, rr,
              longest);
        CHECK(m == 0 ? r.m.state == AUTO_MV_DONE : (r.m.state == AUTO_MV_STOPPED && r.m.end == AUTO_MVE_VIEW_FAST),
              "limit 8.0 at 323 ms: %s %s", moves[m].name, m == 0 ? "runs on to DONE" : "still stops VIEW_FAST");
      } else {
        CHECK(rr == 0, "limit %s: %s has no residual refusal", limits[L].name, moves[m].name);
        if (m == 0) {
          CHECK(r.m.state == AUTO_MV_DONE && stoppedAt < 0 && longest <= 181,
                "limit %s: move 1 runs on to DONE with no setting (its longest coast %d ms: the flow's own refusals)",
                limits[L].name, longest);
        } else {
          /* move 2's end: the gate's expansion test (the EMA of scale / dt over 0.3 /s, the observer's gate) refuses
           * the last frames of the approach: the user's mvwmax 30 plans the approach at 0.30 /s, the test's own limit.
           * 363 ms of lost view: still VIEW_FAST at 323 ms; mvview 400 (or mvwmax 25) lets it run on. */
          CHECK(re >= 3 && longest >= 355 && longest <= 370 && r.m.state == AUTO_MV_STOPPED &&
                    r.m.end == AUTO_MVE_VIEW_FAST,
                "limit %s: move 2 still coasts %d ms on the expansion test (%d frames) and stops VIEW_FAST at 323 ms",
                limits[L].name, longest, re);
          autoMotionSetViewDeb(400);
          replayMove(&r, moves[m].fwdCm, gaps, n, &stoppedAt);
          CHECK(r.m.state == AUTO_MV_DONE && stoppedAt < 0, "limit %s, mvview 400: move 2 runs on to DONE",
                limits[L].name);
          autoMotionSetViewDeb(0);
        }
      }
    }
  }
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
