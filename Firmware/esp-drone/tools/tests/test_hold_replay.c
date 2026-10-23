/*
 * Replay of the logged autonomy sorties through the camera drift hold with
 * and without the HOVER scene gate (components/core/crazyflie/utils/src/
 * camera_hold.c camHoldSceneOk; docs/autonomy-a2.md section 12).
 *
 * Build and run from Firmware/esp-drone (skipped without the CSV directory):
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_hold_replay.c components/core/crazyflie/utils/src/camera_hold.c \
 *      -lm -o /tmp/test_hold_replay && /tmp/test_hold_replay ../../firmware-backups/cal [-v]
 *
 * Input: the 100 Hz recorder files of the autonomy sorties. Every new
 * flowSeq is one camera frame; the hold gets what camera_flow.c gives it from
 * that frame (the view test: fit ok and fresh, texture >= 8, residual < 15,
 * dt > 0, slow yaw; wFwd = scale / dt, wLat = -dxRes / 102.4 / dt) and from
 * the recorder (sticks centred = rollDes and pitchDes 0, airborne = thrust
 * above 30000); the gate gets autonomy HOVER from autoState. As the
 * investigators' replay (fix-ih/synth/hframes.py + hrep2.c).
 *
 * Checks:
 *   (a) the eight clean 2026-10-12 sorties and 2026-10-13 flight 2: the hold's
 *       outputs (pitch, roll, engaged, coasting) bit-identical gated and
 *       ungated on every frame, the gate never refusing;
 *   (b) 2026-10-13 flight 1, 28 s from HOVER entry to the contact (34.44 s):
 *       the person walking across and away in front of the camera. Ungated
 *       (the drone): pitch to its -2.5 deg limit, roll to -1.07 deg (it flew
 *       forward-left into the contact); gated: pitch >= -1.75 deg, roll >=
 *       -0.35 deg, >= 45 of the frames coasting (ungated 17);
 *   (c) flight 1, 2-12 s from HOVER entry (a clean view): mean |pitch
 *       difference| <= 0.02 deg;
 *   (d) the exposure-step proxy (docs/autonomy-a2b.md 19.4; no image
 *       sequences were logged): frames whose logged mean brightness moved
 *       more than 8 % refused, as camera_flow.c refuses a step it cannot
 *       correct. The good flights (2026-10-14 flight 4, tiger f1/f2, tiger2
 *       f1, mission60 f1, night f6, pwmA1 f7): < 1 % of the HOVER frames,
 *       no new coasting run over 0.5 s; the 20 MHz bright-window failures
 *       (interactive f2, preAB f3) printed;
 *   (e) every logged autonomy take-off with the forward hold off in TAKEOFF
 *       (docs/autonomy-a2b.md 19.5): the hold's pitch entering HOVER (the
 *       last TAKEOFF frame's output) within +-0.3 deg; after the first HOVER
 *       frame (its own P response to the motion it sees) reported;
 *   (f) the gain scale with the scene distance (docs/autonomy-a2b.md 20) on
 *       the 2026-10-14 sorties: s from the logged cvGain / cvFlags / mvState /
 *       autoState through camHoldGainTarget and camHoldGainSlew as
 *       camera_flow.c computes it. kmax 1: bit-identical to the hold without
 *       it; kmax 2.5: every output finite and within the 2.5 deg limit, s 1
 *       until the observer's first tracked G in the air (1.0-2.0 s after
 *       lift-off), then 1.5-2.5 (mean >= 1.9) in HOVER until the first MOVE /
 *       YAW; during a MOVE never above its value at the move's start (the
 *       gain following a move toward the scene, camHoldGainMove, its D
 *       taken from the latched G as the recorder has no motion D), and 1
 *       again within 2.1 s of the MOVE's end (0.75 /s from 2.5 in HOVER once
 *       a MOVE ran, docs/autonomy-a2b.md 21.3; 1 s of the MOVE at 2 /s
 *       before) or within 1 s of LAND entry (LAND flies s 1 at 2 /s: the
 *       descent's floor parallax).
 * Times are recorder ticks from the first HOVER row (the investigators'
 * t_hover); the gate's own clock is the frames' time.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera_hold.h"

static int failures = 0;
static int checks = 0;
static int verbose = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define MAX_ROWS 20000   /* a 180 s recorder dump (docs/autonomy-a2b.md 19.3) */
#define MAX_FRAMES 3000
#define KX_GX (-1.0f)     /* camera_flow.c FLOW_KX_* */
#define KX_GY (-4.9f)
#define KX_GZ 102.4f

enum { C_TICK, C_FID, C_THRUST, C_RDES, C_PDES, C_SEQ, C_OK, C_TEX, C_RES, C_DT, C_DX, C_SC, C_FGX, C_FGY, C_FGZ,
       C_HP, C_HR, C_AUTO, C_LUM, C_CVG, C_CVF, C_MV, C_REFF, C_N };
#define C_REQUIRED C_CVG   /* the columns from here on are optional (0 when a file has none) */
static const char *colNames[C_N] = {
  "tickMs", "flightId", "thrust", "rollDes_cdeg", "pitchDes_cdeg", "flowSeq", "flowOk", "flowTexture",
  "flowResidual_dg", "flowDt_100us", "flowDx_cpx", "flowScale_e4", "flowGx_e4", "flowGy_e4", "flowGz_e4",
  "holdPitch_cdeg", "holdRoll_cdeg", "autoState", "flowLum", "cvGain_dpm", "cvFlags", "mvState", "mvRefFwd_e4",
};
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
  for (int c = 0; c < C_REQUIRED; c++) {
    if (idx[c] < 0) {
      fclose(f);
      return false;
    }
  }
  csvRows = 0;
  while (csvRows < MAX_ROWS && fgets(line, sizeof(line), f) != NULL) {
    for (int c = C_REQUIRED; c < C_N; c++) {
      csv[csvRows][c] = 0;
    }
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

typedef struct {
  float t;            /* s since the first HOVER row (recorder ticks); 0-based from the flight start without one */
  bool air, sticks, ok;
  bool lumStep;       /* the logged mean brightness moved > 8 % from the previous frame (the exposure proxy) */
  float tex, res, dt, dx, scale, g[3];
  int autoState;
  uint8_t mvState, cvFlags;   /* the recorder's (0 in a file without them) */
  uint16_t cvGain;            /* 0.1 px/m */
  float refFwd;               /* the motion's forward reference (ln), the recorder's mvRefFwd_e4 */
} frame_t;

static frame_t fr[MAX_FRAMES];
static int nf;

static bool prepare(int fid)
{
  nf = 0;
  long tHover = -1, t0 = -1;
  for (int i = 0; i < csvRows; i++) {
    if (csv[i][C_FID] != fid) {
      continue;
    }
    if (t0 < 0) {
      t0 = csv[i][C_TICK];
    }
    if (tHover < 0 && csv[i][C_AUTO] == 3) {
      tHover = csv[i][C_TICK];
    }
  }
  if (t0 < 0) {
    return false;
  }
  const long tRef = tHover >= 0 ? tHover : t0;
  long prev = 0, prevLum = -1;
  bool first = true;
  for (int i = 0; i < csvRows && nf < MAX_FRAMES; i++) {
    const long *r = csv[i];
    if (r[C_FID] != fid) {
      continue;
    }
    const bool isNew = r[C_SEQ] > 0 && (first || r[C_SEQ] != prev);
    first = false;
    prev = r[C_SEQ];
    if (!isNew) {
      continue;
    }
    frame_t *f = &fr[nf++];
    f->t = (float)(r[C_TICK] - tRef) / 1000.0f;
    f->air = r[C_THRUST] > 30000;
    f->sticks = r[C_RDES] == 0 && r[C_PDES] == 0;
    f->ok = r[C_OK] != 0;
    f->lumStep = prevLum > 0 && labs(r[C_LUM] - prevLum) * 100 > 8 * prevLum;
    prevLum = r[C_LUM];
    f->tex = (float)r[C_TEX];
    f->res = (float)r[C_RES] / 10.0f;
    f->dt = (float)r[C_DT] * 1e-4f;
    f->dx = (float)r[C_DX] / 100.0f;
    f->scale = (float)r[C_SC] / 1e4f;
    f->g[0] = (float)r[C_FGX] / 1e4f;
    f->g[1] = (float)r[C_FGY] / 1e4f;
    f->g[2] = (float)r[C_FGZ] / 1e4f;
    f->autoState = (int)r[C_AUTO];
    f->mvState = (uint8_t)r[C_MV];
    f->cvFlags = (uint8_t)r[C_CVF];
    f->cvGain = (uint16_t)r[C_CVG];
    f->refFwd = (float)r[C_REFF] * 1e-4f;
  }
  return nf > 0;
}

typedef struct {
  float pitch[MAX_FRAMES], roll[MAX_FRAMES];
  bool engaged[MAX_FRAMES], coasting[MAX_FRAMES];
  int refused;
  camHoldScene_t scene;
} out_t;

static out_t outU, outG;

static bool lumProxy;   /* refuse the frames with lumStep (camera_flow.c's exposure gate without a gain) */
static bool takeoffFwdOff;   /* camera_flow.c since docs/autonomy-a2b.md 19.5: no forward hold in TAKEOFF */
static float gainKmax;       /* > 0: the gain scale (docs/autonomy-a2b.md 20) with this kmax, as camera_flow.c */
static float gainS[MAX_FRAMES];   /* the scale applied per frame (gainKmax > 0) */

/* camera_flow.c's holdUpdate() on the logged frames. */
static void run(bool gated, out_t *o)
{
  camHoldState_t st;
  camHoldReset(&st);
  camHoldSceneReset(&o->scene);
  o->refused = 0;
  camHoldGainLatch_t latch;
  camHoldGainReset(&latch);
  float s = 1.0f;
  /* camHoldGainMove's inputs the recorder lacks: the motion's D lower bound (the latched G's D at the move's start)
   * and the measured progress (the replay's own pFwd since the move's start) */
  float dLowRep = 0.0f, pFwd0 = 0.0f;
  bool wasMove = false;
  for (int i = 0; i < nf; i++) {
    const frame_t *f = &fr[i];
    const float dt = f->dt;
    const bool viewOk = f->ok && !(lumProxy && f->lumStep) && f->tex >= 8.0f && f->res < 15.0f && dt > 0.0f &&
                        fabsf(f->g[2]) < 0.35f * dt;
    const float dxRes = f->dx - (KX_GX * f->g[0] + KX_GY * f->g[1] + KX_GZ * f->g[2]);
    const bool sceneOk = camHoldSceneOk(&o->scene, f->autoState == 3, dt, f->res, dxRes, f->scale);
    o->refused += !sceneOk;
    camHoldInput_t in = {
      .enabled = true, .sticksCentred = f->sticks, .airborne = f->air, .viewOk = viewOk && (!gated || sceneOk),
      .wFwd = dt > 0.0f ? f->scale / dt : 0.0f, .wLat = dt > 0.0f ? -dxRes / KX_GZ / dt : 0.0f, .dt = dt,
      .fwdOff = takeoffFwdOff && f->autoState == 2,
    };
    if (gainKmax > 0.0f) {   /* camera_flow.c holdUpdate */
      /* (the target first: it sets the latch's translated, which the rate reads) */
      float tgt = camHoldGainTarget(&latch, (uint8_t)f->autoState, f->mvState, f->cvFlags, f->cvGain, gainKmax);
      const bool isMove = (f->mvState >> 4) == 1u && (f->mvState & 15u) >= 1u && (f->mvState & 15u) <= 3u;
      if (isMove && !wasMove) {
        dLowRep = latch.g > 0.0f ? 99.79f / latch.g : 0.0f;   /* AUTO_MV_FY_PX / G */
        pFwd0 = st.pFwd;
      }
      wasMove = isMove;
      tgt = camHoldGainMove(&latch, tgt, s, (uint8_t)f->autoState, f->mvState, dLowRep, f->refFwd, st.pFwd - pFwd0);
      s = camHoldGainSlew(s, tgt, dt, camHoldGainRate(&latch, (uint8_t)f->autoState));
      in.gainScale = s;
      gainS[i] = s;
    }
    camHoldStep(&st, &in);
    o->pitch[i] = st.pitch;
    o->roll[i] = st.roll;
    o->engaged[i] = st.engaged;
    o->coasting[i] = st.coasting;
  }
}

/* (a) gated and ungated identical. */
static void clean(const char *name)
{
  run(false, &outU);
  run(true, &outG);
  int diff = 0, hover = 0;
  for (int i = 0; i < nf; i++) {
    diff += outU.pitch[i] != outG.pitch[i] || outU.roll[i] != outG.roll[i] || outU.engaged[i] != outG.engaged[i] ||
            outU.coasting[i] != outG.coasting[i];
    hover += fr[i].autoState == 3;
  }
  if (verbose) {
    printf("  %-34s frames %4d (HOVER %3d): gate refused %d, outputs differing %d\n", name, nf, hover, outG.refused,
           diff);
  }
  CHECK(diff == 0 && outG.refused == 0 && hover > 0, "%s: %d of %d frames differ gated vs ungated, %d refused", name,
        diff, nf, outG.refused);
}

static void window(const out_t *o, float t0, float t1, int *n, float *pMin, float *rMin, int *coast)
{
  *n = 0;
  *coast = 0;
  *pMin = 99.0f;
  *rMin = 99.0f;
  for (int i = 0; i < nf; i++) {
    if (fr[i].autoState != 3 || fr[i].t < t0 || fr[i].t > t1) {
      continue;
    }
    (*n)++;
    *coast += o->coasting[i];
    *pMin = fminf(*pMin, o->pitch[i]);
    *rMin = fminf(*rMin, o->roll[i]);
  }
}

static void flight1(void)
{
  run(false, &outU);
  run(true, &outG);
  int n, nU, cU, cG;
  float pU, rU, pG, rG;
  window(&outU, 28.0f, 34.44f, &nU, &pU, &rU, &cU);
  window(&outG, 28.0f, 34.44f, &n, &pG, &rG, &cG);
  printf("2026-10-13 flight 1, 28-34.44 s from HOVER entry (the person; %d HOVER frames): ungated min pitch %+.2f "
         "min roll %+.2f deg, coasting %d; gated min pitch %+.2f min roll %+.2f deg, coasting %d; gate refusals: "
         "residual %u, lateral %u, expansion %u (the whole flight)\n", n, pU, rU, cU, pG, rG, cG,
         (unsigned)outG.scene.rejRes, (unsigned)outG.scene.rejDx, (unsigned)outG.scene.rejExp);
  CHECK(pU <= -2.45f && rU <= -1.0f && cU <= 20, "flight 1 ungated reproduces the lean into the contact: pitch %.2f, "
        "roll %.2f, %d coasting", pU, rU, cU);
  CHECK(pG >= -1.75f && rG >= -0.35f && cG >= 45, "flight 1 gated: min pitch %.2f (>= -1.75), min roll %.2f "
        "(>= -0.35), coasting %d of %d (>= 45)", pG, rG, cG, n);
  double sum = 0.0;
  int m = 0;
  for (int i = 0; i < nf; i++) {
    if (fr[i].autoState == 3 && fr[i].t >= 2.0f && fr[i].t <= 12.0f) {
      sum += fabsf(outG.pitch[i] - outU.pitch[i]);
      m++;
    }
  }
  printf("  2-12 s from HOVER entry (clean view, %d frames): mean |pitch difference| %.4f deg\n", m, m ? sum / m : 0.0);
  CHECK(m > 100 && sum / m <= 0.02, "flight 1, 2-12 s: mean |pitch difference| %.4f deg (<= 0.02)", m ? sum / m : 0.0);
}

/* The longest coasting run (s) in HOVER. */
static double longestCoast(const out_t *o)
{
  double best = 0.0, t0 = -1.0;
  for (int i = 0; i < nf; i++) {
    if (fr[i].autoState == 3 && o->coasting[i]) {
      if (t0 < 0.0) {
        t0 = fr[i].t - fr[i].dt;
      }
      best = fmax(best, fr[i].t - t0);
    } else {
      t0 = -1.0;
    }
  }
  return best;
}

/* (d) the exposure-step proxy (docs/autonomy-a2b.md 19.4): the frames whose logged mean brightness moved more than
 * 8 % refused, as camera_flow.c refuses an exposure step it cannot correct. Good flights: < 1 % of the HOVER frames,
 * no coasting run in HOVER longer than 0.5 s that was not there; the 20 MHz bright-window failures printed. */
static void exposureProxy(const char *dir)
{
  static const struct { const char *file; int fid; bool good; } cases[] = {
    { "flight-2026-10-14-morning.csv", 4, true },    { "flight-2026-10-13-tiger.csv", 1, true },
    { "flight-2026-10-13-tiger.csv", 2, true },      { "flight-2026-10-13-tiger2.csv", 1, true },
    { "flight-2026-10-12-mission60.csv", 1, true },  { "flight-2026-10-13-night.csv", 6, true },
    { "flight-2026-10-13-pwmA1.csv", 7, true },
    { "flight-2026-10-13-interactive.csv", 2, false }, { "flight-2026-10-13-preAB.csv", 3, false },
  };
  char path[1024];
  for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
    snprintf(path, sizeof(path), "%s/%s", dir, cases[k].file);
    const bool ok = loadCsv(path) && prepare(cases[k].fid);
    CHECK(ok, "%s flight %d", cases[k].file, cases[k].fid);
    if (!ok) {
      continue;
    }
    lumProxy = false;
    run(true, &outU);
    lumProxy = true;
    run(true, &outG);
    lumProxy = false;
    int hover = 0, stepsHover = 0, stepsAir = 0, coastU = 0, coastG = 0;
    for (int i = 0; i < nf; i++) {
      hover += fr[i].autoState == 3 && fr[i].air;
      stepsHover += fr[i].autoState == 3 && fr[i].air && fr[i].lumStep;
      stepsAir += fr[i].air && fr[i].lumStep;
      coastU += fr[i].autoState == 3 && outU.coasting[i];
      coastG += fr[i].autoState == 3 && outG.coasting[i];
    }
    const double cU = longestCoast(&outU), cG = longestCoast(&outG);
    const double pct = hover ? 100.0 * stepsHover / hover : 0.0;
    printf("  exposure proxy %-30s f%d: refused %2d airborne frames, %2d of %4d in HOVER (%.2f %%); HOVER coasting "
           "%d -> %d frames, longest run %.2f -> %.2f s%s\n", cases[k].file + 7, cases[k].fid, stepsAir, stepsHover,
           hover, pct, coastU, coastG, cU, cG, cases[k].good ? "" : " (20 MHz bright-window failure)");
    if (cases[k].good) {
      CHECK(pct < 1.0, "%s f%d: %.2f %% of the HOVER frames refused", cases[k].file, cases[k].fid, pct);
      CHECK(cG <= fmax(cU, 0.5), "%s f%d: a new coasting run of %.2f s (%.2f without)", cases[k].file, cases[k].fid,
            cG, cU);
    }
  }
}

/* (e) every logged autonomy take-off (2026-10-12..14): the forward hold off in TAKEOFF (docs/autonomy-a2b.md 19.5).
 * The hold's pitch entering HOVER within +-0.3 deg (with the forward hold in the climb: the take-off's fake expansion
 * integrated, +0.5..+2.5 deg nose-up on the drone). Open loop on the logged frames, so the difference that stays
 * later is the forward integral the climb left (reported at HOVER + 5 s): the logged frames carry the motion the old
 * hold's kick caused, so no convergence can be shown open loop (the closed-loop sim: test_interactive_sim.c
 * testTakeoffKick). */
static void takeoffs(const char *dir)
{
  static const char *files[] = {
    "flight-2026-10-12-a2b-hops.csv", "flight-2026-10-12-calib1.csv", "flight-2026-10-12-calib2.csv",
    "flight-2026-10-12-evening.csv", "flight-2026-10-12-hop5s.csv", "flight-2026-10-12-mission5s.csv",
    "flight-2026-10-12-mission60.csv", "flight-2026-10-13-battfwd.csv", "flight-2026-10-13-battfwd2.csv",
    "flight-2026-10-13-interactive.csv", "flight-2026-10-13-night.csv", "flight-2026-10-13-preAB.csv",
    "flight-2026-10-13-propswap.csv", "flight-2026-10-13-pwmA1.csv", "flight-2026-10-13-tiger.csv",
    "flight-2026-10-13-tiger2.csv", "flight-2026-10-14-morning.csv",
  };
  char path[1024];
  int n = 0, positive = 0, worse = 0;
  double maxOff = 0.0, sumOn = 0.0, maxFirst = 0.0;
  for (unsigned k = 0; k < sizeof(files) / sizeof(files[0]); k++) {
    snprintf(path, sizeof(path), "%s/%s", dir, files[k]);
    if (!loadCsv(path)) {
      continue;
    }
    for (int fid = 1; fid <= 12; fid++) {
      if (!prepare(fid)) {
        continue;
      }
      int iH = -1, nTo = 0;
      double scaleTo = 0.0;
      for (int i = 0; i < nf; i++) {
        if (fr[i].autoState == 2 && fr[i].air) {
          scaleTo += fr[i].scale;
          nTo++;
        }
        if (iH < 0 && fr[i].autoState == 3) {
          iH = i;
        }
      }
      if (iH < 0 || nTo < 5) {
        continue;   /* no climb with frames before a HOVER (or the ring began after it) */
      }
      takeoffFwdOff = false;
      run(true, &outU);
      takeoffFwdOff = true;
      run(true, &outG);
      takeoffFwdOff = false;
      int i5 = iH;
      while (i5 + 1 < nf && fr[i5].t < fr[iH].t + 5.0f) {
        i5++;
      }
      n++;
      positive += scaleTo > 0.0;
      /* entering HOVER: the output of the last TAKEOFF frame (the first HOVER frame adds its own P response to the
       * motion it sees, reported) */
      const int iE = iH > 0 ? iH - 1 : iH;
      maxOff = fmax(maxOff, fabs(outG.pitch[iE]));
      maxFirst = fmax(maxFirst, fabs(outG.pitch[iH]));
      sumOn += outU.pitch[iE];
      worse += fabs(outG.pitch[iE]) > fabs(outU.pitch[iE]) + 0.05;
      if (verbose || fabs(outG.pitch[iE]) > 0.3) {
        printf("  take-off %-34s f%d: TAKEOFF sum(scale) %+.3f over %d frames; hold pitch entering HOVER %+.2f deg "
               "(forward hold in the climb %+.2f), after the first HOVER frame %+.2f (%+.2f), at HOVER + 5 s %+.2f "
               "(%+.2f)\n", files[k] + 7, fid, scaleTo, nTo, outG.pitch[iE], outU.pitch[iE], outG.pitch[iH],
               outU.pitch[iH], outG.pitch[i5], outU.pitch[i5]);
      }
    }
  }
  printf("autonomy take-offs replayed: %d (TAKEOFF expansion positive in %d); the hold's pitch entering HOVER: "
         "|max| %.2f deg without the forward hold in the climb (after the first HOVER frame's own update %.2f), mean "
         "%+.2f deg with it\n", n, positive, maxOff, maxFirst, n ? sumOn / n : 0.0);
  CHECK(n >= 17 && maxOff <= 0.3 && worse == 0, "take-offs: %d replayed (>= 17), |hold pitch| entering HOVER "
        "%.2f deg (<= 0.3), %d larger than with the forward hold", n, maxOff, worse);
}

/* (f) the gain scale on the 2026-10-14 sorties (docs/autonomy-a2b.md 20). Lift-off: the observer's airborne bit
 * (cvFlags bit 0); a flight whose ring starts in the air has no lift-off and is checked from its HOVER only. */
static void gainScale(const char *dir)
{
  static const struct { const char *file; int fid; } cases[] = {
    { "flight-2026-10-14-afternoon.csv", 3 }, { "flight-2026-10-14-afternoon.csv", 4 },
    { "flight-2026-10-14-afternoon.csv", 5 }, { "flight-2026-10-14-p2b.csv", 6 },
    { "flight-2026-10-14-p2b.csv", 7 },     { "flight-2026-10-14-p2b.csv", 8 },
    { "flight-2026-10-14-p2b.csv", 9 },     { "flight-2026-10-14-morning.csv", 4 },
  };
  char path[1024];
  int n = 0, nLift = 0, identical = 0, bounded = 0, timing = 0, steady = 0, afterMove = 0, nMove = 0, inLand = 0,
      nLand = 0;
  for (unsigned k = 0; k < sizeof(cases) / sizeof(cases[0]); k++) {
    snprintf(path, sizeof(path), "%s/%s", dir, cases[k].file);
    const bool ok = loadCsv(path) && prepare(cases[k].fid);
    CHECK(ok, "%s flight %d", cases[k].file, cases[k].fid);
    if (!ok) {
      continue;
    }
    n++;
    takeoffFwdOff = true;
    gainKmax = 0.0f;
    run(true, &outU);
    gainKmax = 1.0f;
    run(true, &outG);
    int diff = 0;
    for (int i = 0; i < nf; i++) {
      diff += memcmp(&outU.pitch[i], &outG.pitch[i], sizeof(float)) != 0 ||
              memcmp(&outU.roll[i], &outG.roll[i], sizeof(float)) != 0 || outU.engaged[i] != outG.engaged[i] ||
              outU.coasting[i] != outG.coasting[i] || gainS[i] != 1.0f;
    }
    identical += diff == 0;
    gainKmax = CAM_HOLD_GAIN_KMAX_DEFAULT;
    run(true, &outG);
    gainKmax = 0.0f;
    takeoffFwdOff = false;
    /* lift-off, the first tracked G, the HOVER until the first motion, the first MOVE */
    float tLift = NAN, tTrk = NAN, tS = NAN, tMove = NAN, tLand = NAN, sMin = 9.0f, sMax = 0.0f, maxOut = 0.0f,
          maxOut1 = 0.0f, sLand = 1.0f, tMoveEnd = NAN, sMoveStart = 1.0f, sMoveMax = 0.0f;
    bool finite = true, groundStart = false, moveOk = true, landOk = true;
    int nH = 0;
    double sSum = 0.0;
    bool motion = false;
    for (int i = 0; i < nf; i++) {
      const frame_t *f = &fr[i];
      groundStart = groundStart || (i == 0 && !(f->cvFlags & 1u));
      if (isnan(tLift) && groundStart && (f->cvFlags & 1u)) {
        tLift = f->t;
      }
      if (isnan(tTrk) && (f->cvFlags & 1u) && (f->cvFlags & 4u)) {
        tTrk = f->t;
      }
      if (isnan(tS) && gainS[i] > 1.0f) {
        tS = f->t;
      }
      const unsigned kind = f->mvState >> 4;
      if (kind != 0u) {
        motion = true;
      }
      const unsigned mst = f->mvState & 15u;
      if (isnan(tMove) && kind == 1u) {
        tMove = f->t;
        sMoveStart = i > 0 ? gainS[i - 1] : 1.0f;
      }
      if (!isnan(tMove) && isnan(tMoveEnd) && kind == 1u && (mst == 4u || mst == 5u)) {
        tMoveEnd = f->t;
      }
      if (!isnan(tMove) && isnan(tMoveEnd) && gainS[i] > sMoveStart + 1e-6f) {
        moveOk = false;   /* during the MOVE: never above the scale it started with (camHoldGainMove) */
      }
      if (!isnan(tMoveEnd) && f->t > tMoveEnd + 2.1f && gainS[i] != 1.0f) {
        moveOk = false;   /* 1 within 2.1 s of the MOVE's end (2.0 s of slew from 2.5 at 0.75 /s) and then for good */
      }
      if (!isnan(tMove) && isnan(tMoveEnd)) {
        sMoveMax = fmaxf(sMoveMax, gainS[i]);
      }
      if (isnan(tLand) && f->autoState == 4) {
        tLand = f->t;
        sLand = i > 0 ? gainS[i - 1] : 1.0f;
      }
      if (!isnan(tLand) && f->t > tLand + 1.0f && gainS[i] != 1.0f) {
        landOk = false;   /* LAND flies s 1 (the descent's floor parallax): within 1 s of its entry, for good */
      }
      if (f->autoState == 3 && !motion && !isnan(tTrk) && f->t > tTrk + 1.0f) {
        sMin = fminf(sMin, gainS[i]);
        sMax = fmaxf(sMax, gainS[i]);
        sSum += gainS[i];
        nH++;
      }
      finite = finite && isfinite(outG.pitch[i]) && isfinite(outG.roll[i]);
      maxOut = fmaxf(maxOut, fmaxf(fabsf(outG.pitch[i]), fabsf(outG.roll[i])));
      maxOut1 = fmaxf(maxOut1, fmaxf(fabsf(outU.pitch[i]), fabsf(outU.roll[i])));
    }
    bounded += finite && maxOut <= CAM_HOLD_LIMIT_DEG + 1e-6f;
    const float sMean = nH ? (float)(sSum / nH) : 0.0f;
    const bool timed = groundStart && !isnan(tLift) && !isnan(tS) && tS - tLift >= 1.0f && tS - tLift <= 2.0f &&
                       tS >= tTrk;
    nLift += groundStart && !isnan(tLift);
    timing += timed;
    steady += nH > 0 && sMin >= 1.5f && sMax <= 2.5f && sMean >= 1.9f;
    nMove += !isnan(tMove);
    afterMove += !isnan(tMove) && moveOk;
    nLand += !isnan(tLand) && sLand > 1.0f;
    inLand += !isnan(tLand) && sLand > 1.0f && landOk;
    printf("  gain %-30s f%d: lift-off %s, G tracked %+.2f s, s > 1 from %+.2f s after it; HOVER before a motion %d "
           "frames: s %.2f..%.2f (mean %.2f); first MOVE %s (s %.2f at its start, %.2f at most in its %.1f s); LAND %s "
           "(s %.2f entering it); |hold| max %.2f deg "
           "(kmax 1: %.2f); kmax 1 identical %s\n", cases[k].file + 7, cases[k].fid,
           groundStart ? "logged" : "before the ring", tTrk - tLift, tS - tLift, nH, sMin, sMax, sMean,
           isnan(tMove) ? "none" : (moveOk ? "s <= its start's in it, 1 after it" : "s NOT as it should"),
           sMoveStart, isnan(tMove) ? 0.0f : sMoveMax, isnan(tMoveEnd) || isnan(tMove) ? 0.0f : tMoveEnd - tMove,
           isnan(tLand) ? "none" : (landOk ? "s 1 after 1 s" : "s NOT 1"), sLand, maxOut, maxOut1,
           diff == 0 ? "yes" : "NO");
  }
  printf("gain scale on the 2026-10-14 sorties: %d replayed (%d with the lift-off logged), kmax 1 bit-identical %d, "
         "bounded %d, s from the first tracked G 1.0-2.0 s after lift-off %d, HOVER s 1.5..2.5 (mean >= 1.9) %d, s 1 "
         "after a MOVE %d of %d, s 1 in LAND (entered at s > 1) %d of %d\n", n, nLift, identical, bounded, timing,
         steady, afterMove, nMove, inLand, nLand);
  CHECK(n == 8 && identical == n && bounded == n, "gain scale: kmax 1 bit-identical (%d of %d), outputs finite and "
        "within the limit (%d)", identical, n, bounded);
  CHECK(nLift >= 6 && timing == nLift, "gain scale: s 1 until the first tracked G, 1.0-2.0 s after lift-off (%d of %d)",
        timing, nLift);
  CHECK(steady >= 6, "gain scale: s 1.5..2.5 (mean >= 1.9) in HOVER before a motion (%d flights)", steady);
  CHECK(nMove >= 3 && afterMove == nMove, "gain scale: s never above its value at a MOVE's start while it runs, back "
        "to 1 within 2.1 s of its end and for good (%d of %d)",
        afterMove, nMove);
  CHECK(nLand >= 2 && inLand == nLand, "gain scale: s back to 1 within 1 s of LAND entry and for good (%d of %d "
        "LANDs entered at s > 1)", inLand, nLand);
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
  static const struct { const char *file; int fid; } clean9[] = {
    { "flight-2026-10-12-a2b-hops.csv", 1 }, { "flight-2026-10-12-a2b-hops.csv", 2 },
    { "flight-2026-10-12-calib1.csv", 1 },   { "flight-2026-10-12-calib2.csv", 1 },
    { "flight-2026-10-12-evening.csv", 4 },  { "flight-2026-10-12-hop5s.csv", 3 },
    { "flight-2026-10-12-mission5s.csv", 1 }, { "flight-2026-10-12-mission60.csv", 1 },
    { "flight-2026-10-13-interactive.csv", 2 },
  };
  char path[1024];
  int n = 0;
  for (unsigned k = 0; k < sizeof(clean9) / sizeof(clean9[0]); k++) {
    snprintf(path, sizeof(path), "%s/%s", dir, clean9[k].file);
    const bool ok = loadCsv(path) && prepare(clean9[k].fid);
    CHECK(ok, "%s flight %d", clean9[k].file, clean9[k].fid);
    if (ok) {
      char name[96];
      snprintf(name, sizeof(name), "%.25s f%d", clean9[k].file + 12, clean9[k].fid);
      clean(name);
      n++;
    }
  }
  printf("clean sorties (2026-10-12 x 8, 2026-10-13 flight 2): %d replayed, hold outputs gated vs ungated identical\n",
         n);
  snprintf(path, sizeof(path), "%s/flight-2026-10-13-interactive.csv", dir);
  const bool ok = loadCsv(path) && prepare(1);
  CHECK(ok, "2026-10-13 flight 1");
  if (ok) {
    flight1();
  }
  exposureProxy(dir);
  takeoffs(dir);
  gainScale(dir);
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
