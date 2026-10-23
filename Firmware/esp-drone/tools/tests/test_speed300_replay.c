/*
 * Replay of the 300 % flights of 2026-10-17 through the real motion module and camera hold
 * (components/core/crazyflie/utils/src/auto_motion.c, camera_hold.c): the regression test of the move fixes
 * (auto_motion.h "move fixes", docs/autonomy-a2b.md 33; the forensics firmware-backups/handoff-2026-10-12/
 * overshoot300-verdict.md).
 *
 * Logs (firmware-backups/cal): flight-2026-10-17-speed300.csv flight 6 (4dea542, mvspeed 300: MOVE forward 1.00, then
 * back 1.00) and flight-2026-10-17-speed300b.csv flight 7 (forward 0.80, back 0.80, back 0.70), both flown with the
 * guard margins before 2026-10-17 and no move fixes.
 *
 * The camera hold runs on the recorder's frames as camera_flow.c runs it (the replay of the forensics, a1's replay.c:
 * the frame's own view tests, the yaw gate, the HOVER scene gate, the gain scale from the observer's G with its MOVE
 * rule, the moving reference, the fast-move lean limit and slew at 300 %); the motion module steps on every record
 * (the recorder's clock: 950.6 ticks a second, which makes the logged RUNs last the profiles' T), its MOVEs sent where
 * the log's mvState starts one, the hold's measured progress and epoch as its inputs, D from the observer's logged G.
 * The drone flew the old code, so the replay is open loop: its motion is the logged one. A decision of this build is
 * meaningful up to the first point it differs from the logged one (after it the log is the drone's answer to the old
 * decision).
 *
 * Checks:
 *  - fidelity (mvfix 0, the margins of before): flight 6's move 1 accepted at D 2.30 m, SHORT at the logged time,
 *    dTrack 1.669 m; move 2 planned at 1.669 m (-0.4705 ln), DONE at the logged time; the replayed hold's pitch and roll
 *    within 0.10 deg rms of the logged ones over both moves; flight 7's moves end where the log has them end (DONE, then
 *    the two PROGRESS stops: the replayed guard trips within 0.1 s of the logged stop);
 *  - the move fixes (DRIFT, LONG; STILL, which would start the profiles later than the log, off): flight 6's move 2 is
 *    planned at the camera's own distance 2.02 m (the drift since move 1's end: x e^0.13), its target -0.40 ln (1.00 m,
 *    not 1.21 m), and the logged motion that went 0.2-0.45 m past it never ends DONE (LONG or PROGRESS); with LONG alone
 *    (the old target) it ends LONG with the measured end the hold's reference;
 *  - STILL: the replayed hold's moving flag (camHoldMoving) at each acceptance (flight 6's move 2: drifting back 0.15
 *    m/s, moving);
 *  - GAIN, BRAKE (the replayed hold's numbers, open loop): in move 1 the gain scale from the measured progress stays
 *    above the reference's (1.0 from 15.99 s in the log) and the braking lean is scaled by the measured speed (0.3-0.5 x:
 *    the drone at 0.09 m/s, the reference at 0.24);
 *  - flight 7 with the margins of 2026-10-17 and the fixes: its two back moves run on past their logged PROGRESS stops
 *    (the sideways 0.25-0.26 m inside 0.40 m).
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=c99 -O2 -Wall -Wextra -I$C/utils/interface tools/tests/test_speed300_replay.c \
 *      $C/utils/src/auto_motion.c $C/utils/src/camera_hold.c -lm -o /tmp/test_speed300_replay && \
 *   /tmp/test_speed300_replay <firmware-backups/cal>
 * Lines starting with "info:" are measured numbers, not checks.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_fsm.h"       /* AUTO_HOVER, AUTO_TAKEOFF (the recorder's autoState) */
#include "auto_motion.h"
#include "camera_hold.h"


static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define MAX_ROWS 20000
#define TICKS_PER_S 950.6f   /* the recorder's clock against the state machine's (the logged RUNs last T) */

enum { C_TICK, C_FID, C_THR, C_RDES, C_PDES, C_ROLL, C_PITCH, C_SEQ, C_OK, C_TEX, C_RES, C_DT, C_AGE, C_DX, C_SCALE,
       C_GX, C_GY, C_GZ, C_HOLD, C_HROLL, C_HPITCH, C_CVGAIN, C_CVFLAGS, C_AST, C_VZH, C_MV, C_REFLAT, C_REFFWD, C_N };
static const char *colName[C_N] = {"tickMs", "flightId", "thrust", "rollDes_cdeg", "pitchDes_cdeg", "roll_cdeg",
                                   "pitch_cdeg", "flowSeq", "flowOk", "flowTexture", "flowResidual_dg", "flowDt_100us",
                                   "flowAge_ms", "flowDx_cpx", "flowScale_e4", "flowGx_e4", "flowGy_e4", "flowGz_e4",
                                   "holdFlags", "holdRoll_cdeg", "holdPitch_cdeg", "cvGain_dpm", "cvFlags",
                                   "autoState", "vzH_mm", "mvState", "mvRefLat_e4", "mvRefFwd_e4"};
static long rows[MAX_ROWS][C_N];
static int nRows;

/* The rows of flight fid (0 rows: the file is missing). */
static int loadFlight(const char *path, int fid)
{
  nRows = 0;
  FILE *f = fopen(path, "r");
  if (!f) {
    return 0;
  }
  static char line[16384];
  int idx[C_N];
  for (int c = 0; c < C_N; c++) {
    idx[c] = -1;
  }
  if (!fgets(line, sizeof(line), f)) {
    fclose(f);
    return 0;
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
      return 0;
    }
  }
  while (nRows < MAX_ROWS && fgets(line, sizeof(line), f)) {
    long v[160];
    int n = 0;
    for (char *tok = strtok(line, ",\r\n"); tok && n < 160; tok = strtok(NULL, ",\r\n")) {
      v[n++] = strtol(tok, NULL, 10);
    }
    if (idx[C_FID] >= n || v[idx[C_FID]] != fid) {
      continue;
    }
    for (int c = 0; c < C_N; c++) {
      rows[nRows][c] = idx[c] < n ? v[idx[c]] : 0;
    }
    nRows++;
  }
  fclose(f);
  return nRows;
}

#define MAX_MOVES 4

typedef struct {
  int n;                              /* MOVEs sent */
  float accT[MAX_MOVES];              /* s from the flight's first record */
  float d0[MAX_MOVES], tgt[MAX_MOVES];
  bool movingAtAcc[MAX_MOVES];        /* the replayed hold's camHoldMoving on the last frame before the acceptance */
  float endT[MAX_MOVES];              /* DONE or STOPPED (-1: none in the replay) */
  autoMvEnd_t end[MAX_MOVES];
  float endFwd[MAX_MOVES];            /* the measured forward progress at the end (ln) */
  float dTrackEnd[MAX_MOVES];
  float refFwdAfter[MAX_MOVES];       /* the hold's forward reference 0.3 s after the end (a LONG / SHORT: the end) */
  float pitchRms, rollRms;            /* the replayed hold's output against the logged one over the moves */
  int pitchN;
  float sAt[MAX_MOVES], brakeScaleMin;   /* move 1: the gain scale at 15.99 s; the braking lean's smallest scale */
  int hoverFrames, hoverMoving;       /* engaged HOVER frames without a motion, and of them the hold's moving flag */
  float stillRun[MAX_MOVES];          /* the still run (camHoldMoving's stillS) on the last frame before each acceptance */
} replay_t;

/* Replays the loaded flight: fwdCm[k] the k-th MOVE's forward cm, margins 0 before 2026-10-17 / 1 the defaults, fix the
 * mvfix bits (the camera's GAIN and BRAKE too), pct the mvspeed it was flown at; the speed steps' caps as flown (A_H
 * 0.30 m/s^2, the settle 1.0 s at 300: the logged profiles): NVS mvacc 30, mvsettle 1000 (2026-10-17 "no, I want real
 * 300%"; a host-only hook before them), the defaults again at the end. */
static void replay(const int16_t *fwdCm, int nMoves, int margins, uint8_t fix, uint16_t pct, replay_t *r)
{
  autoMotionSetFastCaps(30, 1000);
  memset(r, 0, sizeof(*r));
  for (int k = 0; k < MAX_MOVES; k++) {
    r->endT[k] = -1.0f;
  }
  r->brakeScaleMin = 9.0f;
  autoMvMargins_t g;
  if (margins == 0) {
    autoMvMarginsOld(&g);
  } else {
    autoMvMarginsDefault(&g);
  }
  g.fix = fix;
  autoMotionSetMargins(&g);
  autoMotion_t m;
  autoMotionInit(&m);
  autoMotionSetSpeed(&m, pct);
  camHoldState_t h;
  camHoldReset(&h);
  camHoldScene_t sc;
  camHoldSceneReset(&sc);
  camHoldGainLatch_t gl;
  camHoldGainReset(&gl);
  float s = 1.0f, lim, slew;
  camHoldMoveLimits(pct, &lim, &slew);
  autoMvOut_t out;
  memset(&out, 0, sizeof(out));
  long lastSeq = -1, prevMv = 0;
  bool moving = false;
  float prevPitch = 0.0f, prevRoll = 0.0f;
  const long tick0 = rows[0][C_TICK];
  autoMvState_t prevState = AUTO_MV_IDLE;
  for (int i = 0; i < nRows; i++) {
    const long *v = rows[i];
    const float t = (float)(v[C_TICK] - tick0) / TICKS_PER_S;
    const long st = v[C_AST], mvs = v[C_MV];
    autoMvIn_t in;
    memset(&in, 0, sizeof(in));
    in.nowMs = (uint32_t)lrintf(t * 1000.0f);
    in.flight = true;
    in.sceneDM = v[C_CVGAIN] > 0 ? AUTO_MV_FY_PX / ((float)v[C_CVGAIN] * 0.1f) : 0.0f;
    in.sceneTracked = (v[C_CVFLAGS] & 4) != 0 && v[C_CVGAIN] > 0;
    in.holdApplied = (v[C_HOLD] & 2) != 0;
    in.holdCoasting = (v[C_HOLD] & 4) != 0;
    in.camFrameAgeMs = (uint32_t)v[C_AGE];
    in.tiltDeg = hypotf((float)v[C_ROLL], (float)v[C_PITCH]) * 0.01f;
    in.camHeight = true;
    in.vzH = (float)v[C_VZH] * 0.001f;
    in.hTargetM = 0.9f;
    in.progLat = h.pLat;
    in.progFwd = h.pFwd;
    in.holdEpochSeen = h.epoch;
    in.airMs = 10000;
    in.holdMoving = moving;
    if ((mvs & 15) == AUTO_MV_RUN && (mvs >> 4) == AUTO_MVK_MOVE && (prevMv & 15) != AUTO_MV_RUN &&
        (prevMv & 15) != AUTO_MV_PAUSED && st == AUTO_HOVER && r->n < nMoves) {
      const int k = r->n++;
      r->movingAtAcc[k] = moving;
      r->stillRun[k] = h.stillS;
      const autoMvWhy_t w = autoMotionMove(&m, &in, 0, fwdCm[k], 0);
      r->accT[k] = t;
      r->d0[k] = w == AUTO_MVW_NONE ? m.d0 : -1.0f;
      r->tgt[k] = m.fwdTgt;
    }
    prevMv = mvs;
    if (st == AUTO_HOVER) {
      autoMotionStep(&m, &in, &out);
    } else {
      autoMotionIdleOut(&m, &out);
    }
    if (r->n > 0 && (out.state == AUTO_MV_DONE || out.state == AUTO_MV_STOPPED) && prevState != out.state &&
        (prevState == AUTO_MV_RUN || prevState == AUTO_MV_PAUSED || prevState == AUTO_MV_SETTLE)) {
      const int k = r->n - 1;
      r->endT[k] = t;
      r->end[k] = out.end;
      r->endFwd[k] = m.progFwd;
      r->dTrackEnd[k] = m.dTrack;
    }
    prevState = out.state;
    for (int k = 0; k < r->n; k++) {
      if (r->endT[k] >= 0.0f && t >= r->endT[k] + 0.3f && r->refFwdAfter[k] == 0.0f && k == r->n - 1) {
        r->refFwdAfter[k] = out.refFwd;
      }
    }
    if (v[C_SEQ] != lastSeq && v[C_SEQ] > 0) {
      lastSeq = v[C_SEQ];
      const uint8_t myMv = (uint8_t)(out.state | (out.kind << 4));
      const float fdt = (float)v[C_DT] * 1e-4f, scale = (float)v[C_SCALE] * 1e-4f, fx = (float)v[C_DX] * 0.01f;
      const float a0 = (float)v[C_GX] * 1e-4f, a1 = (float)v[C_GY] * 1e-4f, a2 = (float)v[C_GZ] * 1e-4f;
      const float dxRes = fx - (-1.0f * a0 - 4.9f * a1 + 102.4f * a2);   /* camera_flow.c FLOW_KX_* */
      bool viewOk = v[C_OK] && v[C_TEX] >= 8 && (float)v[C_RES] * 0.1f < 15.0f && fdt > 0.0f;
      viewOk = viewOk && camHoldYawOk(a2, fdt, camHoldTurnActive(myMv));
      const bool sok = camHoldSceneOk(&sc, st == AUTO_HOVER, fdt, (float)v[C_RES] * 0.1f, dxRes, scale);
      float tgt = camHoldGainTarget(&gl, (uint8_t)st, myMv, (uint8_t)v[C_CVFLAGS], (uint16_t)v[C_CVGAIN], 2.5f);
      const bool fixGain = (fix & AUTO_MVFIX_GAIN) != 0u;
      camHoldGainMoveFix(&gl, fixGain, fixGain && pct > 100 ? CAM_HOLD_GAIN_GRACE_S : 0.0f, fdt);
      tgt = camHoldGainMove(&gl, tgt, s, (uint8_t)st, myMv, out.dLowM, out.refFwd, h.epoch == out.epoch ? h.pFwd : 0.0f);
      s = camHoldGainSlew(s, tgt, fdt, camHoldGainRate(&gl, (uint8_t)st));
      camHoldInput_t hin;
      memset(&hin, 0, sizeof(hin));
      hin.enabled = true;
      hin.sticksCentred = v[C_RDES] == 0 && v[C_PDES] == 0;
      hin.airborne = v[C_THR] > 30000;
      hin.viewOk = viewOk && sok;
      hin.wFwd = fdt > 0.0f ? scale / fdt : 0.0f;
      hin.wLat = fdt > 0.0f ? -dxRes / 102.4f / fdt : 0.0f;
      hin.dt = fdt;
      hin.refEpoch = out.epoch;
      hin.refSnapEpoch = out.snapEpoch;
      hin.refLat = out.refLat;
      hin.refFwd = out.refFwd;
      hin.rateLat = out.rateLat;
      hin.rateFwd = out.rateFwd;
      hin.ffRoll = out.ffRollDeg;
      hin.ffPitch = out.ffPitchDeg;
      hin.fwdOff = st == AUTO_TAKEOFF;
      hin.gainScale = s;
      hin.moveLimDeg = lim;
      hin.moveSlewDegS = slew;
      hin.ffBrakeScale = (fix & AUTO_MVFIX_BRAKE) != 0u;
      camHoldStep(&h, &hin);
      moving = camHoldMoving(&h, fdt);
      if (st == AUTO_HOVER && h.engaged && !camHoldMotionActive(myMv) && t > 6.0f) {
        r->hoverFrames++;
        r->hoverMoving += moving;
      }
      /* the braking lean's scale (move 1, the decel: the feedforward nose up against the forward reference) */
      if (r->n == 1 && out.ffPitchDeg > 0.3f && out.rateFwd > 0.0f && h.engaged && fabsf(h.rateFwdF) >= 0.01f &&
          h.pFwd * h.rFwd > 0.0f && fabsf(h.pFwd) >= 0.5f * fabsf(h.rFwd)) {
        r->brakeScaleMin = fminf(r->brakeScaleMin, fminf(fmaxf(h.fwdF / h.rateFwdF, 0.0f), 1.0f));
      }
      if (r->n == 1 && r->sAt[0] == 0.0f && t >= 15.99f) {
        r->sAt[0] = s;
      }
      /* fidelity: the replayed output against the logged one while a move runs (the logged correction is the slewed
       * one; the hold's own output is what camera_flow.c publishes) */
      if (r->n > 0 && ((mvs & 15) == AUTO_MV_RUN || (mvs & 15) == AUTO_MV_SETTLE)) {
        /* (the recorder's row may still carry the previous frame's correction: the nearer of the two) */
        const float lp = (float)v[C_HPITCH] * 0.01f, lr = (float)v[C_HROLL] * 0.01f;
        const float dp = fminf(fabsf(h.pitch - lp), fabsf(prevPitch - lp)), dr = fminf(fabsf(h.roll - lr), fabsf(prevRoll - lr));
        r->pitchRms += dp * dp;
        r->rollRms += dr * dr;
        r->pitchN++;
      }
      prevPitch = h.pitch;
      prevRoll = h.roll;
    }
  }
  if (r->pitchN > 0) {
    r->pitchRms = sqrtf(r->pitchRms / (float)r->pitchN);
    r->rollRms = sqrtf(r->rollRms / (float)r->pitchN);
  }
  autoMotionSetMargins(NULL);
  autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
}

static void printReplay(const char *what, const replay_t *r)
{
  for (int k = 0; k < r->n; k++) {
    printf("info: %s, MOVE %d: accepted at %.3f s (the hold %s), D %.4f m, target %+.4f ln; end %s at %.3f s, measured "
           "%+.4f ln, dTrack %.4f m, the hold's forward reference 0.3 s after %+.4f\n", what, k + 1, r->accT[k],
           r->movingAtAcc[k] ? "moving" : "still", r->d0[k], r->tgt[k], autoMvEndName(r->end[k]), r->endT[k],
           r->endFwd[k], r->dTrackEnd[k], r->refFwdAfter[k]);
  }
  printf("info: %s: the replayed hold against the logged correction while the moves ran: pitch %.3f, roll %.3f deg rms "
         "(%d frames); hover frames without a motion flagged moving %d of %d\n", what, r->pitchRms, r->rollRms,
         r->pitchN, r->hoverMoving, r->hoverFrames);
}

static void testFlight6(const char *dir)
{
  char path[1024];
  snprintf(path, sizeof(path), "%s/flight-2026-10-17-speed300.csv", dir);
  if (loadFlight(path, 6) < 1000) {
    printf("info: %s not found: flight 6 not replayed\n", path);
    return;
  }
  static const int16_t f6[2] = {100, -100};
  replay_t o, f, l;
  replay(f6, 2, 0, 0, 300, &o);
  printReplay("flight 6, 4dea542's logic (no move fixes, the margins before 2026-10-17)", &o);
  CHECK(o.n == 2 && fabsf(o.d0[0] - 2.2993f) < 0.005f && o.end[0] == AUTO_MVE_SHORT && fabsf(o.endT[0] - 18.44f) < 0.1f &&
            fabsf(o.dTrackEnd[0] - 1.6688f) < 0.01f,
        "flight 6 replayed as flown: move 1 at D %.4f (2.2993), SHORT at %.2f s (18.44), dTrack %.4f (1.6688)", o.d0[0],
        o.endT[0], o.dTrackEnd[0]);
  CHECK(fabsf(o.d0[1] - 1.6688f) < 0.01f && fabsf(o.tgt[1] + 0.4705f) < 0.003f && o.end[1] == AUTO_MVE_DONE &&
            fabsf(o.endT[1] - 32.43f) < 0.1f,
        "flight 6's move 2 as flown: planned at %.4f m (1.6688), target %+.4f (-0.4705), DONE at %.2f s (32.43) while "
        "%.3f ln measured", o.d0[1], o.tgt[1], o.endT[1], o.endFwd[1]);
  CHECK(o.pitchRms < 0.10f && o.rollRms < 0.10f && o.pitchN > 150, "the replayed hold reproduces the logged correction "
        "(pitch %.3f, roll %.3f deg rms over %d frames)", o.pitchRms, o.rollRms, o.pitchN);
  CHECK(!o.movingAtAcc[0] || o.movingAtAcc[1], "info-like: the moving flag at move 2's acceptance (%d; move 1 %d)",
        o.movingAtAcc[1], o.movingAtAcc[0]);
  CHECK(o.movingAtAcc[1], "STILL: at move 2's acceptance (drifting back 0.15-0.17 m/s) the replayed hold reports moving: "
        "its profile would have waited");
  /* the move fixes (STILL off: it would start the profiles later than the log) */
  const uint8_t fixOpen = (uint8_t)(AUTO_MVFIX_DEFAULT & ~AUTO_MVFIX_STILL);
  replay(f6, 2, 0, fixOpen, 300, &f);
  printReplay("flight 6, the move fixes (DRIFT, LONG, GAIN, BRAKE)", &f);
  CHECK(f.end[0] == AUTO_MVE_SHORT && fabsf(f.d0[1] - 2.02f) < 0.03f && fabsf(f.tgt[1] - logf(f.d0[1] / (f.d0[1] + 1.0f))) <
            1e-4f && f.tgt[1] > -0.41f,
        "DRIFT: flight 6's move 2 planned at %.3f m, the camera's own count (2.02 m; 1.669 as flown), its target %+.4f ln "
        "(1.00 m there; -0.4705 was 1.21 m)", f.d0[1], f.tgt[1]);
  CHECK(f.end[1] != AUTO_MVE_DONE && (f.end[1] == AUTO_MVE_LONG || f.end[1] == AUTO_MVE_PROGRESS),
        "the logged motion past the new target is not DONE: %s at %.2f s (%.3f ln measured against %+.4f)",
        autoMvEndName(f.end[1]), f.endT[1], f.endFwd[1], f.tgt[1]);
  printf("info: flight 6 move 1 (open loop): the gain scale at 15.99 s %.2f with GAIN, %.2f without (the log 1.00); the "
         "braking lean's smallest scale with BRAKE %.2f; move 2 with the fixes reported %.2f m (at its D), as flown "
         "%.2f m\n", f.sAt[0], o.sAt[0], f.brakeScaleMin, f.d0[1] * (1.0f - expf(-f.endFwd[1])),
         o.d0[1] * (1.0f - expf(-o.endFwd[1])));
  CHECK(f.sAt[0] > o.sAt[0] + 0.05f && f.brakeScaleMin < 0.6f,
        "GAIN, BRAKE (open loop): at 15.99 s of move 1 the gain scale %.2f from the measured progress (%.2f from the "
        "reference's, the log 1.00); the braking lean scaled down to %.2f x (the drone at 0.09 m/s, its reference 0.24)",
        f.sAt[0], o.sAt[0], f.brakeScaleMin);
  replay(f6, 2, 0, AUTO_MVFIX_LONG, 300, &l);
  printReplay("flight 6, LONG alone", &l);
  CHECK(fabsf(l.d0[1] - 1.6688f) < 0.01f && l.end[1] == AUTO_MVE_LONG && fabsf(l.refFwdAfter[1] - l.endFwd[1]) < 0.01f &&
            l.endFwd[1] < -0.55f,
        "LONG: move 2 at the old target ends LONG (%s), its measured end %.3f ln (0.10 ln past -0.4705) the hold's "
        "reference after it (%+.4f): the drone holds there and says so (4dea542: DONE, then the hold walked it back)",
        autoMvEndName(l.end[1]), l.endFwd[1], l.refFwdAfter[1]);
}

static void testFlight7(const char *dir)
{
  char path[1024];
  snprintf(path, sizeof(path), "%s/flight-2026-10-17-speed300b.csv", dir);
  if (loadFlight(path, 7) < 1000) {
    printf("info: %s not found: flight 7 not replayed\n", path);
    return;
  }
  static const int16_t f7[3] = {80, -80, -70};
  static const float stopT[3] = {11.845f, 19.209f, 31.296f};   /* the logged ends: DONE, STOPPED, STOPPED */
  replay_t o, n;
  replay(f7, 3, 0, 0, 300, &o);
  printReplay("flight 7, 4dea542's logic", &o);
  CHECK(o.n == 3 && o.end[0] == AUTO_MVE_DONE && o.end[1] == AUTO_MVE_PROGRESS && o.end[2] == AUTO_MVE_PROGRESS &&
            fabsf(o.endT[1] - stopT[1]) < 0.10f && fabsf(o.endT[2] - stopT[2]) < 0.10f,
        "flight 7 replayed as flown: DONE, then PROGRESS at %.2f s (%.2f) and %.2f s (%.2f)", o.endT[1], stopT[1],
        o.endT[2], stopT[2]);
  replay(f7, 3, 1, (uint8_t)(AUTO_MVFIX_DEFAULT & ~AUTO_MVFIX_STILL), 300, &n);
  printReplay("flight 7, the margins of 2026-10-17 and the move fixes", &n);
  CHECK(n.n == 3 && (n.endT[1] < 0.0f || n.endT[1] > stopT[1] + 0.05f) && (n.endT[2] < 0.0f || n.endT[2] > stopT[2] + 0.05f),
        "flight 7 with the margins of 2026-10-17: the back moves run on past their logged PROGRESS stops (%s at %.2f s, "
        "%s at %.2f s; the log after the stops is the drone's answer to them)", autoMvEndName(n.end[1]), n.endT[1],
        autoMvEndName(n.end[2]), n.endT[2]);
}

int main(int argc, char **argv)
{
  const char *dir = argc > 1 ? argv[1] : "../../firmware-backups/cal";
  testFlight6(dir);
  testFlight7(dir);
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
