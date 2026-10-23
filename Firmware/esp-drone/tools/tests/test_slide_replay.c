/*
 * The left slide of the forward MOVEs (2026-10-22): the logged moves replayed closed loop through the real motion
 * module (components/core/crazyflie/utils/src/auto_motion.c) and the real camera hold (camera_hold.c), with the
 * lateral pull each log showed, with and without the two settings of this build (NVS mvlatk, mvffr).
 *
 * What the logs say (firmware-backups/energy-2026-10-17/slide/slide_lateral.py, slide_frames.py --kin): on every
 * forward MOVE of 2026-10-17/18/22 the camera measured the drone sliding left of its reference while the hover before
 * it held position; the hold answered by its law (+0.3..+1.7 deg of roll over the move) and still lost 0.3-0.8 m. The
 * kinematics of each move (the camera's lateral speed, the estimated roll against the hover's, drag 0.30 /s) leave a
 * lateral force the attitude does not explain: -0.04..-0.56 m/s^2 per m/s of forward speed (a 0.3-0.7 deg leftward
 * lean's worth at 0.3 m/s on 7 of the 9 moves, ~0 on two). The optical leaks (a camera yawed or rolled on its mount)
 * fit no single angle across the moves (the per-move fits give "angles" of -13..+27 deg that change sign between
 * moves of one day), and the slide reverses once the forward reference stops: a real, speed-proportional pull.
 *
 * The replay: a lateral point mass (lean -> acceleration, drag 0.30 /s, the attitude 80 ms behind the hold's output,
 * which the controller applies at the hold's own slew) pulled by k_F x v_fwd during the move, a body-fixed trim error
 * the hold learns in the 10 s hover before it (today's hover leans -0.8..-1.2 deg); the forward axis tracks the
 * reference (D(t) = D0 exp(-refFwd)); the camera a frame every 91 ms (native QVGA) delivered 124 ms late, wLat = the
 * frame's lateral displacement / D / dt, wFwd = the reference rate; the hold's gain scale from the observer's G = f / D
 * (kmax 2.5, the MOVE rule), the scene gate, the fast-move limits at 300 % (today's NVS: mvspeed 300, mvacc 20,
 * mvsettle 2500, mvwmax 30, the guards off, mvfix 59). The MOVE is the logged one (its right / forward metres at D0).
 *
 * Checks: with today's settings (mvlatk 100, mvffr 0) the replay slides where the log slid (within the log's own
 * numbers: the end error of the same sign, at least half of the logged one); mvlatk 300 alone cuts every sliding
 * move's end error; mvffr 18 (1.8 deg per m/s right) alone cuts it on the sliding moves and costs the two moves
 * without a pull at most 0.25 m the other way; both together end every move within 0.30 m of its target and never
 * stop a move the defaults finish (no new guard: the settings only move the drone's trajectory).
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=c99 -O2 -Wall -Wextra -DESPFLY_NATIVE_VGA=1 -I$C/utils/interface tools/tests/test_slide_replay.c \
 *      $C/utils/src/auto_motion.c $C/utils/src/camera_hold.c -lm -o /tmp/test_slide_replay && /tmp/test_slide_replay
 * Lines starting with "info:" are measured numbers, not checks.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_fsm.h"       /* AUTO_HOVER */
#include "auto_motion.h"
#include "camera_cadence.h"
#include "camera_hold.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define G_MPS2        9.81f
#define DRAG          0.30f      /* 1/s (auto_motion.h AUTO_MV_DRAG_PER_S; the logs' ax / v_fwd 0.3-0.7) */
#define ATT_TAU_S     0.08f      /* the attitude behind the applied setpoint */
#define FRAME_S       0.091f     /* native QVGA (camera_cadence.h CAM_CADENCE_VGA_PERIOD_MS) */
#define CAM_LAG_S     0.124f     /* capture to the hold's step (camera_cadence.h: the navigation lag) */
#define PLANT_DT      0.001f
#define HOVER_S       20.0f      /* the hover before the move: the hold has learned the trim (the logged hovers) */
#define AFTER_S       2.0f       /* after the move's end */

typedef struct {
  const char *name;
  float d0;            /* m, the observer's D at the acceptance */
  int16_t rightCm, fwdCm;   /* the logged move (its reference at the RUN end, in metres at D0) */
  float kF;            /* m/s^2 per m/s of forward speed: the fitted lateral pull (slide_frames.py --kin f / v_fwd) */
  float hoverLean;     /* deg: the hold's roll in the hover before (the trim error it has learned; + right) */
  float loggedErrM;    /* m: the logged lateral end error at the RUN end: the camera's lateral metres along D(t)
                          (sum of wLat D(t) dt over the engaged frames, slide_lateral.py (1)) minus the asked metres.
                          The status's "travelled right" multiplies the hold's x / D integral by the distance at the
                          acceptance and over-states a slide that happens nearer the scene (table4: 1.13 m reported,
                          0.69 m travelled, 0.39 beyond the 0.30 asked). */
} moveCase_t;

/* The nine logged forward moves (slide_lateral.py / slide_frames.py --kin, 2026-10-22). */
static const moveCase_t cases[] = {
  {"10-22 table 10:36 (no pull)", 3.09f, -6, 130, -0.050f, -0.82f, -0.02f},
  {"10-22 table2 11:40 (little pull)", 3.13f, -40, 199, -0.066f, -0.76f, 0.01f},
  {"10-22 table3 11:46", 2.74f, -35, 132, -0.309f, -1.17f, -0.26f},
  {"10-22 table4 13:18", 2.97f, -30, 190, -0.277f, -1.03f, -0.39f},
  {"10-18 13:59", 3.35f, -10, 220, -0.315f, -0.28f, -0.18f},
  {"10-18 noon f1 a", 3.33f, -20, 174, -0.352f, -0.37f, -0.08f},
  {"10-18 noon f2 a", 3.16f, 10, 180, -0.557f, -0.03f, -0.37f},
  {"10-18 noon f4 (no pull)", 3.06f, 0, 130, -0.038f, 0.02f, -0.05f},
  {"10-17 table2 f2", 2.62f, -41, 168, -0.360f, 1.03f, -0.12f},
};
#define PULLED(k) (cases[k].kF <= -0.20f)   /* the moves with a pull (7 of 9) */
#define N_CASES ((int)(sizeof(cases) / sizeof(cases[0])))

typedef struct {
  autoMvEnd_t end;
  float endS;          /* s from the acceptance to the end (-1: none) */
  float errEndM;       /* m: the true lateral position minus the asked one at the end */
  float errAfterM;     /* ... 2 s after it */
  float errMaxM;       /* the largest |error| during the move */
  float rollMax;       /* the hold's largest roll output during the move */
  float askRightM;     /* the asked lateral metres */
  float fwdM;          /* m: the forward metres of the move (at D0) */
  float hoverDriftM;   /* m: the hover's drift from its start while the hold learned the trim */
  float errRunEndM;    /* m: the error when the RUN ends (the settle's start: the logs' number) */
  float vFwdMax;       /* m/s: the reference's top forward speed */
  float runS;          /* s: the RUN */
} result_t;

/* One closed-loop move. latPct: mvlatk; ffr: mvffr; speedPct: mvspeed (300 today). */
static uint8_t simWmax = 30;   /* mvwmax in force (today's NVS 30) */
static void simulateAt(const moveCase_t *c, uint16_t latPct, int32_t ffr, uint16_t speedPct, result_t *r)
{
  memset(r, 0, sizeof(*r));
  r->endS = -1.0f;
  autoMotionSetFastCaps(20, 2500);
  autoMotionSetFwdRate(simWmax);
  autoMotionSetViewDeb(0);
  autoMotionSetFfRoll(ffr);
  autoMvMargins_t g;
  autoMvMarginsFrom(0, 0, 0, 0, 40, 0, AUTO_MVFIX_DEFAULT, &g);   /* today's NVS: the guards off */
  autoMvMarginsExtra(&g, 100, 0);
  autoMotionSetMargins(&g);
  autoMotion_t m;
  autoMotionInit(&m);
  autoMotionSetSpeed(&m, speedPct);
  camHoldState_t h;
  camHoldReset(&h);
  camHoldScene_t sc;
  camHoldSceneReset(&sc);
  camHoldGainLatch_t gl;
  camHoldGainReset(&gl);
  float s = 1.0f, lim, slew;
  camHoldMoveLimits(speedPct, &lim, &slew);
  autoMvOut_t out;
  memset(&out, 0, sizeof(out));
  autoMotionIdleOut(&m, &out);

  /* the plant */
  float x = 0.0f, v = 0.0f, lean = 0.0f, applied = 0.0f;   /* lateral m, m/s; the attitude; the applied setpoint */
  const float bias = -c->hoverLean;   /* a body-fixed trim error: the hover's lean cancels it */
  float D = c->d0;
  float t = 0.0f, nextFrameT = 0.0f, nextStepT = 0.0f;
  float xCap = 0.0f, tCap = -1.0f;   /* the frame being exposed: capture position and time */
  float dCap = c->d0;
  /* frames in flight (the lag is longer than a frame): captured at .t, delivered CAM_LAG_S later */
  typedef struct { float t, wLat, wFwd, dt, D; bool valid; } frame_t;
  frame_t ring[4];
  memset(ring, 0, sizeof(ring));
  int ringIn = 0, ringOut = 0;
  float prevCapT = -FRAME_S, prevCapX = 0.0f;
  bool accepted = false, ended = false;
  float tAcc = 0.0f, tEnd = 0.0f, xAcc = 0.0f;
  uint32_t seed = 12345u;
  const float askRight = (float)c->rightCm * 0.01f;
  r->askRightM = askRight;
  r->fwdM = (float)c->fwdCm * 0.01f;
  while (t < HOVER_S + 40.0f) {
    if (!accepted && t >= HOVER_S - 0.5f && t < HOVER_S - 0.5f + PLANT_DT) {
      r->hoverDriftM = x;   /* where the hover has taken the drone (the hold's learned trim) */
    }
    /* the motion module every 10 ms */
    if (t >= nextStepT) {
      nextStepT += 0.01f;
      autoMvIn_t in;
      memset(&in, 0, sizeof(in));
      in.nowMs = (uint32_t)lrintf(t * 1000.0f);
      in.flight = true;
      in.sceneDM = D;
      in.sceneTracked = true;
      in.holdApplied = h.engaged || h.coasting;
      in.holdCoasting = h.coasting;
      in.camFrameAgeMs = (uint32_t)(CAM_LAG_S * 1000.0f);
      in.tiltDeg = fabsf(lean);
      in.camHeight = true;
      in.vzH = 0.9f;
      in.hTargetM = 0.9f;
      in.progLat = h.pLat;
      in.progFwd = h.pFwd;
      in.holdEpochSeen = h.epoch;
      in.airMs = (uint32_t)lrintf(t * 1000.0f) + 5000u;
      in.holdMoving = false;
      if (!accepted && t >= HOVER_S) {
        const autoMvWhy_t w = autoMotionMove(&m, &in, c->rightCm, c->fwdCm, 0);
        if (w != AUTO_MVW_NONE) {
          printf("info: %s refused: %s\n", c->name, autoMvWhyName(w));
          return;
        }
        accepted = true;
        tAcc = t;
        xAcc = x;   /* the move's reference starts where the drone is */
      }
      const autoMvState_t before = m.state;
      autoMotionStep(&m, &in, &out);
      if (accepted && !ended && before == AUTO_MV_RUN && out.state != AUTO_MV_RUN && r->runS == 0.0f) {
        r->errRunEndM = (x - xAcc) - askRight;
        r->runS = t - tAcc;
      }
      if (accepted && !ended && (out.state == AUTO_MV_DONE || out.state == AUTO_MV_STOPPED) && before != out.state) {
        ended = true;
        tEnd = t;
        r->end = out.end;
        r->endS = t - tAcc;
        r->errEndM = (x - xAcc) - askRight;
      }
    }
    /* the forward axis tracks the reference: D from it */
    D = c->d0 * expf(-out.refFwd);
    const float vFwd = out.rateFwd * D;
    r->vFwdMax = fmaxf(r->vFwdMax, vFwd);
    /* the camera: a frame every FRAME_S, delivered CAM_LAG_S later */
    if (t >= nextFrameT) {
      nextFrameT += FRAME_S;
      if (tCap >= 0.0f) {
        ring[ringIn].t = t;   /* the frame just ended: its displacement over the exposure interval */
        ring[ringIn].dt = t - prevCapT;
        ring[ringIn].wLat = (x - prevCapX) / ring[ringIn].dt / D;
        ring[ringIn].wFwd = out.rateFwd;
        ring[ringIn].D = D;
        ring[ringIn].valid = true;
        ringIn = (ringIn + 1) & 3;
        (void)xCap;
        (void)dCap;
      }
      prevCapT = t;
      prevCapX = x;
      tCap = t;
    }
    if (ring[ringOut].valid && t >= ring[ringOut].t + CAM_LAG_S) {
      const frame_t pending = ring[ringOut];
      ring[ringOut].valid = false;
      ringOut = (ringOut + 1) & 3;
      /* a little measurement noise (the logged hovers: ~0.03 /s on wLat) */
      seed = seed * 1664525u + 1013904223u;
      const float noise = ((float)(seed >> 8) / 16777216.0f - 0.5f) * 0.02f;
      const uint8_t myMv = (uint8_t)(out.state | (out.kind << 4));
      const float G = AUTO_MV_FY_PX / pending.D;
      const uint16_t cvGain = (uint16_t)lrintf(G * 10.0f);
      const bool sok = camHoldSceneOk(&sc, true, pending.dt, 2.0f, 0.0f, pending.wFwd * pending.dt);
      float tgt = camHoldGainTarget(&gl, AUTO_HOVER, myMv, 0x05u, cvGain, 2.5f);
      camHoldGainMoveFix(&gl, true, speedPct > 100u ? CAM_HOLD_GAIN_GRACE_S : 0.0f, pending.dt);
      tgt = camHoldGainMove(&gl, tgt, s, AUTO_HOVER, myMv, out.dLowM, out.refFwd, h.epoch == out.epoch ? h.pFwd : 0.0f);
      s = camHoldGainSlew(s, tgt, pending.dt, camHoldGainRate(&gl, AUTO_HOVER));
      camHoldInput_t hin;
      memset(&hin, 0, sizeof(hin));
      hin.enabled = true;
      hin.sticksCentred = true;
      hin.airborne = true;
      hin.viewOk = sok;
      hin.wFwd = pending.wFwd;
      hin.wLat = pending.wLat + noise;
      hin.dt = pending.dt;
      hin.refEpoch = out.epoch;
      hin.refSnapEpoch = out.snapEpoch;
      hin.refLat = out.refLat;
      hin.refFwd = out.refFwd;
      hin.rateLat = out.rateLat;
      hin.rateFwd = out.rateFwd;
      hin.ffRoll = out.ffRollDeg;
      hin.ffPitch = out.ffPitchDeg;
      hin.gainScale = s;
      hin.moveLimDeg = lim;
      hin.moveSlewDegS = slew;
      hin.ffBrakeScale = true;
      hin.latGain = latPct > 100u && camHoldMoveActive(myMv) ? (float)latPct * 0.01f : 1.0f;
      camHoldStep(&h, &hin);
      (void)camHoldMoving(&h, pending.dt);
    }
    /* the plant at 1 kHz: the controller applies the hold's roll at its slew, the attitude follows in 80 ms */
    {
      const float step = camHoldSlewOf(&h, &(camHoldInput_t){.moveLimDeg = lim, .moveSlewDegS = slew,
                                                               .ffRoll = out.ffRollDeg, .ffPitch = out.ffPitchDeg}) *
                         PLANT_DT;
      const float d = h.roll - applied;
      applied += d > step ? step : (d < -step ? -step : d);
      lean += (applied - lean) * (PLANT_DT / ATT_TAU_S);
      const bool moving = accepted && out.state == AUTO_MV_RUN;
      const float pull = moving ? c->kF * vFwd : 0.0f;
      const float a = G_MPS2 * tanf((lean + bias) * 0.0174533f) - DRAG * v + pull;
      v += a * PLANT_DT;
      x += v * PLANT_DT;
    }
    if (accepted && !ended) {
      r->errMaxM = fmaxf(r->errMaxM, fabsf((x - xAcc) - askRight));
      r->rollMax = fmaxf(r->rollMax, h.roll);
    }
    if (ended && t >= tEnd + AFTER_S) {
      r->errAfterM = (x - xAcc) - askRight;
      break;
    }
    t += PLANT_DT;
  }
  autoMotionSetMargins(NULL);
  autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
  autoMotionSetFwdRate(AUTO_MV_W_FWD_PCT);
  autoMotionSetFfRoll(0);
}

static void simulate(const moveCase_t *c, uint16_t latPct, int32_t ffr, result_t *r)
{
  simulateAt(c, latPct, ffr, 300, r);
}

/* The speed dependence (the user's observation of 2026-10-22 13:44: slower forward moves slide less; the logs' fit
 * slide_speed.py: the pull grows with the speed, f = -0.32 v on the pulled moves, and the slide per forward metre with
 * it): the two worst moves of today at mvspeed 150 / 200 / 300, the RUN-end error per forward metre, today's settings
 * against mvlatk 300 and both. */
static void speedSweep(bool check)
{
  /* today's NVS (300, mvwmax 30), the speed step lowered, and mvwmax lowered (the expansion cap, 0.01/s x D: with 30
   * it is what sets a 300 % forward move's speed at D 2-3 m, so 200 and 300 % fly the same profile) */
  static const struct { uint16_t speed; uint8_t wmax; const char *what; } cfgs[5] = {
    {300, 30, "today: mvspeed 300, mvwmax 30"}, {200, 30, "mvspeed 200, mvwmax 30"}, {150, 30, "mvspeed 150, mvwmax 30"},
    {300, 20, "mvspeed 300, mvwmax 20"}, {300, 15, "mvspeed 300, mvwmax 15 (the default cap)"},
  };
  static const int which[2] = {2, 3};   /* table3, table4 */
  printf("info: the speed dependence (RUN-end error per forward metre, m/m; + right), the two worst moves of today:\n");
  for (int w = 0; w < 2; w++) {
    const moveCase_t *c = &cases[which[w]];
    float perM[5][3], vTop[5];
    for (int q = 0; q < 5; q++) {
      simWmax = cfgs[q].wmax;
      for (int cfgI = 0; cfgI < 3; cfgI++) {
        result_t r;
        simulateAt(c, cfgI == 0 ? 100 : 300, cfgI == 2 ? 10 : 0, cfgs[q].speed, &r);
        perM[q][cfgI] = r.errRunEndM / r.fwdM;
        if (cfgI == 0) {
          vTop[q] = r.vFwdMax;
          printf("info:   %-20s %-40s top speed %.2f m/s, RUN %.1f s: today %+.2f m/m", c->name, cfgs[q].what, r.vFwdMax,
                 r.runS, perM[q][0]);
        } else {
          printf(", %s %+.2f", cfgI == 1 ? "mvlatk 300" : "both", perM[q][cfgI]);
        }
      }
      printf("\n");
    }
    simWmax = 30;
    if (check) {
      CHECK(vTop[1] == vTop[0] && vTop[2] < vTop[0] && vTop[3] < vTop[0] && vTop[4] < vTop[3],
            "%s: mvspeed 200 flies the same top speed as 300 (%.2f m/s: mvwmax 30 is the cap); 150 (%.2f), mvwmax 20 "
            "(%.2f) and 15 (%.2f) are slower", c->name, vTop[0], vTop[2], vTop[3], vTop[4]);
      CHECK(perM[0][0] < 0.0f && perM[2][0] > perM[0][0] && perM[3][0] > perM[0][0] && perM[4][0] > perM[3][0],
            "%s: today's slide per forward metre shrinks with the top speed (%+.2f at today's; %+.2f at 150 %%; %+.2f "
            "/ %+.2f at mvwmax 20 / 15 m/m): a slower move slides less, as observed", c->name, perM[0][0], perM[2][0],
            perM[3][0], perM[4][0]);
      for (int q = 0; q < 5; q++) {
        CHECK(fabsf(perM[q][1]) <= 0.6f * fabsf(perM[q][0]), "%s, %s: mvlatk 300 cuts the slide per metre %+.2f -> %+.2f "
              "(<= 0.6 x)", c->name, cfgs[q].what, perM[q][0], perM[q][1]);
      }
    }
  }
}

static void grid(void)
{
  static const uint16_t lats[4] = {100, 200, 300, 400};
  static const int32_t ffrs[5] = {0, 6, 10, 14, 18};
  printf("info: grid (RUN-end error over the nine moves: worst |e|, mean e on the 7 pulled moves, worst |e| on the 2 "
         "without a pull; then the same at the move's end):\n");
  for (int a = 0; a < 4; a++) {
    for (int b = 0; b < 5; b++) {
      float worst = 0.0f, meanP = 0.0f, worstNo = 0.0f, worstE = 0.0f, meanPE = 0.0f, worstNoE = 0.0f;
      int nP = 0;
      for (int k = 0; k < N_CASES; k++) {
        result_t r;
        simulate(&cases[k], lats[a], ffrs[b], &r);
        worst = fmaxf(worst, fabsf(r.errRunEndM));
        worstE = fmaxf(worstE, fabsf(r.errEndM));
        if (PULLED(k)) {
          meanP += r.errRunEndM;
          meanPE += r.errEndM;
          nP++;
        } else {
          worstNo = fmaxf(worstNo, fabsf(r.errRunEndM));
          worstNoE = fmaxf(worstNoE, fabsf(r.errEndM));
        }
      }
      printf("info:   mvlatk %3u mvffr %2ld: RUN end worst %.2f mean(pulled) %+.2f worst(no pull) %.2f | end worst %.2f "
             "mean(pulled) %+.2f worst(no pull) %.2f\n", lats[a], (long)ffrs[b], worst, meanP / (float)nP, worstNo,
             worstE, meanPE / (float)nP, worstNoE);
    }
  }
}

int main(int argc, char **argv)
{
  if (argc > 1 && strcmp(argv[1], "--grid") == 0) {
    grid();
    return 0;
  }
  if (argc > 1 && strcmp(argv[1], "--speed") == 0) {
    speedSweep(false);
    return 0;
  }
  static const struct { const char *what; uint16_t latPct; int32_t ffr; } cfg[4] = {
    {"today (mvlatk 100, mvffr 0)", 100, 0}, {"mvlatk 300", 300, 0}, {"mvffr 10", 100, 10}, {"mvlatk 300 + mvffr 10", 300, 10},
  };
  result_t res[4][N_CASES];
  printf("info: the nine logged forward moves closed loop: the lateral error (true position - asked, m; + right) when\n"
         "info: the RUN ends (the logs' number), at the move's end and 2 s after, the worst excursion, the hold's largest\n"
         "info: roll (deg), the end; the hover's drift while the hold learned the trim (before the move: not an error):\n");
  for (int q = 0; q < 4; q++) {
    printf("info: --- %s ---\n", cfg[q].what);
    for (int k = 0; k < N_CASES; k++) {
      simulate(&cases[k], cfg[q].latPct, cfg[q].ffr, &res[q][k]);
      const result_t *r = &res[q][k];
      printf("info:   %-33s pull %+.3f, logged %+.2f m: RUN end %+.2f m (%.1f s, v %.2f m/s), end %+.2f m, +2 s %+.2f m, "
             "worst %.2f m, roll max %+.2f deg, %s at %.1f s (hover drift %+.2f m)\n", cases[k].name, cases[k].kF,
             cases[k].loggedErrM, r->errRunEndM, r->runS, r->vFwdMax, r->errEndM, r->errAfterM, r->errMaxM, r->rollMax,
             autoMvEndName(r->end), r->endS, r->hoverDriftM);
    }
  }
  /* today's settings reproduce the slide: at the RUN end the same sign and size as the log on the pulled moves */
  for (int k = 0; k < N_CASES; k++) {
    const result_t *r = &res[0][k];
    CHECK(r->endS > 0.0f && r->end == AUTO_MVE_DONE, "%s: today's settings end the move DONE (%s at %.1f s)",
          cases[k].name, autoMvEndName(r->end), r->endS);
    if (PULLED(k)) {
      CHECK(r->errRunEndM <= -0.15f && r->errRunEndM >= -0.80f && r->errRunEndM <= 0.5f * cases[k].loggedErrM,
            "%s: today's settings slide the replay %+.2f m by the RUN end (the log %+.2f: left, 0.15-0.80 m, at least "
            "half the log's)", cases[k].name, r->errRunEndM, cases[k].loggedErrM);
    } else {
      CHECK(fabsf(r->errRunEndM) < 0.10f && fabsf(r->errEndM) < 0.10f, "%s: today's settings end within 0.10 m "
            "(%+.2f / %+.2f; the log %+.2f)", cases[k].name, r->errRunEndM, r->errEndM, cases[k].loggedErrM);
    }
  }
  /* each setting alone, and both */
  float sumA = 0.0f, sumB = 0.0f, sumC = 0.0f, sumD = 0.0f;
  for (int k = 0; k < N_CASES; k++) {
    const result_t *a = &res[0][k], *b = &res[1][k], *c = &res[2][k], *d = &res[3][k];
    CHECK(b->end == AUTO_MVE_DONE && c->end == AUTO_MVE_DONE && d->end == AUTO_MVE_DONE,
          "%s: every configuration ends DONE (%s / %s / %s): nothing stops a move the defaults finish", cases[k].name,
          autoMvEndName(b->end), autoMvEndName(c->end), autoMvEndName(d->end));
    if (PULLED(k)) {
      sumA += fabsf(a->errRunEndM);
      sumB += fabsf(b->errRunEndM);
      sumC += fabsf(c->errRunEndM);
      sumD += fabsf(d->errRunEndM);
      /* (10-17 table2 f2 hovers at +1.03 deg: the integral's 1.5 deg contribution limit leaves it 0.5 deg of position
       * authority, so the gain cuts its slide by 0.78 x only; the others 0.3-0.65 x) */
      CHECK(fabsf(b->errRunEndM) <= 0.8f * fabsf(a->errRunEndM) && fabsf(b->errEndM) <= fabsf(a->errEndM) + 0.02f &&
                b->errMaxM <= a->errMaxM + 0.02f,
            "%s: mvlatk 300 alone cuts the RUN-end error %+.2f -> %+.2f m (<= 0.8 x; the end %+.2f -> %+.2f, the worst "
            "excursion %.2f -> %.2f)", cases[k].name, a->errRunEndM, b->errRunEndM, a->errEndM, b->errEndM, a->errMaxM,
            b->errMaxM);
      CHECK(fabsf(c->errRunEndM) <= 0.75f * fabsf(a->errRunEndM), "%s: mvffr 10 alone cuts the RUN-end error %+.2f -> "
            "%+.2f m (<= 0.75 x)", cases[k].name, a->errRunEndM, c->errRunEndM);
      CHECK(fabsf(d->errRunEndM) <= 0.20f && fabsf(d->errEndM) <= 0.15f && fabsf(d->errAfterM) <= 0.15f,
            "%s: both end within 0.20 m at the RUN end, 0.15 m at the end and 2 s after (%+.2f / %+.2f / %+.2f; today "
            "%+.2f / %+.2f / %+.2f)", cases[k].name, d->errRunEndM, d->errEndM, d->errAfterM, a->errRunEndM, a->errEndM,
            a->errAfterM);
    } else {
      CHECK(fabsf(b->errRunEndM) <= 0.10f && fabsf(c->errRunEndM) <= 0.20f && fabsf(d->errRunEndM) <= 0.20f &&
                fabsf(c->errEndM) <= 0.20f && fabsf(d->errEndM) <= 0.20f,
            "%s (no pull): mvlatk 300 costs at most 0.10 m (%+.2f), mvffr 10 at most 0.20 m the other way (%+.2f, the "
            "end %+.2f), both 0.20 (%+.2f / %+.2f)", cases[k].name, b->errRunEndM, c->errRunEndM, c->errEndM,
            d->errRunEndM, d->errEndM);
    }
  }
  printf("info: the 7 pulled moves' mean |RUN-end error|: today %.2f m, mvlatk 300 %.2f, mvffr 10 %.2f, both %.2f\n",
         sumA / 7.0f, sumB / 7.0f, sumC / 7.0f, sumD / 7.0f);
  CHECK(sumB <= 0.5f * sumA && sumC <= 0.5f * sumA && sumD <= 0.35f * sumA,
        "over the pulled moves: mvlatk 300 and mvffr 10 each halve the mean RUN-end error, both cut it to a third "
        "(%.2f -> %.2f / %.2f / %.2f m)", sumA / 7.0f, sumB / 7.0f, sumC / 7.0f, sumD / 7.0f);
  speedSweep(true);
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
