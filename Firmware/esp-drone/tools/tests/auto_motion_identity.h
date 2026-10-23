/*
 * The interactive hover's motions (auto_motion.c) over a long deterministic script, hashed bit for bit
 * (tools/tests/test_auto_motion.c testSpeedIdentity; mvspeed, 2026-10-16, docs/autonomy-a2b.md 31). It uses only the
 * API auto_motion.h had at 7ab74b9, so the same function compiles against the motions before the speed step: the hash
 * in test_auto_motion.c was taken from 7ab74b9's auto_motion.c, and the motions at mvspeed 100 (autoMotionInit alone,
 * or autoMotionSetSpeed(100) after it) must reproduce it exactly.
 *
 * Hashed on every 1 ms step: autoMotionStep's outputs (autoMvOut_t) and stop request, every command's answer, and the
 * motion memory up to its 7ab74b9 end (shOff0; the speed fields appended after it are not part of the old layout).
 * The camera: the measured progress follows the lagged reference (a 0.25 s lag, a gain error per scenario, noise),
 * the epoch seen 80 ms after it changes; the scale tracker loses its G 20 deg into a turn and regains it after 1.2 s
 * of bob; the height estimate follows the vertical reference. Scenarios (each from autoMotionInit): forward, sideways,
 * diagonal, back, up/down moves and a STOP; turns with a re-track bob, a small turn, a turn that loses its view (turn
 * back) and a blind turn; view losses (a pause, NO_CAM), a tilt, a frozen view, an overshoot (PROGRESS) and a short
 * settle (SHORT); SET_HEIGHT in both forms, the take-off's plan, a contact, a loop gap, leaving HOVER, a snap, the
 * idle outputs; DRY on the sim plant; refusals (BATT, RANGE, TOO_CLOSE, TOO_WIDE, NO_DIST, BUSY).
 */
#ifndef AUTO_MOTION_IDENTITY_H_
#define AUTO_MOTION_IDENTITY_H_

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "auto_motion.h"

typedef void (*autoMvIdSetup_t)(autoMotion_t *m);

typedef struct {
  autoMotion_t m;
  autoMvIn_t in;
  autoMvOut_t out;
  uint64_t h;
  uint32_t rng;
  /* the camera */
  uint32_t lastEpoch, epochAtMs;
  float gainLat, gainFwd;   /* measured / reference */
  bool frozen, viewLost;
  float yawAtTrack;         /* the heading offset the tracker last had a G at */
  float bobMs;              /* bob time since the tracker lost its G */
  float tiltHigh;
  autoMvIdSetup_t setup;
} autoMvId_t;

static float autoMvIdUni(autoMvId_t *r)
{
  r->rng ^= r->rng << 13;
  r->rng ^= r->rng >> 17;
  r->rng ^= r->rng << 5;
  return (float)(r->rng >> 8) * (1.0f / 16777216.0f);
}

static void autoMvIdMix(autoMvId_t *r, const void *p, size_t n)
{
  const uint8_t *b = (const uint8_t *)p;
  for (size_t i = 0; i < n; i++) {
    r->h ^= b[i];
    r->h *= 0x100000001B3ull;   /* FNV-1a 64 */
  }
}

static void autoMvIdMixU(autoMvId_t *r, uint32_t v)
{
  autoMvIdMix(r, &v, sizeof(v));
}

/* A new scenario: the motion memory from autoMotionInit (then the setup, the speed step), the inputs of a hover at
 * scene distance d. */
static void autoMvIdStart(autoMvId_t *r, float d, float gainLat, float gainFwd)
{
  autoMotionInit(&r->m);
  if (r->setup != NULL) {
    r->setup(&r->m);
  }
  const uint32_t now = r->in.nowMs + 1000u;
  memset(&r->in, 0, sizeof(r->in));
  r->in.nowMs = now;
  r->in.flight = true;
  r->in.sceneDM = d;
  r->in.sceneTracked = true;
  r->in.holdApplied = true;
  r->in.camFrameAgeMs = 90;
  r->in.tiltDeg = 1.0f;
  r->in.camHeight = true;
  r->in.vzH = 0.80f;
  r->in.hTargetM = 0.80f;
  r->in.airMs = 20000;
  r->lastEpoch = 0;
  r->epochAtMs = now;
  r->gainLat = gainLat;
  r->gainFwd = gainFwd;
  r->frozen = r->viewLost = false;
  r->yawAtTrack = 0.0f;
  r->bobMs = 0.0f;
  r->tiltHigh = 0.0f;
}

static void autoMvIdStep(autoMvId_t *r)
{
  autoMvIn_t *in = &r->in;
  in->nowMs++;
  in->airMs++;
  in->holdApplied = !r->viewLost;
  in->camFrameAgeMs = r->viewLost ? 450u : 90u + (uint32_t)(20.0f * autoMvIdUni(r));
  in->tiltDeg = r->tiltHigh > 0.0f ? r->tiltHigh : 0.8f + 0.6f * autoMvIdUni(r);
  const autoMvLand_t land = autoMotionStep(&r->m, in, &r->out);
  autoMvIdMix(r, &r->out, sizeof(r->out));
  autoMvIdMixU(r, (uint32_t)land);
  autoMvIdMix(r, &r->m, offsetof(autoMotion_t, shOff0) + sizeof(float));
  if (land == AUTO_MVL_NO_CAM || land == AUTO_MVL_TILT) {
    /* the state machine's stop (auto_fsm.c mvStopReq) */
    autoMotionStop(&r->m, in, land == AUTO_MVL_TILT ? AUTO_MVE_TILT : AUTO_MVE_NO_VIEW);
    autoMotionIdleOut(&r->m, &r->out);
    autoMvIdMix(r, &r->out, sizeof(r->out));
  }
  /* the camera: the epoch seen 80 ms after it changed (its progress from 0), then the lagged reference tracked */
  if (r->out.epoch != r->lastEpoch) {
    r->lastEpoch = r->out.epoch;
    r->epochAtMs = in->nowMs;
  }
  if (in->holdEpochSeen != r->lastEpoch && in->nowMs - r->epochAtMs >= 80u) {
    in->holdEpochSeen = r->lastEpoch;
    in->progLat = in->progFwd = 0.0f;
  }
  if (in->holdEpochSeen == r->lastEpoch && !r->viewLost) {
    const float a = 0.001f / 0.25f;
    const float nl = 0.0004f * (autoMvIdUni(r) - 0.5f), nf = 0.0004f * (autoMvIdUni(r) - 0.5f);
    if (!r->frozen) {
      in->progLat += a * (r->gainLat * r->out.refLat - in->progLat) + nl;
      in->progFwd += a * (r->gainFwd * r->out.refFwd - in->progFwd) + nf;
    } else {
      in->progLat += nl;
      in->progFwd += nf;
    }
  }
  /* the height estimate follows the vertical reference */
  in->vzH += r->out.vRef * 0.001f * (0.95f + 0.1f * autoMvIdUni(r));
  /* the scale tracker: lost 20 deg into a turn, back after 1.2 s of bob */
  if (in->sceneTracked && fabsf(r->out.yawOffDeg - r->yawAtTrack) > 20.0f) {
    in->sceneTracked = false;
    in->sceneResets++;
    r->bobMs = 0.0f;
  }
  if (!in->sceneTracked && (r->out.flags & AUTO_MVF_BOB) != 0u) {
    r->bobMs += 1.0f;
    if (r->bobMs >= 1200.0f) {
      in->sceneTracked = true;
      r->yawAtTrack = r->out.yawOffDeg;
    }
  }
  if (in->sceneTracked) {
    r->yawAtTrack = r->m.yawOff;
  }
}

static void autoMvIdRun(autoMvId_t *r, int ms)
{
  for (int i = 0; i < ms; i++) {
    autoMvIdStep(r);
  }
}

/* Until the motion ends (at most maxMs), then settleMs more. */
static void autoMvIdUntilEnd(autoMvId_t *r, int maxMs, int settleMs)
{
  for (int i = 0; i < maxMs && autoMotionActive(&r->m); i++) {
    autoMvIdStep(r);
  }
  autoMvIdRun(r, settleMs);
}

static void autoMvIdMove(autoMvId_t *r, int16_t rc, int16_t fc, int16_t uc)
{
  autoMvIdMixU(r, (uint32_t)autoMotionMove(&r->m, &r->in, rc, fc, uc));
}

static void autoMvIdYaw(autoMvId_t *r, int16_t deg)
{
  autoMvIdMixU(r, (uint32_t)autoMotionYaw(&r->m, &r->in, deg));
}

static uint64_t autoMvIdentityHash(autoMvIdSetup_t setup)
{
  static autoMvId_t r;   /* (large: static) */
  memset(&r, 0, sizeof(r));
  r.h = 0xCBF29CE484222325ull;
  r.rng = 0x2468ACEu;
  r.setup = setup;
  r.in.nowMs = 5000u;

  /* 1: the moves of a mission at D 3 m, a STOP mid-move, a back move */
  autoMvIdStart(&r, 3.0f, 1.0f, 1.0f);
  autoMvIdRun(&r, 300);
  autoMvIdMove(&r, 0, 100, 0);
  autoMvIdRun(&r, 50);
  autoMvIdMove(&r, 20, 0, 0);   /* BUSY */
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMove(&r, 60, 0, 0);
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMove(&r, -70, 70, 0);
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMove(&r, 0, -50, 0);
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMove(&r, 0, 0, 20);
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMove(&r, 0, 0, -30);
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMove(&r, 0, 40, 0);
  autoMvIdRun(&r, 2000);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  autoMvIdRun(&r, 3000);
  autoMvIdMove(&r, 100, 0, 0);   /* a long sideways move after the earlier ones: the re-view bob */
  autoMvIdUntilEnd(&r, 30000, 500);

  /* 2: turns: a re-track bob, a small turn that keeps its G, a turn that loses its view (turned back), a blind turn,
   * a STOP in a bob, a tilt in a turn */
  autoMvIdStart(&r, 2.5f, 1.0f, 1.0f);
  autoMvIdRun(&r, 300);
  autoMvIdYaw(&r, 90);
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdYaw(&r, -15);
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdYaw(&r, 120);
  autoMvIdRun(&r, 1200);
  r.viewLost = true;
  autoMvIdRun(&r, 2500);
  r.viewLost = false;
  autoMvIdUntilEnd(&r, 30000, 500);
  r.viewLost = true;
  autoMvIdRun(&r, 50);
  autoMvIdYaw(&r, 100);   /* blind: NO_VIEW */
  autoMvIdYaw(&r, -60);   /* blind turn */
  autoMvIdRun(&r, 1500);
  r.viewLost = false;
  autoMvIdUntilEnd(&r, 30000, 500);
  autoMvIdYaw(&r, 180);
  autoMvIdRun(&r, 7200);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);   /* in the dwell or the bob: no effect */
  autoMvIdRun(&r, 300);
  r.tiltHigh = 9.0f;
  autoMvIdRun(&r, 150);
  r.tiltHigh = 0.0f;
  autoMvIdUntilEnd(&r, 30000, 500);
  autoMvIdYaw(&r, -45);
  autoMvIdRun(&r, 600);
  r.tiltHigh = 8.5f;
  autoMvIdRun(&r, 200);
  r.tiltHigh = 0.0f;
  autoMvIdUntilEnd(&r, 30000, 500);

  /* 3: view losses in a move (a pause, then NO_CAM), a tilt, a frozen view, an overshoot, a short settle */
  autoMvIdStart(&r, 3.5f, 1.0f, 1.0f);
  autoMvIdRun(&r, 300);
  autoMvIdMove(&r, 0, 120, 0);
  autoMvIdRun(&r, 2500);
  r.viewLost = true;
  autoMvIdRun(&r, 500);
  r.viewLost = false;
  autoMvIdRun(&r, 1500);
  r.viewLost = true;
  autoMvIdRun(&r, 2600);
  r.viewLost = false;
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMove(&r, 50, 0, 0);
  autoMvIdRun(&r, 1500);
  r.tiltHigh = 8.6f;
  autoMvIdRun(&r, 150);
  r.tiltHigh = 0.0f;
  autoMvIdUntilEnd(&r, 20000, 500);
  r.frozen = true;
  autoMvIdMove(&r, 0, 150, 0);
  autoMvIdUntilEnd(&r, 30000, 500);
  r.frozen = false;
  r.gainFwd = 1.6f;
  autoMvIdMove(&r, 0, 80, 0);
  autoMvIdUntilEnd(&r, 30000, 500);
  r.gainFwd = 0.8f;
  r.gainLat = 0.7f;
  autoMvIdMove(&r, 40, 60, 0);
  autoMvIdUntilEnd(&r, 30000, 500);

  /* 4: SET_HEIGHT, the plan, a contact, a loop gap, leaving HOVER, a snap, the idle outputs */
  autoMvIdStart(&r, 2.2f, 1.0f, 1.0f);
  autoMvIdRun(&r, 300);
  autoMvIdMixU(&r, (uint32_t)autoMotionSetHeight(&r.m, &r.in, 0.10f, -0.10f));
  r.in.vzH += 0.10f;
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMixU(&r, (uint32_t)autoMotionSetHeightRef(&r.m, &r.in, -0.05f, 0.20f, 0.15f));
  r.in.vzH -= 0.05f;
  autoMvIdUntilEnd(&r, 20000, 500);
  autoMvIdMixU(&r, (uint32_t)autoMotionMovePlan(&r.m, &r.in, -30, 90, 0));
  autoMvIdRun(&r, 2500);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_CONTACT);
  autoMvIdRun(&r, 3000);
  autoMotionSnap(&r.m);
  autoMvIdRun(&r, 500);
  autoMvIdMove(&r, 0, -60, 0);
  autoMvIdRun(&r, 1500);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_LOOP_GAP);
  autoMvIdRun(&r, 1000);
  autoMvIdMove(&r, 30, 30, 0);
  autoMvIdRun(&r, 1200);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_LEFT_HOVER);
  autoMotionIdleOut(&r.m, &r.out);
  autoMvIdMix(&r, &r.out, sizeof(r.out));
  autoMvIdMix(&r, &r.m, offsetof(autoMotion_t, shOff0) + sizeof(float));
  {
    const float fl = autoMotionHeightFloor(&r.in);
    autoMvIdMix(&r, &fl, sizeof(fl));
    const float ln = autoMotionFwdLn(&r.m);
    autoMvIdMix(&r, &ln, sizeof(ln));
  }

  /* 5: DRY on the sim plant: a forward move, a turn, a sideways move, a STOP */
  autoMvIdStart(&r, 2.0f, 1.0f, 1.0f);
  r.in.flight = false;
  r.in.simPlant = true;
  autoMvIdRun(&r, 300);
  autoMvIdMove(&r, 0, 100, 0);
  autoMvIdUntilEnd(&r, 20000, 300);
  autoMvIdYaw(&r, 90);
  autoMvIdUntilEnd(&r, 20000, 300);
  autoMvIdMove(&r, 60, 0, 0);
  autoMvIdUntilEnd(&r, 20000, 300);
  autoMvIdMove(&r, -60, 80, 10);
  autoMvIdRun(&r, 1800);
  autoMotionStop(&r.m, &r.in, AUTO_MVE_STOP_CMD);
  autoMvIdRun(&r, 2000);

  /* 6: refusals: BATT, RANGE, TOO_CLOSE, TOO_WIDE, NO_DIST, NO_VIEW, NO_CAM_HEIGHT, HEIGHT */
  autoMvIdStart(&r, 1.6f, 1.0f, 1.0f);
  autoMvIdRun(&r, 300);
  r.in.battLow = true;
  autoMvIdMove(&r, 0, 30, 0);
  autoMvIdYaw(&r, 30);
  r.in.battLow = false;
  autoMvIdMove(&r, 0, 450, 0);
  autoMvIdMove(&r, 0, 0, 0);
  autoMvIdYaw(&r, 0);
  autoMvIdYaw(&r, 181);
  autoMvIdMove(&r, 0, 130, 0);
  autoMvIdMove(&r, 100, 0, 0);
  autoMvIdMove(&r, 0, 0, -60);
  autoMvIdMove(&r, 0, 0, 40);
  r.in.vzH = 1.90f;
  r.in.hTargetM = 1.90f;
  autoMvIdMove(&r, 0, 0, 20);
  r.in.camHeight = false;
  autoMvIdMove(&r, 0, 0, -10);
  r.in.camHeight = true;
  r.viewLost = true;
  autoMvIdRun(&r, 5);
  autoMvIdMove(&r, 0, 50, 0);
  r.viewLost = false;
  r.in.sceneTracked = false;
  autoMvIdMove(&r, 0, 50, 0);
  r.in.sceneTracked = true;
  autoMvIdRun(&r, 100);
  autoMvIdMove(&r, 0, 50, 0);
  autoMvIdUntilEnd(&r, 20000, 500);
  return r.h;
}

#endif /* AUTO_MOTION_IDENTITY_H_ */
