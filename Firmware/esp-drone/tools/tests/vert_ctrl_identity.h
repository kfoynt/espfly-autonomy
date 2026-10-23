/*
 * The vertical controller's outputs over whole simulated sorties, hashed bit
 * for bit (tools/tests/test_vert_ctrl.c). Only the input and output fields
 * the controller had before the interactive hover (8f51285) are used (the
 * new inputs vRef and hOff are zero by the designated initializer), so the
 * same function compiles against vert_ctrl before and after: the hash was
 * taken from 8f51285's vert_ctrl.c, and with vRef and hOff 0 the controller
 * must reproduce it exactly, for every take-off height of the envelope that
 * existed then (0.20..0.60 m).
 *
 * Sorties: heights 0.20..0.60 m, plant gain errors, in-flight accelerometer
 * offsets, camera on (healthy, a camera offset c, a loss window) and off,
 * Mac LAND at different times (CLIMB, HOLD), the HOLD timeout, a NaN input.
 */
#ifndef VERT_CTRL_IDENTITY_H_
#define VERT_CTRL_IDENTITY_H_

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "vert_ctrl.h"

static uint64_t vzIdMix(uint64_t h, const void *p, size_t n)
{
  const uint8_t *b = (const uint8_t *)p;
  for (size_t i = 0; i < n; i++) {
    h ^= b[i];
    h *= 0x100000001B3ull;   /* FNV-1a 64 */
  }
  return h;
}

static uint64_t vzIdOut(uint64_t h, const vzOutput_t *o)
{
  const float f[17] = {o->thrust, o->h, o->v, o->vHp, o->aRel, o->g0, o->khat, o->kfrz, o->tLo, o->tHold, o->tRef,
                       o->ceiling, o->tAir, o->tPhase, (float)o->phase, (float)o->reason, (float)o->landReason};
  const uint8_t b[9] = {o->liftedOff, o->liftoffNow, o->holdReached, o->holdSteady, o->atCap, o->tdThrustOk,
                        o->touchdown, o->camHealthy, o->contact};
  h = vzIdMix(h, f, sizeof(f));
  return vzIdMix(h, b, sizeof(b));
}

/* One sortie: TAKEOFF until landAtS (s of controller time, < 0: never), then LAND, for maxS. */
static uint64_t vzIdSortie(uint64_t h, float hTarget, float kErr, float accAirG, float camWakeS, float camC,
                           float camLossS, float landAtS, float nanAtS, float maxS, uint32_t seed)
{
  vzConfig_t c;
  vzDefaultConfig(&c);
  vzConfigSetHeight(&c, hTarget);
  vzState_t st;
  vzInit(&st, &c);
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.kTrue = c.kPrior * (1.0f + kErr);
  pc.accAirG = accAirG;
  pc.camWakeS = camWakeS;
  pc.camVzOffset = camC;
  pc.camLossS = camLossS;
  pc.camLossLenS = 2.0f;
  vzSimState_t sim;
  vzSimInit(&sim, &pc, seed);
  vzOutput_t o;
  memset(&o, 0, sizeof(o));
  float applied = 0.0f, vbat = sim.vbat;
  const int n = (int)(maxS * 1000.0f);
  for (int k = 0; k < n; k++) {
    const float t = (float)k * 0.001f;
    if (k % 100 == 0) {
      vbat = sim.vbat;
    }
    vzInput_t in = {
      .dt = 0.001f, .cmd = (landAtS >= 0.0f && t >= landAtS) ? VZ_CMD_LAND : VZ_CMD_TAKEOFF, .accZ = sim.accZ,
      .gravZ = 1.0f, .tiltDeg = 0.5f, .gyroMaxDps = 0.3f, .vbat = vbat, .thrustApplied = applied, .fromSim = false,
      .camHealthy = sim.camHealthy, .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias,
    };
    if (nanAtS >= 0.0f && t >= nanAtS && t < nanAtS + 0.002f) {
      in.accZ = NAN;
    }
    vzStep(&st, &in, &o);
    h = vzIdOut(h, &o);
    vzSimStep(&sim, &pc, o.thrust, 0.001f);
    applied = o.thrust;
    if (o.phase == VZ_DONE || o.phase == VZ_CUT) {
      break;
    }
  }
  float k = c.kPrior;
  const bool learned = vzLearnK(&st, &k);
  const float tail[2] = {learned ? 1.0f : 0.0f, k};
  return vzIdMix(h, tail, sizeof(tail));
}

static uint64_t vertCtrlIdentityHash(void)
{
  uint64_t h = 0xCBF29CE484222325ull;
  const float heights[5] = {0.20f, 0.30f, 0.40f, 0.50f, 0.60f};
  uint32_t seed = 17;
  for (int i = 0; i < 5; i++) {
    const float ht = heights[i];
    h = vzIdSortie(h, ht, 0.0f, -0.0023f, 0.0f, 0.0f, -1.0f, 9.0f, -1.0f, 20.0f, seed++);     /* camera, Mac LAND */
    h = vzIdSortie(h, ht, 0.04f, 0.002f, 1.5f, 0.03f, -1.0f, 7.0f, -1.0f, 20.0f, seed++);     /* heavier, c +0.03 */
    h = vzIdSortie(h, ht, -0.05f, -0.006f, -1.0f, 0.0f, -1.0f, -1.0f, -1.0f, 20.0f, seed++); /* no camera */
    h = vzIdSortie(h, ht, 0.02f, 0.0f, 0.0f, -0.04f, 2.0f, 8.0f, -1.0f, 20.0f, seed++);      /* camera loss */
    h = vzIdSortie(h, ht, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 2.2f, -1.0f, 12.0f, seed++);        /* LAND in CLIMB */
    h = vzIdSortie(h, ht, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, -1.0f, 4.0f, 6.0f, seed++);         /* NaN: CUT */
  }
  return h;
}

#endif /* VERT_CTRL_IDENTITY_H_ */
