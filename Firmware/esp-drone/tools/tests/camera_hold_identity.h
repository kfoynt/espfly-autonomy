/*
 * The camera drift hold's outputs over a long deterministic input sequence,
 * hashed bit for bit (tools/tests/test_camera_hold.c). It uses only the
 * input fields the hold had before the moving reference (8f51285), the rest
 * are zero by the designated initializer, so the same function compiles
 * against the hold before and after that change: the hash below was taken
 * from 8f51285's camera_hold.c, and the hold with every reference field 0 must
 * reproduce it exactly (the interactive hover's "no motion: bit-identical").
 *
 * Regimes: steady hover with noise and slow drifts, view dropouts and
 * coasting, stick takeovers and landings (integral resets), frame gaps, NaN,
 * negative and zero dt, saturating drifts (clamps and the slew).
 */
#ifndef CAMERA_HOLD_IDENTITY_H_
#define CAMERA_HOLD_IDENTITY_H_

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "camera_hold.h"

#define HOLD_IDENTITY_STEPS 200000

static uint32_t holdIdRng = 0x1234567u;

static float holdIdUni(void)
{
  holdIdRng ^= holdIdRng << 13;
  holdIdRng ^= holdIdRng >> 17;
  holdIdRng ^= holdIdRng << 5;
  return (float)(holdIdRng >> 8) * (1.0f / 16777216.0f);
}

static uint64_t holdIdMix(uint64_t h, const void *p, size_t n)
{
  const uint8_t *b = (const uint8_t *)p;
  for (size_t i = 0; i < n; i++) {
    h ^= b[i];
    h *= 0x100000001B3ull;   /* FNV-1a 64 */
  }
  return h;
}

static uint64_t holdIdentityHash(void)
{
  camHoldState_t st;
  camHoldReset(&st);
  holdIdRng = 0x1234567u;
  uint64_t h = 0xCBF29CE484222325ull;
  float driftF = 0.0f, driftL = 0.0f;
  for (int k = 0; k < HOLD_IDENTITY_STEPS; k++) {
    const int regime = (k / 5000) % 6;
    camHoldInput_t in = {
      .enabled = true, .sticksCentred = true, .airborne = true, .viewOk = true, .wFwd = 0.0f, .wLat = 0.0f,
      .dt = 0.06f + 0.02f * holdIdUni(),
    };
    if (holdIdUni() < 0.002f) {
      driftF = (holdIdUni() - 0.5f) * 0.4f;
      driftL = (holdIdUni() - 0.5f) * 0.4f;
    }
    in.wFwd = driftF + 0.05f * (holdIdUni() - 0.5f);
    in.wLat = driftL + 0.05f * (holdIdUni() - 0.5f);
    switch (regime) {
      case 1:   /* view dropouts: coasting */
        in.viewOk = holdIdUni() > 0.3f;
        break;
      case 2:   /* stick takeovers and landings */
        in.sticksCentred = holdIdUni() > 0.02f;
        in.airborne = holdIdUni() > 0.01f;
        in.enabled = holdIdUni() > 0.005f;
        break;
      case 3:   /* frame gaps, bad dt, NaN */
        if (holdIdUni() < 0.05f) {
          in.dt = 0.5f;
        } else if (holdIdUni() < 0.02f) {
          in.dt = 0.0f;
        } else if (holdIdUni() < 0.02f) {
          in.dt = -0.05f;
        }
        if (holdIdUni() < 0.01f) {
          in.wFwd = NAN;
        }
        if (holdIdUni() < 0.01f) {
          in.wLat = INFINITY;
        }
        break;
      case 4:   /* saturating drifts */
        in.wFwd = 3.0f * (holdIdUni() - 0.3f);
        in.wLat = -2.0f * holdIdUni();
        break;
      default:
        break;
    }
    const bool r = camHoldStep(&st, &in);
    const float out[10] = {st.roll, st.pitch, st.fwdF, st.latF, st.iFwd, st.iLat, st.airborneTime, st.idleTime,
                           r ? 1.0f : 0.0f, (st.engaged ? 2.0f : 0.0f) + (st.coasting ? 1.0f : 0.0f)};
    h = holdIdMix(h, out, sizeof(out));
  }
  return h;
}

#endif /* CAMERA_HOLD_IDENTITY_H_ */
