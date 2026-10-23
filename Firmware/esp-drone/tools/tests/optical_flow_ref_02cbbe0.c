/*
 * ESP-Drone Firmware
 *
 * optical_flow.c - Global image-motion estimate between consecutive frames
 */
#include <math.h>
#include <string.h>

#include "optical_flow.h"

#define W0 OF_WIDTH
#define H0 OF_HEIGHT
#define W1 (OF_WIDTH / 2)
#define H1 (OF_HEIGHT / 2)
#define NP 5           /* tx, ty, s, r, c */
#define MARGIN 4       /* pixels kept clear of the border at each level */

static void downsample(const uint8_t *src, uint8_t *dst)
{
  for (int y = 0; y < H1; y++) {
    const uint8_t *r0 = src + (2 * y) * W0;
    const uint8_t *r1 = r0 + W0;
    for (int x = 0; x < W1; x++) {
      dst[y * W1 + x] = (uint8_t)((r0[2 * x] + r0[2 * x + 1] + r1[2 * x] + r1[2 * x + 1] + 2) >> 2);
    }
  }
}

void ofLumaFromYuyv(const uint8_t *yuyv, uint8_t *y)
{
  /* little-endian words: w0 = Y0 | U0 << 8 | Y1 << 16 | V0 << 24, w1 = Y2 | U1 << 8 | Y3 << 16 | V1 << 24 (byte
   * buffers read and written as words: may_alias) */
  typedef uint32_t __attribute__((may_alias)) word_t;
  const word_t *src = (const word_t *)yuyv;
  word_t *dst = (word_t *)y;
  for (int i = 0; i < W0 * H0 / 4; i++) {
    const uint32_t w0 = src[2 * i], w1 = src[2 * i + 1];
    dst[i] = (w0 & 0xFFu) | ((w0 >> 8) & 0xFF00u) | ((w1 & 0xFFu) << 16) | ((w1 << 8) & 0xFF000000u);
  }
}

bool ofSolve(float *A, float *b, int n)
{
  for (int c = 0; c < n; c++) {
    int piv = c;
    for (int r = c + 1; r < n; r++) {
      if (fabsf(A[r * n + c]) > fabsf(A[piv * n + c])) {
        piv = r;
      }
    }
    if (!(fabsf(A[piv * n + c]) > 1e-9f)) {
      return false;
    }
    if (piv != c) {
      for (int k = 0; k < n; k++) {
        float t = A[c * n + k];
        A[c * n + k] = A[piv * n + k];
        A[piv * n + k] = t;
      }
      float t = b[c];
      b[c] = b[piv];
      b[piv] = t;
    }
    for (int r = c + 1; r < n; r++) {
      float f = A[r * n + c] / A[c * n + c];
      for (int k = c; k < n; k++) {
        A[r * n + k] -= f * A[c * n + k];
      }
      b[r] -= f * b[c];
    }
  }
  for (int c = n - 1; c >= 0; c--) {
    float s = b[c];
    for (int k = c + 1; k < n; k++) {
      s -= A[c * n + k] * b[k];
    }
    b[c] = s / A[c * n + c];
  }
  return true;
}

/* Saturation (a window): the template pixel or a neighbour its gradient uses is at or above OF_SAT_LEVEL, or the
 * template pixel is where the current frame, a / 1 times brighter, clips (a < 1). A test of the template only, so
 * the pixel set does not depend on the motion (a test of the warped sample would refuse pixels by the sign of their
 * residual and bias the shift). */
static inline bool saturated(const uint8_t *t, int w, float satT)
{
  return (float)t[0] >= satT || t[1] >= OF_SAT_LEVEL || t[-1] >= OF_SAT_LEVEL || t[w] >= OF_SAT_LEVEL ||
         t[-w] >= OF_SAT_LEVEL;
}

/* What one refineLevel() call does besides the plain fit (each a specialised copy of the pixel loop). */
typedef enum {
  OFK_PLAIN = 0,   /* the plain fit */
  OFK_STATS,       /* the plain fit, and in its first iteration the gain's flat-pixel statistics and the saturated
                    * pixels counted (satFrac) */
  OFK_GAIN,        /* the gain in the model */
  OFK_GAIN_MASK,   /* the gain in the model, the saturated pixels left out */
} ofKind_t;

typedef struct {
  float sI, sJ;     /* sums over the flat unsaturated pixels: template, warped sample */
  int nF;
  int nSat;         /* saturated pixels of the last iteration (left out, or only counted) */
  int nIn;          /* the counted iteration's pixels (in bounds) */
} ofStats_t;

/*
 * Gauss-Newton refinement of p = {tx, ty, s, r, c} at one pyramid level,
 * using the template (previous frame) gradients. s and r are estimated with
 * centre-relative coordinates normalised by k = 1/cx for conditioning.
 * kind is a constant in every call (refineLevel): the compiler keeps one
 * loop per kind, and OFK_PLAIN / OFK_STATS / OFK_COUNT compute the plain
 * fit's arithmetic exactly.
 */
static inline __attribute__((always_inline)) bool refineKind(const uint8_t *I, const uint8_t *J, int w, int h,
                                                             int step, int iters, float p[NP], float *texture,
                                                             float *residual, int *count, const ofKind_t kind,
                                                             float a, int statRows, ofStats_t *stats)
{
  const float cx = 0.5f * (w - 1), cy = 0.5f * (h - 1);
  const float k = 1.0f / cx;
  const bool gainK = kind == OFK_GAIN || kind == OFK_GAIN_MASK;
  const float satT = (float)OF_SAT_LEVEL * fminf(a, 1.0f);
  bool ok = true;

  for (int it = 0; it < iters; it++) {
    float A[NP * NP] = {0};
    float b[NP] = {0};
    float sse = 0.0f;
    int n = 0, nSat = 0;

    for (int y = MARGIN; y < h - MARGIN; y++) {
      const bool statRow = kind == OFK_STATS && it == 0 && (y % statRows) == 0;
      for (int x = MARGIN + (step > 1 ? (y & 1) : 0); x < w - MARGIN; x += step) {
        const float xr = x - cx, yr = y - cy;
        const float wx = x + p[0] + p[2] * xr - p[3] * yr;
        const float wy = y + p[1] + p[2] * yr + p[3] * xr;
        if (wx < 0.0f || wy < 0.0f || wx >= (float)(w - 1) || wy >= (float)(h - 1)) {
          continue;
        }
        const int ix = (int)wx, iy = (int)wy;
        const float fx = wx - ix, fy = wy - iy;
        const uint8_t *q = J + iy * w + ix;
        const float top = q[0] + fx * (q[1] - q[0]);
        const float bot = q[w] + fx * (q[w + 1] - q[w]);
        const float jv = top + fy * (bot - top);

        const uint8_t *t = I + y * w + x;
        if (kind == OFK_GAIN_MASK && saturated(t, w, satT)) {
          nSat++;
          continue;   /* clipped: no motion, and no exposure gain, in it */
        }
        const float gx = 0.5f * ((float)t[1] - (float)t[-1]);
        const float gy = 0.5f * ((float)t[w] - (float)t[-w]);
        if (kind == OFK_STATS && it == 0) {
          if (t[0] >= OF_SAT_LEVEL) {
            nSat++;   /* only counted: the pixel itself */
          } else if (statRow && fabsf(gx) + fabsf(gy) <= OF_GAIN_FLAT_GRAD && t[0] >= 8) {
            stats->sI += (float)t[0];
            stats->sJ += jv;
            stats->nF++;
          }
        }
        const float xn = xr * k, yn = yr * k;
        const float G[NP] = {gx, gy, gx * xn + gy * yn, gy * xn - gx * yn, 1.0f};
        const float e = gainK ? (float)t[0] - a * jv - p[4] : (float)t[0] - jv - p[4];

        for (int i = 0; i < NP; i++) {
          b[i] += G[i] * e;
          for (int j = 0; j <= i; j++) {
            A[i * NP + j] += G[i] * G[j];
          }
        }
        sse += e * e;
        n++;
      }
    }

    if (n < 100) {
      return false;
    }
    for (int i = 0; i < NP; i++) {
      for (int j = i + 1; j < NP; j++) {
        A[i * NP + j] = A[j * NP + i];
      }
    }

    /* Texture: smallest eigenvalue of the translation block, per in-bounds pixel (the left-out saturated ones
     * included: a window does not loosen the texture gate). */
    const float a00 = A[0], c11 = A[NP + 1], bb = A[1];
    const float half = 0.5f * (a00 + c11);
    const float disc = sqrtf(0.25f * (a00 - c11) * (a00 - c11) + bb * bb);
    *texture = (half - disc) / (kind == OFK_GAIN_MASK ? n + nSat : n);
    *residual = sqrtf(sse / n);
    *count = n;
    if (kind == OFK_GAIN_MASK || (kind == OFK_STATS && it == 0)) {
      stats->nSat = nSat;
      stats->nIn = kind == OFK_GAIN_MASK ? n + nSat : n;
    }

    if (!ofSolve(A, b, NP)) {
      ok = false;
      break;
    }
    p[0] += b[0];
    p[1] += b[1];
    p[2] += b[2] * k;
    p[3] += b[3] * k;
    p[4] += b[4];
  }
  return ok;
}

static bool refineLevel(const uint8_t *I, const uint8_t *J, int w, int h, int step, int iters, float p[NP],
                        float *texture, float *residual, int *count, ofKind_t kind, float a, int statRows,
                        ofStats_t *stats)
{
  switch (kind) {
    case OFK_STATS:
      return refineKind(I, J, w, h, step, iters, p, texture, residual, count, OFK_STATS, 1.0f, statRows, stats);
    case OFK_GAIN:
      return refineKind(I, J, w, h, step, iters, p, texture, residual, count, OFK_GAIN, a, 1, stats);
    case OFK_GAIN_MASK:
      return refineKind(I, J, w, h, step, iters, p, texture, residual, count, OFK_GAIN_MASK, a, 1, stats);
    default:
      return refineKind(I, J, w, h, step, iters, p, texture, residual, count, OFK_PLAIN, 1.0f, 1, stats);
  }
}

void ofInit(ofState_t *st)
{
  const uint8_t mode = st->mode;
  memset(st, 0, sizeof(*st));
  st->mode = mode <= OF_FIT_PLAIN ? mode : OF_FIT_ROBUST;
}

bool ofExposureStepTooLarge(const ofResult_t *r, float lum, float prevLum)
{
  if (r->flatPixels >= OF_GAIN_MIN_PX) {
    return !(fabsf(r->gain - 1.0f) <= OF_GAIN_STEP_MAX);
  }
  /* no gain measured (OF_FIT_PLAIN, or too few flat pixels): the mean brightness */
  return prevLum > 0.0f && fabsf(lum - prevLum) > OF_GAIN_STEP_MAX * prevLum;
}

float ofGainFromStats(float sI, float sJ, int nF)
{
  if (nF < OF_GAIN_MIN_PX || !(sJ > 0.0f)) {
    return 1.0f;
  }
  const float a = fminf(fmaxf(sI / sJ, OF_GAIN_MIN), OF_GAIN_MAX);
  return fabsf(a - 1.0f) < OF_GAIN_DEADBAND ? 1.0f : a;
}

bool ofProcess(ofState_t *st, const uint8_t *frame, const float init[2], ofResult_t *out)
{
  memset(out, 0, sizeof(*out));
  downsample(frame, st->cur1);

  if (!st->havePrev) {
    memcpy(st->prev0, frame, sizeof(st->prev0));
    memcpy(st->prev1, st->cur1, sizeof(st->prev1));
    st->havePrev = true;
    out->gain = 1.0f;
    return false;
  }

  float p[NP] = {init ? 0.5f * init[0] : 0.0f, init ? 0.5f * init[1] : 0.0f, 0.0f, 0.0f, 0.0f};
  float texture = 0.0f, residual = 0.0f;
  int count = 0;
  ofStats_t stats = {0};
  float gain = 1.0f;
  bool ok, step = false;
  if (st->mode == OF_FIT_PLAIN) {
    ok = refineLevel(st->prev1, st->cur1, W1, H1, 1, 3, p, &texture, &residual, &count, OFK_PLAIN, 1.0f, 1, &stats);
    if (ok) {
      p[0] *= 2.0f;
      p[1] *= 2.0f;
      ok = refineLevel(st->prev0, frame, W0, H0, 2, 2, p, &texture, &residual, &count, OFK_PLAIN, 1.0f, 1, &stats);
    }
  } else {
    /* Coarse: the plain fit's iterations, the gain's statistics in iteration OF_GAIN_ITER (at a warp near the
     * motion), then the gain: a frame without an exposure step (|a - 1| under the dead band) is fitted plain to the
     * end, bit for bit; a step frame fits the gain from the next iteration on and leaves the saturated pixels out
     * (OF_FIT_FINE_MASK: on the fine level only). */
    ok = OF_GAIN_ITER == 0 ||
         refineLevel(st->prev1, st->cur1, W1, H1, 1, OF_GAIN_ITER, p, &texture, &residual, &count, OFK_PLAIN, 1.0f, 1,
                     &stats);
    ok = ok && refineLevel(st->prev1, st->cur1, W1, H1, 1, 1, p, &texture, &residual, &count, OFK_STATS, 1.0f,
                           st->mode == OF_FIT_FINE_MASK ? 2 : 1, &stats);
    gain = ofGainFromStats(stats.sI, stats.sJ, stats.nF);
    step = gain != 1.0f;
    const ofKind_t coarse = !step ? OFK_PLAIN : (st->mode == OF_FIT_ROBUST ? OFK_GAIN_MASK : OFK_GAIN);
    if (ok && 2 - OF_GAIN_ITER > 0) {
      ok = refineLevel(st->prev1, st->cur1, W1, H1, 1, 2 - OF_GAIN_ITER, p, &texture, &residual, &count, coarse,
                       gain, 1, &stats);
    }
    if (ok) {
      p[0] *= 2.0f;
      p[1] *= 2.0f;
      /* fine: masked on a step frame (satFrac then the fine level's), plain on the others (satFrac the coarse
       * level's, counted in the statistics' iteration) */
      ok = refineLevel(st->prev0, frame, W0, H0, 2, 2, p, &texture, &residual, &count,
                       step ? OFK_GAIN_MASK : OFK_PLAIN, gain, 1, &stats);
    }
  }

  memcpy(st->prev0, frame, sizeof(st->prev0));
  memcpy(st->prev1, st->cur1, sizeof(st->prev1));

  for (int i = 0; i < NP; i++) {
    ok = ok && isfinite(p[i]);
  }
  out->tx = p[0];
  out->ty = p[1];
  out->scale = p[2];
  out->rot = p[3];
  out->bias = p[4];
  out->texture = texture;
  out->residual = residual;
  out->pixels = count;
  out->gain = gain;
  out->flatPixels = stats.nF;
  out->satFrac = stats.nIn > 0 ? (float)stats.nSat / (float)stats.nIn : 0.0f;
  out->ok = ok;
  return ok;
}
