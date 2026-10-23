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
#define MARGIN (4 * OF_SAMPLE_STRIDE)       /* pixels kept clear of the border at each level */

/* The calling core's cycle counter (the stage times, ofState_t.stageCycles); 0 on the host. */
#if defined(__XTENSA__)
static inline uint32_t ofCycles(void)
{
  uint32_t c;
  __asm__ __volatile__("rsr.ccount %0" : "=a"(c));
  return c;
}
#else
static inline uint32_t ofCycles(void)
{
  return 0;
}
#endif

/* Host tests only (tools/tests/test_flow_fast.c builds with -DOF_TEST_COUNT_RESTARTS): counts the Hessian reuse's
 * restarts so the test can hold their coverage; absent from the firmware (its object is unchanged). */
#ifdef OF_TEST_COUNT_RESTARTS
int ofTestRestarts;
#define OF_COUNT_RESTART() (ofTestRestarts++)
#else
#define OF_COUNT_RESTART() ((void)0)
#endif

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

/* One used pixel's terms of the normal equations: G[0..3] and the residual e (G[4] = 1). */
typedef struct {
  float g[4];
  float e;
} ofTerm_t;
#define OF_TERMS 32   /* buffered pixels (640 B of stack) */
#define OF_HESS  15   /* the Hessian's lower triangle */

/* The warp's p[3] * yr as 02cbbe0's firmware rounded it (refineKind): on the ESP32-S3 its own rounded product in
 * every kind but OFK_STATS; on the host fused, as the host compilers fuse 02cbbe0's expression. */
#if defined(__XTENSA__)
#define WARP_ROW_PRODUCT 1
#else
#define WARP_ROW_PRODUCT 0
#endif

/* The normal equations' sums of one pass. Each is the rolled loop's of 02cbbe0
 * (for i { b[i] += G[i] * e; for j <= i: A[i][j] += G[i] * G[j]; } sse += e * e)
 * pixel for pixel in the same order and with the same operation: one fused
 * multiply-add per pixel (the firmware's madd.s; fmaf() so that no compiler
 * splits it, e.g. a host vectoriser), and for the G[4] = 1 terms an add,
 * exactly the fma(1, x, s) the rolled loop does. Only the order between
 * different sums changes, and no sum depends on another. */
typedef struct {
  float a[10];   /* A[i][j], j <= i < 4: 00 10 11 20 21 22 30 31 32 33 */
  float a4[4];   /* A[4][j], j < 4: sums of G[j] */
  float b[5];
  float sse;
} ofSums_t;

/* A[i][j] (i, j < 4) over m buffered pixels: ten accumulators and four terms in registers. */
static void sumsHessian(ofSums_t *s, const ofTerm_t *t, int m)
{
  float a00 = s->a[0], a10 = s->a[1], a11 = s->a[2], a20 = s->a[3], a21 = s->a[4], a22 = s->a[5], a30 = s->a[6],
        a31 = s->a[7], a32 = s->a[8], a33 = s->a[9];
  for (int k = 0; k < m; k++) {
    const float g0 = t[k].g[0], g1 = t[k].g[1], g2 = t[k].g[2], g3 = t[k].g[3];
    a00 = fmaf(g0, g0, a00);
    a10 = fmaf(g1, g0, a10);
    a11 = fmaf(g1, g1, a11);
    a20 = fmaf(g2, g0, a20);
    a21 = fmaf(g2, g1, a21);
    a22 = fmaf(g2, g2, a22);
    a30 = fmaf(g3, g0, a30);
    a31 = fmaf(g3, g1, a31);
    a32 = fmaf(g3, g2, a32);
    a33 = fmaf(g3, g3, a33);
  }
  s->a[0] = a00;
  s->a[1] = a10;
  s->a[2] = a11;
  s->a[3] = a20;
  s->a[4] = a21;
  s->a[5] = a22;
  s->a[6] = a30;
  s->a[7] = a31;
  s->a[8] = a32;
  s->a[9] = a33;
}

/* b, sse and (hess) A[4][j], j < 4, over m buffered pixels. */
static void sumsGradient(ofSums_t *s, const ofTerm_t *t, int m, bool hess)
{
  float b0 = s->b[0], b1 = s->b[1], b2 = s->b[2], b3 = s->b[3], b4 = s->b[4], sse = s->sse;
  if (hess) {
    float a40 = s->a4[0], a41 = s->a4[1], a42 = s->a4[2], a43 = s->a4[3];
    for (int k = 0; k < m; k++) {
      const float g0 = t[k].g[0], g1 = t[k].g[1], g2 = t[k].g[2], g3 = t[k].g[3], e = t[k].e;
      b0 = fmaf(g0, e, b0);
      b1 = fmaf(g1, e, b1);
      b2 = fmaf(g2, e, b2);
      b3 = fmaf(g3, e, b3);
      b4 += e;
      a40 += g0;
      a41 += g1;
      a42 += g2;
      a43 += g3;
      sse = fmaf(e, e, sse);
    }
    s->a4[0] = a40;
    s->a4[1] = a41;
    s->a4[2] = a42;
    s->a4[3] = a43;
  } else {
    for (int k = 0; k < m; k++) {
      const float g0 = t[k].g[0], g1 = t[k].g[1], g2 = t[k].g[2], g3 = t[k].g[3], e = t[k].e;
      b0 = fmaf(g0, e, b0);
      b1 = fmaf(g1, e, b1);
      b2 = fmaf(g2, e, b2);
      b3 = fmaf(g3, e, b3);
      b4 += e;
      sse = fmaf(e, e, sse);
    }
  }
  s->b[0] = b0;
  s->b[1] = b1;
  s->b[2] = b2;
  s->b[3] = b3;
  s->b[4] = b4;
  s->sse = sse;
}

/* The buffered pixels into the sums (out of line: one copy for every pass kind). */
static void __attribute__((noinline)) sumsFlush(ofSums_t *s, const ofTerm_t *t, int m, bool hess)
{
  if (m <= 0) {
    return;
  }
  if (hess) {
    sumsHessian(s, t, m);
  }
  sumsGradient(s, t, m, hess);
}

/*
 * Gauss-Newton refinement of p = {tx, ty, s, r, c} at one pyramid level,
 * using the template (previous frame) gradients. s and r are estimated with
 * centre-relative coordinates normalised by k = 1/cx for conditioning.
 * kind is a constant in every call (refineLevel): the compiler keeps one
 * loop per kind, and OFK_PLAIN / OFK_STATS / OFK_COUNT compute the plain
 * fit's arithmetic exactly.
 *
 * The pixel loop computes each used pixel's terms (G[0..3], e) into a short
 * buffer; the sums run over the buffer in two short loops whose accumulators
 * stay in registers (sumsHessian / sumsGradient). Hessian reuse: A depends on
 * the template and the set of used pixels only, not on p, so an iteration
 * after one that used every pixel in bounds takes that iteration's A and
 * sums only b, sse and n; a pixel out of bounds there restarts the pass with
 * the full sums. Both are the rolled loop of 02cbbe0 bit for bit (see
 * ofSums_t; tools/tests/test_flow_fast.c).
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
  const float wMax = (float)(w - 1), hMax = (float)(h - 1);
  bool ok = true;
  ofTerm_t terms[OF_TERMS];
  float held[OF_HESS];   /* the last full pass's Hessian sums (valid while reuse) */
  bool reuse = false;    /* that pass used every pixel in bounds */

  for (int it = 0; it < iters; it++) {
    /* The warp, rounded as 02cbbe0's firmware rounds it. Its GCC forms the fused multiply-adds late, after moving
     * loop invariants out of the pixel loop: in the plain, gain and mask kinds (nothing stored in the loop could
     * change p) p[3] * yr left the loop as a rounded product of its own and was then subtracted; in OFK_STATS
     * (its statistics' stores could) it stayed in the loop, fused into the subtraction. Spelled out here with
     * fmaf() and the row's product, so that the code around it cannot move it (WARP_ROW_PRODUCT). The host
     * compilers fuse 02cbbe0's expression whole in every kind: the same fmaf() chain. The host test
     * (test_flow_fast.c) and the emulator check (docs/autonomy-a2b.md 30) hold both to 02cbbe0. */
    const float p0 = p[0], p1 = p[1], p2 = p[2], p3 = p[3];
    const bool rowProduct = WARP_ROW_PRODUCT && kind != OFK_STATS;
    ofSums_t s;
    int n, nSat;
    bool hess = !reuse;   /* this pass sums the Hessian */
    for (;;) {
      memset(&s, 0, sizeof(s));
      n = 0;
      nSat = 0;
      int m = 0;
      bool out = false;   /* a pixel out of bounds */
      for (int y = MARGIN; y < h - MARGIN; y += OF_SAMPLE_STRIDE) {
        const bool statRow = kind == OFK_STATS && it == 0 && (y % statRows) == 0;
        const float p3yr = p3 * (y - cy);   /* rowProduct: p[3] * yr, rounded on its own */
        for (int x = MARGIN + (step > 1 ? ((y / OF_SAMPLE_STRIDE) & 1) * OF_SAMPLE_STRIDE : 0); x < w - MARGIN; x += step * OF_SAMPLE_STRIDE) {
          const float xr = x - cx, yr = y - cy;
          const float wx0 = fmaf(p2, xr, x + p0);   /* x + p[0] + p[2] * xr */
          const float wx = rowProduct ? wx0 - p3yr : fmaf(-p3, yr, wx0);   /* - p[3] * yr */
          const float wy = fmaf(p3, xr, fmaf(p2, yr, y + p1));   /* y + p[1] + p[2] * yr + p[3] * xr */
          if (__builtin_expect(wx < 0.0f || wy < 0.0f || wx >= wMax || wy >= hMax, 0)) {
            out = true;
            if (!hess) {
              break;   /* the held Hessian is not this pass's: restart with the full sums */
            }
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
          /* (float)(u - v) of two bytes: the same value as (float)u - (float)v, one conversion */
          const float gx = 0.5f * (float)(t[1] - t[-1]);
          const float gy = 0.5f * (float)(t[w] - t[-w]);
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

          ofTerm_t *tm = &terms[m];
          tm->g[0] = G[0];
          tm->g[1] = G[1];
          tm->g[2] = G[2];
          tm->g[3] = G[3];
          tm->e = e;
          if (__builtin_expect(++m == OF_TERMS, 0)) {
            sumsFlush(&s, terms, m, hess);
            n += m;
            m = 0;
          }
        }
        if (out && !hess) {
          break;
        }
      }
      if (out && !hess) {
        OF_COUNT_RESTART();
        hess = true;
        continue;
      }
      sumsFlush(&s, terms, m, hess);
      n += m;
      reuse = !out;
      break;
    }

    if (n < 100) {
      return false;
    }
    float A[NP * NP], b[NP];
    if (hess) {
      held[0] = s.a[0];   /* A00 */
      held[1] = s.a[1];   /* A10 */
      held[2] = s.a[2];   /* A11 */
      held[3] = s.a[3];   /* A20 */
      held[4] = s.a[4];   /* A21 */
      held[5] = s.a[5];   /* A22 */
      held[6] = s.a[6];   /* A30 */
      held[7] = s.a[7];   /* A31 */
      held[8] = s.a[8];   /* A32 */
      held[9] = s.a[9];   /* A33 */
      held[10] = s.a4[0];   /* A40: sum of G[0] (G[4] = 1) */
      held[11] = s.a4[1];
      held[12] = s.a4[2];
      held[13] = s.a4[3];
      held[14] = (float)n;   /* A44: 1 x 1 summed n times, exact */
    }
    {
      int q = 0;
      for (int i = 0; i < NP; i++) {
        for (int j = 0; j <= i; j++) {
          A[i * NP + j] = held[q++];
        }
      }
    }
    for (int i = 0; i < NP; i++) {
      b[i] = s.b[i];
    }
    const float sse = s.sse;
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
  st->prevPinned = NULL;
  st->pyramidFlip = false;
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

static bool process(ofState_t *st, const uint8_t *frame, const float init[2], ofResult_t *out, bool pin)
{
  uint8_t *coarsePrev = st->pyramidFlip ? st->cur1 : st->prev1;
  uint8_t *coarseCur = st->pyramidFlip ? st->prev1 : st->cur1;
  const uint8_t *finePrev = pin ? st->prevPinned : st->prev0;
  const uint32_t c0 = ofCycles();
  memset(out, 0, sizeof(*out));
  memset(st->stageCycles, 0, sizeof(st->stageCycles));
  downsample(frame, coarseCur);

  if (!st->havePrev) {
    if (pin) st->prevPinned = frame;
    else memcpy(st->prev0, frame, sizeof(st->prev0));
    st->pyramidFlip = !st->pyramidFlip;
    st->havePrev = true;
    out->gain = 1.0f;
    return false;
  }
  const uint32_t c1 = ofCycles();
  uint32_t c2 = c1;   /* the coarse level's end */

  float p[NP] = {init ? 0.5f * init[0] : 0.0f, init ? 0.5f * init[1] : 0.0f, 0.0f, 0.0f, 0.0f};
  float texture = 0.0f, residual = 0.0f;
  int count = 0;
  ofStats_t stats = {0};
  float gain = 1.0f;
  bool ok, step = false;
  if (st->mode == OF_FIT_PLAIN) {
    ok = refineLevel(coarsePrev, coarseCur, W1, H1, 1, 3, p, &texture, &residual, &count, OFK_PLAIN, 1.0f, 1, &stats);
    c2 = ofCycles();
    if (ok) {
      p[0] *= 2.0f;
      p[1] *= 2.0f;
      ok = refineLevel(finePrev, frame, W0, H0, 2, 2, p, &texture, &residual, &count, OFK_PLAIN, 1.0f, 1, &stats);
    }
  } else {
    /* Coarse: the plain fit's iterations, the gain's statistics in iteration OF_GAIN_ITER (at a warp near the
     * motion), then the gain: a frame without an exposure step (|a - 1| under the dead band) is fitted plain to the
     * end, bit for bit; a step frame fits the gain from the next iteration on and leaves the saturated pixels out
     * (OF_FIT_FINE_MASK: on the fine level only). */
    ok = OF_GAIN_ITER == 0 ||
         refineLevel(coarsePrev, coarseCur, W1, H1, 1, OF_GAIN_ITER, p, &texture, &residual, &count, OFK_PLAIN, 1.0f, 1,
                     &stats);
    ok = ok && refineLevel(coarsePrev, coarseCur, W1, H1, 1, 1, p, &texture, &residual, &count, OFK_STATS, 1.0f,
                           st->mode == OF_FIT_FINE_MASK ? 2 : 1, &stats);
    gain = ofGainFromStats(stats.sI, stats.sJ, stats.nF);
    step = gain != 1.0f;
    const ofKind_t coarse = !step ? OFK_PLAIN : (st->mode == OF_FIT_ROBUST ? OFK_GAIN_MASK : OFK_GAIN);
    if (ok && 2 - OF_GAIN_ITER > 0) {
      ok = refineLevel(coarsePrev, coarseCur, W1, H1, 1, 2 - OF_GAIN_ITER, p, &texture, &residual, &count, coarse,
                       gain, 1, &stats);
    }
    c2 = ofCycles();
    if (ok) {
      p[0] *= 2.0f;
      p[1] *= 2.0f;
      /* fine: masked on a step frame (satFrac then the fine level's), plain on the others (satFrac the coarse
       * level's, counted in the statistics' iteration) */
      ok = refineLevel(finePrev, frame, W0, H0, 2, 2, p, &texture, &residual, &count,
                       step ? OFK_GAIN_MASK : OFK_PLAIN, gain, 1, &stats);
    }
  }

  const uint32_t c3 = ofCycles();
  if (pin) st->prevPinned = frame;
    else memcpy(st->prev0, frame, sizeof(st->prev0));
  st->pyramidFlip = !st->pyramidFlip;

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
  st->stageCycles[OF_STAGE_DOWNSAMPLE] = c1 - c0;
  st->stageCycles[OF_STAGE_COARSE] = c2 - c1;
  st->stageCycles[OF_STAGE_FINE] = c3 - c2;
  st->stageCycles[OF_STAGE_COPY] = ofCycles() - c3;
  return ok;
}

bool ofProcess(ofState_t *st, const uint8_t *frame, const float init[2], ofResult_t *out)
{
  return process(st, frame, init, out, false);
}
bool ofProcessPinned(ofState_t *st, const uint8_t *frame, const float init[2], ofResult_t *out)
{
  return process(st, frame, init, out, true);
}
