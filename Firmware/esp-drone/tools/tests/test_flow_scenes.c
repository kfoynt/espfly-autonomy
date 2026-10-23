/*
 * Synthetic camera scenes through the real flow fit and the camera drift
 * hold's HOVER scene gate (components/core/crazyflie/utils/src/
 * optical_flow.c ofProcess, camera_hold.c camHoldSceneOk; docs/autonomy-a2.md
 * section 12): what the gate refuses, and what it does not see.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_flow_scenes.c components/core/crazyflie/utils/src/optical_flow.c \
 *      components/core/crazyflie/utils/src/camera_hold.c -lm -o /tmp/test_flow_scenes && /tmp/test_flow_scenes
 *
 * Every sequence: 160x120 grey frames 67 ms apart (15 fps) of the textured
 * test scene (test_optical_flow.c) drifting 0.3 / -0.2 px a frame (a hovering
 * drone), no rotation (dxRes = tx). A frame is "gated" when the hold would not
 * use it: camera_flow.c's own view test (fit ok, texture >= 8, residual < 15)
 * or the armed scene gate (residual < 8, |dxRes| <= 1.5 px, smoothed |scale /
 * dt| <= 0.3 /s). Renderers from the 2026-10-13 synthesis (fix-ih/synth
 * syn_walk.c, syn_of.c, syn_win.c).
 *   (i)   an articulated, motion-blurred walker (dark silhouette: head, torso,
 *         swinging arms and legs) crossing or receding: refused on most frames
 *         (a person 1-2 m ahead, flight 1); standing still: never;
 *   (ii)  clean background translation and expansion: never refused, and the
 *         hold's outputs identical with and without the gate;
 *   (iii) KNOWN_GAP: smooth-textured rigid blobs (15-75 % of the width,
 *         textured or dark) moving sideways, receding, approaching or
 *         vertically drag the fit by up to ~1.3 px and 0.03 a frame at
 *         residuals the gate passes. Pinned (with a tolerance) so that a
 *         future robust fit is measured against today's numbers;
 *   (iv)  a saturated window (clipped background) barely biases the fit;
 *         exposure steps with a window in a third of the view stay below the
 *         gate's residual (a window filling half the view is borderline).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera_hold.h"
#include "optical_flow.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define DT 0.067f
#define W OF_WIDTH
#define H OF_HEIGHT

static ofState_t of;
static uint8_t img[W * H];

static float scene(float x, float y)
{
  return 128.0f + 40.0f * sinf(0.21f * x + 0.5f) * cosf(0.17f * y) + 25.0f * sinf(0.09f * x - 0.13f * y + 1.0f) +
         20.0f * cosf(0.31f * y + 0.07f * x) + 15.0f * sinf(0.43f * x + 0.29f * y + 2.0f);
}

static uint8_t clip8(float v)
{
  return (uint8_t)lrintf(v < 0.0f ? 0.0f : (v > 255.0f ? 255.0f : v));
}

/* One sequence's verdicts. */
typedef struct {
  int n, gated;
  double maxRes, sumRes;
  double maxErrT, maxErrS;     /* over the frames the gate let through: |fit - background motion| */
  double sumTx, sumTy, sumS;   /* fit minus background motion, all frames */
} seqRes_t;

typedef struct {
  camHoldScene_t g;
  seqRes_t r;
  float bgTx, bgTy, bgS;       /* the background's true motion per frame */
  bool started;                /* the first frame (no fit) is in */
} seq_t;

static void seqStart(seq_t *q, float tx, float ty, float s)
{
  ofInit(&of);
  memset(q, 0, sizeof(*q));
  camHoldSceneReset(&q->g);
  q->g.inHover = true;                     /* armed: autonomy HOVER for 2 s already */
  q->g.hoverS = CAM_HOLD_SCENE_ARM_S;
  q->bgTx = tx;
  q->bgTy = ty;
  q->bgS = s;
}

/* The frame in img through the fit and the hold's view tests; false for the first frame. */
static bool seqFrame(seq_t *q, ofResult_t *out)
{
  ofResult_t r;
  const bool ok = ofProcess(&of, img, NULL, &r);
  if (!q->started) {
    q->started = true;   /* the first frame: no fit yet */
    return false;
  }
  const bool view = ok && r.texture >= 8.0f && r.residual < 15.0f;
  const bool scene = camHoldSceneOk(&q->g, true, DT, r.residual, r.tx, r.scale);
  const bool gated = !(view && scene);
  q->r.n++;
  q->r.gated += gated;
  q->r.maxRes = fmax(q->r.maxRes, r.residual);
  q->r.sumRes += r.residual;
  const double eT = hypot(r.tx - q->bgTx, r.ty - q->bgTy), eS = fabs(r.scale - q->bgS);
  if (!gated) {
    q->r.maxErrT = fmax(q->r.maxErrT, eT);
    q->r.maxErrS = fmax(q->r.maxErrS, eS);
  }
  q->r.sumTx += r.tx - q->bgTx;
  q->r.sumTy += r.ty - q->bgTy;
  q->r.sumS += r.scale - q->bgS;
  if (out) {
    *out = r;
  }
  return true;
}

/* ---- (i) the articulated walker (syn_walk.c) --------------------------------- */

/* Inside the walker at gait phase ph, centre cx, scale sc (1 = 120 px tall). */
static int walker(float x, float y, float cx, float sc, float ph)
{
  const float hgt = 120.0f * sc, top = 60.0f - 0.5f * hgt;
  const float u = (x - cx) / sc, v = (y - top) / sc;
  if (v < 0 || v > 120) {
    return 0;
  }
  if (v < 16) {
    return hypotf(u, v - 8) < 8;                         /* head */
  }
  if (v < 66) {
    if (fabsf(u) < 14) {
      return 1;                                          /* torso */
    }
    const float sw = 10.0f * sinf(ph);                   /* arms swing */
    return fabsf(u - 18 - sw * (v - 16) / 50) < 4 || fabsf(u + 18 + sw * (v - 16) / 50) < 4;
  }
  const float t = (v - 66) / 54.0f, sw = 14.0f * sinf(ph) * t;   /* legs in antiphase */
  return fabsf(u - 6 - sw) < 6 || fabsf(u + 6 + sw) < 6;
}

/* Background shifted by (bgx, bgy); the walker blurred over 1/3 of the frame interval. */
static void renderWalker(float bgx, float bgy, float cx, float sc, float ph, float vx, float dph)
{
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      float acc = 0.0f;
      for (int s = 0; s < 4; s++) {
        const float f = s / 12.0f;
        float v = scene(x - bgx, y - bgy);
        if (sc > 0 && walker(x, y, cx + f * vx, sc, ph + f * dph)) {
          v = 50.0f + 6.0f * sinf(0.4f * x + 0.3f * y);
        }
        acc += v;
      }
      img[y * W + x] = clip8(acc / 4.0f);
    }
  }
}

static seqRes_t walkerSeq(float sc, float vx, float dph, float ds)
{
  seq_t q;
  seqStart(&q, 0.3f, -0.2f, 0.0f);
  float cx = vx > 0 ? 30.0f : 80.0f, ph = 0.0f;
  for (int k = 0; k <= 14; k++) {
    renderWalker(0.3f * k, -0.2f * k, cx, sc, ph, vx, dph);
    seqFrame(&q, NULL);
    cx += vx;
    ph += dph;
    sc *= 1.0f + ds;
  }
  return q.r;
}

static void testWalker(void)
{
  static const struct { const char *name; float sc, vx, dph, ds; int minGated, maxGated; } cs[] = {
    { "walker 1.0 (~2 m) crossing 2 px/frame", 1.0f, 2.0f, 0.6f, 0.0f, 11, 14 },
    { "walker 1.5 (~1.3 m) crossing 3 px/frame", 1.5f, 3.0f, 0.6f, 0.0f, 13, 14 },
    { "walker 2.0 (~1 m) crossing 4 px/frame", 2.0f, 4.0f, 0.6f, 0.0f, 13, 14 },
    { "walker 1.5 receding 2 %/frame", 1.5f, 0.0f, 0.6f, -0.02f, 12, 14 },
    { "walker 1.5 standing still", 1.5f, 0.0f, 0.0f, 0.0f, 0, 0 },
  };
  for (unsigned i = 0; i < sizeof(cs) / sizeof(cs[0]); i++) {
    const seqRes_t r = walkerSeq(cs[i].sc, cs[i].vx, cs[i].dph, cs[i].ds);
    printf("  (i) %-42s gated %2d of %d | residual mean %4.1f max %4.1f\n", cs[i].name, r.gated, r.n,
           r.sumRes / r.n, r.maxRes);
    CHECK(r.n == 14 && r.gated >= cs[i].minGated && r.gated <= cs[i].maxGated, "%s: gated %d of %d (want %d..%d)",
          cs[i].name, r.gated, r.n, cs[i].minGated, cs[i].maxGated);
  }
}

/* ---- (ii) clean background --------------------------------------------------- */

/* The test scene moved by (tx, ty) per frame and scaled by (1 + s) per frame about the centre. */
static void renderBg(int k, float tx, float ty, float s, float win, float wfrac, float gain)
{
  const float cx = 79.5f, cy = 59.5f, bs = powf(1.0f + s, (float)k);
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      const float bx = (x - cx - k * tx) / bs + cx, by = (y - cy - k * ty) / bs + cy;
      float v = scene(bx, by);
      if (win > 0.0f && bx >= W * (1.0f - wfrac)) {
        v += win;
      }
      img[y * W + x] = clip8(v * gain);
    }
  }
}

static void testClean(void)
{
  static const struct { const char *name; float tx, ty, s; } cs[] = {
    { "hover drift 0.3/-0.2 px/frame", 0.3f, -0.2f, 0.0f },
    { "drift 1.0/0.6 px/frame", 1.0f, 0.6f, 0.0f },
    { "expansion 0.5 %/frame (0.075 /s)", 0.0f, 0.0f, 0.005f },
    { "contraction 0.5 %/frame + drift 0.3/-0.2", 0.3f, -0.2f, -0.005f },
    { "expansion 1 %/frame (0.15 /s) + drift 0.5/0.3", 0.5f, 0.3f, 0.01f },
  };
  for (unsigned i = 0; i < sizeof(cs) / sizeof(cs[0]); i++) {
    /* the hold on the same frames, gated and ungated */
    camHoldState_t hu, hg;
    camHoldReset(&hu);
    camHoldReset(&hg);
    seq_t q;
    seqStart(&q, cs[i].tx, cs[i].ty, cs[i].s);
    bool same = true;
    for (int k = 0; k <= 30; k++) {
      renderBg(k, cs[i].tx, cs[i].ty, cs[i].s, 0.0f, 0.0f, 1.0f);
      ofResult_t r;
      const int gatedBefore = q.r.gated;
      if (!seqFrame(&q, &r)) {
        continue;
      }
      const bool view = r.ok && r.texture >= 8.0f && r.residual < 15.0f;
      const bool sceneOk = q.r.gated == gatedBefore;
      camHoldInput_t in = { .enabled = true, .sticksCentred = true, .airborne = true, .viewOk = view,
                            .wFwd = r.scale / DT, .wLat = -r.tx / 102.4f / DT, .dt = DT };
      camHoldStep(&hu, &in);
      in.viewOk = view && sceneOk;
      camHoldStep(&hg, &in);
      same = same && memcmp(&hu, &hg, sizeof(hu)) == 0;
    }
    printf("  (ii) %-46s gated %d of %d | residual max %.1f | fit error max %.3f px, %.5f\n", cs[i].name, q.r.gated,
           q.r.n, q.r.maxRes, q.r.maxErrT, q.r.maxErrS);
    CHECK(q.r.n == 30 && q.r.gated == 0 && same, "%s: gated %d of %d, hold outputs identical with the gate %d",
          cs[i].name, q.r.gated, q.r.n, same);
  }
}

/* ---- (iii) KNOWN_GAP: rigid blobs (syn_of.c) --------------------------------- */

static int ptype;   /* 0 textured, 1 dark silhouette with mild texture */

static float blob(float x, float y)
{
  if (ptype == 0) {
    return 90.0f + 35.0f * sinf(0.33f * x + 1.3f) * sinf(0.27f * y) + 20.0f * cosf(0.19f * x + 0.41f * y);
  }
  return 45.0f + 8.0f * sinf(0.5f * x) * cosf(0.4f * y);
}

typedef struct { float w, h, px0, pvx, pvy, ps; } blobMove_t;

static void renderBlob(int k, const blobMove_t *m)
{
  const float cx = 79.5f, cy = 59.5f;
  const float psk = powf(1.0f + m->ps, (float)k);
  const float pcx = m->px0 + k * m->pvx, pcy = cy + k * m->pvy;
  const float hw = 0.5f * m->w * W * psk, hh = 0.5f * m->h * H * psk;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      const float bx = x - k * 0.3f, by = y + k * 0.2f;
      float v = scene(bx, by);
      if (fabsf(x - pcx) < hw && fabsf(y - pcy) < hh) {
        v = blob((x - pcx) / psk, (y - pcy) / psk);
      }
      img[y * W + x] = clip8(v);
    }
  }
  (void)cx;
}

/* Today's values (2026-10-13, optical_flow.c unchanged since 8f51285): per texture and motion over the widths
 * 15/30/50/75 %, the frames gated (of 4 x 12, vertical 4 x 10), the largest residual, and over the frames the
 * gate passes the largest fit error against the background (px, scale per frame). */
typedef struct { const char *name; int gated; double maxRes, maxErrT, maxErrS; } gapPin_t;
static const gapPin_t gapPins[2][4] = {
  { { "textured, sideways 2 px/frame", 12, 7.7, 1.20, 0.0103 },
    { "textured, receding 2 %/frame", 0, 6.9, 0.32, 0.0215 },
    { "textured, approaching 2 %/frame", 0, 6.1, 0.39, 0.0229 },
    { "textured, vertical 1.5 px/frame", 0, 7.7, 1.26, 0.0055 } },
  { { "dark, sideways 2 px/frame", 25, 8.5, 1.22, 0.0108 },
    { "dark, receding 2 %/frame", 8, 9.5, 0.30, 0.0277 },
    { "dark, approaching 2 %/frame", 16, 9.4, 0.31, 0.0282 },   /* w 75 %: texture < 8 (the hold's own test) */
    { "dark, vertical 1.5 px/frame", 16, 9.1, 1.29, 0.0064 } },
};
#define PIN_TOL_RES   0.3
#define PIN_TOL_T     0.10
#define PIN_TOL_S     0.003
#define PIN_TOL_GATED 3

static void testKnownGap(void)
{
  static const float widths[] = { 0.15f, 0.30f, 0.50f, 0.75f };
  for (ptype = 0; ptype < 2; ptype++) {
    for (int mv = 0; mv < 4; mv++) {
      int gated = 0, n = 0;
      double maxRes = 0.0, maxT = 0.0, maxS = 0.0;
      for (int wi = 0; wi < 4; wi++) {
        blobMove_t m = { .w = widths[wi], .h = 1.0f, .px0 = 79.5f };
        int frames = 12;
        switch (mv) {
          case 0: m.px0 = 40.0f; m.pvx = 2.0f; break;
          case 1: m.ps = -0.02f; break;
          case 2: m.ps = 0.02f; break;
          default: m.h = 0.6f; m.pvy = 1.5f; frames = 10; break;
        }
        seq_t q;
        seqStart(&q, 0.3f, -0.2f, 0.0f);
        for (int k = 0; k <= frames; k++) {
          renderBlob(k, &m);
          seqFrame(&q, NULL);
        }
        gated += q.r.gated;
        n += q.r.n;
        maxRes = fmax(maxRes, q.r.maxRes);
        maxT = fmax(maxT, q.r.maxErrT);
        maxS = fmax(maxS, q.r.maxErrS);
      }
      const gapPin_t *p = &gapPins[ptype][mv];
      printf("  (iii) KNOWN_GAP %-34s gated %2d of %d | residual max %4.1f | passed frames: fit error max %.2f px, "
             "%.4f (pinned %d, %.1f, %.2f, %.4f)\n", p->name, gated, n, maxRes, maxT, maxS, p->gated, p->maxRes,
             p->maxErrT, p->maxErrS);
      CHECK(abs(gated - p->gated) <= PIN_TOL_GATED && fabs(maxRes - p->maxRes) <= PIN_TOL_RES &&
            fabs(maxT - p->maxErrT) <= PIN_TOL_T && fabs(maxS - p->maxErrS) <= PIN_TOL_S,
            "KNOWN_GAP %s moved from its pin: gated %d (%d), residual %.2f (%.1f), errors %.3f (%.2f) px, %.5f "
            "(%.4f): a change of the fit (update the pin, docs/autonomy-a2.md 12)", p->name, gated, p->gated,
            maxRes, p->maxRes, maxT, p->maxErrT, maxS, p->maxErrS);
    }
  }
}

/* ---- (iv) bright windows -------------------------------------------------------- */

static void testWindow(void)
{
  /* saturated window in the right third (syn_win.c): the clipped background barely biases the fit */
  static const float win[] = { 60.0f, 120.0f, 200.0f };
  for (unsigned i = 0; i < sizeof(win) / sizeof(win[0]); i++) {
    seq_t q;
    seqStart(&q, 0.3f, -0.2f, 0.0f);
    int sat = 0;
    for (int k = 0; k <= 12; k++) {
      for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
          const float bx = x - k * 0.3f, by = y + k * 0.2f;
          float v = scene(bx, by);
          if (bx >= 107.0f) {
            v += win[i];
          }
          img[y * W + x] = clip8(v);
        }
      }
      seqFrame(&q, NULL);
    }
    for (int p = 0; p < W * H; p++) {
      sat += img[p] >= 250;
    }
    const double mx = q.r.sumTx / q.r.n, my = q.r.sumTy / q.r.n, ms = q.r.sumS / q.r.n;
    printf("  (iv) window +%3.0f grey (%4.1f %% of the pixels >= 250): mean fit bias tx %+.3f ty %+.3f px scale %+.5f, "
           "gated %d of %d\n", win[i], 100.0 * sat / (W * H), mx, my, ms, q.r.gated, q.r.n);
    CHECK(fabs(mx) <= 0.03 && fabs(my) <= 0.01 && fabs(ms) <= 0.0005 && q.r.gated == 0,
          "window +%.0f: bias tx %.3f ty %.3f scale %.5f, gated %d", win[i], mx, my, ms, q.r.gated);
  }
  /* exposure steps (auto exposure hunting on a bright window): alternate frames darker */
  static const struct { const char *name; float wfrac, g1; bool required; } ex[] = {
    { "third-view window, exposure x1/x0.95", 0.33f, 0.95f, true },
    { "third-view window, exposure x1/x0.90", 0.33f, 0.90f, true },
    { "half-view window, exposure x1/x1.10 (informative)", 0.50f, 1.10f, false },
  };
  for (unsigned i = 0; i < sizeof(ex) / sizeof(ex[0]); i++) {
    seq_t q;
    seqStart(&q, 0.3f, -0.2f, 0.0f);
    for (int k = 0; k <= 12; k++) {
      renderBg(k, 0.3f, -0.2f, 0.0f, 120.0f, ex[i].wfrac, (k % 2) ? ex[i].g1 : 1.0f);
      seqFrame(&q, NULL);
    }
    printf("  (iv) %-50s residual max %.1f, gated %d of %d\n", ex[i].name, q.r.maxRes, q.r.gated, q.r.n);
    if (ex[i].required) {
      CHECK(q.r.maxRes < CAM_HOLD_SCENE_MAX_RESIDUAL && q.r.gated == 0, "%s: residual max %.2f (< 8), gated %d",
            ex[i].name, q.r.maxRes, q.r.gated);
    }
  }
}

int main(void)
{
  testWalker();
  testClean();
  testKnownGap();
  testWindow();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
