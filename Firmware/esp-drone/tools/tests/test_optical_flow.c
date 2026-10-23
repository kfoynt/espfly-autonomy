/*
 * Host unit tests for components/core/crazyflie/utils/src/optical_flow.c
 * (and its luma of a YUYV colour frame, ofLumaFromYuyv)
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_optical_flow.c components/core/crazyflie/utils/src/optical_flow.c \
 *      -lm -o /tmp/test_optical_flow && /tmp/test_optical_flow
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "optical_flow.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

/* Smooth, textured test scene (continuous, so it can be resampled exactly). */
static float scene(float x, float y)
{
  return 128.0f
       + 40.0f * sinf(0.21f * x + 0.5f) * cosf(0.17f * y)
       + 25.0f * sinf(0.09f * x - 0.13f * y + 1.0f)
       + 20.0f * cosf(0.31f * y + 0.07f * x)
       + 15.0f * sinf(0.43f * x + 0.29f * y + 2.0f);
}

/* Render the frame in which the scene point at prev-pixel x appears at
 * x' = c + M (x - c) + t, with M = [[1+s, -r], [r, 1+s]] (the model in
 * optical_flow.h), plus a brightness change and optional noise. */
static void render(uint8_t *img, float tx, float ty, float s, float r, float bright, int noise)
{
  const float cx = 0.5f * (OF_WIDTH - 1), cy = 0.5f * (OF_HEIGHT - 1);
  const float det = (1 + s) * (1 + s) + r * r;
  for (int y = 0; y < OF_HEIGHT; y++) {
    for (int x = 0; x < OF_WIDTH; x++) {
      float u = x - cx - tx, v = y - cy - ty;
      float px = ((1 + s) * u + r * v) / det + cx;
      float py = (-r * u + (1 + s) * v) / det + cy;
      float val = scene(px, py) + bright;
      if (noise) {
        val += (float)(rand() % (2 * noise + 1) - noise);
      }
      if (val < 0) val = 0;
      if (val > 255) val = 255;
      img[y * OF_WIDTH + x] = (uint8_t)lrintf(val);
    }
  }
}

static ofResult_t estimate(float tx, float ty, float s, float r, float bright, int noise)
{
  static ofState_t st;
  static uint8_t a[OF_WIDTH * OF_HEIGHT], b[OF_WIDTH * OF_HEIGHT];
  ofResult_t res;
  ofInit(&st);
  render(a, 0, 0, 0, 0, 0, noise);
  render(b, tx, ty, s, r, bright, noise);
  ofProcess(&st, a, NULL, &res);
  ofProcess(&st, b, NULL, &res);
  return res;
}

static void testSolve(void)
{
  float A[9] = {4, 1, 0, 1, 3, 1, 0, 1, 2};
  float b[3] = {1, 2, 3};
  /* x solves A x = b: verify by substitution. */
  float A0[9];
  memcpy(A0, A, sizeof(A));
  float b0[3] = {1, 2, 3};
  CHECK(ofSolve(A, b, 3), "solve succeeds");
  for (int i = 0; i < 3; i++) {
    float s = 0;
    for (int j = 0; j < 3; j++) s += A0[i * 3 + j] * b[j];
    CHECK(fabsf(s - b0[i]) < 1e-5f, "row %d residual %g", i, s - b0[i]);
  }
  float S[4] = {1, 2, 2, 4};
  float sb[2] = {1, 1};
  CHECK(!ofSolve(S, sb, 2), "singular system rejected");
}

static void testFirstFrame(void)
{
  static ofState_t st;
  static uint8_t a[OF_WIDTH * OF_HEIGHT];
  ofResult_t res;
  ofInit(&st);
  render(a, 0, 0, 0, 0, 0, 0);
  CHECK(!ofProcess(&st, a, NULL, &res) && !res.ok, "first frame has no estimate");
  CHECK(ofProcess(&st, a, NULL, &res) && res.ok, "identical second frame ok");
  CHECK(fabsf(res.tx) < 0.01f && fabsf(res.ty) < 0.01f, "no motion for identical frames (%g, %g)", res.tx, res.ty);
}

static void testTranslation(void)
{
  const float cases[][2] = {{1.3f, -0.7f}, {-0.25f, 0.4f}, {0.05f, -0.03f}, {3.5f, 2.0f}, {-2.8f, -1.6f}};
  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    ofResult_t r = estimate(cases[i][0], cases[i][1], 0, 0, 0, 0);
    float tol = (fabsf(cases[i][0]) > 2.0f) ? 0.1f : 0.03f;
    CHECK(r.ok, "translation case %u ok", i);
    CHECK(fabsf(r.tx - cases[i][0]) < tol && fabsf(r.ty - cases[i][1]) < tol,
          "translation (%g, %g) estimated (%g, %g)", cases[i][0], cases[i][1], r.tx, r.ty);
    CHECK(fabsf(r.scale) < 2e-3f && fabsf(r.rot) < 2e-3f, "no spurious scale/rotation (%g, %g)", r.scale, r.rot);
  }
}

static void testRotationScale(void)
{
  ofResult_t r = estimate(0.6f, -0.4f, 0.0f, 0.012f, 0, 0);
  CHECK(r.ok && fabsf(r.rot - 0.012f) < 1e-3f, "rotation 0.012 estimated %g", r.rot);
  CHECK(fabsf(r.tx - 0.6f) < 0.05f && fabsf(r.ty + 0.4f) < 0.05f, "translation with rotation (%g, %g)", r.tx, r.ty);

  r = estimate(0.0f, 0.0f, 0.015f, 0.0f, 0, 0);
  CHECK(r.ok && fabsf(r.scale - 0.015f) < 1e-3f, "scale 0.015 estimated %g", r.scale);
  CHECK(fabsf(r.tx) < 0.05f && fabsf(r.ty) < 0.05f, "no translation from pure scale (%g, %g)", r.tx, r.ty);

  r = estimate(-0.8f, 0.5f, -0.01f, -0.008f, 0, 0);
  CHECK(r.ok && fabsf(r.scale + 0.01f) < 1e-3f && fabsf(r.rot + 0.008f) < 1e-3f,
        "combined scale/rotation estimated (%g, %g)", r.scale, r.rot);
}

static void testBrightnessAndNoise(void)
{
  ofResult_t r = estimate(0.9f, 0.3f, 0, 0, 12.0f, 0);
  CHECK(r.ok && fabsf(r.bias + 12.0f) < 1.0f, "brightness change estimated as bias %g", r.bias);
  CHECK(fabsf(r.tx - 0.9f) < 0.05f && fabsf(r.ty - 0.3f) < 0.05f, "translation robust to brightness (%g, %g)", r.tx, r.ty);

  srand(1);
  r = estimate(-1.1f, 0.6f, 0, 0, 0, 4);
  CHECK(r.ok && fabsf(r.tx + 1.1f) < 0.1f && fabsf(r.ty - 0.6f) < 0.1f, "translation with +/-4 noise (%g, %g)", r.tx, r.ty);
  CHECK(r.residual > 1.0f, "noise shows up in the residual (%g)", r.residual);
}

static void testFlatImage(void)
{
  static ofState_t st;
  static uint8_t flat[OF_WIDTH * OF_HEIGHT];
  ofResult_t res;
  memset(flat, 100, sizeof(flat));
  ofInit(&st);
  ofProcess(&st, flat, NULL, &res);
  ofProcess(&st, flat, NULL, &res);
  CHECK(!res.ok || res.texture < 1e-3f, "flat image reports no texture (ok=%d texture=%g)", res.ok, res.texture);

  ofResult_t r = estimate(0.5f, 0.5f, 0, 0, 0, 0);
  CHECK(r.texture > 10.0f, "textured scene reports texture (%g)", r.texture);
}

/* Colour photos (docs/autonomy-a2b.md 24): the fit of a YUYV frame's luma is the fit of the grey frame with the same
 * Y, bit for bit, whatever the chroma; ofLumaFromYuyv takes every even byte in order. */
static void testLumaFromYuyv(void)
{
  static uint8_t a[OF_WIDTH * OF_HEIGHT], b[OF_WIDTH * OF_HEIGHT], y[OF_WIDTH * OF_HEIGHT];
  static uint32_t ya[OF_WIDTH * OF_HEIGHT / 2], yb[OF_WIDTH * OF_HEIGHT / 2];   /* word-aligned YUYV frames */
  uint8_t *const fa = (uint8_t *)ya, *const fb = (uint8_t *)yb;
  srand(7);
  render(a, 0, 0, 0, 0, 0, 2);
  render(b, 0.8f, -0.4f, 0.01f, 0.004f, 3.0f, 2);
  for (int i = 0; i < OF_WIDTH * OF_HEIGHT; i++) {
    fa[2 * i] = a[i];
    fa[2 * i + 1] = (uint8_t)(rand() & 0xFF);   /* U, V: anything */
    fb[2 * i] = b[i];
    fb[2 * i + 1] = (uint8_t)(rand() & 0xFF);
  }
  ofLumaFromYuyv(fa, y);
  CHECK(memcmp(y, a, sizeof(a)) == 0, "the luma of a YUYV frame is its even bytes");
  const uint8_t pair[8] = {11, 22, 33, 44, 55, 66, 77, 88};
  memcpy(fa, pair, sizeof(pair));
  ofLumaFromYuyv(fa, y);
  CHECK(y[0] == 11 && y[1] == 33 && y[2] == 55 && y[3] == 77 && y[4] == a[4], "Y0 U Y1 V: %u %u %u %u", y[0], y[1],
        y[2], y[3]);
  fa[0] = a[0]; fa[2] = a[1]; fa[4] = a[2]; fa[6] = a[3];   /* the scene's first 4 pixels back */

  static ofState_t s1, s2;
  ofResult_t r1, r2;
  ofInit(&s1);
  ofInit(&s2);
  ofProcess(&s1, a, NULL, &r1);
  ofProcess(&s1, b, NULL, &r1);
  static uint8_t la[OF_WIDTH * OF_HEIGHT], lb[OF_WIDTH * OF_HEIGHT];
  ofLumaFromYuyv(fa, la);
  ofLumaFromYuyv(fb, lb);
  ofProcess(&s2, la, NULL, &r2);
  ofProcess(&s2, lb, NULL, &r2);
  CHECK(r1.ok && r2.ok && memcmp(&r1, &r2, sizeof(r1)) == 0, "the fit of the YUYV frames' luma is the grey frames' fit "
        "bit for bit (tx %g / %g)", r1.tx, r2.tx);
}

int main(void)
{
  testSolve();
  testFirstFrame();
  testTranslation();
  testRotationScale();
  testBrightnessAndNoise();
  testFlatImage();
  testLumaFromYuyv();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
