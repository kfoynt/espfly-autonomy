/*
 * ESP-Drone Firmware
 *
 * optical_flow.h - Global image-motion estimate between consecutive frames
 *
 * Fits a similarity motion model plus a brightness offset between the
 * previous and current greyscale frame with a two-level Lucas-Kanade
 * (Gauss-Newton) estimator:
 *
 *   I_prev(x) = a * I_cur(x + d(x)) + c,   d(x) = (tx + s*x' - r*y',  ty + s*y' + r*x')
 *
 * where x' = x - cx, y' = y - cy are pixel coordinates relative to the image
 * centre (+x right, +y down). tx/ty is the scene shift in pixels, s the
 * relative scale change (+ = scene grows, i.e. approaching), r the rotation in
 * radians (+ = from +x toward +y, clockwise on screen) and c the brightness
 * change. Valid for small inter-frame motion (a few pixels).
 *
 * Bright windows and exposure steps (docs/autonomy-a2b.md section 19.4).
 * The auto exposure's frame-to-frame steps are multiplicative: a plain
 * brightness offset c cannot absorb them, and on a centre-bright frame the
 * residual looks like expansion (-0.010..-0.019 /s of fake scale at
 * -2 %/frame on the flight-4 window photo); the clipped window does not
 * scale at all. The gain a = mean(I_prev) / mean(I_cur) is measured over
 * the flat unsaturated pixels (|gx| + |gy| <= OF_GAIN_FLAT_GRAD, I_prev >= 8)
 * in the coarse level's iteration OF_GAIN_ITER (at a warp near the motion:
 * in iteration 0 the selection of flat pixels at the wrong warp biased it
 * by up to 0.5 % without any exposure change), clamped OF_GAIN_MIN..MAX, 1
 * with fewer than OF_GAIN_MIN_PX flat pixels or inside OF_GAIN_DEADBAND.
 * A frame with a = 1 is fitted plain to the end, bit for bit the fit of
 * 14bcfc6 (a saturated window with constant exposure moves rigidly; leaving
 * it out only lost accuracy). A step frame fits the gain from the next
 * iteration on and leaves the saturated pixels out: the template pixel or
 * a neighbour its gradient uses at or above OF_SAT_LEVEL, or the template
 * pixel where the brighter current frame clips (>= OF_SAT_LEVEL x a, a < 1).
 * The test reads the template only: one on the warped sample refused pixels
 * by the sign of their residual and shifted the fit (-0.08 px on a 30 %
 * window). Saturated pixels are counted on every frame (satFrac). The caller
 * refuses a step too large to correct (camera_flow.c, |a - 1| >
 * CAM_FLOW_GAIN_STEP_MAX). OF_FIT_PLAIN (the CPU fallback's last step, and
 * the tests' reference) is the fit of 14bcfc6 bit for bit, gain 1.
 *
 * No ESP-IDF dependencies; unit tested by tools/tests/test_optical_flow.c.
 */
#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

#include <stdbool.h>
#include <stdint.h>

#include "camera_profile.h"
#define OF_WIDTH   CAMERA_NATIVE_WIDTH
#define OF_HEIGHT  CAMERA_NATIVE_HEIGHT
/* Fit native QVGA with the original angular sampling density. No 160x120 image. */
#define OF_SAMPLE_STRIDE CAMERA_NATIVE_STRIDE

#define OF_SAT_LEVEL       250     /* grey: this or above is clipped (a window) */
#define OF_GAIN_FLAT_GRAD  3.0f    /* grey/px: |gx| + |gy| of a flat pixel (the gain's statistics) */
#define OF_GAIN_MIN_PX     300     /* flat pixels needed for a gain (else 1) */
#define OF_GAIN_MIN        0.80f
#define OF_GAIN_MAX        1.25f
#define OF_GAIN_DEADBAND   0.006f  /* |a - 1| below this: no step (the estimate's own error at a steady exposure:
                                    * <= 0.3 % on the window photos) */
#define OF_GAIN_ITER       1       /* the coarse iteration (0..2) with the gain's statistics */

typedef struct {
  float tx, ty;    /* pixels at OF_WIDTH x OF_HEIGHT */
  float scale;     /* relative */
  float rot;       /* radians */
  float bias;      /* grey levels */
  float texture;   /* weakest-direction gradient energy per in-bounds pixel, saturated ones included (grey^2/px^2) */
  float residual;  /* RMS residual after the fit over the pixels used (grey levels) */
  int pixels;      /* pixels used at the fine level */
  float gain;      /* a: mean(I_prev) / mean(I_cur) of the flat pixels, exactly 1 without an exposure step (< 1:
                    * brighter now) */
  float satFrac;   /* saturated / in-bounds pixels: on a step frame the fine level's left out, otherwise the coarse
                    * level's >= OF_SAT_LEVEL (counted in the gain's iteration) */
  int flatPixels;  /* the flat unsaturated pixels the gain was measured over (0 in OF_FIT_PLAIN) */
  bool ok;
} ofResult_t;

/* The fit's variant (the CPU fallback of docs/autonomy-a2b.md 19.4, in this order). */
typedef enum {
  OF_FIT_ROBUST = 0,     /* step frames: the mask on both levels; the gain from every flat coarse pixel */
  OF_FIT_FINE_MASK = 1,  /* step frames: the mask on the fine level only; the gain from every other coarse row */
  OF_FIT_PLAIN = 2,      /* the fit of 14bcfc6 bit for bit (no mask, a = 1, gain reported 1, satFrac 0) */
} ofFitMode_t;

/* The stages of one ofProcess() call, timed in CPU cycles (the ESP32-S3's CCOUNT of the calling core; 0 on the
 * host): the clear and the 2x2 downsample, the coarse level's fit, the fine level's fit (0 when the coarse one
 * failed), the previous-frame copies and the result. All 0 for a first frame (no fit). */
enum { OF_STAGE_DOWNSAMPLE = 0, OF_STAGE_COARSE, OF_STAGE_FINE, OF_STAGE_COPY, OF_STAGES };

typedef struct {
  uint8_t prev0[OF_WIDTH * OF_HEIGHT];
  uint8_t prev1[(OF_WIDTH / 2) * (OF_HEIGHT / 2)];
  uint8_t cur1[(OF_WIDTH / 2) * (OF_HEIGHT / 2)];
  const uint8_t *prevPinned;
  bool pyramidFlip;
  bool havePrev;
  uint8_t mode;    /* ofFitMode_t; ofInit() keeps it (OF_FIT_ROBUST after a zeroed allocation) */
  uint32_t stageCycles[OF_STAGES];   /* the last call's (written by ofProcess()) */
} ofState_t;

/* Clears the previous frame; keeps st->mode. */
void ofInit(ofState_t *st);

/* The gain from the flat pixels' sums (the clamp, OF_GAIN_MIN_PX, OF_GAIN_DEADBAND): exactly 1 for no step. */
float ofGainFromStats(float sI, float sJ, int nF);

/* A frame whose exposure step is too large to correct (camera_flow.c refuses it: the drift hold coasts, the camera
 * height observer refuses the frame): |gain - 1| > OF_GAIN_STEP_MAX, or without a measured gain (OF_FIT_PLAIN, or
 * fewer than OF_GAIN_MIN_PX flat pixels) the mean brightness lum (any scale) moved more than OF_GAIN_STEP_MAX of
 * prevLum (> 0; 0: no previous frame). A NaN gain is too large. */
#define OF_GAIN_STEP_MAX   0.08f
bool ofExposureStepTooLarge(const ofResult_t *r, float lum, float prevLum);

/* Estimate motion from the stored previous frame to 'frame' (OF_WIDTH x
 * OF_HEIGHT, 8-bit grey) and store 'frame' as the new previous frame.
 * 'init' (optional) seeds tx/ty, e.g. from the gyro. Returns false (and
 * out->ok = false) for the first frame or a degenerate fit. */
bool ofProcess(ofState_t *st, const uint8_t *frame, const float init[2], ofResult_t *out);
/* Same native QVGA estimator without copying the fine frame. Caller retains each
 * frame until the next call returns, including through resets. Do not mix APIs. */
bool ofProcessPinned(ofState_t *st, const uint8_t *frame, const float init[2], ofResult_t *out);

/* The luma of a YUV 4:2:2 frame (YUYV: Y0 U Y1 V per pixel pair, OF_WIDTH x OF_HEIGHT pixels, 2 x OF_WIDTH bytes a
 * row) into y (OF_WIDTH x OF_HEIGHT bytes, the 8-bit grey frame ofProcess() takes): every even byte, in order (the
 * colour photos, docs/autonomy-a2b.md 24; the OV3660's Y8 output is the same Y). One pass, 32-bit words: both
 * buffers 4-byte aligned. Since the colour split (docs/autonomy-a2b.md 28) the firmware no longer calls it (the
 * camera driver's own copy extracts the Y bytes; the linker drops it): kept as the host tests' reference
 * (tools/tests/test_cam_split.c, test_optical_flow.c). */
void ofLumaFromYuyv(const uint8_t *yuyv, uint8_t *y);

/* Solve the symmetric n x n system A x = b in place (n <= 5). False if singular. */
bool ofSolve(float *A, float *b, int n);

#endif /* OPTICAL_FLOW_H_ */
