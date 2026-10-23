/* Camera cadence conversions. Flow translation remains in legacy 160x120 units.
 * The VGA build is explicit; host legacy regressions keep their original profile. */
#ifndef CAMERA_CADENCE_H_
#define CAMERA_CADENCE_H_
#include <math.h>
#include <stdint.h>

#define CAM_CADENCE_REF_S (1.0f / 15.0f)
#define CAM_CADENCE_VGA_PERIOD_US 90045u /* native QVGA; historical macro name */
/* The fast MOVE's view-loss timer (auto_motion.h AUTO_MV_FAST_VIEW_DEB_MS), derived from the profile's frame period
 * (2026-10-22, the table flight: both forward MOVEs stopped VIEW_FAST with the camera healthy; the hold had refused
 * three and six frames in a row for their fit residual, 275 and 544 ms of coasting against the 231 ms timer).
 * The hold refuses frames one at a time, so a lost view ends at the publication of the next accepted frame:
 * CAM_CADENCE_FAST_VIEW_FRAMES refused frames in a row recover at FRAMES x period after the loss, plus the
 * publication jitter. The timer is that time: FRAMES x the period rounded up to whole ms + the jitter, never below
 * VGA31's 231 ms (one 180 ms VGA frame + 50 ms; at the 90 ms QVGA period that let two refused frames recover).
 * QVGA: 3 x 91 + 50 = 323 ms (three refused frames recover, a fourth stops the move with its brake). The brief's
 * "period + navigation lag (124 ms) + margin" (265 ms) and "2.5 periods + margin" (277 ms) are both below it.
 * The near-scene FRONT brake keeps its own 150 ms timer. NVS mvview (auto_glue_core.h) lengthens it to 1000 ms. */
#define CAM_CADENCE_VGA_PERIOD_MS ((CAM_CADENCE_VGA_PERIOD_US + 999u) / 1000u)   /* 91: rounded up */
#define CAM_CADENCE_FAST_VIEW_FRAMES 3u             /* refused frames in a row a fast MOVE's pause rides out */
#define CAM_CADENCE_FAST_VIEW_JITTER_MS 50u         /* publication jitter after the recovering frame */
#define CAM_CADENCE_VGA31_FAST_VIEW_MS 231u         /* VGA31's timer: the floor, the timer never gets shorter */
#define CAM_CADENCE_VGA_FAST_VIEW_RAW_MS \
  (CAM_CADENCE_FAST_VIEW_FRAMES * CAM_CADENCE_VGA_PERIOD_MS + CAM_CADENCE_FAST_VIEW_JITTER_MS)
#define CAM_CADENCE_VGA_FAST_VIEW_MS \
  (CAM_CADENCE_VGA_FAST_VIEW_RAW_MS > CAM_CADENCE_VGA31_FAST_VIEW_MS ? CAM_CADENCE_VGA_FAST_VIEW_RAW_MS \
                                                                     : CAM_CADENCE_VGA31_FAST_VIEW_MS)
/* Measured frame-centre -> fit publication <= 230 ms on the first flight,
 * plus one 180 ms frame and 40 ms margin. Missing the next frame still expires. */
#define CAM_CADENCE_VGA_MAX_AGE_MS 450u
#ifdef ESPFLY_NATIVE_VGA
#define CAM_CADENCE_SAMPLE_MAX_AGE_US (CAM_CADENCE_VGA_MAX_AGE_MS * 1000u)
#else
#define CAM_CADENCE_SAMPLE_MAX_AGE_US 300000u
#endif

static inline float camCadenceExpAlpha(float dt)
{
  /* Original 0.2 at 15 Hz has a 0.29876 s time constant. */
  return dt > 0.0f && isfinite(dt) ? 1.0f - expf(-dt / 0.29876134f) : 0.0f;
}

static inline float camCadenceDxLimit(float pixels, float dt)
{
  return pixels * dt / CAM_CADENCE_REF_S;
}

/* Optical flow measures average motion between the two capture centres.
 * Its reference belongs at the interval midpoint, not at receipt time. */
static inline uint32_t camCadenceMotionLagUs(uint64_t now, uint64_t captureCentre, uint32_t period)
{
  if (now < captureCentre || now - captureCentre > CAM_CADENCE_VGA_MAX_AGE_MS * 1000u ||
      period == 0u || period > 300000u) {
    return 0u;
  }
  return (uint32_t)(now - captureCentre) + period / 2u;
}
#endif
