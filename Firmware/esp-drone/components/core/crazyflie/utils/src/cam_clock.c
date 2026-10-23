/*
 * ESP-Drone Firmware
 *
 * cam_clock.c - The flow camera's input clock against the flow loop's rate
 * (cam_clock.h; pure, host-tested by tools/tests/test_cam_clock.c). Flash
 * code: the camera task calls it once per frame.
 */
#include <string.h>

#include "cam_clock.h"

uint32_t camClockXclkHz(uint8_t code)
{
  return code == 0 ? CAM_CLOCK_FAST_HZ : CAM_CLOCK_SLOW_HZ;
}

bool camClockXclkOk(uint32_t hz)
{
  return hz == CAM_CLOCK_FAST_HZ || hz == CAM_CLOCK_SLOW_HZ;
}

float camClockFps(uint32_t hz)
{
  return (float)hz / (float)(CAM_CLOCK_LINE_PCLKS * CAM_CLOCK_FRAME_LINES);
}

uint32_t camClockPeriodUs(uint32_t hz)
{
  if (hz == 0) {
    return 0;
  }
  /* Native QVGA raw profile: SYSCLK = XCLK; HTS 2300, VTS 783. */
  const uint64_t clocks = (uint64_t)CAM_CLOCK_LINE_PCLKS * CAM_CLOCK_FRAME_LINES * 1000000u;
  return (uint32_t)((clocks + hz / 2u) / (uint64_t)hz);
}

uint32_t camClockReadoutUs(uint32_t hz)
{
  return (uint32_t)(((uint64_t)camClockPeriodUs(hz) * CAM_CLOCK_READOUT_LINES + CAM_CLOCK_FRAME_LINES / 2u) /
                    CAM_CLOCK_FRAME_LINES);
}

uint32_t camClockGyroOffsetUs(uint32_t hz)
{
  /* Physical rolling-readout model for QVGA (90.045 ms frame at 20 MHz).
   * The VGA-only residual correction is deliberately not transplanted.
   * Validate this offset against motors-off hand-motion data before flight. */
  const int32_t extra = (int32_t)camClockReadoutUs(hz) - 44160;
  int32_t us = (int32_t)CAM_CLOCK_OFFSET_REF_US + extra / 2;
  us = (us + 500) / 1000 * 1000;   /* the gyro's 1 ms samples */
  return us < 0 ? 0u : (uint32_t)us;
}

bool camClockSkip(uint32_t dtUs, uint32_t hz)
{
  const uint32_t p = camClockPeriodUs(hz);
  return p > 0 && (uint64_t)dtUs * CAM_CLOCK_SKIP_DEN > (uint64_t)p * CAM_CLOCK_SKIP_NUM;
}

float camClockProcAvg(float avgUs, uint32_t busyUs)
{
  return avgUs <= 0.0f ? (float)busyUs : avgUs + CAM_CLOCK_PROC_ALPHA * ((float)busyUs - avgUs);
}

bool camClockFallback(const camClockIn_t *in)
{
  if (in->xclkHz != CAM_CLOCK_SLOW_HZ || in->fellBack || !in->awake || in->autonomyActive || !in->onGround ||
      in->takeoffWindow) {
    return false;
  }
  return (in->clockFrames >= CAM_CLOCK_DECIDE_FRAMES && in->procAvgUs > CAM_CLOCK_PROC_MAX_US) ||
         (in->windowValid && in->windowFps < CAM_CLOCK_FPS_MIN);
}

void camClockTrackInit(camClockTrack_t *t, uint32_t xclkHz)
{
  const bool fellBack = t->fellBack;
  memset(t, 0, sizeof(*t));
  t->xclkHz = xclkHz;
  t->fellBack = fellBack;   /* a camera restart does not undo the boot's fallback */
}

void camClockTrackRestart(camClockTrack_t *t)
{
  t->winFrames = 0;
  t->dtAvgUs = 0.0f;
}

void camClockTrackWake(camClockTrack_t *t)
{
  t->winFrames = 0;
}

void camClockTrackFrame(camClockTrack_t *t, uint64_t frameUs, uint32_t dtUs, uint32_t busyUs)
{
  t->procAvgUs = camClockProcAvg(t->procAvgUs, busyUs);
  if (busyUs > t->procMaxUs) {
    t->procMaxUs = busyUs;
  }
  if (t->clockFrames < UINT32_MAX) {
    t->clockFrames++;
  }
  if (dtUs > 0) {
    if (camClockSkip(dtUs, t->xclkHz)) {
      t->skips++;
    }
    t->dtAvgUs = t->dtAvgUs <= 0.0f ? (float)dtUs : t->dtAvgUs + CAM_CLOCK_DT_ALPHA * ((float)dtUs - t->dtAvgUs);
  }
  if (t->winFrames == 0) {
    t->winT0Us = frameUs;
  }
  t->winFrames++;
  if (t->winFrames > CAM_CLOCK_WINDOW_FRAMES && frameUs > t->winT0Us) {
    t->winFps = (float)CAM_CLOCK_WINDOW_FRAMES * 1e6f / (float)(frameUs - t->winT0Us);
    t->winValid = true;
    t->winFrames = 1;
    t->winT0Us = frameUs;
  }
}

void camClockTrackFellBack(camClockTrack_t *t, uint32_t xclkHz)
{
  const uint32_t skips = t->skips;
  memset(t, 0, sizeof(*t));
  t->xclkHz = xclkHz;
  t->fellBack = true;
  t->skips = skips;
}

camClockIn_t camClockTrackIn(const camClockTrack_t *t, bool awake, bool autonomyActive, bool onGround,
                             bool takeoffWindow)
{
  const camClockIn_t in = {
    .xclkHz = t->xclkHz,
    .fellBack = t->fellBack,
    .clockFrames = t->clockFrames,
    .windowValid = t->winValid,
    .windowFps = t->winFps,
    .procAvgUs = t->procAvgUs,
    .awake = awake,
    .autonomyActive = autonomyActive,
    .onGround = onGround,
    .takeoffWindow = takeoffWindow,
  };
  return in;
}

bool camClockPending(const camClockTrack_t *t)
{
  return t->xclkHz == CAM_CLOCK_SLOW_HZ && !t->fellBack &&
         (t->clockFrames < CAM_CLOCK_DECIDE_FRAMES || !t->winValid);
}

bool camClockTooSlow(const camClockTrack_t *t)
{
  return t->xclkHz == CAM_CLOCK_SLOW_HZ && t->clockFrames >= CAM_CLOCK_DECIDE_FRAMES &&
         t->procAvgUs > CAM_CLOCK_PROC_MAX_US;
}

float camClockPublishFps(const camClockTrack_t *t, uint32_t framesSinceWake, bool leadVisible)
{
  if (framesSinceWake < CAM_CLOCK_FRESH_FRAMES || !leadVisible || camClockPending(t) || camClockTooSlow(t) ||
      !(t->dtAvgUs > 0.0f)) {
    return 0.0f;
  }
  return 1e6f / t->dtAvgUs;
}
