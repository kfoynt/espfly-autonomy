/*
 * ESP-Drone Firmware
 *
 * cam_clock.h - The flow camera's input clock against the flow loop's rate
 * (docs/autonomy-a2b.md section 17.2)
 *
 * The OV3660 at QQVGA grayscale (esp32-camera's set_pll(8, 1, 0, ...), PCLK
 * divider 8) runs a frame of 2300 x 783 pixel clocks, PCLK = XCLK / 2, so
 * fps = 2 x XCLK / (2300 x 783): 22.21 fps at the old 20 MHz (45.0 ms), while
 * the flow loop processes a frame in ~61 ms (the logged intervals were exactly
 * 45 / 90 ms, 14.73-14.90 fps processed). At XCLK = 160 MHz / 12 = 13.33 MHz
 * the sensor runs at 14.81 fps (67.5 ms), the rate the loop already uses,
 * with regular frames. Never 16 MHz (esp32-camera's cam_hal.c switches to its
 * PSRAM DMA mode there) and not 160 / 11 (16.15 fps: 61.9 ms is shorter than
 * the processing, and the processed rate would fall to ~10.7-11.8 fps).
 *
 * The rolling shutter reads the frame out over 768 of its 783 lines, so the
 * frame's motion is centred later on a slower clock: the gyro window offset
 * (15 ms fitted at 20 MHz on 2026-10-10) grows by half the extra readout,
 * rounded to the gyro's 1 ms: 26 ms at 13.33 MHz (a model value, re-fitted on
 * the bench with espfly_diag.py flow-cal --offset-scan).
 *
 * The fallback: on the ground, awake, autonomy idle and outside a take-off
 * window, a flow loop slower than CAM_CLOCK_PROC_MAX_US on average (after
 * CAM_CLOCK_DECIDE_FRAMES frames at 13.33 MHz, counted across standby wakes),
 * or processing fewer than CAM_CLOCK_FPS_MIN frames a second over a full window
 * of CAM_CLOCK_WINDOW_FRAMES contiguous frames, switches the sensor back to
 * 20 MHz for the rest of the boot (once; never in flight). Until the check has
 * decided (camClockPending()) the camera does not sleep on the ground (cam_gnd.h),
 * so the decision is made in the first ~10-15 s after the camera starts, also on
 * a slow boot (P ~88 ms: 9.9 fps at 13.33 MHz). No frame rate is published
 * (camClockPublishFps(): a FLIGHT ARM is refused CAMERA, G4) before the check
 * has decided, nor while the loop is too slow for 13.33 MHz and could not fall
 * back. The published rate is the processed one, 1 / EMA(frame interval) (the
 * EMA of 1 / interval read 10.9-11.3 fps on the 67.5 / 135 ms alternation of a
 * 9.9 fps loop). Pure,
 * host-tested (tools/tests/test_cam_clock.c, with a model of the driver's frame
 * queue and the whole boot -> sleep -> PHOTO -> ARM timeline).
 */
#ifndef CAM_CLOCK_H_
#define CAM_CLOCK_H_

#include <stdbool.h>
#include <stdint.h>

#define CAM_CLOCK_SLOW_HZ        13333333u   /* 160 MHz / 12: 14.81 fps (NVS camclk 1, the default) */
#define CAM_CLOCK_FAST_HZ        20000000u   /* 160 MHz / 8: 22.21 fps (camclk 0, and the fallback) */
#define CAM_CLOCK_PSRAM_MODE_HZ  16000000u   /* esp32-camera's PSRAM DMA mode: never */
#define CAM_CLOCK_LINE_PCLKS     2300u       /* OV3660 binned frame: HTS ... */
#define CAM_CLOCK_FRAME_LINES    783u        /* ... and VTS */
#define CAM_CLOCK_READOUT_LINES  768u        /* lines read out of each frame */
#define CAM_CLOCK_OFFSET_REF_US  15000u      /* gyro window offset fitted at 20 MHz (2026-10-10) */
#define CAM_CLOCK_DECIDE_FRAMES  100u        /* frames at 13.33 MHz before the processing-time rule decides (the
                                              * EMA's start has faded to 0.6 %): ~7 s at 14.8 fps, ~10 s at 9.9 */
#define CAM_CLOCK_WINDOW_FRAMES  150u        /* the processed-rate window (contiguous frames, restarted at every
                                              * wake): ~10 s at 14.8 fps, ~15 s at 9.9 fps */
#define CAM_CLOCK_PROC_MAX_US    62000.0f    /* average frame processing above this: fall back */
#define CAM_CLOCK_FPS_MIN        14.0f       /* processed frames per second below this: fall back */
#define CAM_CLOCK_PROC_ALPHA     0.05f       /* EMA weight of the frame processing time */
#define CAM_CLOCK_DT_ALPHA       0.10f       /* EMA weight of the processed frame interval (the published rate) */
#define CAM_CLOCK_FRESH_FRAMES   2u          /* frames after a wake before a rate is published (one interval) */
#define CAM_CLOCK_SKIP_NUM       3u          /* a frame interval over 3 / 2 periods is a skip */
#define CAM_CLOCK_SKIP_DEN       2u

/* The XCLK of an NVS camclk code: 0 = 20 MHz, anything else 13.33 MHz. */
uint32_t camClockXclkHz(uint8_t code);
/* One of the two clocks this build runs (never 16 MHz, never 160 / 11). */
bool camClockXclkOk(uint32_t hz);
/* Sensor frame rate (fps), frame period (us) and readout (us) at this XCLK. */
float camClockFps(uint32_t hz);
uint32_t camClockPeriodUs(uint32_t hz);
uint32_t camClockReadoutUs(uint32_t hz);
/* The gyro window offset for the flow and the height observer (us). */
uint32_t camClockGyroOffsetUs(uint32_t hz);
/* A frame interval that skipped at least one frame at this clock. */
bool camClockSkip(uint32_t dtUs, uint32_t hz);
/* One processing time sample into the EMA (0 starts it). */
float camClockProcAvg(float avgUs, uint32_t busyUs);

typedef struct {
  uint32_t xclkHz;       /* the clock now */
  bool fellBack;         /* switched to 20 MHz earlier this boot */
  uint32_t clockFrames;  /* frames processed at this clock (kept across standby wakes) */
  bool windowValid;      /* a full window of CAM_CLOCK_WINDOW_FRAMES frames at this clock was measured */
  float windowFps;       /* its processed frame rate */
  float procAvgUs;       /* the processing time EMA */
  bool awake;            /* streaming */
  bool autonomyActive;
  bool onGround;
  bool takeoffWindow;
} camClockIn_t;

/* True: switch to 20 MHz now. */
bool camClockFallback(const camClockIn_t *in);

/* The camera task's per-frame bookkeeping (camera_flow.c owns one; all of it
 * here so the host test runs the same code). */
typedef struct {
  uint32_t xclkHz;       /* the clock the counts below belong to */
  bool fellBack;         /* the fallback ran this boot (whatever its outcome) */
  float procAvgUs;       /* frame processing time EMA (0: no sample) */
  uint32_t procMaxUs;
  uint32_t skips;        /* frame intervals over 1.5 sensor periods */
  uint32_t clockFrames;  /* frames processed at this clock: not reset by a standby wake */
  uint32_t winFrames;    /* frames in the current window (restarted at every wake) */
  uint64_t winT0Us;      /* the window's first frame time */
  bool winValid;         /* a full window was measured at this clock (kept across wakes) */
  float winFps;          /* the last full window's processed rate */
  float dtAvgUs;         /* EMA of the processed frame intervals (0: none yet; kept across standby wakes) */
} camClockTrack_t;

/* The camera's start at this clock: everything anew (a fallback earlier this
 * boot is kept). */
void camClockTrackInit(camClockTrack_t *t, uint32_t xclkHz);
/* A camera re-initialisation (after a stall) at the same clock: the decision
 * (frame count, processing statistics, the last full window) is kept, the
 * window in progress and the interval average start anew. */
void camClockTrackRestart(camClockTrack_t *t);
/* A wake from either standby: the rate window restarts (its frames must be
 * contiguous); the processing statistics, the frame count and the interval
 * average are kept. */
void camClockTrackWake(camClockTrack_t *t);
/* One processed frame: its capture time, its interval from the previous
 * processed frame (0: the first after a start or a wake), the busy time. */
void camClockTrackFrame(camClockTrack_t *t, uint64_t frameUs, uint32_t dtUs, uint32_t busyUs);
/* The fallback ran: the clock is now xclkHz (20 MHz, or unchanged if the
 * switch failed); the measurements start anew at it; never again this boot. */
void camClockTrackFellBack(camClockTrack_t *t, uint32_t xclkHz);
/* The fallback's inputs from the bookkeeping and the context. */
camClockIn_t camClockTrackIn(const camClockTrack_t *t, bool awake, bool autonomyActive, bool onGround,
                             bool takeoffWindow);
/* The check has not decided yet: at 13.33 MHz, not fallen back, and fewer than
 * CAM_CLOCK_DECIDE_FRAMES frames or no full window. The ground standby waits. */
bool camClockPending(const camClockTrack_t *t);
/* At 13.33 MHz with the decision's frames and a processing average over
 * CAM_CLOCK_PROC_MAX_US: the loop cannot keep up (the fallback did not or could
 * not switch). */
bool camClockTooSlow(const camClockTrack_t *t);
/* The rate to publish (cameraFlowFps(), G4): 1e6 / the interval EMA, once
 * CAM_CLOCK_FRESH_FRAMES frames came since the latest wake, the thermal lead
 * allows it, the clock check has decided and the loop is not too slow for the
 * clock; else 0. */
float camClockPublishFps(const camClockTrack_t *t, uint32_t framesSinceWake, bool leadVisible);

#endif /* CAM_CLOCK_H_ */
