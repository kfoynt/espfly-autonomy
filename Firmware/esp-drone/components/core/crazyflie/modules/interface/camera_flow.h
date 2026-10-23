/*
 * ESP-Drone Firmware
 *
 * camera_flow.h - Image-motion measurement from the XIAO ESP32S3 Sense camera
 *
 * Measurement only: nothing here feeds the controller. A low-priority task
 * captures native 320x240 frames (grey, or YUV 4:2:2 for colour photos: the fit uses
 * their luma), estimates the global image motion
 * between consecutive frames (optical_flow.h) and pairs it with the body
 * rotation integrated from the gyro over the same interval, so the
 * rotation-induced part can be removed offline or later on board.
 *
 * The camera is started after system start in its own task. If it is
 * missing or fails, flight is unaffected (cameraFlowState() reports it).
 */
#ifndef CAMERA_FLOW_H_
#define CAMERA_FLOW_H_

#include <stdbool.h>
#include <stdint.h>

#include "imu_types.h"

typedef enum {
  CAMERA_FLOW_OFF = 0,
  CAMERA_FLOW_STARTING,
  CAMERA_FLOW_RUNNING,
  CAMERA_FLOW_FAILED,
  CAMERA_FLOW_RESTARTING,   /* no frame for 4 s; re-initialising the camera */
  CAMERA_FLOW_STANDBY,      /* sensor powered down for a take-off window, or idle on the ground */
} cameraFlowState_t;

typedef struct {
  uint32_t seq;            /* processed frame counter, starts at 1 */
  uint64_t frameTimeUs;    /* physical capture-window centre time, esp_timer clock */
  uint32_t dtUs;           /* interval since the previous frame */
  float dx, dy;            /* scene shift between the frames, px at 160x120, +right/+down */
  float scale;             /* relative scale change between the frames */
  float rot;               /* image rotation between the frames, rad */
  float gyroAngle[3];      /* body rotation over the same interval, rad (body x, y, z) */
  float texture;           /* optical_flow texture measure */
  float residual;          /* optical_flow RMS residual, grey levels */
  uint8_t lum;             /* mean image brightness */
  bool ok;                 /* flow fit succeeded, gyro interval available, and no exposure step too large to correct
                            * (optical_flow.h ofExposureStepTooLarge) */
  int8_t gainE3;           /* the fit's exposure gain a (prev / cur), (a - 1) x 1000 clamped +-127; 0 without a step */
  uint8_t satPct;          /* saturated % of the fit's pixels (0..100); bit 7 (CAMERA_FLOW_SAT_EXPGATE): this frame
                            * was refused for its exposure step */
} cameraFlowSample_t;

#define CAMERA_FLOW_SAT_EXPGATE 0x80

/* The flow fit's variant (optical_flow.h ofFitMode_t; the CPU fallback of docs/autonomy-a2b.md 19.4): NVS
 * "flowfit" (default OF_FIT_ROBUST), used from the next frame. False out of range. */
bool cameraFlowSetFitMode(uint8_t mode);
uint8_t cameraFlowFitMode(void);
/* Frames refused for an exposure step since boot, and the steps the fit corrected (gain != 1). */
void cameraFlowExposureStats(uint32_t *gated, uint32_t *corrected);

void cameraFlowInit(void);

/* Called by the stabilizer task every loop with the IMU sample time
 * (esp_timer microseconds) and the body rates in deg/s. */
void cameraFlowGyroSample(uint64_t timeUs, const Axis3f *gyroDps);

bool cameraFlowGetLatest(cameraFlowSample_t *out);
extern volatile uint32_t cameraFlowMotionLagUsV; /* measured midpoint latency, published once per frame */
cameraFlowState_t cameraFlowState(void);
const char *cameraFlowStateName(void);
float cameraFlowFps(void);
int cameraFlowSensorPid(void);

/* Camera drift hold (camera_hold.h): latest correction in degrees (+roll =
 * right side down, +pitch = nose up), whether it should be applied (engaged,
 * or coasting on the learned integral while the view is lost) and when it was
 * computed (esp_timer us). The switch is persisted separately (NVS). */
void cameraFlowGetHold(float *roll, float *pitch, bool *engaged, uint64_t *timeUs);
bool cameraFlowHoldCoasting(void);
/* The HOVER scene gate (camera_hold.h camHoldSceneOk): the latest frame was
 * refused by it (recorder holdFlags bit 3), and its refusals since boot by
 * reason (counted by the camera task, reset at a camera restart). */
bool cameraFlowHoldSceneGated(void);
void cameraFlowHoldSceneStats(uint32_t *rejRes, uint32_t *rejDx, uint32_t *rejExp);
/* Interactive hover (camera_hold.h moving reference): the hold's measured
 * motion since the start of its reference epoch (lateral x / D and forward
 * ln(D0 / D), engaged frames) and that epoch. IRAM: the autonomy glue reads
 * it every 10th loop. */
void cameraFlowGetHoldProgress(float *pLat, float *pFwd, uint32_t *epoch);
/* The hold at its limit (camera_hold.h camHoldLimitStep, CAM_HOLD_LIM_*; docs/autonomy-a2b.md 27.4): written by the
 * camera task per frame; the autonomy glue reads it with its slow inputs (one byte load, inline: no call in IRAM). */
extern volatile uint8_t cameraFlowHoldLimitBitsV;
static inline uint8_t cameraFlowHoldLimitBits(void)
{
  return cameraFlowHoldLimitBitsV;
}
/* The hold's output slew (deg/s) of its latest frame (camera_hold.h camHoldSlewOf): CAM_HOLD_SLEW_DEG_S (6) but on a fast
 * move's frames (mvspeed > 100, 2026-10-16), where controller_pid.c applies the correction at the same rate. Written by
 * the camera task per frame; one float load, inline (no call in the controller). */
extern volatile float cameraFlowHoldSlewV;
static inline float cameraFlowHoldSlew(void)
{
  return cameraFlowHoldSlewV;
}
/* The hold's measured rates were not still within CAM_HOLD_STILL_S (camera_hold.h camHoldMoving; mvfix STILL,
 * 2026-10-17): written by the camera task per frame, read with the autonomy glue's slow inputs (one byte, inline). */
extern volatile uint8_t cameraFlowHoldMovingV;
static inline bool cameraFlowHoldMoving(void)
{
  return cameraFlowHoldMovingV != 0u;
}
void cameraFlowSetHoldEnabled(bool on);
bool cameraFlowHoldEnabled(void);
/* The hold's gain scale with the scene distance (camera_hold.h camHoldGainTarget, docs/autonomy-a2b.md 20): kmax
 * (NVS holdkmax, 1.0 = off .. CAM_HOLD_GAIN_SCALE_MAX, default CAM_HOLD_GAIN_KMAX_DEFAULT) is read by the camera task
 * every frame; Set refuses a value outside 1..3 (false). Info: the scale applied, its target, the latched G (px/m,
 * 0: none) and whether a MOVE ran in this sortie (written by the camera task, read by the console). */
bool cameraFlowSetHoldKmax(float kmax);
float cameraFlowHoldKmax(void);
/* The HOVER scene gate's residual limit in force, grey levels x 10 (NVS holdres, camera_hold.h CAM_HOLD_SCENE_RES_*;
 * 0: no residual test). autonomy.c sets it at ARM; the get is for auto status. */
void cameraFlowSetSceneResidual(uint8_t x10);
uint8_t cameraFlowSceneResidual(void);
/* The hold's lateral position gain on a MOVE's frames, % (NVS mvlatk, camera_hold.h latGain; 100: the hold as before;
 * clamped to 100..CAM_HOLD_LAT_GAIN_MAX x 100). autonomy.c sets it at ARM; the get is for auto status. */
void cameraFlowSetMoveLatGain(uint16_t pct);
uint16_t cameraFlowMoveLatGain(void);
void cameraFlowHoldGainInfo(float *scale, float *target, float *gPxPerM, bool *translated);

/* Frames processed, fb_get timeouts, camera restarts, age of the last frame. */
void cameraFlowStats(uint32_t *frames, uint32_t *timeouts, uint32_t *restartCount, uint32_t *ageMs);

/* Invalid frame pointers rejected (the camera stops after the first) and the
 * lowest free stack in bytes of the driver's cam_task and of the flow task
 * (UINT32_MAX until measured). */
void cameraFlowHealth(uint32_t *invalidFrames, uint32_t *camTaskFree, uint32_t *flowTaskFree);

/* Take-off standby: the sensor is powered down (software standby) during the
 * stabilizer's take-off window to take its current off the XIAO supply.
 * The switch is persisted separately (NVS). Counts: windows slept through,
 * failed SCCB writes, total ms asleep. */
void cameraFlowSetStandbyEnabled(bool on);
/* Autonomy: true keeps the camera out of (or wakes it from) the take-off
 * standby for the current take-off; the A2b glue passes auto_fsm's
 * out.camNoStandby every loop (docs/autonomy-a2.md section 4.11). */
void cameraFlowSetTakeoffStandbySkip(bool skip);
bool cameraFlowStandbyEnabled(void);
bool cameraFlowInStandby(void);
/* How long the camera sleeps after a warm or a cold ground start (200..8000
 * ms; also until 0.5 s past the lift-off thrust level). Returns false if out
 * of range. */
bool cameraFlowSetStandbyMs(uint16_t ms, bool cold);
uint16_t cameraFlowStandbyMs(bool cold);
/* Enabled, and the camera is running a sensor that supports standby. */
bool cameraFlowStandbyArmed(void);
void cameraFlowStandbyStats(uint32_t *count, uint32_t *fails, uint32_t *totalMs);

/* Ground standby (cam_gnd.h, docs/autonomy-a2b.md 17.1): the same sensor
 * standby while autonomy is idle on the ground, from 30 s after the camera
 * starts, once the wake hold has run out. The state reads
 * CAMERA_FLOW_STANDBY, as in the take-off standby; cameraFlowInStandby() and
 * the take-off counters keep their take-off meaning. Wake reasons: a wake
 * request (below), autonomy out of IDLE, off the ground (a ground start goes
 * to the take-off standby when that takes it), the switch off, a bench hold.
 * Any task: wakes the camera within ~20 ms (if asleep) and keeps it awake for
 * at least holdMs (a longer hold already running is kept; at most 1 h). The
 * link calls it with cameraFlowGroundHoldMs() on a PHOTO in IDLE and on every
 * ARM and TAKEOFF. */
void cameraFlowGroundWake(uint32_t holdMs);
/* Ends the wake hold now (bench: 'tkoff camwake 0'). */
void cameraFlowGroundRelease(void);
/* Bench: 'stream N > 0' keeps the camera awake (the 'camvz air' modes do too). */
void cameraFlowGroundBenchHold(bool on);
/* Asleep in the ground standby (not the take-off standby). */
bool cameraFlowGroundStandby(void);
/* Streaming, with at least nFrames processed since the latest wake from
 * either standby (or the camera's start). */
bool cameraFlowAwakeFresh(uint32_t nFrames);
/* NVS camgnd (default on), camgndms (5000..65000, default 45000) and camlead
 * (0..60000 ms, default 0: in IDLE the frame rate is published, so a FLIGHT
 * ARM's camera guard can pass, only this long after a ground wake). The
 * setters return false out of range. */
void cameraFlowSetGroundStandby(bool on);
bool cameraFlowGroundStandbyEnabled(void);
bool cameraFlowSetGroundHoldMs(uint32_t ms);
uint32_t cameraFlowGroundHoldMs(void);
bool cameraFlowSetGroundLeadMs(uint32_t ms);
uint32_t cameraFlowGroundLeadMs(void);
typedef struct {
  uint32_t count;        /* ground sleeps this boot */
  uint32_t fails;        /* failed standby / wake writes of the ground standby */
  uint32_t asleepMs;     /* total ms asleep on the ground (incl. a sleep in progress) */
  uint32_t awakeMs;      /* ms since the latest ground wake; 0 asleep; UINT32_MAX: never slept this boot */
  uint32_t holdLeftMs;   /* the wake hold still running, ms */
  bool asleep;
} cameraFlowGroundStats_t;
void cameraFlowGroundStats(cameraFlowGroundStats_t *st);

/* Native VGA uses the standard raw 20 MHz timing (~5.55 fps), without NVS writes.
 * Published frame and photo timestamps are the modelled capture-window centre.
 * The observer's additional gyro offset is therefore zero; ClockStats reports
 * the physical start-to-centre offset used for the flow fit itself (53 ms).
 * The original 81 ms readout model is corrected by -28 ms from accepted
 * hand-motion frames, with improvement on an independent repeat.
 * Flight stability still requires validation. */
uint32_t cameraFlowGyroOffsetUs(void);
typedef struct {
  uint32_t xclkHz;       /* the clock now */
  bool fellBack;         /* switched to 20 MHz this boot */
  uint32_t gyroOffsetUs;
  uint32_t procAvgUs;    /* frame processing time (frameGet() return to the end of the iteration), EMA 0.05 */
  uint32_t procMaxUs;    /* its maximum since the camera's start (or the fallback) */
  uint32_t skips;        /* frame intervals over 1.5 sensor periods */
  float windowFps;       /* processed frames per second over the last full 150-frame window (0: none yet) */
  uint32_t clockFrames;  /* frames processed at this clock (the decision needs CAM_CLOCK_DECIDE_FRAMES) */
  bool pending;          /* the clock check has not decided yet (no ground standby before) */
  bool tooSlow;          /* 13.33 MHz and a loop too slow for it: no frame rate published (G4 refuses CAMERA) */
} cameraFlowClockStats_t;
void cameraFlowClockStats(cameraFlowClockStats_t *st);

/* Per-stage processing times of the camera task (USB 'flowfit prof [reset]', docs/autonomy-a2b.md 30): each processed
 * frame's procAvgUs interval split into the flow fit's stages (optical_flow.h OF_STAGE_*: the downsample, the coarse
 * and the fine level, the copies) and the task's own ('snap': the mean brightness, the photo snapshot, the frame's
 * return; 'post': the gyro window, the drift hold, the sample), and the whole ('total', from the frame's arrival).
 * Measured with the core's cycle counter: wall time on core 1, so the minimum is close to the stage's own CPU time
 * and the average and the maximum include the preemption by the 1 kHz tasks and interrupts. Frames with a fit only,
 * since the boot or the last reset. */
#define CAMERA_FLOW_STAGES 7
typedef struct {
  uint32_t frames;
  uint32_t avgUs[CAMERA_FLOW_STAGES];
  uint32_t minUs[CAMERA_FLOW_STAGES];
  uint32_t maxUs[CAMERA_FLOW_STAGES];
} cameraFlowStageStats_t;
/* False: no statistics (no frame yet, or no PSRAM for them). */
bool cameraFlowStageStats(cameraFlowStageStats_t *st);
/* Starts the statistics again from the next frame. */
void cameraFlowStageReset(void);

/* Frame snapshot for another task (the autonomy link, auto_link.c). Lock-free, for a single requester: state idle
 * -> requested -> (copying ->) filled; the requester takes the frame, which returns it to idle.
 *
 * Grey (NVS photocol 0, the default): a copy of the next processed frame is made inside the camera task between the
 * flow fit and returning the frame, one memcpy (160x120 Y8, 19200 bytes), so the flow and the drift hold are not
 * disturbed.
 * Colour (photocol 1, always on for the boot: docs/autonomy-a2b.md 28): the camera driver's own copy task writes the
 * next frame to *start* after the request, raw YUV 4:2:2 (YUYV, 38400 bytes, Y0 U Y1 V), straight into buf
 * (cam_split_core.h); the camera task only fills meta when it processes that frame (copying is meta only). A photo
 * arrives ~70-190 ms later than in grey; the flow fit gets the frame's Y bytes, the same values the sensor's Y8 output
 * gives. The buffer must hold CAMERA_FLOW_SNAPSHOT_MAX_BYTES. */
bool cameraVgaTimingOk(void);

#include "camera_profile.h"
#define CAMERA_FLOW_SNAPSHOT_BYTES     (CAMERA_NATIVE_WIDTH * CAMERA_NATIVE_HEIGHT)          /* a grey frame */
#define CAMERA_FLOW_SNAPSHOT_MAX_BYTES (2 * CAMERA_NATIVE_WIDTH * CAMERA_NATIVE_HEIGHT)      /* a colour frame */
#define CAMERA_FLOW_PIXFMT_JPEG         2
#define CAMERA_FLOW_PIXFMT_GRAY8       0                    /* = auto_proto.h AUTO_PIXFMT_GRAY8 */
#define CAMERA_FLOW_PIXFMT_YUYV        1                    /* = AUTO_PIXFMT_YUYV: Y0 U Y1 V, 320 bytes a row */

typedef struct {
  uint32_t seq;            /* processed frame counter (cameraFlowSample_t.seq) */
  uint64_t frameTimeUs;    /* capture (start of frame) time, esp_timer clock */
  float texture;           /* optical_flow texture measure */
  uint8_t lum;             /* mean image brightness */
  uint8_t pixFmt;          /* CAMERA_FLOW_PIXFMT_*: the copy's format */
  uint32_t bytes;          /* the copy's length: 19200 (grey) or 38400 (YUYV); u16 keeps the struct at 24 bytes (the
                            * link's three photo frames are internal RAM) */
} cameraFlowSnapshot_t;

/* The photos' pixel format (CAMERA_FLOW_PIXFMT_*): YUYV while the colour split runs (NVS photocol 1 and its boot
 * check passed, docs/autonomy-a2b.md 28), else GRAY8. */
uint8_t cameraFlowPhotoPixFmt(void);

/* The colour split (USB 'status': camSplit=, camSplitArm= ...; cam_split_core.h). split: 0 off (photocol 0), 1 ok, 2
 * bad (its boot check failed: grey for the boot). The counters are the snapshot core's (u16, they wrap). */
typedef struct {
  uint8_t split;
  uint16_t arm;          /* captures latched at a frame start */
  uint16_t done;         /* photos filled */
  uint16_t rearm;        /* a newer frame than the captured one reached the flow (GRAB_LATEST drop, stale skip) */
  uint16_t cancel;       /* pending requests cancelled (auto_link's 1 s timeout, a released set) */
  uint16_t restart;      /* captures restarted by an aborted frame (or a chunk out of sequence) */
  uint16_t resets;       /* captures re-armed by a camera (re)initialisation */
} cameraFlowSplitStats_t;
void cameraFlowSplitStats(cameraFlowSplitStats_t *st);
const char *cameraFlowSplitName(uint8_t split);   /* "off", "ok", "bad" */

/* Asks for the next processed frame in buf (CAMERA_FLOW_SNAPSHOT_MAX_BYTES) and
 * its data in meta; both must stay valid until taken or cancelled. False
 * unless the camera is running, or while an earlier request is outstanding. */
bool cameraFlowSnapshotRequest(uint8_t *buf, cameraFlowSnapshot_t *meta);
/* True once the copy is complete; the request is then closed. */
bool cameraFlowSnapshotTake(void);
/* Withdraws an outstanding request (or drops a finished one). False while the
 * camera task is copying (a fraction of a millisecond; call again). */
bool cameraFlowSnapshotCancel(void);

#endif /* CAMERA_FLOW_H_ */
