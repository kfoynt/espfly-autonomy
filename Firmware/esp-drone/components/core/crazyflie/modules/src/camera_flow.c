/*
 * ESP-Drone Firmware
 *
 * camera_flow.c - Image-motion measurement from the XIAO ESP32S3 Sense camera
 */
#include "sdkconfig.h"
#include "esp_attr.h"
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "esp_camera.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_timer.h"

#include "camera_flow.h"
#include "camera_cadence.h"
#include "camera_hold.h"
#include "cam_split.h"
#include "cam_clock.h"
#include "cam_gnd.h"
#include "cam_vz_glue.h"
#include "autonomy.h"
#include "flight_recorder.h"
#include "imu_level_cal_store.h"
#include "optical_flow.h"
#include "stabilizer.h"
#include "config.h"
#include "system.h"
#include "stm32_legacy.h"
#include "bench_bypass.h"
#define DEBUG_MODULE "CAMFLOW"
#include "debug_cf.h"

#if !DRONE_SENSE_CAMERA_PINS_RESERVED
#error "camera_flow needs the Sense camera pin reservation (DRONE_SENSE_CAMERA_PINS_RESERVED)"
#endif

#define CAMERA_FLOW_TASK_STACK  4096 /* measured VGA high-water use < 2.8 KiB; >1.3 KiB reserve */
#define CAMERA_START_DELAY_MS   1500   /* after system start, away from boot inrush */
#define GYRO_RING_LEN           512    /* 0.5 s of 1 kHz samples */
#define DEG2RAD                 0.017453292f
/* The frame timestamp is taken at start of frame; with the rolling shutter
 * the image is effectively exposed later. A hand-held rotation recording
 * (2026-10-10, OV3660, 160x120, XCLK 20 MHz) fitted best with the gyro window
 * shifted by +10..+20 ms (15 ms used). The offset now follows the clock
 * (cam_clock.h camClockGyroOffsetUs(): 15 ms at 20 MHz, 26 ms at 13.33 MHz,
 * the readout being 1.5 x longer), and the camera height observer uses the
 * same value (cam_vz_glue.c). */

/* Image shift per radian of body rotation, from the hand-held calibration
 * (2026-10-10, OV3660 160x120, R^2 0.97): dx = KX_GX*gx + KX_GY*gy + KX_GZ*gz.
 * KX_GZ is also the horizontal focal length in pixels. */
#define FLOW_KX_GX   (-1.0f)
#define FLOW_KX_GY   (-4.9f)
#define FLOW_KX_GZ   102.4f

/* Drift-hold gating: lift-off is above ~30000 thrust with the camera; the
 * view must have texture, a good fit and slow yaw (camera_hold.h
 * camHoldYawOk: 0.35 rad/s, AUTO_MV_TURN_GATE_RPS in a commanded turn). */
#define CAM_HOLD_AIRBORNE_THRUST 30000
#define CAM_HOLD_MIN_TEXTURE     8.0f
#define CAM_HOLD_MAX_RESIDUAL    15.0f
/* An exposure step larger than this is refused, not corrected (optical_flow.h OF_GAIN_STEP_MAX): gamma and the
 * saturation leave part of a large step uncorrected. One flag (s.ok) gates the hold (it coasts) and the camera
 * height observer (it refuses the frame). */
#define CAM_FLOW_GAIN_STEP_MAX   OF_GAIN_STEP_MAX

/* Take-off standby. OV3660/OV5640 SYSTEM CTROL0 bit 6 is software power down;
 * the whole register is written with the values esp32-camera's own init uses
 * (0x42 while configuring, 0x02 to stream), never read-modify-write: a failed
 * SCCB read would otherwise set bit 7, a soft reset. The sensor is fed from
 * the XIAO's VIN behind the battery P-FET, the path whose margin runs out at
 * lift-off. */
#define CAMERA_STANDBY_REG       0x3008
#define CAMERA_STANDBY_ON        0x42
#define CAMERA_STANDBY_OFF       0x02
/* Asleep for this long after a ground start (most take-off resets came
 * 0.65-1.0 s after throttle-up, a few up to 2.2 s; the hold needs the camera
 * soon after for a steady climb-out), longer on a cold take-off (the first
 * after a break, which has the least supply margin: one browned out 0.7 s
 * after a 1.5 s wake), and until this long past the lift-off thrust level on
 * a slow throttle-up. */
#define CAMERA_STANDBY_MS_DEFAULT      1500
#define CAMERA_STANDBY_COLD_MS_DEFAULT 3000
#define CAMERA_STANDBY_MS_MIN          200
#define CAMERA_STANDBY_MS_MAX          8000
#define CAMERA_STANDBY_AFTER_LIFT_MS   500

/* XIAO ESP32S3 Sense camera connector (OV2640 / OV3660). The motors use LEDC
 * timer 0 / channels 4-7, so XCLK uses timer 2 / channel 2. SCCB uses I2C
 * port 1, unused by this build (I2C0 is the IMU). */
static camera_config_t cameraConfig = {
  .pin_pwdn = -1,
  .pin_reset = -1,
  .pin_xclk = 10,
  .pin_sccb_sda = 40,
  .pin_sccb_scl = 39,
  .pin_d7 = 48,
  .pin_d6 = 11,
  .pin_d5 = 12,
  .pin_d4 = 14,
  .pin_d3 = 16,
  .pin_d2 = 18,
  .pin_d1 = 17,
  .pin_d0 = 15,
  .pin_vsync = 38,
  .pin_href = 47,
  .pin_pclk = 13,
  .xclk_freq_hz = CAM_CLOCK_SLOW_HZ,   /* set from NVS camclk by cameraFlowInit() (cam_clock.h, docs/autonomy-a2b.md 17.2) */
  .ledc_timer = LEDC_TIMER_2,
  .ledc_channel = LEDC_CHANNEL_2,
  .pixel_format = PIXFORMAT_GRAYSCALE,   /* PIXFORMAT_YUV422 for colour photos: set by cameraStart() (splitWanted) */
  .frame_size = FRAMESIZE_QVGA,
  .jpeg_quality = 12,
  .fb_count = 3,
  .fb_location = CAMERA_FB_IN_PSRAM,  /* internal RAM is too fragmented for 2 x 19.2 KB (the same in colour) */
  .grab_mode = CAMERA_GRAB_LATEST,
  .sccb_i2c_port = 1,
};

/* Colour photos, always on when enabled (user requests 2026-10-15 and 2026-10-16, docs/autonomy-a2b.md 28): with NVS
 * photocol 1 (opt-in; default 0) the sensor streams YUV 4:2:2 (YUYV) instead of its Y8 output on every frame, at the
 * same 160x120 and clock (the OV3660's frame timing and PLL are the same for both formats: the frame rate, the rolling
 * shutter and the gyro window offset do not change). The driver's own copy splits it (cam_split.c, three link-time
 * wrappers; cam_split_core.h): the frame buffers get the Y bytes (19200 B, the driver's IRAM "YUV to grayscale" copy,
 * grey's DMA geometry), so the flow, the hold and the camera height read exactly grey's frame layout and code path, and
 * only the frame a photo request arms is also copied raw (38400 B YUYV) from the internal DMA ring straight into the
 * requester's PSRAM slot. No luma buffer, no extra pass in this task, and no snapshot copy here in colour: the photo
 * frame costs this task less than a grey one. A boot check (camSplitInitInfo: fb 19200, recv 38400, half 3840, cnt 4)
 * falls back to grey for the boot (camSplit=bad). photocol 0 is the Y8 path of before, unchanged. Read at boot
 * (cameraFlowInit). */
static bool splitWanted;                  /* NVS photocol 1 */
#define SPLIT_OFF 0u
#define SPLIT_OK  1u
#define SPLIT_BAD 2u
static volatile uint8_t splitStat = SPLIT_OFF;
static volatile bool photoColour;         /* the split is running: colour photos (set by cameraStart) */

typedef struct {
  uint64_t t;
  float angle[3];
} gyroIntegral_t;

/* Written only by the stabilizer task; entries are read well behind the head.
 * 12 KB, in PSRAM, allocated by cameraFlowInit() only when the camera is
 * enabled: both users are regular tasks, which never run with the cache
 * off (IRAM code placement is for performance, not cache-off execution).
 * PSRAM only: the frame buffers need it anyway, and 12 KB of
 * internal RAM would take the heap below autonomy's G9 minimum. NULL with the
 * camera off or before the allocation: cameraFlowGyroSample() returns at
 * once and gyroAngleAt() has no data. */
static gyroIntegral_t *gyroRing;
static uint32_t gyroHead;            /* number of samples written (atomic) */
static float gyroAccum[3];
static uint64_t gyroLastT;

static portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;
static cameraFlowSample_t latest;
static bool haveLatest;
static volatile cameraFlowState_t state = CAMERA_FLOW_OFF;
static volatile float fps;
static int sensorPid = -1;
static volatile uint32_t framesTotal;
static volatile uint32_t nullFrames;     /* esp_camera_fb_get() timeouts (4 s each) */
static volatile uint32_t restarts;
static volatile uint32_t badFrames;      /* invalid frame pointers from the driver */
static volatile uint32_t camTaskStackFree = UINT32_MAX;   /* bytes, lowest seen */
static volatile uint32_t flowTaskStackFree = UINT32_MAX;
static volatile uint64_t lastFrameUs;
volatile uint32_t cameraFlowMotionLagUsV;

static volatile bool standbyEnabled = true;
static volatile bool inStandby;
static volatile uint32_t standbyCount;
static volatile uint32_t standbyFails;
static volatile uint32_t standbyMs;
static volatile uint16_t standbyLenMs = CAMERA_STANDBY_MS_DEFAULT;
static volatile uint16_t standbyColdMs = CAMERA_STANDBY_COLD_MS_DEFAULT;
/* Autonomy (auto_fsm out.camNoStandby): no take-off standby for this take-off,
 * set by the glue every loop. Clearing it does not put an awake camera to sleep
 * (the standby is entered once per ground start); a camera re-initialisation
 * still waits for the window to end. */
static volatile bool standbySkip;

/* Ground standby (cam_gnd.h, docs/autonomy-a2b.md 17.1): the same register
 * write as the take-off standby, while autonomy is idle on the ground. Kept
 * apart from the take-off standby's inStandby and counters, so
 * cameraFlowInStandby() and FSUM_MOTOR_CAMSB_DID keep their take-off meaning;
 * the state reads CAMERA_FLOW_STANDBY for both. gnd: the camera task, except
 * the wake hold (camGndWake / camGndRelease, any task, under gndMux). */
static camGnd_t gnd;
static portMUX_TYPE gndMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool gndEnabled = true;                          /* NVS camgnd */
static volatile uint16_t gndHoldMs = CAM_GND_HOLD_MS_DEFAULT;    /* NVS camgndms */
static volatile uint16_t gndLeadMs;                              /* NVS camlead */
static volatile bool gndBenchHold;                               /* 'stream N > 0' */
static volatile bool gndAsleep;                                  /* gnd.asleep, for other tasks */
static uint32_t camStartMs;                                      /* the camera's latest start or restart */
/* Frames processed since the latest wake from either standby (or the start):
 * a photo after a ground standby waits for settled frames (auto_link.c), and
 * a rate is published only after CAM_CLOCK_FRESH_FRAMES of them. */
static volatile uint32_t framesSinceWake;

/* The sensor clock (cam_clock.h, docs/autonomy-a2b.md 17.2): NVS camclk picks
 * 13.33 MHz (14.81 fps, the default) or 20 MHz (22.21 fps); a flow loop too
 * slow for 13.33 MHz on the ground switches to 20 MHz for the rest of the
 * boot, decided before the camera first sleeps on the ground. clk (the
 * per-frame processing statistics, the decision's frame count, the rate
 * window and the published rate's interval average) is written by the camera
 * task only; other tasks read its fields for 'status'. */
static volatile uint32_t xclkHz = CAM_CLOCK_SLOW_HZ;
static volatile uint32_t gyroOffsetUs;      /* the flow's gyro window offset (set with the clock) */
static camClockTrack_t clk;

/* The per-stage times (cameraFlowStageStats): CPU cycles of core 1 (the camera task is pinned there), the stages in
 * cameraFlowStageStats_t's order. PSRAM, allocated by the camera task with the fit state (NULL: none); written by the
 * camera task once a frame, read by the console, both under stageMux (not sampleMux: the camera height observer's
 * IRAM path takes that one 100 times a second). */
typedef struct {
  uint32_t frames;
  uint64_t sum[CAMERA_FLOW_STAGES];
  uint32_t min[CAMERA_FLOW_STAGES];
  uint32_t max[CAMERA_FLOW_STAGES];
} stageAcc_t;
static stageAcc_t *stageAcc;
static volatile bool stageResetReq;
static portMUX_TYPE stageMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t msNow(void)
{
  return (uint32_t)(esp_timer_get_time() / 1000);
}

/* The flow fit's variant (NVS flowfit, 'flowfit' on the console): read by the camera task before every fit. */
static volatile uint8_t fitMode = OF_FIT_ROBUST;
static volatile uint32_t expGated, expCorrected;

static volatile bool holdEnabled;
static camHoldState_t hold;          /* camera task only */
static camHoldScene_t holdScene;     /* camera task only (lastRejected also read by the recorder): the HOVER scene
                                      * gate (camera_hold.h) */
static portMUX_TYPE holdMux = portMUX_INITIALIZER_UNLOCKED;
static float holdRoll, holdPitch;    /* latest correction, deg */
static bool holdEngaged;                 /* output applied: engaged or coasting */
static bool holdCoasting;
static uint64_t holdTimeUs;
static float holdProgLat, holdProgFwd;   /* interactive hover: progress in the reference epoch holdEpoch */
static uint32_t holdEpoch;
/* The hold at its limit (camera_hold.h camHoldLimitStep, docs/autonomy-a2b.md 27.4): the camera task's runs and the
 * bits it publishes for the autonomy glue (camera_flow.h cameraFlowHoldLimitBits: one aligned byte, no lock). */
static camHoldLimit_t holdLim;           /* camera task only */
volatile uint8_t cameraFlowHoldLimitBitsV;
volatile float cameraFlowHoldSlewV = CAM_HOLD_SLEW_DEG_S;   /* (camera_flow.h cameraFlowHoldSlew) */
volatile uint8_t cameraFlowHoldMovingV;   /* (camera_flow.h cameraFlowHoldMoving: mvfix STILL) */
/* The gain scale with the scene distance (camera_hold.h camHoldGainTarget, docs/autonomy-a2b.md 20): the latch, the
 * scale applied and its target are written by the camera task only ('holdk show' reads them: single aligned 32-bit
 * stores, no tearing); kmax (NVS holdkmax) is set by the console. */
static camHoldGainLatch_t holdGain;
static volatile float holdGainS = 1.0f, holdGainTgt = 1.0f;
static float holdLatGainS = 1.0f;
static volatile float holdKmax = CAM_HOLD_GAIN_KMAX_DEFAULT;
/* the HOVER scene gate's residual limit, grey levels x 10 (NVS holdres, latched at ARM by autonomy.c; camera_hold.h
 * CAM_HOLD_SCENE_RES_*; 2026-10-22): applied to holdScene by the setter and again after every camHoldSceneReset */
static volatile uint8_t holdSceneResX10 = CAM_HOLD_SCENE_RES_X10_DEFAULT;
/* the hold's lateral position gain on a MOVE's frames, % (NVS mvlatk, latched at ARM by autonomy.c; camera_hold.h
 * latGain; 2026-10-22 "the left slide"): 100 is the hold as before */
static volatile uint16_t holdMoveLatPct;   /* (0 = 100: no initialiser, .bss) */

static void holdSceneApplyRes(void)
{
  camHoldSceneSetMaxResidual(&holdScene, (float)holdSceneResX10 * 0.1f);
}

/* Frame snapshot handshake (camera_flow.h). The requester writes the
 * destination before publishing SNAP_REQUESTED; the camera task claims it
 * (SNAP_COPYING) so a cancel cannot race the copy. */
#define SNAP_IDLE       0u
#define SNAP_REQUESTED  1u
#define SNAP_FILLED     2u
#define SNAP_COPYING    3u
static uint32_t snapState = SNAP_IDLE;   /* atomic */
static uint8_t *snapBuf;
static cameraFlowSnapshot_t *snapMeta;

_Static_assert(CAMERA_FLOW_SNAPSHOT_BYTES == OF_WIDTH * OF_HEIGHT, "snapshot size is the flow frame size");
_Static_assert(CAMERA_FLOW_SNAPSHOT_MAX_BYTES == 2 * OF_WIDTH * OF_HEIGHT &&
               CAMERA_FLOW_SNAPSHOT_MAX_BYTES <= UINT32_MAX, "a colour snapshot is the YUYV frame (its length a u16)");
_Static_assert(sizeof(cameraFlowSnapshot_t) == 32, "the snapshot's format and length fill its padding");
_Static_assert(sizeof(cameraFlowSample_t) == 64, "the exposure fields fill the sample's padding (docs/autonomy-a2b.md 19.4)");

void cameraFlowGyroSample(uint64_t timeUs, const Axis3f *gyroDps)
{
  gyroIntegral_t *const ring = __atomic_load_n(&gyroRing, __ATOMIC_ACQUIRE);
  if (ring == NULL) {
    return;   /* camera off (or no PSRAM): nothing to record */
  }
  if (gyroLastT != 0 && timeUs > gyroLastT) {
    float dt = (timeUs - gyroLastT) * 1e-6f;
    if (dt < 0.05f) {
      gyroAccum[0] += gyroDps->x * DEG2RAD * dt;
      gyroAccum[1] += gyroDps->y * DEG2RAD * dt;
      gyroAccum[2] += gyroDps->z * DEG2RAD * dt;
    }
  }
  gyroLastT = timeUs;

  uint32_t head = __atomic_load_n(&gyroHead, __ATOMIC_RELAXED);
  gyroIntegral_t *e = &ring[head % GYRO_RING_LEN];
  e->t = timeUs;
  e->angle[0] = gyroAccum[0];
  e->angle[1] = gyroAccum[1];
  e->angle[2] = gyroAccum[2];
  /* Publish only after the entry is complete (the reader may be on the other core). */
  __atomic_store_n(&gyroHead, head + 1, __ATOMIC_RELEASE);
}

/* Integrated body angle at time t, linearly interpolated. */
static bool gyroAngleAt(uint64_t t, float out[3])
{
  const gyroIntegral_t *const ring = __atomic_load_n(&gyroRing, __ATOMIC_ACQUIRE);
  uint32_t head = __atomic_load_n(&gyroHead, __ATOMIC_ACQUIRE);
  if (ring == NULL || head < 2) {
    return false;
  }
  /* Walk back from the newest sample; frames are at most ~0.2 s old. */
  uint32_t oldest = head > GYRO_RING_LEN - 64 ? head - (GYRO_RING_LEN - 64) : 0;
  for (uint32_t i = head - 1; i > oldest; i--) {
    const gyroIntegral_t *a = &ring[(i - 1) % GYRO_RING_LEN];
    const gyroIntegral_t *b = &ring[i % GYRO_RING_LEN];
    if (a->t <= t && t <= b->t) {
      float f = (b->t > a->t) ? (float)(t - a->t) / (float)(b->t - a->t) : 0.0f;
      for (int k = 0; k < 3; k++) {
        out[k] = a->angle[k] + f * (b->angle[k] - a->angle[k]);
      }
      return true;
    }
  }
  return false;
}

/* esp32-camera 2.1.0 (cam_hal.c): the frame queue behind esp_camera_fb_get(). */
extern camera_fb_t *cam_take(TickType_t timeout);

/*
 * esp_camera_fb_get() writes into the frame header before returning it. On
 * 2026-10-10 the pointer it took from the driver's queue was garbage
 * (~0x0000abab) and that write crashed the drone in flight. Take the frame
 * from the queue directly and check the pointer before touching it: the frame
 * headers live in RAM and the pixel buffers in PSRAM (fb_location).
 */
static camera_fb_t *frameGet(bool *bad)
{
  *bad = false;
  camera_fb_t *fb = cam_take(4000 / portTICK_PERIOD_MS);
  if (!fb) {
    return NULL;
  }
  /* colour (the split): exactly the 19200 B Y frame, grey's checks otherwise */
  if (((uintptr_t)fb & 3u) != 0 || !(esp_ptr_internal(fb) || esp_ptr_external_ram(fb)) ||
      !esp_ptr_external_ram(fb->buf) || fb->len < OF_WIDTH * OF_HEIGHT ||
      fb->len > (photoColour ? 1u : 2u) * OF_WIDTH * OF_HEIGHT || !esp_ptr_external_ram(fb->buf + fb->len - 1)) {
    *bad = true;
    return NULL;
  }
  return fb;
}

/* One processed frame's stage times (cycles) into the statistics. */
static void stageAdd(const uint32_t cyc[CAMERA_FLOW_STAGES])
{
  stageAcc_t *a = stageAcc;
  if (a == NULL) {
    return;
  }
  portENTER_CRITICAL(&stageMux);
  if (stageResetReq || a->frames == 0) {
    stageResetReq = false;
    memset(a, 0, sizeof(*a));
    memset(a->min, 0xFF, sizeof(a->min));
  }
  a->frames++;
  for (int i = 0; i < CAMERA_FLOW_STAGES; i++) {
    a->sum[i] += cyc[i];
    if (cyc[i] < a->min[i]) {
      a->min[i] = cyc[i];
    }
    if (cyc[i] > a->max[i]) {
      a->max[i] = cyc[i];
    }
  }
  portEXIT_CRITICAL(&stageMux);
}

/* Lowest free stack (bytes) of the driver's capture task and of this task.
 * Called from this task only, so the driver task cannot be deleted under us
 * (restarts happen here too). */
static void updateStackStats(void)
{
  TaskHandle_t camTask = xTaskGetHandle("cam_task");
  if (camTask) {
    uint32_t f = uxTaskGetStackHighWaterMark(camTask);
    if (f < camTaskStackFree) {
      camTaskStackFree = f;
    }
  }
  uint32_t f = uxTaskGetStackHighWaterMark(NULL);
  if (f < flowTaskStackFree) {
    flowTaskStackFree = f;
  }
}

/* Every (re)start. In colour the split's boot check after the init: its geometry, or grey for the rest of the boot
 * (deinit, then the Y8 init). The snapshot core was reset inside the init (the sample-mode wrapper). */
static esp_err_t cameraStart(void)
{
  /* User-selected native QVGA at the standard 20 MHz raw timing (~11.1 fps).
   * Camera backend only; saved NVS and controller gains remain unchanged. */
  xclkHz = CAM_CLOCK_FAST_HZ;
  gyroOffsetUs = camClockGyroOffsetUs(xclkHz);
  camVzGlueSetGyroOffsetUs(0);
  cameraConfig.xclk_freq_hz = (int)xclkHz;
  const bool split = splitWanted && splitStat != SPLIT_BAD;
  camSplitEnable(split);
  cameraConfig.pixel_format = split ? PIXFORMAT_YUV422 : PIXFORMAT_GRAYSCALE;
  esp_err_t err = esp_camera_init(&cameraConfig);
  if (err != ESP_OK) {
    return err;
  }
  if (!cameraVgaTimingOk()) {
    esp_camera_deinit();
    return ESP_ERR_NOT_SUPPORTED;
  }
  if (split) {
    camSplitInitInfo_t info;
    camSplitInitInfo(&info);
    if (info.active && info.geometryOk) {
      if (splitStat != SPLIT_OK) {
        DEBUG_PRINTI("camera: colour split ok (fb %u recv %u half %u cnt %u)", (unsigned)info.fbSize,
                     (unsigned)info.recvSize, (unsigned)info.halfSize, (unsigned)info.halfCnt);
      }
      splitStat = SPLIT_OK;
      photoColour = true;
    } else {
      DEBUG_PRINTW("camera: colour split boot check failed (active %d fb %u recv %u half %u cnt %u): grey photos "
                   "for this boot", info.active, (unsigned)info.fbSize, (unsigned)info.recvSize,
                   (unsigned)info.halfSize, (unsigned)info.halfCnt);
      splitStat = SPLIT_BAD;
      photoColour = false;
      (void)camSplitCancel(&camSplitSnap);   /* a request armed in the split: the requester times it out */
      esp_camera_deinit();
      vTaskDelay(M2T(200));   /* as the stall restart */
      camSplitEnable(false);
      cameraConfig.pixel_format = PIXFORMAT_GRAYSCALE;
      err = esp_camera_init(&cameraConfig);
      if (err != ESP_OK) {
        return err;
      }
    }
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) {
    sensorPid = sensor->id.PID;
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);
  }
  return ESP_OK;
}

static bool standbySupported(void)
{
  return sensorPid == OV3660_PID || sensorPid == OV5640_PID;
}

static bool sensorStandby(bool on)
{
  sensor_t *sensor = esp_camera_sensor_get();
  return sensor && sensor->set_reg &&
         sensor->set_reg(sensor, CAMERA_STANDBY_REG, 0xFF, on ? CAMERA_STANDBY_ON : CAMERA_STANDBY_OFF) == 0;
}

/* The wake write, at most 3 tries 10 ms apart; returns the failed writes. If
 * none landed, the next frame wait times out and the stall path
 * re-initialises the camera (SCCB reset). */
static uint32_t sensorWake(void)
{
  uint32_t failed = 0;
  for (int i = 0; i < 3; i++) {
    if (sensorStandby(false)) {
      return failed;
    }
    failed++;
    vTaskDelay(M2T(10));
  }
  return failed;
}

/* After a wake from either standby (or a start): frames from now on are
 * fresh, and no rate is published until CAM_CLOCK_FRESH_FRAMES of them came
 * (autonomy's G4, camFps over AUTO_FPS_MIN, needs a streaming camera, not the
 * rate from before the sleep). The published rate itself is the interval
 * average kept across the sleep (one lucky interval cannot pass G4), and the
 * rate window restarts (a full one before stays valid). */
static void markWake(void)
{
  framesSinceWake = 0;
  fps = 0.0f;
  camClockTrackWake(&clk);
}

/* The camera's part of the stabilizer's take-off window. */
static bool cameraTakeoffWindow(void)
{
  return stabilizerTakeoffPhase(stabilizerTakeoffCold() ? standbyColdMs : standbyLenMs, CAMERA_STANDBY_AFTER_LIFT_MS);
}

/* Asleep for it, unless autonomy keeps the camera awake for this take-off. */
static bool cameraTakeoffPhase(void)
{
  return !standbySkip && cameraTakeoffWindow();
}

/* No camera (re)initialisation while it should be asleep for a take-off: it
 * is the heaviest camera load (SCCB burst, sensor start-up). */
static void waitOutTakeoff(void)
{
  while (cameraTakeoffWindow()) {   /* also when the standby is skipped */
    vTaskDelay(M2T(50));
  }
}

/* One drift-hold update; publishes the output for the controller. viewOk:
 * the frame's own tests but the yaw rate; yawAngle: its gyro angle about z
 * (rad) for the yaw gate (wider in a commanded turn of the interactive hover:
 * the recorder's mvState, as the scene gate takes its HOVER). residual, dxRes
 * (px, rotation removed) and scale are the frame's, for the HOVER scene gate
 * (NaN without a frame: the take-off standby). */
static void holdUpdate(bool viewOk, float yawAngle, float wFwd, float wLat, float dt, float residual, float dxRes,
                       float scale)
{
  flightRecord_t ctx;
  bool haveCtx = flightRecorderGetLatest(&ctx);
  viewOk = viewOk && camHoldYawOk(yawAngle, dt, haveCtx && camHoldTurnActive(ctx.mvState));
  /* The camera height observer's gates, in autonomy HOVER after 2 s of it:
   * a person crossing or receding in front of the camera is not the drone's
   * drift (2026-10-13 flight 1). A refused frame is a lost view (coast). */
  const bool sceneOk = camHoldSceneOk(&holdScene, haveCtx && ctx.autoState == AUTO_HOVER, dt, residual, dxRes, scale);
  /* The interactive hover's moving reference (all 0 without a motion: the
   * hold then computes what it always did). */
  autonomyHoldRef_t ref;
  autonomyGetHoldRef(&ref);
  /* The gain scale (docs/autonomy-a2b.md 20): from the observer's tracked G in an autonomy sortie's TAKEOFF and
   * HOVER until its first MOVE, slewed at 2 /s; 1 in LAND, in a manual flight, without the recorder, or with kmax
   * 1. Once a MOVE ran, s moves at 0.75 /s in HOVER; during a MOVE it follows the scene distance the move takes the
   * drone to (camHoldGainMove; LAND keeps 2 /s to 1: 21.3). */
  holdGainTgt = haveCtx ? camHoldGainTarget(&holdGain, ctx.autoState, ctx.mvState, ctx.cvFlags, ctx.cvGain, holdKmax)
                        : 1.0f;
  /* the move fixes (mvfix, auto_motion.h "move fixes"; 0 outside a sortie): GAIN the measured progress and, above 100 %,
   * no tracking test for the first CAM_HOLD_GAIN_GRACE_S of a move; BRAKE the braking lean by the measured speed */
  const uint8_t mvFix = autonomyMvFixBits();
  const bool fixGain = (mvFix & AUTO_MVFIX_GAIN) != 0u;
  camHoldGainMoveFix(&holdGain, fixGain,
                     fixGain && autonomyMvSpeedPct() > AUTO_MV_SPEED_DEFAULT_PCT ? CAM_HOLD_GAIN_GRACE_S : 0.0f, dt);
  if (haveCtx) {   /* during a MOVE: the scene distance the reference is taking the drone to (21.3) */
    holdGainTgt = camHoldGainMove(&holdGain, holdGainTgt, holdGainS, ctx.autoState, ctx.mvState, ref.dLowM,
                                  ref.refFwd, hold.epoch == ref.epoch ? hold.pFwd : 0.0f);
  }
  holdGainS = camHoldGainSlew(holdGainS, holdGainTgt, dt,
                              haveCtx ? camHoldGainRate(&holdGain, ctx.autoState) : CAM_HOLD_GAIN_SLEW_PER_S);
#ifdef ESPFLY_NATIVE_VGA
  const float latTarget = haveCtx ? camHoldStartupLatTarget(ctx.autoState, ref.epoch, ctx.cvFlags, ctx.cvGain, holdScene.hoverS) : 1.0f;
  /* Return immediately to the original gains for any motion, LAND or unknown/near
   * scene. Increasing the startup feedback is gradual; the integral's lean is
   * preserved by camHoldStep across either transition. */
  holdLatGainS = latTarget > 1.0f ? camHoldGainSlew(holdLatGainS, latTarget, dt, 1.0f) : 1.0f;
#else
  holdLatGainS = 1.0f;
#endif
  camHoldInput_t hin = {
    .enabled = holdEnabled,
    .sticksCentred = haveCtx && ctx.rollDes == 0 && ctx.pitchDes == 0,
    .airborne = haveCtx && ctx.thrust > CAM_HOLD_AIRBORNE_THRUST && !autonomyHoldBlocked(),   /* LAND contact */
    .viewOk = viewOk && sceneOk,
    .wFwd = wFwd,
    .wLat = wLat,
    .dt = dt,
    .refEpoch = ref.epoch,
    .refSnapEpoch = ref.snapEpoch,
    .refLat = ref.refLat,
    .refFwd = ref.refFwd,
    .rateLat = ref.rateLat,
    .rateFwd = ref.rateFwd,
    .ffRoll = ref.ffRoll,
    .ffPitch = ref.ffPitch,
    /* no forward hold in the take-off climb (camera_hold.h fwdOff, docs/autonomy-a2b.md 19.5): it starts at the
     * first HOVER frame from fwdF 0 and iFwd 0 */
    .fwdOff = haveCtx && ctx.autoState == AUTO_TAKEOFF,
    .gainScale = holdGainS,
    .ffBrakeScale = (mvFix & AUTO_MVFIX_BRAKE) != 0u,
    .latGainScale = holdLatGainS,
    /* the lateral position gain on a MOVE's frames (mvlatk, camera_hold.h latGain; 1 at 100: the hold as before) */
    .latGain = haveCtx && holdMoveLatPct > 100u && camHoldMoveActive(ctx.mvState) ? (float)holdMoveLatPct * 0.01f
                                                                                   : 1.0f,
#ifdef ESPFLY_NATIVE_VGA
    .earlyTakeoffLat = haveCtx && ctx.autoState == AUTO_TAKEOFF && ctx.vzPhase == VZ_CLIMB,
#endif
  };
  /* fast moves (mvspeed > 100, camera_hold.h camHoldMoveLimits): the feedforward above the feedback's limit, a faster
   * slew; 0 at 100 (the hold as before) */
  camHoldMoveLimits(autonomyMvSpeedPct(), &hin.moveLimDeg, &hin.moveSlewDegS);
  bool engaged = camHoldStep(&hold, &hin);
  cameraFlowHoldSlewV = camHoldSlewOf(&hold, &hin);   /* controller_pid.c's apply slew */
  cameraFlowHoldMovingV = camHoldMoving(&hold, dt) ? 1u : 0u;   /* a new MOVE waits for a still drone (mvfix STILL) */
  /* the hold at its limit: HOVER outside any motion, its output applied (engaged or coasting on what it learned) */
  const bool limQuiet = haveCtx && ctx.autoState == AUTO_HOVER && !camHoldMotionActive(ctx.mvState);
  cameraFlowHoldLimitBitsV = camHoldLimitStep(&holdLim, hold.roll, hold.pitch, engaged && limQuiet,
                                             (uint32_t)(esp_timer_get_time() / 1000));
  portENTER_CRITICAL(&holdMux);
  holdRoll = hold.roll;
  holdPitch = hold.pitch;
  holdEngaged = engaged;
  holdCoasting = hold.coasting;
  holdTimeUs = esp_timer_get_time();
  holdProgLat = hold.pLat;
  holdProgFwd = hold.pFwd;
  holdEpoch = hold.epoch;
  portEXIT_CRITICAL(&holdMux);
}

/* Camera task, colour (the split): the photo's pixels are already in the requester's slot (the driver's copy task
 * wrote them, cam_split_core.h); for the armed frame itself (its buffer and capture time) only the meta is filled
 * here, and a newer frame than the captured one re-arms the request. A plain load on every other frame. */
static void snapshotFillSplit(const uint8_t *frame, uint32_t frameSeq, uint64_t timeUs, float texture, uint8_t lum)
{
  if (camSplitFlowFrame(&camSplitSnap, frame, timeUs) != CAM_SPLIT_FLOW_FILL) {
    return;
  }
  snapMeta->seq = frameSeq;
  snapMeta->frameTimeUs = timeUs + gyroOffsetUs;
  snapMeta->texture = texture;
  snapMeta->lum = lum;
  snapMeta->pixFmt = CAMERA_FLOW_PIXFMT_YUYV;
  snapMeta->bytes = CAMERA_FLOW_SNAPSHOT_MAX_BYTES;
  camSplitFilled(&camSplitSnap);
}

/* Camera task: serve a pending snapshot request with this frame (one memcpy;
 * a plain load when nothing is requested). frame: the camera's frame as
 * captured (grey); in colour the split's meta only (snapshotFillSplit). */
static void snapshotFill(const uint8_t *frame, uint32_t frameSeq, uint64_t timeUs, float texture, uint8_t lum)
{
  if (photoColour) {
    snapshotFillSplit(frame, frameSeq, timeUs, texture, lum);
    return;
  }
  if (__atomic_load_n(&snapState, __ATOMIC_RELAXED) != SNAP_REQUESTED) {
    return;
  }
  uint32_t expect = SNAP_REQUESTED;
  if (!__atomic_compare_exchange_n(&snapState, &expect, SNAP_COPYING, false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
    return;
  }
  const uint32_t bytes = CAMERA_FLOW_SNAPSHOT_BYTES;
  memcpy(snapBuf, frame, bytes);
  snapMeta->seq = frameSeq;
  snapMeta->frameTimeUs = timeUs + gyroOffsetUs;
  snapMeta->texture = texture;
  snapMeta->lum = lum;
  snapMeta->pixFmt = CAMERA_FLOW_PIXFMT_GRAY8;
  snapMeta->bytes = bytes;
  __atomic_store_n(&snapState, SNAP_FILLED, __ATOMIC_RELEASE);
}

/* A snapshot request is outstanding (the ground standby waits for it). */
static bool snapPending(void)
{
  return photoColour ? camSplitPending(&camSplitSnap) : __atomic_load_n(&snapState, __ATOMIC_ACQUIRE) != SNAP_IDLE;
}

/* Sleep through the current take-off window. Returns false (and the caller
 * keeps streaming) if the sensor did not accept the standby write. *seen gets
 * the ground-start count read just before the window was seen closed, so a
 * ground start during the wake writes is still handled by the caller.
 * asleep: the ground standby handed this ground start over with the sensor
 * already asleep (no second standby write, no wake in between). */
static bool standbyThroughTakeoff(uint32_t *seen, bool asleep)
{
  if (!asleep && !sensorStandby(true)) {
    standbyFails++;
    return false;
  }
  inStandby = true;
  state = CAMERA_FLOW_STANDBY;
  standbyCount++;
  const uint64_t t0 = esp_timer_get_time();
  uint64_t tPrev = t0;
  for (;;) {
    const uint32_t n = stabilizerTakeoffCount();   /* before the phase check */
    if (!cameraTakeoffPhase()) {
      *seen = n;
      break;
    }
    vTaskDelay(M2T(10));
    /* No view while asleep, as when texture is lost: the hold's airborne
     * clock keeps running (so it engages on the first good frames after the
     * wake instead of 1 s later) and it coasts on what it has learned. */
    const uint64_t tNow = esp_timer_get_time();
    holdUpdate(false, 0.0f, 0.0f, 0.0f, (tNow - tPrev) * 1e-6f, NAN, NAN, NAN);
    tPrev = tNow;
  }
  standbyFails += sensorWake();
  standbyMs += (uint32_t)((esp_timer_get_time() - t0) / 1000);
  inStandby = false;
  markWake();
  state = CAMERA_FLOW_RUNNING;
  return true;
}

/* The ground standby's inputs (cam_gnd.h). takeoffSeen: the ground starts the
 * camera task has handled. */
static void groundInputs(camGndIn_t *in, uint32_t takeoffSeen)
{
  memset(in, 0, sizeof(*in));
  /* onGround before the count: a ground start between the two reads is then
   * seen as a ground start (the stabilizer clears onGround in the same loop) */
  const bool onGround = stabilizerOnGround();
  const uint32_t takeoffs = stabilizerTakeoffCount();
  in->nowMs = msNow();
  in->enabled = gndEnabled;
  in->supported = standbySupported();
  in->running = state == CAMERA_FLOW_RUNNING;
  in->autonomyActive = autonomyActive();
  in->onGround = onGround;
  in->takeoffWindow = cameraTakeoffWindow();
  in->snapPending = snapPending();
  in->benchHold = gndBenchHold || camVzGlueBenchAir() != 0;
  in->clockPending = camClockPending(&clk);
  in->groundStart = takeoffs != takeoffSeen;
  in->takeoffStandby = standbyEnabled && standbySupported() && cameraTakeoffPhase();
  in->startMs = camStartMs;
}

/* Asleep on the ground until a wake reason (cam_gnd.h). Returns CAM_GND_STAY
 * (the standby write failed: still streaming, retried after a back-off),
 * CAM_GND_WAKE (awake again: the caller restarts the flow fit) or
 * CAM_GND_HANDOVER (a ground start the take-off standby takes over: the sensor
 * stays asleep, the caller's take-off branch runs next). The drift hold keeps
 * its clocks and what it learned, as in the take-off standby. */
static camGndAction_t groundStandby(uint32_t takeoffSeen)
{
  if (!sensorStandby(true)) {
    camGndSleepResult(&gnd, msNow(), false);
    return CAM_GND_STAY;
  }
  camGndSleepResult(&gnd, msNow(), true);
  gndAsleep = true;
  state = CAMERA_FLOW_STANDBY;
  markWake();
  uint64_t tPrev = esp_timer_get_time();
  camGndAction_t act;
  for (;;) {
    vTaskDelay(M2T(CAM_GND_POLL_MS));
    camGndIn_t in;
    groundInputs(&in, takeoffSeen);
    act = camGndAsleepStep(&gnd, &in);
    if (act != CAM_GND_STAY) {
      break;
    }
    const uint64_t tNow = esp_timer_get_time();
    if (tNow - tPrev >= CAM_GND_HOLD_UPDATE_MS * 1000u) {
      holdUpdate(false, 0.0f, 0.0f, 0.0f, (tNow - tPrev) * 1e-6f, NAN, NAN, NAN);
      tPrev = tNow;
    }
  }
  if (act == CAM_GND_HANDOVER) {
    camGndWoke(&gnd, msNow(), true);   /* asleep on: the take-off standby's time from here */
    gndAsleep = false;
    return act;
  }
  const uint32_t failed = sensorWake();
  for (uint32_t i = 0; i < failed; i++) {
    camGndWakeFailed(&gnd);
  }
  camGndWoke(&gnd, msNow(), false);
  gndAsleep = false;
  markWake();
  state = CAMERA_FLOW_RUNNING;
  return CAM_GND_WAKE;
}

/* The flow loop is too slow for 13.33 MHz (cam_clock.h): 20 MHz for the rest
 * of this boot. On the ground only (camClockFallback()); the sensor sleeps
 * while its clock changes (the PLL multiplies the new XCLK), and the frames of
 * the switch are skipped by the caller's fresh-frame time. */
static void clockFallback(void)
{
  DEBUG_PRINTW("camera: flow loop too slow at 13.33 MHz (proc %.0f us after %u frames, window %.1f fps): XCLK 20 MHz "
               "for this boot", (double)clk.procAvgUs, (unsigned)clk.clockFrames, (double)clk.winFps);
  sensor_t *sensor = esp_camera_sensor_get();
  const bool asleep = sensorStandby(true);
  bool switched = false;
  if (sensor && sensor->set_xclk && sensor->set_xclk(sensor, LEDC_TIMER_2, (int)(CAM_CLOCK_FAST_HZ / 1000000u)) == 0) {
    xclkHz = CAM_CLOCK_FAST_HZ;
    gyroOffsetUs = camClockGyroOffsetUs(CAM_CLOCK_FAST_HZ);
    camVzGlueSetGyroOffsetUs(0);
    switched = true;
  }
  if (asleep) {
    standbyFails += sensorWake();
  }
  if (!switched) {
    /* still 13.33 MHz and too slow for it: no rate is published (camClockTooSlow()), so a FLIGHT ARM is refused
     * CAMERA for this boot ('tkoff camclk 20' and a reboot fly at 20 MHz) */
    DEBUG_PRINTW("camera: XCLK switch failed: no frame rate published at 13.33 MHz with this loop");
  }
  camClockTrackFellBack(&clk, xclkHz);   /* once, whatever the outcome */
  markWake();
}

static void cameraFlowTask(void *param)
{
  systemWaitStart();
  vTaskDelay(M2T(CAMERA_START_DELAY_MS));
  waitOutTakeoff();

  state = CAMERA_FLOW_STARTING;
  esp_err_t err = cameraStart();
  if (err != ESP_OK) {
    state = CAMERA_FLOW_FAILED;
    DEBUG_PRINTW("camera init failed (%s); flight unaffected", esp_err_to_name(err));
    vTaskDelete(NULL);
    return;
  }

  /* PSRAM: internal RAM is needed by Wi-Fi and the camera DMA buffer. */
  ofState_t *of = heap_caps_malloc(sizeof(ofState_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!of) {
    of = heap_caps_malloc(sizeof(ofState_t), MALLOC_CAP_8BIT);
  }
  if (!of) {
    state = CAMERA_FLOW_FAILED;
    DEBUG_PRINTW("camera flow: no memory");
    esp_camera_deinit();
    vTaskDelete(NULL);
    return;
  }
  ofInit(of);
  stageAcc = heap_caps_calloc(1, sizeof(stageAcc_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);   /* NULL: no 'flowfit prof' */
  camStartMs = msNow();
  camGndRestarted(&gnd, camStartMs);
  camClockTrackInit(&clk, xclkHz);
  markWake();
  state = CAMERA_FLOW_RUNNING;
  DEBUG_PRINTI("camera running, sensor PID 0x%x", sensorPid);

  camera_fb_t *heldPrev = NULL;  /* pinned native QVGA frame, released after the next fit */
  uint64_t prevT = 0;
  float prevLum = 0.0f;   /* the previous frame's mean brightness (0: none, after every restart of the fit) */
  uint32_t seq = 0;
  /* 0, not the current count: a ground start during camera start-up is still
   * handled while its window is open (a closed window just syncs the count). */
  uint32_t takeoffSeen = 0;
  uint64_t freshAfterUs = 0;   /* frames captured before this are stale */
  bool handedOver = false;     /* asleep: the ground standby handed a ground start to the take-off standby */
  while (1) {
    /* Once per ground start: power the sensor down until the window ends.
     * The frame wait below returns within one frame (~60 ms) of the start. */
    const uint32_t takeoffs = stabilizerTakeoffCount();
    if (takeoffs != takeoffSeen) {
      takeoffSeen = takeoffs;
      const bool asleep = handedOver;
      handedOver = false;
      if (standbyEnabled && standbySupported() && cameraTakeoffPhase() &&
          standbyThroughTakeoff(&takeoffSeen, asleep)) {
        /* Restart the flow fit; frames queued before or during the standby
         * are skipped by their capture time. */
        ofInit(of);
        prevT = 0;
        freshAfterUs = esp_timer_get_time();
        continue;
      }
      if (asleep) {
        /* the window closed between the hand-over and here: wake now */
        const uint32_t failed = sensorWake();
        for (uint32_t i = 0; i < failed; i++) {
          camGndWakeFailed(&gnd);
        }
        markWake();
        state = CAMERA_FLOW_RUNNING;
        ofInit(of);
        prevT = 0;
        freshAfterUs = esp_timer_get_time();
        continue;
      }
    }
    /* Ground standby (cam_gnd.h): asleep while autonomy is idle on the ground. */
    if (gndEnabled && state == CAMERA_FLOW_RUNNING) {
      camGndIn_t gin;
      groundInputs(&gin, takeoffSeen);
      if (camGndAwakeStep(&gnd, &gin) == CAM_GND_SLEEP) {
        const camGndAction_t act = groundStandby(takeoffSeen);
        if (act == CAM_GND_HANDOVER) {
          handedOver = true;
          continue;
        }
        if (act == CAM_GND_WAKE) {
          ofInit(of);
          prevT = 0;
          freshAfterUs = esp_timer_get_time();
          continue;
        }
      }
    }
    bool bad;
    camera_fb_t *fb = frameGet(&bad);
    if (bad) {
      /* The driver's memory is corrupt: stop using the camera for this boot
       * (no deinit, which would walk the same corrupt state). The hold goes
       * stale and the controller fades its correction out within 300 ms. */
      badFrames++;
      state = CAMERA_FLOW_FAILED;
      portENTER_CRITICAL(&holdMux);
      holdEngaged = false;
      holdCoasting = false;
      portEXIT_CRITICAL(&holdMux);
      DEBUG_PRINTW("camera: invalid frame from driver after %u frames, camera stopped", (unsigned)framesTotal);
      vTaskDelete(NULL);
      return;
    }
    if (!fb) {
      /* No frame for 4 s (the driver's timeout): restart the camera. The
       * sensor has no power-down pin, so this re-runs its SCCB reset. */
      nullFrames++;
      state = CAMERA_FLOW_RESTARTING;
      DEBUG_PRINTW("camera stalled after %u frames, restarting", (unsigned)framesTotal);
      if (heldPrev) { esp_camera_fb_return(heldPrev); heldPrev = NULL; }
      esp_camera_deinit();
      vTaskDelay(M2T(200));
      waitOutTakeoff();
      while ((err = cameraStart()) != ESP_OK) {
        DEBUG_PRINTW("camera restart failed (%s)", esp_err_to_name(err));
        vTaskDelay(M2T(2000));
        waitOutTakeoff();
      }
      restarts++;
      camHoldReset(&hold);
      camHoldSceneReset(&holdScene);
      holdSceneApplyRes();
      ofInit(of);
      prevT = 0;
      camStartMs = msNow();
      camGndRestarted(&gnd, camStartMs);
      camClockTrackRestart(&clk);   /* the same loop: the clock decision is kept */
      markWake();
      state = CAMERA_FLOW_RUNNING;
      continue;
    }

    const uint64_t tGotUs = esp_timer_get_time();   /* the frame's processing starts */
    const uint32_t cycGot = esp_cpu_get_cycle_count();   /* the same, for the stage times */
    const uint64_t t = (uint64_t)fb->timestamp.tv_sec * 1000000ULL + (uint64_t)fb->timestamp.tv_usec;
    if (t < freshAfterUs) {
      esp_camera_fb_return(fb);
      continue;
    }
    ofResult_t res;
    /* The bisect toggle (bench_bypass.h): 14bcfc6's camera loop, the plain fit without the exposure gate, from this
     * frame; the NVS / console mode (fitMode) is untouched. */
    const bool flowBench = benchBypassOn(BENCH_BYPASS_FLOW);
    of->mode = flowBench ? OF_FIT_PLAIN : fitMode;
    if (prevT == 0) {
      prevLum = 0.0f;   /* the fit was restarted (ofInit): no previous frame */
    }
    /* The fit's grey frame: the sensor's Y8, or in colour the Y bytes the driver's copy split out (cam_split.c),
     * in the same buffer: the same frame, code and cost. */
    const uint8_t *luma = fb->buf;
    bool ok = ofProcessPinned(of, luma, NULL, &res);
    const uint32_t cycFit = esp_cpu_get_cycle_count();
    uint32_t lumSum = 0;
    for (int i = 0; i < OF_WIDTH * OF_HEIGHT; i += 256) {
      lumSum += luma[i];
    }
    bool expStep = false;
    if (!flowBench) {
      const float lumNow = (float)lumSum / (float)(OF_WIDTH * OF_HEIGHT / 256);
      expStep = ok && ofExposureStepTooLarge(&res, lumNow, prevLum);
      prevLum = lumNow;
    } else {
      prevLum = 0.0f;   /* toggled off again, the gate starts without a previous frame */
    }
    snapshotFill(fb->buf, seq + 1, t, res.texture, (uint8_t)(lumSum / (OF_WIDTH * OF_HEIGHT / 256)));
    if (heldPrev) esp_camera_fb_return(heldPrev);
    heldPrev = fb;
    const uint32_t cycSnap = esp_cpu_get_cycle_count();

    cameraFlowSample_t s;
    memset(&s, 0, sizeof(s));
    float a0[3], a1[3];
    const uint32_t offUs = gyroOffsetUs;
    bool gyroOk = prevT != 0 && gyroAngleAt(prevT + offUs, a0) && gyroAngleAt(t + offUs, a1);
    s.seq = ++seq;
    /* Publish the physical capture-window centre, not DMA start or receipt time.
     * The same modelled offset already selects the gyro integration window below.
     * Downstream observer offset is zero so it is applied exactly once. */
    s.frameTimeUs = t + offUs;
    s.dtUs = prevT ? (uint32_t)(t - prevT) : 0;
    /* Native QVGA fit; legacy 160-scale output units keep downstream control gains and gates unchanged. */
    s.dx = res.tx / CAMERA_NATIVE_SCALE;
    s.dy = res.ty / CAMERA_NATIVE_SCALE;
    s.scale = res.scale;
    s.rot = res.rot;
    if (gyroOk) {
      for (int k = 0; k < 3; k++) {
        s.gyroAngle[k] = a1[k] - a0[k];
      }
    }
    s.texture = res.texture * CAMERA_NATIVE_SCALE * CAMERA_NATIVE_SCALE;
    s.residual = res.residual;
    s.lum = (uint8_t)(lumSum / (OF_WIDTH * OF_HEIGHT / 256));
    s.ok = ok && gyroOk && !expStep;
    if (!flowBench) {   /* bypassed: gainE3 and satPct stay 0 (memset) */
      const float g = (res.gain - 1.0f) * 1000.0f;
      s.gainE3 = (int8_t)(g > 127.0f ? 127 : (g < -127.0f ? -127 : lrintf(g)));   /* a NaN gain reads -127 */
      const float sp = res.satFrac * 100.0f;
      s.satPct = (uint8_t)(sp > 100.0f ? 100 : (sp > 0.0f ? lrintf(sp) : 0)) | (expStep ? CAMERA_FLOW_SAT_EXPGATE : 0);
      if (expStep) {
        expGated++;
      } else if (ok && res.gain != 1.0f) {
        expCorrected++;
      }
    }
    framesTotal++;
    if (framesSinceWake < UINT32_MAX) {
      framesSinceWake++;
    }
    lastFrameUs = s.frameTimeUs;
    if (framesTotal % 16 == 1) {
      updateStackStats();
    }

    cameraFlowMotionLagUsV = camCadenceMotionLagUs((uint64_t)esp_timer_get_time(), s.frameTimeUs, s.dtUs);
    /* Drift hold: relative velocities -> attitude corrections. */
    {
      const float dt = s.dtUs * 1e-6f;
      const bool viewOk = s.ok && s.texture >= CAM_HOLD_MIN_TEXTURE && s.residual < CAM_HOLD_MAX_RESIDUAL &&
                          dt > 0.0f;   /* and the yaw rate: holdUpdate */
      const float wFwd = dt > 0.0f ? s.scale / dt : 0.0f;
      const float dxRes = s.dx - (FLOW_KX_GX * s.gyroAngle[0] + FLOW_KX_GY * s.gyroAngle[1] + FLOW_KX_GZ * s.gyroAngle[2]);
      const float wLat = dt > 0.0f ? -dxRes / FLOW_KX_GZ / dt : 0.0f;
      holdUpdate(viewOk, s.gyroAngle[2], wFwd, wLat, dt, s.residual, dxRes, s.scale);
    }

    portENTER_CRITICAL(&sampleMux);
    latest = s;
    haveLatest = true;
    portEXIT_CRITICAL(&sampleMux);
    prevT = t;

    /* The flow loop's own cost and regularity (cam_clock.h): processing time,
     * skipped frames, the processed rate, the published rate; on the ground a
     * loop too slow for 13.33 MHz falls back to 20 MHz. */
    const uint32_t busyUs = (uint32_t)(esp_timer_get_time() - tGotUs);
    if (of->stageCycles[OF_STAGE_COARSE] != 0) {   /* a frame with a fit */
      const uint32_t cycEnd = esp_cpu_get_cycle_count();
      const uint32_t cyc[CAMERA_FLOW_STAGES] = {
        of->stageCycles[OF_STAGE_DOWNSAMPLE], of->stageCycles[OF_STAGE_COARSE], of->stageCycles[OF_STAGE_FINE],
        of->stageCycles[OF_STAGE_COPY], cycSnap - cycFit, cycEnd - cycSnap, cycEnd - cycGot,
      };
      stageAdd(cyc);
    }
    camClockTrackFrame(&clk, t, s.dtUs, busyUs);
    /* The thermal lead (NVS camlead, default 0): in IDLE the rate is
     * published only that long after a ground wake, so a FLIGHT ARM's G4
     * refuses CAMERA until then. */
    fps = camClockPublishFps(&clk, framesSinceWake, camGndFpsVisible(&gnd, msNow(), gndLeadMs, autonomyActive()));
    if (xclkHz == CAM_CLOCK_SLOW_HZ && !clk.fellBack) {
      const camClockIn_t cin = camClockTrackIn(&clk, state == CAMERA_FLOW_RUNNING, autonomyActive(),
                                               stabilizerOnGround(), cameraTakeoffWindow());
      if (camClockFallback(&cin)) {
        clockFallback();
        ofInit(of);
        prevT = 0;
        freshAfterUs = esp_timer_get_time();
      }
    }
  }
}

void cameraFlowInit(void)
{
  static bool isInit;
  if (isInit) {
    return;
  }
  bool on = false;
  if (camHoldStoreLoad(&on) == ESP_OK) {
    holdEnabled = on;
  }
  bool sb = true;
  if (camStandbyStoreLoad(&sb) == ESP_OK) {
    standbyEnabled = sb;
  }
  uint16_t sbMs = 0;
  if (camStandbyMsStoreLoad(&sbMs) == ESP_OK && sbMs >= CAMERA_STANDBY_MS_MIN && sbMs <= CAMERA_STANDBY_MS_MAX) {
    standbyLenMs = sbMs;
  }
  if (camStandbyColdMsStoreLoad(&sbMs) == ESP_OK && sbMs >= CAMERA_STANDBY_MS_MIN && sbMs <= CAMERA_STANDBY_MS_MAX) {
    standbyColdMs = sbMs;
  }
  bool slowClock = true;
  if (camClockStoreLoad(&slowClock) == ESP_OK) {
    xclkHz = camClockXclkHz(slowClock ? 1u : 0u);
  }
  gyroOffsetUs = camClockGyroOffsetUs(xclkHz);
  camVzGlueSetGyroOffsetUs(0);   /* the observer was set up before (stabilizerInit) */
  bool gndOn = true;
  if (camGroundStoreLoad(&gndOn) == ESP_OK) {
    gndEnabled = gndOn;
  }
  if (camGroundMsStoreLoad(&sbMs) == ESP_OK && sbMs >= CAM_GND_HOLD_MS_MIN && sbMs <= CAM_GND_HOLD_MS_MAX) {
    gndHoldMs = sbMs;
  }
  if (camLeadMsStoreLoad(&sbMs) == ESP_OK && sbMs <= CAM_GND_LEAD_MS_MAX) {
    gndLeadMs = sbMs;
  }
  uint8_t fm = OF_FIT_ROBUST;
  if (flowFitStoreLoad(&fm) == ESP_OK && fm <= OF_FIT_PLAIN) {
    fitMode = fm;
  }
  float km = CAM_HOLD_GAIN_KMAX_DEFAULT;
  if (camHoldKmaxStoreLoad(&km) == ESP_OK) {   /* the store validates 1..3 */
    holdKmax = km;
  }
  camHoldGainReset(&holdGain);
  camGndInit(&gnd, msNow());
  camHoldReset(&hold);
  camHoldSceneReset(&holdScene);
  holdSceneApplyRes();

  /* Off unless explicitly enabled: on 2026-10-10 the camera driver crashed
   * in flight (garbage frame pointer in esp_camera_fb_get), rebooting the
   * drone. With the camera off its driver is never started. */
  bool camOn = false;
  if (camEnableStoreLoad(&camOn) != ESP_OK || !camOn) {
    state = CAMERA_FLOW_OFF;
    isInit = true;
    return;
  }
  /* The photos' pixel format (NVS photocol, auto_glue_core.h AUTO_SET_PHOTOCOL, default 0: grey), read here once:
   * 1 runs the camera in the colour split for the boot (cameraStart: its boot check, else grey). No memory of its
   * own: the frame buffers keep grey's size. */
  const autoSettingInfo_t *pc = autoSettingInfo(AUTO_SET_PHOTOCOL);
  int32_t col = pc->def;
  if (autoSettingStoreLoad(pc->name, (uint8_t)pc->type, &col) != ESP_OK ||
      !autoSettingInRange(AUTO_SET_PHOTOCOL, col)) {
    col = pc->def;
  }
  splitWanted = col != 0;
  if (gyroRing == NULL) {
    const size_t bytes = sizeof(gyroIntegral_t) * GYRO_RING_LEN;
    gyroIntegral_t *r = heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (r == NULL) {
      /* Without it no frame has its rotation removed (never ok): the drift
       * hold could not engage. */
      DEBUG_PRINTW("no PSRAM for the gyro ring (%u B): camera not started", (unsigned)bytes);
      state = CAMERA_FLOW_FAILED;
      return;
    }
    __atomic_store_n(&gyroRing, r, __ATOMIC_RELEASE);
  }
  if (xTaskCreatePinnedToCore(cameraFlowTask, CAMERA_FLOW_TASK_NAME, CAMERA_FLOW_TASK_STACK, NULL,
                              CAMERA_FLOW_TASK_PRI, NULL, 1) != pdPASS) {
    state = CAMERA_FLOW_FAILED;
    return;
  }
  isInit = true;
}

bool cameraFlowSetFitMode(uint8_t mode)
{
  if (mode > OF_FIT_PLAIN) {
    return false;
  }
  fitMode = mode;
  return true;
}

uint8_t cameraFlowFitMode(void)
{
  return fitMode;
}

void cameraFlowExposureStats(uint32_t *gated, uint32_t *corrected)
{
  *gated = expGated;
  *corrected = expCorrected;
}

/* IRAM: polled by the camvz task every 10 ms (cam_vz_glue.c) and by the recorder. */
bool IRAM_ATTR cameraFlowGetLatest(cameraFlowSample_t *out)
{
  bool ok;
  portENTER_CRITICAL(&sampleMux);
  ok = haveLatest;
  *out = latest;
  portEXIT_CRITICAL(&sampleMux);
  return ok;
}

cameraFlowState_t IRAM_ATTR cameraFlowState(void)
{
  return state;
}

const char *cameraFlowStateName(void)
{
  switch (state) {
    case CAMERA_FLOW_STARTING: return "starting";
    case CAMERA_FLOW_RUNNING:  return "running";
    case CAMERA_FLOW_RESTARTING: return "restarting";
    case CAMERA_FLOW_FAILED:   return "failed";
    case CAMERA_FLOW_STANDBY:  return "standby";
    default:                   return "off";
  }
}

float IRAM_ATTR cameraFlowFps(void)
{
  return fps;
}

int cameraFlowSensorPid(void)
{
  return sensorPid;
}

void IRAM_ATTR cameraFlowGetHold(float *roll, float *pitch, bool *engaged, uint64_t *timeUs)
{
  portENTER_CRITICAL(&holdMux);
  *roll = holdRoll;
  *pitch = holdPitch;
  *engaged = holdEngaged;
  *timeUs = holdTimeUs;
  portEXIT_CRITICAL(&holdMux);
}

/* Flash: only the 100 Hz recorder sample (holdFlags bit 3, flash code itself) reads it. */
bool cameraFlowHoldSceneGated(void)
{
  return __atomic_load_n(&holdScene.lastRejected, __ATOMIC_RELAXED);
}

void cameraFlowHoldSceneStats(uint32_t *rejRes, uint32_t *rejDx, uint32_t *rejExp)
{
  /* written by the camera task only; a torn read of a counter is harmless */
  *rejRes = holdScene.rejRes;
  *rejDx = holdScene.rejDx;
  *rejExp = holdScene.rejExp;
}

bool IRAM_ATTR cameraFlowHoldCoasting(void)
{
  bool c;
  portENTER_CRITICAL(&holdMux);
  c = holdCoasting;
  portEXIT_CRITICAL(&holdMux);
  return c;
}

void IRAM_ATTR cameraFlowGetHoldProgress(float *pLat, float *pFwd, uint32_t *epoch)
{
  portENTER_CRITICAL(&holdMux);
  *pLat = holdProgLat;
  *pFwd = holdProgFwd;
  *epoch = holdEpoch;
  portEXIT_CRITICAL(&holdMux);
}

void cameraFlowSetHoldEnabled(bool on)
{
  holdEnabled = on;
}

bool IRAM_ATTR cameraFlowHoldEnabled(void)
{
  return holdEnabled;
}

bool cameraFlowSetHoldKmax(float kmax)
{
  if (!(kmax >= 1.0f && kmax <= CAM_HOLD_GAIN_SCALE_MAX)) {
    return false;
  }
  holdKmax = kmax;
  return true;
}

float cameraFlowHoldKmax(void)
{
  return holdKmax;
}

void cameraFlowSetSceneResidual(uint8_t x10)
{
  holdSceneResX10 = x10;
  holdSceneApplyRes();   /* (a float store, atomic; the camera task reads it on its next frame) */
}

uint8_t cameraFlowSceneResidual(void)
{
  return holdSceneResX10;
}

void cameraFlowSetMoveLatGain(uint16_t pct)
{
  const uint16_t hi = (uint16_t)(CAM_HOLD_LAT_GAIN_MAX * 100.0f + 0.5f);
  holdMoveLatPct = pct < 100u ? 100u : (pct > hi ? hi : pct);   /* (a 16-bit store, atomic; read per frame) */
}

uint16_t cameraFlowMoveLatGain(void)
{
  return holdMoveLatPct < 100u ? 100u : holdMoveLatPct;
}

void cameraFlowHoldGainInfo(float *scale, float *target, float *gPxPerM, bool *translated)
{
  *scale = holdGainS;
  *target = holdGainTgt;
  *gPxPerM = holdGain.g;
  *translated = holdGain.translated;
}

void cameraFlowSetStandbyEnabled(bool on)
{
  standbyEnabled = on;
}

bool cameraFlowStandbyEnabled(void)
{
  return standbyEnabled;
}

void cameraFlowSetTakeoffStandbySkip(bool skip)
{
  standbySkip = skip;
}

bool cameraFlowSetStandbyMs(uint16_t ms, bool cold)
{
  if (ms < CAMERA_STANDBY_MS_MIN || ms > CAMERA_STANDBY_MS_MAX) {
    return false;
  }
  if (cold) {
    standbyColdMs = ms;
  } else {
    standbyLenMs = ms;
  }
  return true;
}

uint16_t cameraFlowStandbyMs(bool cold)
{
  return cold ? standbyColdMs : standbyLenMs;
}

bool cameraFlowStandbyArmed(void)
{
  const cameraFlowState_t st = state;
  return standbyEnabled && standbySupported() && (st == CAMERA_FLOW_RUNNING || st == CAMERA_FLOW_STANDBY);
}

bool cameraFlowInStandby(void)
{
  return inStandby;
}

void cameraFlowStandbyStats(uint32_t *count, uint32_t *fails, uint32_t *totalMs)
{
  *count = standbyCount;
  *fails = standbyFails;
  *totalMs = standbyMs;
}

void cameraFlowHealth(uint32_t *invalidFrames, uint32_t *camTaskFree, uint32_t *flowTaskFree)
{
  *invalidFrames = badFrames;
  *camTaskFree = camTaskStackFree;
  *flowTaskFree = flowTaskStackFree;
}

void cameraFlowStats(uint32_t *frames, uint32_t *timeouts, uint32_t *restartCount, uint32_t *ageMs)
{
  *frames = framesTotal;
  *timeouts = nullFrames;
  *restartCount = restarts;
  uint64_t last = lastFrameUs, now = esp_timer_get_time();
  *ageMs = (last && now > last) ? (uint32_t)((now - last) / 1000) : 0;
}

uint32_t cameraFlowGyroOffsetUs(void)
{
  return 0;  /* Published VGA timestamps already include the capture-window offset. */
}

void cameraFlowClockStats(cameraFlowClockStats_t *st)
{
  st->xclkHz = xclkHz;
  st->fellBack = clk.fellBack;
  st->gyroOffsetUs = camClockGyroOffsetUs(xclkHz);
  st->procAvgUs = (uint32_t)(clk.procAvgUs + 0.5f);
  st->procMaxUs = clk.procMaxUs;
  st->skips = clk.skips;
  st->windowFps = clk.winValid ? clk.winFps : 0.0f;
  st->clockFrames = clk.clockFrames;
  st->pending = state == CAMERA_FLOW_RUNNING || state == CAMERA_FLOW_STANDBY ? camClockPending(&clk) : false;
  st->tooSlow = camClockTooSlow(&clk);
}

bool cameraFlowStageStats(cameraFlowStageStats_t *st)
{
  memset(st, 0, sizeof(*st));
  stageAcc_t *a = stageAcc;
  if (a == NULL) {
    return false;
  }
  uint64_t sum[CAMERA_FLOW_STAGES];
  uint32_t mn[CAMERA_FLOW_STAGES], mx[CAMERA_FLOW_STAGES];
  portENTER_CRITICAL(&stageMux);
  const uint32_t n = stageResetReq ? 0 : a->frames;
  memcpy(sum, a->sum, sizeof(sum));
  memcpy(mn, a->min, sizeof(mn));
  memcpy(mx, a->max, sizeof(mx));
  portEXIT_CRITICAL(&stageMux);
  if (n == 0) {
    return false;
  }
  const uint32_t mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;   /* fixed: no power management in this build */
  st->frames = n;
  for (int i = 0; i < CAMERA_FLOW_STAGES; i++) {
    st->avgUs[i] = (uint32_t)((sum[i] / n + mhz / 2) / mhz);
    st->minUs[i] = (mn[i] + mhz / 2) / mhz;
    st->maxUs[i] = (mx[i] + mhz / 2) / mhz;
  }
  return true;
}

void cameraFlowStageReset(void)
{
  stageResetReq = true;
}

void cameraFlowGroundWake(uint32_t holdMs)
{
  const uint32_t now = msNow();
  portENTER_CRITICAL(&gndMux);
  camGndWake(&gnd, now, holdMs);
  portEXIT_CRITICAL(&gndMux);
}

void cameraFlowGroundRelease(void)
{
  const uint32_t now = msNow();
  portENTER_CRITICAL(&gndMux);
  camGndRelease(&gnd, now);
  portEXIT_CRITICAL(&gndMux);
}

void cameraFlowGroundBenchHold(bool on)
{
  gndBenchHold = on;
}

bool cameraFlowGroundStandby(void)
{
  return gndAsleep;
}

bool cameraFlowAwakeFresh(uint32_t nFrames)
{
  return state == CAMERA_FLOW_RUNNING && framesSinceWake >= nFrames;
}

void cameraFlowSetGroundStandby(bool on)
{
  gndEnabled = on;
}

bool cameraFlowGroundStandbyEnabled(void)
{
  return gndEnabled;
}

bool cameraFlowSetGroundHoldMs(uint32_t ms)
{
  if (ms < CAM_GND_HOLD_MS_MIN || ms > CAM_GND_HOLD_MS_MAX) {
    return false;
  }
  gndHoldMs = (uint16_t)ms;
  return true;
}

uint32_t cameraFlowGroundHoldMs(void)
{
  return gndHoldMs;
}

bool cameraFlowSetGroundLeadMs(uint32_t ms)
{
  if (ms > CAM_GND_LEAD_MS_MAX) {
    return false;
  }
  gndLeadMs = (uint16_t)ms;
  return true;
}

uint32_t cameraFlowGroundLeadMs(void)
{
  return gndLeadMs;
}

void cameraFlowGroundStats(cameraFlowGroundStats_t *st)
{
  const uint32_t now = msNow();
  portENTER_CRITICAL(&gndMux);   /* against a hold change only; the counters are the camera task's */
  st->count = gnd.count;
  st->fails = gnd.fails;
  st->asleepMs = camGndAsleepTotalMs(&gnd, now);
  st->awakeMs = camGndAwakeMs(&gnd, now);
  st->holdLeftMs = camGndHoldActive(&gnd, now) ? gnd.holdUntilMs - now : 0u;
  portEXIT_CRITICAL(&gndMux);
  st->asleep = gndAsleep;
}

uint8_t cameraFlowPhotoPixFmt(void)
{
  return photoColour ? CAMERA_FLOW_PIXFMT_YUYV : CAMERA_FLOW_PIXFMT_GRAY8;
}

void cameraFlowSplitStats(cameraFlowSplitStats_t *st)
{
  st->split = splitStat;
  st->arm = camSplitSnap.arm;
  st->done = camSplitSnap.done;
  st->rearm = camSplitSnap.rearm;
  st->cancel = camSplitSnap.cancel;
  st->restart = camSplitSnap.restart;
  st->resets = camSplitSnap.resets;
}

const char *cameraFlowSplitName(uint8_t split)
{
  return split == SPLIT_OK ? "ok" : (split == SPLIT_BAD ? "bad" : "off");
}

bool cameraFlowSnapshotRequest(uint8_t *buf, cameraFlowSnapshot_t *meta)
{
  if (photoColour) {   /* the split: the driver's copy task fills buf from the next frame to start */
    if (buf == NULL || meta == NULL || state != CAMERA_FLOW_RUNNING || camSplitPending(&camSplitSnap)) {
      return false;
    }
    snapMeta = meta;   /* published by the request's release store */
    return camSplitRequest(&camSplitSnap, buf);
  }
  if (buf == NULL || meta == NULL || state != CAMERA_FLOW_RUNNING ||
      __atomic_load_n(&snapState, __ATOMIC_ACQUIRE) != SNAP_IDLE) {
    return false;
  }
  snapBuf = buf;
  snapMeta = meta;
  __atomic_store_n(&snapState, SNAP_REQUESTED, __ATOMIC_RELEASE);
  return true;
}

bool cameraFlowSnapshotTake(void)
{
  if (photoColour) {
    return camSplitTake(&camSplitSnap);
  }
  if (__atomic_load_n(&snapState, __ATOMIC_ACQUIRE) != SNAP_FILLED) {
    return false;
  }
  __atomic_store_n(&snapState, SNAP_IDLE, __ATOMIC_RELEASE);
  return true;
}

bool cameraFlowSnapshotCancel(void)
{
  if (photoColour) {   /* REQUESTED, CAPTURING, CAPTURED (the generation first) or FILLED -> IDLE; COPYING: false */
    return camSplitCancel(&camSplitSnap);
  }
  uint32_t s = SNAP_REQUESTED;
  if (__atomic_compare_exchange_n(&snapState, &s, SNAP_IDLE, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
    return true;
  }
  if (s == SNAP_FILLED) {
    __atomic_store_n(&snapState, SNAP_IDLE, __ATOMIC_RELEASE);
    return true;
  }
  return s == SNAP_IDLE;
}
