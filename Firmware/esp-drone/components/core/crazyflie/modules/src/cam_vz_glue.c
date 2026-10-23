/*
 * ESP-Drone Firmware
 *
 * cam_vz_glue.c - Runs the camera height observer in shadow mode on every flight
 *
 * The 1 kHz stabilizer loop has little headroom (it waits on the IMU's I2C
 * read every cycle): run inside it, the observer cost ~40-70 us a loop on the
 * drone (its code misses the instruction cache the camera task shares) and the
 * loop fell from ~1000 to ~860-910 Hz. So the stabilizer only folds each
 * sample into a 10 ms block (a few float operations) and hands finished
 * blocks to a low-priority task through a single-producer ring; that task
 * runs the observer at 100 Hz (the rate it was validated at on the logged
 * flights), polls the camera after each block, and publishes a snapshot that
 * every reader (vert_ctrl's input, the recorder, the flight summary, USB)
 * copies under a short lock.
 *
 * That task's per-block code is in IRAM (CAM_VZ_HOT in cam_vz_obs.h,
 * IRAM_ATTR here, the libm/libgcc helpers it calls through linker_camvz.lf):
 * run from flash it evicted the stabilizer's code from the 16 KB instruction
 * cache both cores share, and the loop ran ~962 Hz in flight instead of
 * ~1000 (bench check: 'camvz air full' and 'status' loops). The per-frame
 * update stays in flash (~0.3 lost loops per frame in flight).
 *
 * The late offset noise (cam_vz_obs.h, docs/autonomy-a2b.md 25) comes from NVS
 * cvqlate / cvqlates (auto_glue_core.h), read once here at boot: a stored value
 * out of range is refused loudly (the boot log, 'camvz' qlateCfg=bad, the
 * flight summary's FSUM_CVL_CFG_BAD) and the multiplier stays 1, the filter of
 * before; the rest of the config is never dropped for it (camVzObsInit falls
 * back to the whole default on any invalid field, the camera's gyro window
 * offset included).
 *
 * The offset drift rate T2 (cam_vz_obs.h, docs/autonomy-a2b.md 29) comes from
 * NVS cvt2k (kR x 100, 0 off) / cvt2r (its prior, ug/s), read the same way:
 * a stored value out of range or unreadable, or cvt2k > 0 with cvqlate > 1
 * (R1 and T2 together: never tested), is refused loudly (the boot log,
 * 'camvz' / 'status' t2Cfg=bad, FSUM_CVL_T2_CFG_BAD) and T2 stays off; the
 * late offset noise keeps its own setting.
 *
 * The healthy offset speed-up (cam_vz_obs.h, docs/autonomy-a2b.md 37) comes
 * from NVS cvhb (hbMult x 10, 0 off, 10..80), read the same way: a stored
 * value out of range (1..9 included) or unreadable, or cvhb > 0 with
 * cvqlate > 1 or cvt2k > 0 (untested together), is refused loudly (the boot
 * log, 'camvz' hbCfg=bad, FSUM_CVL_HB_CFG_BAD) and the speed-up stays off;
 * the other two keep their settings.
 */
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "cam_vz_glue.h"
#include "auto_glue_core.h"
#include "camera_flow.h"
#include "imu_level_cal_store.h"
#include "sensfusion6.h"
#include "stabilizer.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "CAMVZ"
#include "debug_cf.h"

/* The gain re-anchor window (cam_vz_obs.h) covers the controller's longest take-off climb (climbMaxS at the take-off
 * cap, VZ_CLIMB_MAX_LIMIT_S less its 0.5 s margin) plus the tracker's lag after it: a take-off cap raised without it
 * would end the window during a high climb (a 2.00 m take-off's 6.0 s climb against the 6.0 s window of the 1.20 m
 * one; 2026-10-15 the hover goes to 2.00 m by MOVEs up, the take-off stays at 1.20 m) */
_Static_assert(CAM_VZ_REANCHOR_MAX_S >= (VZ_CLIMB_MAX_LIMIT_S - 0.5f) + CAM_VZ_REANCHOR_AFTER_S - 1e-4f,
               "the re-anchor window covers the longest climb plus its lag");

#define BLOCK_CLOSE_US    8500u   /* close a block at the sample >= this after its first: 10 samples at 1 kHz
                                    * (10 IMU periods apart, 9990-9999 us on a fast MPU clock: the shadow's
                                    * filter gate allows CVS_OBS_BLOCK_TOL_US, so every block is a step) */
#define RING_LEN          16       /* blocks queued for the task (160 ms) */
#define TASK_STACK        4096     /* bytes, in PSRAM (the task never touches flash) */
#define TASK_PRI          1
#define STACK_CHECK_EVERY 512      /* task wake-ups: its stack high-water mark */

typedef struct {
  uint64_t tEndUs;      /* IMU time of the block's last sample */
  float meanG;          /* mean world-up specific force (NAN if any sample was non-finite) */
  float maxG;           /* highest sample */
  float gyroMaxDps;     /* highest max(|gx|, |gy|) */
  float thrust;         /* thrust of the block's last sample */
  float rateCorr[2];    /* rad/s, mean of the attitude filter's roll/pitch rate correction (gyro bias) */
  bool groundStart;     /* any sample left a ground stop */
  bool resting;         /* last sample: resting on the ground */
  bool still;           /* every sample still */
  bool liftHint;        /* last sample's external lift-off hint */
} block_t;

/* Published for readers in other tasks. */
typedef struct {
  camVzObsOut_t out;
  camVzShadowPhase_t phase;
  camVzShadowRec_t rec;
  fsumCamVz_t stats;
  uint32_t sortie;
  uint32_t sortieAgeMs;
  uint64_t pubUs;       /* esp_timer time of the publish */
  float sceneDM;        /* kyGy / gain, m (0 without a gain): the interactive hover's D */
  uint32_t yawResets;   /* the tracker's restarts since boot */
  fsumCvLate_t late;    /* the late offset noise's and T2's flight-summary fields (refreshed with stats) */
} snap_t;

/* An estimate older than this is not used (the camvz task, priority 1,
 * publishes every ~10 ms block): a starved or stuck task must not leave a
 * frozen "healthy" estimate in the controller (review E2E-2). */
#define SNAP_MAX_AGE_US   100000u
#define CAM_VZ_HEALTHY_WINDOW_US 2000000u   /* camVzObsConfig_t healthyWindowUs default (camVzGlueWhyBits) */

/* stabilizer side */
static block_t acc;
static bool accOpen;
static bool accNonFinite;
static float accSum;
static float accCorr[2];
static uint32_t accN;
static uint64_t accStartUs;
static volatile bool liftHint;
static volatile bool paused;          /* bench diagnostic: no blocks handed over (RAM only) */
static volatile uint32_t batch = 1;   /* bench diagnostic: wake the task every this many blocks */
static volatile uint8_t benchMode;    /* bench diagnostic 'camvz air' (camVzShadowBenchMode_t, RAM only) */
static volatile bool guessAllowed;    /* interactive hover: the estimate may use a guessed G (a commanded turn) */
volatile bool camVzGlueMotionIn;      /* interactive hover: a commanded motion runs (camVzGlueSetMotion) */
volatile float camVzGlueFwdLnIn;      /* interactive hover: the MOVEs' measured forward progress (camVzGlueSetFwdLn) */
static uint32_t stabSamples;
/* single producer (stabilizer) -> single consumer (task) */
static block_t ring[RING_LEN];
static uint32_t ringHead;             /* written by the producer */
static uint32_t ringTail;             /* written by the consumer */
static volatile uint32_t ringDrops;
static TaskHandle_t task;

/* task side */
static camVzShadow_t *sh;             /* PSRAM when there is some */
static bool inPsram;
static camVzGlueStatus_t next;        /* built outside the lock */
static bool haveSeq;
static uint32_t lastSeq;
static uint32_t statsAge;             /* publishes since the flight statistics were refreshed */
static uint32_t statsSortie;
static camVzShadowPhase_t statsPhase;
static uint32_t taskStackFree = UINT32_MAX;
static camVzShadowBench_t bench;      /* task only */

static portMUX_TYPE pubMux = portMUX_INITIALIZER_UNLOCKED;
static camVzGlueStatus_t pub;
static snap_t snap;
static bool pubReady;

/* cost (us): [0] the stabilizer's per-sample work (every 16th timed), [1] the
 * task's block step, [2] the task's frame update, [3] the task's publish */
static uint32_t profMax[4], profN[4];
static uint64_t profSum[4];

static inline IRAM_ATTR void profAdd(int i, uint32_t us)
{
  profN[i]++;
  profSum[i] += us;
  if (us > profMax[i]) {
    profMax[i] = us;
  }
}

void camVzGlueProfile(uint32_t maxUs[4], uint32_t avgUs10[4])
{
  for (int i = 0; i < 4; i++) {
    maxUs[i] = profMax[i];
    avgUs10[i] = profN[i] ? (uint32_t)(profSum[i] * 10 / profN[i]) : 0;
  }
}

void camVzGlueProfileReset(void)
{
  for (int i = 0; i < 4; i++) {
    profMax[i] = 0;
    profN[i] = 0;
    profSum[i] = 0;
  }
}

static IRAM_ATTR void toObsFrame(const cameraFlowSample_t *c, camVzObsFrame_t *f)
{
  f->seq = c->seq;
  f->frameTimeUs = c->frameTimeUs;
  f->dtUs = c->dtUs;
  f->dx = c->dx;
  f->dy = c->dy;
  f->scale = c->scale;
  f->rot = c->rot;
  for (int k = 0; k < 3; k++) {
    f->gyroAngle[k] = c->gyroAngle[k];
  }
  f->texture = c->texture;
  f->residual = c->residual;
  f->ok = c->ok;
  f->fwdLn = camVzGlueFwdLnIn;   /* the gain follows the scene distance through a forward move (cam_vz_obs.h) */
}

#define STATS_EVERY 10   /* publishes (~100 ms) between refreshes of the flight statistics */

static IRAM_ATTR void publish(void)
{
  const uint64_t t0 = (uint64_t)esp_timer_get_time();
  snap_t s;
  s.out = sh->out;
  s.phase = sh->phase;
  camVzShadowRecord(sh, &s.rec);
  /* The per-flight statistics (flash code, ~1.1 KB) only at a new sortie or
   * phase (lift-off, touchdown: the values the flight log keeps) and every
   * STATS_EVERY publishes in between; otherwise the last ones. */
  if (++statsAge >= STATS_EVERY || sh->sortie != statsSortie || sh->phase != statsPhase) {
    statsAge = 0;
    statsSortie = sh->sortie;
    statsPhase = sh->phase;
    camVzShadowFlightStats(sh, &s.stats);
    camVzShadowLateStats(sh, &s.late);
  } else {
    s.stats = snap.stats;   /* only this task writes snap */
    s.late = snap.late;
  }
  s.sortie = sh->sortie;
  s.sceneDM = sh->out.gain > 0.0f ? sh->obs.cfg.kyGy / sh->out.gain : 0.0f;
  s.yawResets = sh->obs.cnt.yawResets;
  const uint64_t ageMs = sh->lastUs > sh->sortieUs ? (sh->lastUs - sh->sortieUs) / 1000u : 0;
  s.sortieAgeMs = sh->sortie == 0 ? UINT32_MAX : (ageMs > UINT32_MAX ? UINT32_MAX : (uint32_t)ageMs);

  camVzGlueStatus_t *p = &next;
  p->out = sh->out;
  p->cnt = sh->obs.cnt;
  p->stats = s.stats;
  p->phase = (uint8_t)sh->phase;
  p->liftBy = (uint8_t)sh->liftBy;
  p->lastDecision = (uint8_t)sh->lastDecision;
  p->inPsram = inPsram;
  p->benchAir = (uint8_t)bench.mode;
  p->sortie = sh->sortie;
  p->g0 = sh->g0;
  for (int k = 0; k < 3; k++) {
    p->gyroBiasDps[k] = sh->gyroBias[k] * 57.29578f;
  }
  p->bytes = sizeof(camVzShadow_t);
  p->stepMaxUs = profMax[1];
  p->stepSamples = profN[1];
  p->stepSumUs = profSum[1];
  p->frameMaxUs = profMax[2];
  p->frameCalls = profN[2];
  p->frameSumUs = profSum[2];
  p->stabStackFree = taskStackFree;
  s.pubUs = (uint64_t)esp_timer_get_time();
  portENTER_CRITICAL(&pubMux);
  snap = s;
  pub = *p;
  pubReady = true;
  portEXIT_CRITICAL(&pubMux);
  profAdd(3, (uint32_t)((uint64_t)esp_timer_get_time() - t0));
}

static IRAM_ATTR void runBlock(const block_t *b)
{
  const uint64_t t0 = (uint64_t)esp_timer_get_time();
  camVzShadowIn_t in = {
    .timeUs = b->tEndUs,
    .accUpG = b->meanG,
    .accMaxG = b->maxG,
    .thrust = b->thrust,
    .gyroMaxDps = b->gyroMaxDps,
    .groundStart = b->groundStart,
    .restingOnGround = b->resting,
    .still = b->still,
    .liftoffHint = b->liftHint,
    /* yaw: the accelerometer cannot see a yaw bias, its correction is ~0 (and kyGz is small) */
    .rateCorr = { b->rateCorr[0], b->rateCorr[1], 0.0f },
  };
  /* Bench load test: real thrust or a real ground start ends it on this very
   * block, which then goes through unchanged (except the props-off spin mode,
   * which usb_diag only allows under a gcap bench cap). */
  if ((b->thrust > 0.0f || b->groundStart) && benchMode != CVS_BENCH_SPIN) {
    benchMode = CVS_BENCH_OFF;
  }
  bench.mode = (camVzShadowBenchMode_t)benchMode;
  if (bench.mode != CVS_BENCH_OFF || bench.started) {
    camVzShadowBenchIn(&bench, &in);
  }
  camVzObsSetMotion(&sh->obs, camVzGlueMotionIn);   /* a commanded motion: a guard event of the late offset noise */
  camVzShadowStep(sh, &in);
  const uint64_t t1 = (uint64_t)esp_timer_get_time();
  profAdd(1, (uint32_t)(t1 - t0));

  /* the newest camera sample, right after the block it follows (as in the replay) */
  cameraFlowSample_t c;
  if (cameraFlowGetLatest(&c) && (!haveSeq || c.seq != lastSeq)) {
    haveSeq = true;
    lastSeq = c.seq;
    camVzObsFrame_t f;
    toObsFrame(&c, &f);
    if (bench.mode >= CVS_BENCH_FULL) {
      camVzShadowBenchFrame(&bench, &f);
    }
    camVzObsSetTurn(&sh->obs, guessAllowed);   /* a commanded turn: its frames pass a 45 deg/s yaw gate */
    camVzShadowFrame(sh, &f, t1);
    profAdd(2, (uint32_t)((uint64_t)esp_timer_get_time() - t1));
  }
}

static void IRAM_ATTR camVzTask(void *param)
{
  uint32_t wakes = 0;
  while (1) {
    ulTaskNotifyTake(pdTRUE, M2T(50));
    bool any = false;
    while (__atomic_load_n(&ringTail, __ATOMIC_RELAXED) != __atomic_load_n(&ringHead, __ATOMIC_ACQUIRE)) {
      const uint32_t tail = ringTail;
      const block_t b = ring[tail % RING_LEN];
      __atomic_store_n(&ringTail, tail + 1, __ATOMIC_RELEASE);
      runBlock(&b);
      any = true;
    }
    if (any) {
      publish();
    }
    if (++wakes % STACK_CHECK_EVERY == 1) {
      taskStackFree = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    }
  }
}

/* n autonomy settings read at boot: true if every one is absent or stored in range (v: the value or the default); a
 * stored value out of range, or one that cannot be read, is refused loudly (false, v the defaults). */
static bool loadSettings(const autoSettingId_t ids[], int32_t v[], int n, const char *what)
{
  bool ok = true;
  for (int i = 0; i < n; i++) {
    const autoSettingInfo_t *si = autoSettingInfo(ids[i]);
    v[i] = si->def;
    const esp_err_t err = autoSettingStoreLoad(si->name, (uint8_t)si->type, &v[i]);
    if ((err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) || (err == ESP_OK && !autoSettingInRange(ids[i], v[i]))) {
      DEBUG_PRINTW("camera height observer: stored %s %ld refused (%s, %ld..%ld): %s is OFF", si->name, (long)v[i],
                   err == ESP_OK ? "out of range" : esp_err_to_name(err), (long)si->min, (long)si->max, what);
      ok = false;
    }
  }
  if (!ok) {
    for (int i = 0; i < n; i++) {
      v[i] = autoSettingInfo(ids[i])->def;
    }
  }
  return ok;
}

void camVzGlueInit(void)
{
  if (sh) {
    return;
  }
  /* PSRAM first: internal RAM is the scarce one (docs/autonomy-a2.md). */
  sh = heap_caps_malloc(sizeof(*sh), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  inPsram = sh != NULL;
  if (!sh) {
    sh = heap_caps_malloc(sizeof(*sh), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (!sh) {
    DEBUG_PRINTW("camera height observer off: no memory for %u bytes", (unsigned)sizeof(camVzShadow_t));
    return;
  }
  memset(sh, 0, sizeof(*sh));
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  cfg.gyroOffsetUs = cameraFlowGyroOffsetUs();   /* the camera's clock (docs/autonomy-a2b.md 17.2) */
  /* The late offset noise (NVS cvqlate x, cvqlates s; absent: 1 and 40, off). A stored value out of range, or one
   * that cannot be read, is refused loudly and leaves it off; never the rest of the config. */
  static const autoSettingId_t ids[2] = {AUTO_SET_CVQLATE, AUTO_SET_CVQLATES};
  int32_t v[2];
  bool qLateCfgBad = !loadSettings(ids, v, 2, "the late offset noise");   /* kept in the shadow (PSRAM): 'camvz', the
                                                                         * flight summary */
  if (!qLateCfgBad) {
    cfg.qLateMult = (float)v[0];
    cfg.qLateS = (float)v[1];
  }
  /* The offset drift rate T2 (NVS cvt2k = kR x 100, cvt2r = the prior in ug/s; absent: 0 and 124, off), the same way;
   * refused too with the late offset noise on (an untested combination: camVzObsConfigValid would refuse the whole
   * config), which keeps its own setting */
  static const autoSettingId_t tids[2] = {AUTO_SET_CVT2K, AUTO_SET_CVT2R};
  int32_t tv[2];
  bool t2CfgBad = !loadSettings(tids, tv, 2, "the offset drift rate T2");
  if (!t2CfgBad && tv[0] > 0 && cfg.qLateMult > 1.0f) {
    DEBUG_PRINTW("camera height observer: cvt2k %ld with cvqlate %.0f refused (R1 and T2 together): T2 is OFF",
                 (long)tv[0], (double)cfg.qLateMult);
    t2CfgBad = true;
  }
  if (!t2CfgBad) {
    /* divided, not multiplied by 0.01 / 0.001: the nearest float, so 124 is the default 0.124f and 350 the bound */
    cfg.t2kR = (float)tv[0] / 100.0f;
    cfg.t2PriorMgS = (float)tv[1] / 1000.0f;
  }
  /* The healthy offset speed-up (NVS cvhb = hbMult x 10; absent: 0, off), the same way; refused too with the late
   * offset noise or T2 on (untested together: camVzObsConfigValid would refuse the whole config), which keep theirs */
  static const autoSettingId_t hid[1] = {AUTO_SET_CVHB};
  int32_t hv[1];
  bool hbCfgBad = !loadSettings(hid, hv, 1, "the healthy offset speed-up");
  if (!hbCfgBad && hv[0] > 0 && (cfg.qLateMult > 1.0f || cfg.t2kR > 0.0f)) {
    DEBUG_PRINTW("camera height observer: cvhb %ld with cvqlate %.0f / cvt2k %.0f refused (untested together): the "
                 "healthy offset speed-up is OFF", (long)hv[0], (double)cfg.qLateMult, (double)(cfg.t2kR * 100.0f));
    hbCfgBad = true;
  }
  if (!hbCfgBad) {
    cfg.hbMult = (float)hv[0] / 10.0f;   /* divided: the nearest float (30 is 3.0f) */
  }
  if (!camVzShadowInit(sh, &cfg)) {
    /* never on these values (camVzObsConfigValid takes every setting in range): say so, and keep the camera's
     * offset on the default the observer fell back to */
    DEBUG_PRINTW("camera height observer: config refused, defaults (the late offset noise, T2 and the healthy offset "
                 "speed-up OFF)");
    qLateCfgBad = true;
    t2CfgBad = t2CfgBad || tv[0] > 0;
    hbCfgBad = hbCfgBad || hv[0] > 0;
    sh->obs.cfg.gyroOffsetUs = cfg.gyroOffsetUs;
  }
  sh->lateCfgBad = qLateCfgBad;
  sh->t2CfgBad = t2CfgBad;
  sh->hbCfgBad = hbCfgBad;

  /* The task's stack in PSRAM (CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY): it
   * only computes and copies, never writes flash. Its control block must be
   * internal. */
  static StaticTask_t tcb;
  StackType_t *stack = heap_caps_malloc(TASK_STACK, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!stack) {
    stack = heap_caps_malloc(TASK_STACK, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (stack) {
    /* Core 0: core 1 is the camera task's (pinned, ~saturated by the flow fit). */
    task = xTaskCreateStaticPinnedToCore(camVzTask, "camvz", TASK_STACK, NULL, TASK_PRI, stack, &tcb, 0);
  }
  if (!task) {
    DEBUG_PRINTW("camera height observer off: no task");
    return;
  }
  DEBUG_PRINTI("camera height observer (shadow, 100 Hz task): %u bytes in %s, late offset noise x%.0f from %.0f s%s, "
               "T2 k%.2f r%+.3f mg/s%s, healthy offset speed-up x%.1f%s", (unsigned)sizeof(camVzShadow_t),
               inPsram ? "PSRAM" : "internal RAM", (double)sh->obs.cfg.qLateMult, (double)sh->obs.cfg.qLateS,
               sh->lateCfgBad ? " (stored value refused)" : "", (double)sh->obs.cfg.t2kR,
               (double)sh->obs.cfg.t2PriorMgS, sh->t2CfgBad ? " (stored value refused)" : "",
               (double)sh->obs.cfg.hbMult, sh->hbCfgBad ? " (stored value refused)" : "");
}

void camVzGlueSetGyroOffsetUs(uint32_t us)
{
  if (sh != NULL && us <= 100000u) {
    __atomic_store_n(&sh->obs.cfg.gyroOffsetUs, us, __ATOMIC_RELAXED);
  }
}

void camVzGlueSetPaused(bool p)
{
  paused = p;
}

bool camVzGluePaused(void)
{
  return paused;
}

void camVzGlueSetBatch(uint32_t blocks)
{
  batch = blocks < 1 ? 1 : (blocks > RING_LEN / 2 ? RING_LEN / 2 : blocks);
}

void camVzGlueSetBenchAir(uint8_t mode)
{
  benchMode = mode > CVS_BENCH_SPIN ? CVS_BENCH_OFF : mode;
}

uint8_t camVzGlueBenchAir(void)
{
  return benchMode;
}

/* In IRAM: from flash this ran ~20 us a sample (cache misses: the 16 KB
 * instruction cache is shared with the camera task's flow fit). */
void IRAM_ATTR camVzGlueStep(const sensorData_t *sensors, float thrustApplied, bool groundStart, bool still)
{
  if (!task || paused) {
    return;
  }
  const bool timed = (++stabSamples & 15u) == 0;
  const uint64_t t0 = timed ? (uint64_t)esp_timer_get_time() : 0;

  const uint64_t t = sensors->interruptTimestamp;
  const float a = sensfusion6GetAccZ(sensors->acc.x, sensors->acc.y, sensors->acc.z);
  const float gyro = fmaxf(fabsf(sensors->gyro.x), fabsf(sensors->gyro.y));
  if (!accOpen) {
    accOpen = true;
    memset(&acc, 0, sizeof(acc));
    acc.maxG = -INFINITY;
    acc.still = true;
    accSum = 0.0f;
    accCorr[0] = accCorr[1] = 0.0f;
    accN = 0;
    accNonFinite = false;
    accStartUs = t;
  }
  if (isfinite(a)) {
    accSum += a;
    accCorr[0] += sensfusion6RateCorr[0];
    accCorr[1] += sensfusion6RateCorr[1];
    accN++;
    if (a > acc.maxG) {
      acc.maxG = a;
    }
  } else {
    accNonFinite = true;
  }
  if (gyro > acc.gyroMaxDps) {
    acc.gyroMaxDps = gyro;
  }
  acc.thrust = thrustApplied;
  acc.groundStart = acc.groundStart || groundStart;
  acc.resting = stabilizerOnGround();
  acc.still = acc.still && still;
  acc.liftHint = liftHint;

  /* Close the block after ~10 samples (or at once on a non-finite one: the
   * observer faults on it). */
  if (accNonFinite || (t > accStartUs && t - accStartUs >= BLOCK_CLOSE_US)) {
    acc.tEndUs = t;
    acc.meanG = (!accNonFinite && accN > 0) ? accSum / (float)accN : NAN;
    acc.rateCorr[0] = accN > 0 ? accCorr[0] / (float)accN : 0.0f;
    acc.rateCorr[1] = accN > 0 ? accCorr[1] / (float)accN : 0.0f;
    if (!isfinite(acc.maxG)) {
      acc.maxG = acc.meanG;
    }
    const uint32_t head = ringHead;
    const uint32_t queued = head - __atomic_load_n(&ringTail, __ATOMIC_ACQUIRE);
    if (queued < RING_LEN) {
      ring[head % RING_LEN] = acc;
      __atomic_store_n(&ringHead, head + 1, __ATOMIC_RELEASE);
      if (queued + 1 >= batch) {
        xTaskNotifyGive(task);
      }
    } else {
      ringDrops++;
    }
    accOpen = false;
  }
  if (timed) {
    profAdd(0, (uint32_t)((uint64_t)esp_timer_get_time() - t0));
  }
}

void IRAM_ATTR camVzGlueSetLiftoffHint(bool liftedOff)
{
  liftHint = liftedOff;
}

bool IRAM_ATTR camVzGlueVzInput(float vzG0, vzInput_t *in)
{
  camVzObsOut_t o;
  camVzShadowPhase_t phase;
  const uint64_t nowUs = (uint64_t)esp_timer_get_time();
  portENTER_CRITICAL(&pubMux);
  const bool ok = pubReady;
  o = snap.out;
  phase = snap.phase;
  const uint64_t pubUs = snap.pubUs;
  portEXIT_CRITICAL(&pubMux);
  /* never the bench load test's make-believe flight, never a snapshot frozen
   * by a pause (it keeps its last phase, CVS_AIR if paused in the air), and
   * never one the task has stopped refreshing */
  const bool fresh = nowUs >= pubUs ? nowUs - pubUs <= SNAP_MAX_AGE_US : true;
  const bool valid = ok && fresh && benchMode == CVS_BENCH_OFF && !paused && !o.fault && isfinite(o.v) &&
                     isfinite(o.biasHeldG) && isfinite(vzG0);
  in->camVzValid = valid;
  /* the snapshot is at most SNAP_MAX_AGE_US old: the sortie phase decides on the ground; through a
   * commanded turn (interactive hover) a guessed G counts (cam_vz_obs.h healthyGuess) */
  in->camHealthy = valid && (o.healthy || (guessAllowed && o.healthyGuess)) && o.airborne && phase == CVS_AIR;
  in->camVz = valid ? o.v : 0.0f;
  /* the held offset (biasG unless the late offset noise is past its step, cam_vz_obs.h): the one the controller
   * keeps when the camera drops out */
  in->camBias = valid ? o.biasHeldG + 1.0f - vzG0 : 0.0f;
  return ok;
}

void IRAM_ATTR camVzGlueSetGuessAllowed(bool allowed)
{
  guessAllowed = allowed;
}

bool IRAM_ATTR camVzGlueScene(float *dM, bool *tracked, uint32_t *resets)
{
  camVzObsOut_t o;
  const uint64_t nowUs = (uint64_t)esp_timer_get_time();
  portENTER_CRITICAL(&pubMux);
  const bool ok = pubReady;
  o = snap.out;
  const float d = snap.sceneDM;
  const uint32_t n = snap.yawResets;
  const uint64_t pubUs = snap.pubUs;
  portEXIT_CRITICAL(&pubMux);
  const bool fresh = nowUs >= pubUs ? nowUs - pubUs <= SNAP_MAX_AGE_US : true;
  *dM = ok ? d : 0.0f;
  *tracked = ok && fresh && o.gainTracked && !o.fault && benchMode == CVS_BENCH_OFF && !paused && isfinite(d) &&
             d > 0.0f;
  *resets = ok ? n : 0u;
  return ok;
}

uint8_t camVzGlueWhyBits(void)
{
  camVzObsOut_t o;
  const uint64_t nowUs = (uint64_t)esp_timer_get_time();
  portENTER_CRITICAL(&pubMux);
  const bool ok = pubReady;
  o = snap.out;
  const uint64_t pubUs = snap.pubUs;
  portEXIT_CRITICAL(&pubMux);
  const bool fresh = nowUs >= pubUs ? nowUs - pubUs <= SNAP_MAX_AGE_US : true;
  if (!ok || !fresh || o.fault || benchMode != CVS_BENCH_OFF || paused) {
    return 0x08u;   /* AUTO_CB_FAULT */
  }
  uint8_t b = 0;
  const bool recent = o.sinceAcceptUs != UINT32_MAX && o.sinceAcceptUs <= CAM_VZ_HEALTHY_WINDOW_US;
  if (!recent) {
    b |= 0x01u;     /* AUTO_CB_NO_FRAMES */
  }
  if (!o.gainTracked && !(guessAllowed && o.healthyGuess)) {
    b |= 0x02u;     /* AUTO_CB_NO_G */
  }
  if (recent && o.gainTracked && !o.healthy) {
    b |= 0x04u;     /* AUTO_CB_CUSUM: healthy needs no alarm holding, the rest is there */
  }
  return b;
}

void camVzGlueRecord(camVzShadowRec_t *r)
{
  portENTER_CRITICAL(&pubMux);
  if (pubReady) {
    *r = snap.rec;
  } else {
    memset(r, 0, sizeof(*r));
  }
  portEXIT_CRITICAL(&pubMux);
}

bool camVzGlueFlightStats(fsumCamVz_t *out, fsumCvLate_t *late, uint32_t *sortie, uint32_t *sortieAgeMs)
{
  portENTER_CRITICAL(&pubMux);
  const bool ok = pubReady;
  if (ok) {
    *out = snap.stats;
    *late = snap.late;
    *sortie = snap.sortie;
    *sortieAgeMs = snap.sortieAgeMs;
  }
  portEXIT_CRITICAL(&pubMux);
  if (!ok) {
    memset(out, 0, sizeof(*out));
    memset(late, 0, sizeof(*late));
    *sortie = 0;
    *sortieAgeMs = UINT32_MAX;
  }
  return ok;
}

bool camVzGlueLate(camVzGlueLate_t *l)
{
  memset(l, 0, sizeof(*l));
  if (sh == NULL) {
    return false;
  }
  /* word reads of the task's state without the lock: diagnostics (a value may be one frame old) */
  l->qLateMult = sh->obs.cfg.qLateMult;
  l->qLateS = sh->obs.cfg.qLateS;
  l->cfgBad = sh->lateCfgBad;
  l->state = camVzObsLateState(&sh->obs);
  l->lateMs = sh->obs.lateUs / 1000u;
  l->guards = sh->obs.lateGuards;
  return true;
}

bool camVzGlueT2(camVzGlueT2_t *t)
{
  memset(t, 0, sizeof(*t));
  if (sh == NULL) {
    return false;
  }
  /* word reads of the task's state without the lock: diagnostics (a value may be one frame old) */
  t->kR = sh->obs.cfg.t2kR;
  t->priorMgS = sh->obs.cfg.t2PriorMgS;
  t->boundMgS = sh->obs.cfg.t2BoundMgS;
  t->startS = sh->obs.cfg.t2StartS;
  t->cfgBad = sh->t2CfgBad;
  camVzObsT2Get(&sh->obs, &t->t2);
  return true;
}

bool camVzGlueHb(camVzGlueHb_t *h)
{
  memset(h, 0, sizeof(*h));
  if (sh == NULL) {
    return false;
  }
  /* word reads of the task's state without the lock: diagnostics (a value may be one block old) */
  h->mult = sh->obs.cfg.hbMult;
  h->cfgBad = sh->hbCfgBad;
  camVzObsOut_t o;
  camVzObsGet(&sh->obs, &o);
  h->on = o.hbOn;
  h->onMs = sh->obs.hbUs / 1000u;
  return true;
}

bool camVzGlueGetStatus(camVzGlueStatus_t *st)
{
  portENTER_CRITICAL(&pubMux);
  const bool ok = pubReady;
  if (ok) {
    *st = pub;
  }
  portEXIT_CRITICAL(&pubMux);
  return ok;
}

uint32_t camVzGlueRingDrops(void)
{
  return ringDrops;
}
