/*
 * Host test of the camera height observer glue's pause and bench rules
 * (components/core/crazyflie/modules/src/cam_vz_glue.c, docs/autonomy-a2b.md
 * section 7.3): while 'camvz off' pauses the observer no block reaches it and
 * the published snapshot freezes, so camVzGlueVzInput() reports the estimate
 * unusable, a snapshot frozen in the air (CVS_AIR, healthy) included;
 * camVzGluePaused() reports the pause (the autonomy glue refuses a FLIGHT or
 * BENCH-imu ARM on it, tools/tests/test_autonomy_glue_host.c); a bench mode
 * stays unusable as before; 'camvz on' hands blocks over again. The
 * observer's gyro window offset is the camera's (docs/autonomy-a2b.md 17.2:
 * 26 ms at the 13.33 MHz clock) and follows a change of the camera's clock
 * from the next frame on (the observer reads it once per frame).
 *
 * The late offset noise (2026-10-15, cam_vz_obs.h, docs/autonomy-a2b.md 25):
 * NVS cvqlate / cvqlates read at boot, every value in range taken, one out of
 * range or unreadable refused loudly (the flag 'camvz' and the flight summary
 * show, the multiplier 1, the camera's gyro window offset kept); the
 * controller gets the held offset (biasHeldG); a commanded motion reaches the
 * observer before each block.
 *
 * The offset drift rate T2 (2026-10-16, cam_vz_obs.h, docs/autonomy-a2b.md
 * 29): NVS cvt2k / cvt2r read at boot the same way, refused (T2 off, R1 as
 * stored) with cvqlate > 1, shown by 'camvz' / 'status' (camVzGlueT2) and
 * the flight summary.
 *
 * The healthy offset speed-up (2026-10-18, cam_vz_obs.h, docs/autonomy-a2b.md
 * 37): NVS cvhb read at boot the same way (hbMult x 10: 0 off, 10..80; 1..9
 * refused), refused (off, the others as stored) with cvqlate > 1 or cvt2k > 0,
 * shown by 'camvz' (camVzGlueHb) and the flight summary's two flag bits.
 *
 * cam_vz_glue.c is included, so the test reaches its static snapshot; the
 * stubs are tools/tests/autonomy_glue_host/stub (defined below).
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=gnu99 -O1 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function \
 *      -Itools/tests/autonomy_glue_host/stub -I$C/utils/interface -I$C/modules/interface -I$C/hal/interface \
 *      -I$C/modules/src tools/tests/test_cam_vz_glue_host.c $C/utils/src/cam_vz_shadow.c $C/utils/src/cam_vz_obs.c \
 *      $C/utils/src/auto_glue_core.c $C/utils/src/auto_proto.c -lm -o /tmp/test_cam_vz_glue_host && \
 *      /tmp/test_cam_vz_glue_host
 */
#include <math.h>
#include <stdlib.h>

#include "glue_host.h"
#include "cam_vz_glue.c"

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); \
                                                 printf("\n"); } } while (0)

/* ---- stubs ---- */
bool glueHostQuiet = true;
static int lockDepth;
void glueHostLock(portMUX_TYPE *m) { (void)m; lockDepth++; }
void glueHostUnlock(portMUX_TYPE *m) { (void)m; lockDepth--; }
static int64_t nowUs = 1000000;
int64_t esp_timer_get_time(void) { return nowUs; }
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t t) { (void)t; return 1000; }
void *heap_caps_malloc(size_t size, uint32_t caps) { (void)caps; return malloc(size); }
static int dummyTask;
TaskHandle_t xTaskCreateStaticPinnedToCore(void (*fn)(void *), const char *name, uint32_t depth, void *arg, int pri,
                                           StackType_t *stack, StaticTask_t *tcb, int core)
{
  return &dummyTask;
}
static int notifies;
void xTaskNotifyGive(TaskHandle_t t) { notifies++; }
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t wait) { return 0; }
float sensfusion6RateCorr[3];
float sensfusion6GetAccZ(const float ax, const float ay, const float az) { return az; }
bool stabilizerOnGround(void) { return true; }
bool cameraFlowGetLatest(cameraFlowSample_t *out) { memset(out, 0, sizeof(*out)); return false; }
static uint32_t camOffsetUs = 26000;   /* camera_flow.c at 13.33 MHz (cam_clock.h) */
uint32_t cameraFlowGyroOffsetUs(void) { return camOffsetUs; }
/* the autonomy settings store: cvqlate / cvqlates / cvt2k / cvt2r / cvhb (NVS_ABSENT: absent), or an error for all */
#define NVS_ABSENT INT32_MIN
static int32_t nvsQLate = NVS_ABSENT, nvsQLateS = NVS_ABSENT, nvsT2K = NVS_ABSENT, nvsT2R = NVS_ABSENT,
               nvsHb = NVS_ABSENT;
static esp_err_t nvsErr = ESP_OK;
esp_err_t autoSettingStoreLoad(const char *key, uint8_t nvsType, int32_t *value)
{
  (void)nvsType;
  const int32_t v = strcmp(key, "cvqlate") == 0 ? nvsQLate
                    : strcmp(key, "cvqlates") == 0 ? nvsQLateS
                    : strcmp(key, "cvt2k") == 0 ? nvsT2K
                    : strcmp(key, "cvt2r") == 0 ? nvsT2R
                    : strcmp(key, "cvhb") == 0 ? nvsHb : NVS_ABSENT;
  if (nvsErr != ESP_OK) {
    return nvsErr;
  }
  if (v == NVS_ABSENT) {
    return ESP_ERR_NVS_NOT_FOUND;
  }
  *value = v;
  return ESP_OK;
}
const char *esp_err_to_name(esp_err_t e) { return e == ESP_ERR_NVS_TYPE_MISMATCH ? "ESP_ERR_NVS_TYPE_MISMATCH" : "?"; }

/* 20 stabilizer samples 1 ms apart (two 10 ms blocks). */
static void steps(void)
{
  sensorData_t sd;
  memset(&sd, 0, sizeof(sd));
  sd.acc.z = 1.0f;
  for (int i = 0; i < 20; i++) {
    nowUs += 1000;
    sd.interruptTimestamp = (uint64_t)nowUs;
    camVzGlueStep(&sd, 0.0f, false, true);
  }
}

static bool usable(float *vz)
{
  vzInput_t in;
  memset(&in, 0x55, sizeof(in));
  const bool ok = camVzGlueVzInput(1.0f, &in);
  *vz = in.camVz;
  return ok && in.camHealthy && in.camVzValid;
}

static bool unusable(void)
{
  vzInput_t in;
  memset(&in, 0x55, sizeof(in));
  camVzGlueVzInput(1.0f, &in);
  return !in.camHealthy && !in.camVzValid && in.camVz == 0.0f && in.camBias == 0.0f;
}

int main(void)
{
  camVzGlueInit();
  CHECK(sh != NULL && task != NULL, "observer and task");
  /* the camera's gyro window offset (17.2) */
  camVzObsConfig_t def;
  camVzObsDefaultConfig(&def);
#ifdef ESPFLY_NATIVE_VGA
  CHECK(def.gyroOffsetUs == 0, "VGA capture-centre timestamps need no additional observer offset");
#else
  CHECK(def.gyroOffsetUs == 15000, "legacy start timestamps retain the fitted 15 ms offset");
#endif
  CHECK(sh->obs.cfg.gyroOffsetUs == 26000 && camVzObsConfigValid(&sh->obs.cfg),
        "init: the camera's offset (%u us), a valid config", (unsigned)sh->obs.cfg.gyroOffsetUs);
  def.gyroOffsetUs = 26000;
  CHECK(memcmp(&def, &sh->obs.cfg, sizeof(def)) == 0, "otherwise the default config");
  camVzGlueSetGyroOffsetUs(15000);
  CHECK(sh->obs.cfg.gyroOffsetUs == 15000, "the fallback to 20 MHz: 15 ms from the next frame");
  camVzObsReset(&sh->obs);
  CHECK(sh->obs.cfg.gyroOffsetUs == 15000, "kept through the observer's reset (a new sortie)");
  camVzGlueSetGyroOffsetUs(100001);
  CHECK(sh->obs.cfg.gyroOffsetUs == 15000, "an offset over 100 ms is ignored");
  camVzGlueSetGyroOffsetUs(26000);
  CHECK(!camVzGluePaused() && unusable(), "no snapshot yet: unusable");

  /* a snapshot published in the air: healthy, airborne, CVS_AIR */
  portENTER_CRITICAL(&pubMux);
  snap.out.healthy = true;
  snap.out.airborne = true;
  snap.out.fault = false;
  snap.out.v = 0.25f;
  snap.out.biasG = 0.001f;
  snap.phase = CVS_AIR;
  snap.pubUs = (uint64_t)nowUs;
  pubReady = true;
  portEXIT_CRITICAL(&pubMux);
  float vz = 0.0f;
  CHECK(usable(&vz) && vz == 0.25f, "airborne snapshot: usable (%.2f)", (double)vz);

  /* 'camvz off' in the air: no blocks, the snapshot freezes, and it is unusable */
  camVzGlueSetPaused(true);
  CHECK(camVzGluePaused(), "paused reported");
  const uint32_t head0 = ringHead;
  steps();
  CHECK(ringHead == head0, "paused: no block handed over (%u)", (unsigned)(ringHead - head0));
  CHECK(unusable(), "paused: the frozen CVS_AIR snapshot is unusable");
  vzInput_t in;
  CHECK(camVzGlueVzInput(1.0f, &in), "paused: the observer still exists (true)");

  /* 'camvz on': blocks again, usable again */
  camVzGlueSetPaused(false);
  CHECK(!camVzGluePaused(), "resumed");
  steps();
  CHECK(ringHead - head0 >= 1, "resumed: blocks handed over (%u)", (unsigned)(ringHead - head0));
  CHECK(usable(&vz), "resumed: usable");

  /* a bench mode: unusable, as before */
  camVzGlueSetBenchAir(CVS_BENCH_AIR);
  CHECK(camVzGlueBenchAir() == CVS_BENCH_AIR && unusable(), "bench air: unusable");
  camVzGlueSetBenchAir(CVS_BENCH_OFF);
  CHECK(usable(&vz), "bench off: usable");

  /* a task that stops publishing (starved or stuck): its last snapshot,
   * healthy in the air, becomes unusable after SNAP_MAX_AGE_US (E2E-2) */
  nowUs += SNAP_MAX_AGE_US - 40000;   /* 40 ms of steps() above */
  CHECK(usable(&vz), "100 ms after the publish: still usable");
  nowUs += 1;
  CHECK(unusable(), "older than 100 ms: unusable");
  portENTER_CRITICAL(&pubMux);
  snap.pubUs = (uint64_t)nowUs;
  portEXIT_CRITICAL(&pubMux);
  CHECK(usable(&vz), "published again: usable");

  /* Interactive hover: a commanded turn restarts the scale tracker. Only
   * healthyGuess (a guessed G) then: unusable unless the guess is allowed. */
  portENTER_CRITICAL(&pubMux);
  snap.out.healthy = false;
  snap.out.healthyGuess = true;
  snap.pubUs = (uint64_t)nowUs;
  portEXIT_CRITICAL(&pubMux);
  vzInput_t gi;
  camVzGlueVzInput(1.0f, &gi);
  CHECK(!gi.camHealthy && gi.camVzValid, "the tracker restarted: not healthy (the guess not allowed)");
  camVzGlueSetGuessAllowed(true);
  camVzGlueVzInput(1.0f, &gi);
  CHECK(gi.camHealthy, "a turn: the guessed G allowed, healthy");
  portENTER_CRITICAL(&pubMux);
  snap.out.healthyGuess = false;
  portEXIT_CRITICAL(&pubMux);
  camVzGlueVzInput(1.0f, &gi);
  CHECK(!gi.camHealthy, "allowed, but no guess either (no recent update): not healthy");
  camVzGlueSetGuessAllowed(false);
  portENTER_CRITICAL(&pubMux);
  snap.out.healthy = true;
  portEXIT_CRITICAL(&pubMux);
  /* the scene distance D = kyGy / G */
  portENTER_CRITICAL(&pubMux);
  snap.out.gain = 50.0f;
  snap.out.gainTracked = true;
  snap.sceneDM = sh->obs.cfg.kyGy / 50.0f;
  snap.yawResets = 3;
  snap.pubUs = (uint64_t)nowUs;
  portEXIT_CRITICAL(&pubMux);
  float d = 0.0f;
  bool tracked = false;
  uint32_t resets = 0;
  CHECK(camVzGlueScene(&d, &tracked, &resets) && tracked && fabsf(d - 1.9958f) < 1e-3f && resets == 3,
        "scene: G 50 px/m, D %.3f m (kyGy / G), tracked, 3 restarts", (double)d);
  portENTER_CRITICAL(&pubMux);
  snap.out.gainTracked = false;
  portEXIT_CRITICAL(&pubMux);
  CHECK(camVzGlueScene(&d, &tracked, &resets) && !tracked, "the tracker restarted: not tracked");
  portENTER_CRITICAL(&pubMux);
  snap.out.gainTracked = true;
  portEXIT_CRITICAL(&pubMux);
  camVzGlueSetPaused(true);
  CHECK(camVzGlueScene(&d, &tracked, &resets) && !tracked, "paused: not tracked");
  camVzGlueSetPaused(false);
  nowUs += SNAP_MAX_AGE_US + 1;
  CHECK(camVzGlueScene(&d, &tracked, &resets) && !tracked, "a stale snapshot: not tracked");

  /* the controller's offset is the held one (biasHeldG: biasG itself unless the late offset noise is past its step) */
  portENTER_CRITICAL(&pubMux);
  snap.out.biasG = 0.001f;
  snap.out.biasHeldG = 0.003f;
  snap.pubUs = (uint64_t)nowUs;
  portEXIT_CRITICAL(&pubMux);
  camVzGlueVzInput(0.999f, &gi);
  CHECK(gi.camVzValid && fabsf(gi.camBias - 0.004f) < 1e-6f, "camBias = biasHeldG + 1 - g0 (%.4f)",
        (double)gi.camBias);
  portENTER_CRITICAL(&pubMux);
  snap.out.biasHeldG = NAN;
  portEXIT_CRITICAL(&pubMux);
  CHECK(unusable(), "a non-finite held offset: unusable");
  portENTER_CRITICAL(&pubMux);
  snap.out.biasHeldG = 0.001f;
  portEXIT_CRITICAL(&pubMux);

  /* a commanded motion reaches the observer before the block's prediction */
  block_t b;
  memset(&b, 0, sizeof(b));
  b.tEndUs = (uint64_t)nowUs;
  b.meanG = b.maxG = 1.0f;
  camVzGlueSetMotion(true);
  runBlock(&b);
  CHECK(sh->obs.moving, "a motion: the observer's flag set");
  camVzGlueSetMotion(false);
  b.tEndUs += 10000;
  runBlock(&b);
  CHECK(!sh->obs.moving, "no motion: cleared");

  /* NVS cvqlate / cvqlates at boot */
  CHECK(sh->obs.cfg.qLateMult == 1.0f && sh->obs.cfg.qLateS == 40.0f && !sh->lateCfgBad,
        "absent: x1 (off, the filter of before), 40 s");
  static const struct {
    int32_t q, qs;
    esp_err_t err;
    float mult, step;
    bool bad;
    const char *what;
  } nv[] = {
    {5, 40, ESP_OK, 5.0f, 40.0f, false, "x5 from 40 s"},
    {5, 120, ESP_OK, 5.0f, 120.0f, false, "x5 from 120 s (the upper limits)"},
    {1, 20, ESP_OK, 1.0f, 20.0f, false, "x1 from 20 s (the lower limits)"},
    {3, -1, ESP_OK, 3.0f, 40.0f, false, "x3, the step absent: 40 s"},
    {6, 40, ESP_OK, 1.0f, 40.0f, true, "x6 (above the x5 cap)"},
    {10, 40, ESP_OK, 1.0f, 40.0f, true, "x10 (the range of before the cap)"},
    {0, 40, ESP_OK, 1.0f, 40.0f, true, "x0"},
    {5, 19, ESP_OK, 1.0f, 40.0f, true, "19 s"},
    {5, 121, ESP_OK, 1.0f, 40.0f, true, "121 s"},
    {5, 40, ESP_ERR_NVS_TYPE_MISMATCH, 1.0f, 40.0f, true, "an item that cannot be read"},
  };
  for (size_t i = 0; i < sizeof(nv) / sizeof(nv[0]); i++) {
    sh = NULL;   /* a new boot (the old state leaks: a test) */
    task = NULL;
    statsSortie = 0;   /* the publish refreshes the flight statistics at the new sortie */
    nvsQLate = nv[i].q < 0 ? NVS_ABSENT : nv[i].q;
    nvsQLateS = nv[i].qs < 0 ? NVS_ABSENT : nv[i].qs;
    nvsErr = nv[i].err;
    camVzGlueInit();
    CHECK(sh != NULL && sh->obs.cfg.qLateMult == nv[i].mult && sh->obs.cfg.qLateS == nv[i].step &&
              sh->lateCfgBad == nv[i].bad && sh->obs.cfg.gyroOffsetUs == camOffsetUs &&
              camVzObsConfigValid(&sh->obs.cfg),
          "NVS %s: x%.0f from %.0f s, %s, the camera's offset kept (%u us)", nv[i].what,
          (double)sh->obs.cfg.qLateMult, (double)sh->obs.cfg.qLateS, sh->lateCfgBad ? "refused" : "taken",
          (unsigned)sh->obs.cfg.gyroOffsetUs);
    /* what 'camvz' and the flight summary show */
    sh->sortie = 1;
    publish();
    camVzGlueLate_t lt;
    fsumCamVz_t cvs;
    fsumCvLate_t late;
    uint32_t srt, age;
    CHECK(camVzGlueLate(&lt) && lt.qLateMult == nv[i].mult && lt.qLateS == nv[i].step && lt.cfgBad == nv[i].bad &&
              lt.state == CAM_VZ_LATE_OFF && camVzGlueFlightStats(&cvs, &late, &srt, &age) && srt == 1 &&
              late.qLate == (uint8_t)nv[i].mult &&
              late.qLateS == (uint8_t)nv[i].step && (late.flags & FSUM_CVL_RAN) &&
              !!(late.flags & FSUM_CVL_CFG_BAD) == nv[i].bad,
          "NVS %s: 'camvz' x%.0f %.0f s cfgBad %d, summary x%u %u s flags 0x%02x", nv[i].what, (double)lt.qLateMult,
          (double)lt.qLateS, lt.cfgBad, late.qLate, late.qLateS, late.flags);
  }
  nvsErr = ESP_OK;
  nvsQLate = nvsQLateS = NVS_ABSENT;

  /* NVS cvt2k / cvt2r at boot (the offset drift rate T2, 2026-10-16) */
  static const struct {
    int32_t k, r, q;
    esp_err_t err;
    float kR, prior;
    bool bad;
    const char *what;
  } t2v[] = {
    {NVS_ABSENT, NVS_ABSENT, NVS_ABSENT, ESP_OK, 0.0f, 0.124f, false, "absent: off, the prior 0.124"},
    {50, NVS_ABSENT, NVS_ABSENT, ESP_OK, 0.5f, 0.124f, false, "cvt2k 50: kR 0.5, the default prior"},
    {50, 60, NVS_ABSENT, ESP_OK, 0.5f, 0.060f, false, "cvt2k 50, cvt2r 60 (the G4 fallback prior)"},
    {200, -350, 1, ESP_OK, 2.0f, -0.350f, false, "cvt2k 200, cvt2r -350 (the limits), cvqlate 1"},
    {0, 124, 5, ESP_OK, 0.0f, 0.124f, false, "cvt2k 0 with cvqlate 5 (R1 alone)"},
    {50, 124, 5, ESP_OK, 0.0f, 0.124f, true, "cvt2k 50 with cvqlate 5: refused, R1 kept"},
    {201, 124, NVS_ABSENT, ESP_OK, 0.0f, 0.124f, true, "cvt2k 201"},
    {50, 351, NVS_ABSENT, ESP_OK, 0.0f, 0.124f, true, "cvt2r 351 (beyond the bound)"},
    {50, -351, NVS_ABSENT, ESP_OK, 0.0f, 0.124f, true, "cvt2r -351"},
    {50, 124, NVS_ABSENT, ESP_ERR_NVS_TYPE_MISMATCH, 0.0f, 0.124f, true, "an item that cannot be read"},
  };
  for (size_t i = 0; i < sizeof(t2v) / sizeof(t2v[0]); i++) {
    sh = NULL;
    task = NULL;
    statsSortie = 0;
    nvsT2K = t2v[i].k;
    nvsT2R = t2v[i].r;
    nvsQLate = t2v[i].q;
    nvsErr = t2v[i].err;
    camVzGlueInit();
    const float qWant = (t2v[i].q == NVS_ABSENT || t2v[i].err != ESP_OK) ? 1.0f : (float)t2v[i].q;
    CHECK(sh != NULL && sh->obs.cfg.t2kR == t2v[i].kR && fabsf(sh->obs.cfg.t2PriorMgS - t2v[i].prior) < 1e-6f &&
              sh->t2CfgBad == t2v[i].bad && sh->obs.cfg.qLateMult == qWant && camVzObsConfigValid(&sh->obs.cfg) &&
              sh->obs.cfg.gyroOffsetUs == camOffsetUs,
          "NVS %s: kR %.2f prior %+.3f, %s, R1 x%.0f", t2v[i].what, (double)sh->obs.cfg.t2kR,
          (double)sh->obs.cfg.t2PriorMgS, sh->t2CfgBad ? "refused" : "taken", (double)sh->obs.cfg.qLateMult);
    sh->sortie = 1;
    publish();
    camVzGlueT2_t t2;
    fsumCamVz_t cvs;
    fsumCvLate_t late;
    uint32_t srt, age;
    CHECK(camVzGlueT2(&t2) && t2.kR == t2v[i].kR && t2.cfgBad == t2v[i].bad && !t2.t2.on &&
              camVzGlueFlightStats(&cvs, &late, &srt, &age) &&
              late.t2K == (uint8_t)lrintf(t2v[i].kR * 100.0f) &&
              late.t2PriorUgs == (int16_t)lrintf(t2v[i].prior * 1000.0f) &&
              !!(late.flags & FSUM_CVL_T2_CFG_BAD) == t2v[i].bad && !(late.flags & FSUM_CVL_T2_ON),
          "NVS %s: 'camvz' kR %.2f cfgBad %d, summary k%u prior %d flags 0x%02x", t2v[i].what, (double)t2.kR,
          t2.cfgBad, late.t2K, late.t2PriorUgs, late.flags);
  }
  nvsErr = ESP_OK;
  nvsT2K = nvsT2R = nvsQLate = NVS_ABSENT;

  /* NVS cvhb at boot (the healthy offset speed-up, 2026-10-18) */
  static const struct {
    int32_t hb, q, k;
    esp_err_t err;
    float mult;
    bool bad;
    const char *what;
  } hbv[] = {
    {NVS_ABSENT, NVS_ABSENT, NVS_ABSENT, ESP_OK, 0.0f, false, "absent: off"},
    {0, NVS_ABSENT, NVS_ABSENT, ESP_OK, 0.0f, false, "cvhb 0: off"},
    {30, NVS_ABSENT, NVS_ABSENT, ESP_OK, 3.0f, false, "cvhb 30: x3 (the study's step)"},
    {10, NVS_ABSENT, NVS_ABSENT, ESP_OK, 1.0f, false, "cvhb 10: x1 (the lower limit)"},
    {80, NVS_ABSENT, NVS_ABSENT, ESP_OK, 8.0f, false, "cvhb 80: x8 (the upper limit)"},
    {50, 1, 0, ESP_OK, 5.0f, false, "cvhb 50 with cvqlate 1 and cvt2k 0 stored"},
    {5, NVS_ABSENT, NVS_ABSENT, ESP_OK, 0.0f, true, "cvhb 5 (under x1)"},
    {81, NVS_ABSENT, NVS_ABSENT, ESP_OK, 0.0f, true, "cvhb 81"},
    {30, 5, NVS_ABSENT, ESP_OK, 0.0f, true, "cvhb 30 with cvqlate 5: refused, R1 kept"},
    {30, NVS_ABSENT, 50, ESP_OK, 0.0f, true, "cvhb 30 with cvt2k 50: refused, T2 kept"},
    {30, NVS_ABSENT, NVS_ABSENT, ESP_ERR_NVS_TYPE_MISMATCH, 0.0f, true, "an item that cannot be read"},
  };
  for (size_t i = 0; i < sizeof(hbv) / sizeof(hbv[0]); i++) {
    sh = NULL;
    task = NULL;
    statsSortie = 0;
    nvsHb = hbv[i].hb;
    nvsQLate = hbv[i].q;
    nvsT2K = hbv[i].k;
    nvsErr = hbv[i].err;
    camVzGlueInit();
    const bool other = hbv[i].err == ESP_OK;   /* an unreadable store refuses every setting: R1 and T2 off too */
    const float qWant = other && hbv[i].q != NVS_ABSENT ? (float)hbv[i].q : 1.0f;
    const float kWant = other && hbv[i].k != NVS_ABSENT ? (float)hbv[i].k / 100.0f : 0.0f;
    CHECK(sh != NULL && sh->obs.cfg.hbMult == hbv[i].mult && sh->hbCfgBad == hbv[i].bad &&
              sh->obs.cfg.qLateMult == qWant && sh->obs.cfg.t2kR == kWant && camVzObsConfigValid(&sh->obs.cfg) &&
              sh->obs.cfg.gyroOffsetUs == camOffsetUs,
          "NVS %s: x%.1f, %s, R1 x%.0f, T2 k%.2f", hbv[i].what, (double)sh->obs.cfg.hbMult,
          sh->hbCfgBad ? "refused" : "taken", (double)sh->obs.cfg.qLateMult, (double)sh->obs.cfg.t2kR);
    sh->sortie = 1;
    publish();
    camVzGlueHb_t hb;
    fsumCamVz_t cvs;
    fsumCvLate_t late;
    uint32_t srt, age;
    CHECK(camVzGlueHb(&hb) && hb.mult == hbv[i].mult && hb.cfgBad == hbv[i].bad && !hb.on && hb.onMs == 0 &&
              camVzGlueFlightStats(&cvs, &late, &srt, &age) && !!(late.flags & FSUM_CVL_HB_ON) == (hbv[i].mult > 0.0f) &&
              !!(late.flags & FSUM_CVL_HB_CFG_BAD) == hbv[i].bad,
          "NVS %s: 'camvz' x%.1f cfgBad %d on %d, summary flags 0x%02x", hbv[i].what, (double)hb.mult, hb.cfgBad,
          hb.on, late.flags);
  }
  nvsErr = ESP_OK;
  nvsHb = nvsQLate = nvsT2K = NVS_ABSENT;
  CHECK(lockDepth == 0, "locks balanced");

  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
