/*
 * ESP-Drone Firmware
 *
 * flight_rtc_log.c - Flight summaries and a pre-reset trace in RTC memory
 */
#include <math.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "crash_log.h"
#include "flight_rtc_log.h"
#include "ground_cool_glue.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#include "wifi_esp32.h"
#include "stabilizer.h"
#include "motors.h"
#include "camera_flow.h"
#include "takeoff_aux.h"
#include "controller_pid.h"
#include "cam_vz_glue.h"
#include "auto_fsm.h"
#include "bench_bypass.h"

#define FLIGHT_RTC_MAGIC 0xF1A6C0EAu   /* bump when rtcFlightStore_t changes (E4: fsumCamVz_t added; E5: imuT0Hc,
                                        * imuT1Hc, camAwakeDs, docs/autonomy-a2b.md 17.1; E6: gtcYEndMdps,
                                        * mkiEndMdps, 19.3; E7: gtcRes*, the post-flight re-zero, 19.1; E8:
                                        * camVzLate, the observer's late offset noise, 25; E9: camVzLate's T2
                                        * fields, the offset drift rate, 29) */
#define VBAT_MON_STACK   2048
#define VBAT_MON_PRI     1
#define VBAT_WIN_SAMPLES 10            /* 1 kHz samples per window */

typedef struct {
  uint32_t magic;
  uint32_t total;                 /* summaries ever started; newest is total-1 */
  flightSummary_t flights[FLIGHT_RTC_LOG_FLIGHTS];
  bool open;                      /* flights[(total-1) % N] is still in progress */
  uint8_t sampleDiv;
  uint32_t crumbHead;             /* live trace, samples ever written */
  fsumCrumb_t crumbs[FLIGHT_RTC_LOG_CRUMBS];
  uint32_t snapCount;             /* trace copied at the last interrupted flight */
  uint16_t snapBoot;
  int32_t snapReason;
  fsumCrumb_t snap[FLIGHT_RTC_LOG_CRUMBS];
  uint16_t vCurMin;               /* running minimum of the window in progress */
  uint8_t vCurN;
  uint32_t vHead;                 /* complete windows ever written */
  uint16_t vMin[FLIGHT_RTC_LOG_VWIN];
  uint16_t rCurMin;               /* rail meter, same windows (0: no reading) */
  uint16_t rMin[FLIGHT_RTC_LOG_VWIN];
  uint32_t snapVCount;
  uint16_t snapV[FLIGHT_RTC_LOG_VWIN];
  uint16_t snapR[FLIGHT_RTC_LOG_VWIN];
} rtcFlightStore_t;

static RTC_NOINIT_ATTR rtcFlightStore_t store;
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t bootId;
static uint8_t lastFlightId;
static uint16_t vMinSinceCrumb;   /* battery minimum since the last trace sample */
static uint16_t rMinSinceCrumb;   /* rail minimum since the last trace sample */
static uint16_t lastVbatMv, lastRailMv;
/* Wi-Fi counters at the start of the open flight / at the last trace sample. */
static uint32_t txAtStart, rxAtStart, txAtCrumb, rxAtCrumb;
/* The camera height observer's first sortie that belongs to the open flight
 * (a ground start at or after its first thrust). */
static uint32_t cvSortieMin;
/* The observer runs in its own task on 10 ms blocks: it registers the
 * touchdown a few tens of ms after the stop this loop sees. So the closed
 * flight keeps taking its fields for this many samples (100 Hz), or until
 * they show the touchdown. */
#define CV_LATE_SAMPLES 30
static uint8_t cvLate;
/* The previous sample's autonomy state and reason (FSUM_END_AUTO). */
static uint8_t prevAutoState, prevAutoReason;
/* The driver's airborne re-zero count already seen: a new one is the first
 * ground re-zero after the newest flight (sensorsMpu6050GetGyroTcAir()). */
static uint32_t gtcAirSeqSeen;
static uint32_t atcAirSeqSeen;   /* the accelerometer check's (docs/autonomy-a2b.md 36.2) */
static bool isInit;

/* IMU temperature in 0.5 C (flight_summary.h imuT0Hc / imuT1Hc). */
static int8_t tempHalfC(float c)
{
  if (!isfinite(c)) {
    return FSUM_TEMP_NONE;
  }
  const long v = lroundf(c * 2.0f);
  return (int8_t)(v < -127 ? -127 : (v > 127 ? 127 : v));
}

/* 0.1 s from ms, FSUM_AGE_NONE for UINT32_MAX, saturating below it. */
static uint16_t dsOf(uint32_t ms)
{
  if (ms == UINT32_MAX) {
    return FSUM_AGE_NONE;
  }
  const uint32_t ds = ms / 100u;
  return ds >= FSUM_AGE_NONE ? FSUM_AGE_NONE - 1u : (uint16_t)ds;
}

static flightSummary_t *newest(void)
{
  return &store.flights[(store.total - 1) % FLIGHT_RTC_LOG_FLIGHTS];
}

static uint16_t toMv(float v)
{
  return (uint16_t)fminf(fmaxf(v * 1000.0f, 0.0f), 65535.0f);
}

/* Keep the lower of two readings where 0 means "none yet". */
static inline uint16_t minMv(uint16_t cur, uint16_t mv)
{
  return (mv && (cur == 0 || mv < cur)) ? mv : cur;
}

/* 1 kHz single-conversion battery reads: the 10 Hz power-management reading
 * averages 30 conversions and cannot show a dip of a few milliseconds. The
 * diagnostic rail meter (XIAO 3.3 V, GPIO9) is read right after each one. */
static void vbatMonitorTask(void *param)
{
  while (1) {
    /* Discard one battery conversion first: the previous conversion was the
     * rail channel (a pad driven to ~3.1 V), and charge left on the sampling
     * capacitor could pull a high-impedance divider reading up. */
    (void)pmMeasureExtBatteryVoltageFast();
    uint16_t mv = toMv(pmMeasureExtBatteryVoltageFast());
    if (mv == 0) {
      vTaskDelay(M2T(100));      /* ADC or battery pin not set up yet */
      continue;
    }
    uint16_t rmv = toMv(pmMeasureRailVoltageFast());   /* 0 without the meter */
    portENTER_CRITICAL(&logMux);
    if (store.vCurN == 0) {
      store.vCurMin = mv;
      store.rCurMin = rmv;
    } else {
      store.vCurMin = minMv(store.vCurMin, mv);
      store.rCurMin = minMv(store.rCurMin, rmv);
    }
    if (++store.vCurN >= VBAT_WIN_SAMPLES) {
      store.vMin[store.vHead % FLIGHT_RTC_LOG_VWIN] = store.vCurMin;
      store.rMin[store.vHead % FLIGHT_RTC_LOG_VWIN] = store.rCurMin;
      store.vHead++;
      store.vCurN = 0;
    }
    vMinSinceCrumb = minMv(vMinSinceCrumb, mv);
    rMinSinceCrumb = minMv(rMinSinceCrumb, rmv);
    if (store.open && store.total > 0) {
      flightSummary_t *fs = newest();
      fs->vbatFastMinMv = minMv(fs->vbatFastMinMv, mv);
      fs->railMinMv = minMv(fs->railMinMv, rmv);
    }
    lastVbatMv = mv;
    lastRailMv = rmv;
    portEXIT_CRITICAL(&logMux);
    vTaskDelay(1);
  }
}

/* Time since an uptime stamp (ms) in 0.1 s, FSUM_AGE_NONE if never stamped. */
static uint16_t ageDs(uint32_t nowMs, uint32_t stampMs)
{
  if (stampMs == 0 || nowMs < stampMs) {
    return FSUM_AGE_NONE;
  }
  uint32_t ds = (nowMs - stampMs) / 100;
  return ds >= FSUM_AGE_NONE ? FSUM_AGE_NONE - 1 : (uint16_t)ds;
}

static uint16_t sat16u(uint32_t v)
{
  return v > 0xFFFF ? 0xFFFF : (uint16_t)v;
}

static uint8_t sat8u(uint32_t v)
{
  return v > 0xFF ? 0xFF : (uint8_t)v;
}

/* A flight the state machine flew ends FSUM_END_AUTO with its reason (the
 * LAND or KILLED trigger) when the closing sample, or the one before it, was
 * not IDLE. r: the closing sample, or NULL when the previous one closed it.
 * Called under logMux. */
static bool endAuto(flightSummary_t *fs, const flightRecord_t *r)
{
  if (r != NULL && r->autoState != AUTO_IDLE) {
    fs->end = FSUM_END_AUTO;
    fs->autoReason = r->autoReason;
    return true;
  }
  if (prevAutoState != AUTO_IDLE) {
    fs->end = FSUM_END_AUTO;
    fs->autoReason = prevAutoReason;
    return true;
  }
  return false;
}

void flightRtcLogInit(void)
{
  if (isInit) {
    return;
  }
  esp_reset_reason_t reason = esp_reset_reason();
  /* Only the magic tells a real power-up (RTC memory lost) apart: a supply
   * collapse to the power-on-reset level also reports ESP_RST_POWERON on the
   * S3 while RTC memory survives, and that is exactly the event to keep. */
  if (store.magic != FLIGHT_RTC_MAGIC || store.snapCount > FLIGHT_RTC_LOG_CRUMBS) {
    memset(&store, 0, sizeof(store));   /* RTC no-init memory is garbage after power-on */
    store.magic = FLIGHT_RTC_MAGIC;
  } else if (store.open && store.total > 0) {
    /* The previous boot reset in the middle of a flight. */
    flightSummary_t *fs = newest();
    uint32_t nv = store.vHead < FLIGHT_RTC_LOG_VWIN - 1 ? store.vHead : FLIGHT_RTC_LOG_VWIN - 1;
    for (uint32_t i = 0; i < nv; i++) {
      store.snapV[i] = store.vMin[(store.vHead - nv + i) % FLIGHT_RTC_LOG_VWIN];
      store.snapR[i] = store.rMin[(store.vHead - nv + i) % FLIGHT_RTC_LOG_VWIN];
    }
    if (store.vCurN > 0) {
      store.snapV[nv] = store.vCurMin;   /* the window the reset cut short */
      store.snapR[nv] = store.rCurMin;
      nv++;
    }
    store.snapVCount = nv;
    fs->end = FSUM_END_RESET;
    fs->resetReason = (uint8_t)reason;
    uint32_t n = store.crumbHead < FLIGHT_RTC_LOG_CRUMBS ? store.crumbHead : FLIGHT_RTC_LOG_CRUMBS;
    for (uint32_t i = 0; i < n; i++) {
      store.snap[i] = store.crumbs[(store.crumbHead - n + i) % FLIGHT_RTC_LOG_CRUMBS];
    }
    store.snapCount = n;
    store.snapBoot = fs->boot;
    store.snapReason = reason;
  }
  store.open = false;
  store.crumbHead = 0;
  store.sampleDiv = 0;
  store.vHead = 0;
  store.vCurN = 0;
  if (store.snapVCount > FLIGHT_RTC_LOG_VWIN) {
    store.snapVCount = 0;
  }

  crashBoot_t b;
  uint32_t boots = 0;
  crashLogBoots(&b, 1, &boots);
  bootId = (uint16_t)boots;
  isInit = true;
  xTaskCreatePinnedToCore(vbatMonitorTask, "VBATMON", VBAT_MON_STACK, NULL, VBAT_MON_PRI, NULL, 0);
}

void flightRtcLogSample(const flightRecord_t *r)
{
  if (!isInit) {
    return;
  }
  fsumSample_t s = {
    .thrust = r->thrust,
    .vbatMv = r->vbatMv,
    .roll = r->roll,
    .pitch = r->pitch,
    .gx = r->gyro[0],
    .gy = r->gyro[1],
    .motor = {r->motor[0], r->motor[1], r->motor[2], r->motor[3]},
    .holdEngaged = (r->holdFlags & 2) != 0,
    .holdCoasting = (r->holdFlags & 4) != 0,
    .flowOk = r->flowOk != 0,
    .thrustLocked = (r->linkFlags & 1) != 0,
    .holdRoll = r->holdRoll,
    .holdPitch = r->holdPitch,
    .rollRateI = r->rollRateI,
    .pitchRateI = r->pitchRateI,
    .linkAgeMs = r->linkAgeMs,
  };

  uint32_t tx, rx, joinMs, appMs;
  wifiGetCounters(&tx, &rx, &joinMs, &appMs);
  const uint32_t nowMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
  uint32_t gapMaxUs, gaps2500;
  stabilizerTakeLoopGaps(&gapMaxUs, &gaps2500);   /* we run in the stabilizer task */
  /* Camera height observer (same task), read outside the lock: its state may
   * be in PSRAM. A flight starting now owns the current sortie if its ground
   * start came within the last 50 ms (the recorder samples at 100 Hz; a new
   * flight follows >= 1 s of zero thrust, a ground start >= 0.5 s at rest),
   * else only a later one (no ground start, e.g. a hand launch: none). */
  fsumCamVz_t cv;
  uint32_t cvSortie = 0, cvAgeMs = UINT32_MAX;
  fsumCvLate_t cvl;
  const bool haveCv = camVzGlueFlightStats(&cv, &cvl, &cvSortie, &cvAgeMs);
  const uint32_t cvFirstIfNew = !haveCv ? UINT32_MAX : (cvAgeMs <= 50u ? cvSortie : cvSortie + 1u);
  /* The gyro compensation's post-flight residual (outside the lock: the driver's own lock), read only when the
   * driver's airborne re-zero count moved (an inline load first: about once a flight; docs/autonomy-a2b.md 19.11).
   * The bisect toggle skips the poll (bench_bypass.h). */
  sensorsGyroTcRezero_t gtcRz = {0};
  float gtcDt = 0.0f;
  uint32_t gtcLandMs = 0;
  const uint32_t gtcSeq = !benchBypassOn(BENCH_BYPASS_REC) && sensorsMpu6050GyroTcAirSeq() != gtcAirSeqSeen
                              ? sensorsMpu6050GetGyroTcAir(gtcAirSeqSeen, &gtcRz, &gtcDt, &gtcLandMs)
                              : gtcAirSeqSeen;
  sensorsAccTcRezero_t atcRz = {0};
  float atcDt = 0.0f;
  uint32_t atcLandMs = 0;
  const uint32_t atcSeq = !benchBypassOn(BENCH_BYPASS_REC) && sensorsMpu6050AccTcAirSeq() != atcAirSeqSeen
                              ? sensorsMpu6050GetAccTcAir(atcAirSeqSeen, &atcRz, &atcDt, &atcLandMs)
                              : atcAirSeqSeen;

  portENTER_CRITICAL(&logMux);
  if (gtcSeq != gtcAirSeqSeen) {
    /* The first ground re-zero after an airborne stretch (>= 2 s after the motors stopped, so the newest summary
     * is closed): it belongs to the newest flight if that one is this boot's and flew. */
    if (!store.open && store.total > 0 && newest()->boot == bootId && newest()->airCs > 0) {
      fsumSetGtcResidual(newest(), (int32_t)gtcRz.stepMdps[0] - gtcRz.corrMdps[0],
                         (int32_t)gtcRz.stepMdps[1] - gtcRz.corrMdps[1],
                         isfinite(gtcDt) ? (int32_t)lroundf(gtcDt * 100.0f) : 0, gtcLandMs,
                         (gtcRz.flags & SENSORS_GTC_RZ_ON) != 0);
    }
    gtcAirSeqSeen = gtcSeq;
  }
  if (atcSeq != atcAirSeqSeen) {
    if (!store.open && store.total > 0 && newest()->boot == bootId && newest()->airCs > 0) {
      fsumSetAtcResidual(newest(), atcRz.stepCmg[2], atcRz.corrCmg[2],
                         isfinite(atcDt) ? (int32_t)lroundf(atcDt * 100.0f) : 0, (atcRz.flags & ACC_TC_RZ_ON) != 0);
    }
    atcAirSeqSeen = atcSeq;
  }
  if (s.thrust > 0) {
    cvLate = 0;
    if (!store.open && store.total > 0 && r->flightId == lastFlightId &&
        newest()->boot == bootId && newest()->end == FSUM_END_ZERO) {
      /* Thrust dipped to zero for less than the recorder's flight gap. */
      store.open = true;
      newest()->end = FSUM_END_OPEN;
      txAtCrumb = tx;
      rxAtCrumb = rx;
    }
    if (!store.open || r->flightId != lastFlightId) {
      if (store.open && !endAuto(newest(), NULL)) {
        newest()->end = FSUM_END_ZERO;   /* a new flight id: the last one ended */
      }
      store.total++;
      vMinSinceCrumb = 0;
      rMinSinceCrumb = 0;
      fsumStart(newest(), bootId, r->flightId, s.vbatMv);
      newest()->uptimeS = sat16u(nowMs / 1000);
      newest()->pwmKhz = sat8u(motorsGetPwmFreq() / 1000);
      newest()->motorFlags = (stabilizerSoftStartEnabled() ? FSUM_MOTOR_SOFTSTART : 0) |
                             (cameraFlowStandbyArmed() ? FSUM_MOTOR_CAMSB : 0) |
                             (takeoffAuxTxLowEnabled() ? FSUM_MOTOR_TXLO : 0) |
                             (wifiQuiet() ? FSUM_MOTOR_WIFIQ : 0) |
                             (stabilizerTakeoffCold() ? FSUM_MOTOR_COLD : 0) |
                             (groundCoolGlueRunning() ? FSUM_MOTOR_COOL : 0);
      {
        float tr, tp;
        int load;
        controllerPidGetAttitudeTrim(&tr, &tp, &load);
        newest()->atrimRoll = (int16_t)lroundf(tr * 100.0f);
        newest()->atrimPitch = (int16_t)lroundf(tp * 100.0f);
      }
      newest()->sinceJoinDs = ageDs(nowMs, joinMs);
      newest()->sinceAppDs = ageDs(nowMs, appMs);
      newest()->imuT0Hc = tempHalfC(sensorsMpu6050GetTempC());
      newest()->imuT1Hc = newest()->imuT0Hc;
      {
        cameraFlowGroundStats_t g;
        cameraFlowGroundStats(&g);
        newest()->camAwakeDs = dsOf(g.awakeMs);
      }
      txAtStart = txAtCrumb = tx;
      rxAtStart = rxAtCrumb = rx;
      uint32_t rezeros;
      float drift[3];
      sensorsMpu6050GetGyroRezero(&rezeros, drift);
      newest()->gyroDrift[0] = (int16_t)fminf(fmaxf(drift[0] * 100.0f, -32767.0f), 32767.0f);
      newest()->gyroDrift[1] = (int16_t)fminf(fmaxf(drift[1] * 100.0f, -32767.0f), 32767.0f);
      store.open = true;
      lastFlightId = r->flightId;
      cvSortieMin = cvFirstIfNew;
    }
    fsumAdd(newest(), &s);
    if (haveCv && cvSortieMin != UINT32_MAX && cvSortie >= cvSortieMin) {
      newest()->camVz = cv;
      newest()->camVzLate = cvl;
    }
    if (s.thrust >= FSUM_AIRBORNE_THRUST) {
      newest()->imuT1Hc = tempHalfC(sensorsMpu6050GetTempC());   /* the end value is the last airborne one */
      newest()->gtcYEndMdps = r->gtcCorrY;
      newest()->mkiEndMdps[0] = r->mkiBiasX;
      newest()->mkiEndMdps[1] = r->mkiBiasY;
    }
    if (cameraFlowInStandby()) {
      newest()->motorFlags |= FSUM_MOTOR_CAMSB_DID;
    }
    if (wifiTxLow()) {
      newest()->motorFlags |= FSUM_MOTOR_TXLO_DID;
    }
    if (gapMaxUs > newest()->loopGapMaxUs) {
      newest()->loopGapMaxUs = sat16u(gapMaxUs);
    }
    newest()->loopGaps2500 = sat16u(newest()->loopGaps2500 + gaps2500);
    newest()->txPackets = sat16u(tx - txAtStart);
    newest()->rxPackets = sat16u(rx - rxAtStart);
    if (store.sampleDiv++ % 4 == 0) {
      fsumCrumb_t *c = &store.crumbs[store.crumbHead % FLIGHT_RTC_LOG_CRUMBS];
      fsumCrumb(c, &s);
      if (vMinSinceCrumb) {
        c->vbatMv = vMinSinceCrumb;   /* fast minimum over the last 40 ms */
        vMinSinceCrumb = 0;
      }
      if (rMinSinceCrumb) {
        c->rail10mv = fsumRailCode(rMinSinceCrumb);
        rMinSinceCrumb = 0;
      }
      c->flags |= (cameraFlowInStandby() ? FSUM_CRUMB_CAMSB : 0) | (wifiTxLow() ? FSUM_CRUMB_TXLO : 0);
      c->txPk = sat8u(tx - txAtCrumb);
      c->rxPk = sat8u(rx - rxAtCrumb);
      txAtCrumb = tx;
      rxAtCrumb = rx;
      store.crumbHead++;
    }
  } else if (store.open) {
    if (haveCv && cvSortieMin != UINT32_MAX && cvSortie >= cvSortieMin) {
      newest()->camVz = cv;
      newest()->camVzLate = cvl;
    }
    if (!endAuto(newest(), r)) {
      newest()->end = fsumEndCause(&s);
    }
    store.open = false;
    cvLate = CV_LATE_SAMPLES;
  } else if (cvLate > 0) {
    cvLate--;
    if (newest()->boot == bootId && haveCv && cvSortieMin != UINT32_MAX && cvSortie >= cvSortieMin) {
      newest()->camVz = cv;
      newest()->camVzLate = cvl;
      if (cv.flags & FSUM_CVZ_TD) {
        cvLate = 0;
      }
    }
  }
  prevAutoState = r->autoState;
  prevAutoReason = r->autoReason;
  portEXIT_CRITICAL(&logMux);
}

int flightRtcLogFlightCount(uint32_t *total)
{
  portENTER_CRITICAL(&logMux);
  *total = store.total;
  uint32_t n = store.total < FLIGHT_RTC_LOG_FLIGHTS ? store.total : FLIGHT_RTC_LOG_FLIGHTS;
  portEXIT_CRITICAL(&logMux);
  return (int)n;
}

bool flightRtcLogFlight(int index, flightSummary_t *out)
{
  portENTER_CRITICAL(&logMux);
  uint32_t n = store.total < FLIGHT_RTC_LOG_FLIGHTS ? store.total : FLIGHT_RTC_LOG_FLIGHTS;
  bool ok = index >= 0 && (uint32_t)index < n;
  if (ok) {
    *out = store.flights[(store.total - n + index) % FLIGHT_RTC_LOG_FLIGHTS];
  }
  portEXIT_CRITICAL(&logMux);
  return ok;
}

int flightRtcLogResetTraceInfo(uint16_t *boot, int32_t *resetReason)
{
  portENTER_CRITICAL(&logMux);
  int n = (int)store.snapCount;
  *boot = store.snapBoot;
  *resetReason = store.snapReason;
  portEXIT_CRITICAL(&logMux);
  return n;
}

bool flightRtcLogResetTraceGet(int index, fsumCrumb_t *out)
{
  portENTER_CRITICAL(&logMux);
  bool ok = index >= 0 && (uint32_t)index < store.snapCount;
  if (ok) {
    *out = store.snap[index];
  }
  portEXIT_CRITICAL(&logMux);
  return ok;
}

int flightRtcLogResetVbat(uint16_t *out, int max)
{
  portENTER_CRITICAL(&logMux);
  int n = (int)store.snapVCount < max ? (int)store.snapVCount : max;
  memcpy(out, store.snapV, n * sizeof(uint16_t));
  portEXIT_CRITICAL(&logMux);
  return n;
}

int flightRtcLogResetRail(uint16_t *out, int max)
{
  portENTER_CRITICAL(&logMux);
  int n = (int)store.snapVCount < max ? (int)store.snapVCount : max;
  memcpy(out, store.snapR, n * sizeof(uint16_t));
  portEXIT_CRITICAL(&logMux);
  return n;
}

/* IRAM with flightRtcLogLiveVbatMin(): the autonomy glue's slow inputs read
 * it every 10th stabilizer loop in a sortie (autonomy.c). */
void IRAM_ATTR flightRtcLogLive(uint16_t *vbatNow, uint16_t *vbatMin, uint16_t *railNow, uint16_t *railMin)
{
  uint16_t vm = 0, rm = 0;
  portENTER_CRITICAL(&logMux);
  uint32_t n = store.vHead < FLIGHT_RTC_LOG_VWIN ? store.vHead : FLIGHT_RTC_LOG_VWIN;
  for (uint32_t i = 0; i < n; i++) {
    vm = minMv(vm, store.vMin[(store.vHead - 1 - i) % FLIGHT_RTC_LOG_VWIN]);
    rm = minMv(rm, store.rMin[(store.vHead - 1 - i) % FLIGHT_RTC_LOG_VWIN]);
  }
  *vbatNow = lastVbatMv;
  *railNow = lastRailMv;
  portEXIT_CRITICAL(&logMux);
  *vbatMin = vm;
  *railMin = rm;
}

uint16_t IRAM_ATTR flightRtcLogLiveVbatMin(void)
{
  uint16_t vn, vm, rn, rm;
  flightRtcLogLive(&vn, &vm, &rn, &rm);
  return vm;
}

void flightRtcLogClear(void)
{
  portENTER_CRITICAL(&logMux);
  bool open = store.open;
  flightSummary_t cur = *newest();
  store.total = 0;
  store.snapCount = 0;
  store.snapVCount = 0;
  store.crumbHead = 0;
  if (open) {
    store.total = 1;               /* keep the flight in progress */
    store.flights[0] = cur;
  }
  portEXIT_CRITICAL(&logMux);
}
