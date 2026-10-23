/*
 * ESP-Drone Firmware
 *
 * flight_recorder.c - RAM flight recorder and latest-sample snapshot
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "flight_recorder.h"
#include "flight_rtc_log.h"
#include "camera_flow.h"
#include "camera_cadence.h"
#include "cam_vz_glue.h"
#include "pm_esplane.h"
#include "controller_pid.h"
#include "crtp_commander.h"
#include "attitude_controller.h"
#include "autonomy.h"
#include "motors.h"
#include "system.h"
#include "sensfusion6.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "bench_bypass.h"
#define DEBUG_MODULE "FREC"
#include "debug_cf.h"

#define POST_THRUST_SAMPLES FLIGHT_RECORDER_RATE_HZ  /* keep 1 s after thrust stops (and autonomy ends) */

_Static_assert(sizeof(flightRecord_t) == 126, "record size: update the sizes in flight_recorder.h");

static portMUX_TYPE recMux = portMUX_INITIALIZER_UNLOCKED;
static flightRecord_t *ring;
static uint32_t capacity;
static uint32_t head;      /* next write position */
static uint32_t count;
static bool frozen;
static bool writing;       /* a ring slot is being written outside the lock */
static uint8_t flightId;
static uint32_t samplesSinceThrust = POST_THRUST_SAMPLES;
static bool hasLatest;
static flightRecord_t latest;

/* Out of line on purpose (docs/autonomy-a2b.md 19.11): GCC inlined it at all 28 call sites, ~72 B each, so
 * flightRecorderSample was 3.3 KB of flash code run on the stabilizer every 10 ms. One body (the scale is
 * multiplied in the caller; this only compares and rounds) gives the same int16 for every input
 * (test_flight_recorder_host.c pins the records' hash). */
static __attribute__((noinline)) int16_t sat16(float v)
{
  if (!(v == v)) {
    return 0;
  }
  if (v > 32767.0f) {
    return 32767;
  }
  if (v < -32767.0f) {
    return -32767;
  }
  return (int16_t)lrintf(v);
}

void flightRecorderInit(void)
{
  if (ring) {
    return;
  }

  /* Prefer PSRAM (180 s, or 120 s where that would leave under 1 MB); fall back to a shorter ring in internal
   * RAM. */
  const size_t psramFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  capacity = psramFree >= (size_t)FLIGHT_RECORDER_SAMPLES_PSRAM * sizeof(flightRecord_t) + FLIGHT_RECORDER_PSRAM_KEEP
               ? FLIGHT_RECORDER_SAMPLES_PSRAM
               : FLIGHT_RECORDER_SAMPLES_PSRAM_LOW;
  ring = heap_caps_malloc(capacity * sizeof(flightRecord_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!ring) {
    capacity = FLIGHT_RECORDER_SAMPLES_INTERNAL;
    ring = heap_caps_malloc(capacity * sizeof(flightRecord_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (!ring) {
    capacity = 0;
    DEBUG_PRINTW("flight recorder disabled: no memory");
  } else {
    DEBUG_PRINTI("flight recorder: %u samples", (unsigned)capacity);
  }
}

void flightRecorderSample(const setpoint_t *setpoint, const state_t *state,
                          const sensorData_t *sensors, uint32_t tick)
{
  flightRecord_t r;
  float rollI, pitchI, rollRateI, pitchRateI;

  attitudeControllerGetIterms(&rollI, &pitchI, &rollRateI, &pitchRateI);

  r.tickMs = tick;
  r.flags = (systemIsArmed() ? FLIGHT_REC_FLAG_ARMED : 0) |
            (motorsIsInhibited() ? FLIGHT_REC_FLAG_INHIBITED : 0) |
            (setpoint->mode.roll == modeAbs ? FLIGHT_REC_FLAG_ANGLE : 0);
  r.thrust = (uint16_t)fminf(fmaxf(setpoint->thrust, 0.0f), 65535.0f);
  r.rollDes = sat16(setpoint->attitude.roll * 100.0f);
  r.pitchDes = sat16(setpoint->attitude.pitch * 100.0f);
  r.roll = sat16(state->attitude.roll * 100.0f);
  r.pitch = sat16(state->attitude.pitch * 100.0f);
  r.yaw = sat16(state->attitude.yaw * 100.0f);
  r.gyro[0] = sat16(sensors->gyro.x * 10.0f);
  r.gyro[1] = sat16(sensors->gyro.y * 10.0f);
  r.gyro[2] = sat16(sensors->gyro.z * 10.0f);
  r.acc[0] = sat16(sensors->acc.x * 1000.0f);
  r.acc[1] = sat16(sensors->acc.y * 1000.0f);
  r.acc[2] = sat16(sensors->acc.z * 1000.0f);
  for (int i = 0; i < 4; i++) {
    r.motor[i] = (uint16_t)motorsGetRatio(i);
  }
  r.rollI = sat16(rollI * 100.0f);
  r.pitchI = sat16(pitchI * 100.0f);
  r.rollRateI = sat16(rollRateI);
  r.pitchRateI = sat16(pitchRateI);
  r.yawRateDes = sat16(setpoint->attitudeRate.yaw * 10.0f);
  r.vbatMv = (uint16_t)fminf(fmaxf(pmGetBatteryVoltage() * 1000.0f, 0.0f), 65535.0f);

  {
    float hr, hp, cr, cp;
    bool eng;
    uint64_t ht;
    cameraFlowGetHold(&hr, &hp, &eng, &ht);
    controllerPidGetCameraCorrection(&cr, &cp);
    r.holdFlags = (cameraFlowHoldEnabled() ? 1 : 0) | (eng ? 2 : 0) | (cameraFlowHoldCoasting() ? 4 : 0) |
                  (cameraFlowHoldSceneGated() ? 8 : 0);
    r.holdRoll = sat16(cr * 100.0f);
    r.holdPitch = sat16(cp * 100.0f);
  }

  {
    rpytLinkStats_t link;
    crtpCommanderRpytGetLinkStats(&link);
    uint32_t nowMs = xTaskGetTickCount() * portTICK_PERIOD_MS;  /* same clock as lastPacketMs */
    r.rawThrust = link.lastRawThrust;
    r.linkAgeMs = link.packets ? (uint16_t)fminf((float)(nowMs - link.lastPacketMs), 65535.0f) : 65535;
    r.linkFlags = link.thrustLocked ? 1 : 0;
  }

  cameraFlowSample_t flow;
  if (cameraFlowGetLatest(&flow)) {
    uint64_t now = esp_timer_get_time();
    r.flowSeq = (uint8_t)(flow.seq % 255 + 1);
    /* Only a fresh sample counts (the camera may have stalled). */
    r.flowOk = (flow.ok && now >= flow.frameTimeUs && now - flow.frameTimeUs < CAM_CADENCE_SAMPLE_MAX_AGE_US) ? 1 : 0;
    r.flowLum = flow.lum;
    r.flowTexture = (uint8_t)fminf(fmaxf(flow.texture, 0.0f), 255.0f);
    r.flowResidual = (uint8_t)fminf(fmaxf(flow.residual * 10.0f, 0.0f), 255.0f);
    r.flowDt = (uint16_t)fminf(flow.dtUs / 100.0f, 65535.0f);
    r.flowAgeMs = (uint16_t)fminf((now > flow.frameTimeUs ? (float)(now - flow.frameTimeUs) : 0.0f) / 1000.0f, 65535.0f);
    r.flowDx = sat16(flow.dx * 100.0f);
    r.flowDy = sat16(flow.dy * 100.0f);
    r.flowScale = sat16(flow.scale * 1e4f);
    r.flowRot = sat16(flow.rot * 1e4f);
    for (int i = 0; i < 3; i++) {
      r.flowGyro[i] = sat16(flow.gyroAngle[i] * 1e4f);
    }
    if (!benchBypassOn(BENCH_BYPASS_REC)) {
      r.flowGain = flow.gainE3;
      r.flowSat = flow.satPct;
    } else {
      r.flowGain = 0;
      r.flowSat = 0;
    }
  } else {
    r.flowSeq = 0;
    r.flowOk = 0;
    r.flowLum = 0;
    r.flowTexture = 0;
    r.flowResidual = 0;
    r.flowDt = 0;
    r.flowAgeMs = 0;
    r.flowDx = r.flowDy = r.flowScale = r.flowRot = 0;
    r.flowGyro[0] = r.flowGyro[1] = r.flowGyro[2] = 0;
    r.flowGain = 0;
    r.flowSat = 0;
  }

  /* The thermal fields (docs/autonomy-a2b.md 19.3): inline reads of the driver's and the filter's published values,
   * no calls or locks (19.11); 0 with the bisect toggle (bench_bypass.h). */
  if (!benchBypassOn(BENCH_BYPASS_REC)) {
    float tc[3], lb[3], ta[3];
    sensorsMpu6050GetGyroTcCorr(tc);
    sensorsMpu6050GetAccTcCorr(ta);
    sensfusion6GetLearnedBiasDpsXY(lb);
    r.imuTemp = sat16(sensorsMpu6050GetTempC() * 100.0f);
    r.gtcCorrX = sat16(tc[0] * 1000.0f);
    r.gtcCorrY = sat16(tc[1] * 1000.0f);
    r.mkiBiasX = sat16(lb[0] * 1000.0f);
    r.mkiBiasY = sat16(lb[1] * 1000.0f);
    r.atcCorrZ = sat16(ta[2] * 100000.0f);   /* g -> 0.01 mg (docs/autonomy-a2b.md 36) */
  } else {
    r.imuTemp = r.gtcCorrX = r.gtcCorrY = r.mkiBiasX = r.mkiBiasY = 0;
    r.atcCorrZ = 0;
  }

  {
    camVzShadowRec_t cv;
    camVzGlueRecord(&cv);
    r.cvH = cv.hMm;
    r.cvV = cv.vMms;
    r.cvBias = cv.biasCmg;
    r.cvGain = cv.gainDpm;
    r.cvFlags = cv.flags;
    r.cvSigV = cv.sigVMms;
  }

  {
    autoRecFields_t af;
    autonomyRecordFields(&af);
    r.autoState = af.autoState;
    r.autoReason = af.autoReason;
    r.vzPhase = af.vzPhase;
    r.vzHmm = af.vzHmm;
    r.vzVmms = af.vzVmms;
    r.vzThrust = af.vzThrust;
    r.autoFlags = af.autoFlags;
    r.mvState = af.mvState;
    r.mvRefLat = af.mvRefLat;
    r.mvRefFwd = af.mvRefFwd;
    r.mvVRef = af.mvVRef;
    r.mvYawOff = af.mvYawOff;
  }

  /* A DRY sortie has no thrust: autonomy being active records too. */
  bool thrustOn = r.thrust > 0 || r.autoState != AUTO_IDLE;
  if (thrustOn && samplesSinceThrust >= POST_THRUST_SAMPLES) {
    flightId++;
  }
  samplesSinceThrust = thrustOn ? 0 : samplesSinceThrust + 1;
  r.flightId = flightId;
  flightRtcLogSample(&r);

  bool record = samplesSinceThrust <= POST_THRUST_SAMPLES;

  uint32_t slot = 0;
  portENTER_CRITICAL(&recMux);
  latest = r;
  hasLatest = true;
  record = record && !frozen && capacity > 0;
  if (record) {
    slot = head;
    writing = true;
  }
  portEXIT_CRITICAL(&recMux);

  if (record) {
    /* The ring may be in PSRAM: copy into it without holding the spinlock. */
    ring[slot] = r;
    portENTER_CRITICAL(&recMux);
    head = (slot + 1) % capacity;
    if (count < capacity) {
      count++;
    }
    writing = false;
    portEXIT_CRITICAL(&recMux);
  }
}

bool flightRecorderGetLatest(flightRecord_t *out)
{
  bool ok;
  portENTER_CRITICAL(&recMux);
  ok = hasLatest;
  *out = latest;
  portEXIT_CRITICAL(&recMux);
  return ok;
}

void flightRecorderFreeze(bool freeze)
{
  bool busy;
  portENTER_CRITICAL(&recMux);
  frozen = freeze;
  busy = writing;
  portEXIT_CRITICAL(&recMux);

  /* A write takes microseconds in the 1 kHz stabilizer task. */
  while (freeze && busy) {
    vTaskDelay(1);
    portENTER_CRITICAL(&recMux);
    busy = writing;
    portEXIT_CRITICAL(&recMux);
  }
}

uint32_t flightRecorderCount(void)
{
  uint32_t n;
  portENTER_CRITICAL(&recMux);
  n = count;
  portEXIT_CRITICAL(&recMux);
  return n;
}

uint32_t flightRecorderCapacity(void)
{
  return capacity;
}

bool flightRecorderGet(uint32_t index, flightRecord_t *out)
{
  uint32_t h, n;
  portENTER_CRITICAL(&recMux);
  h = head;
  n = count;
  portEXIT_CRITICAL(&recMux);
  if (index >= n) {
    return false;
  }
  /* Frozen: no writer touches the ring, so read it without the lock. */
  uint32_t oldest = (h + capacity - n) % capacity;
  *out = ring[(oldest + index) % capacity];
  return true;
}

void flightRecorderClear(void)
{
  portENTER_CRITICAL(&recMux);
  head = 0;
  count = 0;
  portEXIT_CRITICAL(&recMux);
}
