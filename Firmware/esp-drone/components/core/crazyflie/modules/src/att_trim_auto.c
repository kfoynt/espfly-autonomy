/*
 * ESP-Drone Firmware
 *
 * att_trim_auto.c - Learn the attitude trim from the camera drift hold
 *
 * Follows the flight recorder's latest sample (100 Hz). A flight runs from
 * thrust leaving zero to thrust back at zero; at its end the rule in
 * att_trim_learn.c may move part of the hold's steady correction into the
 * attitude trim, which applies from the next flight on. The trim is saved to
 * NVS only after the drone has rested on the ground for a few seconds: a
 * flash write briefly stalls the other tasks. Samples count only with a fresh
 * camera fit (a stalled camera leaves the hold flags latched), outside the
 * take-off window (camera standby, lift-off kick) and never during capped
 * ground tests. A hand-set, unsaved trim (atrim set) pauses learning until
 * atrim save or atrim clear, so the learner never stores a trial value.
 */
#include "att_trim_auto.h"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "att_trim_learn.h"
#include "autonomy.h"
#include "controller_pid.h"
#include "flight_recorder.h"
#include "imu_level_cal_store.h"
#include "stabilizer.h"
#include "torque_trim.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "ATLEARN"
#include "debug_cf.h"

#define ATT_AUTO_AIRBORNE_THRUST 30000   /* as the camera hold */
#define ATT_AUTO_SAVE_REST_MS    3000
#define ATT_AUTO_SAVE_RETRY_MS   30000

static volatile bool learnOn = true;
static bool flightOpen;
static uint32_t lastSampleMs;
static bool haveSample;
static attLearnFlight_t flight;
static uint32_t zeroSinceMs;
static bool zeroSeen;
static volatile bool savePending;
static uint32_t saveNotBeforeMs;
static volatile uint32_t learnedFlights;
static volatile float lastMeanRoll, lastMeanPitch;
static volatile float learnedRoll, learnedPitch;   /* the trim the learner last set */
static volatile bool manualLiveOnly;
/* How the open flight ended (attLearnEndOk): autonomy seen in any of its
 * records, and the latest record's state and reason (the thrust-0 record
 * that closes it included). */
static bool flightAuto;
static uint8_t flightEndState, flightEndReason;

void attTrimAutoInit(void)
{
  bool on = true;
  if (attTrimLearnStoreLoad(&on) == ESP_OK) {
    learnOn = on;
  }
}

static void finishFlight(void)
{
  if (!learnOn || manualLiveOnly) {
    return;
  }
  const uint16_t events = flightAuto ? autonomySortieEvents() : 0;   /* the sortie's (until the next ARM) */
  if (!attLearnEndOk(flightAuto, flightEndState, flightEndReason, events)) {
    DEBUG_PRINTI("hold mean not learned: sortie ended %s %s, %u events", autoStateName((autoState_t)flightEndState),
                 autoReasonName((autoReason_t)flightEndReason), (unsigned)events);
    return;
  }
  float trimRoll, trimPitch;
  int load;
  controllerPidGetAttitudeTrim(&trimRoll, &trimPitch, &load);
  const float oldRoll = trimRoll, oldPitch = trimPitch;
  float meanRoll, meanPitch;
  if (!attLearnFinish(&flight, FLIGHT_RECORDER_RATE_HZ, &trimRoll, &trimPitch, &meanRoll, &meanPitch)) {
    return;
  }
  if (!controllerPidSetAttitudeTrim(trimRoll, trimPitch)) {
    return;
  }
  learnedRoll = trimRoll;
  learnedPitch = trimPitch;
  lastMeanRoll = meanRoll;
  lastMeanPitch = meanPitch;
  learnedFlights++;
  savePending = true;
  DEBUG_PRINTI("hold mean %.2f,%.2f deg: attitude trim %.2f,%.2f -> %.2f,%.2f", (double)meanRoll,
               (double)meanPitch, (double)oldRoll, (double)oldPitch, (double)trimRoll, (double)trimPitch);
}

static void maybeSave(uint32_t nowMs)
{
  /* No flash write while autonomy is active (docs/autonomy-a2b.md 7.4):
   * the save waits for the end of the sortie. */
  if (!savePending || autonomyActive() || !zeroSeen || (uint32_t)(nowMs - zeroSinceMs) < ATT_AUTO_SAVE_REST_MS ||
      (int32_t)(nowMs - saveNotBeforeMs) < 0 || !stabilizerOnGround() || stabilizerTakeoffWindow()) {
    return;
  }
  float r, p;
  int load;
  controllerPidGetAttitudeTrim(&r, &p, &load);
  /* Store only what the learner produced, and only while it is on: a console
   * set/clear or learn off in the meantime cancels the save. */
  if (!learnOn || manualLiveOnly || r != learnedRoll || p != learnedPitch) {
    savePending = false;
    return;
  }
  attTrimRecord_t rec;
  attTrimRecordInit(&rec, r, p);
  esp_err_t err = attTrimStoreSave(&rec);
  if (err == ESP_OK) {
    savePending = false;
  } else {
    /* ESP_ERR_INVALID_STATE: the flash interlock refused it (an ARM came
     * first); savePending stays, the save follows the sortie. */
    if (err != ESP_ERR_INVALID_STATE) {
      DEBUG_PRINTW("learned attitude trim not saved (%s); retrying", esp_err_to_name(err));
    }
    saveNotBeforeMs = nowMs + ATT_AUTO_SAVE_RETRY_MS;
  }
}

void attTrimAutoPoll(void)
{
  const uint32_t nowMs = T2M(xTaskGetTickCount());
  flightRecord_t r;
  if (flightRecorderGetLatest(&r) && (!haveSample || r.tickMs != lastSampleMs)) {
    haveSample = true;
    lastSampleMs = r.tickMs;
    if (r.thrust > 0) {
      zeroSeen = false;
      if (!flightOpen) {
        flightOpen = true;
        flightAuto = false;
        attLearnStart(&flight, FLIGHT_RECORDER_RATE_HZ);
      }
      flightAuto = flightAuto || r.autoState != AUTO_IDLE;
      flightEndState = r.autoState;
      flightEndReason = r.autoReason;
      const uint8_t mv = r.mvState & 0x0F;   /* autoMvState_t (the kind in bits 4-7) */
      const float gx = fabsf(r.gyro[0] / 10.0f), gy = fabsf(r.gyro[1] / 10.0f);
      const bool applied = (r.holdFlags & 2) != 0;
      const bool coasting = (r.holdFlags & 4) != 0;
      attLearnSample_t s = {
        /* Engaged without a fresh fit is a stalled camera, not a hold. */
        .holdApplied = applied && (coasting || r.flowOk),
        .holdOnView = applied && !coasting && r.flowOk,
        .sticksCentred = r.rollDes == 0 && r.pitchDes == 0,
        .airborne = r.thrust > ATT_AUTO_AIRBORNE_THRUST && !stabilizerTakeoffWindow() &&
                    stabilizerGetThrustCap() == 0,
        .corrRollDeg = r.holdRoll / 100.0f,
        .corrPitchDeg = r.holdPitch / 100.0f,
        .rollDeg = r.roll / 100.0f,
        .pitchDeg = r.pitch / 100.0f,
        .gyroDps = gx > gy ? gx : gy,
        .motionActive = mv == AUTO_MV_RUN || mv == AUTO_MV_PAUSED || mv == AUTO_MV_SETTLE,
      };
      attLearnAdd(&flight, &s);
    } else {
      if (!zeroSeen) {
        zeroSeen = true;
        zeroSinceMs = nowMs;
      }
      if (flightOpen) {
        flightOpen = false;
        flightAuto = flightAuto || r.autoState != AUTO_IDLE;   /* the record that closes it */
        flightEndState = r.autoState;
        flightEndReason = r.autoReason;
        finishFlight();
      }
    }
  }
  maybeSave(nowMs);
}

void attTrimAutoNoteManual(bool liveOnly)
{
  manualLiveOnly = liveOnly;
  savePending = false;
}

bool attTrimAutoManualLiveOnly(void)
{
  return manualLiveOnly;
}

void attTrimAutoSetEnabled(bool on)
{
  learnOn = on;
}

bool attTrimAutoEnabled(void)
{
  return learnOn;
}

void attTrimAutoStats(uint32_t *flights, float *meanRoll, float *meanPitch, bool *pending)
{
  *flights = learnedFlights;
  *meanRoll = lastMeanRoll;
  *meanPitch = lastMeanPitch;
  *pending = savePending;
}
