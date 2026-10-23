/*
 * ESP-Drone Firmware
 *
 * ground_cool_glue.c - The ground cooling spin in the stabilizer (ground_cool.h, docs/autonomy-a2b.md 36.3)
 */
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "ground_cool_glue.h"
#include "autonomy.h"
#include "flight_recorder.h"
#include "motors.h"
#include "pm_esplane.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "stabilizer.h"
#include "stm32_legacy.h"

volatile bool groundCoolGlueRunningWord;
/* The stabilizer task owns gc while the word is set; the console starts it only when it is clear and reads it
 * for a status (a torn float in a status line is harmless). */
static groundCool_t gc;

static void inputsFromState(groundCoolIn_t *in, const sensorData_t *sensors, const state_t *state)
{
  in->autonomyActive = autonomyActive();
  in->motorsReady = motorsReadyNoLock() && !stabilizerRestartLocked();
  in->tiltDeg = sqrtf(state->attitude.roll * state->attitude.roll + state->attitude.pitch * state->attitude.pitch);
  in->gyroDps[0] = sensors->gyro.x;
  in->gyroDps[1] = sensors->gyro.y;
  in->gyroDps[2] = sensors->gyro.z;
  in->accNormG = sqrtf(sensors->acc.x * sensors->acc.x + sensors->acc.y * sensors->acc.y + sensors->acc.z * sensors->acc.z);
  in->imuTempC = sensorsMpu6050GetTempC();
}

bool groundCoolGlueStart(uint32_t pwm, uint32_t seconds, const char **why)
{
  if (groundCoolGlueRunningWord) {
    *why = groundCoolRefusalName(GC_E_RUNNING);
    return false;
  }
  flightRecord_t r;
  if (!flightRecorderGetLatest(&r)) {
    *why = "stabilizer not running yet";
    return false;
  }
  /* The latest recorder sample stands in for the loop's sensors and state (100 Hz, filtered). */
  groundCoolIn_t in;
  memset(&in, 0, sizeof(in));
  in.autonomyActive = autonomyActive();
  in.motorsReady = motorsReadyNoLock() && !stabilizerRestartLocked() && !autonomyResetLocked();
  in.dutyZero = !motorsAnyDutyNonZero();
  in.thrustZero = r.thrust == 0 && r.rawThrust == 0;
  in.onGround = stabilizerOnGround();
  in.vbat = pmGetBatteryVoltage();
  in.vminV = (float)autonomySettingGet(AUTO_SET_VMIN) * 0.001f;
  in.tiltDeg = sqrtf((float)r.roll * (float)r.roll + (float)r.pitch * (float)r.pitch) * 0.01f;
  for (int i = 0; i < 3; i++) {
    in.gyroDps[i] = r.gyro[i] * 0.1f;
  }
  in.accNormG = sqrtf((float)r.acc[0] * r.acc[0] + (float)r.acc[1] * r.acc[1] + (float)r.acc[2] * r.acc[2]) * 0.001f;
  in.imuTempC = sensorsMpu6050GetTempC();
  const groundCoolRefusal_t res = groundCoolStart(&gc, &in, pwm, seconds, T2M(xTaskGetTickCount()));
  if (res != GC_OK) {
    *why = groundCoolRefusalName(res);
    return false;
  }
  /* The accelerometer compensation's reference before the spin: the still reading now (the drone rested 1 s for
   * the guards above), its next entry marked as the spin's (accel_tempco.h ACC_TC_RZ_COOL). */
  sensorsMpu6050AccTcLatch(true);
  groundCoolGlueRunningWord = true;   /* last: the stabilizer takes over gc */
  return true;
}

void groundCoolGlueStop(void)
{
  groundCoolStop(&gc);
}

void groundCoolGlueKill(void)
{
  groundCoolKill(&gc);
}

void groundCoolGlueStatus(groundCoolGlueStatus_t *st)
{
  memset(st, 0, sizeof(*st));
  st->gc = gc;
  st->running = groundCoolGlueRunningWord;
  st->tempC = sensorsMpu6050GetTempC();
  if (st->running) {
    const uint32_t el = T2M(xTaskGetTickCount()) - gc.startMs;
    const uint32_t total = (uint32_t)gc.seconds * 1000u;
    st->leftMs = el < total ? total - el : 0;
  }
}

bool groundCoolGlueArbitrate(setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, uint32_t nowMs)
{
  groundCoolIn_t in;
  memset(&in, 0, sizeof(in));
  inputsFromState(&in, sensors, state);
  const float thrust = groundCoolStep(&gc, &in, nowMs);
  if (!gc.running) {
    groundCoolGlueRunningWord = false;   /* ended this loop: the normal setpoint (thrust 0 in IDLE) stands */
    return false;
  }
  /* The whole setpoint: a level attitude at the current heading and the spin's PWM; the attitude outputs are
   * zeroed after the controller anyway (groundCoolGlueLevelOutputs). */
  memset(setpoint, 0, sizeof(*setpoint));
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;
  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = state->attitude.yaw;
  setpoint->thrust = thrust;
  setpoint->timestamp = xTaskGetTickCount();
  return true;
}

void groundCoolGlueLevelOutputs(control_t *control)
{
  control->roll = 0;
  control->pitch = 0;
  control->yaw = 0;
}
