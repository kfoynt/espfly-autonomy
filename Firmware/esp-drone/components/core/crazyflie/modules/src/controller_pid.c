
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "debug_cf.h"
#include "param.h"
#include "math3d.h"
#include "FreeRTOS.h"
#include "esp_attr.h"
#include "torque_trim.h"
#include "imu_level_cal_store.h"
#include "camera_flow.h"
#include "autonomy.h"
#include "usec_time.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

// Below this thrust the drone is still resting on the ground (lift-off was
// recorded at >= 32000 without the camera and >= 30000 with it), so roll/
// pitch errors only reflect the surface it sits on. Holding the roll/pitch
// integrators at zero there stops them winding up during a slow throttle
// ramp and releasing as a lurch at lift-off. P and D terms keep acting.
// Raised from 20000 after recordings showed +1500..+3800 of pitch-rate
// integral accumulating between 20000 and lift-off. An autonomous take-off
// also holds them until the vertical controller's lift-off
// (autonomyTakeoffIntegratorHold(); user decision 2026-10-13, evening): its
// RAMP spends ~0.4-0.8 s above 26000 on the ground, where a floor tilt winds
// the rate integrators (-765..-1843 by lift-off on the manual floor take-offs
// of 2026-10-12, ~-4100 modelled for the RAMP) and kicks the drone past level
// at lift-off (0.7-1.4 deg logged, ~2.9 deg modelled). Manual flight is
// unchanged.
#define GROUND_INTEGRATOR_THRUST 26000.0f
static float groundIntegratorThrust = GROUND_INTEGRATOR_THRUST;

// Feed-forward roll/pitch torque trim (torque_trim.h), loaded from NVS. It is
// ramped in from TORQUE_TRIM_START_THRUST over TORQUE_TRIM_RAMP_THRUST, so it
// is fully applied before lift-off and never acts on an idle drone.
#define TORQUE_TRIM_START_THRUST 20000.0f
#define TORQUE_TRIM_RAMP_THRUST 10000.0f
static portMUX_TYPE torqueTrimMux = portMUX_INITIALIZER_UNLOCKED;
static float torqueTrimRoll;
static float torqueTrimPitch;
static esp_err_t torqueTrimLoadResult = ESP_ERR_NVS_NOT_FOUND;

// Yaw lock: ignore the yaw-rate command and hold the heading captured at
// take-off (attitudeDesired.yaw is reset to the current yaw at zero thrust).
// For throttle-only flying, where the thumb drifts sideways on the phone's
// throttle stick. The setpoint itself is left untouched so it is still logged.
static volatile bool yawLock;

// Attitude trim (degrees) added to the roll/pitch attitude setpoint, i.e. the
// level the controller holds (see torque_trim.h). Includes the link-loss
// levelling setpoint.
static float attTrimRoll;
static float attTrimPitch;
static esp_err_t attTrimLoadResult = ESP_ERR_NVS_NOT_FOUND;

// Camera drift-hold correction actually applied (deg). Follows the camera's
// value while it is fresh, otherwise fades to zero; zeroed at once on an axis
// with pilot input and at zero thrust.
#define CAM_HOLD_STALE_US     300000
/* The apply slew is the hold's own (camera_flow.h cameraFlowHoldSlew): CAM_HOLD_SLEW_DEG_S, 6 deg/s as this file's
 * constant was, faster only on a fast move's frames (mvspeed > 100, 2026-10-16). */
static float camCorrRoll;
static float camCorrPitch;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerPidInit(void)
{
  static bool trimLoaded;

  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();

  if (!trimLoaded) {
    torqueTrimRecord_t rec;
    torqueTrimLoadResult = torqueTrimStoreLoad(&rec);
    if (torqueTrimLoadResult == ESP_OK) {
      controllerPidSetTorqueTrim(rec.roll, rec.pitch);
      DEBUG_PRINTI("torque trim loaded: roll = %.0f, pitch = %.0f", (double)rec.roll, (double)rec.pitch);
    } else if (torqueTrimLoadResult != ESP_ERR_NVS_NOT_FOUND) {
      DEBUG_PRINTW("torque trim: stored record rejected (%s), trim 0", esp_err_to_name(torqueTrimLoadResult));
    }
    attTrimRecord_t att;
    attTrimLoadResult = attTrimStoreLoad(&att);
    if (attTrimLoadResult == ESP_OK) {
      controllerPidSetAttitudeTrim(att.rollDeg, att.pitchDeg);
      DEBUG_PRINTI("attitude trim loaded: roll = %.2f, pitch = %.2f deg", (double)att.rollDeg, (double)att.pitchDeg);
    }
    bool lock = false;
    if (yawLockStoreLoad(&lock) == ESP_OK) {
      yawLock = lock;
      DEBUG_PRINTI("yaw lock %s", lock ? "on" : "off");
    }
    trimLoaded = true;
  }
}

void controllerPidGetTorqueTrim(float *roll, float *pitch, int *loadResult)
{
  portENTER_CRITICAL(&torqueTrimMux);
  *roll = torqueTrimRoll;
  *pitch = torqueTrimPitch;
  portEXIT_CRITICAL(&torqueTrimMux);
  *loadResult = torqueTrimLoadResult;
}

bool controllerPidSetTorqueTrim(float roll, float pitch)
{
  if (!torqueTrimValuesValid(roll, pitch)) {
    return false;
  }
  portENTER_CRITICAL(&torqueTrimMux);
  torqueTrimRoll = roll;
  torqueTrimPitch = pitch;
  portEXIT_CRITICAL(&torqueTrimMux);
  return true;
}

void controllerPidGetAttitudeTrim(float *rollDeg, float *pitchDeg, int *loadResult)
{
  portENTER_CRITICAL(&torqueTrimMux);
  *rollDeg = attTrimRoll;
  *pitchDeg = attTrimPitch;
  portEXIT_CRITICAL(&torqueTrimMux);
  *loadResult = attTrimLoadResult;
}

bool controllerPidSetAttitudeTrim(float rollDeg, float pitchDeg)
{
  if (!attTrimValuesValid(rollDeg, pitchDeg)) {
    return false;
  }
  portENTER_CRITICAL(&torqueTrimMux);
  attTrimRoll = rollDeg;
  attTrimPitch = pitchDeg;
  portEXIT_CRITICAL(&torqueTrimMux);
  return true;
}

void controllerPidGetCameraCorrection(float *rollDeg, float *pitchDeg)
{
  *rollDeg = camCorrRoll;
  *pitchDeg = camCorrPitch;
}

/* IRAM: the autonomy glue reads it on every full step (its attErrDeg). */
void IRAM_ATTR controllerPidGetAttitudeDesired(float *rollDeg, float *pitchDeg)
{
  *rollDeg = attitudeDesired.roll;
  *pitchDeg = attitudeDesired.pitch;
}

static float slewTo(float cur, float target, float step)
{
  float d = target - cur;
  if (d > step) {
    d = step;
  } else if (d < -step) {
    d = -step;
  }
  return cur + d;
}

void controllerPidSetYawLock(bool on)
{
  yawLock = on;
}

bool controllerPidGetYawLock(void)
{
  return yawLock;
}

static int16_t clampInt16(float in)
{
  if (in > INT16_MAX) {
    return INT16_MAX;
  }
  if (in < -INT16_MAX) {
    return -INT16_MAX;
  }
  return (int16_t)in;
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();
  DEBUG_PRINTD("controller_Pid_Test = %d", pass);

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerPid(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {
       const float yawRate = yawLock ? 0.0f : setpoint->attitudeRate.yaw;
       attitudeDesired.yaw += yawRate * ATTITUDE_UPDATE_DT;
    } else {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      float trimR, trimP;
      portENTER_CRITICAL(&torqueTrimMux);
      trimR = attTrimRoll;
      trimP = attTrimPitch;
      portEXIT_CRITICAL(&torqueTrimMux);
      float holdR, holdP;
      bool holdEng;
      uint64_t holdT;
      cameraFlowGetHold(&holdR, &holdP, &holdEng, &holdT);
      const uint64_t nowUs = usecTimestamp();
      const bool fresh = holdEng && nowUs >= holdT && (nowUs - holdT) < CAM_HOLD_STALE_US;
      float targetR = fresh ? holdR : 0.0f;
      float targetP = fresh ? holdP : 0.0f;
      if (!fresh) {
        /* a fast MOVE's brake with the camera stalled (mvspeed > 100, autonomy.h autonomyBrakeLean): the brake lean
         * instead of fading to 0; never at 100. Pilot input and a blocked hold still win (below). */
        (void)autonomyBrakeLean(&targetR, &targetP);
      }
      if (setpoint->attitude.roll != 0.0f) {
        camCorrRoll = 0.0f;
        targetR = 0.0f;
      }
      if (setpoint->attitude.pitch != 0.0f) {
        camCorrPitch = 0.0f;
        targetP = 0.0f;
      }
      if (autonomyHoldBlocked()) {   /* autonomous LAND after contact: no camera lean (autonomy.h) */
        camCorrRoll = 0.0f;
        camCorrPitch = 0.0f;
        targetR = 0.0f;
        targetP = 0.0f;
      }
      /* the hold's own output slew (6 deg/s; faster on a fast move's frames, camera_flow.h cameraFlowHoldSlew) */
      const float step = cameraFlowHoldSlew() * ATTITUDE_UPDATE_DT;
      camCorrRoll = slewTo(camCorrRoll, targetR, step);
      camCorrPitch = slewTo(camCorrPitch, targetP, step);
      attitudeDesired.roll = setpoint->attitude.roll + trimR + camCorrRoll;
      attitudeDesired.pitch = setpoint->attitude.pitch + trimP + camCorrPitch;
    }

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;

    if (actuatorThrust < groundIntegratorThrust || autonomyTakeoffIntegratorHold()) {
      attitudeControllerResetRollPitchIntegrals();
    }

    float trimRoll, trimPitch;
    portENTER_CRITICAL(&torqueTrimMux);
    trimRoll = torqueTrimRoll;
    trimPitch = torqueTrimPitch;
    portEXIT_CRITICAL(&torqueTrimMux);
    if (trimRoll != 0.0f || trimPitch != 0.0f) {
      float scale = torqueTrimScale(actuatorThrust, TORQUE_TRIM_START_THRUST, TORQUE_TRIM_RAMP_THRUST);
      control->roll = clampInt16(control->roll + scale * trimRoll);
      control->pitch = clampInt16(control->pitch + scale * trimPitch);
    }

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();
    camCorrRoll = 0.0f;
    camCorrPitch = 0.0f;

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}


LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

PARAM_GROUP_START(controller)
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_ADD(PARAM_FLOAT, iGroundThr, &groundIntegratorThrust)
PARAM_GROUP_STOP(controller)
