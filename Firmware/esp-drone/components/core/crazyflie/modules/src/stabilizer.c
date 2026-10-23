/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

#include "esp_attr.h"
#include <math.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32_legacy.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "motors.h"
#include "pm_esplane.h"
#include "esp_timer.h"
#include "stabilizer.h"
#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"
//#include "collision_avoidance.h"

#include "estimator.h"
//#include "usddeck.h" //usddeckLoggingMode_e
#include "quatcompress.h"
#include "statsCnt.h"
#define DEBUG_MODULE "STAB"
#include "debug_cf.h"
#include "static_mem.h"
#include "rateSupervisor.h"
#include "config.h"
#include "flight_recorder.h"
#include "camera_flow.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "imu_level_cal_store.h"
#include "thrust_softstart.h"
#include "motor_guard.h"
#include "cam_vz_glue.h"
#include "autonomy.h"
#include "ground_cool_glue.h"
#include "ledseq.h"
#include "esp_system.h"

static bool isInit;
static bool emergencyStop = false;
/* Diagnostic ground-test thrust cap (0 = off), persisted in NVS ("gcap") so it
 * survives the reboots a ground test needs. It only ever lowers the thrust
 * setpoint, so a forgotten cap fails safe (the drone cannot take off). */
static volatile uint16_t thrustCap;
/* Take-off thrust ramp after a ground stop (thrust_softstart.h); switchable,
 * persisted in NVS ("soft"), default on. */
static thrustSoftStart_t softStart;
static volatile bool softStartOn = true;
/* Restart lock (2026-10-11): after any reset other than a power-on (brownout,
 * software reboot, firmware flash) the first take-off browned out 14 of 15
 * times, while every take-off after a battery replug flew: something on the
 * XIAO keeps drawing extra current until the battery is disconnected. Thrust
 * is held at zero for the rest of such a boot and the red LED double-blinks;
 * replugging the battery clears it. Switchable ("plock", NVS, default on). */
static volatile bool restartLockOn = true;
static volatile bool restartLocked;
/* Bench fault injection (USB 'mguard test hang', docs/autonomy-a2.md 7.3):
 * the next loop busy-waits this long, motors left at their last duty, so the
 * motor watchdog can be seen to cut. 0 = none. */
static volatile uint32_t faultHangMs;

/* Take-off window: the first seconds after a ground start, when the battery
 * sags most and the XIAO's 3.3 V rail has the least margin. Other tasks shed
 * load during it (camera standby, low Wi-Fi TX power). A ground start is
 * thrust leaving zero while the soft-start is armed, i.e. after a rest. */
#define TAKEOFF_WINDOW_MS     3000    /* at least this long after the ground start */
#define TAKEOFF_AFTER_LIFT_MS 1500    /* and this long after thrust first reaches lift-off level */
#define TAKEOFF_WINDOW_MAX_MS 8000    /* a slow throttle-up or idle never keeps it open longer */
#define TAKEOFF_LIFT_THRUST   40000   /* near lift-off: ~45-52k on a part-charged battery; ground resets at 39-43k */
static volatile uint32_t takeoffStartMs;
static volatile uint32_t takeoffLiftMs;
static volatile bool takeoffLifted;
static volatile uint32_t takeoffCount;
static uint16_t prevThrustReq;
static bool takeoffIsCool;             /* the latest ground start was a cooling spin (ground_cool_glue.h) */
/* Cold take-off: the first after boot or after a break. Every session's
 * first take-off had the largest XIAO rail drop (0.16-0.34 V below the
 * battery), and it settled after ~2-3 s of flying. A take-off is warm when
 * the drone flew at least TAKEOFF_WARM_FLY_MS at high thrust, with no gap
 * longer than TAKEOFF_WARM_KEEP_MS since. */
#define TAKEOFF_WARM_FLY_MS   4000
#define TAKEOFF_WARM_KEEP_MS  120000
static uint32_t highThrustMs;          /* high-thrust time, reset after a long break */
static uint32_t lastHighThrustMs;
static volatile bool takeoffCold = true;
/* Motor-on time of the latest flight (ground start to thrust 0): autonomy's
 * heat-soak wait (G2, auto_fsm.h AUTO_SOAK_*; stabilizer.h). */
volatile uint32_t stabilizerLastFlightMotorMsWord;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static bool checkStops;

#define PROPTEST_NBR_OF_VARIANCE_VALUES   100
static bool startPropTest = false;

uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static portMUX_TYPE benchTelemetryMux = portMUX_INITIALIZER_UNLOCKED;
static stabilizerBenchTelemetry_t benchTelemetry = {
  .motors_locked = false,
};
static bool benchTelemetryReady;

bool stabilizerGetBenchTelemetry(stabilizerBenchTelemetry_t *out)
{
  if (out == NULL) {
    return false;
  }
  portENTER_CRITICAL(&benchTelemetryMux);
  *out = benchTelemetry;
  const bool ready = benchTelemetryReady;
  portEXIT_CRITICAL(&benchTelemetryMux);
  return ready;
}

static uint32_t benchSaturateUs(uint64_t value)
{
  return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

/* Loop-gap maximum and >2.5 ms count since the last stabilizerTakeLoopGaps()
 * (the flight log reads them at 100 Hz from this same task). */
static uint32_t gapWindowMaxUs;
static uint32_t gapWindowOver2500;

void stabilizerTakeLoopGaps(uint32_t *maxUs, uint32_t *over2500)
{
  *maxUs = gapWindowMaxUs;
  *over2500 = gapWindowOver2500;
  gapWindowMaxUs = 0;
  gapWindowOver2500 = 0;
}

static void publishBenchTelemetry(uint64_t loopStartUs, uint64_t previousStartUs)
{
  // Only this task writes the snapshot; compute outside the short copy lock.
  static stabilizerBenchTelemetry_t next;
  const uint64_t nowUs = usecTimestamp();
  next.timestamp_us = nowUs;
  next.imu_timestamp_us = sensorData.interruptTimestamp;
  next.loop_count++;
  next.last_loop_gap_us = previousStartUs ? benchSaturateUs(loopStartUs - previousStartUs) : 0;
  if (next.last_loop_gap_us > next.max_loop_gap_us) {
    next.max_loop_gap_us = next.last_loop_gap_us;
  }
  if (next.last_loop_gap_us > 1500) next.loop_gap_over_1500us++;
  if (next.last_loop_gap_us > 2500) next.loop_gap_over_2500us++;
  if (next.last_loop_gap_us > gapWindowMaxUs) gapWindowMaxUs = next.last_loop_gap_us;
  if (next.last_loop_gap_us > 2500) gapWindowOver2500++;
  if (next.last_loop_gap_us > 5000) next.loop_gap_over_5000us++;
  next.last_loop_exec_us = benchSaturateUs(nowUs - loopStartUs);
  if (next.last_loop_exec_us > next.max_loop_exec_us) {
    next.max_loop_exec_us = next.last_loop_exec_us;
  }
  next.sensor_to_output_us = sensorData.interruptTimestamp && nowUs >= sensorData.interruptTimestamp
      ? benchSaturateUs(nowUs - sensorData.interruptTimestamp) : UINT32_MAX;
  next.exec_sum_us += next.last_loop_exec_us;
  if (next.sensor_to_output_us != UINT32_MAX) {
    next.sensor_to_output_sum_us += next.sensor_to_output_us;
    next.sensor_to_output_n++;
    if (next.sensor_to_output_us > 800) next.sensor_to_output_over_800us++;
  }
  next.acceleration_g[0] = sensorData.acc.x;
  next.acceleration_g[1] = sensorData.acc.y;
  next.acceleration_g[2] = sensorData.acc.z;
  next.angular_rate_dps[0] = sensorData.gyro.x;
  next.angular_rate_dps[1] = sensorData.gyro.y;
  next.angular_rate_dps[2] = sensorData.gyro.z;
  next.attitude_deg[0] = state.attitude.roll;
  next.attitude_deg[1] = state.attitude.pitch;
  next.attitude_deg[2] = state.attitude.yaw;
  next.motors_locked = motorsAreBenchLocked();
  next.calibrated = sensorsAreCalibrated();
  portENTER_CRITICAL(&benchTelemetryMux);
  benchTelemetry = next;
  benchTelemetryReady = true;
  portEXIT_CRITICAL(&benchTelemetryMux);
}

static StateEstimatorType estimatorType;
static ControllerType controllerType;

typedef enum { configureAcc, measureNoiseFloor, measureProp, testBattery, restartBatTest, evaluateResult, testDone } TestState;
#if 0 // No automatic propeller tests in manual flight.
  static TestState testState = configureAcc;
#else
  static TestState testState = testDone;
#endif

static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);
static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;

static float accVarX[NBR_OF_MOTORS];
static float accVarY[NBR_OF_MOTORS];
static float accVarZ[NBR_OF_MOTORS];
// Bit field indicating if the motors passed the motor test.
// Bit 0 - 1 = M1 passed
// Bit 1 - 1 = M2 passed
// Bit 2 - 1 = M3 passed
// Bit 3 - 1 = M4 passed
static uint8_t motorPass = 0;
static uint16_t motorTestCount = 0;

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void* param);
static void testProps(sensorData_t *sensors) __attribute__((unused));   // propeller test, never run in manual flight

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

static void compressState()
{
  stateCompressed.x = state.position.x * 1000.0f;
  stateCompressed.y = state.position.y * 1000.0f;
  stateCompressed.z = state.position.z * 1000.0f;

  stateCompressed.vx = state.velocity.x * 1000.0f;
  stateCompressed.vy = state.velocity.y * 1000.0f;
  stateCompressed.vz = state.velocity.z * 1000.0f;

  stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
  stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
  stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

  float const q[4] = {
    state.attitudeQuaternion.x,
    state.attitudeQuaternion.y,
    state.attitudeQuaternion.z,
    state.attitudeQuaternion.w};
  stateCompressed.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

static void compressSetpoint()
{
  setpointCompressed.x = setpoint.position.x * 1000.0f;
  setpointCompressed.y = setpoint.position.y * 1000.0f;
  setpointCompressed.z = setpoint.position.z * 1000.0f;

  setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
  setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
  setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

  setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
  setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
  setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  uint16_t cap = 0;
  if (thrustCapStoreLoad(&cap) == ESP_OK) {
    thrustCap = cap;
  }
  if (estimator == anyEstimator) {
    estimator = deckGetRequiredEstimator();
  }
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  {
    /* Motor drive settings (motors are initialised and stopped here). */
    bool soft = true;
    if (softStartStoreLoad(&soft) == ESP_OK) {
      softStartOn = soft;
    }
    bool plock = true;
    if (restartLockStoreLoad(&plock) == ESP_OK) {
      restartLockOn = plock;
    }
    restartLocked = restartLockOn && esp_reset_reason() != ESP_RST_POWERON;
    if (restartLocked) {
      ledseqRun(&seq_restartLock);
      DEBUG_PRINTW("restart lock: not a power-on reset, thrust held at 0 until the battery is replugged");
    }
    thrustSoftStartInit(&softStart);
    uint32_t hz = 0;
    if (motorPwmStoreLoad(&hz) == ESP_OK && hz != motorsGetPwmFreq() && !motorsSetPwmFreq(hz)) {
      DEBUG_PRINTW("motor PWM %u Hz from NVS refused; staying at %u Hz", (unsigned)hz, (unsigned)motorsGetPwmFreq());
    }
    /* Motor watchdog: cuts the motors if this task stops (motor_guard.h).
     * Started before the task, with the motors initialised and stopped. */
    bool mguard = true;
    if (motorGuardStoreLoad(&mguard) != ESP_OK) {
      mguard = true;
    }
    motorGuardInit(mguard);
  }
  flightRecorderInit();
  /* Camera height observer, shadow mode (cam_vz_glue.h): measures only. */
  camVzGlueInit();
  /* Autonomy glue (autonomy.h): settings, reset-lock, IDLE. Before the task. */
  autonomyInit();
  sitAwInit();
  //collisionAvoidanceInit();
  estimatorType = getStateEstimator();
  controllerType = getControllerType();

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;
}

void stabilizerSetRestartLock(bool on)
{
  restartLockOn = on;
  if (!on && restartLocked) {
    restartLocked = false;
    ledseqStop(&seq_restartLock);
  }
}

bool stabilizerRestartLockEnabled(void)
{
  return restartLockOn;
}

bool IRAM_ATTR stabilizerRestartLocked(void)
{
  return restartLocked;
}

bool stabilizerTakeoffPhase(uint32_t minMs, uint32_t afterLiftMs)
{
  if (takeoffCount == 0) {
    return false;
  }
  const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  const uint32_t elapsed = now - takeoffStartMs;
  if (elapsed >= TAKEOFF_WINDOW_MAX_MS) {
    return false;
  }
  /* Open for minMs, then until afterLiftMs past the lift-off thrust level
   * (which a slow throttle-up reaches late). A phase only ever extends while
   * open, so a sleeper woken by its end is not put back to sleep. */
  return elapsed < minMs || !takeoffLifted || (uint32_t)(now - takeoffLiftMs) < afterLiftMs;
}

/* Autonomy: the vertical controller's detected lift-off is the window's
 * lift-off (stabilizer.h). Called from autonomyArbitrate(), this task. */
void stabilizerNoteLiftoff(void)
{
  if (takeoffCount > 0 && !takeoffLifted) {
    takeoffLiftMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
    takeoffLifted = true;
  }
}

bool IRAM_ATTR stabilizerOnGround(void)
{
  return prevThrustReq == 0 && thrustSoftStartActive(&softStart);
}

bool stabilizerTakeoffCold(void)
{
  return takeoffCold;
}

bool stabilizerTakeoffWindow(void)
{
  return stabilizerTakeoffPhase(TAKEOFF_WINDOW_MS, TAKEOFF_AFTER_LIFT_MS);
}

uint32_t stabilizerTakeoffCount(void)
{
  return takeoffCount;
}

void stabilizerSetSoftStart(bool on)
{
  softStartOn = on;
}

bool stabilizerSoftStartEnabled(void)
{
  return softStartOn;
}

void stabilizerSetThrustCap(uint16_t cap)
{
  thrustCap = cap;
}

uint16_t IRAM_ATTR stabilizerGetThrustCap(void)
{
  return thrustCap;
}

void stabilizerFaultHang(uint32_t ms)
{
  faultHangMs = ms;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();
  //pass &= collisionAvoidanceTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  uint64_t previousLoopStartUs = 0;

#ifdef configUSE_APPLICATION_TASK_TAG
	#if configUSE_APPLICATION_TASK_TAG == 1
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);
  #endif
#endif

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  DEBUG_PRINTI("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

  DEBUG_PRINTI("Manual stabilization ready; await fresh zero-thrust app command.\n");

  while(1) {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();
    const uint32_t hangMs = faultHangMs;
    if (hangMs) {
      faultHangMs = 0;
      const uint64_t untilUs = usecTimestamp() + (uint64_t)hangMs * 1000;
      while (usecTimestamp() < untilUs) {
      }
    }
    const uint64_t loopStartUs = usecTimestamp();

    // Motor diagnostics never run in the flight loop.
    startPropTest = false;
    testState = testDone;

    {
      // allow to update estimator dynamically
      if (getStateEstimator() != estimatorType) {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = getStateEstimator();
      }
      // allow to update controller dynamically
      if (getControllerType() != controllerType) {
        controllerInit(controllerType);
        controllerType = getControllerType();
      }

      stateEstimator(&state, &sensorData, &control, tick);
      cameraFlowGyroSample(sensorData.interruptTimestamp, &sensorData.gyro);
      compressState();

      commanderGetSetpoint(&setpoint, &state);
      /* Autonomy (docs/autonomy-a2b.md section 2): in IDLE it leaves the
       * setpoint alone; while active it replaces all of it (thrust at most
       * the mode maximum). Everything below only lowers thrust. */
      autonomyArbitrate(&setpoint, &sensorData, &state, T2M(xTaskGetTickCount()),
                        previousLoopStartUs ? benchSaturateUs(loopStartUs - previousLoopStartUs) : 0);
      /* The ground cooling spin (ground_cool_glue.h, docs/autonomy-a2b.md 36.3): from IDLE only, its PWM as the
       * thrust setpoint; one load and a branch when none runs. Everything below only lowers thrust. */
      const bool coolRun = groundCoolGlueRunning() &&
                           groundCoolGlueArbitrate(&setpoint, &sensorData, &state, T2M(xTaskGetTickCount()));
      if (restartLocked) {
        setpoint.thrust = 0.0f;
      }
      const uint16_t cap = thrustCap;
      if (cap > 0 && setpoint.thrust > (float)cap) {
        setpoint.thrust = (float)cap;
      }
      bool groundStart = false;   /* thrust left a ground stop this loop */
      bool resting;
      {
        /* At rest: gyro still and about 1 g (a falling drone reads ~0 g). */
        const float accNorm = sqrtf(sensorData.acc.x * sensorData.acc.x + sensorData.acc.y * sensorData.acc.y +
                                    sensorData.acc.z * sensorData.acc.z);
        resting = fabsf(sensorData.gyro.x) < 10.0f && fabsf(sensorData.gyro.y) < 10.0f &&
                  fabsf(sensorData.gyro.z) < 10.0f && fabsf(accNorm - 1.0f) < 0.15f;
        const uint16_t req = (uint16_t)fminf(fmaxf(setpoint.thrust, 0.0f), 65535.0f);
        const bool falling = accNorm < 0.5f;
        const uint32_t nowMs = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (req > 0 && prevThrustReq == 0 && thrustSoftStartActive(&softStart)) {
          takeoffCold = !(highThrustMs >= TAKEOFF_WARM_FLY_MS &&
                          (uint32_t)(nowMs - lastHighThrustMs) < TAKEOFF_WARM_KEEP_MS);
          takeoffLifted = false;
          takeoffStartMs = nowMs;
          takeoffCount++;
          groundStart = true;
          takeoffIsCool = coolRun;   /* a cooling spin is not a flight for the heat-soak wait */
        }
        if (req >= TAKEOFF_LIFT_THRUST) {
          if ((uint32_t)(nowMs - lastHighThrustMs) >= TAKEOFF_WARM_KEEP_MS) {
            highThrustMs = 0;
          }
          if (highThrustMs < UINT32_MAX) {
            highThrustMs++;   /* one 1 kHz loop */
          }
          lastHighThrustMs = nowMs;
        }
        if (!takeoffLifted && takeoffCount > 0 && req >= TAKEOFF_LIFT_THRUST) {
          takeoffLiftMs = nowMs;
          takeoffLifted = true;
        }
        if (req == 0 && prevThrustReq > 0 && takeoffCount > 0 && !takeoffIsCool) {
          stabilizerLastFlightMotorMsWord = nowMs - takeoffStartMs;
        }
        prevThrustReq = req;
        const uint16_t soft = thrustSoftStartApply(&softStart, req, resting, falling, softStartOn);
        if (soft < req) {
          setpoint.thrust = (float)soft;
        }
      }
      compressSetpoint();
      sensorsMpu6050SetThrust(setpoint.thrust);

      sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
      //collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);

      controller(&control, &setpoint, &sensorData, &state, tick);
      if (coolRun) {
        groundCoolGlueLevelOutputs(&control);   /* the four motors get the spin's PWM alone */
      }

      checkEmergencyStopTimeout();
      /* Autonomy's second barrier, on the controller's output: 0 with DRY,
       * ZERO or CUT, the mode maximum while it flies, untouched (65535) in
       * manual flight. Thrust 0 makes powerDistribution() stop the motors. */
      control.thrust = autoClampThrust(control.thrust, autonomyThrustMax());

      checkStops = systemIsArmed();
      if (emergencyStop || (systemIsArmed() == false)) {
        powerStop();
      } else {
        powerDistribution(&control);
      }
      motorGuardStabilizerAlive();

      /* Camera height observer, shadow mode: the thrust the motors really got
       * (0 once powerStop(), a trip or the inhibit zeroed every duty). Before
       * the recorder, which logs its outputs. */
      camVzGlueStep(&sensorData, motorsAnyDutyNonZero() ? setpoint.thrust : 0.0f, groundStart, resting);
      /* What the motors really got, for the vertical controller's next step. */
      autonomyNoteApplied((emergencyStop || !systemIsArmed() || !motorsAnyDutyNonZero()) ? 0.0f : control.thrust);

      if (RATE_DO_EXECUTE(FLIGHT_RECORDER_RATE_HZ, tick)) {
        /* The loop counter under-reports elapsed time if an IMU period is missed.
         * Keep its rate divider, but stamp the record on the real RTOS clock. */
        flightRecorderSample(&setpoint, &state, &sensorData, T2M(xTaskGetTickCount()));
      }

      //TODO: Log data to uSD card if configured
      /*if (usddeckLoggingEnabled()
          && usddeckLoggingMode() == usddeckLoggingMode_SynchronousStabilizer
          && RATE_DO_EXECUTE(usddeckFrequency(), tick)) {
        usddeckTriggerLogging();
      }*/
    }
    calcSensorToOutputLatency(&sensorData);
    publishBenchTelemetry(loopStartUs, previousLoopStartUs);
    previousLoopStartUs = loopStartUs;
    tick++;
    STATS_CNT_RATE_EVENT(&stabilizerRate);

    if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount())) {
      if (!rateWarningDisplayed) {
        DEBUG_PRINT("WARNING: stabilizer loop rate is off (%"PRIu32")\n", rateSupervisorLatestCount(&rateSupervisorContext));
        rateWarningDisplayed = true;
      }
    }
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

bool IRAM_ATTR stabilizerEmergencyStopActive(void)
{
  return emergencyStop;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

static float variance(float *buffer, uint32_t length)
{
  uint32_t i;
  float sum = 0;
  float sumSq = 0;

  for (i = 0; i < length; i++)
  {
    sum += buffer[i];
    sumSq += buffer[i] * buffer[i];
  }

  return sumSq - (sum * sum) / length;
}

/** Evaluate the values from the propeller test
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
static bool evaluateTest(float low, float high, float value, uint8_t motor)
{
  if (value < low || value > high)
  {
    DEBUG_PRINTI("Propeller test on M%d [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                motor + 1, (double)low, (double)high, (double)value);
    return false;
  }

  motorPass |= (1 << motor);

  return true;
}


static void testProps(sensorData_t *sensors)
{
  static uint32_t i = 0;
  NO_DMA_CCM_SAFE_ZERO_INIT static float accX[PROPTEST_NBR_OF_VARIANCE_VALUES];
  NO_DMA_CCM_SAFE_ZERO_INIT static float accY[PROPTEST_NBR_OF_VARIANCE_VALUES];
  NO_DMA_CCM_SAFE_ZERO_INIT static float accZ[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accVarXnf;
  static float accVarYnf;
  static float accVarZnf;
  static int motorToTest = 0;
  static uint8_t nrFailedTests = 0;
  static float idleVoltage;
  static float minSingleLoadedVoltage[NBR_OF_MOTORS];
  static float minLoadedVoltage;

  if (testState == configureAcc)
  {
    motorPass = 0;
    sensorsSetAccMode(ACC_MODE_PROPTEST);
    testState = measureNoiseFloor;
    minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    minSingleLoadedVoltage[MOTOR_M1] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M2] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M3] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M4] = minLoadedVoltage;
  }
  if (testState == measureNoiseFloor)
  {
    accX[i] = sensors->acc.x;
    accY[i] = sensors->acc.y;
    accZ[i] = sensors->acc.z;

    if (++i >= PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      i = 0;
      accVarXnf = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarYnf = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZnf = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINTI("Acc noise floor variance X+Y:%f, (Z:%f)\n",
                  (double)accVarXnf + (double)accVarYnf, (double)accVarZnf);
      testState = measureProp;
    }

  }
  else if (testState == measureProp)
  {
    if (i < PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accX[i] = sensors->acc.x;
      accY[i] = sensors->acc.y;
      accZ[i] = sensors->acc.z;
      if (pmGetBatteryVoltage() < minSingleLoadedVoltage[motorToTest])
      {
        minSingleLoadedVoltage[motorToTest] = pmGetBatteryVoltage();
      }
    }
    i++;

    if (i == 1)
    {
      motorsSetRatio(motorToTest, 0xFFFF);
    }
    else if (i == 50)
    {
      motorsSetRatio(motorToTest, 0);
    }
    else if (i == PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accVarX[motorToTest] = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarY[motorToTest] = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZ[motorToTest] = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINTI("Motor M%d variance X+Y:%f (Z:%f)\n",
                   motorToTest+1, (double)accVarX[motorToTest] + (double)accVarY[motorToTest],
                   (double)accVarZ[motorToTest]);
    }
    else if (i >= 1000)
    {
      i = 0;
      motorToTest++;
      if (motorToTest >= NBR_OF_MOTORS)
      {
        i = 0;
        motorToTest = 0;
        testState = evaluateResult;
        sensorsSetAccMode(ACC_MODE_FLIGHT);
      }
    }
  }
  else if (testState == testBattery)
  {
    if (i == 0)
    {
      minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    }
    if (i == 1)
    {
      motorsSetRatio(MOTOR_M1, 0xFFFF);
      motorsSetRatio(MOTOR_M2, 0xFFFF);
      motorsSetRatio(MOTOR_M3, 0xFFFF);
      motorsSetRatio(MOTOR_M4, 0xFFFF);
    }
    else if (i < 50)
    {
      if (pmGetBatteryVoltage() < minLoadedVoltage)
        minLoadedVoltage = pmGetBatteryVoltage();
    }
    else if (i == 50)
    {
      motorsSetRatio(MOTOR_M1, 0);
      motorsSetRatio(MOTOR_M2, 0);
      motorsSetRatio(MOTOR_M3, 0);
      motorsSetRatio(MOTOR_M4, 0);
//      DEBUG_PRINT("IdleV: %f, minV: %f, M1V: %f, M2V: %f, M3V: %f, M4V: %f\n", (double)idleVoltage,
//                  (double)minLoadedVoltage,
//                  (double)minSingleLoadedVoltage[MOTOR_M1],
//                  (double)minSingleLoadedVoltage[MOTOR_M2],
//                  (double)minSingleLoadedVoltage[MOTOR_M3],
//                  (double)minSingleLoadedVoltage[MOTOR_M4]);
      DEBUG_PRINTI("%f %f %f %f %f %f\n", (double)idleVoltage,
                  (double)(idleVoltage - minLoadedVoltage),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M1]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M2]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M3]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M4]));
      testState = restartBatTest;
      i = 0;
    }
    i++;
  }
  else if (testState == restartBatTest)
  {
    if (i++ > 2000)
    {
      testState = configureAcc;
      i = 0;
    }
  }
  else if (testState == evaluateResult)
  {
    for (int m = 0; m < NBR_OF_MOTORS; m++)
    {
      if (!evaluateTest(0, PROPELLER_BALANCE_TEST_THRESHOLD,  accVarX[m] + accVarY[m], m))
      {
        nrFailedTests++;
        for (int j = 0; j < 3; j++)
        {
          motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
          vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
          motorsBeep(m, false, 0, 0);
          vTaskDelay(M2T(100));
        }
      }
    }
#ifdef PLAY_STARTUP_MELODY_ON_MOTORS
    if (nrFailedTests == 0)
    {
      for (int m = 0; m < NBR_OF_MOTORS; m++)
      {
        motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsBeep(m, false, 0, 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
      }
    }
#endif
    motorTestCount++;
    testState = testDone;
  }
}
PARAM_GROUP_START(health)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, startPropTest, &startPropTest)
PARAM_GROUP_STOP(health)


PARAM_GROUP_START(stabilizer)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, estimator, &estimatorType)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, controller, &controllerType)
PARAM_ADD(PARAM_UINT8, stop, &emergencyStop)
PARAM_GROUP_STOP(stabilizer)

LOG_GROUP_START(health)
LOG_ADD(LOG_FLOAT, motorVarXM1, &accVarX[0])
LOG_ADD(LOG_FLOAT, motorVarYM1, &accVarY[0])
LOG_ADD(LOG_FLOAT, motorVarXM2, &accVarX[1])
LOG_ADD(LOG_FLOAT, motorVarYM2, &accVarY[1])
LOG_ADD(LOG_FLOAT, motorVarXM3, &accVarX[2])
LOG_ADD(LOG_FLOAT, motorVarYM3, &accVarY[2])
LOG_ADD(LOG_FLOAT, motorVarXM4, &accVarX[3])
LOG_ADD(LOG_FLOAT, motorVarYM4, &accVarY[3])
LOG_ADD(LOG_UINT8, motorPass, &motorPass)
LOG_ADD(LOG_UINT16, motorTestCount, &motorTestCount)
LOG_ADD(LOG_UINT8, checkStops, &checkStops)
LOG_GROUP_STOP(health)

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, x, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, y, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, z, &setpoint.position.z)

LOG_ADD(LOG_FLOAT, vx, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &setpoint.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &setpoint.acceleration.x)
LOG_ADD(LOG_FLOAT, ay, &setpoint.acceleration.y)
LOG_ADD(LOG_FLOAT, az, &setpoint.acceleration.z)

LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(ctrltargetZ)
LOG_ADD(LOG_INT16, x, &setpointCompressed.x)   // position - mm
LOG_ADD(LOG_INT16, y, &setpointCompressed.y)
LOG_ADD(LOG_INT16, z, &setpointCompressed.z)

LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx) // velocity - mm / sec
LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)
LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)

LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax) // acceleration - mm / sec^2
LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)
LOG_ADD(LOG_INT16, az, &setpointCompressed.az)
LOG_GROUP_STOP(ctrltargetZ)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)

STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)

LOG_ADD(LOG_FLOAT, vx, &state.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &state.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &state.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &state.acc.x)
LOG_ADD(LOG_FLOAT, ay, &state.acc.y)
LOG_ADD(LOG_FLOAT, az, &state.acc.z)

LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)

LOG_ADD(LOG_FLOAT, qx, &state.attitudeQuaternion.x)
LOG_ADD(LOG_FLOAT, qy, &state.attitudeQuaternion.y)
LOG_ADD(LOG_FLOAT, qz, &state.attitudeQuaternion.z)
LOG_ADD(LOG_FLOAT, qw, &state.attitudeQuaternion.w)
LOG_GROUP_STOP(stateEstimate)

LOG_GROUP_START(stateEstimateZ)
LOG_ADD(LOG_INT16, x, &stateCompressed.x)                 // position - mm
LOG_ADD(LOG_INT16, y, &stateCompressed.y)
LOG_ADD(LOG_INT16, z, &stateCompressed.z)

LOG_ADD(LOG_INT16, vx, &stateCompressed.vx)               // velocity - mm / sec
LOG_ADD(LOG_INT16, vy, &stateCompressed.vy)
LOG_ADD(LOG_INT16, vz, &stateCompressed.vz)

LOG_ADD(LOG_INT16, ax, &stateCompressed.ax)               // acceleration - mm / sec^2
LOG_ADD(LOG_INT16, ay, &stateCompressed.ay)
LOG_ADD(LOG_INT16, az, &stateCompressed.az)

LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat)           // compressed quaternion, see quatcompress.h

LOG_ADD(LOG_INT16, rateRoll, &stateCompressed.rateRoll)   // angular velocity - milliradians / sec
LOG_ADD(LOG_INT16, ratePitch, &stateCompressed.ratePitch)
LOG_ADD(LOG_INT16, rateYaw, &stateCompressed.rateYaw)
LOG_GROUP_STOP(stateEstimateZ)
