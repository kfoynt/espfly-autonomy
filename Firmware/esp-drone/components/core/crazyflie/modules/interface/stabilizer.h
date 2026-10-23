/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * stabilizer.h: Stabilizer orchestrator
 */
#ifndef STABILIZER_H_
#define STABILIZER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "estimator.h"

#define EMERGENCY_STOP_TIMEOUT_DISABLED (-1)

// Read-only bench snapshot. Native flight axes; attitude order roll/pitch/yaw.
// Times use the flight usecTimestamp clock. Execution time excludes publication.
typedef struct {
  uint64_t timestamp_us;
  uint64_t imu_timestamp_us;
  uint32_t loop_count;
  uint32_t last_loop_gap_us;
  uint32_t max_loop_gap_us;
  uint32_t loop_gap_over_1500us;
  uint32_t loop_gap_over_2500us;
  uint32_t loop_gap_over_5000us;
  uint32_t last_loop_exec_us;
  uint32_t max_loop_exec_us;
  uint32_t sensor_to_output_us;
  /* since boot, for loop-rate bench tests (deltas between two reads): the sums
   * and counts of the loop's execution time and of the IMU-interrupt-to-output
   * latency, and the loops whose latency exceeded 800 us */
  uint64_t exec_sum_us;
  uint64_t sensor_to_output_sum_us;
  uint32_t sensor_to_output_n;
  uint32_t sensor_to_output_over_800us;
  float acceleration_g[3];
  float angular_rate_dps[3];
  float attitude_deg[3];
  bool motors_locked;
  bool calibrated;
} stabilizerBenchTelemetry_t;

// Atomic copy for camera/HTTP tasks. False before the first completed loop;
// the returned snapshot is then zero except for the permanent motor-lock flag.
bool stabilizerGetBenchTelemetry(stabilizerBenchTelemetry_t *out);


/**
 * Initialize the stabilizer subsystem and launch the stabilizer loop task.
 * The stabilizer loop task will wait on systemWaitStart() before running.
 */
void stabilizerInit(StateEstimatorType estimator);

/**
 * Test the stabilizer subsystem. Calls test for all the stabilizer related
 * sensors.
 * @return True if all test has passed. False otherwise.
 */
bool stabilizerTest(void);

/* Diagnostic thrust cap (0 = off): the thrust setpoint is clamped to it before
 * the controller. Not persisted here (see thrustCapStoreSave). */
void stabilizerSetThrustCap(uint16_t cap);
uint16_t stabilizerGetThrustCap(void);

/* Restart lock: after a reset other than power-on, thrust is held at 0 for
 * the rest of the boot (until the battery is replugged). Turning it off also
 * releases a current lock. */
void stabilizerSetRestartLock(bool on);
bool stabilizerRestartLockEnabled(void);
bool stabilizerRestartLocked(void);

/* Take-off thrust ramp after a ground stop (thrust_softstart.h). */
/* Take-off window after a ground start (thrust leaving zero after a rest):
 * at least 3 s, held open until 1.5 s after thrust first reaches the lift-off
 * level, at most 8 s. And the count of ground starts since boot. Any task may
 * call these. */
bool stabilizerTakeoffWindow(void);
/* The same with other lengths: open for minMs after the ground start and
 * until afterLiftMs past the lift-off thrust level (at most 8 s). */
bool stabilizerTakeoffPhase(uint32_t minMs, uint32_t afterLiftMs);
/* Whether the latest ground start was a cold take-off: the first since boot,
 * or without ~4 s of flying at high thrust in the 2 minutes before it. */
bool stabilizerTakeoffCold(void);
/* Motor-on time of the latest flight since boot (its ground start to thrust 0), ms; 0 before the first.
 * One word the stabilizer task writes, read without a call: autonomy.c's IRAM slow inputs take it every
 * 10th active loop (a getter cost 16 B of IRAM, which this change's budget did not have). */
extern volatile uint32_t stabilizerLastFlightMotorMsWord;
static inline uint32_t stabilizerLastFlightMotorMs(void)
{
  return stabilizerLastFlightMotorMsWord;
}
/* Zero thrust and resting on the ground since (the soft-start's ground stop:
 * >= 0.5 s still at about 1 g), or not yet flown since boot. */
bool stabilizerOnGround(void);
uint32_t stabilizerTakeoffCount(void);
/* Autonomy (docs/autonomy-a2b.md section 5): the vertical controller's
 * detected lift-off (vzOutput_t.liftoffNow) counts as the take-off window's
 * lift-off, so the window (camera standby, low Wi-Fi TX, the photo-push
 * pause) ends TAKEOFF_AFTER_LIFT_MS after it. The RAMP reaches
 * TAKEOFF_LIFT_THRUST (40000) only near its plateau, after the lift-off, and
 * an autonomous hover sits near it on a fresh battery (165k / 4.0 V loaded =
 * 41k; the A2a prior gave 149.5k / 3.75 V = 39.9k, which never reached it).
 * No effect before the first ground start or once a lift-off is recorded for
 * the current ground start. Stabilizer task only. */
void stabilizerNoteLiftoff(void);

void stabilizerSetSoftStart(bool on);
bool stabilizerSoftStartEnabled(void);

/* Bench fault injection for the motor watchdog (USB 'mguard test hang',
 * BENCH only, gated by the caller): the next stabilizer loop busy-waits ms
 * milliseconds, leaving the motors at their last duty. */
void stabilizerFaultHang(uint32_t ms);

/* Largest stabilizer loop gap (us) and number of gaps > 2.5 ms since the last
 * call, then reset. Call only from the stabilizer task (the flight log does). */
void stabilizerTakeLoopGaps(uint32_t *maxUs, uint32_t *over2500);

/**
 * Enable emergency stop, will shut-off energy to the motors.
 */
void stabilizerSetEmergencyStop();

/**
 * Disable emergency stop, will enable energy to the motors.
 */
void stabilizerResetEmergencyStop();

/* Whether the emergency stop flag is set (any task; a plain read). */
bool stabilizerEmergencyStopActive(void);

/**
 * Restart the countdown until emergercy stop will be enabled.
 *
 * @param timeout Timeout in stabilizer loop tick. The stabilizer loop rate is
 *                RATE_MAIN_LOOP.
 */
void stabilizerSetEmergencyStopTimeout(int timeout);


#endif /* STABILIZER_H_ */
