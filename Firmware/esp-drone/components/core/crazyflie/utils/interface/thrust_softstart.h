/*
 * ESP-Drone Firmware
 *
 * thrust_softstart.h - Gentle take-off from a ground stop (thrust slew limit)
 *
 * A brushed motor at standstill has no back-EMF, so a sudden command draws
 * close to stall current; on 2026-10-11 a half-charged battery collapsed from
 * 3.46 V to 2.43 V within 50-70 ms of a spin-up at ~10 % throttle and reset
 * the drone (a USB-powered XIAO survived the same dips). After a ground stop
 * the collective thrust setpoint may rise by at most THRUST_SOFT_STEP per
 * control loop (0 -> THRUST_SOFT_END in 200 ms; the app's 80 % cap in ~0.45 s).
 * It is applied to the setpoint before the controller, so the integrator
 * ground hold, torque-trim ramp and in-flight sensor offsets all see the
 * thrust the motors really get, and roll/pitch/yaw corrections are untouched.
 *
 * A ground stop is THRUST_SOFT_ARM_LOOPS loops at zero thrust with the drone
 * at rest (gyro still, about 1 g): a falling or tumbling drone never arms it,
 * and boot counts as a ground stop. The ramp ends once the setpoint has caught
 * up with a request of at least THRUST_SOFT_END, or after THRUST_SOFT_FALL_LOOPS
 * of free fall (a hand-launched or dropped drone needs full thrust at once).
 * Lower requests during the ramp lower the ramp level by at most one step per
 * loop (motors spinning down), so a long low hold ramps again while a short
 * dip does not. Never raises a request. Pure code, host-tested by
 * tools/tests/test_thrust_softstart.c.
 */
#ifndef THRUST_SOFTSTART_H_
#define THRUST_SOFTSTART_H_

#include <stdbool.h>
#include <stdint.h>

#define THRUST_SOFT_ARM_LOOPS  500     /* 0.5 s at rest with zero thrust (1 kHz loop) */
#define THRUST_SOFT_END        23040   /* ~35 % of full scale: motors clearly spinning */
#define THRUST_SOFT_RAMP_LOOPS 200     /* slope: 0 -> END in 200 loops */
#define THRUST_SOFT_STEP ((THRUST_SOFT_END + THRUST_SOFT_RAMP_LOOPS - 1) / THRUST_SOFT_RAMP_LOOPS)
#define THRUST_SOFT_FALL_LOOPS 20      /* free fall this long ends the ramp (hand launch, slip off a ledge) */

typedef struct {
  uint16_t restLoops;   /* consecutive zero-thrust loops at rest, saturating */
  uint16_t fallLoops;   /* consecutive loops in free fall, saturating */
  uint16_t level;       /* ramp level: thrust allowed last loop */
  bool active;
} thrustSoftStart_t;

void thrustSoftStartInit(thrustSoftStart_t *s);

/* One control loop. 'request' is the thrust setpoint (0..65535); 'resting'
 * whether the IMU shows the drone at rest (still, ~1 g); 'falling' whether it
 * is in free fall (well under 1 g). Returns the thrust to use. */
uint16_t thrustSoftStartApply(thrustSoftStart_t *s, uint16_t request, bool resting, bool falling, bool enabled);

/* True while the ramp is armed or ramping: on the ground since the last rest
 * (also with the ramp disabled). The stabilizer's take-off window keys on it. */
bool thrustSoftStartActive(const thrustSoftStart_t *s);

#endif /* THRUST_SOFTSTART_H_ */
