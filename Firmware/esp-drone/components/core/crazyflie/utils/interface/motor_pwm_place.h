/*
 * ESP-Drone Firmware
 *
 * motor_pwm_place.h - Where each motor's PWM pulse sits in the period
 *
 * The four motor channels share one LEDC timer (8 bits): the counter runs
 * 0..255 once per period (25 us at 40 kHz) and a channel is on from its
 * hpoint until hpoint + duty, duty 0..255. hpoint + duty <= 255 keeps every
 * pulse inside its period (none wraps over the end), and duty <= 255 leaves
 * at least one count low: no channel is ever full-on.
 *
 * Spread (2026-10-11, at or below 50 % duty): hpoint = id * (255 - duty) / 3,
 * the four pulse starts a third of the slack apart, so spin-up current peaks
 * do not coincide (coinciding starts pulled a half-charged cell to 2.5 V at
 * 2..10 % duty and browned the drone out).
 *
 * Aligned (user decision 2026-10-13, evening; above 50 % duty): at hover
 * duty (65-85 %) the spread has 1, 2, 3, 4, 3, 2, 1 motors on in turn, so
 * the battery current swings by up to three motor currents every period and
 * the cell and its connector (~0.15-0.2 ohm) heat with the square of it. The
 * aligned layout puts one diagonal pair, M1 and M3 (ids 0 and 2, the same
 * spin direction), at the period start (hpoint 0) and the other, M2 and M4
 * (ids 1 and 3), at its end (hpoint = 255 - duty): two motors on outside the
 * overlap and four inside it, the least swing possible without wrapping a
 * pulse (variance over a period of the number of motors on, all four at
 * 65/75/85 %: 1.33/1.37/1.07 spread -> 0.87/1.03/0.88 aligned, -35/-25/-18 %;
 * the mean, i.e. the average current, is unchanged). Same-spin pairs: a
 * difference between the start and end groups (supply sag seen at a
 * different point of the period) is a yaw torque, not roll or pitch.
 *
 * M1 (id 0) is at hpoint 0 and M4 (id 3) at 255 - duty in both layouts; only
 * M2 and M3 move. Each motor switches on its own duty with hysteresis:
 * aligned from MOTOR_PWM_PLACE_ALIGN_ON, spread again at or below
 * MOTOR_PWM_PLACE_ALIGN_OFF, unchanged in between. So the spread is kept at
 * or below 50 % whatever came before, and a motor near 50 % (the take-off
 * ramp, a landing) does not change layout every loop. Duty 0 is hpoint 0
 * (as before) and resets the motor to the spread.
 *
 * Only pulse positions change: each motor's duty, and so its average
 * voltage, is exactly what the controller commanded. Pure code, host-tested
 * by tools/tests/test_motor_pwm_place.c.
 */
#ifndef MOTOR_PWM_PLACE_H_
#define MOTOR_PWM_PLACE_H_

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_PLACE_TOP        255u  /* last counter value, largest duty (8-bit LEDC) */
#define MOTOR_PWM_PLACE_MOTORS     4u
#define MOTOR_PWM_PLACE_ALIGN_ON   141u  /* duty >= 141 (55 % of the period): aligned */
#define MOTOR_PWM_PLACE_ALIGN_OFF  128u  /* duty <= 128 (50 %): spread, as before */

typedef struct {
  uint8_t aligned;   /* bit id: motor id uses the aligned layout */
} motorPwmPlace_t;

/* Every motor spread (boot). */
void motorPwmPlaceInit(motorPwmPlace_t *p);

/* The hpoint for motor 'id' (0..3) at 'duty' (clamped to TOP), updating the
 * motor's layout first. Always hpoint + min(duty, TOP) <= TOP; duty 0 gives
 * 0. An id out of range gives 0 and changes nothing (the driver rejects it
 * before calling). */
uint32_t motorPwmPlaceHpoint(motorPwmPlace_t *p, uint32_t id, uint32_t duty);

/* Whether motor 'id' currently uses the aligned layout. */
bool motorPwmPlaceAligned(const motorPwmPlace_t *p, uint32_t id);

#endif /* MOTOR_PWM_PLACE_H_ */
