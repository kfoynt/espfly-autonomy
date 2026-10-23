/*
 * ESP-Drone Firmware
 *
 * ground_cool_glue.h - The ground cooling spin in the stabilizer (ground_cool.h, docs/autonomy-a2b.md 36.3)
 *
 * The console ('cool', usb_diag.c) starts and stops it; the stabilizer task
 * runs it from IDLE: while a spin runs, the setpoint is replaced by the
 * spin's PWM with a level attitude and the controller's attitude outputs are
 * zeroed, so every motor gets exactly the PWM through the usual thrust path
 * (restart lock, 'gcap', soft start, powerDistribution, the driver's STOP
 * latch). The per-loop cost when no spin runs is one load and a branch.
 * The accelerometer compensation's reference is latched at the start
 * (sensorsMpu6050AccTcLatch(true)), so the first ground re-zero after the
 * spin logs the still accelerometer's step over the cooling ('atc show',
 * cool=1): a bench measurement of k.
 */
#ifndef GROUND_COOL_GLUE_H_
#define GROUND_COOL_GLUE_H_

#include <stdbool.h>
#include <stdint.h>

#include "ground_cool.h"
#include "stabilizer_types.h"

/* The spin runs (one word, any task; the stabilizer's per-loop test). */
extern volatile bool groundCoolGlueRunningWord;
static inline bool groundCoolGlueRunning(void)
{
  return groundCoolGlueRunningWord;
}

/* Console (any task but the stabilizer): start pwm for seconds; false with the refusal's name in *why. The
 * guards of ground_cool.h on the stabilizer's latest state. */
bool groundCoolGlueStart(uint32_t pwm, uint32_t seconds, const char **why);
void groundCoolGlueStop(void);
/* The link's kill (autonomyKillFromLink) and the console's: the spin ends at the next loop. */
void groundCoolGlueKill(void);

typedef struct {
  groundCool_t gc;          /* a copy */
  bool running;
  float tempC;              /* the IMU now */
  uint32_t leftMs;          /* of the run */
} groundCoolGlueStatus_t;
void groundCoolGlueStatus(groundCoolGlueStatus_t *st);

/* Stabilizer task, every loop while groundCoolGlueRunning(): the spin's step on this loop's sensors and state;
 * replaces *setpoint while the spin runs (true), else leaves it (the spin ended this loop or earlier). */
bool groundCoolGlueArbitrate(setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, uint32_t nowMs);
/* After the controller: the attitude outputs zeroed while the spin runs, so the four motors get the PWM alone. */
void groundCoolGlueLevelOutputs(control_t *control);

#endif /* GROUND_COOL_GLUE_H_ */
