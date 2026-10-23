/*
 * ESP-Drone Firmware
 *
 * ground_cool.h - Ground prop-wash cooling spin (pure helper)
 *
 * The IMU sits at 47-53 C on the ground (board heat, worst right off the USB
 * charger) and cools 10-16 C in flight under the prop wash; the accelerometer
 * and gyro zeros move with it, so a take-off from a hot IMU flies the first
 * minute with a drifting vertical reference (docs/autonomy-a2b.md 36). The
 * cooling spin runs all four motors at one low, equal PWM, well below
 * lift-off, for a few tens of seconds before the take-off, so that the IMU is
 * closer to its flight temperature when the sortie's references are taken.
 * The stabilizer runs it from IDLE only (ground_cool_glue.c): the thrust
 * setpoint is the spin's PWM and the attitude outputs are zeroed, so each
 * motor gets exactly the PWM (powerDistribution adds nothing); every existing
 * barrier (the restart lock, 'gcap', the soft start, the motor driver's STOP
 * latch and kill) stays in front of it.
 *
 * Guards. Start: autonomy IDLE, every motor duty 0 and no thrust requested,
 * on the ground and resting (gyro < 10 dps, |a| within 0.15 g), the motor
 * driver ready (no STOP latch, no restart lock), the resting battery at or
 * above the autonomy's vmin, PWM and duration in range. Abort at once: tilt
 * over GROUND_COOL_TILT_MAX_DEG, any gyro over GROUND_COOL_GYRO_MAX_DPS,
 * |a| off 1 g by more than GROUND_COOL_ACC_TOL_G (after the first
 * GROUND_COOL_SPINUP_MS, when the motors come up), a kill, a stop, autonomy
 * leaving IDLE, the motor driver no longer ready. The numbers: hover is
 * ~40000 of 65535 and lift-off 30000-36000 on this drone (vert_ctrl.h: the
 * idle spin before every take-off runs 0.45 x hover, up to 20000, for
 * 0.8 s); the default 12000 is 30 % of hover, the maximum 20000 the idle
 * spin's cap ('gcap' bench spins run at 15000, docs/autonomy-bench.md B).
 *
 * No ESP-IDF dependencies; unit tested by tools/tests/test_ground_cool.c.
 */
#ifndef GROUND_COOL_H_
#define GROUND_COOL_H_

#include <stdbool.h>
#include <stdint.h>

#define GROUND_COOL_PWM_DEFAULT   12000u   /* ~30 % of hover */
#define GROUND_COOL_PWM_MIN       1000u
#define GROUND_COOL_PWM_MAX       20000u   /* the idle spin's cap; lift-off needs >= 30000 */
#define GROUND_COOL_S_DEFAULT     20u
#define GROUND_COOL_S_MAX         40u
#define GROUND_COOL_TILT_MAX_DEG  8.0f
#define GROUND_COOL_GYRO_MAX_DPS  30.0f
#define GROUND_COOL_ACC_TOL_G     0.15f
#define GROUND_COOL_SPINUP_MS     300u     /* the accelerometer norm is not checked while the motors come up */
#define GROUND_COOL_REST_GYRO_DPS 10.0f    /* the start's resting test (the stabilizer's) */

typedef enum {
  GC_OK = 0,
  GC_E_RANGE,          /* PWM or seconds out of range */
  GC_E_RUNNING,        /* a spin is running */
  GC_E_AUTO,           /* autonomy is not IDLE */
  GC_E_THRUST,         /* thrust requested or a motor duty nonzero */
  GC_E_GROUND,         /* not on the ground, or not resting */
  GC_E_LOCKED,         /* the motor driver's STOP latch or the restart lock */
  GC_E_BATT,           /* battery below vmin */
} groundCoolRefusal_t;

typedef enum {
  GC_R_NONE = 0,       /* running, or never ran */
  GC_R_DONE,           /* the time ran out */
  GC_R_STOP,           /* 'cool stop' */
  GC_R_KILL,
  GC_R_TILT,
  GC_R_GYRO,
  GC_R_ACC,
  GC_R_AUTO,           /* autonomy left IDLE */
  GC_R_LOCKED,         /* the motor driver stopped being ready */
} groundCoolEnd_t;

/* The inputs of a start or a step, read by the caller from the stabilizer's loop. */
typedef struct {
  bool autonomyActive;
  bool motorsReady;        /* motorsReadyNoLock() && !restartLocked */
  bool dutyZero;           /* every motor duty 0 (start) */
  bool thrustZero;         /* no thrust requested by anyone (start) */
  bool onGround;           /* stabilizerOnGround() (start) */
  float vbat;              /* V (start) */
  float vminV;             /* the autonomy's vmin, V (start) */
  float tiltDeg;
  float gyroDps[3];        /* body, filtered */
  float accNormG;
  uint32_t killCount;      /* the link's kill counter: a change aborts */
  float imuTempC;
} groundCoolIn_t;

typedef struct {
  bool running;
  uint16_t pwm;
  uint16_t seconds;
  uint32_t startMs;
  uint32_t killSeen;
  uint8_t end;             /* groundCoolEnd_t of the latest run */
  bool stopReq;            /* 'cool stop' or a kill, served at the next step */
  bool killReq;
  float tStartC, tEndC;    /* the IMU temperature at the start and at the end (or now) */
  float tiltMaxDeg, gyroMaxDps;   /* over the run */
  uint32_t elapsedMs;      /* of the latest run */
  uint32_t runs;           /* started since boot */
} groundCool_t;

void groundCoolInit(groundCool_t *gc);
/* Start a spin: pwm (GROUND_COOL_PWM_MIN..MAX) for seconds (1..GROUND_COOL_S_MAX) with the guards above on in.
 * GC_OK and running, else the refusal and nothing changes. */
groundCoolRefusal_t groundCoolStart(groundCool_t *gc, const groundCoolIn_t *in, uint32_t pwm, uint32_t seconds,
                                    uint32_t nowMs);
/* One stabilizer loop while running: the thrust to request (the PWM, or 0 once it ended this step). The abort
 * tests and the time run here; the end reason is left in gc->end. Returns 0 when not running. */
float groundCoolStep(groundCool_t *gc, const groundCoolIn_t *in, uint32_t nowMs);
/* Asked to stop (served at the next step: the loop owns the state). */
void groundCoolStop(groundCool_t *gc);
void groundCoolKill(groundCool_t *gc);
const char *groundCoolRefusalName(groundCoolRefusal_t r);
const char *groundCoolEndName(groundCoolEnd_t e);

#endif /* GROUND_COOL_H_ */
