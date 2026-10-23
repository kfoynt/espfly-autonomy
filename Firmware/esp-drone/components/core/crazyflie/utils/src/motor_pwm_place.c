/*
 * ESP-Drone Firmware
 *
 * motor_pwm_place.c - Where each motor's PWM pulse sits in the period
 */
#include "motor_pwm_place.h"

/* The aligned layout's end group: M2 and M4. M1 and M3 start the period. */
#define END_GROUP ((1u << 1) | (1u << 3))

void motorPwmPlaceInit(motorPwmPlace_t *p)
{
  p->aligned = 0;
}

uint32_t motorPwmPlaceHpoint(motorPwmPlace_t *p, uint32_t id, uint32_t duty)
{
  if (id >= MOTOR_PWM_PLACE_MOTORS) {
    return 0;
  }
  const uint8_t bit = (uint8_t)(1u << id);
  if (duty > MOTOR_PWM_PLACE_TOP) {
    duty = MOTOR_PWM_PLACE_TOP;
  }
  if (duty >= MOTOR_PWM_PLACE_ALIGN_ON) {
    p->aligned |= bit;
  } else if (duty <= MOTOR_PWM_PLACE_ALIGN_OFF) {
    p->aligned &= (uint8_t)~bit;
  }
  if (duty == 0) {
    return 0;
  }
  const uint32_t slack = MOTOR_PWM_PLACE_TOP - duty;
  if (p->aligned & bit) {
    return (END_GROUP & bit) ? slack : 0;
  }
  return id * slack / (MOTOR_PWM_PLACE_MOTORS - 1);
}

bool motorPwmPlaceAligned(const motorPwmPlace_t *p, uint32_t id)
{
  return id < MOTOR_PWM_PLACE_MOTORS && (p->aligned & (1u << id)) != 0;
}
