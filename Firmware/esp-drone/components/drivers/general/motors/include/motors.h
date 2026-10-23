/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
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
 * Motors.h - Motor driver header file
 *
 */
#ifndef __MOTORS_H__
#define __MOTORS_H__

#include <stdint.h>
#include <stdbool.h>

#include "driver/ledc.h"

#include "config.h"
#include "stm32_legacy.h"


/******** Defines ********/

// CF2 PWM ripple is filtered better at 328kHz. At 168kHz the NCP702 regulator is affected.

#define MOTORS_PWM_BITS           LEDC_TIMER_8_BIT
#define MOTORS_PWM_PERIOD         ((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_FREQ_STOCK     15000    /* Hz, the stock ESP-FLY motor PWM */
#define MOTORS_PWM_FREQ_MIN       10000
#define MOTORS_PWM_FREQ_MAX       100000   /* 8-bit duty needs a 25.6 MHz timer clock (APB 80 MHz) */
#define MOTORS_TIM_BEEP_CLK_FREQ  4000000

// Compensate thrust depending on battery voltage so it will produce about the same
// amount of thrust independent of the battery voltage. Based on thrust measurement.

//#define ENABLE_THRUST_BAT_COMPENSATED


#define NBR_OF_MOTORS 4
// Motors IDs define
#define MOTOR_M1  0
#define MOTOR_M2  1
#define MOTOR_M3  2
#define MOTOR_M4  3

#ifdef CONFIG_TARGET_ESPLANE_V2_S2
    #define MOTOR3_GPIO  CONFIG_MOTOR01_PIN         // M1 for ESP32FC
    #define MOTOR4_GPIO  CONFIG_MOTOR02_PIN        // M2 for ESP32FC
    #define MOTOR1_GPIO  CONFIG_MOTOR03_PIN        // M3 for ESP32FC
    #define MOTOR2_GPIO  CONFIG_MOTOR04_PIN        // M4 for ESP32FC
#else
    #define MOTOR1_GPIO  CONFIG_MOTOR01_PIN         // M1 for ESP32FC
    #define MOTOR2_GPIO  CONFIG_MOTOR02_PIN        // M2 for ESP32FC
    #define MOTOR3_GPIO  CONFIG_MOTOR03_PIN        // M3 for ESP32FC
    #define MOTOR4_GPIO  CONFIG_MOTOR04_PIN        // M4 for ESP32FC
#endif

#define MOT_PWM_CH1  4      // Motor M1 pwmchannel
#define MOT_PWM_CH2  5      // Motor M2 pwmchannel
#define MOT_PWM_CH3  6      // Motor M3 pwmchannel
#define MOT_PWM_CH4  7      // Motor M4 pwmchannel     

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.2*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

// Sound defines
#define C4    262
#define DES4  277
#define D4    294
#define ES4   311
#define E4    330
#define F4    349
#define GES4  370
#define G4    392
#define AS4   415
#define A4    440
#define B4    466
#define H4    493
#define C5    523
#define DES5  554
#define D5    587
#define ES5   622
#define E5    659
#define F5    698
#define GES5  740
#define G5    783
#define AS5   830
#define A5    880
#define B5    932
#define H5    987
#define C6    1046
#define DES6  1108
#define D6    1174
#define ES6   1244
#define E6    1318
#define F6    1396
#define GES6  1479
#define G6    1567
#define AS6   1661
#define A6    1760
#define B6    1864
#define H6    1975
#define C7    2093
#define DES7  2217
#define D7    2349
#define ES7   2489
#define E7    2637
#define F7    2793
#define GES7  2959
#define G7    3135
#define AS7   3322
#define A7    3520
#define H7    3729
#define B7    3951

// Sound duration defines
#define EIGHTS 125
#define QUAD 250
#define HALF 500
#define FULL 1000
#define STOP 0

typedef enum {
    BRUSHED,
    BRUSHLESS
} motorsDrvType;

typedef struct {
    motorsDrvType drvType;

} MotorPerifDef;

/**
 * Motor mapping configurations
 */
//extern const MotorPerifDef* motorMapNoMotors[NBR_OF_MOTORS];
extern const MotorPerifDef *motorMapDefaultBrushed[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapDefaltConBrushless[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapBigQuadDeck[NBR_OF_MOTORS];
// extern const MotorPerifDef* motorMapBoltBrushless[NBR_OF_MOTORS];

/**
 * Test sound tones
 */
extern const uint16_t testsound[NBR_OF_MOTORS];
/*** Public interface ***/

// Early LOW initialization, called once from app_main before other tasks.
void motorsBenchInit(void);
bool motorsAreBenchLocked(void); // false: this separate build permits manual flight
bool motorsFlightReady(void);   // initialized and no driver fault or latched STOP
bool motorsIsStopped(void);
bool motorsHasFault(void);
void motorsEmergencyStop(void); // task-context; irreversible until reboot
/* The same latched STOP without ever blocking on the driver mutex (motor
 * watchdog, Mac KILL): pins low by register writes first, then the driver
 * state (try-lock; if the stabilizer holds the mutex, stopped and faulted are
 * set without it). Any task; not from an ISR. */
void motorsEmergencyStopNoWait(void);
/* Drives the four motor pins low through the GPIO matrix: register writes and
 * a ROM call only, no lock, no log, no flash access (IRAM). For the panic
 * path; it does not touch the driver state. */
void motorsPinsLowNoLock(void);
/* Whether any last-written motor duty is nonzero (no mutex, any task). */
bool motorsAnyDutyNonZero(void);
/* motorsFlightReady() without the driver mutex: initialised, no latched STOP,
 * no fault (volatile reads; any task, never blocks). The autonomy glue reads
 * it at 100 Hz from the stabilizer task (G1, row 7). */
bool motorsReadyNoLock(void);

bool pwm_timmer_init(void);
/**
 * Initialisation. Will set all motors ratio to 0%
 */
void motorsInit(const MotorPerifDef **motorMapSelect);

/**
 * DeInitialisation. Reset to default
 */
void motorsDeInit(const MotorPerifDef **motorMapSelect);

/**
 * Non-actuating initialization check. Never spins motors at startup.
 */
bool motorsTest(void);

/**
 * Set the PWM ratio of the motor 'id'
 */
void motorsSetRatio(uint32_t id, uint16_t ratio);

/**
 * Get checked LEDC duty in stock 16-bit units, not measured thrust.
 * Invalid ID/readback latches STOP and returns zero.
 */
int motorsGetRatio(uint32_t id);

/* Live pulse position and length of each motor channel (LEDC hpoint and
 * running duty, 0..255) and the motors in the aligned layout (bit per motor,
 * motor_pwm_place.h), for the USB 'pwm' report. False (zeros) before init or
 * after a STOP. */
bool motorsGetPulses(uint32_t hpoint[NBR_OF_MOTORS], uint32_t duty[NBR_OF_MOTORS], uint32_t *aligned);

/**
 * FreeRTOS Task to test the Motors driver
 */
void motorsTestTask(void *params);

/* Set PWM frequency for motor controller
 * This function will set all motors into a "beep"-mode,
 * each of the motor will turned on with a given ratio and frequency.
 * The higher the ratio the higher the given power to the motors.
 * ATTENTION: To much ratio can push your crazyflie into the air and hurt you!
 * Example:
 *     motorsBeep(true, 1000, (uint16_t)(72000000L / frequency)/ 20);
 *     motorsBeep(false, 0, 0); *
 * */
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

/**
 * Bench/calibration motor inhibit. While set, motorsSetRatio() writes zero
 * duty. Unlike motorsEmergencyStop() it does not latch; it is never set
 * during normal flight.
 */
void motorsSetInhibit(bool inhibit);
bool motorsIsInhibited(void);

/* Motor PWM frequency in Hz (MOTORS_PWM_FREQ_MIN..MAX, stock 15000). Setting it
 * is refused (false) unless every motor output is zero. */
bool motorsSetPwmFreq(uint32_t hz);
uint32_t motorsGetPwmFreq(void);

#endif /* __MOTORS_H__ */
