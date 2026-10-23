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
 * motors.c - Motor driver
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_rom_gpio.h"
#include "hal/ledc_ll.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/ledc_struct.h"
#include "motors.h"
#include "motor_pwm_place.h"
#include "log.h"

#if !defined(DRONE_MANUAL_FLIGHT_CAMERA) || DRONE_MANUAL_FLIGHT_CAMERA != 1
#error "This motor driver belongs only to the separate manual flight/camera build"
#endif
#if MOTOR1_GPIO != 7 || MOTOR2_GPIO != 4 || MOTOR3_GPIO != 3 || MOTOR4_GPIO != 1
#error "Manual flight requires the verified ESP-FLY motor pin map 7/4/3/1"
#endif
#if defined(ENABLE_THRUST_BAT_COMPENSATED)
#error "Retain the stock ESP-FLY uncompensated motor command mapping"
#endif
_Static_assert(MOTORS_PWM_BITS == LEDC_TIMER_8_BIT, "Retain stock 8-bit motor PWM");
_Static_assert(MOTORS_PWM_PERIOD == MOTOR_PWM_PLACE_TOP && NBR_OF_MOTORS == MOTOR_PWM_PLACE_MOTORS,
               "The pulse placement assumes the 8-bit, four-motor driver");
/* ledcWriteLocked() writes the LEDC channel registers the way IDF 5.0.x's
 * ledc.c does, on the ESP32-S3 register map: re-check it before moving on. */
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "ledcWriteLocked() is checked against the ESP32-S3 LEDC only"
#endif
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0) || ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
#error "ledcWriteLocked() mirrors IDF 5.0.x ledc.c: re-check it against this IDF's driver"
#endif

/* Stock electrical mapping: M1..M4 -> GPIO7/4/3/1, LEDC channels4..7,
 * low-speed timer0, 15 kHz, 8 bits. Camera uses timer2/channel2 independently.
 * Only this task-context driver writes motor LEDC channels. One mutex serializes
 * updates, readback and permanent STOP; no queued callback can re-enable them.
 * The existing commander owns command freshness/zero-thrust startup unlocking.
 * This module provides NO automatic motor test, tone, retry, or fault recovery.
 */
static const gpio_num_t pins[NBR_OF_MOTORS] = {MOTOR1_GPIO,MOTOR2_GPIO,MOTOR3_GPIO,MOTOR4_GPIO};
static const ledc_channel_t channels[NBR_OF_MOTORS] = {MOT_PWM_CH1,MOT_PWM_CH2,MOT_PWM_CH3,MOT_PWM_CH4};
const uint32_t MOTORS[] = {MOTOR_M1,MOTOR_M2,MOTOR_M3,MOTOR_M4};
const uint16_t testsound[NBR_OF_MOTORS] = {A4,A5,F5,D5};
const MotorPerifDef **motorMap;
uint32_t motor_ratios[NBR_OF_MOTORS]; // quantized commanded duty in stock 16-bit units
static SemaphoreHandle_t mutex;
static bool earlyAttempted, isInit, timerReady;
/* Volatile: the no-wait stop (motor watchdog, esp_timer task) may set them
 * without the mutex while the stabilizer is inside motorsSetRatio(). */
static volatile bool stopped, faulted;
static bool configured[NBR_OF_MOTORS];
static volatile bool inhibited; // bench/calibration hold; forces zero duty, never latches
/* Battery-friendly drive (2026-10-11): a selectable PWM frequency (stock
 * 15 kHz), switchable at runtime while the motors are stopped and set from
 * NVS at init. (The take-off thrust ramp lives in the stabilizer.) */
static volatile uint32_t pwmFreqHz = MOTORS_PWM_FREQ_STOCK;

/* Caller owns the mutex (or is in early startup before task creation). LOW is
 * preloaded before GPIO configuration detaches LEDC. Errors leave the software
 * fault latched; UINT32_MAX explicitly means output state was not established.
 */
static bool gpioLowLocked(void)
{
    bool low=true;
    uint64_t mask=0;
    for (unsigned i=0;i<NBR_OF_MOTORS;++i) {
        low &= gpio_set_level(pins[i],0)==ESP_OK;
        mask |= UINT64_C(1)<<pins[i];
    }
    const gpio_config_t config = {
        .pin_bit_mask=mask, .mode=GPIO_MODE_OUTPUT,
        .pull_up_en=GPIO_PULLUP_DISABLE, .pull_down_en=GPIO_PULLDOWN_ENABLE,
        .intr_type=GPIO_INTR_DISABLE,
    };
    low &= gpio_config(&config)==ESP_OK;
    if (!low) faulted=true;
    for (unsigned i=0;i<NBR_OF_MOTORS;++i) motor_ratios[i]=low ? 0 : UINT32_MAX;
    return low;
}

static void stopLocked(bool fault)
{
    stopped=true;
    faulted |= fault;
    (void)gpioLowLocked();
    for (unsigned i=0;i<NBR_OF_MOTORS;++i) {
        if (configured[i] && ledc_stop(LEDC_LOW_SPEED_MODE,channels[i],0)!=ESP_OK)
            faulted=true;
    }
}

void motorsBenchInit(void)
{
    /* app_main calls once before tasks. Later calls are deliberately no-ops:
     * never reconfigure GPIO or reconnect LEDC after initialization/STOP. */
    if (earlyAttempted) return;
    earlyAttempted=true;
    mutex=xSemaphoreCreateMutex();
    if (!gpioLowLocked() || !mutex) {
        stopped=true;
        faulted=true;
    }
}

static bool lockDriver(void)
{
    // Before the required early initialization, refuse all activation.
    if (!mutex) return false;
    return xSemaphoreTake(mutex,portMAX_DELAY)==pdTRUE;
}
static void unlockDriver(void) { xSemaphoreGive(mutex); }
static bool readyLocked(void) { return isInit && !stopped && !faulted; }

bool motorsAreBenchLocked(void) { return false; }
bool motorsFlightReady(void)
{
    if (!lockDriver()) return false;
    bool ready=readyLocked(); unlockDriver(); return ready;
}
bool motorsIsStopped(void)
{
    if (!lockDriver()) return true;
    bool value=stopped; unlockDriver(); return value;
}
bool motorsHasFault(void)
{
    if (!lockDriver()) return true;
    bool value=faulted; unlockDriver(); return value;
}
void motorsEmergencyStop(void)
{
    if (!lockDriver()) {
        stopped=true; faulted=true; (void)gpioLowLocked(); return;
    }
    stopLocked(false); unlockDriver();
}

/* The motor pin map as immediates: the panic path may run with the flash
 * cache off, so nothing here may read pins[] (flash .rodata). */
#define MOTOR_PIN_MASK ((1u<<MOTOR1_GPIO)|(1u<<MOTOR2_GPIO)|(1u<<MOTOR3_GPIO)|(1u<<MOTOR4_GPIO))
_Static_assert(MOTOR1_GPIO<32 && MOTOR2_GPIO<32 && MOTOR3_GPIO<32 && MOTOR4_GPIO<32,
               "motor pins must be in the low GPIO bank");

void IRAM_ATTR motorsPinsLowNoLock(void)
{
    /* Preload LOW, route each pin from its LEDC signal to the plain GPIO
     * output (what gpio_config() does for an output), enable the output. */
    REG_WRITE(GPIO_OUT_W1TC_REG, MOTOR_PIN_MASK);
    esp_rom_gpio_connect_out_signal(MOTOR1_GPIO, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(MOTOR2_GPIO, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(MOTOR3_GPIO, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(MOTOR4_GPIO, SIG_GPIO_OUT_IDX, false, false);
    REG_WRITE(GPIO_ENABLE_W1TS_REG, MOTOR_PIN_MASK);
}

/* Motor watchdog / Mac KILL stop: never blocks. The pins go low first, by
 * register writes, so neither a stabilizer holding the mutex nor a slow
 * console can delay them. No gpio_config() here (it logs four ESP_LOGI
 * lines, which can spin up to 50 ms on a full USB-Serial-JTAG FIFO, in the
 * caller's 3.5 KB esp_timer stack). Once rerouted, LEDC writes no longer
 * reach the pins (only ledc_channel_config() routes a pin to LEDC, and
 * motorsInit() refuses after a STOP) and readyLocked() refuses every later
 * write: latched until reboot, like motorsEmergencyStop(). With the mutex
 * busy the channels are not ledc_stop()ped: they keep counting inside the
 * LEDC but drive no pin. */
void motorsEmergencyStopNoWait(void)
{
    motorsPinsLowNoLock();
    if (mutex && xSemaphoreTake(mutex,0)==pdTRUE) {
        stopped=true;
        for (unsigned i=0;i<NBR_OF_MOTORS;++i) {
            if (configured[i] && ledc_stop(LEDC_LOW_SPEED_MODE,channels[i],0)!=ESP_OK)
                faulted=true;
            motor_ratios[i]=0;
        }
        unlockDriver();
    } else {
        // stopped is published before the zeros (seq_cst); a holder inside motorsSetRatio
        // re-checks it after storing its duty, so no stale nonzero reading survives.
        __atomic_store_n(&stopped,true,__ATOMIC_SEQ_CST);
        faulted=true;
        for (unsigned i=0;i<NBR_OF_MOTORS;++i) __atomic_store_n(&motor_ratios[i],0,__ATOMIC_SEQ_CST);
    }
}

bool IRAM_ATTR motorsReadyNoLock(void)
{
    // Volatile reads, no mutex (autonomy glue, 100 Hz): readyLocked() without the lock.
    return isInit && !stopped && !faulted;
}

bool motorsAnyDutyNonZero(void)
{
    // Aligned 32-bit reads, no mutex: callable from any task.
    const volatile uint32_t *r=motor_ratios;
    uint32_t any=0;
    for (unsigned i=0;i<NBR_OF_MOTORS;++i) any|=r[i];
    return any!=0;
}

static bool timerInitLocked(void)
{
    if (stopped || faulted) return false;
    if (timerReady) return true;
    const ledc_timer_config_t config = {
        .duty_resolution=MOTORS_PWM_BITS, .freq_hz=MOTORS_PWM_FREQ_STOCK,
        .speed_mode=LEDC_LOW_SPEED_MODE, .timer_num=LEDC_TIMER_0,
        .clk_cfg=LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&config)!=ESP_OK) { stopLocked(true); return false; }
    timerReady=true; return true;
}
bool pwm_timmer_init(void)
{
    if (!lockDriver()) return false;
    const bool result=timerInitLocked(); unlockDriver(); return result;
}

void motorsInit(const MotorPerifDef **motorMapSelect)
{
    if (!lockDriver()) return;
    if (isInit || stopped || faulted) { unlockDriver(); return; }
    motorMap=motorMapSelect;
    if (timerInitLocked()) {
        bool success=true;
        for (unsigned i=0;i<NBR_OF_MOTORS;++i) {
            const ledc_channel_config_t config = {
                .channel=channels[i], .duty=0, .gpio_num=pins[i],
                .speed_mode=LEDC_LOW_SPEED_MODE, .timer_sel=LEDC_TIMER_0,
                .intr_type=LEDC_INTR_DISABLE, .hpoint=0,
            };
            if (ledc_channel_config(&config)!=ESP_OK) {
                // A failed channel call may have partially configured its GPIO;
                // detach every pin, not merely the successfully registered ones.
                stopLocked(true); success=false; break;
            }
            configured[i]=true;
        }
        isInit=success;
    }
    unlockDriver();
}
void motorsDeInit(const MotorPerifDef **motorMapSelect)
{
    (void)motorMapSelect; motorsEmergencyStop();
}
bool motorsTest(void) { return motorsFlightReady(); } // never spins at startup

/* All four channels share one timer. With every hpoint at 0 the four motors
 * switch on at the same instant each period; at spin-up (no back-EMF) those
 * coinciding pulses pulled the battery to 2.5 V at 2..10 % duty on
 * 2026-10-11 and browned the drone out. So the pulse starts are spread over
 * the slack of the period at or below 50 % duty, and above it (user decision
 * 2026-10-13, evening) one diagonal pair starts the period and the other ends
 * it: 2..4 motors on instead of 1..4 at hover duty, 18-35 % less current
 * variance (motor_pwm_place.h). hpoint + duty stays within the period, so
 * a pulse never wraps into the next one; duty 0 keeps hpoint 0, exactly as
 * before. Written under the driver mutex. */
static motorPwmPlace_t place;

/* One channel's duty and hpoint, taken by the hardware at the same period
 * start. IDF 5.0.7's ledc_set_duty_with_hpoint() sets LEDC_PARA_UP right
 * after writing the hpoint, and ledc_update_duty() sets it again with
 * LEDC_DUTY_START (which loads the duty), each in its own critical section.
 * A timer overflow between the two runs one period with the new hpoint and
 * the old duty; when a right-aligned pulse shortens (M4 before this change,
 * M2 and M4 now above 50 %), that sum passes 255, the pulse does not end in
 * its period and stays on into the next one. Here every field is written
 * first and LEDC_PARA_UP is set once (as IDF 5.3's driver does: its
 * ledc_duty_config() no longer sets it), so hpoint and duty change together
 * at the next overflow. The register values are those of the two IDF calls:
 * hpoint, the duty's integer part (no dithering), no fade (increase,
 * num/cycle/scale 0), output enabled, duty start. Only this driver writes
 * the motor channels, under its mutex, so masking this core's interrupts
 * keeps the sequence uninterrupted (no spinlock needed). Flash code like the
 * IDF calls it replaces (motorsSetRatio() is flash). */
static void ledcWriteLocked(ledc_channel_t ch, uint32_t duty, uint32_t hpoint)
{
    const UBaseType_t irq=portSET_INTERRUPT_MASK_FROM_ISR();
    ledc_ll_set_hpoint(&LEDC,LEDC_LOW_SPEED_MODE,ch,hpoint);
    ledc_ll_set_duty_int_part(&LEDC,LEDC_LOW_SPEED_MODE,ch,duty);
    ledc_ll_set_duty_direction(&LEDC,LEDC_LOW_SPEED_MODE,ch,LEDC_DUTY_DIR_INCREASE);
    ledc_ll_set_duty_num(&LEDC,LEDC_LOW_SPEED_MODE,ch,0);
    ledc_ll_set_duty_cycle(&LEDC,LEDC_LOW_SPEED_MODE,ch,0);
    ledc_ll_set_duty_scale(&LEDC,LEDC_LOW_SPEED_MODE,ch,0);
    ledc_ll_set_sig_out_en(&LEDC,LEDC_LOW_SPEED_MODE,ch,true);
    ledc_ll_set_duty_start(&LEDC,LEDC_LOW_SPEED_MODE,ch,true);
    ledc_ll_ls_channel_update(&LEDC,LEDC_LOW_SPEED_MODE,ch);
    portCLEAR_INTERRUPT_MASK_FROM_ISR(irq);
}

void motorsSetRatio(uint32_t id, uint16_t ratio)
{
    if (!lockDriver()) return;
    if (!readyLocked()) { unlockDriver(); return; }
    if (id>=NBR_OF_MOTORS) { stopLocked(true); unlockDriver(); return; }
    const uint32_t duty=inhibited ? 0 : ratio>>(16-MOTORS_PWM_BITS);
    const uint32_t hpoint=motorPwmPlaceHpoint(&place,id,duty);
    if (duty>MOTORS_PWM_PERIOD || hpoint+duty>MOTORS_PWM_PERIOD) {
        stopLocked(true);   // never by construction (host-tested); never write a wrapping pulse
    } else {
        ledcWriteLocked(channels[id],duty,hpoint);
        // Store, then re-check: a no-wait stop that landed in between (pins already low)
        // must not leave a nonzero reading for motorsAnyDutyNonZero(). Pairs with the
        // seq_cst stores in motorsEmergencyStopNoWait().
        __atomic_store_n(&motor_ratios[id],duty<<(16-MOTORS_PWM_BITS),__ATOMIC_SEQ_CST);
        if (__atomic_load_n(&stopped,__ATOMIC_SEQ_CST)) __atomic_store_n(&motor_ratios[id],0,__ATOMIC_SEQ_CST);
    }
    unlockDriver();
}
int motorsGetRatio(uint32_t id)
{
    if (!lockDriver()) return 0;
    int result=0;
    if (readyLocked()) {
        if (id>=NBR_OF_MOTORS) stopLocked(true);
        else {
            const uint32_t duty=ledc_get_duty(LEDC_LOW_SPEED_MODE,channels[id]);
            if (duty>MOTORS_PWM_PERIOD) stopLocked(true);
            else result=(int)(duty<<(16-MOTORS_PWM_BITS));
        }
    }
    unlockDriver(); return result;
}
bool motorsGetPulses(uint32_t hpoint[NBR_OF_MOTORS], uint32_t duty[NBR_OF_MOTORS], uint32_t *aligned)
{
    if (!lockDriver()) return false;
    const bool ok=readyLocked();
    uint32_t bits=0;
    for (unsigned i=0;i<NBR_OF_MOTORS;++i) {
        hpoint[i]=ok ? (uint32_t)ledc_get_hpoint(LEDC_LOW_SPEED_MODE,channels[i]) : 0;
        duty[i]=ok ? ledc_get_duty(LEDC_LOW_SPEED_MODE,channels[i]) : 0;
        if (motorPwmPlaceAligned(&place,i)) bits|=1u<<i;
    }
    *aligned=bits;
    unlockDriver(); return ok;
}
/* motorsSetRatio() is the only motor-write path in this driver, so forcing zero
 * there covers flight control and forceArm. The stabilizer rewrites every motor
 * at 1 kHz, so all duties read back zero within milliseconds of setting it. */
void motorsSetInhibit(bool inhibit) { inhibited=inhibit; }
bool IRAM_ATTR motorsIsInhibited(void) { return inhibited; }

uint32_t motorsGetPwmFreq(void) { return pwmFreqHz; }

/* Change the shared motor PWM timer frequency (duty resolution unchanged).
 * Refused unless every motor output reads zero. */
bool motorsSetPwmFreq(uint32_t hz)
{
    if (hz < MOTORS_PWM_FREQ_MIN || hz > MOTORS_PWM_FREQ_MAX) return false;
    if (!lockDriver()) return false;
    bool ok=false;
    if (readyLocked()) {
        bool idle=true;
        for (unsigned i=0;i<NBR_OF_MOTORS;++i)
            idle &= ledc_get_duty(LEDC_LOW_SPEED_MODE,channels[i])==0;
        if (idle && ledc_set_freq(LEDC_LOW_SPEED_MODE,LEDC_TIMER_0,hz)==ESP_OK) {
            pwmFreqHz=hz;
            ok=true;
        }
    }
    unlockDriver();
    return ok;
}

void motorsBeep(int id,bool enable,uint16_t frequency,uint16_t ratio)
{ (void)id; (void)enable; (void)frequency; (void)ratio; }
void motorsPlayTone(uint16_t frequency,uint16_t duration_msec)
{ (void)frequency; (void)duration_msec; }
void motorsPlayMelody(uint16_t *notes) { (void)notes; }

LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32,m1_pwm,&motor_ratios[0])
LOG_ADD(LOG_UINT32,m2_pwm,&motor_ratios[1])
LOG_ADD(LOG_UINT32,m3_pwm,&motor_ratios[2])
LOG_ADD(LOG_UINT32,m4_pwm,&motor_ratios[3])
LOG_GROUP_STOP(pwm)
