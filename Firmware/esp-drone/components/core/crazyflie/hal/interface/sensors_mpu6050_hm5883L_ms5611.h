/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2018 Bitcraze AB
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
 */

#ifndef __SENSORS_MPU9250_LPS25H_H__
#define __SENSORS_MPU9250_LPS25H_H__

#include "sensors.h"
#include "gyro_tempco.h"
#include "accel_tempco.h"

void sensorsMpu6050Hmc5883lMs5611Init(void);
bool sensorsMpu6050Hmc5883lMs5611Test(void);
bool sensorsMpu6050Hmc5883lMs5611AreCalibrated(void);
bool sensorsMpu6050Hmc5883lMs5611ManufacturingTest(void);
void sensorsMpu6050Hmc5883lMs5611Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsMpu6050Hmc5883lMs5611WaitDataReady(void);
/* Bench fault injection for the motor watchdog (USB 'mguard test imu', BENCH
 * only, gated by the caller): withhold the next ms IMU samples (~1 per ms)
 * from the stabilizer, as an MPU-6050 that stopped interrupting would. */
void sensorsMpu6050FaultHoldData(uint32_t ms);
bool sensorsMpu6050Hmc5883lMs5611ReadGyro(Axis3f *gyro);
bool sensorsMpu6050Hmc5883lMs5611ReadAcc(Axis3f *acc);
bool sensorsMpu6050Hmc5883lMs5611ReadMag(Axis3f *mag);
bool sensorsMpu6050Hmc5883lMs5611ReadBaro(baro_t *baro);
void sensorsMpu6050Hmc5883lMs5611SetAccMode(accModes accMode);

/*
 * Stationary-measurement and level-calibration support (USB diagnostics).
 * All vectors are in the body frame after the fixed axis mapping.
 */
typedef struct {
  uint32_t count;      /* number of 1 kHz samples */
  float accMean[3];    /* g, raw: before level offset, scale and alignment */
  float accStd[3];     /* g */
  float gyroMean[3];   /* deg/s, after the startup gyro bias is removed */
  float gyroStd[3];    /* deg/s */
  float gyroBias[3];   /* deg/s, the startup bias currently subtracted */
  bool gyroBiasFound;
} sensorsImuStats_t;

/* Start (or restart) accumulating raw samples; stops after 60 s of samples. */
void sensorsMpu6050AccumStart(void);
/* Stop accumulating and return statistics. False if no samples were taken. */
bool sensorsMpu6050AccumStop(sensorsImuStats_t *out);

/* Level offset (g) currently applied; loadResult is the esp_err_t of the boot-time NVS load. */
void sensorsMpu6050GetLevelOffset(float *x, float *y, int *loadResult);
/* Apply a new level offset immediately (not persisted). False if out of bounds. */
bool sensorsMpu6050SetLevelOffset(float x, float y);
float sensorsMpu6050GetAccScale(void);

/* The driver's per-loop state behind the inline getters below (docs/autonomy-a2b.md 19.11: the per-loop readers
 * make no calls, no lock). Written by the driver only; each is one aligned word, read by any task. */
extern volatile int16_t sensorsMpu6050ImuTempRaw;        /* TEMP_OUT of the latest sample (sensors task) */
extern volatile bool sensorsMpu6050InFlightNow;          /* the stabilizer's latest thrust > 30000 (SetThrust) */
extern volatile uint32_t sensorsMpu6050RezeroCountNow;   /* accepted ground re-zeros since boot (sensors task) */
extern float sensorsMpu6050GyroTcCorrNow[3];             /* the gyro compensation subtracted now, dps (sensors task,
                                                          * once per GYRO_TC_BLOCK samples and at a latch) */
extern volatile uint32_t sensorsMpu6050GyroTcAirSeqNow;  /* the airborne re-zeros logged since boot (sensors task) */
extern float sensorsMpu6050AccTcCorrNow[3];              /* the accelerometer compensation subtracted now, g (sensors
                                                          * task, once per block and at a latch; accel_tempco.h) */
extern volatile uint32_t sensorsMpu6050AccTcAirSeqNow;   /* its entries after a flight since boot (sensors task) */

/* MPU6050 die temperature, deg C (last sample). */
static inline float sensorsMpu6050GetTempC(void)
{
  return (float)sensorsMpu6050ImuTempRaw / 340.0f + 36.53f;
}

/* Thrust setpoint from the stabilizer, every loop: above lift-off thrust the
 * in-flight accelerometer offset applies; after 2 s at zero thrust the gyro
 * bias is re-measured whenever the drone is still. */
void sensorsMpu6050SetThrust(float thrust);
/* Gyro re-zeros since boot and the total change from the startup bias (what a
 * still gyro would read without them), deg/s, body frame. */
void sensorsMpu6050GetGyroRezero(uint32_t *count, float drift[3]);
/* Above lift-off thrust (the stabilizer's latest setpoint, > 30000): the
 * in-flight accelerometer offset applies; the attitude filter's in-flight
 * bias learning (sensfusion6.h) uses it. Any task. */
static inline bool sensorsMpu6050InFlight(void)
{
  return sensorsMpu6050InFlightNow;
}
/* Accepted ground re-zeros since boot (changes when the bias in use does). */
static inline uint32_t sensorsMpu6050GyroRezeroCount(void)
{
  return sensorsMpu6050RezeroCountNow;
}

/* Gyro temperature compensation (gyro_tempco.h, docs/autonomy-a2b.md 19.1):
 * the correction (dps, body frame) subtracted from the latest sample, 0 when
 * off, bypassed (bench_bypass.h) or before the startup bias. No lock: the
 * axes are separate words (two may be one block, 50 ms, apart). */
static inline void sensorsMpu6050GetGyroTcCorr(float out[3])
{
  out[0] = sensorsMpu6050GyroTcCorrNow[0];
  out[1] = sensorsMpu6050GyroTcCorrNow[1];
  out[2] = sensorsMpu6050GyroTcCorrNow[2];
}
/* The airborne re-zeros logged since boot: sensorsMpu6050GetGyroTcAir()'s count, without its lock (a reader polls
 * this first and calls that only when it moved). */
static inline uint32_t sensorsMpu6050GyroTcAirSeq(void)
{
  return sensorsMpu6050GyroTcAirSeqNow;
}
/* One logged ground re-zero: its bias change (what a still gyro read with
 * the old bias) against the model's correction at that moment; step - corr
 * is the error the compensation left since the previous latch. Which ones
 * are logged, and the airborne one kept on its own: gyro_tempco.h. */
#define SENSORS_GTC_RZ_FLIGHT GYRO_TC_RZ_FLIGHT   /* thrust since the previous latch (a flight, or a ground spin) */
#define SENSORS_GTC_RZ_ON     GYRO_TC_RZ_ON       /* the compensation was on */
#define SENSORS_GTC_RZ_AIR    GYRO_TC_RZ_AIR      /* thrust above lift-off since the previous latch: a flight */
#define SENSORS_GTC_RZ_HOLD   GYRO_TC_RZ_HOLD     /* a 'gtc hold' since the previous latch */
#define SENSORS_GTC_RZ_COOL   GYRO_TC_RZ_COOL     /* a ground cooling spin since the previous latch */
typedef gyroTcRezero_t sensorsGyroTcRezero_t;
typedef struct {
  gyroTcConfig_t cfg;
  int loadResult;              /* esp_err_t of the boot load, or of the latest 'gtc' change (ESP_OK: nvs) */
  float tempC;                 /* the latest sample */
  float tLp, tRef;             /* the model's temperature (the latest block's, 20 Hz) and reference */
  bool refSet;
  uint32_t latchAgeMs;
  float corr[3];               /* subtracted now */
  float model[3];              /* the model now (also when off) */
  uint32_t rezeros;            /* accepted re-zeros since boot */
  uint32_t logged;             /* re-zeros logged (the ring holds the last n) */
  uint8_t n;
  sensorsGyroTcRezero_t last[GYRO_TC_LOG_RING];   /* oldest first */
  /* The first re-zero after the latest airborne stretch (the post-flight
   * check), kept until the next airborne one: */
  uint32_t airSeq;             /* such re-zeros since boot (0: none; air unset) */
  sensorsGyroTcRezero_t air;
  uint32_t airAgeMs;           /* since it */
  uint32_t airLandMs;          /* from the stretch's last airborne sample to it */
  uint32_t airSpanMs;          /* the stretch's first to last airborne sample */
  bool holdOn;
  uint32_t holdLeftMs;
  bool thrustSinceLatch;
  bool airSinceLatch;          /* above lift-off since the latest latch (the next re-zero is an airborne one) */
  bool bypassed;               /* the bisect toggle BENCH_BYPASS_GTC is on (bench_bypass.h) */
} sensorsGyroTcStatus_t;
void sensorsMpu6050GetGyroTc(sensorsGyroTcStatus_t *st);
/* The latest airborne re-zero (status 'air'), for the flight summary: its
 * count since boot (0: none); when it differs from seenSeq, *e and its dTeff
 * (min(T, knee) - min(Tref, knee) with the config now, C) and landMs are
 * filled. Any task; one short critical section. */
uint32_t sensorsMpu6050GetGyroTcAir(uint32_t seenSeq, sensorsGyroTcRezero_t *e, float *dTeff, uint32_t *landMs);
/* Live config (validated: gyroTcConfigValid); loadResult is what 'gtc show'
 * reports as the source. Not persisted here. */
bool sensorsMpu6050SetGyroTcConfig(const gyroTcConfig_t *c, int loadResult);
/* Bench: no ground re-zero for ms (<= 300000; 0 ends it); any thrust or an
 * active autonomy ends it. RAM only. */
bool sensorsMpu6050GyroTcHold(uint32_t ms);
/* The bisect toggle BENCH_BYPASS_GTC (bench_bypass.h; the USB task, motors
 * stopped): on, the sensors task skips the whole compensation path (the gyro
 * as in 14bcfc6, no latch, no log); off again, it restarts unseeded (0 until
 * the next accepted ground re-zero). RAM only. */
void sensorsMpu6050GyroTcBypass(bool on);

/* Accelerometer temperature compensation (accel_tempco.h, docs/autonomy-a2b.md
 * 36): the correction (g, body frame) subtracted from the latest sample, 0
 * when off, bypassed (the gyro's bisect toggle covers it) or before a latch.
 * No lock: the axes are separate words. */
static inline void sensorsMpu6050GetAccTcCorr(float out[3])
{
  out[0] = sensorsMpu6050AccTcCorrNow[0];
  out[1] = sensorsMpu6050AccTcCorrNow[1];
  out[2] = sensorsMpu6050AccTcCorrNow[2];
}
static inline uint32_t sensorsMpu6050AccTcAirSeq(void)
{
  return sensorsMpu6050AccTcAirSeqNow;
}
/* Take the accelerometer's reference now (the sensors task does it at its
 * next block, <= 50 ms): the compensation restarts from the temperature now
 * and the resting reading becomes the reference of the check. The autonomy
 * glue calls it at ARM, the ground cooling spin at its start (coolFollows:
 * the check's next entry is marked as the spin's), the console at 'atc
 * latch'. Any task. */
void sensorsMpu6050AccTcLatch(bool coolFollows);
typedef accTcRezero_t sensorsAccTcRezero_t;
typedef struct {
  accTcConfig_t cfg;
  int loadResult;              /* esp_err_t of the boot load, or of the latest 'atc' change (ESP_OK: nvs) */
  float tempC;                 /* the latest sample */
  float tLp, tRef;             /* the model's temperature (the latest block's, 20 Hz) and reference */
  bool refSet;
  bool refValid;               /* the reference reading was taken (the drone had rested 1 s at the latch) */
  float refMg[3];              /* it, mg, body frame, raw */
  float stillMg[3];            /* the latest 1 s mean, mg (what the check compares with the reference) */
  bool stillValid;
  uint32_t latchAgeMs;
  float corr[3];               /* subtracted now, mg */
  float model[3];              /* the model now (also when off), mg */
  uint32_t logged;             /* entries logged (the ring holds the last n) */
  uint8_t n;
  sensorsAccTcRezero_t last[ACC_TC_LOG_RING];   /* oldest first */
  uint32_t airSeq;             /* entries after a flight since boot (0: none; air unset) */
  sensorsAccTcRezero_t air;    /* the latest of them (the post-flight check) */
  uint32_t airAgeMs;
  uint32_t airLandMs;          /* from the stretch's last airborne sample to it */
  bool thrustSinceLatch;       /* the gyro re-zero's flags (shared): thrust since the latest gyro latch */
  bool airSinceLatch;
  bool coolSince;              /* a ground cooling spin started since the latest gyro latch */
  bool latchPending;
  bool bypassed;
} sensorsAccTcStatus_t;
void sensorsMpu6050GetAccTc(sensorsAccTcStatus_t *st);
/* The latest post-flight entry, for the flight summary: as sensorsMpu6050GetGyroTcAir(). */
uint32_t sensorsMpu6050GetAccTcAir(uint32_t seenSeq, sensorsAccTcRezero_t *e, float *dTeff, uint32_t *landMs);
/* Live config (validated: accTcConfigValid); loadResult is what 'atc show' reports as the source. */
bool sensorsMpu6050SetAccTcConfig(const accTcConfig_t *c, int loadResult);
bool sensorsMpu6050AccTcOn(void);   /* the live config's on (one bool, no lock; 'status') */

/* In-flight accelerometer offset (g, +-0.05), added to the level offset while
 * the stabilizer reports flight (ramped over 0.5 s). Not persisted here. */
void sensorsMpu6050GetFlightAccOffset(float *x, float *y);
bool sensorsMpu6050SetFlightAccOffset(float x, float y);

#endif // __SENSORS_MPU9250_LPS25H_H__