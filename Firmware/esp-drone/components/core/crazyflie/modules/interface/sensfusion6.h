/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
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
 */

#ifndef SENSORFUSION6_H_
#define SENSORFUSION6_H_
#include <stdbool.h>
#include <stdint.h>

void sensfusion6Init(void);
bool sensfusion6Test(void);

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void sensfusion6GetQuaternion(float* qx, float* qy, float* qz, float* qw);
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);
/* World-up specific force in g: acc . the estimated gravity direction (~1.0
 * at rest, updated with the attitude at 250 Hz). The camera height observer
 * (cam_vz_glue.c) and vert_ctrl's accZ use it. */
float sensfusion6GetAccZ(const float ax, const float ay, const float az);
float sensfusion6GetInvThrustCompensationForTilt();
/* The rate (rad/s, body x/y/z: the gyro's axes) the attitude filter added to
 * the gyro in its latest update: the Mahony proportional + integral feedback
 * (0 after an update without a usable accelerometer; always 0 with the
 * Madgwick variant). It settles at minus the gyro bias (time constant
 * 1/Kp = 2.5 s); the camera height observer low-passes it to remove an
 * in-flight gyro bias from its rotation compensation (cam_vz_shadow.h). A
 * plain global so that cam_vz_glue.c's IRAM step reads it without a flash
 * call; written by the stabilizer task at 250 Hz, read by the same task. */
extern float sensfusion6RateCorr[3];

/* In-flight gyro-bias learning (docs/autonomy-a2b.md 19.2). While airborne
 * (sensfusion6SetAirborne, every update from the driver's thrust > 30000),
 * with | |a|^2 - 1 | <= 0.20 and |gx|, |gy| <= 30 deg/s, the x / y integral
 * runs at twoKiAir (default 0.040: Ki 0.02 /s^2, tau ~17 s) instead of the
 * ground twoKi 0.002; it is clamped at 1.5 deg/s and zeroed when the
 * driver's bias changes (sensfusion6NoteGyroBiasEpoch with the re-zero
 * count). z always learns at twoKi. twoKiAir 0 is the filter of 14bcfc6 bit
 * for bit (no gate, clamp or zeroing). Stabilizer task, except the setter. */
#define SENSFUSION6_KI_AIR_MAX 0.2f
/* The state behind the two inline setters below (sensfusion6.c; the 250 Hz
 * path makes no call for them, docs/autonomy-a2b.md 19.11). */
extern bool sensfusion6AirborneNow;
extern uint32_t sensfusion6BiasEpochNow;   /* UINT32_MAX until the first note */
/* Out of line: the epoch moved (the first note only sets it). */
void sensfusion6GyroBiasEpochMoved(uint32_t epoch);
static inline void sensfusion6SetAirborne(bool on)
{
  sensfusion6AirborneNow = on;
}
static inline void sensfusion6NoteGyroBiasEpoch(uint32_t epoch)
{
  if (epoch != sensfusion6BiasEpochNow) {
    sensfusion6GyroBiasEpochMoved(epoch);
  }
}
bool sensfusion6Airborne(void);
/* 0..SENSFUSION6_KI_AIR_MAX (USB 'mki set'; NVS mkiair); false out of range. */
bool sensfusion6SetKiAir(float twoKiAir);
float sensfusion6GetKiAir(void);
/* The gyro bias the integral has learned, deg/s, body x / y / z (minus the
 * integral feedback). */
void sensfusion6GetLearnedBiasDps(float out[3]);
/* x and y of it, inline (the recorder at 100 Hz): the same expression, so the
 * same values. The integrals are sensfusion6.c's (stabilizer task). */
extern float integralFBx, integralFBy;
static inline void sensfusion6GetLearnedBiasDpsXY(float out[2])
{
  out[0] = -integralFBx * 180.0f / 3.14159265358979323846f;
  out[1] = -integralFBy * 180.0f / 3.14159265358979323846f;
}

#endif /* SENSORFUSION6_H_ */
