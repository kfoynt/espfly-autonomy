/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * controller_pid.h - PID Controller Interface
 */
#ifndef __CONTROLLER_PID_H__
#define __CONTROLLER_PID_H__

#include "stabilizer_types.h"

void controllerPidInit(void);
bool controllerPidTest(void);
void controllerPid(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

/* Feed-forward roll/pitch torque trim (see torque_trim.h). Set applies
 * immediately and is not persisted; loadResult is the boot-time NVS load. */
void controllerPidGetTorqueTrim(float *roll, float *pitch, int *loadResult);
bool controllerPidSetTorqueTrim(float roll, float pitch);

/* Yaw lock (heading hold, yaw stick ignored). Set applies immediately and is
 * not persisted; the boot value comes from NVS. */
void controllerPidGetAttitudeTrim(float *rollDeg, float *pitchDeg, int *loadResult);
bool controllerPidSetAttitudeTrim(float rollDeg, float pitchDeg);

/* Camera drift-hold correction currently added to the attitude setpoint (deg). */
void controllerPidGetCameraCorrection(float *rollDeg, float *pitchDeg);

/* The roll/pitch the attitude PID holds now: setpoint + attitude trim +
 * camera correction (deg), as of the latest 500 Hz attitude update.
 * Stabilizer task only (autonomy's attErrDeg, docs/autonomy-a2b.md). */
void controllerPidGetAttitudeDesired(float *rollDeg, float *pitchDeg);

void controllerPidSetYawLock(bool on);
bool controllerPidGetYawLock(void);

#endif //__CONTROLLER_PID_H__
