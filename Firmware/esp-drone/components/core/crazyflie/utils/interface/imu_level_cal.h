/*
 * ESP-Drone Firmware
 *
 * imu_level_cal.h - Accelerometer level-reference calibration (pure helpers)
 *
 * This module has no ESP-IDF or FreeRTOS dependencies so it can be unit
 * tested on a host computer (see tools/tests/test_imu_level_cal.c).
 *
 * Frame and units contract
 * ------------------------
 * All vectors handled here are in the drone body frame produced by
 * processAccGyroMeasurements() AFTER the fixed MPU6050 axis mapping
 * (x = -nativeY, y = +nativeX, z = +nativeZ) and are expressed in g.
 *
 * Accelerometer correction order in the sensor task:
 *   1. raw counts -> mapped body axes, in g           (existing)
 *   2. subtract level offset (x, y only)              (this module)
 *   3. divide by the startup magnitude scale          (existing processAccScale)
 *   4. Kconfig ROLL_CALIB / PITCH_CALIB rotation      (existing, default 0)
 *   5. 2nd-order low-pass filter                      (existing)
 *
 * The level offset is the mean x/y acceleration the sensor reports while the
 * drone rests level. It combines the MPU6050 zero-g offset and any small tilt
 * of the IMU board relative to the airframe; for the small angles involved
 * (< 9 deg) subtracting it is equivalent to a rotation to first order.
 */
#ifndef IMU_LEVEL_CAL_H_
#define IMU_LEVEL_CAL_H_

#include <stdbool.h>
#include <stdint.h>

#define IMU_LEVEL_CAL_VERSION       1
/* 0.15 g corresponds to ~8.6 deg. Larger values indicate a bad measurement. */
#define IMU_LEVEL_CAL_MAX_OFFSET_G  0.15f

typedef struct {
  uint16_t version;     /* IMU_LEVEL_CAL_VERSION */
  uint16_t size;        /* sizeof(imuLevelCalRecord_t) */
  float accOffsetX;     /* g, body frame after axis mapping */
  float accOffsetY;     /* g, body frame after axis mapping */
  uint32_t crc;         /* crc32 over all preceding bytes */
} imuLevelCalRecord_t;

uint32_t imuLevelCalCrc32(const void *data, uint32_t len);

/* Fill version/size/crc for the given offsets. */
void imuLevelCalRecordInit(imuLevelCalRecord_t *rec, float offsetX, float offsetY);

/* True if both offsets are finite and within IMU_LEVEL_CAL_MAX_OFFSET_G. */
bool imuLevelCalOffsetsValid(float offsetX, float offsetY);

/* Validate a record read from storage: size, version, crc and offset bounds. */
bool imuLevelCalRecordValid(const imuLevelCalRecord_t *rec, uint32_t storedSize);

/*
 * Rotation used for the Kconfig ROLL_CALIB / PITCH_CALIB alignment.
 * Rotates about x by rollDeg, then about y by pitchDeg, with the sign
 * convention documented in Kconfig: if the drone reads +N deg roll (pitch)
 * while level, setting rollDeg (pitchDeg) = +N removes the error.
 * The result is a proper rotation (orthonormal, determinant +1).
 */
typedef struct {
  float cosRoll, sinRoll, cosPitch, sinPitch;
} imuAlignment_t;

void imuAlignmentInit(imuAlignment_t *align, float rollDeg, float pitchDeg);
void imuAlignmentApply(const imuAlignment_t *align, const float in[3], float out[3]);

#endif /* IMU_LEVEL_CAL_H_ */
