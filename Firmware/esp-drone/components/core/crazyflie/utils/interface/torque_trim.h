/*
 * ESP-Drone Firmware
 *
 * torque_trim.h - Feed-forward roll/pitch torque trim record (pure helpers)
 *
 * The trim is a constant roll/pitch actuator offset, in the same units as the
 * rate PID outputs that feed the motor mixer (control_t roll/pitch). It
 * supplies, from the moment of lift-off, the steady correction the PID
 * integrators otherwise have to build up during every take-off, e.g. for an
 * off-centre centre of gravity. It is separate from the accelerometer level
 * calibration: it changes motor commands, not the attitude reference.
 *
 * No ESP-IDF dependencies; unit tested by tools/tests/test_imu_level_cal.c.
 */
#ifndef TORQUE_TRIM_H_
#define TORQUE_TRIM_H_

#include <stdbool.h>
#include <stdint.h>

#define TORQUE_TRIM_VERSION 1
#define TORQUE_TRIM_MAGIC 0x4D495254u  /* "TRIM": distinguishes it from other records */
/* Hover needs up to ~7500 on this ESP-FLY; beyond 10000 something is wrong. */
#define TORQUE_TRIM_MAX 10000.0f

typedef struct {
  uint16_t version;  /* TORQUE_TRIM_VERSION */
  uint16_t size;     /* sizeof(torqueTrimRecord_t) */
  uint32_t magic;    /* TORQUE_TRIM_MAGIC */
  float roll;       /* actuator units, + raises the left motors (M3, M4) */
  float pitch;       /* actuator units, + raises the front motors (M1, M4) */
  uint32_t crc;      /* crc32 over all preceding bytes */
} torqueTrimRecord_t;

bool torqueTrimValuesValid(float roll, float pitch);
void torqueTrimRecordInit(torqueTrimRecord_t *rec, float roll, float pitch);
bool torqueTrimRecordValid(const torqueTrimRecord_t *rec, uint32_t storedSize);

/*
 * Attitude trim: a small roll/pitch offset (degrees) added to the attitude
 * setpoint, i.e. a change of the level the controller holds. Used for a bias
 * the IMU cannot see (e.g. measured by the camera); kept separate from the
 * accelerometer level calibration, which describes the sensor.
 * +roll = right side down, +pitch = nose up (controller convention).
 */
#define ATT_TRIM_VERSION 1
#define ATT_TRIM_MAGIC 0x4D525441u     /* "ATRM" */
#define ATT_TRIM_MAX_DEG 3.0f

typedef struct {
  uint16_t version;  /* ATT_TRIM_VERSION */
  uint16_t size;     /* sizeof(attTrimRecord_t) */
  uint32_t magic;    /* ATT_TRIM_MAGIC */
  float rollDeg;
  float pitchDeg;
  uint32_t crc;      /* crc32 over all preceding bytes */
} attTrimRecord_t;

bool attTrimValuesValid(float rollDeg, float pitchDeg);
void attTrimRecordInit(attTrimRecord_t *rec, float rollDeg, float pitchDeg);
bool attTrimRecordValid(const attTrimRecord_t *rec, uint32_t storedSize);

/* Fraction of the trim applied at a given thrust: 0 at or below groundThrust,
 * rising linearly to 1 at groundThrust + rampThrust. */
float torqueTrimScale(float thrust, float groundThrust, float rampThrust);

#endif /* TORQUE_TRIM_H_ */
