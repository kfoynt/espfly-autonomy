/*
 * ESP-Drone Firmware
 *
 * torque_trim.c - Feed-forward roll/pitch torque trim record (pure helpers)
 */
#include <math.h>
#include <stddef.h>

#include "imu_level_cal.h"
#include "torque_trim.h"

bool torqueTrimValuesValid(float roll, float pitch)
{
  return isfinite(roll) && isfinite(pitch) &&
         fabsf(roll) <= TORQUE_TRIM_MAX && fabsf(pitch) <= TORQUE_TRIM_MAX;
}

void torqueTrimRecordInit(torqueTrimRecord_t *rec, float roll, float pitch)
{
  rec->version = TORQUE_TRIM_VERSION;
  rec->size = sizeof(torqueTrimRecord_t);
  rec->magic = TORQUE_TRIM_MAGIC;
  rec->roll = roll;
  rec->pitch = pitch;
  rec->crc = imuLevelCalCrc32(rec, offsetof(torqueTrimRecord_t, crc));
}

bool torqueTrimRecordValid(const torqueTrimRecord_t *rec, uint32_t storedSize)
{
  if (storedSize != sizeof(torqueTrimRecord_t) ||
      rec->size != sizeof(torqueTrimRecord_t) ||
      rec->version != TORQUE_TRIM_VERSION || rec->magic != TORQUE_TRIM_MAGIC) {
    return false;
  }
  if (rec->crc != imuLevelCalCrc32(rec, offsetof(torqueTrimRecord_t, crc))) {
    return false;
  }
  return torqueTrimValuesValid(rec->roll, rec->pitch);
}

bool attTrimValuesValid(float rollDeg, float pitchDeg)
{
  return isfinite(rollDeg) && isfinite(pitchDeg) &&
         fabsf(rollDeg) <= ATT_TRIM_MAX_DEG && fabsf(pitchDeg) <= ATT_TRIM_MAX_DEG;
}

void attTrimRecordInit(attTrimRecord_t *rec, float rollDeg, float pitchDeg)
{
  rec->version = ATT_TRIM_VERSION;
  rec->size = sizeof(attTrimRecord_t);
  rec->magic = ATT_TRIM_MAGIC;
  rec->rollDeg = rollDeg;
  rec->pitchDeg = pitchDeg;
  rec->crc = imuLevelCalCrc32(rec, offsetof(attTrimRecord_t, crc));
}

bool attTrimRecordValid(const attTrimRecord_t *rec, uint32_t storedSize)
{
  if (storedSize != sizeof(attTrimRecord_t) || rec->size != sizeof(attTrimRecord_t) ||
      rec->version != ATT_TRIM_VERSION || rec->magic != ATT_TRIM_MAGIC) {
    return false;
  }
  if (rec->crc != imuLevelCalCrc32(rec, offsetof(attTrimRecord_t, crc))) {
    return false;
  }
  return attTrimValuesValid(rec->rollDeg, rec->pitchDeg);
}

float torqueTrimScale(float thrust, float groundThrust, float rampThrust)
{
  if (!(rampThrust > 0.0f)) {
    return thrust > groundThrust ? 1.0f : 0.0f;
  }
  float scale = (thrust - groundThrust) / rampThrust;
  return fminf(fmaxf(scale, 0.0f), 1.0f);
}
