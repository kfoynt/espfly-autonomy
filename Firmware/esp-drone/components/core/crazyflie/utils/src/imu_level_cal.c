/*
 * ESP-Drone Firmware
 *
 * imu_level_cal.c - Accelerometer level-reference calibration (pure helpers)
 */
#include <math.h>
#include <stddef.h>

#include "imu_level_cal.h"

uint32_t imuLevelCalCrc32(const void *data, uint32_t len)
{
  const uint8_t *p = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFu;

  for (uint32_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
  }

  return ~crc;
}

void imuLevelCalRecordInit(imuLevelCalRecord_t *rec, float offsetX, float offsetY)
{
  rec->version = IMU_LEVEL_CAL_VERSION;
  rec->size = sizeof(imuLevelCalRecord_t);
  rec->accOffsetX = offsetX;
  rec->accOffsetY = offsetY;
  rec->crc = imuLevelCalCrc32(rec, offsetof(imuLevelCalRecord_t, crc));
}

bool imuLevelCalOffsetsValid(float offsetX, float offsetY)
{
  return isfinite(offsetX) && isfinite(offsetY) &&
         fabsf(offsetX) <= IMU_LEVEL_CAL_MAX_OFFSET_G &&
         fabsf(offsetY) <= IMU_LEVEL_CAL_MAX_OFFSET_G;
}

bool imuLevelCalRecordValid(const imuLevelCalRecord_t *rec, uint32_t storedSize)
{
  if (storedSize != sizeof(imuLevelCalRecord_t) ||
      rec->size != sizeof(imuLevelCalRecord_t) ||
      rec->version != IMU_LEVEL_CAL_VERSION) {
    return false;
  }

  if (rec->crc != imuLevelCalCrc32(rec, offsetof(imuLevelCalRecord_t, crc))) {
    return false;
  }

  return imuLevelCalOffsetsValid(rec->accOffsetX, rec->accOffsetY);
}

void imuAlignmentInit(imuAlignment_t *align, float rollDeg, float pitchDeg)
{
  align->cosRoll = cosf(rollDeg * (float)M_PI / 180.0f);
  align->sinRoll = sinf(rollDeg * (float)M_PI / 180.0f);
  align->cosPitch = cosf(pitchDeg * (float)M_PI / 180.0f);
  align->sinPitch = sinf(pitchDeg * (float)M_PI / 180.0f);
}

void imuAlignmentApply(const imuAlignment_t *align, const float in[3], float out[3])
{
  float rx[3];

  // Rotate around x-axis
  rx[0] = in[0];
  rx[1] = in[1] * align->cosRoll - in[2] * align->sinRoll;
  rx[2] = in[1] * align->sinRoll + in[2] * align->cosRoll;

  // Rotate around y-axis. The original code used "-x*sin + z*cos" for z,
  // which together with "x*cos - z*sin" for x is not a rotation (det = cos 2a)
  // and changes vector length for nonzero pitch. The x row keeps the
  // documented Kconfig sign convention; z is its orthogonal complement.
  out[0] = rx[0] * align->cosPitch - rx[2] * align->sinPitch;
  out[1] = rx[1];
  out[2] = rx[0] * align->sinPitch + rx[2] * align->cosPitch;
}
