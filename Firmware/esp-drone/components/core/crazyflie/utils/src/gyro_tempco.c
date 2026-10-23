/*
 * ESP-Drone Firmware
 *
 * gyro_tempco.c - Gyro zero-rate temperature compensation (pure helpers)
 */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "gyro_tempco.h"
#include "imu_level_cal.h"   /* imuLevelCalCrc32 */

void gyroTcDefaultConfig(gyroTcConfig_t *c)
{
  c->k[0] = GYRO_TC_KX_DEFAULT;
  c->k[1] = GYRO_TC_KY_DEFAULT;
  c->k[2] = GYRO_TC_KZ_DEFAULT;
  c->kneeC = GYRO_TC_KNEE_DEFAULT;
  c->clampDps = GYRO_TC_CLAMP_DEFAULT;
  c->on = true;
}

bool gyroTcConfigValid(const gyroTcConfig_t *c)
{
  for (int i = 0; i < 3; i++) {
    if (!isfinite(c->k[i]) || fabsf(c->k[i]) > GYRO_TC_K_MAX) {
      return false;
    }
  }
  return isfinite(c->kneeC) && c->kneeC >= GYRO_TC_KNEE_MIN && c->kneeC <= GYRO_TC_KNEE_MAX &&
         isfinite(c->clampDps) && c->clampDps >= 0.0f && c->clampDps <= 2.0f;
}

void gyroTcInit(gyroTcState_t *st)
{
  memset(st, 0, sizeof(*st));
}

void gyroTcStep(gyroTcState_t *st, float tempC, float dt)
{
  if (!isfinite(tempC)) {
    return;
  }
  if (!st->seeded) {
    st->tLp = tempC;
    st->seeded = true;
    return;
  }
  if (!(dt > 0.0f)) {
    return;
  }
  st->tLp += (tempC - st->tLp) * (dt / (GYRO_TC_TAU_S + dt));
}

void gyroTcLatch(gyroTcState_t *st)
{
  if (!st->seeded) {
    return;
  }
  st->tRef = st->tLp;
  st->refSet = true;
}

float gyroTcEffDelta(const gyroTcConfig_t *c, float tNow, float tRef)
{
  return fminf(tNow, c->kneeC) - fminf(tRef, c->kneeC);
}

void gyroTcModel(const gyroTcConfig_t *c, const gyroTcState_t *st, float out[3])
{
  out[0] = out[1] = out[2] = 0.0f;
  if (!st->refSet || !st->seeded || !isfinite(st->tLp) || !isfinite(st->tRef)) {
    return;
  }
  const float d = gyroTcEffDelta(c, st->tLp, st->tRef);
  for (int i = 0; i < 3; i++) {
    const float v = c->k[i] * d;
    out[i] = fminf(fmaxf(v, -c->clampDps), c->clampDps);
  }
}

void gyroTcCorr(const gyroTcConfig_t *c, const gyroTcState_t *st, float out[3])
{
  if (!c->on) {
    out[0] = out[1] = out[2] = 0.0f;
    return;
  }
  gyroTcModel(c, st, out);
}

float gyroTcTempFromRawSum(int32_t rawSum, int n)
{
  /* |rawSum| <= GYRO_TC_BLOCK x 32768 is exact in a float; n x 340 too */
  return (float)rawSum / ((float)n * 340.0f) + 36.53f;
}

void gyroTcBlock(gyroTcState_t *st, const gyroTcConfig_t *c, int32_t rawSum, int n, float corrOut[3])
{
  if (n >= 1) {
    gyroTcStep(st, gyroTcTempFromRawSum(rawSum, n), (float)n * 0.001f);
  }
  gyroTcCorr(c, st, corrOut);
}

static int16_t sat16(float v)
{
  if (!(v == v)) {
    return 0;
  }
  return (int16_t)lrintf(fminf(fmaxf(v, -32767.0f), 32767.0f));
}

void gyroTcRezeroFill(gyroTcRezero_t *e, float tRef, float tNow, const float stepDps[3], const float corrDps[3],
                      uint8_t flags)
{
  e->tRefOldCc = sat16(tRef * 100.0f);
  e->tNowCc = sat16(tNow * 100.0f);
  for (int i = 0; i < 3; i++) {
    e->stepMdps[i] = sat16(stepDps[i] * 1000.0f);
    e->corrMdps[i] = sat16(corrDps[i] * 1000.0f);
  }
  e->flags = flags;
}

float gyroTcRezeroResidual(const gyroTcRezero_t *e, int axis)
{
  return ((int32_t)e->stepMdps[axis] - (int32_t)e->corrMdps[axis]) / 1000.0f;
}

void gyroTcLogInit(gyroTcLog_t *log)
{
  memset(log, 0, sizeof(*log));
}

bool gyroTcLogRezero(gyroTcLog_t *log, const gyroTcRezero_t *e, uint32_t nowMs, uint32_t airFirstMs,
                     uint32_t airLastMs)
{
  if (e->flags & GYRO_TC_RZ_AIR) {
    log->air = *e;
    log->airSeq++;
    log->airAtMs = nowMs;
    log->airLandMs = nowMs - airLastMs;
    log->airSpanMs = airLastMs - airFirstMs;
  }
  bool news = (e->flags & (GYRO_TC_RZ_FLIGHT | GYRO_TC_RZ_AIR | GYRO_TC_RZ_HOLD)) != 0;
  for (int i = 0; i < 3; i++) {
    news = news || abs(e->stepMdps[i]) >= GYRO_TC_LOG_STEP_MDPS;
  }
  if (!news) {
    return false;
  }
  log->ring[log->n % GYRO_TC_LOG_RING] = *e;
  log->n++;
  return true;
}

uint32_t gyroTcLogCount(const gyroTcLog_t *log)
{
  return log->n < GYRO_TC_LOG_RING ? log->n : GYRO_TC_LOG_RING;
}

const gyroTcRezero_t *gyroTcLogEntry(const gyroTcLog_t *log, uint32_t i)
{
  const uint32_t n = gyroTcLogCount(log);
  return &log->ring[(log->n - n + i) % GYRO_TC_LOG_RING];
}

void gyroTcRecordInit(gyroTcRecord_t *rec, const gyroTcConfig_t *c)
{
  memset(rec, 0, sizeof(*rec));
  rec->version = GYRO_TC_VERSION;
  rec->size = sizeof(gyroTcRecord_t);
  rec->magic = GYRO_TC_MAGIC;
  for (int i = 0; i < 3; i++) {
    rec->k[i] = c->k[i];
  }
  rec->kneeC = c->kneeC;
  rec->on = c->on ? 1u : 0u;
  rec->crc = imuLevelCalCrc32(rec, offsetof(gyroTcRecord_t, crc));
}

bool gyroTcRecordValid(const gyroTcRecord_t *rec, uint32_t storedSize)
{
  if (storedSize != sizeof(gyroTcRecord_t) || rec->size != sizeof(gyroTcRecord_t) ||
      rec->version != GYRO_TC_VERSION || rec->magic != GYRO_TC_MAGIC || rec->on > 1u) {
    return false;
  }
  if (rec->crc != imuLevelCalCrc32(rec, offsetof(gyroTcRecord_t, crc))) {
    return false;
  }
  gyroTcConfig_t c;
  gyroTcDefaultConfig(&c);
  gyroTcRecordApply(rec, &c);
  return gyroTcConfigValid(&c);
}

void gyroTcRecordApply(const gyroTcRecord_t *rec, gyroTcConfig_t *c)
{
  for (int i = 0; i < 3; i++) {
    c->k[i] = rec->k[i];
  }
  c->kneeC = rec->kneeC;
  c->on = rec->on != 0u;
  c->clampDps = GYRO_TC_CLAMP_DEFAULT;
}
