/*
 * ESP-Drone Firmware
 *
 * accel_tempco.c - Accelerometer zero temperature compensation (pure helpers)
 */
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "accel_tempco.h"
#include "imu_level_cal.h"   /* imuLevelCalCrc32 */

void accTcDefaultConfig(accTcConfig_t *c)
{
  c->k[0] = ACC_TC_KX_DEFAULT;
  c->k[1] = ACC_TC_KY_DEFAULT;
  c->k[2] = ACC_TC_KZ_DEFAULT;
  c->kneeC = ACC_TC_KNEE_DEFAULT;
  c->clampMg = ACC_TC_CLAMP_DEFAULT;
  c->on = ACC_TC_ON_DEFAULT;
}

bool accTcConfigValid(const accTcConfig_t *c)
{
  for (int i = 0; i < 3; i++) {
    if (!isfinite(c->k[i]) || fabsf(c->k[i]) > ACC_TC_K_MAX) {
      return false;
    }
  }
  return isfinite(c->kneeC) && c->kneeC >= ACC_TC_KNEE_MIN && c->kneeC <= ACC_TC_KNEE_MAX &&
         isfinite(c->clampMg) && c->clampMg >= 0.0f && c->clampMg <= 50.0f;
}

void accTcInit(accTcState_t *st)
{
  memset(st, 0, sizeof(*st));
}

void accTcStep(accTcState_t *st, float tempC, float dt)
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
  st->tLp += (tempC - st->tLp) * (dt / (ACC_TC_TAU_S + dt));
}

void accTcLatch(accTcState_t *st)
{
  if (!st->seeded) {
    return;
  }
  st->tRef = st->tLp;
  st->refSet = true;
}

float accTcEffDelta(const accTcConfig_t *c, float tNow, float tRef)
{
  return fminf(tNow, c->kneeC) - fminf(tRef, c->kneeC);
}

void accTcModel(const accTcConfig_t *c, const accTcState_t *st, float out[3])
{
  out[0] = out[1] = out[2] = 0.0f;
  if (!st->refSet || !st->seeded || !isfinite(st->tLp) || !isfinite(st->tRef)) {
    return;
  }
  const float d = accTcEffDelta(c, st->tLp, st->tRef);
  for (int i = 0; i < 3; i++) {
    const float v = c->k[i] * d;
    out[i] = fminf(fmaxf(v, -c->clampMg), c->clampMg);
  }
}

void accTcCorr(const accTcConfig_t *c, const accTcState_t *st, float out[3])
{
  if (!c->on) {
    out[0] = out[1] = out[2] = 0.0f;
    return;
  }
  accTcModel(c, st, out);
}

void accTcBlock(accTcState_t *st, const accTcConfig_t *c, int32_t rawSum, int n, float corrOut[3])
{
  if (n >= 1) {
    accTcStep(st, (float)rawSum / ((float)n * 340.0f) + 36.53f, (float)n * 0.001f);
  }
  accTcCorr(c, st, corrOut);
}

void accTcWindowInit(accTcWindow_t *w)
{
  memset(w, 0, sizeof(*w));
}

void accTcWindowPush(accTcWindow_t *w, const int32_t rawSum[3], int n, float mgPerLsb)
{
  if (n < 1) {
    return;
  }
  const float s = mgPerLsb / (float)n;
  for (int i = 0; i < 3; i++) {
    w->mg[w->next][i] = (float)rawSum[i] * s;
  }
  w->next = (uint8_t)((w->next + 1u) % ACC_TC_WIN_BLOCKS);
  if (w->n < ACC_TC_WIN_BLOCKS) {
    w->n++;
  }
}

bool accTcWindowMean(const accTcWindow_t *w, float out[3])
{
  out[0] = out[1] = out[2] = 0.0f;
  if (w->n < ACC_TC_WIN_BLOCKS) {
    return false;
  }
  for (int b = 0; b < ACC_TC_WIN_BLOCKS; b++) {
    for (int i = 0; i < 3; i++) {
      out[i] += w->mg[b][i];
    }
  }
  for (int i = 0; i < 3; i++) {
    out[i] *= 1.0f / ACC_TC_WIN_BLOCKS;
  }
  return true;
}

static int16_t sat16(float v)
{
  if (!(v == v)) {
    return 0;
  }
  return (int16_t)lrintf(fminf(fmaxf(v, -32767.0f), 32767.0f));
}

void accTcRezeroFill(accTcRezero_t *e, float tRef, float tNow, const float stepMg[3], const float corrMg[3],
                     uint8_t flags)
{
  e->tRefCc = sat16(tRef * 100.0f);
  e->tNowCc = sat16(tNow * 100.0f);
  for (int i = 0; i < 3; i++) {
    e->stepCmg[i] = sat16(stepMg[i] * 100.0f);
    e->corrCmg[i] = sat16(corrMg[i] * 100.0f);
  }
  e->flags = flags;
}

float accTcRezeroResidual(const accTcRezero_t *e, int axis)
{
  return ((int32_t)e->stepCmg[axis] - (int32_t)e->corrCmg[axis]) / 100.0f;
}

void accTcLogInit(accTcLog_t *log)
{
  memset(log, 0, sizeof(*log));
}

bool accTcLogRezero(accTcLog_t *log, const accTcRezero_t *e, uint32_t nowMs, uint32_t airLastMs)
{
  if (e->flags & ACC_TC_RZ_AIR) {
    log->air = *e;
    log->airSeq++;
    log->airAtMs = nowMs;
    log->airLandMs = nowMs - airLastMs;
  }
  if ((e->flags & (ACC_TC_RZ_FLIGHT | ACC_TC_RZ_AIR | ACC_TC_RZ_COOL)) == 0) {
    return false;
  }
  log->ring[log->n % ACC_TC_LOG_RING] = *e;
  log->n++;
  return true;
}

uint32_t accTcLogCount(const accTcLog_t *log)
{
  return log->n < ACC_TC_LOG_RING ? log->n : ACC_TC_LOG_RING;
}

const accTcRezero_t *accTcLogEntry(const accTcLog_t *log, uint32_t i)
{
  const uint32_t n = accTcLogCount(log);
  return &log->ring[(log->n - n + i) % ACC_TC_LOG_RING];
}

void accTcRecordInit(accTcRecord_t *rec, const accTcConfig_t *c)
{
  memset(rec, 0, sizeof(*rec));
  rec->version = ACC_TC_VERSION;
  rec->size = sizeof(accTcRecord_t);
  rec->magic = ACC_TC_MAGIC;
  for (int i = 0; i < 3; i++) {
    rec->k[i] = c->k[i];
  }
  rec->kneeC = c->kneeC;
  rec->on = c->on ? 1u : 0u;
  rec->crc = imuLevelCalCrc32(rec, offsetof(accTcRecord_t, crc));
}

bool accTcRecordValid(const accTcRecord_t *rec, uint32_t storedSize)
{
  if (storedSize != sizeof(accTcRecord_t) || rec->size != sizeof(accTcRecord_t) ||
      rec->version != ACC_TC_VERSION || rec->magic != ACC_TC_MAGIC || rec->on > 1u) {
    return false;
  }
  if (rec->crc != imuLevelCalCrc32(rec, offsetof(accTcRecord_t, crc))) {
    return false;
  }
  accTcConfig_t c;
  accTcDefaultConfig(&c);
  accTcRecordApply(rec, &c);
  return accTcConfigValid(&c);
}

void accTcRecordApply(const accTcRecord_t *rec, accTcConfig_t *c)
{
  for (int i = 0; i < 3; i++) {
    c->k[i] = rec->k[i];
  }
  c->kneeC = rec->kneeC;
  c->on = rec->on != 0u;
  c->clampMg = ACC_TC_CLAMP_DEFAULT;
}
