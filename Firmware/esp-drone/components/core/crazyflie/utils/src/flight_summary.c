/*
 * ESP-Drone Firmware
 *
 * flight_summary.c - Per-flight statistics small enough to keep in RTC memory
 */
#include <stdlib.h>
#include <string.h>

#include "flight_summary.h"

static uint32_t addSat(uint32_t a, uint32_t b)
{
  return (a > UINT32_MAX - b) ? UINT32_MAX : a + b;
}

static uint16_t inc16(uint16_t v)
{
  return v == UINT16_MAX ? v : (uint16_t)(v + 1);
}

static int absi(int v)
{
  return v < 0 ? -v : v;
}

static int clampi(int v, int lo, int hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

void fsumStart(flightSummary_t *fs, uint16_t boot, uint8_t flightId, uint16_t vbatMv)
{
  memset(fs, 0, sizeof(*fs));
  fs->boot = boot;
  fs->flightId = flightId;
  fs->end = FSUM_END_OPEN;
  fs->vbatStartMv = vbatMv;
  fs->vbatMinMv = vbatMv;
  fs->imuT0Hc = FSUM_TEMP_NONE;   /* the caller's fields, until it sets them */
  fs->imuT1Hc = FSUM_TEMP_NONE;
  fs->camAwakeDs = FSUM_AGE_NONE;
}

void fsumAdd(flightSummary_t *fs, const fsumSample_t *s)
{
  if (s->thrust == 0) {
    return;
  }
  fs->thrustCs = inc16(fs->thrustCs);
  if (s->thrust > fs->thrustMax) {
    fs->thrustMax = s->thrust;
  }
  if (s->vbatMv > 0 && (fs->vbatMinMv == 0 || s->vbatMv < fs->vbatMinMv)) {
    fs->vbatMinMv = s->vbatMv;
  }
  if (s->holdEngaged) {
    fs->holdCs = inc16(fs->holdCs);
    if (s->holdCoasting) {
      fs->coastCs = inc16(fs->coastCs);
    }
    fs->holdRollSum += s->holdRoll;
    fs->holdPitchSum += s->holdPitch;
  }
  if (s->thrust <= FSUM_AIRBORNE_THRUST) {
    return;
  }

  fs->airCs = inc16(fs->airCs);
  if (s->flowOk) {
    fs->camCs = inc16(fs->camCs);
  }
  for (int i = 0; i < 4; i++) {
    fs->motorSum[i] = addSat(fs->motorSum[i], s->motor[i] >> 8);
  }
  fs->rollRateISum += s->rollRateI;
  fs->pitchRateISum += s->pitchRateI;
  if (fs->airCs <= FSUM_SETTLE_SAMPLES) {
    return;
  }

  fs->steadyCs = inc16(fs->steadyCs);
  const int rDd = s->roll / 10, pDd = s->pitch / 10;          /* 0.1 deg */
  const int gxD = s->gx / 10, gyD = s->gy / 10;               /* deg/s */
  fs->tiltSq = addSat(fs->tiltSq, (uint32_t)(rDd * rDd + pDd * pDd));
  fs->gyroSq = addSat(fs->gyroSq, (uint32_t)(gxD * gxD + gyD * gyD));
  int tilt = absi(rDd) > absi(pDd) ? absi(rDd) : absi(pDd);
  int gyro = absi(gxD) > absi(gyD) ? absi(gxD) : absi(gyD);
  if (tilt > fs->tiltMaxDd) {
    fs->tiltMaxDd = (uint16_t)clampi(tilt, 0, UINT16_MAX);
  }
  if (gyro > fs->gyroMaxDps) {
    fs->gyroMaxDps = (uint16_t)clampi(gyro, 0, UINT16_MAX);
  }
}

uint8_t fsumEndCause(const fsumSample_t *s)
{
  return (s->thrustLocked || s->linkAgeMs > FSUM_LINK_LOST_MS) ? FSUM_END_LINK : FSUM_END_ZERO;
}

void fsumCrumb(fsumCrumb_t *c, const fsumSample_t *s)
{
  c->thrust = s->thrust;
  c->vbatMv = s->vbatMv;
  for (int i = 0; i < 4; i++) {
    c->motor[i] = (uint8_t)(s->motor[i] >> 8);
  }
  c->roll = (int8_t)clampi(s->roll / 50, -127, 127);
  c->pitch = (int8_t)clampi(s->pitch / 50, -127, 127);
  int gyro = absi(s->gx) > absi(s->gy) ? absi(s->gx) : absi(s->gy);
  c->gyroDps = (uint8_t)clampi(gyro / 10, 0, 255);
  c->flags = (s->holdEngaged ? FSUM_CRUMB_HOLD : 0) | (s->flowOk ? FSUM_CRUMB_FLOW : 0) |
             (s->thrustLocked ? FSUM_CRUMB_LOCKED : 0) |
             (s->thrust > FSUM_AIRBORNE_THRUST ? FSUM_CRUMB_AIR : 0);
  c->linkAge20ms = (uint8_t)clampi(s->linkAgeMs / 20, 0, 255);
  c->rail10mv = 0;
  c->txPk = 0;
  c->rxPk = 0;
}

uint8_t fsumRailCode(uint16_t mv)
{
  return mv ? (uint8_t)clampi(((int)mv - 2000) / 10, 1, 255) : 0;
}

static int16_t sat16i(int32_t v)
{
  return (int16_t)clampi(v, -32767, 32767);
}

void fsumSetGtcResidual(flightSummary_t *fs, int32_t rxMdps, int32_t ryMdps, int32_t dTeffCc, uint32_t landMs,
                        bool on)
{
  fs->gtcResMdps[0] = sat16i(rxMdps);
  fs->gtcResMdps[1] = sat16i(ryMdps);
  fs->gtcResDtCc = sat16i(dTeffCc);
  fs->gtcResLandDs = (uint8_t)(landMs / 100u >= 255u ? 255u : landMs / 100u);
  fs->gtcResFlags = FSUM_GTCR_SET | (on ? FSUM_GTCR_ON : 0);
}

void fsumSetAtcResidual(flightSummary_t *fs, int32_t stepZCmg, int32_t corrZCmg, int32_t dTeffCc, bool on)
{
  fs->atcStepZCmg = sat16i(stepZCmg);
  fs->atcCorrZCmg = sat16i(corrZCmg);
  fs->atcResDtCc = sat16i(dTeffCc);
  fs->atcResFlags = FSUM_GTCR_SET | (on ? FSUM_GTCR_ON : 0);
}
