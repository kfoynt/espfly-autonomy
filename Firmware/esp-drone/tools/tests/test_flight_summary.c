/*
 * Host unit tests for components/core/crazyflie/utils/src/flight_summary.c
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_flight_summary.c components/core/crazyflie/utils/src/flight_summary.c \
 *      -o /tmp/test_flight_summary && /tmp/test_flight_summary
 */
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "flight_summary.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static fsumSample_t hover(void)
{
  fsumSample_t s;
  memset(&s, 0, sizeof(s));
  s.thrust = 45000;
  s.vbatMv = 3500;
  s.roll = 100;           /* 1.00 deg */
  s.pitch = -200;         /* -2.00 deg */
  s.gx = 50;              /* 5.0 deg/s */
  s.gy = -30;
  s.motor[0] = 40000;
  s.motor[1] = 42000;
  s.motor[2] = 44000;
  s.motor[3] = 46000;
  s.rollRateI = 10;
  s.pitchRateI = -20;
  return s;
}

static void testStartAndGround(void)
{
  flightSummary_t fs;
  fsumStart(&fs, 7, 3, 3900);
  CHECK(fs.boot == 7 && fs.flightId == 3 && fs.end == FSUM_END_OPEN, "start fields");
  CHECK(fs.vbatStartMv == 3900 && fs.vbatMinMv == 3900, "start voltage");

  fsumSample_t s = hover();
  s.thrust = 0;
  fsumAdd(&fs, &s);
  CHECK(fs.thrustCs == 0, "zero thrust ignored, got %u", fs.thrustCs);

  s.thrust = 20000;     /* spinning on the ground */
  s.vbatMv = 3700;
  fsumAdd(&fs, &s);
  CHECK(fs.thrustCs == 1 && fs.airCs == 0, "ground sample counted only as thrust");
  CHECK(fs.vbatMinMv == 3700 && fs.thrustMax == 20000, "min voltage and max thrust");
  CHECK(fs.motorSum[0] == 0 && fs.steadyCs == 0, "no airborne sums on the ground");
}

static void testSettleAndSums(void)
{
  flightSummary_t fs;
  fsumStart(&fs, 1, 1, 3800);
  fsumSample_t s = hover();
  for (int i = 0; i < FSUM_SETTLE_SAMPLES; i++) {
    fsumAdd(&fs, &s);
  }
  CHECK(fs.airCs == FSUM_SETTLE_SAMPLES && fs.steadyCs == 0, "settle samples excluded, steady=%u", fs.steadyCs);
  CHECK(fs.motorSum[0] == (uint32_t)FSUM_SETTLE_SAMPLES * (40000 >> 8), "motor sum during settle");

  for (int i = 0; i < 50; i++) {
    fsumAdd(&fs, &s);
  }
  CHECK(fs.steadyCs == 50, "steady count %u", fs.steadyCs);
  /* roll 10 dd, pitch -20 dd -> 100 + 400 per sample */
  CHECK(fs.tiltSq == 50u * 500u, "tilt sum %u", (unsigned)fs.tiltSq);
  /* gx 5, gy -3 dps -> 25 + 9 */
  CHECK(fs.gyroSq == 50u * 34u, "gyro sum %u", (unsigned)fs.gyroSq);
  CHECK(fs.tiltMaxDd == 20 && fs.gyroMaxDps == 5, "maxima %u %u", fs.tiltMaxDd, fs.gyroMaxDps);
  CHECK(fs.rollRateISum == 150 * 10 && fs.pitchRateISum == 150 * -20, "integrator sums");
  CHECK(fs.camCs == 0 && fs.holdCs == 0, "no camera");
}

static void testHoldAndCamera(void)
{
  flightSummary_t fs;
  fsumStart(&fs, 1, 1, 3800);
  fsumSample_t s = hover();
  s.flowOk = true;
  s.holdEngaged = true;
  s.holdRoll = -30;
  s.holdPitch = 80;
  for (int i = 0; i < 10; i++) {
    s.holdCoasting = i >= 6;       /* view lost for the last 4 samples */
    fsumAdd(&fs, &s);
  }
  CHECK(fs.camCs == 10 && fs.holdCs == 10, "camera and hold counts");
  CHECK(fs.coastCs == 4, "coasting counted within hold (%u)", fs.coastCs);
  CHECK(fs.holdRollSum == -300 && fs.holdPitchSum == 800, "hold sums %d %d", (int)fs.holdRollSum,
        (int)fs.holdPitchSum);
}

static void testSaturation(void)
{
  flightSummary_t fs;
  fsumStart(&fs, 1, 1, 3800);
  fs.airCs = FSUM_SETTLE_SAMPLES + 1;
  fs.tiltSq = UINT32_MAX - 10;
  fs.thrustCs = UINT16_MAX;
  fsumSample_t s = hover();
  s.roll = 9000;   /* 90 deg */
  fsumAdd(&fs, &s);
  CHECK(fs.tiltSq == UINT32_MAX, "tilt sum saturates");
  CHECK(fs.thrustCs == UINT16_MAX, "counter saturates");
}

static void testEndCause(void)
{
  fsumSample_t s = hover();
  s.thrust = 0;
  s.linkAgeMs = 20;
  CHECK(fsumEndCause(&s) == FSUM_END_ZERO, "stick release");
  s.linkAgeMs = 900;
  CHECK(fsumEndCause(&s) == FSUM_END_LINK, "silent link");
  s.linkAgeMs = 10;
  s.thrustLocked = true;
  CHECK(fsumEndCause(&s) == FSUM_END_LINK, "thrust lock");
}

/* A2b: 'pad' became autoReason at the same offset (old summaries read 0 =
 * AUTO_RS_NONE) and FSUM_END_AUTO = 4 is set by flight_rtc_log.c, never by
 * fsumEndCause(). */
static void testAutoEnd(void)
{
  CHECK(FSUM_END_AUTO == 4 && FSUM_END_RESET == 3, "end codes");
  CHECK(offsetof(flightSummary_t, resetReason) == 4 && offsetof(flightSummary_t, autoReason) == 5 &&
        offsetof(flightSummary_t, coastCs) == 6, "autoReason in pad's place (%u)",
        (unsigned)offsetof(flightSummary_t, autoReason));
  /* 132 until 2026-10-14; +4 for imuT0Hc, imuT1Hc and camAwakeDs (docs/autonomy-a2b.md 17.1), with
   * FLIGHT_RTC_MAGIC bumped to 0xF1A6C0E5 (flight_rtc_log.c); +4 for gtcYEndMdps and mkiEndMdps (19.3; 0xF1A6C0E6);
   * +8 for the post-flight re-zero gtcRes* (19.1; 0xF1A6C0E7); +8 for camVzLate, the observer's late offset noise
   * (6 bytes and the tail's alignment, 25; 0xF1A6C0E8); +8 for camVzLate's T2 fields, the offset drift rate (29;
   * 0xF1A6C0E9); +8 for the accelerometer compensation's check atc* (36.2; 0xF1A6C0EA) */
  CHECK(sizeof(flightSummary_t) == 172, "summary size (%u): a change needs a FLIGHT_RTC_MAGIC bump",
        (unsigned)sizeof(flightSummary_t));
  CHECK(offsetof(fsumCvLate_t, t2K) == 6 && offsetof(fsumCvLate_t, t2OnDs) == 10 &&
        offsetof(fsumCvLate_t, t2PriorUgs) == 12, "T2's fields appended to the late offset noise's (%u %u %u)",
        (unsigned)offsetof(fsumCvLate_t, t2K), (unsigned)offsetof(fsumCvLate_t, t2OnDs),
        (unsigned)offsetof(fsumCvLate_t, t2PriorUgs));
  CHECK(sizeof(fsumCvLate_t) == 14 && offsetof(flightSummary_t, camVzLate) == 148,
        "the late offset noise's fields after the re-zero (%u at %u)", (unsigned)sizeof(fsumCvLate_t),
        (unsigned)offsetof(flightSummary_t, camVzLate));
  CHECK(offsetof(flightSummary_t, gtcResMdps) == 140 && offsetof(flightSummary_t, gtcResDtCc) == 144 &&
        offsetof(flightSummary_t, gtcResLandDs) == 146 && offsetof(flightSummary_t, gtcResFlags) == 147,
        "the post-flight re-zero at the end (%u %u %u %u)", (unsigned)offsetof(flightSummary_t, gtcResMdps),
        (unsigned)offsetof(flightSummary_t, gtcResDtCc), (unsigned)offsetof(flightSummary_t, gtcResLandDs),
        (unsigned)offsetof(flightSummary_t, gtcResFlags));
  CHECK(offsetof(flightSummary_t, gtcYEndMdps) == 134 && offsetof(flightSummary_t, mkiEndMdps) == 136,
        "the thermal fields at the end (%u %u)", (unsigned)offsetof(flightSummary_t, gtcYEndMdps),
        (unsigned)offsetof(flightSummary_t, mkiEndMdps));
  CHECK(offsetof(flightSummary_t, imuT0Hc) == 130 && offsetof(flightSummary_t, imuT1Hc) == 131 &&
        offsetof(flightSummary_t, camAwakeDs) == 132, "the new fields at the end (in the old tail padding and after) (%u %u %u)",
        (unsigned)offsetof(flightSummary_t, imuT0Hc), (unsigned)offsetof(flightSummary_t, imuT1Hc),
        (unsigned)offsetof(flightSummary_t, camAwakeDs));
  flightSummary_t fs;
  memset(&fs, 0xA5, sizeof(fs));
  fsumStart(&fs, 1, 2, 3900);
  CHECK(fs.autoReason == 0 && fs.end == FSUM_END_OPEN, "start clears autoReason");
  CHECK(fs.imuT0Hc == FSUM_TEMP_NONE && fs.imuT1Hc == FSUM_TEMP_NONE && fs.camAwakeDs == FSUM_AGE_NONE,
        "start marks the caller's temperature and camera fields unset");
  CHECK(fs.gtcYEndMdps == 0 && fs.mkiEndMdps[0] == 0 && fs.mkiEndMdps[1] == 0, "start zeroes the thermal fields");
  CHECK(fs.gtcResFlags == 0 && fs.gtcResMdps[0] == 0 && fs.gtcResMdps[1] == 0 && fs.gtcResDtCc == 0 &&
        fs.gtcResLandDs == 0, "start: no post-flight re-zero yet");
  fsumSample_t s = hover();
  fsumAdd(&fs, &s);
  CHECK(fs.autoReason == 0, "fsumAdd leaves it to the caller");
  CHECK(fs.imuT0Hc == FSUM_TEMP_NONE && fs.imuT1Hc == FSUM_TEMP_NONE && fs.camAwakeDs == FSUM_AGE_NONE,
        "and the temperature and camera fields");
  CHECK(fs.gtcYEndMdps == 0 && fs.mkiEndMdps[0] == 0 && fs.mkiEndMdps[1] == 0, "and the thermal fields");
  CHECK(fs.gtcResFlags == 0, "and the post-flight re-zero");
  /* the post-flight re-zero (set by flight_rtc_log.c when the driver's airborne re-zero comes) */
  fsumSetGtcResidual(&fs, 30, 50, -960, 2540, true);
  CHECK(fs.gtcResMdps[0] == 30 && fs.gtcResMdps[1] == 50 && fs.gtcResDtCc == -960 && fs.gtcResLandDs == 25 &&
        fs.gtcResFlags == (FSUM_GTCR_SET | FSUM_GTCR_ON), "residual set: %d %d %d %u %u", fs.gtcResMdps[0],
        fs.gtcResMdps[1], fs.gtcResDtCc, fs.gtcResLandDs, fs.gtcResFlags);
  fsumSetGtcResidual(&fs, -65534, 70000, 40000, 600000, false);
  CHECK(fs.gtcResMdps[0] == -32767 && fs.gtcResMdps[1] == 32767 && fs.gtcResDtCc == 32767 && fs.gtcResLandDs == 255 &&
        fs.gtcResFlags == FSUM_GTCR_SET, "residual saturates; off");
  s.thrust = 0;
  s.linkAgeMs = 2000;
  s.thrustLocked = true;
  CHECK(fsumEndCause(&s) != FSUM_END_AUTO, "fsumEndCause never gives AUTO");
}

static void testCrumb(void)
{
  fsumSample_t s = hover();
  s.roll = -10000;      /* -100 deg: clamps */
  s.pitch = 75;         /* 0.75 deg -> 1 half-degree */
  s.gx = -3000;         /* 300 deg/s: caps at 255 */
  s.holdEngaged = true;
  s.linkAgeMs = 9000;
  fsumCrumb_t c;
  fsumCrumb(&c, &s);
  CHECK(c.thrust == 45000 && c.vbatMv == 3500, "crumb thrust/voltage");
  CHECK(c.motor[3] == 46000 >> 8, "crumb motor");
  CHECK(c.roll == -127 && c.pitch == 1, "crumb attitude %d %d", c.roll, c.pitch);
  CHECK(c.gyroDps == 255, "crumb gyro cap");
  CHECK(c.flags == (FSUM_CRUMB_HOLD | FSUM_CRUMB_AIR), "crumb flags 0x%x", c.flags);
  CHECK(c.linkAge20ms == 255, "crumb link age cap");
  CHECK(sizeof(fsumCrumb_t) == 16, "crumb size %u", (unsigned)sizeof(fsumCrumb_t));
  CHECK(c.rail10mv == 0 && c.txPk == 0 && c.rxPk == 0, "caller-filled crumb fields start at 0");
  CHECK(fsumRailCode(2612) == 61 && fsumRailCode(1900) == 1 && fsumRailCode(5000) == 255 &&
        fsumRailCode(0) == 0 && fsumRailCode(65535) == 255,
        "rail code %u %u %u %u %u", fsumRailCode(2612), fsumRailCode(1900), fsumRailCode(5000), fsumRailCode(0),
        fsumRailCode(65535));
}

int main(void)
{
  testStartAndGround();
  testSettleAndSums();
  testHoldAndCamera();
  testSaturation();
  testEndCause();
  testAutoEnd();
  testCrumb();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
