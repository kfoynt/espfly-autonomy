/*
 * Host regression test of the flight recorder's records
 * (components/core/crazyflie/modules/src/flight_recorder.c, docs/autonomy-a2b.md 19.11): flight_recorder.c
 * compiled with the stubs of tools/tests/flight_recorder_host/stub (motors.h) and
 * tools/tests/autonomy_glue_host/stub (FreeRTOS, ESP-IDF, power), the real drone headers otherwise. Every
 * function the recorder calls is defined below and returns a value drawn for the sample (seeded xorshift): the
 * attitude, rates, accelerations, integrals, hold, flow, camera height and autonomy fields over their ranges and
 * past the int16 limits, NaN, +-inf, -0, half-integers after the scale (lrintf's ties to even), the thermal fields
 * with the bisect toggle on and off, flow present and absent, thrust and autonomy starting and ending flights.
 *
 * The first 124 B of every record the recorder hands to the flight summary (flightRtcLogSample), the latest sample
 * and the ring are hashed (FNV-1a 64). The hash is pinned from 39e80e2, where GCC inlined sat16 at its 28 call
 * sites: since sat16 is out of line (19.11, the recorder's flash code every 10 ms) the records must stay byte for
 * byte the same. Any later change to the recorder's arithmetic must keep it, or say why it changes. The 126 B
 * record (atcCorrZ appended, docs/autonomy-a2b.md 36) is pinned separately from its addition.
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=gnu99 -O2 -Wall -Wextra -Itools/tests/flight_recorder_host/stub -I$C/modules/interface \
 *      -Itools/tests/autonomy_glue_host/stub -I$C/hal/interface -I$C/utils/interface -Icomponents/config/include \
 *      tools/tests/test_flight_recorder_host.c $C/modules/src/flight_recorder.c -lm -o /tmp/test_flight_recorder_host
 *   /tmp/test_flight_recorder_host          (--print: the hashes only)
 */
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "flight_recorder.h"
#include "flight_rtc_log.h"
#include "camera_flow.h"
#include "cam_vz_glue.h"
#include "pm_esplane.h"
#include "controller_pid.h"
#include "crtp_commander.h"
#include "attitude_controller.h"
#include "autonomy.h"
#include "motors.h"
#include "system.h"
#include "sensfusion6.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "bench_bypass.h"

/* 39e80e2's flight_recorder.c (sat16 inlined at all 28 sites), this test's inputs (-O0 and -O2 alike). The test
 * is sensitive to sat16: lroundf for lrintf, -32768 as the lower limit, 1 for NaN or a plain cast each change both
 * (checked once by hand, scratchpad lr/fixer/mut.sh). */
#define PINNED_RTC_HASH  0xE92337A1EB681F4Cull
#define PINNED_RING_HASH 0x68472DAEA6FCF5B5ull
/* The 126 B record with atcCorrZ (a2b 36), pinned at its addition; the first 124 B keep the pins above. */
#define PINNED_RTC_HASH_126  0x5BFE246C112D6674ull
#define PINNED_RING_HASH_126 0x4F92A7F4559E8475ull
#define SAMPLES 6000

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

/* ---- the inputs of one sample ---- */
typedef struct {
  float iterm[4];
  bool armed, inhibited;
  int motor[4];
  float vbat;
  float holdR, holdP, corrR, corrP;
  bool holdEng, holdOn, holdCoast, holdGated;
  rpytLinkStats_t link;
  uint32_t tick;
  bool haveFlow;
  cameraFlowSample_t flow;
  int64_t nowUs;
  camVzShadowRec_t cv;
  autoRecFields_t af;
} frecIn_t;

static frecIn_t in;

static uint64_t rng = 0x9E3779B97F4A7C15ull;
static uint64_t next64(void)
{
  rng ^= rng << 13;
  rng ^= rng >> 7;
  rng ^= rng << 17;
  return rng;
}
static uint32_t below(uint32_t n)
{
  return (uint32_t)(next64() % n);
}
static float uni(float lo, float hi)
{
  return lo + (hi - lo) * (float)((next64() >> 40) * (1.0 / 16777216.0));
}

/* A raw input that the recorder scales by k before sat16: specials, the int16 limits, ties, wide values. */
static float gen(float k)
{
  static const float special[] = { NAN, INFINITY, -INFINITY, 0.0f, -0.0f, 1e30f, -1e30f, FLT_MIN, -FLT_MIN };
  static const float edge[] = { 32767.0f, 32767.4f, 32767.5f, 32767.6f, 32768.0f, 40000.0f, -32767.0f, -32767.5f,
                                -32768.0f, -32769.0f, 0.5f, 1.5f, 2.5f, -0.5f, -1.5f, -2.5f, 0.49999997f };
  switch (below(8)) {
    case 0:
      return special[below(sizeof(special) / sizeof(special[0]))];
    case 1:
      return edge[below(sizeof(edge) / sizeof(edge[0]))] / k;
    case 2:
      return ((float)((int32_t)below(65536) - 32768) + 0.5f) / k;   /* a tie after the scale (or near one) */
    default:
      return uni(-36000.0f, 36000.0f) / k;
  }
}

/* ---- the recorder's dependencies ---- */
bool glueHostQuiet = true;
void glueHostLock(portMUX_TYPE *m) { (void)m; }
void glueHostUnlock(portMUX_TYPE *m) { (void)m; }
TickType_t xTaskGetTickCount(void) { return in.tick; }
void vTaskDelay(TickType_t t) { (void)t; }
int64_t esp_timer_get_time(void) { return in.nowUs; }
size_t heap_caps_get_free_size(uint32_t caps) { (void)caps; return 8u * 1024u * 1024u; }
void *heap_caps_malloc(size_t size, uint32_t caps) { (void)caps; return malloc(size); }

volatile uint32_t benchBypassMask;
volatile int16_t sensorsMpu6050ImuTempRaw;
float sensorsMpu6050GyroTcCorrNow[3];
float sensorsMpu6050AccTcCorrNow[3];   /* the accelerometer compensation (accel_tempco.h), g */
float integralFBx, integralFBy;

void attitudeControllerGetIterms(float *rollI, float *pitchI, float *rollRateI, float *pitchRateI)
{
  *rollI = in.iterm[0];
  *pitchI = in.iterm[1];
  *rollRateI = in.iterm[2];
  *pitchRateI = in.iterm[3];
}
bool systemIsArmed() { return in.armed; }
bool motorsIsInhibited(void) { return in.inhibited; }
int motorsGetRatio(uint32_t id) { return in.motor[id & 3]; }
float pmGetBatteryVoltage(void) { return in.vbat; }
void cameraFlowGetHold(float *roll, float *pitch, bool *engaged, uint64_t *timeUs)
{
  *roll = in.holdR;
  *pitch = in.holdP;
  *engaged = in.holdEng;
  *timeUs = 0;
}
void controllerPidGetCameraCorrection(float *rollDeg, float *pitchDeg)
{
  *rollDeg = in.corrR;
  *pitchDeg = in.corrP;
}
bool cameraFlowHoldEnabled(void) { return in.holdOn; }
bool cameraFlowHoldCoasting(void) { return in.holdCoast; }
bool cameraFlowHoldSceneGated(void) { return in.holdGated; }
void crtpCommanderRpytGetLinkStats(rpytLinkStats_t *out) { *out = in.link; }
bool cameraFlowGetLatest(cameraFlowSample_t *out)
{
  if (!in.haveFlow) {
    return false;
  }
  *out = in.flow;
  return true;
}
void camVzGlueRecord(camVzShadowRec_t *r) { *r = in.cv; }
void autonomyRecordFields(autoRecFields_t *out) { *out = in.af; }

static uint64_t rtcHash = 0xcbf29ce484222325ull, rtcHashAll = 0xcbf29ce484222325ull;
static uint32_t rtcCalls;
static void fnv(uint64_t *h, const void *p, size_t n)
{
  const uint8_t *b = p;
  for (size_t i = 0; i < n; i++) {
    *h = (*h ^ b[i]) * 0x100000001b3ull;
  }
}
void flightRtcLogSample(const flightRecord_t *r)
{
  fnv(&rtcHash, r, 124);   /* the 19fe168 record: the old pins */
  fnv(&rtcHashAll, r, sizeof(*r));
  rtcCalls++;
}

/* ---- the samples ---- */
static void draw(uint32_t i, setpoint_t *sp, state_t *st, sensorData_t *se)
{
  memset(sp, 0, sizeof(*sp));
  memset(st, 0, sizeof(*st));
  memset(se, 0, sizeof(*se));
  memset(&in, 0, sizeof(in));

  /* Flights: thrust blips, a thrust burst (with NaN and out-of-range now and then), >= 1 s of neither (the
   * recorder's flight gap), an autonomy sortie. */
  const uint32_t phase = (i / 400) % 4;
  sp->thrust = phase == 1 ? uni(-1000.0f, 70000.0f) : (phase == 0 && below(50) == 0 ? uni(1.0f, 40000.0f) : 0.0f);
  if (phase == 1 && below(100) == 0) {
    sp->thrust = below(2) ? NAN : 1e9f;
  }
  sp->mode.roll = below(3) == 0 ? modeVelocity : modeAbs;
  sp->attitude.roll = gen(100.0f);
  sp->attitude.pitch = gen(100.0f);
  sp->attitudeRate.yaw = gen(10.0f);
  st->attitude.roll = gen(100.0f);
  st->attitude.pitch = gen(100.0f);
  st->attitude.yaw = gen(100.0f);
  se->gyro.x = gen(10.0f);
  se->gyro.y = gen(10.0f);
  se->gyro.z = gen(10.0f);
  se->acc.x = gen(1000.0f);
  se->acc.y = gen(1000.0f);
  se->acc.z = gen(1000.0f);

  in.iterm[0] = gen(100.0f);
  in.iterm[1] = gen(100.0f);
  in.iterm[2] = gen(1.0f);
  in.iterm[3] = gen(1.0f);
  in.armed = below(2);
  in.inhibited = below(5) == 0;
  for (int m = 0; m < 4; m++) {
    in.motor[m] = (int)below(65536);
  }
  in.vbat = below(100) == 0 ? NAN : uni(-0.5f, 70.0f);
  in.holdR = uni(-5.0f, 5.0f);
  in.holdP = uni(-5.0f, 5.0f);
  in.corrR = gen(100.0f);
  in.corrP = gen(100.0f);
  in.holdEng = below(2);
  in.holdOn = below(2);
  in.holdCoast = below(4) == 0;
  in.holdGated = below(4) == 0;

  in.tick = 1000u + i * 10u;
  in.link.packets = below(4) ? 1u + below(100000) : 0u;
  in.link.lastPacketMs = in.tick - below(80000);
  in.link.lastRawThrust = (uint16_t)below(65536);
  in.link.thrustLocked = below(3) == 0;

  in.nowUs = (int64_t)in.tick * 1000 + (int64_t)below(1000);
  in.haveFlow = below(6) != 0;
  in.flow.seq = below(100000);
  const int64_t age = (int64_t)below(400000);   /* one draw a statement: the order is the same on every compiler */
  const int64_t ahead = below(20) == 0 ? 50000 : 0;
  in.flow.frameTimeUs = (uint64_t)(in.nowUs - age + ahead);
  in.flow.dtUs = below(10000000);
  in.flow.dx = gen(100.0f);
  in.flow.dy = gen(100.0f);
  in.flow.scale = gen(1e4f);
  in.flow.rot = gen(1e4f);
  for (int k = 0; k < 3; k++) {
    in.flow.gyroAngle[k] = gen(1e4f);
  }
  in.flow.texture = below(30) == 0 ? NAN : uni(-10.0f, 400.0f);
  in.flow.residual = below(30) == 0 ? NAN : uni(-1.0f, 40.0f);
  in.flow.lum = (uint8_t)below(256);
  in.flow.ok = below(2);
  in.flow.gainE3 = (int8_t)((int)below(255) - 127);
  in.flow.satPct = (uint8_t)below(101);
  in.flow.satPct |= below(8) == 0 ? CAMERA_FLOW_SAT_EXPGATE : 0;

  in.cv.hMm = (int16_t)below(65536);
  in.cv.vMms = (int16_t)below(65536);
  in.cv.biasCmg = (int16_t)below(65536);
  in.cv.gainDpm = (uint16_t)below(65536);
  in.cv.flags = (uint8_t)below(256);
  in.cv.sigVMms = (uint8_t)below(256);

  in.af.autoState = phase == 3 ? (uint8_t)(1 + below(8)) : 0;   /* AUTO_IDLE outside the sortie phase */
  in.af.autoReason = (uint8_t)below(32);
  in.af.vzPhase = (uint8_t)below(8);
  in.af.vzHmm = (int16_t)below(65536);
  in.af.vzVmms = (int16_t)below(65536);
  in.af.vzThrust = (uint16_t)below(65536);
  in.af.autoFlags = (uint8_t)below(256);
  in.af.mvState = (uint8_t)below(256);
  in.af.mvRefLat = (int16_t)below(65536);
  in.af.mvRefFwd = (int16_t)below(65536);
  in.af.mvVRef = (int16_t)below(65536);
  in.af.mvYawOff = (int16_t)below(65536);

  /* The thermal fields: the driver's and the filter's published values, the bisect toggle now and then. */
  benchBypassMask = below(10) == 0 ? BENCH_BYPASS_REC : (below(10) == 0 ? BENCH_BYPASS_GTC : 0u);
  sensorsMpu6050ImuTempRaw = (int16_t)below(65536);
  for (int k = 0; k < 3; k++) {
    sensorsMpu6050GyroTcCorrNow[k] = gen(1000.0f);
  }
  integralFBx = gen(1000.0f * 180.0f / 3.14159265f);
  integralFBy = gen(1000.0f * 180.0f / 3.14159265f);
}

int main(int argc, char **argv)
{
  const bool printOnly = argc > 1 && strcmp(argv[1], "--print") == 0;
  CHECK(sizeof(flightRecord_t) == 126, "record size %zu", sizeof(flightRecord_t));

  flightRecorderInit();
  CHECK(flightRecorderCapacity() == FLIGHT_RECORDER_SAMPLES_PSRAM, "capacity %u",
        (unsigned)flightRecorderCapacity());

  uint64_t latestHash = 0xcbf29ce484222325ull, latestHashAll = 0xcbf29ce484222325ull;
  for (uint32_t i = 0; i < SAMPLES; i++) {
    setpoint_t sp;
    state_t st;
    sensorData_t se;
    draw(i, &sp, &st, &se);
    flightRecorderSample(&sp, &st, &se, in.tick);
    flightRecord_t latest;
    CHECK(flightRecorderGetLatest(&latest), "no latest sample at %u", (unsigned)i);
    fnv(&latestHash, &latest, 124);
    fnv(&latestHashAll, &latest, sizeof(latest));
  }

  flightRecorderFreeze(true);
  const uint32_t n = flightRecorderCount();
  uint64_t ringHash = 0xcbf29ce484222325ull, ringHashAll = 0xcbf29ce484222325ull;
  for (uint32_t i = 0; i < n; i++) {
    flightRecord_t r;
    CHECK(flightRecorderGet(i, &r), "ring %u", (unsigned)i);
    fnv(&ringHash, &r, 124);
    fnv(&ringHashAll, &r, sizeof(r));
  }
  flightRecorderFreeze(false);

  printf("records %u (summary calls %u), ring %u: rtc hash 0x%016llX, latest hash 0x%016llX, ring hash 0x%016llX "
         "(the first 124 B); all 126 B: rtc 0x%016llX, latest 0x%016llX, ring 0x%016llX\n",
         (unsigned)SAMPLES, (unsigned)rtcCalls, (unsigned)n, (unsigned long long)rtcHash,
         (unsigned long long)latestHash, (unsigned long long)ringHash, (unsigned long long)rtcHashAll,
         (unsigned long long)latestHashAll, (unsigned long long)ringHashAll);
  if (printOnly) {
    return 0;
  }
  CHECK(rtcCalls == SAMPLES, "summary calls %u", (unsigned)rtcCalls);
  CHECK(n > SAMPLES / 4 && n < SAMPLES, "ring count %u: the inputs should start and end flights", (unsigned)n);
  /* The latest sample is the record of the same call. */
  CHECK(latestHash == rtcHash, "latest 0x%016llX != summary 0x%016llX", (unsigned long long)latestHash,
        (unsigned long long)rtcHash);
  CHECK(rtcHash == PINNED_RTC_HASH, "records 0x%016llX, pinned (39e80e2) 0x%016llX", (unsigned long long)rtcHash,
        (unsigned long long)PINNED_RTC_HASH);
  CHECK(ringHash == PINNED_RING_HASH, "ring 0x%016llX, pinned (39e80e2) 0x%016llX", (unsigned long long)ringHash,
        (unsigned long long)PINNED_RING_HASH);
  /* The whole 126 B record (atcCorrZ appended, docs/autonomy-a2b.md 36; the stub's correction is 0): pinned here. */
  CHECK(latestHashAll == rtcHashAll, "latest all 0x%016llX != summary all 0x%016llX", (unsigned long long)latestHashAll,
        (unsigned long long)rtcHashAll);
  CHECK(rtcHashAll == PINNED_RTC_HASH_126, "records (126 B) 0x%016llX, pinned 0x%016llX", (unsigned long long)rtcHashAll,
        (unsigned long long)PINNED_RTC_HASH_126);
  CHECK(ringHashAll == PINNED_RING_HASH_126, "ring (126 B) 0x%016llX, pinned 0x%016llX", (unsigned long long)ringHashAll,
        (unsigned long long)PINNED_RING_HASH_126);

  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
