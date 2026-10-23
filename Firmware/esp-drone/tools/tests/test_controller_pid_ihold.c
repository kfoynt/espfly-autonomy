/*
 * Host test of the autonomy take-off integrator hold in the REAL attitude
 * controller (controller_pid.c, attitude_pid_controller.c, pid.c,
 * torque_trim.c; the gains of CONFIG_TARGET_ESP32_S2_DRONE_V1_2). User
 * decision 2026-10-13, evening (docs/autonomy-a2b.md 2.4): in an autonomous
 * take-off the roll/pitch attitude and rate integrators stay at zero from
 * the sortie's start until the vertical controller's lift-off, also above
 * the 26000 ground thrust; manual flight is unchanged.
 * autonomyTakeoffIntegratorHold() is stubbed here (the glue's side is
 * test_autonomy_glue_host.c's scenario 'ihold').
 *
 * Checks: below 26000 all four are zero; not held, 0.4 s at 30000 resting
 * at roll +3.9 / pitch -0.6 winds them (the floor kick); held, all four are
 * exactly 0 on every loop up to 36000 while P, D, the yaw integrator and the
 * torque-trim feed-forward act; released, they start from 0 and grow; an
 * autonomous RAMP at 13250/s from a tilted floor: exactly 0 on every loop
 * above 26000 before the lift-off, nonzero after it; manual flight: the
 * controller's outputs and integrators over a long manual sequence hash to
 * the value of e46bf90's controller (bit-identical). On e46bf90's
 * controller_pid.c the 'held' checks fail (the integrators wind above 26000).
 *
 * Fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31): the camera correction's apply slew is the hold's own
 * (camera_flow.h cameraFlowHoldSlew): at 6 deg/s every loop's correction is bit-identical to the constant slew of
 * before; at 12 deg/s it moves twice as fast; with the hold stale a fast MOVE's brake lean (autonomy.h
 * autonomyBrakeLean, stubbed) is the target instead of 0, never over pilot input or a blocked hold.
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=gnu99 -O1 -Wall -Wno-absolute-value -DCONFIG_TARGET_ESP32_S2_DRONE_V1_2 \
 *      -Itools/tests/controller_pid_host/stub -I$C/modules/interface -I$C/utils/interface -I$C/hal/interface \
 *      tools/tests/test_controller_pid_ihold.c $C/modules/src/controller_pid.c \
 *      $C/modules/src/attitude_pid_controller.c $C/modules/src/pid.c $C/utils/src/torque_trim.c \
 *      $C/utils/src/num.c $C/utils/src/filter.c -lm -o /tmp/test_controller_pid_ihold && \
 *   /tmp/test_controller_pid_ihold
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "controller_pid.h"
#include "attitude_controller.h"
#include "pid.h"
#include "imu_level_cal_store.h"
#include "camera_flow.h"
#include "usec_time.h"

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* The manual sequence's hash on e46bf90's controller (the same flags, this file): bit-identical manual flight. */
#define MANUAL_HASH_E46BF90 0xF20A1A1FFB479D9Full

/* ---- stubs ---- */
static bool ihold;
static uint64_t nowUs;
bool autonomyTakeoffIntegratorHold(void) { return ihold; }
static bool holdBlocked;
bool autonomyHoldBlocked(void) { return holdBlocked; }
static bool brakeOn;
static float brakeR, brakeP;
bool autonomyBrakeLean(float *r, float *p) { if (!brakeOn) return false; *r = brakeR; *p = brakeP; return true; }
volatile float cameraFlowHoldSlewV = 6.0f;
static float hR, hP;
static bool hEng;
static uint64_t hT;   /* the hold's time stamp (0: the stub's 'now', as before) */
static bool hStale;
void positionControllerInit(void) {}
void positionControllerResetAllPID(void) {}
void positionController(float *thrust, attitude_t *attitude, setpoint_t *setpoint, const state_t *state)
{ (void)thrust; (void)attitude; (void)setpoint; (void)state; }
esp_err_t torqueTrimStoreLoad(torqueTrimRecord_t *rec) { (void)rec; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t attTrimStoreLoad(attTrimRecord_t *rec) { (void)rec; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t yawLockStoreLoad(bool *on) { (void)on; return ESP_ERR_NVS_NOT_FOUND; }
void cameraFlowGetHold(float *r, float *p, bool *e, uint64_t *t)
{
  *r = hR;
  *p = hP;
  *e = hEng;
  *t = hStale ? hT : nowUs;
}
uint64_t usecTimestamp(void) { return nowUs; }
float sensfusion6GetInvThrustCompensationForTilt() { return 1.0f; }
uint32_t imuLevelCalCrc32(const void *d, uint32_t n) { (void)d; (void)n; return 0; }

extern PidObject pidYaw;
static control_t ctl; static setpoint_t sp; static sensorData_t sd; static state_t st;
static uint32_t tick = 1;
/* One 1 kHz stabilizer loop; the attitude and rate PIDs run on every other one (500 Hz). */
static void stepG(float thrust, float roll, float pitch, float yaw, float gx, float gy)
{
  memset(&sp, 0, sizeof(sp));
  sp.mode.x = sp.mode.y = sp.mode.z = modeDisable;
  sp.mode.roll = sp.mode.pitch = sp.mode.yaw = modeAbs;
  sp.thrust = thrust;
  st.attitude.roll = roll; st.attitude.pitch = pitch; st.attitude.yaw = yaw;
  sd.gyro.x = gx; sd.gyro.y = gy; sd.gyro.z = 0.0f;
  controllerPid(&ctl, &sp, &sd, &st, tick);
  tick++; nowUs += 1000;
}
static void step(float thrust, float roll, float pitch, float yaw) { stepG(thrust, roll, pitch, yaw, 0.0f, 0.0f); }
/* One loop with a pilot roll input (the setpoint's attitude.roll). */
static void stepPilot(float thrust, float rollCmd)
{
  memset(&sp, 0, sizeof(sp));
  sp.mode.x = sp.mode.y = sp.mode.z = modeDisable;
  sp.mode.roll = sp.mode.pitch = sp.mode.yaw = modeAbs;
  sp.thrust = thrust;
  sp.attitude.roll = rollCmd;
  controllerPid(&ctl, &sp, &sd, &st, tick);
  tick++; nowUs += 1000;
}

/* The camera correction's apply slew (mvspeed, 2026-10-16). */
static void testHoldSlew(void)
{
  controllerPidInit();
  ihold = false;
  for (int i = 0; i < 4; i++) step(0.0f, 0.0f, 0.0f, 0.0f);
  /* 6 deg/s: every loop's correction equals the constant slew of before, bit for bit (a moving hold target) */
  const float k6 = 6.0f * (float)(1.0f / ATTITUDE_RATE);   /* the folded constant of before */
  float refR = 0.0f, refP = 0.0f, cr, cp;
  controllerPidGetCameraCorrection(&cr, &cp);
  refR = cr;
  refP = cp;
  bool same = true;
  hEng = true;
  hStale = false;
  cameraFlowHoldSlewV = 6.0f;
  for (int i = 0; i < 3000; i++) {
    hR = 2.0f * sinf((float)i * 0.003f) + ((i / 700) % 2 ? 0.7f : -0.4f);
    hP = -1.5f * cosf((float)i * 0.0021f);
    const bool att = (tick % (RATE_MAIN_LOOP / ATTITUDE_RATE)) == 0;
    step(40000.0f, 0.0f, 0.0f, 0.0f);
    if (att) {
      float d = hR - refR;
      d = d > k6 ? k6 : (d < -k6 ? -k6 : d);
      refR += d;
      d = hP - refP;
      d = d > k6 ? k6 : (d < -k6 ? -k6 : d);
      refP += d;
    }
    controllerPidGetCameraCorrection(&cr, &cp);
    same = same && memcmp(&cr, &refR, sizeof(cr)) == 0 && memcmp(&cp, &refP, sizeof(cp)) == 0;
  }
  CHECK(same, "slew 6 deg/s: the applied correction bit-identical to the constant 6 deg/s slew on every loop");
  /* 12 deg/s: a 2 deg step is reached in ~0.17 s (6 deg/s: 0.33 s) */
  hR = 0.0f;
  hP = 0.0f;
  for (int i = 0; i < 1000; i++) step(40000.0f, 0.0f, 0.0f, 0.0f);
  cameraFlowHoldSlewV = 12.0f;
  hR = 2.0f;
  for (int i = 0; i < 100; i++) step(40000.0f, 0.0f, 0.0f, 0.0f);
  controllerPidGetCameraCorrection(&cr, &cp);
  CHECK(fabsf(cr - 1.2f) < 0.03f, "slew 12 deg/s: 1.2 deg after 0.1 s (%.3f)", cr);
  for (int i = 0; i < 100; i++) step(40000.0f, 0.0f, 0.0f, 0.0f);
  controllerPidGetCameraCorrection(&cr, &cp);
  CHECK(cr == 2.0f, "slew 12 deg/s: at the 2 deg target after 0.2 s (%.3f)", cr);
  /* the hold stale (no frame for 0.3 s): no brake -> it fades to 0 as before; a brake -> the brake lean */
  hStale = true;
  hT = nowUs - 400000u;
  for (int i = 0; i < 400; i++) step(40000.0f, 0.0f, 0.0f, 0.0f);
  controllerPidGetCameraCorrection(&cr, &cp);
  CHECK(cr == 0.0f && cp == 0.0f, "stale, no brake: faded to 0 (%.3f %.3f)", cr, cp);
  brakeOn = true;
  brakeR = 0.0f;
  brakeP = 2.2f;   /* nose up: braking a forward move */
  for (int i = 0; i < 400; i++) step(40000.0f, 0.0f, 0.0f, 0.0f);
  controllerPidGetCameraCorrection(&cr, &cp);
  CHECK(cr == 0.0f && cp == 2.2f, "stale, a fast MOVE's brake: the brake lean applied (%.3f %.3f)", cr, cp);
  /* a fresh hold wins over the brake (the hold carries it itself) */
  hStale = false;
  hR = 0.3f;
  hP = 0.5f;
  for (int i = 0; i < 600; i++) step(40000.0f, 0.0f, 0.0f, 0.0f);
  controllerPidGetCameraCorrection(&cr, &cp);
  CHECK(cr == 0.3f && cp == 0.5f, "fresh: the hold's own output, the brake ignored (%.3f %.3f)", cr, cp);
  /* pilot input and a blocked hold keep priority over the brake */
  hStale = true;
  hT = nowUs - 400000u;
  stepPilot(40000.0f, 1.0f);
  stepPilot(40000.0f, 1.0f);
  controllerPidGetCameraCorrection(&cr, &cp);
  CHECK(cr == 0.0f, "stale with a brake, pilot roll input: the roll correction zeroed (%.3f)", cr);
  holdBlocked = true;
  step(40000.0f, 0.0f, 0.0f, 0.0f);
  step(40000.0f, 0.0f, 0.0f, 0.0f);
  controllerPidGetCameraCorrection(&cr, &cp);
  CHECK(cr == 0.0f && cp == 0.0f, "stale with a brake, the hold blocked: zero (%.3f %.3f)", cr, cp);
  holdBlocked = false;
  brakeOn = false;
  hEng = false;
  hStale = false;
  hR = hP = 0.0f;
  cameraFlowHoldSlewV = 6.0f;
  for (int i = 0; i < 4; i++) step(0.0f, 0.0f, 0.0f, 0.0f);
}
static void it(float *a, float *b, float *c, float *d) { attitudeControllerGetIterms(a, b, c, d); }
static bool allZero(void)
{
  float a, b, c, d;
  it(&a, &b, &c, &d);
  return a == 0.0f && b == 0.0f && c == 0.0f && d == 0.0f;
}

static uint64_t fnv(uint64_t h, const void *p, size_t n)
{
  const unsigned char *b = p;
  for (size_t i = 0; i < n; i++) {
    h ^= b[i];
    h *= 0x100000001B3ull;
  }
  return h;
}

/* A long manual flight (the hold never set): throttle ramps through the
 * gate, a tilted floor, stick inputs, gyro noise, zero-thrust resets. */
static uint64_t manualHash(void)
{
  uint64_t h = 0xCBF29CE484222325ull;
  uint32_t rng = 12345u;
  for (int i = 0; i < 20000; i++) {
    rng = rng * 1664525u + 1013904223u;
    const float noise = (float)((int)(rng >> 16) % 2001 - 1000) * 0.002f;
    const float ph = (float)(i % 5000);
    const float thrust = i % 5000 < 300 ? 0.0f : fminf(48000.0f, 15000.0f + 12.0f * ph);
    const float roll = 3.9f * expf(-ph / 800.0f) + 0.4f * sinf(ph * 0.01f) + noise;
    const float pitch = -0.6f + 0.3f * cosf(ph * 0.013f) - noise;
    stepG(thrust, roll, pitch, 2.0f * sinf(ph * 0.002f), 10.0f * noise, -8.0f * noise);
    float ia[4];
    it(&ia[0], &ia[1], &ia[2], &ia[3]);
    h = fnv(h, &ctl.roll, sizeof(ctl.roll));
    h = fnv(h, &ctl.pitch, sizeof(ctl.pitch));
    h = fnv(h, &ctl.yaw, sizeof(ctl.yaw));
    h = fnv(h, ia, sizeof(ia));
  }
  return h;
}

int main(void)
{
  controllerPidInit();
  float a, b, c, d;
  /* below the 26000 ground gate: all zero, hold or not */
  for (int i = 0; i < 1000; i++) step(20000.0f, 3.9f, -0.6f, 0.0f);
  CHECK(allZero(), "below 26000: all four zero");

  /* manual (hold false): 0.4 s at 30000 resting at roll +3.9, pitch -0.6 */
  for (int i = 0; i < 400; i++) step(30000.0f, 3.9f, -0.6f, 0.0f);
  it(&a, &b, &c, &d);
  CHECK(a < -3.5f && c < -3500.0f && b > 0.5f && d > 500.0f, "not held: all four wind (%.2f %.0f %.2f %.0f)", a, c, b, d);
  printf("info: 0.4 s at 30000, roll err -3.9, pitch err +0.6, not held: rollI %.2f rollRateI %.0f pitchI %.2f "
         "pitchRateI %.0f\n", a, c, b, d);
  for (int i = 0; i < 4; i++) step(0.0f, 3.9f, -0.6f, 0.0f);
  CHECK(allZero(), "zero thrust resets");

  /* held: 0.4 s at 30000..36000 at the same rest tilt, with a yaw error */
  ihold = true;
  bool zero = true;
  for (int i = 0; i < 400; i++) {
    step(30000.0f + 15.0f * i, 3.9f, -0.6f, 5.0f);
    zero = zero && allZero();
  }
  it(&a, &b, &c, &d);
  CHECK(zero, "held: all four roll/pitch integrators exactly 0 on every loop (%.2f %.0f %.2f %.0f)", a, c, b, d);
  CHECK(ctl.roll < -3500 && ctl.roll > -4300, "held: the roll command is the P path alone, KP_R KP_A x err (%d)",
        ctl.roll);
  CHECK(pidYaw.integ != 0.0f, "held: the yaw integrator untouched (%g)", (double)pidYaw.integ);
  printf("info: held: roll command %d (the P path; -8676 with the 0.4 s of wind-up)\n", ctl.roll);
  /* the torque-trim feed-forward acts while held (fully in from 30000) */
  const int16_t r0 = ctl.roll, p0 = ctl.pitch;
  CHECK(controllerPidSetTorqueTrim(1500.0f, -2000.0f), "torque trim set");
  step(36000.0f, 3.9f, -0.6f, 5.0f);
  step(36000.0f, 3.9f, -0.6f, 5.0f);
  CHECK(allZero() && abs(ctl.roll - (r0 + 1500)) <= 2 && abs(ctl.pitch - (p0 - 2000)) <= 2,
        "held: the torque trim is applied in full (roll %d -> %d, pitch %d -> %d)", r0, ctl.roll, p0, ctl.pitch);
  controllerPidSetTorqueTrim(0.0f, 0.0f);

  /* released (the controller's lift-off): they start from zero */
  ihold = false;
  step(36000.0f, 3.9f, -0.6f, 0.0f);
  step(36000.0f, 3.9f, -0.6f, 0.0f);
  it(&a, &b, &c, &d);
  CHECK(a < 0.0f && a > -0.05f && c < 0.0f && c > -60.0f, "released: they start from 0 (%.4f %.1f)", a, c);
  for (int i = 0; i < 4; i++) step(0.0f, 0.0f, 0.0f, 0.0f);

  /* an autonomous take-off from a tilted floor: the RAMP at 13250/s from 20000 (vert_ctrl.c), the hold set from the
   * sortie's start, the lift-off at 35000 (the floor's rest tilt until then), then level flight */
  ihold = true;
  int above = 0, heldAbove = 0, after = 0, windAfter = 0;
  float thrust = 0.0f;
  for (int i = 0; i < 3000; i++) {
    const bool lifted = thrust >= 35000.0f;
    if (lifted) {
      ihold = false;   /* the step that latches the lift-off releases it */
    }
    thrust = i < 200 ? 0.0f : fminf(20000.0f + 13.25f * (float)(i - 200), 38000.0f);
    const float roll = lifted ? 0.5f : 3.9f, pitch = lifted ? -0.2f : -0.6f;
    step(thrust, roll, pitch, 0.0f);
    if (!lifted && thrust >= 26000.0f) {
      above++;
      heldAbove += allZero();
    }
    if (lifted) {
      after++;
      it(&a, &b, &c, &d);
      windAfter += a != 0.0f && b != 0.0f && c != 0.0f && d != 0.0f;
    }
  }
  CHECK(above > 600 && heldAbove == above, "autonomous RAMP: all four exactly 0 on every loop above 26000 before the "
        "lift-off (%d of %d)", heldAbove, above);
  CHECK(after > 1000 && windAfter >= after - 2, "after the lift-off: all four integrate (%d of %d loops)", windAfter,
        after);
  for (int i = 0; i < 4; i++) step(0.0f, 0.0f, 0.0f, 0.0f);

  testHoldSlew();

  /* manual flight: bit-identical to e46bf90's controller */
  ihold = false;
  controllerPidInit();
  const uint64_t h = manualHash();
  printf("info: manual sequence hash 0x%016llX (e46bf90: 0x%016llX)\n", (unsigned long long)h,
         (unsigned long long)MANUAL_HASH_E46BF90);
  CHECK(h == MANUAL_HASH_E46BF90, "manual flight (the hold never set): outputs and integrators bit-identical to "
        "e46bf90's controller over 20 s");
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
