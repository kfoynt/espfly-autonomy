/*
 * Host unit tests for components/core/crazyflie/utils/src/imu_level_cal.c
 * and torque_trim.c
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_imu_level_cal.c components/core/crazyflie/utils/src/imu_level_cal.c \
 *      components/core/crazyflie/utils/src/torque_trim.c \
 *      -lm -o /tmp/test_imu_level_cal && /tmp/test_imu_level_cal
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu_level_cal.h"
#include "torque_trim.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define DEG2RAD(d) ((d) * (float)M_PI / 180.0f)
#define RAD2DEG(r) ((r) * 180.0f / (float)M_PI)

static float norm3(const float v[3])
{
  return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/* Estimator conventions from sensfusion6GetEulerRPY() for a normalised
 * gravity direction in the body frame. */
static float estPitchDeg(const float a[3]) { return RAD2DEG(asinf(a[0] / norm3(a))); }
static float estRollDeg(const float a[3]) { return RAD2DEG(atan2f(a[1], a[2])); }

static void matrixOf(const imuAlignment_t *al, float m[3][3])
{
  for (int c = 0; c < 3; c++) {
    float e[3] = {0, 0, 0};
    float out[3];
    e[c] = 1.0f;
    imuAlignmentApply(al, e, out);
    for (int r = 0; r < 3; r++) {
      m[r][c] = out[r];
    }
  }
}

static void testCrc(void)
{
  const char *s = "123456789";
  CHECK(imuLevelCalCrc32(s, 9) == 0xCBF43926u, "crc32 check value 0x%08x", (unsigned)imuLevelCalCrc32(s, 9));
}

static void testRecord(void)
{
  imuLevelCalRecord_t rec;
  imuLevelCalRecordInit(&rec, 0.031f, -0.012f);
  CHECK(imuLevelCalRecordValid(&rec, sizeof(rec)), "fresh record valid");
  CHECK(!imuLevelCalRecordValid(&rec, sizeof(rec) - 1), "short stored size rejected");

  imuLevelCalRecord_t bad = rec;
  bad.accOffsetX = 0.032f;
  CHECK(!imuLevelCalRecordValid(&bad, sizeof(bad)), "corrupted payload rejected by crc");

  bad = rec;
  bad.version = IMU_LEVEL_CAL_VERSION + 1;
  CHECK(!imuLevelCalRecordValid(&bad, sizeof(bad)), "unknown version rejected");

  imuLevelCalRecordInit(&bad, 0.2f, 0.0f);
  CHECK(!imuLevelCalRecordValid(&bad, sizeof(bad)), "out-of-range offset rejected");

  imuLevelCalRecordInit(&bad, NAN, 0.0f);
  CHECK(!imuLevelCalRecordValid(&bad, sizeof(bad)), "NaN offset rejected");

  imuLevelCalRecordInit(&bad, 0.0f, INFINITY);
  CHECK(!imuLevelCalRecordValid(&bad, sizeof(bad)), "Inf offset rejected");

  CHECK(imuLevelCalOffsetsValid(IMU_LEVEL_CAL_MAX_OFFSET_G, -IMU_LEVEL_CAL_MAX_OFFSET_G), "boundary accepted");
}

static void testLevelOffsetRemovesTilt(void)
{
  /* A level drone whose sensor reports a 2.5 deg pitch / -1.2 deg roll error. */
  float a[3] = {sinf(DEG2RAD(2.5f)), -sinf(DEG2RAD(1.2f)), 1.0f};
  float corrected[3] = {a[0] - a[0], a[1] - a[1], a[2]};
  CHECK(fabsf(estPitchDeg(a) - 2.5f) < 0.01f, "pitch error visible before correction");
  CHECK(fabsf(estPitchDeg(corrected)) < 1e-4f && fabsf(estRollDeg(corrected)) < 1e-4f,
        "subtracting the level offset reads level");
}

static void testAlignmentIdentity(void)
{
  imuAlignment_t al;
  imuAlignmentInit(&al, 0.0f, 0.0f);
  float in[3] = {0.1f, -0.2f, 0.97f};
  float out[3];
  imuAlignmentApply(&al, in, out);
  for (int i = 0; i < 3; i++) {
    CHECK(out[i] == in[i], "zero alignment is identity (axis %d)", i);
  }
}

static void testAlignmentIsRotation(void)
{
  const float angles[][2] = {{0, 2}, {2, 0}, {1.5f, -1.8f}, {-2, 2}, {10, -25}, {-45, 30}};
  for (unsigned k = 0; k < sizeof(angles) / sizeof(angles[0]); k++) {
    imuAlignment_t al;
    float m[3][3];
    imuAlignmentInit(&al, angles[k][0], angles[k][1]);
    matrixOf(&al, m);

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        float dot = m[0][i] * m[0][j] + m[1][i] * m[1][j] + m[2][i] * m[2][j];
        CHECK(fabsf(dot - (i == j ? 1.0f : 0.0f)) < 1e-5f,
              "orthonormal roll=%g pitch=%g (%d,%d)=%g", angles[k][0], angles[k][1], i, j, dot);
      }
    }

    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    CHECK(fabsf(det - 1.0f) < 1e-5f, "det +1 roll=%g pitch=%g det=%g", angles[k][0], angles[k][1], det);

    /* Inverse (transpose) round trip. */
    float v[3] = {0.3f, -0.4f, 0.86f};
    float w[3], back[3];
    imuAlignmentApply(&al, v, w);
    for (int r = 0; r < 3; r++) {
      back[r] = m[0][r] * w[0] + m[1][r] * w[1] + m[2][r] * w[2];
    }
    CHECK(fabsf(norm3(w) - norm3(v)) < 1e-5f, "norm preserved");
    for (int r = 0; r < 3; r++) {
      CHECK(fabsf(back[r] - v[r]) < 1e-5f, "transpose inverts (axis %d)", r);
    }
  }
}

static void testAlignmentSignConvention(void)
{
  /* Kconfig help: "if PITCH error angle is 0.90, set this 90". */
  const float errs[] = {0.9f, -0.9f, 2.0f, -1.5f};
  for (unsigned k = 0; k < sizeof(errs) / sizeof(errs[0]); k++) {
    float e = errs[k];
    imuAlignment_t al;
    float out[3];

    float pitchErr[3] = {sinf(DEG2RAD(e)), 0.0f, cosf(DEG2RAD(e))};
    CHECK(fabsf(estPitchDeg(pitchErr) - e) < 1e-3f, "pitch fixture");
    imuAlignmentInit(&al, 0.0f, e);
    imuAlignmentApply(&al, pitchErr, out);
    CHECK(fabsf(estPitchDeg(out)) < 1e-3f, "PITCH_CALIB=%g removes +%g pitch reading (got %g)", e, e, estPitchDeg(out));

    float rollErr[3] = {0.0f, sinf(DEG2RAD(e)), cosf(DEG2RAD(e))};
    CHECK(fabsf(estRollDeg(rollErr) - e) < 1e-3f, "roll fixture");
    imuAlignmentInit(&al, e, 0.0f);
    imuAlignmentApply(&al, rollErr, out);
    CHECK(fabsf(estRollDeg(out)) < 1e-3f, "ROLL_CALIB=%g removes +%g roll reading (got %g)", e, e, estRollDeg(out));
  }
}

static void testOldPitchFormulaRegression(void)
{
  /* The original firmware pitch step: x' = x c - z s ; z' = -x s + z c. */
  float s = sinf(DEG2RAD(2.0f)), c = cosf(DEG2RAD(2.0f));
  float in[3] = {sinf(DEG2RAD(2.0f)), 0.0f, cosf(DEG2RAD(2.0f))};
  float oldOut[3] = {in[0] * c - in[2] * s, in[1], -in[0] * s + in[2] * c};
  float oldDet = c * c - s * s;
  CHECK(fabsf(oldDet - 1.0f) > 1e-3f, "old pitch formula is not a rotation (det=%g)", oldDet);

  imuAlignment_t al;
  float newOut[3];
  imuAlignmentInit(&al, 0.0f, 2.0f);
  imuAlignmentApply(&al, in, newOut);
  CHECK(fabsf(norm3(newOut) - 1.0f) < 1e-6f, "fixed formula keeps unit length");
  CHECK(fabsf(norm3(oldOut) - 1.0f) > 1e-3f, "old formula changed length to %g", norm3(oldOut));
}

static void testTorqueTrimRecord(void)
{
  torqueTrimRecord_t rec;
  torqueTrimRecordInit(&rec, 2370.0f, -4435.0f);
  CHECK(torqueTrimRecordValid(&rec, sizeof(rec)), "fresh trim record valid");
  CHECK(!torqueTrimRecordValid(&rec, sizeof(rec) + 4), "wrong stored size rejected");

  torqueTrimRecord_t bad = rec;
  bad.pitch = -4436.0f;
  CHECK(!torqueTrimRecordValid(&bad, sizeof(bad)), "corrupted trim rejected by crc");

  bad = rec;
  bad.version = TORQUE_TRIM_VERSION + 1;
  CHECK(!torqueTrimRecordValid(&bad, sizeof(bad)), "unknown trim version rejected");

  torqueTrimRecordInit(&bad, TORQUE_TRIM_MAX + 1.0f, 0.0f);
  CHECK(!torqueTrimRecordValid(&bad, sizeof(bad)), "out-of-range trim rejected");
  torqueTrimRecordInit(&bad, 0.0f, NAN);
  CHECK(!torqueTrimRecordValid(&bad, sizeof(bad)), "NaN trim rejected");
  CHECK(torqueTrimValuesValid(-TORQUE_TRIM_MAX, TORQUE_TRIM_MAX), "trim boundary accepted");

  bad = rec;
  bad.magic ^= 1u;
  CHECK(!torqueTrimRecordValid(&bad, sizeof(bad)), "wrong magic rejected");

  /* A level-cal record must not validate as a trim record and vice versa. */
  union { imuLevelCalRecord_t level; torqueTrimRecord_t trim; unsigned char raw[64]; } u;
  memset(&u, 0, sizeof(u));
  imuLevelCalRecordInit(&u.level, 0.01f, 0.02f);
  CHECK(!torqueTrimRecordValid(&u.trim, sizeof(u.level)), "level record is not accepted as a trim record");
  memset(&u, 0, sizeof(u));
  torqueTrimRecordInit(&u.trim, 100.0f, 200.0f);
  CHECK(!imuLevelCalRecordValid(&u.level, sizeof(u.trim)), "trim record is not accepted as a level record");
}

static void testTorqueTrimScale(void)
{
  CHECK(torqueTrimScale(0.0f, 20000.0f, 10000.0f) == 0.0f, "no trim at zero thrust");
  CHECK(torqueTrimScale(20000.0f, 20000.0f, 10000.0f) == 0.0f, "no trim at the ground threshold");
  CHECK(fabsf(torqueTrimScale(25000.0f, 20000.0f, 10000.0f) - 0.5f) < 1e-6f, "half trim mid-ramp");
  CHECK(torqueTrimScale(30000.0f, 20000.0f, 10000.0f) == 1.0f, "full trim at end of ramp");
  CHECK(torqueTrimScale(60000.0f, 20000.0f, 10000.0f) == 1.0f, "full trim above ramp");
  float prev = -1.0f;
  for (float t = 0.0f; t <= 60000.0f; t += 250.0f) {
    float s = torqueTrimScale(t, 20000.0f, 10000.0f);
    CHECK(s >= prev && s >= 0.0f && s <= 1.0f, "scale monotonic and bounded at %g", t);
    prev = s;
  }
  CHECK(torqueTrimScale(15000.0f, 20000.0f, 0.0f) == 0.0f && torqueTrimScale(25000.0f, 20000.0f, 0.0f) == 1.0f,
        "zero ramp degenerates to a step");
}

static void testAttTrimRecord(void)
{
  attTrimRecord_t rec;
  attTrimRecordInit(&rec, 0.0f, 0.35f);
  CHECK(attTrimRecordValid(&rec, sizeof(rec)), "fresh attitude trim valid");
  attTrimRecord_t bad = rec;
  bad.pitchDeg = 0.36f;
  CHECK(!attTrimRecordValid(&bad, sizeof(bad)), "corrupted attitude trim rejected");
  attTrimRecordInit(&bad, 0.0f, ATT_TRIM_MAX_DEG + 0.1f);
  CHECK(!attTrimRecordValid(&bad, sizeof(bad)), "attitude trim beyond limit rejected");
  torqueTrimRecord_t tt;
  torqueTrimRecordInit(&tt, 0.0f, 0.35f);
  CHECK(!attTrimRecordValid((const attTrimRecord_t *)&tt, sizeof(tt)), "torque trim record not accepted as attitude trim");
}

int main(void)
{
  testCrc();
  testRecord();
  testLevelOffsetRemovesTilt();
  testAlignmentIdentity();
  testAlignmentIsRotation();
  testAlignmentSignConvention();
  testOldPitchFormulaRegression();
  testTorqueTrimRecord();
  testTorqueTrimScale();
  testAttTrimRecord();

  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
