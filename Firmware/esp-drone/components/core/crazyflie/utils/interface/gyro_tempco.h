/*
 * ESP-Drone Firmware
 *
 * gyro_tempco.h - Gyro zero-rate temperature compensation (pure helpers)
 *
 * The MPU-6050's gyro zero moves with its die temperature. The driver
 * measures the zero on the ground (the startup bias, then a re-zero every
 * second while the drone rests with the motors off), but in flight the IMU
 * cools under the prop wash and the zero walks away from the last ground
 * value. On 2026-10-14 (flight 4, 48.5 s air) the IMU cooled 43 -> 33 C and
 * the pitch gyro read +0.65 dps still after landing (+0.060 dps per C of
 * cooling; roll -0.018 dps per C); the attitude filter (sensfusion6.c, Kp
 * 0.4 /s) turned that into 1.5 deg of nose-down estimate error, the
 * controller held the estimate level and the drone flew backward
 * (docs/autonomy-a2b.md section 19.1).
 *
 * The model is linear in the temperature change since the latest bias latch
 * (the ground measurement the driver is using), flat above a knee (the zero
 * moved 0.02 dps between 43 and 47 C):
 *   corr[i] = clamp(k[i] * (min(T, knee) - min(Tref, knee)), +/-clampDps)
 * with T the die temperature low-passed with GYRO_TC_TAU_S (the bias the
 * driver measures is a 1 s mean). The driver subtracts corr from the gyro
 * (deg/s, body frame: the recorder's gx/gy/gz) before its low-pass filter.
 * The sign: b_y = -0.060 (T - Tref) dps is what the pitch gyro reads after
 * cooling (T < Tref: positive), so k_y = -0.060 and the corrected gyro reads
 * b - corr = 0.
 *
 * No ESP-IDF dependencies; unit tested by tools/tests/test_gyro_tempco.c.
 */
#ifndef GYRO_TEMPCO_H_
#define GYRO_TEMPCO_H_

#include <stdbool.h>
#include <stdint.h>

/* Compiled defaults (2026-10-14 forensic): pitch from the ground re-zeros at 39.5 / 41 / 43 C (0.057) and flight 4
 * (0.065, +0.65 dps after 43 -> 33 C); roll from flight 4's -0.15..-0.18 dps after 10 C; yaw unknown (0). */
#define GYRO_TC_KX_DEFAULT     0.018f   /* dps per C, body x (roll gyro) */
#define GYRO_TC_KY_DEFAULT    -0.060f   /* body y (pitch gyro) */
#define GYRO_TC_KZ_DEFAULT     0.000f   /* body z (yaw gyro) */
#define GYRO_TC_KNEE_DEFAULT   44.0f    /* C: no slope above */
#define GYRO_TC_CLAMP_DEFAULT  1.00f    /* dps per axis (2.5 deg of estimate error at Kp 0.4) */
#define GYRO_TC_K_MAX          0.15f    /* |k| accepted by 'gtc set' */
#define GYRO_TC_KNEE_MIN       30.0f
#define GYRO_TC_KNEE_MAX       60.0f
#define GYRO_TC_TAU_S          0.5f     /* temperature low-pass */

typedef struct {
  float k[3];       /* dps per C, body x / y / z */
  float kneeC;
  float clampDps;
  bool on;
} gyroTcConfig_t;

typedef struct {
  float tLp;        /* low-passed die temperature, C */
  float tRef;       /* tLp at the latest bias latch */
  bool seeded;      /* tLp holds a sample */
  bool refSet;      /* a latch happened */
} gyroTcState_t;

void gyroTcDefaultConfig(gyroTcConfig_t *c);
/* Finite, |k| <= GYRO_TC_K_MAX, knee GYRO_TC_KNEE_MIN..MAX, clamp 0..2 dps. */
bool gyroTcConfigValid(const gyroTcConfig_t *c);

void gyroTcInit(gyroTcState_t *st);
/* One temperature sample: first-order low-pass (tau GYRO_TC_TAU_S), seeded by the first finite sample; a NaN or
 * infinite sample is skipped. */
void gyroTcStep(gyroTcState_t *st, float tempC, float dt);
/* The driver's bias changed (startup, ground re-zero): the reference is the temperature now. No-op unseeded. */
void gyroTcLatch(gyroTcState_t *st);
/* The temperature change the model uses, min(tNow, knee) - min(tRef, knee) (negative when cooling). */
float gyroTcEffDelta(const gyroTcConfig_t *c, float tNow, float tRef);
/* The model's correction (dps), whatever c->on says; 0 before a latch or with a non-finite temperature. */
void gyroTcModel(const gyroTcConfig_t *c, const gyroTcState_t *st, float out[3]);
/* What the driver subtracts: the model when on, else 0. */
void gyroTcCorr(const gyroTcConfig_t *c, const gyroTcState_t *st, float out[3]);

/* The driver's update rate (docs/autonomy-a2b.md 19.11). The die temperature is a slow signal (1/340 C per LSB;
 * flight 4 cooled at most ~0.6 C/s, the bench test 0.04 C/s; low-passed at tau 0.5 s), so the driver adds the raw
 * TEMP_OUT of GYRO_TC_BLOCK 1 kHz samples and runs gyroTcBlock once per block (20 Hz) on their mean with dt =
 * GYRO_TC_BLOCK ms; between blocks it subtracts the cached correction from every sample (and recomputes it at a
 * latch). Against the per-sample update of b40a827 the correction differs by <= 2.2 mdps on flight 4's cooling with
 * re-zeros, 0.2 on the bench cooling test, 5.6 on an instant 1 C step, about 0.044 mdps per sample of block
 * (test_gyro_tempco.c; a gyro LSB is 61 mdps, the correction up to 600, its +-20 % coefficient error 120). */
#define GYRO_TC_BLOCK 50
/* TEMP_OUT to C (MPU-6050: raw / 340 + 36.53) for the mean of n raw samples given their sum; n = 1 is the
 * per-sample decode bit for bit. */
float gyroTcTempFromRawSum(int32_t rawSum, int n);
/* One block of n >= 1 raw samples (their sum): gyroTcStep on their mean with dt = n ms, then gyroTcCorr into
 * corrOut (what the driver subtracts until the next block). */
void gyroTcBlock(gyroTcState_t *st, const gyroTcConfig_t *c, int32_t rawSum, int n, float corrOut[3]);

/* The re-zero log: what the compensation left, read after a flight (docs/autonomy-a2b.md 19.1).
 *
 * Each accepted ground re-zero's bias step (what a still gyro read with the old bias, body frame) against the
 * model's correction at that moment: r = step - corr is the still-gyro error the compensation left since the
 * previous latch. The driver re-zeros about once a second while the drone rests, so the first re-zero after a
 * flight would leave an 8-entry ring of every re-zero ~10 s after the motors stop, before USB is plugged in. The
 * ring therefore takes only the re-zeros that carry news: after thrust (a flight or a ground spin), after a 'gtc
 * hold' (the bench cooling test), or with a step of GYRO_TC_LOG_STEP_MDPS or more on an axis. And the first re-zero
 * after an airborne stretch (thrust above lift-off since the previous latch) is also kept on its own ('air') until
 * the next airborne one, whatever comes between. */
#define GYRO_TC_RZ_FLIGHT 0x01   /* thrust > 0 since the previous latch (a flight, or a ground spin) */
#define GYRO_TC_RZ_ON     0x02   /* the compensation was on */
#define GYRO_TC_RZ_AIR    0x04   /* thrust above lift-off (> 30000) since the previous latch: a flight */
#define GYRO_TC_RZ_HOLD   0x08   /* a 'gtc hold' started since the previous latch (the bench cooling test) */
#define GYRO_TC_RZ_COOL   0x10   /* a ground cooling spin started since the previous latch (ground_cool.h) */
#define GYRO_TC_LOG_RING       8
#define GYRO_TC_LOG_STEP_MDPS 50   /* a quiet re-zero (no thrust, no hold) is logged only with a step this large */

typedef struct {
  int16_t tRefOldCc, tNowCc;   /* the previous latch's and the current temperature, 0.01 C */
  int16_t stepMdps[3];         /* mdps */
  int16_t corrMdps[3];         /* the model's correction (also when off), mdps */
  uint8_t flags;               /* GYRO_TC_RZ_* */
} gyroTcRezero_t;

typedef struct {
  gyroTcRezero_t ring[GYRO_TC_LOG_RING];
  uint32_t n;              /* re-zeros logged since boot (the ring holds the last GYRO_TC_LOG_RING) */
  gyroTcRezero_t air;      /* the latest first re-zero after an airborne stretch (valid when airSeq > 0) */
  uint32_t airSeq;         /* such re-zeros since boot */
  uint32_t airAtMs;        /* when it was taken (the caller's ms clock) */
  uint32_t airLandMs;      /* from the stretch's last airborne sample to it */
  uint32_t airSpanMs;      /* the stretch's first to last airborne sample */
} gyroTcLog_t;

/* One re-zero from the driver's values (dps, C), each saturated to int16 (a NaN reads 0). */
void gyroTcRezeroFill(gyroTcRezero_t *e, float tRef, float tNow, const float stepDps[3], const float corrDps[3],
                      uint8_t flags);
/* r = step - corr on one axis, dps. */
float gyroTcRezeroResidual(const gyroTcRezero_t *e, int axis);
void gyroTcLogInit(gyroTcLog_t *log);
/* One accepted re-zero at nowMs (the airborne stretch's first / last sample times are read only with
 * GYRO_TC_RZ_AIR). Returns whether the ring took it. */
bool gyroTcLogRezero(gyroTcLog_t *log, const gyroTcRezero_t *e, uint32_t nowMs, uint32_t airFirstMs,
                     uint32_t airLastMs);
/* How many the ring holds (min(n, GYRO_TC_LOG_RING)), and the i-th of them, oldest first. */
uint32_t gyroTcLogCount(const gyroTcLog_t *log);
const gyroTcRezero_t *gyroTcLogEntry(const gyroTcLog_t *log, uint32_t i);

/* NVS record "gyrotc" (namespace imucal), same contract as the attitude trim (torque_trim.h): version, size,
 * magic, crc32 over the preceding bytes. The clamp is not stored (always GYRO_TC_CLAMP_DEFAULT). */
#define GYRO_TC_VERSION 1
#define GYRO_TC_MAGIC   0x43544347u   /* "GCTC" */

typedef struct {
  uint16_t version;  /* GYRO_TC_VERSION */
  uint16_t size;     /* sizeof(gyroTcRecord_t) */
  uint32_t magic;    /* GYRO_TC_MAGIC */
  float k[3];
  float kneeC;
  uint32_t on;       /* 0 or 1 */
  uint32_t crc;      /* crc32 over all preceding bytes */
} gyroTcRecord_t;

void gyroTcRecordInit(gyroTcRecord_t *rec, const gyroTcConfig_t *c);
bool gyroTcRecordValid(const gyroTcRecord_t *rec, uint32_t storedSize);
/* A valid record's values over the defaults (the clamp stays the default). */
void gyroTcRecordApply(const gyroTcRecord_t *rec, gyroTcConfig_t *c);

#endif /* GYRO_TEMPCO_H_ */
