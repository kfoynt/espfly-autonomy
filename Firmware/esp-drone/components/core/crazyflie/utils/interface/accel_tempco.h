/*
 * ESP-Drone Firmware
 *
 * accel_tempco.h - Accelerometer zero temperature compensation (pure helpers)
 *
 * The MPU-6050's accelerometer zero moves with its die temperature too. The
 * drone has no height sensor: the vertical controller (vert_ctrl.h) and the
 * camera height observer (cam_vz_obs.h) integrate the body z accelerometer
 * against a ground baseline g0 taken in the idle spin before every take-off,
 * so a zero that walks after g0 is a false vertical acceleration (0.2 mg for
 * 30 s is 1 m of height). The IMU sits at 47-53 C on the ground (board heat,
 * worst right off the USB charger) and cools to 31-36 C in flight under the
 * prop wash (docs/autonomy-a2b.md section 36).
 *
 * The model is the gyro's (gyro_tempco.h), in mg:
 *   corr[i] = clamp(k[i] * (min(T, knee) - min(Tref, knee)), +/-clampMg)
 * with T the die temperature low-passed with ACC_TC_TAU_S and Tref latched
 * where the accelerometer's reference is taken: at the start of every sortie
 * (ARM, on the ground before the idle spin's g0), at the start of a ground
 * cooling spin (ground_cool.h) and at a bench 'atc latch'. The driver
 * subtracts corr from the body-frame accelerometer (g, before the alignment
 * and the low-pass) so every reader sees it. The sign: b_z = k_z (T - Tref)
 * is what a still accelerometer reads above its reference after the change,
 * and the corrected reading is b - corr = 0; k_z > 0 means the zero drops as
 * the IMU cools, which the controller reads as sinking and climbs against.
 *
 * The fit (section 36.1) could not pin k_z: the observer's offset state gave
 * -0.7..+0.7 mg/C across 33 flights (median +0.20) and the still ground rows
 * before and after four clean flights gave +0.07 +/- 0.45 mg/C over 12-17 C
 * of cooling. The compensation therefore ships OFF with a small positive
 * default the user can enable ('atc on'); the post-flight check and the
 * ground cooling spin measure k on the drone.
 *
 * No ESP-IDF dependencies; unit tested by tools/tests/test_accel_tempco.c.
 */
#ifndef ACCEL_TEMPCO_H_
#define ACCEL_TEMPCO_H_

#include <stdbool.h>
#include <stdint.h>

/* Compiled defaults (2026-10-18 fit, section 36.1): z the observer's median, x / y unknown (0); no knee (the
 * ground rows at 45-48 C showed a slope too), OFF until the drone's own check confirms the sign. */
#define ACC_TC_KX_DEFAULT      0.00f    /* mg per C, body x */
#define ACC_TC_KY_DEFAULT      0.00f    /* body y */
#define ACC_TC_KZ_DEFAULT      0.20f    /* body z */
#define ACC_TC_KNEE_DEFAULT    60.0f    /* C: no slope above (60 = none in the IMU's range) */
#define ACC_TC_CLAMP_DEFAULT   15.0f    /* mg per axis (the +-20 mg take-off envelope of docs/autonomy-a2.md 4.9) */
#define ACC_TC_ON_DEFAULT      false
#define ACC_TC_K_MAX           2.0f     /* |k| accepted by 'atc set' (the datasheet's worst case is ~0.9 mg/C on z) */
#define ACC_TC_KNEE_MIN        30.0f
#define ACC_TC_KNEE_MAX        60.0f
#define ACC_TC_TAU_S           0.5f     /* temperature low-pass */

typedef struct {
  float k[3];       /* mg per C, body x / y / z */
  float kneeC;
  float clampMg;
  bool on;
} accTcConfig_t;

typedef struct {
  float tLp;        /* low-passed die temperature, C */
  float tRef;       /* tLp at the latest latch */
  bool seeded;      /* tLp holds a sample */
  bool refSet;      /* a latch happened */
} accTcState_t;

void accTcDefaultConfig(accTcConfig_t *c);
/* Finite, |k| <= ACC_TC_K_MAX, knee ACC_TC_KNEE_MIN..MAX, clamp 0..50 mg. */
bool accTcConfigValid(const accTcConfig_t *c);

void accTcInit(accTcState_t *st);
/* One temperature sample: first-order low-pass (tau ACC_TC_TAU_S), seeded by the first finite sample; a NaN or
 * infinite sample is skipped. */
void accTcStep(accTcState_t *st, float tempC, float dt);
/* The reference was taken: Tref is the temperature now. No-op unseeded. */
void accTcLatch(accTcState_t *st);
/* The temperature change the model uses, min(tNow, knee) - min(tRef, knee) (negative when cooling). */
float accTcEffDelta(const accTcConfig_t *c, float tNow, float tRef);
/* The model's correction (mg), whatever c->on says; 0 before a latch or with a non-finite temperature. */
void accTcModel(const accTcConfig_t *c, const accTcState_t *st, float out[3]);
/* What the driver subtracts (mg): the model when on, else exactly 0. */
void accTcCorr(const accTcConfig_t *c, const accTcState_t *st, float out[3]);

/* The driver's block rate is the gyro compensation's (GYRO_TC_BLOCK samples, 20 Hz, docs/autonomy-a2b.md
 * 19.11): one accTcBlock on the block's mean temperature, the cached correction subtracted from every sample
 * between blocks. The accelerometer's reference and the still readings for the check (below) are means of the
 * raw samples over the same blocks, kept as a ring of ACC_TC_WIN_BLOCKS block means (1 s). */
#define ACC_TC_BLOCK      50
#define ACC_TC_WIN_BLOCKS 20

/* One block of n >= 1 raw temperature samples (their sum; MPU-6050: raw / 340 + 36.53): accTcStep on their mean
 * with dt = n ms, then accTcCorr into corrOut. */
void accTcBlock(accTcState_t *st, const accTcConfig_t *c, int32_t rawSum, int n, float corrOut[3]);

typedef struct {
  float mg[ACC_TC_WIN_BLOCKS][3];   /* block means, mg, body frame, before any compensation */
  uint8_t n;                        /* blocks held (<= ACC_TC_WIN_BLOCKS) */
  uint8_t next;                     /* the slot the next block takes */
} accTcWindow_t;

void accTcWindowInit(accTcWindow_t *w);
/* One block: the sums of its n >= 1 raw samples per axis (mapped body frame) and the raw LSB to mg scale. */
void accTcWindowPush(accTcWindow_t *w, const int32_t rawSum[3], int n, float mgPerLsb);
/* The mean of the blocks held (mg); false (out 0) until the ring is full. */
bool accTcWindowMean(const accTcWindow_t *w, float out[3]);

/* The check (docs/autonomy-a2b.md 36.2): at a latch the window's mean is the reference reading (motors off,
 * resting, right before the sortie or the cooling spin); at the first accepted gyro re-zero after thrust (the
 * drone rests again, >= 2 s after the motors stopped) the window's mean minus the reference is the step a still
 * accelerometer took over that stretch, against the model's correction at that moment: r = step - corr is what
 * the compensation left (a sortie's g0 is taken in the idle spin, so r is also the offset the controller flew
 * with, up to the spin's own vibration offset). A change of the resting attitude moves x / y by g x the angle
 * (17 mg per deg) and z by little (1 - cos): the reader flags a large x / y step. */
#define ACC_TC_RZ_FLIGHT 0x01   /* thrust > 0 since the latch (a flight, or a ground spin) */
#define ACC_TC_RZ_ON     0x02   /* the compensation was on */
#define ACC_TC_RZ_AIR    0x04   /* thrust above lift-off since the latch: a flight */
#define ACC_TC_RZ_COOL   0x08   /* a ground cooling spin started since the latch (ground_cool.h) */
#define ACC_TC_LOG_RING  4

typedef struct {
  int16_t tRefCc, tNowCc;      /* the latch's and the current temperature, 0.01 C */
  int16_t stepCmg[3];          /* still reading now minus the reference, 0.01 mg (saturated) */
  int16_t corrCmg[3];          /* the model's correction (also when off), 0.01 mg */
  uint8_t flags;               /* ACC_TC_RZ_* */
} accTcRezero_t;

typedef struct {
  accTcRezero_t ring[ACC_TC_LOG_RING];
  uint32_t n;                  /* entries logged since boot (the ring holds the last ACC_TC_LOG_RING) */
  accTcRezero_t air;           /* the latest entry after an airborne stretch (valid when airSeq > 0) */
  uint32_t airSeq;             /* such entries since boot */
  uint32_t airAtMs;            /* when it was taken (the caller's ms clock) */
  uint32_t airLandMs;          /* from the stretch's last airborne sample to it */
} accTcLog_t;

/* One entry from the driver's values (mg, C), each saturated to int16 (a NaN reads 0). */
void accTcRezeroFill(accTcRezero_t *e, float tRef, float tNow, const float stepMg[3], const float corrMg[3],
                     uint8_t flags);
/* r = step - corr on one axis, mg. */
float accTcRezeroResidual(const accTcRezero_t *e, int axis);
void accTcLogInit(accTcLog_t *log);
/* One entry at nowMs (airLastMs read only with ACC_TC_RZ_AIR). Only entries after thrust or a cooling spin are
 * kept (a quiet re-zero once a second says nothing new); returns whether the ring took it. */
bool accTcLogRezero(accTcLog_t *log, const accTcRezero_t *e, uint32_t nowMs, uint32_t airLastMs);
uint32_t accTcLogCount(const accTcLog_t *log);
const accTcRezero_t *accTcLogEntry(const accTcLog_t *log, uint32_t i);   /* oldest first */

/* NVS record "acctc" (namespace imucal), the gyro's contract (gyro_tempco.h): version, size, magic, crc32 over
 * the preceding bytes. The clamp is not stored (always ACC_TC_CLAMP_DEFAULT). */
#define ACC_TC_VERSION 1
#define ACC_TC_MAGIC   0x43544341u   /* "ACTC" */

typedef struct {
  uint16_t version;  /* ACC_TC_VERSION */
  uint16_t size;     /* sizeof(accTcRecord_t) */
  uint32_t magic;    /* ACC_TC_MAGIC */
  float k[3];
  float kneeC;
  uint32_t on;       /* 0 or 1 */
  uint32_t crc;      /* crc32 over all preceding bytes */
} accTcRecord_t;

void accTcRecordInit(accTcRecord_t *rec, const accTcConfig_t *c);
bool accTcRecordValid(const accTcRecord_t *rec, uint32_t storedSize);
/* A valid record's values over the defaults (the clamp stays the default). */
void accTcRecordApply(const accTcRecord_t *rec, accTcConfig_t *c);

#endif /* ACCEL_TEMPCO_H_ */
