/*
 * ESP-Drone Firmware
 *
 * cam_vz_shadow.h - The camera height observer (cam_vz_obs.h) in shadow mode:
 * run on every flight, manual ones included, and logged, so the camera's slow
 * speed offset c can be measured on today's drone before the autonomy's hover
 * wait relies on it (docs/autonomy-a2.md section 4.10)
 *
 * Nothing here commands the motors. The module owns one observer and gives it
 * what the camera height study gave it offline, from the drone's own signals:
 *
 * Sortie: from a ground start (thrust leaving zero after the soft-start's
 *   ground stop, i.e. >= 0.5 s at rest; stabilizer.c counts them) to the next
 *   one. Each ground start resets the observer (its 1.5 s camera-wake clock
 *   starts at that first step with thrust).
 *
 * Ground flag (cam_vz_obs.h: true before lift-off and after touchdown only):
 *   IDLE    on the ground, no sortie (motors off and resting, or since boot)
 *   GROUND  sortie open, motors running, not lifted off yet
 *   AIR     lifted off; the observer integrates h and v
 *   LANDED  the motors stopped after AIR, until the drone rests again (IDLE);
 *           thrust coming back before that (no ground start) lifts off again
 *           like GROUND (a hop: h and v restart from 0)
 *   Lift-off, the first of:
 *     - thrust >= CVS_LIFT_THRUST (30000, the firmware's airborne level: the
 *       camera hold, the trim learner, the flight summary). On a weak battery
 *       this fires while the drone still sits: harmless, on the ground with
 *       spinning motors a_z - g0 is -0.18 +/- 1.71 mg (study, 84 take-offs);
 *     - the accelerometer (a fresh battery lifts off below 30000), as
 *       vert_ctrl's lift-off detector: 20 Hz low-pass of a_z - g0 above
 *       25 mg for 30 ms, or a CUSUM of g (a_z - g0 - 7 mg) reaching 0.02 m/s
 *       (needs CVS_G0_MIN_SAMPLES of baseline);
 *     - an external hint (A2b: vert_ctrl's own lift-off).
 *   Touchdown: the motors stop (thrust 0) after AIR. The touchdown values
 *   (h, v, offset) are those just before the first impact (a_z > 1.5 g or
 *   max(|gx|, |gy|) > 200 dps, at least 0.2 s after lift-off: the study's
 *   contact rule) of the impact cluster that reaches into the last 0.5 s
 *   before the stop, or at the stop. A cluster runs on while its impacts come
 *   within 0.5 s of the previous one (2026-10-13 flight 1: five in-air
 *   contacts over 0.83 s before the motors stopped; the touchdown values came
 *   from the third, after two impulses had been integrated as a climb).
 *
 * Offset prior: at lift-off, g0 = median of the ground baseline ring (10 ms
 * means of the world-up specific force while resting with the motors off
 * before the ground start and on the ground with the motors running after it,
 * the last CVS_G0_RING of them, without the last CVS_G0_SKIP_LAST, which may
 * already be airborne); the prior is g0 - 1 + CAM_VZ_PRIOR_OFFSET_G, sigma
 * CAM_VZ_PRIOR_SIGMA_G, the pair the study learned (its g0: the median over
 * the ground phase after throttle-up; the motors-off rest level differs from
 * it by -1.5..+0.3 mg). Fewer than CVS_G0_PRIOR_MIN samples: the observer's
 * vague prior.
 *
 * Per-flight statistics (fsumCamVz_t, flight_summary.h) for the RTC flight
 * log, a 100 Hz record for the flight recorder (camVzShadowRec_t) and the
 * external-estimate fields of vert_ctrl's input (camVzShadowVzInput, the
 * mapping of docs/autonomy-a2.md section 4.10).
 *
 * In-flight gyro bias: the step's rateCorr (the attitude filter's correction,
 * minus the gyro bias once settled) is low-passed while the motors run and
 * the estimate (times the frame's dt) is taken off each frame's gyroAngle
 * before the observer removes the rotation (CVS_GYRO_BIAS_TAU_S).
 *
 * Use (stabilizer task): camVzShadowInit() once; camVzShadowStep() every
 * stabilizer loop, after the motors were set; camVzShadowFrame() for every new
 * camera sample, right after that loop's step (on the ground too: the
 * observer's missed-frame gate needs every seq). Pure code: no ESP-IDF
 * dependency, host-tested by tools/tests/test_cam_vz_shadow.c.
 */
#ifndef CAM_VZ_SHADOW_H_
#define CAM_VZ_SHADOW_H_

#include <stdbool.h>
#include <stdint.h>

#include "cam_vz_obs.h"
#include "flight_summary.h"
#include "vert_ctrl.h"

#define CVS_LIFT_THRUST         30000.0f  /* the firmware's airborne thrust (camera hold, trim learner, flight summary) */
#define CVS_G0_RING             128       /* ground baseline entries, one per CVS_G0_PERIOD_US: 1.28 s */
#define CVS_G0_PERIOD_US        10000u    /* each entry: the mean a_z of a 10 ms bucket (the study's logs: 100 Hz) */
#define CVS_G0_MIN_SAMPLES      20        /* the accelerometer lift-off rule needs this many */
#define CVS_G0_PRIOR_MIN        5         /* fewer: no g0, the observer keeps its vague prior */
#define CVS_G0_SKIP_LAST        5         /* left out of the median at lift-off (may already be airborne) */
#define CVS_G0_GAP_US           50000u    /* a longer gap between baseline samples restarts the ring */
#define CVS_LIFT_ACC_G          0.025f    /* vert_ctrl liftAcc */
#define CVS_LIFT_HOLD_US        30000u    /* vert_ctrl liftHoldS */
#define CVS_LIFT_CUSUM_V        0.02f     /* m/s, vert_ctrl liftCusumV */
#define CVS_LIFT_CUSUM_K_G      0.007f    /* vert_ctrl liftCusumK */
#define CVS_IMPACT_G            1.5f      /* world-up specific force of a touchdown impact (study: 1500 mg) */
#define CVS_IMPACT_DPS          200.0f    /* or max(|gx|, |gy|) (study: 200 dps) */
#define CVS_IMPACT_MIN_AIR_US   200000u   /* impacts this soon after lift-off do not count */
#define CVS_IMPACT_WINDOW_US    500000u   /* an impact cluster's gaps, and its last impact before the motors stop */
#define CVS_BIAS_AT1_US         3000000u  /* offset logged this long after the camera wake ... */
#define CVS_BIAS_AT2_US         5000000u  /* ... and this long */
#define CVS_DT_MAX_US           20000u    /* longer loop gaps count as this in the statistics and detectors */
#define CVS_OBS_BLOCK_US        10000u    /* the filter runs on 10 ms means of the specific force (its prediction
                                           * already works in 10 ms blocks; per-step calls cost ~70 us on the
                                           * drone, about 7 % of the 1 kHz loop) */
#define CVS_OBS_BLOCK_TOL_US    500u      /* a block this much shorter still steps the filter. cam_vz_glue.c's
                                           * blocks are 10 IMU periods: 9990-9999 us when the MPU's clock is a
                                           * hair fast, plus timestamp jitter. Without the tolerance every other
                                           * block fell short and was merged into the next (the filter ran at
                                           * ~50 Hz, and sigAcc is a per-10-ms-block noise: twice the speed
                                           * process noise per second of the tuned filter). A 1 kHz caller still
                                           * gets 10-sample blocks (9 samples < 9500 us within +-5 %). */
/* In-flight gyro bias (docs/autonomy-a2.md 4.10): the MPU6050's zero moves in
 * flight by up to ~0.8 deg/s (pitch, 2026-10-11 flights 2-4; ~0 on files
 * 16-21) after the ground re-zero, and the camera's rotation removal turns a
 * pitch-gyro bias b into a speed offset kyGy * b / G (~0.03 m/s at 0.77 deg/s,
 * G 46 px/m). The attitude estimator's correction (sensfusion6: Mahony P + I
 * feedback, time constant 2.5 s) settles at minus the bias; its low-pass since
 * the ground start is subtracted from each frame's gyro angles. */
#define CVS_GYRO_BIAS_TAU_S     2.0f      /* s, the low-pass */
#define CVS_GYRO_BIAS_MAX_RADS  0.035f    /* |estimate| clamp, rad/s (2 deg/s; the logged in-flight biases <= 0.9) */

typedef enum {
  CVS_IDLE = 0,
  CVS_GROUND = 1,
  CVS_AIR = 2,
  CVS_LANDED = 3,
} camVzShadowPhase_t;

/* How the latest lift-off was detected (fsumCamVz_t flags). */
typedef enum {
  CVS_LIFT_NONE = 0,
  CVS_LIFT_BY_THRUST = 1,
  CVS_LIFT_BY_ACC = 2,
  CVS_LIFT_BY_HINT = 3,
} camVzShadowLift_t;

/* One stabilizer step. */
typedef struct {
  uint64_t timeUs;        /* IMU sample time (esp_timer us, the camera's clock) */
  float accUpG;           /* world-up specific force, g: acc . gravity direction (sensfusion6), ~1.0 at rest;
                           * fed at 100 Hz as 10 ms means (cam_vz_glue.c) */
  float accMaxG;          /* the highest sample behind accUpG (impact rule); 0: use accUpG */
  float thrust;           /* thrust the motors got this step: 0 when they are stopped */
  float gyroMaxDps;       /* max(|gx|, |gy|) */
  bool groundStart;       /* this step left a ground stop (stabilizer.c's take-off count advanced) */
  bool restingOnGround;   /* zero thrust and resting since the soft-start's ground stop (stabilizerOnGround()) */
  bool still;             /* gyro still and about 1 g this step (the soft-start's resting test) */
  bool liftoffHint;       /* optional: an external lift-off decision (A2b: vzOutput_t.liftedOff); false without one */
  float rateCorr[3];      /* rad/s, body x/y/z: the rate the attitude estimator adds to the gyro (sensfusion6's
                           * Mahony feedback), mean over the step; minus the gyro bias in steady flight. All 0
                           * (zero-initialised callers): no gyro-bias correction */
} camVzShadowIn_t;

/* 100 Hz flight-recorder fields (flight_recorder.h cv*). */
typedef struct {
  int16_t hMm;            /* height estimate, mm (0 on the ground) */
  int16_t vMms;           /* vertical speed estimate, mm/s, up */
  int16_t biasCmg;        /* offset vs 1 g, 0.01 mg */
  uint16_t gainDpm;       /* gain the next update would use, 0.1 px/m (0: none) */
  uint8_t flags;          /* CVS_REC_* | (latest new frame's camVzObsDecision_t << 4) */
  uint8_t sigVMms;        /* sigma of v, mm/s, capped at 255 */
} camVzShadowRec_t;

#define CVS_REC_AIRBORNE  0x01   /* the observer's ground flag is off (AIR) */
#define CVS_REC_HEALTHY   0x02
#define CVS_REC_GAIN      0x04   /* the gain tracker has a G */
#define CVS_REC_FAULT     0x08

typedef struct {
  camVzObs_t obs;
  camVzObsOut_t out;      /* after the latest step or frame */
  camVzShadowPhase_t phase;
  uint32_t sortie;        /* ground starts since init */
  uint64_t sortieUs;      /* time of the latest ground start */
  /* ground baseline: (a_z - 1 g) in 0.1 mg, ring of 10 ms means */
  int16_t g0Ring[CVS_G0_RING];
  int16_t g0Scratch[CVS_G0_RING];
  uint8_t g0Head, g0N;
  int32_t g0Sum;
  bool g0Has;             /* a sample since the last restart (g0LastUs valid) */
  uint64_t g0LastUs;      /* latest sample */
  uint64_t g0Bucket;      /* time bucket being averaged */
  float g0AccSum;
  uint16_t g0AccN;
  float g0;               /* g, the baseline the prior used (1 if none) */
  /* lift-off detector */
  float lp20, cusum;
  uint32_t liftHighUs;
  camVzShadowLift_t liftBy;
  /* clock */
  bool hasT;
  uint64_t lastUs;
  /* sortie times, us since the ground start */
  bool lifted;            /* lifted off at least once in this sortie */
  uint32_t tLift, tWake;  /* first lift-off; first frame the observer used (the camera wake) */
  bool hasWake;
  /* touchdown */
  float prevH, prevV, prevBiasG;
  bool hasImpact;
  uint32_t tImpact;       /* the latest impact of the current cluster (impH/V/BiasG: before its first) */
  float impH, impV, impBiasG;
  bool touchdown, tdImpact;
  float tdH, tdV, tdBiasG;
  /* statistics (us since the ground start, durations in us) */
  uint32_t airUs, camAirUs, okUs, gapMaxUs, firstOkUs;
  bool hasFirstOk;
  bool hasBias1, hasBias2;
  float bias1G, bias2G, sigBias2G;
  float hMax;
  float gainLast;
  bool gainEver, fault, priorSet;
  camVzObsDecision_t lastDecision;   /* latest new frame (not DUPLICATE) */
  /* the current 10 ms block of the specific force for the filter */
  float blkSum;
  uint32_t blkN;
  uint64_t blkT0;
  bool blkHasT;
  /* gyro bias, rad/s (body x, y, z): low-pass (CVS_GYRO_BIAS_TAU_S) of -rateCorr while the motors run,
   * from the first correction after each ground start (the estimator has settled on the ground);
   * subtracted (x dt) from every frame's gyroAngle */
  float gyroBias[3];
  bool gyroBiasSeeded;    /* gyroBias holds this sortie's first correction */
  bool lateCfgBad;        /* the caller's: the stored late offset noise setting was refused at boot (cam_vz_glue.c;
                           * FSUM_CVL_CFG_BAD); kept through every sortie */
  bool t2CfgBad;          /* the caller's: the stored offset drift rate setting (cvt2k / cvt2r) was refused at boot, or
                           * came with the late offset noise on (FSUM_CVL_T2_CFG_BAD; in padding) */
  bool hbCfgBad;          /* the caller's: the stored healthy offset speed-up setting (cvhb) was refused at boot, or came
                           * with R1 or T2 on (FSUM_CVL_HB_CFG_BAD; in padding) */
} camVzShadow_t;

/* Initialises the observer with cfg (NULL: the FINAL default); false (and the
 * default config) if cfg is invalid. Phase IDLE. */
bool camVzShadowInit(camVzShadow_t *s, const camVzObsConfig_t *cfg);
void camVzShadowStep(camVzShadow_t *s, const camVzShadowIn_t *in);
/* A camera sample; nowUs for its age (cam_vz_obs.h). */
camVzObsDecision_t camVzShadowFrame(camVzShadow_t *s, const camVzObsFrame_t *f, uint64_t nowUs);

/* The external-estimate fields of vert_ctrl's input (docs/autonomy-a2.md
 * section 4.10): camHealthy = healthy && airborne && !fault, camVzValid =
 * !fault, camVz = v, camBias = biasHeldG + 1 - vzG0 (vzG0 = vzOutput_t.g0: the
 * controller's offset is against its own ground baseline; biasHeldG is biasG
 * unless the late offset noise is past its step, cam_vz_obs.h). Every field is
 * finite; anything non-finite gives the "unusable" values (false, 0). Only
 * these four fields are written. */
void camVzShadowVzInput(const camVzShadow_t *s, float vzG0, vzInput_t *in);

void camVzShadowRecord(const camVzShadow_t *s, camVzShadowRec_t *r);
/* The flight-summary fields of the current sortie (all 0, flags 0, before the
 * first ground start). */
void camVzShadowFlightStats(const camVzShadow_t *s, fsumCamVz_t *out);
/* The late offset noise's, the offset drift rate's and the healthy offset
 * speed-up's flight-summary fields of the current sortie (all 0 before the
 * first ground start; FSUM_CVL_CFG_BAD from lateCfgBad, FSUM_CVL_T2_CFG_BAD
 * from t2CfgBad, FSUM_CVL_HB_CFG_BAD from hbCfgBad). */
void camVzShadowLateStats(const camVzShadow_t *s, fsumCvLate_t *out);

const char *camVzShadowPhaseName(camVzShadowPhase_t p);

/* Bench load test (the RAM-only USB knob 'camvz air'): makes a drone resting
 * on the bench look airborne to the shadow, so that every block and every
 * camera frame takes the in-flight path and the stabilizer's loop rate can be
 * measured under the flight load without flying. Only the inputs handed to
 * camVzShadowStep() / camVzShadowFrame() change; nothing reaches the motors.
 * The caller ends it at once on any real thrust (cam_vz_glue.c). */
typedef enum {
  CVS_BENCH_OFF = 0,
  CVS_BENCH_AIR = 1,      /* airborne inputs; the real frames go through the usual gates */
  CVS_BENCH_FULL = 2,     /* and every frame passes the quality gate (the full update path) */
  CVS_BENCH_SPIN = 3,     /* FULL that stays on with the motors spinning (props off, under a gcap bench
                           * cap): the stabilizer's own motor path runs as in flight */
} camVzShadowBenchMode_t;

typedef struct {
  camVzShadowBenchMode_t mode;
  bool started;           /* the bench sortie's ground start was given */
} camVzShadowBench_t;

/* Rewrites one block's inputs: a ground start on the first block after the
 * mode was set, then thrust CVS_LIFT_THRUST (lift-off by the thrust rule on
 * that same block), not resting, not still. With the mode off it only clears
 * 'started' and leaves the inputs alone. */
void camVzShadowBenchIn(camVzShadowBench_t *b, camVzShadowIn_t *in);
/* CVS_BENCH_FULL and CVS_BENCH_SPIN: the frame passes the quality gate (ok,
 * texture >= 20, residual <= 1, no yaw, finite motion); otherwise unchanged. */
void camVzShadowBenchFrame(const camVzShadowBench_t *b, camVzObsFrame_t *f);

#endif /* CAM_VZ_SHADOW_H_ */
