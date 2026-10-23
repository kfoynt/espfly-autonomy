/*
 * ESP-Drone Firmware
 *
 * cam_vz_glue.h - Runs the camera height observer in shadow mode on every
 * flight (cam_vz_shadow.h, docs/autonomy-a2.md section 4.10)
 *
 * The stabilizer task feeds it every loop (the world-up specific force as
 * sensfusion6 computes it, the thrust the motors got, the ground start and
 * the resting test; the step also reads sensfusion6's rate correction for
 * the in-flight gyro bias), but only folds each sample into a 10 ms block: run in
 * the 1 kHz loop the observer cost ~40-70 us a loop on the drone and the loop
 * fell to ~860-910 Hz. Finished blocks go through a single-producer ring to a
 * low-priority task ("camvz", core 0, stack in PSRAM) that runs the observer at
 * 100 Hz (the rate it was validated at on the logged flights), takes the
 * newest camera sample after each block, and publishes a snapshot; every
 * getter below copies from that snapshot (up to ~20 ms old) under a short
 * lock. The measured cost is shown by 'camvz prof'.
 *
 * Nothing here commands the motors. The observer state (~2.8 KB) and the
 * task's stack live in PSRAM when there is some, else in internal RAM;
 * without memory the observer is off and every getter reports "no observer".
 */
#ifndef CAM_VZ_GLUE_H_
#define CAM_VZ_GLUE_H_

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"
#include "cam_vz_shadow.h"

/* Once, from stabilizerInit(), before the stabilizer task starts. The
 * observer's gyro window offset is the camera's (cameraFlowGyroOffsetUs(),
 * docs/autonomy-a2b.md 17.2); cam_vz_obs.c's default 15000 stays for the
 * replays of the logs flown at 20 MHz. */
void camVzGlueInit(void);
/* The camera's gyro window offset changed (camera_flow.c: the clock read at
 * boot, the fallback to 20 MHz on the ground): from the next frame on. One
 * aligned word the observer reads per frame; ignored without the observer or
 * above 100000 us. */
void camVzGlueSetGyroOffsetUs(uint32_t us);

/* Every stabilizer loop, after the motors were set. thrustApplied: the thrust
 * setpoint if any motor duty is nonzero, else 0; groundStart: this loop left
 * a ground stop (the take-off count advanced); still: the soft-start's
 * resting test (gyro still, about 1 g). */
void camVzGlueStep(const sensorData_t *sensors, float thrustApplied, bool groundStart, bool still);

/* Stabilizer task only. */
/* A2b: vert_ctrl's lift-off (vzOutput_t.liftedOff) as an extra lift-off rule. */
void camVzGlueSetLiftoffHint(bool liftedOff);

/* Bench diagnostics (RAM only): hand no blocks to the observer; wake its task
 * every 'blocks' blocks (1-8, default 1) instead of every block. While paused
 * the published snapshot freezes, so camVzGlueVzInput() reports the estimate
 * unusable, and the autonomy glue refuses an ARM whose controller would read
 * it (FLIGHT, BENCH on the IMU; docs/autonomy-a2b.md section 7.3). */
void camVzGlueSetPaused(bool paused);
bool camVzGluePaused(void);
void camVzGlueSetBatch(uint32_t blocks);
/* Bench load test (RAM only, camVzShadowBenchMode_t): the shadow treats the
 * resting drone as airborne (and with CVS_BENCH_FULL passes every frame's
 * quality gate) so the in-flight work runs on the bench. It ends by itself
 * at the first block with real thrust or a real ground start (CVS_BENCH_SPIN:
 * not, for a props-off spin under a gcap cap); while it is on,
 * camVzGlueVzInput() reports the estimate unusable. */
void camVzGlueSetBenchAir(uint8_t mode);
uint8_t camVzGlueBenchAir(void);
/* Bench profile: max and average x10 (us) of the stabilizer's per-sample work,
 * the task's block step, its frame update and its publish. */
void camVzGlueProfile(uint32_t maxUs[4], uint32_t avgUs10[4]);
/* Restarts that profile (bench: one condition at a time; a sample taken
 * while it clears may be lost). */
void camVzGlueProfileReset(void);
/* Blocks dropped because the task fell 16 behind (should stay 0). */
uint32_t camVzGlueRingDrops(void);
/* The four external-estimate fields of vert_ctrl's input (camHealthy,
 * camVzValid, camVz, camBias) for vzG0 = vzOutput_t.g0; without the observer
 * they get the unusable values (false, 0) and false is returned. The values
 * are also unusable (but true is returned) while paused or in a bench mode. */
bool camVzGlueVzInput(float vzG0, vzInput_t *in);
/* The 100 Hz recorder fields (all 0 without the observer). */
void camVzGlueRecord(camVzShadowRec_t *r);
/* Interactive hover (auto_motion.h): through a commanded turn and its
 * re-track bob the observer's restarted scale tracker has only a guessed G
 * (its last one): camVzGlueVzInput() then reports the estimate healthy on
 * healthyGuess (cam_vz_obs.h) instead of healthy. The autonomy glue sets it
 * every full step (autoMvOut_t.camGuessOk); false otherwise. IRAM. */
void camVzGlueSetGuessAllowed(bool allowed);
/* Interactive hover (2026-10-15): the measured forward progress ln(D0 / D) of the sortie's MOVEs (auto_fsm.h
 * autoFsmOut_t fwdLn), copied into every camera frame the observer gets (cam_vz_obs.h camVzObsFrame_t fwdLn: its gain
 * follows the scene distance through a forward move). The autonomy glue sets it every full step of a sortie. Inline,
 * a plain store: the stabilizer's IRAM path makes no call for it. */
extern volatile float camVzGlueFwdLnIn;
static inline void camVzGlueSetFwdLn(float fwdLn)
{
  camVzGlueFwdLnIn = fwdLn;
}
/* Interactive hover (2026-10-15): a commanded motion runs (auto_motion.h autoMvOut_t holdWait: a MOVE, a
 * SET_HEIGHT's correction or a YAW, to the end of its settle) or a contact's hush (auto_fsm.h autoFsmOut_t hush), a
 * guard event of the observer's late offset noise (cam_vz_obs.h camVzObsSetMotion), handed over before each block.
 * The autonomy glue sets it every controller step of a sortie (DRY's sim plant too), false outside one. Inline, a
 * plain store: the one store R1 added to the stabilizer task's autonomy step (docs/autonomy-a2b.md 25.9). */
extern volatile bool camVzGlueMotionIn;
static inline void camVzGlueSetMotion(bool moving)
{
  camVzGlueMotionIn = moving;
}
/* The scene distance for the interactive hover's moves: D = f / G (m, the
 * observer's vertical focal length kyGy over its gain), whether the tracker
 * has a G (and the estimate is fresh, not paused, not a bench mode, not
 * faulted), and its restarts since boot. False (all 0) without a snapshot.
 * IRAM: the autonomy glue reads it every 10th loop. */
bool camVzGlueScene(float *dM, bool *tracked, uint32_t *resets);
/* Why the camera height estimate is not healthy now (2026-10-15, docs/autonomy-a2b.md 22): AUTO_CB_* of auto_fsm.h,
 * 0x01 no accepted update for the observer's healthy window (2 s), 0x02 no tracker G (and no guess allowed), 0x04
 * its CUSUM alarm holds, 0x08 faulted, or the snapshot stale, paused or a bench mode. Flash: the glue calls it at
 * 10 Hz only while the estimate is out. */
uint8_t camVzGlueWhyBits(void);
/* The flight-summary fields of the current sortie (and its late offset noise's, cam_vz_shadow.h
 * camVzShadowLateStats, with FSUM_CVL_CFG_BAD when the stored setting was refused at boot), its number (ground
 * starts since boot, 0 before the first) and its age (ms since its ground start, on the IMU clock of this loop).
 * False without the observer. */
bool camVzGlueFlightStats(fsumCamVz_t *out, fsumCvLate_t *late, uint32_t *sortie, uint32_t *sortieAgeMs);

/* Snapshot for other tasks (USB 'camvz'), refreshed at 100 Hz. */
typedef struct {
  camVzObsOut_t out;
  camVzObsCounters_t cnt;
  fsumCamVz_t stats;
  uint8_t phase;          /* camVzShadowPhase_t */
  uint8_t liftBy;         /* camVzShadowLift_t of the sortie's first lift-off */
  uint8_t lastDecision;   /* camVzObsDecision_t of the latest new frame */
  bool inPsram;
  uint8_t benchAir;       /* camVzShadowBenchMode_t of 'camvz air' (0: off) */
  uint32_t sortie;
  float g0;               /* g, the prior's ground baseline (1 if none) */
  float gyroBiasDps[3];   /* in-flight gyro bias estimate subtracted from the camera's gyro angles (deg/s) */
  uint32_t bytes;         /* observer state */
  /* stabilizer-task cost (esp_timer), us: the step without the frame (IMU step, camera poll, publish)
   * on every 16th loop; the frame update on every frame */
  uint32_t stepMaxUs, stepSamples;
  uint64_t stepSumUs;
  uint32_t frameMaxUs, frameCalls;
  uint64_t frameSumUs;
  uint32_t stabStackFree; /* bytes: the camvz task's lowest free stack (UINT32_MAX: not yet measured) */
} camVzGlueStatus_t;

/* False until the first published snapshot (or without the observer). */
bool camVzGlueGetStatus(camVzGlueStatus_t *st);

/* The late offset noise (cam_vz_obs.h "Late offset noise", docs/autonomy-a2b.md 25) for USB 'camvz': in use this boot
 * and its counts this sortie (flash, read without the lock; not in the IRAM snapshot). False without the observer. */
typedef struct {
  float qLateMult, qLateS;  /* in use (NVS cvqlate / cvqlates at boot) */
  bool cfgBad;              /* a stored value was refused at boot: the multiplier 1 runs */
  uint8_t state;            /* CAM_VZ_LATE_* */
  uint32_t lateMs, guards;  /* this sortie: with the multiplier on; guard stops */
} camVzGlueLate_t;
bool camVzGlueLate(camVzGlueLate_t *l);

/* The offset drift rate T2 (cam_vz_obs.h "Offset drift rate", docs/autonomy-a2b.md 29) for USB 'camvz' and
 * 'status': in use this boot and its state this sortie (flash, read without the lock). False without the observer. */
typedef struct {
  float kR, priorMgS, boundMgS, startS;   /* in use (NVS cvt2k / cvt2r at boot; kR 0: off) */
  bool cfgBad;              /* a stored value was refused at boot, or came with the late offset noise on: T2 off */
  camVzObsT2_t t2;          /* this sortie */
} camVzGlueT2_t;
bool camVzGlueT2(camVzGlueT2_t *t);

/* The healthy offset speed-up (cam_vz_obs.h "Healthy offset speed-up", docs/autonomy-a2b.md 37) for USB 'camvz': in
 * use this boot and its state this sortie (flash, read without the lock). False without the observer. */
typedef struct {
  float mult;               /* in use (NVS cvhb / 10 at boot; 0: off) */
  bool cfgBad;              /* a stored value was refused at boot, or came with R1 or T2 on: off */
  bool on;                  /* applies to the current block (healthy, airborne) */
  uint32_t onMs;            /* this sortie: with the multiplier applied */
} camVzGlueHb_t;
bool camVzGlueHb(camVzGlueHb_t *h);

#endif /* CAM_VZ_GLUE_H_ */
