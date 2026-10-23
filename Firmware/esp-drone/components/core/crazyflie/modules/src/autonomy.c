/*
 * ESP-Drone Firmware
 *
 * autonomy.c - Autonomy glue (stage A2b, autonomy.h, docs/autonomy-a2b.md)
 *
 * Runs the state machine (auto_fsm.h) and the vertical controller
 * (vert_ctrl.h) inside the 1 kHz stabilizer loop. Each loop the IRAM
 * dispatcher autonomyArbitrate() decides whether the state machine has to
 * step: in IDLE with nothing waiting (manual flight, nearly all the time) it
 * returns at once, and the state machine steps every 100th loop (10 Hz) or
 * on a command, a Mac KILL or a new session. Otherwise it fills the fast
 * inputs (attitude, tilt, rates, link and phone ages) from IRAM, then runs
 * stepFull(), also in IRAM with autoFsmStep(), vzStep() and vzSimStep()
 * (from flash they cost ~340 us a loop on the bench and the loop fell to
 * ~630-700 Hz): the slow inputs every 10th loop (camera, battery,
 * locks, the aux task's heap reading), the mailbox take, autoFsmStep(), the
 * one-step actions and the controller, and finally overwrites the whole
 * setpoint (section 8.1 of docs/autonomy-a2.md) unless the action is NONE.
 *
 * Two thrust barriers: the override's thrust is autoSelectThrust() (<= the
 * mode maximum), and the stabilizer clamps control.thrust with
 * autoClampThrust(control.thrust, autonomyThrustMax()) after the controller,
 * which is 0 for ZERO and CUT in every mode and for DRY in every state.
 *
 * Every time stamp kept here is T2M(xTaskGetTickCount()), the FSM's clock,
 * and every age goes through autoAgeMs / autoAgeUsToMs (autonomy.h "Clock").
 * Everything the 1 kHz loop touches is static in internal RAM; nothing large
 * goes on the stabilizer's stack.
 */
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "autonomy.h"
#include "cam_vz_glue.h"
#include "camera_flow.h"
#include "commander.h"
#include "controller_pid.h"
#include "crtp_commander.h"
#include "flight_rtc_log.h"
#include "ground_cool_glue.h"
#include "imu_level_cal_store.h"
#include "ledseq.h"
#include "motor_guard.h"
#include "motors.h"
/* pm_esplane.h pulls in IDF's legacy ADC driver header and its deprecation
 * #warning; only pmGetBatteryVoltage() is used here. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#include "pm_esplane.h"
#pragma GCC diagnostic pop
#include "sensfusion6.h"
#include "sensors_mpu6050_hm5883L_ms5611.h"
#include "stabilizer.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "AUTO"
#include "debug_cf.h"

#define HOLD_FRESH_MS        300    /* the drift hold's output counts as applied this fresh (controller_pid.c) */
#define SIM_VBAT_EVERY_MS    100    /* the sim's battery as the 10 Hz reading (test_auto_sortie.c) */
#define SIM_CAM_WAKE_S       1.5f   /* DRY/BENCH sim camera model: warm, awake 1.5 s after the first thrust */
#define STACK_CHECK_EVERY    50     /* IDLE periodic steps (5 s) between stack high-water reads */
#define K_SAVE_RETRY_MS      5000   /* a failed vzK save is retried this much later */
#define PROF_EVERY           16     /* loops: the quiescent path and active steps are timed on every 16th */
#define HEAP_MAX_AGE_MS      200    /* G9: an older internal-heap reading (or none) counts as 0 bytes */

/* ---- shared with the other tasks ------------------------------------------ */

/* The mailbox, the capture request, the status snapshot and its seq share
 * one spinlock; it is held for the copies and the mailbox's short calls
 * (autoMbTake/autoMbPublish/autoMbSetState: flash code, only on a command or
 * a transition). */
static portMUX_TYPE mbMux = portMUX_INITIALIZER_UNLOCKED;
static autoMailbox_t mb;
static uint32_t postMs;                 /* the pending command's post, for the latency profile */
static autoStatusMsg_t statusSnap;
static uint32_t statusSeq;
static struct {
  uint32_t reqId;
  uint8_t n;
  uint16_t gapMs;
  bool pending;                         /* not yet taken by the link */
  volatile uint8_t progress;            /* autoCapture_t */
  volatile uint16_t photoId;
} cap;

/* Single aligned words, one writer each. */
static volatile uint32_t killCount;     /* link task */
static volatile uint32_t sessionSeq;    /* link task */
static volatile bool sessionValid;
static volatile uint32_t hbMs;
static volatile bool hbAny;
static autoAppHook_t appHook;           /* CRTP RX task */
static volatile bool camAwakeNow;       /* stabilizer task */
static volatile bool holdBlock;         /* stabilizer task: autonomyHoldBlocked() */
/* Interactive hover: the hold's moving reference (stabilizer task writes on
 * every full step, the camera task reads per frame). */
static portMUX_TYPE holdRefMux = portMUX_INITIALIZER_UNLOCKED;
static autonomyHoldRef_t holdRef;
static volatile autoBenchSrc_t benchSrc = AUTO_BENCH_SRC_SIM;   /* USB, IDLE only */
/* The settings' RAM copy. Written by init, and after it only by a settings
 * writer holding settingClaim() (the flash interlock plus the one-writer
 * flag), so an ARM taken while a value changes is refused (G1) and every
 * sortie sees the values of a finished write. */
static volatile int32_t setting[AUTO_SET_COUNT];
static bool settingWriter;              /* under mbMux: a settings writer (USB task or aux task) holds the claim */
static volatile bool savePending;       /* stabilizer sets, aux task clears: a learned vzK to store */
static volatile int32_t pendingK;
/* G9's internal heap. heap_caps_get_free_size() is flash code, so the aux
 * task samples it (autonomyPoll(), every 10 ms) with its time; slowInputs()
 * fails G9 on a reading older than HEAP_MAX_AGE_MS or on none, so a stalled
 * or missing aux task cannot pass an old value. Written value, time, flag;
 * read in the opposite order (atomic release / acquire). */
static uint32_t heapFree, heapMs;
static bool heapAny;

/* Reset-lock (section 6): locked unless autonomyInit() decides otherwise,
 * so a skipped or reordered init leaves FLIGHT and BENCH locked. .data.
 * After init only autonomyUsbUnlock() (USB task) writes it, and only to
 * false; the stabilizer reads it on every full step. */
static volatile bool resetLocked = true;
static RTC_NOINIT_ATTR autoRtcRecord_t rtc;
static bool rtcWasValid;
static int bootResetReason;

/* The per-loop words (IRAM path). thrustMax starts free: until the first
 * step, manual flight is untouched. */
static struct {
  volatile uint8_t state;               /* autoState_t of the last step: autonomyActive() and the fast path */
  volatile uint8_t lastAction;          /* autoThrust_t of the last step */
  volatile float thrustMax;             /* barrier 2: autoBarrierMax of the last step */
  float applied;                        /* what the motors got (autonomyNoteApplied) */
  float gravZ;
  bool wasActive;                       /* the last full step ran outside IDLE */
  uint32_t loops;
  uint32_t lastKill, lastSession;
} g = {
  .state = AUTO_IDLE,
  .lastAction = AUTO_THR_NONE,
  .thrustMax = AUTO_THRUST_MAX_FREE,
};

/* Profile (written by the stabilizer task, read by USB; diagnostic, a torn
 * read is harmless). A reset is requested and done by the stabilizer. */
static struct {
  uint32_t fullSteps, fastMaxUs;
  uint32_t idleMaxUs, idleN, activeMaxUs, activeN;
  uint64_t idleSumUs, activeSumUs;
  uint32_t activeLoops, activeMs, cmdLatMaxMs;
  uint32_t stackFreeMin;
  uint32_t idleTicks;
  bool wasActive;
  uint32_t lastActiveMs;
} prof;
static volatile bool profResetReq;

/* ---- stabilizer task only ------------------------------------------------- */

static autoFsm_t fsm;
static autoFsmIn_t fin;
static autoFsmOut_t fout;               /* zero: IDLE, action NONE */
_Static_assert(AUTO_IDLE == 0 && AUTO_THR_NONE == 0, "the zeroed output is IDLE with action NONE");
static vzState_t vz;
static vzConfig_t vzCfg;
static vzOutput_t vo;
static vzInput_t vin;
static vzSimState_t sim;
static vzSimConfig_t simCfg;
static bool vzActive;                   /* from the sortie's vzReset until IDLE */
static bool simPlant;                   /* latched at ARM: DRY, or BENCH on the sim */
static autoMode_t sortieMode;           /* latched at ARM: the commanded mode (the FSM's own copy is private) */
static int32_t sortieK, sortieBair, sortieCapLo;   /* vzK, vzBair, vzcaplo latched at ARM */
static int32_t sortieEstOn, sortieEstMs;           /* vzest, vzestms latched at ARM (the EST_RANGE guard, 2026-10-22) */
static bool vzParamsOk;
static float kSortie;                   /* the sortie's kPrior */
static float restVbat;                  /* resting battery at ARM (the sim's vRest) */
static uint32_t hoverWaitMs;            /* NVS autowait latched at ARM (AUTO_STATUS, saturated at 65535) */
static volatile uint16_t sortieMvSpeed = AUTO_MV_SPEED_DEFAULT_PCT;   /* NVS mvspeed latched at ARM (2026-10-16) */
static volatile uint8_t sortieMvFix;    /* NVS mvfix latched at ARM (2026-10-17; the camera's GAIN, BRAKE bits) */
static float yawHold;
static uint64_t lastImuUs;
static float simVbat, simThrust;
static uint32_t simVbatMs;
static bool motorsReadyReal;
static bool ledOn, camSkip;
static bool anyOkFrame;
static uint64_t lastOkFrameUs;
static autoSinceTrack_t viewTrack, groundTrack;
static uint8_t lastCmdType, lastCmdResult;
static uint16_t lastEvCount;            /* the report's evCount last published */
static autoStatusMsg_t staging;
static uint32_t kRetryMs;               /* aux task: no vzK save before */
static uint32_t shiftRefused;           /* SET_HEIGHT shifts vzShiftHeight() refused (autonomyProfile_t; never expected) */
static uint8_t holdLimLast;             /* the camera's hold-at-limit bits last given to the state machine */

/* ---- IRAM helpers ---------------------------------------------------------- */

/* max that keeps a NaN (fmaxf would drop it, and the FSM must see it). */
static inline IRAM_ATTR float maxNan(float a, float b)
{
  return (a > b || a != a) ? a : b;
}

/* ---- the full step (IRAM; vzStart() runs once a sortie, from flash) ------- */

/* Every 10th loop while active, every IDLE full step. In IRAM with the
 * getters it calls (from flash they cost ~400 us every 10th loop on the
 * bench). */
/* The camera hold at its limit (auto_fsm.h autoFsmHoldLimit, docs/autonomy-a2b.md 27.4): on a change of the camera's
 * bits only (a few per sortie): flash. */
static void __attribute__((noinline)) holdLimitNote(uint8_t bits, uint32_t nowMs)
{
  holdLimLast = bits;
  autoFsmHoldLimit(&fsm, bits, nowMs);
}

static void IRAM_ATTR __attribute__((noinline)) slowInputs(autoFsmIn_t *in, uint32_t nowMs)
{
  in->vbat = pmGetBatteryVoltage();
  in->vbatFastMin = (float)flightRtcLogLiveVbatMin() * 0.001f;
  in->camState = (uint8_t)cameraFlowState();
  in->camFps = cameraFlowFps();
  cameraFlowSample_t c;
  const bool have = cameraFlowGetLatest(&c);
  const uint64_t nowUs = (uint64_t)esp_timer_get_time();
  if (have && c.ok) {
    lastOkFrameUs = c.frameTimeUs;
    anyOkFrame = true;
  }
  in->camLagUs = cameraFlowMotionLagUsV;
  in->camFrameAgeMs = anyOkFrame ? autoAgeUsToMs(nowUs, lastOkFrameUs) : UINT32_MAX;
  in->camViewMs = autoSinceStep(&viewTrack, in->camState == CAMERA_FLOW_RUNNING && have && c.ok &&
                                in->camFrameAgeMs <= AUTO_VIEW_FRAME_AGE_MS, nowMs);
  float hr, hp;
  bool eng;
  uint64_t ht;
  cameraFlowGetHold(&hr, &hp, &eng, &ht);
  in->holdEnabled = cameraFlowHoldEnabled();
  in->holdApplied = eng && autoAgeUsToMs(nowUs, ht) < HOLD_FRESH_MS;
  in->holdCoasting = cameraFlowHoldCoasting();
  cameraFlowGetHoldProgress(&in->holdProgLat, &in->holdProgFwd, &in->holdEpochSeen);
  in->holdMoving = cameraFlowHoldMoving();   /* (a byte load: mvfix STILL) */
  const uint8_t holdLim = cameraFlowHoldLimitBits();   /* (a byte load; the note only on a change) */
  if (holdLim != holdLimLast) {
    holdLimitNote(holdLim, nowMs);
  }
  camVzGlueScene(&in->sceneDM, &in->sceneTracked, &in->sceneResets);   /* the sim plant's D: auto_motion.c */
  /* why the camera height is out (flash, only while it is: the camera rule's CAM_LOST detail) */
  in->camBits = (vzActive && !simPlant && !vo.camHealthy) ? camVzGlueWhyBits() : 0u;
  in->onGround = stabilizerOnGround();
  in->onGroundMs = autoSinceStep(&groundTrack, in->onGround, nowMs);
  in->lastFlightMotorMs = stabilizerLastFlightMotorMs();
  in->restartLocked = stabilizerRestartLocked();
  in->emergencyStop = stabilizerEmergencyStopActive();
  motorsReadyReal = motorsReadyNoLock() && !motorsIsInhibited();
  in->motorGuardOn = motorGuardEnabled();
  in->motorGuardTripped = motorGuardTripped();
  in->gcap = stabilizerGetThrustCap();
  const bool heapSeen = __atomic_load_n(&heapAny, __ATOMIC_ACQUIRE);
  const uint32_t heapAt = __atomic_load_n(&heapMs, __ATOMIC_ACQUIRE);
  const uint32_t heapNow = __atomic_load_n(&heapFree, __ATOMIC_RELAXED);
  in->heapIntFree = autoAgeMs(nowMs, heapAt, heapSeen) <= HEAP_MAX_AGE_MS ? heapNow : 0u;
  in->autoEnabled = setting[AUTO_SET_AUTOON] != 0;
  in->appRequired = setting[AUTO_SET_APPREQ] != 0;
  in->gestureRequired = setting[AUTO_SET_AUTOGEST] != 0;
  in->vmin = (float)setting[AUTO_SET_VMIN] * 0.001f;
  in->airMaxNvsMs = (uint16_t)setting[AUTO_SET_AUTOAIR];
  in->hoverWaitNvsMs = (uint32_t)setting[AUTO_SET_AUTOWAIT];   /* no u16 cast: 120000 would wrap to 54464 */
  in->hoverAccelNvsMs = (uint16_t)setting[AUTO_SET_AUTOACCEL];
  in->vzParamsOk = vzParamsOk;
}

/* vzReset (section 2.4): this sortie's controller, and the sim plant. */
static void __attribute__((noinline)) vzStart(const autoFsmOut_t *out, uint32_t nowMs)
{
  vzDefaultConfig(&vzCfg);
  vzConfigSetHeight(&vzCfg, out->hTargetM);   /* above 0.40 m: hMax, the climb timeout and LAND flat part grow */
  vzCfg.kPrior = (float)sortieK;   /* the values of the ARM step */
  vzCfg.bAir = (float)sortieBair * 1e-4f;
  vzCfg.capLo = (float)sortieCapLo * 0.001f;
  vzCfg.estRangeOn = sortieEstOn != 0;   /* (vzest; vzestms: the LAND's grace; both ARM-latched, 2026-10-22) */
  vzCfg.estGraceS = (float)(sortieEstMs < 0 ? 0 : sortieEstMs) * 0.001f;
  kSortie = vzCfg.kPrior;
  vzInit(&vz, &vzCfg);   /* an invalid config leaves it in CUT: the FSM kills */
  memset(&vo, 0, sizeof(vo));
  camVzGlueSetLiftoffHint(false);
  camVzGlueSetMotion(false);
  lastImuUs = 0;
  simThrust = 0.0f;
  if (simPlant) {
    vzSimDefaultConfig(&simCfg);
    simCfg.kTrue = vzCfg.kPrior;   /* the drone matches its prior */
    if (restVbat >= 3.0f && restVbat <= 4.5f) {
      simCfg.vRest = restVbat;
    }
    simCfg.camWakeS = SIM_CAM_WAKE_S;
    simCfg.camConvS = 0.3f;
    simCfg.camVzOffset = 0.0f;
    vzSimInit(&sim, &simCfg, esp_random() | 1u);
    simVbat = sim.vbat;
    simVbatMs = nowMs + SIM_VBAT_EVERY_MS;
  }
  vzActive = true;
}

/* One controller step (section 2.4), on the real IMU or on the sim. */
static void IRAM_ATTR __attribute__((noinline)) vzRun(const sensorData_t *sd, const autoFsmIn_t *in, const autoFsmOut_t *out)
{
  vzInput_t *vi = &vin;
  const uint64_t imuUs = sd->interruptTimestamp;
  vi->dt = (lastImuUs != 0 && imuUs > lastImuUs) ? (float)(imuUs - lastImuUs) * 1e-6f : 0.001f;
  lastImuUs = imuUs;
  vi->cmd = out->vzCmd;
  vi->fromSim = simPlant;
  vi->vRef = out->mv.vRef;   /* the interactive hover: 0 without a vertical move */
  vi->hOff = out->mv.hOff;
  vi->blind = out->blind;   /* every LAND after a contact of the sortie: blind to the estimate (user decision (B)) */
  vi->hush = out->hush;         /* a contact's hush: the estimate frozen (2026-10-15) */
  vi->resume = out->resume;     /* a SINK resumed: the camera height back */
  /* the fast LAND's allow list: the reason of the latest transition, read by the controller at its LAND's start
   * (a LAND the controller starts itself is never fast; the FSM's LAND sets its reason on the same step) */
  vi->landFastOk = autoFsmLandFastReason(out->reason);
  camVzGlueSetFwdLn(out->fwdLn);   /* the observer's gain follows the scene distance through a forward move */
  /* a commanded motion or a contact's hush: the observer's late offset noise pauses (a DRY sortie's too, so a bench
   * observer in camvz air sees it; the one byte store this path gained for R1, docs/autonomy-a2b.md 25.9) */
  camVzGlueSetMotion(out->mv.holdWait || out->hush);
  if (simPlant) {
    if ((int32_t)(in->nowMs - simVbatMs) >= 0) {
      simVbat = sim.vbat;
      simVbatMs = in->nowMs + SIM_VBAT_EVERY_MS;
    }
    vi->accZ = sim.accZ;
    vi->gravZ = 1.0f;
    vi->tiltDeg = 0.0f;
    vi->gyroMaxDps = 0.0f;
    vi->vbat = simVbat;
    vi->thrustApplied = simThrust;   /* what the sim got */
    vi->camHealthy = sim.camHealthy;
    vi->camVzValid = sim.camVzValid;
    vi->camVz = sim.camVz;
    vi->camBias = sim.camBias;
  } else {
    vi->accZ = sensfusion6GetAccZ(sd->acc.x, sd->acc.y, sd->acc.z);
    vi->gravZ = g.gravZ;
    vi->tiltDeg = in->tiltDeg;
    vi->gyroMaxDps = maxNan(maxNan(fabsf(sd->gyro.x), fabsf(sd->gyro.y)), fabsf(sd->gyro.z));
    vi->vbat = in->vbat;
    vi->thrustApplied = g.applied;
    camVzGlueSetGuessAllowed(out->mv.camGuessOk);   /* a commanded turn: the tracker's guessed G may serve */
    camVzGlueVzInput(vo.g0, vi);
  }
  vzStep(&vz, vi, &vo);
  if (simPlant) {
    vzSimStep(&sim, &simCfg, vo.thrust, vi->dt);   /* unclamped: BENCH motors get min(thrust, cap) */
    simThrust = vo.thrust;
  } else {
    camVzGlueSetLiftoffHint(vo.liftedOff);
  }
  if (vo.liftoffNow && sortieMode != AUTO_MODE_DRY) {
    stabilizerNoteLiftoff();   /* the take-off window ends 1.5 s after it (BENCH: the sim's) */
  }
}

static uint8_t recFlags(const autoFsmOut_t *o, const autoFsmIn_t *in)
{
  return (uint8_t)((o->waiting ? AUTO_STF_WAITING : 0) | (o->stable ? AUTO_STF_STABLE : 0) |
                   (vzActive && vo.camHealthy ? AUTO_STF_CAM_HEIGHT : 0) | (camSkip ? AUTO_STF_CAM_AWAKE : 0) |
                   (resetLocked ? AUTO_STF_RESET_LOCKED : 0) | (o->prespinLed ? AUTO_STF_PRESPIN : 0) |
                   (vzActive && simPlant ? AUTO_STF_SIM : 0) | (in->holdApplied ? AUTO_STF_HOLD : 0));
}

/* The AUTO_STATUS snapshot: filled outside the lock, copied in under it. A
 * transition also moves the flash interlock's state and the seq, after the
 * snapshot, so the link never reads a new seq with an old one. */
static void IRAM_ATTR __attribute__((noinline)) publishStatus(const autoFsmOut_t *out, const autoFsmIn_t *in, bool transition)
{
  autoStatusExtra_t x = {
    .upMs = in->nowMs,
    .transitions = rtc.transitions,
    .killCount = in->killCount,
    .resetLocked = resetLocked,
    .simPlant = vzActive && simPlant,
    .holdApplied = in->holdApplied,
    .camAwake = camSkip,
    .trips = autoTripBits(in, vzActive && vo.atCap),
    .capture = (autoCapture_t)cap.progress,
    .photoId = cap.photoId,
    .vbat = in->vbat,
    .vbatFastMin = in->vbatFastMin,
    .kPrior = vzActive ? kSortie : (float)setting[AUTO_SET_VZK],
    .lastCmdType = lastCmdType,
    .lastCmdResult = lastCmdResult,
    .rep = autoFsmReport(&fsm),
    .vzActive = vzActive,
  };
  autoStatusFill(&staging, out, &vo, &x);
  staging.hoverWaitMs = autoSatMs16(out->state == AUTO_IDLE ? (uint32_t)setting[AUTO_SET_AUTOWAIT] : hoverWaitMs);
  portENTER_CRITICAL(&mbMux);
  if (transition) {
    autoMbSetState(&mb, out->state);
  }
  statusSnap = staging;
  if (transition) {
    statusSeq++;
  }
  portEXIT_CRITICAL(&mbMux);
}

/* The sortie's motion speed step (2026-10-16, auto_motion.h "fast moves"): NVS mvspeed latched at the ARM step, after
 * its autoMotionInit (enter ARMED), into the state machine's motions; with it the guard margins (2026-10-17,
 * auto_motion.h "guard margins": mvfloor, mvfstop, mvprogm, mvprogr, mvlatm, mvnear, clamped by autoMvMarginsFrom),
 * the move fixes (mvfix, auto_motion.h "move fixes"), the fast steps' acceleration and settle (mvacc, mvsettle,
 * 2026-10-17: before the speed step, which copies its settle into the motions) and the guards-off settings (mvklow,
 * mvlat into the margins, mvwmax before the speed step as the fast caps; 2026-10-18, auto_motion.h "guards off"). Once
 * per sortie: flash. */
static void __attribute__((noinline)) sortieSpeedLatch(void)
{
  const int32_t v = setting[AUTO_SET_MVSPEED];
  sortieMvSpeed = (uint16_t)(v < AUTO_MV_SPEED_MIN_PCT ? AUTO_MV_SPEED_MIN_PCT
                                                       : (v > AUTO_MV_SPEED_MAX_PCT ? AUTO_MV_SPEED_MAX_PCT : v));
  autoMotionSetFastCaps(setting[AUTO_SET_MVACC], setting[AUTO_SET_MVSETTLE]);
  autoMotionSetFwdRate(setting[AUTO_SET_MVWMAX]);
  autoMotionSetViewDeb(setting[AUTO_SET_MVVIEW]);   /* (the VIEW_FAST timer, 2026-10-22) */
  cameraFlowSetSceneResidual((uint8_t)setting[AUTO_SET_HOLDRES]);   /* (the scene gate's residual limit, 2026-10-22) */
  /* the left slide of the forward MOVEs (2026-10-22): the hold's lateral position gain on a MOVE's frames and the
   * motions' feedforward roll per forward speed */
  cameraFlowSetMoveLatGain((uint16_t)setting[AUTO_SET_MVLATK]);
  autoMotionSetFfRoll(setting[AUTO_SET_MVFFR]);
  autoFsmSetMvSpeed(&fsm, sortieMvSpeed);
  autoMvMargins_t mg;
  autoMvMarginsFrom(setting[AUTO_SET_MVFLOOR], setting[AUTO_SET_MVFSTOP], setting[AUTO_SET_MVPROGM],
                    setting[AUTO_SET_MVPROGR], setting[AUTO_SET_MVLATM], setting[AUTO_SET_MVNEAR],
                    setting[AUTO_SET_MVFIX], &mg);
  autoMvMarginsExtra(&mg, setting[AUTO_SET_MVKLOW], setting[AUTO_SET_MVLAT]);
  autoMotionSetMargins(&mg);
  sortieMvFix = mg.fix;   /* (the camera task's: autonomyMvFixBits) */
}

/* A FLIGHT sortie that landed: the learned hover gain, saved later by
 * autonomyPoll() (never from this task: a flash write stops both cores). The
 * mode is the glue's own copy from the ARM step (autoFsm_t is private). */
static void learnK(void)
{
  float k = kSortie;
  if (!simPlant && sortieMode == AUTO_MODE_FLIGHT && vzLearnK(&vz, &k) && isfinite(k)) {
    pendingK = (int32_t)lrintf(k);
    savePending = true;
  }
}

/* SET_HEIGHT (auto_fsm.h AUTO_SH_*, docs/autonomy-a2b.md 23): the step that accepted it shifts the controller's height
 * estimate by d before its vzRun (the state machine checked HOLD on the previous step's output, which is the state's
 * phase now). Called on every step that took a command (flash: a few per sortie, as the mailbox's calls). */
static void __attribute__((noinline)) applyHeightShift(autoFsmOut_t *out)
{
  if (out->hShift != 0.0f) {
    if (vzActive && !vzShiftHeight(&vz, out->hShift)) {
      shiftRefused++;
    }
    out->hShift = 0.0f;
  }
}

static void IRAM_ATTR __attribute__((noinline)) stepFull(const state_t *st, const sensorData_t *sd, bool pending, bool slow,
                                               bool publish)
{
  autoFsmIn_t *in = &fin;
  autoFsmOut_t *out = &fout;
  const uint32_t nowMs = in->nowMs;
  const autoState_t before = out->state;
  const autoReason_t reasonBefore = out->reason;

  in->vz = vo;
  in->resetLocked = resetLocked;
  in->capture = (autoCapture_t)cap.progress;

  /* The command, if any: taken under the lock together with the flash
   * interlock's state (an NVS write or a console change in progress refuses
   * an ARM, G1). */
  memset(&in->cmd, 0, sizeof(in->cmd));
  uint32_t ticket = 0;
  bool took = false, flashBusy = false;
  if (pending) {
    portENTER_CRITICAL(&mbMux);
    took = autoMbTake(&mb, &in->cmd, &ticket, &flashBusy);
    portEXIT_CRITICAL(&mbMux);
  }
  /* The slow inputs after the take, and on every step that took a command:
   * once an ARM is taken with the interlock free no writer can begin until
   * its result is out, so they show only finished console changes (an ARM
   * taken while one runs is refused, G1). Read before the take, a change
   * ending in between let an ARM pass on the values before it (review R2). */
  if (slow || took) {
    slowInputs(in, nowMs);
  }
  bool lockArm = took && flashBusy;
  in->simPlant = simPlant;   /* latched from ARM to IDLE */
  if (took && in->cmd.type == AUTO_CMD_ARM) {
    /* The commanded mode decides the plant (G12 sees the real choice); FLIGHT never runs on the sim. */
    const autoMode_t m = (autoMode_t)in->cmd.mode;
    in->simPlant = m == AUTO_MODE_DRY || (m == AUTO_MODE_BENCH && benchSrc == AUTO_BENCH_SRC_SIM);
    /* A controller on the real inputs reads the camera height: a camvz bench
     * mode or a paused observer (its snapshot frozen, possibly in CVS_AIR)
     * set before the sortie must not ride into it (section 7.3). */
    if ((m == AUTO_MODE_FLIGHT || (m == AUTO_MODE_BENCH && !in->simPlant)) &&
        (camVzGlueBenchAir() != CVS_BENCH_OFF || camVzGluePaused())) {
      lockArm = true;
    }
    in->randToken = esp_random() | 1u;
  }
  in->motorsReady = motorsReadyReal && !lockArm;

  autoFsmStep(&fsm, in, out);

  const bool transition = out->state != before || out->reason != reasonBefore;
  if (transition && before == AUTO_IDLE && out->state == AUTO_ARMED) {
    /* A new sortie (only an ARM command leaves IDLE for ARMED). Settings
     * cannot change while it runs (the flash interlock refuses every settings
     * writer outside IDLE); the controller's are latched here all the same. */
    simPlant = in->simPlant;
    sortieMode = (autoMode_t)in->cmd.mode;
    sortieK = setting[AUTO_SET_VZK];
    sortieBair = setting[AUTO_SET_VZBAIR];
    sortieCapLo = setting[AUTO_SET_VZCAPLO];
    sortieEstOn = setting[AUTO_SET_VZEST];
    sortieEstMs = setting[AUTO_SET_VZESTMS];
    restVbat = in->vbat;
    hoverWaitMs = (uint32_t)(setting[AUTO_SET_AUTOWAIT] < AUTO_HOVER_WAIT_MAX_MS ? setting[AUTO_SET_AUTOWAIT]
                                                                                  : AUTO_HOVER_WAIT_MAX_MS);
    sortieSpeedLatch();   /* mvspeed, mvacc, mvsettle and the guard margins into the motions (flash) */
    /* The accelerometer compensation's reference: the drone rests with the motors off here, ~1 s before the
     * idle spin takes g0 (accel_tempco.h, docs/autonomy-a2b.md 36); the sensors task serves it within 50 ms. */
    sensorsMpu6050AccTcLatch(false);
    portENTER_CRITICAL(&mbMux);
    cap.pending = false;
    cap.progress = AUTO_CAP_NONE;
    cap.photoId = 0;
    portEXIT_CRITICAL(&mbMux);
  }

  /* One-step actions (section 2.2 step 3). */
  if (out->yawCapture || out->state == AUTO_ARMED || (out->state == AUTO_TAKEOFF && !vzActive)) {
    yawHold = st->attitude.yaw;   /* follows the heading until the motors start */
  }
  if (out->vzReset) {
    vzStart(out, nowMs);
  }
  if (out->captureStart) {
    portENTER_CRITICAL(&mbMux);
    cap.reqId++;
    if (cap.reqId == 0) {
      cap.reqId = 1;
    }
    cap.n = out->captureN;
    cap.gapMs = out->captureGapMs;
    cap.pending = true;
    cap.progress = AUTO_CAP_BUSY;
    cap.photoId = 0;
    portEXIT_CRITICAL(&mbMux);
  }
  if (out->lockAppThrust) {
    /* Hand-back: the app's throttle needs a fresh zero, and the app setpoint
     * still queued in the commander (its thrust kept up to 2 s after the app
     * falls silent) is dropped (review E2E-1). */
    crtpCommanderRpytLockThrust();
    commanderDropSetpoint();
  }
  if (out->setEmergencyStop) {
    stabilizerSetEmergencyStop();
  }
  if (out->clearEmergencyStop) {
    stabilizerResetEmergencyStop();
  }
  if (out->prespinLed != ledOn && (out->prespinLed ? ledseqRun : ledseqStop)(&seq_autoPrespin)) {
    ledOn = out->prespinLed;   /* a full LED queue retries on the next step */
  }
  if (out->camNoStandby != camSkip) {
    camSkip = out->camNoStandby;
    cameraFlowSetTakeoffStandbySkip(camSkip);
    camAwakeNow = camSkip;
  }
  if (took) {
    applyHeightShift(out);   /* an accepted SET_HEIGHT: the estimate shifted before this step's vzRun (flash) */
    /* After the capture request, so a PHOTO's set exists before its ACK.
     * The FSM answers every command it is given. */
    const autoResult_t r = out->cmdDone ? out->cmdResult : AUTO_R_BAD_STATE;
    const uint32_t d = out->cmdDone ? out->cmdDetail : (uint32_t)before;
    lastCmdType = (uint8_t)in->cmd.type;
    lastCmdResult = (uint8_t)r;
    portENTER_CRITICAL(&mbMux);
    autoMbPublish(&mb, ticket, r, d, out->state);
    const uint32_t lat = autoAgeMs(nowMs, postMs, true);
    portEXIT_CRITICAL(&mbMux);
    if (lat > prof.cmdLatMaxMs) {
      prof.cmdLatMaxMs = lat;
    }
  }
  if (transition) {
    autoRtcNoteTransition(&rtc, nowMs, before, out->state, out->reason, (uint8_t)out->vzLandReason, out->mode);
    if (out->state == AUTO_IDLE) {
      if (before == AUTO_LAND && vzActive) {
        learnK();
      }
      vzActive = false;
      simPlant = false;
      memset(&vo, 0, sizeof(vo));   /* VZ_IDLE outside a sortie */
      camVzGlueSetLiftoffHint(false);
      portENTER_CRITICAL(&mbMux);
      cap.pending = false;          /* a set the link has not started belongs to no sortie now */
      portEXIT_CRITICAL(&mbMux);
    }
  }

  /* The hold's moving reference for the camera task (interactive hover; all
   * 0 without a motion). */
  {
    autonomyHoldRef_t r = {
      .epoch = out->mv.epoch,
      .snapEpoch = out->mv.snapEpoch,
      .refLat = out->mv.refLat,
      .refFwd = out->mv.refFwd,
      .rateLat = out->mv.rateLat,
      .rateFwd = out->mv.rateFwd,
      .ffRoll = out->mv.ffRollDeg,
      .ffPitch = out->mv.ffPitchDeg,
      .dLowM = out->mv.dLowM,
    };
    portENTER_CRITICAL(&holdRefMux);
    holdRef = r;
    portEXIT_CRITICAL(&holdRefMux);
  }

  /* The controller (section 2.4), from vzReset until IDLE. */
  if (vzActive) {
    vzRun(sd, in, out);
  }
  /* Contact in LAND, latched at the loop rate (an impact lasts 1-3 recorder
   * samples; the hold steps once per camera frame, ~120 ms late): from then
   * on the camera hold commands no lean (A2b hops, 2026-10-12). LAND
   * COLLISION blocks it from the contact's own step for the whole LAND, so it
   * no longer leans toward what was hit (user decision 2026-10-13), and so
   * does a LAND begun in a contact's window (collided). A LAND long after a
   * contact (blind since 2026-10-15) keeps the hold until it touches down:
   * it holds the drone at its anchor, off what it hit. */
  holdBlock = out->state == AUTO_LAND && (holdBlock || out->collided || (vzActive && (vo.contact || vo.touchdown)) ||
                                          in->accNormG > AUTO_TD_CONTACT_G);

  /* an event (hold and report) is pushed at once, as a transition is */
  const uint16_t evN = autoFsmReport(&fsm)->evCount;
  const bool evNew = evN != lastEvCount;
  lastEvCount = evN;
  if (publish || transition || took || evNew) {
    publishStatus(out, in, transition || evNew);
  }
}

/* ---- stabilizer task, every loop (IRAM) ------------------------------------ */

void IRAM_ATTR autonomyArbitrate(setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state,
                                 uint32_t nowMs, uint32_t loopGapUs)
{
  const uint32_t loop = ++g.loops;
  const bool timed = (loop % PROF_EVERY) == 0;
  const int64_t t0 = timed ? esp_timer_get_time() : 0;
  const bool pending = autoMbPending(&mb);
  const uint32_t kc = killCount, ss = sessionSeq;
  const autoState_t s0 = (autoState_t)g.state;
  if (autoIdleSkip(s0, (autoThrust_t)g.lastAction, pending, kc != g.lastKill, ss != g.lastSession, loop,
                   AUTONOMY_IDLE_STEP_EVERY)) {
    if (timed) {
      const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
      if (us > prof.fastMaxUs) {
        prof.fastMaxUs = us;
      }
    }
    return;
  }
  const bool idleStep = s0 == AUTO_IDLE;
  const bool timeIt = timed || idleStep;
  const int64_t ts = timed ? t0 : (idleStep ? esp_timer_get_time() : 0);
  const bool periodic = (loop % AUTONOMY_PUBLISH_EVERY) == 0;
  const bool events = pending || kc != g.lastKill || ss != g.lastSession;

  /* The fast inputs: every full step. */
  autoFsmIn_t *in = &fin;
  in->nowMs = nowMs;
  in->killCount = kc;
  g.lastKill = kc;
  in->sessionChanged = ss != g.lastSession;
  g.lastSession = ss;
  in->sessionValid = sessionValid;
  in->hbAgeMs = autoAgeMs(nowMs, hbMs, hbAny);
  in->appInputCount = __atomic_load_n(&appHook.inputCount, __ATOMIC_ACQUIRE);
  in->appAgeMs = autoAgeMs(nowMs, appHook.lastAppMs, appHook.any);
  in->appInputAgeMs = autoAgeMs(nowMs, appHook.lastInputMs, appHook.anyInput);
  in->appGestureAgeMs = autoAgeMs(nowMs, appHook.gestureMs, appHook.gestured);
  in->appThrustZero = appHook.thrustZero;
  in->rollDeg = state->attitude.roll;
  in->pitchDeg = state->attitude.pitch;
  g.gravZ = sensfusion6GetInvThrustCompensationForTilt();
  in->tiltDeg = autoTiltDegFromGravZ(g.gravZ);
  float rDes, pDes;
  controllerPidGetAttitudeDesired(&rDes, &pDes);
  in->attErrDeg = maxNan(fabsf(state->attitude.roll - rDes), fabsf(state->attitude.pitch - pDes));
  in->gyroDps[0] = sensors->gyro.x;
  in->gyroDps[1] = sensors->gyro.y;
  in->gyroDps[2] = sensors->gyro.z;
  in->accNormG = sqrtf(sensors->acc.x * sensors->acc.x + sensors->acc.y * sensors->acc.y +
                       sensors->acc.z * sensors->acc.z);
  in->accXG = sensors->acc.x;   /* a contact's side (body frame: x forward, y left) */
  in->accYG = sensors->acc.y;
  in->loopGapUs = loopGapUs;

  /* IDLE: the slow inputs on every full step (an ARM sees fresh ones). */
  stepFull(state, sensors, pending, idleStep || (loop % AUTONOMY_SLOW_EVERY) == 0, periodic || (idleStep && events));

  const autoFsmOut_t *out = &fout;
  if (out->thrust != AUTO_THR_NONE) {
    /* the interactive hover's turns: + clockwise, so the yaw angle (+ counter-clockwise) goes down */
    float yaw = yawHold;
    if (out->mv.yawOffDeg != 0.0f) {
      yaw = yawHold - out->mv.yawOffDeg;
      if (yaw > 180.0f) {
        yaw -= 360.0f;
      } else if (yaw < -180.0f) {
        yaw += 360.0f;
      }
    }
    autoBuildSetpoint(setpoint, autoSelectThrust(out->thrust, vo.thrust, out->thrustMax), yaw, xTaskGetTickCount());
  }
  g.thrustMax = autoBarrierMax(out->thrust, out->thrustMax);
  g.lastAction = (uint8_t)out->thrust;
  g.state = (uint8_t)out->state;

  /* Profile. */
  prof.fullSteps++;
  const bool active = out->state != AUTO_IDLE;
  if (active) {
    prof.activeLoops++;
    if (prof.wasActive) {
      prof.activeMs += nowMs - prof.lastActiveMs;
    }
    prof.lastActiveMs = nowMs;
  }
  prof.wasActive = active;
  if (timeIt) {
    const uint32_t us = (uint32_t)(esp_timer_get_time() - ts);
    if (idleStep) {
      prof.idleN++;
      prof.idleSumUs += us;
      if (us > prof.idleMaxUs) {
        prof.idleMaxUs = us;
      }
    } else {
      prof.activeN++;
      prof.activeSumUs += us;
      if (us > prof.activeMaxUs) {
        prof.activeMaxUs = us;
      }
    }
  }
  /* The stack high-water mark every STACK_CHECK_EVERY periodic IDLE steps,
   * and on the first IDLE step after a sortie, so a read right after it
   * includes the sortie's deepest call chain (final review RT-2). */
  const bool sortieEnded = idleStep && g.wasActive;
  g.wasActive = !idleStep;
  if (sortieEnded || (idleStep && periodic && (profResetReq || ++prof.idleTicks % STACK_CHECK_EVERY == 1))) {
    if (profResetReq) {
      profResetReq = false;
      memset(&prof, 0, sizeof(prof));
      prof.idleTicks = 1;
    }
    /* Lifetime minimum: a read after a sortie includes its deepest call chain. */
    prof.stackFreeMin = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
  }
}

float IRAM_ATTR autonomyThrustMax(void)
{
  return g.thrustMax;
}

void IRAM_ATTR autonomyNoteApplied(float thrust)
{
  g.applied = thrust;
}

/* ---- init ------------------------------------------------------------------ */

void autonomyInit(void)
{
  static bool isInit;
  if (isInit) {
    return;
  }
  portENTER_CRITICAL(&mbMux);
  autoMbInit(&mb);
  portEXIT_CRITICAL(&mbMux);
  autoFsmInit(&fsm);
  memset(&fin, 0, sizeof(fin));
  prof.stackFreeMin = UINT32_MAX;

  /* Settings: an absent, mistyped or out-of-range key uses the default. A key absent with its type is read once
   * more with its legacyType (autowait: the u16 item a drone stored before 2026-10-14 keeps its meaning; nothing is
   * written at boot). */
  for (int i = 0; i < AUTO_SET_COUNT; i++) {
    const autoSettingInfo_t *s = autoSettingInfo((autoSettingId_t)i);
    int32_t v = s->def;
    esp_err_t err = autoSettingStoreLoad(s->name, (uint8_t)s->type, &v);
    if (err == ESP_ERR_NVS_NOT_FOUND && s->legacyType != 0) {
      err = autoSettingStoreLoad(s->name, (uint8_t)s->legacyType, &v);
      if (err == ESP_OK) {
        static const char *const typeName[] = {"?", "u8", "u16", "i16", "u32"};
        DEBUG_PRINTI("setting %s: legacy %s %ld %s", s->name,
                     typeName[(unsigned)s->legacyType < 5u ? (unsigned)s->legacyType : 0u], (long)v, s->unit);
      }
    }
    if (err == ESP_OK && autoSettingInRange((autoSettingId_t)i, v)) {
      setting[i] = v;
    } else {
      setting[i] = s->def;
      if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        DEBUG_PRINTW("setting %s: stored value rejected (%s), default %ld", s->name, esp_err_to_name(err),
                     (long)s->def);
      } else if (err == ESP_OK) {
        DEBUG_PRINTW("setting %s: stored %ld out of range, default %ld", s->name, (long)v, (long)s->def);
      }
    }
  }
  vzParamsOk = autoSettingInRange(AUTO_SET_VZK, setting[AUTO_SET_VZK]) &&
               autoSettingInRange(AUTO_SET_VZBAIR, setting[AUTO_SET_VZBAIR]) &&
               autoSettingInRange(AUTO_SET_VZCAPLO, setting[AUTO_SET_VZCAPLO]);

  /* Reset-lock (section 6): only a battery power-on (RTC memory lost, no
   * magic) unlocks; the record is then rewritten for this boot. */
  const esp_reset_reason_t rr = esp_reset_reason();
  bootResetReason = (int)rr;
  rtcWasValid = autoRtcValid(&rtc);
  const bool locked = autoRtcBootLocked(rr == ESP_RST_POWERON, &rtc);
  autoRtcStartBoot(&rtc, locked);
  if (!locked) {
    resetLocked = false;
  }

  /* A first snapshot for STATUS before the stabilizer runs. */
  staging.state = AUTO_IDLE;
  staging.flags = resetLocked ? AUTO_STF_RESET_LOCKED : 0;
  staging.kPriorDiv10 = (uint16_t)(setting[AUTO_SET_VZK] / 10);
  staging.hoverWaitMs = autoSatMs16((uint32_t)setting[AUTO_SET_AUTOWAIT]);
  portENTER_CRITICAL(&mbMux);
  statusSnap = staging;
  portEXIT_CRITICAL(&mbMux);

  DEBUG_PRINTI("autonomy: reset-lock %s (reset %d, RTC record %s, boot %u), autoon=%ld vzK=%ld vmin=%ld",
               resetLocked ? "LOCKED" : "off", (int)rr, rtcWasValid ? "kept" : "new", (unsigned)rtc.boots,
               (long)setting[AUTO_SET_AUTOON], (long)setting[AUTO_SET_VZK], (long)setting[AUTO_SET_VMIN]);
  isInit = true;
}

/* ---- phone hook (CRTP RX task) --------------------------------------------- */

void autonomyNoteAppPacket(float rollDeg, float pitchDeg, uint16_t rawThrust)
{
  autoAppHookNote(&appHook, rollDeg, pitchDeg, rawThrust, T2M(xTaskGetTickCount()));
}

/* ---- link task ------------------------------------------------------------- */

bool autonomyPostCommand(const autoCmd_t *cmd, uint32_t *ticket)
{
  const uint32_t now = T2M(xTaskGetTickCount());
  portENTER_CRITICAL(&mbMux);
  const bool ok = autoMbPost(&mb, cmd, ticket) == AUTO_MB_POSTED;
  if (ok) {
    postMs = now;
  }
  portEXIT_CRITICAL(&mbMux);
  return ok;
}

bool autonomyTakeResult(uint32_t ticket, autoCmdResult_t *out)
{
  portENTER_CRITICAL(&mbMux);
  const bool ok = autoMbTakeResult(&mb, ticket, out);
  portEXIT_CRITICAL(&mbMux);
  return ok;
}

autoMbCancel_t autonomyCancelCommand(uint32_t ticket)
{
  portENTER_CRITICAL(&mbMux);
  const autoMbCancel_t r = autoMbCancel(&mb, ticket);
  portEXIT_CRITICAL(&mbMux);
  return r;
}

void autonomyKillFromLink(void)
{
  groundCoolGlueKill();   /* a cooling spin ends at the next loop (the STOP below already cut its motors) */
  if (motorsAnyDutyNonZero()) {
    motorsEmergencyStopNoWait();
  }
  __atomic_add_fetch(&killCount, 1u, __ATOMIC_SEQ_CST);
}

void autonomyNoteSession(void)
{
  sessionValid = true;
  __atomic_add_fetch(&sessionSeq, 1u, __ATOMIC_SEQ_CST);
}

void autonomyNoteHeartbeat(void)
{
  hbMs = T2M(xTaskGetTickCount());
  hbAny = true;
}

bool autonomyTakeCaptureRequest(uint32_t *reqId, uint8_t *n, uint16_t *gapMs)
{
  portENTER_CRITICAL(&mbMux);
  const bool ok = cap.pending;
  if (ok) {
    cap.pending = false;
    *reqId = cap.reqId;
    *n = cap.n;
    *gapMs = cap.gapMs;
  }
  portEXIT_CRITICAL(&mbMux);
  return ok;
}

void autonomyNoteCapture(uint32_t reqId, autoCapture_t progress, uint16_t photoId)
{
  portENTER_CRITICAL(&mbMux);
  if (reqId != 0 && reqId == cap.reqId) {
    cap.progress = (uint8_t)progress;
    cap.photoId = photoId;
  }
  portEXIT_CRITICAL(&mbMux);
}

bool IRAM_ATTR autonomyHoldBlocked(void)
{
  return holdBlock;
}

/* The take-off integrator hold (autonomy.h; user decision 2026-10-13,
 * evening): from vzReset until the controller's lift-off, released on the
 * lift-off step itself, and bounded: released too once the RAMP reaches its
 * plateau or the controller leaves RAMP without a lift-off (an abort LAND,
 * DONE, CUT), vert_ctrl's own "the drone may be up" cases, so an undetected
 * lift-off flies on P and D alone for at most the rest of the ramp, not
 * through the float and the abort's descent. Stabilizer task only
 * (controller_pid.c, after this loop's autonomyArbitrate): it reads the
 * glue's own stabilizer-task state, which the step that enters IDLE clears
 * (vzActive, vo). Flash code, and no IRAM in the glue: a flag set in
 * stepFull cost it 134 B (register spills). */
bool autonomyTakeoffIntegratorHold(void)
{
  return vzActive && !vo.liftedOff && vo.phase <= VZ_RAMP && !vzRampAtTop(&vz);
}

/* mvspeed (autonomy.h): the camera task's hold limits. */
uint16_t autonomyMvSpeedPct(void)
{
  return g.state != AUTO_IDLE ? sortieMvSpeed : (uint16_t)AUTO_MV_SPEED_DEFAULT_PCT;
}

/* mvfix (autonomy.h): the camera task's GAIN and BRAKE bits; 0 in IDLE. */
uint8_t autonomyMvFixBits(void)
{
  return g.state != AUTO_IDLE ? sortieMvFix : 0u;
}

/* The brake lean of a stopped fast MOVE for controller_pid's stale-hold branch (autonomy.h): the state machine's latest
 * motion outputs, read on the task that wrote them (the stabilizer's). Flash: the controller is flash code. */
bool autonomyBrakeLean(float *rollDeg, float *pitchDeg)
{
  if (sortieMvSpeed <= AUTO_MV_SPEED_DEFAULT_PCT || g.state != AUTO_HOVER || fout.mv.holdWait) {
    return false;
  }
  const float r = fout.mv.ffRollDeg, p = fout.mv.ffPitchDeg;
  if (r == 0.0f && p == 0.0f) {
    return false;   /* no brake running (the brake is the only lean outside an active motion) */
  }
  *rollDeg = r;
  *pitchDeg = p;
  return true;
}

void IRAM_ATTR autonomyGetHoldRef(autonomyHoldRef_t *out)
{
  portENTER_CRITICAL(&holdRefMux);
  *out = holdRef;
  portEXIT_CRITICAL(&holdRefMux);
}

bool autonomyCameraAwake(void)
{
  return camAwakeNow;
}

/* ---- any task ---------------------------------------------------------------- */

bool autonomyActive(void)
{
  return g.state != AUTO_IDLE;
}

autoState_t autonomyState(void)
{
  return (autoState_t)g.state;
}

uint16_t autonomySortieEvents(void)
{
  return autoFsmTrimEvents(&fsm);   /* (u16s written by the stabilizer task; the information-only HOLD_LIMIT left out:
                                       the trim learner as before it, docs/autonomy-a2b.md 27.4) */
}

bool autonomyResetLocked(void)
{
  return resetLocked;
}

uint32_t autonomyStatusSeq(void)
{
  return __atomic_load_n(&statusSeq, __ATOMIC_ACQUIRE);
}

uint32_t autonomyGetStatus(autoStatusMsg_t *out)
{
  portENTER_CRITICAL(&mbMux);
  *out = statusSnap;
  const uint32_t seq = statusSeq;
  portEXIT_CRITICAL(&mbMux);
  out->bootNonce = 0;
  /* the speed step (bytes 136-137, 2026-10-16): the NVS mvspeed, in IDLE the next ARM's, in a sortie the latched one
   * (no settings writer runs outside IDLE: the same value). Filled here, on the readers' side (flash), not in the
   * stabilizer's IRAM snapshot. */
  out->mvSpeedPct = (uint16_t)setting[AUTO_SET_MVSPEED];
  return seq;
}

static int16_t recI16(float v)
{
  if (!(v == v)) {
    return 0;
  }
  return (int16_t)lrintf(fminf(fmaxf(v, -32767.0f), 32767.0f));
}

/* Stabilizer task (the recorder, 100 Hz). */
void autonomyRecordFields(autoRecFields_t *out)
{
  if (g.state == AUTO_IDLE && g.lastAction == AUTO_THR_NONE && !vzActive) {
    memset(out, 0, sizeof(*out));
    return;
  }
  out->autoState = g.state;
  out->autoReason = (uint8_t)fout.reason;
  out->vzPhase = (uint8_t)vo.phase;
  out->vzHmm = recI16(vo.h * 1000.0f);
  out->vzVmms = recI16(vo.v * 1000.0f);
  out->vzThrust = vo.thrust > 0.0f ? (uint16_t)fminf(vo.thrust + 0.5f, 65535.0f) : 0;
  out->autoFlags = recFlags(&fout, &fin);
  out->mvState = (uint8_t)((uint8_t)fout.mv.state | ((uint8_t)fout.mv.kind << 4));
  out->mvRefLat = recI16(fout.mv.refLat * 10000.0f);
  out->mvRefFwd = recI16(fout.mv.refFwd * 10000.0f);
  out->mvVRef = recI16(fout.mv.vRef * 1000.0f);
  out->mvYawOff = recI16(fout.mv.yawOffDeg * 100.0f);
}

/* ---- flash writers ------------------------------------------------------------ */

bool autonomyFlashBegin(void)
{
  portENTER_CRITICAL(&mbMux);
  const bool ok = autoMbFlashBegin(&mb);
  portEXIT_CRITICAL(&mbMux);
  return ok;
}

void autonomyFlashEnd(void)
{
  portENTER_CRITICAL(&mbMux);
  autoMbFlashEnd(&mb);
  portEXIT_CRITICAL(&mbMux);
}

/* ---- settings writers (USB task, aux task) ------------------------------------ */

/* One settings writer at a time, inside the flash interlock, both taken
 * under one lock BEFORE anything changes: an ARM that came first (posted or
 * taken) makes the claim fail with nothing changed, and an ARM taken while the
 * claim is held is refused by G1 (flashBusy). The NVS write, its read-back and
 * the RAM copy all change inside the claim, so no sortie ever starts between
 * the NVS write and the RAM update, and the console and the aux task's vzK
 * save never interleave. The store brackets its own write again (nested). */
static bool settingClaim(void)
{
  portENTER_CRITICAL(&mbMux);
  const bool ok = !settingWriter && autoMbFlashBegin(&mb);
  if (ok) {
    settingWriter = true;
  }
  portEXIT_CRITICAL(&mbMux);
  return ok;
}

static void settingRelease(void)
{
  portENTER_CRITICAL(&mbMux);
  settingWriter = false;
  autoMbFlashEnd(&mb);
  portEXIT_CRITICAL(&mbMux);
}

/* ---- settings (USB task) ---------------------------------------------------- */

int32_t autonomySettingGet(autoSettingId_t id)
{
  return autoSettingInfo(id) != NULL ? setting[id] : 0;
}

bool autonomySettingSet(autoSettingId_t id, int32_t value, const char **why)
{
  const autoSettingInfo_t *s = autoSettingInfo(id);
  if (s == NULL || !autoSettingInRange(id, value)) {
    *why = "range";
    return false;
  }
  /* Active, an ARM in the mailbox, or the aux task storing a learned vzK. */
  if (autonomyActive() || !settingClaim()) {
    *why = "busy: autonomy active";
    return false;
  }
  /* Writes, commits and reads back (ESP_ERR_INVALID_STATE: an ARM was posted
   * after the claim; that ARM is refused, and nothing changed). A key whose
   * type changed (legacyType) is erased first, every item of it, so the old
   * item can never come back (power lost between the two: the key absent,
   * the default, the fail-safe side). */
  esp_err_t err = s->legacyType != 0 ? autoSettingStoreErase(s->name) : ESP_OK;
  const bool erased = s->legacyType != 0 && err == ESP_OK;
  if (err == ESP_OK) {
    err = autoSettingStoreSave(s->name, (uint8_t)s->type, value);
  }
  if (err == ESP_OK) {
    setting[id] = value;
    if (id == AUTO_SET_VZK) {
      savePending = false;   /* the console's value wins over a learned one not yet stored */
    }
  } else if (erased) {
    setting[id] = s->def;    /* the key is gone from NVS (the write failed after the erase): RAM follows it */
  }
  settingRelease();
  if (err == ESP_ERR_INVALID_STATE) {
    *why = "busy: autonomy active";
    return false;
  }
  if (err != ESP_OK) {
    DEBUG_PRINTW("setting %s: NVS save failed (%s)", s->name, esp_err_to_name(err));
    *why = "nvs";
    return false;
  }
  return true;
}

bool autonomySettingReset(autoSettingId_t id, const char **why)
{
  const autoSettingInfo_t *s = autoSettingInfo(id);
  if (s == NULL) {
    *why = "range";
    return false;
  }
  if (autonomyActive() || !settingClaim()) {
    *why = "busy: autonomy active";
    return false;
  }
  const esp_err_t err = autoSettingStoreErase(s->name);
  if (err == ESP_OK) {
    setting[id] = s->def;
    if (id == AUTO_SET_VZK) {
      savePending = false;
    }
  }
  settingRelease();
  if (err == ESP_ERR_INVALID_STATE) {
    *why = "busy: autonomy active";
    return false;
  }
  if (err != ESP_OK) {
    DEBUG_PRINTW("setting %s: NVS erase failed (%s)", s->name, esp_err_to_name(err));
    *why = "nvs";
    return false;
  }
  return true;
}

/* ---- reset-lock USB unlock (USB task: usb_diag.c 'auto unlock' only) ---------- */

/* User decision 2026-10-12 (evening): the USB console's 'auto unlock' is the
 * deliberate physical act after an unexpected reset, as a battery power-on
 * is. Inside the flash interlock: taken under the glue's lock only with the
 * published state IDLE and no ARM in the mailbox, and an ARM taken while it
 * is held is refused by G1, so no sortie starts while the flag and the
 * record change. In IDLE the stabilizer writes the RTC record only on a
 * transition, and the only transition out of IDLE is an accepted ARM, so it
 * cannot write the record meanwhile either (its CRC stays whole). */
bool autonomyUsbUnlock(bool *wasLocked, const char **why)
{
  *wasLocked = resetLocked;
  /* Active, or IDLE in the release hold (action ZERO for 600 ms after a sortie). */
  if (autonomyActive() || g.lastAction != AUTO_THR_NONE) {
    *why = "busy: autonomy active";
    return false;
  }
  if (motorsAnyDutyNonZero()) {
    *why = "a motor output is nonzero";
    return false;
  }
  if (!autonomyFlashBegin()) {
    *why = "busy: autonomy active";
    return false;
  }
  const bool was = resetLocked;
  bool noted = false;
  if (was) {
    resetLocked = false;
    noted = autoRtcNoteUsbUnlock(&rtc);
  }
  autonomyFlashEnd();
  *wasLocked = was;
  if (was) {
    /* A warning, so the crash log's warning ring (RTC memory, 'crash') keeps it across resets. */
    DEBUG_PRINTW("reset-lock cleared from the USB console ('auto unlock'; reset %d, boot %u, RTC record %s); "
                 "other locks unchanged", bootResetReason, (unsigned)rtc.boots, noted ? "noted" : "not valid");
  }
  return true;
}

/* ---- low-priority task (takeoff_aux, every 10 ms) ----------------------------- */

void autonomyPoll(void)
{
  /* G9's heap reading (heapFree): value, then its time, then the flag. */
  __atomic_store_n(&heapFree, (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), __ATOMIC_RELAXED);
  __atomic_store_n(&heapMs, T2M(xTaskGetTickCount()), __ATOMIC_RELEASE);
  __atomic_store_n(&heapAny, true, __ATOMIC_RELEASE);
  if (!savePending) {
    return;
  }
  const uint32_t now = T2M(xTaskGetTickCount());
  /* IDLE (not in the release hold), every duty 0, resting on the ground. */
  if ((int32_t)(now - kRetryMs) < 0 || autonomyActive() || g.lastAction != AUTO_THR_NONE ||
      motorsAnyDutyNonZero() || !stabilizerOnGround()) {
    return;
  }
  /* The console may be writing a setting, or an ARM came first: later. */
  if (!settingClaim()) {
    kRetryMs = now + K_SAVE_RETRY_MS;
    return;
  }
  const int32_t k = pendingK;
  esp_err_t err = ESP_OK;
  bool stored = false;
  if (!savePending) {
    /* a console vzK write cancelled it before the claim */
  } else if (!autoSettingInRange(AUTO_SET_VZK, k)) {
    savePending = false;
  } else {
    const autoSettingInfo_t *s = autoSettingInfo(AUTO_SET_VZK);
    err = autoSettingStoreSave(s->name, (uint8_t)s->type, k);
    if (err == ESP_OK) {
      setting[AUTO_SET_VZK] = k;
      savePending = false;
      stored = true;
    }
  }
  settingRelease();
  if (stored) {
    DEBUG_PRINTI("vzK %ld stored", (long)k);
  } else if (err != ESP_OK) {
    kRetryMs = now + K_SAVE_RETRY_MS;   /* refused by the interlock (an ARM came first) or NVS: later */
    if (err != ESP_ERR_INVALID_STATE) {
      DEBUG_PRINTW("vzK %ld not stored (%s); retrying", (long)k, esp_err_to_name(err));
    }
  }
}

/* ---- bench and diagnostics ---------------------------------------------------- */

bool autonomySetBenchSource(autoBenchSrc_t src)
{
  if (autonomyActive() || (src != AUTO_BENCH_SRC_SIM && src != AUTO_BENCH_SRC_IMU)) {
    return false;
  }
  benchSrc = src;
  return true;
}

autoBenchSrc_t autonomyBenchSource(void)
{
  return benchSrc;
}

void autonomyGetProfile(autonomyProfile_t *out)
{
  memset(out, 0, sizeof(*out));
  out->loops = g.loops;
  out->fullSteps = prof.fullSteps;
  out->idleMaxUs = prof.idleMaxUs;
  out->idleAvgUs10 = prof.idleN ? (uint32_t)(prof.idleSumUs * 10u / prof.idleN) : 0;
  out->activeMaxUs = prof.activeMaxUs;
  out->activeAvgUs10 = prof.activeN ? (uint32_t)(prof.activeSumUs * 10u / prof.activeN) : 0;
  out->fastMaxUs = prof.fastMaxUs;
  out->cmdLatencyMaxMs = prof.cmdLatMaxMs;
  out->activeLoops = prof.activeLoops;
  out->activeMs = prof.activeMs;
  out->stabStackFreeMin = prof.stackFreeMin;
  portENTER_CRITICAL(&mbMux);
  out->flashRefused = mb.flashRefused;
  out->armsBlocked = mb.armsBlocked;
  portEXIT_CRITICAL(&mbMux);
  out->shiftRefused = shiftRefused;
}

void autonomyProfileReset(void)
{
  profResetReq = true;   /* done by the stabilizer on its next periodic IDLE step */
}

void autonomyGetRtc(autonomyRtcView_t *out)
{
  memset(out, 0, sizeof(*out));
  out->locked = resetLocked;
  out->recordWasValid = rtcWasValid;
  out->resetReason = bootResetReason;
  out->prevState = rtc.prevState;
  out->prevReason = rtc.prevReason;
  out->prevMode = rtc.prevMode;
  out->prevLocked = rtc.prevLocked;
  out->prevTransitions = rtc.prevTransitions;
  const uint32_t t = rtc.transitions;   /* the stabilizer may append meanwhile: a diagnostic copy */
  out->transitions = t;
  out->n = (uint8_t)(t < AUTO_RTC_TRANSITIONS ? t : AUTO_RTC_TRANSITIONS);
  for (uint32_t i = 0; i < out->n; i++) {
    const autoRtcTransition_t *e = &rtc.trans[(t - out->n + i) % AUTO_RTC_TRANSITIONS];
    out->trans[i].ms = e->ms;
    out->trans[i].from = e->from;
    out->trans[i].to = e->to;
    out->trans[i].reason = e->reason;
    out->trans[i].vzReason = e->vzReason;
  }
}
