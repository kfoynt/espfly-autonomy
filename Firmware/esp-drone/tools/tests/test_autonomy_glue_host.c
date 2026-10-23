/*
 * Host regression test of the autonomy glue's per-loop logic
 * (components/core/crazyflie/modules/src/autonomy.c, docs/autonomy-a2b.md):
 * autonomy.c compiled with the stubs in tools/tests/autonomy_glue_host/stub
 * (ESP-IDF, FreeRTOS and the drone headers that are not host-clean; the
 * functions are defined below) and the real state machine, vertical
 * controller, mailbox and glue core. It emulates the stabilizer loop
 * (barrier 1: the override of the setpoint; barrier 2: autoClampThrust on
 * control.thrust), the link task (posts, heartbeat, photo sets), the phone,
 * the USB console (settings, the flash interlock) and the aux task
 * (autonomyPoll).
 *
 * Covered: DRY gives 0 at both barriers in every state (ARMED, TAKEOFF,
 * HOVER, LAND, KILLED and the release hold), also for a controller gone
 * wrong; the override's exact setpoint; the ARM refused (G1 LOCKS) while an
 * NVS write holds the flash interlock, while a camvz bench mode is on or the
 * camera height observer is paused (FLIGHT and BENCH-imu; DRY and BENCH-sim
 * are not affected); the sim plant chosen from the ARM command; the IDLE
 * decimation (10 Hz) and the immediate step on a command, KILL or session;
 * every (snapshot, seq) pair a reader can see belongs together (the
 * transition's snapshot is stored with its seq); settings written while an
 * ARM arrives (no sortie starts between the NVS write and the RAM copy; an
 * ARM in the mailbox refuses the write); the console and the aux task's
 * learned-vzK save never interleave; the sortie's vzK is the one of its ARM;
 * FLIGHT on a simulated plant through the real-input path, the learned vzK
 * stored once afterwards; the reset-lock and its USB unlock (refused while
 * active, with an ARM in the mailbox or a motor running); G9 on the aux task's heap reading
 * (too little, stale after the aux task stalls, or none: refused); LAND
 * COLLISION (a tap on the frame in DRY HOVER): the hold blocked from the
 * transition's own loop to the end of the LAND, the transition in the RTC
 * record and the recorder fields, both barriers 0.
 *
 * Every scenario runs in its own forked process, so autonomy.c's statics
 * start fresh (autonomyInit runs once per boot).
 *
 * Build and run from Firmware/esp-drone (the repo's state machine and
 * controller):
 *   C=components/core/crazyflie
 *   cc -std=gnu99 -O1 -Wall -Wextra -Itools/tests/autonomy_glue_host/stub -I$C/utils/interface \
 *      -I$C/modules/interface -I$C/hal/interface tools/tests/test_autonomy_glue_host.c \
 *      $C/modules/src/autonomy.c $C/utils/src/auto_glue_core.c $C/utils/src/auto_mailbox.c \
 *      $C/utils/src/auto_fsm.c $C/utils/src/vert_ctrl.c $C/utils/src/auto_proto.c \
 *      $C/utils/src/auto_motion.c -lm -o /tmp/test_autonomy_glue_host && /tmp/test_autonomy_glue_host
 * Another state machine and controller (a WP4 checkout): put its
 * utils/interface before $C/utils/interface and compile its auto_fsm.c and
 * vert_ctrl.c instead. One scenario only: /tmp/test_autonomy_glue_host <name>.
 * Lines starting with "info:" are measured numbers, not checks.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "glue_host.h"
#include "autonomy.h"
#include "cam_vz_glue.h"
#include "camera_flow.h"
#include "controller_pid.h"
#include "flight_rtc_log.h"
#include "motor_guard.h"
#include "sensfusion6.h"
#include "stabilizer.h"

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); \
                                                 printf("\n"); } } while (0)

/* ---- what the stubs report ------------------------------------------------- */

bool glueHostQuiet = true;
static uint32_t nowMs = 1;
static esp_reset_reason_t resetReason = ESP_RST_POWERON;
static float vbatStub = 4.05f;
static uint16_t gcapStub;
static bool estop;
static int prespinRuns, prespinStops, liftoffNotes, estopSets, estopClears, thrustLocks, noWaitStops;
static bool camSkipStub;
static uint8_t benchAirStub;   /* camVzGlueBenchAir() */
static bool pausedStub;        /* camVzGluePaused() */
static uint8_t whyBitsStub;    /* camVzGlueWhyBits(): why the camera height is out (AUTO_CB_*) */
static bool camOutStub;        /* camVzGlueVzInput(): the camera height unusable (frames lost) */
static int whyBitsCalls;
static int vzInputCalls;       /* camVzGlueVzInput(): the controller read the real camera height */
static float dutyThrust;       /* what the motors got this loop */
static size_t heapStub = 20000;   /* heap_caps_get_free_size(): what the aux task samples */
static float tapG;                /* added to the IMU's z (a tap on the frame) */

/* FLIGHT on a plant: the real-input path fed by a vzSim this test owns. */
static bool usePlant;
static vzSimState_t plant;
static vzSimConfig_t plantCfg;
static float plantV;
/* A palm under the plant (the palm catch, harness only): a unilateral spring-damper whose top rises at palmV until
 * the contact, then stops in 0.1 s while it lifts palmLift over 0.2 s (minimum jerk); forces on the plant after its
 * step, as test_interactive_sim.c's hand */
static struct {
  bool on;
  float top, v, lift, k, c, contactMs;
  bool held;
} palm;

/* The spinlock, and a reader that looks at the status at every release: the
 * snapshot and its seq must always belong together (the snapshot's
 * transition count moves with the seq), and the lock-free seq must equal the
 * one read under the lock. A seq++ in a critical section of its own (before
 * or after the snapshot copy) fails this. */
static int lockDepth;
static bool seqObserve, inObserver, seqBaseSet;
static uint32_t seqBase, transBase, seqObservations, seqMismatches;
/* One-shot hooks: another task running at a chosen point (inside a store's
 * own flash bracket, right after it, or at the next spinlock release). */
typedef void (*hook_t)(void);
static hook_t duringWrite, afterWrite, nextUnlock, beforeLock;

static void runHook(hook_t *h)
{
  const hook_t f = *h;
  *h = NULL;
  if (f) {
    f();
  }
}

void glueHostLock(portMUX_TYPE *m)
{
  (void)m;
  if (lockDepth == 0) {
    runHook(&beforeLock);   /* one-shot, before a lock is taken (scenRace) */
  }
  if (++lockDepth != 1) {
    printf("FAIL: nested spinlock\n");
    fflush(stdout);
    _exit(2);
  }
}

void glueHostUnlock(portMUX_TYPE *m)
{
  (void)m;
  lockDepth--;
  if (seqObserve && !inObserver) {
    inObserver = true;
    autoStatusMsg_t s;
    const uint32_t seq = autonomyGetStatus(&s);
    const uint32_t lockFree = autonomyStatusSeq();
    if (!seqBaseSet) {
      seqBaseSet = true;
      seqBase = seq;
      transBase = s.transitions;
    }
    seqObservations++;
    if (lockFree != seq || seq - seqBase != s.transitions - transBase) {
      seqMismatches++;
    }
    inObserver = false;
  }
  if (lockDepth == 0 && !inObserver) {
    runHook(&nextUnlock);
  }
}

TickType_t xTaskGetTickCount(void) { return nowMs; }
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t t) { (void)t; return 1234; }
int64_t esp_timer_get_time(void) { return (int64_t)nowMs * 1000; }
static uint32_t rnd = 77;
uint32_t esp_random(void) { rnd = rnd * 1103515245u + 12345u; return rnd; }
size_t heap_caps_get_free_size(uint32_t caps) { (void)caps; return heapStub; }
const char *esp_err_to_name(esp_err_t e) { (void)e; return "ERR"; }
esp_reset_reason_t esp_reset_reason(void) { return resetReason; }

volatile uint32_t cameraFlowMotionLagUsV; /* legacy fake camera: default reference lag */
bool cameraFlowGetLatest(cameraFlowSample_t *out)
{
  memset(out, 0, sizeof(*out));
  out->seq = nowMs / 59;
  out->frameTimeUs = (uint64_t)(nowMs / 59) * 59000;
  out->ok = true;
  return true;
}
cameraFlowState_t cameraFlowState(void) { return CAMERA_FLOW_RUNNING; }
float cameraFlowFps(void) { return 16.9f; }
void cameraFlowGetHold(float *r, float *p, bool *e, uint64_t *t)
{
  *r = *p = 0.0f;
  *e = usePlant && !plant.onGround;
  *t = (uint64_t)nowMs * 1000;
}
bool cameraFlowHoldCoasting(void) { return false; }
bool cameraFlowHoldEnabled(void) { return true; }
static uint8_t sceneResStub = CAM_HOLD_SCENE_RES_X10_DEFAULT;   /* (holdres as the ARM latched it) */
void cameraFlowSetSceneResidual(uint8_t x10) { sceneResStub = x10; }
uint8_t cameraFlowSceneResidual(void) { return sceneResStub; }
static uint16_t moveLatStub = 100;   /* (mvlatk as the ARM latched it, clamped as camera_flow.c does) */
void cameraFlowSetMoveLatGain(uint16_t pct) { moveLatStub = pct < 100u ? 100u : (pct > 400u ? 400u : pct); }
uint16_t cameraFlowMoveLatGain(void) { return moveLatStub; }
void cameraFlowSetTakeoffStandbySkip(bool skip) { camSkipStub = skip; }
/* Interactive hover: the camera applies the hold's reference epoch at once and
 * tracks it (ideal), the scene 2.0 m ahead with a tracked G. */
static bool sceneTrackedStub = true;
static float sceneDStub = 2.0f;
static uint32_t sceneResetsStub;
static bool guessAllowedStub;
static int guessAllowedOn;   /* loops with the guess allowed */
volatile uint8_t cameraFlowHoldLimitBitsV;   /* the camera's hold-at-limit bits (camera_flow.h; 0: never at it) */
volatile uint8_t cameraFlowHoldMovingV;      /* the hold's measured rates not still (camera_flow.h; 0: still) */
void cameraFlowGetHoldProgress(float *pLat, float *pFwd, uint32_t *epoch)
{
  autonomyHoldRef_t r;
  autonomyGetHoldRef(&r);
  *pLat = r.refLat;
  *pFwd = r.refFwd;
  *epoch = r.epoch;
}
bool camVzGlueScene(float *dM, bool *tracked, uint32_t *resets)
{
  *dM = sceneDStub;
  *tracked = sceneTrackedStub;
  *resets = sceneResetsStub;
  return true;
}
void camVzGlueSetGuessAllowed(bool allowed)
{
  guessAllowedStub = allowed;
  guessAllowedOn += allowed;
}
volatile float camVzGlueFwdLnIn;   /* cam_vz_glue.c: what the glue copies into each frame (camVzGlueSetFwdLn) */
volatile bool camVzGlueMotionIn;   /* cam_vz_glue.c: a commanded motion runs (camVzGlueSetMotion, 2026-10-15) */

/* As cam_vz_glue.c: unusable without an observer, in a bench mode or paused. */
bool camVzGlueVzInput(float vzG0, vzInput_t *in)
{
  (void)vzG0;
  vzInputCalls++;
  const bool valid = usePlant && benchAirStub == CVS_BENCH_OFF && !pausedStub && !camOutStub;
  in->camHealthy = valid && plant.camHealthy;
  in->camVzValid = valid && plant.camVzValid;
  in->camVz = valid ? plant.camVz : 0.0f;
  in->camBias = valid ? plant.camBias : 0.0f;
  return usePlant;
}
void camVzGlueSetLiftoffHint(bool liftedOff) { (void)liftedOff; }
uint8_t camVzGlueBenchAir(void) { return benchAirStub; }
bool camVzGluePaused(void) { return pausedStub; }
uint8_t camVzGlueWhyBits(void)
{
  whyBitsCalls++;
  return whyBitsStub;
}

void controllerPidGetAttitudeDesired(float *rollDeg, float *pitchDeg) { *rollDeg = *pitchDeg = 0.0f; }
void crtpCommanderRpytLockThrust(void) { thrustLocks++; }
uint16_t flightRtcLogLiveVbatMin(void) { return (uint16_t)(vbatStub * 1000.0f); }

ledseqContext_t seq_autoPrespin;
bool ledseqRun(ledseqContext_t *c) { (void)c; prespinRuns++; return true; }
bool ledseqStop(ledseqContext_t *c) { (void)c; prespinStops++; return true; }
bool motorGuardEnabled(void) { return true; }
bool motorGuardTripped(void) { return false; }
bool motorsAnyDutyNonZero(void) { return dutyThrust > 0.0f; }
void motorsEmergencyStopNoWait(void) { noWaitStops++; }
bool motorsReadyNoLock(void) { return true; }
bool motorsIsInhibited(void) { return false; }
float pmGetBatteryVoltage(void) { return usePlant ? plantV : vbatStub; }
float sensfusion6GetAccZ(const float ax, const float ay, const float az) { (void)ax; (void)ay; return az; }
float sensfusion6GetInvThrustCompensationForTilt(void) { return 1.0f; }
bool stabilizerOnGround(void) { return usePlant ? (plant.onGround && dutyThrust == 0.0f) : true; }
static int accTcLatches;   /* the accelerometer compensation's reference at ARM (accel_tempco.h) */
void sensorsMpu6050AccTcLatch(bool coolFollows) { (void)coolFollows; accTcLatches++; }
void groundCoolGlueKill(void) {}   /* the ground cooling spin ends on a link kill (ground_cool_glue.h) */
bool stabilizerRestartLocked(void) { return false; }
volatile uint32_t stabilizerLastFlightMotorMsWord;   /* 0: no heat soak, G2's 20 s as before (auto_fsm.h AUTO_SOAK_*) */
bool stabilizerEmergencyStopActive(void) { return estop; }
void stabilizerSetEmergencyStop(void) { estop = true; estopSets++; }
void stabilizerResetEmergencyStop(void) { estop = false; estopClears++; }
uint16_t stabilizerGetThrustCap(void) { return gcapStub; }
void stabilizerNoteLiftoff(void) { liftoffNotes++; }

/* ---- NVS: a small map, written as imu_level_cal_store.c does ---------------- */

/* Typed as ESP-IDF 5.0.7's NVS (docs/autonomy-a2b.md 18.2): an item is a key AND a type; a read with a type no
 * item of the key has is ESP_ERR_NVS_NOT_FOUND (Storage::findItem turns the page's TYPE_MISMATCH into it); a write
 * adds or replaces the item of its own type and leaves an item of another type alone; nvs_erase_key removes one
 * item of any type (the store's erase loops, autoSettingStoreErase). */
static struct {
  char key[16];
  int32_t v;
  uint8_t t;
  bool used;
} nvs[16];
static int flashWrites;

static int nvsFindT(const char *key, int t)   /* t < 0: any type */
{
  for (int i = 0; i < 16; i++) {
    if (nvs[i].used && strcmp(nvs[i].key, key) == 0 && (t < 0 || nvs[i].t == (uint8_t)t)) {
      return i;
    }
  }
  return -1;
}

static int nvsFind(const char *key)
{
  return nvsFindT(key, -1);
}

static int nvsCount(const char *key)
{
  int n = 0;
  for (int i = 0; i < 16; i++) {
    n += nvs[i].used && strcmp(nvs[i].key, key) == 0;
  }
  return n;
}

static void nvsPut(const char *key, uint8_t t, int32_t v)
{
  int i = nvsFindT(key, t);
  for (int j = 0; i < 0 && j < 16; j++) {
    if (!nvs[j].used) {
      i = j;
    }
  }
  snprintf(nvs[i].key, sizeof(nvs[i].key), "%s", key);
  nvs[i].v = v;
  nvs[i].t = t;
  nvs[i].used = true;
}

static int32_t nvsGet(const char *key)
{
  const int i = nvsFind(key);
  return i < 0 ? -1 : nvs[i].v;
}

static int32_t nvsGetT(const char *key, uint8_t t)
{
  const int i = nvsFindT(key, t);
  return i < 0 ? -1 : nvs[i].v;
}

esp_err_t autoSettingStoreLoad(const char *key, uint8_t t, int32_t *v)
{
  const int i = nvsFindT(key, t);
  if (i < 0) {
    return ESP_ERR_NVS_NOT_FOUND;   /* absent, or stored with another type only */
  }
  *v = nvs[i].v;
  return ESP_OK;
}

esp_err_t autoSettingStoreSave(const char *key, uint8_t t, int32_t v)
{
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  runHook(&duringWrite);
  nvsPut(key, t, v);
  flashWrites++;
  autonomyFlashEnd();
  runHook(&afterWrite);   /* where the real store reads the value back */
  return ESP_OK;
}

esp_err_t autoSettingStoreErase(const char *key)
{
  if (!autonomyFlashBegin()) {
    return ESP_ERR_INVALID_STATE;
  }
  runHook(&duringWrite);
  for (int k = 0; k < 4; k++) {   /* every item of the key, any type */
    const int i = nvsFind(key);
    if (i < 0) {
      break;
    }
    nvs[i].used = false;
  }
  flashWrites++;
  autonomyFlashEnd();
  runHook(&afterWrite);
  return ESP_OK;
}

/* ---- the emulated tasks ----------------------------------------------------- */

#define NSTATES 6
static setpoint_t sp;
static float appRoll;
static uint16_t appThrust;
static bool pollOn = true;   /* takeoff_aux: autonomyPoll() every 10 ms (G9's heap reading, the vzK save) */
static float maxB1[NSTATES], maxB2[NSTATES], maxRogue[NSTATES];
static float maxB1Hold, maxB2Hold, maxRogueHold;   /* IDLE release hold (action ZERO) */
static bool seenState[NSTATES], seenHold;
static int overrideBad, overrideLoops, manualTouched;
static bool yawFree;         /* the interactive hover turns: the setpoint yaw is checked by the scenario */
static float spYaw = 33.0f;  /* the override's latest yaw */
static bool camHeightSeen;

/* The commander's one-slot setpoint queue (commander.c): the phone's latest
 * setpoint stays there after the phone goes silent; commanderGetSetpoint
 * keeps its thrust (roll/pitch levelled after 500 ms) until it is 2000 ms
 * old, then returns the null setpoint. commanderDropSetpoint() empties it. */
static bool appOnline = true;
static setpoint_t cmdQ;
static uint32_t cmdQMs;
static bool cmdQNull = true;
static int setpointDrops;
void commanderDropSetpoint(void)
{
  cmdQNull = true;
  setpointDrops++;
}

/* The take-off integrator hold (autonomyTakeoffIntegratorHold, read by
 * controller_pid.c), per loop (scenario ihold). */
static bool ihTrack;
static int ihHeldIdle, ihHeldAfterLift, ihFreeBeforeLift, ihHeldHigh, ihHeldTotal, ihTakeoffSpin, ihHighBeforeLift;
/* the bound (a lift-off never latched): the first release before a lift-off, what held after it, the thrust then */
static bool ihPrevHeld, ihReleased;
static int ihHeldAfterRelease, ihFreeBeforeRelease, ihHeldLand;
static float ihRelThrust, ihMaxRampThrust;

static void loopOnce(void)
{
  /* the phone (CRTP RX, 50 Hz) */
  if (appOnline && nowMs % 20 == 0) {
    autonomyNoteAppPacket(appRoll, 0.0f, appThrust);
  }
  /* the commander's setpoint: manual flight */
  if (appOnline) {
    memset(&cmdQ, 0, sizeof(cmdQ));
    cmdQ.thrust = appThrust >= 1000 ? appThrust : 0;
    cmdQ.attitude.roll = appRoll;
    cmdQ.mode.roll = cmdQ.mode.pitch = modeAbs;
    cmdQ.mode.yaw = modeVelocity;
    cmdQMs = nowMs;
    cmdQNull = false;
  }
  memset(&sp, 0, sizeof(sp));
  if (!cmdQNull && nowMs - cmdQMs <= 2000) {
    sp = cmdQ;
    if (nowMs - cmdQMs > 500) {
      sp.attitude.roll = sp.attitude.pitch = 0.0f;   /* levelled, thrust kept */
    }
  }
  const setpoint_t manual = sp;
  sensorData_t sd;
  memset(&sd, 0, sizeof(sd));
  sd.acc.z = (usePlant ? (plant.onGround && plant.jolt < 0.01f ? 1.0f : plant.accZ) : 1.0f) + tapG;
  sd.interruptTimestamp = (uint64_t)nowMs * 1000;
  state_t st;
  memset(&st, 0, sizeof(st));
  st.attitude.yaw = 33.0f;

  autonomyArbitrate(&sp, &sd, &st, nowMs, 1000);

  const autoState_t s = autonomyState();
  const float tmax = autonomyThrustMax();
  const bool untouched = memcmp(&sp, &manual, sizeof(sp)) == 0;
  if (s == AUTO_IDLE && tmax == AUTO_THRUST_MAX_FREE) {
    if (!untouched) {
      manualTouched++;
    }
  } else {
    overrideLoops++;
    const float zero = 0.0f;
    if (memcmp(&sp.attitude.roll, &zero, sizeof(zero)) != 0 || memcmp(&sp.attitude.pitch, &zero, sizeof(zero)) != 0 ||
        sp.mode.x != modeDisable || sp.mode.y != modeDisable || sp.mode.z != modeDisable ||
        sp.mode.roll != modeAbs || sp.mode.pitch != modeAbs || sp.mode.yaw != modeAbs ||
        (!yawFree && sp.attitude.yaw != 33.0f)) {
      overrideBad++;
    }
    spYaw = sp.attitude.yaw;
  }
  /* the stabilizer after the dispatcher: gcap only lowers */
  if (gcapStub > 0 && sp.thrust > gcapStub) {
    sp.thrust = gcapStub;
  }
  const float b1 = sp.thrust;
  const float b2 = autoClampThrust(sp.thrust, tmax);   /* the controller passes the thrust through */
  const float rogue = autoClampThrust(40000.0f, tmax); /* a controller gone wrong */
  if (!(s == AUTO_IDLE && tmax == AUTO_THRUST_MAX_FREE)) {
    float *m1 = s == AUTO_IDLE ? &maxB1Hold : &maxB1[s];
    float *m2 = s == AUTO_IDLE ? &maxB2Hold : &maxB2[s];
    float *mr = s == AUTO_IDLE ? &maxRogueHold : &maxRogue[s];
    if (s == AUTO_IDLE) {
      seenHold = true;
    } else {
      seenState[s] = true;
    }
    *m1 = fmaxf(*m1, b1);
    *m2 = fmaxf(*m2, b2);
    *mr = fmaxf(*mr, rogue);
  }
  dutyThrust = estop ? 0.0f : b2;
  autonomyNoteApplied(dutyThrust);
  if (ihTrack) {
    /* what controller_pid.c reads this loop (autonomyArbitrate runs before controller()) */
    const bool h = autonomyTakeoffIntegratorHold();
    ihHeldTotal += h;
    ihHeldIdle += h && s == AUTO_IDLE;
    ihHeldAfterLift += h && liftoffNotes > 0;
    ihFreeBeforeLift += !h && s == AUTO_TAKEOFF && dutyThrust > 0.0f && liftoffNotes == 0;
    ihTakeoffSpin += s == AUTO_TAKEOFF && dutyThrust > 0.0f && liftoffNotes == 0;
    ihHeldHigh += h && dutyThrust >= 26000.0f;
    ihHighBeforeLift += dutyThrust >= 26000.0f && liftoffNotes == 0 && s != AUTO_IDLE;
    if (ihPrevHeld && !h && liftoffNotes == 0 && s != AUTO_IDLE && !ihReleased) {
      ihReleased = true;
      ihRelThrust = dutyThrust;
    }
    ihHeldAfterRelease += h && ihReleased;
    ihFreeBeforeRelease += !h && !ihReleased && s == AUTO_TAKEOFF && dutyThrust > 0.0f && liftoffNotes == 0;
    ihHeldLand += h && s == AUTO_LAND;
    if (s == AUTO_TAKEOFF && liftoffNotes == 0) {
      ihMaxRampThrust = fmaxf(ihMaxRampThrust, dutyThrust);
    }
    ihPrevHeld = h;
  }
  if (usePlant) {
    vzSimStep(&plant, &plantCfg, dutyThrust, 0.001f);
    if (nowMs % 100 == 0) {
      plantV = plant.vbat;
    }
    if (palm.on) {
      const float tc = palm.contactMs >= 0.0f ? ((float)nowMs - palm.contactMs) * 0.001f : -1.0f;
      float vp = palm.v;
      if (tc >= 0.0f) {
        vp = palm.v * fmaxf(0.0f, 1.0f - tc / 0.1f);
        if (palm.lift > 0.0f && tc < 0.2f) {
          const float u = tc / 0.2f;
          vp += palm.lift / 0.2f * 30.0f * u * u * (1.0f - u) * (1.0f - u);
        }
      }
      palm.top += vp * 0.001f;
      const float pen = palm.top - plant.h;
      const float n = pen > 0.0f ? fmaxf(0.0f, (palm.k * pen + palm.c * (vp - plant.v)) / (0.028f * VZ_G)) : 0.0f;
      palm.held = n > 0.0f;
      if (palm.held && palm.contactMs < 0.0f) {
        palm.contactMs = (float)nowMs;
      }
      if (palm.held && !plant.onGround) {
        plant.v += VZ_G * n * 0.001f;
        plant.h += VZ_G * n * 0.001f * 0.001f;
        plant.accZ += n;
      }
    }
  }
  if (pollOn && nowMs % 10 == 0) {
    autonomyPoll();
  }
  nowMs++;
}

/* The link's background: the heartbeat at 10 Hz and the photo sets (a set of
 * n frames gap apart is done (n - 1) x gap + one frame after it starts). */
static uint32_t capReq, capDoneAt;
static int captures;

static void linkBackground(void)
{
  if (nowMs % 100 == 0) {
    autonomyNoteHeartbeat();
  }
  uint32_t req;
  uint8_t n;
  uint16_t gap;
  if (autonomyTakeCaptureRequest(&req, &n, &gap)) {
    capReq = req;
    capDoneAt = nowMs + (uint32_t)(n - 1) * gap + 60;
    autonomyNoteCapture(req, AUTO_CAP_BUSY, (uint16_t)(100 + req));
    captures++;
  }
  if (capReq && nowMs == capDoneAt) {
    autonomyNoteCapture(capReq, AUTO_CAP_DONE, (uint16_t)(100 + capReq));
  }
}

static void run(uint32_t ms)
{
  for (uint32_t i = 0; i < ms; i++) {
    linkBackground();
    loopOnce();
  }
}

static bool runUntil(autoState_t s, uint32_t maxMs)
{
  for (uint32_t i = 0; i < maxMs && autonomyState() != s; i++) {
    linkBackground();
    loopOnce();
  }
  return autonomyState() == s;
}

static uint16_t takeoffCm = 40;   /* TAKEOFF heightCm of makeCmd() */

static autoCmd_t makeCmd(autoCmdType_t type, uint8_t mode, uint32_t token)
{
  autoCmd_t c;
  memset(&c, 0, sizeof(c));
  c.type = type;
  c.mode = mode;
  c.token = token;
  if (type == AUTO_CMD_TAKEOFF) {
    c.heightCm = takeoffCm;
    c.photoN = 1;   /* the Mac tool's default sortie (integrator decision D3) */
    c.photoGapMs = 300;
  }
  if (type == AUTO_CMD_PHOTO) {
    c.photoN = 1;
    c.photoGapMs = 150;
  }
  return c;
}

/* Posts like the link task and steps the loop until the result is there. */
static bool command(autoCmdType_t type, uint8_t mode, uint32_t token, autoCmdResult_t *r, int maxLoops)
{
  const autoCmd_t c = makeCmd(type, mode, token);
  uint32_t t;
  if (!autonomyPostCommand(&c, &t)) {
    return false;
  }
  for (int i = 0; i < maxLoops; i++) {
    loopOnce();
    if (autonomyTakeResult(t, r)) {
      return true;
    }
  }
  return false;
}

/* The consent gesture on the phone: roll pushed and released, then 1.1 s without input. */
static void gesture(void)
{
  appRoll = 12.0f;
  run(100);
  appRoll = 0.0f;
  run(1100);
}

static autoCmdResult_t arm(autoMode_t mode)
{
  autoCmdResult_t r;
  memset(&r, 0xff, sizeof(r));
  gesture();
  if (!command(AUTO_CMD_ARM, (uint8_t)mode, 0, &r, 3)) {
    r.result = (autoResult_t)0xff;
  }
  return r;
}

static bool refusedLocks(const autoCmdResult_t *r)
{
  return r->result == AUTO_R_GUARDS && (r->detail & AUTO_G_LOCKS) != 0 && autonomyState() == AUTO_IDLE;
}

/* ARMED -> IDLE by DISARM, then past the release hold. */
static void disarm(void)
{
  autoCmdResult_t r;
  command(AUTO_CMD_DISARM, 0, 0, &r, 3);
  runUntil(AUTO_IDLE, 1000);
  run(700);
}

/* A KILL, then IDLE, then past the 2 s kill latch. */
static void killAndWait(void)
{
  autonomyKillFromLink();
  runUntil(AUTO_IDLE, 5000);
  run(2100);
}

static uint8_t statusFlags(void)
{
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  return m.flags;
}

static void boot(esp_reset_reason_t rr)
{
  resetReason = rr;
  nvsPut("autoon", 1, 1);   /* autoon 1 stored */
  autonomyInit();
}

/* Boot, 4 s of manual IDLE, a Mac session. */
static void bootToSession(esp_reset_reason_t rr)
{
  boot(rr);
  run(4000);
  autonomyNoteSession();
  run(500);
}

/* ---- scenarios -------------------------------------------------------------- */

/* IDLE: the state machine steps every 100th loop, and at once on a command,
 * a KILL or a new session; manual flight is untouched. */
static void scenIdle(void)
{
  boot(ESP_RST_POWERON);
  CHECK(autonomyThrustMax() == AUTO_THRUST_MAX_FREE, "barrier 2 free before the first step");
  CHECK(autonomySettingGet(AUTO_SET_AUTOON) == 1 && autonomySettingGet(AUTO_SET_AUTOWAIT) == 5000, "settings loaded");
  autonomyProfile_t p0, p1;
  run(4000);
  autonomyGetProfile(&p1);
  CHECK(p1.loops == 4000 && p1.fullSteps == 40, "IDLE: 40 full steps in 4000 loops (%u in %u)", (unsigned)p1.fullSteps,
        (unsigned)p1.loops);
  run(37);
  autonomyGetProfile(&p0);
  autoCmdResult_t r;
  CHECK(command(AUTO_CMD_DISARM, 0, 0, &r, 1), "a command in IDLE is answered on the next loop");
  autonomyGetProfile(&p1);
  CHECK(p1.fullSteps == p0.fullSteps + 1, "a command: one extra full step (%u)", (unsigned)(p1.fullSteps - p0.fullSteps));
  run(20);
  autonomyGetProfile(&p0);
  autonomyKillFromLink();
  loopOnce();
  autonomyGetProfile(&p1);
  CHECK(p1.fullSteps == p0.fullSteps + 1, "a KILL: stepped at once");
  run(20);
  autonomyGetProfile(&p0);
  autonomyNoteSession();
  loopOnce();
  autonomyGetProfile(&p1);
  CHECK(p1.fullSteps == p0.fullSteps + 1, "a session: stepped at once");
  autonomyGetProfile(&p0);
  run(1000);
  autonomyGetProfile(&p1);
  CHECK(p1.fullSteps == p0.fullSteps + 10, "back to 10 Hz (%u in 1000 loops)", (unsigned)(p1.fullSteps - p0.fullSteps));
  appThrust = 30000;
  run(200);
  appThrust = 0;
  run(100);
  CHECK(manualTouched == 0 && overrideLoops == 0, "manual setpoint untouched on every IDLE loop (%d)", manualTouched);
  CHECK(autonomyThrustMax() == AUTO_THRUST_MAX_FREE && autonomyState() == AUTO_IDLE, "IDLE, barrier 2 free");
  CHECK(autonomyFlashBegin(), "flash allowed in IDLE");
  autonomyFlashEnd();
}

/* DRY: three sorties (photos and the Mac's LAND at the wait's end, a Mac
 * KILL, the phone stick), 0 at both barriers in every state; the status seq
 * observer on. */
static void scenDry(void)
{
  seqObserve = true;
  bootToSession(ESP_RST_POWERON);
  autoStatusMsg_t m;
  uint32_t seq0 = autonomyGetStatus(&m);

  /* 1: TAKEOFF, HOVER, photos, PHOTO while waiting, the Mac's LAND at the wait's end (HOVER_TIMEOUT until
   * 2026-10-15: the state machine lands on no clock now), TOUCHDOWN */
  autoCmdResult_t r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK && r.state == AUTO_ARMED, "DRY ARM ok (%d 0x%x)", r.result, (unsigned)r.detail);
  const uint32_t token = r.detail;
  CHECK(autonomyStatusSeq() == seq0 + 1, "one transition, seq +1");
  autonomyGetStatus(&m);
  CHECK(m.state == AUTO_ARMED && m.mode == AUTO_MODE_DRY, "snapshot ARMED DRY");
  CHECK(!autonomyFlashBegin(), "flash refused while ARMED");
  const char *why = "";
  CHECK(!autonomySettingSet(AUTO_SET_AUTOWAIT, 4000, &why) && strcmp(why, "busy: autonomy active") == 0, "set busy");
  CHECK(!autonomySetBenchSource(AUTO_BENCH_SRC_IMU), "bench source refused while active");
  CHECK(!camSkipStub, "DRY: no camera standby skip");
  run(100);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, token, &r, 3) && r.result == AUTO_R_OK, "TAKEOFF ok (%d)", r.result);
  run(500);
  CHECK(prespinRuns == 1 && autonomyState() == AUTO_TAKEOFF && (statusFlags() & AUTO_STF_PRESPIN), "pre-spin LED");
  CHECK(runUntil(AUTO_HOVER, 8000), "HOVER reached (state %d)", autonomyState());
  CHECK(prespinStops >= 1, "pre-spin LED off");
  autonomyGetStatus(&m);
  CHECK(m.flags & AUTO_STF_SIM, "sim flag in HOVER");
  CHECK(liftoffNotes == 0, "DRY: no lift-off note (%d)", liftoffNotes);
  autoRecFields_t rf;
  autonomyRecordFields(&rf);
  CHECK(rf.autoState == AUTO_HOVER && rf.vzPhase >= VZ_CLIMB && (rf.autoFlags & AUTO_STF_SIM), "recorder fields");
  for (int i = 0; i < 4000 && !(m.flags & AUTO_STF_WAITING); i++) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
  }
  CHECK(m.flags & AUTO_STF_WAITING, "waiting after the photo (captures %d)", captures);
  CHECK(captures == 1 && m.capture == AUTO_CAP_DONE && m.photoId == 101, "capture reported (%d %u %u)", captures,
        m.capture, m.photoId);
  CHECK(m.hoverWaitMs == 5000, "hoverWaitMs latched (%u)", m.hoverWaitMs);
  run(1000);
  CHECK(command(AUTO_CMD_PHOTO, 0, 0, &r, 3) && r.result == AUTO_R_OK, "PHOTO while waiting ok (%d)", r.result);
  run(800);
  CHECK(captures == 2, "second set requested");
  run(3200);
  autonomyGetStatus(&m);
  CHECK(m.state == AUTO_HOVER && (m.flags & AUTO_STF_WAITING) && m.waitLeftMs == 0, "5.0 s into the wait: still "
        "HOVER, waiting, waitLeftMs 0 (the wait is information: %u)", m.waitLeftMs);
  CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK && autonomyState() == AUTO_LAND, "the Mac's LAND");
  autonomyGetStatus(&m);
  CHECK(m.reason == AUTO_RS_LAND_CMD, "LAND_CMD (%d)", m.reason);
  CHECK(runUntil(AUTO_IDLE, 10000), "IDLE");
  autonomyGetStatus(&m);
  CHECK(m.reason == AUTO_RS_TOUCHDOWN, "TOUCHDOWN (%d)", m.reason);
  run(700);
  CHECK(autonomyThrustMax() == AUTO_THRUST_MAX_FREE, "free again after the release hold");
  CHECK(autonomyFlashBegin(), "flash allowed again in IDLE");
  autonomyFlashEnd();
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  CHECK(rv.transitions == m.transitions && rv.n == 5 && rv.trans[rv.n - 1].to == AUTO_IDLE, "RTC ring (%u, n %u)",
        (unsigned)rv.transitions, rv.n);

  /* 2: a Mac KILL in HOVER */
  run(3000);
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "DRY ARM 2 ok (%d 0x%x)", r.result, (unsigned)r.detail);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "TAKEOFF 2 ok");
  CHECK(runUntil(AUTO_HOVER, 8000), "HOVER 2");
  autonomyKillFromLink();
  run(2);
  CHECK(autonomyState() == AUTO_KILLED && estopSets == 1, "KILLED, e-stop set");
  run(500);
  CHECK(runUntil(AUTO_IDLE, 4000) && estopClears == 1, "KILLED -> IDLE, e-stop cleared (%d)", autonomyState());

  /* 3: the phone stick in HOVER */
  run(3000);
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "DRY ARM 3 ok (%d 0x%x)", r.result, (unsigned)r.detail);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "TAKEOFF 3 ok");
  CHECK(runUntil(AUTO_HOVER, 8000), "HOVER 3");
  run(500);
  appThrust = 2000;
  run(25);
  appThrust = 0;
  autonomyGetStatus(&m);
  CHECK(autonomyState() == AUTO_LAND && m.reason == AUTO_RS_APP_INPUT, "phone: LAND APP_INPUT (%d %d)",
        autonomyState(), m.reason);
  CHECK(runUntil(AUTO_IDLE, 10000) && thrustLocks >= 1, "IDLE after the phone landing, app thrust locked");
  run(700);

  /* Both barriers 0 in every state, also for a controller gone wrong. */
  static const char *const names[NSTATES] = {"IDLE", "ARMED", "TAKEOFF", "HOVER", "LAND", "KILLED"};
  for (int s = AUTO_ARMED; s <= AUTO_KILLED; s++) {
    CHECK(seenState[s] && maxB1[s] == 0.0f && maxB2[s] == 0.0f && maxRogue[s] == 0.0f,
          "DRY %s: seen %d, barrier 1 %.0f, barrier 2 %.0f, 40000 from the controller -> %.0f", names[s], seenState[s],
          maxB1[s], maxB2[s], maxRogue[s]);
  }
  CHECK(seenHold && maxB1Hold == 0.0f && maxB2Hold == 0.0f && maxRogueHold == 0.0f,
        "DRY release hold: seen %d, %.0f %.0f %.0f", seenHold, maxB1Hold, maxB2Hold, maxRogueHold);
  CHECK(overrideLoops > 10000 && overrideBad == 0, "override: roll/pitch +0.0f, x/y/z disabled, yaw absolute (%d of %d)",
        overrideBad, overrideLoops);
  CHECK(manualTouched == 0, "IDLE loops untouched (%d)", manualTouched);
  CHECK(vzInputCalls == 0, "DRY never reads the real camera height (%d)", vzInputCalls);
  CHECK(flashWrites == 0, "DRY learns nothing (%d writes)", flashWrites);
  autonomyGetStatus(&m);
  CHECK(seqObservations > 1000 && seqMismatches == 0, "snapshot and seq together at every lock release (%u of %u)",
        (unsigned)seqMismatches, (unsigned)seqObservations);
  CHECK(autonomyStatusSeq() - seq0 == m.transitions, "seq == transitions (%u, %u)",
        (unsigned)(autonomyStatusSeq() - seq0), (unsigned)m.transitions);
  CHECK(lockDepth == 0, "locks balanced");
}

/* BENCH on the sim: <= the bench cap at both barriers, the lift-off note. */
static void scenBench(void)
{
  bootToSession(ESP_RST_POWERON);
  gcapStub = 15000;
  autoCmdResult_t r = arm(AUTO_MODE_BENCH);
  CHECK(r.result == AUTO_R_OK, "BENCH ARM ok (%d 0x%x)", r.result, (unsigned)r.detail);
  run(100);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "TAKEOFF ok");
  CHECK(runUntil(AUTO_HOVER, 8000), "HOVER");
  CHECK((statusFlags() & AUTO_STF_SIM) && liftoffNotes == 1, "sim flag, the sim's lift-off noted (%d)", liftoffNotes);
  run(1500);
  CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK, "LAND ok (%d)", r.result);
  CHECK(runUntil(AUTO_IDLE, 10000), "IDLE");
  run(700);
  float m1 = 0.0f, m2 = 0.0f, mr = 0.0f;
  for (int s = AUTO_ARMED; s <= AUTO_KILLED; s++) {
    m1 = fmaxf(m1, maxB1[s]);
    m2 = fmaxf(m2, maxB2[s]);
    mr = fmaxf(mr, maxRogue[s]);
  }
  CHECK(m1 > 0.0f && m1 <= 15000.0f && m2 <= 15000.0f && mr <= 15000.0f, "BENCH: <= 15000 (%.0f %.0f %.0f)", m1, m2, mr);
  CHECK(maxB2Hold == 0.0f && maxRogueHold == 0.0f, "release hold 0");
  CHECK(vzInputCalls == 0 && flashWrites == 0, "BENCH-sim: no real camera height, nothing learned");
  CHECK(overrideBad == 0, "override exact");
}

/* The plant comes from the ARM command, never from the previous sortie. */
static void scenPlant(void)
{
  bootToSession(ESP_RST_POWERON);
  vbatStub = 4.1f;
  run(17000);   /* FLIGHT needs 20 s on the ground */
  autoCmdResult_t r;
  int calls;

  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "DRY ARM ok (0x%x)", (unsigned)r.detail);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "DRY TAKEOFF");
  run(2500);
  CHECK((statusFlags() & AUTO_STF_SIM) && vzInputCalls == 0, "DRY: sim plant (calls %d)", vzInputCalls);
  killAndWait();

  r = arm(AUTO_MODE_FLIGHT);
  CHECK(r.result == AUTO_R_OK, "FLIGHT ARM after DRY ok, G12 sees a real plant (%d 0x%x)", r.result, (unsigned)r.detail);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "FLIGHT TAKEOFF");
  calls = vzInputCalls;
  run(2500);
  CHECK(!(statusFlags() & AUTO_STF_SIM) && vzInputCalls > calls, "FLIGHT: real inputs (calls %d)", vzInputCalls - calls);
  CHECK(camSkipStub, "FLIGHT: camera standby skip");
  killAndWait();
  CHECK(!camSkipStub, "skip cleared after the sortie");

  gcapStub = 15000;
  CHECK(autonomySetBenchSource(AUTO_BENCH_SRC_SIM), "bench sim");
  r = arm(AUTO_MODE_BENCH);
  CHECK(r.result == AUTO_R_OK, "BENCH-sim ARM ok (0x%x)", (unsigned)r.detail);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "BENCH-sim TAKEOFF");
  calls = vzInputCalls;
  run(2500);
  CHECK((statusFlags() & AUTO_STF_SIM) && vzInputCalls == calls, "BENCH-sim: sim plant");
  CHECK(!autonomySetBenchSource(AUTO_BENCH_SRC_IMU), "bench source refused while active");
  killAndWait();

  CHECK(autonomySetBenchSource(AUTO_BENCH_SRC_IMU), "bench imu");
  r = arm(AUTO_MODE_BENCH);
  CHECK(r.result == AUTO_R_OK, "BENCH-imu ARM after BENCH-sim ok (0x%x)", (unsigned)r.detail);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "BENCH-imu TAKEOFF");
  calls = vzInputCalls;
  run(2500);
  CHECK(!(statusFlags() & AUTO_STF_SIM) && vzInputCalls > calls, "BENCH-imu: real inputs");
  killAndWait();

  CHECK(autonomySetBenchSource(AUTO_BENCH_SRC_SIM), "bench sim again");
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "DRY after BENCH-imu");
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "DRY TAKEOFF 2");
  calls = vzInputCalls;
  run(2500);
  CHECK((statusFlags() & AUTO_STF_SIM) && vzInputCalls == calls, "DRY after BENCH-imu: sim plant");
  killAndWait();
  CHECK(flashWrites == 0, "no vzK learned from these sorties (%d)", flashWrites);
}

/* G1 LOCKS from the glue: an NVS write in progress (any mode), a camvz bench
 * mode or a paused observer (the modes whose controller reads the camera
 * height: FLIGHT and BENCH-imu). */
static void scenLocks(void)
{
  bootToSession(ESP_RST_POWERON);
  vbatStub = 4.1f;
  run(17000);
  autoCmdResult_t r;
  autonomyProfile_t p;

  CHECK(autonomyFlashBegin(), "a writer holds the flash interlock");
  r = arm(AUTO_MODE_DRY);
  CHECK(refusedLocks(&r), "DRY ARM during an NVS write: GUARDS LOCKS (%d 0x%x)", r.result, (unsigned)r.detail);
  autonomyFlashEnd();
  autonomyGetProfile(&p);
  CHECK(p.armsBlocked == 1, "armsBlocked 1 (%u)", (unsigned)p.armsBlocked);
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "DRY ARM after the write ok (%d 0x%x)", r.result, (unsigned)r.detail);
  disarm();

  for (int c = 0; c < 4; c++) {
    static const char *const cond[] = {"camvz air on", "camvz air spin", "camvz off", "nothing"};
    benchAirStub = c == 0 ? CVS_BENCH_AIR : (c == 1 ? CVS_BENCH_SPIN : CVS_BENCH_OFF);
    pausedStub = c == 2;
    const bool locked = c < 3;
    gcapStub = 0;
    r = arm(AUTO_MODE_FLIGHT);
    if (locked) {
      CHECK(refusedLocks(&r), "%s: FLIGHT ARM GUARDS LOCKS (%d 0x%x)", cond[c], r.result, (unsigned)r.detail);
    } else {
      CHECK(r.result == AUTO_R_OK, "%s: FLIGHT ARM ok (%d 0x%x)", cond[c], r.result, (unsigned)r.detail);
      disarm();
    }
    gcapStub = 15000;
    autonomySetBenchSource(AUTO_BENCH_SRC_IMU);
    r = arm(AUTO_MODE_BENCH);
    if (locked) {
      CHECK(refusedLocks(&r), "%s: BENCH-imu ARM GUARDS LOCKS (%d 0x%x)", cond[c], r.result, (unsigned)r.detail);
    } else {
      CHECK(r.result == AUTO_R_OK, "%s: BENCH-imu ARM ok (%d 0x%x)", cond[c], r.result, (unsigned)r.detail);
      disarm();
    }
    autonomySetBenchSource(AUTO_BENCH_SRC_SIM);
    r = arm(AUTO_MODE_BENCH);
    CHECK(r.result == AUTO_R_OK, "%s: BENCH-sim ARM ok (%d 0x%x)", cond[c], r.result, (unsigned)r.detail);
    disarm();
    gcapStub = 0;
    r = arm(AUTO_MODE_DRY);
    CHECK(r.result == AUTO_R_OK, "%s: DRY ARM ok (%d 0x%x)", cond[c], r.result, (unsigned)r.detail);
    disarm();
  }
  autonomyGetProfile(&p);
  CHECK(p.armsBlocked == 1, "only the NVS write counts in armsBlocked (%u)", (unsigned)p.armsBlocked);
  CHECK(lockDepth == 0, "locks balanced");
}

/* Settings writers and ARMs (R7): the NVS write, its read-back and the RAM
 * copy change inside one claim of the flash interlock. */
static autoCmdResult_t hookRes;
static bool hookTook;

static void hookPostArm(void)
{
  const autoCmd_t c = makeCmd(AUTO_CMD_ARM, AUTO_MODE_DRY, 0);
  uint32_t t;
  hookTook = false;
  if (!autonomyPostCommand(&c, &t)) {
    return;
  }
  for (int i = 0; i < 3 && !hookTook; i++) {
    loopOnce();
    hookTook = autonomyTakeResult(t, &hookRes);
  }
}

static void hookArmAtRelease(void)
{
  nextUnlock = hookPostArm;
}

/* R2: a console change (here 'gcap 15000': changeBegin, live set, store,
 * changeEnd) that ends while the step taking an ARM posted during it reads
 * the guard inputs. The inputs must be read after the take: an ARM taken
 * with the interlock free cannot see a half-done change, and one taken while
 * it is held is refused LOCKS. Read before the take, this FLIGHT ARM was
 * accepted on gcap 0 with gcap already 15000. */
static void hookEndGcapChange(void)
{
  gcapStub = 15000;
  autonomyFlashEnd();
}

static void scenRace(void)
{
  bootToSession(ESP_RST_POWERON);
  vbatStub = 4.1f;
  run(17000);
  gcapStub = 0;
  gesture();
  CHECK(autonomyFlashBegin(), "a console change holds the flash interlock");
  const autoCmd_t c = makeCmd(AUTO_CMD_ARM, AUTO_MODE_FLIGHT, 0);
  uint32_t t;
  CHECK(autonomyPostCommand(&c, &t), "FLIGHT ARM posted during the change");
  beforeLock = hookEndGcapChange;   /* the next loop's first lock: the command's take */
  autoCmdResult_t r;
  memset(&r, 0, sizeof(r));
  bool took = false;
  for (int i = 0; i < 3 && !took; i++) {
    loopOnce();
    took = autonomyTakeResult(t, &r);
  }
  CHECK(beforeLock == NULL && gcapStub == 15000, "the change ended just before the ARM's take");
  CHECK(took && r.result == AUTO_R_GUARDS && autonomyState() == AUTO_IDLE,
        "the ARM is refused, never accepted on the values before the change (%d 0x%x, state %d)", r.result,
        (unsigned)r.detail, (int)autonomyState());
  r = arm(AUTO_MODE_FLIGHT);
  CHECK(r.result == AUTO_R_GUARDS && (r.detail & AUTO_G_GCAP) && autonomyState() == AUTO_IDLE,
        "a later FLIGHT ARM sees gcap 15000: GUARDS GCAP (%d 0x%x)", r.result, (unsigned)r.detail);
  CHECK(lockDepth == 0, "locks balanced");
}

/* Review E2E-1: autonomy hands back to the pilot (IDLE, the app thrust
 * locked) while the phone's last setpoint, with its throttle up, is still in
 * the commander and the phone has gone silent. The commander keeps that
 * thrust for 2000 ms; it must never reach the motors after the release hold. */
static void scenHandback(void)
{
  bootToSession(ESP_RST_POWERON);
  vbatStub = 4.1f;
  run(17000);
  autoCmdResult_t r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK && autonomyState() == AUTO_ARMED, "DRY armed (%d)", r.result);
  appThrust = 30000;   /* the pilot pushes the throttle: APP_INPUT */
  int i = 0;
  while (autonomyState() != AUTO_IDLE && i++ < 200) {
    loopOnce();
  }
  CHECK(autonomyState() == AUTO_IDLE, "phone input in ARMED: IDLE (APP_INPUT)");
  appOnline = false;   /* and the phone goes silent with 30000 in the commander */
  const int drops0 = setpointDrops;
  float maxDuty = 0.0f;
  for (i = 0; i < 1900; i++) {
    loopOnce();
    maxDuty = fmaxf(maxDuty, dutyThrust);
  }
  CHECK(maxDuty == 0.0f, "no stale phone thrust after the hand-back (max %.0f)", maxDuty);
  CHECK(setpointDrops > drops0 || cmdQNull, "the commander's queued setpoint was dropped at the hand-back");
  appOnline = true;
  appThrust = 0;
  run(100);
  CHECK(lockDepth == 0, "locks balanced");
}

static void scenSettings(void)
{
  bootToSession(ESP_RST_POWERON);
  const char *why = "";
  autonomyProfile_t p;

  /* an ARM taken while the store writes */
  gesture();
  duringWrite = hookPostArm;
  CHECK(autonomySettingSet(AUTO_SET_AUTOWAIT, 4000, &why), "set autowait 4000");
  CHECK(hookTook && refusedLocks(&hookRes), "ARM during the write: GUARDS LOCKS (%d 0x%x)", hookRes.result,
        (unsigned)hookRes.detail);
  CHECK(autonomySettingGet(AUTO_SET_AUTOWAIT) == 4000 && nvsGet("autowait") == 4000, "RAM and NVS 4000");

  /* an ARM taken after the store's own bracket, before the RAM copy changes */
  gesture();
  afterWrite = hookPostArm;
  CHECK(autonomySettingSet(AUTO_SET_AUTOWAIT, 3000, &why), "set autowait 3000");
  CHECK(hookTook && refusedLocks(&hookRes), "ARM between the NVS write and the RAM copy: GUARDS LOCKS (%d 0x%x)",
        hookRes.result, (unsigned)hookRes.detail);
  CHECK(autonomySettingGet(AUTO_SET_AUTOWAIT) == 3000 && nvsGet("autowait") == 3000, "RAM and NVS 3000");

  /* the same for an erase */
  gesture();
  afterWrite = hookPostArm;
  CHECK(autonomySettingReset(AUTO_SET_AUTOWAIT, &why), "reset autowait");
  CHECK(hookTook && refusedLocks(&hookRes), "ARM between the erase and the RAM copy: GUARDS LOCKS (%d 0x%x)",
        hookRes.result, (unsigned)hookRes.detail);
  CHECK(autonomySettingGet(AUTO_SET_AUTOWAIT) == 5000 && nvsGet("autowait") == -1, "RAM default, NVS erased");
  autonomyGetProfile(&p);
  CHECK(p.armsBlocked == 3, "armsBlocked 3 (%u)", (unsigned)p.armsBlocked);

  /* an ARM waiting in the mailbox: the write is refused and changes nothing */
  gesture();
  const int32_t k0 = autonomySettingGet(AUTO_SET_VZK);
  const autoCmd_t c = makeCmd(AUTO_CMD_ARM, AUTO_MODE_DRY, 0);
  uint32_t t;
  CHECK(autonomyPostCommand(&c, &t), "ARM posted");
  why = "";
  CHECK(!autonomySettingSet(AUTO_SET_VZK, 150000, &why) && strcmp(why, "busy: autonomy active") == 0,
        "set with an ARM in the mailbox: busy (%s)", why);
  CHECK(autonomySettingGet(AUTO_SET_VZK) == k0 && nvsGet("vzK") == -1, "vzK unchanged");
  autoCmdResult_t r;
  loopOnce();
  CHECK(autonomyTakeResult(t, &r) && r.result == AUTO_R_OK, "that ARM proceeds (%d 0x%x)", r.result,
        (unsigned)r.detail);
  disarm();

  /* the sortie's vzK is the one in RAM at its ARM */
  CHECK(autonomySettingSet(AUTO_SET_VZK, 150000, &why), "set vzK 150000");
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "ARM ok");
  CHECK(!autonomySettingSet(AUTO_SET_VZK, 160000, &why), "vzK busy while ARMED");
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "TAKEOFF");
  CHECK(runUntil(AUTO_HOVER, 8000), "HOVER");
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  CHECK(m.kPriorDiv10 == 15000, "the sortie's kPrior 150000 (%u0)", m.kPriorDiv10);
  killAndWait();
  CHECK(autonomySettingGet(AUTO_SET_VZK) == 150000 && nvsGet("vzK") == 150000, "vzK 150000 kept");

  /* an ARM taken the moment the claim is released arms with the new value
   * (the RAM copy changed before the release) */
  gesture();
  afterWrite = hookArmAtRelease;
  CHECK(autonomySettingSet(AUTO_SET_VZK, 155000, &why), "set vzK 155000");
  CHECK(hookTook && hookRes.result == AUTO_R_OK && autonomyState() == AUTO_ARMED,
        "ARM at the claim's release accepted (%d 0x%x)", hookRes.result, (unsigned)hookRes.detail);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, hookRes.detail, &r, 3) && r.result == AUTO_R_OK, "TAKEOFF");
  CHECK(runUntil(AUTO_HOVER, 8000), "HOVER");
  autonomyGetStatus(&m);
  CHECK(m.kPriorDiv10 == 15500, "that sortie's kPrior 155000 (%u0)", m.kPriorDiv10);
  killAndWait();
  CHECK(lockDepth == 0, "locks balanced");
}

/* FLIGHT on a simulated plant through the real-input path. */
typedef struct {
  bool hover;
  uint32_t waitAirMs;     /* air clock when the hover wait started (0: never) */
  uint32_t landAirMs;     /* air clock on the first step out of HOVER */
  uint16_t hoverWaitMs;   /* AUTO_STATUS hoverWaitMs in the wait (the autowait latched at ARM) */
  uint8_t landReason;
  int32_t k0;
  bool macLand;           /* the emulated Mac posted LAND at the wait's end */
} flightRun_t;

/* Since 2026-10-15 ("Claude decides", docs/autonomy-a2b.md 22) the state machine lands on none of the hover's
 * clocks: the Mac's sortie lands at the wait's end. These scenarios play the Mac: once the status has shown the
 * wait (AUTO_STF_WAITING) for waitMs loops (1 ms each, no motion), LAND is posted. Steps while HOVER; returns the
 * loops from the wait's start to the post (0: none), *waitAir the air clock (ms) at the wait's start (status airDs
 * x 100 past 65.5 s). */
static uint32_t hoverToMacLand(uint32_t waitMs, uint32_t *waitAir)
{
  autoStatusMsg_t m;
  uint32_t since = 0, at = 0;
  bool posted = false;
  *waitAir = 0;
  while (autonomyState() == AUTO_HOVER) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (m.state == AUTO_HOVER && (m.flags & AUTO_STF_WAITING)) {
      if (since == 0) {
        *waitAir = m.airMs < 65535u ? m.airMs : (uint32_t)m.airDs * 100u;
      }
      since++;
    }
    if (!posted && since >= waitMs) {
      autoCmd_t c;
      memset(&c, 0, sizeof(c));
      c.type = AUTO_CMD_LAND;
      uint32_t t;
      posted = autonomyPostCommand(&c, &t);
      at = since;
    }
  }
  return at;
}

static void flightSortie(float kFactor, uint8_t photoN, flightRun_t *fr)
{
  memset(fr, 0, sizeof(*fr));
  vbatStub = 4.1f;
  run(17000);
  fr->k0 = autonomySettingGet(AUTO_SET_VZK);
  vzSimDefaultConfig(&plantCfg);
  plantCfg.kTrue = (float)fr->k0 * kFactor;
  plantCfg.vRest = 4.08f;
  plantCfg.camWakeS = 0.0f;   /* camera awake through the take-off */
  vzSimInit(&plant, &plantCfg, 99u);
  plantV = plant.vbat;
  usePlant = true;
  run(200);
  autoCmdResult_t r = arm(AUTO_MODE_FLIGHT);
  CHECK(r.result == AUTO_R_OK, "FLIGHT ARM ok (%d 0x%x)", r.result, (unsigned)r.detail);
  run(100);
  autoCmd_t c = makeCmd(AUTO_CMD_TAKEOFF, 0, r.detail);
  c.photoN = photoN;
  uint32_t t;
  CHECK(autonomyPostCommand(&c, &t), "TAKEOFF posted");
  loopOnce();
  CHECK(autonomyTakeResult(t, &r) && r.result == AUTO_R_OK, "TAKEOFF ok (%d)", r.result);
  fr->hover = runUntil(AUTO_HOVER, 8000);
  autoStatusMsg_t m;
  uint32_t waitLoops = 0;
  while (autonomyState() == AUTO_HOVER) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (m.flags & AUTO_STF_CAM_HEIGHT) {
      camHeightSeen = true;
    }
    if (!fr->waitAirMs && (m.flags & AUTO_STF_WAITING)) {
      fr->waitAirMs = m.airMs;
      fr->hoverWaitMs = m.hoverWaitMs;
    }
    waitLoops += fr->waitAirMs != 0;
    if (!fr->macLand && fr->waitAirMs != 0 && waitLoops >= fr->hoverWaitMs) {
      autoCmd_t c;
      memset(&c, 0, sizeof(c));
      c.type = AUTO_CMD_LAND;   /* the Mac's sortie: LAND at the wait's end */
      uint32_t t;
      fr->macLand = autonomyPostCommand(&c, &t);
    }
  }
  autonomyGetStatus(&m);
  fr->landReason = m.reason;
  fr->landAirMs = m.airMs;
  runUntil(AUTO_IDLE, 10000);
}

static void scenFlightSim(void)
{
  bootToSession(ESP_RST_POWERON);
  flightRun_t fr;
  const int fw0 = flashWrites;
  flightSortie(1.03f, 1, &fr);   /* 3 % heavier than stored: it learns */
  CHECK(fr.hover && liftoffNotes == 1, "FLIGHT on the plant: HOVER, lift-off noted (%d)", liftoffNotes);
  CHECK(camHeightSeen, "the camera height in use in HOVER");
  CHECK(fr.waitAirMs > 0, "the hover wait started (land reason %d)", fr.landReason);
  CHECK(autonomyState() == AUTO_IDLE, "landed");
  CHECK(maxB2[AUTO_TAKEOFF] > 0.0f && maxB2[AUTO_HOVER] <= VZ_THRUST_ABS_MAX &&
            maxRogue[AUTO_HOVER] <= VZ_THRUST_ABS_MAX,
        "FLIGHT max %.0f", maxB2[AUTO_HOVER]);
  run(3000);
  const int32_t k1 = autonomySettingGet(AUTO_SET_VZK);
  printf("info: flightsim vzK %ld -> %ld, hover wait at air %u ms, land reason %d, hMax %.3f m\n", (long)fr.k0,
         (long)k1, (unsigned)fr.waitAirMs, fr.landReason, (double)plant.hMax);
  CHECK(k1 != fr.k0 && flashWrites - fw0 == 1, "learned vzK stored once by autonomyPoll (%d writes)",
        flashWrites - fw0);
  CHECK(nvsGet("vzK") == k1, "NVS holds it");
  run(6000);
  CHECK(flashWrites - fw0 == 1, "and only once");
  CHECK(overrideBad == 0, "override exact");
}

/* The pre-wait air-clock margin of the Mac's default request (40 cm, photoN
 * photos 300 ms apart, the default autoair): info only. */
static void marginRun(float kFactor, uint8_t photoN)
{
  bootToSession(ESP_RST_POWERON);
  flightRun_t fr;
  flightSortie(kFactor, photoN, &fr);
  printf("info: margin K x%.2f photoN %u: hover wait at air %u ms (autoair %u), land reason %d\n", (double)kFactor,
         photoN, (unsigned)fr.waitAirMs, (unsigned)AUTO_AIR_MAX_DEFAULT_MS, fr.landReason);
}

/* The user's sortie of 2026-10-12 (evening) through the glue: TAKEOFF
 * 60 cm (the controller's climb timeout and LAND flat part lengthened by
 * vzConfigSetHeight in vzStart), 1 photo, 'auto set autowait 10000', the
 * default autoair: the wait starts >= 1 s inside the pre-wait limit and
 * the Mac lands 10 s later (HOVER_TIMEOUT until 2026-10-15). */
static void margin60Run(float kFactor)
{
  bootToSession(ESP_RST_POWERON);
  const char *why = "";
  CHECK(autonomySettingSet(AUTO_SET_AUTOWAIT, 10000, &why) && autonomySettingGet(AUTO_SET_AUTOWAIT) == 10000,
        "auto set autowait 10000 accepted (%s)", why);
  CHECK(!autonomySettingSet(AUTO_SET_AUTOWAIT, AUTO_HOVER_WAIT_MAX_MS + 1, &why) &&
        autonomySettingGet(AUTO_SET_AUTOWAIT) == 10000, "autowait %u refused (%s)",
        (unsigned)AUTO_HOVER_WAIT_MAX_MS + 1u, why);
  takeoffCm = 60;
  flightRun_t fr;
  flightSortie(kFactor, 1, &fr);
  printf("info: margin 60 cm K x%.2f photoN 1, autowait 10000: hover wait at air %u ms (autoair %u), LAND at air %u "
         "ms, land reason %d, true peak %.3f m\n", (double)kFactor, (unsigned)fr.waitAirMs,
         (unsigned)AUTO_AIR_MAX_DEFAULT_MS, (unsigned)fr.landAirMs, fr.landReason, (double)plant.hMax);
  CHECK(fr.hover && fr.waitAirMs > 0 && fr.waitAirMs + 1000u <= AUTO_AIR_MAX_DEFAULT_MS && fr.hoverWaitMs == 10000,
        "60 cm: the wait (%u ms) starts at air %u ms, >= 1 s inside autoair %u", (unsigned)fr.hoverWaitMs,
        (unsigned)fr.waitAirMs, (unsigned)AUTO_AIR_MAX_DEFAULT_MS);
  /* both air clocks are AUTO_STATUS samples (10 Hz): +/- 150 ms */
  CHECK(fr.macLand && fr.landReason == AUTO_RS_LAND_CMD && fr.landAirMs + 150u >= fr.waitAirMs + 10000u &&
        fr.landAirMs <= fr.waitAirMs + 10150u,
        "60 cm: the Mac's LAND 10 s into the wait (reason %d, at air %u ms)", fr.landReason, (unsigned)fr.landAirMs);
  CHECK(plant.hMax >= 0.45f && plant.hMax <= 0.80f && autonomyState() == AUTO_IDLE,
        "60 cm: true peak %.3f m (0.45..0.80), landed", (double)plant.hMax);
}
static void scenMargin1(void) { marginRun(1.00f, 1); }
static void scenMargin1h(void) { marginRun(1.03f, 1); }
static void scenMargin2(void) { marginRun(1.00f, 2); }
static void scenMargin2h(void) { marginRun(1.03f, 2); }
static void scenMargin60(void) { margin60Run(1.00f); }
static void scenMargin60h(void) { margin60Run(1.03f); }

/* ---- the interactive hover (auto_motion.h) ---------------------------------- */

/* A command with its fields, posted like the link task; the result. */
static bool post(const autoCmd_t *c, autoCmdResult_t *r)
{
  uint32_t t;
  if (!autonomyPostCommand(c, &t)) {
    return false;
  }
  for (int i = 0; i < 3; i++) {
    linkBackground();
    loopOnce();
    if (autonomyTakeResult(t, r)) {
      return true;
    }
  }
  return false;
}

static autoCmd_t moveCmd(int16_t right, int16_t fwd, int16_t up)
{
  autoCmd_t c;
  memset(&c, 0, sizeof(c));
  c.type = AUTO_CMD_MOVE;
  c.moveRightCm = right;
  c.moveFwdCm = fwd;
  c.moveUpCm = up;
  return c;
}

static autoCmd_t yawCmd(int16_t deg)
{
  autoCmd_t c;
  memset(&c, 0, sizeof(c));
  c.type = AUTO_CMD_YAW;
  c.yawDeg = deg;
  return c;
}

static autoCmd_t simpleCmd(autoCmdType_t t)
{
  autoCmd_t c;
  memset(&c, 0, sizeof(c));
  c.type = t;
  return c;
}

/* ARM in mode, TAKEOFF heightCm with this sortie's wait, until waiting. */
static bool toInteractiveWait(autoMode_t mode, uint16_t heightCm, uint32_t waitMs)
{
  autoCmdResult_t r = arm(mode);
  CHECK(r.result == AUTO_R_OK, "%s ARM ok (%d 0x%x)", mode == AUTO_MODE_DRY ? "DRY" : "FLIGHT", r.result,
        (unsigned)r.detail);
  run(100);
  autoCmd_t c = makeCmd(AUTO_CMD_TAKEOFF, 0, r.detail);
  c.heightCm = heightCm;
  c.actWaitMs = waitMs;
  CHECK(post(&c, &r) && r.result == AUTO_R_OK, "TAKEOFF %u cm, wait %u ms ok (%d)", heightCm, (unsigned)waitMs,
        r.result);
  autoStatusMsg_t m;
  for (int i = 0; i < 15000; i++) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (m.state == AUTO_HOVER && (m.flags & AUTO_STF_WAITING)) {
      return true;
    }
  }
  return false;
}

/* Steps until the motion is no longer active (RUN, PAUSED, SETTLE) in AUTO_STATUS. */
static bool untilMotionDone(uint32_t maxMs)
{
  autoStatusMsg_t m;
  for (uint32_t i = 0; i < maxMs; i++) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (m.mvState != AUTO_MV_RUN && m.mvState != AUTO_MV_PAUSED && m.mvState != AUTO_MV_SETTLE) {
      return true;
    }
  }
  return false;
}

/* DRY: MOVE and YAW through the glue: the hold's reference for the camera
 * task, the setpoint's yaw (clockwise: the yaw angle goes down), roll and
 * pitch exactly 0, both barriers 0 throughout; the status fields. */
static void scenInteractiveDry(void)
{
  bootToSession(ESP_RST_POWERON);
  yawFree = true;
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting at 1.00 m (sim), wait 30 s");
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  CHECK(m.waitLeftMs == 0, "the TAKEOFF's 30 s wait: information for the Mac since 2026-10-15, waitLeftMs 0 (%u)",
        m.waitLeftMs);
  CHECK(m.sceneDCm == 200 && (m.mvFlags & AUTO_MVF_D_SIM) && m.hTargetCm == 100, "D 2.00 m (the sim's), height "
        "100 cm (%u %d)", m.sceneDCm, m.hTargetCm);
  autoCmdResult_t r;
  CHECK(post((const autoCmd_t[]){moveCmd(50, -30, 0)}, &r) &&
        r.result == AUTO_R_OK && (r.detail >> 16) == 1, "MOVE right 50 back 30: OK (detail 0x%x)", (unsigned)r.detail);
  float maxLat = 0.0f, minFwd = 0.0f, maxB = 0.0f, ffSeen = 0.0f;
  for (int i = 0; i < 2500; i++) {
    linkBackground();
    loopOnce();
    autonomyHoldRef_t h;
    autonomyGetHoldRef(&h);
    maxLat = fmaxf(maxLat, h.refLat);
    minFwd = fminf(minFwd, h.refFwd);
    ffSeen = fmaxf(ffSeen, fabsf(h.ffRoll));
    maxB = fmaxf(maxB, fmaxf(sp.thrust, autonomyThrustMax()));
  }
  CHECK(maxLat > 0.02f && minFwd < -0.01f && ffSeen > 0.05f, "the camera task gets the reference: right %.3f, back "
        "%.3f (x / D, ln), feedforward %.2f deg", maxLat, minFwd, ffSeen);
  CHECK(untilMotionDone(20000), "MOVE done");
  autonomyGetStatus(&m);
  autonomyHoldRef_t h;
  autonomyGetHoldRef(&h);
  CHECK(m.mvState == AUTO_MV_DONE && m.progRightCm == 50 && m.progFwdCm == -30 && m.tgtRightCm == 50 &&
        fabsf(h.refLat - 0.25f) < 1e-5f && h.rateLat == 0.0f && h.ffRoll == 0.0f,
        "DONE: progress 50 / -30 cm, the hold keeps the end (%.4f)", h.refLat);
  CHECK(post((const autoCmd_t[]){yawCmd(30)}, &r) && r.result == AUTO_R_OK, "YAW 30 (clockwise): OK");
  float minYaw = 33.0f, prevYaw = 33.0f;
  bool mono = true;
  for (int i = 0; i < 4000; i++) {
    linkBackground();
    loopOnce();
    mono = mono && spYaw <= prevYaw + 1e-4f;
    prevYaw = spYaw;
    minYaw = fminf(minYaw, spYaw);
  }
  autonomyGetStatus(&m);
  CHECK(mono && fabsf(spYaw - 3.0f) < 1e-3f && m.yawOffCdeg == 3000 && m.mvState == AUTO_MV_DONE &&
        guessAllowedOn == 0, "the setpoint yaw goes down from 33 to %.3f deg (clockwise), yawOff 30.00 deg; DRY "
        "never lets the real camera use a guess", spYaw);
  CHECK(post((const autoCmd_t[]){moveCmd(0, 0, 15)}, &r) && r.result == AUTO_R_OK, "MOVE up 15 (the sim plant)");
  CHECK(untilMotionDone(10000), "done");
  autonomyGetStatus(&m);
  CHECK(m.hTargetCm == 115 && m.progUpCm >= 10 && m.progUpCm <= 20, "height 115 cm, the sim rose %d cm",
        m.progUpCm);
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_STOP)}, &r) && r.result == AUTO_R_OK, "STOP: OK");
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed (from 1.15 m on the sim)");
  CHECK(maxB == 0.0f && maxB1[AUTO_HOVER] == 0.0f && maxB2[AUTO_HOVER] == 0.0f && maxB2[AUTO_LAND] == 0.0f,
        "DRY: both barriers 0 through the motions");
  CHECK(overrideBad == 0 && fabsf(spYaw - 3.0f) < 1e-3f, "override exact (roll/pitch 0.0f), the turned heading "
        "kept to the end (%.3f)", spYaw);
}

/* mvspeed (2026-10-16, auto_motion.h "fast moves", docs/autonomy-a2b.md 31): NVS mvspeed 200 written in IDLE shows in
 * AUTO_STATUS at once (IDLE: the NVS value) while the camera task's step stays 100; the ARM latches it (the status and
 * the camera's step 200, the motions' profiles at 200, the setting refused while active); a STOP mid-move gives
 * controller_pid's stale-hold branch the brake lean (autonomyBrakeLean) only while the brake runs; after the LAND the
 * camera's step is 100 again. The same STOP at the default 100 gives no brake lean to it. */
static void mvSpeedRun(uint16_t pct)
{
  bootToSession(ESP_RST_POWERON);
  const char *why = "";
  autoStatusMsg_t m;
  if (pct != 100) {
    CHECK(autonomySettingSet(AUTO_SET_MVSPEED, pct, &why) && autonomySettingGet(AUTO_SET_MVSPEED) == pct &&
          nvsGet("mvspeed") == pct, "set mvspeed %u (RAM and NVS)", pct);
    CHECK(!autonomySettingSet(AUTO_SET_MVSPEED, 301, &why) && !autonomySettingSet(AUTO_SET_MVSPEED, 99, &why),
          "mvspeed 301 / 99 refused");
  }
  run(200);
  autonomyGetStatus(&m);
  CHECK(m.mvSpeedPct == pct && autonomyMvSpeedPct() == 100, "IDLE: AUTO_STATUS mvSpeedPct %u (the NVS value), the "
        "camera's step 100 (%u)", m.mvSpeedPct, autonomyMvSpeedPct());
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting at 1.00 m (sim, D 2.00 m)");
  autonomyGetStatus(&m);
  CHECK(m.mvSpeedPct == pct && autonomyMvSpeedPct() == pct, "the sortie: mvSpeedPct %u, the camera's step %u",
        m.mvSpeedPct, autonomyMvSpeedPct());
  CHECK(!autonomySettingSet(AUTO_SET_MVSPEED, 150, &why) && strcmp(why, "busy: autonomy active") == 0,
        "mvspeed refused while active (%s)", why);
  autoCmdResult_t r;
  CHECK(post((const autoCmd_t[]){moveCmd(0, 100, 0)}, &r) && r.result == AUTO_R_OK, "MOVE forward 100: OK");
  uint32_t runMs = 0;
  for (int i = 0; i < 20000; i++) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (m.mvState != AUTO_MV_RUN) {
      break;
    }
    runMs++;
  }
  /* the profile at D 2.00 m: at 100 W_FWD x the end distance binds (0.15 m/s): 8.17 s; at 200 the forward peak rule
   * (W_FWD 0.17 since the review of cbccec5, ~0.18 m/s): 6.40 s (5.56 s at W_FWD 0.20) */
  const float want = pct == 100 ? 8.17f : 6.40f;
  CHECK(fabsf((float)runMs * 0.001f - want) < 0.1f, "mvspeed %u: forward 1.00 m at D 2 runs %.2f s (%.2f)", pct,
        (float)runMs * 0.001f, want);
  CHECK(untilMotionDone(20000), "done");
  float br = 0.0f, bp = 0.0f;
  CHECK(!autonomyBrakeLean(&br, &bp), "no brake after a DONE move");
  CHECK(post((const autoCmd_t[]){moveCmd(0, -100, 0)}, &r) && r.result == AUTO_R_OK, "MOVE back 100: OK");
  run(2500);
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_STOP)}, &r) && r.result == AUTO_R_OK, "STOP mid-move: OK");
  int brakeLoops = 0;
  float bpMin = 0.0f;
  for (int i = 0; i < 3000; i++) {
    linkBackground();
    loopOnce();
    float a = 0.0f, b = 0.0f;
    if (autonomyBrakeLean(&a, &b)) {
      brakeLoops++;
      bpMin = fminf(bpMin, b);
    }
  }
  if (pct == 100) {
    CHECK(brakeLoops == 0, "mvspeed 100: no brake lean for the controller's stale branch (%d loops)", brakeLoops);
  } else {
    /* a back move's brake leans forward (nose down, pitch < 0) for v / BRAKE: ~0.40 m/s / 0.30 m/s^2 */
    CHECK(brakeLoops > 500 && brakeLoops < 2000 && bpMin < -1.0f, "mvspeed %u: the brake lean for the stale branch "
          "%d loops, pitch %.2f deg", pct, brakeLoops, bpMin);
  }
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  run(200);
  autonomyGetStatus(&m);
  CHECK(autonomyMvSpeedPct() == 100 && m.mvSpeedPct == pct, "IDLE again: the camera's step 100, the status the NVS %u",
        m.mvSpeedPct);
}

/* The guard margins (2026-10-17, auto_motion.h "guard margins", docs/autonomy-a2b.md 32): NVS mvfloor / mvfstop /
 * mvlatm written in IDLE are latched at the ARM step with mvspeed (sortieSpeedLatch: the stop clamped to the floor - 5,
 * the plan the floor + 5), refused while active; a MOVE the default floor takes is refused TOO_CLOSE under mvfloor 45;
 * after 'auto reset' the next ARM has the defaults again. */
static void scenGuardMargins(void)
{
  bootToSession(ESP_RST_POWERON);
  const char *why = "";
  autoMvMargins_t g;
  CHECK(autonomySettingGet(AUTO_SET_MVFLOOR) == 30 && autonomySettingGet(AUTO_SET_MVFSTOP) == 25 &&
        autonomySettingGet(AUTO_SET_MVPROGM) == 20 && autonomySettingGet(AUTO_SET_MVPROGR) == 40 &&
        autonomySettingGet(AUTO_SET_MVLATM) == 40, "the margins' defaults after a boot without them in NVS");
  CHECK(autonomySettingSet(AUTO_SET_MVFLOOR, 45, &why) && autonomySettingSet(AUTO_SET_MVFSTOP, 60, &why) &&
        autonomySettingSet(AUTO_SET_MVLATM, 30, &why) && nvsGet("mvfloor") == 45 && nvsGet("mvfstop") == 60 &&
        nvsGet("mvlatm") == 30, "set mvfloor 45, mvfstop 60, mvlatm 30 (RAM and NVS)");
  CHECK(!autonomySettingSet(AUTO_SET_MVFLOOR, 19, &why) && !autonomySettingSet(AUTO_SET_MVFSTOP, 81, &why) &&
        !autonomySettingSet(AUTO_SET_MVPROGR, 101, &why), "out of range refused");
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting at 1.00 m (sim, D 2.00 m)");
  autoMotionGetMargins(&g);
  CHECK(g.floorCm == 45 && g.planCm == 50 && g.stopCm == 40 && g.progCm == 20 && g.progPct == 40 && g.latCm == 30,
        "the ARM latched them: floor 45, plan 50, the stop 60 clamped to 40, 20 cm + 40 %%, sideways 30 (%u %u %u %u "
        "%u %u)", g.floorCm, g.planCm, g.stopCm, g.progCm, g.progPct, g.latCm);
  CHECK(!autonomySettingSet(AUTO_SET_MVFLOOR, 30, &why) && strcmp(why, "busy: autonomy active") == 0,
        "mvfloor refused while active (%s)", why);
  autoCmdResult_t r;
  autoStatusMsg_t m;
  CHECK(post((const autoCmd_t[]){moveCmd(0, 125, 0)}, &r) && r.result == AUTO_R_RANGE, "MOVE forward 125 at D 2.0: "
        "refused under mvfloor 45 (ends 0.37 m at k 0.6)");
  autonomyGetStatus(&m);
  CHECK(m.mvWhy == AUTO_MVW_TOO_CLOSE, "... TOO_CLOSE (%s)", autoMvWhyName((autoMvWhy_t)m.mvWhy));
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  CHECK(autonomySettingReset(AUTO_SET_MVFLOOR, &why) && autonomySettingReset(AUTO_SET_MVFSTOP, &why) &&
        autonomySettingReset(AUTO_SET_MVLATM, &why), "auto reset mvfloor, mvfstop, mvlatm");
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting again");
  autoMotionGetMargins(&g);
  CHECK(g.floorCm == 30 && g.planCm == 35 && g.stopCm == 25 && g.latCm == 40, "the next ARM: the defaults again");
  CHECK(post((const autoCmd_t[]){moveCmd(0, 125, 0)}, &r) && r.result == AUTO_R_OK, "MOVE forward 125 at D 2.0: OK "
        "under the default 0.30 m");
  CHECK(untilMotionDone(20000), "done");
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");

  /* the guards down to off (2026-10-18, auto_motion.h "guards off", docs/autonomy-a2b.md 38): mvfloor / mvfstop /
   * mvprogm / mvprogr 0 and mvklow, mvlat, mvwmax written in IDLE (1..min - 1 still refused), latched at the ARM, a MOVE
   * up to the scene accepted, the defaults again after 'auto reset' */
  CHECK(autonomySettingGet(AUTO_SET_MVKLOW) == 60 && autonomySettingGet(AUTO_SET_MVLAT) == 50 &&
        autonomySettingGet(AUTO_SET_MVWMAX) == 15 && autoMotionGetFwdRate() == 15,
        "mvklow 60, mvlat 50, mvwmax 15 after a boot without them in NVS");
  CHECK(!autonomySettingSet(AUTO_SET_MVFLOOR, 19, &why) && !autonomySettingSet(AUTO_SET_MVFLOOR, 1, &why) &&
        !autonomySettingSet(AUTO_SET_MVFSTOP, 14, &why) && !autonomySettingSet(AUTO_SET_MVPROGM, 9, &why) &&
        !autonomySettingSet(AUTO_SET_MVPROGR, 5, &why) && !autonomySettingSet(AUTO_SET_MVKLOW, 59, &why) &&
        !autonomySettingSet(AUTO_SET_MVKLOW, 101, &why) && !autonomySettingSet(AUTO_SET_MVLAT, 201, &why) &&
        !autonomySettingSet(AUTO_SET_MVWMAX, 9, &why) && !autonomySettingSet(AUTO_SET_MVWMAX, 41, &why),
        "1..min - 1 and the new keys out of range refused");
  CHECK(autonomySettingSet(AUTO_SET_MVFLOOR, 0, &why) && autonomySettingSet(AUTO_SET_MVFSTOP, 0, &why) &&
        autonomySettingSet(AUTO_SET_MVPROGM, 0, &why) && autonomySettingSet(AUTO_SET_MVPROGR, 0, &why) &&
        autonomySettingSet(AUTO_SET_MVKLOW, 100, &why) && autonomySettingSet(AUTO_SET_MVLAT, 0, &why) &&
        autonomySettingSet(AUTO_SET_MVWMAX, 30, &why) && nvsGet("mvfloor") == 0 && nvsGet("mvfstop") == 0 &&
        nvsGet("mvprogm") == 0 && nvsGet("mvprogr") == 0 && nvsGet("mvklow") == 100 && nvsGet("mvlat") == 0 &&
        nvsGet("mvwmax") == 30, "every guard off, k 100, mvwmax 30 (RAM and NVS)");
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting at 1.00 m (sim, D 2.00 m)");
  autoMotionGetMargins(&g);
  CHECK(g.floorCm == 0 && g.planCm == 0 && g.stopCm == 0 && g.progCm == 0 && g.progPct == 0 && g.kLowPct == 100 &&
            g.latPct == 0 && autoMotionGetFwdRate() == 30,
        "the ARM latched them: floor, plan, stop, progress 0, k 100, the cap off, the rate 30 (%u %u %u %u %u %u %u %u)",
        g.floorCm, g.planCm, g.stopCm, g.progCm, g.progPct, g.kLowPct, g.latPct, autoMotionGetFwdRate());
  CHECK(!autonomySettingSet(AUTO_SET_MVKLOW, 60, &why) && strcmp(why, "busy: autonomy active") == 0,
        "mvklow refused while active (%s)", why);
  CHECK(post((const autoCmd_t[]){moveCmd(0, 180, 0)}, &r) && r.result == AUTO_R_OK, "MOVE forward 180 at D 2.0: OK "
        "with no floor (ends 0.20 m; TOO_CLOSE at every setting of before: 0.04 m at k 0.6)");
  CHECK(untilMotionDone(120000), "done (0.30 x 0.20 m = 0.06 m/s at mvwmax 30: ~30 s)");
  autonomyGetStatus(&m);
  CHECK(m.mvState == AUTO_MV_DONE && m.mvEnd == AUTO_MVE_DONE, "... DONE (%s %s)", autoMvStateName((autoMvState_t)m.mvState),
        autoMvEndName((autoMvEnd_t)m.mvEnd));
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  CHECK(autonomySettingReset(AUTO_SET_MVFLOOR, &why) && autonomySettingReset(AUTO_SET_MVFSTOP, &why) &&
        autonomySettingReset(AUTO_SET_MVPROGM, &why) && autonomySettingReset(AUTO_SET_MVPROGR, &why) &&
        autonomySettingReset(AUTO_SET_MVKLOW, &why) && autonomySettingReset(AUTO_SET_MVLAT, &why) &&
        autonomySettingReset(AUTO_SET_MVWMAX, &why), "auto reset the seven");
  run(200);
  /* the VIEW_FAST timer and the EST_RANGE guard as settings (2026-10-22): mvview 0 (the profile's; 150 in this legacy
   * host build), 1..149 refused, 150..1000 taken, latched at the ARM; vzest 0/1, vzestms 0..3000 */
  CHECK(autonomySettingGet(AUTO_SET_MVVIEW) == 0 && autoMotionGetViewDeb() == AUTO_MV_FAST_VIEW_DEB_MS &&
            autonomySettingGet(AUTO_SET_VZEST) == 1 && autonomySettingGet(AUTO_SET_VZESTMS) == 0,
        "mvview 0 (the timer %u ms), vzest 1, vzestms 0 after a boot without them in NVS", autoMotionGetViewDeb());
  CHECK(!autonomySettingSet(AUTO_SET_MVVIEW, 1, &why) && !autonomySettingSet(AUTO_SET_MVVIEW, 149, &why) &&
            !autonomySettingSet(AUTO_SET_MVVIEW, 1001, &why) && !autonomySettingSet(AUTO_SET_VZEST, 2, &why) &&
            !autonomySettingSet(AUTO_SET_VZESTMS, 3001, &why),
        "mvview 1, 149 (below the profile's 150) and 1001, vzest 2, vzestms 3001 refused");
  CHECK(autonomySettingSet(AUTO_SET_MVVIEW, 400, &why) && autonomySettingSet(AUTO_SET_VZEST, 0, &why) &&
            autonomySettingSet(AUTO_SET_VZESTMS, 500, &why) && nvsGet("mvview") == 400 && nvsGet("vzest") == 0 &&
            nvsGet("vzestms") == 500 && autoMotionGetViewDeb() == AUTO_MV_FAST_VIEW_DEB_MS,
        "mvview 400, vzest 0, vzestms 500 (RAM and NVS); the timer in force still the profile's until the ARM");
  /* the scene gate's residual limit (holdres, 2026-10-22): the profile's default (80 in this legacy host build), 1..9
   * and 256 refused, 0 (off) and 120 taken, latched at the ARM into the camera task's limit */
  CHECK(autonomySettingGet(AUTO_SET_HOLDRES) == CAM_HOLD_SCENE_RES_X10_DEFAULT && cameraFlowSceneResidual() == 80 &&
            !autonomySettingSet(AUTO_SET_HOLDRES, 9, &why) && !autonomySettingSet(AUTO_SET_HOLDRES, 1, &why) &&
            !autonomySettingSet(AUTO_SET_HOLDRES, 256, &why) && autonomySettingSet(AUTO_SET_HOLDRES, 120, &why) &&
            nvsGet("holdres") == 120 && cameraFlowSceneResidual() == 80,
        "holdres 80 by default, 1, 9 and 256 refused, 120 stored; the camera's limit still 80 until the ARM");
  /* the left slide's two (mvlatk, mvffr; 2026-10-22): the defaults are the hold and the motions as before, out of range
   * refused, stored values latched at the ARM into the camera task's gain and the motions' feedforward */
  CHECK(autonomySettingGet(AUTO_SET_MVLATK) == 100 && autonomySettingGet(AUTO_SET_MVFFR) == 0 &&
            cameraFlowMoveLatGain() == 100 && autoMotionGetFfRoll() == 0 && !autonomySettingSet(AUTO_SET_MVLATK, 99, &why) &&
            !autonomySettingSet(AUTO_SET_MVLATK, 401, &why) && !autonomySettingSet(AUTO_SET_MVFFR, 51, &why) &&
            !autonomySettingSet(AUTO_SET_MVFFR, -51, &why) && autonomySettingSet(AUTO_SET_MVLATK, 300, &why) &&
            autonomySettingSet(AUTO_SET_MVFFR, -18, &why) && nvsGet("mvlatk") == 300 && nvsGet("mvffr") == -18 &&
            cameraFlowMoveLatGain() == 100 && autoMotionGetFfRoll() == 0,
        "mvlatk 100 / mvffr 0 by default, 99, 401, 51, -51 refused, 300 and -18 stored; the gain and the feedforward in "
        "force unchanged until the ARM");
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting at 1.00 m");
  CHECK(cameraFlowMoveLatGain() == 300 && autoMotionGetFfRoll() == -18,
        "the ARM latched mvlatk 300 and mvffr -18 (%u, %d)", cameraFlowMoveLatGain(), autoMotionGetFfRoll());
  CHECK(autoMotionGetViewDeb() == 400, "the ARM latched mvview: the timer in force 400 ms (%u)", autoMotionGetViewDeb());
  CHECK(cameraFlowSceneResidual() == 120, "the ARM latched holdres: the camera's limit 120 (%u)", cameraFlowSceneResidual());
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  CHECK(autonomySettingReset(AUTO_SET_MVVIEW, &why) && autonomySettingReset(AUTO_SET_VZEST, &why) &&
            autonomySettingReset(AUTO_SET_VZESTMS, &why) && autonomySettingReset(AUTO_SET_HOLDRES, &why) &&
            autonomySettingGet(AUTO_SET_MVVIEW) == 0 && autonomySettingGet(AUTO_SET_VZEST) == 1 &&
            autonomySettingGet(AUTO_SET_VZESTMS) == 0 && autonomySettingGet(AUTO_SET_HOLDRES) == 80 &&
            autonomySettingSet(AUTO_SET_HOLDRES, 0, &why) && autonomySettingReset(AUTO_SET_HOLDRES, &why),
        "auto reset the four (holdres 0, off, is taken too)");
  CHECK(autonomySettingReset(AUTO_SET_MVLATK, &why) && autonomySettingReset(AUTO_SET_MVFFR, &why) &&
            autonomySettingGet(AUTO_SET_MVLATK) == 100 && autonomySettingGet(AUTO_SET_MVFFR) == 0 &&
            autonomySettingSet(AUTO_SET_MVFFR, 50, &why) && autonomySettingSet(AUTO_SET_MVFFR, -50, &why) &&
            autonomySettingSet(AUTO_SET_MVLATK, 400, &why) && autonomySettingReset(AUTO_SET_MVLATK, &why) &&
            autonomySettingReset(AUTO_SET_MVFFR, &why),
        "auto reset mvlatk and mvffr (the range ends 400, 50 and -50 taken)");
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting again");
  CHECK(cameraFlowMoveLatGain() == 100 && autoMotionGetFfRoll() == 0, "the next ARM: mvlatk 100, mvffr 0 again");
  autoMotionGetMargins(&g);
  CHECK(g.floorCm == 30 && g.planCm == 35 && g.stopCm == 25 && g.progCm == 20 && g.progPct == 40 && g.kLowPct == 60 &&
            g.latPct == 50 && autoMotionGetFwdRate() == 15, "the next ARM: the defaults again");
  CHECK(post((const autoCmd_t[]){moveCmd(0, 180, 0)}, &r) && r.result == AUTO_R_RANGE, "MOVE forward 180 at D 2.0: "
        "refused again");
  autonomyGetStatus(&m);
  CHECK(m.mvWhy == AUTO_MVW_TOO_CLOSE, "... TOO_CLOSE (%s)", autoMvWhyName((autoMvWhy_t)m.mvWhy));
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
}

/* The fast steps' acceleration and settle (2026-10-17 "no, I want real 300%", auto_motion.h "the fast steps'
 * acceleration and settle as settings", docs/autonomy-a2b.md 35): NVS mvacc / mvsettle written in IDLE (out of range
 * refused) are latched at the ARM step with mvspeed 300 (autoMotionSetFastCaps before the speed step), refused while
 * active; the forward 1.00 m at D 2 runs faster at mvacc 30 than at the default 20 and its settle times out at 1.0 s;
 * after 'auto reset' the next ARM has 20 / 2500 again. Returns the RUN's ms. */
static uint32_t fastCapsRun(void)
{
  autoCmdResult_t r;
  autoStatusMsg_t m;
  CHECK(post((const autoCmd_t[]){moveCmd(0, 100, 0)}, &r) && r.result == AUTO_R_OK, "MOVE forward 100: OK");
  uint32_t runMs = 0;
  for (int i = 0; i < 20000; i++) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (m.mvState != AUTO_MV_RUN) {
      break;
    }
    runMs++;
  }
  CHECK(untilMotionDone(20000), "done");
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  run(200);
  return runMs;
}

static void scenFastCaps(void)
{
  bootToSession(ESP_RST_POWERON);
  const char *why = "";
  uint8_t acc = 0;
  uint16_t settle = 0;
  CHECK(autonomySettingGet(AUTO_SET_MVACC) == 20 && autonomySettingGet(AUTO_SET_MVSETTLE) == 2500,
        "mvacc 20, mvsettle 2500: the defaults after a boot without them in NVS");
  CHECK(!autonomySettingSet(AUTO_SET_MVACC, 9, &why) && strcmp(why, "range") == 0 &&
        !autonomySettingSet(AUTO_SET_MVACC, 41, &why) && !autonomySettingSet(AUTO_SET_MVSETTLE, 999, &why) &&
        !autonomySettingSet(AUTO_SET_MVSETTLE, 3001, &why) && nvsGet("mvacc") < 0 && nvsGet("mvsettle") < 0,
        "mvacc 9 / 41, mvsettle 999 / 3001 refused (range), nothing stored");
  CHECK(autonomySettingSet(AUTO_SET_MVSPEED, 300, &why), "set mvspeed 300");
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting at 1.00 m (sim, D 2.00 m), the defaults");
  autoMotionGetFastCaps(&acc, &settle);
  CHECK(acc == 20 && settle == 2500, "the ARM latched mvacc %u, mvsettle %u", acc, settle);
  const uint32_t runDef = fastCapsRun();
  CHECK(autonomySettingSet(AUTO_SET_MVACC, 30, &why) && autonomySettingSet(AUTO_SET_MVSETTLE, 1000, &why) &&
        nvsGet("mvacc") == 30 && nvsGet("mvsettle") == 1000 && autonomySettingGet(AUTO_SET_MVACC) == 30 &&
        autonomySettingGet(AUTO_SET_MVSETTLE) == 1000, "set mvacc 30, mvsettle 1000 (RAM and NVS): 'real 300 %%'");
  autoMotionGetFastCaps(&acc, &settle);
  CHECK(acc == 20 && settle == 2500, "IDLE: the motions keep the latched pair until the next ARM");
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting again");
  autoMotionGetFastCaps(&acc, &settle);
  CHECK(acc == 30 && settle == 1000, "the ARM latched mvacc %u, mvsettle %u", acc, settle);
  CHECK(!autonomySettingSet(AUTO_SET_MVACC, 20, &why) && strcmp(why, "busy: autonomy active") == 0 &&
        !autonomySettingSet(AUTO_SET_MVSETTLE, 2500, &why), "mvacc, mvsettle refused while active (%s)", why);
  const uint32_t runFast = fastCapsRun();
  /* forward 1.00 m at D 2.00 at 300: the expansion's peak rule binds (W_FWD 0.17); a larger A_H reaches a slightly
   * higher peak sooner: 6.40 -> 6.20 s (testFastCaps in test_auto_motion.c has the profiles) */
  printf("info: mvspeed 300, forward 1.00 m at D 2: RUN %.3f s at mvacc 20, %.3f s at mvacc 30\n", runDef * 0.001,
         runFast * 0.001);
  CHECK(fabsf((float)runDef * 0.001f - 6.40f) < 0.05f && fabsf((float)runFast * 0.001f - 6.20f) < 0.05f,
        "the RUN at mvacc 30 %.2f s (6.20), at the default %.2f s (6.40, as 200 %%)", runFast * 0.001,
        runDef * 0.001);
  CHECK(autonomySettingReset(AUTO_SET_MVACC, &why) && autonomySettingReset(AUTO_SET_MVSETTLE, &why) &&
        nvsGet("mvacc") < 0 && nvsGet("mvsettle") < 0, "auto reset mvacc, mvsettle");
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting again");
  autoMotionGetFastCaps(&acc, &settle);
  CHECK(acc == 20 && settle == 2500, "the next ARM: the defaults again (%u, %u)", acc, settle);
  autoCmdResult_t r;
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
}

/* A stored mvacc / mvsettle out of range (or of another type) is refused at boot, loudly (the settings' warning), and
 * the default latched: the motions as d3bedee's. */
static void scenFastCapsStored(void)
{
  nvsPut("mvacc", AUTO_NVS_U8, 41);
  nvsPut("mvsettle", AUTO_NVS_U16, 999);
  bootToSession(ESP_RST_POWERON);
  CHECK(autonomySettingGet(AUTO_SET_MVACC) == 20 && autonomySettingGet(AUTO_SET_MVSETTLE) == 2500,
        "stored mvacc 41, mvsettle 999: refused at boot, the defaults (%ld, %ld)",
        (long)autonomySettingGet(AUTO_SET_MVACC), (long)autonomySettingGet(AUTO_SET_MVSETTLE));
  const char *why = "";
  CHECK(autonomySettingSet(AUTO_SET_MVSPEED, 300, &why), "set mvspeed 300");
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting");
  uint8_t acc = 0;
  uint16_t settle = 0;
  autoMotionGetFastCaps(&acc, &settle);
  CHECK(acc == 20 && settle == 2500, "the ARM latched the defaults (%u, %u)", acc, settle);
  autoCmdResult_t r;
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
}

static void scenMvSpeed200(void)
{
  mvSpeedRun(200);
}

static void scenMvSpeed100(void)
{
  mvSpeedRun(100);
}

/* FLIGHT on the plant at 1.00 m: a vertical move reaches the controller (the
 * true height changes), a turn lets the camera height use the guess, LAND. */
static void scenInteractiveFlight(void)
{
  bootToSession(ESP_RST_POWERON);
  yawFree = true;
  vbatStub = 4.1f;
  run(17000);
  vzSimDefaultConfig(&plantCfg);
  plantCfg.kTrue = (float)autonomySettingGet(AUTO_SET_VZK);
  plantCfg.vRest = 4.08f;
  plantCfg.camWakeS = 0.0f;
  vzSimInit(&plant, &plantCfg, 7u);
  plantV = plant.vbat;
  usePlant = true;
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_FLIGHT, 100, 30000), "FLIGHT waiting at 1.00 m");
  const float h0 = plant.h;
  autoCmdResult_t r;
  CHECK(!camVzGlueMotionIn, "waiting: no commanded motion for the observer");
  CHECK(post((const autoCmd_t[]){moveCmd(0, 0, 20)}, &r) && r.result == AUTO_R_OK, "MOVE up 20: OK (%d 0x%x)",
        r.result, (unsigned)r.detail);
  run(200);
  CHECK(camVzGlueMotionIn, "MOVE up: the observer's late offset noise sees a commanded motion");
  CHECK(untilMotionDone(10000), "done");
  run(1500);
  const float h1 = plant.h;
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  printf("info: interactive flight: true height %.3f -> %.3f m (+%.3f for +0.20), estimate %+d cm, status hTarget "
         "%d cm\n", (double)h0, (double)h1, (double)(h1 - h0), m.progUpCm, m.hTargetCm);
  CHECK(h1 - h0 >= 0.15f && h1 - h0 <= 0.25f && m.hTargetCm == 120, "the true height rose %.3f m (0.15..0.25)",
        (double)(h1 - h0));
  CHECK(post((const autoCmd_t[]){moveCmd(30, 0, 0)}, &r) && r.result == AUTO_R_OK, "MOVE right 30 (D 2.0 m): OK");
  CHECK(untilMotionDone(15000), "done");
  /* a turn: the tracker restarts (no G) through the turn, the bob, then a G */
  CHECK(post((const autoCmd_t[]){yawCmd(-45)}, &r) && r.result == AUTO_R_OK, "YAW -45 (left): OK");
  sceneTrackedStub = false;
  sceneResetsStub++;
  run(3000);
  CHECK(guessAllowedStub && autonomyState() == AUTO_HOVER, "turning: the camera height may use the guessed G");
  CHECK(camVzGlueMotionIn, "turning: the observer's late offset noise sees a commanded motion");
  run(5000);
  autonomyGetStatus(&m);
  CHECK((m.mvFlags & AUTO_MVF_BOB) && m.mvState == AUTO_MV_SETTLE && autonomyState() == AUTO_HOVER,
        "the turn done without a G: the re-track bob (flags 0x%x)", m.mvFlags);
  sceneTrackedStub = true;
  sceneDStub = 1.5f;
  CHECK(untilMotionDone(6000), "the bob ends");
  autonomyGetStatus(&m);
  CHECK(m.mvState == AUTO_MV_DONE && m.mvEnd == AUTO_MVE_DONE && !guessAllowedStub && m.sceneDCm == 150 &&
        m.yawOffCdeg == -4500 && fabsf(spYaw - 78.0f) < 1e-3f, "DONE: G again (D 1.50 m), no guess, heading "
        "33 + 45 = %.2f deg", spYaw);
  CHECK(!camVzGlueMotionIn, "DONE: no commanded motion for the observer");
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  printf("info: interactive flight: true peak %.3f m, impact %.2f m/s\n", (double)plant.hMax, (double)plant.impactV);
  CHECK(plant.hMax >= 0.9f && plant.hMax <= 1.5f && plant.impactV <= 0.6f, "true peak %.2f m, touchdown %.2f m/s",
        (double)plant.hMax, (double)plant.impactV);
  CHECK(overrideBad == 0 && maxB2[AUTO_HOVER] <= VZ_THRUST_ABS_MAX, "override exact, FLIGHT max %.0f",
        (double)maxB2[AUTO_HOVER]);
}


/* DRY: a TAKEOFF with a planned move (2026-10-14, docs/autonomy-a2b.md 21.4) through the glue: AUTO_STATUS shows
 * motion 1 RUN at the wait's start (within a step), the move flies on the sim's D 2.0 m to DONE; the sortie's
 * timing (TAKEOFF, HOVER, the wait, the plan's end) printed. */
static void scenPlanDry(void)
{
  bootToSession(ESP_RST_POWERON);
  yawFree = true;
  autoCmdResult_t r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "DRY ARM ok");
  run(100);
  autoCmd_t c = makeCmd(AUTO_CMD_TAKEOFF, 0, r.detail);
  c.heightCm = 100;
  c.actWaitMs = 30000;
  c.moveRightCm = 50;
  c.moveFwdCm = 100;   /* D 2.0 (the sim's): the floor at k 0.6 leaves 0.63 m */
  const uint32_t t0 = nowMs;
  CHECK(post(&c, &r) && r.result == AUTO_R_OK, "TAKEOFF 100 cm with a plan right 0.50 forward 1.00: OK (%d)",
        r.result);
  autoStatusMsg_t m;
  uint32_t tHover = 0, tWait = 0, tRun = 0, tEnd = 0;
  int statusAtWait = -1;
  for (int i = 0; i < 40000 && tEnd == 0; i++) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (!tHover && m.state == AUTO_HOVER) {
      tHover = nowMs;
    }
    if (!tWait && m.state == AUTO_HOVER && (m.flags & AUTO_STF_WAITING)) {
      tWait = nowMs;
      statusAtWait = m.mvSeq;
    }
    if (!tRun && m.mvSeq == 1 && m.mvState == AUTO_MV_RUN) {
      tRun = nowMs;
    }
    if (tRun && (m.mvState == AUTO_MV_DONE || m.mvState == AUTO_MV_STOPPED)) {
      tEnd = nowMs;
    }
    if (m.state == AUTO_LAND || m.state == AUTO_IDLE) {
      break;
    }
  }
  printf("info: DRY plan: TAKEOFF -> HOVER %.2f s, the wait %.2f s (HOVER + %.2f), the plan RUN on AUTO_STATUS %.3f s "
         "after the wait's start (mvSeq %d at its first waiting status), DONE %.2f s after TAKEOFF (the move %.2f s)\n",
         (tHover - t0) * 0.001, (tWait - t0) * 0.001, (tWait - tHover) * 0.001, (tRun - tWait) * 0.001,
         statusAtWait, (tEnd - t0) * 0.001, (tEnd - tRun) * 0.001);
  CHECK(tHover && tWait && tRun && tRun - tWait <= 2 && tEnd, "the plan RUN within a step of the wait's start (%u ms)",
        (unsigned)(tRun - tWait));
  CHECK(m.state == AUTO_HOVER && m.mvState == AUTO_MV_DONE && m.mvEnd == AUTO_MVE_DONE && m.mvSeq == 1 &&
        m.progRightCm == 50 && m.progFwdCm == 100 && m.tgtRightCm == 50 && m.tgtFwdCm == 100 &&
        (m.flags & AUTO_STF_WAITING), "DONE: motion 1, progress 50 / 100 cm (DRY: ideal), still waiting");
  /* the camera height observer's gain follows the move's measured forward progress (2026-10-15): DRY's ideal
   * progress, ln(2.0 / 1.0) at the sim's D 2.0 m, handed to the glue every step */
  CHECK(fabsf(camVzGlueFwdLnIn - logf(2.0f)) < 0.002f, "the observer gets the forward progress ln(D0 / D) %.4f "
        "(ln 2 = 0.6931)", (double)camVzGlueFwdLnIn);
  CHECK(tWait - tHover >= 1000 && tWait - tHover <= 2500, "the wait (and the plan) starts after STABLE 1 s and the "
        "photo: HOVER + %.2f s", (tWait - tHover) * 0.001);
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  CHECK(maxB1[AUTO_HOVER] == 0.0f && maxB2[AUTO_HOVER] == 0.0f, "DRY: both barriers 0");
}

/* The battery falls under load in the wait: LAND BATT_LOW (slow). */
static void scenBattLow(void)
{
  bootToSession(ESP_RST_POWERON);
  vbatStub = 4.1f;
  run(17000);
  vzSimDefaultConfig(&plantCfg);
  plantCfg.kTrue = (float)autonomySettingGet(AUTO_SET_VZK);
  plantCfg.vRest = 4.08f;
  plantCfg.camWakeS = 0.0f;
  vzSimInit(&plant, &plantCfg, 9u);
  plantV = plant.vbat;
  usePlant = true;
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_FLIGHT, 60, 60000), "FLIGHT waiting at 0.60 m");
  run(2000);
  CHECK(autonomyState() == AUTO_HOVER, "hovering at %.2f V loaded", (double)plantV);
  plantCfg.vRest = AUTO_VBAT_LOW_LAND_V + 0.38f;   /* the loaded voltage falls just under AUTO_VBAT_LOW_LAND_V */
  autoStatusMsg_t m;
  uint32_t t = 0;
  for (; t < 3000 && autonomyState() == AUTO_HOVER; t++) {
    linkBackground();
    loopOnce();
  }
  autonomyGetStatus(&m);
  printf("info: battery low: loaded %.3f V, LAND %s %u ms after the drop\n", (double)plantV,
         autoReasonName((autoReason_t)m.reason), (unsigned)t);
  /* The end of battery (user decision 2026-10-13, late): the floor, 3.00 V for 0.5 s (or a T_hold over 57000),
   * lands it BATT_LOW the slow way (with the 3.10 V level of before, BATT_LOW, VBAT_LOW or the controller's own
   * LAND, up to 1.2 m/s, were accepted here). */
  CHECK(m.state == AUTO_LAND && m.reason == AUTO_RS_BATT_LOW && t <= 3000, "loaded %.2f V: LAND %s after %u ms",
        (double)plantV, autoReasonName((autoReason_t)m.reason), (unsigned)t);
  const bool landed = runUntil(AUTO_IDLE, 10000);
  printf("info: battery low: touchdown %.2f m/s\n", (double)plant.impactV);
  CHECK(landed && plant.impactV <= 0.6f, "landed slowly (%.2f <= 0.6 m/s)", (double)plant.impactV);
}

/* TAKEOFF 1.20 m (the take-off cap; 2.00 m is reached by moves up), a 60 s wait from the TAKEOFF: the wait starts
 * inside the pre-wait limit (+25 ms per cm above 0.60 m: +1.5 s), the Mac's LAND 60 s later (HOVER_TIMEOUT inside a
 * 120 s air clock until 2026-10-15); the true peak; a soft touchdown from there. */
static void marginHighRun(uint16_t cm, float kFactor)
{
  const float hM = (float)cm * 0.01f;
  bootToSession(ESP_RST_POWERON);
  vbatStub = 4.1f;
  run(17000);
  vzSimDefaultConfig(&plantCfg);
  plantCfg.kTrue = (float)autonomySettingGet(AUTO_SET_VZK) * kFactor;
  plantCfg.vRest = 4.08f;
  plantCfg.camWakeS = 0.0f;
  vzSimInit(&plant, &plantCfg, 99u);
  plantV = plant.vbat;
  usePlant = true;
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_FLIGHT, cm, 60000), "waiting at %.2f m", (double)hM);
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  const uint32_t waitAir = m.airMs;
  const uint32_t preLimit = AUTO_AIR_MAX_DEFAULT_MS + (uint32_t)(cm - 60u) * AUTO_AIR_PER_CM_ABOVE_60_MS;
  uint32_t waitAir2;
  const uint32_t macAt = hoverToMacLand(60000u, &waitAir2);
  autonomyGetStatus(&m);
  /* airMs saturates at 65535: the air clock from airDs (0.1 s, byte 118) */
  const uint32_t landAir = (uint32_t)m.airDs * 100u;
  const uint8_t why = m.reason;
  CHECK(m.airMs == 65535 && m.airLeftDs == 0, "AUTO_STATUS airMs saturates at 65535 ms (%u), airLeftDs 0 (no air "
        "clock)", m.airMs);
  const float peak = plant.hMax;
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  printf("info: margin %u cm K x%.2f: hover wait at air %u ms (pre-wait limit %u), LAND at air %u ms (reason %d), "
         "true peak %.3f m, touchdown %.2f m/s\n", (unsigned)cm, (double)kFactor, (unsigned)waitAir, (unsigned)preLimit,
         (unsigned)landAir, why, (double)peak, (double)plant.impactV);
  CHECK(waitAir > 0 && waitAir + 1000u <= preLimit, "%.2f m: the wait starts at air %u ms, >= 1 s inside %u",
        (double)hM, (unsigned)waitAir, (unsigned)preLimit);
  CHECK(macAt == 60000u && why == AUTO_RS_LAND_CMD && landAir + 250u >= waitAir + 60000u &&
        landAir <= waitAir + 60250u, "the Mac's LAND 60 s into the wait (at air %u, airDs %u)", (unsigned)landAir,
        m.airDs);
  CHECK(peak >= hM - 0.20f && peak <= hM + 0.30f && plant.impactV <= 0.6f, "true peak %.3f m (%.1f..%.1f), "
        "touchdown %.2f m/s (<= 0.6)", (double)peak, (double)(hM - 0.20f), (double)(hM + 0.30f),
        (double)plant.impactV);
}
static void scenMargin120(void) { marginHighRun(120, 1.00f); }
static void scenMargin120h(void) { marginHighRun(120, 1.03f); }

/* The two vzK writers (R7): the console's 'auto set vzK' and the aux task's
 * learned-vzK save never interleave; RAM and NVS end equal. */
static bool consoleOk;
static const char *consoleWhy;

static void hookConsoleSetK(void)
{
  consoleWhy = "";
  consoleOk = autonomySettingSet(AUTO_SET_VZK, 150000, &consoleWhy);
}

static void hookPoll(void)
{
  autonomyPoll();
}

static void scenLearnConsole(void)
{
  bootToSession(ESP_RST_POWERON);
  flightRun_t fr;
  flightSortie(1.03f, 1, &fr);
  pollOn = false;   /* from the landing (release hold: nothing stored yet); the ARM needed its heap reading */
  run(1000);   /* past the release hold */
  const int fw0 = flashWrites;
  afterWrite = hookConsoleSetK;   /* the console between the aux task's NVS write and its RAM copy */
  autonomyPoll();
  const int32_t k = autonomySettingGet(AUTO_SET_VZK);
  CHECK(flashWrites - fw0 == 1 && k != fr.k0, "the learned vzK stored (%ld)", (long)k);
  CHECK(!consoleOk && strcmp(consoleWhy, "busy: autonomy active") == 0, "the console's set meanwhile: busy (%s)",
        consoleWhy);
  CHECK(nvsGet("vzK") == k, "RAM %ld == NVS %ld", (long)k, (long)nvsGet("vzK"));
  pollOn = true;
  run(6000);
  CHECK(flashWrites - fw0 == 1, "nothing more to store");
}

static void scenLearnPoll(void)
{
  bootToSession(ESP_RST_POWERON);
  flightRun_t fr;
  flightSortie(1.03f, 1, &fr);
  pollOn = false;   /* as scenLearnConsole */
  run(1000);
  const int fw0 = flashWrites;
  afterWrite = hookPoll;   /* the aux task between the console's NVS write and its RAM copy */
  const char *why = "";
  CHECK(autonomySettingSet(AUTO_SET_VZK, 150000, &why), "console set vzK 150000 (%s)", why);
  CHECK(autonomySettingGet(AUTO_SET_VZK) == 150000 && nvsGet("vzK") == 150000, "RAM %ld == NVS %ld == 150000",
        (long)autonomySettingGet(AUTO_SET_VZK), (long)nvsGet("vzK"));
  pollOn = true;
  run(6000);
  CHECK(flashWrites - fw0 == 1, "the console's value cancelled the learned one (%d writes)", flashWrites - fw0);
  CHECK(autonomySettingGet(AUTO_SET_VZK) == 150000 && nvsGet("vzK") == 150000, "still 150000");
}

/* Reset-lock: after a software reset BENCH is refused LOCKS, DRY is not. */
static void scenResetLock(void)
{
  bootToSession(ESP_RST_SW);
  CHECK(autonomyResetLocked() && (statusFlags() & AUTO_STF_RESET_LOCKED), "reset-locked after a software reset");
  gcapStub = 15000;
  autoCmdResult_t r = arm(AUTO_MODE_BENCH);
  CHECK(refusedLocks(&r), "BENCH refused LOCKS (%d 0x%x)", r.result, (unsigned)r.detail);
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "DRY accepted (%d 0x%x)", r.result, (unsigned)r.detail);
  disarm();
}

/* The reset-lock's USB unlock (usb_diag.c 'auto unlock', user decision
 * 2026-10-12 evening): after a non-power-on reset FLIGHT is refused LOCKS;
 * the unlock is refused while a sortie is active (ARMED, HOVER, the release
 * hold), while an ARM waits in the mailbox and while a motor runs, with
 * nothing changed; in IDLE with the motors stopped it clears the reset-lock
 * as a battery power-on does (an ARM taken meanwhile is refused LOCKS) and a
 * FLIGHT ARM is accepted; DRY is accepted throughout; the reset reason stays. */
static bool unlockRefused(const char *expectWhy, const char *what)
{
  bool was = false;
  const char *why = "";
  const bool ok = autonomyUsbUnlock(&was, &why);
  CHECK(!ok && strcmp(why, expectWhy) == 0 && was && autonomyResetLocked() &&
        (statusFlags() & AUTO_STF_RESET_LOCKED),
        "%s: unlock refused '%s', still locked (ok %d, why '%s', was %d)", what, expectWhy, ok, why, was);
  return !ok;
}

static void scenUsbUnlock(void)
{
  bootToSession(ESP_RST_SW);
  vbatStub = 4.1f;
  run(17000);   /* FLIGHT needs 20 s on the ground */
  gcapStub = 0;
  CHECK(autonomyResetLocked() && (statusFlags() & AUTO_STF_RESET_LOCKED), "reset-locked after a software reset");
  autoCmdResult_t r = arm(AUTO_MODE_FLIGHT);
  CHECK(refusedLocks(&r) && r.detail == AUTO_G_LOCKS, "FLIGHT ARM refused, LOCKS only: the reset-lock (%d 0x%x)",
        r.result, (unsigned)r.detail);

  /* DRY is exempt; no unlock while a sortie is active. */
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK && autonomyState() == AUTO_ARMED, "DRY ARM accepted while reset-locked (%d 0x%x)",
        r.result, (unsigned)r.detail);
  unlockRefused("busy: autonomy active", "ARMED");
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "DRY TAKEOFF (%d)", r.result);
  CHECK(runUntil(AUTO_HOVER, 8000), "DRY HOVER");
  unlockRefused("busy: autonomy active", "HOVER");
  CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK, "LAND (%d)", r.result);
  CHECK(runUntil(AUTO_IDLE, 10000), "DRY back in IDLE");
  CHECK(autonomyThrustMax() == 0.0f, "in the release hold");
  unlockRefused("busy: autonomy active", "release hold");
  run(700);

  /* An ARM waiting in the mailbox: refused, and that ARM still sees the lock. */
  gesture();
  const autoCmd_t c = makeCmd(AUTO_CMD_ARM, AUTO_MODE_FLIGHT, 0);
  uint32_t t;
  CHECK(autonomyPostCommand(&c, &t), "FLIGHT ARM posted");
  unlockRefused("busy: autonomy active", "an ARM in the mailbox");
  bool took = false;
  for (int i = 0; i < 3 && !took; i++) {
    loopOnce();
    took = autonomyTakeResult(t, &r);
  }
  CHECK(took && refusedLocks(&r), "that FLIGHT ARM: refused LOCKS (%d 0x%x)", r.result, (unsigned)r.detail);

  /* Manual flight in IDLE (the phone's throttle up): a motor runs. */
  appThrust = 30000;
  run(200);
  CHECK(motorsAnyDutyNonZero() && autonomyState() == AUTO_IDLE, "manual flight: a motor runs, IDLE");
  unlockRefused("a motor output is nonzero", "manual flight");
  appThrust = 0;
  run(21000);

  /* IDLE, motors stopped: unlocked. A DRY ARM taken while the unlock holds the
   * interlock is refused LOCKS (a sortie never starts on a half-done change). */
  gesture();
  nextUnlock = hookPostArm;   /* at the release of the lock the interlock is taken under */
  bool was = false;
  const char *why = "";
  CHECK(autonomyUsbUnlock(&was, &why) && was && !autonomyResetLocked(), "IDLE, motors stopped: unlocked (was %d, %s)",
        was, why);
  CHECK(nextUnlock == NULL && hookTook && refusedLocks(&hookRes),
        "the DRY ARM taken during the unlock: refused LOCKS (%d 0x%x)", hookRes.result, (unsigned)hookRes.detail);
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  CHECK(!rv.locked && rv.resetReason == ESP_RST_SW,
        "auto rtc: locked 0, the reset reason still software (%d)", rv.resetReason);
  run(200);
  CHECK(!(statusFlags() & AUTO_STF_RESET_LOCKED), "AUTO_STATUS: the reset-locked flag cleared");
  CHECK(autonomyUsbUnlock(&was, &why) && !was && !autonomyResetLocked(), "a second unlock: OK, nothing to do");

  r = arm(AUTO_MODE_FLIGHT);
  CHECK(r.result == AUTO_R_OK && autonomyState() == AUTO_ARMED, "FLIGHT ARM accepted after the unlock (%d 0x%x)",
        r.result, (unsigned)r.detail);
  disarm();
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "DRY still accepted (%d 0x%x)", r.result, (unsigned)r.detail);
  disarm();
  CHECK(autonomySettingSet(AUTO_SET_AUTOWAIT, 6000, &why), "the interlock was released: a setting writes (%s)", why);
  CHECK(lockDepth == 0, "locks balanced");
}

/* A battery power-on boot is not locked: the unlock has nothing to do. */
static void scenUsbUnlockPowerOn(void)
{
  bootToSession(ESP_RST_POWERON);
  bool was = true;
  const char *why = "";
  CHECK(!autonomyResetLocked() && autonomyUsbUnlock(&was, &why) && !was && !autonomyResetLocked(),
        "power-on boot: unlock OK, was 0 (%s)", why);
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  CHECK(!rv.locked && rv.resetReason == ESP_RST_POWERON, "rst poweron");
}

/* G9 (review F4 of 0cf6fbf): the internal heap is sampled by the aux task
 * (autonomyPoll); an ARM or TAKEOFF reads that sample. Too little refuses;
 * so do a sample older than 200 ms (the aux task stalled after the heap
 * dropped: the last sample still showed plenty) and no sample at all (no
 * aux task since boot). An old value never passes. */
static bool refusedHeap(const autoCmdResult_t *r)
{
  return r->result == AUTO_R_GUARDS && r->detail == AUTO_G_HEAP;
}

static void scenHeap(void)
{
  /* no aux task since boot: no sample */
  pollOn = false;
  bootToSession(ESP_RST_POWERON);
  autoCmdResult_t r = arm(AUTO_MODE_DRY);
  CHECK(refusedHeap(&r) && autonomyState() == AUTO_IDLE, "no heap sample since boot: ARM refused HEAP (%d 0x%x)",
        r.result, (unsigned)r.detail);
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  CHECK(m.failedGuards & AUTO_G_HEAP, "IDLE status shows HEAP (0x%x)", (unsigned)m.failedGuards);

  /* the aux task runs: a fresh sample of 20000 B */
  pollOn = true;
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "fresh sample, 20000 B: ARM ok (%d 0x%x)", r.result, (unsigned)r.detail);
  disarm();

  /* the heap drops and the aux task sees it */
  heapStub = AUTO_HEAP_MIN - 1;
  r = arm(AUTO_MODE_DRY);
  CHECK(refusedHeap(&r), "fresh sample, %u B: ARM refused HEAP (%d 0x%x)", (unsigned)heapStub, r.result,
        (unsigned)r.detail);
  heapStub = 20000;
  run(20);

  /* the heap drops right after a sample and the aux task stalls: within
   * 200 ms the last sample (20000 B) still counts, after that it fails */
  gesture();
  pollOn = false;
  heapStub = AUTO_HEAP_MIN - 1;
  run(150);
  CHECK(command(AUTO_CMD_ARM, AUTO_MODE_DRY, 0, &r, 3) && r.result == AUTO_R_OK,
        "a sample at most 160 ms old counts: ARM ok (%d 0x%x)", r.result, (unsigned)r.detail);
  disarm();
  CHECK(autonomyState() == AUTO_IDLE, "disarmed");
  r = arm(AUTO_MODE_DRY);
  CHECK(refusedHeap(&r), "aux task stalled after the drop: the 20000 B sample is stale, ARM refused HEAP (%d 0x%x)",
        r.result, (unsigned)r.detail);

  /* a stall alone refuses too (fail closed), until the aux task samples again */
  heapStub = 20000;
  r = arm(AUTO_MODE_DRY);
  CHECK(refusedHeap(&r), "aux task stalled, heap fine: ARM refused HEAP (%d 0x%x)", r.result, (unsigned)r.detail);
  pollOn = true;
  r = arm(AUTO_MODE_DRY);
  CHECK(r.result == AUTO_R_OK, "sampling again: ARM ok (%d 0x%x)", r.result, (unsigned)r.detail);

  /* ARMED, then the aux task stalls: TAKEOFF refused HEAP, still ARMED */
  const uint32_t token = r.detail;
  pollOn = false;
  run(250);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, token, &r, 3) && refusedHeap(&r) && autonomyState() == AUTO_ARMED,
        "TAKEOFF on a stale sample: refused HEAP (%d 0x%x, state %d)", r.result, (unsigned)r.detail,
        autonomyState());
  pollOn = true;
  run(20);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, token, &r, 3) && r.result == AUTO_R_OK, "TAKEOFF once it samples again (%d)",
        r.result);
  killAndWait();
  CHECK(lockDepth == 0, "locks balanced");
}

/* Take-off integrator hold (autonomyTakeoffIntegratorHold, read by
 * controller_pid.c; user decision 2026-10-13, evening): never in manual
 * flight; in a FLIGHT sortie from the spin-up to the step before the
 * controller's lift-off, never from that step on; a sortie without a latched
 * lift-off (a plant too heavy to leave the ground: the same rule as one that
 * left it undetected) holds until its RAMP reaches the plateau and never
 * after it (not in the float, not in the abort LAND); a KILL in the ramp
 * releases it at once (vert_ctrl CUT) and IDLE clears it.
 * test_controller_pid_ihold.c shows what the real controller does with it
 * (the four roll/pitch integrators exactly 0 while it holds, manual flight
 * bit-identical). On e46bf90 this does not build: the getter does not
 * exist. */
static void scenIhold(void)
{
  bootToSession(ESP_RST_POWERON);
  ihTrack = true;
  appOnline = true;            /* manual flight in IDLE: never held */
  appThrust = 40000;
  run(2000);
  appThrust = 0;
  run(3000);
  CHECK(ihHeldTotal == 0, "manual flight at 40000: never held (%d loops)", ihHeldTotal);

  flightRun_t fr;
  flightSortie(1.03f, 1, &fr);
  CHECK(fr.hover && liftoffNotes == 1, "FLIGHT on the plant: HOVER, lift-off noted (%d)", liftoffNotes);
  CHECK(ihTakeoffSpin > 0 && ihFreeBeforeLift == 0, "held on every spinning TAKEOFF loop before the lift-off (%d of "
        "%d free)", ihFreeBeforeLift, ihTakeoffSpin);
  CHECK(ihHeldAfterLift == 0, "released from the lift-off step on (%d held loops after it)", ihHeldAfterLift);
  CHECK(ihHeldHigh > 300 && ihHeldHigh == ihHighBeforeLift, "held on every loop above the 26000 ground gate before "
        "the lift-off (%d of %d)", ihHeldHigh, ihHighBeforeLift);
  CHECK(ihHeldIdle == 0 && !autonomyTakeoffIntegratorHold(), "IDLE: never held (%d)", ihHeldIdle);
  CHECK(!ihReleased && ihMaxRampThrust > 26000.0f, "no release before the lift-off (the plateau not reached: ramp "
        "max %.0f)", (double)ihMaxRampThrust);
  printf("info: ihold: %d loops above 26000 before the lift-off, all held\n", ihHeldHigh);

  /* too heavy to lift off (as an undetected lift-off to the glue): RAMP to its plateau, NO_LIFTOFF, LAND, IDLE */
  run(3000);
  ihHeldTotal = ihHeldHigh = ihTakeoffSpin = ihFreeBeforeLift = ihHighBeforeLift = 0;
  ihHeldAfterRelease = ihFreeBeforeRelease = ihHeldLand = 0;
  ihReleased = ihPrevHeld = false;
  ihRelThrust = ihMaxRampThrust = 0.0f;
  liftoffNotes = 0;
  flightSortie(1.40f, 1, &fr);
  CHECK(!fr.hover && liftoffNotes == 0, "heavy plant: no lift-off (%d)", liftoffNotes);
  CHECK(ihTakeoffSpin > 0 && ihFreeBeforeRelease == 0 && ihHeldHigh > 300,
        "heavy plant: held on every spinning loop until the release (%d free before it, %d held above 26000)",
        ihFreeBeforeRelease, ihHeldHigh);
  /* the plateau (kPrior / Vf) still rises a little after it as the battery sags */
  CHECK(ihReleased && ihRelThrust >= 0.97f * ihMaxRampThrust && ihMaxRampThrust > 30000.0f,
        "heavy plant: released at the RAMP's plateau (%.0f of its %.0f)", (double)ihRelThrust,
        (double)ihMaxRampThrust);
  CHECK(ihHeldAfterRelease == 0 && ihHeldLand == 0 && ihFreeBeforeLift > 500,
        "heavy plant: never held after it, in the float or the abort LAND (%d, %d held; %d free spinning TAKEOFF loops)",
        ihHeldAfterRelease, ihHeldLand, ihFreeBeforeLift);
  CHECK(ihHeldIdle == 0 && !autonomyTakeoffIntegratorHold(), "heavy plant: IDLE clears it (%d)", ihHeldIdle);
  printf("info: ihold heavy plant: released at %.0f (plateau %.0f), %d spinning TAKEOFF loops after it\n",
         (double)ihRelThrust, (double)ihMaxRampThrust, ihFreeBeforeLift);

  /* KILL in the ramp (the plant set up as flightSortie does) */
  vzSimInit(&plant, &plantCfg, 7u);
  plantV = plant.vbat;
  run(21000);                  /* G2 STILL: >= 20 s on the ground before a FLIGHT ARM */
  autoCmdResult_t r = arm(AUTO_MODE_FLIGHT);
  CHECK(r.result == AUTO_R_OK, "ARM ok (%d 0x%x)", r.result, (unsigned)r.detail);
  CHECK(command(AUTO_CMD_TAKEOFF, 0, r.detail, &r, 3) && r.result == AUTO_R_OK, "TAKEOFF");
  for (int i = 0; i < 6000 && dutyThrust < 30000.0f; i++) {
    linkBackground();
    loopOnce();
  }
  CHECK(dutyThrust >= 30000.0f && autonomyTakeoffIntegratorHold() && liftoffNotes == 0, "ramp at %.0f: held",
        (double)dutyThrust);
  autonomyKillFromLink();
  loopOnce();
  CHECK(dutyThrust == 0.0f && !autonomyTakeoffIntegratorHold(), "KILL: motors 0, released at once");
  runUntil(AUTO_IDLE, 5000);
  loopOnce();
  CHECK(!autonomyTakeoffIntegratorHold() && ihHeldIdle == 0, "after the KILL, IDLE: not held (%d)", ihHeldIdle);
  ihTrack = false;
  run(2100);
  CHECK(lockDepth == 0, "locks balanced");
}

/* ---- main --------------------------------------------------------------------- */

/* A firm tap on the frame in DRY HOVER (|acc| 3 g for 4 ms, the bench test of docs/autonomy-a2.md 12.7). Since
 * 2026-10-15 ("Claude decides", docs/autonomy-a2b.md 22; LAND COLLISION from 2026-10-13): no LAND, the hold
 * and report: the CONTACT event in AUTO_STATUS (detail the controller's phase, HOLD), the hush bit for 0.5 s, the
 * contact count, the hold not blocked, no transition in the RTC record; a second tap 0.8 s later hushes again,
 * a third within 2 s of the first is reported without a hush (the chain bound). The Mac's LAND then: a further tap
 * in the LAND changes nothing, IDLE TOUCHDOWN; DRY: both barriers 0. */
static void scenCollisionDry(void)
{
  bootToSession(ESP_RST_POWERON);
  yawFree = true;
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 30000), "DRY waiting at 1.00 m (sim)");
  run(500);
  CHECK(!autonomyHoldBlocked() && autonomyState() == AUTO_HOVER, "hovering, the hold not blocked");
  autonomyRtcView_t rv0;
  autonomyGetRtc(&rv0);
  autoStatusMsg_t m;
  int evLoop = -1;
  bool hushSeen = false, hushMotion = false;
  CHECK(!camVzGlueMotionIn, "hovering: no motion or hush for the observer");
  for (int i = 0; i < 4; i++) {
    tapG = 2.0f;   /* |acc| 3 g */
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (evLoop < 0 && m.evCode == AUTO_EV_CONTACT) {
      evLoop = i;
      hushSeen = (m.holdBits & AUTO_HB_HUSH) != 0;
      hushMotion = camVzGlueMotionIn;
    }
  }
  CHECK(hushMotion, "the contact's hush: a guard event of the observer's late offset noise (a DRY sortie sets it too)");
  tapG = 0.0f;
  autonomyGetStatus(&m);
  CHECK(evLoop == 3 && hushSeen && m.state == AUTO_HOVER && m.evCount >= 1 && (m.evDetail & 0x3Fu) == VZ_HOLD &&
        m.contacts == 1 && !autonomyHoldBlocked(),
        "tap held 3 ms: the CONTACT event on loop %d (pushed at once), the hush bit, HOVER, detail 0x%02x, contacts "
        "%u, the hold not blocked (%s; hush at the event %d, bits now 0x%02x)", evLoop, m.evDetail, m.contacts,
        autoStateName((autoState_t)m.state), hushSeen, m.holdBits);
  CHECK(m.peakAccCg >= 290 && m.sinceCmdDs == 65535, "the contact's peak |acc| %u cg (3 g), no motion command yet",
        m.peakAccCg);
  run(600);
  autonomyGetStatus(&m);
  CHECK(m.state == AUTO_HOVER && !(m.holdBits & AUTO_HB_HUSH), "0.6 s later: the hush over, still HOVER (bits 0x%02x)",
        m.holdBits);
  CHECK(!camVzGlueMotionIn, "the hush over: no guard event for the observer");
  run(200);   /* 0.8 s after the first */
  for (int k = 0; k < 2; k++) {
    for (int i = 0; i < 4; i++) {
      tapG = 2.0f;
      linkBackground();
      loopOnce();
    }
    tapG = 0.0f;
    autonomyGetStatus(&m);
    if (k == 0) {
      CHECK(m.contacts == 2 && m.evCode == AUTO_EV_CONTACT && !(m.evDetail & AUTO_CONTACT_NO_HUSH) &&
            (m.holdBits & AUTO_HB_HUSH), "a second tap 0.8 s later: hushed again (contacts %u, detail 0x%02x)",
            m.contacts, m.evDetail);
      run(600);
    } else {
      CHECK(m.contacts == 3 && m.evCode == AUTO_EV_CONTACT && (m.evDetail & AUTO_CONTACT_NO_HUSH) &&
            !(m.holdBits & AUTO_HB_HUSH), "a third within 2 s of the first: reported, no hush (the chain bound; "
            "contacts %u, detail 0x%02x)", m.contacts, m.evDetail);
    }
  }
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  CHECK(autonomyState() == AUTO_HOVER && rv.n == rv0.n, "three contacts: still HOVER, no transition recorded (%u, %u)",
        rv.n, rv0.n);
  autoCmdResult_t r;
  CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK && autonomyState() == AUTO_LAND, "the Mac's LAND");
  int loops = 0;
  for (; loops < 20000 && autonomyState() == AUTO_LAND; loops++) {
    if (loops == 300) {
      tapG = 3.0f;   /* a further contact in the LAND */
    } else if (loops == 310) {
      tapG = 0.0f;
    }
    linkBackground();
    loopOnce();
  }
  autonomyGetStatus(&m);
  printf("info: DRY taps at 1.00 m (sim): contacts %u; the Mac's LAND to IDLE in %.1f s\n", m.contacts, loops * 0.001f);
  CHECK(autonomyState() == AUTO_IDLE && m.reason == AUTO_RS_TOUCHDOWN && !autonomyHoldBlocked(),
        "a tap in the LAND changes nothing: IDLE TOUCHDOWN after %.1f s (%s)", loops * 0.001f,
        autoReasonName((autoReason_t)m.reason));
  CHECK(maxB1[AUTO_LAND] == 0.0f && maxB2[AUTO_LAND] == 0.0f && maxB2[AUTO_HOVER] == 0.0f && estopSets == 0,
        "DRY: both barriers 0, no emergency stop");
}

/* ---- a palm under the hover (the palm cue of 2026-10-14 removed 2026-10-15, docs/autonomy-a2b.md 22) ------- */

/* A FLIGHT sortie on the plant (as flightSortie, 3 % heavier than stored: it learns K) to HOVER and 2.5 s of hold */
static void palmFlightToHover(int32_t *k0)
{
  bootToSession(ESP_RST_POWERON);
  memset(&palm, 0, sizeof(palm));
  vbatStub = 4.1f;
  run(17000);
  *k0 = autonomySettingGet(AUTO_SET_VZK);
  vzSimDefaultConfig(&plantCfg);
  plantCfg.kTrue = (float)*k0 * 1.03f;
  plantCfg.vRest = 4.08f;
  plantCfg.camWakeS = 0.0f;
  vzSimInit(&plant, &plantCfg, 99u);
  plantV = plant.vbat;
  usePlant = true;
  run(200);
  autoCmdResult_t r = arm(AUTO_MODE_FLIGHT);
  CHECK(r.result == AUTO_R_OK, "FLIGHT ARM ok (%d 0x%x)", r.result, (unsigned)r.detail);
  run(100);
  takeoffCm = 70;
  autoCmd_t c = makeCmd(AUTO_CMD_TAKEOFF, 0, r.detail);
  takeoffCm = 40;
  c.actWaitMs = 30000;
  uint32_t t;
  CHECK(autonomyPostCommand(&c, &t), "TAKEOFF posted");
  loopOnce();
  CHECK(autonomyTakeResult(t, &r) && r.result == AUTO_R_OK, "TAKEOFF ok (%d)", r.result);
  CHECK(runUntil(AUTO_HOVER, 10000), "HOVER");
  run(2500);
  CHECK(autonomyState() == AUTO_HOVER, "still hovering after 2.5 s");
}

/* The user puts the drone down from the palm (the aux task stores a learned vzK only with the drone at rest on the
 * ground: stabilizerOnGround()) */
static void palmPutDown(void)
{
  palm.on = false;
  plant.h = 0.0f;
  plant.v = 0.0f;
  plant.onGround = true;
  plant.accZ = 1.0f;
}

/* Steps until IDLE; returns the LAND's thrust record: the first LAND loop's vzThrust and the lowest nonzero one
 * (no probe since 2026-10-15: the LAND's floor is 0.65 x T_ref), and when the motors stopped */
static void palmLandWatch(float *first, float *lowest, uint32_t *offMs, bool *blockedAtLand)
{
  *first = 0.0f;
  *lowest = 1e9f;
  *offMs = 0;
  *blockedAtLand = false;
  bool inLand = false;
  for (int i = 0; i < 20000 && autonomyState() != AUTO_IDLE; i++) {
    linkBackground();
    loopOnce();
    autoRecFields_t rf;
    autonomyRecordFields(&rf);
    if (autonomyState() == AUTO_LAND && !inLand) {
      inLand = true;
      *first = rf.vzThrust;
      *blockedAtLand = autonomyHoldBlocked();
    }
    if (inLand && rf.vzThrust > 0 && rf.vzThrust < *lowest) {
      *lowest = rf.vzThrust;
    }
    if (inLand && *offMs == 0 && dutyThrust == 0.0f) {
      *offMs = nowMs;
    }
  }
}

/* A soft palm (k 200 N/m, c 2 N s/m) brought up at 0.05 m/s under the FLIGHT hover at 0.70 m, lifting 3 cm. Until
 * 2026-10-15 the PALM cue landed (and probed); now nothing on the drone lands for it: it stays in HOVER on the
 * palm (the camera height reads the lift; the controller holds and reports, or lands K_RANGE at its K floor, a
 * physical fallback), never KILLED, for the Mac to see it (a photo) and LAND. The Mac's LAND 3 s after the palm
 * came up: IDLE TOUCHDOWN on the palm (the LAND's stall rule and FORCE), the hold's K learned. */
static void scenPalmFlight(void)
{
  int32_t k0;
  palmFlightToHover(&k0);
  const int fw0 = flashWrites;
  palm.on = true;
  palm.top = plant.h - 0.05f;
  palm.v = 0.05f;
  palm.lift = 0.03f;
  palm.k = 200.0f;
  palm.c = 2.0f;
  palm.contactMs = -1.0f;
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  const uint16_t tHold = m.tHold;
  bool left = false;
  for (int i = 0; i < 3000 && !left; i++) {
    linkBackground();
    loopOnce();
    left = autonomyState() != AUTO_HOVER;
  }
  autonomyGetStatus(&m);
  printf("info: palm under the FLIGHT hover: after 3 s %s %s (vz %s %s), events %u (latest %s 0x%02x), hold bits "
         "0x%02x, contacts %u, the plant on the palm %d at %.3f m, thrust %u (T_hold %u)\n",
         autoStateName((autoState_t)m.state), autoReasonName((autoReason_t)m.reason),
         vzPhaseName((vzPhase_t)m.vzPhase), vzReasonName((vzReason_t)m.vzReason), m.evCount,
         autoEventName((autoEvent_t)m.evCode), m.evDetail, m.holdBits, m.contacts, palm.held, (double)plant.h,
         m.thrust, tHold);
  CHECK(autonomyState() != AUTO_KILLED && estopSets == 0 &&
            (autonomyState() == AUTO_HOVER || (m.reason == AUTO_RS_VZ && m.vzLandReason == VZ_R_K_RANGE)),
        "the palm: no LAND by a cue, HOVER (or the controller's K-floor LAND), never KILLED (%s %s)",
        autoStateName((autoState_t)m.state), autoReasonName((autoReason_t)m.reason));
  CHECK(palm.contactMs >= 0.0f, "the palm touched the drone (on it now: %d, at %.3f m)", palm.held, (double)plant.h);
  if (autonomyState() == AUTO_HOVER) {
    autoCmdResult_t r;
    CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK, "the Mac's LAND (%d)", r.result);
  }
  float first, lowest;
  uint32_t offMs;
  bool blocked;
  palmLandWatch(&first, &lowest, &offMs, &blocked);
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  autonomyGetStatus(&m);
  const uint8_t vzWhy = rv.n > 0 ? rv.trans[rv.n - 1].vzReason : 0;
  printf("info: palm flight: the LAND's thrust %.0f at its start, lowest %.0f (%.2f x T_hold), end %s (vz %s), the "
         "plant on the palm %d at %.3f m\n", (double)first, (double)lowest,
         (double)(lowest / (float)(tHold ? tHold : 1)), autoReasonName((autoReason_t)m.reason),
         vzReasonName((vzReason_t)vzWhy), palm.held, (double)plant.h);
  CHECK(autonomyState() == AUTO_IDLE && m.reason == AUTO_RS_TOUCHDOWN && estopSets == 0 &&
            (vzWhy == VZ_R_TOUCHDOWN_SUPPORT || vzWhy == VZ_R_TOUCHDOWN_FORCE || vzWhy == VZ_R_TOUCHDOWN_JOLT),
        "IDLE TOUCHDOWN (vz %s), never KILLED", vzReasonName((vzReason_t)vzWhy));
  CHECK(palm.held && plant.h > 0.5f, "the drone on the palm at %.2f m", (double)plant.h);
  palmPutDown();
  run(3000);
  const int32_t k1 = autonomySettingGet(AUTO_SET_VZK);
  printf("info: palm flight: vzK %ld -> %ld (%d writes)\n", (long)k0, (long)k1, flashWrites - fw0);
  CHECK(lockDepth == 0, "locks balanced");
}

/* A tap in the FLIGHT hover (|acc| 3 g for 4 ms) with the camera height in use, then a palm held still under the
 * drone. Since 2026-10-15: the CONTACT event and its hush, no LAND (LAND COLLISION and its blind descent until
 * then); the Mac's LAND onto the palm 3 s later (after the controller's open window and the camera back): IDLE
 * TOUCHDOWN, never KILLED, the camera hold not blocked by the contact. That LAND is the blind one (user decision (B)
 * 2026-10-15: every LAND after a contact; it was the camera's until then): the ceiling alone, the touchdown by the
 * thrust condition on the palm. */
static void scenCollisionPalmFlight(void)
{
  int32_t k0;
  palmFlightToHover(&k0);
  autoStatusMsg_t m;
  int evLoop = -1;
  for (int i = 0; i < 4; i++) {
    tapG = 2.0f;
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (evLoop < 0 && m.evCode == AUTO_EV_CONTACT) {
      evLoop = i;
    }
  }
  tapG = 0.0f;
  autonomyGetStatus(&m);
  CHECK(evLoop == 3 && autonomyState() == AUTO_HOVER && (m.holdBits & AUTO_HB_HUSH) && !autonomyHoldBlocked() &&
        m.contacts == 1, "the tap: CONTACT on loop %d, the hush, HOVER, the hold not blocked", evLoop);
  /* the hush (0.5 s), the controller's open window (VZ_HUSH_OPEN_S 1.5 s), the camera back AUTO_CAM_BACK_MS: a LAND
   * before then also blocks the camera hold (collided); every LAND after the contact is the blind one */
  run(3000);
  autonomyGetStatus(&m);
  CHECK(autonomyState() == AUTO_HOVER && !(m.holdBits & AUTO_HB_HUSH) && (m.flags & AUTO_STF_CAM_HEIGHT),
        "3.0 s later: HOVER, the hush over, the camera height in use again (%s)", autoStateName((autoState_t)m.state));
  palm.on = true;
  palm.top = plant.h - 0.002f;
  palm.v = 0.0f;
  palm.lift = 0.0f;
  palm.k = 500.0f;
  palm.c = 5.0f;
  palm.contactMs = -1.0f;
  autoCmdResult_t r;
  CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK, "the Mac's LAND (%d)", r.result);
  float first, lowest;
  uint32_t offMs;
  bool blocked;
  palmLandWatch(&first, &lowest, &offMs, &blocked);
  autonomyRtcView_t rv;
  autonomyGetRtc(&rv);
  autonomyGetStatus(&m);
  const uint8_t vzWhy = rv.n > 0 ? rv.trans[rv.n - 1].vzReason : 0;
  const float off = offMs && palm.contactMs >= 0.0f ? ((float)offMs - palm.contactMs) * 0.001f : -1.0f;
  printf("info: a tap, then the Mac's LAND onto a palm: lowest LAND thrust %.0f, motors off %.2f s after the palm's "
         "contact, end %s (vz %s), the plant on the palm %d at %.3f m, the hold blocked at the LAND %d\n",
         (double)lowest, (double)off, autoReasonName((autoReason_t)m.reason), vzReasonName((vzReason_t)vzWhy),
         palm.held, (double)plant.h, blocked);
  CHECK(autonomyState() == AUTO_IDLE && m.reason == AUTO_RS_TOUCHDOWN && estopSets == 0 &&
            (vzWhy == VZ_R_TOUCHDOWN_SUPPORT || vzWhy == VZ_R_TOUCHDOWN_FORCE || vzWhy == VZ_R_TOUCHDOWN_JOLT),
        "IDLE TOUCHDOWN (vz %s), never KILLED", vzReasonName((vzReason_t)vzWhy));
  CHECK(palm.held && plant.h > 0.5f && !blocked, "the drone on the palm, the hold not blocked by the earlier contact");
  palmPutDown();
  run(3000);
  CHECK(lockDepth == 0, "locks balanced");
}

/* A Mac LAND onto a palm held still under the drone: the thrust stays at or above the LAND's floor, 0.65 x T_ref
 * (no probe since 2026-10-15; only a contact LAND had one before); the stall rule and FORCE end it on the palm */
static void scenMacLandPalmFlight(void)
{
  int32_t k0;
  palmFlightToHover(&k0);
  autoCmdResult_t r;
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  const uint16_t tHold = m.tHold;
  CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK, "Mac LAND (%d)", r.result);
  palm.on = true;
  palm.top = plant.h - 0.10f;
  palm.v = 0.0f;
  palm.k = 500.0f;
  palm.c = 5.0f;
  palm.contactMs = -1.0f;
  float first, lowest;
  uint32_t offMs;
  bool blocked;
  palmLandWatch(&first, &lowest, &offMs, &blocked);
  autonomyGetStatus(&m);
  printf("info: Mac LAND onto a palm: T_hold %u, thrust at the LAND %.0f, lowest %.0f (%.2f x T_hold), end %s, the "
         "plant on the palm %d\n", tHold, (double)first, (double)lowest, (double)(lowest / (float)tHold),
         autoReasonName((autoReason_t)m.reason), palm.held);
  CHECK(tHold > 0 && lowest >= 0.62f * (float)tHold && autonomyState() == AUTO_IDLE && m.reason == AUTO_RS_TOUCHDOWN &&
            palm.held, "a Mac LAND stays at its floor (lowest %.2f x T_hold, the LAND's floor 0.65 x T_ref), IDLE TOUCHDOWN "
        "on the palm", (double)(lowest / (float)(tHold ? tHold : 1)));
}

/* ---- the camera height lost in the FLIGHT hover (docs/autonomy-a2b.md 22) ---------------------------------- */

/* Steps until ev is among the status's three latest events (or maxMs); true when it came. (The controller's events
 * of a step are recorded after the state machine's own.) */
static bool untilEvent(autoEvent_t ev, uint32_t maxMs)
{
  autoStatusMsg_t m;
  for (uint32_t i = 0; i < maxMs; i++) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    if (m.evCode == ev || m.ev1Code == ev || m.ev2Code == ev) {
      return true;
    }
  }
  return false;
}

/* The glue's camera-lost path: the camera height out in the FLIGHT hover (frames lost, camVzGlueWhyBits NO_FRAMES):
 * CAM_LOST with its why bits and the open-loop hold, then back within the budget: CAM_BACK, still HOVER. Out again
 * for good: SINK at the budget's end (LAND SINK, the SINKING bit, the controller's LAND); back in the SINK: RESUME
 * (vzInput_t.resume through the glue: the controller back in HOLD) and HOVER. The Mac's LAND: IDLE TOUCHDOWN. */
static void scenCamLostFlight(void)
{
  int32_t k0;
  palmFlightToHover(&k0);
  run(3000);   /* the camera-closed hover's bound (capQ) measured */
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  CHECK(m.state == AUTO_HOVER && m.capQ > 0 && (m.flags & AUTO_STF_CAM_HEIGHT), "HOVER on the camera height, capQ %u",
        m.capQ);
  whyBitsStub = AUTO_CB_NO_FRAMES;
  camOutStub = true;
  const int calls0 = whyBitsCalls;
  CHECK(untilEvent(AUTO_EV_CAM_LOST, 1000), "CAM_LOST reported");
  autonomyGetStatus(&m);
  CHECK(m.state == AUTO_HOVER && m.camBits == AUTO_CB_NO_FRAMES && m.evDetail == AUTO_CB_NO_FRAMES &&
        (m.holdBits & (AUTO_HB_CAM_LOST | AUTO_HB_OPEN_LOOP)) == (AUTO_HB_CAM_LOST | AUTO_HB_OPEN_LOOP) &&
        whyBitsCalls > calls0 && m.sinkInMs > 4000, "CAM_LOST: HOVER, why NO_FRAMES (bits 0x%02x, detail 0x%02x), "
        "the open-loop hold (0x%02x), SINK in %u ms", m.camBits, m.evDetail, m.holdBits, m.sinkInMs);
  run(1500);
  camOutStub = false;
  CHECK(untilEvent(AUTO_EV_CAM_BACK, 2000), "CAM_BACK reported");
  autonomyGetStatus(&m);
  CHECK(m.state == AUTO_HOVER && !(m.holdBits & AUTO_HB_CAM_LOST), "back: HOVER, closed again (0x%02x)", m.holdBits);
  run(2000);
  const float h0 = plant.h;
  camOutStub = true;
  CHECK(untilEvent(AUTO_EV_SINK, 7000), "SINK at the budget's end");
  autonomyGetStatus(&m);
  CHECK(m.state == AUTO_LAND && m.reason == AUTO_RS_SINK && (m.holdBits & AUTO_HB_SINKING),
        "LAND SINK, the SINKING bit (%s %s 0x%02x), the height %.3f -> %.3f m over the 5 s open loop",
        autoStateName((autoState_t)m.state), autoReasonName((autoReason_t)m.reason), m.holdBits, (double)h0,
        (double)plant.h);
  CHECK(plant.h - h0 <= 0.10f, "the open-loop hold did not climb (%+.3f m)", (double)(plant.h - h0));
  run(400);
  camOutStub = false;
  whyBitsStub = 0;
  CHECK(untilEvent(AUTO_EV_RESUME, 3000), "RESUME when the camera height came back in the SINK");
  autonomyGetStatus(&m);
  autoRecFields_t rf;
  autonomyRecordFields(&rf);
  printf("info: camera lost in the FLIGHT hover: SINK then RESUME at %.3f m (vz %s), events %u\n", (double)plant.h,
         vzPhaseName((vzPhase_t)rf.vzPhase), m.evCount);
  CHECK(m.state == AUTO_HOVER && m.reason == AUTO_RS_RESUME && rf.vzPhase == VZ_HOLD && plant.h > 0.3f,
        "HOVER RESUME, the controller in HOLD, still up (%s, %.3f m)", vzPhaseName((vzPhase_t)rf.vzPhase),
        (double)plant.h);
  autoCmdResult_t r;
  CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK, "the Mac's LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  autonomyGetStatus(&m);
  CHECK(m.reason == AUTO_RS_TOUCHDOWN && estopSets == 0, "IDLE TOUCHDOWN, never KILLED (%s)",
        autoReasonName((autoReason_t)m.reason));
  CHECK(lockDepth == 0, "locks balanced");
}

/* ---- the hover wait over 65.5 s (user decision 2026-10-14, docs/autonomy-a2b.md 18.2) ------------------- */

/* NVS 'autowait' became u32 ms (0..120000); the drone's stored u16 item keeps its meaning through a read with
 * the legacy type, and nothing is written at boot. */
static void scenWaitLegacyOnly(void)
{
  nvsPut("autowait", AUTO_NVS_U16, 10000);   /* the user's 'auto set autowait 10000' on an older firmware */
  const int w0 = flashWrites;
  boot(ESP_RST_POWERON);
  CHECK(autonomySettingGet(AUTO_SET_AUTOWAIT) == 10000 && nvsCount("autowait") == 1 &&
        nvsGetT("autowait", AUTO_NVS_U16) == 10000 && nvsGetT("autowait", AUTO_NVS_U32) == -1 && flashWrites == w0,
        "only the u16 10000 stored: autowait %ld, nothing written at boot", (long)autonomySettingGet(AUTO_SET_AUTOWAIT));
  /* 'auto set autowait 10000' re-saves it as u32: both items erased first, then the u32 written */
  const char *why = "";
  CHECK(autonomySettingSet(AUTO_SET_AUTOWAIT, 10000, &why), "set autowait 10000 (%s)", why);
  CHECK(nvsCount("autowait") == 1 && nvsGetT("autowait", AUTO_NVS_U32) == 10000 &&
        nvsGetT("autowait", AUTO_NVS_U16) == -1 && autonomySettingGet(AUTO_SET_AUTOWAIT) == 10000,
        "after the set: one u32 item 10000, the u16 one gone");
  CHECK(!autonomySettingSet(AUTO_SET_AUTOWAIT, 120001, &why) && strcmp(why, "range") == 0 &&
        autonomySettingSet(AUTO_SET_AUTOWAIT, 120000, &why) && nvsGetT("autowait", AUTO_NVS_U32) == 120000,
        "120001 refused (range), 120000 stored as u32");
  /* a stale u16 item next to the u32 one (written by an older firmware after a downgrade): 'reset' erases both */
  nvsPut("autowait", AUTO_NVS_U16, 20000);
  CHECK(nvsCount("autowait") == 2, "both items present");
  CHECK(autonomySettingReset(AUTO_SET_AUTOWAIT, &why) && nvsCount("autowait") == 0 &&
        autonomySettingGet(AUTO_SET_AUTOWAIT) == AUTO_HOVER_WAIT_DEFAULT_MS, "reset: no item of either type left, "
        "the default %u", (unsigned)AUTO_HOVER_WAIT_DEFAULT_MS);
}

static void scenWaitU32Wins(void)
{
  nvsPut("autowait", AUTO_NVS_U16, 10000);
  nvsPut("autowait", AUTO_NVS_U32, 30000);
  boot(ESP_RST_POWERON);
  CHECK(autonomySettingGet(AUTO_SET_AUTOWAIT) == 30000, "both stored: the u32 item is used (%ld)",
        (long)autonomySettingGet(AUTO_SET_AUTOWAIT));
}

static void scenWaitNone(void)
{
  boot(ESP_RST_POWERON);
  CHECK(autonomySettingGet(AUTO_SET_AUTOWAIT) == AUTO_HOVER_WAIT_DEFAULT_MS && nvsCount("autowait") == 0,
        "none stored: the default 5000 (%ld)", (long)autonomySettingGet(AUTO_SET_AUTOWAIT));
}

/* NVS autowait 120000 in the status (a u16 cast would make it 54464): IDLE and the DRY sortie's wait show 65535
 * ("65.5 s or more"). Since 2026-10-15 the wait is information for the Mac (waitLeftMs 0) and no clock lands the
 * hover: it goes on past 120 s of air (AIR_TIME until then) until the Mac's LAND, here at 125 s of air. */
static void scenWaitNvs120(void)
{
  bootToSession(ESP_RST_POWERON);
  const char *why = "";
  CHECK(autonomySettingSet(AUTO_SET_AUTOWAIT, 120000, &why), "set autowait 120000 (%s)", why);
  run(600);   /* the next periodic AUTO_STATUS */
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  CHECK(m.hoverWaitMs == 65535, "IDLE status: hoverWaitMs saturated (%u)", m.hoverWaitMs);
  CHECK(toInteractiveWait(AUTO_MODE_DRY, 100, 0), "DRY waiting (the NVS wait)");
  autonomyGetStatus(&m);
  const uint32_t air0 = m.airMs;
  CHECK(m.waitLeftMs == 0 && m.hoverWaitMs == 65535 && m.airLeftDs == 0, "the wait's start: waitLeftMs %u (the wait "
        "is information), hoverWaitMs %u (65535: over 65.5 s; 54464 would be the u16 wrap), airLeftDs %u", m.waitLeftMs,
        m.hoverWaitMs, m.airLeftDs);
  bool left = false;
  for (int i = 0; i < 125000 && !left; i++) {
    linkBackground();
    loopOnce();
    autonomyGetStatus(&m);
    left = m.state != AUTO_HOVER || (uint32_t)m.airDs * 100u >= 125000u;
  }
  autonomyGetStatus(&m);
  printf("info: NVS autowait 120000 (DRY): the wait from air %u ms; at air %u ms: %s %s\n", (unsigned)air0,
         (unsigned)m.airDs * 100u, autoStateName((autoState_t)m.state), autoReasonName((autoReason_t)m.reason));
  CHECK(m.state == AUTO_HOVER && (uint32_t)m.airDs * 100u >= 125000u && m.airMs == 65535 && m.airLeftDs == 0,
        "125 s of air: still HOVER, no air clock (%s %s, airDs %u)", autoStateName((autoState_t)m.state),
        autoReasonName((autoReason_t)m.reason), m.airDs);
  autoCmdResult_t r;
  CHECK(command(AUTO_CMD_LAND, 0, 0, &r, 3) && r.result == AUTO_R_OK, "the Mac's LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
}

/* TAKEOFF 1.20 m with a 110 s wait (FLIGHT on the plant): the wait starts inside the pre-wait limit, the Mac lands
 * 110 s later (HOVER_TIMEOUT inside a 120 s air clock until 2026-10-15). */
static void scenMargin120Long(void)
{
  bootToSession(ESP_RST_POWERON);
  vbatStub = 4.1f;
  run(17000);
  vzSimDefaultConfig(&plantCfg);
  plantCfg.kTrue = (float)autonomySettingGet(AUTO_SET_VZK);
  plantCfg.vRest = 4.08f;
  plantCfg.camWakeS = 0.0f;
  vzSimInit(&plant, &plantCfg, 99u);
  plantV = plant.vbat;
  usePlant = true;
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_FLIGHT, 120, 110000), "waiting at 1.20 m, 110 s");
  autoStatusMsg_t m;
  autonomyGetStatus(&m);
  const uint32_t waitAir = m.airMs;
  CHECK(m.waitLeftMs == 0, "the wait's start: waitLeftMs 0 (the wait is information for the Mac; %u)", m.waitLeftMs);
  uint32_t waitAir2;
  const uint32_t macAt = hoverToMacLand(110000u, &waitAir2);
  autonomyGetStatus(&m);
  const uint32_t landAir = (uint32_t)m.airDs * 100u;
  const uint8_t why = m.reason;
  CHECK(runUntil(AUTO_IDLE, 25000), "landed");
  printf("info: 1.20 m, 110 s wait: the wait at air %u ms, LAND at air %u ms (reason %s), true peak %.3f m, "
         "touchdown %.2f m/s\n", (unsigned)waitAir, (unsigned)landAir, autoReasonName((autoReason_t)why),
         (double)plant.hMax, (double)plant.impactV);
  CHECK(macAt == 110000u && why == AUTO_RS_LAND_CMD && landAir + 250u >= waitAir + 110000u &&
        landAir <= waitAir + 110250u, "the Mac's LAND 110 s into the wait (at air %u)", (unsigned)landAir);
  CHECK(plant.impactV <= 0.6f, "touchdown %.2f m/s (<= 0.6)", (double)plant.impactV);
}

/* SET_HEIGHT through the glue (2026-10-15, docs/autonomy-a2b.md 23), FLIGHT on the plant at 1.00 m: the ACK with the
 * old estimate, the controller's estimate shifted on the accepting step (before its vzRun: the status that step
 * publishes has it), the correction flown (the true height moves by -d), the reference back. */
static void scenSetHeightFlight(void)
{
  bootToSession(ESP_RST_POWERON);
  yawFree = true;
  vbatStub = 4.1f;
  autoCmdResult_t r;
  autoCmd_t sh;
  memset(&sh, 0, sizeof(sh));
  sh.type = AUTO_CMD_SET_HEIGHT;
  sh.heightCm = 120;
  sh.moveUpCm = 100;
  CHECK(post(&sh, &r) && r.result == AUTO_R_BAD_STATE && ((r.detail >> 8) & 0xFF) == AUTO_MVW_NOT_WAITING &&
        (r.detail & 0xFF) == AUTO_IDLE && (r.detail >> 16) == 0, "IDLE: BAD_STATE NOT_WAITING, detail 0x%x (never 0: "
        "an older drone's unknown type)", (unsigned)r.detail);
  run(17000);
  vzSimDefaultConfig(&plantCfg);
  plantCfg.kTrue = (float)autonomySettingGet(AUTO_SET_VZK);
  plantCfg.vRest = 4.08f;
  plantCfg.camWakeS = 0.0f;
  vzSimInit(&plant, &plantCfg, 11u);
  plantV = plant.vbat;
  usePlant = true;
  run(200);
  CHECK(toInteractiveWait(AUTO_MODE_FLIGHT, 100, 60000), "FLIGHT waiting at 1.00 m");
  run(2000);
  autoStatusMsg_t m0, m;
  autonomyGetStatus(&m0);
  const float h0 = plant.h;
  CHECK(post(&sh, &r) && r.result == AUTO_R_OK && (r.detail & 0xFF) == AUTO_HOVER && ((r.detail >> 8) & 0xFF) == 0,
        "X 1.20 at 1.00: OK (0x%x)", (unsigned)r.detail);
  autonomyGetStatus(&m);
  const int oldCm = (int)(int16_t)(uint16_t)(r.detail >> 16);
  CHECK(abs(oldCm * 10 - m0.hMm) <= 10, "the ACK's old estimate %d cm (the status before: %d mm)", oldCm, m0.hMm);
  CHECK(m.hMm - m0.hMm >= 190 && m.hMm - m0.hMm <= 210 && m.lastCmdType == AUTO_CMD_SET_HEIGHT &&
        m.lastCmdResult == AUTO_R_OK && m.mvKind == AUTO_MVK_MOVE && (m.mvFlags & AUTO_MVF_SET_HEIGHT) &&
        m.tgtUpCm == -20 && m.mvSeq == m0.mvSeq + 1 && m.hTargetCm == 120 && m.mvState == AUTO_MV_RUN,
        "the accepting step's status: the estimate %d -> %d mm (shifted before its vzRun), lastCmdType 10, a MOVE "
        "with mvFlags 0x40, target up -20 cm, hTarget 120 cm (the reference at old + d)", m0.hMm, m.hMm);
  CHECK(post(&sh, &r) && r.result == AUTO_R_BAD_STATE && ((r.detail >> 8) & 0xFF) == AUTO_MVW_BUSY,
        "a second one while it runs: BUSY");
  CHECK(untilMotionDone(10000), "the correction ends");
  run(1500);
  autonomyGetStatus(&m);
  const float h1 = plant.h;
  printf("info: SET_HEIGHT +0.20 through the glue: true height %.3f -> %.3f m (%+.3f for -0.20), estimate %d mm, "
         "hTarget %d cm\n", (double)h0, (double)h1, (double)(h1 - h0), m.hMm, m.hTargetCm);
  CHECK(m.mvState == AUTO_MV_DONE && m.mvEnd == AUTO_MVE_DONE && m.hTargetCm == 100 && abs(m.hMm - m0.hMm) <= 40 &&
        h1 - h0 <= -0.15f && h1 - h0 >= -0.25f && autonomyState() == AUTO_HOVER,
        "DONE: the reference back at 100 cm, the estimate back near %d mm (%d), the drone flew %.3f m (-0.20)", m0.hMm,
        m.hMm, (double)(h1 - h0));
  CHECK(post((const autoCmd_t[]){simpleCmd(AUTO_CMD_LAND)}, &r) && r.result == AUTO_R_OK, "LAND");
  CHECK(runUntil(AUTO_IDLE, 15000), "landed");
  CHECK(plant.impactV <= 0.6f && overrideBad == 0, "touchdown %.2f m/s", (double)plant.impactV);
  autonomyProfile_t p;
  autonomyGetProfile(&p);
  CHECK(p.shiftRefused == 0, "no shift refused by the controller");
}

static const struct {
  const char *name;
  void (*fn)(void);
} scenarios[] = {
  {"idle", scenIdle},
  {"dry", scenDry},
  {"bench", scenBench},
  {"plant", scenPlant},
  {"locks", scenLocks},
  {"race", scenRace},
  {"handback", scenHandback},
  {"settings", scenSettings},
  {"flightsim", scenFlightSim},
  {"learn-console", scenLearnConsole},
  {"learn-poll", scenLearnPoll},
  {"resetlock", scenResetLock},
  {"usb-unlock", scenUsbUnlock},
  {"usb-unlock-poweron", scenUsbUnlockPowerOn},
  {"heap", scenHeap},
  {"ihold", scenIhold},
  {"margin-1", scenMargin1},
  {"margin-1-heavy", scenMargin1h},
  {"margin-2", scenMargin2},
  {"margin-2-heavy", scenMargin2h},
  {"margin-60", scenMargin60},
  {"margin-60-heavy", scenMargin60h},
  {"interactive-dry", scenInteractiveDry},
  {"mvspeed-200", scenMvSpeed200},
  {"mvspeed-100", scenMvSpeed100},
  {"guard-margins", scenGuardMargins},
  {"fast-caps", scenFastCaps},
  {"fast-caps-stored", scenFastCapsStored},
  {"interactive-flight", scenInteractiveFlight},
  {"plan-dry", scenPlanDry},
  {"batt-low", scenBattLow},
  {"margin-120", scenMargin120},
  {"margin-120-heavy", scenMargin120h},
  {"collision-dry", scenCollisionDry},
  {"palm-flight", scenPalmFlight},
  {"collision-palm-flight", scenCollisionPalmFlight},
  {"macland-palm-flight", scenMacLandPalmFlight},
  {"camlost-flight", scenCamLostFlight},
  {"wait-legacy-u16", scenWaitLegacyOnly},
  {"wait-u32-wins", scenWaitU32Wins},
  {"wait-none", scenWaitNone},
  {"wait-nvs-120000", scenWaitNvs120},
  {"margin-120-long", scenMargin120Long},
  {"set-height-flight", scenSetHeightFlight},
};

int main(int argc, char **argv)
{
  long totalChecks = 0, totalFailures = 0;
  int ran = 0;
  for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
    if (argc > 1 && strcmp(argv[1], scenarios[i].name) != 0) {
      continue;
    }
    ran++;
    int fd[2];
    if (pipe(fd) != 0) {
      perror("pipe");
      return 2;
    }
    fflush(stdout);
    const pid_t pid = fork();
    if (pid == 0) {
      close(fd[0]);
      scenarios[i].fn();
      printf("%s: %d checks, %d failures\n", scenarios[i].name, checks, failures);
      fflush(stdout);
      const int v[2] = {checks, failures};
      if (write(fd[1], v, sizeof(v)) != (ssize_t)sizeof(v)) {
        _exit(2);
      }
      _exit(failures ? 1 : 0);
    }
    close(fd[1]);
    int v[2] = {0, 0};
    const ssize_t n = read(fd[0], v, sizeof(v));
    close(fd[0]);
    int status = 0;
    waitpid(pid, &status, 0);
    if (n != (ssize_t)sizeof(v) || !WIFEXITED(status) || (WEXITSTATUS(status) != 0 && v[1] == 0)) {
      printf("FAIL: scenario %s ended abnormally (status 0x%x)\n", scenarios[i].name, (unsigned)status);
      v[1]++;
    }
    totalChecks += v[0];
    totalFailures += v[1];
  }
  if (ran == 0) {
    printf("unknown scenario '%s'\n", argc > 1 ? argv[1] : "");
    return 2;
  }
  printf("%ld checks, %ld failures\n", totalChecks, totalFailures);
  return totalFailures ? 1 : 0;
}
