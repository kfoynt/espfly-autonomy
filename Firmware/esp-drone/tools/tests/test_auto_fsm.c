/*
 * Host unit tests for components/core/crazyflie/utils/src/auto_fsm.c
 * (docs/autonomy-a2.md section 9.2): scripted scenarios at 1 kHz with a fake
 * vertical controller, a fake camera, the phone hook of section 8.2 and the
 * glue's thrust disposition of section 2 (setpoint, then the mode clamp).
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_auto_fsm.c components/core/crazyflie/utils/src/auto_fsm.c \
 *      components/core/crazyflie/utils/src/auto_motion.c -lm -o /tmp/test_auto_fsm && /tmp/test_auto_fsm
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_fsm.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define CAM_RUNNING  2   /* CAMERA_FLOW_RUNNING */
#define CAM_STANDBY  5   /* CAMERA_FLOW_STANDBY */
#define CAM_OFF      0
#define TOKEN        0xC0FFEE01u

/* ---- fake vertical controller ------------------------------------------
 * Follows vzCmd like vert_ctrl.h says (table 4.3) on a fixed script: SPIN
 * spinMs, RAMP rampMs (lift-off at its end), CLIMB climbMs, then HOLD, with
 * holdSteady after steadyAfterMs. LAND sets tdThrustOk after tdOkAfterMs and
 * ends DONE after doneAfterMs. Airborne, the camera height estimate is
 * healthy unless noCam. */
typedef struct {
  bool active;            /* stepped by the glue from vzReset until IDLE */
  vzOutput_t o;           /* output of the latest step (the FSM reads it on the next one) */
  uint32_t t;             /* ms in the current phase */
  uint32_t spinMs, rampMs, climbMs, steadyAfterMs, tdOkAfterMs, doneAfterMs;
  bool neverSteady, neverDone, noCam;
  float holdThrust;
  float ceiling;          /* HOLD's ceiling in force (vzOutput_t.ceiling): 0 = VZ_THRUST_ABS_MAX */
  bool tHoldBad, ceilNan; /* HOLD publishes tHoldBadV as T_hold (the thrust stays holdThrust) / a NaN ceiling: the
                             headroom rule's non-finite inputs (2026-10-16 review) */
  float tHoldBadV;
  bool noCapQ;            /* no camera-measured hover bound (vzOutput_t capQ 0); else after capQAfterMs of HOLD on the
                             camera (the real one: holdLoopS + a 1 s window) */
  uint32_t capQAfterMs, camHoldMs;
  vzReason_t climbEnd;    /* the HOLD entry's reason (HOLD_REACHED; CONTACT_HOLD, K_RANGE...: a climb fault) */
} fakeVz_t;

static void fzSet(fakeVz_t *z, vzPhase_t p, vzReason_t why)
{
  z->o.phase = p;
  z->o.reason = why;
  z->t = 0;
  if (p == VZ_HOLD) {
    z->o.noCamS = 0.0f;   /* as vert_ctrl.c: noCamS starts again at HOLD */
  }
}

static void fzReset(fakeVz_t *z)
{
  memset(&z->o, 0, sizeof(z->o));
  z->o.g0 = 1.0f;
  z->t = 0;
  z->camHoldMs = 0;
}

static void fzStep(fakeVz_t *z, vzCmd_t cmd, bool resume)
{
  vzOutput_t *o = &z->o;
  o->liftoffNow = false;
  z->t++;
  if (cmd == VZ_CMD_CUT && o->phase != VZ_CUT) {
    fzSet(z, VZ_CUT, VZ_R_CMD_CUT);
  }
  switch (o->phase) {
    case VZ_IDLE:
      if (cmd == VZ_CMD_TAKEOFF) {
        fzSet(z, VZ_SPIN, VZ_R_TAKEOFF);
      } else if (cmd == VZ_CMD_LAND) {
        o->landReason = VZ_R_CMD_LAND;
        fzSet(z, VZ_DONE, VZ_R_CMD_LAND);
      }
      break;
    case VZ_SPIN:
    case VZ_RAMP:
    case VZ_CLIMB:
    case VZ_HOLD:
      if (cmd != VZ_CMD_TAKEOFF) {
        o->landReason = cmd == VZ_CMD_LAND ? VZ_R_CMD_LAND : VZ_R_CMD_MISSING;
        o->tRef = o->holdReached ? o->tHold : o->tLo;
        fzSet(z, VZ_LAND, o->landReason);
      } else if (o->phase == VZ_SPIN && z->t >= z->spinMs) {
        fzSet(z, VZ_RAMP, VZ_R_SPIN_DONE);
      } else if (o->phase == VZ_RAMP && z->t >= z->rampMs) {
        o->liftedOff = true;
        o->liftoffNow = true;
        o->tLo = o->thrust;
        fzSet(z, VZ_CLIMB, VZ_R_LIFTOFF);
      } else if (o->phase == VZ_CLIMB && z->t >= z->climbMs) {
        o->holdReached = true;
        fzSet(z, VZ_HOLD, z->climbEnd != VZ_R_NONE ? z->climbEnd : VZ_R_HOLD_REACHED);
      }
      break;
    case VZ_LAND:
      /* a SINK resumed (vzInput_t.resume with VZ_CMD_TAKEOFF): HOLD after this step, as vert_ctrl.c */
      if (resume && cmd == VZ_CMD_TAKEOFF && o->camHealthy && o->landReason == VZ_R_CMD_LAND && o->holdReached) {
        o->evt = (uint8_t)VZ_R_RESUME;
        o->evtN++;
        fzSet(z, VZ_HOLD, VZ_R_RESUME);
        break;
      }
      if (!z->neverDone && z->t >= z->doneAfterMs) {
        o->touchdown = o->liftedOff;
        fzSet(z, VZ_DONE, o->liftedOff ? VZ_R_TOUCHDOWN_FORCE : VZ_R_ABORTED);
      }
      break;
    default:
      break;
  }
  switch (o->phase) {
    case VZ_SPIN:
      o->thrust = 18000.0f;
      break;
    case VZ_RAMP:
      o->thrust = 20000.0f + 12.0f * (float)z->t;
      break;
    case VZ_CLIMB:
      o->thrust = 38000.0f;
      o->v = 0.4f;
      o->h += 0.0004f;
      break;
    case VZ_HOLD:
      o->thrust = z->holdThrust;
      o->tHold = z->tHoldBad ? z->tHoldBadV : z->holdThrust;
      o->ceiling = z->ceilNan ? NAN : z->ceiling > 0.0f ? z->ceiling : VZ_THRUST_ABS_MAX;   /* the battery bound (the
                                                                                             climb's headroom) */
      o->v = 0.0f;
      break;
    case VZ_LAND: {
      const float th = 34000.0f - 8.0f * (float)z->t;
      o->thrust = th > 20000.0f ? th : 20000.0f;
      o->v = -0.3f;
      break;
    }
    default:
      o->thrust = 0.0f;
      o->v = 0.0f;
      break;
  }
  o->tPhase = 0.001f * (float)z->t;
  o->holdSteady = o->phase == VZ_HOLD && !z->neverSteady && z->t >= z->steadyAfterMs;
  o->tdThrustOk = o->phase == VZ_LAND && z->t >= z->tdOkAfterMs;
  o->camHealthy = !z->noCam && o->liftedOff && (o->phase == VZ_CLIMB || o->phase == VZ_HOLD || o->phase == VZ_LAND);
  /* s since the later of the HOLD entry and the last step with the camera in use, airborne (vert_ctrl.c noCamT) */
  if (o->liftedOff && (o->phase == VZ_CLIMB || o->phase == VZ_HOLD || o->phase == VZ_LAND)) {
    o->noCamS = o->camHealthy ? 0.0f : o->noCamS + 0.001f;
  }
  z->camHoldMs = (o->phase == VZ_HOLD && o->camHealthy) ? z->camHoldMs + 1u : (o->phase == VZ_HOLD ? z->camHoldMs : 0u);
  if (!z->noCapQ && o->capQ <= 0.0f && z->camHoldMs >= z->capQAfterMs) {
    o->capQ = 0.995f * z->holdThrust;
  }
  if (o->liftedOff && o->phase != VZ_DONE && o->phase != VZ_CUT) {
    o->tAir += 0.001f;
  }
}

/* ---- test rig: FSM + glue + Mac + phone + camera, one step = 1 ms ------- */
typedef struct {
  autoFsm_t f;
  autoFsmIn_t in;
  autoFsmOut_t out;
  fakeVz_t vz;
  uint32_t steps;
  bool hbStream;          /* Mac heartbeat every 100 ms */
  bool appStream;         /* phone: zero-thrust, centred packets every 50 ms */
  bool pushed;            /* phone hook gesture state */
  float appThrust;        /* what the app setpoint carries (passes through with action NONE) */
  bool camAuto;           /* the photo set completes after n x gap */
  bool camViewCounting;   /* camViewMs grows by 1 per step */
  uint32_t capEndMs;
  bool estop;             /* the stabilizer's emergency stop flag */
  bool vzIdle;            /* the FSM reads the controller's output with phase IDLE (STABLE hard bound) */
  bool camFrozen;         /* interactive hover: the camera stops applying the hold's reference (progress 0) */
  bool holdBlock;         /* the glue's autonomyHoldBlocked() after this step (autonomy.c stepFull) */
  /* statistics */
  float applied, maxApplied;   /* thrust that reaches the motors after both clamps */
  bool sawCut;
  int captureStarts, vzResets, yawCaptures, lockApps, setEstops, clearEstops;
  int blindSteps;         /* steps the controller was given vzInput_t.blind (out.blind) */
  int hushSteps;          /* steps it was given vzInput_t.hush */
  int shiftSteps;         /* steps the glue shifted the controller's estimate (out.hShift, SET_HEIGHT) */
} rig_t;

static void imuNormal(autoFsmIn_t *in)
{
  in->rollDeg = 0.4f;
  in->pitchDeg = -0.3f;
  in->tiltDeg = 0.5f;
  in->attErrDeg = 0.2f;
  in->gyroDps[0] = 0.3f;
  in->gyroDps[1] = -0.2f;
  in->gyroDps[2] = 0.1f;
  in->accNormG = 1.002f;
}

static void rigInit(rig_t *r, uint32_t t0)
{
  memset(r, 0, sizeof(*r));
  autoFsmInit(&r->f);
  autoFsmIn_t *in = &r->in;
  in->nowMs = t0;
  in->sessionValid = true;
  in->hbAgeMs = 20;
  in->randToken = TOKEN;
  in->appAgeMs = 20;
  in->appInputAgeMs = 5000;
  in->appGestureAgeMs = 3000;
  in->appThrustZero = true;
  in->autoEnabled = true;
  in->appRequired = true;
  in->gestureRequired = true;
  in->vmin = AUTO_VMIN_DEFAULT;
  in->airMaxNvsMs = AUTO_AIR_MAX_DEFAULT_MS;
  in->hoverWaitNvsMs = AUTO_HOVER_WAIT_DEFAULT_MS;
  in->hoverAccelNvsMs = AUTO_HOVER_ACCEL_DEFAULT_MS;
  in->vzParamsOk = true;
  imuNormal(in);
  in->loopGapUs = 1000;
  in->vbat = 4.05f;
  in->vbatFastMin = 3.98f;
  in->camState = CAM_RUNNING;
  in->camFps = 16.9f;
  in->camFrameAgeMs = 30;
  in->camViewMs = 20000;
  in->holdEnabled = true;
  in->holdApplied = true;
  in->onGround = true;
  in->onGroundMs = 20000;
  in->motorsReady = true;
  in->motorGuardOn = true;
  in->heapIntFree = 15700;
  r->hbStream = true;
  r->appStream = true;
  r->camAuto = true;
  r->vz.spinMs = 800;
  r->vz.rampMs = 500;
  r->vz.climbMs = 700;
  r->vz.steadyAfterMs = 300;
  r->vz.tdOkAfterMs = 1200;
  r->vz.doneAfterMs = 1800;
  r->vz.holdThrust = 36000.0f;
  r->vz.capQAfterMs = 1500;
}

static uint32_t sat1(uint32_t v)
{
  return v == UINT32_MAX ? v : v + 1;
}

/* The phone hook of section 8.2, as the glue will run it per app packet. */
static void appPacket(rig_t *r, float roll, float pitch, float yawDps, float thrust)
{
  autoFsmIn_t *in = &r->in;
  (void)yawDps;   /* yaw is never input */
  in->appAgeMs = 0;
  if (thrust >= 1000.0f || fabsf(roll) >= 2.0f || fabsf(pitch) >= 2.0f) {
    in->appInputCount++;
    in->appInputAgeMs = 0;
  }
  if (fabsf(roll) >= 10.0f) {
    r->pushed = true;
  } else if (fabsf(roll) < 2.0f && r->pushed) {
    r->pushed = false;
    in->appGestureAgeMs = 0;
  }
  in->appThrustZero = thrust < 1000.0f;
  r->appThrust = thrust >= 1000.0f ? thrust : 0.0f;
}

static void rigStep(rig_t *r)
{
  autoFsmIn_t *in = &r->in;
  in->vz = r->vz.o;
  if (r->vzIdle) {
    in->vz.phase = VZ_IDLE;
  }
  in->emergencyStop = r->estop;
  autoFsmStep(&r->f, in, &r->out);
  const autoFsmOut_t *o = &r->out;

  /* glue: one-step actions */
  if (o->setEmergencyStop) {
    r->estop = true;
    r->setEstops++;
  }
  if (o->clearEmergencyStop) {
    r->estop = false;
    r->clearEstops++;
  }
  r->lockApps += o->lockAppThrust;
  r->yawCaptures += o->yawCapture;
  if (o->vzReset) {
    r->vzResets++;
    fzReset(&r->vz);
    r->vz.active = true;
    in->capture = AUTO_CAP_NONE;
  }
  if (o->captureStart) {
    r->captureStarts++;
    in->capture = AUTO_CAP_BUSY;
    r->capEndMs = in->nowMs + (uint32_t)o->captureN * o->captureGapMs;
  } else if (in->capture == AUTO_CAP_BUSY && r->camAuto && (int32_t)(in->nowMs - r->capEndMs) >= 0) {
    in->capture = AUTO_CAP_DONE;
  }
  if (o->state == AUTO_IDLE) {
    r->vz.active = false;
  }
  if (r->vz.active) {
    if (o->hShift != 0.0f && r->vz.o.phase == VZ_HOLD) {
      r->vz.o.h += o->hShift;   /* autonomy.c applyHeightShift(): vzShiftHeight() before this step's vzStep */
      r->shiftSteps++;
    }
    fzStep(&r->vz, o->vzCmd, o->resume);
    if (r->vz.o.phase == VZ_HOLD) {
      r->vz.o.h += o->mv.vRef * 0.001f;   /* the interactive hover's vertical reference, tracked */
    }
    r->blindSteps += o->blind;            /* vzRun: vi->blind = out->blind (out->collided until 2026-10-15) */
    r->hushSteps += o->hush;              /* vzRun: vi->hush = out->hush */
  }
  /* glue: the camera hold block, after the controller step (autonomy.c stepFull) */
  r->holdBlock = o->state == AUTO_LAND && (r->holdBlock || o->collided ||
                                           (r->vz.active && (r->vz.o.contact || r->vz.o.touchdown)) ||
                                           in->accNormG > AUTO_TD_CONTACT_G);
  /* the camera: applies the hold's reference epoch at once and tracks it (ideal) */
  if (!r->camFrozen) {
    in->holdEpochSeen = o->mv.epoch;
    in->holdProgLat = o->mv.refLat;
    in->holdProgFwd = o->mv.refFwd;
  } else if (in->holdEpochSeen != o->mv.epoch) {
    in->holdEpochSeen = o->mv.epoch;
    in->holdProgLat = in->holdProgFwd = 0.0f;
  }

  /* glue: thrust disposition (section 2.5) and the last-stage clamp */
  float thr = r->appThrust;
  switch (o->thrust) {
    case AUTO_THR_NONE:
      break;
    case AUTO_THR_ZERO:
    case AUTO_THR_CUT:
      thr = 0.0f;
      break;
    case AUTO_THR_VERT_CTRL:
    case AUTO_THR_VERT_LAND:
      thr = r->vz.o.thrust;
      break;
  }
  if (thr > o->thrustMax) {
    thr = o->thrustMax;
  }
  r->applied = thr;
  if (thr > r->maxApplied) {
    r->maxApplied = thr;
  }
  if (o->thrust == AUTO_THR_CUT) {
    r->sawCut = true;
  }

  /* next 1 ms */
  memset(&in->cmd, 0, sizeof(in->cmd));
  in->sessionChanged = false;
  in->nowMs++;
  r->steps++;
  in->hbAgeMs = sat1(in->hbAgeMs);
  in->appAgeMs = sat1(in->appAgeMs);
  in->appInputAgeMs = sat1(in->appInputAgeMs);
  in->appGestureAgeMs = sat1(in->appGestureAgeMs);
  in->onGroundMs = in->onGround ? sat1(in->onGroundMs) : 0;
  in->camViewMs = r->camViewCounting ? sat1(in->camViewMs) : in->camViewMs;
  if (r->hbStream && r->steps % 100 == 0) {
    in->hbAgeMs = 0;
  }
  if (r->appStream && r->steps % 50 == 0) {
    appPacket(r, 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

static void run(rig_t *r, uint32_t ms)
{
  for (uint32_t i = 0; i < ms; i++) {
    rigStep(r);
  }
}

/* Steps until the FSM reports state s; returns the steps taken (the last is
 * the transition step) or -1 after maxMs. */
static long untilState(rig_t *r, autoState_t s, uint32_t maxMs)
{
  for (uint32_t i = 1; i <= maxMs; i++) {
    rigStep(r);
    if (r->out.state == s) {
      return (long)i;
    }
  }
  return -1;
}

static autoCmd_t cmdOf(autoCmdType_t t)
{
  autoCmd_t c;
  memset(&c, 0, sizeof(c));
  c.type = t;
  return c;
}

static autoCmd_t cmdArm(autoMode_t mode, uint16_t airMaxMs)
{
  autoCmd_t c = cmdOf(AUTO_CMD_ARM);
  c.mode = (uint8_t)mode;
  c.airMaxMs = airMaxMs;
  return c;
}

static autoCmd_t cmdTakeoff(uint32_t token, uint16_t heightCm, uint8_t photoN, uint16_t gapMs)
{
  autoCmd_t c = cmdOf(AUTO_CMD_TAKEOFF);
  c.token = token;
  c.heightCm = heightCm;
  c.photoN = photoN;
  c.photoGapMs = gapMs;
  return c;
}

static autoFsmOut_t doCmd(rig_t *r, autoCmd_t c)
{
  r->in.cmd = c;
  rigStep(r);
  return r->out;
}

static void prepMode(rig_t *r, autoMode_t mode)
{
  r->in.simPlant = mode != AUTO_MODE_FLIGHT;
  r->in.gcap = mode == AUTO_MODE_BENCH ? 15000 : 0;
}

static uint16_t rigTakeoffCm = 30;   /* launch()'s TAKEOFF height (the SET_HEIGHT replays take off to 1.00 m) */

static bool launch(rig_t *r, autoMode_t mode, uint8_t photoN, uint16_t airMaxMs)
{
  prepMode(r, mode);
  if (airMaxMs > r->in.airMaxNvsMs) {
    r->in.airMaxNvsMs = airMaxMs;   /* ARM may not exceed NVS autoair: the user set it */
  }
  const autoFsmOut_t a = doCmd(r, cmdArm(mode, airMaxMs));
  if (a.cmdResult != AUTO_R_OK || a.state != AUTO_ARMED) {
    printf("  launch: ARM refused (result %d, detail 0x%x)\n", (int)a.cmdResult, (unsigned)a.cmdDetail);
    return false;
  }
  run(r, 50);
  const autoFsmOut_t t = doCmd(r, cmdTakeoff(a.cmdDetail, rigTakeoffCm, photoN, 150));
  if (t.cmdResult != AUTO_R_OK || t.state != AUTO_TAKEOFF) {
    printf("  launch: TAKEOFF refused (result %d, detail 0x%x)\n", (int)t.cmdResult, (unsigned)t.cmdDetail);
    return false;
  }
  return true;
}

static bool toHover(rig_t *r, autoMode_t mode, uint8_t photoN, uint16_t airMaxMs)
{
  return launch(r, mode, photoN, airMaxMs) && untilState(r, AUTO_HOVER, 6000) > 0;
}

static bool toWait(rig_t *r, autoMode_t mode);

/* Runs to the end of the sortie (IDLE), with at most maxMs steps. */
static bool toIdle(rig_t *r, uint32_t maxMs)
{
  return untilState(r, AUTO_IDLE, maxMs) > 0;
}

/* Steps until the controller's landing has ended (LAND with thrust ZERO). */
static bool untilLandEnded(rig_t *r, uint32_t maxMs)
{
  for (uint32_t i = 0; i < maxMs; i++) {
    rigStep(r);
    if (r->out.state == AUTO_LAND && r->out.thrust == AUTO_THR_ZERO) {
      return true;
    }
  }
  return false;
}

static bool untilStable(rig_t *r, uint32_t maxMs)
{
  for (uint32_t i = 0; i < maxMs; i++) {
    rigStep(r);
    if (r->out.stable) {
      return true;
    }
  }
  return false;
}

static bool untilCapture(rig_t *r, uint32_t maxMs)
{
  for (uint32_t i = 0; i < maxMs; i++) {
    rigStep(r);
    if (r->out.captureStart) {
      return true;
    }
  }
  return false;
}

/* ---- 1. full sortie ------------------------------------------------------ */

static void testFullSortie(uint32_t t0, uint32_t waitMs)
{
  rig_t r;
  rigInit(&r, t0);
  r.in.hoverWaitNvsMs = waitMs;
  printf("full sortie from t0 = %lu, hover wait %u ms\n", (unsigned long)t0, (unsigned)waitMs);

  autoFsmOut_t o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(o.cmdDone && o.cmdResult == AUTO_R_OK && o.cmdDetail == TOKEN, "ARM accepted with armToken (res %d)",
        (int)o.cmdResult);
  CHECK(o.state == AUTO_ARMED && o.reason == AUTO_RS_ARM && o.failedGuards == 0, "ARMED, no failed guard");
  CHECK(o.thrust == AUTO_THR_ZERO && o.thrustMax == VZ_THRUST_ABS_MAX && o.vzCmd == VZ_CMD_NONE,
        "ARMED holds thrust 0");
  run(&r, 200);
  CHECK(r.out.state == AUTO_ARMED, "ARMED waits");

  o = doCmd(&r, cmdTakeoff(TOKEN, 30, 2, 150));
  CHECK(o.cmdResult == AUTO_R_OK && o.state == AUTO_TAKEOFF && o.reason == AUTO_RS_TAKEOFF, "TAKEOFF accepted");
  CHECK(o.yawCapture && o.prespinLed && o.thrust == AUTO_THR_ZERO && !o.vzReset, "yaw captured, pre-spin starts");
  bool prespinOk = true;
  for (int i = 1; i < AUTO_PRESPIN_MS; i++) {
    rigStep(&r);
    prespinOk &= r.out.state == AUTO_TAKEOFF && r.out.thrust == AUTO_THR_ZERO && r.out.prespinLed &&
                 !r.out.vzReset && r.out.vzCmd == VZ_CMD_NONE && r.applied == 0.0f;
  }
  CHECK(prespinOk, "1 s pre-spin: motors off, LED on");
  rigStep(&r);
  CHECK(r.out.vzReset && r.out.thrust == AUTO_THR_VERT_CTRL && r.out.vzCmd == VZ_CMD_TAKEOFF && !r.out.prespinLed,
        "vzReset exactly at 1000 ms (stateMs %u)", (unsigned)r.out.stateMs);
  CHECK(fabsf(r.out.hTargetM - 0.30f) < 1e-6f, "hTargetM 0.30 (%.3f)", r.out.hTargetM);

  const long toHov = untilState(&r, AUTO_HOVER, 5000);
  CHECK(toHov > 0 && r.out.reason == AUTO_RS_HOLD, "HOVER when the controller holds (%ld ms after vzReset)", toHov);
  const uint32_t airAtHold = r.out.airMs;
  long toCap = -1;
  for (long i = 1; i <= 3000; i++) {
    rigStep(&r);
    if (r.out.captureStart) {
      toCap = i;
      break;
    }
  }
  CHECK(toCap > 0 && r.out.stableMs == AUTO_STABLE_MS && r.out.captureN == 2 && r.out.captureGapMs == 150,
        "captureStart (2, 150) after exactly 1 s STABLE (stableMs %u, %ld ms after HOLD)", (unsigned)r.out.stableMs,
        toCap);
  const uint32_t airAtCap = r.out.airMs;
  /* "Claude decides" (2026-10-15): the wait always starts (photoN done; no SORTIE_DONE without a wait) and is
   * information only: no HOVER_TIMEOUT, no air clock; the sortie ends by Claude's LAND */
  long toWait = -1;
  for (long i = 1; i <= 2000; i++) {
    rigStep(&r);
    if (r.out.waiting) {
      toWait = i;
      break;
    }
  }
  CHECK(toWait == 301 && r.out.state == AUTO_HOVER && r.out.waitLeftMs == 0 && r.out.thrust == AUTO_THR_VERT_CTRL &&
        r.out.vzCmd == VZ_CMD_TAKEOFF, "photos done (%ld ms after capture): HOVER waits for the Mac (no countdown)",
        toWait);
  const long early = untilState(&r, AUTO_LAND, waitMs + 3000u);
  CHECK(early < 0 && r.out.state == AUTO_HOVER && r.out.waiting && r.out.waitLeftMs == 0,
        "%u ms of wait + 3 s: still HOVER (no HOVER_TIMEOUT, no SORTIE_DONE)", (unsigned)waitMs);
  const autoReason_t endWhy = AUTO_RS_LAND_CMD;
  autoFsmOut_t lo = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(lo.state == AUTO_LAND && lo.reason == endWhy && !lo.waiting, "Claude's LAND: LAND LAND_CMD");
  CHECK(r.out.thrust == AUTO_THR_VERT_LAND && r.out.vzCmd == VZ_CMD_LAND, "LAND runs the controller's landing");
  const uint32_t airAtLand = r.out.airMs;
  long toEnd = -1;
  for (long i = 1; i <= 7000; i++) {
    rigStep(&r);
    if (r.out.thrust == AUTO_THR_ZERO) {
      toEnd = i;
      break;
    }
  }
  CHECK(toEnd > 0 && r.out.state == AUTO_LAND && r.out.reason == endWhy,
        "touchdown: thrust 0, still LAND with the trigger as reason (%ld ms)", toEnd);
  const long toRest = untilState(&r, AUTO_IDLE, 4000);
  CHECK(toRest == AUTO_LAND_REST_MS, "IDLE 1 s after touchdown at rest (%ld)", toRest);
  CHECK(r.out.reason == AUTO_RS_TOUCHDOWN && r.out.vzLandReason == VZ_R_TOUCHDOWN_FORCE && r.out.lockAppThrust,
        "IDLE reason TOUCHDOWN (%s), lockAppThrust", autoReasonName(r.out.reason));
  int zeros = 1;
  while (zeros < 2000) {
    rigStep(&r);
    if (r.out.thrust != AUTO_THR_ZERO) {
      break;
    }
    zeros++;
  }
  CHECK(zeros == AUTO_RELEASE_HOLD_MS, "600 ms of ZERO after the sortie (%d)", zeros);
  CHECK(r.out.thrust == AUTO_THR_NONE && r.out.thrustMax == 65535.0f, "then the phone flies again");
  CHECK(r.captureStarts == 1 && r.vzResets == 1 && r.yawCaptures == 1 && r.lockApps == 1, "one-shots once each");
  CHECK(!r.sawCut && r.setEstops == 0, "no cut anywhere");
  CHECK(r.maxApplied <= 54000.0f, "thrust <= 54000 (%.0f)", r.maxApplied);
  CHECK(airAtHold == 1200, "the air clock runs from the first RAMP step (airMs %u at HOVER)", (unsigned)airAtHold);
  printf("  vzReset->HOLD %ld ms, airMs at HOLD %u, capture %ld ms after HOLD (airMs %u), LAND at airMs %u, "
         "touchdown %ld ms later, IDLE +%ld ms\n",
         toHov, (unsigned)airAtHold, toCap, (unsigned)airAtCap, (unsigned)airAtLand, toEnd, toRest);
}

/* ---- 2. phone app -------------------------------------------------------- */

static void testAppStream(void)
{
  rig_t r;
  rigInit(&r, 1000);
  const uint32_t before = r.in.appInputCount;
  bool quiet = true;
  for (int i = 0; i < 60000; i++) {
    rigStep(&r);
    quiet &= r.out.state == AUTO_IDLE && r.out.thrust == AUTO_THR_NONE && !r.out.lockAppThrust;
  }
  CHECK(quiet && r.in.appInputCount == before, "60 s of zero-thrust centred packets in IDLE: nothing");

  /* The same stream through a whole sortie (airMax 5000) changes nothing. */
  r.in.appGestureAgeMs = 3000;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "sortie with the stream on");
  run(&r, 5000);
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting, "it hovers in the wait (no input)");
  r.in.cmd = cmdOf(AUTO_CMD_LAND);
  CHECK(toIdle(&r, 15000) && r.out.reason == AUTO_RS_TOUCHDOWN, "it ends normally (Claude's LAND)");

  /* Airborne: each kind of input lands, never cuts. */
  const struct { float roll, pitch, yaw, thrust; bool input; const char *what; } pk[] = {
    { 0.0f, 0.0f, 0.0f, 1000.0f, true, "thrust 1000" },
    { 2.1f, 0.0f, 0.0f, 0.0f, true, "roll 2.1" },
    { 0.0f, -2.1f, 0.0f, 0.0f, true, "pitch -2.1" },
    { 0.0f, 0.0f, 100.0f, 0.0f, false, "yaw 100 dps" },
    { 1.9f, -1.9f, 0.0f, 999.0f, false, "roll 1.9, pitch -1.9, thrust 999" },
  };
  for (size_t k = 0; k < sizeof(pk) / sizeof(pk[0]); k++) {
    rigInit(&r, 5000);
    r.vz.neverSteady = true;
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "%s: hover", pk[k].what);
    run(&r, 100);
    appPacket(&r, pk[k].roll, pk[k].pitch, pk[k].yaw, pk[k].thrust);
    rigStep(&r);
    if (pk[k].input) {
      CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_APP_INPUT && r.out.thrust == AUTO_THR_VERT_LAND,
            "%s airborne: LAND APP_INPUT on the next step (%s)", pk[k].what, autoStateName(r.out.state));
      appPacket(&r, 0.0f, 0.0f, 0.0f, 0.0f);
      CHECK(toIdle(&r, 6000) && r.out.reason == AUTO_RS_TOUCHDOWN, "%s: lands, motors off on touchdown", pk[k].what);
      CHECK(!r.sawCut && r.setEstops == 0, "%s: the phone never cuts", pk[k].what);
    } else {
      CHECK(r.out.state == AUTO_HOVER, "%s: ignored (%s)", pk[k].what, autoStateName(r.out.state));
    }
  }

  /* ARMED and pre-spin: back to IDLE. */
  rigInit(&r, 5000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
  run(&r, 30);
  appPacket(&r, 2.1f, 0.0f, 0.0f, 0.0f);
  rigStep(&r);
  CHECK(r.out.state == AUTO_IDLE && r.out.reason == AUTO_RS_APP_INPUT && r.out.lockAppThrust &&
        r.out.thrust == AUTO_THR_ZERO, "ARMED + roll 2.1: IDLE APP_INPUT, thrust locked, release hold");
  rigInit(&r, 5000);
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  run(&r, 500);
  r.appStream = false;   /* the thumb stays on the throttle */
  appPacket(&r, 0.0f, 0.0f, 0.0f, 1000.0f);
  rigStep(&r);
  CHECK(r.out.state == AUTO_IDLE && r.out.reason == AUTO_RS_APP_INPUT && r.vzResets == 0,
        "pre-spin + thrust 1000: IDLE before any motor turns");
  CHECK(r.applied == 0.0f, "the stale app thrust does not reach the motors in the release hold (%.0f)", r.applied);
  run(&r, AUTO_RELEASE_HOLD_MS - 1);
  CHECK(r.out.thrust == AUTO_THR_ZERO && r.applied == 0.0f && r.out.stateMs == 599, "release hold still on at 599 ms");
  rigStep(&r);
  CHECK(r.out.thrust == AUTO_THR_NONE && r.applied == 1000.0f, "at 600 ms the setpoint is the app's again (%.0f)",
        r.applied);

  /* LAND and KILLED ignore app input. */
  rigInit(&r, 5000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  run(&r, 50);
  appPacket(&r, 5.0f, 0.0f, 0.0f, 20000.0f);
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_LAND_CMD && r.out.thrust == AUTO_THR_VERT_LAND,
        "app input in LAND changes nothing");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_KILL)).state == AUTO_KILLED, "killed");
  appPacket(&r, 5.0f, 0.0f, 0.0f, 20000.0f);
  rigStep(&r);
  CHECK(r.out.state == AUTO_KILLED && r.out.thrust == AUTO_THR_CUT && r.applied == 0.0f,
        "app input in KILLED changes nothing");
}

/* ---- 3. app silent ------------------------------------------------------- */

static void testAppSilent(void)
{
  rig_t r;
  rigInit(&r, 7000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  r.appStream = false;
  bool held = true;
  while (r.in.appAgeMs <= AUTO_APP_SILENT_MS) {
    rigStep(&r);
    held &= r.out.state == AUTO_HOVER;
  }
  CHECK(held, "app silent up to 500 ms: nothing");
  CHECK(r.in.appAgeMs == AUTO_APP_SILENT_MS + 1, "next step sees 501 ms");
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_APP_SILENT, "silent 501 ms: LAND APP_SILENT");

  rigInit(&r, 7000);
  r.vz.neverSteady = true;
  r.in.appRequired = false;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover, app not required");
  r.appStream = false;
  run(&r, 1000);
  CHECK(r.out.state == AUTO_HOVER, "app not required: silence is fine");

  rigInit(&r, 7000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
  r.appStream = false;
  CHECK(untilState(&r, AUTO_IDLE, 700) > 0 && r.out.reason == AUTO_RS_APP_SILENT &&
        r.in.appAgeMs == AUTO_APP_SILENT_MS + 2, "ARMED: silent 501 ms -> IDLE APP_SILENT");
}

/* ---- 4. heartbeat -------------------------------------------------------- */

static void testHeartbeat(void)
{
  rig_t r;
  rigInit(&r, 9000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  r.hbStream = false;
  bool held = true;
  long n = 0;
  while (r.in.hbAgeMs <= AUTO_HB_LAND_MS) {
    rigStep(&r);
    n++;
    held &= r.out.state == AUTO_HOVER;
  }
  CHECK(held, "heartbeat 300..1000 ms old airborne: nothing (%ld steps)", n);
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_HB_LOST, "heartbeat 1001 ms: LAND HB_LOST");
  CHECK(r.in.hbAgeMs < 1100, "so a 1.1 s gap lands");
  CHECK(toIdle(&r, 6000) && r.out.reason == AUTO_RS_TOUCHDOWN && !r.sawCut,
        "with no heartbeat at all the landing completes, no cut");

  rigInit(&r, 9000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
  r.hbStream = false;
  CHECK(untilState(&r, AUTO_IDLE, 1200) > 0 && r.out.reason == AUTO_RS_HB_LOST && r.in.hbAgeMs == 1002,
        "ARMED: heartbeat 1001 ms -> IDLE HB_LOST");

  rigInit(&r, 9000);
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  r.in.hbAgeMs = 1001;
  rigStep(&r);
  CHECK(r.out.state == AUTO_IDLE && r.out.reason == AUTO_RS_HB_LOST, "pre-spin: heartbeat lost -> IDLE");
}

/* ---- 5. trips ------------------------------------------------------------ */

typedef enum { TRIP_TILT, TRIP_RATE_X, TRIP_RATE_Y, TRIP_FALL, TRIP_ATTERR } tripKind_t;

static void setTrip(autoFsmIn_t *in, tripKind_t k, bool on)
{
  imuNormal(in);
  if (!on) {
    return;
  }
  switch (k) {
    case TRIP_TILT: in->tiltDeg = 31.0f; break;
    case TRIP_RATE_X: in->gyroDps[0] = 501.0f; break;
    case TRIP_RATE_Y: in->gyroDps[1] = -501.0f; break;
    case TRIP_FALL: in->accNormG = 0.39f; break;
    case TRIP_ATTERR: in->attErrDeg = 10.5f; break;
  }
}

static void testTrips(void)
{
  /* The rate trip in BENCH: in FLIGHT and DRY HOVER a gx/gy above 250 dps
   * is an in-air contact (LAND COLLISION after 3 ms, testCollision) long
   * before the 30 ms of the rate trip, which then cuts in that LAND. */
  const struct { tripKind_t k; int ms; autoReason_t why; const char *what; autoMode_t mode; } t[] = {
    { TRIP_TILT, AUTO_TILT_KILL_MS, AUTO_RS_TILT, "tilt 31 deg", AUTO_MODE_FLIGHT },
    { TRIP_RATE_X, AUTO_RATE_KILL_MS, AUTO_RS_RATE, "gx 501 dps (BENCH)", AUTO_MODE_BENCH },
    { TRIP_RATE_Y, AUTO_RATE_KILL_MS, AUTO_RS_RATE, "gy -501 dps (BENCH)", AUTO_MODE_BENCH },
    { TRIP_FALL, AUTO_FALL_KILL_MS, AUTO_RS_FREEFALL, "|acc| 0.39 g", AUTO_MODE_FLIGHT },
    { TRIP_ATTERR, AUTO_ATTERR_KILL_MS, AUTO_RS_ATT_ERR, "attErr 10.5 deg", AUTO_MODE_FLIGHT },
  };
  for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
    rig_t r;
    rigInit(&r, 11000);
    r.vz.neverSteady = true;
    CHECK(toHover(&r, t[i].mode, 2, 5000), "%s: hover", t[i].what);
    /* ms - 1 steps of the condition hold it for ms - 2 ms: ignored */
    for (int s = 0; s < t[i].ms - 1; s++) {
      setTrip(&r.in, t[i].k, true);
      rigStep(&r);
    }
    setTrip(&r.in, t[i].k, false);
    rigStep(&r);
    CHECK(r.out.state == AUTO_HOVER && !r.sawCut, "%s for %d ms: ignored", t[i].what, t[i].ms - 1);
    int s = 0;
    while (s < t[i].ms + 10 && r.out.state == AUTO_HOVER) {
      setTrip(&r.in, t[i].k, true);
      rigStep(&r);
      s++;
    }
    CHECK(s == t[i].ms + 1, "%s: KILLED once it held %d ms (step %d)", t[i].what, t[i].ms, s);
    CHECK(r.out.state == AUTO_KILLED && r.out.reason == t[i].why && r.out.thrust == AUTO_THR_CUT &&
          r.out.setEmergencyStop && r.out.vzCmd == VZ_CMD_CUT && r.applied == 0.0f,
          "%s: KILLED %s, CUT, setEmergencyStop", t[i].what, autoReasonName(r.out.reason));
  }

  /* An interrupted condition starts over. */
  rig_t r;
  rigInit(&r, 11000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  for (int k = 0; k < 4; k++) {
    for (int s = 0; s < 40; s++) {
      setTrip(&r.in, TRIP_TILT, true);
      rigStep(&r);
    }
    setTrip(&r.in, TRIP_TILT, false);
    rigStep(&r);
  }
  CHECK(r.out.state == AUTO_HOVER, "four 40 ms tilts with gaps: no trip");
  r.in.gyroDps[2] = 900.0f;
  run(&r, 100);
  CHECK(r.out.state == AUTO_HOVER, "|gz| is not a rate trip");
  imuNormal(&r.in);

  /* During the controller's spin (TAKEOFF after pre-spin) the trips apply. */
  rigInit(&r, 11000);
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  run(&r, AUTO_PRESPIN_MS + 100);
  CHECK(r.out.thrust == AUTO_THR_VERT_CTRL, "spinning");
  for (int s = 0; s <= AUTO_TILT_KILL_MS; s++) {
    setTrip(&r.in, TRIP_TILT, true);
    rigStep(&r);
  }
  CHECK(r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_TILT, "tilt during the spin: KILLED");

  /* Pre-spin (motors off): the drone was moved, back to IDLE, no cut. */
  rigInit(&r, 11000);
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  run(&r, 100);
  int s = 0;
  while (r.out.state == AUTO_TAKEOFF && s < 500) {
    setTrip(&r.in, TRIP_TILT, true);
    rigStep(&r);
    s++;
  }
  CHECK(r.out.state == AUTO_IDLE && r.out.reason == AUTO_RS_MOVED && s == AUTO_MOVED_MS + 1 && !r.sawCut,
        "tilt in pre-spin: IDLE MOVED after 100 ms (%d), no cut", s);
}

/* ---- in-air contact (LAND COLLISION, 2026-10-13 flight 1) ------------------ */

/* Holds the IMU on acc (g) / gx / gz (dps) for n steps, then normal; returns
 * the step (1-based) the state left st, 0 if it never did. */
static int holdImu(rig_t *r, int n, float acc, float gx, float gz, autoState_t st)
{
  int left = 0;
  for (int k = 1; k <= n; k++) {
    imuNormal(&r->in);
    r->in.accNormG = acc;
    r->in.gyroDps[0] = gx;
    r->in.gyroDps[2] = gz;
    rigStep(r);
    if (left == 0 && r->out.state != st) {
      left = k;
    }
  }
  imuNormal(&r->in);
  return left;
}

/* The step after a contact in the climb without the camera height (the blind LAND, 2026-10-13 "try a slow landing";
 * since 2026-10-15 only there): LAND COLLISION on the controller's landing, no cut, collided (the blind descent and
 * the hold block) from this very step. */
static bool collisionLand(const rig_t *r)
{
  return r->out.state == AUTO_LAND && r->out.reason == AUTO_RS_COLLISION && r->out.thrust == AUTO_THR_VERT_LAND &&
         r->out.vzCmd == VZ_CMD_LAND && r->out.collided && !r->out.setEmergencyStop && !r->sawCut &&
         r->holdBlock && r->applied > 0.0f;
}

/* The latest event of the report: its code (0: none). */
static autoEvent_t lastEvt(const rig_t *r)
{
  return (autoEvent_t)autoFsmReport(&r->f)->evCode[0];
}

static uint8_t lastEvtDetail(const rig_t *r)
{
  return autoFsmReport(&r->f)->evDetail[0];
}

/* Was e among the three latest events (its detail in *d)? */
static bool hadEvt(const rig_t *r, autoEvent_t e, uint8_t *d)
{
  const autoFsmReport_t *p = autoFsmReport(&r->f);
  for (int i = 0; i < 3; i++) {
    if (p->evCode[i] == (uint8_t)e) {
      if (d != NULL) {
        *d = p->evDetail[i];
      }
      return true;
    }
  }
  return false;
}

/* A contact in HOVER ("Claude decides", 2026-10-15; LAND COLLISION until then): CONTACT, a 500 ms hush
 * (vzInput_t.hush), the event with its peaks and side, no LAND; two hushes at most in any 2 s; a LAND that begins
 * in the hush or before the camera height is back is blind. */
static void testContact(void)
{
  rig_t r;
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(holdImu(&r, 3, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && !r.sawCut && !r.out.collided && !r.out.hush &&
        autoFsmReport(&r.f)->contacts == 0, "4 g for 2 ms: nothing");
  run(&r, 10);
  const uint16_t n0 = autoFsmReport(&r.f)->evCount;
  for (int k = 1; k <= 4; k++) {
    imuNormal(&r.in);
    r.in.accNormG = k == 4 ? 4.4f : 4.0f;
    r.in.accXG = -3.9f;   /* pushed backward: the obstacle in front */
    r.in.accYG = 0.5f;
    rigStep(&r);
  }
  imuNormal(&r.in);
  r.in.accXG = r.in.accYG = 0.0f;
  const autoFsmReport_t *p = autoFsmReport(&r.f);
  CHECK(r.out.state == AUTO_HOVER && r.out.hush && lastEvt(&r) == AUTO_EV_CONTACT &&
            (lastEvtDetail(&r) & 0x3F) == VZ_HOLD && p->contacts == 1 && p->evCount == n0 + 1u &&
            (p->holdBits & AUTO_HB_HUSH) && !r.out.collided && !r.sawCut && !r.holdBlock,
        "4 g held 3 ms in HOVER: CONTACT (no LAND), the hush, the event (detail %u)", (unsigned)lastEvtDetail(&r));
  CHECK(fabsf(p->peakAccG - 4.4f) < 1e-6f && fabsf(p->contactAxG + 3.9f) < 1e-6f && fabsf(p->contactAyG - 0.5f) < 1e-6f,
        "the peak %.2f g and the body-frame force (%.2f, %.2f) at it", p->peakAccG, p->contactAxG, p->contactAyG);
  const int hush0 = r.hushSteps;
  int hushLen = 1;
  while (r.out.hush && hushLen < 1000) {
    rigStep(&r);
    hushLen += r.out.hush;
  }
  CHECK(hushLen == AUTO_CONTACT_HUSH_MS && r.hushSteps - hush0 == AUTO_CONTACT_HUSH_MS - 1 &&
            !(autoFsmReport(&r.f)->holdBits & AUTO_HB_HUSH) && r.out.state == AUTO_HOVER,
        "the hush lasts %d ms (the controller given vzInput_t.hush), then HOVER on", hushLen);
  /* blind until the camera height has been in use 500 ms after the hush */
  CHECK(r.f.contactBlind, "after the hush: blind until the camera height is back 500 ms");
  run(&r, AUTO_CAM_BACK_MS + 2);
  CHECK(!r.f.contactBlind, "the camera height in use 500 ms: not blind any more");
  /* the chain: a second contact hushes, a third within 2 s of the first two's is reported only */
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  int hushes = 0, contacts = 0;
  bool hushPrev = false;
  for (int k = 0; k < 3000; k++) {   /* a prop rubbing a wall: 1.8 g for 3 s */
    imuNormal(&r.in);
    r.in.accNormG = 1.8f;
    rigStep(&r);
    hushes += r.out.hush && !hushPrev;
    hushPrev = r.out.hush;
    contacts = autoFsmReport(&r.f)->contacts;
  }
  imuNormal(&r.in);
  /* contacts at 3, 503 (hushed back to back: 1 s), 1003 (the bound: reported only), 2003 and 2503 (hushed): at most
   * 1 s of hush in any 2 s */
  CHECK(r.out.state == AUTO_HOVER && hushes == 2 && contacts == 5 && r.hushSteps >= 1990 && r.hushSteps <= 2000,
        "1.8 g for 3 s: %d hush runs of two each, %d contacts reported, %d ms hushed (<= 1 s in any 2 s), no LAND",
        hushes, contacts, r.hushSteps);
  /* a LAND in the hush is blind */
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT");
  run(&r, 100);
  autoFsmOut_t lo = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(lo.state == AUTO_LAND && lo.reason == AUTO_RS_LAND_CMD && lo.collided && r.holdBlock,
        "Claude's LAND 100 ms into the hush: blind (collided), the hold blocked");
  /* ... and after it, until the camera height is back */
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  run(&r, 2000);   /* capQ: the controller's open loop is bounded (no SINK at once) */
  r.vz.noCam = true;
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT, the camera height out");
  run(&r, 600);
  CHECK(r.out.state == AUTO_HOVER && !r.out.hush, "the hush over, HOVER (the budget from capQ: %.0f)", r.vz.o.capQ);
  lo = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(lo.state == AUTO_LAND && lo.collided, "a LAND after the hush with the camera height out: blind");

  /* gz 400 dps (a turn), 1.5 g and gx 249 dps for 1 s do nothing; gx / gy 300 dps held 3 ms: CONTACT */
  const struct { float acc, gx, gz; const char *what; } quiet[] = {
    { 1.0f, 0.0f, 400.0f, "gz 400 dps" }, { 1.5f, 0.0f, 0.0f, "1.5 g" }, { 1.0f, 249.0f, 0.0f, "gx 249 dps" },
  };
  for (unsigned i = 0; i < sizeof(quiet) / sizeof(quiet[0]); i++) {
    rigInit(&r, 21000);
    r.vz.neverSteady = true;
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
    CHECK(holdImu(&r, 1000, quiet[i].acc, quiet[i].gx, quiet[i].gz, AUTO_HOVER) == 0 && !r.sawCut &&
          autoFsmReport(&r.f)->contacts == 0, "%s for 1 s: nothing (%s %s)", quiet[i].what,
          autoStateName(r.out.state), autoReasonName(r.out.reason));
  }
  CHECK(holdImu(&r, 4, 1.0f, 300.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush && autoFsmReport(&r.f)->contacts == 1,
        "gx 300 dps held 3 ms: CONTACT");
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  for (int k = 1; k <= 4; k++) {
    imuNormal(&r.in);
    r.in.gyroDps[1] = -300.0f;
    rigStep(&r);
  }
  imuNormal(&r.in);
  CHECK(r.out.hush && r.out.state == AUTO_HOVER && fabsf(autoFsmReport(&r.f)->peakRateDps - 300.0f) < 1e-3f,
        "gy -300 dps held 3 ms: CONTACT, its peak rate %.0f dps", autoFsmReport(&r.f)->peakRateDps);

  /* TAKEOFF CLIMB: armed 200 ms after the controller's lift-off; with the camera height a CONTACT (the controller
   * ends the climb in HOLD), without it the blind LAND COLLISION */
  for (int late = 0; late < 3; late++) {
    rigInit(&r, 21000);
    r.vz.climbMs = 2000;   /* still climbing at lift-off + 250 ms */
    r.vz.noCam = late == 2;
    CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
    int guard = 0;
    while (!r.vz.o.liftoffNow && guard++ < 6000) {
      rigStep(&r);
    }
    /* the FSM reads the lift-off on the next step (the controller's previous output) */
    run(&r, late ? 250 : 150);
    CHECK(r.vz.o.phase == VZ_CLIMB && r.out.state == AUTO_TAKEOFF, "climbing at lift-off + %d ms", late ? 250 : 150);
    const int left = holdImu(&r, 6, 4.0f, 0.0f, 0.0f, AUTO_TAKEOFF);
    if (late == 2) {
      CHECK(left == 4 && collisionLand(&r), "4 g at lift-off + 250 ms without the camera height: LAND COLLISION "
            "(blind) from TAKEOFF (step %d)", left);
      rigStep(&r);
      CHECK(r.vz.o.phase == VZ_LAND, "the controller lands from CLIMB");
    } else if (late == 1) {
      CHECK(left == 0 && r.out.state == AUTO_TAKEOFF && r.out.hush && !r.out.collided && !r.sawCut,
            "4 g at lift-off + 250 ms with the camera height: CONTACT, the hush (the controller ends the climb)");
    } else {
      CHECK(left == 0 && r.out.state == AUTO_TAKEOFF && !r.sawCut && !r.out.collided && !r.out.hush,
            "4 g at lift-off + 150 ms: nothing (the lift-off kick is not a contact)");
    }
  }
  /* before the lift-off (RAMP): never */
  rigInit(&r, 21000);
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  run(&r, AUTO_PRESPIN_MS + r.vz.spinMs + 100);
  CHECK(r.vz.o.phase == VZ_RAMP, "in RAMP");
  CHECK(holdImu(&r, 50, 4.0f, 0.0f, 0.0f, AUTO_TAKEOFF) == 0 && !r.out.collided && !r.out.hush, "4 g in RAMP: nothing");

  /* LAND: 8 g and 800 dps at a touchdown under power: the touchdown rules as before, never a contact */
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  run(&r, r.vz.tdOkAfterMs + 400);
  CHECK(r.vz.o.tdThrustOk, "tdThrustOk");
  int s = 0;
  while (r.out.thrust == AUTO_THR_VERT_LAND && s < 1000) {
    imuNormal(&r.in);
    r.in.accNormG = 8.0f;
    r.in.gyroDps[0] = 800.0f;
    rigStep(&r);
    s++;
  }
  imuNormal(&r.in);
  CHECK(r.out.state == AUTO_LAND && r.out.landTrip == AUTO_RS_RATE && !r.sawCut && r.out.reason != AUTO_RS_COLLISION &&
        !r.out.collided && autoFsmReport(&r.f)->contacts == 0, "LAND, 8 g / 800 dps with tdThrustOk: touchdown (trip "
        "%s), not KILLED, no contact", autoReasonName(r.out.landTrip));
  CHECK(toIdle(&r, 4000) && r.out.reason == AUTO_RS_TOUCHDOWN_TRIP, "then IDLE TOUCHDOWN_TRIP");
  /* LAND without the thrust condition: the rate trip (30 ms) */
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  run(&r, 200);
  const int left = holdImu(&r, 100, 8.0f, 800.0f, 0.0f, AUTO_LAND);
  CHECK(left == AUTO_RATE_KILL_MS + 1 && r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_RATE,
        "LAND early, 8 g / 800 dps: KILLED RATE after 30 ms (step %d, %s)", left, autoReasonName(r.out.reason));

  /* DRY: a finger tap is a CONTACT (the bench check); BENCH: nothing */
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_DRY, 2, 5000), "DRY hover");
  CHECK(holdImu(&r, 4, 3.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.state == AUTO_HOVER && r.out.hush &&
        lastEvt(&r) == AUTO_EV_CONTACT && !r.sawCut, "DRY: 3 g held 3 ms: CONTACT");
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_BENCH, 2, 5000), "BENCH hover");
  CHECK(holdImu(&r, 25, 3.0f, 300.0f, 0.0f, AUTO_HOVER) == 0 && !r.sawCut && !r.out.hush &&
        autoFsmReport(&r.f)->contacts == 0, "BENCH: 3 g / 300 dps for 24 ms: nothing");
}

/* After a CONTACT every trip still cuts (a drone that flips, spins or falls after the contact), the Mac KILL cuts. */
static void testContactThenTrips(void)
{
  const struct { tripKind_t k; int ms; autoReason_t why; const char *what; } t[] = {
    { TRIP_TILT, AUTO_TILT_KILL_MS, AUTO_RS_TILT, "tilt 31 deg" },
    { TRIP_RATE_Y, AUTO_RATE_KILL_MS, AUTO_RS_RATE, "gy -501 dps" },
    { TRIP_FALL, AUTO_FALL_KILL_MS, AUTO_RS_FREEFALL, "|acc| 0.39 g" },
    { TRIP_ATTERR, AUTO_ATTERR_KILL_MS, AUTO_RS_ATT_ERR, "attErr 10.5 deg" },
  };
  for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
    rig_t r;
    rigInit(&r, 23000);
    r.vz.neverSteady = true;
    r.vz.neverDone = true;
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
    CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "%s: CONTACT", t[i].what);
    run(&r, 50);
    int s = 0;
    while (s < t[i].ms + 10 && r.out.state == AUTO_HOVER) {
      setTrip(&r.in, t[i].k, true);
      rigStep(&r);
      s++;
    }
    setTrip(&r.in, t[i].k, false);
    CHECK(s == t[i].ms + 1 && r.out.state == AUTO_KILLED && r.out.reason == t[i].why && r.out.thrust == AUTO_THR_CUT &&
          r.out.setEmergencyStop && r.applied == 0.0f,
          "CONTACT, then %s: KILLED %s once it held %d ms (step %d)", t[i].what, autoReasonName(r.out.reason),
          t[i].ms, s);
  }
  /* a contact that spins the drone: CONTACT after 3 ms, KILLED RATE when the rate held 30 ms */
  rig_t r;
  rigInit(&r, 23000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  int contact = 0, kill = 0;
  for (int k = 1; k <= 40 && kill == 0; k++) {
    imuNormal(&r.in);
    r.in.accNormG = 2.5f;
    r.in.gyroDps[0] = 600.0f;
    rigStep(&r);
    if (contact == 0 && r.out.hush) {
      contact = k;
    }
    if (r.out.state == AUTO_KILLED) {
      kill = k;
    }
  }
  imuNormal(&r.in);
  CHECK(contact == 4 && kill == AUTO_RATE_KILL_MS + 1 && r.out.reason == AUTO_RS_RATE,
        "600 dps held: CONTACT at step %d, then KILLED RATE at step %d (%s)", contact, kill,
        autoReasonName(r.out.reason));
  /* the Mac KILL in a hush */
  rigInit(&r, 23000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT");
  run(&r, 100);
  autoFsmOut_t o = doCmd(&r, cmdOf(AUTO_CMD_KILL));
  CHECK(o.state == AUTO_KILLED && o.reason == AUTO_RS_KILL_CMD && r.applied == 0.0f, "then the Mac KILL: KILLED");
  /* HB loss and the battery in a hush: LAND, blind */
  rigInit(&r, 23000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  run(&r, 2000);   /* capQ */
  r.vz.noCam = true;   /* the camera height out after the contact: blind until it is back */
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT");
  r.hbStream = false;
  CHECK(untilState(&r, AUTO_LAND, 1500) > 0 && r.out.reason == AUTO_RS_HB_LOST && r.out.collided,
        "the link lost after a contact: LAND HB_LOST, blind (the camera height was not back after it)");
}

/* ---- 6. trips in LAND ---------------------------------------------------- */

typedef enum { CONTACT_NONE, CONTACT_ACC, CONTACT_JOLT, CONTACT_OLD } contact_t;

/* LAND with tdThrustOk set, a contact sign (or none) just before the trip,
 * then the trip held until something happens. */
static void landTripCase(rig_t *r, tripKind_t k, contact_t ct)
{
  rigInit(r, 13000);
  r->vz.neverSteady = true;
  r->vz.neverDone = true;
  CHECK(toHover(r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  run(r, r->vz.tdOkAfterMs + 400);
  CHECK(r->vz.o.tdThrustOk, "tdThrustOk");
  if (ct == CONTACT_ACC || ct == CONTACT_OLD) {
    r->in.accNormG = 2.0f;   /* a touchdown under power: 1.8-3.1 g logged */
    rigStep(r);
    imuNormal(&r->in);
    if (ct == CONTACT_OLD) {
      run(r, AUTO_TD_CONTACT_WIN_MS);   /* too long ago when the trip fires */
    }
  } else if (ct == CONTACT_JOLT) {
    r->vz.o.aRel = 0.7f;
    rigStep(r);
    r->vz.o.aRel = 0.0f;
  }
  int s = 0;
  while (r->out.thrust == AUTO_THR_VERT_LAND && r->out.state == AUTO_LAND && s < 1000) {
    setTrip(&r->in, k, true);
    rigStep(r);
    s++;
  }
  setTrip(&r->in, k, false);
}

static void testLandTrips(void)
{
  const tripKind_t kinds[] = { TRIP_TILT, TRIP_RATE_X, TRIP_FALL, TRIP_ATTERR };
  const autoReason_t whys[] = { AUTO_RS_TILT, AUTO_RS_RATE, AUTO_RS_FREEFALL, AUTO_RS_ATT_ERR };
  const contact_t cts[] = { CONTACT_NONE, CONTACT_ACC, CONTACT_JOLT, CONTACT_OLD };
  const char *const ctName[] = { "no contact", "|acc| 2 g", "aRel 0.7 g", "contact 300 ms before" };
  for (int k = 0; k < 4; k++) {
    for (int j = 0; j < 4; j++) {
      rig_t r;
      landTripCase(&r, kinds[k], cts[j]);
      /* touchdown only for tilt/rate with a fresh contact sign */
      const bool td = (kinds[k] == TRIP_TILT || kinds[k] == TRIP_RATE_X) &&
                      (cts[j] == CONTACT_ACC || cts[j] == CONTACT_JOLT);
      if (td) {
        CHECK(r.out.state == AUTO_LAND && r.out.thrust == AUTO_THR_ZERO && r.out.vzCmd == VZ_CMD_CUT &&
              r.out.reason == AUTO_RS_LAND_CMD && r.out.landTrip == whys[k] && !r.sawCut,
              "%s trip with tdThrustOk and %s: touchdown, not KILLED (%s, trip %s)", autoReasonName(whys[k]),
              ctName[j], autoStateName(r.out.state), autoReasonName(r.out.landTrip));
        rigStep(&r);
        CHECK(r.vz.o.phase == VZ_CUT, "the controller is cut");
        CHECK(toIdle(&r, 4000) && r.out.reason == AUTO_RS_TOUCHDOWN_TRIP && r.setEstops == 0,
              "then IDLE TOUCHDOWN_TRIP");
      } else {
        CHECK(r.out.state == AUTO_KILLED && r.out.reason == whys[k] && r.out.setEmergencyStop,
              "%s trip with tdThrustOk and %s: KILLED %s (%s)", autoReasonName(whys[k]), ctName[j],
              autoReasonName(whys[k]), autoReasonName(r.out.reason));
      }
    }

    /* without the thrust condition: KILLED, contact or not */
    rig_t r;
    rigInit(&r, 13000);
    r.vz.neverSteady = true;
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
    CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
    run(&r, 200);
    CHECK(!r.vz.o.tdThrustOk, "tdThrustOk not yet");
    r.in.accNormG = 2.0f;
    rigStep(&r);
    imuNormal(&r.in);
    int s = 0;
    while (r.out.state == AUTO_LAND && s < 1000) {
      setTrip(&r.in, kinds[k], true);
      rigStep(&r);
      s++;
    }
    CHECK(r.out.state == AUTO_KILLED && r.out.reason == whys[k], "%s trip early in LAND (contact seen): KILLED",
          autoReasonName(whys[k]));
  }

  /* After the controller ended, trips are ignored. */
  rig_t r;
  rigInit(&r, 13000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  CHECK(untilLandEnded(&r, 7000), "touchdown");
  for (int s = 0; s < 200; s++) {
    setTrip(&r.in, TRIP_RATE_X, true);
    rigStep(&r);
  }
  setTrip(&r.in, TRIP_RATE_X, false);
  CHECK(r.out.state == AUTO_LAND && !r.sawCut, "a trip after touchdown is ignored");
  CHECK(toIdle(&r, 4000) && r.out.reason == AUTO_RS_TOUCHDOWN && r.out.landTrip == AUTO_RS_NONE, "IDLE TOUCHDOWN");
}

/* ---- 7. non-finite inputs ------------------------------------------------ */

static float *nanField(rig_t *r, int i, const char **name)
{
  autoFsmIn_t *in = &r->in;
  float *const f[] = { &in->rollDeg, &in->pitchDeg, &in->tiltDeg, &in->attErrDeg, &in->gyroDps[0], &in->gyroDps[1],
                       &in->gyroDps[2], &in->accNormG, &in->vbat, &in->vbatFastMin, &r->vz.o.h, &r->vz.o.v,
                       &r->vz.o.thrust };
  const char *const n[] = { "roll", "pitch", "tilt", "attErr", "gx", "gy", "gz", "accNorm", "vbat", "vbatFastMin",
                            "vz.h", "vz.v", "vz.thrust" };
  if (i < 0 || i >= (int)(sizeof(f) / sizeof(f[0]))) {
    return NULL;
  }
  *name = n[i];
  return f[i];
}

static void testNaN(void)
{
  const char *name = "";
  rig_t r;
  for (int i = 0; nanField(&r, i, &name) != NULL; i++) {
    for (int inf = 0; inf < 2; inf++) {
      rigInit(&r, 15000);
      r.vz.neverSteady = true;
      CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
      float *p = nanField(&r, i, &name);
      *p = inf ? -INFINITY : NAN;
      rigStep(&r);
      CHECK(r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_NAN && r.out.thrust == AUTO_THR_CUT,
            "%s %s airborne: KILLED NAN in that step (%s)", name, inf ? "-Inf" : "NaN", autoStateName(r.out.state));
    }
  }
  /* LAND, too */
  rigInit(&r, 15000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  run(&r, 100);
  r.in.accNormG = NAN;
  rigStep(&r);
  CHECK(r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_NAN, "NaN in LAND: KILLED NAN");

  /* IDLE: every sensor and battery float refuses ARM. The controller's
   * output is the previous sortie's in IDLE (vzReset starts it afresh), so
   * it does not. */
  for (int i = 0; nanField(&r, i, &name) != NULL; i++) {
    rigInit(&r, 15000);
    run(&r, 10);
    float *p = nanField(&r, i, &name);
    const bool vzField = p == &r.vz.o.h || p == &r.vz.o.v || p == &r.vz.o.thrust;
    *p = NAN;
    const autoFsmOut_t o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
    if (vzField) {
      CHECK(o.cmdResult == AUTO_R_OK, "%s NaN (stale controller output) in IDLE: ARM not blocked", name);
    } else {
      CHECK(o.cmdResult == AUTO_R_GUARDS && o.cmdDetail != 0 && o.state == AUTO_IDLE,
            "%s NaN in IDLE: ARM refused (guards 0x%x)", name, (unsigned)o.cmdDetail);
    }
  }
  rigInit(&r, 15000);
  r.in.vmin = NAN;
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).cmdDetail == AUTO_G_BATT, "vmin NaN: G3");
  rigInit(&r, 15000);
  r.in.camFps = 5.55f;
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).cmdResult == AUTO_R_OK, "native VGA 5.55 fps: camera readiness accepts ARM in simulation");
  rigInit(&r, 15000);
  r.in.camFps = 4.49f;
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).cmdDetail == AUTO_G_CAMERA, "native VGA below 4.5 fps: camera readiness refuses ARM");
  rigInit(&r, 15000);
  r.in.camFps = NAN;
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).cmdDetail == AUTO_G_CAMERA, "camFps NaN: G4");
}

/* ---- 8. Mac KILL ---------------------------------------------------------- */

typedef enum { AT_IDLE, AT_ARMED, AT_PRESPIN, AT_SPIN, AT_HOVER, AT_LAND, AT_LANDED, AT_KILLED, AT_N } where_t;
static const char *const whereName[AT_N] = { "IDLE", "ARMED", "pre-spin", "TAKEOFF spin", "HOVER", "LAND",
                                             "LAND after touchdown", "KILLED" };

/* Brings a fresh rig to a state. */
static bool goTo(rig_t *r, where_t w)
{
  r->vz.neverSteady = true;
  switch (w) {
    case AT_IDLE:
      run(r, 10);
      return r->out.state == AUTO_IDLE;
    case AT_ARMED:
      prepMode(r, AUTO_MODE_FLIGHT);
      r->in.airMaxNvsMs = 5000;
      return doCmd(r, cmdArm(AUTO_MODE_FLIGHT, 5000)).state == AUTO_ARMED;
    case AT_PRESPIN:
      if (!launch(r, AUTO_MODE_FLIGHT, 2, 5000)) {
        return false;
      }
      run(r, 300);
      return r->out.state == AUTO_TAKEOFF && r->out.prespinLed;
    case AT_SPIN:
      if (!launch(r, AUTO_MODE_FLIGHT, 2, 5000)) {
        return false;
      }
      run(r, AUTO_PRESPIN_MS + 200);
      return r->out.state == AUTO_TAKEOFF && r->out.thrust == AUTO_THR_VERT_CTRL;
    case AT_HOVER:
      return toHover(r, AUTO_MODE_FLIGHT, 2, 5000);
    case AT_LAND:
    case AT_LANDED:
      if (!toHover(r, AUTO_MODE_FLIGHT, 2, 5000) || doCmd(r, cmdOf(AUTO_CMD_LAND)).state != AUTO_LAND) {
        return false;
      }
      if (w == AT_LANDED && !untilLandEnded(r, 7000)) {
        return false;
      }
      run(r, 100);
      return r->out.state == AUTO_LAND;
    case AT_KILLED:
      if (!toHover(r, AUTO_MODE_FLIGHT, 2, 5000)) {
        return false;
      }
      r->in.killCount++;
      rigStep(r);
      run(r, 100);
      return r->out.state == AUTO_KILLED;
    default:
      return false;
  }
}

static void testKill(void)
{
  for (int w = 0; w < AT_N; w++) {
    for (int viaCount = 0; viaCount < 2; viaCount++) {
      rig_t r;
      rigInit(&r, 17000);
      CHECK(goTo(&r, (where_t)w), "reach %s", whereName[w]);
      const autoState_t before = r.out.state;
      if (viaCount) {
        r.in.killCount++;
        rigStep(&r);
      } else {
        doCmd(&r, cmdOf(AUTO_CMD_KILL));
        CHECK(r.out.cmdDone && r.out.cmdResult == AUTO_R_OK && r.out.cmdDetail == (uint32_t)before,
              "KILL acknowledged OK in %s", whereName[w]);
      }
      if (w == AT_IDLE) {
        CHECK(r.out.state == AUTO_IDLE && r.out.thrust == AUTO_THR_NONE, "KILL in IDLE: stays IDLE");
      } else {
        CHECK(r.out.state == AUTO_KILLED && r.out.thrust == AUTO_THR_CUT && r.applied == 0.0f,
              "Mac KILL (%s) in %s: KILLED and thrust 0 in the same step", viaCount ? "count" : "command",
              whereName[w]);
        CHECK(w == AT_KILLED || (r.out.reason == AUTO_RS_KILL_CMD && r.out.setEmergencyStop),
              "reason KILL_CMD, setEmergencyStop");
      }
    }
  }

  /* Kill latch */
  rig_t r;
  rigInit(&r, 17000);
  run(&r, 10);
  doCmd(&r, cmdOf(AUTO_CMD_KILL));
  CHECK(autoFsmGuards(&r.f, &r.in, AUTO_MODE_FLIGHT) == AUTO_G_KILL, "G11 is the only failed guard after a KILL");
  CHECK(r.out.failedGuards == AUTO_G_KILL, "and STATUS shows it");
  run(&r, AUTO_KILL_LATCH_MS - 2);
  autoFsmOut_t o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(o.cmdResult == AUTO_R_KILL_LATCHED && o.state == AUTO_IDLE, "ARM 1999 ms after KILL: KILL_LATCHED");
  o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(o.cmdResult == AUTO_R_OK && o.state == AUTO_ARMED, "ARM 2000 ms after KILL: accepted");

  rigInit(&r, 17000);
  r.in.killCount = 7;   /* a KILL counted by the link task between two steps */
  CHECK(autoFsmGuards(&r.f, &r.in, AUTO_MODE_FLIGHT) & AUTO_G_KILL, "a KILL not yet stepped already fails G11");
  o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(o.cmdResult == AUTO_R_KILL_LATCHED, "ARM in the step that sees a KILL: KILL_LATCHED");
}

/* ---- 9. guards ------------------------------------------------------------ */

typedef struct {
  const char *what;
  autoMode_t mode;
  uint32_t expect;   /* failed bitmap, 0 = accepted */
  void (*apply)(rig_t *r);
} guardCase_t;

static void gLinkSession(rig_t *r) { r->in.sessionValid = false; }
static void gLinkHb(rig_t *r) { r->in.hbAgeMs = 300; }
static void gLinkHbOk(rig_t *r) { r->in.hbAgeMs = 299; }
static void gRestart(rig_t *r) { r->in.restartLocked = true; }
static void gResetLock(rig_t *r) { r->in.resetLocked = true; }
static void gEstop(rig_t *r) { r->estop = true; }
static void gMotors(rig_t *r) { r->in.motorsReady = false; }
static void gGuardOff(rig_t *r) { r->in.motorGuardOn = false; }
static void gGuardTrip(rig_t *r) { r->in.motorGuardTripped = true; }
static void gNotGround(rig_t *r) { r->in.onGround = false; }
static void gGroundShort(rig_t *r) { r->in.onGroundMs = 2999; }
static void gSettleShort(rig_t *r) { r->in.onGroundMs = AUTO_SETTLE_MIN_MS - 1; }
static void gSettleOk(rig_t *r) { r->in.onGroundMs = AUTO_SETTLE_MIN_MS; }
static void gGroundMin(rig_t *r) { r->in.onGroundMs = AUTO_ONGROUND_MIN_MS; }
static void gAcc(rig_t *r) { r->in.accNormG = 1.051f; }
static void gGyro(rig_t *r) { r->in.gyroDps[2] = -3.1f; }
static void gTilt(rig_t *r) { r->in.tiltDeg = 8.1f; }
static void gBatt(rig_t *r) { r->in.vbat = AUTO_VMIN_DEFAULT - 0.01f; }
static void gBattEdge(rig_t *r) { r->in.vbat = AUTO_VMIN_DEFAULT; }
static void gBattDry(rig_t *r) { r->in.vbat = 3.50f; }
static void gBattNan(rig_t *r) { r->in.vbat = NAN; }
static void gVminLow(rig_t *r) { r->in.vmin = AUTO_VMIN_LO - 0.2f; }
static void gVminHi(rig_t *r) { r->in.vmin = 4.0f; r->in.vbat = 4.05f; }
static void gCamStandby(rig_t *r) { r->in.camState = CAM_STANDBY; }
static void gCamHold(rig_t *r) { r->in.holdEnabled = false; }
static void gCamFps(rig_t *r) { r->in.camFps = AUTO_FPS_MIN; }
static void gCamOff(rig_t *r) { r->in.camState = CAM_OFF; r->in.camFps = 0.0f; r->in.holdEnabled = false; }
static void gAppOld(rig_t *r) { r->in.appAgeMs = 500; }
static void gAppInput(rig_t *r) { r->in.appInputAgeMs = 999; }
static void gAppOff(rig_t *r) { r->in.appRequired = false; r->in.appAgeMs = UINT32_MAX; r->in.appGestureAgeMs = UINT32_MAX; }
static void gGcap(rig_t *r) { r->in.gcap = 15000; }
static void gGcap0(rig_t *r) { r->in.gcap = 0; }
static void gGcapHigh(rig_t *r) { r->in.gcap = 20001; }
static void gGcapMax(rig_t *r) { r->in.gcap = 20000; }
static void gGcapDry(rig_t *r) { r->in.gcap = 30000; }
static void gEnabled(rig_t *r) { r->in.autoEnabled = false; }
static void gVzParams(rig_t *r) { r->in.vzParamsOk = false; }
static void gHeap(rig_t *r) { r->in.heapIntFree = AUTO_HEAP_MIN - 1; }
static void gHeapOk(rig_t *r) { r->in.heapIntFree = AUTO_HEAP_MIN; }
static void gGesture(rig_t *r) { r->in.appGestureAgeMs = AUTO_GESTURE_WINDOW_MS; }
static void gGestureOff(rig_t *r) { r->in.gestureRequired = false; r->in.appGestureAgeMs = UINT32_MAX; }
static void gSim(rig_t *r) { r->in.simPlant = true; }
static void gNone(rig_t *r) { (void)r; }

/* Camera awake through every FLIGHT and BENCH take-off (user decision
 * 2026-10-12; docs/autonomy-a2.md section 4.11): out.camNoStandby from ARMED
 * to HOVER whatever the battery, never in IDLE or LAND, never in DRY (no
 * motors, no take-off window), latched at ARM. */
static void testCamAwake(void)
{
  rig_t r;
  rigInit(&r, 21000);
  CHECK(!r.out.camNoStandby, "IDLE: no camera override");
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0) && r.out.camNoStandby, "FLIGHT: camera awake in HOVER");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND && !r.out.camNoStandby, "LAND: override off");

  rigInit(&r, 22000);
  r.in.vmin = AUTO_VMIN_LO;
  r.in.vbat = AUTO_VMIN_LO + 0.01f;   /* a low NVS vmin: still awake */
  bool on = launch(&r, AUTO_MODE_FLIGHT, 2, 0) && r.out.camNoStandby;
  for (int i = 0; i < 3000 && on && (r.out.state == AUTO_TAKEOFF || r.out.state == AUTO_HOVER); i++) {
    rigStep(&r);
    on = r.out.camNoStandby || !(r.out.state == AUTO_TAKEOFF || r.out.state == AUTO_HOVER);
  }
  CHECK(on, "FLIGHT armed at 3.71 V (vmin 3.70): camera awake through TAKEOFF/HOVER");

  rigInit(&r, 23000);
  const autoFsmOut_t a = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(a.state == AUTO_ARMED && a.camNoStandby, "ARMED: override on before the ground start");
  r.in.vbat = 3.50f;   /* sag after ARM */
  run(&r, 50);
  CHECK(r.out.state == AUTO_ARMED && r.out.camNoStandby, "latched at ARM: a later sag does not drop it (%s, %d)",
        autoStateName(r.out.state), (int)r.out.camNoStandby);

  rigInit(&r, 24000);
  CHECK(launch(&r, AUTO_MODE_DRY, 2, 0) && !r.out.camNoStandby, "DRY: never an override");
  rigInit(&r, 25000);
  CHECK(launch(&r, AUTO_MODE_BENCH, 2, 0) && r.out.camNoStandby, "BENCH: camera awake (a take-off rehearsal)");
}

/* The settle rule as a sequence (docs/autonomy-a2.md section 4.11): after a
 * flight the ground clock restarts; a FLIGHT ARM is refused with STILL (and
 * the IDLE status reports STILL) until AUTO_SETTLE_MIN_MS of zero thrust, then
 * accepted; BENCH and DRY need only AUTO_ONGROUND_MIN_MS. */
static void testSettleSequence(void)
{
  rig_t r;
  rigInit(&r, 26000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  r.in.onGround = false;   /* the last flight */
  run(&r, 100);
  r.in.onGround = true;    /* landed: the glue's ground clock restarts at 0 */
  run(&r, AUTO_SETTLE_MIN_MS - 1);
  r.in.appGestureAgeMs = 2000;   /* a fresh phone gesture (its 15 s window is shorter than the settle) */
  CHECK(r.in.onGroundMs == AUTO_SETTLE_MIN_MS - 1 && (r.out.failedGuards & AUTO_G_STILL) != 0,
        "settle: IDLE reports STILL while settling (%u ms on the ground)", (unsigned)r.in.onGroundMs);
  autoFsmOut_t a = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(a.cmdResult == AUTO_R_GUARDS && a.cmdDetail == AUTO_G_STILL && a.state == AUTO_IDLE && !a.camNoStandby,
        "settle: FLIGHT ARM at 19999 ms refused with STILL only (result %d, detail 0x%x)", (int)a.cmdResult,
        (unsigned)a.cmdDetail);
  a = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(a.cmdResult == AUTO_R_OK && a.state == AUTO_ARMED, "settle: FLIGHT ARM at 20000 ms accepted (result %d, detail 0x%x)",
        (int)a.cmdResult, (unsigned)a.cmdDetail);

  /* A gesture made right after the landing has expired by the end of the
   * settle: the ARM is refused with GESTURE (the Mac daemon must ask for a
   * new one, not just retry). */
  rigInit(&r, 26500);
  prepMode(&r, AUTO_MODE_FLIGHT);
  r.in.onGround = false;
  run(&r, 100);
  r.in.onGround = true;
  r.in.appGestureAgeMs = 0;
  run(&r, AUTO_SETTLE_MIN_MS);
  a = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(a.cmdResult == AUTO_R_GUARDS && a.cmdDetail == AUTO_G_GESTURE,
        "settle: a gesture from the landing has expired after the settle (result %d, detail 0x%x)", (int)a.cmdResult,
        (unsigned)a.cmdDetail);

  const autoMode_t other[2] = { AUTO_MODE_BENCH, AUTO_MODE_DRY };
  for (int k = 0; k < 2; k++) {
    rigInit(&r, 27000 + 1000 * (uint32_t)k);
    prepMode(&r, other[k]);
    r.in.onGround = false;
    run(&r, 100);
    r.in.onGround = true;
    run(&r, AUTO_ONGROUND_MIN_MS);
    a = doCmd(&r, cmdArm(other[k], 0));
    CHECK(a.cmdResult == AUTO_R_OK && a.state == AUTO_ARMED, "settle: %s ARM 3 s after a flight accepted (result %d, detail 0x%x)",
          other[k] == AUTO_MODE_BENCH ? "BENCH" : "DRY", (int)a.cmdResult, (unsigned)a.cmdDetail);
  }
}

/* G2's heat soak after a flight (needs the user's sign-off): FLIGHT waits
 * 20 s + 1.5 x the last flight's motor time on the ground, at most 90 s,
 * refused with STILL meanwhile; DRY and BENCH are unchanged. */
static autoFsmOut_t soakArm(uint32_t lastMs, autoMode_t mode, uint32_t groundMs)
{
  static rig_t r;
  rigInit(&r, 41000);
  prepMode(&r, mode);
  r.in.lastFlightMotorMs = lastMs;
  r.in.onGround = false;   /* the last flight */
  run(&r, 100);
  r.in.onGround = true;    /* landed */
  run(&r, groundMs);
  r.in.appGestureAgeMs = 2000;
  return doCmd(&r, cmdArm(mode, 0));
}

static void testSoak(void)
{
  CHECK(soakArm(41000, AUTO_MODE_FLIGHT, 60000).cmdResult == AUTO_R_GUARDS &&
        soakArm(41000, AUTO_MODE_FLIGHT, 60000).cmdDetail == AUTO_G_STILL,
        "after a 41 s flight: FLIGHT ARM at 60 s refused with STILL only (detail 0x%x)",
        (unsigned)soakArm(41000, AUTO_MODE_FLIGHT, 60000).cmdDetail);
  CHECK(soakArm(41000, AUTO_MODE_FLIGHT, 81499).cmdDetail == AUTO_G_STILL, "after 41 s: refused at 81.499 s");
  CHECK(soakArm(41000, AUTO_MODE_FLIGHT, 81500).state == AUTO_ARMED, "after 41 s: accepted at 81.5 s");
  CHECK(soakArm(41000, AUTO_MODE_FLIGHT, 82000).state == AUTO_ARMED, "after 41 s: accepted at 82 s");
  CHECK(soakArm(5000, AUTO_MODE_FLIGHT, 27499).cmdDetail == AUTO_G_STILL, "after a 5 s hop: refused at 27.499 s");
  CHECK(soakArm(5000, AUTO_MODE_FLIGHT, 27500).state == AUTO_ARMED, "after a 5 s hop: accepted at 27.5 s");
  CHECK(soakArm(15000, AUTO_MODE_FLIGHT, 42500).state == AUTO_ARMED &&
        soakArm(15000, AUTO_MODE_FLIGHT, 42499).cmdDetail == AUTO_G_STILL, "after 15 s: 42.5 s");
  CHECK(soakArm(0, AUTO_MODE_FLIGHT, AUTO_SETTLE_MIN_MS).state == AUTO_ARMED &&
        soakArm(0, AUTO_MODE_FLIGHT, AUTO_SETTLE_MIN_MS - 1).cmdDetail == AUTO_G_STILL,
        "the first flight of a boot: G2's 20 s");
  CHECK(soakArm(200000, AUTO_MODE_FLIGHT, AUTO_SOAK_MAX_MS - 1).cmdDetail == AUTO_G_STILL &&
        soakArm(200000, AUTO_MODE_FLIGHT, AUTO_SOAK_MAX_MS).state == AUTO_ARMED &&
        soakArm(UINT32_MAX, AUTO_MODE_FLIGHT, AUTO_SOAK_MAX_MS).state == AUTO_ARMED, "at most 90 s");
  CHECK(soakArm(41000, AUTO_MODE_DRY, AUTO_ONGROUND_MIN_MS).state == AUTO_ARMED &&
        soakArm(41000, AUTO_MODE_BENCH, AUTO_ONGROUND_MIN_MS).state == AUTO_ARMED,
        "DRY and BENCH: 3 s after a 41 s flight as before");
  /* IDLE reports it (STATUS failedGuards, the guards for FLIGHT) */
  static rig_t r;
  rigInit(&r, 41000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  r.in.lastFlightMotorMs = 41000;
  r.in.onGround = false;
  run(&r, 100);
  r.in.onGround = true;
  run(&r, 30000);
  CHECK((r.out.failedGuards & AUTO_G_STILL) != 0, "IDLE 30 s after a 41 s flight: failedGuards STILL (0x%x)",
        (unsigned)r.out.failedGuards);
  run(&r, 52000);
  CHECK((r.out.failedGuards & AUTO_G_STILL) == 0, "IDLE 82 s after it: no STILL (0x%x)",
        (unsigned)r.out.failedGuards);
}

static void testGuards(void)
{
  const guardCase_t gc[] = {
    { "all pass (FLIGHT)", AUTO_MODE_FLIGHT, 0, gNone },
    { "G0 session invalid", AUTO_MODE_FLIGHT, AUTO_G_LINK, gLinkSession },
    { "G0 heartbeat 300 ms", AUTO_MODE_FLIGHT, AUTO_G_LINK, gLinkHb },
    { "G0 heartbeat 299 ms passes", AUTO_MODE_FLIGHT, 0, gLinkHbOk },
    { "G1 restart lock", AUTO_MODE_FLIGHT, AUTO_G_LOCKS, gRestart },
    { "G1 reset-lock", AUTO_MODE_FLIGHT, AUTO_G_LOCKS, gResetLock },
    { "G1 reset-lock: DRY exempt", AUTO_MODE_DRY, 0, gResetLock },
    { "G1 reset-lock (BENCH too)", AUTO_MODE_BENCH, AUTO_G_LOCKS, gResetLock },
    { "G1 restart lock: DRY exempt", AUTO_MODE_DRY, 0, gRestart },
    { "G1 emergency stop", AUTO_MODE_FLIGHT, AUTO_G_LOCKS, gEstop },
    { "G1 motors not ready", AUTO_MODE_FLIGHT, AUTO_G_LOCKS, gMotors },
    { "G1 motor watchdog off", AUTO_MODE_FLIGHT, AUTO_G_LOCKS, gGuardOff },
    { "G1 motor watchdog tripped", AUTO_MODE_FLIGHT, AUTO_G_LOCKS, gGuardTrip },
    { "G2 not on ground", AUTO_MODE_FLIGHT, AUTO_G_STILL, gNotGround },
    { "G2 on ground 2999 ms", AUTO_MODE_FLIGHT, AUTO_G_STILL, gGroundShort },
    { "G2 FLIGHT: 19999 ms since the last flight (heat soak)", AUTO_MODE_FLIGHT, AUTO_G_STILL, gSettleShort },
    { "G2 FLIGHT: 20000 ms passes", AUTO_MODE_FLIGHT, 0, gSettleOk },
    { "G2 FLIGHT: 3000 ms refused", AUTO_MODE_FLIGHT, AUTO_G_STILL, gGroundMin },
    { "G2 BENCH: 3000 ms passes", AUTO_MODE_BENCH, 0, gGroundMin },
    { "G2 DRY: 3000 ms passes", AUTO_MODE_DRY, 0, gGroundMin },
    { "G2 BENCH: 2999 ms refused", AUTO_MODE_BENCH, AUTO_G_STILL, gGroundShort },
    { "G2 |acc| 1.051 g", AUTO_MODE_FLIGHT, AUTO_G_STILL, gAcc },
    { "G2 gz -3.1 dps", AUTO_MODE_FLIGHT, AUTO_G_STILL, gGyro },
    { "G2 tilt 8.1 deg", AUTO_MODE_FLIGHT, AUTO_G_STILL, gTilt },
    { "G3 vbat 3.79 V (default vmin 3.80 V)", AUTO_MODE_FLIGHT, AUTO_G_BATT, gBatt },
    { "G3 vbat 3.80 V passes", AUTO_MODE_FLIGHT, 0, gBattEdge },
    { "G3 BENCH vbat 3.99 V", AUTO_MODE_BENCH, AUTO_G_BATT, gBatt },
    { "G3 DRY exempt (vbat 3.50 V)", AUTO_MODE_DRY, 0, gBattDry },
    { "G3 DRY vbat NaN", AUTO_MODE_DRY, AUTO_G_BATT, gBattNan },
    { "G3 vmin below AUTO_VMIN_LO out of range", AUTO_MODE_FLIGHT, AUTO_G_BATT, gVminLow },
    { "G3 vmin 4.0 V", AUTO_MODE_FLIGHT, 0, gVminHi },
    { "G4 camera standby", AUTO_MODE_FLIGHT, AUTO_G_CAMERA, gCamStandby },
    { "G4 hold disabled", AUTO_MODE_FLIGHT, AUTO_G_CAMERA, gCamHold },
    { "G4 minimum fps", AUTO_MODE_FLIGHT, AUTO_G_CAMERA, gCamFps },
    { "G4 waived in DRY", AUTO_MODE_DRY, 0, gCamOff },
    { "G4 waived in BENCH", AUTO_MODE_BENCH, 0, gCamOff },
    { "G5 app packet 500 ms old", AUTO_MODE_FLIGHT, AUTO_G_APP, gAppOld },
    { "G5 app input 999 ms ago", AUTO_MODE_FLIGHT, AUTO_G_APP, gAppInput },
    { "G5+G10 app not required", AUTO_MODE_FLIGHT, 0, gAppOff },
    { "G6 FLIGHT with gcap", AUTO_MODE_FLIGHT, AUTO_G_GCAP, gGcap },
    { "G6 BENCH without gcap", AUTO_MODE_BENCH, AUTO_G_GCAP, gGcap0 },
    { "G6 BENCH gcap 20001", AUTO_MODE_BENCH, AUTO_G_GCAP, gGcapHigh },
    { "G6 BENCH gcap 20000", AUTO_MODE_BENCH, 0, gGcapMax },
    { "G6 DRY any gcap", AUTO_MODE_DRY, 0, gGcapDry },
    { "G7 disabled", AUTO_MODE_FLIGHT, AUTO_G_ENABLED, gEnabled },
    { "G8 vertical values", AUTO_MODE_FLIGHT, AUTO_G_VZPARAMS, gVzParams },
    { "G9 heap 12287", AUTO_MODE_FLIGHT, AUTO_G_HEAP, gHeap },
    { "G9 heap 12288", AUTO_MODE_FLIGHT, 0, gHeapOk },
    { "G10 gesture 15 s ago", AUTO_MODE_FLIGHT, AUTO_G_GESTURE, gGesture },
    { "G10 gesture not required", AUTO_MODE_FLIGHT, 0, gGestureOff },
    { "G12 FLIGHT on the sim", AUTO_MODE_FLIGHT, AUTO_G_MODE, gSim },
  };
  for (size_t i = 0; i < sizeof(gc) / sizeof(gc[0]); i++) {
    rig_t r;
    rigInit(&r, 19000);
    run(&r, 5);
    prepMode(&r, gc[i].mode);
    gc[i].apply(&r);
    r.in.emergencyStop = r.estop;
    const autoFsm_t f0 = r.f;
    const uint32_t direct = autoFsmGuards(&r.f, &r.in, gc[i].mode);
    CHECK(memcmp(&f0, &r.f, sizeof(f0)) == 0, "%s: autoFsmGuards does not change the FSM", gc[i].what);
    const autoFsmOut_t o = doCmd(&r, cmdArm(gc[i].mode, 0));
    if (gc[i].expect == 0) {
      CHECK(direct == 0 && o.cmdResult == AUTO_R_OK && o.state == AUTO_ARMED, "%s: ARM accepted (0x%x, res %d)",
            gc[i].what, (unsigned)direct, (int)o.cmdResult);
    } else {
      CHECK(direct == gc[i].expect && o.cmdResult == AUTO_R_GUARDS && o.cmdDetail == gc[i].expect &&
            o.failedGuards == gc[i].expect && o.state == AUTO_IDLE,
            "%s: ARM refused with exactly bit 0x%x (got 0x%x, res %d)", gc[i].what, (unsigned)gc[i].expect,
            (unsigned)o.cmdDetail, (int)o.cmdResult);
    }
  }

  /* IDLE reports the FLIGHT guards every step. */
  rig_t r;
  rigInit(&r, 19000);
  r.in.gcap = 15000;
  r.in.heapIntFree = 100;
  run(&r, 3);
  CHECK(r.out.failedGuards == (AUTO_G_GCAP | AUTO_G_HEAP), "IDLE STATUS bitmap (0x%x)", (unsigned)r.out.failedGuards);

  /* TAKEOFF re-checks every guard for the ARM mode. */
  struct { const char *what; uint32_t bit; void (*apply)(rig_t *r); } re[] = {
    { "heartbeat 400 ms", AUTO_G_LINK, NULL },
    { "gz 4 dps", AUTO_G_STILL, NULL },
    { "gesture expired", AUTO_G_GESTURE, NULL },
    { "heap low", AUTO_G_HEAP, gHeap },
    { "reset-lock", AUTO_G_LOCKS, NULL },
  };
  for (size_t i = 0; i < sizeof(re) / sizeof(re[0]); i++) {
    rigInit(&r, 19000);
    prepMode(&r, AUTO_MODE_FLIGHT);
    if (re[i].bit == AUTO_G_GESTURE) {
      r.in.appGestureAgeMs = AUTO_GESTURE_WINDOW_MS - 500;   /* valid at ARM */
    }
    CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
    run(&r, 600);
    autoCmd_t t = cmdTakeoff(TOKEN, 30, 2, 150);
    if (re[i].bit == AUTO_G_LINK) {
      r.hbStream = false;
      r.in.hbAgeMs = 400;
    } else if (re[i].bit == AUTO_G_STILL) {
      r.in.gyroDps[2] = 4.0f;
    } else if (re[i].bit == AUTO_G_LOCKS) {
      /* a lock in ARMED returns to IDLE (row LOCKED); the TAKEOFF in the
       * same step is refused */
      r.in.resetLocked = true;
      const autoFsmOut_t o = doCmd(&r, t);
      CHECK(o.state == AUTO_IDLE && o.reason == AUTO_RS_LOCKED && o.cmdResult == AUTO_R_BAD_STATE,
            "reset-lock in ARMED: IDLE LOCKED, TAKEOFF refused (%s, res %d)", autoStateName(o.state),
            (int)o.cmdResult);
      continue;
    } else if (re[i].apply != NULL) {
      re[i].apply(&r);
    }
    const autoFsmOut_t o = doCmd(&r, t);
    CHECK(o.cmdResult == AUTO_R_GUARDS && o.cmdDetail == re[i].bit && o.state == AUTO_ARMED,
          "TAKEOFF with %s: GUARDS 0x%x, still ARMED (got 0x%x)", re[i].what, (unsigned)re[i].bit,
          (unsigned)o.cmdDetail);
  }

  /* DRY ignores the reset-lock all the way: ARM, the pre-spin and the sortie
   * (it cannot spin a motor); FLIGHT with the same lock is refused. */
  rigInit(&r, 19000);
  r.in.resetLocked = true;
  CHECK(toHover(&r, AUTO_MODE_DRY, 2, 0) && r.maxApplied == 0.0f, "reset-locked: a DRY sortie reaches HOVER, motors 0");
  rigInit(&r, 19000);
  r.in.resetLocked = true;
  prepMode(&r, AUTO_MODE_FLIGHT);
  autoFsmOut_t lo = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(lo.cmdResult == AUTO_R_GUARDS && lo.cmdDetail == AUTO_G_LOCKS, "reset-locked: FLIGHT ARM refused (G1)");

  /* Tokens */
  rigInit(&r, 19000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
  autoFsmOut_t o = doCmd(&r, cmdTakeoff(TOKEN ^ 1u, 30, 2, 150));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && o.cmdDetail == 0 && o.state == AUTO_ARMED, "wrong token: BAD_STATE, 0");
  o = doCmd(&r, cmdTakeoff(0, 30, 2, 150));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && o.state == AUTO_ARMED, "token 0: BAD_STATE");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_DISARM)).state == AUTO_IDLE, "disarm");
  run(&r, 100);
  r.in.randToken = 0x12345678u;
  o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(o.cmdResult == AUTO_R_OK && o.cmdDetail == 0x12345678u, "re-ARM: new token");
  o = doCmd(&r, cmdTakeoff(TOKEN, 30, 2, 150));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && o.state == AUTO_ARMED, "the previous ARM's token: BAD_STATE");
  /* expired: ARM timeout, then TAKEOFF */
  const long t = untilState(&r, AUTO_IDLE, 20000);
  CHECK(t == AUTO_ARM_TIMEOUT_MS - 1 && r.out.reason == AUTO_RS_ARM_TIMEOUT && r.out.lockAppThrust,
        "ARMED -> IDLE ARM_TIMEOUT 10 s after ARM (%ld + 1)", t);
  o = doCmd(&r, cmdTakeoff(0x12345678u, 30, 2, 150));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && o.state == AUTO_IDLE, "TAKEOFF with an expired token: BAD_STATE");
  /* randToken 0 never makes token 0 valid */
  rigInit(&r, 19000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  r.in.randToken = 0;
  o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(o.cmdResult == AUTO_R_OK && o.cmdDetail != 0, "randToken 0: a nonzero armToken (%u)", (unsigned)o.cmdDetail);
  CHECK(doCmd(&r, cmdTakeoff(0, 30, 2, 150)).cmdResult == AUTO_R_BAD_STATE, "and token 0 is refused");

  /* TAKEOFF parameter ranges */
  const struct { uint16_t h; uint8_t n; uint16_t gap; uint32_t wait; bool ok; } pr[] = {
    { 19, 2, 150, 0, false }, { 121, 2, 150, 0, false }, { 30, 4, 150, 0, false }, { 30, 2, 149, 0, false },
    { 30, 2, 1001, 0, false }, { 30, 2, 150, 1, false }, { 20, 3, 1000, 0, true }, { 40, 0, 150, 0, true },
    { 41, 1, 300, 0, true }, { 60, 1, 300, 0, true },   /* 0.20..0.60 m since 2026-10-12 (evening; 0.40 before) */
    { 61, 1, 300, 0, true }, { 120, 1, 300, 0, true },  /* 0.20..1.20 m: the interactive hover (late) */
    { 200, 1, 300, 0, false },   /* 2.00 m (2026-10-15, a2b 23.9) by MOVEs up only: the take-off stays <= 1.20 m */
    { 100, 1, 300, 999, false }, { 100, 1, 300, 1000, true }, { 100, 1, 300, 60000, true },
    { 100, 1, 300, 60001, true }, { 100, 1, 300, 65535, true }, { 100, 1, 300, 65536, true },
    { 100, 1, 300, 120000, true }, { 100, 1, 300, 120001, false }, { 100, 1, 300, 131072, false },
    { 100, 1, 300, 0xFFFFFFu, false },   /* actWaitMs: this sortie's wait, 0 or 1000..120000 (60000 until 2026-10-14) */
  };
  for (size_t i = 0; i < sizeof(pr) / sizeof(pr[0]); i++) {
    rigInit(&r, 19000);
    prepMode(&r, AUTO_MODE_FLIGHT);
    CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
    autoCmd_t c = cmdTakeoff(TOKEN, pr[i].h, pr[i].n, pr[i].gap);
    c.actWaitMs = pr[i].wait;
    o = doCmd(&r, c);
    if (pr[i].ok) {
      CHECK(o.cmdResult == AUTO_R_OK && o.state == AUTO_TAKEOFF, "TAKEOFF h %u n %u gap %u accepted", pr[i].h,
            pr[i].n, pr[i].gap);
      run(&r, AUTO_PRESPIN_MS);
      CHECK(fabsf(r.out.hTargetM - (float)pr[i].h / 100.0f) < 1e-6f, "hTargetM %.2f", r.out.hTargetM);
    } else {
      CHECK(o.cmdResult == AUTO_R_RANGE && o.state == AUTO_ARMED && o.cmdDetail == AUTO_ARMED,
            "TAKEOFF h %u n %u gap %u wait %u: RANGE", pr[i].h, pr[i].n, pr[i].gap, (unsigned)pr[i].wait);
    }
  }

  /* ARM ranges */
  const struct { uint8_t mode; uint16_t air, nvs; autoResult_t res; } ar[] = {
    { 3, 0, 4500, AUTO_R_RANGE }, { 0, 1999, 4500, AUTO_R_RANGE }, { 0, 4501, 4500, AUTO_R_RANGE },
    { 0, 2000, 4500, AUTO_R_OK }, { 0, 4500, 4500, AUTO_R_OK }, { 0, 6000, 6000, AUTO_R_OK },
    { 0, 6001, 6000, AUTO_R_RANGE }, { 0, 8000, 8000, AUTO_R_OK }, { 0, 8001, 9000, AUTO_R_RANGE },
    { 0, 0, 9000, AUTO_R_OK },
  };
  for (size_t i = 0; i < sizeof(ar) / sizeof(ar[0]); i++) {
    rigInit(&r, 19000);
    prepMode(&r, AUTO_MODE_FLIGHT);
    r.in.airMaxNvsMs = ar[i].nvs;
    o = doCmd(&r, cmdArm((autoMode_t)ar[i].mode, ar[i].air));
    CHECK(o.cmdResult == ar[i].res, "ARM mode %u airMax %u (NVS %u): result %d (got %d)", ar[i].mode, ar[i].air,
          ar[i].nvs, (int)ar[i].res, (int)o.cmdResult);
  }
}

/* ---- 10. airborne time ---------------------------------------------------- */

/* Air clock to LAND in ms; camera view never fresh so NOT_STABLE stays out
 * (the camera height estimate is healthy, so the hover is allowed). */
/* The pre-wait air clock (TAKEOFF_SLOW since 2026-10-15; LAND AIR_TIME until then): a hover that never reaches STABLE
 * 1 s nor its view (camViewMs 0: no NOT_STABLE) starts its wait at airMaxMs; returns that air time, *evt the event. */
static long airTimeTo(uint16_t airMaxMs, uint16_t nvsMs, bool *evt)
{
  rig_t r;
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  r.in.camViewMs = 0;
  r.in.airMaxNvsMs = nvsMs;
  if (!launch(&r, AUTO_MODE_FLIGHT, 2, airMaxMs)) {
    return -1;
  }
  while (!r.out.waiting && r.out.state != AUTO_LAND && r.steps < 20000) {
    rigStep(&r);
  }
  *evt = lastEvt(&r) == AUTO_EV_TAKEOFF_SLOW && r.out.state == AUTO_HOVER;
  return r.out.waiting ? (long)r.out.airMs : -1;
}

static bool toMoveReady(rig_t *r, float d, uint32_t waitMs);
static long untilMotionEnd(rig_t *r, uint32_t maxMs);
static autoCmd_t cmdMove(int16_t right, int16_t fwd, int16_t up);

/* No air clock ("Claude decides", 2026-10-15, docs/autonomy-a2b.md 22): the pre-wait limit starts the wait
 * (TAKEOFF_SLOW), the wait is information only (no HOVER_TIMEOUT, waitLeftMs 0), nothing lands at 120 s; the TAKEOFF
 * backstop (8 s + 25 ms per cm above 60) still lands a take-off that never reaches HOLD. */
static void testAirTime(void)
{
  bool evt = false;
  long a = airTimeTo(2000, 4500, &evt);
  CHECK(a == 2000 && evt, "ARM airMax 2000: TAKEOFF_SLOW, the wait at airMs %ld", a);
  a = airTimeTo(0, 4500, &evt);
  CHECK(a == 4500 && evt, "NVS 4500: TAKEOFF_SLOW at airMs %ld", a);
  a = airTimeTo(0, AUTO_AIR_MAX_DEFAULT_MS, &evt);
  CHECK(a == AUTO_AIR_MAX_DEFAULT_MS && evt, "NVS default %u: TAKEOFF_SLOW at airMs %ld",
        (unsigned)AUTO_AIR_MAX_DEFAULT_MS, a);
  a = airTimeTo(0, 9000, &evt);
  CHECK(a == AUTO_AIR_MAX_HI_MS && evt, "a broken NVS 9000 never exceeds %u (%ld)", (unsigned)AUTO_AIR_MAX_HI_MS, a);
  a = airTimeTo(0, 0, &evt);
  CHECK(a == AUTO_AIR_MAX_MIN_MS && evt, "a broken NVS 0 gives the 2000 minimum (%ld)", a);

  /* the air clock from the first RAMP step: a lift-off detected 2 s into the ramp starts the wait 6 s after it */
  rig_t r;
  rigInit(&r, 21000);
  r.vz.neverSteady = true;
  r.vz.rampMs = 2000;
  r.in.camViewMs = 0;
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  uint32_t airAtLift = 0;
  while (!r.out.waiting && r.out.state != AUTO_LAND && r.steps < 20000) {
    rigStep(&r);
    if (r.vz.o.liftoffNow) {
      airAtLift = r.out.airMs;
    }
  }
  CHECK(r.out.waiting && r.out.state == AUTO_HOVER && r.out.airMs == AUTO_AIR_MAX_DEFAULT_MS && airAtLift >= 1999,
        "late lift-off: the wait at airMs %u (TAKEOFF_SLOW), no LAND", (unsigned)r.out.airMs);

  /* a take-off that never lifts off: the TAKEOFF backstop (not the air clock) lands it */
  rigInit(&r, 21000);
  r.vz.rampMs = 60000;
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  long t = untilState(&r, AUTO_LAND, 12000);
  CHECK(t == AUTO_TAKEOFF_MAX_MS && r.out.reason == AUTO_RS_TAKEOFF_TIMEOUT,
        "no lift-off: LAND TAKEOFF_TIMEOUT at stateMs %ld (air clock %u: no AIR_TIME)", t, (unsigned)r.out.airMs);
  CHECK(toIdle(&r, 5000) && r.out.reason == AUTO_RS_TOUCHDOWN && r.out.vzLandReason == VZ_R_ABORTED,
        "abort ends DONE: IDLE TOUCHDOWN (vz %d)", (int)r.out.vzLandReason);
  /* a controller that never reaches RAMP: TAKEOFF_TIMEOUT 8 s after TAKEOFF */
  rigInit(&r, 21000);
  r.vz.spinMs = 60000;
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  t = untilState(&r, AUTO_LAND, 10000);
  CHECK(t == AUTO_TAKEOFF_MAX_MS && r.out.reason == AUTO_RS_TAKEOFF_TIMEOUT && r.out.airMs == 0,
        "stuck in SPIN: LAND TAKEOFF_TIMEOUT at stateMs 8000 (%ld)", t);

  /* the wait: no countdown, no HOVER_TIMEOUT (NVS 60 s), nothing at 120 s (the air clock until 2026-10-15) */
  rigInit(&r, 21000);
  r.in.hoverWaitNvsMs = 60000;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0), "hover");
  bool zero = true;
  while (r.out.state == AUTO_HOVER && r.steps < 160000) {
    rigStep(&r);
    zero = zero && r.out.waitLeftMs == 0;
  }
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting && zero && r.out.airMs > 150000,
        "a 60 s NVS wait, no command: HOVER at airMs %u, waitLeftMs always 0 (no HOVER_TIMEOUT, no AIR_TIME)",
        (unsigned)r.out.airMs);
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND && toIdle(&r, 6000) && r.out.reason == AUTO_RS_TOUCHDOWN,
        "Claude's LAND ends it");
  /* TAKEOFF's 66000 ms wait: accepted (the wire range), information only */
  rigInit(&r, 21000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
  autoCmd_t tc = cmdTakeoff(TOKEN, 100, 2, 150);
  tc.actWaitMs = 66000;
  CHECK(doCmd(&r, tc).cmdResult == AUTO_R_OK, "TAKEOFF with a 66000 ms wait: OK");
  while (!r.out.waiting && (r.out.state == AUTO_TAKEOFF || r.out.state == AUTO_HOVER)) {
    rigStep(&r);
  }
  long tw = untilState(&r, AUTO_LAND, 70000);
  CHECK(r.out.waiting && tw < 0 && r.out.state == AUTO_HOVER, "70 s into the 66 s wait: still HOVER");
  /* MOVE and STOP in a long wait: accepted */
  rigInit(&r, 54000);
  CHECK(toMoveReady(&r, 2.0f, 100000), "waiting");
  run(&r, 40000);
  autoFsmOut_t ow = doCmd(&r, cmdOf(AUTO_CMD_STOP));
  CHECK(ow.cmdResult == AUTO_R_OK && ow.state == AUTO_HOVER, "STOP 40 s in: OK");
  run(&r, 10000);
  ow = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(ow.cmdResult == AUTO_R_OK, "a MOVE 50 s in: OK");
  CHECK(untilMotionEnd(&r, 20000) > 0 && r.out.state == AUTO_HOVER, "it ends, HOVER");

  /* 1.20 m (the take-off cap): the pre-wait limit and the TAKEOFF backstop grow by 25 ms per cm above 60 (+1500 ms);
   * 0.60 m keeps them */
  for (int k = 0; k < 2; k++) {
    const uint16_t hcm = k == 0 ? 60 : 120;
    const uint32_t extra = hcm > 60 ? (uint32_t)(hcm - 60) * AUTO_AIR_PER_CM_ABOVE_60_MS : 0u;
    rigInit(&r, 21000);
    r.vz.climbMs = 60000;   /* never reaches HOLD */
    prepMode(&r, AUTO_MODE_FLIGHT);
    autoFsmOut_t ao = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
    run(&r, 50);
    ao = doCmd(&r, cmdTakeoff(ao.cmdDetail, hcm, 1, 300));
    CHECK(ao.cmdResult == AUTO_R_OK, "TAKEOFF %u cm", hcm);
    t = untilState(&r, AUTO_LAND, 20000);
    CHECK(t == (long)(AUTO_TAKEOFF_MAX_MS + extra) && r.out.reason == AUTO_RS_TAKEOFF_TIMEOUT,
          "%u cm, stuck in CLIMB: LAND TAKEOFF_TIMEOUT at stateMs %ld (no AIR_TIME)", hcm, t);
    rigInit(&r, 21000);
    r.vz.neverSteady = true;
    r.in.camViewMs = 0;
    prepMode(&r, AUTO_MODE_FLIGHT);
    ao = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
    run(&r, 50);
    ao = doCmd(&r, cmdTakeoff(ao.cmdDetail, hcm, 1, 300));
    while (!r.out.waiting && r.out.state != AUTO_LAND && r.steps < 20000) {
      rigStep(&r);
    }
    CHECK(r.out.waiting && r.out.airMs == AUTO_AIR_MAX_DEFAULT_MS + extra,
          "%u cm, never STABLE: TAKEOFF_SLOW at air %u ms", hcm, (unsigned)r.out.airMs);
  }
}

/* ---- 11. battery, loop gap ------------------------------------------------ */

static void testBattery(void)
{
  /* VBAT_LOW (10 Hz below AUTO_VBAT_LAND_V for AUTO_VBAT_LAND_MS) in TAKEOFF, the controller in CLIMB: there
   * BATT_LOW does not apply (HOVER only), so VBAT_LOW is the take-off's guard. */
  rig_t r;
  rigInit(&r, 23000);
  r.vz.climbMs = 60000;   /* stays in CLIMB */
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 5000), "launched");
  for (int i = 0; i < 6000 && r.vz.o.phase != VZ_CLIMB; i++) {
    rigStep(&r);
  }
  CHECK(r.out.state == AUTO_TAKEOFF && r.vz.o.phase == VZ_CLIMB, "TAKEOFF, the controller in CLIMB");
  for (int i = 0; i < AUTO_VBAT_LAND_MS; i++) {   /* held 999 ms */
    r.in.vbat = AUTO_VBAT_LAND_V - 0.01f;
    rigStep(&r);
  }
  r.in.vbat = (AUTO_VBAT_LOW_LAND_V + 0.01f);   /* above both (AUTO_VBAT_LAND_V and the HOVER AUTO_VBAT_LOW_LAND_V) */
  run(&r, 200);
  CHECK(r.out.state == AUTO_TAKEOFF, "below VBAT_LOW for 999 ms in TAKEOFF, then above both: nothing");
  int s = 0;
  r.in.vbat = AUTO_VBAT_LAND_V - 0.01f;
  while (r.out.state == AUTO_TAKEOFF && s < 2000) {
    rigStep(&r);
    s++;
  }
  CHECK(s == AUTO_VBAT_LAND_MS + 1 && r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_VBAT_LOW,
        "below VBAT_LOW held 1000 ms in TAKEOFF (CLIMB): LAND VBAT_LOW (step %d)", s);
  /* In HOVER the same reading lands BATT_LOW at AUTO_VBAT_LOW_LAND_MS, before VBAT_LOW's AUTO_VBAT_LAND_MS. */
  rigInit(&r, 23000);
  r.vz.neverSteady = true;
  r.in.camViewMs = 0;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  s = 0;
  r.in.vbat = AUTO_VBAT_LAND_V - 0.01f;
  while (r.out.state == AUTO_HOVER && s < 2000) {
    rigStep(&r);
    s++;
  }
  CHECK(s == AUTO_VBAT_LOW_LAND_MS + 1 && r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_BATT_LOW &&
            r.out.thrust == AUTO_THR_VERT_LAND,
        "%.2f V in HOVER: LAND BATT_LOW (slow) at step %d, before VBAT_LOW at %d", AUTO_VBAT_LAND_V - 0.01f, s,
        AUTO_VBAT_LAND_MS + 1);

  /* Battery low in HOVER (interactive hover): below AUTO_VBAT_LOW_LAND_V for AUTO_VBAT_LOW_LAND_MS. */
  rigInit(&r, 23000);
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 5000), "launched (at rest 4.05 V)");
  r.in.vbat = (AUTO_VBAT_LOW_LAND_V - 0.01f);   /* through the whole take-off: TAKEOFF does not land BATT_LOW */
  CHECK(untilState(&r, AUTO_HOVER, 6000) > 0, "hover at %.2f V (TAKEOFF ignores the AUTO_VBAT_LOW_LAND_V rule)",
        AUTO_VBAT_LOW_LAND_V - 0.01f);
  r.in.vbat = (AUTO_VBAT_LOW_LAND_V + 0.01f);
  run(&r, 50);
  for (int i = 0; i < AUTO_VBAT_LOW_LAND_MS; i++) {
    r.in.vbat = i % 2 ? (AUTO_VBAT_LOW_LAND_V - 0.01f) : (AUTO_VBAT_LOW_LAND_V - 0.10f);
    rigStep(&r);
  }
  r.in.vbat = AUTO_VBAT_LOW_LAND_V;   /* not below: the streak ends */
  rigStep(&r);
  r.in.vbat = (AUTO_VBAT_LOW_LAND_V - 0.01f);
  run(&r, AUTO_VBAT_LOW_LAND_MS - 1);
  CHECK(r.out.state == AUTO_HOVER && r.out.battLowNow && r.out.battLowHeld,
        "just below it for %u ms, one AUTO_VBAT_LOW_LAND_V, %u ms more: nothing (low now, held %u ms)",
        AUTO_VBAT_LOW_LAND_MS - 1, AUTO_VBAT_LOW_LAND_MS - 1, AUTO_VBAT_LOW_REFUSE_MS);
  s = 0;
  r.in.vbat = (AUTO_VBAT_LOW_LAND_V - 0.05f);
  rigStep(&r);
  run(&r, AUTO_VBAT_LOW_LAND_MS);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_BATT_LOW && r.out.thrust == AUTO_THR_VERT_LAND,
        "below AUTO_VBAT_LOW_LAND_V for %u ms in HOVER: LAND BATT_LOW (slow)", AUTO_VBAT_LOW_LAND_MS);
  /* AUTO_STATUS battLow: one reading below is 'now', AUTO_VBAT_LOW_REFUSE_MS of them 'held' (the Mac refuses on
   * held only) */
  rigInit(&r, 23000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  r.in.vbat = (AUTO_VBAT_LOW_LAND_V - 0.01f);
  rigStep(&r);
  CHECK(r.out.battLowNow && !r.out.battLowHeld, "one reading below AUTO_VBAT_LOW_LAND_V: now, not held");
  run(&r, AUTO_VBAT_LOW_REFUSE_MS - 2);
  CHECK(r.out.battLowNow && !r.out.battLowHeld, "%u ms: not held", AUTO_VBAT_LOW_REFUSE_MS - 1);
  run(&r, 2);
  CHECK(r.out.battLowHeld && r.out.state == AUTO_HOVER, "%u ms: held (MOVE / YAW refused), still hovering",
        AUTO_VBAT_LOW_REFUSE_MS);
  r.in.vbat = (AUTO_VBAT_LOW_LAND_V + 0.01f);
  rigStep(&r);
  CHECK(!r.out.battLowNow && !r.out.battLowHeld, "%.2f V: neither", AUTO_VBAT_LOW_LAND_V + 0.01f);
  /* a loaded battery at the logged 15 s sortie's levels never trips it */
  rigInit(&r, 23000);
  r.in.hoverWaitNvsMs = 15000;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  uint32_t k15 = 0;
  while (r.out.state == AUTO_HOVER && k15 < 16000) {
    /* mission60's 10 Hz readings at 8-15 s of air: 3.37-3.53 V, mean 3.46 */
    if (k15 % 100 == 0) {
      r.in.vbat = 3.37f + 0.16f * (float)((k15 * 7919u) % 101u) / 100.0f;
    }
    rigStep(&r);
    k15++;
  }
  CHECK(r.out.state == AUTO_HOVER && !r.out.battLowHeld, "mission60's loaded readings for 16 s: no BATT_LOW (%s)",
        autoStateName(r.out.state));

  const struct { float v; bool land; } fm[] = { { 2.55f, false }, { 2.62f, false }, { 0.0f, false }, { 2.54f, true } };
  for (int i = 0; i < 4; i++) {
    rigInit(&r, 23000);
    r.vz.neverSteady = true;
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
    r.in.vbatFastMin = fm[i].v;
    rigStep(&r);
    CHECK((r.out.state == AUTO_LAND) == fm[i].land && (!fm[i].land || r.out.reason == AUTO_RS_VBAT_DIP),
          "1 kHz minimum %.2f V: %s", fm[i].v, fm[i].land ? "LAND VBAT_DIP" : "nothing");
  }

  rigInit(&r, 23000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  r.in.loopGapUs = AUTO_LOOP_GAP_LAND_US;
  rigStep(&r);
  CHECK(r.out.state == AUTO_HOVER && autoFsmReport(&r.f)->loopGaps == 0, "loop gap 20 ms: nothing");
  /* one or two gaps: the event (the ms), a running MOVE stopped; the third within 1 s: LAND LOOP_GAP
   * (2026-10-15; one gap landed until then) */
  r.in.loopGapUs = 25400;
  rigStep(&r);
  r.in.loopGapUs = 1000;
  CHECK(r.out.state == AUTO_HOVER && lastEvt(&r) == AUTO_EV_LOOP_GAP && lastEvtDetail(&r) == 25 &&
        autoFsmReport(&r.f)->loopGaps == 1, "loop gap 25.4 ms: the event (25 ms), HOVER");
  run(&r, 600);
  r.in.loopGapUs = AUTO_LOOP_GAP_LAND_US + 1;
  rigStep(&r);
  r.in.loopGapUs = 1000;
  CHECK(r.out.state == AUTO_HOVER, "a second gap 600 ms later: HOVER");
  run(&r, 600);
  r.in.loopGapUs = 60000;
  rigStep(&r);
  r.in.loopGapUs = 1000;
  CHECK(r.out.state == AUTO_HOVER && lastEvtDetail(&r) == 60, "a third 1.2 s after the first (two within 1 s): HOVER");
  run(&r, 300);
  r.in.loopGapUs = 300000;
  rigStep(&r);
  r.in.loopGapUs = 1000;
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_LOOP_GAP && lastEvtDetail(&r) == 255,
        "a fourth 0.3 s later (three within 0.9 s): LAND LOOP_GAP (detail saturated 255)");

  rigInit(&r, 23000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  r.in.sessionChanged = true;
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_SESSION, "new Mac session airborne: LAND SESSION");
}

/* ---- 11b. end of battery (user decision 2026-10-13, late) ------------------
 * BATT_LOW has two triggers on one persistence tracker, either one held for
 * AUTO_VBAT_LOW_LAND_MS in HOVER: the 10 Hz battery below AUTO_VBAT_LOW_LAND_V
 * (3.00 V) or the controller's T_hold above AUTO_HEADROOM_LAND_T (57000).
 * MOVE / YAW are refused above a T_hold of AUTO_MV_REFUSE_T (53400) or once
 * the condition has held AUTO_VBAT_LOW_REFUSE_MS. The fake controller's
 * output reaches the FSM one step later: a holdThrust set before step k is
 * read at step k + 1. */

static autoCmd_t cmdMove(int16_t right, int16_t fwd, int16_t up);
static autoCmd_t cmdYaw(int16_t deg);
static uint8_t mvWhyOf(const autoFsmOut_t *o);
static bool toMoveReady(rig_t *r, float d, uint32_t waitMs);
static long untilMotionEnd(rig_t *r, uint32_t maxMs);
static autoCmd_t cmdPhoto(uint8_t n, uint16_t gapMs);

/* Steps until the state leaves s; returns the steps taken (the last is the transition step) or -1. */
static long whileState(rig_t *r, autoState_t s, uint32_t maxMs)
{
  for (uint32_t i = 1; i <= maxMs; i++) {
    rigStep(r);
    if (r->out.state != s) {
      return (long)i;
    }
  }
  return -1;
}

static void testEndOfBattery(void)
{
  CHECK(VZ_THRUST_ABS_MAX == 58000.0f && AUTO_HEADROOM_LAND_T == 57000.0f && AUTO_MV_REFUSE_T == 53400.0f &&
            AUTO_VBAT_LOW_LAND_V == 3.00f && AUTO_VBAT_LOW_LAND_MS == 500 && AUTO_VBAT_LOW_REFUSE_MS == 250,
        "the numbers of the user decision 2026-10-13 (late): 58000 / 57000 / 53400, 3.00 V, 500 / 250 ms");
  CHECK(AUTO_MV_REFUSE_T < AUTO_HEADROOM_LAND_T && AUTO_HEADROOM_LAND_T < VZ_THRUST_ABS_MAX &&
            AUTO_VBAT_LOW_LAND_V > AUTO_VBAT_LAND_V && AUTO_VBAT_LAND_V > AUTO_VBAT_FAST_LAND_V &&
            AUTO_VBAT_LOW_REFUSE_MS < AUTO_VBAT_LOW_LAND_MS && AUTO_VBAT_LOW_LAND_MS < AUTO_VBAT_LAND_MS,
        "the order: refuse < headroom < ceiling; BATT_LOW > VBAT_LOW > VBAT_DIP; refuse 250 < BATT_LOW 500 < 1000 ms");

  /* (a) headroom: T_hold above 57000 for 500 ms in HOVER */
  rig_t r;
  rigInit(&r, 61000);
  r.in.hoverWaitNvsMs = 30000;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting (T_hold 36000)");
  CHECK(!r.out.battLowNow && !r.out.battLowHeld, "36000: no battLow");
  r.vz.holdThrust = AUTO_HEADROOM_LAND_T;   /* exactly 57000: not above */
  run(&r, 5000);
  CHECK(r.out.state == AUTO_HOVER && !r.out.battLowNow, "T_hold exactly %.0f for 5 s: HOVER, no battLow",
        AUTO_HEADROOM_LAND_T);
  r.vz.holdThrust = AUTO_HEADROOM_LAND_T + 1.0f;
  rigStep(&r);                              /* the FSM reads 57001 from the next step on */
  run(&r, 400);
  CHECK(r.out.state == AUTO_HOVER && r.out.battLowNow && r.out.battLowHeld,
        "57001 for 400 ms: HOVER, battLow now and held (0.25 s)");
  r.vz.holdThrust = AUTO_HEADROOM_LAND_T - 1.0f;
  rigStep(&r);                              /* reads 57001 (401 ms) */
  r.vz.holdThrust = AUTO_HEADROOM_LAND_T + 1.0f;
  rigStep(&r);                              /* reads 56999: the streak ends */
  CHECK(r.out.state == AUTO_HOVER && !r.out.battLowNow && !r.out.battLowHeld, "one step at 56999: the streak ends");
  long s = whileState(&r, AUTO_HOVER, 2000);
  CHECK(s == AUTO_VBAT_LOW_LAND_MS + 1 && r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_BATT_LOW &&
            r.out.vzCmd == VZ_CMD_LAND && r.out.thrust == AUTO_THR_VERT_LAND,
        "57001 again: HOVER through %u ms of it, LAND BATT_LOW at %u ms (step %ld), vzCmd LAND, VERT_LAND",
        AUTO_VBAT_LOW_LAND_MS - 1, AUTO_VBAT_LOW_LAND_MS, s);
  /* the same T_hold in TAKEOFF (the controller in CLIMB; the real one reports T_hold 0 before HOLD): no BATT_LOW */
  rigInit(&r, 61000);
  r.vz.climbMs = 60000;
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 5000), "launched");
  for (int i = 0; i < 6000 && r.vz.o.phase != VZ_CLIMB; i++) {
    rigStep(&r);
  }
  r.vz.o.tHold = AUTO_HEADROOM_LAND_T + 1.0f;   /* CLIMB keeps it */
  run(&r, 2000);
  CHECK(r.out.state == AUTO_TAKEOFF && r.vz.o.phase == VZ_CLIMB,
        "T_hold 57001 for 2 s in TAKEOFF (CLIMB): no BATT_LOW (HOVER only; %s)", autoReasonName(r.out.reason));

  /* (b) the floor: 3.00 V exactly never; 2.99 / 3.00 alternating at 10 Hz never; 2.99 V for 500 ms */
  rigInit(&r, 62000);
  r.in.hoverWaitNvsMs = 30000;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  r.in.vbat = AUTO_VBAT_LOW_LAND_V;
  run(&r, 3000);
  CHECK(r.out.state == AUTO_HOVER && !r.out.battLowNow, "%.2f V for 3 s: HOVER, no battLow", AUTO_VBAT_LOW_LAND_V);
  for (int i = 0; i < 3000; i++) {
    r.in.vbat = (i / 100) % 2 ? AUTO_VBAT_LOW_LAND_V : AUTO_VBAT_LOW_LAND_V - 0.01f;
    rigStep(&r);
  }
  CHECK(r.out.state == AUTO_HOVER, "2.99 / 3.00 V alternating every 100 ms for 3 s: HOVER");
  r.in.vbat = AUTO_VBAT_LOW_LAND_V;
  rigStep(&r);
  r.in.vbat = AUTO_VBAT_LOW_LAND_V - 0.01f;
  s = whileState(&r, AUTO_HOVER, 2000);
  CHECK(s == AUTO_VBAT_LOW_LAND_MS + 1 && r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_BATT_LOW &&
            r.out.vzCmd == VZ_CMD_LAND && r.out.thrust == AUTO_THR_VERT_LAND,
        "2.99 V: LAND BATT_LOW at %u ms (step %ld)", AUTO_VBAT_LOW_LAND_MS, s);

  /* (c) refusals in the wait */
  autoFsmOut_t o0;
  rigInit(&r, 63000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting, D 2.0 m");
  o0 = doCmd(&r, cmdMove(0, 0, 50));   /* up first: room for the moves down below (the floor 0.25 m + the drift term) */
  CHECK(o0.cmdResult == AUTO_R_OK && untilMotionEnd(&r, 10000) > 0 && r.out.mv.state == AUTO_MV_DONE, "up 50: DONE");
  r.vz.holdThrust = AUTO_MV_REFUSE_T + 1.0f;
  rigStep(&r);
  autoFsmOut_t o = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_BATT && o.state == AUTO_HOVER &&
            !o.battLowNow && !o.battLowHeld,
        "T_hold 53401: MOVE LOW_BATT (BATT), the battLow bits clear");
  o = doCmd(&r, cmdYaw(30));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_BATT, "T_hold 53401: YAW LOW_BATT (BATT)");
  /* a vertical MOVE down too (2026-10-15 review: a descent accepted at T_hold 54600-56000 ended HOLD_CAPPED, a fast
   * LAND, in the drain sim; the runbook comes down from 52000) */
  o = doCmd(&r, cmdMove(0, 0, 10));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_UP_HEADROOM && o.mv.why == AUTO_MVW_UP_HEADROOM &&
            o.mv.state != AUTO_MV_RUN,
        "T_hold 53401: MOVE up 10 LOW_BATT, named UP_HEADROOM (the 58000 ceiling minus 4600, 2026-10-16)");
  o = doCmd(&r, cmdMove(0, 0, -10));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_BATT && o.mv.state != AUTO_MV_RUN,
        "T_hold 53401: MOVE down 10 LOW_BATT (BATT)");
  o = doCmd(&r, cmdOf(AUTO_CMD_STOP));
  CHECK(o.cmdResult == AUTO_R_OK && o.state == AUTO_HOVER, "T_hold 53401: STOP OK");
  o = doCmd(&r, cmdPhoto(1, 150));
  CHECK(o.cmdResult == AUTO_R_OK && o.captureStart, "T_hold 53401: PHOTO OK");
  run(&r, 300);
  r.vz.holdThrust = AUTO_MV_REFUSE_T;
  rigStep(&r);
  o = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.state == AUTO_MV_RUN, "T_hold exactly 53400: MOVE accepted");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_STOP)).cmdResult == AUTO_R_OK && untilMotionEnd(&r, 3000) > 0, "STOP");
  o = doCmd(&r, cmdYaw(30));
  CHECK(o.cmdResult == AUTO_R_OK, "T_hold exactly 53400: YAW accepted");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_STOP)).cmdResult == AUTO_R_OK && untilMotionEnd(&r, 3000) > 0, "STOP");
  run(&r, 100);
  /* a battery streak: 249 ms accepts, 250 ms refuses */
  r.in.vbat = AUTO_VBAT_LOW_LAND_V - 0.01f;
  run(&r, AUTO_VBAT_LOW_REFUSE_MS - 1);     /* ages 0 .. 248 */
  o = doCmd(&r, cmdMove(20, 0, 0));         /* age 249 */
  CHECK(o.cmdResult == AUTO_R_OK && !o.battLowHeld, "battery below 3.00 V for %u ms: MOVE accepted",
        AUTO_VBAT_LOW_REFUSE_MS - 1);
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_STOP)).cmdResult == AUTO_R_OK && untilMotionEnd(&r, 3000) > 0, "STOP");
  r.in.vbat = 3.80f;
  run(&r, 100);
  r.in.vbat = AUTO_VBAT_LOW_LAND_V - 0.01f;
  run(&r, AUTO_VBAT_LOW_REFUSE_MS);         /* ages 0 .. 249 */
  o = doCmd(&r, cmdMove(20, 0, 0));         /* age 250 */
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_BATT && o.battLowHeld && o.state == AUTO_HOVER,
        "battery below 3.00 V for %u ms: MOVE LOW_BATT (BATT), battLow held", AUTO_VBAT_LOW_REFUSE_MS);
  o = doCmd(&r, cmdMove(0, 0, -10));        /* age 251 */
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_BATT, "battLow held: MOVE down 10 LOW_BATT (BATT)");
  s = whileState(&r, AUTO_HOVER, 1000);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_BATT_LOW &&
            (uint32_t)s + AUTO_VBAT_LOW_REFUSE_MS + 2 == AUTO_VBAT_LOW_LAND_MS + 1,
        "and LAND BATT_LOW at %u ms (step %ld after the second refusal)", AUTO_VBAT_LOW_LAND_MS, s);

  /* (d) unchanged: VBAT_DIP in TAKEOFF and HOVER, the controller's HOLD_CAPPED as LAND VZ */
  rigInit(&r, 64000);
  r.vz.climbMs = 60000;
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 5000), "launched");
  for (int i = 0; i < 6000 && r.vz.o.phase != VZ_CLIMB; i++) {
    rigStep(&r);
  }
  r.in.vbatFastMin = AUTO_VBAT_FAST_LAND_V - 0.01f;
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_VBAT_DIP, "1 kHz minimum 2.54 V in TAKEOFF: LAND VBAT_DIP");
  rigInit(&r, 64000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  r.in.vbatFastMin = AUTO_VBAT_FAST_LAND_V - 0.01f;
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_VBAT_DIP, "1 kHz minimum 2.54 V in HOVER: LAND VBAT_DIP");
  rigInit(&r, 64000);
  r.in.hoverWaitNvsMs = 30000;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  r.vz.holdThrust = VZ_THRUST_ABS_MAX;
  run(&r, 100);
  r.vz.o.phase = VZ_LAND;
  r.vz.o.reason = VZ_R_HOLD_CAPPED;
  r.vz.o.landReason = VZ_R_HOLD_CAPPED;
  r.vz.t = 0;
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_VZ && r.out.vzLandReason == VZ_R_HOLD_CAPPED,
        "the controller's HOLD_CAPPED: LAND VZ (HOLD_CAPPED), as before");
}


/* Steps until the event e is the latest (or the state leaves HOVER); returns the steps, -1 after maxMs. */
static long untilEvt(rig_t *r, autoEvent_t e, uint32_t maxMs)
{
  for (uint32_t i = 1; i <= maxMs; i++) {
    rigStep(r);
    if (lastEvt(r) == e || r->out.state != AUTO_HOVER) {
      return lastEvt(r) == e ? (long)i : -1;
    }
  }
  return -1;
}

/* NOT_STABLE (row 24): the event at the old LAND's time, the wait starts, no LAND (2026-10-15). */
static void testNotStable(void)
{
  rig_t r;
  rigInit(&r, 25000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "warm: hover");
  long t = untilEvt(&r, AUTO_EV_NOT_STABLE, 4000);
  CHECK(t == AUTO_NOT_STABLE_HOLD_MS && r.out.state == AUTO_HOVER && r.out.waiting &&
            (lastEvtDetail(&r) & AUTO_NS_VZ) && (autoFsmReport(&r.f)->holdBits & AUTO_HB_NOT_STABLE),
        "warm (camera view since before HOLD): NOT_STABLE at HOLD + %ld ms, the wait starts (detail 0x%x)", t,
        (unsigned)lastEvtDetail(&r));
  CHECK(untilState(&r, AUTO_LAND, 10000) < 0 && r.out.waiting, "10 s more: still HOVER, waiting");
  autoFsmOut_t mo = doCmd(&r, cmdOf(AUTO_CMD_PHOTO));
  r.in.cmd = cmdOf(AUTO_CMD_YAW);
  r.in.cmd.yawDeg = 30;
  rigStep(&r);
  CHECK(r.out.cmdResult == AUTO_R_OK, "after NOT_STABLE a YAW is accepted (the wait started; %s)",
        r.out.cmdDone ? "done" : "?");
  (void)mo;

  rigInit(&r, 25000);
  r.vz.neverSteady = true;
  r.in.camViewMs = 0;
  r.in.camState = CAM_STANDBY;   /* camera asleep from the take-off window */
  r.in.camFps = 0.0f;
  prepMode(&r, AUTO_MODE_FLIGHT);
  r.in.airMaxNvsMs = 5000;
  autoFsmOut_t o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 5000));
  CHECK(o.cmdResult == AUTO_R_GUARDS && o.cmdDetail == AUTO_G_CAMERA, "(G4 refuses ARM with the camera asleep)");
  r.in.camState = CAM_RUNNING;
  r.in.camFps = 16.9f;
  r.in.airMaxNvsMs = 8000;
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 8000), "cold: launched");
  r.in.camState = CAM_STANDBY;
  CHECK(untilState(&r, AUTO_HOVER, 6000) > 0, "cold: hover");
  run(&r, 999);
  r.in.camState = CAM_RUNNING;
  r.camViewCounting = true;   /* the step at HOLD + 1000 sees camViewMs 0: fresh view from there */
  t = untilEvt(&r, AUTO_EV_NOT_STABLE, 4000);
  CHECK(t + 999 == 2500 && r.out.waiting, "cold (view from HOLD + 1 s): NOT_STABLE at HOLD + %ld", t + 999);

  rigInit(&r, 25000);
  r.vz.neverSteady = true;
  r.in.camViewMs = 0;
  CHECK(toHover(&r, AUTO_MODE_DRY, 2, 5000), "DRY: hover");
  t = untilEvt(&r, AUTO_EV_NOT_STABLE, 4000);
  CHECK(t == AUTO_NOT_STABLE_HOLD_MS && r.out.waiting, "DRY waives the camera: HOLD + %ld", t);
}

/* STABLE scenarios (section 3.5). A break function sets one input (on) or
 * restores it (off). */
typedef void (*breakFn)(rig_t *r, bool on);
static void bFrame(rig_t *r, bool on) { r->in.camFrameAgeMs = on ? AUTO_STABLE_FRAME_AGE_MS + 1 : 30; }
static void bCoast(rig_t *r, bool on) { r->in.holdCoasting = on; }
static void bApplied(rig_t *r, bool on) { r->in.holdApplied = !on; }
static void bSteady(rig_t *r, bool on) { r->vz.neverSteady = on; }   /* the FSM sees it one step later */
static void bRoll(rig_t *r, bool on) { r->in.rollDeg = on ? 4.1f : 0.4f; }
static void bPitch(rig_t *r, bool on) { r->in.pitchDeg = on ? -4.1f : -0.3f; }
static void bGx(rig_t *r, bool on) { r->in.gyroDps[0] = on ? 30.5f : 0.3f; }
static void bGy(rig_t *r, bool on) { r->in.gyroDps[1] = on ? -30.5f : -0.2f; }
static void bGz(rig_t *r, bool on) { r->in.gyroDps[2] = on ? 10.5f : 0.1f; }
/* just inside the hard bounds: still soft */
static void sRoll(rig_t *r, bool on) { r->in.rollDeg = on ? AUTO_STABLE_HARD_TILT_DEG - 0.1f : 0.4f; }
static void sGx(rig_t *r, bool on) { r->in.gyroDps[0] = on ? AUTO_STABLE_HARD_GXY_DPS - 0.5f : 0.3f; }
static void sGz(rig_t *r, bool on) { r->in.gyroDps[2] = on ? -(AUTO_STABLE_HARD_GZ_DPS - 0.5f) : 0.1f; }
/* beyond a hard bound */
static void hRoll(rig_t *r, bool on) { r->in.rollDeg = on ? AUTO_STABLE_HARD_TILT_DEG + 0.1f : 0.4f; }
static void hPitch(rig_t *r, bool on) { r->in.pitchDeg = on ? -(AUTO_STABLE_HARD_TILT_DEG + 0.1f) : -0.3f; }
static void hGx(rig_t *r, bool on) { r->in.gyroDps[0] = on ? -(AUTO_STABLE_HARD_GXY_DPS + 0.5f) : 0.3f; }
static void hGy(rig_t *r, bool on) { r->in.gyroDps[1] = on ? AUTO_STABLE_HARD_GXY_DPS + 0.5f : -0.2f; }
static void hGz(rig_t *r, bool on) { r->in.gyroDps[2] = on ? AUTO_STABLE_HARD_GZ_DPS + 0.5f : 0.1f; }
static void hCam(rig_t *r, bool on) { r->in.camState = on ? CAM_STANDBY : CAM_RUNNING; }
static void hVz(rig_t *r, bool on) { r->vzIdle = on; }

typedef struct {
  bool hovered;        /* reached HOVER and a first STABLE step S */
  long capAt;          /* captureStart step - S; -1: none within 3000 steps */
  uint32_t capStableMs;
  long brokeAt;        /* first step - S without a streak (stableMs 0); -1: never before the capture */
  long failSteps;      /* steps with stable false from S to the break (or the capture) */
  long restartAt;      /* the next streak's first step - S; -1: none */
  bool countedOn;      /* stableMs == step - S on every step before the break */
} stabRes_t;

typedef struct {
  long at, dur;        /* broken on steps S + at + 1 .. S + at + dur */
} drop_t;

/* Hover, find the streak's first step S (stable, stableMs 0), and step on
 * to the capture (photoN 2), breaking with fn during each of the n
 * dropouts and restoring between them. */
static stabRes_t stableDrops(autoMode_t mode, breakFn fn, const drop_t *d, int n)
{
  stabRes_t s = { false, -1, 0, -1, 0, -1, true };
  rig_t r;
  rigInit(&r, 27000);
  if (!toHover(&r, mode, 2, 5000) || !untilStable(&r, 3000) || r.out.stableMs != 0) {
    return s;
  }
  s.hovered = true;
  for (long k = 1; k <= 3000 && r.out.state == AUTO_HOVER; k++) {
    bool bad = false;
    for (int i = 0; i < n; i++) {
      bad = bad || (k > d[i].at && k <= d[i].at + d[i].dur);
    }
    fn(&r, bad);
    rigStep(&r);
    if (s.brokeAt < 0) {
      s.failSteps += !r.out.stable;
      if (r.out.stableMs == 0) {
        s.brokeAt = k;
      } else if (r.out.stableMs != (uint32_t)k) {
        s.countedOn = false;
      }
    } else if (s.restartAt < 0 && r.out.stable && r.out.stableMs == 0) {
      s.restartAt = k;
    }
    if (r.out.captureStart) {
      s.capAt = k;
      s.capStableMs = r.out.stableMs;
      break;
    }
  }
  return s;
}

/* One dropout of dur steps after `at`. */
static stabRes_t stableCase(autoMode_t mode, breakFn fn, long at, long dur)
{
  const drop_t d = { at, dur };
  return stableDrops(mode, fn, &d, 1);
}

static const char *modeName(autoMode_t m)
{
  return m == AUTO_MODE_FLIGHT ? "FLIGHT" : (m == AUTO_MODE_DRY ? "DRY" : "BENCH");
}

static void testStableGrace(void)
{
  const struct { const char *what; breakFn fn; bool camera; } soft[] = {
    { "frame age 301 ms", bFrame, true }, { "hold coasting", bCoast, true }, { "hold not applied", bApplied, true },
    { "holdSteady false", bSteady, false }, { "roll 4.1", bRoll, false }, { "pitch -4.1", bPitch, false },
    { "gx 30.5", bGx, false }, { "gy -30.5", bGy, false }, { "gz 10.5", bGz, false },
    { "roll 5.9 (inside the hard bound)", sRoll, false }, { "gx 59.5 (inside)", sGx, false },
    { "gz -29.5 (inside)", sGz, false },
  };
  const autoMode_t modes[] = { AUTO_MODE_FLIGHT, AUTO_MODE_DRY, AUTO_MODE_BENCH };
  for (size_t i = 0; i < sizeof(soft) / sizeof(soft[0]); i++) {
    for (size_t m = 0; m < 3; m++) {
      const bool waived = soft[i].camera && modes[m] != AUTO_MODE_FLIGHT;
      const char *mn = modeName(modes[m]);
      /* 100 ms at 500 ms into the streak: it survives, stableMs counts on
       * through the dropout, and the photos start 1 s after the streak began */
      stabRes_t s = stableCase(modes[m], soft[i].fn, 500, 100);
      CHECK(s.hovered && s.brokeAt < 0 && s.countedOn && s.failSteps == (waived ? 0 : 100) &&
            s.capAt == AUTO_STABLE_MS && s.capStableMs == AUTO_STABLE_MS,
            "%s: %s for 100 ms keeps the streak (fail steps %ld, broke %ld, capture at S + %ld)", mn, soft[i].what,
            s.failSteps, s.brokeAt, s.capAt);
      /* 200 ms (from 100 ms in, so the new streak's 1 s comes before
       * NOT_STABLE): the streak ends on the dropout's 150th step; a new
       * one starts when the conditions return, the photos 1 s after it */
      s = stableCase(modes[m], soft[i].fn, 100, 200);
      if (waived) {
        CHECK(s.hovered && s.brokeAt < 0 && s.capAt == AUTO_STABLE_MS, "%s: %s waived (capture at S + %ld)", mn,
              soft[i].what, s.capAt);
      } else {
        CHECK(s.hovered && s.failSteps == AUTO_STABLE_GRACE_MS && s.brokeAt >= 100 + AUTO_STABLE_GRACE_MS &&
              s.brokeAt <= 101 + AUTO_STABLE_GRACE_MS && s.restartAt >= 301 && s.restartAt <= 302 &&
              s.capAt == s.restartAt + AUTO_STABLE_MS,
              "%s: %s for 200 ms restarts it (broke at S + %ld after %ld fail steps, new streak S + %ld, capture "
              "S + %ld)", mn, soft[i].what, s.brokeAt, s.failSteps, s.restartAt, s.capAt);
      }
    }
  }
  /* the grace boundary: 149 steps survive, 150 end the streak */
  stabRes_t s = stableCase(AUTO_MODE_FLIGHT, bRoll, 500, AUTO_STABLE_GRACE_MS - 1);
  CHECK(s.brokeAt < 0 && s.capAt == AUTO_STABLE_MS, "a 149 ms dropout keeps the streak (capture S + %ld)", s.capAt);
  s = stableCase(AUTO_MODE_FLIGHT, bRoll, 500, AUTO_STABLE_GRACE_MS);
  CHECK(s.brokeAt == 500 + AUTO_STABLE_GRACE_MS && s.restartAt == 501 + AUTO_STABLE_GRACE_MS &&
        s.capAt == s.restartAt + AUTO_STABLE_MS, "a 150 ms dropout ends it on its last step (broke S + %ld)", s.brokeAt);
  /* two dropouts of 140 ms each, 10 ms apart: two dropouts, not one of
   * 290, and 280 ms in all, inside the 300 ms budget */
  {
    rig_t r;
    rigInit(&r, 27000);
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000) && untilStable(&r, 3000), "hover, STABLE");
    long k = 0, cap = -1;
    bool broke = false;
    while (k < 3000 && cap < 0) {
      k++;
      bCoast(&r, (k > 300 && k <= 440) || (k > 450 && k <= 590));
      rigStep(&r);
      broke = broke || r.out.stableMs == 0;
      cap = r.out.captureStart ? k : -1;
    }
    CHECK(!broke && cap == AUTO_STABLE_MS, "two 140 ms dropouts 10 ms apart keep the streak (capture S + %ld)", cap);
  }
  /* the photos never start inside a dropout: one over the 1 s mark delays
   * them to its end */
  s = stableCase(AUTO_MODE_FLIGHT, bGz, 950, 100);
  CHECK(s.brokeAt < 0 && s.capAt == 1051 && s.capStableMs == 1051,
        "a dropout over the 1 s mark: the photos at its end (S + %ld, stableMs %u)", s.capAt, (unsigned)s.capStableMs);
  /* ... and NOT_STABLE is not fooled by a streak that ends in a dropout */
  {
    rig_t r;
    rigInit(&r, 27000);
    r.vz.steadyAfterMs = 600;   /* streak from HOLD + 600: 1 s would be HOLD + 1600 */
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
    long k = 0;
    for (; k < 4000 && r.out.state == AUTO_HOVER && lastEvt(&r) != AUTO_EV_NOT_STABLE; k++) {
      bRoll(&r, k >= 1500);   /* conditions lost from HOLD + 1.5 s on */
      rigStep(&r);
    }
    CHECK(r.out.state == AUTO_HOVER && lastEvt(&r) == AUTO_EV_NOT_STABLE && k == AUTO_NOT_STABLE_HOLD_MS &&
          r.captureStarts == 0, "a streak lost 0.9 s in: NOT_STABLE at HOLD + %ld, no photos", k);
  }

  /* a single step beyond a hard bound ends the streak at once */
  const struct { const char *what; breakFn fn; bool camera; } hard[] = {
    { "roll 6.1", hRoll, false }, { "pitch -6.1", hPitch, false }, { "gx -60.5", hGx, false },
    { "gy 60.5", hGy, false }, { "gz 30.5", hGz, false }, { "camera standby", hCam, true },
    { "vz phase IDLE", hVz, false },
  };
  for (size_t i = 0; i < sizeof(hard) / sizeof(hard[0]); i++) {
    for (size_t m = 0; m < 3; m++) {
      const bool waived = hard[i].camera && modes[m] != AUTO_MODE_FLIGHT;
      s = stableCase(modes[m], hard[i].fn, 500, 1);
      if (waived) {
        CHECK(s.brokeAt < 0 && s.capAt == AUTO_STABLE_MS, "%s: %s waived (capture S + %ld)", modeName(modes[m]),
              hard[i].what, s.capAt);
      } else {
        CHECK(s.brokeAt == 501 && s.failSteps == 1 && s.restartAt == 502 && s.capAt == 502 + AUTO_STABLE_MS,
              "%s: %s for one step ends the streak at once (broke S + %ld, new S + %ld, capture S + %ld)",
              modeName(modes[m]), hard[i].what, s.brokeAt, s.restartAt, s.capAt);
      }
    }
  }
}

/* Frame age: 250 and 300 ms pass, 301 and 350 ms never STABLE (NOT_STABLE).
 * A camera with 130 ms latency and a frame every 90 ms (ages 130..219 ms)
 * is STABLE without a single dropout. */
static void testStableFrameAge(void)
{
  const struct { uint32_t age; bool ok; } fa[] = { { AUTO_STABLE_FRAME_AGE_MS-50, true }, { AUTO_STABLE_FRAME_AGE_MS, true }, { AUTO_STABLE_FRAME_AGE_MS+1, false }, { AUTO_STABLE_FRAME_AGE_MS+50, false } };
  for (int i = 0; i < 4; i++) {
    rig_t r;
    rigInit(&r, 31000);
    r.in.camFrameAgeMs = fa[i].age;
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
    long k = 0, cap = -1;
    bool everStable = false;
    for (; k < 4000 && r.out.state == AUTO_HOVER && lastEvt(&r) != AUTO_EV_NOT_STABLE; k++) {
      rigStep(&r);
      everStable = everStable || r.out.stable;
      if (cap < 0 && r.out.captureStart) {
        cap = k + 1;
      }
    }
    if (fa[i].ok) {
      CHECK(cap == 300 + AUTO_STABLE_MS, "frame age %u ms: photos at HOLD + %ld (holdSteady + 1 s)", (unsigned)fa[i].age,
            cap);
    } else {
      CHECK(!everStable && cap < 0 && lastEvt(&r) == AUTO_EV_NOT_STABLE && k == AUTO_NOT_STABLE_HOLD_MS,
            "frame age %u ms: never STABLE, NOT_STABLE at HOLD + %ld", (unsigned)fa[i].age, k);
    }
  }
  rig_t r;
  rigInit(&r, 31000);
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  long k = 0, cap = -1, drops = 0;
  bool seen = false;
  for (; k < 3000 && cap < 0; k++) {
    r.in.camFrameAgeMs = 130 + (uint32_t)(k % 90);
    rigStep(&r);
    seen = seen || r.out.stable;
    drops += seen && !r.out.stable;
    cap = r.out.captureStart ? k + 1 : -1;
  }
  CHECK(cap == 300 + AUTO_STABLE_MS && drops == 0, "frame ages 130..219 ms: photos at HOLD + %ld, %ld dropouts", cap,
        drops);
}

/* NOT_STABLE still fires when STABLE never lasts 1 s: dropouts of 200 ms
 * every 800 ms, or one step beyond a hard bound every 500 ms. */
static void testStableNever(void)
{
  for (int c = 0; c < 2; c++) {
    rig_t r;
    rigInit(&r, 33000);
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
    long k = 0;
    uint32_t maxMs = 0;
    for (; k < 4000 && r.out.state == AUTO_HOVER && lastEvt(&r) != AUTO_EV_NOT_STABLE; k++) {
      if (c == 0) {
        bCoast(&r, k % 800 >= 600);
      } else {
        hGx(&r, k % 500 == 499);
      }
      rigStep(&r);
      maxMs = r.out.stableMs > maxMs ? r.out.stableMs : maxMs;
    }
    CHECK(lastEvt(&r) == AUTO_EV_NOT_STABLE && k == AUTO_NOT_STABLE_HOLD_MS && r.captureStarts == 0 &&
          maxMs < AUTO_STABLE_MS, "%s: NOT_STABLE at HOLD + %ld (longest streak %u ms), no photos",
          c == 0 ? "200 ms coasts every 800 ms" : "gx 60.5 one step every 500 ms", k, (unsigned)maxMs);
  }
}

/* The dropout budget (3.5): a streak ends on the step where its dropouts
 * together reach AUTO_STABLE_DROP_BUDGET_MS (300 ms), so STABLE 1 s means
 * the conditions held >= 70 % of it, and a lasting violation that comes
 * back for a moment now and then is never STABLE. */
typedef void (*lowFn)(rig_t *r, long k);
static void lowCoast(rig_t *r, long k) { bCoast(r, k % 150 >= 10); }
static void lowSteady(rig_t *r, long k) { bSteady(r, k % 150 >= 10); }
static void lowCoastAlt(rig_t *r, long k) { bCoast(r, (k / 90) % 2 == 1); }
static void lowRotate(rig_t *r, long k)
{
  const long ph = k % (5 * 149), which = ph / 149;
  bCoast(r, false);
  bRoll(r, false);
  bGz(r, false);
  bGy(r, false);
  r->in.camFrameAgeMs = 30;
  if (ph % 149 == 148) {
    return;   /* the one good step between */
  }
  switch (which) {
    case 0: bCoast(r, true); break;
    case 1: r->in.rollDeg = 5.5f; break;
    case 2: r->in.gyroDps[2] = 25.0f; break;
    case 3: r->in.camFrameAgeMs = 2000; break;
    default: r->in.gyroDps[1] = 55.0f; break;
  }
}
static void lowYaw(rig_t *r, long k) { r->in.gyroDps[2] = k % 100 == 0 ? 9.5f : 11.0f; }
static void lowLean(rig_t *r, long k) { r->in.rollDeg = k % 100 == 0 ? 3.9f : 4.5f; }

static void testStableBudget(void)
{
  const autoMode_t modes[] = { AUTO_MODE_FLIGHT, AUTO_MODE_DRY, AUTO_MODE_BENCH };
  for (size_t m = 0; m < 3; m++) {
    const char *mn = modeName(modes[m]);
    /* 3 x 99 ms = 297 ms: kept, the photos 1 s after the streak began */
    const drop_t d99[] = { { 100, 99 }, { 300, 99 }, { 500, 99 } };
    stabRes_t s = stableDrops(modes[m], bGz, d99, 3);
    CHECK(s.hovered && s.brokeAt < 0 && s.countedOn && s.failSteps == 297 && s.capAt == AUTO_STABLE_MS &&
          s.capStableMs == AUTO_STABLE_MS, "%s: dropouts of 3 x 99 ms keep the streak (broke %ld, capture S + %ld)",
          mn, s.brokeAt, s.capAt);
    /* 3 x 100 ms: the third's last step (S + 600) brings the sum to 300 ms
     * and ends the streak; the next one (S + 601) starts with a fresh
     * budget, so a 140 ms dropout in it is kept: photos at S + 1601 */
    const drop_t d100[] = { { 100, 100 }, { 300, 100 }, { 500, 100 }, { 800, 140 } };
    s = stableDrops(modes[m], bGz, d100, 4);
    CHECK(s.hovered && s.brokeAt == 600 && s.failSteps == 300 && s.restartAt == 601 &&
          s.capAt == 601 + AUTO_STABLE_MS && s.capStableMs == AUTO_STABLE_MS,
          "%s: 3 x 100 ms end the streak at S + %ld; the next (S + %ld) keeps a 140 ms dropout, photos S + %ld", mn,
          s.brokeAt, s.restartAt, s.capAt);
  }

  /* lasting violations that come back for a moment: never STABLE 1 s,
   * NOT_STABLE at HOLD + 2 s (the camera ones in FLIGHT only) */
  const struct { const char *what; lowFn fn; bool camera; } low[] = {
    { "hold coasting but 10 ms in every 150 ms", lowCoast, true },
    { "holdSteady only 10 ms in every 150 ms", lowSteady, false },
    { "hold coasting on every other 90 ms frame", lowCoastAlt, true },
    { "coast, roll 5.5, gz 25, frame age 2 s, gy 55 in turn, 148 ms each, 1 ms good between", lowRotate, true },
    { "gz 11 dps but one 9.5 dps step in every 100 ms", lowYaw, false },
    { "roll 4.5 deg but one 3.9 deg step in every 100 ms", lowLean, false },
  };
  for (size_t i = 0; i < sizeof(low) / sizeof(low[0]); i++) {
    for (size_t m = 0; m < 2; m++) {
      if (m == 1 && low[i].camera) {
        continue;
      }
      rig_t r;
      rigInit(&r, 33000);
      CHECK(toHover(&r, modes[m], 2, 5000), "hover");
      long k = 0, stableSteps = 0;
      uint32_t maxMs = 0;
      for (; k < 4000 && r.out.state == AUTO_HOVER && lastEvt(&r) != AUTO_EV_NOT_STABLE; k++) {
        low[i].fn(&r, k);
        rigStep(&r);
        stableSteps += r.out.stable;
        maxMs = r.out.stableMs > maxMs ? r.out.stableMs : maxMs;
      }
      CHECK(lastEvt(&r) == AUTO_EV_NOT_STABLE && k == AUTO_NOT_STABLE_HOLD_MS && r.captureStarts == 0 &&
            maxMs < AUTO_STABLE_MS && stableSteps > 0,
            "%s: %s: NOT_STABLE at HOLD + %ld, no photos (stable on %ld steps, longest streak %u ms)",
            modeName(modes[m]), low[i].what, k, stableSteps, (unsigned)maxMs);
    }
  }

  /* On the drone now is the 1 kHz tick and the loop runs slower, so now
   * moves 1 or 2 ms a step; a dropout counts in ms, whatever the spacing.
   * With 1.5 ms steps, dropouts of 3 x 90 ms (270 ms) keep the streak and
   * the photos come when it is 1 s old; 4 x 90 ms end it in the fourth. */
  for (int n = 3; n <= 4; n++) {
    rig_t r;
    rigInit(&r, 27000);
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000) && untilStable(&r, 3000) && r.out.stableMs == 0, "hover, STABLE");
    const uint32_t s0 = r.in.nowMs - 1;   /* now on the streak's first step */
    long brokeAt = -1, capAt = -1;
    uint32_t capStableMs = 0;
    for (long k = 1; k <= 2000 && r.out.state == AUTO_HOVER && brokeAt < 0 && capAt < 0; k++) {
      r.in.nowMs += (uint32_t)(k & 1);   /* 2 ms on odd steps, 1 on even ones */
      const uint32_t t = r.in.nowMs - s0;
      bool bad = false;
      for (int i = 0; i < n; i++) {
        bad = bad || (t > 100u + 200u * (uint32_t)i && t <= 190u + 200u * (uint32_t)i);
      }
      bCoast(&r, bad);
      rigStep(&r);
      if (r.out.stableMs == 0) {
        brokeAt = (long)t;
      } else if (r.out.captureStart) {
        capAt = (long)t;
        capStableMs = r.out.stableMs;
      }
    }
    if (n == 3) {
      CHECK(brokeAt < 0 && capAt >= AUTO_STABLE_MS && capAt <= AUTO_STABLE_MS + 1 && capStableMs == (uint32_t)capAt,
            "1.5 ms steps, 3 x 90 ms dropouts: kept, photos at S + %ld ms (stableMs %u)", capAt,
            (unsigned)capStableMs);
    } else {
      CHECK(brokeAt > 700 && brokeAt <= 790 && capAt < 0,
            "1.5 ms steps, 4 x 90 ms dropouts: the fourth ends the streak (S + %ld ms)", brokeAt);
    }
  }
}

static void testStable(void)
{
  /* holdSteady from the controller */
  rig_t r;
  rigInit(&r, 27000);
  r.vz.neverSteady = true;
  r.in.camViewMs = 0;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  run(&r, 1500);
  CHECK(!r.out.stable && r.captureStarts == 0, "without holdSteady never STABLE");

  /* capture exactly once; DONE -> the hover wait (wait 0: SORTIE_DONE) */
  rigInit(&r, 27000);
  r.camAuto = false;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(untilCapture(&r, 3000), "capture");
  run(&r, 500);
  CHECK(r.captureStarts == 1 && r.out.state == AUTO_HOVER && r.out.stable && !r.out.waiting,
        "capture requested once, stable on, not waiting yet");
  r.in.capture = AUTO_CAP_DONE;
  rigStep(&r);
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting && r.captureStarts == 1, "capture DONE: HOVER waits");
  rigInit(&r, 27000);
  r.camAuto = false;
  r.in.hoverWaitNvsMs = 0;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover, wait 0");
  CHECK(untilCapture(&r, 3000), "capture");
  run(&r, 500);
  r.in.capture = AUTO_CAP_DONE;
  rigStep(&r);
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting && r.captureStarts == 1,
        "wait 0, capture DONE: the wait starts anyway (no SORTIE_DONE since 2026-10-15)");

  /* FAILED -> the event CAPTURE_FAIL, the wait starts (LAND CAPTURE_FAIL until 2026-10-15) */
  rigInit(&r, 27000);
  r.camAuto = false;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(untilCapture(&r, 3000), "capture");
  run(&r, 100);
  r.in.capture = AUTO_CAP_FAILED;
  rigStep(&r);
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting && lastEvt(&r) == AUTO_EV_CAPTURE_FAIL,
        "capture FAILED: CAPTURE_FAIL, the wait");

  /* A stuck encoder still reports CAPTURE_FAIL at the bounded profile deadline. */
  rigInit(&r, 27000);
  r.camAuto = false;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(untilCapture(&r, 3000), "capture");
  const long t = untilEvt(&r, AUTO_EV_CAPTURE_FAIL, AUTO_CAPTURE_TIMEOUT_MS(2, 150) + 2);
  CHECK(t == AUTO_CAPTURE_TIMEOUT_MS(2, 150) + 1 && r.out.state == AUTO_HOVER && r.out.waiting,
        "stuck capture: CAPTURE_FAIL %ld ms after the start, the wait", t);

  /* a stale DONE from before the start is not this sortie's */
  rigInit(&r, 27000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  r.in.capture = AUTO_CAP_DONE;
  run(&r, 200);
  CHECK(r.out.state == AUTO_HOVER, "capture DONE before any start is ignored");

  /* photoN 0: after 1 s STABLE the wait (no capture); wait 0: SORTIE_DONE */
  rigInit(&r, 27000);
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 0, 5000), "hover, no photos");
  long w = 0;
  while (!r.out.waiting && w < 3000) {
    rigStep(&r);
    w++;
  }
  CHECK(r.out.waiting && r.out.state == AUTO_HOVER && r.captureStarts == 0 && r.out.stableMs == AUTO_STABLE_MS,
        "photoN 0: waiting after 1 s STABLE (%ld), no capture", w);
  rigInit(&r, 27000);
  r.in.hoverWaitNvsMs = 0;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 0, 5000), "hover, no photos, wait 0");
  long s = 0;
  long firstStable = -1;
  while (!r.out.waiting && r.out.state == AUTO_HOVER && s < 3000) {
    rigStep(&r);
    s++;
    if (firstStable < 0 && r.out.stable) {
      firstStable = s;
    }
  }
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting && r.captureStarts == 0 && s - firstStable == AUTO_STABLE_MS,
        "photoN 0, wait 0: the wait 1 s after STABLE (%ld), no capture, no SORTIE_DONE", s - firstStable);
  r.in.cmd = cmdOf(AUTO_CMD_LAND);
  CHECK(untilState(&r, AUTO_LAND, 10) > 0 && !r.out.stable && r.out.stableMs == 0,
        "STABLE is only reported in HOVER");
}

/* ---- 12b. hover wait, PHOTO, camera height rule ------------------------------ */

static autoCmd_t cmdPhoto(uint8_t n, uint16_t gapMs)
{
  autoCmd_t c = cmdOf(AUTO_CMD_PHOTO);
  c.photoN = n;
  c.photoGapMs = gapMs;
  return c;
}

/* Hover until the wait has started; returns false if it never does. */
static bool toWait(rig_t *r, autoMode_t mode)
{
  if (!toHover(r, mode, 2, 0)) {
    return false;
  }
  for (int i = 0; i < 4000 && r->out.state == AUTO_HOVER; i++) {
    rigStep(r);
    if (r->out.waiting) {
      return true;
    }
  }
  return false;
}

static void testHoverWait(void)
{
  rig_t r;
  /* a Mac LAND during the wait: LAND_CMD */
  rigInit(&r, 39000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  run(&r, 1500);
  autoFsmOut_t o = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(o.cmdResult == AUTO_R_OK && o.state == AUTO_LAND && o.reason == AUTO_RS_LAND_CMD && !o.waiting,
        "LAND during the wait: LAND LAND_CMD");
  CHECK(toIdle(&r, 6000) && r.out.reason == AUTO_RS_TOUCHDOWN, "and lands");

  /* the phone during the wait: slow LAND */
  rigInit(&r, 39000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  run(&r, 700);
  appPacket(&r, 0.0f, 0.0f, 0.0f, 1500.0f);
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_APP_INPUT && r.out.thrust == AUTO_THR_VERT_LAND,
        "phone stick during the wait: LAND APP_INPUT, no cut");

  /* no CAPTURE_FAIL during the wait: the set is done, its timeout is moot */
  rigInit(&r, 39000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  run(&r, 2 * 150 + AUTO_CAPTURE_EXTRA_MS + 500);
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting, "the capture timeout does not fire after DONE");

  /* PHOTO: accepted any time in HOVER with no set running (2026-10-15; only while waiting before): before the wait it
   * is the take-off's set, and the wait starts when it is done */
  rigInit(&r, 39000);
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0), "hover");
  o = doCmd(&r, cmdPhoto(1, 150));
  CHECK(o.cmdResult == AUTO_R_OK && o.captureStart && o.state == AUTO_HOVER && !o.waiting,
        "PHOTO before the wait: OK (captureStart)");
  while (!r.out.waiting && r.out.state == AUTO_HOVER) {
    rigStep(&r);
  }
  CHECK(r.out.waiting && r.captureStarts == 1, "the wait starts when it is done (one set)");
  run(&r, 1000);
  o = doCmd(&r, cmdPhoto(4, 150));
  CHECK(o.cmdResult == AUTO_R_RANGE && !o.captureStart, "PHOTO n 4: RANGE");
  o = doCmd(&r, cmdPhoto(1, 149));
  CHECK(o.cmdResult == AUTO_R_RANGE && !o.captureStart, "PHOTO gap 149: RANGE");
  o = doCmd(&r, cmdPhoto(3, 200));
  CHECK(o.cmdResult == AUTO_R_OK && o.captureStart && o.captureN == 3 && o.captureGapMs == 200 && o.waiting &&
        r.captureStarts == 2, "PHOTO 3 x 200 ms while waiting: OK, captureStart");
  o = doCmd(&r, cmdPhoto(1, 150));
  CHECK(o.cmdResult == AUTO_R_BUSY && !o.captureStart && r.captureStarts == 2 && o.waitLeftMs == 0,
        "PHOTO while a set runs: BUSY (the wait shows no countdown)");
  run(&r, 700);
  CHECK(r.in.capture == AUTO_CAP_DONE && r.out.waiting, "second set done, still waiting");
  o = doCmd(&r, cmdPhoto(1, 150));
  CHECK(o.cmdResult == AUTO_R_OK && o.captureStart && r.captureStarts == 3, "and another PHOTO is accepted");
  CHECK(untilState(&r, AUTO_LAND, 8000) < 0 && r.out.waiting, "8 s without a command: still waiting (no HOVER_TIMEOUT)");

  /* a set that fails in the wait: the event CAPTURE_FAIL, the set consumed (PHOTO again), no LAND */
  rigInit(&r, 39000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  r.camAuto = false;
  CHECK(doCmd(&r, cmdPhoto(1, 150)).cmdResult == AUTO_R_OK, "PHOTO");
  run(&r, 100);
  r.in.capture = AUTO_CAP_FAILED;
  rigStep(&r);
  CHECK(r.out.state == AUTO_HOVER && lastEvt(&r) == AUTO_EV_CAPTURE_FAIL && r.out.waiting,
        "a PHOTO set FAILED in the wait: the event CAPTURE_FAIL, HOVER");
  run(&r, 50);
  CHECK(lastEvt(&r) == AUTO_EV_CAPTURE_FAIL && autoFsmReport(&r.f)->evCount >= 1, "reported once");
  r.camAuto = true;
  o = doCmd(&r, cmdPhoto(1, 150));
  CHECK(o.cmdResult == AUTO_R_OK && o.captureStart, "PHOTO again: OK (not BUSY)");
  /* a set that times out before the wait: the event, the wait starts */
  rigInit(&r, 39000);
  r.camAuto = false;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0) && untilCapture(&r, 3000), "the take-off set starts");
  long tf = untilEvt(&r, AUTO_EV_CAPTURE_FAIL, AUTO_CAPTURE_TIMEOUT_MS(2, 150) + 2);
  CHECK(tf == AUTO_CAPTURE_TIMEOUT_MS(2, 150) + 1 && r.out.waiting && r.out.state == AUTO_HOVER,
        "the take-off set times out (%ld ms): CAPTURE_FAIL, the wait starts", tf);

  /* PHOTO elsewhere: BAD_STATE */
  rigInit(&r, 39000);
  run(&r, 5);
  o = doCmd(&r, cmdPhoto(1, 150));
  CHECK(o.cmdDone && o.cmdResult == AUTO_R_BAD_STATE && o.state == AUTO_IDLE && !o.captureStart, "PHOTO in IDLE: BAD_STATE");

  /* DRY waives the camera rule: the whole wait on the sim */
  rigInit(&r, 39000);
  r.vz.noCam = true;
  CHECK(toWait(&r, AUTO_MODE_DRY), "DRY waiting without a camera");
  CHECK(untilState(&r, AUTO_LAND, 6000) < 0 && r.out.state == AUTO_HOVER, "DRY: 6 s without a camera, no LAND");
}

/* The camera rule (auto_fsm.h AUTO_CAM_*; "Claude decides", 2026-10-15): CAM_LOST at 300 ms, CAM_BACK after 500 ms
 * in use, the SINK 5 s after the last step in use with capQ (at once without), resumable while it lasts, a LAND that
 * Claude or a minimum rule asks for takes it over. HOVER_NO_CAM until then. */
static void testHoverCamera(void)
{
  rig_t r;
  /* no camera height from the start (no capQ): SINK at HOVER + 2 s (today's timing), then resumed when it comes */
  rigInit(&r, 41000);
  r.vz.noCam = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0), "hover");
  long t = untilState(&r, AUTO_LAND, 4000);
  CHECK(t == AUTO_HOVER_ACCEL_DEFAULT_MS && r.out.reason == AUTO_RS_SINK && r.captureStarts == 1 &&
            hadEvt(&r, AUTO_EV_SINK, NULL) && (autoFsmReport(&r.f)->holdBits & AUTO_HB_SINKING),
        "no camera height, no capQ: SINK at HOVER + %ld ms (photos started at 1.3 s)", t);
  CHECK(r.out.thrust == AUTO_THR_VERT_LAND && r.out.vzCmd == VZ_CMD_LAND, "the SINK is the controller's LAND");
  run(&r, 300);
  r.vz.noCam = false;
  long tr = untilState(&r, AUTO_HOVER, 2000);
  CHECK(tr > 0 && r.out.reason == AUTO_RS_RESUME && hadEvt(&r, AUTO_EV_RESUME, NULL) &&
            !(autoFsmReport(&r.f)->holdBits & (AUTO_HB_SINKING | AUTO_HB_CAM_LOST)),
        "the camera height back 500 ms: the controller resumes, HOVER RESUME (%ld ms)", tr);

  /* healthy (capQ after 1.5 s of HOLD), then lost mid-wait: CAM_LOST at 300 ms, SINK at 5 s */
  rigInit(&r, 41000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  run(&r, 1500);
  CHECK(r.vz.o.capQ > 0.0f, "capQ measured");
  r.in.camBits = AUTO_CB_NO_G;
  r.vz.noCam = true;
  rigStep(&r);   /* the controller's output loses it */
  long tl = untilEvt(&r, AUTO_EV_CAM_LOST, 1000);
  CHECK(tl == AUTO_CAM_LOST_REPORT_MS && r.out.state == AUTO_HOVER && lastEvtDetail(&r) == AUTO_CB_NO_G &&
            (autoFsmReport(&r.f)->holdBits & AUTO_HB_CAM_LOST),
        "camera height lost: CAM_LOST %ld ms after the last step in use (detail no_G), HOVER", tl);
  const autoFsmReport_t *p = autoFsmReport(&r.f);
  CHECK(p->sinkAtMs - p->camOutMs == AUTO_CAM_LOST_MAX_MS, "the SINK due 5 s after it (sinkAtMs)");
  t = untilState(&r, AUTO_LAND, 6000);
  CHECK(t + tl == AUTO_CAM_LOST_MAX_MS && r.out.reason == AUTO_RS_SINK && lastEvtDetail(&r) == 0,
        "SINK %ld ms after the last step in use (the 5 s budget)", t + tl);

  /* lost 2 s, back: CAM_BACK 500 ms after its return, no SINK */
  rigInit(&r, 41000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  run(&r, 1500);
  r.vz.noCam = true;
  run(&r, 2000);
  CHECK(r.out.state == AUTO_HOVER && hadEvt(&r, AUTO_EV_CAM_LOST, NULL), "lost 2 s: CAM_LOST, HOVER");
  r.vz.noCam = false;
  long tb = untilEvt(&r, AUTO_EV_CAM_BACK, 2000);
  /* (the controller's output one step later, the FSM reads it the next) */
  CHECK(tb == AUTO_CAM_BACK_MS + 2 && !(autoFsmReport(&r.f)->holdBits & AUTO_HB_CAM_LOST),
        "back: CAM_BACK %ld ms later", tb);
  CHECK(untilState(&r, AUTO_LAND, 8000) < 0, "no SINK after it");

  /* a flicker shorter than 500 ms does not restart the budget */
  rigInit(&r, 41000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  run(&r, 1500);
  r.vz.noCam = true;
  rigStep(&r);
  const uint32_t lost0 = r.in.nowMs;
  run(&r, 2000);
  r.vz.noCam = false;
  run(&r, 400);
  r.vz.noCam = true;
  t = untilState(&r, AUTO_LAND, 6000);
  CHECK(t > 0 && r.out.reason == AUTO_RS_SINK && r.in.nowMs - lost0 <= AUTO_CAM_LOST_MAX_MS + 2,
        "lost, back 400 ms, lost again: SINK %u ms after the first loss", (unsigned)(r.in.nowMs - lost0));

  /* in a SINK Claude's LAND takes it over: no resume when the camera height returns */
  rigInit(&r, 41000);
  r.vz.noCam = true;
  r.vz.neverDone = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0) && untilState(&r, AUTO_LAND, 4000) > 0 && r.out.reason == AUTO_RS_SINK,
        "SINK");
  autoFsmOut_t lo = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(lo.cmdResult == AUTO_R_OK && lo.state == AUTO_LAND && lo.reason == AUTO_RS_LAND_CMD,
        "a LAND command in the SINK: OK, the LAND is LAND_CMD now");
  r.vz.noCam = false;
  run(&r, 2000);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_LAND_CMD && !r.out.resume && r.vz.o.phase == VZ_LAND,
        "the camera height back: no resume (Claude's LAND stands)");
  /* ... and a lost link */
  rigInit(&r, 41000);
  r.vz.noCam = true;
  r.vz.neverDone = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0) && untilState(&r, AUTO_LAND, 4000) > 0, "SINK");
  r.hbStream = false;
  run(&r, 1100);
  r.vz.noCam = false;
  run(&r, 2000);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_HB_LOST && r.vz.o.phase == VZ_LAND,
        "the link lost in the SINK: HB_LOST, no resume");

  /* a camera that becomes healthy within the 2 s is fine */
  rigInit(&r, 41000);
  r.vz.noCam = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0), "hover");
  run(&r, AUTO_HOVER_ACCEL_DEFAULT_MS - 10);
  r.vz.noCam = false;
  run(&r, 3000);
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting, "healthy by HOVER + 1.99 s: the wait goes on");

  /* NVS: autoaccel 3000 sinks at 3 s; a broken 9000 is forced to 3000; 0 needs the camera from the first step */
  const struct { uint16_t nvs; long want; } nv[] = { { 3000, 3000 }, { 9000, AUTO_HOVER_ACCEL_MAX_MS }, { 0, 1 } };
  for (int i = 0; i < 3; i++) {
    rigInit(&r, 41000);
    r.vz.noCam = true;
    r.in.hoverAccelNvsMs = nv[i].nvs;
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0), "hover");
    t = untilState(&r, AUTO_LAND, 5000);
    CHECK(t == nv[i].want && r.out.reason == AUTO_RS_SINK, "autoaccel %u: SINK %ld ms into HOVER (%s)",
          (unsigned)nv[i].nvs, t, autoReasonName(r.out.reason));
  }

  /* a HOLD a climb fault started (CONTACT_HOLD here: the hover not measured): the camera height out sinks at once */
  rigInit(&r, 41000);
  r.vz.climbEnd = VZ_R_CONTACT_HOLD;
  r.vz.noCapQ = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0) && r.f.faultHover, "hover from a contact's HOLD");
  run(&r, 200);
  r.vz.noCam = true;
  t = untilState(&r, AUTO_LAND, 1000);
  CHECK(t == 2 && r.out.reason == AUTO_RS_SINK && lastEvtDetail(&r) == 2,
        "the camera height out after a climb fault: SINK at once (%ld ms, detail 2)", t);

  /* BENCH and DRY waive it */
  for (int m = 1; m <= 2; m++) {
    rigInit(&r, 41000);
    r.vz.noCam = true;
    r.vz.neverSteady = true;
    CHECK(toHover(&r, (autoMode_t)m, 2, 0), "hover");
    t = untilState(&r, AUTO_LAND, 4000);
    CHECK(t < 0 && hadEvt(&r, AUTO_EV_NOT_STABLE, NULL), "%s: no camera rule (NOT_STABLE, no LAND)",
          m == AUTO_MODE_DRY ? "DRY" : "BENCH");
  }
}

/* ---- 13. KILLED exit --------------------------------------------------------- */

static void killInHover(rig_t *r)
{
  r->vz.neverSteady = true;
  CHECK(toHover(r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  for (int s = 0; s <= AUTO_TILT_KILL_MS; s++) {
    setTrip(&r->in, TRIP_TILT, true);
    rigStep(r);
  }
  CHECK(r->out.state == AUTO_KILLED && r->estop, "KILLED, emergency stop set");
}

static void testKilledExit(void)
{
  rig_t r;
  rigInit(&r, 29000);
  killInHover(&r);
  imuNormal(&r.in);   /* at rest at once */
  long t = untilState(&r, AUTO_IDLE, 10000);
  CHECK(t == AUTO_KILLED_MIN_MS && r.out.reason == AUTO_RS_KILL_CLEARED,
        "at rest: KILLED -> IDLE at 2000 ms (%ld)", t);
  CHECK(r.out.lockAppThrust && r.out.clearEmergencyStop && !r.estop && r.clearEstops == 1,
        "lockAppThrust, and the FSM clears its own emergency stop");
  CHECK(r.out.thrust == AUTO_THR_ZERO, "release hold after KILLED");

  rigInit(&r, 29000);
  killInHover(&r);
  r.in.gyroDps[1] = 40.0f;   /* tumbling, then still at 2.5 s */
  run(&r, 2500);
  CHECK(r.out.state == AUTO_KILLED, "not at rest: still KILLED at 2.5 s");
  imuNormal(&r.in);
  t = untilState(&r, AUTO_IDLE, 10000);
  CHECK(t == AUTO_KILLED_REST_MS + 1, "IDLE once at rest for 1 s (first rest step + 1000: %ld)", t);

  rigInit(&r, 29000);
  killInHover(&r);
  imuNormal(&r.in);
  r.appStream = false;
  for (int i = 0; i < 11; i++) {   /* the thumb still on the throttle, packets every 300 ms */
    appPacket(&r, 0.0f, 0.0f, 0.0f, 30000.0f);
    run(&r, 300);
  }
  appPacket(&r, 0.0f, 0.0f, 0.0f, 30000.0f);
  rigStep(&r);
  CHECK(r.out.state == AUTO_KILLED && r.applied == 0.0f, "app thrust held: still KILLED at 3.3 s");
  appPacket(&r, 0.0f, 0.0f, 0.0f, 0.0f);
  rigStep(&r);
  CHECK(r.out.state == AUTO_IDLE && r.out.reason == AUTO_RS_KILL_CLEARED, "app thrust 0: IDLE");
  run(&r, AUTO_RELEASE_HOLD_MS);
  CHECK(r.out.thrust == AUTO_THR_NONE, "then the phone (thrust-locked by the glue) has the setpoint");

  rigInit(&r, 29000);
  killInHover(&r);
  imuNormal(&r.in);
  r.appStream = false;
  appPacket(&r, 0.0f, 0.0f, 0.0f, 30000.0f);
  t = untilState(&r, AUTO_IDLE, 10000);
  CHECK(t > 0 && r.in.appAgeMs > AUTO_APP_ARM_MAX_AGE_MS, "app absent > 500 ms counts as released");

  /* Emergency stop set by someone else: KILLED ESTOP_EXT, never cleared by us. */
  rigInit(&r, 29000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  r.estop = true;
  rigStep(&r);
  CHECK(r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_ESTOP_EXT, "external emergency stop: KILLED ESTOP_EXT");
  CHECK(toIdle(&r, 5000) && !r.out.clearEmergencyStop && r.estop && r.clearEstops == 0,
        "KILLED -> IDLE leaves the external emergency stop alone");

  const struct { const char *what; autoReason_t why; } mf[] = {
    { "motor watchdog tripped", AUTO_RS_MOTOR_GUARD }, { "motor driver stopped", AUTO_RS_MOTOR_STOP },
  };
  for (int i = 0; i < 2; i++) {
    rigInit(&r, 29000);
    r.vz.neverSteady = true;
    CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
    if (i == 0) {
      r.in.motorGuardTripped = true;
    } else {
      r.in.motorsReady = false;
    }
    rigStep(&r);
    CHECK(r.out.state == AUTO_KILLED && r.out.reason == mf[i].why, "%s: KILLED %s", mf[i].what,
          autoReasonName(r.out.reason));
  }
}

/* ---- vertical controller outcomes ----------------------------------------- */

static void testVzOutcomes(void)
{
  rig_t r;
  /* controller CUT airborne */
  rigInit(&r, 31000);
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  run(&r, AUTO_PRESPIN_MS + 1500);
  r.vz.o.phase = VZ_CUT;
  r.vz.o.reason = VZ_R_NAN_INPUT;
  r.vz.active = false;
  rigStep(&r);
  CHECK(r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_VZ_CUT && r.out.vzLandReason == VZ_R_NAN_INPUT,
        "controller CUT in TAKEOFF: KILLED VZ_CUT (vz NAN_INPUT)");

  /* controller lands on its own */
  rigInit(&r, 31000);
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  run(&r, AUTO_PRESPIN_MS + 1500);
  CHECK(r.vz.o.phase == VZ_CLIMB, "climbing");
  r.vz.o.phase = VZ_LAND;
  r.vz.o.reason = VZ_R_CLIMB_CAPPED;
  r.vz.o.landReason = VZ_R_CLIMB_CAPPED;
  r.vz.t = 0;
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_VZ && r.out.vzLandReason == VZ_R_CLIMB_CAPPED,
        "controller LAND (CLIMB_CAPPED): LAND VZ");
  CHECK(toIdle(&r, 5000) && r.out.reason == AUTO_RS_TOUCHDOWN && r.out.vzLandReason == VZ_R_TOUCHDOWN_FORCE,
        "then IDLE TOUCHDOWN");

  /* the LAND timeout cut of the controller ends LAND without a kill */
  rigInit(&r, 31000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  run(&r, 3000);
  r.vz.o.phase = VZ_CUT;
  r.vz.o.reason = VZ_R_LAND_TIMEOUT;
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.thrust == AUTO_THR_ZERO, "controller CUT in LAND: ended, thrust 0");
  CHECK(toIdle(&r, 5000) && r.out.reason == AUTO_RS_LAND_TIMEOUT && !r.sawCut, "IDLE LAND_TIMEOUT");

  rigInit(&r, 31000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  run(&r, 100);
  r.vz.o.phase = VZ_CUT;
  r.vz.o.reason = VZ_R_NAN_INPUT;
  rigStep(&r);
  CHECK(toIdle(&r, 5000) && r.out.reason == AUTO_RS_VZ_CUT, "other controller CUT in LAND: IDLE VZ_CUT");

  /* a controller that never ends: backstop at 6.5 s */
  rigInit(&r, 31000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  const long t = untilState(&r, AUTO_KILLED, 8000);
  CHECK(t == AUTO_LAND_BACKSTOP_MS && r.out.reason == AUTO_RS_LAND_BACKSTOP, "LAND backstop at %ld ms", t);

  /* never at rest after touchdown: IDLE 3 s after the end */
  rigInit(&r, 31000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  CHECK(untilLandEnded(&r, 7000), "touchdown");
  r.in.gyroDps[0] = 12.0f;
  const long t2 = untilState(&r, AUTO_IDLE, 5000);
  CHECK(t2 == AUTO_LAND_REST_MAX_MS && r.out.reason == AUTO_RS_TOUCHDOWN, "not at rest: IDLE 3000 ms after (%ld)", t2);

  /* rest counted from the end, not before it (a steady descent reads 1 g) */
  rigInit(&r, 31000);
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND (IMU at rest all along)");
  CHECK(untilLandEnded(&r, 7000), "touchdown");
  const long t3 = untilState(&r, AUTO_IDLE, 5000);
  CHECK(t3 == AUTO_LAND_REST_MS, "IDLE exactly 1 s after the end (%ld)", t3);
}

/* ---- ARMED and pre-spin exits ----------------------------------------------- */

static void testArmedExits(void)
{
  rig_t r;
  /* MOVED: 100 ms */
  rigInit(&r, 33000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
  int s = 0;
  while (r.out.state == AUTO_ARMED && s < 500) {
    r.in.gyroDps[0] = 10.5f;
    rigStep(&r);
    s++;
  }
  CHECK(r.out.state == AUTO_IDLE && r.out.reason == AUTO_RS_MOVED && s == AUTO_MOVED_MS + 1,
        "ARMED, gx 10.5 dps: IDLE MOVED once it held 100 ms (%d)", s);
  imuNormal(&r.in);

  const struct { const char *what; autoReason_t why; } ex[] = {
    { "session", AUTO_RS_SESSION }, { "LAND", AUTO_RS_LAND_CMD }, { "DISARM", AUTO_RS_DISARM_CMD },
    { "lock", AUTO_RS_LOCKED },
  };
  for (int pre = 0; pre < 2; pre++) {
    for (size_t i = 0; i < sizeof(ex) / sizeof(ex[0]); i++) {
      rigInit(&r, 33000);
      if (pre) {
        CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
      } else {
        prepMode(&r, AUTO_MODE_FLIGHT);
        CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
      }
      run(&r, 200);
      autoFsmOut_t o;
      if (ex[i].why == AUTO_RS_SESSION) {
        r.in.sessionChanged = true;
        rigStep(&r);
        o = r.out;
      } else if (ex[i].why == AUTO_RS_LOCKED) {
        r.in.restartLocked = true;
        rigStep(&r);
        o = r.out;
      } else {
        o = doCmd(&r, cmdOf(ex[i].why == AUTO_RS_LAND_CMD ? AUTO_CMD_LAND : AUTO_CMD_DISARM));
        CHECK(o.cmdResult == AUTO_R_OK, "accepted");
      }
      CHECK(o.state == AUTO_IDLE && o.reason == ex[i].why && o.lockAppThrust && o.thrust == AUTO_THR_ZERO &&
            r.vzResets == 0, "%s: %s in %s -> IDLE, no motor turned", pre ? "pre-spin" : "ARMED", ex[i].what,
            autoReasonName(o.reason));
    }
  }

  /* ARM timeout exactly at 10 s (also in the token test) and nothing else
   * leaves ARMED with all streams healthy. */
  rigInit(&r, 33000);
  prepMode(&r, AUTO_MODE_FLIGHT);
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED, "armed");
  run(&r, AUTO_ARM_TIMEOUT_MS - 2);
  CHECK(r.out.state == AUTO_ARMED && r.out.stateMs == AUTO_ARM_TIMEOUT_MS - 2, "ARMED at 9998 ms");
  rigStep(&r);
  rigStep(&r);
  CHECK(r.out.state == AUTO_IDLE && r.out.reason == AUTO_RS_ARM_TIMEOUT, "IDLE ARM_TIMEOUT at 10000 ms");
}

/* ---- 14. modes --------------------------------------------------------------- */

static void testModes(void)
{
  /* DRY: the controller flies the sim, the motors get 0 at every step. */
  rig_t r;
  rigInit(&r, 35000);
  CHECK(launch(&r, AUTO_MODE_DRY, 2, 0), "DRY launched");
  bool dryOk = true;
  float vzMax = 0.0f;
  while (r.out.state != AUTO_IDLE && r.steps < 20000) {
    if (r.out.waiting && r.in.cmd.type == AUTO_CMD_NONE && r.out.state == AUTO_HOVER) {
      r.in.cmd = cmdOf(AUTO_CMD_LAND);   /* Claude's LAND in the wait (no HOVER_TIMEOUT since 2026-10-15) */
    }
    rigStep(&r);
    if (r.out.state != AUTO_IDLE) {
      dryOk &= r.out.thrustMax == 0.0f && r.out.mode == AUTO_MODE_DRY;
    }
    if (r.vz.o.thrust > vzMax) {
      vzMax = r.vz.o.thrust;
    }
  }
  CHECK(dryOk && r.maxApplied == 0.0f && vzMax >= 36000.0f && r.captureStarts == 1,
        "DRY: full sortie, controller up to %.0f, motors 0 throughout", vzMax);
  CHECK(r.out.reason == AUTO_RS_TOUCHDOWN && r.out.thrustMax == 0.0f, "DRY ends IDLE; release hold still clamped");
  run(&r, AUTO_RELEASE_HOLD_MS);
  CHECK(r.out.thrustMax == 65535.0f, "IDLE after the release hold: 65535");

  /* BENCH: min(gcap at ARM, 15000), then only lowered. */
  const struct { uint16_t gcap; float cap; } bc[] = { { 12000, 12000.0f }, { 18000, 15000.0f }, { 1, 1.0f } };
  for (int i = 0; i < 3; i++) {
    rigInit(&r, 35000);
    CHECK(launch(&r, AUTO_MODE_BENCH, 2, 0), "BENCH launched");
    r.in.gcap = bc[i].gcap;   /* launch() armed at 15000; re-arm at this gcap */
    CHECK(doCmd(&r, cmdOf(AUTO_CMD_DISARM)).state == AUTO_IDLE, "disarm");
    run(&r, 100);
    const autoFsmOut_t o = doCmd(&r, cmdArm(AUTO_MODE_BENCH, 0));
    CHECK(o.state == AUTO_ARMED && o.thrustMax == bc[i].cap, "BENCH gcap %u: cap %.0f (%.0f)", bc[i].gcap, bc[i].cap,
          o.thrustMax);
  }
  rigInit(&r, 35000);
  r.in.gcap = 12000;
  r.in.simPlant = true;
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_BENCH, 0)).state == AUTO_ARMED, "BENCH armed at gcap 12000");
  run(&r, 50);
  CHECK(doCmd(&r, cmdTakeoff(TOKEN, 30, 2, 150)).state == AUTO_TAKEOFF, "BENCH take-off");
  CHECK(untilState(&r, AUTO_HOVER, 6000) > 0, "BENCH hover");
  CHECK(r.out.thrustMax == 12000.0f && r.applied == 12000.0f, "BENCH clamps 36000 to 12000 (%.0f)", r.applied);
  r.in.gcap = 9000;
  rigStep(&r);
  CHECK(r.out.thrustMax == 9000.0f && r.applied == 9000.0f, "a smaller gcap lowers the cap");
  r.in.gcap = 0;
  rigStep(&r);
  CHECK(r.out.thrustMax == 9000.0f, "gcap 0 does not raise it");
  r.in.gcap = 20000;
  rigStep(&r);
  CHECK(r.out.thrustMax == 9000.0f && r.maxApplied <= 12000.0f, "a larger gcap does not raise it");

  /* FLIGHT: VZ_THRUST_ABS_MAX (58000) at the last stage even if the controller asks more */
  rigInit(&r, 35000);
  r.vz.holdThrust = 60000.0f;
  r.vz.neverSteady = true;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 5000), "hover");
  rigStep(&r);
  CHECK(r.out.thrustMax == VZ_THRUST_ABS_MAX && r.applied == VZ_THRUST_ABS_MAX && VZ_THRUST_ABS_MAX == 58000.0f,
        "FLIGHT: 60000 asked, 58000 applied (%.0f)", r.applied);
  /* ... and a T_hold of 60000 is over AUTO_HEADROOM_LAND_T: BATT_LOW AUTO_VBAT_LOW_LAND_MS after the HOVER entry
   * (the step the FSM first read the controller's HOLD) */
  run(&r, AUTO_VBAT_LOW_LAND_MS - 2);
  CHECK(r.out.state == AUTO_HOVER, "T_hold 60000 for %u ms: HOVER", AUTO_VBAT_LOW_LAND_MS - 1);
  rigStep(&r);
  CHECK(r.out.state == AUTO_LAND && r.out.reason == AUTO_RS_BATT_LOW && r.out.thrust == AUTO_THR_VERT_LAND &&
            r.out.vzCmd == VZ_CMD_LAND,
        "T_hold 60000 for %u ms: LAND BATT_LOW (%s)", AUTO_VBAT_LOW_LAND_MS, autoReasonName(r.out.reason));

  /* IDLE: the app's setpoint passes untouched */
  rigInit(&r, 35000);
  appPacket(&r, 0.0f, 0.0f, 0.0f, 30000.0f);
  rigStep(&r);
  CHECK(r.out.thrust == AUTO_THR_NONE && r.out.thrustMax == 65535.0f && r.applied == 30000.0f,
        "IDLE: setpoint untouched");
}

/* ---- 15. stop-type commands never refused (table 3.3) ----------------------- */

static void testCommandTable(void)
{
  /* expected state after each command, per table 3.3 */
  const autoState_t afterLand[AT_N] = { AUTO_IDLE, AUTO_IDLE, AUTO_IDLE, AUTO_LAND, AUTO_LAND, AUTO_LAND, AUTO_LAND,
                                        AUTO_KILLED };
  const autoCmdType_t stops[] = { AUTO_CMD_LAND, AUTO_CMD_DISARM, AUTO_CMD_KILL };
  for (int w = 0; w < AT_N; w++) {
    for (int k = 0; k < 3; k++) {
      rig_t r;
      rigInit(&r, 37000);
      CHECK(goTo(&r, (where_t)w), "reach %s", whereName[w]);
      const autoState_t before = r.out.state;
      const autoFsmOut_t o = doCmd(&r, cmdOf(stops[k]));
      CHECK(o.cmdDone && o.cmdResult == AUTO_R_OK && o.cmdDetail == (uint32_t)before, "%s in %s: OK",
            stops[k] == AUTO_CMD_LAND ? "LAND" : stops[k] == AUTO_CMD_DISARM ? "DISARM" : "KILL", whereName[w]);
      const autoState_t want = stops[k] == AUTO_CMD_KILL ? (w == AT_IDLE ? AUTO_IDLE : AUTO_KILLED) : afterLand[w];
      CHECK(o.state == want, "  -> %s (got %s)", autoStateName(want), autoStateName(o.state));
    }
    if (w != AT_IDLE) {
      rig_t r;
      rigInit(&r, 37000);
      CHECK(goTo(&r, (where_t)w), "reach %s", whereName[w]);
      const autoState_t before = r.out.state;
      autoFsmOut_t o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
      CHECK(o.cmdResult == AUTO_R_BAD_STATE && o.state == before, "ARM in %s: BAD_STATE", whereName[w]);
      if (w != AT_ARMED) {
        o = doCmd(&r, cmdTakeoff(TOKEN, 30, 2, 150));
        CHECK(o.cmdResult == AUTO_R_BAD_STATE && o.state == r.out.state, "TAKEOFF in %s: BAD_STATE", whereName[w]);
      }
    }
  }
  /* PHOTO outside HOVER: BAD_STATE, nothing changes; in HOVER (2026-10-15: before the wait too) OK */
  for (int w = 0; w < AT_N; w++) {
    rig_t r;
    rigInit(&r, 37000);
    CHECK(goTo(&r, (where_t)w), "reach %s", whereName[w]);
    const autoState_t before = r.out.state;
    autoCmd_t ph = cmdOf(AUTO_CMD_PHOTO);
    ph.photoN = 1;
    ph.photoGapMs = 150;
    const autoFsmOut_t o = doCmd(&r, ph);
    if (w == AT_HOVER) {
      CHECK(o.cmdDone && o.cmdResult == AUTO_R_OK && o.captureStart && o.state == before, "PHOTO in %s: OK",
            whereName[w]);
    } else {
      CHECK(o.cmdDone && o.cmdResult == AUTO_R_BAD_STATE && !o.captureStart && o.state == before,
            "PHOTO in %s: BAD_STATE", whereName[w]);
    }
  }
  /* MOVE and YAW outside the hover wait: BAD_STATE NOT_WAITING; STOP: OK everywhere */
  for (int w = 0; w < AT_N; w++) {
    for (int k = 0; k < 3; k++) {
      rig_t r;
      rigInit(&r, 37000);
      r.in.sceneTracked = true;
      r.in.sceneDM = 2.0f;
      CHECK(goTo(&r, (where_t)w), "reach %s", whereName[w]);
      const autoState_t before = r.out.state;
      autoCmd_t c = cmdOf(k == 0 ? AUTO_CMD_MOVE : k == 1 ? AUTO_CMD_YAW : AUTO_CMD_STOP);
      c.moveRightCm = 30;
      c.yawDeg = 20;
      const autoFsmOut_t o = doCmd(&r, c);
      if (k == 2) {
        CHECK(o.cmdDone && o.cmdResult == AUTO_R_OK && (o.cmdDetail & 0xFF) == (uint32_t)before &&
              o.mv.state == AUTO_MV_IDLE, "STOP in %s: OK, nothing ran", whereName[w]);
      } else {
        CHECK(o.cmdDone && o.cmdResult == AUTO_R_BAD_STATE && (o.cmdDetail & 0xFF) == (uint32_t)before &&
              ((o.cmdDetail >> 8) & 0xFF) == AUTO_MVW_NOT_WAITING && o.mv.state == AUTO_MV_IDLE &&
              o.mv.epoch == 0, "%s in %s: BAD_STATE NOT_WAITING", k == 0 ? "MOVE" : "YAW", whereName[w]);
      }
      CHECK(o.state == before, "  state unchanged (%s)", autoStateName(o.state));
    }
  }
  rig_t r;
  rigInit(&r, 37000);
  run(&r, 10);
  autoFsmOut_t o = doCmd(&r, cmdTakeoff(TOKEN, 30, 2, 150));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && o.state == AUTO_IDLE, "TAKEOFF in IDLE: BAD_STATE");
  o = doCmd(&r, cmdOf((autoCmdType_t)42));
  CHECK(o.cmdDone && o.cmdResult == AUTO_R_RANGE && o.state == AUTO_IDLE, "unknown command: RANGE");
  run(&r, 3);
  CHECK(!r.out.cmdDone, "no command, no result");
}

/* ---- names ---------------------------------------------------------------------- */

static void testNames(void)
{
  bool ok = true;
  for (int s = AUTO_IDLE; s <= AUTO_KILLED; s++) {
    ok &= strcmp(autoStateName((autoState_t)s), "?") != 0;
  }
  for (int r = AUTO_RS_NONE; r <= AUTO_RS_RESUME; r++) {
    ok &= strcmp(autoReasonName((autoReason_t)r), "?") != 0;
  }
  for (int e = AUTO_EV_NONE; e <= AUTO_EV_VIEW_BACK; e++) {
    ok &= strcmp(autoEventName((autoEvent_t)e), "?") != 0;
  }
  CHECK(ok, "every state, reason and event has a name");
  CHECK(AUTO_RS_SINK == 44 && AUTO_RS_RESUME == 45 && strcmp(autoReasonName(AUTO_RS_SINK), "SINK") == 0 &&
        AUTO_EV_CAM_LOST == 1 && AUTO_EV_CONTACT == 5 && AUTO_EV_VIEW_BACK == 14 &&
        strcmp(autoEventName(AUTO_EV_RETRACK_UNDO), "RETRACK_UNDO") == 0 && strcmp(autoEventName((autoEvent_t)99), "?") == 0,
        "the 2026-10-15 values on the wire: SINK 44, RESUME 45, events 0..14");
  CHECK(strcmp(autoStateName((autoState_t)9), "?") == 0 && strcmp(autoReasonName((autoReason_t)99), "?") == 0,
        "unknown values give ?");
  CHECK(strcmp(autoReasonName(AUTO_RS_TOUCHDOWN_TRIP), "TOUCHDOWN_TRIP") == 0 &&
        strcmp(autoReasonName(AUTO_RS_HOVER_TIMEOUT), "HOVER_TIMEOUT") == 0 &&
        strcmp(autoReasonName(AUTO_RS_HOVER_NO_CAM), "HOVER_NO_CAM") == 0 &&
        strcmp(autoReasonName(AUTO_RS_BATT_LOW), "BATT_LOW") == 0 &&
        strcmp(autoReasonName(AUTO_RS_MOVE_NO_CAM), "MOVE_NO_CAM") == 0 &&
        strcmp(autoReasonName(AUTO_RS_MOVE_TILT), "MOVE_TILT") == 0 &&
        strcmp(autoReasonName(AUTO_RS_COLLISION), "COLLISION") == 0 && strcmp(autoReasonName(AUTO_RS_PALM), "PALM") == 0 &&
        AUTO_RS_PALM == 43 && VZ_R_TOUCHDOWN_SUPPORT == 28, "names match the enum (PALM 43; vz TOUCHDOWN_SUPPORT 28)");
}


/* ---- interactive hover: MOVE, YAW, STOP (auto_motion.h) -------------------- */


static autoCmd_t cmdMove(int16_t right, int16_t fwd, int16_t up)
{
  autoCmd_t c = cmdOf(AUTO_CMD_MOVE);
  c.moveRightCm = right;
  c.moveFwdCm = fwd;
  c.moveUpCm = up;
  return c;
}

static autoCmd_t cmdYaw(int16_t deg)
{
  autoCmd_t c = cmdOf(AUTO_CMD_YAW);
  c.yawDeg = deg;
  return c;
}

static uint8_t mvWhyOf(const autoFsmOut_t *o)
{
  return (uint8_t)((o->cmdDetail >> 8) & 0xFF);
}

/* Waiting in FLIGHT with a tracked scene D m ahead and the given wait. */
static bool toMoveReady(rig_t *r, float d, uint32_t waitMs)
{
  r->in.sceneTracked = true;
  r->in.sceneDM = d;
  r->in.hoverWaitNvsMs = waitMs;
  return toWait(r, AUTO_MODE_FLIGHT);
}

/* Steps until the motion is no longer active (or the sortie left HOVER). */
static long untilMotionEnd(rig_t *r, uint32_t maxMs)
{
  for (uint32_t i = 1; i <= maxMs; i++) {
    rigStep(r);
    const autoMvState_t s = r->out.mv.state;
    if (r->out.state != AUTO_HOVER || !(s == AUTO_MV_RUN || s == AUTO_MV_PAUSED || s == AUTO_MV_SETTLE)) {
      return (long)i;
    }
  }
  return -1;
}

static void testMotionAccept(void)
{
  rig_t r;
  /* before the wait (STABLE photos not done yet): NOT_WAITING */
  rigInit(&r, 51000);
  r.in.sceneTracked = true;
  r.in.sceneDM = 2.0f;
  CHECK(toHover(&r, AUTO_MODE_FLIGHT, 2, 0), "hover");
  autoFsmOut_t o = doCmd(&r, cmdMove(30, 0, 0));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_NOT_WAITING && o.mv.state == AUTO_MV_IDLE &&
        o.mv.epoch == 0, "MOVE in HOVER before the first photos: BAD_STATE NOT_WAITING");

  /* in the wait: accepted, the wait held, BUSY for another, STOP */
  rigInit(&r, 51000);
  CHECK(toMoveReady(&r, 2.0f, 10000), "waiting, D 2.0 m");
  run(&r, 3000);
  CHECK(r.out.waitLeftMs == 0 && !autoFsmReport(&r.f)->lastCmdAny, "3 s into the wait (no countdown, no command yet)");
  o = doCmd(&r, cmdMove(50, 0, 0));
  CHECK(o.cmdResult == AUTO_R_OK && mvWhyOf(&o) == 0 && (o.cmdDetail >> 16) == 1 && o.mv.state == AUTO_MV_RUN &&
        o.mv.kind == AUTO_MVK_MOVE && o.mv.epoch == 1 && !o.mv.snap && autoFsmReport(&r.f)->lastCmdAny &&
        autoFsmReport(&r.f)->lastCmdMs == r.in.nowMs - 1u,
        "MOVE right 50: OK, seq 1, RUN, epoch 1, Claude's latest command time noted");
  run(&r, 2000);
  CHECK(r.out.mv.state == AUTO_MV_RUN && r.out.mv.refLat > 0.0f &&
        r.out.mv.rateLat > 0.0f && r.out.mv.ffRollDeg > 0.0f && r.out.mv.refFwd == 0.0f && r.out.mv.vRef == 0.0f,
        "running: the wait held, lateral reference right (%.4f, rate %.4f, ff %.2f deg)", r.out.mv.refLat,
        r.out.mv.rateLat, r.out.mv.ffRollDeg);
  o = doCmd(&r, cmdMove(0, 20, 0));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_BUSY && o.mv.epoch == 1,
        "a second MOVE while one runs: BAD_STATE BUSY (never BUSY: the Mac would resend)");
  o = doCmd(&r, cmdYaw(30));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_BUSY, "YAW while a MOVE runs: BUSY");
  const float latAtStop = r.out.mv.refLat;
  o = doCmd(&r, cmdOf(AUTO_CMD_STOP));
  CHECK(o.cmdResult == AUTO_R_OK && mvWhyOf(&o) == AUTO_MV_RUN && o.mv.state == AUTO_MV_STOPPED &&
        o.mv.end == AUTO_MVE_STOP_CMD && o.mv.epoch == 2 && o.mv.snap && o.mv.refLat == 0.0f &&
        o.mv.rateLat == 0.0f && o.mv.ffRollDeg < -0.5f,
        "STOP: OK (found RUN), STOPPED, snap epoch 2, references 0, a braking lean left (%.2f deg) (ref was %.4f)",
        o.mv.ffRollDeg, latAtStop);
  run(&r, 2000);
  CHECK(r.out.mv.ffRollDeg == 0.0f, "the brake ends (v / %.2f m/s^2)", (double)AUTO_MV_BRAKE_ACC);
  CHECK(r.out.mv.state == AUTO_MV_STOPPED && r.out.state == AUTO_HOVER, "stopped, hovering");
  o = doCmd(&r, cmdOf(AUTO_CMD_STOP));
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.epoch == 2, "STOP with nothing running: OK, no new epoch");

  /* a move runs to DONE: progress = command, the wait counts from its end */
  rigInit(&r, 51000);
  CHECK(toMoveReady(&r, 2.0f, 8000), "waiting");
  o = doCmd(&r, cmdMove(-40, 30, 0));
  CHECK(o.cmdResult == AUTO_R_OK, "MOVE left 40 forward 30: OK");
  const long t = untilMotionEnd(&r, 20000);
  CHECK(t > 0 && r.out.mv.state == AUTO_MV_DONE && r.out.mv.end == AUTO_MVE_DONE && r.out.state == AUTO_HOVER &&
        fabsf(r.out.mv.progRightM + 0.40f) < 0.01f && fabsf(r.out.mv.progFwdM - 0.30f) < 0.01f &&
        r.out.mv.tgtRightM == -0.40f && r.out.mv.tgtFwdM == 0.30f,
        "DONE after %ld ms: progress right %.3f fwd %.3f (tracked)", t, r.out.mv.progRightM, r.out.mv.progFwdM);
  CHECK(fabsf(r.out.mv.refLat - (-0.40f / 2.0f)) < 1e-5f && fabsf(r.out.mv.refFwd - logf(2.0f / 1.7f)) < 1e-5f &&
        r.out.mv.rateLat == 0.0f && !r.out.mv.snap, "DONE: the hold keeps the end of the reference (%.4f, %.4f)",
        r.out.mv.refLat, r.out.mv.refFwd);

  /* the D after a forward move: the measured one, smaller than the observer's */
  CHECK(fabsf(r.out.mv.sceneDM - 1.70f) < 0.01f && (r.out.mv.flags & AUTO_MVF_D_TRACK),
        "the next command's D: 1.70 m from the forward move (%.3f, flags 0x%x)", r.out.mv.sceneDM, r.out.mv.flags);
  const long tl = untilState(&r, AUTO_LAND, 9000);
  CHECK(tl < 0 && r.out.state == AUTO_HOVER, "9 s after the motion's end: still HOVER (no HOVER_TIMEOUT)");
}

static void testMotionRefuse(void)
{
  rig_t r;
  rigInit(&r, 52000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  const struct { int16_t r, f, u; uint8_t why; autoResult_t res; } rr[] = {
    { 151, 0, 0, AUTO_MVW_RANGE, AUTO_R_RANGE }, { 0, -151, 0, AUTO_MVW_RANGE, AUTO_R_RANGE },
    { 0, 0, 51, AUTO_MVW_RANGE, AUTO_R_RANGE }, { 0, 0, 0, AUTO_MVW_RANGE, AUTO_R_RANGE },
    { 0, 0, -10, AUTO_MVW_HEIGHT, AUTO_R_RANGE },   /* 0.30 m take-off: 0.20 m < 0.25 */
    { 0, 401, 0, AUTO_MVW_RANGE, AUTO_R_RANGE }, { 150, 371, 0, AUTO_MVW_RANGE, AUTO_R_RANGE },   /* 401.2 cm */
    { 0, 151, 10, AUTO_MVW_RANGE, AUTO_R_RANGE },                                         /* up with forward > 150 */
    { 0, 151, 0, AUTO_MVW_TOO_CLOSE, AUTO_R_RANGE },   /* in range since 2026-10-14; the scene at D 2.0 refuses it */
  };
  for (size_t i = 0; i < sizeof(rr) / sizeof(rr[0]); i++) {
    const autoFsmOut_t o = doCmd(&r, cmdMove(rr[i].r, rr[i].f, rr[i].u));
    CHECK(o.cmdResult == rr[i].res && mvWhyOf(&o) == rr[i].why && o.mv.state == AUTO_MV_IDLE && o.mv.epoch == 0 &&
          o.mv.why == rr[i].why, "MOVE %d %d %d: %s (%s)", rr[i].r, rr[i].f, rr[i].u, autoMvWhyName(rr[i].why),
          autoMvWhyName((autoMvWhy_t)mvWhyOf(&o)));
  }
  r.in.sceneDM = 1.6f;
  /* the floor at the expansion scale k 0.6: 1.6 ((1.6 - F) / 1.6)^(1/0.6) >= mvfloor 0.30 m (0.50 m and 80 cm until
   * 2026-10-17, auto_motion.h "guard margins") */
  autoFsmOut_t o = doCmd(&r, cmdMove(0, 102, 0));
  CHECK(o.cmdResult == AUTO_R_RANGE && mvWhyOf(&o) == AUTO_MVW_TOO_CLOSE, "forward 102 with D 1.6 (0.295 m at k 0.6): "
        "TOO_CLOSE");
  o = doCmd(&r, cmdMove(0, 101, 0));
  CHECK(o.cmdResult == AUTO_R_OK, "forward 101 with D 1.6 (0.303 m at k 0.6): OK");
  r.in.sceneDM = 2.0f;
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_STOP)).cmdResult == AUTO_R_OK, "STOP");
  o = doCmd(&r, cmdYaw(181));
  CHECK(o.cmdResult == AUTO_R_RANGE && mvWhyOf(&o) == AUTO_MVW_RANGE, "YAW 181: RANGE");
  o = doCmd(&r, cmdYaw(0));
  CHECK(o.cmdResult == AUTO_R_RANGE && mvWhyOf(&o) == AUTO_MVW_RANGE, "YAW 0: RANGE");
  /* no D, stale view */
  r.in.sceneTracked = false;
  o = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(o.cmdResult == AUTO_R_CAM_UNAVAIL && mvWhyOf(&o) == AUTO_MVW_NO_DIST, "no tracked D: CAM_UNAVAIL NO_DIST");
  o = doCmd(&r, cmdMove(0, 0, 10));
  CHECK(o.cmdResult == AUTO_R_OK, "a vertical move needs no D: OK");
  CHECK(untilMotionEnd(&r, 10000) > 0 && r.out.mv.state == AUTO_MV_DONE, "done");
  r.in.sceneTracked = true;
  r.in.sceneDM = 9.0f;
  o = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(mvWhyOf(&o) == AUTO_MVW_NO_DIST, "D 9 m (beyond 6 m): NO_DIST");
  r.in.sceneDM = 2.0f;
  r.in.holdApplied = false;
  o = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(o.cmdResult == AUTO_R_CAM_UNAVAIL && mvWhyOf(&o) == AUTO_MVW_NO_VIEW, "the hold not applied: NO_VIEW");
  r.in.holdApplied = true;
  r.in.holdCoasting = true;
  o = doCmd(&r, cmdYaw(120));
  CHECK(mvWhyOf(&o) == AUTO_MVW_NO_VIEW, "the hold coasting: YAW 120 NO_VIEW (a blind turn is at most 90 deg)");
  r.in.holdCoasting = false;
  r.in.camFrameAgeMs = AUTO_MV_VIEW_MAX_AGE_MS + 1;
  o = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(mvWhyOf(&o) == AUTO_MVW_NO_VIEW, "a frame above the age limit: NO_VIEW");
  r.in.camFrameAgeMs = 30;
  /* battery low: refused once held AUTO_VBAT_LOW_REFUSE_MS (before the AUTO_VBAT_LOW_LAND_MS LAND) */
  r.in.vbat = AUTO_VBAT_LOW_LAND_V - 0.05f;
  run(&r, AUTO_VBAT_LOW_REFUSE_MS + 1);
  o = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_BATT && o.state == AUTO_HOVER,
        "battery low for %u ms: LOW_BATT BATT", AUTO_VBAT_LOW_REFUSE_MS + 1);
  r.in.vbat = 4.0f;
  run(&r, 5);
  /* the height after a vertical move: the cap at most (commanded and estimated; 2.00 m since 2026-10-15, 1.20 m
   * before) */
  rigInit(&r, 52000);
  r.in.sceneTracked = true;
  r.in.sceneDM = 2.0f;
  r.in.hoverWaitNvsMs = 30000;
  prepMode(&r, AUTO_MODE_FLIGHT);
  o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  run(&r, 50);
  o = doCmd(&r, cmdTakeoff(o.cmdDetail, 120, 1, 300));
  CHECK(o.cmdResult == AUTO_R_OK, "TAKEOFF 120 cm (the take-off cap)");
  CHECK(untilState(&r, AUTO_HOVER, 10000) > 0, "hover");
  while (!r.out.waiting && r.out.state == AUTO_HOVER) {
    rigStep(&r);
  }
  /* up to 1.80 m by two moves (each |up| <= 0.50 m) */
  o = doCmd(&r, cmdMove(0, 0, 50));
  CHECK(o.cmdResult == AUTO_R_OK && untilMotionEnd(&r, 10000) > 0 && r.out.mv.state == AUTO_MV_DONE,
        "1.20 m + 50 cm: OK, DONE");
  o = doCmd(&r, cmdMove(0, 0, 10));
  CHECK(o.cmdResult == AUTO_R_OK && untilMotionEnd(&r, 10000) > 0 && fabsf(r.out.mv.hOff - 0.60f) < 1e-5f,
        "+ 10 cm: OK, DONE at 1.80 m (offset %.3f)", r.out.mv.hOff);
  o = doCmd(&r, cmdMove(0, 0, 21));
  CHECK(mvWhyOf(&o) == AUTO_MVW_HEIGHT, "1.80 m + 21 cm: HEIGHT");
  o = doCmd(&r, cmdMove(0, 0, 20));
  CHECK(o.cmdResult == AUTO_R_OK, "1.80 m + 20 cm: OK (2.00 m)");
  CHECK(untilMotionEnd(&r, 10000) > 0 && r.out.mv.state == AUTO_MV_DONE && fabsf(r.out.mv.hOff - 0.80f) < 1e-5f,
        "DONE, height offset +0.80 m (%.3f)", r.out.mv.hOff);
  o = doCmd(&r, cmdMove(0, 0, 1));
  CHECK(mvWhyOf(&o) == AUTO_MVW_HEIGHT, "2.00 m + 1 cm: HEIGHT");
  /* the vertical move needs the camera height estimate (FLIGHT) */
  r.vz.noCam = true;
  rigStep(&r);
  o = doCmd(&r, cmdMove(0, 0, -20));
  CHECK(mvWhyOf(&o) == AUTO_MVW_NO_CAM_HEIGHT || o.state == AUTO_LAND,
        "no camera height: NO_CAM_HEIGHT (or the HOVER_NO_CAM LAND)");
}

static void testMotionGuards(void)
{
  rig_t r;
  /* the view lost while moving: PAUSED, the reference held; back within 2 s: resumes */
  rigInit(&r, 53000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdMove(0, 60, 0)).cmdResult == AUTO_R_OK, "MOVE forward 60");
  run(&r, 1500);
  r.in.holdCoasting = true;
  run(&r, 20);
  const float f0 = r.out.mv.refFwd;
  run(&r, 1500);
  CHECK(r.out.mv.state == AUTO_MV_PAUSED && r.out.mv.refFwd == f0 && r.out.mv.rateFwd == 0.0f &&
        r.out.mv.ffPitchDeg == 0.0f && r.out.state == AUTO_HOVER,
        "coasting 1.5 s: PAUSED, the reference held (%.4f)", f0);
  r.in.holdCoasting = false;
  run(&r, 20);
  CHECK(r.out.mv.state == AUTO_MV_RUN && r.out.mv.refFwd > f0, "the view back: RUN again");
  CHECK(untilMotionEnd(&r, 20000) > 0 && r.out.mv.state == AUTO_MV_DONE, "and DONE");
  /* ... 2 s: STOPPED NO_VIEW, the hold snapped, reported, hovering (LAND MOVE_NO_CAM until 2026-10-15) */
  rigInit(&r, 53000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdMove(50, 0, 0)).cmdResult == AUTO_R_OK, "MOVE right 50");
  run(&r, 1000);
  r.in.camFrameAgeMs = AUTO_MV_VIEW_MAX_AGE_MS + 100;
  const long t = untilEvt(&r, AUTO_EV_MOVE_STOPPED, 3000);
  rigStep(&r);
  CHECK(t == AUTO_MV_NO_CAM_LAND_MS + 1 && r.out.state == AUTO_HOVER && lastEvtDetail(&r) == AUTO_MVE_NO_VIEW &&
        r.out.mv.state == AUTO_MV_STOPPED && r.out.mv.end == AUTO_MVE_NO_VIEW && r.out.mv.snap &&
        r.out.mv.refLat == 0.0f, "frames above the age limit for 2 s: STOPPED NO_VIEW, the hold snapped, MOVE_STOPPED, HOVER "
        "(%ld)", t);
  CHECK(untilEvt(&r, AUTO_EV_HOLD_NO_VIEW, 1000) > 0 && (autoFsmReport(&r.f)->holdBits & AUTO_HB_NO_VIEW) &&
            lastEvtDetail(&r) == 3, "then, no motion, the view still lost: HOLD_NO_VIEW (detail 3: no fresh frame)");
  r.in.camFrameAgeMs = 30;
  rigStep(&r);
  CHECK(lastEvt(&r) == AUTO_EV_VIEW_BACK && !(autoFsmReport(&r.f)->holdBits & AUTO_HB_NO_VIEW), "VIEW_BACK");
  /* tilt over 8 deg for 100 ms in a turn: the turn ends where it is, its dwell and bob run on (LAND MOVE_TILT until
   * 2026-10-15) */
  rigInit(&r, 53000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdYaw(-40)).cmdResult == AUTO_R_OK, "YAW -40");
  run(&r, 500);
  r.in.tiltDeg = 8.5f;
  const long tt = untilEvt(&r, AUTO_EV_MOVE_STOPPED, 500);
  r.in.tiltDeg = 0.5f;
  rigStep(&r);
  CHECK(tt == AUTO_MV_TILT_LAND_MS + 1 && r.out.state == AUTO_HOVER && lastEvtDetail(&r) == AUTO_MVE_TILT &&
        r.out.mv.state == AUTO_MV_SETTLE && r.out.mv.camGuessOk, "tilt 8.5 deg 100 ms in a turn: the event (TILT), the "
        "turn's dwell on (%ld)", tt);
  CHECK(r.out.mv.yawOffDeg < 0.0f && r.out.mv.yawOffDeg > -40.0f, "the turn ends where it is (%.1f deg)",
        r.out.mv.yawOffDeg);
  /* tilt in a MOVE: STOPPED TILT */
  rigInit(&r, 53000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdMove(0, 60, 0)).cmdResult == AUTO_R_OK, "MOVE forward 60");
  run(&r, 500);
  r.in.tiltDeg = 8.5f;
  CHECK(untilEvt(&r, AUTO_EV_MOVE_STOPPED, 500) == AUTO_MV_TILT_LAND_MS + 1, "the event");
  r.in.tiltDeg = 0.5f;
  rigStep(&r);
  CHECK(r.out.state == AUTO_HOVER && r.out.mv.state == AUTO_MV_STOPPED && r.out.mv.end == AUTO_MVE_TILT,
        "tilt in a MOVE: STOPPED TILT, HOVER");
  /* no motion: the same tilt changes nothing (the trips are the FSM's) */
  rigInit(&r, 53000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  r.in.tiltDeg = 8.5f;
  run(&r, 1000);
  CHECK(r.out.state == AUTO_HOVER, "no motion, tilt 8.5 deg: hovering");
  /* progress far behind (the camera stops measuring): STOP PROGRESS, hold */
  rigInit(&r, 53000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  r.camFrozen = true;
  CHECK(doCmd(&r, cmdMove(100, 0, 0)).cmdResult == AUTO_R_OK, "MOVE right 100");
  const long tp = untilMotionEnd(&r, 20000);
  CHECK(tp > 0 && r.out.state == AUTO_HOVER && r.out.mv.state == AUTO_MV_STOPPED &&
        r.out.mv.end == AUTO_MVE_PROGRESS && r.out.mv.snap,
        "no measured progress: STOPPED PROGRESS after %ld ms, snap, still hovering", tp);
  /* DRY: no camera guards, ideal progress, no thrust */
  rigInit(&r, 53000);
  r.in.holdApplied = false;
  r.in.hoverWaitNvsMs = 30000;
  CHECK(toWait(&r, AUTO_MODE_DRY), "DRY waiting");
  autoFsmOut_t o = doCmd(&r, cmdMove(50, -30, 10));
  CHECK(o.cmdResult == AUTO_R_OK && (o.mv.flags & AUTO_MVF_D_SIM) && fabsf(o.mv.sceneDM - AUTO_MV_SIM_D_M) < 1e-6f,
        "DRY: MOVE accepted on the sim's nominal D (%.2f)", o.mv.sceneDM);
  float maxThr = 0.0f, maxCap = 0.0f;
  for (int i = 0; i < 15000 && r.out.mv.state != AUTO_MV_DONE; i++) {
    rigStep(&r);
    maxThr = fmaxf(maxThr, r.applied);
    maxCap = fmaxf(maxCap, r.out.thrustMax);
  }
  CHECK(r.out.mv.state == AUTO_MV_DONE && fabsf(r.out.mv.progRightM - 0.5f) < 0.01f && maxThr == 0.0f &&
        maxCap == 0.0f, "DRY: DONE (progress %.3f m), thrust and its clamp 0 throughout", r.out.mv.progRightM);
}

/* An in-air contact during a MOVE, a vertical MOVE or a YAW (2026-10-15; LAND COLLISION until then): CONTACT, a MOVE
 * STOPPED CONTACT on that step (a snap, a brake against its speed, no rate, no vRef), a YAW turned back to its start
 * heading (the same seq, RETRACK_UNDO); no LAND, the hold not blocked. */
static void testCollisionMotion(void)
{
  static const struct { int16_t r, f, u, yaw; const char *what; } m[] = {
    { 50, 0, 0, 0, "MOVE right 50" }, { 0, 60, 0, 0, "MOVE forward 60" }, { 0, 0, 30, 0, "MOVE up 30" },
    { 0, 0, 0, 60, "YAW 60" },
  };
  for (unsigned i = 0; i < sizeof(m) / sizeof(m[0]); i++) {
    rig_t r;
    rigInit(&r, 55000);
    CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
    const autoFsmOut_t o = doCmd(&r, m[i].yaw ? cmdYaw(m[i].yaw) : cmdMove(m[i].r, m[i].f, m[i].u));
    CHECK(o.cmdResult == AUTO_R_OK, "%s accepted", m[i].what);
    run(&r, 700);
    CHECK(r.out.mv.state == AUTO_MV_RUN && (r.out.mv.rateLat != 0.0f || r.out.mv.rateFwd != 0.0f ||
                                            r.out.mv.vRef != 0.0f || r.out.mv.yawOffDeg != 0.0f),
          "%s running", m[i].what);
    const float yaw0 = r.out.mv.yawOffDeg;
    const uint16_t seq = r.out.mv.seq;
    CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush && r.out.state == AUTO_HOVER,
          "%s: CONTACT, HOVER", m[i].what);
    uint8_t d = 0;
    CHECK(hadEvt(&r, AUTO_EV_CONTACT, &d) && (d & AUTO_CONTACT_MOTION), "%s: the event says a motion was stopped",
          m[i].what);
    if (m[i].yaw) {
      CHECK(hadEvt(&r, AUTO_EV_RETRACK_UNDO, &d) && d == AUTO_MVE_CONTACT && (r.out.mv.flags & AUTO_MVF_UNDO) &&
            r.out.mv.seq == seq && r.out.mv.state == AUTO_MV_RUN, "%s: turning back (UNDO, the same seq)", m[i].what);
      run(&r, 3000);
      CHECK(r.out.state == AUTO_HOVER && fabsf(r.out.mv.yawOffDeg) < 0.5f && !r.holdBlock,
            "%s: back at the start heading (%.1f -> %.1f deg)", m[i].what, yaw0, r.out.mv.yawOffDeg);
    } else {
      CHECK(r.out.mv.state == AUTO_MV_STOPPED && r.out.mv.end == AUTO_MVE_CONTACT && r.out.mv.snap &&
            r.out.mv.rateLat == 0.0f && r.out.mv.rateFwd == 0.0f && r.out.mv.vRef == 0.0f,
            "%s: the motion stopped on the contact's step (%s/%s), a snap, no rate or vRef", m[i].what,
            autoMvStateName(r.out.mv.state), autoMvEndName(r.out.mv.end));
      run(&r, 500);
      CHECK(r.out.state == AUTO_HOVER && r.out.mv.state == AUTO_MV_STOPPED && r.out.mv.vRef == 0.0f && !r.holdBlock,
            "%s: 500 ms later still stopped, hovering", m[i].what);
    }
  }
  /* a contact with no motion: a snap epoch (the hold re-anchors where the contact left it) */
  rig_t r;
  rigInit(&r, 55000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  const uint32_t e0 = r.out.mv.epoch;
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush && r.out.mv.epoch == e0 + 1 && r.out.mv.snap,
        "a contact without a motion: a snap epoch (%u -> %u)", (unsigned)e0, (unsigned)r.out.mv.epoch);
}

static void testLateYawRecoveryAndRealLoss(void)
{
  rig_t r;
  rigInit(&r, 54000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "late reset hover");
  run(&r, 1500);
  for (int i=0;i<3;i++) {
    CHECK(doCmd(&r, cmdYaw(-6)).cmdResult==AUTO_R_OK, "small YAW accepted");
    CHECK(untilMotionEnd(&r, 5000)>0 && r.out.mv.end==AUTO_MVE_DONE, "small YAW completed");
  }
  run(&r, 3000);
  r.in.sceneTracked=false; r.in.sceneResets++;
  r.vz.noCam=true;
  rigStep(&r);
  CHECK(r.out.state==AUTO_HOVER && r.out.mv.camGuessOk && (r.out.mv.flags&AUTO_MVF_BOB),
        "late reset reopens bounded recovery while in HOVER");
  for (int i=0;i<7000;i++) {
    /* Glue permits a guess only if recent accepted frames and no alarm.
     * Model that existing permission, not an unconditional healthy override. */
    if (i==1200) { r.in.sceneTracked=true; r.in.sceneDM=1.5f; }
    r.vz.noCam=!(r.in.sceneTracked || r.out.mv.camGuessOk);
    rigStep(&r);
  }
  CHECK(r.out.state==AUTO_HOVER && r.out.mv.end==AUTO_MVE_DONE && !hadEvt(&r,AUTO_EV_SINK,NULL),
        "new scale recovery avoids SINK without another heading turn");
  CHECK(fabsf(r.out.mv.yawOffDeg+18)<1e-4f, "recovery keeps heading");
  /* A genuine outage still follows the unchanged five-second fallback. */
  r.vz.noCam=true;
  r.in.camBits=AUTO_CB_NO_FRAMES;
  const long lost=untilState(&r,AUTO_LAND,6000);
  CHECK(lost>0 && lost<=AUTO_CAM_LOST_MAX_MS+2 && r.out.reason==AUTO_RS_SINK,
        "genuine outage still lands, %ldms",lost);
}

static void testMotionYaw(void)
{
  rig_t r;
  rigInit(&r, 54000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  autoFsmOut_t o = doCmd(&r, cmdYaw(90));
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.kind == AUTO_MVK_YAW && o.mv.epoch == 1 && !o.mv.snap && o.mv.camGuessOk &&
        (o.mv.flags & AUTO_MVF_D_STALE) && o.mv.sceneDM == 0.0f, "YAW 90: OK, epoch 1, D stale, the camera guess "
        "allowed");
  float maxRate = 0.0f, prev = 0.0f;
  bool mono = true;
  for (int i = 0; i < 8000 && o.mv.state == AUTO_MV_RUN; i++) {
    rigStep(&r);
    const float y = r.out.mv.yawOffDeg;
    maxRate = fmaxf(maxRate, (y - prev) * 1000.0f);
    mono = mono && y >= prev;
    prev = y;
    o = r.out;
  }
  CHECK(mono && maxRate <= AUTO_MV_YAW_RATE_DPS + 0.1f && fabsf(r.out.mv.yawOffDeg - 90.0f) < 1e-3f,
        "the yaw offset ramps to +90 (clockwise) at <= %.0f deg/s (max %.2f)", (double)AUTO_MV_YAW_RATE_DPS, maxRate);
  CHECK(r.out.mv.state == AUTO_MV_SETTLE && r.out.mv.epoch == 2 && r.out.mv.snap,
        "the turn done: SETTLE, the hold re-captured (snap epoch 2)");
  /* the tracker lost its G with the view: the bob runs until it has one again */
  r.in.sceneTracked = false;
  run(&r, AUTO_MV_YAW_DWELL_MS + 5);
  CHECK(r.out.mv.state == AUTO_MV_SETTLE && (r.out.mv.flags & AUTO_MVF_BOB) && r.out.mv.camGuessOk,
        "no G after the dwell: the re-track bob");
  float vmax = 0.0f, vmin = 0.0f;
  for (int i = 0; i < 2500; i++) {
    rigStep(&r);
    vmax = fmaxf(vmax, r.out.mv.vRef);
    vmin = fminf(vmin, r.out.mv.vRef);
  }
  CHECK(vmax > 0.05f && vmin < -0.05f && vmax <= AUTO_MV_UP_VREF_MAX && r.out.mv.state == AUTO_MV_SETTLE,
        "the bob moves up and down (vRef %.3f..%.3f m/s)", vmin, vmax);
  r.in.sceneTracked = true;
  r.in.sceneDM = 1.4f;
  const long t = untilMotionEnd(&r, 6000);
  CHECK(t > 0 && r.out.mv.state == AUTO_MV_DONE && r.out.mv.end == AUTO_MVE_DONE && !r.out.mv.camGuessOk &&
        r.out.mv.vRef == 0.0f && (r.out.mv.flags & AUTO_MVF_D_VALID) && fabsf(r.out.mv.sceneDM - 1.4f) < 1e-6f &&
        fabsf(r.out.mv.yawOffDeg - 90.0f) < 1e-3f, "G back: DONE at the end of a bob cycle, D 1.4 m valid again "
        "(t %ld state %s end %s guess %d vRef %.3f flags 0x%x D %.2f yaw %.2f)", t, autoMvStateName(r.out.mv.state),
        autoMvEndName(r.out.mv.end), r.out.mv.camGuessOk, r.out.mv.vRef, r.out.mv.flags, r.out.mv.sceneDM,
        r.out.mv.yawOffDeg);
  /* a turn back past 180: the offset stays in -180..180 */
  o = doCmd(&r, cmdYaw(180));
  CHECK(o.cmdResult == AUTO_R_OK, "YAW 180");
  CHECK(untilMotionEnd(&r, 20000) > 0 && fabsf(r.out.mv.yawOffDeg - (-90.0f)) < 1e-3f,
        "90 + 180 = -90 deg (%.2f)", r.out.mv.yawOffDeg);
  /* no G after the longest bob: DONE RETRACK (the HOVER_NO_CAM rule decides next) */
  rigInit(&r, 54000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdYaw(-30)).cmdResult == AUTO_R_OK, "YAW -30");
  r.in.sceneTracked = false;
  const long t2 = untilMotionEnd(&r, 30000);
  CHECK(t2 > 0 && r.out.mv.state == AUTO_MV_DONE && r.out.mv.end == AUTO_MVE_RETRACK && !r.out.mv.camGuessOk &&
        (r.out.mv.flags & AUTO_MVF_D_STALE), "never re-tracked: DONE RETRACK after %ld ms, D stale", t2);
  /* STOP during a turn: the turn ends there, the bob still runs; a second STOP does not end it either */
  rigInit(&r, 54000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdYaw(60)).cmdResult == AUTO_R_OK, "YAW 60");
  run(&r, 1000);   /* at 30 deg/s (60 deg/s^2): 22.5 deg into the 2.5 s turn */
  o = doCmd(&r, cmdOf(AUTO_CMD_STOP));
  const float yStop = o.mv.yawOffDeg;
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.state == AUTO_MV_SETTLE && yStop > 10.0f && yStop < 40.0f && o.mv.snap,
        "STOP in the turn: it ends at %.1f deg, SETTLE (the re-track check), snap", yStop);
  r.in.sceneTracked = false;
  run(&r, AUTO_MV_YAW_DWELL_MS + 300);
  CHECK(r.out.mv.flags & AUTO_MVF_BOB, "the bob runs");
  o = doCmd(&r, cmdOf(AUTO_CMD_STOP));
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.state == AUTO_MV_SETTLE && (o.mv.flags & AUTO_MVF_BOB) && o.mv.camGuessOk &&
        fabsf(o.mv.yawOffDeg - yStop) < 1e-4f && o.state == AUTO_HOVER,
        "a STOP in the bob: OK, it runs on with the guess (ending it would land HOVER_NO_CAM)");
  run(&r, 1000);
  CHECK(r.out.state == AUTO_HOVER && (r.out.mv.flags & AUTO_MVF_BOB), "still hovering, bobbing");
  r.in.sceneTracked = true;
  CHECK(untilMotionEnd(&r, 10000) > 0 && r.out.state == AUTO_HOVER && r.out.mv.state == AUTO_MV_DONE &&
        r.out.mv.end == AUTO_MVE_DONE, "the tracker back: DONE, hovering");
  /* LAND during a turn: the offset kept for the landing */
  rigInit(&r, 54000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdYaw(-60)).cmdResult == AUTO_R_OK, "YAW -60");
  run(&r, 2000);
  o = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  const float yLand = o.mv.yawOffDeg;
  CHECK(o.state == AUTO_LAND && o.mv.state == AUTO_MV_STOPPED && o.mv.end == AUTO_MVE_LEFT_HOVER && yLand < -10.0f,
        "LAND in the turn: STOPPED LEFT_HOVER at %.1f deg", yLand);
  run(&r, 500);
  CHECK(r.out.mv.yawOffDeg == yLand && r.out.mv.refLat == 0.0f && r.out.mv.vRef == 0.0f,
        "the heading stays there through the LAND");
  /* the air time: never a refusal since 2026-10-15 (docs/autonomy-a2b.md 22; AIR refused a YAW 180 at 100 s) */
  rigInit(&r, 54000);
  CHECK(toMoveReady(&r, 2.0f, 60000), "waiting 60 s");
  while (r.out.state == AUTO_HOVER && r.out.airMs < 100000u) {
    rigStep(&r);
  }
  CHECK(r.out.state == AUTO_HOVER && r.out.airMs == 100000, "hovering at 100 s of air (%u)", (unsigned)r.out.airMs);
  o = doCmd(&r, cmdYaw(180));
  CHECK(o.cmdResult == AUTO_R_OK, "100 s of air: YAW 180 accepted (no AIR)");
  o = doCmd(&r, cmdOf(AUTO_CMD_STOP));
  CHECK(o.cmdResult == AUTO_R_OK && o.state == AUTO_HOVER, "STOP: hovering");
  while (r.out.state == AUTO_HOVER && r.out.airMs < 106000u) {
    rigStep(&r);
  }
  o = doCmd(&r, cmdMove(10, 0, 0));
  CHECK(o.cmdResult == AUTO_R_OK, "106 s of air: a MOVE accepted");
}


/* After a MOVE that finished (DONE), leaving HOVER keeps the hold's reference
 * where the move left it: same epoch, same end reference, no snap. A drop to 0
 * inside the epoch would be taken off the hold's integral, and the drone would
 * fly back toward the move's start while landing (591f176's verification). */
static void testLandAfterDoneMove(void)
{
  static const struct { int16_t r, f; autoCmdType_t land; } cs[] = {
    {80, 0, AUTO_CMD_LAND}, {0, -50, AUTO_CMD_LAND}, {-60, 40, AUTO_CMD_LAND}, {50, 0, AUTO_CMD_NONE},
  };
  for (size_t k = 0; k < sizeof(cs) / sizeof(cs[0]); k++) {
    rig_t r;
    rigInit(&r, 55000);
    CHECK(toMoveReady(&r, 2.0f, 6000), "waiting");
    CHECK(doCmd(&r, cmdMove(cs[k].r, cs[k].f, 0)).cmdResult == AUTO_R_OK, "MOVE %d %d", cs[k].r, cs[k].f);
    CHECK(untilMotionEnd(&r, 30000) > 0 && r.out.mv.state == AUTO_MV_DONE, "DONE");
    run(&r, 200);
    const autoMvOut_t before = r.out.mv;
    autoFsmOut_t o;
    if (cs[k].land == AUTO_CMD_LAND) {
      o = doCmd(&r, cmdOf(AUTO_CMD_LAND));
    } else {
      r.in.vbat = AUTO_VBAT_LOW_LAND_V - 0.05f;   /* an automatic LAND: BATT_LOW (HOVER_TIMEOUT until 2026-10-15) */
      untilState(&r, AUTO_LAND, 10000);
      r.in.vbat = 4.05f;
      o = r.out;
    }
    bool same = true;
    for (int i = 0; i < 1500 && r.out.state == AUTO_LAND; i++) {
      rigStep(&r);
      same = same && r.out.mv.epoch == before.epoch && r.out.mv.snapEpoch == before.snapEpoch &&
             r.out.mv.refLat == before.refLat && r.out.mv.refFwd == before.refFwd && r.out.mv.rateLat == 0.0f &&
             r.out.mv.rateFwd == 0.0f && r.out.mv.ffRollDeg == 0.0f && r.out.mv.ffPitchDeg == 0.0f;
    }
    CHECK(o.state == AUTO_LAND && (before.refLat != 0.0f || before.refFwd != 0.0f) && o.mv.epoch == before.epoch &&
          o.mv.snapEpoch == before.snapEpoch && o.mv.refLat == before.refLat && o.mv.refFwd == before.refFwd && same,
          "MOVE %d %d DONE, then LAND (%s): the hold's epoch %u and end reference (%.4f, %.4f) kept through it",
          cs[k].r, cs[k].f, autoReasonName(o.reason), (unsigned)before.epoch, before.refLat, before.refFwd);
    /* back on the ground the reference is 0, so a later manual flight's hold
     * does not fly the move again (final check of 7ff8965) */
    const bool idle = untilState(&r, AUTO_IDLE, 20000) > 0;
    bool zero = idle;
    for (int i = 0; i < 3000 && idle; i++) {
      rigStep(&r);
      zero = zero && r.out.state == AUTO_IDLE && r.out.mv.refLat == 0.0f && r.out.mv.refFwd == 0.0f;
    }
    CHECK(idle && zero, "MOVE %d %d: IDLE after the landing, the hold's reference 0 for 3 s (idle %d)", cs[k].r,
          cs[k].f, idle);
  }
  /* the two height-offset limits agree (vert_ctrl clamps hOff; auto_motion commits it), and hold the commanded range
   * plus a SET_HEIGHT's transient (2026-10-15) */
  CHECK(VZ_HOFF_MAX == AUTO_MV_HOFF_MAX &&
        AUTO_MV_HOFF_MAX >= AUTO_MV_H_MAX_M - (float)AUTO_HEIGHT_CM_MIN * 0.01f + AUTO_SH_MAX_M - 1e-6f &&
        AUTO_MV_HOFF_MAX >= VZ_H_TARGET_MAX - VZ_H_TARGET_MIN + AUTO_SH_MAX_M - 1e-6f,
        "VZ_HOFF_MAX %.2f == AUTO_MV_HOFF_MAX %.2f >= cap %.2f - 0.20 + 0.50", (double)VZ_HOFF_MAX,
        (double)AUTO_MV_HOFF_MAX, (double)AUTO_MV_H_MAX_M);
}

/* LAND from 1.20 m and 2.00 m: the backstop grows by the extra height at 0.30 m/s (8500 and 11167 ms; vert_ctrl's
 * LAND timeout from there 8.0 and 10.67 s). 2.00 m: the 1.20 m take-off cap moved up 0.50 + 0.30 m. */
static void testLandBackstopHigh(void)
{
  for (int k = 0; k < 3; k++) {
    rig_t r;
    rigInit(&r, 55000);
    r.vz.neverDone = true;
    const uint16_t to = k == 0 ? 60 : 120, hcm = k == 0 ? 60 : k == 1 ? 120 : 200;
    prepMode(&r, AUTO_MODE_FLIGHT);
    autoFsmOut_t o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
    run(&r, 50);
    o = doCmd(&r, cmdTakeoff(o.cmdDetail, to, 1, 300));
    CHECK(untilState(&r, AUTO_HOVER, 8000) > 0, "hover at %u cm", to);
    if (hcm > to) {
      r.in.sceneTracked = true;
      r.in.sceneDM = 2.0f;
      while (!r.out.waiting && r.out.state == AUTO_HOVER) {
        rigStep(&r);
      }
      for (int m = 0; m < 2; m++) {
        o = doCmd(&r, cmdMove(0, 0, m == 0 ? 50 : 30));
        CHECK(o.cmdResult == AUTO_R_OK && untilMotionEnd(&r, 10000) > 0 && r.out.mv.state == AUTO_MV_DONE,
              "%u cm: MOVE up %d cm DONE", to, m == 0 ? 50 : 30);
      }
      CHECK(fabsf(r.out.mv.hOff - 0.80f) < 1e-5f, "moved up to 2.00 m (offset %.3f)", r.out.mv.hOff);
    }
    o = doCmd(&r, cmdOf(AUTO_CMD_LAND));
    const long t = untilState(&r, AUTO_KILLED, 15000);
    const long want = AUTO_LAND_BACKSTOP_MS + (hcm > 60 ? (long)((hcm - 60) * 1000.0 / 30.0 + 0.5) : 0);
    CHECK(t == want && r.out.reason == AUTO_RS_LAND_BACKSTOP, "%u cm, the controller never done: KILLED "
          "LAND_BACKSTOP %ld ms into LAND (want %ld)", hcm, t, want);
  }
}

/* The fast LAND's allow list (auto_fsm.h autoFsmLandFastReason(), docs/autonomy-a2b.md 17.4): exactly the Mac's
 * LAND, the air clock, the hover wait's end and the sortie done; never a battery rule, a loop gap, a lost link or
 * app, a guard or a contact. */
static void testLandFastReasons(void)
{
  int allowed = 0;
  for (int r = 0; r <= AUTO_RS_COLLISION; r++) {
    const bool want = r == AUTO_RS_LAND_CMD || r == AUTO_RS_AIR_TIME || r == AUTO_RS_HOVER_TIMEOUT ||
                      r == AUTO_RS_SORTIE_DONE;
    CHECK(autoFsmLandFastReason((autoReason_t)r) == want, "fast LAND allowed for %s: %d", autoReasonName((autoReason_t)r),
          want);
    allowed += autoFsmLandFastReason((autoReason_t)r);
  }
  CHECK(allowed == 4 && !autoFsmLandFastReason(AUTO_RS_VBAT_DIP) && !autoFsmLandFastReason(AUTO_RS_VBAT_LOW) &&
        !autoFsmLandFastReason(AUTO_RS_BATT_LOW) && !autoFsmLandFastReason(AUTO_RS_LOOP_GAP) &&
        !autoFsmLandFastReason(AUTO_RS_HB_LOST) && !autoFsmLandFastReason(AUTO_RS_APP_SILENT) &&
        !autoFsmLandFastReason(AUTO_RS_COLLISION), "four reasons; never VBAT_DIP, VBAT_LOW, BATT_LOW, LOOP_GAP, "
        "HB_LOST, APP_SILENT, COLLISION");
}


/* ---- the TAKEOFF's planned move (2026-10-14, docs/autonomy-a2b.md 21.4) ------------------------------------------ */

/* ARM, then a TAKEOFF with a plan (right, fwd cm), photoN photos, this sortie's wait (0: NVS). */
static autoFsmOut_t planTakeoff(rig_t *r, uint8_t photoN, int16_t right, int16_t fwd, uint32_t actWaitMs)
{
  prepMode(r, AUTO_MODE_FLIGHT);
  const autoFsmOut_t a = doCmd(r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(a.cmdResult == AUTO_R_OK, "plan: ARM");
  run(r, 50);
  autoCmd_t c = cmdTakeoff(a.cmdDetail, 100, photoN, 150);
  c.moveRightCm = right;
  c.moveFwdCm = fwd;
  c.actWaitMs = actWaitMs;
  return doCmd(r, c);
}

/* Steps until the wait starts (out.waiting); the step count, or -1. */
static long untilWaiting(rig_t *r, uint32_t maxMs)
{
  for (uint32_t i = 1; i <= maxMs; i++) {
    rigStep(r);
    if (r->out.waiting) {
      return (long)i;
    }
    if (r->out.state != AUTO_TAKEOFF && r->out.state != AUTO_HOVER) {
      return -1;
    }
  }
  return -1;
}

static void testPlanTakeoff(void)
{
  rig_t r;
  /* latched at TAKEOFF; the RANGE rules */
  rigInit(&r, 61000);
  r.in.sceneTracked = true;
  r.in.sceneDM = 4.0f;
  r.in.hoverWaitNvsMs = 0;   /* NVS autowait 0: a plan needs this sortie's wait */
  const struct { int16_t r, f; uint32_t wait; autoResult_t res; const char *what; } tk[] = {
    {0, 401, 30000, AUTO_R_RANGE, "forward 401"}, {151, 0, 30000, AUTO_R_RANGE, "right 151"},
    {-151, 100, 30000, AUTO_R_RANGE, "left 151"}, {0, -151, 30000, AUTO_R_RANGE, "back 151"},
    {150, 371, 30000, AUTO_R_RANGE, "right 150 forward 371 (401.2 cm)"},
  };
  for (unsigned i = 0; i < sizeof tk / sizeof tk[0]; i++) {
    rigInit(&r, 61000);
    r.in.hoverWaitNvsMs = 0;
    const autoFsmOut_t o = planTakeoff(&r, 1, tk[i].r, tk[i].f, tk[i].wait);
    CHECK(o.cmdResult == tk[i].res && o.state == AUTO_ARMED && o.cmdDetail == AUTO_ARMED && !r.f.planPending,
          "TAKEOFF with a plan, %s: RANGE (detail the state, ARMED; no take-off)", tk[i].what);
  }
  rigInit(&r, 61000);
  r.in.hoverWaitNvsMs = 0;
  autoFsmOut_t o = planTakeoff(&r, 1, 0, 0, 0);
  CHECK(o.cmdResult == AUTO_R_OK && o.state == AUTO_TAKEOFF && !r.f.planPending, "no plan, no wait: OK (as before)");
  rigInit(&r, 61000);
  r.in.hoverWaitNvsMs = 0;
  o = planTakeoff(&r, 1, 50, 200, 0);
  CHECK(o.cmdResult == AUTO_R_OK && o.state == AUTO_TAKEOFF && r.f.planPending,
        "a plan with no wait (actWaitMs 0, NVS autowait 0): OK since 2026-10-15 (the wait always starts)");
  rigInit(&r, 61000);
  r.in.hoverWaitNvsMs = 5000;
  o = planTakeoff(&r, 1, 150, 370, 0);
  CHECK(o.cmdResult == AUTO_R_OK && r.f.planPending && r.f.planRightCm == 150 && r.f.planFwdCm == 370,
        "right 150 forward 370 (398.6 cm) with the NVS wait: OK, latched");
  rigInit(&r, 61000);
  r.in.hoverWaitNvsMs = 0;
  o = planTakeoff(&r, 1, -40, -150, 20000);
  CHECK(o.cmdResult == AUTO_R_OK && r.f.planPending && r.f.planRightCm == -40 && r.f.planFwdCm == -150,
        "left 40 back 150 with this sortie's wait 20 s: OK, latched");

  /* the plan starts on the step the wait starts (row 26), for photoN 0 and 3, and runs to DONE */
  static const uint8_t pn[2] = {0, 3};
  for (int k = 0; k < 2; k++) {
    rigInit(&r, 62000);
    r.in.sceneTracked = true;
    r.in.sceneDM = 4.0f;
    r.in.hoverWaitNvsMs = 30000;
    o = planTakeoff(&r, pn[k], 60, 150, 0);
    CHECK(o.cmdResult == AUTO_R_OK && r.f.planPending, "photoN %u: TAKEOFF with a plan OK", pn[k]);
    const long w = untilWaiting(&r, 10000);
    CHECK(w > 0 && r.out.state == AUTO_HOVER && r.f.mv.seq == 1 && r.f.mv.state == AUTO_MV_RUN && !r.f.planPending &&
          r.out.mv.seq == 0, "photoN %u: the plan accepted on the wait's first step (motion 1, RUN; this step's "
          "outputs from before it)", pn[k]);
    rigStep(&r);
    CHECK(r.out.mv.seq == 1 && r.out.mv.state == AUTO_MV_RUN && r.out.mv.kind == AUTO_MVK_MOVE &&
          r.out.mv.tgtRightM == 0.60f && r.out.mv.tgtFwdM == 1.50f && r.out.waiting && r.out.waitLeftMs == 0,
          "photoN %u: the next step: AUTO_STATUS mvSeq 1, RUN, target right 0.60 forward 1.50, waiting", pn[k]);
    const long t = untilMotionEnd(&r, 30000);
    CHECK(t > 0 && r.out.state == AUTO_HOVER && r.out.mv.state == AUTO_MV_DONE && r.out.mv.end == AUTO_MVE_DONE &&
          fabsf(r.out.mv.progFwdM - 1.50f) < 0.01f && fabsf(r.out.mv.progRightM - 0.60f) < 0.01f && r.out.waiting,
          "photoN %u: the planned move DONE after %ld ms (ideal camera), the drone waits", pn[k], t);
    autoFsmOut_t m = doCmd(&r, cmdMove(0, 50, 0));
    CHECK(m.cmdResult == AUTO_R_OK && (m.cmdDetail >> 16) == 2, "photoN %u: the next Mac MOVE: motion 2", pn[k]);
  }

  /* refusals at the wait's start: the drone waits, mvWhy set, motion 0 */
  for (int k = 0; k < 5; k++) {
    rigInit(&r, 63000);
    r.in.sceneTracked = true;
    r.in.sceneDM = 2.0f;
    r.in.hoverWaitNvsMs = 30000;
    int16_t pr = 0, pf = 100;   /* forward 1.00 at D 2.0: within the scene floor (0.63 m at k 0.6) */
    autoMvWhy_t want = AUTO_MVW_NONE;
    switch (k) {
      case 0: r.in.sceneTracked = false; want = AUTO_MVW_NO_DIST; break;
      case 1: pr = 150; pf = 50; want = AUTO_MVW_TOO_WIDE; break;      /* > 0.5 x D 2.0 */
      case 2: pf = 200; want = AUTO_MVW_TOO_CLOSE; break;               /* the scene 2.0 m ahead */
      case 3: r.vz.holdThrust = AUTO_MV_REFUSE_T + 600.0f; want = AUTO_MVW_BATT; break;   /* T_hold 54000 */
      default: want = AUTO_MVW_NO_VIEW; break;                          /* the hold coasting at the wait's start */
    }
    o = planTakeoff(&r, 1, pr, pf, 0);
    CHECK(o.cmdResult == AUTO_R_OK, "refusal %s: TAKEOFF OK", autoMvWhyName(want));
    if (k == 4) {
      CHECK(untilCapture(&r, 8000), "the take-off photo started");
      r.in.holdCoasting = true;   /* (STABLE is behind; the wait needs only the photo done) */
    }
    const long w = untilWaiting(&r, 10000);
    rigStep(&r);
    CHECK(w > 0 && r.out.state == AUTO_HOVER && r.out.waiting && r.out.mv.seq == 0 && r.out.mv.why == want &&
          r.out.mv.state == AUTO_MV_IDLE, "a plan refused %s at the wait's start: hovering in the wait, mvWhy %s, "
          "motion 0 (%s)", autoMvWhyName(want), autoMvWhyName(want), autoMvWhyName(r.out.mv.why));
    r.in.holdCoasting = false;
    r.in.sceneTracked = true;
    r.vz.holdThrust = 36000.0f;
    run(&r, 300);
    autoFsmOut_t m = doCmd(&r, cmdMove(20, 0, 0));
    CHECK(r.out.state == AUTO_HOVER && m.cmdResult == AUTO_R_OK && (m.cmdDetail >> 16) == 1,
          "after the refused plan (%s): a Mac MOVE OK, motion 1", autoMvWhyName(want));
  }
  /* the plan flies the Mac's floor (AUTO_PLAN_MIN_FRONT_CM, 2026-10-15 review; 35 since 2026-10-17, 60 before): at
   * D 2.0 forward 1.30 m leaves 0.348 m at k 0.6 (inside the drone's own 0.30 m, which a Mac MOVE keeps: 1.35 m),
   * 1.29 m leaves 0.356 m */
  for (int k = 0; k < 3; k++) {
    rigInit(&r, 63500);
    r.in.sceneTracked = true;
    r.in.sceneDM = 2.0f;
    r.in.hoverWaitNvsMs = 30000;
    o = planTakeoff(&r, 1, 0, k == 0 ? 130 : (k == 1 ? 129 : 0), 0);   /* (k 2: no plan, a Mac MOVE in the wait) */
    const long w = untilWaiting(&r, 10000);
    rigStep(&r);
    if (k < 2) {
      CHECK(w > 0 && r.out.waiting && (k == 0 ? r.out.mv.seq == 0 && r.out.mv.why == AUTO_MVW_TOO_CLOSE
                                              : r.out.mv.seq == 1 && r.out.mv.state == AUTO_MV_RUN),
            "a plan forward %d at D 2.0: %s (the Mac's 0.35 m floor at k 0.6)", k == 0 ? 130 : 129,
            k == 0 ? "TOO_CLOSE" : "running");
    } else {
      run(&r, 300);
      autoFsmOut_t m = doCmd(&r, cmdMove(0, 135, 0));
      CHECK(w > 0 && m.cmdResult == AUTO_R_OK, "... a Mac MOVE forward 135 at D 2.0 keeps the drone's 0.30 m: OK");
    }
  }

  /* a STOP before the wait cancels it: in the pre-spin, the climb, and HOVER before the wait */
  for (int k = 0; k < 3; k++) {
    rigInit(&r, 64000);
    r.in.sceneTracked = true;
    r.in.sceneDM = 4.0f;
    r.in.hoverWaitNvsMs = 30000;
    o = planTakeoff(&r, 1, 0, 150, 0);
    if (k == 0) {
      run(&r, 300);   /* the pre-spin (1 s) */
    } else if (k == 1) {
      while (r.out.state == AUTO_TAKEOFF && !(r.vz.o.phase == VZ_CLIMB)) {
        rigStep(&r);
      }
    } else {
      CHECK(untilState(&r, AUTO_HOVER, 8000) > 0, "HOVER");
      run(&r, 200);
    }
    const autoState_t st = r.out.state;
    o = doCmd(&r, cmdOf(AUTO_CMD_STOP));
    CHECK(o.cmdResult == AUTO_R_OK && !r.f.planPending, "STOP in %s before the wait: OK, the plan cancelled",
          k == 0 ? "the pre-spin" : k == 1 ? "the climb" : "HOVER");
    const long w = untilWaiting(&r, 10000);
    run(&r, 2);
    CHECK(w > 0 && st != AUTO_IDLE && r.out.waiting && r.out.mv.seq == 0 && r.out.mv.why == AUTO_MVW_NONE &&
          r.out.mv.state == AUTO_MV_IDLE, "... the wait starts without a move");
  }
  /* LAND or KILL before the wait: never flown; the next TAKEOFF without a plan clears it */
  for (int k = 0; k < 2; k++) {
    rigInit(&r, 65000);
    r.in.sceneTracked = true;
    r.in.sceneDM = 4.0f;
    r.in.hoverWaitNvsMs = 30000;
    o = planTakeoff(&r, 1, 0, 150, 0);
    CHECK(untilState(&r, AUTO_HOVER, 8000) > 0, "HOVER");
    o = doCmd(&r, cmdOf(k ? AUTO_CMD_KILL : AUTO_CMD_LAND));
    bool moved = false;
    for (int i = 0; i < 12000 && r.out.state != AUTO_IDLE; i++) {
      rigStep(&r);
      moved = moved || r.out.mv.seq != 0 || r.out.waiting;
    }
    CHECK(r.out.state == AUTO_IDLE && !moved && r.f.mv.seq == 0, "%s before the wait: IDLE, the plan never flown",
          k ? "KILL" : "LAND");
    run(&r, 2500);   /* the kill latch, the soak */
    r.in.onGroundMs = 120000;
    o = planTakeoff(&r, 1, 0, 0, 0);
    CHECK(o.cmdResult == AUTO_R_OK && !r.f.planPending, "the next TAKEOFF without a plan: the latch cleared");
    const long w = untilWaiting(&r, 10000);
    run(&r, 2);
    CHECK(w > 0 && r.out.mv.seq == 0 && r.out.mv.state == AUTO_MV_IDLE, "... its wait starts without a move");
  }
  /* a Mac MOVE before the wait: NOT_WAITING; during the plan: BUSY; STOP ends the plan STOP_CMD */
  rigInit(&r, 66000);
  r.in.sceneTracked = true;
  r.in.sceneDM = 4.0f;
  r.in.hoverWaitNvsMs = 30000;
  o = planTakeoff(&r, 1, 0, 200, 0);
  CHECK(untilState(&r, AUTO_HOVER, 8000) > 0, "HOVER");
  o = doCmd(&r, cmdMove(30, 0, 0));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_NOT_WAITING && r.f.planPending,
        "a Mac MOVE before the wait: NOT_WAITING, the plan still pending");
  CHECK(untilWaiting(&r, 10000) > 0, "the wait");
  run(&r, 1000);
  o = doCmd(&r, cmdMove(30, 0, 0));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_BUSY && r.out.mv.seq == 1,
        "a Mac MOVE during the plan: BUSY");
  o = doCmd(&r, cmdOf(AUTO_CMD_STOP));
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.state == AUTO_MV_STOPPED && o.mv.end == AUTO_MVE_STOP_CMD && o.waiting,
        "STOP during the plan: STOPPED STOP_CMD, waiting");

  /* An event before the wait cancels the plan (2026-10-15 verification of "Claude decides"): each of these landed
   * until then, so the plan never flew after it; now the drone hovers in the wait (mvWhy PLAN_CANCELLED, motion 0)
   * and Claude decides from a photo. A contact in the climb (the camera height in use: CONTACT_HOLD) and in HOVER
   * 0.3 s before the wait, NOT_STABLE, a climb fault that ended in HOLD (CLIMB_TIMEOUT), a controller event, a loop
   * gap, CAPTURE_FAIL, TAKEOFF_SLOW. */
  static const char *const ev[] = {"a CONTACT in the climb", "a CONTACT 0.3 s into HOVER", "NOT_STABLE",
                                   "a CLIMB_TIMEOUT HOLD", "a VZ K_RANGE event", "a loop gap", "CAPTURE_FAIL",
                                   "TAKEOFF_SLOW"};
  for (int k = 0; k < 8; k++) {
    rigInit(&r, 67000);
    r.in.sceneTracked = true;
    r.in.sceneDM = 4.0f;
    r.in.hoverWaitNvsMs = 30000;
    if (k == 2) {
      r.vz.neverSteady = true;
    } else if (k == 3) {
      r.vz.climbEnd = VZ_R_CLIMB_TIMEOUT;
    } else if (k == 6) {
      r.camAuto = false;   /* the photo set never completes: CAPTURE_FAIL at n x gap + AUTO_CAPTURE_EXTRA_MS */
    } else if (k == 7) {
      r.camAuto = false;
      r.vz.neverSteady = true;   /* no STABLE, no photo: NOT_STABLE first (its event cancels it too) ... */
    }
    o = planTakeoff(&r, k == 7 ? 0 : 1, 0, 150, 0);
    CHECK(o.cmdResult == AUTO_R_OK && r.f.planPending, "%s: TAKEOFF with a plan forward 150 OK", ev[k]);
    if (k == 0) {
      while (r.out.state == AUTO_TAKEOFF && !(r.vz.o.phase == VZ_CLIMB && r.vz.t > 300)) {
        rigStep(&r);
      }
      CHECK(holdImu(&r, 4, 2.0f, 0.5f, 0.0f, AUTO_TAKEOFF) == 0 && hadEvt(&r, AUTO_EV_CONTACT, NULL) &&
                !r.f.planPending,
            "a contact in the climb (the camera height in use): CONTACT, no LAND, the plan cancelled");
    } else if (k == 1) {
      CHECK(untilState(&r, AUTO_HOVER, 8000) > 0, "HOVER");
      run(&r, 300);
      CHECK(!r.out.waiting && r.f.planPending, "HOVER 0.3 s, not yet waiting, the plan pending");
      holdImu(&r, 4, 2.0f, 0.5f, 0.0f, AUTO_HOVER);
      CHECK(hadEvt(&r, AUTO_EV_CONTACT, NULL) && !r.f.planPending, "a contact before the wait: the plan cancelled");
    } else if (k == 4) {
      CHECK(untilState(&r, AUTO_HOVER, 8000) > 0, "HOVER");
      r.vz.o.evt = (uint8_t)VZ_R_K_RANGE;
      r.vz.o.evtN++;
      run(&r, 2);
      CHECK(hadEvt(&r, AUTO_EV_VZ, NULL) && !r.f.planPending, "a controller event before the wait: the plan cancelled");
    } else if (k == 5) {
      CHECK(untilState(&r, AUTO_HOVER, 8000) > 0, "HOVER");
      r.in.loopGapUs = 25000;
      rigStep(&r);
      r.in.loopGapUs = 1000;
      CHECK(hadEvt(&r, AUTO_EV_LOOP_GAP, NULL) && !r.f.planPending && r.out.state == AUTO_HOVER,
            "a loop gap before the wait: the event, the plan cancelled");
    }
    const long w = untilWaiting(&r, 20000);
    bool flew = false;
    for (int i = 0; i < 3000 && r.out.state == AUTO_HOVER; i++) {
      rigStep(&r);
      flew = flew || r.f.mv.seq != 0 || autoMotionActive(&r.f.mv);
    }
    CHECK(w > 0 && r.out.state == AUTO_HOVER && r.out.waiting && !flew && r.out.mv.seq == 0 &&
              r.out.mv.state == AUTO_MV_IDLE && r.out.mv.why == AUTO_MVW_PLAN_CANCELLED && !r.f.planPending,
          "%s before the wait: the wait starts, the planned 1.5 m MOVE never starts (mvWhy %s), the drone hovers",
          ev[k], autoMvWhyName(r.out.mv.why));
    if (k == 6) {
      CHECK(hadEvt(&r, AUTO_EV_CAPTURE_FAIL, NULL), "... the event CAPTURE_FAIL");
    } else if (k == 3) {
      CHECK(r.f.faultHover, "... HOVER from the climb fault's HOLD (faultHover)");
    } else if (k == 2 || k == 7) {
      CHECK(hadEvt(&r, AUTO_EV_NOT_STABLE, NULL) || hadEvt(&r, AUTO_EV_TAKEOFF_SLOW, NULL),
            "... the event NOT_STABLE or TAKEOFF_SLOW");
    }
    r.vz.neverSteady = false;
    r.in.capture = AUTO_CAP_NONE;
    r.camAuto = true;
    run(&r, 300);
    autoFsmOut_t m = doCmd(&r, cmdMove(0, 50, 0));
    CHECK(r.out.state == AUTO_HOVER && m.cmdResult == AUTO_R_OK && (m.cmdDetail >> 16) == 1,
          "%s: Claude's own MOVE after it is accepted (motion 1)", ev[k]);
  }
}

/* ---- "Claude decides": the report (2026-10-15, docs/autonomy-a2b.md 22) --------------------------------------- */

static void testReport(void)
{
  rig_t r;
  /* the controller's events forwarded (AUTO_EV_VZ, detail its reason); the latest three kept, the count */
  rigInit(&r, 71000);
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  const uint16_t n0 = autoFsmReport(&r.f)->evCount;
  r.vz.o.evt = (uint8_t)VZ_R_K_RANGE;
  r.vz.o.evtN++;
  rigStep(&r);
  rigStep(&r);
  CHECK(lastEvt(&r) == AUTO_EV_VZ && lastEvtDetail(&r) == VZ_R_K_RANGE && autoFsmReport(&r.f)->evCount == n0 + 1u,
        "a controller event: VZ K_RANGE forwarded once");
  r.vz.o.evt = (uint8_t)VZ_R_EST_RANGE;
  r.vz.o.evtN++;
  run(&r, 2);
  r.in.loopGapUs = 30000;
  rigStep(&r);
  r.in.loopGapUs = 1000;
  const autoFsmReport_t *p = autoFsmReport(&r.f);
  CHECK(p->evCode[0] == AUTO_EV_LOOP_GAP && p->evCode[1] == AUTO_EV_VZ && p->evDetail[1] == VZ_R_EST_RANGE &&
            p->evCode[2] == AUTO_EV_VZ && p->evDetail[2] == VZ_R_K_RANGE && p->evCount == n0 + 3u &&
            p->evMs[0] == r.in.nowMs - 1u && p->evMs[1] < p->evMs[0],
        "the latest three: LOOP_GAP, VZ EST_RANGE, VZ K_RANGE; the count +3, their times");
  /* a turn whose re-track finds no G: RETRACK_UNDO, back to its start heading, UNDONE when it tracks there */
  rigInit(&r, 72000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdYaw(45)).cmdResult == AUTO_R_OK, "YAW 45");
  r.in.sceneTracked = false;
  long k = untilEvt(&r, AUTO_EV_RETRACK_UNDO, 30000);
  CHECK(k > 0 && lastEvtDetail(&r) == AUTO_MVE_RETRACK && (r.out.mv.flags & AUTO_MVF_UNDO) && r.out.state == AUTO_HOVER,
        "the bob ends without a G: RETRACK_UNDO (%ld ms), turning back", k);
  r.in.sceneTracked = true;
  CHECK(untilMotionEnd(&r, 20000) > 0 && r.out.mv.end == AUTO_MVE_UNDONE && fabsf(r.out.mv.yawOffDeg) < 1e-3f,
        "back at 0 deg and tracked: DONE UNDONE");
  /* a loop gap in a YAW's turn: the turn ends where it is, its dwell on (the camera height guess kept) */
  rigInit(&r, 73000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting");
  CHECK(doCmd(&r, cmdYaw(90)).cmdResult == AUTO_R_OK, "YAW 90");
  run(&r, 1500);
  r.in.loopGapUs = 25000;
  rigStep(&r);
  r.in.loopGapUs = 1000;
  rigStep(&r);
  CHECK(r.out.state == AUTO_HOVER && r.out.mv.state == AUTO_MV_SETTLE && r.out.mv.camGuessOk &&
            r.out.mv.yawOffDeg > 10.0f && r.out.mv.yawOffDeg < 80.0f,
        "a loop gap in the turn: it ends at %.1f deg, the dwell on, the guess allowed", r.out.mv.yawOffDeg);
  /* a 600 s DRY hover, no command: no LAND (the battery ends a real flight) */
  rigInit(&r, 74000);
  r.vz.noCam = true;
  CHECK(toWait(&r, AUTO_MODE_DRY), "DRY waiting");
  CHECK(untilState(&r, AUTO_LAND, 600000) < 0 && r.out.state == AUTO_HOVER && r.out.airMs > 600000,
        "600 s of DRY hover without a command: no LAND (air %u ms)", (unsigned)r.out.airMs);
  /* the report is cleared at the next ARM */
  r.in.cmd = cmdOf(AUTO_CMD_LAND);
  CHECK(toIdle(&r, 8000), "landed");
  run(&r, 3000);
  r.in.appGestureAgeMs = 3000;   /* (G10's gesture, long expired after 600 s) */
  const autoFsmOut_t ao = doCmd(&r, cmdArm(AUTO_MODE_DRY, 0));
  CHECK(ao.state == AUTO_ARMED && autoFsmReport(&r.f)->evCount == 0 && autoFsmReport(&r.f)->holdBits == 0,
        "ARM clears the report (%s, result %d detail 0x%x, events %u, bits 0x%x)", autoStateName(ao.state),
        (int)ao.cmdResult, (unsigned)ao.cmdDetail, (unsigned)autoFsmReport(&r.f)->evCount,
        (unsigned)autoFsmReport(&r.f)->holdBits);
}

/* ---- every LAND after a contact is the blind one (user decision (B) 2026-10-15, docs/autonomy-a2b.md 22.9) ---- */

/* The LAND triggers: each makes the state machine (or the controller) land from the hover wait. */
static void trLandCmd(rig_t *r) { r->in.cmd = cmdOf(AUTO_CMD_LAND); }
static void trDisarm(rig_t *r) { r->in.cmd = cmdOf(AUTO_CMD_DISARM); }
static void trHbLost(rig_t *r) { r->hbStream = false; }
static void trSession(rig_t *r) { r->in.sessionChanged = true; }
static void trAppInput(rig_t *r) { appPacket(r, 0.0f, 0.0f, 0.0f, 5000.0f); }
static void trAppSilent(rig_t *r) { r->appStream = false; }
static void trBattLow(rig_t *r) { r->in.vbat = AUTO_VBAT_LAND_V - 0.01f; }
static void trHeadroom(rig_t *r) { r->vz.holdThrust = AUTO_HEADROOM_LAND_T + 500.0f; }
static void trVbatDip(rig_t *r) { r->in.vbatFastMin = AUTO_VBAT_FAST_LAND_V - 0.01f; }
static void trSink(rig_t *r) { r->vz.noCam = true; }
static void trVzOwn(rig_t *r)
{
  fzSet(&r->vz, VZ_LAND, VZ_R_EST_RANGE);   /* the controller's own LAND (its runaway guard) */
  r->vz.o.landReason = VZ_R_EST_RANGE;
}
static void trLoopGap(rig_t *r)
{
  for (int g = 0; g < AUTO_LOOP_GAP_N && r->out.state == AUTO_HOVER; g++) {
    r->in.loopGapUs = 25000;
    rigStep(r);
    r->in.loopGapUs = 1000;
    run(r, 100);
  }
}

typedef struct {
  autoReason_t why;
  bool blindAtEntry, blindEveryStep, collided, holdBlocked, idle, killed;
  int landSteps;
} blindLandRes_t;

/* The hover wait, a contact 3 s before (contact) or none, then trig: the LAND's reason, the blind input on its first
 * step and on every LAND step, collided (the camera hold blocked) at its entry, the end. */
static blindLandRes_t blindLandCase(void (*trig)(rig_t *), bool contact)
{
  blindLandRes_t res;
  memset(&res, 0, sizeof(res));
  rig_t r;
  rigInit(&r, 71000);
  r.vz.neverSteady = true;
  if (!toWait(&r, AUTO_MODE_FLIGHT)) {
    printf("  blindLandCase: no wait\n");
    return res;
  }
  run(&r, 2000);   /* capQ measured: a camera loss sinks after its 5 s */
  if (contact) {
    CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush && r.out.blind && !r.out.collided,
          "a CONTACT in the wait: the hush, blind from its own step (a controller LAND would be), not collided");
    run(&r, 3000);
    CHECK(r.out.state == AUTO_HOVER && !r.f.contactBlind && r.out.blind && !r.out.collided,
          "3 s after it: HOVER, the contact's window over, still blind (out.blind), not collided");
  }
  CHECK(r.out.state == AUTO_HOVER && r.out.blind == contact, "HOVER before the LAND, out.blind %d", r.out.blind);
  trig(&r);
  for (int i = 0; i < 7000 && r.out.state == AUTO_HOVER; i++) {
    rigStep(&r);
  }
  res.why = r.out.reason;
  res.blindAtEntry = r.out.state == AUTO_LAND && r.out.blind;
  res.collided = r.out.collided;
  res.holdBlocked = r.holdBlock;
  res.blindEveryStep = true;
  for (int i = 0; i < 8000 && (r.out.state == AUTO_LAND); i++) {
    res.landSteps++;
    res.blindEveryStep = res.blindEveryStep && r.out.blind == contact && r.out.collided == res.collided;
    rigStep(&r);
  }
  res.idle = r.out.state == AUTO_IDLE;
  res.killed = r.out.state == AUTO_KILLED;
  /* the next ARM: not blind */
  if (res.idle) {
    run(&r, AUTO_LAND_REST_MAX_MS);
    r.in.onGroundMs = 30000;
    r.in.appGestureAgeMs = 2000;
    r.hbStream = r.appStream = true;
    r.in.sessionValid = true;
    r.in.vbat = 4.05f;
    run(&r, 200);
    r.in.appGestureAgeMs = 2000;
    const autoFsmOut_t a = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
    CHECK(a.state == AUTO_ARMED && !a.blind && !a.collided, "the next ARM: not blind (%s, guards 0x%x)",
          autoStateName(a.state), (unsigned)a.cmdDetail);
  }
  return res;
}

static void testBlindEveryLand(void)
{
  static const struct { void (*trig)(rig_t *); autoReason_t why; const char *name; } k[] = {
    {trLandCmd, AUTO_RS_LAND_CMD, "Claude's LAND"},
    {trDisarm, AUTO_RS_DISARM_CMD, "a DISARM (a LAND in the air)"},
    {trHbLost, AUTO_RS_HB_LOST, "the Mac link lost (HB_LOST)"},
    {trSession, AUTO_RS_SESSION, "a session change"},
    {trAppInput, AUTO_RS_APP_INPUT, "the phone's stick (APP_INPUT)"},
    {trAppSilent, AUTO_RS_APP_SILENT, "the phone silent (APP_SILENT)"},
    {trBattLow, AUTO_RS_BATT_LOW, "BATT_LOW (2.99 V)"},
    {trHeadroom, AUTO_RS_BATT_LOW, "BATT_LOW (T_hold 57500)"},
    {trVbatDip, AUTO_RS_VBAT_DIP, "VBAT_DIP"},
    {trLoopGap, AUTO_RS_LOOP_GAP, "three loop gaps (LOOP_GAP)"},
    {trSink, AUTO_RS_SINK, "the camera height out (SINK)"},
    {trVzOwn, AUTO_RS_VZ, "the controller's own LAND (EST_RANGE)"},
  };
  for (size_t i = 0; i < sizeof(k) / sizeof(k[0]); i++) {
    const blindLandRes_t a = blindLandCase(k[i].trig, true);
    CHECK(a.why == k[i].why && a.blindAtEntry && a.blindEveryStep && !a.collided && !a.holdBlocked && a.idle &&
              a.landSteps > 100,
          "after a CONTACT 3 s before, %s: LAND %s, blind from its first step and on every one of its %d steps, the hold "
          "not blocked, then IDLE (%s%s)", k[i].name, autoReasonName(a.why), a.landSteps, a.idle ? "IDLE" : "not IDLE",
          a.killed ? ", KILLED" : "");
    const blindLandRes_t b = blindLandCase(k[i].trig, false);
    CHECK(b.why == k[i].why && !b.blindAtEntry && b.blindEveryStep && !b.collided && b.idle,
          "without a contact, %s: LAND %s, never blind", k[i].name, autoReasonName(b.why));
  }

  /* a LAND in the contact's window: collided too (the hold blocked), blind */
  rig_t r;
  rigInit(&r, 71000);
  r.vz.neverSteady = true;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT");
  run(&r, 200);
  autoFsmOut_t lo = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(lo.state == AUTO_LAND && lo.blind && lo.collided && r.holdBlock,
        "Claude's LAND in the hush: blind and collided (the hold blocked), as before");
  /* COLLISION (a contact in the climb without the camera height): blind */
  rigInit(&r, 71000);
  r.vz.climbMs = 2000;
  r.vz.noCam = true;
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  for (int i = 0; i < 6000 && !r.vz.o.liftoffNow; i++) {
    rigStep(&r);
  }
  run(&r, 250);
  CHECK(holdImu(&r, 6, 4.0f, 0.0f, 0.0f, AUTO_TAKEOFF) == 4 && collisionLand(&r) && r.out.blind,
        "LAND COLLISION: blind (and collided)");
  /* a contact in the climb with the camera height (the climb ends in HOLD): every later LAND blind */
  rigInit(&r, 71000);
  r.vz.climbMs = 2000;
  r.vz.neverSteady = true;
  CHECK(launch(&r, AUTO_MODE_FLIGHT, 2, 0), "launched");
  for (int i = 0; i < 6000 && !r.vz.o.liftoffNow; i++) {
    rigStep(&r);
  }
  run(&r, 250);
  CHECK(holdImu(&r, 6, 4.0f, 0.0f, 0.0f, AUTO_TAKEOFF) == 0 && r.out.hush && r.out.blind && !r.out.collided,
        "a CONTACT in the climb with the camera height: the hush, blind, not collided");
  r.vz.o.liftedOff = true;
  fzSet(&r.vz, VZ_HOLD, VZ_R_CONTACT_HOLD);   /* the controller ends the climb in HOLD */
  r.vz.o.holdReached = true;
  CHECK(untilState(&r, AUTO_HOVER, 2000) > 0, "HOVER");
  run(&r, 5000);
  lo = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(lo.state == AUTO_LAND && lo.blind && !lo.collided, "Claude's LAND 5 s later: blind, not collided");

  /* a SINK after an old contact descends blind and resumes when the camera height is back; a later LAND blind too */
  rigInit(&r, 71000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  run(&r, 2000);
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT");
  run(&r, 3000);
  r.vz.noCam = true;
  CHECK(untilState(&r, AUTO_LAND, 7000) > 0 && r.out.reason == AUTO_RS_SINK && r.out.blind && !r.out.collided,
        "the camera height out 3 s after the contact: SINK, blind, not collided");
  run(&r, 500);
  r.vz.noCam = false;
  bool asked = false;
  long tr = -1;
  for (int i = 1; i <= 2000; i++) {
    rigStep(&r);
    asked = asked || r.out.resume;
    if (r.out.state == AUTO_HOVER) {
      tr = i;
      break;
    }
  }
  CHECK(asked && tr > 0 && r.out.reason == AUTO_RS_RESUME && r.out.blind,
        "the camera height back: the resume asked, HOVER RESUME after %ld ms, still blind", tr);
  run(&r, 1000);
  lo = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(lo.state == AUTO_LAND && lo.reason == AUTO_RS_LAND_CMD && lo.blind && !lo.collided,
        "Claude's LAND after the RESUME: blind, the hold not blocked");
  /* a SINK begun in the contact's window (no capQ: at once): collided, never asked to resume, it lands */
  rigInit(&r, 71000);
  r.vz.neverSteady = true;
  r.vz.noCapQ = true;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT");
  r.vz.noCam = true;   /* the camera height lost in the hush */
  CHECK(untilState(&r, AUTO_LAND, 2000) > 0, "SINK");
  CHECK(r.out.reason == AUTO_RS_SINK && r.out.collided && r.out.blind, "the SINK in the window: collided, blind (%s)",
        autoReasonName(r.out.reason));
  r.vz.noCam = false;
  asked = false;
  for (int i = 0; i < 3000 && r.out.state == AUTO_LAND; i++) {
    rigStep(&r);
    asked = asked || r.out.resume;
  }
  CHECK(!asked && r.out.state != AUTO_HOVER, "the camera height back: never asked to resume (%s)",
        autoStateName(r.out.state));

  /* the minimum constraints after an old contact: the battery's MOVE refusal and LAND, a flip's cut in the hover and in
   * the blind LAND, the k kill in the blind LAND; a later contact is still stopped and reported */
  rigInit(&r, 71000);
  CHECK(toMoveReady(&r, 2.0f, 30000), "waiting, the scene tracked");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT");
  run(&r, 3000);
  r.in.vbat = AUTO_VBAT_LAND_V - 0.01f;
  run(&r, AUTO_VBAT_LOW_REFUSE_MS + 5);
  autoFsmOut_t o = doCmd(&r, cmdMove(20, 0, 0));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_BATT && o.state == AUTO_HOVER,
        "the battery under 3.00 V for 255 ms after an old contact: MOVE LOW_BATT (BATT)");
  CHECK(untilState(&r, AUTO_LAND, 400) > 0 && r.out.reason == AUTO_RS_BATT_LOW && r.out.blind,
        "then LAND BATT_LOW, blind");
  rigInit(&r, 71000);
  r.vz.neverSteady = true;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0, "CONTACT");
  run(&r, 3000);
  int sTilt = 0;
  while (sTilt < AUTO_TILT_KILL_MS + 10 && r.out.state == AUTO_HOVER) {
    setTrip(&r.in, TRIP_TILT, true);
    rigStep(&r);
    sTilt++;
  }
  setTrip(&r.in, TRIP_TILT, false);
  CHECK(r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_TILT && r.applied == 0.0f,
        "a flip 3 s after a contact: KILLED TILT (step %d)", sTilt);
  rigInit(&r, 71000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0, "CONTACT");
  run(&r, 3000);
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).blind && r.out.state == AUTO_LAND, "the blind LAND");
  run(&r, 200);
  sTilt = 0;
  while (sTilt < AUTO_TILT_KILL_MS + 10 && r.out.state == AUTO_LAND) {
    setTrip(&r.in, TRIP_TILT, true);
    rigStep(&r);
    sTilt++;
  }
  setTrip(&r.in, TRIP_TILT, false);
  CHECK(r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_TILT, "a flip early in the blind LAND: KILLED TILT");
  rigInit(&r, 71000);
  r.vz.neverSteady = true;
  r.vz.neverDone = true;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0, "CONTACT");
  run(&r, 3000);
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).blind, "the blind LAND");
  run(&r, 300);
  o = doCmd(&r, cmdOf(AUTO_CMD_KILL));
  CHECK(o.state == AUTO_KILLED && o.reason == AUTO_RS_KILL_CMD && r.applied == 0.0f, "k in the blind LAND: KILLED");
  rigInit(&r, 71000);
  r.vz.neverSteady = true;
  CHECK(toWait(&r, AUTO_MODE_FLIGHT), "waiting");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0, "CONTACT");
  run(&r, 10000);
  const uint16_t n0 = autoFsmReport(&r.f)->evCount;
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush && lastEvt(&r) == AUTO_EV_CONTACT &&
            autoFsmReport(&r.f)->contacts == 2 && autoFsmReport(&r.f)->evCount == n0 + 1u && r.out.blind,
        "a second contact 10 s later: CONTACT again (the hush, the event), no LAND");
}



/* ---- SET_HEIGHT ("you are at X m", 2026-10-15, auto_fsm.h AUTO_SH_*, docs/autonomy-a2b.md 23) -------------------- */

static autoCmd_t cmdSetHeight(int16_t xCm, int16_t atCm)
{
  autoCmd_t c = cmdOf(AUTO_CMD_SET_HEIGHT);
  c.heightCm = (uint16_t)xCm;
  c.moveUpCm = atCm;
  return c;
}

/* the ACK detail's old estimate, cm (bits 16-31, i16) */
static int shOldCm(const autoFsmOut_t *o)
{
  return (int)(int16_t)(uint16_t)(o->cmdDetail >> 16);
}

/* In the wait at a height estimate h (the fake controller's; the climb leaves it ~0.28 m), D 2.0 m. */
static bool toShReady(rig_t *r, uint32_t t0, float h)
{
  rigInit(r, t0);
  if (!toMoveReady(r, 2.0f, 60000)) {
    return false;
  }
  run(r, 200);
  r->vz.o.h = h;
  return r->out.state == AUTO_HOVER && r->out.waiting && r->vz.o.phase == VZ_HOLD;
}

static void testSetHeight(void)
{
  rig_t r;
  autoFsmOut_t o;
  /* outside the wait: BAD_STATE NOT_WAITING (cmdOutside: the state, no estimate; HOVER before the wait: motionCmd, the
   * estimate) */
  for (int w = 0; w < AT_N; w++) {
    rigInit(&r, 61000);
    CHECK(goTo(&r, (where_t)w), "reach %s", whereName[w]);
    const autoState_t before = r.out.state;
    const bool hover = before == AUTO_HOVER;
    if (hover) {
      CHECK(!r.out.waiting, "HOVER before the wait");
      r.vz.o.h = 0.42f;
    }
    o = doCmd(&r, cmdSetHeight(100, 90));
    CHECK(o.cmdDone && o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_NOT_WAITING &&
          (o.cmdDetail & 0xFF) == (uint32_t)before && shOldCm(&o) == (hover ? 42 : 0) && o.hShift == 0.0f &&
          r.shiftSteps == 0 && o.mv.state != AUTO_MV_RUN,
          "SET_HEIGHT in %s: BAD_STATE NOT_WAITING (detail 0x%08x), nothing shifted", whereName[w],
          (unsigned)o.cmdDetail);
  }
  /* LAND SINK (the camera height out 5 s): NOT_WAITING */
  CHECK(toShReady(&r, 61500, 1.00f), "waiting");
  r.vz.noCam = true;
  CHECK(untilState(&r, AUTO_LAND, 8000) > 0 && r.out.reason == AUTO_RS_SINK, "LAND SINK");
  run(&r, 50);
  o = doCmd(&r, cmdSetHeight(100, 90));
  CHECK(o.state == AUTO_LAND && o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_NOT_WAITING &&
        (o.cmdDetail & 0xFF) == AUTO_LAND && o.hShift == 0.0f, "SET_HEIGHT in LAND SINK: BAD_STATE NOT_WAITING");

  /* accepted (the offset form): the shift, the reference, the correction flown back, the ACK */
  CHECK(toShReady(&r, 62000, 1.00f), "waiting at 1.00 m");
  const uint16_t ev0 = autoFsmReport(&r.f)->evCount;
  const uint16_t seq0 = r.out.mv.seq;
  o = doCmd(&r, cmdSetHeight(110, 95));   /* judged 1.10 m on a photo taken at 0.95 m: d +0.15 */
  CHECK(o.cmdResult == AUTO_R_OK && (o.cmdDetail & 0xFFFF) == AUTO_HOVER && shOldCm(&o) == 100 &&
        fabsf(o.hShift - 0.15f) < 1e-6f && r.shiftSteps == 1 && fabsf(r.vz.o.h - 1.15f) < 1e-5f,
        "X 1.10 at 0.95: OK, detail HOVER | 0 | old 100 cm (0x%08x), hShift %.3f, the controller's estimate 1.15 (%.4f)",
        (unsigned)o.cmdDetail, o.hShift, r.vz.o.h);
  CHECK(o.mv.state == AUTO_MV_RUN && o.mv.kind == AUTO_MVK_MOVE && (o.mv.flags & AUTO_MVF_SET_HEIGHT) &&
        o.mv.seq == seq0 + 1 && fabsf(o.mv.tgtUpM + 0.15f) < 1e-6f && o.mv.progUpM == 0.0f &&
        fabsf(o.mv.hOff - 0.15f) < 1e-6f && fabsf(o.mv.vRef) < 0.001f && o.mv.tgtRightM == 0.0f && o.mv.tgtFwdM == 0.0f,
        "the correction: a vertical MOVE of -0.15 (mvFlags 0x40), progress 0 on this step (the estimate shifted), the "
        "reference at old + d (hOff %.3f), no vRef jump (%.4f)", o.mv.hOff, o.mv.vRef);
  CHECK(autoFsmReport(&r.f)->evCount == ev0 && autoFsmReport(&r.f)->lastCmdAny &&
        autoFsmReport(&r.f)->lastCmdMs == r.in.nowMs - 1u, "no event (the trim learner), Claude's latest command noted");
  rigStep(&r);
  CHECK(r.out.hShift == 0.0f && r.shiftSteps == 1, "hShift on the accepting step only");
  o = doCmd(&r, cmdSetHeight(100, 100 - 20));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_BUSY && o.hShift == 0.0f,
        "a second SET_HEIGHT while the correction runs: BUSY");
  o = doCmd(&r, cmdMove(0, 0, 10));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_BUSY, "a MOVE while it runs: BUSY");
  const long te = untilMotionEnd(&r, 10000);
  CHECK(te > 0 && te < 4100 + 3000 && r.out.state == AUTO_HOVER && r.out.mv.state == AUTO_MV_DONE &&
        r.out.mv.end == AUTO_MVE_DONE && r.out.mv.hOff == 0.0f && fabsf(r.vz.o.h - 1.00f) < 0.01f &&
        autoFsmReport(&r.f)->evCount == ev0 && r.out.waiting,
        "DONE after %ld ms: the offset back to 0 exactly (%.6f), the estimate back to 1.00 m (%.3f): the drone where it "
        "believed it was; no event", te, r.out.mv.hOff, r.vz.o.h);

  /* the absolute form (h_at INT16_MIN): d = X - the estimate now */
  CHECK(toShReady(&r, 63000, 1.00f), "waiting at 1.00 m");
  o = doCmd(&r, cmdSetHeight(80, AUTO_SH_AT_NOW));
  CHECK(o.cmdResult == AUTO_R_OK && shOldCm(&o) == 100 && fabsf(o.hShift + 0.20f) < 1e-5f &&
        fabsf(o.mv.tgtUpM - 0.20f) < 1e-5f, "X 0.80 now at 1.00: d -0.20, the correction +0.20 (%.3f)", o.mv.tgtUpM);

  /* the rate bound: one accepted per 2 s (a STOP ends the correction; the bound counts accepted ones only) */
  CHECK(toShReady(&r, 64000, 1.00f), "waiting");
  CHECK(doCmd(&r, cmdSetHeight(120, 100)).cmdResult == AUTO_R_OK, "SET_HEIGHT +0.20");
  const uint32_t at = r.f.setHeightMs;
  run(&r, 300);
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_STOP)).cmdResult == AUTO_R_OK && r.out.mv.state == AUTO_MV_STOPPED, "STOP it");
  o = doCmd(&r, cmdSetHeight(9, 100));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_SH_RATE, "a refused one in the 2 s: SH_RATE first "
        "(%s)", autoMvWhyName((autoMvWhy_t)mvWhyOf(&o)));
  /* the order: the view and the camera height come before the rate bound */
  r.in.holdApplied = false;
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(mvWhyOf(&o) == AUTO_MVW_NO_VIEW, "in the 2 s without the view: NO_VIEW before SH_RATE (%s)",
        autoMvWhyName((autoMvWhy_t)mvWhyOf(&o)));
  r.in.holdApplied = true;
  r.vz.noCam = true;
  rigStep(&r);
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(mvWhyOf(&o) == AUTO_MVW_NO_CAM_HEIGHT, "in the 2 s without the camera height: NO_CAM_HEIGHT before SH_RATE "
        "(%s)", autoMvWhyName((autoMvWhy_t)mvWhyOf(&o)));
  r.vz.noCam = false;
  rigStep(&r);
  while (r.in.nowMs - at < AUTO_SH_GAP_MS - 1u) {
    rigStep(&r);
  }
  o = doCmd(&r, cmdSetHeight(100, 110));
  CHECK(o.cmdResult == AUTO_R_BAD_STATE && mvWhyOf(&o) == AUTO_MVW_SH_RATE && o.hShift == 0.0f,
        "1999 ms after the accepted one: BAD_STATE SH_RATE");
  o = doCmd(&r, cmdSetHeight(100, 110));
  CHECK(o.cmdResult == AUTO_R_OK && fabsf(o.hShift + 0.10f) < 1e-6f, "2000 ms after it: OK");

  /* the ranges, in the checks' order */
  static const struct { float h; int16_t x, at; autoMvWhy_t why; autoResult_t res; const char *what; } rc[] = {
    { 1.00f, 9, 10, AUTO_MVW_RANGE, AUTO_R_RANGE, "X 9 cm" },
    { 1.00f, 251, 250, AUTO_MVW_RANGE, AUTO_R_RANGE, "X 251 cm" },
    { 1.00f, 100, 100, AUTO_MVW_RANGE, AUTO_R_RANGE, "d 0 cm" },
    { 1.00f, 100, AUTO_SH_AT_NOW, AUTO_MVW_RANGE, AUTO_R_RANGE, "X = the estimate now (d rounds to 0)" },
    { 1.00f, 150, 100, AUTO_MVW_NONE, AUTO_R_OK, "|d| 0.50" },
    { 1.00f, 151, 100, AUTO_MVW_SH_STEP, AUTO_R_RANGE, "|d| 0.51" },
    { 1.00f, 49, 100, AUTO_MVW_SH_STEP, AUTO_R_RANGE, "|d| 0.51 down" },
    { 0.996f, 150, AUTO_SH_AT_NOW, AUTO_MVW_NONE, AUTO_R_OK, "the absolute form, d 0.504 (50 cm)" },
    { 0.994f, 150, AUTO_SH_AT_NOW, AUTO_MVW_SH_STEP, AUTO_R_RANGE, "the absolute form, d 0.506 (51 cm)" },
    { 1.004f, 50, AUTO_SH_AT_NOW, AUTO_MVW_NONE, AUTO_R_OK, "the absolute form, d -0.504 (-50 cm)" },
    { 1.006f, 50, AUTO_SH_AT_NOW, AUTO_MVW_SH_STEP, AUTO_R_RANGE, "the absolute form, d -0.506 (-51 cm)" },
    { 0.40f, 70, 100, AUTO_MVW_NONE, AUTO_R_OK, "the new estimate 0.10 m" },
    { 0.40f, 69, 100, AUTO_MVW_HEIGHT, AUTO_R_RANGE, "the new estimate 0.09 m" },
    { VZ_H_TARGET_MAX + 0.10f, 140, 100, AUTO_MVW_NONE, AUTO_R_OK, "the new estimate cap + 0.50 (upM -0.50)" },
    { VZ_H_TARGET_MAX + 0.10f, 141, 100, AUTO_MVW_HEIGHT, AUTO_R_RANGE, "the new estimate cap + 0.51 (upM -0.51)" },
    { VZ_H_TARGET_MAX + 0.10f, 146, 100, AUTO_MVW_SH_STEP, AUTO_R_RANGE, "upM -0.56 (above the cap)" },
  };
  for (size_t k = 0; k < sizeof(rc) / sizeof(rc[0]); k++) {
    CHECK(toShReady(&r, 65000 + 100u * (uint32_t)k, rc[k].h), "waiting at %.2f m", rc[k].h);
    o = doCmd(&r, cmdSetHeight(rc[k].x, rc[k].at));
    CHECK(o.cmdResult == rc[k].res && mvWhyOf(&o) == rc[k].why && (o.cmdResult == AUTO_R_OK) == (o.hShift != 0.0f) &&
          (o.cmdResult == AUTO_R_OK || (o.mv.why == rc[k].why && o.mv.state != AUTO_MV_RUN)),
          "%s (h %.2f, X %d at %d): %s %s", rc[k].what, rc[k].h, rc[k].x, rc[k].at, autoMvWhyName(rc[k].why),
          autoMvWhyName((autoMvWhy_t)mvWhyOf(&o)));
  }
  /* above the cap, d +0.40: the flight goes to the cap, not back to the estimate */
  CHECK(toShReady(&r, 66500, VZ_H_TARGET_MAX + 0.10f), "waiting above the cap");
  o = doCmd(&r, cmdSetHeight(140, 100));
  CHECK(o.cmdResult == AUTO_R_OK && fabsf(o.mv.tgtUpM + 0.50f) < 1e-5f, "h cap + 0.10, d +0.40: the flight -0.50 to the "
        "cap (%.3f)", o.mv.tgtUpM);
  CHECK(untilMotionEnd(&r, 10000) > 0 && r.out.mv.end == AUTO_MVE_DONE && fabsf(r.out.mv.hOff - (-0.10f)) < 1e-5f &&
        fabsf(r.vz.o.h - VZ_H_TARGET_MAX) < 0.01f, "DONE at the cap (%.3f), the offset old + d + upM = -0.10 (%.3f)",
        r.vz.o.h, r.out.mv.hOff);

  /* the camera height, the view, the battery */
  CHECK(toShReady(&r, 67000, 1.00f), "waiting");
  r.in.holdApplied = false;
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(o.cmdResult == AUTO_R_CAM_UNAVAIL && mvWhyOf(&o) == AUTO_MVW_NO_VIEW, "the hold not applied: NO_VIEW");
  r.in.holdApplied = true;
  r.vz.noCam = true;
  rigStep(&r);
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(o.cmdResult == AUTO_R_CAM_UNAVAIL && mvWhyOf(&o) == AUTO_MVW_NO_CAM_HEIGHT && o.hShift == 0.0f,
        "the camera height out: NO_CAM_HEIGHT (the open loop ignores vRef)");
  r.vz.noCam = false;
  r.in.holdApplied = false;
  rigStep(&r);
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(mvWhyOf(&o) == AUTO_MVW_NO_VIEW, "the camera height back, the view not: NO_VIEW");
  r.in.holdApplied = true;
  CHECK(toShReady(&r, 67500, 1.00f), "waiting");
  r.vz.holdThrust = AUTO_MV_REFUSE_T + 600.0f;   /* the hover needs more than 53400: MOVEs refused, no BATT_LOW */
  run(&r, 300);
  o = doCmd(&r, cmdSetHeight(80, 100));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_UP_HEADROOM && o.state == AUTO_HOVER,
        "the battery's end: a climbing correction (d -0.20) LOW_BATT, named UP_HEADROOM (2026-10-16)");
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_BATT && o.hShift == 0.0f && o.state == AUTO_HOVER,
        "... a descending one (d +0.20) too (its braking may end HOLD_CAPPED there, 2026-10-15 review)");

  /* a contact: its hush and its window refuse (SH_CONTACT), the camera back 500 ms after the hush accepts */
  CHECK(toShReady(&r, 68000, 1.00f), "waiting");
  CHECK(holdImu(&r, 4, 4.0f, 0.0f, 0.0f, AUTO_HOVER) == 0 && r.out.hush, "CONTACT: the hush");
  r.vz.o.h = 1.00f;
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(o.cmdResult == AUTO_R_CAM_UNAVAIL && mvWhyOf(&o) == AUTO_MVW_SH_CONTACT && o.hShift == 0.0f,
        "in the hush: CAM_UNAVAIL SH_CONTACT");
  while (r.out.hush) {
    rigStep(&r);
  }
  run(&r, 100);
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(mvWhyOf(&o) == AUTO_MVW_SH_CONTACT && r.f.contactBlind, "after the hush, the camera not back 500 ms: SH_CONTACT");
  run(&r, AUTO_CAM_BACK_MS + 10);
  o = doCmd(&r, cmdSetHeight(120, 100));
  CHECK(o.cmdResult == AUTO_R_OK && !r.f.contactBlind, "the camera back 500 ms: OK");

  /* a LAND during the correction: LAND at once, the motion LEFT_HOVER with the measured travel committed, the
   * corrected estimate kept */
  CHECK(toShReady(&r, 69000, 1.00f), "waiting");
  CHECK(doCmd(&r, cmdSetHeight(140, 100)).cmdResult == AUTO_R_OK, "SET_HEIGHT +0.40");
  run(&r, 1500);
  const float hMid = r.vz.o.h, prog = hMid - 1.40f;
  o = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(o.state == AUTO_LAND && o.reason == AUTO_RS_LAND_CMD && o.mv.state == AUTO_MV_STOPPED &&
        o.mv.end == AUTO_MVE_LEFT_HOVER && prog < -0.05f && fabsf(o.mv.hOff - (0.40f + prog)) < 1e-4f && hMid > 1.20f,
        "LAND 1.5 s into it: LAND_CMD, LEFT_HOVER, the offset old + d + travel (%.3f), the estimate corrected (%.3f)",
        o.mv.hOff, hMid);
  /* k during a correction: KILLED at once */
  CHECK(toShReady(&r, 69500, 1.00f), "waiting");
  CHECK(doCmd(&r, cmdSetHeight(70, 100)).cmdResult == AUTO_R_OK, "SET_HEIGHT -0.30");
  run(&r, 800);
  r.in.killCount++;
  rigStep(&r);
  CHECK(r.out.state == AUTO_KILLED && r.out.reason == AUTO_RS_KILL_CMD && r.out.thrust == AUTO_THR_CUT,
        "k during the correction: KILLED at once");
  /* BATT_LOW during a correction: the LAND (measured commit) */
  CHECK(toShReady(&r, 69800, 1.00f), "waiting");
  CHECK(doCmd(&r, cmdSetHeight(130, 100)).cmdResult == AUTO_R_OK, "SET_HEIGHT +0.30");
  run(&r, 500);
  r.in.vbat = AUTO_VBAT_LOW_LAND_V - 0.05f;
  CHECK(untilState(&r, AUTO_LAND, 1000) > 0 && r.out.reason == AUTO_RS_BATT_LOW && r.out.mv.end == AUTO_MVE_LEFT_HOVER,
        "BATT_LOW during the correction: LAND BATT_LOW, the motion LEFT_HOVER");
  r.in.vbat = 4.0f;
  /* the ARM clears the rate bound (the same state machine, a new sortie) */
  CHECK(r.f.setHeightMs != 0u, "the sortie had a SET_HEIGHT");
  CHECK(untilState(&r, AUTO_IDLE, 15000) > 0, "landed");
  run(&r, AUTO_RELEASE_HOLD_MS + 100);
  r.in.onGroundMs = 30000;
  r.in.appGestureAgeMs = 3000;
  prepMode(&r, AUTO_MODE_FLIGHT);
  o = doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0));
  CHECK(o.state == AUTO_ARMED && r.f.setHeightMs == 0u, "ARM again: no SET_HEIGHT in the new sortie (%s)",
        autoStateName(o.state));
  CHECK(AUTO_CMD_SET_HEIGHT == 10 && AUTO_MVW_SH_STEP == 13 && AUTO_MVW_SH_RATE == 14 && AUTO_MVW_SH_CONTACT == 15 &&
        AUTO_MVF_SET_HEIGHT == 0x40 && strcmp(autoMvWhyName(AUTO_MVW_SH_CONTACT), "SH_CONTACT") == 0,
        "the wire values: SET_HEIGHT 10, SH_STEP 13, SH_RATE 14, SH_CONTACT 15, mvFlags 0x40");
}

/* ---- SET_HEIGHT "go to the target" (2026-10-16, auto_fsm.h AUTO_SH_FORM_TO, docs/autonomy-a2b.md 27) ------------- */

static autoCmd_t cmdSetHeightTo(int16_t xCm, int16_t atCm, int16_t toCm)
{
  autoCmd_t c = cmdSetHeight(xCm, atCm);
  c.mode = AUTO_SH_FORM_TO;
  c.moveRightCm = toCm;
  return c;
}

/* In the wait after a take-off to hTargetCm, the estimate at h (the climb's own leaves it lower). */
static bool toShReadyAt(rig_t *r, uint32_t t0, uint16_t hTargetCm, float h)
{
  rigTakeoffCm = hTargetCm;
  const bool ok = toShReady(r, t0, h);
  rigTakeoffCm = 30;
  return ok && r->f.heightCm == hTargetCm;
}

static void testSetHeightTo(void)
{
  rig_t r;
  autoFsmOut_t o;
  /* the flaw it fixes: HOLD sits 0.10 m under the commanded 1.00 m; "you are at 1.10" (a photo at 0.90): the old form
   * flies back to 0.90 (true), the new one to 1.00 */
  CHECK(toShReadyAt(&r, 70000, 100, 0.90f), "waiting, take-off 1.00 m, the estimate 0.90");
  const uint16_t ev0 = autoFsmReport(&r.f)->evCount;
  o = doCmd(&r, cmdSetHeightTo(110, 90, AUTO_SH_TO_REF));
  CHECK(o.cmdResult == AUTO_R_OK && (o.cmdDetail & 0xFFFF) == AUTO_HOVER && shOldCm(&o) == 90 &&
        fabsf(o.hShift - 0.20f) < 1e-6f && fabsf(r.vz.o.h - 1.10f) < 1e-5f && fabsf(o.mv.tgtUpM + 0.10f) < 1e-5f &&
        (o.mv.flags & AUTO_MVF_SET_HEIGHT) && fabsf(o.mv.hOff - 0.10f) < 1e-5f,
        "X 1.10 at 0.90, the target the commanded 1.00: OK, the shift +0.20, the flight -0.10 (%.3f), the reference at "
        "the corrected estimate (hOff %.3f)", o.mv.tgtUpM, o.mv.hOff);
  long te = untilMotionEnd(&r, 10000);
  CHECK(te > 0 && r.out.mv.end == AUTO_MVE_DONE && fabsf(r.out.mv.hOff) < 1e-5f && fabsf(r.vz.o.h - 1.00f) < 0.01f &&
        autoFsmReport(&r.f)->evCount == ev0,
        "DONE after %ld ms at the target: the estimate (= the true height) %.3f, the reference back at 1.00 (hOff %.6f), "
        "no event", te, r.vz.o.h, r.out.mv.hOff);
  /* the same with the old form: back to 0.90 */
  CHECK(toShReadyAt(&r, 70500, 100, 0.90f), "waiting");
  o = doCmd(&r, cmdSetHeight(110, 90));
  CHECK(o.cmdResult == AUTO_R_OK && fabsf(o.mv.tgtUpM + 0.20f) < 1e-5f && untilMotionEnd(&r, 10000) > 0 &&
        fabsf(r.vz.o.h - 0.90f) < 0.01f && fabsf(r.out.mv.hOff) < 1e-5f,
        "the 4-byte form unchanged: flies -0.20 back to 0.90 (%.3f)", r.vz.o.h);

  /* an explicit target, d 0 (the estimate was right): the flight alone; the commanded height follows it */
  CHECK(toShReadyAt(&r, 71000, 100, 0.90f), "waiting");
  o = doCmd(&r, cmdSetHeightTo(90, 90, 120));
  CHECK(o.cmdResult == AUTO_R_OK && o.hShift == 0.0f && fabsf(o.mv.tgtUpM - 0.30f) < 1e-5f &&
        fabsf(o.mv.hOff + 0.10f) < 1e-5f, "X 0.90 at 0.90 --to 1.20: OK, no shift, +0.30 (%.3f)", o.mv.tgtUpM);
  CHECK(untilMotionEnd(&r, 10000) > 0 && r.out.mv.end == AUTO_MVE_DONE && fabsf(r.vz.o.h - 1.20f) < 0.01f &&
        fabsf(r.out.mv.hOff - 0.20f) < 1e-5f, "DONE at 1.20 (%.3f), hOff +0.20 (%.4f): the commanded height 1.20",
        r.vz.o.h, r.out.mv.hOff);
  run(&r, AUTO_SH_GAP_MS);
  o = doCmd(&r, cmdSetHeightTo(115, AUTO_SH_AT_NOW, AUTO_SH_TO_REF));   /* the target is now 1.20 */
  CHECK(o.cmdResult == AUTO_R_OK && fabsf(o.mv.tgtUpM - 0.05f) < 0.006f, "then 'at 1.15 now' to the commanded 1.20: "
        "+0.05 (%.3f)", o.mv.tgtUpM);
  CHECK(untilMotionEnd(&r, 10000) > 0 && fabsf(r.vz.o.h - 1.20f) < 0.01f && fabsf(r.out.mv.hOff - 0.20f) < 1e-4f,
        "DONE at 1.20 (%.3f, hOff %.4f)", r.vz.o.h, r.out.mv.hOff);

  /* the ranges of the new form, in the checks' order */
  static const struct { float h; int16_t x, at, to; autoMvWhy_t why; const char *what; } rc[] = {
    { 1.00f, 100, 100, 100, AUTO_MVW_RANGE, "nothing to do: d 0, the flight 0" },
    { 0.90f, 90, 90, 24, AUTO_MVW_RANGE, "--to 0.24 m" },
    { 0.90f, 90, 90, 201, AUTO_MVW_RANGE, "--to 2.01 m" },
    { 0.90f, 45, 90, AUTO_SH_TO_REF, AUTO_MVW_NONE, "X 0.45 at 0.90: d -0.45, the flight +0.55" },
    { 0.90f, 44, 90, AUTO_SH_TO_REF, AUTO_MVW_SH_STEP, "X 0.44 at 0.90: the flight +0.56" },
    { 0.90f, 140, 90, AUTO_SH_TO_REF, AUTO_MVW_NONE, "X 1.40 at 0.90: d +0.50, the flight -0.40" },
    { 0.90f, 141, 90, AUTO_SH_TO_REF, AUTO_MVW_SH_STEP, "X 1.41 at 0.90: d +0.51" },
    { 0.90f, 60, 90, 25, AUTO_MVW_HEIGHT, "X 0.60 --to 0.25: the flight -0.35 under the floor (0.25 + the drift)" },
    { 0.90f, 90, 90, 60, AUTO_MVW_NONE, "X 0.90 --to 0.60: the flight -0.30 above any floor (<= 0.55)" },
    { 0.40f, 10, 40, 25, AUTO_MVW_NONE, "X 0.10 at 0.40 --to 0.25: d -0.30, the estimate 0.10" },
    { 0.40f, 9, 40, 25, AUTO_MVW_RANGE, "X 0.09" },
  };
  for (size_t k = 0; k < sizeof(rc) / sizeof(rc[0]); k++) {
    CHECK(toShReadyAt(&r, 72000 + 100u * (uint32_t)k, 100, rc[k].h), "waiting at %.2f m", rc[k].h);
    o = doCmd(&r, cmdSetHeightTo(rc[k].x, rc[k].at, rc[k].to));
    CHECK(mvWhyOf(&o) == rc[k].why && (o.cmdResult == AUTO_R_OK) == (rc[k].why == AUTO_MVW_NONE) &&
          (o.cmdResult == AUTO_R_OK || (o.hShift == 0.0f && o.mv.state != AUTO_MV_RUN)),
          "%s: %s (%s)", rc[k].what, autoMvWhyName(rc[k].why), autoMvWhyName((autoMvWhy_t)mvWhyOf(&o)));
  }

  /* the target form's descent keeps a MOVE down's floor (2026-10-16 review): 0.25 m + 0.01 m/s of air, at most 0.55 */
  CHECK(toShReadyAt(&r, 73000, 100, 0.90f), "waiting");
  run(&r, 31000);   /* over 30 s of air: the floor 0.55 */
  CHECK(r.out.state == AUTO_HOVER && r.out.waiting, "still waiting after 31 s more");
  o = doCmd(&r, cmdSetHeightTo(90, AUTO_SH_AT_NOW, 54));
  CHECK(mvWhyOf(&o) == AUTO_MVW_HEIGHT && o.cmdResult == AUTO_R_RANGE && o.hShift == 0.0f && o.mv.state != AUTO_MV_RUN,
        "after 31 s of air --to 0.54 down: HEIGHT (the floor 0.55; %s)", autoMvWhyName((autoMvWhy_t)mvWhyOf(&o)));
  o = doCmd(&r, cmdSetHeightTo(90, AUTO_SH_AT_NOW, 56));
  CHECK(o.cmdResult == AUTO_R_OK && fabsf(o.mv.tgtUpM + 0.34f) < 0.006f, "... --to 0.56: OK (%.3f)", o.mv.tgtUpM);
  CHECK(untilMotionEnd(&r, 10000) > 0 && fabsf(r.vz.o.h - 0.56f) < 0.01f, "DONE at 0.56 (%.3f)", r.vz.o.h);
  run(&r, AUTO_SH_GAP_MS);
  o = doCmd(&r, cmdSetHeightTo(30, AUTO_SH_AT_NOW, 54));
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.tgtUpM > 0.23f, "a climb to 0.54 from a judged 0.30: not a descent, no floor "
        "(%.3f)", o.mv.tgtUpM);
  CHECK(untilMotionEnd(&r, 10000) > 0, "DONE");
  run(&r, AUTO_SH_GAP_MS);
  o = doCmd(&r, cmdSetHeight(70, AUTO_SH_AT_NOW));
  CHECK(o.cmdResult == AUTO_R_OK, "the 4-byte form has no floor (as the build before it)");
  CHECK(untilMotionEnd(&r, 10000) > 0, "DONE");

  /* a LAND during it: the measured travel committed, the reference where the drone is (its corrected estimate) */
  CHECK(toShReadyAt(&r, 73500, 100, 0.90f), "waiting");
  CHECK(doCmd(&r, cmdSetHeightTo(60, 90, AUTO_SH_TO_REF)).cmdResult == AUTO_R_OK, "X 0.60: the flight +0.40");
  run(&r, 1500);
  const float hMid = r.vz.o.h;
  o = doCmd(&r, cmdOf(AUTO_CMD_LAND));
  CHECK(o.state == AUTO_LAND && o.mv.end == AUTO_MVE_LEFT_HOVER && hMid > 0.62f && hMid < 0.98f &&
        fabsf(1.00f + o.mv.hOff - hMid) < 0.002f,
        "LAND 1.5 s into it: the reference where the drone is (1.00 + %.3f against %.3f)", o.mv.hOff, hMid);

  /* thrust headroom, never the voltage (2026-10-16): the up flight refused at the ceiling, the descent accepted */
  CHECK(toShReadyAt(&r, 74000, 100, 0.90f), "waiting");
  r.vz.ceiling = 56000.0f;   /* a battery bound under 58000 */
  r.vz.holdThrust = 51500.0f;
  r.in.vbat = 3.12f;         /* the tiger flight's loaded battery: no rule on it */
  run(&r, 300);
  o = doCmd(&r, cmdSetHeightTo(70, 90, AUTO_SH_TO_REF));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_UP_HEADROOM && o.hShift == 0.0f &&
        o.state == AUTO_HOVER, "ceiling 56000, T_hold 51500 (headroom 4500): a flight up refused UP_HEADROOM");
  o = doCmd(&r, cmdMove(0, 0, 20));
  CHECK(o.cmdResult == AUTO_R_LOW_BATT && mvWhyOf(&o) == AUTO_MVW_UP_HEADROOM && o.mv.state != AUTO_MV_RUN,
        "... a MOVE up too");
  o = doCmd(&r, cmdSetHeightTo(120, 90, AUTO_SH_TO_REF));
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.tgtUpM < 0.0f, "... a correction down accepted (%.2f)", o.mv.tgtUpM);
  CHECK(untilMotionEnd(&r, 10000) > 0 && fabsf(r.vz.o.h - 1.00f) < 0.01f, "DONE at 1.00");
  r.vz.ceiling = 0.0f;       /* 58000 */
  run(&r, AUTO_SH_GAP_MS);
  o = doCmd(&r, cmdSetHeightTo(70, AUTO_SH_AT_NOW, AUTO_SH_TO_REF));
  CHECK(o.cmdResult == AUTO_R_OK && o.mv.tgtUpM > 0.29f, "the 58000 ceiling, T_hold 51500 at 3.12 V: the flight up "
        "+0.30 accepted (%.2f)", o.mv.tgtUpM);
  CHECK(untilMotionEnd(&r, 10000) > 0, "DONE");
  /* the relative floor: a hover on a support (T_hold 30000) under the model's soft ceiling 1.117 x: allowed */
  r.vz.holdThrust = 30000.0f;
  r.vz.ceiling = 33500.0f;
  run(&r, 300);
  o = doCmd(&r, cmdMove(0, 0, 20));
  CHECK(o.cmdResult == AUTO_R_OK, "T_hold 30000, ceiling 33500 (need min(4600, 3000)): MOVE up accepted");
  CHECK(untilMotionEnd(&r, 10000) > 0, "DONE");
  r.vz.ceiling = 32900.0f;
  run(&r, 300);
  o = doCmd(&r, cmdMove(0, 0, 20));
  CHECK(mvWhyOf(&o) == AUTO_MVW_UP_HEADROOM, "... ceiling 32900 (2900 < 3000): UP_HEADROOM");
  /* a non-finite T_hold or ceiling refuses (2026-10-16 review: the comment promised it, the code accepted a NaN) */
  r.vz.ceiling = 0.0f;
  static const float bad[] = {NAN, INFINITY, -1.0f, 36000.0f};
  for (size_t k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) {
    rig_t q;
    CHECK(toShReadyAt(&q, 75000 + 100u * (uint32_t)k, 100, 0.90f), "waiting");
    const bool nanCeil = k == 3;   /* the last: a fine T_hold under a NaN ceiling */
    q.vz.tHoldBad = !nanCeil;
    q.vz.tHoldBadV = bad[k];
    q.vz.ceilNan = nanCeil;
    rigStep(&q);
    rigStep(&q);
    o = doCmd(&q, cmdMove(0, 0, 20));
    CHECK(mvWhyOf(&o) == AUTO_MVW_UP_HEADROOM && o.mv.state != AUTO_MV_RUN, "%s %g: MOVE up UP_HEADROOM (%s)",
          nanCeil ? "a NaN ceiling, T_hold" : "T_hold", (double)bad[k], autoMvWhyName((autoMvWhy_t)mvWhyOf(&o)));
  }
  r.vz.holdThrust = 36000.0f;
  r.in.vbat = 4.05f;
  run(&r, 300);
  CHECK(AUTO_MV_UP_HEADROOM_T == 4600.0f && VZ_THRUST_ABS_MAX - AUTO_MV_UP_HEADROOM_T == AUTO_MV_REFUSE_T &&
        AUTO_MVW_UP_HEADROOM == 16 && strcmp(autoMvWhyName(AUTO_MVW_UP_HEADROOM), "UP_HEADROOM") == 0 &&
        AUTO_SH_FORM_TO == 1 && AUTO_SH_TO_REF == INT16_MIN,
        "the values: 4600 (= 58000 - 53400), UP_HEADROOM 16, the target form 1, TO_REF INT16_MIN");
}

/* The model replay of the 2026-10-15 / 10-16 corrections (docs/autonomy-a2b.md 27.2; Claude's judgements in RESUME.md,
 * the estimates from events.jsonl and the flight logs): the photo's error carried to the command (moves since the photo
 * are real: the offset form's assumption), the plant tracking the vertical reference exactly. The new form must end
 * within 0.03 m of the intended 1.00 m; the 4-byte form as it was sent is printed (its true end h_now + X - x). */
static void testSetHeightReplay(void)
{
  static const struct {
    const char *what;
    float hNow;          /* the estimate at the command (the ACK's old estimate) */
    int16_t atCm, xCm;   /* the photo's estimate at capture, Claude's judged true height then */
    int16_t sentCm;      /* what was sent (the 4-byte form; X, or the est-rule's h_at + X - 1.00) */
    bool noCam;          /* refused NO_CAM_HEIGHT then */
  } rp[] = {
    { "10-15 p2 18:18 (1.05 judged)", 0.90f, 91, 105, 105, false },
    { "10-15 p1 19:44 (1.08)", 0.89f, 90, 108, 108, false },
    { "10-15 p1 19:45 (0.83, CAM_LOST)", 1.03f, 89, 83, 83, true },
    { "10-15 fastcheck 20:05 (1.28)", 0.93f, 93, 128, 128, false },
    { "10-15 fastcheck 20:06 (0.82)", 0.93f, 92, 82, 82, false },
    { "10-16 F1 09:45 (1.12)", 0.90f, 90, 112, 112, false },
    { "10-16 F2 09:52:16 (1.15, est-rule 1.04)", 0.89f, 89, 115, 104, false },
    { "10-16 F2 09:52:43 (0.60, est-rule 0.49)", 0.90f, 89, 60, 49, false },
    { "10-16 tiger 11:09 (0.70, est-rule 0.55)", 1.06f, 85, 70, 55, false },
  };
  for (size_t k = 0; k < sizeof(rp) / sizeof(rp[0]); k++) {
    rig_t r;
    const float trueNow = rp[k].hNow + (float)(rp[k].xCm - rp[k].atCm) * 0.01f;
    const float legacyEnd = rp[k].hNow + (float)(rp[k].xCm - rp[k].sentCm) * 0.01f;
    CHECK(toShReadyAt(&r, 76000 + 200u * (uint32_t)k, 100, rp[k].hNow), "%s: waiting", rp[k].what);
    if (rp[k].noCam) {
      r.vz.noCam = true;
      rigStep(&r);
      const autoFsmOut_t o = doCmd(&r, cmdSetHeightTo(rp[k].xCm, rp[k].atCm, AUTO_SH_TO_REF));
      CHECK(mvWhyOf(&o) == AUTO_MVW_NO_CAM_HEIGHT && o.hShift == 0.0f, "%s: refused NO_CAM_HEIGHT as then", rp[k].what);
      continue;
    }
    const autoFsmOut_t o = doCmd(&r, cmdSetHeightTo(rp[k].xCm, rp[k].atCm, AUTO_SH_TO_REF));
    const long te = untilMotionEnd(&r, 15000);
    /* the model's truth: the corrected estimate is the true height (the shift put it there), and the plant follows */
    const float end = r.vz.o.h;
    CHECK(o.cmdResult == AUTO_R_OK && te > 0 && r.out.mv.end == AUTO_MVE_DONE && fabsf(end - 1.00f) <= 0.03f &&
          fabsf(o.hShift - (trueNow - rp[k].hNow)) < 1e-4f,
          "%s: true now %.2f, the flight %+.2f, ends at %.3f (intended 1.00; the 4-byte form as sent: %.2f)",
          rp[k].what, trueNow, o.mv.tgtUpM, end, legacyEnd);
    printf("info: replay %-42s true now %.2f fly %+.2f end %.3f m (sent as then: %.2f m)\n", rp[k].what, trueNow,
           o.mv.tgtUpM, end, legacyEnd);
  }
}

/* The hold at its limit (auto_fsm.h autoFsmHoldLimit, AUTO_EV_HOLD_LIMIT, docs/autonomy-a2b.md 27.4). */
static void testHoldLimit(void)
{
  rig_t r;
  CHECK(toShReady(&r, 80000, 1.00f), "waiting");
  const uint16_t ev0 = autoFsmReport(&r.f)->evCount, trim0 = autoFsmTrimEvents(&r.f);
  autoFsmHoldLimit(&r.f, AUTO_HL_ROLL, r.in.nowMs);   /* roll positive at its limit */
  rigStep(&r);
  const autoFsmReport_t *rep = autoFsmReport(&r.f);
  CHECK(rep->evCount == ev0 + 1 && rep->evCode[0] == AUTO_EV_HOLD_LIMIT && rep->evDetail[0] == AUTO_HL_ROLL &&
        autoFsmTrimEvents(&r.f) == trim0 && r.out.state == AUTO_HOVER && r.out.waiting &&
        strcmp(autoEventName(AUTO_EV_HOLD_LIMIT), "HOLD_LIMIT") == 0 && AUTO_EV_HOLD_LIMIT == 15,
        "the roll bit rose: HOLD_LIMIT 15 detail ROLL, the trim learner's count unchanged, still hovering");
  autoFsmHoldLimit(&r.f, AUTO_HL_ROLL | AUTO_HL_PITCH | AUTO_HL_PITCH_NEG, r.in.nowMs);
  CHECK(rep->evCount == ev0 + 2 && rep->evDetail[0] == (AUTO_HL_PITCH | AUTO_HL_PITCH_NEG),
        "then pitch (negative): one more, detail its axis and sign (0x%02x)", rep->evDetail[0]);
  autoFsmHoldLimit(&r.f, AUTO_HL_ROLL | AUTO_HL_PITCH | AUTO_HL_PITCH_NEG, r.in.nowMs);
  CHECK(rep->evCount == ev0 + 2, "the same bits again: nothing");
  autoFsmHoldLimit(&r.f, AUTO_HL_PITCH | AUTO_HL_PITCH_NEG, r.in.nowMs);
  autoFsmHoldLimit(&r.f, AUTO_HL_ROLL | AUTO_HL_PITCH | AUTO_HL_PITCH_NEG, r.in.nowMs);
  CHECK(rep->evCount == ev0 + 3 && rep->evDetail[0] == (AUTO_HL_ROLL | AUTO_HL_PITCH_NEG),
        "roll fell and rose again: reported again (0x%02x)", rep->evDetail[0]);
  /* during a motion: noted, not reported */
  autoFsmHoldLimit(&r.f, 0, r.in.nowMs);
  CHECK(doCmd(&r, cmdMove(0, 0, 20)).cmdResult == AUTO_R_OK, "a MOVE up runs");
  autoFsmHoldLimit(&r.f, AUTO_HL_ROLL, r.in.nowMs);
  CHECK(rep->evCount == ev0 + 3, "a bit rising during a motion: no event");
  CHECK(untilMotionEnd(&r, 10000) > 0, "DONE");
  /* the rise during the motion was not taken as seen: the camera's next change reports it (2026-10-16 review) */
  autoFsmHoldLimit(&r.f, AUTO_HL_ROLL | AUTO_HL_ROLL_NEG, r.in.nowMs);
  CHECK(rep->evCount == ev0 + 4 && rep->evDetail[0] == (AUTO_HL_ROLL | AUTO_HL_ROLL_NEG),
        "after the motion, the next change (the sign): the roll reported (0x%02x)", rep->evDetail[0]);
  autoFsmHoldLimit(&r.f, AUTO_HL_ROLL, r.in.nowMs);
  CHECK(rep->evCount == ev0 + 4, "the sign again: nothing (the roll seen)");
  /* the trim learner: a sortie with only HOLD_LIMIT counts no event */
  CHECK(autoFsmTrimEvents(&r.f) == trim0 && r.f.infoEvents == 4, "trim events %u (the 4 HOLD_LIMIT left out)",
        (unsigned)autoFsmTrimEvents(&r.f));
  /* outside HOVER (LAND): noted only; the next ARM clears the record */
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND, "LAND");
  autoFsmHoldLimit(&r.f, 0, r.in.nowMs);
  autoFsmHoldLimit(&r.f, AUTO_HL_PITCH, r.in.nowMs);
  CHECK(rep->evCount == ev0 + 4, "in LAND: no event");
  CHECK(untilState(&r, AUTO_IDLE, 15000) > 0, "landed");
  run(&r, AUTO_RELEASE_HOLD_MS + 100);
  r.in.onGroundMs = 30000;
  r.in.appGestureAgeMs = 3000;
  prepMode(&r, AUTO_MODE_FLIGHT);
  CHECK(doCmd(&r, cmdArm(AUTO_MODE_FLIGHT, 0)).state == AUTO_ARMED && r.f.infoEvents == 0 && r.f.holdLimSeen == 0 &&
        autoFsmTrimEvents(&r.f) == 0, "ARM: cleared");
  /* DRY: never reported (the camera's bits come from a real view only) */
  rigInit(&r, 81000);
  CHECK(toWait(&r, AUTO_MODE_DRY), "DRY waiting");
  const uint16_t e1 = autoFsmReport(&r.f)->evCount;
  autoFsmHoldLimit(&r.f, AUTO_HL_ROLL, r.in.nowMs);
  CHECK(autoFsmReport(&r.f)->evCount == e1, "DRY: no event");
}

/* mvspeed (2026-10-16, auto_motion.h "fast moves", docs/autonomy-a2b.md 31): the glue's autoFsmSetMvSpeed right after
 * the ARM step latches the step into the motions (the ARM's autoMotionInit put it back to 100); a fast MOVE whose view
 * stays lost at speed is stopped by the step with its brake and reported as MOVE_STOPPED VIEW_FAST, the drone in HOVER,
 * no LAND; the next sortie's ARM starts at 100 again. */
static void testMvSpeed(void)
{
  rig_t r;
  rigInit(&r, 53000);
  CHECK(toMoveReady(&r, 4.0f, 30000), "waiting, D 4.0 m");
  CHECK(r.f.mv.speedPct == 100, "the ARM's motions at 100 (%u)", r.f.mv.speedPct);
  autoFsmSetMvSpeed(&r.f, 200);
  CHECK(r.f.mv.speedPct == 200 && r.f.mv.settleMaxMs == 2500 && r.f.mv.dwellMs == 300,   /* (1500 before 2026-10-17) */
        "autoFsmSetMvSpeed(200): the motions at 200");
  CHECK(doCmd(&r, cmdMove(0, 150, 0)).cmdResult == AUTO_R_OK && r.f.mv.ph.vPk > 0.39f,
        "MOVE forward 150 at 200: accepted, peak %.2f m/s", r.f.mv.ph.vPk);
  run(&r, 2500);
  const uint16_t ev0 = autoFsmReport(&r.f)->evCount;
  r.in.camFrameAgeMs = AUTO_MV_VIEW_MAX_AGE_MS + 100;
  const long t = untilEvt(&r, AUTO_EV_MOVE_STOPPED, 500);
  CHECK(t >= AUTO_MV_FAST_VIEW_DEB_MS && t <= AUTO_MV_FAST_VIEW_DEB_MS + 2 && lastEvtDetail(&r) == AUTO_MVE_VIEW_FAST && autoFsmReport(&r.f)->evCount == (uint16_t)(ev0 + 1u) &&
            r.out.state == AUTO_HOVER && r.out.mv.state == AUTO_MV_STOPPED && r.out.mv.end == AUTO_MVE_VIEW_FAST &&
            r.out.mv.snap && r.out.mv.ffPitchDeg > 1.0f,
        "the view lost at 0.40 m/s: AUTO_MV_FAST_VIEW_DEB_MS later (a refused frame or two only pause) MOVE_STOPPED "
        "VIEW_FAST, STOPPED, a snap, the brake lean %.2f deg, HOVER (%ld ms)", r.out.mv.ffPitchDeg, t);
  r.in.camFrameAgeMs = 30;
  run(&r, 3000);
  CHECK(r.out.state == AUTO_HOVER && r.out.mv.ffPitchDeg == 0.0f, "the brake over, hovering");
  /* the next sortie starts at 100 (the glue sets it again at its ARM) */
  CHECK(doCmd(&r, cmdOf(AUTO_CMD_LAND)).state == AUTO_LAND && toIdle(&r, 30000), "LAND, IDLE");
  r.in.gestureRequired = false;   /* (the consent gesture is per sortie: the rig gave the first one) */
  CHECK(toMoveReady(&r, 4.0f, 30000) && r.f.mv.speedPct == 100 && r.f.mv.settleMaxMs == AUTO_MV_SETTLE_MAX_MS,
        "a new sortie: its ARM put the motions back to 100");
}

int main(void)
{
  testMvSpeed();
  testFullSortie(0, AUTO_HOVER_WAIT_DEFAULT_MS);
  testFullSortie(1000000, 0);
  testFullSortie(0xFFFFFFFFu - 2500u, AUTO_HOVER_WAIT_DEFAULT_MS);   /* the ms clock wraps during the climb */
  testFullSortie(0xFFFFFFFFu - 6000u, AUTO_HOVER_WAIT_MAX_MS);       /* ... during the wait */
  testLandFastReasons();
  testAppStream();
  testAppSilent();
  testHeartbeat();
  testTrips();
  testContact();
  testContactThenTrips();
  testLandTrips();
  testNaN();
  testKill();
  testCamAwake();
  testLateYawRecoveryAndRealLoss();
  testSettleSequence();
  testSoak();
  testGuards();
  testAirTime();
  testBattery();
  testEndOfBattery();
  testNotStable();
  testStable();
  testStableGrace();
  testStableFrameAge();
  testStableNever();
  testStableBudget();
  testHoverWait();
  testHoverCamera();
  testKilledExit();
  testVzOutcomes();
  testArmedExits();
  testModes();
  testCommandTable();
  testNames();
  testReport();
  testMotionAccept();
  testMotionRefuse();
  testMotionGuards();
  testMotionYaw();
  testCollisionMotion();
  testLandAfterDoneMove();
  testLandBackstopHigh();
  testPlanTakeoff();
  testBlindEveryLand();
  testSetHeight();
  testSetHeightTo();
  testSetHeightReplay();
  testHoldLimit();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
