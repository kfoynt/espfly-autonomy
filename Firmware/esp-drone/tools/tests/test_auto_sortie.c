/*
 * Coupled host test: the real state machine (auto_fsm.c) drives the real
 * vertical controller (vert_ctrl.c) on the sim plant and its camera observer
 * model, through a small emulation of the A2b glue (docs/autonomy-a2.md
 * section 2): FLIGHT mode, Mac heartbeat and phone stream healthy, the
 * camera running on the ground, asleep in the take-off window until the
 * sim's camera wakes, the photo sets completing after n x gap. What the
 * fake controller of test_auto_fsm.c cannot show: the air clock against the
 * true lift-off, the hover wait on the camera, a trip in LAND against the
 * real touchdown thrust condition, the true peak heights and landing times
 * of whole sorties (the user range's corners included), and the hover gain
 * learned over chains of sorties. The emulation decimates its inputs and
 * its IDLE steps with the glue's own constants and fast-path rule
 * (autonomy.h, auto_glue_core.h autoIdleSkip; header-only here).
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      -Icomponents/core/crazyflie/modules/interface -Icomponents/core/crazyflie/hal/interface \
 *      tools/tests/test_auto_sortie.c components/core/crazyflie/utils/src/auto_fsm.c \
 *      components/core/crazyflie/utils/src/vert_ctrl.c components/core/crazyflie/utils/src/auto_motion.c \
 *      -lm -o /tmp/tas && /tmp/tas
 *
 * Lines starting with "info:" are measured numbers, not checks.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_fsm.h"
#include "autonomy.h"   /* AUTONOMY_SLOW_EVERY, AUTONOMY_IDLE_STEP_EVERY, autoIdleSkip (auto_glue_core.h) */

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

/* The glue's input decimation (autonomy.h, docs/autonomy-a2b.md section 2.3):
 * camera, hold, ground, battery-minimum and lock inputs are sampled every
 * AUTONOMY_SLOW_EVERY-th loop; in IDLE with nothing pending the FSM steps
 * every AUTONOMY_IDLE_STEP_EVERY-th loop (autoIdleSkip). */
#define CAM_RUNNING  2   /* CAMERA_FLOW_RUNNING */
#define CAM_STANDBY  5
#define TOKEN        0x5EED0001u
/* The evening envelope of 2026-10-12 (a 12 s wait at most, an 18 s air
 * clock): the rows written for it keep its wait. Since 2026-10-15 ("Claude
 * decides", docs/autonomy-a2b.md 22) the state machine lands on none of the
 * hover's clocks (no HOVER_TIMEOUT, no AIR_TIME, no SORTIE_DONE): the wait is
 * information for the Mac. This emulation plays the Mac's 'sortie': its LAND
 * once the wait has run its length from the last motion (the state
 * machine's waitMs, what HOVER_TIMEOUT measured), so a sortie that ended
 * HOVER_TIMEOUT now ends LAND_CMD at the same step plus one. */
#define EVENING_WAIT_MAX_MS 12000
/* The reason of a sortie the emulated Mac landed at the wait's end (HOVER_TIMEOUT until 2026-10-15) */
#define RS_WAIT_END AUTO_RS_LAND_CMD
#define EV_BIT(e) (1u << (unsigned)(e))
#define NWHY 48   /* autoReason_t codes counted (AUTO_RS_RESUME 45 the last) */
_Static_assert(AUTO_RS_RESUME < NWHY, "NWHY covers every autoReason_t");

static uint32_t rngState = 4242u;
static int armRefused = 0;   /* sorties whose ARM the state machine refused (none may be) */

static float urand(void)
{
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  return (float)(rngState >> 8) * (1.0f / 16777216.0f);
}

/* ---- one sortie --------------------------------------------------------- */

typedef struct {
  float bias, kerr, camC;  /* residual in-flight offset after bAir (g), K error, camera offset c (m/s) */
  float camWakeS;          /* < 0: no camera */
  float camLossS;          /* >= 0: the observer is lost for good this long after the lift-off */
  uint32_t waitMs;         /* NVS autowait (u32 since 2026-10-14) */
  uint8_t photoN;
  uint16_t photoGapMs;
  float landAfterLiftS;    /* >= 0: Mac LAND this long after the controller's lift-off */
  float tripAtLandS;       /* >= 0: tilt 31 deg from this long into LAND, held */
  bool contactBeforeTrip;  /* |acc| 2 g on the step before the trip */
  bool tripAtTdOk;         /* the tilt starts on the first LAND step with tdThrustOk (tripAtLandS ignored) */
  float appAtWaitS;        /* >= 0: phone stick this long into the hover wait */
  float kPrior;            /* > 0: the stored hover gain (NVS vzK); else VZ_K_DEFAULT */
  float kTrue;             /* > 0: the drone's true gain; else kPrior x (1 + kerr) */
  float camConvS;          /* > 0: the observer's convergence time (sim camConvS); else the sim default 0.3 s */
  float vRest;             /* > 0: resting battery (sim vRest); else drawn from the seed in 4.00-4.15 V, a battery
                              a FLIGHT sortie can arm with (G3, AUTO_VMIN_DEFAULT 3.80 V) */
  float vminOverride;      /* > 0: NVS vmin (a G3 override for a stress row); else AUTO_VMIN_DEFAULT */
  uint16_t accelMs;        /* > 0: NVS autoaccel (HOVER on the accelerometer alone); else the default */
  bool camFirmware;        /* the camera wakes as the firmware does it: camWakeS (the take-off standby), or at
                              once when the state machine armed with camNoStandby */
  float appAfterLiftS;     /* >= 0: phone stick this long after the controller's lift-off */
  float appAfterRampS;     /* >= 0: phone stick this long after the first RAMP step */
  float abortSlowRel;      /* > 0: vzConfig_t.abortSlowRel */
  float landClimbRefRel;   /* > 0: vzConfig_t.landClimbRefRel */
  uint16_t heightCm;       /* > 0: TAKEOFF heightCm; else 30 */
  uint16_t airNvsMs;       /* > 0: NVS autoair; else AUTO_AIR_MAX_DEFAULT_MS */
  /* the take-off's planned move (2026-10-14, docs/autonomy-a2b.md 21.4): planR / planF cm in the TAKEOFF, flown at
   * the wait's start; sceneD > 0: the observer tracks a scene this far ahead from the lift-off on, and the camera
   * hold follows the motion's reference ideally (its progress the reference) */
  int16_t planR, planF;
  float sceneD;
  bool noMacLand;          /* the Mac never lands at the wait's end (a hover left to the drone's own rules) */
  float macLandAtS;        /* > 0: the Mac's LAND this long after the first RAMP step, whatever the wait */
} scen_t;

typedef struct {
  autoReason_t landWhy;    /* the transition into LAND (or KILLED) */
  autoReason_t endWhy;     /* the final IDLE reason */
  autoReason_t killWhy;    /* KILLED: why */
  autoState_t lastActive;  /* LAND or KILLED */
  vzReason_t vzLand;
  bool killed, waited, photos, aborted, held;
  float rampT;             /* sim time of the controller's first RAMP step, -1 before */
  float liftDet;           /* sim time of the controller's lift-off detection, -1 before */
  float liftTrue, landEntryT, landEntryAir, trueAirAtLand, landS, hMax, impactV, hAtLand, airS;
  bool learned;            /* vzLearnK() took the sortie ... */
  float kLearned;          /* ... and gave this kPrior (the old one when refused) */
  bool tripped;            /* the tilt of tripAtLandS / tripAtTdOk started */
  bool tdOkAtTrip;
  float hAtTrip;
  int captures;
  float appT, hAtApp, hMaxAfterApp;   /* phone stick: sim time, true height then, true peak after it (-1: none) */
  bool upAtApp;            /* the sim was off the ground at the stick */
  float waitS;             /* hover wait start to LAND entry (0 without a wait) */
  float airAtHover, airAtWait;   /* the air clock (s) at HOVER entry and at the wait's start (0: not reached) */
  /* the planned move: TAKEOFF accepted, HOVER entry, the wait's start, the plan's RUN, its end (sim s; -1: never),
   * its end, the motion count at the wait's first step, the true height's and the controller's estimate's largest
   * excursion from their values at the plan's start while it ran (m) */
  float tTakeoff, tHover, tWait, tPlanRun, tPlanEnd;
  autoMvEnd_t planEnd;
  int seqAtWait;
  float planDh, planDhEst;
  bool macWaitLand;        /* the emulated Mac's LAND at the wait's end (the old HOVER_TIMEOUT) */
  uint32_t evMask;         /* 1 << autoEvent_t of every event the state machine reported */
  int contacts, sinks, resumes;
} sortie_t;

static void scenDefault(scen_t *s)
{
  memset(s, 0, sizeof(*s));
  s->camWakeS = 1.5f;
  s->camLossS = -1.0f;
  s->waitMs = AUTO_HOVER_WAIT_DEFAULT_MS;
  s->photoN = 2;
  s->photoGapMs = 150;
  s->landAfterLiftS = -1.0f;
  s->tripAtLandS = -1.0f;
  s->appAtWaitS = -1.0f;
  s->appAfterLiftS = -1.0f;
  s->appAfterRampS = -1.0f;
}

static void runSortie(const scen_t *sc, uint32_t seed, sortie_t *res)
{
  memset(res, 0, sizeof(*res));
  res->liftTrue = res->landEntryT = res->rampT = res->liftDet = -1.0f;
  res->appT = res->hAtApp = res->hMaxAfterApp = -1.0f;
  res->tTakeoff = res->tHover = res->tWait = res->tPlanRun = res->tPlanEnd = -1.0f;
  res->seqAtWait = -1;
  float planH0 = 0.0f, planHEst0 = 0.0f;
  vzConfig_t vc;
  vzDefaultConfig(&vc);
  if (sc->kPrior > 0.0f) {
    vc.kPrior = sc->kPrior;
  }
  if (sc->abortSlowRel > 0.0f) {
    vc.abortSlowRel = sc->abortSlowRel;
  }
  if (sc->landClimbRefRel > 0.0f) {
    vc.landClimbRefRel = sc->landClimbRefRel;
  }
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.accAirG = vc.bAir + sc->bias;
  pc.kTrue = sc->kTrue > 0.0f ? sc->kTrue : vc.kPrior * (1.0f + sc->kerr);
  pc.camWakeS = sc->camWakeS;
  if (sc->camConvS > 0.0f) {
    pc.camConvS = sc->camConvS;
  }
  pc.vRest = sc->vRest > 0.0f ? sc->vRest : 4.00f + 0.15f * (float)((seed * 2654435761u) >> 8) * (1.0f / 16777216.0f);
  pc.camVzOffset = sc->camC;
  pc.camLossS = sc->camLossS;
  pc.camLossLenS = 100.0f;
  vzSimState_t sim;
  vzSimInit(&sim, &pc, seed);
  vzState_t vz;
  vzOutput_t vo;
  memset(&vo, 0, sizeof(vo));
  bool vzActive = false;
  autoFsm_t f;
  autoFsmInit(&f);
  autoFsmIn_t in;
  memset(&in, 0, sizeof(in));
  autoFsmOut_t out;
  memset(&out, 0, sizeof(out));

  in.nowMs = 100000u;
  in.sessionValid = true;
  in.randToken = TOKEN;
  in.appGestureAgeMs = 2000;
  in.appInputAgeMs = 60000;
  in.appThrustZero = true;
  in.autoEnabled = in.appRequired = in.gestureRequired = true;
  in.vmin = sc->vminOverride > 0.0f ? sc->vminOverride : AUTO_VMIN_DEFAULT;   /* G3 as built (3.80 V) */
  in.airMaxNvsMs = sc->airNvsMs > 0 ? sc->airNvsMs : AUTO_AIR_MAX_DEFAULT_MS;
  in.hoverWaitNvsMs = sc->waitMs;
  in.hoverAccelNvsMs = sc->accelMs > 0 ? sc->accelMs : AUTO_HOVER_ACCEL_DEFAULT_MS;
  in.vzParamsOk = true;
  in.loopGapUs = 1000;
  in.holdEnabled = true;
  in.motorsReady = in.motorGuardOn = true;
  in.heapIntFree = 15700;

  float applied = 0.0f, vbat = sim.vbat, vbatNext = 0.0f;
  uint32_t camRunMs = 20000, capEnd = 0, step = 0, landMs = 0, tripMs = 0;
  bool camWasRunning = true, sentArm = false, sentTakeoff = false, sentLand = false, tripping = false;
  bool appSent = false, inLand = false;
  autoState_t prev = AUTO_IDLE;
  uint32_t waitStartMs = 0;

  for (step = 0; step < 200000; step++) {   /* 200 s: the 120 s air clock and a LAND */
    const float t = (float)step * 0.001f;
    /* sensors, link, phone, camera, as the glue fills autoFsmIn_t */
    if (t >= vbatNext) {
      vbat = sim.vbat;
      vbatNext += 0.1f;
    }
    const bool slow = step % AUTONOMY_SLOW_EVERY == 0 || out.state == AUTO_IDLE;
    in.hbAgeMs = step % 100;
    in.appAgeMs = step % 50;
    in.vbat = vbat;
    if (slow) {
      in.vbatFastMin = sim.vbat;
    }
    in.rollDeg = in.pitchDeg = 0.0f;
    in.tiltDeg = 0.5f;
    in.attErrDeg = 0.2f;
    in.gyroDps[0] = in.gyroDps[1] = in.gyroDps[2] = 0.2f;
    /* |acc|: 1 g at rest on the ground (the 1-D sim's noise would fail the
     * ARM stillness test), the sim's reading in the air and at a touchdown */
    in.accNormG = sim.onGround && sim.jolt < 0.01f ? 1.0f : fabsf(sim.accZ);
    if (tripping) {
      in.tiltDeg = 31.0f;
      in.rollDeg = 31.0f;
    }
    if (sc->contactBeforeTrip && tripMs != 0 && in.nowMs + 1u == tripMs) {
      in.accNormG = 2.0f;
    }
    const bool camRunning = sim.cmdT < 0.0f || sim.camAwake;
    camRunMs = camRunning ? (camWasRunning ? camRunMs + 1u : 0u) : 0u;
    camWasRunning = camRunning;
    if (slow) {
      in.onGround = sim.onGround && applied == 0.0f;
      in.onGroundMs = in.onGround ? 20000u : 0u;
      in.camState = camRunning ? CAM_RUNNING : CAM_STANDBY;
      in.camFps = camRunning ? 16.9f : 0.0f;
      in.camFrameAgeMs = camRunning ? 30u : 5000u;
      in.camViewMs = camRunMs;
      in.holdApplied = camRunning && !sim.onGround;
      in.holdCoasting = false;
    }
    in.vz = vo;
    if (sc->sceneD > 0.0f) {   /* the observer's D and an ideal hold (its progress the motion's reference) */
      in.sceneTracked = !sim.onGround;
      in.sceneDM = sc->sceneD;
      in.holdEpochSeen = out.mv.epoch;
      in.holdProgLat = out.mv.refLat;
      in.holdProgFwd = out.mv.refFwd;
    }
    memset(&in.cmd, 0, sizeof(in.cmd));
    if (!sentArm && step == 10) {
      in.cmd.type = AUTO_CMD_ARM;
      in.cmd.mode = AUTO_MODE_FLIGHT;
      sentArm = true;
    } else if (sentArm && !sentTakeoff && out.state == AUTO_ARMED && step >= 60) {
      in.cmd.type = AUTO_CMD_TAKEOFF;
      in.cmd.token = TOKEN;
      in.cmd.heightCm = sc->heightCm > 0 ? sc->heightCm : 30;
      in.cmd.photoN = sc->photoN;
      in.cmd.photoGapMs = sc->photoGapMs;
      in.cmd.moveRightCm = sc->planR;
      in.cmd.moveFwdCm = sc->planF;
      sentTakeoff = true;
    } else if (!sentLand && sc->landAfterLiftS >= 0.0f && vo.liftedOff &&
               vo.tAir >= sc->landAfterLiftS) {
      in.cmd.type = AUTO_CMD_LAND;
      sentLand = true;
    } else if (!sentLand && sc->macLandAtS > 0.0f && res->rampT >= 0.0f && t - res->rampT >= sc->macLandAtS) {
      in.cmd.type = AUTO_CMD_LAND;
      sentLand = true;
    } else if (!sentLand && !sc->noMacLand && sc->macLandAtS <= 0.0f && out.state == AUTO_HOVER && out.waiting &&
               in.nowMs - f.waitMs >= f.hoverWaitMs) {
      in.cmd.type = AUTO_CMD_LAND;   /* the Mac's sortie: LAND at the wait's end */
      sentLand = true;
      res->macWaitLand = true;
    }
    if (!appSent && sc->appAtWaitS >= 0.0f && out.waiting && waitStartMs != 0 &&
        in.nowMs - waitStartMs >= (uint32_t)(sc->appAtWaitS * 1000.0f)) {
      in.appInputCount++;
      appSent = true;
    }
    if (!appSent && ((sc->appAfterLiftS >= 0.0f && vo.liftedOff && vo.tAir >= sc->appAfterLiftS) ||
                     (sc->appAfterRampS >= 0.0f && res->rampT >= 0.0f && t - res->rampT >= sc->appAfterRampS))) {
      in.appInputCount++;
      appSent = true;
      res->appT = t;
      res->hAtApp = res->hMaxAfterApp = sim.h;
      res->upAtApp = !sim.onGround;
    }
    if (appSent && res->appT >= 0.0f) {
      res->hMaxAfterApp = fmaxf(res->hMaxAfterApp, sim.h);
    }

    /* IDLE fast path, the glue's rule: no step without a command, KILL or
     * session change (this emulation sends neither), except every
     * AUTONOMY_IDLE_STEP_EVERY-th loop. */
    if (autoIdleSkip(out.state, out.thrust, in.cmd.type != AUTO_CMD_NONE, false, false, step,
                     AUTONOMY_IDLE_STEP_EVERY)) {
      in.nowMs++;
      continue;
    }
    const uint16_t evN0 = autoFsmReport(&f)->evCount;
    autoFsmStep(&f, &in, &out);
    {
      const autoFsmReport_t *rep = autoFsmReport(&f);
      const unsigned nNew = (uint16_t)(rep->evCount - evN0);
      for (unsigned j = 0; j < nNew && j < 3u; j++) {
        res->evMask |= 1u << (rep->evCode[j] & 31u);
        res->contacts += rep->evCode[j] == AUTO_EV_CONTACT;
        res->sinks += rep->evCode[j] == AUTO_EV_SINK;
        res->resumes += rep->evCode[j] == AUTO_EV_RESUME;
      }
    }
    if (in.cmd.type == AUTO_CMD_ARM && out.state != AUTO_ARMED) {
      armRefused++;
      printf("  ARM refused (seed %u, guards 0x%x, vbat %.3f V)\n", (unsigned)seed, (unsigned)out.cmdDetail, in.vbat);
    }
    if (sc->camFirmware && out.state == AUTO_ARMED) {
      pc.camWakeS = out.camNoStandby ? 0.0f : sc->camWakeS;   /* before the first command */
    }

    /* glue actions */
    if (out.vzReset) {
      vzConfig_t c = vc;
      vzConfigSetHeight(&c, out.hTargetM);   /* as the glue's vzStart() */
      vzInit(&vz, &c);
      memset(&vo, 0, sizeof(vo));
      vzActive = true;
    }
    if (out.captureStart) {
      in.capture = AUTO_CAP_BUSY;
      capEnd = in.nowMs + (uint32_t)out.captureN * out.captureGapMs;
      res->captures++;
    } else if (in.capture == AUTO_CAP_BUSY && (int32_t)(in.nowMs - capEnd) >= 0) {
      in.capture = AUTO_CAP_DONE;
    }
    if (out.state == AUTO_HOVER && res->airAtHover == 0.0f) {
      res->airAtHover = (float)out.airMs * 0.001f;
      res->tHover = t;
    }
    if (in.cmd.type == AUTO_CMD_TAKEOFF && out.cmdResult == AUTO_R_OK) {
      res->tTakeoff = t;
    }
    if (out.waiting && !res->waited) {
      res->waited = true;
      res->airAtWait = (float)out.airMs * 0.001f;
      waitStartMs = in.nowMs;
      res->tWait = t;
      res->seqAtWait = f.mv.seq;
    }
    if (res->tPlanRun < 0.0f && out.mv.seq == 1 && out.mv.state == AUTO_MV_RUN) {
      res->tPlanRun = t;
      planH0 = sim.h;
      planHEst0 = vo.h;
    }
    if (res->tPlanRun >= 0.0f && res->tPlanEnd < 0.0f) {
      if (out.mv.state == AUTO_MV_DONE || out.mv.state == AUTO_MV_STOPPED || out.state != AUTO_HOVER) {
        res->tPlanEnd = t;
        res->planEnd = out.mv.end;
      } else {
        res->planDh = fmaxf(res->planDh, fabsf(sim.h - planH0));
        res->planDhEst = fmaxf(res->planDhEst, fabsf(vo.h - planHEst0));
      }
    }
    if (out.state == AUTO_IDLE && prev != AUTO_IDLE && prev != AUTO_ARMED) {
      res->endWhy = out.reason;
      break;
    }
    if ((out.state == AUTO_LAND || out.state == AUTO_KILLED) && !inLand) {
      inLand = true;
      res->landWhy = out.reason;
      res->landEntryT = t;
      res->landEntryAir = (float)out.airMs * 0.001f;
      res->trueAirAtLand = res->liftTrue >= 0.0f ? t - res->liftTrue : 0.0f;
      res->hAtLand = sim.h;
      res->waitS = waitStartMs != 0 ? (float)(in.nowMs - waitStartMs) * 0.001f : 0.0f;
      res->lastActive = out.state;
      landMs = in.nowMs;
      if (sc->tripAtLandS >= 0.0f) {
        tripMs = landMs + (uint32_t)(sc->tripAtLandS * 1000.0f);
      }
    }
    if (out.state == AUTO_KILLED && !res->killed) {
      res->killWhy = out.reason;
    }
    if (out.state == AUTO_KILLED) {
      res->killed = true;
      res->lastActive = AUTO_KILLED;
      res->landWhy = res->landWhy == AUTO_RS_NONE ? out.reason : res->landWhy;
    }
    if (sc->tripAtTdOk && inLand && tripMs == 0 && out.state == AUTO_LAND && vo.tdThrustOk && vo.phase == VZ_LAND) {
      tripMs = in.nowMs + 2u;   /* contact sign on the next step, tilt from the one after */
    }
    if (tripMs != 0 && !tripping && in.nowMs + 1u >= tripMs && out.state == AUTO_LAND) {
      tripping = true;
      res->tripped = true;
      res->tdOkAtTrip = vo.tdThrustOk;
      res->hAtTrip = sim.h;
    }
    prev = out.state;

    float thr = 0.0f;
    if (vzActive) {
      vzInput_t vi = {
        .dt = 0.001f, .cmd = out.vzCmd, .accZ = sim.accZ, .gravZ = 1.0f, .tiltDeg = in.tiltDeg,
        .gyroMaxDps = 0.2f, .vbat = vbat, .thrustApplied = applied, .fromSim = false,
        .camHealthy = sim.camHealthy, .camVzValid = sim.camVzValid, .camVz = sim.camVz, .camBias = sim.camBias,
        .landFastOk = autoFsmLandFastReason(out.reason),   /* as autonomy.c vzRun() */
        .hush = out.hush, .resume = out.resume, .blind = out.blind,
      };
      vzStep(&vz, &vi, &vo);
      if (vo.phase == VZ_RAMP && res->rampT < 0.0f) {
        res->rampT = t;
      }
      if (vo.liftoffNow) {
        res->liftDet = t;
      }
      if (vo.phase == VZ_DONE && res->landS == 0.0f && inLand) {
        res->landS = t - res->landEntryT;
        res->vzLand = vo.landReason;
        res->aborted = vo.reason == VZ_R_ABORTED;
      }
    }
    switch (out.thrust) {
      case AUTO_THR_VERT_CTRL:
      case AUTO_THR_VERT_LAND:
        thr = vo.thrust;
        break;
      default:
        thr = 0.0f;
        break;
    }
    if (thr > out.thrustMax) {
      thr = out.thrustMax;
    }
    const bool groundBefore = sim.liftT < 0.0f;
    vzSimStep(&sim, &pc, thr, 0.001f);
    if (groundBefore && sim.liftT >= 0.0f) {
      res->liftTrue = sim.liftT;
    }
    if (!sim.onGround) {
      res->airS += 0.001f;
    }
    applied = thr;
    in.nowMs++;
  }
  res->photos = res->captures > 0;
  res->hMax = sim.hMax;
  res->impactV = sim.impactV;
  res->held = vo.holdReached;
  /* What the glue does in FLIGHT once the state machine is back in IDLE. */
  res->kLearned = vc.kPrior;
  res->learned = vzActive && vzLearnK(&vz, &res->kLearned);
}

/* ---- tests -------------------------------------------------------------- */

static int cmpf(const void *a, const void *b);

/* "Always slow land" (user decision): the phone stick during the climb, and
 * during the RAMP of a lift-off hidden by a negative offset. A2a landed from
 * CLIMB on T_ref = T_lo (~0.7 x hover) and aborted a RAMP below its plateau
 * at 60000/s: the drone fell. A2b: T_ref from the CLIMB average, and the
 * slow abort from abortSlowRel x kPrior / Vf. */
static void testSlowLandFromClimb(void)
{
  static const float at[4] = { 0.35f, 0.50f, 0.70f, 0.90f };
  int n = 0, ok = 0;
  float worstImp = 0.0f, worstRise = 0.0f;
  for (int j = 0; j < 4; j++) {
    float wi = 0.0f;
    for (int i = 0; i < 20; i++) {
      scen_t s;
      scenDefault(&s);
      s.camFirmware = true;
      s.vRest = 4.05f;
      s.appAfterLiftS = at[j];
      sortie_t r;
      runSortie(&s, 7700u + 20u * (uint32_t)j + (uint32_t)i, &r);
      n++;
      ok += r.landWhy == AUTO_RS_APP_INPUT && !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN;
      wi = fmaxf(wi, r.impactV);
      worstRise = fmaxf(worstRise, r.hMaxAfterApp - r.hAtApp);
    }
    worstImp = fmaxf(worstImp, wi);
    printf("info: phone stick %.2f s after the detected lift-off: impact max %.2f m/s\n", at[j], wi);
  }
  CHECK(ok == n, "phone stick in CLIMB: LAND APP_INPUT, touchdown, never KILLED (%d/%d)", ok, n);
  CHECK(worstImp <= 0.65f, "phone stick in CLIMB: impact max %.2f m/s (<= 0.65)", worstImp);
  CHECK(worstRise <= 0.10f, "phone stick in CLIMB: rise after the stick %.3f m (<= 0.10)", worstRise);

  /* In RAMP: residual -10..-20 mg (the hidden lift-offs), K -12..+12 %. */
  for (int v = 0; v < 2; v++) {
    const float rel = v == 0 ? 1.01f /* only from the plateau, as A2a */ : 0.0f /* the default */;
    int nr = 0, killed = 0, up = 0, okR = 0;
    float wi = 0.0f, wiUp = 0.0f, wh = 0.0f;
    for (int b = 0; b < 3; b++) {
      for (int k = 0; k < 3; k++) {
        for (int j = 0; j < 8; j++) {
          for (int i = 0; i < 4; i++) {
            scen_t s;
            scenDefault(&s);
            s.camFirmware = true;
            s.vRest = 4.05f;
            s.bias = -0.010f - 0.005f * (float)b;
            s.kerr = -0.12f + 0.12f * (float)k;
            s.appAfterRampS = 0.30f + 0.30f * (float)j;
            s.abortSlowRel = rel > 1.0f ? 1.0f : 0.0f;
            sortie_t r;
            runSortie(&s, 7800u + (uint32_t)(((b * 3 + k) * 8 + j) * 4 + i), &r);
            if (r.liftDet >= 0.0f && r.liftDet <= r.appT) {
              continue;   /* detected before the stick: a CLIMB row */
            }
            nr++;
            killed += r.killed;
            okR += r.landWhy == AUTO_RS_APP_INPUT && !r.killed;
            up += r.upAtApp;
            wi = fmaxf(wi, r.impactV);
            wh = fmaxf(wh, r.hMaxAfterApp);
            if (r.upAtApp) {
              wiUp = fmaxf(wiUp, r.impactV);
            }
          }
        }
      }
    }
    printf("info: phone stick in RAMP (%s), -10..-20 mg, K -12..+12 %%: %d sorties, %d with the drone already up "
           "(undetected), impact max %.2f m/s (drone up at the stick: %.2f), true peak max %.2f m, KILLED %d\n",
           v == 0 ? "A2a: slow abort only from the plateau" : "A2b: slow abort from 0.60 x kPrior / Vf", nr, up, wi, wiUp,
           wh, killed);
    if (v == 1) {
      CHECK(okR == nr && killed == 0, "phone stick in RAMP: LAND APP_INPUT, never KILLED (%d/%d)", okR, nr);
    }
  }
}

static int cmpf(const void *a, const void *b)
{
  const float x = *(const float *)a, y = *(const float *)b;
  return x < y ? -1 : (x > y ? 1 : 0);
}

static float pct(float *v, int n, float p)
{
  qsort(v, (size_t)n, sizeof(float), cmpf);
  int i = (int)(p * (float)(n - 1) + 0.5f);
  return v[i < 0 ? 0 : (i >= n ? n - 1 : i)];
}

/* A late lift-off (residual -20 mg, no camera; finding: the true airborne
 * time passed the pre-wait limit by ~1.1 s). This case does not reach the
 * air clock: the detector is late or never fires, so the controller's own
 * RAMP limit (NO_LIFTOFF_TIME, 2.5 s) or the camera rule's SINK (HOVER_NO_CAM
 * until 2026-10-15, at the same time: no camera, no camera-measured bound) lands every sortie
 * first. It bounds the true lift-off to LAND for such sorties (at A2b's
 * 4.5 s, kept when revision 5 raised the pre-wait clock to 6 s: this bound
 * does not come from the clock); the air clock itself is testAirClock's (and
 * test_auto_fsm.c's) subject. */
static void testLateLiftoff(void)
{
  const int n = 200;
  int ok = 0, lifted = 0, air = 0, vz = 0, noCam = 0, other = 0;
  int vzWhy[32] = { 0 };
  float worst = 0.0f, worstH = 0.0f;
  for (int i = 0; i < n; i++) {
    scen_t s;
    scenDefault(&s);
    s.bias = -0.020f;
    s.camWakeS = -1.0f;
    sortie_t r;
    runSortie(&s, 7000u + (uint32_t)i, &r);
    lifted += r.liftTrue >= 0.0f;
    air += (r.evMask & EV_BIT(AUTO_EV_TAKEOFF_SLOW)) != 0u;
    noCam += r.landWhy == AUTO_RS_SINK;
    if (r.landWhy == AUTO_RS_VZ) {
      vz++;
      vzWhy[r.vzLand & 31]++;
    }
    other += r.landWhy != AUTO_RS_SINK && r.landWhy != AUTO_RS_VZ;
    ok += r.trueAirAtLand <= 4.501f && r.landEntryAir <= 4.501f;
    worst = fmaxf(worst, r.trueAirAtLand);
    worstH = fmaxf(worstH, r.hMax);
  }
  CHECK(ok == n, "residual -20 mg, no camera: true lift-off to LAND <= 4.5 s in every sortie (%d/%d, worst %.2f s)", ok,
        n, worst);
  printf("info: residual -20 mg, no camera: %d/%d left the ground; LAND by the controller %d (", lifted, n, vz);
  const char *sep = "";
  for (int k = 0; k < 32; k++) {
    if (vzWhy[k] > 0) {
      printf("%s%s %d", sep, vzReasonName((vzReason_t)k), vzWhy[k]);
      sep = ", ";
    }
  }
  printf("), SINK %d, TAKEOFF_SLOW reported %d, other %d; true lift-off to LAND max %.2f s, true peak max %.2f m\n", noCam,
         air, other, worst, worstH);
}

/* The pre-wait clock on the real controller: it starts at the first RAMP
 * step, not at the detected lift-off (0.10-0.28 s after the true one at
 * these offsets, never at -20 mg). Warm camera, so the camera rules do not
 * land first, and the longest photo set (3 x 1 s), so the photos are not
 * done by the pre-wait limit (6 s). Until 2026-10-15 the clock landed the
 * sortie (AIR_TIME); since then ("Claude decides", docs/autonomy-a2b.md 22)
 * it reports TAKEOFF_SLOW and starts the wait anyway, the drone hovering for
 * the Mac: the wait starts at the limit after the first RAMP step, never
 * later, and nothing lands. Started at the detected lift-off, the clock ran
 * late in every sortie (A2a, 4.5 s: 4.60-4.78 s on a mutated auto_fsm.c). */
static void testAirClock(void)
{
  const int n = 100;
  int slow = 0, lifted = 0, okRamp = 0, okHover = 0;
  float worstRamp = 0.0f, minRamp = 99.0f, lagMin = 99.0f, lagMax = 0.0f;
  const float airMaxS = (float)AUTO_AIR_MAX_DEFAULT_MS * 0.001f;   /* 6 s (4.5 s before revision 5) */
  for (int i = 0; i < n; i++) {
    scen_t s;
    scenDefault(&s);
    s.bias = -0.005f * (float)(i % 3);   /* 0, -5 and -10 mg residual: later detections */
    s.photoN = AUTO_PHOTO_N_MAX;
    s.photoGapMs = AUTO_PHOTO_GAP_MAX_MS;
    s.waitMs = 4000;
    sortie_t r;
    runSortie(&s, 7400u + (uint32_t)i, &r);
    lifted += r.liftTrue >= 0.0f;
    if (r.liftDet >= 0.0f && r.liftTrue >= 0.0f) {
      lagMin = fminf(lagMin, r.liftDet - r.liftTrue);
      lagMax = fmaxf(lagMax, r.liftDet - r.liftTrue);
    }
    if (!(r.evMask & EV_BIT(AUTO_EV_TAKEOFF_SLOW))) {
      continue;
    }
    slow++;
    const float fromRamp = r.tWait - r.rampT;
    okRamp += r.rampT >= 0.0f && r.tWait >= 0.0f && fromRamp <= airMaxS + 0.002f && fromRamp >= airMaxS - 0.002f;
    /* the wait ran its 4 s and the Mac landed: the hover went on past the clock, nothing landed it */
    okHover += r.landWhy == RS_WAIT_END && r.macWaitLand && r.landEntryT - r.tWait >= 3.99f && !r.killed &&
               r.endWhy == AUTO_RS_TOUCHDOWN;
    worstRamp = fmaxf(worstRamp, fromRamp);
    minRamp = fminf(minRamp, fromRamp);
  }
  CHECK(slow >= n * 3 / 4, "photos 3 x 1 s, warm camera: the %.1f s pre-wait clock reached, TAKEOFF_SLOW in most "
        "sorties (%d/%d)", airMaxS, slow, n);
  CHECK(okRamp == slow && okHover == slow,
        "TAKEOFF_SLOW: the wait starts %.1f s after the first RAMP step (%d/%d, %.3f..%.3f s), the hover goes on to "
        "the Mac's LAND (%d/%d)", airMaxS, okRamp, slow, minRamp, worstRamp, okHover, slow);
  printf("info: pre-wait clock, photos 3 x 1 s, residual 0/-5/-10 mg, warm camera: %d/%d lifted off (detected "
         "%.2f..%.2f s late), TAKEOFF_SLOW %d; first RAMP step to the wait %.3f..%.3f s\n", lifted, n, lagMin, lagMax,
         slow, minRamp, worstRamp);
}

/* Learning the hover gain over chains of sorties, each starting from the
 * kPrior the previous one learned (what the glue stores in NVS), with the
 * camera hold on. Finding: with the hold's running Kfrz as the sample, a
 * camera offset that sinks the drone (c > 0; the study saw +0.04..+0.12 m/s
 * on 5 of 17 take-offs) taught a gain lowered by ground effect sortie after
 * sortie (0.86 x true after 4 sorties at c +0.06), and from then on every
 * sortie landed K_RANGE in CLIMB. The sample is now Khat at holdLoopS, and a
 * K_RANGE, HOLD_CAPPED or EST_RANGE sortie is refused. */
typedef struct {
  float c;          /* camera offset, m/s */
  float kTrue;      /* the drone's gain */
  float kStart;     /* the first sortie's kPrior (NVS vzK) */
  float kLo, kHi;   /* every kPrior in the chain within [kLo, kHi] x kTrue ... */
  float kEnd;       /* ... and the last one >= kEnd x kTrue */
} chain_t;

static void testLearnChain(void)
{
  static const chain_t chains[] = {
    { 0.05f, VZ_K_DEFAULT, VZ_K_DEFAULT, 0.95f, 1.05f, 0.95f },
    { 0.06f, VZ_K_DEFAULT, VZ_K_DEFAULT, 0.95f, 1.05f, 0.95f },
    { -0.05f, VZ_K_DEFAULT, VZ_K_DEFAULT, 0.95f, 1.05f, 0.95f },
    { 0.0f, VZ_K_DEFAULT, VZ_K_DEFAULT, 0.95f, 1.05f, 0.95f },
    /* a stale prior: an NVS vzK still holding the A2a default (149500) on
     * today's drone (165000, ratio 1.104 as in A2a's chain), c at the study's
     * median: it must still learn upward */
    { 0.03f, VZ_K_DEFAULT, 149500.0f, 0.90f, 1.05f, 0.95f },
  };
  const int nSorties = 10;
  for (unsigned ci = 0; ci < sizeof(chains) / sizeof(chains[0]); ci++) {
    const chain_t *ch = &chains[ci];
    float k = ch->kStart, kMin = k, kMax = k;
    int learned = 0, why[32] = { 0 };
    bool lastHeld = false;
    printf("info: learning chain, c %+.2f m/s, true K %.0f: kPrior %.0f", ch->c, ch->kTrue, k);
    for (int i = 0; i < nSorties; i++) {
      scen_t s;
      scenDefault(&s);
      s.kPrior = k;
      s.kTrue = ch->kTrue;
      s.camC = ch->c;
      sortie_t r;
      runSortie(&s, 9100u + 20u * ci + (uint32_t)i, &r);
      learned += r.learned;
      why[r.vzLand & 31]++;
      k = r.kLearned;
      kMin = fminf(kMin, k);
      kMax = fmaxf(kMax, k);
      lastHeld = r.held;
      printf(" %s%.0f", r.learned ? "" : "=", k);
    }
    printf(" (%.3f x true at the end, %.3f..%.3f); learned %d/%d; controller LAND by:", k / ch->kTrue,
           kMin / ch->kTrue, kMax / ch->kTrue, learned, nSorties);
    for (int j = 0; j < 32; j++) {
      if (why[j] > 0) {
        printf(" %s %d", vzReasonName((vzReason_t)j), why[j]);
      }
    }
    printf("\n");
    CHECK(kMin >= ch->kLo * ch->kTrue && kMax <= ch->kHi * ch->kTrue && k >= ch->kEnd * ch->kTrue,
          "chain c %+.2f m/s, true K %.0f: kPrior stays within %.2f..%.2f x true (%.3f..%.3f), ends >= %.2f (%.3f)",
          ch->c, ch->kTrue, ch->kLo, ch->kHi, kMin / ch->kTrue, kMax / ch->kTrue, ch->kEnd, k / ch->kTrue);
    CHECK(lastHeld, "chain c %+.2f m/s, true K %.0f: the last sortie still reaches HOLD", ch->c, ch->kTrue);
  }
}

/* Finding: a tilt trip in LAND counted as touchdown whenever tdThrustOk was
 * set, and the real controller sets it in mid-air. A2a set it from the first
 * step of every LAND from CLIMB (T_ref = T_lo); A2b's CLIMB average arms it
 * near the ground, except on a drone lighter than its stored gain: with
 * K_ref = kPrior 12 % high, T_ref <= tdThrust / landStart x kPrior / Vf from
 * the first LAND step (and after a capped step or inside tHoldTauS). The
 * tilt starts 2 ms after the first LAND step with tdThrustOk, so "armed"
 * checks the scenario (the condition still held when the tilt started), not
 * a rate. Every one of the 20 sorties must be a Mac LAND whose tilt starts
 * in the air: A2a's premise height (true h >= 0.09 m at the trip) is checked
 * on each, not used to filter (measured: all 20 at >= 0.16 m). At 0.35 s
 * after lift-off 2 of 20 lose tdThrustOk within the 50 ms tilt persistence
 * and end KILLED TILT with the contact sign too, which is the rule; 0.70 s
 * keeps the thrust condition through the trip in every sortie. */
static void testTripInAir(void)
{
  enum { N_TRIP = 20 };
  int killed = 0, armed = 0, td = 0, n = 0, other = 0, noTrip = 0, low = 0;
  float hMin = 9.0f;
  for (int i = 0; i < N_TRIP; i++) {
    scen_t s;
    scenDefault(&s);
    s.kerr = -0.12f;              /* the stored gain 12 % high (the user range's edge) */
    s.landAfterLiftS = 0.70f;     /* a Mac LAND in the climb */
    s.tripAtTdOk = true;          /* then a flip as soon as the touchdown thrust condition holds */
    sortie_t r;
    runSortie(&s, 7300u + (uint32_t)i, &r);
    if (r.landWhy != AUTO_RS_LAND_CMD) {
      other++;                    /* the sortie did not reach the Mac LAND: fails the count below */
      continue;
    }
    if (!r.tripped) {
      noTrip++;                   /* tdThrustOk never set in LAND: no trip to test */
      continue;
    }
    n++;
    low += !(r.hAtTrip >= 0.09f);
    armed += r.tdOkAtTrip;
    killed += r.killed && r.killWhy == AUTO_RS_TILT;
    hMin = fminf(hMin, r.hAtTrip);
    s.contactBeforeTrip = true;   /* the same with a 2 g touchdown sign just before */
    runSortie(&s, 7300u + (uint32_t)i, &r);
    td += !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN_TRIP;
  }
  CHECK(n == N_TRIP && armed == n,
        "LAND from CLIMB, K -12 %%: every sortie a Mac LAND with tdThrustOk set at the trip (%d/%d; %d other LAND, %d no trip)",
        armed, N_TRIP, other, noTrip);
  CHECK(n > 0 && low == 0, "the premise: every trip in the air, true h >= 0.09 m (%d of %d below, min %.3f m)", low, n,
        hMin);
  CHECK(killed == n, "a tilt trip in the air then is KILLED TILT, not a touchdown (%d/%d, true h >= %.2f m)", killed, n,
        hMin);
  CHECK(td == n, "with a 2 g contact sign just before it, it is TOUCHDOWN_TRIP (%d/%d)", td, n);
  printf("info: tilt trip on the first armed step of a LAND 0.70 s after lift-off, K -12 %%: %d sorties, tdThrustOk set in %d, true h at the trip >= %.2f m, KILLED %d\n",
         n, armed, hMin, killed);
}

/* Whole sorties over the user range with the real state machine: n uniform
 * draws (the percentiles and the uniform maxima), then the range's 8 corners
 * (offset x K error x camera offset c) with CORNER_SEEDS noise seeds each,
 * in their own maxima. KILLED, the LAND times and the air clock cover both. */
#define CORNER_SEEDS 100

typedef struct {
  int n, waited, killed, aborted, slow, why[NWHY];   /* slow: TAKEOFF_SLOW reported (the pre-wait clock; AIR_TIME before) */
  float worstH, worstImpact, worstLand, worstAbort, worstTrueAir, worstAirAtLand, worstAirS;
  float p95H, p50Land, p95Land;
  int cornerN, cornerWaited, cornerAborted, cornerWhy[NWHY];
  float cornerH, cornerImpact;
  float cornerHMax[8], cornerImpMax[8];   /* per corner */
} stats_t;

/* bias, K error and camera offset uniform in [-span, +span] around a centre */
typedef struct {
  float biasMid, biasSpan, kSpan, cSpan;
} range_t;

static const range_t userRange = { 0.0f, 0.020f, 0.12f, 0.05f };
/* The camera study's bias truth: the in-flight offset vs g0 is +1.4 +/- 2.2 mg,
 * so +3.7 mg after the default bAir (-2.3 mg); a learned kPrior within 5 %;
 * the study's camera offset c in flights 16-21 (median ~0.03 m/s). */
static const range_t realRange = { 0.0037f, 0.0044f, 0.05f, 0.03f };
/* Since the in-flight gyro-bias fix (1996b09): the 8 camera hovers of
 * 2026-10-12a drifted +0.003..+0.010 m/s (h at touchdown / camera time). */
static const range_t fixRange = { 0.0037f, 0.0044f, 0.05f, 0.010f };

/* Corner j: bit 0 the offset, bit 1 the K error, bit 2 c (set: the + side). */
static void cornerOf(const range_t *rg, int j, scen_t *s)
{
  s->bias = rg->biasMid + ((j & 1) ? rg->biasSpan : -rg->biasSpan);
  s->kerr = (j & 2) ? rg->kSpan : -rg->kSpan;
  s->camC = (j & 4) ? rg->cSpan : -rg->cSpan;
}

static void addSortie(stats_t *m, const sortie_t *r, bool corner)
{
  m->killed += r->killed;
  if (r->aborted) {
    m->worstAbort = fmaxf(m->worstAbort, r->landS);
  } else {
    m->worstLand = fmaxf(m->worstLand, r->landS);
  }
  m->worstTrueAir = fmaxf(m->worstTrueAir, r->trueAirAtLand);
  m->worstAirAtLand = fmaxf(m->worstAirAtLand, r->landEntryAir);
  m->worstAirS = fmaxf(m->worstAirS, r->airS);
  m->slow += (r->evMask & EV_BIT(AUTO_EV_TAKEOFF_SLOW)) != 0u;
  if (corner) {
    m->cornerN++;
    m->cornerWaited += r->waited;
    m->cornerAborted += r->aborted;
    m->cornerWhy[r->landWhy < NWHY ? r->landWhy : 0]++;
    m->cornerH = fmaxf(m->cornerH, r->hMax);
    m->cornerImpact = fmaxf(m->cornerImpact, r->impactV);
  } else {
    m->waited += r->waited;
    m->aborted += r->aborted;
    m->why[r->landWhy < NWHY ? r->landWhy : 0]++;
    m->worstH = fmaxf(m->worstH, r->hMax);
    m->worstImpact = fmaxf(m->worstImpact, r->impactV);
  }
}

static void printWhy(const int *why)
{
  for (int k = 0; k < NWHY; k++) {
    if (why[k] > 0) {
      printf(" %s %d", autoReasonName((autoReason_t)k), why[k]);
    }
  }
  printf("\n");
}

static float mcVRest = 0.0f, mcVmin = 0.0f;   /* > 0: every sortie of runMc rests at / arms with these */

static void runMc(const char *label, int n, const range_t *rg, float camWakeS, float camLossS, uint32_t waitMs,
                  stats_t *m)
{
  float *hs = malloc(sizeof(float) * (size_t)n), *ls = malloc(sizeof(float) * (size_t)n);
  int nl = 0;
  memset(m, 0, sizeof(*m));
  m->n = n;
  rngState = 4242u;
  for (int i = 0; i < n; i++) {
    scen_t s;
    scenDefault(&s);
    s.bias = rg->biasMid + rg->biasSpan * (2.0f * urand() - 1.0f);
    s.kerr = rg->kSpan * (2.0f * urand() - 1.0f);
    s.camC = rg->cSpan * (2.0f * urand() - 1.0f);
    s.camWakeS = camWakeS;
    s.camLossS = camLossS;
    s.waitMs = waitMs;
    s.vRest = mcVRest;
    s.vminOverride = mcVmin;
    sortie_t r;
    runSortie(&s, 8000u + (uint32_t)i, &r);
    addSortie(m, &r, false);
    hs[i] = r.hMax;
    if (r.landS > 0.0f && !r.aborted) {
      ls[nl++] = r.landS;
    }
  }
  m->p95H = pct(hs, n, 0.95f);
  m->p50Land = nl > 0 ? pct(ls, nl, 0.5f) : 0.0f;
  m->p95Land = nl > 0 ? pct(ls, nl, 0.95f) : 0.0f;
  free(hs);
  free(ls);
  for (int j = 0; j < 8; j++) {
    for (int k = 0; k < CORNER_SEEDS; k++) {
      scen_t s;
      scenDefault(&s);
      cornerOf(rg, j, &s);
      s.camWakeS = camWakeS;
      s.camLossS = camLossS;
      s.waitMs = waitMs;
      s.vRest = mcVRest;
      s.vminOverride = mcVmin;
      sortie_t r;
      runSortie(&s, 9000u + 100u * (uint32_t)j + (uint32_t)k, &r);
      addSortie(m, &r, true);
      m->cornerHMax[j] = fmaxf(m->cornerHMax[j], r.hMax);
      m->cornerImpMax[j] = fmaxf(m->cornerImpMax[j], r.impactV);
    }
  }
  CHECK(m->cornerN == 8 * CORNER_SEEDS, "the corner pass ran (%d of %d)", m->cornerN, 8 * CORNER_SEEDS);
  printf("info: %s: n %d, hover wait reached %d, RAMP aborts %d; LAND by:", label, n, m->waited, m->aborted);
  printWhy(m->why);
  printf("info:   true peak p95 %.2f max %.2f m, impact max %.2f m/s; LAND to touchdown p50 %.2f p95 %.2f\n", m->p95H,
         m->worstH, m->worstImpact, m->p50Land, m->p95Land);
  printf("info:   corners (8 x %d seeds): true peak max %.2f m, impact max %.2f m/s, hover wait reached %d, RAMP aborts %d; LAND by:",
         CORNER_SEEDS, m->cornerH, m->cornerImpact, m->cornerWaited, m->cornerAborted);
  printWhy(m->cornerWhy);
  printf("info:   per corner (offset mg / K %% / c m/s: peak m, impact m/s):");
  for (int j = 0; j < 8; j++) {
    scen_t s;
    cornerOf(rg, j, &s);
    printf("%s %+.1f/%+.0f/%+.2f: %.2f, %.2f", j == 0 ? "" : ";", s.bias * 1000.0f, s.kerr * 100.0f, s.camC,
           m->cornerHMax[j], m->cornerImpMax[j]);
  }
  printf("\n");
  printf("info:   all %d: KILLED %d; LAND to touchdown max %.2f s (aborts %.2f s); at LAND: air clock max %.2f s, true airborne max %.2f s; true airborne in all max %.2f s\n",
         m->n + m->cornerN, m->killed, m->worstLand, m->worstAbort, m->worstAirAtLand, m->worstTrueAir, m->worstAirS);
}

static void testUserRange(void)
{
  static stats_t m[7];
  runMc("real FSM, user range, warm camera, wait 5 s", 200, &userRange, 1.5f, -1.0f, AUTO_HOVER_WAIT_DEFAULT_MS, &m[0]);
  runMc("real FSM, user range, warm camera, wait 12 s (the evening's max)", 200, &userRange, 1.5f, -1.0f,
        EVENING_WAIT_MAX_MS, &m[1]);
  runMc("real FSM, user range, no camera", 200, &userRange, -1.0f, -1.0f, AUTO_HOVER_WAIT_DEFAULT_MS, &m[2]);
  runMc("real FSM, user range, cold camera (3.0 s)", 200, &userRange, 3.0f, -1.0f, AUTO_HOVER_WAIT_DEFAULT_MS, &m[3]);
  runMc("real FSM, user range, camera lost 3 s after lift-off", 200, &userRange, 1.5f, 3.0f,
        AUTO_HOVER_WAIT_DEFAULT_MS, &m[4]);
  runMc("real FSM, realistic (offset +3.7 +/- 4.4 mg, K +/-5 %, c +/-0.03 m/s), warm camera, wait 5 s", 200,
        &realRange, 1.5f, -1.0f, AUTO_HOVER_WAIT_DEFAULT_MS, &m[5]);
  /* Stress: a battery resting at 3.90 V cannot arm FLIGHT at the default
   * vmin (3.80 V); G3 is overridden here (vmin 3.70) to keep the sag model's
   * low-battery corner covered (more thrust per K, a lower thrust ceiling). */
  mcVRest = 3.90f;
  mcVmin = AUTO_VMIN_LO;
  runMc("real FSM, user range, warm camera, wait 5 s, STRESS 3.90 V (G3 overridden)", 200, &userRange, 1.5f, -1.0f,
        AUTO_HOVER_WAIT_DEFAULT_MS, &m[6]);
  mcVRest = mcVmin = 0.0f;
  static const uint32_t waitRow[7] = { AUTO_HOVER_WAIT_DEFAULT_MS, EVENING_WAIT_MAX_MS, AUTO_HOVER_WAIT_DEFAULT_MS,
                                       AUTO_HOVER_WAIT_DEFAULT_MS, AUTO_HOVER_WAIT_DEFAULT_MS,
                                       AUTO_HOVER_WAIT_DEFAULT_MS, AUTO_HOVER_WAIT_DEFAULT_MS };
  for (int k = 0; k < 7; k++) {
    CHECK(m[k].killed == 0, "row %d: no sortie ends KILLED (%d)", k, m[k].killed);
    /* the wait starts by the pre-wait clock at the latest (TAKEOFF_SLOW: it starts anyway), the Mac lands at its end */
    const float airLim = (float)(AUTO_AIR_MAX_DEFAULT_MS + waitRow[k]) * 0.001f + 0.002f;
    CHECK(m[k].worstAirAtLand <= airLim && m[k].worstLand <= 6.0f && m[k].worstAbort <= 6.0f,
          "row %d: LAND by the pre-wait clock plus the wait (%.2f s <= %.2f), touchdown within 6 s (%.2f s, aborts "
          "%.2f s)", k, m[k].worstAirAtLand, airLim, m[k].worstLand, m[k].worstAbort);
  }
  CHECK(m[0].why[RS_WAIT_END] >= 100, "warm camera: most sorties wait and the Mac lands them at its end (%d)",
        m[0].why[RS_WAIT_END]);
  /* The longest wait (12 s) after the default 6 s pre-wait limit: the Mac's LAND at its end; the wait never started
   * by the pre-wait clock (TAKEOFF_SLOW; AIR_TIME before 2026-10-15). */
  CHECK(m[1].why[RS_WAIT_END] >= 100 && m[1].slow == 0 &&
        m[1].worstAirAtLand <= (float)(AUTO_AIR_MAX_DEFAULT_MS + EVENING_WAIT_MAX_MS) * 0.001f,
        "wait 12 s: the Mac's LAND at its end (%d), never TAKEOFF_SLOW (%d), air clock at LAND %.2f s",
        m[1].why[RS_WAIT_END], m[1].slow, m[1].worstAirAtLand);
  CHECK(m[2].waited == 0 && m[2].cornerWaited == 0 && m[2].why[AUTO_RS_SINK] > 0,
        "no camera: never waits (corners included), SINK (HOVER_NO_CAM before 2026-10-15) (%d)",
        m[2].why[AUTO_RS_SINK]);
  CHECK(m[4].why[AUTO_RS_SINK] > 0, "camera lost: SINK (%d)", m[4].why[AUTO_RS_SINK]);
  CHECK(m[5].why[RS_WAIT_END] >= 190 && m[5].cornerWhy[RS_WAIT_END] >= m[5].cornerN * 95 / 100,
        "realistic: the Mac's LAND at the wait's end (%d/%d, corners %d/%d)", m[5].why[RS_WAIT_END], m[5].n,
        m[5].cornerWhy[RS_WAIT_END], m[5].cornerN);
  /* True peak and impact over the uniform draws and the corners: the
   * measured maxima (docs/autonomy-a2.md section 4.9) plus about 10 %. The
   * no-camera worst case is the corner -20 mg, K -12 % (2.08 m, 2.20 m/s
   * over 100 seeds), outside the uniform draw's 1.84 m. The 12 s wait
   * lets the camera's velocity offset (c up to 0.05 m/s) move the drone
   * longer: 1.07 m (the 8 s wait of A2b ended at the 10 s clock: 0.79 m).
   * 2026-10-15 (docs/autonomy-a2b.md 22): a camera HOLD no longer lands K_RANGE at khatMinRel (its floor is
   * VZ_KHAT_HOLD_LO_REL), so the corners K -12 % with c -0.05 m/s hover on and climb at c to the wait's end: warm 5 s
   * corners 0.88 m, 12 s 1.28, cold 1.16, stress 0.88 (the uniform draws 0.68, 1.06, 0.85, 0.69); limits those + ~10 %.
   * The camera lost 3 s after lift-off no longer lands HOVER_NO_CAM at once: the hold goes on open loop under capQ
   * (CAM_LOST) up to AUTO_CAM_LOST_MAX_MS, then SINK, or the Mac's LAND at the wait's end comes first; the corner
   * -20 mg / K -12 % / c -0.05 m/s, which climbs at c while the camera is in use, peaks 0.73 m (0.61 with the
   * immediate LAND; the same corner with the camera in use to the wait's end: 0.88). */
  static const float hLim[7] = { 0.95f, 1.40f, 2.2f, 1.30f, 0.80f, 0.70f, 0.95f };
  static const float impLim[7] = { 1.2f, 1.2f, 2.3f, 1.5f, 1.2f, 1.0f, 1.2f };
  static const char *const row[7] = { "warm camera, wait 5 s", "warm camera, wait 12 s", "no camera", "cold camera",
                                      "camera lost", "realistic", "stress 3.90 V" };
  for (int k = 0; k < 7; k++) {
    const float h = fmaxf(m[k].worstH, m[k].cornerH), imp = fmaxf(m[k].worstImpact, m[k].cornerImpact);
    CHECK(h <= hLim[k] && imp <= impLim[k],
          "%s: true peak <= %.2f m (uniform %.2f, corners %.2f), impact <= %.2f m/s (uniform %.2f, corners %.2f)",
          row[k], hLim[k], m[k].worstH, m[k].cornerH, impLim[k], m[k].worstImpact, m[k].cornerImpact);
  }
}

/* The user's decision of 2026-10-12 (evening): take-off height up to
 * 0.60 m and a 10 s hover wait (NVS autowait 10000). FLIGHT timing as the
 * firmware runs it: the camera awake from the ARM (camNoStandby), the Mac
 * tool's default TAKEOFF (1 photo; the set done one gap, 300 ms, after it
 * starts), the default autoair. heightCm 40 (the sorties flown so far) and
 * 60 on the same draws, realistic range and user range, 200 each: the
 * pre-wait air clock's margin, the hover wait, the landing, the impact at
 * 0.60 m against 0.40 m.
 * Then at 0.60 m: the 18 s air clock reached (the longest take-off photo
 * set, 3 x 1 s, under NVS autoair 8 s and autowait 12 s), the hover without
 * a camera height estimate, and the phone stick in the longer climb. */
#define HI_N 200

typedef struct {
  int n, waited, killed, timeout, airTime, climbTimeout, noCam, kRange, why[NWHY];
  int nw, nt;
  float imp[HI_N], peak[HI_N], hLand[HI_N], air[HI_N], waitAir[HI_N], landAir[HI_N];
  float hoverAirMax, landSMax;
} hiStats_t;

static void hiRun(const range_t *rg, uint16_t heightCm, uint32_t waitMs, uint8_t photoN, uint16_t gapMs,
                  uint16_t airNvsMs, bool camera, int n, hiStats_t *m)
{
  memset(m, 0, sizeof(*m));
  rngState = 6600u;
  for (int i = 0; i < n && i < HI_N; i++) {
    scen_t s;
    scenDefault(&s);
    s.bias = rg->biasMid + rg->biasSpan * (2.0f * urand() - 1.0f);
    s.kerr = rg->kSpan * (2.0f * urand() - 1.0f);
    s.camC = camera ? rg->cSpan * (2.0f * urand() - 1.0f) : 0.0f;
    s.camFirmware = camera;
    s.camWakeS = camera ? s.camWakeS : -1.0f;
    s.heightCm = heightCm;
    s.waitMs = waitMs;
    s.photoN = photoN;
    s.photoGapMs = gapMs;
    s.airNvsMs = airNvsMs;
    sortie_t r;
    runSortie(&s, 6600u + (uint32_t)i, &r);
    m->n++;
    m->waited += r.waited;
    m->killed += r.killed;
    m->timeout += r.landWhy == RS_WAIT_END && r.macWaitLand;
    m->airTime += (r.evMask & EV_BIT(AUTO_EV_TAKEOFF_SLOW)) != 0u;   /* the pre-wait clock reached */
    m->noCam += r.landWhy == AUTO_RS_SINK;
    m->climbTimeout += r.landWhy == AUTO_RS_VZ && r.vzLand == VZ_R_CLIMB_TIMEOUT;
    m->kRange += r.landWhy == AUTO_RS_VZ && r.vzLand == VZ_R_K_RANGE;
    m->why[r.landWhy < NWHY ? r.landWhy : 0]++;
    m->imp[i] = r.impactV;
    m->peak[i] = r.hMax;
    m->hLand[i] = r.hAtLand;
    m->air[i] = r.airS;
    m->hoverAirMax = fmaxf(m->hoverAirMax, r.airAtHover);
    if (r.waited) {
      m->waitAir[m->nw++] = r.airAtWait;
    }
    if (r.landWhy == RS_WAIT_END && r.macWaitLand) {
      m->landAir[m->nt++] = r.landEntryAir;
    }
    if (!r.aborted) {
      m->landSMax = fmaxf(m->landSMax, r.landS);
    }
  }
}

static void hiPrint(const char *label, hiStats_t *m)
{
  printf("info: %s: n %d, hover wait reached %d, KILLED %d; LAND by:", label, m->n, m->waited, m->killed);
  printWhy(m->why);
  printf("info:   air clock at HOVER max %.2f s, at the wait p50 %.2f max %.2f s, at a timed LAND max %.2f s; LAND to "
         "touchdown max %.2f s; true airborne p50 %.2f max %.2f s\n", m->hoverAirMax,
         m->nw > 0 ? pct(m->waitAir, m->nw, 0.5f) : 0.0f, m->nw > 0 ? pct(m->waitAir, m->nw, 1.0f) : 0.0f,
         m->nt > 0 ? pct(m->landAir, m->nt, 1.0f) : 0.0f, m->landSMax, pct(m->air, m->n, 0.5f),
         pct(m->air, m->n, 1.0f));
  printf("info:   true h at LAND p50 %.2f m; true peak p50 %.2f p95 %.2f max %.2f m; impact p50 %.2f p95 %.2f max "
         "%.2f m/s\n", pct(m->hLand, m->n, 0.5f), pct(m->peak, m->n, 0.5f), pct(m->peak, m->n, 0.95f),
         pct(m->peak, m->n, 1.0f), pct(m->imp, m->n, 0.5f), pct(m->imp, m->n, 0.95f), pct(m->imp, m->n, 1.0f));
}

static void testHigh60(void)
{
  static hiStats_t m[4];
  const float airMaxS = (float)AUTO_AIR_MAX_DEFAULT_MS * 0.001f;
  static const char *const lab[4] = { "0.40 m, realistic, wait 10 s", "0.60 m, realistic, wait 10 s",
                                      "0.40 m, user range, wait 10 s", "0.60 m, user range, wait 10 s" };
  for (int k = 0; k < 4; k++) {
    hiRun(k < 2 ? &realRange : &userRange, (k & 1) ? 60 : 40, 10000, 1, 300, 0, true, HI_N, &m[k]);
    hiPrint(lab[k], &m[k]);
  }
  float imp[4][3];   /* p50, p95, max */
  for (int k = 0; k < 4; k++) {
    imp[k][0] = pct(m[k].imp, m[k].n, 0.5f);
    imp[k][1] = pct(m[k].imp, m[k].n, 0.95f);
    imp[k][2] = pct(m[k].imp, m[k].n, 1.0f);
    CHECK(m[k].killed == 0 && m[k].climbTimeout == 0 && m[k].airTime == 0,
          "%s: never KILLED (%d), never CLIMB_TIMEOUT (%d), never the pre-wait clock (TAKEOFF_SLOW %d)", lab[k],
          m[k].killed, m[k].climbTimeout, m[k].airTime);
  }
  /* Realistic: every sortie reaches the wait >= 1 s inside the pre-wait
   * limit and the Mac lands it at the wait's end (HOVER_TIMEOUT until 2026-10-15). */
  for (int k = 0; k < 2; k++) {
    CHECK(m[k].waited == m[k].n && m[k].timeout >= m[k].n * 98 / 100 && m[k].nw > 0 &&
          pct(m[k].waitAir, m[k].nw, 1.0f) <= airMaxS - 1.0f,
          "%s: hover wait reached (%d/%d) with the air clock <= %.1f s (max %.2f), the Mac's LAND at its end (%d)", lab[k],
          m[k].waited, m[k].n, airMaxS - 1.0f, m[k].nw > 0 ? pct(m[k].waitAir, m[k].nw, 1.0f) : 0.0f, m[k].timeout);
  }
  /* The slow LAND from 0.60 m (landFlatS 2.0 s) at contact against 0.40 m
   * on the same draws: realistic p50 / p95 / max within 0.10 / 0.15 m/s and
   * at most 0.90 m/s; the user range (its c +/-0.05 m/s moves the drone up
   * to ~0.6 m over the wait) at most 1.3 m/s. */
  CHECK(imp[1][0] <= imp[0][0] + 0.10f && imp[1][1] <= imp[0][1] + 0.15f && imp[1][2] <= 0.90f,
        "realistic, wait 10 s: impact at 0.60 m p50 %.2f p95 %.2f max %.2f m/s against 0.40 m %.2f %.2f %.2f",
        imp[1][0], imp[1][1], imp[1][2], imp[0][0], imp[0][1], imp[0][2]);
  CHECK(imp[3][2] <= 1.30f && pct(m[3].peak, m[3].n, 1.0f) <= 1.40f,
        "user range, wait 10 s, 0.60 m: impact max %.2f m/s (<= 1.30), true peak max %.2f m (<= 1.40)", imp[3][2],
        pct(m[3].peak, m[3].n, 1.0f));

  /* A 60 s wait (the interactive hover's longest until 2026-10-14) after the
   * longest take-off photo set under NVS autoair 8 s (the wait starts ~7.3 s
   * after the first RAMP step): the Mac's LAND 60 s later (no air clock since
   * 2026-10-15). */
  static hiStats_t h;
  hiRun(&fixRange, 60, 60000, AUTO_PHOTO_N_MAX, AUTO_PHOTO_GAP_MAX_MS, AUTO_AIR_MAX_HI_MS, true, 50,
        &h);
  hiPrint("0.60 m, c +/-0.010 m/s (since the gyro-bias fix), photos 3 x 1 s, autoair 8 s, wait 60 s", &h);
  const float hWait = h.nw > 0 ? pct(h.waitAir, h.nw, 1.0f) : 0.0f;
  /* A camera reading a climb that is not there (c > 0) sinks the drone into
   * ground effect over the minute, and the controller lands it K_RANGE (its
   * Khat walks to the HOLD floor, vert_ctrl.h VZ_KHAT_HOLD_LO_REL): a safe landing too. */
  CHECK(h.killed == 0 && h.waited == h.n && h.timeout + h.kRange == h.n &&
        pct(h.landAir, h.nt, 1.0f) <= hWait + 60.0f + 0.002f,
        "0.60 m, c +/-0.010 m/s, wait 60 s from air <= %.2f s: the Mac's LAND at its end (%d) or, sunk into ground "
        "effect, K_RANGE (%d) in every sortie (%d), at most %.2f s of air", hWait, h.timeout, h.kRange, h.n,
        h.nt > 0 ? pct(h.landAir, h.nt, 1.0f) : 0.0f);
  CHECK(pct(h.peak, h.n, 1.0f) <= 1.6f && pct(h.imp, h.n, 1.0f) <= 0.8f,
        "0.60 m, c +/-0.010 m/s, wait 60 s: true peak %.2f m (<= 1.6: the camera offset drifts the truth ~0.6 m in "
        "60 s), impact %.2f m/s (<= 0.8)", pct(h.peak, h.n, 1.0f), pct(h.imp, h.n, 1.0f));
  /* The study's older camera offset (c +/-0.03 m/s, before the gyro-bias
   * fix) over 60 s: info only. It drifts the truth up to ~1.8 m from the
   * estimate: the reason the operator watches the height (docs/autonomy-a2b.md
   * section 14). */
  static hiStats_t hr;
  hiRun(&realRange, 60, 60000, 1, 300, 0, true, 50, &hr);
  hiPrint("0.60 m, realistic (c +/-0.03 m/s, before the gyro-bias fix), wait 60 s (info)", &hr);
  CHECK(hr.killed == 0, "c +/-0.03 m/s, wait 60 s: never KILLED (%d)", hr.killed);

  /* The longest wait (AUTO_HOVER_WAIT_MAX_MS 120 s) under the same draws as the 60 s row: until 2026-10-15 the
   * 120 s air clock landed it (AIR_TIME); now nothing does but the Mac at the wait's end, unless the drone sank into
   * ground effect first (K_RANGE, c > 0 over two minutes); never KILLED. */
  static hiStats_t hl;
  hiRun(&fixRange, 105, AUTO_HOVER_WAIT_MAX_MS, 1, 300, 0, true, 50, &hl);
  hiPrint("1.05 m, c +/-0.010 m/s, wait 120 s (NVS autowait 120000)", &hl);
  const float hlWait = hl.nw > 0 ? pct(hl.waitAir, hl.nw, 1.0f) : 0.0f;
  CHECK(hl.killed == 0 && hl.waited == hl.n && hl.timeout + hl.kRange == hl.n &&
        (hl.nt == 0 || pct(hl.landAir, hl.nt, 1.0f) <= hlWait + 120.0f + 0.002f) && pct(hl.imp, hl.n, 1.0f) <= 0.8f,
        "1.05 m, wait 120 s: the Mac's LAND at its end (%d) or K_RANGE (%d) in every sortie (%d), at most %.2f s of "
        "air, never KILLED (%d), impact %.2f m/s (<= 0.8)", hl.timeout, hl.kRange, hl.n,
        hl.nt > 0 ? pct(hl.landAir, hl.nt, 1.0f) : 0.0f, hl.killed, pct(hl.imp, hl.n, 1.0f));

  /* No camera height estimate at 0.60 m (the observer never healthy): the
   * hover sinks (SINK: no camera, no camera-measured bound; HOVER_NO_CAM until 2026-10-15) 2 s in, never waits. The accelerometer alone
   * climbs longer to a higher target: measured true peak max 2.56 m, impact
   * 2.45 m/s over the user range's uniform draws (0.40 m on the same draws:
   * 1.95, 2.09). */
  static hiStats_t nc[2];
  for (int k = 0; k < 2; k++) {
    hiRun(&userRange, k == 0 ? 40 : 60, 10000, 1, 300, 0, false, HI_N, &nc[k]);
    hiPrint(k == 0 ? "0.40 m, user range, no camera" : "0.60 m, user range, no camera", &nc[k]);
  }
  CHECK(nc[1].killed == 0 && nc[1].waited == 0 && nc[1].noCam > 0 && pct(nc[1].peak, nc[1].n, 1.0f) <= 2.8f &&
        pct(nc[1].imp, nc[1].n, 1.0f) <= 2.7f,
        "0.60 m, no camera: never KILLED (%d), never waits (%d), SINK (%d), true peak %.2f m (<= 2.8), "
        "impact %.2f m/s (<= 2.7)", nc[1].killed, nc[1].waited, nc[1].noCam, pct(nc[1].peak, nc[1].n, 1.0f),
        pct(nc[1].imp, nc[1].n, 1.0f));
  /* The no-camera worst case is a corner (4.9), not a uniform draw: the
   * user range's 8 corners with runMc's CORNER_SEEDS seeds each, as above.
   * Without a camera c plays no part, so the corners with c +/-0.05 differ
   * only in their seeds. Measured: -20 mg, K -12 % reaches 2.83 / 2.96 m
   * true at 2.58 / 2.63 m/s at 0.60 m, every one HOVER_NO_CAM (0.40 m:
   * 2.18 / 2.29 m, 2.25 / 2.32 m/s; 0.30 m in testUserRange: 2.08 m,
   * 2.20 m/s). With the 4.5 s pre-wait clock of A2b the 0.60 m corner
   * reached 2.51 / 2.61 m: the 6 s clock lets its slowest hovers run on to
   * HOVER_NO_CAM. Checked against the measured maxima plus about 10 %. */
  float cPeak[2][8], cImp[2][8], cAir[2] = { 0.0f, 0.0f }, cLand[2] = { 0.0f, 0.0f };
  int cKilled[2] = { 0, 0 }, cWaited[2] = { 0, 0 }, cN[2] = { 0, 0 }, cNoCam[2] = { 0, 0 };
  for (int k = 0; k < 2; k++) {
    printf("info: %s m, user range, no camera, corners (8 x %d seeds; offset mg / K %%: true peak m, impact m/s):",
           k == 0 ? "0.40" : "0.60", CORNER_SEEDS);
    for (int j = 0; j < 8; j++) {
      cPeak[k][j] = cImp[k][j] = 0.0f;
      for (int i = 0; i < CORNER_SEEDS; i++) {
        scen_t s;
        scenDefault(&s);
        cornerOf(&userRange, j, &s);
        s.camC = 0.0f;
        s.camFirmware = false;
        s.camWakeS = -1.0f;
        s.heightCm = k == 0 ? 40 : 60;
        s.waitMs = 10000;
        s.photoN = 1;
        s.photoGapMs = 300;
        sortie_t r;
        runSortie(&s, 9000u + 100u * (uint32_t)j + (uint32_t)i, &r);
        cN[k]++;
        cKilled[k] += r.killed;
        cWaited[k] += r.waited;
        cNoCam[k] += r.landWhy == AUTO_RS_SINK;
        cPeak[k][j] = fmaxf(cPeak[k][j], r.hMax);
        cImp[k][j] = fmaxf(cImp[k][j], r.impactV);
        cAir[k] = fmaxf(cAir[k], r.landEntryAir);
        if (!r.aborted) {
          cLand[k] = fmaxf(cLand[k], r.landS);
        }
      }
      printf("%s %+.0f/%+.0f: %.2f, %.2f", j == 0 ? "" : ";", (j & 1) ? 20.0 : -20.0, (j & 2) ? 12.0 : -12.0,
             cPeak[k][j], cImp[k][j]);
    }
    printf("; KILLED %d, waited %d, SINK %d/%d, air clock at LAND max %.2f s, LAND to touchdown max %.2f s\n",
           cKilled[k], cWaited[k], cNoCam[k], cN[k], cAir[k], cLand[k]);
  }
  float cH = 0.0f, cI = 0.0f;
  for (int j = 0; j < 8; j++) {
    cH = fmaxf(cH, cPeak[1][j]);
    cI = fmaxf(cI, cImp[1][j]);
  }
  CHECK(cN[1] == 8 * CORNER_SEEDS && cKilled[1] == 0 && cWaited[1] == 0 && cNoCam[1] > 0 &&
        cAir[1] <= airMaxS + 0.001f && cLand[1] <= 6.0f && cH <= 3.25f && cI <= 2.9f,
        "0.60 m, no camera, corners (%d): never KILLED (%d), never waits (%d), SINK (%d), LAND by the "
        "%.1f s pre-wait clock (%.2f s), touchdown within 6 s (%.2f s); true peak %.2f m (<= 3.25), impact %.2f m/s "
        "(<= 2.9)", cN[1], cKilled[1], cWaited[1], cNoCam[1], airMaxS, cAir[1], cLand[1], cH, cI);

  /* The phone stick during the climb to 0.60 m (~1.7 s): LAND from CLIMB,
   * never a cut. Late in the climb the drone is 0.4-0.55 m up and climbing,
   * and the LAND from CLIMB (estimate-independent ceiling from 0.95 T_ref)
   * touches down harder than at 0.30 m (<= 0.65 m/s): measured up to 0.94. */
  static const float at[5] = { 0.35f, 0.70f, 1.00f, 1.30f, 1.60f };
  int ok = 0, n = 0;
  float worst = 0.0f;
  printf("info: phone stick in the climb to 0.60 m, impact max (20 seeds each):");
  for (int j = 0; j < 5; j++) {
    float wi = 0.0f;
    for (int i = 0; i < 20; i++) {
      scen_t s;
      scenDefault(&s);
      s.camFirmware = true;
      s.vRest = 4.05f;
      s.heightCm = 60;
      s.appAfterLiftS = at[j];
      sortie_t r;
      runSortie(&s, 7700u + 20u * (uint32_t)j + (uint32_t)i, &r);
      n++;
      ok += r.landWhy == AUTO_RS_APP_INPUT && !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN;
      wi = fmaxf(wi, r.impactV);
    }
    worst = fmaxf(worst, wi);
    printf(" %.2f s: %.2f m/s;", at[j], wi);
  }
  printf("\n");
  CHECK(ok == n && worst <= 1.05f,
        "phone stick in the climb to 0.60 m: LAND APP_INPUT, touchdown, never KILLED (%d/%d), impact max %.2f m/s "
        "(<= 1.05)", ok, n, worst);
}

/* The camera's wake on an autonomous take-off (docs/autonomy-a2.md section
 * 4.11). The firmware's take-off standby ends 0.5 s after the thrust first
 * reaches 40000 (or after 1.5 s warm, 3 s cold), which on the RAMP is ~2.4-2.8 s
 * after the first command and ~0.6 s after the lift-off (3 s on a cold take-off,
 * the first of a power-on), not the 1.5 s of the rows
 * above; the observer then needed 1.1-2.4 s of frames on the logged flights.
 * Until then the height estimate runs on the accelerometer, and the camera
 * cannot undo the height error built up by then (it measures velocity).
 * Offsets: the settled take-offs (+1.4 +/- 1.1 mg vs g0, so +3.7 after the
 * default bAir; +/-2 mg here), K +/-5 %, c 0. */
static void testCameraTiming(void)
{
  const int n = 200;
  float hm[2] = { 0.0f, 0.0f };
  int timeout[2] = { 0, 0 }, kRange[2] = { 0, 0 }, killed[2] = { 0, 0 };
  static const char *const name[2] = { "standby forced (resting 4.05 V)", "camera awake (resting 4.05 V)" };
  for (int k = 0; k < 2; k++) {
    float *h = malloc(sizeof(float) * (size_t)n);
    rngState = 5150u;
    for (int i = 0; i < n; i++) {
      scen_t s;
      scenDefault(&s);
      s.bias = 0.0037f + 0.0020f * (2.0f * urand() - 1.0f);
      s.kerr = 0.05f * (2.0f * urand() - 1.0f);
      s.camC = 0.0f;
      s.camWakeS = 2.5f;
      s.camConvS = 1.3f;
      s.camFirmware = k == 1;   /* row 0: the take-off standby of manual flights (2.5 s) forced; row 1: the
                                   state machine's camNoStandby, which A2b sets on every FLIGHT sortie */
      s.vRest = 4.05f;          /* both rows arm at the default vmin (3.80 V); A2a ran the standby row at 3.95 V */
      sortie_t r;
      runSortie(&s, 6200u + (uint32_t)i, &r);
      h[i] = r.hAtLand;
      timeout[k] += r.landWhy == RS_WAIT_END && r.macWaitLand;
      kRange[k] += r.vzLand == VZ_R_K_RANGE;
      killed[k] += r.killed;
    }
    hm[k] = pct(h, n, 0.5f);
    printf("info: camera timing, %s: true h at LAND entry p5 %.2f p50 %.2f m; the Mac's LAND at the wait's end %d/%d, K_RANGE %d, KILLED %d\n",
           name[k], pct(h, n, 0.05f), hm[k], timeout[k], n, kRange[k], killed[k]);
    free(h);
  }
  CHECK(killed[0] == 0 && killed[1] == 0, "camera timing: no sortie KILLED (%d, %d)", killed[0], killed[1]);
  CHECK(timeout[1] >= n * 95 / 100 && kRange[1] <= n / 50, "camera awake: the Mac's LAND at the wait's end (%d/%d), K_RANGE <= 2 %% (%d)",
        timeout[1], n, kRange[1]);
  CHECK(hm[1] >= 0.17f && hm[1] >= hm[0] + 0.04f, "camera awake holds the hover higher: median %.2f m (standby %.2f m)",
        hm[1], hm[0]);
}

/* Risk 2 of docs/autonomy-a2b.md: the time to the observer's first healthy
 * sample. Logged 2026-10-12 (manual take-offs, camera woken after the
 * take-off standby): first healthy 1.3-5.4 s after the wake, and never
 * unhealthy again in the air. Here the camera is awake from the first command
 * (camNoStandby) and the convergence is drawn from the logged 1.3-5.4 s,
 * counted from the lift-off (the observer needs the air): pessimistic, the
 * logged times include the standby. Realistic offsets, K +/-5 %, c +/-0.03. */
static void testCameraConvergence(void)
{
  static const uint16_t accel[2] = { AUTO_HOVER_ACCEL_DEFAULT_MS, AUTO_HOVER_ACCEL_MAX_MS };
  for (int a = 0; a < 2; a++) {
    const int n = 200;
    int waited = 0, noCam = 0, timeout = 0, killed = 0, nw = 0, resumed = 0;
    float convWait = 0.0f, convNoCam = 9.0f, w[200];
    rngState = 5300u;
    for (int i = 0; i < n; i++) {
      scen_t s;
      scenDefault(&s);
      s.bias = realRange.biasMid + realRange.biasSpan * (2.0f * urand() - 1.0f);
      s.kerr = realRange.kSpan * (2.0f * urand() - 1.0f);
      s.camC = realRange.cSpan * (2.0f * urand() - 1.0f);
      s.camFirmware = true;
      s.camConvS = 1.3f + 4.1f * urand();
      s.accelMs = accel[a];
      sortie_t r;
      runSortie(&s, 6400u + (uint32_t)i, &r);
      waited += r.waited;
      timeout += r.landWhy == RS_WAIT_END && r.macWaitLand;
      killed += r.killed;
      resumed += r.resumes > 0;
      if (r.sinks > 0) {   /* SINK (HOVER_NO_CAM until 2026-10-15: a LAND) */
        noCam++;
        convNoCam = fminf(convNoCam, s.camConvS);
        if (r.waited) {
          w[nw++] = r.waitS;
        }
      } else if (r.landWhy == RS_WAIT_END && r.macWaitLand) {
        convWait = fmaxf(convWait, s.camConvS);
      }
    }
    printf("info: observer convergence 1.3-5.4 s after lift-off, camera awake, autoaccel %u ms: hover wait reached %d/%d, the Mac's LAND at its end %d, SINK %d (resumed %d; convergence >= %.2f s; their wait p50 %.2f min %.2f s of 5), longest convergence that waited out 5 s with no SINK %.2f s, KILLED %d\n",
           (unsigned)accel[a], waited, n, timeout, noCam, resumed, convNoCam, nw > 0 ? pct(w, nw, 0.5f) : 0.0f,
           nw > 0 ? pct(w, nw, 0.0f) : 0.0f, convWait, killed);
    CHECK(killed == 0, "observer convergence 1.3-5.4 s, autoaccel %u ms: never KILLED (%d)", (unsigned)accel[a], killed);
  }
}

/* The phone during the wait: slow LAND in place, never a cut. */
/* The take-off's planned move (2026-10-14, docs/autonomy-a2b.md 21.4) through the real vertical controller on the
 * sim plant: TAKEOFF to 1.00 m with a plan of 1.50 m forward (D 4.0 m, the camera ideal), the wait 30 s, LAND 25 s
 * after the lift-off. The plan starts on the wait's first step (motion 1), at HOVER + STABLE 1 s + the photos; it
 * flies to DONE; the height holds through it; the sortie lands. Over 1 and 3 photos (300 ms apart), 10 seeds each. */
static void testPlanSortie(void)
{
  for (int k = 0; k < 2; k++) {
    int ok = 0, n = 0;
    float hoverMin = 99.0f, hoverMax = 0.0f, waitMin = 99.0f, waitMax = 0.0f, dh = 0.0f, dhEst = 0.0f, mv = 0.0f;
    for (int i = 0; i < 10; i++) {
      scen_t s;
      scenDefault(&s);
      s.heightCm = 100;
      s.photoN = k ? 3 : 1;
      s.photoGapMs = 300;
      s.waitMs = 30000;
      s.planF = 150;
      s.sceneD = 4.0f;
      s.landAfterLiftS = 25.0f;
      sortie_t r;
      runSortie(&s, 9100u + 13u * (uint32_t)i + 100u * (uint32_t)k, &r);
      n++;
      const bool good = r.waited && r.seqAtWait == 1 && r.tPlanRun >= r.tWait && r.tPlanRun - r.tWait <= 0.002f &&
                        r.planEnd == AUTO_MVE_DONE && r.landWhy == AUTO_RS_LAND_CMD && r.endWhy == AUTO_RS_TOUCHDOWN &&
                        !r.killed && r.planDh < 0.10f;
      ok += good;
      if (!good) {
        printf("  plan sortie seed %d: waited %d seq %d run %.3f wait %.3f end %s land %s end %s dh %.3f\n", i,
               r.waited, r.seqAtWait, r.tPlanRun, r.tWait, autoMvEndName(r.planEnd), autoReasonName(r.landWhy),
               autoReasonName(r.endWhy), r.planDh);
      }
      hoverMin = fminf(hoverMin, r.tHover - r.tTakeoff);
      hoverMax = fmaxf(hoverMax, r.tHover - r.tTakeoff);
      waitMin = fminf(waitMin, r.tWait - r.tHover);
      waitMax = fmaxf(waitMax, r.tWait - r.tHover);
      dh = fmaxf(dh, r.planDh);
      dhEst = fmaxf(dhEst, r.planDhEst);
      mv = fmaxf(mv, r.tPlanEnd - r.tPlanRun);
    }
    printf("info: planned 1.50 m forward from a 1.00 m take-off, %d photo(s): TAKEOFF -> HOVER %.2f-%.2f s, HOVER -> "
           "the wait and the plan %.2f-%.2f s (TAKEOFF + %.2f-%.2f s), the move to its end <= %.2f s; height "
           "excursion while it ran: true <= %.3f m, the estimate <= %.3f m\n", k ? 3 : 1, hoverMin, hoverMax,
           waitMin, waitMax, hoverMin + waitMin, hoverMax + waitMax, mv, dh, dhEst);
    CHECK(ok == n, "%d photo(s): the plan RUN on the wait's first step, DONE, then LAND and a touchdown, the height "
          "within 0.10 m through it (%d/%d)", k ? 3 : 1, ok, n);
  }
  /* refused at the wait's start (the scene 2.0 m ahead: forward 2.00 TOO_CLOSE): the drone waits, then lands */
  scen_t s;
  scenDefault(&s);
  s.heightCm = 100;
  s.photoN = 1;
  s.photoGapMs = 300;
  s.waitMs = 8000;
  s.planF = 200;
  s.sceneD = 2.0f;
  sortie_t r;
  runSortie(&s, 9500u, &r);
  CHECK(r.waited && r.seqAtWait == 0 && r.tPlanRun < 0.0f && r.landWhy == RS_WAIT_END && r.macWaitLand &&
        r.endWhy == AUTO_RS_TOUCHDOWN && !r.killed, "a plan refused (TOO_CLOSE): the wait as without one, the Mac's "
        "LAND 8 s later (%s)", autoReasonName(r.landWhy));
}

static void testPhoneInWait(void)
{
  int ok = 0, n = 0;
  float worstImpact = 0.0f, worstLand = 0.0f;
  for (int i = 0; i < 20; i++) {
    scen_t s;
    scenDefault(&s);
    s.appAtWaitS = 2.0f;
    sortie_t r;
    runSortie(&s, 7600u + (uint32_t)i, &r);
    if (!r.waited) {
      continue;
    }
    n++;
    ok += r.landWhy == AUTO_RS_APP_INPUT && !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN;
    worstImpact = fmaxf(worstImpact, r.impactV);
    worstLand = fmaxf(worstLand, r.landS);
  }
  CHECK(n >= 15 && ok == n, "phone stick 2 s into the wait: LAND APP_INPUT, touchdown, never KILLED (%d/%d)", ok, n);
  printf("info: phone stick during the wait: impact max %.2f m/s, LAND max %.2f s\n", worstImpact, worstLand);
}

int main(void)
{
  testLateLiftoff();
  testAirClock();
  testTripInAir();
  testPhoneInWait();
  testSlowLandFromClimb();
  testUserRange();
  testHigh60();
  testLearnChain();
  testCameraTiming();
  testCameraConvergence();
  testPlanSortie();
  CHECK(armRefused == 0, "every sortie armed: G3 at the default vmin passes at the drawn resting battery (%d refused)",
        armRefused);
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
