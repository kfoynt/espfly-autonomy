/*
 * ESP-Drone Firmware
 *
 * auto_fsm.c - Autonomy state machine (docs/autonomy-a2.md section 3)
 *
 * Every transition table of section 3.2 is one function below, with its rows
 * in the table's order: the first row that fires wins. Times are unsigned
 * differences of in->nowMs, so the wrapping ms clock is harmless.
 */
#include "auto_fsm.h"

/* Placement of the code that runs on every stabilizer loop while autonomy is
 * active (the A2b bench measured ~340 us a loop from flash: the 16 KB
 * instruction cache is shared and thrashed by the camera's flow fit, so the
 * loop fell to ~630-700 Hz). IRAM on the drone, nothing on the host. Besides
 * autoFsmStep(), the helpers GCC keeps out of line carry it too: the guards
 * (every ARMED step), groundExit() (ARMED, pre-spin), trip() (TAKEOFF, HOVER,
 * LAND), enter() and stopCmd(); updateStable() (HOVER) is inlined today and
 * carries it in case it is not. */
#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define AUTO_HOT IRAM_ATTR
#else
#define AUTO_HOT
#endif

#include <math.h>
#include <string.h>

#define AUTO_CAM_RUNNING     2          /* CAMERA_FLOW_RUNNING (camera_flow.h is not a pure header) */
#define AUTO_THRUST_FREE     65535.0f   /* thrustMax while autonomy leaves the setpoint alone */

typedef struct {
  autoFsm_t *f;
  const autoFsmIn_t *in;
  autoFsmOut_t *out;
  uint32_t now;
  bool kill;           /* Mac KILL this step (killCount changed or a KILL command) */
  bool appInput;       /* appInputCount changed this step */
  bool guardsSet;      /* out->failedGuards already holds a checked bitmap */
  bool stable;         /* the STABLE conditions hold this step */
  uint32_t stableMs;   /* age of the STABLE streak, 0 without one */
  bool stable1s;       /* STABLE 1 s: stable, and the streak is AUTO_STABLE_MS old */
  bool mvStepped;      /* autoMotionStep() filled out->mv this step */
} ctx_t;

/* ---- persistence timers ------------------------------------------------- */

static void track(autoHeld_t *h, bool cond, uint32_t now)
{
  if (!cond) {
    h->on = false;
  } else if (!h->on) {
    h->on = true;
    h->sinceMs = now;
  }
}

/* The condition has held on every step for at least ms. */
static bool heldFor(const autoHeld_t *h, uint32_t now, uint32_t ms)
{
  return h->on && now - h->sinceMs >= ms;
}

/* Same, counting only from refMs on (rest after a landing or a kill). */
static bool heldSince(const autoHeld_t *h, uint32_t now, uint32_t refMs, uint32_t ms)
{
  if (!h->on) {
    return false;
  }
  const uint32_t run = now - h->sinceMs;
  const uint32_t ref = now - refMs;
  return (run < ref ? run : ref) >= ms;
}

/* ---- input tests (a non-finite value fails every one of them) ----------- */

static bool imuFinite(const autoFsmIn_t *in)
{
  return isfinite(in->rollDeg) && isfinite(in->pitchDeg) && isfinite(in->tiltDeg) && isfinite(in->attErrDeg) &&
         isfinite(in->gyroDps[0]) && isfinite(in->gyroDps[1]) && isfinite(in->gyroDps[2]) &&
         isfinite(in->accNormG);
}

static bool battFinite(const autoFsmIn_t *in)
{
  return isfinite(in->vbat) && isfinite(in->vbatFastMin);
}

/* Section 3.2 row 2: what the trips and battery rules read, plus the
 * controller's height, speed and thrust once it runs. */
static AUTO_HOT bool inputsFinite(const autoFsmIn_t *in, bool withVz)
{
  return imuFinite(in) && battFinite(in) &&
         (!withVz || (isfinite(in->vz.h) && isfinite(in->vz.v) && isfinite(in->vz.thrust)));
}

static bool gyroBelow(const autoFsmIn_t *in, float dps)
{
  return fabsf(in->gyroDps[0]) < dps && fabsf(in->gyroDps[1]) < dps && fabsf(in->gyroDps[2]) < dps;
}

/* ARMED and pre-spin: stillness lost (tilt >= 8 deg, a gyro axis > 10 dps or
 * | |acc| - 1 g | > 0.10 g). Written as "not still" so NaN counts as moved. */
static bool moved(const autoFsmIn_t *in)
{
  const bool still = in->tiltDeg < AUTO_STILL_TILT_DEG && fabsf(in->gyroDps[0]) <= AUTO_MOVED_GYRO_DPS &&
                     fabsf(in->gyroDps[1]) <= AUTO_MOVED_GYRO_DPS && fabsf(in->gyroDps[2]) <= AUTO_MOVED_GYRO_DPS &&
                     fabsf(in->accNormG - 1.0f) <= AUTO_MOVED_ACC_G;
  return !still;
}

/* The stabilizer's soft-start rest test (stabilizer.c:646-649). */
static bool atRest(const autoFsmIn_t *in)
{
  return gyroBelow(in, AUTO_REST_GYRO_DPS) && fabsf(in->accNormG - 1.0f) < AUTO_REST_ACC_G;
}

/* G1. DRY is exempt from the reset-lock (lead decision) and from the
 * restart lock (integrator, A2b): it cannot spin a motor (thrustMax 0 at
 * both barriers of section 2), and after a 'reboot' the bench checks the
 * reset-lock's DRY exemption with the default plock on. */
static bool locked(const autoFsmIn_t *in, autoMode_t mode)
{
  const bool dry = mode == AUTO_MODE_DRY;
  return (in->restartLocked && !dry) || (in->resetLocked && !dry) || in->emergencyStop || !in->motorsReady ||
         !in->motorGuardOn || in->motorGuardTripped;
}

static bool killLatched(const autoFsm_t *f, const autoFsmIn_t *in)
{
  return in->killCount != f->lastKillCount ||   /* a KILL not yet stepped (STATUS between steps) */
         (f->killSeen && in->nowMs - f->killMs < AUTO_KILL_LATCH_MS);
}

/* NVS settings forced into their ranges: the glue range-checks them, and a
 * broken value must never exceed a maximum. */
static uint16_t clampU16(uint16_t x, uint16_t lo, uint16_t hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}

static uint16_t airMaxNvs(const autoFsmIn_t *in)
{
  return clampU16(in->airMaxNvsMs, AUTO_AIR_MAX_MIN_MS, AUTO_AIR_MAX_HI_MS);
}

/* Above 0.60 m: the extra climb time (the pre-wait air limit and the TAKEOFF
 * backstop grow by it; 0 up to 0.60 m). */
static uint32_t climbExtraMs(const autoFsm_t *f)
{
  return f->heightCm > 60u ? (uint32_t)(f->heightCm - 60u) * AUTO_AIR_PER_CM_ABOVE_60_MS : 0u;
}

/* LAND from a reference height (take-off height plus the vertical moves)
 * above 0.60 m: the extra descent at 0.30 m/s; after a move up, at least the
 * take-off height's extra plus the move at 0.30 m/s (vert_ctrl's flat part
 * grows by it, and its timeout keeps 4 s after the flat part); after a HOVER
 * longer than vert_ctrl's VZ_LAND_DRIFT_AFTER_S, its drift allowance too (as
 * its LAND timeout; the HOVER is entered one step after HOLD, so this is >=
 * it). An upper bound of vert_ctrl's landMaxS - VZ_LAND_MAX_S. Once per
 * LAND (at its entry): flash, out of line (internal RAM is short). */
static __attribute__((noinline)) uint32_t landExtraMs(const autoFsm_t *f)
{
  const float hCm = (float)f->heightCm + f->mv.hOff * 100.0f;
  const float upMs = (f->heightCm > 60u ? (float)(f->heightCm - 60u) : 0.0f) * 100.0f / 3.0f +
                     (f->mv.hOff > 0.0f ? f->mv.hOff * 100.0f * 100.0f / 3.0f : 0.0f);
  float ms = fmaxf(hCm > 60.0f ? (hCm - 60.0f) * 100.0f / 3.0f : 0.0f, upMs);
  const float hoverS = (float)f->hoverDurMs * 0.001f + 0.1f;
  if (hoverS > VZ_LAND_DRIFT_AFTER_S) {   /* at vert_ctrl's landVnear, 0.20 m/s */
    ms += fminf(VZ_LAND_DRIFT_MAX_M, VZ_LAND_DRIFT_MPS * hoverS) * 1000.0f / 0.20f;
  }
  return (uint32_t)(ms + 0.5f);
}

/* The motion module's inputs from this step's. */
static AUTO_HOT void mvIn(const ctx_t *c, autoMvIn_t *mi)
{
  const autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const bool vzOk = in->vz.phase != VZ_IDLE;
  const uint32_t air = f->rampSeen ? c->now - f->rampMs : 0;
  memset(mi, 0, sizeof(*mi));
  mi->nowMs = c->now;
  mi->flight = f->mode == AUTO_MODE_FLIGHT;
  mi->simPlant = in->simPlant;
  mi->sceneDM = in->sceneDM;
  mi->sceneTracked = in->sceneTracked;
  mi->sceneResets = in->sceneResets;
  mi->holdApplied = in->holdApplied;
  mi->holdCoasting = in->holdCoasting;
  mi->camFrameAgeMs = in->camFrameAgeMs;
  mi->camLagUs = in->camLagUs;
  mi->tiltDeg = in->tiltDeg;
  mi->camHeight = vzOk && in->vz.camHealthy;
  mi->vzH = vzOk ? in->vz.h : 0.0f;
  mi->hTargetM = (float)f->heightCm * 0.01f;
  mi->progLat = in->holdProgLat;
  mi->progFwd = in->holdProgFwd;
  mi->holdEpochSeen = in->holdEpochSeen;
  mi->holdMoving = in->holdMoving;
  mi->airMs = air;
  /* MOVE / YAW refused (BATT): the BATT_LOW condition held 0.25 s, or the hover already needs more than
   * AUTO_MV_REFUSE_T (a motion's extra thrust must stay under the 57000 trigger; auto_fsm.h) */
  mi->battLow = heldFor(&f->battLow, c->now, AUTO_VBAT_LOW_REFUSE_MS) || in->vz.tHold > AUTO_MV_REFUSE_T;
}

/* G2 in FLIGHT: the time on the ground after the latest flight (integer: AUTO_SOAK_PER_MOTOR is 1.5, and a
 * flight of SOAK_FULL_MS or more gives AUTO_SOAK_MAX_MS). */
#define SOAK_FULL_MS 46667u
_Static_assert(AUTO_SOAK_PER_MOTOR == 1.5f, "soakMs() computes 1.5 x as x + x / 2");
_Static_assert(AUTO_SETTLE_MIN_MS + SOAK_FULL_MS + SOAK_FULL_MS / 2u == AUTO_SOAK_MAX_MS, "the cap");
static AUTO_HOT uint32_t soakMs(uint32_t lastFlightMotorMs)
{
  const uint32_t m = lastFlightMotorMs < SOAK_FULL_MS ? lastFlightMotorMs : SOAK_FULL_MS;
  return AUTO_SETTLE_MIN_MS + m + m / 2u;
}

AUTO_HOT uint32_t autoFsmGuards(const autoFsm_t *f, const autoFsmIn_t *in, autoMode_t mode)
{
  uint32_t g = 0;
  const bool flight = mode == AUTO_MODE_FLIGHT;

  if (!(in->sessionValid && in->hbAgeMs < AUTO_HB_ARM_MAX_MS)) {
    g |= AUTO_G_LINK;
  }
  if (locked(in, mode)) {
    g |= AUTO_G_LOCKS;
  }
  /* FLIGHT also waits out the accelerometer's heat soak after a flight
   * (docs/autonomy-a2.md section 4.11): 20 s, longer after a longer flight
   * (AUTO_SOAK_*); DRY and BENCH do not fly. */
  const uint32_t groundMinMs = flight ? soakMs(in->lastFlightMotorMs) : AUTO_ONGROUND_MIN_MS;
  if (!(in->onGround && in->onGroundMs >= groundMinMs && imuFinite(in) &&
        fabsf(in->accNormG - 1.0f) < AUTO_STILL_ACC_G && gyroBelow(in, AUTO_STILL_GYRO_DPS) &&
        in->tiltDeg < AUTO_STILL_TILT_DEG)) {
    g |= AUTO_G_STILL;
  }
  /* G3. DRY is exempt from the resting minimum (A2b): it cannot spin a
   * motor, and a props-off DRY session must not need a lowered NVS vmin
   * (which would also let FLIGHT arm below it). A non-finite reading still
   * refuses: it is a sensor fault in every mode. */
  const bool vminOk = in->vmin >= AUTO_VMIN_LO - 0.001f && in->vmin <= AUTO_VMIN_HI + 0.001f;
  if (!(battFinite(in) && (mode == AUTO_MODE_DRY || (vminOk && in->vbat >= in->vmin)))) {
    g |= AUTO_G_BATT;
  }
  if (flight && !(in->camState == AUTO_CAM_RUNNING && in->holdEnabled && in->camFps > AUTO_FPS_MIN)) {
    g |= AUTO_G_CAMERA;
  }
  if (in->appRequired && !(in->appAgeMs < AUTO_APP_ARM_MAX_AGE_MS && in->appInputAgeMs >= AUTO_APP_CENTRED_MS)) {
    g |= AUTO_G_APP;
  }
  switch (mode) {
    case AUTO_MODE_FLIGHT:
      if (in->gcap != 0) {
        g |= AUTO_G_GCAP;
      }
      break;
    case AUTO_MODE_BENCH:
      if (in->gcap < 1 || in->gcap > AUTO_GCAP_BENCH_MAX) {
        g |= AUTO_G_GCAP;
      }
      break;
    case AUTO_MODE_DRY:
      break;
    default:
      g |= AUTO_G_GCAP;
      break;
  }
  if (!in->autoEnabled) {
    g |= AUTO_G_ENABLED;
  }
  if (!in->vzParamsOk) {
    g |= AUTO_G_VZPARAMS;
  }
  if (in->heapIntFree < AUTO_HEAP_MIN) {
    g |= AUTO_G_HEAP;
  }
  if (in->appRequired && in->gestureRequired && !(in->appGestureAgeMs < AUTO_GESTURE_WINDOW_MS)) {
    g |= AUTO_G_GESTURE;
  }
  if (killLatched(f, in)) {
    g |= AUTO_G_KILL;
  }
  if ((flight && in->simPlant) || (mode != AUTO_MODE_FLIGHT && mode != AUTO_MODE_DRY && mode != AUTO_MODE_BENCH)) {
    g |= AUTO_G_MODE;
  }
  return g;
}

/* ---- transitions -------------------------------------------------------- */

static void reply(ctx_t *c, autoResult_t r, uint32_t detail)
{
  c->out->cmdDone = true;
  c->out->cmdResult = r;
  c->out->cmdDetail = detail;
}

/* ---- "Claude decides" (user decisions 2026-10-14 ~20:30-20:40, docs/autonomy-a2b.md 22) --------------------
 * What replaced the give-up LANDs: an event with its evidence (autoFsmReport_t), the drone holding as well as its
 * sensors allow. Every function here runs on an event, or while an abnormal condition lasts (a hush, the camera
 * height out, the view lost, a SINK): flash, out of line (the per-loop tests that call them are IRAM). */

static void enter(ctx_t *c, autoState_t to, autoReason_t why);   /* (AUTO_HOT at its definition) */
static __attribute__((noinline)) void waitStart(ctx_t *c, const autoMvIn_t *mi);

/* The TAKEOFF's planned move not started (auto_fsm.h planPending; 2026-10-15 verification): an event before the hover
 * wait. Every one of them landed until "Claude decides", so the plan never flew after a contact, NOT_STABLE ...; it
 * must not now fly 1-4 m on its own seconds after one, before Claude has seen it. The drone hovers in the wait;
 * AUTO_STATUS mvWhy says why. */
static void planCancel(autoFsm_t *f)
{
  if (f->planPending) {
    f->planPending = 0;
    f->mv.why = AUTO_MVW_PLAN_CANCELLED;
  }
}

/* An event: the latest three kept (AUTO_STATUS evCode[3]), the count for the Mac; one before the wait cancels the
 * TAKEOFF's planned move. */
static __attribute__((noinline)) void pushEvent(autoFsm_t *f, uint32_t now, autoEvent_t e, uint32_t detail)
{
  autoFsmReport_t *r = &f->rep;
  for (int i = 2; i > 0; i--) {
    r->evCode[i] = r->evCode[i - 1];
    r->evDetail[i] = r->evDetail[i - 1];
    r->evMs[i] = r->evMs[i - 1];
  }
  r->evCode[0] = (uint8_t)e;
  r->evDetail[0] = (uint8_t)(detail > 255u ? 255u : detail);
  r->evMs[0] = now;
  r->evCount++;
}

static __attribute__((noinline)) void recordEvent(autoFsm_t *f, uint32_t now, autoEvent_t e, uint32_t detail)
{
  planCancel(f);
  pushEvent(f, now, e, detail);
}

/* The camera hold at its limit (auto_fsm.h autoFsmHoldLimit, docs/autonomy-a2b.md 27.4): information only, pushed
 * without recordEvent()'s plan cancel (it is reported in the wait only) and left out of the trim learner's count. */
void autoFsmHoldLimit(autoFsm_t *f, uint8_t bits, uint32_t nowMs)
{
  const uint8_t axes = AUTO_HL_ROLL | AUTO_HL_PITCH;
  const uint8_t rose = (uint8_t)(bits & (uint8_t)~f->holdLimSeen & axes);
  if (rose != 0u && f->state == AUTO_HOVER && f->mode == AUTO_MODE_FLIGHT && f->waiting &&
      !autoMotionActive(&f->mv)) {
    pushEvent(f, nowMs, AUTO_EV_HOLD_LIMIT, (uint32_t)(rose | (bits & (uint8_t)~axes)));
    f->infoEvents++;
    f->holdLimSeen = bits;
  } else {
    /* not reported now: only the falls are noted, so a rise kept from here is reported at the camera's next change
     * (the other axis, a sign) instead of being taken as seen (2026-10-16 review) */
    f->holdLimSeen = (uint8_t)(bits & f->holdLimSeen);
  }
}

/* HOVER again from a SINK (LAND -> HOVER RESUME): the wait, STABLE and the photos as they were. */
static __attribute__((noinline)) void hoverResume(ctx_t *c)
{
  autoFsm_t *f = c->f;
  f->camLostOn = false;
  f->camLostReported = false;
  f->camBackSince = 0u;
  f->rep.holdBits &= (uint8_t)~(AUTO_HB_CAM_LOST | AUTO_HB_SINKING);
  f->rep.sinkAtMs = 0u;
  f->resumeSince = 0u;
  recordEvent(f, c->now, AUTO_EV_RESUME, 0);
}

/* The heading offset now (the hold's yaw reference: a turn's live one while it turns). */
static float yawNowDeg(const autoMotion_t *m)
{
  return (m->kind == AUTO_MVK_YAW && autoMotionActive(m) && !m->turnDone) ? m->yawOffNow : m->yawOff;
}

/* An in-air contact (row 8b, auto_fsm.h AUTO_CONTACT_*): in the climb without the camera height the blind LAND
 * COLLISION (true); otherwise CONTACT: the hush, a running motion stopped (CONTACT: a snap and a brake; a YAW turns
 * back) or a snap epoch, the peaks from here, no LAND. Two hushes at most in any AUTO_CONTACT_CHAIN_MS. */
static __attribute__((noinline)) bool contactStart(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const uint32_t now = c->now;
  const vzPhase_t ph = in->vz.phase;
  if (f->rep.contacts < 255u) {
    f->rep.contacts++;
  }
  if (ph == VZ_CLIMB && !in->vz.camHealthy) {
    f->collided = true;   /* blocks the camera hold (the glue), until the next ARM; the descent is blind (out.blind) */
    enter(c, AUTO_LAND, AUTO_RS_COLLISION);
    return true;
  }
  uint32_t detail = (uint32_t)ph;
  if (f->hushPrevMs != 0u && now - f->hushPrevMs < AUTO_CONTACT_CHAIN_MS) {
    f->contactNextMs = f->hushPrevMs + AUTO_CONTACT_CHAIN_MS;   /* the chain bound: reported, no hush */
    recordEvent(f, now, AUTO_EV_CONTACT, detail | AUTO_CONTACT_NO_HUSH);
    return false;
  }
  f->hushPrevMs = f->hushMs != 0u ? f->hushMs | 1u : 0u;   /* the hush before this one (0: none this sortie) */
  f->hushMs = now;
  f->contactNextMs = now + AUTO_CONTACT_HUSH_MS;
  f->contactBlind = true;
  f->blindCamSince = 0u;
  autoFsmReport_t *r = &f->rep;
  r->holdBits |= AUTO_HB_HUSH;
  r->peakAccG = in->accNormG;
  r->peakRateDps = fmaxf(fabsf(in->gyroDps[0]), fabsf(in->gyroDps[1]));
  r->contactAxG = in->accXG;
  r->contactAyG = in->accYG;
  r->contactYawDeg = yawNowDeg(&f->mv);
  bool undone = false;
  f->hushSnap = 0;
  if (f->state == AUTO_HOVER) {
    autoMvIn_t mi;
    mvIn(c, &mi);
    f->hushSnap = !autoMotionActive(&f->mv);
    if (autoMotionActive(&f->mv)) {
      const bool undo0 = f->mv.undo;
      autoMotionStop(&f->mv, &mi, AUTO_MVE_CONTACT);
      detail |= AUTO_CONTACT_MOTION;
      undone = !undo0 && f->mv.undo;
    } else {
      autoMotionTrackNow(&f->mv, &mi);   /* mvfix DRIFT: dTrack at the snap (the new epoch restarts the progress) */
      autoMotionSnap(&f->mv);   /* the hold re-anchors here: it must not pull back toward what was hit */
    }
  }
  recordEvent(f, now, AUTO_EV_CONTACT, detail);
  if (undone) {
    recordEvent(f, now, AUTO_EV_RETRACK_UNDO, AUTO_MVE_CONTACT);
  }
  return false;
}

/* While a hush runs: vzInput_t.hush, the contact's peaks; its end clears AUTO_HB_HUSH. A contact in HOVER with no
 * motion running re-anchors the hold at the hush's end, where the bounce left the drone (the contact's snap anchored it
 * at the contact point, touching what was hit: the hold walked it back there; 2026-10-15 verification), unless a motion
 * started since. */
/* The hush's end re-snap (flash, once per contact): dTrack first follows the bounce the contact's snap epoch measured
 * (mvfix DRIFT; the review of 2026-10-17: the new epoch restarts the progress, so without it the bounce was never in
 * the scene distance), then the snap. */
static __attribute__((noinline)) void hushSnapEnd(ctx_t *c)
{
  autoMvIn_t mi;
  mvIn(c, &mi);
  autoMotionTrackNow(&c->f->mv, &mi);
  autoMotionSnap(&c->f->mv);
}

static void hushStep(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  if (c->now - f->hushMs >= AUTO_CONTACT_HUSH_MS) {
    f->rep.holdBits &= (uint8_t)~AUTO_HB_HUSH;
    if (f->hushSnap && f->state == AUTO_HOVER && !autoMotionActive(&f->mv)) {
      hushSnapEnd(c);
    }
    f->hushSnap = 0;
    return;
  }
  c->out->hush = true;
  autoFsmReport_t *r = &f->rep;
  if (in->accNormG > r->peakAccG) {
    r->peakAccG = in->accNormG;
    r->contactAxG = in->accXG;
    r->contactAyG = in->accYG;
  }
  r->peakRateDps = fmaxf(r->peakRateDps, fmaxf(fabsf(in->gyroDps[0]), fabsf(in->gyroDps[1])));
}

/* After a contact: its window (a LAND then blocks the camera hold: collided) until the camera height has been in use
 * AUTO_CAM_BACK_MS after its hush (the estimate re-seeded from it). Every LAND of the sortie is blind anyway (out.blind,
 * user decision (B) 2026-10-15). */
static void blindStep(ctx_t *c)
{
  autoFsm_t *f = c->f;
  if ((f->rep.holdBits & AUTO_HB_HUSH) == 0u && c->in->vz.phase != VZ_IDLE && c->in->vz.camHealthy) {
    if (f->blindCamSince == 0u) {
      f->blindCamSince = c->now | 1u;
    } else if (c->now - f->blindCamSince >= AUTO_CAM_BACK_MS) {
      f->contactBlind = false;
    }
  } else {
    f->blindCamSince = 0u;
  }
}

/* While contactBlind (every contact sets it; a hush runs inside it): the hush's steps, then the blind period's. */
static __attribute__((noinline)) void contactStep(ctx_t *c)
{
  if (c->f->rep.holdBits & AUTO_HB_HUSH) {
    hushStep(c);
  }
  blindStep(c);
}

/* A loop gap over AUTO_LOOP_GAP_LAND_US: the event, a running MOVE stopped (a YAW's turn ends where it is); the
 * AUTO_LOOP_GAP_N-th within AUTO_LOOP_GAP_WIN_MS: true (LAND LOOP_GAP: control cannot be held). */
static __attribute__((noinline)) bool gapEvent(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const uint32_t now = c->now;
  const uint32_t ms = c->in->loopGapUs / 1000u;
  if (f->rep.loopGaps < 255u) {
    f->rep.loopGaps++;
  }
  recordEvent(f, now, AUTO_EV_LOOP_GAP, ms);
  _Static_assert(AUTO_LOOP_GAP_N == 3, "gapMs keeps the two gaps before this one");
  if (f->gapN >= 2u && now - f->gapMs[1] < AUTO_LOOP_GAP_WIN_MS) {
    return true;
  }
  f->gapMs[1] = f->gapMs[0];
  f->gapMs[0] = now;
  if (f->gapN < 2u) {
    f->gapN++;
  }
  if (f->state == AUTO_HOVER && autoMotionActive(&f->mv)) {
    autoMvIn_t mi;
    mvIn(c, &mi);
    autoMotionStop(&f->mv, &mi, AUTO_MVE_LOOP_GAP);
    if (!autoMotionActive(&f->mv)) {
      recordEvent(f, now, AUTO_EV_MOVE_STOPPED, AUTO_MVE_LOOP_GAP);
    }
  }
  return false;
}

/* A motion guard's stop request (autoMotionStep(): the view lost 2 s, a tilt): the motion stopped (a YAW turns back,
 * or its turn ends where it is), reported; this step's motion outputs from autoMotionIdleOut. UNDO and FAST (the step's
 * own turn back, its own fast stop) are reports only: this step's outputs are the step's. */
static __attribute__((noinline)) void mvStopReq(ctx_t *c, const autoMvIn_t *mi, autoMvLand_t ml)
{
  autoFsm_t *f = c->f;
  if (ml == AUTO_MVL_UNDO) {
    recordEvent(f, c->now, AUTO_EV_RETRACK_UNDO, AUTO_MVE_RETRACK);   /* its re-track found no G: turning back */
    return;
  }
  if (ml == AUTO_MVL_FAST) {
    /* the step stopped a MOVE that lost its view, with its brake (auto_motion.h): VIEW_FAST (mvspeed > 100), or FRONT
     * (a forward MOVE near the scene, mvnear; 2026-10-17): its end */
    recordEvent(f, c->now, AUTO_EV_MOVE_STOPPED, f->mv.end);
    return;
  }
  const autoMvEnd_t why = ml == AUTO_MVL_TILT ? AUTO_MVE_TILT : AUTO_MVE_NO_VIEW;
  const bool undo0 = f->mv.undo;
  autoMotionStop(&f->mv, mi, why);
  if (!undo0 && f->mv.undo) {
    recordEvent(f, c->now, AUTO_EV_RETRACK_UNDO, why);
  } else {
    recordEvent(f, c->now, AUTO_EV_MOVE_STOPPED, why);
  }
  c->mvStepped = false;
}

/* The camera rule (HOVER, FLIGHT, auto_fsm.h AUTO_CAM_*), while the camera height is out or coming back: CAM_LOST,
 * CAM_BACK, the SINK (true: LAND SINK entered). */
static __attribute__((noinline)) bool camRule(ctx_t *c, bool camIn)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const uint32_t now = c->now;
  autoFsmReport_t *r = &f->rep;
  if (!(now - f->holdMs >= f->hoverAccelMs || f->contactBlind || f->faultHover)) {
    return false;   /* from hoverAccelMs into HOVER on, at once after a contact or a climb fault */
  }
  if (camIn) {
    if (f->camBackSince == 0u) {
      f->camBackSince = now | 1u;
    } else if (now - f->camBackSince >= AUTO_CAM_BACK_MS) {
      f->camLostOn = false;
      r->sinkAtMs = 0u;
      if (f->camLostReported) {
        f->camLostReported = false;
        r->holdBits &= (uint8_t)~AUTO_HB_CAM_LOST;
        recordEvent(f, now, AUTO_EV_CAM_BACK, 0);
      }
    }
    return false;
  }
  f->camBackSince = 0u;
  if (!f->camLostOn && (in->vz.camOpen || now - f->camOpenMs <= 1u)) {
    /* the controller's open window after a contact, the observer healthy: its own, not a loss (and the window's last
     * step, which the controller ends after it read the camera) */
    if (in->vz.camOpen) {
      f->camOpenMs = now;
    }
    return false;
  }
  if (!f->camLostOn) {
    f->camLostOn = true;
    f->camLostReported = false;
    /* the budget's start: the later of the HOLD entry and the last step with the camera estimate in use (the
     * controller's noCamS; it counts in CLIMB too and starts again at HOLD) */
    const float ns = in->vz.noCamS;
    r->camOutMs = now - (ns > 0.0f && ns < 600.0f ? (uint32_t)(ns * 1000.0f + 0.5f) : 0u);
    r->camBits = 0;
  }
  r->camBits |= (uint8_t)(in->camBits | (in->camState != AUTO_CAM_RUNNING ? AUTO_CB_NOT_RUNNING : 0u));
  const bool bound = in->vz.capQ > 0.0f;   /* the controller's open loop under the measured hover: it cannot climb */
  r->sinkAtMs = bound ? r->camOutMs + AUTO_CAM_LOST_MAX_MS : now;
  if (!bound || now - r->camOutMs >= AUTO_CAM_LOST_MAX_MS) {
    recordEvent(f, now, AUTO_EV_SINK, bound ? 0u : ((f->contactBlind || f->faultHover) ? 2u : 1u));
    enter(c, AUTO_LAND, AUTO_RS_SINK);
    return true;
  }
  if (!f->camLostReported && now - r->camOutMs >= AUTO_CAM_LOST_REPORT_MS) {
    f->camLostReported = true;
    r->holdBits |= AUTO_HB_CAM_LOST;
    recordEvent(f, now, AUTO_EV_CAM_LOST, r->camBits);
  }
  return false;
}

/* HOVER without a motion (FLIGHT): the hold not applied on a fresh frame AUTO_VIEW_LOST_REPORT_MS (the drone drifts):
 * HOLD_NO_VIEW; applied again: VIEW_BACK. */
static __attribute__((noinline)) void viewRule(ctx_t *c, bool viewBad)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  if (viewBad) {
    if (!f->viewBadOn) {
      f->viewBadOn = true;
      f->viewBadSince = c->now;
    } else if (!f->viewReported && c->now - f->viewBadSince >= AUTO_VIEW_LOST_REPORT_MS) {
      f->viewReported = true;
      f->rep.holdBits |= AUTO_HB_NO_VIEW;
      recordEvent(f, c->now, AUTO_EV_HOLD_NO_VIEW, in->holdCoasting ? 1u : (!in->holdApplied ? 2u : 3u));
    }
    return;
  }
  f->viewBadOn = false;
  if (f->viewReported) {
    f->viewReported = false;
    f->rep.holdBits &= (uint8_t)~AUTO_HB_NO_VIEW;
    recordEvent(f, c->now, AUTO_EV_VIEW_BACK, 0);
  }
}

/* NOT_STABLE (row 24): no STABLE 1 s by HOVER + 2 s once the camera had a 1.5 s view: the event with the conditions
 * that failed, AUTO_HB_NOT_STABLE until a STABLE 1 s, the wait starts (a LAND until 2026-10-15). */
static __attribute__((noinline)) void notStable(ctx_t *c, const autoMvIn_t *mi)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  uint32_t d = 0;
  d |= (fabsf(in->rollDeg) > AUTO_STABLE_TILT_DEG || fabsf(in->pitchDeg) > AUTO_STABLE_TILT_DEG) ? AUTO_NS_TILT : 0u;
  d |= (fabsf(in->gyroDps[0]) > AUTO_STABLE_GXY_DPS || fabsf(in->gyroDps[1]) > AUTO_STABLE_GXY_DPS ||
        fabsf(in->gyroDps[2]) > AUTO_STABLE_GZ_DPS) ? AUTO_NS_GYRO : 0u;
  d |= (f->mode == AUTO_MODE_FLIGHT && !(in->camState == AUTO_CAM_RUNNING && in->camFrameAgeMs <= AUTO_STABLE_FRAME_AGE_MS &&
                                          in->holdApplied && !in->holdCoasting)) ? AUTO_NS_HOLD : 0u;
  d |= !in->vz.holdSteady ? AUTO_NS_VZ : 0u;
  f->notStableSeen = true;
  f->rep.holdBits |= AUTO_HB_NOT_STABLE;
  recordEvent(f, c->now, AUTO_EV_NOT_STABLE, d);
  if (!f->waiting) {
    waitStart(c, mi);
  }
}

/* TAKEOFF_SLOW: the air clock at airMaxMs (+ the climb's extra) before the wait: the wait starts anyway (the air clock
 * landed until 2026-10-15). */
static __attribute__((noinline)) void takeoffSlow(ctx_t *c, const autoMvIn_t *mi)
{
  recordEvent(c->f, c->now, AUTO_EV_TAKEOFF_SLOW, 0);
  waitStart(c, mi);
}

/* A photo set that failed (or timed out): the event, the set consumed (a PHOTO is not BUSY), the wait starts if it
 * had not (LAND CAPTURE_FAIL until 2026-10-15). */
static __attribute__((noinline)) void captureFail(ctx_t *c, const autoMvIn_t *mi)
{
  autoFsm_t *f = c->f;
  f->capFailSeen = true;
  recordEvent(f, c->now, AUTO_EV_CAPTURE_FAIL, 0);
  if (!f->waiting) {
    waitStart(c, mi);
  }
}

/* The controller's event (vzOutput_t evtN changed): forwarded with its reason. */
static __attribute__((noinline)) void vzEvtFwd(autoFsm_t *f, const autoFsmIn_t *in)
{
  f->vzEvtSeen = in->vz.evtN;
  recordEvent(f, in->nowMs, AUTO_EV_VZ, in->vz.evt);
}

/* LAND SINK, every step until it ends: a LAND that Claude, the user or a minimum rule asks for takes it over (its
 * reason; never resumed: a LAND or DISARM command, HB loss, a session change, the phone, the battery rules); else the
 * camera height in use AUTO_CAM_BACK_MS asks the controller to resume (vzInput_t.resume, VZ_CMD_TAKEOFF), and its
 * HOLD is HOVER RESUME, still waiting. */
static __attribute__((noinline)) bool sinkStep(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const uint32_t now = c->now;
  const autoCmdType_t t = in->cmd.type;
  autoReason_t why = AUTO_RS_NONE;
  if (t == AUTO_CMD_LAND || t == AUTO_CMD_DISARM) {
    reply(c, AUTO_R_OK, (uint32_t)f->state);
    why = t == AUTO_CMD_LAND ? AUTO_RS_LAND_CMD : AUTO_RS_DISARM_CMD;
  } else if (c->appInput) {
    why = AUTO_RS_APP_INPUT;
  } else if (in->hbAgeMs > AUTO_HB_LAND_MS) {
    why = AUTO_RS_HB_LOST;
  } else if (in->appRequired && in->appAgeMs > AUTO_APP_SILENT_MS) {
    why = AUTO_RS_APP_SILENT;
  } else if (in->sessionChanged) {
    why = AUTO_RS_SESSION;
  } else if (heldFor(&f->vbatLow, now, AUTO_VBAT_LAND_MS)) {
    why = AUTO_RS_VBAT_LOW;
  } else if (heldFor(&f->battLow, now, AUTO_VBAT_LOW_LAND_MS)) {
    why = AUTO_RS_BATT_LOW;
  } else if (in->vbatFastMin > 0.0f && in->vbatFastMin < AUTO_VBAT_FAST_LAND_V) {
    why = AUTO_RS_VBAT_DIP;
  }
  if (why != AUTO_RS_NONE) {
    f->reason = why;   /* the same LAND goes on; never resumed */
    f->rep.holdBits &= (uint8_t)~AUTO_HB_SINKING;
    f->resumeSince = 0u;
    return false;
  }
  const bool vzOk = in->vz.phase != VZ_IDLE;
  if (vzOk && in->vz.phase == VZ_HOLD) {
    enter(c, AUTO_HOVER, AUTO_RS_RESUME);   /* the controller resumed on the previous step */
    return true;
  }
  if (vzOk && in->vz.camHealthy) {
    if (f->resumeSince == 0u) {
      f->resumeSince = now | 1u;
    }
  } else {
    f->resumeSince = 0u;
  }
  /* a SINK begun in a contact's window (collided) never resumes (its hold blocked, its anchor where it hit); one after an
   * older contact descends blind (out.blind) and resumes as any (user decision (B) 2026-10-15: the hold keeps the camera) */
  c->out->resume = !f->collided && f->resumeSince != 0u && now - f->resumeSince >= AUTO_CAM_BACK_MS;
  return false;
}

/* HOVER entry: the camera rule's clock; a RESUME keeps the wait, STABLE and the photos as they were; else they start
 * again, and a HOLD a climb fault or a contact started has no measured hover (the camera rule may SINK at once).
 * Flash: once a transition. */
static __attribute__((noinline)) void hoverEnter(ctx_t *c, autoReason_t why)
{
  autoFsm_t *f = c->f;
  if (why == AUTO_RS_RESUME) {
    hoverResume(c);
    return;
  }
  f->stable.on = false;
  f->stableOnce = false;
  f->captureStarted = false;
  f->waiting = false;
  f->faultHover = c->in->vz.reason != VZ_R_HOLD_REACHED;
  if (f->faultHover) {
    planCancel(f);   /* a climb fault or a contact ended the climb (its event comes on this step too) */
  }
}

/* LAND entry: the camera hold blocked (collided) when it begins during a contact's hush or before the camera height was
 * back after it; SINKING. (Blind after any contact of the sortie: out.blind.) Flash: once a transition. */
static __attribute__((noinline)) void landEnterRules(autoFsm_t *f, autoReason_t why)
{
  f->collided = f->collided || f->contactBlind;
  if (why == AUTO_RS_SINK) {
    f->rep.holdBits |= AUTO_HB_SINKING;
  }
}

/* ARM: this sortie's report and the hold-and-report rules' state (flash: on the ground, once a sortie). */
static __attribute__((noinline)) void armReset(autoFsm_t *f, uint32_t now)
{
  memset(&f->rep, 0, sizeof(f->rep));
  f->hushMs = f->hushPrevMs = f->blindCamSince = f->resumeSince = 0u;
  f->hushSnap = 0;
  f->contactNextMs = now;
  f->gapN = 0;
  f->vzEvtSeen = 0;
  f->camLostOn = f->camLostReported = f->contactBlind = f->viewBadOn = f->viewReported = false;
  f->capFailSeen = f->faultHover = f->notStableSeen = false;
  f->setHeightMs = 0u;   /* SET_HEIGHT's rate bound: none this sortie */
  f->infoEvents = 0u;    /* (rep.evCount is 0 too) */
  f->holdLimSeen = 0u;   /* an axis already at its limit reports again in this sortie */
}

static AUTO_HOT void enter(ctx_t *c, autoState_t to, autoReason_t why)
{
  autoFsm_t *f = c->f;
  autoFsmOut_t *out = c->out;
  const autoState_t from = f->state;

  if (from == AUTO_HOVER && to != AUTO_HOVER) {
    f->hoverDurMs = c->now - f->holdMs;   /* the LAND backstop's drift allowance */
  }
  if (from == AUTO_HOVER && to != AUTO_HOVER && autoMotionActive(&f->mv)) {
    autoMvIn_t mi;
    mvIn(c, &mi);
    autoMotionStop(&f->mv, &mi, AUTO_MVE_LEFT_HOVER);   /* the hold holds where it is (snap) */
  }
  f->state = to;
  f->reason = why;
  f->stateSinceMs = c->now;
  switch (to) {
    case AUTO_IDLE:
      /* Leaving an active state: the app's throttle needs a fresh zero, and
       * a stale app setpoint still in the commander must not reach the
       * motors (release hold). */
      out->lockAppThrust = true;
      f->releaseHold = true;
      f->idleSinceMs = c->now;
      if (from == AUTO_KILLED && f->weSetEstop) {
        out->clearEmergencyStop = true;
      }
      f->weSetEstop = false;
      break;
    case AUTO_ARMED:
      f->armMs = c->now;
      f->releaseHold = false;
      f->prespinDone = false;
      f->rampSeen = false;
      f->captureStarted = false;
      f->waiting = false;
      f->landEnded = false;
      f->landCut = false;
      f->contactSeen = false;
      f->stableOnce = false;
      f->stable.on = false;
      f->heightCm = 0;
      f->photoN = 0;
      f->photoGapMs = 0;
      f->vzLandReason = VZ_R_NONE;
      f->landEndReason = AUTO_RS_NONE;
      f->landTrip = AUTO_RS_NONE;
      autoMotionInit(&f->mv);
      f->hoverDurMs = 0;
      f->collided = false;
      armReset(f, c->now);   /* "Claude decides": this sortie's report and rules (flash) */
      break;
    case AUTO_TAKEOFF:
      out->yawCapture = true;
      f->prespinDone = false;
      break;
    case AUTO_HOVER:
      f->holdMs = c->now;
      hoverEnter(c, why);   /* (flash) */
      break;
    case AUTO_LAND:
      landEnterRules(f, why);   /* (flash) */
      f->landEnded = false;
      f->landCut = false;
      f->contactSeen = false;
      f->landTrip = AUTO_RS_NONE;
      f->landBackstopMs = AUTO_LAND_BACKSTOP_MS + landExtraMs(f);   /* the height and moves are final now */
      break;
    case AUTO_KILLED:
      out->setEmergencyStop = true;
      if (!c->in->emergencyStop) {
        f->weSetEstop = true;   /* only then is it ours to clear */
      }
      break;
  }
}

/* LAND or DISARM command: never refused. */
static AUTO_HOT bool stopCmd(ctx_t *c, autoState_t to)
{
  const autoCmdType_t t = c->in->cmd.type;
  if (t != AUTO_CMD_LAND && t != AUTO_CMD_DISARM) {
    return false;
  }
  reply(c, AUTO_R_OK, (uint32_t)c->f->state);
  enter(c, to, t == AUTO_CMD_LAND ? AUTO_RS_LAND_CMD : AUTO_RS_DISARM_CMD);
  return true;
}

/* ARMED and pre-spin, motors off: the rows that go back to IDLE. */
static AUTO_HOT autoReason_t groundExit(ctx_t *c)
{
  const autoFsmIn_t *in = c->in;
  if (c->appInput) {
    return AUTO_RS_APP_INPUT;
  }
  if (in->hbAgeMs > AUTO_HB_LAND_MS) {
    return AUTO_RS_HB_LOST;
  }
  if (in->sessionChanged) {
    return AUTO_RS_SESSION;
  }
  if (in->appRequired && in->appAgeMs > AUTO_APP_SILENT_MS) {
    return AUTO_RS_APP_SILENT;
  }
  if (heldFor(&c->f->moved, c->now, AUTO_MOVED_MS)) {
    return AUTO_RS_MOVED;
  }
  return AUTO_RS_NONE;
}

/* Tilt, rate, free-fall and tracking trips (section 3.2 rows 3-6). */
static AUTO_HOT autoReason_t trip(const ctx_t *c)
{
  const autoFsm_t *f = c->f;
  if (heldFor(&f->tilt, c->now, AUTO_TILT_KILL_MS)) {
    return AUTO_RS_TILT;
  }
  if (heldFor(&f->rate, c->now, AUTO_RATE_KILL_MS)) {
    return AUTO_RS_RATE;
  }
  if (heldFor(&f->fall, c->now, AUTO_FALL_KILL_MS)) {
    return AUTO_RS_FREEFALL;
  }
  if (heldFor(&f->attErr, c->now, AUTO_ATTERR_KILL_MS)) {
    return AUTO_RS_ATT_ERR;
  }
  return AUTO_RS_NONE;
}

/* Motor watchdog, motor driver and emergency stop (row 7). In an active
 * state other than KILLED the FSM never set the emergency stop itself. */
static autoReason_t motorFault(const autoFsmIn_t *in)
{
  if (in->motorGuardTripped) {
    return AUTO_RS_MOTOR_GUARD;
  }
  if (!in->motorsReady) {
    return AUTO_RS_MOTOR_STOP;
  }
  if (in->emergencyStop) {
    return AUTO_RS_ESTOP_EXT;
  }
  return AUTO_RS_NONE;
}

/* ARM (IDLE) and TAKEOFF (ARMED), motors off: once per command, so flash,
 * out of line (internal RAM is short: these two pay for the LAND COLLISION's
 * IRAM, docs/autonomy-a2b.md; motionCmd() and landExtraMs() are flash for the
 * same reason). The guards they call are IRAM. */
static __attribute__((noinline)) void armCmd(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const autoCmd_t *cmd = &in->cmd;
  const uint16_t nvsAir = airMaxNvs(in);

  if (f->killSeen && c->now - f->killMs < AUTO_KILL_LATCH_MS) {
    reply(c, AUTO_R_KILL_LATCHED, (uint32_t)f->state);
    return;
  }
  if (cmd->mode > AUTO_MODE_BENCH ||
      (cmd->airMaxMs != 0 && (cmd->airMaxMs < AUTO_AIR_MAX_MIN_MS || cmd->airMaxMs > nvsAir))) {   /* nvsAir <= HI */
    reply(c, AUTO_R_RANGE, (uint32_t)f->state);
    return;
  }
  const autoMode_t mode = (autoMode_t)cmd->mode;
  const uint32_t g = autoFsmGuards(f, in, mode);
  c->out->failedGuards = g;
  c->guardsSet = true;
  if (g != 0) {
    reply(c, AUTO_R_GUARDS, g);
    return;
  }
  f->mode = mode;
  f->airMaxMs = cmd->airMaxMs != 0 ? cmd->airMaxMs : nvsAir;
  f->hoverWaitMs = in->hoverWaitNvsMs < AUTO_HOVER_WAIT_MAX_MS ? in->hoverWaitNvsMs : AUTO_HOVER_WAIT_MAX_MS;
  f->hoverAccelMs = clampU16(in->hoverAccelNvsMs, 0, AUTO_HOVER_ACCEL_MAX_MS);
  f->camAwake = mode == AUTO_MODE_FLIGHT || mode == AUTO_MODE_BENCH;
  f->benchCap = 0.0f;
  if (mode == AUTO_MODE_BENCH) {
    f->benchCap = (float)(in->gcap < AUTO_BENCH_THRUST_MAX ? in->gcap : AUTO_BENCH_THRUST_MAX);
  }
  f->armToken = in->randToken != 0 ? in->randToken : 1u;   /* the glue promises nonzero; 0 must never match */
  f->planPending = 0;   /* (a TAKEOFF latches it anyway) */
  enter(c, AUTO_ARMED, AUTO_RS_ARM);
  reply(c, AUTO_R_OK, f->armToken);
}

static __attribute__((noinline)) void takeoffCmd(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const autoCmd_t *cmd = &c->in->cmd;

  if (cmd->token != f->armToken) {
    reply(c, AUTO_R_BAD_STATE, 0);
    return;
  }
  /* the planned move (auto_fsm.h): a MOVE's limits (no wait needed since 2026-10-15: the wait always starts) */
  const bool plan = cmd->moveRightCm != 0 || cmd->moveFwdCm != 0;
  if (cmd->heightCm < AUTO_HEIGHT_CM_MIN || cmd->heightCm > AUTO_HEIGHT_CM_MAX || cmd->photoN > AUTO_PHOTO_N_MAX ||
      cmd->photoGapMs < AUTO_PHOTO_GAP_MIN_MS || cmd->photoGapMs > AUTO_PHOTO_GAP_MAX_MS ||
      (cmd->actWaitMs != 0 &&
       (cmd->actWaitMs < AUTO_HOVER_WAIT_TAKEOFF_MIN_MS || cmd->actWaitMs > AUTO_HOVER_WAIT_MAX_MS)) ||
      (plan && !autoMvRangeOk(cmd->moveRightCm, cmd->moveFwdCm, 0))) {
    reply(c, AUTO_R_RANGE, (uint32_t)f->state);
    return;
  }
  const uint32_t g = autoFsmGuards(f, c->in, f->mode);
  c->out->failedGuards = g;
  c->guardsSet = true;
  if (g != 0) {
    reply(c, AUTO_R_GUARDS, g);
    return;
  }
  f->heightCm = cmd->heightCm;
  f->photoN = cmd->photoN;
  f->photoGapMs = cmd->photoGapMs;
  if (cmd->actWaitMs != 0) {
    f->hoverWaitMs = cmd->actWaitMs;   /* this sortie's wait (the Mac's 'sortie --wait-s') */
  }
  f->planRightCm = cmd->moveRightCm;   /* latched on every TAKEOFF: one without a plan clears it */
  f->planFwdCm = cmd->moveFwdCm;
  f->planPending = plan;
  reply(c, AUTO_R_OK, (uint32_t)f->state);
  enter(c, AUTO_TAKEOFF, AUTO_RS_TAKEOFF);
}

static void stepIdle(ctx_t *c)
{
  /* A Mac KILL here only starts the kill latch (done in autoFsmStep). */
  if (c->in->cmd.type == AUTO_CMD_ARM) {
    armCmd(c);
  }
}

static void stepArmed(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const uint32_t g = autoFsmGuards(f, c->in, f->mode);
  c->out->failedGuards = g;
  c->guardsSet = true;

  if (c->kill) {
    enter(c, AUTO_KILLED, AUTO_RS_KILL_CMD);
    return;
  }
  if (stopCmd(c, AUTO_IDLE)) {
    return;
  }
  const autoReason_t why = groundExit(c);
  if (why != AUTO_RS_NONE) {
    enter(c, AUTO_IDLE, why);
  } else if (g & AUTO_G_LOCKS) {
    enter(c, AUTO_IDLE, AUTO_RS_LOCKED);
  } else if (c->now - f->armMs >= AUTO_ARM_TIMEOUT_MS) {
    enter(c, AUTO_IDLE, AUTO_RS_ARM_TIMEOUT);
  } else if (c->in->cmd.type == AUTO_CMD_TAKEOFF) {
    takeoffCmd(c);
  }
}

/* STABLE (section 3.5). A streak starts on a step where every condition
 * holds. A dropout shorter than AUTO_STABLE_GRACE_MS keeps it (its start is
 * kept, so stableMs counts on); it ends on the step where a dropout reaches
 * the grace, where its dropouts together reach AUTO_STABLE_DROP_BUDGET_MS,
 * or on a single step beyond a hard bound. A dropout's length is now -
 * stableOkMs on its last step, the time since the last good step: no step
 * spacing is assumed (on the drone now is the 1 kHz tick and the loop runs
 * slower, so it moves 1 or 2 ms a step). STABLE 1 s also needs the
 * conditions on the step itself, so the photos never start inside a dropout.
 * Written so that NaN fails the conditions and the hard bounds. Per loop in
 * HOVER: IRAM, no calls. */
static AUTO_HOT void updateStable(ctx_t *c, bool vzOk)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const float roll = fabsf(in->rollDeg), pitch = fabsf(in->pitchDeg);
  const float gx = fabsf(in->gyroDps[0]), gy = fabsf(in->gyroDps[1]), gz = fabsf(in->gyroDps[2]);
  /* The camera part is waived in DRY and BENCH: the drift hold needs
   * airborne thrust to engage. */
  const bool flight = f->mode == AUTO_MODE_FLIGHT;
  const bool camRunning = !flight || in->camState == AUTO_CAM_RUNNING;
  const bool cam = !flight || (camRunning && in->camFrameAgeMs <= AUTO_STABLE_FRAME_AGE_MS && in->holdApplied &&
                               !in->holdCoasting);
  const bool s = vzOk && in->vz.holdSteady && cam && roll <= AUTO_STABLE_TILT_DEG && pitch <= AUTO_STABLE_TILT_DEG &&
                 gx <= AUTO_STABLE_GXY_DPS && gy <= AUTO_STABLE_GXY_DPS && gz <= AUTO_STABLE_GZ_DPS;
  const bool hardOk = vzOk && camRunning && roll <= AUTO_STABLE_HARD_TILT_DEG && pitch <= AUTO_STABLE_HARD_TILT_DEG &&
                      gx <= AUTO_STABLE_HARD_GXY_DPS && gy <= AUTO_STABLE_HARD_GXY_DPS && gz <= AUTO_STABLE_HARD_GZ_DPS;
  if (s) {
    if (!f->stable.on) {
      f->stable.on = true;
      f->stable.sinceMs = c->now;
      f->stableDropMs = 0;
    } else {
      f->stableDropMs += f->stableGapMs;   /* a dropout that ended here; 0 otherwise */
    }
    f->stableGapMs = 0;
    f->stableOkMs = c->now;
  } else if (f->stable.on) {
    f->stableGapMs = c->now - f->stableOkMs;
    if (!hardOk || f->stableGapMs >= AUTO_STABLE_GRACE_MS ||
        f->stableDropMs + f->stableGapMs >= AUTO_STABLE_DROP_BUDGET_MS) {
      f->stable.on = false;
    }
  }
  c->stable = s;
  c->stableMs = f->stable.on ? c->now - f->stable.sinceMs : 0;
  c->stable1s = s && c->stableMs >= AUTO_STABLE_MS;
  if (c->stable1s) {
    f->stableOnce = true;
    f->rep.holdBits &= (uint8_t)~AUTO_HB_NOT_STABLE;
  }
}

/* Starts one photo set (the glue sets the capture progress to BUSY). Once per set: flash, out of line (IRAM for the
 * palm catch, docs/autonomy-a2b.md 18.3). */
static __attribute__((noinline)) void startCapture(ctx_t *c, uint8_t n, uint16_t gapMs)
{
  autoFsm_t *f = c->f;
  f->captureStarted = true;
  f->capFailSeen = false;
  f->captureMs = c->now;
  f->capN = n;
  f->capGapMs = gapMs;
  c->out->captureStart = true;
  c->out->captureN = n;
  c->out->captureGapMs = gapMs;
}

/* PHOTO: any time in HOVER (2026-10-15; only while waiting before) with no set in progress. Once per command: flash,
 * out of line. */
static __attribute__((noinline)) void photoCmd(ctx_t *c, bool busy)
{
  autoFsm_t *f = c->f;
  const autoCmd_t *cmd = &c->in->cmd;
  if (f->state != AUTO_HOVER) {
    reply(c, AUTO_R_BAD_STATE, (uint32_t)f->state);
  } else if (busy) {
    reply(c, AUTO_R_BUSY, (uint32_t)f->state);
  } else if (cmd->photoN < 1 || cmd->photoN > AUTO_PHOTO_N_MAX || cmd->photoGapMs < AUTO_PHOTO_GAP_MIN_MS ||
             cmd->photoGapMs > AUTO_PHOTO_GAP_MAX_MS) {
    reply(c, AUTO_R_RANGE, (uint32_t)f->state);
  } else {
    startCapture(c, cmd->photoN, cmd->photoGapMs);
    f->rep.lastCmdMs = c->now;
    f->rep.lastCmdAny = true;
    reply(c, AUTO_R_OK, (uint32_t)f->state);
  }
}

/* The result code of a refused MOVE or YAW (its reason goes in the detail). */
static autoResult_t mvResult(autoMvWhy_t w)
{
  switch (w) {
    case AUTO_MVW_RANGE:
    case AUTO_MVW_HEIGHT:
    case AUTO_MVW_TOO_CLOSE:
    case AUTO_MVW_AIR:
    case AUTO_MVW_SH_STEP:
      return AUTO_R_RANGE;
    case AUTO_MVW_NO_DIST:
    case AUTO_MVW_NO_VIEW:
    case AUTO_MVW_NO_CAM_HEIGHT:
    case AUTO_MVW_SH_CONTACT:
      return AUTO_R_CAM_UNAVAIL;
    case AUTO_MVW_BATT:
    case AUTO_MVW_UP_HEADROOM:
      return AUTO_R_LOW_BATT;
    default:
      return AUTO_R_BAD_STATE;   /* NOT_WAITING, BUSY, SH_RATE: final, never BUSY (the Mac would resend it) */
  }
}

/* ACK detail of MOVE / YAW / STOP: bits 0-7 the state, 8-15 the reason
 * (autoMvWhy_t; STOP: the motion state it found), 16-31 the motion seq. */
static uint32_t mvDetail(const autoFsm_t *f, uint8_t code)
{
  return (uint32_t)f->state | ((uint32_t)code << 8) | ((uint32_t)f->mv.seq << 16);
}

/* m to cm, rounded half away from zero, within i16 (the ACK's old estimate; SET_HEIGHT's "d rounds to 0 cm"). */
static int32_t roundCm(float m)
{
  const float x = m * 100.0f;
  if (!(x > -32767.0f)) {
    return -32767;   /* (NaN too) */
  }
  if (!(x < 32767.0f)) {
    return 32767;
  }
  return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

/* ACK detail of SET_HEIGHT: mvDetail()'s layout with the motion seq replaced by the estimate when the command came
 * (the old one, cm, i16; 0 with the controller inactive): bits 0-7 the state, 8-15 the reason (autoMvWhy_t, 0 OK). */
static uint32_t shDetail(const autoFsm_t *f, uint8_t why, float hNow)
{
  return (uint32_t)f->state | ((uint32_t)why << 8) | ((uint32_t)(uint16_t)(int16_t)roundCm(hNow) << 16);
}

/* A climb's thrust headroom (auto_fsm.h AUTO_MV_UP_HEADROOM_*): the ceiling in force at least min(4600, 10 % of
 * T_hold) above T_hold. Before HOLD (T_hold exactly 0) nothing climbs on a command anyway: accepted. A NaN, infinite
 * or negative T_hold, or a NaN ceiling, refuses (2026-10-16 review: the build before accepted a NaN T_hold through
 * battLow's compare; vert_ctrl never publishes one in HOLD, so only the comment's promise changes). Flash: per
 * command. */
static bool upHeadroomOk(const autoFsmIn_t *in)
{
  const float t = in->vz.tHold;
  if (t == 0.0f) {
    return true;
  }
  if (!(t > 0.0f && isfinite(t))) {
    return false;
  }
  const float need = fminf(AUTO_MV_UP_HEADROOM_T, AUTO_MV_UP_HEADROOM_REL * t);
  return in->vz.ceiling - t >= need;   /* (a NaN ceiling: false) */
}

/* SET_HEIGHT in HOVER (auto_fsm.h AUTO_SH_*, docs/autonomy-a2b.md 23): the checks in their order, the first failure
 * answers; accepted, the motion's correction starts (autoMotionSetHeight), the glue shifts the controller's estimate on
 * this step (out->hShift), and this step's motion inputs see the shifted estimate (mi->vzH += d: the correction's
 * measured progress starts at 0; with the old one the vertical guard, 0.15 m + 0.5 |up|, stopped every correction of
 * 0.3 m or more on its first step). No event: the trim learner skips any sortie with one (att_trim_auto.c), and a
 * correction is routine. Once per command: flash, out of line. */
static __attribute__((noinline)) void setHeightCmd(ctx_t *c, autoMvIn_t *mi)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const autoCmd_t *cmd = &in->cmd;
  const float hNow = mi->vzH;   /* the estimate now (the previous step's controller output; 0 when inactive) */
  float d = 0.0f, upM = 0.0f;
  autoMvWhy_t w = AUTO_MVW_NONE;
  if (!(f->state == AUTO_HOVER && f->waiting && in->vz.phase == VZ_HOLD)) {
    w = AUTO_MVW_NOT_WAITING;
  } else if (autoMotionActive(&f->mv)) {
    w = AUTO_MVW_BUSY;   /* a MOVE, a YAW, a bob or re-view, a running correction, the take-off's planned move */
  } else if (f->contactBlind || (f->rep.holdBits & AUTO_HB_HUSH) != 0u) {
    w = AUTO_MVW_SH_CONTACT;   /* the estimate after a contact is not the camera's yet */
  } else if (!mi->camHeight) {
    w = AUTO_MVW_NO_CAM_HEIGHT;   /* open loop: the controller ignores vRef, the correction could not fly */
  } else if (mi->flight && !(mi->holdApplied && !mi->holdCoasting && mi->camFrameAgeMs <= AUTO_MV_VIEW_MAX_AGE_MS)) {
    w = AUTO_MVW_NO_VIEW;   /* as any vertical MOVE (auto_motion.c viewOk()) */
  } else if (f->setHeightMs != 0u && c->now - f->setHeightMs < AUTO_SH_GAP_MS) {
    w = AUTO_MVW_SH_RATE;
  } else {
    /* the offset form in whole cm (exact); the absolute form against the estimate now */
    const bool atNow = cmd->moveUpCm == AUTO_SH_AT_NOW;
    d = atNow ? (float)cmd->heightCm * 0.01f - hNow : (float)((int32_t)cmd->heightCm - (int32_t)cmd->moveUpCm) * 0.01f;
    const float hNew = hNow + d;
    /* the "go to the target" form (6 bytes, 2026-10-16): the commanded height (the take-off height + the committed
     * moves) or an explicit one; the reference then starts at the corrected estimate */
    const bool toForm = cmd->mode == AUTO_SH_FORM_TO;
    const bool toRef = cmd->moveRightCm == AUTO_SH_TO_REF;
    const float tgt = toRef ? mi->hTargetM + f->mv.hOff : (float)cmd->moveRightCm * 0.01f;
    float refShift = d;
    if (toForm) {
      upM = tgt - hNew;
      refShift = hNew - mi->hTargetM - f->mv.hOff;   /* hOff = h_new - hTarget at acceptance, target - hTarget at DONE */
    } else {
      /* the flight: back to the height the drone believed it held (at most the move cap), in the corrected frame:
       * min(h_now, cap) - h_new, written so that it is exactly -d below the cap (the DONE restores the offset
       * exactly) */
      upM = hNow <= AUTO_MV_H_MAX_M ? -d : AUTO_MV_H_MAX_M - hNew;
    }
    /* |d| in whole cm, as the offset form's: X - the estimate now in the absolute form may round either way; the
     * target form may have d 0 (the estimate was right: the flight alone), not both 0 */
    const int32_t dCm = roundCm(d), dMaxCm = (int32_t)(AUTO_SH_MAX_M * 100.0f + 0.5f);
    const bool nothing = toForm ? (dCm == 0 && roundCm(upM) == 0) : dCm == 0;
    if (cmd->heightCm < AUTO_SH_X_CM_MIN || cmd->heightCm > AUTO_SH_X_CM_MAX || !isfinite(d) || !isfinite(upM) ||
        !isfinite(refShift) || nothing ||
        (toForm && !toRef && (cmd->moveRightCm < AUTO_SH_TO_CM_MIN || cmd->moveRightCm > AUTO_SH_TO_CM_MAX))) {
      w = AUTO_MVW_RANGE;
    } else if (dCm > dMaxCm || dCm < -dMaxCm || fabsf(upM) > AUTO_SH_FLY_MAX_M + 1e-4f) {
      w = AUTO_MVW_SH_STEP;
    } else if (!(hNew >= AUTO_SH_H_MIN_M - 1e-4f && hNew <= VZ_H_TARGET_MAX + AUTO_SH_MAX_M + 1e-4f)) {
      w = AUTO_MVW_HEIGHT;
    } else if (toForm && upM < 0.0f && !(tgt >= autoMotionHeightFloor(mi) - 1e-4f)) {
      w = AUTO_MVW_HEIGHT;   /* the target form's descent keeps a MOVE down's floor (2026-10-16 review): 0.25 m + the
                                drift the estimate may have gathered since the take-off (auto_motion.c heightFloor);
                                an old photo can leave the drone lower than X says (the tiger correction) */
    } else if (upM > 0.0f && !upHeadroomOk(in)) {
      w = AUTO_MVW_UP_HEADROOM;   /* a flight up without the thrust a climb needs (2026-10-16): the thrust, not the
                                     voltage */
    } else if (mi->battLow) {
      w = AUTO_MVW_BATT;   /* near the end of the battery no correction flies (2026-10-15 review: a descending one
                              accepted at T_hold 54600-56000 ended HOLD_CAPPED in the drain sim, auto_motion.c) */
    } else {
      w = autoMotionSetHeightRef(&f->mv, mi, d, upM, refShift);   /* (its own checks pass: they were made above) */
    }
  }
  if (w != AUTO_MVW_NONE) {
    f->mv.why = w;   /* AUTO_STATUS mvWhy: the latest refusal */
    reply(c, mvResult(w), shDetail(f, (uint8_t)w, hNow));
    return;
  }
  c->out->hShift = d;   /* the glue: vzShiftHeight() before this step's vzStep() */
  mi->vzH = hNow + d;   /* this step's autoMotionStep sees the shifted estimate (progress 0) */
  f->setHeightMs = c->now | 1u;
  f->waitMs = c->now;
  reply(c, AUTO_R_OK, shDetail(f, 0, hNow));
}

/* MOVE, YAW, STOP, SET_HEIGHT in HOVER (interactive hover, auto_motion.h): a MOVE, YAW or SET_HEIGHT only while
 * waiting (the first STABLE photos done); an accepted one restarts the wait. STOP is never refused. Once per command:
 * flash, out of line (internal RAM is short; auto_motion.c's AUTO_COLD). */
static __attribute__((noinline)) void motionCmd(ctx_t *c, autoMvIn_t *mi)
{
  autoFsm_t *f = c->f;
  const autoCmd_t *cmd = &c->in->cmd;
  f->rep.lastCmdMs = c->now;   /* Claude's latest command (the status's sinceCmd), refused ones too */
  f->rep.lastCmdAny = true;
  if (cmd->type == AUTO_CMD_SET_HEIGHT) {
    setHeightCmd(c, mi);
    return;
  }
  if (cmd->type == AUTO_CMD_STOP) {
    const uint8_t was = (uint8_t)f->mv.state;
    f->planPending = 0;   /* a STOP before the wait cancels the planned move (in it: the move's own STOP) */
    autoMotionStop(&f->mv, mi, AUTO_MVE_STOP_CMD);
    if (f->waiting) {
      f->waitMs = c->now;
    }
    reply(c, AUTO_R_OK, mvDetail(f, was));
    return;
  }
  autoMvWhy_t w;
  if (!(f->state == AUTO_HOVER && f->waiting)) {   /* the wait always starts since 2026-10-15 (no STABLE needed) */
    w = AUTO_MVW_NOT_WAITING;
    f->mv.why = w;
  } else if (cmd->type == AUTO_CMD_MOVE && cmd->moveUpCm > 0 && !autoMotionActive(&f->mv) &&
             autoMvRangeOk(cmd->moveRightCm, cmd->moveFwdCm, cmd->moveUpCm) && !upHeadroomOk(c->in)) {
    w = AUTO_MVW_UP_HEADROOM;   /* a MOVE up without a climb's thrust headroom (2026-10-16; BUSY and RANGE first) */
    f->mv.why = w;
  } else if (cmd->type == AUTO_CMD_MOVE) {
    w = autoMotionMove(&f->mv, mi, cmd->moveRightCm, cmd->moveFwdCm, cmd->moveUpCm);
  } else {
    w = autoMotionYaw(&f->mv, mi, cmd->yawDeg);
  }
  if (w == AUTO_MVW_NONE) {
    f->waitMs = c->now;
    reply(c, AUTO_R_OK, mvDetail(f, 0));
  } else {
    reply(c, mvResult(w), mvDetail(f, (uint8_t)w));
  }
}

/* Rows 25-26, the take-off photos done (or photoN 0 and STABLE 1 s): without a hover wait LAND SORTIE_DONE; else
 * the wait starts, and the TAKEOFF's planned move with it (auto_fsm.h; docs/autonomy-a2b.md 21.4): a MOVE with
 * every acceptance check on the live scene distance at the wait's start (NO_DIST, TOO_WIDE, TOO_CLOSE, NO_VIEW, BATT,
 * AIR; the forward floor the Mac's AUTO_PLAN_MIN_FRONT_CM, which the Mac cannot check before the take-off, 21.9);
 * refused, its reason stays in mv.why and the drone hovers in the wait. The wait's start, not the HOVER entry:
 * by then (STABLE 1 s and the take-off photos, HOVER + ~1.5 s) the observer's G is within ~1 % of its settled value
 * (16-27 % high at the HOVER entry: a move 16-27 % too long). The motion steps from the next step on (this one's
 * outputs were made). Once per sortie: flash, out of line (it pays for its own call in IRAM: the two rows' code
 * was there). */
static __attribute__((noinline)) void waitStart(ctx_t *c, const autoMvIn_t *mi)
{
  autoFsm_t *f = c->f;
  /* row 25 (LAND SORTIE_DONE without a wait) removed 2026-10-15: a hop is Claude's own LAND */
  f->waiting = true;                                  /* row 26: hover in place for the Mac */
  f->waitMs = c->now;   /* (an accepted plan holds the wait from the next step on, as any motion) */
  if (f->planPending) {
    f->planPending = 0;
    (void)autoMotionMovePlan(&f->mv, mi, f->planRightCm, f->planFwdCm, 0);   /* the Mac's 0.60 m floor */
  }
}

/* Commands no row consumed (table 3.3): stop-type commands are never refused (a STOP outside HOVER finds nothing
 * running, and cancels the TAKEOFF's planned move: a STOP in the climb); ARM and TAKEOFF are refused in the wrong
 * state. Once per command: flash, out of line (in IRAM until 2026-10-14; moved to pay for the planned move's and
 * the guards' IRAM, docs/autonomy-a2b.md 21.6). */
static __attribute__((noinline)) void cmdOutside(ctx_t *c, autoState_t before)
{
  autoFsm_t *f = c->f;
  switch (c->in->cmd.type) {
    case AUTO_CMD_LAND:
    case AUTO_CMD_DISARM:
    case AUTO_CMD_KILL:
      reply(c, AUTO_R_OK, (uint32_t)before);
      break;
    case AUTO_CMD_STOP:   /* nothing runs outside HOVER: done */
      f->planPending = 0;
      reply(c, AUTO_R_OK, (uint32_t)before | ((uint32_t)f->mv.state << 8) | ((uint32_t)f->mv.seq << 16));
      break;
    case AUTO_CMD_MOVE:
    case AUTO_CMD_YAW:
      f->mv.why = AUTO_MVW_NOT_WAITING;
      reply(c, AUTO_R_BAD_STATE,
            (uint32_t)before | ((uint32_t)AUTO_MVW_NOT_WAITING << 8) | ((uint32_t)f->mv.seq << 16));
      break;
    case AUTO_CMD_SET_HEIGHT:   /* IDLE, ARMED, TAKEOFF, LAND (SINK too), KILLED, or a HOVER step a row ended first */
      f->mv.why = AUTO_MVW_NOT_WAITING;
      reply(c, AUTO_R_BAD_STATE, (uint32_t)before | ((uint32_t)AUTO_MVW_NOT_WAITING << 8));
      break;
    case AUTO_CMD_ARM:
    case AUTO_CMD_TAKEOFF:
    case AUTO_CMD_PHOTO:
      reply(c, AUTO_R_BAD_STATE, (uint32_t)before);
      break;
    default:
      reply(c, AUTO_R_RANGE, (uint32_t)before);
      break;
  }
}

/* TAKEOFF after the pre-spin, and HOVER (section 3.2, rows 1-27). */
static void stepAirborne(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const bool vzOk = in->vz.phase != VZ_IDLE;   /* the glue zeroes the output at vzReset */
  const bool hover = f->state == AUTO_HOVER;
  const vzPhase_t ph = in->vz.phase;

  /* The air clock starts at the first thrust above the idle spin (RAMP),
   * because the detected lift-off can lag the true one by over a second. */
  if (vzOk && !f->rampSeen && (ph == VZ_RAMP || ph == VZ_CLIMB || ph == VZ_HOLD || in->vz.liftedOff)) {
    f->rampSeen = true;
    f->rampMs = c->now;
  }
  const uint32_t air = f->rampSeen ? c->now - f->rampMs : 0;
  if (hover) {
    updateStable(c, vzOk);
  }

  /* Cut at once: rows 1-8. */
  autoReason_t why = AUTO_RS_NONE;
  if (c->kill) {
    why = AUTO_RS_KILL_CMD;
  } else if (!inputsFinite(in, vzOk)) {
    why = AUTO_RS_NAN;
  } else {
    why = trip(c);
    if (why == AUTO_RS_NONE) {
      why = motorFault(in);
    }
    if (why == AUTO_RS_NONE && vzOk && ph == VZ_CUT) {
      f->vzLandReason = in->vz.reason;
      why = AUTO_RS_VZ_CUT;
    }
  }
  if (why != AUTO_RS_NONE) {
    enter(c, AUTO_KILLED, why);
    return;
  }

  /* In-air contact (row 8b; auto_fsm.h AUTO_COLLIDE_*, AUTO_CONTACT_*): armed in the controller's CLIMB and HOLD
   * from AUTO_COLLIDE_ARM_MS after its lift-off (its tAir; CLIMB or HOLD also means vzOk), in FLIGHT and DRY, once the
   * previous hush (and the chain bound) allow (contactNextMs). contactStart() (flash): in the climb without the camera
   * height the blind LAND COLLISION (it returns true); else CONTACT: the hush, the motion stopped or a snap, the event
   * (since 2026-10-15; a LAND until then). It does not return: every LAND and KILL row below still runs. */
  if (heldFor(&f->collide, c->now, AUTO_COLLIDE_MS) && f->mode != AUTO_MODE_BENCH &&
      (ph == VZ_CLIMB || ph == VZ_HOLD) && in->vz.tAir >= AUTO_COLLIDE_ARM_MS * 0.001f &&
      (int32_t)(c->now - f->contactNextMs) >= 0 && contactStart(c)) {
    return;
  }
  if (f->contactBlind) {
    contactStep(c);   /* a contact's hush (vzInput_t.hush, the peaks), then blind until the camera is back */
  }

  /* Land: rows 9-19. */
  if (vzOk && (ph == VZ_LAND || ph == VZ_DONE)) {
    f->vzLandReason = in->vz.landReason;
    enter(c, AUTO_LAND, AUTO_RS_VZ);
    return;
  }
  if (stopCmd(c, AUTO_LAND)) {
    return;
  }
  if (c->appInput) {
    why = AUTO_RS_APP_INPUT;
  } else if (in->hbAgeMs > AUTO_HB_LAND_MS) {
    why = AUTO_RS_HB_LOST;
  } else if (in->appRequired && in->appAgeMs > AUTO_APP_SILENT_MS) {
    why = AUTO_RS_APP_SILENT;
  } else if (in->sessionChanged) {
    why = AUTO_RS_SESSION;
  } else if (heldFor(&f->vbatLow, c->now, AUTO_VBAT_LAND_MS)) {
    why = AUTO_RS_VBAT_LOW;
  } else if (hover && heldFor(&f->battLow, c->now, AUTO_VBAT_LOW_LAND_MS)) {
    why = AUTO_RS_BATT_LOW;
  } else if (in->vbatFastMin > 0.0f && in->vbatFastMin < AUTO_VBAT_FAST_LAND_V) {
    why = AUTO_RS_VBAT_DIP;
  } else if (in->loopGapUs > AUTO_LOOP_GAP_LAND_US && gapEvent(c)) {
    why = AUTO_RS_LOOP_GAP;   /* the third gap within 1 s (one: the event, the motion stopped) */
  } else if (!hover && c->now - f->stateSinceMs >= AUTO_TAKEOFF_MAX_MS + climbExtraMs(f)) {
    why = AUTO_RS_TAKEOFF_TIMEOUT;
  }
  if (why != AUTO_RS_NONE) {
    enter(c, AUTO_LAND, why);
    return;
  }

  if (!hover) {
    if (vzOk && ph == VZ_HOLD) {   /* row 20 */
      enter(c, AUTO_HOVER, AUTO_RS_HOLD);
    }
    return;
  }

  /* The interactive hover: a MOVE, YAW, STOP or SET_HEIGHT of this step first (an
   * accepted motion runs from this step), then the motion's references and
   * guards; a guard stops the motion (mvStopReq(), since 2026-10-15; a LAND until then). */
  autoMvIn_t mi;
  mvIn(c, &mi);
  const autoCmdType_t ct = in->cmd.type;
  if ((unsigned)ct - (unsigned)AUTO_CMD_MOVE <= (unsigned)(AUTO_CMD_SET_HEIGHT - AUTO_CMD_MOVE)) {
    motionCmd(c, &mi);   /* MOVE, YAW, STOP, SET_HEIGHT (consecutive: auto_fsm.h) */
  }
  const autoMvLand_t ml = autoMotionStep(&f->mv, &mi, &c->out->mv);
  c->mvStepped = true;
  if (ml != AUTO_MVL_NONE) {
    mvStopReq(c, &mi, ml);   /* a guard's stop, or the re-track's turn back (UNDO: the report only) */
  }
  if (c->out->mv.holdWait && f->waiting) {
    f->waitMs = c->now;
  }

  /* HOVER: rows 21-27 ("Claude decides", 2026-10-15: none of them lands but the camera rule's SINK). The capture state
   * is only read for a set this sortie started; it describes the latest set. */
  const bool capDone = f->captureStarted && in->capture == AUTO_CAP_DONE;
  const bool capFail = f->captureStarted && !capDone && !f->capFailSeen &&
                       (in->capture == AUTO_CAP_FAILED ||
                        c->now - f->captureMs > AUTO_CAPTURE_TIMEOUT_MS(f->capN, f->capGapMs));
  const bool camWaived = f->mode != AUTO_MODE_FLIGHT;
  const uint32_t inHover = c->now - f->holdMs;
  /* row 21, the camera rule (auto_fsm.h AUTO_CAM_*): from hoverAccelMs on, at once after a contact or a climb fault */
  const bool camIn = vzOk && in->vz.camHealthy;
  if (!camWaived && (!camIn || f->camLostOn) && camRule(c, camIn)) {
    return;   /* LAND SINK */
  }
  /* the hold's view without a motion (FLIGHT): HOLD_NO_VIEW / VIEW_BACK (viewRule(), flash: only while it is bad or
   * coming back; nothing in flash runs every step of a normal hover) */
  const bool viewBad = !camWaived && !(in->holdApplied && !in->holdCoasting &&
                                       in->camFrameAgeMs <= AUTO_MV_VIEW_MAX_AGE_MS);
  if ((viewBad || f->viewBadOn) && !autoMotionActive(&f->mv)) {
    viewRule(c, viewBad);
  }
  /* row 22 (HOVER_TIMEOUT) removed 2026-10-15: the wait is information only */
  if (capFail) {
    captureFail(c, &mi);                                /* row 23: the event, the wait starts */
  } else if (!f->stableOnce && !f->notStableSeen && inHover >= AUTO_NOT_STABLE_HOLD_MS &&
             (camWaived || in->camViewMs >= AUTO_NOT_STABLE_CAM_MS)) {
    notStable(c, &mi);                                  /* row 24: the event, the wait starts */
  } else if (!f->waiting && f->rampSeen && air >= f->airMaxMs + climbExtraMs(f) &&
             !(AUTO_CAPTURE_FRAME_MS > 0u && f->captureStarted && !f->capFailSeen)) {
    /* A stable hover waiting for native JPEG work is not a slow take-off.
     * capFail above bounds a stalled encoder; other flight guards still run. */
    takeoffSlow(c, &mi);                                /* the wait starts anyway (the air clock landed until then) */
  } else if (!f->waiting && (capDone || (f->photoN == 0 && c->stable1s))) {
    waitStart(c, &mi);   /* row 26: the wait, and the planned move */
  } else if (c->stable1s && f->photoN > 0 && !f->captureStarted) {
    startCapture(c, f->photoN, f->photoGapMs);          /* row 27 */
  }
  if (f->state == AUTO_HOVER && in->cmd.type == AUTO_CMD_PHOTO) {
    photoCmd(c, (f->captureStarted && !capDone && !f->capFailSeen) || c->out->captureStart);
  }
}

static void stepTakeoff(ctx_t *c)
{
  autoFsm_t *f = c->f;
  if (f->prespinDone) {
    stepAirborne(c);
    return;
  }
  /* Pre-spin: motors off, LED warning, stillness re-checked. */
  if (c->kill) {
    enter(c, AUTO_KILLED, AUTO_RS_KILL_CMD);
    return;
  }
  if (stopCmd(c, AUTO_IDLE)) {
    return;
  }
  const autoReason_t why = groundExit(c);
  if (why != AUTO_RS_NONE) {
    enter(c, AUTO_IDLE, why);
  } else if (locked(c->in, f->mode)) {
    enter(c, AUTO_IDLE, AUTO_RS_LOCKED);   /* do not spin up into a lock */
  } else if (c->now - f->stateSinceMs >= AUTO_PRESPIN_MS) {
    /* The controller starts on this step; the airborne rows from the next
     * one, when in.vz is this sortie's output. */
    f->prespinDone = true;
    c->out->vzReset = true;
  }
}

static void stepLand(ctx_t *c)
{
  autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  const bool vzOk = in->vz.phase != VZ_IDLE;
  const vzPhase_t ph = in->vz.phase;

  /* Contact evidence for row 3 (a NaN compares false: no evidence). */
  if (in->accNormG > AUTO_TD_CONTACT_G || (vzOk && in->vz.aRel > AUTO_TD_JOLT_G)) {
    f->contactSeen = true;
    f->contactMs = c->now;
  }

  /* Rows 1-2. */
  autoReason_t why = AUTO_RS_NONE;
  if (c->kill) {
    why = AUTO_RS_KILL_CMD;
  } else if (!inputsFinite(in, vzOk)) {
    why = AUTO_RS_NAN;
  } else {
    why = motorFault(in);
  }
  if (why != AUTO_RS_NONE) {
    enter(c, AUTO_KILLED, why);
    return;
  }

  /* LAND SINK (the camera rule): a LAND that Claude, the user or a minimum rule asks for takes it over; else the
   * camera height back resumes it (sinkStep(), flash, while it lasts). After the cut rows, before the trips. */
  if (f->reason == AUTO_RS_SINK && !f->landEnded && sinkStep(c)) {
    return;   /* HOVER RESUME */
  }

  /* Rows 3-7, while the controller is still landing. Once it has ended the
   * thrust is already 0: later phases and trips change nothing. */
  if (!f->landEnded) {
    autoReason_t end = AUTO_RS_NONE;
    const autoReason_t t = trip(c);
    if (t != AUTO_RS_NONE) {
      /* A touchdown under power trips rate or tilt: not a kill, but only
       * with low thrust AND contact just before. The controller's thrust
       * condition alone can hold in mid-air: a LAND from CLIMB within
       * tHoldTauS of the lift-off or of a capped climb step (T_ref = T_lo),
       * or on a drone lighter than its stored gain, starts below it. A free
       * fall or a tracking error is never a touchdown. */
      const bool contact = f->contactSeen && c->now - f->contactMs <= AUTO_TD_CONTACT_WIN_MS;
      if (!((t == AUTO_RS_TILT || t == AUTO_RS_RATE) && vzOk && in->vz.tdThrustOk && contact)) {
        enter(c, AUTO_KILLED, t);
        return;
      }
      end = AUTO_RS_TOUCHDOWN_TRIP;
      f->landTrip = t;
      f->landCut = true;
    } else if (vzOk && ph == VZ_DONE) {
      end = AUTO_RS_TOUCHDOWN;
      f->vzLandReason = in->vz.reason;
    } else if (vzOk && ph == VZ_CUT) {
      end = in->vz.reason == VZ_R_LAND_TIMEOUT ? AUTO_RS_LAND_TIMEOUT : AUTO_RS_VZ_CUT;
      f->vzLandReason = in->vz.reason;
    } else if (c->now - f->stateSinceMs >= f->landBackstopMs) {
      enter(c, AUTO_KILLED, AUTO_RS_LAND_BACKSTOP);
      return;
    }
    if (end != AUTO_RS_NONE) {
      f->landEnded = true;
      f->landEndMs = c->now;
      f->landEndReason = end;
    }
  }

  /* Row 8. */
  if (f->landEnded && (heldSince(&f->rest, c->now, f->landEndMs, AUTO_LAND_REST_MS) ||
                       c->now - f->landEndMs >= AUTO_LAND_REST_MAX_MS)) {
    enter(c, AUTO_IDLE, f->landEndReason);
  }
}

static void stepKilled(ctx_t *c)
{
  const autoFsm_t *f = c->f;
  const autoFsmIn_t *in = c->in;
  if (c->now - f->stateSinceMs >= AUTO_KILLED_MIN_MS &&
      heldSince(&f->rest, c->now, f->stateSinceMs, AUTO_KILLED_REST_MS) &&
      (in->appAgeMs > AUTO_APP_ARM_MAX_AGE_MS || in->appThrustZero)) {
    enter(c, AUTO_IDLE, AUTO_RS_KILL_CLEARED);
  }
}

static float modeMax(const autoFsm_t *f)
{
  switch (f->mode) {
    case AUTO_MODE_FLIGHT:
      return VZ_THRUST_ABS_MAX;
    case AUTO_MODE_BENCH:
      return f->benchCap;
    case AUTO_MODE_DRY:
    default:
      return 0.0f;
  }
}

void autoFsmSetMvSpeed(autoFsm_t *f, uint16_t pct)
{
  autoMotionSetSpeed(&f->mv, pct);
}

void autoFsmInit(autoFsm_t *f)
{
  memset(f, 0, sizeof(*f));
  f->state = AUTO_IDLE;
  f->mode = AUTO_MODE_FLIGHT;
  f->reason = AUTO_RS_NONE;
  f->landEndReason = AUTO_RS_NONE;
  f->vzLandReason = VZ_R_NONE;
}

AUTO_HOT void autoFsmStep(autoFsm_t *f, const autoFsmIn_t *in, autoFsmOut_t *out)
{
  const uint32_t now = in->nowMs;
  const autoState_t before = f->state;
  memset(out, 0, sizeof(*out));

  ctx_t c = {
    .f = f, .in = in, .out = out, .now = now,
    .kill = in->killCount != f->lastKillCount || in->cmd.type == AUTO_CMD_KILL,
    .appInput = in->appInputCount != f->lastAppInputCount,
  };
  f->lastKillCount = in->killCount;
  f->lastAppInputCount = in->appInputCount;
  if (c.kill) {
    f->killSeen = true;   /* kill latch, in every state */
    f->killMs = now;
  }

  /* Persistence timers run in every state, so a condition that started a
   * moment before a state change keeps its age. */
  track(&f->tilt, in->tiltDeg > AUTO_TILT_KILL_DEG, now);
  track(&f->rate, fabsf(in->gyroDps[0]) > AUTO_RATE_KILL_DPS || fabsf(in->gyroDps[1]) > AUTO_RATE_KILL_DPS, now);
  track(&f->fall, in->accNormG < AUTO_FALL_KILL_G, now);
  track(&f->attErr, in->attErrDeg > AUTO_ATTERR_KILL_DEG, now);
  track(&f->collide, in->accNormG > AUTO_COLLIDE_ACC_G || fabsf(in->gyroDps[0]) > AUTO_COLLIDE_DPS ||
                     fabsf(in->gyroDps[1]) > AUTO_COLLIDE_DPS, now);
  track(&f->vbatLow, in->vbat < AUTO_VBAT_LAND_V, now);
  /* BATT_LOW's two triggers (auto_fsm.h): the battery floor, or the hover's thrust headroom. tHold is 0 before
   * HOLD and the glue zeroes vz at vzReset and IDLE, so no phase test; BATT_LOW itself is HOVER only. */
  track(&f->battLow, in->vbat < AUTO_VBAT_LOW_LAND_V || in->vz.tHold > AUTO_HEADROOM_LAND_T, now);
  track(&f->moved, moved(in), now);
  track(&f->rest, atRest(in), now);

  /* A gcap change can only lower the bench cap. */
  if (f->mode == AUTO_MODE_BENCH && in->gcap > 0 && (float)in->gcap < f->benchCap) {
    f->benchCap = (float)in->gcap;
  }

  /* the controller's events (TAKEOFF..LAND: the glue zeroes its output at vzReset and IDLE) */

  switch (f->state) {
    case AUTO_IDLE:
      stepIdle(&c);
      break;
    case AUTO_ARMED:
      stepArmed(&c);
      break;
    case AUTO_TAKEOFF:
    case AUTO_HOVER:
      stepTakeoff(&c);
      break;
    case AUTO_LAND:
      stepLand(&c);
      break;
    case AUTO_KILLED:
      stepKilled(&c);
      break;
    default:
      enter(&c, AUTO_KILLED, AUTO_RS_NONE);   /* corrupted state: cut */
      break;
  }

  /* the controller's events (TAKEOFF..LAND before this step: the glue zeroes its output at vzReset and IDLE), after
   * the state machine's own of this step (here, not before the step: 356 B less IRAM in autoFsmStep) */
  if (in->vz.evtN != f->vzEvtSeen && before >= AUTO_TAKEOFF && before <= AUTO_LAND) {
    vzEvtFwd(f, in);
  }
  /* Commands no row consumed (table 3.3; cmdOutside, flash). */
  if (in->cmd.type != AUTO_CMD_NONE && !out->cmdDone) {
    cmdOutside(&c, before);
  }

  switch (f->state) {
    case AUTO_IDLE:
      if (f->releaseHold && now - f->idleSinceMs < AUTO_RELEASE_HOLD_MS) {
        out->thrust = AUTO_THR_ZERO;
      } else {
        f->releaseHold = false;
        out->thrust = AUTO_THR_NONE;
      }
      out->vzCmd = VZ_CMD_NONE;
      if (!c.guardsSet) {
        out->failedGuards = autoFsmGuards(f, in, AUTO_MODE_FLIGHT);
      }
      break;
    case AUTO_ARMED:
      out->thrust = AUTO_THR_ZERO;
      out->vzCmd = VZ_CMD_NONE;
      break;
    case AUTO_TAKEOFF:
      out->thrust = f->prespinDone ? AUTO_THR_VERT_CTRL : AUTO_THR_ZERO;
      out->vzCmd = f->prespinDone ? VZ_CMD_TAKEOFF : VZ_CMD_NONE;
      out->prespinLed = !f->prespinDone;
      break;
    case AUTO_HOVER:
      out->thrust = AUTO_THR_VERT_CTRL;
      out->vzCmd = VZ_CMD_TAKEOFF;
      out->stable = c.stable;
      out->stableMs = c.stableMs;
      out->waiting = f->waiting;   /* (waitLeftMs 0: the wait is information only) */
      break;
    case AUTO_LAND:
      out->thrust = f->landEnded ? AUTO_THR_ZERO : AUTO_THR_VERT_LAND;
      out->vzCmd = f->landCut ? VZ_CMD_CUT : (out->resume ? VZ_CMD_TAKEOFF : VZ_CMD_LAND);   /* a SINK resuming */
      break;
    case AUTO_KILLED:
    default:
      out->thrust = AUTO_THR_CUT;
      out->vzCmd = VZ_CMD_CUT;
      break;
  }

  out->state = f->state;
  out->mode = f->mode;
  out->thrustMax = out->thrust == AUTO_THR_NONE ? AUTO_THRUST_FREE : modeMax(f);
  out->hTargetM = f->heightCm != 0 ? (float)f->heightCm / 100.0f : VZ_H_TARGET_DEFAULT;
  out->stateMs = now - f->stateSinceMs;
  out->airMs = (f->state != AUTO_IDLE && f->state != AUTO_ARMED && f->rampSeen) ? now - f->rampMs : 0;
  out->reason = f->reason;
  out->vzLandReason = f->vzLandReason;
  out->landTrip = f->landTrip;
  out->camNoStandby = f->camAwake && (f->state == AUTO_ARMED || f->state == AUTO_TAKEOFF || f->state == AUTO_HOVER);
  /* The motion outputs: HOVER filled them in its step (when it was stepped
   * there); anywhere else the offsets and epoch the last step left. */
  if (!c.mvStepped || f->state != AUTO_HOVER) {
    autoMotionIdleOut(&f->mv, &out->mv);
  }
  /* On the ground (IDLE, ARMED, KILLED) the hold gets no reference: camera_hold
   * also runs in manual flight, and a finished move's end kept there would be
   * flown again on the next manual take-off. LAND keeps it (the hold is still
   * engaged on the way down; dropping it inside the epoch would fly the drone
   * back toward the move's start). */
  if (f->state == AUTO_IDLE || f->state == AUTO_ARMED || f->state == AUTO_KILLED) {
    out->mv.refLat = 0.0f;
    out->mv.refFwd = 0.0f;
  }
  out->battLowNow = f->battLow.on;
  out->battLowHeld = heldFor(&f->battLow, now, AUTO_VBAT_LOW_REFUSE_MS);
  out->heightCm = f->heightCm;
  out->collided = f->collided;
  out->blind = f->rep.contacts != 0u;   /* (B): any contact this sortie (the report is cleared at ARM; COLLISION counts) */
  out->fwdLn = autoMotionFwdLn(&f->mv);
}

const char *autoStateName(autoState_t s)
{
  switch (s) {
    case AUTO_IDLE: return "IDLE";
    case AUTO_ARMED: return "ARMED";
    case AUTO_TAKEOFF: return "TAKEOFF";
    case AUTO_HOVER: return "HOVER";
    case AUTO_LAND: return "LAND";
    case AUTO_KILLED: return "KILLED";
  }
  return "?";
}

const char *autoReasonName(autoReason_t r)
{
  static const char *const names[] = {
    [AUTO_RS_NONE] = "NONE",
    [AUTO_RS_ARM] = "ARM",
    [AUTO_RS_TAKEOFF] = "TAKEOFF",
    [AUTO_RS_HOLD] = "HOLD",
    [AUTO_RS_DISARM_CMD] = "DISARM_CMD",
    [AUTO_RS_ARM_TIMEOUT] = "ARM_TIMEOUT",
    [AUTO_RS_MOVED] = "MOVED",
    [AUTO_RS_LOCKED] = "LOCKED",
    [AUTO_RS_LAND_CMD] = "LAND_CMD",
    [AUTO_RS_HB_LOST] = "HB_LOST",
    [AUTO_RS_APP_INPUT] = "APP_INPUT",
    [AUTO_RS_APP_SILENT] = "APP_SILENT",
    [AUTO_RS_SESSION] = "SESSION",
    [AUTO_RS_AIR_TIME] = "AIR_TIME",
    [AUTO_RS_VBAT_LOW] = "VBAT_LOW",
    [AUTO_RS_VBAT_DIP] = "VBAT_DIP",
    [AUTO_RS_LOOP_GAP] = "LOOP_GAP",
    [AUTO_RS_VZ] = "VZ",
    [AUTO_RS_NOT_STABLE] = "NOT_STABLE",
    [AUTO_RS_SORTIE_DONE] = "SORTIE_DONE",
    [AUTO_RS_CAPTURE_FAIL] = "CAPTURE_FAIL",
    [AUTO_RS_TAKEOFF_TIMEOUT] = "TAKEOFF_TIMEOUT",
    [AUTO_RS_TOUCHDOWN] = "TOUCHDOWN",
    [AUTO_RS_TOUCHDOWN_TRIP] = "TOUCHDOWN_TRIP",
    [AUTO_RS_LAND_TIMEOUT] = "LAND_TIMEOUT",
    [AUTO_RS_KILL_CMD] = "KILL_CMD",
    [AUTO_RS_TILT] = "TILT",
    [AUTO_RS_RATE] = "RATE",
    [AUTO_RS_FREEFALL] = "FREEFALL",
    [AUTO_RS_ATT_ERR] = "ATT_ERR",
    [AUTO_RS_NAN] = "NAN",
    [AUTO_RS_VZ_CUT] = "VZ_CUT",
    [AUTO_RS_MOTOR_GUARD] = "MOTOR_GUARD",
    [AUTO_RS_MOTOR_STOP] = "MOTOR_STOP",
    [AUTO_RS_ESTOP_EXT] = "ESTOP_EXT",
    [AUTO_RS_LAND_BACKSTOP] = "LAND_BACKSTOP",
    [AUTO_RS_KILL_CLEARED] = "KILL_CLEARED",
    [AUTO_RS_HOVER_TIMEOUT] = "HOVER_TIMEOUT",
    [AUTO_RS_HOVER_NO_CAM] = "HOVER_NO_CAM",
    [AUTO_RS_BATT_LOW] = "BATT_LOW",
    [AUTO_RS_MOVE_NO_CAM] = "MOVE_NO_CAM",
    [AUTO_RS_MOVE_TILT] = "MOVE_TILT",
    [AUTO_RS_COLLISION] = "COLLISION",
    [AUTO_RS_PALM] = "PALM",
    [AUTO_RS_SINK] = "SINK",
    [AUTO_RS_RESUME] = "RESUME",
  };
  if ((unsigned)r < sizeof(names) / sizeof(names[0]) && names[r] != NULL) {
    return names[r];
  }
  return "?";
}

const char *autoEventName(autoEvent_t e)
{
  static const char *const names[] = {
    [AUTO_EV_NONE] = "NONE",
    [AUTO_EV_CAM_LOST] = "CAM_LOST",
    [AUTO_EV_CAM_BACK] = "CAM_BACK",
    [AUTO_EV_SINK] = "SINK",
    [AUTO_EV_RESUME] = "RESUME",
    [AUTO_EV_CONTACT] = "CONTACT",
    [AUTO_EV_VZ] = "VZ",
    [AUTO_EV_NOT_STABLE] = "NOT_STABLE",
    [AUTO_EV_TAKEOFF_SLOW] = "TAKEOFF_SLOW",
    [AUTO_EV_CAPTURE_FAIL] = "CAPTURE_FAIL",
    [AUTO_EV_MOVE_STOPPED] = "MOVE_STOPPED",
    [AUTO_EV_LOOP_GAP] = "LOOP_GAP",
    [AUTO_EV_RETRACK_UNDO] = "RETRACK_UNDO",
    [AUTO_EV_HOLD_NO_VIEW] = "HOLD_NO_VIEW",
    [AUTO_EV_VIEW_BACK] = "VIEW_BACK",
    [AUTO_EV_HOLD_LIMIT] = "HOLD_LIMIT",
  };
  if ((unsigned)e < sizeof(names) / sizeof(names[0]) && names[e] != NULL) {
    return names[e];
  }
  return "?";
}
