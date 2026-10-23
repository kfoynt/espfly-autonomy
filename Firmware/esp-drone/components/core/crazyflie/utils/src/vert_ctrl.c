/*
 * ESP-Drone Firmware
 *
 * vert_ctrl.c - Vertical control without a height sensor, and the point-mass
 * sim plant that feeds it in the host tests and in DRY/BENCH (pure; autonomy
 * stage A2, docs/autonomy-a2.md section 4)
 */
#include <math.h>
#include <string.h>

#include "vert_ctrl.h"

/* Placement of the code that runs on every stabilizer loop while autonomy is
 * active (the A2b bench measured ~340 us a loop from flash: the 16 KB
 * instruction cache is shared and thrashed by the camera's flow fit, so the
 * loop fell to ~630-700 Hz). IRAM on the drone, nothing on the host. */
#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define AUTO_HOT IRAM_ATTR
#else
#define AUTO_HOT
#endif

/* After the finiteness check every input is clamped into its physical range,
 * so no finite value (say 1e30 g) can overflow the estimate into inf/NaN. */
#define VZ_IN_VBAT_MIN      2.0f      /* V; also keeps every K / Vf finite */
#define VZ_IN_VBAT_MAX      5.0f
#define VZ_IN_ACC_MAX       16.0f     /* g, MPU-6050 full scale */
#define VZ_IN_GYRO_MAX      4000.0f   /* dps */
#define VZ_IN_THRUST_MAX    65535.0f

#define VZ_LP20_TAU_S       (1.0f / (2.0f * 3.14159265f * 20.0f))   /* 20 Hz first order */

#define VZ_LEARN_GAIN       0.3f      /* section 4.5 */
#define VZ_LEARN_STEP_MAX   0.05f     /* x kPrior per sortie */
#define VZ_LEARN_AIR_MIN_S  1.5f

#define VZ_SIM_JOLT_TAU_S   0.010f
#define VZ_SIM_JOLT_MAX_G   4.0f
#define VZ_SIM_SPIN_THRUST  1000.0f   /* accSpinG applies above this rotor thrust */
#define VZ_SIM_CAM_TAU_S    0.1f      /* correlation time of the camera velocity noise */

static float clampf(float x, float lo, float hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}

/* First-order low-pass, time constant tau. */
static float lowpass(float y, float x, float dt, float tau)
{
  return y + (x - y) * dt / (tau + dt);
}

static bool inRange(float x, float lo, float hi)
{
  return isfinite(x) && x >= lo && x <= hi;
}

static bool positive(float x)
{
  return isfinite(x) && x > 0.0f;
}

void vzDefaultConfig(vzConfig_t *c)
{
  memset(c, 0, sizeof(*c));
  c->hTarget = VZ_H_TARGET_DEFAULT;
  c->kPrior = VZ_K_DEFAULT;
  c->bAir = VZ_BAIR_DEFAULT_G;

  c->capKhat = 1.15f;
  c->capLo = 1.65f;     /* lead's default: hover is 1.28-1.57 x lift-off (median 1.41) in ground effect */
  c->tLoMaxRel = 0.90f; /* logged lift-off <= 0.84 x the flight's own hover (0.91 x today's prior: clipped 1.5 %) */
  c->tiltDivMin = 0.94f;

  c->spinS = 0.8f;
  c->spinThrustMax = 20000.0f;
  c->spinThrustRel = 0.45f;
  c->g0WindowS = 0.5f;
  c->spinStdMax = 0.040f;
  c->spinTiltMax = 8.0f;
  c->spinGyroMax = 15.0f;
  c->spinGyroHoldS = 0.020f;
  c->spinSagMax = 0.35f;

  c->rampStartRel = 0.50f;
  c->rampTopRel = 1.00f;
  c->rampRate = 13250.0f;
  c->rampMaxS = 2.5f;
  c->abortRate = 60000.0f;
  c->abortSlowRel = 0.60f;
  c->liftAcc = 0.025f;
  c->liftHoldS = 0.030f;
  c->liftCusumV = 0.02f;     /* the design's 0.04 m/s and 10 mg detected 28 % of the sim's lift-offs */
  c->liftCusumK = 0.007f;    /* ~1 s late (T_lo +40 %) at a -5 mg residual in-flight offset */
  c->liftTiltMax = 15.0f;
  c->liftGyroMax = 250.0f;
  c->liftLatencyS = 0.030f;

  c->vClimbMax = VZ_CLIMB_V_DEFAULT;
  c->aClimb = 1.0f;
  c->kH = 2.0f;         /* the design's 1.2: at hTarget 0.40 only 1 sortie in 3 finished its photos in 3.5 s */
  c->vDescMax = 0.30f;
  c->kpRel = 0.35f;
  c->kiRel = 3.0f;      /* the design's 1.5 let a drone 3 % weaker than kPrior time out in CLIMB */
  c->khatMinRel = 0.88f;
  c->khatMaxRel = 1.12f;
  c->khatClampS = 0.3f;
  c->khat0LoRel = 1.2f;
  c->climbFloorRel = 0.75f;
  c->holdBand = 0.05f;
  c->climbMaxS = VZ_CLIMB_BASE_S;   /* up to VZ_H_TIMING_REF; vzConfigSetHeight() lengthens it above */
  c->capHitClimbS = 0.7f;
  c->capHitHoldS = 0.5f;
  c->holdLoopS = 0.5f;
  c->kfrzBias = 0.005f;
  c->holdMaxS = VZ_HOLD_MAX_LIMIT_S - 1.0f;   /* 599 s: a backstop no battery reaches (119 under the 120 s air clock
                                                  until 2026-10-15) */
  c->holdNoCamS = VZ_HOLD_NO_CAM_DEFAULT_S;   /* 2 s behind the FSM's SINK (AUTO_CAM_LOST_MAX_MS, a static assert in
                                                 auto_fsm.h; 3.0 s of HOLD until 2026-10-15) */
  c->camTauS = 0.2f;
  c->steadyVMax = 0.15f;
  c->steadyBand = 0.06f;
  c->vHpTauS = 1.0f;
  c->vfTauS = 0.5f;
  c->tHoldTauS = 0.25f;
  c->hMin = -0.3f;
  c->hMax = 1.0f;        /* 0.6 m over VZ_H_TIMING_REF; vzConfigSetHeight() raises it above */
  c->vAbsMax = 1.0f;
  c->estRangeOn = true;  /* the EST_RANGE LAND as before (NVS vzest 1) ... */
  c->estGraceS = 0.0f;   /* ... on the condition's first step (NVS vzestms 0) */

  c->landStart = 0.95f;
  c->landRate = 0.10f;
  c->landFloor = 0.65f;
  c->landVref = -VZ_LAND_V_DEFAULT;
  c->tdThrust = 0.75f;
  c->tdForce = 1.3f;
  c->tdForceS = 0.2f;
  c->tdJoltG = 0.6f;
  c->landClimbRefRel = 1.00f;
  c->landTop = 0.98f;
  c->landFlatS = 1.5f;   /* up to VZ_H_TIMING_REF; vzConfigSetHeight() lengthens it above */
  c->landVnear = -0.20f;
  c->landSlowH = 0.20f;
  c->tdNearH = 0.15f;
  c->tdNearJoltG = 0.30f;
  c->stallV = 0.08f;
  c->stallS = 0.10f;
  c->stallRate = 1.00f;
  /* the fast LAND (docs/autonomy-a2b.md 17.4) */
  c->landVfast = -0.50f;
  c->landFastH = 0.60f;      /* the slow-down band 0.85 -> 0.60 m (2026-10-14 fix: 0.65 -> 0.50 with the brake in the */
  c->landFastBand = 0.25f;   /* cruise made the mean touchdown up to +0.057 m/s harder; docs/autonomy-a2b.md 17.4) */
  c->landFastMinH = 1.00f;
  c->landBrakeTop = 1.03f;
  c->landBrakeH = 0.40f;
  c->landBrakeMaxS = 0.5f;
}

bool vzConfigValid(const vzConfig_t *c)
{
  if (c == NULL) {
    return false;
  }
  /* Hard limits, docs/autonomy-a2.md section 4.6. */
  if (!inRange(c->hTarget, VZ_H_TARGET_MIN, VZ_H_TAKEOFF_MAX) ||   /* the take-off; 2.00 m by moves (vert_ctrl.h) */
      !inRange(c->kPrior, VZ_K_MIN, VZ_K_MAX) ||
      !inRange(c->bAir, -VZ_BAIR_MAX_G, VZ_BAIR_MAX_G) ||
      !inRange(c->capKhat, 1.0f, 1.25f) ||
      !inRange(c->capLo, 1.0f, 2.0f) ||
      !inRange(c->tLoMaxRel, 0.60f, 1.0f) ||
      !inRange(c->rampTopRel, 0.80f, 1.10f) ||
      !inRange(c->tiltDivMin, 0.90f, 1.0f) ||
      !positive(c->spinThrustMax) || c->spinThrustMax > 25000.0f ||
      !inRange(c->rampRate, 1000.0f, 30000.0f) ||
      !positive(c->rampMaxS) || c->rampMaxS > 3.0f ||
      !positive(c->climbMaxS) || c->climbMaxS > VZ_CLIMB_MAX_LIMIT_S ||
      !positive(c->holdMaxS) || c->holdMaxS > VZ_HOLD_MAX_LIMIT_S ||
      !positive(c->holdNoCamS) || c->holdNoCamS > VZ_HOLD_NO_CAM_MAX_S ||
      !inRange(c->estGraceS, 0.0f, VZ_EST_GRACE_MAX_S) ||
      !inRange(c->landStart, 0.80f, 1.0f) ||
      !inRange(c->landRate, 0.02f, 0.30f) ||
      !inRange(c->landFloor, 0.40f, c->landStart) ||
      !inRange(c->tdThrust, 0.50f, 0.90f) ||
      !inRange(c->tdForce, 1.10f, 2.0f) ||
      !inRange(c->landClimbRefRel, 0.80f, 1.00f) ||
      !inRange(c->abortSlowRel, 0.40f, 1.00f) ||
      !inRange(c->landTop, 0.80f, 1.00f) || !inRange(c->landFlatS, 0.0f, VZ_LAND_FLAT_LIMIT_S) ||
      !inRange(c->tdNearH, 0.0f, 0.30f) || !inRange(c->tdNearJoltG, 0.15f, c->tdJoltG) ||
      !inRange(c->stallRate, c->landRate, 2.0f) ||
      !inRange(c->landSlowH, 0.0f, 0.50f)) {
    return false;
  }
  if (!(isfinite(c->hMin) && c->hMin < 0.0f) || !(isfinite(c->landVref) && c->landVref < 0.0f) ||
      !inRange(c->landVnear, c->landVref, -0.05f)) {
    return false;
  }
  /* the fast LAND */
  if (!inRange(c->landVfast, -0.60f, c->landVref) || !inRange(c->landFastH, 0.30f, 1.00f) ||
      !inRange(c->landFastBand, 0.05f, 0.50f) || !inRange(c->landFastMinH, c->landFastH + c->landFastBand, 1.50f) ||
      !inRange(c->landBrakeTop, 1.00f, 1.05f) || !inRange(c->landBrakeH, c->landSlowH, c->landFastH) ||
      !inRange(c->landBrakeMaxS, 0.0f, 1.0f)) {
    return false;
  }
  if (!inRange(c->spinGyroHoldS, 0.0f, 0.2f)) {
    return false;
  }
  /* Every other field finite and > 0. */
  const float pos[] = {
    c->spinS, c->spinThrustRel, c->g0WindowS, c->spinStdMax, c->spinTiltMax, c->spinGyroMax, c->spinSagMax,
    c->rampStartRel, c->abortRate, c->liftAcc, c->liftHoldS, c->liftCusumV, c->liftCusumK, c->liftTiltMax,
    c->liftGyroMax, c->liftLatencyS, c->vClimbMax, c->aClimb, c->kH, c->vDescMax, c->kpRel, c->kiRel,
    c->khatMinRel, c->khatMaxRel, c->khatClampS, c->khat0LoRel, c->climbFloorRel, c->holdBand,
    c->capHitClimbS, c->capHitHoldS, c->holdLoopS, c->kfrzBias, c->camTauS, c->steadyVMax, c->steadyBand,
    c->vHpTauS, c->vfTauS, c->tHoldTauS, c->hMax, c->vAbsMax, c->tdForceS, c->tdJoltG, c->stallV, c->stallS,
  };
  for (size_t i = 0; i < sizeof(pos) / sizeof(pos[0]); i++) {
    if (!positive(pos[i])) {
      return false;
    }
  }
  /* Consistency (stricter than "> 0"; the defaults pass): fractions stay
   * fractions and the K clamp brackets the prior. */
  if (c->spinThrustRel > 1.0f || c->rampStartRel > 1.0f || c->climbFloorRel > 1.0f ||
      c->kfrzBias >= 0.1f || c->steadyBand >= 1.0f ||
      c->khatMinRel > 1.0f || c->khatMaxRel < 1.0f) {
    return false;
  }
  return true;
}

void vzConfigSetHeight(vzConfig_t *c, float hTarget)
{
  c->hTarget = hTarget;
  /* Written so that NaN changes nothing here (vzConfigValid() refuses it). */
  if (!(hTarget > VZ_H_TIMING_REF) || !(c->vClimbMax > 0.0f) || !(c->landVref < 0.0f)) {
    return;
  }
  const float above = hTarget - VZ_H_TIMING_REF;
  c->hMax += above;
  c->climbMaxS += above / c->vClimbMax;
  /* up to the knee as before (capped at VZ_LAND_FLAT_MAX_S), above it the extra height at |landVref| */
  c->landFlatS = fminf(VZ_LAND_FLAT_MAX_S, c->landFlatS + fminf(above, VZ_H_KNEE - VZ_H_TIMING_REF) / -c->landVref);
  if (hTarget > VZ_H_KNEE) {
    c->landFlatS += (hTarget - VZ_H_KNEE) / -c->landVref;
  }
}

/* ---- controller ---------------------------------------------------------- */

static void setPhase(vzState_t *st, vzPhase_t p, vzReason_t r)
{
  st->phase = p;
  st->reason = r;
  st->tPhase = 0.0f;
  st->capT = 0.0f;
  st->clampT = 0.0f;
  st->tdT = 0.0f;
  st->stallT = 0.0f;
}

static void enterCut(vzState_t *st, vzReason_t r)
{
  if (st->phase != VZ_CUT) {
    setPhase(st, VZ_CUT, r);
  }
}

/* A blind LAND (an in-air contact, vzInput_t.blind) flies its ceiling open loop (landTimes(), once per LAND:
 * flash). docs/autonomy-a2.md section 12.8.
 * - From HOLD its reference is the lowest of T_hold, T_hold from before the last vertical move or bob (tHoldQ) and
 *   the camera-measured quiet hover (tQc, the latest 1 s window of a quiet camera-closed hold, scaled to the battery
 *   now by Vf then / Vf now: a step sag of 0.4 V before a BATT_LOW left it 25 % low, 1.64 m/s): T_hold, a 0.25 s
 *   average of the applied thrust, also carries a motion's accelerations (a MOVE up, a MOVE down's braking, the
 *   bob's legs: up to ~4 % over the hover), and 0.98 x that held for the flat part climbed up to ~1 m on the sim
 *   (the verification of 1c2a43f); and since every LAND after a contact is blind (user decision (B) 2026-10-15) a
 *   LAND begins in a closed-loop hold that may be braking or pushing hard (a hold braking the descent a wrong camera
 *   had flown: T_hold 4 % over the hover, the flat part climbed 1.3 m in the sim; a hold climbing on a camera wrong by
 *   1 m/s: 15 % over, it rose to 8.5 m), where tQc is the hover (a steady window: thrust = weight + drag). Before
 *   2026-10-15 a blind LAND began in the contact's open loop, under capQ, which is below tQc.
 * - After a move down (hOff < 0) its flat part is the one of its reference height hTarget + hOff, not of the
 *   take-off height (the knee's 2.0 s at or below the knee, plus the drift allowance): the timeout before
 *   landTimes()' last rule minus that rule's 4 s. The ceiling alone touches down ~2.9 s after the flat part (no
 *   loop, no stall rule), and a take-off above the knee moved down to it or below kept the take-off height's
 *   3.3-4.0 s with the knee's 6 s timeout, so those LANDs ended LAND_TIMEOUT on the floor. A move up keeps the
 *   longer flat part (landTimes()). */
static __attribute__((noinline)) void blindLand(vzState_t *st)
{
  float t = st->tRef;
  if (st->tHoldQ > 0.0f) {
    t = fminf(t, st->tHoldQ);
  }
  if (st->tQc > 0.0f && st->vfQc > 0.0f) {
    t = fminf(t, st->tQc * st->vfQc / st->vf);   /* in thrust units at the battery now (the K model) */
  }
  if (st->hoverKnown && t < st->tRef) {
    st->tRef = t;
    st->kRef = st->tRef * st->vf;
    if (!(st->kRef > 1.0f)) {
      st->kRef = st->cfg.kPrior;   /* as startLand(): keep T_hov finite */
    }
  }
  if (st->hOff < 0.0f) {
    st->landFlatEff = fminf(st->landFlatEff, st->landMaxS - (VZ_LAND_MAX_S - VZ_LAND_FLAT_MAX_S));
  }
}

/* The flat part and the timeout of a LAND from above the knee, after a
 * vertical move up, after a long HOLD (the interactive hover), or blind. Once
 * per LAND, so flash (out of line: internal RAM is short, docs/autonomy-a2b.md
 * 14.7); so is startLand() itself since 2026-10-15. */
static __attribute__((noinline)) void landTimes(vzState_t *st, float hRef, bool high, float holdS)
{
  const vzConfig_t *c = &st->cfg;
  const float driftS = holdS > VZ_LAND_DRIFT_AFTER_S
                           ? fminf(VZ_LAND_DRIFT_MAX_M, VZ_LAND_DRIFT_MPS * holdS) / -c->landVnear
                           : 0.0f;
  const float upS = st->hOff > 0.0f ? st->hOff / -c->landVref : 0.0f;
  st->landFlatEff = (upS > 0.0f || driftS > 0.0f) ? c->landFlatS + upS + driftS : c->landFlatS;
  st->landMaxS = (high || driftS > 0.0f)
                     ? VZ_LAND_MAX_S + (fmaxf(hRef, VZ_H_KNEE) - VZ_H_KNEE) / -c->landVref + driftS
                     : VZ_LAND_MAX_S;
  if (st->blind) {
    blindLand(st);
    /* the flat part without the drift allowance: from there the floor ends it (blindFloor()) */
    st->landFlatBase = fmaxf(0.0f, st->landFlatEff - driftS);
  }
  /* at least as long after the flat part as at the knee (6.0 - 2.0 s): a low take-off moved up a lot has a
   * longer flat part than the same height's take-off (0.20 + 1.00 m: 4.8 s against 4.0 s) */
  st->landMaxS = fmaxf(st->landMaxS, st->landFlatEff + (VZ_LAND_MAX_S - VZ_LAND_FLAT_MAX_S));
}

/* A blind LAND's floor (2026-10-15, docs/autonomy-a2b.md 22.10; vzStep calls it on the steps a blind LAND has not been
 * descending for stallS, and after the contact latch; flash: only at the floor or on a support, after a bump, or in
 * the LAND's first ~0.2 s). A blind LAND flies landTop x the hover open loop for its flat part: the reference height's
 * time plus the drift allowance (landTimes(): up to +10 s after a long HOLD, for a drone truly higher than its
 * estimate). A drone that arrived sooner floats there in ground effect at near the hover thrust (in the sim ~8 cm up,
 * the hold engaged), and an estimate that drifted reads it high: 5-16 s of float after a 60-118 s HOLD (the
 * verification of e51f4b7).
 * - The stall rule (the ceiling at stallRate, the contact latch) acts with the estimate below tdNearH, as in any
 *   LAND, but only from VZ_BLIND_STALL_ARM_S into the LAND: landTop x the reference starts the descent slowly
 *   (-0.08 m/s after ~0.5 s), and an estimate drifted low read that start as a stall in the air (a resumed SINK's
 *   LAND in the sim: the ceiling cut 7 %, touchdown 1.27 m/s).
 * - Once the flat part without the allowance (landFlatBase) has run, not descending for VZ_BLIND_FLOOR_S (the stall's
 *   speed test, whatever the height estimate reads), or the contact latch, ends the flat part: the ceiling falls at
 *   landRate from there, the same fall that ends every blind LAND's flat part. In the air that is the right answer
 *   too (landTop x the reference does not bring the drone down), and a speed that reads a descent too slowly (a
 *   camera wrong for good) can at most take away the drift allowance. Not before the base: there it would take the
 *   wrong camera's error into the touchdown (test_vert_ctrl.c testContactLand: 1.26-1.70 m/s against <= 1.1). */
static __attribute__((noinline)) void blindFloor(vzState_t *st, float dt)
{
  const vzConfig_t *c = &st->cfg;
  if (st->stallT >= c->stallS && st->h < c->tdNearH && st->tPhase >= VZ_BLIND_STALL_ARM_S) {
    st->contact = true;
    st->ceiling = fminf(st->ceiling, fmaxf(c->landFloor * st->tRef, st->ceiling - c->stallRate * st->tRef * dt));
  }
  if (st->landSlow && st->tPhase >= st->landFlatBase && st->tPhase < st->landFlatEff &&
      (st->contact || st->stallT >= VZ_BLIND_FLOOR_S)) {
    st->landFlatEff = st->tPhase;
  }
}

/* The fast LAND (vert_ctrl.h landVfast .. landBrakeMaxS, docs/autonomy-a2b.md
 * 17.4): once per LAND, so flash (out of line; startLand() calls it only for a
 * slow LAND with the camera estimate from a reference height of landFastMinH or
 * more). Only the state machine's CMD_LAND for a reason on its allow list
 * (vzInput_t.landFastOk), never blind, never near the end of the battery (T_ref
 * over the MOVE refusal level, Vf under 3.15 V loaded): every other LAND,
 * BATT_LOW's and VBAT_DIP's included, is exactly as before. */
static __attribute__((noinline)) void landFastStart(vzState_t *st, vzReason_t r)
{
  const vzConfig_t *c = &st->cfg;
  st->landFast = st->landFastOk && !st->blind && r == VZ_R_CMD_LAND && st->tRef <= VZ_LAND_FAST_TREF_MAX &&
                 st->vf >= VZ_LAND_FAST_VMIN && c->landVfast < c->landVref;
  st->landFastFlown = false;
  st->brakeS = 0.0f;
}

/* LAND after lift-off, or the abort ramp before it. The reference thrust and
 * the starting ceiling are fixed here, from measured thrust only. Once per
 * LAND (every call site changes the phase to LAND): flash, out of line since
 * 2026-10-15 (575 B of IRAM: three 256-byte steps of the internal heap, whose
 * margin over the 19 KB floor was ~0.2 KB; a flash call at a LAND's start costs
 * tens of us once, docs/autonomy-a2b.md 22.8). */
static __attribute__((noinline)) void startLand(vzState_t *st, vzReason_t r)
{
  const vzConfig_t *c = &st->cfg;
  if (st->liftedOff) {
    /* hoverKnown: HOLD reached at the target, or a camera-measured quiet hover since. A HOLD a climb fault or a
     * contact in the climb started (its T_hold seeded from the climb, not measured) lands as from CLIMB. */
    if (st->hoverKnown) {
      st->tRef = st->tHold;
      st->kRef = st->tHold * st->vf;
    } else {
      /* LAND from CLIMB (open issue 1: from T_lo, ~0.75 x hover, the drone
       * fell at up to ~0.3 g): the thrust applied in CLIMB, averaged as
       * T_hold, within [T_lo, landClimbRefRel kPrior / Vf]. After a capped
       * step in the last tHoldTauS the average measures the ceiling, not the
       * hover (a lift-off hidden by a negative offset climbs at the cap): T_lo.
       * Not in a blind LAND: there the cap is the contact's own (its impulse
       * read as a descent drives the loop into it within the 3 ms before the
       * FSM lands), and T_lo (~0.75 x hover) would drop the drone with the
       * touchdown thrust condition met, so a second contact reads as
       * touchdown in the air (sim, contacts in CLIMB: DONE in the air in 21 of
       * 30 normal LANDs, 1 blind one on T_lo, none on the average;
       * test_vert_ctrl.c testContactLand). */
      st->tRef = st->climbFreeS < c->tHoldTauS && !st->blind
                     ? st->tLo
                     : fmaxf(st->tLo, fminf(st->tClimb, c->landClimbRefRel * c->kPrior / st->vf));
      st->kRef = c->kPrior;
    }
    st->tRef = clampf(st->tRef, 0.0f, VZ_THRUST_ABS_MAX);
    if (!(st->kRef > 1.0f)) {
      st->kRef = c->kPrior;   /* T_hold of 0 (thrust cut downstream): keep T_hov finite */
    }
    /* From HOLD with the external estimate in use, and not after one of the
     * controller's own guards (EST_RANGE, K_RANGE, HOLD_CAPPED: the estimate
     * or the model left its range), the loop flies a slow descent: the
     * ceiling starts at landTop (< the hover thrust just measured) and holds
     * it landFlatS. */
    /* Interactive hover: from a reference height above VZ_H_KNEE the slow LAND
     * also applies without the camera estimate (HOLD_NO_CAM, a LAND after the
     * camera dropped out): falling at once from up there would reach ~1 m/s.
     * Its ceiling is still measured thrust and time only. The flat part and the
     * timeout grow with the reference height; at or below the knee and the
     * take-off height both are exactly as before. */
    const float hRef = c->hTarget + st->hOff;
    const bool high = hRef > VZ_H_KNEE;
    /* A blind LAND (an in-air contact, vzInput_t.blind) flies the ceiling alone: from HOLD its flat part
     * applies with or without the camera estimate, as above the knee, and whatever its reason, the controller's
     * own guards (EST_RANGE, K_RANGE, HOLD_CAPPED, HOLD_NO_CAM) included (user decision (B) 2026-10-15: every LAND
     * after a contact is the slow blind one; falling at once from landStart, EST_RANGE touched down at 1.51 m/s
     * after a 20 s hover and 1.84 m/s after 100 s, docs/autonomy-a2b.md 22.10). */
    st->landSlow = st->hoverKnown && (st->blind || ((st->camNow || high) &&
                                                    (r == VZ_R_CMD_LAND || r == VZ_R_CMD_MISSING ||
                                                     r == VZ_R_HOLD_TIMEOUT || (high && r == VZ_R_HOLD_NO_CAM))));
    /* a long HOLD: the drift its estimate may hide (0 for every HOLD of the envelope before) */
    const float holdS = st->phase == VZ_HOLD ? st->tPhase : 0.0f;
    if (high || holdS > VZ_LAND_DRIFT_AFTER_S || st->hOff > 0.0f || st->blind) {
      landTimes(st, hRef, high, holdS);
    } else {
      /* at or below the knee (a take-off above it moved down): the take-off height's flat part, and as in landTimes()
       * at least 4 s after it (2026-10-15: the 6 s timeout fired inside a 6.67 s flat part, a 2.00 m take-off moved
       * down 1.5 m: a CUT up to 0.67 m in the air; from the 1.20 m take-off cap 4.0 s flat, 6 -> 8 s) */
      st->landFlatEff = c->landFlatS;
      st->landMaxS = fmaxf(VZ_LAND_MAX_S, st->landFlatEff + (VZ_LAND_MAX_S - VZ_LAND_FLAT_MAX_S));
    }
    st->ceiling = st->tRef * (st->landSlow ? c->landTop : c->landStart);
    st->landFast = false;
    if (st->landSlow && st->camNow && hRef >= c->landFastMinH) {
      landFastStart(st, r);
    }
  } else if (st->phase == VZ_RAMP && r != VZ_R_LIFTOFF_ATTITUDE &&
             (st->rampTop || st->thrust >= c->abortSlowRel * c->kPrior / st->vf)) {
    /* The ramp sat at its plateau, or passed a plausible lift-off thrust:
     * the drone may be up without a detected lift-off (a negative offset
     * hides a slow climb out of ground effect), so the abort first falls
     * like LAND's ceiling (A2b "always slow land": a phone stick in RAMP).
     * Not after a lift-off signature with a tilt or rate over the lift-off
     * limits: one leg up, so the thrust falls at abortRate at once. */
    st->abortRef = st->thrust;
  }
  st->landReason = r;
  setPhase(st, VZ_LAND, r);
}

/* ---- hold and report (vert_ctrl.h "Hold and report"; docs/autonomy-a2b.md 22) --------------------------------
 * Every function here runs once per event (a clamp, a cap, a re-seed, a climb ended early, a contact's edges, a
 * resume): flash, out of line (internal RAM is short; the per-loop tests that call them stay in vzStep). */

/* An event: vzOutput_t evt / evtN, at most one of the same reason per VZ_EVT_GAP_S (tAir). */
static __attribute__((noinline)) void vzEvent(vzState_t *st, vzReason_t r)
{
  if (st->evt == (uint8_t)r && st->tAir - st->evtLastT < VZ_EVT_GAP_S) {
    return;
  }
  st->evt = (uint8_t)r;
  st->evtN++;
  st->evtAny = true;
  st->evtLastT = st->tAir;
}

/* The hover thrust estimate less kfrzBias (a slight sink): the camera-measured quiet mean when there is one, else the
 * lower of the HOLD's quiet T_hold (or T_hold, or the CLIMB average) and the model's Khat / Vf. A contact's hush flies
 * it; a HOLD entered from a climb fault, a contact in the climb or a resume starts from it. */
static __attribute__((noinline)) float hoverSink(const vzState_t *st)
{
  const vzConfig_t *c = &st->cfg;
  float t;
  if (st->tQc > 0.0f) {
    t = st->tQc;
  } else {
    t = st->holdReached ? (st->tHoldQ > 0.0f ? st->tHoldQ : st->tHold) : st->tClimb;
    t = fminf(t, st->khat / st->vf);
  }
  return t * (1.0f - c->kfrzBias);
}

/* HOLD from CLIMB (reached, a climb fault, a contact) or from a resumed LAND: T_hold seeded, the closed loop on the
 * accelerometer for holdLoopS, the no-camera and thrust-bound clocks restarted. */
static __attribute__((noinline)) void holdEnter(vzState_t *st, vzReason_t r, float tHold)
{
  setPhase(st, VZ_HOLD, r);
  st->holdReached = true;
  st->hoverKnown = st->hoverKnown || r == VZ_R_HOLD_REACHED;
  st->tHold = tHold;
  st->loopEnd = st->cfg.holdLoopS;
  st->noCamT = 0.0f;
  st->hiT = 0.0f;
  st->estOut = false;
  st->estOutS = 0.0f;
  st->qSum = 0.0f;
  st->qT = 0.0f;
}

/* A climb fault (K_RANGE, CLIMB_CAPPED, CLIMB_TIMEOUT; EST_RANGE by its speed alone) with the camera estimate in use:
 * the climb ends in HOLD at the height reached, T_hold and tHoldQ from the hover estimate, and is reported. Without
 * it, or a height out of range, LAND as before (the estimate may read low: a hold could climb without bound). True
 * when it entered HOLD. */
static __attribute__((noinline)) bool climbFault(vzState_t *st, vzReason_t r, bool camUsed, float camVz)
{
  const vzConfig_t *c = &st->cfg;
  const bool hOut = st->h < c->hMin || st->h > c->hMax + st->hOff;
  if (!camUsed || (r == VZ_R_EST_RANGE && hOut)) {
    startLand(st, r);
    return false;
  }
  if (r == VZ_R_EST_RANGE) {
    st->v = clampf(camVz, -c->vAbsMax, c->vAbsMax);
    st->vHp = st->v;
  }
  const float t = hoverSink(st);
  holdEnter(st, r, t);
  st->tHoldQ = t;
  vzEvent(st, r);
  return true;
}

/* A contact's hush (vzInput_t.hush), its edges, and the open window after it. Rising: the speed from before the
 * contact kept (vPre), the hush's thrust fixed (the hover estimate less kfrzBias, braking an upward speed), and a
 * CLIMB ends in HOLD there (CONTACT_HOLD). Falling: v and vHp re-seeded from vPre moved toward the camera's speed by
 * at most VZ_HUSH_DV_MAX (the impulses in between were not integrated; a jolt's frames may read a speed that is not
 * there), and in HOLD VZ_HUSH_OPEN_S of open loop (postOn: the camera estimate unused, capQ the bound). The window's
 * clock is tPhase (no dt here: passing it cost vzStep ~280 B of IRAM). */
static __attribute__((noinline)) void hushEdge(vzState_t *st, bool on, bool camUsed, float camVz, vzCmd_t cmd)
{
  if (on == st->hushOn) {
    if (st->postOn) {   /* the open window after a hush: to its end, in HOLD */
      st->postOn = st->tPhase < st->postT && st->phase == VZ_HOLD;
    }
    return;
  }
  st->hushOn = on;
  if (on) {
    /* the speed from before the contact: the FSM's trip held |acc| AUTO_COLLIDE_MS (3 ms) before this step and those
     * steps were integrated; aLp20 x its tau (8 ms) is their integral to within a few % at 3-4 samples (2026-10-13
     * flight 1's pulses: +0.42..+0.85 m/s of climb that was not there) */
    st->vPre = st->v - VZ_G * VZ_LP20_TAU_S * st->aLp20;
    st->postOn = false;
    /* the velocity loop's P term at vref 0 on the speed before it, never above the hover estimate: a contact in the
     * climb brakes the climb's 0.2-0.4 m/s (at the hover estimate alone the drone coasted up ~0.3 m) */
    const float hov = hoverSink(st);
    st->hushT = hov * clampf(1.0f - st->cfg.kpRel * st->vPre, 0.80f, 1.0f);
    if (st->phase == VZ_CLIMB && cmd == VZ_CMD_TAKEOFF) {
      holdEnter(st, VZ_R_CONTACT_HOLD, hov);   /* T_hold and tHoldQ (a LAND's reference): the hover, not the brake */
      st->tHoldQ = hov;
      vzEvent(st, VZ_R_CONTACT_HOLD);
    }
  } else {
    const float dv = camUsed ? clampf(camVz - st->vPre, -VZ_HUSH_DV_MAX, VZ_HUSH_DV_MAX) : 0.0f;
    st->v = st->vPre + dv;
    st->vHp = st->v;
    st->postOn = st->phase == VZ_HOLD;
    st->postT = st->tPhase + VZ_HUSH_OPEN_S;
    if (st->phase == VZ_HOLD) {
      st->loopEnd = st->tPhase;
    }
  }
}

/* HOLD with the estimate out of range, or back in it (estOut): the height out of hMin..hMax + hOff while this loop
 * can climb (the camera estimate in use, or no capQ yet) is the runaway guard: LAND EST_RANGE (true) once that has
 * held estGraceS without a break (estOutS; 0: on its first step), never with estRangeOn off (2026-10-22, NVS vzest /
 * vzestms). Otherwise never a LAND: a speed beyond vAbsMax is re-seeded from the camera's (or clamped: the open loop
 * reads v only for its damping), the height is reported, an event on the step it left the range. */
static __attribute__((noinline)) bool estHold(vzState_t *st, bool camUsed, const vzInput_t *in)
{
  const vzConfig_t *c = &st->cfg;
  const float camVz = in->camVz;
  const bool hOut = st->h < c->hMin || st->h > c->hMax + st->hOff;
  const bool landCond = hOut && (camUsed || !(st->tQc > 0.0f));
  st->estOutS = landCond ? st->estOutS + in->dt : 0.0f;
  if (landCond && c->estRangeOn && st->estOutS >= c->estGraceS) {
    startLand(st, VZ_R_EST_RANGE);
    return true;
  }
  const bool vOut = fabsf(st->v) > c->vAbsMax;
  if (vOut) {
    st->v = camUsed ? clampf(camVz, -c->vAbsMax, c->vAbsMax) : clampf(st->v, -c->vAbsMax, c->vAbsMax);
    st->vHp = st->v;
  }
  if ((vOut || hOut) && !st->estOut) {
    vzEvent(st, VZ_R_EST_RANGE);
  }
  st->estOut = vOut || hOut;
  return false;
}

static float climbCeiling(vzState_t *st);

/* A contact's hush in HOLD: hushEdge()'s thrust under this step's ceiling, lower while the camera sees a rise over
 * the speed from before the contact (a push from below: braked as the velocity loop's P term would, at most
 * VZ_HUSH_DV_MAX of it; a sink it reads adds nothing). */
static __attribute__((noinline)) float hushThrust(vzState_t *st, bool camUsed, float camVz)
{
  st->ceiling = climbCeiling(st);
  float t = st->hushT;
  if (camUsed && camVz > st->vPre) {
    t *= clampf(1.0f - st->cfg.kpRel * fminf(camVz - st->vPre, VZ_HUSH_DV_MAX), 0.80f, 1.0f);
  }
  return fminf(t, st->ceiling);
}

/* HOLD at a bound (holdStep's gate: a cap, a Khat clamp, hiT counting, Khat at its upper clamp or across khatMinRel;
 * flash, only then). The upper thrust bound, leaky (vert_ctrl.h "Hold and report"): hiT counts while Khat is at its
 * upper clamp with the loop closed or the thrust at the model's ceiling, down at twice the rate otherwise; VZ_HI_LAND_S
 * of it: LAND K_RANGE or HOLD_CAPPED. A battery bound (58000, capLo x K_lo) for capHitHoldS: LAND HOLD_CAPPED, as before.
 * Khat at its floor in force for khatClampS (VZ_KHAT_HOLD_LO_REL on the camera, khatMinRel on the accelerometer): LAND
 * K_RANGE (the model cannot hold the drone down). The events: capT through capHitHoldS at the model's ceiling
 * (HOLD_CAPPED), clampT through khatClampS at the upper clamp (K_RANGE), Khat at or below khatMinRel (K_RANGE: a light
 * drone, the ground or a ceiling). */
static __attribute__((noinline)) void holdBound(vzState_t *st, bool kLow, float capPrev, float clampPrev, bool closed,
                                                bool atCap, float dt)
{
  const vzConfig_t *c = &st->cfg;
  st->hiT = ((closed && st->khat >= st->kHiTh) || (atCap && st->capModel)) ? st->hiT + dt
                                                                            : fmaxf(0.0f, st->hiT - 2.0f * dt);
  if (st->hiT >= VZ_HI_LAND_S) {
    startLand(st, atCap ? VZ_R_HOLD_CAPPED : VZ_R_K_RANGE);
    return;
  }
  if (st->capT >= c->capHitHoldS && !st->capModel) {
    startLand(st, VZ_R_HOLD_CAPPED);
    return;
  }
  if (st->clampT >= c->khatClampS && st->khat < c->kPrior) {
    startLand(st, VZ_R_K_RANGE);
    return;
  }
  if (kLow != st->kLowOn) {
    st->kLowOn = kLow;
    if (kLow) {
      vzEvent(st, VZ_R_K_RANGE);
    }
  }
  if (st->capT >= c->capHitHoldS && capPrev < c->capHitHoldS) {
    vzEvent(st, VZ_R_HOLD_CAPPED);
  }
  if (st->clampT >= c->khatClampS && clampPrev < c->khatClampS) {
    vzEvent(st, VZ_R_K_RANGE);
  }
}

/* A complete VZ_QCAM_S window of camera-closed quiet thrust (holdQuiet()): its mean, capQ, the open loop's bound. */
static __attribute__((noinline)) void qcamDone(vzState_t *st)
{
  st->tQc = st->qSum / st->qT;
  st->vfQc = st->vf;
  st->capQ = st->tQc * (1.0f - st->cfg.kfrzBias);
  st->capLim = st->capQ;
  st->hoverKnown = true;
  st->qSum = 0.0f;
  st->qT = 0.0f;
}

/* A SINK LAND (CMD_LAND from HOLD) resumed (vzInput_t.resume with VZ_CMD_TAKEOFF): HOLD again from the hover
 * estimate, the next VZ_QUIET_S not quiet; only after lift-off and HOLD, with the camera estimate in use, no contact
 * latch, no touchdown, and the LAND the state machine's own (CMD_LAND). A blind one too since 2026-10-15 (user decision
 * (B): every LAND after a contact is blind, a SINK's descent with it; the HOLD it resumes is the camera's own, seeded
 * from the measured hover, never from the descent): the state machine does not ask in a contact's window. A blind one
 * only in its flat part and while descending (not stallS without a descent, the stall rule's own time): at the floor
 * no stall rule touches it down first, as it does a camera LAND within ~0.5 s (the sim resumed blind SINKs floating
 * 3-8 cm over the floor into a HOVER there, the estimate reading 0.2-1.3 m; the camera back takes ~0.15 s to correct
 * the speed and the state machine asks 0.5 s after it; docs/autonomy-a2b.md 22.10). */
static __attribute__((noinline)) void landResume(vzState_t *st, bool camUsed, vzCmd_t cmd)
{
  if (!(st->phase == VZ_LAND && cmd == VZ_CMD_TAKEOFF && st->liftedOff && st->holdReached && camUsed && !st->contact && !st->touchdown &&
        st->landReason == VZ_R_CMD_LAND)) {
    return;
  }
  if (st->blind && !(st->landSlow && st->tPhase < st->landFlatEff && st->stallT < st->cfg.stallS)) {
    return;
  }
  const float t = hoverSink(st);
  holdEnter(st, VZ_R_RESUME, t);
  st->tHoldQ = t;
  st->vRefT = 0.0f;
  st->landFast = false;
  st->resumed = true;
  vzEvent(st, VZ_R_RESUME);
}

/* The end of SPIN (once a sortie, so flash, out of line: internal RAM is short; it carries the double math): the
 * g0 window's mean and spread, the sag test, then RAMP or the abort. */
static __attribute__((noinline)) void spinEnd(vzState_t *st)
{
  const vzConfig_t *c = &st->cfg;
  const double n = st->g0N > 0 ? (double)st->g0N : 1.0;
  const double mean = st->g0N > 0 ? st->g0Sum / n : 1.0;
  const double var = st->g0SumSq / n - mean * mean;
  const float sd = var > 0.0 ? (float)sqrt(var) : 0.0f;
  if (st->g0N == 0 || sd > c->spinStdMax) {
    startLand(st, VZ_R_SPIN_NOISY);
  } else if (st->vRest - st->vf > c->spinSagMax) {
    startLand(st, VZ_R_SPIN_SAG);
  } else {
    st->g0 = (float)mean;
    st->aLp20 = 0.0f;
    st->cusumV = 0.0f;
    st->cusumH = 0.0f;
    st->cusumT = 0.0f;
    st->liftT = 0.0f;
    st->rampStart = fmaxf(st->spinThrust, c->rampStartRel * c->kPrior / st->vf);
    setPhase(st, VZ_RAMP, VZ_R_SPIN_DONE);
  }
}

/* The lift-off (once a sortie: flash, out of line): T_lo, the Khat seed, the estimate seeded from the CUSUM, CLIMB. */
static __attribute__((noinline)) void liftoffSeed(vzState_t *st, float dt)
{
  const vzConfig_t *c = &st->cfg;
  /* T_lo: the thrust applied liftLatencyS before the detection
   * (hist[histHead] is the previous step's). */
  int back = (int)(c->liftLatencyS / dt + 0.5f);
  back = back < 1 ? 1 : (back > VZ_THRUST_HIST ? VZ_THRUST_HIST : back);
  st->tLo = fminf(st->hist[(st->histHead + VZ_THRUST_HIST - (back - 1)) % VZ_THRUST_HIST], VZ_THRUST_ABS_MAX);
  /* A late detection (the ramp kept going while the drone was
   * already up) must not raise the climb ceiling capLo * T_lo, Khat0
   * or the LAND reference: T_lo is bounded by the stored hover gain,
   * never by anything measured in this sortie. */
  st->tLo = fminf(st->tLo, c->tLoMaxRel * c->kPrior / st->vf);
  st->vLo = st->vf;
  st->khat = clampf(fmaxf(c->kPrior, c->khat0LoRel * st->tLo * st->vLo), c->khatMinRel * c->kPrior,
                    c->khatMaxRel * c->kPrior);
  /* The drone has been rising since the CUSUM left zero: seed the
   * estimate from it with the allowance added back over that run
   * (the CUSUM change-point estimate), or it starts low by
   * ~0.012 m/s, the unsafe direction. The run integrated
   * accZ - g0 - liftCusumK; the estimate integrates
   * accZ - g0 - bAir, so the difference over the run is added. */
  const float kA = VZ_G * (c->liftCusumK - c->bAir);
  st->v = fmaxf(0.0f, st->cusumV + kA * st->cusumT);
  st->h = fmaxf(0.0f, st->cusumH + 0.5f * kA * st->cusumT * st->cusumT);
  st->vHp = st->v;
  st->liftedOff = true;
  st->tClimb = st->tLo;
  st->climbFreeS = 0.0f;
  st->tAir = 0.0f;
  setPhase(st, VZ_CLIMB, VZ_R_LIFTOFF);
}

/* HOLD, every step after T_hold: the blind LAND's hover reference (blindLand()), T_hold away from vertical
 * moves and bobs; and capQ (vert_ctrl.h "Hold and report"), the mean applied thrust over windows of VZ_QCAM_S
 * while the camera estimate closes the loop, quiet, uncapped and past the first holdLoopS (camQuiet): a window
 * that breaks starts again, the last complete one stays. Out of line and integer-tested: inlined, it cost vzStep
 * 84 B of IRAM in register spills (IRAM had 73 B to its next 256-byte step). */
static AUTO_HOT __attribute__((noinline)) void holdQuiet(vzState_t *st, float vRef, bool camQuiet, float applied,
                                                         float dt)
{
  uint32_t b;
  memcpy(&b, &vRef, sizeof(b));
  if (b << 1) {   /* vRef != 0 (sanitized and clamped by the caller; -0 counts as 0) */
    st->vRefT = st->tPhase;
  }
  const bool quiet = st->tPhase - st->vRefT >= VZ_QUIET_S;
  if (quiet) {
    st->tHoldQ = st->tHold;
  }
  if (quiet && camQuiet) {
    st->qSum += applied * dt;
    st->qT += dt;
    if (st->qT >= VZ_QCAM_S) {
      qcamDone(st);   /* flash, once a second */
    }
  } else {
    st->qSum = 0.0f;
    st->qT = 0.0f;
  }
}

/* CLIMB law (also the first holdLoopS of HOLD): returns the command before
 * the tilt division; adapts Khat with anti-windup and runs the clamp timer. */
static AUTO_HOT float velocityLoop(vzState_t *st, float vref, float applied, float dt, float kLo)
{
  const vzConfig_t *c = &st->cfg;
  const float err = vref - st->v;
  const bool limited = st->lastAtCap || applied < 0.98f * st->thrust;
  if (!(limited && err > 0.0f)) {
    const float kHi = c->khatMaxRel * c->kPrior;
    const float k = st->khat + c->kiRel * c->kpRel * st->khat * err * dt;
    st->khat = clampf(k, kLo, kHi);
    st->clampT = (k < kLo || k > kHi) ? st->clampT + dt : 0.0f;
  }
  const float tHov = st->khat / st->vf;
  const float u = tHov + c->kpRel * tHov * err;
  return fmaxf(u, c->climbFloorRel * tHov);
}

/* CLIMB: capLo x T_lo in thrust units (the climb's runaway bound, as
 * before). HOLD: capLo x K_lo in K units, capLo x T_lo x max(V_lo, Vf) / Vf
 * (user decision 2026-10-13, late, docs/autonomy-a2b.md "End of battery"):
 * in thrust units it bound at 51.7-58.9k at the end of full-charge sorties
 * as the battery drained, so the 58000 ceiling would have gained nothing;
 * the logged end hovers need 1.21-1.35 x K_lo. Never tighter than before:
 * CLIMB, and HOLD while Vf >= V_lo, give capLo x T_lo (up to the float
 * rounding of x Vf / Vf, at most 1 ulp: both bounds share one division in K
 * units, for the IRAM budget). Scaling CLIMB the same way failed the capLo
 * 1.10 runaway tests (peak 1.82-1.98 m against 0.80). AUTO_HOT noinline: one
 * copy, in IRAM (vzStep is IRAM code; a static the compiler stopped inlining
 * would land in flash). */
static AUTO_HOT __attribute__((noinline)) float climbCeiling(vzState_t *st)
{
  const vzConfig_t *c = &st->cfg;
  const float vs = st->phase == VZ_HOLD ? fmaxf(st->vLo, st->vf) : st->vf;
  const float kLo = c->capLo * st->tLo * vs, kMod = c->capKhat * st->khat;
  const float top = fminf(fminf(kLo, kMod) / st->vf, VZ_THRUST_ABS_MAX);
  /* the model's ceiling (capKhat x Khat), not a battery bound (capLo x K_lo, 58000): at it HOLD reports and lands
   * only after VZ_HI_LAND_S (vert_ctrl.h "Hold and report") */
  st->capModel = kMod < kLo && top < VZ_THRUST_ABS_MAX;
  return top;
}

static bool estimateOutOfRange(const vzState_t *st)
{
  const vzConfig_t *c = &st->cfg;
  /* hOff: a vertical move keeps the margin over the reference (exactly hMax without one) */
  return st->h < c->hMin || st->h > c->hMax + st->hOff || fabsf(st->v) > c->vAbsMax;
}

/* CLIMB with the estimate out of range (estimateOutOfRange), or on the step after (estOut): the runaway guard's climb
 * fault (climbFault EST_RANGE: LAND, or HOLD from a speed glitch with the camera estimate in use) once the condition
 * has held estGraceS without a break (estOutS; 0: on its first step, as before), never with estRangeOn off (the climb
 * runs on toward its target; 2026-10-22, NVS vzest / vzestms). True when it acted (the step's climb is skipped, as
 * before). Flash. */
static __attribute__((noinline)) bool estClimb(vzState_t *st, bool camUsed, const vzInput_t *in)
{
  const vzConfig_t *c = &st->cfg;
  const bool out = estimateOutOfRange(st);
  st->estOutS = out ? st->estOutS + in->dt : 0.0f;
  st->estOut = out;
  if (!out || !c->estRangeOn || st->estOutS < c->estGraceS) {
    return false;
  }
  st->estOut = false;
  (void)climbFault(st, VZ_R_EST_RANGE, camUsed, in->camVz);
  return true;
}

void vzInit(vzState_t *st, const vzConfig_t *cfg)
{
  memset(st, 0, sizeof(*st));
  st->g0 = 1.0f;
  if (!vzConfigValid(cfg)) {
    vzDefaultConfig(&st->cfg);   /* never run on the bad values, even by mistake */
    st->phase = VZ_CUT;
    st->reason = VZ_R_BAD_CONFIG;
    return;
  }
  st->cfg = *cfg;
  st->landMaxS = VZ_LAND_MAX_S;
  st->landFlatEff = cfg->landFlatS;
  st->vRefT = -VZ_QUIET_S;   /* HOLD starts quiet: tHoldQ follows T_hold until the first vertical reference */
  st->evtLastT = -VZ_EVT_GAP_S;
  st->loopEnd = cfg->holdLoopS;
  st->lastBias = cfg->bAir;   /* the open loop's offset until the observer gives one */
  st->capLim = VZ_THRUST_ABS_MAX;   /* no open-loop bound before capQ */
  st->kLoCam = VZ_KHAT_HOLD_LO_REL * cfg->kPrior;
  st->kLoMin = cfg->khatMinRel * cfg->kPrior;
  st->kHiTh = VZ_HI_CLAMP_FRAC * cfg->khatMaxRel * cfg->kPrior;
  st->phase = VZ_IDLE;
  st->reason = VZ_R_NONE;
  st->landReason = VZ_R_NONE;
}

AUTO_HOT void vzStep(vzState_t *st, const vzInput_t *in, vzOutput_t *out)
{
  const vzConfig_t *c = &st->cfg;
  float thrust = 0.0f;
  float brakeTop = 0.0f;   /* the fast LAND's brake ceiling this step (0: none) */
  bool liftoffNow = false, atCap = false, holdSteady = false, tdThrustOk = false, holdEntered = false, camUsed = false;

  vzCmd_t cmd = in->cmd;
  if (cmd != VZ_CMD_TAKEOFF && cmd != VZ_CMD_LAND && cmd != VZ_CMD_CUT) {
    cmd = VZ_CMD_NONE;
  }
  const bool finite = isfinite(in->dt) && isfinite(in->accZ) && isfinite(in->gravZ) && isfinite(in->tiltDeg) &&
                      isfinite(in->gyroMaxDps) && isfinite(in->vbat) && isfinite(in->thrustApplied);
  if (!finite) {
    enterCut(st, VZ_R_NAN_INPUT);
  } else {
    const float dt = clampf(in->dt, VZ_DT_MIN_S, VZ_DT_MAX_S);
    const float accZ = clampf(in->accZ, -VZ_IN_ACC_MAX, VZ_IN_ACC_MAX);
    const float tiltDiv = fmaxf(clampf(in->gravZ, -1.0f, 1.0f), c->tiltDivMin);
    const float tilt = clampf(in->tiltDeg, 0.0f, 180.0f);
    const float gyro = clampf(in->gyroMaxDps, 0.0f, VZ_IN_GYRO_MAX);
    const float vbat = clampf(in->vbat, VZ_IN_VBAT_MIN, VZ_IN_VBAT_MAX);
    const float applied = clampf(in->thrustApplied, 0.0f, VZ_IN_THRUST_MAX);
    if (in->fromSim) {
      st->fromSimSeen = true;
    }
    /* interactive hover: 0 without a vertical move (then nothing below changes) */
    const float vRefHold = isfinite(in->vRef) ? clampf(in->vRef, -VZ_VREF_MAX, VZ_VREF_MAX) : 0.0f;
    st->hOff = isfinite(in->hOff) ? clampf(in->hOff, -VZ_HOFF_MAX, VZ_HOFF_MAX) : 0.0f;
    st->blind = in->blind;   /* LAND after an in-air contact: no estimate in the descent */
    st->landFastOk = in->landFastOk;   /* the state machine allows a fast LAND (read at the LAND's start) */

    /* Filters, clocks and the estimate. */
    if (!st->started) {
      st->vf = vbat;
      st->started = true;
    } else {
      st->vf = lowpass(st->vf, vbat, dt, c->vfTauS);
    }
    st->histHead = (uint8_t)((st->histHead + 1) % VZ_THRUST_HIST);
    st->hist[st->histHead] = applied;
    st->aLp20 = lowpass(st->aLp20, accZ - st->g0, dt, VZ_LP20_TAU_S);
    st->tPhase += dt;
    const bool flying = st->liftedOff && (st->phase == VZ_CLIMB || st->phase == VZ_HOLD || st->phase == VZ_LAND);
    /* The external estimate (section 4.10) is used only airborne, never in
     * a contact's open window (postOn), and only on a step where the
     * observer says healthy and both values are finite and plausible. A bad
     * value is never a CUT: it is just not used, and HOLD_NO_CAM or the
     * FSM's camera rule land. */
    camUsed = !st->postOn && flying && in->camHealthy && in->camVzValid && isfinite(in->camVz) &&
              isfinite(in->camBias) && fabsf(in->camVz) <= VZ_CAM_VZ_MAX && fabsf(in->camBias) <= VZ_CAM_BIAS_MAX_G;
    st->camNow = camUsed;
    /* A contact's hush (vzInput_t.hush; CLIMB or HOLD) and its open window (postOn: the camera estimate unused, above):
     * flash while they run (hushEdge(): a CLIMB ends in HOLD). */
    if (in->hush | st->hushOn | st->postOn) {
      hushEdge(st, in->hush && flying && (st->phase == VZ_CLIMB || st->phase == VZ_HOLD), camUsed, in->camVz, cmd);
    }
    if (flying) {
      st->tAir += dt;
      st->noCamT = camUsed ? 0.0f : st->noCamT + dt;
      /* The offset: the observer's while in use; without it its last one (the in-flight offset it measured a moment
       * ago; vert_ctrl.h "Hold and report"), the stored bAir (vzInit()) only before it was ever in use. */
      if (camUsed) {
        st->lastBias = in->camBias;
      }
      const float a = VZ_G * (accZ - st->g0 - st->lastBias);
      if (!st->hushOn) {   /* a contact's impulses are not integrated */
        st->v += a * dt;
        st->h += st->v * dt;
        st->vHp += a * dt - st->vHp * dt / c->vHpTauS;
        if (camUsed) {
          /* Complementary filter: the accelerometer for the fast part, the
           * observer's velocity for the slow part (the offset drift). */
          const float k = dt / (c->camTauS + dt);
          st->v += (in->camVz - st->v) * k;
          st->vHp += (in->camVz - st->vHp) * k;
        }
      }
    }

    if (cmd == VZ_CMD_CUT) {
      enterCut(st, VZ_R_CMD_CUT);
    }

    /* Phases in their forward order, so a phase entered in this step
     * computes this step's thrust (HOLD entry keeps CLIMB's). */
    if (st->phase == VZ_IDLE) {
      if (cmd == VZ_CMD_TAKEOFF) {
        st->vRest = st->vf;
        st->spinThrust = fminf(c->spinThrustMax, c->spinThrustRel * c->kPrior / st->vf);
        st->g0Sum = 0.0;
        st->g0SumSq = 0.0;
        st->g0N = 0;
        st->spinGyroS = 0.0f;
        setPhase(st, VZ_SPIN, VZ_R_TAKEOFF);
      } else if (cmd == VZ_CMD_LAND) {
        st->landReason = VZ_R_CMD_LAND;
        setPhase(st, VZ_DONE, VZ_R_CMD_LAND);
      }
    }

    if (st->phase == VZ_SPIN) {
      if (cmd != VZ_CMD_TAKEOFF) {
        startLand(st, cmd == VZ_CMD_LAND ? VZ_R_CMD_LAND : VZ_R_CMD_MISSING);
      } else {
        st->ceiling = st->spinThrust;
        thrust = st->spinThrust;
        if (st->tPhase > c->spinS - c->g0WindowS) {
          if (tilt > c->spinTiltMax) {
            startLand(st, VZ_R_SPIN_TILT);
          } else if (gyro > c->spinGyroMax) {
            /* a rotation held spinGyroHoldS, not one vibration spike (docs/autonomy-a2b.md 19.7); the sample stays
             * out of g0 */
            st->spinGyroS += dt;
            if (st->spinGyroS >= c->spinGyroHoldS) {
              startLand(st, VZ_R_SPIN_GYRO);
            }
          } else {
            st->spinGyroS = 0.0f;
            st->g0Sum += (double)accZ;
            st->g0SumSq += (double)accZ * (double)accZ;
            st->g0N++;
          }
        }
        if (st->phase == VZ_SPIN && st->tPhase >= c->spinS) {
          spinEnd(st);
        }
      }
    }

    if (st->phase == VZ_RAMP) {
      if (cmd != VZ_CMD_TAKEOFF) {
        startLand(st, cmd == VZ_CMD_LAND ? VZ_R_CMD_LAND : VZ_R_CMD_MISSING);
      } else {
        st->ceiling = fminf(c->capKhat * c->kPrior / st->vf, VZ_THRUST_ABS_MAX);
        /* The ramp stops rising at the hover thrust of the stored gain
         * (rampTopRel kPrior / Vf): a drone that left the ground
         * undetected then floats instead of being pushed out of ground
         * effect, and RAMP ends at rampMaxS (NO_LIFTOFF_TIME). */
        const float plateau = c->rampTopRel * c->kPrior / st->vf;
        const float ramp = st->rampStart + c->rampRate * st->tPhase;
        st->rampTop = st->rampTop || ramp >= plateau;
        const float u = fminf(ramp, plateau) / tiltDiv;
        /* Lift-off: the CUSUM's allowance keeps zero-mean noise from climbing
         * it. Neither test subtracts bAir: on the ground the in-flight
         * offset is absent (the accelerometer reads 1 g whatever the thrust),
         * so a stored negative bAir would turn the ground drift positive and
         * fire before the drone moves. A negative in-flight offset still
         * delays the detection once airborne: the plateau above, the T_lo
         * bound at the detection and the FSM's air clock (from the first
         * RAMP step) cover that. */
        const float aUp = accZ - st->g0;
        st->cusumV = fmaxf(0.0f, st->cusumV + VZ_G * (aUp - c->liftCusumK) * dt);
        st->cusumH = st->cusumV > 0.0f ? st->cusumH + st->cusumV * dt : 0.0f;
        st->cusumT = st->cusumV > 0.0f ? st->cusumT + dt : 0.0f;
        st->liftT = st->aLp20 > c->liftAcc ? st->liftT + dt : 0.0f;
        if (st->liftT >= c->liftHoldS || st->cusumV >= c->liftCusumV) {
          if (tilt > c->liftTiltMax || gyro > c->liftGyroMax) {
            startLand(st, VZ_R_LIFTOFF_ATTITUDE);   /* one leg up looks like a lift-off */
          } else {
            liftoffSeed(st, dt);
            liftoffNow = true;
          }
        } else if (u >= st->ceiling) {
          startLand(st, VZ_R_NO_LIFTOFF_CEIL);
        } else if (st->tPhase >= c->rampMaxS) {
          startLand(st, VZ_R_NO_LIFTOFF_TIME);
        } else {
          thrust = u;
        }
      }
    }

    if (st->phase == VZ_CLIMB) {
      if (cmd != VZ_CMD_TAKEOFF) {
        startLand(st, cmd == VZ_CMD_LAND ? VZ_R_CMD_LAND : VZ_R_CMD_MISSING);
      } else if ((st->estOut || estimateOutOfRange(st)) && estClimb(st, camUsed, in)) {
        /* LAND, or (a speed glitch with the camera estimate in use) HOLD from this step: its block below flies it
         * (estClimb(), flash: at once by default, after the grace of vzestms, never with vzest 0) */
      } else {
        const float vref = fminf(fminf(c->vClimbMax, c->aClimb * st->tPhase),
                                 fmaxf(-c->vDescMax, c->kH * (c->hTarget - st->h)));
        st->tClimb = lowpass(st->tClimb, applied, dt, c->tHoldTauS);
        const float u = velocityLoop(st, vref, applied, dt, st->kLoMin) / tiltDiv;
        st->ceiling = climbCeiling(st);
        atCap = u > st->ceiling;
        thrust = atCap ? st->ceiling : u;
        st->capT = atCap ? st->capT + dt : 0.0f;
        st->climbFreeS = atCap ? 0.0f : st->climbFreeS + dt;
        /* The climb faults: HOLD at the height reached with the camera estimate in use (this step's thrust is
         * CLIMB's), else LAND (climbFault()). */
        if (st->clampT >= c->khatClampS) {
          holdEntered = climbFault(st, VZ_R_K_RANGE, camUsed, in->camVz);
        } else if (st->capT >= c->capHitClimbS) {
          holdEntered = climbFault(st, VZ_R_CLIMB_CAPPED, camUsed, in->camVz);
        } else if (st->h >= c->hTarget - c->holdBand) {
          holdEnter(st, VZ_R_HOLD_REACHED, applied);
          holdEntered = true;
        } else if (st->tPhase >= c->climbMaxS) {
          holdEntered = climbFault(st, VZ_R_CLIMB_TIMEOUT, camUsed, in->camVz);   /* a slow climb: the estimate reads low */
        }
      }
    }

    if (st->phase == VZ_HOLD && !holdEntered) {
      if (cmd != VZ_CMD_TAKEOFF) {
        startLand(st, cmd == VZ_CMD_LAND ? VZ_R_CMD_LAND : VZ_R_CMD_MISSING);
      } else if ((st->estOut || estimateOutOfRange(st)) && estHold(st, camUsed, in)) {
        /* LAND EST_RANGE (estHold(), flash: the height guard while this loop can climb; else the speed re-seeded) */
      } else if (st->tPhase >= c->holdMaxS) {
        startLand(st, VZ_R_HOLD_TIMEOUT);
      } else if (!st->fromSimSeen && st->noCamT >= c->holdNoCamS) {
        startLand(st, VZ_R_HOLD_NO_CAM);   /* backstop: the FSM's SINK lands 2 s earlier */
      } else if (st->hushOn) {
        thrust = hushThrust(st, camUsed, in->camVz);   /* a contact's hush (flash): nothing adapts, nothing times */
      } else {
        /* The first holdLoopS (not after a hush: its open window): the CLIMB law at vref 0. Then, while the
         * external estimate measures the velocity, the same loop stays
         * closed on it (Khat adapts, Kfrz follows, Khat's floor VZ_KHAT_HOLD_LO_REL); without it the gain is
         * frozen and the thrust is open loop with damping from the leaky
         * velocity, as an offset drifting the estimate must not drive the
         * integrator, at most capQ once there is one. Khat on the first step at holdLoopS (the drone at the
         * target height, out of ground effect) is the learning sample; a
         * camera hold keeps adapting Khat after it for control only. */
        const bool closed = st->tPhase < st->loopEnd || camUsed;
        const float clampPrev = st->clampT, capPrev = st->capT;
        float uPre, tHov;
        if (closed) {
          /* Khat's floor: VZ_KHAT_HOLD_LO_REL on the camera, else khatMinRel (never above Khat: no jump up) */
          uPre = velocityLoop(st, vRefHold, applied, dt, camUsed ? st->kLoCam : fminf(st->kLoMin, st->khat));
          tHov = st->khat / st->vf;
          if (st->tPhase >= st->loopEnd) {
            if (!st->kFrozen) {
              st->kLearn = st->khat;
            }
            st->kfrz = st->khat * (1.0f - c->kfrzBias);
            st->kFrozen = true;
          }
        } else {
          if (!st->kFrozen) {
            st->kLearn = st->khat;
            st->kfrz = st->khat * (1.0f - c->kfrzBias);   /* a slight sink */
            st->kFrozen = true;
          }
          st->clampT = 0.0f;   /* Khat does not move in open loop */
          tHov = st->kfrz / st->vf;
          uPre = fmaxf(0.0f, tHov - c->kpRel * tHov * st->vHp);
        }
        const float u = uPre / tiltDiv;
        st->ceiling = climbCeiling(st);
        atCap = u > st->ceiling;
        thrust = atCap ? st->ceiling : u;
        if (!closed) {
          thrust = fminf(thrust, st->capLim);   /* open loop: under the measured hover (capQ), so it cannot climb */
        }
        st->capT = atCap ? st->capT + dt : 0.0f;
        st->tHold = lowpass(st->tHold, applied, dt, c->tHoldTauS);
        holdQuiet(st, vRefHold, camUsed && !atCap && st->tPhase >= st->loopEnd, applied, dt);
        if (st->kFrozen) {
          holdSteady = fabsf(st->vHp) < c->steadyVMax && fabsf(uPre - tHov) <= c->steadyBand * tHov && !atCap;
        }
        /* At a bound (a cap, a Khat clamp, the upper thrust bound's count) or Khat across khatMinRel: holdBound()
         * (flash, only then) counts hiT, lands and reports. */
        const bool kLow = st->khat <= st->kLoMin * (st->kLowOn ? 1.01f : 1.0f);   /* 1 % hysteresis */
        if (st->capT > 0.0f || st->clampT > 0.0f || st->hiT > 0.0f || st->khat >= st->kHiTh || kLow != st->kLowOn) {
          holdBound(st, kLow, capPrev, clampPrev, closed, atCap, dt);
        }
        if (st->phase != VZ_HOLD) {
          holdSteady = false;
        }
      }
    }

    if (st->phase == VZ_LAND) {
      atCap = false;
      if (!st->liftedOff) {
        /* Abort before lift-off: ramp down from the last command. From the
         * ramp plateau or a RAMP thrust >= abortSlowRel kPrior / Vf (the
         * drone may be up) it first follows LAND's ceiling with T_ref = that
         * thrust, down to landFloor x it, then falls at abortRate; otherwise
         * (and after LIFTOFF_ATTITUDE, abortRef 0) at abortRate at once. */
        const float slow = st->abortRef * (c->landStart - c->landRate * st->tPhase);
        thrust = st->abortRef > 0.0f && slow > c->landFloor * st->abortRef
                     ? fminf(st->thrust, slow)
                     : fmaxf(0.0f, st->thrust - c->abortRate * dt);
        st->ceiling = thrust;
        if (thrust <= 0.0f) {
          setPhase(st, VZ_DONE, VZ_R_ABORTED);
        }
      } else {
        /* The ceiling depends on measured thrust and time only; the loop can
         * only take the thrust below it. */
        const float tHov = st->kRef / st->vf;
        /* landSlow: the ceiling holds landTop (<= the hover thrust just
         * measured) for landFlatS, so the loop can fly the slow descent. */
        const float top = st->landSlow ? c->landTop : c->landStart;
        const float flat = st->landSlow ? st->landFlatEff : 0.0f;
        const float ramp = st->tRef * fmaxf(c->landFloor, top - c->landRate * fmaxf(0.0f, st->tPhase - flat));
        st->ceiling = fminf(st->ceiling, ramp);
        /* Stalled: not descending although the loop asks for it (on the
         * ground, or floating in ground effect): the ceiling falls at
         * stallRate. It only ever lowers the thrust: in the air the drone then
         * descends and the stall ends. Only a stall near the surface counts as
         * contact, so a float higher up keeps the camera hold. A blind LAND has its own floor rules (blindFloor(),
         * flash, below: its estimate may be drifted or corrupted, and its open loop at landTop x the hover is slow
         * to start descending); stallT counts there too (and refuses a blind SINK's resume). */
        st->stallT = (st->tPhase >= VZ_STALL_ARM_S && st->v > -c->stallV) ? st->stallT + dt : 0.0f;
        if (st->stallT >= c->stallS && !st->blind) {
          st->contact = st->contact || st->h < c->tdNearH;
          st->ceiling = fminf(st->ceiling, fmaxf(c->landFloor * st->tRef, st->ceiling - c->stallRate * st->tRef * dt));
        }
        if (st->blind && (st->stallT >= c->stallS || st->contact)) {
          blindFloor(st, dt);   /* the blind LAND's floor (flash): not descending, or the contact latch (here: +15 B of
                                   IRAM; after the bump's latch below it spilled registers, +244 B) */
        }
        /* vref: landVref down to landSlowH, then linear to landVnear at h 0. */
        const float hs = c->landSlowH > 0.0f ? clampf(st->h / c->landSlowH, 0.0f, 1.0f) : 1.0f;
        float vref = c->landVnear + (c->landVref - c->landVnear) * hs;
        brakeTop = st->ceiling;
        if (st->landFast) {
          /* The fast LAND: landVfast above landFastH + landFastBand, on steps with the camera estimate (a
           * dropout reverts to landVref at once). The brake takes back only the speed the fast reference added:
           * once that was flown (its blend at least halfway), in the slow-down band (the estimate between
           * landBrakeH and landFastH + landFastBand; the cruise above keeps the slow LAND's ceiling, so the
           * budget is not spent holding landVfast on a low-drag drone), landBrakeMaxS in all, in the flat part
           * (no camera needed: 0.5 s of the propagated velocity). */
          const float blend = clampf((st->h - c->landFastH) / c->landFastBand, 0.0f, 1.0f);
          if (st->camNow) {
            vref += (c->landVfast - c->landVref) * blend;
            st->landFastFlown |= blend >= 0.5f;
          }
          if (st->landFastFlown && blend < 1.0f && st->h > c->landBrakeH && st->brakeS < c->landBrakeMaxS &&
              st->tPhase < st->landFlatEff) {
            brakeTop = fmaxf(brakeTop, st->tRef * c->landBrakeTop);
          }
        }
        const float u = tHov * (1.0f + c->kpRel * (vref - st->v)) / tiltDiv;
        /* blind: the ceiling alone (the loop reads the estimate the contact corrupted) */
        thrust = st->blind ? st->ceiling : fminf(brakeTop, fmaxf(c->landFloor * st->tRef, u));
        if (thrust > st->ceiling) {
          st->brakeS += dt;
        }
        atCap = u > brakeTop;
        tdThrustOk = thrust <= c->tdThrust * tHov;
        /* Touchdown (critique B4): low thrust AND more specific force than
         * that thrust can make in the air, or a jolt with low thrust, or a
         * jolt at any thrust below tdNearH (estimated; not in a blind LAND, whose estimate may read the ground
         * while the drone is still up, so a second contact would cut it in the air). */
        const float fMeas = st->aLp20 + 1.0f;
        const float fPred = thrust * st->vf / st->kRef;
        st->tdT = (tdThrustOk && fMeas > c->tdForce * fPred) ? st->tdT + dt : 0.0f;
        const bool bump = st->aLp20 > c->tdNearJoltG;
        st->contact = st->contact || bump;
        if ((tdThrustOk && st->aLp20 > c->tdJoltG) || (bump && st->h < c->tdNearH && !st->blind)) {
          st->touchdown = true;
          setPhase(st, VZ_DONE, VZ_R_TOUCHDOWN_JOLT);
        } else if (st->tdT >= c->tdForceS) {
          st->touchdown = true;
          setPhase(st, VZ_DONE, VZ_R_TOUCHDOWN_FORCE);
        } else if (st->tPhase >= st->landMaxS) {
          enterCut(st, VZ_R_LAND_TIMEOUT);
        }
      }
    }
  }

  /* A SINK LAND resumed (vzInput_t.resume; the FSM asks with VZ_CMD_TAKEOFF): HOLD from the next step (landResume(),
   * flash; this step flew the LAND's thrust). */
  if (in->resume) {
    landResume(st, st->camNow, in->cmd);
  }
  if (st->phase == VZ_IDLE || st->phase == VZ_DONE || st->phase == VZ_CUT) {
    thrust = 0.0f;
    st->ceiling = 0.0f;
    atCap = false;
    tdThrustOk = st->phase == VZ_DONE;
  }
  /* Final clamp of every phase: 0 <= thrust <= ceiling <= VZ_THRUST_ABS_MAX. */
  if (!isfinite(thrust) || !isfinite(st->ceiling)) {
    enterCut(st, VZ_R_NAN_INPUT);
    thrust = 0.0f;
    st->ceiling = 0.0f;
  }
  st->ceiling = clampf(st->ceiling, 0.0f, VZ_THRUST_ABS_MAX);
  /* the fast LAND's brake may exceed the (monotone) LAND ceiling; brakeTop is 0 in every other step */
  thrust = clampf(thrust, 0.0f, fminf(fmaxf(st->ceiling, brakeTop), VZ_THRUST_ABS_MAX));
  st->thrust = thrust;
  st->lastAtCap = atCap;

  out->thrust = thrust;
  out->phase = st->phase;
  out->reason = st->reason;
  out->landReason = st->landReason;
  out->h = st->h;
  out->v = st->v;
  out->vHp = st->vHp;
  out->aRel = st->aLp20;
  out->g0 = st->g0;
  out->khat = st->khat;
  out->kfrz = st->kfrz;
  out->tLo = st->tLo;
  out->tHold = st->holdReached ? st->tHold : 0.0f;
  out->tRef = st->tRef;
  out->ceiling = st->ceiling;
  out->tAir = st->tAir;
  out->tPhase = st->tPhase;
  out->liftedOff = st->liftedOff;
  out->liftoffNow = liftoffNow;
  out->holdReached = st->holdReached;
  out->holdSteady = holdSteady;
  out->atCap = atCap;
  out->tdThrustOk = tdThrustOk;
  out->touchdown = st->touchdown;
  out->camHealthy = camUsed && st->phase != VZ_CUT && st->phase != VZ_DONE;
  out->camOpen = st->postOn & in->camHealthy;   /* the window, not a loss (the FSM's camera rule) */
  out->contact = st->contact;
  out->landMaxS = st->landMaxS;
  out->evt = st->evt;
  out->evtN = st->evtN;
  out->capQ = st->capQ;
  out->noCamS = st->noCamT;
  out->hiS = st->hiT;
}

/* The starts of LAND after which the learning sample is trusted (see
 * vert_ctrl.h): an allow list, so a reason appended later is refused. */
static bool learnableLand(vzReason_t r)
{
  return r == VZ_R_CMD_LAND || r == VZ_R_HOLD_NO_CAM;
}

bool vzLearnK(const vzState_t *st, float *kPrior)
{
  if (st == NULL || kPrior == NULL || !isfinite(*kPrior)) {
    return false;
  }
  if (st->phase != VZ_DONE || !st->touchdown || !st->kFrozen || st->tAir < VZ_LEARN_AIR_MIN_S ||
      st->fromSimSeen || !positive(st->kLearn) || !learnableLand(st->landReason) || st->evtAny || st->resumed) {
    return false;
  }
  const float k = *kPrior;
  const float lim = VZ_LEARN_STEP_MAX * k;
  const float step = clampf(VZ_LEARN_GAIN * (st->kLearn - k), -lim, lim);
  *kPrior = clampf(k + step, VZ_K_MIN, VZ_K_MAX);
  return true;
}

__attribute__((noinline)) bool vzShiftHeight(vzState_t *st, float d)
{
  if (st == NULL || st->phase != VZ_HOLD || !isfinite(d) || !(fabsf(d) <= VZ_SHIFT_MAX_M)) {
    return false;
  }
  st->h += d;
  return true;
}

bool vzRampAtTop(const vzState_t *st)
{
  return st != NULL && st->rampTop;
}

/* ---- sim plant ----------------------------------------------------------- */

void vzSimDefaultConfig(vzSimConfig_t *c)
{
  memset(c, 0, sizeof(*c));
  c->kTrue = VZ_K_DEFAULT;
  c->vRest = 3.90f;
  c->vSag = 9e-6f;
  c->geGain = 0.33f;
  c->geHeight = 0.03f;
  c->rotorTauS = 0.03f;
  c->damp = 0.07f;
  c->accNoiseG = 0.03f;
  c->joltPerMs = 0.5f;
  c->camWakeS = -1.0f;
  c->camConvS = 0.3f;
  c->camVzNoise = 0.02f;
  c->camLossS = -1.0f;
}

void vzSimInit(vzSimState_t *s, const vzSimConfig_t *c, uint32_t seed)
{
  memset(s, 0, sizeof(*s));
  s->vbat = c->vRest;
  s->accZ = 1.0f + c->accOffsetG;
  s->onGround = true;
  s->liftT = -1.0f;
  s->cmdT = -1.0f;
  s->rng = seed != 0 ? seed : 0x9E3779B9u;
}

static uint32_t xorshift32(uint32_t *x)
{
  uint32_t v = *x;
  v ^= v << 13;
  v ^= v >> 17;
  v ^= v << 5;
  *x = v;
  return v;
}

/* Unit Gaussian as a sum of 12 uniforms: no libm, so a seed gives the same
 * noise on every machine. */
static float gauss(uint32_t *x)
{
  float s = 0.0f;
  for (int i = 0; i < 12; i++) {
    s += (float)(xorshift32(x) >> 8) * (1.0f / 16777216.0f);
  }
  return s - 6.0f;
}

AUTO_HOT void vzSimStep(vzSimState_t *s, const vzSimConfig_t *c, float thrustCmd, float dt)
{
  if (!(dt > 0.0f) || !isfinite(dt)) {
    return;
  }
  const float cmd = isfinite(thrustCmd) ? clampf(thrustCmd, 0.0f, VZ_IN_THRUST_MAX) : 0.0f;
  s->tr += (cmd - s->tr) * fminf(1.0f, dt / c->rotorTauS);
  s->vbat = c->vRest - c->vSag * s->tr;
  const float ge = 1.0f + c->geGain * expf(-fmaxf(s->h, 0.0f) / c->geHeight);
  const float f = s->tr * s->vbat * ge / c->kTrue - c->damp * s->v;
  if (s->onGround && f > 1.0f) {
    s->onGround = false;
    if (s->liftT < 0.0f) {
      s->liftT = s->t;
    }
  }
  if (!s->onGround) {
    s->v += VZ_G * (f - 1.0f) * dt;
    s->h += s->v * dt;
    if (s->h <= 0.0f) {
      if (-s->v > s->impactV) {
        s->impactV = -s->v;
      }
      s->jolt = fminf(VZ_SIM_JOLT_MAX_G, c->joltPerMs * fabsf(s->v));
      s->h = 0.0f;
      s->v = 0.0f;
      s->onGround = true;
    }
  }
  if (s->h > s->hMax) {
    s->hMax = s->h;
  }
  s->accZ = (s->onGround ? 1.0f : f) + c->accOffsetG + (s->tr > VZ_SIM_SPIN_THRUST ? c->accSpinG : 0.0f) +
            (s->onGround ? 0.0f : c->accAirG) + s->jolt + c->accNoiseG * gauss(&s->rng);
  s->jolt *= expf(-dt / VZ_SIM_JOLT_TAU_S);

  /* Camera observer model (see vert_ctrl.h). */
  if (s->cmdT < 0.0f && cmd > 0.0f) {
    s->cmdT = s->t;
  }
  s->camAwake = c->camWakeS >= 0.0f && s->cmdT >= 0.0f && s->t - s->cmdT >= c->camWakeS;
  if (s->camAwake && s->liftT >= 0.0f) {
    const float tUse = fmaxf(s->cmdT + c->camWakeS, s->liftT);
    const float tLift = s->t - s->liftT;
    const bool lost = c->camLossS >= 0.0f && tLift >= c->camLossS && tLift < c->camLossS + c->camLossLenS;
    const float a = fminf(1.0f, dt / VZ_SIM_CAM_TAU_S);
    s->camN += -a * s->camN + sqrtf(2.0f * a) * c->camVzNoise * gauss(&s->rng);
    s->camVzValid = true;
    s->camHealthy = s->t - tUse >= c->camConvS && !lost;
    s->camVz = s->v + c->camVzOffset + s->camN;
    s->camBias = c->accAirG + c->camBiasErrG;
  } else {
    s->camVzValid = false;
    s->camHealthy = false;
    s->camVz = 0.0f;
    s->camBias = 0.0f;
  }
  s->t += dt;
}

/* ---- names --------------------------------------------------------------- */

const char *vzPhaseName(vzPhase_t p)
{
  switch (p) {
    case VZ_IDLE: return "IDLE";
    case VZ_SPIN: return "SPIN";
    case VZ_RAMP: return "RAMP";
    case VZ_CLIMB: return "CLIMB";
    case VZ_HOLD: return "HOLD";
    case VZ_LAND: return "LAND";
    case VZ_DONE: return "DONE";
    case VZ_CUT: return "CUT";
  }
  return "?";
}

const char *vzReasonName(vzReason_t r)
{
  switch (r) {
    case VZ_R_NONE: return "NONE";
    case VZ_R_TAKEOFF: return "TAKEOFF";
    case VZ_R_SPIN_DONE: return "SPIN_DONE";
    case VZ_R_LIFTOFF: return "LIFTOFF";
    case VZ_R_HOLD_REACHED: return "HOLD_REACHED";
    case VZ_R_CMD_LAND: return "CMD_LAND";
    case VZ_R_CMD_MISSING: return "CMD_MISSING";
    case VZ_R_SPIN_NOISY: return "SPIN_NOISY";
    case VZ_R_SPIN_TILT: return "SPIN_TILT";
    case VZ_R_SPIN_GYRO: return "SPIN_GYRO";
    case VZ_R_SPIN_SAG: return "SPIN_SAG";
    case VZ_R_NO_LIFTOFF_TIME: return "NO_LIFTOFF_TIME";
    case VZ_R_NO_LIFTOFF_CEIL: return "NO_LIFTOFF_CEIL";
    case VZ_R_LIFTOFF_ATTITUDE: return "LIFTOFF_ATTITUDE";
    case VZ_R_CLIMB_CAPPED: return "CLIMB_CAPPED";
    case VZ_R_CLIMB_TIMEOUT: return "CLIMB_TIMEOUT";
    case VZ_R_HOLD_CAPPED: return "HOLD_CAPPED";
    case VZ_R_HOLD_TIMEOUT: return "HOLD_TIMEOUT";
    case VZ_R_K_RANGE: return "K_RANGE";
    case VZ_R_EST_RANGE: return "EST_RANGE";
    case VZ_R_ABORTED: return "ABORTED";
    case VZ_R_TOUCHDOWN_FORCE: return "TOUCHDOWN_FORCE";
    case VZ_R_TOUCHDOWN_JOLT: return "TOUCHDOWN_JOLT";
    case VZ_R_LAND_TIMEOUT: return "LAND_TIMEOUT";
    case VZ_R_CMD_CUT: return "CMD_CUT";
    case VZ_R_NAN_INPUT: return "NAN_INPUT";
    case VZ_R_BAD_CONFIG: return "BAD_CONFIG";
    case VZ_R_HOLD_NO_CAM: return "HOLD_NO_CAM";
    case VZ_R_TOUCHDOWN_SUPPORT: return "TOUCHDOWN_SUPPORT";
    case VZ_R_RESUME: return "RESUME";
    case VZ_R_CONTACT_HOLD: return "CONTACT_HOLD";
  }
  return "?";
}
