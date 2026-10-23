/*
 * ESP-Drone Firmware
 *
 * vert_ctrl.h - Vertical control without a height sensor: take-off, a short
 * hold and landing from the accelerometer and a thrust model (autonomy stage
 * A2, docs/autonomy-a2.md section 4)
 *
 * The drone has only an MPU-6050, so height and vertical speed are the double
 * and single integral of the world-up specific force minus a ground baseline
 * g0 captured at idle spin before every take-off. That estimate drifts by
 * about +5 cm of true overshoot per mg of residual accelerometer offset over
 * a 3.5 s sortie, and the accelerometer alone cannot see that drift. An
 * optional external estimate (the camera + accelerometer observer: camVz,
 * camBias, camHealthy in vzInput_t) corrects the velocity and the offset
 * while it is healthy, and then HOLD keeps its velocity loop closed on it
 * instead of freezing the gain; without it HOLD is open loop under the
 * camera-measured hover thrust (capQ, "Hold and report" below) and lands
 * holdNoCamS after it was last used (the FSM's SINK comes first). Every limit
 * that protects against a height runaway is independent of both estimates:
 *   - thrust never exceeds VZ_THRUST_ABS_MAX (58000), nor capKhat * Khat / Vf;
 *   - the RAMP stops rising at rampTopRel * kPrior / Vf (hover at the stored
 *     gain), so a lift-off it fails to detect is not pushed on out of ground
 *     effect; an abort from that plateau or from any RAMP thrust at or above
 *     abortSlowRel * kPrior / Vf (A2b: the drone may be up undetected) falls
 *     like LAND's ceiling first, except a lift-off signature with a tilt or
 *     rate over the lift-off limits (LIFTOFF_ATTITUDE, one leg up), which
 *     falls at abortRate like an abort from below that thrust;
 *   - after lift-off, CLIMB never exceeds capLo * T_lo, where T_lo is the
 *     thrust that was applied when the drone left the ground, itself never
 *     above tLoMaxRel * kPrior / Vf (a late lift-off detection must not raise
 *     the climb ceiling); in HOLD the bound is battery-scaled,
 *     capLo * T_lo * max(V_lo, Vf) / Vf with V_lo the filtered voltage at
 *     lift-off, i.e. capLo * K_lo in K units (thrust units grow as the
 *     battery drains: in thrust units it bound at 51.7-58.9k at the end of
 *     full-charge sorties; the logged end hovers need 1.21-1.35 x K_lo);
 *   - LAND starts at landStart * T_ref and that ceiling falls at
 *     landRate * T_ref per second down to landFloor * T_ref; the velocity loop
 *     may only lower the thrust below the ceiling, never raise it. From HOLD
 *     with the camera estimate in use (T_ref is then the hover thrust just
 *     measured, and the velocity is the observer's) and started by the state
 *     machine it starts at landTop * T_ref (below that hover thrust) and
 *     stays there landFlatS
 *     before it falls, so the loop can hold its slow descent. The fast LAND
 *     (2026-10-14, docs/autonomy-a2b.md 17.4): a CMD_LAND from that slow LAND
 *     that the state machine allows (vzInput_t.landFastOk: its LAND reason is
 *     the Mac's LAND, the air clock, the hover wait's end or the sortie done;
 *     never a battery rule, a loop gap, a lost link, a guard, a contact) at a
 *     reference height of landFastMinH or more, with the camera estimate in
 *     use, T_ref <= VZ_LAND_FAST_TREF_MAX and Vf >= VZ_LAND_FAST_VMIN (never
 *     blind, never another controller reason, never the end of the battery)
 *     flies landVfast above landFastH + landFastBand (back to landVref by
 *     landFastH; only on steps with the camera estimate), and only to take that
 *     speed back, once the fast reference was flown, it may hold up to
 *     landBrakeTop * T_ref while the estimate reads between landBrakeH and
 *     landFastH + landFastBand, for at most landBrakeMaxS in all and only in the
 *     flat part (the cruise above stays at the slow LAND's ceiling); while LAND is
 *     stalled (not descending, from VZ_STALL_ARM_S on) it falls at stallRate
 *     instead: on the ground it reaches the touchdown thrust in a fraction of a
 *     second, in the air the drone then descends. T_ref is
 *     the thrust measured in HOLD; from CLIMB (A2b, open issue 1) it is the
 *     thrust averaged in CLIMB, kept within [T_lo, landClimbRefRel * kPrior /
 *     Vf], and T_lo itself if the climb was at a ceiling within the last
 *     tHoldTauS (a capped climb measures the ceiling, not the hover);
 *   - LAND ends in a cut after VZ_LAND_MAX_S whatever the estimate says.
 * Touchdown is declared only when the thrust is at or below
 * tdThrust * T_hov and either the measured specific force exceeds
 * tdForce times the thrust model's prediction for tdForceS, or a jolt is
 * seen (critique B4: a steady descent reads 1 g and is not "quiet"); below
 * tdNearH (estimated) a smaller jolt (tdNearJoltG) is touchdown at any
 * thrust (A2b hops: the first contact under power bounced and tipped while
 * the thrust waited for the ceiling). A bump over tdNearJoltG, or a stall
 * below tdNearH, in LAND sets the contact latch (vzOutput_t).
 *
 * Phases: IDLE -> SPIN (idle spin, g0) -> RAMP (thrust ramp until lift-off)
 * -> CLIMB -> HOLD -> LAND -> DONE (thrust 0). CUT is thrust 0 at once,
 * latched until vzInit(). Any non-finite input gives CUT in the same step.
 * The output thrust is final for the autonomy setpoint: tilt division and
 * every cap are already applied. The caller (autonomy glue) still applies its
 * mode clamp (DRY 0, BENCH <= 15000) and the stabilizer's own limits after it.
 *
 * A point-mass sim plant (vzSim*) with thrust gain K = thrust * V, ground
 * effect, rotor lag, battery sag, damping, accelerometer offset/noise and an
 * optional model of the camera observer's outputs feeds the controller in the
 * host tests (tools/tests/test_vert_ctrl.c, test_auto_sortie.c) and in the
 * drone's DRY and BENCH modes. Pure code: no ESP-IDF dependency.
 */
#ifndef VERT_CTRL_H_
#define VERT_CTRL_H_

#include <stdbool.h>
#include <stdint.h>

/* The hard thrust ceiling (RAMP, CLIMB, HOLD, the final clamp, the FSM's FLIGHT thrustMax). User decision
 * 2026-10-13 (late), docs/autonomy-a2b.md "End of battery": 58000 (was 54000). The hardest motor (FL +3.5 %,
 * battery forward) reaches 59.0k at the 57000 BATT_LOW level (AUTO_HEADROOM_LAND_T), 62.0k with the p99 steady
 * roll/pitch/yaw correction (+3.0k), of the 65280 the 8-bit duty reaches; with the old imbalance (BL +5.3 %)
 * 63.0k. 2000 below the 60000 at which power_distribution_stock.c stops every motor (its _Static_assert). */
#define VZ_THRUST_ABS_MAX     58000.0f
#define VZ_H_TARGET_MIN       0.20f     /* m, user decision: 0.20..0.40, default 0.30; the user raised the */
#define VZ_H_TARGET_MAX       2.00f     /* maximum to 0.60 on 2026-10-12 (evening), to 1.20 the same night (late,
                                           the interactive hover) and to 2.00 on 2026-10-15 (with SET_HEIGHT: Claude
                                           corrects the drifting estimate from photos, docs/autonomy-a2b.md 23.9).
                                           The hover cap: the moves' (AUTO_MV_H_MAX_M, asserted in auto_fsm.h) and
                                           VZ_HOFF_MAX follow from it; the take-off's is VZ_H_TAKEOFF_MAX */
#define VZ_H_TAKEOFF_MAX      1.20f     /* m: the take-off target (hTarget, vzConfigValid; AUTO_HEIGHT_CM_MAX, asserted
                                           in auto_fsm.h). 2.00 m is reached only by MOVEs up from HOLD (2026-10-15
                                           fix, docs/autonomy-a2b.md 23.9): a LAND in CLIMB has no measured hover, so
                                           its ceiling falls from landStart at once (startLand()); in the sim a LAND
                                           late in a 2.00 m climb touched down at 1.8-2.05 m/s (1.3-1.5 in a 1.20 m
                                           one, the firmware flown), a LAND from HOLD after the moves at <= 0.32 */
#define VZ_H_TARGET_DEFAULT   0.30f
#define VZ_LAND_MAX_S         6.0f      /* LAND longer than this ends in CUT (user decision), from a reference height
                                           up to VZ_H_KNEE; from higher, this plus the extra height at |landVref|
                                           (8.0 s from 1.20 m, 10.67 s from 2.00 m), so a LAND from up there is
                                           never cut in the air;
                                           and at least 4.0 s after a longer flat part (a move up: 8.8 s for
                                           0.20 + 1.00 m) */
#define VZ_HOLD_MAX_LIMIT_S   600.0f    /* holdMaxS may not exceed this. "Claude decides" (user decisions 2026-10-14
                                           ~20:30-20:40, docs/autonomy-a2b.md 22): the FSM has no air clock any more;
                                           the battery ends every flight (64-68 s per flight, ~177 s per charge
                                           measured), so HOLD_TIMEOUT is a backstop no battery reaches (120 s, the air
                                           clock AUTO_AIR_HARD_MS, from 2026-10-14 until then) */
#define VZ_H_TIMING_REF       0.40f     /* m: climbMaxS, landFlatS and hMax are sized for targets up to this (the
                                           user range until 2026-10-12); vzConfigSetHeight() raises them above it */
#define VZ_LAND_FLAT_MAX_S    2.0f      /* landFlatS for targets up to VZ_H_KNEE (vzConfigSetHeight caps it here) */
#define VZ_H_KNEE             0.60f     /* m: above this reference height the LAND flat part and the LAND timeout grow
                                           by the extra height at |landVref| (the interactive hover's 1.20 m) */
#define VZ_CLIMB_V_DEFAULT    0.40f     /* m/s: vClimbMax (vzDefaultConfig) */
#define VZ_LAND_V_DEFAULT     0.30f     /* m/s: |landVref| (vzDefaultConfig) */
#define VZ_CLIMB_BASE_S       2.0f      /* climbMaxS for targets up to VZ_H_TIMING_REF (vzDefaultConfig) */
/* The two timing limits (vzConfigValid) scale with the take-off cap: what vzConfigSetHeight() gives at
 * VZ_H_TAKEOFF_MAX with the defaults, plus 0.5 s: 4.5 and 4.5 s at 1.20 m (the values flown; a 2.00 m take-off would
 * give 6.5 and 7.17 s). A LAND's own flat part may add a vertical move and the drift allowance on top (landTimes():
 * not checked here; a move up to 2.00 m gives the 2.00 m take-off's flat part and timeout exactly). */
#define VZ_CLIMB_MAX_LIMIT_S  (VZ_CLIMB_BASE_S + (VZ_H_TAKEOFF_MAX - VZ_H_TIMING_REF) / VZ_CLIMB_V_DEFAULT + 0.5f)
#define VZ_LAND_FLAT_LIMIT_S  (VZ_LAND_FLAT_MAX_S + (VZ_H_TAKEOFF_MAX - VZ_H_KNEE) / VZ_LAND_V_DEFAULT + 0.5f)
#define VZ_VREF_MAX           0.20f     /* m/s: |vzInput_t.vRef| is clamped to this */
/* Long hovers (the interactive hover's waits, up to the 120 s air clock): the
 * height estimate drifts against the truth (3-10 mm/s on the logged camera
 * hovers; up to ~17 mm/s on the sim with a camera offset c of 10 mm/s and a K
 * error; the controller's estimate at touchdown after the logged hovers longer
 * than 20 s: -0.011..+0.031 m/s, RMS 0.020, docs/autonomy-a2b.md 18.1).
 * A LAND after a HOLD longer than VZ_LAND_DRIFT_AFTER_S keeps its flat part
 * (and its timeout) longer by the drift the whole HOLD may hide,
 * VZ_LAND_DRIFT_MPS per second of it, at most VZ_LAND_DRIFT_MAX_M, at
 * |landVnear| (the estimate below landSlowH asks for that speed): a drone
 * truly higher than its estimate still descends slowly to the ground, one
 * lower touches down earlier (the stall rule ends a flat ceiling on the
 * ground). Every HOLD of the envelope before (an 18 s air clock) is shorter:
 * unchanged. The cap is 2.0 m from the 120 s air clock (2026-10-14; 1.0 m at
 * 80 s): 0.017 m/s over 118 s, +10 s of flat part at most; a drone reading
 * 0.011 m/s low for 118 s is 1.3 m above its estimate. Without the air clock
 * (2026-10-15, docs/autonomy-a2b.md 22) a HOLD lasts until the battery ends it
 * (~65 s a flight measured; VZ_HOLD_MAX_LIMIT_S is a backstop): the 2.0 m cap
 * applies to every HOLD length, and a drone still higher than that after the
 * flat part is brought down by the falling ceiling (measured thrust and time
 * only), which ends the LAND whatever the estimate says. */
#define VZ_LAND_DRIFT_AFTER_S 18.0f
#define VZ_LAND_DRIFT_MPS     0.017f
#ifndef VZ_LAND_DRIFT_MAX_M   /* host builds only may override it (test_interactive_sim.c's comparison at 1.0 m) */
#define VZ_LAND_DRIFT_MAX_M   2.00f
#elif defined(ESP_PLATFORM)
#error "VZ_LAND_DRIFT_MAX_M is not configurable on the drone"
#endif
/* m: |vzInput_t.hOff| is clamped to this (= AUTO_MV_HOFF_MAX, asserted in auto_fsm.h): the commanded range (a 0.20 m
 * take-off plus moves up to the cap) plus a SET_HEIGHT's transient of up to 0.50 m (AUTO_SH_MAX_M: the reference sits
 * beyond the commanded range while a correction flies back; a smaller clamp would make hMax + hOff under-read: a false
 * EST_RANGE LAND). 2.30 at the 2.00 m cap, 1.50 at 1.20 m (1.00 until 2026-10-15; 0.60 in 591f176 sized the LAND for
 * 0.80 m). */
#define VZ_HOFF_MAX           ((VZ_H_TARGET_MAX - VZ_H_TARGET_MIN) + 0.50f)
/* SET_HEIGHT (2026-10-15, docs/autonomy-a2b.md 23): vzShiftHeight() takes |d| up to this (AUTO_SH_MAX_M + rounding) */
#define VZ_SHIFT_MAX_M        0.51f
#define VZ_EST_GRACE_MAX_S    3.0f      /* estGraceS at most this (NVS vzestms 0..3000 ms) */
#define VZ_HOLD_NO_CAM_MAX_S  8.0f      /* holdNoCamS may not exceed this: a backstop behind the FSM's SINK (5 s
                                           after the camera height was last in use, AUTO_CAM_LOST_MAX_MS; a static
                                           assert in auto_fsm.h), whose open loop cannot climb (thrust <= capQ, the
                                           camera-measured quiet hover thrust less kfrzBias; 5.0 until 2026-10-15) */
#define VZ_HOLD_NO_CAM_DEFAULT_S 7.0f    /* holdNoCamS's default (vzDefaultConfig) */
#define VZ_CAM_BIAS_MAX_G     0.050f    /* an external offset estimate beyond this is not used */
#define VZ_CAM_VZ_MAX         5.0f      /* m/s; an external velocity beyond this is not used */
#define VZ_DT_MIN_S           0.0002f   /* dt is clamped into this range before use */
#define VZ_DT_MAX_S           0.005f    /* a longer loop gap is integrated as 5 ms (the FSM lands on gaps > 20 ms) */
#define VZ_THRUST_HIST        32        /* look-back ring of applied thrust, one entry per step (~1 ms) */
#define VZ_G                  9.81f     /* m/s^2 per g */
#define VZ_STALL_ARM_S        0.30f     /* LAND: the stall test runs from this long into LAND (the descent starts from v 0) */
#define VZ_QUIET_S            1.0f      /* blind LAND: T_hold counts as the hover thrust this long after the last
                                           vertical reference (a move's, a bob's: T_hold, a 0.25 s average, also
                                           carries their accelerations, up to ~4 % over the hover) */
#define VZ_BLIND_FLOOR_S      0.5f      /* blind LAND (2026-10-15, docs/autonomy-a2b.md 22.10): not descending this
                                           long (the stall's speed test, whatever the height estimate reads), once
                                           the flat part without the drift allowance has run, ends the flat part
                                           (the floor in ground effect, a support; vert_ctrl.c blindFloor()) */
#define VZ_BLIND_STALL_ARM_S  1.0f      /* blind LAND: the stall rule (the estimate below tdNearH) from this long into
                                           it, not VZ_STALL_ARM_S: its open loop at landTop x the hover starts the
                                           descent slowly, and an estimate drifted low read that as a stall in the air */
/* The fast LAND (2026-10-14, docs/autonomy-a2b.md 17.4) is never flown near the end of the battery: T_ref (the
 * hover need) above the MOVE refusal level (auto_fsm.h AUTO_MV_REFUSE_T, a static assert there), or a loaded Vf
 * below VZ_LAND_FAST_VMIN. BATT_LOW lands at T_hold over 57000 or a 10 Hz battery under 3.00 V for 0.5 s, so
 * its LAND is never fast. */
#define VZ_LAND_FAST_TREF_MAX 53400.0f
#define VZ_LAND_FAST_VMIN     3.15f
/* Hold and report (user decisions 2026-10-14 ~20:30-20:40, docs/autonomy-a2b.md 22: "Claude decides"). The
 * controller's guards that only meant "this hold is unusual" no longer land: the drone holds as well as its sensors
 * allow and reports an event (vzOutput_t evt / evtN, which the FSM forwards to the Mac). Only a hold that is
 * physically impossible lands:
 *   - the upper thrust bound for VZ_HI_LAND_S (leaky: it counts down at twice the rate when off): Khat at
 *     VZ_HI_CLAMP_FRAC of its upper clamp with the loop closed, or the thrust at the model's ceiling (capKhat x Khat
 *     / Vf, below the battery bounds: capModel): LAND K_RANGE or HOLD_CAPPED (pulled down, too heavy, a failing
 *     cell: a drone that sinks against the bound). A drone held still or snagged (a hand, a curtain, a plant) is
 *     NOT caught by it: its camera speed reads ~0, so the closed loop never drives Khat to the clamp, and the props
 *     keep the hover thrust until Claude's LAND, the user's k, a trip or the battery (2026-10-15 verification, a
 *     grab at HOLD + 5 s for 65 s: 7 of 12 runs never landed, 34.7-52.3k thrust; docs/autonomy-a2b.md 22.7). The
 *     battery bounds (58000, capLo x K_lo) keep their capHitHoldS LAND HOLD_CAPPED;
 *   - Khat at its HOLD floor VZ_KHAT_HOLD_LO_REL x kPrior (with the camera estimate closing the loop the floor is
 *     there, not khatMinRel: the P term alone held a drone 12-17 % lighter than its prior in a steady climb,
 *     docs/autonomy-a2b.md 22.3) for khatClampS: LAND K_RANGE;
 *   - the height estimate outside hMin..hMax + hOff while the loop can climb (the camera estimate in use, or no
 *     camera-measured hover bound yet): LAND EST_RANGE, the runaway guard. A speed beyond vAbsMax alone is
 *     re-seeded (from camVz, or clamped) and reported. Settings (2026-10-22, user request 2026-10-18: it fired when
 *     the drone crossed from the floor to over a 0.45 m table at 0.9 m, the scene distance jumping; NVS vzest,
 *     vzestms): estRangeOn false never starts that LAND (the re-seed and the report stay, as do K_RANGE,
 *     HOLD_CAPPED, CLIMB_* and the FSM's SINK), estGraceS lets the LAND wait until the condition has held that long
 *     without a break (0: its first step, as before);
 *   - a climb fault (K_RANGE, CLIMB_CAPPED, CLIMB_TIMEOUT, a speed beyond vAbsMax in CLIMB) without the camera
 *     estimate in use (the estimate may read low: a hold could climb without bound); with it the climb ends in
 *     HOLD at the height reached and is reported.
 * Without the camera estimate the hold is open loop on the frozen gain and the observer's last offset (vzInput_t
 * camBias of the last step it was used, not the stored bAir), and its thrust is at most capQ = (1 - kfrzBias) x
 * the mean applied thrust over the latest VZ_QCAM_S of a quiet, uncapped camera-closed hold (vzOutput_t qCam): it
 * can sink, never climb. Before such a mean exists the open loop is today's (the FSM's SINK then comes at the
 * camera rule's first step). At most one event of the same reason per VZ_EVT_GAP_S (the event clock is tAir). */
#define VZ_HI_LAND_S          5.0f      /* s at the upper thrust bound: LAND */
#define VZ_HI_CLAMP_FRAC      0.995f    /* Khat at or above this x the upper clamp counts as at it */
#define VZ_KHAT_HOLD_LO_REL   0.75f     /* x kPrior: Khat's floor in HOLD while the camera estimate closes the loop */
#define VZ_QCAM_S             1.0f      /* s: the camera-closed quiet thrust is averaged over windows this long */
#define VZ_EVT_GAP_S          1.0f      /* s: the same event reason at most this often */
/* After a contact's hush the camera's speed is not trusted for VZ_HUSH_OPEN_S (the jolt's frames: a camera speed
 * wrong by 1 m/s for 1 s after the contact rose the hold 0.41 m when it was used at once; test_interactive_sim.c
 * testContactSim): the speed is the one from before the contact moved toward the camera's by at most
 * VZ_HUSH_DV_MAX (a push or a bounce the camera sees is damped; a wrong reading moves it no further), and the hold
 * flies open loop (the frozen gain, at most capQ: it cannot climb), then closes on the camera again. vzOutput_t
 * camHealthy is false in the window (the estimate's use) and camOpen true while the observer itself is healthy: the
 * FSM's camera rule does not count the window as a loss. */
#define VZ_HUSH_OPEN_S        1.5f
#define VZ_HUSH_DV_MAX        0.3f      /* m/s */

/* Stored values (NVS "vzK", learned by vzLearnK(); "vzBair", set over USB). */
#define VZ_K_DEFAULT          165000.0f /* thrust x loaded volts at hover at 40 kHz PWM: 163k (3.4-3.6 V) to 167k
                                           (3.25-3.3 V), logs 2026-10-11v and 2026-10-12a (A2a: 149.5k, 11 % low) */
#define VZ_K_MIN              120000.0f
#define VZ_K_MAX              180000.0f
#define VZ_BAIR_DEFAULT_G     (-0.0023f)  /* in-flight offset vs the pre-take-off baseline, 26 flights: -2.3 +/- 4.2 mg */
#define VZ_BAIR_MAX_G         0.030f

typedef enum {
  VZ_IDLE = 0,   /* thrust 0, waiting for VZ_CMD_TAKEOFF */
  VZ_SPIN,       /* idle spin, g0 captured, pre-checks */
  VZ_RAMP,       /* thrust ramp until lift-off is detected */
  VZ_CLIMB,      /* velocity profile up to hTarget */
  VZ_HOLD,       /* short hold at the estimated height */
  VZ_LAND,       /* estimate-independent ramp-down (or, before lift-off, the abort ramp) */
  VZ_DONE,       /* on the ground, thrust 0 (touchdown, abort complete or LAND before any spin) */
  VZ_CUT,        /* thrust 0 at once, latched until vzInit() */
} vzPhase_t;

/* What the state machine wants this step (level, not edge). */
typedef enum {
  VZ_CMD_NONE = 0,  /* only valid in IDLE, DONE and CUT; in any other phase it is treated as LAND */
  VZ_CMD_TAKEOFF,   /* fly the profile: IDLE -> SPIN, otherwise continue (never leaves LAND) */
  VZ_CMD_LAND,      /* land now: SPIN/RAMP -> abort ramp, CLIMB/HOLD -> LAND, IDLE -> DONE */
  VZ_CMD_CUT,       /* thrust 0 now, phase CUT */
} vzCmd_t;

/* Why the latest phase change happened. Values are logged and sent to the
 * Mac; only append. */
typedef enum {
  VZ_R_NONE = 0,
  VZ_R_TAKEOFF = 1,          /* IDLE -> SPIN */
  VZ_R_SPIN_DONE = 2,        /* SPIN -> RAMP, g0 captured */
  VZ_R_LIFTOFF = 3,          /* RAMP -> CLIMB */
  VZ_R_HOLD_REACHED = 4,     /* CLIMB -> HOLD */
  VZ_R_CMD_LAND = 5,         /* LAND commanded */
  VZ_R_CMD_MISSING = 6,      /* VZ_CMD_NONE while flying: treated as LAND */
  VZ_R_SPIN_NOISY = 7,       /* accZ std over the g0 window > spinStdMax */
  VZ_R_SPIN_TILT = 8,
  VZ_R_SPIN_GYRO = 9,
  VZ_R_SPIN_SAG = 10,        /* resting minus spinning battery > spinSagMax */
  VZ_R_NO_LIFTOFF_TIME = 11, /* RAMP lasted rampMaxS */
  VZ_R_NO_LIFTOFF_CEIL = 12, /* RAMP reached its ceiling without lift-off */
  VZ_R_LIFTOFF_ATTITUDE = 13,/* lift-off signature with tilt/gyro over the lift-off limits */
  VZ_R_CLIMB_CAPPED = 14,    /* CLIMB thrust at a ceiling for capHitClimbS */
  VZ_R_CLIMB_TIMEOUT = 15,   /* CLIMB lasted climbMaxS (a slow climb means the estimate reads low: unsafe sign) */
  VZ_R_HOLD_CAPPED = 16,     /* HOLD thrust at a ceiling for capHitHoldS */
  VZ_R_HOLD_TIMEOUT = 17,    /* HOLD lasted holdMaxS (backstop: 599 s, no battery reaches it) */
  VZ_R_K_RANGE = 18,         /* Khat at its clamp for khatClampS (CLIMB without the camera estimate: LAND; with it:
                                HOLD; in HOLD an event, a LAND after VZ_HI_LAND_S at the upper clamp or at the HOLD
                                floor VZ_KHAT_HOLD_LO_REL) */
  VZ_R_EST_RANGE = 19,       /* height or speed estimate outside hMin..hMax or |v| > vAbsMax (the height part
                                lands while the loop can climb; the speed part alone is re-seeded and reported) */
  VZ_R_ABORTED = 20,         /* abort ramp-down before lift-off reached 0: LAND -> DONE */
  VZ_R_TOUCHDOWN_FORCE = 21, /* LAND -> DONE: measured specific force > tdForce x predicted */
  VZ_R_TOUCHDOWN_JOLT = 22,  /* LAND -> DONE: jolt with the touchdown thrust condition met */
  VZ_R_LAND_TIMEOUT = 23,    /* LAND -> CUT after VZ_LAND_MAX_S */
  VZ_R_CMD_CUT = 24,
  VZ_R_NAN_INPUT = 25,       /* any non-finite input -> CUT */
  VZ_R_BAD_CONFIG = 26,      /* vzInit() with a config that fails vzConfigValid() -> CUT */
  VZ_R_HOLD_NO_CAM = 27,     /* HOLD holdNoCamS since the later of its entry and the last step with the external
                                estimate in use (a backstop behind the FSM's SINK) */
  VZ_R_TOUCHDOWN_SUPPORT = 28,/* LAND -> DONE: the palm catch's support probe (removed 2026-10-15, docs/autonomy-a2b.md
                                22: never produced; the value stays on the wire) */
  VZ_R_RESUME = 29,          /* LAND -> HOLD: a SINK LAND (CMD_LAND from HOLD) resumed, the camera estimate back
                                (vzInput_t.resume) */
  VZ_R_CONTACT_HOLD = 30,    /* CLIMB -> HOLD: an in-air contact in the climb with the camera estimate in use
                                (vzInput_t.hush) ends the climb where it is; also its event */
} vzReason_t;

/* Gains, limits and timings. vzDefaultConfig() gives the values of
 * docs/autonomy-a2.md section 4; the glue sets hTarget, kPrior and bAir per
 * sortie. vzInit() copies the config, so it cannot change mid-sortie. */
typedef struct {
  /* sortie */
  float hTarget;          /* m, VZ_H_TARGET_MIN..MAX */
  float kPrior;           /* thrust x V at hover (NVS vzK), VZ_K_MIN..MAX */
  float bAir;             /* g, in-flight offset vs g0 (NVS vzBair), |.| <= VZ_BAIR_MAX_G; used from lift-off on */
  /* ceilings (all applied after the tilt division) */
  float capKhat;          /* 1.15: thrust <= capKhat * Khat / Vf in RAMP, CLIMB and HOLD */
  float capLo;            /* 1.65 (NVS vzcaplo; hover is 1.28-1.57 x lift-off): CLIMB thrust <= capLo * T_lo,
                             HOLD <= capLo * T_lo * max(V_lo, Vf) / Vf (battery-scaled, 2026-10-14) */
  float tLoMaxRel;        /* 0.90: T_lo <= tLoMaxRel * kPrior / Vf (logged lift-off <= 0.84 x the flight's own hover) */
  float tiltDivMin;       /* 0.94: thrust is divided by max(gravZ, tiltDivMin) */
  /* SPIN */
  float spinS;            /* 0.8 */
  float spinThrustMax;    /* 20000 */
  float spinThrustRel;    /* 0.45: spin thrust = min(spinThrustMax, spinThrustRel * kPrior / Vf) */
  float g0WindowS;        /* 0.5: g0 = mean accZ over the last g0WindowS of SPIN */
  float spinStdMax;       /* 0.040 g */
  float spinTiltMax;      /* 8 deg */
  float spinGyroMax;      /* 15 dps ... */
  float spinSagMax;       /* 0.35 V */
  /* RAMP and lift-off */
  float rampStartRel;     /* 0.50: first RAMP thrust = max(spin thrust, rampStartRel * kPrior / Vf) */
  float rampTopRel;       /* 1.00: the RAMP stops rising at rampTopRel * kPrior / Vf (hover at the prior; logged
                             lift-off up to 0.91 x the default prior on today's drone) and waits for rampMaxS */
  float rampRate;         /* 13250 per s: 12000 x 165000 / 149500, the same fraction of the hover thrust per second
                             as validated at the A2a prior (an absolute 12000 stretched the ramp and cut the plateau
                             dwell inside rampMaxS: hidden lift-offs at -10 mg went 85 -> 36 % detected in 250 ms) */
  float rampMaxS;         /* 2.5 */
  float abortRate;        /* 60000 per s: ramp-down before lift-off */
  float abortSlowRel;     /* 0.60 (A2b): an abort from a RAMP thrust at or above abortSlowRel * kPrior / Vf (a
                             plausible lift-off: logged lift-off 0.71 x hover, K -12 % gives 0.62 x the prior)
                             falls like LAND's ceiling first, as one from the plateau; 0.40..1.00 */
  float liftAcc;          /* 0.025 g: 20 Hz filtered accZ - g0 above this (no bAir: on the ground it is absent) ... */
  float liftHoldS;        /* 0.030 ... for this long, or */
  float liftCusumV;       /* 0.02 m/s: CUSUM of g*(accZ - g0 - liftCusumK) reaches this */
  float liftCusumK;       /* 0.007 g allowance: zero-mean noise cannot climb the CUSUM */
  float liftTiltMax;      /* 15 deg */
  float liftGyroMax;      /* 250 dps */
  float liftLatencyS;     /* 0.030: T_lo = applied thrust this long before the detection */
  /* CLIMB and HOLD */
  float vClimbMax;        /* 0.40 m/s */
  float aClimb;           /* 1.0 m/s^2: vref ramps up at this rate */
  float kH;               /* 2.0 1/s: vref <= kH * (hTarget - h) */
  float vDescMax;         /* 0.30 m/s: vref >= -vDescMax */
  float kpRel;            /* 0.35: Kp = kpRel * Khat / Vf per m/s */
  float kiRel;            /* 3.0: Khat += kiRel * kpRel * Khat * (vref - v) * dt */
  float khatMinRel;       /* 0.88 x kPrior */
  float khatMaxRel;       /* 1.12 x kPrior */
  float khatClampS;       /* 0.3: Khat at its clamp this long -> LAND K_RANGE */
  float khat0LoRel;       /* 1.2: Khat0 = max(kPrior, khat0LoRel * T_lo * V_lo) */
  float climbFloorRel;    /* 0.75: CLIMB thrust >= climbFloorRel * Khat / Vf (below every ceiling) */
  float holdBand;         /* 0.05 m: HOLD when h >= hTarget - holdBand */
  float climbMaxS;        /* 2.0 (targets up to VZ_H_TIMING_REF; vzConfigSetHeight() adds the extra height at
                             vClimbMax: 2.5 at 0.60 m, 4.0 at 1.20 m, the take-off cap) -> LAND CLIMB_TIMEOUT */
  float capHitClimbS;     /* 0.7: at a ceiling this long in CLIMB -> LAND CLIMB_CAPPED */
  float capHitHoldS;      /* 0.5 */
  float holdLoopS;        /* 0.5: closed loop at vref 0, then the gain is frozen (while the external estimate is
                             usable the loop stays closed on it and Kfrz follows Khat); Khat here is the learning
                             sample. A contact's hush restarts it on its falling edge */
  float kfrzBias;         /* 0.005: Kfrz = Khat * (1 - kfrzBias), a slight sink */
  float holdMaxS;         /* 599.0 backstop, <= VZ_HOLD_MAX_LIMIT_S (119.0 under the 120 s air clock until 2026-10-15;
                             the battery ends every flight long before) */
  float holdNoCamS;       /* 7.0: in HOLD, this long since the later of its entry and the last step with the external
                             estimate in use (noCamS) -> LAND HOLD_NO_CAM (backstop: the FSM's SINK comes 2 s earlier;
                             never on the sim), <= VZ_HOLD_NO_CAM_MAX_S (3.0 s of HOLD until 2026-10-15) */
  float camTauS;          /* 0.2 s: v and vHp follow camVz with this time constant while it is usable */
  float steadyVMax;       /* 0.15 m/s: holdSteady needs |vHp| below this ... */
  float steadyBand;       /* 0.06: ... and thrust within +/- this of Kfrz / Vf, not at a ceiling */
  float vHpTauS;          /* 1.0: leaky velocity used for damping and holdSteady */
  float vfTauS;           /* 0.5: battery filter for every K / V */
  float tHoldTauS;        /* 0.25: average of the applied thrust in HOLD = T_hold */
  float hMin, hMax;       /* -0.3, 1.0 m: estimate guards (they cannot see offset drift); hMax is 0.6 m over the
                             0.40 m target (VZ_H_TIMING_REF), and vzConfigSetHeight() keeps that margin above it:
                             1.2 m at 0.60 m, 1.8 m at 1.20 m; hMax + hOff after moves: 2.6 m at 2.00 m */
  float vAbsMax;          /* 1.0 m/s */
  /* LAND (estimate-independent ceiling; the loop may only lower it) */
  float landStart;        /* 0.95 x T_ref */
  float landRate;         /* 0.10 x T_ref per s */
  float landFloor;        /* 0.65 x T_ref */
  float landVref;         /* -0.30 m/s */
  float tdThrust;         /* 0.75: touchdown needs thrust <= tdThrust * T_hov */
  float tdForce;          /* 1.3: and measured > tdForce x predicted specific force ... */
  float tdForceS;         /* 0.2 s ... */
  float tdJoltG;          /* 0.6 g: or a jolt (20 Hz accZ - g0) above this */
  float landClimbRefRel;  /* 1.00 (A2b): LAND from CLIMB: T_ref <= landClimbRefRel * kPrior / Vf; 0.80..1.00 */
  /* LAND after HOLD and the touchdown near the expected surface (A2b hops, 2026-10-12) */
  float landTop;          /* 0.98: LAND from HOLD with the camera estimate in use: the ceiling starts at landTop *
                             T_ref (0.80..1.00; below 1 so a wrong estimate cannot hold the drone up) ... */
  float landFlatS;        /* 1.5 s: ... and falls only after this (0..VZ_LAND_FLAT_LIMIT_S; vzConfigSetHeight() adds the
                             extra height at |landVref| above VZ_H_TIMING_REF: 2.0 at 0.60 m (capped there), 4.0 at
                             1.20 m; a LAND above the take-off height after a vertical move adds
                             hOff / |landVref|);
                             otherwise landStart, falling at once. Above VZ_H_KNEE the flat LAND applies without the
                             camera estimate too (its ceiling is measured thrust and time only) */
  float landVnear;        /* -0.20 m/s: vref is landVref at h >= landSlowH, landVnear at h <= 0 (landVref..-0.05) */
  float landSlowH;        /* 0.20 m */
  float tdNearH;          /* 0.15 m: below this estimated height a jolt (aRel > tdNearJoltG) is touchdown at any
                             thrust (0..0.30) */
  float tdNearJoltG;      /* 0.30 g (0.15..tdJoltG): in LAND the ceiling keeps the in-air aRel below ~+0.05 g
                             (A2b hops: <= +0.04 g in LAND before contact, <= +0.15 g in CLIMB) */
  float stallV;           /* 0.08 m/s: LAND is stalled while v > -stallV ... */
  float stallS;           /* 0.10 s: ... for this long (from VZ_STALL_ARM_S into LAND); then the ceiling ... */
  float stallRate;        /* 1.00 x T_ref per s: ... falls at this rate (landRate..2.0) */
  /* The fast LAND (2026-10-14, docs/autonomy-a2b.md 17.4): a CMD_LAND from the slow LAND (landSlow) at a
   * reference height of landFastMinH or more with the camera estimate in use, T_ref <= VZ_LAND_FAST_TREF_MAX
   * and Vf >= VZ_LAND_FAST_VMIN, not blind. Today's LAND cannot brake (thrust <= landTop x T_ref), so a faster
   * vref alone reaches the floor faster; the brake is a small, time-bounded authority above T_ref. */
  float landVfast;        /* -0.50 m/s: vref above landFastH + landFastBand, on steps with the camera estimate
                             (-0.60..landVref; landVref turns the fast LAND off) */
  float landFastH;        /* 0.60 m: ... back to landVref linearly by this height (0.30..1.00) */
  float landFastBand;     /* 0.25 m: the width of that blend, the slow-down band (0.05..0.50) */
  float landFastMinH;     /* 1.00 m: the lowest reference height (hTarget + hOff) of a fast LAND
                             (landFastH + landFastBand..1.50) */
  float landBrakeTop;     /* 1.03: the brake may hold up to this x T_ref (1.00..1.05), once the fast reference was
                             flown (its blend at least halfway) ... */
  float landBrakeH;       /* 0.40 m: ... while the estimate reads above this (landSlowH..landFastH) and at most
                             landFastH + landFastBand (the slow-down band: never in the cruise above) ... */
  float landBrakeMaxS;    /* 0.5 s: ... for at most this long in all, in the flat part only (0..1.0) */
  /* SPIN (docs/autonomy-a2b.md 19.7), last: the fields above keep their offsets in the IRAM code */
  float spinGyroHoldS;    /* 0.020 s: SPIN_GYRO needs spinGyroMax exceeded this long (0..0.2; 0: one step, as before);
                             26 logged idle spins peak at 5.7-14.4 dps in single 100 Hz samples, three in a row at
                             <= 9.3; a handling or tip event holds well over 15 dps for over 100 ms */
  /* the EST_RANGE guard as settings (2026-10-22; NVS vzest, vzestms; appended: the offsets above stay) */
  bool estRangeOn;        /* true: the height estimate out of range LANDs EST_RANGE (vzConfigValid takes either);
                             false: never that LAND (the speed re-seed and the report stay) */
  float estGraceS;        /* 0 s: the out-of-range condition must hold this long, unbroken, before the LAND
                             (0..VZ_EST_GRACE_MAX_S; 0: on its first step, as before) */
} vzConfig_t;

void vzDefaultConfig(vzConfig_t *c);
/* Every field finite and inside the hard limits of docs/autonomy-a2.md
 * section 4.6 (for example capKhat <= 1.25, landRate > 0, hTarget 0.20..1.20, VZ_H_TAKEOFF_MAX). */
bool vzConfigValid(const vzConfig_t *c);

/* The sortie's take-off height (the glue's vzReset; the host tests' glue
 * emulations): sets hTarget, and above VZ_H_TIMING_REF raises the three
 * values sized for the old 0.40 m maximum: the estimate guard hMax by the
 * extra height (hTarget - VZ_H_TIMING_REF), so it stays 0.6 m over the
 * target, and the two timings by the time the extra height takes:
 * climbMaxS by (hTarget - VZ_H_TIMING_REF) / vClimbMax (the climb profile
 * flies it at its top speed) and landFlatS by (hTarget - VZ_H_TIMING_REF) /
 * |landVref|, at most VZ_LAND_FLAT_MAX_S up to VZ_H_KNEE and by the height
 * above the knee at |landVref| beyond it. At 0.60 m: hMax 1.0 -> 1.2 m,
 * climbMaxS 2.0 -> 2.5 s, landFlatS 1.5 -> 2.0 s; at 1.20 m: hMax 1.8 m,
 * climbMaxS 4.0 s, landFlatS 4.0 s (2.00 m, above the take-off cap: hMax 2.6 m,
 * climbMaxS 6.0 s, landFlatS 6.67 s; invalid). A target up to VZ_H_TIMING_REF (or NaN)
 * leaves all three unchanged. Call it once, on a config whose hMax, climbMaxS
 * and landFlatS are the base values (vzDefaultConfig()'s); vzInit() still
 * checks the result. */
void vzConfigSetHeight(vzConfig_t *c, float hTarget);

/* One control step, normally at 1 kHz from the stabilizer task. */
typedef struct {
  float dt;               /* s since the previous step; clamped to VZ_DT_MIN_S..VZ_DT_MAX_S */
  vzCmd_t cmd;
  float accZ;             /* g, world-up specific force: acc . gravity direction (~1.0 at rest) */
  float gravZ;            /* z of the gravity direction in the body frame, cos(tilt) */
  float tiltDeg;          /* acos(gravZ) */
  float gyroMaxDps;       /* max(|gx|, |gy|, |gz|) */
  float vbat;             /* V: 10 Hz battery reading (or the sim plant's) */
  float thrustApplied;    /* what the plant really got on the previous step, after every downstream limit */
  bool fromSim;           /* inputs come from vzSim (DRY/BENCH): vzLearnK() refuses the sortie, no camera rule */
  /* External vertical estimate: the camera + accelerometer observer
   * (docs/autonomy-a2.md section 4.10), wired in by the glue. Optional: all
   * false/0 without it. Used only from lift-off on and only while camHealthy;
   * a non-finite or out-of-range value makes it unusable for that step (it
   * never gives CUT: HOLD_NO_CAM and the FSM's camera rule land instead). */
  bool camHealthy;        /* the observer's confidence: offset converged and camera updates accepted recently */
  bool camVzValid;        /* camVz is this step's estimate */
  float camVz;            /* m/s, up: the observer's vertical velocity */
  float camBias;          /* g: the observer's in-flight world-up offset vs this sortie's g0 (what bAir stands for);
                             cam_vz_obs.c reports it vs 1 g, so the glue passes biasG + 1 - vzOutput_t.g0 */
  /* Interactive hover (auto_motion.h): both exactly 0 without a vertical move, and then every output is
   * bit-identical to the controller without them. */
  float vRef;             /* m/s, up: HOLD's vertical speed reference (closed loop only: the first holdLoopS and
                             while the camera estimate is in use); clamped to +/-VZ_VREF_MAX, non-finite 0 */
  float hOff;             /* m: the commanded height offset from hTarget: hMax follows it (the estimate guard stays
                             the same margin over the reference), and a LAND above VZ_H_KNEE or above hTarget gets
                             the longer flat part and timeout; clamped to +/-VZ_HOFF_MAX, non-finite 0 */
  /* LAND after an in-air contact (the FSM's LAND COLLISION in a climb without the camera estimate, and since the user's
   * decision (B) of 2026-10-15 every LAND of a sortie that had a contact, whatever its reason and however long after it;
   * docs/autonomy-a2b.md 22.9): the contact's impulses corrupt the height and speed estimate (2026-10-13 flight 1:
   * +0.42..+0.85 m/s of climb that was not there), and a camera that stays wrong after it would land the drone at its
   * error (a 1 m/s one touched down at 3.9 m/s in the sim), so a blind LAND reads neither. Its thrust is the ceiling
   * alone (measured thrust and time: from HOLD landTop x the lowest of T_hold, T_hold from before the last vertical
   * move or bob, and the camera-measured quiet hover tQc scaled to the battery now, for the flat part, with or
   * without the camera estimate, then falling at landRate (T_hold carries a motion's accelerations, and a late LAND
   * may begin in a hold that pushes: blindLand()); the controller's own guard LANDs (EST_RANGE, K_RANGE, HOLD_CAPPED,
   * HOLD_NO_CAM) the same since docs/autonomy-a2b.md 22.10 (from landStart, falling at once, until then); from CLIMB
   * landStart x the CLIMB average, falling at once, also after a capped step: that cap is the contact's); the stall
   * rule acts only with the estimate below tdNearH (the floor) and from VZ_BLIND_STALL_ARM_S into the LAND, and the
   * bump-below-tdNearH touchdown is off; once the flat part without the drift allowance has run, VZ_BLIND_FLOOR_S
   * without a descent (whatever the height estimate reads) or the contact latch ends the flat part (the floor in
   * ground effect; 22.10: 5-16 s of float after a long hover until then); the touchdown needs the thrust condition
   * (jolt or force rule). The LAND timeout is unchanged; after a move down
   * the flat part is the reference height's, not the take-off height's (it ran into the knee's 6 s timeout). Level,
   * read on every LAND step and at the LAND's start; false: every output exactly as without it. Outside LAND (HOLD)
   * it changes nothing: the hold keeps the camera. docs/autonomy-a2.md section 12.8. */
  bool blind;
  /* The fast LAND (docs/autonomy-a2b.md 17.4) is allowed for this LAND's reason: the state machine's allow list
   * (auto_fsm.h autoFsmLandFastReason(): the Mac's LAND, the air clock, the hover wait's end, the sortie done).
   * Read at the LAND's start; false (any other reason: VBAT_DIP, VBAT_LOW, BATT_LOW, LOOP_GAP, a lost link or app,
   * a guard): every output exactly as without the fast LAND. */
  bool landFastOk;
  /* An in-air contact (the FSM's CONTACT, AUTO_CONTACT_HUSH_MS; docs/autonomy-a2b.md 22), in CLIMB or HOLD: while
   * true h, v and vHp are not integrated (the impulses would corrupt them), the thrust is the hover estimate less
   * kfrzBias (capQ's mean, else min(T_hold or the CLIMB average, Khat / Vf)) under the ceiling, and Khat, the
   * clamp, cap and hiT timers and T_hold are frozen; a CLIMB ends in HOLD (VZ_R_CONTACT_HOLD, T_hold and tHoldQ
   * seeded with that estimate). Its v before the contact (vPre) leaves out the pulse the FSM's trip integrated. On
   * its falling edge v and vHp are re-seeded from vPre (moved toward camVz by at most VZ_HUSH_DV_MAX) and, in HOLD,
   * VZ_HUSH_OPEN_S of open loop follow (the camera estimate unused, capQ the bound), then the loop closes on the
   * camera again. Level; false on every step: every output exactly as without it. (The palm catch's support probe
   * had this slot until 2026-10-15.) */
  bool hush;
  /* A SINK (the FSM's LAND after the camera budget): in a LAND started by CMD_LAND after HOLD, with the camera
   * estimate in use, no contact latch and no touchdown, true makes it HOLD again (VZ_R_RESUME), blind or not since
   * 2026-10-15 (the FSM does not ask in a contact's window); a blind one only in its flat part and while descending
   * (not after stallS at rest: the floor, docs/autonomy-a2b.md 22.10). Level; false: as without it. */
  bool resume;
} vzInput_t;

typedef struct {
  float thrust;           /* final command 0..VZ_THRUST_ABS_MAX; 0 in IDLE, DONE and CUT */
  vzPhase_t phase;
  vzReason_t reason;      /* latest phase change */
  vzReason_t landReason;  /* why LAND (or the abort) started; NONE until then; kept through DONE/CUT */
  float h, v;             /* m, m/s: integrated estimate (0 until lift-off) */
  float vHp;              /* m/s: leaky velocity */
  float aRel;             /* g: 20 Hz filtered accZ - g0 */
  float g0;               /* g: ground baseline of this sortie (1.0 until captured) */
  float khat;             /* thrust x V model, adapted in CLIMB, the first holdLoopS of HOLD and HOLD on the camera */
  float kfrz;             /* hold gain from holdLoopS on (0 before): frozen without the external estimate */
  float tLo;              /* applied thrust at lift-off (0 before) */
  float tHold;            /* average applied thrust in HOLD (0 before HOLD) */
  float tRef;             /* LAND reference: tHold if HOLD was reached; from CLIMB the clamped CLIMB average (or tLo
                             after a capped climb); 0 before LAND */
  float ceiling;          /* the ceiling in force this step */
  float tAir;             /* s since lift-off */
  float tPhase;           /* s in the current phase */
  bool liftedOff;         /* latched at lift-off */
  bool liftoffNow;        /* true for the one step that detected lift-off */
  bool holdReached;       /* HOLD was entered in this sortie */
  bool holdSteady;        /* vertical part of STABLE this step (HOLD after holdLoopS, see the doc) */
  bool atCap;             /* thrust limited by a ceiling this step */
  bool tdThrustOk;        /* LAND: the touchdown thrust condition holds (with contact, a tilt/rate trip counts as touchdown) */
  bool touchdown;         /* latched: DONE through TOUCHDOWN_FORCE, TOUCHDOWN_JOLT or TOUCHDOWN_SUPPORT */
  bool camHealthy;        /* the external estimate was usable and used this step (airborne; the FSM's HOVER rule reads it) */
  bool contact;           /* LAND after lift-off: a bump (aRel > tdNearJoltG) or a stall below tdNearH; latched until vzInit()
                             (the glue stops the camera hold's lean on it) */
  float landMaxS;         /* the LAND timeout in force (VZ_LAND_MAX_S, longer from above VZ_H_KNEE) */
  /* hold and report (docs/autonomy-a2b.md 22; appended: the fields above keep their offsets in the IRAM code) */
  uint8_t evt;            /* vzReason_t of the latest event: a guard that held and reported instead of landing
                             (K_RANGE, HOLD_CAPPED, EST_RANGE, CLIMB_CAPPED, CLIMB_TIMEOUT, CONTACT_HOLD, RESUME) */
  uint8_t evtN;           /* events this sortie (wraps): a change is a new event */
  float capQ;             /* the open loop's thrust bound, (1 - kfrzBias) x the camera-measured quiet hover thrust
                             (vert_ctrl.h "Hold and report"); 0: none yet (the FSM's SINK then comes at once) */
  float noCamS;           /* s since the later of the HOLD entry and the last step with the camera estimate in use */
  float hiS;              /* s at the upper thrust bound (the leaky VZ_HI_LAND_S count) */
  bool camOpen;           /* a contact's open window (VZ_HUSH_OPEN_S) with the observer healthy: camHealthy is false
                             for the window, not for a loss (the FSM's camera rule does not count it) */
} vzOutput_t;

/* Controller state. Fields are private to vert_ctrl.c (its owner may change
 * them freely); callers only allocate it and pass it back. */
typedef struct {
  vzConfig_t cfg;
  vzPhase_t phase;
  vzReason_t reason, landReason;
  float tPhase, tAir;
  float vf, vRest, spinThrust, rampStart;
  double g0Sum, g0SumSq;
  uint32_t g0N;
  float g0, aLp20;
  float cusumV, cusumH, cusumT, liftT;
  float h, v, vHp;
  float khat, kfrz, tLo, vLo, tHold, kRef, tRef;
  float tClimb;           /* average applied thrust in CLIMB (tau tHoldTauS), T_lo at lift-off */
  float climbFreeS;       /* CLIMB time since the last step at a ceiling (0 while capped) */
  float kLearn;           /* Khat on the first HOLD step at holdLoopS (0 before): vzLearnK()'s sample */
  float thrust, ceiling;
  float hist[VZ_THRUST_HIST];
  uint8_t histHead;
  float capT, clampT, tdT, stallT;
  float abortRef;         /* RAMP thrust when a slow abort started: at the plateau or >= abortSlowRel * kPrior / Vf,
                             not LIFTOFF_ATTITUDE (0: fast abort at abortRate) */
  bool started, liftedOff, holdReached, kFrozen, touchdown, fromSimSeen, lastAtCap, rampTop;
  bool camNow;            /* this step's camUsed (read by startLand) */
  bool landSlow;          /* LAND from HOLD with the external estimate in use, started by the state machine (CMD_LAND,
                             CMD_MISSING) or HOLD_TIMEOUT, or from HOLD blind whatever its reason (a measured hover):
                             landTop / landFlatS apply */
  bool contact;           /* LAND contact latch (vzOutput_t.contact) */
  float hOff;             /* this step's vzInput_t.hOff (sanitized) */
  float landMaxS;         /* LAND timeout of this LAND (set by startLand) */
  float landFlatEff;      /* LAND flat part of this LAND (set by startLand) */
  bool blind;             /* this step's vzInput_t.blind */
  float tHoldQ;           /* T_hold as of the last HOLD step without a vertical reference in the VZ_QUIET_S before it
                             (0 before HOLD): a blind LAND's hover reference (blindLand()) */
  float vRefT;            /* HOLD: tPhase of the last step with a vertical reference (vRef != 0); -VZ_QUIET_S before */
  bool landFast;          /* this LAND is a fast one (set by startLand) */
  bool landFastOk;        /* vzInput_t.landFastOk of the latest step (read at the LAND's start) */
  bool landFastFlown;     /* the fast LAND's reference reached at least half of its blend: the brake may act */
  float brakeS;           /* the fast LAND's time with the thrust above the ceiling (the brake) */
  /* hold and report (docs/autonomy-a2b.md 22; the support probe's slot until 2026-10-15) */
  float lastBias;         /* the external offset estimate on the last step it was used (the open loop's; bAir before) */
  float noCamT;           /* vzOutput_t noCamS */
  float hiT;              /* vzOutput_t hiS */
  float vPre;             /* v on the first step of a contact's hush */
  float loopEnd;          /* HOLD: closed on the accelerometer while tPhase is below this (holdLoopS; a hush's
                             falling edge ends it: its open window) */
  float qSum, qT;         /* the current window of camera-closed quiet thrust: sum x dt, its time */
  float tQc;              /* the latest complete window's mean (0: none) ... */
  float capQ;             /* ... and (1 - kfrzBias) x it: vzOutput_t capQ */
  float capLim;           /* the open loop's thrust bound: capQ, VZ_THRUST_ABS_MAX before it */
  float kLoCam, kLoMin, kHiTh;   /* kPrior x VZ_KHAT_HOLD_LO_REL, x khatMinRel, x khatMaxRel x VZ_HI_CLAMP_FRAC */
  float hushT;            /* the hush's thrust (hushEdge()) */
  float evtLastT;         /* tAir of the latest event */
  uint8_t evt, evtN;      /* vzOutput_t's */
  bool hushOn;            /* a contact's hush runs */
  bool postOn;            /* HOLD after a hush: the camera estimate unused for VZ_HUSH_OPEN_S (until postT) */
  bool capModel;          /* the ceiling in force this step is the model's (climbCeiling()) */
  bool estOut;            /* the estimate was out of range on the previous HOLD step (its event on the edge) */
  bool kLowOn;            /* Khat at or below khatMinRel x kPrior (its event on the edge) */
  bool resumed;           /* a SINK LAND resumed in this sortie (vzLearnK refuses it) */
  bool evtAny;            /* any event this sortie (vzLearnK refuses it) */
  bool hoverKnown;        /* HOLD reached at the target, or capQ measured since: a LAND from HOLD uses T_hold; a HOLD
                             that a climb fault or a contact started without either lands as from CLIMB */
  float spinGyroS;        /* SPIN: the time the gyro has been over spinGyroMax, consecutive (last: the offsets) */
  float postT;            /* tPhase at the open window's end (postOn) */
  float vfQc;             /* Vf at the end of tQc's window (2026-10-15, appended): a blind LAND scales tQc to the battery
                             now (thrust x V, the K model: a battery that sagged since needs more thrust) */
  float landFlatBase;     /* a blind LAND's flat part without the drift allowance (landTimes(); 2026-10-15, appended):
                             from there VZ_BLIND_FLOOR_S without a descent ends the flat part (blindFloor()) */
  float estOutS;          /* s the EST_RANGE LAND's condition has held, unbroken (estGraceS; 2026-10-22, appended) */
} vzState_t;

/* Copies cfg and resets to IDLE. An invalid cfg leaves the state in CUT
 * (reason BAD_CONFIG), so a bad config can never spin the motors. */
void vzInit(vzState_t *st, const vzConfig_t *cfg);
void vzStep(vzState_t *st, const vzInput_t *in, vzOutput_t *out);

/* After a sortie that qualifies, moves 0.3 of the way from kPrior to this
 * sortie's learning sample, at most 5 % per sortie, within VZ_K_MIN..MAX.
 * The sample is Khat on the first HOLD step at holdLoopS: at the target
 * height, out of ground effect, before the hover wait. It is not the running
 * Kfrz, which follows Khat for the rest of a camera hold (a camera velocity
 * offset sinks the drone into ground effect and walks Khat to its clamp)
 * and carries the kfrzBias sink.
 * Qualifies: DONE through touchdown, the sample taken, tAir >= 1.5 s at
 * touchdown, no step had fromSim set, and LAND started by CMD_LAND (the
 * state machine's landings) or HOLD_NO_CAM (the camera dropped out after the
 * sample). Any other start of LAND is refused: K_RANGE, HOLD_CAPPED and
 * EST_RANGE say the model or the estimate left its range in the hold;
 * HOLD_TIMEOUT and CMD_MISSING say the state machine did not land the
 * sortie; a reason appended later stays refused until it is reviewed here.
 * Also refused: a sortie with any event (evtN > 0: a clamp, a cap, a
 * re-seeded estimate, a climb ended early, a contact) or a resumed SINK.
 * Returns false and leaves kPrior unchanged otherwise. (bAir keeps its NVS
 * value in stage A2; learning it needs rest baselines before and after the
 * sortie and is specified with stage B.) */
bool vzLearnK(const vzState_t *st, float *kPrior);

/* SET_HEIGHT ("you are at X m", auto_fsm.h AUTO_SH_*, docs/autonomy-a2b.md 23): the height estimate h moves by d (m),
 * nothing else of the state (v, vHp, Khat, Kfrz, T_hold, tHoldQ, tQc / capQ, kLearn, the LAND fields): the hold, a
 * speed loop, flies exactly as before (the correction itself is the state machine's vertical MOVE). Only in HOLD, d
 * finite and |d| <= VZ_SHIFT_MAX_M; false (nothing changed) otherwise. The glue calls it on the step the state machine
 * accepted the command, before that step's vzStep() (autonomy.c applyHeightShift()). Flash. */
bool vzShiftHeight(vzState_t *st, float d);

/* True once this sortie's RAMP has reached its plateau (rampTopRel x kPrior /
 * Vf, the hover thrust of the stored gain), latched until vzInit(). Every
 * logged lift-off came before it (0.76-0.88 x the plateau thrust, 0.39-0.72 s
 * before it, the 13 autonomous take-offs of 2026-10-12/13): a sortie still
 * without a lift-off there may be up undetected (a negative in-flight
 * offset). The glue's take-off integrator hold ends here (autonomy.c). Flash;
 * the caller's task only. */
bool vzRampAtTop(const vzState_t *st);

/* ---- sim plant ----------------------------------------------------------
 * 1-D point mass in thrust-gain units: the rotors produce a lagged thrust
 * Tr (first order, rotorTauS), the battery sags with it, and the specific
 * force in g is
 *   f = Tr * V * (1 + geGain * exp(-h / geHeight)) / kTrue - damp * v
 * so the drone hovers out of ground effect at Tr * V = kTrue and leaves the
 * ground at Tr * V = kTrue / (1 + geGain). In the air a = g * (f - 1); on the
 * ground (h = 0) it stays put until f > 1. The accelerometer reads 1.0 on the
 * ground and f in the air, plus a static offset, an offset while the rotors
 * run (seen by g0), an extra in-flight offset (not seen by g0), white noise
 * and a decaying jolt at touchdown. Defaults reproduce the logged ratio of
 * hover thrust to lift-off thrust (median 1.41, 36 take-offs) at the
 * default hover gain (VZ_K_DEFAULT); see docs/autonomy-a2.md section 4.7.
 *
 * Camera observer model (off by default, camWakeS < 0): the camera wakes
 * camWakeS after the first nonzero command (the take-off window), the
 * observer uses frames from the later of that and the first lift-off, and is
 * healthy camConvS after it, except during an optional loss window. It then
 * reports camVz = v + camVzOffset (the camera's slow velocity offset c) +
 * Gauss-Markov noise (camVzNoise rms, tau 0.1 s) and camBias = accAirG +
 * camBiasErrG (the in-flight offset vs g0, as the observer would learn it).
 * The noise draws from the same xorshift32 state only while the camera is
 * on, so runs without it are unchanged. */
typedef struct {
  float kTrue;            /* VZ_K_DEFAULT */
  float vRest;            /* 3.90 V */
  float vSag;             /* 9e-6 V per thrust unit (0.4 V at 45k) */
  float geGain;           /* 0.33 */
  float geHeight;         /* 0.03 m */
  float rotorTauS;        /* 0.03 */
  float damp;             /* 0.07 g per m/s (rotor inflow damping, momentum theory) */
  float accOffsetG;       /* 0: static sensor offset (always) */
  float accSpinG;         /* 0: extra while the rotors run (thrust > 1000), captured by g0 */
  float accAirG;          /* 0: extra while airborne, not seen by g0 (the dangerous one) */
  float accNoiseG;        /* 0.03 g rms per sample */
  float joltPerMs;        /* 0.5 g per m/s of touchdown speed, decaying with tau 10 ms, at most 4 g */
  float camWakeS;         /* -1: no camera; else s from the first nonzero command to the camera's first frame */
  float camConvS;         /* 0.3 s of frames after the later of the wake and the lift-off: healthy */
  float camVzOffset;      /* 0 m/s: c, positive = the camera reads a climb that is not there */
  float camVzNoise;       /* 0.02 m/s rms */
  float camBiasErrG;      /* 0 g */
  float camLossS;         /* -1: never; else the observer is unhealthy from this long after the first lift-off ... */
  float camLossLenS;      /* ... for this long */
} vzSimConfig_t;

typedef struct {
  float h, v;             /* true height (m) and vertical speed (m/s) */
  float tr;               /* lagged rotor thrust */
  float vbat;             /* V, what the controller reads as the battery */
  float accZ;             /* g, what the controller reads as the accelerometer */
  float jolt;
  bool onGround;
  float hMax;             /* highest true height this run */
  float impactV;          /* fastest touchdown speed, m/s (positive) */
  float liftT;            /* sim time of the first true lift-off, -1 before */
  float t;                /* sim time, s */
  uint32_t rng;           /* xorshift32 state, never 0 */
  /* camera observer model (all false/0 without the camera) */
  float cmdT;             /* sim time of the first nonzero command, -1 before */
  float camN;             /* noise state, m/s */
  bool camAwake;          /* the camera delivers frames (running, fresh view) */
  bool camHealthy, camVzValid;
  float camVz, camBias;   /* what the observer reports, as vzInput_t takes it */
} vzSimState_t;

void vzSimDefaultConfig(vzSimConfig_t *c);
/* On the ground, rotors stopped, vbat = vRest, accZ = 1 + accOffsetG. */
void vzSimInit(vzSimState_t *s, const vzSimConfig_t *c, uint32_t seed);
/* Advances by dt with the commanded thrust and updates vbat and accZ. */
void vzSimStep(vzSimState_t *s, const vzSimConfig_t *c, float thrustCmd, float dt);

const char *vzPhaseName(vzPhase_t p);
const char *vzReasonName(vzReason_t r);

#endif /* VERT_CTRL_H_ */
