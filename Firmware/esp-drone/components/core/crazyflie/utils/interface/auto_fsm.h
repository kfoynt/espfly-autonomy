/*
 * ESP-Drone Firmware
 *
 * auto_fsm.h - Autonomy state machine: arming guards, take-off, hover with
 * photos, landing and kill (autonomy stage A2, docs/autonomy-a2.md section 3)
 *
 * States: IDLE (autonomy off, the phone app flies as before), ARMED (motors
 * held at 0, waiting for TAKEOFF), TAKEOFF (1 s pre-spin warning, then the
 * vertical controller from idle spin to HOLD), HOVER (STABLE for 1 s, the
 * photos, then hovering in place waiting for the Mac's next command for at
 * most hoverWaitMs; in that wait the interactive hover flies one MOVE, YAW,
 * STOP or SET_HEIGHT correction at a time, auto_motion.h (SET_HEIGHT: "you are at
 * X m", docs/autonomy-a2b.md 23), and a TAKEOFF's planned move starts as
 * a MOVE when the wait starts, docs/autonomy-a2b.md 21.4), LAND (the vertical controller's
 * estimate-independent landing, motors off on touchdown) and KILLED (motors
 * cut, latched for at least 2 s).
 *
 * Kill semantics (user decisions, 2026-10-11):
 *   - any phone-app stick input while autonomous (thrust >= 1000 or |roll| or
 *     |pitch| >= 2 deg; yaw ignored) stops autonomous movement and lands
 *     slowly in place; the phone never cuts the motors outright;
 *   - the Mac KILL and the automatic trips (tilt, rate, free fall, tracking
 *     error, non-finite input, motor watchdog) cut the motors at once;
 *   - Mac heartbeat older than 1 s, phone silent > 500 ms (when required),
 *     a new Mac session, the battery rules and three loop gaps within 1 s
 *     land; so do the vertical controller's physical fallbacks;
 *   - "Claude decides" (user decisions 2026-10-14 ~20:30-20:40,
 *     docs/autonomy-a2b.md 22): nothing else lands. What used to give up
 *     (HOVER_NO_CAM, NOT_STABLE, HOVER_TIMEOUT, the air clock, CAPTURE_FAIL,
 *     the motion guards, an in-air contact in the hover, the palm cue, a
 *     single loop gap) now holds and reports an event (autoFsmReport_t,
 *     AUTO_STATUS bytes 96-135) and waits for Claude. The two physical
 *     fallbacks of the state machine: the camera height out for
 *     AUTO_CAM_LOST_MAX_MS (or at once without a camera-measured hover bound)
 *     sinks to the ground (LAND SINK, resumable when the camera returns), and
 *     an in-air contact in the climb without the camera height lands blind
 *     (LAND COLLISION, as since 2026-10-13); every trip above still cuts.
 *     After any contact (CONTACT or COLLISION) every later LAND of the
 *     sortie, whatever its reason, is the blind one (autoFsmOut_t.blind;
 *     user decision (B) 2026-10-15); the hold itself keeps the camera.
 *
 * Airborne time is counted from the first step the controller is in RAMP
 * (the first thrust above the idle spin), not from the detected lift-off,
 * which can lag the true one by more than 1 s.
 *
 * One autoFsmStep() per stabilizer loop (1 kHz). The state machine never
 * touches hardware: it reads one autoFsmIn_t (filled by the glue in
 * modules/src/autonomy.c from the sensors, the link mailbox, the phone hook
 * and the previous step's vzOutput_t) and writes one autoFsmOut_t that says
 * what to do with the thrust, which command to give the vertical controller,
 * and which one-shot actions to take. Every guard is written so that a
 * non-finite input fails it. Pure code, host-tested by
 * tools/tests/test_auto_fsm.c.
 */
#ifndef AUTO_FSM_H_
#define AUTO_FSM_H_

#include <stdbool.h>
#include <stdint.h>
#include "camera_cadence.h"

#include "auto_motion.h"  /* the interactive hover's motions (MOVE, YAW, STOP) */
#include "auto_proto.h"   /* autoResult_t for command results */
#include "vert_ctrl.h"

/* ---- timing and thresholds (docs/autonomy-a2.md section 3.7) ---------- */
#define AUTO_ARM_TIMEOUT_MS        10000   /* ARMED -> IDLE without TAKEOFF */
#define AUTO_PRESPIN_MS            1000    /* LED warning and stillness re-check before any motor turns */
#define AUTO_HB_ARM_MAX_MS         300     /* G0: three 10 Hz heartbeats */
#define AUTO_HB_LAND_MS            1000    /* heartbeat older than this: LAND (ARMED: IDLE) */
#define AUTO_APP_ARM_MAX_AGE_MS    500     /* G5: an app packet this fresh ... */
#define AUTO_APP_CENTRED_MS        1000    /* ... and no app input for this long */
#define AUTO_APP_SILENT_MS         500     /* app required and silent this long: LAND (per-boot app gaps reach 319 ms) */
#define AUTO_GESTURE_WINDOW_MS     15000   /* G10: roll push >= 10 deg released within this */
#define AUTO_ONGROUND_MIN_MS       3000    /* G2: zero thrust this long (gyro re-zero ran) */
#define AUTO_SETTLE_MIN_MS         20000   /* G2 in FLIGHT: zero thrust this long since the last flight. The
                                            * accelerometer heats up after landing (heat soak, ~0.5 mg per
                                            * motor-second, saturating near +40 mg); take-offs within 20 s of
                                            * the last flight had in-flight offsets vs g0 of -5.6..+7.9 mg
                                            * (sd 2.8, n = 24), later ones -0.4..+3.6 mg (sd 1.1, n = 14) */
/* Heat soak (G2 in FLIGHT; 2026-10-13, needs the user's sign-off: it
 * lengthens the user's 20 s settle after longer flights). The accelerometer
 * keeps warming for 10-80 s after a landing, so its offset moves in the
 * next flight: 2026-10-13 flight 2 took off 60 s after a 41 s flight with a
 * ground baseline 16.5 mg above flight 1's and an offset that moved 5.8 mg
 * in the air (normal +0.3 +/- 1.8 mg), which put the camera height
 * observer's prior on the alarming side (docs/autonomy-a2.md 4.11). FLIGHT
 * waits min(AUTO_SETTLE_MIN_MS + AUTO_SOAK_PER_MOTOR x the last flight's
 * motor time, AUTO_SOAK_MAX_MS) on the ground: 81.5 s after a 41 s flight,
 * 42.5 s after 15 s, 27.5 s after a 5 s hop, 20 s for the first flight of a
 * boot. The refusal is G2 STILL (the Mac's daemon tells the soak from its
 * last sortie: espfly_auto.py soak_hint). */
#define AUTO_SOAK_PER_MOTOR        1.5f
#define AUTO_SOAK_MAX_MS           90000
/* Camera awake through every autonomous take-off (user decision 2026-10-12;
 * A2a kept it awake only at >= 4.00 V): out.camNoStandby in every FLIGHT and
 * BENCH sortie, ARMED..HOVER. The camera drift hold and the camera height
 * observer need frames from the lift-off on; with the standby they began
 * only ~0.6 s (warm) or 1.0-1.2 s (cold) after it. The supply margin that
 * the standby protected was bought back by G3 (AUTO_VMIN_DEFAULT, 4.00 V
 * until the user lowered it to 3.80 V the same evening). */
#define AUTO_STILL_ACC_G           0.05f   /* G2: | |acc| - 1 g | below */
#define AUTO_STILL_GYRO_DPS        3.0f    /* G2: every gyro axis below */
#define AUTO_STILL_TILT_DEG        8.0f    /* G2 and ARMED/pre-spin stillness */
#define AUTO_MOVED_GYRO_DPS        10.0f   /* ARMED/pre-spin: any axis above this, or tilt >= AUTO_STILL_TILT_DEG, ... */
#define AUTO_MOVED_ACC_G           0.10f   /* ... or | |acc| - 1 g | above this ... */
#define AUTO_MOVED_MS              100     /* ... for this long: MOVED */
#define AUTO_VMIN_DEFAULT          3.80f   /* G3 resting battery (NVS vmin). User decision 2026-10-12 (evening),
                                            * accepting the brown-out risk: 4.00 V was chosen because manual
                                            * full-throttle take-offs with the camera awake browned out at
                                            * <= 3.87 V (none at 3.91-3.98 V, n = 2), and autonomous hops from
                                            * 3.93-3.96 V took the 3.3 V rail to 2.82-2.87 V */
#define AUTO_VMIN_LO               3.30f   /* allowed NVS range (user decision 2026-10-13: was 3.70) */
#define AUTO_VMIN_HI               4.20f
#define AUTO_FPS_MIN               4.5f    /* G4: user-approved native VGA ~5.55 fps, 2026-10-21. */
#define AUTO_GCAP_BENCH_MAX        20000   /* G6: BENCH needs 1..20000 */
#define AUTO_BENCH_THRUST_MAX      15000   /* BENCH thrust clamp: min(gcap at ARM, this), only ever lowered */
#define AUTO_HEAP_MIN              12288   /* G9: internal heap free, bytes */
#define AUTO_KILL_LATCH_MS         2000    /* G11: no ARM this long after a Mac KILL */

#define AUTO_HEIGHT_CM_MIN         20      /* TAKEOFF heightCm: user decision 0.20..0.40 m, raised to */
#define AUTO_HEIGHT_CM_MAX         120     /* 0.60 m by the user on 2026-10-12 (evening), to 1.20 m the same night
                                              (late: the interactive hover, docs/autonomy-a2b.md section 14).
                                              2026-10-15: the hover goes to 2.00 m (VZ_H_TARGET_MAX) by MOVEs up only;
                                              the take-off stays at 1.20 m (vert_ctrl's VZ_H_TAKEOFF_MAX: a LAND in
                                              the climb falls from landStart at once, 1.8-2.05 m/s from a 2.00 m
                                              climb in the sim; a2b 23.9) */
_Static_assert(AUTO_HEIGHT_CM_MAX == (int)(VZ_H_TAKEOFF_MAX * 100.0f + 0.5f), "the take-off cap is the controller's");
_Static_assert(AUTO_MV_H_MAX_M == VZ_H_TARGET_MAX, "the moves' cap is the controller's");
_Static_assert(VZ_H_TAKEOFF_MAX <= VZ_H_TARGET_MAX, "the take-off within the hover range");
/* Above 0.60 m the climb takes longer (vClimbMax 0.4 m/s): the pre-wait air
 * limit (airMaxMs) and the TAKEOFF backstop grow by this per cm above 60
 * (1.5 s at 1.20 m, the take-off cap: the default pre-wait limit 7.5 s; a
 * 1.20 m climb reaches the wait ~1.5 s later than a 0.60 m one in the sim,
 * glue host test margin-120). 1 cm at 0.40 m/s = 25 ms. */
#define AUTO_AIR_PER_CM_ABOVE_60_MS 25
#define AUTO_PHOTO_N_MAX           3
#define AUTO_PHOTO_GAP_MIN_MS      150
#define AUTO_PHOTO_GAP_MAX_MS      1000
/* Airborne time: the air clock runs from the first step the controller is in
 * RAMP. Since 2026-10-15 ("Claude decides", docs/autonomy-a2b.md 22) it lands
 * nothing: airMaxMs (+ the climb's extra) is when a hover that has not started
 * its wait starts it anyway (TAKEOFF_SLOW), the hover wait is information only
 * (no HOVER_TIMEOUT), and the battery ends every flight (64-68 s of air a
 * flight, ~177 s a charge measured). History, until 2026-10-15: until the
 * hover wait started the clock could reach airMaxMs; then the wait had
 * hoverWaitMs; and the air clock never passed AUTO_AIR_HARD_MS, wait
 * included. User decision 2026-10-12 (evening): fly at up to 0.60 m with a
 * 10 s wait (A2b: 0.40 m, 5 s wait, hard 10 s, wait at most 8 s, pre-wait
 * 4500 = the old 3.5 s from lift-off plus the ~1 s ramp, at most 6000).
 * The pre-wait default covers a 0.60 m sortie with >= 1 s to spare: the
 * logged 0.32-0.40 m sorties reached HOVER 2.25-2.49 s after the first RAMP
 * step and their wait 1.62-1.83 s later (1 photo); the climb to 0.60 m takes
 * ~0.45 s more (sim), so the wait starts at ~4.3-4.8 s (sim, realistic range:
 * max 4.83 s). The highest NVS value adds 2 s for more take-off photos.
 * The hard limit: 120 s (user decision 2026-10-14 ~01:40; 80 s for the
 * interactive hover's 60 s wait, 18 s before it, 10 s in A2b). Without a
 * height sensor the true height after a long hover is uncertain by about
 * 1-2 m (the controller's estimate at touchdown drifted -0.011..+0.031 m/s on
 * the logged hovers longer than 20 s, RMS 0.020 m/s; docs/autonomy-a2b.md
 * 18.1): a drift down ends in a slow LAND (the floor rules), a drift up of
 * the worst logged rate climbs ~1.3 m in 115 s. The controller's HOLD
 * backstop follows it (VZ_HOLD_MAX_LIMIT_S, holdMaxS 1 s under it), and the
 * LAND after a long HOLD keeps its flat part longer by the drift it may hide
 * (VZ_LAND_DRIFT_MAX_M 2.0 m: up to +10 s). The battery usually ends a flight
 * first (BATT_LOW; ~86 s of air per charge measured). */
#define AUTO_AIR_MAX_DEFAULT_MS    6000    /* air clock until the hover wait starts anyway: TAKEOFF_SLOW (NVS autoair) */
#define AUTO_AIR_MAX_MIN_MS        2000
#define AUTO_AIR_MAX_HI_MS         8000    /* the highest pre-wait limit NVS or ARM may give */
#define AUTO_HOVER_WAIT_DEFAULT_MS 5000    /* NVS autowait: information only since 2026-10-15 (the Mac's --wait-s; no
                                              HOVER_TIMEOUT); the wire ranges below are unchanged */
#define AUTO_HOVER_WAIT_MAX_MS     120000  /* user decision 2026-10-14 (~01:50). uint32_t ms everywhere in the firmware,
                                              u24 on the wire (TAKEOFF), u32 in NVS (autowait; docs/autonomy-a2b.md 18.2) */
#define AUTO_HOVER_WAIT_TAKEOFF_MIN_MS 1000 /* TAKEOFF actWaitMs, when not 0 (= NVS autowait): this..MAX */
#define AUTO_HOVER_ACCEL_DEFAULT_MS 2000   /* FLIGHT: HOVER this long on the accelerometer alone, then the camera rule runs
                                              (NVS autoaccel) */
#define AUTO_HOVER_ACCEL_MAX_MS    3000    /* below the controller's holdNoCamS backstop */
/* The camera rule (FLIGHT, HOVER from hoverAccelMs on; "Claude decides", docs/autonomy-a2b.md 22): the camera height
 * estimate out of use (vzOutput_t camHealthy false) AUTO_CAM_LOST_REPORT_MS: the event CAM_LOST (detail camBits); in
 * use again AUTO_CAM_BACK_MS: CAM_BACK (a shorter return keeps the budget running). With a camera-measured hover
 * bound (vzOutput_t capQ > 0) the controller holds open loop under it, so it can sink but not climb, for
 * AUTO_CAM_LOST_MAX_MS from the later of the last step in use and the HOVER entry, then SINK: LAND SINK, the
 * controller's CMD_LAND (from 1.2 m its flat part), resumable while it lasts. Without one (the camera not yet in use
 * for a quiet second of HOLD, or a HOLD a climb fault or a contact started) the SINK comes at once: the open loop has
 * no bound (today's HOVER_NO_CAM timing). */
#define AUTO_CAM_LOST_REPORT_MS    300
#define AUTO_CAM_LOST_MAX_MS       5000
#define AUTO_CAM_BACK_MS           500     /* also the SINK's resume: the camera height in use this long */
_Static_assert(AUTO_CAM_LOST_MAX_MS + 1000 <= (uint32_t)(VZ_HOLD_NO_CAM_DEFAULT_S * 1000.0f),
               "the FSM's SINK comes at least 1 s before the controller's HOLD_NO_CAM backstop");
_Static_assert(VZ_HOLD_MAX_LIMIT_S >= 600.0f, "no air clock: the controller's HOLD backstop is beyond any battery");

#define AUTO_STABLE_MS             1000    /* STABLE this long, then the photos */
#define AUTO_STABLE_TILT_DEG       4.0f
#define AUTO_STABLE_GXY_DPS        30.0f
#define AUTO_STABLE_GZ_DPS         10.0f
#ifdef ESPFLY_NATIVE_VGA
#define AUTO_STABLE_FRAME_AGE_MS CAM_CADENCE_VGA_MAX_AGE_MS
#else
#define AUTO_STABLE_FRAME_AGE_MS   300     /* camera frame at most this old: ~120-130 ms latency + up to ~90 ms
                                            * between frames (hops of 2026-10-12: mean 149, max 197 at 100 Hz) */
#endif
/* A STABLE streak survives a dropout of its conditions shorter than this
 * (start kept, stableMs counts on); a longer one, or any step beyond a hard
 * bound below, ends it at once. The 2026-10-12 evening hovers lost 1 s
 * streaks to 75 ms hold coasts and single 10.2 dps yaw samples. */
#define AUTO_STABLE_GRACE_MS       150
/* ... and its dropouts together stay below this: the step they reach it
 * ends the streak, so STABLE 1 s means the conditions held >= 70 % of it and
 * a lasting violation that recovers for a step now and then never passes.
 * The 2026-10-12 hovers that reach STABLE 1 s spend 0-150 ms of it in
 * dropouts. */
#define AUTO_STABLE_DROP_BUDGET_MS 300
#define AUTO_STABLE_HARD_TILT_DEG  6.0f    /* 1.5 x AUTO_STABLE_TILT_DEG */
#define AUTO_STABLE_HARD_GXY_DPS   60.0f   /* 2 x AUTO_STABLE_GXY_DPS */
#define AUTO_STABLE_HARD_GZ_DPS    30.0f   /* 3 x AUTO_STABLE_GZ_DPS; also hard: camera not RUNNING (FLIGHT), vz IDLE */
#ifdef ESPFLY_NATIVE_VGA
#define AUTO_VIEW_FRAME_AGE_MS CAM_CADENCE_VGA_MAX_AGE_MS
#else
#define AUTO_VIEW_FRAME_AGE_MS     200     /* the glue's camViewMs counts frames this fresh (NOT_STABLE's view, unchanged) */
#endif
#define AUTO_NOT_STABLE_HOLD_MS    2000    /* no STABLE 1 s by HOLD + this ... */
#define AUTO_NOT_STABLE_CAM_MS     1500    /* ... once the camera has had a fresh view this long: the event NOT_STABLE and
                                              the wait starts (a LAND until 2026-10-15) */
#define AUTO_VIEW_LOST_REPORT_MS   500     /* HOVER without a motion: the hold not applied on a fresh frame this long:
                                              the event HOLD_NO_VIEW (its back: VIEW_BACK) */
/* DONE follows the last frame's asynchronous JPEG encode, not just its raw copy.
 * Native VGA encodes sequentially on the low-priority worker (observed 3-6 s/frame).
 * Budget each frame, including the first; this never blocks the control task.
 * Transfer to the host has its own deadline and is not part of capture completion. */
#if ESPFLY_NATIVE_VGA
#define AUTO_CAPTURE_FRAME_MS      10000u
#else
#define AUTO_CAPTURE_FRAME_MS      0u
#endif
#define AUTO_CAPTURE_EXTRA_MS      600u
#define AUTO_CAPTURE_TIMEOUT_MS(n, gap) \
  ((uint32_t)(n) * ((uint32_t)(gap) + AUTO_CAPTURE_FRAME_MS) + AUTO_CAPTURE_EXTRA_MS)
#define AUTO_TAKEOFF_MAX_MS        8000    /* TAKEOFF state backstop (pre-spin included; 1 + 0.8 + ramp <= 2.5 +
                                              climb <= 2.5 at 0.60 m = 6.8 s; above autoair ~6.2 s it lands a
                                              take-off stuck in RAMP before the air clock) */

#define AUTO_VBAT_LAND_V           2.60f   /* 10 Hz reading below this ... (user decision 2026-10-13: was 3.05 V;
                                            * the 1 kHz fast-sag LAND at 2.55 V is the brown-out edge guard).
                                            * Any airborne state; the TAKEOFF guard: in HOVER BATT_LOW (below)
                                            * always fires first */
#define AUTO_VBAT_LAND_MS          1000    /* ... this long: LAND */
/* End of battery in HOVER: LAND BATT_LOW (user decision 2026-10-13, late;
 * docs/autonomy-a2b.md "End of battery (2026-10-14)"). One persistence
 * tracker (battLow), two triggers; either one held on every step for
 * AUTO_VBAT_LOW_LAND_MS lands the slow way (the controller's CMD_LAND from
 * HOLD, T_ref = T_hold, ceiling 0.98 x T_ref):
 *   (1) every 10 Hz battery reading (pmGetBatteryVoltage: a 30-sample ADC
 *       mean every 100 ms) below AUTO_VBAT_LOW_LAND_V. They scatter with an
 *       sd of 29-39 mV around the loaded voltage, so 5-6 in a row below
 *       3.00 V mean a loaded mean of about 2.95-2.97 V;
 *   (2) the controller's T_hold (vzOutput_t.tHold, the 0.25 s average of the
 *       applied thrust, 0 before HOLD) above AUTO_HEADROOM_LAND_T = the
 *       ceiling - 1000 = 57000: 2.89-3.0 V loaded at the measured K of
 *       165-171k. The floor ends the low-K flights, the headroom the high-K
 *       ones before the ceiling.
 * The end must be this slow LAND, never the controller's HOLD_CAPPED (0.5 s
 * at the ceiling: T_ref measured at the cap, a fast LAND). Replayed on the
 * real hover thrust noise (5 flights, sd 170-334) rising toward the 58000
 * cap: 1000 counts + 0.5 s land first in 100 % of the cases up to a rise of
 * 1000 counts/s (~43 mV/s), 99 % at 2000; 1.0 s persistence only 82 % at
 * 1000 counts/s, a 600-count margin 70 % at 2000. With the 54000 ceiling the
 * flights ended HOLD_CAPPED at ~3.06-3.12 V loaded. Below the new end: the
 * deepest 40 kHz in-flight survival is 2.838 V (10 Hz), the 15 kHz in-flight
 * resets were at 2.72-2.75 V, the 1 kHz minimum at the LAND start is
 * expected at ~2.75-2.86 V, above AUTO_VBAT_FAST_LAND_V and the brown-out
 * detector (level 7, ~2.44 V). The 3.3 V rail estimate is no trigger (one
 * ADC conversion at <= 1 kHz, saturates at ~3.16-3.27 V, misses sub-ms dips;
 * its minima before the logged brown-outs overlap flights that did not
 * reset): a post-flight check only. BATT_LOW is HOVER only; VBAT_LOW guards
 * TAKEOFF. History: 3.30 V for 1 s (2026-10-12, late), 3.10 V, 2.65 V
 * (2026-10-13: limits removed). */
#define AUTO_VBAT_LOW_LAND_V       3.00f
#define AUTO_VBAT_LOW_LAND_MS      500
#define AUTO_HEADROOM_LAND_T       (VZ_THRUST_ABS_MAX - 1000.0f)   /* 57000: trigger (2), T_hold above it */
/* A new MOVE or YAW is refused (AUTO_MVW_BATT, reply LOW_BATT) when T_hold is
 * above AUTO_MV_REFUSE_T (checked once, when the command arrives: below it a
 * MOVE up or bob (+4 %), a lean and the settle's loop swing stay under the
 * 57000 trigger; in the drain sim a 55100 level let moves started near the
 * end hit HOLD_CAPPED, impact up to 1.4 m/s, 53400 removed every case), or
 * once the BATT_LOW condition has held AUTO_VBAT_LOW_REFUSE_MS. STOP, PHOTO
 * and LAND are never refused; a running motion ends with the LAND itself. */
#define AUTO_MV_REFUSE_T           (VZ_THRUST_ABS_MAX - 4600.0f)   /* 53400 */
/* Upward motions on thrust headroom, never on the battery voltage (user 2026-10-16: "you could have used more thrust";
 * docs/autonomy-a2b.md 27.3): a MOVE up or a SET_HEIGHT whose flight goes up is refused (AUTO_MVW_UP_HEADROOM, reply
 * LOW_BATT) when the controller's ceiling in force (vzOutput_t.ceiling: 58000, the battery bound capLo x K_lo, the
 * model's capKhat x Khat) is less than min(AUTO_MV_UP_HEADROOM_T, AUTO_MV_UP_HEADROOM_REL x T_hold) above T_hold. The
 * logged 0.15 m/s climbs needed at most +2.2k (4.3 %) over T_hold (F2 +0.42 m at T_hold 51.7k, peak 53.9k; 10-15
 * fastcheck +0.13 m +1.5k), the hold loop's thrust noise sd 170-334 (3 sd ~1k): T_hold + 3.2k must stay under the
 * 57000 BATT_LOW trigger, which 4600 keeps (with the 58000 ceiling exactly AUTO_MV_REFUSE_T, so nothing the build
 * before accepted is refused there). The relative floor keeps the soft model ceiling (1.15 x the hover, a slower
 * climb, never a LAND) from refusing a climb off a support (T_hold 30-41k). Checked when the command arrives, before
 * the battery-end rule (BATT), so a refused climb names the thrust; a descent keeps the rules it had. */
#define AUTO_MV_UP_HEADROOM_T      4600.0f
#define AUTO_MV_UP_HEADROOM_REL    0.10f
_Static_assert(VZ_THRUST_ABS_MAX - AUTO_MV_UP_HEADROOM_T == AUTO_MV_REFUSE_T,
               "at the 58000 ceiling the upward headroom rule is the MOVE refusal level");
/* vert_ctrl's fast LAND stays off from this hover need up (docs/autonomy-a2b.md 17.4). */
_Static_assert(AUTO_MV_REFUSE_T == VZ_LAND_FAST_TREF_MAX, "the fast LAND's T_ref limit is the MOVE refusal level");
#define AUTO_VBAT_LOW_REFUSE_MS    250
#define AUTO_VBAT_FAST_LAND_V      2.55f   /* 1 kHz minimum (last 320 ms) below this: LAND (good flights dipped to 2.62 V) */
#define AUTO_LOOP_GAP_LAND_US      20000   /* a stabilizer loop gap above this: the event LOOP_GAP, a running motion
                                              stopped; AUTO_LOOP_GAP_N of them within AUTO_LOOP_GAP_WIN_MS: LAND (control
                                              cannot be held; one gap landed until 2026-10-15) */
#define AUTO_LOOP_GAP_N            3
#define AUTO_LOOP_GAP_WIN_MS       1000

#define AUTO_TILT_KILL_DEG         30.0f   /* trips (user decisions) */
#define AUTO_TILT_KILL_MS          50
#define AUTO_RATE_KILL_DPS         500.0f  /* |gx| or |gy| */
#define AUTO_RATE_KILL_MS          30
#define AUTO_FALL_KILL_G           0.4f    /* |acc| below */
#define AUTO_FALL_KILL_MS          100
#define AUTO_ATTERR_KILL_DEG       10.0f
#define AUTO_ATTERR_KILL_MS        500
/* In-air contact (LAND COLLISION; 2026-10-13 flight 1: the drone flew into
 * something 34.45 s into an interactive hover; its first contact, 1.79 ->
 * 3.98 g and 214 -> 492 dps, passed every trip above and the motors ran on
 * through five contacts for 0.86 s until the vertical controller's K_RANGE
 * LAND). Armed in TAKEOFF CLIMB/HOLD and HOVER (the controller in CLIMB or
 * HOLD) from AUTO_COLLIDE_ARM_MS after the controller's lift-off, in FLIGHT
 * and DRY (a finger tap tests it on the bench), never in BENCH (board
 * vibration) nor in LAND (touchdowns: 1.3-8.8 g, up to 828 dps; LAND keeps
 * its own touchdown rules), so it fires at most once a sortie. The recorder's
 * CLIMB/HOLD from lift-off + 0.2 s of the ten logged autonomy sorties of
 * 2026-10-12/13 (flight 1 before the contact included) reach 1.16 g and
 * 77 dps (an external brush 12.71 s into the flight 1 hover); their lift-off
 * windows 1.20 g and 63 dps. Yaw is excluded (commanded turns). 98cd4ae cut
 * the motors on it (KILLED); the user's decision of 2026-10-13 lands instead
 * (autoFsmOut_t.collided, docs/autonomy-a2.md section 12.8): KILL from 1 m
 * hits the floor at ~4.4 m/s, the blind LAND from 1.0-1.2 m at <= 1.0 m/s
 * in the sim. */
#define AUTO_COLLIDE_ACC_G         1.6f    /* |acc| above this ... */
#define AUTO_COLLIDE_DPS           250.0f  /* ... or max(|gx|, |gy|) above this ... */
#define AUTO_COLLIDE_MS            3       /* ... held this long: a contact */
#define AUTO_COLLIDE_ARM_MS        200     /* armed this long after the controller's lift-off (vzOutput_t tAir) */
/* "Claude decides" (2026-10-15, docs/autonomy-a2b.md 22): a contact in HOLD, or in the climb with the camera height in
 * use, is CONTACT: a hush of AUTO_CONTACT_HUSH_MS (vzInput_t.hush: the estimate frozen, the hover thrust), a running
 * motion stopped (AUTO_MVE_CONTACT: a snap and a brake) or a snap epoch without one, the event with its peaks and the
 * body-frame force (the obstacle's side), no LAND. At most two hushes start in any AUTO_CONTACT_CHAIN_MS (a prop
 * rubbing a wall must not freeze the estimate for good): a contact the bound refuses is counted and reported (detail
 * AUTO_CONTACT_NO_HUSH) only. Until the camera height has been in use AUTO_CAM_BACK_MS after a hush (after its open
 * window), a LAND also blocks the camera hold (collided: the contact's window). A contact in the climb without the camera
 * height lands blind (LAND COLLISION): there the hover thrust is not measured and the estimate may read low. Every LAND
 * of a sortie that had a contact is the blind one (autoFsmOut_t.blind, user decision (B) 2026-10-15: ~1 m/s touchdowns
 * whatever the camera says; until then only those in the window). */
#define AUTO_CONTACT_HUSH_MS       500
#define AUTO_CONTACT_CHAIN_MS      2000
/* After a hush the controller does not use the camera estimate for VZ_HUSH_OPEN_S (vert_ctrl.h: a jolt's frames may
 * read a speed that is not there): its camHealthy is false for that long with vzOutput_t camOpen set while the
 * observer itself is healthy, which the camera rule does not count as a loss (a loss that began before counts on). */
#define AUTO_CONTACT_NO_HUSH       0x40    /* CONTACT detail bit: no hush (the chain bound) */
#define AUTO_CONTACT_MOTION        0x80    /* CONTACT detail bit: a running motion was stopped */
/* The palm catch (2026-10-14, docs/autonomy-a2b.md 18.3; AUTO_PALM_*) was removed on 2026-10-15 (the user no longer
 * needs the hand catch; docs/autonomy-a2b.md 22): its reason PALM stays on the wire, never produced. */

/* In LAND a tilt or rate trip with the touchdown thrust condition counts as
 * touchdown only after contact: |acc| above AUTO_TD_CONTACT_G or the
 * controller's aRel above AUTO_TD_JOLT_G within the last
 * AUTO_TD_CONTACT_WIN_MS (logged touchdowns under power: 1.8-3.1 g). A free
 * fall or a tracking trip is never a touchdown. */
#define AUTO_TD_CONTACT_G          1.5f
#define AUTO_TD_JOLT_G             0.6f    /* the controller's own jolt rule (tdJoltG) */
#define AUTO_TD_CONTACT_WIN_MS     300

#define AUTO_LAND_BACKSTOP_MS      6500    /* LAND state with the controller still landing: KILLED (from a reference
                                              height above 0.60 m: + the extra height at 0.30 m/s, as vert_ctrl's
                                              LAND timeout: 8500 ms from 1.20 m, 11167 ms from 2.00 m; after a
                                              move up at least the take-off's extra + the move at 0.30 m/s: 9833 ms
                                              for 0.20 + 1.00 m;
                                              after a HOVER longer than VZ_LAND_DRIFT_AFTER_S + its drift allowance,
                                              min(VZ_LAND_DRIFT_MAX_M, 0.017 m/s x HOVER) at 0.20 m/s: up to +10 s,
                                              18.5 s after a 118 s HOVER at 1.20 m) */
#define AUTO_LAND_REST_MS          1000    /* after the controller ended: at rest this long ... */
#define AUTO_LAND_REST_MAX_MS      3000    /* ... or this long in any case: IDLE */
#define AUTO_KILLED_MIN_MS         2000
#define AUTO_KILLED_REST_MS        1000
#define AUTO_REST_GYRO_DPS         10.0f   /* "at rest" (same test as the stabilizer's soft-start) */
#define AUTO_REST_ACC_G            0.15f
#define AUTO_RELEASE_HOLD_MS       600     /* IDLE keeps thrust 0 this long after a hand-back (the glue also drops the
                                            * commander's queued app setpoint, which keeps its thrust up to 2000 ms) */

typedef enum {
  AUTO_IDLE = 0,          /* values go on the wire (STATUS byte 7) */
  AUTO_ARMED = 1,
  AUTO_TAKEOFF = 2,
  AUTO_HOVER = 3,
  AUTO_LAND = 4,
  AUTO_KILLED = 5,
} autoState_t;

typedef enum {            /* ARM payload byte 0 */
  AUTO_MODE_FLIGHT = 0,   /* real thrust, vertical controller on the real IMU */
  AUTO_MODE_DRY = 1,      /* thrust forced to 0; vertical controller on the sim plant */
  AUTO_MODE_BENCH = 2,    /* real thrust clamped to min(gcap, 15000) every loop, props off */
} autoMode_t;

/* What the glue does with setpoint.thrust this loop. */
typedef enum {
  AUTO_THR_NONE = 0,      /* autonomy inactive: the setpoint is not touched */
  AUTO_THR_ZERO,          /* full autonomy setpoint with thrust 0 (ARMED, pre-spin, after touchdown, release hold) */
  AUTO_THR_VERT_CTRL,     /* thrust = vzOutput_t.thrust with VZ_CMD_TAKEOFF (TAKEOFF, HOVER) */
  AUTO_THR_VERT_LAND,     /* thrust = vzOutput_t.thrust with VZ_CMD_LAND (LAND) */
  AUTO_THR_CUT,           /* thrust 0 now, emergency (KILLED) */
} autoThrust_t;

typedef enum {
  AUTO_CMD_NONE = 0,
  AUTO_CMD_ARM,
  AUTO_CMD_TAKEOFF,
  AUTO_CMD_LAND,
  AUTO_CMD_KILL,          /* also counted in killCount by the link, so it is never lost */
  AUTO_CMD_DISARM,
  AUTO_CMD_PHOTO,         /* HOVER, while waiting for a command: another photo set */
  AUTO_CMD_MOVE,          /* HOVER waiting: move right/forward/up (auto_motion.h) */
  AUTO_CMD_YAW,           /* HOVER waiting: turn (+ = clockwise, right) */
  AUTO_CMD_STOP,          /* end any motion now, hold where it is; never refused */
  AUTO_CMD_SET_HEIGHT,    /* HOVER waiting: "you are at X m" (2026-10-15, docs/autonomy-a2b.md 23): the height estimate
                             corrected, the difference flown back as a vertical correction (AUTO_SH_*) */
} autoCmdType_t;
/* MOVE..SET_HEIGHT go to motionCmd() (stepAirborne's one range test, IRAM): append-only and consecutive. */
_Static_assert(AUTO_CMD_SET_HEIGHT == AUTO_CMD_STOP + 1 && AUTO_CMD_STOP == AUTO_CMD_MOVE + 2,
               "the motion commands MOVE, YAW, STOP, SET_HEIGHT are consecutive");

/* SET_HEIGHT ("you are at X m", user decision 2026-10-15, docs/autonomy-a2b.md 23). Claude judges the true height X from
 * a photo and sends it with the drone's own estimate h_at when that photo was captured (the offset form: d = X - h_at,
 * so moves between the photo and the command cancel out; h_at AUTO_SH_AT_NOW: d = X - h_now). HOLD has no height loop
 * (a speed loop on the camera's speed), so the estimate alone would change nothing: on acceptance the estimate shifts
 * by d (vert_ctrl.h vzShiftHeight(), the glue on the same step), the committed height offset by d (the reference moves
 * into the corrected frame: hMax + hOff keeps its margin over h), and a vertical-only MOVE (mvFlags
 * AUTO_MVF_SET_HEIGHT) flies upM = min(h_now, AUTO_MV_H_MAX_M) - (h_now + d) at 0.15 m/s back to the height the drone
 * believed it held, now truly there. Generic bounds only (no camera or room logic): HOVER in the wait with the
 * controller in HOLD, no motion running, no contact window, the camera height in use and the hold's view, at most one
 * accepted per AUTO_SH_GAP_MS, |d| <= AUTO_SH_MAX_M, the new estimate within AUTO_SH_H_MIN_M..VZ_H_TARGET_MAX +
 * AUTO_SH_MAX_M, and every correction refused near the end of the battery (as MOVE: a descent's braking there can end
 * HOLD_CAPPED, 2026-10-15 review). Every minimum rule (BATT_LOW, the
 * cuts, HB_LOST, k) is untouched. No event (the trim learner skips sorties with one: att_trim_auto.c).
 * The flaw of that form (2026-10-16, docs/autonomy-a2b.md 27): HOLD's estimate sits ~0.1 m below the commanded height
 * (the climb ends HOLD at hTarget - holdBand and brakes), so flying back to it left the drone ~0.1 m low after every
 * correction. The "go to the target" form (AUTO_SH_FORM_TO, the 6-byte SET_HEIGHT): the same shift by d (the true
 * height X now), then the flight upM = target - (h_now + d), target the commanded height (hTarget + hOff,
 * AUTO_SH_TO_REF) or an explicit toCm (AUTO_SH_TO_CM_MIN..MAX, the MOVE range); the reference starts at the corrected
 * estimate (hOff = h_new - hTarget) and ends at the target (DONE commits hOff = target - hTarget): the drone ends
 * truly at the target, its estimate there too. d may round to 0 cm (the estimate was right; the flight alone), not
 * both. Limits as before: |d| <= AUTO_SH_MAX_M, |upM| <= AUTO_SH_FLY_MAX_M (SH_STEP: the Mac sends a nearer toCm),
 * one per AUTO_SH_GAP_MS, the slewed 0.15 m/s, and every correction refused near the battery end; a flight up also
 * needs the thrust headroom (AUTO_MV_UP_HEADROOM_*, both forms); a descent of the target form keeps a MOVE down's floor
 * (autoMotionHeightFloor: the target under it is refused HEIGHT; 2026-10-16 review). The 4-byte form is unchanged
 * (the build before it) but for UP_HEADROOM, which at the 58000 ceiling refuses exactly what BATT refused there. */
#define AUTO_SH_MAX_M              0.50f   /* |d| per command, in whole cm (a larger error: send 0.5 m, a new photo, again;
                                              the absolute form's X - h_now up to 0.505 rounds to 50 cm) */
#define AUTO_SH_FLY_MAX_M          0.55f   /* |upM|: the flight (d, or more when the estimate read above AUTO_MV_H_MAX_M) */
#define AUTO_SH_GAP_MS             2000    /* at most one accepted per this (refused: SH_RATE) */
#define AUTO_SH_H_MIN_M            0.10f   /* the corrected estimate at least this ... */
#define AUTO_SH_X_CM_MIN           10      /* ... the judged height X (cm) within this .. */
#define AUTO_SH_X_CM_MAX           250     /* .. this (RANGE) */
#define AUTO_SH_AT_NOW             INT16_MIN   /* cmd.moveUpCm (h_at): the absolute form, d = X - the estimate now */
#define AUTO_SH_FORM_TO            1           /* cmd.mode: the "go to the target" form (6-byte SET_HEIGHT) */
#define AUTO_SH_TO_REF             INT16_MIN   /* cmd.moveRightCm (toCm): the commanded height (hTarget + hOff) */
#define AUTO_SH_TO_CM_MIN          25          /* an explicit target, cm: AUTO_MV_H_MIN_M .. */
#define AUTO_SH_TO_CM_MAX          200         /* .. AUTO_MV_H_MAX_M (RANGE) */
_Static_assert(AUTO_SH_TO_CM_MIN == (int)(AUTO_MV_H_MIN_M * 100.0f + 0.5f) &&
                   AUTO_SH_TO_CM_MAX == (int)(AUTO_MV_H_MAX_M * 100.0f + 0.5f),
               "an explicit SET_HEIGHT target spans the MOVE height range");
/* The height offset's limit (vert_ctrl clamps vzInput_t.hOff, auto_motion commits it): the commanded range plus a
 * correction's transient (the reference sits up to AUTO_SH_MAX_M beyond it while the correction flies back). Smaller,
 * the clamp would make hMax + hOff under-read and a correction could end in a false EST_RANGE LAND. */
_Static_assert(VZ_HOFF_MAX == AUTO_MV_HOFF_MAX, "the two height-offset limits agree");
_Static_assert(AUTO_MV_HOFF_MAX >= (VZ_H_TARGET_MAX - VZ_H_TARGET_MIN) + AUTO_SH_MAX_M - 1e-6f,
               "the height offset holds the commanded range plus a SET_HEIGHT's transient");

/* A decoded Mac command (docs/autonomy-a2.md section 8.3); the link has
 * already checked the tag, session and sequence number. */
typedef struct {           /* ordered so the u32 wait keeps the size at 28 bytes (autoFsmIn_t's offsets) */
  autoCmdType_t type;
  uint32_t token;         /* TAKEOFF: the armToken from the ARM ack (LAND/DISARM ignore it) */
  uint32_t actWaitMs;     /* TAKEOFF: this sortie's hover wait, ms: 0 = NVS autowait, else
                             AUTO_HOVER_WAIT_TAKEOFF_MIN_MS..AUTO_HOVER_WAIT_MAX_MS (interactive hover; u16 until
                             2026-10-14) */
  uint16_t airMaxMs;      /* ARM: 0 = NVS value; else AUTO_AIR_MAX_MIN_MS..min(NVS value, AUTO_AIR_MAX_HI_MS) */
  uint16_t heightCm;      /* TAKEOFF: AUTO_HEIGHT_CM_MIN..MAX; SET_HEIGHT: X, the judged true height
                             (AUTO_SH_X_CM_MIN..MAX) */
  uint16_t photoGapMs;    /* TAKEOFF, PHOTO: AUTO_PHOTO_GAP_MIN_MS..MAX */
  int16_t moveRightCm;    /* MOVE: + right, - left (|.| <= AUTO_MV_HORIZ_MAX_CM); TAKEOFF: the planned move's (0: none);
                             SET_HEIGHT with mode AUTO_SH_FORM_TO: the target, cm (AUTO_SH_TO_REF: the commanded) */
  int16_t moveFwdCm;      /* MOVE: + forward (toward the scene ahead, <= AUTO_MV_FWD_MAX_CM), - back
                             (>= -AUTO_MV_HORIZ_MAX_CM), the vector <= AUTO_MV_LEN_MAX_CM; TAKEOFF: the planned move's */
  int16_t moveUpCm;       /* MOVE: + up, - down (|.| <= AUTO_MV_UP_MAX_CM); SET_HEIGHT: h_at, the drone's estimate
                             when the judged photo was captured, cm (AUTO_SH_AT_NOW: now) */
  int16_t yawDeg;         /* YAW: + clockwise (right), - left (|.| <= AUTO_MV_YAW_MAX_DEG) */
  uint8_t mode;           /* ARM: autoMode_t; SET_HEIGHT: AUTO_SH_FORM_TO (the 6-byte form), 0 the 4-byte one */
  uint8_t photoN;         /* TAKEOFF: 0..AUTO_PHOTO_N_MAX; PHOTO: 1..AUTO_PHOTO_N_MAX */
} autoCmd_t;

/* Progress of the photo set the FSM started (glue -> FSM). */
typedef enum {
  AUTO_CAP_NONE = 0,      /* nothing started in this sortie */
  AUTO_CAP_BUSY,
  AUTO_CAP_DONE,          /* every frame copied */
  AUTO_CAP_FAILED,        /* camera not running, or a frame missing */
} autoCapture_t;

/* ARM guards, one bit each in failedGuards (set = failed). Values go on
 * the wire; only append. */
enum {
  AUTO_G_LINK     = 1u << 0,   /* G0 session valid, heartbeat < 300 ms */
  AUTO_G_LOCKS    = 1u << 1,   /* G1 restart lock, reset-lock (not in DRY), emergency stop, motor driver STOP/fault/inhibit, motor watchdog off or tripped */
  AUTO_G_STILL    = 1u << 2,   /* G2 on the ground >= 3 s (FLIGHT: >= 20 s, after a flight the heat soak
                                  AUTO_SOAK_*), still, level */
  AUTO_G_BATT     = 1u << 3,   /* G3 resting vbat >= vmin (FLIGHT, BENCH; DRY only needs a finite reading, A2b) */
  AUTO_G_CAMERA   = 1u << 4,   /* G4 camera running, hold enabled, fps > AUTO_FPS_MIN (FLIGHT only) */
  AUTO_G_APP      = 1u << 5,   /* G5 app required: packet < 500 ms old, no input for >= 1 s */
  AUTO_G_GCAP     = 1u << 6,   /* G6 FLIGHT: gcap 0; BENCH: 1..20000 */
  AUTO_G_ENABLED  = 1u << 7,   /* G7 NVS autoon */
  AUTO_G_VZPARAMS = 1u << 8,   /* G8 learned vertical values in range */
  AUTO_G_HEAP     = 1u << 9,   /* G9 internal heap >= AUTO_HEAP_MIN */
  AUTO_G_GESTURE  = 1u << 10,  /* G10 consent gesture (when required) */
  AUTO_G_KILL     = 1u << 11,  /* G11 Mac KILL in the last 2 s */
  AUTO_G_MODE     = 1u << 12,  /* G12 FLIGHT must not run on the sim plant */
};

/* Reason of the latest state transition (and of a refused command). Values
 * are logged, kept in RTC memory and sent to the Mac; only append. */
typedef enum {
  AUTO_RS_NONE = 0,
  AUTO_RS_ARM = 1,
  AUTO_RS_TAKEOFF = 2,
  AUTO_RS_HOLD = 3,            /* TAKEOFF -> HOVER: vertical controller in HOLD */
  AUTO_RS_DISARM_CMD = 4,
  AUTO_RS_ARM_TIMEOUT = 5,
  AUTO_RS_MOVED = 6,           /* ARMED or pre-spin: stillness lost */
  AUTO_RS_LOCKED = 7,          /* ARMED: a G1 lock appeared */
  /* LAND (or, from ARMED and pre-spin, IDLE) */
  AUTO_RS_LAND_CMD = 8,
  AUTO_RS_HB_LOST = 9,
  AUTO_RS_APP_INPUT = 10,
  AUTO_RS_APP_SILENT = 11,
  AUTO_RS_SESSION = 12,        /* new Mac session while active */
  AUTO_RS_AIR_TIME = 13,       /* air clock (never produced since 2026-10-15: no air clock) */
  AUTO_RS_VBAT_LOW = 14,       /* 10 Hz < AUTO_VBAT_LAND_V for AUTO_VBAT_LAND_MS */
  AUTO_RS_VBAT_DIP = 15,       /* 1 kHz minimum < AUTO_VBAT_FAST_LAND_V */
  AUTO_RS_LOOP_GAP = 16,       /* AUTO_LOOP_GAP_N gaps within AUTO_LOOP_GAP_WIN_MS (one gap until 2026-10-15) */
  AUTO_RS_VZ = 17,             /* the vertical controller started LAND itself: see vzLandReason */
  AUTO_RS_NOT_STABLE = 18,     /* never produced since 2026-10-15 (the event NOT_STABLE, the wait starts) */
  AUTO_RS_SORTIE_DONE = 19,    /* never produced since 2026-10-15 (a hop is Claude's own LAND) */
  AUTO_RS_CAPTURE_FAIL = 20,   /* never produced since 2026-10-15 (the event CAPTURE_FAIL) */
  AUTO_RS_TAKEOFF_TIMEOUT = 21,
  /* end of LAND */
  AUTO_RS_TOUCHDOWN = 22,      /* vertical controller DONE (touchdown, or abort finished on the ground) */
  AUTO_RS_TOUCHDOWN_TRIP = 23, /* tilt/rate trip with the touchdown thrust condition met after contact */
  AUTO_RS_LAND_TIMEOUT = 24,   /* vertical controller cut after 6 s of LAND */
  /* KILLED */
  AUTO_RS_KILL_CMD = 25,
  AUTO_RS_TILT = 26,
  AUTO_RS_RATE = 27,
  AUTO_RS_FREEFALL = 28,
  AUTO_RS_ATT_ERR = 29,
  AUTO_RS_NAN = 30,
  AUTO_RS_VZ_CUT = 31,         /* vertical controller in CUT outside LAND (NaN, bad config) */
  AUTO_RS_MOTOR_GUARD = 32,    /* motor watchdog tripped */
  AUTO_RS_MOTOR_STOP = 33,     /* motor driver STOP/fault latched by someone else */
  AUTO_RS_ESTOP_EXT = 34,      /* stabilizer emergency stop set outside autonomy */
  AUTO_RS_LAND_BACKSTOP = 35,
  AUTO_RS_KILL_CLEARED = 36,   /* KILLED -> IDLE */
  AUTO_RS_HOVER_TIMEOUT = 37,  /* never produced since 2026-10-15 (the wait is information only) */
  AUTO_RS_HOVER_NO_CAM = 38,   /* never produced since 2026-10-15 (the camera rule: CAM_LOST, then SINK) */
  AUTO_RS_BATT_LOW = 39,       /* LAND: HOVER with the 10 Hz battery below AUTO_VBAT_LOW_LAND_V, or T_hold above
                                  AUTO_HEADROOM_LAND_T, for AUTO_VBAT_LOW_LAND_MS (0.5 s) */
  AUTO_RS_MOVE_NO_CAM = 40,    /* never produced since 2026-10-15 (the motion STOPPED NO_VIEW) */
  AUTO_RS_MOVE_TILT = 41,      /* never produced since 2026-10-15 (the motion STOPPED TILT) */
  AUTO_RS_COLLISION = 42,      /* LAND: an in-air contact in the climb without the camera height (FLIGHT, DRY): blind
                                  descent, camera hold blocked (in HOVER too until 2026-10-15; KILLED in
                                  98cd4ae..eb9264d) */
  AUTO_RS_PALM = 43,           /* never produced since 2026-10-15 (the palm catch removed) */
  AUTO_RS_SINK = 44,           /* LAND: the camera height out of use AUTO_CAM_LOST_MAX_MS (or at once without a
                                  camera-measured hover bound): the controller's CMD_LAND; resumable while it lasts */
  AUTO_RS_RESUME = 45,         /* LAND SINK -> HOVER: the camera height back AUTO_CAM_BACK_MS, the controller in HOLD */
} autoReason_t;

/* Events (AUTO_STATUS evCode, the latest three; "Claude decides", docs/autonomy-a2b.md 22): what the drone did on its
 * own instead of landing, with a detail byte. Values go on the wire; only append. */
typedef enum {
  AUTO_EV_NONE = 0,
  AUTO_EV_CAM_LOST = 1,        /* the camera height out AUTO_CAM_LOST_REPORT_MS: holding open loop; detail camBits */
  AUTO_EV_CAM_BACK = 2,        /* in use again AUTO_CAM_BACK_MS: closed again */
  AUTO_EV_SINK = 3,            /* LAND SINK started; detail 0 the 5 s budget, 1 no camera-measured hover, 2 after a
                                  contact or a climb fault */
  AUTO_EV_RESUME = 4,          /* SINK -> HOVER, the camera height back */
  AUTO_EV_CONTACT = 5,         /* an in-air contact: hush, motion stopped or a snap; detail the controller's phase |
                                  AUTO_CONTACT_MOTION | AUTO_CONTACT_NO_HUSH */
  AUTO_EV_VZ = 6,              /* the controller held and reported; detail its vzReason_t (K_RANGE, HOLD_CAPPED,
                                  EST_RANGE, CLIMB_CAPPED, CLIMB_TIMEOUT, CONTACT_HOLD, RESUME) */
  AUTO_EV_NOT_STABLE = 7,      /* no STABLE 1 s by HOVER + 2 s: the wait starts anyway; detail AUTO_NS_* */
  AUTO_EV_TAKEOFF_SLOW = 8,    /* the air clock at airMaxMs before the wait: the wait starts anyway */
  AUTO_EV_CAPTURE_FAIL = 9,    /* a photo set failed (or timed out): the set is consumed, PHOTO again */
  AUTO_EV_MOVE_STOPPED = 10,   /* the drone stopped a motion; detail autoMvEnd_t (NO_VIEW, TILT, LOOP_GAP, VIEW_FAST) */
  AUTO_EV_LOOP_GAP = 11,       /* a stabilizer loop gap over AUTO_LOOP_GAP_LAND_US; detail its ms (255: more) */
  AUTO_EV_RETRACK_UNDO = 12,   /* a YAW turns back to its start heading; detail autoMvEnd_t of the cause (RETRACK, NO_VIEW,
                                  CONTACT) */
  AUTO_EV_HOLD_NO_VIEW = 13,   /* HOVER without a motion: the hold not applied AUTO_VIEW_LOST_REPORT_MS (the drone
                                  drifts); detail 1 coasting, 2 not applied, 3 no fresh frame */
  AUTO_EV_VIEW_BACK = 14,      /* the hold applied on a fresh frame again */
  AUTO_EV_HOLD_LIMIT = 15,     /* information (2026-10-16, docs/autonomy-a2b.md 27.4): the camera hold's roll or pitch
                                  correction has sat at >= CAM_HOLD_LIMIT_WARN_DEG (of its 1.5 deg integral range) for
                                  CAM_HOLD_LIMIT_WARN_MS in HOVER outside any motion (camera_hold.h camHoldLimitStep):
                                  the trim, a leg, a prop or the air needs that lean and the hold may not keep the drone
                                  in place; detail AUTO_HL_* of the axes that rose. Nothing else: the drone holds */
} autoEvent_t;

/* HOLD_LIMIT's detail (= camHoldLimitStep()'s bits, camera_hold.h CAM_HOLD_LIM_*): the axes and their signs. */
#define AUTO_HL_ROLL               0x01
#define AUTO_HL_PITCH              0x02
#define AUTO_HL_ROLL_NEG           0x04    /* the roll correction is negative (left side down) */
#define AUTO_HL_PITCH_NEG          0x08    /* the pitch correction is negative (nose down) */

/* AUTO_NOT_STABLE's detail: the STABLE conditions that failed on the event's step. */
#define AUTO_NS_TILT               0x01
#define AUTO_NS_GYRO               0x02
#define AUTO_NS_HOLD               0x04    /* the camera hold not applied on a fresh frame */
#define AUTO_NS_VZ                 0x08    /* the controller not steady */

/* AUTO_STATUS holdBits (autoFsmReport_t holdBits, the glue adds the controller's and the hold's). */
#define AUTO_HB_CAM_LOST           0x01    /* the camera height out (CAM_LOST reported, no CAM_BACK yet) */
#define AUTO_HB_SINKING            0x02    /* LAND SINK */
#define AUTO_HB_HUSH               0x04    /* a contact's hush runs */
#define AUTO_HB_K_CLAMP            0x08    /* Khat at its upper clamp or at / below khatMinRel (the glue) */
#define AUTO_HB_CAPPED             0x10    /* the thrust at a ceiling (the glue) */
#define AUTO_HB_NO_VIEW            0x20    /* the hold not applied on a fresh frame (HOLD_NO_VIEW reported) */
#define AUTO_HB_NOT_STABLE         0x40    /* the wait started without STABLE 1 s (until the first one) */
#define AUTO_HB_OPEN_LOOP          0x80    /* HOLD without the camera height: open loop (the glue) */

/* What the state machine did on its own and the evidence (autoFsmReport()): AUTO_STATUS bytes 96-135 (auto_proto.h).
 * Times are the FSM's ms clock; the status takes ages from them. Cleared at ARM. */
typedef struct {
  uint16_t evCount;            /* events this sortie (wraps): the Mac logs one per change */
  uint8_t evCode[3];           /* autoEvent_t: [0] the latest, [1], [2] the two before */
  uint8_t evDetail[3];
  uint32_t evMs[3];
  uint8_t holdBits;            /* AUTO_HB_CAM_LOST, SINKING, HUSH, NO_VIEW, NOT_STABLE */
  uint8_t camBits;             /* why the camera height was out (in->camBits, latched at its CAM_LOST, ORed while out) */
  uint8_t contacts;            /* contacts this sortie (saturating) */
  uint8_t loopGaps;            /* loop gaps this sortie (saturating) */
  uint32_t camOutMs;           /* with AUTO_HB_CAM_LOST: the budget's start (the last step in use, or the HOLD entry:
                                  vzOutput_t noCamS) */
  uint32_t sinkAtMs;           /* with AUTO_HB_CAM_LOST: when the SINK comes (0: no budget: at once) */
  float peakAccG, peakRateDps; /* the latest contact's peaks over its hush: |acc|, max(|gx|, |gy|) */
  float contactAxG, contactAyG;   /* its body-frame force at the |acc| peak (x forward, y left; the obstacle is opposite) */
  float contactYawDeg;         /* the heading offset (out.mv.yawOffDeg) at the contact: the side is relative to it */
  uint32_t lastCmdMs;          /* the latest accepted MOVE, YAW, STOP or PHOTO (lastCmdAny) */
  bool lastCmdAny;
} autoFsmReport_t;

/* The LAND reasons whose LAND may be the vertical controller's fast LAND (vzInput_t.landFastOk,
 * docs/autonomy-a2b.md 17.4): the Mac's LAND, the air clock, the hover wait's end, the sortie done. Never a
 * battery rule (VBAT_LOW, VBAT_DIP: a brown-out-level dip that a 0.5 s-filtered Vf does not see, BATT_LOW), a loop
 * gap, a lost link or app, a motion's or the camera's guard, a contact (COLLISION is blind anyway). The glue
 * passes it on every LAND step (autonomy.c vzRun()); the controller reads it at the LAND's start. */
#define AUTO_LAND_FAST_RS0 AUTO_RS_LAND_CMD   /* the allow list as a bit mask from the lowest reason (IRAM: small) */
#define AUTO_LAND_FAST_MASK ((1u << (AUTO_RS_LAND_CMD - AUTO_LAND_FAST_RS0)) | \
                             (1u << (AUTO_RS_AIR_TIME - AUTO_LAND_FAST_RS0)) | \
                             (1u << (AUTO_RS_HOVER_TIMEOUT - AUTO_LAND_FAST_RS0)) | \
                             (1u << (AUTO_RS_SORTIE_DONE - AUTO_LAND_FAST_RS0)))
_Static_assert(AUTO_RS_HOVER_TIMEOUT - AUTO_LAND_FAST_RS0 < 32 && AUTO_RS_SORTIE_DONE - AUTO_LAND_FAST_RS0 < 32 &&
               AUTO_RS_AIR_TIME > AUTO_LAND_FAST_RS0, "the fast LAND's reasons fit the 32-bit mask");
static inline bool autoFsmLandFastReason(autoReason_t r)
{
  const uint32_t k = (uint32_t)r - (uint32_t)AUTO_LAND_FAST_RS0;
  return k < 32u && ((AUTO_LAND_FAST_MASK >> k) & 1u) != 0u;
}

/* Everything the guards need, sampled once per stabilizer loop. Times are
 * ms; an age of UINT32_MAX means "never". */
typedef struct {
  uint32_t nowMs;              /* FreeRTOS tick in ms (wraps; compare with unsigned differences) */
  /* Mac link */
  autoCmd_t cmd;               /* at most one per step from the link mailbox; AUTO_CMD_NONE otherwise */
  uint32_t killCount;          /* Mac KILLs received since boot (monotonic); a change is a KILL */
  bool sessionValid;
  bool sessionChanged;         /* true for one step when HELLO replaced the session */
  uint32_t hbAgeMs;            /* since the last valid heartbeat */
  uint32_t randToken;          /* nonzero random value; becomes armToken when an ARM is accepted */
  /* phone app (crtp_commander_rpyt.c hook; docs/autonomy-a2.md section 8.2) */
  uint32_t appInputCount;      /* app packets that were input (thrust >= 1000 or |roll|/|pitch| >= 2 deg), monotonic */
  uint32_t appAgeMs;           /* since the last valid app packet */
  uint32_t appInputAgeMs;      /* since the last app packet that was input */
  uint32_t appGestureAgeMs;    /* since a roll push >= 10 deg was released below 2 deg */
  bool appThrustZero;          /* the last app packet had thrust < 1000 */
  /* settings (NVS, USB console only) */
  bool autoEnabled;            /* autoon */
  bool appRequired;            /* appreq, default on */
  bool gestureRequired;        /* autogest, default on */
  float vmin;                  /* V, AUTO_VMIN_LO..HI, default AUTO_VMIN_DEFAULT */
  uint16_t airMaxNvsMs;        /* autoair, AUTO_AIR_MAX_MIN_MS..AUTO_AIR_MAX_HI_MS (forced into it) */
  uint16_t reservedWait16;     /* the u16 autowait until 2026-10-14: unused (the IRAM code's offsets) */
  uint16_t hoverAccelNvsMs;    /* autoaccel, 0..AUTO_HOVER_ACCEL_MAX_MS (forced into it); latched at ARM */
  bool vzParamsOk;             /* vzK and vzBair in range */
  /* attitude and IMU: always the real sensors, in every mode */
  float rollDeg, pitchDeg;     /* estimated attitude */
  float tiltDeg;               /* acos(gravZ) */
  float attErrDeg;             /* max(|roll - rollDesired|, |pitch - pitchDesired|), trim and camera hold included */
  float gyroDps[3];            /* body rates x, y, z */
  float accNormG;              /* |acc| */
  uint32_t loopGapUs;          /* this stabilizer loop's start minus the previous one's */
  /* battery: always the real battery */
  float vbat;                  /* V, 10 Hz reading */
  float vbatFastMin;           /* V, lowest 1 kHz reading over the last 320 ms; 0 = no reading */
  /* vertical controller: its output from the previous step */
  vzOutput_t vz;
  bool simPlant;               /* vz runs on vzSim (DRY, or BENCH by default) */
  /* camera */
  uint8_t camState;            /* cameraFlowState_t */
  float camFps;
  uint32_t camLagUs;          /* measured optical-flow midpoint latency */
  uint32_t camFrameAgeMs;      /* since the last processed frame with a valid fit */
  uint32_t camViewMs;          /* camera running with fitted frames <= AUTO_VIEW_FRAME_AGE_MS old continuously for this long */
  bool holdEnabled;            /* camera drift hold switch */
  bool holdApplied;            /* hold correction applied this loop (engaged or coasting) */
  bool holdCoasting;           /* applied from the learned integral only (view lost) */
  autoCapture_t capture;       /* progress of the photo set this FSM started */
  /* interactive hover (auto_motion.h) */
  float sceneDM;               /* the camera height observer's scene distance f / G, m (FLIGHT, BENCH-imu) */
  bool sceneTracked;           /* its scale tracker has a G and the estimate is fresh */
  uint32_t sceneResets;        /* its tracker restarts (yaw) since boot */
  float holdProgLat, holdProgFwd;   /* the hold's measured motion in its reference epoch holdEpochSeen */
  uint32_t holdEpochSeen;
  bool holdMoving;             /* the hold's measured rates not still (camera_hold.h camHoldMoving): mvfix STILL */
  /* ground, locks, emergencies */
  bool onGround;               /* stabilizerOnGround() */
  uint32_t onGroundMs;         /* how long it has been true */
  bool restartLocked;          /* stabilizerRestartLocked() */
  bool resetLocked;            /* autonomy reset-lock (section 6): not a battery power-on */
  bool emergencyStop;          /* stabilizer emergency stop flag */
  bool motorsReady;            /* motorsFlightReady() && !motorsIsInhibited() */
  bool motorGuardOn;           /* motor watchdog enabled */
  bool motorGuardTripped;
  uint16_t gcap;               /* stabilizer ground-test thrust cap, 0 = off */
  uint32_t heapIntFree;        /* bytes */
  uint32_t lastFlightMotorMs;  /* motor-on time of the latest flight since boot (stabilizer.c), 0 if none (G2's
                                  heat soak) */
  uint32_t hoverWaitNvsMs;     /* autowait, 0..AUTO_HOVER_WAIT_MAX_MS (forced into it); latched at ARM. Last: the
                                  fields above keep their offsets in the IRAM code */
  /* "Claude decides" (2026-10-15), appended */
  uint8_t camBits;             /* why the camera height estimate is not healthy (the glue, 10 Hz while it is out):
                                  AUTO_CB_* */
  float accXG, accYG;          /* body-frame specific force x (forward), y (left), g, every loop (a contact's side) */
} autoFsmIn_t;

/* autoFsmIn_t camBits (cam_vz_glue.c camVzGlueWhyBits; AUTO_STATUS camBits). */
#define AUTO_CB_NO_FRAMES          0x01    /* no accepted update for the observer's healthy window (2 s) */
#define AUTO_CB_NO_G               0x02    /* the scale tracker has no G (and no guess allowed) */
#define AUTO_CB_CUSUM              0x04    /* the observer's CUSUM alarm holds */
#define AUTO_CB_FAULT              0x08    /* the observer faulted, or its snapshot is stale or paused */
#define AUTO_CB_NOT_RUNNING        0x10    /* the camera is not RUNNING (the state machine adds it) */

typedef struct {
  autoState_t state;
  autoMode_t mode;             /* of the current (or last) ARM */
  autoThrust_t thrust;
  float thrustMax;             /* mode clamp, applied by the glue to setpoint.thrust AND as the very last
                                  stage on control.thrust: 65535 when NONE, FLIGHT VZ_THRUST_ABS_MAX (58000), DRY 0,
                                  BENCH bench cap */
  vzCmd_t vzCmd;               /* for this step's vzStep() */
  bool vzReset;                /* vzInit() before this step's vzStep() (pre-spin over) */
  float hTargetM;              /* for the vz config at vzReset */
  bool yawCapture;             /* one step: yawHold = current yaw (TAKEOFF entry) */
  bool prespinLed;             /* pre-spin warning pattern on */
  bool captureStart;           /* one step: start the photo set */
  uint8_t captureN;
  uint16_t captureGapMs;
  bool lockAppThrust;          /* one step: crtpCommanderRpytLockThrust() (leaving an active state) */
  bool setEmergencyStop;       /* one step: stabilizerSetEmergencyStop() (KILLED entry) */
  bool clearEmergencyStop;     /* one step: stabilizerResetEmergencyStop() (KILLED -> IDLE, only if we set it) */
  bool stable;                 /* STABLE conditions hold this step */
  uint32_t stableMs;           /* age of the STABLE streak, from its first step; it counts on through a dropout
                                  shorter than AUTO_STABLE_GRACE_MS (stable false meanwhile) while the streak's
                                  dropouts stay below AUTO_STABLE_DROP_BUDGET_MS; 0 without a streak.
                                  STABLE 1 s (the photos) = stable && stableMs >= AUTO_STABLE_MS */
  bool waiting;                /* HOVER: photos done (or the wait started anyway), hovering for the Mac's next command */
  uint32_t waitLeftMs;         /* 0 since 2026-10-15 (no HOVER_TIMEOUT: the wait is information only) */
  uint32_t stateMs;            /* time in the current state */
  uint32_t airMs;              /* air clock: since the controller's first RAMP step, 0 before */
  autoReason_t reason;         /* of the latest transition (sticky) */
  vzReason_t vzLandReason;     /* when reason is AUTO_RS_VZ or LAND ended: the controller's reason */
  autoReason_t landTrip;       /* LAND ended TOUCHDOWN_TRIP: which trip (TILT or RATE); NONE otherwise */
  uint32_t failedGuards;       /* IDLE: guards for FLIGHT mode; on ARM/TAKEOFF: the bitmap checked */
  bool cmdDone;                /* in.cmd was processed this step; result below */
  autoResult_t cmdResult;
  uint32_t cmdDetail;          /* ARM OK: armToken; GUARDS: failedGuards; otherwise the state */
  bool camNoStandby;           /* ARMED..HOVER of every FLIGHT and BENCH sortie: the glue calls
                                  cameraFlowSetTakeoffStandbySkip(true) (camera awake through the take-off) */
  autoMvOut_t mv;              /* the interactive hover: the hold's reference, the vertical reference and offset,
                                  the yaw offset, the motion status (all 0 without a motion, the epoch kept) */
  bool battLowNow;             /* the BATT_LOW condition holds now: the 10 Hz battery below AUTO_VBAT_LOW_LAND_V
                                  (one reading) or T_hold above AUTO_HEADROOM_LAND_T */
  bool battLowHeld;            /* ... for AUTO_VBAT_LOW_REFUSE_MS: MOVE and YAW are refused (BATT). A refusal for
                                  T_hold above AUTO_MV_REFUSE_T alone does not show here (the LOW_BATT reply does) */
  uint16_t heightCm;           /* the sortie's TAKEOFF height (0 before) */
  bool collided;               /* this sortie's LAND is LAND COLLISION (an in-air contact in the climb without the
                                  camera height), or a LAND that began in a contact's window (its hush, or before the
                                  camera height was back AUTO_CAM_BACK_MS after it), from that step until the next ARM:
                                  in LAND the glue blocks the camera hold from that very step (autonomyHoldBlocked()),
                                  and a SINK so begun never resumes. (The descent's blindness is blind's, below.) */
  float fwdLn;                 /* the measured forward progress ln(D0 / D) of the sortie's MOVEs (autoMotionFwdLn):
                                  the glue hands it to the camera height observer, whose gain follows it
                                  (cam_vz_obs.h camVzObsFrame_t fwdLn; 2026-10-15). Last: the IRAM code's offsets */
  bool hush;                   /* a contact's hush runs: vzInput_t.hush (2026-10-15) */
  bool resume;                 /* LAND SINK with the camera height back: vzInput_t.resume, vzCmd TAKEOFF */
  bool blind;                  /* vzInput_t.blind (autonomy.c vzRun()): a contact (CONTACT, any of them, or COLLISION)
                                  since this sortie's ARM, so every LAND of it, whatever its reason (Claude's, a battery
                                  rule, a SINK's descent, the link, the phone, a loop gap, the controller's own), is the
                                  blind one (user decision (B) 2026-10-15; only a LAND in the contact's window until
                                  then). Set in every state: a LAND the controller starts itself is blind from its first
                                  step; the controller reads it only in LAND (the hold keeps the camera) */
  float hShift;                /* one step, m: an accepted SET_HEIGHT's offset d; the glue shifts the controller's height
                                  estimate by it before this step's vzStep() (autonomy.c applyHeightShift(),
                                  vzShiftHeight()); 0 on every other step. Appended (2026-10-15) */
} autoFsmOut_t;

/* Persistence timer (private to auto_fsm.c): a condition that has held on
 * every step since sinceMs. A flag, not a 0 sentinel: the ms clock wraps
 * and can read 0. */
typedef struct {
  uint32_t sinceMs;
  bool on;
} autoHeld_t;

/* State machine memory. Fields are private to auto_fsm.c (its owner may
 * change them freely); callers only allocate it. */
typedef struct {
  autoState_t state;
  autoMode_t mode;
  autoReason_t reason;
  autoReason_t landEndReason;  /* how the controller's landing ended; LAND -> IDLE carries it */
  autoReason_t landTrip;
  vzReason_t vzLandReason;
  uint32_t stateSinceMs, armMs, rampMs, holdMs, captureMs, waitMs, idleSinceMs, landEndMs, killMs, contactMs;
  uint32_t armToken;
  uint16_t heightCm, photoGapMs, airMaxMs, reservedWait16, hoverAccelMs, capGapMs;   /* reservedWait16: the u16
                                                                                       hoverWaitMs until 2026-10-14 */
  uint8_t photoN, capN;
  float benchCap;
  bool prespinDone, rampSeen, captureStarted, waiting, releaseHold, weSetEstop, landEnded, landCut, stableOnce;
  bool camAwake;       /* latched at ARM: FLIGHT or BENCH */
  bool contactSeen;
  uint32_t lastKillCount, lastAppInputCount;
  bool killSeen;
  uint32_t stableOkMs;     /* last step the STABLE conditions held (a dropout's grace counts from it) */
  uint32_t stableDropMs;   /* the streak's ended dropouts, summed (each as now - stableOkMs on its last step) */
  uint32_t stableGapMs;    /* the current dropout so far, now - stableOkMs; 0 when the conditions hold */
  autoHeld_t tilt, rate, fall, attErr, vbatLow, moved, rest, stable;   /* stable: the STABLE streak */
  autoHeld_t battLow;      /* below AUTO_VBAT_LOW_LAND_V, or T_hold above AUTO_HEADROOM_LAND_T */
  uint32_t hoverDurMs;     /* how long the last HOVER lasted (the LAND backstop's drift allowance) */
  uint32_t landBackstopMs; /* this LAND's backstop: AUTO_LAND_BACKSTOP_MS + the extra height, moves up, drift */
  autoMotion_t mv;         /* the interactive hover's motions */
  autoHeld_t collide;      /* |acc| or a gx/gy rate over the in-air contact thresholds (AUTO_COLLIDE_*); last: the
                              fields above keep their offsets in the IRAM code */
  bool collided;           /* this sortie's LAND blocks the camera hold (out.collided): LAND COLLISION, or a LAND that
                              began during a contact's hush or before the camera height was back after it (cleared at
                              ARM) */
  autoHeld_t reservedPalm; /* the palm cue's tracker until 2026-10-15 (the IRAM code's offsets) */
  uint32_t hoverWaitMs;    /* this sortie's hover wait (NVS autowait at ARM, or TAKEOFF actWaitMs), ms: information */
  uint32_t reservedVRefMs; /* the palm cue's vertical-reference time until 2026-10-15 (the offsets) */
  /* The TAKEOFF's planned move (2026-10-14, docs/autonomy-a2b.md 21.4): latched by every accepted TAKEOFF (a
   * TAKEOFF without a plan clears it), started as a MOVE (autoMotionMovePlan: every acceptance check on the live D,
   * the forward floor the Mac's AUTO_PLAN_MIN_FRONT_CM) on
   * the step the hover wait starts (row 26); a STOP before the wait cancels it; a LAND or KILL never reaches the
   * wait; so does any event before the wait since 2026-10-15 (recordEvent(): a contact, NOT_STABLE, TAKEOFF_SLOW,
   * CAPTURE_FAIL, the camera height lost, a controller event, a HOVER a climb fault started ...: every one of them
   * landed until then, so the plan never flew after it; mv.why AUTO_MVW_PLAN_CANCELLED). A refusal leaves mv.why
   * (AUTO_STATUS mvWhy) and the drone hovering in the wait, as after a refused Mac MOVE. AUTO_STATUS shows it as it
   * shows a Mac MOVE (mvSeq 1, mvState RUN ...). Last: the IRAM code's offsets. */
  int16_t planRightCm, planFwdCm;
  uint8_t planPending;
  uint8_t hushSnap;        /* the latest hush's contact came in HOVER with no motion running (2026-10-15; planPad
                              until then): at its end the hold re-anchors where the bounce left the drone */
  /* "Claude decides" (2026-10-15, docs/autonomy-a2b.md 22), appended: the report and the rules that replaced the
   * give-up LANDs. */
  autoFsmReport_t rep;
  uint32_t camBackSince;   /* the camera height in use again since (camLostOn) */
  uint32_t hushMs;         /* the latest hush's start (AUTO_HB_HUSH while it runs) */
  uint32_t hushPrevMs;     /* the hush before it | 1 (0: none; the chain bound) */
  uint32_t contactNextMs;  /* no contact starts before this (the hush running, the chain bound) */
  uint32_t gapMs[2];       /* the two latest loop gaps before this one (AUTO_LOOP_GAP_N within the window) */
  uint32_t blindCamSince;  /* contactBlind: the camera height in use since (| 1; 0: not in use) */
  uint32_t viewBadSince;   /* HOVER without a motion: the hold not applied since */
  uint32_t resumeSince;    /* LAND SINK: the camera height in use since (| 1; 0: not) */
  uint8_t gapN;            /* loop gaps so far this sortie, at most 2 (gapMs valid) */
  uint8_t vzEvtSeen;       /* the controller's evtN last forwarded */
  bool camLostOn;          /* the camera height out (the budget runs from rep.camOutMs) */
  bool camLostReported;    /* CAM_LOST reported for this outage */
  bool contactBlind;       /* a contact since the camera height was last back AUTO_CAM_BACK_MS */
  bool viewBadOn, viewReported;
  bool capFailSeen;        /* the latest set's CAPTURE_FAIL reported (the set consumed) */
  bool faultHover;         /* HOVER entered from a HOLD a climb fault or a contact started (no hover measured) */
  bool notStableSeen;      /* the NOT_STABLE event of this hover */
  uint32_t camOpenMs;      /* the latest step the camera rule saw the controller's open window (vzOutput_t camOpen) */
  uint32_t setHeightMs;    /* the latest accepted SET_HEIGHT | 1 (0: none this sortie; cleared at ARM): AUTO_SH_GAP_MS.
                              Appended (2026-10-15) */
  /* The hold-at-limit warning (2026-10-16, docs/autonomy-a2b.md 27.4), appended: */
  uint16_t infoEvents;     /* this sortie's information-only events (HOLD_LIMIT): autoFsmTrimEvents() leaves them out */
  uint8_t holdLimSeen;     /* the camera's AUTO_HL_* bits reported (a rise not reportable when it came is not kept;
                              0 at ARM) */
} autoFsm_t;

void autoFsmInit(autoFsm_t *f);       /* IDLE, as after every boot */
/* What the state machine did on its own this sortie (AUTO_STATUS bytes 96-135). */
static inline const autoFsmReport_t *autoFsmReport(const autoFsm_t *f)
{
  return &f->rep;
}
void autoFsmStep(autoFsm_t *f, const autoFsmIn_t *in, autoFsmOut_t *out);
/* The events that bar a sortie from the trim learner (att_trim_learn.h attLearnEndOk): all of them but the
 * information-only ones (HOLD_LIMIT), so the learner runs exactly as before the warning existed. */
static inline uint16_t autoFsmTrimEvents(const autoFsm_t *f)
{
  return (uint16_t)(f->rep.evCount - f->infoEvents);
}
/* The camera hold's at-limit bits (camera_hold.h camHoldLimitStep, AUTO_HL_*), given by the glue on the stabilizer task
 * whenever they change (autonomy.c slowInputs: a compare at the slow-input rate; this runs on a change only, flash):
 * an axis bit that rose reports HOLD_LIMIT (detail: the rising axes and every sign) in HOVER in the wait with no motion
 * running, FLIGHT mode; elsewhere it is not taken as seen, so the camera's next change reports it. A bit that fell is
 * noted; the next rise reports again (the camera needs another CAM_HOLD_LIMIT_WARN_MS at the limit). */
void autoFsmHoldLimit(autoFsm_t *f, uint8_t bits, uint32_t nowMs);
/* mvspeed (auto_motion.h "fast moves", 2026-10-16): the sortie's speed step, set by the glue once per sortie right
 * after the ARM step (whose autoMotionInit reset it to 100); every later MOVE / YAW / SET_HEIGHT plans with it. Flash. */
void autoFsmSetMvSpeed(autoFsm_t *f, uint16_t pct);

/* ARM guard bitmap for a mode, as an ARM would see it now (STATUS uses it in
 * IDLE). Pure: does not change f. */
uint32_t autoFsmGuards(const autoFsm_t *f, const autoFsmIn_t *in, autoMode_t mode);

const char *autoStateName(autoState_t s);
const char *autoReasonName(autoReason_t r);
const char *autoEventName(autoEvent_t e);

#endif /* AUTO_FSM_H_ */
