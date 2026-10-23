/*
 * ESP-Drone Firmware
 *
 * auto_motion.h - Interactive hover: the motions the Mac may ask for in the
 * hover wait (MOVE, YAW, STOP), as references for the camera drift hold, the
 * vertical controller and the yaw lock (user decision 2026-10-12, late:
 * "the real thing"; docs/autonomy-a2b.md section 14)
 *
 * The drone has no position sensor. A horizontal move is a moving reference
 * fed into the camera drift hold (camera_hold.h), in the hold's own units:
 *   lateral   x / D        (the integral of wLat = v_right / D)
 *   forward   ln(D0 / D)   (the integral of the image expansion wFwd)
 * where D is the distance to the scene ahead. D comes from the camera height
 * observer's gain G = f / D (cam_vz_obs.h; f = AUTO_MV_FY_PX), taken when a
 * command is accepted (and refused while it is not valid). So a lateral move
 * of R metres is the reference R / D0, and a forward move of F metres is
 * ln(D0 / (D0 - F)); the hold then flies the drone there with its own gains.
 *
 * Which D (fixes after the verification of 591f176): after a forward or back
 * move the expansion it measured gives the scene distance dTrack (k = 1)
 * and a lower bound dTrackLow (k >= AUTO_MV_K_LOW); a turn or a re-view
 * bob (below) forgets them. With mvfix DRIFT (2026-10-17, "move fixes"
 * below) they follow the drift the hold measured since (autoMotionTrackNow).
 *   - forward conversion: dTrack when there is one, else the observer's D;
 *   - lateral conversion: the larger of the observer's D and dTrack: a D too
 *     large makes a move short, never long;
 *   - the TOO_CLOSE floor: the lower bound min(D, dTrackLow), and the
 *     expansion scale k at its low AUTO_MV_K_LOW: the hold drives the measured
 *     k ln(D0 / D) to the target, so the true end is D0 ((D0 - F) / D0)^(1/k)
 *     (k < 1 with the floor in view; docs/autonomy-c-plan.md).
 * The hold integrates v / D_now: a lateral move off a near object onto a far
 * background (a depth step) goes D_far / D_near too far. So a lateral move is
 * at most AUTO_MV_LAT_D_REL x D, and once the lateral shift since the last
 * fresh view reaches AUTO_MV_VIEW_SHIFT_REL x D (the observer's own 0.35 rad
 * view change, as a turn) the move ends with a re-view bob of
 * AUTO_MV_REVIEW_CYCLES (the tracker's G follows the new view's depth;
 * tools/tests/test_cam_vz_obs.c testViewShiftBob), D stale meanwhile.
 * A vertical move is a vertical speed reference for vert_ctrl's HOLD (with a
 * position correction on its height estimate while the move runs), and a
 * turn is a ramp of the yaw-lock setpoint.
 *
 * Profiles: trapezoids (time-optimal under a speed and an acceleration
 * limit; a triangle when short). Horizontal: one straight-line profile in
 * metres, at most AUTO_MV_V_H_MAX and AUTO_MV_A_H, and slow enough that the
 * camera keeps its view: |wLat| <= AUTO_MV_W_LAT_MAX and |wFwd| <=
 * AUTO_MV_W_FWD_MAX (the camera height observer rejects frames beyond 1.5 px
 * of lateral flow or an expansion of 0.3 /s). A feedforward lean
 * (acceleration plus a drag estimate) helps the hold follow; the reference the
 * hold compares its (~120 ms old) frames with is delayed by AUTO_MV_CAM_LAG_MS.
 *
 * Exactly zero without a motion: every output is 0 and the epoch never
 * changes, so the hold and the vertical controller compute exactly what they
 * did before this module existed (tools/tests/test_camera_hold.c,
 * test_vert_ctrl.c prove it bit for bit).
 *
 * Hold epochs: every MOVE and YAW starts a new epoch of the hold's reference
 * (camera_hold.h): positions and progress count from 0 at the epoch start.
 * A snap (STOP, a progress guard, the end of a turn, leaving HOVER) starts an
 * epoch that first takes the hold's integral back to its value at the start of
 * the epoch it ends, i.e. the drone holds where it is, with the lean it had
 * learned before the move; a move stopped while it runs also brakes with a
 * feedforward lean (AUTO_MV_BRAKE_ACC) for as long as its speed needs.
 *
 * Guards while a motion runs (FLIGHT; DRY and BENCH do not fly, the
 * references run as if tracked perfectly). Since 2026-10-15 ("Claude decides",
 * docs/autonomy-a2b.md 22) none of them lands: autoMotionStep() returns a stop
 * request (autoMvLand_t) and the state machine stops the motion with
 * autoMotionStop() and reports it:
 *   - the hold not applied, coasting, or a frame older than
 *     AUTO_MV_VIEW_MAX_AGE_MS: the reference pauses (the hold keeps the drone
 *     where the reference stopped); AUTO_MV_NO_CAM_LAND_MS of that: a stop
 *     request (AUTO_MVL_NO_CAM: a MOVE ends STOPPED NO_VIEW, a YAW turns back,
 *     autoMotionStop());
 *   - tilt over AUTO_MV_TILT_LAND_DEG for AUTO_MV_TILT_LAND_MS: a stop request
 *     (AUTO_MVL_TILT: a MOVE ends STOPPED TILT, a YAW's turn ends where it is);
 *   - progress (the hold's measured motion) further than the margin behind
 *     or ahead of the reference while the profile runs (RUN): STOP (snap)
 *     and hold; the sideways margin is in metres at the reference's distance
 *     now (margLatM, at least NVS mvlatm; 2026-10-14; the margins "guard
 *     margins" below, looser and NVS settings since 2026-10-17);
 *   - a forward move's measured progress past progFwdMax (the live front
 *     guard, NVS mvfstop, default AUTO_MV_FRONT_STOP_CM), in RUN and SETTLE: STOP, end FRONT
 *     (with mvfix FRONT, 2026-10-17 review: every horizontal move's);
 *   - once the profile has ended (SETTLE; user decision 2026-10-13,
 *     evening): the same margins around the target (the reference has
 *     reached it; the sideways one in metres as in RUN, the forward axis,
 *     toward the scene, keeps its margin): an under-shoot within the
 *     margin does not stop it, an overshoot or a fall back toward the start
 *     past the margin does (PROGRESS). The settle ends DONE within its
 *     tolerance, STOP PROGRESS with less than half of the move measured (a
 *     frozen view), otherwise, timed out short of the target along the move
 *     or off its line by more than mvprogm, SHORT (past it: LONG, mvfix
 *     LONG, 2026-10-17): the
 *     measured end becomes the move's end, so the drone holds where it is
 *     (a LONG move with a part toward the scene keeps its target: the hold
 *     walks it back, 2026-10-17 review).
 * Every rule of the state machine keeps running around it (auto_fsm.h).
 * The status's measured progress of a MOVE stays at its end value once the
 * move has ended (the measured travel), whatever ended it.
 *
 * Pure code, no ESP-IDF dependency; host-tested by
 * tools/tests/test_auto_motion.c. autoMotionStep() runs on every stabilizer
 * loop in HOVER: IRAM on the drone (AUTO_HOT), and it calls no libm function
 * but expf (IRAM, linker_camvz.lf) and sqrtf (IRAM).
 */
#ifndef AUTO_MOTION_H_
#define AUTO_MOTION_H_

#include <stdbool.h>
#include <stdint.h>
#include "camera_cadence.h"

/* ---- command limits (user decision 2026-10-12, late; long moves 2026-10-14, docs/autonomy-a2b.md 21) -------
 * One MOVE (or the take-off's planned move, auto_fsm.h) is RANGE unless (autoMvRangeOk):
 *   |right| <= AUTO_MV_HORIZ_MAX_CM, -AUTO_MV_HORIZ_MAX_CM <= forward <= AUTO_MV_FWD_MAX_CM (back: the camera does
 *   not see behind), sqrt(right^2 + forward^2) <= AUTO_MV_LEN_MAX_CM, |up| <= AUTO_MV_UP_MAX_CM, not all 0, and
 *   with up != 0 |forward| <= AUTO_MV_HORIZ_MAX_CM (the camera reads vertical motion partly as forward).
 * The scene ahead bounds a forward move further (TOO_CLOSE below: the binding limit below D ~5.5 m), the scene
 * distance a sideways one (TOO_WIDE). Claude judges the distance to a person; the drone checks only what its
 * camera measures (the scene ahead, the view, the battery; no air time since 2026-10-15). */
#define AUTO_MV_HORIZ_MAX_CM     150     /* |right| per MOVE, and back */
#define AUTO_MV_FWD_MAX_CM       400     /* forward (toward the scene ahead) per MOVE (150 until 2026-10-14) */
#define AUTO_MV_LEN_MAX_CM       400     /* the horizontal vector's length per MOVE */
#define AUTO_MV_UP_MAX_CM        50      /* |up| per MOVE */
#define AUTO_MV_YAW_MAX_DEG      180     /* |deg| per YAW */
#define AUTO_MV_BLIND_YAW_MAX_DEG 90     /* |deg| of a YAW without a view (2026-10-15: toward texture, the hold not
                                            applied; AUTO_MVF_BLIND_TURN): its turn is exempt from the pause and the
                                            view guard, its dwell and bob are not */
#define AUTO_MV_H_MIN_M          0.25f   /* a move down may not end below this (commanded and estimated) ... */
#define AUTO_MV_H_DRIFT_MPS      0.010f  /* ... plus this per second of air: the height estimate drifts (logged
                                            0.003-0.010 m/s, reading high: 'h at touchdown' +0.08 m after
                                            15.9 s), so the drone may be lower than it thinks late in a sortie */
#define AUTO_MV_H_DRIFT_CAP_M    0.30f   /* ... that drift term at most this (2026-10-15: it reached 1.20 m at 95 s
                                            of air and refused every move down; now the floor is at most 0.55 m) */
#define AUTO_MV_H_MAX_M          2.00f   /* a move up may not end above this (commanded and estimated). One-sided
                                            since 2026-10-15: a move down is checked against the floor only, a move
                                            up against this only, so a drone reading above the top may move down.
                                            = VZ_H_TARGET_MAX (asserted in auto_fsm.h): 1.20 until 2026-10-15
                                            (docs/autonomy-a2b.md 23.9) */
#define AUTO_MV_HOFF_MAX         2.30f   /* |height offset| (= VZ_HOFF_MAX, asserted in auto_fsm.h: the commanded range
                                            2.00 - 0.20 m plus a SET_HEIGHT's 0.50 m transient; 1.50 at the 1.20 m cap
                                            from 2026-10-15, 1.00 before) */
#define AUTO_MOVE_MIN_FRONT_CM   30      /* a forward move must leave this much to the scene ahead (estimate, at
                                            the expansion scale AUTO_MV_K_LOW): the default of NVS mvfloor (50 until
                                            2026-10-17, "you are too scared to approach things"; "guard margins"
                                            below, docs/autonomy-a2b.md 32) */
#define AUTO_PLAN_FRONT_EXTRA_CM 5       /* ... the TAKEOFF's planned move this much more (autoMotionMovePlan): the
                                            Mac's own floor (espfly_auto.py mac_min_front(): mvfloor + 0.05 m), which
                                            the Mac cannot check before the take-off (2026-10-15 review) */
#define AUTO_PLAN_MIN_FRONT_CM   35      /* = AUTO_MOVE_MIN_FRONT_CM + AUTO_PLAN_FRONT_EXTRA_CM, the plan's floor at the
                                            default mvfloor (60 with the drone's 50 until 2026-10-17: the plan's worst
                                            true end at k 0.6 and the observer's D -10 % was >= 0.40 m then,
                                            docs/autonomy-a2b.md 21.9 and 32) */
#define AUTO_MV_K_LOW            0.60f   /* the lowest expansion scale k the forward floor allows for (k ~1 for a
                                            frontal wall, 0.73 for a wall plus plain floor at 3 m; lower: shorter).
                                            The default of NVS mvklow (x 100; "guards off" below, 2026-10-18): the
                                            latched value is what the code reads, this the default */
#define AUTO_MV_LAT_D_REL        0.50f   /* |lateral| <= this x D per MOVE (TOO_WIDE): bounds a depth step's
                                            overshoot (1.00 m at D 2.0 m, 1.50 m from D 3.0 m). Kept on 2026-10-17
                                            (docs/autonomy-a2b.md 32.4): the overshoot is on the blind side, x
                                            D_far / D_near off a near object (a person at 0.5-1 m). The default of
                                            NVS mvlat (x 100, 0 off; "guards off" below, 2026-10-18) */
#define AUTO_MV_VIEW_SHIFT_REL   0.35f   /* lateral shift / D since the last fresh view that ends a MOVE with a
                                            re-view bob (as the observer's 0.35 rad turn) */
#define AUTO_MV_REVIEW_CYCLES    2       /* the re-view bob's whole cycles (~3.6 s; the tracker's G follows the
                                            new depth within ~2.5 s of bob on the observer's synthetic flight) */

/* ---- profiles ------------------------------------------------------------- */
/* m/s horizontal: the user's limit, 0.20 until 2026-10-14, then 0.30 (user request 2026-10-14, 0.25-0.30 approved;
 * docs/autonomy-a2b.md 17.5). It binds only for long moves with the scene far away: the image-rate caps below hold
 * a lateral move to AUTO_MV_W_LAT_MAX x D (0.30 m/s from D 2.5 m) and a forward one to AUTO_MV_W_FWD_MAX x its
 * end distance (from 2.0 m). Overridable at compile time: the replays of the logs flown at 0.20 pin it
 * (test_motion_replay.c). */
#ifndef AUTO_MV_V_H_MAX
#define AUTO_MV_V_H_MAX          0.30f
#endif
/* m/s: the speed toward the scene ahead (a move's forward component, fwdM > 0) stays at the 0.20 of before
 * 2026-10-14, so a person approach (espfly_auto.py 'approach --person': forward steps, whose W_FWD cap uses the
 * scene D, mostly the background, and does not bind) and its STOP braking distance toward the person (0.13 m) are
 * as before; lateral and backward moves fly up to AUTO_MV_V_H_MAX (verifier 2026-10-14, docs/autonomy-a2b.md 17.5). */
#define AUTO_MV_V_FWD_MAX        0.20f
#define AUTO_MV_A_H              0.10f   /* m/s^2 horizontal (feedforward lean 0.6 deg) */
#define AUTO_MV_W_LAT_MAX        0.12f   /* 1/s: lateral image rate cap (0.8 px/frame at 15 fps; observer gate 1.5) */
#define AUTO_MV_W_FWD_MAX        0.15f   /* 1/s: expansion rate cap (observer gate 0.3 /s). The default of NVS mvwmax
                                            (x 100; "guards off" below, 2026-10-18): at 100 % a forward move's speed
                                            is at most this x its end distance, 0.07 m/s for a move ending 0.45 m
                                            from the scene */
#define AUTO_MV_V_UP_MAX         0.15f   /* m/s vertical (the user limit; the camera reads vertical motion
                                            partly as forward, ~0.45 x, so slower) */
#define AUTO_MV_A_UP             0.20f   /* m/s^2 */
#define AUTO_MV_UP_KP            1.0f    /* 1/s: position correction on the height estimate while a vertical move runs */
#define AUTO_MV_UP_VREF_MAX      0.15f   /* m/s: |vertical speed reference| including the correction */
#define AUTO_MV_YAW_RATE_DPS     30.0f   /* deg/s (user decision 2026-10-13, evening: 15 -> 30; the spec's ceiling
                                            is 45). The hold and the observer drop frames turning faster than
                                            0.35 rad/s (20 deg/s) except while a commanded turn runs (its turn,
                                            dwell and bob): AUTO_MV_TURN_GATE_RPS there (camera_hold.h
                                            camHoldYawOk, cam_vz_obs.h camVzObsSetTurn) */
#define AUTO_MV_YAW_ACC_DPS2     60.0f   /* deg/s^2 (30 before): the same 0.5 s ramps */
#define AUTO_MV_TURN_GATE_RPS    0.785f  /* rad/s (45 deg/s): the per-frame yaw gate of the hold and the observer
                                            in a commanded turn. The hand-held rotation recording behind the
                                            rotation model (flowcal1, 2026-10-10) kept |dxRes| <= 1.24 px (p90
                                            0.9) and the fit ok up to 45 deg/s; the turns of 2026-10-13 ran
                                            2.0 deg/s (sd) around the commanded 15, at most +3.8 */
#define AUTO_MV_DRAG_PER_S       0.30f   /* 1/s: drag in the feedforward lean (stage C estimate 0.2-0.5) */
#define AUTO_MV_G                9.81f
#define AUTO_MV_CAM_LAG_MS       120     /* the hold's reference is the profile this long ago (frame age) */
#define AUTO_MV_SETTLE_MAX_MS    3000    /* after the profile: settle at most this long ... */
#define AUTO_MV_SETTLE_TOL       0.10f   /* ... or until within this fraction of the move (+ AUTO_MV_SETTLE_TOL_M) */
#define AUTO_MV_SETTLE_TOL_M     0.04f
#define AUTO_MV_YAW_DWELL_MS     500     /* after a turn: the observer reports its tracker reset within this */
#define AUTO_MV_BRAKE_ACC        0.15f   /* m/s^2: a STOPped move brakes with this feedforward lean (0.9 deg) for
                                            v / this, on top of the hold's own braking (weak far from the scene) */
/* A MOVE stopped by an in-air contact (AUTO_MVE_CONTACT; 2026-10-15 verification of "Claude decides"): the hold's
 * anchor is this far back along the move's horizontal direction (the way it came: space it flew through), not at the
 * contact point. Anchored at the contact point, the hold's integral walked the drone back onto what it hit after the
 * brake had pushed it off (the verifier's wall sim: 18 of 25 runs back within 0.01 m of the obstacle 7-18 s after
 * the contact, touching it until Claude acted). Not capped at the move's travel: a contact early in a move (the drone
 * had drifted next to what it hit) puts the anchor up to this far behind the move's start, where the drone hovered
 * and came from (Claude's runbook backs off 0.2-0.3 m the same way; capped at the travel + 0.05 m, two of 24 sim runs
 * stayed within 0.05 m of the obstacle). A back move's anchor (toward the scene ahead) keeps at least
 * the move floor (mvfloor) to the scene. A pure vertical move has no horizontal anchor offset. */
#define AUTO_MV_CONTACT_BACK_M   0.25f

/* ---- scene distance -------------------------------------------------------- */
#define AUTO_MV_FY_PX            99.79f  /* vertical focal length, px at 160x120 (cam_vz_obs kyGy): D = f / G */
#define AUTO_MV_D_MIN_M          0.30f   /* a D outside this range is not valid */
#define AUTO_MV_D_MAX_M          6.0f
#define AUTO_MV_SIM_D_M          2.0f    /* DRY and BENCH on the sim plant: a nominal scene (no camera in the air) */

/* ---- re-track bob after a turn ------------------------------------------- */
/* A turn makes the observer restart its scale tracker (yaw > 0.35 rad
 * accumulated): G, so D, is lost until vertical motion excites the tracker
 * again (a hover's own vertical motion, hp rms ~0.02 m/s on the logged
 * hovers, is below its 0.03 m/s need). After a turn whose tracker reset (or
 * that left the tracker without a G) a small vertical bob z = A (1 - cos wt)
 * runs, whole cycles only (it ends where it started), until the tracker has a
 * G again, at most AUTO_MV_BOB_CYCLES cycles. Through the turn and the bob the
 * camera height estimate may use the tracker's old G as a guess (the glue's
 * camVzGlueVzInput(allowGuess), out.camGuessOk); after them it must be healthy
 * again (the state machine's camera rule: CAM_LOST, its SINK after 5 s). A bob
 * that ends without a G turns the drone back to its start heading once
 * (2026-10-15: that view had one), then re-tracks there (end UNDONE). The bob goes on one whole
 * cycle after the tracker has a G again (its first G is rough). On the
 * observer's synthetic flight (test_cam_vz_obs.c testRetrackBob) a still hover
 * never re-tracks within 10 s, the bob within ~1.5 s, its G within ~10 % after
 * one more cycle. */
#define AUTO_MV_BOB_A_M          0.04f
#define AUTO_MV_BOB_HZ           0.5f
#define AUTO_MV_BOB_CYCLES       4

/* ---- guards ---------------------------------------------------------------- */
#ifdef ESPFLY_NATIVE_VGA
#define AUTO_MV_VIEW_MAX_AGE_MS CAM_CADENCE_VGA_MAX_AGE_MS
#else
#define AUTO_MV_VIEW_MAX_AGE_MS  300     /* pause while the hold is not applied on a frame this fresh */
#endif
#define AUTO_MV_NO_CAM_LAND_MS   2000    /* ... this long: LAND MOVE_NO_CAM */
#define AUTO_MV_TILT_LAND_DEG    8.0f
#define AUTO_MV_TILT_LAND_MS     100
/* The progress guard's margins (the defaults of NVS mvprogm / mvprogr / mvlatm since 2026-10-17, "guard margins"
 * below): PROGRESS when |measured - reference| > the margin. Forward (rel units): mvprogm / max(D_end, D_MIN) + mvprogr
 * x |the move's ln target|; a back move's at most AUTO_MV_PROG_MARGIN_M / AUTO_MV_PROG_MARGIN_REL (its overshoot is
 * blind: the 300 % back move of 2026-10-17 went 0.45 m past its target uncaught). Sideways (metres, below):
 * max(mvlatm, mvprogm + mvprogr x |lateral m|). These three are the values before 2026-10-17 (0.30 m + 50 % in
 * 591f176: a move of 0.60 m or less could never trip it): a back move's forward cap, and the replays of the flights
 * flown with them (tools/tests/test_motion_replay.c). */
#define AUTO_MV_PROG_MARGIN_M    0.15f
#define AUTO_MV_PROG_MARGIN_REL  0.30f
/* The sideways guard in metres at the reference's current distance (2026-10-14, docs/autonomy-a2b.md 21.2):
 * PROGRESS when |progLat - rLat| x max(dLat0 - max(sFwdM, 0), AUTO_MV_LAT_GUARD_D_MIN_M) > margLatM,
 * margLatM = max(mvlatm, mvprogm + mvprogr x |lateral m|), the same in RUN and SETTLE (sFwdM: the lagged reference's
 * forward metres). The measured lateral motion is the integral of v_right / D_now, so its error in metres is the
 * radians times the distance now: 5 of the 7 PROGRESS stops of 2026-10-14 were 0.15 m of sideways hover wander on
 * straight forward moves, and near the end of a long approach D_now / D0 is 0.3-0.5, where the old radian margin
 * (0.15 m + 30 % at the start distance; SETTLE 0.25 m) was 2-3x tighter in metres. The floor never binds on an
 * accepted move: autoMotionMove refuses (TOO_CLOSE) a forward move that would end nearer than it by the lateral D,
 * and the scene floor below already keeps >= the move floor (mvfloor^0.6 x D_low^0.4 >= mvfloor at D_low >= mvfloor;
 * >= 0.30 m at every mvfloor), so the step uses dLat0 - max(sFwdM, 0) as it is. */
#define AUTO_MV_SETTLE_LAT_M     0.25f   /* the sideways margin's floor before 2026-10-17 (user decision 2026-10-13,
                                            evening: the tiger2 forward move was stopped in its settle by 0.151 m of
                                            sideways hover wander against 0.15 m; RUN too since 2026-10-14) */
#define AUTO_MV_LAT_GUARD_D_MIN_M 0.30f  /* the sideways guard's distance floor (enforced at acceptance) */
/* The live front guard (2026-10-14, docs/autonomy-a2b.md 21.2): a forward MOVE (fwdM > 0) is STOPPED with end
 * AUTO_MVE_FRONT, in RUN and SETTLE (FLIGHT, the camera on this epoch), when its measured forward progress passes
 * progFwdMax = AUTO_MV_K_LOW x ln(D_low / mvfstop) (fixed at acceptance; other moves: never; with mvfix FRONT, the
 * review of 2026-10-17, every horizontal MOVE: a sideways or back move from the 0.30 m floor had only its forward
 * PROGRESS margin, ~0.11 m to the scene at k 0.6, and a 0.10-0.20 m/s^2 push reached the scene; docs/autonomy-a2b.md
 * 34.1. A vertical move or a turn holds as a hover does): at the lowest expansion
 * scale k that is the scene ahead at mvfstop (default AUTO_MV_FRONT_STOP_CM), whatever the move's length. An accepted
 * move's target is at most AUTO_MV_K_LOW x ln(D_low / mvfloor), AUTO_MV_K_LOW x ln(mvfloor / mvfstop) below it (0.109
 * at 0.30 / 0.25; 0.134 at the 0.50 / 0.40 before 2026-10-17), so an ideally tracked move never trips it; the forward
 * PROGRESS margin's ahead side grows with the move, this does not. Braking from the fastest a tracked approach can
 * close at that distance (the image rates: v <= 0.30 x / k at the hold's 0.30 /s expansion gate) leaves 0.13-0.16 m at
 * k 0.6 (docs/autonomy-a2b.md 32.2). */
#define AUTO_MV_FRONT_STOP_CM    25      /* the default of NVS mvfstop (40 until 2026-10-17) */
#define AUTO_MV_FRONT_STOP_M     ((float)AUTO_MV_FRONT_STOP_CM / 100.0f)
/* A frozen view (frames that pass the checks but show no motion; docs/autonomy-a2b.md 21.2): in RUN and SETTLE
 * (FLIGHT, the camera on this epoch), a forward MOVE whose lagged reference is past AUTO_MV_FROZEN_REF (ln) while
 * the measured forward progress is under AUTO_MV_FROZEN_FRAC of it is STOPPED (PROGRESS). The forward PROGRESS
 * margin's behind side grows with 30 % of the move: in the sim a frozen 3.3 m approach from D 4.7 m (and 4 m from
 * 5.5 m) tripped it only ~10 s in, the hold's push carrying the drone into the scene ahead in 2 of 4 seeds (the
 * front guard reads the camera too; b6a54da's gain path the same); with this rule it stops 2.6-3.9 s in, <= 1.3 m
 * out. The hold drives the measured progress to the reference (k >= 0.6 or not), so a tracking move reads near 1 x
 * it; 0.06 instead of 0.10 stopped 3.4 % of the long-move grid's genuine moves (0.10: 1.1 %). */
#define AUTO_MV_FROZEN_REF       0.10f
#define AUTO_MV_FROZEN_FRAC      0.25f
#define AUTO_MV_SETTLE_MIN_PROG  0.50f   /* the settle timed out with less measured progress along the move than
                                            this fraction: STOP PROGRESS (a frozen view must not keep its target);
                                            more, but short of the target by more than the settle tolerance or off
                                            its line by more than mvprogm: SHORT */
#define AUTO_MV_UP_MARGIN_M      0.15f   /* vertical: |height change - reference| > this + 0.5 x the move: STOP */

/* ---- fast moves (mvspeed; user request 2026-10-16 "you should be fast", docs/autonomy-a2b.md 31) ----------------
 * One NVS setting, mvspeed (%, AUTO_MV_SPEED_MIN_PCT..AUTO_MV_SPEED_MAX_PCT, default AUTO_MV_SPEED_DEFAULT_PCT), latched
 * at ARM (autonomy.c, autoFsmSetMvSpeed). s = pct / 100; each limit scales with s up to its own cap:
 *   V_H     min(AUTO_MV_V_H_MAX s, AUTO_MV_FAST_V_H_CAP)       sideways (and diagonals)
 *   V_BACK  min(AUTO_MV_V_H_MAX s, AUTO_MV_FAST_V_BACK_CAP)    the component away from the scene
 *   V_FWD   min(AUTO_MV_V_FWD_MAX s, AUTO_MV_FAST_V_FWD_CAP)   the component toward the scene
 *   A_H     min(AUTO_MV_A_H s, mvacc / 100)                   (NVS mvacc, default AUTO_MV_FAST_A_H_CAP: below)
 *   W_LAT   min(AUTO_MV_W_LAT_MAX s, AUTO_MV_FAST_W_LAT_CAP)   (0.97 px/frame against the 1.5 px gate)
 *   W_FWD   min(AUTO_MV_W_FWD_MAX s, AUTO_MV_FAST_W_FWD_CAP)   checked on the profile's true peak of v / (D - s) (the
 *           closed form in auto_motion.c fwdPeakCap), not at the end distance
 *   BRAKE   min(AUTO_MV_BRAKE_ACC s, AUTO_MV_FAST_BRAKE_CAP)
 *   SETTLE  max(mvsettle, 3000 / s) ms (NVS mvsettle, default AUTO_MV_FAST_SETTLE_MIN_MS: below);
 *           YAW_DWELL max(AUTO_MV_FAST_DWELL_MIN_MS, 500 / s) ms
 *   YAW     min(30 s, AUTO_MV_FAST_YAW_RATE_CAP) deg/s, min(60 s, AUTO_MV_FAST_YAW_ACC_CAP) deg/s^2 (the 45 deg/s turn
 *           gate is unchanged)
 *   BOB     min(0.20 s, AUTO_MV_FAST_BOB_ACC_CAP) m/s^2 (the cap is 0.20: the bob as at 100)
 * plus, only above 100: the progress margins grow by AUTO_MV_FAST_T_TRACK_S x the profile's peak speed (the hold's
 * lead), the frozen-view reference is AUTO_MV_FROZEN_REF + AUTO_MV_FAST_T_TRACK_S x W_FWD, and a RUN that loses its
 * view is stopped with its brake (AUTO_MVE_VIEW_FAST) instead of coasting in a pause when the view stays lost
 * AUTO_MV_FAST_VIEW_DEB_MS (a refused frame or two only pause it, as at 100) and the reference ran faster than
 * AUTO_MV_FAST_PAUSE_V then or AUTO_MV_FAST_T_TRACK_S before (the drone lags it). At 100 every value is the header
 * constant and no new code path runs (bit-identical to the build before it with the margins before 2026-10-17:
 * tools/tests/test_auto_motion.c testSpeedIdentity; the near-scene braked view loss of the "guard margins" runs at every
 * step). The guard distances (mvfstop,
 * mvfloor, K_LOW; NVS since 2026-10-17), the tilt guard, the view age, the length caps, the vertical limits and CAM_LAG are the same
 * at every step. */
#define AUTO_MV_SPEED_MIN_PCT        100
#define AUTO_MV_SPEED_MAX_PCT        300
#define AUTO_MV_SPEED_DEFAULT_PCT    100
#define AUTO_MV_FAST_V_H_CAP         0.40f   /* m/s: the camera does not see to the sides (0.60 until the review of
                                                cbccec5: the sideways contact speed at most 1.33x 0.30, until flights) */
#define AUTO_MV_FAST_V_BACK_CAP      0.40f   /* m/s: the camera does not see behind */
#define AUTO_MV_FAST_V_FWD_CAP       0.50f   /* m/s */
#define AUTO_MV_FAST_A_H_CAP         0.20f   /* m/s^2, the default of mvacc (1.2 deg of feedforward lean; 0.30 until
                                                the 300 % forensics of 2026-10-17: the speed-up and brake leans ran
                                                ahead of a hold that did not follow them, docs/autonomy-a2b.md 33; an
                                                NVS setting since 2026-10-17, below) */
#define AUTO_MV_FAST_W_LAT_CAP       0.14f   /* 1/s */
#define AUTO_MV_FAST_W_FWD_CAP       0.17f   /* 1/s: the hold's smoothed expansion gate is 0.3 /s; at 0.20 the sim's
                                                k 0.6 approaches put its EMA at 0.29 (review of cbccec5), at 0.17 at
                                                ~0.25 */
#define AUTO_MV_FAST_BRAKE_CAP       0.40f   /* m/s^2 */
#define AUTO_MV_FAST_SETTLE_MIN_MS   2500    /* the default of mvsettle: the hold's response (3-6 s; 1000 until the
                                                300 % forensics of 2026-10-17: its 1.0 s settle timed out with the
                                                drone still sliding, move 1 SHORT, move 2 DONE 0.2 m past;
                                                docs/autonomy-a2b.md 33; an NVS setting since 2026-10-17, below) */
#define AUTO_MV_FAST_YAW_RATE_CAP    35.0f   /* deg/s: the 45 deg/s per-frame turn gate over the logged overshoot
                                                (30 deg/s turns ran 36-38 deg/s over a frame; 40 until the review of
                                                cbccec5) */
#define AUTO_MV_FAST_YAW_ACC_CAP     70.0f   /* deg/s^2: the ramps' overshoot near today's 60 (120 until that review) */
#define AUTO_MV_FAST_DWELL_MIN_MS    300     /* the observer reports its tracker reset within ~1-2 frames at 94 ms age */
#define AUTO_MV_FAST_BOB_ACC_CAP     0.20f   /* m/s^2: the bob as at 100 (0.40 until the review of cbccec5: twice the
                                                vertical thrust demand at the end of the battery; the early return
                                                hides the bob anyway) */
#define AUTO_MV_FAST_T_TRACK_S       0.40f   /* s: the hold's tracking lead at speed (margins, frozen view) */
#define AUTO_MV_FAST_PAUSE_V         0.20f   /* m/s: a lost view above this reference speed stops the move (VIEW_FAST) */
#ifdef ESPFLY_NATIVE_VGA
#define AUTO_MV_FAST_VIEW_DEB_MS CAM_CADENCE_VGA_FAST_VIEW_MS /* 323 at QVGA: three refused frames in a row recover
                                                                (camera_cadence.h; 231 until 2026-10-22) */
#else
#define AUTO_MV_FAST_VIEW_DEB_MS     150     /* original camera profile */
#endif
#define AUTO_MV_FRONT_VIEW_DEB_MS    150     /* near-scene FRONT brake: original response at every cadence */
/* The VIEW_FAST timer as a setting (NVS mvview, ms; auto_glue_core.c AUTO_SET_MVVIEW; 2026-10-22): 0 the profile's
 * AUTO_MV_FAST_VIEW_DEB_MS, else at least that (the timer only ever gets longer: 1..default - 1 refused) and at most
 * AUTO_MV_VIEW_DEB_MAX_MS, latched at ARM (autoMotionSetViewDeb); the value in force is auto status mvView. The
 * FRONT brake's 150 ms and the NO_VIEW stop after AUTO_MV_NO_CAM_LAND_MS are not settings. */
#define AUTO_MV_VIEW_DEB_MAX_MS     1000
#define AUTO_MV_BOB_ACC              0.20f   /* m/s^2: the re-track / re-view bob's legs at 100 % */

/* ---- the fast steps' acceleration and settle as settings (user request 2026-10-17 "no, I want real 300%";
 * docs/autonomy-a2b.md 35) -----------------------------------------------------------------------------------------
 * The two caps the 300 % forensics lowered (A_H 0.30 -> 0.20 m/s^2, the settle floor 1000 -> 2500 ms, so 250 and 300 %
 * flew as 200 %) are NVS settings (auto_glue_core.c AUTO_SET_MVACC, AUTO_SET_MVSETTLE; `espfly_diag.py auto set mvacc
 * 30`), read at the next ARM with mvspeed (autonomy.c sortieSpeedLatch: autoMotionSetFastCaps, before
 * autoFsmSetMvSpeed), used above 100 % only:
 *   mvacc     cm/s^2  A_H's cap above 100: min(AUTO_MV_A_H s, mvacc / 100); the profile's speed-up and slowing (and so
 *                     their feedforward leans: 0.30 m/s^2 is 1.75 deg); the expansion cap's closed form uses it too
 *   mvsettle  ms      the settle's floor above 100: max(3000 / s, mvsettle)
 * The defaults are AUTO_MV_FAST_A_H_CAP and AUTO_MV_FAST_SETTLE_MIN_MS: the motions of d3bedee bit for bit
 * (tools/tests/test_auto_motion.c testFastCapsIdentity). mvacc 30 with mvsettle 1000 are the caps of 4dea542 (the
 * 300 % flights of 2026-10-17 morning): every profile of its golden (tools/tests/mvspeed_golden_4dea542.json) and the
 * settle of each step as then; the move fixes and the margins of d3bedee stay (mvfix, mvfloor ...). At 100 neither is
 * read. The latched pair lives in auto_motion.c (8 B of .data, as the margins: autoMotion_t is unchanged); only the
 * once-per-command code (autoMvSpeedCfg) and the ARM's autoMotionSetSpeed read it (flash). */
#define AUTO_MV_ACC_DEFAULT_CM       20      /* AUTO_MV_FAST_A_H_CAP in cm/s^2 */
#define AUTO_MV_ACC_MIN_CM           10      /* AUTO_MV_A_H: never under the 100 % acceleration */
#define AUTO_MV_ACC_MAX_CM           40      /* the range asked for; A_H s is at most 0.30 at 300 %, so 31..40 plan as
                                                30 (only a larger AUTO_MV_SPEED_MAX_PCT would reach them) */
#define AUTO_MV_SETTLE_FLOOR_MIN_MS  1000    /* 4dea542's floor (the 300 % flights) */
#define AUTO_MV_SETTLE_FLOOR_MAX_MS  3000    /* AUTO_MV_SETTLE_MAX_MS: the settle at 100 */

/* The limits of one speed step (autoMvSpeedCfg). */
typedef struct {
  uint16_t pct;             /* the step, clamped to AUTO_MV_SPEED_MIN_PCT..AUTO_MV_SPEED_MAX_PCT */
  bool fast;                /* pct above AUTO_MV_SPEED_DEFAULT_PCT: the fast rules apply */
  float vH, vBack, vFwd;    /* m/s */
  float aH;                 /* m/s^2 */
  float wLat, wFwd;         /* 1/s */
  float brake;              /* m/s^2 */
  uint16_t settleMaxMs, dwellMs;
  float yawRate, yawAcc;    /* deg/s, deg/s^2 */
  float bobAcc;             /* m/s^2 */
  float tTrack;             /* s (0 at 100) */
  float frozenRef;          /* ln */
} autoMvSpeed_t;

/* ---- guard margins (user decisions 2026-10-17: "you are too scared to approach things ... you can take risks",
 * "the drone is tiny and harmless"; docs/autonomy-a2b.md 32) -------------------------------------------------------
 * Five NVS settings (auto_glue_core.c AUTO_SET_MVFLOOR .. AUTO_SET_MVLATM; `espfly_diag.py auto set mvfloor 40`), read
 * at the next ARM with mvspeed (autonomy.c sortieSpeedLatch: autoMvMarginsFrom, autoMotionSetMargins), the same at
 * every speed step:
 *   mvfloor  cm  a forward MOVE's floor: the scene ahead at its true end at k AUTO_MV_K_LOW (TOO_CLOSE below it); the
 *                TAKEOFF's plan keeps mvfloor + AUTO_PLAN_FRONT_EXTRA_CM (the Mac's), a back move's contact back-off
 *                anchor mvfloor
 *   mvfstop  cm  the live front stop (FRONT), at most mvfloor - AUTO_MV_FRONT_STOP_GAP_CM (clamped at the latch)
 *   mvprogm  cm  the progress guard's base margin, and the SHORT end's off-the-line limit
 *   mvprogr  %   the progress guard's part of the move
 *   mvlatm   cm  the sideways margin's floor
 *   mvnear   cm  a forward MOVE whose view is lost with the scene this near (k AUTO_MV_K_LOW) brakes: FRONT (0 off;
 *                with mvfix FRONT any horizontal MOVE's)
 * The values before 2026-10-17 (50 / 40 / 15 / 30 / 25 / 0 and the plan's 60: autoMvMarginsOld) give the motions of before
 * bit for bit (tools/tests/test_auto_motion.c testSpeedIdentity). The latched margins live in auto_motion.c, not in
 * autoMotion_t: the motion memory's layout (the IRAM code's offsets, autoFsm_t's after it) is unchanged, and only the
 * once-per-command code reads them (flash; the step reads the per-move margins it already had). */
#define AUTO_MV_MG_FLOOR_MIN_CM   20
#define AUTO_MV_MG_FLOOR_MAX_CM   100
#define AUTO_MV_MG_STOP_MIN_CM    15
#define AUTO_MV_MG_STOP_MAX_CM    80
#define AUTO_MV_FRONT_STOP_GAP_CM 5      /* mvfstop <= mvfloor - this: an ideally tracked move ends >= 0.6 ln(floor /
                                            stop) short of the stop */
#define AUTO_MV_PROG_MARGIN_CM    20     /* the default of mvprogm (AUTO_MV_PROG_MARGIN_M 0.15 until 2026-10-17) */
#define AUTO_MV_MG_PROG_MIN_CM    10
#define AUTO_MV_MG_PROG_MAX_CM    60
#define AUTO_MV_PROG_MARGIN_PCT   40     /* the default of mvprogr (AUTO_MV_PROG_MARGIN_REL 0.30 until 2026-10-17) */
#define AUTO_MV_MG_PCT_MIN        10
#define AUTO_MV_MG_PCT_MAX        100
#define AUTO_MV_LAT_MARGIN_CM     40     /* the default of mvlatm (AUTO_MV_SETTLE_LAT_M 0.25 until 2026-10-17: the
                                            approach steps near a bright window of 2026-10-17 stopped PROGRESS at
                                            0.25-0.36 m sideways) */
#define AUTO_MV_MG_LAT_MIN_CM     15
#define AUTO_MV_MG_LAT_MAX_CM     100
#define AUTO_MV_NEAR_SCENE_CM     80     /* the default of mvnear (0: off, the motions before 2026-10-17; 60 until the
                                            review of 2026-10-17: the measured progress lags the drone ~0.1-0.2 s, and a
                                            300 % approach whose gate tripped at a true 0.54 m read 0.85 against the
                                            0.86 of 0.60 m, paused, coasted into the scene; docs/autonomy-a2b.md 34.2): a forward
                                            MOVE whose view is lost AUTO_MV_FAST_VIEW_DEB_MS with its measured progress
                                            putting the scene ahead within this at k AUTO_MV_K_LOW is STOPPED FRONT with
                                            its brake from at least AUTO_MV_FAST_PAUSE_V, at every mvspeed: near a scene
                                            the hold's 0.3 /s expansion gate refuses the frames of an approach faster
                                            than ~0.5 x the distance (k 0.6) before the live front stop can trip, and a
                                            pause coasted into the scene (docs/autonomy-a2b.md 32.2) */
#define AUTO_MV_MG_NEAR_MAX_CM    150

/* ---- move fixes (the 300 % forensics of 2026-10-17, firmware-backups/handoff-2026-10-12/overshoot300-verdict.md;
 * docs/autonomy-a2b.md 33) -------------------------------------------------------------------------------------------
 * One NVS setting, mvfix (a bit mask, default AUTO_MVFIX_DEFAULT = all but STILL; 0: the motions before them, bit for bit with
 * the margins of before: tools/tests/test_auto_motion.c testSpeedIdentity), latched at ARM with the margins
 * (autoMvMargins_t fix). The camera's two (GAIN, BRAKE) reach camera_flow.c through autonomyMvFixBits().
 *   DRIFT  the scene distance a forward / back move measured (dTrack) follows the drift since: the hold's measured
 *          forward progress since dTrack was taken (dTrackRef in the hold's epoch) scales it (k 1; its lower bound at
 *          k AUTO_MV_K_LOW toward the scene), when a command reads it (flash: autoMotionTrackNow). Move 2 of the 300 %
 *          flight was planned at 1.67 m, the camera's own count put the drone at 2.02 m: 1.00 m became 1.21 m.
 *   LONG   a MOVE whose settle times out past its target along the move by more than the settle tolerance ends LONG
 *          (state DONE, as SHORT): the measured end becomes the move's end, the drone holds where it is, the end is
 *          reported (it reported DONE 0.2-0.45 m past, then the hold walked it back unreported). A move with a part
 *          toward the scene (fwdM > 0) keeps its target and only reports LONG (the review of 2026-10-17: re-anchored at
 *          the overshoot, an unlearned forward lean crept it ~0.1 m further toward the scene or a person; its D then
 *          follows the hold back to the target, mvfix DRIFT). A back move's
 *          forward progress margin is in metres at its end distance (D0 + |F|), not at its start (0.9 m there).
 *   STILL  a horizontal MOVE in FLIGHT holds its profile at its start (tRun 0) until the camera hold's measured rates
 *          are under CAM_HOLD_STILL_RATE for CAM_HOLD_STILL_S (autoMvIn_t holdMoving false), at most
 *          AUTO_MV_STILL_CAP_MS: move 2 was accepted at 0.15-0.17 m/s of drift and flew it on top of the move. Off by
 *          default: the replay of the 300 % flights (tools/tests/test_speed300_replay.c) has the hold's filtered rates
 *          at 0.04-0.07 /s (median) in plain hover, so at the forensics' 0.03 /s the start waited the 1.5 s cap 53-90 %
 *          of the time, and 0.7-1.1 s on average at CAM_HOLD_STILL_RATE 0.05; the sim (testDriftLean) shows no gain
 *          from it once DRIFT measures the drift before the acceptance (docs/autonomy-a2b.md 33.4).
 *   GAIN   (camera_hold.h camHoldGainMoveFix) the hold's gain scale in a MOVE toward the scene follows the measured
 *          progress, not the reference's, and above 100 % no 'untracked' cut for CAM_HOLD_GAIN_GRACE_S of a move.
 *   BRAKE  (camera_hold.h camHoldInput_t ffBrakeScale) a move's braking feedforward lean is scaled by the measured
 *          over the reference rate (0..CAM_HOLD_FF_BRAKE_MAX): the planned brake hit a drone at 0.09 m/s, not 0.24.
 *   FRONT  (the review of 2026-10-17, docs/autonomy-a2b.md 34.1) the live front stop (mvfstop) and the near-scene
 *          braked view loss (mvnear) on every horizontal MOVE, not only on one toward the scene: the 0.30 m floor parks
 *          the drone where a sideways or back move had only its forward PROGRESS margin.
 * Not here: the hold's integral at a new epoch (the split reference) stays: it is the lean the hover needs (its
 * position offset is that lean / KI s), and clearing it would restart the drift (docs/autonomy-a2b.md 33.5). */
#define AUTO_MVFIX_DRIFT          0x01u
#define AUTO_MVFIX_LONG           0x02u
#define AUTO_MVFIX_STILL          0x04u
#define AUTO_MVFIX_GAIN           0x08u
#define AUTO_MVFIX_BRAKE          0x10u
#define AUTO_MVFIX_FRONT          0x20u
#define AUTO_MVFIX_DEFAULT        0x3Bu  /* DRIFT, LONG, GAIN, BRAKE, FRONT (STILL off: above); 0x1B before the review */
#define AUTO_MVFIX_MAX            0x3Fu
#define AUTO_MV_STILL_CAP_MS      1500u  /* STILL: the profile starts at the latest this long after its RUN began */

/* ---- guards off (user decision 2026-10-18 ~13:30, "remove the guards or make them absolutely minimum"; docs/autonomy-
 * a2b.md 38) -------------------------------------------------------------------------------------------------------
 * The table approach of 2026-10-18 noon: every long MOVE toward the low table was stopped by the firmware well short
 * of its target (the reference reached 0.75 of 1.75 m, 0.85 of 1.8, 0.55 of 1.3): the near table top expands fast in
 * the lower half of the image, the expansion-based front logic reads the scene as close and stops. The user accepts
 * the contact risk (28 g). So every MOVE distance guard is adjustable down to OFF through the NVS settings (no
 * rebuild), the defaults the behaviour of before, bit for bit (tools/tests/test_auto_motion.c testSpeedIdentity):
 *   mvklow   %   the expansion scale k the floor, the front stop, the near brake and the measured lower bound of D
 *                allow for (AUTO_MV_K_LOW x 100; 60..100): 100 means no expansion safety factor, the plan uses the
 *                measured scene distance as it is (the true end D ((D - F) / D)^(1/k) is then D - F)
 *   mvfloor  0   no planning floor: a forward MOVE may be planned up to the measured scene distance (D - F >= 0.05 m,
 *                AUTO_MVW_NO_DIST / RANGE as before; the sideways guard's distance floor at acceptance goes with it),
 *                the TAKEOFF's plan the same, a back move's contact anchor keeps nothing ahead
 *   mvfstop  0   the live front stop off (progFwdMax 1e30 on every MOVE: never FRONT from it; the near brake mvnear is
 *                its own setting, 0 off, as before)
 *   mvprogm  0 or mvprogr 0   the progress guard off: the sideways and forward margins 1e30 (never PROGRESS from them,
 *                in RUN or SETTLE), the frozen-view rule off (the step's frozenRef 1e30), and with mvprogm 0 no SHORT
 *                for an end off the line (the settle's own SHORT / LONG along the move, and its STOP PROGRESS for less
 *                than half of the move measured, stay: they end a move that has stopped moving, they never stop one)
 *   mvlat    %   the sideways cap |lateral| <= mvlat / 100 x D (AUTO_MV_LAT_D_REL x 100; 0 off: never TOO_WIDE)
 *   mvwmax   %   the expansion rate cap AUTO_MV_W_FWD_MAX x 100 (10..40 /s x 0.01): a forward move's speed at 100 % is
 *                at most this x its end distance; above 100 % min(this x s, max(AUTO_MV_FAST_W_FWD_CAP, this)). The
 *                hold's own expansion gate refuses frames over 0.3 /s (smoothed), so above ~25 the end of an approach
 *                loses its view (NO_VIEW after 2 s, or the near brake): it is the user's call
 * What stays with everything off: the view guards (the pause, NO_VIEW, VIEW_FAST above 100 %, the near brake unless
 * mvnear 0), the tilt stop, the state machine's contact / tilt / rate / fall kills and its battery rules (BATT,
 * LOW_BATT, UP_HEADROOM), the camera-loss SINK, the hold's own limits, the command limits (RANGE), the height floor
 * and top (HEIGHT), the person rule (the Mac's), the vertical progress margin and the re-view bob. */
#define AUTO_MV_K_LOW_PCT         60     /* the default of mvklow (AUTO_MV_K_LOW x 100) */
#define AUTO_MV_MG_KLOW_MIN_PCT   60
#define AUTO_MV_MG_KLOW_MAX_PCT   100
#define AUTO_MV_LAT_D_REL_PCT     50     /* the default of mvlat (AUTO_MV_LAT_D_REL x 100) */
#define AUTO_MV_MG_LAT_REL_MAX_PCT 200   /* (0 off) */
#define AUTO_MV_W_FWD_PCT         15     /* the default of mvwmax (AUTO_MV_W_FWD_MAX x 100) */
#define AUTO_MV_MG_WFWD_MIN_PCT   10
#define AUTO_MV_MG_WFWD_MAX_PCT   40
#define AUTO_MV_GUARD_OFF         1e30f  /* a margin or a progress limit this large never trips */

/* The guard margins in force (autoMotionSetMargins), cm and %; with the move fixes (mvfix). Since 2026-10-18 ("guards
 * off" above) floorCm, planCm, stopCm, progCm and progPct may be 0 (off), and the expansion scale and the sideways cap
 * are here too. */
typedef struct {
  uint8_t floorCm;          /* a MOVE's forward floor (mvfloor; 0 off) */
  uint8_t planCm;           /* the TAKEOFF's planned move's floor (mvfloor + AUTO_PLAN_FRONT_EXTRA_CM; 0 with mvfloor 0) */
  uint8_t stopCm;           /* the live front stop (mvfstop; 0 off) */
  uint8_t progCm;           /* the progress guard's base (mvprogm; 0 off) */
  uint8_t progPct;          /* its part of the move (mvprogr; 0 off) */
  uint8_t latCm;            /* the sideways margin's floor (mvlatm) */
  uint8_t nearCm;           /* the near-scene distance of a forward MOVE's braked view loss (mvnear; 0 off) */
  uint8_t fix;              /* AUTO_MVFIX_* (mvfix; 0: none, the motions before 2026-10-17's forensics) */
  uint8_t kLowPct;          /* the lowest expansion scale x 100 (mvklow; 100: none) */
  uint8_t latPct;           /* the sideways cap x 100 (mvlat; 0 off) */
} autoMvMargins_t;

/* Motion state (AUTO_STATUS mvState). Values go on the wire; only append. */
typedef enum {
  AUTO_MV_IDLE = 0,        /* none since the take-off */
  AUTO_MV_RUN = 1,         /* the reference moves */
  AUTO_MV_PAUSED = 2,      /* the reference waits for the camera (view lost, the hold coasting) */
  AUTO_MV_SETTLE = 3,      /* the profile ended; the drone settles (a turn: the re-track bob) */
  AUTO_MV_DONE = 4,        /* the latest motion ended normally (or with RETRACK_TIMEOUT) */
  AUTO_MV_STOPPED = 5,     /* the latest motion was stopped: STOP, a progress guard, or the sortie left HOVER */
} autoMvState_t;

typedef enum {
  AUTO_MVK_NONE = 0,
  AUTO_MVK_MOVE = 1,
  AUTO_MVK_YAW = 2,
} autoMvKind_t;

/* Why a MOVE or YAW was refused (ACK detail bits 8-15, AUTO_STATUS mvWhy).
 * Values go on the wire; only append. */
typedef enum {
  AUTO_MVW_NONE = 0,
  AUTO_MVW_NOT_WAITING = 1,   /* not hovering in the wait (before the first STABLE photo, TAKEOFF, LAND, ...) */
  AUTO_MVW_BUSY = 2,          /* a motion is running: STOP it or wait for DONE */
  AUTO_MVW_RANGE = 3,         /* |right| > 150 cm, forward > 400 or < -150 cm, sqrt(right^2 + forward^2) > 400 cm,
                                 |up| > 50 cm (or up with |forward| > 150 cm), |deg| > 180, or nothing to do
                                 (autoMvRangeOk; a TAKEOFF's plan too, auto_fsm.h) */
  AUTO_MVW_HEIGHT = 4,        /* the height after the move would leave 0.25..2.00 m (AUTO_MV_H_MAX_M), or a SET_HEIGHT's
                                 corrected estimate 0.10 m..the cap + 0.50 m */
  AUTO_MVW_NO_DIST = 5,       /* no valid scene distance D (not tracked, stale after a turn) */
  AUTO_MVW_TOO_CLOSE = 6,     /* a forward move would leave less than mvfloor by the estimate (the TAKEOFF's plan:
                                 mvfloor + AUTO_PLAN_FRONT_EXTRA_CM) */
  AUTO_MVW_NO_VIEW = 7,       /* the hold is not applied on a fresh view */
  AUTO_MVW_NO_CAM_HEIGHT = 8, /* a vertical move without the camera height estimate in use */
  AUTO_MVW_BATT = 9,          /* the battery-low condition holds, or T_hold > AUTO_MV_REFUSE_T (auto_fsm.h) */
  AUTO_MVW_AIR = 10,          /* the motion and a LAND would not fit in the 120 s air clock (never produced since
                                 2026-10-15: there is no air clock; the value stays on the wire) */
  AUTO_MVW_TOO_WIDE = 11,     /* |lateral| > AUTO_MV_LAT_D_REL x D: split it */
  AUTO_MVW_PLAN_CANCELLED = 12, /* the TAKEOFF's planned move was not started: an event (a contact, NOT_STABLE,
                                   TAKEOFF_SLOW, CAPTURE_FAIL, the camera height lost, a controller event, a loop
                                   gap, ...) came before the hover wait (auto_fsm.c; 2026-10-15 verification): the
                                   drone hovers in the wait, Claude decides from a photo */
  /* SET_HEIGHT (2026-10-15, auto_fsm.h AUTO_SH_*, docs/autonomy-a2b.md 23); it also refuses NOT_WAITING, BUSY,
   * RANGE, HEIGHT (the corrected estimate outside 0.10..cap + 0.50 m), NO_CAM_HEIGHT, NO_VIEW and BATT (any) */
  AUTO_MVW_SH_STEP = 13,      /* |d| over AUTO_SH_MAX_M (or the flight over AUTO_SH_FLY_MAX_M): send 0.5 m, then again */
  AUTO_MVW_SH_RATE = 14,      /* an accepted SET_HEIGHT less than AUTO_SH_GAP_MS ago */
  AUTO_MVW_SH_CONTACT = 15,   /* a contact's hush or its window (the camera height not back AUTO_CAM_BACK_MS since) */
  AUTO_MVW_UP_HEADROOM = 16,  /* a MOVE up or a SET_HEIGHT flying up with less thrust headroom than a climb needs: the
                                 ceiling in force minus T_hold under min(AUTO_MV_UP_HEADROOM_T, AUTO_MV_UP_HEADROOM_REL x
                                 T_hold) (auto_fsm.h, 2026-10-16; the state machine's check, never the voltage) */
} autoMvWhy_t;

/* How the latest motion ended (AUTO_STATUS mvEnd). Values go on the wire; only append. */
typedef enum {
  AUTO_MVE_NONE = 0,
  AUTO_MVE_DONE = 1,
  AUTO_MVE_STOP_CMD = 2,
  AUTO_MVE_PROGRESS = 3,      /* the measured motion left the reference by more than the margin */
  AUTO_MVE_LEFT_HOVER = 4,    /* LAND or KILLED while it ran */
  AUTO_MVE_RETRACK = 5,       /* a turn whose bob ended without a tracked scene distance, after its turn back did
                                 too (the state machine's camera rule then reports CAM_LOST unless the height estimate
                                 is healthy) */
  AUTO_MVE_SHORT = 6,         /* a MOVE whose settle timed out short of its target or off its line (at least half
                                 of it measured along the move): its measured end is its end, the drone holds there
                                 (state DONE; user decision 2026-10-13, evening) */
  AUTO_MVE_FRONT = 7,         /* a forward MOVE stopped by the live front guard (mvfstop): the scene
                                 ahead got close; it holds there (state STOPPED, a snap; 2026-10-14); since 2026-10-17
                                 also its view lost near the scene (mvnear, AUTO_MV_NEAR_SCENE_CM), braking; with
                                 mvfix FRONT any horizontal MOVE's */
  /* "Claude decides" (2026-10-15, docs/autonomy-a2b.md 22): the stops that replaced the motion LANDs */
  AUTO_MVE_CONTACT = 8,       /* an in-air contact: STOPPED, a snap and a brake against the motion's speed */
  AUTO_MVE_NO_VIEW = 9,       /* the view lost AUTO_MV_NO_CAM_LAND_MS while a MOVE ran (MOVE_NO_CAM until then); a
                                 YAW turns back first (UNDONE) */
  AUTO_MVE_TILT = 10,         /* tilt over AUTO_MV_TILT_LAND_DEG for AUTO_MV_TILT_LAND_MS (MOVE_TILT until then) */
  AUTO_MVE_UNDONE = 11,       /* a YAW turned back to its start heading (its re-track found no scene distance, or
                                 its view or a contact stopped it) and re-tracked there: state DONE */
  AUTO_MVE_LOOP_GAP = 12,     /* a stabilizer loop gap over 20 ms (the state machine's) */
  AUTO_MVE_VIEW_FAST = 13,    /* mvspeed > 100 (2026-10-16): a MOVE's view stayed lost AUTO_MV_FAST_VIEW_DEB_MS with its
                                 reference faster than AUTO_MV_FAST_PAUSE_V (then or AUTO_MV_FAST_T_TRACK_S before):
                                 STOPPED with its brake (no PAUSED coast) */
  AUTO_MVE_LONG = 14,         /* (mvfix LONG, 2026-10-17) a MOVE whose settle timed out past its target along the move by
                                 more than the settle tolerance: its measured end is its end, the drone holds there
                                 (state DONE, as SHORT); one with a part toward the scene keeps its target (the hold
                                 walks it back; the review of 2026-10-17) */
} autoMvEnd_t;

/* What autoMotionStep() asks the state machine to do: a stop (2026-10-15; a LAND until then), or a report. The state
 * machine calls autoMotionStop() with AUTO_MVE_NO_VIEW or AUTO_MVE_TILT and reports it; AUTO_MVL_UNDO it reports
 * only (the re-track found no scene distance: the YAW turns back to its start heading, once). */
typedef enum {
  AUTO_MVL_NONE = 0,
  AUTO_MVL_NO_CAM = 1,        /* the view lost for AUTO_MV_NO_CAM_LAND_MS while moving: stop (NO_VIEW) */
  AUTO_MVL_TILT = 2,          /* tilt over AUTO_MV_TILT_LAND_DEG while moving: stop (TILT) */
  AUTO_MVL_UNDO = 3,          /* the YAW's re-track turned it back (RETRACK_UNDO): nothing to stop */
  AUTO_MVL_FAST = 4,          /* mvspeed > 100: a fast MOVE's view stayed lost and the step stopped it with its brake
                                 (end AUTO_MVE_VIEW_FAST; or, any speed, a forward MOVE's near the scene: FRONT,
                                 mvnear): the report (of its end) only, nothing to stop */
} autoMvLand_t;

/* AUTO_STATUS mvFlags. */
#define AUTO_MVF_D_VALID     0x01   /* the scene distance a command would use is valid */
#define AUTO_MVF_D_STALE     0x02   /* a turn or a lateral shift changed the scene: D is invalid until the tracker
                                       has a G of the new view (the re-track or re-view bob) */
#define AUTO_MVF_D_SIM       0x04   /* D is the nominal one of the sim plant (DRY, BENCH-sim) */
#define AUTO_MVF_CAM_GUESS   0x08   /* the camera height estimate may use the tracker's old G (a turn, its bob) */
#define AUTO_MVF_BOB         0x10   /* the re-track (turn) or re-view (lateral shift) bob runs */
#define AUTO_MVF_D_TRACK     0x20   /* the forward D is the one the last forward / back move measured (dTrack) */
#define AUTO_MVF_BLIND_TURN  0x40   /* the YAW was accepted without a view (AUTO_MV_BLIND_YAW_MAX_DEG) */
#define AUTO_MVF_SET_HEIGHT  AUTO_MVF_BLIND_TURN   /* 0x40 with mvKind MOVE (the bit's meaning depends on the kind):
                                                         the MOVE is a SET_HEIGHT's correction (autoMotionSetHeight),
                                                         not Claude's MOVE (2026-10-15) */
#define AUTO_MVF_UNDO        0x80   /* the YAW is turning back (autoMotionStop / its re-track: end UNDONE) */

/* Inputs, sampled by the state machine on every HOVER step. */
typedef struct {
  uint32_t nowMs;
  bool flight;              /* FLIGHT: the camera guards and the measured progress apply (DRY/BENCH: ideal) */
  bool simPlant;            /* the vertical controller flies the sim: D is AUTO_MV_SIM_D_M */
  float sceneDM;            /* the observer's D (m), when sceneTracked */
  bool sceneTracked;        /* the observer's scale tracker has a G (and its estimate is fresh) */
  uint32_t sceneResets;     /* the observer's tracker restarts (yaw) since boot */
  bool holdApplied;         /* the hold's correction is applied (engaged or coasting, fresh) */
  bool holdCoasting;
  uint32_t camFrameAgeMs;
  uint32_t camLagUs;         /* capture-interval midpoint -> fit publication; 0 uses legacy lag */
  float tiltDeg;
  bool camHeight;           /* the vertical controller uses the camera height estimate this step */
  float vzH;                /* m, the vertical controller's height estimate */
  float hTargetM;           /* the take-off height (TAKEOFF heightCm) */
  float progLat, progFwd;   /* the hold's measured motion since the start of epoch holdEpochSeen (rel units) */
  uint32_t holdEpochSeen;   /* the latest epoch the camera has applied */
  uint32_t airMs;           /* the air clock (the move-down floor's drift term) */
  bool battLow;             /* refuse a new MOVE / YAW (BATT): the BATT_LOW condition has held
                               AUTO_VBAT_LOW_REFUSE_MS, or T_hold is above AUTO_MV_REFUSE_T (auto_fsm.h) */
  bool holdMoving;          /* the camera hold's measured rates were over CAM_HOLD_STILL_RATE within the last
                               CAM_HOLD_STILL_S (camera_hold.h camHoldMoving; false without a view): mvfix STILL */
} autoMvIn_t;

/* Outputs, written by autoMotionStep() on every step (all exactly 0 without a motion, epoch kept). */
typedef struct {
  /* the camera hold (camera_hold.h camHoldRef_t) */
  uint32_t epoch;
  bool snap;                /* this epoch began with a snap (the camera restores the integral first) */
  uint32_t snapEpoch;       /* the latest epoch that began with a snap (0: none this sortie) */
  float refLat, refFwd;     /* rel units since the epoch start, AUTO_MV_CAM_LAG_MS ago */
  float rateLat, rateFwd;   /* 1/s, AUTO_MV_CAM_LAG_MS ago */
  float ffRollDeg, ffPitchDeg;   /* feedforward lean now (+roll right side down, +pitch nose up) */
  float dLowM;              /* the latest MOVE's lower bound of the scene distance at its acceptance if it has a part
                               toward the scene (m; else 0): the camera's gain scale during such a MOVE (camera_hold.h
                               camHoldGainMove) */
  /* the vertical controller */
  float vRef;               /* m/s, HOLD vertical speed reference */
  float hOff;               /* m, the commanded height offset from the take-off height (hMax, LAND times) */
  /* the yaw lock */
  float yawOffDeg;          /* deg, + = clockwise (right): the glue sets yaw = take-off heading - this */
  bool camGuessOk;          /* the glue's camVzGlueVzInput(allowGuess) */
  /* status */
  autoMvState_t state;
  autoMvKind_t kind;
  autoMvWhy_t why;          /* of the latest refusal */
  autoMvEnd_t end;          /* of the latest motion */
  uint8_t flags;            /* AUTO_MVF_* */
  uint16_t seq;             /* motions accepted this sortie */
  float progRightM, progFwdM, progUpM;   /* measured progress of the latest motion */
  float tgtRightM, tgtFwdM, tgtUpM;      /* its command */
  float yawTgtDeg;          /* the latest YAW's command */
  float sceneDM;            /* the D a forward MOVE would use now (0: none; a lateral one uses max(this, the
                               observer's)) */
  uint32_t phaseMs;         /* time in the current state */
  bool holdWait;            /* a motion is active: the hover wait does not count down */
} autoMvOut_t;

/* Trapezoid (private). */
typedef struct {
  float dist;               /* >= 0 */
  float tA, tC, vPk, a, T;
} autoMvProf_t;

/* Motion memory (fields private to auto_motion.c; the state machine owns one). */
typedef struct {
  autoMvState_t state;
  autoMvKind_t kind;
  autoMvWhy_t why;
  autoMvEnd_t end;
  uint16_t seq;
  uint32_t epoch;
  bool snap;
  bool backOff;             /* a MOVE stopped by a contact: latTgt / fwdTgt are its anchor offset in the stop's
                               snap epoch (AUTO_MV_CONTACT_BACK_M), the hold's reference until the next motion or
                               snap (2026-10-15; in snap's padding: the fields after it keep their offsets) */
  uint32_t snapEpoch;
  uint32_t stateMs;         /* entry of the current state */
  uint32_t lastMs;          /* previous step (profile time advances by the difference while running) */
  bool started;
  bool stillWait;           /* mvfix STILL: this MOVE's profile waits at its start for a still drone (in started's
                               padding: the fields after it keep their offsets) */
  uint16_t stillMs;         /* mvfix STILL: the low 16 bits of the acceptance's time: the wait's cap counts from it (a
                               PAUSED -> RUN restarts stateMs; review of 2026-10-17; in the same padding) */
  float tRun;               /* s of profile time (pauses excluded) */
  /* horizontal */
  float d0;                 /* m: the forward conversion's D */
  float dLat0;              /* m: the lateral conversion's D */
  float dLow0;              /* m: the lower bound of the scene distance at acceptance */
  float latM, fwdM, lenM;
  autoMvProf_t ph;
  float latTgt, fwdTgt;     /* rel units at the end (a SHORT end: the measured end) */
  float margLatM, margFwd;  /* the sideways margin in metres at the reference's distance (RUN and SETTLE; it
                               replaced the radian margLat in their slot, 2026-10-14), the forward one (rel) */
  float progFwdMax;         /* the live front guard's progress limit (rel; a large value but for forward moves, and
                               with mvfix FRONT for every horizontal MOVE) */
  float dTrack;             /* m: D after the last forward / back move (k = 1; 0: none) */
  float dTrackLow;          /* m: its lower bound (k >= AUTO_MV_K_LOW) */
  float viewShift;          /* lateral shift / D since the last fresh view */
  bool review;              /* the MOVE ends with its re-view bob (D stale until it ends) */
  bool shortEnd;            /* its settle ended SHORT: the re-view bob ends it SHORT too */
  bool upDone;              /* the MOVE's vertical offset is committed */
  /* vertical */
  float upM;
  autoMvProf_t pv;
  float hStart;             /* vzH at the start */
  float hOff;               /* committed offset (moves done) */
  float hOffNow;
  /* yaw */
  float yawCmd;             /* deg, signed */
  autoMvProf_t py;
  float yawOff;             /* committed offset (turns done), deg */
  float yawOffNow;
  bool bob;
  autoMvProf_t pb;          /* one bob leg (planned at the command, so the step plans nothing) */
  float bobH0;              /* the height estimate at the bob's start */
  float bobT;               /* s of bob time */
  int32_t bobLeg;           /* the bob's leg on the previous step (a cycle ends entering an even one) */
  int32_t bobTrackedLeg;    /* the leg the tracker had a G again from (-1: not yet) */
  bool turnDone;
  bool dStale;
  /* guards */
  bool viewBad;
  uint32_t viewBadMs;
  bool tiltOn;
  uint32_t tiltMs;
  float progLat, progFwd, progUp;   /* rel, rel, m: the latest measured progress (kept at its end value once the
                                       motion has ended) */
  float brakeLat, brakeFwd; /* m/s: the reference's speed when a move was stopped (the brake's direction) */
  float brakeS;             /* s of braking left */
  float fwdUnitPos;         /* max(fwdM, 0) / lenM: the sideways guard's sFwdM is s x this (horizAt) */
  float dLowFwd;            /* a MOVE toward the scene (fwdM > 0): dLow0, else 0 (autoMvOut_t dLowM) */
  float fwdLnBase;          /* rel: the measured forward progress ln(D0 / D) of the MOVEs before the latest motion,
                               summed since ARM (accept() adds each one's end value; autoMotionFwdLn) */
  /* 2026-10-15 (docs/autonomy-a2b.md 22), last: the fields above keep their offsets in the IRAM code */
  float yawOff0;            /* the YAW's start heading offset (its undo turns back to it) */
  bool blind;               /* the YAW was accepted without a view: its turn skips the view guard */
  bool undo;                /* the YAW is turning back (or has: its end is UNDONE) */
  uint8_t yawFlags;         /* AUTO_MVF_BLIND_TURN, AUTO_MVF_UNDO of the latest YAW, AUTO_MVF_SET_HEIGHT of a
                               SET_HEIGHT's MOVE (0 for Claude's MOVE) */
  uint8_t doneEnd;          /* autoMvEnd_t of the latest motion's normal end: DONE, or UNDONE after a turn back */
  /* SET_HEIGHT (2026-10-15), appended: the correction's d and the committed offset before it (its DONE commits
   * shOff0 + (d + upM), exactly shOff0 for the usual upM = -d) */
  float shD, shOff0;
  /* mvspeed (2026-10-16), appended: the sortie's speed step and the limits the step reads (autoMotionSetSpeed; the
   * header constants after autoMotionInit) */
  uint16_t speedPct;
  uint16_t settleMaxMs;     /* AUTO_MV_SETTLE_MAX_MS at 100 */
  uint16_t dwellMs;         /* AUTO_MV_YAW_DWELL_MS at 100 */
  bool viewFastArm;         /* mvspeed > 100: this MOVE's pause is checked for VIEW_FAST after AUTO_MV_FAST_VIEW_DEB_MS
                               (set by the RUN -> PAUSED transition, cleared by the check; at 100 only for a forward
                               MOVE near the scene, mvnear, since 2026-10-17; in the padding after dwellMs: the struct's
                               size is unchanged) */
  float brakeAcc;           /* AUTO_MV_BRAKE_ACC at 100 */
  float frozenRef;          /* AUTO_MV_FROZEN_REF at 100 */
  /* mvfix DRIFT (2026-10-17), appended: dTrack is the scene distance at this measured forward progress of the hold's
   * current epoch (0 at a new epoch: trackD, the commands' autoMotionTrackNow move it along with the drift) */
  float dTrackRef;
  /* A scale reset can arrive after a small turn has reported DONE. Keep
   * the recovery in that turn's bounded re-track state, once per command. */
  uint32_t yawResetSeen;
  bool lateYawRecoveryUsed;
} autoMotion_t;

/* The camera height observer's gain follows the scene distance through forward moves (2026-10-15 review,
 * docs/autonomy-a2b.md 21.9): the observer turns the vertical image motion into speed with G = f / D, which only
 * vertical motion re-measures, so through a level move toward the scene the G it latched at D0 made the speed read
 * D0 / D_now too fast (3.4-4.2 x at the end of the largest moves at k 1, 8-11 x at k 0.6; the logged moves kept G
 * within 0.2 px/m while the reference reached 0.07-0.51). This is the measured forward progress of every MOVE since
 * ARM (the running one's so far; 0 without a move): the observer scales the G it latched by exp(the change since it
 * latched it) (cam_vz_obs.h camVzObsFrame_t fwdLn). Inline: autoFsmStep's IRAM. */
static inline __attribute__((always_inline)) float autoMotionFwdLn(const autoMotion_t *m)
{
  return m->fwdLnBase + (m->kind == AUTO_MVK_MOVE ? m->progFwd : 0.0f);
}

/* mvfix DRIFT (auto_motion.h "move fixes"): dTrack moved by the drift the camera hold measured since it was taken (in
 * FLIGHT, the hold on the current epoch; its reference then this epoch's progress now). Each command does it first (a
 * MOVE, a SET_HEIGHT); the state machine before a contact's snap without a motion (the snap starts a new epoch). No
 * dTrack, DRY, BENCH, the bit off: nothing. Flash. */
void autoMotionTrackNow(autoMotion_t *m, const autoMvIn_t *in);
/* At ARM: everything zero (epoch 0, no offsets), the speed step AUTO_MV_SPEED_DEFAULT_PCT (the header's limits). */
void autoMotionInit(autoMotion_t *m);
/* mvspeed: the limits of the speed step pct (clamped to AUTO_MV_SPEED_MIN_PCT..AUTO_MV_SPEED_MAX_PCT; at 100 and below
 * the header constants themselves; above 100 A_H and the settle with the latched mvacc / mvsettle,
 * autoMotionSetFastCaps). Flash; pure but for that pair. */
void autoMvSpeedCfg(uint16_t pct, autoMvSpeed_t *c);
/* Guard margins (auto_motion.h "guard margins"): the defaults (AUTO_MOVE_MIN_FRONT_CM, AUTO_PLAN_MIN_FRONT_CM,
 * AUTO_MV_FRONT_STOP_CM, AUTO_MV_PROG_MARGIN_CM, AUTO_MV_PROG_MARGIN_PCT, AUTO_MV_LAT_MARGIN_CM, AUTO_MV_NEAR_SCENE_CM);
 * the values before 2026-10-17 (50, 60, 40, 15, 30, 25, 0); the ones the NVS settings give (each clamped to its range, the plan mvfloor +
 * AUTO_PLAN_FRONT_EXTRA_CM, the stop at most mvfloor - AUTO_MV_FRONT_STOP_GAP_CM). Flash, pure. */
void autoMvMarginsDefault(autoMvMargins_t *g);
void autoMvMarginsOld(autoMvMargins_t *g);
void autoMvMarginsFrom(int32_t floorCm, int32_t stopCm, int32_t progCm, int32_t progPct, int32_t latCm,
                       int32_t nearCm, int32_t fix, autoMvMargins_t *g);
/* The guards-off settings into g (auto_motion.h "guards off": NVS mvklow, mvlat), each clamped to its range (0 stays 0
 * for mvlat: off). autoMvMarginsFrom, autoMvMarginsDefault and autoMvMarginsOld leave them at their defaults (60, 50).
 * Flash, pure. */
void autoMvMarginsExtra(autoMvMargins_t *g, int32_t kLowPct, int32_t latPct);
/* The margins every later command plans with (the glue at ARM, after autoFsmSetMvSpeed; NULL: the defaults). Each
 * field is clamped to its range (the plan's to mvfloor..AUTO_MV_MG_FLOOR_MAX_CM + AUTO_PLAN_FRONT_EXTRA_CM, the stop
 * below the floor; 0 stays 0 where it means off: floor, plan, stop, progress base and part, the sideways cap). A
 * RUNNING motion keeps the margins it was accepted with. autoMotionInit does not touch them. The margins in force:
 * autoMotionGetMargins. Flash. */
void autoMotionSetMargins(const autoMvMargins_t *g);
void autoMotionGetMargins(autoMvMargins_t *g);
/* The expansion rate cap (auto_motion.h "guards off": NVS mvwmax, x 0.01 /s), clamped to its range: autoMvSpeedCfg reads
 * it from the next call on, so the glue sets it at ARM before autoFsmSetMvSpeed (as the fast caps). autoMotionInit does
 * not touch it; the get gives the value in force. Flash. */
void autoMotionSetFwdRate(int32_t wPct);
uint8_t autoMotionGetFwdRate(void);
/* The VIEW_FAST timer (auto_motion.h "fast moves": NVS mvview, ms): 0 (and anything below the profile's
 * AUTO_MV_FAST_VIEW_DEB_MS) the profile's, else at most AUTO_MV_VIEW_DEB_MAX_MS; the step reads it from the next call
 * on, so the glue sets it at ARM. autoMotionInit does not touch it; the get gives the value in force. Flash. */
void autoMotionSetViewDeb(int32_t ms);
uint16_t autoMotionGetViewDeb(void);
/* The feedforward roll per forward speed (NVS mvffr, x 0.1 deg per m/s, AUTO_MV_FFR_MIN_DDEG..MAX, default 0;
 * 2026-10-22 "the left slide", a port of a2b-int 5ee4fcd): a MOVE's RUN adds this x the reference forward speed (m/s)
 * to the hold's roll feedforward (+ right side down: a positive value leans the drone right in forward flight, left in
 * backward flight), against a lateral pull that comes with forward flight (the kinematic fit of the 10-17/18/22 moves:
 * 1.3-3.3, mean ~1.8 deg per m/s to the left, two moves without it). Latched at ARM with the fast caps (autonomy.c
 * sortieSpeedLatch). Clamped to its range; 0: bit-identical to the motions before it (the identity hash stands).
 * autoMotionInit does not touch it; the get gives the value in force. Flash. */
#define AUTO_MV_FFR_MIN_DDEG     (-50)
#define AUTO_MV_FFR_MAX_DDEG     50
void autoMotionSetFfRoll(int32_t dDegPerMps);
int8_t autoMotionGetFfRoll(void);
/* The sortie's speed step (autonomy.c at ARM, through autoFsmSetMvSpeed): every later command plans with its limits,
 * the step reads its settle, dwell, brake and frozen-view values. Flash. */
void autoMotionSetSpeed(autoMotion_t *m, uint16_t pct);
/* The fast steps' acceleration cap (cm/s^2) and settle floor (ms) (auto_motion.h "the fast steps' acceleration and
 * settle as settings": NVS mvacc, mvsettle), each clamped to its range: autoMvSpeedCfg reads them from the next call on,
 * so the glue sets them at ARM before autoFsmSetMvSpeed (which copies the step's settle into the motion memory). A value
 * out of range is clamped here (the settings table refuses it before: auto set ERR, a stored one the default, loudly).
 * autoMotionInit does not touch them; the get gives the pair in force. Flash. */
void autoMotionSetFastCaps(int32_t accCm, int32_t settleMs);
void autoMotionGetFastCaps(uint8_t *accCm, uint16_t *settleMs);
/* The longest peak speed (m/s) of a straight approach of f m with acceleration a toward a scene d m ahead whose image
 * expansion v / (d - s) never exceeds w anywhere along the trapezoid (s the distance flown; the decel's own peak is
 * sqrt(a / (2 (d - f))) at a remaining distance of d - f): 1e30 when no speed reaches w. Flash, pure (host tests). */
float autoMvFwdPeakCap(float d, float f, float a, float w);
/* The command limits of one MOVE (cm; the header's "command limits"), without the scene checks: false is RANGE.
 * Also the TAKEOFF's planned move (auto_fsm.c takeoffCmd, up 0). */
bool autoMvRangeOk(int32_t rightCm, int32_t fwdCm, int32_t upCm);
/* MOVE: AUTO_MVW_NONE when accepted. The caller has checked the state
 * (HOVER waiting: AUTO_MVW_NOT_WAITING otherwise). */
autoMvWhy_t autoMotionMove(autoMotion_t *m, const autoMvIn_t *in, int16_t rightCm, int16_t fwdCm, int16_t upCm);
/* The TAKEOFF's planned move (auto_fsm.c, the wait's start): autoMotionMove with the forward floor at
 * the plan's (mvfloor + AUTO_PLAN_FRONT_EXTRA_CM, the Mac's) instead of mvfloor. */
autoMvWhy_t autoMotionMovePlan(autoMotion_t *m, const autoMvIn_t *in, int16_t rightCm, int16_t fwdCm, int16_t upCm);
/* SET_HEIGHT's correction (auto_fsm.h AUTO_SH_*; the state machine has checked the state, the contact window, the rate
 * and the ranges): the committed height offset moves by d (the reference into the corrected frame), then a
 * vertical-only MOVE of upM (normally -d) from hStart = in->vzH + d, the estimate the glue shifts on this step
 * (vzShiftHeight()); mvFlags AUTO_MVF_SET_HEIGHT. Its own checks: BUSY, NO_VIEW, NO_CAM_HEIGHT (any mode: in open loop
 * the controller ignores vRef), BATT when it climbs (upM > 0), RANGE for a non-finite d or upM. Ends as a vertical
 * MOVE: DONE (the settle commits upM: hOff back to its old value + d + upM), or STOPPED with the measured travel
 * committed. Flash. */
autoMvWhy_t autoMotionSetHeight(autoMotion_t *m, const autoMvIn_t *in, float d, float upM);
/* The same with the reference's own shift (the "go to the target" form, auto_fsm.h AUTO_SH_FORM_TO): the committed
 * offset moves by refShift instead of d (hOff = h_new - hTarget: the reference at the corrected estimate), so DONE's
 * commit (the offset before it + refShift + upM) puts the reference at the target; STOP and the guards commit the
 * measured travel from there, as before. autoMotionSetHeight is this with refShift = d. Flash. */
autoMvWhy_t autoMotionSetHeightRef(autoMotion_t *m, const autoMvIn_t *in, float d, float upM, float refShift);
/* The lowest height a move down may leave the drone at (AUTO_MV_H_MIN_M + min(AUTO_MV_H_DRIFT_CAP_M,
 * AUTO_MV_H_DRIFT_MPS x the air time)): the floor a MOVE down and the target form's SET_HEIGHT descent keep. Flash. */
float autoMotionHeightFloor(const autoMvIn_t *in);
/* YAW: deg > 0 clockwise (right). Without a view (FLIGHT) up to AUTO_MV_BLIND_YAW_MAX_DEG (a blind turn), larger
 * refused NO_VIEW. */
autoMvWhy_t autoMotionYaw(autoMotion_t *m, const autoMvIn_t *in, int16_t deg);
/* STOP (or the sortie leaving HOVER: AUTO_MVE_LEFT_HOVER, a guard:
 * AUTO_MVE_PROGRESS, the state machine's stops: CONTACT, NO_VIEW, TILT,
 * LOOP_GAP): ends a running motion where it is (snap; a MOVE also brakes). A
 * STOP (AUTO_MVE_STOP_CMD) never ends a turn's dwell or its re-track bob, nor a
 * move's re-view bob: the drone holds its place meanwhile, and the camera
 * height estimate needs the tracker's new G (a STOP that ended the bob would
 * lose it); they end by themselves within ~8 s. A STOP during the turn itself
 * ends the turn where it is; so do TILT and LOOP_GAP (2026-10-15: a loop gap
 * or a disturbance must not take the turn's camera height guess away). CONTACT
 * and NO_VIEW during a YAW (FLIGHT, the camera) turn it back to its start
 * heading with the same seq (end UNDONE), once. CONTACT ending a MOVE with a
 * horizontal part also moves the hold's anchor AUTO_MV_CONTACT_BACK_M back
 * along the move (backOff; the reference until the next motion or snap).
 * Nothing running: no change. */
void autoMotionStop(autoMotion_t *m, const autoMvIn_t *in, autoMvEnd_t why);
/* A snap epoch without a motion (2026-10-15): the hold holds where the drone is now (an in-air contact with no motion
 * running: it must not pull back toward what was hit; again at the end of that contact's hush, where the bounce left
 * it). Ends a contact's back-off anchor (backOff). Flash. */
void autoMotionSnap(autoMotion_t *m);
/* Every HOVER step: advances the references, runs the guards, fills out. */
autoMvLand_t autoMotionStep(autoMotion_t *m, const autoMvIn_t *in, autoMvOut_t *out);
/* Outside HOVER: the outputs that keep the hold and the yaw where the last
 * step left them (offsets, epoch, and a finished MOVE's end as the hold's
 * reference, as the step gives it: without it a LAND after a MOVE would fly
 * back toward the move's start; a contact's back-off anchor likewise), no
 * rates, no feedforward, no motion. */
void autoMotionIdleOut(const autoMotion_t *m, autoMvOut_t *out);
/* Active: RUN, PAUSED or SETTLE. */
bool autoMotionActive(const autoMotion_t *m);

/* ---- helpers (exposed for the tests) --------------------------------------- */
/* Natural logarithm without libm (|relative error| < 2e-7 for x in [1e-30, 1e30]); x <= 0 gives -1e30. */
float autoMvLn(float x);
/* Trapezoid of distance dist >= 0, speed limit v, acceleration a (> 0). */
void autoMvProfPlan(autoMvProf_t *p, float dist, float v, float a);
/* Position, speed and acceleration at t (s); t clamped to 0..T. */
void autoMvProfEval(const autoMvProf_t *p, float t, float *s, float *v, float *acc);

const char *autoMvStateName(autoMvState_t s);
const char *autoMvWhyName(autoMvWhy_t w);
const char *autoMvEndName(autoMvEnd_t e);

#endif /* AUTO_MOTION_H_ */
