/*
 * ESP-Drone Firmware
 *
 * camera_hold.h - Camera drift hold: small attitude corrections that cancel
 * horizontal drift measured by the forward camera (pure control law)
 *
 * Inputs are the camera's relative velocities (velocity divided by the
 * unknown distance to the scene ahead):
 *   wFwd  [1/s]   = v_forward / D  (image expansion rate; + = toward the scene)
 *   wLat  [rad/s] = v_right / D    (rotation-compensated sideways image motion)
 * Outputs are attitude-setpoint corrections in degrees in the controller
 * convention (+pitch = nose up, +roll = right side down):
 *   pitch = +(KP * wFwd_f + KI * integral(wFwd))   nose up to brake forward drift
 *   roll  = -(KP * wLat_f + KI * integral(wLat))   left side down to brake right drift
 * The integral term is a weak pull back toward the take-off position, and it
 * learns the steady lean the drone needs (gyro / accelerometer drift).
 *
 * Engages only when enabled, the sticks are centred, the drone has been
 * airborne for CAM_HOLD_SETTLE_S and the camera view is usable. When the view
 * is lost (too little texture, fast yaw, frame gap) with the sticks still
 * centred, it coasts (camera_flow.c; its yaw test is camHoldYawOk: 20 deg/s,
 * 45 deg/s in a commanded turn): the measured terms drop out but the learned integral
 * keeps being applied. Outputs are limited to +/-CAM_HOLD_LIMIT_DEG and
 * slew-rate limited; when neither engaged nor coasting they decay to zero at
 * the same rate. No ESP-IDF dependencies; unit tested by
 * tools/tests/test_camera_hold.c.
 *
 * Moving reference (interactive hover, auto_motion.h; user decision
 * 2026-10-12, late). The integrals are position errors in the hold's own
 * units (lateral x / D, forward ln(D0 / D)); a reference r moves them:
 * each engaged frame takes the reference's change since the last one off
 * the integral (clamped as the measurement is), the P term compares the
 * filtered rates (fwdF - rateFwdF), and a feedforward lean is added before
 * the limit:
 *   pitch = +(KP (fwdF - rateFwdF) + KI iFwd) + ffPitch
 *   roll  = -(KP (latF - rateLatF) + KI iLat) + ffRoll
 * References come in epochs: a new refEpoch starts positions (reference,
 * measured progress pLat/pFwd, what the clamps took) from 0; one that began
 * with a snap (refSnapEpoch newer than the epoch it ends) first sets each
 * integral back to its value at the start of the ending epoch (integral +
 * clamp losses - (progress - reference)): the drone then holds where it is
 * with the lean it had learned before the move. With every reference field 0
 * (no motion ever, or none running) the outputs are bit-identical to the hold
 * without references: the reference code runs only on nonzero values, and
 * the bookkeeping it adds never feeds the outputs.
 *
 * HOVER scene gate (camHoldSceneOk, 2026-10-13). The hold's own view test
 * (camera_flow.c) passes frames with a fit residual up to 15 grey levels and
 * has no test for a moving object. In flight 1 of 2026-10-13 a person walking
 * across and then away from the camera 1-1.5 m ahead was integrated as the
 * drone's own drift: the hold leaned the drone toward him (pitch at its
 * -2.5 deg limit, roll -1 deg) until it hit something 34.45 s into the hover.
 * The camera height observer refused the same frames with its own gates
 * (cam_vz_obs.h: residual 8, |dxRes| 1.5 px, smoothed |scale / dt| 0.3 1/s).
 * The scene gate applies those three to the hold, but only in autonomy HOVER
 * after CAM_HOLD_SCENE_ARM_S of it, so manual flights, TAKEOFF, LAND and the
 * first 2 s of every HOVER fly exactly as before; on the logged clean sorties
 * it never fires there. A gated frame is a lost view: the hold coasts on its
 * learned integral (a MOVE pauses and a new one is refused NO_VIEW), and a
 * view gated for 2 s is also unaccepted by the observer, so the existing
 * HOVER_NO_CAM / MOVE_NO_CAM rules bound the coasting. The expansion EMA runs
 * on every frame, armed or not, exactly as the observer's.
 * Not refused (KNOWN_GAP, docs/autonomy-a2.md 12.5): a smooth-textured
 * object that moves as a whole, e.g. a person in plain clothes filling 60-90 %
 * of the view and slowly receding (-0.07..-0.15 /s through the fit, under
 * the 0.3 /s gate): the hold reads it as backward drift. A tighter HOVER-only
 * expansion gate would need its own coasting bound (12.7, the user's call).
 *
 * Gain scale with the scene distance (camHoldInput_t.gainScale, 2026-10-14,
 * docs/autonomy-a2b.md 20). The hold's inputs are velocities over D, so its
 * fixed gains are weaker the farther the scene: at today's D 2.55-3.46 m KI / D
 * was 0.57-0.78 deg/m, 0.15 m of steady offset per 0.1 deg of lean error with
 * a ~3 s time constant. With s > 1 both gains are multiplied by s (kp = KP s,
 * ki = KI s) and every integral clamp becomes CAM_HOLD_I_LIMIT_DEG / ki, so
 * the integral's contribution stays <= 1.5 deg; the output limit (2.5 deg)
 * and the slews (6 deg/s) are unchanged. A change of s first rescales both
 * integrals by s_old / s_new (the learned lean KI s i stays continuous) and
 * the reference bookkeeping with them (a snap restores the lean it had when
 * its epoch began). s = 1 (also 0, NaN, inf or below 1) is bit-identical to
 * the hold before it. camHoldGainTarget() picks s from the camera height
 * observer's tracked G (k = 80 / G = D / 1.25 m, <= kmax) in an autonomy
 * sortie's TAKEOFF and HOVER until its first translation MOVE (1 in LAND);
 * camera_flow.c slews s toward it at CAM_HOLD_GAIN_SLEW_PER_S, at
 * CAM_HOLD_GAIN_SLEW_MOVE_PER_S once a MOVE ran in HOVER (camHoldGainRate).
 */
#ifndef CAMERA_HOLD_H_
#define CAMERA_HOLD_H_

#include <stdbool.h>
#include <stdint.h>

#include "auto_motion.h"   /* AUTO_MV_TURN_GATE_RPS, autoMvState_t, autoMvKind_t */

#define CAM_HOLD_KP            6.0f    /* deg per (1/s) */
#define CAM_HOLD_KI            2.0f    /* deg per unit of integrated relative motion */
#define CAM_HOLD_I_LIMIT_DEG   1.5f    /* integral contribution limit */
#define CAM_HOLD_LIMIT_DEG     2.5f    /* total correction limit per axis */
#define CAM_HOLD_SLEW_DEG_S    6.0f    /* output slew-rate limit */
#define CAM_HOLD_FILTER_TAU_S  0.25f   /* low-pass on the camera velocities */
#define CAM_HOLD_SETTLE_S      1.0f    /* airborne time before engaging */
#define CAM_HOLD_RESET_S       1.0f    /* disengaged time before the integrals reset */
#define CAM_HOLD_MAX_DT_S      0.3f    /* a longer frame gap is treated as a dropout */
#define CAM_HOLD_TAKEOFF_LAT_SETTLE_S 0.35f /* only after detected lift-off, forward axis still off */
#define CAM_HOLD_STARTUP_LAT_MAX 2.0f
#define CAM_HOLD_STARTUP_HOVER_S 3.0f

typedef struct {
  bool enabled;          /* user switch */
  bool sticksCentred;    /* pilot roll/pitch commands exactly zero */
  bool airborne;         /* thrust above the lift-off level */
  bool viewOk;           /* fresh frame, fit ok, enough texture, slow rotation */
  float wFwd;            /* 1/s */
  float wLat;            /* rad/s */
  float dt;              /* s since the previous camera frame */
  /* Moving reference (all 0 without a motion; auto_motion.h autoMvOut_t). */
  uint32_t refEpoch;     /* a change starts a new epoch (positions from 0) */
  uint32_t refSnapEpoch; /* the latest epoch that began with a snap */
  float refLat, refFwd;  /* reference positions since the epoch start (x / D, ln(D0 / D)) */
  float rateLat, rateFwd;/* reference rates, 1/s (the P term's velocity reference) */
  float ffRoll, ffPitch; /* feedforward lean, deg, controller convention */
  /* No forward hold (autonomy TAKEOFF, docs/autonomy-a2b.md 19.5): the climb's floor geometry and auto exposure
   * read as expansion (+0.06..+0.79 ln over 15 of 17 take-offs), which the hold integrated into a +0.5..+2.5 deg
   * nose-up lean at the HOVER entry, a backward kick. While set the forward axis neither filters nor integrates
   * (fwdF 0, iFwd and the forward reference bookkeeping unchanged) and its output is KI iFwd + ffPitch; the
   * lateral axis, engaged and coasting are unchanged. false: bit-identical to the hold before it. */
  bool fwdOff;
  /* Gain scale s (docs/autonomy-a2b.md 20): kp = KP s, ki = KI s, integral clamps I_LIMIT / ki. 0, NaN, inf or <= 1
   * mean 1 (bit-identical to the hold before it); clamped to CAM_HOLD_GAIN_SCALE_MAX. */
  float gainScale;
  /* Fast moves (mvspeed > 100, 2026-10-16, docs/autonomy-a2b.md 31; camHoldMoveLimits): moveLimDeg > 0 while a
   * motion's reference or feedforward is active (camHoldFastActive) clamps the camera feedback (P + I, the learned trim
   * included) at CAM_HOLD_LIMIT_DEG as before and adds the feedforward on top, the sum clamped at moveLimDeg; coasting
   * also applies the feedforward (a stopped move's brake); the output slews at moveSlewDegS. 0 (both): bit-identical to
   * the hold before it. */
  float moveLimDeg, moveSlewDegS;
  /* The move's braking lean scaled by the measured speed (mvfix BRAKE, auto_motion.h "move fixes"; the 300 % forensics
   * of 2026-10-17, docs/autonomy-a2b.md 33.6): on an engaged frame, each axis's feedforward that opposes its reference
   * rate (rateFwd, rateLat: a brake) is multiplied by the measured over the reference rate through the same filter
   * (fwdF / rateFwdF, latF / rateLatF; 1 below CAM_HOLD_FF_BRAKE_WMIN), clamped to 0..CAM_HOLD_FF_BRAKE_MAX: the planned
   * brake of 1.5-1.8 deg hit a drone at 0.09 m/s, not the reference's 0.24, and slid it back. Only a brake is scaled and
   * only down (a drone faster than its reference is the feedback's: 1.5 x over-braked the sim's moves near their end,
   * where the reference rate is small), and only once the camera has seen that axis move (its measured progress at
   * least CAM_HOLD_FF_BRAKE_TRACK of the reference taken: a frozen view keeps the full brake). The speed-up and the
   * drag lean are not scaled (a drone behind its reference needs more of them, not less). false: bit-identical to the
   * hold before it. */
  bool ffBrakeScale;
  /* Autonomous startup only: separate lateral gain, 0/1 retains the original law.
   * The integral is rescaled on changes to preserve its learned lean. The same
   * 1.5-degree integral / 2.5-degree output caps and output slew still apply. */
  float latGainScale;
  bool earlyTakeoffLat; /* shorter settle only with fwdOff (detected climb) */
  /* The lateral position gain during a MOVE (NVS mvlatk, 2026-10-22 "the left slide"; a port of a2b-int cc4d7b5):
   * the lateral axis's integral gain is latKi x this (its clamp CAM_HOLD_I_LIMIT_DEG / that), the P term's kp
   * unchanged. camera_flow.c sets it from the sortie's latched mvlatk while the recorder's mvState shows a MOVE
   * running, paused or settling (camHoldMoveActive), 1 otherwise. The integral is a position error (x / D); with ki
   * alone against a steady lateral force the drone settles where ki x error cancels it: on 2026-10-17/18/22 the
   * forward moves met a ~0.3-0.7 deg leftward pull in forward flight (not in the hover) and slid 0.3-0.8 m left
   * (0.45 deg over KI s 2-3 deg per unit = 0.15-0.25 units of x / D). A change rescales iLat (latKi x iLat continuous,
   * as gainScale does; the reference bookkeeping with it). 0, NaN, inf or <= 1 mean 1: bit-identical to the hold
   * before it; clamped to CAM_HOLD_LAT_GAIN_MAX. */
  float latGain;
} camHoldInput_t;

typedef struct {
  float fwdF, latF;      /* filtered relative velocities */
  float iFwd, iLat;      /* integrated relative motion */
  float airborneTime;
  float idleTime;        /* time not engaged */
  float roll, pitch;     /* output corrections, deg */
  bool engaged;
  bool coasting;         /* view lost, applying the learned integral only */
  /* moving reference (camHoldInput_t ref*) */
  uint32_t epoch;        /* the reference epoch applied */
  float pFwd, pLat;      /* measured relative motion since the epoch start (engaged frames, unclamped) */
  float rFwd, rLat;      /* reference taken off the integrals since the epoch start */
  float lostFwd, lostLat;/* what the integral clamps removed since the epoch start, and the gain scale's share:
                          * i + lost - (p - r) is always the epoch start's integral in the current scale (the snap) */
  float rateFwdF, rateLatF; /* the rate references through the measurements' filter */
  float gainScale;       /* the gain scale applied last (0 after a reset: 1) */
  float latGainScale;    /* the lateral scale applied last: the startup scale x the MOVE position gain (latGain; 0 after
                            a reset: 1; without a MOVE gain exactly the startup scale, as before) */
  float stillS;          /* camHoldMoving: engaged time with both filtered rates under CAM_HOLD_STILL_RATE */
} camHoldState_t;

#define CAM_HOLD_LAT_GAIN_MAX   4.0f    /* camHoldInput_t latGain is clamped to this (mvlatk 400 %) */
/* A MOVE (not a YAW) running, paused or settling from the recorder's mvState: the frames camera_flow.c applies the
 * sortie's mvlatk to. */
bool camHoldMoveActive(uint8_t mvState);

void camHoldReset(camHoldState_t *st);
/* Increase lateral feedback only before any motion, in TAKEOFF/HOVER, with a
 * healthy tracked scene at least ~2 m away (G <= 50 legacy px/m), ending after
 * three seconds of HOVER. Otherwise 1. */
float camHoldStartupLatTarget(uint8_t autoState, uint32_t refEpoch, uint8_t cvFlags, uint16_t cvGainDpm, float hoverS);

#define CAM_HOLD_FF_BRAKE_MAX   1.0f    /* camHoldInput_t ffBrakeScale: the largest scale of a braking lean (down only) */
#define CAM_HOLD_FF_BRAKE_WMIN  0.01f   /* 1/s: a filtered reference rate under this: the brake unscaled */
#define CAM_HOLD_FF_BRAKE_TRACK 0.5f    /* the axis's measured progress at least this x the reference: scaled */
/* The drone still for a new MOVE's start (mvfix STILL, auto_motion.h "move fixes"; the 300 % forensics: move 2 was
 * accepted with the drone still drifting at 0.15-0.17 m/s): after camHoldStep, once per frame: true while engaged and
 * either filtered measured rate (fwdF, latF) was at or above CAM_HOLD_STILL_RATE within the last CAM_HOLD_STILL_S of
 * engaged frames; false when not engaged (no measurement: a MOVE pauses without a view anyway). camera_flow.c publishes
 * it for the state machine (autoMvIn_t holdMoving). */
#define CAM_HOLD_STILL_RATE     0.05f   /* 1/s (0.10 m/s at D 2 m; the forensics' 0.03 is under the plain hover's median
                                           0.04-0.07 /s of the 300 % flights: auto_motion.h "move fixes" STILL) */
#define CAM_HOLD_STILL_S        0.3f
bool camHoldMoving(camHoldState_t *st, float dt);

/* Fast moves (camHoldInput_t moveLimDeg, moveSlewDegS) from the sortie's mvspeed pct (camera_flow.c): s = pct / 100,
 * limit min(CAM_HOLD_LIMIT_DEG + CAM_HOLD_MOVE_LIM_PER_S x (s - 1), CAM_HOLD_MOVE_LIM_MAX_DEG), slew
 * min(CAM_HOLD_SLEW_DEG_S x s, CAM_HOLD_MOVE_SLEW_MAX_DEG_S); both 0 at pct <= 100 (the hold as before). The
 * feedforward needs 2.6 deg at 0.30 m/s^2 + drag at 0.5 m/s and ~1.5-2.3 deg to brake at 0.40 m/s^2; with 1-2 deg of
 * logged wobble the commanded 4.5 deg stays under the 8 deg tilt guard. */
#define CAM_HOLD_MOVE_LIM_PER_S       1.0f    /* deg per unit of s above 1 */
#define CAM_HOLD_MOVE_LIM_MAX_DEG     4.5f
#define CAM_HOLD_MOVE_SLEW_MAX_DEG_S  12.0f
void camHoldMoveLimits(uint16_t pct, float *limDeg, float *slewDegS);
/* The fast path applies this frame: moveLimDeg > 0 and a motion's reference active (the filtered rate references or
 * the feedforward nonzero; after camHoldStep, as it decided). */
bool camHoldFastActive(const camHoldState_t *st, const camHoldInput_t *in);
/* The output slew camHoldStep used (deg/s): moveSlewDegS on the fast path, else CAM_HOLD_SLEW_DEG_S; camera_flow.c
 * publishes it for controller_pid's apply slew (cameraFlowHoldSlew). */
float camHoldSlewOf(const camHoldState_t *st, const camHoldInput_t *in);

/* HOVER scene gate: the camera height observer's frame gates
 * (camVzObsDefaultConfig), armed after this long in autonomy HOVER. */
#define CAM_HOLD_SCENE_MAX_RESIDUAL 8.0f   /* grey levels (= maxResidual): the legacy 160x120 profile's limit */
/* The residual limit as a setting (NVS holdres, grey levels x 10; auto_glue_core.c AUTO_SET_HOLDRES; 2026-10-22, the
 * table flight: at the native 11 fps profile a 300 % approach's fit residual reads 8-13 grey levels where the legacy
 * 60 ms frames stayed under 7, and the gate's refusals, three and six frames in a row, stopped both MOVEs VIEW_FAST).
 * The native default is 16.0: above today's largest approach residual (13.4; p95 of the move frames 8.4, p99 13.4;
 * hover frames at most 3.4) and above the hold's own per-frame limit (camera_flow.c CAM_HOLD_MAX_RESIDUAL 15.0),
 * so the scene gate adds no residual refusal of its own at native; the lateral and expansion tests stay. 0: no
 * residual test at all. The legacy profile keeps 8.0. Applied with camHoldSceneSetMaxResidual (camera_flow.c, latched
 * at ARM); camHoldSceneReset restores the profile's default. */
#ifdef ESPFLY_NATIVE_VGA
#define CAM_HOLD_SCENE_RES_DEFAULT  16.0f
#define CAM_HOLD_SCENE_RES_X10_DEFAULT 160
#else
#define CAM_HOLD_SCENE_RES_DEFAULT  CAM_HOLD_SCENE_MAX_RESIDUAL
#define CAM_HOLD_SCENE_RES_X10_DEFAULT 80
#endif
#define CAM_HOLD_SCENE_RES_X10_MIN  10     /* holdres: 0 (off) or 10..255 (1.0..25.5 grey levels) */
#define CAM_HOLD_SCENE_RES_X10_MAX  255
#define CAM_HOLD_SCENE_MAX_DX_PX    1.5f   /* |dxRes| px/frame (= maxDxPx) */
#define CAM_HOLD_SCENE_MAX_EXP_RATE 0.3f   /* smoothed |scale / dt|, 1/s (= maxExpRate) */
#define CAM_HOLD_SCENE_EXP_ALPHA    0.2f   /* per-frame EMA weight of scale / dt (= expAlpha) */
#define CAM_HOLD_SCENE_ARM_S        2.0f   /* autonomy HOVER this long before the gate applies: frame time (the
                                              * camera clock = the FSM's) from the first frame seen in HOVER, so a
                                              * frame exposed before the HOVER entry never counts */

typedef struct {
  float expEma;          /* EMA of scale / dt, 1/s, every frame */
  float hoverS;          /* time in autonomy HOVER: frame intervals summed from its first frame (0), 0 outside it */
  bool inHover;          /* the previous frame was in HOVER */
  bool lastRejected;     /* the latest frame was refused by the gate */
  uint32_t rejRes, rejDx, rejExp;   /* refusals by reason (residual first, then lateral, then expansion) */
  float maxResidual;     /* the residual limit in force (CAM_HOLD_SCENE_RES_DEFAULT after a reset; 0: no residual
                            test; camHoldSceneSetMaxResidual) */
} camHoldScene_t;

void camHoldSceneReset(camHoldScene_t *g);
/* The residual limit (grey levels; 0 or negative, NaN: no residual test; NVS holdres / 10). */
void camHoldSceneSetMaxResidual(camHoldScene_t *g, float maxResidual);
/* One camera frame: hover = autonomy HOVER now; dt s since the previous
 * frame; residual = the fit's (grey levels); dxResPx = dx minus the rotation
 * model (camera_flow.c FLOW_KX_*); scale = the relative scale change. True
 * when the frame may be used (always true while unarmed); a NaN residual or
 * dxRes is refused when armed. */
bool camHoldSceneOk(camHoldScene_t *g, bool hover, float dt, float residual, float dxResPx, float scale);

/* One update per camera frame. Returns true while the output should be
 * applied (engaged or coasting). */
bool camHoldStep(camHoldState_t *st, const camHoldInput_t *in);

/* The yaw part of the hold's view test (camera_flow.c): the body turned
 * less than CAM_HOLD_MAX_YAW_RATE x dt over the frame (yawAngleRad, the
 * frame's gyro angle about z); while a commanded turn of the interactive
 * hover runs (turn), AUTO_MV_TURN_GATE_RPS x dt (user decision 2026-10-13,
 * evening: turns at 30 deg/s, which every frame of the turn would fail at
 * 20 deg/s: the hold would coast through it and the turn pause). False for
 * a NaN angle or dt <= 0. */
#define CAM_HOLD_MAX_YAW_RATE    0.35f    /* rad/s (20 deg/s) */
bool camHoldYawOk(float yawAngleRad, float dt, bool turn);
/* A commanded turn from the recorder's mvState (autoMvState_t | autoMvKind_t
 * << 4, flight_recorder.h): a YAW running, paused or settling (its dwell and
 * re-track bob), as the motion's camGuessOk. */
bool camHoldTurnActive(uint8_t mvState);
/* Any interactive-hover motion from the recorder's mvState: a MOVE (a SET_HEIGHT's correction too) or a YAW running,
 * paused or settling (camHoldLimitStep's "outside MOVE/YAW"). */
bool camHoldMotionActive(uint8_t mvState);

/* The gain scale's target (docs/autonomy-a2b.md 20), from the recorder fields camera_flow.c already reads:
 *   - autoState not TAKEOFF / HOVER / LAND (IDLE, ARMED: a new sortie; KILLED) clears the latch, target 1;
 *   - mvState of kind AUTO_MVK_MOVE (in any of its states, a vertical MOVE too) sets 'translated' for the rest
 *     of the sortie: an approach shrinks D while G is stale, so every MOVE flies with s 1 as before;
 *   - a commanded turn (camHoldTurnActive: a YAW running, paused or settling) forgets the latched G (a new view);
 *   - otherwise cvFlags bits 0 and 2 (the observer airborne, its G tracked) with cvGain in
 *     CAM_HOLD_GAIN_DPM_MIN..MAX latch G = cvGain / 10 px/m. Airborne: on the ground the observer keeps the
 *     previous flight's last tracked G (2026-10-14 f4, f5: 18.5 and 22.2 px/m, the landing's view, flagged
 *     tracked through the next take-off's spin-up), which is not this flight's scene;
 *   - target clamp(CAM_HOLD_GAIN_G_REF / G, 1, kmax) in TAKEOFF / HOVER when kmax > 1, not translated and a G
 *     is latched; else 1. LAND keeps the latch and the MOVE / YAW tracking but its target is 1: the descent's
 *     floor parallax reads as forward motion, and a larger gain turns it into a larger creep (a log replay
 *     through touchdown, 7 reference LANDs without a MOVE: +0.10..+0.26 m forward at s 1.4-2.5).
 * kmax: NaN or <= 1 is off (1), above CAM_HOLD_GAIN_SCALE_MAX it is clamped. */
#define CAM_HOLD_GAIN_SCALE_MAX     3.0f    /* camHoldInput_t.gainScale and kmax are clamped to this */
#define CAM_HOLD_GAIN_G_REF         80.0f   /* px/m: the observer's gDefault, D = AUTO_MV_FY_PX / G = 1.25 m */
#define CAM_HOLD_GAIN_DPM_MIN       150u    /* cvGain (0.1 px/m) latched only within 15..500 px/m */
#define CAM_HOLD_GAIN_DPM_MAX       5000u
#define CAM_HOLD_GAIN_CVF_LATCH     0x05u   /* cvFlags bits 0 (airborne) and 2 (gain tracked), flight_recorder.h */
#define CAM_HOLD_GAIN_KMAX_DEFAULT  2.5f    /* NVS holdkmax (1.0 = off .. 3.0) */
#define CAM_HOLD_GAIN_SLEW_PER_S    2.0f    /* camera_flow.c moves s toward the target at most this fast ... */
/* ... and at this rate once a MOVE ran in an autonomy HOVER (camHoldGainRate; 2026-10-14, docs/autonomy-a2b.md 21.3):
 * every gym move (D 4.2-4.7 m) of 2026-10-14 stopped in RUN while s fell 2.5 -> 1 within 0.75 s, and the a2b 20.8
 * sim at D 3.5 m ended 31 of 480 moves PROGRESS / SHORT at 2 /s, 12 at 0.75 /s. An approach at 0.20 m/s lowers the
 * scene's need (D / 1.25 m) by at most 0.16 /s, under 0.75 /s: s / need never rises during a move. LAND (to s 1, the
 * 20.8 #2 creep fix) and the hover before the first MOVE keep 2 /s. */
#define CAM_HOLD_GAIN_SLEW_MOVE_PER_S 0.75f

typedef struct {
  float g;               /* the latched tracked G, px/m (0: none) */
  bool translated;       /* a MOVE ran in this sortie */
  bool moveActive;       /* the previous frame's mvState was a MOVE running, paused or settling */
  bool untracked;        /* camHoldGainMove found a MOVE toward the scene the camera does not see: 2 /s (the rate) */
  bool confirmed;        /* the camera has seen the latest MOVE toward the scene go (the gain follows it from then) */
  float sMove;           /* the gain scale when the latest MOVE started (camHoldGainMove's ceiling) */
  /* mvfix GAIN (camHoldGainMoveFix; all 0 after camHoldGainReset: camHoldGainMove as before) */
  bool fromMeasured;     /* the target from the measured progress (progFwd), not the reference's */
  float graceS;          /* no tracking test (untracked) for this long from a MOVE's start */
  float moveS;           /* time since the latest MOVE started (frames' dt) */
} camHoldGainLatch_t;

void camHoldGainReset(camHoldGainLatch_t *l);
float camHoldGainTarget(camHoldGainLatch_t *l, uint8_t autoState, uint8_t mvState, uint8_t cvFlags,
                        uint16_t cvGainDpm, float kmax);
/* During a MOVE toward the scene (2026-10-14, docs/autonomy-a2b.md 21.3; the spec's escalation, built because the
 * long-move sims ended 4.9 % of the accepted moves at D <= 4.7 m PROGRESS / SHORT with s falling to 1, gate 3 %): in
 * HOVER, while mvState is a MOVE running, paused or settling, a G was latched and the move has a part toward the
 * scene (dLowM > 0, autoMvOut_t), the target is
 *   clamp(D_ref_now / 1.25 m, 1, s at the move's start),  D_ref_now = dLowM x exp(-refFwd / AUTO_MV_K_LOW)
 * (the reference's scene distance now, its lower bound: k at its low; the hold's own s(D) rule of 20 with the D the
 * move is taking the drone to), so a far approach keeps the gain its scene needs instead of falling to 1 in its
 * first 2 s, and the gain never exceeds the scene's need: approaching lowers D_ref_now, never raises s. Only once
 * the camera has seen the move go (confirmed: the reference past CAM_HOLD_GAIN_TRACK_MIN and the measured forward
 * progress progFwd, the hold's pFwd, at least CAM_HOLD_GAIN_TRACK_FRAC of it), and not while it does not (untracked:
 * the target unchanged and camHoldGainRate 2 /s): in a frozen view the hold's push on the reference grows with s
 * (the sim's frozen 1.5 m approach went 2.83 m with s kept, 1.91 m with s falling at 0.75 /s, 1.87 m with this
 * rule; b6a54da's 2 /s 1.79 m). Sideways and back moves (dLowM 0), kmax 1, no G latched: target unchanged. sNow:
 * the scale applied now (latched as the ceiling on the frame a MOVE starts). Call after camHoldGainTarget; slew at
 * camHoldGainRate (0.75 /s). */
#define CAM_HOLD_GAIN_TRACK_MIN     0.01f   /* ln: the reference this far along before the tracking test applies */
#define CAM_HOLD_GAIN_TRACK_FRAC    0.3f    /* measured >= this x the reference: the camera sees the move */
float camHoldGainMove(camHoldGainLatch_t *l, float target, float sNow, uint8_t autoState, uint8_t mvState,
                      float dLowM, float refFwd, float progFwd);
/* mvfix GAIN (auto_motion.h "move fixes"; the 300 % forensics of 2026-10-17: the gain fell 1.84 -> 1.0 from the
 * reference's progress while the drone lagged it by 0.21 ln, and an 'untracked' dip in the first 0.7 s of the fast
 * speed-up took it to 1.22): once per frame before camHoldGainMove. measured: its target from the measured progress
 * D_now = dLowM x exp(-max(progFwd, 0) / AUTO_MV_K_LOW) (clamped as before; the target unchanged only while untracked),
 * and before the tracking test applies (the reference under CAM_HOLD_GAIN_TRACK_MIN, or in the grace) the measured one
 * too, not 1; graceS: the tracking test waits this long from a MOVE's start (camera_flow.c: CAM_HOLD_GAIN_GRACE_S above
 * 100 %, 0 at 100); dt the frame's (the move's clock). measured false and graceS 0: camHoldGainMove as before. */
#define CAM_HOLD_GAIN_GRACE_S       1.0f    /* the 300 % speed-up read under 0.3 x its reference for ~0.7 s */
void camHoldGainMoveFix(camHoldGainLatch_t *l, bool measured, float graceS, float dt);
/* The slew rate camera_flow.c moves s at (1/s): CAM_HOLD_GAIN_SLEW_MOVE_PER_S once the latch is translated (a MOVE
 * ran this sortie) and the sortie is in HOVER, else CAM_HOLD_GAIN_SLEW_PER_S (call it after camHoldGainTarget, which
 * sets translated). */
float camHoldGainRate(const camHoldGainLatch_t *l, uint8_t autoState);
/* s moved toward target by at most ratePerS x dt (dt clamped to CAM_HOLD_MAX_DT_S; NaN or <= 0: no move; a rate
 * NaN or <= 0: CAM_HOLD_GAIN_SLEW_PER_S); a NaN s restarts from 1. */
float camHoldGainSlew(float s, float target, float dt, float ratePerS);

/* ---- The hold at its limit (2026-10-16, docs/autonomy-a2b.md 27.4) ---------------------------------------------
 * The hold's integral can lean at most CAM_HOLD_I_LIMIT_DEG (1.5 deg) for a steady offset; on 2026-10-16 (F1, F2) the
 * drone needed ~1.4 deg more right lean than the day before, the roll correction sat at 1.36-1.50 deg and the drone
 * crept 0.4-1 m left. A correction at or above CAM_HOLD_LIMIT_WARN_DEG on one axis for CAM_HOLD_LIMIT_WARN_MS, in
 * HOVER outside any motion (a MOVE's lean or a turn's view is not a trim), on frames the hold's output is applied
 * (engaged, or coasting on the integral it learned: the lean stays applied), sets that axis's bit (and its sign) until
 * the run breaks (the correction under CAM_HOLD_LIMIT_KEEP_DEG, or a frame that does not count); the state machine reports the rise as the event HOLD_LIMIT (auto_fsm.h autoFsmHoldLimit):
 * information only, nothing flies differently. The logs (tools/tests/test_hold_limit_replay.c): no 10-15 hover
 * reaches it (the longest run 1.7 s: fastcheck's pitch); 10-16 F1 rolls 8.1 s and pitches 11.8 s at it (fires at 48.2
 * and 56.2 s of its frames' clock); F2 rolls 7.6 s (does not fire); of the 10-14 flights only the gym's flight 1 (boot
 * 12) fires, rolling 1.5-2.1 deg for 12.8 s: the hover that needed +1.58 deg, the same condition as F1. Camera task,
 * per frame: flash; the state 12 bytes (two u32 starts, the run and the bits). Pure. */
#define CAM_HOLD_LIMIT_WARN_DEG     1.3f      /* |correction|, deg (0.87 x CAM_HOLD_I_LIMIT_DEG) */
#define CAM_HOLD_LIMIT_WARN_MS      8000u     /* held this long */
#define CAM_HOLD_LIMIT_KEEP_DEG     1.2f      /* an open run stays open down to this (hysteresis, 2026-10-16 review: F1's
                                                 roll run 8.1 -> 8.6 s, pitch 11.8 -> 13.8 s; the same flights fire, at
                                                 the same times; 1.15 would make F2 fire) */
#define CAM_HOLD_LIM_ROLL           0x01u     /* = auto_fsm.h AUTO_HL_* */
#define CAM_HOLD_LIM_PITCH          0x02u
#define CAM_HOLD_LIM_ROLL_NEG       0x04u
#define CAM_HOLD_LIM_PITCH_NEG      0x08u

typedef struct {
  uint32_t sinceMs[2];   /* roll, pitch: the run's start (with its run bit) */
  uint8_t run;           /* CAM_HOLD_LIM_ROLL / PITCH: a run at the limit is open */
  uint8_t bits;          /* CAM_HOLD_LIM_* */
} camHoldLimit_t;

/* One frame: roll and pitch the hold's correction (deg, its outputs), counting whether this frame counts (HOVER, no
 * motion, the hold applied on a fresh view); nowMs any ms clock. Returns the bits; not counting clears every run. */
uint8_t camHoldLimitStep(camHoldLimit_t *s, float roll, float pitch, bool counting, uint32_t nowMs);

#endif /* CAMERA_HOLD_H_ */
