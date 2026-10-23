/*
 * Closed-loop simulation of the interactive hover (docs/autonomy-a2b.md
 * section 14): the real state machine (auto_fsm.c) with its motions
 * (auto_motion.c), the real vertical controller (vert_ctrl.c) on its sim
 * plant with the camera observer model, and the real camera drift hold
 * (camera_hold.c) closing a horizontal plant through a camera model, in a
 * room. The glue is emulated as autonomy.c does it (decimated inputs, the
 * hold's reference published every step, the yaw setpoint, vRef/hOff).
 *
 * Horizontal plant: a point mass leaning on its attitude (roll right side
 * down accelerates right, pitch nose up backward), the attitude following
 * the hold's correction through controller_pid's 6 deg/s apply slew and a
 * 0.08 s response, a body-fixed trim error the hold has to learn, drag 0.35/s;
 * the heading follows the yaw setpoint (0.15 s). Camera: a frame every
 * ~64 ms delivered 120 ms late, wLat = v_right / D, wFwd = k v_fwd / D +
 * c_vf v_up / D (the vertical motion the forward channel reads), noise as the
 * logged hovers (0.03 / s), D the distance to the wall ahead in a 6 x 6 m
 * room. The observer's D: the true one times (1 + eD) per view, its scale
 * tracker restarting after 20 deg of turn and re-tracking after 2 s of bob.
 *
 * Mission (the user's 2026-10-12 request): TAKEOFF 100 cm, a photo, 10 s of
 * hover (Claude reads the photo), MOVE right 50, MOVE back 50, YAW 90,
 * MOVE up 20, MOVE forward 40, STOP, LAND. Every command within +/-20 % of
 * itself (true displacement in the body frame of the command, from its
 * acceptance to 2 s after its end), no guard trip, a soft touchdown. Guard
 * cases: the camera lost in the middle of a move (PAUSED, then LAND
 * MOVE_NO_CAM), the battery low (LAND BATT_LOW), the air limit (LAND
 * AIR_TIME at 80 s with the wait restarted by STOP), STOP in the middle of a
 * move (the drone stops close to where it was).
 *
 * Added after the verification of 591f176 (its harness verify-ih-safety):
 * the hold stays engaged through LAND until contact, as camera_flow.c does
 * (it was off for the whole LAND here, which hid a LAND after a finished
 * MOVE flying the drone back toward the move's start); a LAND after single
 * moves; the expansion scale k < 1 (the forward floor); a depth step (a box
 * face in front of the wall: the lateral cap and the re-view); a frozen view;
 * a 0.20 m take-off moved up to 1.20 m, then LAND; moves down late in a long
 * hover with a drifting height estimate; a STOP during the re-track bob.
 *
 * Added for LAND COLLISION (user decision 2026-10-13, docs/autonomy-a2.md
 * 12.8): an in-air contact in the hover at 1.00 / 1.20 m or during a forward
 * MOVE, with the speed estimate off by e at the contact (the accelerometer
 * pulse and the observer's camVz, persistent) and one contact or flight 1's
 * train of five: LAND COLLISION, the camera hold blocked from that step as
 * the glue does it (controller_pid zeroes the applied correction, the hold
 * is not airborne), the controller blind, no climb, a slow touchdown. After
 * its verification: contacts through a MOVE up, a MOVE down (and the hover
 * at 0.50 m after it) and the re-track bob of a YAW (the blind LAND's hover
 * thrust from before the motion: at most the drone's own upward speed
 * climbs; a touchdown, never LAND_TIMEOUT, below the knee).
 *
 * Added for the end of battery (user decision 2026-10-13, late; docs/
 * autonomy-a2b.md "End of battery (2026-10-14)"): the drain scenario. The
 * battery drains with the thrust (a generic LiPo OCV(q) table, a sag of
 * 1.15e-5 V per count growing 50 % below 20 % charge, capacity C in A s),
 * K follows the loaded voltage (206.4k - 12.3k x V, the pooled fit, + an
 * offset), the 10 Hz reading carries 35 mV of noise; the Mac tries a MOVE
 * every 6 s and STOPs to keep the wait alive. From 3.82 V at rest (night f6)
 * every sortie must end BATT_LOW (3.00 V or T_hold > 57000 for 0.5 s), a
 * slow LAND, never the controller's HOLD_CAPPED / CLIMB_CAPPED, no MOVE
 * accepted above a T_hold of 53400; from 4.10 V with C 400 the 80 s air
 * clock lands first.
 *
 * Added after the verification of cb09352: the step-drop battery case at
 * both rest levels (3.40 V, just over the floor: BATT_LOW; the old 3.05 V
 * collapse, pinned: a backstop LAND, bounded); the drain grid at the drone's
 * own K prior (NVS vzK 160400, and 158000), where Khat's K_RANGE clamp ends
 * some take-offs of a drone heavier than the pooled fit; a sweep of
 * take-offs from 3.60-3.85 V at rest (where the take-off fast LANDs start,
 * and how deep the loaded voltage goes in the air).
 *
 * Added for more air per charge (2026-10-14, docs/autonomy-a2b.md 17.4 and
 * 17.5): the drain check allows the fast LAND's brake (<= 1.03 x T_ref with
 * T_ref <= 53400) and checks that no BATT_LOW LAND is fast; the LAND
 * durations of the mission, of a LAND after a move and of 0.20 + 1.00 m;
 * far-scene moves at 0.30 m/s against the same seeds at 0.20
 * (testFarMoves); a STOP mid-move drifts at most 0.10 m further.
 *
 * Added for the hold's gain scale with the scene distance (2026-10-14,
 * docs/autonomy-a2b.md 20): the hold gets camera_flow.c's gain scale (the
 * observer's G = AUTO_MV_FY_PX / (D (1 + eD)), its tracked bit from 1.3 s
 * after lift-off as logged, camHoldGainTarget, camHoldGainSlew) with kmax
 * from the command line (default 2.5, the firmware's; 1 = the hold before
 * it): every check must pass with both. An option couples the attitude
 * filter (scen_t.coupled, docs/autonomy-a2b.md 20.2): the controller holds
 * the estimate, the true lean is the estimate minus the filter's error d,
 * d' = 0.384 (drag v / g - e) + I, I' = 0.02 (drag v / g - e) (Mahony Kp at
 * 960 loops/s, mki 0.04; the accelerometer reads only the rotor drag in the
 * air), the lean error a setpoint offset. testTakeoffGrid flies plain
 * hovers with it: 20 seeds x lean error +-0.3 deg x D 1/2/3/4 m x the
 * observer's D error -0.3/0/+1.0, kmax 2.5 against 1.
 *
 * Added after the verification of 5679a6b (2026-10-14 fixer, docs/autonomy-a2b.md 20.4): testFarMovesSeeds, the
 * far-scene moves over 16 seed sets at the run's kmax and at 1 (testFarMoves pins one set, whose outcome flips with
 * the seed): the rate of moves that do not end DONE, bounded and compared with kmax 1.
 *
 * Added after the review of 148af89 (2026-10-15, docs/autonomy-a2b.md 21.9): the camera height observer's latched G
 * (scen_t.gLatch: camVz's part below ~1 s read x G_true / G_used, G taken after 2 s of vertical excitation, or never
 * in HOVER for the review's pessimistic tracker; the firmware's gain following the MOVEs' measured progress, or not)
 * in the long-move grid and testLatchedG; the expansion scale below K_LOW (testBelowKLow, documented); the TAKEOFF's
 * plan at its largest with the Mac's floor (testPlanFloorSim; 0.60 m until 2026-10-17).
 *
 * Added for the bolder ahead margins (user decisions 2026-10-17, docs/autonomy-a2b.md 32): scen_t margins (the guard
 * margins a run latches: the defaults, or the ones before), the Mac's floor following the drone's (0.35 m), fwdMax at
 * the drone's floor; testFrontBraking: the drone's largest forward MOVE with an overshoot forced by a forward push
 * (scen_t fwdPush) near the scene, at the margins of 2026-10-17 against the ones before: contacts, their speed, the
 * FRONT stops' travel (the analysis behind 0.30 / 0.25 / 0.35 m and the near-scene braked view loss, mvnear).
 *
 * Added for the move fixes (the 300 % forensics of 2026-10-17, auto_motion.h "move fixes", docs/autonomy-a2b.md 33): the
 * camera task's part as camera_flow.c does it (camHoldGainMoveFix, ffBrakeScale, camHoldMoving into the slow inputs;
 * scen_t fixSet / fixBits, the margins' mvfix otherwise); testDriftLean: forward / back 1.00 m from D 2.3 m at 100-300 %
 * with the drone drifting when the MOVE is sent (scen_t driftDeg, kickV) or an unlearned 0.8 deg ramped in during it
 * (leanDeg), with the default mvfix, with STILL too, and without them (the true end, the end the drone reports, how it
 * ended); the fast moves' profile time less the still wait, the stall bound + 0.15 s.
 *
 * Added for the review of 2026-10-17 (docs/autonomy-a2b.md 34): testSideNearScene, the largest sideways MOVE from just
 * above the floor with a forward push, the default mvfix (FRONT: the live front stop and the near-scene brake on every
 * horizontal MOVE) against without FRONT.
 *
 * Build and run from Firmware/esp-drone:
 *   C=components/core/crazyflie
 *   cc -std=c99 -O2 -Wall -Wextra -DAUTO_MV_HOST_VH_OVERRIDE -I$C/utils/interface -I$C/modules/interface \
 *      -I$C/hal/interface tools/tests/test_interactive_sim.c $C/utils/src/auto_fsm.c $C/utils/src/auto_motion.c \
 *      $C/utils/src/vert_ctrl.c $C/utils/src/camera_hold.c $C/utils/src/cam_vz_obs.c -lm \
 *      -o /tmp/test_interactive_sim && /tmp/test_interactive_sim [kmax]
 * ('/tmp/test_interactive_sim 2.5 real300': the fast moves' safety cases at mvacc 30 / mvsettle 1000, information.)
 * (-DAUTO_MV_HOST_VH_OVERRIDE, host only: testFarMoves flies the same seeds at
 * 0.20 m/s too; without it that comparison is skipped.)
 * Lines starting with "info:" are measured numbers, not checks.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_fsm.h"
#include "camera_hold.h"
#include "cam_vz_obs.h"    /* CAM_VZ_TURN_MAX_YAW_RATE */
#include "autonomy.h"   /* AUTONOMY_SLOW_EVERY */

static int failures = 0;
static int checks = 0;
/* The hold's largest gain scale (camera_flow.c holdkmax; argv[1], default the firmware's). */
static float simKmax = CAM_HOLD_GAIN_KMAX_DEFAULT;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define D2R 0.0174532925f
#define G 9.81f
#define ROOM 3.0f           /* walls at +/-3 m (the default room) */
#define SH_MAX_PART AUTO_SH_MAX_M   /* Claude sends at most this part of a larger error (the runbook) */
static float roomR = ROOM;  /* this run's walls at +/-roomR (scen_t.room; the long moves: 6 m) */
/* An optional box face in front of the +y wall: the plane y = BOX_Y for x in BOX_X0..BOX_X1. */
#define BOX_Y 1.5f
#define BOX_X0 (-0.4f)
#define BOX_X1 0.4f
static bool boxOn;
#define CAM_RUNNING 2
#define TOKEN 0xC0DE0001u

static uint32_t rng = 1u;
static float uni(void)
{
  rng ^= rng << 13;
  rng ^= rng >> 17;
  rng ^= rng << 5;
  return (float)(rng >> 8) * (1.0f / 16777216.0f);
}
static float gauss(void)
{
  float s = 0.0f;
  for (int i = 0; i < 12; i++) {
    s += uni();
  }
  return s - 6.0f;
}

/* The guard margins a run's ARM latches (scen_t margins; auto_motion.h "guard margins", 2026-10-17): 0 the defaults, 1
 * the margins before 2026-10-17 (autoMvMarginsOld). The Mac's floor is the drone's + 0.05 m (espfly_auto.py
 * mac_min_front: 0.35 m at the default 0.30, 0.60 m with the 0.50 m before it as the Mac had then). */
static void simMargins(int which)
{
  autoMvMargins_t g;
  if (which == 1) {
    autoMvMarginsOld(&g);
  } else {
    autoMvMarginsDefault(&g);
#ifdef SIM_FIX_DEFAULT
    g.fix = SIM_FIX_DEFAULT;   /* (development: the default margins with other move fixes) */
#endif
  }
  autoMotionSetMargins(&g);
}

static float macFloorM(void)
{
  autoMvMargins_t g;
  autoMotionGetMargins(&g);
  return g.planCm == 60 ? 0.60f : (float)g.floorCm * 0.01f + 0.05f;   /* (the margins before: the Mac's 0.60 m) */
}

/* The Mac's largest forward move (espfly_auto.py fwd_max): ends >= the Mac's floor (macFloorM: 0.35 m at the default
 * margins, 0.60 m before 2026-10-17) from the scene d ahead at k 0.6, <= 4 m. */
static int16_t macFwdMax(float d)
{
  const float fl = macFloorM();
  for (int fc = AUTO_MV_FWD_MAX_CM; fc > 0; fc--) {
    if (d * powf((d - 0.01f * (float)fc) / d, 1.0f / AUTO_MV_K_LOW) >= fl) {
      return (int16_t)fc;
    }
  }
  return 0;
}

/* ---- the world ---------------------------------------------------------------- */

typedef struct {
  /* horizontal */
  float x, y, vx, vy;       /* m, m/s, world */
  float psi;                /* heading, rad, + counter-clockwise (the yaw angle) */
  float roll, pitch;        /* applied lean, deg */
  float camCorrR, camCorrP; /* controller_pid's slewed camera correction */
  float trimR, trimP;       /* body-fixed lean error, deg */
  /* camera */
  float k;                  /* expansion scale of the scene */
  float cvf;                /* vertical motion read as forward */
  float eD[8];              /* the observer's D error per view (index: heading / 45 deg) */
  float nextFrameT, lastFrameT;
  float pendT[8], pendW[8][2], pendDt[8];   /* frames in flight: delivery time, wFwd/wLat, dt */
  int pendN;
  bool camLost;             /* no frames at all */
  float lastHoldT;          /* sim time of the hold's last update */
  float yawAcc;             /* turn since the tracker's last restart, rad */
  float psiPrev;
  bool tracked;
  uint32_t resets;
  float bobS;               /* s of vertical excitation since the restart */
  float lastObsT;           /* the observer's last frame its yaw gate passed (sim time) */
  float airT;               /* s since the plant left the ground (0 on it) */
  float dR, dF, iR, iF;     /* coupled: the attitude filter's error (estimate - true, deg) and its integral, roll and
                               forward (-pitch) */
  /* the observer's latched G (scen_t.gLatch): the true scene distance when it last took G, the motions' forward
   * progress then (autoFsmOut_t fwdLn), the camera speed's part below ~1 s, the latest over-read */
  float gD, gLnAt, camLp, gRatio, gRatioMax, gRatioMin;
  float gVLp, gMs, gExcS;   /* ... its excitation: the vertical speed's low-pass (1 s), the high-pass's mean square
                               (1 s), the time it has stayed >= 0.03 m/s rms */
} world_t;

/* Distance from (x, y) along heading psi to the room's wall. */
static float wallD(float x, float y, float psi)
{
  const float c = cosf(psi), s = sinf(psi);
  float d = 1e9f;
  if (c > 1e-6f) d = fminf(d, (roomR - x) / c);
  if (c < -1e-6f) d = fminf(d, (-roomR - x) / c);
  if (s > 1e-6f) d = fminf(d, (roomR - y) / s);
  if (s < -1e-6f) d = fminf(d, (-roomR - y) / s);
  if (boxOn && s > 1e-6f && y < BOX_Y) {
    const float db = (BOX_Y - y) / s, xh = x + db * c;
    if (xh >= BOX_X0 && xh <= BOX_X1) d = fminf(d, db);
  }
  return d;
}

static int viewIdx(float psi)
{
  float a = fmodf(psi / D2R + 3600.0f + 22.5f, 360.0f);
  return (int)(a / 45.0f) & 7;
}

/* ---- one simulation ----------------------------------------------------------- */

typedef struct {
  /* scenario */
  uint32_t seed;
  float x0, y0, psi0;       /* start; the camera faces the wall at about 2 m */
  float eDmax;              /* the observer's D error, +/- */
  float trim;               /* +/- lean error, deg */
  bool mission;             /* the scripted mission */
  float camLossAtMoveS;     /* >= 0: the camera lost this long into the first move, for 3 s */
  float battDropAtS;        /* >= 0: the battery rest voltage falls at this sim time ... */
  float battDropV;          /* ... to this rest voltage (V; 0: AUTO_VBAT_LOW_LAND_V + 0.40, just over the floor) */
  bool airLimit;            /* hover until the Mac's LAND at MAC_AIR_LAND_MS of air (the wait never ends it) */
  float dipAtS;             /* > 0: one 1 kHz battery sample at 2.50 V at this sim time (VBAT_DIP; Vf stays up) */
  bool stopMid;             /* STOP 2.5 s into a 1.0 m move */
  /* one MOVE 10 s into the wait, LAND 2 s after its end */
  bool oneMove;
  int16_t oneR, oneF;
  int fwdMax;               /* 1: oneF is the largest forward move the drone accepts at the D then; 2: 1 cm more */
  float kFix;               /* > 0: the expansion scale k */
  bool frozen;              /* frames 'ok' but no motion in them from the MOVE on */
  bool box;                 /* the box face in front of the wall */
  /* vertical scripts: TAKEOFF toCm, nUp moves (the first firstAfterS into the wait, then 1 s after each end),
   * LAND landAfterS after the last; camVzOff: the camera's vertical speed offset (the estimate's drift) */
  uint16_t toCm;
  int nUp;
  int16_t ups[3];
  float firstAfterS, landAfterS, camVzOff;
  /* a hover above the take-off cap (AUTO_HEIGHT_CM_MAX 120 cm; 2.00 m by moves up only, 2026-10-15): preN MOVEs up of
   * preUps cm first, 1 s into the wait and 1 s after each end; every other script starts 1 s after the last one ends,
   * the wait's clock (phaseT) restarted there (toHeight()) */
  int preN;
  int16_t preUps[3];
  bool stopInBob;           /* YAW 90 with the tracker lost; STOP during its bob */
  /* LAND COLLISION (contact): a contact contactAtS into the wait (contactInMove: 1.5 s into a MOVE forward 60
   * sent 10 s into the wait) */
  bool contact;
  float contactAtS;
  float contactE;           /* the speed estimate's error from the contact on, m/s (+ reads a climb) */
  float contactEPersistS;   /* > 0: the camera's part of it lasts this long (the jolt's frames), then the sim's own
                               offset again; 0: persistent (the blind LAND's worst case until 2026-10-15) */
  bool contactTrain;        /* flight 1's five contacts (0 / 0.25 / 0.37 / 0.66 / 0.83 s), not one */
  bool contactInMove;
  int16_t contactUpCm;      /* contactInMove: a MOVE up (+) or down (-) this far instead of forward 60 */
  float contactInMoveS;     /* contactInMove: the contact this long after the MOVE was sent (0: 1.5 s) */
  bool contactInBob;        /* YAW 90 3 s into the wait (the tracker lost), the contact bobAtS into its re-track bob */
  float bobAtS;
  /* faster turns (user decision 2026-10-13, evening): one YAW turnDeg 3 s into the wait, LAND 2 s after it ends;
   * oldHoldGate / oldObsGate: the hold's / the observer's yaw gate at 0.35 rad/s in the turn too (before it) */
  int16_t turnDeg;
  bool oldHoldGate, oldObsGate;
  /* the forward hold in the take-off climb (camera_hold.h fwdOff, docs/autonomy-a2b.md 19.5): on, as before it */
  bool oldTakeoffHold;
  /* the end of battery: the battery drains from drainV0 at rest (capacity drainC A s, sag drainVs V per count of
   * thrust), K = 206.4k - 12.3k x the loaded V + kOff, 10 Hz reading noise vNoise (sd, V); a MOVE right +/-20 every
   * 6 s, STOPs keep the wait alive (airLimit) */
  bool drain;
  float drainV0, drainC, kOff, drainVs, vNoise;
  bool drainDescend;        /* drain: Claude's runbook above 1.2 m (docs/autonomy-a2b.md 23.5): from T_hold 52000 or a
                               reading of 3.20 V on, a vertical MOVE down (0.50 m at most, to 1.20 m) 1 s after the
                               previous one ends */
  float drainDescendT;      /* > 0: drainDescend from this T_hold (and no voltage cue) instead */
  float kPrior;             /* > 0: the controller's K prior (the drone's NVS vzK) instead of vzDefaultConfig's */
  /* the 120 s air clock (docs/autonomy-a2b.md 18.1): kTrueAbs > 0: the plant's K at the start, growing kDriftPerS
   * (a fraction per second; the battery's drain on a long hover); waitMs > 0: TAKEOFF actWaitMs (else 60000) */
  float kTrueAbs, kDriftPerS;
  uint32_t waitMs;
  /* the palm catch (docs/autonomy-a2b.md 18.3): a hand under or around the drone (harness only, never drone code) */
  int handKind;             /* 0 none, 1 a palm (pushes only), 2 a grab (holds both ways) */
  int handDuring;           /* 0 the hover wait (handAtS into it), 1 a MOVE right 100 (handAtS after it was sent),
                               2 a Mac LAND onto a palm held still handGap under the true height (sent handAtS into
                               the wait), 3 the TAKEOFF climb (a grab when the true height first passes handAtH),
                               4 the re-track bob (contactInBob's YAW 90, handAtS into the bob), 5 a MOVE up
                               (contactInMove with contactUpCm, handAtS after it was sent) */
  float handAtS, handAtH, handGap, handV, handLift, handK, handC, handWobM, handWobHz, handTilt, handGe;
  float handReleaseS;       /* > 0: the hand drops away at 0.5 m/s this long after the contact */
  float grabRollDps, grabYawDps;   /* the grab's rotation from 0.6 s after the contact */
  bool handFastLand;        /* handDuring 2: a fast LAND (the reason and height allow it) */
  float handGiveUpS;        /* > 0: this long after the contact the hand lowers away, the Mac LANDs 3 s later */
  float macLandAfterContactS;   /* > 0: the Mac's LAND this long after the hand's first touch or the in-air contact
                                   (it saw it in a photo), in TAKEOFF or HOVER */
  /* the false-positive environment: a vertical gust step gustG (g) for gustS from gustAtS into the wait, vertical
   * turbulence (Gauss-Markov, sd turbG g, tau 0.2 s), a lateral gust latGust (m/s^2, 0.5 s at gustAtS), a ceiling */
  float gustG, gustS, gustAtS, turbG, latGust;
  float ceilH;              /* > 0: a ceiling at this true height: thrust gain up to 1.5 within 1 cm, a stiff spring */
  /* hard contacts after a PALM cue (a soft first touch, then knocks: fingers, a bounce off the palm): hardN of them,
   * hardT[i] s after the cue, |acc| 1 + hardG[i] g for hardMs[i] ms (the FSM's COLLISION size from 0.6 g for 3 ms)
   * and the controller's accZ + hardG[i], an upward kick hardDv[i] m/s at the start */
  int hardN;
  float hardT[16], hardG[16], hardDv[16];
  int hardMs[16];
  /* the hold's gain scale (docs/autonomy-a2b.md 20): coupled: the attitude filter's coupling (the header); eDfix: the
   * observer's D error eDval in every view instead of +-eDmax; trimFixed: the lean error exactly +-trim per axis
   * (signs from the seed); endAfterLiftS > 0: the run ends that long after lift-off */
  bool coupled, eDfix, trimFixed;
  float eDval, endAfterLiftS;
  bool noCamNoise;          /* the camera's 0.03 /s per frame of noise off (the draws kept: the same seeds) */
  /* long moves and the planned move from take-off (2026-10-14, docs/autonomy-a2b.md 21): room > 0: walls at +-room m
   * (else 3); drag > 0: the plant's drag, 1/s (else 0.35); moveAtS > 0: oneMove's MOVE this long into the wait (else
   * 10 s); planR / planF: the TAKEOFF's planned move (oneMove's results track it from the wait's start; LAND 2 s
   * after its end); planAtHover: instead, the same move put into the state machine's motion at the HOVER entry
   * (white-box: the rejected alternative, docs 21.4); gTrans > 0: the observer's G x (1 + gTrans e^(-t / 0.4 s))
   * from the HOVER entry (gTrans before it), its D and the gain scale's G with it; csv: the hold's output and the
   * gain inputs at 100 Hz in the recorder's columns (tools/hold_band.py) */
  float room, drag, moveAtS;
  bool macCap;              /* oneMove: the forward part capped at the Mac's largest (macFwdMax) at the D then */
  float hoverWinS;          /* > 0: no move; the true height's excursion over this long from 3 s into the wait (res
                               hExcHover), LAND 2 s after */
  bool noGainMove;          /* the gain scale without camHoldGainMove (s falls to 1 from a MOVE's start): b6a54da + 21.3's
                               0.75 /s alone, the comparison */
  int16_t planR, planF;
  bool planAtHover;
  float gTrans;
  const char *csv;
  /* the observer's latched G (2026-10-15 review, docs/autonomy-a2b.md 21.9): it turns the vertical image motion into
   * speed with G = f / D, and takes G only with vertical motion (here, as its tracker: the true vertical speed's
   * high-pass (1 s) at >= 0.03 m/s rms for 2 s, the climb, a bob, a vertical move, LAND; a yaw restart keeps it as
   * the guess). camVz's part below ~1 s (the camera's
   * band; the accelerometer's above) is read x (G_true / G_used): 1 the firmware (G_used follows the MOVEs' measured
   * progress, cam_vz_obs.h), 2 without that (b6a54da / 148af89), 0 off (camVz as before); gNoRetake: the
   * pessimistic tracker (the review's stale_drv2): no G taken in HOVER at all, however the drone bobs */
  int gLatch;
  bool gNoRetake;
  /* a physical obstacle the camera does not see (2026-10-15 verification of "Claude decides"; the verifier's wall
   * sim): a rigid plane at world y = wallY (wallMode 1) or x = wallX (2), the drone's guard radius 0.05 m, restitution
   * wallE; a hit's speed change dv makes the state machine's |acc| pulse 1 + dv / (g 4 ms) for 4 ms (and gx 300 dps)
   * when over 1.6 g. wallMode 1: a MOVE forward wallFwdCm wallMoveAtS into the wait (into the plane); 2: no motion,
   * latGust pushes the drone into the plane. The Mac's LAND only wallLandAtS into the wait (Claude's 5-30 s per
   * decision; no LAND 3 s after the contact as the contact scenarios have). */
  int wallMode;
  float wallY, wallX, wallE, wallMoveAtS, wallLandAtS;
  int16_t wallFwdCm;
  int16_t wallBackCm;       /* > 0 (wallMode 1): Claude's runbook after a CONTACT, a MOVE back this far 5 s after it */
  /* a LAND long after a contact (user decision (B) 2026-10-15: every LAND of a sortie that had a contact is the blind
   * one): with contact (contactAtS into the wait, no Mac LAND for it), the LAND of kind lateLand lateAtS into the
   * wait: 1 the Mac's LAND, 2 BATT_LOW (the rest voltage just over the floor), 3 VBAT_DIP (one 2.50 V sample), 4 the
   * heartbeat stops (HB_LOST), 5 the camera out (no frames, the observer unhealthy) for good (SINK), 6 the camera out
   * lateCamS, then back (SINK, RESUME), the Mac's LAND 5 s after the RESUME, 7 the phone's stick (APP_INPUT), 8 three
   * loop gaps of 25 ms 0.1 s apart (LOOP_GAP), 9 a session change (SESSION), 10 the controller's own LAND: its height
   * estimate pushed 2 m out of range (white-box; EST_RANGE), 11 the controller's own LAND at holdMaxS of HOLD
   * (HOLD_TIMEOUT, its backstop shortened) */
  int lateLand;
  float lateAtS, lateCamS, holdMaxS;
  /* SET_HEIGHT (2026-10-15, docs/autonomy-a2b.md 23): shLoop 1 a scripted Claude, 2 the baseline (the same timeline,
   * no command). Claude takes a 'photo' shP s after the previous correction's acceptance or the previous photo (the
   * first 1 s into the wait): X = the true height then + N(0, shNoise) clipped +-0.15 + shBias, h_at = the estimate
   * then; sent shL s later (the offset form) only if |X - h_at| > 0.10, its 0.5 m part when larger; a BUSY retried
   * after the motion's end, SH_RATE 2 s later. The Mac's LAND at shDurS of air. shScale: the camera's vertical speed
   * read (1 + shScale) x the true one. shSingle != 0: instead one SET_HEIGHT X = the estimate + shSingle (now) shAtS
   * into the wait, the LAND 8 s after its end. */
  int shLoop;
  float shP, shL, shNoise, shBias, shScale, shDurS, shSingle, shAtS;
  /* The replay of a logged correction (2026-10-16, docs/autonomy-a2b.md 27.2): shRep 1 the "go to the target" form, 2
   * the 4-byte form as it was sent. shAtS into the wait the logged geometry is set (white-box): the plant's true
   * height to shRepX (Claude's judged height; the plant's height enters only its ground effect, nil at 1 m) and the
   * controller's estimate to shRepAt (the photo's estimate at capture), so the true height, the estimate and the
   * commanded height stand as they did; then Claude's photo judges the true height exactly; shRepLat s later
   * SET_HEIGHT goes: X with h_at and the commanded height as the target, or (2) X + shRepSentOff (the logged sent -
   * judged, the est-rule) in the 4-byte form. The Mac's LAND 8 s after its end. */
  int shRep;
  float shRepX, shRepAt, shRepLat, shRepSentOff;
  /* Fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31): mvSpeed > 100 the sortie's speed step (set right after
   * the ARM step, as autonomy.c does), the hold's move lean limit and slew (camHoldMoveLimits), controller_pid's apply
   * slew following the hold's (cameraFlowHoldSlew) and its brake lean with the hold stale (autonomyBrakeLean); 0 or
   * 100: as before. oneMove: stopAtS > 0 a STOP this long after the MOVE's acceptance; camStallAtS > 0 the camera's
   * frames stop that long after it, for camStallS (the frame age grows from the last frame, as camera_flow.c's) */
  uint16_t mvSpeed;
  float stopAtS, camStallAtS, camStallS;
  /* The hold's HOVER scene gate (camera_hold.c camHoldSceneOk, camera_flow.c holdUpdate; added after the review of
   * cbccec5): the measured expansion's EMA over 0.3 /s, or |dxRes| over 1.5 px (the measured sideways rate x
   * AUTO_MV_FY_PX x the frame interval), refuses the frame in autonomy HOVER after 2 s of it (the hold coasts, a
   * MOVE pauses). On in every scenario; noSceneGate: the sim before it. */
  bool noSceneGate;
  /* oneMove: camRefuseS > 0 the frames delivered from camRefuseAtS after the MOVE's acceptance, for camRefuseS, are
   * refused (a person crossing: the hold coasts, the move pauses) */
  float camRefuseAtS, camRefuseS;
  /* The guard margins (2026-10-17, simMargins): 0 the defaults, 1 the margins before (set before the run, so the
   * scenario's own cap computations, macFwdMax and fwdMax, see them; the ARM latch emulation sets them again). */
  int margins;
  /* oneMove: a forward push (m/s^2 along the heading: what makes a move overshoot at speed, a gust, a lean the hold
   * has not learned) from pushAtS after the MOVE's acceptance (< 0: that long before its profile's planned end) for at
   * most pushS, and only while the MOVE runs (it ends with the motion: the braking after a stop is the drone's) */
  float fwdPush, pushAtS, pushS;
  /* The move fixes (2026-10-17, auto_motion.h "move fixes", mvfix): fixSet the latched bits are fixBits instead of the
   * margins' (the defaults 0x1F, the margins before 0); the camera's GAIN and BRAKE follow them as camera_flow.c's */
  bool fixSet;
  uint8_t fixBits;
  /* oneMove (the 300 % forensics' cases): from driftAtS before the MOVE is sent, a pitch error driftDeg the hold has
   * not learned (deg, + nose up: backward; kept: flight 6's drift between its moves, ~0.8 deg of hover lean shift
   * while the IMU cooled, still moving back at 0.15-0.17 m/s at move 2's acceptance) and / or the drone's speed set to
   * kickV m/s along -heading then; leanDeg a pitch error ramped in over leanS from the MOVE's acceptance and kept (deg,
   * + nose up; flight 6's ~0.8 deg) */
  float driftDeg, driftAtS, kickV, leanDeg, leanS;
} scen_t;

typedef struct {
  int ncmd;
  char name[8][24];
  float want[8][3];         /* right, forward, up (m) */
  float got[8][3];          /* true displacement from the hold's equilibrium point when the command came */
  float gotNow[8][3];       /* ... from the drone's position then (the hover's own wander included) */
  float yawWant, yawGot;
  autoReason_t landWhy, endWhy;
  bool killed, touchdown;
  float impactV, hMax;
  int refused;
  float stopDrift;          /* stopMid: true travel after the STOP, m */
  float stopTravelAtStop;
  uint32_t landAirMs;
  bool pausedSeen;
  float maxLean;
  /* oneMove */
  int oneWhy;               /* the MOVE's refusal (autoMvWhy_t; 0 accepted) */
  autoMvEnd_t oneEnd;
  float oneEndS;            /* from its acceptance to its end */
  float d0Obs;              /* the observer's D when it was sent */
  int16_t oneF;             /* the forward cm sent (fwdMax) */
  float mvR, mvF;           /* true displacement from its acceptance to the LAND command (its body frame) */
  float landR, landF;       /* true displacement from the LAND command to contact (the same frame) */
  float landHV, tdHV;       /* the largest horizontal speed in LAND, and at contact */
  float minWall, wallAtLand;
  /* vertical scripts */
  float hMinHover;          /* the lowest true height in HOVER after the first vertical move was sent */
  bool groundInHover;
  /* stopInBob */
  bool bobSeen, stopAcked, bobDone, landedSoon;
  /* contact */
  bool contactDone, blockedAtLand, collidedAtLand, mvStoppedAtLand;
  float hAtContact, hMaxAfterContact, corrAfter, travelLand;
  uint32_t contactLandMs;   /* from the contact to the touchdown (IDLE's step) */
  /* turnDeg */
  float turnS;              /* from the YAW's acceptance to the end of the turn itself (turnDone) */
  float turnDrift;          /* the largest horizontal distance from the YAW's start until it ended, m */
  float turnYawErr;         /* the heading 1 s after the turn against the command, deg */
  int turnFrames, turnFramesEng;   /* hold frames during the turn itself, and of them engaged */
  bool turnPaused, turnEnded;
  autoMvEnd_t turnEnd;
  float turnRateMax;        /* the heading's largest rate in the turn, deg/s */
  uint8_t vzLandWhy;        /* the controller's reason of a LAND VZ */
  /* drain */
  float tAt538;             /* sim time T_hold first reached 53800 in HOVER (-1: never) */
  float landT, landTHold, landV, landVRest;   /* at the FSM's LAND: sim time, T_hold, loaded and rest voltage */
  float minVLand;           /* the lowest loaded voltage in LAND while airborne */
  float maxThr;             /* the largest thrust applied */
  float maxLandOverRef;     /* the largest LAND thrust / T_ref (the controller in LAND) */
  float maxMoveTHold;       /* the largest T_hold the FSM read when it accepted a MOVE */
  int nMoveOk, nLowBatt;
  int nDescend;             /* drainDescend's MOVEs down sent */
  float minVAir;            /* the lowest loaded voltage while airborne (any state) */
  float hAtLand;            /* the true height at the FSM's LAND */
  float kRelAtLand;         /* the plant's K over the controller's kPrior at the FSM's LAND */
  autoState_t stateAtLand;  /* the FSM's state on the step before its LAND (AUTO_TAKEOFF: a take-off LAND) */
  float kRelHover;          /* the plant's K over kPrior when the FSM first entered HOVER (0: never) */
  /* the controller's LAND (the fast LAND, docs/autonomy-a2b.md 17.4) */
  bool vzLandSeen, landFast;
  float vzLandT, landTRef;  /* sim time of the controller's LAND start, its T_ref */
  float landDurS;           /* the controller's LAND start to DONE (-1: never) */
  /* the palm catch */
  bool handContact, cued, gaveUp, released, supInAir, rotated;
  uint8_t endVzWhy;         /* the controller's reason when the sortie ended (IDLE) */
  float contactT, cueT, offT, hContact, hLowHeld, cueTHold, thrAfterCue, floorT, offAfterFloorS;
  /* the Mac's LANDs (MAC_AIR_LAND_MS) and the hold-and-report events */
  bool macWaitLand, macAirLand;
  uint32_t evMask;          /* 1 << autoEvent_t of every event the state machine reported */
  int contacts, sinks, resumes;
  bool hushSeen;            /* vzInput_t.hush asked at least once */
  float hMaxAfterHand;      /* the highest true height from the hand's first touch to the LAND */
  float hMaxAfterContact2;  /* the highest true height from the in-air contact to the LAND (contact scenarios) */
  /* the 120 s air clock */
  bool groundInWait;        /* the plant on the ground while the FSM hovered in its wait */
  bool cutInAir;            /* the controller's LAND_TIMEOUT cut with the plant off the ground */
  float hMaxHover;          /* the highest true height while the FSM was in HOVER */
  bool offSeen, offFree, offCeil;   /* the motors' first stop in LAND/KILLED: with the drone free in the air (not on
                                       the floor, a palm or a ceiling: a fall), or pressed against a ceiling */
  float hOff;               /* the true height then */
  bool collidedInLand;      /* the FSM's collided set during the LAND (a hard contact in a PALM LAND) */
  /* the take-off: the true displacement from the start 2 s after the HOVER entry (forward / right, m), the hold's
   * pitch at the HOVER entry */
  float hoverT0, dFwdHover2, dRightHover2, holdPHover;
  bool hover2Seen;
  /* the gain scale (testTakeoffGrid): lift-off time, |displacement| from the start at HOVER + 3 s and lift-off + 8 /
   * 15 / 45 s (-1: not reached), the largest over lift-off..+15 s; the applied hold correction over lift-off +
   * 20..45 s: its peak-to-peak (the larger of roll and pitch), the same in the 0.3-1.5 Hz band (the flight check's
   * oscillation, first-order high-pass 0.3 Hz then low-pass 1.5 Hz) and that band's rms; the gain scale's largest
   * value and its value at lift-off + 15 s */
  float liftT, xH3, xL8, xL15, xL45, max15, p2p, p2pBand, rmsBand, sMax, sL15;
  /* the long moves: while the move (or the plan) ran, the true height's largest excursion from its value at the
   * acceptance and the controller's estimate's largest error against its reference (m); the plan's motion count at
   * the wait's first step; the move's true travel along its own frame from its acceptance to its end + 2 s */
  float hExcMove, hErrMove;
  int planSeqAtWait;
  float accT;               /* sim time of the move's acceptance */
  /* at the move's end: the motion module's measured progress (rel units) and its conversions, the true travel in the
   * move's frame (m), the true distance to the scene */
  float endPLat, endPFwd, endDLat0, endD0, endLatM, endFwdM, endTrueR, endTrueF, endWall;
  float eqR0, eqF0;         /* the hold's equilibrium point at the move's acceptance, from the drone (as the mission's) */
  float trackMin;           /* the lowest measured / reference forward progress past AUTO_MV_FROZEN_REF in the move */
  float hExcHover;          /* hoverWinS: the true height's largest excursion over the window */
  float gRatioMax;          /* gLatch: the camera speed's largest over-read in HOVER (G_true / G_used) */
  float gRatioMin;          /* ... and its lowest (k > 1: the gain over-corrected, the speed reads slow) */
  /* the obstacle (wallMode): the first CONTACT event in HOVER (sim time, -1: none), the gap to the plane every 0.5 s
   * from it (m), the gap 3 s after it, the lowest gap from then to the LAND, the time pressed against it from then,
   * hits, KILLED */
  float wallT, wallGap[100], wallGap3, wallMinLate, wallPressLate;   /* (wallMode 1: the CONTACT that stopped the
                                                                        MOVE, AUTO_CONTACT_MOTION) */
  int wallGapN, wallHits;
  int wallHitsLand;         /* hits in LAND (the descent's drift back into the plane) ... */
  float wallLandHitV, wallLandHitH;   /* ... their largest speed into the plane and true height */
  /* every controller LAND (a SINK and a later one count apart): their number, how many began blind (vzInput_t.blind
   * on their first step) and how many with the camera hold blocked from their start (out.collided); the true height's
   * largest rise over its value at a LAND's start, until the LAND ends; the hold blocked while the plant was in the
   * air before the controller's contact or touchdown */
  int vzLands, vzLandsBlind, vzLandsCollided;
  float hRiseLand;
  bool blockedInAir;
  /* the floor in LAND: the first arrival (sim time, -1: none), the highest true height after it (a bounce or a float in
   * ground effect), the time from it to the motors off */
  float floor1T, hAfterFloor, floorToOffS;
  /* the last controller LAND (docs/autonomy-a2b.md 22.10): the true height at its start, the time with the motors on
   * and the true height under 0.12 m in it (the floor, a float in ground effect or the last centimetres of the
   * descent: a float that never touches counts too), its flat part without and with the drift allowance at its start
   * (vzState_t landFlatBase, set in a blind LAND only, and landFlatEff); the true height at the first RESUME (-1:
   * none) */
  float hLastLand0, nearFloorOnS, lastLandBase, lastLandFlat, resumeH;
  /* SET_HEIGHT: the true height's error from the take-off height every 0.1 s from 10 s into the wait (shErr), at the
   * LAND command (shErrEnd); the highest true height in HOVER; corrections accepted, refusals by reason, BUSY / RATE
   * retries, corrections that ended PROGRESS; the thrust's largest relative deviation from its 2 s mean (frozen at a
   * correction's start) during corrections and in the plain hover; the shifts vzShiftHeight refused; the single
   * correction's end, its duration and its final error against its physical target */
  int shErrN, shAcc, shRef[16], shRetries, shProgress, shShiftRefused;
  float shErr[2000], shErrEnd, shHMax, shThrDevCorr, shThrDevHover;
  autoMvEnd_t shSingleEnd;
  float shSingleS, shSingleErr;
  /* the replay (scen_t shRep): the true height at the photo, at the command, 2 s after the correction's end, its result */
  float shRepH0, shRepHCmd, shRepH2;
  int shRepResult;
  /* fast moves (scen_t mvSpeed; oneMove): the profile's planned time (max of the horizontal and vertical), the time in
   * RUN / PAUSED, its peak speed, the largest lead of the true image expansion over the reference's (k v / D over
   * rateFwd, where rateFwd is over half the step's W_FWD), the largest true image rates (k v_fwd / D, |v_right| / D) and the true speed
   * while it ran; stopAtS / camStallAtS: the reference speed then, the true travel from then to the move's end + 2 s
   * (m), the true speed then */
  float profT, oneRunS, oneVpk, leadMax, wFwdTrueMax, wLatTrueMax, vTrueMax;
  /* the scene gate (scen_t noSceneGate): the measured expansion EMA's largest |value| while a MOVE ran or settled, and
   * the frames it refused then (any reason) */
  float expEmaMax;
  int sceneRej;
  float evtVRef, evtTravel, evtVTrue;
  float evtX, evtY;
  bool evtSeen;
  float endVTrue;           /* the true speed when the MOVE ended (oneEnd; with endWall: a FRONT stop's) */
  float stillS;             /* oneMove: the time its profile waited at its start for a still drone (mvfix STILL) */
  float vAtSend;            /* oneMove: the true speed along -heading when the MOVE was sent (+: moving back) */
  bool wallTouch;           /* oneMove, before the LAND: the drone's 0.05 m guard radius reached the wall ahead ... */
  float wallTouchV;         /* ... at this true speed (the first time) */
} result_t;

/* The drain scenario's cell: a generic LiPo open-circuit voltage against the charge q (%, 5 % steps). */
static const float OCV_V[21] = {3.27f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f, 3.84f,
                                3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};
static float ocvOf(float q)
{
  if (q <= 0.0f) {
    return OCV_V[0];
  }
  if (q >= 100.0f) {
    return OCV_V[20];
  }
  int i = (int)(q / 5.0f);
  i = i > 19 ? 19 : i;
  return OCV_V[i] + (OCV_V[i + 1] - OCV_V[i]) * (q - 5.0f * (float)i) / 5.0f;
}
static float qOfV(float v)
{
  float lo = 0.0f, hi = 100.0f;
  for (int k = 0; k < 50; k++) {
    const float m = 0.5f * (lo + hi);
    if (ocvOf(m) < v) {
      lo = m;
    } else {
      hi = m;
    }
  }
  return 0.5f * (lo + hi);
}

/* Every run without a hand (handKind 0) and without a gust or a ceiling: the controller's SUPPORT touchdown in the air
 * must never happen (counted over the whole program, checked at the end of main). */
static int fpRuns, fpSupAir;

#define HAND_M_KG 0.028f   /* the drone's mass, for the hand's force in g */
#define HAND_LIFT_S 0.20f  /* the palm's lift (handLift) takes this long from the contact */
/* Since 2026-10-15 ("Claude decides", docs/autonomy-a2b.md 22) the state machine lands on none of the hover's clocks
 * (no HOVER_TIMEOUT, no AIR_TIME). The harness plays the Mac: its LAND once the wait has run its length from the
 * last motion (the state machine's waitMs, what HOVER_TIMEOUT measured: macWaitLand), and with airLimit its LAND at
 * MAC_AIR_LAND_MS of air (the 120 s air clock's time until then: macAirLand). Both are LAND_CMD. */
#define MAC_AIR_LAND_MS 120000u

static autoCmd_t cmdOf(autoCmdType_t t)
{
  autoCmd_t c;
  memset(&c, 0, sizeof(c));
  c.type = t;
  return c;
}

static void runSim(const scen_t *sc, result_t *res)
{
  simMargins(sc->margins);   /* (the scenario's caps were computed with them too: the caller set them) */
  memset(res, 0, sizeof(*res));
  res->resumeH = -1.0f;
  int wallPulse = 0;        /* steps of the obstacle's |acc| pulse left */
  float wallPulseG = 0.0f, wallT0 = -1.0f;
  res->wallT = -1.0f;
  res->wallMinLate = 9.0f;
  rng = sc->seed * 2654435761u + 1u;
  roomR = sc->room > 0.0f ? sc->room : ROOM;
  res->trackMin = 9.0f;
  const float drag = sc->drag > 0.0f ? sc->drag : 0.35f;
  const bool planned = (sc->planR != 0 || sc->planF != 0) && !sc->planAtHover;
  float hoverT = -1.0f;     /* the HOVER entry (the G transient) */
  float hAcc = 0.0f;        /* the true height at the move's acceptance (hoverWinS: the window's start) */
  bool hoverWinOn = false;
  uint8_t csvFlags = 0;
  uint16_t csvGain = 0;
  FILE *csvF = sc->csv ? fopen(sc->csv, "w") : NULL;
  if (csvF) {
    fprintf(csvF, "flightId,tickMs,autoState,mvState,cvFlags,cvGain_dpm,holdRoll_cdeg,holdPitch_cdeg\n");
  }
  world_t w;
  memset(&w, 0, sizeof(w));
  w.x = sc->x0;
  w.y = sc->y0;
  w.psi = sc->psi0;
  w.psiPrev = w.psi;
  w.trimR = sc->trim * (2.0f * uni() - 1.0f);
  w.trimP = sc->trim * (2.0f * uni() - 1.0f);
  w.k = 0.95f + 0.1f * uni();
  if (sc->kFix > 0.0f) {
    w.k = sc->kFix;
  }
  if (sc->trimFixed) {   /* the draws above, kept, give the signs */
    w.trimR = w.trimR < 0.0f ? -sc->trim : sc->trim;
    w.trimP = w.trimP < 0.0f ? -sc->trim : sc->trim;
  }
  boxOn = sc->box;
  res->hMinHover = 9.0f;
  res->minWall = 9.0f;
  w.cvf = 0.3f;
  for (int i = 0; i < 8; i++) {
    w.eD[i] = sc->eDmax * (2.0f * uni() - 1.0f);
    if (sc->eDfix) {
      w.eD[i] = sc->eDval;
    }
  }
  w.tracked = true;
  w.gRatioMin = 9.0f;
  w.lastHoldT = -1.0f;
  w.lastObsT = 0.0f;

  /* vertical: the controller on its sim plant with the camera observer model */
  vzConfig_t vc;
  vzDefaultConfig(&vc);
  if (sc->kPrior > 0.0f) {
    vc.kPrior = sc->kPrior;
  }
  vzSimConfig_t pc;
  vzSimDefaultConfig(&pc);
  pc.kTrue = vc.kPrior * (1.0f + 0.02f * (2.0f * uni() - 1.0f));
  pc.accAirG = vc.bAir + 0.002f * (2.0f * uni() - 1.0f);
  pc.camWakeS = 0.0f;
  pc.camConvS = 0.3f;
  pc.camVzOffset = 0.004f * (2.0f * uni() - 1.0f);
  if (sc->camVzOff != 0.0f) {
    pc.camVzOffset = sc->camVzOff;
  }
  pc.vRest = 4.10f;
  if (sc->kTrueAbs > 0.0f) {
    pc.kTrue = sc->kTrueAbs;
  }
  float dq = 0.0f;          /* drain: the charge, % */
  if (sc->drain) {
    pc.vRest = sc->drainV0;
    dq = qOfV(sc->drainV0);
    pc.vSag = sc->drainVs;
    pc.kTrue = 206400.0f - 12300.0f * (pc.vRest - pc.vSag * 45000.0f) + sc->kOff;
  }
  res->tAt538 = -1.0f;
  res->minVLand = 9.0f;
  res->minVAir = 9.0f;
  vzSimState_t sim;
  vzSimInit(&sim, &pc, sc->seed + 7u);
  vzState_t vz;
  vzOutput_t vo;
  memset(&vo, 0, sizeof(vo));
  bool vzActive = false;

  camHoldState_t hold;
  camHoldReset(&hold);
  camHoldScene_t scene;
  camHoldSceneReset(&scene);
  camHoldGainLatch_t gainLatch;   /* camera_flow.c's gain scale */
  camHoldGainReset(&gainLatch);
  float gainS = 1.0f;
  float holdR = 0.0f, holdP = 0.0f;
  bool holdEng = false, holdCoast = false;

  autoFsm_t f;
  autoFsmInit(&f);
  autoFsmIn_t in;
  memset(&in, 0, sizeof(in));
  autoFsmOut_t out;
  memset(&out, 0, sizeof(out));
  in.nowMs = 50000;
  in.sessionValid = true;
  in.randToken = TOKEN;
  in.appGestureAgeMs = 2000;
  in.appInputAgeMs = 60000;
  in.appThrustZero = true;
  in.autoEnabled = in.appRequired = in.gestureRequired = true;
  in.vmin = sc->drain ? AUTO_VMIN_LO : AUTO_VMIN_DEFAULT;   /* drain: the user's NVS vmin 3.30 V */
  in.airMaxNvsMs = AUTO_AIR_MAX_DEFAULT_MS;
  in.hoverWaitNvsMs = AUTO_HOVER_WAIT_DEFAULT_MS;
  in.hoverAccelNvsMs = AUTO_HOVER_ACCEL_DEFAULT_MS;
  in.vzParamsOk = true;
  in.loopGapUs = 1000;
  in.holdEnabled = true;
  in.motorsReady = in.motorGuardOn = true;
  in.heapIntFree = 15700;
  in.onGround = true;
  in.onGroundMs = 30000;
  in.camState = CAM_RUNNING;
  in.camFps = 15.6f;
  in.camViewMs = 20000;
  in.camFrameAgeMs = 100;

  float applied = 0.0f, vbat = sim.vbat;
  int phase = 0;            /* script position */
  float phaseT = 0.0f;      /* sim time the current script phase began */
  int cmdIdx = -1;
  float cmdX = 0, cmdY = 0, cmdZ = 0, cmdPsi = 0, cmdEndT = -1.0f;
  float eqR = 0, eqF = 0;   /* the hold's equilibrium point when the command came, body axes, from the drone */
  float stopX = 0, stopY = 0, stopT = -1.0f;
  bool landSent = false, inLand = false;
  float lastStopCmdT = -1.0f;
  float landX = 0, landY = 0, landPsi = 0;
  bool contactSeen = false, oneAcc = false, bobStopSent = false, blocked = false;
  int kc = -1;               /* the in-air contact's step (LAND COLLISION scenarios) */
  float camOff0 = 0.0f;      /* the camera's own speed offset before the contact */
  /* the hand and the environment (palm catch) */
  bool handOn = false, handGone = false, wasGround = true;
  float handTop = 0.0f, handVel = 0.0f, giveUpT = -1.0f, turbN = 0.0f, envN = 0.0f, handTiltNow = 0.0f;
  res->contactT = res->cueT = res->offT = res->floorT = res->floor1T = -1.0f;
  res->hLowHeld = 9.0f;
  float contactX = 0.0f, contactY = 0.0f;
  float bobStopT = -1.0f, bobFirstT = -1.0f;
  bool hardKicked[16] = {false};
  autoState_t prevState = AUTO_IDLE;
  const float dt = 0.001f;
  /* fast moves (scen_t mvSpeed): the sortie's step once ARMED, the hold's slew the controller applies at */
  bool speedSet = false;
  uint16_t simPct = 100;
  uint8_t simFix = 0;        /* the latched mvfix (autonomyMvFixBits: 0 until ARM) */
  bool driftKicked = false;  /* scen_t kickV / driftDeg: started */
  bool holdMovingPub = false;   /* camera_flow.c cameraFlowHoldMovingV */
  float holdSlew = CAM_HOLD_SLEW_DEG_S;
  float stallT0 = -1.0f;
  /* the late LAND (scen_t lateLand) */
  bool lateOn = false, lateCamOut = false;
  float lateT0 = -1.0f, resumeT = -1.0f, vzLandH0 = 0.0f;
  vzPhase_t prevVzPhase = VZ_IDLE;
  /* SET_HEIGHT's scripted Claude (scen_t shLoop / shSingle) */
  float shNextPhotoT = -1.0f, shSendT = -1.0f, shRetryT = -1.0f, shX = 0.0f, shAt = 0.0f, shNextErrT = 0.0f;
  float shThrLp = 0.0f, shThrFast = 0.0f, shThrRef = 0.0f, shAccT = -1.0f, shTarget = 0.0f, shEndT = -1.0f;
  float shLastD = 0.0f, shH0 = -1.0f;   /* shH0: the estimate the drone held at the wait's start (the intended height:
                                           what a correction flies back to; the climb ends within holdBand of its
                                           target) */
  bool shPend = false, shCorr = false, shSent = false;
  bool shRepShifted = false;
  float shRepPhotoT = -1.0f;
  /* the moves up before the scripts (scen_t preN) */
  int preIdx = -1;
  float preEndT = -1.0f;
  bool preDone = sc->preN <= 0;
  if (sc->holdMaxS > 0.0f) {
    vc.holdMaxS = sc->holdMaxS;
  }

  for (int step = 0; step < 200000; step++) {
    const float t = (float)step * dt;
    if (step % 100 == 0) {
      vbat = sim.vbat + (sc->drain ? sc->vNoise * gauss() : 0.0f);
    }
    if (sc->drain) {
      /* the cell: R = 0.18 ohm (x 1.5 at 0 % charge from 20 % down), I = sag / R, q -= I dt / C */
      const float rf = 1.0f + 0.5f * fmaxf(0.0f, (20.0f - dq) / 20.0f);
      pc.vSag = sc->drainVs * rf;
      const float amps = sim.tr > 1.0f ? pc.vSag * sim.tr / (0.18f * rf) : 0.2f;
      dq -= amps * dt / sc->drainC * 100.0f;
      pc.vRest = ocvOf(dq);
      pc.kTrue = 206400.0f - 12300.0f * sim.vbat + sc->kOff;
    }
    if (sc->kTrueAbs > 0.0f && sc->kDriftPerS != 0.0f) {
      pc.kTrue = sc->kTrueAbs * (1.0f + sc->kDriftPerS * t);
    }
    if (sc->battDropAtS >= 0.0f && t >= sc->battDropAtS) {
      /* default: loaded ~0.43 V lower, just under the BATT_LOW floor */
      pc.vRest = sc->battDropV > 0.0f ? sc->battDropV : AUTO_VBAT_LOW_LAND_V + 0.40f;
    }

    /* ---- the camera: frames captured, delivered 120 ms later to the hold ---- */
    const float cpsi = cosf(w.psi), spsi = sinf(w.psi);
    const float vF = w.vx * cpsi + w.vy * spsi, vR = w.vx * spsi - w.vy * cpsi;
    const float D = wallD(w.x, w.y, w.psi);
    if (t >= w.nextFrameT) {
      const float fdt = w.lastFrameT > 0.0f ? t - w.lastFrameT : 0.064f;
      w.lastFrameT = t;
      /* the observer's yaw gate on this frame (cam_vz_obs.c; the glue sets the turn from camGuessOk) */
      const float obsRate = fabsf(w.psi - w.psiPrev) / dt;
      if (obsRate < ((out.mv.camGuessOk && !sc->oldObsGate) ? CAM_VZ_TURN_MAX_YAW_RATE : 0.35f)) {
        w.lastObsT = t;
      }
      w.nextFrameT = t + 0.060f + 0.008f * uni();
      if (!w.camLost && w.pendN < 8) {
        const int i = w.pendN++;
        w.pendT[i] = t + 0.12f;
        const float noise = sc->noCamNoise ? 0.0f : 0.03f;
        w.pendW[i][0] = w.k * vF / D + w.cvf * sim.v / D + noise * gauss();
        w.pendW[i][1] = vR / D + noise * gauss();
        w.pendDt[i] = fdt;
        if (sc->frozen && cmdIdx >= 0) {   /* the view 'ok' but nothing moves in it */
          w.pendW[i][0] = 0.03f * gauss();
          w.pendW[i][1] = 0.03f * gauss();
        }
      }
    }
    while (w.pendN > 0 && w.pendT[0] <= t) {
      const float yawRate = fabsf((w.psi - w.psiPrev) / dt);
      /* camera_flow.c: the yaw gate, wider in a commanded turn (the recorder's mvState) */
      const bool turnNow = !sc->oldHoldGate && camHoldTurnActive((uint8_t)((uint8_t)out.mv.state | ((uint8_t)out.mv.kind << 4)));
      const bool yawOk = camHoldYawOk(yawRate * w.pendDt[0], w.pendDt[0], turnNow);
      if (sc->turnDeg != 0 && out.mv.kind == AUTO_MVK_YAW && out.mv.state != AUTO_MV_DONE &&
          out.mv.state != AUTO_MV_STOPPED && res->turnS == 0.0f) {
        res->turnFrames++;
        res->turnFramesEng += yawOk;
      }
      /* camera_flow.c holdUpdate: the HOVER scene gate on every frame (scen_t noSceneGate) */
      const bool sceneOk = camHoldSceneOk(&scene, out.state == AUTO_HOVER, w.pendDt[0], 0.0f,
                                          w.pendW[0][1] * AUTO_MV_FY_PX * w.pendDt[0],
                                          w.pendW[0][0] * w.pendDt[0]) || sc->noSceneGate;
      const bool refused = sc->oneMove && sc->camRefuseS > 0.0f && cmdIdx == 0 && res->accT > 0.0f &&
                           t - res->accT >= sc->camRefuseAtS && t - res->accT < sc->camRefuseAtS + sc->camRefuseS;
      if (out.mv.kind == AUTO_MVK_MOVE && (out.mv.state == AUTO_MV_RUN || out.mv.state == AUTO_MV_SETTLE ||
                                           out.mv.state == AUTO_MV_PAUSED)) {
        res->expEmaMax = fmaxf(res->expEmaMax, fabsf(scene.expEma));
        res->sceneRej += !sceneOk;
      }
      camHoldInput_t hin = {
        /* camera_flow.c: airborne while thrust > 30000 and the hold is not blocked (autonomyHoldBlocked: a contact in
         * LAND, or LAND COLLISION) */
        .enabled = true, .sticksCentred = true, .airborne = vzActive && vo.thrust > 30000.0f && !blocked,
        .viewOk = yawOk && sceneOk && !refused, .wFwd = w.pendW[0][0], .wLat = w.pendW[0][1], .dt = w.pendDt[0],
        .refEpoch = out.mv.epoch, .refSnapEpoch = out.mv.snapEpoch, .refLat = out.mv.refLat, .refFwd = out.mv.refFwd,
        .rateLat = out.mv.rateLat, .rateFwd = out.mv.rateFwd, .ffRoll = out.mv.ffRollDeg, .ffPitch = out.mv.ffPitchDeg,
        /* camera_flow.c (docs/autonomy-a2b.md 19.5): no forward hold in TAKEOFF */
        .fwdOff = !sc->oldTakeoffHold && out.state == AUTO_TAKEOFF,
      };
      {
        /* camera_flow.c (docs/autonomy-a2b.md 20): the gain scale from the recorder's autoState, mvState and the
         * observer's G (f / its D) with its airborne and tracked bits (tracked from 1.3 s after lift-off, as logged) */
        const float gT = sc->gTrans > 0.0f ? 1.0f + sc->gTrans * (hoverT < 0.0f ? 1.0f : expf(-(t - hoverT) / 0.4f))
                                           : 1.0f;
        const float dObs = D * (1.0f + w.eD[viewIdx(w.psi)]) / gT;
        const float gDpm = dObs > 0.0f ? 10.0f * AUTO_MV_FY_PX / dObs : 65535.0f;
        /* the observer tracks G only within its own 15..500 px/m (cam_vz_obs.c gMin, gMax) */
        const bool gIn = gDpm >= 150.0f && gDpm <= 5000.0f;
        const uint8_t cvFlags = (uint8_t)((sim.onGround ? 0u : 1u) | (w.tracked && gIn && w.airT >= 1.3f ? 4u : 0u));
        const uint8_t mvSt = (uint8_t)((uint8_t)out.mv.state | ((uint8_t)out.mv.kind << 4));
        float tgt = camHoldGainTarget(&gainLatch, (uint8_t)out.state, mvSt, cvFlags,
                                      (uint16_t)fminf(gDpm + 0.5f, 65535.0f), simKmax);
        if (!sc->noGainMove) {   /* camera_flow.c: during a MOVE the scene distance it takes the drone to (21.3) */
          /* camera_flow.c: the move fixes' GAIN (the measured progress; above 100 % a grace from the move's start) */
          const bool fixGain = (simFix & AUTO_MVFIX_GAIN) != 0u;
          camHoldGainMoveFix(&gainLatch, fixGain, fixGain && simPct > 100 ? CAM_HOLD_GAIN_GRACE_S : 0.0f,
                             w.pendDt[0]);
          tgt = camHoldGainMove(&gainLatch, tgt, gainS, (uint8_t)out.state, mvSt, out.mv.dLowM, out.mv.refFwd,
                                hold.epoch == out.mv.epoch ? hold.pFwd : 0.0f);
        }
        csvFlags = cvFlags;
        csvGain = (uint16_t)fminf(gDpm + 0.5f, 65535.0f);
        gainS = camHoldGainSlew(gainS, tgt, w.pendDt[0], camHoldGainRate(&gainLatch, (uint8_t)out.state));
        hin.gainScale = gainS;
      }
      /* camera_flow.c (mvspeed): the move's lean limit and slew, 0 at 100 */
      camHoldMoveLimits(simPct, &hin.moveLimDeg, &hin.moveSlewDegS);
      hin.ffBrakeScale = (simFix & AUTO_MVFIX_BRAKE) != 0u;   /* camera_flow.c: the move fixes' BRAKE */
      holdEng = camHoldStep(&hold, &hin);
      holdMovingPub = camHoldMoving(&hold, w.pendDt[0]);   /* camera_flow.c: the move fixes' STILL */
      holdSlew = camHoldSlewOf(&hold, &hin);   /* controller_pid.c's apply slew (cameraFlowHoldSlew) */
      holdCoast = hold.coasting;
      holdR = hold.roll;
      holdP = hold.pitch;
      w.lastHoldT = t;
      for (int i = 1; i < w.pendN; i++) {
        w.pendT[i - 1] = w.pendT[i];
        w.pendW[i - 1][0] = w.pendW[i][0];
        w.pendW[i - 1][1] = w.pendW[i][1];
        w.pendDt[i - 1] = w.pendDt[i];
      }
      w.pendN--;
    }
    const bool stall = sc->camStallAtS > 0.0f && sc->oneMove && cmdIdx == 0 && res->accT > 0.0f &&
                       t - res->accT >= sc->camStallAtS && t - res->accT < sc->camStallAtS + sc->camStallS;
    if ((sc->camLossAtMoveS >= 0.0f && cmdIdx == 0 && cmdEndT < 0.0f && t - phaseT >= sc->camLossAtMoveS &&
         t - phaseT < sc->camLossAtMoveS + 3.0f) || lateCamOut || stall) {
      w.camLost = true;
    } else {
      w.camLost = false;
    }
    if (stall && stallT0 < 0.0f) {
      stallT0 = t;
    } else if (!stall) {
      stallT0 = -1.0f;
    }

    /* ---- the observer's scale tracker ---- */
    const float dpsi = w.psi - w.psiPrev;
    const float yawRateDps = dpsi / D2R / dt;
    w.yawAcc += dpsi;
    if (fabsf(w.yawAcc) > 0.35f) {
      w.yawAcc = 0.0f;
      w.tracked = false;
      w.resets++;
      w.bobS = 0.0f;
      if (sc->gLatch == 1 && w.gD > 0.0f) {   /* the firmware keeps the scaled G as the guess (cam_vz_obs.c) */
        w.gD /= expf(fminf(fmaxf(out.fwdLn - w.gLnAt, -CAM_VZ_FWD_LN_MAX), CAM_VZ_FWD_LN_MAX));
        w.gLnAt = out.fwdLn;
      }
    }
    if (sc->gLatch && vzActive) {
      /* vertical excitation as the tracker needs it (cam_vz_obs.h minWRms 0.03 m/s of its high-passed speed, minN 12
       * frames): >= 0.03 m/s rms for 2 s; then it takes G at the distance now */
      w.gVLp += dt * (sim.v - w.gVLp);
      const float hp = sim.v - w.gVLp;
      w.gMs += dt * (hp * hp - w.gMs);
      w.gExcS = w.gMs >= 0.03f * 0.03f ? w.gExcS + dt : 0.0f;
      if ((w.gExcS >= 2.0f && !(sc->gNoRetake && out.state == AUTO_HOVER)) || w.gD <= 0.0f) {
        w.gD = D;
        w.gLnAt = out.fwdLn;
      }
    }
    if (!w.tracked && fabsf(out.mv.vRef) > 0.02f) {
      w.bobS += dt;
      if (w.bobS >= 2.0f) {
        w.tracked = true;
      }
    }
    w.psiPrev = w.psi;

    /* ---- the glue: inputs (decimated as autonomy.c) ---- */
    const bool slow = step % AUTONOMY_SLOW_EVERY == 0 || out.state == AUTO_IDLE;
    in.hbAgeMs = step % 100;
    if (lateOn && sc->lateLand == 4) {
      in.hbAgeMs = 100u + (uint32_t)((t - lateT0) * 1000.0f);   /* the Mac's heartbeat stopped */
    }
    in.appAgeMs = step % 50;
    in.vbat = vbat;
    in.rollDeg = w.roll;
    in.pitchDeg = w.pitch;
    in.tiltDeg = sqrtf(w.roll * w.roll + w.pitch * w.pitch);
    in.attErrDeg = 0.3f;
    in.gyroDps[0] = in.gyroDps[1] = 0.5f;
    in.gyroDps[2] = yawRateDps;
    in.accNormG = sim.onGround && sim.jolt < 0.01f ? 1.0f : fabsf(sim.accZ);
    if (handTiltNow > 0.0f) {
      /* the hand tilts the drone against its level setpoint while it touches it (handTilt over 0.3 s of contact; the
       * attitude loop levels it at 100 deg/s once free): tilt, roll, the attitude error the FSM's ATT_ERR trip
       * reads, and the controller's gravZ (below) */
      in.rollDeg = w.roll + handTiltNow;
      in.tiltDeg = fmaxf(in.tiltDeg, handTiltNow);
      in.attErrDeg = fmaxf(0.3f, handTiltNow);
    }
    if (res->handContact && sc->handKind == 2 && t - res->contactT >= 0.6f &&
        (sc->grabRollDps > 0.0f || sc->grabYawDps > 0.0f)) {
      in.gyroDps[0] = sc->grabRollDps;   /* the grab turns the drone in the hand */
      in.gyroDps[2] = sc->grabYawDps;
      res->rotated = true;
    }
    /* the in-air contact (LAND COLLISION scenarios): |acc| 4 g and gx 300 dps for 4 ms, the train's later
     * ones 4 g for 3 ms; the estimate's error enters the controller and the observer below */
    if (sc->contactInBob && bobFirstT < 0.0f && (out.mv.flags & AUTO_MVF_BOB)) {
      bobFirstT = t;
    }
    const float inMoveS = sc->contactInMoveS > 0.0f ? sc->contactInMoveS : 1.5f;
    if (sc->contact && kc < 0 && out.state == AUTO_HOVER &&
        (sc->contactInBob ? (bobFirstT >= 0.0f && t - bobFirstT >= sc->bobAtS)
                          : sc->contactInMove ? (cmdIdx == 0 && t - phaseT >= inMoveS)
                                              : (phase >= 3 && t - phaseT >= sc->contactAtS))) {
      kc = step;
      res->contactDone = true;
      res->hAtContact = sim.h;
      res->hMaxAfterContact = sim.h;
      contactX = w.x;
      contactY = w.y;
    }
    float contactAcc = 0.0f;
    if (kc >= 0) {
      static const int trainMs[5] = {0, 250, 370, 660, 830};
      const int ks = step - kc;
      for (int i = 0; i < (sc->contactTrain ? 5 : 1); i++) {
        if (ks >= trainMs[i] && ks < trainMs[i] + (i == 0 ? 4 : 3)) {
          in.accNormG = 4.0f;
          in.gyroDps[0] = i == 0 ? 300.0f : 0.5f;
          contactAcc = 3.0f;
        }
      }
      if (ks < 10) {
        contactAcc += sc->contactE / (G * 0.010f);   /* integrates to the error */
      }
      if (ks == 0) {
        camOff0 = pc.camVzOffset;
      }
      pc.camVzOffset = sc->contactEPersistS > 0.0f && (float)ks * dt >= sc->contactEPersistS ? camOff0 : sc->contactE;
      res->hMaxAfterContact = fmaxf(res->hMaxAfterContact, sim.h);
    }
    if (wallPulse > 0) {   /* the obstacle's hit: the accelerometer's pulse (the state machine's CONTACT detector) */
      in.accNormG = fmaxf(in.accNormG, wallPulseG);
      in.gyroDps[0] = 300.0f;
      wallPulse--;
    }
    if (sc->hardN > 0 && res->cued) {
      const float tq = t - res->cueT;
      for (int i = 0; i < sc->hardN; i++) {
        if (tq >= sc->hardT[i] && tq < sc->hardT[i] + 0.001f * (float)sc->hardMs[i] - 1e-6f) {
          in.accNormG = fmaxf(in.accNormG, 1.0f + sc->hardG[i]);
          contactAcc += sc->hardG[i];
          if (!hardKicked[i]) {
            hardKicked[i] = true;
            if (!sim.onGround) {
              sim.v += sc->hardDv[i];
            }
          }
        }
      }
    }
    if (sc->dipAtS > 0.0f && t >= sc->dipAtS && t < sc->dipAtS + 0.0015f) {
      in.vbatFastMin = 2.50f;   /* a brown-out-level dip the 0.5 s-filtered Vf does not see */
    } else if (slow) {
      in.vbatFastMin = sim.vbat;
      in.onGround = sim.onGround && applied == 0.0f;
      in.onGroundMs = in.onGround ? 30000u : 0u;
      in.camFrameAgeMs = w.lastFrameT > 0.0f && !w.camLost ? (uint32_t)((t - w.lastFrameT + 0.12f) * 1000.0f)
                                                         : (w.camLost ? 5000u : 100u);
      if (stallT0 >= 0.0f) {   /* frames stopped: the latest one's age grows (camera_flow.c) */
        in.camFrameAgeMs = (uint32_t)((t - stallT0 + 0.12f) * 1000.0f);
      }
      in.holdApplied = holdEng && w.lastHoldT >= 0.0f && t - w.lastHoldT < 0.3f;
      in.holdCoasting = holdCoast;
      in.holdProgLat = hold.pLat;
      in.holdProgFwd = hold.pFwd;
      in.holdEpochSeen = hold.epoch;
      in.holdMoving = holdMovingPub;
      in.sceneDM = D * (1.0f + w.eD[viewIdx(w.psi)]) /
                   (sc->gTrans > 0.0f ? 1.0f + sc->gTrans * (hoverT < 0.0f ? 1.0f : expf(-(t - hoverT) / 0.4f)) : 1.0f);
      in.sceneTracked = w.tracked;
      in.sceneResets = w.resets;
    }
    in.simPlant = false;
    in.vz = vo;
    memset(&in.cmd, 0, sizeof(in.cmd));

    /* ---- the late LAND (scen_t lateLand): its trigger lateAtS into the wait, in HOVER after the contact ---- */
    in.sessionChanged = false;
    if (sc->lateLand && !lateOn && (kc >= 0 || !sc->contact) && phase >= 3 && out.state == AUTO_HOVER &&
        t - phaseT >= sc->lateAtS) {
      lateOn = true;
      lateT0 = t;
      switch (sc->lateLand) {
        case 1:
          in.cmd = cmdOf(AUTO_CMD_LAND);
          landSent = true;
          break;
        case 2:
          pc.vRest = AUTO_VBAT_LOW_LAND_V + 0.40f;   /* loaded ~0.43 V lower: under the BATT_LOW floor */
          break;
        case 3:
          in.vbatFastMin = 2.50f;
          break;
        case 5:
        case 6:
          lateCamOut = true;
          break;
        case 7:
          in.appInputCount++;
          break;
        case 9:
          in.sessionChanged = true;
          break;
        case 10:
          vz.h += 2.0f;   /* white-box: the estimate out of its range (EST_RANGE, the runaway guard) */
          break;
        default:
          break;
      }
    }
    if (lateOn && sc->lateLand == 8) {
      const int ms = (int)lroundf((t - lateT0) * 1000.0f);
      in.loopGapUs = (ms == 0 || ms == 100 || ms == 200) ? 25000u : 1000u;
    }
    if (lateOn && sc->lateLand == 6 && lateCamOut && t - lateT0 >= sc->lateCamS) {
      lateCamOut = false;   /* the camera back (during the SINK's descent) */
    }
    if (lateOn && sc->lateLand == 6 && resumeT < 0.0f && res->resumes > 0) {
      resumeT = t;
    }
    if (lateOn && sc->lateLand == 6 && resumeT >= 0.0f && !landSent && out.state == AUTO_HOVER && t - resumeT >= 5.0f) {
      in.cmd = cmdOf(AUTO_CMD_LAND);   /* the Mac's LAND after the RESUME (it saw the photo) */
      landSent = true;
    }

    /* ---- the Mac: the script ---- */
    if (out.state == AUTO_IDLE && phase == 0 && step == 20) {
      in.cmd.type = AUTO_CMD_ARM;
      in.cmd.mode = AUTO_MODE_FLIGHT;
      phase = 1;
    } else if (phase == 1 && out.state == AUTO_ARMED && step > 100) {
      in.cmd = cmdOf(AUTO_CMD_TAKEOFF);
      in.cmd.token = TOKEN;
      in.cmd.heightCm = sc->toCm ? sc->toCm : 100;
      in.cmd.photoN = 1;
      in.cmd.photoGapMs = 300;
      in.cmd.actWaitMs = sc->waitMs > 0 ? sc->waitMs : 60000;
      if (planned) {
        in.cmd.moveRightCm = sc->planR;   /* the planned move (auto_fsm.h): flown from the wait's start */
        in.cmd.moveFwdCm = sc->planF;
      }
      phase = 2;
    } else if (phase == 2 && out.state == AUTO_HOVER && out.waiting) {
      phase = 3;
      phaseT = t;
      if (planned) {   /* the FSM started (or refused) the plan on this wait's first step: track it as oneMove */
        res->planSeqAtWait = f.mv.seq;
        res->oneWhy = f.mv.seq == 1 ? 0 : (int)f.mv.why;
        cmdIdx = 0;
        cmdEndT = res->oneWhy != 0 ? t : -1.0f;
        cmdX = w.x;
        cmdY = w.y;
        cmdPsi = w.psi;
        res->d0Obs = in.sceneDM;
        res->accT = t;
        hAcc = sim.h;
        {
          const float kis = CAM_HOLD_KI * (hold.gainScale > 1.0f ? hold.gainScale : 1.0f);
          res->eqR0 = -D * (hold.iLat - w.trimR / kis);
          res->eqF0 = -D * (hold.iFwd + w.trimP / kis);
        }
      }
    } else if (phase >= 3 && out.state == AUTO_HOVER && !landSent) {
      static const struct { autoCmdType_t t; int16_t r, f, u, yaw; const char *name; } script[] = {
        {AUTO_CMD_MOVE, 50, 0, 0, 0, "right 50"}, {AUTO_CMD_MOVE, 0, -50, 0, 0, "back 50"},
        {AUTO_CMD_YAW, 0, 0, 0, 90, "yaw 90"}, {AUTO_CMD_MOVE, 0, 0, 20, 0, "up 20"},
        {AUTO_CMD_MOVE, 0, 40, 0, 0, "forward 40"},
      };
      const int nScript = sc->mission ? 5 : 0;
      const bool motionActive = out.mv.state == AUTO_MV_RUN || out.mv.state == AUTO_MV_PAUSED ||
                                out.mv.state == AUTO_MV_SETTLE;
      if (cmdIdx >= 0 && !motionActive && cmdEndT < 0.0f) {
        cmdEndT = t;
      }
      const float gap = cmdIdx < 0 ? 10.0f : 2.0f;   /* 10 s for Claude to read the photo, then 2 s settles */
      if (sc->wallMode && wallT0 < 0.0f) {
        wallT0 = phaseT;
      }
      if (!preDone) {
        if (preIdx >= 0 && !motionActive && preEndT < 0.0f) {
          preEndT = t;
        }
        if (preIdx < 0 ? t - phaseT >= 1.0f : (preEndT >= 0.0f && t - preEndT >= 1.0f)) {
          if (preIdx < sc->preN - 1) {
            preIdx++;
            in.cmd = cmdOf(AUTO_CMD_MOVE);   /* up toward the hover above the take-off cap */
            in.cmd.moveUpCm = sc->preUps[preIdx];
            preEndT = -1.0f;
          } else {
            preDone = true;
            phaseT = t;   /* the scripts' wait clock from here */
            wallT0 = -1.0f;
          }
        }
      } else if (sc->wallMode == 1 && cmdIdx < 0 && t - wallT0 >= sc->wallMoveAtS) {
        in.cmd = cmdOf(AUTO_CMD_MOVE);   /* into the obstacle the camera does not see */
        in.cmd.moveFwdCm = sc->wallFwdCm;
        cmdIdx = 0;
        cmdEndT = -1.0f;
      } else if (sc->wallMode == 1 && sc->wallBackCm > 0 && cmdIdx == 0 && res->wallT >= 0.0f &&
                 t - res->wallT >= 5.0f) {
        in.cmd = cmdOf(AUTO_CMD_MOVE);   /* the runbook: back away from what was hit */
        in.cmd.moveFwdCm = (int16_t)-sc->wallBackCm;
        cmdIdx = 1;
      } else if (sc->wallMode && t - wallT0 >= sc->wallLandAtS) {
        in.cmd = cmdOf(AUTO_CMD_LAND);
        landSent = true;
      } else if (sc->wallMode) {
        /* Claude has not decided yet: no command */
      } else if (sc->oneMove && cmdIdx < 0 && t - phaseT >= (sc->moveAtS > 0.0f ? sc->moveAtS : 10.0f)) {
        in.cmd = cmdOf(AUTO_CMD_MOVE);
        in.cmd.moveRightCm = sc->oneR;
        in.cmd.moveFwdCm = sc->oneF;
        if (sc->fwdMax) {
          /* the largest F the drone takes: D ((D - F) / D)^(1/k_low) >= the move floor (0.30 m at the default margins,
           * 0.50 m before 2026-10-17), as auto_motion.c computes it */
          const float dd = in.sceneDM;
          autoMvMargins_t mg;
          autoMotionGetMargins(&mg);
          const float floorM = (float)mg.floorCm * 0.01f;
          int16_t fc = 150;
          while (fc > 0 && !(dd * expf(autoMvLn((dd - fc * 0.01f) / dd) * (1.0f / AUTO_MV_K_LOW)) >= floorM &&
                             dd - fc * 0.01f >= AUTO_MV_LAT_GUARD_D_MIN_M)) {
            fc--;
          }
          in.cmd.moveFwdCm = (int16_t)(fc + (sc->fwdMax - 1));
          res->oneF = in.cmd.moveFwdCm;
        } else if (sc->macCap) {
          const int16_t cap = macFwdMax(in.sceneDM);   /* what the Mac sends at the D it reads */
          in.cmd.moveFwdCm = sc->oneF < cap ? sc->oneF : cap;
          res->oneF = in.cmd.moveFwdCm;
        }
        cmdIdx = 0;
        phaseT = t;
        cmdEndT = -1.0f;
        cmdX = w.x;
        cmdY = w.y;
        cmdPsi = w.psi;
        res->vAtSend = -(w.vx * cosf(w.psi) + w.vy * sinf(w.psi));   /* (+: moving back) */
        res->d0Obs = in.sceneDM;
        res->accT = t;
        hAcc = sim.h;
        {
          const float kis = CAM_HOLD_KI * (hold.gainScale > 1.0f ? hold.gainScale : 1.0f);
          res->eqR0 = -D * (hold.iLat - w.trimR / kis);
          res->eqF0 = -D * (hold.iFwd + w.trimP / kis);
        }
        oneAcc = true;
      } else if (sc->oneMove && sc->stopAtS > 0.0f && cmdIdx == 0 && res->oneWhy == 0 && !res->evtSeen &&
                 res->accT > 0.0f && t - res->accT >= sc->stopAtS) {
        in.cmd = cmdOf(AUTO_CMD_STOP);   /* scen_t stopAtS */
        {
          float s0, v0, a0;
          autoMvProfEval(&f.mv.ph, f.mv.tRun, &s0, &v0, &a0);
          res->evtVRef = f.mv.state == AUTO_MV_RUN ? v0 : 0.0f;
        }
        res->evtSeen = true;
        res->evtX = w.x;
        res->evtY = w.y;
        res->evtVTrue = hypotf(w.vx, w.vy);
      } else if (sc->oneMove && cmdIdx == 0 && (cmdEndT >= 0.0f || res->oneWhy != 0) &&
                 (res->oneWhy != 0 || t - cmdEndT >= 2.0f)) {
        if (res->evtSeen) {
          res->evtTravel = hypotf(w.x - res->evtX, w.y - res->evtY);
        }
        const float dx = w.x - cmdX, dy = w.y - cmdY, c0 = cosf(cmdPsi), s0 = sinf(cmdPsi);
        res->mvR = dx * s0 - dy * c0;
        res->mvF = dx * c0 + dy * s0;
        res->wallAtLand = wallD(w.x, w.y, w.psi);
        in.cmd = cmdOf(AUTO_CMD_LAND);
        landSent = true;
      } else if (sc->hoverWinS > 0.0f && t - phaseT >= 3.0f + sc->hoverWinS + 2.0f) {
        in.cmd = cmdOf(AUTO_CMD_LAND);
        landSent = true;
      } else if (sc->nUp > 0 && cmdIdx < sc->nUp - 1 &&
                 (cmdIdx < 0 ? t - phaseT >= sc->firstAfterS : (cmdEndT >= 0.0f && t - cmdEndT >= 1.0f))) {
        cmdIdx++;
        in.cmd = cmdOf(AUTO_CMD_MOVE);
        in.cmd.moveUpCm = sc->ups[cmdIdx];
        cmdEndT = -1.0f;
      } else if (sc->nUp > 0 && cmdIdx == sc->nUp - 1 && cmdEndT >= 0.0f && t - cmdEndT >= sc->landAfterS) {
        in.cmd = cmdOf(AUTO_CMD_LAND);
        landSent = true;
      } else if (sc->contactInMove && cmdIdx < 0 && t - phaseT >= 10.0f) {
        in.cmd = cmdOf(AUTO_CMD_MOVE);
        if (sc->contactUpCm != 0) {
          in.cmd.moveUpCm = sc->contactUpCm;
        } else {
          in.cmd.moveFwdCm = 60;
        }
        cmdIdx = 0;
        phaseT = t;
      } else if (sc->turnDeg != 0 && cmdIdx < 0 && t - phaseT >= 3.0f) {
        in.cmd = cmdOf(AUTO_CMD_YAW);
        in.cmd.yawDeg = sc->turnDeg;
        cmdIdx = 0;
        phaseT = t;
        cmdEndT = -1.0f;
        cmdX = w.x;
        cmdY = w.y;
        cmdPsi = w.psi;
      } else if (sc->turnDeg != 0 && cmdIdx == 0 && cmdEndT >= 0.0f && t - cmdEndT >= 2.0f) {
        float d = (cmdPsi - w.psi) / D2R;
        while (d > 180.0f) d -= 360.0f;
        while (d < -180.0f) d += 360.0f;
        res->turnYawErr = d - (float)sc->turnDeg;
        in.cmd = cmdOf(AUTO_CMD_LAND);
        landSent = true;
      } else if ((sc->stopInBob || sc->contactInBob) && cmdIdx < 0 && t - phaseT >= 3.0f) {
        in.cmd = cmdOf(AUTO_CMD_YAW);
        in.cmd.yawDeg = 90;
        cmdIdx = 0;
      } else if (sc->stopInBob && cmdIdx == 0 && !bobStopSent && (out.mv.flags & AUTO_MVF_BOB) && w.bobS > 0.3f) {
        in.cmd = cmdOf(AUTO_CMD_STOP);
        bobStopSent = true;
        bobStopT = t;
        res->bobSeen = true;
      } else if (sc->stopInBob && bobStopSent && !motionActive && t - bobStopT >= 1.0f) {
        res->bobDone = out.mv.state == AUTO_MV_DONE && out.mv.end == AUTO_MVE_DONE;
        in.cmd = cmdOf(AUTO_CMD_LAND);
        landSent = true;
      } else if (sc->stopMid && cmdIdx < 0 && t - phaseT >= 10.0f) {
        in.cmd = cmdOf(AUTO_CMD_MOVE);
        in.cmd.moveRightCm = 100;
        cmdIdx = 0;
        phaseT = t;
        cmdEndT = -1.0f;
      } else if (sc->stopMid && cmdIdx == 0 && stopT < 0.0f && t - phaseT >= 2.5f) {
        in.cmd = cmdOf(AUTO_CMD_STOP);
        stopT = t;
        stopX = w.x;
        stopY = w.y;
        res->stopTravelAtStop = hypotf(w.x - sc->x0, w.y - sc->y0);
      } else if (sc->stopMid && stopT >= 0.0f && t - stopT >= 4.0f) {
        res->stopDrift = hypotf(w.x - stopX, w.y - stopY);
        in.cmd = cmdOf(AUTO_CMD_LAND);
        landSent = true;
      } else if (sc->camLossAtMoveS >= 0.0f && cmdIdx < 0 && t - phaseT >= 10.0f) {
        in.cmd = cmdOf(AUTO_CMD_MOVE);
        in.cmd.moveFwdCm = 60;
        cmdIdx = 0;
        phaseT = t;
        cmdEndT = -1.0f;
      } else if (sc->handDuring == 2 && t - phaseT >= sc->handAtS) {
        in.cmd = cmdOf(AUTO_CMD_LAND);   /* the Mac's LAND onto a palm held under the drone */
        landSent = true;
      } else if (sc->handKind != 0 && giveUpT >= 0.0f && t - giveUpT >= 3.0f) {
        in.cmd = cmdOf(AUTO_CMD_LAND);   /* no cue: the hand lowered away, the Mac lands */
        landSent = true;
      } else if (sc->drain && sc->drainDescend && vo.h > 1.25f &&
                 (sc->drainDescendT > 0.0f ? in.vz.tHold >= sc->drainDescendT
                                           : (in.vz.tHold >= 52000.0f || in.vbat <= 3.20f)) &&
                 !motionActive && t - lastStopCmdT > 1.0f) {
        in.cmd = cmdOf(AUTO_CMD_MOVE);   /* the runbook: down to <= 1.2 m near the end of the battery */
        in.cmd.moveUpCm = (int16_t)-lroundf(fminf(0.50f, vo.h - 1.20f) * 100.0f);
        lastStopCmdT = t;
        res->nDescend++;
      } else if (sc->drain && t - lastStopCmdT > 6.0f && !motionActive) {
        in.cmd = cmdOf(AUTO_CMD_MOVE);   /* the Mac tries a MOVE every 6 s (BATT refuses it near the end) */
        in.cmd.moveRightCm = (res->nMoveOk % 2) ? -20 : 20;
        lastStopCmdT = t;
      } else if (sc->airLimit && out.airMs >= MAC_AIR_LAND_MS) {
        in.cmd = cmdOf(AUTO_CMD_LAND);   /* the Mac's LAND at the old air clock's 120 s */
        landSent = true;
        res->macAirLand = true;
      } else if (nScript > 0 && (cmdIdx < 0 ? t - phaseT >= gap : (cmdEndT >= 0.0f && t - cmdEndT >= gap))) {
        if (cmdIdx >= 0 && cmdIdx < nScript) {
          /* the finished command's true displacement in its own body frame */
          const float dx = w.x - cmdX, dy = w.y - cmdY, c0 = cosf(cmdPsi), s0 = sinf(cmdPsi);
          res->gotNow[cmdIdx][0] = dx * s0 - dy * c0;
          res->gotNow[cmdIdx][1] = dx * c0 + dy * s0;
          res->gotNow[cmdIdx][2] = sim.h - cmdZ;
          res->got[cmdIdx][0] = res->gotNow[cmdIdx][0] - eqR;
          res->got[cmdIdx][1] = res->gotNow[cmdIdx][1] - eqF;
          res->got[cmdIdx][2] = res->gotNow[cmdIdx][2];
          if (script[cmdIdx].t == AUTO_CMD_YAW) {
            float d = (cmdPsi - w.psi) / D2R;
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            res->yawGot = d;
          }
        }
        cmdIdx++;
        if (cmdIdx < nScript) {
          in.cmd = cmdOf(script[cmdIdx].t);
          in.cmd.moveRightCm = script[cmdIdx].r;
          in.cmd.moveFwdCm = script[cmdIdx].f;
          in.cmd.moveUpCm = script[cmdIdx].u;
          in.cmd.yawDeg = script[cmdIdx].yaw;
          snprintf(res->name[cmdIdx], sizeof(res->name[cmdIdx]), "%s", script[cmdIdx].name);
          res->want[cmdIdx][0] = script[cmdIdx].r * 0.01f;
          res->want[cmdIdx][1] = script[cmdIdx].f * 0.01f;
          res->want[cmdIdx][2] = script[cmdIdx].u * 0.01f;
          if (script[cmdIdx].t == AUTO_CMD_YAW) {
            res->yawWant = script[cmdIdx].yaw;
          }
          res->ncmd = cmdIdx + 1;
          cmdX = w.x;
          cmdY = w.y;
          cmdZ = sim.h;
          cmdPsi = w.psi;
          cmdEndT = -1.0f;
          /* Where the hold was taking the drone: its integrals are the position error
           * from that point (x / D and ln(D0 / D)) plus the lean the trim error needs
           * (roll -KI iLat cancels trimR, pitch KI iFwd cancels trimP). */
          {
            /* (the gain scale: the integral's lean is KI s i; the hold settles where i = trim / (KI s). The clamp
             * and the scale's rescale bookkeeping in lost* do not move that point; lost* is 0 at kmax 1 here.) */
            const float kis = CAM_HOLD_KI * (hold.gainScale > 1.0f ? hold.gainScale : 1.0f);
            eqR = -D * (hold.iLat - w.trimR / kis);
            eqF = -D * (hold.iFwd + w.trimP / kis);
          }
        } else {
          in.cmd = cmdOf(cmdIdx == nScript ? AUTO_CMD_STOP : AUTO_CMD_LAND);
          cmdEndT = t;
          if (cmdIdx > nScript) {
            landSent = true;
          }
        }
      } else if (sc->shLoop || sc->shSingle != 0.0f || sc->shRep) {
        /* SET_HEIGHT's Claude (scen_t shLoop / shSingle / shRep); its LANDs the only ones */
        const float tw = t - phaseT;
        if (sc->shRep) {
          if (!shRepShifted && tw >= sc->shAtS && !motionActive && vo.phase == VZ_HOLD) {
            /* the logged geometry (white-box): the true height Claude judged, the estimate the photo had */
            sim.h = sc->shRepX;
            if (!vzShiftHeight(&vz, sc->shRepAt - vo.h)) {
              res->shShiftRefused++;
            }
            shRepShifted = true;
          } else if (shRepShifted && shRepPhotoT < 0.0f) {
            shRepPhotoT = t;   /* the photo, one step after the shift (the outputs carry it) */
            shX = sim.h;       /* a perfect judgement */
            shAt = vo.h;
            res->shRepH0 = sim.h;
          } else if (!shSent && shRepPhotoT >= 0.0f && t - shRepPhotoT >= sc->shRepLat && !motionActive) {
            in.cmd = cmdOf(AUTO_CMD_SET_HEIGHT);
            in.cmd.moveUpCm = (int16_t)lroundf(shAt * 100.0f);
            if (sc->shRep == 1) {
              in.cmd.heightCm = (uint16_t)lroundf(shX * 100.0f);
              in.cmd.mode = AUTO_SH_FORM_TO;
              in.cmd.moveRightCm = AUTO_SH_TO_REF;
            } else {
              in.cmd.heightCm = (uint16_t)lroundf((shX + sc->shRepSentOff) * 100.0f);
            }
            res->shRepHCmd = sim.h;
            shSent = true;
            shTarget = 1.00f;
          } else if (shSent && shEndT >= 0.0f && t - shEndT >= 2.0f && res->shRepH2 == 0.0f) {
            res->shRepH2 = sim.h;
          } else if (shSent && shEndT >= 0.0f && t - shEndT >= 8.0f) {
            in.cmd = cmdOf(AUTO_CMD_LAND);
            landSent = true;
          }
        } else if (sc->shSingle != 0.0f) {
          if (!shSent && tw >= sc->shAtS && !motionActive) {
            in.cmd = cmdOf(AUTO_CMD_SET_HEIGHT);
            in.cmd.heightCm = (uint16_t)lroundf((vo.h + sc->shSingle) * 100.0f);
            in.cmd.moveUpCm = AUTO_SH_AT_NOW;
            shSent = true;
            shTarget = sim.h - sc->shSingle;   /* it flies -d: where it physically ends */
          } else if (shSent && shEndT >= 0.0f && t - shEndT >= 8.0f) {
            in.cmd = cmdOf(AUTO_CMD_LAND);
            landSent = true;
          }
        } else if (out.airMs >= (uint32_t)(sc->shDurS * 1000.0f)) {
          res->shErrEnd = sim.h - shH0;
          in.cmd = cmdOf(AUTO_CMD_LAND);
          landSent = true;
        } else if (sc->shLoop == 1) {
          if (shNextPhotoT < 0.0f) {
            shNextPhotoT = phaseT + 1.0f;   /* the reference photo right after the take-off */
          }
          if (!shPend && t >= shNextPhotoT) {
            const float n = fminf(0.15f, fmaxf(-0.15f, sc->shNoise * gauss()));
            shX = sim.h + n + sc->shBias;   /* Claude's judgement off this photo */
            shAt = vo.h;                    /* the photo's estimate at capture (its JSON's h_est_capture_m) */
            if (fabsf(shX - shAt) > 0.10f) {
              shPend = true;
              shSendT = t + sc->shL;
            } else {
              shNextPhotoT = t + sc->shP;
            }
          }
          if (shPend && t >= shSendT && t >= shRetryT && !motionActive) {
            const float dd = fminf(SH_MAX_PART, fmaxf(-SH_MAX_PART, shX - shAt));   /* its 0.5 m part */
            shLastD = dd;
            in.cmd = cmdOf(AUTO_CMD_SET_HEIGHT);
            in.cmd.heightCm = (uint16_t)lroundf((shAt + dd) * 100.0f);
            in.cmd.moveUpCm = (int16_t)lroundf(shAt * 100.0f);
          }
        }
      } else if (!sc->airLimit && !sc->drain && out.waiting && in.nowMs - f.waitMs >= f.hoverWaitMs) {
        in.cmd = cmdOf(AUTO_CMD_LAND);   /* the Mac's LAND at the wait's end (HOVER_TIMEOUT until 2026-10-15) */
        landSent = true;
        res->macWaitLand = true;
      }
    }

    /* the Mac's LAND after a contact it saw (a photo): macLandAfterContactS after the hand's first touch or the in-air
     * contact */
    if (sc->macLandAfterContactS > 0.0f && !landSent && in.cmd.type == AUTO_CMD_NONE &&
        (out.state == AUTO_TAKEOFF || out.state == AUTO_HOVER)) {
      const float tc0 = res->handContact ? res->contactT : (kc >= 0 ? (float)kc * dt : -1.0f);
      if (tc0 >= 0.0f && t - tc0 >= sc->macLandAfterContactS) {
        in.cmd = cmdOf(AUTO_CMD_LAND);
        landSent = true;
      }
    }

    /* ---- the state machine ---- */
    const uint16_t evN0 = autoFsmReport(&f)->evCount;
    autoFsmStep(&f, &in, &out);
    if (!speedSet && out.state == AUTO_ARMED) {
      /* autonomy.c: the ARM step's sortieSpeedLatch (after the ARM's autoMotionInit): mvspeed and the margins */
      speedSet = true;
      simMargins(sc->margins);
      if (sc->fixSet) {
        autoMvMargins_t g;
        autoMotionGetMargins(&g);
        g.fix = sc->fixBits;
        autoMotionSetMargins(&g);
      }
      {
        autoMvMargins_t g;
        autoMotionGetMargins(&g);
        simFix = g.fix;   /* (autonomy.c sortieMvFix) */
      }
      if (sc->mvSpeed > 100) {
        autoFsmSetMvSpeed(&f, sc->mvSpeed);
        simPct = f.mv.speedPct;
      }
    }
    {
      const autoFsmReport_t *rep = autoFsmReport(&f);
      const unsigned nNew = (uint16_t)(rep->evCount - evN0);
      for (unsigned j = 0; j < nNew && j < 3u; j++) {
        res->evMask |= 1u << (rep->evCode[j] & 31u);
        res->contacts += rep->evCode[j] == AUTO_EV_CONTACT;
        if (sc->wallMode && rep->evCode[j] == AUTO_EV_CONTACT && res->wallT < 0.0f && out.state == AUTO_HOVER &&
            (sc->wallMode == 2 || (rep->evDetail[j] & AUTO_CONTACT_MOTION))) {
          res->wallT = t;   /* the obstacle's first CONTACT in HOVER (wallMode 1: the one that stopped the MOVE) */
        }
        res->sinks += rep->evCode[j] == AUTO_EV_SINK;
        res->resumes += rep->evCode[j] == AUTO_EV_RESUME;
      }
      res->hushSeen = res->hushSeen || out.hush;
    }
    if (out.state == AUTO_HOVER && hoverT < 0.0f) {
      hoverT = t;
      if (sc->planAtHover && (sc->planR != 0 || sc->planF != 0)) {
        /* white-box: the move started at the HOVER entry (the rejected alternative), with the inputs mvIn() gives */
        autoMvIn_t mi;
        memset(&mi, 0, sizeof(mi));
        mi.nowMs = in.nowMs;
        mi.flight = true;
        mi.sceneDM = in.sceneDM;
        mi.sceneTracked = in.sceneTracked;
        mi.sceneResets = in.sceneResets;
        mi.holdApplied = in.holdApplied;
        mi.holdCoasting = in.holdCoasting;
        mi.camFrameAgeMs = in.camFrameAgeMs;
        mi.tiltDeg = in.tiltDeg;
        mi.camHeight = in.vz.camHealthy;
        mi.vzH = in.vz.h;
        mi.hTargetM = (float)f.heightCm * 0.01f;
        mi.progLat = in.holdProgLat;
        mi.progFwd = in.holdProgFwd;
        mi.holdEpochSeen = in.holdEpochSeen;
        mi.airMs = out.airMs;
        /* (autoMotionMove, not the plan's 0.60 m floor: the comparison is the G transient's, as measured before
         * 21.9; with the plan's floor the transient's low D at the HOVER entry refuses some of the 2.5 m moves) */
        res->oneWhy = (int)autoMotionMove(&f.mv, &mi, sc->planR, sc->planF, 0);
        cmdIdx = 0;
        cmdEndT = res->oneWhy != 0 ? t : -1.0f;
        cmdX = w.x;
        cmdY = w.y;
        cmdPsi = w.psi;
        res->d0Obs = in.sceneDM;
        res->accT = t;
        hAcc = sim.h;
        {
          const float kis = CAM_HOLD_KI * (hold.gainScale > 1.0f ? hold.gainScale : 1.0f);
          res->eqR0 = -D * (hold.iLat - w.trimR / kis);
          res->eqF0 = -D * (hold.iFwd + w.trimP / kis);
        }
      }
    }
    if (sc->oneMove && oneAcc && in.cmd.type == AUTO_CMD_MOVE) {
      res->oneWhy = out.cmdResult == AUTO_R_OK ? 0 : (int)((out.cmdDetail >> 8) & 0xFF);
      if (res->oneWhy != 0) {
        cmdEndT = t;
      } else {
        res->profT = fmaxf(f.mv.ph.T, f.mv.pv.T);
        res->oneVpk = f.mv.ph.vPk;
      }
      oneAcc = false;
    }
    if (sc->oneMove && cmdIdx == 0 && res->oneWhy == 0 && res->oneEnd == AUTO_MVE_NONE &&
        (out.mv.state == AUTO_MV_RUN || out.mv.state == AUTO_MV_PAUSED)) {
      /* fast moves: the time it ran, the true image rates and the lead over the reference */
      res->oneRunS += dt;
      res->stillS += f.mv.stillWait ? dt : 0.0f;   /* (mvfix STILL: the profile waits at its start) */
      const float cps = cosf(w.psi), sps = sinf(w.psi);
      const float vFt = w.vx * cps + w.vy * sps, vRt = w.vx * sps - w.vy * cps, Dt = wallD(w.x, w.y, w.psi);
      const float wF = w.k * vFt / Dt, wL = fabsf(vRt) / Dt;
      res->wFwdTrueMax = fmaxf(res->wFwdTrueMax, wF);
      res->wLatTrueMax = fmaxf(res->wLatTrueMax, wL);
      res->vTrueMax = fmaxf(res->vTrueMax, hypotf(w.vx, w.vy));
      {
        autoMvSpeed_t spc;
        autoMvSpeedCfg(f.mv.speedPct, &spc);
        if (out.mv.rateFwd > 0.5f * spc.wFwd) {   /* (the cruise and its ends, not the first tenths of the ramp) */
          res->leadMax = fmaxf(res->leadMax, wF / out.mv.rateFwd);
        }
      }
    }
    if (sc->oneMove && sc->camStallAtS > 0.0f && stallT0 == t && !res->evtSeen) {
      float s0, v0, a0;
      autoMvProfEval(&f.mv.ph, f.mv.tRun, &s0, &v0, &a0);
      res->evtVRef = v0;
      res->evtSeen = true;
      res->evtX = w.x;
      res->evtY = w.y;
      res->evtVTrue = hypotf(w.vx, w.vy);
    }
    if (sc->oneMove && cmdIdx == 0 && res->oneEnd == AUTO_MVE_NONE && res->oneWhy == 0 &&
        (out.mv.state == AUTO_MV_DONE || out.mv.state == AUTO_MV_STOPPED)) {
      res->oneEnd = out.mv.end;
      res->oneEndS = t - res->accT;
      res->endPLat = f.mv.progLat;
      res->endPFwd = f.mv.progFwd;
      res->endDLat0 = f.mv.dLat0;
      res->endD0 = f.mv.d0;
      res->endLatM = f.mv.latM;
      res->endFwdM = f.mv.fwdM;
      {
        const float dx = w.x - cmdX, dy = w.y - cmdY, c0 = cosf(cmdPsi), s0 = sinf(cmdPsi);
        res->endTrueR = dx * s0 - dy * c0;
        res->endTrueF = dx * c0 + dy * s0;
      }
      res->endWall = wallD(w.x, w.y, w.psi);
      res->endVTrue = hypotf(w.vx, w.vy);
    }
    if (sc->oneMove && cmdIdx == 0 && res->oneWhy == 0 && res->oneEnd == AUTO_MVE_NONE && vzActive) {
      res->hExcMove = fmaxf(res->hExcMove, fabsf(sim.h - hAcc));
      res->hErrMove = fmaxf(res->hErrMove, fabsf(vo.h - (out.hTargetM + out.mv.hOff)));
      if (out.mv.refFwd > AUTO_MV_FROZEN_REF && in.holdEpochSeen == out.mv.epoch) {
        res->trackMin = fminf(res->trackMin, in.holdProgFwd / out.mv.refFwd);   /* the frozen-view rule's ratio */
      }
    }
    if (sc->hoverWinS > 0.0f && phase >= 3 && out.state == AUTO_HOVER) {
      const float tw = t - phaseT;
      if (tw >= 3.0f && !hoverWinOn) {
        hAcc = sim.h;
        hoverWinOn = true;
      } else if (hoverWinOn && tw <= 3.0f + sc->hoverWinS) {
        res->hExcHover = fmaxf(res->hExcHover, fabsf(sim.h - hAcc));
      }
    }
    if (csvF && step % 10 == 0) {
      fprintf(csvF, "1,%u,%d,%d,%u,%u,%d,%d\n", (unsigned)in.nowMs, (int)out.state,
              (int)((uint8_t)out.mv.state | ((uint8_t)out.mv.kind << 4)), (unsigned)csvFlags, (unsigned)csvGain,
              (int)lroundf(holdR * 100.0f), (int)lroundf(holdP * 100.0f));
    }
    if (sc->turnDeg != 0 && cmdIdx == 0 && out.mv.kind == AUTO_MVK_YAW && !res->turnEnded) {
      if (out.mv.state == AUTO_MV_PAUSED) {
        res->turnPaused = true;
      }
      if (out.mv.state == AUTO_MV_SETTLE && res->turnS == 0.0f) {
        res->turnS = t - phaseT;
      }
      if (res->turnS == 0.0f) {
        res->turnRateMax = fmaxf(res->turnRateMax, fabsf(yawRateDps));
      }
      res->turnDrift = fmaxf(res->turnDrift, hypotf(w.x - cmdX, w.y - cmdY));
      if (out.mv.state == AUTO_MV_DONE || out.mv.state == AUTO_MV_STOPPED) {
        res->turnEnded = true;
        res->turnEnd = out.mv.end;
      }
    }
    /* SET_HEIGHT's answers and the correction's measurements */
    if (in.cmd.type == AUTO_CMD_SET_HEIGHT && out.cmdDone) {
      if (out.cmdResult == AUTO_R_OK) {
        res->shAcc++;
        shPend = false;
        shNextPhotoT = t + sc->shP;
        shAccT = t;
        shCorr = true;
        shThrRef = shThrLp;
      } else {
        const unsigned why = (out.cmdDetail >> 8) & 0xFFu;
        res->shRef[why & 15u]++;
        res->shRetries++;
        shRetryT = t + (why == AUTO_MVW_SH_RATE ? 2.0f : why == AUTO_MVW_BUSY ? 0.0f : 1.0f);
        if (sc->shSingle != 0.0f || sc->shRep) {
          shEndT = t;
        }
      }
      if (sc->shRep) {
        res->shRepResult = (int)out.cmdResult;
      }
    }
    if (shCorr && out.mv.state != AUTO_MV_RUN && out.mv.state != AUTO_MV_PAUSED && out.mv.state != AUTO_MV_SETTLE) {
      shCorr = false;
      res->shProgress += out.mv.end == AUTO_MVE_PROGRESS;
      if (fabsf(shLastD) >= 0.30f - 1e-4f) {
        shNextPhotoT = fminf(shNextPhotoT, t + 1.0f);   /* the runbook: a photo to confirm a correction of >= 0.3 m */
      }
      if (sc->shSingle != 0.0f || sc->shRep) {
        res->shSingleEnd = out.mv.end;
        res->shSingleS = t - shAccT;
        res->shSingleErr = sim.h - shTarget;
        shEndT = t;
      }
    }
    if ((sc->shLoop || sc->shSingle != 0.0f || sc->shRep) && phase >= 3 && preDone && out.state == AUTO_HOVER &&
        vzActive) {
      res->shHMax = fmaxf(res->shHMax, sim.h);
      if (shH0 < 0.0f) {
        shH0 = vo.h;
      }
      if (t - phaseT >= 10.0f && t >= shNextErrT && res->shErrN < 2000) {
        res->shErr[res->shErrN++] = sim.h - shH0;
        shNextErrT = t + 0.1f;
      }
      /* the thrust's 0.1 s mean against its 2 s mean (frozen at a correction's start): the rotor-band noise left out */
      if (shThrLp == 0.0f) {
        shThrLp = shThrFast = applied;   /* the first sample of the wait */
      }
      shThrLp += dt / 2.0f * (applied - shThrLp);
      shThrFast += dt / 0.1f * (applied - shThrFast);
      if (t - phaseT >= 5.0f && shThrLp > 0.0f) {
        if (shCorr) {
          res->shThrDevCorr = fmaxf(res->shThrDevCorr, fabsf(shThrFast / shThrRef - 1.0f));
        } else {
          res->shThrDevHover = fmaxf(res->shThrDevHover, fabsf(shThrFast / shThrLp - 1.0f));
        }
      }
      if (res->shErrN > 0) {
        res->shErrEnd = res->shErr[res->shErrN - 1];   /* (the Mac's LAND at shDurS sets it at the LAND) */
      }
    }
    if (sc->stopInBob && in.cmd.type == AUTO_CMD_STOP) {
      res->stopAcked = out.cmdResult == AUTO_R_OK;
    }
    if (sc->stopInBob && bobStopT >= 0.0f && t - bobStopT < 1.0f && out.state != AUTO_HOVER) {
      res->landedSoon = true;
    }
    if (sc->drain && in.cmd.type == AUTO_CMD_MOVE && out.cmdDone) {
      if (out.cmdResult == AUTO_R_OK) {
        res->nMoveOk++;
        res->maxMoveTHold = fmaxf(res->maxMoveTHold, in.vz.tHold);
      } else if (out.cmdResult == AUTO_R_LOW_BATT) {
        res->nLowBatt++;
      }
    }
    if ((in.cmd.type == AUTO_CMD_MOVE || in.cmd.type == AUTO_CMD_YAW) && !sc->drain) {
      if (out.cmdResult != AUTO_R_OK && !(sc->oneMove || sc->nUp > 0)) {
        res->refused++;
        printf("  seed %u: %s refused %d (why %s)\n", (unsigned)sc->seed, in.cmd.type == AUTO_CMD_MOVE ? "MOVE" : "YAW",
               out.cmdResult, autoMvWhyName((autoMvWhy_t)((out.cmdDetail >> 8) & 0xFF)));
      }
    }
    res->pausedSeen = res->pausedSeen || out.mv.state == AUTO_MV_PAUSED;
    if (out.vzReset) {
      vzConfig_t c = vc;
      vzConfigSetHeight(&c, out.hTargetM);
      vzInit(&vz, &c);
      memset(&vo, 0, sizeof(vo));
      vzActive = true;
    }
    if (out.captureStart) {
      in.capture = AUTO_CAP_DONE;   /* the set completes at once */
    }
    if (sc->nUp > 0 && (in.cmd.type == AUTO_CMD_MOVE) && out.cmdResult != AUTO_R_OK) {
      res->refused++;
    }
    if (inLand && out.state == AUTO_HOVER) {
      inLand = false;   /* a SINK resumed (RESUME): the next LAND is recorded as the sortie's LAND */
      contactSeen = false;
    }
    if ((out.state == AUTO_LAND || out.state == AUTO_KILLED) && !inLand) {
      inLand = true;
      res->collidedAtLand = out.collided;
      res->mvStoppedAtLand = out.mv.state != AUTO_MV_RUN && out.mv.state != AUTO_MV_PAUSED &&
                             out.mv.state != AUTO_MV_SETTLE && out.mv.rateLat == 0.0f && out.mv.rateFwd == 0.0f &&
                             out.mv.ffRollDeg == 0.0f && out.mv.ffPitchDeg == 0.0f;
      res->landWhy = out.reason;
      res->vzLandWhy = (uint8_t)out.vzLandReason;
      res->landAirMs = out.airMs;
      res->landT = t;
      res->landTHold = in.vz.tHold;   /* what the FSM read */
      res->landV = sim.vbat;
      res->landVRest = pc.vRest;
      res->hAtLand = sim.h;
      res->kRelAtLand = pc.kTrue / vc.kPrior;
      res->stateAtLand = prevState;
      landX = w.x;
      landY = w.y;
      landPsi = w.psi;
    }
    if (sc->nUp > 0 && cmdIdx >= 0 && out.state == AUTO_HOVER) {
      res->hMinHover = fminf(res->hMinHover, sim.h);
      res->groundInHover = res->groundInHover || sim.onGround;
    }
    if (sc->oneMove && cmdIdx >= 0 && !inLand) {
      res->minWall = fminf(res->minWall, wallD(w.x, w.y, w.psi));
      if (!res->wallTouch && wallD(w.x, w.y, w.psi) < 0.05f) {
        res->wallTouch = true;
        res->wallTouchV = hypotf(w.vx, w.vy);
      }
    }
    if (out.state == AUTO_KILLED) {
      res->killed = true;
    }
    if (inLand && out.collided && !res->collidedAtLand) {
      res->collidedInLand = true;
    }
    if (res->handContact && !inLand) {
      res->hMaxAfterHand = fmaxf(res->hMaxAfterHand, sim.h);
    }
    if (kc >= 0 && !inLand) {
      res->hMaxAfterContact2 = fmaxf(res->hMaxAfterContact2, sim.h);
    }
    if (out.state == AUTO_HOVER) {
      res->hMaxHover = fmaxf(res->hMaxHover, sim.h);
      res->groundInWait = res->groundInWait || (out.waiting && sim.onGround);
    }
    if (out.state == AUTO_HOVER && res->kRelHover == 0.0f) {
      res->kRelHover = pc.kTrue / vc.kPrior;
    }
    if (out.state == AUTO_IDLE && prevState != AUTO_IDLE && prevState != AUTO_ARMED) {
      res->endWhy = out.reason;
      res->endVzWhy = (uint8_t)out.vzLandReason;
      if (kc >= 0) {
        res->contactLandMs = (uint32_t)(step - kc);
      }
      break;
    }
    /* the glue's hold block (autonomy.c stepFull, after the FSM step): LAND COLLISION from its step, or a contact
     * the controller saw in LAND; latched in LAND */
    blocked = inLand && (blocked || out.collided || vo.contact);
    if (blocked && !sim.onGround && !vo.contact && !vo.touchdown) {
      res->blockedInAir = true;
    }
    if (out.state == AUTO_LAND && out.reason == AUTO_RS_COLLISION && prevState != AUTO_LAND) {
      res->blockedAtLand = blocked;
    }
    prevState = out.state;

    /* ---- the vertical controller ---- */
    float thr = 0.0f;
    float camVzSeen = sim.camVz + sc->shScale * sim.v;   /* the camera's vertical scale error (SET_HEIGHT runs) */
    if (sc->gLatch && vzActive && sim.camVzValid && w.gD > 0.0f) {
      /* the latched G (scen_t.gLatch): the camera's part below ~1 s read x G_true / G_used */
      const float lnF = sc->gLatch == 1 ? fminf(fmaxf(out.fwdLn - w.gLnAt, -CAM_VZ_FWD_LN_MAX), CAM_VZ_FWD_LN_MAX)
                                        : 0.0f;
      w.gRatio = (w.gD / fmaxf(D, 0.2f)) / expf(lnF);
      if (out.state == AUTO_HOVER) {
        w.gRatioMax = fmaxf(w.gRatioMax, w.gRatio);   /* (LAND's own descent may not re-take G either) */
        w.gRatioMin = fminf(w.gRatioMin, w.gRatio);
      }
      w.camLp += dt / (1.0f + dt) * (sim.camVz - w.camLp);
      camVzSeen = (sim.camVz - w.camLp) + w.gRatio * w.camLp;
    }
    if (vzActive) {
      /* the observer: healthy (or on a guess in a commanded turn) with an accepted frame in the last 2 s */
      const bool healthy = sim.camHealthy && (w.tracked || out.mv.camGuessOk) && t - w.lastObsT < 2.0f && !lateCamOut;
      vzInput_t vi = {
        .dt = dt, .cmd = out.vzCmd, .accZ = sim.accZ, .gravZ = cosf(in.tiltDeg * D2R), .tiltDeg = in.tiltDeg,
        .gyroMaxDps = 0.5f, .vbat = vbat, .thrustApplied = applied, .camHealthy = healthy,
        .camVzValid = sim.camVzValid && !lateCamOut, .camVz = camVzSeen, .camBias = sim.camBias, .vRef = out.mv.vRef,
        .hOff = out.mv.hOff, .blind = out.blind,   /* as autonomy.c vzRun() (out.collided until 2026-10-15) */
        .landFastOk = autoFsmLandFastReason(out.reason),   /* as autonomy.c vzRun() */
        .hush = out.hush, .resume = out.resume,
      };
      vi.accZ += contactAcc;
      /* autonomy.c applyHeightShift(): an accepted SET_HEIGHT shifts the estimate before this step's vzStep */
      if (out.hShift != 0.0f && !vzShiftHeight(&vz, out.hShift)) {
        res->shShiftRefused++;
      }
      vzStep(&vz, &vi, &vo);
      if (vo.phase == VZ_LAND && prevVzPhase != VZ_LAND && vo.liftedOff) {
        res->vzLands++;
        res->vzLandsBlind += vi.blind;
        res->vzLandsCollided += out.collided;
        vzLandH0 = sim.h;
        res->hLastLand0 = sim.h;
        res->nearFloorOnS = 0.0f;
        res->lastLandBase = vz.landFlatBase;
        res->lastLandFlat = vz.landFlatEff;
      }
      if (vo.phase == VZ_HOLD && prevVzPhase == VZ_LAND && vo.reason == VZ_R_RESUME && res->resumeH < 0.0f) {
        res->resumeH = sim.h;
      }
      if (vo.phase == VZ_LAND && vo.liftedOff) {
        res->hRiseLand = fmaxf(res->hRiseLand, sim.h - vzLandH0);
      }
      prevVzPhase = vo.phase;
      if (vo.touchdown) {
        res->touchdown = true;
      }
      if (vo.phase == VZ_LAND && vo.liftedOff && !res->vzLandSeen) {
        res->vzLandSeen = true;
        res->landFast = vz.landFast;
        res->vzLandT = t;
        res->landTRef = vo.tRef;
        res->landDurS = -1.0f;
      }
      if (vo.phase == VZ_DONE && res->vzLandSeen && res->landDurS < 0.0f) {
        res->landDurS = t - res->vzLandT;
      }
      if (vo.phase == VZ_CUT && vo.reason == VZ_R_LAND_TIMEOUT && !sim.onGround) {
        res->cutInAir = true;
      }
    }
    if (out.thrust == AUTO_THR_VERT_CTRL || out.thrust == AUTO_THR_VERT_LAND) {
      thr = fminf(vo.thrust, out.thrustMax);
    }
    vzSimStep(&sim, &pc, thr, dt);
    applied = thr;
    if (vzActive && vo.phase == VZ_LAND && vo.liftedOff && thr > 0.0f && sim.h < 0.12f) {
      res->nearFloorOnS += dt;
    }
    /* ---- the hand and the environment: forces on the plant in g (n), after its step, on its speed, height and
     * accelerometer (harness only) ---- */
    {
      float gain = 1.0f, n = 0.0f;
      bool held = false;
      const bool waitStart = phase >= 3 && out.state == AUTO_HOVER;
      if (sc->handKind != 0 && !handOn && !handGone) {
        if ((sc->handDuring == 0 && waitStart && cmdIdx < 0 && t - phaseT >= sc->handAtS) ||
            (sc->handDuring == 1 && waitStart && cmdIdx == 0 && t - phaseT >= sc->handAtS) ||
            (sc->handDuring == 2 && landSent) ||
            (sc->handDuring == 4 && bobFirstT >= 0.0f && t - bobFirstT >= sc->handAtS) ||
            (sc->handDuring == 5 && waitStart && cmdIdx == 0 && t - phaseT >= sc->handAtS) ||
            (sc->handDuring == 3 && out.state == AUTO_TAKEOFF && vo.phase == VZ_CLIMB && sim.h >= sc->handAtH)) {
          handOn = true;
          handTop = sc->handKind == 2 ? sim.h : sim.h - sc->handGap;
          handVel = sc->handKind == 2 ? 0.0f : sc->handV;
        }
      }
      if (handOn) {
        const float tc = res->handContact ? t - res->contactT : -1.0f;
        float vp = sc->handDuring == 2 || sc->handKind == 2 ? 0.0f : sc->handV;
        if (tc >= 0.0f && sc->handKind == 1 && sc->handDuring != 2) {
          /* from the contact: the approach speed stops in 0.1 s (a linear ramp), and the lift (handLift over
           * HAND_LIFT_S, a minimum-jerk profile: a hand's smooth stroke) adds to it; then the palm holds still */
          vp = sc->handV * fmaxf(0.0f, 1.0f - tc / 0.1f);
          if (sc->handLift > 0.0f && tc < HAND_LIFT_S) {
            const float u = tc / HAND_LIFT_S;
            vp += sc->handLift / HAND_LIFT_S * 30.0f * u * u * (1.0f - u) * (1.0f - u);
          }
          if (sc->handWobM > 0.0f) {
            vp += sc->handWobM * 2.0f * 3.14159265f * sc->handWobHz * cosf(2.0f * 3.14159265f * sc->handWobHz * tc);
          }
        }
        if (tc >= 0.0f && sc->handReleaseS > 0.0f && tc >= sc->handReleaseS) {
          vp = -0.5f;              /* released: the hand drops away */
          res->released = true;
        }
        if (tc >= 0.0f && sc->handGiveUpS > 0.0f && (giveUpT >= 0.0f || (!inLand && tc >= sc->handGiveUpS))) {
          vp = -0.2f;              /* no LAND (no cue, no COLLISION): the operator lowers the hand away ... */
          if (giveUpT < 0.0f) {
            giveUpT = t;
            res->gaveUp = true;
          }
          if (sim.h - handTop > 0.20f) {
            handOn = false;        /* ... and takes it out from under the drone */
            handGone = true;
          }
        }
        handVel = vp;
        handTop += vp * dt;
        if (sc->handGe > 0.0f && sc->handKind == 1) {
          gain += sc->handGe * expf(-fmaxf(0.0f, sim.h - handTop) / 0.03f);   /* the palm's own ground effect */
        }
        const float pen = handTop - sim.h;
        float nh = (sc->handK * pen + sc->handC * (handVel - sim.v)) / (HAND_M_KG * VZ_G);
        if (sc->handKind == 1) {
          nh = pen > 0.0f ? fmaxf(0.0f, nh) : 0.0f;   /* a palm only pushes */
        }
        if (sc->handTilt > 0.0f) {
          handTiltNow = nh != 0.0f ? fminf(sc->handTilt, handTiltNow + sc->handTilt / 0.3f * dt)
                                   : fmaxf(0.0f, handTiltNow - 100.0f * dt);
        }
        if (nh != 0.0f) {
          held = true;
          if (!res->handContact) {
            res->handContact = true;
            res->contactT = t;
            res->hContact = sim.h;
          }
        }
        n += nh;
      }
      if (sc->ceilH > 0.0f) {
        gain += 0.5f * expf(-fmaxf(0.0f, sc->ceilH - sim.h) / 0.01f);
        if (sim.h > sc->ceilH) {
          n -= fmaxf(0.0f, (2000.0f * (sim.h - sc->ceilH) + 10.0f * sim.v) / (HAND_M_KG * VZ_G));
          held = true;
        }
      }
      /* gusts: a vertical step, turbulence (both forces on the drone), a lateral push */
      const float tw = phase >= 3 ? t - phaseT : -1.0f;
      envN = 0.0f;
      if (sc->gustG != 0.0f && tw >= sc->gustAtS && tw < sc->gustAtS + sc->gustS) {
        envN += sc->gustG;
      }
      if (sc->turbG > 0.0f && phase >= 3) {
        const float a = dt / 0.2f;
        turbN += -a * turbN + sqrtf(2.0f * a) * sc->turbG * gauss();
        envN += turbN;
      }
      if (sc->latGust > 0.0f && tw >= sc->gustAtS && tw < sc->gustAtS + 0.5f && !sim.onGround) {
        w.vx += sc->latGust * dt;
      }
      n += envN + (gain - 1.0f) * sim.tr * sim.vbat / pc.kTrue;
      if (!sim.onGround && n != 0.0f) {
        sim.v += VZ_G * n * dt;
        sim.h += VZ_G * n * dt * dt;
        sim.accZ += n;
      } else if (sim.onGround && n > 0.0f && sc->handKind == 2) {
        sim.onGround = false;   /* a grab can lift it off */
      }
      if (held && sc->handKind != 0) {
        res->hLowHeld = fminf(res->hLowHeld, sim.h);
        w.vx *= 1.0f - dt / 0.1f;   /* friction on the hand */
        w.vy *= 1.0f - dt / 0.1f;
      }
      if (res->cued && thr > 0.0f && res->cueTHold > 0.0f) {
        res->thrAfterCue = fmaxf(res->thrAfterCue, thr / res->cueTHold);
      }
      if (res->offT < 0.0f && res->handContact && inLand && thr == 0.0f) {
        res->offT = t;
      }
      if (!res->offSeen && inLand && thr == 0.0f) {
        res->offSeen = true;
        res->hOff = sim.h;
        res->offFree = !sim.onGround && !held && sim.h > 0.03f;
        res->offCeil = sc->ceilH > 0.0f && sim.h > sc->ceilH - 0.02f;
      }
      if (sim.onGround && !wasGround && inLand) {
        res->floorT = t;   /* the latest floor arrival in LAND */
        if (res->floor1T < 0.0f) {
          res->floor1T = t;
        }
      }
      if (res->floor1T >= 0.0f && inLand) {
        res->hAfterFloor = fmaxf(res->hAfterFloor, sim.h);
        if (res->floorToOffS == 0.0f && thr == 0.0f) {
          res->floorToOffS = t - res->floor1T + 1e-6f;
        }
      }
      wasGround = sim.onGround;
      if (vzActive && vo.phase == VZ_DONE && vo.reason == VZ_R_TOUCHDOWN_SUPPORT && !sim.onGround && sim.h > 0.03f &&
          !held) {
        res->supInAir = true;
      }
      if (vzActive && (vo.phase == VZ_DONE || vo.phase == VZ_CUT) && res->offAfterFloorS == 0.0f && sim.onGround &&
          res->floorT >= 0.0f) {
        res->offAfterFloorS = t - res->floorT + 1e-6f;
      }
    }
    if (!sim.onGround) {
      res->minVAir = fminf(res->minVAir, sim.vbat);
    }
    if (vzActive && vo.phase == VZ_LAND && vo.liftedOff && vo.tRef > 0.0f) {
      res->maxLandOverRef = fmaxf(res->maxLandOverRef, thr / vo.tRef);
    }
    if (out.state == AUTO_HOVER && res->hoverT0 <= 0.0f) {
      res->hoverT0 = t;
      res->holdPHover = holdP;
    }
    if (res->hoverT0 > 0.0f && !res->hover2Seen && t - res->hoverT0 >= 2.0f) {
      res->hover2Seen = true;
      /* facing psi0: forward along (cos psi0, sin psi0), right along (sin psi0, -cos psi0) */
      const float dx = w.x - sc->x0, dy = w.y - sc->y0;
      res->dFwdHover2 = dx * cosf(sc->psi0) + dy * sinf(sc->psi0);
      res->dRightHover2 = dx * sinf(sc->psi0) - dy * cosf(sc->psi0);
    }
    if (sc->drain) {
      res->maxThr = fmaxf(res->maxThr, thr);
      if (inLand && !sim.onGround) {
        res->minVLand = fminf(res->minVLand, sim.vbat);
      }
      if (out.state == AUTO_HOVER && res->tAt538 < 0.0f && vo.tHold >= 53800.0f) {
        res->tAt538 = t;
      }
    }

    /* ---- the horizontal plant ---- */
    const bool camApplied = holdEng && w.lastHoldT >= 0.0f && t - w.lastHoldT < 0.3f && out.state != AUTO_IDLE &&
                            !blocked;
    const float step6 = holdSlew * dt;   /* 6 deg/s (controller_pid.c), the hold's own slew on a fast move's frames */
    float tR = camApplied ? holdR : 0.0f, tP = camApplied ? holdP : 0.0f;
    if (!camApplied && simPct > 100 && out.state == AUTO_HOVER && !out.mv.holdWait &&
        (out.mv.ffRollDeg != 0.0f || out.mv.ffPitchDeg != 0.0f) && !blocked) {
      tR = out.mv.ffRollDeg;   /* controller_pid.c: a fast MOVE's brake with the hold stale (autonomyBrakeLean) */
      tP = out.mv.ffPitchDeg;
    }
    w.camCorrR += fmaxf(-step6, fminf(step6, tR - w.camCorrR));
    w.camCorrP += fmaxf(-step6, fminf(step6, tP - w.camCorrP));
    if (blocked) {
      w.camCorrR = w.camCorrP = 0.0f;   /* controller_pid.c: autonomyHoldBlocked() zeroes the correction at once */
    }
    if (kc >= 0 && inLand) {
      res->corrAfter = fmaxf(res->corrAfter, fmaxf(fabsf(w.camCorrR), fabsf(w.camCorrP)));
      res->travelLand = fmaxf(res->travelLand, hypotf(w.x - contactX, w.y - contactY));
    }
    /* scen_t leanDeg: an unlearned pitch error ramped in from the MOVE's acceptance (and kept); driftDeg / kickV from
     * driftAtS before the MOVE is sent (the send at the wait's moveAtS) */
    float leanNow = (sc->leanDeg != 0.0f && sc->oneMove && res->accT > 0.0f)
                        ? sc->leanDeg * fminf(1.0f, (t - res->accT) / fmaxf(sc->leanS, 1e-3f))
                        : 0.0f;
    if ((sc->driftDeg != 0.0f || sc->kickV != 0.0f) && sc->oneMove && out.state == AUTO_HOVER && phase >= 3) {
      const float tSend = res->accT > 0.0f ? res->accT : phaseT + (sc->moveAtS > 0.0f ? sc->moveAtS : 10.0f);
      if (t >= tSend - sc->driftAtS) {
        leanNow += sc->driftDeg;
        if (!driftKicked && sc->kickV != 0.0f) {
          w.vx = -sc->kickV * cosf(w.psi);
          w.vy = -sc->kickV * sinf(w.psi);
        }
        driftKicked = true;
      }
    }
    const float cmdR = w.camCorrR + w.trimR, cmdP = w.camCorrP + w.trimP + leanNow;
    w.roll += (cmdR - w.roll) * dt / 0.08f;
    w.pitch += (cmdP - w.pitch) * dt / 0.08f;
    /* the attitude filter's coupling (sc->coupled): w.roll / w.pitch are the estimate the controller holds; the true
     * lean is the estimate minus the filter's error, which the accelerometer (drag only in the air) moves */
    float trueR = w.roll, trueP = w.pitch;
    if (sc->coupled) {
      if (sim.onGround) {
        w.dR = w.dF = w.iR = w.iF = 0.0f;   /* the floor: the filter levels on the accelerometer */
      } else {
        const float vRb = w.vx * spsi - w.vy * cpsi, vFb = w.vx * cpsi + w.vy * spsi;
        const float aR = drag * vRb / G / D2R, aFw = drag * vFb / G / D2R;   /* the accelerometer's lean, deg */
        const float eR = w.roll, eF = -w.pitch;
        w.dR += (0.384f * (aR - eR) + w.iR) * dt;
        w.dF += (0.384f * (aFw - eF) + w.iF) * dt;
        w.iR += 0.02f * (aR - eR) * dt;
        w.iF += 0.02f * (aFw - eF) * dt;
      }
      trueR = w.roll - w.dR;
      trueP = w.pitch + w.dF;   /* forward = -pitch: true forward lean eF - dF */
    }
    res->maxLean = fmaxf(res->maxLean, fmaxf(fabsf(w.roll), fabsf(w.pitch)));
    if (inLand && !contactSeen) {
      const float hv = hypotf(w.vx, w.vy);
      res->landHV = fmaxf(res->landHV, hv);
      if (vo.contact || sim.onGround) {
        contactSeen = true;
        res->tdHV = hv;
        const float dx = w.x - landX, dy = w.y - landY, c0 = cosf(landPsi), s0 = sinf(landPsi);
        res->landR = dx * s0 - dy * c0;
        res->landF = dx * c0 + dy * s0;
      }
    }
    if (!sim.onGround) {
      const float aR = G * tanf(trueR * D2R), aF = -G * tanf(trueP * D2R);
      float ax = aF * cpsi + aR * spsi, ay = aF * spsi - aR * cpsi;
      if (sc->fwdPush != 0.0f && sc->oneMove && res->accT > 0.0f && res->oneWhy == 0 && res->oneEnd == AUTO_MVE_NONE) {
        const float t0 = sc->pushAtS < 0.0f ? res->profT + sc->pushAtS : sc->pushAtS, ta = t - res->accT;
        if (ta >= t0 && ta < t0 + sc->pushS) {
          ax += sc->fwdPush * cpsi;   /* (scen_t fwdPush: along the heading) */
          ay += sc->fwdPush * spsi;
        }
      }
      w.vx += (ax - drag * w.vx) * dt;
      w.vy += (ay - drag * w.vy) * dt;
      w.x += w.vx * dt;
      w.y += w.vy * dt;
      if (sc->wallMode) {
        /* the obstacle: a rigid plane at y = wallY (or x = wallX), the drone's guard radius 0.05 m */
        float *pos = sc->wallMode == 2 ? &w.x : &w.y, *vel = sc->wallMode == 2 ? &w.vx : &w.vy;
        const float lim = (sc->wallMode == 2 ? sc->wallX : sc->wallY) - 0.05f;
        if (*pos > lim) {
          *pos = lim;
          if (*vel > 0.0f) {
            const float dv = (1.0f + sc->wallE) * *vel;
            *vel = -sc->wallE * *vel;
            res->wallHits++;
            res->wallHitsLand += inLand;
            if (inLand) {
              res->wallLandHitV = fmaxf(res->wallLandHitV, dv / (1.0f + sc->wallE));
              res->wallLandHitH = fmaxf(res->wallLandHitH, sim.h);
            }
            const float g4 = 1.0f + dv / (G * 0.004f);
            if (g4 > 1.6f) {
              wallPulse = 4;
              wallPulseG = g4;
            }
          }
        }
        const float gapNow = lim - *pos;
        if (res->wallT >= 0.0f && out.state == AUTO_HOVER && !landSent) {
          const float tc = t - res->wallT;
          if (res->wallGapN < 100 && tc >= 0.5f * (float)res->wallGapN) {
            res->wallGap[res->wallGapN++] = gapNow;
          }
          if (tc >= 3.0f && res->wallGap3 == 0.0f) {
            res->wallGap3 = gapNow + 1e-6f;
          }
          if (tc >= 3.0f) {
            res->wallMinLate = fminf(res->wallMinLate, gapNow);
            res->wallPressLate += gapNow <= 0.002f ? dt : 0.0f;
          }
        }
      }
    } else {
      w.vx = w.vy = 0.0f;
    }
    w.airT = sim.onGround ? 0.0f : w.airT + dt;
    /* the gain scale's take-off grid (testTakeoffGrid) */
    if (res->liftT <= 0.0f && !sim.onGround && vzActive) {
      res->liftT = t;
      res->xH3 = res->xL8 = res->xL15 = res->xL45 = -1.0f;
    }
    if (res->liftT > 0.0f) {
      const float tl = t - res->liftT, dist = hypotf(w.x - sc->x0, w.y - sc->y0);
      if (tl <= 15.0f) {
        res->max15 = fmaxf(res->max15, dist);
      }
      if (res->xL8 < 0.0f && tl >= 8.0f) {
        res->xL8 = dist;
      }
      if (res->xL15 < 0.0f && tl >= 15.0f) {
        res->xL15 = dist;
        res->sL15 = gainS;
      }
      if (res->xL45 < 0.0f && tl >= 45.0f) {
        res->xL45 = dist;
      }
      if (res->xH3 < 0.0f && res->hoverT0 > 0.0f && t - res->hoverT0 >= 3.0f) {
        res->xH3 = dist;
      }
      res->sMax = fmaxf(res->sMax, gainS);
      /* the 0.3-1.5 Hz band of the applied correction, from lift-off */
      static float hpInR, hpInP, hpR, hpP, bpR, bpP, bSum;
      static int bN;
      if (tl - dt < 0.0f) {
        hpInR = w.camCorrR;
        hpInP = w.camCorrP;
        hpR = hpP = bpR = bpP = bSum = 0.0f;
        bN = 0;
      }
      const float aH = 0.5305f / (0.5305f + dt), aL = dt / (0.1061f + dt);   /* tau 1 / (2 pi f) */
      hpR = aH * (hpR + w.camCorrR - hpInR);
      hpP = aH * (hpP + w.camCorrP - hpInP);
      hpInR = w.camCorrR;
      hpInP = w.camCorrP;
      bpR += aL * (hpR - bpR);
      bpP += aL * (hpP - bpP);
      if (tl >= 20.0f && tl <= 45.0f) {
        static float rLo, rHi, pLo, pHi, brLo, brHi, bpLo, bpHi;
        if (tl - dt < 20.0f) {
          rLo = rHi = w.camCorrR;
          pLo = pHi = w.camCorrP;
          brLo = brHi = bpR;
          bpLo = bpHi = bpP;
        }
        rLo = fminf(rLo, w.camCorrR);
        rHi = fmaxf(rHi, w.camCorrR);
        pLo = fminf(pLo, w.camCorrP);
        pHi = fmaxf(pHi, w.camCorrP);
        brLo = fminf(brLo, bpR);
        brHi = fmaxf(brHi, bpR);
        bpLo = fminf(bpLo, bpP);
        bpHi = fmaxf(bpHi, bpP);
        bSum += 0.5f * (bpR * bpR + bpP * bpP);
        bN++;
        res->p2p = fmaxf(rHi - rLo, pHi - pLo);
        res->p2pBand = fmaxf(brHi - brLo, bpHi - bpLo);
        res->rmsBand = sqrtf(bSum / (float)bN);
      }
      if (sc->endAfterLiftS > 0.0f && tl >= sc->endAfterLiftS) {
        break;
      }
    }
    /* the yaw lock: the setpoint is the take-off heading minus the offset (+ clockwise) */
    const float psiSet = sc->psi0 - out.mv.yawOffDeg * D2R;
    float e = psiSet - w.psi;
    while (e > 3.14159265f) e -= 6.2831853f;
    while (e < -3.14159265f) e += 6.2831853f;
    w.psi += e * dt / 0.15f;

    in.nowMs++;
  }
  res->impactV = sim.impactV;
  res->hMax = sim.hMax;
  if (csvF) {
    fclose(csvF);
  }
  res->gRatioMax = w.gRatioMax;
  res->gRatioMin = w.gRatioMin;
  /* (not the runs outside the design on purpose: the latched G without the firmware's gain following the move, the
   * expansion scale below the floor's AUTO_MV_K_LOW: their LANDs are what those tests measure) */
  if (sc->handKind == 0 && sc->gustG == 0.0f && sc->turbG == 0.0f && sc->ceilH == 0.0f && sc->gLatch != 2 &&
      !(sc->kFix > 0.0f && sc->kFix < AUTO_MV_K_LOW)) {
    fpRuns++;
    fpSupAir += res->supInAir;
  }
}

/* ---- tests -------------------------------------------------------------------- */

static int cmpf(const void *a, const void *b)
{
  const float x = *(const float *)a, y = *(const float *)b;
  return x < y ? -1 : x > y;
}

static void testMission(void)
{
  const int n = 30;
  int ok = 0, okMoves = 0, soft = 0, refused = 0;
  float worst[6] = {0, 0, 0, 0, 0, 0};
  float worstLean = 0.0f, worstNow = -1.0f, mDurMax = 0.0f, mDurSum = 0.0f;
  int mFast = 0;
  static float errs[30 * 5];
  int nErr = 0;
  for (int i = 0; i < n; i++) {
    scen_t sc = {.seed = 100u + (uint32_t)i, .x0 = 0.8f * (uni() - 0.5f), .y0 = 1.0f + 0.4f * (uni() - 0.5f),
                 .psi0 = 1.5708f, .eDmax = 0.10f, .trim = 0.3f, .mission = true, .camLossAtMoveS = -1.0f,
                 .battDropAtS = -1.0f};
    result_t r;
    runSim(&sc, &r);
    bool all = r.ncmd == 5;
    for (int c = 0; c < r.ncmd; c++) {
      for (int a = 0; a < 3; a++) {
        const float wv = r.want[c][a], gv = r.got[c][a];
        float err;
        if (wv != 0.0f) {
          err = fabsf(gv - wv) / fabsf(wv);
          worst[c] = fmaxf(worst[c], err);
          all = all && err <= 0.25f;
          if (nErr < 30 * 5) {
            errs[nErr++] = err;
          }
          worstNow = fmaxf(worstNow, fabsf(r.gotNow[c][a] - wv) - 0.20f * fabsf(wv));
        }
      }
      if (c == 2) {
        const float err = fabsf(r.yawGot - r.yawWant) / fabsf(r.yawWant);
        worst[c] = fmaxf(worst[c], err);
        all = all && err <= 0.05f;
      }
    }
    okMoves += all;
    refused += r.refused;
    soft += r.touchdown && r.impactV <= 0.5f;
    mFast += r.landFast;
    mDurMax = fmaxf(mDurMax, r.landDurS);
    mDurSum += r.landDurS;
    worstLean = fmaxf(worstLean, r.maxLean);
    ok += !r.killed && r.landWhy == AUTO_RS_LAND_CMD && r.endWhy == AUTO_RS_TOUCHDOWN;
    if (i < 3 || !all) {
      printf("info: mission seed %u:", (unsigned)sc.seed);
      for (int c = 0; c < r.ncmd; c++) {
        printf(" %s -> (%+.2f %+.2f %+.2f; from the drone %+.2f %+.2f)", r.name[c], r.got[c][0], r.got[c][1],
               r.got[c][2], r.gotNow[c][0], r.gotNow[c][1]);
      }
      printf(" yaw %.1f, LAND %s, end %s, impact %.2f m/s, peak %.2f m, lean max %.1f deg\n", r.yawGot,
             autoReasonName(r.landWhy), autoReasonName(r.endWhy), r.impactV, r.hMax, r.maxLean);
    }
  }
  printf("info: mission: worst relative error per command: right %.0f %%, back %.0f %%, yaw %.0f %%, up %.0f %%, "
         "forward %.0f %%; lean max %.1f deg; LAND from ~1.2 m: fast %d/%d, to DONE mean %.2f max %.2f s\n",
         worst[0] * 100.0f, worst[1] * 100.0f, worst[2] * 100.0f, worst[3] * 100.0f, worst[4] * 100.0f, worstLean,
         mFast, n, mDurSum / (float)n, mDurMax);
  CHECK(refused == 0, "every command of the mission accepted (%d refused)", refused);
  qsort(errs, (size_t)nErr, sizeof(errs[0]), cmpf);
  int in20 = 0;
  for (int i = 0; i < nErr; i++) {
    in20 += errs[i] <= 0.20f;
  }
  printf("info: mission moves (%d): relative error p50 %.0f %%, p90 %.0f %%, p95 %.0f %%, max %.0f %%; within 20 %%: "
         "%d\n", nErr, errs[nErr / 2] * 100.0f, errs[nErr * 9 / 10] * 100.0f, errs[nErr * 19 / 20] * 100.0f,
         errs[nErr - 1] * 100.0f, in20);
  CHECK(in20 * 100 >= nErr * 95, "the moves within +/-20 %% of their command from where the hold held the drone in "
        ">= 95 %% (%d of %d; D error +/-10 %%, the camera's position random walk ~0.03 /s per frame)", in20, nErr);
  CHECK(okMoves == n, "every move within +/-25 %%, every yaw within +/-5 %% (%d/%d)", okMoves, n);
  printf("info: from the drone's position when the command came (the hover's own wander toward its hold point "
         "included): worst error %.3f m beyond 20 %% of the command\n", worstNow);
  CHECK(worstNow <= 0.10f, "from the drone's position then: within 20 %% + 0.10 m (%.3f m beyond 20 %%)", worstNow);
  CHECK(ok == n, "LAND by the Mac, touchdown, never a guard or a kill (%d/%d)", ok, n);
  CHECK(soft == n, "slow touchdown (<= 0.5 m/s) from about 1.2 m (%d/%d)", soft, n);
  CHECK(worstLean < 6.0f, "lean stays small (max %.1f deg)", worstLean);

  /* the observer's D off by +/-25 %: info only (the moves scale with it) */
  float w25 = 0.0f;
  int ok25 = 0;
  for (int i = 0; i < 8; i++) {
    scen_t sc = {.seed = 300u + (uint32_t)i, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.25f, .trim = 0.3f,
                 .mission = true, .camLossAtMoveS = -1.0f, .battDropAtS = -1.0f};
    result_t r;
    runSim(&sc, &r);
    for (int c = 0; c < r.ncmd; c++) {
      for (int a = 0; a < 3; a++) {
        if (r.want[c][a] != 0.0f) {
          w25 = fmaxf(w25, fabsf(r.got[c][a] - r.want[c][a]) / fabsf(r.want[c][a]));
        }
      }
    }
    ok25 += !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN;
  }
  printf("info: D error +/-25 %%: worst relative move error %.0f %%\n", w25 * 100.0f);
  CHECK(ok25 == 8, "D error +/-25 %%: every mission still ends in a touchdown, no kill (%d/8)", ok25);
}

static void testGuards(void)
{
  /* the camera lost 1.5 s into a forward move, for 3 s: PAUSED, then (since 2026-10-15; LAND MOVE_NO_CAM before)
   * the move stopped NO_VIEW with the MOVE_STOPPED event, the hold on, no LAND; the Mac's LAND at the wait's end */
  scen_t sc = {.seed = 500u, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.1f, .trim = 0.3f,
               .camLossAtMoveS = 1.5f, .battDropAtS = -1.0f};
  result_t r;
  runSim(&sc, &r);
  printf("info: camera lost mid-move: LAND %s, end %s, impact %.2f m/s\n", autoReasonName(r.landWhy),
         autoReasonName(r.endWhy), r.impactV);
  CHECK(r.pausedSeen && (r.evMask & (1u << AUTO_EV_MOVE_STOPPED)) && r.landWhy == AUTO_RS_LAND_CMD &&
        r.macWaitLand && !r.killed && r.touchdown && r.impactV <= 0.6f,
        "camera lost mid-move: PAUSED, MOVE_STOPPED, no LAND for it (the Mac's at the wait's end: %s), slow touchdown "
        "(%.2f)", autoReasonName(r.landWhy), r.impactV);
  /* the battery falls in the wait to 0.40 V over the BATT_LOW floor at rest (3.40 V; loaded just under 3.00 V):
   * LAND BATT_LOW. The end of battery (user decision 2026-10-13, late): the floor (3.00 V for 0.5 s) lands it the
   * slow way (28.7 s, 0.48 m/s); 6742d91 (floor 2.65 V for 1 s, ceiling 54000) on this same 3.40 V step ran out of
   * thrust first: LAND VZ (HOLD_CAPPED) at 28.9 s, 1.15 m/s. (Until cb09352 this case followed the floor down to a
   * 3.05 V step: the next case.) */
  scen_t sb = {.seed = 501u, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.1f, .trim = 0.3f,
               .camLossAtMoveS = -1.0f, .battDropAtS = 30.0f};
  runSim(&sb, &r);
  printf("info: battery low (rest %.2f V): LAND %s (vz %s) at air %u ms, loaded %.3f V, impact %.2f m/s\n",
         r.landVRest, autoReasonName(r.landWhy), vzReasonName((vzReason_t)r.vzLandWhy), (unsigned)r.landAirMs, r.landV,
         r.impactV);
  CHECK(r.landWhy == AUTO_RS_BATT_LOW && !r.killed && r.touchdown && r.impactV <= 0.6f,
        "battery low in the wait: LAND BATT_LOW (%s), touchdown %.2f <= 0.6 m/s", autoReasonName(r.landWhy),
        r.impactV);
  /* The old deep collapse, pinned at 3.05 V at rest (the step above before the floor moved; loaded about 2.5 V at
   * the hover thrust): faster than the BATT_LOW floor can act, so the backstops land it, the 1 kHz VBAT_DIP
   * (2.55 V) or the controller's HOLD_CAPPED at the 58000 ceiling: cb09352 LAND VBAT_DIP at 28.3 s, 1.69 m/s;
   * 6742d91 LAND VZ (HOLD_CAPPED at 54000) at 28.8 s, 1.53 m/s. Bounded as before: no KILLED, a touchdown,
   * <= 2.0 m/s. */
  scen_t sd = sb;
  sd.battDropV = 3.05f;
  runSim(&sd, &r);
  printf("info: battery collapse (rest %.2f V): LAND %s (vz %s) at air %u ms, loaded %.3f V, impact %.2f m/s\n",
         r.landVRest, autoReasonName(r.landWhy), vzReasonName((vzReason_t)r.vzLandWhy), (unsigned)r.landAirMs, r.landV,
         r.impactV);
  CHECK((r.landWhy == AUTO_RS_BATT_LOW || r.landWhy == AUTO_RS_VBAT_LOW || r.landWhy == AUTO_RS_VBAT_DIP ||
         r.landWhy == AUTO_RS_VZ) && !r.killed && r.touchdown && r.impactV <= 2.0f,
        "battery collapse to 3.05 V at rest in the wait: LAND %s (a battery LAND or the controller's), no KILLED, "
        "touchdown %.2f <= 2.0 m/s", autoReasonName(r.landWhy), r.impactV);
  /* A VBAT_DIP LAND from 1.0 m with the camera estimate and Vf well above 3.15 V (verifier 2026-10-14): a
   * brown-out guard, off the fast LAND's allow list, so the LAND is the slow one (never above 0.98 x T_ref). */
  scen_t sdip = sb;
  sdip.battDropAtS = -1.0f;
  sdip.dipAtS = 15.0f;
  runSim(&sdip, &r);
  printf("info: VBAT_DIP at 15 s: LAND %s, fast %d, LAND/T_ref max %.4f, impact %.2f m/s\n", autoReasonName(r.landWhy),
         r.landFast, (double)r.maxLandOverRef, (double)r.impactV);
  CHECK(r.landWhy == AUTO_RS_VBAT_DIP && r.vzLandSeen && !r.landFast && r.maxLandOverRef <= 0.98f * 1.0001f &&
        r.touchdown && !r.killed, "a VBAT_DIP LAND from 1.0 m with the camera: not fast (%d), thrust <= 0.98 x T_ref "
        "(%.4f), a touchdown", r.landFast, (double)r.maxLandOverRef);
  /* the long hover: no clock lands it since 2026-10-15 (AIR_TIME at 120 s until then; the controller's HOLD_TIMEOUT
   * backstop is 599 s); the Mac lands at 120 s of air: LAND_CMD, the fast LAND allowed, a touchdown within the fast
   * LAND's bounds (the mission's 0.5 m/s) */
  scen_t sa = {.seed = 502u, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.1f, .trim = 0.3f,
               .camLossAtMoveS = -1.0f, .battDropAtS = -1.0f, .airLimit = true};
  runSim(&sa, &r);
  printf("info: air limit: LAND %s at air %u ms, true height then %.2f m, true peak %.2f m, fast %d, LAND to DONE "
         "%.2f s, impact %.2f m/s\n", autoReasonName(r.landWhy), (unsigned)r.landAirMs, r.hAtLand, r.hMax,
         r.landFast, r.landDurS, r.impactV);
  CHECK(r.landWhy == AUTO_RS_LAND_CMD && r.macAirLand && r.landAirMs >= MAC_AIR_LAND_MS &&
        r.landAirMs <= MAC_AIR_LAND_MS + 2u && !r.killed && r.touchdown && r.endWhy == AUTO_RS_TOUCHDOWN &&
        r.impactV <= 0.5f, "a 120 s hover: nothing lands it, the Mac's LAND at %u ms, a touchdown <= 0.5 m/s after "
        "~119 s of hover (%.2f)", (unsigned)r.landAirMs, r.impactV);
  CHECK(r.landFast == (r.landTRef <= VZ_LAND_FAST_TREF_MAX), "the Mac's LAND is on the fast LAND's allow list: fast %d "
        "(T_ref %.0f)", r.landFast, (double)r.landTRef);
  /* STOP 2.5 s into a 1.0 m move: the drone stops near where it was */
  scen_t ss = {.seed = 503u, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.1f, .trim = 0.3f,
               .camLossAtMoveS = -1.0f, .battDropAtS = -1.0f, .stopMid = true};
  runSim(&ss, &r);
  printf("info: STOP mid-move: %.2f m done at the STOP, then %.2f m more in 4 s (top speed %.2f m/s)\n",
         r.stopTravelAtStop, r.stopDrift, (double)AUTO_MV_V_H_MAX);
  CHECK(r.stopDrift <= 0.10f && r.stopTravelAtStop > 0.05f && !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN,
        "STOP mid-move: at most 0.10 m further (%.2f), then LAND", r.stopDrift);
}

/* ---- after the verification of 591f176 ------------------------------------------ */

static scen_t baseScen(uint32_t seed, float y0)
{
  scen_t sc;
  memset(&sc, 0, sizeof(sc));
  sc.seed = seed;
  sc.x0 = 0.0f;
  sc.y0 = y0;
  sc.psi0 = 1.5708f;
  sc.eDmax = 0.10f;
  sc.trim = 0.3f;
  sc.camLossAtMoveS = -1.0f;
  sc.battDropAtS = -1.0f;
  return sc;
}

/* The hover at cm: the TAKEOFF there up to the take-off cap (AUTO_HEIGHT_CM_MAX), above it the cap and moves up of at
 * most AUTO_MV_UP_MAX_CM (2.00 m: 120 + 50 + 30; docs/autonomy-a2b.md 23.9) */
static void toHeight(scen_t *sc, uint16_t cm)
{
  sc->toCm = cm < AUTO_HEIGHT_CM_MAX ? cm : AUTO_HEIGHT_CM_MAX;
  sc->preN = 0;
  for (int left = (int)cm - (int)sc->toCm; left > 0 && sc->preN < 3; left -= AUTO_MV_UP_MAX_CM) {
    sc->preUps[sc->preN++] = (int16_t)(left < AUTO_MV_UP_MAX_CM ? left : AUTO_MV_UP_MAX_CM);
  }
}

/* A LAND after a finished MOVE: the hold (engaged until contact) keeps the
 * move's end; 591f176 flew the drone back toward the move's start during the
 * descent (this sim: 0.21-0.40 m sideways after a lateral move, 0.23-0.42 m
 * forward after a back one, 0.06-0.09 m back after a forward one). The
 * descent itself, read partly as image expansion, makes the hold creep
 * forward 0.05-0.18 m in every LAND (no move: the last case; stage C's 'LAND
 * forward creep'), so the forward band is -0.03..+0.22 m. */
static void testLandAfterMove(void)
{
  static const int16_t mv[][2] = {{80, 0}, {-60, 0}, {0, -50}, {0, 50}, {50, -50}, {0, 0}};
  float worst = 0.0f, worstAll = 0.0f, worstV = 0.0f, worstImp = 0.0f;
  float creepLo = 9.0f, creepHi = -9.0f, durMax = 0.0f, durSum = 0.0f;
  int ok = 0, n = 0, fast = 0;
  for (int k = 0; k < 6; k++) {
    for (int i = 0; i < 4; i++) {
      scen_t sc = baseScen(700u + 10u * (uint32_t)k + (uint32_t)i, 1.0f);
      sc.oneMove = true;
      sc.oneR = mv[k][0];
      sc.oneF = mv[k][1];
      result_t r;
      runSim(&sc, &r);
      if (k == 5) {   /* no move (the MOVE 0 0 is refused RANGE): the LAND's own creep */
        creepLo = fminf(creepLo, r.landF);
        creepHi = fmaxf(creepHi, r.landF);
        continue;
      }
      const float land = hypotf(r.landR, r.landF);
      worst = fmaxf(worst, fabsf(r.landR));
      worstAll = fmaxf(worstAll, land);
      worstV = fmaxf(worstV, r.tdHV);
      worstImp = fmaxf(worstImp, r.impactV);
      fast += r.landFast;
      durMax = fmaxf(durMax, r.landDurS);
      durSum += r.landDurS;
      n++;
      ok += r.oneWhy == 0 && r.oneEnd == AUTO_MVE_DONE && r.landWhy == AUTO_RS_LAND_CMD &&
            r.endWhy == AUTO_RS_TOUCHDOWN && !r.killed && fabsf(r.landR) <= 0.08f && r.landF >= -0.03f &&
            r.landF <= 0.22f && r.tdHV <= 0.12f;
      if (i == 0) {
        printf("info: MOVE %+d %+d: moved %+.2f %+.2f m (DONE after %.1f s); LAND to contact %+.2f %+.2f m, "
               "horizontal speed max %.2f, at contact %.2f m/s\n", mv[k][0], mv[k][1], r.mvR, r.mvF, r.oneEndS,
               r.landR, r.landF, r.landHV, r.tdHV);
      }
    }
  }
  printf("info: LAND after a finished MOVE (hold engaged until contact): sideways <= %.2f m, in all <= %.2f m, at "
         "contact <= %.2f m/s, impact <= %.2f m/s; a LAND without a move creeps forward %.2f..%.2f m; fast LAND %d/%d, "
         "LAND to DONE mean %.2f max %.2f s\n", worst, worstAll, worstV, worstImp, creepLo, creepHi, fast, n,
         durSum / (float)n, durMax);
  CHECK(ok == n, "LAND 2 s after a finished MOVE: <= 0.08 m sideways, forward within the LAND's own creep "
        "(-0.03..+0.22 m), <= 0.12 m/s at contact, a touchdown (%d/%d; worst %.2f m sideways, %.2f m/s)", ok, n,
        worst, worstV);
}

/* Faster moves (user request 2026-10-14, docs/autonomy-a2b.md 17.5): AUTO_MV_V_H_MAX 0.20 -> 0.30 m/s binds
 * only for long moves with the scene far away (lateral D > V / AUTO_MV_W_LAT_MAX, 2.5 m at 0.30; forward end
 * distance > V / AUTO_MV_W_FWD_MAX, 2.0 m). The far scene (y0 -0.5: D ~3.7 m): right 100, left 150, forward
 * 100 and 150, diagonal 70 / 70, 10 seeds; built with AUTO_MV_HOST_VH_OVERRIDE the same seeds also fly at 0.20
 * (the designer's move_sweep.c): every move DONE, the error mean <= 15 % and max <= 25 %, the lean <= 1.6 deg,
 * and never slower than the same seed at 0.20. */
#ifdef AUTO_MV_HOST_VH_OVERRIDE
extern float autoMvVhMaxOverride;
#endif
static void testFarMoves(void)
{
  static const int16_t mv[5][2] = {{100, 0}, {-150, 0}, {0, 100}, {0, 150}, {70, 70}};
  int n = 0, okDone = 0, okErr = 0, okLean = 0;
  float worstLean = 0.0f;
#ifdef AUTO_MV_HOST_VH_OVERRIDE
  int okDur = 0, nDur = 0;
  float worstSlower = -99.0f;
#endif
  for (int k = 0; k < 5; k++) {
    float errSum = 0.0f, errMax = 0.0f, d0 = 0.0f, dur = 0.0f, dur20 = 0.0f, lean = 0.0f;
    int done = 0, cnt = 0, n20 = 0;
    for (uint32_t sd = 0; sd < 10; sd++) {
      scen_t sc = baseScen(4100u + 10u * (uint32_t)k + sd, -0.5f);
      sc.oneMove = true;
      sc.oneR = mv[k][0];
      sc.oneF = mv[k][1];
      result_t r;
#ifdef AUTO_MV_HOST_VH_OVERRIDE
      autoMvVhMaxOverride = 0.0f;
#endif
      runSim(&sc, &r);
      n++;
      cnt++;
      const float want = hypotf(mv[k][0] * 0.01f, mv[k][1] * 0.01f);
      const float e = hypotf(r.mvR - mv[k][0] * 0.01f, r.mvF - mv[k][1] * 0.01f) / want;
      const bool isDone = r.oneWhy == 0 && r.oneEnd == AUTO_MVE_DONE && !r.killed && r.touchdown;
      done += isDone;
      errSum += e;
      errMax = fmaxf(errMax, e);
      lean = fmaxf(lean, r.maxLean);
      d0 += r.d0Obs;
      dur += r.oneEndS;
#ifdef AUTO_MV_HOST_VH_OVERRIDE
      result_t r20;
      autoMvVhMaxOverride = 0.20f;
      runSim(&sc, &r20);
      autoMvVhMaxOverride = 0.0f;
      if (r20.oneWhy == 0 && r20.oneEnd == AUTO_MVE_DONE) {
        n20++;
        dur20 += r20.oneEndS;
        nDur++;
        okDur += r.oneEndS <= r20.oneEndS + 0.05f;
        worstSlower = fmaxf(worstSlower, r.oneEndS - r20.oneEndS);
      }
#endif
    }
    okDone += done;
    okErr += errSum / (float)cnt <= 0.15f && errMax <= 0.25f;
    okLean += lean <= 1.6f;
    worstLean = fmaxf(worstLean, lean);
    printf("info: far scene (D %.2f m) MOVE %+d %+d at %.2f m/s: DONE %d/%d, error mean %.0f %% max %.0f %%, lean max "
           "%.1f deg, MOVE to its end %.2f s", d0 / (float)cnt, mv[k][0], mv[k][1], (double)AUTO_MV_V_H_MAX, done, cnt,
           100.0f * errSum / (float)cnt, 100.0f * errMax, lean, dur / (float)cnt);
    if (n20 > 0) {
      printf(" (at 0.20 m/s, the same seeds: %.2f s)", dur20 / (float)n20);
    }
    printf("\n");
  }
  CHECK(okDone == n, "far-scene moves at %.2f m/s: every one DONE, a touchdown after (%d/%d)", (double)AUTO_MV_V_H_MAX,
        okDone, n);
  CHECK(okErr == 5, "far-scene moves: error mean <= 15 %% and max <= 25 %% per move (%d/5)", okErr);
  CHECK(okLean == 5, "far-scene moves: lean <= 1.6 deg (max %.1f)", worstLean);
#ifdef AUTO_MV_HOST_VH_OVERRIDE
  CHECK(nDur == n && okDur == nDur, "far-scene moves: never slower than the same seed at 0.20 m/s (%d/%d, worst %+.2f s)",
        okDur, nDur, worstSlower);
#else
  printf("info: far-scene moves: built without AUTO_MV_HOST_VH_OVERRIDE, no comparison at 0.20 m/s\n");
#endif
}

/* testFarMoves' five far-scene moves over 16 seed sets (800 moves, seed bases 5100..20100) at the run's kmax and at
 * kmax 1 (docs/autonomy-a2b.md 20.4): testFarMoves pins one set, and whether a far MOVE ends PROGRESS / SHORT is
 * chaotic per seed, so the gain scale's cost is a rate. Measured (2026-10-14 fixer): kmax 1 not DONE 41 (5.1 %:
 * PROGRESS 19, SHORT 2, refused 20), kmax 2.5 61 (7.6 %: 44, 9, 8); PROGRESS + SHORT of the accepted 2.7 -> 6.7 %.
 * The cause is s falling 2.5 -> 1 at 2 /s in the move's first 0.75 s (the hold at s 1 tracks a far move worse, and
 * the drop disturbs it): with s at 1 before the MOVE the same seeds give 20 not DONE, keeping s through the move 9;
 * at D <= 2.5 m no PROGRESS / SHORT at either kmax. Checks: every run a LAND by the Mac and a touchdown, never
 * KILLED; not DONE <= 10 %, PROGRESS + SHORT <= 8 % of the accepted, at most 4 points over kmax 1. */
static void testFarMovesSeeds(void)
{
  static const int16_t mv[5][2] = {{100, 0}, {-150, 0}, {0, 100}, {0, 150}, {70, 70}};
  const float kSave = simKmax;
  const float km[2] = {simKmax, 1.0f};
  int n[2] = {0, 0}, done[2] = {0, 0}, prog[2] = {0, 0}, shrt[2] = {0, 0}, refused[2] = {0, 0}, bad[2] = {0, 0};
  for (int q = 0; q < (simKmax > 1.0f ? 2 : 1); q++) {
    simKmax = km[q];
    for (uint32_t b = 0; b < 16; b++) {
      for (int k = 0; k < 5; k++) {
        for (uint32_t sd = 0; sd < 10; sd++) {
          scen_t sc = baseScen(5100u + 1000u * b + 10u * (uint32_t)k + sd, -0.5f);
          sc.oneMove = true;
          sc.oneR = mv[k][0];
          sc.oneF = mv[k][1];
          result_t r;
#ifdef AUTO_MV_HOST_VH_OVERRIDE
          autoMvVhMaxOverride = 0.0f;
#endif
          runSim(&sc, &r);
          n[q]++;
          bad[q] += r.killed || !r.touchdown || r.landWhy != AUTO_RS_LAND_CMD || r.endWhy != AUTO_RS_TOUCHDOWN;
          if (r.oneWhy != 0) {
            refused[q]++;
          } else if (r.oneEnd == AUTO_MVE_DONE) {
            done[q]++;
          } else if (r.oneEnd == AUTO_MVE_PROGRESS) {
            prog[q]++;
          } else if (r.oneEnd == AUTO_MVE_SHORT) {
            shrt[q]++;
          }
        }
      }
    }
  }
  simKmax = kSave;
  if (n[1] == 0) {   /* kmax 1 run: the comparison is itself */
    n[1] = n[0];
    done[1] = done[0];
    prog[1] = prog[0];
    shrt[1] = shrt[0];
    refused[1] = refused[0];
    bad[1] = bad[0];
  }
  float notDone[2], failAcc[2];
  for (int q = 0; q < 2; q++) {
    notDone[q] = 100.0f * (float)(n[q] - done[q]) / (float)n[q];
    failAcc[q] = 100.0f * (float)(prog[q] + shrt[q]) / (float)(n[q] - refused[q] > 0 ? n[q] - refused[q] : 1);
    if (q == 1 && !(kSave > 1.0f)) {
      break;
    }
    printf("info: far-scene moves over 16 seed sets at kmax %.2f: %d moves, DONE %d, PROGRESS %d, SHORT %d, refused %d: "
           "not DONE %.1f %%, PROGRESS + SHORT %.1f %% of the accepted; not a LAND by the Mac and a touchdown %d\n",
           q ? 1.0 : (double)kSave, n[q], done[q], prog[q], shrt[q], refused[q], notDone[q], failAcc[q], bad[q]);
  }
  CHECK(bad[0] == 0 && bad[1] == 0, "far-scene moves over 16 seed sets: every run a LAND by the Mac and a touchdown, "
        "never KILLED (%d at kmax %.2f, %d at 1)", bad[0], (double)kSave, bad[1]);
  CHECK(notDone[0] <= 10.0f && failAcc[0] <= 8.0f && notDone[0] - notDone[1] <= 4.0f, "far-scene moves over 16 seed "
        "sets at kmax %.2f: not DONE %.1f %% (<= 10, kmax 1 %.1f, at most +4 points), PROGRESS + SHORT %.1f %% of the "
        "accepted (<= 8)", (double)kSave, notDone[0], notDone[1], failAcc[0]);
}

/* The expansion scale k < 1 (the floor in view): the forward floor holds for
 * k down to AUTO_MV_K_LOW. */
static void testKFloorSim(void)
{
  static const float ks[] = {1.0f, 0.73f, 0.6f, 0.5f};
  for (int j = 0; j < 4; j++) {
    for (int f = 0; f < 2; f++) {
      scen_t sc = baseScen(900u, 1.0f);   /* the wall about 2 m ahead */
      sc.eDmax = 0.0f;
      sc.oneMove = true;
      sc.fwdMax = f ? 2 : 1;   /* the largest forward move the drone accepts at the D then, and 1 cm more */
      sc.kFix = ks[j];
      result_t r;
      runSim(&sc, &r);
      if (f == 1) {
        CHECK(r.oneWhy == (r.oneF > 150 ? AUTO_MVW_RANGE : AUTO_MVW_TOO_CLOSE), "k %.2f: forward %.2f m at D %.2f: "
              "%s (%s)", ks[j], r.oneF * 0.01f, r.d0Obs, r.oneF > 150 ? "RANGE" : "TOO_CLOSE",
              autoMvWhyName((autoMvWhy_t)r.oneWhy));
        continue;
      }
      printf("info: k %.2f, forward %.2f m at D %.2f (k 1: %.2f m left): end %s, %.2f m from the wall at the LAND "
             "command, closest %.2f m\n", ks[j], r.oneF * 0.01f, r.d0Obs, r.d0Obs - r.oneF * 0.01f,
             autoMvEndName(r.oneEnd), r.wallAtLand, r.minWall);
      if (ks[j] >= AUTO_MV_K_LOW) {
        CHECK(r.oneWhy == 0 && r.minWall >= 0.25f && !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN,
              "k %.2f: the largest forward move accepted, never closer than 0.25 m to the wall (%.2f; the floor "
              "0.30 m at k 0.6, 0.50 m and 0.45 until 2026-10-17)", ks[j], r.minWall);
      }
    }
  }
}

/* A depth step: a 0.8 m box face 1.5 m ahead of a wall 3 m ahead. A lateral
 * move is at most 0.5 D; the hold integrates v / D_now, so moving off the box
 * flies further, at most about 0.5 D_far; then the re-view gives the new D. */
static void testDepthStep(void)
{
  static const int16_t mv[] = {100, 75, -75};
  for (int k = 0; k < 3; k++) {
    for (int b = 0; b < 2; b++) {
      scen_t sc = baseScen(1200u, 0.0f);
      sc.eDmax = 0.0f;
      /* no lean error: the scenario is the depth step, at the box's 1.5 m (this seed's 0.3 deg error drifted the
       * drone 0.24 m forward in the climb once the forward hold was off there, docs/autonomy-a2b.md 19.5; before
       * that the take-off's fake expansion kicked it 0.15-0.32 m back) */
      sc.trim = 0.0f;
      sc.box = b == 1;
      sc.oneMove = true;
      sc.oneR = mv[k];
      result_t r;
      runSim(&sc, &r);
      printf("info: depth step: box %d, MOVE right %+d at D %.2f: %s, true %+.2f m (end %s)\n", b, mv[k], r.d0Obs,
             r.oneWhy ? autoMvWhyName((autoMvWhy_t)r.oneWhy) : "accepted", r.mvR, autoMvEndName(r.oneEnd));
      if (b == 1 && k == 0) {
        CHECK(r.oneWhy == AUTO_MVW_TOO_WIDE, "box: right 1.00 m at D 1.5: TOO_WIDE (0.5 D = 0.75 m)");
      } else if (b == 1) {
        CHECK(r.oneWhy == 0 && fabsf(r.mvR) <= AUTO_MV_LAT_D_REL * 3.0f + 0.10f && !r.killed,
              "box: %+d cm off the box onto the 3 m wall: %+.2f m true (<= 0.5 D_far + 0.10 m)", mv[k], r.mvR);
      } else {
        CHECK(r.oneWhy == 0 && fabsf(r.mvR - mv[k] * 0.01f) <= 0.25f * fabsf(mv[k] * 0.01f),
              "no box: %+d cm within 25 %% (%+.2f)", mv[k], r.mvR);
      }
    }
  }
}

/* The take-off without the forward hold in its climb (camera_hold.h fwdOff, docs/autonomy-a2b.md 19.5) against the
 * hold before it: the sim's climb reads 0.3 x v / D of fake expansion (w.cvf), which the old forward hold integrated
 * into a nose-up lean at the HOVER entry and a backward kick; its lean error (+-0.3 deg, trim) is uncorrected in the
 * climb now. 20 seeds facing a wall 2 m ahead. */
static void testTakeoffKick(void)
{
  double sumOld = 0.0, sumNew = 0.0, pOld = 0.0, pNew = 0.0, maxNew = 0.0, sum0Old = 0.0, sum0New = 0.0;
  int n = 0;
  for (uint32_t s = 0; s < 20; s++) {
    for (int z = 0; z < 2; z++) {
      for (int old = 0; old < 2; old++) {
        scen_t sc = baseScen(1500u + 7u * s, 0.0f);
        sc.eDmax = 0.0f;
        sc.oldTakeoffHold = old == 1;
        if (z) {
          sc.trim = 0.0f;
        }
        result_t r;
        runSim(&sc, &r);
        if (!r.hover2Seen) {
          continue;
        }
        if (z) {
          *(old ? &sum0Old : &sum0New) += r.dFwdHover2;
          continue;
        }
        if (old) {
          sumOld += fabs(r.dFwdHover2);
          pOld += fabs(r.holdPHover);
        } else {
          sumNew += fabs(r.dFwdHover2);
          pNew += fabs(r.holdPHover);
          maxNew = fmax(maxNew, fabs(r.dFwdHover2));
          n++;
        }
      }
    }
  }
  printf("info: take-off (20 seeds, lean error +-0.3 deg): forward displacement 2 s into HOVER |mean| %.2f m (the "
         "forward hold in the climb: %.2f m), max %.2f m; the hold's |pitch| at the HOVER entry %.2f deg (%.2f); "
         "without a lean error the mean displacement %+.2f m (%+.2f m: the kick)\n", sumNew / n, sumOld / n, maxNew,
         pNew / n, pOld / n, sum0New / n, sum0Old / n);
  CHECK(n == 20 && fabs(sum0New / n) < 0.5 * fabs(sum0Old / n) && pNew / n < 0.5 * pOld / n,
        "the take-off kick: %+.2f m (old %+.2f), the hold's pitch at the HOVER entry %.2f deg (old %.2f)", sum0New / n,
        sum0Old / n, pNew / n, pOld / n);
  CHECK(maxNew <= 0.40, "a 0.3 deg lean error uncorrected in the climb: at most %.2f m (<= 0.40)", maxNew);
}

/* A frozen view (frames 'ok' but no motion seen): the tighter progress guard
 * or the settle check drops the move; the drift stays bounded. */
static void testFrozenSim(void)
{
  static const int16_t mv[][2] = {{50, 0}, {100, 0}, {0, -100}, {20, 0}, {0, 100}, {0, 150}, {60, 120}};
  for (int k = 0; k < 7; k++) {
    scen_t sc = baseScen(950u, 0.0f);   /* D 3.0 */
    sc.eDmax = 0.0f;
    sc.frozen = true;
    sc.oneMove = true;
    sc.oneR = mv[k][0];
    sc.oneF = mv[k][1];
    result_t r;
    runSim(&sc, &r);
    const float at = hypotf(r.mvR, r.mvF), cmd = hypotf((float)mv[k][0], (float)mv[k][1]) * 0.01f;
    printf("info: frozen view, MOVE %+d %+d: end %s after %.1f s, %.2f m at the LAND command (2 s later), lean max "
           "%.1f deg\n", mv[k][0], mv[k][1], autoMvEndName(r.oneEnd), r.oneEndS, at, r.maxLean);
    CHECK(r.oneEnd == AUTO_MVE_PROGRESS && at <= cmd + 0.35f && !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN,
          "frozen view, MOVE %+d %+d: STOPPED PROGRESS, %.2f m at the LAND command (<= the move + 0.35 m)", mv[k][0],
          mv[k][1], at);
  }
  /* Long forward moves (2026-10-14, docs/autonomy-a2b.md 21.2): the forward margin's behind side grows with 30 % of
   * the move, and the front guard reads the camera too: without the frozen-view rule (AUTO_MV_FROZEN_*) a frozen
   * 3.3 m approach from D 4.7 m (4 m from 5.5) tripped PROGRESS ~10 s in, 3.1-5.0 m out, into the scene ahead in 2
   * of 4 seeds (b6a54da's gain path: the same). With it: PROGRESS 2.6-3.9 s in, <= 1.31 m out. */
  static const struct { float d; int16_t f; } lf[] = {{3.5f, 150}, {4.7f, 250}, {4.7f, 330}, {5.5f, 400}, {6.0f, 400}};
  int nLong = 0, okLong = 0;
  float worstOut = 0.0f, worstWall = 9.0f;
  for (int k = 0; k < 5; k++) {
    for (uint32_t sd = 0; sd < 4; sd++) {
      scen_t sc = baseScen(960u + sd, 6.0f - lf[k].d);
      sc.room = 6.0f;
      sc.eDmax = 0.0f;
      sc.frozen = true;
      sc.oneMove = true;
      sc.macCap = true;
      sc.oneF = lf[k].f;
      result_t r;
      runSim(&sc, &r);
      if (r.oneWhy != 0) {
        continue;   /* (the take-off drift took D under the 4 m floor: refused, nothing flown) */
      }
      nLong++;
      const float at = hypotf(r.mvR, r.mvF);
      okLong += r.oneEnd == AUTO_MVE_PROGRESS && r.oneEndS < 5.0f && at <= 1.5f && r.minWall >= 1.0f && !r.killed &&
                r.endWhy == AUTO_RS_TOUCHDOWN;
      worstOut = fmaxf(worstOut, at);
      worstWall = fminf(worstWall, r.minWall);
      if (sd == 0) {
        printf("info: frozen view, D %.1f m, MOVE forward %d (sent %d): end %s after %.1f s, %.2f m at the LAND command, "
               "the wall %.2f m at the nearest, lean max %.1f deg\n", lf[k].d, lf[k].f, r.oneF,
               autoMvEndName(r.oneEnd), r.oneEndS, at, r.minWall, r.maxLean);
      }
    }
  }
  CHECK(nLong >= 15 && okLong == nLong, "frozen view on long forward moves (D 3.5-6 m, 1.5-4 m): PROGRESS within 5 s, "
        "<= 1.5 m out, the scene ahead >= 1.0 m (%d of %d; %.2f m out, %.2f m from the scene at the worst)", okLong,
        nLong, worstOut, worstWall);
}

/* 0.20 m take-off + 0.50 + 0.50 m up, LAND 1 s later: as slow as from 1.20 m. */
static void testHighLandSim(void)
{
  static const float offs[] = {0.0f, 0.01f, -0.01f};
  float worst = 0.0f, durMax = 0.0f, durSum = 0.0f;
  int ok = 0, n = 0, fast = 0;
  for (int a = 0; a < 3; a++) {
    for (int sd = 0; sd < 3; sd++) {
      scen_t sc = baseScen(1000u + (uint32_t)sd, 0.0f);
      sc.eDmax = 0.0f;
      sc.toCm = 20;
      sc.nUp = 2;
      sc.ups[0] = 50;
      sc.ups[1] = 50;
      sc.firstAfterS = 3.0f;
      sc.landAfterS = 1.0f;
      sc.camVzOff = offs[a];
      result_t r;
      runSim(&sc, &r);
      worst = fmaxf(worst, r.impactV);
      n++;
      ok += r.refused == 0 && r.touchdown && !r.killed && r.impactV <= 0.45f && r.landWhy == AUTO_RS_LAND_CMD;
      fast += r.landFast;
      durMax = fmaxf(durMax, r.landDurS);
      durSum += r.landDurS;
    }
  }
  printf("info: 0.20 m + 0.50 + 0.50 m, LAND: impact max %.2f m/s (591f176: up to 0.68 with c -0.01 m/s); fast LAND "
         "%d/%d, LAND to DONE mean %.2f max %.2f s\n", worst, fast, n, durSum / (float)n, durMax);
  CHECK(ok == n, "0.20 m take-off moved up to 1.20 m, LAND: both moves accepted, touchdown <= 0.45 m/s (%d/%d, %.2f)",
        ok, n, worst);
}

/* Moves down late in a long hover, the estimate reading high (c +0.01 m/s):
 * the floor grows with the air time, the drone never reaches the floor in HOVER. */
static void testDownLate(void)
{
  static const float waits[] = {3.0f, 30.0f, 50.0f};
  for (int b = 0; b < 3; b++) {
    scen_t sc = baseScen(1100u, 0.0f);
    sc.eDmax = 0.0f;
    sc.toCm = 100;
    sc.nUp = 2;
    sc.ups[0] = -50;
    sc.ups[1] = -15;
    sc.firstAfterS = waits[b];
    sc.landAfterS = 3.0f;
    sc.camVzOff = 0.01f;
    result_t r;
    runSim(&sc, &r);
    printf("info: down 0.50 then 0.15 from 1.00 m, the first %.0f s into the wait (c +0.01 m/s): refused %d, lowest "
           "true height in HOVER %.2f m, LAND %s\n", waits[b], r.refused, r.hMinHover, autoReasonName(r.landWhy));
    CHECK(r.hMinHover >= 0.15f && !r.groundInHover && !r.killed && r.touchdown,
          "first move down %.0f s into the wait: never below 0.15 m true in HOVER (%.2f; 591f176: 0.04 m at 50 s)",
          waits[b], r.hMinHover);
  }
}

/* A STOP during the re-track bob after a turn: the bob runs on, no LAND. */
/* Faster turns (user decision 2026-10-13, evening): YAW 90 / 180 / -45 at
 * 30 deg/s, 60 deg/s^2. The hold and the observer drop frames turning faster
 * than 0.35 rad/s (20 deg/s); in a commanded turn their gate is 45 deg/s
 * (camera_hold.h camHoldYawOk, cam_vz_obs.h camVzObsSetTurn). With the old
 * gates in the turn the hold coasts (the turn pauses) and the observer has no
 * frame for > 2 s (CAM_LOST; HOVER_NO_CAM until 2026-10-15). */
static void testFastTurn(void)
{
  const int16_t degs[] = {90, 180, -45};
  int ok = 0;
  float worstDrift = 0.0f, worstErr = 0.0f, rateMax = 0.0f;
  for (int k = 0; k < 3; k++) {
    for (int i = 0; i < 4; i++) {
      scen_t sc = baseScen(1400u + (uint32_t)(10 * k + i), 1.0f);
      sc.turnDeg = degs[k];
      result_t r;
      runSim(&sc, &r);
      const float tProf = (fabsf((float)degs[k]) / AUTO_MV_YAW_RATE_DPS) + AUTO_MV_YAW_RATE_DPS / AUTO_MV_YAW_ACC_DPS2;
      const bool good = r.turnEnded && r.turnEnd == AUTO_MVE_DONE && !r.turnPaused &&
                        fabsf(r.turnS - tProf) < 0.05f && r.turnFramesEng == r.turnFrames && r.turnFrames > 10 &&
                        fabsf(r.turnYawErr) <= 0.05f * fabsf((float)degs[k]) && r.landWhy == AUTO_RS_LAND_CMD &&
                        r.endWhy == AUTO_RS_TOUCHDOWN && !r.killed;
      ok += good;
      worstDrift = fmaxf(worstDrift, r.turnDrift);
      worstErr = fmaxf(worstErr, fabsf(r.turnYawErr));
      rateMax = fmaxf(rateMax, r.turnRateMax);
      if (i == 0 || !good) {
        printf("info: YAW %d at %.0f deg/s: turn %.2f s (profile %.2f), paused %d, hold frames engaged %d of %d, end %s, "
               "drift %.2f m, heading error %+.1f deg, rate max %.1f deg/s, LAND %s\n", degs[k],
               (double)AUTO_MV_YAW_RATE_DPS, r.turnS, tProf, r.turnPaused, r.turnFramesEng, r.turnFrames,
               autoMvEndName(r.turnEnd), r.turnDrift, r.turnYawErr, r.turnRateMax, autoReasonName(r.landWhy));
      }
    }
  }
  printf("info: YAW 90 / 180 / -45 at %.0f deg/s over 12 sorties: drift during the turn and its dwell / bob <= %.2f m, "
         "heading error <= %.1f deg, heading rate <= %.1f deg/s\n", (double)AUTO_MV_YAW_RATE_DPS, worstDrift, worstErr,
         rateMax);
  CHECK(ok == 12, "YAW 90 / 180 / -45 at 30 deg/s: the profile's time, never paused, the hold engaged on every turn "
        "frame, DONE, the heading within 5 %%, LAND by the Mac (%d/12); drift <= %.2f m, error <= %.1f deg, "
        "rate <= %.1f deg/s", ok, worstDrift, worstErr, rateMax);
  /* the gates of before the decision in a 30 deg/s turn: both (the hold coasts, the turn pauses: no faster than
   * before), and the observer's alone (the hold's widened: no accepted frame for > 2 s, *_NO_CAM) */
  int slow = 0, noCam = 0;
  for (int k = 0; k < 2; k++) {
    for (int g = 0; g < 2; g++) {
      scen_t sc = baseScen(1450u + (uint32_t)(2 * k + g), 1.0f);
      sc.turnDeg = degs[k];
      sc.oldHoldGate = g == 0;
      sc.oldObsGate = true;
      result_t r;
      runSim(&sc, &r);
      const float tProf = (fabsf((float)degs[k]) / AUTO_MV_YAW_RATE_DPS) + AUTO_MV_YAW_RATE_DPS / AUTO_MV_YAW_ACC_DPS2;
      if (g == 0) {
        slow += r.turnPaused && r.turnS > 1.3f * tProf;
      } else {
        noCam += (r.evMask & (1u << AUTO_EV_CAM_LOST)) != 0u || r.landWhy == AUTO_RS_SINK ||
                 (r.landWhy == AUTO_RS_VZ && r.vzLandWhy == (uint8_t)VZ_R_HOLD_NO_CAM);
      }
      printf("info: YAW %d at 30 deg/s, %s: turn %.2f s (profile %.2f), paused %d, hold frames engaged %d of %d, "
             "rate max %.1f deg/s, end %s, LAND %s%s%s\n", degs[k],
             g == 0 ? "both gates 20 deg/s" : "the observer's 20 deg/s", r.turnS, tProf, r.turnPaused, r.turnFramesEng,
             r.turnFrames, r.turnRateMax, autoMvEndName(r.turnEnd), autoReasonName(r.landWhy),
             r.landWhy == AUTO_RS_VZ ? " " : "", r.landWhy == AUTO_RS_VZ ? vzReasonName((vzReason_t)r.vzLandWhy) : "");
    }
  }
  CHECK(slow == 2 && noCam == 2, "the reason for the turn gates: with both at 20 deg/s a 30 deg/s turn pauses and "
        "takes > 1.3 x its profile (%d/2); with the observer's alone at 20 deg/s it loses the camera height (CAM_LOST; %d/2)", slow, noCam);
}

static void testStopInBob(void)
{
  int ok = 0;
  for (int i = 0; i < 4; i++) {
    scen_t sc = baseScen(1300u + (uint32_t)i, 1.0f);
    sc.stopInBob = true;
    result_t r;
    runSim(&sc, &r);
    ok += r.bobSeen && r.stopAcked && !r.landedSoon && r.bobDone && r.landWhy == AUTO_RS_LAND_CMD &&
          r.endWhy == AUTO_RS_TOUCHDOWN;
    if (i == 0) {
      printf("info: STOP in the re-track bob: acked %d, landed within 1 s %d, the turn DONE %d, LAND %s\n",
             r.stopAcked, r.landedSoon, r.bobDone, autoReasonName(r.landWhy));
    }
  }
  CHECK(ok == 4, "YAW 90, STOP during its re-track bob: OK, still hovering, the turn ends DONE, LAND by the Mac (%d/4; "
        "591f176 landed HOVER_NO_CAM at once)", ok);
}


/* An in-air contact (docs/autonomy-a2.md 12.8; LAND COLLISION from 2026-10-13, hold and report since 2026-10-15,
 * docs/autonomy-a2b.md 22): in the hover at 1.00 / 1.20 m, the speed estimate off by e -1 / 0 / +1 m/s at the contact
 * (the accelerometer's pulse, which the hush keeps out of the estimate, and the observer's camVz for the jolt's 1 s of
 * frames), one contact or flight 1's train; during a forward MOVE of 60 cm; through a MOVE up, a MOVE down and the
 * re-track bob. The state machine reports CONTACT (the hush, the motion stopped or the hold re-anchored), never
 * lands for it nor KILLs; the Mac lands 3 s after it (it saw it). Checked: the event, the hush, no LAND before the
 * Mac's but the controller's runaway guard (EST_RANGE: pulses of a train beyond the hush chain took the height
 * estimate out of its range; a physical fallback, counted), the true height's rise from the contact bounded (the
 * hush's brake, the speed from before the contact, the open window: vert_ctrl.h VZ_HUSH_OPEN_S; 0.41 m when the
 * camera's speed was used at once; the old blind LAND's "no climb" was 0.02 m), a touchdown <= 1.1 m/s. The
 * persistent camera error (the blind LAND's worst case) is reported, not
 * checked: a camera speed that stays wrong moves the hold at that speed, as any camera offset (risk 3 of
 * docs/autonomy-a2b.md 22): the Mac's photo after a contact is the check. */
/* The Mac's LAND, or the controller's runaway guard (EST_RANGE) before it */
static bool contactEndOk(const result_t *r, int *est)
{
  const bool e = r->landWhy == AUTO_RS_VZ && r->vzLandWhy == (uint8_t)VZ_R_EST_RANGE;
  *est += e;
  return r->landWhy == AUTO_RS_LAND_CMD || e;
}

static void testContactSim(void)
{
  int n = 0, ok = 0, est = 0;
  float worstV = 0.0f, worstClimb = 0.0f, worstTravel = 0.0f;
  for (int h = 0; h < 2; h++) {
    for (int ie = 0; ie < 3; ie++) {
      for (int tr = 0; tr < 2; tr++) {
        for (int k = 0; k < 3; k++) {
          scen_t sc = {.seed = 900u + (uint32_t)(k + 3 * (tr + 2 * (ie + 3 * h))), .x0 = 0.2f * (float)(k - 1),
                       .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.10f, .trim = 0.3f, .camLossAtMoveS = -1.0f,
                       .battDropAtS = -1.0f, .toCm = h ? 120 : 100, .contact = true, .contactAtS = 8.0f + (float)k,
                       .contactE = -1.0f + (float)ie, .contactEPersistS = 1.0f, .contactTrain = tr != 0,
                       .macLandAfterContactS = 3.0f};
          result_t r;
          runSim(&sc, &r);
          n++;
          const float climb = r.hMaxAfterContact2 - r.hAtContact;
          const bool good = r.contactDone && r.contacts >= 1 && r.hushSeen && !r.killed && contactEndOk(&r, &est) &&
                            r.endWhy == AUTO_RS_TOUCHDOWN && r.touchdown && r.impactV <= 1.1f && climb <= 0.10f;
          ok += good;
          if (!good || (h == 0 && ie == 2 && tr == 1 && k == 0)) {
            printf("info: contact at %.2f m (true %.2f), e %+.0f m/s, %s: CONTACT events %d, hush %d, LAND %s%s%s%s, "
                   "end %s, touchdown %.2f m/s, rise %.3f m, travel %.2f m\n", h ? 1.20f : 1.00f, r.hAtContact,
                   sc.contactE, tr ? "train" : "one contact", r.contacts, r.hushSeen, autoReasonName(r.landWhy),
                   r.landWhy == AUTO_RS_VZ ? " " : "", r.landWhy == AUTO_RS_VZ ? vzReasonName((vzReason_t)r.vzLandWhy) : "",
                   r.killed ? " KILLED" : "", autoReasonName(r.endWhy), r.impactV, climb, r.travelLand);
          }
          worstV = fmaxf(worstV, r.impactV);
          worstClimb = fmaxf(worstClimb, climb);
          worstTravel = fmaxf(worstTravel, r.travelLand);
        }
      }
    }
  }
  printf("info: CONTACT in the hover (%d sorties, 1.00 / 1.20 m, e -1..+1 m/s for 1 s, trim error 0.3 deg), the "
         "Mac's LAND 3 s later: touchdown max %.2f m/s, rise from the contact max %.3f m, horizontal travel from it "
         "max %.2f m; the controller's EST_RANGE first in %d\n", n, worstV, worstClimb, worstTravel, est);
  CHECK(ok == n, "contact in the hover: CONTACT and the hush, no LAND for it, the Mac's LAND (or EST_RANGE): "
        "touchdown <= 1.1 m/s, rise <= 0.10 m (%d/%d; worst %.2f m/s, %.3f m)", ok, n, worstV, worstClimb);
  /* the persistent camera error (info) */
  for (int ie = 0; ie < 3; ie += 2) {
    scen_t sc = {.seed = 990u + (uint32_t)ie, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.10f, .trim = 0.3f,
                 .camLossAtMoveS = -1.0f, .battDropAtS = -1.0f, .toCm = 100, .contact = true, .contactAtS = 8.0f,
                 .contactE = -1.0f + (float)ie, .macLandAfterContactS = 3.0f};
    result_t r;
    runSim(&sc, &r);
    printf("info: contact at 1.00 m with the camera speed wrong by %+.0f m/s for good: rise %.2f m, LAND %s, end %s, "
           "killed %d, touchdown %.2f m/s (a camera fault the Mac's photo has to catch)\n", sc.contactE,
           r.hMaxAfterContact2 - r.hAtContact, autoReasonName(r.landWhy), autoReasonName(r.endWhy), r.killed,
           r.impactV);
  }

  /* during a MOVE forward 60 */
  int okM = 0;
  est = 0;
  for (int k = 0; k < 6; k++) {
    scen_t sc = {.seed = 950u + (uint32_t)k, .x0 = 0.0f, .y0 = 0.5f, .psi0 = 1.5708f, .eDmax = 0.10f, .trim = 0.3f,
                 .camLossAtMoveS = -1.0f, .battDropAtS = -1.0f, .toCm = 100, .contact = true,
                 .contactE = k % 2 ? 0.85f : -0.85f, .contactEPersistS = 1.0f, .contactTrain = k >= 3,
                 .contactInMove = true, .macLandAfterContactS = 3.0f};
    result_t r;
    runSim(&sc, &r);
    const float climb = r.hMaxAfterContact2 - r.hAtContact;
    const bool good = r.contactDone && r.contacts >= 1 && !r.killed && contactEndOk(&r, &est) &&
                      r.mvStoppedAtLand && (r.evMask & (1u << AUTO_EV_CONTACT)) && r.endWhy == AUTO_RS_TOUCHDOWN &&
                      r.impactV <= 1.1f && climb <= 0.10f;
    okM += good;
    if (!good || k == 0) {
      printf("info: contact 1.5 s into MOVE forward 60, e %+.2f m/s, %s: CONTACT events %d, LAND %s, motion stopped %d, "
             "end %s, touchdown %.2f m/s, rise %.3f m, travel %.2f m\n", sc.contactE,
             k >= 3 ? "train" : "one contact", r.contacts, autoReasonName(r.landWhy), r.mvStoppedAtLand,
             autoReasonName(r.endWhy), r.impactV, climb, r.travelLand);
    }
  }
  CHECK(okM == 6, "contact during a MOVE: CONTACT, the motion stopped, no LAND for it, the Mac's LAND (or EST_RANGE "
        "%d): a soft touchdown, rise <= 0.10 m (%d/6)", est, okM);

  /* During the hover's vertical motions (the verification of 1c2a43f): a contact through a MOVE up 50 from 0.70 m,
   * through a MOVE down 50 from 1.00 m (its braking, its settle and the hover after it at 0.50 m), and through the
   * re-track bob of a YAW 90 at 1.00 m; e -0.85 / 0 / +0.85 m/s for 1 s, one contact or the train. The motion stops
   * where it is (the bob ends); what rises is the drone's own upward speed at the contact, braked by the hush. */
  static const struct {
    int kind, n;
    float t0, dt, climbMax, impMax;
    const char *name;
  } vm[3] = {
    /* rises measured 0.036 / 0.126 / 0.095 m (after the MOVE down capQ is the 1.00 m hover's, a little over the
     * 0.50 m one's: the open window's +10 % from a camera reading a sink climbs a few cm) */
    {1, 30, 0.2f, 0.15f, 0.15f, 1.3f, "MOVE up 50 from 0.70 m"},
    {2, 30, 0.2f, 0.30f, 0.15f, 1.1f, "MOVE down 50 from 1.00 m (and the hover after it)"},
    {3, 40, 0.1f, 0.18f, 0.15f, 1.2f, "the re-track bob of a YAW 90 at 1.00 m"},
  };
  for (int m = 0; m < 3; m++) {
    int okV = 0, nV = 0;
    float wV = 0.0f, wC = 0.0f, wAt = 0.0f;
    est = 0;
    for (int it = 0; it < vm[m].n; it++) {
      scen_t sc = baseScen(1950u + 100u * (uint32_t)m + (uint32_t)it, m == 2 ? 1.0f : 0.5f);
      sc.toCm = m == 0 ? 70 : 100;
      sc.contact = true;
      sc.contactE = 0.85f * (float)((it % 3) - 1);
      sc.contactEPersistS = 1.0f;
      sc.contactTrain = (it % 2) != 0;
      sc.macLandAfterContactS = 3.0f;
      if (m < 2) {
        sc.contactInMove = true;
        sc.contactUpCm = m == 0 ? 50 : -50;
        sc.contactInMoveS = vm[m].t0 + vm[m].dt * (float)it;
      } else {
        sc.contactInBob = true;
        sc.bobAtS = vm[m].t0 + vm[m].dt * (float)it;
      }
      result_t r;
      runSim(&sc, &r);
      nV++;
      const float climb = r.hMaxAfterContact2 - r.hAtContact;
      const bool good = r.contactDone && r.contacts >= 1 && !r.killed && contactEndOk(&r, &est) &&
                        r.endWhy == AUTO_RS_TOUCHDOWN && r.touchdown && r.impactV <= vm[m].impMax &&
                        climb <= vm[m].climbMax;
      okV += good;
      wV = fmaxf(wV, r.impactV);
      if (climb > wC) {
        wC = climb;
        wAt = m < 2 ? sc.contactInMoveS : sc.bobAtS;
      }
      if (!good) {
        printf("info: contact %.2f s into %s, e %+.2f m/s, %s: contact %d (events %d), LAND %s%s, end %s, touchdown "
               "%.2f m/s, rise %.3f m\n", m < 2 ? sc.contactInMoveS : sc.bobAtS, vm[m].name, sc.contactE,
               sc.contactTrain ? "train" : "one contact", r.contactDone, r.contacts, autoReasonName(r.landWhy),
               r.killed ? " KILLED" : "", autoReasonName(r.endWhy), r.impactV, climb);
      }
    }
    printf("info: CONTACT during %s (%d sorties): rise max %.3f m (contact %.2f s into it), touchdown max %.2f m/s, "
           "the controller's EST_RANGE first in %d\n", vm[m].name, nV, wC, wAt, wV, est);
    CHECK(okV == nV, "contact during %s: CONTACT, no LAND for it, the Mac's LAND (or EST_RANGE): a touchdown <= %.1f "
          "m/s, rise <= "
          "%.2f m (%d/%d; worst %.2f m/s, %.3f m)", vm[m].name, vm[m].impMax, vm[m].climbMax, okV, nV, wV, wC);
  }
}

/* Every LAND after a contact is the blind one (user decision (B) 2026-10-15, docs/autonomy-a2b.md 22.9 and 22.10): a
 * CONTACT 5 s into the wait (one contact, the camera's speed off by e -0.85 / 0 / +0.85 m/s for its 1 s of frames), no
 * LAND for it, then later (the contact's window long over: the camera height back, the hold closed on it) each kind of
 * LAND: the Mac's, BATT_LOW, VBAT_DIP, the link lost (HB_LOST), the camera out for good (SINK), the camera out then back
 * during the SINK (RESUME, then the Mac's LAND), the phone's stick, three loop gaps, a session change, the controller's
 * own (EST_RANGE, white-box; HOLD_TIMEOUT, its backstop shortened), from 1.00 and 1.20 m. Checked: the LAND's reason,
 * every controller LAND blind from its first step and none with the hold blocked (the hold keeps the drone at its
 * anchor down to the touchdown), a touchdown <= 1.1 m/s (the blind LAND's "~1 m/s", the controller's guard LANDs too
 * since 22.10: EST_RANGE fell from landStart at once until then, 1.51 m/s after 20 s and 1.84 after 100 s), the true
 * height never above its value at the LAND's start by more than 0.05 m (no climb), never KILLED, never LAND_TIMEOUT, and
 * the floor: the motors on with the true height under 0.12 m (a float in ground effect that never touches counts) at
 * most the flat part without the drift allowance + 3.0 s (its floor rule ends the flat part from there, then the
 * ceiling falls to the touchdown thrust in ~2.3 s); the camera out for good (no speed to see the floor by, no bump in
 * the sim's smooth ground effect) at most its whole flat part + 3.0 s. A SINK whose camera comes back resumes, unless
 * it began within 0.12 m of the floor (the contact's camera error pushed the hold down there): then it lands (a blind
 * SINK does not resume at rest). Without a contact the same LANDs are the camera's, never blind (their touchdowns and
 * floor times printed to compare). Two sets: the LAND 15 s after the contact (camera offsets at random, 2 seeds), and
 * 60 / 100 s into a 120 s wait (the drift allowance +5.1 / +8.5 s) with the camera's speed offset -0.004 / +0.004 m/s
 * (the estimate drifts 0.24-0.40 m against the truth), the verification of e51f4b7: 5-16 s of float then. */
typedef struct {
  int kind;
  autoReason_t why;
  uint8_t vzWhy;
  const char *name;
} lateKind_t;
static const lateKind_t LATE_KINDS[] = {
  {1, AUTO_RS_LAND_CMD, 0, "the Mac's LAND"},
  {2, AUTO_RS_BATT_LOW, 0, "BATT_LOW"},
  {3, AUTO_RS_VBAT_DIP, 0, "VBAT_DIP"},
  {4, AUTO_RS_HB_LOST, 0, "the link lost (HB_LOST)"},
  {5, AUTO_RS_SINK, 0, "the camera out for good (SINK)"},
  {6, AUTO_RS_LAND_CMD, 0, "a SINK resumed, then the Mac's LAND"},
  {7, AUTO_RS_APP_INPUT, 0, "the phone's stick (APP_INPUT)"},
  {8, AUTO_RS_LOOP_GAP, 0, "three loop gaps (LOOP_GAP)"},
  {9, AUTO_RS_SESSION, 0, "a session change (SESSION)"},
  {10, AUTO_RS_VZ, (uint8_t)VZ_R_EST_RANGE, "the controller's EST_RANGE"},
  {11, AUTO_RS_VZ, (uint8_t)VZ_R_HOLD_TIMEOUT, "the controller's HOLD_TIMEOUT"},
};

/* One late-LAND sortie: lateAtS 0 is the 15 s set (camera offset at random), else the long set (a 120 s wait). */
static void lateRun(const lateKind_t *k, int h, int ie, int sd, float lateAtS, float camOff, bool contact, uint32_t seed,
                    result_t *r)
{
  scen_t sc = baseScen(seed, 1.0f);
  sc.toCm = h ? 120 : 100;
  sc.contact = contact;
  sc.contactAtS = 5.0f;
  sc.contactE = 0.85f * (float)(ie - 1);
  sc.contactEPersistS = 1.0f;
  sc.lateLand = k->kind;
  sc.lateAtS = lateAtS > 0.0f ? lateAtS : 20.0f;
  sc.lateCamS = 6.0f;   /* kind 6: the SINK at 5 s, the camera back 1 s into it */
  if (lateAtS > 0.0f) {
    sc.waitMs = 120000;   /* else the Mac's LAND at the wait's end (60 s) comes first */
    sc.camVzOff = camOff;
  }
  if (k->kind == 11) {
    sc.holdMaxS = lateAtS > 0.0f ? lateAtS + 3.0f : 25.0f;   /* HOLD_TIMEOUT (no trigger needed) */
    sc.lateAtS = sc.holdMaxS + 60.0f;
  }
  (void)sd;
  runSim(&sc, r);
}

/* The checks of one late-LAND sortie with its contact; prints the failure. Accepted besides the kind's own LAND: the
 * controller's K_RANGE first with the drone at the floor (the contact's camera error, e +0.85 m/s for 1 s, pushed the
 * hold down there: Khat at its floor, the hold cannot hold it down), and after a RESUME the controller's own guard (an
 * estimate drifted out of range). A SINK that began within 0.25 m of the floor lands without a resume (at rest there
 * by the time the camera is back 0.5 s). BATT_LOW long into the wait: <= 1.2 m/s (the step sag at the end of the
 * battery: the reference is fixed at the LAND's start while the battery falls ~0.2 V more, 1.11-1.13 m/s, the same
 * before 22.10). */
static bool lateGood(const lateKind_t *k, const result_t *r, const char *what, bool longSet, int *kFirst)
{
  const bool sink0 = k->kind == 6 && r->resumes == 0 && r->hLastLand0 < 0.25f;
  const autoReason_t want = sink0 ? AUTO_RS_SINK : k->why;
  const bool own = r->landWhy == want && (k->vzWhy == 0 || r->vzLandWhy == k->vzWhy);
  const bool kAtFloor = r->landWhy == AUTO_RS_VZ && r->vzLandWhy == VZ_R_K_RANGE && r->hLastLand0 < 0.12f &&
                        r->resumes == 0;
  const bool guardAfterResume = k->kind == 6 && r->resumes >= 1 && r->landWhy == AUTO_RS_VZ && r->vzLands == 2;
  *kFirst += !own && kAtFloor;
  const float impMax = longSet && k->kind == 2 ? 1.2f : 1.1f;
  const float floorMax = (k->kind == 5 ? r->lastLandFlat : r->lastLandBase) + 3.0f;
  const bool good = r->contactDone && r->contacts >= 1 && (own || kAtFloor || guardAfterResume) && r->vzLands >= 1 &&
                    r->vzLandsBlind == r->vzLands && r->vzLandsCollided == 0 && !r->blockedInAir && !r->killed &&
                    r->endWhy == AUTO_RS_TOUCHDOWN && r->touchdown && r->impactV <= impMax && r->hRiseLand <= 0.05f &&
                    r->lastLandBase > 0.0f && r->nearFloorOnS <= floorMax &&
                    (k->kind != 6 || sink0 || kAtFloor || (r->resumes >= 1 && r->vzLands == 2));
  if (!good) {
    printf("info: late LAND %s, %s: LAND %s%s%s (want %s), controller LANDs %d (blind %d, hold blocked %d, blocked in "
           "the air %d), resumes %d (at %.3f m), end %s%s, touchdown %.2f m/s, rise in LAND %.3f m, motors on under "
           "0.12 m %.2f s (<= %.2f), began at %.3f m\n", k->name, what, autoReasonName(r->landWhy),
           r->landWhy == AUTO_RS_VZ ? " " : "", r->landWhy == AUTO_RS_VZ ? vzReasonName((vzReason_t)r->vzLandWhy) : "",
           autoReasonName(want), r->vzLands, r->vzLandsBlind, r->vzLandsCollided, r->blockedInAir, r->resumes,
           r->resumeH, autoReasonName(r->endWhy), r->killed ? " KILLED" : "", r->impactV, r->hRiseLand,
           r->nearFloorOnS, floorMax, r->hLastLand0);
  }
  return good;
}

static void testLateBlindLands(void)
{
  const int nk = (int)(sizeof(LATE_KINDS) / sizeof(LATE_KINDS[0]));
  for (int set = 0; set < 3; set++) {
    const float lateAtS = set == 0 ? 0.0f : set == 1 ? 60.0f : 100.0f;
    int nAll = 0, okAll = 0, noContactBlind = 0;
    float wVAll = 0.0f, wRiseAll = 0.0f, wNearAll = 0.0f, wNear0All = 0.0f, wNearAirAll = 0.0f;
    for (int i = 0; i < nk; i++) {
      const lateKind_t *k = &LATE_KINDS[i];
      int n = 0, ok = 0, resumed = 0, landedLow = 0, n0 = 0, kFirst = 0;
      float wV = 0.0f, wRise = 0.0f, wTravel = 0.0f, wV0 = 0.0f, wRise0 = 0.0f, wNear = 0.0f, wNear0 = 0.0f;
      float wBounce = 0.0f, wFloorOff = 0.0f, wBounce0 = 0.0f, wFloorOff0 = 0.0f, resLow = 9.0f, wNearAir = 0.0f;
      for (int h = 0; h < 2; h++) {
        for (int ie = 0; ie < 3; ie++) {
          for (int sd = 0; sd < 2; sd++) {
            /* the 15 s set: 2 seeds, the camera offset at random (baseScen); the long sets: offset -0.004 / +0.004 */
            const float camOff = set == 0 ? 0.0f : (sd ? 0.004f : -0.004f);
            const uint32_t seed = set == 0 ? 6100u + 100u * (uint32_t)i + (uint32_t)(sd + 2 * (ie + 3 * h))
                                           : 9100u + 1000u * (uint32_t)set + 100u * (uint32_t)i +
                                                 (uint32_t)(sd + 2 * (ie + 3 * h));
            result_t r;
            lateRun(k, h, ie, sd, lateAtS, camOff, true, seed, &r);
            n++;
            char what[64];
            snprintf(what, sizeof(what), "%.2f m, e %+.2f, %s", h ? 1.20f : 1.00f, 0.85f * (float)(ie - 1),
                     set == 0 ? "15 s after it" : set == 1 ? "60 s into the wait" : "100 s into the wait");
            ok += lateGood(k, &r, what, set != 0, &kFirst);
            resumed += r.resumes > 0;
            landedLow += k->kind == 6 && r.resumes == 0;
            if (r.resumes > 0) {
              resLow = fminf(resLow, r.resumeH);
            }
            wV = fmaxf(wV, r.impactV);
            wRise = fmaxf(wRise, r.hRiseLand);
            wTravel = fmaxf(wTravel, hypotf(r.landR, r.landF));
            wBounce = fmaxf(wBounce, r.hAfterFloor);
            wFloorOff = fmaxf(wFloorOff, r.floorToOffS);
            wNear = fmaxf(wNear, r.nearFloorOnS);
            if (r.hLastLand0 >= 0.25f) {
              wNearAir = fmaxf(wNearAir, r.nearFloorOnS);   /* a LAND that began in the air */
            }
            /* the same LAND without a contact: the camera's LAND (info) */
            if (ie == 1) {
              result_t r0;
              lateRun(k, h, ie, sd, lateAtS, camOff, false, seed, &r0);
              n0++;
              wV0 = fmaxf(wV0, r0.impactV);
              wRise0 = fmaxf(wRise0, r0.hRiseLand);
              wBounce0 = fmaxf(wBounce0, r0.hAfterFloor);
              wFloorOff0 = fmaxf(wFloorOff0, r0.floorToOffS);
              wNear0 = fmaxf(wNear0, r0.nearFloorOnS);
              if (r0.vzLandsBlind != 0 || r0.landWhy != k->why) {
                printf("info: without a contact, %s: LAND %s, blind LANDs %d\n", k->name, autoReasonName(r0.landWhy),
                       r0.vzLandsBlind);
              }
              noContactBlind += r0.vzLandsBlind;
            }
          }
        }
      }
      printf("info: a LAND %s after a CONTACT, %s (%d sorties): touchdown max %.2f m/s, rise in LAND max %.3f m, "
             "horizontal travel in the LAND max %.2f m, after the floor up to %.3f m and %.2f s to the motors off, the "
             "motors on under 0.12 m up to %.2f s (%.2f s in a LAND that began 0.25 m up or more); the controller's "
             "K_RANGE first, the hold at the floor, %d; without the contact (the camera's LAND, %d) %.2f m/s, %.3f m, "
             "%.3f m, %.2f s, %.2f s\n",
             set == 0 ? "15 s" : set == 1 ? "60 s into the wait" : "100 s into the wait", k->name, n, wV, wRise, wTravel,
             wBounce, wFloorOff, wNear, wNearAir, kFirst, n0, wV0, wRise0, wBounce0, wFloorOff0, wNear0);
      if (k->kind == 6) {
        printf("info: ... %d resumed (the lowest at %.3f m true), %d began within 0.25 m of the floor and landed\n",
               resumed, resumed ? resLow : -1.0f, landedLow);
      }
      CHECK(ok == n, "a LAND %s after a contact, %s: that LAND, blind from its first step, the hold not blocked, a "
            "touchdown <= 1.1 m/s, no climb, the motors on under 0.12 m within the flat part's base + 3 s (%d/%d; worst "
            "%.2f m/s, rise %.3f m, %.2f s under 0.12 m)", set == 0 ? "15 s" : set == 1 ? "60 s into the wait" :
            "100 s into the wait", k->name, ok, n, wV, wRise, wNear);
      nAll += n;
      okAll += ok;
      wVAll = fmaxf(wVAll, wV);
      wRiseAll = fmaxf(wRiseAll, wRise);
      wNearAll = k->kind == 5 ? wNearAll : fmaxf(wNearAll, wNear);
      wNearAirAll = k->kind == 5 ? wNearAirAll : fmaxf(wNearAirAll, wNearAir);
      wNear0All = k->kind == 5 ? wNear0All : fmaxf(wNear0All, wNear0);
    }
    printf("info: every LAND kind %s after a contact (%d sorties, %d as wanted): touchdown max %.2f m/s, rise in LAND "
           "max %.3f m, the motors on under 0.12 m up to %.2f s (%.2f s from 0.25 m up or more) but the camera out for "
           "good (the camera's LAND: %.2f s)\n", set == 0 ? "15 s" : set == 1 ? "60 s into the wait" :
           "100 s into the wait", nAll, okAll, wVAll, wRiseAll, wNearAll, wNearAirAll, wNear0All);
    CHECK(noContactBlind == 0, "without a contact no LAND is blind (%d were)", noContactBlind);
  }
}

/* ---- the end of battery (user decision 2026-10-13, late) ---------------------------- */

/* kPrior: the controller's K prior (0: vzDefaultConfig's 165000; the drone's NVS vzK was 160391 on 2026-10-12) */
static void drainRunK(float v0, float C, float kOff, uint16_t cm, uint32_t seed, float kPrior, result_t *r)
{
  scen_t sc = {.seed = seed, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.1f, .trim = 0.3f,
               .camLossAtMoveS = -1.0f, .battDropAtS = -1.0f, .airLimit = true, .toCm = cm, .drain = true,
               .drainV0 = v0, .drainC = C, .kOff = kOff, .drainVs = 1.15e-5f, .vNoise = 0.035f, .kPrior = kPrior};
  toHeight(&sc, cm);   /* above the take-off cap: moves up first */
  runSim(&sc, r);
}
static void drainRun(float v0, float C, float kOff, uint16_t cm, uint32_t seed, result_t *r)
{
  drainRunK(v0, C, kOff, cm, seed, 0.0f, r);
}
/* drainRun with Claude's runbook descent above 1.2 m (scen_t drainDescend) from T_hold fromT */
static void drainRunDescend(float v0, float C, float kOff, uint16_t cm, uint32_t seed, float fromT, result_t *r)
{
  scen_t sc = {.seed = seed, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.1f, .trim = 0.3f,
               .camLossAtMoveS = -1.0f, .battDropAtS = -1.0f, .airLimit = true, .toCm = cm, .drain = true,
               .drainV0 = v0, .drainC = C, .kOff = kOff, .drainVs = 1.15e-5f, .vNoise = 0.035f, .drainDescend = true,
               .drainDescendT = fromT};
  toHeight(&sc, cm);
  runSim(&sc, r);
}

/* One drain sortie's checks; returns true when every one holds (the first failure is printed). */
static bool drainOk(const char *what, float C, const result_t *r)
{
  const bool capped = r->landWhy == AUTO_RS_VZ &&
                      (r->vzLandWhy == VZ_R_HOLD_CAPPED || r->vzLandWhy == VZ_R_CLIMB_CAPPED);
  const float impMax = C >= 250.0f ? 0.50f : 0.75f;
  const bool batt = r->landWhy == AUTO_RS_BATT_LOW;
  /* LAND thrust <= 0.98 x T_ref, or <= 1.03 x T_ref in a fast LAND (T_ref <= 53400, docs/autonomy-a2b.md 17.4);
   * the end of the battery's LAND is never fast */
  const bool landTop = r->landFast ? r->maxLandOverRef <= 1.03f * 1.0001f && r->landTRef <= VZ_LAND_FAST_TREF_MAX
                                   : r->maxLandOverRef <= 0.98f * 1.0001f;
  const bool ok = !capped && !r->killed && r->touchdown && r->impactV <= impMax && r->maxThr <= VZ_THRUST_ABS_MAX &&
                  landTop && !(batt && r->landFast) && r->landTHold <= VZ_THRUST_ABS_MAX &&
                  (!batt || r->landV >= 2.88f) && r->maxMoveTHold <= AUTO_MV_REFUSE_T;
  if (!ok) {
    printf("  %s: LAND %s (vz %s, fast %d) killed %d td %d impact %.2f (<= %.2f) max thrust %.0f LAND/T_ref %.4f at "
           "LAND T_hold %.0f loaded %.3f V, largest T_hold of an accepted MOVE %.0f\n", what,
           autoReasonName(r->landWhy), vzReasonName((vzReason_t)r->vzLandWhy), r->landFast, r->killed, r->touchdown,
           r->impactV, impMax, r->maxThr, r->maxLandOverRef, r->landTHold, r->landV, r->maxMoveTHold);
  }
  return ok;
}

/* The 2.00 m cap (2026-10-15, docs/autonomy-a2b.md 23.9) on the whole stack, reached by moves up from the 1.20 m
 * take-off cap: (a) a TAKEOFF to 1.20 m moved up 0.50 + 0.30 m, the Mac's LAND 10 s into the wait after them; (b)
 * 1.00 m moved up 0.50 + 0.50 m, LAND; (c) the battery drained from 3.82 V to its BATT_LOW LAND from 2.00 m
 * (testDrain's grid at 2.00 m); camera offsets 0 / +-0.01 m/s for (a) and (b). */
static void testHigh200Sim(void)
{
  static const float offs[] = {0.0f, 0.01f, -0.01f};
  float worst[2] = {0.0f, 0.0f}, durMax[2] = {0.0f, 0.0f}, peak = 0.0f;
  int ok[2] = {0, 0}, n[2] = {0, 0}, slow = 0;
  for (int k = 0; k < 2; k++) {
    for (int a = 0; a < 3; a++) {
      for (int sd = 0; sd < 3; sd++) {
        scen_t sc = baseScen(26000u + 100u * (uint32_t)k + 10u * (uint32_t)a + (uint32_t)sd, 0.0f);
        sc.eDmax = 0.0f;
        sc.camVzOff = offs[a];
        if (k == 0) {
          toHeight(&sc, 200);
          sc.waitMs = 10000;
        } else {
          sc.toCm = 100;
          sc.nUp = 2;
          sc.ups[0] = 50;
          sc.ups[1] = 50;
          sc.firstAfterS = 3.0f;
          sc.landAfterS = 1.0f;
        }
        result_t r;
        runSim(&sc, &r);
        n[k]++;
        worst[k] = fmaxf(worst[k], r.impactV);
        durMax[k] = fmaxf(durMax[k], r.landDurS);
        if (k == 0) {
          slow += (r.evMask & (1u << AUTO_EV_TAKEOFF_SLOW)) != 0u;
          peak = fmaxf(peak, r.hMax);
        }
        ok[k] += r.refused == 0 && r.touchdown && !r.killed && !r.cutInAir && r.impactV <= 0.45f &&
                 r.landWhy == AUTO_RS_LAND_CMD;
      }
    }
  }
  printf("info: TAKEOFF 1.20 m + 0.50 + 0.30 m, LAND 10 s into the wait: impact max %.2f m/s, LAND to DONE max %.2f s, "
         "TAKEOFF_SLOW %d of %d, true peak %.2f m; 1.00 m + 0.50 + 0.50 m, LAND: impact max %.2f m/s, LAND to DONE max "
         "%.2f s\n", worst[0], durMax[0], slow, n[0], peak, worst[1], durMax[1]);
  CHECK(ok[0] == n[0] && slow == 0, "TAKEOFF 1.20 m moved up to 2.00 m: the wait starts by itself (no TAKEOFF_SLOW: "
        "%d), both moves accepted, the Mac's LAND touches down <= 0.45 m/s, never killed or cut in the air (%d/%d, "
        "%.2f)", slow, ok[0], n[0], worst[0]);
  CHECK(ok[1] == n[1], "1.00 m moved up to 2.00 m, LAND: both moves accepted, touchdown <= 0.45 m/s (%d/%d, %.2f)",
        ok[1], n[1], worst[1]);

  /* (c) the end of the battery at 2.00 m. The measured battery is C ~400 A s (testDrain: night f6's 28.5 s to
   * 54k, ~84 s a charge): there every drainOk rule holds. The grid's weak end (150 / 250 A s) sags on through the
   * longer LAND from 2.00 m (10.7 s against 8.0 s from 1.20 m) and touches down harder than drainOk's 0.75 / 0.50 m/s
   * (0.59-1.01 when written; testDrain's 1.20 m grid at most 0.71): reported and bounded, and the runbook brings the
   * drone down to <= 1.2 m from ~3.20 V loaded or T_hold ~52000 on (docs/autonomy-a2b.md 23.5, 23.9). */
  static const float Cs[4] = {150.0f, 250.0f, 400.0f, 600.0f};
  static const float kOffs[3] = {-5000.0f, 0.0f, 5000.0f};
  int nd = 0, okd = 0, batt = 0, nReal = 0, okReal = 0;
  float imp = 0.0f, impReal = 0.0f, airLo = 1e9f, airHi = 0.0f;
  for (int a = 0; a < 4; a++) {
    for (int b = 0; b < 3; b++) {
      result_t r;
      char what[64];
      const uint32_t seed = 27000u + 10u * (uint32_t)a + (uint32_t)b;
      snprintf(what, sizeof(what), "2.00 m: C %.0f kOff %+.0f seed %u", Cs[a], kOffs[b], (unsigned)seed);
      drainRun(3.82f, Cs[a], kOffs[b], 200, seed, &r);
      nd++;
      const bool capped = r.landWhy == AUTO_RS_VZ &&
                          (r.vzLandWhy == VZ_R_HOLD_CAPPED || r.vzLandWhy == VZ_R_CLIMB_CAPPED);
      okd += !capped && !r.killed && !r.cutInAir && r.touchdown && r.impactV <= 1.1f && !r.landFast &&
             r.maxThr <= VZ_THRUST_ABS_MAX;
      if (Cs[a] >= 400.0f) {
        nReal++;
        okReal += drainOk(what, Cs[a], &r) && !r.cutInAir;
        impReal = fmaxf(impReal, r.impactV);
      } else if (r.impactV > (Cs[a] >= 250.0f ? 0.50f : 0.75f)) {
        printf("info: %s: touchdown %.2f m/s, over drainOk's %.2f (the weak battery's sag through the long LAND)\n",
               what, r.impactV, Cs[a] >= 250.0f ? 0.50f : 0.75f);
      }
      batt += r.landWhy == AUTO_RS_BATT_LOW;
      imp = fmaxf(imp, r.impactV);
      airLo = fminf(airLo, r.landAirMs / 1000.0f);
      airHi = fmaxf(airHi, r.landAirMs / 1000.0f);
    }
  }
  printf("info: the battery drained at 2.00 m (C 150..600 A s, K -5k..+5k): %d of %d BATT_LOW, LAND at %.1f..%.1f s of "
         "air, impact max %.2f m/s (C >= 400: %.2f)\n", batt, nd, airLo, airHi, imp, impReal);
  CHECK(okReal == nReal && batt == nd, "2.00 m, the battery's end, C 400 / 600 A s: every drainOk rule (%d/%d), "
        "BATT_LOW in every sortie (%d/%d)", okReal, nReal, batt, nd);
  CHECK(okd == nd, "2.00 m, the battery's end, C 150..600: a slow LAND to a touchdown <= 1.1 m/s (%.2f), never capped, "
        "killed or cut in the air (%d/%d)", imp, okd, nd);

  /* (d) the same grid with Claude's runbook descent (docs/autonomy-a2b.md 23.5): from T_hold 52000 or a 3.20 V reading
   * on, vertical MOVEs down to 1.20 m (accepted up to the 53400 refusal level). With the measured battery (C >= 400)
   * every BATT_LOW LAND then starts at or below ~1.4 m and keeps drainOk's bounds; the weak grid (C 150 / 250: 12-23 s
   * of air from 3.82 V, so the climb to 2.00 m itself breaks the runbook's 3.25 V rule) is reported. A descent sent
   * later (from 54600) is refused: near the ceiling one ended HOLD_CAPPED (a fast LAND, 1.2-1.5 m/s) in 1-2 of 40-50
   * sorties when the drone took it (2026-10-15 review), so nothing flies above 53400, a vertical MOVE down and a
   * descending SET_HEIGHT included: never HOLD_CAPPED. */
  int okDd = 0, nDd = 0, low = 0, descOk = 0, capLate = 0, refLate = 0, nRealD = 0, lowReal = 0;
  float impD = 0.0f, hLo = 9.0f, hHi = 0.0f;
  for (int a = 0; a < 4; a++) {
    for (int b = 0; b < 3; b++) {
      result_t r;
      char what[80];
      const uint32_t seed = 27000u + 10u * (uint32_t)a + (uint32_t)b;
      snprintf(what, sizeof(what), "2.00 m, runbook descent: C %.0f kOff %+.0f seed %u", Cs[a], kOffs[b],
               (unsigned)seed);
      result_t rl;
      drainRunDescend(3.82f, Cs[a], kOffs[b], 200, seed, 54600.0f, &rl);
      capLate += rl.landWhy == AUTO_RS_VZ && (rl.vzLandWhy == VZ_R_HOLD_CAPPED || rl.vzLandWhy == VZ_R_CLIMB_CAPPED);
      refLate += rl.nDescend > 0 && rl.nLowBatt > 0 && rl.maxMoveTHold <= AUTO_MV_REFUSE_T;
      drainRunDescend(3.82f, Cs[a], kOffs[b], 200, seed, 0.0f, &r);
      nDd++;
      impD = fmaxf(impD, r.impactV);
      hLo = fminf(hLo, r.hAtLand);
      hHi = fmaxf(hHi, r.hAtLand);
      const bool lowLand = r.hAtLand <= 1.40f;
      low += lowLand;
      nRealD += Cs[a] >= 400.0f;
      lowReal += Cs[a] >= 400.0f && lowLand;
      descOk += r.nDescend > 0;
      okDd += lowLand ? drainOk(what, Cs[a], &r) && !r.cutInAir : (!r.killed && !r.cutInAir && r.impactV <= 1.1f);
      if (!lowLand) {
        printf("info: %s: BATT_LOW came at %.2f m true (%d descents sent): touchdown %.2f m/s\n", what, r.hAtLand,
               r.nDescend, r.impactV);
      }
    }
  }
  printf("info: the battery drained at 2.00 m with the runbook descent: %d of %d LANDs from <= 1.40 m true (%.2f..%.2f "
         "m), %d sorties sent descents, touchdown max %.2f m/s; a descent sent from T_hold 54600: refused in %d, "
         "HOLD_CAPPED %d\n", low, nDd, hLo, hHi, descOk, impD, refLate, capLate);
  CHECK(lowReal == nRealD && okDd == nDd, "2.00 m with the runbook descent: the measured battery (C >= 400) LANDs from "
        "<= 1.40 m in every sortie (%d/%d); drainOk where the LAND came from there, <= 1.1 m/s and never cut otherwise "
        "(%d/%d)", lowReal, nRealD, okDd, nDd);
  CHECK(capLate == 0, "2.00 m, a descent sent from T_hold 54600 (refused, LOW_BATT %d): never HOLD_CAPPED (%d)",
        refLate, capLate);
}

static void testDrain(void)
{
  /* the grid: 3.82 V at rest (night f6), C 150 / 250 / 400 / 600 A s (end slopes at 54k ~ -12 to -3 mV/s; 400
   * matches night f6's 28.5 s to 54k and ~84 s per charge), K offset -5k / 0 / +5k, 0.30 / 0.60 / 1.20 m, 3 seeds */
  static const float Cs[4] = {150.0f, 250.0f, 400.0f, 600.0f};
  static const float kOffs[3] = {-5000.0f, 0.0f, 5000.0f};
  static const uint16_t cms[3] = {30, 60, 120};
  int n = 0, ok = 0, batt = 0, capped = 0, killed = 0, lowBatt = 0, other = 0, fastBatt = 0;
  float worstLandOverRef = 0.0f, maxThr = 0.0f, worstMoveTHold = 0.0f, minLandV = 9.0f;
  static float imp[108];
  static float gainC[4][27];
  float impMaxC[4] = {0, 0, 0, 0}, minVLandC[4] = {9, 9, 9, 9};
  float airLo[4] = {999, 999, 999, 999}, airHi[4] = {0, 0, 0, 0};
  float vLo[4] = {9, 9, 9, 9}, vHi[4] = {0, 0, 0, 0}, thLo[4] = {1e9f, 1e9f, 1e9f, 1e9f}, thHi[4] = {0, 0, 0, 0};
  int gainN[4] = {0, 0, 0, 0};
  uint32_t seed = 5000u;
  for (int rep = 0; rep < 3; rep++) {
    for (int a = 0; a < 4; a++) {
      for (int b = 0; b < 3; b++) {
        for (int k = 0; k < 3; k++) {
          result_t r;
          char what[64];
          snprintf(what, sizeof(what), "C %.0f kOff %+.0f %u cm seed %u", Cs[a], kOffs[b], cms[k], (unsigned)seed);
          drainRun(3.82f, Cs[a], kOffs[b], cms[k], seed++, &r);
          ok += drainOk(what, Cs[a], &r);
          batt += r.landWhy == AUTO_RS_BATT_LOW;
          const bool cap = r.landWhy == AUTO_RS_VZ &&
                           (r.vzLandWhy == VZ_R_HOLD_CAPPED || r.vzLandWhy == VZ_R_CLIMB_CAPPED);
          capped += cap;
          killed += r.killed;
          other += r.landWhy != AUTO_RS_BATT_LOW && !cap;
          if (r.landWhy != AUTO_RS_BATT_LOW) {
            printf("info: drain %s: LAND %s (vz %s) at air %.1f s, impact %.2f m/s\n", what, autoReasonName(r.landWhy),
                   vzReasonName((vzReason_t)r.vzLandWhy), r.landAirMs / 1000.0f, r.impactV);
          }
          lowBatt += r.nLowBatt;
          fastBatt += r.landWhy == AUTO_RS_BATT_LOW && r.landFast;
          if (!r.landFast) {
            worstLandOverRef = fmaxf(worstLandOverRef, r.maxLandOverRef);
          }
          maxThr = fmaxf(maxThr, r.maxThr);
          worstMoveTHold = fmaxf(worstMoveTHold, r.maxMoveTHold);
          imp[n++] = r.impactV;
          impMaxC[a] = fmaxf(impMaxC[a], r.impactV);
          if (r.landWhy == AUTO_RS_BATT_LOW) {
            minLandV = fminf(minLandV, r.landV);
            minVLandC[a] = fminf(minVLandC[a], r.minVLand);
            airLo[a] = fminf(airLo[a], r.landAirMs / 1000.0f);
            airHi[a] = fmaxf(airHi[a], r.landAirMs / 1000.0f);
            vLo[a] = fminf(vLo[a], r.landV);
            vHi[a] = fmaxf(vHi[a], r.landV);
            thLo[a] = fminf(thLo[a], r.landTHold);
            thHi[a] = fmaxf(thHi[a], r.landTHold);
            if (r.tAt538 >= 0.0f) {
              gainC[a][gainN[a]++] = r.landT - r.tAt538;
            }
          }
        }
      }
    }
  }
  qsort(imp, (size_t)n, sizeof(float), cmpf);
  printf("info: drain grid (3.82 V at rest, %d sorties): BATT_LOW %d, HOLD_CAPPED / CLIMB_CAPPED %d, other %d, "
         "KILLED %d; touchdown p50 %.2f p90 %.2f max %.2f m/s; LOW_BATT refusals %d\n", n, batt, capped, other, killed,
         imp[n / 2], imp[(n * 9) / 10], imp[n - 1], lowBatt);
  for (int a = 0; a < 4; a++) {
    qsort(gainC[a], (size_t)gainN[a], sizeof(float), cmpf);
    printf("info:   C %3.0f: BATT_LOW at air %.1f-%.1f s, T_hold %.0f-%.0f, loaded %.3f-%.3f V; min loaded in LAND "
           "%.3f V; touchdown max %.2f m/s; the LAND after the first T_hold >= 53800 (near the old 54000 end): "
           "median +%.1f s\n", Cs[a], airLo[a],
           airHi[a], thLo[a], thHi[a], vLo[a], vHi[a], minVLandC[a], impMaxC[a],
           gainN[a] > 0 ? gainC[a][gainN[a] / 2] : 0.0f);
  }
  CHECK(n == 108 && ok == n, "drain grid: every sortie lands slowly, never capped or KILLED, touchdown <= 0.50 m/s "
        "(C >= 250) / 0.75 m/s (C 150), thrust <= 58000 and <= 0.98 T_ref in LAND (<= 1.03 in a fast LAND, T_ref <= "
        "53400), loaded >= 2.88 V at the BATT_LOW, no MOVE accepted above a T_hold of 53400 (%d/%d)", ok, n);
  CHECK(fastBatt == 0, "drain grid: no BATT_LOW LAND is a fast one (%d)", fastBatt);
  CHECK(capped == 0 && killed == 0 && batt >= n - 2, "drain grid: no HOLD_CAPPED / CLIMB_CAPPED (%d), no KILLED (%d), "
        "BATT_LOW %d of %d", capped, killed, batt, n);
  CHECK(maxThr <= VZ_THRUST_ABS_MAX && worstLandOverRef <= 0.98f * 1.0001f && worstMoveTHold <= AUTO_MV_REFUSE_T,
        "drain grid: thrust max %.0f <= %.0f, a LAND that is not fast <= %.4f x T_ref, accepted MOVEs at T_hold <= %.0f",
        maxThr, VZ_THRUST_ABS_MAX, worstLandOverRef, worstMoveTHold);
  CHECK(lowBatt > 0, "drain grid: the drone refused MOVEs LOW_BATT near the end (%d)", lowBatt);
  CHECK(minLandV >= 2.88f, "drain grid: loaded >= 2.88 V when BATT_LOW starts the LAND (%.3f)", minLandV);

  /* a full hover from 4.10 V, C 200 / 300 / 400 (and 600: a cell that outlasts 120 s): the battery's BATT_LOW ends
   * it (at ~91 s for C 400), a slow LAND, or for C 600 the Mac's LAND at 120 s of air (AIR_TIME until 2026-10-15);
   * whichever ends first is reported, and it must be BATT_LOW (never fast) or the Mac's at 120 s */
  static const float Cf[4] = {200.0f, 300.0f, 400.0f, 600.0f};
  int battFirst = 0, airFirst = 0;
  for (int a = 0; a < 4; a++) {
    for (int k = 1; k < 3; k++) {
      result_t r;
      char what[64];
      snprintf(what, sizeof(what), "from 4.10 V, C %.0f, %u cm", Cf[a], cms[k]);
      drainRun(4.10f, Cf[a], 0.0f, cms[k], 6000u + (uint32_t)(3 * a + k), &r);
      printf("info: drain %s: LAND %s at air %.1f s, T_hold %.0f, loaded %.3f V (rest %.3f), min loaded in LAND "
             "%.3f V, fast %d, touchdown %.2f m/s", what, autoReasonName(r.landWhy), r.landAirMs / 1000.0f,
             r.landTHold, r.landV, r.landVRest, r.minVLand, r.landFast, r.impactV);
      if (r.landWhy == AUTO_RS_BATT_LOW && r.tAt538 >= 0.0f) {
        printf(", %.1f s after the first T_hold >= 53800", r.landT - r.tAt538);
      }
      printf("\n");
      battFirst += r.landWhy == AUTO_RS_BATT_LOW;
      airFirst += r.landWhy == AUTO_RS_LAND_CMD && r.macAirLand;
      const bool end = (r.landWhy == AUTO_RS_BATT_LOW && !r.landFast) ||
                       (r.landWhy == AUTO_RS_LAND_CMD && r.macAirLand && r.landAirMs <= MAC_AIR_LAND_MS + 2u);
      CHECK(drainOk(what, Cf[a], &r) && end && (Cf[a] > 400.0f || r.landWhy == AUTO_RS_BATT_LOW),
            "drain %s: LAND %s (BATT_LOW, slow, up to C 400; the Mac's at 120 s allowed beyond), slow touchdown "
            "%.2f m/s", what, autoReasonName(r.landWhy), r.impactV);
    }
  }
  printf("info: drain from 4.10 V over C 200-600: BATT_LOW first %d, the Mac's LAND at 120 s first %d\n", battFirst,
         airFirst);
}

/* The drone's own K prior (after the verification of cb09352). The grid above runs at vzDefaultConfig's kPrior of
 * 165000; the drone's NVS vzK is about 160.4k (160391 on 2026-10-12, learned after slow LANDs). The plant's K is the
 * pooled fit (+ kOff) whatever the prior, so a lower prior puts the plant higher against the K_RANGE clamp (1.12 x
 * kPrior, 0.3 s): with the 58000 CLIMB ceiling Khat reaches that clamp in the take-off's climb before the ceiling
 * freezes it, and the controller lands K_RANGE (a fast LAND from about 0.25 m). The same grid at 160400 and 158000:
 * report the K_RANGE count and touchdowns, and check that nothing else changes (no capped ending, no KILLED, every
 * BATT_LOW as in the grid). */
static bool isTakeoffLand(const result_t *r)
{
  return r->stateAtLand == AUTO_TAKEOFF;
}

static void testDrainVzK(void)
{
  static const float Cs[4] = {150.0f, 250.0f, 400.0f, 600.0f};
  static const float kOffs[3] = {-5000.0f, 0.0f, 5000.0f};
  static const uint16_t cms[3] = {30, 60, 120};
  static const float kps[2] = {160400.0f, 158000.0f};
  for (int q = 0; q < 2; q++) {
    int n = 0, batt = 0, kr = 0, krTo = 0, krToK0 = 0, capped = 0, killed = 0, noTd = 0, other = 0, battOk = 0;
    float impBatt = 0.0f, impKr = 0.0f, hKr = 0.0f, krRelMin = 9.0f, hovRelMax = 0.0f;
    uint32_t seed = 5000u;
    for (int rep = 0; rep < 3; rep++) {
      for (int a = 0; a < 4; a++) {
        for (int b = 0; b < 3; b++) {
          for (int k = 0; k < 3; k++) {
            result_t r;
            char what[80];
            snprintf(what, sizeof(what), "vzK %.0f C %.0f kOff %+.0f %u cm seed %u", kps[q], Cs[a], kOffs[b], cms[k],
                     (unsigned)seed);
            drainRunK(3.82f, Cs[a], kOffs[b], cms[k], seed++, kps[q], &r);
            n++;
            const bool cap = r.landWhy == AUTO_RS_VZ &&
                             (r.vzLandWhy == VZ_R_HOLD_CAPPED || r.vzLandWhy == VZ_R_CLIMB_CAPPED);
            const bool isKr = r.landWhy == AUTO_RS_VZ && r.vzLandWhy == VZ_R_K_RANGE;
            capped += cap;
            killed += r.killed;
            noTd += !r.touchdown;
            if (r.kRelHover > 0.0f) {
              hovRelMax = fmaxf(hovRelMax, r.kRelHover);
            }
            if (r.landWhy == AUTO_RS_BATT_LOW) {
              batt++;
              battOk += drainOk(what, Cs[a], &r);
              impBatt = fmaxf(impBatt, r.impactV);
            } else if (isKr) {
              kr++;
              impKr = fmaxf(impKr, r.impactV);
              if (isTakeoffLand(&r)) {
                krTo++;
                krToK0 += kOffs[b] <= 0.0f;
                hKr = fmaxf(hKr, r.hAtLand);
                krRelMin = fminf(krRelMin, r.kRelAtLand);
              }
            } else {
              other++;
              printf("info: drain %s: LAND %s (vz %s) at air %.1f s, impact %.2f m/s\n", what,
                     autoReasonName(r.landWhy), vzReasonName((vzReason_t)r.vzLandWhy), r.landAirMs / 1000.0f,
                     r.impactV);
            }
          }
        }
      }
    }
    printf("info: drain grid at vzK %.0f (3.82 V at rest, %d sorties): BATT_LOW %d (touchdown max %.2f m/s), K_RANGE %d "
           "(%d in the take-off climb, %d of them with kOff <= 0; from up to %.2f m, the plant K >= %.3f x vzK there; "
           "touchdown max %.2f m/s), other %d; the largest plant K / vzK of a take-off that reached HOVER %.3f\n",
           kps[q], n, batt, impBatt, kr, krTo, krToK0, hKr, krTo > 0 ? krRelMin : 0.0f, impKr, other, hovRelMax);
    CHECK(capped == 0 && killed == 0 && noTd == 0 && other == 0 && battOk == batt,
          "drain grid at vzK %.0f: every sortie ends BATT_LOW (as in the grid at 165000) or K_RANGE, never capped "
          "(%d), KILLED (%d) or without a touchdown (%d), other %d", kps[q], capped, killed, noTd, other);
    CHECK(impKr <= 0.80f, "drain grid at vzK %.0f: the K_RANGE LANDs touch down <= 0.80 m/s (%.2f)", kps[q], impKr);
    if (kps[q] > 160000.0f) {
      CHECK(krToK0 == 0, "drain grid at vzK %.0f: a drone on the pooled fit (kOff <= 0) never lands K_RANGE in the "
            "take-off (%d)", kps[q], krToK0);
    }
  }
}

/* Take-offs from a weaker cell (after the verification of cb09352): the 58000 CLIMB ceiling lets a sortie leave the
 * ground from about 3.6-3.7 V at rest (6742d91: about 3.80 V), and the take-off is where every logged 40 kHz in-air
 * reset happened. The sweep reports how these take-offs end and how deep the loaded voltage goes in the air, per
 * rest voltage, at the drone's vzK 160400 and at 158000. */
static void testWeakTakeoff(void)
{
  static const float V0[7] = {3.60f, 3.65f, 3.70f, 3.75f, 3.78f, 3.80f, 3.85f};
  static const float Cs[2] = {250.0f, 400.0f};
  static const float kOffs[3] = {-5000.0f, 0.0f, 5000.0f};
  static const uint16_t cms[3] = {30, 60, 120};
  static const float kps[2] = {160400.0f, 158000.0f};
  for (int q = 0; q < 2; q++) {
    int killed = 0, noTd = 0, holdCap = 0, toLandAt380 = 0;
    float impAll = 0.0f;
    for (int a = 0; a < 7; a++) {
      int n = 0, toFast = 0, toFastK0 = 0, toCap = 0, toNoLift = 0, toKr = 0, ground = 0, batt = 0, hover = 0;
      int other = 0;
      float impTo = 0.0f, hTo = 0.0f, vAirTo = 9.0f, vAir = 9.0f, impRow = 0.0f;
      for (int c = 0; c < 2; c++) {
        for (int b = 0; b < 3; b++) {
          for (int k = 0; k < 3; k++) {
            result_t r;
            drainRunK(V0[a], Cs[c], kOffs[b], cms[k], 8000u + 100u * (uint32_t)a + 10u * (uint32_t)c +
                      3u * (uint32_t)b + (uint32_t)k, kps[q], &r);
            n++;
            killed += r.killed;
            impAll = fmaxf(impAll, r.impactV);
            impRow = fmaxf(impRow, r.impactV);
            vAir = fminf(vAir, r.minVAir);
            const bool airborne = r.minVAir < 9.0f;
            /* a hop of a few cm in the RAMP ends on the state machine's TOUCHDOWN without the controller's */
            noTd += airborne && !(r.touchdown || r.endWhy == AUTO_RS_TOUCHDOWN);
            holdCap += r.landWhy == AUTO_RS_VZ && r.vzLandWhy == VZ_R_HOLD_CAPPED;
            hover += r.kRelHover > 0.0f;
            if (isTakeoffLand(&r) && r.landWhy == AUTO_RS_VZ) {
              toFast++;
              toFastK0 += kOffs[b] <= 0.0f;
              toCap += r.vzLandWhy == VZ_R_CLIMB_CAPPED;
              toNoLift += r.vzLandWhy == VZ_R_NO_LIFTOFF_CEIL || r.vzLandWhy == VZ_R_NO_LIFTOFF_TIME;
              toKr += r.vzLandWhy == VZ_R_K_RANGE;
              impTo = fmaxf(impTo, r.impactV);
              hTo = fmaxf(hTo, r.hAtLand);
              vAirTo = fminf(vAirTo, r.minVAir);
              if (V0[a] >= 3.80f - 1e-4f && kOffs[b] <= 0.0f) {
                toLandAt380++;
              }
            } else if (isTakeoffLand(&r)) {
              ground++;   /* VBAT_DIP / VBAT_LOW in the take-off (the 1 kHz minimum below 2.55 V in the RAMP) */
            } else if (r.landWhy == AUTO_RS_BATT_LOW) {
              batt++;
            } else {
              other++;
            }
          }
        }
      }
      printf("info: take-off from %.2f V at rest, vzK %.0f (%d sorties): fast take-off LAND %d (kOff <= 0: %d; "
             "CLIMB_CAPPED %d, NO_LIFTOFF %d, K_RANGE %d; touchdown max %.2f m/s from up to %.2f m, loaded down to "
             "%.3f V in the air), battery LAND in the take-off %d, reached HOVER %d (then BATT_LOW %d, other %d); "
             "lowest loaded V in the air %.3f, touchdown max %.2f m/s\n", V0[a], kps[q], n, toFast, toFastK0, toCap,
             toNoLift, toKr, impTo, hTo, toFast > 0 ? vAirTo : 0.0f, ground, hover, batt, other,
             vAir < 9.0f ? vAir : 0.0f, impRow);
    }
    /* HOLD_CAPPED: the end of the battery's race between BATT_LOW (T_hold over 57000 for 0.5 s) and the controller's
     * HOLD_CAPPED (thrust at the 58000 ceiling for 0.5 s) when T_hold oscillates up to the ceiling. Over 6 seed sets
     * of the 3.78-3.85 V cells at vzK 158000 (324 sorties) it is 1 in 324 both with the forward hold in the take-off
     * climb and without it (docs/autonomy-a2b.md 19.5); this grid's seeds hit it once since then (3.85 V, C 250,
     * kOff 0, 60 cm: LAND HOLD_CAPPED at 31.4 s, touchdown 0.72 m/s, instead of BATT_LOW at 32.0 s, 0.33 m/s). */
    CHECK(killed == 0 && noTd == 0 && holdCap <= 1 && impAll <= 1.30f,
          "weak-cell take-offs at vzK %.0f: no KILLED (%d), a touchdown after every lift-off (%d missing), "
          "HOLD_CAPPED at most once (%d), touchdown <= 1.30 m/s (%.2f)", kps[q], killed, noTd, holdCap, impAll);
    if (kps[q] > 160000.0f) {
      CHECK(toLandAt380 == 0, "weak-cell take-offs at vzK %.0f: from 3.80 V at rest or more, a drone on the pooled fit "
            "(kOff <= 0) never lands in the take-off (%d)", kps[q], toLandAt380);
    }
  }
}

/* ---- the long hover (the 120 s air clock of 2026-10-14 removed 2026-10-15: the Mac lands at 120 s here) -------- */

/* Long hovers against the height estimate's drift: the camera's vertical speed offset c (the estimate's drift; +
 * reads a climb that is not there, so the drone sinks) over the logged range and beyond (-0.017, -0.011 (the worst
 * logged reading low), -0.005, 0 (the sim's own +/-0.004), +0.010, +0.020, +0.031 (the worst logged reading high)
 * m/s), at 1.00 / 1.05 / 1.20 m, 10 seeds, hovering to the Mac's LAND at 120 s of air (the air clock's AIR_TIME
 * until 2026-10-15). Per cell: the true height at the LAND, the highest true height in HOVER (the ceiling exposure),
 * how it ended (the Mac's LAND, the controller's K_RANGE, SINK, ...), the touchdown. Checks: never KILLED, never the controller's
 * LAND_TIMEOUT cut in the air, every sortie a touchdown <= 1.0 m/s. Built with -DVZ_LAND_DRIFT_MAX_M=1.0f (host
 * only) it shows the 2.0 m drift allowance's gain at c <= -0.011 (docs/autonomy-a2b.md 18.1). */
static void testLongHover(void)
{
  static const float cs[7] = {-0.017f, -0.011f, -0.005f, 0.0f, 0.010f, 0.020f, 0.031f};
  static const uint16_t hs[3] = {100, 105, 120};
  int n = 0, killed = 0, cutAir = 0, noTd = 0, fastImp = 0;
  float impAll = 0.0f, impLogged = 0.0f, ceilAll = 0.0f, ceilLogged = 0.0f;
  for (int a = 0; a < 7; a++) {
    for (int b = 0; b < 3; b++) {
      int air = 0, kr = 0, noCam = 0, other = 0, ground = 0, fast = 0;
      float hLandLo = 9.0f, hLandHi = 0.0f, ceil = 0.0f, imp = 0.0f, durMax = 0.0f;
      for (int i = 0; i < 10; i++) {
        scen_t sc = baseScen(7000u + 100u * (uint32_t)a + 10u * (uint32_t)b + (uint32_t)i, 1.0f);
        sc.airLimit = true;
        sc.toCm = hs[b];
        sc.camVzOff = cs[a];
        result_t r;
        runSim(&sc, &r);
        n++;
        killed += r.killed;
        cutAir += r.cutInAir;
        noTd += !(r.touchdown || r.endWhy == AUTO_RS_TOUCHDOWN || r.endWhy == AUTO_RS_TOUCHDOWN_TRIP);
        air += r.landWhy == AUTO_RS_LAND_CMD && r.macAirLand;
        kr += r.landWhy == AUTO_RS_VZ && r.vzLandWhy == VZ_R_K_RANGE;
        noCam += r.landWhy == AUTO_RS_SINK;
        other += !((r.landWhy == AUTO_RS_LAND_CMD && r.macAirLand) ||
                   (r.landWhy == AUTO_RS_VZ && r.vzLandWhy == VZ_R_K_RANGE) || r.landWhy == AUTO_RS_SINK);
        ground += r.groundInWait;
        fast += r.landFast;
        hLandLo = fminf(hLandLo, r.hAtLand);
        hLandHi = fmaxf(hLandHi, r.hAtLand);
        ceil = fmaxf(ceil, r.hMaxHover);
        imp = fmaxf(imp, r.impactV);
        if (r.landDurS > durMax) {
          durMax = r.landDurS;
        }
        if (r.landFast && r.impactV > 0.71f) {
          fastImp++;
        }
        if (r.killed || r.cutInAir || r.impactV > 1.0f) {
          printf("info: long hover c %+.3f %u cm seed %u: LAND %s (vz %s) at air %.1f s, end %s, killed %d, cut in "
                 "the air %d, impact %.2f m/s\n", (double)cs[a], hs[b], (unsigned)sc.seed, autoReasonName(r.landWhy),
                 vzReasonName((vzReason_t)r.vzLandWhy), r.landAirMs / 1000.0f, autoReasonName(r.endWhy), r.killed,
                 r.cutInAir, r.impactV);
        }
      }
      impAll = fmaxf(impAll, imp);
      ceilAll = fmaxf(ceilAll, ceil);
      if (cs[a] >= -0.011f - 1e-6f) {   /* the logged range: -0.011..+0.031 m/s */
        impLogged = fmaxf(impLogged, imp);
        ceilLogged = fmaxf(ceilLogged, ceil);
      }
      printf("info: long hover c %+.3f m/s at %.2f m (10 seeds): the Mac's LAND at 120 s %d, K_RANGE %d, SINK %d, other "
             "%d; on the floor in the wait %d; true h at the LAND %.2f..%.2f m; highest in HOVER %.2f m; fast %d; LAND "
             "to DONE max %.1f s; touchdown max %.2f m/s\n", (double)cs[a], hs[b] * 0.01f, air, kr, noCam, other,
             ground, hLandLo, hLandHi, ceil, fast, durMax, imp);
    }
  }
  printf("info: long hover (%d sorties, VZ_LAND_DRIFT_MAX_M %.1f m): highest true height in HOVER %.2f m (c within "
         "the logged -0.011..+0.031 m/s: %.2f m), touchdown max %.2f m/s (logged range: %.2f), fast LANDs over 0.71 m/s "
         "%d\n", n, (double)VZ_LAND_DRIFT_MAX_M, ceilAll, ceilLogged, impAll, impLogged, fastImp);
  CHECK(killed == 0 && cutAir == 0 && noTd == 0, "long hovers to 120 s, c -0.017..+0.031 m/s, 1.00-1.20 m: never "
        "KILLED (%d), never cut in the air by the LAND timeout (%d), a touchdown in every sortie (%d missing)", killed,
        cutAir, noTd);
#ifndef VZ_HOST_DRIFT_CAP_OVERRIDDEN
  /* c -0.017 m/s (0.017 x 118 s = the 2.0 m allowance) plus this sim's own g0 capture error (+/-1.3 mg through the
   * observer's 0.2 s blend: +/-0.3 m in 120 s; the drone's observer includes g0, the sim's does not) reaches
   * 2.5 m over the estimate: the last of it flies after the flat part, faster */
  CHECK(impLogged <= 1.0f && impAll <= 1.3f, "long hovers: touchdown <= 1.0 m/s with c in the logged range (%.2f), "
        "<= 1.3 m/s at c -0.017 m/s (%.2f)", impLogged, impAll);
#endif

}

/* The K drift of a long hover: the plant's K grows 1 % per 10 s (the battery's drain) from 160k against the
 * drone's own prior vzK 150873 (6 % low): Khat walks to its clamp (1.12 x 150873 = 169.0k) and the controller lands
 * K_RANGE (a LAND that is not slow: landStart, falling at once) before the 120 s clock. Reported: when, from where,
 * the touchdown; checked: no KILLED, a touchdown. */
static void testKDrift(void)
{
  static const uint16_t hs[3] = {60, 105, 120};
  int n = 0, killed = 0, noTd = 0, kr = 0, krLate = 0;
  float imp = 0.0f, krLo = 999.0f, krHi = 0.0f, hKr = 0.0f;
  for (int b = 0; b < 3; b++) {
    for (int i = 0; i < 5; i++) {
      scen_t sc = baseScen(7600u + 10u * (uint32_t)b + (uint32_t)i, 1.0f);
      sc.airLimit = true;
      sc.toCm = hs[b];
      sc.kPrior = 150873.0f;
      sc.kTrueAbs = 160000.0f;
      sc.kDriftPerS = 0.001f;
      result_t r;
      runSim(&sc, &r);
      n++;
      killed += r.killed;
      noTd += !(r.touchdown || r.endWhy == AUTO_RS_TOUCHDOWN);
      imp = fmaxf(imp, r.impactV);
      const bool isKr = r.landWhy == AUTO_RS_VZ && r.vzLandWhy == VZ_R_K_RANGE;
      kr += isKr;
      if (isKr) {
        krLate += r.landAirMs > 80000u;
        krLo = fminf(krLo, r.landAirMs / 1000.0f);
        krHi = fmaxf(krHi, r.landAirMs / 1000.0f);
        hKr = fmaxf(hKr, r.hAtLand);
      }
      if (i == 0 || !isKr) {
        printf("info: K drift +1 %%/10 s from 160k, vzK 150873, %u cm seed %u: LAND %s (vz %s) at air %.1f s from %.2f "
               "m, plant K / vzK %.3f, touchdown %.2f m/s\n", hs[b], (unsigned)sc.seed, autoReasonName(r.landWhy),
               vzReasonName((vzReason_t)r.vzLandWhy), r.landAirMs / 1000.0f, r.hAtLand, r.kRelAtLand, r.impactV);
      }
    }
  }
  printf("info: K drift (%d sorties): K_RANGE %d (%d after 80 s of air) at air %.1f-%.1f s from up to %.2f m, "
         "touchdown max %.2f m/s\n", n, kr, krLate, kr ? krLo : 0.0f, krHi, hKr, imp);
  CHECK(killed == 0 && noTd == 0, "K drift: no KILLED (%d), a touchdown in every sortie (%d missing)", killed, noTd);
}

/* ---- a hand at the drone (the palm cue of 2026-10-14 removed 2026-10-15, docs/autonomy-a2b.md 22) ------------- */

static scen_t handScen(uint32_t seed, uint16_t cm, float v, float lift, float k, float c)
{
  scen_t sc = baseScen(seed, 1.0f);
  sc.toCm = cm;
  sc.handKind = 1;
  sc.handDuring = 0;
  sc.handAtS = 5.0f;
  sc.handGap = 0.10f;
  sc.handV = v;
  sc.handLift = lift;
  sc.handK = k;
  sc.handC = c;
  sc.handGiveUpS = 3.0f;
  return sc;
}

/* Until 2026-10-15 a palm under the hover landed PALM (the cue) or COLLISION; now nothing on the drone lands for a
 * hand: a hard touch is a CONTACT (the hush, the event), a soft one nothing; the drone holds and reports, and the
 * Mac decides from a photo (docs/autonomy-a2b.md 22's runbook: LAND on a hand). (1) A palm brought up under the hover
 * (a unilateral spring-damper from 10 cm below at the approach speed, the lift over 0.2 s from the contact), lowered
 * away 3 s after the contact, the Mac's LAND 3 s later: a normal end, never KILLED, never a motor stop free in the
 * air, never the controller's SUPPORT in the air, a touchdown <= 1.0 m/s, the true height above the contact's
 * bounded (that LAND is the blind one when the palm made a CONTACT, user decision (B) 2026-10-15: touchdowns up to
 * 0.74 m/s, 0.40 at faa18db with the camera's). (2) A Mac LAND onto a palm held still under the drone: a touchdown on it (the stall rule and FORCE).
 * (3) A grab in the TAKEOFF climb at 0.40 m (the hand holds both ways): CONTACT with the camera height (the
 * controller's CONTACT_HOLD: HOLD from the climb) or LAND COLLISION without it; the Mac's LAND 3 s after the
 * contact: never KILLED, never a stop free in the air. */
static void testHandHold(void)
{
  static const float vs[3] = {0.02f, 0.05f, 0.30f};
  static const float lifts[2] = {0.0f, 0.03f};
  static const float kc[3][2] = {{200.0f, 2.0f}, {500.0f, 5.0f}, {2000.0f, 10.0f}};
  static const uint16_t hs[4] = {40, 70, 100, 120};
  int n = 0, ok = 0, contacts = 0, killed = 0, free = 0, supAir = 0, events = 0;
  float imp = 0.0f, climb = 0.0f;
  uint32_t seed = 12000u;
  for (int ih = 0; ih < 4; ih++) {
    for (int iv = 0; iv < 3; iv++) {
      for (int il = 0; il < 2; il++) {
        for (int ik = 0; ik < 3; ik += 2) {
          scen_t sc = handScen(seed++, hs[ih], vs[iv], lifts[il], kc[ik][0], kc[ik][1]);
          result_t r;
          runSim(&sc, &r);
          n++;
          contacts += r.contacts > 0;
          events += r.evMask != 0u;
          killed += r.killed;
          free += r.offFree;
          supAir += r.supInAir;
          imp = fmaxf(imp, r.impactV);
          const float up = r.handContact ? r.hMaxAfterHand - r.hContact : 0.0f;
          climb = fmaxf(climb, up);
          const bool good = r.handContact && !r.killed && !r.offFree && !r.supInAir &&
                            r.endWhy == AUTO_RS_TOUCHDOWN && r.landWhy == AUTO_RS_LAND_CMD && r.impactV <= 1.0f &&
                            up <= 0.15f;
          ok += good;
          if (!good) {
            printf("info: hand %u cm v %.2f lift %.0f cm k %.0f seed %u: contact %d, LAND %s, end %s (vz %s), killed "
                   "%d, free %d, SUPPORT in the air %d, touchdown %.2f m/s, above the contact %+.3f m\n", hs[ih],
                   vs[iv], lifts[il] * 100.0f, kc[ik][0], (unsigned)sc.seed, r.handContact,
                   autoReasonName(r.landWhy), autoReasonName(r.endWhy), vzReasonName((vzReason_t)r.endVzWhy), r.killed,
                   r.offFree, r.supInAir, r.impactV, up);
          }
        }
      }
    }
  }
  printf("info: a palm under the hover (%d runs: 0.40-1.20 m, approach 0.02-0.30 m/s, lift 0 / 3 cm, k 200 / 2000): "
         "CONTACT in %d, any event in %d; touchdown max %.2f m/s; the true height above the contact's max %+.3f m\n", n,
         contacts, events, imp, climb);
  CHECK(ok == n && killed == 0 && free == 0 && supAir == 0, "a palm under the hover: no LAND for it on the drone, "
        "the Mac's LAND after the hand left: a touchdown <= 1.0 m/s, never KILLED, never a stop free in the air or "
        "SUPPORT in the air, <= 0.15 m above the contact (%d/%d)", ok, n);

  /* (2) a Mac LAND onto a palm held still under the drone (0.20 / 0.30 m under the true height) */
  int okL = 0, nL = 0;
  float offL = 0.0f;
  for (int fast = 0; fast < 2; fast++) {
    for (int ik = 0; ik < 3; ik++) {
      scen_t sc = handScen(seed++, fast ? 100 : 70, 0.0f, 0.0f, kc[ik][0], kc[ik][1]);
      sc.handDuring = 2;
      sc.handAtS = 3.0f;
      sc.handGap = fast ? 0.30f : 0.20f;
      sc.handGiveUpS = 0.0f;
      result_t r;
      runSim(&sc, &r);
      nL++;
      const float off = r.offT >= 0.0f ? r.offT - r.contactT : 99.0f;
      offL = fmaxf(offL, off);
      okL += r.handContact && !r.killed && r.endWhy == AUTO_RS_TOUCHDOWN && off <= 1.5f && !r.supInAir &&
             !r.offFree && r.landWhy == AUTO_RS_LAND_CMD;
    }
  }
  CHECK(okL == nL, "a Mac LAND onto a palm: a touchdown on it (the stall rule and FORCE, not height-gated), motors off "
        "<= 1.5 s after the contact, never KILLED (%d/%d, max %.2f s)", okL, nL, offL);

  /* (3) a grab in the TAKEOFF climb at 0.40 m, the Mac's LAND 3 s after the contact */
  int okG = 0, nG = 0, hold = 0, coll = 0;
  for (int ik = 0; ik < 3; ik++) {
    for (int i = 0; i < 3; i++) {
      scen_t sc = handScen(seed++, 100, 0.0f, 0.0f, kc[ik][0], kc[ik][1]);
      sc.handKind = 2;
      sc.handDuring = 3;
      sc.handAtH = 0.40f;
      sc.handGiveUpS = 0.0f;
      sc.macLandAfterContactS = 3.0f;
      result_t r;
      runSim(&sc, &r);
      nG++;
      hold += r.contacts > 0;
      coll += r.landWhy == AUTO_RS_COLLISION;
      const bool good = r.handContact && !r.killed && !r.offFree && !r.supInAir &&
                        (r.endWhy == AUTO_RS_TOUCHDOWN || r.endWhy == AUTO_RS_TOUCHDOWN_TRIP);
      okG += good;
      if (!good || i == 0) {
        printf("info: grab in the climb k %.0f seed %u: CONTACT events %d, LAND %s, end %s (vz %s), killed %d, free "
               "%d, touchdown %.2f m/s\n", kc[ik][0], (unsigned)sc.seed, r.contacts, autoReasonName(r.landWhy),
               autoReasonName(r.endWhy), vzReasonName((vzReason_t)r.endVzWhy), r.killed, r.offFree, r.impactV);
      }
    }
  }
  CHECK(okG == nG, "a grab in the climb: CONTACT (%d) or LAND COLLISION without the camera height (%d); the Mac's "
        "LAND: a touchdown in the hand, never KILLED, never a stop free in the air (%d/%d)", hold, coll, okG, nG);
}

/* ---- the hold's gain scale with the scene distance (docs/autonomy-a2b.md 20) ------------------------------------ */

typedef struct {
  double xH3, xL8, xL15, xL45, max15, p2pMax, bandMax, rmsBand, sL15;
  int n, nH3, n45;
} gridCell_t;

static void gridRun(float D, float eD, float kmax, bool noNoise, gridCell_t *c)
{
  memset(c, 0, sizeof(*c));
  const float kSave = simKmax;
  simKmax = kmax;
  for (uint32_t sd = 0; sd < 20; sd++) {
    scen_t sc = baseScen(9100u + 37u * sd, ROOM - D);   /* facing +y: the wall D ahead */
    sc.coupled = true;
    sc.trimFixed = true;
    sc.trim = 0.3f;
    sc.eDfix = true;
    sc.eDval = eD;
    sc.endAfterLiftS = 45.5f;
    sc.noCamNoise = noNoise;
    result_t r;
    runSim(&sc, &r);
    c->n++;
    if (r.xH3 >= 0.0f) {
      c->xH3 += r.xH3;
      c->nH3++;
    }
    c->xL8 += r.xL8;
    c->xL15 += r.xL15;
    c->max15 += r.max15;
    c->sL15 += r.sL15;
    if (r.xL45 >= 0.0f) {
      c->xL45 += r.xL45;
      c->n45++;
    }
    c->p2pMax = fmax(c->p2pMax, r.p2p);
    c->bandMax = fmax(c->bandMax, r.p2pBand);
    c->rmsBand += r.rmsBand;
  }
  c->rmsBand /= c->n;
  c->xH3 /= c->nH3 ? c->nH3 : 1;
  c->xL8 /= c->n;
  c->xL15 /= c->n;
  c->max15 /= c->n;
  c->sL15 /= c->n;
  c->xL45 /= c->n45 ? c->n45 : 1;
  simKmax = kSave;
}

/* Plain hovers with the attitude filter's coupling (the controller holds the estimate; d' = 0.384 (drag v / g - e) +
 * mki): TAKEOFF 1.00 m, a photo, HOVER to lift-off + 45 s, no motion. 20 seeds x lean error +-0.3 deg (each axis,
 * signs from the seed) x D 1 / 2 / 3 / 4 m x the observer's D error -0.3 / 0 / +1.0 (its G = f / (D (1 + eD)), so
 * +1.0 asks for twice the gain the scene needs), kmax 2.5 against 1. Checks (the designer's, docs/autonomy-a2b.md
 * 20.4): |x| at HOVER + 3 s, lift-off + 15 and + 45 s smaller than with kmax 1 at D >= 2 m wherever the observer can
 * track (D (1 + eD) <= 6.65 m, its gMin 15 px/m; D 4 m eD +1.0 is not tracked until the drone drifts in: never worse
 * there), never worse at D 1 m (<= +0.01 m); no oscillation: with the camera noise off the hold's 0.3-1.5 Hz
 * peak-to-peak over lift-off + 20..45 s < 0.3 deg in every cell (eD +1.0 at D 1 m included); with the logged noise
 * (0.03 /s per frame, the hovers' own) the band carries the noise times s: its rms < 0.16 deg and its peak-to-peak
 * < 1.25 deg (measured 0.87-1.12 at eD +1.0 where s is 1.6-2.5 x the scene's need). Then the mission's moves and
 * lean at both kmax (a MOVE takes s back to 1). */
static void testTakeoffGrid(void)
{
  static const float Ds[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  static const float eDs[3] = {-0.3f, 0.0f, 1.0f};
  int better = 0, nBetter = 0, notWorse = 0, nNear = 0;
  double worstNear = -9.0, bandQuiet = 0.0, bandNoisy = 0.0, rmsNoisy = 0.0;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      gridCell_t a, b, q;
      gridRun(Ds[i], eDs[j], 1.0f, false, &a);
      gridRun(Ds[i], eDs[j], CAM_HOLD_GAIN_KMAX_DEFAULT, false, &b);
      gridRun(Ds[i], eDs[j], CAM_HOLD_GAIN_KMAX_DEFAULT, true, &q);
      bandQuiet = fmax(bandQuiet, q.bandMax);
      bandNoisy = fmax(bandNoisy, b.bandMax);
      rmsNoisy = fmax(rmsNoisy, b.rmsBand);
      printf("info: gain grid D %.0f m eD %+.1f: |x| HOVER+3 %.2f -> %.2f, lift-off+8 %.2f -> %.2f, +15 %.2f -> %.2f, "
             "+45 %.2f -> %.2f m, max over 15 s %.2f -> %.2f m (s at +15 s %.2f); the hold over 20-45 s: p2p %.2f -> "
             "%.2f deg, 0.3-1.5 Hz p2p %.2f -> %.2f deg (rms %.3f -> %.3f; without the camera noise %.2f)\n", Ds[i],
             eDs[j], a.xH3, b.xH3, a.xL8, b.xL8, a.xL15, b.xL15, a.xL45, b.xL45, a.max15, b.max15, b.sL15, a.p2pMax,
             b.p2pMax, a.bandMax, b.bandMax, a.rmsBand, b.rmsBand, q.bandMax);
      const bool tracked = Ds[i] * (1.0f + eDs[j]) <= AUTO_MV_FY_PX / 15.0f;
      if (Ds[i] >= 2.0f && tracked) {
        nBetter++;
        better += b.xH3 < a.xH3 && b.xL15 < a.xL15 && b.xL45 < a.xL45;
      } else {
        nNear++;
        const double d = fmax(b.xH3 - a.xH3, fmax(b.xL15 - a.xL15, b.xL45 - a.xL45));
        worstNear = fmax(worstNear, d);
        notWorse += d <= 0.01;
      }
    }
  }
  CHECK(better == nBetter && nBetter == 8, "gain scale: |x| at HOVER + 3 s, lift-off + 15 and + 45 s smaller than "
        "with kmax 1 at D 2 / 3 / 4 m where the observer tracks (%d of %d cells)", better, nBetter);
  CHECK(notWorse == nNear, "gain scale: never worse at D 1 m and where the observer cannot track (%d of %d cells; "
        "worst %+.3f m)", notWorse, nNear, worstNear);
  CHECK(bandQuiet < 0.3, "gain scale: no oscillation: without the camera noise the hold's 0.3-1.5 Hz peak-to-peak "
        "over lift-off + 20..45 s %.2f deg (< 0.3) in every cell", bandQuiet);
  CHECK(rmsNoisy < 0.16 && bandNoisy < 1.25, "gain scale: with the hovers' camera noise the hold's 0.3-1.5 Hz band "
        "rms %.3f deg (< 0.16), peak-to-peak %.2f deg (< 1.25)", rmsNoisy, bandNoisy);

  /* the mission (moves, a YAW) at both kmax: the same moves and lean */
  float errMax[2] = {0, 0}, leanMax[2] = {0, 0}, errP95[2] = {0, 0};
  int okN[2] = {0, 0};
  for (int kk = 0; kk < 2; kk++) {
    const float kSave = simKmax;
    simKmax = kk ? CAM_HOLD_GAIN_KMAX_DEFAULT : 1.0f;
    static float errs[16 * 5];
    int ne = 0;
    for (uint32_t i = 0; i < 16; i++) {
      scen_t sc = {.seed = 100u + i, .x0 = 0.0f, .y0 = 1.0f, .psi0 = 1.5708f, .eDmax = 0.10f, .trim = 0.3f,
                   .mission = true, .camLossAtMoveS = -1.0f, .battDropAtS = -1.0f};
      result_t r;
      runSim(&sc, &r);
      okN[kk] += !r.killed && r.landWhy == AUTO_RS_LAND_CMD && r.endWhy == AUTO_RS_TOUCHDOWN && r.ncmd == 5;
      leanMax[kk] = fmaxf(leanMax[kk], r.maxLean);
      for (int c = 0; c < r.ncmd; c++) {
        for (int a = 0; a < 3; a++) {
          if (r.want[c][a] != 0.0f && ne < 16 * 5) {
            errs[ne++] = fabsf(r.got[c][a] - r.want[c][a]) / fabsf(r.want[c][a]);
          }
        }
      }
    }
    qsort(errs, (size_t)ne, sizeof(errs[0]), cmpf);
    errMax[kk] = errs[ne - 1];
    errP95[kk] = errs[ne * 19 / 20];
    simKmax = kSave;
  }
  printf("info: gain scale, the mission (16 seeds): moves relative error p95 %.0f %% -> %.0f %%, max %.0f %% -> "
         "%.0f %%; lean max %.2f -> %.2f deg; LAND by the Mac %d -> %d (kmax 1 -> %.1f)\n", errP95[0] * 100.0f,
         errP95[1] * 100.0f, errMax[0] * 100.0f, errMax[1] * 100.0f, leanMax[0], leanMax[1], okN[0], okN[1],
         CAM_HOLD_GAIN_KMAX_DEFAULT);
  CHECK(okN[1] == 16 && errP95[1] <= errP95[0] + 0.03f && errMax[1] <= errMax[0] + 0.05f &&
        leanMax[1] <= leanMax[0] + 0.3f, "gain scale: the mission's moves and lean unchanged (p95 %.0f / %.0f %%, max "
        "%.0f / %.0f %%, lean %.2f / %.2f deg)", errP95[1] * 100.0f, errP95[0] * 100.0f, errMax[1] * 100.0f,
        errMax[0] * 100.0f, leanMax[1], leanMax[0]);
}


/* ---- long moves and the planned move from take-off (2026-10-14, docs/autonomy-a2b.md 21) ------------------------ */


typedef struct {
  int n, refused, done, prog, shrt, front, other, bad;
  float minWall, hExc, hErr, hExc20, errSum, trackMin;
  int nErr;
} lmAgg_t;

static void lmAdd(lmAgg_t *a, const scen_t *sc, const result_t *r, int16_t f)
{
  a->n++;
  a->bad += r->killed || !r->touchdown || r->landWhy != AUTO_RS_LAND_CMD || r->endWhy != AUTO_RS_TOUCHDOWN;
  if (r->oneWhy != 0) {
    a->refused++;
    return;
  }
  switch (r->oneEnd) {
    case AUTO_MVE_DONE: a->done++; break;
    case AUTO_MVE_PROGRESS: a->prog++; break;
    case AUTO_MVE_SHORT: a->shrt++; break;
    case AUTO_MVE_FRONT: a->front++; break;
    default: a->other++; break;
  }
  a->minWall = fminf(a->minWall, r->minWall);
  if (r->oneEnd == AUTO_MVE_DONE) {
    a->trackMin = fminf(a->trackMin, r->trackMin);   /* how near the frozen-view rule a completed move came */
  }
  a->hExc = fmaxf(a->hExc, r->hExcMove);
  a->hErr = fmaxf(a->hErr, r->hErrMove);
  if (f >= 300) {
    a->hExc20 = fmaxf(a->hExc20, fmaxf(r->hExcMove, r->hErrMove));
  }
  if (r->oneEnd == AUTO_MVE_DONE && sc->kFix == 1.0f) {
    a->errSum += fabsf(r->mvF - 0.01f * (float)f) / (0.01f * (float)f);
    a->nErr++;
  }
}

static float lmFail(const lmAgg_t *a)
{
  const int acc = a->n - a->refused;
  return acc > 0 ? 100.0f * (float)(a->prog + a->shrt + a->front) / (float)acc : 0.0f;
}

/* The long-move grid (docs/autonomy-a2b.md 21.6, the spec's H5): one MOVE forward 3 s into the wait, the scene D
 * {3.5, 4.0, 4.7, 5.5} m ahead (a 12 x 12 m room), F {1.5, 2.5, 3.0, 4.0} m capped by the Mac's floor (0.60 m at
 * k 0.6; at the observed D per seed), the expansion scale k {0.6, 1.0, 1.6}, the plant's drag {0.15, 0.30, 0.45} /s,
 * kmax {1, 2.5}, 16 seeds; LAND 2 s after its end. Checks: PROGRESS + SHORT + FRONT <= 3 % of the accepted at D <=
 * 4.7 m and no worse than at kmax 1; the true end >= 0.40 m from the scene; never KILLED, every run a LAND by the Mac
 * and a touchdown; the height within 0.10 m (true and estimated) through the moves of 3 m and more (17-22 s). A few
 * runs (seed 0, k 1, drag 0.30, kmax 2.5) write the hold's output for tools/hold_band.py (SIM_HOLD_CSV_DIR). */
static float gRatioGrid;   /* the grid's largest camera speed over-read (gLatch 1) */

static void testLongMovesGrid(void)
{
  static const float ds[4] = {3.5f, 4.0f, 4.7f, 5.5f};
  static const int16_t fs[4] = {150, 250, 300, 400};
  static const float ks[3] = {0.6f, 1.0f, 1.6f};
  static const float drags[3] = {0.15f, 0.30f, 0.45f};
  const char *csvDir = getenv("SIM_HOLD_CSV_DIR");
  const float kSave = simKmax;
  lmAgg_t byD[2][4], tot[2];
  memset(byD, 0, sizeof(byD));
  memset(tot, 0, sizeof(tot));
  for (int q = 0; q < 2; q++) {
    for (int i = 0; i < 4; i++) {
      byD[q][i].minWall = byD[q][i].trackMin = 9.0f;
    }
    tot[q].minWall = tot[q].trackMin = 9.0f;
  }
  printf("info: long moves (docs/autonomy-a2b.md 21.6): D x F (capped) x k x drag, 16 seeds; per cell kmax 2.5 / 1: "
         "PROGRESS + SHORT + FRONT of the accepted\n");
  for (int di = 0; di < 4; di++) {
    const float d = ds[di];
    const int16_t cap = macFwdMax(d);
    int16_t prevF = 0;
    for (int fi = 0; fi < 4; fi++) {
      const int16_t f = fs[fi] < cap ? fs[fi] : cap;
      if (f == prevF) {
        continue;
      }
      prevF = f;
      lmAgg_t cell[2];
      memset(cell, 0, sizeof(cell));
      cell[0].minWall = cell[1].minWall = cell[0].trackMin = cell[1].trackMin = 9.0f;
      for (int q = 0; q < 2; q++) {
        simKmax = q ? 1.0f : CAM_HOLD_GAIN_KMAX_DEFAULT;
        for (int ki = 0; ki < 3; ki++) {
          for (int gi = 0; gi < 3; gi++) {
            for (uint32_t sd = 0; sd < 16; sd++) {
              scen_t sc = baseScen(30000u + 1000u * (uint32_t)di + 100u * (uint32_t)fi + 16u * (uint32_t)(3 * ki + gi)
                                   + sd, 6.0f - d);
              sc.room = 6.0f;
              sc.oneMove = true;
              sc.moveAtS = 3.0f;
              sc.kFix = ks[ki];
              sc.drag = drags[gi];
              sc.oneF = f;
              char path[512];
              if (csvDir && q == 0 && ki == 1 && gi == 1 && sd == 0) {
                snprintf(path, sizeof(path), "%s/long-D%.1f-F%d.csv", csvDir, d, f);
                sc.csv = path;
              }
              sc.macCap = true;   /* the Mac caps it at the D it reads (the observer's, with its error) */
              sc.gLatch = 1;      /* the observer's latched G, its gain following the move (21.9) */
              result_t r;
              runSim(&sc, &r);
              gRatioGrid = fmaxf(gRatioGrid, r.gRatioMax);
              if (getenv("SIM_LONG_DEBUG") && r.oneWhy == 0 && r.oneEnd != AUTO_MVE_DONE) {
                const float dNow = r.endDLat0 - fmaxf(r.endFwdM, 0.0f);
                printf("dbg D %.1f F %d k %.1f drag %.2f kmax %.1f: %s after %.1f s, measured lat %+.4f (x D0 %+.3f m, "
                       "x D_now %+.3f m) fwd %.4f (%.3f m, target %.3f m); true right %+.3f fwd %+.3f, wall %.2f\n", d,
                       r.oneF, ks[ki], drags[gi], simKmax, autoMvEndName(r.oneEnd), r.oneEndS, r.endPLat,
                       r.endPLat * r.endDLat0, r.endPLat * dNow, r.endPFwd, r.endD0 * (1.0f - expf(-r.endPFwd)),
                       r.endFwdM, r.endTrueR, r.endTrueF, r.endWall);
              }
              lmAdd(&cell[q], &sc, &r, r.oneF);
              lmAdd(&byD[q][di], &sc, &r, r.oneF);
              lmAdd(&tot[q], &sc, &r, r.oneF);
            }
          }
        }
      }
      printf("info:   D %.1f m F %.2f m: %3d moves, refused %d / %d, DONE %3d / %3d, PROGRESS %d / %d, SHORT %d / %d, "
             "FRONT %d / %d: %.1f / %.1f %%; nearest the scene %.2f m; height excursion <= %.3f m, error <= %.3f m; "
             "travel error (DONE, k 1) %.0f / %.0f %%\n", d, f * 0.01f, cell[0].n, cell[0].refused, cell[1].refused,
             cell[0].done, cell[1].done, cell[0].prog, cell[1].prog, cell[0].shrt, cell[1].shrt, cell[0].front,
             cell[1].front, lmFail(&cell[0]), lmFail(&cell[1]), fminf(cell[0].minWall, cell[1].minWall),
             fmaxf(cell[0].hExc, cell[1].hExc), fmaxf(cell[0].hErr, cell[1].hErr),
             cell[0].nErr ? 100.0f * cell[0].errSum / (float)cell[0].nErr : 0.0f,
             cell[1].nErr ? 100.0f * cell[1].errSum / (float)cell[1].nErr : 0.0f);
    }
  }
  simKmax = kSave;
  lmAgg_t near[2];
  memset(near, 0, sizeof(near));
  for (int q = 0; q < 2; q++) {
    for (int di = 0; di < 3; di++) {   /* D <= 4.7 m */
      near[q].n += byD[q][di].n;
      near[q].refused += byD[q][di].refused;
      near[q].prog += byD[q][di].prog;
      near[q].shrt += byD[q][di].shrt;
      near[q].front += byD[q][di].front;
    }
  }
  for (int di = 0; di < 4; di++) {
    printf("info: long moves at D %.1f m: kmax 2.5 %.1f %% (%d of %d accepted), kmax 1 %.1f %% (%d of %d)\n", ds[di],
           lmFail(&byD[0][di]), byD[0][di].prog + byD[0][di].shrt + byD[0][di].front,
           byD[0][di].n - byD[0][di].refused, lmFail(&byD[1][di]),
           byD[1][di].prog + byD[1][di].shrt + byD[1][di].front, byD[1][di].n - byD[1][di].refused);
  }
  printf("info: long moves, all %d runs per kmax: kmax 2.5 refused %d, DONE %d, PROGRESS %d, SHORT %d, FRONT %d, other "
         "%d; kmax 1 refused %d, DONE %d, PROGRESS %d, SHORT %d, FRONT %d, other %d; nearest the scene %.2f m; height "
         "through the >= 3 m moves within %.3f m; the frozen-view rule's ratio (measured / reference forward) in the "
         "completed moves at its lowest %.2f / %.2f (it trips under %.2f)\n", tot[0].n, tot[0].refused, tot[0].done, tot[0].prog, tot[0].shrt,
         tot[0].front, tot[0].other, tot[1].refused, tot[1].done, tot[1].prog, tot[1].shrt, tot[1].front,
         tot[1].other, fminf(tot[0].minWall, tot[1].minWall), fmaxf(tot[0].hExc20, tot[1].hExc20), tot[0].trackMin,
         tot[1].trackMin, (double)AUTO_MV_FROZEN_FRAC);
  CHECK(lmFail(&near[0]) <= 3.0f && lmFail(&near[0]) <= lmFail(&near[1]),
        "long moves at D <= 4.7 m: PROGRESS + SHORT + FRONT %.1f %% of the accepted at kmax 2.5 (<= 3 %%, and no worse "
        "than kmax 1's %.1f %%)", lmFail(&near[0]), lmFail(&near[1]));
  /* (>= 0.40 m with the Mac's 0.60 m floor until 2026-10-17; 0.24-0.27 m at its 0.35 m: k 0.6, the observer's D -10 %) */
  CHECK(tot[0].minWall >= 0.20f && tot[1].minWall >= 0.20f, "long moves: the true end >= 0.20 m from the scene (%.2f "
        "/ %.2f m; the Mac's floor 0.35 m at k 0.6)", tot[0].minWall, tot[1].minWall);
  CHECK(tot[0].bad == 0 && tot[1].bad == 0, "long moves: never KILLED, every run a LAND by the Mac and a touchdown "
        "(%d / %d not)", tot[0].bad, tot[1].bad);
  /* the true height over as long with no move (the longest move and its settle, 25 s: the plain hover's own
   * estimator drift; LAND after) */
  float hHover = 0.0f;
  for (uint32_t sd = 0; sd < 16; sd++) {
    scen_t sc = baseScen(30900u + sd, 6.0f - 4.7f);
    sc.room = 6.0f;
    sc.hoverWinS = 25.0f;
    result_t r;
    runSim(&sc, &r);
    hHover = fmaxf(hHover, r.hExcHover);
  }
  printf("info: long moves: the true height's excursion from the move's start through the 17-22 s moves <= %.3f m "
         "(the controller's own error against its reference <= %.3f m); a 25 s plain hover from the same point drifts "
         "<= %.3f m (the camera height estimate's offset); the camera speed's over-read with the latched G (its gain "
         "following the move) <= %.2f x\n", fmaxf(tot[0].hExc20, tot[1].hExc20), fmaxf(tot[0].hErr, tot[1].hErr),
         hHover, gRatioGrid);
  CHECK(tot[0].hErr < 0.10f && tot[1].hErr < 0.10f, "long moves: the controller's height error under 0.10 m through "
        "every move (%.3f / %.3f m)", tot[0].hErr, tot[1].hErr);
  /* +0.05 m until 2026-10-17 (0.266 m): the Mac's 0.35 m floor lets the long approaches end nearer (0.24-0.30 m), where
   * the observer's G, scaled by the measured progress at k 0.6, over-reads by (D0 / D_now)^(1 - k): 0.335 m (docs/
   * autonomy-a2b.md 32.6, an open point for the flights) */
  CHECK(fmaxf(tot[0].hExc20, tot[1].hExc20) <= hHover + 0.10f, "long moves: the true height drifts no more than in a "
        "plain hover of the same length (+0.10 m; %.3f vs %.3f m)", fmaxf(tot[0].hExc20, tot[1].hExc20), hHover);
}

/* The observer's latched G through the largest moves (2026-10-15 review, docs/autonomy-a2b.md 21.9; its stale_drv2):
 * F the Mac's largest at D 3.5 / 4.0 / 4.7 / 5.5 m, k 0.6 / 1 / 1.6, drag 0.15 / 0.30 / 0.45, 8 seeds, kmax 2.5; the
 * tracker as the observer's (a G after 2 s of excitation) and the review's pessimistic one (none in HOVER). Without
 * the gain following the move (gLatch 2, 148af89) the camera's speed below ~1 s reads up to D0 / D_now too fast
 * (7-11 x at k 0.6; the review: 23 of 288 automatic LANDs, PALM and VZ, the height error up to 0.235 m); with it
 * (gLatch 1, the firmware) every run must end in the Mac's LAND with the height error under 0.10 m, either tracker. */
static void testLatchedG(void)
{
  static const float ds[4] = {3.5f, 4.0f, 4.7f, 5.5f};
  static const float ks[3] = {0.6f, 1.0f, 1.6f};
  static const float drags[3] = {0.15f, 0.30f, 0.45f};
  int bad[2][3] = {{0}}, badK[2][3][3] = {{{0}}}, n = 0;
  float hErr[2][3] = {{0}}, ratio[2][3] = {{0}}, under[2][3] = {{9.0f, 9.0f, 9.0f}, {9.0f, 9.0f, 9.0f}};
  for (int pess = 0; pess < 2; pess++) {
    for (int mode = 1; mode <= 2; mode++) {
      for (int di = 0; di < 4; di++) {
        for (int ki = 0; ki < 3; ki++) {
          for (int gi = 0; gi < 3; gi++) {
            for (uint32_t sd = 0; sd < 8; sd++) {
              scen_t sc = baseScen(50000u + 1000u * (uint32_t)di + 16u * (uint32_t)(3 * ki + gi) + sd, 6.0f - ds[di]);
              sc.room = 6.0f;
              sc.oneMove = true;
              sc.moveAtS = 3.0f;
              sc.kFix = ks[ki];
              sc.drag = drags[gi];
              sc.oneF = AUTO_MV_FWD_MAX_CM;
              sc.macCap = true;
              sc.gLatch = mode;
              sc.gNoRetake = pess != 0;
              result_t r;
              runSim(&sc, &r);
              n += pess == 0 && mode == 1;
              const int b = r.killed || !r.touchdown || r.landWhy != AUTO_RS_LAND_CMD;
              bad[pess][mode] += b;
              badK[pess][mode][ki] += b;
              hErr[pess][mode] = fmaxf(hErr[pess][mode], r.hErrMove);
              ratio[pess][mode] = fmaxf(ratio[pess][mode], r.gRatioMax);
              if (ks[ki] == 1.6f) {   /* the over-correction at k > 1: the speed reads slow */
                under[pess][mode] = fminf(under[pess][mode], r.gRatioMin);
              }
              if (b && getenv("SIM_GLATCH_DEBUG")) {
                printf("dbg gLatch %d%s D %.1f k %.1f drag %.2f seed %u: F %d end %s at %.1f s, LAND %s at air %.1f s, "
                       "over-read max %.1f\n", mode, pess ? " (no G in HOVER)" : "", ds[di], ks[ki], drags[gi],
                       (unsigned)sd, r.oneF, autoMvEndName(r.oneEnd), r.oneEndS, autoReasonName(r.landWhy),
                       r.landAirMs * 0.001f, r.gRatioMax);
              }
            }
          }
        }
      }
      printf("info: the latched G%s, %s, %d runs: not a clean Mac LAND %d (k 0.6 / 1 / 1.6: %d / %d / %d), the height "
             "error <= %.3f m, the camera speed read in HOVER %.2f..%.2f x\n",
             pess ? " (the review's tracker: no G in HOVER)" : " (G after 2 s of excitation)",
             mode == 1 ? "the gain following the move (the firmware)" : "without it (148af89)", n, bad[pess][mode],
             badK[pess][mode][0], badK[pess][mode][1], badK[pess][mode][2], hErr[pess][mode], under[pess][mode],
             ratio[pess][mode]);
    }
  }
  CHECK(bad[0][1] == 0 && bad[1][1] == 0 && hErr[0][1] < 0.10f && hErr[1][1] < 0.10f, "the latched G with the gain "
        "following the move: every run the Mac's LAND (%d / %d not), the height error under 0.10 m (%.3f / %.3f), "
        "either tracker", bad[0][1], bad[1][1], hErr[0][1], hErr[1][1]);
  CHECK(bad[1][2] > 0 && ratio[1][2] > 2.0f * ratio[1][1], "the model shows the review's finding without it (the "
        "pessimistic tracker: %d not clean, over-read %.1f x against %.2f x)", bad[1][2], ratio[1][2], ratio[1][1]);
}

/* Below AUTO_MV_K_LOW (2026-10-15 review, docs/autonomy-a2b.md 21.9; its klow_drv): the floor allows for k down to
 * 0.6 and nothing on the drone measures k (the frozen-view rule's ratio stays ~1, the front guard reads the same
 * camera). The nearest true approach to the scene at k 0.5 / 0.4 / 0.3 with the forward part capped at 1.5 m (b6a54da)
 * and at the Mac's floor (this build, up to 4 m), D 3.5 / 4.7 / 5.5 m, 3 drags, 8 seeds: documented, not a gate (the
 * runbook keeps forward moves at 1.5-2 m in a room until one long move there has measured k >= 0.7 by photo). */
static void testBelowKLow(void)
{
  static const float ds[3] = {3.5f, 4.7f, 5.5f};
  static const float ks[3] = {0.5f, 0.4f, 0.3f};
  static const int16_t fs[2] = {150, AUTO_MV_FWD_MAX_CM};
  for (int di = 0; di < 3; di++) {
    for (int ki = 0; ki < 3; ki++) {
      float minW[2] = {9.0f, 9.0f};
      int near[2] = {0}, acc[2] = {0};
      for (int fi = 0; fi < 2; fi++) {
        for (int gi = 0; gi < 3; gi++) {
          static const float drags[3] = {0.15f, 0.30f, 0.45f};
          for (uint32_t sd = 0; sd < 8; sd++) {
            scen_t sc = baseScen(60000u + 1000u * (uint32_t)di + 100u * (uint32_t)ki + 16u * (uint32_t)gi + sd,
                                 6.0f - ds[di]);
            sc.room = 6.0f;
            sc.oneMove = true;
            sc.moveAtS = 3.0f;
            sc.kFix = ks[ki];
            sc.drag = drags[gi];
            sc.oneF = fs[fi];
            sc.macCap = true;
            sc.gLatch = 1;
            result_t r;
            runSim(&sc, &r);
            if (r.oneWhy != 0) {
              continue;
            }
            acc[fi]++;
            minW[fi] = fminf(minW[fi], r.minWall);
            near[fi] += r.minWall < 0.40f;
          }
        }
      }
      printf("info: below K_LOW, D %.1f m k %.1f: forward <= 1.5 m nearest the scene %.2f m (%d of %d under 0.40 m); "
             "the Mac's floor (<= 4 m) %.2f m (%d of %d)\n", ds[di], ks[ki], minW[0], near[0], acc[0], minW[1], near[1],
             acc[1]);
    }
  }
}

/* The planned take-off move at its largest (2026-10-15 review, docs/autonomy-a2b.md 21.9): the drone flies the plan
 * with the Mac's floor (AUTO_PLAN_MIN_FRONT_CM: 0.35 m since 2026-10-17, 0.60 before), at the observer's D (+-10 %) at
 * the wait's start, k 0.6 (the floor's), 3 drags, 16 seeds, D 2.5 .. 5.5 m: the true end >= 0.20 m from the scene, as
 * the Mac-capped grid (0.40 m with the 0.60 m floor; with the drone's 0.50 m it was 0.34-0.40 m), never KILLED; 2 cm
 * more is refused TOO_CLOSE. */
static void testPlanFloorSim(void)
{
  static const float ds[5] = {2.5f, 3.5f, 4.0f, 4.7f, 5.5f};
  static const float drags[3] = {0.15f, 0.30f, 0.45f};
  float minW = 9.0f;
  int n = 0, acc = 0, moreN = 0, moreRefused = 0, killed = 0, notTracked = 0;
  for (int di = 0; di < 5; di++) {
    float minD = 9.0f;
    for (int gi = 0; gi < 3; gi++) {
      for (uint32_t sd = 0; sd < 16; sd++) {
        scen_t sc = baseScen(70000u + 1000u * (uint32_t)di + 16u * (uint32_t)gi + sd, 6.0f - ds[di]);
        sc.room = 6.0f;
        sc.oneMove = true;
        sc.kFix = AUTO_MV_K_LOW;
        sc.drag = drags[gi];
        sc.gLatch = 1;
        sc.planF = 1;                 /* pass 1: the observer's D at the wait's start (the take-off is the same) */
        result_t r0;
        runSim(&sc, &r0);
        const float d = r0.d0Obs;
        if (!(d > 0.0f) || r0.oneWhy != 0) {
          notTracked++;
          continue;
        }
        /* the plan's largest at that D: autoMotionMovePlan's floor, as it computes it */
        const float floorM = (float)AUTO_PLAN_MIN_FRONT_CM * 0.01f;
        int16_t fc = AUTO_MV_FWD_MAX_CM;
        for (;; fc--) {
          const float fm = (float)fc / 100.0f;
          if (fc <= 1 || (d - fm >= AUTO_MV_LAT_GUARD_D_MIN_M &&
                          d * expf(autoMvLn((d - fm) / d) * (1.0f / AUTO_MV_K_LOW)) >= floorM)) {
            break;
          }
        }
        sc.planF = fc;
        result_t r;
        runSim(&sc, &r);
        if (r.oneWhy != 0) {
          sc.planF = --fc;            /* the float rounding of the floor at its boundary */
          runSim(&sc, &r);
        }
        n++;
        killed += r.killed;
        if (r.oneWhy == 0) {
          acc++;
          minW = fminf(minW, r.minWall);
          minD = fminf(minD, r.minWall);
        }
        if (fc < AUTO_MV_FWD_MAX_CM - 1) {   /* (near 400 cm the scene does not bind: 401 is RANGE at the TAKEOFF) */
          sc.planF = (int16_t)(fc + 2);       /* (2 cm: the host's float rounding of the floor at its boundary) */
          result_t r1;
          runSim(&sc, &r1);
          moreN++;
          moreRefused += r1.oneWhy == AUTO_MVW_TOO_CLOSE;
        }
      }
    }
    printf("info: the plan at its largest (the %.2f m floor), D %.1f m k 0.6: nearest the scene %.2f m\n",
           (float)AUTO_PLAN_MIN_FRONT_CM * 0.01f, ds[di], minD);
  }
  printf("info: the plan at its largest: %d runs, %d accepted, 2 cm more TOO_CLOSE %d of %d (the rest near 4 m), "
         "nearest the scene %.2f m, killed %d (no tracked D at the wait %d)\n", n, acc, moreRefused, moreN, minW,
         killed, notTracked);
  CHECK(acc == n && n >= 200 && moreRefused == moreN && minW >= 0.20f && killed == 0, "the planned move at its "
        "largest (the Mac's 0.35 m floor at the observer's D): accepted %d of %d, 2 cm more TOO_CLOSE %d of %d, the "
        "true end >= 0.20 m from the scene at k 0.6 (%.2f m), never KILLED", acc, n, moreRefused, moreN, minW);
}

/* The planned move from take-off and the observer's G transient after the HOVER entry, G x (1 + 0.25 e^(-t / 0.4 s))
 * (docs/autonomy-a2b.md 21.4): the plan at the wait's start ends within +-10 % of plan, as the same Mac MOVE sent
 * 0.4 s later (+-5 % of it); the same move started at the HOVER entry ends 15-25 % long (why it is not done there).
 * D 4.7 m, k 1, the observer's D exact but for the transient, forward 1.5 and 2.5 m, 8 seeds each. */
/* A physical obstacle the camera does not see (2026-10-15 verification of "Claude decides", the verifier's wall sim):
 * the drone hovers until Claude decides (the Mac's LAND only 45 s into the wait, never 3 s after the contact).
 * (a) A MOVE forward 60 / 80 / 100 cm into a rigid plane 0.25-0.35 m ahead (restitution 0.2-0.4): the contact stops
 * the move (a brake), and the hold's anchor goes AUTO_MV_CONTACT_BACK_M back along the move. The spec's test (f):
 * from 3 s after the contact (the hush, the brake) to the LAND the gap to the plane never comes back below its value
 * then minus 0.05 m, the drone never presses against the plane again, and nothing lands or cuts it. Before the anchor
 * moved (29f8091) 18 of 25 such runs were back within 0.01 m of the plane 7-18 s after the contact and stayed there.
 * (b) No motion: a lateral push (1.0-1.5 m/s^2 for 0.5 s) into a plane 0.15-0.25 m to the right: the hold
 * re-anchors where the bounce left the drone at the hush's end; measured (info), with no LAND and no cut. */
static void wallScen(scen_t *sc, uint32_t seed)
{
  memset(sc, 0, sizeof(*sc));
  sc->seed = seed;
  sc->x0 = 0.0f;
  sc->y0 = 1.0f;
  sc->psi0 = 1.5708f;   /* facing +y: the room's wall 2 m ahead is the camera's scene */
  sc->eDmax = 0.10f;
  sc->trim = 0.3f;
  sc->camLossAtMoveS = -1.0f;
  sc->battDropAtS = -1.0f;
  sc->toCm = 100;
  sc->wallLandAtS = 45.0f;
}

static void testWallSim(void)
{
  int n = 0, nHit = 0, nBack = 0, nPress = 0, nBad = 0, nNear = 0, nLate = 0, landHits = 0, landHitRuns = 0, notBlind = 0;
  float worstDrop = -9.0f, minLate = 9.0f, minAt20 = 9.0f, worstTd = 0.0f, worstRise = 0.0f, hitV = 0.0f, hitH = 0.0f;
  for (int k = 0; k < 27; k++) {
    scen_t sc;
    wallScen(&sc, 4000u + (uint32_t)k);
    sc.wallMode = 1;
    sc.wallY = 1.0f + 0.30f + 0.05f * (float)(k % 3);
    sc.wallE = 0.2f + 0.1f * (float)((k / 3) % 3);
    sc.wallMoveAtS = 10.0f;
    sc.wallFwdCm = (int16_t)(60 + 20 * ((k / 9) % 3));
    result_t r;
    runSim(&sc, &r);
    n++;
    printf("info: wall seed %u: plane %.2f m ahead, e %.1f, MOVE forward %d: CONTACT at %.1f s, contacts %d, hits %d, "
           "gap 3 s on %.3f m, lowest after %.3f m, pressed %.2f s; killed %d, end %s; gap every 0.5 s:",
           (unsigned)sc.seed, sc.wallY - 1.0f, sc.wallE, sc.wallFwdCm, r.wallT, r.contacts, r.wallHits, r.wallGap3,
           r.wallMinLate, r.wallPressLate, r.killed, autoReasonName(r.endWhy));
    for (int i = 0; i < r.wallGapN; i += 2) {
      printf(" %.2f", r.wallGap[i]);
    }
    printf("\n");
    nBad += r.killed || r.endWhy != AUTO_RS_TOUCHDOWN || r.landWhy != AUTO_RS_LAND_CMD;
    if (r.wallT < 0.0f) {
      continue;   /* (the MOVE refused TOO_CLOSE, or the hover drifted into the plane first: case (b); or it missed) */
    }
    /* the Mac's LAND 35 s after the contact: blind (user decision (B)), the hold kept down to the touchdown */
    landHits += r.wallHitsLand;
    landHitRuns += r.wallHitsLand > 0;
    hitV = fmaxf(hitV, r.wallLandHitV);
    hitH = fmaxf(hitH, r.wallLandHitH);
    notBlind += r.vzLandsBlind != r.vzLands || r.vzLandsCollided != 0 || r.blockedInAir;
    worstTd = fmaxf(worstTd, r.impactV);
    worstRise = fmaxf(worstRise, r.hRiseLand);
    nHit++;
    const float drop = r.wallGap3 - r.wallMinLate;
    const float at20 = r.wallGapN > 40 ? r.wallGap[40] : 0.0f;
    worstDrop = fmaxf(worstDrop, drop);
    minLate = fminf(minLate, r.wallMinLate);
    minAt20 = fminf(minAt20, at20);
    nBack += drop > 0.05f;
    nPress += r.wallPressLate > 0.0f;
    nNear += r.wallMinLate < fminf(r.wallGap3, 0.10f) - 0.005f;   /* (3 s on it may still be backing off) */
    nLate += !(at20 >= 0.12f);
  }
  printf("info: wall (a): %d runs, %d with a CONTACT that stopped the MOVE; from 3 s after it to the LAND (42 s): the "
         "lowest gap %.3f m, pressed again %d, 20 s after it the lowest gap %.3f m; the spec's 22 (f) measure, back "
         "toward the plane by more than 0.05 m from the gap 3 s after the contact: %d runs, the worst %.3f m (the "
         "hold's own wander over 40 s at D 2 m, around the anchor; 29f8091: 24 runs, 0.29 m, 18 pressed)\n", n, nHit,
         minLate, nPress, minAt20, nBack, worstDrop);
  CHECK(nHit >= 20, "the obstacle grid: %d of %d MOVEs stopped by a CONTACT with the plane", nHit, n);
  CHECK(nPress == 0 && nNear == 0, "a MOVE into an obstacle the camera does not see: from 3 s after the CONTACT to "
        "Claude's LAND 42 s later the drone never touches it again, the gap never below min(its value 3 s on, 0.10 m) "
        "(lowest %.3f m; %d pressed, %d nearer)", minLate, nPress, nNear);
  CHECK(nLate == 0, "... the hold does not walk it back: 20 s after the CONTACT the gap >= 0.12 m (lowest %.3f m; the "
        "anchor %.2f m back)", minAt20, (double)AUTO_MV_CONTACT_BACK_M);
  CHECK(nBad == 0, "... nothing landed or cut it: the Mac's LAND at 45 s, a touchdown (%d other ends)", nBad);
  /* (Claude has not backed away here: the hold's forward creep in every LAND, the descent read as image expansion,
   * ~0.2 x the height, took faa18db's camera LAND to within 0.03 m of the plane; the blind LAND's float at the floor adds
   * a few cm) */
  printf("info: wall (a): the Mac's LAND 35 s after the contact, blind: touchdown max %.2f m/s, rise in it max %.3f m; "
         "it touched the plane in %d runs (%d steps), at most %.3f m/s into it, at most %.3f m up\n", worstTd, worstRise,
         landHitRuns, landHits, hitV, hitH);
  CHECK(notBlind == 0 && worstTd <= 1.1f && worstRise <= 0.05f && hitV <= 0.10f && hitH <= 0.20f, "... and that LAND "
        "is the blind one with the hold kept (%d not), a touchdown <= 1.1 m/s (%.2f), no climb (%.3f m); a touch of the "
        "plane in it only at the floor (<= 0.20 m up: %.3f) and slow (<= 0.10 m/s: %.3f)", notBlind, worstTd, worstRise,
        hitH, hitV);
  /* Claude's runbook: back away 0.30 m 5 s after the CONTACT, then the LAND: never back into the plane */
  int nB = 0, bHits = 0, bBad = 0, bNotBlind = 0;
  float bTd = 0.0f, bGap = 9.0f;
  for (int k = 0; k < 27; k++) {
    scen_t sc;
    wallScen(&sc, 4000u + (uint32_t)k);
    sc.wallMode = 1;
    sc.wallY = 1.0f + 0.30f + 0.05f * (float)(k % 3);
    sc.wallE = 0.2f + 0.1f * (float)((k / 3) % 3);
    sc.wallMoveAtS = 10.0f;
    sc.wallFwdCm = (int16_t)(60 + 20 * ((k / 9) % 3));
    sc.wallBackCm = 30;
    result_t r;
    runSim(&sc, &r);
    if (r.wallT < 0.0f) {
      continue;
    }
    nB++;
    bHits += r.wallMinLate <= 0.002f || r.wallHitsLand > 0;   /* from 3 s after the contact (its own bounces before) */
    bBad += r.killed || r.endWhy != AUTO_RS_TOUCHDOWN || r.landWhy != AUTO_RS_LAND_CMD;
    bNotBlind += r.vzLandsBlind != r.vzLands || r.vzLandsCollided != 0;
    bTd = fmaxf(bTd, r.impactV);
    bGap = fminf(bGap, r.wallMinLate);
  }
  printf("info: wall (a) with Claude backing away 0.30 m 5 s after the CONTACT (%d runs): touched again in %d, the lowest "
         "gap from 3 s after the contact to the LAND %.3f m, touchdown max %.2f m/s\n", nB, bHits, bGap, bTd);
  CHECK(nB >= 20 && bHits == 0 && bBad == 0 && bNotBlind == 0 && bTd <= 1.1f, "the runbook (back away 0.30 m, then "
        "LAND): never back into the plane, the hover nor the blind LAND (%d of %d), a touchdown <= 1.1 m/s (%.2f)",
        bHits, nB, bTd);

  int m = 0, mHit = 0, mPress = 0, mBad = 0;
  for (int k = 0; k < 18; k++) {
    scen_t sc;
    wallScen(&sc, 5000u + (uint32_t)k);
    sc.wallMode = 2;
    sc.wallX = 0.15f + 0.05f * (float)(k % 3);
    sc.wallE = 0.2f + 0.1f * (float)((k / 3) % 3);
    sc.latGust = 1.0f + 0.5f * (float)((k / 9) % 2);
    sc.gustAtS = 12.0f;
    result_t r;
    runSim(&sc, &r);
    m++;
    printf("info: side plane seed %u: %.2f m right, e %.1f, push %.1f m/s^2: CONTACT at %.1f s, contacts %d, gap 3 s "
           "on %.3f m, lowest after %.3f m, pressed %.2f s; killed %d, end %s\n", (unsigned)sc.seed, sc.wallX, sc.wallE,
           sc.latGust, r.wallT, r.contacts, r.wallGap3, r.wallMinLate, r.wallPressLate, r.killed,
           autoReasonName(r.endWhy));
    mBad += r.killed || r.endWhy != AUTO_RS_TOUCHDOWN;
    if (r.wallT >= 0.0f) {
      mHit++;
      mPress += r.wallPressLate > 0.0f;
    }
  }
  printf("info: wall (b), no motion: %d runs, %d with a CONTACT in HOVER, %d pressed against the plane again from 3 s "
         "after it\n", m, mHit, mPress);
  CHECK(mBad == 0, "a push into an obstacle in the hover: never cut, the Mac's LAND ends it (%d other ends)", mBad);
}

static void testPlanTransient(void)
{
  static const int16_t fs[2] = {150, 250};
  int okPlan = 0, okMatch = 0, okHover = 0, n = 0, nHov = 0;
  float pLo = 9, pHi = -9, mLo = 9, mHi = -9, hLo = 9, hHi = -9, pSum = 0.0f, hSum = 0.0f;
  for (int fi = 0; fi < 2; fi++) {
    for (uint32_t sd = 0; sd < 8; sd++) {
      result_t r[3];
      for (int v = 0; v < 3; v++) {
        scen_t sc = baseScen(41000u + 100u * (uint32_t)fi + sd, 6.0f - 4.7f);
        sc.room = 6.0f;
        sc.eDfix = true;
        sc.eDval = 0.0f;
        sc.kFix = 1.0f;
        sc.gTrans = 0.25f;
        sc.oneMove = true;
        if (v == 0) {
          sc.planF = fs[fi];             /* the TAKEOFF's plan: at the wait's start */
        } else if (v == 1) {
          sc.oneF = fs[fi];              /* the Mac's MOVE 0.4 s after the wait's start (plan-via mac) */
          sc.moveAtS = 0.4f;
        } else {
          sc.planF = fs[fi];             /* white-box: at the HOVER entry */
          sc.planAtHover = true;
        }
        runSim(&sc, &r[v]);
      }
      n++;
      const float want = 0.01f * (float)fs[fi];
      /* the travel the move made: from the hold's equilibrium point at its acceptance (the hover's own return to
       * it is not the move's; as the mission measures its commands) to where it held 2 s after its end */
      const float g0 = r[0].mvF - r[0].eqF0, g1 = r[1].mvF - r[1].eqF0, g2 = r[2].mvF - r[2].eqF0;
      const float ePlan = g0 / want - 1.0f, eMac = g1 / want - 1.0f, eHov = g2 / want - 1.0f;
      const float match = g0 / g1 - 1.0f;
      const bool ended0 = r[0].oneEnd == AUTO_MVE_DONE || r[0].oneEnd == AUTO_MVE_SHORT;
      const bool ended1 = r[1].oneEnd == AUTO_MVE_DONE || r[1].oneEnd == AUTO_MVE_SHORT;
      const bool ended2 = r[2].oneWhy == 0 && (r[2].oneEnd == AUTO_MVE_DONE || r[2].oneEnd == AUTO_MVE_SHORT);
      okPlan += r[0].oneWhy == 0 && r[0].planSeqAtWait == 1 && ended0 && fabsf(ePlan) <= 0.15f;
      okMatch += r[1].oneWhy == 0 && ended0 && ended1 && fabsf(match) <= 0.05f;
      pSum += ePlan;
      if (ended2) {   /* (one start at the HOVER entry, before the forward hold settled, ends PROGRESS) */
        nHov++;
        hSum += eHov;
        okHover += eHov > 0.10f;
      }
      pLo = fminf(pLo, ePlan);
      pHi = fmaxf(pHi, ePlan);
      mLo = fminf(mLo, match);
      mHi = fmaxf(mHi, match);
      if (ended2) {
        hLo = fminf(hLo, eHov);
        hHi = fmaxf(hHi, eHov);
      }
      if (getenv("SIM_TRANS_DEBUG")) {
        printf("dbg transient F %d seed %u: plan %s %+.0f %% (D %.2f, eqF %+.2f, mvF %+.2f, mvR %+.2f), mac %s %+.0f %% (D %.2f), "
               "hover %s %+.0f %% (D %.2f, eqF %+.2f, mvF %+.2f)\n", fs[fi], sd, autoMvEndName(r[0].oneEnd), 100 * ePlan,
               r[0].d0Obs, r[0].eqF0, r[0].mvF, r[0].mvR, autoMvEndName(r[1].oneEnd), 100 * eMac, r[1].d0Obs,
               autoMvEndName(r[2].oneEnd), 100 * eHov, r[2].d0Obs, r[2].eqF0, r[2].mvF);
      }
      if (sd == 0) {
        printf("info: G transient, forward %.1f m at D 4.7: the plan %+.2f m (%+.0f %%, the observer's D at the wait's "
               "start %.2f), the Mac's MOVE 0.4 s later %+.2f m (%+.0f %%), at the HOVER entry %+.2f m (%+.0f %%, D "
               "%.2f; %s)\n", want, g0, 100 * ePlan, r[0].d0Obs, g1, 100 * eMac, g2, 100 * eHov,
               r[2].d0Obs, autoMvEndName(r[2].oneEnd));
      }
    }
  }
  printf("info: G transient over %d pairs: the plan %+.0f..%+.0f %% of its command, against the Mac's MOVE 0.4 s later "
         "%+.1f..%+.1f %%, at the HOVER entry %+.0f..%+.0f %%\n", n, 100 * pLo, 100 * pHi, 100 * mLo, 100 * mHi,
         100 * hLo, 100 * hHi);
  const float pMean = pSum / (float)n, hMean = nHov ? hSum / (float)nHov : 0.0f;
  printf("info: G transient: the plan's mean error %+.1f %%, the HOVER entry's %+.1f %% (%d of %d ended DONE / SHORT)\n",
         100 * pMean, 100 * hMean, nHov, n);
  CHECK(okPlan == n && fabsf(pMean) <= 0.10f, "the planned move at the wait's start: accepted on the wait's first "
        "step, DONE or SHORT, within +-15 %% of its command, the mean within +-10 %% (%d of %d; %+.0f..%+.0f %%, mean "
        "%+.1f %%; the sim's far 1.5 m moves themselves err up to ~15 %%, testFarMoves)", okPlan, n, 100 * pLo,
        100 * pHi, 100 * pMean);
  CHECK(okMatch == n, "the plan against the same Mac MOVE 0.4 s later: within +-5 %% (%d of %d; %+.1f..%+.1f %%)",
        okMatch, n, 100 * mLo, 100 * mHi);
  CHECK(okHover == nHov && nHov >= n - 2 && hMean >= 0.15f && hMean <= 0.25f, "the same move at the HOVER entry: "
        "15-25 %% long on average, every one > +10 %% (%d of %d; %+.0f..%+.0f %%, mean %+.1f %%): the rejected start",
        okHover, nHov, 100 * hLo, 100 * hHi, 100 * hMean);
}

/* ---- SET_HEIGHT in the loop (2026-10-15, docs/autonomy-a2b.md 23) ------------------------------------------------
 * A scripted Claude corrects the drifting height estimate from 'photos' (scen_t shLoop): a photo P s after the previous
 * correction's acceptance or the previous photo, X = the true height + N(0, 0.07) clipped +-0.15 + a bias of 0 / +-0.05,
 * h_at the estimate then, sent L s later (8 or 15) when |X - h_at| > 0.10; against the baseline on the same seeds
 * without commands. The camera's speed offset c is the drift (-0.03..+0.03 m/s); a variant reads the vertical speed
 * 25 % off (the camera's vertical scale). 150 s of air, the Mac's LAND then. The error is the true height against the
 * height the drone held at the wait's start (what a correction flies back to: the climb ends within holdBand, 0.05 m,
 * of its target). */
static int cmpf2(const void *a, const void *b)
{
  const float x = *(const float *)a, y = *(const float *)b;
  return x < y ? -1 : x > y;
}

typedef struct {
  int n, killed, cutAir, otherLand, progress, acc, retries, ref[16], shiftRefused;
  float errAbs[40000];
  int errN;
  float errMax, errEndSum, hMax, devCorr, devHover;
} shAgg_t;

static shAgg_t shAgg;   /* (large: static) */

static void shRun(uint16_t cm, float c, int mode, float scale, uint32_t seed, int i, result_t *r)
{
  scen_t sc = baseScen(seed, 1.0f);
  toHeight(&sc, cm);   /* 2.00 m: the 1.20 m take-off cap moved up 0.50 + 0.30 m */
  sc.camVzOff = c;
  sc.waitMs = 120000;
  sc.shLoop = mode;
  sc.shP = cm > 120 ? 7.0f : 12.0f;     /* every 10-15 s at <= 1.2 m, 5-10 s above (the runbook) */
  sc.shL = (i % 2) ? 15.0f : 8.0f;
  sc.shNoise = 0.07f;
  sc.shBias = (i % 3 == 0) ? 0.0f : (i % 3 == 1 ? 0.05f : -0.05f);
  sc.shScale = scale;
  sc.shDurS = 150.0f;
  runSim(&sc, r);
  if (r->killed || r->cutInAir || r->landWhy != AUTO_RS_LAND_CMD || r->shShiftRefused) {
    printf("info:   seed %u mode %d c %+.3f L %.0f bias %+.2f: LAND %s (vz %s) at air %.1f s, highest %.2f m, end err "
           "%+.2f, killed %d cut %d\n", (unsigned)seed, mode, (double)c, sc.shL, sc.shBias, autoReasonName(r->landWhy),
           vzReasonName((vzReason_t)r->vzLandWhy), r->landAirMs / 1000.0f, r->shHMax, r->shErrEnd, r->killed,
           r->cutInAir);
  }
}

static void shAdd(shAgg_t *a, const result_t *r)
{
  a->n++;
  a->killed += r->killed;
  a->cutAir += r->cutInAir;
  a->otherLand += r->landWhy != AUTO_RS_LAND_CMD;
  a->progress += r->shProgress;
  a->acc += r->shAcc;
  a->retries += r->shRetries;
  a->shiftRefused += r->shShiftRefused;
  for (int k = 0; k < 16; k++) {
    a->ref[k] += r->shRef[k];
  }
  for (int k = 0; k < r->shErrN && a->errN < 40000; k++) {
    a->errAbs[a->errN++] = fabsf(r->shErr[k]);
    a->errMax = fmaxf(a->errMax, fabsf(r->shErr[k]));
  }
  a->errEndSum += fabsf(r->shErrEnd);
  a->hMax = fmaxf(a->hMax, r->shHMax);
  a->devCorr = fmaxf(a->devCorr, r->shThrDevCorr);
  a->devHover = fmaxf(a->devHover, r->shThrDevHover);
}

static void testSetHeightLoop(void)
{
  static const float cs[6] = {-0.03f, -0.015f, -0.005f, 0.005f, 0.015f, 0.03f};
  static const uint16_t hs[] = {100, 200};   /* 2.00 m: the cap since 2026-10-15 (the 1.20 m take-off moved up),
                                                 photos every 7 s (the runbook above 1.2 m) */
  const int seeds = 10;
  for (size_t ih = 0; ih < sizeof(hs) / sizeof(hs[0]); ih++) {
    const uint16_t cm = hs[ih];
    const float h0 = cm * 0.01f, P = cm > 120 ? 7.0f : 12.0f;
    int bad = 0, badLand = 0, floorRuns = 0, badProg = 0, badErr = 0, badEnd = 0, behind = 0;
    for (int a = 0; a < 6; a++) {
      /* per photo-to-command delay L: 8 s (even seeds) and 15 s (odd) */
      for (int il = 0; il < 2; il++) {
        const float L = il ? 15.0f : 8.0f;
        shAgg_t *g = &shAgg;
        memset(g, 0, sizeof(*g));
        float baseEnd = 0.0f, baseMax = 0.0f;
        int nb = 0, other = 0;
        for (int i = il; i < seeds; i += 2) {
          const uint32_t seed = 21000u + 1000u * (uint32_t)ih + 100u * (uint32_t)a + (uint32_t)i;
          result_t r;
          shRun(cm, cs[a], 1, 0.0f, seed, i, &r);
          shAdd(g, &r);
          other += r.landWhy != AUTO_RS_LAND_CMD;
          result_t b;
          shRun(cm, cs[a], 2, 0.0f, seed, i, &b);
          nb++;
          /* the baseline's error at 150 s; on the floor before it (a sinking drift at this height), what the drift
           * reaches by then: |c| over the 141 s from the wait's start */
          baseEnd += b.landWhy == AUTO_RS_LAND_CMD ? fabsf(b.shErrEnd) : fmaxf(fabsf(b.shErrEnd), fabsf(cs[a]) * 141.0f);
          for (int k = 0; k < b.shErrN; k++) {
            baseMax = fmaxf(baseMax, fabsf(b.shErr[k]));
          }
          bad += b.killed || b.cutInAir;
        }
        qsort(g->errAbs, (size_t)g->errN, sizeof(float), cmpf2);
        const float p50 = g->errN ? g->errAbs[g->errN / 2] : 0.0f;
        const float p95 = g->errN ? g->errAbs[g->errN * 95 / 100] : 0.0f;
        const float lim = fabsf(cs[a]) * (P + 2.0f * L) + 0.25f;
        /* Claude keeps up when one 0.5 m command per cycle (L, the correction ~4 s, its confirming photo 1 s later)
         * outruns the drift; the floor is out of reach when the bound stays above it */
        const bool keepsUp = fabsf(cs[a]) * (L + 5.0f) <= AUTO_SH_MAX_M;
        const bool offFloor = cs[a] < 0.0f || h0 - lim > 0.10f;
        printf("info: SET_HEIGHT loop %.2f m, c %+.3f m/s, L %2.0f s (%d seeds, P %.0f s): true-height error p50 %.3f "
               "p95 %.3f max %.3f m (bound %.2f%s%s); at 150 s mean %.3f m against %.3f without corrections (max "
               "%.2f); highest %.2f m; %d corrections, refused BUSY %d SH_RATE %d other %d; PROGRESS %d; LAND other "
               "than the Mac's %d; thrust deviation %.1f %% in the corrections, %.1f %% in the hover\n", h0,
               (double)cs[a], L, g->n, P, p50, p95, g->errMax, lim, keepsUp ? "" : ", 0.5 m a cycle cannot keep up",
               offFloor ? "" : ", the floor within it", g->errEndSum / (float)g->n, baseEnd / (float)nb, baseMax,
               g->hMax, g->acc, g->ref[AUTO_MVW_BUSY], g->ref[AUTO_MVW_SH_RATE],
               g->retries - g->ref[AUTO_MVW_BUSY] - g->ref[AUTO_MVW_SH_RATE], g->progress, other,
               100.0f * g->devCorr, 100.0f * g->devHover);
        bad += g->killed + g->cutAir + g->shiftRefused;
        if (offFloor && keepsUp) {
          badLand += other;
          badErr += g->errMax > lim;
        } else {
          floorRuns += other;
          behind += !keepsUp;
        }
        badProg += fabsf(cs[a]) <= 0.02f ? g->progress : 0;
        badEnd += fabsf(cs[a]) >= 0.015f && keepsUp && !(g->errEndSum <= baseEnd / 3.0f);
      }
    }
    printf("info: SET_HEIGHT loop at %.2f m: %d runs reached the floor or another LAND where the bound reaches it or "
           "0.5 m a cycle cannot keep up (|c| (L + 5 s) > 0.5 m: %d cells)\n", h0, floorRuns, behind);
    CHECK(bad == 0, "SET_HEIGHT loop at %.2f m: never KILLED or cut in the air, no shift refused (%d)", h0, bad);
    CHECK(badLand == 0, "SET_HEIGHT loop at %.2f m: no LAND but the Mac's where Claude keeps up and the bound stays off "
          "the floor (%d)", h0, badLand);
    CHECK(badProg == 0, "SET_HEIGHT loop at %.2f m: no correction stopped PROGRESS for |c| <= 0.02 (%d)", h0, badProg);
    CHECK(badErr == 0, "SET_HEIGHT loop at %.2f m: the true height within |c| (P + 2 L) + 0.25 m of the intended where "
          "Claude keeps up (%d cells over)", h0, badErr);
    CHECK(badEnd == 0, "SET_HEIGHT loop at %.2f m: at 150 s at most 1/3 of the baseline's error for |c| >= 0.015 (%d "
          "cells over)", h0, badEnd);
    /* the camera's vertical scale 25 % off (report only) */
    for (int sgn = -1; sgn <= 1; sgn += 2) {
      shAgg_t *g = &shAgg;
      memset(g, 0, sizeof(*g));
      for (int i = 0; i < 4; i++) {
        result_t r;
        shRun(cm, 0.015f * (float)(i % 2 ? 1 : -1), 1, 0.25f * (float)sgn, 23000u + 1000u * (uint32_t)ih + (uint32_t)i,
              i, &r);
        shAdd(g, &r);
      }
      qsort(g->errAbs, (size_t)g->errN, sizeof(float), cmpf2);
      printf("info: SET_HEIGHT loop %.2f m, the camera's vertical scale %+.0f %% (c +-0.015, 4 seeds): error p50 %.3f "
             "max %.3f m, %d corrections, PROGRESS %d, killed %d\n", h0, 25.0f * (float)sgn,
             g->errN ? g->errAbs[g->errN / 2] : 0.0f, g->errMax, g->acc, g->progress, g->killed);
    }
    /* single corrections of +-0.5 m */
    for (int sgn = -1; sgn <= 1; sgn += 2) {
      for (int sc = 0; sc < 2; sc++) {
        scen_t s = baseScen(24000u + 10u * (uint32_t)ih + (uint32_t)(sgn + 1) + (uint32_t)sc, 1.0f);
        toHeight(&s, cm);
        s.waitMs = 120000;
        s.shSingle = 0.5f * (float)sgn;
        s.shAtS = 5.0f;
        s.shScale = sc ? 0.25f : 0.0f;
        result_t r;
        runSim(&s, &r);
        printf("info: SET_HEIGHT %+.2f m at %.2f m%s: accepted %d, end %s after %.2f s, %+.3f m from its target, thrust "
               "deviation %.1f %% (the hover %.1f %%)\n", s.shSingle, h0, sc ? " (the camera's scale +25 %)" : "",
               r.shAcc, autoMvEndName(r.shSingleEnd), r.shSingleS, r.shSingleErr, 100.0f * r.shThrDevCorr,
               100.0f * r.shThrDevHover);
        if (!sc) {
          CHECK(r.shAcc == 1 && r.shSingleEnd == AUTO_MVE_DONE && r.shSingleS <= 4.1f + 3.0f + 0.1f &&
                fabsf(r.shSingleErr) <= 0.10f && r.shThrDevCorr <= 0.07f && !r.killed && !r.cutInAir,
                "a single %+.2f m correction at %.2f m: DONE within 7.1 s (%.2f), within 0.10 m of its target (%+.3f), "
                "thrust within 7 %% of the hover (%.1f %%)", s.shSingle, h0, r.shSingleS, r.shSingleErr,
                100.0f * r.shThrDevCorr);
        }
      }
    }
  }
}

/* The 2026-10-15 / 10-16 corrections replayed in the closed loop (docs/autonomy-a2b.md 27.2; Claude's judgements in
 * RESUME.md, the estimates and times from events.jsonl and the photos): take-off to 1.00 m, the logged photo error
 * (judged - estimate at capture) put into the estimate, a perfect judgement, then SET_HEIGHT 0.5 s after the photo
 * (the Mac's own delay: the drift during Claude's delay is the late sink's, not the command's), the camera's scale
 * and speed unbiased. The "go to the target" form must end DONE within a vertical MOVE's settle tolerance of the
 * intended 1.00 m (0.04 m + 10 % of the flight, + 0.01 m of the sim's noise) and within SH_REPLAY_TOL_M 2 s later
 * (the 0.03 m of the arithmetic, auto_fsm's replay, is exact there: test_auto_fsm.c testSetHeightReplay; the closed
 * loop adds the settle). Printed too: the logged photo-to-command time (10-29 s: the sim's own drift in it), and the
 * 4-byte form as it was sent (judged X, or the est-rule's h_at + X - 1.00). */
#define SH_REPLAY_TOL_M 0.07f   /* 2 s after DONE: the settle tolerance of a 0.3 m flight + the sim noise */
static void testSetHeightReplaySim(void)
{
  static const struct { const char *what; float x, at, lat, sentOff; } rp[] = {
    { "10-15 p2 18:18 (1.05 at 0.91)", 1.05f, 0.91f, 17.0f, 0.00f },
    { "10-15 p1 19:44 (1.08 at 0.90)", 1.08f, 0.90f, 12.0f, 0.00f },
    { "10-15 fastcheck 20:05 (1.28 at 0.93)", 1.28f, 0.93f, 12.0f, 0.00f },
    { "10-15 fastcheck 20:06 (0.82 at 0.92)", 0.82f, 0.92f, 18.0f, 0.00f },
    { "10-16 F1 09:45 (1.12 at 0.90)", 1.12f, 0.90f, 10.0f, 0.00f },
    { "10-16 F2 09:52:16 (1.15 at 0.89; sent 1.04)", 1.15f, 0.89f, 10.0f, -0.11f },
    { "10-16 F2 09:52:43 (0.60 at 0.89; sent 0.49)", 0.60f, 0.89f, 10.0f, -0.11f },
    { "10-16 tiger 11:09 (0.70 at 0.85; sent 0.55)", 0.70f, 0.85f, 29.0f, -0.15f },
  };
  float worst = 0.0f, worstOld = 0.0f;
  for (size_t k = 0; k < sizeof(rp) / sizeof(rp[0]); k++) {
    result_t rr[3];   /* the target form after 0.5 s, after the logged delay; the 4-byte form as sent after 0.5 s */
    for (int v = 0; v < 3; v++) {
      scen_t s = baseScen(31000u + 10u * (uint32_t)k, 1.0f);
      toHeight(&s, 100);
      s.waitMs = 120000;
      s.shRep = v < 2 ? 1 : 2;
      s.shAtS = 5.0f;
      s.shRepX = rp[k].x;
      s.shRepAt = rp[k].at;
      s.shRepLat = v == 1 ? rp[k].lat : 0.5f;
      s.shRepSentOff = rp[k].sentOff;
      runSim(&s, &rr[v]);
    }
    const result_t *r = &rr[0];
    worst = fmaxf(worst, fabsf(r->shRepH2 - 1.00f));
    worstOld = fmaxf(worstOld, fabsf(rr[2].shRepH2 - 1.00f));
    printf("info: SET_HEIGHT replay %-44s the target form %s after %.1f s %+.3f, 2 s later %+.3f m from 1.00 (with the "
           "logged %.0f s: %+.3f, the sim's own drift in it %+.3f); the 4-byte form as sent %+.3f\n", rp[k].what,
           autoMvEndName(r->shSingleEnd), r->shSingleS, r->shSingleErr, r->shRepH2 - 1.00f, rp[k].lat,
           rr[1].shRepH2 - 1.00f, rr[1].shRepHCmd - rr[1].shRepH0, rr[2].shRepH2 - 1.00f);
    const float fly = 1.00f - r->shRepHCmd;
    const float tolDone = AUTO_MV_SETTLE_TOL_M + AUTO_MV_SETTLE_TOL * fabsf(fly) + 0.01f;
    CHECK(r->shAcc == 1 && r->shRepResult == AUTO_R_OK && r->shSingleEnd == AUTO_MVE_DONE &&
          fabsf(r->shSingleErr) <= tolDone && fabsf(r->shRepH2 - 1.00f) <= SH_REPLAY_TOL_M && !r->killed &&
          !r->cutInAir && r->shShiftRefused == 0 && rr[2].shShiftRefused == 0,
          "replay %s: the target form DONE %+.3f m from 1.00 (the settle's %.3f), %.3f m 2 s later (within %.2f)",
          rp[k].what, r->shSingleErr, tolDone, r->shRepH2, SH_REPLAY_TOL_M);
  }
  printf("info: SET_HEIGHT replays: the target form's worst %.3f m from 1.00, the 4-byte form's as sent %.3f m\n", worst,
         worstOld);
}

/* ---- fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31) --------------------------------------------------- */

/* The closed loop at mvspeed 150 / 200 / 300 (scen_t mvSpeed: the motions' limits, the hold's move lean limit and slew,
 * controller_pid's apply slew and its brake lean with the hold stale; drag 0.35 /s, the trim error +-0.3 deg, the
 * observer's D +-10 %):
 *  - testFarMoves' five far-scene moves (D ~3.5 m) over 6 seeds per step: every one accepted and ending DONE or SHORT
 *    (no PROGRESS, FRONT, VIEW_FAST, NO_VIEW on a clean view), within +-20 % of itself, run in its planned profile
 *    time (no pause; within 0.2 s), the true expansion's lead over the reference's <= 1.35 (the safety case's lead),
 *    the true image rates inside the camera's gates (0.3 /s, 1.5 px/frame = 0.217 /s), the lean < 6 deg (TILT 8);
 *  - the mission (MOVE right 50, back 50, YAW 90, up 20, forward 40) over 8 seeds per step: every command accepted,
 *    >= 95 % of the moves within +-20 %, every one within +-25 %, the yaw within 5 %, a touchdown;
 *  - the Mac's longest approach at an expansion scale k 0.6 from D 2.5 / 3.5 / 4.5 m: the closest true approach to
 *    the wall >= 0.25 m;
 *  - a STOP at speed (2 s into a 2 m approach from D 4.5 m): the true travel after it within the stopping distance
 *    v t_d + v^2 / (2 BRAKE) + 0.05 m (t_d 0.35 s, v the true speed then);
 *  - the camera's frames stopping at speed (2 s in, for 3 s): VIEW_FAST, the true travel after the stall within
 *    v (0.30 + 0.35 + 0.15) s + v^2 / (2 BRAKE) + 0.05 m (the frame age passes 300 ms 0.18 s after the last frame, then
 *    the AUTO_MV_FAST_VIEW_DEB_MS pause; 0.15 s for the speed-up lean to reverse since the 2 s speed-up of 2026-10-17),
 *    >= 0.25 m from the wall; the same at 100: the pause of before (info);
 *  - a frozen view on a 2.5 m approach from D 4.5 m: PROGRESS, <= 1.3 m out;
 *  - YAW 90 / 180 / -45 at 200 (35 deg/s, 70 deg/s^2), 4 seeds each: the profile's time, never paused, the hold
 *    engaged on every turn frame, DONE (the re-track bob at 0.20 m/s^2, as at 100), the heading within 5 %, no
 *    CAM_LOST, a touchdown (the sim's heading has no overshoot: the logged 30 deg/s turns ran 1.2-1.26x over a frame,
 *    which the 35 deg/s cap leaves under the 45 deg/s gate; the flights check it).
 * Added after the review of cbccec5 (the sim models the hold's HOVER scene gate, scen_t noSceneGate):
 *  - A and C: the scene gate never refuses a frame; its expansion EMA <= 0.25 /s (far scenes) and <= 0.27 /s (the
 *    Mac's longest approach at k 0.6; it was 0.29 at W_FWD 0.20);
 *  - H: the frames stopping on the profile's slowing part (the reference at 0.25 m/s, D 5.5 m, 1.3 m forward):
 *    VIEW_FAST from the reference speed AUTO_MV_FAST_T_TRACK_S before the pause, the travel after it within
 *    v 0.80 s + v^2 / (2 BRAKE) + 0.05 m; at 100 the pause of before (info);
 *  - I: 1-2 refused frames in the cruise only pause a fast move (DONE or SHORT), 4-5 stop it (VIEW_FAST). */
static float fastStopBound(float v, float tD, uint16_t pct)
{
  autoMvSpeed_t c;
  autoMvSpeedCfg(pct, &c);
  return v * tD + v * v / (2.0f * c.brake) + 0.05f;
}

static void testFastMoves(void)
{
  static const uint16_t pcts[3] = {150, 200, 300};
  static const uint16_t pctsA[4] = {100, 150, 200, 300};
  static const int16_t mv[5][2] = {{100, 0}, {-150, 0}, {0, 100}, {0, 150}, {70, 70}};
  /* A: the far-scene moves, each seed at 100 first: a fast step is checked against the same seed at 100 (the sim's D
   * errors, its wander and its D-dependent refusals make some moves 20-26 % off, or refused, at every speed) */
  static float err100[5][6];
  static int good100[5][6], acc100[5][6];
  for (int qa = 0; qa < 4; qa++) {
    const uint16_t pq = pctsA[qa];
    int n = 0, ok = 0, okErr = 0, okTime = 0, okLead = 0, okRate = 0, okLean = 0, ends[16] = {0};
    float errMax = 0.0f, leadMax = 0.0f, wfMax = 0.0f, wlMax = 0.0f, leanMax = 0.0f, dtMax = 0.0f, vMax = 0.0f, stillMax = 0.0f;
    float emaMax = 0.0f;
    int rejN = 0;
    float durSum[5] = {0}, profSum[5] = {0};
    for (int k = 0; k < 5; k++) {
      for (uint32_t sd = 0; sd < 6; sd++) {
        scen_t sc = baseScen(6100u + 10u * (uint32_t)k + sd, -0.5f);
        sc.oneMove = true;
        sc.oneR = mv[k][0];
        sc.oneF = mv[k][1];
        sc.mvSpeed = pq;
        result_t r;
        runSim(&sc, &r);
        n++;
        const float want = hypotf(mv[k][0] * 0.01f, mv[k][1] * 0.01f);
        const float e = hypotf(r.mvR - mv[k][0] * 0.01f, r.mvF - mv[k][1] * 0.01f) / want;
        const bool acc = r.oneWhy == 0;
        const bool good = acc && (r.oneEnd == AUTO_MVE_DONE || r.oneEnd == AUTO_MVE_SHORT) && !r.killed && r.touchdown;
        ends[r.oneEnd & 15]++;
        if (pq == 100) {
          err100[k][sd] = e;
          good100[k][sd] = good;
          acc100[k][sd] = acc;
        }
        /* against 100 on this seed: accepted when 100 was, DONE or SHORT when 100 was, the error within 20 % or at most
         * 100's + 5 points */
        const bool okRun = (acc || !acc100[k][sd]) && (good || !good100[k][sd]) && !r.killed && r.touchdown;
        const bool okE = !acc || e <= fmaxf(0.20f, err100[k][sd] + 0.05f);
        ok += okRun;
        okErr += okE;
        /* (the profile's own time: less the wait for a still drone at its start, mvfix STILL, at most 1.5 s) */
        okTime += !acc || fabsf(r.oneRunS - r.stillS - r.profT) <= 0.2f || !good;
        stillMax = fmaxf(stillMax, r.stillS);
        okLead += mv[k][1] <= 0 || !acc || r.leadMax <= 1.35f;
        okRate += r.wFwdTrueMax <= 0.30f && r.wLatTrueMax <= 0.217f;
        okLean += r.maxLean < 6.0f;
        if (acc) {
          errMax = fmaxf(errMax, e);
          dtMax = fmaxf(dtMax, good ? fabsf(r.oneRunS - r.stillS - r.profT) : 0.0f);
        }
        if (mv[k][1] > 0) {
          leadMax = fmaxf(leadMax, r.leadMax);
        }
        wfMax = fmaxf(wfMax, r.wFwdTrueMax);
        wlMax = fmaxf(wlMax, r.wLatTrueMax);
        leanMax = fmaxf(leanMax, r.maxLean);
        vMax = fmaxf(vMax, r.vTrueMax);
        emaMax = fmaxf(emaMax, r.expEmaMax);
        rejN += r.sceneRej;
        durSum[k] += r.oneEndS;
        profSum[k] += r.profT;
        if (!okRun || !okE || (!good && pq != 100)) {
          printf("info: fast %u far MOVE %+d %+d seed %u: %s, end %s, error %.0f %% (100: %.0f %%, %s), run %.2f s "
                 "(profile %.2f)\n", pq, mv[k][0], mv[k][1], (unsigned)sc.seed,
                 acc ? "accepted" : autoMvWhyName((autoMvWhy_t)r.oneWhy), autoMvEndName(r.oneEnd), 100.0f * e,
                 100.0f * err100[k][sd], good100[k][sd] ? "DONE/SHORT" : "not", r.oneRunS, r.profT);
        }
      }
    }
    printf("info: mvspeed %u, far-scene moves (D ~3.5 m, 30): DONE %d SHORT %d PROGRESS %d FRONT %d VIEW_FAST %d "
           "NO_VIEW %d; error max %.0f %% (accepted); profile - run <= %.2f s; lead max %.2f; true image rates max "
           "%.3f /s forward, %.3f /s sideways; true speed max %.2f m/s; lean max %.1f deg; the wait for a still drone "
           "max %.2f s\n", pq, ends[AUTO_MVE_DONE],
           ends[AUTO_MVE_SHORT], ends[AUTO_MVE_PROGRESS], ends[AUTO_MVE_FRONT], ends[AUTO_MVE_VIEW_FAST],
           ends[AUTO_MVE_NO_VIEW], 100.0f * errMax, dtMax, leadMax, wfMax, wlMax, vMax, leanMax, stillMax);
    printf("info: mvspeed %u, far-scene moves: the scene gate's expansion EMA max %.3f /s (gate 0.30), frames refused "
           "in a MOVE %d\n", pq, emaMax, rejN);
    printf("info: mvspeed %u, the five moves' profile / to their end (mean): %.2f/%.2f, %.2f/%.2f, %.2f/%.2f, "
           "%.2f/%.2f, %.2f/%.2f s\n", pq, profSum[0] / 6, durSum[0] / 6, profSum[1] / 6, durSum[1] / 6,
           profSum[2] / 6, durSum[2] / 6, profSum[3] / 6, durSum[3] / 6, profSum[4] / 6, durSum[4] / 6);
    if (pq == 100) {
      continue;
    }
    CHECK(ok == n && okErr == n && ends[AUTO_MVE_VIEW_FAST] == 0 && ends[AUTO_MVE_FRONT] == 0,
          "mvspeed %u: the far-scene moves against the same seed at 100: accepted and DONE or SHORT when 100 was "
          "(%d/%d), within 20 %% or 100's error + 5 points (%d/%d; worst %.0f %%), no VIEW_FAST or FRONT on a clean view",
          pq, ok, n, okErr, n, 100.0f * errMax);
    CHECK(okTime == n && stillMax <= 1.5f + 1e-3f, "mvspeed %u: every move ran in its planned profile time after its "
          "wait for a still drone (no pause; worst %.2f s; the wait at most %.2f s, cap 1.5)", pq, dtMax, stillMax);
    CHECK(emaMax <= 0.25f && rejN == 0, "mvspeed %u: the hold's scene gate never refused a frame of the far-scene moves "
          "(%d), its expansion EMA <= 0.25 /s against the 0.30 gate (%.3f)", pq, rejN, emaMax);
    CHECK(okLead == n && okRate == n && okLean == n, "mvspeed %u: the expansion's lead <= 1.35 (%.2f), the true image "
          "rates inside the gates (%.3f / 0.30, %.3f / 0.217 /s), the lean < 6 deg (%.1f)", pq, leadMax, wfMax,
          wlMax, leanMax);
  }

  /* B: the mission; 100 first on the same seeds */
  float worst100 = 0.0f;
  int in20of100 = 0;
  for (int qa = 0; qa < 4; qa++) {
    const int q = qa - 1;   /* (-1: 100) */
    const uint16_t pq = qa == 0 ? 100 : pcts[q];
    int n = 0, okAll = 0, refused = 0, nErr = 0, in20 = 0, landed = 0;
    float worst = 0.0f;
    for (int i = 0; i < 8; i++) {
      scen_t sc = baseScen(6300u + (uint32_t)i, 1.0f);
      sc.mission = true;
      sc.mvSpeed = pq;
      result_t r;
      runSim(&sc, &r);
      n++;
      bool all = r.ncmd == 5;
      for (int c = 0; c < r.ncmd; c++) {
        for (int a = 0; a < 3; a++) {
          if (r.want[c][a] != 0.0f) {
            const float err = fabsf(r.got[c][a] - r.want[c][a]) / fabsf(r.want[c][a]);
            worst = fmaxf(worst, err);
            all = all && err <= 0.25f;
            nErr++;
            in20 += err <= 0.20f;
          }
        }
      }
      all = all && fabsf(r.yawGot - r.yawWant) <= 0.05f * fabsf(r.yawWant);
      okAll += all;
      refused += r.refused;
      landed += !r.killed && r.landWhy == AUTO_RS_LAND_CMD && r.endWhy == AUTO_RS_TOUCHDOWN;
    }
    printf("info: mvspeed %u, the mission over 8 seeds: worst relative error %.0f %%, within 20 %% %d of %d\n", pq,
           100.0f * worst, in20, nErr);
    if (qa == 0) {
      worst100 = worst;
      in20of100 = in20;
      continue;
    }
    CHECK(refused == 0 && in20 * 100 >= nErr * 95 && in20 >= in20of100 && worst <= worst100 + 0.05f && landed == n,
          "mvspeed %u: the mission: every command accepted (%d refused), >= 95 %% within 20 %% and as many as at 100 "
          "(%d of %d, 100: %d), the worst at most 100's + 5 points (%.0f %%, 100: %.0f %%; every move within 25 %% and "
          "the yaw within 5 %%: %d/%d), LAND by the Mac and a touchdown (%d)", pq, refused, in20, nErr, in20of100,
          100.0f * worst, 100.0f * worst100, okAll, n, landed);
  }

  /* C: the Mac's longest approach at k 0.6 */
  {
    static const float y0s[3] = {0.5f, -0.5f, -1.5f};   /* D 2.5 / 3.5 / 4.5 m */
    for (int q = 0; q < 3; q++) {
      float worst = 9.0f, emaC = 0.0f;
      int okW = 0, rejC = 0;
      for (int j = 0; j < 3; j++) {
        for (uint32_t sd = 0; sd < 3; sd++) {
          scen_t sc = baseScen(6400u + 10u * (uint32_t)j + sd, y0s[j]);
          sc.oneMove = true;
          sc.oneF = 400;
          sc.macCap = true;
          sc.kFix = 0.6f;
          sc.mvSpeed = pcts[q];
          result_t r;
          runSim(&sc, &r);
          worst = fminf(worst, r.minWall);
          emaC = fmaxf(emaC, r.expEmaMax);
          rejC += r.sceneRej;
          okW += r.oneWhy == 0 && r.minWall >= 0.25f && !r.killed && r.touchdown;
          if (sd == 0 || r.sceneRej > 0) {
            printf("info: mvspeed %u, k 0.6, the Mac's longest forward %.2f m from D %.2f: end %s after %.1f s, the "
                   "closest true approach %.2f m, lead max %.2f, true speed max %.2f m/s, expansion EMA max %.3f /s, "
                   "frames refused %d\n", pcts[q], r.oneF * 0.01f, r.d0Obs, autoMvEndName(r.oneEnd), r.oneEndS,
                   r.minWall, r.leadMax, r.vTrueMax, r.expEmaMax, r.sceneRej);
          }
        }
      }
      CHECK(okW == 9, "mvspeed %u: the Mac's longest approach at k 0.6 (D 2.5-4.5 m): >= 0.25 m from the wall (%d/9, "
            "closest %.2f m)", pcts[q], okW, worst);
      CHECK(emaC <= 0.27f && rejC == 0, "mvspeed %u: the Mac's longest approach at k 0.6: the hold's scene gate never "
            "refused a frame (%d), its expansion EMA <= 0.27 /s against the 0.30 gate (%.3f; 0.29 at W_FWD 0.20)",
            pcts[q], rejC, emaC);
    }
  }

  /* D, E: a STOP at speed, the camera's frames stopping at speed */
  for (int q = 0; q < 3; q++) {
    for (int kind = 0; kind < 2; kind++) {
      int okS = 0;
      float worstMargin = -9.0f;
      for (uint32_t sd = 0; sd < 4; sd++) {
        scen_t sc = baseScen(6500u + 10u * (uint32_t)kind + sd, -1.5f);
        sc.oneMove = true;
        sc.oneF = 200;
        sc.mvSpeed = pcts[q];
        if (kind == 0) {
          sc.stopAtS = 2.0f;
        } else {
          sc.camStallAtS = 2.0f;
          sc.camStallS = 3.0f;
        }
        result_t r;
        runSim(&sc, &r);
        /* (the stall: 0.65 s + 0.15 s for the speed-up lean to reverse at the move slew: since 2026-10-17 the speed-up
         * at 250-300 lasts 2 s, as at 200, and the stall at 2 s comes at its end) */
        const float bound = fastStopBound(r.evtVTrue, kind == 0 ? 0.35f : 0.80f, pcts[q]);
        worstMargin = fmaxf(worstMargin, r.evtTravel - bound);
        const bool endOk = kind == 0 ? r.oneEnd == AUTO_MVE_STOP_CMD
                                     : (r.evtVRef > AUTO_MV_FAST_PAUSE_V ? r.oneEnd == AUTO_MVE_VIEW_FAST
                                                                         : r.oneEnd == AUTO_MVE_NO_VIEW);
        okS += r.evtSeen && endOk && r.evtTravel <= bound && r.minWall >= 0.25f && !r.killed && r.touchdown;
        if (sd == 0 || !endOk || r.evtTravel > bound) {
          printf("info: mvspeed %u, %s 2 s into a 2 m approach from D 4.5 m: reference %.2f m/s, true %.2f m/s; end %s, "
                 "true travel after it %.2f m (bound %.2f), closest %.2f m\n", pcts[q], kind ? "the frames stop" : "STOP",
                 r.evtVRef, r.evtVTrue, autoMvEndName(r.oneEnd), r.evtTravel, bound, r.minWall);
        }
      }
      CHECK(okS == 4, "mvspeed %u, %s at speed: %s, the true travel after it within the stopping distance (worst %+.2f m "
            "over it), >= 0.25 m from the wall (%d/4)", pcts[q], kind ? "the camera's frames stopping" : "a STOP",
            kind ? "VIEW_FAST (NO_VIEW below 0.20 m/s)" : "STOPPED STOP_CMD", worstMargin, okS);
    }
  }
  {
    scen_t sc = baseScen(6510u, -1.5f);
    sc.oneMove = true;
    sc.oneF = 200;
    sc.camStallAtS = 2.0f;
    sc.camStallS = 3.0f;
    result_t r;
    runSim(&sc, &r);
    printf("info: mvspeed 100, the frames stop 2 s into a 2 m approach from D 4.5 m (reference %.2f m/s, true %.2f m/s): "
           "end %s, true travel after it %.2f m (the pause of before: the reference frozen, coasting on drag)\n",
           r.evtVRef, r.evtVTrue, autoMvEndName(r.oneEnd), r.evtTravel);
  }

  /* F: a frozen view at speed */
  for (int q = 0; q < 3; q++) {
    int okF = 0;
    float worstAt = 0.0f;
    for (uint32_t sd = 0; sd < 3; sd++) {
      scen_t sc = baseScen(6600u + sd, -1.5f);
      sc.oneMove = true;
      sc.oneF = 250;
      sc.frozen = true;
      sc.mvSpeed = pcts[q];
      result_t r;
      runSim(&sc, &r);
      const float at = hypotf(r.endTrueR, r.endTrueF);
      worstAt = fmaxf(worstAt, at);
      okF += r.oneEnd == AUTO_MVE_PROGRESS && at <= 1.3f && !r.killed && r.touchdown;
    }
    CHECK(okF == 3, "mvspeed %u: a frozen view on a 2.5 m approach from D 4.5 m: PROGRESS, <= 1.3 m out (%d/3, worst "
          "%.2f m)", pcts[q], okF, worstAt);
  }

  /* H: the view lost on the profile's slowing part (review of cbccec5): far scene (D ~5.5 m), 1.3 m forward, the frames
   * stopping (3 s) where the reference has slowed to 0.25 m/s, against the same seeds at 100 stopping at 0.15 m/s */
  for (int qa = 0; qa < 4; qa++) {
    const uint16_t pq = pctsA[qa];
    autoMvSpeed_t c;
    autoMvSpeedCfg(pq, &c);
    const float vAt = pq > 100 ? 0.25f : 0.15f;
    int okH = 0, fastN = 0, accN = 0;
    float overMax = -9.0f, travelMax = 0.0f, boundMin = 9.0f;
    for (uint32_t sd = 0; sd < 4; sd++) {
      scen_t sc = baseScen(6800u + sd, 0.5f);
      sc.room = 6.0f;
      sc.oneMove = true;
      sc.oneF = 130;
      sc.mvSpeed = pq;
      result_t r0;
      runSim(&sc, &r0);   /* the plan (no stall): its profile time */
      if (r0.oneWhy != 0) {
        continue;   /* (a seed whose observer D refuses the move at every step) */
      }
      accN++;
      sc.camStallAtS = r0.profT - vAt / c.aH;
      sc.camStallS = 3.0f;
      result_t r;
      runSim(&sc, &r);
      const float bound = fastStopBound(r.evtVTrue, 0.80f, pq > 100 ? pq : 150);
      const float over = r.mvF - sc.oneF * 0.01f;
      overMax = fmaxf(overMax, over);
      travelMax = fmaxf(travelMax, r.evtTravel);
      boundMin = fminf(boundMin, bound);
      fastN += r.oneEnd == AUTO_MVE_VIEW_FAST;
      const bool good = r.oneWhy == 0 && r.evtSeen &&
                        (pq == 100 || (r.oneEnd == AUTO_MVE_VIEW_FAST && r.evtTravel <= bound)) && r.minWall >= 0.25f &&
                        !r.killed && r.touchdown;
      okH += good;
      if (sd == 0 || !good) {
        printf("info: mvspeed %u, the frames stop on the slowing part of 1.3 m forward from D 5.5 m (reference %.2f m/s, "
               "true %.2f m/s): end %s, true travel after it %.2f m (bound %.2f), %+.2f m past the plan\n", pq,
               r.evtVRef, r.evtVTrue, autoMvEndName(r.oneEnd), r.evtTravel, bound, over);
      }
    }
    printf("info: mvspeed %u, the frames stop on the slowing part (%d accepted of 4 seeds): VIEW_FAST %d, true travel "
           "after it <= %.2f m, past the plan <= %+.2f m\n", pq, accN, fastN, travelMax, overMax);
    if (pq > 100) {
      CHECK(accN >= 3 && okH == accN, "mvspeed %u: the frames stopping on the profile's slowing part (reference "
            "0.25 m/s): VIEW_FAST from the lagged reference speed, the travel after it within the stopping distance "
            "(%d/%d accepted; <= %.2f m, past the plan <= %+.2f m)", pq, okH, accN, travelMax, overMax);
    }
  }

  /* I: one or two refused frames in the cruise (a person crossing the view) pause a fast MOVE, which runs on (no
   * VIEW_FAST); three or more stop it */
  for (int q = 0; q < 3; q++) {
    int okI = 0, okI3 = 0;
    for (uint32_t sd = 0; sd < 4; sd++) {
      for (int n = 0; n < 2; n++) {
        scen_t sc = baseScen(6900u + sd, -1.5f);
        sc.oneMove = true;
        sc.oneF = 200;
        sc.mvSpeed = pcts[q];
        sc.camRefuseAtS = 2.0f;
        sc.camRefuseS = n == 0 ? 0.13f : 0.30f;   /* 1-2 frames, 4-5 frames */
        result_t r;
        runSim(&sc, &r);
        if (n == 0) {
          okI += r.oneWhy == 0 && (r.oneEnd == AUTO_MVE_DONE || r.oneEnd == AUTO_MVE_SHORT) && !r.killed && r.touchdown;
        } else {
          okI3 += r.oneWhy == 0 && r.oneEnd == AUTO_MVE_VIEW_FAST && !r.killed && r.touchdown;
        }
        if (sd == 0) {
          printf("info: mvspeed %u, %s refused 2 s into a 2 m approach from D 4.5 m: paused %d, end %s\n", pcts[q],
                 n == 0 ? "1-2 frames" : "4-5 frames", r.pausedSeen, autoMvEndName(r.oneEnd));
        }
      }
    }
    CHECK(okI == 4 && okI3 == 4, "mvspeed %u: 1-2 refused frames in the cruise only pause the move (DONE or SHORT: "
          "%d/4); 4-5 stop it, VIEW_FAST (%d/4)", pcts[q], okI, okI3);
  }

  /* G: turns at 200 */
  {
    const int16_t degs[] = {90, 180, -45};
    autoMvSpeed_t c;
    autoMvSpeedCfg(200, &c);
    int ok = 0;
    float rateMax = 0.0f, errMax = 0.0f;
    for (int k = 0; k < 3; k++) {
      for (int i = 0; i < 4; i++) {
        scen_t sc = baseScen(6700u + (uint32_t)(10 * k + i), 1.0f);
        sc.turnDeg = degs[k];
        sc.mvSpeed = 200;
        result_t r;
        runSim(&sc, &r);
        const float tProf = fabsf((float)degs[k]) / c.yawRate + c.yawRate / c.yawAcc;
        const bool good = r.turnEnded && r.turnEnd == AUTO_MVE_DONE && !r.turnPaused && fabsf(r.turnS - tProf) < 0.05f &&
                          r.turnFramesEng == r.turnFrames && r.turnFrames > 10 &&
                          fabsf(r.turnYawErr) <= 0.05f * fabsf((float)degs[k]) &&
                          !(r.evMask & (1u << AUTO_EV_CAM_LOST)) && r.landWhy == AUTO_RS_LAND_CMD &&
                          r.endWhy == AUTO_RS_TOUCHDOWN && !r.killed;
        ok += good;
        rateMax = fmaxf(rateMax, r.turnRateMax);
        errMax = fmaxf(errMax, fabsf(r.turnYawErr));
        if (i == 0 || !good) {
          printf("info: mvspeed 200, YAW %d: turn %.2f s (profile %.2f), paused %d, hold frames engaged %d of %d, end %s, "
                 "heading error %+.1f deg, rate max %.1f deg/s, CAM_LOST %d, LAND %s\n", degs[k], r.turnS, tProf,
                 r.turnPaused, r.turnFramesEng, r.turnFrames, autoMvEndName(r.turnEnd), r.turnYawErr, r.turnRateMax,
                 (r.evMask & (1u << AUTO_EV_CAM_LOST)) != 0u, autoReasonName(r.landWhy));
        }
      }
    }
    CHECK(ok == 12, "mvspeed 200: YAW 90 / 180 / -45 at 35 deg/s: the profile's time, never paused, the hold engaged on "
          "every turn frame, DONE, the heading within 5 %%, no CAM_LOST, LAND by the Mac (%d/12; error <= %.1f deg, "
          "rate <= %.1f deg/s)", ok, errMax, rateMax);
  }
}

/* ---- the bolder ahead margins: braking near the scene (2026-10-17, docs/autonomy-a2b.md 32.2) -----------------------
 * The drone's largest forward MOVE (its floor at k 0.6) from D 2.0 / 3.0 / 4.5 m at k 0.6 / 0.73 / 1.0, mvspeed 100 /
 * 150 / 200 / 300, 4 seeds, the observer's D exact, with an overshoot forced by a forward push while the MOVE runs
 * (scen_t fwdPush from 1.5 s before its profile's end: 0, 0.10, 0.137 = the forensics' 0.8 deg of unlearned lean,
 * 0.20 m/s^2), at the margins of 2026-10-17 and, for the record, at the margins before (0.50 / 0.40 m, no near-scene
 * brake). Measured: the closest true approach, a contact (the drone's 0.05 m guard radius at the wall) and its speed,
 * how the MOVE ended, a FRONT stop's travel after it against v 0.55 s + v^2 / (2 BRAKE) + 0.05 m (a STOP's 0.35 s, plus
 * the frame's age and interval: the stop reads a frame ~0.12-0.19 s old). Checks at the defaults: no contact up to
 * 0.137 m/s^2 at every step (0.20: information; at kmax 2.5 none at 150-300, one at 0.08 m/s at 100, where the margins
 * before had 0 / 4-10 at up to 0.41 m/s), the undisturbed moves >= 0.20 m from the wall, every FRONT stop's travel within
 * the bound, never KILLED. */
static void testFrontBraking(void)
{
  static const float y0s[3] = {1.0f, 0.0f, -1.5f};   /* D ~2.0 / 3.0 / 4.5 m */
  static const float ks[3] = {0.6f, 0.73f, 1.0f};
  static const uint16_t pcts[4] = {100, 150, 200, 300};
  static const float pushes[4] = {0.0f, 0.10f, 0.137f, 0.20f};
  int bad = 0, killed = 0, travelBad = 0, frontAll = 0;
  float worstOver = -9.0f, minUndisturbed = 9.0f;
  for (int mg = 0; mg < 2; mg++) {
    for (int pi = 0; pi < 4; pi++) {
      for (int q = 0; q < 4; q++) {
        int n = 0, contacts = 0, front = 0, noView = 0;
        float vHit = 0.0f, minW = 9.0f;
        for (int ki = 0; ki < 3; ki++) {
          for (int di = 0; di < 3; di++) {
            for (uint32_t sd = 0; sd < 4; sd++) {
              scen_t sc = baseScen(81000u + 1000u * (uint32_t)q + 100u * (uint32_t)ki + 10u * (uint32_t)di + sd,
                                   y0s[di]);
              sc.oneMove = true;
              sc.fwdMax = 1;
              sc.kFix = ks[ki];
              sc.eDmax = 0.0f;
              sc.mvSpeed = pcts[q];
              sc.margins = mg;
              sc.fwdPush = pushes[pi];
              sc.pushAtS = -1.5f;
              sc.pushS = 30.0f;
              simMargins(mg);
              result_t r;
              runSim(&sc, &r);
              if (r.oneWhy != 0) {
                continue;
              }
              n++;
              minW = fminf(minW, r.minWall);
              contacts += r.wallTouch;
              if (r.wallTouch) {
                vHit = fmaxf(vHit, r.wallTouchV);
              }
              noView += r.oneEnd == AUTO_MVE_NO_VIEW;
              if (r.oneEnd == AUTO_MVE_FRONT) {
                front++;
                autoMvSpeed_t c;
                autoMvSpeedCfg(pcts[q], &c);
                const float v = r.endVTrue, after = r.endWall - r.minWall;
                const float over = after - (v * 0.55f + v * v / (2.0f * c.brake) + 0.05f);
                if (mg == 0) {
                  frontAll++;
                  travelBad += over > 0.0f;
                  worstOver = fmaxf(worstOver, over);
                }
              }
              if (mg == 0) {
                killed += r.killed;
                if (pi == 0) {
                  minUndisturbed = fminf(minUndisturbed, r.minWall);
                }
                bad += r.wallTouch && pushes[pi] <= 0.137f;
              }
            }
          }
        }
        printf("info: near the scene (%s margins), push %.3f m/s^2, mvspeed %u: %d moves, FRONT %d, NO_VIEW %d, "
               "contacts %d (at <= %.2f m/s), the closest true approach %.2f m\n", mg ? "the old" : "the default",
               pushes[pi], pcts[q], n, front, noView, contacts, vHit, minW);
      }
    }
  }
  CHECK(bad == 0 && killed == 0 && travelBad == 0 && frontAll >= 20 && minUndisturbed >= 0.20f,
        "near the scene at the default margins: no contact with a push up to 0.137 m/s^2 (%d), the undisturbed moves >= 0.20 m from the wall (%.2f), every FRONT stop's travel within v 0.55 s + v^2 / "
        "(2 BRAKE) + 0.05 m (%d of %d over; worst %+.2f m), never KILLED (%d)", bad, minUndisturbed, travelBad,
        frontAll, worstOver, killed);
  simMargins(0);
}

/* The review of 2026-10-17 (docs/autonomy-a2b.md 34.1, mvfix FRONT): the 0.30 m floor parks the drone where a sideways
 * MOVE had no live front stop and no near-scene brake. The largest sideways MOVE (0.5 x D, either side) from D0 0.32 /
 * 0.36 / 0.42 m at 100-300 %, k 0.6 / 0.8 / 1, 3 seeds, undisturbed or with a forward push of 0.10 / 0.137 / 0.20
 * m/s^2 while it runs, with the default mvfix against without FRONT: contacts with the wall ahead. The default must
 * have none up to the forensics' 0.137 (0.8 deg; without FRONT 3 at 0.10 and 5 at 0.137, up to 0.18 m/s), at most 2
 * FRONT stops undisturbed, and never KILLED; the 0.20 push is information. */
static void testSideNearScene(void)
{
  static const float d0s[3] = {0.32f, 0.36f, 0.42f};
  static const float ks[3] = {0.6f, 0.8f, 1.0f};
  static const uint16_t pcts[3] = {100, 200, 300};
  static const float pushes[4] = {0.0f, 0.10f, 0.137f, 0.20f};
  int bad = 0, killed = 0, frontQuiet = 0, nAll = 0, oldBad = 0;
  for (int fx = 0; fx < 2; fx++) {
    for (int pi = 0; pi < 4; pi++) {
      for (int di = 0; di < 3; di++) {
        int n = 0, touch = 0, front = 0;
        float vHit = 0.0f, minW = 9.0f;
        for (int q = 0; q < 3; q++) {
          for (int ki = 0; ki < 3; ki++) {
            for (int sg = 0; sg < 2; sg++) {
              for (uint32_t sd = 0; sd < 3; sd++) {
                scen_t sc = baseScen(91000u + 1000u * (uint32_t)q + 100u * (uint32_t)ki + 10u * (uint32_t)di + sd +
                                         7u * (uint32_t)sg,
                                     3.0f - d0s[di]);
                sc.oneMove = true;
                sc.kFix = ks[ki];
                sc.eDmax = 0.0f;
                sc.mvSpeed = pcts[q];
                sc.margins = 0;
                const int16_t r = (int16_t)floorf(AUTO_MV_LAT_D_REL * d0s[di] * 100.0f) - 1;
                sc.oneR = sg ? r : (int16_t)-r;
                sc.oneF = 0;
                sc.fwdPush = pushes[pi];
                sc.pushAtS = 0.0f;
                sc.pushS = 30.0f;
                sc.fixSet = true;
                sc.fixBits = (uint8_t)(fx ? (AUTO_MVFIX_DEFAULT & ~AUTO_MVFIX_FRONT) : AUTO_MVFIX_DEFAULT);
                simMargins(0);
                result_t res;
                runSim(&sc, &res);
                if (res.oneWhy != 0) {
                  continue;
                }
                n++;
                minW = fminf(minW, res.minWall);
                touch += res.wallTouch;
                if (res.wallTouch) {
                  vHit = fmaxf(vHit, res.wallTouchV);
                }
                front += res.oneEnd == AUTO_MVE_FRONT;
                if (fx == 0) {
                  nAll++;
                  killed += res.killed;
                  bad += res.wallTouch && pushes[pi] <= 0.137f;
                  frontQuiet += pi == 0 && res.oneEnd == AUTO_MVE_FRONT;
                } else {
                  oldBad += res.wallTouch && pushes[pi] <= 0.137f;
                }
              }
            }
          }
        }
        printf("info: sideways near the scene (%s), D0 %.2f m, push %.3f m/s^2: %d moves, FRONT %d, contacts %d (at <= "
               "%.2f m/s), the closest true approach %.2f m\n", fx ? "without FRONT" : "the default mvfix", d0s[di],
               pushes[pi], n, front, touch, vHit, minW);
      }
    }
  }
  CHECK(nAll >= 200 && bad == 0 && killed == 0 && frontQuiet <= 2,
        "sideways from 0.32-0.42 m at the defaults (mvfix FRONT): no contact with a push up to 0.137 m/s^2 (%d; %d "
        "without FRONT), at most 2 FRONT stops undisturbed (%d), never KILLED (%d), %d moves", bad, oldBad, frontQuiet,
        killed, nAll);
  simMargins(0);
}

/* The 300 % forensics' cases (2026-10-17, firmware-backups/handoff-2026-10-12/overshoot300-verdict.md; the move fixes,
 * auto_motion.h "move fixes", docs/autonomy-a2b.md 33.8): one MOVE forward or back 1.00 m from D 2.3 m (flight 6's)
 * at 100 / 200 / 300 %, 4 seeds, (A) the drone drifting back when Claude sends it: a 0.8 deg nose-up pitch error the
 * hold has not learned from 3 s before the send (flight 6: ~0.8 deg of hover lean shift, 0.35 m of drift between the
 * moves, 0.15-0.17 m/s back at move 2's acceptance; here 0.10-0.17 m/s at the send), or a 0.17 m/s kick 0.5 s before;
 * (B) a 0.8 deg pitch error ramped in over 5 s from the acceptance (either sign), the move fixes (mvfix 31) against
 * without them (0). Measured: the true travel from where the drone was when the MOVE was sent to the LAND command (2 s
 * after its end) against the command; the travel the drone reported at its end (its measured progress at its D)
 * against the true one then; its end. */
typedef struct {
  int n, ends[16];
  int doneOff;              /* ended DONE with its true travel more than 0.20 m off the command (a DONE that hides it) */
  float errMax, errSum, repMax, vSendMax, vSendMin, stillMax;
} dlAgg_t;

static void driftLeanRun(uint16_t pct, int16_t f, float driftDeg, float kickV, float lean, uint32_t seed, int fixBits,
                         dlAgg_t *a)
{
  scen_t sc = baseScen(seed, 3.0f - 2.3f);
  sc.oneMove = true;
  sc.oneF = f;
  sc.mvSpeed = pct;
  sc.driftDeg = driftDeg;
  sc.kickV = kickV;
  sc.driftAtS = kickV != 0.0f ? 0.5f : 3.0f;
  sc.leanDeg = lean;
  sc.leanS = 5.0f;
  sc.fixSet = fixBits >= 0;   /* (< 0: the defaults' mvfix) */
  sc.fixBits = (uint8_t)(fixBits >= 0 ? fixBits : 0);
  result_t r;
  runSim(&sc, &r);
  if (r.oneWhy != 0 || r.oneEnd == AUTO_MVE_NONE) {
    a->ends[0]++;
    return;
  }
  const float want = 0.01f * (float)f;
  const float err = fabsf(r.mvF - want);
  const float rep = r.endD0 > 0.0f ? r.endD0 * (1.0f - expf(-r.endPFwd)) : 0.0f;
  if (a->n == 0) {
    a->vSendMin = 9.0f;
  }
  a->n++;
  a->ends[r.oneEnd & 15]++;
  a->doneOff += r.oneEnd == AUTO_MVE_DONE && err > 0.20f;
  a->errMax = fmaxf(a->errMax, err);
  a->errSum += err;
  a->repMax = fmaxf(a->repMax, fabsf(rep - r.endTrueF));
  a->vSendMax = fmaxf(a->vSendMax, r.vAtSend);
  a->vSendMin = fminf(a->vSendMin, r.vAtSend);
  a->stillMax = fmaxf(a->stillMax, r.stillS);
}

static void testDriftLean(void)
{
  static const uint16_t pcts[] = {100, 200, 300};
  /* cls: 0 no disturbance, 1 a transient drift at the send (a kick), 2 a persistent one (a lean step before the
   * send), 3 a lean ramped in during the move */
  static const struct {
    int16_t f;
    float driftDeg, kickV, lean;
    int cls;
    const char *what;
  } cs[] = {
    {-100, 0.0f, 0.0f, 0.0f, 0, "back 1.00, no disturbance"},
    {100, 0.0f, 0.0f, 0.0f, 0, "forward 1.00, no disturbance"},
    {-100, 0.8f, 0.0f, 0.0f, 2, "back 1.00, drifting back (0.8 deg from 3 s before)"},
    {100, 0.8f, 0.0f, 0.0f, 2, "forward 1.00, drifting back (0.8 deg from 3 s before)"},
    {-100, 0.0f, 0.17f, 0.0f, 1, "back 1.00, a 0.17 m/s kick back 0.5 s before"},
    {100, 0.0f, 0.17f, 0.0f, 1, "forward 1.00, a 0.17 m/s kick back 0.5 s before"},
    {-100, 0.0f, 0.0f, 0.8f, 3, "back 1.00, 0.8 deg nose up over 5 s"},
    {-100, 0.0f, 0.0f, -0.8f, 3, "back 1.00, 0.8 deg nose down over 5 s"},
    {100, 0.0f, 0.0f, -0.8f, 3, "forward 1.00, 0.8 deg nose down over 5 s"},
    {100, 0.0f, 0.0f, 0.8f, 3, "forward 1.00, 0.8 deg nose up over 5 s"},
  };
  const int nc = (int)(sizeof cs / sizeof cs[0]);
  static const char *const clsName[4] = {"no disturbance", "a kick at the send", "a lean step before the send",
                                         "a lean in the move"};
  /* [variant][class] (variant 1 the default mvfix, 0 none, 2 the default and STILL): the worst error of a normal end
   * (DONE / SHORT / LONG), the other ends, the DONEs more than 0.20 m off, the runs; the reported travel's worst error
   * against the true one */
  float worst[3][4], worstRep[3] = {0.0f, 0.0f, 0.0f};
  int notNormal[3][4], doneOff[3][4], runs[3][4];
  memset(worst, 0, sizeof(worst));
  memset(notNormal, 0, sizeof(notNormal));
  memset(doneOff, 0, sizeof(doneOff));
  memset(runs, 0, sizeof(runs));
  float stillMax = 0.0f;
  static const char *const varName[3] = {"without them", "the move fixes", "the move fixes and STILL"};
  for (int fx = 1; fx >= 0; fx = fx == 1 ? 2 : (fx == 2 ? 0 : -1)) {
    for (int q = 0; q < 3; q++) {
      for (int c = 0; c < nc; c++) {
        dlAgg_t a;
        memset(&a, 0, sizeof(a));
        float errNormal = 0.0f;
        for (uint32_t sd = 0; sd < 4; sd++) {
          dlAgg_t one;
          memset(&one, 0, sizeof(one));
          driftLeanRun(pcts[q], cs[c].f, cs[c].driftDeg, cs[c].kickV, cs[c].lean, 7100u + 10u * (uint32_t)c + sd,
                       fx == 1 ? -1 : (fx == 2 ? (int)(AUTO_MVFIX_DEFAULT | AUTO_MVFIX_STILL) : 0), &one);
          const int normal = one.ends[AUTO_MVE_DONE] + one.ends[AUTO_MVE_SHORT] + one.ends[AUTO_MVE_LONG];
          if (normal) {
            errNormal = fmaxf(errNormal, one.errMax);
          }
          /* (merge) */
          for (int e = 0; e < 16; e++) {
            a.ends[e] += one.ends[e];
          }
          if (one.n) {
            if (a.n == 0) {
              a.vSendMin = one.vSendMin;
            }
            a.vSendMin = fminf(a.vSendMin, one.vSendMin);
          }
          a.n += one.n;
          a.doneOff += one.doneOff;
          a.errMax = fmaxf(a.errMax, one.errMax);
          a.errSum += one.errSum;
          a.repMax = fmaxf(a.repMax, one.repMax);
          a.vSendMax = fmaxf(a.vSendMax, one.vSendMax);
          a.stillMax = fmaxf(a.stillMax, one.stillMax);
        }
        printf("info: drift / lean, %s, mvspeed %u, %s (%.2f-%.2f m/s back at the send, the still wait <= %.2f s): "
               "DONE %d SHORT %d LONG %d PROGRESS %d other %d; the true end (2 s after) against the command: mean %.2f "
               "max %.2f m (DONE more than 0.20 m off: %d); the reported travel against the true one: max %.2f m\n",
               varName[fx], pcts[q], cs[c].what, a.vSendMin, a.vSendMax, a.stillMax,
               a.ends[AUTO_MVE_DONE], a.ends[AUTO_MVE_SHORT], a.ends[AUTO_MVE_LONG], a.ends[AUTO_MVE_PROGRESS],
               a.ends[0] + a.ends[AUTO_MVE_FRONT] + a.ends[AUTO_MVE_VIEW_FAST] + a.ends[AUTO_MVE_NO_VIEW],
               a.n ? a.errSum / (float)a.n : 0.0f, a.errMax, a.doneOff, a.repMax);
        const int k = cs[c].cls;
        runs[fx][k] += 4;
        notNormal[fx][k] += 4 - (a.ends[AUTO_MVE_DONE] + a.ends[AUTO_MVE_SHORT] + a.ends[AUTO_MVE_LONG]);
        doneOff[fx][k] += a.doneOff;
        worst[fx][k] = fmaxf(worst[fx][k], errNormal);
        worstRep[fx] = fmaxf(worstRep[fx], a.repMax);
        if (fx == 1) {
          stillMax = fmaxf(stillMax, a.stillMax);   /* (the defaults: none) */
        }
      }
    }
  }
  for (int k = 0; k < 4; k++) {
    printf("info: drift / lean, %s (%d runs): the worst normal end (DONE / SHORT / LONG) %.2f m off the command with "
           "the move fixes, %.2f m without, %.2f m with STILL too; ended otherwise (PROGRESS ...) %d / %d / %d; DONE more "
           "than 0.20 m off %d / %d / %d\n", clsName[k], runs[1][k], worst[1][k], worst[0][k], worst[2][k],
           notNormal[1][k], notNormal[0][k], notNormal[2][k], doneOff[1][k], doneOff[0][k], doneOff[2][k]);
  }
  printf("info: drift / lean: the reported travel's worst error against the true one at the end %.2f m with the move "
         "fixes, %.2f m without; the defaults' wait for a still drone %.2f s. NOT MET: a persistent 0.8 deg the hold has "
         "not learned (a lean step before the send, or a lean in the move) leaves the true end 0.3-1.1 m off with or "
         "without the fixes (the hold's position gain: KI s = 2-3.7 deg per ln unit, 0.8 deg of lean is a 0.2-0.4 ln "
         "offset); the fixes report it (LONG / SHORT / PROGRESS) instead of a DONE\n", worstRep[1], worstRep[0],
         stillMax);
  CHECK(notNormal[1][0] == 0 && doneOff[1][0] == 0 && worst[1][0] <= 0.20f && worst[1][0] <= worst[0][0] + 0.03f,
        "drift / lean: undisturbed, every move DONE within 0.20 m with the move fixes (worst %.2f m, %.2f without)",
        worst[1][0], worst[0][0]);
  CHECK(notNormal[1][1] <= notNormal[0][1] && worst[1][1] <= 0.20f && doneOff[1][1] == 0,
        "drift / lean: a transient drift of 0.11-0.14 m/s at the send: with the move fixes no more non-normal ends "
        "(%d, %d without), the normal ends within 0.20 m (%.2f; %.2f without), no DONE more than 0.20 m off",
        notNormal[1][1], notNormal[0][1], worst[1][1], worst[0][1]);
  CHECK(4 * (doneOff[1][2] + doneOff[1][3]) <= doneOff[0][2] + doneOff[0][3] && doneOff[0][2] + doneOff[0][3] >= 8,
        "drift / lean: an unlearned 0.8 deg lean: the DONEs that hide more than 0.20 m %d with the move fixes, at most a "
        "quarter of the %d without (LONG / SHORT / PROGRESS report it)", doneOff[1][2] + doneOff[1][3],
        doneOff[0][2] + doneOff[0][3]);
  CHECK(worstRep[1] <= 0.25f && stillMax == 0.0f, "drift / lean: the travel the drone reports at its end within 0.25 m "
        "of the true one (%.2f), no still wait with the defaults (%.2f s)", worstRep[1], stillMax);
}

int main(int argc, char **argv)
{
  if (argc >= 2) {
    simKmax = strtof(argv[1], NULL);
  }
  printf("info: the hold's gain scale kmax %.2f (docs/autonomy-a2b.md 20; 1 = the hold before it)\n", simKmax);
  if (argc >= 3 && strcmp(argv[2], "shrep") == 0) {   /* the SET_HEIGHT replays alone (development) */
    testSetHeightReplaySim();
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
  }
  if (argc >= 3 && strcmp(argv[2], "front") == 0) {   /* the front stop's braking alone (development) */
    testFrontBraking();
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
  }
  if (argc >= 3 && strcmp(argv[2], "side") == 0) {   /* the sideways moves near the scene alone (mvfix FRONT) */
    testSideNearScene();
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
  }
  if (argc >= 3 && strcmp(argv[2], "drift") == 0) {   /* the 300 % forensics' drift and lean cases alone */
    testDriftLean();
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
  }
  if (argc >= 3 && strcmp(argv[2], "dev") == 0) {   /* the tests the move fixes touch (development) */
    testFarMoves();
    testPlanTransient();
    testFrozenSim();
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
  }
  if (argc >= 3 && strcmp(argv[2], "real300") == 0) {
    /* 'real 300 %' (2026-10-17, docs/autonomy-a2b.md 35): the fast moves, the front braking, the sideways moves near
     * the scene and the drift / lean cases with NVS mvacc 30 / mvsettle 1000 latched at every ARM (the caps of 4dea542,
     * the move fixes and margins of d3bedee). Information: the checks are calibrated at the defaults */
    autoMotionSetFastCaps(30, 1000);
    printf("info: real300: mvacc 30 cm/s^2, mvsettle 1000 ms at every ARM\n");
    testFastMoves();
    testFrontBraking();
    testSideNearScene();
    testDriftLean();
    autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
  }
  if (argc >= 3 && strcmp(argv[2], "fast") == 0) {   /* the fast moves alone (development) */
    testFastMoves();
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
  }
  if (argc >= 3 && strcmp(argv[2], "sh") == 0) {   /* the SET_HEIGHT loop alone (development) */
    testSetHeightLoop();
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
  }
  testMission();
  testGuards();
  testLandAfterMove();
  testFarMoves();
  testFarMovesSeeds();
  testFastMoves();
  if (simKmax > 1.0f) {   /* (both kmax inside) */
    testPlanTransient();
    testLongMovesGrid();
    testLatchedG();
    testBelowKLow();
    testPlanFloorSim();
  } else {
    printf("info: the long-move grid and the plan's G transient run with the default kmax only (both kmax inside)\n");
  }
  testKFloorSim();
  testFrontBraking();
  testSideNearScene();
  testDriftLean();
  testDepthStep();
  testTakeoffKick();
  testFrozenSim();
  testHighLandSim();
  testHigh200Sim();
  testDownLate();
  testStopInBob();
  testFastTurn();
  testContactSim();
  testLateBlindLands();
  testWallSim();
  testDrain();
  testDrainVzK();
  testWeakTakeoff();
  testLongHover();
  testKDrift();
  testSetHeightLoop();
  testSetHeightReplaySim();
  if (simKmax > 1.0f) {
    testTakeoffGrid();
  } else {
    printf("info: the gain scale's take-off grid runs with the default kmax only\n");
  }
  printf("info: every run without a hand, a gust or a ceiling (%d, the scenarios above): SUPPORT in the air %d\n",
         fpRuns, fpSupAir);
  CHECK(fpRuns > 400 && fpSupAir == 0, "the sims without a hand: never the controller's SUPPORT in the air (%d), over "
        "%d sorties", fpSupAir, fpRuns);
  /* The hand tests fly at kmax 1 whatever the argument (docs/autonomy-a2b.md 20.4: the palm tests' seed set) */
  {
    const float kSave = simKmax;
    simKmax = 1.0f;
    testHandHold();
    simKmax = kSave;
  }
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
