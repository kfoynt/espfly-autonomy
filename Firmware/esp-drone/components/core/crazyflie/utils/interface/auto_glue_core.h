/*
 * ESP-Drone Firmware
 *
 * auto_glue_core.h - The pure parts of the autonomy glue (stage A2b,
 * docs/autonomy-a2b.md): the reset-lock RTC record and its boot rule, the
 * autonomy settings table, the phone hook, the setpoint override, the
 * thrust selection, the IDLE fast-path rule, the camera-view tracker and the
 * AUTO_STATUS fill. modules/src/autonomy.c is the drone side.
 *
 * No ESP-IDF or FreeRTOS header: host-tested by
 * tools/tests/test_auto_glue_core.c. The static inline functions are used
 * inside autonomy.c's IRAM path (they inline there; nothing here calls
 * flash code). Of the out-of-line ones (auto_glue_core.c), autoStatusFill and
 * autoTripBits are IRAM (AUTO_HOT) and run at 10 Hz and on transitions; the
 * others run at boot, on a transition or from the console.
 */
#ifndef AUTO_GLUE_CORE_H_
#define AUTO_GLUE_CORE_H_

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "stabilizer_types.h"   /* setpoint_t, modeAbs, modeDisable */
#include "auto_fsm.h"           /* autoState_t, autoThrust_t, AUTO_* thresholds */
#include "auto_proto.h"         /* autoStatusMsg_t, AUTO_STF_*, AUTO_TRF_* */
#include "camera_hold.h"   /* (CAM_HOLD_SCENE_RES_*: holdres) */

/* ---- reset-lock record (docs/autonomy-a2.md section 6) ------------------ */

#define AUTO_RTC_MAGIC        0xA2B0C0DEu
#define AUTO_RTC_VERSION      1u
#define AUTO_RTC_TRANSITIONS  8

/* autoRtcRecord_t.locked (and prevLocked, the boot before). */
#define AUTO_RTC_UNLOCKED     0u   /* not locked at boot (a battery power-on) */
#define AUTO_RTC_LOCKED       1u   /* locked at boot, still locked */
#define AUTO_RTC_USB_UNLOCKED 2u   /* locked at boot, then cleared by the USB console's 'auto unlock'
                                    * (user decision 2026-10-12, evening) */

typedef struct {
  uint32_t ms;             /* FreeRTOS tick ms of the transition */
  uint8_t from, to;        /* autoState_t */
  uint8_t reason;          /* autoReason_t */
  uint8_t vzReason;        /* vzReason_t: autoFsmOut_t.vzLandReason at the transition */
} autoRtcTransition_t;

/* Lives in RTC_NOINIT memory (autonomy.c). crc covers every byte after it. */
typedef struct {
  uint32_t magic;
  uint32_t crc;            /* autoProtoCrc32 of bytes [8, sizeof) */
  uint32_t version;
  uint32_t boots;          /* boots since the last battery power-on (1 on that boot) */
  uint8_t lastState, lastReason, lastMode, locked;           /* this boot so far (locked: AUTO_RTC_*LOCKED) */
  uint8_t prevState, prevReason, prevMode, prevLocked;       /* the boot before (0 after a power-on) */
  uint32_t transitions;    /* this boot */
  uint32_t prevTransitions;
  autoRtcTransition_t trans[AUTO_RTC_TRANSITIONS];           /* ring: entry i at i % N, newest (transitions-1) % N */
} autoRtcRecord_t;

/* Magic, version and CRC all match. */
bool autoRtcValid(const autoRtcRecord_t *r);
/* The boot rule: locked = !(powerOnReset && magic absent). A battery
 * power-on is the only boot on which RTC memory lost power; a power-on with
 * the magic present (EN button, a supply dip that kept RTC memory), a
 * present magic with a bad CRC and every other reset reason all lock. */
bool autoRtcBootLocked(bool powerOnReset, const autoRtcRecord_t *r);
/* Rewrites the record for this boot (magic, version, CRC): keeps the
 * previous boot's last state/reason/mode/lock/transitions when r was valid,
 * zeroes them otherwise; boots = valid ? boots + 1 : 1. */
void autoRtcStartBoot(autoRtcRecord_t *r, bool locked);
/* Appends one transition and updates last* and the CRC (stabilizer task,
 * on a transition only: ~90 bytes and a bitwise CRC, a few us). */
void autoRtcNoteTransition(autoRtcRecord_t *r, uint32_t ms, autoState_t from, autoState_t to, autoReason_t reason,
                           uint8_t vzReason, autoMode_t mode);
/* The USB console's 'auto unlock' cleared this boot's reset-lock: a valid
 * record that is locked (AUTO_RTC_LOCKED) becomes AUTO_RTC_USB_UNLOCKED,
 * with a new CRC, so the next boot's prevLocked tells a USB unlock from a
 * battery power-on. false (nothing written) for an invalid record (its CRC
 * is never re-made over unknown bytes) or one not locked. It never changes
 * the boot rule: autoRtcBootLocked() reads the magic only, so the next
 * reset locks again. Called by the glue inside the flash interlock, where
 * the stabilizer cannot write the record (autonomy.c autonomyUsbUnlock). */
bool autoRtcNoteUsbUnlock(autoRtcRecord_t *r);

/* ---- settings (docs/autonomy-a2.md section 8.5, user decisions 2026-10-12) */

typedef enum {
  AUTO_NVS_U8 = 1,
  AUTO_NVS_U16 = 2,
  AUTO_NVS_I16 = 3,
  AUTO_NVS_U32 = 4,
} autoNvsType_t;

typedef enum {
  AUTO_SET_AUTOON = 0,     /* u8  0/1, default 0: autonomy enabled (G7) */
  AUTO_SET_APPREQ,         /* u8  0/1, default 1: phone app required (G5, APP_SILENT) */
  AUTO_SET_AUTOGEST,       /* u8  0/1, default 1: per-sortie consent gesture (G10) */
  AUTO_SET_VMIN,           /* u16 mV 3700..4200, default 3800 (AUTO_VMIN_DEFAULT): resting battery at ARM (G3) */
  AUTO_SET_AUTOAIR,        /* u16 ms 2000..6000, default 4500: air clock until the hover wait */
  AUTO_SET_AUTOWAIT,       /* u32 ms 0..120000 (u16 0..60000 until 2026-10-14, still read), default 5000: the hover
                              wait for the Mac */
  AUTO_SET_AUTOACCEL,      /* u16 ms 0..3000, default 2000: HOVER on the accelerometer alone */
  AUTO_SET_VZK,            /* u32 thrust x V 120000..180000, default 165000 (VZ_K_DEFAULT) */
  AUTO_SET_VZBAIR,         /* i16 0.1 mg -300..300, default -23 */
  AUTO_SET_VZCAPLO,        /* u16 x1000 1000..2000, default 1650 */
  AUTO_SET_PHOTOCOL,       /* u8  0/1, default 0: grey (1: colour photos, YUYV, always on for the boot: the driver's
                              copy splits every frame; opt-in), read by the camera at boot (camera_flow.c,
                              docs/autonomy-a2b.md 28) */
  AUTO_SET_CVQLATE,        /* u8  1..5, default 1 (off): the camera height observer's late offset noise multiplier,
                              read at boot (cam_vz_glue.c; cam_vz_obs.h qLateMult, docs/autonomy-a2b.md 25) */
  AUTO_SET_CVQLATES,       /* u8  s 20..120, default 40: its step after the sortie's first lift-off (qLateS) */
  AUTO_SET_CVT2K,          /* u8  0..200, default 0 (off): the camera height observer's offset drift rate gain kR x 100
                              (1/s), read at boot (cam_vz_glue.c; cam_vz_obs.h t2kR, docs/autonomy-a2b.md 29); refused
                              with cvqlate > 1 (R1 and T2 together) */
  AUTO_SET_CVT2R,          /* i16 ug/s -350..350, default 124: its prior rate (t2PriorMgS x 1000, within the bound) */
  AUTO_SET_MVSPEED,        /* u16 % 100..300, default 100: the motion speed step, latched at ARM (auto_motion.h "fast
                              moves", docs/autonomy-a2b.md 31; 2026-10-16) */
  /* the guard margins (user decisions 2026-10-17, auto_motion.h "guard margins", docs/autonomy-a2b.md 32), latched at
   * ARM with mvspeed (autonomy.c sortieSpeedLatch -> autoMvMarginsFrom, autoMotionSetMargins) */
  AUTO_SET_MVFLOOR,        /* u8  cm 0 or 20..100, default 30 (50 before): a forward MOVE's floor at k mvklow (TOO_CLOSE);
                              the TAKEOFF plan's is this + 5 cm; 0 (2026-10-18, auto_motion.h "guards off"): no floor */
  AUTO_SET_MVFSTOP,        /* u8  cm 0 or 15..80, default 25 (40 before): the live front stop (FRONT), at most mvfloor - 5;
                              0: off */
  AUTO_SET_MVPROGM,        /* u8  cm 0 or 10..60, default 20 (15 before): the progress guard's base margin; 0: the guard
                              off (with mvprogr) */
  AUTO_SET_MVPROGR,        /* u8  % 0 or 10..100, default 40 (30 before): the progress guard's part of the move; 0: off */
  AUTO_SET_MVLATM,         /* u8  cm 15..100, default 40 (25 before): the sideways margin's floor */
  AUTO_SET_MVNEAR,         /* u8  cm 0..150, default 80 (60 until the review of 2026-10-17; 0 = off, as before): a
                              forward MOVE's (mvfix FRONT: any horizontal MOVE's) view lost with the scene this near at
                              k 0.6 brakes, end FRONT */
  AUTO_SET_MVFIX,          /* u8  bits 0..63, default 59 (27 until the review of 2026-10-17; 0 = none, as before): the
                              move fixes of the 300 % forensics and its review (auto_motion.h AUTO_MVFIX_*: 1 DRIFT, 2
                              LONG, 4 STILL (off by default), 8 GAIN, 16 BRAKE, 32 FRONT), latched at ARM */
  /* the fast steps' acceleration and settle (user request 2026-10-17 "no, I want real 300%", auto_motion.h "the fast
   * steps' acceleration and settle as settings", docs/autonomy-a2b.md 35), latched at ARM with mvspeed (autonomy.c
   * sortieSpeedLatch -> autoMotionSetFastCaps), read above 100 % only */
  AUTO_SET_MVACC,          /* u8  cm/s^2 10..40, default 20 (the cap of d3bedee; 30 the 300 % flights' of 4dea542): the
                              profile's acceleration cap above 100 % */
  AUTO_SET_MVSETTLE,       /* u16 ms 1000..3000, default 2500 (d3bedee's; 1000 4dea542's): the settle's floor above
                              100 % */
  AUTO_SET_CVHB,           /* u8  x0.1 0 or 10..80, default 0 (off): the camera height observer's healthy offset
                              speed-up, hbMult x 10, read at boot (cam_vz_glue.c; cam_vz_obs.h hbMult, docs/autonomy-
                              a2b.md 37); 1..9 refused; refused at boot with cvqlate > 1 or cvt2k > 0 */
  /* the guards down to off (user decision 2026-10-18 "remove the guards or make them absolutely minimum", auto_motion.h
   * "guards off", docs/autonomy-a2b.md 38), latched at ARM with the margins (sortieSpeedLatch -> autoMvMarginsExtra,
   * autoMotionSetFwdRate) */
  AUTO_SET_MVKLOW,         /* u8  % 60..100, default 60 (AUTO_MV_K_LOW x 100): the lowest expansion scale the floor, the
                              front stop, the near brake and the measured lower bound of D allow for; 100: none */
  AUTO_SET_MVLAT,          /* u8  % 0..200, default 50 (AUTO_MV_LAT_D_REL x 100): a sideways MOVE at most this x D
                              (TOO_WIDE); 0: no cap */
  AUTO_SET_MVWMAX,         /* u8  x0.01/s 10..40, default 15 (AUTO_MV_W_FWD_MAX x 100): the expansion rate cap, a forward
                              move's speed at most this x its end distance at 100 % */
  /* the VIEW_FAST timer (2026-10-22, auto_motion.h "fast moves"; the table flight's two VIEW_FAST stops with the
   * camera healthy), latched at ARM with the fast caps (sortieSpeedLatch -> autoMotionSetViewDeb) */
  AUTO_SET_MVVIEW,         /* u16 ms 0 or AUTO_MV_FAST_VIEW_DEB_MS..1000, default 0: the time a fast MOVE's view may
                              stay lost before it is stopped with its brake (VIEW_FAST); 0 the profile's timer (323 ms
                              at native QVGA, 150 at 160x120); 1..profile - 1 refused: the timer only ever gets longer */
  /* the vertical controller's runaway guard (user request 2026-10-18; vert_ctrl.h EST_RANGE), latched at ARM with
   * vzcaplo (sortie start -> vzStart) */
  AUTO_SET_VZEST,          /* u8  0/1, default 1: the EST_RANGE LAND (the height estimate outside hMin..hMax + hOff);
                              0: never that LAND (K_RANGE, HOLD_CAPPED, CLIMB_* and the FSM's SINK stay) */
  AUTO_SET_VZESTMS,        /* u16 ms 0..3000, default 0: the out-of-range condition must hold this long before the
                              EST_RANGE LAND (0: on its first step, as before) */
  /* the camera hold's HOVER scene gate residual limit (2026-10-22, the table flight's cause; camera_hold.h
   * CAM_HOLD_SCENE_RES_*), latched at ARM (sortieSpeedLatch -> cameraFlowSetSceneResidual) */
  AUTO_SET_HOLDRES,        /* u8  grey levels x 10, 0 or 10..255, default 160 at native QVGA (80 = the legacy 8.0 on
                              the 160x120 profile): a frame whose fit residual reaches this is refused by the scene
                              gate (the hold coasts); 0: no residual test (the lateral and expansion tests stay) */
  /* the left slide of the forward MOVEs (2026-10-22; camera_hold.h latGain, auto_motion.h autoMotionSetFfRoll), both
   * latched at ARM (sortieSpeedLatch -> cameraFlowSetMoveLatGain, autoMotionSetFfRoll); defaults = the hold and the
   * motions as before */
  AUTO_SET_MVLATK,         /* u16 % 100..400, default 100: the hold's lateral position gain (KI) on a MOVE's frames,
                              against a steady lateral pull in forward flight (300: a third of the slide) */
  AUTO_SET_MVFFR,          /* i16 x0.1 deg per m/s -50..50, default 0: a MOVE's RUN leans this x the reference forward
                              speed to the right (negative: left) on top of its feedforward (the fit: ~18 = 1.8 deg
                              per m/s left on 7 of 9 logged moves, 0 on two) */
  AUTO_SET_COUNT
} autoSettingId_t;

typedef struct {
  const char *name;        /* NVS key ("imucal" namespace) and USB name */
  autoNvsType_t type;
  int32_t min, max;
  int32_t def;             /* derived from the header constants (AUTO_VMIN_DEFAULT, VZ_K_DEFAULT, ...) */
  const char *unit;        /* "", "mV", "ms", "Kv", "0.1mg", "x1000" */
  autoNvsType_t legacyType;   /* 0, or the type the key had before (autowait: AUTO_NVS_U16 until 2026-10-14). NVS
                                 lookups are typed (an item of another type reads as absent): a key absent with its
                                 type is read once more with this one (autonomyInit), and a write or a reset erases
                                 every item of the key first (autoSettingStoreErase), so the old item never comes
                                 back. docs/autonomy-a2b.md 18.2 */
} autoSettingInfo_t;

const autoSettingInfo_t *autoSettingInfo(autoSettingId_t id);   /* NULL if out of range */
int autoSettingFind(const char *name);                           /* -1 if unknown */
bool autoSettingInRange(autoSettingId_t id, int32_t value);
/* A key whose 0 means off while its other values start higher (cvhb 10, mvfloor 20, mvfstop 15, mvprogm 10, mvprogr
 * 10): that lowest "on" value, 1..min - 1 being refused by autoSettingInRange; 0 for every other key (its range is
 * min..max as the table says). The console's range message. */
int32_t autoSettingOnMin(autoSettingId_t id);

/* ---- phone hook (section 8.2) ------------------------------------------- */

#define AUTO_APP_INPUT_THRUST   1000    /* raw app thrust at or above this is input */
#define AUTO_APP_INPUT_DEG      2.0f    /* |roll| or |pitch| at or above this is input */
#define AUTO_GESTURE_PUSH_DEG   10.0f   /* consent gesture: |roll| at or above this, then released below 2 deg */

/* One writer (CRTP RX task, through autonomyNoteAppPacket, which stamps
 * nowMs = T2M(xTaskGetTickCount()) itself), readers in the stabilizer task:
 * aligned 32-bit words and bools only. */
typedef struct {
  volatile uint32_t lastAppMs, lastInputMs, gestureMs;
  volatile uint32_t inputCount;    /* monotonic */
  volatile bool any, anyInput, gestured, pushed, thrustZero;
} autoAppHook_t;

static inline void autoAppHookNote(autoAppHook_t *h, float rollDeg, float pitchDeg, uint16_t rawThrust,
                                   uint32_t nowMs)
{
  const float ar = fabsf(rollDeg), ap = fabsf(pitchDeg);
  h->lastAppMs = nowMs;
  if (rawThrust >= AUTO_APP_INPUT_THRUST || ar >= AUTO_APP_INPUT_DEG || ap >= AUTO_APP_INPUT_DEG) {
    h->lastInputMs = nowMs;
    h->anyInput = true;
    __atomic_add_fetch(&h->inputCount, 1u, __ATOMIC_RELEASE);
  }
  if (ar >= AUTO_GESTURE_PUSH_DEG) {
    h->pushed = true;
  } else if (h->pushed && ar < AUTO_APP_INPUT_DEG) {
    h->pushed = false;
    h->gestureMs = nowMs;
    h->gestured = true;
  }
  h->thrustZero = rawThrust < AUTO_APP_INPUT_THRUST;
  h->any = true;
}

/* A ms value in a u16 AUTO_STATUS field: saturated at 65535 ("65.5 s or more"; the waits reach 120000). */
static inline uint16_t autoSatMs16(uint32_t ms)
{
  return ms > 65535u ? 65535u : (uint16_t)ms;
}

/* Age of a stamp, UINT32_MAX when never stamped (the FSM's "never").
 * Every stamp and nowMs are T2M(xTaskGetTickCount()) (autonomy.h, "Clock").
 * A stamp written by another task after the stabilizer read nowMs is up to a
 * few ms in the future: its age is 0, never a wrapped ~4e9 ms (a spurious
 * HB_LOST / APP_SILENT on a single step, critique R1). */
static inline uint32_t autoAgeMs(uint32_t nowMs, uint32_t stampMs, bool any)
{
  const int32_t d = (int32_t)(nowMs - stampMs);
  return any ? (d < 0 ? 0u : (uint32_t)d) : UINT32_MAX;
}

/* Age in ms of an esp_timer stamp (the camera frame's frameTimeUs) at nowUs;
 * a stamp newer than nowUs (another core) is 0 old, as flight_recorder.c
 * does it; saturates at UINT32_MAX. */
static inline uint32_t autoAgeUsToMs(uint64_t nowUs, uint64_t stampUs)
{
  if (nowUs <= stampUs) {
    return 0u;
  }
  const uint64_t ms = (nowUs - stampUs) / 1000u;
  return ms > UINT32_MAX ? UINT32_MAX : (uint32_t)ms;
}

/* ---- tilt (section 2.2): acos(gravZ) in degrees without libm ------------ */

/* sensfusion6GetInvThrustCompensationForTilt() is gravZ. libm acosf is flash
 * code; this runs every loop from IRAM (only sqrtf, which linker_camvz.lf
 * already places in IRAM). Abramowitz & Stegun 4.4.45, |error| <= 0.004 deg
 * over gz in [-1, 1] (0.014 deg against float cosf over 0..60 deg), monotone;
 * gz is clamped to [-1, 1]; NaN stays NaN (the FSM's finiteness rule). */
static inline float autoTiltDegFromGravZ(float gz)
{
  if (!(gz == gz)) {
    return gz;
  }
  const float x = gz > 1.0f ? 1.0f : (gz < -1.0f ? -1.0f : gz);
  const float ax = x < 0.0f ? -x : x;
  const float p = 1.5707288f + ax * (-0.2121144f + ax * (0.0742610f + ax * -0.0187293f));
  const float r = sqrtf(1.0f - ax) * p;
  return (x < 0.0f ? 3.14159265f - r : r) * 57.2957795f;
}

/* ---- setpoint override (section 8.1) ------------------------------------ */

/* The whole setpoint: x/y/z disabled, roll and pitch absolute at exactly
 * 0.0f (controller_pid.c applies the camera drift hold only on exact zeros,
 * and camera_flow.c's sticksCentred reads the recorder's rollDes/pitchDes ==
 * 0), yaw absolute at the captured heading (modeAbs bypasses the yaw lock). */
static inline void autoBuildSetpoint(setpoint_t *sp, float thrust, float yawHoldDeg, uint32_t timestamp)
{
  memset(sp, 0, sizeof(*sp));
  sp->mode.x = modeDisable;
  sp->mode.y = modeDisable;
  sp->mode.z = modeDisable;
  sp->mode.roll = modeAbs;
  sp->mode.pitch = modeAbs;
  sp->mode.yaw = modeAbs;
  sp->attitude.roll = 0.0f;
  sp->attitude.pitch = 0.0f;
  sp->attitude.yaw = yawHoldDeg;
  sp->thrust = thrust;
  sp->timestamp = timestamp;
}

/* Section 2 step 2.5: the thrust for the override. VERT_CTRL / VERT_LAND:
 * the controller's command; ZERO / CUT / NONE: 0. A non-finite or negative
 * command gives 0; the result never exceeds thrustMax (the first barrier). */
static inline float autoSelectThrust(autoThrust_t action, float vzThrust, float thrustMax)
{
  float t = (action == AUTO_THR_VERT_CTRL || action == AUTO_THR_VERT_LAND) ? vzThrust : 0.0f;
  if (!(t > 0.0f)) {        /* NaN, negative, zero */
    t = 0.0f;
  }
  if (!(t <= thrustMax)) {  /* also a NaN thrustMax */
    t = thrustMax > 0.0f ? thrustMax : 0.0f;
  }
  return t;
}

#define AUTO_THRUST_MAX_FREE 65535.0f   /* thrustMax while autonomy leaves the setpoint alone (action NONE) */

/* The second barrier's limit (section 2.5): 0 whenever the FSM's action is
 * ZERO or CUT, so ARMED, pre-spin, the release hold, a landed LAND and KILLED
 * keep the motors off at both barriers, not through the override alone
 * (the FSM's own thrustMax is the mode maximum there). NONE: free (65535).
 * VERT_CTRL / VERT_LAND: the FSM's mode maximum, 0 unless finite and >= 0. */
static inline float autoBarrierMax(autoThrust_t action, float fsmThrustMax)
{
  if (action == AUTO_THR_NONE) {
    return AUTO_THRUST_MAX_FREE;
  }
  if (action != AUTO_THR_VERT_CTRL && action != AUTO_THR_VERT_LAND) {
    return 0.0f;   /* ZERO, CUT and any unknown action */
  }
  return (fsmThrustMax >= 0.0f && fsmThrustMax <= AUTO_THRUST_MAX_FREE) ? fsmThrustMax : 0.0f;
}

/* The last-stage clamp (section 2 step 5) on control.thrust. Fails closed: a
 * NaN or out-of-range thrustMax gives 0. thrustMax 65535 (autonomy idle)
 * leaves control.thrust untouched, so manual flight is unchanged; below it a
 * NaN or negative thrust gives 0. */
static inline float autoClampThrust(float thrust, float thrustMax)
{
  if (!(thrustMax >= 0.0f && thrustMax <= AUTO_THRUST_MAX_FREE)) {
    return 0.0f;
  }
  if (thrustMax >= AUTO_THRUST_MAX_FREE) {
    return thrust;
  }
  return thrust > 0.0f ? (thrust < thrustMax ? thrust : thrustMax) : 0.0f;
}

/* ---- IDLE fast path (docs/autonomy-a2b.md section 2.3) ------------------- */

/* true: this loop needs no FSM step. Only in IDLE with the thrust action
 * NONE (the 600 ms release hold steps every loop), no command waiting, no
 * new KILL and no new session; and not on every everyLoops-th loop, when the
 * FSM steps anyway (failedGuards for STATUS, the kill latch, the gesture and
 * ground clocks). */
static inline bool autoIdleSkip(autoState_t state, autoThrust_t lastAction, bool cmdPending, bool killChanged,
                                bool sessionChanged, uint32_t loop, uint32_t everyLoops)
{
  return state == AUTO_IDLE && lastAction == AUTO_THR_NONE && !cmdPending && !killChanged && !sessionChanged &&
         (everyLoops == 0 || (loop % everyLoops) != 0);
}

/* ---- continuous-condition timer (camViewMs, onGroundMs) ----------------- */

typedef struct {
  bool on;
  uint32_t sinceMs;
} autoSinceTrack_t;

/* How long cond has held without a break (0 when it does not hold now). */
static inline uint32_t autoSinceStep(autoSinceTrack_t *t, bool cond, uint32_t nowMs)
{
  if (!cond) {
    t->on = false;
    return 0;
  }
  if (!t->on) {
    t->on = true;
    t->sinceMs = nowMs;
  }
  return nowMs - t->sinceMs;
}

/* ---- AUTO_STATUS (auto_proto.h autoStatusMsg_t) ------------------------- */

/* What the glue adds to the FSM's and the controller's outputs. */
typedef struct {
  uint32_t upMs;
  uint32_t transitions;
  uint32_t killCount;
  bool resetLocked, simPlant, holdApplied, camAwake;
  uint8_t trips;           /* AUTO_TRF_* conditions true now */
  autoCapture_t capture;
  uint16_t photoId;
  float vbat, vbatFastMin; /* V */
  float kPrior;            /* the sortie's vzConfig_t.kPrior (NVS vzK in IDLE) */
  uint8_t lastCmdType, lastCmdResult;
  const autoFsmReport_t *rep;   /* the state machine's report (autoFsmReport()): bytes 96-135; NULL: all 0 */
  bool vzActive;           /* the controller runs this sortie (its K clamp, cap and open loop bits) */
} autoStatusExtra_t;

/* Fills every field but bootNonce (the link's), saturating every integer
 * field and mapping a non-finite float to 0. IRAM (AUTO_HOT): the glue calls it
 * every AUTONOMY_PUBLISH_EVERY-th active loop (10 Hz) and on a transition,
 * never every loop (critique R7). */
void autoStatusFill(autoStatusMsg_t *m, const autoFsmOut_t *o, const vzOutput_t *vz, const autoStatusExtra_t *x);

/* AUTO_TRF_* bits from the FSM's inputs (the conditions behind the trip and
 * rest timers, true now). IRAM (AUTO_HOT), with autoStatusFill only (10 Hz). */
uint8_t autoTripBits(const autoFsmIn_t *in, bool vzAtCap);

#endif /* AUTO_GLUE_CORE_H_ */
