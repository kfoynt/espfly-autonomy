/*
 * ESP-Drone Firmware
 *
 * autonomy.h - Autonomy glue (stage A2b, docs/autonomy-a2b.md): runs the state
 * machine (auto_fsm.h) and the vertical controller (vert_ctrl.h) inside the
 * 1 kHz stabilizer loop, owns the command mailbox the link task posts into
 * (auto_mailbox.h), the phone hook, the reset-lock RTC record, the autonomy
 * NVS settings, the flash-write interlock and the DRY/BENCH sim plant.
 *
 * Callers and the only functions each may call:
 *   - stabilizer.c (stabilizer task): autonomyInit (from stabilizerInit, before
 *     the task is created), autonomyArbitrate (right after
 *     commanderGetSetpoint), autonomyThrustMax (the last-stage clamp on
 *     control.thrust), autonomyNoteApplied (after camVzGlueStep).
 *   - crtp_commander_rpyt.c (CRTP RX task): autonomyNoteAppPacket.
 *   - auto_link.c (link task): the "link task" block below.
 *   - usb_diag.c (USB task): the settings, status, bench and profile calls,
 *     and autonomyUsbUnlock (its only caller: the USB console's 'auto unlock').
 *   - flight_recorder.c (stabilizer task, 100 Hz): autonomyRecordFields.
 *   - flight_rtc_log.c (stabilizer task, 100 Hz): autonomyRecordFields.
 *   - takeoff_aux.c (aux task, 10 ms): autonomyPoll.
 *   - camera_flow.c (camera task, per frame): autonomyHoldBlocked, autonomyGetHoldRef.
 *   - controller_pid.c (stabilizer task, 500 Hz): autonomyHoldBlocked,
 *     autonomyTakeoffIntegratorHold, autonomyBrakeLean (its stale-hold branch only).
 *   - camera_flow.c (camera task, per frame) also: autonomyMvSpeedPct, autonomyMvFixBits.
 *   - att_trim_auto.c (aux task): autonomyActive.
 *   - imu_level_cal_store.c (any task that writes NVS): autonomyFlashBegin /
 *     autonomyFlashEnd around every write, commit and erase.
 * "Any task" functions read a snapshot under a short spinlock or a volatile
 * word; none blocks, none writes flash.
 *
 * Clock: every time stamp the glue keeps (heartbeat, session, phone packets,
 * command latency) is taken inside autonomy.c with T2M(xTaskGetTickCount()),
 * the FSM's clock; no caller passes a time except the stabilizer's nowMs,
 * read from the same tick. The link's esp_timer clock (auto_link.c nowMs())
 * never enters the glue. Ages are autoAgeMs (auto_glue_core.h), clamped at 0
 * when a stamp from the other core is newer than nowMs.
 *
 * Header contract: no ESP-IDF or FreeRTOS header here, so the host harness of
 * auto_link.c (tools/tests/auto_link_host) can include it. Enum values that
 * go on the wire or into logs come from auto_fsm.h / vert_ctrl.h /
 * auto_proto.h (append only).
 */
#ifndef AUTONOMY_H_
#define AUTONOMY_H_

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"   /* setpoint_t, sensorData_t, state_t */
#include "auto_fsm.h"           /* autoState_t, autoMode_t, autoCmd_t, autoCapture_t, autoReason_t */
#include "auto_proto.h"         /* autoResult_t, autoStatusMsg_t */
#include "auto_glue_core.h"     /* autoSettingId_t and the settings table */
#include "auto_mailbox.h"       /* autoCmdResult_t, autoMbCancel_t, AUTO_DETAIL_TAKEN_UNKNOWN */

/* ---- timing of the glue (docs/autonomy-a2b.md section 2.3) ------------- */
#define AUTONOMY_IDLE_STEP_EVERY   100   /* IDLE and quiescent: a full FSM step every 100th loop (10 Hz) */
#define AUTONOMY_SLOW_EVERY        10    /* camera, battery minimum, heap, lock inputs every 10th loop (100 Hz) */
#define AUTONOMY_PUBLISH_EVERY     100   /* active: status snapshot (autoStatusFill, autoTripBits) every 100th
                                          * loop (10 Hz, the push rate) and at each transition */
#define AUTONOMY_CMD_WAIT_MS       100   /* link: a command not taken by then is withdrawn (ACK BUSY) */
#define AUTONOMY_CMD_WAIT_HARD_MS  200   /* link: a taken command's result is awaited this long, then ACK BUSY
                                          * with detail AUTO_DETAIL_TAKEN_UNKNOWN */

/* ---- init ---------------------------------------------------------------- */

/* Once, from stabilizerInit() after camVzGlueInit(), before the stabilizer
 * task is created: loads the settings from NVS (defaults for absent or
 * out-of-range keys), decides the reset-lock from esp_reset_reason() and the
 * RTC record, rewrites the record for this boot, and starts in IDLE.
 * Reset-lock default: the flag is `static volatile bool resetLocked = true`
 * (.data), cleared here only by autoRtcBootLocked() returning false (a
 * battery power-on), and otherwise only by autonomyUsbUnlock() (the USB
 * console); a skipped or reordered init leaves FLIGHT and BENCH locked. */
void autonomyInit(void);

/* ---- stabilizer task, every loop ----------------------------------------- */

/* Section 2 step 2. nowMs: T2M(xTaskGetTickCount()); loopGapUs: this loop's
 * start minus the previous one's (0 on the first loop). In IDLE (and not in
 * the 600 ms release hold) it returns without touching the setpoint; in any
 * other state it overwrites the whole setpoint (section 8.1: roll and pitch
 * exactly 0.0f, modeAbs, yaw absolute at the captured heading) with
 * thrust = min(action thrust, thrustMax). IRAM: its quiescent-IDLE path costs
 * well under 1 us; the full step it runs (stepFull, autoFsmStep, vzStep or
 * vzSimStep) is IRAM too, and only commands, transitions and once-a-sortie
 * calls reach flash code (docs/autonomy-a2b.md 2.6). */
void autonomyArbitrate(setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, uint32_t nowMs,
                       uint32_t loopGapUs);

/* Section 2 step 5, the second barrier: the stabilizer clamps control.thrust
 * with autoClampThrust(control.thrust, autonomyThrustMax()) right before the
 * emergency-stop test and powerDistribution(). autoBarrierMax of the last
 * step: 65535 with action NONE (manual flight untouched); 0 with ZERO or CUT
 * in every mode (ARMED, pre-spin, release hold, landed, KILLED); FLIGHT
 * VZ_THRUST_ABS_MAX (58000), DRY 0, BENCH the bench cap with
 * VERT_CTRL/VERT_LAND. IRAM, a single load. */
float autonomyThrustMax(void);

/* Section 2 step 8: the thrust the motors really got this loop (0 after
 * powerStop(), an emergency stop or with every duty at zero). The controller
 * reads it as thrustApplied on the next step in FLIGHT and BENCH-imu. IRAM. */
void autonomyNoteApplied(float thrust);

/* ---- phone hook (CRTP RX task) ------------------------------------------ */

/* Section 8.2: every valid RPYT packet, on the bounded values, after the
 * link statistics (crtp_commander_rpyt.c). Stamps the FSM's clock itself.
 * Input = rawThrust >= 1000 or |roll| or |pitch| >= 2 deg (yaw ignored); a
 * roll push >= 10 deg released below 2 deg is the consent gesture; the last
 * rawThrust < 1000 is "thrust zero". Lock-free; never changes the decoded
 * setpoint. */
void autonomyNoteAppPacket(float rollDeg, float pitchDeg, uint16_t rawThrust);

/* ---- link task (auto_link.c) -------------------------------------------- */

/* The mailbox (auto_mailbox.h) under the glue's spinlock. The link is
 * asynchronous (section 3.1): it posts, keeps {ticket, seq, type, t0} as its
 * one pending command and goes on receiving and serving packets (KILL, HB,
 * STATUS, photos); each loop it polls autonomyTakeResult. It never waits
 * inside a call and never retries a post in a loop.
 *
 * Post: false = the slot still holds an earlier command the FSM took and has
 * not answered (the stabilizer is stalled): ACK BUSY at once, uncached. A
 * second command while one is pending: ACK BUSY (detail = the pending seq),
 * uncached, without posting. Never pass AUTO_CMD_KILL (autonomyKillFromLink). */
bool autonomyPostCommand(const autoCmd_t *cmd, uint32_t *ticket);
/* true once the FSM published this ticket's result (normally one stabilizer
 * loop after the post): copies it. ACK byte 6 is out->state. */
bool autonomyTakeResult(uint32_t ticket, autoCmdResult_t *out);
/* At AUTONOMY_CMD_WAIT_MS: CANCELLED (never processed: ACK BUSY, detail =
 * autonomyState(), uncached), TAKEN (keep polling to AUTONOMY_CMD_WAIT_HARD_MS,
 * then ACK BUSY with AUTO_DETAIL_TAKEN_UNKNOWN, uncached: it may have been
 * acted on), DONE (read the result now) or UNKNOWN (treat as TAKEN-unknown). */
autoMbCancel_t autonomyCancelCommand(uint32_t ticket);

/* Mac KILL, M9 (section 7.1): motorsEmergencyStopNoWait() if any motor duty
 * is nonzero, then killCount++ (seen by the FSM on its next step in any
 * state: KILLED, or the 2 s kill latch in IDLE). Any task, never blocks,
 * never takes the driver mutex, never touches the mailbox. */
void autonomyKillFromLink(void);
/* A HELLO opened a new session: sessionValid from now on, sessionChanged on
 * the FSM's next step (a session change while active lands: the Mac tool
 * never re-HELLOs while a sortie is active). */
void autonomyNoteSession(void);
/* A valid HB in the current session (hbAgeMs restarts). */
void autonomyNoteHeartbeat(void);

/* Photo sets the FSM starts (captureStart: at STABLE 1 s, or an accepted
 * PHOTO while waiting). The link takes each request once, starts the set
 * (releasing an A1 set still held; an autonomy set still held makes the link
 * refuse the PHOTO before posting it, section 3.4) and reports its progress
 * with the same reqId: AUTO_CAP_BUSY when started, AUTO_CAP_DONE when every
 * frame was copied, AUTO_CAP_FAILED if the camera was not running or a frame
 * is missing. Reports for an older reqId are ignored. */
bool autonomyTakeCaptureRequest(uint32_t *reqId, uint8_t *n, uint16_t *gapMs);
void autonomyNoteCapture(uint32_t reqId, autoCapture_t progress, uint16_t photoId);
/* The camera is held out of its take-off standby for this sortie
 * (out.camNoStandby applied): a capture need not wait for the take-off
 * window to end (the photo push still does). */
bool autonomyCameraAwake(void);
/* LAND after contact (a bump or stall the controller saw, its touchdown, or
 * |acc| > AUTO_TD_CONTACT_G), latched at the stabilizer rate until the sortie
 * leaves LAND, and the whole of a LAND COLLISION or of a LAND begun in a
 * contact's window from its first step on (autoFsmOut_t.collided; a blind
 * LAND long after a contact is not blocked before its touchdown): the
 * camera drift hold commands no lean
 * (controller_pid.c zeroes the applied correction at once, camera_flow.c
 * treats the drone as not airborne, which disengages the hold and clears its
 * integrals). IRAM, a single load; any task. */
bool autonomyHoldBlocked(void);
/* Take-off integrator hold (user decision 2026-10-13, evening; autonomous
 * sorties only): true from the sortie's vzReset until the vertical
 * controller latches its lift-off (vzOutput_t.liftedOff); false in IDLE
 * (every manual flight) and from the lift-off step on. Bounded for a
 * lift-off the controller never latches: false too once its RAMP has
 * reached the plateau (vzRampAtTop(); every logged lift-off came 0.39 s or
 * more before it) and in any phase after RAMP (the abort LAND, DONE, CUT),
 * so an undetected lift-off flies on P and D alone for at most the rest of
 * the ramp. While it is true controller_pid.c keeps the
 * roll/pitch attitude and rate integrators at zero, as it does below its
 * 26000 ground thrust, so the RAMP's ~0.4-0.8 s above 26000 with the drone
 * resting tilted on the floor winds none of them up (the kick at lift-off,
 * docs/autonomy-a2b.md 2.4); P, D and the torque-trim feed-forward act as
 * before. Stabilizer task only: it reads the state autonomyArbitrate left
 * this loop (the controller runs after it); flash code (its caller is). */
bool autonomyTakeoffIntegratorHold(void);

/* Interactive hover (auto_motion.h, docs/autonomy-a2b.md section 14): the
 * camera drift hold's moving reference, as the state machine's last full
 * step left it (camera_hold.h camHoldInput_t ref*). All 0 (epoch kept)
 * without a motion. snapEpoch: the latest epoch that began with a snap.
 * Camera task, per frame; IRAM, a short spinlock. */
typedef struct {
  uint32_t epoch, snapEpoch;
  float refLat, refFwd;     /* x / D and ln(D0 / D) since the epoch start */
  float rateLat, rateFwd;   /* 1/s */
  float ffRoll, ffPitch;    /* deg */
  float dLowM;              /* the latest MOVE's scene distance lower bound at its acceptance, m (autoMvOut_t dLowM):
                               the gain scale during a MOVE (camera_hold.h camHoldGainMove; 2026-10-14) */
} autonomyHoldRef_t;
void autonomyGetHoldRef(autonomyHoldRef_t *out);
/* mvspeed (auto_motion.h "fast moves", 2026-10-16): the sortie's latched motion speed step, %; 100 in IDLE (manual
 * flights, the hold as before). Camera task, per frame (the hold's move lean limit and slew, camera_hold.h
 * camHoldMoveLimits). Flash, two loads. */
uint16_t autonomyMvSpeedPct(void);
/* mvfix (auto_motion.h "move fixes", 2026-10-17): the sortie's latched AUTO_MVFIX_* bits, 0 in IDLE (manual flights:
 * the hold as before). Camera task, per frame (camera_hold.h camHoldGainMoveFix, camHoldInput_t ffBrakeScale). Flash. */
uint8_t autonomyMvFixBits(void);
/* The brake lean (deg, controller convention) of a fast MOVE stopped in HOVER (mvspeed > 100, a stopped MOVE's brake
 * running: no motion active, a feedforward lean in the state machine's outputs): controller_pid.c applies it when the
 * camera hold is stale (no frames for CAM_HOLD_STALE_US) instead of fading to 0, so a camera stall at speed still
 * brakes. false (outputs untouched) otherwise, always at 100. Stabilizer task only (it reads the outputs the glue's
 * step left this loop); flash. */
bool autonomyBrakeLean(float *rollDeg, float *pitchDeg);

/* ---- any task ----------------------------------------------------------- */

bool autonomyActive(void);            /* any state but IDLE */
autoState_t autonomyState(void);      /* STATUS byte 7; ACK byte 6 of a BUSY */
uint16_t autonomySortieEvents(void);  /* the latest sortie's reported events (AUTO_STATUS evCount, less the
                                       * information-only HOLD_LIMIT ones: autoFsmTrimEvents; 0 from its ARM) */
bool autonomyResetLocked(void);       /* section 6, decided at boot (true until autonomyInit decides);
                                       * cleared later only by autonomyUsbUnlock */
/* Increments at every FSM transition, after the snapshot of that step is
 * stored (the link pushes AUTO_STATUS on a change). */
uint32_t autonomyStatusSeq(void);
/* Snapshot in wire units (auto_proto.h); bootNonce is left 0 (the link owns
 * it). Returns the statusSeq this snapshot belongs to, read under the same
 * lock, so the link never sends an old snapshot for a new seq. */
uint32_t autonomyGetStatus(autoStatusMsg_t *out);

/* The 100 Hz recorder columns (flight_recorder.h), all 0 in IDLE outside a sortie. */
typedef struct {
  uint8_t autoState;       /* autoState_t */
  uint8_t autoReason;      /* autoReason_t of the latest transition */
  uint8_t vzPhase;         /* vzPhase_t, 0 (VZ_IDLE) outside a sortie */
  int16_t vzHmm;           /* vzOutput_t.h, mm */
  int16_t vzVmms;          /* vzOutput_t.v, mm/s */
  uint16_t vzThrust;       /* vzOutput_t.thrust (the controller's command, before the mode clamp) */
  uint8_t autoFlags;       /* AUTO_STF_* (auto_proto.h) */
  uint8_t mvState;         /* interactive hover: autoMvState_t | autoMvKind_t << 4 */
  int16_t mvRefLat;        /* the hold's lateral reference, 1e-4 (x / D) */
  int16_t mvRefFwd;        /* its forward reference, 1e-4 (ln(D0 / D)) */
  int16_t mvVRef;          /* the vertical speed reference, mm/s */
  int16_t mvYawOff;        /* the heading offset, 0.01 deg, + clockwise */
} autoRecFields_t;
void autonomyRecordFields(autoRecFields_t *out);

/* ---- flash writers (imu_level_cal_store.c) -------------------------------- */

/* Every NVS write, commit and erase in the firmware goes through
 * imu_level_cal_store.c, which brackets it with these. Begin: true and the
 * write may start only in IDLE with no ARM in the mailbox (auto_mailbox.h:
 * an ARM taken during a write is refused by G1); false: the store returns
 * ESP_ERR_INVALID_STATE without touching flash, and the caller reports
 * "busy: autonomy active". Every true is paired with one End. Any task,
 * never blocks. */
bool autonomyFlashBegin(void);
void autonomyFlashEnd(void);

/* ---- settings (USB console only; never over Wi-Fi) ----------------------- */

/* The keys, types, ranges and defaults are auto_glue_core.h's table
 * (autoSettingId_t). Values are in the table's units (mV, ms, thrust x V,
 * 0.1 mg, x1000). Get returns the RAM copy (what the next ARM uses). */
int32_t autonomySettingGet(autoSettingId_t id);
/* IDLE only (an NVS write stops both cores; the store's flash interlock
 * enforces it): range-checks, writes, commits, reads back, then updates the
 * RAM copy used from the next ARM. false with *why = "busy: autonomy active"
 * | "range" | "nvs" otherwise. */
bool autonomySettingSet(autoSettingId_t id, int32_t value, const char **why);
/* Erases the key (default from the next ARM); IDLE only, as Set. */
bool autonomySettingReset(autoSettingId_t id, const char **why);

/* ---- reset-lock USB unlock (USB console only; never over Wi-Fi) ---------- */

/* The USB console's 'auto unlock' (user decision 2026-10-12, evening): a
 * command typed on the physical USB cable is the deliberate act after an
 * unexpected reset that a battery power-on used to be the only form of. It
 * clears the reset-lock as a battery power-on does (resetLocked false: G1
 * and the AUTO_STF_RESET_LOCKED flag from the next step) and marks the RTC
 * record AUTO_RTC_USB_UNLOCKED; the reset reason and the boot rule stay, so
 * the next reset locks again. Only in IDLE outside the release hold, with
 * every motor duty 0, and inside the flash interlock (no ARM in the
 * mailbox; an ARM taken meanwhile is refused by G1). false with *why =
 * "busy: autonomy active" | "a motor output is nonzero", nothing changed.
 * true with *wasLocked = whether it was locked (an unlocked boot: nothing
 * to do). It touches no other lock: the restart lock (plock), the motor
 * driver's STOP latch, the motor watchdog and the emergency stop stay as
 * they are. usb_diag.c is the only caller: neither the link task nor any
 * CRTP or Wi-Fi path may call it. */
bool autonomyUsbUnlock(bool *wasLocked, const char **why);

/* ---- low-priority task (takeoff_aux, every 10 ms) ------------------------ */

/* Samples the internal heap for G9 on every call (heap_caps_get_free_size()
 * is flash code, kept out of the stabilizer loop), with its time: the guards
 * (ARM, TAKEOFF) take a reading older than 200 ms, or none, as no heap, so
 * without these calls every ARM is refused HEAP. Then the deferred NVS
 * work: saves a learned vzK (vzLearnK after a qualifying FLIGHT sortie) only
 * in IDLE, with every motor duty at 0 and the drone resting on the ground
 * (section 8.7), through the flash interlock. */
void autonomyPoll(void);

/* ---- bench and diagnostics ----------------------------------------------- */

typedef enum {
  AUTO_BENCH_SRC_SIM = 0,  /* BENCH: the vertical controller flies the sim plant (default) */
  AUTO_BENCH_SRC_IMU = 1,  /* BENCH: the real IMU and battery (hand-lift tests) */
} autoBenchSrc_t;
/* RAM only, IDLE only (false otherwise). */
bool autonomySetBenchSource(autoBenchSrc_t src);
autoBenchSrc_t autonomyBenchSource(void);

typedef struct {
  uint32_t loops;          /* autonomyArbitrate calls since the last reset */
  uint32_t fullSteps;      /* of which full steps (FSM run) */
  uint32_t idleMaxUs, idleAvgUs10;      /* full IDLE steps: max and mean x10, us (esp_timer) */
  uint32_t activeMaxUs, activeAvgUs10;  /* active steps (FSM + controller + override) */
  uint32_t fastMaxUs;      /* quiescent IDLE path, max us (timed on every 16th) */
  uint32_t cmdLatencyMaxMs;             /* post to FSM result, max */
  uint32_t activeLoops, activeMs;       /* loops and ms spent active since the reset: loops/s while active */
  uint32_t stabStackFreeMin;            /* uxTaskGetStackHighWaterMark of the stabilizer task, bytes (lifetime) */
  uint32_t flashRefused, armsBlocked;   /* flash interlock: writes refused, ARMs refused during a write */
  uint32_t shiftRefused;   /* SET_HEIGHT shifts the controller refused (vzShiftHeight() false: never expected) */
} autonomyProfile_t;
void autonomyGetProfile(autonomyProfile_t *out);
void autonomyProfileReset(void);

/* The reset-lock record (section 6) as stored in RTC memory: this boot and
 * the previous one, and the last AUTO_RTC_TRANSITIONS transitions. */
typedef struct {
  bool locked;             /* this boot, now (false after a USB unlock) */
  bool recordWasValid;     /* an intact record survived the reset */
  int resetReason;         /* esp_reset_reason_t of this boot (a USB unlock keeps it) */
  uint8_t prevState, prevReason, prevMode, prevLocked;   /* prevLocked: AUTO_RTC_*LOCKED (2: USB unlock) */
  uint32_t prevTransitions, transitions;
  uint8_t n;               /* entries in trans[], oldest first */
  struct {
    uint32_t ms;
    uint8_t from, to, reason, vzReason;
  } trans[8];
} autonomyRtcView_t;
void autonomyGetRtc(autonomyRtcView_t *out);

#endif /* AUTONOMY_H_ */
