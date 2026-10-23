/*
 * ESP-Drone Firmware
 *
 * auto_glue_core.c - The pure parts of the autonomy glue that run at boot, on
 * a transition, at 10 Hz or from the console (auto_glue_core.h,
 * docs/autonomy-a2b.md): the reset-lock RTC record, the settings table, the
 * AUTO_STATUS fill and the trip bits. Host-tested by
 * tools/tests/test_auto_glue_core.c.
 */
#include "auto_glue_core.h"
#include "cam_vz_obs.h"

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

/* ---- reset-lock record --------------------------------------------------- */

#define RTC_CRC_OFF offsetof(autoRtcRecord_t, version)   /* the CRC covers every byte after it */

static uint32_t rtcCrc(const autoRtcRecord_t *r)
{
  return autoProtoCrc32((const uint8_t *)r + RTC_CRC_OFF, sizeof(*r) - RTC_CRC_OFF);
}

bool autoRtcValid(const autoRtcRecord_t *r)
{
  return r->magic == AUTO_RTC_MAGIC && r->version == AUTO_RTC_VERSION && r->crc == rtcCrc(r);
}

bool autoRtcBootLocked(bool powerOnReset, const autoRtcRecord_t *r)
{
  /* Only a battery power-on loses RTC memory: its magic is absent. */
  return !(powerOnReset && r->magic != AUTO_RTC_MAGIC);
}

void autoRtcStartBoot(autoRtcRecord_t *r, bool locked)
{
  const bool valid = autoRtcValid(r);
  const uint32_t boots = valid ? r->boots + 1u : 1u;
  const uint8_t ps = valid ? r->lastState : 0, pr = valid ? r->lastReason : 0;
  const uint8_t pm = valid ? r->lastMode : 0, pl = valid ? r->locked : 0;
  const uint32_t pt = valid ? r->transitions : 0;
  memset(r, 0, sizeof(*r));
  r->magic = AUTO_RTC_MAGIC;
  r->version = AUTO_RTC_VERSION;
  r->boots = boots == 0 ? 1u : boots;
  r->lastState = AUTO_IDLE;
  r->lastReason = AUTO_RS_NONE;
  r->locked = locked ? AUTO_RTC_LOCKED : AUTO_RTC_UNLOCKED;
  r->prevState = ps;
  r->prevReason = pr;
  r->prevMode = pm;
  r->prevLocked = pl;
  r->prevTransitions = pt;
  r->crc = rtcCrc(r);
}

void autoRtcNoteTransition(autoRtcRecord_t *r, uint32_t ms, autoState_t from, autoState_t to, autoReason_t reason,
                           uint8_t vzReason, autoMode_t mode)
{
  autoRtcTransition_t *t = &r->trans[r->transitions % AUTO_RTC_TRANSITIONS];
  t->ms = ms;
  t->from = (uint8_t)from;
  t->to = (uint8_t)to;
  t->reason = (uint8_t)reason;
  t->vzReason = vzReason;
  r->transitions++;
  r->lastState = (uint8_t)to;
  r->lastReason = (uint8_t)reason;
  r->lastMode = (uint8_t)mode;
  r->crc = rtcCrc(r);
}

bool autoRtcNoteUsbUnlock(autoRtcRecord_t *r)
{
  if (!autoRtcValid(r) || r->locked != AUTO_RTC_LOCKED) {
    return false;
  }
  r->locked = AUTO_RTC_USB_UNLOCKED;
  r->crc = rtcCrc(r);
  return true;
}

/* ---- settings ------------------------------------------------------------ */

#define MV(v) ((int32_t)((v) * 1000.0f + 0.5f))

static const autoSettingInfo_t settings[AUTO_SET_COUNT] = {
  [AUTO_SET_AUTOON] = {"autoon", AUTO_NVS_U8, 0, 1, 0, "", 0},
  [AUTO_SET_APPREQ] = {"appreq", AUTO_NVS_U8, 0, 1, 1, "", 0},
  [AUTO_SET_AUTOGEST] = {"autogest", AUTO_NVS_U8, 0, 1, 1, "", 0},
  [AUTO_SET_VMIN] = {"vmin", AUTO_NVS_U16, MV(AUTO_VMIN_LO), MV(AUTO_VMIN_HI), MV(AUTO_VMIN_DEFAULT), "mV", 0},
  [AUTO_SET_AUTOAIR] = {"autoair", AUTO_NVS_U16, AUTO_AIR_MAX_MIN_MS, AUTO_AIR_MAX_HI_MS, AUTO_AIR_MAX_DEFAULT_MS,
                        "ms", 0},
  /* u32 since 2026-10-14 (waits up to 120 s); the u16 item a drone stored before is still read (legacyType) */
  [AUTO_SET_AUTOWAIT] = {"autowait", AUTO_NVS_U32, 0, AUTO_HOVER_WAIT_MAX_MS, AUTO_HOVER_WAIT_DEFAULT_MS, "ms",
                         AUTO_NVS_U16},
  [AUTO_SET_AUTOACCEL] = {"autoaccel", AUTO_NVS_U16, 0, AUTO_HOVER_ACCEL_MAX_MS, AUTO_HOVER_ACCEL_DEFAULT_MS, "ms",
                          0},
  [AUTO_SET_VZK] = {"vzK", AUTO_NVS_U32, (int32_t)VZ_K_MIN, (int32_t)VZ_K_MAX, (int32_t)VZ_K_DEFAULT, "Kv", 0},
  /* 0.1 mg: -0.0023 g is -23 */
  [AUTO_SET_VZBAIR] = {"vzBair", AUTO_NVS_I16, -(int32_t)(VZ_BAIR_MAX_G * 10000.0f + 0.5f),
                       (int32_t)(VZ_BAIR_MAX_G * 10000.0f + 0.5f), (int32_t)(VZ_BAIR_DEFAULT_G * 10000.0f - 0.5f),
                       "0.1mg", 0},
  /* x1000; vzConfigValid's hard limit is 1.0..2.0, vzDefaultConfig's value 1.65 */
  [AUTO_SET_VZCAPLO] = {"vzcaplo", AUTO_NVS_U16, 1000, 2000, 1650, "x1000", 0},
  /* the camera's pixel format at the next boot (2026-10-15): 1 colour photos, 0 grey (the frames of before, the
   * default: colour is opt-in until bench N0-N3 pass with the Mac tool of this tree, docs/autonomy-bench.md N) */
  [AUTO_SET_PHOTOCOL] = {"photocol", AUTO_NVS_U8, 0, 1, 0, "", 0},
  /* the camera height observer's late offset noise at the next boot (2026-10-15, R1 of the height study): 1 off */
  [AUTO_SET_CVQLATE] = {"cvqlate", AUTO_NVS_U8, 1, (int32_t)CAM_VZ_QLATE_MULT_MAX, 1, "x", 0},
  [AUTO_SET_CVQLATES] = {"cvqlates", AUTO_NVS_U8, (int32_t)CAM_VZ_QLATE_S_MIN, (int32_t)CAM_VZ_QLATE_S_MAX, 40, "s", 0},
  /* its offset drift rate T2 at the next boot (2026-10-16, the late-sink fix): kR x 100, 0 off; the prior in ug/s */
  [AUTO_SET_CVT2K] = {"cvt2k", AUTO_NVS_U8, 0, (int32_t)(CAM_VZ_T2_KR_MAX * 100.0f + 0.5f), 0, "x0.01/s", 0},
  [AUTO_SET_CVT2R] = {"cvt2r", AUTO_NVS_I16, -(int32_t)(CAM_VZ_T2_BOUND_MGS * 1000.0f + 0.5f),
                      (int32_t)(CAM_VZ_T2_BOUND_MGS * 1000.0f + 0.5f), (int32_t)(CAM_VZ_T2_PRIOR_MGS * 1000.0f + 0.5f),
                      "ug/s", 0},
  /* the motion speed step (2026-10-16, auto_motion.h "fast moves"): read at the next ARM, no reboot */
  [AUTO_SET_MVSPEED] = {"mvspeed", AUTO_NVS_U16, AUTO_MV_SPEED_MIN_PCT, AUTO_MV_SPEED_MAX_PCT, AUTO_MV_SPEED_DEFAULT_PCT,
                        "%", 0},
  /* the guard margins (2026-10-17, auto_motion.h "guard margins"): read at the next ARM, no reboot. Since 2026-10-18
   * ("guards off") the first four take 0 (off) as well: 0 or their range (1..min - 1 refused, autoSettingInRange) */
  [AUTO_SET_MVFLOOR] = {"mvfloor", AUTO_NVS_U8, 0, AUTO_MV_MG_FLOOR_MAX_CM, AUTO_MOVE_MIN_FRONT_CM, "cm", 0},
  [AUTO_SET_MVFSTOP] = {"mvfstop", AUTO_NVS_U8, 0, AUTO_MV_MG_STOP_MAX_CM, AUTO_MV_FRONT_STOP_CM, "cm", 0},
  [AUTO_SET_MVPROGM] = {"mvprogm", AUTO_NVS_U8, 0, AUTO_MV_MG_PROG_MAX_CM, AUTO_MV_PROG_MARGIN_CM, "cm", 0},
  [AUTO_SET_MVPROGR] = {"mvprogr", AUTO_NVS_U8, 0, AUTO_MV_MG_PCT_MAX, AUTO_MV_PROG_MARGIN_PCT, "%", 0},
  [AUTO_SET_MVLATM] = {"mvlatm", AUTO_NVS_U8, AUTO_MV_MG_LAT_MIN_CM, AUTO_MV_MG_LAT_MAX_CM, AUTO_MV_LAT_MARGIN_CM,
                       "cm", 0},
  [AUTO_SET_MVNEAR] = {"mvnear", AUTO_NVS_U8, 0, AUTO_MV_MG_NEAR_MAX_CM, AUTO_MV_NEAR_SCENE_CM, "cm", 0},
  /* the move fixes (2026-10-17, auto_motion.h "move fixes"): read at the next ARM, no reboot */
  [AUTO_SET_MVFIX] = {"mvfix", AUTO_NVS_U8, 0, AUTO_MVFIX_MAX, AUTO_MVFIX_DEFAULT, "bits", 0},
  /* the fast steps' acceleration and settle (2026-10-17, auto_motion.h "the fast steps' acceleration and settle as
   * settings"): read at the next ARM, no reboot; above 100 % only */
  [AUTO_SET_MVACC] = {"mvacc", AUTO_NVS_U8, AUTO_MV_ACC_MIN_CM, AUTO_MV_ACC_MAX_CM, AUTO_MV_ACC_DEFAULT_CM, "cm/s^2", 0},
  [AUTO_SET_MVSETTLE] = {"mvsettle", AUTO_NVS_U16, AUTO_MV_SETTLE_FLOOR_MIN_MS, AUTO_MV_SETTLE_FLOOR_MAX_MS,
                         AUTO_MV_FAST_SETTLE_MIN_MS, "ms", 0},
  /* the camera height observer's healthy offset speed-up at the next boot (2026-10-18, the height estimator study):
   * hbMult x 10, 0 off; 1..9 are refused too (autoSettingInRange) */
  [AUTO_SET_CVHB] = {"cvhb", AUTO_NVS_U8, 0, (int32_t)(CAM_VZ_HB_MULT_MAX * 10.0f + 0.5f), 0, "x0.1", 0},
  /* the guards down to off (2026-10-18, auto_motion.h "guards off"): read at the next ARM, no reboot */
  [AUTO_SET_MVKLOW] = {"mvklow", AUTO_NVS_U8, AUTO_MV_MG_KLOW_MIN_PCT, AUTO_MV_MG_KLOW_MAX_PCT, AUTO_MV_K_LOW_PCT, "%", 0},
  [AUTO_SET_MVLAT] = {"mvlat", AUTO_NVS_U8, 0, AUTO_MV_MG_LAT_REL_MAX_PCT, AUTO_MV_LAT_D_REL_PCT, "%", 0},
  [AUTO_SET_MVWMAX] = {"mvwmax", AUTO_NVS_U8, AUTO_MV_MG_WFWD_MIN_PCT, AUTO_MV_MG_WFWD_MAX_PCT, AUTO_MV_W_FWD_PCT,
                       "x0.01/s", 0},
  /* the VIEW_FAST timer (2026-10-22, auto_motion.h "fast moves"): read at the next ARM, no reboot; 0 the profile's,
   * 1..AUTO_MV_FAST_VIEW_DEB_MS - 1 refused (autoSettingInRange: the timer only ever gets longer) */
  [AUTO_SET_MVVIEW] = {"mvview", AUTO_NVS_U16, 0, AUTO_MV_VIEW_DEB_MAX_MS, 0, "ms", 0},
  /* the vertical controller's EST_RANGE guard (2026-10-22, user request of 2026-10-18; vert_ctrl.h): read at the next
   * ARM, no reboot */
  [AUTO_SET_VZEST] = {"vzest", AUTO_NVS_U8, 0, 1, 1, "", 0},
  [AUTO_SET_VZESTMS] = {"vzestms", AUTO_NVS_U16, 0, (int32_t)(VZ_EST_GRACE_MAX_S * 1000.0f + 0.5f), 0, "ms", 0},
  /* the hold's scene gate residual limit (2026-10-22, camera_hold.h CAM_HOLD_SCENE_RES_*): read at the next ARM, no
   * reboot; 0 off, 1..9 refused (autoSettingInRange) */
  [AUTO_SET_HOLDRES] = {"holdres", AUTO_NVS_U8, 0, CAM_HOLD_SCENE_RES_X10_MAX, CAM_HOLD_SCENE_RES_X10_DEFAULT, "x0.1",
                        0},
  /* the left slide of the forward MOVEs (2026-10-22): read at the next ARM, no reboot; the defaults are the hold's
   * lateral gain and the motions' feedforward as before */
  [AUTO_SET_MVLATK] = {"mvlatk", AUTO_NVS_U16, 100, (int32_t)(CAM_HOLD_LAT_GAIN_MAX * 100.0f + 0.5f), 100, "%", 0},
  [AUTO_SET_MVFFR] = {"mvffr", AUTO_NVS_I16, AUTO_MV_FFR_MIN_DDEG, AUTO_MV_FFR_MAX_DDEG, 0, "x0.1deg/mps", 0},
};
/* every stored value in range is a valid observer config (camVzObsConfigValid), so cam_vz_glue.c never loses the
 * rest of the config for one */
_Static_assert(CAM_VZ_QLATE_MULT_MAX == 5.0f && CAM_VZ_QLATE_S_MIN == 20.0f && CAM_VZ_QLATE_S_MAX == 120.0f,
               "cvqlate 1..5, cvqlates 20..120 s: whole numbers inside the observer's ranges");
_Static_assert(CAM_VZ_T2_KR_MAX == 2.0f && CAM_VZ_T2_BOUND_MGS == 0.35f && CAM_VZ_T2_PRIOR_MGS == 0.124f,
               "cvt2k 0..200 (kR 0..2), cvt2r -350..350 ug/s (the default bound), default 124 (the default prior)");
_Static_assert(CAM_VZ_HB_MULT_MIN == 1.0f && CAM_VZ_HB_MULT_MAX == 8.0f, "cvhb 0 or 10..80 (hbMult x 10)");

const autoSettingInfo_t *autoSettingInfo(autoSettingId_t id)
{
  return ((unsigned)id < AUTO_SET_COUNT) ? &settings[id] : NULL;
}

int autoSettingFind(const char *name)
{
  for (int i = 0; name != NULL && i < AUTO_SET_COUNT; i++) {
    if (strcmp(name, settings[i].name) == 0) {
      return i;
    }
  }
  return -1;
}

int32_t autoSettingOnMin(autoSettingId_t id)
{
  switch (id) {
    case AUTO_SET_CVHB: return (int32_t)(CAM_VZ_HB_MULT_MIN * 10.0f + 0.5f);   /* a multiplier under 1 would slow the
                                                                                  offset */
    case AUTO_SET_MVFLOOR: return AUTO_MV_MG_FLOOR_MIN_CM;   /* (the guards' ranges of 2026-10-17 below their off) */
    case AUTO_SET_MVFSTOP: return AUTO_MV_MG_STOP_MIN_CM;
    case AUTO_SET_MVPROGM: return AUTO_MV_MG_PROG_MIN_CM;
    case AUTO_SET_MVPROGR: return AUTO_MV_MG_PCT_MIN;
    case AUTO_SET_MVVIEW: return AUTO_MV_FAST_VIEW_DEB_MS;   /* 0 the profile's timer; never shorter than it */
    case AUTO_SET_HOLDRES: return CAM_HOLD_SCENE_RES_X10_MIN;   /* 0 off; under 1.0 grey level refuses every frame */
    default: return 0;
  }
}

bool autoSettingInRange(autoSettingId_t id, int32_t value)
{
  const autoSettingInfo_t *s = autoSettingInfo(id);
  if (value > 0 && value < autoSettingOnMin(id)) {
    return false;   /* 0 (off) or the lowest "on" value and up */
  }
  return s != NULL && value >= s->min && value <= s->max;
}

/* ---- AUTO_STATUS ----------------------------------------------------------- */

static uint16_t satU16(float v);
static int16_t satI16(float v);

/* An age from the FSM's ms clock, saturated at 65535 (0 for a time not yet reached). */
static uint16_t ageMs16(uint32_t now, uint32_t then)
{
  const uint32_t d = now - then;
  return (int32_t)d < 0 ? 0u : (d > 65535u ? 65535u : (uint16_t)d);
}

/* AUTO_STATUS bytes 96-135, hold and report (docs/autonomy-a2b.md 22): the state machine's report, and the bits the
 * controller adds. Flash, out of line: 10 Hz and on transitions and events, never every loop (its IRAM is the
 * per-loop code's). */
static __attribute__((noinline)) void autoStatusFillReport(autoStatusMsg_t *m, const autoFsmOut_t *o,
                                                          const vzOutput_t *vz, const autoStatusExtra_t *x)
{
  const autoFsmReport_t *r = x->rep;
  const uint32_t now = x->upMs;
  m->airDs = (uint16_t)(o->airMs / 100u > 65535u ? 65535u : o->airMs / 100u);
  const float kRel = x->kPrior > 0.0f ? vz->khat / x->kPrior : 0.0f;
  m->khatPm = satU16(kRel * 1000.0f);
  m->capQ = satU16(vz->capQ);
  uint8_t bits = 0;
  if (x->vzActive) {
    /* Khat at its upper clamp (0.995 x 1.12) or at / below khatMinRel 0.88 (vert_ctrl.c's defaults) */
    bits |= (vz->khat > 0.0f && (kRel >= VZ_HI_CLAMP_FRAC * 1.12f || kRel <= 0.88f)) ? AUTO_HB_K_CLAMP : 0u;
    bits |= vz->atCap ? AUTO_HB_CAPPED : 0u;
    bits |= (vz->phase == VZ_HOLD && !vz->camHealthy) ? AUTO_HB_OPEN_LOOP : 0u;
  }
  if (r == NULL) {
    m->holdBits = bits;
    m->sinceCmdDs = 65535u;
    return;
  }
  m->evCode = r->evCode[0];
  m->evDetail = r->evDetail[0];
  m->evCount = r->evCount;
  m->evAgeMs = r->evCode[0] != 0 ? ageMs16(now, r->evMs[0]) : 0u;
  m->ev1Code = r->evCode[1];
  m->ev1Detail = r->evDetail[1];
  m->ev1AgeMs = r->evCode[1] != 0 ? ageMs16(now, r->evMs[1]) : 0u;
  m->ev2Code = r->evCode[2];
  m->ev2Detail = r->evDetail[2];
  m->ev2AgeMs = r->evCode[2] != 0 ? ageMs16(now, r->evMs[2]) : 0u;
  m->holdBits = (uint8_t)(bits | r->holdBits);
  m->camBits = r->camBits;
  if (r->holdBits & AUTO_HB_CAM_LOST) {
    m->camOffMs = ageMs16(now, r->camOutMs);
    m->sinkInMs = ageMs16(r->sinkAtMs, now);
  }
  m->peakAccCg = satU16(r->peakAccG * 100.0f);
  m->peakRateDps = satU16(r->peakRateDps);
  m->contactAxCg = satI16(r->contactAxG * 100.0f);
  m->contactAyCg = satI16(r->contactAyG * 100.0f);
  m->contactYawCdeg = satI16(r->contactYawDeg * 100.0f);
  m->sinceCmdDs = r->lastCmdAny ? (uint16_t)((now - r->lastCmdMs) / 100u > 65534u ? 65534u : (now - r->lastCmdMs) / 100u)
                                : 65535u;
  m->contacts = r->contacts;
  m->loopGaps = r->loopGaps;
}

/* Non-finite gives 0; rounds to nearest; saturates. Out of line (IRAM, the
 * status fill's): a call each instead of ~60 bytes of IRAM per field. */
static AUTO_HOT __attribute__((noinline)) uint16_t satU16(float v)
{
  if (!(v > 0.0f)) {
    return 0;
  }
  return v >= 65535.0f ? 65535u : (uint16_t)(v + 0.5f);
}

static AUTO_HOT __attribute__((noinline)) int16_t satI16(float v)
{
  if (!(v == v)) {
    return 0;
  }
  if (v >= 32767.0f) {
    return 32767;
  }
  if (v <= -32767.0f) {
    return -32767;
  }
  return (int16_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

AUTO_HOT void autoStatusFill(autoStatusMsg_t *m, const autoFsmOut_t *o, const vzOutput_t *vz, const autoStatusExtra_t *x)
{
  memset(m, 0, sizeof(*m));
  m->upMs = x->upMs;
  m->transitions = x->transitions;
  m->failedGuards = o->failedGuards;
  m->state = (uint8_t)o->state;
  m->mode = (uint8_t)o->mode;
  m->reason = (uint8_t)o->reason;
  m->vzLandReason = (uint8_t)o->vzLandReason;
  m->landTrip = (uint8_t)o->landTrip;
  m->vzPhase = (uint8_t)vz->phase;
  m->vzReason = (uint8_t)vz->reason;
  m->flags = (uint8_t)((o->waiting ? AUTO_STF_WAITING : 0) | (o->stable ? AUTO_STF_STABLE : 0) |
                       (vz->camHealthy ? AUTO_STF_CAM_HEIGHT : 0) | (x->camAwake ? AUTO_STF_CAM_AWAKE : 0) |
                       (x->resetLocked ? AUTO_STF_RESET_LOCKED : 0) | (o->prespinLed ? AUTO_STF_PRESPIN : 0) |
                       (x->simPlant ? AUTO_STF_SIM : 0) | (x->holdApplied ? AUTO_STF_HOLD : 0));
  m->trips = x->trips;
  m->capture = (uint8_t)x->capture;
  m->photoId = x->photoId;
  m->stateMs = autoSatMs16(o->stateMs);
  m->airMs = autoSatMs16(o->airMs);
  m->stableMs = autoSatMs16(o->stableMs);
  m->waitLeftMs = autoSatMs16(o->waitLeftMs);
  m->hoverWaitMs = 0;   /* latched inside the FSM: the glue fills it */
  m->vbatMv = satU16(x->vbat * 1000.0f);
  m->vbatFastMinMv = satU16(x->vbatFastMin * 1000.0f);
  m->hMm = satI16(vz->h * 1000.0f);
  m->vzMms = satI16(vz->v * 1000.0f);
  m->thrust = satU16(vz->thrust);
  m->tLo = satU16(vz->tLo);
  m->tHold = satU16(vz->tHold);
  m->tRef = satU16(vz->tRef);
  m->kPriorDiv10 = satU16(x->kPrior * 0.1f);
  m->khatDiv10 = satU16(vz->khat * 0.1f);
  m->lastCmdType = x->lastCmdType;
  m->lastCmdResult = x->lastCmdResult;
  m->killCount = x->killCount;
  /* the interactive hover (auto_motion.h, AUTO_STATUS bytes 64..95) */
  const autoMvOut_t *mv = &o->mv;
  m->mvState = (uint8_t)mv->state;
  m->mvKind = (uint8_t)mv->kind;
  m->mvWhy = (uint8_t)mv->why;
  m->mvEnd = (uint8_t)mv->end;
  m->mvFlags = mv->flags;
  /* bit 0: held AUTO_VBAT_LOW_REFUSE_MS (MOVE / YAW refused; what the Mac acts on), bit 1: one reading */
  m->battLow = (uint8_t)((o->battLowHeld ? AUTO_BATTLOW_HELD : 0u) | (o->battLowNow ? AUTO_BATTLOW_NOW : 0u));
  m->mvSeq = mv->seq;
  m->progRightCm = satI16(mv->progRightM * 100.0f);
  m->progFwdCm = satI16(mv->progFwdM * 100.0f);
  m->progUpCm = satI16(mv->progUpM * 100.0f);
  m->tgtRightCm = satI16(mv->tgtRightM * 100.0f);
  m->tgtFwdCm = satI16(mv->tgtFwdM * 100.0f);
  m->tgtUpCm = satI16(mv->tgtUpM * 100.0f);
  m->sceneDCm = satU16(mv->sceneDM * 100.0f);
  m->hTargetCm = o->heightCm != 0 ? satI16((float)o->heightCm + mv->hOff * 100.0f) : 0;
  m->yawOffCdeg = satI16(mv->yawOffDeg * 100.0f);
  m->yawTgtDeg = satI16(mv->yawTgtDeg);
  m->mvPhaseMs = autoSatMs16(mv->phaseMs);
  m->airLeftDs = 0;   /* no air clock since 2026-10-15 */
  /* mvSpeedPct (bytes 136-137) 0 here: the glue fills it when the status is read (autonomy.c autonomyGetStatus) */
  autoStatusFillReport(m, o, vz, x);
}

/* The conditions behind auto_fsm.c's trip, rest and stillness timers (its
 * tests are private to it; these copy them). */
AUTO_HOT uint8_t autoTripBits(const autoFsmIn_t *in, bool vzAtCap)
{
  const float ax = fabsf(in->gyroDps[0]), ay = fabsf(in->gyroDps[1]), az = fabsf(in->gyroDps[2]);
  const float accErr = fabsf(in->accNormG - 1.0f);
  /* "not still", so a NaN counts as moved (auto_fsm.c moved()) */
  const bool still = in->tiltDeg < AUTO_STILL_TILT_DEG && ax <= AUTO_MOVED_GYRO_DPS && ay <= AUTO_MOVED_GYRO_DPS &&
                     az <= AUTO_MOVED_GYRO_DPS && accErr <= AUTO_MOVED_ACC_G;
  const bool rest = ax < AUTO_REST_GYRO_DPS && ay < AUTO_REST_GYRO_DPS && az < AUTO_REST_GYRO_DPS &&
                    accErr < AUTO_REST_ACC_G;
  uint8_t b = 0;
  b |= in->tiltDeg > AUTO_TILT_KILL_DEG ? AUTO_TRF_TILT : 0;
  b |= (ax > AUTO_RATE_KILL_DPS || ay > AUTO_RATE_KILL_DPS) ? AUTO_TRF_RATE : 0;
  b |= in->accNormG < AUTO_FALL_KILL_G ? AUTO_TRF_FALL : 0;
  b |= in->attErrDeg > AUTO_ATTERR_KILL_DEG ? AUTO_TRF_ATTERR : 0;
  b |= in->vbat < AUTO_VBAT_LAND_V ? AUTO_TRF_VBAT_LOW : 0;
  b |= still ? 0 : AUTO_TRF_MOVED;
  b |= rest ? AUTO_TRF_REST : 0;
  b |= vzAtCap ? AUTO_TRF_VZ_CAP : 0;
  return b;
}
