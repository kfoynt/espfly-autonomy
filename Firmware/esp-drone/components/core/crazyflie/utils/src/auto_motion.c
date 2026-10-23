/*
 * ESP-Drone Firmware
 *
 * auto_motion.c - Interactive hover motions: MOVE, YAW, STOP (auto_motion.h)
 */
#include "auto_motion.h"

#ifdef AUTO_MV_HOST_VH_OVERRIDE
float autoMvVhMaxOverride;   /* host tests: > 0 replaces AUTO_MV_V_H_MAX in the planning (0: the header's) */
#endif

#include <math.h>
#include <string.h>

/* autoMotionStep() and what it calls run on every stabilizer loop in HOVER:
 * IRAM on the drone (the 16 KB instruction cache is shared with the camera's
 * flow fit, docs/autonomy-a2b.md 2.6); nothing it calls on every loop is flash
 * code (sqrtf, expf and the soft-float helpers are IRAM or ROM). What runs
 * once per command or event stays in flash (AUTO_COLD, out of line so the
 * compiler cannot inline it into an IRAM caller): autoMotionInit (ARM),
 * autoMotionMove / autoMotionYaw (a command, with the profiles it plans:
 * autoMvProfPlan), autoMotionStop (a STOP, a guard, leaving HOVER). Internal
 * RAM is short (the heap at boot, docs/autonomy-a2b.md 14.7): IRAM is spent
 * on the per-loop path only. */
#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define AUTO_HOT IRAM_ATTR
#else
#define AUTO_HOT
#endif
#define AUTO_COLD __attribute__((noinline))
/* The small helpers are always inlined: out of line they would be flash code
 * (this file is built with -Os, CMakeLists.txt) called on every loop. */
#define AUTO_INLINE static inline __attribute__((always_inline))

#define BOB_ACC        AUTO_MV_BOB_ACC   /* m/s^2: the re-track bob's legs (at 100 %; autoMvSpeedCfg) */
#define D_FLOOR_M      0.05f    /* a scene closer than this along a forward path is never planned */

AUTO_INLINE float absf(float x)
{
  return x < 0.0f ? -x : x;
}

AUTO_INLINE float minf2(float a, float b)
{
  return a < b ? a : b;
}

AUTO_INLINE float maxf2(float a, float b)
{
  return a > b ? a : b;
}

AUTO_INLINE float clampf2(float x, float lo, float hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}

AUTO_INLINE bool finitef2(float x)
{
  return x == x && x - x == 0.0f;
}

/* ---- helpers --------------------------------------------------------------- */

AUTO_HOT float autoMvLn(float x)
{
  if (!(x > 0.0f)) {
    return -1e30f;
  }
  union {
    float f;
    uint32_t u;
  } v = {x};
  int e = (int)((v.u >> 23) & 0xFFu) - 127;
  if (e == -127) {   /* subnormal */
    v.f = x * 8388608.0f;
    e = (int)((v.u >> 23) & 0xFFu) - 127 - 23;
  }
  v.u = (v.u & 0x007FFFFFu) | 0x3F800000u;   /* mantissa in [1, 2) */
  float mt = v.f;
  if (mt > 1.41421356f) {
    mt *= 0.5f;
    e++;
  }
  /* ln(m) = 2 atanh(s), s = (m - 1) / (m + 1), |s| <= 0.172 */
  const float s = (mt - 1.0f) / (mt + 1.0f);
  const float s2 = s * s;
  const float p = s * (2.0f + s2 * (0.66666667f + s2 * (0.4f + s2 * (0.28571429f + s2 * 0.22222222f))));
  return p + (float)e * 0.69314718f;
}

void autoMvProfPlan(autoMvProf_t *p, float dist, float v, float a)
{
  memset(p, 0, sizeof(*p));
  if (!(dist > 0.0f) || !(v > 0.0f) || !(a > 0.0f) || !finitef2(dist)) {
    return;
  }
  p->dist = dist;
  p->a = a;
  float tA = v / a;
  if (a * tA * tA >= dist) {   /* triangle: never reaches v */
    tA = sqrtf(dist / a);
    p->vPk = a * tA;
    p->tC = 0.0f;
  } else {
    p->vPk = v;
    p->tC = (dist - v * tA) / v;
  }
  p->tA = tA;
  p->T = 2.0f * tA + p->tC;
}

AUTO_HOT void autoMvProfEval(const autoMvProf_t *p, float t, float *s, float *v, float *acc)
{
  if (!(p->T > 0.0f) || !(t > 0.0f)) {
    *s = 0.0f;
    *v = 0.0f;
    *acc = 0.0f;
    return;
  }
  if (t >= p->T) {
    *s = p->dist;
    *v = 0.0f;
    *acc = 0.0f;
    return;
  }
  if (t < p->tA) {
    *s = 0.5f * p->a * t * t;
    *v = p->a * t;
    *acc = p->a;
  } else if (t < p->tA + p->tC) {
    *s = 0.5f * p->a * p->tA * p->tA + p->vPk * (t - p->tA);
    *v = p->vPk;
    *acc = 0.0f;
  } else {
    const float td = p->T - t;
    *s = p->dist - 0.5f * p->a * td * td;
    *v = p->a * td;
    *acc = -p->a;
  }
}

/* ---- state ------------------------------------------------------------------ */

AUTO_COLD void autoMotionInit(autoMotion_t *m)
{
  memset(m, 0, sizeof(*m));
  m->state = AUTO_MV_IDLE;
  m->kind = AUTO_MVK_NONE;
  /* the speed step 100: the header's limits (the step reads these fields) */
  m->speedPct = AUTO_MV_SPEED_DEFAULT_PCT;
  m->settleMaxMs = AUTO_MV_SETTLE_MAX_MS;
  m->dwellMs = AUTO_MV_YAW_DWELL_MS;
  m->brakeAcc = AUTO_MV_BRAKE_ACC;
  m->frozenRef = AUTO_MV_FROZEN_REF;
}

/* ---- mvspeed (auto_motion.h "fast moves", 2026-10-16) ------------------------------------------------------------- */

/* The fast steps' acceleration cap and settle floor in force (NVS mvacc, mvsettle; auto_motion.h "the fast steps'
 * acceleration and settle as settings", 2026-10-17): autoMotionSetFastCaps at ARM; 8 B of .data. aH is the cap itself
 * (mvacc / 100.0f, correctly rounded: the default is AUTO_MV_FAST_A_H_CAP bit for bit). Read by autoMvSpeedCfg only. */
_Static_assert(AUTO_MV_ACC_DEFAULT_CM / 100.0f == AUTO_MV_FAST_A_H_CAP && AUTO_MV_ACC_MIN_CM <= AUTO_MV_ACC_DEFAULT_CM &&
               AUTO_MV_ACC_DEFAULT_CM <= AUTO_MV_ACC_MAX_CM && AUTO_MV_ACC_MAX_CM <= 255 &&
               AUTO_MV_ACC_MIN_CM / 100.0f == AUTO_MV_A_H &&
               AUTO_MV_SETTLE_FLOOR_MIN_MS <= AUTO_MV_FAST_SETTLE_MIN_MS &&
               AUTO_MV_FAST_SETTLE_MIN_MS <= AUTO_MV_SETTLE_FLOOR_MAX_MS && AUTO_MV_SETTLE_FLOOR_MAX_MS <= 65535,
               "mvacc and mvsettle: the defaults are the caps of d3bedee, inside their ranges, each fits its field");
/* With them the expansion rate cap (NVS mvwmax, auto_motion.h "guards off", 2026-10-18): wFwd is the cap itself
 * (mvwmax / 100.0f, correctly rounded: the default is AUTO_MV_W_FWD_MAX bit for bit), at every speed step. */
_Static_assert(AUTO_MV_W_FWD_PCT / 100.0f == AUTO_MV_W_FWD_MAX && AUTO_MV_MG_WFWD_MIN_PCT <= AUTO_MV_W_FWD_PCT &&
               AUTO_MV_W_FWD_PCT <= AUTO_MV_MG_WFWD_MAX_PCT && AUTO_MV_MG_WFWD_MAX_PCT <= 255,
               "mvwmax: the default is AUTO_MV_W_FWD_MAX, inside its range, fits its byte");
/* With them the VIEW_FAST timer (NVS mvview, auto_motion.h "fast moves", 2026-10-22): the profile's by default. */
_Static_assert(AUTO_MV_FAST_VIEW_DEB_MS >= 150 && AUTO_MV_FAST_VIEW_DEB_MS <= AUTO_MV_VIEW_DEB_MAX_MS &&
               AUTO_MV_VIEW_DEB_MAX_MS < AUTO_MV_NO_CAM_LAND_MS,
               "mvview: the profile's timer inside the range, the range below the NO_VIEW stop");
static struct {
  float aH;            /* m/s^2: A_H's cap above 100 */
  float wFwd;          /* 1/s: the expansion rate cap (mvwmax) */
  uint16_t settleMs;   /* the settle's floor above 100 */
  uint16_t viewDebMs;  /* ms: the VIEW_FAST timer (mvview; the profile's AUTO_MV_FAST_VIEW_DEB_MS by default) */
  uint8_t accCm;       /* (the setting, for autoMotionGetFastCaps) */
  uint8_t wFwdPct;     /* (the setting, for autoMotionGetFwdRate) */
  int8_t ffRollD;      /* x 0.1 deg per m/s of reference forward speed: the roll feedforward's share (mvffr; 0 none; in
                          the struct's padding: no new bytes) */
} mvFast = {AUTO_MV_FAST_A_H_CAP, AUTO_MV_W_FWD_MAX, AUTO_MV_FAST_SETTLE_MIN_MS, AUTO_MV_FAST_VIEW_DEB_MS,
            AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_W_FWD_PCT, 0};

AUTO_COLD void autoMotionSetFastCaps(int32_t accCm, int32_t settleMs)
{
  const int32_t a = accCm < AUTO_MV_ACC_MIN_CM ? AUTO_MV_ACC_MIN_CM : (accCm > AUTO_MV_ACC_MAX_CM ? AUTO_MV_ACC_MAX_CM
                                                                                                  : accCm);
  const int32_t t = settleMs < AUTO_MV_SETTLE_FLOOR_MIN_MS
                        ? AUTO_MV_SETTLE_FLOOR_MIN_MS
                        : (settleMs > AUTO_MV_SETTLE_FLOOR_MAX_MS ? AUTO_MV_SETTLE_FLOOR_MAX_MS : settleMs);
  mvFast.accCm = (uint8_t)a;
  mvFast.aH = (float)a / 100.0f;
  mvFast.settleMs = (uint16_t)t;
}

AUTO_COLD void autoMotionGetFastCaps(uint8_t *accCm, uint16_t *settleMs)
{
  *accCm = mvFast.accCm;
  *settleMs = mvFast.settleMs;
}

AUTO_COLD void autoMotionSetFwdRate(int32_t wPct)
{
  const int32_t w = wPct < AUTO_MV_MG_WFWD_MIN_PCT ? AUTO_MV_MG_WFWD_MIN_PCT
                                                   : (wPct > AUTO_MV_MG_WFWD_MAX_PCT ? AUTO_MV_MG_WFWD_MAX_PCT : wPct);
  mvFast.wFwdPct = (uint8_t)w;
  mvFast.wFwd = (float)w / 100.0f;
}

AUTO_COLD uint8_t autoMotionGetFwdRate(void)
{
  return mvFast.wFwdPct;
}

AUTO_COLD void autoMotionSetViewDeb(int32_t ms)
{
  /* 0 and anything shorter than the profile's timer: the profile's (the timer only ever gets longer) */
  const int32_t v = ms < (int32_t)AUTO_MV_FAST_VIEW_DEB_MS
                        ? (int32_t)AUTO_MV_FAST_VIEW_DEB_MS
                        : (ms > AUTO_MV_VIEW_DEB_MAX_MS ? AUTO_MV_VIEW_DEB_MAX_MS : ms);
  mvFast.viewDebMs = (uint16_t)v;
}

AUTO_COLD uint16_t autoMotionGetViewDeb(void)
{
  return mvFast.viewDebMs;
}

AUTO_COLD void autoMotionSetFfRoll(int32_t dDegPerMps)
{
  const int32_t d = dDegPerMps < AUTO_MV_FFR_MIN_DDEG ? AUTO_MV_FFR_MIN_DDEG
                                                      : (dDegPerMps > AUTO_MV_FFR_MAX_DDEG ? AUTO_MV_FFR_MAX_DDEG
                                                                                           : dDegPerMps);
  mvFast.ffRollD = (int8_t)d;
}

AUTO_COLD int8_t autoMotionGetFfRoll(void)
{
  return mvFast.ffRollD;
}

AUTO_COLD void autoMvSpeedCfg(uint16_t pct, autoMvSpeed_t *c)
{
  memset(c, 0, sizeof(*c));
  if (pct > AUTO_MV_SPEED_MAX_PCT) {
    pct = AUTO_MV_SPEED_MAX_PCT;
  }
  /* 100 (and below): the header constants themselves, no new rule */
  c->pct = AUTO_MV_SPEED_DEFAULT_PCT;
  c->vH = AUTO_MV_V_H_MAX;
  c->vBack = AUTO_MV_V_H_MAX;
  c->vFwd = AUTO_MV_V_FWD_MAX;
  c->aH = AUTO_MV_A_H;
  c->wLat = AUTO_MV_W_LAT_MAX;
  c->wFwd = mvFast.wFwd;   /* (mvwmax; the header's AUTO_MV_W_FWD_MAX by default) */
  c->brake = AUTO_MV_BRAKE_ACC;
  c->settleMaxMs = AUTO_MV_SETTLE_MAX_MS;
  c->dwellMs = AUTO_MV_YAW_DWELL_MS;
  c->yawRate = AUTO_MV_YAW_RATE_DPS;
  c->yawAcc = AUTO_MV_YAW_ACC_DPS2;
  c->bobAcc = AUTO_MV_BOB_ACC;
  c->tTrack = 0.0f;
  c->frozenRef = AUTO_MV_FROZEN_REF;
  if (pct <= AUTO_MV_SPEED_DEFAULT_PCT) {
    return;
  }
  const float s = (float)pct * 0.01f;
  c->pct = pct;
  c->fast = true;
  c->vH = minf2(AUTO_MV_V_H_MAX * s, AUTO_MV_FAST_V_H_CAP);
  c->vBack = minf2(AUTO_MV_V_H_MAX * s, AUTO_MV_FAST_V_BACK_CAP);
  c->vFwd = minf2(AUTO_MV_V_FWD_MAX * s, AUTO_MV_FAST_V_FWD_CAP);
  c->aH = minf2(AUTO_MV_A_H * s, mvFast.aH);   /* (mvacc) */
  c->wLat = minf2(AUTO_MV_W_LAT_MAX * s, AUTO_MV_FAST_W_LAT_CAP);
  c->wFwd = minf2(mvFast.wFwd * s, maxf2(AUTO_MV_FAST_W_FWD_CAP, mvFast.wFwd));   /* (mvwmax: a cap set above 0.17
                                                                                      is itself the fast cap) */
  c->brake = minf2(AUTO_MV_BRAKE_ACC * s, AUTO_MV_FAST_BRAKE_CAP);
  const float settle = (float)AUTO_MV_SETTLE_MAX_MS / s + 0.5f, dwell = (float)AUTO_MV_YAW_DWELL_MS / s + 0.5f;
  c->settleMaxMs = (uint16_t)maxf2(settle, (float)mvFast.settleMs);   /* (mvsettle) */
  c->dwellMs = (uint16_t)maxf2(dwell, (float)AUTO_MV_FAST_DWELL_MIN_MS);
  c->yawRate = minf2(AUTO_MV_YAW_RATE_DPS * s, AUTO_MV_FAST_YAW_RATE_CAP);
  c->yawAcc = minf2(AUTO_MV_YAW_ACC_DPS2 * s, AUTO_MV_FAST_YAW_ACC_CAP);
  c->bobAcc = minf2(AUTO_MV_BOB_ACC * s, AUTO_MV_FAST_BOB_ACC_CAP);
  c->tTrack = AUTO_MV_FAST_T_TRACK_S;
  c->frozenRef = AUTO_MV_FROZEN_REF + AUTO_MV_FAST_T_TRACK_S * c->wFwd;
}

AUTO_COLD void autoMotionSetSpeed(autoMotion_t *m, uint16_t pct)
{
  autoMvSpeed_t c;
  autoMvSpeedCfg(pct, &c);
  m->speedPct = c.pct;
  m->settleMaxMs = c.settleMaxMs;
  m->dwellMs = c.dwellMs;
  m->brakeAcc = c.brake;
  m->frozenRef = c.frozenRef;
}

/* ---- guard margins (auto_motion.h "guard margins", 2026-10-17) ----------------------------------------------------- */

_Static_assert(AUTO_PLAN_MIN_FRONT_CM == AUTO_MOVE_MIN_FRONT_CM + AUTO_PLAN_FRONT_EXTRA_CM,
               "the plan's default floor is the default mvfloor + the Mac's extra");
_Static_assert(AUTO_MOVE_MIN_FRONT_CM >= AUTO_MV_MG_FLOOR_MIN_CM && AUTO_MOVE_MIN_FRONT_CM <= AUTO_MV_MG_FLOOR_MAX_CM &&
               AUTO_MV_FRONT_STOP_CM >= AUTO_MV_MG_STOP_MIN_CM &&
               AUTO_MV_FRONT_STOP_CM <= AUTO_MOVE_MIN_FRONT_CM - AUTO_MV_FRONT_STOP_GAP_CM &&
               AUTO_MV_PROG_MARGIN_CM >= AUTO_MV_MG_PROG_MIN_CM && AUTO_MV_PROG_MARGIN_CM <= AUTO_MV_MG_PROG_MAX_CM &&
               AUTO_MV_PROG_MARGIN_PCT >= AUTO_MV_MG_PCT_MIN && AUTO_MV_PROG_MARGIN_PCT <= AUTO_MV_MG_PCT_MAX &&
               AUTO_MV_LAT_MARGIN_CM >= AUTO_MV_MG_LAT_MIN_CM && AUTO_MV_LAT_MARGIN_CM <= AUTO_MV_MG_LAT_MAX_CM &&
               AUTO_MV_NEAR_SCENE_CM <= AUTO_MV_MG_NEAR_MAX_CM && AUTO_MVFIX_DEFAULT <= AUTO_MVFIX_MAX,
               "the default margins inside their NVS ranges");
_Static_assert(AUTO_MV_MG_FLOOR_MIN_CM - AUTO_MV_FRONT_STOP_GAP_CM >= AUTO_MV_MG_STOP_MIN_CM &&
               AUTO_MV_MG_FLOOR_MAX_CM + AUTO_PLAN_FRONT_EXTRA_CM <= 255 && AUTO_MV_MG_STOP_MAX_CM <= 255 &&
               AUTO_MV_MG_PROG_MAX_CM <= 255 && AUTO_MV_MG_PCT_MAX <= 255 && AUTO_MV_MG_LAT_MAX_CM <= 255 &&
               AUTO_MV_MG_NEAR_MAX_CM <= 255,
               "the clamped stop stays in its range; every margin fits its byte");
_Static_assert(AUTO_MV_K_LOW_PCT / 100.0f == AUTO_MV_K_LOW && AUTO_MV_MG_KLOW_MIN_PCT <= AUTO_MV_K_LOW_PCT &&
               AUTO_MV_K_LOW_PCT <= AUTO_MV_MG_KLOW_MAX_PCT && AUTO_MV_MG_KLOW_MAX_PCT <= 100 &&
               AUTO_MV_MG_KLOW_MIN_PCT > 0 && AUTO_MV_LAT_D_REL_PCT / 100.0f == AUTO_MV_LAT_D_REL &&
               AUTO_MV_LAT_D_REL_PCT <= AUTO_MV_MG_LAT_REL_MAX_PCT && AUTO_MV_MG_LAT_REL_MAX_PCT <= 255,
               "mvklow and mvlat: the defaults are AUTO_MV_K_LOW and AUTO_MV_LAT_D_REL, inside their ranges, each fits its "
               "byte");

/* The margins in force (autoMotionSetMargins at ARM; 10 B of .data). Read by the once-per-command code only (flash). */
static autoMvMargins_t mvMg = {AUTO_MOVE_MIN_FRONT_CM, AUTO_PLAN_MIN_FRONT_CM, AUTO_MV_FRONT_STOP_CM,
                               AUTO_MV_PROG_MARGIN_CM, AUTO_MV_PROG_MARGIN_PCT, AUTO_MV_LAT_MARGIN_CM,
                               AUTO_MV_NEAR_SCENE_CM, AUTO_MVFIX_DEFAULT, AUTO_MV_K_LOW_PCT, AUTO_MV_LAT_D_REL_PCT};

static int32_t clampI(int32_t x, int32_t lo, int32_t hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}

/* Into lo..hi, but 0 stays 0 (off; auto_motion.h "guards off"). A negative value clamps up to lo as before. */
static int32_t clampOff(int32_t x, int32_t lo, int32_t hi)
{
  return x == 0 ? 0 : clampI(x, lo, hi);
}

/* The lowest expansion scale k in force (mvklow / 100; 0.60f bit for bit at the default) and the reciprocal the floor's
 * power uses. Flash. */
static float kLow(void)
{
  return (float)mvMg.kLowPct / 100.0f;
}

/* The progress guard is off: mvprogm 0 or mvprogr 0 (auto_motion.h "guards off"). */
static bool progOff(void)
{
  return mvMg.progCm == 0u || mvMg.progPct == 0u;
}

AUTO_COLD void autoMvMarginsDefault(autoMvMargins_t *g)
{
  g->floorCm = AUTO_MOVE_MIN_FRONT_CM;
  g->planCm = AUTO_PLAN_MIN_FRONT_CM;
  g->stopCm = AUTO_MV_FRONT_STOP_CM;
  g->progCm = AUTO_MV_PROG_MARGIN_CM;
  g->progPct = AUTO_MV_PROG_MARGIN_PCT;
  g->latCm = AUTO_MV_LAT_MARGIN_CM;
  g->nearCm = AUTO_MV_NEAR_SCENE_CM;
  g->fix = AUTO_MVFIX_DEFAULT;
  g->kLowPct = AUTO_MV_K_LOW_PCT;
  g->latPct = AUTO_MV_LAT_D_REL_PCT;
}

AUTO_COLD void autoMvMarginsOld(autoMvMargins_t *g)
{
  g->floorCm = 50;
  g->planCm = 60;
  g->stopCm = 40;
  g->progCm = 15;
  g->progPct = 30;
  g->latCm = 25;
  g->nearCm = 0;   /* (no near-scene brake before 2026-10-17) */
  g->fix = 0;      /* (none of the move fixes) */
  g->kLowPct = AUTO_MV_K_LOW_PCT;      /* (k 0.6 and 0.5 x D: constants until 2026-10-18) */
  g->latPct = AUTO_MV_LAT_D_REL_PCT;
}

/* Every field into its range: the plan at least the floor, the stop AUTO_MV_FRONT_STOP_GAP_CM below the floor; 0 stays
 * 0 (off) for the floor (then the plan too), the stop, the progress base and part, and the sideways cap. */
static AUTO_COLD void marginsClamp(autoMvMargins_t *g, int32_t floorCm, int32_t planCm, int32_t stopCm, int32_t progCm,
                                   int32_t progPct, int32_t latCm, int32_t nearCm, int32_t fix, int32_t kLowPct,
                                   int32_t latPct)
{
  g->fix = (uint8_t)((uint32_t)clampI(fix, 0, (int32_t)AUTO_MVFIX_MAX) & AUTO_MVFIX_MAX);
  g->nearCm = (uint8_t)clampI(nearCm, 0, AUTO_MV_MG_NEAR_MAX_CM);
  const int32_t fl = clampOff(floorCm, AUTO_MV_MG_FLOOR_MIN_CM, AUTO_MV_MG_FLOOR_MAX_CM);
  g->floorCm = (uint8_t)fl;
  g->planCm = (uint8_t)(fl == 0 ? 0 : clampI(planCm, fl, AUTO_MV_MG_FLOOR_MAX_CM + AUTO_PLAN_FRONT_EXTRA_CM));
  /* the stop below the floor when there is one; with no floor its own range */
  const int32_t stopHi = fl == 0 ? AUTO_MV_MG_STOP_MAX_CM
                                 : (fl - AUTO_MV_FRONT_STOP_GAP_CM < AUTO_MV_MG_STOP_MAX_CM ? fl - AUTO_MV_FRONT_STOP_GAP_CM
                                                                                            : AUTO_MV_MG_STOP_MAX_CM);
  g->stopCm = (uint8_t)clampOff(stopCm, AUTO_MV_MG_STOP_MIN_CM, stopHi);
  g->progCm = (uint8_t)clampOff(progCm, AUTO_MV_MG_PROG_MIN_CM, AUTO_MV_MG_PROG_MAX_CM);
  g->progPct = (uint8_t)clampOff(progPct, AUTO_MV_MG_PCT_MIN, AUTO_MV_MG_PCT_MAX);
  g->latCm = (uint8_t)clampI(latCm, AUTO_MV_MG_LAT_MIN_CM, AUTO_MV_MG_LAT_MAX_CM);
  g->kLowPct = (uint8_t)clampI(kLowPct, AUTO_MV_MG_KLOW_MIN_PCT, AUTO_MV_MG_KLOW_MAX_PCT);
  g->latPct = (uint8_t)clampI(latPct, 0, AUTO_MV_MG_LAT_REL_MAX_PCT);
}

AUTO_COLD void autoMvMarginsFrom(int32_t floorCm, int32_t stopCm, int32_t progCm, int32_t progPct, int32_t latCm,
                                 int32_t nearCm, int32_t fix, autoMvMargins_t *g)
{
  const int32_t fl = clampOff(floorCm, AUTO_MV_MG_FLOOR_MIN_CM, AUTO_MV_MG_FLOOR_MAX_CM);
  marginsClamp(g, fl, fl + AUTO_PLAN_FRONT_EXTRA_CM, stopCm, progCm, progPct, latCm, nearCm, fix, AUTO_MV_K_LOW_PCT,
               AUTO_MV_LAT_D_REL_PCT);
}

AUTO_COLD void autoMvMarginsExtra(autoMvMargins_t *g, int32_t kLowPct, int32_t latPct)
{
  g->kLowPct = (uint8_t)clampI(kLowPct, AUTO_MV_MG_KLOW_MIN_PCT, AUTO_MV_MG_KLOW_MAX_PCT);
  g->latPct = (uint8_t)clampI(latPct, 0, AUTO_MV_MG_LAT_REL_MAX_PCT);
}

AUTO_COLD void autoMotionSetMargins(const autoMvMargins_t *g)
{
  autoMvMargins_t d;
  if (g == NULL) {
    autoMvMarginsDefault(&d);
    g = &d;
  }
  marginsClamp(&mvMg, g->floorCm, g->planCm, g->stopCm, g->progCm, g->progPct, g->latCm, g->nearCm, g->fix, g->kLowPct,
               g->latPct);
}

AUTO_COLD void autoMotionGetMargins(autoMvMargins_t *g)
{
  *g = mvMg;
}

/* The forward image-rate cap on the profile's true peak (auto_motion.h autoMvFwdPeakCap). Along the decel (remaining
 * distance r, speed sqrt(2 a r), the scene De + r ahead) the rate is sqrt(2 a r) / (De + r), largest at r = De:
 * sqrt(a / (2 De)); at the cruise's end (r = v^2 / (2 a)) it is v / (De + v^2 / (2 a)), which stays <= w for v up to
 * the smaller root of (w / (2 a)) v^2 - v + w De (that root's r is <= De, so the decel's interior peak is not
 * reached). */
AUTO_COLD float autoMvFwdPeakCap(float d, float f, float a, float w)
{
  const float de = maxf2(d - maxf2(f, 0.0f), D_FLOOR_M);
  if (!(a > 0.0f) || !(w > 0.0f)) {
    return 1e30f;
  }
  const float disc = 1.0f - 2.0f * w * w * de / a;
  if (!(disc > 0.0f)) {
    return 1e30f;   /* sqrt(a / (2 De)) <= w: no speed reaches w */
  }
  return (a / w) * (1.0f - sqrtf(disc));
}

AUTO_HOT bool autoMotionActive(const autoMotion_t *m)
{
  return m->state == AUTO_MV_RUN || m->state == AUTO_MV_PAUSED || m->state == AUTO_MV_SETTLE;
}

static AUTO_HOT void setState(autoMotion_t *m, autoMvState_t s, uint32_t now)
{
  m->state = s;
  m->stateMs = now;
}

/* A new epoch of the hold's reference; with snap the camera first takes the
 * integral back to its value at the start of the epoch it ends. The measured
 * progress is not cleared here: a stopped MOVE keeps it as its travel (a new
 * motion clears it, accept()). */
static AUTO_HOT void newEpoch(autoMotion_t *m, bool snap)
{
  m->epoch++;
  m->dTrackRef = 0.0f;   /* dTrack is the scene distance at this epoch's start (mvfix DRIFT; unread without it) */
  m->snap = snap;
  if (snap) {
    m->snapEpoch = m->epoch;
  }
}

/* The scene distances a command would use now (all 0: none; auto_motion.h
 * "Which D"): fwd converts a forward move (and is the one reported), lat a
 * lateral one (the larger: a D too large makes a move short, never long),
 * low is the lower bound the forward floor is checked against. */
typedef struct {
  float obs, fwd, lat, low;
} sceneDs_t;

static AUTO_HOT uint8_t sceneD(const autoMotion_t *m, const autoMvIn_t *in, sceneDs_t *ds)
{
  uint8_t f = 0;
  float d = 0.0f;
  const bool stale = m->dStale || (m->kind == AUTO_MVK_MOVE && m->bob);   /* a turn; a move's re-view bob */
  if (in->simPlant) {
    d = AUTO_MV_SIM_D_M;
    f |= AUTO_MVF_D_SIM;
  } else if (!stale && in->sceneTracked && finitef2(in->sceneDM) && in->sceneDM >= AUTO_MV_D_MIN_M &&
             in->sceneDM <= AUTO_MV_D_MAX_M) {
    d = in->sceneDM;
  }
  ds->obs = ds->fwd = ds->lat = ds->low = d;
  if (d > 0.0f && m->dTrack > 0.0f) {
    /* the last forward / back move measured where the scene is (G barely moves in a still hover) */
    f |= AUTO_MVF_D_TRACK;
    if (m->dTrack >= AUTO_MV_D_MIN_M) {
      ds->fwd = m->dTrack;
      ds->lat = maxf2(d, m->dTrack);
      ds->low = minf2(d, m->dTrackLow > 0.0f ? m->dTrackLow : m->dTrack);
    } else {
      ds->obs = ds->fwd = ds->lat = ds->low = 0.0f;
    }
  }
  if (stale) {
    f |= AUTO_MVF_D_STALE;
  }
  if (ds->fwd > 0.0f) {
    f |= AUTO_MVF_D_VALID;
  }
  return f;
}

/* The lowest height a move down may leave the drone at (commanded and
 * estimated): the floor plus the drift the estimate may have gathered, that
 * term capped (2026-10-15: at most 0.55 m). */
static float heightFloor(const autoMvIn_t *in)
{
  return AUTO_MV_H_MIN_M + minf2(AUTO_MV_H_DRIFT_CAP_M, AUTO_MV_H_DRIFT_MPS * (float)in->airMs * 0.001f);
}

AUTO_COLD float autoMotionHeightFloor(const autoMvIn_t *in)
{
  return heightFloor(in);
}

static AUTO_HOT bool viewOk(const autoMvIn_t *in)
{
  return !in->flight ||
         (in->holdApplied && !in->holdCoasting && in->camFrameAgeMs <= AUTO_MV_VIEW_MAX_AGE_MS);
}

static autoMvWhy_t refuse(autoMotion_t *m, autoMvWhy_t w)
{
  m->why = w;
  return w;
}

/* Common to MOVE and YAW. Near the end of the battery (battLow) nothing flies, a vertical MOVE down included: in the
 * drain sim one 0.5 m descent accepted at T_hold 54600-56000 ended HOLD_CAPPED (its braking and the loop's swing at the
 * 58000 ceiling for 0.5 s: a fast LAND, 1.2-1.5 m/s) in 1-2 of 40-50 sorties, none below 54000 (2026-10-15 review,
 * docs/autonomy-a2b.md 23.9; the runbook comes down from T_hold 52000). */
static autoMvWhy_t precheck(autoMotion_t *m, const autoMvIn_t *in)
{
  if (autoMotionActive(m)) {
    return refuse(m, AUTO_MVW_BUSY);
  }
  if (in->battLow) {
    return refuse(m, AUTO_MVW_BATT);
  }
  return AUTO_MVW_NONE;
}

static void accept(autoMotion_t *m, const autoMvIn_t *in, autoMvKind_t kind)
{
  m->brakeS = 0.0f;
  m->backOff = false;
  m->why = AUTO_MVW_NONE;
  m->end = AUTO_MVE_NONE;
  m->seq++;
  m->tRun = 0.0f;
  m->lastMs = in->nowMs;
  m->started = true;
  m->stillWait = false;
  m->viewBad = false;
  m->tiltOn = false;
  if (m->kind == AUTO_MVK_MOVE) {
    m->fwdLnBase += m->progFwd;   /* (m->kind is still the previous motion's) its travel toward the scene: the
                                     observer's gain follows the sum (autoMotionFwdLn) */
  }
  m->kind = kind;
  m->yawFlags = 0;
  m->doneEnd = AUTO_MVE_DONE;
  m->progLat = 0.0f;
  m->progFwd = 0.0f;
  m->progUp = 0.0f;
  m->shortEnd = false;
  m->yawResetSeen = in->sceneResets;
  m->lateYawRecoveryUsed = false;
  newEpoch(m, false);
  setState(m, AUTO_MV_RUN, in->nowMs);
}

AUTO_COLD bool autoMvRangeOk(int32_t rightCm, int32_t fwdCm, int32_t upCm)
{
  const int32_t r = rightCm, f = fwdCm, u = upCm;
  if (r > AUTO_MV_HORIZ_MAX_CM || r < -AUTO_MV_HORIZ_MAX_CM || f > AUTO_MV_FWD_MAX_CM || f < -AUTO_MV_HORIZ_MAX_CM ||
      u > AUTO_MV_UP_MAX_CM || u < -AUTO_MV_UP_MAX_CM || (r == 0 && f == 0 && u == 0)) {
    return false;
  }
  if (r * r + f * f > (int32_t)AUTO_MV_LEN_MAX_CM * AUTO_MV_LEN_MAX_CM) {
    return false;   /* (150, 371): 401.2 cm */
  }
  /* a vertical move reads partly as forward on the camera: no long forward part with it */
  return u == 0 || f <= AUTO_MV_HORIZ_MAX_CM;
}

/* autoMotionMove / autoMotionMovePlan: floorM is the forward floor (the scene ahead at the true end, at its
 * lowest estimate). */
static AUTO_COLD autoMvWhy_t moveFloor(autoMotion_t *m, const autoMvIn_t *in, int16_t rightCm, int16_t fwdCm,
                                       int16_t upCm, float floorM)
{
  autoMvWhy_t w = precheck(m, in);
  if (w != AUTO_MVW_NONE) {
    return w;
  }
  const int32_t r = rightCm, f = fwdCm, u = upCm;
  if (!autoMvRangeOk(r, f, u)) {
    return refuse(m, AUTO_MVW_RANGE);
  }
  autoMotionTrackNow(m, in);   /* mvfix DRIFT: the measured D moved by the drift since it was taken */
  const float latM = (float)r / 100.0f, fwdM = (float)f / 100.0f, upM = (float)u / 100.0f;
  const bool horiz = r != 0 || f != 0;
  sceneDs_t ds = {0.0f, 0.0f, 0.0f, 0.0f};
  if (horiz) {
    (void)sceneD(m, in, &ds);
    if (!(ds.fwd > 0.0f)) {
      return refuse(m, AUTO_MVW_NO_DIST);
    }
    if (mvMg.latPct != 0u && absf(latM) > (float)mvMg.latPct / 100.0f * ds.lat + 1e-4f) {
      return refuse(m, AUTO_MVW_TOO_WIDE);   /* a depth step under it would take it D_far / D_near too far (mvlat;
                                                0: never) */
    }
    if (fwdM > 0.0f) {
      /* the true end D_now ((D - F) / D)^(1/k), at its lowest: D_now at its
       * lower bound, k at its lowest (mvklow; AUTO_MV_K_LOW by default) */
      const float rr = (ds.fwd - fwdM) / ds.fwd;
      if (!(ds.fwd - fwdM >= D_FLOOR_M) ||
          !(ds.low * expf(autoMvLn(rr) * (1.0f / kLow())) >= floorM) ||
          (floorM > 0.0f && !(ds.lat - fwdM >= AUTO_MV_LAT_GUARD_D_MIN_M))) {
        /* (the last: the sideways guard's distance floor, which the step then never needs; the scene floor
         * already keeps >= 0.41 m, auto_motion.h. With no floor (mvfloor 0) it goes too: a move may end nearer
         * than 0.30 m by the lateral D, where the sideways guard reads its error at that smaller distance) */
        return refuse(m, AUTO_MVW_TOO_CLOSE);
      }
    }
  }
  if (!viewOk(in)) {
    return refuse(m, AUTO_MVW_NO_VIEW);   /* the hold holds the drone during any move, vertical too */
  }
  if (u != 0 && in->flight && !in->camHeight) {
    return refuse(m, AUTO_MVW_NO_CAM_HEIGHT);
  }
  if (u != 0) {
    /* one-sided (2026-10-15): a move down against the floor plus the drift the height estimate may have gathered (it
     * reads high: the drone is lower than it thinks), a move up against the top; commanded and estimated */
    const float hRef = in->hTargetM + m->hOff + upM, hEst = in->vzH + upM;
    if (u < 0 ? !(hRef >= heightFloor(in) && hEst >= heightFloor(in))
              : !(hRef <= AUTO_MV_H_MAX_M && hEst <= AUTO_MV_H_MAX_M)) {
      return refuse(m, AUTO_MVW_HEIGHT);
    }
  }
  /* Horizontal: one straight-line profile in metres, as fast as the limits
   * allow: speed, and the image rates the camera keeps its view at
   * (|wLat| = v_lat / D, |wFwd| = v_fwd / D_now, D_now smallest at the
   * forward end; the smaller D estimate). */
  autoMvProf_t ph, pv;
  const float len = sqrtf(latM * latM + fwdM * fwdM);
  const float dRate = minf2(ds.obs, ds.fwd);
  /* the sortie's speed step (auto_motion.h "fast moves"): at 100 the header constants */
  autoMvSpeed_t sp;
  autoMvSpeedCfg(m->speedPct, &sp);
#ifdef AUTO_MV_HOST_VH_OVERRIDE
  /* host tests only (never on the drone): one sim binary compares two top speeds on the same seeds */
  float vmax = autoMvVhMaxOverride > 0.0f ? autoMvVhMaxOverride : sp.vH;
#else
  float vmax = sp.vH;
#endif
  if (horiz) {
    if (latM != 0.0f) {
      /* the sideways image rate is v_lat / D_now, smallest D_now at a diagonal's forward end (2026-10-14: at the
       * start D a long diagonal reached 1.3-2.5x the cap there, past the observer's 1.5 px gate) */
      vmax = minf2(vmax, sp.wLat * maxf2(dRate - maxf2(fwdM, 0.0f), D_FLOOR_M) * len / absf(latM));
    }
    if (fwdM != 0.0f && !(sp.fast && fwdM > 0.0f)) {
      vmax = minf2(vmax, sp.wFwd * maxf2(dRate - maxf2(fwdM, 0.0f), D_FLOOR_M) * len / absf(fwdM));
    }
    if (fwdM > 0.0f) {
      vmax = minf2(vmax, sp.vFwd * len / fwdM);   /* toward the scene: the forward component <= 0.20 m/s at 100 */
    }
    if (sp.fast && fwdM > 0.0f) {
      /* the expansion's true peak along the profile (not v / D_end): its forward component's acceleration a uy */
      vmax = minf2(vmax, autoMvFwdPeakCap(dRate, fwdM, sp.aH * fwdM / len, sp.wFwd) * len / fwdM);
    }
    if (sp.fast && fwdM < 0.0f) {
      vmax = minf2(vmax, sp.vBack * len / -fwdM);   /* away from the scene: the camera does not see behind */
    }
  }
  autoMvProfPlan(&ph, horiz ? len : 0.0f, vmax, sp.aH);
  autoMvProfPlan(&pv, absf(upM), AUTO_MV_V_UP_MAX, AUTO_MV_A_UP);
  const float d = ds.fwd, dl = ds.lat;
  m->d0 = d;
  m->dLat0 = dl;
  m->dLow0 = ds.low;
  m->latM = latM;
  m->fwdM = fwdM;
  m->lenM = horiz ? len : 0.0f;
  m->ph = ph;
  m->latTgt = horiz ? latM / dl : 0.0f;
  m->fwdTgt = horiz ? autoMvLn(d / (d - fwdM)) : 0.0f;
  /* The sideways margin in metres at the reference's distance now (RUN and SETTLE, auto_motion.h): at least mvlatm
   * (the hover's own sideways wander; the tiger2 forward move was stopped by 0.151 m, the approach steps near a
   * bright window of 2026-10-17 by 0.25-0.36 m), mvprogm + mvprogr of the sideways part. The forward one: mvprogm at
   * the end distance + mvprogr of the ln target (its behind side catches a frozen view, its ahead side toward the
   * scene is the front stop's); a back move's at most the margins before 2026-10-17 (its overshoot is blind). The
   * margins in force at acceptance (auto_motion.h "guard margins"; the values before 2026-10-17 give the same floats
   * as the constants did: n / 100.0f is the literal's rounding). */
  const float progM = (float)mvMg.progCm / 100.0f, progRel = (float)mvMg.progPct / 100.0f;
  const float fwdPm = fwdM < 0.0f ? minf2(progM, AUTO_MV_PROG_MARGIN_M) : progM;
  const float fwdPr = fwdM < 0.0f ? minf2(progRel, AUTO_MV_PROG_MARGIN_REL) : progRel;
  /* the forward margin's metres at the move's end distance: toward the scene its nearest (D0 - F); a back move's (mvfix
   * LONG, 2026-10-17) its farthest, D0 + |F| (at its start distance the 300 % back move's margin was ~0.9 m at its
   * end), else its start distance as before */
  const float dMarg = (fwdM < 0.0f && (mvMg.fix & AUTO_MVFIX_LONG) != 0u) ? d - fwdM : d - maxf2(fwdM, 0.0f);
  m->margLatM = horiz ? maxf2((float)mvMg.latCm / 100.0f, progM + progRel * absf(latM)) : 0.0f;
  m->margFwd = horiz ? fwdPm / maxf2(dMarg, AUTO_MV_D_MIN_M) + fwdPr * absf(m->fwdTgt) : 0.0f;
  if (sp.fast && horiz) {
    /* the hold's lead grows with the speed (AUTO_MV_FAST_T_TRACK_S x the peak): the margins grow with it */
    const float lead = sp.tTrack * ph.vPk;
    m->margLatM += lead * absf(latM) / len;
    m->margFwd += lead * absf(fwdM) / len / maxf2(dMarg, AUTO_MV_LAT_GUARD_D_MIN_M);
  }
  /* the progress guard off (mvprogm 0 or mvprogr 0; auto_motion.h "guards off"): margins that never trip, and no
   * frozen-view rule (the step's frozenRef; back to the speed step's value once the guard is on again, as
   * autoMotionSetSpeed set it; untouched otherwise) */
  if (horiz && progOff()) {
    m->margLatM = m->margFwd = AUTO_MV_GUARD_OFF;
  }
  if (progOff()) {
    m->frozenRef = AUTO_MV_GUARD_OFF;
  } else if (m->frozenRef >= AUTO_MV_GUARD_OFF) {
    m->frozenRef = sp.frozenRef;
  }
  /* the live front guard: the scene ahead at mvfstop for the lowest expansion scale (forward moves; with mvfix FRONT,
   * the review of 2026-10-17, every horizontal move: a sideways or back move from near the floor is pushed toward
   * the scene as easily); mvfstop 0: none */
  m->progFwdMax = (mvMg.stopCm != 0u && (fwdM > 0.0f || ((mvMg.fix & AUTO_MVFIX_FRONT) != 0u && horiz && ds.low > 0.0f)))
                      ? kLow() * autoMvLn(ds.low / ((float)mvMg.stopCm / 100.0f))
                      : AUTO_MV_GUARD_OFF;
  /* the sideways guard's forward unit (auto_motion.h: max(sFwdM, 0); a back move keeps the start distance) */
  m->fwdUnitPos = horiz ? maxf2(fwdM, 0.0f) / len : 0.0f;
  m->dLowFwd = fwdM > 0.0f ? ds.low : 0.0f;   /* the camera's gain scale during a move toward the scene */
  /* the lateral shift since the last fresh view reaches the observer's view change: a re-view bob at the end */
  m->review = horiz && in->flight && !in->simPlant && m->viewShift + absf(latM) / dl >= AUTO_MV_VIEW_SHIFT_REL;
  m->upM = upM;
  m->upDone = u == 0;
  m->pv = pv;
  m->hStart = in->vzH;
  m->hOffNow = m->hOff;
  m->yawCmd = 0.0f;
  m->bob = false;
  autoMvProfPlan(&m->pb, AUTO_MV_BOB_A_M, AUTO_MV_V_UP_MAX, sp.bobAcc);
  accept(m, in, AUTO_MVK_MOVE);
  /* mvfix STILL: a horizontal move flown on the camera waits at its start for a still drone (the step, at most
   * AUTO_MV_STILL_CAP_MS) */
  m->stillWait = (mvMg.fix & AUTO_MVFIX_STILL) != 0u && horiz && in->flight && !in->simPlant;
  if (m->stillWait) {
    m->stillMs = (uint16_t)in->nowMs;   /* (the cap from the acceptance, not from the latest PAUSED -> RUN) */
  }
  return AUTO_MVW_NONE;
}

AUTO_COLD autoMvWhy_t autoMotionMove(autoMotion_t *m, const autoMvIn_t *in, int16_t rightCm, int16_t fwdCm,
                                     int16_t upCm)
{
  return moveFloor(m, in, rightCm, fwdCm, upCm, (float)mvMg.floorCm * 0.01f);
}

AUTO_COLD autoMvWhy_t autoMotionMovePlan(autoMotion_t *m, const autoMvIn_t *in, int16_t rightCm, int16_t fwdCm,
                                         int16_t upCm)
{
  return moveFloor(m, in, rightCm, fwdCm, upCm, (float)mvMg.planCm * 0.01f);
}

/* SET_HEIGHT's correction (auto_motion.h, auto_fsm.h AUTO_SH_*): the set-up of moveFloor()'s vertical-only branch,
 * with the committed offset moved into the corrected frame first and the start at the corrected estimate. Once per
 * command: flash. */
AUTO_COLD autoMvWhy_t autoMotionSetHeight(autoMotion_t *m, const autoMvIn_t *in, float d, float upM)
{
  return autoMotionSetHeightRef(m, in, d, upM, d);
}

AUTO_COLD autoMvWhy_t autoMotionSetHeightRef(autoMotion_t *m, const autoMvIn_t *in, float d, float upM, float refShift)
{
  if (autoMotionActive(m)) {
    return refuse(m, AUTO_MVW_BUSY);
  }
  if (!viewOk(in)) {
    return refuse(m, AUTO_MVW_NO_VIEW);   /* the hold holds the drone during any move, vertical too */
  }
  if (!in->camHeight) {
    return refuse(m, AUTO_MVW_NO_CAM_HEIGHT);   /* any mode: in open loop the controller ignores vRef */
  }
  if (in->battLow) {
    return refuse(m, AUTO_MVW_BATT);   /* near the end of the battery no correction flies, a descending one included
                                          (precheck(): its braking may end HOLD_CAPPED) */
  }
  if (!finitef2(d) || !finitef2(upM) || !finitef2(refShift) || !finitef2(in->vzH)) {
    return refuse(m, AUTO_MVW_RANGE);
  }
  autoMotionTrackNow(m, in);   /* mvfix DRIFT: before the new epoch */
  m->d0 = m->dLat0 = m->dLow0 = 0.0f;
  m->latM = m->fwdM = m->lenM = 0.0f;
  memset(&m->ph, 0, sizeof(m->ph));
  m->latTgt = m->fwdTgt = 0.0f;
  m->margLatM = m->margFwd = 0.0f;
  m->progFwdMax = 1e30f;     /* no front guard (a forward MOVE's) */
  m->fwdUnitPos = 0.0f;
  m->dLowFwd = 0.0f;
  m->review = false;
  /* the reference into the corrected frame: hMax + hOff keeps its margin over the shifted estimate (the target form:
   * the reference at the corrected estimate itself) */
  m->shOff0 = m->hOff;
  m->shD = refShift;
  m->hOff = clampf2(m->hOff + refShift, -AUTO_MV_HOFF_MAX, AUTO_MV_HOFF_MAX);
  m->upM = upM;
  m->upDone = upM == 0.0f;
  autoMvProfPlan(&m->pv, absf(upM), AUTO_MV_V_UP_MAX, AUTO_MV_A_UP);
  m->hStart = in->vzH + d;   /* the estimate the glue shifts on this step */
  m->hOffNow = m->hOff;
  m->yawCmd = 0.0f;
  m->bob = false;
  {
    autoMvSpeed_t sp;
    autoMvSpeedCfg(m->speedPct, &sp);
    autoMvProfPlan(&m->pb, AUTO_MV_BOB_A_M, AUTO_MV_V_UP_MAX, sp.bobAcc);
  }
  accept(m, in, AUTO_MVK_MOVE);
  m->yawFlags = AUTO_MVF_SET_HEIGHT;   /* (out->flags carries it: not Claude's MOVE) */
  return AUTO_MVW_NONE;
}

AUTO_COLD autoMvWhy_t autoMotionYaw(autoMotion_t *m, const autoMvIn_t *in, int16_t deg)
{
  autoMvWhy_t w = precheck(m, in);
  if (w != AUTO_MVW_NONE) {
    return w;
  }
  if (deg == 0 || deg > AUTO_MV_YAW_MAX_DEG || deg < -AUTO_MV_YAW_MAX_DEG) {
    return refuse(m, AUTO_MVW_RANGE);
  }
  /* without a view: a blind turn up to AUTO_MV_BLIND_YAW_MAX_DEG (toward texture; 2026-10-15) */
  const bool blind = !viewOk(in);
  if (blind && (deg > AUTO_MV_BLIND_YAW_MAX_DEG || deg < -AUTO_MV_BLIND_YAW_MAX_DEG)) {
    return refuse(m, AUTO_MVW_NO_VIEW);
  }
  autoMvSpeed_t sp;
  autoMvSpeedCfg(m->speedPct, &sp);
  autoMvProfPlan(&m->py, absf((float)deg), sp.yawRate, sp.yawAcc);
  m->blind = blind;
  m->undo = false;
  m->yawOff0 = m->yawOff;
  m->yawCmd = (float)deg;
  autoMvProfPlan(&m->pb, AUTO_MV_BOB_A_M, AUTO_MV_V_UP_MAX, sp.bobAcc);
  m->yawOffNow = m->yawOff;
  m->turnDone = false;
  m->bob = false;
  m->bobT = 0.0f;
  m->latM = m->fwdM = m->lenM = m->upM = 0.0f;
  m->upDone = true;
  m->review = false;
  m->latTgt = m->fwdTgt = 0.0f;
  m->progFwdMax = 1e30f;     /* no front guard in a turn (it is a MOVE's) */
  m->dLowFwd = 0.0f;
  memset(&m->ph, 0, sizeof(m->ph));
  memset(&m->pv, 0, sizeof(m->pv));
  m->hStart = in->vzH;
  m->hOffNow = m->hOff;
  m->dStale = true;          /* a new view: D until the tracker has a G again */
  m->dTrack = 0.0f;          /* the last forward move's D belongs to the old view */
  m->dTrackLow = 0.0f;
  accept(m, in, AUTO_MVK_YAW);
  m->yawFlags = blind ? AUTO_MVF_BLIND_TURN : 0;
  return AUTO_MVW_NONE;
}

/* Ends a turn where it is: the yaw offset committed, the hold re-captured
 * (the scene changed), then the dwell and the re-track check. Once per turn:
 * flash (since 2026-10-13 evening: its 99 B back in IRAM would cross the
 * text's 256-byte step, which has 61 B to spare). */
static AUTO_COLD void endTurn(autoMotion_t *m, uint32_t now)
{
  m->yawOff = m->yawOffNow;
  /* keep it in -180..180 (the yaw lock wraps it anyway) */
  if (m->yawOff > 180.0f) {
    m->yawOff -= 360.0f;
  } else if (m->yawOff < -180.0f) {
    m->yawOff += 360.0f;
  }
  m->yawOffNow = m->yawOff;
  m->turnDone = true;
  newEpoch(m, true);
  setState(m, AUTO_MV_SETTLE, now);
}

/* The vertical offset reached (measured), committed for hMax and the LAND
 * times. Once per motion (its end, a STOP): flash. */
static AUTO_COLD void commitUp(autoMotion_t *m, const autoMvIn_t *in, bool measured)
{
  float dh = measured ? in->vzH - m->hStart : m->upM;
  if (!finitef2(dh)) {
    dh = 0.0f;
  }
  if (!measured && (m->yawFlags & AUTO_MVF_SET_HEIGHT) != 0u) {
    /* a SET_HEIGHT's correction flown: the offset before it plus d + upM (0 for the usual upM = -d: the reference
     * exactly where it was, the drone where it believed it held) */
    m->hOff = clampf2(m->shOff0 + (m->shD + m->upM), -AUTO_MV_HOFF_MAX, AUTO_MV_HOFF_MAX);
  } else {
    m->hOff = clampf2(m->hOff + dh, -AUTO_MV_HOFF_MAX, AUTO_MV_HOFF_MAX);
  }
  m->hOffNow = m->hOff;
  m->upDone = true;
}

static AUTO_HOT void finish(autoMotion_t *m, autoMvState_t s, autoMvEnd_t e, uint32_t now)
{
  m->end = e;
  m->bob = false;
  setState(m, s, now);
}

/* mvfix DRIFT (auto_motion.h "move fixes"): dTrack (and its lower bound) moved by the hold's measured forward
 * progress p of the current epoch since dTrackRef: x exp(-(p - ref)) at k 1, the lower bound x exp(-(p - ref) / K_LOW)
 * toward the scene (x exp(-(p - ref)) away from it: k <= 1), the reference then p. Flash. */
static AUTO_COLD void trackShift(autoMotion_t *m, float p)
{
  const float dp = p - m->dTrackRef;
  if (!(m->dTrack > 0.0f) || !finitef2(dp)) {
    return;
  }
  const float dNow = m->dTrack * expf(-dp);
  const float dLow = (m->dTrackLow > 0.0f ? m->dTrackLow : m->dTrack) *
                     expf(dp > 0.0f ? -dp * (1.0f / kLow()) : -dp);
  if (finitef2(dNow) && dNow > 0.0f && finitef2(dLow)) {
    m->dTrack = dNow;
    m->dTrackLow = minf2(dLow, dNow);
    m->dTrackRef = p;
  }
}

AUTO_COLD void autoMotionTrackNow(autoMotion_t *m, const autoMvIn_t *in)
{
  if ((mvMg.fix & AUTO_MVFIX_DRIFT) != 0u && in->flight && !in->simPlant && in->holdEpochSeen == m->epoch &&
      !autoMotionActive(m)) {
    trackShift(m, in->progFwd);
  }
}

/* After a forward / back move: the D its measured expansion gives (k = 1;
 * DRY and BENCH: the ideal progress, the commanded D) and the lower bound
 * (forward: k >= AUTO_MV_K_LOW from the lower bound at acceptance); its
 * reference the move's measured progress (mvfix DRIFT). A level sideways or
 * a vertical move keeps the D of before, moved by its own measured forward
 * progress with mvfix DRIFT. */
static AUTO_COLD void trackD(autoMotion_t *m)
{
  if (m->fwdM == 0.0f || !(m->d0 > 0.0f)) {
    if ((mvMg.fix & AUTO_MVFIX_DRIFT) != 0u) {
      trackShift(m, m->progFwd);
    }
    return;
  }
  const float dNow = m->d0 * expf(-m->progFwd);
  const float dLow = m->progFwd > 0.0f ? m->dLow0 * expf(-m->progFwd * (1.0f / kLow())) : m->dLow0;
  if (finitef2(dNow) && dNow > 0.0f && finitef2(dLow)) {
    m->dTrack = dNow;
    m->dTrackLow = minf2(dLow, dNow);
    m->dTrackRef = m->progFwd;
  }
}

/* A YAW turned back to its start heading (2026-10-15, docs/autonomy-a2b.md 22): the turned-to view gave no scene
 * distance (its re-track bob ended without a G), or its view or a contact stopped it. The same seq, a turn of the
 * angle turned so far back, exempt from the view guard as a blind turn (it goes back to the view it had); then the
 * dwell and the re-track bob as after any turn, whose guess is the old view's G; the end is UNDONE (a second failure
 * ends it RETRACK / STOPPED). Flash: once per YAW. */
static AUTO_COLD void turnBack(autoMotion_t *m, uint32_t now)
{
  if (!m->turnDone) {
    endTurn(m, now);
  }
  float back = m->yawOff0 - m->yawOff;
  if (back > 180.0f) {
    back -= 360.0f;
  } else if (back < -180.0f) {
    back += 360.0f;
  }
  m->undo = true;
  m->blind = true;
  m->yawFlags |= AUTO_MVF_UNDO;
  m->doneEnd = AUTO_MVE_UNDONE;
  m->yawCmd = back;
  autoMvSpeed_t sp;
  autoMvSpeedCfg(m->speedPct, &sp);
  autoMvProfPlan(&m->py, absf(back), sp.yawRate, sp.yawAcc);
  m->yawOffNow = m->yawOff;
  m->turnDone = !(absf(back) > 0.5f);
  m->bob = false;
  m->bobT = 0.0f;
  m->tRun = 0.0f;
  m->viewBad = false;
  m->tiltOn = false;
  m->dStale = true;
  setState(m, m->turnDone ? AUTO_MV_SETTLE : AUTO_MV_RUN, now);
}

/* A turn's re-track bob ended without a scene distance (flash, once): back to the start heading once (FLIGHT, the
 * camera; AUTO_MVL_UNDO, the state machine reports it), else DONE RETRACK. */
static AUTO_COLD autoMvLand_t retrackEnd(autoMotion_t *m, const autoMvIn_t *in)
{
  if (!m->undo && in->flight && !in->simPlant) {
    turnBack(m, in->nowMs);
    return AUTO_MVL_UNDO;
  }
  finish(m, AUTO_MV_DONE, AUTO_MVE_RETRACK, in->nowMs);
  return AUTO_MVL_NONE;
}

AUTO_COLD void autoMotionSnap(autoMotion_t *m)
{
  m->brakeS = 0.0f;
  m->backOff = false;
  newEpoch(m, true);
}

/* A MOVE stopped by an in-air contact (auto_motion.h AUTO_MV_CONTACT_BACK_M): the hold's anchor in the stop's snap
 * epoch goes AUTO_MV_CONTACT_BACK_M back along the move's horizontal direction, the way it came, instead of staying
 * at the contact point. In the hold's units from where the drone is at the stop: lateral metres / D, forward
 * ln(D_stop / D_anchor), with D_stop the move's D scaled by its measured forward progress (ln(D0 / D_now)). A back
 * move's anchor goes toward the scene ahead: at most to the move floor (mvfloor) from it. Flash: once per contact. */
static AUTO_COLD void contactBackOff(autoMotion_t *m)
{
  const float ux = m->latM / m->lenM, uy = m->fwdM / m->lenM;
  const float shrink = finitef2(m->progFwd) ? expf(-m->progFwd) : 1.0f;
  const float dF = m->d0 * shrink, dL = m->dLat0 * shrink;
  const float back = AUTO_MV_CONTACT_BACK_M;
  float lat = 0.0f, fwd = 0.0f;
  if (finitef2(dL) && dL >= AUTO_MV_D_MIN_M * 0.5f) {
    lat = -back * ux / dL;
  }
  if (finitef2(dF) && dF >= AUTO_MV_D_MIN_M * 0.5f) {
    /* the anchor's metres toward the scene ahead (+: a back move's anchor, ahead of the stop) */
    const float ahead = minf2(-back * uy, maxf2(0.0f, dF - (float)mvMg.floorCm * 0.01f));
    fwd = autoMvLn(dF / (dF - ahead));
  }
  if (finitef2(lat) && finitef2(fwd) && (lat != 0.0f || fwd != 0.0f)) {
    m->latTgt = lat;
    m->fwdTgt = fwd;
    m->backOff = true;
  }
}

AUTO_COLD void autoMotionStop(autoMotion_t *m, const autoMvIn_t *in, autoMvEnd_t why)
{
  if (!autoMotionActive(m)) {
    return;
  }
  const uint32_t now = in->nowMs;
  if (m->kind == AUTO_MVK_YAW) {
    /* a contact or the view lost: back to the start heading, once (FLIGHT, the camera; the sim has no view) */
    if ((why == AUTO_MVE_CONTACT || why == AUTO_MVE_NO_VIEW) && !m->undo && in->flight && !in->simPlant) {
      turnBack(m, now);
      return;
    }
    if (!m->turnDone) {
      endTurn(m, now);   /* the turn ends where it is; the dwell and the re-track run on ... */
    }
    if (why == AUTO_MVE_STOP_CMD || why == AUTO_MVE_TILT || why == AUTO_MVE_LOOP_GAP) {
      m->tiltOn = false;   /* ... a STOP, a disturbance or a loop gap never ends them (the height estimate needs the
                              tracker's G: its guess stays allowed while the YAW is active) */
      return;
    }
    finish(m, AUTO_MV_STOPPED, why, now);   /* leaving HOVER, a second failure */
    return;
  }
  if (m->bob && why == AUTO_MVE_STOP_CMD) {
    return;              /* the move is done; its re-view bob runs on (<= ~4 s) */
  }
  if (why == AUTO_MVE_PROGRESS && m->progFwd > m->progFwdMax) {
    why = AUTO_MVE_FRONT;   /* the step's guard (one branch for both, IRAM): past the live front stop */
  }
  /* MOVE: hold where it is, braking from the reference's speed */
  m->brakeS = 0.0f;
  if (!(m->lenM > 0.0f)) {
    trackD(m);   /* a vertical move: its forward drift (mvfix DRIFT; nothing without it) */
  }
  if (m->lenM > 0.0f) {
    float s, v, a;
    autoMvProfEval(&m->ph, m->tRun, &s, &v, &a);
    trackD(m);
    if (m->state == AUTO_MV_RUN && v > 0.0f) {
      m->brakeLat = v * m->latM / m->lenM;
      m->brakeFwd = v * m->fwdM / m->lenM;
      m->brakeS = v / m->brakeAcc;
    }
    if (!m->bob) {
      m->viewShift += absf(m->progLat);   /* the lateral shift so far, in D */
    }
  }
  if (!m->upDone) {
    commitUp(m, in, true);
  }
  m->review = false;
  newEpoch(m, true);
  m->backOff = false;
  if (why == AUTO_MVE_CONTACT && m->lenM > 0.0f) {
    contactBackOff(m);   /* the hold's anchor off what was hit (2026-10-15 verification) */
  }
  finish(m, AUTO_MV_STOPPED, why, now);
}

/* A RUN that just lost its view (the step's RUN -> PAUSED; flash: once per view loss, out of the IRAM step): the pause
 * of before. At mvspeed > 100 (auto_motion.h "fast moves") a MOVE's pause is armed for the VIEW_FAST check
 * (pausedFast) AUTO_MV_FAST_VIEW_DEB_MS later. */
/* A forward MOVE whose last measured progress puts the scene ahead within the near-scene distance (mvnear) at the lowest
 * expansion scale (auto_motion.h "guard margins", 2026-10-17): its view lost there is the camera's expansion gate
 * refusing a fast approach (or anything else close ahead), so its pause brakes (pausedFast). With mvfix FRONT (the
 * review of 2026-10-17) any horizontal MOVE's: a sideways or back one near the scene coasts toward it as easily. 0
 * mvnear: never. Flash. */
static AUTO_COLD bool nearScene(const autoMotion_t *m)
{
  return mvMg.nearCm > 0u && m->kind == AUTO_MVK_MOVE && (m->fwdM > 0.0f || (mvMg.fix & AUTO_MVFIX_FRONT) != 0u) &&
         m->lenM > 0.0f && !m->bob &&
         m->dLow0 > 0.0f && m->progFwd > kLow() * autoMvLn(m->dLow0 / ((float)mvMg.nearCm / 100.0f));
}

static AUTO_COLD autoMvLand_t runViewLost(autoMotion_t *m, const autoMvIn_t *in, autoMvLand_t land)
{
  m->viewFastArm = (m->speedPct > AUTO_MV_SPEED_DEFAULT_PCT && m->kind == AUTO_MVK_MOVE && m->lenM > 0.0f && !m->bob) ||
                   nearScene(m);
  setState(m, AUTO_MV_PAUSED, in->nowMs);
  return land;
}

/* A fast MOVE still paused AUTO_MV_FAST_VIEW_DEB_MS after its view loss (the step, once per armed pause; flash): with no
 * other stop request on the step and the reference faster than AUTO_MV_FAST_PAUSE_V at the pause or
 * AUTO_MV_FAST_T_TRACK_S before it (the hold's lag: a loss on the profile's slowing part, the drone still faster than
 * its reference), stopped with its brake from that speed (VIEW_FAST: the pause would freeze the reference and coast on
 * drag alone, ~1.7 m from 0.5 m/s): AUTO_MVL_FAST, the report. Otherwise the pause runs on as at 100 (NO_CAM after
 * AUTO_MV_NO_CAM_LAND_MS). A forward MOVE paused near the scene (nearScene; at every speed step, 2026-10-17) is stopped
 * so whatever its reference's speed, with its brake from at least AUTO_MV_FAST_PAUSE_V, end FRONT: the expansion gate
 * blinds the live front stop once the approach is faster than ~0.5 x the distance (k 0.6), and a pause there coasted
 * into the scene (docs/autonomy-a2b.md 32.2). Only a RUN pauses: a view lost in the settle ends NO_VIEW after
 * AUTO_MV_NO_CAM_LAND_MS as before. */
static AUTO_COLD autoMvLand_t pausedFast(autoMotion_t *m, const autoMvIn_t *in, autoMvLand_t land)
{
  m->viewFastArm = false;
  if (land != AUTO_MVL_NONE || m->kind != AUTO_MVK_MOVE || !(m->lenM > 0.0f) || m->bob) {
    return land;
  }
  const bool near = nearScene(m);
  float s, v, v0, a;
  autoMvProfEval(&m->ph, m->tRun, &s, &v, &a);
  autoMvProfEval(&m->ph, maxf2(m->tRun - AUTO_MV_FAST_T_TRACK_S, 0.0f), &s, &v0, &a);
  v = maxf2(v, v0);
  if (near) {
    v = maxf2(v, AUTO_MV_FAST_PAUSE_V);
  } else if (!(v > AUTO_MV_FAST_PAUSE_V) || m->speedPct <= AUTO_MV_SPEED_DEFAULT_PCT) {
    return land;
  }
  autoMotionStop(m, in, near ? AUTO_MVE_FRONT : AUTO_MVE_VIEW_FAST);   /* PAUSED: no brake set there ... */
  if (m->state == AUTO_MV_STOPPED) {
    m->brakeLat = v * m->latM / m->lenM;       /* ... the brake from the lagged speed */
    m->brakeFwd = v * m->fwdM / m->lenM;
    m->brakeS = v / m->brakeAcc;
  }
  return AUTO_MVL_FAST;
}

/* ---- the step ------------------------------------------------------------------ */

/* The horizontal reference at profile time t: hold units and metric (out of
 * line: the step takes it twice). sFwdM: its forward metres toward the scene
 * (0 for a back move: the sideways guard keeps the start distance there). */
static AUTO_HOT __attribute__((noinline)) void horizAt(const autoMotion_t *m, float t, float *rLat, float *rFwd,
                                                       float *wLat, float *wFwd, float *aLat, float *aFwd, float *vLat,
                                                       float *vFwd, float *sFwdM)
{
  float s, v, a;
  autoMvProfEval(&m->ph, t, &s, &v, &a);
  const float ux = m->latM / m->lenM, uy = m->fwdM / m->lenM;
  const float dNow = m->d0 - s * uy;
  *rLat = s * ux / m->dLat0;
  *wLat = v * ux / m->dLat0;
  *rFwd = autoMvLn(m->d0 / dNow);
  *wFwd = v * uy / dNow;
  *aLat = a * ux;
  *aFwd = a * uy;
  *vLat = v * ux;
  *vFwd = v * uy;
  *sFwdM = s * m->fwdUnitPos;
}

/* The bob's vertical position at bob time t: legs of +A then -A, each a
 * triangle at BOB_ACC (m->pb); returns the leg count started. */
static AUTO_HOT int bobAt(const autoMotion_t *m, float t, float *z, float *vz)
{
  const autoMvProf_t *p = &m->pb;
  if (!(p->T > 0.0f) || t < 0.0f) {
    *z = 0.0f;
    *vz = 0.0f;
    return 0;
  }
  const int leg = (int)(t / p->T);
  const float tl = t - (float)leg * p->T;
  float s, v, a;
  autoMvProfEval(p, tl, &s, &v, &a);
  if (leg % 2 == 0) {
    *z = s;
    *vz = v;
  } else {
    *z = AUTO_MV_BOB_A_M - s;
    *vz = -v;
  }
  return leg;
}

/* The bob (a turn's re-track, a move's re-view) from here: z from the height estimate now. */
static AUTO_HOT void bobStart(autoMotion_t *m, const autoMvIn_t *in)
{
  m->bob = true;
  m->bobT = 0.0f;
  m->bobLeg = 0;
  m->bobTrackedLeg = -1;
  m->bobH0 = finitef2(in->vzH) ? in->vzH : 0.0f;
}

/* One bob step: its vertical speed reference (running), its height so far
 * (m->progUp for a turn's); returns the leg, *cycleEnd when a whole cycle
 * just ended (back at the start height). */
static AUTO_HOT int bobStep(autoMotion_t *m, const autoMvIn_t *in, bool running, float dt, float *vRef,
                            bool *cycleEnd)
{
  if (running) {
    m->bobT += dt;
  }
  float z, vz;
  const int leg = bobAt(m, m->bobT, &z, &vz);
  const float pu = finitef2(in->vzH) ? in->vzH - m->bobH0 : 0.0f;
  if (m->kind == AUTO_MVK_YAW) {
    m->progUp = pu;
  }
  if (running) {
    *vRef = clampf2(vz + AUTO_MV_UP_KP * (z - pu), -AUTO_MV_UP_VREF_MAX, AUTO_MV_UP_VREF_MAX);
  }
  *cycleEnd = leg != m->bobLeg && leg % 2 == 0;
  m->bobLeg = leg;
  return leg;
}

/* A MOVE's settle ended (once per move: flash): the D it measured, the
 * height offset, then a STOP for a frozen view, the re-view bob, or DONE
 * (SHORT when it timed out short of the target or off its line). settled:
 * within the settle tolerance on a frame of this epoch. */
static AUTO_COLD void settleEnd(autoMotion_t *m, const autoMvIn_t *in, float pR, float pF, bool settled)
{
  const uint32_t now = in->nowMs;
  trackD(m);
  if (!m->upDone) {
    commitUp(m, in, false);   /* the commanded offset (the estimate drifts) */
  }
  /* along the move: a settle that timed out short of half of it saw no
   * motion (a frozen view): its target must not stay in the hold */
  const float along = m->lenM > 0.0f ? (pR * m->latM + pF * m->fwdM) / (m->lenM * m->lenM) : 1.0f;
  if (in->flight && m->lenM > 0.0f && !(along >= AUTO_MV_SETTLE_MIN_PROG)) {
    autoMotionStop(m, in, AUTO_MVE_PROGRESS);
    return;
  }
  /* Timed out short of the target along the move by more than the settle
   * tolerance, or off the move's line by more than the RUN guard's base
   * margin mvprogm (the settle's lateral margin is wider; user decision 2026-10-13,
   * evening): no fault, but the hold must not push on toward a target the
   * camera may never confirm (k < 1, D off): the measured end becomes the
   * move's end, so the drone holds where it is (the hold's integral as a
   * snap leaves it), and the move ends SHORT. Off its line: the cross-track
   * error at the target, the sideways error in metres at the distance now as
   * the guard reads it (2026-10-14: at the start distance a long forward
   * move's 0.06 m read 0.17 m at its end, D0 / D_now = 2.8). */
  const float off = m->lenM > 0.0f ? absf((m->progLat - m->latTgt) * (m->dLat0 - m->fwdUnitPos * m->lenM) * m->fwdM -
                                          (pF - m->fwdM) * m->latM) / m->lenM
                                   : 0.0f;
  /* mvfix LONG (2026-10-17): timed out past the target along the move by more than the settle tolerance: its measured
   * end is its end too (the drone holds there; the 300 % back move reported DONE 0.2 m past, and the hold then walked
   * it back unreported), end LONG (before SHORT's off-the-line test: the overshoot is the news) */
  if (in->flight && m->lenM > 0.0f && !settled) {
    if ((mvMg.fix & AUTO_MVFIX_LONG) != 0u &&
        along * m->lenM > m->lenM + AUTO_MV_SETTLE_TOL_M + AUTO_MV_SETTLE_TOL * m->lenM) {
      /* a move with a part toward the scene keeps its target (the review of 2026-10-17: held at the overshoot, an
       * unlearned forward lean crept it ~0.1 m further toward the scene or a person; the hold walks it back, and its
       * D follows it there with DRIFT); any other holds where it is */
      if (!(m->fwdM > 0.0f)) {
        m->latTgt = m->progLat;
        m->fwdTgt = m->progFwd;
      }
      m->doneEnd = AUTO_MVE_LONG;
    } else if (along * m->lenM < m->lenM - AUTO_MV_SETTLE_TOL_M - AUTO_MV_SETTLE_TOL * m->lenM ||
               (mvMg.progCm != 0u && off > (float)mvMg.progCm / 100.0f)) {   /* (mvprogm 0: no off-the-line SHORT) */
      m->latTgt = m->progLat;
      m->fwdTgt = m->progFwd;
      m->shortEnd = true;
    }
  }
  if (m->review) {
    bobStart(m, in);
    setState(m, AUTO_MV_SETTLE, now);
  } else {
    m->viewShift += absf(m->latTgt);
    finish(m, AUTO_MV_DONE, m->shortEnd ? AUTO_MVE_SHORT : (autoMvEnd_t)m->doneEnd, now);   /* (doneEnd: DONE, LONG) */
  }
}

AUTO_HOT autoMvLand_t autoMotionStep(autoMotion_t *m, const autoMvIn_t *in, autoMvOut_t *out)
{
  const uint32_t now = in->nowMs;
  autoMvLand_t land = AUTO_MVL_NONE;
  memset(out, 0, sizeof(*out));
  const float dt = m->started ? (float)(now - m->lastMs) * 0.001f : 0.0f;
  m->lastMs = now;
  m->started = true;

  /* A small commanded turn can finish before residual yaw crosses the
   * observer's accumulated-view threshold (recorded QVGA flight5). Reopen
   * that completed turn's existing bounded re-track, without turning again.
   * A real view loss cannot start it, and all view/height/battery guards and
   * the FSM's loss-of-height fallback still run below. Once per command. */
  if (m->kind == AUTO_MVK_YAW && m->state == AUTO_MV_DONE && m->end == AUTO_MVE_DONE &&
      !m->lateYawRecoveryUsed && in->sceneResets != m->yawResetSeen && !in->sceneTracked &&
      in->flight && !in->simPlant && !in->battLow && viewOk(in) && in->tiltDeg <= AUTO_MV_TILT_LAND_DEG) {
    m->lateYawRecoveryUsed = true;
    m->dStale = true;
    m->end = AUTO_MVE_NONE;
    setState(m, AUTO_MV_SETTLE, now);
    bobStart(m, in);
  }
  m->yawResetSeen = in->sceneResets;

  /* The latest measured progress of this epoch (FLIGHT, once the camera has
   * seen it), while the motion runs: an ended MOVE keeps its travel. */
  if (in->flight && in->holdEpochSeen == m->epoch && autoMotionActive(m) && finitef2(in->progLat) &&
      finitef2(in->progFwd)) {
    m->progLat = in->progLat;
    m->progFwd = in->progFwd;
  }
  if (m->dStale && !(m->kind == AUTO_MVK_YAW && autoMotionActive(m)) && in->sceneTracked) {
    m->dStale = false;   /* the tracker has a G of the new view */
  }

  float rLat = 0.0f, rFwd = 0.0f, wLat = 0.0f, wFwd = 0.0f, ffR = 0.0f, ffP = 0.0f, vRef = 0.0f;
  if (autoMotionActive(m)) {
    /* guards */
    const bool vertNeeded = m->bob || (m->kind == AUTO_MVK_MOVE && !m->upDone);
    /* a blind turn's turn (or a turn back) needs no view: it goes to (or back to) one */
    const bool ok = (viewOk(in) || (m->blind && !m->turnDone)) && (!vertNeeded || !in->flight || in->camHeight);
    if (ok) {
      m->viewBad = false;
    } else if (!m->viewBad) {
      m->viewBad = true;
      m->viewBadMs = now;
    }
    if (m->viewBad && now - m->viewBadMs >= AUTO_MV_NO_CAM_LAND_MS) {
      land = AUTO_MVL_NO_CAM;
    }
    if (in->tiltDeg > AUTO_MV_TILT_LAND_DEG || !(in->tiltDeg == in->tiltDeg)) {
      if (!m->tiltOn) {
        m->tiltOn = true;
        m->tiltMs = now;
      }
      if (now - m->tiltMs >= AUTO_MV_TILT_LAND_MS) {
        land = AUTO_MVL_TILT;
      }
    } else {
      m->tiltOn = false;
    }
    if (m->state == AUTO_MV_RUN && !ok) {
      land = runViewLost(m, in, land);   /* PAUSED (flash); a fast MOVE's pause armed for VIEW_FAST */
    } else if (m->state == AUTO_MV_PAUSED && ok) {
      setState(m, AUTO_MV_RUN, now);
    } else if (m->viewFastArm && m->state == AUTO_MV_PAUSED &&
               now - m->stateMs >= (nearScene(m) ? AUTO_MV_FRONT_VIEW_DEB_MS : (uint32_t)mvFast.viewDebMs)) {
      land = pausedFast(m, in, land);    /* still lost: VIEW_FAST (the timer: mvview), stopped with its brake (flash;
                                            never at 100) */
    }
    const bool running = ok && (m->state == AUTO_MV_RUN || m->state == AUTO_MV_SETTLE);

    if (m->kind == AUTO_MVK_MOVE) {
      if (m->state == AUTO_MV_RUN) {
        if (!m->stillWait) {
          m->tRun += dt;
        } else if (!in->holdMoving || (uint16_t)((uint16_t)now - m->stillMs) >= AUTO_MV_STILL_CAP_MS) {
          m->stillWait = false;   /* mvfix STILL: the drone is still (or the cap): the profile starts */
        }
      }
      const float tProf = maxf2(m->ph.T, m->pv.T);
      if (m->state == AUTO_MV_RUN && m->tRun >= tProf) {
        m->tRun = tProf;
        setState(m, AUTO_MV_SETTLE, now);
      }
      const float lagS = in->camLagUs > 0u && in->camLagUs <= 600000u ?
                         (float)in->camLagUs * 1e-6f : (float)AUTO_MV_CAM_LAG_MS * 0.001f;
      const float tLag = m->tRun - lagS;
      float sFwdM = 0.0f, sFwdNowM = 0.0f;
      float idealLat = 0.0f, idealFwd = 0.0f;
      if (m->lenM > 0.0f) {
        float aL, aF, vL, vF, rl, rf, wl, wf;
        horizAt(m, tLag, &rLat, &rFwd, &wLat, &wFwd, &aL, &aF, &vL, &vF, &sFwdM);
        horizAt(m, m->tRun, &rl, &rf, &wl, &wf, &aL, &aF, &vL, &vF, &sFwdNowM);
        idealLat = rl;
        idealFwd = rf;
        if (running && m->state == AUTO_MV_RUN) {
          /* lean for the acceleration and the drag: +roll accelerates right, -pitch forward; the roll also the
           * feedforward per forward speed (mvffr, auto_motion.h autoMotionSetFfRoll; 0: adds 0) */
          ffR = (aL + AUTO_MV_DRAG_PER_S * vL) / AUTO_MV_G * 57.29578f + (float)mvFast.ffRollD * (0.1f * vF);
          ffP = -(aF + AUTO_MV_DRAG_PER_S * vF) / AUTO_MV_G * 57.29578f;
        }
        if (m->state != AUTO_MV_RUN) {
          wLat = 0.0f;
          wFwd = 0.0f;
        }
        if (m->state == AUTO_MV_SETTLE) {
          /* the profile is over: the whole move (the lagged reference is within 1 mm of it) */
          rLat = idealLat = m->latTgt;
          rFwd = idealFwd = m->fwdTgt;
        }
      }
      if (!in->flight) {
        m->progLat = idealLat;   /* DRY, BENCH: as if tracked perfectly */
        m->progFwd = idealFwd;
      }
      /* vertical */
      float z = 0.0f, vz = 0.0f, az;
      if (m->upM != 0.0f && !m->upDone) {
        autoMvProfEval(&m->pv, m->tRun, &z, &vz, &az);
        const float sg = m->upM > 0.0f ? 1.0f : -1.0f;
        z *= sg;
        vz *= sg;
        m->hOffNow = m->hOff + z;
        m->progUp = finitef2(in->vzH) ? in->vzH - m->hStart : 0.0f;
        if (running) {
          vRef = clampf2((m->state == AUTO_MV_RUN ? vz : 0.0f) + AUTO_MV_UP_KP * (z - m->progUp),
                         -AUTO_MV_UP_VREF_MAX, AUTO_MV_UP_VREF_MAX);
        }
      }
      const bool seen = !in->flight || in->holdEpochSeen == m->epoch;
      if (m->bob) {
        /* the re-view bob: the move is done (the hold keeps its end), the
         * tracker's G follows the new view's depth; D stale until it ends */
        bool cycleEnd;
        const int leg = bobStep(m, in, running, dt, &vRef, &cycleEnd);
        if (cycleEnd && leg >= 2 * AUTO_MV_REVIEW_CYCLES) {
          m->viewShift = 0.0f;
          m->dTrack = m->dTrackLow = 0.0f;   /* a fresh G of this view */
          m->review = false;
          finish(m, AUTO_MV_DONE, m->shortEnd ? AUTO_MVE_SHORT : (autoMvEnd_t)m->doneEnd, now);   /* (or LONG) */
          vRef = 0.0f;
        }
      } else if (in->flight && seen && m->state != AUTO_MV_PAUSED && m->lenM > 0.0f &&
                 (absf(m->progLat - rLat) * (m->dLat0 - sFwdM) > m->margLatM ||
                  absf(m->progFwd - rFwd) > m->margFwd || m->progFwd > m->progFwdMax ||
                  (rFwd > m->frozenRef && m->progFwd < AUTO_MV_FROZEN_FRAC * rFwd))) {
        /* progress guard (FLIGHT, the camera on this epoch): the measured
         * motion against the reference the frames are compared with (in the
         * settle the target itself), sideways in metres at the reference's
         * distance now (its floor kept at acceptance, auto_motion.h); the
         * live front guard (autoMotionStop ends it FRONT, in flash); a frozen
         * view on a forward move (AUTO_MV_FROZEN_*) */
        autoMotionStop(m, in, AUTO_MVE_PROGRESS);
      } else if (in->flight && !m->upDone && m->state != AUTO_MV_PAUSED &&
                 absf(m->progUp - (m->hOffNow - m->hOff)) > AUTO_MV_UP_MARGIN_M + 0.5f * absf(m->upM)) {
        autoMotionStop(m, in, AUTO_MVE_PROGRESS);
      } else if (m->state == AUTO_MV_SETTLE) {
        const float tol = AUTO_MV_SETTLE_TOL_M;
        const float pR = m->progLat * m->dLat0;
        const float pF = m->d0 > 0.0f ? m->d0 * (1.0f - expf(-m->progFwd)) : 0.0f;
        const float eLat = pR - m->latM;
        const float eFwd = m->d0 > 0.0f ? (m->progFwd - m->fwdTgt) * (m->d0 - m->fwdM) : 0.0f;
        const float eUp = m->progUp - m->upM;
        const bool settled = absf(eLat) <= tol + AUTO_MV_SETTLE_TOL * absf(m->latM) &&
                             absf(eFwd) <= tol + AUTO_MV_SETTLE_TOL * absf(m->fwdM) &&
                             (m->upM == 0.0f || absf(eUp) <= tol + AUTO_MV_SETTLE_TOL * absf(m->upM));
        if ((seen && settled) || now - m->stateMs >= (uint32_t)m->settleMaxMs) {
          settleEnd(m, in, pR, pF, seen && settled);
        }
      }
      if (!autoMotionActive(m)) {
        /* STOPPED: a snap epoch (references 0); DONE: set below */
        rLat = rFwd = wLat = wFwd = ffR = ffP = vRef = 0.0f;
      }
    } else if (m->kind == AUTO_MVK_YAW) {
      if (!m->turnDone) {
        if (m->state == AUTO_MV_RUN) {
          m->tRun += dt;
        }
        float s, v, a;
        autoMvProfEval(&m->py, m->tRun, &s, &v, &a);
        m->yawOffNow = m->yawOff + (m->yawCmd > 0.0f ? s : -s);
        if (m->tRun >= m->py.T) {
          endTurn(m, now);
        }
      } else if (m->state == AUTO_MV_SETTLE && !m->bob) {
        if (now - m->stateMs >= (uint32_t)m->dwellMs) {
          if (!in->sceneTracked && in->flight && !in->simPlant) {
            bobStart(m, in);   /* the tracker lost its G with the view: excite it */
          } else {
            m->dStale = !in->sceneTracked && !in->simPlant && in->flight;
            /* the tracker kept its G (a small turn): the view shifted by the turn's angle */
            m->viewShift += absf(m->yawCmd) * 0.01745329f;
            finish(m, AUTO_MV_DONE, (autoMvEnd_t)m->doneEnd, now);
          }
        }
      } else if (m->bob) {
        bool cycleEnd;   /* just back at the start height */
        const int leg = bobStep(m, in, running, dt, &vRef, &cycleEnd);
        /* the tracker's first G is rough (+/-35 % on the observer's own
         * synthetic flights, test_cam_vz_obs.c): one more whole cycle with it */
        if (!in->sceneTracked) {
          m->bobTrackedLeg = -1;
        } else if (m->bobTrackedLeg < 0) {
          m->bobTrackedLeg = leg;
        }
        if (cycleEnd) {
          const bool refined = m->bobTrackedLeg >= 0 && leg - m->bobTrackedLeg >= 2;
          if (refined || (in->sceneTracked && leg >= 2 * AUTO_MV_BOB_CYCLES)) {
            m->dStale = false;
            m->viewShift = 0.0f;   /* a G of the new view */
            finish(m, AUTO_MV_DONE, (autoMvEnd_t)m->doneEnd, now);
            vRef = 0.0f;
          } else if (leg >= 2 * AUTO_MV_BOB_CYCLES) {
            /* no scene distance after the turn: back to the start heading once (its report), then RETRACK (a guard's
             * stop request is not lost: without the view the bob does not advance, and a tilt asks again next step) */
            land = retrackEnd(m, in);
            vRef = 0.0f;
          }
        }
      }
    }
  }

  /* a stopped move's brake: a lean against the speed it had */
  if (!autoMotionActive(m) && m->brakeS > 0.0f) {
    m->brakeS -= dt;
    const float v = sqrtf(m->brakeLat * m->brakeLat + m->brakeFwd * m->brakeFwd);
    if (m->brakeS > 0.0f && v > 0.0f) {
      const float aLat = -m->brakeAcc * m->brakeLat / v, aFwd = -m->brakeAcc * m->brakeFwd / v;
      ffR = aLat / AUTO_MV_G * 57.29578f;
      ffP = -aFwd / AUTO_MV_G * 57.29578f;
    } else {
      m->brakeS = 0.0f;
    }
  }

  out->epoch = m->epoch;
  out->snap = m->snap;
  out->snapEpoch = m->snapEpoch;
  out->refLat = rLat;
  out->refFwd = rFwd;
  out->rateLat = wLat;
  out->rateFwd = wFwd;
  out->ffRollDeg = ffR;
  out->ffPitchDeg = ffP;
  out->dLowM = m->dLowFwd;   /* (meaningful while the recorder's mvState shows a MOVE running) */
  out->vRef = vRef;
  if ((!autoMotionActive(m) && m->kind == AUTO_MVK_MOVE && m->state == AUTO_MV_DONE && !m->snap) || m->backOff) {
    out->refLat = m->latTgt;   /* a finished move keeps its end as the hold's reference (a contact's stop: its */
    out->refFwd = m->fwdTgt;   /* back-off anchor, backOff: only ever set on a stopped MOVE) */
  }
  out->hOff = autoMotionActive(m) ? m->hOffNow : m->hOff;
  out->yawOffDeg = (m->kind == AUTO_MVK_YAW && autoMotionActive(m) && !m->turnDone) ? m->yawOffNow : m->yawOff;
  out->camGuessOk = m->kind == AUTO_MVK_YAW && autoMotionActive(m);
  out->state = m->state;
  out->kind = m->kind;
  out->why = m->why;
  out->end = m->end;
  out->seq = m->seq;
  uint8_t fl = 0;
  sceneDs_t ds;
  fl = sceneD(m, in, &ds);
  out->sceneDM = ds.fwd;
  out->flags = (uint8_t)(fl | (out->camGuessOk ? AUTO_MVF_CAM_GUESS : 0) | (m->bob ? AUTO_MVF_BOB : 0) | m->yawFlags);
  if (m->kind == AUTO_MVK_MOVE) {
    out->progRightM = m->progLat * m->dLat0;
    out->progFwdM = m->d0 > 0.0f ? m->d0 * (1.0f - expf(-m->progFwd)) : 0.0f;
    out->progUpM = m->progUp;
    out->tgtRightM = m->latM;
    out->tgtFwdM = m->fwdM;
    out->tgtUpM = m->upM;
  } else if (m->kind == AUTO_MVK_YAW) {
    out->progUpM = m->bob ? m->progUp : 0.0f;
  }
  out->yawTgtDeg = m->yawCmd;
  out->phaseMs = now - m->stateMs;
  out->holdWait = autoMotionActive(m);
  return land;
}

AUTO_HOT void autoMotionIdleOut(const autoMotion_t *m, autoMvOut_t *out)
{
  memset(out, 0, sizeof(*out));
  out->epoch = m->epoch;
  out->snap = m->snap;
  out->snapEpoch = m->snapEpoch;
  if ((m->kind == AUTO_MVK_MOVE && m->state == AUTO_MV_DONE && !m->snap) || m->backOff) {
    out->refLat = m->latTgt;   /* a finished move keeps its end as the hold's reference, as in HOVER (a contact's */
    out->refFwd = m->fwdTgt;   /* back-off anchor too) */
  }
  out->hOff = m->hOff;
  out->yawOffDeg = m->yawOff;
  out->state = m->state;
  out->kind = m->kind;
  out->why = m->why;
  out->end = m->end;
  out->seq = m->seq;
  out->yawTgtDeg = m->yawCmd;
}

const char *autoMvStateName(autoMvState_t s)
{
  switch (s) {
    case AUTO_MV_IDLE: return "IDLE";
    case AUTO_MV_RUN: return "RUN";
    case AUTO_MV_PAUSED: return "PAUSED";
    case AUTO_MV_SETTLE: return "SETTLE";
    case AUTO_MV_DONE: return "DONE";
    case AUTO_MV_STOPPED: return "STOPPED";
  }
  return "?";
}

const char *autoMvWhyName(autoMvWhy_t w)
{
  switch (w) {
    case AUTO_MVW_NONE: return "NONE";
    case AUTO_MVW_NOT_WAITING: return "NOT_WAITING";
    case AUTO_MVW_BUSY: return "BUSY";
    case AUTO_MVW_RANGE: return "RANGE";
    case AUTO_MVW_HEIGHT: return "HEIGHT";
    case AUTO_MVW_NO_DIST: return "NO_DIST";
    case AUTO_MVW_TOO_CLOSE: return "TOO_CLOSE";
    case AUTO_MVW_NO_VIEW: return "NO_VIEW";
    case AUTO_MVW_NO_CAM_HEIGHT: return "NO_CAM_HEIGHT";
    case AUTO_MVW_BATT: return "BATT";
    case AUTO_MVW_AIR: return "AIR";
    case AUTO_MVW_TOO_WIDE: return "TOO_WIDE";
    case AUTO_MVW_PLAN_CANCELLED: return "PLAN_CANCELLED";
    case AUTO_MVW_SH_STEP: return "SH_STEP";
    case AUTO_MVW_SH_RATE: return "SH_RATE";
    case AUTO_MVW_SH_CONTACT: return "SH_CONTACT";
    case AUTO_MVW_UP_HEADROOM: return "UP_HEADROOM";
  }
  return "?";
}

const char *autoMvEndName(autoMvEnd_t e)
{
  switch (e) {
    case AUTO_MVE_NONE: return "NONE";
    case AUTO_MVE_DONE: return "DONE";
    case AUTO_MVE_STOP_CMD: return "STOP_CMD";
    case AUTO_MVE_PROGRESS: return "PROGRESS";
    case AUTO_MVE_LEFT_HOVER: return "LEFT_HOVER";
    case AUTO_MVE_RETRACK: return "RETRACK";
    case AUTO_MVE_SHORT: return "SHORT";
    case AUTO_MVE_FRONT: return "FRONT";
    case AUTO_MVE_CONTACT: return "CONTACT";
    case AUTO_MVE_NO_VIEW: return "NO_VIEW";
    case AUTO_MVE_TILT: return "TILT";
    case AUTO_MVE_UNDONE: return "UNDONE";
    case AUTO_MVE_LOOP_GAP: return "LOOP_GAP";
    case AUTO_MVE_VIEW_FAST: return "VIEW_FAST";
    case AUTO_MVE_LONG: return "LONG";
  }
  return "?";
}
