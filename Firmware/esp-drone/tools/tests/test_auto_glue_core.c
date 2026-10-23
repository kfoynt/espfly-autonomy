/*
 * Host unit tests for the pure parts of the autonomy glue
 * (components/core/crazyflie/utils/interface/auto_glue_core.h and
 * utils/src/auto_glue_core.c, docs/autonomy-a2b.md): the reset-lock RTC
 * record (and its USB-unlock mark), the settings table, the setpoint override and both thrust
 * barriers, the clock ages, the tilt polynomial, the IDLE fast path, the
 * phone hook, the continuous-condition timer, the AUTO_STATUS fill and the
 * trip bits.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      -Icomponents/core/crazyflie/modules/interface -Icomponents/core/crazyflie/hal/interface \
 *      tools/tests/test_auto_glue_core.c components/core/crazyflie/utils/src/auto_glue_core.c \
 *      components/core/crazyflie/utils/src/auto_proto.c components/core/crazyflie/utils/src/cam_vz_obs.c \
 *      components/core/crazyflie/utils/src/auto_motion.c -lm -o /tmp/test_auto_glue_core && /tmp/test_auto_glue_core
 *
 * Lines starting with "info:" are measured numbers, not checks.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_glue_core.h"
#include "cam_vz_obs.h"     /* the cvqlate / cvqlates ranges against camVzObsConfigValid */
#include "autonomy.h"   /* AUTONOMY_IDLE_STEP_EVERY; also proves the glue API header is host-clean */

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static bool sameFloat(float a, float b)
{
  return (isnan(a) && isnan(b)) || memcmp(&a, &b, sizeof(a)) == 0;
}

/* ---- reset-lock record ---------------------------------------------------- */

static void garbage(void *p, size_t n, uint32_t seed)
{
  uint8_t *b = p;
  for (size_t i = 0; i < n; i++) {
    seed = seed * 1103515245u + 12345u;
    b[i] = (uint8_t)(seed >> 16);
  }
}

static void testRtcLock(void)
{
  autoRtcRecord_t r;

  /* Battery power-on: RTC memory is garbage, the magic absent: unlocked. */
  garbage(&r, sizeof(r), 1);
  r.magic = 0x12345678u;
  CHECK(!autoRtcValid(&r), "garbage record invalid");
  CHECK(!autoRtcBootLocked(true, &r), "power-on, magic absent: unlocked");
  CHECK(autoRtcBootLocked(false, &r), "other reset, magic absent: locked");

  /* A valid record (any earlier boot) locks every boot, power-on included. */
  autoRtcStartBoot(&r, false);
  CHECK(autoRtcValid(&r), "fresh record valid");
  CHECK(autoRtcBootLocked(true, &r), "power-on with the magic present (EN button, dip): locked");
  CHECK(autoRtcBootLocked(false, &r), "software/brownout/watchdog reset: locked");

  /* Magic present, CRC broken (corruption): locked. */
  autoRtcRecord_t bad = r;
  bad.trans[3].reason ^= 0x40;
  CHECK(!autoRtcValid(&bad), "flipped byte invalidates the CRC");
  CHECK(autoRtcBootLocked(true, &bad), "power-on, magic present, bad CRC: locked");
  CHECK(autoRtcBootLocked(false, &bad), "reset, magic present, bad CRC: locked");
  bad = r;
  bad.version = AUTO_RTC_VERSION + 1;
  bad.crc = autoProtoCrc32((const uint8_t *)&bad + 8, sizeof(bad) - 8);
  CHECK(!autoRtcValid(&bad), "other version invalid");
  CHECK(autoRtcBootLocked(true, &bad), "power-on, other version: locked (magic present)");
}

static void testRtcRecord(void)
{
  autoRtcRecord_t r;
  garbage(&r, sizeof(r), 7);
  autoRtcStartBoot(&r, false);
  CHECK(r.magic == AUTO_RTC_MAGIC && r.version == AUTO_RTC_VERSION, "magic and version");
  CHECK(r.boots == 1 && r.transitions == 0 && r.prevTransitions == 0, "first boot counters");
  CHECK(r.prevState == 0 && r.prevReason == 0 && r.prevMode == 0 && r.prevLocked == 0, "no previous boot after garbage");
  CHECK(r.locked == 0 && r.lastState == AUTO_IDLE, "this boot unlocked, IDLE");
  CHECK(r.crc == autoProtoCrc32((const uint8_t *)&r + 8, sizeof(r) - 8), "CRC is CRC-32 of bytes [8, end)");
  CHECK(offsetof(autoRtcRecord_t, version) == 8, "CRC covers from offset 8");
  CHECK(sizeof(autoRtcRecord_t) == 96, "record is 96 bytes (%u)", (unsigned)sizeof(autoRtcRecord_t));

  /* Ring: 11 transitions keep the last 8, newest at (n - 1) % 8. */
  for (uint32_t i = 0; i < 11; i++) {
    autoRtcNoteTransition(&r, 1000u + i, (autoState_t)(i % 6), (autoState_t)((i + 1) % 6),
                          (autoReason_t)(i + 1), (uint8_t)(20 + i), AUTO_MODE_BENCH);
    CHECK(autoRtcValid(&r), "valid after transition %u", (unsigned)i);
  }
  CHECK(r.transitions == 11, "11 transitions");
  bool ringOk = true;
  for (uint32_t i = 3; i < 11; i++) {
    const autoRtcTransition_t *t = &r.trans[i % AUTO_RTC_TRANSITIONS];
    ringOk &= t->ms == 1000u + i && t->from == i % 6 && t->to == (i + 1) % 6 && t->reason == i + 1 &&
              t->vzReason == 20 + i;
  }
  CHECK(ringOk, "ring wraps: entries 3..10 kept in place");
  CHECK(r.lastState == (10 + 1) % 6 && r.lastReason == 11 && r.lastMode == AUTO_MODE_BENCH, "last* follow the newest");

  /* The next boot carries this one's last state into prev*. */
  const autoRtcRecord_t before = r;
  autoRtcStartBoot(&r, true);
  CHECK(autoRtcValid(&r), "valid after the second boot");
  CHECK(r.boots == 2, "boots counts (%u)", (unsigned)r.boots);
  CHECK(r.prevState == before.lastState && r.prevReason == before.lastReason && r.prevMode == before.lastMode &&
        r.prevLocked == 0 && r.prevTransitions == 11, "previous boot carried");
  CHECK(r.locked == 1 && r.transitions == 0 && r.lastState == AUTO_IDLE && r.lastReason == AUTO_RS_NONE,
        "this boot starts locked, IDLE, no transitions");
  autoRtcStartBoot(&r, true);
  CHECK(r.boots == 3 && r.prevLocked == 1 && r.prevTransitions == 0, "third boot: prevLocked carried");

  /* A corrupted record restarts the boot count. */
  r.crc ^= 1u;
  autoRtcStartBoot(&r, true);
  CHECK(r.boots == 1 && r.prevState == 0 && r.prevTransitions == 0, "corrupted record: boots 1, nothing carried");
}

/* The USB console's 'auto unlock' (user decision 2026-10-12, evening): the
 * record of a locked boot is marked, stays valid, and the next boot carries
 * the mark in prevLocked; the boot rule is untouched (the next reset locks). */
static void testRtcUsbUnlock(void)
{
  autoRtcRecord_t r;
  garbage(&r, sizeof(r), 11);
  autoRtcStartBoot(&r, false);   /* a battery power-on */
  autoRtcStartBoot(&r, true);    /* then a software reset: locked */
  CHECK(r.locked == AUTO_RTC_LOCKED && r.boots == 2, "locked boot (%u)", (unsigned)r.locked);
  autoRtcNoteTransition(&r, 500u, AUTO_IDLE, AUTO_ARMED, AUTO_RS_ARM, 0, AUTO_MODE_DRY);   /* a DRY sortie */
  const autoRtcRecord_t before = r;
  CHECK(autoRtcNoteUsbUnlock(&r), "unlock noted");
  CHECK(autoRtcValid(&r) && r.locked == AUTO_RTC_USB_UNLOCKED, "valid, locked=%u", (unsigned)r.locked);
  autoRtcRecord_t same = r;
  same.locked = before.locked;
  same.crc = before.crc;
  CHECK(memcmp(&same, &before, sizeof(same)) == 0, "only locked and the CRC change");
  CHECK(!autoRtcNoteUsbUnlock(&r) && r.locked == AUTO_RTC_USB_UNLOCKED && autoRtcValid(&r),
        "a second unlock: nothing to note");
  autoRtcNoteTransition(&r, 900u, AUTO_ARMED, AUTO_IDLE, AUTO_RS_DISARM_CMD, 0, AUTO_MODE_DRY);
  CHECK(autoRtcValid(&r) && r.locked == AUTO_RTC_USB_UNLOCKED, "a later transition keeps the mark");
  CHECK(autoRtcBootLocked(false, &r) && autoRtcBootLocked(true, &r), "the next reset locks again (magic present)");
  autoRtcStartBoot(&r, true);
  CHECK(r.prevLocked == AUTO_RTC_USB_UNLOCKED && r.locked == AUTO_RTC_LOCKED && r.prevTransitions == 2,
        "next boot: prevLocked %u (USB unlock), locked again", (unsigned)r.prevLocked);

  /* Nothing to note on an unlocked boot or an invalid record (no CRC over unknown bytes). */
  garbage(&r, sizeof(r), 12);
  autoRtcStartBoot(&r, false);
  CHECK(!autoRtcNoteUsbUnlock(&r) && r.locked == AUTO_RTC_UNLOCKED, "power-on boot: nothing to note");
  autoRtcStartBoot(&r, true);
  r.trans[0].ms ^= 1u;   /* corrupted */
  const uint32_t crc = r.crc;
  CHECK(!autoRtcNoteUsbUnlock(&r) && r.crc == crc && r.locked == AUTO_RTC_LOCKED && !autoRtcValid(&r),
        "invalid record: left as it is");
}

/* ---- settings -------------------------------------------------------------- */

static void testSettings(void)
{
  bool namesOk = true, defsInRange = true, typesOk = true;
  for (int i = 0; i < AUTO_SET_COUNT; i++) {
    const autoSettingInfo_t *s = autoSettingInfo((autoSettingId_t)i);
    if (s == NULL || s->name == NULL || s->unit == NULL) {
      namesOk = false;
      continue;
    }
    namesOk &= strlen(s->name) >= 1 && strlen(s->name) <= 15;   /* NVS key limit */
    namesOk &= autoSettingFind(s->name) == i;
    defsInRange &= s->min <= s->def && s->def <= s->max && autoSettingInRange((autoSettingId_t)i, s->def);
    defsInRange &= !autoSettingInRange((autoSettingId_t)i, s->min - 1) &&
                   !autoSettingInRange((autoSettingId_t)i, s->max + 1);
    switch (s->type) {
      case AUTO_NVS_U8:
        typesOk &= s->min >= 0 && s->max <= 255;
        break;
      case AUTO_NVS_U16:
        typesOk &= s->min >= 0 && s->max <= 65535;
        break;
      case AUTO_NVS_I16:
        typesOk &= s->min >= -32768 && s->max <= 32767;
        break;
      case AUTO_NVS_U32:
        typesOk &= s->min >= 0;
        break;
      default:
        typesOk = false;
        break;
    }
    printf("info: %-9s type %d %7ld..%-7ld default %7ld %s\n", s->name, (int)s->type, (long)s->min, (long)s->max,
           (long)s->def, s->unit);
  }
  CHECK(namesOk, "names 1..15 chars, unique, found by name");
  CHECK(defsInRange, "defaults inside their ranges, ranges closed");
  CHECK(typesOk, "ranges fit the NVS types");
  CHECK(autoSettingInfo(AUTO_SET_COUNT) == NULL && autoSettingInfo((autoSettingId_t)-1) == NULL, "id range");
  CHECK(autoSettingFind("nope") == -1 && autoSettingFind("") == -1 && autoSettingFind(NULL) == -1, "unknown name");
  CHECK(!autoSettingInRange(AUTO_SET_COUNT, 0), "unknown id out of range");

  /* The defaults are the header constants (WP4's 4.00 V / 165000 flow in). */
  const autoSettingInfo_t *vmin = autoSettingInfo(AUTO_SET_VMIN), *k = autoSettingInfo(AUTO_SET_VZK);
  CHECK(vmin->def == (int32_t)lrintf(AUTO_VMIN_DEFAULT * 1000.0f), "vmin default = AUTO_VMIN_DEFAULT (%ld)",
        (long)vmin->def);
  CHECK(vmin->min == (int32_t)lrintf(AUTO_VMIN_LO * 1000.0f) && vmin->max == 4200 && vmin->type == AUTO_NVS_U16,
        "vmin AUTO_VMIN_LO..4200 mV u16 (%ld..)", (long)vmin->min);
  CHECK(k->def == (int32_t)VZ_K_DEFAULT && k->min == 120000 && k->max == 180000 && k->type == AUTO_NVS_U32,
        "vzK default = VZ_K_DEFAULT (%ld), 120000..180000 u32", (long)k->def);
  printf("info: defaults vmin %ld mV, vzK %ld (A2b: 4000 / 165000 with WP4's headers)\n", (long)vmin->def,
         (long)k->def);
  const autoSettingInfo_t *s;
  s = autoSettingInfo(AUTO_SET_AUTOON);
  CHECK(strcmp(s->name, "autoon") == 0 && s->def == 0 && s->max == 1 && s->type == AUTO_NVS_U8, "autoon");
  s = autoSettingInfo(AUTO_SET_APPREQ);
  CHECK(strcmp(s->name, "appreq") == 0 && s->def == 1 && s->max == 1, "appreq");
  s = autoSettingInfo(AUTO_SET_AUTOGEST);
  CHECK(strcmp(s->name, "autogest") == 0 && s->def == 1 && s->max == 1, "autogest");
  s = autoSettingInfo(AUTO_SET_AUTOAIR);
  CHECK(strcmp(s->name, "autoair") == 0 && s->def == 6000 && s->min == 2000 && s->max == 8000,
        "autoair 6000, 2000..8000 (2026-10-12 evening: a 0.60 m climb; A2b 4500, 2000..6000)");
  s = autoSettingInfo(AUTO_SET_AUTOWAIT);
  CHECK(strcmp(s->name, "autowait") == 0 && s->def == 5000 && s->min == 0 && s->max == 120000 &&
        s->type == AUTO_NVS_U32 && s->legacyType == AUTO_NVS_U16,
        "autowait 5 s, at most 120 s, u32 with the legacy u16 item read (2026-10-14; 60 s u16 for the interactive "
        "hover, 12 s that evening, A2b 8 s)");
  CHECK(autoSettingInRange(AUTO_SET_AUTOWAIT, 120000) && !autoSettingInRange(AUTO_SET_AUTOWAIT, 120001) &&
        autoSettingInRange(AUTO_SET_AUTOWAIT, 10000) && autoSettingInRange(AUTO_SET_AUTOWAIT, 0) &&
        !autoSettingInRange(AUTO_SET_AUTOWAIT, -1), "autowait 0..120000 ms (the user's stored 10000 too)");
  int legacy = 0;
  for (int i = 0; i < AUTO_SET_COUNT; i++) {
    legacy += autoSettingInfo((autoSettingId_t)i)->legacyType != 0;
  }
  CHECK(legacy == 1, "only autowait has a legacy type (%d)", legacy);
  CHECK(autoSatMs16(110000u) == 65535u && autoSatMs16(65535u) == 65535u && autoSatMs16(65534u) == 65534u &&
        autoSatMs16(0u) == 0u, "u16 ms fields saturate at 65535 (110000 -> %u)", autoSatMs16(110000u));
  s = autoSettingInfo(AUTO_SET_AUTOACCEL);
  CHECK(strcmp(s->name, "autoaccel") == 0 && s->def == 2000 && s->min == 0 && s->max == 3000, "autoaccel");
  s = autoSettingInfo(AUTO_SET_VZBAIR);
  CHECK(strcmp(s->name, "vzBair") == 0 && s->def == -23 && s->min == -300 && s->max == 300 &&
        s->type == AUTO_NVS_I16, "vzBair -23 in 0.1 mg, -300..300 i16 (%ld)", (long)s->def);
  s = autoSettingInfo(AUTO_SET_VZCAPLO);
  CHECK(strcmp(s->name, "vzcaplo") == 0 && s->def == 1650 && s->min == 1000 && s->max == 2000, "vzcaplo");
  s = autoSettingInfo(AUTO_SET_PHOTOCOL);
  CHECK(strcmp(s->name, "photocol") == 0 && s->def == 0 && s->min == 0 && s->max == 1 && s->type == AUTO_NVS_U8 &&
        autoSettingFind("photocol") == AUTO_SET_PHOTOCOL, "photocol: u8 0/1, default 0 (grey; colour photos opt-in, "
        "2026-10-15)");
  s = autoSettingInfo(AUTO_SET_CVQLATE);
  CHECK(strcmp(s->name, "cvqlate") == 0 && s->def == 1 && s->min == 1 && s->max == 5 && s->type == AUTO_NVS_U8 &&
        autoSettingFind("cvqlate") == AUTO_SET_CVQLATE && !autoSettingInRange(AUTO_SET_CVQLATE, 0) &&
        autoSettingInRange(AUTO_SET_CVQLATE, 5) && !autoSettingInRange(AUTO_SET_CVQLATE, 6),
        "cvqlate: u8 1..5, default 1 (the late offset noise off, 2026-10-15; x10 cut, a2b 25.8)");
  s = autoSettingInfo(AUTO_SET_CVQLATES);
  CHECK(strcmp(s->name, "cvqlates") == 0 && s->def == 40 && s->min == 20 && s->max == 120 && s->type == AUTO_NVS_U8 &&
        autoSettingFind("cvqlates") == AUTO_SET_CVQLATES, "cvqlates: u8 20..120 s, default 40");
  /* every value in range is a valid observer config (cam_vz_glue.c reads them at boot) */
  bool allValid = true;
  for (int32_t q = 1; q <= 5; q++) {
    for (int32_t t = 20; t <= 120; t++) {
      camVzObsConfig_t oc;
      camVzObsDefaultConfig(&oc);
      oc.qLateMult = (float)q;
      oc.qLateS = (float)t;
      allValid = allValid && camVzObsConfigValid(&oc);
    }
  }
  CHECK(allValid, "every stored cvqlate x cvqlates in range is a valid observer config");
  camVzObsConfig_t oc6;
  camVzObsDefaultConfig(&oc6);
  oc6.qLateMult = 6.0f;
  CHECK(!camVzObsConfigValid(&oc6), "x6 is not a valid observer config (the strength is capped at x5)");
  /* the offset drift rate T2 (2026-10-16, docs/autonomy-a2b.md 29) */
  s = autoSettingInfo(AUTO_SET_CVT2K);
  CHECK(strcmp(s->name, "cvt2k") == 0 && s->def == 0 && s->min == 0 && s->max == 200 && s->type == AUTO_NVS_U8 &&
        autoSettingFind("cvt2k") == AUTO_SET_CVT2K && !autoSettingInRange(AUTO_SET_CVT2K, 201),
        "cvt2k: u8 0..200 (kR x 100), default 0 (T2 off)");
  s = autoSettingInfo(AUTO_SET_CVT2R);
  CHECK(strcmp(s->name, "cvt2r") == 0 && s->def == 124 && s->min == -350 && s->max == 350 &&
        s->type == AUTO_NVS_I16 && autoSettingFind("cvt2r") == AUTO_SET_CVT2R &&
        !autoSettingInRange(AUTO_SET_CVT2R, 351) && !autoSettingInRange(AUTO_SET_CVT2R, -351),
        "cvt2r: i16 -350..350 ug/s (within the 0.35 mg/s bound), default 124 (the prior 0.124 mg/s)");
  /* the motion speed step (2026-10-16, auto_motion.h "fast moves") */
  s = autoSettingInfo(AUTO_SET_MVSPEED);
  CHECK(strcmp(s->name, "mvspeed") == 0 && s->def == 100 && s->min == 100 && s->max == 300 &&
        s->type == AUTO_NVS_U16 && strcmp(s->unit, "%") == 0 && s->legacyType == 0 &&
        autoSettingFind("mvspeed") == AUTO_SET_MVSPEED && AUTO_SET_MVSPEED == AUTO_SET_CVT2R + 1 &&
        autoSettingInRange(AUTO_SET_MVSPEED, 100) &&
        autoSettingInRange(AUTO_SET_MVSPEED, 300) && !autoSettingInRange(AUTO_SET_MVSPEED, 99) &&
        !autoSettingInRange(AUTO_SET_MVSPEED, 301) && s->def == AUTO_MV_SPEED_DEFAULT_PCT &&
        s->min == AUTO_MV_SPEED_MIN_PCT && s->max == AUTO_MV_SPEED_MAX_PCT,
        "mvspeed: u16 100..300 %%, default 100, appended after cvt2r");
  /* the guard margins (2026-10-17, auto_motion.h "guard margins"), appended after mvspeed: u8, the header's defaults
   * and ranges; every value in range is taken by autoMvMarginsFrom as it is (but the stop over the floor - 5). Since
   * 2026-10-18 ("guards off") the first four take 0 (off) too: the table's min is 0, autoSettingOnMin the range's
   * lower end, 1..on - 1 refused */
  {
    static const struct {
      autoSettingId_t id;
      const char *name, *unit;
      int32_t min, on, max, def;
    } mg[6] = {
      {AUTO_SET_MVFLOOR, "mvfloor", "cm", 0, 20, 100, 30}, {AUTO_SET_MVFSTOP, "mvfstop", "cm", 0, 15, 80, 25},
      {AUTO_SET_MVPROGM, "mvprogm", "cm", 0, 10, 60, 20}, {AUTO_SET_MVPROGR, "mvprogr", "%", 0, 10, 100, 40},
      {AUTO_SET_MVLATM, "mvlatm", "cm", 15, 0, 100, 40}, {AUTO_SET_MVNEAR, "mvnear", "cm", 0, 0, 150, 80},
    };
    int okMg = 0, okOff = 0;
    for (int i = 0; i < 6; i++) {
      s = autoSettingInfo(mg[i].id);
      okMg += (int)mg[i].id == (int)AUTO_SET_MVSPEED + 1 + i && strcmp(s->name, mg[i].name) == 0 &&
              strcmp(s->unit, mg[i].unit) == 0 && s->type == AUTO_NVS_U8 && s->legacyType == 0 && s->min == mg[i].min &&
              s->max == mg[i].max && s->def == mg[i].def && autoSettingFind(mg[i].name) == (int)mg[i].id &&
              autoSettingInRange(mg[i].id, mg[i].min) && autoSettingInRange(mg[i].id, mg[i].max) &&
              !autoSettingInRange(mg[i].id, mg[i].min - 1) && !autoSettingInRange(mg[i].id, mg[i].max + 1) &&
              autoSettingOnMin(mg[i].id) == mg[i].on;
      if (mg[i].on > 0) {
        okOff += autoSettingInRange(mg[i].id, 0) && !autoSettingInRange(mg[i].id, 1) &&
                 !autoSettingInRange(mg[i].id, mg[i].on - 1) && autoSettingInRange(mg[i].id, mg[i].on);
      }
    }
    CHECK(okMg == 6, "mvfloor 0 or 20..100 cm (30), mvfstop 0 or 15..80 cm (25), mvprogm 0 or 10..60 cm (20), mvprogr 0 or "
          "10..100 %% (40), mvlatm 15..100 cm (40), mvnear 0..150 cm (80; 60 until the review of 2026-10-17): u8, appended "
          "after mvspeed (%d of 6)", okMg);
    CHECK(okOff == 4, "mvfloor, mvfstop, mvprogm, mvprogr: 0 (off) taken, 1 and the range's lower end - 1 refused (%d of 4)",
          okOff);
    CHECK(autoSettingOnMin(AUTO_SET_CVHB) == 10 && autoSettingOnMin(AUTO_SET_MVSPEED) == 0 &&
              autoSettingOnMin(AUTO_SET_MVKLOW) == 0 && autoSettingOnMin(AUTO_SET_MVLAT) == 0 &&
              autoSettingOnMin(AUTO_SET_COUNT) == 0,
          "autoSettingOnMin: cvhb 10, the keys whose range is the table's 0");
    /* the move fixes (2026-10-17, auto_motion.h "move fixes"), appended after mvnear, last */
    s = autoSettingInfo(AUTO_SET_MVFIX);
    CHECK(AUTO_SET_MVFIX == AUTO_SET_MVNEAR + 1 && strcmp(s->name, "mvfix") == 0 &&
          strcmp(s->unit, "bits") == 0 && s->type == AUTO_NVS_U8 && s->legacyType == 0 && s->min == 0 && s->max == 63 &&
          s->def == 59 && s->def == (int32_t)AUTO_MVFIX_DEFAULT && autoSettingFind("mvfix") == (int)AUTO_SET_MVFIX &&
          autoSettingInRange(AUTO_SET_MVFIX, 0) && autoSettingInRange(AUTO_SET_MVFIX, 63) &&
          !autoSettingInRange(AUTO_SET_MVFIX, -1) && !autoSettingInRange(AUTO_SET_MVFIX, 64),
          "mvfix: u8 bits 0..63, default 59 (every move fix but STILL; 27 before the review's FRONT), appended after "
          "mvnear");
    /* the fast steps' acceleration and settle (2026-10-17 "no, I want real 300%", auto_motion.h), appended after mvfix,
     * last: the defaults are the caps of d3bedee, mvacc 30 / mvsettle 1000 those of 4dea542 */
    s = autoSettingInfo(AUTO_SET_MVACC);
    CHECK(AUTO_SET_MVACC == AUTO_SET_MVFIX + 1 && strcmp(s->name, "mvacc") == 0 && strcmp(s->unit, "cm/s^2") == 0 &&
          s->type == AUTO_NVS_U8 && s->legacyType == 0 && s->min == 10 && s->max == 40 && s->def == 20 &&
          s->def == AUTO_MV_ACC_DEFAULT_CM && s->def / 100.0f == AUTO_MV_FAST_A_H_CAP &&
          autoSettingFind("mvacc") == (int)AUTO_SET_MVACC && autoSettingInRange(AUTO_SET_MVACC, 10) &&
          autoSettingInRange(AUTO_SET_MVACC, 30) && autoSettingInRange(AUTO_SET_MVACC, 40) &&
          !autoSettingInRange(AUTO_SET_MVACC, 9) && !autoSettingInRange(AUTO_SET_MVACC, 41) &&
          !autoSettingInRange(AUTO_SET_MVACC, 0),
          "mvacc: u8 cm/s^2 10..40, default 20 (AUTO_MV_FAST_A_H_CAP 0.20 m/s^2), appended after mvfix");
    s = autoSettingInfo(AUTO_SET_MVSETTLE);
    CHECK(AUTO_SET_MVSETTLE == AUTO_SET_MVACC + 1 &&
          strcmp(s->name, "mvsettle") == 0 && strcmp(s->unit, "ms") == 0 && s->type == AUTO_NVS_U16 &&
          s->legacyType == 0 && s->min == 1000 && s->max == 3000 && s->def == 2500 &&
          s->def == AUTO_MV_FAST_SETTLE_MIN_MS && autoSettingFind("mvsettle") == (int)AUTO_SET_MVSETTLE &&
          autoSettingInRange(AUTO_SET_MVSETTLE, 1000) && autoSettingInRange(AUTO_SET_MVSETTLE, 3000) &&
          !autoSettingInRange(AUTO_SET_MVSETTLE, 999) && !autoSettingInRange(AUTO_SET_MVSETTLE, 3001) &&
          !autoSettingInRange(AUTO_SET_MVSETTLE, 25),
          "mvsettle: u16 ms 1000..3000, default 2500 (AUTO_MV_FAST_SETTLE_MIN_MS), appended after mvacc");
    /* the camera height observer's healthy offset speed-up (2026-10-18, cam_vz_obs.h hbMult, docs/autonomy-a2b.md
     * 37), appended after mvsettle, last: hbMult x 10, 0 off, 10..80; 1..9 refused (a multiplier under 1) */
    s = autoSettingInfo(AUTO_SET_CVHB);
    CHECK(AUTO_SET_CVHB == AUTO_SET_MVSETTLE + 1 &&
          strcmp(s->name, "cvhb") == 0 && strcmp(s->unit, "x0.1") == 0 && s->type == AUTO_NVS_U8 &&
          s->legacyType == 0 && s->min == 0 && s->max == 80 && s->def == 0 &&
          autoSettingFind("cvhb") == (int)AUTO_SET_CVHB && autoSettingInRange(AUTO_SET_CVHB, 0) &&
          autoSettingInRange(AUTO_SET_CVHB, 10) && autoSettingInRange(AUTO_SET_CVHB, 30) &&
          autoSettingInRange(AUTO_SET_CVHB, 80) && !autoSettingInRange(AUTO_SET_CVHB, 1) &&
          !autoSettingInRange(AUTO_SET_CVHB, 5) && !autoSettingInRange(AUTO_SET_CVHB, 9) &&
          !autoSettingInRange(AUTO_SET_CVHB, 81) && !autoSettingInRange(AUTO_SET_CVHB, -1),
          "cvhb: u8 x0.1, 0 (off, the default) or 10..80, 1..9 and 81 refused, appended after mvsettle, last");
    /* the guards down to off (2026-10-18, auto_motion.h "guards off"), appended after cvhb, last: mvklow (k x 100,
     * 60..100), mvlat (the sideways cap x 100, 0..200, 0 off), mvwmax (the expansion rate cap x 100, 10..40) */
    s = autoSettingInfo(AUTO_SET_MVKLOW);
    CHECK(AUTO_SET_MVKLOW == AUTO_SET_CVHB + 1 && strcmp(s->name, "mvklow") == 0 && strcmp(s->unit, "%") == 0 &&
          s->type == AUTO_NVS_U8 && s->legacyType == 0 && s->min == 60 && s->max == 100 && s->def == 60 &&
          s->def == AUTO_MV_K_LOW_PCT && s->def / 100.0f == AUTO_MV_K_LOW && autoSettingFind("mvklow") == (int)AUTO_SET_MVKLOW &&
          autoSettingInRange(AUTO_SET_MVKLOW, 60) && autoSettingInRange(AUTO_SET_MVKLOW, 100) &&
          !autoSettingInRange(AUTO_SET_MVKLOW, 59) && !autoSettingInRange(AUTO_SET_MVKLOW, 101) &&
          !autoSettingInRange(AUTO_SET_MVKLOW, 0),
          "mvklow: u8 %% 60..100, default 60 (AUTO_MV_K_LOW 0.60), appended after cvhb");
    s = autoSettingInfo(AUTO_SET_MVLAT);
    CHECK(AUTO_SET_MVLAT == AUTO_SET_MVKLOW + 1 && strcmp(s->name, "mvlat") == 0 && strcmp(s->unit, "%") == 0 &&
          s->type == AUTO_NVS_U8 && s->legacyType == 0 && s->min == 0 && s->max == 200 && s->def == 50 &&
          s->def == AUTO_MV_LAT_D_REL_PCT && s->def / 100.0f == AUTO_MV_LAT_D_REL &&
          autoSettingFind("mvlat") == (int)AUTO_SET_MVLAT && autoSettingInRange(AUTO_SET_MVLAT, 0) &&
          autoSettingInRange(AUTO_SET_MVLAT, 1) && autoSettingInRange(AUTO_SET_MVLAT, 200) &&
          !autoSettingInRange(AUTO_SET_MVLAT, -1) && !autoSettingInRange(AUTO_SET_MVLAT, 201),
          "mvlat: u8 %% 0..200 (0 off), default 50 (AUTO_MV_LAT_D_REL 0.50), appended after mvklow");
    s = autoSettingInfo(AUTO_SET_MVWMAX);
    CHECK(AUTO_SET_MVWMAX == AUTO_SET_MVLAT + 1 && AUTO_SET_MVVIEW == AUTO_SET_MVWMAX + 1 &&
          strcmp(s->name, "mvwmax") == 0 && strcmp(s->unit, "x0.01/s") == 0 && s->type == AUTO_NVS_U8 &&
          s->legacyType == 0 && s->min == 10 && s->max == 40 && s->def == 15 && s->def == AUTO_MV_W_FWD_PCT &&
          s->def / 100.0f == AUTO_MV_W_FWD_MAX && autoSettingFind("mvwmax") == (int)AUTO_SET_MVWMAX &&
          autoSettingInRange(AUTO_SET_MVWMAX, 10) && autoSettingInRange(AUTO_SET_MVWMAX, 40) &&
          !autoSettingInRange(AUTO_SET_MVWMAX, 9) && !autoSettingInRange(AUTO_SET_MVWMAX, 41) &&
          !autoSettingInRange(AUTO_SET_MVWMAX, 0),
          "mvwmax: u8 x0.01/s 10..40, default 15 (AUTO_MV_W_FWD_MAX 0.15), appended after mvlat, last");
    /* the VIEW_FAST timer (2026-10-22, auto_motion.h "fast moves"): u16 ms, 0 the profile's, 1..profile - 1 refused
     * (the timer only ever gets longer), up to 1000; appended after mvwmax */
    s = autoSettingInfo(AUTO_SET_MVVIEW);
    CHECK(strcmp(s->name, "mvview") == 0 && s->type == AUTO_NVS_U16 && s->min == 0 && s->max == 1000 && s->def == 0 &&
              strcmp(s->unit, "ms") == 0 && s->legacyType == 0 && autoSettingFind("mvview") == (int)AUTO_SET_MVVIEW &&
              autoSettingOnMin(AUTO_SET_MVVIEW) == (int32_t)AUTO_MV_FAST_VIEW_DEB_MS &&
              autoSettingInRange(AUTO_SET_MVVIEW, 0) && !autoSettingInRange(AUTO_SET_MVVIEW, 1) &&
              !autoSettingInRange(AUTO_SET_MVVIEW, (int32_t)AUTO_MV_FAST_VIEW_DEB_MS - 1) &&
              autoSettingInRange(AUTO_SET_MVVIEW, (int32_t)AUTO_MV_FAST_VIEW_DEB_MS) &&
              autoSettingInRange(AUTO_SET_MVVIEW, 1000) && !autoSettingInRange(AUTO_SET_MVVIEW, 1001) &&
              !autoSettingInRange(AUTO_SET_MVVIEW, -1),
          "mvview: u16 ms 0 or %d..1000, default 0 (the profile's timer), appended after mvwmax",
          (int)AUTO_MV_FAST_VIEW_DEB_MS);
    /* the EST_RANGE guard (2026-10-22; vert_ctrl.h): vzest 0/1 default 1, vzestms 0..3000 default 0; last */
    s = autoSettingInfo(AUTO_SET_VZEST);
    const autoSettingInfo_t *s2 = autoSettingInfo(AUTO_SET_VZESTMS);
    CHECK(AUTO_SET_VZEST == AUTO_SET_MVVIEW + 1 && AUTO_SET_VZESTMS == AUTO_SET_VZEST + 1 &&
              strcmp(s->name, "vzest") == 0 && s->type == AUTO_NVS_U8 &&
              s->min == 0 && s->max == 1 && s->def == 1 && autoSettingFind("vzest") == (int)AUTO_SET_VZEST &&
              autoSettingInRange(AUTO_SET_VZEST, 0) && autoSettingInRange(AUTO_SET_VZEST, 1) &&
              !autoSettingInRange(AUTO_SET_VZEST, 2) && strcmp(s2->name, "vzestms") == 0 &&
              s2->type == AUTO_NVS_U16 && s2->min == 0 && s2->max == 3000 && s2->def == 0 &&
              strcmp(s2->unit, "ms") == 0 && autoSettingFind("vzestms") == (int)AUTO_SET_VZESTMS &&
              autoSettingInRange(AUTO_SET_VZESTMS, 0) && autoSettingInRange(AUTO_SET_VZESTMS, 3000) &&
              !autoSettingInRange(AUTO_SET_VZESTMS, 3001) && s2->max == (int32_t)(VZ_EST_GRACE_MAX_S * 1000.0f + 0.5f),
          "vzest: u8 0/1, default 1; vzestms: u16 ms 0..3000, default 0; appended after mvview");
    /* the hold's scene gate residual limit (2026-10-22, camera_hold.h CAM_HOLD_SCENE_RES_*): u8 x0.1 grey levels, 0
     * (off) or 10..255, the profile's default (80 legacy, 160 native); last */
    s = autoSettingInfo(AUTO_SET_HOLDRES);
    CHECK(AUTO_SET_HOLDRES == AUTO_SET_VZESTMS + 1 && AUTO_SET_MVLATK == AUTO_SET_HOLDRES + 1 &&
              strcmp(s->name, "holdres") == 0 && s->type == AUTO_NVS_U8 && s->min == 0 && s->max == 255 &&
              s->def == CAM_HOLD_SCENE_RES_X10_DEFAULT && s->def / 10.0f == CAM_HOLD_SCENE_RES_DEFAULT &&
              strcmp(s->unit, "x0.1") == 0 && autoSettingFind("holdres") == (int)AUTO_SET_HOLDRES &&
              autoSettingOnMin(AUTO_SET_HOLDRES) == 10 && autoSettingInRange(AUTO_SET_HOLDRES, 0) &&
              !autoSettingInRange(AUTO_SET_HOLDRES, 1) && !autoSettingInRange(AUTO_SET_HOLDRES, 9) &&
              autoSettingInRange(AUTO_SET_HOLDRES, 10) && autoSettingInRange(AUTO_SET_HOLDRES, 255) &&
              !autoSettingInRange(AUTO_SET_HOLDRES, 256),
          "holdres: u8 x0.1 0 or 10..255, default %d (%.1f grey levels), appended after vzestms",
          (int)CAM_HOLD_SCENE_RES_X10_DEFAULT, (double)CAM_HOLD_SCENE_RES_DEFAULT);
    /* the left slide's two (2026-10-22): mvlatk u16 % 100..400 default 100 (the hold's lateral position gain on a
     * MOVE's frames, camera_hold.h CAM_HOLD_LAT_GAIN_MAX), mvffr i16 x0.1 deg per m/s -50..50 default 0
     * (auto_motion.h AUTO_MV_FFR_*); last */
    s = autoSettingInfo(AUTO_SET_MVLATK);
    s2 = autoSettingInfo(AUTO_SET_MVFFR);
    CHECK(AUTO_SET_MVFFR == AUTO_SET_MVLATK + 1 && AUTO_SET_COUNT == AUTO_SET_MVFFR + 1 &&
              strcmp(s->name, "mvlatk") == 0 && s->type == AUTO_NVS_U16 && s->min == 100 && s->max == 400 &&
              s->def == 100 && s->max == (int32_t)(CAM_HOLD_LAT_GAIN_MAX * 100.0f + 0.5f) && strcmp(s->unit, "%") == 0 &&
              autoSettingFind("mvlatk") == (int)AUTO_SET_MVLATK && autoSettingOnMin(AUTO_SET_MVLATK) == 0 &&
              !autoSettingInRange(AUTO_SET_MVLATK, 99) && autoSettingInRange(AUTO_SET_MVLATK, 100) &&
              autoSettingInRange(AUTO_SET_MVLATK, 400) && !autoSettingInRange(AUTO_SET_MVLATK, 401) &&
              strcmp(s2->name, "mvffr") == 0 && s2->type == AUTO_NVS_I16 && s2->min == AUTO_MV_FFR_MIN_DDEG &&
              s2->max == AUTO_MV_FFR_MAX_DDEG && s2->min == -50 && s2->max == 50 && s2->def == 0 &&
              strcmp(s2->unit, "x0.1deg/mps") == 0 && autoSettingFind("mvffr") == (int)AUTO_SET_MVFFR &&
              autoSettingInRange(AUTO_SET_MVFFR, -50) && autoSettingInRange(AUTO_SET_MVFFR, 50) &&
              !autoSettingInRange(AUTO_SET_MVFFR, 51) && !autoSettingInRange(AUTO_SET_MVFFR, -51),
          "mvlatk: u16 %% 100..400, default 100; mvffr: i16 x0.1 deg per m/s -50..50, default 0; appended after holdres, "
          "last");
    {
      /* every stored value in range is a valid observer config (cam_vz_glue.c reads it at boot), alone */
      bool hbValid = true;
      for (int32_t v = 0; v <= 80; v++) {
        if (!autoSettingInRange(AUTO_SET_CVHB, v)) {
          continue;
        }
        camVzObsConfig_t oc;
        camVzObsDefaultConfig(&oc);
        oc.hbMult = (float)v / 10.0f;
        hbValid = hbValid && camVzObsConfigValid(&oc);
      }
      CHECK(hbValid, "every stored cvhb in range is a valid observer config");
      camVzObsConfig_t oc;
      camVzObsDefaultConfig(&oc);
      oc.hbMult = 3.0f;
      oc.qLateMult = 5.0f;
      const bool r1 = camVzObsConfigValid(&oc);
      oc.qLateMult = 1.0f;
      oc.t2kR = 0.5f;
      CHECK(!r1 && !camVzObsConfigValid(&oc), "cvhb > 0 with cvqlate > 1 or cvt2k > 0 is not (the glue refuses the "
            "speed-up then)");
    }
    {
      /* the settings' defaults latched give autoMvSpeedCfg of d3bedee bit for bit (its caps as constants) */
      uint8_t ac = 0;
      uint16_t st = 0;
      autoMotionSetFastCaps(autoSettingInfo(AUTO_SET_MVACC)->def, autoSettingInfo(AUTO_SET_MVSETTLE)->def);
      autoMotionGetFastCaps(&ac, &st);
      autoMvSpeed_t c;
      autoMvSpeedCfg(300, &c);
      CHECK(ac == 20 && st == 2500 && c.aH == AUTO_MV_FAST_A_H_CAP && c.settleMaxMs == AUTO_MV_FAST_SETTLE_MIN_MS,
            "the defaults latched: A_H at 300 exactly AUTO_MV_FAST_A_H_CAP, the settle AUTO_MV_FAST_SETTLE_MIN_MS");
      autoMotionSetFastCaps(30, 1000);
      autoMvSpeedCfg(300, &c);
      CHECK(c.aH == 0.30f && c.settleMaxMs == 1000, "mvacc 30, mvsettle 1000: A_H 0.30f, the settle 1000 ms at 300");
      autoMotionSetFastCaps(AUTO_MV_ACC_DEFAULT_CM, AUTO_MV_FAST_SETTLE_MIN_MS);
    }
    autoMvMargins_t g, d;
    autoMvMarginsDefault(&d);
    autoMvMarginsFrom(autoSettingInfo(AUTO_SET_MVFLOOR)->def, autoSettingInfo(AUTO_SET_MVFSTOP)->def,
                      autoSettingInfo(AUTO_SET_MVPROGM)->def, autoSettingInfo(AUTO_SET_MVPROGR)->def,
                      autoSettingInfo(AUTO_SET_MVLATM)->def, autoSettingInfo(AUTO_SET_MVNEAR)->def,
                      autoSettingInfo(AUTO_SET_MVFIX)->def, &g);
    CHECK(memcmp(&g, &d, sizeof(g)) == 0, "the settings' defaults give auto_motion.h's default margins");
    int okAll = 1;
    for (int32_t fl = 20; fl <= 100; fl++) {
      for (int32_t st = 15; st <= 80; st++) {
        autoMvMarginsFrom(fl, st, 10, 100, 100, 0, 0, &g);
        okAll &= g.floorCm == fl && g.planCm == fl + 5 && g.stopCm == (st <= fl - 5 ? st : fl - 5);
      }
      autoMvMarginsFrom(fl, 0, 10, 100, 100, 0, 0, &g);
      okAll &= g.floorCm == fl && g.planCm == fl + 5 && g.stopCm == 0;   /* (the stop off under any floor) */
    }
    for (int32_t st = 0; st <= 80; st++) {
      autoMvMarginsFrom(0, st, 10, 100, 100, 0, 0, &g);
      okAll &= g.floorCm == 0 && g.planCm == 0 && g.stopCm == (st == 0 || st >= 15 ? st : 15);   /* (no floor: the
                                                                                                      stop's own range) */
    }
    CHECK(okAll, "every mvfloor x mvfstop in range, and 0: the floor as set, the plan + 5 cm (0 with 0), the stop as set "
          "or floor - 5 (0 stays 0; its own range without a floor)");
    autoMvMarginsExtra(&g, autoSettingInfo(AUTO_SET_MVKLOW)->def, autoSettingInfo(AUTO_SET_MVLAT)->def);
    CHECK(memcmp(&g, &d, sizeof(g)) != 0 && g.kLowPct == d.kLowPct && g.latPct == d.latPct,
          "mvklow's and mvlat's defaults give the default k and cap");
  }
  bool t2Valid = true;
  for (int32_t k = 0; k <= 200; k++) {
    for (int32_t r = -350; r <= 350; r += 7) {
      camVzObsConfig_t oc;
      camVzObsDefaultConfig(&oc);
      oc.t2kR = (float)k / 100.0f;   /* as cam_vz_glue.c */
      oc.t2PriorMgS = (float)r / 1000.0f;
      t2Valid = t2Valid && camVzObsConfigValid(&oc);
    }
  }
  camVzObsConfig_t oe;
  camVzObsDefaultConfig(&oe);
  oe.t2kR = 2.0f;
  oe.t2PriorMgS = 0.35f;
  t2Valid = t2Valid && camVzObsConfigValid(&oe);
  oe.t2PriorMgS = -0.35f;
  t2Valid = t2Valid && camVzObsConfigValid(&oe);
  CHECK(t2Valid, "every stored cvt2k x cvt2r in range is a valid observer config (with cvqlate 1)");
  oe.qLateMult = 2.0f;
  CHECK(!camVzObsConfigValid(&oe), "cvt2k > 0 with cvqlate > 1 is not (the glue refuses T2 then)");
}

/* ---- setpoint override and barriers --------------------------------------- */

static void testBuildSetpoint(void)
{
  const float thrusts[] = {0.0f, 1.0f, 33333.5f, VZ_THRUST_ABS_MAX};
  const float yaws[] = {0.0f, -179.9f, 12.25f, 180.0f};
  bool allOk = true, zeros = true;
  for (int i = 0; i < 4; i++) {
    setpoint_t sp, ref;
    garbage(&sp, sizeof(sp), 100u + (uint32_t)i);
    autoBuildSetpoint(&sp, thrusts[i], yaws[i], 4242u + (uint32_t)i);
    memset(&ref, 0, sizeof(ref));
    ref.mode.x = modeDisable;
    ref.mode.y = modeDisable;
    ref.mode.z = modeDisable;
    ref.mode.roll = modeAbs;
    ref.mode.pitch = modeAbs;
    ref.mode.yaw = modeAbs;
    ref.attitude.yaw = yaws[i];
    ref.thrust = thrusts[i];
    ref.timestamp = 4242u + (uint32_t)i;
    allOk &= memcmp(&sp, &ref, sizeof(sp)) == 0;
    const float z = 0.0f;
    zeros &= memcmp(&sp.attitude.roll, &z, sizeof(z)) == 0 && memcmp(&sp.attitude.pitch, &z, sizeof(z)) == 0;
    zeros &= sp.attitudeRate.roll == 0.0f && sp.attitudeRate.yaw == 0.0f && !sp.velocity_body;
  }
  CHECK(allOk, "autoBuildSetpoint matches memset + fields byte for byte (padding included)");
  CHECK(zeros, "roll and pitch bitwise +0.0f, rates and body flag zero");
}

static float modeMax(int m)
{
  return m == AUTO_MODE_FLIGHT ? VZ_THRUST_ABS_MAX : (m == AUTO_MODE_DRY ? 0.0f : 15000.0f);
}

static void testSelectThrust(void)
{
  const float vz[] = {NAN, -1.0f, 0.0f, 30000.0f, 60000.0f};
  const autoThrust_t acts[] = {AUTO_THR_NONE, AUTO_THR_ZERO, AUTO_THR_VERT_CTRL, AUTO_THR_VERT_LAND, AUTO_THR_CUT,
                               (autoThrust_t)9};
  bool ok = true;
  int n = 0;
  for (unsigned a = 0; a < sizeof(acts) / sizeof(acts[0]); a++) {
    for (int m = 0; m < 3; m++) {
      const float tmax = modeMax(m);
      for (unsigned v = 0; v < sizeof(vz) / sizeof(vz[0]); v++) {
        const float got = autoSelectThrust(acts[a], vz[v], tmax);
        float want = 0.0f;
        if ((acts[a] == AUTO_THR_VERT_CTRL || acts[a] == AUTO_THR_VERT_LAND) && vz[v] > 0.0f) {
          want = vz[v] < tmax ? vz[v] : tmax;
        }
        if (!sameFloat(got, want) || !(got >= 0.0f) || got > tmax) {
          ok = false;
          printf("  select act %d mode %d vz %g: got %g want %g\n", (int)acts[a], m, (double)vz[v], (double)got,
                 (double)want);
        }
        n++;
      }
    }
  }
  CHECK(ok && n == 90, "autoSelectThrust: %d cases (6 actions x 3 modes x 5 thrusts)", n);
  /* A broken thrustMax never lets a command through. */
  CHECK(autoSelectThrust(AUTO_THR_VERT_CTRL, 30000.0f, NAN) == 0.0f, "NaN thrustMax gives 0");
  CHECK(autoSelectThrust(AUTO_THR_VERT_CTRL, 30000.0f, -5.0f) == 0.0f, "negative thrustMax gives 0");
  CHECK(sameFloat(autoSelectThrust(AUTO_THR_VERT_LAND, INFINITY, VZ_THRUST_ABS_MAX), VZ_THRUST_ABS_MAX),
        "+inf command clamps");
}

static void testBarrierMax(void)
{
  const float lim[] = {NAN, -1.0f, 0.0f, 15000.0f, VZ_THRUST_ABS_MAX, 65535.0f, 70000.0f};
  const autoThrust_t acts[] = {AUTO_THR_NONE, AUTO_THR_ZERO, AUTO_THR_VERT_CTRL, AUTO_THR_VERT_LAND, AUTO_THR_CUT,
                               (autoThrust_t)7, (autoThrust_t)255};
  bool ok = true;
  int n = 0;
  for (unsigned a = 0; a < sizeof(acts) / sizeof(acts[0]); a++) {
    for (unsigned l = 0; l < sizeof(lim) / sizeof(lim[0]); l++) {
      const float got = autoBarrierMax(acts[a], lim[l]);
      float want;
      if (acts[a] == AUTO_THR_NONE) {
        want = 65535.0f;
      } else if (acts[a] == AUTO_THR_VERT_CTRL || acts[a] == AUTO_THR_VERT_LAND) {
        want = (lim[l] >= 0.0f && lim[l] <= 65535.0f) ? lim[l] : 0.0f;
      } else {
        want = 0.0f;
      }
      if (!sameFloat(got, want)) {
        ok = false;
        printf("  barrierMax act %d lim %g: got %g want %g\n", (int)acts[a], (double)lim[l], (double)got,
               (double)want);
      }
      n++;
    }
  }
  CHECK(ok, "autoBarrierMax: %d cases (0 for ZERO/CUT/unknown, 65535 for NONE, the limit or 0)", n);
}

static void testClampThrust(void)
{
  const float t[] = {NAN, -INFINITY, -1.0f, 0.0f, 1.0f, 14999.0f, 15000.0f, 15001.0f, VZ_THRUST_ABS_MAX, 65535.0f,
                     70000.0f, INFINITY};
  const float lim[] = {NAN, -1.0f, 65536.0f, INFINITY, 0.0f, 15000.0f, VZ_THRUST_ABS_MAX, 65535.0f};
  bool ok = true;
  int n = 0;
  for (unsigned l = 0; l < sizeof(lim) / sizeof(lim[0]); l++) {
    for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
      const float got = autoClampThrust(t[i], lim[l]);
      float want;
      if (!(lim[l] >= 0.0f && lim[l] <= 65535.0f)) {
        want = 0.0f;                         /* fails closed */
      } else if (lim[l] == 65535.0f) {
        want = t[i];                         /* manual flight: untouched, whatever it is */
      } else {
        want = t[i] > 0.0f ? (t[i] < lim[l] ? t[i] : lim[l]) : 0.0f;
      }
      if (!sameFloat(got, want)) {
        ok = false;
        printf("  clamp t %g lim %g: got %g want %g\n", (double)t[i], (double)lim[l], (double)got, (double)want);
      }
      n++;
    }
  }
  CHECK(ok, "autoClampThrust: %d cases", n);
  CHECK(autoClampThrust(NAN, 15000.0f) == 0.0f && autoClampThrust(-3.0f, VZ_THRUST_ABS_MAX) == 0.0f,
        "NaN or negative under a limit gives 0");
  CHECK(isnan(autoClampThrust(NAN, 65535.0f)) && autoClampThrust(60000.0f, 65535.0f) == 60000.0f,
        "65535 leaves control.thrust untouched");
  /* The two barriers together: DRY and every ZERO/CUT give 0 whatever the controller says. */
  bool dry = true;
  const autoThrust_t acts[] = {AUTO_THR_ZERO, AUTO_THR_VERT_CTRL, AUTO_THR_VERT_LAND, AUTO_THR_CUT};
  for (unsigned a = 0; a < 4; a++) {
    for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
      const float sp = autoSelectThrust(acts[a], t[i], 0.0f);
      dry &= autoClampThrust(sp, autoBarrierMax(acts[a], 0.0f)) == 0.0f;
      dry &= autoClampThrust(t[i], autoBarrierMax(acts[a], 0.0f)) == 0.0f;   /* a bug upstream of barrier 2 */
    }
  }
  CHECK(dry, "DRY: both barriers give 0 for every action and controller output");
  bool zeroCut = true;
  for (int m = 0; m < 3; m++) {
    zeroCut &= autoClampThrust(40000.0f, autoBarrierMax(AUTO_THR_ZERO, modeMax(m))) == 0.0f;
    zeroCut &= autoClampThrust(40000.0f, autoBarrierMax(AUTO_THR_CUT, modeMax(m))) == 0.0f;
  }
  CHECK(zeroCut, "ZERO and CUT: barrier 2 gives 0 in every mode");
}

/* ---- clock --------------------------------------------------------------- */

static void testAges(void)
{
  const uint32_t now = 500000u;
  CHECK(autoAgeMs(now, now, true) == 0, "stamp = now: 0");
  CHECK(autoAgeMs(now, now - 1u, true) == 1, "stamp = now-1: 1");
  CHECK(autoAgeMs(now, now + 1u, true) == 0, "stamp = now+1 (other core): 0, not ~4e9");
  CHECK(autoAgeMs(now, now + 5u, true) == 0, "stamp = now+5: 0");
  CHECK(autoAgeMs(now, now - 1000u, true) == 1000, "1 s old");
  CHECK(autoAgeMs(now, 0u, false) == UINT32_MAX, "never: UINT32_MAX");
  CHECK(autoAgeMs(now, now, false) == UINT32_MAX, "never, whatever the stamp");
  CHECK(autoAgeMs(0x10u, 0xFFFFFFF0u, true) == 0x20u, "2^32 wrap of the ms clock: 32");
  CHECK(autoAgeMs(0xFFFFFFF0u, 0x10u, true) == 0, "stamp after now across the wrap: 0");
  CHECK(autoAgeMs(0u, 0u, true) == 0, "now 0, stamp 0");

  CHECK(autoAgeUsToMs(5000000u, 5000000u) == 0, "frame now: 0");
  CHECK(autoAgeUsToMs(5000000u, 5003000u) == 0, "newer frame (camera core): 0");
  CHECK(autoAgeUsToMs(5000000u, 4969999u) == 30, "30.001 ms old: 30");
  CHECK(autoAgeUsToMs(5000000u, 0u) == 5000, "since 0");
  CHECK(autoAgeUsToMs(UINT64_MAX, 0u) == UINT32_MAX, "saturates");
  CHECK(autoAgeUsToMs((uint64_t)UINT32_MAX * 1000u + 999u, 0u) == UINT32_MAX, "just at the limit");
  CHECK(autoAgeUsToMs((uint64_t)UINT32_MAX * 1000u + 1000u, 0u) == UINT32_MAX, "just over the limit");
}

/* ---- tilt ------------------------------------------------------------------ */

static void testTilt(void)
{
  const int n = 2000000;
  double maxErr = 0.0, at = 0.0;
  bool mono = true;
  float prev = autoTiltDegFromGravZ(-1.0f);
  for (int i = 0; i <= n; i++) {
    const float gz = -1.0f + 2.0f * (float)i / (float)n;
    const float d = autoTiltDegFromGravZ(gz);
    const double ref = acos((double)gz) * 180.0 / M_PI;
    const double e = fabs((double)d - ref);
    if (e > maxErr) {
      maxErr = e;
      at = gz;
    }
    if (i > 0 && d > prev) {
      mono = false;
    }
    prev = d;
  }
  printf("info: tilt polynomial max |error| %.5f deg at gz %.6f (2e6 points)\n", maxErr, at);
  CHECK(maxErr <= 0.01, "within 0.01 deg of acos over [-1, 1] (%.5f)", maxErr);
  CHECK(mono, "monotone (non-increasing in gz)");
  CHECK(autoTiltDegFromGravZ(1.0f) < 0.01f && fabsf(autoTiltDegFromGravZ(-1.0f) - 180.0f) < 0.01f &&
        fabsf(autoTiltDegFromGravZ(0.0f) - 90.0f) < 0.01f, "end points 0 / 90 / 180");
  CHECK(sameFloat(autoTiltDegFromGravZ(1.5f), autoTiltDegFromGravZ(1.0f)) &&
        sameFloat(autoTiltDegFromGravZ(-7.0f), autoTiltDegFromGravZ(-1.0f)) &&
        sameFloat(autoTiltDegFromGravZ(INFINITY), autoTiltDegFromGravZ(1.0f)), "clamps outside [-1, 1]");
  CHECK(isnan(autoTiltDegFromGravZ(NAN)), "NaN in, NaN out");
  CHECK(fabsf(autoTiltDegFromGravZ(cosf(30.0f * (float)M_PI / 180.0f)) - 30.0f) < 0.02f, "30 deg (the tilt trip)");
}

/* ---- IDLE fast path -------------------------------------------------------- */

static void testIdleSkip(void)
{
  const uint32_t every = AUTONOMY_IDLE_STEP_EVERY;
  CHECK(every == 100, "IDLE steps every 100th loop (10 Hz)");
  CHECK(autoIdleSkip(AUTO_IDLE, AUTO_THR_NONE, false, false, false, 1, every), "quiescent IDLE skips");
  CHECK(!autoIdleSkip(AUTO_IDLE, AUTO_THR_NONE, false, false, false, 200, every), "every 100th loop steps");
  CHECK(!autoIdleSkip(AUTO_IDLE, AUTO_THR_NONE, true, false, false, 1, every), "command pending steps");
  CHECK(!autoIdleSkip(AUTO_IDLE, AUTO_THR_NONE, false, true, false, 1, every), "new KILL steps");
  CHECK(!autoIdleSkip(AUTO_IDLE, AUTO_THR_NONE, false, false, true, 1, every), "new session steps");
  CHECK(!autoIdleSkip(AUTO_IDLE, AUTO_THR_ZERO, false, false, false, 1, every), "release hold steps every loop");
  CHECK(autoIdleSkip(AUTO_IDLE, AUTO_THR_NONE, false, false, false, 0xFFFFFFFFu, every), "loop counter wrap");
  CHECK(autoIdleSkip(AUTO_IDLE, AUTO_THR_NONE, false, false, false, 0, 0), "everyLoops 0: no periodic step");
  bool active = true;
  for (int s = AUTO_ARMED; s <= AUTO_KILLED; s++) {
    for (int a = AUTO_THR_NONE; a <= AUTO_THR_CUT; a++) {
      active &= !autoIdleSkip((autoState_t)s, (autoThrust_t)a, false, false, false, 1, every);
    }
  }
  CHECK(active, "every active state steps every loop");
  int skipped = 0;
  for (uint32_t loop = 1; loop <= 1000; loop++) {
    skipped += autoIdleSkip(AUTO_IDLE, AUTO_THR_NONE, false, false, false, loop, every) ? 1 : 0;
  }
  CHECK(skipped == 990, "1000 quiescent loops: 10 steps (%d skipped)", skipped);
}

/* ---- phone hook ------------------------------------------------------------ */

static void testAppHook(void)
{
  autoAppHook_t h;
  memset(&h, 0, sizeof(h));
  autoAppHookNote(&h, 0.0f, 0.0f, 0, 100u);
  CHECK(h.any && h.lastAppMs == 100u && !h.anyInput && h.inputCount == 0 && h.thrustZero, "centred packet: no input");
  autoAppHookNote(&h, 1.99f, -1.99f, 999, 110u);
  CHECK(h.inputCount == 0 && h.thrustZero && h.lastAppMs == 110u, "below every threshold: no input");
  autoAppHookNote(&h, 0.0f, 0.0f, 1000, 120u);
  CHECK(h.inputCount == 1 && h.anyInput && h.lastInputMs == 120u && !h.thrustZero, "thrust 1000: input");
  autoAppHookNote(&h, 0.0f, -2.0f, 0, 130u);
  CHECK(h.inputCount == 2 && h.lastInputMs == 130u && h.thrustZero, "pitch -2 deg: input, thrust zero again");
  autoAppHookNote(&h, 2.0f, 0.0f, 0, 140u);
  CHECK(h.inputCount == 3 && h.lastInputMs == 140u, "roll 2 deg: input");
  /* yaw is not an input: the hook has no yaw at all; a centred packet keeps the count */
  autoAppHookNote(&h, 0.0f, 0.0f, 0, 150u);
  CHECK(h.inputCount == 3 && h.lastInputMs == 140u && h.lastAppMs == 150u, "centred packet (any yaw): no input");

  /* Gesture: roll past 10 deg, then released below 2 deg. */
  CHECK(!h.gestured, "no gesture yet");
  autoAppHookNote(&h, 9.9f, 0.0f, 0, 200u);
  autoAppHookNote(&h, 1.0f, 0.0f, 0, 210u);
  CHECK(!h.gestured && !h.pushed, "9.9 deg is not a push");
  autoAppHookNote(&h, -10.0f, 0.0f, 0, 220u);
  CHECK(h.pushed && !h.gestured, "-10 deg: pushed");
  autoAppHookNote(&h, 5.0f, 0.0f, 0, 230u);
  CHECK(h.pushed && !h.gestured, "not released below 2 deg yet");
  autoAppHookNote(&h, 1.5f, 30.0f, 0, 240u);
  CHECK(h.gestured && h.gestureMs == 240u && !h.pushed, "released: gesture stamped (pitch does not matter)");
  autoAppHookNote(&h, 0.0f, 0.0f, 0, 250u);
  CHECK(h.gestureMs == 240u, "one stamp per release");
  CHECK(h.inputCount == 3 + 4, "the gesture packets >= 2 deg count as input (%u)", (unsigned)h.inputCount);
}

static void testSince(void)
{
  autoSinceTrack_t t;
  memset(&t, 0, sizeof(t));
  CHECK(autoSinceStep(&t, false, 100u) == 0, "false: 0");
  CHECK(autoSinceStep(&t, true, 200u) == 0, "starts at 0");
  CHECK(autoSinceStep(&t, true, 1700u) == 1500, "held 1.5 s");
  CHECK(autoSinceStep(&t, false, 1800u) == 0 && autoSinceStep(&t, true, 1900u) == 0, "a break restarts it");
  autoSinceTrack_t w = {.on = true, .sinceMs = 0xFFFFFF00u};
  CHECK(autoSinceStep(&w, true, 0x100u) == 0x200u, "across the ms wrap");
}

/* ---- AUTO_STATUS ----------------------------------------------------------- */

static void testStatusFill(void)
{
  autoFsmOut_t o;
  vzOutput_t vz;
  autoStatusExtra_t x;
  autoStatusMsg_t m;
  memset(&o, 0, sizeof(o));
  memset(&vz, 0, sizeof(vz));
  memset(&x, 0, sizeof(x));
  o.state = AUTO_HOVER;
  o.mode = AUTO_MODE_BENCH;
  o.reason = AUTO_RS_HOLD;
  o.vzLandReason = VZ_R_CMD_LAND;
  o.landTrip = AUTO_RS_TILT;
  o.failedGuards = 0x1234u;
  o.waiting = true;
  o.stable = true;
  o.prespinLed = true;
  o.stateMs = 1234u;
  o.airMs = 100000u;
  o.stableMs = 70000u;
  o.waitLeftMs = 4321u;
  vz.phase = VZ_HOLD;
  vz.reason = VZ_R_HOLD_REACHED;
  vz.camHealthy = true;
  vz.h = 0.3456f;
  vz.v = -0.1234f;
  vz.thrust = 41000.4f;
  vz.tLo = 30000.6f;
  vz.tHold = 41000.0f;
  vz.tRef = 0.0f;
  vz.khat = 165432.0f;
  x.upMs = 99999u;
  x.transitions = 7u;
  x.killCount = 3u;
  x.resetLocked = true;
  x.simPlant = true;
  x.holdApplied = true;
  x.camAwake = true;
  x.trips = AUTO_TRF_REST | AUTO_TRF_VZ_CAP;
  x.capture = AUTO_CAP_DONE;
  x.photoId = 77;
  x.vbat = 3.9876f;
  x.vbatFastMin = 3.5f;
  x.kPrior = 165000.0f;
  x.lastCmdType = AUTO_CMD_PHOTO;
  x.lastCmdResult = AUTO_R_OK;
  memset(&m, 0xA5, sizeof(m));
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.upMs == 99999u && m.bootNonce == 0 && m.transitions == 7u && m.failedGuards == 0x1234u, "header words");
  CHECK(m.state == AUTO_HOVER && m.mode == AUTO_MODE_BENCH && m.reason == AUTO_RS_HOLD &&
        m.vzLandReason == VZ_R_CMD_LAND && m.landTrip == AUTO_RS_TILT && m.vzPhase == VZ_HOLD &&
        m.vzReason == VZ_R_HOLD_REACHED, "state bytes");
  CHECK(m.flags == 0xFF, "every flag (0x%02x)", m.flags);
  CHECK(m.trips == (AUTO_TRF_REST | AUTO_TRF_VZ_CAP) && m.capture == AUTO_CAP_DONE && m.photoId == 77, "trips, capture");
  CHECK(m.stateMs == 1234 && m.airMs == 65535 && m.stableMs == 65535 && m.waitLeftMs == 4321, "ms fields saturate");
  CHECK(m.hoverWaitMs == 0, "hoverWaitMs left to the glue");
  CHECK(m.mvSpeedPct == 0, "mvSpeedPct 0 in the snapshot (the glue fills it when the status is read)");
  CHECK(m.vbatMv == 3988 && m.vbatFastMinMv == 3500, "battery mV (%u, %u)", m.vbatMv, m.vbatFastMinMv);
  CHECK(m.hMm == 346 && m.vzMms == -123, "height and speed mm (%d, %d)", m.hMm, m.vzMms);
  CHECK(m.thrust == 41000 && m.tLo == 30001 && m.tHold == 41000 && m.tRef == 0, "thrusts");
  CHECK(m.kPriorDiv10 == 16500 && m.khatDiv10 == 16543, "gains / 10 (%u, %u)", m.kPriorDiv10, m.khatDiv10);
  CHECK(m.lastCmdType == AUTO_CMD_PHOTO && m.lastCmdResult == AUTO_R_OK && m.killCount == 3u, "command, kills");
  CHECK(m.mvState == 0 && m.mvKind == 0 && m.mvWhy == 0 && m.mvEnd == 0 && m.mvFlags == 0 && m.battLow == 0 &&
        m.mvSeq == 0 && m.progRightCm == 0 && m.sceneDCm == 0 && m.hTargetCm == 0 && m.yawOffCdeg == 0 &&
        m.mvPhaseMs == 0, "no motion: the interactive hover fields 0");
  CHECK(m.airLeftDs == 0, "no air clock since 2026-10-15: airLeftDs 0 (%u)", m.airLeftDs);
  CHECK(m.airDs == 1000, "air 100 s: airDs 1000 (%u)", m.airDs);
  /* airDs at its ends (byte 118, u16 deciseconds) and airLeftDs 0 at every air time */
  static const struct { uint32_t air; uint16_t ds; } al[] = {
    {100u, 1u}, {1u, 0u}, {65535u, 655u}, {599999u, 5999u}, {6553500u, 65535u}, {7000000u, 65535u}, {0u, 0u},
  };
  for (size_t i = 0; i < sizeof(al) / sizeof(al[0]); i++) {
    o.airMs = al[i].air;
    autoStatusFill(&m, &o, &vz, &x);
    CHECK(m.airDs == al[i].ds && m.airLeftDs == 0, "airMs %u: airDs %u (%u), airLeftDs 0 (%u)", (unsigned)al[i].air,
          al[i].ds, m.airDs, m.airLeftDs);
  }
  o.airMs = 100000u;

  /* The interactive hover fields. */
  o.airMs = 41234u;
  o.heightCm = 100;
  o.battLowNow = true;
  o.mv.state = AUTO_MV_SETTLE;
  o.mv.kind = AUTO_MVK_MOVE;
  o.mv.why = AUTO_MVW_TOO_CLOSE;
  o.mv.end = AUTO_MVE_PROGRESS;
  o.mv.flags = AUTO_MVF_D_VALID | AUTO_MVF_D_TRACK;
  o.mv.seq = 4;
  o.mv.progRightM = 0.4949f;
  o.mv.progFwdM = -1.0051f;
  o.mv.progUpM = 0.2f;
  o.mv.tgtRightM = 0.5f;
  o.mv.tgtFwdM = -1.0f;
  o.mv.tgtUpM = 0.2f;
  o.mv.sceneDM = 1.874f;
  o.mv.hOff = -0.15f;
  o.mv.yawOffDeg = -123.456f;
  o.mv.yawTgtDeg = 90.0f;
  o.mv.phaseMs = 70000u;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.mvState == AUTO_MV_SETTLE && m.mvKind == AUTO_MVK_MOVE && m.mvWhy == AUTO_MVW_TOO_CLOSE &&
        m.mvEnd == AUTO_MVE_PROGRESS && m.mvFlags == (AUTO_MVF_D_VALID | AUTO_MVF_D_TRACK) &&
        m.battLow == AUTO_BATTLOW_NOW && m.mvSeq == 4, "motion bytes (one battery reading low: bit 1 only)");
  o.battLowHeld = true;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.battLow == (AUTO_BATTLOW_HELD | AUTO_BATTLOW_NOW), "BATT_LOW condition held 0.25 s: bits 0 and 1 (%u)",
        m.battLow);
  o.battLowNow = false;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.battLow == AUTO_BATTLOW_HELD, "held, the latest reading above: bit 0 only (%u)", m.battLow);
  CHECK(m.progRightCm == 49 && m.progFwdCm == -101 && m.progUpCm == 20 && m.tgtRightCm == 50 && m.tgtFwdCm == -100 &&
        m.tgtUpCm == 20, "progress and targets, cm rounded (%d %d %d)", m.progRightCm, m.progFwdCm, m.progUpCm);
  CHECK(m.sceneDCm == 187 && m.hTargetCm == 85 && m.yawOffCdeg == -12346 && m.yawTgtDeg == 90 &&
        m.mvPhaseMs == 65535 && m.airLeftDs == 0, "D 187 cm, height 100 - 15 = 85 cm, yaw -123.46 deg, phase "
        "saturated, no air clock (%u %d %d %u)", m.sceneDCm, m.hTargetCm, m.yawOffCdeg, m.airLeftDs);

  /* Saturation and non-finite values. */
  memset(&o, 0, sizeof(o));
  memset(&x, 0, sizeof(x));
  vz.h = 1e9f;
  vz.v = -1e9f;
  vz.thrust = NAN;
  vz.tLo = -5.0f;
  vz.tHold = INFINITY;
  vz.tRef = 1e12f;
  vz.khat = NAN;
  vz.camHealthy = false;
  x.vbat = NAN;
  x.vbatFastMin = -1.0f;
  x.kPrior = 1e9f;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.hMm == 32767 && m.vzMms == -32767, "i16 saturate (%d, %d)", m.hMm, m.vzMms);
  CHECK(m.thrust == 0 && m.tLo == 0 && m.tHold == 65535 && m.tRef == 65535, "u16: NaN/negative 0, large 65535");
  CHECK(m.khatDiv10 == 0 && m.kPriorDiv10 == 65535 && m.vbatMv == 0 && m.vbatFastMinMv == 0, "NaN gives 0");
  vz.h = NAN;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.hMm == 0 && m.flags == 0, "NaN height 0; no flags");
  /* the hover wait over 65.5 s (2026-10-14): the u16 field shows 65535 ("65.5 s or more") */
  o.waitLeftMs = 110000u;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.waitLeftMs == 65535 && m.hoverWaitMs == 0, "waitLeftMs 110000 -> %u (saturated)", m.waitLeftMs);
  o.waitLeftMs = 65534u;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.waitLeftMs == 65534, "65534 as is");
}

/* AUTO_STATUS bytes 96-135 (docs/autonomy-a2b.md 22): the state machine's report and the controller's bits. */
static void testStatusReport(void)
{
  autoFsmOut_t o;
  vzOutput_t vz;
  autoStatusExtra_t x;
  autoStatusMsg_t m;
  autoFsmReport_t r;
  memset(&o, 0, sizeof(o));
  memset(&vz, 0, sizeof(vz));
  memset(&x, 0, sizeof(x));
  memset(&r, 0, sizeof(r));
  o.state = AUTO_HOVER;
  o.airMs = 41234u;
  vz.phase = VZ_HOLD;
  vz.camHealthy = true;
  vz.khat = 165000.0f;
  vz.capQ = 40123.4f;
  x.kPrior = 165000.0f;
  x.upMs = 100000u;
  /* no report: the block's report fields 0, no command yet */
  memset(&m, 0xA5, sizeof(m));
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.evCode == 0 && m.evCount == 0 && m.evAgeMs == 0 && m.ev1Code == 0 && m.ev2Code == 0 && m.holdBits == 0 &&
        m.camBits == 0 && m.camOffMs == 0 && m.sinkInMs == 0 && m.contacts == 0 && m.loopGaps == 0,
        "no report: 0 (ev %u n %u hb 0x%02x)", m.evCode, m.evCount, m.holdBits);
  CHECK(m.sinceCmdDs == 65535 && m.airDs == 412 && m.khatPm == 1000 && m.capQ == 40123,
        "no command 65535, air 41.2 s, Khat 1.000 x prior, capQ 40123 (%u %u %u %u)", m.sinceCmdDs, m.airDs,
        m.khatPm, m.capQ);
  /* the controller's bits only while it runs this sortie */
  vz.atCap = true;
  vz.camHealthy = false;
  vz.khat = 165000.0f * 1.12f;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.holdBits == 0, "controller not active: no controller bits (0x%02x)", m.holdBits);
  x.vzActive = true;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.holdBits == (AUTO_HB_K_CLAMP | AUTO_HB_CAPPED | AUTO_HB_OPEN_LOOP) && m.khatPm == 1120,
        "Khat 1.12 x prior at its clamp, at a cap, HOLD without the camera: 0x%02x (%u)", m.holdBits, m.khatPm);
  vz.khat = 165000.0f * 1.10f;
  vz.atCap = false;
  vz.camHealthy = true;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.holdBits == 0, "Khat 1.10 x prior, camera: no bits (0x%02x)", m.holdBits);
  vz.khat = 165000.0f * 0.88f;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.holdBits == AUTO_HB_K_CLAMP, "Khat 0.88 x prior: at its lower bound (0x%02x)", m.holdBits);
  vz.khat = 165000.0f;
  vz.phase = VZ_CLIMB;
  vz.camHealthy = false;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.holdBits == 0, "CLIMB without the camera is not the open-loop hold (0x%02x)", m.holdBits);
  vz.phase = VZ_HOLD;
  vz.camHealthy = true;
  /* the report: three events newest first, ages from the FSM clock */
  r.evCount = 4242u;
  r.evCode[0] = AUTO_EV_CONTACT;
  r.evDetail[0] = 0x81;
  r.evMs[0] = 99000u;
  r.evCode[1] = AUTO_EV_CAM_LOST;
  r.evDetail[1] = 3;
  r.evMs[1] = 20000u;
  r.evCode[2] = AUTO_EV_VZ;
  r.evDetail[2] = VZ_R_RESUME;
  r.evMs[2] = 100500u;   /* after now (a clock wrap or a stale copy): 0 */
  r.holdBits = AUTO_HB_CAM_LOST | AUTO_HB_HUSH;
  r.camBits = AUTO_CB_NO_FRAMES | AUTO_CB_CUSUM;
  r.contacts = 7;
  r.loopGaps = 2;
  r.camOutMs = 98000u;
  r.sinkAtMs = 103000u;
  r.peakAccG = 3.456f;
  r.peakRateDps = 812.4f;
  r.contactAxG = -1.234f;
  r.contactAyG = 0.5f;
  r.contactYawDeg = -45.678f;
  r.lastCmdMs = 95000u;
  r.lastCmdAny = true;
  x.rep = &r;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.evCode == AUTO_EV_CONTACT && m.evDetail == 0x81 && m.evCount == 4242u && m.evAgeMs == 1000,
        "newest event, its detail, the count, 1.0 s ago (%u %u %u %u)", m.evCode, m.evDetail, m.evCount,
        m.evAgeMs);
  CHECK(m.ev1Code == AUTO_EV_CAM_LOST && m.ev1Detail == 3 && m.ev1AgeMs == 65535 && m.ev2Code == AUTO_EV_VZ &&
        m.ev2Detail == VZ_R_RESUME && m.ev2AgeMs == 0, "older events: 80 s saturated, a future time 0 (%u %u)",
        m.ev1AgeMs, m.ev2AgeMs);
  CHECK(m.holdBits == (AUTO_HB_CAM_LOST | AUTO_HB_HUSH) && m.camBits == (AUTO_CB_NO_FRAMES | AUTO_CB_CUSUM),
        "hold and camera bits (0x%02x 0x%02x)", m.holdBits, m.camBits);
  CHECK(m.camOffMs == 2000 && m.sinkInMs == 3000, "camera out 2.0 s, SINK in 3.0 s (%u %u)", m.camOffMs, m.sinkInMs);
  CHECK(m.peakAccCg == 346 && m.peakRateDps == 812 && m.contactAxCg == -123 && m.contactAyCg == 50 &&
        m.contactYawCdeg == -4568, "peaks and the contact's direction (%u %u %d %d %d)", m.peakAccCg,
        m.peakRateDps, m.contactAxCg, m.contactAyCg, m.contactYawCdeg);
  CHECK(m.sinceCmdDs == 50 && m.contacts == 7 && m.loopGaps == 2, "5.0 s since the last command (%u)", m.sinceCmdDs);
  /* the camera back: no out / sink times */
  r.holdBits = 0;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.camOffMs == 0 && m.sinkInMs == 0 && m.holdBits == 0, "camera in: out and sink times 0 (%u %u)", m.camOffMs,
        m.sinkInMs);
  /* SINK due: 0 left; a command 2 h ago: 65534 */
  r.holdBits = AUTO_HB_CAM_LOST;
  r.sinkAtMs = 90000u;
  r.lastCmdMs = 100000u - 7200000u;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.sinkInMs == 0 && m.sinceCmdDs == 65534, "sink time passed 0, an old command 65534 (%u %u)", m.sinkInMs,
        m.sinceCmdDs);
  /* non-finite evidence: 0 */
  r.peakAccG = NAN;
  r.contactAxG = INFINITY;
  r.contactYawDeg = NAN;
  vz.capQ = NAN;
  x.kPrior = 0.0f;
  autoStatusFill(&m, &o, &vz, &x);
  CHECK(m.peakAccCg == 0 && m.contactAxCg == 32767 && m.contactYawCdeg == 0 && m.capQ == 0 && m.khatPm == 0,
        "NaN 0, +inf saturated, no prior Khat 0 (%u %d %d %u %u)", m.peakAccCg, m.contactAxCg, m.contactYawCdeg,
        m.capQ, m.khatPm);
}

static void testTripBits(void)
{
  autoFsmIn_t in;
  memset(&in, 0, sizeof(in));
  in.tiltDeg = 1.0f;
  in.accNormG = 1.0f;
  in.vbat = 3.9f;
  CHECK(autoTripBits(&in, false) == AUTO_TRF_REST, "level and still: rest only (0x%02x)", autoTripBits(&in, false));
  CHECK(autoTripBits(&in, true) == (AUTO_TRF_REST | AUTO_TRF_VZ_CAP), "vz at a ceiling");
  in.tiltDeg = 30.5f;
  CHECK(autoTripBits(&in, false) == (AUTO_TRF_TILT | AUTO_TRF_MOVED | AUTO_TRF_REST), "tilt > 30: tilt, moved");
  in.tiltDeg = 30.0f;
  CHECK(!(autoTripBits(&in, false) & AUTO_TRF_TILT), "30.0 is not over 30");
  in.tiltDeg = 1.0f;
  in.gyroDps[1] = -501.0f;
  CHECK(autoTripBits(&in, false) == (AUTO_TRF_RATE | AUTO_TRF_MOVED), "rate: also moved, not at rest");
  in.gyroDps[1] = 0.0f;
  in.gyroDps[2] = 600.0f;
  CHECK(!(autoTripBits(&in, false) & AUTO_TRF_RATE), "yaw rate is not the rate trip");
  in.gyroDps[2] = 0.0f;
  in.accNormG = 0.39f;
  CHECK(autoTripBits(&in, false) == (AUTO_TRF_FALL | AUTO_TRF_MOVED), "free fall");
  in.accNormG = 1.0f;
  in.attErrDeg = 10.5f;
  CHECK(autoTripBits(&in, false) == (AUTO_TRF_ATTERR | AUTO_TRF_REST), "tracking error");
  in.attErrDeg = 0.0f;
  in.vbat = AUTO_VBAT_LAND_V - 0.01f;
  CHECK(autoTripBits(&in, false) == (AUTO_TRF_VBAT_LOW | AUTO_TRF_REST), "battery");
  in.vbat = 3.9f;
  in.accNormG = 1.12f;
  CHECK(autoTripBits(&in, false) == (AUTO_TRF_MOVED | AUTO_TRF_REST), "acc 0.12 g off: moved, still at rest");
  in.accNormG = NAN;
  CHECK(autoTripBits(&in, false) == AUTO_TRF_MOVED, "NaN: moved, not at rest");
}

int main(void)
{
  testRtcLock();
  testRtcRecord();
  testRtcUsbUnlock();
  testSettings();
  testBuildSetpoint();
  testSelectThrust();
  testBarrierMax();
  testClampThrust();
  testAges();
  testTilt();
  testIdleSkip();
  testAppHook();
  testSince();
  testStatusFill();
  testStatusReport();
  testTripBits();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
