/*
 * Replay of the logged autonomy sorties through the in-air contact trip's
 * predicate (auto_fsm.h AUTO_COLLIDE_*: LAND COLLISION, KILLED in
 * 98cd4ae..eb9264d; docs/autonomy-a2.md section 3.2 and 12) and through the
 * real state machine (auto_fsm.c).
 *
 * Build and run from Firmware/esp-drone (skipped without the CSV directory):
 *   C=components/core/crazyflie/utils
 *   cc -std=c99 -O2 -Wall -Wextra -I$C/interface tools/tests/test_collision_replay.c $C/src/auto_fsm.c \
 *      $C/src/auto_motion.c -lm -o /tmp/test_collision_replay && \
 *   /tmp/test_collision_replay ../../firmware-backups/cal
 *
 * Input: the 100 Hz recorder files of the autonomy sorties. Every row is held
 * for ten 1 ms steps (the FSM runs at the loop rate; the recorder keeps every
 * 10th sample, so its peaks are lower bounds of the loop's). The condition
 * (|acc| > AUTO_COLLIDE_ACC_G or max(|gx|, |gy|) > AUTO_COLLIDE_DPS) must hold
 * AUTO_COLLIDE_MS, armed while the controller is in CLIMB or HOLD from
 * AUTO_COLLIDE_ARM_MS after its lift-off (the first CLIMB row), times in
 * recorder ticks.
 *
 * Checks: 2026-10-13 flight 1 trips first at its first contact, 34.45 s from
 * HOVER entry (tick 79020; the drone's motors ran on through five contacts
 * for 0.86 s), never earlier (the brush at 12.71 s stays below); no trip in
 * the CLIMB/HOLD of 2026-10-13 flight 2 nor of the eight clean 2026-10-12
 * sorties. Prints each sortie's highest armed |acc| and gx/gy rate (the
 * margins: 1.16 g and 77 dps before the contact).
 *
 * The state machine replay: the FSM is armed and launched at the sortie's
 * logged ARM and TAKEOFF (FLIGHT, 1.20 m, no photo, a 60 s wait), its inputs
 * from the rows from the controller's spin on (|acc|, the body-frame x/y
 * force, gx/gy/gz, roll/pitch, the controller's logged phase, its tAir from
 * the first CLIMB row; the camera and its height estimate healthy, the
 * battery and the links fine). Since 2026-10-15 (docs/autonomy-a2b.md 22; a
 * LAND COLLISION from 2026-10-13) an in-air contact with the camera height in
 * use holds and reports: flight 1 gets its first CONTACT event at the
 * predicate's tick, stays in HOVER with the hush on (vzInput_t.hush) and the
 * contact's direction recorded (body x/y force at the |acc| peak), never
 * LAND COLLISION nor KILLED on that step; the clean sorties get no CONTACT
 * event at all.
 *
 * The same over every autonomy sortie of 2026-10-12/13 (all the recorder
 * files of those days): no CONTACT event in a clean sortie's free flight,
 * none before a contact sortie's logged contact. (The palm cue's replay,
 * 2026-10-14, went with the cue on 2026-10-15.)
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auto_fsm.h"

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

#define MAX_ROWS 20000   /* a 180 s recorder dump (docs/autonomy-a2b.md 19.3) */

enum { C_TICK, C_FID, C_GX, C_GY, C_AX, C_AY, C_AZ, C_PHASE, C_AUTO, C_GZ, C_ROLL, C_PITCH, C_N };
static const char *colNames[C_N] = {
  "tickMs", "flightId", "gx_ddps", "gy_ddps", "ax_mg", "ay_mg", "az_mg", "vzPhase", "autoState", "gz_ddps",
  "roll_cdeg", "pitch_cdeg",
};
static long csv[MAX_ROWS][C_N];
static int csvRows;

static bool loadCsv(const char *path)
{
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    return false;
  }
  static char line[8192];
  int idx[C_N];
  for (int c = 0; c < C_N; c++) {
    idx[c] = -1;
  }
  if (fgets(line, sizeof(line), f) == NULL) {
    fclose(f);
    return false;
  }
  int col = 0;
  for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
    for (int c = 0; c < C_N; c++) {
      if (strcmp(tok, colNames[c]) == 0) {
        idx[c] = col;
      }
    }
  }
  for (int c = 0; c < C_N; c++) {
    if (idx[c] < 0) {
      fclose(f);
      return false;
    }
  }
  csvRows = 0;
  while (csvRows < MAX_ROWS && fgets(line, sizeof(line), f) != NULL) {
    col = 0;
    for (char *tok = strtok(line, ",\r\n"); tok != NULL; tok = strtok(NULL, ",\r\n"), col++) {
      for (int c = 0; c < C_N; c++) {
        if (idx[c] == col) {
          csv[csvRows][c] = strtol(tok, NULL, 10);
        }
      }
    }
    csvRows++;
  }
  fclose(f);
  return true;
}

typedef struct {
  long tripTick;      /* first step the trip fires (-1: never) */
  long hoverTick;     /* first HOVER row (-1: none) */
  double accMax, gyroMax;   /* armed steps before the trip */
  int armedRows;
} replay_t;

static replay_t replayFlight(int fid)
{
  replay_t R = { -1, -1, 0.0, 0.0, 0 };
  long liftTick = -1, sinceTick = 0;
  bool on = false;
  for (int i = 0; i < csvRows; i++) {
    const long *r = csv[i];
    if (r[C_FID] != fid) {
      continue;
    }
    if (R.hoverTick < 0 && r[C_AUTO] == AUTO_HOVER) {
      R.hoverTick = r[C_TICK];
    }
    const int ph = (int)r[C_PHASE];
    if (liftTick < 0 && ph == VZ_CLIMB) {
      liftTick = r[C_TICK];
    }
    const double acc = sqrt((double)(r[C_AX] * r[C_AX] + r[C_AY] * r[C_AY] + r[C_AZ] * r[C_AZ])) / 1000.0;
    const double gx = fabs((double)r[C_GX]) / 10.0, gy = fabs((double)r[C_GY]) / 10.0;
    const bool cond = acc > AUTO_COLLIDE_ACC_G || gx > AUTO_COLLIDE_DPS || gy > AUTO_COLLIDE_DPS;
    bool rowArmed = false;
    for (int k = 0; k < 10; k++) {
      const long t = r[C_TICK] + k;
      if (!cond) {
        on = false;
      } else if (!on) {
        on = true;
        sinceTick = t;
      }
      const bool armed = (ph == VZ_CLIMB || ph == VZ_HOLD) && liftTick >= 0 && t - liftTick >= AUTO_COLLIDE_ARM_MS;
      if (armed && R.tripTick < 0) {
        rowArmed = true;
        if (on && t - sinceTick >= AUTO_COLLIDE_MS) {
          R.tripTick = t;
        }
      }
    }
    if (rowArmed && R.tripTick < 0) {
      R.armedRows++;
      R.accMax = fmax(R.accMax, acc);
      R.gyroMax = fmax(R.gyroMax, fmax(gx, gy));
    }
  }
  return R;
}

/* ---- the same sortie through the state machine ------------------------------ */

typedef struct {
  long exitTick;            /* first step the FSM left TAKEOFF/HOVER after the spin (-1: never) */
  autoState_t exitState;
  autoReason_t exitReason;
  bool collided, hoverSeen, landThrust, holdBlock;
  long contactTick;         /* the first CONTACT event's step (-1: none) */
  int contacts;             /* CONTACT events up to the exit (or the log's end) */
  autoState_t contactState; /* the state right after the first CONTACT */
  uint8_t contactDetail;
  bool contactHush;         /* vzInput_t.hush asked on that step */
  float contactAx, contactAy;   /* the report's body-frame force at the end of the first hush */
} fsmReplay_t;

static void fsmInputsFine(autoFsmIn_t *in)
{
  in->sessionValid = true;
  in->hbAgeMs = 20;
  in->appAgeMs = 20;
  in->appInputAgeMs = 60000;
  in->appGestureAgeMs = 3000;
  in->appThrustZero = true;
  in->autoEnabled = in->appRequired = in->gestureRequired = true;
  in->vmin = AUTO_VMIN_DEFAULT;
  in->airMaxNvsMs = AUTO_AIR_MAX_DEFAULT_MS;
  in->hoverWaitNvsMs = AUTO_HOVER_WAIT_DEFAULT_MS;
  in->hoverAccelNvsMs = AUTO_HOVER_ACCEL_DEFAULT_MS;
  in->vzParamsOk = true;
  in->loopGapUs = 1000;
  in->vbat = 3.95f;
  in->vbatFastMin = 3.80f;
  in->camState = 2;   /* CAMERA_FLOW_RUNNING */
  in->camFps = 15.0f;
  in->camFrameAgeMs = 40;
  in->camViewMs = 30000;
  in->holdEnabled = in->holdApplied = true;
  in->motorsReady = in->motorGuardOn = true;
  in->heapIntFree = 20000;
}

static void fsmImuStill(autoFsmIn_t *in)
{
  in->rollDeg = in->pitchDeg = 0.3f;
  in->tiltDeg = 0.4f;
  in->attErrDeg = 0.2f;
  in->gyroDps[0] = in->gyroDps[1] = in->gyroDps[2] = 0.1f;
  in->accNormG = 1.0f;
}

static fsmReplay_t replayFsm(int fid)
{
  fsmReplay_t R = { -1, AUTO_IDLE, AUTO_RS_NONE, false, false, false, false, -1, 0, AUTO_IDLE, 0, false, 0.0f, 0.0f };
  int first = -1, armRow = -1, toRow = -1;
  long liftTick = -1;
  for (int i = 0; i < csvRows; i++) {
    if (csv[i][C_FID] != fid) {
      continue;
    }
    if (first < 0) {
      first = i;
    }
    if (armRow < 0 && csv[i][C_AUTO] == AUTO_ARMED) {
      armRow = i;
    }
    if (toRow < 0 && csv[i][C_AUTO] == AUTO_TAKEOFF) {
      toRow = i;
    }
    if (liftTick < 0 && csv[i][C_PHASE] == VZ_CLIMB) {
      liftTick = csv[i][C_TICK];
    }
  }
  if (armRow < 0 || toRow < 0 || liftTick < 0) {
    return R;
  }
  static autoFsm_t f;
  autoFsmIn_t in;
  autoFsmOut_t out;
  autoFsmInit(&f);
  memset(&in, 0, sizeof(in));
  memset(&out, 0, sizeof(out));
  fsmInputsFine(&in);
  fsmImuStill(&in);
  in.randToken = 0x5EEDu;
  in.onGround = true;
  in.onGroundMs = 30000;
  /* ARM 10 ms before the logged ARMED row, TAKEOFF 10 ms before the TAKEOFF row */
  in.nowMs = (uint32_t)csv[armRow][C_TICK] - 10u;
  in.cmd.type = AUTO_CMD_ARM;
  in.cmd.mode = AUTO_MODE_FLIGHT;
  autoFsmStep(&f, &in, &out);
  if (out.state != AUTO_ARMED) {
    return R;
  }
  const uint32_t token = out.cmdDetail;
  memset(&in.cmd, 0, sizeof(in.cmd));
  bool spun = false, holdBlock = false;
  for (int i = armRow; i < csvRows && R.exitTick < 0; i++) {
    const long *r = csv[i];
    if (r[C_FID] != fid) {
      continue;
    }
    if (i > toRow && r[C_AUTO] == AUTO_IDLE) {
      break;   /* the logged sortie ended */
    }
    for (int k = 0; k < 10 && R.exitTick < 0; k++) {
      const long t = r[C_TICK] + k;
      in.nowMs = (uint32_t)t;
      memset(&in.cmd, 0, sizeof(in.cmd));
      if (i == toRow && k == 0) {
        in.nowMs = (uint32_t)t - 10u;
        in.cmd.type = AUTO_CMD_TAKEOFF;
        in.cmd.token = token;
        in.cmd.heightCm = 120;
        in.cmd.photoN = 0;
        in.cmd.photoGapMs = 300;
        in.cmd.actWaitMs = 60000;
      }
      in.onGround = !spun;
      in.onGroundMs = spun ? 0 : 30000;
      const int ph = (int)r[C_PHASE];
      memset(&in.vz, 0, sizeof(in.vz));
      if (spun) {
        /* the logged controller: phase, tAir from its first CLIMB row, a healthy camera height after lift-off */
        in.vz.phase = (vzPhase_t)ph;
        in.vz.g0 = 1.0f;
        in.vz.liftedOff = t >= liftTick;
        in.vz.tAir = t >= liftTick ? (float)(t - liftTick) * 0.001f : 0.0f;
        in.vz.camHealthy = in.vz.liftedOff;
        in.vz.holdReached = ph >= VZ_HOLD;
        in.vz.holdSteady = ph == VZ_HOLD;
        in.vz.thrust = 40000.0f;
        /* the logged IMU */
        const double ax = (double)r[C_AX] / 1000.0, ay = (double)r[C_AY] / 1000.0, az = (double)r[C_AZ] / 1000.0;
        const double an = sqrt(ax * ax + ay * ay + az * az);
        in.accNormG = (float)an;
        in.accXG = (float)ax;
        in.accYG = (float)ay;
        in.gyroDps[0] = (float)r[C_GX] / 10.0f;
        in.gyroDps[1] = (float)r[C_GY] / 10.0f;
        in.gyroDps[2] = (float)r[C_GZ] / 10.0f;
        in.rollDeg = (float)r[C_ROLL] / 100.0f;
        in.pitchDeg = (float)r[C_PITCH] / 100.0f;
        in.tiltDeg = an > 0.0 ? (float)(acos(fmin(1.0, fabs(az) / an)) * 57.2957795) : 0.0f;
        in.attErrDeg = 0.2f;
      } else {
        fsmImuStill(&in);
      }
      in.hbAgeMs = (uint32_t)(t % 100);
      in.appAgeMs = (uint32_t)(t % 50);
      const autoState_t before = out.state;
      const uint16_t evN = autoFsmReport(&f)->evCount;
      autoFsmStep(&f, &in, &out);
      const autoFsmReport_t *rep = autoFsmReport(&f);
      if (rep->evCount != evN && rep->evCode[0] == AUTO_EV_CONTACT) {
        R.contacts++;
        if (R.contactTick < 0) {
          R.contactTick = (long)in.nowMs;
          R.contactState = out.state;
          R.contactDetail = rep->evDetail[0];
          R.contactHush = out.hush;
        }
      }
      if (R.contactTick >= 0 && (long)in.nowMs - R.contactTick == AUTO_CONTACT_HUSH_MS - 1) {
        R.contactAx = rep->contactAxG;
        R.contactAy = rep->contactAyG;
      }
      if (out.vzReset) {
        spun = true;
      }
      holdBlock = out.state == AUTO_LAND && (holdBlock || out.collided || in.accNormG > AUTO_TD_CONTACT_G);
      R.hoverSeen = R.hoverSeen || out.state == AUTO_HOVER;
      if (spun && (before == AUTO_TAKEOFF || before == AUTO_HOVER) && out.state != before && out.state != AUTO_HOVER) {
        R.exitTick = (long)in.nowMs;
        R.exitState = out.state;
        R.exitReason = out.reason;
        R.collided = out.collided;
        R.landThrust = out.thrust == AUTO_THR_VERT_LAND && out.vzCmd == VZ_CMD_LAND && !out.setEmergencyStop;
        R.holdBlock = holdBlock;
      }
    }
  }
  return R;
}

int main(int argc, char **argv)
{
  if (argc < 2) {
    printf("replay skipped: pass the directory with the flight-2026-10-1x-*.csv files (firmware-backups/cal)\n");
    printf("%d checks, %d failures\n", checks, failures);
    return 0;
  }
  const char *dir = argv[1];
  static const struct { const char *file; int fid; bool contact; } sorties[] = {
    { "flight-2026-10-12-a2b-hops.csv", 1, false }, { "flight-2026-10-12-a2b-hops.csv", 2, false },
    { "flight-2026-10-12-calib1.csv", 1, false },   { "flight-2026-10-12-calib2.csv", 1, false },
    { "flight-2026-10-12-evening.csv", 4, false },  { "flight-2026-10-12-hop5s.csv", 3, false },
    { "flight-2026-10-12-mission5s.csv", 1, false }, { "flight-2026-10-12-mission60.csv", 1, false },
    { "flight-2026-10-13-interactive.csv", 2, false }, { "flight-2026-10-13-interactive.csv", 1, true },
  };
  double accMax = 0.0, gyroMax = 0.0;
  int n = 0;
  for (unsigned k = 0; k < sizeof(sorties) / sizeof(sorties[0]); k++) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, sorties[k].file);
    const bool ok = loadCsv(path);
    CHECK(ok, "%s", path);
    if (!ok) {
      continue;
    }
    const replay_t R = replayFlight(sorties[k].fid);
    n++;
    accMax = fmax(accMax, R.accMax);
    gyroMax = fmax(gyroMax, R.gyroMax);
    const double tH = R.tripTick >= 0 && R.hoverTick >= 0 ? (R.tripTick - R.hoverTick) / 1000.0 : NAN;
    printf("  %-34s f%d: armed CLIMB/HOLD %4d rows, max |acc| %.2f g, max gx/gy %3.0f dps before any trip; %s",
           sorties[k].file + 7, sorties[k].fid, R.armedRows, R.accMax, R.gyroMax,
           R.tripTick >= 0 ? "COLLISION at " : "no trip\n");
    if (R.tripTick >= 0) {
      printf("tick %ld (%.3f s from HOVER entry)\n", R.tripTick, tH);
    }
    CHECK(R.armedRows > 100, "%s f%d: %d armed rows", sorties[k].file, sorties[k].fid, R.armedRows);
    if (sorties[k].contact) {
      CHECK(R.tripTick >= 79020 && R.tripTick <= 79030 && fabs(tH - 34.45) <= 0.01,
            "flight 1: the trip fires first at the first contact (tick 79020, 34.45 s from HOVER): tick %ld, %.3f s",
            R.tripTick, tH);
    } else {
      CHECK(R.tripTick < 0, "%s f%d: no trip in CLIMB/HOLD (fired at tick %ld)", sorties[k].file, sorties[k].fid,
            R.tripTick);
    }
    const fsmReplay_t F = replayFsm(sorties[k].fid);
    printf("    state machine: %s, %d CONTACT events (first at tick %ld in %s, detail 0x%02x, hush %d, force x %+.2f "
           "y %+.2f g); left at tick %ld: %s %s%s\n", F.hoverSeen ? "HOVER" : "no HOVER", F.contacts, F.contactTick,
           autoStateName(F.contactState), F.contactDetail, F.contactHush, F.contactAx, F.contactAy, F.exitTick,
           autoStateName(F.exitState), autoReasonName(F.exitReason), F.collided ? " (collided)" : "");
    if (sorties[k].contact) {
      CHECK(F.hoverSeen && F.contactTick == R.tripTick && F.contactState == AUTO_HOVER && F.contactHush &&
            (F.contactDetail & 0x3Fu) == VZ_HOLD && (F.exitTick < 0 || F.exitTick > F.contactTick) &&
            !(F.exitReason == AUTO_RS_COLLISION && F.exitTick == F.contactTick),
            "flight 1 through the state machine: the first CONTACT event at the predicate's tick %ld (%ld), in HOVER "
            "with the hush, the controller in HOLD (0x%02x), no LAND or KILL on that step", R.tripTick, F.contactTick,
            F.contactDetail);
      CHECK(hypotf(F.contactAx, F.contactAy) > 0.3f, "flight 1: the contact's body-frame force recorded (%+.2f, %+.2f g)",
            F.contactAx, F.contactAy);
    } else {
      CHECK(F.hoverSeen && F.contacts == 0 && F.exitReason != AUTO_RS_COLLISION && !F.collided,
            "%s f%d through the state machine: no CONTACT event, never COLLISION (%d; %s %s)", sorties[k].file,
            sorties[k].fid, F.contacts, autoStateName(F.exitState), autoReasonName(F.exitReason));
    }
  }
  printf("contact trip replay: %d sorties; highest armed |acc| %.2f g and gx/gy %.0f dps outside the contact "
         "(thresholds %.1f g, %.0f dps, %d ms)\n", n, accMax, gyroMax, (double)AUTO_COLLIDE_ACC_G,
         (double)AUTO_COLLIDE_DPS, AUTO_COLLIDE_MS);
  CHECK(n == 10 && accMax < AUTO_COLLIDE_ACC_G - 0.3 && gyroMax < AUTO_COLLIDE_DPS - 100.0,
        "margins: %.2f g, %.0f dps", accMax, gyroMax);

  /* Every autonomy sortie of 2026-10-12/13 through the state machine (duplicates of a sortie in two files are
   * harmless). contactS: a sortie that ended in an in-air contact, the contact's time from its first row (the logged
   * COLLISION's row; 0: the contact trip's predicate above, 2026-10-13 flight 1, flown before the trip existed;
   * battfwd2:1: the analyst's contact, no COLLISION logged), -1 a clean one. geFromS: from here the drone sank into
   * the floor's ground effect in HOVER (hop5s:3, an accelerometer-only hover), not free flight. A CONTACT event in a
   * clean sortie's free flight, or before a contact sortie's contact, is a false hold-and-report. */
  static const struct { const char *file; int fid; float contactS, geFromS; } all[] = {
    { "flight-2026-10-12-a2b-hops.csv", 1, -1, -1 }, { "flight-2026-10-12-a2b-hops.csv", 2, -1, -1 },
    { "flight-2026-10-12-calib1.csv", 1, -1, -1 },   { "flight-2026-10-12-calib2.csv", 1, -1, -1 },
    { "flight-2026-10-12-evening.csv", 4, -1, -1 },  { "flight-2026-10-12-hop5s.csv", 3, -1, 5.60f },
    { "flight-2026-10-12-mission5s.csv", 1, -1, -1 }, { "flight-2026-10-12-mission60.csv", 1, -1, -1 },
    { "flight-2026-10-13-interactive.csv", 1, 0, -1 }, { "flight-2026-10-13-interactive.csv", 2, -1, -1 },
    { "flight-2026-10-13-night.csv", 3, -1, -1 },    { "flight-2026-10-13-night.csv", 6, -1, -1 },
    { "flight-2026-10-13-propswap.csv", 1, -1, -1 }, { "flight-2026-10-13-battfwd.csv", 1, -1, -1 },
    { "flight-2026-10-13-battfwd.csv", 2, 7.21f, -1 }, { "flight-2026-10-13-battfwd2.csv", 1, 2.70f, -1 },
    { "flight-2026-10-13-battfwd2.csv", 2, -1, -1 }, { "flight-2026-10-13-preAB.csv", 2, -1, -1 },
    { "flight-2026-10-13-preAB.csv", 3, -1, -1 },    { "flight-2026-10-13-preAB.csv", 4, 12.09f, -1 },
    { "flight-2026-10-13-pwmA1.csv", 5, -1, -1 },    { "flight-2026-10-13-pwmA1.csv", 6, 7.11f, -1 },
    { "flight-2026-10-13-pwmA1.csv", 7, -1, -1 },    { "flight-2026-10-13-pwmB60.csv", 8, -1, -1 },
    { "flight-2026-10-13-tiger.csv", 1, -1, -1 },    { "flight-2026-10-13-tiger.csv", 2, -1, -1 },
    { "flight-2026-10-13-tiger2.csv", 1, -1, -1 },
  };  int nAll = 0, cleanEvents = 0, contactEarly = 0, contactSeen = 0;
  for (unsigned k = 0; k < sizeof(all) / sizeof(all[0]); k++) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", dir, all[k].file);
    const bool ok = loadCsv(path);
    CHECK(ok, "%s", path);
    if (!ok) {
      continue;
    }
    long first = -1;
    for (int i = 0; i < csvRows; i++) {
      if (csv[i][C_FID] == all[k].fid) {
        first = csv[i][C_TICK];
        break;
      }
    }
    nAll++;
    const replay_t C = replayFlight(all[k].fid);   /* the contact trip's tick (-1: none) */
    const bool contact = all[k].contactS >= 0.0f;
    const long collTick = !contact ? -1 : all[k].contactS > 0.0f ? first + (long)(all[k].contactS * 1000.0f + 0.5f)
                                                                 : C.tripTick;
    const long geTick = all[k].geFromS >= 0.0f ? first + (long)(all[k].geFromS * 1000.0f + 0.5f) : -1;
    const fsmReplay_t F = replayFsm(all[k].fid);
    printf("  %-34s f%d: %d CONTACT events", all[k].file + 7, all[k].fid, F.contacts);
    if (F.contactTick >= 0) {
      printf(", the first at %.2f s", (F.contactTick - first) / 1000.0);
    }
    if (collTick >= 0) {
      printf(" (the logged contact at %.2f s)", (collTick - first) / 1000.0);
    }
    if (F.exitTick >= 0) {
      printf("; left at %.2f s: %s %s", (F.exitTick - first) / 1000.0, autoStateName(F.exitState),
             autoReasonName(F.exitReason));
    }
    printf("\n");
    if (!contact) {
      cleanEvents += F.contactTick >= 0 && (geTick < 0 || F.contactTick < geTick);
    } else {
      contactEarly += F.contactTick >= 0 && collTick >= 0 && F.contactTick < collTick - 20;
      contactSeen += F.contactTick >= 0;
    }
  }
  printf("CONTACT replay: %d sorties; clean sorties with a CONTACT in free flight %d; contact sorties with a CONTACT "
         "before the logged contact %d; contact sorties with a CONTACT %d\n", nAll, cleanEvents, contactEarly,
         contactSeen);
  CHECK(nAll == 27 && cleanEvents == 0, "no CONTACT event in a clean sortie's free flight (%d)", cleanEvents);
  CHECK(contactEarly == 0, "no CONTACT event before a contact sortie's contact (%d)", contactEarly);
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
