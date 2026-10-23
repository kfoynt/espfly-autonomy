/*
 * Host tests for the camera + accelerometer vertical observer
 * (components/core/crazyflie/utils/src/cam_vz_obs.c)
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -O2 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
 *      tools/tests/test_cam_vz_obs.c components/core/crazyflie/utils/src/cam_vz_obs.c \
 *      -lm -o /tmp/test_cam_vz_obs && /tmp/test_cam_vz_obs [-v] [fixture]
 * (-v: one line per replayed take-off)
 *
 * 1. Replay (fixture tools/tests/fixtures/cam_vz_replay.txt, written by
 *    tools/tests/gen_cam_vz_fixture.py from the study's Python observer): the
 *    43 take-offs of the 13 logged camera flights, 100 Hz rows, each run with
 *    the FINAL (default) and the plain configuration. The C observer must
 *    take the same decision for every frame, reach the same counters, and
 *    match the Python state (h, v, offset, sigmas, tracked gain) at every
 *    100th row and the last row within float tolerances.
 * 2. Unit checks on a synthetic flight (smooth climb, slow vertical bob,
 *    accelerometer offset and noise, camera frames at 16.9 fps with 130 ms
 *    latency and pitch rotation): convergence, every gate, gain re-learn after
 *    a yaw to a nearer view, camera loss and recovery, CUSUM, re-lock, the gain
 *    re-anchor (a climb over a scene whose distance grows with height,
 *    2026-10-13 flight 2), the gain through a level move toward the scene
 *    (the frames' fwdLn, 2026-10-15), 1 kHz versus 100 Hz input, faults,
 *    config validation, timing. The replay of point 1 runs with the re-anchor off
 *    (observer.py has none).
 *    Fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31): a commanded
 *    turn's tracker restart shows within 300 ms of its end (the dwell from
 *    mvspeed 200 on), the re-track and re-view bobs at 0.40 m/s^2, the height
 *    estimate through the fast lean's 12 deg/s pitch-rate steps.
 * 3. The late offset noise (qLateMult, R1 of the height study 2026-10-15,
 *    docs/autonomy-a2b.md 25): on the replay, every row of every take-off
 *    bit-identical with it on (x5, the strongest, from 20 s, the earliest
 *    step) until its step, and, with it off, the hash of the FINAL replay's
 *    outputs as the observer of 65dfcba gave it (pinned; built with
 *    -DCVO_BASE against that commit's cam_vz_obs.c). The replay covers the
 *    path BEFORE the step only: 42 of the 43 take-offs end before 20 s and
 *    file 16 fid 3 reaches it 0.25 s before its end, so what the multiplier
 *    does is tested on the synthetic hovers below and in
 *    test_cam_vz_late.c's closed loop. On a 90 s synthetic hover whose
 *    offset drifts like the 10-15 flights': bit-identical until the step,
 *    the offset closer after it, the controller's held offset (biasHeldG)
 *    2.0-4.6 s old; the step's clock from the first lift-off (a ground toggle
 *    mid-flight neither restarts it nor lets the multiplier run on the
 *    ground, nor counts its time as on; a reset does restart it); every
 *    guard event (base noise for 3 s, the offset's variance held at its step
 *    size plus the base noise's growth since the previous frame, the held
 *    offset frozen), a QUALITY refusal not one; a guard run longer than
 *    CAM_VZ_QLATE_FREEZE_US (a held motion) freezes the held offset only that
 *    long, then the delayed mean again; after a 3 s gap the offset's
 *    variance regrown at the base rate, as today's filter; a camera lost
 *    after a glitch the observer took leaves the controller the offset from
 *    before the glitch; the config limits (x5 the most).
 * 4. The healthy offset speed-up (hbMult, the height estimator study of
 *    2026-10-18, docs/autonomy-a2b.md 37): with it off (the default, NVS cvhb
 *    0) the FINAL replay keeps the pinned hash of point 3 (every logged
 *    take-off bit-identical to the observer before any of this); a twin with
 *    x3 differs from it only after its first accepted frame. On a 60 s
 *    synthetic hover whose offset falls 8 mg over 5-25 s (the logged flights'
 *    early fall, the deepest one): the offset state follows closer (the mean
 *    lag over 8-40 s under 0.75 of x1's), the multiplier on for nearly the
 *    whole hover. The health gating: off before the first accepted frame; a
 *    6 s camera gap stops it 2 s after the last accepted frame (the offset's
 *    variance then grows at the base rate, x9 of it while on with no frame);
 *    a CUSUM alarm stops it for its hold; the ground mid-flight stops it
 *    until the first accepted frame after the re-lift; the config limits (0
 *    or 1..8, refused with R1 or T2).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "cam_vz_obs.h"
#include "auto_motion.h"   /* AUTO_MV_TURN_GATE_RPS */

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
    checks++; \
    if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
  } while (0)

static const char decisionCode[] = {'-', '-', 'q', 'e', 'x', 'w', 'g', 'c', 'r', 'A'};

/* ========================================================================= */
/* 1. replay                                                                  */
/* ========================================================================= */

#define MAX_ROWS 4000
#define MAX_CODES 2000
#define MAX_CK 64

/* Tolerances against the Python (double) reference: about 10x the largest
 * float-versus-double difference seen (6.5e-6 m, 1.7e-6 m/s, 2.9e-5 mg,
 * 2.7e-5 relative sigma, 6.4e-6 relative G, 2.2e-7 s). */
#define TOL_H_M      5e-5
#define TOL_V_MS     2e-5
#define TOL_B_MG     5e-4
#define TOL_SIG_REL  2e-4
#define TOL_G_REL    5e-5
#define TOL_TG_S     2e-6

typedef struct {
  int32_t q;              /* (a_up - 1) in 1e-7 g */
  int dms;
  bool frame;
  long seq, age, dt100, ok, tex, res, dx, dy, sc, gx, gy, gz;
} fxRow_t;

typedef struct {
  int row;
  double z, v, b, sz, sv, sb, G;
  long nu, nr;
} fxCk_t;

typedef struct {
  char codes[MAX_CODES + 1];
  fxCk_t ck[MAX_CK];
  int nCk;
  long relocks, alarms, expRej, yawResets;
  double gFirst;
} fxRef_t;

typedef struct {
  int file, fid;
  char grade[4];
  double b0, s0, tLift, tTouch;
  long tCamUs;
  int n;
  fxRow_t rows[MAX_ROWS];
  fxRef_t ref[2];
} fxTakeoff_t;

static fxTakeoff_t tk;

typedef struct {
  double h, v, b, sig, g, tg;
  int takeoffs, rows, frames, cks, codeMismatch, countMismatch;
  /* informational: the healthy flag after the camera wake, airborne rows */
  long airRows, healthyRows, trackedRows;
  int neverHealthy;
  double firstHealthy[64];
  int nFirst;
} replayErr_t;

static bool verbose = false;

/* FNV-1a 64 over the outputs the observer had before the late offset noise (2026-10-15): the same hash with
 * -DCVO_BASE against an older cam_vz_obs.c */
static uint64_t fnv(uint64_t h, const void *p, size_t n)
{
  const unsigned char *b = p;
  for (size_t i = 0; i < n; i++) {
    h ^= b[i];
    h *= 1099511628211ull;
  }
  return h;
}

static uint64_t hashOut(uint64_t h, const camVzObsOut_t *o)
{
  const float f[] = {o->h, o->v, o->biasG, o->sigmaH, o->sigmaV, o->sigmaBiasG, o->gain, o->lastInnov,
                     o->lastInnovSigma};
  const unsigned char b[] = {o->gainTracked, o->healthy, o->healthyGuess, o->airborne, o->fault};
  h = fnv(h, f, sizeof(f));
  h = fnv(h, b, sizeof(b));
  return fnv(h, &o->sinceAcceptUs, sizeof(o->sinceAcceptUs));
}

/* the FINAL replay's hash with the observer of 65dfcba (cam_vz_obs.c as flashed in 7a78e1b) */
#define REPLAY_HASH_BASE 0xe0ff64f3b4b2983dull
static uint64_t replayHash = 14695981039346656037ull;
static int lateRowsDiff, lateRows, lateRowsPast, lateStepEarly;
/* the healthy offset speed-up's twin (x3): rows differing before / after its first accepted frame */
static int hbRowsBefore, hbRowsDiffBefore, hbRowsAfter, hbRowsDiffAfter;

static int cmpDouble(const void *a, const void *b)
{
  const double x = *(const double *)a, y = *(const double *)b;
  return x < y ? -1 : x > y;
}

static bool readLine(FILE *f, char *buf, int n)
{
  while (fgets(buf, n, f) != NULL) {
    if (buf[0] != '#' && buf[0] != '\n') {
      return true;
    }
  }
  return false;
}

/* Reads one take-off; false at the end of the file. */
static bool readTakeoff(FILE *f, char *line, int lineLen, bool *haveLine)
{
  if (!*haveLine && !readLine(f, line, lineLen)) {
    return false;
  }
  *haveLine = false;
  if (line[0] != 'T') {
    return false;
  }
  memset(&tk, 0, sizeof(tk));
  if (sscanf(line, "T %d %d %3s %lf %lf %ld %lf %lf %d", &tk.file, &tk.fid, tk.grade, &tk.b0, &tk.s0, &tk.tCamUs,
             &tk.tLift, &tk.tTouch, &tk.n) != 9 || tk.n <= 0 || tk.n > MAX_ROWS) {
    printf("bad T line: %s", line);
    return false;
  }
  for (int i = 0; i < tk.n; i++) {
    if (!readLine(f, line, lineLen)) {
      return false;
    }
    fxRow_t *r = &tk.rows[i];
    char *p = line;
    r->q = (int32_t)strtol(p, &p, 10);
    r->dms = 10;
    if (*p == '/') {
      r->dms = (int)strtol(p + 1, &p, 10);
    }
    long v[12];
    int k = 0;
    while (k < 12) {
      char *e;
      long x = strtol(p, &e, 10);
      if (e == p) {
        break;
      }
      v[k++] = x;
      p = e;
    }
    if (k == 12) {
      r->frame = true;
      r->seq = v[0]; r->age = v[1]; r->dt100 = v[2]; r->ok = v[3]; r->tex = v[4]; r->res = v[5];
      r->dx = v[6]; r->dy = v[7]; r->sc = v[8]; r->gx = v[9]; r->gy = v[10]; r->gz = v[11];
    } else if (k != 0) {
      printf("bad row: %s", line);
      return false;
    }
  }
  for (int c = 0; c < 2; c++) {
    tk.ref[c].nCk = 0;
  }
  while (readLine(f, line, lineLen)) {
    int c;
    if (line[0] == 'D') {
      char codes[MAX_CODES + 1];
      if (sscanf(line, "D %d %2000s", &c, codes) != 2 || c < 0 || c > 1) {
        return false;
      }
      strcpy(tk.ref[c].codes, strcmp(codes, "-") == 0 ? "" : codes);
    } else if (line[0] == 'K') {
      fxCk_t k;
      char gs[32];
      if (sscanf(line, "K %d %d %lf %lf %lf %lf %lf %lf %31s %ld %ld", &c, &k.row, &k.z, &k.v, &k.b, &k.sz, &k.sv,
                 &k.sb, gs, &k.nu, &k.nr) != 11 || c < 0 || c > 1 || tk.ref[c].nCk >= MAX_CK) {
        return false;
      }
      k.G = strcmp(gs, "nan") == 0 ? NAN : atof(gs);
      tk.ref[c].ck[tk.ref[c].nCk++] = k;
    } else if (line[0] == 'E') {
      char gs[32];
      fxRef_t *rf;
      if (sscanf(line, "E %d", &c) != 1 || c < 0 || c > 1) {
        return false;
      }
      rf = &tk.ref[c];
      if (sscanf(line, "E %d %ld %ld %ld %ld %31s", &c, &rf->relocks, &rf->alarms, &rf->expRej, &rf->yawResets,
                 gs) != 6) {
        return false;
      }
      rf->gFirst = strcmp(gs, "nan") == 0 ? NAN : atof(gs);
    } else {
      *haveLine = true;
      break;
    }
  }
  return true;
}

static double relErr(double a, double b)
{
  return fabs(a - b) / fmax(fabs(b), 1e-12);
}

static void replayOne(int cfgIdx, replayErr_t *E)
{
  camVzObsConfig_t cfg;
  if (cfgIdx == 0) {
    camVzObsDefaultConfig(&cfg);
    cfg.gainReanchorRel = 0.0f;   /* observer.py has no gain re-anchor (2026-10-13; testGainReanchor) */
  } else {
    camVzObsPlainConfig(&cfg);
  }
  static camVzObs_t o;
  CHECK(camVzObsInit(&o, &cfg), "init");
  CHECK(camVzObsSetBiasPrior(&o, (float)(tk.b0 * 1e-3), (float)(tk.s0 * 1e-3)), "prior");
#ifndef CVO_BASE
  /* the twin with the late offset noise on at its strongest and earliest (every take-off is shorter than 20 s) */
  static camVzObs_t o2;
  camVzObsConfig_t cfg2 = cfg;
  cfg2.qLateMult = CAM_VZ_QLATE_MULT_MAX;
  cfg2.qLateS = CAM_VZ_QLATE_S_MIN;
  CHECK(camVzObsInit(&o2, &cfg2), "init (late)");
  camVzObsSetBiasPrior(&o2, (float)(tk.b0 * 1e-3), (float)(tk.s0 * 1e-3));
  /* the twin with the healthy offset speed-up x3 (the study's step; NVS cvhb 30) */
  static camVzObs_t o3;
  camVzObsConfig_t cfg3 = cfg;
  cfg3.hbMult = 3.0f;
  CHECK(camVzObsInit(&o3, &cfg3), "init (hb)");
  camVzObsSetBiasPrior(&o3, (float)(tk.b0 * 1e-3), (float)(tk.s0 * 1e-3));
#endif
  const uint64_t base = 1234567890123ull;
  long ms = 0;
  bool liftSet = false;
  const long liftUs = lround(tk.tLift * 1e6);
  CHECK((liftUs > (long)cfg.wakeUs ? liftUs : (long)cfg.wakeUs) == tk.tCamUs,
        "file %d fid %d: camera start %ld vs fixture %ld", tk.file, tk.fid, liftUs, tk.tCamUs);
  char codes[MAX_CODES + 1];
  int nc = 0;
  int ck = 0;
  long air = 0, healthy = 0, tracked = 0;
  double firstHealthy = -1.0;
  const fxRef_t *rf = &tk.ref[cfgIdx];
  for (int i = 0; i < tk.n; i++) {
    const fxRow_t *r = &tk.rows[i];
    if (i > 0) {
      ms += r->dms;
    }
    const uint64_t now = base + (uint64_t)ms * 1000u;
    const bool ground = (double)ms / 1000.0 < tk.tLift;
    /* the recorder logs from throttle-up on: thrust > 0 in every row (asserted by the generator) */
    camVzObsImu(&o, now, 1.0f + (float)r->q * 1e-7f, ground, 1.0f);
#ifndef CVO_BASE
    camVzObsImu(&o2, now, 1.0f + (float)r->q * 1e-7f, ground, 1.0f);
    camVzObsImu(&o3, now, 1.0f + (float)r->q * 1e-7f, ground, 1.0f);
#endif
    if (!ground && !liftSet) {
      CHECK(camVzObsSetLiftoffTime(&o, base + (uint64_t)liftUs), "lift-off time");
#ifndef CVO_BASE
      camVzObsSetLiftoffTime(&o2, base + (uint64_t)liftUs);
      camVzObsSetLiftoffTime(&o3, base + (uint64_t)liftUs);
#endif
      liftSet = true;
    }
    camVzObsDecision_t d = CAM_VZ_F_DUPLICATE;
    if (r->frame && i < tk.n - 1) {   /* the last row (first at or after touchdown) is not used */
      camVzObsFrame_t f;
      memset(&f, 0, sizeof(f));
      f.seq = (uint32_t)r->seq;
      f.frameTimeUs = now - (uint64_t)r->age * 1000u + 1000u;   /* the study's +1 ms clock offset */
      f.dtUs = (uint32_t)(r->dt100 * 100);
      f.dx = (float)r->dx / 100.0f;
      f.dy = (float)r->dy / 100.0f;
      f.scale = (float)r->sc / 1e4f;
      f.gyroAngle[0] = (float)r->gx / 1e4f;
      f.gyroAngle[1] = (float)r->gy / 1e4f;
      f.gyroAngle[2] = (float)r->gz / 1e4f;
      f.texture = (float)r->tex;
      f.residual = (float)r->res / 10.0f;
      f.ok = r->ok != 0;
      d = camVzObsFrame(&o, &f, now);
      if (d != CAM_VZ_F_DUPLICATE && d != CAM_VZ_F_SKIPPED && nc < MAX_CODES) {
        codes[nc++] = decisionCode[d];
        E->frames++;
      }
#ifndef CVO_BASE
      const bool before = !o2.late;
      const camVzObsDecision_t d2 = camVzObsFrame(&o2, &f, now);
      lateRowsDiff += before && d2 != d;
      const bool hbBefore = !o3.hasAccept;
      const camVzObsDecision_t d3 = camVzObsFrame(&o3, &f, now);
      hbRowsDiffBefore += hbBefore && d3 != d;
#endif
    }
    {
      camVzObsOut_t out;
      camVzObsGet(&o, &out);
      if (cfgIdx == 0) {
        replayHash = hashOut(fnv(replayHash, &d, sizeof(d)), &out);
      }
#ifndef CVO_BASE
      camVzObsOut_t out2;
      camVzObsGet(&o2, &out2);
      if (!o2.late) {
        lateRowsDiff += memcmp(&out, &out2, sizeof(out)) != 0 || out.biasHeldG != out.biasG;
        lateRows++;
      } else {
        lateRowsPast++;
        lateStepEarly += o2.tState - o2.tFirstLift < 20000000u || o2.tFirstLift != o2.tLift;
      }
      /* the x3 twin: as the FINAL until its first accepted frame (the h, v, offset and sigmas; its hbOn flag apart) */
      camVzObsOut_t out3;
      camVzObsGet(&o3, &out3);
      out3.hbOn = out.hbOn;
      if (!o3.hasAccept) {
        hbRowsDiffBefore += memcmp(&out, &out3, sizeof(out)) != 0;
        hbRowsBefore++;
      } else {
        hbRowsDiffAfter += memcmp(&out, &out3, sizeof(out)) != 0;
        hbRowsAfter++;
      }
#endif
    }
    if (ms * 1000 >= tk.tCamUs && !ground) {
      camVzObsOut_t out;
      camVzObsGet(&o, &out);
      air++;
      healthy += out.healthy;
      tracked += out.gainTracked;
      if (out.healthy && firstHealthy < 0.0) {
        firstHealthy = (ms * 1000 - tk.tCamUs) * 1e-6;
      }
    }
    if (ck < rf->nCk && rf->ck[ck].row == i) {
      const fxCk_t *k = &rf->ck[ck++];
      camVzObsOut_t out;
      camVzObsGet(&o, &out);
      const double dh = fabs(out.h - k->z), dv = fabs(out.v - k->v), db = fabs(out.biasG * 1e3 - k->b);
      const double ds = fmax(relErr(out.sigmaH, k->sz), fmax(relErr(out.sigmaV, k->sv),
                                                              relErr(out.sigmaBiasG * 1e3, k->sb)));
      E->h = fmax(E->h, dh);
      E->v = fmax(E->v, dv);
      E->b = fmax(E->b, db);
      E->sig = fmax(E->sig, ds);
      E->cks++;
      const bool gOk = isnan(k->G) ? !o.gt.hasG : (o.gt.hasG && relErr(o.gt.g, k->G) <= TOL_G_REL);
      if (!isnan(k->G) && o.gt.hasG) {
        E->g = fmax(E->g, relErr(o.gt.g, k->G));
      }
      CHECK(dh <= TOL_H_M && dv <= TOL_V_MS && db <= TOL_B_MG && ds <= TOL_SIG_REL && gOk &&
            (long)o.cnt.accepted == k->nu && (long)o.cnt.rejected == k->nr,
            "file %d fid %d cfg %d row %d: h %.6f/%.6f v %.6f/%.6f b %.5f/%.5f mg sig rel %.2g G %.4f/%.4f "
            "upd %u/%ld rej %u/%ld", tk.file, tk.fid, cfgIdx, i, out.h, k->z, out.v, k->v, out.biasG * 1e3, k->b, ds,
            o.gt.hasG ? o.gt.g : NAN, k->G, (unsigned)o.cnt.accepted, k->nu, (unsigned)o.cnt.rejected, k->nr);
    }
  }
  codes[nc] = 0;
  E->rows += tk.n;
  E->airRows += air;
  E->healthyRows += healthy;
  E->trackedRows += tracked;
  if (firstHealthy < 0.0) {
    E->neverHealthy++;
  } else if (E->nFirst < 64) {
    E->firstHealthy[E->nFirst++] = firstHealthy;
  }
  if (verbose) {
    camVzObsOut_t out;
    camVzObsGet(&o, &out);
    printf("  %s file %2d fid %2d %s: %5.1f s after wake, healthy %3.0f %%, tracked %3.0f %%, first healthy %5.2f s, "
           "yaw restarts %2u, alarms %u, h at touchdown %+.2f m, offset %+.2f mg\n", cfgIdx ? "plain" : "FINAL",
           tk.file, tk.fid, tk.grade, air * 0.01, air ? 100.0 * healthy / air : 0.0,
           air ? 100.0 * tracked / air : 0.0, firstHealthy, (unsigned)o.cnt.yawResets, (unsigned)o.cnt.alarms,
           out.h, out.biasG * 1e3);
  }
  CHECK(ck == rf->nCk, "file %d fid %d cfg %d: %d of %d checkpoints reached", tk.file, tk.fid, cfgIdx, ck, rf->nCk);
  if (strcmp(codes, rf->codes) != 0) {
    E->codeMismatch++;
    int j = 0;
    while (codes[j] && codes[j] == rf->codes[j]) {
      j++;
    }
    CHECK(false, "file %d fid %d cfg %d: decisions differ at frame %d (C '%c', Python '%c'; %d vs %d frames)",
          tk.file, tk.fid, cfgIdx, j, codes[j] ? codes[j] : '$', rf->codes[j] ? rf->codes[j] : '$', nc,
          (int)strlen(rf->codes));
  }
  float tg;
  const bool hasTg = camVzObsFirstGainTime(&o, &tg);
  const bool tgOk = isnan(rf->gFirst) ? !hasTg : (hasTg && fabs(tg - rf->gFirst) <= TOL_TG_S);
  if (hasTg && !isnan(rf->gFirst)) {
    E->tg = fmax(E->tg, fabs(tg - rf->gFirst));
  }
  const bool countsOk = (long)o.cnt.relocks == rf->relocks && (long)o.cnt.alarms == rf->alarms &&
                        (long)(o.cnt.expRejects + o.cnt.latRejects) == rf->expRej &&
                        (long)o.cnt.yawResets == rf->yawResets && tgOk;
  if (!countsOk) {
    E->countMismatch++;
  }
  CHECK(countsOk, "file %d fid %d cfg %d: relocks %u/%ld alarms %u/%ld exp+lat %u/%ld yaw %u/%ld first G %.6f/%.6f",
        tk.file, tk.fid, cfgIdx, (unsigned)o.cnt.relocks, rf->relocks, (unsigned)o.cnt.alarms, rf->alarms,
        (unsigned)(o.cnt.expRejects + o.cnt.latRejects), rf->expRej, (unsigned)o.cnt.yawResets, rf->yawResets,
        hasTg ? tg : NAN, rf->gFirst);
  CHECK(!o.fault, "file %d fid %d cfg %d: fault", tk.file, tk.fid, cfgIdx);
}

static void testReplay(const char *path)
{
  FILE *f = fopen(path, "r");
  CHECK(f != NULL, "replay fixture %s not found (run from Firmware/esp-drone or pass its path)", path);
  if (f == NULL) {
    return;
  }
  static char line[8192];
  bool haveLine = false;
  CHECK(readLine(f, line, sizeof(line)) && strncmp(line, "V 1", 3) == 0, "fixture version");
  replayErr_t E[2];
  memset(E, 0, sizeof(E));
  int files[64];
  int nFiles = 0;
  while (readTakeoff(f, line, sizeof(line), &haveLine)) {
    bool seen = false;
    for (int i = 0; i < nFiles; i++) {
      seen |= files[i] == tk.file;
    }
    if (!seen && nFiles < 64) {
      files[nFiles++] = tk.file;
    }
    for (int c = 0; c < 2; c++) {
      replayOne(c, &E[c]);
      E[c].takeoffs++;
    }
  }
  fclose(f);
  CHECK(nFiles == 13 && E[0].takeoffs == 43, "replay covers %d files, %d take-offs (want 13, 43)", nFiles,
        E[0].takeoffs);
  for (int c = 0; c < 2; c++) {
    qsort(E[c].firstHealthy, (size_t)E[c].nFirst, sizeof(double), cmpDouble);
    printf("replay %-5s: healthy in %.0f %% of the airborne rows after the camera wake (gain tracked %.0f %%); "
           "first healthy after the wake: median %.2f s, max %.2f s; never healthy in %d take-offs\n",
           c == 0 ? "FINAL" : "plain", 100.0 * E[c].healthyRows / fmax(E[c].airRows, 1),
           100.0 * E[c].trackedRows / fmax(E[c].airRows, 1),
           E[c].nFirst ? E[c].firstHealthy[E[c].nFirst / 2] : NAN, E[c].nFirst ? E[c].firstHealthy[E[c].nFirst - 1] : NAN,
           E[c].neverHealthy);
    printf("replay %-5s: %d take-offs, %d rows, %d frames, %d checkpoints; max |dh| %.2e m, |dv| %.2e m/s, "
           "|db| %.2e mg, sigma rel %.2e, G rel %.2e, first-G time %.2e s; decision mismatches %d, counter "
           "mismatches %d\n", c == 0 ? "FINAL" : "plain", E[c].takeoffs, E[c].rows, E[c].frames, E[c].cks, E[c].h,
           E[c].v, E[c].b, E[c].sig, E[c].g, E[c].tg, E[c].codeMismatch, E[c].countMismatch);
  }
  printf("replay FINAL hash %016llx (the observer of 65dfcba: %016llx); late offset noise x%.0f from 20 s: %d of %d "
         "rows before its step differ, %d rows past it\n", (unsigned long long)replayHash,
         (unsigned long long)REPLAY_HASH_BASE, (double)CAM_VZ_QLATE_MULT_MAX, lateRowsDiff, lateRows, lateRowsPast);
  CHECK(replayHash == REPLAY_HASH_BASE, "the FINAL replay (the late offset noise, T2 and the healthy offset speed-up "
        "off: NVS cvqlate 1, cvt2k 0, cvhb 0) bit for bit as before any of them");
  CHECK(lateRowsDiff == 0 && lateRows > 80000 && lateRowsPast > 0 && lateRowsPast <= 60 && lateStepEarly == 0,
        "the late offset noise on: every row and decision bit-identical before its step (%d of %d differ), the step "
        "20 s after the first lift-off (%d rows past it, %d early)", lateRowsDiff, lateRows, lateRowsPast,
        lateStepEarly);
  printf("healthy offset speed-up x3 on the replay: %d of %d rows differ before its first accepted frame, %d of %d "
         "after it\n", hbRowsDiffBefore, hbRowsBefore, hbRowsDiffAfter, hbRowsAfter);
  CHECK(hbRowsDiffBefore == 0 && hbRowsBefore > 20000 && hbRowsAfter > 50000 && hbRowsDiffAfter > hbRowsAfter / 2,
        "the healthy offset speed-up x3: bit-identical until its first accepted frame (%d of %d rows differ), then "
        "the faster offset shows (%d of %d)", hbRowsDiffBefore, hbRowsBefore, hbRowsDiffAfter, hbRowsAfter);
}

/* ========================================================================= */
/* 2. synthetic flight                                                        */
/* ========================================================================= */

typedef struct sim_s sim_t;
/* Changes a frame before it is given to the observer (tc = capture time, s). */
typedef void (*frameMod_t)(sim_t *s, camVzObsFrame_t *f, double tc);

struct sim_s {
  /* scenario */
  double tl;          /* lift-off, s */
  double climbS;      /* smooth climb to h1 */
  double rampV, rampA;    /* rampV > 0: the climb to h1 at rampV m/s instead (raised-cosine speed ramps of rampA s at
                           * both ends, as vert_ctrl's capped climb), climbS then derived */
  double h1;
  double bobA, bobF;  /* vertical bob after the climb: z = h1 + bobA (1 - cos(2 pi bobF t)) */
  double bobFrom;     /* > 0: the bob only from this time (its phase from it), a still hover before */
  double biasG;       /* accelerometer offset (measured minus true), g */
  double biasRate;    /* g/s: the offset drifts at this rate from biasFrom on (the late offset noise's tests) */
  double biasFrom;
  double biasTo;      /* > biasFrom: the drift stops there (the healthy offset speed-up's early fall); 0: never */
  double gndFrom, gndTo;  /* the caller's ground flag set in [gndFrom, gndTo) after the lift-off (a toggle) */
  double gTrue;       /* px/m before tSwitch */
  double gTrue2;      /* px/m from tSwitch on */
  double tSwitch;
  double gHA, gHB, gH0;   /* gHB > 0: G(h) = gHA + gHB / (gH0 + z) instead (a scene whose distance grows with height) */
  double gRampFrom, gRampTo;   /* gRampTo > gRampFrom: a level move toward the scene, D linear in time from f / gTrue
                                * to f / gTrue2 over [gRampFrom, gRampTo] (testForwardGain) */
  double accNoiseG;   /* rms per sample */
  double pxNoise;     /* rms per frame */
  double latS;        /* capture -> available */
  double frameS;      /* frame interval */
  double camOffS, camOnS;   /* no frames captured in [camOffS, camOnS) */
  double thrustOnS;   /* thrust 0 before this */
  frameMod_t mod;
  /* run */
  camVzObs_t *o;
  uint64_t base;
  uint32_t imuUs;
  uint64_t k;         /* next IMU sample index */
  uint32_t rng;
  uint32_t seq;
  double nextTc, prevTc;
  int dec[10];
  camVzObsDecision_t last;
  /* frames waiting for their availability time (constant latency: FIFO) */
  camVzObsFrame_t q[16];
  double qAvail[16];
  int qHead, qN;
};

static double uni(sim_t *s)
{
  s->rng ^= s->rng << 13;
  s->rng ^= s->rng >> 17;
  s->rng ^= s->rng << 5;
  return (s->rng + 0.5) / 4294967296.0;
}

static double gauss(sim_t *s)
{
  return sqrt(-2.0 * log(uni(s))) * cos(2.0 * M_PI * uni(s));
}

/* The rampV climb: distance and speed at tc s after the lift-off (tc < climbS). */
static double rampZ(const sim_t *s, double tc, double *v)
{
  const double vc = s->rampV, ta = s->rampA, tCruise = s->climbS - 2.0 * ta;
  if (tc < ta) {
    *v = vc * 0.5 * (1.0 - cos(M_PI * tc / ta));
    return vc * 0.5 * (tc - ta / M_PI * sin(M_PI * tc / ta));
  }
  if (tc < ta + tCruise) {
    *v = vc;
    return vc * 0.5 * ta + vc * (tc - ta);
  }
  const double td = tc - ta - tCruise;   /* the mirrored ramp down */
  *v = vc * 0.5 * (1.0 + cos(M_PI * td / ta));
  return vc * 0.5 * ta + vc * tCruise + vc * 0.5 * (td + ta / M_PI * sin(M_PI * td / ta));
}

static double simZ(const sim_t *s, double t)
{
  if (t <= s->tl) {
    return 0.0;
  }
  if (s->rampV > 0.0 && t - s->tl < s->climbS) {
    double v;
    return rampZ(s, t - s->tl, &v);
  }
  const double u = (t - s->tl) / s->climbS;
  if (u < 1.0) {
    return s->h1 * u * u * (3.0 - 2.0 * u);
  }
  const double tb = s->bobFrom > 0.0 ? t - s->bobFrom : t - s->tl - s->climbS;
  if (tb < 0.0) {
    return s->h1;
  }
  return s->h1 + s->bobA * (1.0 - cos(2.0 * M_PI * s->bobF * tb));
}

static double simV(const sim_t *s, double t)
{
  if (t <= s->tl) {
    return 0.0;
  }
  if (s->rampV > 0.0 && t - s->tl < s->climbS) {
    double v;
    rampZ(s, t - s->tl, &v);
    return v;
  }
  const double u = (t - s->tl) / s->climbS;
  if (u < 1.0) {
    return s->h1 * 6.0 * u * (1.0 - u) / s->climbS;
  }
  const double tb = s->bobFrom > 0.0 ? t - s->bobFrom : t - s->tl - s->climbS;
  if (tb < 0.0) {
    return 0.0;
  }
  return s->bobA * 2.0 * M_PI * s->bobF * sin(2.0 * M_PI * s->bobF * tb);
}

static double simG(const sim_t *s, double t)
{
  if (s->gRampTo > s->gRampFrom && t >= s->gRampFrom) {
    const double u = fmin(1.0, (t - s->gRampFrom) / (s->gRampTo - s->gRampFrom));
    return 1.0 / (1.0 / s->gTrue + (1.0 / s->gTrue2 - 1.0 / s->gTrue) * u);
  }
  return t < s->tSwitch ? s->gTrue : s->gTrue2;
}

/* Image motion (px, + down) of the height change over [t0, t1]: -G times it,
 * or with a height-dependent G(h) = gHA + gHB / (gH0 + z) its integral. */
static double simDy(const sim_t *s, double t0, double t1)
{
  const double z0 = simZ(s, t0), z1 = simZ(s, t1);
  if (s->gHB > 0.0) {
    return -(s->gHA * (z1 - z0) + s->gHB * log((s->gH0 + z1) / (s->gH0 + z0)));
  }
  return -simG(s, t1) * (z1 - z0);
}

static void simDefaults(sim_t *s, camVzObs_t *o, uint32_t imuUs)
{
  memset(s, 0, sizeof(*s));
  s->tl = 0.6;
  s->climbS = 1.5;
  s->h1 = 0.40;
  s->bobA = 0.03;
  s->bobF = 0.7;
  s->biasG = 0.004;
  s->gTrue = 120.0;
  s->gTrue2 = 120.0;
  s->tSwitch = 1e9;
  s->accNoiseG = 0.02;
  s->pxNoise = 0.15;
  s->latS = 0.13;
  s->frameS = 0.059;
  s->camOffS = 1e9;
  s->camOnS = 1e9;
  s->thrustOnS = 0.0;
  s->o = o;
  s->base = 5000000000ull;
  s->imuUs = imuUs;
  s->rng = 12345;
  s->nextTc = 0.02;
  s->prevTc = -1.0;
}

/* The frame captured at tc (window [tc + 15 ms - dt, tc + 15 ms]). */
static void simFrame(sim_t *s, double tc, camVzObsFrame_t *f)
{
  const camVzObsConfig_t *c = &s->o->cfg;
  memset(f, 0, sizeof(*f));
  const double dt = s->prevTc < 0.0 ? 0.0 : tc - s->prevTc;
  f->seq = ++s->seq;
  f->frameTimeUs = s->base + (uint64_t)llround(tc * 1e6);
  f->dtUs = (uint32_t)llround(dt * 1e6);
  const double t1 = tc + c->gyroOffsetUs * 1e-6, t0 = t1 - f->dtUs * 1e-6;
  const double pitch = 0.004 * sin(2.0 * M_PI * 1.3 * t1) - 0.004 * sin(2.0 * M_PI * 1.3 * t0);
  f->gyroAngle[0] = 0.0005f;
  f->gyroAngle[1] = (float)pitch;
  f->gyroAngle[2] = 0.0002f;
  const double rotY = c->kyGx * f->gyroAngle[0] + c->kyGy * f->gyroAngle[1] + c->kyGz * f->gyroAngle[2] + c->kyC;
  const double rotX = c->kxGx * f->gyroAngle[0] + c->kxGy * f->gyroAngle[1] + c->kxGz * f->gyroAngle[2];
  f->dy = (float)(simDy(s, t0, t1) + rotY + s->pxNoise * gauss(s));
  f->dx = (float)(rotX + 0.1 * gauss(s));
  f->scale = 0.0f;
  f->texture = 20.0f;
  f->residual = 3.0f;
  f->ok = dt > 0.0;
  s->prevTc = tc;
}

static double simNow(const sim_t *s)
{
  return (double)(s->k - 1) * s->imuUs * 1e-6;
}

/* Runs the IMU (and the frames that become available) up to time tEnd (s). */
static void simRun(sim_t *s, double tEnd)
{
  for (;;) {
    const double t = (double)s->k * s->imuUs * 1e-6;
    if (t > tEnd + 1e-9) {
      return;
    }
    const double h = s->imuUs * 1e-6;
    const bool floor = t < s->tl;
    const bool ground = floor || (t >= s->gndFrom && t < s->gndTo);
    const double aMean = floor ? 0.0 : (simV(s, t + h) - simV(s, t)) / h;
    double bias = s->biasG;
    if (s->biasRate != 0.0 && t > s->biasFrom) {
      bias += s->biasRate * ((s->biasTo > s->biasFrom && t > s->biasTo ? s->biasTo : t) - s->biasFrom);
    }
    const double aUp = 1.0 + aMean / CAM_VZ_G + bias + s->accNoiseG * gauss(s);
    camVzObsImu(s->o, s->base + s->k * s->imuUs, (float)aUp, ground, t < s->thrustOnS ? 0.0f : 30000.0f);
    s->k++;
    /* capture */
    while (s->nextTc <= t) {
      const double tc = s->nextTc;
      s->nextTc += s->frameS;
      if (tc >= s->camOffS && tc < s->camOnS) {
        continue;
      }
      if (s->qN < 16) {
        const int i = (s->qHead + s->qN) % 16;
        simFrame(s, tc, &s->q[i]);
        if (s->mod != NULL) {
          s->mod(s, &s->q[i], tc);
        }
        s->qAvail[i] = tc + s->latS;
        s->qN++;
      }
    }
    /* deliver */
    while (s->qN > 0 && s->qAvail[s->qHead] <= t + 1e-9) {
      const camVzObsDecision_t d = camVzObsFrame(s->o, &s->q[s->qHead], s->base + (uint64_t)llround(t * 1e6));
      s->dec[d]++;
      s->last = d;
      s->qHead = (s->qHead + 1) % 16;
      s->qN--;
    }
  }
}

static void simStart(sim_t *s, camVzObs_t *o, const camVzObsConfig_t *cfg, uint32_t imuUs, double priorErrG)
{
  CHECK(camVzObsInit(o, cfg), "init");
  simDefaults(s, o, imuUs);
  CHECK(camVzObsSetBiasPrior(o, (float)(s->biasG + priorErrG), CAM_VZ_PRIOR_SIGMA_G), "prior");
}

/* Prior set after simStart changed the truth. */
static void simPrior(sim_t *s, double priorErrG)
{
  CHECK(camVzObsSetBiasPrior(s->o, (float)(s->biasG + priorErrG), CAM_VZ_PRIOR_SIGMA_G), "prior");
}

static camVzObs_t obs;
static sim_t sim;

static void testConvergence(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  simStart(&sim, &obs, &cfg, 10000, 0.003);   /* prior 3 mg off */
  simRun(&sim, 0.5);
  camVzObsOut_t out;
  camVzObsGet(&obs, &out);
  CHECK(!out.airborne && out.h == 0.0f && out.v == 0.0f && sim.dec[CAM_VZ_F_SKIPPED] > 0 &&
        sim.dec[CAM_VZ_F_ACCEPTED] == 0, "ground: h %.3f v %.3f skipped %d", out.h, out.v, sim.dec[CAM_VZ_F_SKIPPED]);
  simRun(&sim, 1.4);
  CHECK(sim.dec[CAM_VZ_F_ACCEPTED] == 0, "no update before the 1.5 s wake (%d)", sim.dec[CAM_VZ_F_ACCEPTED]);
  CHECK(!camVzObsSetBiasPrior(&obs, 0.0f, 0.002f), "prior refused after lift-off");
  simRun(&sim, 12.0);
  camVzObsGet(&obs, &out);
  const double t = simNow(&sim);
  const double eb = (out.biasG - sim.biasG) * 1e3, eh = out.h - simZ(&sim, t), ev = out.v - simV(&sim, t);
  printf("synthetic 100 Hz, 12 s: offset err %+.2f mg (sigma %.2f), h err %+.3f m, v err %+.3f m/s, G %.1f "
         "(true %.0f), accepted %d, q %d, r %d, c %d\n", eb, out.sigmaBiasG * 1e3, eh, ev, out.gain, sim.gTrue,
         sim.dec[CAM_VZ_F_ACCEPTED], sim.dec[CAM_VZ_F_QUALITY], sim.dec[CAM_VZ_F_GATE], sim.dec[CAM_VZ_F_CUSUM]);
  CHECK(fabs(eb) < 1.0, "offset converged: err %.2f mg", eb);
  CHECK(fabs(eh) < 0.05 && fabs(ev) < 0.03, "h err %.3f m, v err %.3f m/s", eh, ev);
  CHECK(out.gainTracked && fabs(out.gain / sim.gTrue - 1.0) < 0.1, "gain tracked %d: %.1f px/m", out.gainTracked,
        out.gain);
  CHECK(out.healthy && out.sinceAcceptUs < 200000 && !out.fault, "healthy %d since %u", out.healthy,
        (unsigned)out.sinceAcceptUs);
  CHECK(sim.dec[CAM_VZ_F_ACCEPTED] > 150 && sim.dec[CAM_VZ_F_GATE] + sim.dec[CAM_VZ_F_CUSUM] < 5,
        "accepted %d rejected %d", sim.dec[CAM_VZ_F_ACCEPTED], sim.dec[CAM_VZ_F_GATE] + sim.dec[CAM_VZ_F_CUSUM]);
  CHECK(out.sigmaBiasG < 0.0015f && out.sigmaV < 0.05f, "sigmas b %.2f mg v %.3f", out.sigmaBiasG * 1e3,
        out.sigmaV);
  CHECK(fabsf(obs.za) <= CAM_VZ_ZA_REBASE_M + 0.1f, "raw double integral re-based (%.2f m)", obs.za);
  float tg;
  CHECK(camVzObsFirstGainTime(&obs, &tg) && tg > 1.5f && tg < 8.0f, "first gain at %.2f s", tg);
}

/* Camera stops, then resumes. */
static void testCameraLoss(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  sim.camOffS = 8.0;
  sim.camOnS = 11.0;
  simRun(&sim, 8.1);
  camVzObsOut_t out;
  camVzObsGet(&obs, &out);
  CHECK(out.healthy, "healthy before the loss");
  simRun(&sim, 9.9);
  camVzObsGet(&obs, &out);
  CHECK(out.healthy && out.sinceAcceptUs >= 1500000, "still healthy 1.6+ s after the last frame (%u us)",
        (unsigned)out.sinceAcceptUs);
  simRun(&sim, 10.4);
  camVzObsGet(&obs, &out);
  CHECK(!out.healthy && out.sinceAcceptUs >= 2000000 && out.gainTracked, "unhealthy 2 s after the last update "
        "(since %u us)", (unsigned)out.sinceAcceptUs);
  const double eh = out.h - simZ(&sim, simNow(&sim));
  /* 0.02 g white noise at 100 Hz alone walks h by ~3 cm rms over 2.3 s */
  CHECK(fabs(eh) < 0.1, "accelerometer-only coast of 2.3 s: h err %.3f m", eh);
  const int acc0 = sim.dec[CAM_VZ_F_ACCEPTED];
  simRun(&sim, 12.0);
  camVzObsGet(&obs, &out);
  CHECK(out.healthy && sim.dec[CAM_VZ_F_ACCEPTED] > acc0 + 10, "healthy again after the camera returns (%d new)",
        sim.dec[CAM_VZ_F_ACCEPTED] - acc0);
}

/* Each gate on hand-made frames in the middle of a healthy flight. */
static camVzObsDecision_t gateFrame(sim_t *s, void (*mod)(camVzObsFrame_t *f), uint64_t ageUs)
{
  /* a frame captured 130 ms ago with the next seq, given now */
  const double tNow = simNow(s);
  camVzObsFrame_t f;
  simFrame(s, tNow - ageUs * 1e-6, &f);
  if (mod != NULL) {
    mod(&f);
  }
  return camVzObsFrame(s->o, &f, s->base + (uint64_t)llround(tNow * 1e6));
}

static void modNone(camVzObsFrame_t *f) { (void)f; }
static void modNotOk(camVzObsFrame_t *f) { f->ok = false; }
static void modTexture(camVzObsFrame_t *f) { f->texture = 7.9f; }
static void modResidual(camVzObsFrame_t *f) { f->residual = 8.0f; }
static void modYaw(camVzObsFrame_t *f) { f->gyroAngle[2] = 0.35f * f->dtUs * 1e-6f * 1.01f; }
static void modDtZero(camVzObsFrame_t *f) { f->dtUs = 0; }
static void modNanDy(camVzObsFrame_t *f) { f->dy = NAN; }
static void modLateral(camVzObsFrame_t *f) { f->dx += 1.6f; }
static void modOutlier(camVzObsFrame_t *f) { f->dy += 25.0f; }
static void modExpand(camVzObsFrame_t *f) { f->scale = 0.06f; }
/* A commanded turn's frame (user decision 2026-10-13, evening): the body turning at rate rad/s over the frame,
 * the image moving with it as the rotation model says (flowcal1.json). */
static void turnFrame(camVzObsFrame_t *f, float rate)
{
  const float a = rate * f->dtUs * 1e-6f;
  f->gyroAngle[2] += a;
  f->dx += 102.4f * a;
  f->dy += -2.7643f * a;
}
static void modTurn30(camVzObsFrame_t *f) { turnFrame(f, 30.0f * 0.0174533f); }
static void modTurn44(camVzObsFrame_t *f) { turnFrame(f, 44.0f * 0.0174533f); }
static void modTurn46(camVzObsFrame_t *f) { turnFrame(f, 46.0f * 0.0174533f); }

static void testGates(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  simRun(&sim, 6.0);
  /* drain the regular frames so that hand-made ones follow in seq order */
  sim.camOffS = 0.0;
  sim.camOnS = 1e9;
  simRun(&sim, 6.3);
  const uint32_t q0 = obs.cnt.quality;
  sim.prevTc = simNow(&sim) - 0.130 - 0.059;   /* a normal 59 ms interval before each hand-made frame */
  CHECK(gateFrame(&sim, modNone, 130000) == CAM_VZ_F_ACCEPTED, "clean frame accepted");
  struct { void (*mod)(camVzObsFrame_t *f); const char *name; camVzObsDecision_t want; } cases[] = {
    {modNotOk, "ok false", CAM_VZ_F_QUALITY},
    {modTexture, "texture 7.9", CAM_VZ_F_QUALITY},
    {modResidual, "residual 8.0", CAM_VZ_F_QUALITY},
    {modYaw, "yaw rate 0.354 rad/s", CAM_VZ_F_QUALITY},
    {modDtZero, "dt 0", CAM_VZ_F_QUALITY},
    {modNanDy, "dy NaN", CAM_VZ_F_QUALITY},
    {modLateral, "dx 1.6 px", CAM_VZ_F_LATERAL},
    {modOutlier, "dy +25 px", CAM_VZ_F_GATE},
  };
  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    sim.prevTc = simNow(&sim) - 0.130 - 0.059;
    const camVzObsDecision_t d = gateFrame(&sim, cases[i].mod, 130000);
    CHECK(d == cases[i].want, "gate %s: %s (want %s)", cases[i].name, camVzObsDecisionName(d),
          camVzObsDecisionName(cases[i].want));
  }
  CHECK(obs.cnt.quality == q0 + 6 && obs.cnt.latRejects == 1, "quality counter %u",
        (unsigned)(obs.cnt.quality - q0));
  /* the yaw gate in a commanded turn (camVzObsSetTurn): 45 deg/s instead of 20 */
  CHECK(CAM_VZ_TURN_MAX_YAW_RATE == AUTO_MV_TURN_GATE_RPS, "the observer's turn gate is the motion module's");
  {
    struct { void (*mod)(camVzObsFrame_t *f); bool turn; const char *name; camVzObsDecision_t want; } tc[] = {
      {modTurn30, false, "30 deg/s, no turn", CAM_VZ_F_QUALITY},
      {modTurn30, true, "30 deg/s in a turn", CAM_VZ_F_ACCEPTED},
      {modTurn44, true, "44 deg/s in a turn", CAM_VZ_F_ACCEPTED},
      {modTurn46, true, "46 deg/s in a turn", CAM_VZ_F_QUALITY},
      {modTurn30, false, "30 deg/s, the turn over", CAM_VZ_F_QUALITY},
      {modNone, false, "still, the turn over", CAM_VZ_F_ACCEPTED},
    };
    for (unsigned i = 0; i < sizeof(tc) / sizeof(tc[0]); i++) {
      camVzObsSetTurn(&obs, tc[i].turn);
      sim.prevTc = simNow(&sim) - 0.130 - 0.059;
      const camVzObsDecision_t d = gateFrame(&sim, tc[i].mod, 130000);
      CHECK(d == tc[i].want, "yaw gate, %s: %s (want %s)", tc[i].name, camVzObsDecisionName(d),
            camVzObsDecisionName(tc[i].want));
    }
    CHECK(obs.cnt.yawResets == 0, "four turn frames (0.17 rad): no tracker restart");
  }
  /* stale frame (age 300 ms) and a missed seq */
  sim.prevTc = simNow(&sim) - 0.300 - 0.059;
  CHECK(gateFrame(&sim, modNone, 300000) == CAM_VZ_F_QUALITY, "age 300 ms rejected");
  sim.prevTc = simNow(&sim) - 0.299 - 0.059;
  CHECK(gateFrame(&sim, modNone, 299000) == CAM_VZ_F_ACCEPTED, "age 299 ms used");
  sim.seq++;
  sim.prevTc = simNow(&sim) - 0.130 - 0.059;
  const uint32_t gaps = obs.cnt.seqGaps;
  CHECK(gateFrame(&sim, modNone, 130000) == CAM_VZ_F_QUALITY && obs.cnt.seqGaps == gaps + 1, "missed seq");
  sim.prevTc = simNow(&sim) - 0.130 - 0.059;
  CHECK(gateFrame(&sim, modNone, 130000) == CAM_VZ_F_ACCEPTED, "consecutive again");
  /* duplicate seq: ignored, no counter moves */
  {
    camVzObsFrame_t f;
    sim.prevTc = simNow(&sim) - 0.130 - 0.059;
    simFrame(&sim, simNow(&sim) - 0.130, &f);
    const uint64_t now = sim.base + (uint64_t)llround(simNow(&sim) * 1e6);
    CHECK(camVzObsFrame(&obs, &f, now) == CAM_VZ_F_ACCEPTED, "frame");
    const uint32_t fr = obs.cnt.frames, acc = obs.cnt.accepted;
    CHECK(camVzObsFrame(&obs, &f, now) == CAM_VZ_F_DUPLICATE && obs.cnt.frames == fr && obs.cnt.accepted == acc,
          "duplicate ignored");
  }
  /* window outside the ring: too old (1.2 s), and not yet integrated (future) */
  sim.prevTc = simNow(&sim) - 1.2 - 0.059;
  {
    camVzObsFrame_t f;
    simFrame(&sim, simNow(&sim) - 1.2, &f);
    CHECK(camVzObsFrame(&obs, &f, sim.base + (uint64_t)llround((simNow(&sim) - 1.1) * 1e6)) == CAM_VZ_F_NO_WINDOW,
          "window older than the ring");
    sim.prevTc = simNow(&sim) - 0.059;
    simFrame(&sim, simNow(&sim), &f);
    CHECK(camVzObsFrame(&obs, &f, sim.base + (uint64_t)llround(simNow(&sim) * 1e6)) == CAM_VZ_F_NO_WINDOW,
          "window ending after the latest block");
  }
  /* expansion rate: |EMA of scale / dt| > 0.3 1/s after a few forward frames */
  int nExp = 0;
  for (int i = 0; i < 6; i++) {
    sim.prevTc = simNow(&sim) - 0.130 - 0.059;
    nExp += gateFrame(&sim, modExpand, 130000) == CAM_VZ_F_EXPANSION;
  }
  CHECK(nExp >= 5 && obs.expEma > 0.3f, "expansion gate %d of 6 (ema %.2f)", nExp, obs.expEma);
  int back = 0;
  for (int i = 0; i < 20 && back == 0; i++) {
    sim.prevTc = simNow(&sim) - 0.130 - 0.059;
    back = gateFrame(&sim, modNone, 130000) == CAM_VZ_F_ACCEPTED ? i + 1 : 0;
  }
  CHECK(back >= 2 && back <= 10, "expansion EMA decays: accepted again after %d frames", back);
  /* gates off in the plain configuration */
  camVzObsPlainConfig(&cfg);
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  simRun(&sim, 6.0);
  sim.camOffS = 0.0;
  simRun(&sim, 6.3);
  sim.prevTc = simNow(&sim) - 0.130 - 0.059;
  gateFrame(&sim, modNone, 130000);
  sim.prevTc = simNow(&sim) - 0.130 - 0.059;
  const camVzObsDecision_t d = gateFrame(&sim, modLateral, 130000);
  CHECK(d == CAM_VZ_F_ACCEPTED || d == CAM_VZ_F_NO_GAIN, "plain: no lateral gate (%s)", camVzObsDecisionName(d));
}

/* Skips: wake clock from throttle-up, lift-off time. */
static void testSkips(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  sim.thrustOnS = 0.4;      /* wake = 0.4 + 1.5 s */
  simRun(&sim, 1.85);
  CHECK(sim.dec[CAM_VZ_F_ACCEPTED] == 0 && sim.dec[CAM_VZ_F_QUALITY] == 0, "nothing used before throttle-up + wake");
  simRun(&sim, 2.5);
  CHECK(sim.dec[CAM_VZ_F_ACCEPTED] > 0, "used after the wake");
  /* lift-off later than the wake: frames before lift-off skipped */
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  sim.tl = 2.5;
  simPrior(&sim, 0.0);
  simRun(&sim, 2.55);
  CHECK(sim.dec[CAM_VZ_F_ACCEPTED] + sim.dec[CAM_VZ_F_QUALITY] == 0, "nothing before lift-off");
  CHECK(obs.tLift >= 2500000 && obs.tLift <= 2510000, "lift-off time %u", (unsigned)obs.tLift);
  CHECK(camVzObsSetLiftoffTime(&obs, sim.base + 2400000) && obs.tLift == 2400000, "back-dated lift-off");
  CHECK(camVzObsSetLiftoffTime(&obs, sim.base + 99000000) && obs.tLift == obs.tIn, "lift-off clamped to now");
  simRun(&sim, 4.0);
  CHECK(sim.dec[CAM_VZ_F_ACCEPTED] > 0, "used after lift-off");
  /* touchdown: on the ground again, frames skipped, h = v = 0 */
  sim.tl = 1e9;   /* simZ = 0, ground flag on */
  const int sk = sim.dec[CAM_VZ_F_SKIPPED];
  simRun(&sim, 4.5);
  camVzObsOut_t out;
  camVzObsGet(&obs, &out);
  CHECK(!out.airborne && out.h == 0.0f && out.v == 0.0f && sim.dec[CAM_VZ_F_SKIPPED] > sk + 3,
        "touchdown: skipped %d", sim.dec[CAM_VZ_F_SKIPPED] - sk);
}

/* Yaw to a 2x nearer view: the tracker restarts, keeps its G as a guess and learns the new one. */
static void testGainRelearn(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  sim.bobA = 0.04;
  simRun(&sim, 8.0);
  camVzObsOut_t out;
  camVzObsGet(&obs, &out);
  CHECK(out.gainTracked && fabs(out.gain / 120.0 - 1.0) < 0.1, "G before the turn %.1f", out.gain);
  const float gOld = out.gain;
  /* 90 deg turn over 1 s: every frame fails the yaw-rate gate and adds yaw */
  sim.tSwitch = 9.0;
  sim.gTrue2 = 240.0;
  const int q0 = sim.dec[CAM_VZ_F_QUALITY];
  for (double t = 8.0; t < 9.0; t += 0.01) {
    simRun(&sim, t);
    for (int i = 0; i < sim.qN; i++) {
      camVzObsFrame_t *f = &sim.q[(sim.qHead + i) % 16];
      if (f->gyroAngle[2] < 0.05f) {
        f->gyroAngle[2] = (float)(M_PI / 2.0 * sim.frameS);
      }
    }
  }
  simRun(&sim, 9.2);
  camVzObsGet(&obs, &out);
  /* 90 deg = 1.57 rad of accumulated yaw: a restart per 0.35 rad */
  CHECK(obs.cnt.yawResets == 4 && sim.dec[CAM_VZ_F_QUALITY] > q0 + 10, "yaw resets %u, turn frames gated %d",
        (unsigned)obs.cnt.yawResets, sim.dec[CAM_VZ_F_QUALITY] - q0);
  CHECK(!out.gainTracked && !out.healthy && out.gain == gOld, "after the turn: old G %.1f kept as a guess (%.1f)",
        gOld, out.gain);
  /* interactive hover: through a commanded turn the glue may let the height estimate use that guess */
  CHECK(out.healthyGuess && out.sinceAcceptUs < cfg.healthyWindowUs, "after the turn: healthyGuess (an update "
        "%u us ago, the guessed G)", (unsigned)out.sinceAcceptUs);
  simRun(&sim, 18.0);
  camVzObsGet(&obs, &out);
  const double eb = (out.biasG - sim.biasG) * 1e3;
  printf("gain re-learn: G %.1f -> %.1f px/m (true 240), offset err %+.2f mg, h err %+.3f m\n", gOld, out.gain, eb,
         out.h - simZ(&sim, simNow(&sim)));
  CHECK(out.gainTracked && fabs(out.gain / 240.0 - 1.0) < 0.12, "re-learned G %.1f (want 240)", out.gain);
  CHECK(out.healthy && out.healthyGuess && fabs(eb) < 1.5, "healthy %d, offset err %.2f mg", out.healthy, eb);
}

/* The interactive hover's re-track bob (auto_motion.h): a commanded turn
 * restarts the tracker; a still hover gives it no vertical excitation (it
 * stays without a G), the bob (+/-2.5 cm around +2.5 cm, 0.5 Hz, about the
 * firmware's 5 cm triangle legs of 1 s) gives it one within a few cycles. */
static void testRetrackBob(void)
{
  for (int k = 0; k < 2; k++) {
    camVzObsConfig_t cfg;
    camVzObsDefaultConfig(&cfg);
    simStart(&sim, &obs, &cfg, 10000, 0.0);
    sim.bobA = 0.025;
    sim.bobF = 0.5;
    sim.bobFrom = 1e9;   /* still until the turn */
    sim.tSwitch = 9.0;
    sim.gTrue2 = 70.0;   /* the new view: farther */
    simRun(&sim, 8.0);
    camVzObsOut_t out;
    camVzObsGet(&obs, &out);
    const bool trackedBefore = out.gainTracked;   /* the climb gave it a G */
    /* a 45 deg turn at 15 deg/s (3 s): frames pass the yaw-rate gate */
    for (double t = 8.0; t < 11.0; t += 0.01) {
      simRun(&sim, t);
      for (int i = 0; i < sim.qN; i++) {
        camVzObsFrame_t *f = &sim.q[(sim.qHead + i) % 16];
        if (fabsf(f->gyroAngle[2]) < 0.005f) {
          f->gyroAngle[2] = (float)(0.2618 * sim.frameS);
        }
      }
    }
    simRun(&sim, 11.5);
    camVzObsGet(&obs, &out);
    const bool lost = !out.gainTracked && out.healthyGuess;
    if (k == 1) {
      sim.bobFrom = 11.5;   /* the bob from the turn's dwell on */
    }
    double tBack = -1.0;
    for (double t = 11.5; t < 21.5 && tBack < 0.0; t += 0.05) {
      simRun(&sim, t);
      camVzObsGet(&obs, &out);
      if (out.gainTracked) {
        tBack = t - 11.5;
      }
    }
    printf("re-track after a turn, %s: G back after %s%.2f s (G %.1f, true 70)\n", k ? "bob 5 cm 0.5 Hz" : "still hover",
           tBack < 0.0 ? "never in 10 s " : "", tBack < 0.0 ? 10.0 : tBack, out.gain);
    CHECK(trackedBefore && lost, "%s: tracked before, lost (guess only) after the turn", k ? "bob" : "still");
    if (k == 0) {
      CHECK(tBack < 0.0, "a still hover: no G again within 10 s (the reason for the bob)");
    } else {
      CHECK(tBack >= 0.0 && tBack <= 8.0 && fabs(out.gain / 70.0 - 1.0) < 0.25,
            "the bob: a G again within 8 s (4 cycles; %.2f s), within 25 %% of the new view's (%.1f)", tBack, out.gain);
    }
  }
}

/* The interactive hover's re-view bob (auto_motion.h, after the verification
 * of 591f176): a long sideways move changes the depth of the view without a
 * turn, so the tracker is not restarted and keeps the old view's G through a
 * still hover; the move's re-view bob (2 whole cycles of +4 cm, legs of
 * 0.89 s: ~0.56 Hz) makes it follow the new depth. */
static void testViewShiftBob(void)
{
  for (int k = 0; k < 2; k++) {
    camVzObsConfig_t cfg;
    camVzObsDefaultConfig(&cfg);
    simStart(&sim, &obs, &cfg, 10000, 0.0);
    sim.bobA = 0.02;          /* 0..+4 cm */
    sim.bobF = 1.0 / 1.79;
    sim.bobFrom = 1e9;
    sim.tSwitch = 9.0;        /* the view 1.8 x farther (a near box face left the view) */
    sim.gTrue2 = 66.0;
    simRun(&sim, 8.0);
    camVzObsOut_t out;
    camVzObsGet(&obs, &out);
    const double g0 = out.gain;
    if (k == 1) {
      sim.bobFrom = 11.5;
    }
    simRun(&sim, 11.5 + 2.0 * 1.79);
    camVzObsGet(&obs, &out);
    printf("view shift without a turn, %s: G %.1f -> %.1f px/m after %.2f s (true %.0f -> 66), tracked %d, healthy %d\n",
           k ? "the re-view bob (2 cycles)" : "still hover", g0, out.gain, 2.0 * 1.79, sim.gTrue, out.gainTracked,
           out.healthy);
    if (k == 0) {
      CHECK(out.gainTracked && fabs(out.gain / 66.0 - 1.0) > 0.4, "a still hover keeps the old view's G (%.1f)",
            out.gain);
    } else {
      CHECK(out.gainTracked && out.healthy && fabs(out.gain / 66.0 - 1.0) < 0.15 && obs.cnt.yawResets == 0,
            "2 bob cycles: G %.1f within 15 %% of the new view's 66, never restarted, healthy throughout", out.gain);
    }
  }
}

/* A level move toward the scene (2026-10-15 review; cam_vz_obs.h "Gain through a forward move"): the true G grows
 * from 24 px/m (D 4.16 m) by r over a 17 s move (D linear in time) with no vertical excitation the tracker takes
 * (a 0.8 cm bob at 0.3 Hz, hp rms ~0.01 m/s against its 0.03), the frames carrying the measured forward progress
 * k ln(D0 / D) (fwdK k; 0: none, b6a54da). The speed estimate's slope against the true speed over the 10 s after the
 * move: b6a54da reads it ~r x too fast at low frequency; with the progress at k 1 it is right; at k 0.6 (the
 * expansion under-reads the approach) r^0.4 is left. Then a yaw restart keeps the scaled G as its guess, and a bob
 * re-takes G at the distance now with the forward scale back to 1 (no double correction). */
static double fwdK;
static void modFwd(sim_t *s, camVzObsFrame_t *f, double tc)
{
  f->fwdLn = fwdK > 0.0 ? (float)(fwdK * log(simG(s, tc) / s->gTrue)) : 0.0f;
}

typedef struct {
  double slope, rms, g0, gAfter, gGuess;
  bool trackedThrough;
  unsigned takesInMove;
} fwdRun_t;

static fwdRun_t fwdScenario(double gStart, double r, double k, bool yawAfter, uint32_t seed)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  sim.rng = seed;
  sim.h1 = 1.0;
  sim.climbS = 3.0;
  sim.gTrue = gStart;
  sim.gTrue2 = gStart * r;
  sim.bobA = 0.02;           /* 0..4 cm at 0.1 Hz: the camera's band, under the tracker's excitation need */
  sim.bobF = 0.1;
  sim.gRampFrom = 8.0;
  sim.gRampTo = 25.0;
  sim.mod = modFwd;
  fwdK = k;
  fwdRun_t out = { 0 };
  simRun(&sim, sim.gRampFrom);
  camVzObsOut_t o;
  camVzObsGet(&obs, &o);
  const float g0 = o.gain;
  out.g0 = g0;
  bool tracked = o.gainTracked;
  double sxy = 0.0, sxx = 0.0, see = 0.0;
  int n = 0;
  float gPrev = g0;
  for (double t = sim.gRampFrom; t <= sim.gRampTo + 10.0 + 1e-9; t += 0.01) {
    simRun(&sim, t);
    camVzObsGet(&obs, &o);
    tracked = tracked && o.gainTracked;
    if (t < sim.gRampTo && o.gain != gPrev) {
      out.takesInMove++;
    }
    gPrev = o.gain;
    if (t >= sim.gRampTo) {
      const double vt = simV(&sim, simNow(&sim)), ve = o.v;
      sxy += vt * ve;
      sxx += vt * vt;
      see += (ve - vt) * (ve - vt);
      n++;
    }
  }
  out.slope = sxy / sxx;
  out.rms = sqrt(see / n);
  out.trackedThrough = tracked;
  out.gAfter = o.gain;
  if (yawAfter) {
    /* a 45 deg turn at 15 deg/s (testRetrackBob's): the tracker restarts, the scaled G is the guess */
    const double t0 = simNow(&sim);
    for (double t = t0; t < t0 + 3.0; t += 0.01) {
      simRun(&sim, t);
      for (int i = 0; i < sim.qN; i++) {
        camVzObsFrame_t *f = &sim.q[(sim.qHead + i) % 16];
        if (fabsf(f->gyroAngle[2]) < 0.005f) {
          f->gyroAngle[2] = (float)(0.2618 * sim.frameS);
        }
      }
    }
    out.gGuess = obs.hasGuess ? obs.gGuess : 0.0;
  }
  return out;
}

static void testForwardGain(void)
{
  const double r = 3.4;   /* D 4.66 -> 1.37 m: the Mac's largest move there at k 1 */
  const fwdRun_t base = fwdScenario(24.0, r, 0.0, false, 7u), fix = fwdScenario(24.0, r, 1.0, false, 7u),
                 k06 = fwdScenario(24.0, r, 0.6, false, 7u), still = fwdScenario(24.0 * r, 1.0, 1.0, false, 7u);
  printf("forward move G 24 -> %.1f px/m (D x 1 / %.1f), no vertical excitation: the speed estimate's slope against "
         "the true speed over the 10 s after it %.2f without the progress (b6a54da), %.2f with it at k 1, %.2f with it "
         "at k 0.6 (r^0.4 = %.2f), %.2f hovering at the end distance from the take-off on; rms error %.4f / %.4f / "
         "%.4f / %.4f m/s; the tracker's G %.1f "
         "(taken in the move %u times; at D0 %.1f)\n", 24.0 * r, r, base.slope, fix.slope, k06.slope, pow(r, 0.4),
         still.slope, base.rms, fix.rms, k06.rms, still.rms, fix.gAfter, fix.takesInMove, fix.g0);
  CHECK(base.trackedThrough && fix.trackedThrough && fix.takesInMove == 0 && fix.gAfter == fix.g0,
        "the level move: the tracker keeps the G it took at D0 (%.1f px/m, %u takes in the move)", fix.gAfter,
        fix.takesInMove);
  CHECK(base.slope > 1.5, "without the progress the speed reads too fast after a x%.1f approach (slope %.2f): the "
        "review's finding", r, base.slope);
  CHECK(fabs(fix.slope - still.slope) < 0.2 && fix.rms < 1.3 * still.rms + 0.002 && base.slope > fix.slope + 0.8,
        "with the progress (k 1) the speed as hovering at the end distance from the take-off on: slope %.2f (%.2f; "
        "without it %.2f), rms %.4f m/s (%.4f)", fix.slope, still.slope, base.slope, fix.rms, still.rms);
  CHECK(k06.slope > fix.slope && k06.slope < base.slope, "at k 0.6 in between (slope %.2f; %.2f .. %.2f)", k06.slope,
        fix.slope, base.slope);
  /* the guess after a turn: the scaled G (the scene distance now), not the one taken at D0 */
  const fwdRun_t yaw = fwdScenario(24.0, r, 1.0, true, 7u);
  printf("forward move then a 45 deg turn: the guess %.1f px/m (true now %.1f; the G taken at D0 %.1f)\n",
         yaw.gGuess, 24.0 * r, yaw.gAfter);
  CHECK(fabs(yaw.gGuess / (yaw.g0 * r) - 1.0) < 0.01, "the yaw restart keeps the scaled G as its guess (%.1f, the G "
        "taken at D0 x %.1f: %.1f)", yaw.gGuess, r, yaw.g0 * r);
  /* a bob after the move: the tracker takes G at the distance now, the forward scale back to 1 */
  fwdScenario(24.0, r, 1.0, false, 7u);
  const double tb = simNow(&sim);
  sim.bobA = 0.02;
  sim.bobF = 0.5;
  sim.bobFrom = tb;
  simRun(&sim, tb + 8.0);
  camVzObsOut_t o;
  camVzObsGet(&obs, &o);
  const float lnNow = (float)log(simG(&sim, simNow(&sim)) / sim.gTrue);
  double sxy = 0.0, sxx = 0.0;
  for (double t = tb + 8.0; t <= tb + 16.0 + 1e-9; t += 0.01) {
    simRun(&sim, t);
    camVzObsOut_t q;
    camVzObsGet(&obs, &q);
    const double vt = simV(&sim, simNow(&sim));
    sxy += vt * q.v;
    sxx += vt * vt;
  }
  printf("forward move then a 2 cm bob: the tracker's G %.1f px/m (true %.1f), its forward anchor %.4f (the progress "
         "%.4f), the speed's slope %.2f\n", o.gain, 24.0 * r, obs.fwdLnAtG, lnNow, sxy / sxx);
  CHECK(o.gainTracked && fabs(o.gain / (24.0 * r) - 1.0) < 0.15 && fabsf(obs.fwdLnAtG - lnNow) < 0.01f &&
        fabs(sxy / sxx - 1.0) < 0.15, "the bob re-takes G at the distance now (%.1f) and the forward scale starts "
        "again there (anchor %.4f, progress %.4f): no double correction (slope %.2f)", o.gain, obs.fwdLnAtG, lnNow,
        sxy / sxx);
  sim.mod = NULL;
}

/* A moving object drags the fit: a dy offset of dragPx px/frame. */
static double dragFrom, dragTo, dragPx;
static void modDrag(sim_t *s, camVzObsFrame_t *f, double tc)
{
  (void)s;
  if (tc >= dragFrom && tc < dragTo) {
    f->dy += (float)dragPx;
  }
}

static void testCusum(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  /* 1 px/frame (0.14 m/s at G 120, 59 ms frames): alarm within a few frames */
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  dragFrom = 6.0;
  dragTo = 8.0;
  dragPx = 1.0;
  sim.mod = modDrag;
  simRun(&sim, 7.0);
  camVzObsOut_t out;
  camVzObsGet(&obs, &out);
  CHECK(obs.cnt.alarms == 1 && obs.alarm && !out.healthy, "CUSUM alarm during the drag (%u), not healthy",
        (unsigned)obs.cnt.alarms);
  const uint32_t at = obs.alarmT;
  const int acc0 = sim.dec[CAM_VZ_F_ACCEPTED];
  simRun(&sim, 9.0);
  CHECK(sim.dec[CAM_VZ_F_CUSUM] >= 30 && obs.alarm && sim.dec[CAM_VZ_F_ACCEPTED] == acc0,
        "updates refused for the hold (%d)", sim.dec[CAM_VZ_F_CUSUM]);
  simRun(&sim, 10.5);
  camVzObsGet(&obs, &out);
  double eb = (out.biasG - sim.biasG) * 1e3, eh = out.h - simZ(&sim, simNow(&sim));
  printf("CUSUM, 1 px/frame drag for 2 s: alarm %.2f s after its start; after the 3 s hold h err %+.3f m (speed "
         "pulled in before the alarm, integrated through the hold), offset err %+.2f mg\n",
         at * 1e-6 - dragFrom, eh, eb);
  CHECK(!obs.alarm && obs.tIn - at >= 3000000 && out.sinceAcceptUs < 200000 && out.healthy,
        "alarm cleared after 3 s, updates again (since %u us)", (unsigned)out.sinceAcceptUs);
  CHECK(fabs(eb) < 1.5 && fabs(eh) < 0.4, "offset err %.2f mg, h err %.3f m", eb, eh);
  /* 0.5 px/frame (the study's "person, DC 0.5 px/frame"): below the alarm, the filter partly follows it */
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  dragPx = 0.5;
  sim.mod = modDrag;
  simRun(&sim, 10.5);
  camVzObsGet(&obs, &out);
  eb = (out.biasG - sim.biasG) * 1e3;
  eh = out.h - simZ(&sim, simNow(&sim));
  printf("CUSUM, 0.5 px/frame drag for 2 s: %u alarms, h err %+.3f m, offset err %+.2f mg\n",
         (unsigned)obs.cnt.alarms, eh, eb);
  CHECK(fabs(eb) < 1.5 && fabs(eh) < 0.15 && out.healthy, "sub-alarm drag: offset err %.2f mg, h err %.3f m", eb,
        eh);
}

/* CUSUM off: a lasting large offset is gated, and re-locked after relockS.
 * The drag also moves the tracked G (115 -> 154 px/m); the gain re-anchor
 * (default 0.3) stays shut: its window closed 2 s after the climb, and a
 * drag frame is outside the gate (testReanchorGuards). */
static void testRelock(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  cfg.cusumK = -1.0f;
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  dragFrom = 6.0;
  dragTo = 9.0;
  dragPx = 4.0;     /* 0.56 m/s at G 120 */
  sim.mod = modDrag;
  simRun(&sim, 7.4);
  CHECK(sim.dec[CAM_VZ_F_GATE] >= 15 && obs.cnt.relocks == 0, "offset gated (%d), no re-lock yet",
        sim.dec[CAM_VZ_F_GATE]);
  simRun(&sim, 9.0);
  camVzObsOut_t out;
  camVzObsGet(&obs, &out);
  CHECK(obs.cnt.relocks >= 1, "re-lock after 1.5 s of continuous rejection (%u)", (unsigned)obs.cnt.relocks);
  simRun(&sim, 12.0);
  camVzObsGet(&obs, &out);
  CHECK(out.healthy && obs.cnt.rejected < (uint32_t)sim.dec[CAM_VZ_F_GATE] + 1, "healthy after the offset ends");
  CHECK(obs.cnt.reanchors == 0, "no gain re-anchor in the drag (%u)", (unsigned)obs.cnt.reanchors);
}

/* Gain re-anchor (gainReanchorRel, 2026-10-13 flight 2). A 1.05 m climb at
 * 0.35 m/s over a scene whose effective distance grows with height (floor
 * and table parallax under a far window): G(h) = 6 + 15 / h px/m, h = 0.12 m
 * + the climb (131 -> 19 px/m; flight 2's tracker read 92 -> 67 -> 34 -> 27).
 * The tracker's G lags the fall, the camera under-reads the climb and the
 * filter puts the difference into the offset (+1.4 -> ~+8 mg); when G
 * catches up at the top the corrected camera speed disagrees with the
 * offset-inflated prediction and the CUSUM alarms (healthy goes false for
 * 3 s: HOVER_NO_CAM in the FSM). One run of the scenario. */
typedef struct {
  unsigned alarms, reanchors;
  bool healthy2;     /* healthy at the hover + 2 s */
  double vErr2;      /* v error then, m/s */
  double vErrMax;    /* max |v error| in the 6 s hover */
  double lateS;      /* latest re-anchor after the climb end the observer saw (s; -1: none after it) */
  bool climbEnd;     /* the observer saw the climb end */
} reanchorRun_t;

/* The scenario on s / o: heightGain false keeps the sim's constant 120 px/m. */
static void reanchorSetup(sim_t *s, camVzObs_t *o, float rel, uint32_t seed, bool heightGain)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  cfg.gainReanchorRel = rel;
  simStart(s, o, &cfg, 10000, 0.0);
  s->rng = seed;
  s->biasG = 0.0014;                              /* the logged flights' mean in-flight offset */
  simPrior(s, CAM_VZ_PRIOR_OFFSET_G - 0.0014);    /* the standard prior: +1.37 mg, sigma 2.2 mg */
  s->tl = 1.6;                                    /* lift-off after the camera wake: frames from the start */
  s->h1 = 1.05;
  s->rampV = 0.35;
  s->rampA = 0.6;
  s->climbS = s->h1 / s->rampV + s->rampA;
  s->bobA = 0.02;
  s->bobF = 0.5;
  if (heightGain) {
    s->gHA = 6.0;
    s->gHB = 15.0;
    s->gH0 = 0.12;
  }
}

#define REANCHOR_HOVER_S 6.0

static reanchorRun_t reanchorScenario(float rel, uint32_t seed, bool heightGain)
{
  reanchorSetup(&sim, &obs, rel, seed, heightGain);
  const double tH = sim.tl + sim.climbS;
  reanchorRun_t r = { 0 };
  r.lateS = -1.0;
  bool at2 = false;
  unsigned seen = 0;
  for (double t = 0.5; t <= tH + REANCHOR_HOVER_S + 1e-9; t += 0.05) {
    simRun(&sim, t);
    if (obs.cnt.reanchors != seen) {
      seen = obs.cnt.reanchors;
      if (obs.raClimbEnd) {
        r.lateS = fmax(r.lateS, ((double)obs.tState - (double)obs.raClimbEnd50ms * CAM_VZ_REANCHOR_TICK_US) * 1e-6);
      }
    }
    camVzObsOut_t out;
    camVzObsGet(&obs, &out);
    const double ev = out.v - simV(&sim, simNow(&sim));
    if (t >= tH && fabs(ev) > r.vErrMax) {
      r.vErrMax = fabs(ev);
    }
    if (!at2 && t >= tH + 2.0) {
      at2 = true;
      r.healthy2 = out.healthy;
      r.vErr2 = ev;
    }
  }
  r.alarms = obs.cnt.alarms;
  r.reanchors = obs.cnt.reanchors;
  r.climbEnd = obs.raClimbEnd;
  return r;
}

static void testGainReanchor(void)
{
  /* 30 noise seeds: the alarm is a property of the scene, not of one draw */
  int baseAlarmSeeds = 0, fixBad = 0, climbEnds = 0;
  unsigned raMin = 99, raMax = 0;
  double v2Max = 0.0, vHoverMax = 0.0, lateMax = -1.0;
  reanchorRun_t base1 = { 0 }, fix1 = { 0 };
  for (uint32_t seed = 1; seed <= 30; seed++) {
    const reanchorRun_t b = reanchorScenario(0.0f, seed, true);
    const reanchorRun_t f = reanchorScenario(0.3f, seed, true);
    if (seed == 1) {
      base1 = b;
      fix1 = f;
    }
    baseAlarmSeeds += b.alarms > 0;
    const bool ok = f.alarms == 0 && f.healthy2 && fabs(f.vErr2) < 0.10 && f.reanchors >= 1 && f.reanchors <= 4;
    if (!ok) {
      fixBad++;
      printf("  re-anchor seed %u: alarms %u healthy at hover + 2 s %d v err %+.3f m/s re-anchors %u\n", (unsigned)seed,
             f.alarms, f.healthy2, f.vErr2, f.reanchors);
    }
    raMin = f.reanchors < raMin ? f.reanchors : raMin;
    raMax = f.reanchors > raMax ? f.reanchors : raMax;
    v2Max = fmax(v2Max, fabs(f.vErr2));
    vHoverMax = fmax(vHoverMax, f.vErrMax);
    lateMax = fmax(lateMax, f.lateS);
    climbEnds += f.climbEnd;
  }
  printf("gain re-anchor, climb 0.12 -> 1.17 m at 0.35 m/s, G = 6 + 15/h px/m, 30 seeds: off: CUSUM alarm in %d "
         "(seed 1: %u, healthy at hover + 2 s %d, v err %+.3f m/s); 0.3: %d seeds failing, re-anchors %u..%u, max "
         "|v err| at hover + 2 s %.3f m/s (whole %.0f s hover %.3f; seed 1: %u re-anchors, v err %+.3f)\n",
         baseAlarmSeeds, base1.alarms, base1.healthy2, base1.vErr2, fixBad, raMin, raMax, v2Max, REANCHOR_HOVER_S,
         vHoverMax, fix1.reanchors, fix1.vErr2);
  CHECK(base1.alarms >= 1 && !base1.healthy2, "re-anchor off, seed 1: the scenario alarms (%u) and is unhealthy at "
        "hover + 2 s (%d): the test exercises the mechanism", base1.alarms, base1.healthy2);
  CHECK(baseAlarmSeeds >= 15, "re-anchor off: alarms in %d of 30 seeds", baseAlarmSeeds);
  CHECK(fixBad == 0, "re-anchor 0.3: every seed without an alarm, healthy at hover + 2 s, |v err| < 0.10 m/s there "
        "and 1-4 re-anchors (%d failed)", fixBad);
  printf("gain re-anchor window: climb end seen in %d of 30 seeds, latest re-anchor %.2f s after it (window %.1f s)\n",
         climbEnds, lateMax, (double)CAM_VZ_REANCHOR_AFTER_S);
  CHECK(climbEnds == 30 && lateMax <= CAM_VZ_REANCHOR_AFTER_S + 0.06, "re-anchors only in the climb window: climb "
        "end seen in %d of 30, latest re-anchor %.2f s after it", climbEnds, lateMax);

  /* A constant G (the clean take-offs), same climb: never a re-anchor, and
   * every output, state, covariance and decision bit-identical to off. */
  {
    static camVzObs_t o2;
    static sim_t s2;
    int noRa = 0, same = 0;
    for (uint32_t seed = 1; seed <= 10; seed++) {
      reanchorSetup(&sim, &obs, 0.3f, seed, false);
      reanchorSetup(&s2, &o2, 0.0f, seed, false);
      bool eq = true;
      for (double t = 0.5; t <= sim.tl + sim.climbS + REANCHOR_HOVER_S + 1e-9; t += 0.05) {
        simRun(&sim, t);
        simRun(&s2, t);
        camVzObsOut_t a, b;
        camVzObsGet(&obs, &a);
        camVzObsGet(&o2, &b);
        eq = eq && memcmp(&a, &b, sizeof(a)) == 0 && memcmp(obs.x, o2.x, sizeof(obs.x)) == 0 &&
             memcmp(obs.P, o2.P, sizeof(obs.P)) == 0 && memcmp(sim.dec, s2.dec, sizeof(sim.dec)) == 0;
      }
      noRa += obs.cnt.reanchors == 0 && obs.cnt.accepted > 100;
      same += eq;
    }
    CHECK(noRa == 10 && same == 10, "constant G 120 px/m, 10 seeds: no re-anchor in %d, outputs bit-identical to "
          "gainReanchorRel 0 in %d", noRa, same);
  }

  /* configuration range */
  camVzObsConfig_t c;
  camVzObsDefaultConfig(&c);
  CHECK(c.gainReanchorRel == 0.3f, "default 0.3");
  camVzObsPlainConfig(&c);
  CHECK(c.gainReanchorRel == 0.0f, "plain: off");
  const float bad[] = { -0.1f, 1.5f, NAN, INFINITY };
  for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
    camVzObsDefaultConfig(&c);
    c.gainReanchorRel = bad[i];
    CHECK(!camVzObsConfigValid(&c), "gainReanchorRel %g refused", (double)bad[i]);
  }
  const float good[] = { 0.0f, 1.0f };
  for (unsigned i = 0; i < sizeof(good) / sizeof(good[0]); i++) {
    camVzObsDefaultConfig(&c);
    c.gainReanchorRel = good[i];
    CHECK(camVzObsConfigValid(&c), "gainReanchorRel %g accepted", (double)good[i]);
  }
}

/* The gain re-anchor's guards (2026-10-13 review): the window and the gate
 * keep it from re-inflating sigma_v on a G that a dragging object moved in
 * the hover (the default config: CUSUM on). */
typedef struct {
  double vErrMax;    /* max |v error| while healthy, drag start .. drag end + 4 s */
  unsigned alarms, reanchors;
} dragRun_t;

static dragRun_t dragRun(float rel, double gTrue, double px, double fromHover, double dur, uint32_t seed)
{
  reanchorSetup(&sim, &obs, rel, seed, false);
  sim.gTrue = sim.gTrue2 = gTrue;
  const double from = sim.tl + sim.climbS + fromHover;
  dragFrom = from;
  dragTo = from + dur;
  dragPx = px;
  sim.mod = modDrag;
  dragRun_t r = { 0 };
  simRun(&sim, from);
  const unsigned ra0 = obs.cnt.reanchors;
  for (double t = from; t <= dragTo + 4.0 + 1e-9; t += 0.01) {
    simRun(&sim, t);
    camVzObsOut_t out;
    camVzObsGet(&obs, &out);
    if (out.healthy) {
      r.vErrMax = fmax(r.vErrMax, fabs(out.v - simV(&sim, simNow(&sim))));
    }
  }
  r.alarms = obs.cnt.alarms;
  r.reanchors = obs.cnt.reanchors - ra0;
  return r;
}

static void testReanchorGuards(void)
{
  /* 1. testRelock's 4 px/frame drag (0.56 m/s at G 120) with the CUSUM on:
   * it alarms as with the re-anchor off (the drag moves the tracked G
   * 115 -> 154 px/m 4 s after the climb ended), every output identical. */
  {
    static camVzObs_t o2;
    static sim_t s2;
    camVzObsConfig_t cfg;
    camVzObsDefaultConfig(&cfg);
    simStart(&sim, &obs, &cfg, 10000, 0.0);
    cfg.gainReanchorRel = 0.0f;
    simStart(&s2, &o2, &cfg, 10000, 0.0);
    dragFrom = 6.0;
    dragTo = 9.0;
    dragPx = 4.0;
    sim.mod = modDrag;
    s2.mod = modDrag;
    bool same = true, unhealthyInDrag = false;
    for (double t = 0.5; t <= 14.0 + 1e-9; t += 0.01) {
      simRun(&sim, t);
      simRun(&s2, t);
      camVzObsOut_t a, b;
      camVzObsGet(&obs, &a);
      camVzObsGet(&o2, &b);
      same = same && memcmp(&a, &b, sizeof(a)) == 0;
      unhealthyInDrag = unhealthyInDrag || (t > 6.0 && t < 9.0 && !a.healthy);
    }
    camVzObsOut_t out;
    camVzObsGet(&obs, &out);
    const double eh = out.h - simZ(&sim, simNow(&sim));
    printf("re-anchor guards, testRelock's 4 px/frame drag with the CUSUM on: alarms %u (at %.2f s), re-anchors %u, "
           "h err at 14 s %+.3f m, identical to the re-anchor off %d\n", (unsigned)obs.cnt.alarms, obs.alarmT * 1e-6,
           (unsigned)obs.cnt.reanchors, eh, same);
    CHECK(obs.cnt.alarms >= 1 && unhealthyInDrag && obs.cnt.reanchors == 0 && same && fabs(eh) < 0.1,
          "4 px drag, CUSUM on: alarm %u, unhealthy in the drag %d, re-anchors %u, identical %d, h err %.3f",
          (unsigned)obs.cnt.alarms, unhealthyInDrag, (unsigned)obs.cnt.reanchors, same, eh);
  }
  /* 2. Drags in the hover after testGainReanchor's 0.35 m/s climb to 1.05 m
   * at a constant G: inside the re-anchor window (from hover + 0.2 s, the
   * gate guard) and after it. While healthy, the speed error stays at the
   * re-anchor-off level. */
  {
    static const double gs[] = { 50.0, 120.0 };
    static const double pxs[] = { -4.0, -2.0, 1.5, 2.0, 3.0, 4.0 };
    static const double starts[] = { 0.2, 0.6, 1.0, 1.5, 2.5, 4.0 };
    int runs = 0, worse = 0;
    double worstOn = 0.0, worstOff = 0.0, worstDiff = -1.0;
    unsigned ra = 0;
    for (unsigned gi = 0; gi < 2; gi++) {
      for (unsigned pi = 0; pi < sizeof(pxs) / sizeof(pxs[0]); pi++) {
        for (unsigned si = 0; si < sizeof(starts) / sizeof(starts[0]); si++) {
          for (uint32_t seed = 1; seed <= 2; seed++) {
            const dragRun_t off = dragRun(0.0f, gs[gi], pxs[pi], starts[si], 2.5, seed);
            const dragRun_t on = dragRun(0.3f, gs[gi], pxs[pi], starts[si], 2.5, seed);
            runs++;
            worstOn = fmax(worstOn, on.vErrMax);
            worstOff = fmax(worstOff, off.vErrMax);
            worstDiff = fmax(worstDiff, on.vErrMax - off.vErrMax);
            ra += on.reanchors;
            if (on.vErrMax > off.vErrMax + 0.02) {
              worse++;
              printf("  drag G %.0f %+.1f px from hover + %.1f s seed %u: healthy |v err| %.3f m/s (off %.3f), "
                     "re-anchors %u\n", gs[gi], pxs[pi], starts[si], (unsigned)seed, on.vErrMax, off.vErrMax,
                     on.reanchors);
            }
          }
        }
      }
    }
    printf("re-anchor guards, %d hover drags (G 50/120 px/m, -4..+4 px/frame, 2.5 s from hover + 0.2..4 s): worst "
           "healthy |v err| %.3f m/s (re-anchor off %.3f, largest excess %+.3f), re-anchors in the drags %u\n", runs,
           worstOn, worstOff, worstDiff, ra);
    CHECK(worse == 0 && worstOn <= 0.12, "hover drags: %d runs with a healthy speed error over the re-anchor-off "
          "one + 0.02 m/s, worst %.3f m/s", worse, worstOn);
  }
}

/* 1 kHz input with 10 ms blocks against 100 Hz input. */
static void testRates(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  camVzObsOut_t o100, o1k;
  simStart(&sim, &obs, &cfg, 10000, 0.002);
  sim.accNoiseG = 0.0;
  sim.pxNoise = 0.0;
  simRun(&sim, 10.0);
  camVzObsGet(&obs, &o100);
  const double t100 = simNow(&sim);
  simStart(&sim, &obs, &cfg, 1000, 0.002);
  sim.accNoiseG = 0.0;
  sim.pxNoise = 0.0;
  simRun(&sim, 10.0);
  camVzObsGet(&obs, &o1k);
  const double t1k = simNow(&sim);
  printf("rates (no noise): 100 Hz offset err %+.3f mg h err %+.4f m v err %+.4f; 1 kHz %+.3f mg %+.4f m %+.4f\n",
         (o100.biasG - sim.biasG) * 1e3, o100.h - simZ(&sim, t100), o100.v - simV(&sim, t100),
         (o1k.biasG - sim.biasG) * 1e3, o1k.h - simZ(&sim, t1k), o1k.v - simV(&sim, t1k));
  CHECK(fabs(o100.biasG - sim.biasG) * 1e3 < 0.3 && fabs(o1k.biasG - sim.biasG) * 1e3 < 0.3,
        "noise-free offset exact at both rates");
  /* h keeps what the 2 mg prior error and the default gain cost before the
   * tracker had a G (the camera measures speed only) */
  CHECK(fabs(o1k.h - simZ(&sim, t1k)) < 0.03 && fabs(o1k.v - simV(&sim, t1k)) < 0.005 &&
        fabs(o100.h - simZ(&sim, t100)) < 0.03 && fabs(o100.v - simV(&sim, t100)) < 0.005 &&
        fabs(o1k.h - o100.h) < 0.002, "noise-free h and v");
  CHECK(fabs(o1k.sigmaV - o100.sigmaV) < 0.1 * o100.sigmaV && fabs(o1k.sigmaBiasG - o100.sigmaBiasG) <
        0.1 * o100.sigmaBiasG, "same filter at both rates: sigma_v %.4f/%.4f sigma_b %.3f/%.3f mg", o1k.sigmaV,
        o100.sigmaV, o1k.sigmaBiasG * 1e3, o100.sigmaBiasG * 1e3);
  /* latency: 60 ms and 230 ms give the same noise-free result */
  simStart(&sim, &obs, &cfg, 1000, 0.002);
  sim.accNoiseG = 0.0;
  sim.pxNoise = 0.0;
  sim.latS = 0.23;
  simRun(&sim, 10.0);
  camVzObsGet(&obs, &o1k);
  CHECK(fabs(o1k.biasG - sim.biasG) * 1e3 < 0.3 && fabs(o1k.v - simV(&sim, simNow(&sim))) < 0.005,
        "230 ms latency handled: offset err %.3f mg", (o1k.biasG - sim.biasG) * 1e3);
}

static void testFaultsAndConfig(void)
{
  camVzObsConfig_t cfg, bad;
  camVzObsDefaultConfig(&cfg);
  CHECK(camVzObsConfigValid(&cfg), "default valid");
  camVzObsPlainConfig(&bad);
  CHECK(camVzObsConfigValid(&bad), "plain valid");
  bad = cfg;
  bad.sigPx = NAN;
  CHECK(!camVzObsConfigValid(&bad), "NaN refused");
  bad = cfg;
  bad.predictPeriodUs = 1000;
  CHECK(!camVzObsConfigValid(&bad), "1 ms blocks refused (ring too short)");
  bad = cfg;
  bad.gMax = bad.gMin;
  CHECK(!camVzObsConfigValid(&bad), "gMax <= gMin refused");
  bad = cfg;
  bad.sigAcc = 0.0f;
  CHECK(!camVzObsInit(&obs, &bad) && obs.cfg.sigAcc == 0.25f, "invalid config: default used");
  CHECK(camVzObsInit(&obs, NULL), "NULL config: default");
  /* non-finite input latches a fault */
  simStart(&sim, &obs, &cfg, 10000, 0.0);
  simRun(&sim, 5.0);
  camVzObsOut_t out;
  camVzObsGet(&obs, &out);
  CHECK(out.healthy, "healthy before the fault");
  camVzObsImu(&obs, sim.base + sim.k * sim.imuUs, NAN, false, 30000.0f);
  sim.k++;
  simRun(&sim, 5.5);
  camVzObsGet(&obs, &out);
  CHECK(out.fault && !out.healthy && sim.last == CAM_VZ_F_SKIPPED, "NaN: fault latched, frames skipped");
  camVzObsReset(&obs);
  camVzObsGet(&obs, &out);
  CHECK(!out.fault && !out.healthy && out.sinceAcceptUs == UINT32_MAX && obs.cfg.sigAcc == 0.25f,
        "reset clears the fault, keeps the config");
  /* run-time limit */
  camVzObsImu(&obs, 1000, 1.0f, true, 0.0f);
  camVzObsImu(&obs, 1000 + CAM_VZ_MAX_RUN_US + 1, 1.0f, true, 0.0f);
  camVzObsGet(&obs, &out);
  CHECK(out.fault, "30 min without a reset faults");
  /* time going backwards is ignored */
  camVzObsReset(&obs);
  camVzObsImu(&obs, 1000000, 1.0f, true, 0.0f);
  camVzObsImu(&obs, 1010000, 1.0f, true, 0.0f);
  camVzObsImu(&obs, 1005000, 1.0f, true, 0.0f);
  CHECK(obs.cnt.imuIgnored == 1 && obs.tIn == 10000, "non-monotonic sample ignored");
}

#ifndef CVO_BASE
/* The guard events of testLateNoise (lateEvKind: its table's row; 100: the glitch), from lateEvFrom (s). */
static int lateEvKind;
static double lateEvFrom;
static void modLateEvent(sim_t *s, camVzObsFrame_t *f, double tc)
{
  (void)s;
  const double dt = tc - lateEvFrom;
  if (dt < 0.0) {
    return;
  }
  switch (lateEvKind) {
    case 0: if (dt < 0.06) f->dy += 25.0f; break;                /* one frame */
    case 1: if (dt < 0.06) f->dx += 2.0f; break;
    case 2: if (dt < 0.3) f->scale = 0.06f; break;               /* the smoothed expansion over 0.3 /s */
    case 3: if (dt < 0.3) f->dy += 0.5f; break;
    case 4: if (dt < 0.06) f->gyroAngle[2] += 0.4f; break;       /* > yawResetRad at once */
    case 8: if (dt < 0.06) f->ok = false; break;
    case 101: if (dt < 0.06) f->ok = false; break;              /* the first frame back after a gap: QUALITY */
    case 100: if (dt < 1.0) f->dy -= (float)(40.0 * 0.1 * f->dtUs * 1e-6); break;   /* a 0.1 m/s climb read */
    default: break;
  }
}
#endif

#ifndef CVO_BASE
/* ========================================================================= */
/* 3. late offset noise (qLateMult, docs/autonomy-a2b.md 25)                   */
/* ========================================================================= */

/* A 90 s hover over a far scene (G 40 px/m; the sim's bob keeps the tracker's G) whose accelerometer offset turns at
 * 25 s and rises 0.2 mg/s (the 10-15 flights: 0.12-0.25 mg/s from ~20-40 s), the IMU at 100 Hz with the 10-15
 * hovers' noise of a 10 ms mean (4.8-5.3 mg rms) and 0.05 px of camera noise, the multiplier x5 (the study's first
 * strength). The other tests' 20 mg and 0.15 px move a boosted offset more than 1 mg a second (the rate guard's
 * event, CAM_VZ_QLATE_RATE_G) every few seconds; on the 10-15 replays x5 trips it 0-1 times a flight
 * (docs/autonomy-a2b.md 25). */
#define LATE_MULT 5.0f
static void lateSetup(sim_t *s, camVzObs_t *o, float mult, float stepS, uint32_t seed)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  cfg.qLateMult = mult;
  cfg.qLateS = stepS;
  simStart(s, o, &cfg, 10000, 0.0);
  s->rng = seed;
  s->gTrue = s->gTrue2 = 40.0;
  s->h1 = 1.0;
  s->climbS = 3.0;
  s->biasRate = 0.0002;
  s->biasFrom = 25.0;
  s->accNoiseG = 0.005;
  s->pxNoise = 0.05;
}

static double lateTrueBias(const sim_t *s, double t)
{
  return s->biasG + (t > s->biasFrom ? s->biasRate * (t - s->biasFrom) : 0.0);
}

/* the observer's biasG every 10 ms (index = t x 100: every frame's value is seen) */
#define LATE_HIST_N 9100
static float lateHist[LATE_HIST_N];

/* Is v within the observer's offsets between t0 and t1 (s; the held offset is a mean of some of them)? */
static bool lateHeldFrom(float v, double t0, double t1)
{
  float lo = INFINITY, hi = -INFINITY;
  for (int k = (int)floor(t0 * 100.0 + 1e-6); k <= (int)ceil(t1 * 100.0 - 1e-6); k++) {
    if (k >= 1 && k < LATE_HIST_N) {
      lo = fminf(lo, lateHist[k]);
      hi = fmaxf(hi, lateHist[k]);
    }
  }
  return v >= lo - 1e-7f && v <= hi + 1e-7f;
}

/* The guard's bound on the offset's variance: its size at the step plus what qBiasG alone adds since the previous
 * frame (latePre), with the blocks since and a margin. */
static bool lateP22Ok(const camVzObs_t *o)
{
  const float qb = o->cfg.qBiasG * CAM_VZ_G;
  return o->P[2][2] <= 1.05f * (o->p22Norm + qb * qb * ((float)(o->tIn - o->tLatePrev) * 1e-6f + 0.1f));
}

static void testLateNoise(void)
{
  static camVzObs_t o1, oL;
  static sim_t s1, sL;
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  CHECK(cfg.qLateMult == 1.0f && cfg.qLateS == 40.0f, "default: x1 (off) from 40 s");

  /* x1 against x5 from 40 s on the same inputs */
  lateSetup(&s1, &o1, 1.0f, 40.0f, 777u);
  lateSetup(&sL, &oL, LATE_MULT, 40.0f, 777u);
  double firstDiff = -1.0, e1 = 0.0, e10 = 0.0;
  int n = 0, heldOk = 0, heldN = 0, offOk = 0;
  for (int k = 1; k <= 9000; k++) {
    const double t = k * 0.01;
    simRun(&s1, t);
    simRun(&sL, t);
    camVzObsOut_t a, b;
    camVzObsGet(&o1, &a);
    camVzObsGet(&oL, &b);
    lateHist[k] = b.biasG;
    if (k % 10 != 0) {
      continue;
    }
    if (firstDiff < 0.0 && memcmp(&a, &b, sizeof(a)) != 0) {
      firstDiff = t;
    }
    offOk += a.biasHeldG == a.biasG && camVzObsLateState(&o1) == CAM_VZ_LATE_OFF;
    if (t >= 45.0 && camVzObsLateState(&oL) == CAM_VZ_LATE_ON) {
      heldN++;
      heldOk += lateHeldFrom(b.biasHeldG, t - 4.62, t - 2.0);
    }
    if (t >= 60.0) {
      const double tb = lateTrueBias(&s1, simNow(&s1));
      e1 += fabs(a.biasG - tb);
      e10 += fabs(b.biasG - tb);
      n++;
    }
  }
  e1 = e1 / n * 1e3;
  e10 = e10 / n * 1e3;
  printf("late offset noise, 90 s hover, offset +0.2 mg/s from 25 s, G 40: first output difference x5 vs x1 at %.1f s "
         "(step 40 s after the lift-off at 0.6 s); mean |offset error| 60-90 s x1 %.2f mg, x5 %.2f mg; with the "
         "multiplier %.1f s, %u guard stops; the held offset within the one 2.0-4.6 s ago in %d of %d samples\n", firstDiff, e1, e10,
         oL.lateUs * 1e-6, (unsigned)oL.lateGuards, heldOk, heldN);
  CHECK(firstDiff >= 40.6 && firstDiff <= 40.8, "bit-identical until the step (first difference %.1f s)", firstDiff);
  CHECK(offOk == 900, "x1: off, the held offset is biasG (%d of 900)", offOk);
  CHECK(heldN > 400 && heldOk == heldN, "the controller's offset within the observer's of 2.0-4.6 s ago (%d of %d)",
        heldOk, heldN);
  CHECK(e10 < 0.4 * e1, "the drifting offset followed closer: x5 %.2f mg against x1 %.2f mg", e10, e1);
  CHECK(camVzObsLateState(&oL) == CAM_VZ_LATE_ON && oL.lateGuards <= 1 && oL.lateUs * 1e-6 > 45.0,
        "on since the step: %.1f s, %u guard stops (the offset catching up 5 s after the step may move 1 mg a second)",
        oL.lateUs * 1e-6, (unsigned)oL.lateGuards);

  /* The clock: the ground flag for 1 s mid-flight before the step (20-21 s) does not restart it */
  static camVzObs_t o;
  static sim_t s;
  lateSetup(&s, &o, LATE_MULT, 40.0f, 778u);
  s.gndFrom = 20.0;
  s.gndTo = 21.0;
  simRun(&s, 19.9);
  const uint32_t first = o.tFirstLift;
  CHECK(o.hasFirstLift && first == o.tLift && fabs(first * 1e-6 - 0.6) < 0.011, "the first lift-off at %.2f s",
        first * 1e-6);
  simRun(&s, 25.0);
  CHECK(o.tLift > first + 20000000u && o.tFirstLift == first, "re-lift at %.2f s: tLift restarts, the step's clock "
        "does not (%.2f s)", o.tLift * 1e-6, o.tFirstLift * 1e-6);
  simRun(&s, 40.5);
  CHECK(!o.late, "no step before first + 40 s");
  simRun(&s, 40.8);
  CHECK(o.late && camVzObsLateState(&o) == CAM_VZ_LATE_ON, "the step at the first lift-off + 40 s (%.2f s), not the "
        "re-lift's + 40", o.tLatePrev * 1e-6);
  /* ... and on the ground after the step (60-61 s) the multiplier stops */
  simRun(&s, 59.95);
  const uint32_t lateUs0 = o.lateUs;
  s.gndFrom = 60.0;
  s.gndTo = 61.0;
  simRun(&s, 60.3);
  const float p0 = o.P[2][2];
  simRun(&s, 60.5);
  const float qb = cfg.qBiasG * CAM_VZ_G;
  const double grow = (o.P[2][2] - p0) / (qb * qb * 0.2);
  CHECK(camVzObsLateState(&o) == CAM_VZ_LATE_GUARD && grow > 0.99 && grow < 1.01,
        "on the ground: the offset's variance grows at x%.3f of the base noise (the multiplier off)", grow);
  simRun(&s, 62.0);
  CHECK(camVzObsLateState(&o) == CAM_VZ_LATE_GUARD, "back in the air: the guard (no update for 0.5 s)");
  CHECK(o.lateUs - lateUs0 <= 150000u, "the time on the ground not counted as on at the re-lift's first frame (+%.2f "
        "s from 59.95 s)", (o.lateUs - lateUs0) * 1e-6);
  simRun(&s, 65.0);
  CHECK(camVzObsLateState(&o) == CAM_VZ_LATE_ON && o.tFirstLift == first && o.lateGuards >= 1,
        "on again 3 s after the re-lift's first frame (guards %u)", (unsigned)o.lateGuards);
  camVzObsReset(&o);
  CHECK(!o.hasFirstLift && !o.late && !o.lateOn && o.lateUs == 0 && o.lateGuards == 0 && o.cfg.qLateMult == LATE_MULT,
        "a new sortie (reset): the clock restarts, the config stays");

  /* Every guard event at 60 s: the base noise, the offset's variance held at its size at the step, the held offset
   * frozen, on again 3 s after the last event. A QUALITY refusal is not one. */
  static const struct {
    const char *what;
    int kind;   /* 0 a frame modifier, 1 the turn flag, 2 the motion flag, 3 frames stop */
    frameMod_t mod;
    bool guard;
    double backS;   /* on again by 60 + this */
  } ev[] = {
    {"a frame outside the innovation gate (its CUSUM alarm holds 3 s)", 0, NULL, true, 0.0},
    {"a lateral refusal", 0, NULL, true, 0.0},
    {"an expansion refusal", 0, NULL, true, 0.0},
    {"a CUSUM build-up (a 0.5 px/frame drag for 0.3 s)", 0, NULL, true, 0.0},
    {"a yaw restart of the tracker (no G until it re-tracks)", 0, NULL, true, 0.0},
    {"a commanded turn (1 s)", 1, NULL, true, 0.0},
    {"a commanded motion (1 s)", 2, NULL, true, 0.0},
    {"the frames stop for 1 s", 3, NULL, true, 0.0},
    {"one QUALITY refusal", 0, NULL, false, 0.0},
  };
  for (size_t i = 0; i < sizeof(ev) / sizeof(ev[0]); i++) {
    lateSetup(&s, &o, LATE_MULT, 40.0f, 779u);
    simRun(&s, 59.9);
    CHECK(camVzObsLateState(&o) == CAM_VZ_LATE_ON, "%s: on before it", ev[i].what);
    lateEvKind = (int)i;
    lateEvFrom = 60.0;
    s.mod = i <= 4 || i == 8 ? modLateEvent : NULL;
    if (ev[i].kind == 1) {
      camVzObsSetTurn(&o, true);
    } else if (ev[i].kind == 2) {
      camVzObsSetMotion(&o, true);
    } else if (ev[i].kind == 3) {
      s.camOffS = 60.0;
      s.camOnS = 61.0;
    }
    simRun(&s, 60.7);   /* the frame's delivery (130 ms), or 0.5 s without an accepted update */
    const float held0 = o.bHeldG;
    const bool guard0 = camVzObsLateState(&o) == CAM_VZ_LATE_GUARD;
    bool p22Ok = true, heldOk = true;
    camVzObsSetTurn(&o, ev[i].kind == 1);   /* the flags for 1 s */
    camVzObsSetMotion(&o, ev[i].kind == 2);
    for (double t = 60.75; t < 61.0 + 1e-9; t += 0.05) {
      simRun(&s, t);
      p22Ok = p22Ok && (!guard0 || lateP22Ok(&o) || ev[i].kind == 3);
      heldOk = heldOk && (!guard0 || o.bHeldG == held0);
    }
    camVzObsSetTurn(&o, false);
    camVzObsSetMotion(&o, false);
    /* on again 3 s after the last event (o.tGuard), at the first frame after it */
    double tOn = -1.0;
    for (double t = 61.05; t < 80.0 && tOn < 0.0; t += 0.01) {
      simRun(&s, t);
      if (camVzObsLateState(&o) == CAM_VZ_LATE_GUARD) {
        /* the frames stopped: the variance grows until the next frame bounds it before its update */
        p22Ok = p22Ok && (lateP22Ok(&o) || (ev[i].kind == 3 && t < 61.3));
        heldOk = heldOk && o.bHeldG == held0;
      } else {
        tOn = t;
      }
    }
    const double quiet = tOn - o.tGuard * 1e-6;
    if (ev[i].guard) {
      CHECK(guard0 && p22Ok && heldOk && quiet >= 3.0 && quiet <= 3.15,
            "%s: the guard (%d), the offset's variance <= its step size (%d), the held offset frozen (%d), on again "
            "%.2f s after the last event (at %.2f s)", ev[i].what, guard0, p22Ok, heldOk, quiet, tOn);
    } else {
      CHECK(!guard0 && o.lateGuards == 0, "%s: no guard (%d, %u stops)", ev[i].what, guard0,
            (unsigned)o.lateGuards);
    }
  }

  /* A guard run longer than CAM_VZ_QLATE_FREEZE_US (Claude's motion held 20 s from 60 s): the held offset frozen that
   * long, then the delayed mean again (of an offset at qBiasG alone), the guard all the while */
  lateSetup(&s, &o, LATE_MULT, 40.0f, 781u);
  float frozen = 0.0f;
  double tFrozenEnd = -1.0;
  bool guardAll = true, delayedOk = true;
  int delayedN = 0;
  for (int k = 1; k <= 8000; k++) {
    const double t = k * 0.01;
    camVzObsSetMotion(&o, t >= 60.0);
    simRun(&s, t);
    camVzObsOut_t b;
    camVzObsGet(&o, &b);
    lateHist[k] = b.biasG;
    if (k == 6020) {
      frozen = b.biasHeldG;
    }
    if (t >= 60.2) {
      guardAll = guardAll && camVzObsLateState(&o) == CAM_VZ_LATE_GUARD;
      if (tFrozenEnd < 0.0 && b.biasHeldG != frozen) {
        tFrozenEnd = t;
      }
    }
    if (t >= 65.5 && k % 10 == 0) {
      delayedN++;
      delayedOk = delayedOk && lateHeldFrom(b.biasHeldG, t - 4.62, t - 2.0);
    }
  }
  const double freezeS = tFrozenEnd - o.tFreeze * 1e-6;
  printf("late offset noise: a motion held 60-80 s: the held offset frozen until %.2f s (%.2f s after the guard run's "
         "first frame), then the delayed mean (%d samples)\n", tFrozenEnd, freezeS, delayedN);
  CHECK(guardAll && freezeS >= CAM_VZ_QLATE_FREEZE_US * 1e-6 && freezeS <= CAM_VZ_QLATE_FREEZE_US * 1e-6 + 0.1 &&
            delayedOk && delayedN > 100,
        "a held motion: the guard throughout (%d), the held offset frozen %.2f s (%.1f s at most), then within the "
        "observer's offsets of 2.0-4.6 s ago (%d)", guardAll, freezeS, CAM_VZ_QLATE_FREEZE_US * 1e-6, delayedOk);

  /* After a 3 s gap the offset's variance has regrown at the base rate, as today's filter's does; only the
   * multiplier's share is taken back. The first frame back is a QUALITY refusal (no update), so P[2][2] after it is
   * the guard's bound: its size at the step plus qBiasG's growth since the frame before the gap. */
  lateSetup(&s, &o, LATE_MULT, 40.0f, 782u);
  lateEvKind = 101;
  lateEvFrom = 63.0;
  s.mod = modLateEvent;
  s.camOffS = 60.0;
  s.camOnS = 63.0;
  simRun(&s, 62.9);
  const uint32_t tPrev = o.tLatePrev;
  const float pGap = o.P[2][2];
  for (double t = 62.9; o.tLatePrev == tPrev && t < 64.0; t += 0.001) {
    simRun(&s, t);
  }
  const float qbG = cfg.qBiasG * CAM_VZ_G;
  const float want = o.p22Norm + qbG * qbG * (float)(o.tLatePrev - tPrev) * 1e-6f;
  printf("late offset noise: a 3.0 s gap (frames 60-63 s): P22 before the first frame back %.3e (the multiplier's "
         "growth), after it %.3e = the step's %.3e + the base growth over %.2f s (%.3e)\n", pGap, o.P[2][2],
         o.p22Norm, (o.tLatePrev - tPrev) * 1e-6, want);
  CHECK(o.cnt.quality > 0 && fabsf(o.P[2][2] - want) <= 0.01f * want && want >= 1.2f * o.p22Norm && pGap > want,
        "after a 3 s gap: the offset's variance %.3e = its step size + the base noise's growth %.3e (regrown as "
        "today's; the multiplier's share %.3e taken back)", o.P[2][2], want, pGap - want);
  s.mod = NULL;

  /* A glitch the observer takes (the camera reads a 0.1 m/s climb for 1 s: a 0.27 px/frame drag at G 40, inside the
   * gate and below the CUSUM alarm), then the camera is lost: the controller keeps the offset from before the
   * glitch (the held offset), today the one the glitch moved */
  lateSetup(&s1, &o1, 1.0f, 40.0f, 780u);
  lateSetup(&sL, &oL, LATE_MULT, 40.0f, 780u);
  lateEvKind = 100;
  lateEvFrom = 60.0;
  s1.mod = sL.mod = modLateEvent;
  s1.camOffS = sL.camOffS = 61.0;
  s1.camOnS = sL.camOnS = 1e9;
  for (int k = 1; k <= 6400; k++) {
    simRun(&s1, k * 0.01);
    simRun(&sL, k * 0.01);
    camVzObsOut_t b;
    camVzObsGet(&oL, &b);
    lateHist[k] = b.biasG;
  }
  camVzObsOut_t a, b;
  camVzObsGet(&o1, &a);
  camVzObsGet(&oL, &b);
  const double tb = lateTrueBias(&s1, 61.0);
  printf("late offset noise: a 0.1 m/s glitch for 1 s at 60 s, then the camera lost: offset error at 61 s x1 %+.2f "
         "mg (the controller keeps it), x5 %+.2f mg, the held one x5 keeps %+.2f mg (%s)\n", (a.biasG - tb) * 1e3,
         (b.biasG - tb) * 1e3, (b.biasHeldG - tb) * 1e3,
         lateHeldFrom(b.biasHeldG, 55.0, 59.9) ? "from before the glitch" : "NOT from before the glitch");
  CHECK(!a.healthy && !b.healthy && a.biasHeldG == a.biasG, "the camera lost: not healthy; x1 keeps its latest");
  CHECK(lateHeldFrom(b.biasHeldG, 55.0, 59.9) && fabs(b.biasHeldG - tb) < fabs(b.biasG - tb),
        "x5: the held offset is from before the glitch, closer than the one the glitch moved");

  /* the config's limits */
  camVzObsConfig_t c = cfg;
  const struct { float mult, step; bool ok; } lim[] = {
    {1.0f, 40.0f, true}, {5.0f, 40.0f, true}, {3.0f, 40.0f, true}, {1.0f, 20.0f, true}, {1.0f, 120.0f, true},
    {0.99f, 40.0f, false}, {5.01f, 40.0f, false}, {6.0f, 40.0f, false}, {10.0f, 40.0f, false}, {NAN, 40.0f, false},
    {5.0f, 19.9f, false}, {5.0f, 120.1f, false}, {5.0f, INFINITY, false},
  };
  for (size_t i = 0; i < sizeof(lim) / sizeof(lim[0]); i++) {
    c.qLateMult = lim[i].mult;
    c.qLateS = lim[i].step;
    CHECK(camVzObsConfigValid(&c) == lim[i].ok, "x%.2f from %.1f s: %s", lim[i].mult, lim[i].step,
          lim[i].ok ? "valid" : "refused");
  }
  c.qLateMult = 11.0f;
  CHECK(!camVzObsInit(&o, &c) && o.cfg.qLateMult == 1.0f, "an invalid multiplier: refused, the default (off) used");
  camVzObsPlainConfig(&c);
  CHECK(c.qLateMult == 1.0f, "the plain config: off");
}

/* ========================================================================= */
/* 4. healthy offset speed-up (hbMult, docs/autonomy-a2b.md 37)                */
/* ========================================================================= */

/* A 60 s hover over a far scene (G 40 px/m, the late offset noise's sim) whose accelerometer offset FALLS 8 mg over
 * 5-25 s (-0.4 mg/s, the 2026-10-17 15:26 flight's fall, the deepest logged) and stays there. The prior is exact at
 * the lift-off, as the logged take-offs' ground baseline was. */
#define HB_MULT 3.0f
static void hbSetup(sim_t *s, camVzObs_t *o, float mult, uint32_t seed)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  cfg.hbMult = mult;
  simStart(s, o, &cfg, 10000, 0.0);
  s->rng = seed;
  s->gTrue = s->gTrue2 = 40.0;
  s->h1 = 1.0;
  s->climbS = 3.0;
  s->biasRate = -0.0004;
  s->biasFrom = 5.0;
  s->biasTo = 25.0;
  s->accNoiseG = 0.005;
  s->pxNoise = 0.05;
}

static double hbTrueBias(const sim_t *s, double t)
{
  const double te = t > s->biasTo ? s->biasTo : t;
  return s->biasG + (te > s->biasFrom ? s->biasRate * (te - s->biasFrom) : 0.0);
}

/* The offset variance's growth over blocks without a frame, as a multiple of the base noise's (qBiasG^2 d): exactly
 * hbMult^2 with the speed-up on, 1 off (F's offset row is [0 0 1]: nothing else touches P[2][2] between frames). */
static double hbGrow(const camVzObs_t *o, float p0, double dS)
{
  const float qb = o->cfg.qBiasG * CAM_VZ_G;
  return (o->P[2][2] - p0) / (qb * qb * dS);
}

static void testHealthyBoost(void)
{
  static camVzObs_t o1, oH;
  static sim_t s1, sH;
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  CHECK(cfg.hbMult == 0.0f, "default: off (NVS cvhb 0)");

  /* off against x3 on the same inputs */
  hbSetup(&s1, &o1, 0.0f, 991u);
  hbSetup(&sH, &oH, HB_MULT, 991u);
  double firstDiff = -1.0, firstAcc = -1.0, firstG = -1.0, e1 = 0.0, eH = 0.0, lag1 = 0.0, lagH = 0.0;
  int n = 0, onN = 0, airN = 0, earlyOn = 0, offBad = 0;
  for (int k = 1; k <= 6000; k++) {
    const double t = k * 0.01;
    simRun(&s1, t);
    simRun(&sH, t);
    camVzObsOut_t a, b;
    camVzObsGet(&o1, &a);
    camVzObsGet(&oH, &b);
    offBad += a.hbOn || o1.hbUs != 0 || o1.hbOn;
    if (firstAcc < 0.0 && oH.hasAccept) {
      firstAcc = t;
    }
    if (firstG < 0.0 && oH.gt.hasG) {
      firstG = t;
    }
    if (!oH.hasAccept || !oH.gt.hasG) {
      earlyOn += b.hbOn || oH.hbOn || oH.hbUs != 0;
    } else if (b.airborne) {
      airN++;
      onN += b.hbOn;
    }
    if (firstDiff < 0.0 && (a.h != b.h || a.v != b.v || a.biasG != b.biasG || a.sigmaBiasG != b.sigmaBiasG)) {
      firstDiff = t;
    }
    if (t >= 8.0 && t <= 40.0 && k % 10 == 0) {
      const double tb = hbTrueBias(&s1, simNow(&s1));
      e1 += fabs(a.biasG - tb);
      eH += fabs(b.biasG - tb);
      lag1 = fmax(lag1, fabs(a.biasG - tb));
      lagH = fmax(lagH, fabs(b.biasG - tb));
      n++;
    }
  }
  e1 = e1 / n * 1e3;
  eH = eH / n * 1e3;
  const double firstOn = fmax(firstAcc, firstG);
  printf("healthy offset speed-up, 60 s hover, offset -8 mg over 5-25 s, G 40: first accepted frame %.2f s, G tracked "
         "%.2f s, first output difference x3 vs off %.2f s; offset lag 8-40 s off mean %.2f mg (max %.2f), x3 mean "
         "%.2f mg (max %.2f); x3 on in %d of %d airborne samples after it, %.1f s in all\n", firstAcc, firstG,
         firstDiff, e1, lag1 * 1e3, eH, lagH * 1e3, onN, airN, oH.hbUs * 1e-6);
  CHECK(offBad == 0, "off: never on, no time counted (%d samples)", offBad);
  CHECK(earlyOn == 0 && firstAcc > 1.5 && firstDiff >= firstOn && firstDiff <= firstOn + 0.3,
        "nothing before the first accepted frame with a tracked G (%d samples on; first accepted %.2f s, G %.2f s, "
        "first difference %.2f s)", earlyOn, firstAcc, firstG, firstDiff);
  CHECK(eH < 0.75 * e1 && eH > 0.3 * e1 && lagH < 0.8 * lag1,
        "the falling offset followed closer: x3 %.2f mg (max %.2f) against off %.2f mg (max %.2f)", eH, lagH * 1e3,
        e1, lag1 * 1e3);
  CHECK(onN >= airN - airN / 20 && oH.hbUs * 1e-6 > 0.9 * (60.0 - firstOn),
        "on through the healthy hover: %d of %d samples, %.1f s", onN, airN, oH.hbUs * 1e-6);

  /* a 6 s camera gap (30-36 s): on for the healthy window (2 s) after the last accepted frame with the offset's
   * variance growing x9, then off (x1) until the first accepted frame after the gap; the time off not counted */
  static camVzObs_t o;
  static sim_t s;
  camVzObsOut_t out;
  hbSetup(&s, &o, HB_MULT, 992u);
  s.camOffS = 30.0;
  s.camOnS = 36.0;
  simRun(&s, 30.3);
  camVzObsGet(&o, &out);
  const double lastAcc = o.lastAccept * 1e-6;
  CHECK(out.hbOn && out.healthy && lastAcc > 29.9 && lastAcc < 30.3, "the gap's start: on (last accepted %.2f s)",
        lastAcc);
  float p0 = o.P[2][2];
  simRun(&s, 31.3);
  double grow = hbGrow(&o, p0, 1.0);
  camVzObsGet(&o, &out);
  CHECK(out.hbOn && grow > 8.9 && grow < 9.1, "no frame, inside the window: on, the variance grows x%.2f (x9)", grow);
  simRun(&s, 32.5);
  camVzObsGet(&o, &out);
  CHECK(!out.hbOn && !out.healthy && out.sinceAcceptUs > 2000000u && o.hbOn,
        "2 s after the last accepted frame: off (since %.2f s; the frame's flag still set)", out.sinceAcceptUs * 1e-6);
  const uint32_t hbUs0 = o.hbUs;
  p0 = o.P[2][2];
  simRun(&s, 34.5);
  grow = hbGrow(&o, p0, 2.0);
  CHECK(o.hbUs == hbUs0 && grow > 0.99 && grow < 1.01, "in the gap: not counted, the variance grows x%.3f (today's "
        "filter)", grow);
  simRun(&s, 36.7);
  camVzObsGet(&o, &out);
  CHECK(out.hbOn && out.healthy && o.hbUs > hbUs0 && o.hbUs - hbUs0 < 600000u,
        "the camera back: on again at its first accepted frame (+%.2f s counted since 32.5 s)",
        (o.hbUs - hbUs0) * 1e-6);

  /* a CUSUM alarm (a 1 px/frame drag from 40 s): off for its hold */
  hbSetup(&s, &o, HB_MULT, 993u);
  dragFrom = 40.0;
  dragTo = 42.0;
  dragPx = 1.0;
  s.mod = modDrag;
  simRun(&s, 41.0);
  camVzObsGet(&o, &out);
  CHECK(o.cnt.alarms == 1 && o.alarm && !out.hbOn && !o.hbOn, "the alarm (%u): off", (unsigned)o.cnt.alarms);
  const uint32_t hbUs1 = o.hbUs;
  simRun(&s, 42.8);
  CHECK(o.alarm && o.hbUs == hbUs1, "through the hold: not counted");
  simRun(&s, 45.0);
  camVzObsGet(&o, &out);
  CHECK(!o.alarm && out.healthy && out.hbOn && o.hbUs > hbUs1, "the hold over, updates again: on");

  /* the ground mid-flight (20-21 s): off, and after the re-lift off until its first accepted frame */
  hbSetup(&s, &o, HB_MULT, 994u);
  s.gndFrom = 20.0;
  s.gndTo = 21.0;
  simRun(&s, 20.5);
  camVzObsGet(&o, &out);
  CHECK(!out.hbOn && !o.hbOn && !out.airborne, "on the ground: off, the frame's flag cleared");
  const uint32_t hbUs2 = o.hbUs;
  simRun(&s, 21.03);
  camVzObsGet(&o, &out);
  CHECK(out.airborne && !out.hbOn && !o.hbOn && o.hbUs == hbUs2 && out.healthy,
        "re-lifted, no frame taken yet: off although the healthy window has not run out (since %.2f s)",
        out.sinceAcceptUs * 1e-6);
  simRun(&s, 21.6);
  camVzObsGet(&o, &out);
  CHECK(out.hbOn && o.hbUs > hbUs2 && o.lastAccept > 21000000u, "on again at the first accepted frame after the "
        "re-lift (%.2f s)", o.lastAccept * 1e-6);
  camVzObsReset(&o);
  CHECK(!o.hbOn && o.hbUs == 0 && o.cfg.hbMult == HB_MULT, "a new sortie (reset): cleared, the config stays");

  /* the config's limits: 0 (off) or 1..8, alone */
  camVzObsConfig_t c = cfg;
  const struct { float mult; bool ok; } lim[] = {
    {0.0f, true}, {1.0f, true}, {3.0f, true}, {5.0f, true}, {8.0f, true},
    {0.5f, false}, {0.99f, false}, {8.01f, false}, {9.0f, false}, {-1.0f, false}, {NAN, false},
  };
  for (size_t i = 0; i < sizeof(lim) / sizeof(lim[0]); i++) {
    c.hbMult = lim[i].mult;
    CHECK(camVzObsConfigValid(&c) == lim[i].ok, "x%.2f: %s", lim[i].mult, lim[i].ok ? "valid" : "refused");
  }
  c.hbMult = HB_MULT;
  c.qLateMult = 5.0f;
  CHECK(!camVzObsConfigValid(&c), "x3 with the late offset noise (R1 x5): refused (untested together)");
  c.qLateMult = 1.0f;
  c.t2kR = 0.5f;
  CHECK(!camVzObsConfigValid(&c), "x3 with the offset drift rate (T2 kR 0.5): refused");
  c.t2kR = 0.0f;
  c.hbMult = 9.0f;
  CHECK(!camVzObsInit(&o, &c) && o.cfg.hbMult == 0.0f, "an invalid multiplier: refused, the default (off) used");
  camVzObsPlainConfig(&c);
  CHECK(c.hbMult == 0.0f, "the plain config: off");
}
#endif

static void testTiming(void)
{
  camVzObsConfig_t cfg;
  camVzObsDefaultConfig(&cfg);
  simStart(&sim, &obs, &cfg, 1000, 0.0);
  simRun(&sim, 3.0);
  /* time the observer alone on recorded inputs: 1 kHz IMU, one frame per 59 ms */
  enum { N = 20000 };
  static float acc[N];
  for (int i = 0; i < N; i++) {
    acc[i] = (float)(1.0 + sim.biasG + 0.02 * gauss(&sim));
  }
  const uint64_t t0 = sim.base + sim.k * 1000u;
  clock_t c0 = clock();
  for (int i = 0; i < N; i++) {
    camVzObsImu(&obs, t0 + (uint64_t)i * 1000u, acc[i], false, 30000.0f);
  }
  const double imuNs = (double)(clock() - c0) / CLOCKS_PER_SEC / N * 1e9;
  camVzObsFrame_t f;
  int nf = 0;
  c0 = clock();
  for (int i = 0; i < 2000; i++) {
    const uint64_t now = t0 + (uint64_t)(N - 1) * 1000u;
    simFrame(&sim, (double)(now - sim.base) * 1e-6 - 0.13, &f);
    f.dy = 0.03f;
    f.dtUs = 59000;
    camVzObsFrame(&obs, &f, now);
    nf++;
  }
  const double frNs = (double)(clock() - c0) / CLOCKS_PER_SEC / nf * 1e9;
  printf("host timing: camVzObsImu %.0f ns/sample (1 kHz, 10 ms blocks), camVzObsFrame %.0f ns/frame (incl. the "
         "test's frame synthesis); sizeof(camVzObs_t) %u bytes\n", imuNs, frNs, (unsigned)sizeof(camVzObs_t));
  CHECK(sizeof(camVzObs_t) < 2400, "state size %u", (unsigned)sizeof(camVzObs_t));
  /* the re-anchor window's state sits in padding (2026-10-13): the block path's offsets and the shadow's layout
   * behind the observer stay as they were (an IRAM byte more would cost the heap 256 B, docs/autonomy-a2b.md 2.6) */
  /* + fwdLnAtG (2026-10-15, appended: 2028 -> 2032 with the 8-byte alignment; the shadow is in PSRAM, and the build
   * measures the IRAM, docs/autonomy-a2b.md 21.9) */
  /* + the late offset noise (2026-10-15): qLateMult and qLateS in the config (+8: every field behind it moves), its
   * state appended (+112, a 5 s offset history), the flags the 10 ms block path reads in padding: 2032 -> 2152
   * (PSRAM; the build's IRAM, docs/autonomy-a2b.md 25); + the freeze cap's run (the review fixes, a2b 25.9): 2160;
   * + the offset drift rate T2 (2026-10-16): 4 config floats (+16: every field behind them moves) and its state
   * appended (+16: the flag in padding, the time and 3 rates): 2192 (PSRAM; the build's IRAM, a2b 29);
   * + the healthy offset speed-up (2026-10-18): one config float (+4: every field behind it moves) and its state
   * appended (a flag and its time, +8, with the 8-byte alignment): 2208 (a2b 37) */
  CHECK(sizeof(void *) != 8 || sizeof(camVzObs_t) == 2216, "state size %u (2216 on a 64-bit host, including cadence evidence fields)",
        (unsigned)sizeof(camVzObs_t));
}

/* ---- fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31) -------------------------------------------------- */

/* A commanded turn's yaw (rad) at time t (s): a trapezoid of deg at rate (deg/s) and acc (deg/s^2) from fastTurnT0. */
static double fastTurnT0, fastTurnDeg, fastTurnRate, fastTurnAcc;
static double fastTurnYaw(double t)
{
  const double tau = t - fastTurnT0;
  if (tau <= 0.0) {
    return 0.0;
  }
  double ta = fastTurnRate / fastTurnAcc, tc;
  if (fastTurnAcc * ta * ta >= fastTurnDeg) {
    ta = sqrt(fastTurnDeg / fastTurnAcc);
    tc = 0.0;
  } else {
    tc = (fastTurnDeg - fastTurnRate * ta) / fastTurnRate;
  }
  const double T = 2.0 * ta + tc, vpk = fastTurnAcc * ta;
  double s;
  if (tau >= T) {
    s = fastTurnDeg;
  } else if (tau < ta) {
    s = 0.5 * fastTurnAcc * tau * tau;
  } else if (tau < ta + tc) {
    s = 0.5 * fastTurnAcc * ta * ta + vpk * (tau - ta);
  } else {
    const double td = T - tau;
    s = fastTurnDeg - 0.5 * fastTurnAcc * td * td;
  }
  return s * M_PI / 180.0;
}

static double fastTurnEnd(void)
{
  double ta = fastTurnRate / fastTurnAcc, tc;
  if (fastTurnAcc * ta * ta >= fastTurnDeg) {
    ta = sqrt(fastTurnDeg / fastTurnAcc);
    tc = 0.0;
  } else {
    tc = (fastTurnDeg - fastTurnRate * ta) / fastTurnRate;
  }
  return fastTurnT0 + 2.0 * ta + tc;
}

/* the frame's gyro angle about z over its motion window (as camera_flow.c integrates it) */
static void fastTurnMod(sim_t *s, camVzObsFrame_t *f, double tc)
{
  const double t1 = tc + s->o->cfg.gyroOffsetUs * 1e-6, t0 = t1 - f->dtUs * 1e-6;
  f->gyroAngle[2] += (float)(fastTurnYaw(t1) - fastTurnYaw(t0));
}

/* The lean's pitch rate (deg/s) at time t: steps of +-rate, 0.3 s each, every 1.2 s from fastLeanT0 (a fast MOVE's
 * accelerate / brake leans through the hold's 6 or 12 deg/s slew); the gyro window off by fastLeanSkewS from the image's
 * (the 26 ms model's error, a model value never re-fitted on the bench: 5 ms here). */
static double fastLeanT0, fastLeanRate, fastLeanSkewS;
static double fastLeanAngle(double t)
{
  const double tau = t - fastLeanT0;
  if (tau <= 0.0) {
    return 0.0;
  }
  const double per = 1.2, on = 0.3;
  const int n = (int)(tau / per);
  const double r = tau - n * per;
  /* +rate for 0.3 s, then -rate for 0.3 s, then still: each period ends level */
  double a = 0.0;
  if (r < on) {
    a = fastLeanRate * r;
  } else if (r < 2.0 * on) {
    a = fastLeanRate * (2.0 * on - r);
  }
  return a * M_PI / 180.0;
}

static void fastLeanMod(sim_t *s, camVzObsFrame_t *f, double tc)
{
  const double t1 = tc + s->o->cfg.gyroOffsetUs * 1e-6, t0 = t1 - f->dtUs * 1e-6;
  const double img = fastLeanAngle(t1) - fastLeanAngle(t0);                                  /* the image's */
  const double gyr = fastLeanAngle(t1 + fastLeanSkewS) - fastLeanAngle(t0 + fastLeanSkewS);  /* the gyro's window */
  f->gyroAngle[1] += (float)gyr;
  f->dy += (float)(s->o->cfg.kyGy * img);
}

/* (1) The yaw dwell (AUTO_MV_YAW_DWELL_MS 500, 300 from mvspeed 200 on): after a commanded turn the motion waits the
 * dwell, then reads the tracker's state (a lost G starts the re-track bob). The tracker restarts at 0.35 rad of
 * accumulated turn; with the frames delivered 94 ms after capture (7ab74b9's flow) and 67.5 ms apart, the restart must
 * show within 300 ms of the turn's end for every turn that restarts it (the worst: the restart's frame is the turn's
 * last). Turns of 21-90 deg at 40 deg/s and 120 deg/s^2 (and 30 / 60, the 100 % step), the turn starting at every
 * phase of the frame clock. (2) The re-track bob at 0.40 m/s^2 (a 1.265 s cycle of the same 4 cm): G back within its
 * 4 cycles and, at the firmware's end rule (the first cycle end one whole cycle after the tracker's first G), within
 * 10 % of the new view's (or no worse than the 0.20 bob's at its rule + 5 points). (3) The re-view bob's 2 cycles at
 * 0.40: G within 15 % of the new depth's. (4) A hover with the fast lean's pitch-rate steps (+-12 deg/s, 0.3 s; the
 * gyro window 5 ms off the image's): the height estimate's error no worse than with today's 6 deg/s by more than
 * 1 cm. */
static void testFastMoveObserver(void)
{
  /* (1) */
  {
    static const double degs[] = {21.0, 23.0, 25.0, 30.0, 45.0, 90.0};
    static const double rates[2][2] = {{30.0, 60.0}, {40.0, 120.0}};
    double worst[2] = {-9.0, -9.0};
    int resets[2] = {0, 0}, n[2] = {0, 0};
    for (int q = 0; q < 2; q++) {
      for (unsigned di = 0; di < sizeof(degs) / sizeof(degs[0]); di++) {
        for (int ph = 0; ph < 6; ph++) {
          camVzObsConfig_t cfg;
          camVzObsDefaultConfig(&cfg);
          simStart(&sim, &obs, &cfg, 10000, 0.0);
          sim.latS = 0.094;
          sim.frameS = 0.0675;
          sim.bobFrom = 1e9;
          sim.mod = fastTurnMod;
          fastTurnT0 = 8.0 + 0.0675 * ph / 6.0;
          fastTurnDeg = degs[di];
          fastTurnRate = rates[q][0];
          fastTurnAcc = rates[q][1];
          simRun(&sim, 7.9);
          camVzObsSetTurn(&obs, true);
          const uint32_t r0 = obs.cnt.yawResets;
          double tReset = -1.0;
          for (double t = 7.9; t < fastTurnEnd() + 1.0; t += 0.001) {
            simRun(&sim, t);
            if (tReset < 0.0 && obs.cnt.yawResets != r0) {
              tReset = t;
            }
          }
          camVzObsSetTurn(&obs, false);
          sim.mod = NULL;
          n[q]++;
          if (tReset >= 0.0) {
            resets[q]++;
            worst[q] = fmax(worst[q], tReset - fastTurnEnd());
          }
        }
      }
    }
    printf("fast moves: the tracker's restart after a commanded turn (frames 94 ms late, 67.5 ms apart): 30 deg/s %d of "
           "%d restarted, at most %+.3f s from the turn's end; 40 deg/s %d of %d, at most %+.3f s\n", resets[0], n[0],
           worst[0], resets[1], n[1], worst[1]);
    CHECK(resets[1] >= n[1] - 6 && worst[1] <= 0.300 && worst[0] <= 0.300, "a turn's tracker restart shows within "
          "300 ms of its end (the dwell from mvspeed 200 on): %.3f s at 40 deg/s, %.3f s at 30", worst[1], worst[0]);
  }
  /* (2) */
  {
    double errRule[2] = {0.0, 0.0}, tBackS[2] = {-1.0, -1.0};
    static const double accs[2] = {0.20, 0.40};
    for (int q = 0; q < 2; q++) {
      const double cycle = 4.0 * sqrt(0.04 / accs[q]);   /* two triangle legs of 4 cm */
      camVzObsConfig_t cfg;
      camVzObsDefaultConfig(&cfg);
      simStart(&sim, &obs, &cfg, 10000, 0.0);
      sim.bobA = 0.02;
      sim.bobF = 1.0 / cycle;
      sim.bobFrom = 1e9;
      sim.tSwitch = 9.0;
      sim.gTrue2 = 70.0;
      simRun(&sim, 8.0);
      for (double t = 8.0; t < 11.0; t += 0.01) {   /* testRetrackBob's 45 deg turn */
        simRun(&sim, t);
        for (int i = 0; i < sim.qN; i++) {
          camVzObsFrame_t *f = &sim.q[(sim.qHead + i) % 16];
          if (fabsf(f->gyroAngle[2]) < 0.005f) {
            f->gyroAngle[2] = (float)(0.2618 * sim.frameS);
          }
        }
      }
      simRun(&sim, 11.5);
      sim.bobFrom = 11.5;
      camVzObsOut_t out;
      double tBack = -1.0;
      for (double t = 11.5; t < 11.5 + 4.0 * cycle && tBack < 0.0; t += 0.01) {
        simRun(&sim, t);
        camVzObsGet(&obs, &out);
        if (out.gainTracked) {
          tBack = t - 11.5;
        }
      }
      tBackS[q] = tBack;
      if (tBack >= 0.0) {
        const double tRule = (ceil(tBack / cycle) + 1.0) * cycle;   /* the first cycle end a whole cycle on */
        simRun(&sim, 11.5 + tRule);
        camVzObsGet(&obs, &out);
        errRule[q] = fabs(out.gain / 70.0 - 1.0);
        printf("fast moves: the re-track bob at %.2f m/s^2 (cycle %.3f s): G back after %.2f s, %.1f px/m at the end "
               "rule (%.2f s; true 70: %.0f %%)\n", accs[q], cycle, tBack, out.gain, tRule, 100.0 * errRule[q]);
      }
    }
    CHECK(tBackS[1] >= 0.0 && tBackS[1] <= 4.0 * 4.0 * sqrt(0.04 / 0.40) &&
          errRule[1] <= fmax(0.10, errRule[0] + 0.05), "the re-track bob at 0.40 m/s^2: G back within its 4 cycles "
          "(%.2f s), within 10 %% at the end rule or no worse than 0.20's + 5 points (%.0f %%, 0.20: %.0f %%)", tBackS[1],
          100.0 * errRule[1], 100.0 * errRule[0]);
  }
  /* (3) */
  {
    const double cycle = 4.0 * sqrt(0.04 / 0.40);
    camVzObsConfig_t cfg;
    camVzObsDefaultConfig(&cfg);
    simStart(&sim, &obs, &cfg, 10000, 0.0);
    sim.bobA = 0.02;
    sim.bobF = 1.0 / cycle;
    sim.bobFrom = 1e9;
    sim.tSwitch = 9.0;
    sim.gTrue2 = 66.0;
    simRun(&sim, 11.5);
    sim.bobFrom = 11.5;
    simRun(&sim, 11.5 + 2.0 * cycle);
    camVzObsOut_t out;
    camVzObsGet(&obs, &out);
    printf("fast moves: the re-view bob at 0.40 m/s^2, 2 cycles (%.2f s): G %.1f px/m (66), healthy %d\n", 2.0 * cycle,
           out.gain, out.healthy);
    CHECK(out.gainTracked && out.healthy && fabs(out.gain / 66.0 - 1.0) < 0.15 && obs.cnt.yawResets == 0,
          "the re-view bob at 0.40 m/s^2: G %.1f within 15 %% of the new view's 66 after its 2 cycles", out.gain);
  }
  /* (4) */
  {
    double errMax[2] = {0.0, 0.0};
    static const double leanRates[2] = {6.0, 12.0};
    for (int q = 0; q < 2; q++) {
      camVzObsConfig_t cfg;
      camVzObsDefaultConfig(&cfg);
      simStart(&sim, &obs, &cfg, 10000, 0.0);
      sim.latS = 0.094;
      sim.frameS = 0.0675;
      sim.bobA = 0.0;
      sim.mod = fastLeanMod;
      fastLeanT0 = 8.0;
      fastLeanRate = leanRates[q];
      fastLeanSkewS = 0.005;
      simRun(&sim, 8.0);
      for (double t = 8.0; t < 20.0; t += 0.01) {
        simRun(&sim, t);
        camVzObsOut_t out;
        camVzObsGet(&obs, &out);
        errMax[q] = fmax(errMax[q], fabs(out.h - simZ(&sim, simNow(&sim))));
      }
      sim.mod = NULL;
    }
    printf("fast moves: the height estimate through the lean's pitch-rate steps (the gyro window 5 ms off): error max "
           "%.3f m at 6 deg/s, %.3f m at 12 deg/s\n", errMax[0], errMax[1]);
    CHECK(errMax[1] <= errMax[0] + 0.01, "the fast lean's 12 deg/s pitch-rate steps: the height error %.3f m, no worse "
          "than 6 deg/s's %.3f m + 1 cm", errMax[1], errMax[0]);
  }
}

int main(int argc, char **argv)
{
  const char *fixture = "tools/tests/fixtures/cam_vz_replay.txt";
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-v") == 0) {
      verbose = true;
    } else {
      fixture = argv[i];
    }
  }
  testReplay(fixture);
  testConvergence();
  testCameraLoss();
  testGates();
  testSkips();
  testGainRelearn();
  testRetrackBob();
  testViewShiftBob();
  testFastMoveObserver();
  testForwardGain();
  testCusum();
  testRelock();
  testGainReanchor();
  testReanchorGuards();
  testRates();
  testFaultsAndConfig();
#ifndef CVO_BASE
  testLateNoise();
  testHealthyBoost();
#endif
  testTiming();
  printf("%d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
