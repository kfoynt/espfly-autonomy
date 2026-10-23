/*
 * Host test of the flow camera's clock arithmetic, bookkeeping and fallback rule
 * (components/core/crazyflie/utils/src/cam_clock.c, docs/autonomy-a2b.md
 * section 17.2): the frame rate for the 160 MHz dividers 8 / 11 / 12 / 13
 * (22.21 / 16.15 / 14.81 / 13.67 fps), the gyro window offset (15 ms at
 * 20 MHz, 26 ms at 13.33 MHz), 16 MHz refused, the skip test, the
 * processing-time EMA; the fallback: at a processing average over 62 ms after
 * 100 frames at 13.33 MHz (counted across standby wakes), or a processed rate
 * under 14.0 fps over a full 150-frame window, on the ground; never with
 * autonomy active, off the ground, in a take-off window or asleep, at most once
 * per boot, never from 20 MHz; the published rate (1 / the interval EMA, two
 * fresh frames after a wake, none while too slow); and the whole timeline with
 * a model of esp32-camera's frame queue (fb_count 2, GRAB_LATEST: the 45 / 90 ms
 * split of the logs) and the ground standby (cam_gnd.c): a slow boot (P 88 ms)
 * falls back before the camera first sleeps, so the ARM after sleep -> PHOTO
 * flies at 20 MHz as before; a fallback whose switch fails, and an ARM before
 * the decision, are refused CAMERA (G4); a normal boot (P 61 ms) keeps 13.33 MHz
 * and sleeps 30 s after the start.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface tools/tests/test_cam_clock.c \
 *      components/core/crazyflie/utils/src/cam_clock.c components/core/crazyflie/utils/src/cam_gnd.c -lm \
 *      -o /tmp/test_cam_clock && /tmp/test_cam_clock
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "auto_fsm.h"   /* AUTO_FPS_MIN: G4 */
#include "cam_clock.h"
#include "cam_gnd.h"

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); \
                                                 printf("\n"); } } while (0)

static camClockIn_t slowGround(void)
{
  camClockIn_t in = {
    .xclkHz = CAM_CLOCK_SLOW_HZ,
    .fellBack = false,
    .clockFrames = CAM_CLOCK_DECIDE_FRAMES,
    .windowValid = true,
    .windowFps = 14.8f,
    .procAvgUs = 61000.0f,
    .awake = true,
    .autonomyActive = false,
    .onGround = true,
    .takeoffWindow = false,
  };
  return in;
}

static void testArithmetic(void)
{
  const struct { unsigned div; float fps; } t[] = {{8, 22.21f}, {11, 16.15f}, {12, 14.81f}, {13, 13.67f}};
  for (int i = 0; i < 4; i++) {
    const uint32_t hz = 160000000u / t[i].div;
    const float fps = camClockFps(hz);
    CHECK(fabsf(fps - t[i].fps) < 0.006f, "160 MHz / %u: %.3f fps (expected %.2f)", t[i].div, (double)fps,
          (double)t[i].fps);
  }
  CHECK(camClockXclkHz(0) == 20000000u && camClockXclkHz(1) == 13333333u && camClockXclkHz(7) == 13333333u,
        "camclk 0 = 20 MHz, otherwise 13.33 MHz");
  CHECK(camClockPeriodUs(20000000u) == 45023u && camClockPeriodUs(13333333u) == 67534u,
        "periods %u / %u us", (unsigned)camClockPeriodUs(20000000u), (unsigned)camClockPeriodUs(13333333u));
  CHECK(camClockReadoutUs(20000000u) == 44160u && camClockReadoutUs(13333333u) == 66240u,
        "readouts %u / %u us (768 of 783 lines)", (unsigned)camClockReadoutUs(20000000u),
        (unsigned)camClockReadoutUs(13333333u));
  CHECK(camClockGyroOffsetUs(20000000u) == 15000u, "offset at 20 MHz %u (the 2026-10-10 fit)",
        (unsigned)camClockGyroOffsetUs(20000000u));
  CHECK(camClockGyroOffsetUs(13333333u) == 26000u, "offset at 13.33 MHz %u (15 ms + half the longer readout)",
        (unsigned)camClockGyroOffsetUs(13333333u));
  CHECK(camClockXclkOk(20000000u) && camClockXclkOk(13333333u), "the two clocks are accepted");
  CHECK(!camClockXclkOk(CAM_CLOCK_PSRAM_MODE_HZ), "16 MHz refused (esp32-camera's PSRAM DMA mode)");
  CHECK(!camClockXclkOk(160000000u / 11u), "160 / 11 refused (61.9 ms: shorter than the processing)");
  CHECK(!camClockXclkOk(0) && camClockPeriodUs(0) == 0u, "0 Hz refused");
  /* skips: over 1.5 periods */
  CHECK(!camClockSkip(67534u, 13333333u) && !camClockSkip(101301u, 13333333u) && camClockSkip(101302u, 13333333u),
        "13.33 MHz: 67.5 and 101.3 ms are not skips, 101.302 ms is");
  CHECK(!camClockSkip(45000u, 20000000u) && camClockSkip(90000u, 20000000u),
        "20 MHz: the logged 45 ms is regular, 90 ms a skip");
  /* the processing EMA */
  float a = camClockProcAvg(0.0f, 61000u);
  CHECK(a == 61000.0f, "the first sample starts it");
  a = camClockProcAvg(a, 81000u);
  CHECK(fabsf(a - 62000.0f) < 0.01f, "alpha 0.05: 61000 -> %.1f after one 81000", (double)a);
}

static void testFallback(void)
{
  camClockIn_t in = slowGround();
  CHECK(!camClockFallback(&in), "a healthy loop: 61 ms, 14.8 fps");
  in.procAvgUs = 62000.0f;
  CHECK(!camClockFallback(&in), "exactly 62 ms: no");
  in.procAvgUs = 62001.0f;
  CHECK(camClockFallback(&in), "over 62 ms: fall back");
  in = slowGround();
  in.windowFps = 14.0f;
  CHECK(!camClockFallback(&in), "exactly 14.0 fps: no");
  in.windowFps = 13.99f;
  CHECK(camClockFallback(&in), "under 14.0 fps: fall back");

  /* the processing rule needs its frames, the rate rule a full window; each on its own */
  in = slowGround();
  in.procAvgUs = 88000.0f;
  in.windowValid = false;
  in.clockFrames = CAM_CLOCK_DECIDE_FRAMES - 1u;
  CHECK(!camClockFallback(&in), "88 ms but 99 frames and no window: not yet");
  in.clockFrames = CAM_CLOCK_DECIDE_FRAMES;
  CHECK(camClockFallback(&in), "88 ms after 100 frames, no window yet: fall back (the slow boot)");
  in = slowGround();
  in.clockFrames = 0;
  in.windowFps = 13.0f;
  CHECK(camClockFallback(&in), "a full window at 13.0 fps, whatever the frame count: fall back");
  in.windowValid = false;
  CHECK(!camClockFallback(&in), "... not without the window");

  /* a slow boot (P ~88 ms): blocked by every condition on its own */
  const char *what[] = {"no decision frames and no window", "already fell back", "at 20 MHz", "asleep",
                        "autonomy active", "off the ground", "in a take-off window"};
  for (int i = 0; i < 7; i++) {
    in = slowGround();
    in.procAvgUs = 88000.0f;
    in.windowFps = 11.3f;
    switch (i) {
      case 0: in.windowValid = false; in.clockFrames = CAM_CLOCK_DECIDE_FRAMES - 1u; break;
      case 1: in.fellBack = true; break;
      case 2: in.xclkHz = CAM_CLOCK_FAST_HZ; break;
      case 3: in.awake = false; break;
      case 4: in.autonomyActive = true; break;
      case 5: in.onGround = false; break;
      default: in.takeoffWindow = true; break;
    }
    CHECK(!camClockFallback(&in), "a slow loop, but %s: no fallback", what[i]);
  }
  in = slowGround();
  in.procAvgUs = 88000.0f;
  CHECK(camClockFallback(&in), "the slow loop on the ground: fall back");

  /* a window: the processed rate the camera task measures (150 frames of capture time) */
  const float fpsRegular = (float)CAM_CLOCK_WINDOW_FRAMES * 1e6f / (CAM_CLOCK_WINDOW_FRAMES * 67534.0f);
  CHECK(fabsf(fpsRegular - 14.81f) < 0.01f && fpsRegular >= CAM_CLOCK_FPS_MIN, "regular frames: %.2f fps",
        (double)fpsRegular);
  /* every 3rd frame skipped (P a little over the period): 2 frames per 3 periods */
  const float fpsSkips = 1e6f / (67534.0f * 1.5f);
  CHECK(fpsSkips < CAM_CLOCK_FPS_MIN, "a third of the frames skipped: %.2f fps, under the limit", (double)fpsSkips);
}

/* ---- the bookkeeping and the published rate ---------------------------------------------------------------- */
static void testTrack(void)
{
  camClockTrack_t t;
  memset(&t, 0, sizeof(t));
  camClockTrackInit(&t, CAM_CLOCK_SLOW_HZ);
  CHECK(camClockPending(&t) && !camClockTooSlow(&t), "a new camera at 13.33 MHz: pending");
  CHECK(camClockPublishFps(&t, 10, true) == 0.0f, "no interval yet: no rate");
  camClockTrack_t fast;
  memset(&fast, 0, sizeof(fast));
  camClockTrackInit(&fast, CAM_CLOCK_FAST_HZ);
  CHECK(!camClockPending(&fast), "20 MHz (camclk 0): nothing to decide");
  /* regular 67.5 ms frames, 61 ms of processing */
  uint64_t ft = 1000000u;
  camClockTrackFrame(&t, ft, 0, 61000u);
  for (uint32_t i = 1; i < CAM_CLOCK_DECIDE_FRAMES; i++) {
    ft += 67534u;
    camClockTrackFrame(&t, ft, 67534u, 61000u);
  }
  CHECK(t.clockFrames == CAM_CLOCK_DECIDE_FRAMES && !t.winValid && camClockPending(&t),
        "100 frames, no full window yet: still pending (%u frames)", (unsigned)t.clockFrames);
  CHECK(camClockPublishFps(&t, 50, true) == 0.0f, "pending: no rate published (a FLIGHT ARM waits for the decision)");
  camClockTrack_t tw = t;
  camClockTrackWake(&tw);
  CHECK(tw.clockFrames == t.clockFrames && tw.procAvgUs == t.procAvgUs && tw.dtAvgUs == t.dtAvgUs && tw.winFrames == 0,
        "a wake keeps the frame count, the processing and interval averages; the window restarts");
  for (uint32_t i = CAM_CLOCK_DECIDE_FRAMES; i <= CAM_CLOCK_WINDOW_FRAMES; i++) {
    ft += 67534u;
    camClockTrackFrame(&t, ft, 67534u, 61000u);
  }
  CHECK(t.winValid && fabsf(t.winFps - 14.81f) < 0.01f && !camClockPending(&t),
        "151 frames: a full window at %.2f fps, decided", (double)t.winFps);
  CHECK(fabsf(camClockPublishFps(&t, 5, true) - 14.81f) < 0.01f, "published %.2f fps",
        (double)camClockPublishFps(&t, 5, true));
  CHECK(camClockPublishFps(&t, 1, true) == 0.0f && camClockPublishFps(&t, 2, true) > 0.0f,
        "after a wake: a rate from the 2nd fresh frame (one interval) on");
  CHECK(camClockPublishFps(&t, 5, false) == 0.0f, "the thermal lead hides it");
  CHECK(t.skips == 0, "no skips in regular frames");
  camClockTrackFrame(&t, ft + 135068u, 135068u, 61000u);
  CHECK(t.skips == 1, "a 135 ms interval is a skip");

  /* the 67.5 / 135 ms alternation of a loop slower than the period (9.88 fps): the published rate is the
   * processed one; the old EMA of 1 / interval read 10.9-11.3 and passed G4 */
  camClockTrack_t a;
  memset(&a, 0, sizeof(a));
  camClockTrackInit(&a, CAM_CLOCK_FAST_HZ);   /* not too slow by rule: only the rate counts here */
  float oldEma = 0.0f, oldMin = 1e9f, oldMax = 0.0f, newMin = 1e9f, newMax = 0.0f;
  ft = 0;
  for (int i = 0; i < 400; i++) {
    const uint32_t dt = (i % 2) ? 135068u : 67534u;
    ft += dt;
    camClockTrackFrame(&a, ft, dt, 70000u);
    const float inst = 1e6f / (float)dt;
    oldEma = oldEma == 0.0f ? inst : 0.9f * oldEma + 0.1f * inst;
    if (i >= 100) {
      const float f = camClockPublishFps(&a, 10, true);
      newMin = fminf(newMin, f);
      newMax = fmaxf(newMax, f);
      oldMin = fminf(oldMin, oldEma);
      oldMax = fmaxf(oldMax, oldEma);
    }
  }
  CHECK(newMin > 9.6f && newMax < 10.1f, "67.5 / 135 ms: published %.2f..%.2f fps, the processed 9.88 within 2 %%",
        (double)newMin, (double)newMax);
  CHECK(oldMin > AUTO_FPS_MIN, "(the EMA of 1 / interval read %.2f..%.2f: over G4's 10 fps on every frame)",
        (double)oldMin, (double)oldMax);
  printf("info: 67.5 / 135 ms frames (9.88 fps processed): published %.2f..%.2f fps; the old EMA of the rates "
         "%.2f..%.2f\n", (double)newMin, (double)newMax, (double)oldMin, (double)oldMax);

  /* too slow at 13.33 MHz: nothing published; the fallback resets the measurements at the new clock */
  camClockTrack_t sl;
  memset(&sl, 0, sizeof(sl));
  camClockTrackInit(&sl, CAM_CLOCK_SLOW_HZ);
  ft = 0;
  for (uint32_t i = 0; i < CAM_CLOCK_DECIDE_FRAMES; i++) {
    ft += 101301u;
    camClockTrackFrame(&sl, ft, i ? 101301u : 0u, 88000u);
  }
  CHECK(camClockTooSlow(&sl) && camClockPublishFps(&sl, 50, true) == 0.0f, "88 ms at 13.33 MHz: too slow, no rate");
  camClockIn_t in = camClockTrackIn(&sl, true, false, true, false);
  CHECK(camClockFallback(&in), "... and the fallback is due on the ground");
  camClockTrack_t failed = sl;
  camClockTrackFellBack(&failed, CAM_CLOCK_SLOW_HZ);
  CHECK(failed.fellBack && !camClockPending(&failed) && camClockPublishFps(&failed, 50, true) == 0.0f,
        "a switch that failed: decided, still no rate at first");
  for (uint32_t i = 0; i < CAM_CLOCK_DECIDE_FRAMES; i++) {
    camClockTrackFrame(&failed, ft + i * 101301u, 101301u, 88000u);
  }
  in = camClockTrackIn(&failed, true, false, true, false);
  CHECK(camClockTooSlow(&failed) && camClockPublishFps(&failed, 50, true) == 0.0f && !camClockFallback(&in),
        "... and after 100 more frames: too slow, no rate, no second fallback");
  camClockTrackFellBack(&sl, CAM_CLOCK_FAST_HZ);
  CHECK(sl.fellBack && sl.clockFrames == 0 && sl.dtAvgUs == 0.0f && !camClockPending(&sl) && !camClockTooSlow(&sl),
        "switched to 20 MHz: measured anew, never pending, never too slow by rule");
  camClockTrack_t re = sl;
  camClockTrackInit(&re, CAM_CLOCK_FAST_HZ);
  CHECK(re.fellBack, "a camera start keeps the boot's fallback");
  re = t;
  camClockTrackRestart(&re);
  CHECK(!camClockPending(&re) && re.clockFrames == t.clockFrames && re.winValid && re.dtAvgUs == 0.0f &&
        re.winFrames == 0, "a camera re-initialisation keeps the decision, measures the rate anew");
}

/* ---- the timeline: driver queue, camera task, ground standby -------------------------------------------------
 * esp32-camera cam_hal.c with fb_count 2 and CAMERA_GRAB_LATEST (queue length 1), as
 * batch/verifier-safety/src/camsim.py: a frame is captured from one VSYNC and queued at the next (its timestamp
 * the first); a capture starts at a VSYNC only into a free buffer; a newer frame replaces a queued one. The camera
 * task takes the queued frame (or waits for one), processes it for P, returns it; between frames it runs the
 * ground standby decision, the fallback and the published rate as camera_flow.c does. Times in us. */
typedef struct {
  uint64_t period, tV;
  int st[2];          /* 0 free, 1 capturing, 2 queued, 3 held */
  uint64_t ts[2];
  int queued, capturing;
} drv_t;

static void drvStart(drv_t *d, uint64_t now, uint64_t period)
{
  memset(d, 0, sizeof(*d));
  d->period = period;
  d->tV = now + period;   /* the sensor's first VSYNC after a start or a wake */
  d->queued = d->capturing = -1;
}

static void drvVsync(drv_t *d)
{
  if (d->capturing >= 0) {
    if (d->queued >= 0) {
      d->st[d->queued] = 0;
    }
    d->queued = d->capturing;
    d->st[d->capturing] = 2;
    d->capturing = -1;
  }
  for (int i = 0; i < 2; i++) {
    if (d->st[i] == 0) {
      d->st[i] = 1;
      d->ts[i] = d->tV;
      d->capturing = i;
      break;
    }
  }
  d->tV += d->period;
}

/* VSYNCs strictly before t (a buffer returned at a VSYNC is free for it, as in camsim.py) */
static void drvUntil(drv_t *d, uint64_t t)
{
  while (d->tV < t) {
    drvVsync(d);
  }
}

/* frameGet(): the queued frame now, or the first one queued later; returns its timestamp, *now the take time */
static uint64_t drvTake(drv_t *d, uint64_t *now, int *held)
{
  drvUntil(d, *now);
  while (d->queued < 0) {
    *now = d->tV;
    drvVsync(d);
  }
  *held = d->queued;
  d->st[*held] = 3;
  d->queued = -1;
  return d->ts[*held];
}

typedef struct {
  uint32_t procUs;       /* the loop's processing time per frame */
  bool switchFails;      /* the fallback's set_xclk fails */
  uint32_t armAtMs;      /* the ARM (G4 is evaluated at it) */
  uint32_t photoAtMs;    /* a PHOTO in IDLE (0: none) */
} tlScen_t;

typedef struct {
  uint32_t fallbackMs, firstSleepMs, wakeMs;   /* 0: never */
  uint32_t xclkAtArm;
  float fpsAtArm;        /* the published rate at the ARM: G4 passes over AUTO_FPS_MIN */
  bool asleepAtArm, pendingAtSleep;
  float procAtFallback;
} tlRes_t;

static void timeline(const tlScen_t *sc, tlRes_t *r)
{
  memset(r, 0, sizeof(*r));
  uint32_t xclk = CAM_CLOCK_SLOW_HZ;
  camClockTrack_t trk;
  memset(&trk, 0, sizeof(trk));
  camClockTrackInit(&trk, xclk);
  camGnd_t g;
  const uint64_t t0 = 5000000u;   /* the camera starts 5 s after the boot */
  camGndInit(&g, (uint32_t)(t0 / 1000u));
  drv_t d;
  drvStart(&d, t0, camClockPeriodUs(xclk));
  uint64_t now = t0, prevT = 0, freshAfter = t0;
  uint32_t framesSinceWake = 0;
  float fps = 0.0f;
  bool photoDone = sc->photoAtMs == 0;
  const uint64_t armUs = t0 + (uint64_t)sc->armAtMs * 1000u;
  while (now < armUs) {
    /* the top of the loop: the ground standby (autonomy idle, on the ground, no take-off window) */
    camGndIn_t gin;
    memset(&gin, 0, sizeof(gin));
    gin.nowMs = (uint32_t)(now / 1000u);
    gin.enabled = gin.supported = gin.running = gin.onGround = true;
    gin.clockPending = camClockPending(&trk);
    gin.startMs = (uint32_t)(t0 / 1000u);
    if (camGndAwakeStep(&g, &gin) == CAM_GND_SLEEP) {
      camGndSleepResult(&g, gin.nowMs, true);
      if (r->firstSleepMs == 0) {
        r->firstSleepMs = (uint32_t)((now - t0) / 1000u);
        r->pendingAtSleep = gin.clockPending;
      }
      camClockTrackWake(&trk);   /* markWake() at the sleep */
      framesSinceWake = 0;
      fps = 0.0f;
      for (;;) {   /* asleep: 20 ms polls until a wake reason */
        now += CAM_GND_POLL_MS * 1000u;
        if (!photoDone && now >= t0 + (uint64_t)sc->photoAtMs * 1000u) {
          photoDone = true;
          camGndWake(&g, (uint32_t)(now / 1000u), CAM_GND_HOLD_MS_DEFAULT);   /* auto_link.c photoStart() */
        }
        if (now >= armUs) {
          camGndWake(&g, (uint32_t)(now / 1000u), CAM_GND_HOLD_MS_DEFAULT);   /* the ARM packet */
        }
        gin.nowMs = (uint32_t)(now / 1000u);
        if (camGndAsleepStep(&g, &gin) == CAM_GND_WAKE) {
          break;
        }
      }
      camGndWoke(&g, (uint32_t)(now / 1000u), false);
      if (r->wakeMs == 0) {
        r->wakeMs = (uint32_t)((now - t0) / 1000u);
      }
      camClockTrackWake(&trk);
      framesSinceWake = 0;
      prevT = 0;
      freshAfter = now;
      drvStart(&d, now, camClockPeriodUs(xclk));
      if (now >= armUs) {
        break;   /* the ARM came while asleep: G4 reads the rate now (0) and the Mac resends it */
      }
      continue;
    }
    int held;
    const uint64_t ft = drvTake(&d, &now, &held);
    if (ft < freshAfter) {
      d.st[held] = 0;   /* stale: returned at once */
      continue;
    }
    now += sc->procUs;
    drvUntil(&d, now);
    d.st[held] = 0;
    const uint32_t dt = prevT ? (uint32_t)(ft - prevT) : 0u;
    prevT = ft;
    framesSinceWake++;
    camClockTrackFrame(&trk, ft, dt, sc->procUs);
    fps = camClockPublishFps(&trk, framesSinceWake, true);
    if (!photoDone && now >= t0 + (uint64_t)sc->photoAtMs * 1000u) {
      photoDone = true;
      camGndWake(&g, (uint32_t)(now / 1000u), CAM_GND_HOLD_MS_DEFAULT);
    }
    if (xclk == CAM_CLOCK_SLOW_HZ && !trk.fellBack) {
      const camClockIn_t cin = camClockTrackIn(&trk, true, false, true, false);
      if (camClockFallback(&cin)) {
        r->fallbackMs = (uint32_t)((now - t0) / 1000u);
        r->procAtFallback = trk.procAvgUs;
        if (!sc->switchFails) {
          xclk = CAM_CLOCK_FAST_HZ;
        }
        camClockTrackFellBack(&trk, xclk);
        framesSinceWake = 0;
        fps = 0.0f;
        prevT = 0;
        freshAfter = now;
        drvStart(&d, now, camClockPeriodUs(xclk));
      }
    }
  }
  r->xclkAtArm = xclk;
  r->fpsAtArm = fps;
  r->asleepAtArm = g.asleep;
}

static void testTimeline(void)
{
  /* the logged slow boot: P ~88 ms (flowDt 90 ms on 314 of 325 frames at 20 MHz) */
  tlScen_t sc = {.procUs = 88000u, .switchFails = false, .photoAtMs = 60000u, .armAtMs = 80000u};
  tlRes_t r;
  timeline(&sc, &r);
  CHECK(r.fallbackMs > 0 && r.fallbackMs < 15000u, "slow boot: the fallback at %.1f s (proc %.0f us), within 15 s of "
        "the start", r.fallbackMs * 1e-3, (double)r.procAtFallback);
  CHECK(r.firstSleepMs >= CAM_GND_START_MS && r.firstSleepMs > r.fallbackMs && !r.pendingAtSleep,
        "... then the first ground sleep at %.1f s, decided", r.firstSleepMs * 1e-3);
  CHECK(r.wakeMs >= 60000u && r.wakeMs < 60100u, "the PHOTO at 60 s woke it at %.2f s", r.wakeMs * 1e-3);
  CHECK(r.xclkAtArm == CAM_CLOCK_FAST_HZ && r.fpsAtArm > AUTO_FPS_MIN && r.fpsAtArm > 11.0f && r.fpsAtArm < 11.8f,
        "the ARM 20 s later: 20 MHz, %.2f fps published (as today's slow boot: 11.3), G4 passes", (double)r.fpsAtArm);
  printf("info: slow boot (P 88 ms): fallback at %.1f s, first sleep %.1f s, PHOTO wake %.2f s, ARM at 80 s: %.2f MHz "
         "%.2f fps\n", r.fallbackMs * 1e-3, r.firstSleepMs * 1e-3, r.wakeMs * 1e-3, r.xclkAtArm * 1e-6,
         (double)r.fpsAtArm);

  /* the same boot, the switch fails: no rate at 13.33 MHz, G4 refuses CAMERA */
  sc.switchFails = true;
  timeline(&sc, &r);
  CHECK(r.fallbackMs > 0 && r.xclkAtArm == CAM_CLOCK_SLOW_HZ && r.fpsAtArm == 0.0f,
        "slow boot, the switch fails: still 13.33 MHz, %.2f fps published at the ARM (G4 refuses)", (double)r.fpsAtArm);

  /* an ARM before the decision (8 s after the camera's start, slow boot): no rate yet, G4 refuses CAMERA */
  sc.switchFails = false;
  sc.photoAtMs = 0;
  sc.armAtMs = 8000u;
  timeline(&sc, &r);
  CHECK(r.fallbackMs == 0 && r.xclkAtArm == CAM_CLOCK_SLOW_HZ && r.fpsAtArm == 0.0f,
        "an ARM 8 s into a slow boot, before the decision: %.2f fps published, G4 refuses", (double)r.fpsAtArm);
  sc.procUs = 61000u;
  timeline(&sc, &r);
  CHECK(r.fpsAtArm == 0.0f, "... also on a normal boot 8 s after the start (100 frames, no full window yet: %.2f)",
        (double)r.fpsAtArm);
  sc.armAtMs = 12000u;
  timeline(&sc, &r);
  CHECK(fabsf(r.fpsAtArm - 14.81f) < 0.05f, "a normal boot decided 12 s after the start: %.2f fps",
        (double)r.fpsAtArm);
  sc.procUs = 88000u;

  /* an ARM after a long idle without a PHOTO: the ARM wakes the camera; G4 refuses until two fresh frames */
  sc.armAtMs = 80000u;
  timeline(&sc, &r);
  CHECK(r.asleepAtArm == false && r.fpsAtArm == 0.0f && r.xclkAtArm == CAM_CLOCK_FAST_HZ,
        "an ARM while asleep: the camera wakes, %.2f fps at the ARM itself (the Mac resends it)", (double)r.fpsAtArm);

  /* a normal boot (P 61 ms): 13.33 MHz kept, decided within ~11 s, asleep from 30 s, the ARM after the PHOTO at
   * 14.8 fps */
  sc.procUs = 61000u;
  sc.photoAtMs = 60000u;
  sc.armAtMs = 80000u;
  timeline(&sc, &r);
  CHECK(r.fallbackMs == 0 && r.firstSleepMs >= CAM_GND_START_MS && r.firstSleepMs < CAM_GND_START_MS + 100u &&
        !r.pendingAtSleep, "normal boot: no fallback, asleep at %.2f s", r.firstSleepMs * 1e-3);
  CHECK(r.xclkAtArm == CAM_CLOCK_SLOW_HZ && fabsf(r.fpsAtArm - 14.81f) < 0.05f,
        "normal boot: the ARM after the PHOTO at 13.33 MHz, %.2f fps", (double)r.fpsAtArm);

  /* every P from 55 to 100 ms: either 13.33 MHz with >= 14.0 fps published, or the fallback before the first
   * sleep; never an ARM at 13.33 MHz under 14 fps that passes G4 */
  int bad = 0;
  for (uint32_t p = 55000u; p <= 100000u; p += 1000u) {
    tlScen_t s2 = {.procUs = p, .photoAtMs = 60000u, .armAtMs = 80000u};
    timeline(&s2, &r);
    const bool ok = r.xclkAtArm == CAM_CLOCK_SLOW_HZ
                        ? (r.fpsAtArm >= CAM_CLOCK_FPS_MIN && r.fallbackMs == 0)
                        : (r.fallbackMs > 0 && r.fallbackMs < r.firstSleepMs && r.fpsAtArm > AUTO_FPS_MIN);
    if (!ok) {
      bad++;
      printf("  P %u us: fallback %u ms, first sleep %u ms, %.2f MHz, %.2f fps at the ARM\n", (unsigned)p,
             (unsigned)r.fallbackMs, (unsigned)r.firstSleepMs, r.xclkAtArm * 1e-6, (double)r.fpsAtArm);
    }
  }
  CHECK(bad == 0, "P 55-100 ms: 13.33 MHz at >= 14 fps, or the fallback before the first sleep (%d bad)", bad);
}

int main(void)
{
  testArithmetic();
  testFallback();
  testTrack();
  testTimeline();
  printf("%d checks, %d failures\n", checks, failures);
  return failures != 0;
}
