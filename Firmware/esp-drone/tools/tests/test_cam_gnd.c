/*
 * Host test of the camera's ground standby decision (components/core/crazyflie/utils/src/cam_gnd.c,
 * docs/autonomy-a2b.md section 17.1): every sleep condition blocks the sleep on its own (the sensor clock
 * check still deciding, 17.2, included); each wake reason
 * wakes within one step; a ground start while asleep goes to the take-off standby (HANDOVER) when that takes
 * it, else WAKE; a failed standby write backs off 10 s and is counted; the thermal lead hides the frame rate
 * until it has passed (always visible with autonomy active, and at once with lead 0); repeated wakes extend
 * the hold; the clock wraps; the counters and the awake time for the flight summary.
 *
 * Build and run from Firmware/esp-drone:
 *   cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface tools/tests/test_cam_gnd.c \
 *      components/core/crazyflie/utils/src/cam_gnd.c -o /tmp/test_cam_gnd && /tmp/test_cam_gnd
 */
#include <stdio.h>
#include <string.h>

#include "cam_gnd.h"

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { failures++; printf("FAIL %d: ", __LINE__); printf(__VA_ARGS__); \
                                                 printf("\n"); } } while (0)

/* Every sleep condition true at nowMs (the camera started 60 s before). */
static camGndIn_t sleepy(uint32_t nowMs)
{
  camGndIn_t in;
  memset(&in, 0, sizeof(in));
  in.nowMs = nowMs;
  in.enabled = true;
  in.supported = true;
  in.running = true;
  in.onGround = true;
  in.startMs = nowMs - 60000u;
  return in;
}

static void testSleepConditions(void)
{
  const uint32_t t0 = 100000u;
  camGnd_t g;
  camGndInit(&g, t0 - 60000u);
  camGndIn_t in = sleepy(t0);
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_SLEEP, "every condition: SLEEP");

  const char *what[] = {"disabled", "unsupported", "not running", "autonomy active", "off the ground",
                        "take-off window", "snapshot pending", "bench hold", "the clock check not decided"};
  for (int i = 0; i < 9; i++) {
    camGndIn_t b = sleepy(t0);
    switch (i) {
      case 0: b.enabled = false; break;
      case 1: b.supported = false; break;
      case 2: b.running = false; break;
      case 3: b.autonomyActive = true; break;
      case 4: b.onGround = false; break;
      case 5: b.takeoffWindow = true; break;
      case 6: b.snapPending = true; break;
      case 7: b.benchHold = true; break;
      default: b.clockPending = true; break;
    }
    CHECK(camGndAwakeStep(&g, &b) == CAM_GND_STAY, "%s blocks the sleep", what[i]);
  }
  /* the hold not run out */
  camGndWake(&g, t0, CAM_GND_HOLD_MS_DEFAULT);
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_STAY, "a wake hold blocks the sleep");
  in.nowMs = t0 + CAM_GND_HOLD_MS_DEFAULT - 1u;
  in.startMs = in.nowMs - 60000u;
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_STAY, "... until its last ms");
  in.nowMs = t0 + CAM_GND_HOLD_MS_DEFAULT;
  in.startMs = in.nowMs - 60000u;
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_SLEEP, "... and then sleeps");
  /* less than 30 s since the camera's start (its clock check, 17.2) */
  in = sleepy(t0 + 50000u);
  in.startMs = in.nowMs - (CAM_GND_START_MS - 1u);
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_STAY, "29.999 s after the start: awake");
  in.startMs = in.nowMs - CAM_GND_START_MS;
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_SLEEP, "30 s after the start: sleeps");
}

static void testWakeReasons(void)
{
  const uint32_t t0 = 500000u;
  for (int i = 0; i < 6; i++) {
    camGnd_t g;
    camGndInit(&g, t0 - 60000u);
    camGndIn_t in = sleepy(t0);
    CHECK(camGndAwakeStep(&g, &in) == CAM_GND_SLEEP, "sleep due");
    camGndSleepResult(&g, t0, true);
    CHECK(g.asleep && g.count == 1, "asleep, counted");
    in.nowMs = t0 + CAM_GND_POLL_MS;
    CHECK(camGndAsleepStep(&g, &in) == CAM_GND_STAY, "no reason: stays asleep");
    CHECK(camGndAwakeStep(&g, &in) == CAM_GND_STAY, "asleep: no second sleep");
    const char *what = "";
    switch (i) {
      case 0: camGndWake(&g, in.nowMs, 45000u); what = "a wake request (PHOTO, ARM, TAKEOFF)"; break;
      case 1: in.autonomyActive = true; what = "autonomy active"; break;
      case 2: in.onGround = false; what = "off the ground"; break;
      case 3: in.enabled = false; what = "switched off"; break;
      case 4: in.benchHold = true; what = "a bench hold"; break;
      default: camGndWake(&g, in.nowMs, 1u); what = "a 1 ms wake request"; break;
    }
    CHECK(camGndAsleepStep(&g, &in) == CAM_GND_WAKE, "%s wakes it in one step", what);
    camGndWoke(&g, in.nowMs + 15u, false);
    CHECK(!g.asleep && g.asleepMs == CAM_GND_POLL_MS + 15u && g.wokeAtMs == in.nowMs + 15u,
          "woken: %u ms asleep counted", (unsigned)g.asleepMs);
  }
}

static void testGroundStart(void)
{
  const uint32_t t0 = 700000u;
  for (int camsb = 0; camsb < 2; camsb++) {
    camGnd_t g;
    camGndInit(&g, t0 - 60000u);
    camGndIn_t in = sleepy(t0);
    camGndSleepResult(&g, t0, true);
    in.nowMs = t0 + 5000u;
    in.groundStart = true;
    in.onGround = false;          /* the same stabilizer loop clears it */
    in.takeoffStandby = camsb != 0;
    const camGndAction_t a = camGndAsleepStep(&g, &in);
    CHECK(a == (camsb ? CAM_GND_HANDOVER : CAM_GND_WAKE), "ground start, camsb %s: %s", camsb ? "on" : "off",
          camsb ? "HANDOVER" : "WAKE");
    const uint32_t lead0 = g.wokeAtMs;
    camGndWoke(&g, in.nowMs, a == CAM_GND_HANDOVER);
    CHECK(!g.asleep && g.asleepMs == 5000u, "the ground standby's 5 s counted");
    CHECK(camsb ? g.wokeAtMs == lead0 : g.wokeAtMs == in.nowMs, "a hand-over leaves the lead clock alone");
  }
}

static void testFailedWrite(void)
{
  const uint32_t t0 = 900000u;
  camGnd_t g;
  camGndInit(&g, t0 - 60000u);
  camGndIn_t in = sleepy(t0);
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_SLEEP, "sleep due");
  camGndSleepResult(&g, t0, false);
  CHECK(!g.asleep && g.fails == 1 && g.count == 0, "failed write: awake, counted as a failure");
  for (uint32_t dt = 1; dt < CAM_GND_RETRY_MS; dt += 997u) {
    in = sleepy(t0 + dt);
    CHECK(camGndAwakeStep(&g, &in) == CAM_GND_STAY, "backing off at +%u ms", (unsigned)dt);
  }
  in = sleepy(t0 + CAM_GND_RETRY_MS);
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_SLEEP, "retried after 10 s");
  camGndWakeFailed(&g);
  CHECK(g.fails == 2, "a failed wake write counted");
}

static void testLead(void)
{
  const uint32_t t0 = 1200000u;
  camGnd_t g;
  camGndInit(&g, t0 - 60000u);
  CHECK(camGndFpsVisible(&g, t0, 0u, false), "lead 0: visible");
  camGndSleepResult(&g, t0, true);
  CHECK(camGndFpsVisible(&g, t0 + 100u, 0u, false), "lead 0: visible (the rate itself is 0 asleep)");
  CHECK(!camGndFpsVisible(&g, t0 + 100u, 5000u, false), "lead 5 s, asleep: hidden");
  camGndWoke(&g, t0 + 1000u, false);
  CHECK(!camGndFpsVisible(&g, t0 + 1000u + 4999u, 5000u, false), "4.999 s after the wake: hidden");
  CHECK(camGndFpsVisible(&g, t0 + 1000u + 5000u, 5000u, false), "5 s after the wake: visible");
  CHECK(camGndFpsVisible(&g, t0 + 1000u + 10u, 5000u, true), "autonomy active: always visible");
  /* from the camera's start too */
  camGnd_t h;
  camGndInit(&h, t0);
  CHECK(!camGndFpsVisible(&h, t0 + 2000u, 3000u, false) && camGndFpsVisible(&h, t0 + 3000u, 3000u, false),
        "the lead also runs from the camera's start");
}

static void testHoldExtension(void)
{
  const uint32_t t0 = 1500000u;
  camGnd_t g;
  camGndInit(&g, t0);
  camGndWake(&g, t0, 45000u);
  camGndWake(&g, t0 + 30000u, 45000u);   /* a PHOTO, then an ARM 30 s later */
  CHECK(camGndHoldActive(&g, t0 + 74999u) && !camGndHoldActive(&g, t0 + 75000u),
        "the second wake extends the hold to 75 s");
  camGndWake(&g, t0 + 40000u, 5000u);    /* a shorter hold keeps the longer one */
  CHECK(camGndHoldActive(&g, t0 + 74999u), "a shorter wake keeps the longer hold");
  camGndWake(&g, t0 + 40000u, 3600000u * 2u);
  CHECK(camGndHoldActive(&g, t0 + 40000u + 3600000u - 1u) && !camGndHoldActive(&g, t0 + 40000u + 3600000u),
        "a hold is at most 1 h");
  camGndRelease(&g, t0 + 50000u);
  CHECK(!camGndHoldActive(&g, t0 + 50000u), "released at once");
  /* wrap of the 32-bit ms clock */
  const uint32_t w = 0xFFFFF000u;
  camGndInit(&g, w);
  camGndWake(&g, w, 45000u);
  CHECK(camGndHoldActive(&g, w + 10000u) && camGndHoldActive(&g, w + 44999u) && !camGndHoldActive(&g, w + 45000u),
        "the hold across the clock wrap");
  camGndIn_t in = sleepy(w + 45000u);
  CHECK(camGndAwakeStep(&g, &in) == CAM_GND_SLEEP, "sleeps after it, across the wrap");
  camGndInit(&g, 5u);
  in = sleepy(0x90000000u);   /* ~28 days later, never woken: no stale hold */
  CHECK(!camGndHoldActive(&g, in.nowMs) && camGndAwakeStep(&g, &in) == CAM_GND_SLEEP, "no stale hold after 28 days");
}

static void testStats(void)
{
  const uint32_t t0 = 2000000u;
  camGnd_t g;
  camGndInit(&g, t0);
  CHECK(camGndAwakeMs(&g, t0 + 5000u) == UINT32_MAX, "never slept: UINT32_MAX (flight summary 0xFFFF)");
  camGndSleepResult(&g, t0 + 40000u, true);
  CHECK(camGndAwakeMs(&g, t0 + 41000u) == 0u, "asleep: 0");
  CHECK(camGndAsleepTotalMs(&g, t0 + 41000u) == 1000u, "a sleep in progress counts");
  camGndWoke(&g, t0 + 50000u, false);
  CHECK(camGndAwakeMs(&g, t0 + 72000u) == 22000u, "22 s after the wake");
  camGndSleepResult(&g, t0 + 100000u, true);
  camGndWoke(&g, t0 + 160000u, false);
  CHECK(g.count == 2 && camGndAsleepTotalMs(&g, t0 + 200000u) == 70000u, "two sleeps, 70 s in all (%u)",
        (unsigned)camGndAsleepTotalMs(&g, t0 + 200000u));
  camGndSleepResult(&g, t0 + 300000u, true);
  camGndRestarted(&g, t0 + 301000u);   /* a camera restart ends a sleep too */
  CHECK(!g.asleep && camGndAsleepTotalMs(&g, t0 + 400000u) == 71000u && g.wokeAtMs == t0 + 301000u,
        "a restart counts the sleep and restarts the lead clock");
}

int main(void)
{
  testSleepConditions();
  testWakeReasons();
  testGroundStart();
  testFailedWrite();
  testLead();
  testHoldExtension();
  testStats();
  printf("%d checks, %d failures\n", checks, failures);
  return failures != 0;
}
