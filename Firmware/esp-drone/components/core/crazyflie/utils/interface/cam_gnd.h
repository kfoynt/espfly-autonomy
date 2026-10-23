/*
 * ESP-Drone Firmware
 *
 * cam_gnd.h - When the flow camera sleeps on the ground (docs/autonomy-a2b.md
 * section 17.1)
 *
 * On battery the board draws ~0.16-0.25 A while the drone waits on the ground
 * (setup, the IMU heat-soak wait, between flights); the camera and its flow
 * work are an estimated 40-110 mA of it. camera_flow.c puts the sensor in its
 * software standby (the take-off standby's register write) while autonomy is
 * idle on the ground, and wakes it for a photo, an ARM or TAKEOFF, any
 * autonomy state other than IDLE, a ground start, or a bench hold. This file
 * is the decision only, pure and host-tested (tools/tests/test_cam_gnd.c); the
 * camera task feeds it and does the writes.
 *
 * Sleep (camGndAwakeStep): enabled (NVS camgnd), a sensor with a standby,
 * streaming, autonomy idle, resting on the ground, outside the take-off
 * window, no snapshot pending, no bench hold, the wake hold expired, the
 * camera started at least CAM_GND_START_MS ago, its clock check decided
 * (cam_clock.h camClockPending(), section 17.2: it needs contiguous awake
 * frames, ~10-15 s), and not backing off after a failed standby write.
 * Wake (camGndAsleepStep): the wake hold, autonomy active, off the ground,
 * disabled, a bench hold; a ground start is handed over to the take-off
 * standby (HANDOVER: the sensor stays asleep and the take-off standby wakes it
 * at the end of its window, as it always did) when that standby takes it
 * (camsb on and its phase open), else WAKE.
 *
 * Times are a 32-bit ms clock; every comparison is wrap-safe.
 */
#ifndef CAM_GND_H_
#define CAM_GND_H_

#include <stdbool.h>
#include <stdint.h>

#define CAM_GND_START_MS        30000u   /* awake this long after the camera (re)starts */
#define CAM_GND_RETRY_MS        10000u   /* no new standby write this long after a failed one */
#define CAM_GND_HOLD_MS_DEFAULT 45000u   /* NVS camgndms: awake this long after a wake reason (PHOTO, ARM, TAKEOFF);
                                          * the logged planning-PHOTO -> ARM gaps were 15-39 s */
#define CAM_GND_HOLD_MS_MIN     5000u
#define CAM_GND_HOLD_MS_MAX     65000u
#define CAM_GND_LEAD_MS_MAX     60000u   /* NVS camlead: the thermal lead (default 0, off) */
#define CAM_GND_WAKE_MAX_MS     3600000u /* the longest hold camGndWake() takes (bench: 'tkoff camwake <s>') */
#define CAM_GND_POLL_MS         20u      /* asleep: the wake reasons are tested this often */
#define CAM_GND_HOLD_UPDATE_MS  100u     /* asleep: the drift hold's clocks are kept this often */

typedef enum {
  CAM_GND_STAY = 0,   /* nothing to do */
  CAM_GND_SLEEP,      /* awake: put the sensor in standby now */
  CAM_GND_WAKE,       /* asleep: wake the sensor now */
  CAM_GND_HANDOVER,   /* asleep: a ground start the take-off standby takes over; keep the sensor asleep */
} camGndAction_t;

typedef struct {
  uint32_t nowMs;
  bool enabled;          /* NVS camgnd */
  bool supported;        /* the sensor has a software standby (OV3660, OV5640) */
  bool running;          /* the camera streams (CAMERA_FLOW_RUNNING) */
  bool autonomyActive;   /* autonomy out of IDLE */
  bool onGround;         /* stabilizerOnGround(): zero thrust, resting since the last ground stop */
  bool takeoffWindow;    /* the camera's take-off window is open */
  bool snapPending;      /* a frame snapshot is requested or being copied */
  bool benchHold;        /* bench: 'stream N > 0', a 'camvz air' mode */
  bool clockPending;     /* the sensor clock check has not decided yet (cam_clock.h camClockPending()) */
  bool groundStart;      /* asleep: the ground-start count moved since the sleep began */
  bool takeoffStandby;   /* asleep: the take-off standby takes that ground start (camsb on, its phase open) */
  uint32_t startMs;      /* when the camera last started or restarted */
} camGndIn_t;

typedef struct {
  uint32_t holdUntilMs;  /* the wake hold: no sleep before this (camGndWake) */
  uint32_t retryAtMs;    /* no standby write before this (after a failed one) */
  uint32_t sleptAtMs;    /* the current sleep's start */
  uint32_t wokeAtMs;     /* the camera start or the latest ground wake (the thermal lead's clock) */
  bool asleep;           /* in the ground standby */
  bool slept;            /* has slept on the ground this boot */
  uint32_t count;        /* ground sleeps */
  uint32_t fails;        /* failed standby writes (sleep and wake) */
  uint32_t asleepMs;     /* total time asleep on the ground, completed sleeps */
} camGnd_t;

/* At the camera's start: awake, no hold, no back-off. */
void camGndInit(camGnd_t *g, uint32_t nowMs);
/* Restart of the camera (re-initialised after a stall): awake, counters kept. */
void camGndRestarted(camGnd_t *g, uint32_t nowMs);
/* A wake reason: awake (or woken within CAM_GND_POLL_MS) and held awake for
 * at least holdMs from now (an earlier, longer hold is kept). holdMs above
 * CAM_GND_WAKE_MAX_MS is clamped to it. */
void camGndWake(camGnd_t *g, uint32_t nowMs, uint32_t holdMs);
/* Ends the wake hold now (bench: 'tkoff camwake 0'). */
void camGndRelease(camGnd_t *g, uint32_t nowMs);
/* The wake hold is still running. */
bool camGndHoldActive(const camGnd_t *g, uint32_t nowMs);
/* Awake: CAM_GND_SLEEP when every sleep condition holds, else CAM_GND_STAY. */
camGndAction_t camGndAwakeStep(const camGnd_t *g, const camGndIn_t *in);
/* The standby write's outcome: asleep and counted, or a failure counted and
 * CAM_GND_RETRY_MS of back-off. */
void camGndSleepResult(camGnd_t *g, uint32_t nowMs, bool ok);
/* Asleep: CAM_GND_HANDOVER, CAM_GND_WAKE or CAM_GND_STAY. */
camGndAction_t camGndAsleepStep(const camGnd_t *g, const camGndIn_t *in);
/* The ground standby ended (woken, or handed over to the take-off standby):
 * its time is counted; a wake (handover false) also restarts the lead clock. */
void camGndWoke(camGnd_t *g, uint32_t nowMs, bool handover);
/* A failed wake write (the camera task retries; the stall path re-inits). */
void camGndWakeFailed(camGnd_t *g);
/* The thermal lead: the frame rate is published (a FLIGHT ARM's G4 can pass)
 * once leadMs have passed since the camera start or the latest ground wake,
 * always while autonomy is active, and at once with leadMs 0. */
bool camGndFpsVisible(const camGnd_t *g, uint32_t nowMs, uint32_t leadMs, bool autonomyActive);
/* ms since the latest ground wake, 0 while asleep, UINT32_MAX if it never
 * slept on the ground this boot (the flight summary's camAwakeDs). */
uint32_t camGndAwakeMs(const camGnd_t *g, uint32_t nowMs);
/* Total ground sleep, ms, including a sleep in progress. */
uint32_t camGndAsleepTotalMs(const camGnd_t *g, uint32_t nowMs);

#endif /* CAM_GND_H_ */
