/*
 * ESP-Drone Firmware
 *
 * cam_vz_obs.h - Vertical speed, height and accelerometer offset from the
 * accelerometer and the forward camera's vertical image motion (autonomy
 * stage A2: the hover-wait after the photos)
 *
 * C port of the FINAL observer of the camera height study (observer.py,
 * with the parts of flowmodel.py it needs), replayed on the 13 logged camera
 * flights by tools/tests/test_cam_vz_obs.c. The study's verdict: a capped
 * wait of about 5 s at ~0.5 m is fine, 10-20 s is not (a slow camera speed
 * offset of ~0.03 m/s that no on-board sensor sees).
 *
 * States (3-state Kalman filter, float):
 *   h  height since the latest lift-off (m)
 *   v  vertical speed, up (m/s)
 *   b  world-up specific-force offset, measured minus true (m/s^2 inside, g
 *      outside). It is the offset against 1 g, not against a ground baseline:
 *      a caller that wants the offset against its own g0 subtracts g0 - 1.
 *
 * Prediction: every IMU sample (the world-up specific force in g, acc .
 * gravity, ~1.0 at rest) is integrated at the input rate with a zero-order
 * hold, and the filter is propagated in blocks of predictPeriodUs (10 ms, the
 * rate at which the study tuned the noise model). On the ground (the caller's
 * flag) h = v = 0 is enforced.
 *
 * Measurement: each camera frame's vertical image motion with the rotation
 * removed (hand-held calibration flowcal1.json, rounded as in the study's
 * flowmodel.KY_HANDHELD):
 *   dyRes = dy - (kyGx * gx + kyGy * gy + kyGz * gz + kyC)            (px)
 * is minus the mean vertical speed over the frame's motion window
 * [frameTime + gyroOffset - dt, frameTime + gyroOffset] times G * dt, where
 * G = f / D (px/m) depends on the distance D to the scene. The frame arrives
 * 100-230 ms after its window. A short ring of raw accelerometer integrals
 * expresses the window mean in terms of the current state (exact for a
 * zero-order-hold acceleration and a constant offset):
 *   mean v over [t0, t1] = v - Va(now) + (Za(t1) - Za(t0)) / dt + b (now - tMid)
 * so H = [0, 1, now - tMid] and the latency costs no accuracy.
 *   R = ((sigPx / dt)^2 + (sg * u)^2) / G^2 + sigCamV^2,  u = dyRes / dt
 * with sg = sigGrel for a tracked G and sigGrelDefault otherwise.
 *
 * Gain: the camera rate u and the accelerometer-only window speed w (raw
 * integral minus the fixed prior offset ramp, never camera-informed) pass the
 * same causal second-order high-pass (hpTauS) on the good-frame sequence; an
 * exponentially weighted regression (rlsTauS) gives G = -Sxy / Sxx, taken only
 * with enough excitation (rms >= minWRms), correlation (r <= rMax) and
 * G in gMin..gMax. Until the tracker has a G the default gDefault is used with
 * a wide relative error (zero image motion means zero speed whatever G is).
 * An accumulated yaw beyond yawResetRad means a new view (a new D): the
 * tracker restarts and its last G becomes a wide-error guess.
 *
 * Gain re-anchor (gainReanchorRel, 2026-10-13): the speed variance is
 * re-inflated by relockSigV once when the tracked G has moved more than
 * gainReanchorRel from the G it was last anchored at (the first tracked G
 * after a reset or a yaw restart), so a G that catches up with a scene whose
 * effective distance changed during the climb corrects the speed instead of
 * being held against an offset that absorbed the under-read speed. Only in
 * the take-off climb window (CAM_VZ_REANCHOR_*: until 2 s after the speed
 * estimate's climb ended, at most 6 s after the lift-off) and only on a frame
 * whose innovation the gate would take without the re-inflation: in the
 * hover a G moved by an object dragging the image, or collapsing under a
 * dead camera while the drone really climbs, would otherwise let the wrong
 * speed in and keep the CUSUM from seeing it.
 *
 * Gain through a forward move (fwdLn, 2026-10-15): G = f / D changes as the
 * drone flies toward the scene, but the tracker only re-measures it with
 * vertical motion, and a level move has none: the logged moves kept G within
 * 0.2 px/m while the drone closed in, so the speed read D0 / D_now too fast
 * (3.4-4.2 x at the end of the interactive hover's largest moves at k 1, 8-11
 * x at k 0.6) and the height loop's gain grew with it. Each frame carries the
 * interactive hover's measured forward progress ln(D0 / D) (the image's own
 * expansion, auto_motion.h autoMotionFwdLn; 0 without a move) and the update
 * uses G x exp(fwdLn - fwdLnAtG), fwdLnAtG the progress when the G in use was
 * taken (a tracked G, the yaw restart that made the guess, the reset): exact at
 * the expansion scale k 1, (D0 / D)^(1 - k) off otherwise (2.3-2.6 x at k 0.6
 * for the largest moves, against 8-11 x without; 0.6-0.7 x at k 1.6). A yaw
 * restart keeps the scaled G as the guess. The reported gain (out.gain, the
 * interactive hover's D = f / G) stays the tracker's: the motions keep their
 * own scene distances (auto_motion.h "Which D"). Nothing changes while fwdLn
 * stays where it was (every frame without a move: bit for bit as before).
 *
 * Late offset noise (qLateMult, R1 of the height study 2026-10-15, docs/
 * autonomy-a2b.md 25): in HOLD the controller holds this filter's speed at 0,
 * and the accelerometer's vertical offset turns after ~20-40 s and rises
 * 0.12-0.25 mg/s; at qBiasG the offset state lags it by 1-2.4 mg, a false climb
 * the drone answers with a real sink of 1-3 cm/s. From qLateS after the
 * sortie's FIRST lift-off (tLift restarts at every re-lift; the clock does not)
 * the offset's random walk is qBiasG x qLateMult (one step, x5 at most: x10
 * followed slow camera errors too far, docs/autonomy-a2b.md 25.8), never on
 * the ground. A guard keeps qBiasG, and holds the offset's variance at its
 * size at the step (p22Norm, the cross terms scaled with it) plus what qBiasG
 * alone adds since the previous frame (a gap or the ground regrows it as
 * today's filter would; only the multiplier's share is taken back), while any
 * of these is
 * this recent (CAM_VZ_QLATE_QUIET_US): a frame refused for what it shows (the
 * expansion, lateral or gain gate, the CUSUM hold, the innovation gate; not a
 * QUALITY or NO_WINDOW one), a CUSUM sum building up (CAM_VZ_QLATE_CS_REL), a
 * re-lock, a re-anchor, a yaw restart of the tracker, a G that is not tracked
 * (a guess or the default), a commanded turn or move (camVzObsSetTurn,
 * camVzObsSetMotion), no accepted update for CAM_VZ_QLATE_NOACC_US, the
 * offset moving faster than the accelerometer's drift can (CAM_VZ_QLATE_RATE_G
 * in 1-1.5 s: the camera moved it), the ground. The controller takes
 * biasHeldG, not biasG: past the step the mean of the offset 2.0-4.5 s ago
 * (CAM_VZ_QLATE_HOLD_US, CAM_VZ_QLATE_AVG_US), frozen while the guard holds
 * but for CAM_VZ_QLATE_FREEZE_US of one guard run at most (then the delayed
 * mean again, of an offset that moved at qBiasG alone: a long or repeated
 * motion, a drift that keeps the rate guard tripping, would otherwise fly on
 * an ever older offset), so a camera lost after a glitch the faster offset
 * swallowed leaves the controller the offset from before it. Between frames
 * the multiplier grows P[2][2], and through it P[1][1] and P[0][0]; the next
 * frame's guard bounds P[2][2] and its cross terms only (the frames back after
 * a gap weigh more on the speed: benign). Decided at each frame (flash); the
 * 10 ms block path only reads the multiplier's flag. qLateMult 1 (the
 * default) is today's filter, bit for bit; with it on, everything before
 * qLateS is too.
 *
 * Offset drift rate, "T2p" (t2kR, the late-sink fix of 2026-10-16, docs/
 * autonomy-a2b.md 29): the same late offset rise, tracked by a second state
 * instead of a faster random walk. From t2StartS after the sortie's FIRST
 * lift-off (R1's clock: a bounce does not restart it) the offset estimate
 * moves at bRate, x[2] += bRate d once per 10 ms block, but only while the
 * camera is accepting frames (an accepted update within CAM_VZ_T2_ACC_US)
 * and never on the ground. bRate starts at the cross-flight prior t2PriorMgS
 * (0.124 mg/s, the pooled late slope of the 10-14/15 flights) and, at each
 * ACCEPTED frame outside the guard hold, takes kR times the camera's own
 * correction of the offset: bRate += t2kR (x[2] after the update - before),
 * clamped to +-t2BoundMgS. The guard is R1's event set (a refused frame, a
 * CUSUM build-up or alarm, a re-lock, re-anchor or yaw restart, no tracked G,
 * a commanded turn or motion, no accepted update for CAM_VZ_QLATE_NOACC_US,
 * the offset moving faster than CAM_VZ_QLATE_RATE_G per second) held for
 * CAM_VZ_QLATE_QUIET_US; only the bookkeeping is shared (guardPre/guardPost):
 * R1's step, variance bound, held offset and freezing stay R1's, so with T2
 * on and qLateMult 1 biasHeldG == biasG on every block and P[2][2] is never
 * bounded. bRate is frozen while no frame is taken (the ground, a gap) and
 * kept through a bounce. R1 and T2 together are refused (camVzObsConfigValid:
 * an untested combination). It behaves like a PI offset tracker more than a
 * learned rate (bRate swings within its bound on the logged flights); its new
 * risk is a slow climb under a steady downward camera speed bias, so the
 * photo cadence of docs/autonomy-bench.md Q is part of it. t2kR 0 (the
 * default) is today's filter, bit for bit: the block path tests one flag.
 *
 * Healthy offset speed-up, "hb" (hbMult, the height estimator study of
 * 2026-10-18, docs/autonomy-a2b.md 37): the accelerometer's offset FALLS 4-8 mg
 * in the first 15-40 s of every logged flight (the die cooling) and rises
 * again after; at qBiasG the offset state lags it by 2-4.5 mg, a false sink
 * the HOLD answers with a real climb (+0.47 m mean over 16 flights, +1.13 m
 * on 2026-10-17 15:26). R1 starts too late for that and holds a delayed offset.
 * Here the random walk is qBiasG x hbMult on every block in which the
 * observer is HEALTHY: in the air, a tracked G, no CUSUM alarm and an update
 * accepted within healthyWindowUs (out.healthy and airborne), from the first
 * accepted frame after the lift-off; else qBiasG, today's filter (a camera
 * gap, an alarm, the ground, before the first frame). No held offset, no
 * variance bound, no guard: the controller takes biasG as today. The flag
 * the block path reads (hbOn: a tracked G, no alarm, an update ever) is set
 * at each frame (flash); the block adds the window test and the ground. The
 * replays (study section F): x3 halves the early climb (+0.47 -> +0.23 m
 * mean, +1.13 -> +0.66 on 15:26), takes a quarter off the worst low point
 * and half off the late sink (-2.5 -> -1.2 cm/s); its cost is the one every
 * faster offset has: a 5 s camera gap with no drift sinks 0.71 m instead of
 * 0.44 on the worst logged flight, a camera steadily wrong by -2 cm/s climbs
 * +1.55 instead of +1.44 m. hbMult 0 (the default) is today's filter, bit
 * for bit; with R1 or T2 on it is refused (camVzObsConfigValid: untested
 * together).
 *
 * Gates, in order: frames on the ground, before the camera wake (wakeUs after
 * throttle-up) or before lift-off are skipped; quality (ok, texture, residual,
 * yaw rate (maxYawRate; CAM_VZ_TURN_MAX_YAW_RATE in a commanded turn,
 * camVzObsSetTurn), no missed frame, age); expansion rate (forward motion, smoothed
 * |scale / dt|); lateral flow (|dxRes|, a person crossing the view); the
 * window must be inside the ring; a gain must exist; innovation CUSUM (a
 * moving object dragging the fit: updates refused for cusumHoldS, then the
 * speed variance is re-inflated); innovation gate gateK sigmas (after
 * relockS of continuous rejection the speed variance is re-inflated).
 *
 * Use (one instance, stabilizer task, all times esp_timer microseconds):
 *   camVzObsInit() once; at each sortie start, on the ground before
 *   throttle-up, camVzObsReset() and camVzObsSetBiasPrior(g0 - 1 +
 *   CAM_VZ_PRIOR_OFFSET_G, CAM_VZ_PRIOR_SIGMA_G); camVzObsImu() every
 *   stabilizer step; camVzObsFrame() for every new camera sample, on the
 *   ground too (the missed-frame gate needs every seq), right after that
 *   step's camVzObsImu(); camVzObsGet() for the outputs.
 *
 * Cost (ESP32-S3 240 MHz: single-precision FPU, software float division,
 * ~150 cycles per expf/sqrtf; estimated from operation counts and scaled
 * from the host timing printed by the test, 8 ns per sample and 66 ns per
 * frame on an Apple M core):
 *   camVzObsImu: ~15 flops per sample, plus one block close per 10 ms
 *     (3x3 covariance propagation ~110 flops, finiteness check, ring push,
 *     ~2 us) -> ~0.3 us per 1 kHz sample on average.
 *   camVzObsFrame (~17 Hz): 2 binary searches of the ring, 2 expf, 4 sqrtf,
 *     ~10 divisions, 3x3 Joseph update (~170 flops) -> <= ~10 us per frame.
 *   Together < 1 ms per second (< 0.1 % of one core). Memory:
 *   sizeof(camVzObs_t) = 2160 bytes on the host (ring of CAM_VZ_RING_LEN 16-byte
 *   entries; 2024 before the forward gain and the late offset noise), no heap;
 *   code ~7 KB. The late offset noise adds per frame ~40 flops and two loops
 *   over its 10-entry history (flash), per block one flag test (IRAM).
 *
 * Pure code: no ESP-IDF dependency.
 */
#ifndef CAM_VZ_OBS_H_
#define CAM_VZ_OBS_H_

#include <stdbool.h>
#include <stdint.h>

/* Placement of the code the drone runs for every 10 ms block (cam_vz_glue.c's
 * camvz task, 100 times a second). The ESP32-S3's 16 KB instruction cache is
 * shared by both cores and the 1 kHz stabilizer's own flash code nearly fills
 * it: run from flash, this path evicted ~100 of the stabilizer's lines per
 * block and cost ~3.5 % of its loops in flight (~962 instead of ~1000 Hz).
 * In IRAM it never touches the cache. Empty on the host. */
#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define CAM_VZ_HOT IRAM_ATTR
#else
#define CAM_VZ_HOT
#endif

#define CAM_VZ_G                9.80665f     /* m/s^2 per g (the study's constant) */
#define CAM_VZ_RING_LEN         96           /* raw-integral ring, one entry per block: 0.96 s at 10 ms */
#define CAM_VZ_MAX_RUN_US       1800000000u  /* 30 min after a reset the observer faults (time base limit) */
#define CAM_VZ_ZA_REBASE_M      1.0f         /* raw double integral re-based beyond this (float precision) */

/* Prior for the offset (camVzObsSetBiasPrior): the in-flight offset against
 * the take-off's ground baseline g0 AS THE STUDY DEFINES IT (median world-up
 * specific force over the whole pre-lift-off ground phase), mean of 28 graded
 * take-offs: +1.37 mg, sd 2.1 mg; the study used sigma 2.2 mg. vert_ctrl's
 * g0 (idle-spin mean) has its own offset (VZ_BAIR_DEFAULT_G, -2.3 mg): use
 * the pair that belongs together. Without a prior the filter starts at 0
 * with CAM_VZ_PRIOR_SIGMA_VAGUE_G. */
/* Gain re-anchor window (gainReanchorRel; docs/autonomy-a2.md 4.10). The
 * take-off climb is seen once the speed estimate exceeds CLIMB_V on a frame
 * and ends at the first frame after that below STOP_V (once per reset: a
 * later re-lift keeps it); re-anchors are allowed until AFTER_S past that
 * end and never later than MAX_S after the (latest) lift-off: vert_ctrl's
 * longest climb (climbMaxS at the take-off cap, VZ_CLIMB_MAX_LIMIT_S - 0.5 s:
 * 4.0 s at 1.20 m) plus AFTER_S, asserted in cam_vz_glue.c (the 2.00 m hover
 * of 2026-10-15 is reached by MOVEs up, not by the climb: 8.0 s for a
 * 2.00 m take-off, docs/autonomy-a2b.md 23.9). 2026-10-13
 * flights 1 and 2: the climb ended 3.72 and 3.56 s after the lift-off, their
 * re-anchors came 0.00-0.67 s later; in testGainReanchor the latest comes
 * 1.95 s after the end (with AFTER_S 1.5 one seed of 30 alarms; with 2.5 the
 * dead-camera replays of test_cam_vz_replay_ih.c hide 10 faults, not 2). */
#define CAM_VZ_REANCHOR_CLIMB_V 0.15f        /* m/s */
#define CAM_VZ_REANCHOR_STOP_V  0.05f        /* m/s */
#define CAM_VZ_REANCHOR_AFTER_S 2.0f         /* s */
#define CAM_VZ_REANCHOR_MAX_S   6.0f         /* s */
#define CAM_VZ_REANCHOR_TICK_US 50000u       /* the climb end is kept in these units (a uint16 in padding) */

/* |fwdLn - fwdLnAtG| is clamped to this (x 12.2 at most: 6 m to 0.5 m at k 1) */
#define CAM_VZ_FWD_LN_MAX       2.5f

/* Late offset noise (qLateMult, qLateS; header "Late offset noise"). The
 * study's replays used x10 (x3-5 gentler) and a step at 40 s, which replayed
 * like the 30 -> 45 s ramp. */
#define CAM_VZ_QLATE_MULT_MAX   5.0f         /* qLateMult 1 (off) .. this (x10 was cut: slow camera errors, a2b 25.8) */
#define CAM_VZ_QLATE_S_MIN      20.0f        /* qLateS, s after the first lift-off */
#define CAM_VZ_QLATE_S_MAX      120.0f
#define CAM_VZ_QLATE_QUIET_US   3000000u     /* a guard event this recent keeps qBiasG */
#define CAM_VZ_QLATE_CS_REL     0.5f         /* a CUSUM sum above this x cusumH is an event (the build-up) */
#define CAM_VZ_QLATE_NOACC_US   500000u      /* no accepted update this long is an event (frames refused or stopped) */
#define CAM_VZ_QLATE_HOLD_US    2000000u     /* biasHeldG is at least this old past the step ... */
#define CAM_VZ_QLATE_AVG_US     2500000u     /* ... the mean of the offsets CAM_VZ_QLATE_HOLD_US .. + this old */
#define CAM_VZ_QLATE_RATE_G     0.0010f      /* the offset moved more than this (g) ... */
#define CAM_VZ_QLATE_RATE_US    1000000u     /* ... since its history's newest entry at least this old: an event */
#define CAM_VZ_QLATE_FREEZE_US  4500000u     /* biasHeldG frozen this long in one guard run at most (then the delayed
                                              * mean again) */
#define CAM_VZ_QLATE_HIST       10           /* its history: one offset per CAM_VZ_QLATE_HIST_US (5 s) */
#define CAM_VZ_QLATE_HIST_US    500000u

/* Offset drift rate T2 (t2kR; header "Offset drift rate"). */
#define CAM_VZ_T2_KR_MAX        2.0f         /* t2kR 0 (off) .. this, 1/s (NVS cvt2k x 100) */
#define CAM_VZ_T2_S_MIN         20.0f        /* t2StartS, s after the first lift-off */
#define CAM_VZ_T2_S_MAX         120.0f
#define CAM_VZ_T2_BOUND_MAX_MGS 0.5f         /* t2BoundMgS in (0, this] */
#define CAM_VZ_T2_ACC_US        500000u      /* the drift applies only with an accepted update this recent */
#define CAM_VZ_T2_START_S       30.0f        /* t2StartS's default */
#define CAM_VZ_T2_PRIOR_MGS     0.124f       /* t2PriorMgS's default: the pooled late slope of the 10-14/15 flights */
#define CAM_VZ_T2_BOUND_MGS     0.35f        /* t2BoundMgS's default */

/* Healthy offset speed-up (hbMult; header "Healthy offset speed-up"). NVS cvhb = hbMult x 10 (0 off, 10..80). */
#define CAM_VZ_HB_MULT_MIN      1.0f         /* hbMult 0 (off) or this .. */
#define CAM_VZ_HB_MULT_MAX      8.0f         /* .. this (the study replayed x3 and x5; x3 is the step to fly first) */

/* camVzObsLateState() */
#define CAM_VZ_LATE_OFF         0            /* qLateMult 1, or before qLateS: today's filter */
#define CAM_VZ_LATE_ON          1            /* the multiplier applies */
#define CAM_VZ_LATE_GUARD       2            /* past qLateS, qBiasG kept by a guard event or the ground */

#define CAM_VZ_PRIOR_OFFSET_G   0.00137f
#define CAM_VZ_PRIOR_SIGMA_G    0.0022f
#define CAM_VZ_PRIOR_SIGMA_VAGUE_G 0.030f

/* What camVzObsFrame() did with a frame. Logged; only append. The letters
 * are the decision codes of the replay fixture (tools/tests/gen_cam_vz_fixture.py). */
typedef enum {
  CAM_VZ_F_DUPLICATE = 0,  /* same seq as the previous call: ignored entirely */
  CAM_VZ_F_SKIPPED = 1,    /* on the ground, before throttle-up + wakeUs, before lift-off, faulted */
  CAM_VZ_F_QUALITY = 2,    /* 'q': ok, texture, residual, dt, yaw rate, missed frame or age */
  CAM_VZ_F_EXPANSION = 3,  /* 'e': smoothed |scale / dt| > maxExpRate */
  CAM_VZ_F_LATERAL = 4,    /* 'x': |dxRes| > maxDxPx */
  CAM_VZ_F_NO_WINDOW = 5,  /* 'w': motion window not inside the ring */
  CAM_VZ_F_NO_GAIN = 6,    /* 'g': no tracked G, no guess and no default (or useCamera off) */
  CAM_VZ_F_CUSUM = 7,      /* 'c': refused while the innovation CUSUM alarm holds */
  CAM_VZ_F_GATE = 8,       /* 'r': innovation beyond gateK sigmas */
  CAM_VZ_F_ACCEPTED = 9,   /* 'A': Kalman update done */
} camVzObsDecision_t;

/* Tuning. camVzObsDefaultConfig() gives the study's FINAL configuration
 * (observer.py DEFAULT updated with FINAL). A field documented "<= 0 off"
 * disables that feature (the study's None). */
typedef struct {
  /* Kalman filter */
  float sigAcc;           /* 0.25 m/s^2: accelerometer white noise per 10 ms block (speed process noise) */
  float qBiasG;           /* 0.0003 g/sqrt(s): offset random walk */
  float sigPx;            /* 0.25 px/frame: dy noise after rotation removal */
  float sigGrel;          /* 0.20: relative error of a tracked G (speed-proportional camera noise) */
  float sigCamV;          /* 0.02 m/s: extra white camera speed noise (unmodelled contamination) */
  float gateK;            /* 3.5: innovation gate, sigmas */
  float relockS;          /* 1.5 s of continuous gate rejection before sigma_v is re-inflated */
  float relockSigV;       /* 0.10 m/s added to sigma_v at a re-lock (and when a CUSUM alarm ends) */
  /* gain tracker */
  float hpTauS;           /* 1.0 s: second-order high-pass of u and w */
  float rlsTauS;          /* 3.0 s: forgetting time of the regression sums */
  float gMin, gMax;       /* 15, 500 px/m */
  float rMax;             /* -0.5: needed correlation of hp(u) with hp(w) */
  float minWRms;          /* 0.03 m/s: needed rms of hp(w) (vertical excitation) */
  float minN;             /* legacy 12; VGA scales this evidence duration to its nominal cadence */
  float gapResetS;        /* 0.4 s between good frames restarts the high-passes */
  uint16_t minGainSamples; /* independent contributing samples, 0 in legacy profile */
  bool cadenceAware;      /* rates and evidence duration normalized to the camera cadence */
  uint16_t warmN;         /* 4: frames after a restart before the sums are fed */
  bool useCamera;         /* true; false = accelerometer and prior only (shadow comparisons) */
  /* frame gates */
  float minTexture;       /* 8 */
  float maxResidual;      /* 8 grey levels (the firmware hold uses 15; the study's flow section 4 uses 8) */
  float maxYawRate;       /* 0.35 rad/s: |gyroAngle z| < maxYawRate * dt */
  float dyOffsetPx;       /* 0: static per-frame dy offset removed before use */
  float maxExpRate;       /* 0.3 1/s, <= 0 off: smoothed |scale / dt| above this rejects */
  float expAlpha;         /* legacy 0.2/frame; cadenceAware uses its equivalent 0.29876 s time constant */
  float maxDxPx;          /* 1.5 px at 15 Hz; cadenceAware uses the equivalent 22.5 px/s gate, <= 0 off */
  float cusumK;           /* 1.0 sigma allowance, < 0 off */
  float cusumH;           /* 5.0: alarm threshold */
  float cusumHoldS;       /* 3.0 s: updates refused after an alarm */
  float yawResetRad;      /* 0.35 rad, <= 0 off: accumulated yaw that restarts the gain tracker */
  float gDefault;         /* 80 px/m, <= 0 off: G used until the tracker has one */
  float sigGrelDefault;   /* 0.6: relative gain error with the default or the guess */
  /* rotation model: dyRes = dy - (kyGx gx + kyGy gy + kyGz gz + kyC), dxRes = dx - (kxGx gx + kxGy gy + kxGz gz) */
  float kyGx, kyGy, kyGz, kyC;  /* 1.2836, 99.790, -2.7643 px/rad, 0.0294 px (flowcal1.json) */
  float kxGx, kxGy, kxGz;       /* -1.0, -4.9, 102.4 px/rad (camera_flow.c FLOW_KX_*) */
  /* timing */
  uint32_t gyroOffsetUs;  /* 15000: motion window end = frameTimeUs + this (camera_flow.c CAMERA_GYRO_OFFSET_US) */
  uint32_t wakeUs;        /* 1500000: frames whose window starts earlier after throttle-up are skipped */
  uint32_t predictPeriodUs;   /* 10000: filter block length, 5000..20000 */
  uint32_t maxFrameAgeUs;     /* 300000: an older frame fails the quality gate (the recorder's flowOk rule) */
  uint32_t healthyWindowUs;   /* 2000000: healthy needs an accepted update this recent */
  /* gain re-anchor (2026-10-13; appended: the fields above keep their offsets) */
  float gainReanchorRel;  /* 0.3, 0 off (plain), 0..1: a tracked G that moved more than this fraction from its
                           * anchor (the G of the anchor's frame) re-inflates sigma_v once (as a re-lock) and
                           * becomes the new anchor. sigGrel treats a tracked G as accurate to 20 % and white
                           * from frame to frame; when the scene's effective distance changes with height
                           * (2026-10-13 flight 2: floor/table parallax under a far window, G 92 -> 27 px/m in
                           * a 1 m climb) the speed it under-read went into the offset state and the
                           * corrected speed then alarmed the CUSUM at the top. Only in the take-off climb
                           * window (CAM_VZ_REANCHOR_*) and on a frame inside the gate (nu^2 <= gateK^2 S
                           * before the re-inflation). Never fired on the clean autonomy take-offs of
                           * 2026-10-12 (docs/autonomy-a2.md 4.10) */
  /* late offset noise (2026-10-15; appended: the fields above keep their offsets) */
  float qLateMult;        /* 1 (off: today's filter) .. CAM_VZ_QLATE_MULT_MAX: qBiasG's factor from qLateS on
                           * (header "Late offset noise"; NVS cvqlate, read by cam_vz_glue.c at boot) */
  float qLateS;           /* 40 s after the sortie's first lift-off, CAM_VZ_QLATE_S_MIN..MAX (NVS cvqlates) */
  /* offset drift rate T2 (2026-10-16; appended: the fields above keep their offsets) */
  float t2kR;             /* 0 (off: today's filter) .. CAM_VZ_T2_KR_MAX, 1/s: bRate's share of each accepted
                           * frame's offset correction (header "Offset drift rate"; NVS cvt2k / 100, read at boot) */
  float t2StartS;         /* 30 s after the sortie's first lift-off, CAM_VZ_T2_S_MIN..MAX */
  float t2PriorMgS;       /* 0.124 mg/s: bRate at t2StartS (NVS cvt2r, ug/s), |prior| <= t2BoundMgS */
  float t2BoundMgS;       /* 0.35 mg/s, (0, CAM_VZ_T2_BOUND_MAX_MGS]: |bRate| at most */
  /* healthy offset speed-up (2026-10-18; appended: the fields above keep their offsets) */
  float hbMult;           /* 0 (off: today's filter) or CAM_VZ_HB_MULT_MIN..MAX: qBiasG's factor on every block the
                           * observer is healthy in the air (header "Healthy offset speed-up"; NVS cvhb / 10, read at
                           * boot by cam_vz_glue.c); refused with qLateMult > 1 or t2kR > 0 */
} camVzObsConfig_t;

void camVzObsDefaultConfig(camVzObsConfig_t *c);
/* The FINAL extras off (no default gain, no CUSUM, expansion, lateral or yaw
 * gates): the study's "plain filter" (observer.py DEFAULT). */
void camVzObsPlainConfig(camVzObsConfig_t *c);
/* Every field finite and in a sane range. */
bool camVzObsConfigValid(const camVzObsConfig_t *c);

/* The fields of camera_flow.h's cameraFlowSample_t the observer uses (the
 * glue copies them; this module does not include camera_flow.h). */
typedef struct {
  uint32_t seq;           /* processed frame counter */
  uint64_t frameTimeUs;   /* capture (start of frame) time, esp_timer clock */
  uint32_t dtUs;          /* interval since the previous frame */
  float dx, dy;           /* px at 160x120, scene shift +right/+down in the buffer */
  float scale;            /* relative scale change */
  float rot;              /* rad (not used) */
  float gyroAngle[3];     /* body rotation over [frameTime - dt, frameTime] + gyroOffset, rad */
  float texture;
  float residual;
  bool ok;
  float fwdLn;            /* the interactive hover's measured forward progress ln(D0 / D) since ARM (auto_fsm.h
                           * autoFsmOut_t fwdLn; 0 without a move): the gain follows it (header, "Gain through a
                           * forward move"); appended: the fields above keep their offsets */
} camVzObsFrame_t;

typedef struct {
  uint32_t frames;        /* new frames given (duplicates excluded) */
  uint32_t skipped;       /* CAM_VZ_F_SKIPPED */
  uint32_t quality;
  uint32_t expRejects;
  uint32_t latRejects;
  uint32_t noWindow;
  uint32_t noGain;
  uint32_t accepted;      /* Kalman updates */
  uint32_t rejected;      /* innovation gate + CUSUM hold */
  uint32_t relocks;       /* gate re-locks */
  uint32_t alarms;        /* CUSUM alarms */
  uint32_t yawResets;
  uint32_t seqGaps;       /* frames with a missed (or restarted) seq before them */
  uint32_t imuIgnored;    /* samples with a time not after the previous one */
  uint32_t reanchors;     /* gain re-anchors (gainReanchorRel): sigma_v re-inflated on a tracked-G change */
} camVzObsCounters_t;

typedef struct {
  float h;                /* m since the latest lift-off (0 on the ground), at the latest IMU sample */
  float v;                /* m/s, up */
  float biasG;            /* g: offset of the world-up specific force against 1 g (measured minus true) */
  float sigmaH;           /* m */
  float sigmaV;           /* m/s */
  float sigmaBiasG;       /* g */
  float gain;             /* px/m the next update would use (tracked, guess or default); 0 if none */
  bool gainTracked;       /* the tracker has a valid G since its last (re)start */
  bool healthy;           /* not faulted, an update accepted within healthyWindowUs, gainTracked and no
                           * CUSUM alarm holding (the frames before an alarm may have pulled v; that
                           * error integrates into h until the camera is used again) */
  bool healthyGuess;      /* as healthy, but a G guessed (the tracker's last, kept at a yaw restart) counts as well
                           * as a tracked one: the interactive hover lets the height estimate use it through a
                           * commanded turn and its re-track bob (camVzGlueSetGuessAllowed), never otherwise */
  bool airborne;          /* lifted off and the latest sample not on the ground */
  bool fault;             /* non-finite input or state, or CAM_VZ_MAX_RUN_US exceeded: latched until reset */
  uint32_t sinceAcceptUs; /* since the last accepted update (UINT32_MAX if none) */
  float lastInnov;        /* m/s: innovation of the latest update attempt (accepted or rejected) */
  float lastInnovSigma;   /* m/s: its predicted sigma */
  float biasHeldG;        /* g: the offset the controller takes (camVzShadowVzInput): biasG, past qLateS with
                           * qLateMult > 1 the offset at least CAM_VZ_QLATE_HOLD_US old, frozen while the guard holds
                           * (header "Late offset noise") */
  bool hbOn;              /* the healthy offset speed-up applies to the current block (hbMult > 0, healthy, airborne;
                           * header "Healthy offset speed-up"; appended) */
} camVzObsOut_t;

/* Raw-integral ring entry (private). */
typedef struct {
  uint32_t tUs;           /* since the first sample after the reset */
  float va;               /* m/s: integral of (a - 1 g) */
  float za;               /* m: double integral, re-based (only differences are meaningful) */
  float a;                /* m/s^2 applied from tUs to the next entry */
} camVzObsRing_t;

/* Gain tracker (private). Times are window-centre sums t0 + t1 (us). */
typedef struct {
  bool hasT;
  uint32_t tSumLast;
  float uPrev, wPrev, u1, w1, u2, w2;
  uint32_t nSince;
  uint32_t samples;       /* real contributing observations; never a synthetic frame count */
  float sxx, sxy, syy, sw;
  bool hasG;
  float g;
  float r;
  bool validNow;
  bool hasFirst;
  uint32_t tSumFirst;     /* window-centre sum of the first valid G */
} camVzObsGain_t;

/* Observer state. Fields are private to cam_vz_obs.c except cnt (read-only
 * for the caller); callers allocate it (static) and pass it back. */
typedef struct {
  camVzObsConfig_t cfg;
  /* clock (times in us since the first sample after the reset) */
  bool started;
  uint64_t tStartUs;
  uint32_t tIn;           /* latest input sample */
  uint32_t tState;        /* filter state (latest block end) */
  bool throttled, lifted, onGround;
  bool lateOn;            /* the late offset noise's multiplier applies (set at each frame, never used on the ground;
                           * in padding: the 10 ms block path's offsets stay) */
  uint32_t tThrottle, tLift;
  /* raw integrals at the input rate and the open block */
  float aPrev;            /* m/s^2 of the latest sample (held until the next) */
  float va, za;
  float bdV, bdZ;         /* block-local single and double integral */
  uint32_t bN;            /* samples in the open block */
  camVzObsRing_t ring[CAM_VZ_RING_LEN];
  uint16_t ringHead, ringCount;
  /* filter */
  float x[3];             /* h (m), v (m/s), b (m/s^2) */
  float P[3][3];
  float b0;               /* m/s^2: fixed prior offset (accelerometer-only speed of the gain tracker) */
  /* gain */
  camVzObsGain_t gt;
  bool hasGuess;
  bool gAnchorSet;        /* gainReanchorRel: gAnchor holds the tracked G sigma_v was last (re)set for */
  bool raClimb;           /* re-anchor window: the take-off climb seen (v > CAM_VZ_REANCHOR_CLIMB_V) ... */
  bool raClimbEnd;        /* ... and ended (v < CAM_VZ_REANCHOR_STOP_V) at raClimbEnd50ms */
  float gGuess;
  /* frames */
  bool hasSeq;
  uint16_t raClimbEnd50ms;/* the climb end, CAM_VZ_REANCHOR_TICK_US since the start, rounded up (in padding: the
                           * fields the 10 ms block path reads, and the shadow's behind this state, keep their
                           * offsets) */
  uint32_t lastSeq;
  float expEma;
  float yawAcc;
  float csPos, csNeg;
  bool alarm;
  bool turn;              /* camVzObsSetTurn (in alarm's padding: every field keeps its offset) */
  bool moving;            /* camVzObsSetMotion (in padding too): a commanded motion runs, a late noise guard event */
  uint32_t alarmT;
  bool rejecting;
  uint32_t rejSince;
  bool hasAccept;
  uint32_t lastAccept;
  float lastNu, lastS;
  bool fault;
  bool late;              /* the late offset noise's step reached: biasHeldG is bHeld (in fault's padding) */
  camVzObsCounters_t cnt;
  float gAnchor;          /* after cnt (2026-10-13): the fields the 10 ms block path reads keep their offsets */
  float fwdLnAtG;         /* the frames' fwdLn when the G in use was taken (a tracked G, the yaw restart that made
                           * the guess; 0 at the reset): the update's G is x exp(fwdLn - this) (2026-10-15) */
  /* late offset noise (qLateMult, 2026-10-15; last: every field above keeps its offset). Untouched with
   * qLateMult 1 and t2kR 0; with t2kR > 0 the guard's bookkeeping (hasFirstLift, hasGuard, tGuard and the offset
   * history bHist*, kept from t2StartS on) runs too, the rest stays R1's. Decided at each frame (flash); the 10 ms
   * block path only reads lateOn. */
  bool hasFirstLift;      /* tFirstLift is set (at the first frame the filter took after the reset) */
  bool hasGuard;          /* tGuard holds a guard event */
  uint8_t bHistN, bHistHead;
  uint32_t tFirstLift;    /* us since the start: the sortie's first lift-off (tLift at its first used frame) */
  uint32_t tGuard;        /* us since the start: the latest guard event */
  uint32_t tLatePrev;     /* us since the start: the previous frame past the step (lateUs) */
  float p22Norm;          /* (m/s^2)^2: P[2][2] at the step, the bound the guard holds it under */
  float bHeldG;           /* g: out.biasHeldG past the step */
  float bHist[CAM_VZ_QLATE_HIST];      /* m/s^2: the offset every CAM_VZ_QLATE_HIST_US past the step ... */
  uint32_t bHistT[CAM_VZ_QLATE_HIST];  /* ... at these times (us since the start) */
  uint32_t lateUs;        /* with the multiplier on (between frames, a gap counted up to CAM_VZ_QLATE_NOACC_US) */
  uint32_t lateGuards;    /* times a guard event stopped it (on -> off at a frame) */
  bool freezing;          /* a guard run holds bHeldG since tFreeze */
  uint32_t tFreeze;
  /* offset drift rate T2 (t2kR, 2026-10-16; last: every field above keeps its offset). Untouched with t2kR 0. Set
   * and adapted at each frame (flash); the 10 ms block path reads t2Init and, with it, applies the drift. */
  bool t2Init;            /* t2StartS reached: bRate holds the rate (the prior at first) */
  uint32_t t2Us;          /* with the drift applied (blocks in the air with an accepted update this recent) */
  float bRate;            /* m/s^3: the offset's drift rate */
  float bRateMin, bRateMax;   /* m/s^3: its range since t2StartS */
  /* healthy offset speed-up (hbMult, 2026-10-18; last: every field above keeps its offset). Untouched with hbMult 0.
   * hbOn is set at each frame the filter takes (flash); the 10 ms block path reads it, adds the ground and the
   * accepted-update window, and counts hbUs. */
  bool hbOn;              /* hbMult > 0, a tracked G, no CUSUM alarm, an update accepted (ever), no fault */
  uint32_t hbUs;          /* with the multiplier applied (blocks in the air, healthy) */
} camVzObs_t;

/* The offset drift rate's state for reports (camVzObsT2Get, flash). */
typedef struct {
  bool on;                /* t2StartS reached in this sortie (t2kR > 0) */
  bool drifting;          /* the drift applies now (in the air, an accepted update within CAM_VZ_T2_ACC_US) */
  bool guard;             /* a guard event within CAM_VZ_QLATE_QUIET_US: the rate is held */
  float t2RateMgS;        /* bRate, mg/s (0 before t2StartS) */
  float minMgS, maxMgS;   /* its range since t2StartS, mg/s */
  uint32_t onUs;          /* with the drift applied */
} camVzObsT2_t;

/* Copies cfg (NULL: the default) and resets. An invalid cfg is refused: the
 * default is used and false is returned. */
bool camVzObsInit(camVzObs_t *o, const camVzObsConfig_t *cfg);
/* New sortie: clears everything but the config; offset prior 0 with
 * CAM_VZ_PRIOR_SIGMA_VAGUE_G until camVzObsSetBiasPrior(). */
void camVzObsReset(camVzObs_t *o);
/* Offset prior (g against 1 g, and its sigma). Only before lift-off (false
 * and ignored afterwards or with a non-finite or negative sigma). */
bool camVzObsSetBiasPrior(camVzObs_t *o, float b0G, float sigmaG);
/* A better (earlier) lift-off time than the first airborne sample, e.g.
 * back-dated by the lift-off detector: frames whose window starts before it
 * are skipped. Clamped to the samples seen; false before lift-off. */
bool camVzObsSetLiftoffTime(camVzObs_t *o, uint64_t timeUs);
/* One IMU sample: accUpG = world-up specific force in g; onGround = the
 * caller's ground flag, true before lift-off and after touchdown only (h = v
 * = 0 is enforced and frames are skipped while it is set, so a flag that
 * flickers in flight zeroes the height); the first airborne sample is the
 * lift-off time; thrust = the commanded thrust (the wake clock starts at the
 * first sample with thrust > 0). */
void camVzObsImu(camVzObs_t *o, uint64_t timeUs, float accUpG, bool onGround, float thrust);
/* One camera sample, processed at nowUs (for its age). */
camVzObsDecision_t camVzObsFrame(camVzObs_t *o, const camVzObsFrame_t *f, uint64_t nowUs);
/* A commanded turn of the interactive hover runs (its turn, dwell and bob:
 * the glue's camGuessOk; auto_motion.h): until it ends, the quality gate's
 * yaw test takes frames turning up to CAM_VZ_TURN_MAX_YAW_RATE instead of
 * maxYawRate (user decision 2026-10-13, evening: turns at 30 deg/s, whose
 * every frame the 0.35 rad/s gate refuses: no update for the turn, so the
 * healthy window (2 s) ran out and the drone landed for the lost camera
 * height, HOLD_NO_CAM in the interactive sim). The
 * rotation model removes the turn (flowcal1: |dxRes| <= 1.24 px up to
 * 45 deg/s); the yaw restart of the gain tracker is unchanged. Set before
 * each frame; false (the reset's) keeps every frame's decision as before.
 * Inline: the camvz task's IRAM block path sets it without a flash call. */
#define CAM_VZ_TURN_MAX_YAW_RATE 0.785f   /* rad/s, 45 deg/s (= auto_motion.h AUTO_MV_TURN_GATE_RPS) */
static inline void camVzObsSetTurn(camVzObs_t *o, bool turn)
{
  o->turn = turn;
}
/* A commanded motion of the interactive hover runs (a MOVE, a SET_HEIGHT's
 * correction or a YAW, from its start to the end of its settle: auto_motion.h
 * AUTO_MV_RUN, PAUSED, SETTLE): a guard event of the late offset noise
 * (header "Late offset noise"); nothing else reads it. Set before each block;
 * false (the reset's) without one. Inline: the camvz task's IRAM block path. */
static inline void camVzObsSetMotion(camVzObs_t *o, bool moving)
{
  o->moving = moving;
}
void camVzObsGet(const camVzObs_t *o, camVzObsOut_t *out);
/* CAM_VZ_LATE_* now (flash; diagnostics). */
uint8_t camVzObsLateState(const camVzObs_t *o);
/* The offset drift rate now (flash; diagnostics and the flight summary; all 0 with t2kR 0). */
void camVzObsT2Get(const camVzObs_t *o, camVzObsT2_t *t);
/* Window centre of the tracker's first valid G since its last (re)start, s
 * since the first sample after the reset; false if none. */
bool camVzObsFirstGainTime(const camVzObs_t *o, float *tS);
const char *camVzObsDecisionName(camVzObsDecision_t d);

#endif /* CAM_VZ_OBS_H_ */
