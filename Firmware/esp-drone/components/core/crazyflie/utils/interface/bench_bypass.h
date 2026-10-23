/*
 * ESP-Drone Firmware
 *
 * bench_bypass.h - RAM-only bisect toggles for the loop-rate bench (docs/autonomy-bench.md A4, step A4b)
 *
 * Each bit switches one per-loop addition of a2b section 19 fully off, back to
 * what 14bcfc6 ran there, so the A4 loop-rate measurement can price it on the
 * drone (docs/autonomy-a2b.md 19.11). Set over USB only ('bypass', usb_diag.c;
 * motors stopped, autonomy idle), never saved: every boot starts at 0, which is
 * the build as designed. Written by the USB task only; read by the per-loop
 * code as one load and a branch.
 *
 *   GTC   the gyro temperature compensation's whole path in the sensors task
 *         (the per-sample apply and accumulation, the 100 Hz update, the re-zero
 *         latch and log, the thrust bookkeeping), and the accelerometer
 *         compensation's with it (accel_tempco.h, a2b 36: the same block): the
 *         IMU is uncompensated as in 14bcfc6. Clearing it restarts both from an
 *         unseeded state: 0 until the next accepted ground re-zero (~1 s at
 *         rest); the accelerometer's reference until the next ARM or latch.
 *   MKI   the in-flight bias learning: no airborne / epoch inputs, the attitude
 *         filter's 14bcfc6 integral branch (twoKiAir untouched).
 *   REC   the recorder's section 19 fields (written 0: imuTemp, gtcCorr, mkiBias,
 *         flowGain, flowSat) and the flight summary's post-flight re-zero poll.
 *   FLOW  the flow fit forced to OF_FIT_PLAIN without touching NVS, and no
 *         exposure-step gate or gain / saturation bookkeeping (the camera loop of
 *         14bcfc6), from the next frame.
 */
#ifndef BENCH_BYPASS_H_
#define BENCH_BYPASS_H_

#include <stdbool.h>
#include <stdint.h>

#define BENCH_BYPASS_GTC  0x01u
#define BENCH_BYPASS_MKI  0x02u
#define BENCH_BYPASS_REC  0x04u
#define BENCH_BYPASS_FLOW 0x08u
#define BENCH_BYPASS_ALL  0x0Fu

/* bench_bypass.c; 0 at boot. */
extern volatile uint32_t benchBypassMask;

static inline bool benchBypassOn(uint32_t bits)
{
  return (benchBypassMask & bits) != 0u;
}

#endif /* BENCH_BYPASS_H_ */
