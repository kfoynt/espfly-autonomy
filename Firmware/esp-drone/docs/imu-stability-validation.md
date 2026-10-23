# ESP-FLY IMU stability: validation status

Date: 2026-10-10 · Branch `imu-stability`

## Handoff summary

```text
User observations and verified configuration:
  Same-side sideways drift with centred sticks; hops ended in collisions.
  Drone runs ESP_FLY_MANUAL_ONLY_V1 (verified byte-identical from a flash
  read-back), ESP-Drone app, joysticks, app trim 0.
Selected outcome: targeted firmware change, configured from measurements.
Original and revised settings:
  level offset  0, 0 g             -> +0.00166, +0.02266 g (NVS "imucal/level")
  torque trim   none               -> roll +3853, pitch -2598 (NVS "imucal/ttrim")
  integrators   reset only at 0    -> held at 0 below 20000 thrust
  app trim      0                  -> 0 (keep it at 0)
Implemented: level calibration, alignment fix, ground integrator hold,
  torque trim, USB diagnostics console, 60 s RAM flight recorder, host tool.
Verified on host/build: see "Host and build".
Verified on the actual drone: see "On the drone".
Pending observations: see "Pending".
Branches skipped or left disabled: per-axis six-face accelerometer fit,
  estimator/filter changes, PID gain changes, gyro re-calibration while
  landed, Kconfig alignment (fixed but left at 0).
Known limitations: varying mechanical imbalance; yaw rotation; thin thrust
  margin; battery sag/brownout; no horizontal reference (IMU only).
Rollback: see docs/imu-calibration.md "Rollback".
```

## Host and build

| Check | Result |
|---|---|
| Rebuild of the imported running source with ESP-IDF v5.0.7 | identical `.text`/`.iram`/`.data` to the drone's build |
| Firmware build (`idf.py build`, tracked `sdkconfig` unchanged) | passes; no new warnings in changed files |
| `tools/tests/test_imu_level_cal.c` | 374 checks, 0 failures |
| `tools/tests/test_espfly_diag.py` | all tests pass |

The C tests cover:
- CRC check value; record round trip; corrupted, wrong-version, wrong-size,
  out-of-range, NaN and Inf records rejected
- level and trim records cannot be confused with each other
- alignment: identity at zero, orthonormal, determinant +1, transpose
  inverse, norm preservation, Kconfig sign convention, and a regression case
  for the old non-rotation
- trim ramp: bounds and monotonicity

The Python tests cover:
- two-position offset math, including surface-tilt cancellation
- estimator angle conventions
- record checksum parsing
- recovering roll/pitch outputs by inverting the X mixer, with stick input
  and saturation excluded
- analyzer smoke test

## On the drone

Bench (USB, motors held at zero by the inhibit):
- The console answers. `measure` returns 1 kHz sample counts
  (3003–5007 per window).
- Motor outputs read zero throughout every measurement.
- Two-position level calibration was applied and saved. It reloaded after a
  reboot (`boot=ESP_OK`), and the drone reads the table tilt only.
- The torque trim was saved and reloaded after a reboot.
- Axis signs and motor positions were confirmed: pen under one side gives
  +21.5° roll; nose raised gives +19.4° pitch.
- The reset-reason report identified a brownout during the failed take-offs
  on a half-charged battery.

Flight (21 recorded throttle segments over 3 rounds; details in
`imu-stability-diagnosis.md` §4.2). All take-off speeds below are estimates
from integrated attitude, not position measurements.

| | peak take-off tilt (median) | estimated take-off slide (median) | integrators in the first 0.5 s |
|---|---|---|---|
| level calibration only | 2.8° | 0.35 m/s | −1300..−1900 (up to ±14000 on slow ramps) |
| + ground integrator hold | 2.9° | 0.29 m/s | −89..+208 |
| + torque trim (2374 / −4435) | 1.8° | 0.11 m/s | ≈0 |
| + re-learned trim (3853 / −2598) | 2.7°* | **0.09 m/s** (max 0.10) | ≈0 |

\* Two take-offs had a brief 3.7–4.9° roll flick right at lift-off,
probably a leg catching on a brisk launch. It produced no extra slide.
Without them, the median is 1.7°.

Your own assessment after the re-learned trim was "much better". Two
earlier hops ended in collisions; those were confirmed as hitting
obstacles, not control failures.

## Pending

- Your own judgement of drift direction and distance over a fixed interval,
  under similar conditions.
- Whether the yaw rotation comes from the left stick. That needs the yaw
  command in the recorder, which is not logged yet.
- Battery voltage under load per flight. Only the minimum since boot is
  reported (3.04 V on a charged battery).
