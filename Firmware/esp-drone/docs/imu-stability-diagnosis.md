# ESP-FLY throttle-only drift: diagnosis report

Date: 2026-10-10 · Branch `imu-stability`

## 1. Symptom and constraints

- The drone is generally stable but moves sideways. With roll/pitch centred
  and the propellers unchanged, it usually drifts toward the **same side of
  the drone**. In test hops it often drifted until it hit something.
- ESP-Drone phone app, on-screen joysticks, Pitch/Roll trim at 0.
- IMU only (MPU6050). No camera use, no new hardware. Throttle keeps its
  meaning.

## 2. Verified configuration

**Firmware actually on the drone.** The 8 MB flash read back on 2026-10-10
was byte-identical to `drone_autonomy/manual_flight_firmware`
(`ESP_FLY_MANUAL_ONLY_V1`), a modified ESP-Drone build. It was not a build
of this repository's stock source. That tree was imported unchanged as
commit `fee775a`, and all work is layered on it. Rebuilding it here with
ESP-IDF v5.0.7 (xtensa gcc esp-2022r1-11.2.0) reproduces identical
`.text`, `.iram0.text` and `.data` sections. Only toolchain path strings and
the build timestamp differ.

What that build changes relative to stock (all preserved):
- camera removed
- Sense microSD chip-select held high on GPIO21
- PSRAM enabled; USB-Serial-JTAG as the primary console
- no start-up motor test
- validated RPYT input (±30° roll/pitch, ±200 °/s yaw)
- mutex-protected motor driver with a latched stop

| Item | Value |
|---|---|
| Target | ESP32-S3 (XIAO ESP32S3 Sense), partition table: single app large (1.5 MB) |
| IMU | MPU6050 at 1 kHz, DLPF 42 Hz; ±2000 °/s, ±16 g; software LPF 80 Hz gyro / 30 Hz accel |
| Estimator | complementary (Mahony), Kp = 0.4, Ki = 0.001, 250 Hz |
| Controller | cascaded PID: attitude 500 Hz (Kp 5.3, Ki 2.5) → rate (Kp 190, Ki 440, I-limit 33.3) |
| Mode | Angle (self-level) for roll/pitch; yaw rate with heading hold |
| Motor layout (verified on the bench) | M1 front-right, M2 back-right, M3 back-left, M4 front-left |
| Axis signs (verified on the bench) | +roll = left side up, +pitch = nose up |
| Brownout detector | enabled, level 7 |

## 3. Source findings

1. **No accelerometer offset correction.** Only a single magnitude scale is
   applied, so the per-axis zero-g offset becomes the attitude estimator's
   level reference.
2. **PID integrators reset only at thrust == 0**, so they wind up while the
   drone sits on the ground during a throttle ramp.
3. **The Kconfig pitch alignment was not a rotation** (determinant cos 2θ).
   It is inactive at the default 0.
4. **Gyro bias is calibrated once at boot.** The bench data (§5) shows a
   stationary gyro mean within ±0.01 °/s, so this is not a contributor today.

## 4. Measurements on the drone

### 4.1 Two-position level measurement

The drone rested on a table, then was turned 180° on the same spot.
5 s windows, 5000 samples each, still.

| | raw x (g) | raw y (g) | reading |
|---|---|---|---|
| Position A | −0.0008 | +0.0227 | roll +1.35°, pitch −0.05° |
| Position B (180°) | +0.0041 | +0.0226 | roll +1.35°, pitch +0.24° |
| → sensor/airframe offset | **+0.0017** | **+0.0227** | **roll +1.30°, pitch +0.10°** |
| → table tilt (cancelled) | | | pitch −0.14°, roll 0.00° |

The same roll error appeared in both orientations, so it belongs to the
drone, not the table. While holding "level", the old firmware flew 1.3°
left-side-down. That is a steady ≈0.22 m/s² push to the left (about 1 m in
3 s from rest).

After saving the offset, the drone read roll −0.00°, pitch +0.12° at
position B, i.e. only the table's tilt. It reloaded correctly after a reboot.

Gyro during the windows: mean |ω| ≤ 0.011 °/s, std ≈ 0.04 °/s.

### 4.2 Flight recorder (100 Hz, hands-off throttle-only hops)

Round 1 used the level calibration only, 9 throttle segments:
- **Ground wind-up.** On the take-off spot the drone sat at roll +1.5 to
  +4.3°, unlike the level table. During slow throttle ramps the controller
  fought that tilt. In flight 8 the rate integrators reached −14013 / +12745
  (the limit is ±14652), the motor commands split 6:1 (m1 35584 vs m3 5888),
  and the drone tipped before lifting off.
- **Mechanical imbalance.** In steady hover M3 (back-left) needed 1.10–1.27×
  the command of M1 (front-right). The back pair needed 9–17% more than the
  front pair, and the left pair 2–9% more than the right. Inverting the X
  mixer, the steady correction was roll +2560..+3584 and pitch −4096..−7424
  actuator units, every flight.
- **Take-off lean.** The integrators start at zero, so each lift-off leaned
  back-left (peak ≈2.8°, nose up and left side down) for ~1–1.5 s.
  Integrating the estimated tilt gives a **~0.35 m/s** slide at the moment it
  leaves the ground. This is an estimate from attitude, not a position
  measurement. That slide is the repeatable drift that carried the drone
  into obstacles.

Round 2 added the ground integrator hold, 5 segments:
- Integrators stayed ≈0 for the first 0.5 s (−89..+208, previously
  −1300..−1900).
- The take-off lean was unchanged (≈2.9° back-left, ~0.29 m/s), as expected
  with the same mechanics.

Round 3 added the torque trim (roll +2374, pitch −4435) on a charged
battery, 7 segments:
- Peak take-off tilt was **1.8°** (median).
- The estimated take-off slide was **0.11 m/s** (median, max 0.14), about
  3× less than before.
- The front/back component was ≈0. The residual was a small left slide,
  because the hover roll correction had grown to ≈+5900 while the roll trim
  was +2374.

Round 4 used the trim re-learned from all 11 clean hovers (roll +3853,
pitch −2598), 4 segments:
- The estimated take-off slide was **0.09 m/s** (median, max 0.10).
- The left-right component fell to −0.02..−0.05 m/s.
- The steady hover totals were roll +4992..+6144 and pitch −3200..−4608,
  so the trim supplies ≈70% as designed.
- You reported the flights as "much better".

The needed correction is **not constant across sessions**. Across 11 clean
hovers it spans roll +2560..+6272 and pitch −1408..−7424. Within round 3 the
pitch need fell monotonically from −5376 to −1408. This is consistent with
motor temperature, battery state or a shifting mass. A fixed trim therefore
removes most of the take-off lean, and the integrators handle the rest.

**Brownout.** On a half-charged battery the drone reset the moment the
motors spun up. `status` reported `reset=brownout`; the reset also explains
the Wi-Fi disconnects. With a charged battery the supply still dipped to
3.04 V under load.

## 5. Competing explanations: outcome

| Explanation | Verdict |
|---|---|
| Accelerometer level offset | **Confirmed**, 1.30° roll. Corrected (level calibration). |
| Take-off integrator wind-up on the ground | **Confirmed** (±14000). Corrected (integrator hold below 20000 thrust, later 26000; an autonomous take-off holds them until vert_ctrl's lift-off, user decision 2026-10-13, evening: docs/autonomy-a2b.md 2.4). |
| Mechanical imbalance (back-left needs more thrust) | **Confirmed**, consistent every flight but varying in size. Mostly offset by the torque trim. The root cause (mass distribution or a weaker M3) is unchanged: the battery cannot be moved and the M3 prop looks fine. |
| Warm-up gyro bias drift | Not observed on the bench (≤0.01 °/s). No change made. |
| Room airflow / ground effect | Not separable with IMU-only data. Remains possible for the residual. |

## 6. Selected outcome

A targeted firmware change, configured from measurements:
1. accelerometer level offset x = +0.00166 g, y = +0.02266 g (NVS)
2. roll/pitch integrators held at zero below 20000 thrust
3. torque trim roll +3853, pitch −2598 (NVS), learned from all 11 clean
   hovers at 70% of the median

The USB console and flight recorder remain for measuring and re-learning.

## 7. Remaining uncertainty and limits

- **The imbalance varies.** The trim cannot match every flight. Expect a
  small take-off lean in either direction, and re-learn it
  (`trim-learn`) if behaviour changes, e.g. after a crash or a part change.
- **Yaw rotates in hover**, with mean rates of −14..+19 °/s whose sign varies
  between flights. The recorder does not store the yaw command, so a
  sideways component of the throttle thumb on the left stick cannot be
  ruled out.
- **Thin thrust margin.** Hover needs ≈43–48k of 65.5k, and the app caps
  thrust at 52428. M3 sometimes saturates.
- **Battery sag.** The supply dips to ~3.0 V; fly on a well-charged battery
  to avoid brownout resets.
- **No horizontal reference.** A level drone can still drift from air
  movement or residual velocity. The MPU6050 cannot sense or correct this.
