# Autonomy stage A2: state machine, vertical control, kill paths, motor watchdog

Stage A2 lets the Mac arm the drone, take off to a fixed height, hover until
STABLE for 1 s and take the photos TAKEOFF asks for (`photoN`; the A2b Mac
tool's default is one, so the pre-wait air clock keeps its margin, and more
can be asked with PHOTO during the wait). The drone then hovers in place and
waits for the Mac's next command (LAND or another PHOTO; stage C adds YAW and
STEP) for at most `hoverWaitMs` (default 5 s, at most 12 s; 8 s until
revision 5), then lands in place.
The firmware alone decides every thrust value. The wait relies on the camera +
accelerometer height observer (section 4.10). Without a healthy camera height
estimate the hover lands 2 s after it started. All of stage A is props off;
nothing here is flown before stage B.

**Revision 2 (review fixes, 2026-10-11).** Changes against the A2a build
(commit bc8efff), from the confirmed review findings and the user's and
lead's later decisions:

- **HOVER waits for the Mac** after the photos: the hover-wait rows 21-27 of 3.2,
  the new reasons `HOVER_TIMEOUT` and `HOVER_NO_CAM`, and the `PHOTO` command (3.3, 3.8).
- **The air clock starts at the first thrust above the idle spin** (RAMP), not
  at the detected lift-off. The pre-wait limit is `airMaxMs` (default 4.5 s),
  and there is a hard 10 s limit, wait included (3.7; revision 5: 6 s and
  18 s).
- **The ramp and T_lo are bounded by the stored hover gain** (4.2, 4.4):
  - RAMP stops rising at `rampTopRel` x kPrior / Vf (1.00).
  - An abort from that plateau first falls like LAND's ceiling.
  - T_lo <= `tLoMaxRel` x kPrior / Vf (0.90).

  These are ceiling and abort-rule changes, so they need sign-off.
- **The lift-off detector no longer subtracts bAir** on the ground (4.2).
- **The controller uses the camera observer's velocity and offset** while it is
  healthy, and HOLD keeps its loop closed on it (4.2, 4.10). `HOLD_NO_CAM` is a backstop.
- **A tilt or rate trip in LAND counts as touchdown only with contact
  evidence**; a free fall or tracking trip never does (3.2 LAND rows 3-4).
- **Lead's engineering defaults:**
  - capLo 1.65;
  - app-silence LAND at 500 ms;
  - fast battery LAND below 2.55 V;
  - DRY exempt from the reset-lock;
  - the consent gesture as the NVS switch `autogest` (default on, unchanged).
- **The tests reach the airborne limits:**
  - FLIGHT-timing rows in the Monte Carlo;
  - a coupled test of the real state machine and controller (`tools/tests/test_auto_sortie.c`);
  - a mid-climb LAND sweep for false touchdowns.

  Section 4.9 has the new numbers.

**Revision 3 (second review, 2026-10-11).**

- **Learning** takes one sample, Khat on the first HOLD step at `holdLoopS`.
  It no longer learns the running Kfrz. Only a LAND started by `CMD_LAND` or
  `HOLD_NO_CAM` teaches (4.5).
- **Section 4.9 samples the corners of the user range** (8 corners with 100 or
  50 seeds each), next to the uniform draw. Without a camera the worst case is a
  corner: 2.04-2.08 m and 2.17-2.20 m/s.
- **Late lift-off:** the coupled run never reaches the air clock, so it is no
  longer cited as evidence for it. A new coupled case, `testAirClock`, and
  `test_auto_fsm.c` cover the clock (4.9, 9.2b).
- **Motor guard:** a late callback no longer restarts the window, and a trip
  runs the restart-lock blink (7.1, 7.2, 9.3).
- **The camera height observer runs in shadow mode** on every flight. It is
  logged in the recorder, in the flight summaries and by the USB `camvz`
  command (4.10, 8.6). The RTC flight log's magic changed: save `flights`
  and `crash` before flashing (8.6).
- **Open items for stage B** (section 11).

**Revision 4 (A2b build, 2026-10-12).** The state machine and the vertical
controller in this document match the A2b build. The glue as built (the
loop, barriers, phone hook, reset-lock record, NVS table, recorder, gates) is
described in `docs/autonomy-a2b.md`, the link as built in
`docs/autonomy-link.md` and the props-off checklist in
`docs/autonomy-bench.md`; each is owned by its package, and this document
does not repeat them. Changes against revision 3, from the user's decisions
of 2026-10-12 and the A2b review:

- **The stored hover gain is 165000** (`VZ_K_DEFAULT`, NVS `vzK`): the logged
  hover K at 40 kHz PWM is 163k (3.4-3.6 V) to 167k (3.25-3.3 V); A2a's
  149.5k was 11 % low (4.8, open issue 7).
- **`rampRate` 13250/s** (was 12000): 12000 x 165000 / 149500, the same
  fraction of the hover thrust per second as validated. With an absolute
  12000 a 165k prior stretched the RAMP and hid lift-offs (4.2, 4.9). Needs
  sign-off.
- **"Always slow land": LAND from CLIMB** takes T_ref from the thrust averaged
  in CLIMB, kept within [T_lo, `landClimbRefRel` x kPrior / Vf] (1.00), and
  T_lo within 0.25 s of the lift-off or of a capped climb step (4.2; open
  issue 1 resolved, with a residual corner). Needs sign-off.
- **A slow RAMP abort** from any RAMP thrust >= `abortSlowRel` x kPrior / Vf
  (0.60), not only from the plateau (4.2). Needs sign-off.
- **The camera stays awake through every FLIGHT and BENCH take-off**
  (`camNoStandby` at any battery; `AUTO_CAM_AWAKE_VMIN` is gone), so its
  drift hold (camera flow + gyro) and the height observer have frames from
  the lift-off on (4.11, 8.4).
- **`vmin` 4.00 V** (`AUTO_VMIN_DEFAULT`, NVS 4000 mV), above every logged
  camera-awake brownout. **G3 does not apply to DRY** (a finite reading only),
  so a props-off DRY session never needs a lowered `vmin` (3.4, 3.7). Needs
  sign-off.
- Sections 2, 6 and 8 keep the design's rules and add only what the state
  machine, the controller or the frozen A2b interface (`autonomy.h`,
  `auto_glue_core.h`, `auto_mailbox.h`) fixes; for the glue and the link as
  built they point to `docs/autonomy-a2b.md` and `docs/autonomy-link.md`.
  Section 8.4 has the take-off window's rationale with the 165k prior;
  9.4 the bench items that check this document's rules.
- Every number in 4.5, 4.9, 9.1, 9.2 and 9.2b is re-measured on this build,
  and those sections list the new checks.

**Revision 5 (2026-10-12, evening): 0.60 m and a 10 s hover wait.** After
the first full autonomous mission (commit 4d35897: TAKEOFF 40 cm, true
height ~0.32 m, STABLE, photo, a 5 s wait, `HOVER_TIMEOUT`, a soft
touchdown; 9.4 s in the air, battery 4.15 -> 3.49 V in flight, 3.3 V rail
down to 3.14 V) the user decided to fly higher, 50-60 cm, with a 10 s wait.
User decision of 2026-10-12 (evening); the limits it moves were the user's
and the lead's decisions of 2026-10-11/12. Changes:

- **Take-off height up to 0.60 m:** `AUTO_HEIGHT_CM_MAX` 40 -> 60,
  `VZ_H_TARGET_MAX` 0.40 -> 0.60 (minimum 0.20 unchanged).
- **The estimate guard and two controller timings sized for 0.40 m grow
  with the target above it** (`vzConfigSetHeight()`, called by the glue's
  `vzStart()`; targets up to 0.40 m keep these values): `hMax` + (hTarget -
  0.40), so it keeps its 0.6 m over the target (1.0 -> 1.2 m at 0.60 m),
  `climbMaxS` + (hTarget - 0.40) / vClimbMax
  (2.0 -> 2.5 s at 0.60 m: the nominal climb to 0.60 m takes 1.72 s against
  1.27 s to 0.40 m, sim) and `landFlatS` + (hTarget - 0.40) / |landVref|, at
  most 2.0 s (1.5 -> 2.0 s at 0.60 m: with 1.5 s the slow LAND from 0.60 m
  touched down at p50 0.39 m/s, with 2.0 s at 0.27, from 0.40 m 0.25;
  nominal, warm camera, sim).
- **The hover wait at most 12 s** (`AUTO_HOVER_WAIT_MAX_MS` 8000 -> 12000;
  the default `autowait` stays 5000: the operator sets 10000), **the hard
  air clock 18 s** (`AUTO_AIR_HARD_MS` 10000 -> 18000), and the controller's
  `holdMaxS` backstop 9.0 -> 17.0 s (`VZ_HOLD_MAX_LIMIT_S` 10 -> 18), so the
  state machine still lands first.
- **The pre-wait air clock 6.0 s** (`AUTO_AIR_MAX_DEFAULT_MS` 4500 -> 6000,
  `AUTO_AIR_MAX_HI_MS` 6000 -> 8000). Logged at 0.32-0.40 m: HOVER 2.25-2.49 s
  after the first RAMP step (the recorder's clock runs ~7 % slow; corrected),
  the wait 1.62-1.83 s later with one photo. At 0.60 m the climb takes
  ~0.45 s more (sim), so the wait starts at ~4.3-4.8 s: >= 1.2 s inside 6 s
  (sim, realistic range: max 4.83 s; user range max 5.08 s). With the 6 s
  pre-wait limit a 10 s wait ends >= 2 s inside the 18 s clock (sim: the
  wait's LAND at most 14.83 s, realistic). `AUTO_TAKEOFF_MAX_MS` stays
  8000 (1 + 0.8 + 2.5 + 2.5 = 6.8 s at 0.60 m); above an `autoair` of
  ~6.2 s it, not the air clock, lands a take-off stuck in RAMP (earlier).
- Unchanged: every trip and KILL, heartbeat loss, NOT_STABLE, HOVER_NO_CAM,
  the fast-sag LAND, the STABLE grace and budget, the LAND rules (landTop,
  stall, near jolt, the 6 s cut and the 6.5 s backstop), DRY and BENCH.
- **What the sim says about 0.60 m and 10 s** (4.9, "Revision 5"): every
  realistic sortie waits and lands `HOVER_TIMEOUT`, never KILLED; the true
  hover is ~0.5-0.6 m (the 40 cm sorties hold ~0.3-0.4 m in the sim, 0.32 m
  logged); impact p50 / p95 / max 0.34 / 0.67 / 0.79 m/s against 0.28 /
  0.57 / 0.66 from 0.40 m on the same draws; true airborne p50 16.8 s.
  Costs: the camera's slow offset c moves the drone for 10 s instead of 5
  (the camera study's verdict was "~5 s at ~0.5 m fine, 10-20 s not"); a
  LAND during the longer climb (phone, Mac, controller guard) and the
  accelerometer-only LAND without a camera touch down harder (9.1 item 7b,
  9.2b); without a camera the 6 s pre-wait clock no longer cuts the slowest
  hovers (at 0.30 m the no-camera uniform peak 1.68 -> 1.84 m, corners
  unchanged at 2.08 m).
- **The worst case without a camera height estimate is higher at 0.60 m**
  (the observer never healthy, the user range's corner -20 mg, K -12 %;
  4.9): **2.82-2.96 m true at up to 2.63 m/s** at contact, every one landed
  by `HOVER_NO_CAM` (0.40 m: 2.18-2.29 m, 2.32 m/s; 0.30 m: 2.08 m,
  2.20 m/s). The accelerometer alone climbs longer to the higher target, and
  the 6 s pre-wait clock no longer cuts it (with A2b's 4.5 s it was
  2.49-2.61 m, 2.48 m/s, but a 0.60 m sortie would then reach its wait with
  < 1 s to spare). The uniform draw stays lower (2.56 m, 2.45 m/s). Only
  the kill switch stops such a climb sooner: the operator's kill height for
  the 60 cm sortie (`docs/autonomy-bench.md` C2) is set with this in mind.

**The 2026-10-13 interactive-hover failures (section 12).** Two FLIGHT
sorties failed: the camera drift hold followed a person walking in front of
the camera into an in-air contact that no trip caught, and a false CUSUM
alarm of the camera height observer landed the other `HOVER_NO_CAM`. Fixed in
estimation, health and gating only (no gain or LAND-law change, each a no-op
on the clean 2026-10-12 sorties): the observer's gain re-anchor (4.10), the
hold's HOVER scene gate (12.3), the in-air contact rule `COLLISION` (3.2 row
8b; a cut in 98cd4ae, a blind slow LAND since the user's decision of
2026-10-13, 12.8), touchdown values from the first impact of a contact
cluster, trim-learner guards, and the heat-soak settle of G2 (4.11; needs the
user's sign-off).

Sources: the autonomy design and its critiques (synthesis, critique-safety
B1-B5, critique-feasibility), the user's decisions of 2026-10-11 and
2026-10-12 (binding where they differ from the design), the A2b plan and its
review (R1-R21), the code on branch `imu-stability` and
the logged flights in `firmware-backups/cal/flight*.csv` and `flights-*.txt`.
Where this document makes a choice the design left open, it says so and why.

Contents: 1 work split, 2 the 1 kHz loop, 3 state machine, 4 vertical
controller, 5 kill/LAND/cut summary, 6 reset-lock, 7 motor watchdog,
8 integration rules for A2b, 9 tests and acceptance, 10 open issues,
11 open items for stage B, 12 the 2026-10-13 interactive-hover failures.

## 1. Work split

Stage A2a builds three independent pieces against the three headers below.
They share no source file except `CMakeLists.txt`, where each adds its own
line after a different anchor, so they merge without conflicts.

| Engineer | Creates | Edits | Host test | CMake line, after |
|---|---|---|---|---|
| 1 vertical | `utils/src/vert_ctrl.c` | none | `tools/tests/test_vert_ctrl.c` (+ optional `test_vert_replay.c`) | `"./utils/src/thrust_softstart.c"` |
| 2 state machine | `utils/src/auto_fsm.c` | none | `tools/tests/test_auto_fsm.c` | `"./utils/src/auto_proto.c"` |
| 3 motor watchdog | `utils/src/motor_guard_core.c`, `modules/src/motor_guard.c` | `motors.c/.h`, `crash_log.c`, `stabilizer.c` (init + alive call + fault hooks), `sensors_mpu6050_hm5883L_ms5611.c` (fault hook), `usb_diag.c` (`mguard`), `imu_level_cal_store.c/.h` (`mguard` key) | `tools/tests/test_motor_guard.c` | `"./utils/src/flight_summary.c"` and `"./modules/src/att_trim_auto.c"` |

Headers (frozen for A2a; a change needs all three engineers to agree):

- `components/core/crazyflie/utils/interface/vert_ctrl.h`
- `components/core/crazyflie/utils/interface/auto_fsm.h`
- `components/core/crazyflie/modules/interface/motor_guard.h`

The structs `vzState_t` and `autoFsm_t` are private to their module: the
owner may change their fields without asking. Every other type, constant
and enum value in the headers is the contract. Enum values are logged and
sent to the Mac: append only.

`tools/tests/test_auto_sortie.c` (engineers 1 and 2) runs the real
`auto_fsm.c` on the real `vert_ctrl.c` and its sim plant through a small
emulation of the A2b glue; it builds with both sources.

Pure modules (`vert_ctrl.c`, `auto_fsm.c`, `motor_guard_core.c`) include no
ESP-IDF or FreeRTOS header and build on the host like the existing tests:

```
cc -std=c99 -Wall -Wextra -Icomponents/core/crazyflie/utils/interface \
   -Icomponents/core/crazyflie/modules/interface \
   tools/tests/test_vert_ctrl.c components/core/crazyflie/utils/src/vert_ctrl.c \
   -lm -o /tmp/test_vert_ctrl && /tmp/test_vert_ctrl
```

Stage A2b (after A2a, one engineer) writes the glue: `modules/src/autonomy.c`
(the arbitration call, mailbox, RTC record, NVS, sim plant wiring), the link
commands in `auto_link.c`, the phone hook, the recorder fields, the USB
`auto` commands and the Mac tool. Sections 6 and 8 specify it so that A2a's
interfaces fit. Stage C items (YAW, STEP, approach) are out of scope. As
built, A2b was four packages (glue, link, Mac tool, and the controller and
state machine changes of revision 4); `docs/autonomy-a2b.md` describes the
glue as built.

## 2. The 1 kHz loop

Order inside `stabilizerTask` once A2b is in, as designed (the line numbers
are the design's; the code moved since). `docs/autonomy-a2b.md` section 2
describes the loop as built.

1. `sensorsWaitDataReady()`, `stateEstimator()`, `commanderGetSetpoint()` (unchanged).
2. **`autonomyArbitrate(&setpoint, &sensorData, &state, nowMs, loopGapUs)`** right
   after `commanderGetSetpoint` (stabilizer.c:636):
   1. build `autoFsmIn_t` (the previous step's `vzOutput_t` included); `autoFsmStep()`;
   2. carry out the one-step actions of `autoFsmOut_t` (yaw capture, thrust lock, emergency stop, capture start, LED);
   3. if `vzReset`: `vzDefaultConfig()`, then `hTarget` = `hTargetM`, `kPrior`, `bAir` and `capLo` from NVS; `vzInit()`; zero the stored `vzOutput_t` (phase `VZ_IDLE`); `vzSimInit()` when on the sim;
   4. on every step from the sortie's `vzReset` until the FSM is back in IDLE (so also after touchdown and in KILLED): `vzStep()` with `vzCmd`, inputs from the real IMU (FLIGHT, BENCH-imu) or from `vzSim` (DRY, BENCH default), and the camera observer's `camHealthy`, `camVzValid`, `camVz`, `camBias` from `camVzGlueVzInput(vzOutput.g0, &in)` (section 4.10; on the sim, the sim's camera model: warm, wake 1.5 s, c = 0, so DRY exercises the camera branch); then `camVzGlueSetLiftoffHint(vzOutput.liftedOff)` on real inputs; on the sim, then `vzSimStep(vzOutput.thrust, dt)`;
   5. unless the action is `NONE`, overwrite the whole setpoint (section 8.1) with thrust = 0 (`ZERO`, `CUT`) or `vzOutput.thrust` (`VERT_*`), then `thrust = min(thrust, thrustMax)`. The A2b interface does this with `autoSelectThrust()` (auto_glue_core.h): a NaN or negative command gives 0.

   The A2b interface also fixes the glue's step rate (`autonomy.h`,
   `auto_glue_core.h`): in IDLE with the action `NONE` the FSM steps every
   `AUTONOMY_IDLE_STEP_EVERY`-th loop (100, 10 Hz), and at once when a
   command is waiting, a KILL came or the session changed (`autoIdleSkip`);
   the camera, hold, ground, battery-minimum and lock inputs are read every
   `AUTONOMY_SLOW_EVERY`-th loop (10) while active; in every other state the
   FSM and the controller step every loop. `test_auto_sortie.c` emulates the
   glue with the same constants and rule (9.2b).
3. Restart lock, gcap, soft-start (stabilizer.c:637-678, unchanged; they only lower thrust).
4. `sensorsMpu6050SetThrust()`, `sitAwUpdateSetpoint()`, `controller()` (unchanged).
5. **Mode clamp, last stage:** `if (control.thrust > thrustMax) control.thrust = thrustMax;`
   immediately before the `emergencyStop` test and `powerDistribution()`. Thrust 0
   makes `powerDistribution()` call `powerStop()` (power_distribution_stock.c:96-99).
   The A2b interface (`autonomy.h`, `auto_glue_core.h`): `control.thrust =
   autoClampThrust(control.thrust, autonomyThrustMax())`, whose limit
   (`autoBarrierMax`) is 0 whenever the action is `ZERO` or `CUT`, in every
   mode, so ARMED, the pre-spin, the release hold, a landed LAND and KILLED
   keep the motors off at both barriers. With `NONE` it is 65535 and
   `control.thrust` is untouched. It fails closed: a NaN or out-of-range limit
   gives 0.
6. `powerStop()` / `powerDistribution()` (unchanged).
7. **`motorGuardStabilizerAlive()`.**
   - 7a. `camVzGlueStep()` (already built in, section 4.10): the shadow observer's
     step and any new camera frame, before the flight recorder. Step 2.4 of the
     next loop reads its outputs.
8. **`autonomyNoteApplied(control.thrust)`**: the thrust the motors really got,
   used as `thrustApplied` on the next step in FLIGHT and BENCH-imu
   (`autonomy.h`: 0 after `powerStop()`, an emergency stop or with every duty
   at zero).

`thrustMax` comes from the FSM every step: 65535 with action `NONE`; with
any other action FLIGHT `VZ_THRUST_ABS_MAX` (58000; 54000 until 2026-10-14), DRY
0, BENCH the bench cap. Steps 2.5 and 5
are two independent barriers, so a bug in one cannot spin the motors in DRY.

## 3. State machine (`auto_fsm.h`)

### 3.1 States and what each one does

| State | Thrust action | `vzCmd` | Setpoint | Notes |
|---|---|---|---|---|
| IDLE | `NONE` (`ZERO` for `AUTO_RELEASE_HOLD_MS` after leaving any other state) | NONE | untouched | phone flies as before |
| ARMED | `ZERO` | NONE | autonomy, thrust 0 | waits for TAKEOFF |
| TAKEOFF, pre-spin (first `AUTO_PRESPIN_MS`) | `ZERO` | NONE | autonomy, thrust 0 | `prespinLed` on; stillness re-checked |
| TAKEOFF, after pre-spin | `VERT_CTRL` | TAKEOFF | autonomy | `vzReset` on the first step |
| HOVER | `VERT_CTRL` | TAKEOFF | autonomy | STABLE, photos, then the wait for the Mac (`waiting`) |
| LAND, controller landing | `VERT_LAND` | LAND | autonomy | |
| LAND, controller ended (DONE or CUT) | `ZERO` | LAND (CUT after a touchdown trip) | autonomy, thrust 0 | waits for rest |
| KILLED | `CUT` | CUT | autonomy, thrust 0 | latched >= 2 s |

"Active" means any state but IDLE. "Airborne states" means TAKEOFF after
the pre-spin, and HOVER. The FSM ignores `in.vz` while `in.vz.phase` is
`VZ_IDLE` (the glue zeroes it at `vzReset`, so a previous sortie's DONE is
never read as this sortie's).

### 3.2 Transitions

**Since 2026-10-15 ("Claude decides", user decisions 2026-10-14 ~20:30-20:40;
docs/autonomy-a2b.md section 22, the inventory in 22.1)** the rows below that
landed only because something was unusual hold and report instead: no
`HOVER_TIMEOUT` (the wait is information), no `AIR_TIME` (no air clock), no
`SORTIE_DONE`, `NOT_STABLE` / `CAPTURE_FAIL` / the pre-wait clock start the
wait with an event, `HOVER_NO_CAM` became the camera rule (CAM_LOST, LAND
`SINK` after 5 s, `RESUME`), `MOVE_NO_CAM` / `MOVE_TILT` stop the motion, a
contact in HOLD is `CONTACT` (row 8b: LAND `COLLISION` only in the climb
without the camera height), one loop gap is an event (the third in 1 s
lands), `PALM` is gone. Kept: the cut rows, the battery rows (`VBAT_LOW`,
`BATT_LOW`, `VBAT_DIP`), the link rows (`HB_LOST`, `SESSION`, the phone's
`APP_INPUT` / `APP_SILENT`), the Mac's LAND / DISARM / KILL, the take-off and
LAND backstops. The tables keep the history.

Evaluated every step in the order listed; the first that fires wins. Times
are measured with `in.nowMs`; "for X ms" means the condition held on every
step for at least X ms (a persistence timer, reset when it fails). "Mac
KILL" means `killCount` changed since the previous step or `cmd.type` is
`AUTO_CMD_KILL`; "app input" means `appInputCount` changed since the
previous step.

**IDLE**

| Trigger | To | Reason | Result |
|---|---|---|---|
| ARM, kill latch active (KILL < `AUTO_KILL_LATCH_MS` ago) | IDLE | - | `KILL_LATCHED` |
| ARM with mode > 2, or `airMaxMs` not 0 and outside `AUTO_AIR_MAX_MIN_MS`..min(NVS `autoair`, `AUTO_AIR_MAX_HI_MS`) | IDLE | - | `RANGE` |
| ARM, `autoFsmGuards(mode)` != 0 | IDLE | - | `GUARDS`, detail = bitmap |
| ARM, all guards pass | ARMED | `ARM` | `OK`, detail = `armToken` (= `in.randToken`) |
| Mac KILL (killCount changed) | IDLE (kill latch starts) | - | `OK` |
| TAKEOFF | IDLE | - | `BAD_STATE` |
| LAND, DISARM | IDLE | - | `OK` (no-op, detail = state) |

On ARM the FSM latches the mode, `airMaxMs` (0 means the NVS value), the
hover wait `hoverWaitMs` and `hoverAccelMs` (NVS `autowait`, `autoaccel`,
forced into 0..12000 and 0..3000), and in BENCH the bench cap = min(gcap, `AUTO_BENCH_THRUST_MAX`). Every later step
the bench cap becomes min(bench cap, gcap) when gcap > 0: a gcap change can
only lower it.

**ARMED** (motors off)

| Trigger | To | Reason |
|---|---|---|
| Mac KILL | KILLED | `KILL_CMD` |
| DISARM or LAND command | IDLE | `DISARM_CMD` / `LAND_CMD` |
| app input (appInputCount changed) | IDLE | `APP_INPUT` |
| heartbeat age > `AUTO_HB_LAND_MS` | IDLE | `HB_LOST` |
| `sessionChanged` | IDLE | `SESSION` |
| app required and app age > `AUTO_APP_SILENT_MS` | IDLE | `APP_SILENT` |
| moved: tilt >= 8 deg, or any gyro axis > 10 dps, or \|\|acc\| - 1\| > 0.10 g, for `AUTO_MOVED_MS` | IDLE | `MOVED` |
| a G1 lock appears (restart lock and reset-lock except in DRY, emergency stop, motors not ready, watchdog off/tripped) | IDLE | `LOCKED` |
| `AUTO_ARM_TIMEOUT_MS` since ARM | IDLE | `ARM_TIMEOUT` |
| TAKEOFF, token != armToken | ARMED | result `BAD_STATE`, detail 0 |
| TAKEOFF, heightCm / photoN / photoGapMs out of range or actWaitMs != 0 | ARMED | result `RANGE` |
| TAKEOFF, guards re-checked for the ARM mode fail | ARMED | result `GUARDS`, detail = bitmap |
| TAKEOFF, all pass | TAKEOFF | `TAKEOFF`, result `OK`; `yawCapture`; sortie parameters latched |

**TAKEOFF** during the pre-spin (stateMs < `AUTO_PRESPIN_MS`; motors still off)

| Trigger | To | Reason |
|---|---|---|
| Mac KILL | KILLED | `KILL_CMD` |
| LAND or DISARM command, app input, heartbeat > 1000 ms, session change, app silent | IDLE | as in ARMED |
| moved (as in ARMED) | IDLE | `MOVED` |
| a G1 lock (as in ARMED) | IDLE | `LOCKED` |
| stateMs reaches `AUTO_PRESPIN_MS` | (stay) | `vzReset`, `hTargetM` = heightCm / 100, controller starts |

**TAKEOFF** after the pre-spin, and **HOVER**

| # | Trigger | To | Reason |
|---|---|---|---|
| 1 | Mac KILL | KILLED | `KILL_CMD` |
| 2 | any float input non-finite (attitude, tilt, attErr, gyro, acc norm, vbat, vbatFastMin, vz.h, vz.v, vz.thrust) | KILLED | `NAN` |
| 3 | tilt > 30 deg for 50 ms | KILLED | `TILT` |
| 4 | \|gx\| or \|gy\| > 500 dps for 30 ms | KILLED | `RATE` |
| 5 | \|acc\| < 0.4 g for 100 ms | KILLED | `FREEFALL` |
| 6 | attErr > 10 deg for 500 ms | KILLED | `ATT_ERR` |
| 7 | motor watchdog tripped / motors not ready / emergency stop set by someone else | KILLED | `MOTOR_GUARD` / `MOTOR_STOP` / `ESTOP_EXT` |
| 8 | vz phase CUT | KILLED | `VZ_CUT` |
| 8b | in-air contact: \|acc\| > 1.6 g or \|gx\| or \|gy\| > 250 dps (yaw excluded) for 3 ms, the controller in CLIMB or HOLD and its tAir >= 0.2 s (lift-off + 0.2 s), FLIGHT or DRY (never BENCH) | LAND | `COLLISION`; `collided` (latched to the next ARM): the camera hold blocked from this step, the controller's descent blind (`vzInput_t.blind`), a running motion stopped (`LEFT_HOVER`) |
| 8c | palm cue: the controller's aRel > 0.12 g for 20 ms, HOVER with vz HOLD for >= 1 s (`vzOutput_t.tPhase`), FLIGHT or DRY (never BENCH), after the trips and `COLLISION` (docs/autonomy-a2b.md 18.3) | LAND | `PALM`: the normal slow LAND (not fast, not blind, the hold not blocked by the cue); the controller probes for support (`vzInput_t.probe`); a running motion stopped (`LEFT_HOVER`). A hard contact during it (8b's condition) makes it blind from that step (`collided`, the hold blocked), the reason stays `PALM` |
| 9 | vz phase LAND or DONE (the controller landed on its own) | LAND | `VZ` (+ `vzLandReason` = vz.landReason) |
| 10 | LAND command, or DISARM command | LAND | `LAND_CMD` / `DISARM_CMD` |
| 11 | app input | LAND | `APP_INPUT` |
| 12 | heartbeat age > 1000 ms | LAND | `HB_LOST` |
| 13 | app required and app age > 500 ms | LAND | `APP_SILENT` |
| 14 | `sessionChanged` | LAND | `SESSION` |
| 15 | vbat (10 Hz) < 3.05 V for 1000 ms (2.60 V since 2026-10-13; in HOVER `BATT_LOW` comes first since 2026-10-14, autonomy-a2b.md section 16) | LAND | `VBAT_LOW` |
| 16 | 0 < vbatFastMin < 2.55 V | LAND | `VBAT_DIP` |
| 17 | loopGapUs > 20000 | LAND | `LOOP_GAP` |
| 18 | airMs >= `AUTO_AIR_HARD_MS` (18000), or not waiting and airMs >= airMaxMs | LAND | `AIR_TIME` |
| 19 | TAKEOFF only: stateMs >= `AUTO_TAKEOFF_MAX_MS` | LAND | `TAKEOFF_TIMEOUT` |
| 20 | TAKEOFF only: vz phase HOLD | HOVER | `HOLD` |
| 21 | HOVER, FLIGHT: now - HOVER entry >= hoverAccelMs and not `in.vz.camHealthy` | LAND | `HOVER_NO_CAM` |
| 22 | HOVER: waiting and now - wait start >= hoverWaitMs | LAND | `HOVER_TIMEOUT` |
| 23 | HOVER: the latest photo set FAILED, or not DONE n x gap + 600 ms after its start | LAND | `CAPTURE_FAIL` |
| 24 | HOVER: never STABLE for 1 s, now - HOVER entry >= 2000 ms, and (camera waived, or camViewMs >= 1500) | LAND | `NOT_STABLE` |
| 25 | HOVER, not waiting: (capture DONE, or photoN 0 and STABLE 1 s) and hoverWaitMs 0 | LAND | `SORTIE_DONE` |
| 26 | HOVER, not waiting: capture DONE, or photoN 0 and STABLE 1 s | (stay) | `waiting` from now |
| 27 | HOVER: STABLE 1 s, photoN > 0, capture not started | (stay) | `captureStart` with photoN, photoGapMs |

STABLE 1 s is `stable` and `stableMs` >= 1000 on the same step (section 3.5).

Row 8b (2026-10-13, section 12; row 6b, KILLED, in 98cd4ae..eb9264d; LAND
by the user's decision of 2026-10-13, 12.8) comes after every cut (rows 1-8)
and before every other LAND row on the same step. In CLIMB and HOLD anything
above 250 dps for 3 ms is `COLLISION` long before the 30 ms of `RATE`, and
the rate trip then cuts in that LAND when the rate holds on (its timer runs
through the state change; LAND row 4): a drone that flips, spins or falls
after the contact is still cut. It is not part of the LAND rules: touchdowns
reach 1.3-8.8 g and 828 dps, and LAND keeps its own (rows 3-4 below); so it
fires at most once a sortie, and a contact in LAND changes nothing. The
recorder's CLIMB/HOLD of the ten logged autonomy sorties of 2026-10-12/13
from lift-off + 0.2 s reach 1.16 g and 77 dps outside flight 1's contact (an
external brush at 12.71 s); their lift-off windows 1.20 g and 63 dps; flight
1's first contact 1.79 -> 3.98 g and 214 -> 492 dps.

After the rows: a `PHOTO` command in HOVER while waiting, with no set in
progress, starts a set (`captureStart` with its n and gap, result `OK`); it
does not extend the wait. While a set runs it gets `BUSY`, out of range
`RANGE`, and anywhere else `BAD_STATE` (3.3).

`airMs`, the air clock, counts from the first step with `in.vz.phase` RAMP,
CLIMB or HOLD (or `liftedOff`): the first thrust above the idle spin. The
detected lift-off can lag the true one by over a second when an in-flight
offset hides the slow rise out of ground effect, so a clock started there let
the true airborne time pass its limit by ~1.1 s (review finding). The FSM
enters LAND with the controller's next command; the controller turns that into
its abort ramp if it has not lifted off yet.

**LAND**

| # | Trigger | To | Reason |
|---|---|---|---|
| 1 | Mac KILL | KILLED | `KILL_CMD` |
| 2 | non-finite input; watchdog/motor stop/external emergency stop | KILLED | `NAN` / `MOTOR_*` / `ESTOP_EXT` |
| 3 | trip 3 or 4 above (tilt, rate) while the controller is still landing, with `in.vz.tdThrustOk` and contact in the last `AUTO_TD_CONTACT_WIN_MS` (300 ms): \|acc\| > 1.5 g or `in.vz.aRel` > 0.6 g | (stay, controller ended) | end = `TOUCHDOWN_TRIP`; `landTrip` = the trip; `vzCmd` CUT from now |
| 4 | any other trip 3-6 (free fall and tracking always; tilt and rate without the thrust condition or without contact) | KILLED | `TILT` / `RATE` / `FREEFALL` / `ATT_ERR` |
| 5 | vz phase DONE | (stay, controller ended) | end = `TOUCHDOWN`; `vzLandReason` = vz.reason |
| 6 | vz phase CUT | (stay, controller ended) | end = `LAND_TIMEOUT` if vz.reason is `LAND_TIMEOUT`, else `VZ_CUT` |
| 7 | stateMs >= `AUTO_LAND_BACKSTOP_MS` and the controller still landing | KILLED | `LAND_BACKSTOP` |
| 8 | controller ended and (at rest for 1000 ms, or 3000 ms since it ended) | IDLE | the end reason; `lockAppThrust` |

Rows 3, 5 and 6 are not transitions: `reason` stays the LAND trigger (for
example `APP_INPUT`) until row 8, whose transition carries the end reason.
Both are in the RTC transition log. Once the controller has ended, later
vz phases and trips 3-6 are ignored (the thrust is already 0). App input,
LAND and DISARM in LAND change nothing
(result `OK`); heartbeat loss, battery and the other LAND triggers are moot.

**KILLED** (thrust `CUT`; `setEmergencyStop` on entry)

| Trigger | To | Reason |
|---|---|---|
| stateMs >= 2000, at rest for 1000 ms, and (app age > 500 ms or `appThrustZero`) | IDLE | `KILL_CLEARED`; `lockAppThrust`; `clearEmergencyStop` only if this FSM set it |

"At rest": every gyro axis < 10 dps and \|\|acc\| - 1\| < 0.15 g (the
stabilizer's soft-start test, stabilizer.c:646-649).

**Leaving any active state for IDLE** always outputs `lockAppThrust` (so
the app must send zero thrust before its throttle works again,
crtp_commander_rpyt.c:186/228-233) and keeps the thrust action `ZERO` for
`AUTO_RELEASE_HOLD_MS` (600 ms). A setpoint the app sent before the lock
stays in the commander queue and keeps its thrust for up to 2000 ms after the
app falls silent (commander.c: only roll and pitch are levelled after 500 ms),
so the glue also calls `commanderDropSetpoint()` on `lockAppThrust`: the
queued setpoint never reaches the motors when autonomy lets go (final review
E2E-1; `test_autonomy_glue_host` `handback`).

### 3.3 Commands and results

| Command | IDLE | ARMED | TAKEOFF / HOVER | LAND | KILLED |
|---|---|---|---|---|---|
| ARM | see 3.2 | `BAD_STATE` | `BAD_STATE` | `BAD_STATE` | `BAD_STATE` |
| TAKEOFF | `BAD_STATE` | see 3.2 | `BAD_STATE` | `BAD_STATE` | `BAD_STATE` |
| PHOTO | `BAD_STATE` | `BAD_STATE` | HOVER waiting: `OK` (set started), `BUSY` (a set runs) or `RANGE`; otherwise `BAD_STATE` | `BAD_STATE` | `BAD_STATE` |
| LAND | `OK` no-op | `OK` -> IDLE | `OK` -> LAND (pre-spin: -> IDLE) | `OK` no-op | `OK` no-op |
| DISARM | `OK` no-op | `OK` -> IDLE | `OK` -> LAND (pre-spin: -> IDLE) | `OK` no-op | `OK` no-op |
| KILL | `OK`, kill latch | `OK` -> KILLED | `OK` -> KILLED | `OK` -> KILLED | `OK` |

A stop-type command is never refused. `cmdDetail` is the state before the
command unless 3.2 says otherwise. Only TAKEOFF checks the token.

### 3.4 ARM guards

`autoFsmGuards(mode)` returns the failed bits. ARM and TAKEOFF both check
all of them.

| Bit | Name | Passes when |
|---|---|---|
| G0 | `LINK` | `sessionValid` and hbAgeMs < 300 |
| G1 | `LOCKS` | !restartLocked, !resetLocked (neither checked in DRY: it cannot spin a motor), !emergencyStop, motorsReady, motorGuardOn, !motorGuardTripped |
| G2 | `STILL` | onGround and onGroundMs >= 3000 (FLIGHT: >= min(20000 + 1.5 x lastFlightMotorMs, 90000), `AUTO_SETTLE_MIN_MS` and the heat soak `AUTO_SOAK_*`, section 4.11; needs the user's sign-off), \|accNorm - 1\| < 0.05 g, every gyro axis < 3 dps, tilt < 8 deg |
| G3 | `BATT` | FLIGHT and BENCH: vbat >= vmin (resting: G2 holds, so the motors are off) and vmin within 3.70..4.20 V; DRY: vbat finite (A2b) |
| G4 | `CAMERA` | FLIGHT only: camState RUNNING, holdEnabled, camFps > 10 (since 2026-10-14 camFps is the processed rate, 1 / the average frame interval, published only from the second fresh frame after a wake from a standby, after the sensor clock check has decided and while the flow loop keeps up with the clock; the camera sleeps on the ground and the ARM wakes it, docs/autonomy-a2b.md 17.1-17.2) |
| G5 | `APP` | appRequired off, or (appAgeMs < 500 and appInputAgeMs >= 1000) |
| G6 | `GCAP` | FLIGHT: gcap == 0; BENCH: 1 <= gcap <= 20000; DRY: any |
| G7 | `ENABLED` | autoEnabled |
| G8 | `VZPARAMS` | vzParamsOk |
| G9 | `HEAP` | heapIntFree >= `AUTO_HEAP_MIN` |
| G10 | `GESTURE` | appRequired off, or gestureRequired off, or appGestureAgeMs < 15000 |
| G11 | `KILL` | no Mac KILL in the last 2000 ms |
| G12 | `MODE` | FLIGHT: !simPlant |

G10 comes from critique-safety ("per-sortie consent gesture"): the person
holding the phone pushes roll past 10 deg and lets go (G5 then wants 1 s
with the sticks centred), which proves the phone path end to end before
every launch. It is an NVS switch, on by default (`autogest`, USB only; lead
decision). The 1 s pre-spin stays in every mode.

G3 in DRY (A2b, review R15) needs only a finite battery reading. DRY cannot
spin a motor (`thrustMax` 0 at both barriers of section 2), which is why G1
already exempts it from the reset-lock. A props-off DRY session then never
needs `auto set vmin` below 4.00 V, a persistent write that would also let
FLIGHT arm below it. A non-finite reading still refuses in every mode: it is
a sensor fault.

### 3.5 STABLE

In HOVER, `stable` is true on a step when all hold:

- `in.vz.holdSteady` (section 4.2 HOLD);
- \|roll\| and \|pitch\| <= 4 deg; \|gx\| and \|gy\| <= 30 dps; \|gz\| <= 10 dps;
- FLIGHT only: camState RUNNING, camFrameAgeMs <= 300 (A2a: 200), holdApplied and not holdCoasting.

A STABLE streak starts on a step where `stable` is true. It survives a
dropout (steps with `stable` false) shorter than `AUTO_STABLE_GRACE_MS`
(150 ms): its start is kept and `stableMs`, the streak's age, counts on
through it. It ends on the step a dropout reaches 150 ms, on the step its
dropouts together reach `AUTO_STABLE_DROP_BUDGET_MS` (300 ms), and at once
on a step beyond a hard bound: \|roll\| or \|pitch\| > 6 deg (1.5 x),
\|gx\| or \|gy\| > 60 dps (2 x), \|gz\| > 30 dps (3 x), FLIGHT camera not
RUNNING, or the controller's output IDLE. A dropout's length is the time
since the last good step (`now - stableOkMs` on its last step), so it
counts in ms, not steps: on the drone `now` is the 1 kHz tick and the loop
runs ~7 % slower, so `now` moves 1 or 2 ms a step. A new streak starts
with an empty budget. `stableMs` is 0 without a streak. STABLE 1 s
(photos, rows 25-27, and what NOT_STABLE waits for) needs `stable` and
`stableMs` >= 1000 on the same step, so the photos never start inside a
dropout. The grace and the budget apply in every mode.

What a streak can hide: no reading beyond a hard bound, no dropout of
150 ms, and at most 300 ms of dropouts in all, so at STABLE 1 s the
conditions held on >= 70 % of the streak. A soft violation (the hard
bounds do not cover holdSteady, the hold flags or the frame age) can still
sit inside a streak, for less than 150 ms at a time and less than 300 ms in
all. Without the budget (602e6ec) the grace alone let a lasting violation
that came back for one step in every 150 ms reach STABLE 1 s: hold coasting
93 % of the time, holdSteady false 94 %, five soft failures in turn with
0.6 % good steps, a sustained yaw of 12 dps (sd 3) in 200 of 200 random
runs and a 4.5 deg lean in 90 of 200; with the budget all of them land
`NOT_STABLE` at HOLD + 2 s (a yaw of 8 dps, sd 3, inside the 10 dps limit
on average, still passes 181 of 200). The logged dropouts below were single
events of 10-75 ms, 0-150 ms per streak at STABLE 1 s (B's longer coasts,
up to 0.26 s, end its streaks).

Why (hops of 2026-10-12 evening, e66e223, recorder at 100 Hz; the
recorder's loop clock runs ~6-7 % slow, times below are real): with the
A2a rule, every condition had to hold on every step for 1 s. The STABLE
bit was on 87 % of the passed hop's HOVER (calib1) and 68 %, 24 % and 65 %
of the other three, yet only calib1 ever held it 1 s. What broke the
streaks was brief: three 75 ms hold coasts in calib2 (single weak frames),
one 10.2 dps yaw sample in attempt C (yaw rate rms 2.3-3.8 dps, peaks
9.7-10.2 dps in calm hovers), and frame ages (mean 149 ms: ~120-130 ms
latency plus up to ~90 ms between frames) that reach 196-197 ms at the
recorder's 100 Hz, just under the old 200 ms limit. Two calm hovers
(calib2, C) landed `NOT_STABLE`; attempt B landed `HOVER_NO_CAM` (its
camera height observer never converged, so it must still land). Replay of
the HOVER rows through the real `autoFsmStep` (holdSteady, which the
recorder lacks, inferred from the recorded bit; with the A2a rule the
replay reproduces all four recorded outcomes):

| Flight | A2a rule | This rule (dropouts in the streak at STABLE 1 s) |
|---|---|---|
| calib1 (passed) | STABLE 1 s at HOVER + 1.50 s, wait + 1.60 s | the same (0 ms) |
| calib2 | longest streak 0.70 s, `NOT_STABLE` at + 2.00 s | STABLE 1 s at + 1.49 s, wait + 1.59 s (two coasts bridged, 150 ms) |
| B (hop5s) | `HOVER_NO_CAM` at + 2.00 s | the same (longest streak 0.61 s; camera height never healthy) |
| C (evening) | longest streak 0.85 s, `NOT_STABLE` at + 2.00 s | STABLE 1 s at + 1.69 s, wait + 1.79 s (11 ms) |

The results are the same with the grace alone (602e6ec), and within 1 ms
with FSM steps 1.07 or 1.5 ms apart (the slow loop) instead of 1 ms. C's
wait starts 0.25 s before its pre-wait air clock would land it (RAMP ->
HOVER took 2.46 s of the 4.5 s): holdSteady came only at HOVER + 0.69 s
there. In every logged hover the gyro rms is <= 3.0 dps (x, y) and <=
3.8 dps (z), tilt <= 1.8 deg: far inside the hard bounds. With a gz bias
added to the recorded rows, calib1, calib2 and C all still pass at +6 dps
and all land `NOT_STABLE` at +8 dps (the grace alone passed all three at
+8; the A2a rule failed all three from +4).

The camera part is waived in DRY and BENCH because the drift hold needs
airborne thrust to engage. The NOT_STABLE deadline (3.2 #24) counts from
the later of HOLD + 2 s and 1.5 s of fresh camera view (`camViewMs`:
frames <= `AUTO_VIEW_FRAME_AGE_MS`, 200 ms, as in A2a), because the
camera sleeps 1.5 s (warm) or 3 s (cold) after a ground start
(camera_flow.c:71-75) and a cold sortie could otherwise never pass
(critique-feasibility section 3). A2b keeps the camera awake through
autonomous take-offs (4.11), so the view normally runs from the ground on;
the rule stays for a camera that restarts.

### 3.6 Modes

| | FLIGHT | DRY | BENCH |
|---|---|---|---|
| Real thrust | vz thrust, <= 58000 (54000 until 2026-10-14) | 0 (clamped at 2.5 and at the last stage) | min(vz thrust, bench cap) |
| Vertical controller fed by | real IMU and battery | `vzSim` | `vzSim` (default) or real IMU (`auto bench imu`, hand-lift tests) |
| Safety trips (tilt, rate, fall, tracking, NaN) | real IMU | real IMU | real IMU |
| Battery LAND rules | real battery | real battery | real battery |
| Resting battery at ARM (G3) | >= `vmin` (4.00 V) | any finite reading | >= `vmin` (4.00 V) |
| Camera through the take-off (`camNoStandby`) | awake | no window (no motors) | awake (the take-off rehearsal) |
| Camera guards (G4, STABLE camera part) | yes | waived | waived |
| Camera height rule in HOVER (3.2 row 21) | yes | waived | waived (the controller's `HOLD_NO_CAM` still applies on the real IMU) |
| Reset-lock (G1) | yes | exempt (lead decision: it cannot spin a motor) | yes |
| gcap (G6) | must be 0 | any | 1..20000 |
| Learning (`vzLearnK`, NVS writes) | yes | never | never |

The sim plant is unreachable in FLIGHT (G12 and the glue). `thrustApplied`
for the controller is the sim's command in DRY/BENCH-sim and step 8 of
section 2 otherwise.

### 3.7 Timing and thresholds

| Constant | Value | Why |
|---|---|---|
| `AUTO_HB_LAND_MS` | 1000 | user decision; the Mac sends HB at 10 Hz from its main loop |
| `AUTO_HB_ARM_MAX_MS` | 300 | three missed heartbeats |
| `AUTO_APP_SILENT_MS` | 500 | lead decision: the app sends at ~20 Hz (link age p50 23-26 ms, p99 53-61 ms); per-boot app gap maxima 181-319 ms, which 300 would turn into nuisance LANDs |
| `AUTO_APP_ARM_MAX_AGE_MS`, `AUTO_APP_CENTRED_MS` | 500, 1000 | synthesis G5 |
| `AUTO_GESTURE_WINDOW_MS` | 15000 | critique G10 |
| `AUTO_ARM_TIMEOUT_MS` | 10000 | synthesis |
| `AUTO_PRESPIN_MS` | 1000 | critique: warning before spin, re-check stillness (G2 cannot tell a steady hand from the floor) |
| `AUTO_ONGROUND_MIN_MS` | 3000 | gyro re-zero needs 2 s idle + its 1 s period (sensors_mpu6050_hm5883L_ms5611.c:217-218) |
| `AUTO_STABLE_MS` | 1000 | user decision |
| `AUTO_STABLE_FRAME_AGE_MS` | 300 (A2a: 200) | camera latency ~120-130 ms plus up to ~90 ms between frames; logged ages reach 198 ms (3.5) |
| `AUTO_STABLE_GRACE_MS`, hard bounds | 150 ms; tilt 6 deg, gx/gy 60 dps, gz 30 dps | 2026-10-12 hops: dropouts of 10-75 ms broke 1 s streaks of calm hovers (3.5) |
| `AUTO_STABLE_DROP_BUDGET_MS` | 300 | a streak's dropouts in all: STABLE 1 s means >= 70 % good; the hops that pass hold 0-150 ms, low-duty violations do not pass (3.5) |
| `AUTO_VIEW_FRAME_AGE_MS` | 200 | the glue's `camViewMs` (NOT_STABLE's 1.5 s camera view) keeps the A2a freshness, so NOT_STABLE timing is unchanged |
| `AUTO_AIR_MAX_*` | 6000 default, 2000..8000 (NVS `autoair`, ARM may only lower it) | the air clock until the wait starts. Revision 5 (user decision 2026-10-12 evening, 0.60 m): a 0.60 m sortie with one photo reaches its wait at ~4.3-4.8 s (logged 0.32-0.40 m timing plus the sim's +0.45 s climb), >= 1 s inside; the highest value adds 2 s for more take-off photos. A2b: 4500 (the user's 3.5 s from lift-off plus the ~1 s ramp), 2000..6000 |
| `AUTO_AIR_HARD_MS` | 18000 | time-to-STABLE + photos + hoverWaitMs never more than 18 s of air clock: a 10 s wait after the 6 s pre-wait limit ends >= 2 s inside it. User decision 2026-10-12 evening (A2b, lead decision: 10000). The interactive hover: 80000 (docs/autonomy-a2b.md 14); since 2026-10-14 120000 (a2b 18.1) |
| `AUTO_HOVER_WAIT_*` | 5000 default, 0..12000 (NVS `autowait`) | user decisions: 5 s, then 10 s flown from 2026-10-12 evening (`auto set autowait 10000`; A2b maximum 8000); 0 lands right after the photos (`SORTIE_DONE`); the camera study found ~5 s at ~0.5 m safe, 10-20 s not |
| `AUTO_HOVER_ACCEL_*` | 2000 default, 0..3000 (NVS `autoaccel`) | lead decision: HOVER on the accelerometer alone this long, then the camera height estimate must be healthy; below the controller's `holdNoCamS` (3 s) |
| `AUTO_CAPTURE_EXTRA_MS` | 600 | camera 16.9 fps (59 ms) plus the copy; a stuck capture lands |
| `AUTO_TAKEOFF_MAX_MS` | 8000 | backstop over pre-spin 1 + spin 0.8 + ramp <= 2.5 + climb <= 2.0 s (2.5 s at 0.60 m: 6.8 s). With the default `autoair` the air clock lands a take-off stuck in RAMP first (at ~7.8 s); with `autoair` above ~6.2 s this backstop lands it first, at 8 s |
| `AUTO_VMIN_DEFAULT` (G3) | 4.00 V (NVS `vmin` 4000 mV, 3700..4200); FLIGHT and BENCH only, DRY needs a finite reading | user decision 2026-10-12 (A2a: 3.90 V): brownouts with the camera awake through a manual full-throttle take-off started at <= 3.87 V, none at 3.91-3.98 V (2 take-offs); 4.00 V is above every logged brownout and every clean camera-awake start. The camera now stays awake in every FLIGHT and BENCH take-off (4.11), and G3 buys back the margin the standby protected |
| `AUTO_VBAT_LAND_V` / `_MS` | 3.05 V / 1000 ms (2.60 V since 2026-10-13) | user decision (critique B5): logged hover is 2.95-3.43 V loaded, so 3.30 V would trip on most sorties; 2026-10-13: battery limits removed (the 1 kHz 2.55 V dip is the brown-out edge guard) |
| `AUTO_VBAT_FAST_LAND_V` | 2.55 V | lead decision: the 1 kHz minima in the 19 good flights of the first diagnostic session went down to 2.62 V, so the former 2.85 V would have landed good sorties |
| `AUTO_LOOP_GAP_LAND_US` | 20000 | in-flight loop gaps never exceeded 3.23 ms in 57 logged flights |
| Tilt / rate / fall / tracking trips | 30 deg 50 ms; 500 dps 30 ms; 0.4 g 100 ms; 10 deg 500 ms | user decisions; normal tilt RMS ~1 deg, the one flip reached 1404 dps, the soft-start's own free-fall test is 0.5 g / 20 ms |
| `AUTO_COLLIDE_*` (row 8b) | \|acc\| 1.6 g or gx/gy 250 dps, 3 ms, armed 200 ms after the controller's lift-off | 2026-10-13 (section 12): logged CLIMB/HOLD 1.16 g, 77 dps (an external brush); flight 1's contact 3.98 g, 492 dps; touchdowns (LAND, not armed) up to 8.8 g, 828 dps |
| `AUTO_SOAK_PER_MOTOR`, `AUTO_SOAK_MAX_MS` (G2, FLIGHT) | 1.5 x the last flight's motor time, at most 90 s | 2026-10-13 (4.11): the rest level keeps rising 10-80 s after a landing; flight 2 took off 60 s after 41 s of flying. Needs the user's sign-off |
| Touchdown trips | a tilt or rate trip in LAND with `tdThrustOk` and contact (\|acc\| > 1.5 g or aRel > 0.6 g) in the last 300 ms counts as touchdown | touchdowns under power logged 267-509 dps and 1.8-3.1 g; the thrust condition alone can hold in mid-air (a LAND from CLIMB within 0.25 s of the lift-off or of a capped step, or on a drone lighter than its stored gain, starts below it) |
| `AUTO_LAND_BACKSTOP_MS` | 6500 | the controller cuts at 6000; this catches a controller that does not |
| `AUTO_LAND_REST_MS` / `_MAX_MS` | 1000 / 3000 | synthesis |
| `AUTO_KILLED_MIN_MS` / `_REST_MS` | 2000 / 1000 | synthesis |
| `AUTO_RELEASE_HOLD_MS` | 600 | longer than the commander's 500 ms stale timeout |
| `AUTO_KILL_LATCH_MS` | 2000 | the Mac sends KILL 10 times over 100 ms; no re-ARM right after |
| `AUTO_HEAP_MIN` | 12288 | stage A1 measured 15.7 KB free (min 13.0 KB) with the link task; the design's 18 KB would block every ARM until the link task is trimmed (open issue 3) |

### 3.8 The hover wait

When the photos are done (or, with photoN 0, after STABLE 1 s) HOVER keeps
`VERT_CTRL` / `VZ_CMD_TAKEOFF` and sets `waiting` (with `waitLeftMs`). The
wait ends:

- on a Mac `LAND`/`DISARM` (row 10, `LAND_CMD`) or `KILL`, the phone (row 11,
  slow LAND) and every other LAND rule;
- at `hoverWaitMs` after it started (row 22, `HOVER_TIMEOUT`), or at the 18 s
  air clock (row 18) if that comes first;
- in FLIGHT, as soon as the camera height estimate is not healthy once HOVER
  has lasted `hoverAccelMs` (row 21, `HOVER_NO_CAM`).

A `PHOTO` command during the wait starts another set, without extending the
wait. Stage C adds YAW and STEP here. `hoverWaitMs` and `hoverAccelMs` are
latched at ARM from NVS (`autowait`, `autoaccel`) and forced into their
ranges. The overall bound is therefore time-to-STABLE + photos +
`hoverWaitMs`, and the air clock never passes 18 s (revision 5; 10 s before).

## 4. Vertical controller (`vert_ctrl.h`)

### 4.1 Inputs, filters and the estimate

- Any non-finite field of `vzInput_t` gives CUT (`NAN_INPUT`), thrust 0, in that step.
- `dt` is clamped to 0.2..5 ms before use.
- `Vf`: first-order filter of `vbat`, tau `vfTauS` (0.5 s), started at the first
  step. Every K / V in this section uses `Vf` (a filtered V avoids sag feedback).
- `aRel` (`aLp20`): first-order 20 Hz low-pass of `accZ - g0`, alpha = dt / (dt + 1/(2 pi 20)).
  Reset to 0 when RAMP starts.
- Applied-thrust ring: `thrustApplied` pushed every step (32 entries).
- Estimate, from lift-off on: `a = g (accZ - g0 - b)`; `v += a dt`;
  `h += v dt`; `vHp += a dt - vHp dt / vHpTauS`, with `b = camBias` while the
  external estimate is usable (section 4.10) and `bAir` otherwise; while it is
  usable `v` and `vHp` also follow `camVz` with time constant `camTauS`
  (0.2 s). Before lift-off `v = h = vHp = 0`. At lift-off `v` and `h` are
  seeded from the lift-off CUSUM (below): starting them at 0 when the drone
  has already been rising for tens of ms made the prototype overshoot
  (median 0.47 vs 0.41 m).
- Tilt division: every command `u` becomes `u / max(gravZ, tiltDivMin)`
  before the ceilings are applied (tilt compensation is off in the
  controller, controller_pid.c:22).
- Final clamp of every phase: `0 <= thrust <= ceiling <= VZ_THRUST_ABS_MAX`.

### 4.2 Phases

**IDLE.** Thrust 0. `TAKEOFF` -> SPIN (`TAKEOFF`): `vRest = Vf`,
`spinThrust = min(spinThrustMax, spinThrustRel kPrior / Vf)` (about 17-20k).

**SPIN** (`spinS` = 0.8 s). Thrust = `spinThrust` (the stabilizer's soft-start
ramps it in over ~0.2 s). Over the last `g0WindowS` (0.5 s) collect accZ;
during that window abort if `tiltDeg > spinTiltMax` (`SPIN_TILT`) or
`gyroMaxDps > spinGyroMax` (`SPIN_GYRO`). At `spinS`: `g0` = mean, std =
sqrt(mean of squares - mean^2); abort if std > `spinStdMax` (`SPIN_NOISY`)
or `vRest - Vf > spinSagMax` (`SPIN_SAG`); else -> RAMP (`SPIN_DONE`). The
per-sortie g0 is mandatory: the rest baseline moved 970-1041 mg between
flights and 7-10 mg per flight within a boot; `baseZacc` is never used.

**RAMP.** Starts at `max(spinThrust, rampStartRel kPrior / Vf)`, rises
`rampRate` (13250/s) up to the plateau `rampTopRel kPrior / Vf` (1.00: hover
at the stored gain) and stays there until lift-off or `rampMaxS`. Ceiling
`min(capKhat kPrior / Vf, 58000)` (54000 until 2026-10-14: the RAMP's
`NO_LIFTOFF_CEIL` moved from ~2.97 V to ~2.77 V loaded at kPrior 160.4k).

- Why 13250/s (A2b): it is A2a's 12000/s x 165000 / 149500, the same
  fraction of the hover thrust per second, now that the prior is 165000.
  With an absolute 12000/s the RAMP needed 10 % longer from 0.5 to 1.0 x
  kPrior / Vf (1.98 instead of 1.79 s), and the plateau dwell inside
  `rampMaxS` shrank from about 0.7 to 0.5 s. Lift-offs hidden by a -10 mg
  residual were then detected within 250 ms in 36/100 runs instead of 85/100
  (42 never); with 13250/s, 80/100 (7 never). No limit was loosened.

- Why the plateau (review finding, critical): a negative in-flight offset
  hides the slow, almost unaccelerated rise out of ground effect, so the
  lift-off was detected over 1 s late (92/100 runs at -20 mg) while the ramp
  kept adding thrust to the drone in the air. T_lo then came out 1.39-1.43 x
  the thrust at the true lift-off and the sorties ran away to 1.25-1.91 m. At
  the plateau an undetected drone floats in ground effect instead (or, if the
  stored gain is too high for it, climbs slowly); RAMP then ends
  `NO_LIFTOFF_TIME`.
- The plateau is at 1.00, not lower: on today's drone the logged lift-off
  thrust reached 0.91 x the default kPrior / V (flights 19-21, whose own hover
  gain is 1.09-1.15 x 149.5k; lift-off was at most 0.84 x a flight's own
  hover). At 0.90 the sim failed to detect any lift-off for a drone 10 %
  above the prior with that lift-off ratio; at 1.00 it fails only at 15 %
  above the prior (open issue 7). Against the A2b prior (165000) that
  logged lift-off is 0.82 x kPrior / V.
- Lift-off detector, every step:
  `cusumV = max(0, cusumV + g (accZ - g0 - liftCusumK) dt)`;
  `cusumH = cusumV > 0 ? cusumH + cusumV dt : 0`; `cusumT` = the length of
  that run; `liftT = aRel > liftAcc ? liftT + dt : 0`.
  Lift-off when `liftT >= liftHoldS` (30 ms) or `cusumV >= liftCusumV` (0.02 m/s).
  The allowance `liftCusumK` (7 mg) is required: the design's
  `integral of max(0, accZ - g0)` grows on zero-mean noise and fired
  ~50 ms before the real lift-off in the prototype.
- Neither test subtracts `bAir` (review finding): on the ground the in-flight
  offset is absent (the accelerometer reads 1 g whatever the thrust), so a
  stored bAir below about -7 mg turned the ground drift positive and fired
  before the drone moved (41/50 sorties at -10 mg).
- On lift-off: if `tiltDeg > liftTiltMax` or `gyroMaxDps > liftGyroMax`
  -> abort (`LIFTOFF_ATTITUDE`; one leg up looks like lift-off). Otherwise:
  - `T_lo` = min(the applied thrust `liftLatencyS` (30 steps) before,
    `tLoMaxRel kPrior / Vf`). The bound is 0.90, so a late detection cannot
    raise the climb ceiling, Khat0 or the LAND reference.
  - `V_lo = Vf`; `Khat = clamp(max(kPrior, khat0LoRel T_lo V_lo))`.
  - `v = max(0, cusumV + g (liftCusumK - bAir) cusumT)` and
    `h = max(0, cusumH + g (liftCusumK - bAir) cusumT^2 / 2)`. This is the
    CUSUM run restated on the estimate's own offset.
  - `liftedOff`; `liftoffNow` for this step; -> CLIMB (`LIFTOFF`).
- Abort if the thrust reaches the ceiling (`NO_LIFTOFF_CEIL`, now only with
  a large tilt division) or RAMP lasts `rampMaxS` (`NO_LIFTOFF_TIME`).

**CLIMB.**
- `vref = min(vClimbMax, aClimb tPhase, max(-vDescMax, kH (hTarget - h)))`.
- `Kp = kpRel Khat / Vf`; `u = Khat / Vf + Kp (vref - v)`; `u = max(u, climbFloorRel Khat / Vf)`.
- `Khat += kiRel kpRel Khat (vref - v) dt`, except when the last step was at
  a ceiling or `thrustApplied < 0.98` x the last command (downstream limit)
  and `vref - v > 0` (anti-windup). Clamp `Khat` to
  `[khatMinRel, khatMaxRel] kPrior`; at the clamp for `khatClampS` -> LAND (`K_RANGE`).
- Ceiling `min(capLo T_lo, capKhat Khat / Vf, 58000)` (54000 until
  2026-10-14); `atCap` when `u`
  (tilt-divided) exceeds it; at a ceiling for `capHitClimbS` (0.7 s)
  -> LAND (`CLIMB_CAPPED`). The floor never wins over a ceiling.
- `h >= hTarget - holdBand` -> HOLD (`HOLD_REACHED`).
- `tPhase >= climbMaxS` (2.0 s up to a 0.40 m target; revision 5:
  `vzConfigSetHeight()` adds (hTarget - 0.40) / vClimbMax above it, 2.5 s at
  0.60 m, where the nominal climb takes 1.72 s against 1.27 s to 0.40 m)
  -> LAND (`CLIMB_TIMEOUT`). The design went to HOLD here; a climb this slow
  means the estimate reads low, the unsafe sign.
- Estimate guards (CLIMB and HOLD): `h < hMin`, `h > hMax` or `|v| > vAbsMax`
  -> LAND (`EST_RANGE`). They cannot see offset drift; they catch gross faults.
  `hMax` 1.0 m up to a 0.40 m target, 0.6 m over it; revision 5:
  `vzConfigSetHeight()` adds (hTarget - 0.40) above it, so it stays 0.6 m
  over the target (1.2 m at 0.60 m).

**HOLD.**
- For the first `holdLoopS` (0.5 s): the CLIMB law with `vref = 0`.
- Then, on every step where the external estimate is usable (section 4.10):
  the same law, still closed (Khat adapts), with `Kfrz = Khat (1 - kfrzBias)`
  kept current. The camera measures the velocity, so the integrator cannot
  wind up on an offset; with the frozen gain a 1-3 % gain error would move
  the drone at 3-8 cm/s (the damping is on a leaky velocity), 0.15-0.4 m over
  a 5 s wait.
- On every other step: the gain frozen at its latest `Kfrz = Khat (1 - kfrzBias)`
  (Khat does not move) and `u = Kfrz / Vf - kpRel (Kfrz / Vf) vHp`
  (open loop with damping, as in the vertical design).
- On the first step at `holdLoopS`, in either branch, Khat is saved once as
  the learning sample `kLearn` (`kFrozen` set). The drone is then at the target
  height and out of ground effect. A camera hold keeps adapting Khat and Kfrz
  after it, for control only (4.5).
- `tPhase >= holdNoCamS` (3.0 s) on a step without the external estimate, and
  inputs not from the sim -> LAND (`HOLD_NO_CAM`): a backstop under the FSM's
  `HOVER_NO_CAM` (2 s).
- Ceiling: the CLIMB ceiling (`capLo T_lo`, `capKhat Khat / Vf`, 58000); the
  user's rule that the vz loop may not exceed the climb cap in HOLD. Since
  2026-10-14 the `capLo` term is battery-scaled in HOLD,
  `capLo T_lo max(V_lo, Vf) / Vf` (`capLo K_lo` in K units; see
  autonomy-a2b.md "End of battery (2026-10-14)").
  At a ceiling for `capHitHoldS` -> LAND (`HOLD_CAPPED`).
- `T_hold`: exponential average of `thrustApplied`, tau `tHoldTauS`
  (0.25 s), started at HOLD entry with the first sample.
- `holdSteady` = `tPhase >= holdLoopS` and `|vHp| < steadyVMax` and
  `|u_before_tilt_division - T_hov| <= steadyBand T_hov` and not `atCap`, with
  `T_hov` = Khat / Vf on the closed loop, Kfrz / Vf when frozen.
- `tPhase >= holdMaxS` (17.0 s; 9.0 s under A2b's 10 s clock) -> LAND
  (`HOLD_TIMEOUT`, backstop: the FSM's wait and its 18 s air clock land
  first, as HOLD starts >= 1 s after the first RAMP step).

**LAND after lift-off** (accelerometer-independent).
- At entry: if HOLD was reached `T_ref = T_hold`, `K_ref = T_hold Vf`.
  Otherwise (LAND from CLIMB) `K_ref = kPrior` and, as built in A2b:
  - `T_climb`: exponential average of `thrustApplied` in CLIMB, tau
    `tHoldTauS` (0.25 s, as T_hold), seeded with T_lo at the lift-off and
    updated on every CLIMB step;
  - `climbFree`: CLIMB time since the lift-off or since the last step at a
    ceiling (0 while capped);
  - `T_ref = climbFree < tHoldTauS ? T_lo : clamp(T_climb, T_lo,
    landClimbRefRel kPrior / Vf)`, `landClimbRefRel` 1.00.

  `T_hov = K_ref / Vf`. A climb at a ceiling measures the ceiling, not the
  hover: a lift-off hidden by a negative offset climbs at the cap, so after a
  capped step, as in the first 0.25 s, T_ref stays T_lo (A2a's rule).
- Ceiling `T_ref max(landFloor, top - landRate max(0, tPhase - flat))`:
  0.95 T_ref falling 10 % of T_ref per second to 0.65 T_ref (reached at 3 s).
  A2b (hops of 2026-10-12): a LAND from HOLD with the camera estimate in use,
  started by the state machine (`CMD_LAND`, `CMD_MISSING`, `HOLD_TIMEOUT`;
  `landSlow`), starts at `landTop` 0.98 T_ref and holds it `landFlatS` 1.5 s
  (revision 5: up to a 0.40 m target; `vzConfigSetHeight()` adds
  (hTarget - 0.40) / |landVref| above it, at most 2.0 s: 2.0 s at 0.60 m)
  before it falls, so the loop can fly its slow descent. It stays below the
  hover thrust just measured, so a wrong estimate cannot hold the drone up.
  Every other start (from CLIMB, without the camera, after `EST_RANGE`,
  `K_RANGE` or `HOLD_CAPPED`) keeps `top = landStart`, `flat = 0`.
- Stall: from `VZ_STALL_ARM_S` (0.3 s) into LAND, `v > -stallV` (0.08 m/s)
  for `stallS` (0.1 s) makes the ceiling fall at `stallRate` (1.0 T_ref per
  second). It only lowers the thrust: on the ground it reaches the touchdown
  thrust in a fraction of a second, in the air (a float, a battery that
  recovers as the thrust drops) the drone then descends.
- Loop: `u = T_hov (1 + kpRel (vref - v))`, tilt-divided;
  `thrust = min(ceiling, max(landFloor T_ref, u))`. The loop can only take the
  thrust below the ceiling. `vref` is `landVref` (-0.30 m/s) down to
  `landSlowH` (0.20 m) and goes linearly to `landVnear` (-0.20 m/s) at h 0.
- The fast LAND (2026-10-14, docs/autonomy-a2b.md 17.4): a slow LAND
  (`landSlow`) started by `CMD_LAND` for a state machine reason on its allow
  list (`vzInput_t.landFastOk`, `auto_fsm.h autoFsmLandFastReason()`: the
  Mac's LAND, `AIR_TIME`, `HOVER_TIMEOUT`, `SORTIE_DONE`; never `VBAT_DIP`,
  `VBAT_LOW`, `BATT_LOW`, `LOOP_GAP`, a lost link or app, a guard or a
  contact), with the camera estimate in use, at a reference height (hTarget +
  hOff) of `landFastMinH` 1.00 m or more, `T_ref <= VZ_LAND_FAST_TREF_MAX`
  (53400, the MOVE refusal level) and `Vf >= VZ_LAND_FAST_VMIN` (3.15 V
  loaded), never blind. On steps with the camera estimate `vref += (landVfast -
  landVref) clamp((h - landFastH) / landFastBand, 0, 1)`: -0.50 m/s above
  0.85 m, back to -0.30 m/s by 0.60 m (a dropout reverts at once). Only to take
  that speed back, once the fast reference was flown (the blend at least
  halfway), while the estimate reads between `landBrakeH` 0.40 m and
  `landFastH + landFastBand` 0.85 m (never in the cruise above), in the flat
  part, and for at most `landBrakeMaxS` 0.5 s in all, the thrust may reach
  `landBrakeTop` 1.03 x T_ref instead of the ceiling (no camera needed for
  it: 0.5 s of propagated velocity). The ceiling itself (0.98 T_ref, monotone;
  the stall rule, `landFlatEff`, the timeouts) is unchanged. Every other LAND
  (another reason, CLIMB, blind, without the camera, the end of the battery:
  `BATT_LOW` lands at T_hold over 57000 or a battery under 3.00 V) is
  bit-identical to the LAND before.
- `tdThrustOk = thrust <= tdThrust T_hov`.
- Touchdown, force: `fMeas = aRel + 1`, `fPred = thrust Vf / K_ref`; if
  `tdThrustOk` and `fMeas > tdForce fPred` for `tdForceS` (200 ms) -> DONE
  (`TOUCHDOWN_FORCE`).
- Touchdown, jolt: `tdThrustOk` and `aRel > tdJoltG` -> DONE (`TOUCHDOWN_JOLT`),
  or (A2b) `aRel > tdNearJoltG` (0.30 g) below `tdNearH` (0.15 m estimated)
  at any thrust. The hops touched down under power at 0.8-1.0 m/s and
  bounced (2.1-3.7 g at first contact; LAND before contact reads at most
  +0.04 g, CLIMB +0.15 g); the thrust condition waited ~2 s for the ceiling.
- Contact latch `vzOutput_t.contact` (A2b): a bump over `tdNearJoltG`, or a
  stall below `tdNearH`, in LAND after lift-off; the glue stops the camera
  hold's lean on it (docs/autonomy-a2b.md 2.4).
- `tPhase >= VZ_LAND_MAX_S` (6 s) -> CUT (`LAND_TIMEOUT`).
- Blind LAND (`vzInput_t.blind`, the FSM's LAND `COLLISION`, user decision
  2026-10-13, 12.8): the thrust is the ceiling alone, no loop; the stall rule
  and the bump below `tdNearH` are off (both read the estimate the contact
  corrupted); from HOLD the flat part (`landSlow`) applies with or without
  the camera estimate, and T_ref is T_hold from before the last vertical
  move or bob if that is lower (`tHoldQ`: T_hold as of the last HOLD step
  with no vertical reference in the second before it; T_hold itself carries
  a motion's accelerations); from CLIMB T_ref is the CLIMB average also
  after a capped step (the cap is the contact's). The touchdown is the force
  or the jolt rule with `tdThrustOk`, which the ceiling reaches 2.3 s after
  its flat part; after a move down the flat part is the reference height's,
  not the take-off height's (12.8); the timeout is unchanged. Without
  `blind` every output is bit-identical (`vert_ctrl_identity.h`).
- Support probe (`vzInput_t.probe`, the FSM's LAND `COLLISION` and `PALM`,
  docs/autonomy-a2b.md 18.3): in LAND after lift-off the controller lowers its
  own thrust to 0.55 x T_hov at 2.0 x T_ref/s and declares support
  (`VZ_R_TOUCHDOWN_SUPPORT`, the motors stop in that loop) only when the
  specific force stays ~1 g there for 0.15 s; in free air it aborts within
  ~0.12 s (f < 0.86 g for 10 ms) and the LAND goes on. Without `probe` every
  output is bit-identical.
- Why 10 %/s and not the critique's 2 %/s: near the floor the drone floats in
  ground effect (hover/lift-off thrust ratio 1.28-1.57 in the logs), so it
  touches down only once the thrust falls to about 0.75 x hover. At 2 %/s the
  ceiling gets there after 10 s, so any landing in which the loop does not
  push the thrust down would end in the 6 s cut. At 10 %/s it gets there at
  2.0 s. In the prototype the median LAND took 2.4 s, and impact speed was
  driven by the height error, not by the ramp (0.42 m/s from 0.3 m without
  offset).
- Why 0.95 x the hold thrust is enough to start down even with an offset:
  the hold thrust is what kept the estimate level, so with an unsafe offset
  the drone was climbing at the start of LAND. The ceiling is then below
  hover within (T_hold / T_hover - 0.95) / 0.10 s whatever the estimate says.
- False touchdown in the air needs `fMeas > 1.3 fPred`: ground effect
  reaches 1.3 only in contact (geGain 0.33 at h = 0), and damping adds only
  0.07 g per m/s, so it cannot fire above a few cm unless the drone falls at
  more than about 3 m/s. That only happens after a runaway above 2 m, where
  a cut changes little.

- Free-fall margin: the FSM's free-fall trip is 0.4 g. The floor gives
  0.65 g when HOLD was reached, but only about 0.46 g when `T_ref = T_lo`
  (0.7 x hover). The loop reaches that floor only if the estimate says the
  drone is climbing faster than ~1.2 m/s, that is, in a runaway. A free-fall
  trip in LAND is always KILLED (a cut). This is accepted.
- LAND from CLIMB (a Mac or phone LAND mid-climb, `CLIMB_CAPPED`,
  `K_RANGE`). A2a started it at 0.95 x T_lo, below hover: with capLo 1.65 the
  drone descended at up to ~0.3 g and touched down at up to 1.45 m/s in the
  sweep of section 9.1 (open issue 1). The user's rule is "always slow land",
  for the phone during the climb too. With the CLIMB average (A2b), a phone
  stick 0.35-0.9 s after the lift-off touches down at 0.34-0.52 m/s
  (coupled, max over 20 seeds each; A2a 0.55-1.00 m/s), as slow as from HOLD.
  - The sweep of 9.1 #7b (capLo 1.65, no camera, LAND 0.05-1.70 s after the
    lift-off): impact p50 / p95 / max 0.42 / 0.97 / 1.41 m/s (A2a 0.64 /
    1.17 / 1.46); inside K +/-6 % and +/-10 mg, p95 0.84 / max 1.21 m/s
    (A2a 1.19 / 1.36).
  - **The residual corner.** The worst impacts and rises all come from a
    -20 mg offset with a stored gain 12 % too high (and K +6 % capped climbs
    at -10..0 mg). There the lift-off is detected about 1 s late, the drone
    is already 0.25-0.68 m up and still rising, and the accelerometer-only
    estimate does not see it. The worst impact barely moves (1.41 against
    A2a's 1.45-1.46 m/s), and the rise after the LAND reaches 0.48 m (A2a
    0.30 m). Within 0.25 s of the lift-off or of a capped step the LAND is
    still A2a's.
  - **The awake camera removes most of it.** With the observer feeding the
    controller (the A2b configuration) the same sweep gives impact p95 / max
    0.84 / 1.12 m/s and rise 0.085 / 0.16 m (healthy 0.3 s after the lift-off),
    0.86 / 1.20 m/s and 0.096 / 0.22 m (healthy 1.3 s after it).
  - A bound of 0.90 instead of 1.00 keeps the no-camera rise at 0.32 m, but
    makes the nominal climb landings faster and fails two of the new limits:
    the inner p95 (1.01 > 0.95 m/s, 9.1 #7b) and the phone in CLIMB (0.76 >
    0.65 m/s, 9.2b). With the camera in use it buys little (impact max 1.12 /
    1.20 m/s, as with 1.00). 1.00 is the default; `landClimbRefRel` is a
    config field (0.80..1.00), not an NVS key, so the first FLIGHT logs can
    move it.
  - The touchdown detector: A2a's T_lo armed it from the first step of every
    LAND from CLIMB. The CLIMB average arms it near the ground, except inside
    0.25 s, after a capped step and on a drone lighter than its stored gain
    (K_ref = kPrior is then 12 % high).

**LAND before lift-off** (abort, from SPIN or RAMP): thrust falls at
`abortRate` (60000 per s) from its last value; at 0 -> DONE (`ABORTED`).
`landReason` keeps why. If the ramp had reached its plateau, or (A2b) its
last thrust was at or above `abortSlowRel kPrior / Vf` (0.60), and the abort
is not `LIFTOFF_ATTITUDE`, the drone may be up without a detected lift-off.
The thrust then first follows LAND's ceiling with T_ref = that last RAMP
thrust (`landStart` falling `landRate` per second to `landFloor`, 3 s), then
falls at `abortRate`.

- From the plateau: in the sim, undetected lift-offs (-20 mg, drones up to
  12 % stronger than the stored gain) floated up to 0.32 m. A 60000/s abort
  dropped them at up to 1.87 m/s; this one at up to 0.64 m/s.
- Below the plateau (A2b, review R13, "always slow land"): a lift-off hidden
  by a negative offset can come before the plateau. The sim's true lift-off
  is about 0.71 x the true hover thrust, 0.62 x the prior at K -12 %, and at
  -10 mg 7/100 RAMPs never detect it. A phone stick in RAMP over such a
  lift-off (-10..-20 mg, K -12..+12 %, 175 sorties, 67 already up) touched
  down at up to 0.46 m/s with the plateau-only rule and 0.24 m/s with 0.60.
  0.80 would miss the lighter half of the user range.
- On the ground this only keeps the motors turning below lift-off thrust for
  up to about 3 s longer; the FSM ends LAND when the controller reaches DONE.
- A lift-off signature with a tilt or rate over the lift-off limits
  (`LIFTOFF_ATTITUDE`) is one leg up, not a hidden lift-off: it falls at
  `abortRate` at once from any RAMP thrust, the slow rule's range included,
  so a tipped drone is not held near its thrust for 3 s (9.1 #4b).

**DONE.** Thrust 0. Stays DONE until `vzInit()`; only CUT leaves it.

**CUT.** Thrust 0 in the step it is entered. Latched until `vzInit()`.

### 4.3 Commands by phase

| Phase \ cmd | NONE | TAKEOFF | LAND | CUT |
|---|---|---|---|---|
| IDLE | IDLE | SPIN | DONE (`CMD_LAND`) | CUT |
| SPIN, RAMP | abort (`CMD_MISSING`) | continue | abort (`CMD_LAND`) | CUT |
| CLIMB, HOLD | LAND (`CMD_MISSING`) | continue | LAND (`CMD_LAND`) | CUT |
| LAND | continue | continue (never climbs again) | continue | CUT |
| DONE | DONE | DONE | DONE | CUT |
| CUT | CUT | CUT | CUT | CUT |

`landReason` is set once, when LAND or the abort starts, and kept through
DONE and CUT. `reason` is the latest phase change.

### 4.4 Ceilings at a glance

| Phase | Ceiling (after tilt division) |
|---|---|
| SPIN | spinThrust (<= 20000) |
| RAMP | rises to the plateau rampTopRel kPrior / Vf (1.00) and stays; min(capKhat kPrior / Vf, 58000); reaching that aborts |
| CLIMB | min(capLo T_lo, capKhat Khat / Vf, 58000), with T_lo <= tLoMaxRel kPrior / Vf (0.90) |
| HOLD | min(capLo T_lo max(V_lo, Vf) / Vf, capKhat Khat / Vf, 58000) (2026-10-14: the capLo term battery-scaled, capLo K_lo in K units) |
| LAND | T_ref max(0.65, 0.95 - 0.10 t), never raised by the loop; from CLIMB T_ref <= landClimbRefRel kPrior / Vf (1.00); the fast LAND's brake (2026-10-14): up to 1.03 T_ref for <= 0.5 s while the estimate reads 0.40-0.85 m (autonomy-a2b.md 17.4) |
| abort | previous thrust - 60000/s; from the RAMP plateau or a RAMP thrust >= abortSlowRel kPrior / Vf (0.60), unless `LIFTOFF_ATTITUDE`, first T_ramp max(0.65, 0.95 - 0.10 t), then -60000/s |
| always | 58000 (`VZ_THRUST_ABS_MAX`; 54000 until 2026-10-14, when the feasibility critique's single motors at 53-62k p95 in hover made 57000 total clip at 65535). With the battery moved forward the hardest motor is FL at +3.5 %: 59.0k at the 57000 BATT_LOW level, 62.0k with the p99 correction, of 65280; 2000 below the 60000 motor stop of power_distribution_stock.c. See autonomy-a2b.md "End of battery (2026-10-14)" |

### 4.5 Learning

`vzLearnK()` after a qualifying sortie: `kPrior += 0.3 (kLearn - kPrior)`,
limited to +/-5 % of kPrior per sortie and to `VZ_K_MIN..MAX`
(120000..180000). Gain 0.3 is an average over about the last 5 sorties,
which the feasibility critique asked for (per-flight K medians 142-163k
against A2a's fixed 149.5k; the A2b default is 165000). The glue saves it to
NVS only in IDLE with the motors at 0 (section 8.7; as built:
`docs/autonomy-a2b.md` section 7). `bAir` keeps its NVS value (default
-2.3 mg) in A2; learning it needs a rest baseline before and after the sortie
and comes with stage B.

**The sample** `kLearn` is Khat on the first HOLD step at `holdLoopS`
(0.5 s into HOLD), taken once (4.2 HOLD). It is not the running Kfrz, for two
reasons:
- In a camera hold Kfrz follows Khat. A positive camera offset c sinks the
  drone into ground effect, and there Khat walks down to its 0.88 x kPrior
  clamp.
- Kfrz also carries the `kfrzBias` sink (0.5 %).

Learning Kfrz was a review finding. At c = +0.06 m/s it cut kPrior by about
3.7 % per sortie, to 0.87 x true after 4 sorties. From then on every sortie
landed `K_RANGE` in CLIMB, and learning never ran again.

**A sortie qualifies when** it ended DONE through touchdown, the sample was
taken (`kFrozen`, kLearn > 0), `tAir >= 1.5 s` at touchdown, and no step had
`fromSim` set. LAND must also have started by one of an allow list:
- `CMD_LAND`: every landing the state machine commands (Mac LAND, the phone,
  the end of the wait, `HOVER_NO_CAM`, battery, heartbeat and so on);
- `HOLD_NO_CAM`: the camera dropped out after the sample.

Every other start of LAND is refused:
- `K_RANGE`, `HOLD_CAPPED`, `EST_RANGE`: the model or the estimate left its
  range during the hold;
- `HOLD_TIMEOUT`, `CMD_MISSING`: the state machine always lands before these,
  so reaching one means the glue or the state machine failed;
- `CLIMB_*` and aborts: HOLD never reached `holdLoopS`, so there is no sample;
- a reason appended later, until someone reviews it here.

**What it learns (sim).** In `test_auto_sortie.c`, each chain is 10 sorties
with the real state machine and a warm camera, and each sortie starts from
the kPrior the previous one learned. The drone's true K is the A2b default
165000; the chains start there, except the stale-prior chain, which starts
from A2a's default 149500 (ratio 1.104, as A2a's chain):

| c (m/s) | Start | kPrior at the end (x true) | Learned | Controller LAND |
|---|---|---|---|---|
| +0.05 | 165000 | 0.975 (lowest 0.975) | 6/10 | `CMD_LAND` 6, `K_RANGE` 4 |
| +0.06 | 165000 | 1.000 | 0/10 | `K_RANGE` 10 |
| -0.05 | 165000 | 0.977 (lowest 0.976) | 10/10 | `CMD_LAND` 10 |
| 0 | 165000 | 0.974 | 10/10 | `CMD_LAND` 10 |
| +0.03 (stale prior) | 149500 | 0.906 -> 0.971 | 10/10 | `CMD_LAND` 10 |

- Khat at `holdLoopS` reads about 2.5 % below the true gain in the sim, so
  learning settles at about 0.975 x true for any c from -0.05 to
  +0.05 m/s. The old running-Kfrz sample sat at 0.996 at c = 0, but it
  ratcheted down whenever c was positive.
- At c = +0.06 m/s every sortie lands `K_RANGE` in HOLD. Nothing is learned
  and the stored gain stays.
- Without a camera the sample carries the residual offset's error. 40-sortie
  chains settle (they do not ratchet) at 0.967 x true at +3.7 mg, 0.96 at
  +8.1 mg and 0.90-0.92 at +20 mg (open issue 11).

### 4.6 Hard limits (`vzConfigValid`)

`vzInit()` with a config outside these goes straight to CUT (`BAD_CONFIG`):

| Field | Allowed |
|---|---|
| hTarget | 0.20..1.20 m (`VZ_H_TAKEOFF_MAX`; 0.20..0.40 until revision 5, 0.60 then, 1.20 from 2026-10-12 late). Since 2026-10-15 the hover goes to 2.00 m (`VZ_H_TARGET_MAX`) by moves up only, the take-off staying at 1.20 m: docs/autonomy-a2b.md 23.9 |
| kPrior | 120000..180000 |
| \|bAir\| | <= 0.030 g |
| capKhat | 1.00..1.25 |
| capLo | 1.00..2.00 |
| tLoMaxRel | 0.60..1.00 |
| rampTopRel | 0.80..1.10 |
| tiltDivMin | 0.90..1.00 |
| spinThrustMax | <= 25000 |
| rampRate | 1000..30000 /s |
| rampMaxS | (0, 3.0] s |
| climbMaxS | (0, `VZ_CLIMB_MAX_LIMIT_S`] s: 2.0 + (take-off cap - 0.40) / 0.40 + 0.5 = 4.5 s at the 1.20 m take-off cap (3.0 before the interactive hover) |
| holdMaxS | (0, 18.0] s (`VZ_HOLD_MAX_LIMIT_S`; 10.0 until revision 5; 80.0 for the interactive hover, 120.0 since 2026-10-14: docs/autonomy-a2b.md 18.1) |
| holdNoCamS | (0, 5.0] s (`VZ_HOLD_NO_CAM_MAX_S`) |
| landStart | 0.80..1.00 |
| landRate | 0.02..0.30 /s |
| landFloor | 0.40..landStart |
| tdThrust | 0.50..0.90 |
| tdForce | 1.10..2.00 |
| landClimbRefRel | 0.80..1.00 (A2b) |
| abortSlowRel | 0.40..1.00 (A2b) |
| landTop, landFlatS | 0.80..1.00, 0..`VZ_LAND_FLAT_LIMIT_S` s: 2.0 + (take-off cap - 0.60) / 0.30 + 0.5 = 4.5 s at the 1.20 m take-off cap (2.0 in A2b; a move up adds its height at 0.30 m/s to a LAND's own flat part) |
| landVfast, landFastH, landFastBand, landFastMinH | -0.60..landVref m/s, 0.30..1.00 m, 0.05..0.50 m, landFastH + landFastBand..1.50 m (2026-10-14) |
| landBrakeTop, landBrakeH, landBrakeMaxS | 1.00..1.05, landSlowH..landFastH m, 0..1.0 s (2026-10-14) |
| every other field | finite and > 0 (hMin, landVref: < 0) |

### 4.7 Sim plant (`vzSim*`)

Model per step (dt): `tr += (cmd - tr) dt / rotorTauS`;
`vbat = vRest - vSag tr`; `ge = 1 + geGain exp(-h / geHeight)`;
`f = tr vbat ge / kTrue - damp v`. On the ground (h = 0, onGround) nothing
moves until `f > 1` (then `liftT` is set once). In the air
`v += g (f - 1) dt`, `h += v dt`; at `h <= 0`: `impactV = max(impactV, -v)`,
`jolt = min(4, joltPerMs |v|)`, `h = v = 0`, onGround. Reading:
`accZ = (onGround ? 1 : f) + accOffsetG + (tr > 1000 ? accSpinG : 0) +
(!onGround ? accAirG : 0) + jolt + N(0, accNoiseG)`; the jolt decays with
tau 10 ms. Gaussian noise from the xorshift32 state (Box-Muller or a sum of
12 uniforms); the same seed gives the same run on every machine.

Defaults (`vzSimDefaultConfig`): kTrue `VZ_K_DEFAULT` (165000; 149500 in
A2a, and everything in the sim scales with it), vRest 3.90 V, vSag 9e-6 V
per unit (0.4 V at 45k, matching 3.9 V resting -> ~3.5 V in hover),
geGain 0.33 and geHeight 0.03 m (hover / lift-off thrust 1.33 in K units:
36 logged take-offs gave a median of 1.33 in K units and 1.41 in raw thrust,
range 1.28-1.57), rotorTauS 0.03, damp 0.07 g per m/s (momentum theory:
~0.14 m/s steady drift per 1 % thrust error), offsets 0, noise 0.03 g,
joltPerMs 0.5.

Camera observer model (off by default, `camWakeS` < 0). It stands in for the
observer of section 4.10 in the host tests, and in DRY and BENCH-sim, where
the A2b glue turns it on (warm, `camWakeS` 1.5 s, `camConvS` 0.3 s, c = 0;
`docs/autonomy-a2b.md` section 9) so DRY exercises the controller's camera
branch:

- The camera wakes `camWakeS` after the first nonzero command (the take-off
  window): 1.5 s warm, 3.0 s cold in the tests.
- The observer uses frames from the later of the wake and the first
  lift-off, and is healthy `camConvS` (0.3 s) after that, except in an
  optional loss window (`camLossS`, `camLossLenS`, from the lift-off).
- It reports `camVz = v + camVzOffset + n` and `camBias = accAirG +
  camBiasErrG`:
  - `camVzOffset` is the camera's slow velocity offset c. The study found
    \|c\| <= 0.011 m/s on rest-confirmed landings, a median ~0.03 m/s in flights
    16-21, and up to 0.15 m/s.
  - n is Gauss-Markov noise, `camVzNoise` 0.02 m/s rms, tau 0.1 s.
- Its random draws come from the plant's xorshift32 state, and only while it
  runs, so runs without a camera are unchanged.

### 4.8 Defaults (`vzDefaultConfig`)

| Group | Values |
|---|---|
| sortie | hTarget 0.30, kPrior 165000 (`VZ_K_DEFAULT`, NVS `vzK`), bAir -0.0023 |
| ceilings | capKhat 1.15, capLo 1.65 (lead decision, NVS `vzcaplo`; open issue 1), tLoMaxRel 0.90, tiltDivMin 0.94 |
| SPIN | spinS 0.8, spinThrustMax 20000, spinThrustRel 0.45, g0WindowS 0.5, spinStdMax 0.040, spinTiltMax 8, spinGyroMax 15, spinSagMax 0.35 |
| RAMP | rampStartRel 0.50, rampTopRel 1.00, rampRate 13250, rampMaxS 2.5, abortRate 60000, abortSlowRel 0.60, liftAcc 0.025, liftHoldS 0.030, liftCusumV 0.02, liftCusumK 0.007, liftTiltMax 15, liftGyroMax 250, liftLatencyS 0.030 |
| CLIMB/HOLD | vClimbMax 0.40, aClimb 1.0, kH 2.0, vDescMax 0.30, kpRel 0.35, kiRel 3.0, khatMinRel 0.88, khatMaxRel 1.12, khatClampS 0.3, khat0LoRel 1.2, climbFloorRel 0.75, holdBand 0.05, climbMaxS 2.0 (2.5 at 0.60 m, `vzConfigSetHeight`), capHitClimbS 0.7, capHitHoldS 0.5, holdLoopS 0.5, kfrzBias 0.005, holdMaxS 17.0, holdNoCamS 3.0, camTauS 0.2, steadyVMax 0.15, steadyBand 0.06, vHpTauS 1.0, vfTauS 0.5, tHoldTauS 0.25, hMin -0.3, hMax 1.0 (1.2 at 0.60 m, `vzConfigSetHeight`), vAbsMax 1.0 |
| LAND | landStart 0.95, landRate 0.10, landFloor 0.65, landVref -0.30, tdThrust 0.75, tdForce 1.3, tdForceS 0.2, tdJoltG 0.6, landClimbRefRel 1.00, landTop 0.98, landFlatS 1.5 (2.0 at 0.60 m, `vzConfigSetHeight`), landVnear -0.20, landSlowH 0.20, tdNearH 0.15, tdNearJoltG 0.30, stallV 0.08, stallS 0.10, stallRate 1.00; the fast LAND (2026-10-14): landVfast -0.50, landFastH 0.60, landFastBand 0.25, landFastMinH 1.00, landBrakeTop 1.03, landBrakeH 0.40, landBrakeMaxS 0.5 |

Gains come from the vertical design; the K clamp (+/-12 %) from the
feasibility critique. The implementer may tune CLIMB/HOLD gains to pass
section 9.1, but not the ceilings, LAND, touchdown or abort rules. Tuned in
A2a:

- liftCusumV / liftCusumK 0.04 / 10 mg -> 0.02 / 7 mg. With the design's
  values a -5 mg residual made 28 % of lift-offs ~1 s late.
- kH 1.2 -> 2.0. At 0.40 m, 17/50 -> 50/50 sorties finished their photos.
- kiRel 1.5 -> 3.0. A drone 3-6 % weaker than kPrior timed out in CLIMB.

Changed in revision 2, needing sign-off: rampTopRel (new), the slow abort
from the plateau, tLoMaxRel (new), and holdMaxS 3.0 -> 9.0 for the wait.

Changed in A2b (revision 4):
- kPrior 149500 -> 165000, the logged hover K at 40 kHz PWM (163k at
  3.4-3.6 V to 167k at 3.25-3.3 V; logs 2026-10-11v and 2026-10-12a). User
  decision.
- rampRate 12000 -> 13250 (4.2 RAMP): the same fraction of the hover thrust
  per second as validated at 149.5k. Needs sign-off.
- landClimbRefRel 1.00 (new, LAND from CLIMB) and abortSlowRel 0.60 (new,
  the slow RAMP abort), both 4.2. LAND and abort rule changes: they need
  sign-off.

Changed in revision 5 (user decision 2026-10-12 evening, 0.60 m and a 10 s
wait): holdMaxS 9.0 -> 17.0 for every target (the longer wait and the 18 s
air clock apply at any height), and above a 0.40 m target `vzConfigSetHeight()`
raises hMax and lengthens climbMaxS and landFlatS (1.2 m, 2.5 s and 2.0 s at
0.60 m); for targets up to 0.40 m every value but holdMaxS is unchanged.

Why the tests needed the ramp change: at 165k alone (12000/s),
`test_vert_ctrl` failed three checks (two true peaks 1 cm over their limits,
and the camera HOLD at -10 mg reached by 12/20 instead of >= 15; at 162k
`test_auto_sortie` also failed a LAND from CLIMB premise). Everything in the
sim scales with K except the absolute `rampRate` and the fixed `rampMaxS`:
the lift-offs were detected later and climbed further. With 13250/s every
check passes, and no limit was loosened.

### 4.9 What to expect (host tests on the sim plant)

**Sources.**
- `tools/tests/test_auto_sortie.c` runs the real state machine on the real
  controller: 200 uniform draws per row, plus the range's 8 corners with 100
  seeds each.
- `tools/tests/test_vert_ctrl.c` runs the controller with an emulation of the
  state machine's LAND rules: 300 uniform draws per row, plus the 8 corners
  with 50 seeds each.

**Ranges and conventions.**
- "User range": residual in-flight offset after bAir -20..+20 mg, K error
  +/-12 %, camera offset c +/-0.05 m/s. `test_vert_ctrl.c` also draws a
  camera offset error of +/-2 mg; the coupled test has none.
  - The uniform draws give the p95 and max.
  - The corners (offset x K error x c) have their own maxima. In
    `test_vert_ctrl.c` the camera offset error alternates +/-2 mg with the
    seed.
  - K -12 % is a drone that needs 12 % less thrust than kPrior says.
- "Realistic": the camera study's bias truth, +1.4 +/- 2.2 mg against g0,
  so +3.7 +/- 4.4 mg after the default bAir. Also K within +/-5 % and
  c +/-0.03 m/s. Its corners are -0.7/+8.1 mg, K +/-5 % and c +/-0.03 m/s.
- Heights are true heights.
- "LAND" is from LAND entry to touchdown, aborts excluded. Its p50 is from
  the uniform draws; its max, and the air maxima, include the corners.
- "Air" is the air clock at the state machine's LAND, from the first RAMP
  step. "True air" is the true airborne time at that point.
- Battery (A2b, review R8c): every coupled sortie rests at a battery drawn
  per seed in 4.00-4.15 V, one a FLIGHT sortie can arm with, and arms at the
  default `vmin` (4.00 V), so G3 is live in every run; no ARM was refused.
  A2a's 3.90 V stays as one stress row with G3 overridden (`vmin` 3.70).
- The coupled emulation reads the camera, hold, ground and battery-minimum
  inputs every 10th step and steps the FSM in IDLE as the glue does
  (`AUTONOMY_SLOW_EVERY`, `AUTONOMY_IDLE_STEP_EVERY`, `autoIdleSkip`).

| Case (real state machine, FLIGHT, target 0.30 m) | LAND by (uniform) | True peak p95 / max | Impact max | Corners: peak / impact max | LAND p50 / max | Air / true air at LAND, max |
|---|---|---|---|---|---|---|
| realistic, warm camera, wait 5 s | 200 `HOVER_TIMEOUT` | 0.44 / 0.54 m | 0.85 m/s | 0.57 m / 0.88 m/s | 2.38 / 2.58 s | 9.24 / 8.17 s |
| user range, warm camera, wait 5 s | 158 `HOVER_TIMEOUT`, 42 controller (15 RAMP aborts, `K_RANGE`) | 0.57 / 0.69 m | 1.02 m/s | 0.59 m / 1.07 m/s | 2.38 / 2.60 s | 9.48 / 8.67 s |
| user range, warm camera, wait 12 s (the maximum; revision 5) | 117 `HOVER_TIMEOUT`, 83 controller | 0.86 / 1.07 m | 0.96 m/s | 0.59 m / 1.07 m/s | 1.49 / 4.42 s | 16.48 / 15.67 s |
| user range, cold camera (3 s) | 139 `HOVER_TIMEOUT`, 61 controller | 0.63 / 0.86 m | 1.01 m/s | 0.87 m / 1.38 m/s | 1.73 / 4.24 s | 9.52 / 8.72 s |
| user range, camera lost 3 s after lift-off | 174 `HOVER_NO_CAM`, 26 controller | 0.36 / 0.51 m | 0.84 m/s | 0.59 m / 1.07 m/s | 2.38 / 2.62 s | 4.67 / 3.86 s |
| user range, no camera | 166 `HOVER_NO_CAM`, 34 controller | 0.99 / 1.84 m | 2.02 m/s | **2.08 m / 2.20 m/s** | 2.05 / 2.99 s | 4.77 / 3.97 s |
| user range, warm camera, wait 5 s, **stress 3.90 V** (G3 overridden) | 143 `HOVER_TIMEOUT`, 54 controller, 3 `NOT_STABLE` | 0.54 / 0.70 m | 0.93 m/s | 0.59 m / 1.07 m/s | 1.61 / 4.20 s | 9.97 / 8.65 s |

Revision 5 changed the rows above that a time limit ended (the 4.5 s pre-wait
clock is 6 s, the 10 s clock 18 s, the longest wait 12 s): the wait-8-s row
(152 `AIR_TIME` at the 10 s clock, peak 0.79 m) is now the wait-12-s row;
without a camera the sorties the 4.5 s clock landed (7) now hover to
`HOVER_NO_CAM` and the uniform peak rose 1.68 -> 1.84 m (corners unchanged);
the cold-camera and stress rows lost their `AIR_TIME` landings. Their LAND
columns are this build's (A2b printed other LAND percentiles).

- **The no-camera worst case is a corner,** not a uniform draw. At -20 mg
  and K -12 % it reaches 2.08 m / 2.20 m/s at c +0.05, and 1.96 m / 2.12 m/s
  at c -0.05. Without a camera c plays no part, so the two differ only in their
  seeds.
- Every other corner without a camera stays at or below 0.23 m.
- The corners at -20 mg, K +12 % never detect a lift-off (true peak 0.04 m).
  They end in a RAMP abort, 200 of the 800 corner sorties in each user-range
  row.
- No sortie in any row, corners included, ended KILLED.

The controller-only Monte Carlo agrees:

| Case | True peak p95 / max | Impact max | Corners: peak / impact max |
|---|---|---|---|
| user range, DRY timing (camera waived), no wait | 0.83 / 1.73 m | 1.95 m/s | **2.06 m / 2.18 m/s** (-20 mg, K -12 %, c +0.05) |
| user range, FLIGHT without a camera | 0.89 / 1.73 m | 1.94 m/s | **2.07 m / 2.18 m/s** (the same corner) |
| user range, FLIGHT, cold camera (3 s) | 0.60 / 0.90 m | 1.27 m/s | 0.84 m / 1.37 m/s |
| user range, FLIGHT, warm camera, wait 5 s | 0.58 / 0.72 m | 1.03 m/s | 0.58 m / 1.07 m/s |
| user range, FLIGHT, warm camera, wait 12 s (revision 5; A2b's 8 s: 0.62 / 0.74 m) | 0.88 / 1.03 m | 0.99 m/s | 0.58 m / 1.07 m/s |
| the same after 3 photos 1 s apart under `autoair` 8 s: the 18 s clock lands 174/300 | 0.96 / 1.14 m | 1.05 m/s | 0.58 m / 1.07 m/s |
| user range, FLIGHT, camera lost 3 s after lift-off | 0.38 / 0.55 m | 0.99 m/s | 0.58 m / 1.07 m/s |
| user range, warm camera, wait 5 s, c +/-0.10 m/s | 0.91 / 1.02 m | 1.32 m/s | 1.03 m / 1.34 m/s (+20 mg, K -12 %, c -0.10) |
| capLo 1.10, DRY or FLIGHT (no camera) | 0.12 / 0.59 m | 1.07 m/s | 0.70 m / 1.20 m/s (-20 mg, K -12 %, c +0.05) |

- Revision 2 missed the corners. `test_auto_sortie.c` never sampled them,
  and `test_vert_ctrl.c` ran each corner and edge midpoint with one seed
  only. Its maxima (1.63-1.67 m) came from interior points.
- With those single-seed points gone from the uniform draw, its maxima moved
  slightly (DRY 1.67 -> 1.72 m).
- With a 1.10 cap every sortie lands `CLIMB_CAPPED` or aborts. Before
  revision 2, the review's runaways went to 1.91 m (finding 1).
- The 0.70 m corner at the 1.10 cap is a strong drone rising at the RAMP
  plateau before its lift-off, hidden by -20 mg, is detected.
- Against A2a, the A2b build (165k, 13250/s, the CLIMB average, the slow
  RAMP abort, the battery draw and the input decimation) moved the rows'
  peaks by at most 0.06 m and their impacts by at most 0.08 m/s; the
  late-lift-off case below (no peak limit) rose by 0.10 m. No limit of 9.1
  or 9.2b was loosened.

**Revision 5: 0.60 m and the 10 s wait** (user decision 2026-10-12, evening;
`testHigh60` in `test_auto_sortie.c`). FLIGHT timing as the firmware runs
it (camera awake from the ARM), the Mac tool's default TAKEOFF (1 photo),
NVS `autowait` 10000, the default `autoair` (6000); heightCm 40 and 60 on the
same 200 draws each:

| Case | LAND by | Air at HOVER / at the wait, max | Air at the wait's LAND, max | True h at LAND p50, peak p95 / max | Impact p50 / p95 / max | True airborne p50 / max |
|---|---|---|---|---|---|---|
| 0.40 m, realistic | 198 `HOVER_TIMEOUT`, 2 `K_RANGE` | 2.56 / 4.37 s | 14.37 s | 0.42; 0.74 / 0.79 m | 0.28 / 0.57 / 0.66 m/s | 15.65 / 17.24 s |
| 0.60 m, realistic | 200 `HOVER_TIMEOUT` | 3.03 / 4.83 s | 14.83 s | 0.61; 0.94 / 1.00 m | 0.34 / 0.67 / 0.79 m/s | 16.81 / 18.41 s |
| 0.40 m, user range | 116 `HOVER_TIMEOUT`, 84 controller | 2.83 / 4.64 s | 14.64 s | 0.36; 0.94 / 1.06 m | 0.39 / 0.88 / 0.97 m/s | 14.49 / 18.00 s |
| 0.60 m, user range | 109 `HOVER_TIMEOUT`, 91 controller | 3.28 / 5.08 s | 15.08 s | 0.48; 1.14 / 1.27 m | 0.61 / 1.11 / 1.16 m/s | 14.55 / 18.90 s |
| 0.60 m, realistic, 3 photos 1 s apart, `autoair` 8000, `autowait` 12000 (50) | 50 `AIR_TIME` at 18.00 s | 3.00 / 7.51 s | 18.00 s | 0.61; 1.03 / 1.10 m | 0.34 / 0.78 / 0.84 m/s | 20.33 / 21.59 s |
| 0.40 m, user range, no camera | 141 `HOVER_NO_CAM`, 59 controller | 3.16 s / - | - | 0.26; 1.34 / 1.96 m (corners **2.29 m**) | 0.68 / 1.66 / 2.10 m/s (corners **2.32**) | 4.19 / 6.85 s |
| 0.60 m, user range, no camera | 121 `HOVER_NO_CAM`, 79 controller | 3.24 s / - | - | 0.35; 1.87 / 2.56 m (corners **2.96 m**) | 0.79 / 2.02 / 2.45 m/s (corners **2.63**) | 4.09 / 7.65 s |

- No sortie KILLED, none landed `CLIMB_TIMEOUT` (with the old 2.0 s the
  0.60 m climbs would have had 0.13-0.17 s to spare; 2.5 s gives the
  0.6-0.7 s that 0.40 m had), none reached the pre-wait clock at 6 s.
- **Without a camera the worst case is a corner,** as at 0.30 m (the user
  range's 8 corners, 100 seeds each, runMc's seeds; `testHigh60` checks
  the 0.60 m corners: never KILLED, never waits, LAND by the 6 s clock,
  touchdown within 6 s, true peak <= 3.25 m, impact <= 2.9 m/s). At -20 mg
  and K -12 % every sortie hovers until `HOVER_NO_CAM` and reaches
  **2.83 / 2.96 m true at 2.58 / 2.63 m/s** at 0.60 m (c -0.05 / +0.05:
  without a camera c plays no part, the two differ only in their seeds;
  three more seed sets gave 2.82-2.93 m, 2.57-2.61 m/s), against 2.18 /
  2.29 m at 2.25 / 2.32 m/s at 0.40 m and 2.08 m / 2.20 m/s at 0.30 m. Its
  LAND comes at an air clock of 4.9-5.0 s (0.40 m: ~4.5 s), the true
  airborne time is up to 7.75 s (0.40 m: 6.97 s). Over all 800 corner
  sorties at 0.60 m: 228 `HOVER_NO_CAM`, the rest the controller (`K_RANGE`,
  `NO_LIFTOFF_TIME`), LAND at an air clock <= 5.11 s, touchdown <= 3.45 s
  after it. With A2b's 4.5 s pre-wait clock the same corner at
  0.60 m reached 2.51 / 2.61 m, 2.44 / 2.48 m/s (about 30 % of its sorties
  landed `AIR_TIME`); at 0.40 m the clock change moves nothing. The other
  corners stay at or below 0.40 m (0.60 m: -20 mg, K +12 % never lifts off;
  +20 mg lands `K_RANGE` or `HOVER_NO_CAM`). In the realistic range without
  a camera the 0.60 m peak is 0.78 m (corners 0.86 m, 1.20 m/s).
- The wait starts >= 1.17 s inside the 6 s pre-wait limit in the realistic
  range (0.92 s in the user range), and a 10 s wait ends >= 2.9 s inside the
  18 s clock.
- The longer wait is what moves the drone: the camera's slow offset c
  (realistic +/-0.03, user range +/-0.05 m/s) acts for ~11 s instead of ~6:
  the realistic row at 0.30 m with the 5 s wait (first table) peaks at
  0.54 m (0.57 m with its corners), 0.40 m with 10 s at 0.79 m, 0.60 m with
  10 s at 1.00 m. The impact from 0.60 m is within
  0.06 / 0.10 / 0.13 m/s (p50 / p95 / max) of 0.40 m on the same draws.
- The phone stick during the climb to 0.60 m (~1.7 s): LAND `APP_INPUT`,
  never KILLED; impact max 0.35 / 0.46 / 0.65 / 0.83 / 0.94 m/s at 0.35 /
  0.70 / 1.00 / 1.30 / 1.60 s after the lift-off (at 0.30 m: <= 0.52).
  Late in the climb the drone is 0.4-0.55 m up and climbing; the LAND from
  CLIMB (its ceiling from 0.95 T_ref, falling 10 %/s) touches down harder.
- Through the real glue (`test_autonomy_glue_host` `margin-60`, plant gain
  1.00 / 1.03 x the stored one): the wait starts at air 4396 / 4596 ms and
  the 10 s wait lands `HOVER_TIMEOUT` at 14335 / 14498 ms; true peak
  0.65 / 0.59 m.
- Controller only (`test_vert_ctrl.c`, nominal, warm camera, 10 s wait):
  HOLD true h 0.42-0.62 m at 0.60 m (0.23-0.43 m at 0.40 m), impact p50 0.27
  / max 0.39 m/s (0.25 / 0.29); accelerometer only (DRY, no camera, no
  wait) the 0.60 m HOLD reaches 0.50-0.81 m and lands at up to 1.13 m/s
  (0.40 m: 0.32-0.55 m, 0.88 m/s).

**Late lift-off.** At a -20 mg residual with no camera, the coupled run never
reaches the air clock:
- 194/200 sorties land by the controller's `NO_LIFTOFF_TIME`: the lift-off
  is detected late or never.
- 6/200 land by `HOVER_NO_CAM`. The true lift-off to LAND is at most 3.38 s;
  the true peak at most 1.45 m.

The case bounds the airborne time of such sorties. It is not evidence for
the air clock: a lift-off clock gives the same bound.

**The air clock** has its own evidence:
- **`test_auto_fsm.c`**, fake controller: airMs is 1200 at HOVER, and a
  lift-off detected 2 s into the ramp still lands `AIR_TIME` at airMs 6000
  (4500 before revision 5).
- **`testAirClock` in `test_auto_sortie.c`**, real controller: 100 sorties,
  warm camera, residual 0/-5/-10 mg. Photos are 3 x 1 s, so the wait cannot
  start before the pre-wait limit (6 s; 4.5 s before revision 5).
  - `AIR_TIME` lands 100/100.
  - The detected lift-off came 0.10-0.25 s after the true one.
  - LAND comes at most 6.001 s after the first RAMP step, so the true
    lift-off to LAND is 5.06-5.10 s (4.501 s and 3.56-3.60 s with 4.5 s).
  - A copy of `auto_fsm.c` that starts the clock at the detected lift-off
    failed 100/100 (A2a): RAMP to LAND up to 5.75 s, true lift-off to LAND
    4.60-4.78 s.

Rule of thumb, unchanged without the camera: about +5 cm of true overshoot
per mg of residual offset in the unsafe direction over a 3.5 s sortie. The
ceilings bound the acceleration, not this drift. Time bounds it: the 6 s
pre-wait air clock (4.5 s before revision 5), `HOVER_NO_CAM` 2 s into the hover, the
accelerometer-independent LAND and the 6 s cut.

With the camera the drift during the wait is the camera's offset c:
- The drone moves at -c, about 0.25 m over a 5 s wait at
  \|c\| = 0.05 m/s, 0.5 m over the 10 s wait of revision 5.
- A positive c (the camera reads a climb) sinks it toward the floor. There
  the gain clamp lands it (`K_RANGE`) or it touches down. A `K_RANGE` sortie
  is not learned (4.5).

The real offset spread after the per-sortie g0, and the real c, are known
only from logs until stage B. The shadow observer of 4.10 now logs both on
every flight.

### 4.10 External vertical estimate (camera + accelerometer observer)

The camera height study (scratchpad `camheight/`, `observer.py` FINAL)
estimates the vertical velocity and the accelerometer offset from the flow
camera and the accelerometer. Its C port (`cam_vz_obs.c`, another A2 part)
runs in shadow mode on the drone now (below). From A2b it feeds four
`vzInput_t` fields every step, through the glue:

- `camHealthy`: the observer is healthy, meaning:
  - not faulted;
  - a camera update was accepted within the last 2 s;
  - the camera gain is tracked;
  - no CUSUM alarm is holding.
- `camVzValid`: `camVz` is this step's estimate.
- `camVz` (m/s, up).
- `camBias` (g): the in-flight world-up offset against this sortie's g0,
  i.e. what `bAir` stands for.

All are false/0 without the observer.

Glue mapping from `camVzObsOut_t` (cam_vz_obs.h):
- `camHealthy = healthy && airborne && !fault`;
- `camVzValid = !fault`;
- `camVz = v`;
- `camBias = biasG + 1 - g0`, with g0 the controller's `vzOutput_t.g0`. The
  observer's offset is against 1 g; the controller's is against this
  sortie's ground baseline.

The controller uses the observer only after lift-off, and only on a step
where `camHealthy` and `camVzValid` hold and both values are finite and
plausible (\|camVz\| <= 5 m/s, \|camBias\| <= 50 mg). A bad value is never
a CUT. On such a step:

- the estimate subtracts `camBias` instead of `bAir`;
- `v` and `vHp` follow `camVz` with `camTauS` (0.2 s), a complementary filter
  (the accelerometer gives the fast part, the camera the slow part);
- HOLD keeps its velocity loop closed (4.2 HOLD);
- `vzOutput_t.camHealthy` is true. The state machine's HOVER rule 21 reads it:
  in FLIGHT, a hover that has lasted `hoverAccelMs` (2 s) lands
  `HOVER_NO_CAM` on the first step without it.

The controller's own backstop is `HOLD_NO_CAM` after `holdNoCamS` (3 s) of
HOLD, on real inputs only.

The study's verdict, which these limits follow: a ~5 s capped wait at
~0.5 m is safe with the observer; 10-20 s is not, because the camera's slow
offset c is invisible on board.

**Shadow mode (built in now).** The observer runs on every flight, manual
ones included, and is logged, so c can be measured on today's drone before
the hover wait relies on it. Nothing in it commands the motors, and the
controller does not read it until A2b wires the two calls below.

The code:
- `utils/src/cam_vz_obs.c`: the observer.
- `utils/src/cam_vz_shadow.c`: the sortie, ground-flag and prior rules and the
  statistics. It is pure code, host-tested by `tools/tests/test_cam_vz_shadow.c`.
- `modules/src/cam_vz_glue.c`: the drone side.

Where it runs:
- `camVzGlueInit()` runs once from `stabilizerInit()`.
- `camVzGlueStep()` runs every stabilizer loop, after
  `motorGuardStabilizerAlive()` and before the flight recorder (section 2,
  step 7a). Its inputs:
  - the world-up specific force (`sensfusion6GetAccZ()`);
  - the thrust the motors got (the setpoint if any duty is nonzero, else 0);
  - the ground start and the soft-start's resting test.
  It only folds the sample into a 10 ms block (mean and peak force, peak
  gyro, flags) and pushes each finished block into a 16-block
  single-producer ring.
- A task `camvz` (priority 1, pinned to core 0, 4 KB stack in PSRAM) drains
  the ring: the observer runs once per block (100 Hz, the rate it was
  validated at on the logged flights; its `sigAcc` is a per-10-ms-block
  noise), then takes a new camera sample
  (`cameraFlowGetLatest()` with a new seq), then publishes a snapshot. Every
  getter (vert_ctrl's input, the recorder, the flight summary, USB) copies
  from that snapshot under a short spinlock; it is up to ~20 ms old.
- Why not in the stabilizer: on the drone the observer cost 40-70 us a loop
  and the loop fell from ~1000 to ~860-910 Hz. The ESP32-S3's instruction
  cache is 16 KB, shared by both cores, and the camera task's flow fit on
  core 1 keeps evicting it, so code run from flash misses on nearly every
  line (the host runs the same step in ~12 ns). For the same reason
  `camVzGlueStep()` is in IRAM (~9.5 us a sample from IRAM, ~20 us from
  flash). Measured with the task (20 s, on the bench): 995 loops/s with the
  observer on, 1002 with it off; 998 with `camvz batch 5`. Those figures are
  from a426962, where the filter stepped on only every other block (~50 Hz):
  a block is 10 IMU periods, 9990-9999 us on this MPU, and the shadow's gate
  wanted >= 10000 us since the last step. `CVS_OBS_BLOCK_TOL_US` (500 us)
  fixes it; re-measure the loop rate with the fix (the extra work is one
  `camVzObsImu()` + `camVzObsGet()`, ~1.6 KB of flash code, on every other
  task wake).
- In flight a426962 still ran ~962 loops/s (993-1000 in builds without the
  observer). From the camera clock in the recorder: ~3.5 % of the loops were
  lost steadily wherever frames flowed (only ~1.3 % during the take-off
  camera standby) and ~0.3 loops more in the 10-loop interval of each new
  frame. With frames flowing, the 1 kHz sensor read + stabilizer chain runs
  close to its 1 ms budget, and the task's flash code (~4 KB per 10 ms
  block) probably evicted the stabilizer's lines from the shared instruction
  cache 100 times a second (not proven: the motors-off second after each
  landing, frames skipped, also ran only 965-982 loops/s, so the app link
  and the state after a first sortie take part). The task's per-block path
  is in IRAM (`CAM_VZ_HOT`
  in cam_vz_obs.h, the libm helpers through linker_camvz.lf): +4.6 KB of
  IRAM, i.e. ~4.7 KB less internal heap. The ground-only phases and the
  flight statistics (refreshed at a new sortie or phase and every 10th
  block) stay in flash, and so does the per-frame update. Bench, 20 s
  each, no phone app (2026-10-12), loops/s with the observer off / on /
  `camvz air full`: without the IRAM path 995.5 / 988-990 / 987, with it
  999-1000 / 998 / 997-998 (the libm helpers serve the stabilizer too); mean
  IMU-to-output latency 1063-1173 us without, 968-1010 us with. The flight
  logs' motors-off seconds and frame clock will show whether flight follows.
- Not on core 1: the camera task (priority 2, pinned there) leaves core 1's
  idle task almost no time; the task unpinned there starved the USB console.
- Memory: the observer and its snapshot take about 3 KB, in PSRAM (internal
  RAM without PSRAM); the task's control block is internal. With no memory
  the observer is off and every getter reports "no observer".

The rules (`cam_vz_shadow.h`):
- **Sortie:** from a ground start to the next one. A ground start is thrust
  leaving zero after the soft-start's ground stop; stabilizer.c counts them.
  Each ground start resets the observer and starts its 1.5 s camera-wake
  clock.
- **Ground flag:** true before lift-off and after touchdown only, as the
  observer requires. The observer integrates h and v only while the flag is
  false; a flag that flickers in flight would reset them.
- **Lift-off** is the first of three rules:
  - thrust >= 30000 (`CVS_LIFT_THRUST`, the firmware's airborne level);
  - the accelerometer, with vert_ctrl's detector: a 20 Hz low-pass of
    a_z - g0 above 25 mg for 30 ms, or a CUSUM (7 mg allowance) reaching
    0.02 m/s. A fresh battery lifts off below 30000. This rule needs 20
    baseline samples.
  - the external hint.

  On a weak battery the thrust rule can fire while the drone still sits, and
  on an autonomy RAMP it can fire just before the true lift-off. On the ground
  with the motors spinning, a_z - g0 is -0.18 +/- 1.71 mg (study), so the
  early start costs little.
- **Touchdown:** the motors stop after lift-off. The touchdown h, v and offset
  are the values just before the first impact in the last 0.5 s before the
  stop, or the values at the stop. An impact is a_z > 1.5 g or
  max(\|gx\|, \|gy\|) > 200 dps, at least 0.2 s after lift-off (the study's
  contact rule).
- **Hop:** thrust coming back before the drone rests again stays in the same
  sortie. It lifts off again, with h and v restarted from 0.
- **Offset prior at lift-off:**
  - g0 is the median of the ground baseline ring: 10 ms means of the world-up
    specific force over the last 1.28 s, leaving out the last 5. The ring
    takes samples at rest with the motors off before the ground start, and on
    the ground with the motors running after it.
  - The prior is g0 - 1 g + `CAM_VZ_PRIOR_OFFSET_G` (+1.37 mg), sigma
    `CAM_VZ_PRIOR_SIGMA_G` (2.2 mg). That is the pair the study learned
    together. vert_ctrl's spin g0 goes with its own `bAir` (-2.3 mg); the
    `camBias` mapping above converts between them.
  - With fewer than 5 baseline samples the observer keeps its vague prior
    (sigma 30 mg).

The A2b hooks:
- `camVzGlueVzInput(vzOutput.g0, &in)` fills `camHealthy`, `camVzValid`,
  `camVz` and `camBias` with the mapping above. The values are always
  finite: without the observer, or with a non-finite g0, they are false/0.
- `camVzGlueSetLiftoffHint(vzOutput.liftedOff)` adds vert_ctrl's lift-off as
  the third rule. Pass only this sortie's value, false before its lift-off.
  The A2b glue zeroes the stored `vzOutput_t` at `vzReset` (section 2,
  step 2.3).

**In-flight gyro bias.** The rotation removal subtracts kyGy (99.8 px/rad)
times the gyro's pitch angle over each frame, so a pitch-gyro bias b reads as
a climb of kyGy * b / G (0.013 m/s at 0.4 deg/s, 0.029 m/s at 0.77 deg/s with
G ~50 px/m). The logs (take-offs of >= 6 s): +0.30 +/- 0.26 deg/s on files
1-15 (before the ground re-zero), -0.12..+0.15 on files 16-21 (16 of 18
within 0.1), and -0.07, +0.41, +0.77 (+0.37 on a 3.3 s flight 4) on the
2026-10-11 a426962 flights, where it
was present from lift-off and still read +0.87 at rest after flight 3, while
the re-zero reported no drift: not a fixed offset, so it is estimated per
flight. sensfusion6's Mahony feedback (`sensfusion6RateCorr`, Kp 0.4: time
constant 2.5 s; its integral term is ~400 s and does nothing within a flight)
settles at minus the bias. The glue averages it over each 10 ms block, the
shadow low-passes minus it (`CVS_GYRO_BIAS_TAU_S` 2 s, while the motors run,
clamped to 2 deg/s), starting at each ground start from the first block's
correction, which has settled at minus the bias while the drone rested (error
at the camera wake 0.12 deg/s rms against 0.20 from 0), and subtracts the estimate
times dt from every frame's gyro angles (roll and pitch; the accelerometer
cannot see a yaw bias). On the replays it lowered c by 0.0125 m/s (flight 2)
and 0.027 m/s (flight 3), as much as the whole-flight bias would; files 16-21,
the counters and the offset at +3 s / +5 s are unchanged. `camvz` prints the
estimate (`gyroBiasDps`).

Section 8.6 lists the logging: recorder columns, the per-flight block, and
the `camvz` command.

**Replay on the logs.** The replay mode of `test_cam_vz_shadow` runs the module
at 1 kHz, with frames on each new flowSeq, on the 50 take-offs of the 13
logged camera files. It needs the CSV directory and the study's
`bias_truth.csv` as arguments; `-g` feeds each row as one glue block instead
(IMU period 999.9 us, the way the drone calls it) and checks that the filter
steps on every block. Results (1 kHz / `-g`):
- the logged offset was within +/-3 mg of the study's truth in 92.9 / 92.9 %
  of take-offs at the camera wake +3 s, and 96.3 / 96.3 % at +5 s (before the
  gain re-anchor below: 96.4 % at +3 s with 1 kHz samples; a426962's ~50 Hz
  filter: 92.9 / 92.6 %);
- the observer was healthy for a median 66 / 65 % of the airborne time after
  the wake;
- the height estimate at touchdown had a median of 0.45 / 0.47 m and a worst
  case of 1.79 m.

That touchdown height, divided by the airborne time after the wake, estimates
c. `tools/espfly_diag.py flights` prints it as `drift`. It was 0.01-0.03 m/s
on flights 16-21. The figure needs a floor landing: if the drone sits on the
floor with thrust on for more than 0.5 s before the cut, that time counts
too. The touchdown values are those just before the first impact of the
impact cluster that reaches into the last 0.5 s before the motor stop (a
cluster runs on while its impacts come within 0.5 s of each other; before
2026-10-13 a cluster restarted 0.5 s after its first impact, and flight 1's
five in-air contacts gave the values after two of them). After an in-air
autonomy end (`TILT`, `RATE`, `FREEFALL`, `ATT_ERR`, and `COLLISION` while it
was a cut) h at touchdown is the height at the contact; after LAND
`COLLISION` (12.8) the motors run on to the floor and the observer's floor
values carry the contact's impulses: `flights` prints drift 'n/a (in-air
contact)' for all of them.

**Gain re-anchor (2026-10-13, section 12).** `sigGrel` treats a tracked G as
accurate to 20 % and white from frame to frame. Over a scene whose effective
distance changes with height (flight 2: floor and table parallax under a far
window, the tracked G 92 -> 67 -> 34 -> 27 px/m in a 1 m climb) the tracker
lags, the camera under-reads the climb, and the filter puts the difference
into the offset (+6-7 mg); when G catches up at the top the corrected speed
disagrees with the offset-inflated prediction and the CUSUM alarms (a false
`HOVER_NO_CAM`). `gainReanchorRel` (0.3; 0 in the plain config and the
fixture replay, which follows observer.py): a tracked G that moved more than
30 % from its anchor (the first tracked G after a reset or a yaw restart)
re-inflates sigma_v by `relockSigV` once, before that frame's CUSUM, gate and
update, and becomes the new anchor.

Two guards keep it to its purpose (2026-10-13 review). A re-inflated sigma_v
also lets in a camera speed that is wrong, and enlarges the CUSUM's
denominator: in the hover, a G moved by an object dragging the image (a 2-4
px/frame drag moved it 115 -> 154 px/m) or collapsing under a dead camera
while the drone really climbs (a frozen image during a 0.3 m/s climb: G
64 -> 47 -> 32 -> 23) re-anchored, and the observer stayed healthy with its
speed 0.3-0.7 m/s and its height 1-3 m wrong where the re-anchor off alarms
within ~1 s. So a re-anchor needs
- the **take-off climb window** (`CAM_VZ_REANCHOR_*`): the climb is seen
  once the speed estimate exceeds 0.15 m/s on a frame and ends at the first
  frame after that below 0.05 m/s (once per sortie: a bounce keeps it);
  re-anchors are allowed until 2.0 s after that end, never later than 6.0 s
  after the lift-off (vert_ctrl's longest climb, 4.0 s at the 1.20 m take-off
  cap, plus the 2.0 s: asserted in cam_vz_glue.c; the 2.00 m hover of
  2026-10-15 is reached by moves up, not by the climb). The
  end is kept as a 16-bit count of 50 ms in the state's padding. Flights 1
  and 2 of 2026-10-13: the climb ended 3.72 / 3.56 s after the lift-off, the
  re-anchors came 0.00-0.67 s later; in `testGainReanchor` the last one
  comes up to 1.95 s after the end (with 1.5 s one seed of 30 alarms; with
  2.5 s the dead-camera replays of 12.4 hide 10 faults instead of 2);
- **a frame inside the gate**: its innovation within `gateK` sigmas of the
  S before the re-inflation (a re-anchor never turns a gate rejection into
  an accepted update). This stops a drag that starts inside the window.
A CUSUM-sum guard was tried and dropped: the legitimate re-anchor comes while
the CUSUM is building (with max(CUSUM) < 1.5 required, 15 of 30 seeds of
`testGainReanchor` alarm; < 2.5: 5). After the window the observer is the
910cee0 one: a G change from a vertical MOVE or the LAND descent over such a
scene is not re-anchored (flight 2's third re-anchor, 4.3 s into LAND, is
gone: h at its last airborne row -0.60 m instead of -0.53, 1.44 off).

It never fired on the clean 2026-10-12 autonomy take-offs (bit-identical
outputs); on the hand-flown study take-offs it fires in 7 of 50 (13 without
the guards, counted the same way), offsets at wake + 3 s within 3 mg
unchanged (26 of 28). Flight 2 replayed: no alarm, healthy from before HOVER
to LAND, h at HOVER + 2 s 1.07 m (12.4). Cost: a few flops per frame in the
camvz task (the climb flags, one comparison, and S recomputed on a
re-anchor); the new fields sit in padding and after the counters, so the
10 ms block path keeps its offsets.

### 4.11 Accelerometer drift and the accelerometer-only part of a sortie

Measured on the logged take-offs (flight1-21.csv and
flight-2026-10-11v.csv; scratchpad `study2/accdrift/`):

- **The rest level drifts with motor time.** g0 rose +0.55 +/- 0.07 mg per
  second of motor-on time in the previous flight (69 consecutive take-off
  pairs) and saturated near +35..+41 mg after 35-50 s of flying. It fell
  about 2 mg over 1-4 min on the ground, and one session booted warm and
  started at -30 mg. Impacts above 6 g added nothing (-1.4 +/- 1.0 mg), the
  rest attitude moved < 1.5 deg within a session, and the battery voltage
  falls with motor time. Most of each rise appears on the ground in the
  10-80 s after the landing. The cause is heat from the motors and battery
  (inference: no temperature was logged; `status` now prints the MPU6050's
  `imuTempC`).
- **The per-sortie g0 removes the level.** What is left is the in-flight
  offset against the motors-on g0 just before the take-off (floor-pinned
  double integration): +1.4 +/- 2.2 mg over the study's 35 take-offs;
  +1.4, +1.0 and +7.9 mg on flights 2-4 of 2026-10-11v.
  - Take-offs at least 20 s after the previous flight, or the first of a
    boot: +1.4 +/- 1.1 mg, range -0.4..+3.6 (14).
  - Within 20 s: +1.6 +/- 2.8 mg, range -5.6..+7.9 (24). Hence
    `AUTO_SETTLE_MIN_MS`.
  - Within a flight the offset is steady: the replayed observer's offset
    moved +0.3 +/- 1.8 mg from wake + 3 s to touchdown (20 take-offs).
  - **Heat soak in 40-60 s hovers (2026-10-13).** The interactive hover
    flies 40-60 s, so the soak after it is long: flight 2 took off 60.0 s
    after flight 1's 41 s (the 20 s settle allowed it) with a ground
    baseline 16.5 mg above flight 1's, and its offset moved 5.8 mg in the
    air (b at +3 s / +5 s / touchdown +7.2 / +4.7 / +1.4 mg against g0),
    putting the observer's prior a few mg on the alarming side (12.1 RC4).
    Hence the soak rule below; widening the prior instead does not help
    (3-8 mg sigma still alarmed in the replay).
- **One source per comparison.** On the four 2026-10-11v take-offs the
  drone's own world-up specific force (`sensfusion6GetAccZ`: g0, vert_ctrl,
  the observer) read 3-5 mg below the one rebuilt from the recorder's 100 Hz
  acc and attitude: the shadow's g0 against the first recorded ground rows,
  and the observer's own offset estimates against the floor-pinned ones (the
  mechanism is not known). Offsets against a g0 from the same source agree
  (flight 4: +6.3 mg in the drone's, +7.9 mg in the recorder's). Never
  subtract the shadow's logged g0 from recorder-derived in-flight values.

**The camera's wake.** The take-off standby with `stabilizerNoteLiftoff()`
(8.4) wakes the camera 0.5 s after the thrust reaches 40000 or the lift-off.
On the RAMP that is 2.4-2.8 s after the first command, about 0.6 s after the
true lift-off, on a warm take-off. A cold take-off (the first after a power-on,
so the first sortie of every autonomy session, or one after a break) keeps the
camera asleep for at least `standbyColdMs` (3 s), about 1.0-1.2 s after the
true lift-off. The observer then needed 1.1-2.4 s of frames on the logged
flights. The 1.5 s camera with 0.3 s convergence of 4.7 and 4.9 is
optimistic. The camera measures velocity only: the height error built up on
the accelerometer stays for the whole hover. That is why A2b keeps the
camera awake through every autonomous FLIGHT and BENCH take-off (rules
below): the standby above applies to manual take-offs only.

**Time to the observer's first healthy sample** (A2b, review R21). In the
2026-10-12 logs the observer never dropped out after its first healthy
sample; that first sample came 1.3-5.4 s after the camera woke, a range
inflated by the manual take-off's standby. `test_auto_sortie.c`
(`testCameraConvergence`) keeps the camera awake from the first command and
draws the convergence from 1.3-5.4 s, counted from the lift-off (pessimistic),
with realistic offsets, 200 sorties:
- `autoaccel` 2000 ms (default): the photos are taken in 196/200, but
  `HOVER_NO_CAM` ends the 5 s wait 0.12-0.20 s after it starts in 103/200
  (every observer slower than 3.15 s); 90 wait it out.
- `autoaccel` 3000 ms (the NVS maximum): 59/200 are cut, after about 1.2 s.
- Never KILLED.

The first FLIGHT logs with the camera awake measure the real time to first
healthy; until then `HOVER_NO_CAM` may cut many waits short (open issue 5).

With the camera awake, the median true height at LAND entry is 0.20 m
against 0.14 m for the manual take-off's standby forced on (2.5 s wake,
`testCameraTiming`, both at a 4.05 V resting battery).

**Coupled sim** (real state machine and controller, kPrior 163000, kTrue =
kPrior x (1 +/- 3 %), c = 0, 300 draws from the measured offsets, true
heights; `study2/accdrift/src/sortie_drift.c`):

| Case | h at LAND p50 (p5) | Peak max | Not `HOVER_TIMEOUT` |
|---|---|---|---|
| settled, camera as the firmware wakes it (2.8 s, 1.3 s) | 0.12 (0.06) m | 0.30 m | 0 |
| settled, camera awake through the take-off | 0.20 (0.14) m | 0.31 m | 0 |
| within 20 s, firmware camera | 0.11 (0.04) m | 0.44 m | 4 `K_RANGE`, 1 `NOT_STABLE` |
| +7.9 mg, firmware camera / awake | 0.04 / 0.12 m | 0.23 / 0.25 m | 39 `K_RANGE` + 60 `NOT_STABLE` / 0 |
| settled, the tests' warm camera (1.5 s, 0.3 s) | 0.24 (0.20) m | 0.31 m | 0 |

- With c = +0.03 m/s the firmware camera timing lands 58/300 (settled) and
  94/300 (within 20 s) sorties `K_RANGE` in ground effect; the awake camera
  3-4/300.
- Without a camera (no texture: 2 of the 4 v flights; 12 of 50 replayed
  take-offs were never healthy) every sortie lands `HOVER_NO_CAM`:
  - settled offsets: peak 0.28 m, h at LAND 0.06 m (p50);
  - within 20 s: peak 0.49 m;
  - the lowest logged offset (-5.6 mg): 0.55 m, impact 0.89 m/s;
  - +7.9..+12 mg: peak 0.20-0.22 m, then the hover sinks to 2-4 cm in
    ground effect.
- The +/-20 mg user range of 4.9 stays the design envelope; no logged
  take-off came near its -20 mg corner.

**Rules (A2b).**
- G2 in FLIGHT needs 20 s at zero thrust (`AUTO_SETTLE_MIN_MS`). From
  2026-10-13 (needs the user's sign-off; its own commit): min(20 s + 1.5 x
  the latest flight's motor time, 90 s), `AUTO_SOAK_PER_MOTOR`,
  `AUTO_SOAK_MAX_MS`: 81.5 s after a 41 s flight, 42.5 s after 15 s, 27.5 s
  after a 5 s hop, 20 s for the first flight of a boot; DRY and BENCH keep
  3 s. The stabilizer records each flight's motor-on time (ground start to
  thrust 0, `stabilizerLastFlightMotorMs`); the refusal stays G2 `STILL`, and
  the Mac's daemon adds 'IMU heat soak: wait N s' from the last FLIGHT sortie
  it saw (`espfly_auto.py` `soak_hint`).
- `out.camNoStandby`: every FLIGHT and BENCH sortie, at any battery, from
  ARMED to HOVER, latched at ARM (user decision 2026-10-12; A2a needed a
  resting battery >= 4.00 V, `AUTO_CAM_AWAKE_VMIN`, now deleted). BENCH is
  the take-off rehearsal, so it runs the same capture path. DRY never sets
  it: no motors, no take-off window. The glue calls
  `cameraFlowSetTakeoffStandbySkip()` on every change (8.4). In FLIGHT the
  camera drift hold (camera flow + gyro) and the height observer then have
  frames from the lift-off on, and they stabilize the autonomous flight (the
  hold engages above 30000 thrust and 1 s airborne, so never in BENCH, whose
  thrust is at most 15000).
- The supply margin the standby protected is bought back by G3: `vmin`
  4.00 V (3.7).
  - Brownouts with the camera awake through a take-off (40 kHz + soft
    start): starts of 3.68-3.87 V; none at 3.91-3.98 V (2 take-offs).
  - With the standby: brownouts at <= 3.75 V; none at 3.91-4.14 V (5).
  - The autonomy RAMP draws less than those manual full-throttle take-offs.
  - Confirm on the bench (rail minimum of an autonomy-profile take-off with
    the camera awake) before stage B relies on it.
- `bAir` stays -2.3 mg: with it the residual is +3.7 mg on settled
  take-offs, so the drone sinks rather than climbs. Learning it from the
  hover's `camBias`, clamped to <= -0.5 mg, is a stage B option.

## 5. Kill, LAND and cut: who does what

| Source | Detected by | Effect | Motors | Back to manual |
|---|---|---|---|---|
| Phone stick input (thrust >= 1000, \|roll\| or \|pitch\| >= 2 deg) while active | phone hook -> FSM | ARMED/pre-spin: IDLE; airborne: LAND in place | off on touchdown; the phone never cuts | IDLE with the thrust lock: release the throttle to 0 first |
| Phone silent > 500 ms (app required) | FSM | as above | as above | as above |
| Mac heartbeat > 1 s, new session, LAND/DISARM | FSM | LAND (ARMED: IDLE) | off on touchdown | as above |
| Battery: in HOVER `BATT_LOW` (since 2026-10-14: 3.00 V or a hover thrust over 57000, 0.5 s), else 2.60 V/1 s (3.05 V until 2026-10-13), or the 2.55 V dip; loop gap > 20 ms, the air clock (6 s before the wait, 18 s in all), the end of the hover wait, a hover without a healthy camera height estimate, controller guards | FSM / vz | LAND | off on touchdown | as above |
| Mac KILL | link task: `motorsEmergencyStopNoWait()` if any duty > 0, then `killCount++`; FSM | KILLED | at once (driver STOP latched if they were spinning: reboot needed): within 50 ms of the packet with a live or blocked stabilizer, 80-90 ms (the motor watchdog) if the stabilizer busy-loops on core 0 (9.4) | reboot if latched |
| Tilt 30 deg/50 ms, rate 500 dps/30 ms, free fall 0.4 g/100 ms, tracking 10 deg/500 ms, NaN | FSM | KILLED (in LAND, a tilt or rate trip with the touchdown thrust condition and contact in the last 300 ms: touchdown) | thrust 0 at once; `stabilizerSetEmergencyStop()` | KILLED -> IDLE after 2 s and 1 s at rest |
| Stabilizer frozen 80 ms with any duty > 0 | motor watchdog (section 7) | driver STOP latched, red restart-lock blink; FSM later KILLED (`MOTOR_GUARD`) | within 80-90 ms (later if the guard's own callbacks come late, M4) | reboot |
| Panic, assert, interrupt watchdog | crash_log panic wrappers | pins 7/4/3/1 low | before the panic handler prints | reboot |
| LAND lasting 6 s | vz | CUT | at once | IDLE after rest |

## 6. Reset-lock (A2b)

User decision: after any reset autonomy is off and ARM is refused until a
battery power-on; fail closed.

- State lives in `.bss`, so every boot starts in IDLE with a new
  `bootNonce`; nothing is ever resumed.
- An `RTC_NOINIT_ATTR` record {magic, CRC-32, last state, last
  reason, mode, lock flag, 8 transitions {ms, from, to, reason, vzReason}} is
  rewritten (with its CRC) on every transition. The A2b interface fixes it
  as `autoRtcRecord_t` (auto_glue_core.h: magic `AUTO_RTC_MAGIC`, version 1,
  the previous boot's last state and a boot count) and makes the lock fail
  closed: the flag starts locked and only `autonomyInit()` clears it
  (autonomy.h), so a skipped or reordered init leaves FLIGHT and BENCH
  locked. `docs/autonomy-a2b.md` section 6 describes it as built.
- At boot: **`resetLocked = !(esp_reset_reason() == ESP_RST_POWERON && magic absent)`.**
  A battery power-on is the only boot where RTC memory lost power, so its
  magic is absent. A POWERON with the magic present is an EN/R-button reset
  or a supply dip that kept RTC memory; a present magic with a bad CRC is
  corruption; every other reset reason is a reset. All of them lock. (Read
  literally, "fail closed if the record is missing" would lock every boot;
  this reading keeps the intent.) Then a fresh valid record is written, with
  the previous one's last state kept for STATUS and the USB console.
- Unlock: unplug the battery and USB for at least 5 s, then plug the battery
  in (USB alone keeps RTC memory powered). There was no software unlock, not
  even over USB (critique), until the user decided on 2026-10-12 (evening)
  that `auto unlock` typed on the USB console is that deliberate physical act
  too (the battery is hard to unplug): it clears the reset-lock exactly as a
  battery power-on does, only in IDLE with the motors stopped, leaves every
  other lock as it is, and is reachable from the USB cable only, never over
  Wi-Fi, CRTP or the autonomy link (docs/autonomy-a2b.md section 6). DRY is
  exempt (lead decision): G1 does not check the reset-lock in DRY, which
  cannot spin a motor (`thrustMax` 0 at both barriers of section 2).
- The Mac daemon treats a `bootNonce` change as "mission aborted" and never
  re-arms on its own.

## 7. Motor watchdog (`motor_guard.h`)

### 7.1 Rules

- **M1 Timer.** `motorGuardInit()` from `stabilizerInit()` after
  `powerDistributionInit()`, before the stabilizer task is created. One
  periodic `esp_timer` (task dispatch; the esp_timer task runs at priority 22
  on core 0, above the stabilizer's 5) every `MOTOR_GUARD_PERIOD_US` (5 ms).
- **M2 Alive.** `motorGuardStabilizerAlive()` once per loop right after the
  motor write (section 2, step 7): one atomic increment.
- **M3 Decision.** Each callback calls `motorGuardCoreStep(core, nowUs, alive,
  motorsOn, enabled)` (rules in the header). `motorsOn` = any of the driver's
  last-written duties is nonzero, read without the driver mutex
  (`motorsAnyDutyNonZero()`, M6).
- **M4 Late callbacks.** A callback that comes more than `MOTOR_GUARD_LATE_US`
  (15 ms) after the previous one is late:
  - Its gap is not counted (`lateCalls++`, `MG_LATE`), and the stall observed
    before it is kept.
  - `stallUs` adds up on-time gaps only.
  - When the guard itself was held up, the stall was system-wide, not a frozen
    loop, and the stabilizer resumes within 1-2 ms of it. Causes: a flash
    erase with the cache off, a long critical section, a priority >= 23 task
    on core 0 (cam_task printing, Wi-Fi, IPC) or a debugger.
  - The window restarts on progress (the alive counter moved), motors off,
    the guard switched off, the first call, or a clock that goes backwards.
    A backwards clock also counts as late. A late call with progress or with
    the motors off restarts the window and returns `MG_LATE`.
  - So a system-wide stall cannot cut, and a guard that is often late still
    trips on a real freeze.
  - The timer skips missed periods (`skip_unhandled_events`), so a stall gives
    one late call, not a burst.

  The old rule restarted the window on every late call. A guard held up for
  more than 15 ms at least once every 80 ms then never tripped (review
  finding).
- **M5 Trip.** No progress for `MOTOR_GUARD_FREEZE_US` (80 ms) of on-time
  observation with `motorsOn` trips the guard. A trip does four things:
  - calls `motorsEmergencyStopNoWait()`;
  - latches `tripped`, with `trips = 1` and `tripUptimeMs`;
  - writes one line to the crash-log ring with `crashLogNote()`, ending
    "motors off until reboot". The line is appended under the ring's spinlock
    and never printed; the callback does no ESP_LOG (M6).
  - runs the red restart-lock blink (`ledseqRun(&seq_restartLock)`, a queue
    send with no wait), which already means "replug the battery".

  Flying again needs a reboot. A battery replug or the reset button always
  clears it. A software reboot clears it too, but with the restart lock on
  (`plock`, default on) thrust stays 0 until a replug. Autonomy then has the
  reset-lock. Nothing about a trip is stored in NVS.
- **M6 Driver additions (motors.c/.h).**
  - `bool motorsAnyDutyNonZero(void)`: reads `motor_ratios[]` (aligned 32-bit, no mutex).
  - `void motorsEmergencyStopNoWait(void)`: first `motorsPinsLowNoLock()`
    (the M7 register writes: the pins leave LEDC for the GPIO matrix and go
    low), then `xSemaphoreTake(mutex, 0)`; on success set `stopped`,
    `ledc_stop()` the channels and zero `motor_ratios[]`, give; otherwise set
    `stopped` and `faulted` and zero `motor_ratios[]` without the mutex. Once
    rerouted, a stabilizer still holding the mutex cannot drive the pins
    again, and `readyLocked()` refuses every later write (`stopped` and
    `faulted` are volatile for this). As built: no `gpio_config()` on this
    path, unlike `motorsEmergencyStop()`, because it logs four ESP_LOGI lines
    that can spin up to 50 ms each burst on a full USB-Serial-JTAG FIFO
    (vfs_usb_serial_jtag.c, `TX_FLUSH_TIMEOUT_US`) in the 3.5 KB esp_timer
    stack.
  - The existing `motorsEmergencyStop()` (portMAX_DELAY) must never be called from the timer.
- **M7 Panic path.** `motorGuardPanicPinsLow()` (IRAM): `GPIO_OUT_W1TC_REG`
  for pins 1/3/4/7, `esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX,
  false, false)` and `GPIO_ENABLE_W1TS_REG` for each. Called first in
  `__wrap_esp_panic_handler` and `__wrap_esp_system_abort` (crash_log.c:43),
  so it covers exceptions, asserts and the 300 ms interrupt watchdog. The
  task watchdog does not panic (`CONFIG_ESP_TASK_WDT_PANIC` unset); the
  guard covers a starved stabilizer.
- **M8 Switch.** NVS `mguard` (u8, default 1), USB `mguard on|off|status`;
  `status` ends with `mguard=on|off|tripped`. Off restarts the window and
  never clears a trip. `motorGuardEnabled()` is false if the timer could not
  be started. Autonomy ARM needs it on (G1). It applies to manual flight too.
- **M9 Mac KILL.** The link task calls `motorsEmergencyStopNoWait()` itself
  when `motorsAnyDutyNonZero()` (so a KILL works while the loop is frozen,
  and the link task never blocks on the driver mutex), then increments
  `killCount`.

### 7.2 Why 80 ms

- Normal: per-flight loop-gap maxima 1.47-3.23 ms over 57 logged flights
  (`flights-*.txt`, `gapMaxUs`); since-boot maxima 2.9-6.8 ms in boots
  without the event below.
- One-offs: 55 ms (first diagnostic session) and 53.0 ms (round T2, boot
  with `maxGapUs=53041`), both outside any recorded flight, cause unproven.
  The likeliest is an NVS sector erase (cache off on both cores for tens of
  ms), which also stops the guard (M4). Since that is not proven, the
  threshold sits above both with a 1.5x margin: a false cut in flight is a
  crash.
- Cost of waiting: a frozen loop holds the last duties. At the climb ceiling
  the drone gains under 3 cm in 80 ms. Attitude drifts open-loop from a duty
  that was balancing it a moment before, so a few degrees. The fall after the
  cut is the same whether it comes at 20 or 80 ms.
- A failed I2C read costs at most ~11 ms (5-tick mutex + 5-tick command,
  i2cdev_esp32.c:87/101), so single bus hiccups stay far below the threshold.
- Residual risk: the guard is blind only if no callback is ever on time.
  Late calls only pause the count (M4). `lateCalls`, `maxCallGapUs` and
  `maxStallUs` show this on the bench.
- The worst legitimate stabilizer stall is about 61 ms: a 50 ms console print
  on a full USB-Serial-JTAG FIFO plus an 11 ms failed IMU read. With a 45 ms
  flash erase inside it, the new counting reaches about 65 ms.
  `test_motor_guard` checks that this does not trip.

### 7.3 Fault injection (USB, BENCH only: refused unless 1 <= gcap <= 20000)

- `mguard test hang <ms>` (<= 500): the stabilizer busy-waits that long at the top of its loop (`stabilizerFaultHang()`).
- `mguard test imu <ms>` (<= 500): the sensors task stops giving `dataReady` for that many samples, ~1 per ms (`sensorsMpu6050FaultHoldData()`).
- `mguard test panic`: `abort()`.
- `hang` and `imu` are also refused while the guard is off or tripped.

## 8. Integration rules for A2b

### 8.1 Setpoint while active

Unless the action is `NONE`: `memset` 0; `mode.x/y/z = modeDisable`;
`mode.roll = mode.pitch = modeAbs` with `attitude.roll = attitude.pitch =
0.0f` exactly (the camera hold engages only on exact zeros,
controller_pid.c:253-260, camera_flow.c:288); `mode.yaw = modeAbs`,
`attitude.yaw = yawHold` (captured on `yawCapture`; `modeAbs` bypasses yaw
lock, controller_pid.c:220-225); thrust per section 2.5; `timestamp` = now.

On `setEmergencyStop`: `stabilizerSetEmergencyStop()`, and remember it. On
`clearEmergencyStop`: `stabilizerResetEmergencyStop()` only if we set it.
KILLED holds thrust 0 itself, because other code can clear the flag
(crtp_localization_service.c:277).

### 8.2 Phone hook

In `crtpCommanderRpytDecodeSetpoint()` after the stats block
(crtp_commander_rpyt.c:241), on the bounded values:
`autonomyNoteAppPacket(roll, pitch, rawThrust)` (autonomy.h: no time
argument; the hook stamps the FSM's clock itself, review R1; as built:
`docs/autonomy-a2b.md` section 4). Lock-free:
`lastAppMs`; input = `rawThrust >= 1000 || |roll| >= 2 || |pitch| >= 2` (yaw
ignored) -> `lastInputMs`, atomic `appInputCount++`; gesture: `|roll| >= 10`
sets `pushed`, then `|roll| < 2` with `pushed` stamps `gestureMs`; the last
`rawThrust < 1000` -> `appThrustZero`. Invalid packets are not input. The
hook never changes the decoded setpoint.

"Always slow land" (user decision): any phone input while airborne is LAND
`APP_INPUT` (3.2 row 11), slow from HOVER, from CLIMB (the CLIMB average) and
from the RAMP (the slow abort), with the motors off at touchdown. The phone
never cuts.

### 8.3 Link messages (types reserved in docs/autonomy-link.md)

| Type | Payload (little-endian) | Notes |
|---|---|---|
| 0x10 ARM | u8 mode, u8 0, u16 airMaxMs | ACK detail = armToken (OK) or guard bitmap (GUARDS) |
| 0x11 TAKEOFF | u32 armToken, u16 heightCm, u8 photoN, u8 0, u16 photoGapMs, u16 actWaitMs | actWaitMs must be 0 (the hover wait is NVS `autowait`). The interactive hover: actWaitMs 1000..60000; since 2026-10-14 u24 with byte 7 as its bits 16-23, 1000..120000 (docs/autonomy-a2b.md 18.2) |
| 0x12 LAND | empty | never refused |
| 0x13 KILL | empty | accepted with a valid tag alone: any session, any seq (a replayed KILL is fail-safe); M9 |
| 0x14 DISARM | empty | |
| 0x20 PHOTO (A1) | u8 n, u8 0, u16 gapMs | while autonomy is in HOVER, the glue gives it to the FSM as `AUTO_CMD_PHOTO` (3.3); in IDLE it stays the A1 photo command |

The design: commands go through a single-slot mailbox under a portMUX; KILL
also bumps `killCount` so it cannot be lost. The ACK is sent when the FSM
has processed the command (`cmdDone`). STATUS byte 7 carries `autoState_t`.
A new AUTO_STATUS message carries:

- the state machine: reason, vzReason/vzLandReason, failedGuards, landTrip,
  airMs (the air clock), stableMs, `waiting` and waitLeftMs, and the trip
  timers' state;
- the controller: vzPhase, hCm, vzCms, camHealthy, thrust, T_lo, T_hold.

As built, the mailbox is the step-0 `auto_mailbox.h` (also the flash-write
interlock, 8.7), and `docs/autonomy-link.md` is the reference for
everything on the wire: the asynchronous command path, the link's own `BUSY`
answers and their detail kinds, the ACK's state byte, PHOTO routing and the
PHOTO ACK's detail, AUTO_STATUS (0x84, `auto_proto.h`) and its push rule,
and KILL with its rate budget. This document fixes only what the state
machine does with a command it takes and the result it publishes (3.3); a
`BUSY` from the link itself is not a state machine result.

The photos of a sortie are Claude's input for its next command, so they are
pushed during the hover wait, not held until IDLE. The A2b glue must make sure
the take-off window's photo and Wi-Fi pause (section 8.4) has ended by then
(open issue 8). The window ends 1.5 s after the detected lift-off (and at
least 3 s after the ground start, 8.4) and STABLE needs HOLD plus 1 s, so
the take-off photos are captured in HOVER, after the window. How the link
pushes them during the wait is in `docs/autonomy-link.md` (Photos).

### 8.4 Take-off window

The stabilizer opens its take-off window (camera standby, low Wi-Fi TX, the
photo-push pause) when thrust leaves zero, and ends it 1.5 s after thrust
first reaches `TAKEOFF_LIFT_THRUST` (40000), no earlier than 3 s and no later
than 8 s after the ground start (stabilizer.c, `stabilizerTakeoffPhase`). An
autonomous take-off must not wait for 40000. The RAMP lifts the drone off at
about 0.7 x hover, 30-35k (hover with the 165k prior is 42-49k at a loaded
3.4-3.9 V), and reaches 40000 only near its plateau (kPrior / Vf, 40-42k at a
Vf of 3.9-4.1 V), after the lift-off. The climb and the hover sit near 40000
on a fresh battery (165k / 4.0 V loaded = 41k), and under it with a lower
stored gain (A2a's 149.5k / 3.75 V = 39.9k never reached it). Waiting for
40000 would end the window late, or only at its 8 s maximum, and hold the
low Wi-Fi TX and the photo-push pause into the hover. A2b adds
`stabilizerNoteLiftoff()` (stabilizer.h), called on the controller's
`vzOutput.liftoffNow` in FLIGHT and BENCH: the window's lift-off is then the
detected one (as built: `docs/autonomy-a2b.md` section 5). DRY opens no
window (no thrust, no ground start).

In every FLIGHT and BENCH sortie, at any battery, the state machine sets
`out.camNoStandby` (ARMED to HOVER, latched at ARM); the glue passes it to
`cameraFlowSetTakeoffStandbySkip()`, so the camera stays awake through that
take-off (section 4.11; as built: `docs/autonomy-a2b.md` section 5). DRY
never sets it. The low Wi-Fi TX power is unchanged.

### 8.5 NVS keys (imu_level_cal_store pattern; USB console only, never Wi-Fi)

| Key | Type | Default | Range |
|---|---|---|---|
| `autoon` | u8 | 0 | 0/1 |
| `appreq` | u8 | 1 | 0/1 |
| `autogest` | u8 | 1 | 0/1 (the per-sortie consent gesture, G10) |
| `vmin` | u16 mV | 4000 (`AUTO_VMIN_DEFAULT`) | 3700..4200 (G3, FLIGHT and BENCH; DRY exempt) |
| `autoair` | u16 ms | 6000 | 2000..8000 (the air clock until the hover wait; 4500, 2000..6000 until revision 5) |
| `autowait` | u16 ms | 5000 | 0..12000 (0: land right after the photos; at most 8000 until revision 5) |
| `autoaccel` | u16 ms | 2000 | 0..3000 (HOVER on the accelerometer alone, then the camera must be healthy) |
| `vzK` | u32 | 165000 (`VZ_K_DEFAULT`) | 120000..180000 |
| `vzBair` | i16, 0.1 mg | -23 | -300..300 |
| `vzcaplo` | u16, x1000 | 1650 | 1000..2000 (open issue 1) |
| `mguard` | u8 | 1 | 0/1 |

The A2b interface derives the defaults from the header constants
(`AUTO_VMIN_DEFAULT`, `VZ_K_DEFAULT`; the settings table in
`auto_glue_core.h`), so the table above and the build cannot drift. The NVS
table as built, the USB `auto get|set|reset` commands and their gating are
in `docs/autonomy-a2b.md` section 7.

### 8.6 Logging

**A2b adds:**
- autonomy columns at the end of the 100 Hz `flightRecord_t`: the state,
  the latest reason, the controller's phase, height and vertical speed
  estimates and thrust, and the `AUTO_STF_*` flags (auto_proto.h);
- `flightSummary_t.pad` becomes `autoReason` and `FSUM_END_AUTO = 4`
  (autonomy flew it and its thrust went to 0).

The layout, the record size and the rules as built are in
`docs/autonomy-a2b.md` section 8.

**Built in now: the camera observer in shadow mode (section 4.10).**

100 Hz recorder: six columns at the end of `flightRecord_t`.

| Column | Type | Content |
|---|---|---|
| `cvH` | i16, mm | height estimate since lift-off (0 on the ground) |
| `cvV` | i16, mm/s | vertical speed estimate, up |
| `cvBias` | i16, 0.01 mg | accelerometer world-up offset against 1 g |
| `cvGain` | u16, 0.1 px/m | camera gain G = f / D the next update would use (0: none) |
| `cvFlags` | u8 | bit0 airborne, bit1 healthy, bit2 gain tracked, bit3 fault; bits 4-7 the latest new frame's `camVzObsDecision_t` |
| `cvSigV` | u8, mm/s | sigma of the speed estimate, capped at 255 |

- A record is now 93 bytes (83 before), pinned by a `_Static_assert` in
  flight_recorder.c.
- The ring is 6000 records (60 s), 558 KB in PSRAM. That is 60 KB more than
  before; the drone has PSRAM (`CONFIG_SPIRAM`, octal).
- Without PSRAM it falls back to 1500 records (15 s), 140 KB of internal RAM.
- `espfly_diag.py` writes the new columns as `cvH_mm`, `cvV_mms`,
  `cvBias_cmg`, `cvGain_dpm`, `cvFlags` and `cvSigV_mms`. It still reads the
  46-field records of older firmware.

**Per flight (RTC flight log).** `flightSummary_t` ends with `fsumCamVz_t
camVz` (flight_summary.h, 34 bytes). It covers the observer's sortie that
started in this flight; all fields are 0 if none did, as after a hand launch.

| Fields | Content |
|---|---|
| `flags` | `FSUM_CVZ_*`: ran, lifted off, lift-off by the accelerometer, touchdown, from an impact, gain tracked, prior from g0, fault |
| `g0Dmg` | g0 - 1 g at lift-off, 0.1 mg |
| `bias3Cmg`, `bias5Cmg`, `biasTdCmg`, `sigB5Cmg` | offset against 1 g at the camera wake +3 s, +5 s and at touchdown, 0.01 mg (`FSUM_CVZ_NONE` if not reached); its sigma at +5 s |
| `gainDpm` | latest tracked gain, 0.1 px/m |
| `hTdMm`, `vTdMms`, `hMaxMm` | height and speed estimate at touchdown (the drift: the drone is back on the floor) and the highest height estimate |
| `airCs`, `camAirCs`, `okCs`, `gapMaxCs`, `firstOkDs` | airborne time; of it, after the camera wake; of that, healthy; the longest stretch without an accepted update; wake to first healthy |
| `accepted`, `rejected`, `alarms`, `yawResets` | camera update counters (saturating) |

- `flights` prints these as `cv*` fields on each `F` line.
- `tools/espfly_diag.py flights` adds a table: offsets against g0, healthy
  %, h at touchdown, and `drift` (section 4.10).
- The RTC flight store grows by 576 bytes (16 flights).

**RTC magic bump: save `flights` and `crash` before flashing.**
- `FLIGHT_RTC_MAGIC` (flight_rtc_log.c) changes from `0xF1A6C0E3` to
  `0xF1A6C0E4`.
- On the first boot of this firmware the store has the old magic and is
  cleared. That loses all 16 stored flight summaries and the pre-reset trace
  (the `C`, `V` and `R` lines of `flights`).
- So before flashing, save the output of `python3 tools/espfly_diag.py
  flights` and of `... crash` (for example as a `flights-*.txt` next to the
  logs in `firmware-backups/cal/`).
- The crash log's own RTC record keeps its magic, and in this build its
  address: the link map has crash_log at 0x50000000 with 0x520 bytes, and
  the flight store follows it. So `crash` should survive, but save it anyway.
- A2b's `autoRtcRecord_t` (section 6) is another RTC record and can move
  the flight store (its magic check then clears it). Repeat the save before
  that flash (`docs/autonomy-a2b.md` section 6).

**USB console `camvz`** (read-only, any time; `tools/espfly_diag.py camvz`):
- with no observer, it answers `on=0`;
- otherwise one `OK camvz on=1` line with:
  - where the state lives (`mem=psram|internal`, `bytes`);
  - the shadow phase, the sortie number and how its lift-off was detected
    (`lift=thrust|acc|hint`);
  - g0;
  - the estimate: h, v, offset and their sigmas, gain, `healthy`,
    `airborne`, `fault`, the time since the last accepted update, and the
    latest innovation and the decision on the latest frame;
  - the observer counters (frames, skips, quality, expansion, lateral, no
    window, no gain, accepted, rejected, re-locks, CUSUM alarms, yaw resets,
    seq gaps, ignored IMU samples);
  - the task's cost (`stepUsMax`/`Avg`, `frameUsMax`/`Avg`, `frameCalls`;
    wall time, so preemption counts) and the camvz task's lowest free stack
    (`stabStackFree`, -1 until the first check);
  - the current sortie's `cv*` summary fields, as in `flights`.
- Bench diagnostics (RAM only, lost at reboot): `camvz off|on` (hand no
  blocks to the observer), `camvz batch 1-8` (wake the task every N blocks),
  `camvz prof` (avg/max us of the stabilizer's per-sample work, the block
  step, the frame update and the publish, plus ring drops; `camvz prof
  reset` restarts it).
- Bench load test `camvz air on|full|spin|off` (RAM only): the shadow treats
  the drone on the bench as airborne (a bench sortie, lift-off by the thrust
  rule on its first block) so every block and frame runs the in-flight path;
  `full` also passes every frame's quality gate, so each frame after the
  1.5 s wake reaches the Kalman update. Nothing reaches the motors and
  `camVzGlueVzInput()` reports the estimate unusable while it is on. `on`
  and `full` need idle motors and end by themselves at the first block with
  real thrust; `spin` needs a `gcap` cap (props off) and stays on through a
  spin until `camvz air off`. If it is left on, a real take-off without a
  ground start still begins a fresh sortie. Measure the loop rate with two
  `camvz prof` reads 20 s apart (`upMs`, `loops`, and the sums `execSumUs`,
  `s2oSumUs`/`s2oN` = mean IMU-interrupt-to-output latency, `s2o800` = loops
  above 800 us; `camvz prof reset` clears the observer's cost profile). On
  the bench, connect the phone app (throttle at zero) and run one `camvz air`
  sortie first: in the 2026-10-11 session even the motors-off seconds after
  each landing ran at 965-982 loops/s against 995 on an idle bench, so the
  app link and the state after a first sortie are part of the load.

### 8.7 Forbidden while active (any state but IDLE)

- NVS writes of any kind (a sector erase stops both cores for tens of ms):
  autonomy saves `vzK` only in IDLE with all duties 0; `usb_diag` commands
  that write NVS answer `ERR busy: autonomy active`.
- `motorsIdle()` (usb_diag.c:199) reports not idle; `gcap` changes are
  accepted but can only lower the bench cap.
- Camera mode changes or re-initialisation (feasibility section 5).

The step-0 mailbox (`auto_mailbox.h`) fixes the flash-write interlock: a
flash write may begin only while the state machine's published state is
IDLE and no ARM is in the mailbox, and an ARM taken while a write is in
progress fails G1 (`LOCKS`); both decisions are made under one lock
(`test_auto_mailbox.c`). The `vzK` learn save, the console gates, the gcap
rule while active and the commands allowed while active are described as
built in `docs/autonomy-a2b.md` section 7.

## 9. Tests and acceptance

### 9.1 `tools/tests/test_vert_ctrl.c` (engineer 1)

All on `vzSim` with default plant and controller unless stated; check every
step that `0 <= thrust <= ceiling <= VZ_THRUST_ABS_MAX` (54000 then, 58000
since 2026-10-14). As built: 251 checks, 0
failures (A2b 222, A2a 202). The defaults are checked too: kPrior 165000,
rampRate 13250, landClimbRefRel 1.00, abortSlowRel 0.60; revision 5: holdMaxS
17; hMax 1.0, climbMaxS 2.0 and landFlatS 1.5 up to 0.40 m.

Revision 5 (0.60 m, the 10 s wait; the emulated state machine uses 6 / 18 /
12 s): `testHeightRange` flies 0.20, 0.40 and 0.60 m, each set through
`vzConfigSetHeight()`. Accelerometer only (DRY, no camera, no offset):
HOLD within +/-0.10 m in >= 95 % at 0.20 and 0.40 m and impact <= 1 m/s as
before; at 0.60 m within +/-0.15 m (0.50-0.81 m) and impact <= 1.25 m/s
(1.13). Warm camera, 10 s wait, 0.40 and 0.60 m: HOLD entry and peak within
+/-0.10 m in >= 95 %, never 0.20 m below (HOLD has no height feedback: the
camera noise walks it 0.42-0.62 m at 0.60 m), `HOVER_TIMEOUT` with a slow
LAND every time, the wait >= 1 s inside the pre-wait limit and its LAND
>= 2 s inside the hard one, and the 0.60 m impact p50 within 0.05 m/s of
0.40 m's and <= 0.45 m/s (0.27 vs 0.25, max 0.39). `testSetHeight` checks
`vzConfigSetHeight()` (unchanged up to 0.40 m and for NaN; hMax / climbMaxS /
landFlatS 1.1 m / 2.25 s / 1.83 s at 0.50 m, 1.2 m / 2.5 s / 2.0 s at 0.60 m,
valid; 0.61 m invalid) and the nominal climb
(1.27 s to 0.40 m, 1.72 s to 0.60 m, no `CLIMB_TIMEOUT`, the same margin).

1. capLo 1.10, no offset, K exact: ends LAND `CLIMB_CAPPED` -> DONE
   by touchdown; true max height <= 0.08 m; no CUT. (Documents open issue 1.)
2. capLo 1.65 (default), no offset, K exact: reaches HOLD with true h within
   0.30 +/- 0.10 m; `holdSteady` within 1.5 s of HOLD; LAND on command;
   touchdown by force or jolt within 3.0 s; impact <= 0.8 m/s.
3. Lift-off, 100 seeds: detected within 250 ms of the sim's true lift-off in
   at least 95 seeds, never before it; `T_lo` within -5..+15 % of the
   commanded thrust at the true lift-off; no detection in 10 s of thrust held
   just below lift-off at the RAMP plateau (noise only, 0.03 g). A stored bAir
   of -5/-10/-15 mg (sim matching) never detects before the true lift-off;
   RAMP never above its plateau, T_lo never above its bound; a lift-off
   hidden by a -20 mg residual floats below 0.20 m and RAMP ends. A2b, with
   the scaled ramp: a -10 mg residual is detected within 250 ms in >= 75
   seeds (80; 36 with an absolute 12000/s at 165k), and a stored bAir of -5
   or -10 mg is detected in every seed with T_lo within -5..+15 %.
4. No lift-off (sim kTrue = 1.8 kPrior, so lift-off would need about
   1.35 x kPrior / V, above the RAMP plateau): abort `NO_LIFTOFF_TIME` after
   the plateau, thrust down LAND's ceiling then at 60000/s, DONE `ABORTED`.
4b. The slow RAMP abort (A2b, `testRampAbort`; the same heavy drone, battery
   without sag): a LAND on the step after the RAMP thrust reaches
   `abortSlowRel kPrior / Vf` (0.600 x, well below the plateau) follows
   LAND's ceiling 0.95 -> 0.65 x that thrust to within 0.5 thrust units for
   3.0 s, then falls at 60000/s (DONE `ABORTED` after 3.28 s); a LAND one
   ramp step below it falls at 60000/s from its first step (DONE after
   0.42 s). A lift-off signature in that range with a tilt of 20 deg (gravZ
   cos 20 deg; at 0.642 x) or a rate of 300 dps (at 0.603 x) is
   `LIFTOFF_ATTITUDE` and falls at 60000/s from its first step, within 0.5
   thrust units of it throughout (DONE after 0.45 / 0.43 s); a copy of
   `vert_ctrl.c` without the `LIFTOFF_ATTITUDE` exclusion fails both rows
   (3.30 / 3.28 s).
5. LAND properties, 300 seeds, offset N(0, 10 mg), K +/-8 %: every
   step of LAND has thrust <= T_ref max(0.65, 0.95 - 0.10 t) and the ceiling
   never rises; touchdown or abort in 100 % of seeds; no touchdown declared
   with the sim's true h > 0.05 m; report p50/p95 of true max height and
   impact (no threshold: section 4.9 is the reference).
6. Fooled loop: sim `accAirG` switched to -0.030 g at LAND entry (the sim
   config is passed every step; the estimate then reads a fast descent, so
   the LAND loop wants more thrust): thrust follows the ceiling,
   `tdThrustOk` holds 2.1 s after LAND entry at the latest, touchdown before 6 s.
7. Floating: geGain 0.6 (stronger ground effect): no false touchdown above
   0.05 m, and the 6 s cut fires only if contact is never made.
7b. No false touchdown in the air (review finding: the old HOLD part could not
   fail): a Mac LAND 0.05-1.70 s after lift-off, mostly LAND from CLIMB, over
   capLo 1.10/1.65, targets 0.20/0.30/0.40 m, K 0.88-1.12, residual -20..+20 mg:
   at least 30 s of armed LAND above 5 cm (measured 92 s in 1536 sorties), no
   touchdown declared above 5 cm (highest 0.025 m).

   A2b (`climbLandSweep`): the CLIMB average arms the detector near the ground,
   which cut that exposure to 31 s. Eight more LAND times 0.03-0.24 s after
   the lift-off (inside `tHoldTauS`, T_ref = T_lo, armed from the first LAND
   step, in the air) keep the proof's exposure: 2540 LANDs (1746 from CLIMB),
   **>= 60 s** of armed LAND above 5 cm (67.5 s), no touchdown declared above
   5 cm (highest 0.026 m). The same sweep bounds the LAND from CLIMB against
   A2a (statistics over the original 12 LAND times; no camera, the checks):
   - impact from CLIMB, every row: max **<= 1.45 m/s** (A2a's; 1.41);
   - rise after the LAND, every row: max **<= 0.50 m** (0.48);
   - capLo 1.65, K 0.94-1.06, -10..+10 mg: impact p95 **<= 0.95 m/s** (0.84).

   With the camera awake from the first command (info): the observer healthy
   0.3 s after the lift-off, capLo 1.65 impact p95 / max 0.84 / 1.12 m/s, rise
   0.085 / 0.16 m, 53 s armed, no touchdown above 5 cm; healthy 1.3 s after
   it, 0.86 / 1.20 m/s and 0.096 / 0.22 m.

   Revision 5: the same sweep at targets 0.50/0.55/0.60 m (the LAND times
   reach a drone 0.4-0.6 m up and still climbing): no touchdown declared
   above 5 cm (highest 0.024 m) in 124 s of armed LAND above 5 cm (camera
   awake: 149 s), all ended, ceilings held. The LAND from that climb falls
   harder: no camera, impact max 1.71 m/s (checked <= 1.90), capLo 1.65
   inner p95 1.15 (<= 1.25), rise max 0.36 m (<= 0.40); camera awake, max
   1.37, inner p95 1.01.
7c. User range Monte Carlo (section 4.9), with the state machine's DRY and
   FLIGHT timing: 300 uniform draws per row plus the 8 corners with 50 seeds
   each.
   - Every ceiling holds on every step. Every LAND ends, within 6 s. The air
     clock stays <= 18 s (10 s before revision 5).
   - Without a camera: `HOVER_NO_CAM` and never a wait, corners included.
   - With a camera: most sorties land `HOVER_TIMEOUT`; a wait of 12 s (the
     maximum) lands `HOVER_TIMEOUT` too, never at the air clock; after 3
     photos 1 s apart under `autoair` 8 s it lands at the 18 s air clock
     (174/300, at 18.000 s). (A2b: a wait of 8 s landed at the 10 s clock.)
   - True peak and impact are checked on the larger of the uniform and corner
     maxima, against the measured values plus about 10 %:

     | Row | Peak limit | Impact limit |
     |---|---|---|
     | capLo 1.10 (DRY or FLIGHT) | 0.8 m | 1.35 m/s |
     | the accelerometer alone (DRY, FLIGHT without a camera) | 2.25 m | 2.35 m/s |
     | cold camera | 1.0 m | 1.5 m/s |
     | warm camera, wait 5 s | 0.85 m | 1.2 m/s |
     | warm camera, wait 12 s (revision 5; A2b's 8 s row: 0.85 m) | 1.15 m | 1.2 m/s |
     | warm camera, wait 12 s after 3 x 1 s photos (revision 5) | 1.25 m | 1.2 m/s |
     | camera lost | 0.70 m | 1.2 m/s |
     | c +/-0.10 m/s | 1.15 m | 1.5 m/s |

     The capLo 1.10 corners (0.70-0.71 m) would have failed the old 0.7 m
     bound, and the no-camera corners (2.06-2.08 m) the old 1.8 m bound.
7d. The external estimate: at -10/+20 mg a 6 s HOLD on the camera stays in
   0.15..0.45 m; a camera offset c of -0.05 m/s moves the drone at
   +0.05 +/- 0.02 m/s; no camera on real inputs: `HOLD_NO_CAM` at 3.0 s; on
   the sim: none, `HOLD_TIMEOUT`; camera lost: `HOLD_NO_CAM` once HOLD has
   lasted 3 s; a non-finite or implausible camVz/camBias, or not healthy or not
   valid: never CUT, never used; never used before lift-off.
7e. T_ref of a LAND from CLIMB as a unit (A2b, `testClimbLandRef`): the test
   keeps its own copy of the CLIMB average and of the time since the last
   capped step, from the outputs alone, and lands at 7 times 0.05-0.65 s
   after the lift-off and 50/150/300 ms after a capped step, over K
   0.88/1.00/1.12, -10/0/+10 mg, capLo 1.30 (climbs at its ceiling, then
   free of it) and 1.65, `landClimbRefRel` 1.00 and 0.80. In all 287 LANDs
   T_ref equals the rule exactly: T_lo within 0.25 s of the lift-off (60) or
   of a capped step (77), otherwise the CLIMB average inside its bounds (46)
   or at `landClimbRefRel kPrior / Vf` (90).
8. NaN or Inf in each input field, in each phase: CUT and thrust 0 in that step.
9. `VZ_CMD_CUT` in each phase: thrust 0 in that step, CUT latched until `vzInit()`.
10. `VZ_CMD_NONE` in SPIN..HOLD: LAND / abort with `CMD_MISSING`.
11. dt of 0, -1 ms, 100 ms: no NaN, behaves as the clamp says.
12. `vzConfigValid()` rejects each hard limit of 4.6 (tLoMaxRel, rampTopRel, holdMaxS 18 (10 before revision 5), hTarget 0.61, holdNoCamS 5 included; A2b: landClimbRefRel outside 0.80..1.00 and abortSlowRel outside 0.40..1.00, and NaN, while the limits themselves pass); `vzInit()` with a bad config gives CUT.
13. `vzLearnK()` (4.5):
    - the sample `kLearn` equals Khat at `holdLoopS`, and kPrior moves 0.3
      toward it, at most 5 %, clamped to `VZ_K_MAX`;
    - a camera hold at c = +0.05 m/s keeps the sample (145122) while Kfrz
      falls to 132645, and the sortie is learned from the sample;
    - a LAND started by `K_RANGE`, `HOLD_CAPPED`, `EST_RANGE`,
      `HOLD_TIMEOUT`, `CMD_MISSING`, `CLIMB_CAPPED`, `CLIMB_TIMEOUT` or
      `NONE` is refused, and `HOLD_NO_CAM` is learned;
    - refused: no sample, fromSim, no HOLD gain freeze, short sorties,
      aborts and CUT.
14. Same seed, same run (deterministic sim).

`tools/tests/test_vert_replay.c` (optional, skipped without the CSV
directory argument): feed each logged take-off's 100 Hz `az`, `thrust` and
`vbat` (held for 10 steps of 1 ms) through SPIN g0 and the RAMP detector;
lift-off must be detected for every take-off that the vertical design
listed, near its logged thrust. It sets its own `rampRate` (the pilot's
thrust flew), so A2b leaves it unchanged: 6 checks, 0 failures on
`firmware-backups/cal` (37 take-offs, 35 detected, none early).

### 9.2 `tools/tests/test_auto_fsm.c` (engineer 2)

Scripted scenarios with a fake vertical controller (hand-set `in.vz`). As
built: 1027 checks, 0 failures (A2b 1014).

1. Full sortie: ARM -> TAKEOFF -> pre-spin 1 s -> vzReset -> HOLD -> STABLE
   1 s -> captureStart (2, 150 ms) -> DONE -> the wait (`waiting`,
   `waitLeftMs` counting down) -> LAND `HOVER_TIMEOUT` exactly hoverWaitMs
   later -> touchdown -> rest 1 s -> IDLE with `lockAppThrust` and 600 ms of
   `ZERO`. With wait 0: LAND `SORTIE_DONE` at once; with the longest wait
   (12 s; revision 5): LAND `HOVER_TIMEOUT` inside the 18 s air clock (A2b's
   8 s wait ended at the 10 s clock); from clock 0 and across the wrap.
2. App: 60 s of zero-thrust packets: nothing; thrust 1000 or roll 2.1 deg
   while airborne: LAND `APP_INPUT`, never CUT; yaw 100 dps: nothing; in
   ARMED and pre-spin: IDLE.
3. App silent 500 ms: nothing; 501 ms (app required): LAND.
4. Heartbeat 999 ms: nothing; 1001 ms: LAND (ARMED: IDLE).
5. Tilt 31 deg for 49 ms: nothing; 51 ms: KILLED, `CUT`, `setEmergencyStop`.
   Same pattern for rate (29/31 ms), free fall (99/101 ms), tracking (499/501 ms).
6. In LAND with `tdThrustOk`: a tilt or rate trip with contact (\|acc\| 2 g or aRel 0.7 g just before) is
   `TOUCHDOWN_TRIP` (`landTrip` says which); without contact, or contact over 300 ms old, KILLED; a free fall
   or tracking trip KILLED with or without contact; without `tdThrustOk` KILLED.
7. NaN in each float input while airborne: KILLED `NAN`; in IDLE: ARM refused.
8. Mac KILL in every state; kill latch refuses ARM for 2 s.
9. Each guard bit G0-G12 refuses ARM alone; TAKEOFF re-checks them; bad
   token -> `BAD_STATE`; out-of-range params -> `RANGE` (heightCm 19 and 61;
   41 and 60 are accepted since revision 5). G3 at the A2b
   default `vmin` (4.00 V): FLIGHT 3.99 V refused, 4.00 V passes; BENCH
   3.99 V refused; DRY exempt (3.50 V passes) but a NaN battery is refused.
10. The air clock from the first RAMP step: LAND `AIR_TIME` at airMax (2000, NVS 4500, the default 6000, a broken
    NVS 9000 -> 8000, 0 -> 2000); a lift-off detected 2 s into the ramp still lands at the 6 s air clock; no
    lift-off: `AIR_TIME` (with NVS `autoair` 8000: `TAKEOFF_TIMEOUT` at 8 s comes first); stuck in SPIN:
    `TAKEOFF_TIMEOUT` at 8 s; the longest wait (NVS 13000 forced to 12000) from airMs ~2800: `HOVER_TIMEOUT`
    inside 18 s; a 12 s wait that starts after 6 s of air clock (a longer climb, 3 photos 1 s apart, NVS
    `autoair` 8000, TAKEOFF 60 cm): `AIR_TIME` at exactly 18 s. airMaxMs above the NVS value or 8000:
    `RANGE`. (Revision 5; A2b: 4500 / 6000 / 10 s / 8 s.)
11. Battery: 3.04 V for 999 ms: nothing; 1001 ms: LAND; fast min 2.55 V or 2.62 V: nothing; 2.54 V: LAND.
    Since 2026-10-14 (autonomy-a2b.md section 16) `VBAT_LOW` is tested in TAKEOFF (2.59 V for 999 ms:
    nothing; 1001 ms: LAND `VBAT_LOW`), where `BATT_LOW` does not apply; in HOVER the same reading lands
    `BATT_LOW` at 500 ms; 2.54 V gives `VBAT_DIP` in TAKEOFF and HOVER.
12. NOT_STABLE (airMax 5000 so it does not interfere): warm (camera view
    from HOLD) lands at HOLD + 2 s; cold (camera view starts 1 s after HOLD)
    lands at HOLD + 2.5 s.
12a. STABLE streak (3.5), FLIGHT, DRY and BENCH: a 100 ms dropout of each
    soft condition (frame age 301 ms, coasting, hold not applied, holdSteady,
    roll/pitch 4.1 deg, gx/gy 30.5 dps, gz 10.5 dps, and 5.9 deg / 59.5 /
    29.5 dps just inside the hard bounds) 500 ms in keeps the streak,
    `stableMs` counts on through it and the photos start 1 s after the
    streak began; 200 ms ends it on the dropout's 150th step and the photos
    come 1 s after the conditions return; 149 ms survives, 150 ms does not;
    two 140 ms dropouts 10 ms apart (280 ms) survive; one step beyond a
    hard bound (roll/pitch 6.1 deg, gx/gy 60.5 dps, gz 30.5 dps, camera
    standby, vz IDLE) ends it at once (camera waived in DRY and BENCH); a
    dropout over the 1 s mark starts the photos at its end. Frame age 250
    and 300 ms: photos at holdSteady + 1 s; 301 and 350 ms: never STABLE,
    `NOT_STABLE` at HOLD + 2 s; ages 130..219 ms (frames every 90 ms): no
    dropout.
    `NOT_STABLE` still at HOLD + 2 s with 200 ms coasts every 800 ms, with
    gx 60.5 dps one step every 500 ms, and with a streak lost 0.9 s in.
    The dropout budget, FLIGHT, DRY and BENCH: dropouts of 3 x 99 ms keep
    the streak (photos 1 s after it began); 3 x 100 ms end it on the third's
    last step, and the next streak starts with a fresh budget (a 140 ms
    dropout in it is kept). Lasting violations that come back for a moment
    land `NOT_STABLE` at HOLD + 2 s with no photos: hold coasting but 10 ms
    in every 150 ms, holdSteady only 10 ms in every 150 ms, coasting on
    every other 90 ms frame, five soft failures in turn with 1 ms good
    between, gz 11 dps or roll 4.5 deg with one good step in every 100 ms
    (holdSteady, gz and roll in DRY too). With `now` moving 1 and 2 ms on
    alternate steps (1.5 ms a step), 3 x 90 ms of dropouts keep the streak
    and the photos come when it is 1 s old; 4 x 90 ms end it in the fourth.
    On 6733318's auto_fsm.c 75 of these checks fail: every dropout case
    (the old rule restarts on any failing step) and frame ages 250, 300 and
    130..219 ms; the hard-bound, 301/350 ms, camera-waived, low-duty and
    NOT_STABLE cases pass on both. On 602e6ec's (the grace without the
    budget) 13 fail: 3 x 100 ms, every low-duty pattern and 4 x 90 ms at
    1.5 ms steps. An accounting that assumes 1 ms steps (adding
    `now - stableOkMs - 1` on every good step) fails the 3 x 90 ms case at
    1.5 ms steps: it breaks the streak at S + 513 ms.
13. KILLED -> IDLE only after 2 s, 1 s at rest and app thrust 0 or absent;
    `clearEmergencyStop` only if the FSM set it.
14. Modes: `thrustMax` 0 in DRY, bench cap in BENCH (lowered by a later
    smaller gcap, never raised by gcap 0 or a larger one), `VZ_THRUST_ABS_MAX`
    in FLIGHT (54000 then, 58000 since 2026-10-14),
    65535 in IDLE after the release hold.
15. LAND/DISARM/KILL never refused (table 3.3); PHOTO `BAD_STATE` outside the wait.
16. The hover wait: Mac LAND -> `LAND_CMD`; the phone -> `APP_INPUT`, no cut; no
    `CAPTURE_FAIL` after DONE; PHOTO while waiting starts a set (n, gap), `BUSY` while
    it runs, `RANGE` out of range, does not extend the wait; a PHOTO set FAILED ->
    `CAPTURE_FAIL`; DRY: the whole wait without a camera.
17. Camera height rule: no camera -> `HOVER_NO_CAM` at HOVER + 2000 ms; lost
    mid-wait -> at once; healthy by 1.99 s -> the wait goes on; NVS `autoaccel` 3000,
    9000 (-> 3000), 0; DRY and BENCH waive it.
18. G1 reset-lock: DRY exempt (a reset-locked DRY sortie reaches HOVER, motors 0),
    BENCH and FLIGHT refused.
19. G2 settle: FLIGHT refuses ARM at onGroundMs 19999 and 3000 and accepts 20000;
    BENCH and DRY accept 3000. `camNoStandby` (A2b): FLIGHT sets it in ARMED,
    TAKEOFF and HOVER at any battery (also armed at 3.71 V with `vmin` 3.70),
    never in IDLE or LAND; latched at ARM (50 ms after a sag to 3.50 V the
    state is still ARMED and the flag still set); BENCH sets it too; DRY
    never.

### 9.2b `tools/tests/test_auto_sortie.c` (coupled)

The real state machine on the real controller and sim plant, with a small
emulation of the A2b glue (FLIGHT). As built: 77 checks, 0 failures (A2b 61, A2a 40).
The emulation follows the A2b interface's timing (section 2): it reads the
camera, hold, ground and battery-minimum inputs every `AUTONOMY_SLOW_EVERY`
(10th) step and steps the FSM in IDLE through `autoIdleSkip`
(`AUTONOMY_IDLE_STEP_EVERY`, 100), the step-0 constants of `autonomy.h` (the
compile line adds the `modules/interface` and `hal/interface` include dirs).
Every sortie rests at a battery drawn per seed in 4.00-4.15 V and arms at
the default `vmin` 4.00 V; a final check counts refused ARMs (0).

- **Late lift-off** (`testLateLiftoff`): -20 mg residual, no camera, 200
  sorties. The true lift-off to LAND is <= 4.5 s in 200/200; measured at most
  3.38 s.
  - The bound comes from the controller's `NO_LIFTOFF_TIME` (194) and
    `HOVER_NO_CAM` (6). No sortie reaches the air clock.
  - So this case cannot tell a RAMP clock from a lift-off clock, and it is not
    the air clock's test.
  - The info line prints the land-reason split.
- **The air clock** (`testAirClock`): 100 sorties, warm camera, photos
  3 x 1 s, residual 0/-5/-10 mg, the default `autoair` (6 s since revision
  5; 4.5 s before). Requires:
  - `AIR_TIME` in >= 75 % of sorties (measured 100/100);
  - LAND within 6.002 s of the first RAMP step (measured 6.001 s; 4.502 s
    and 4.501 s before revision 5);
  - true lift-off to LAND <= 6.0 s (measured 5.06-5.10 s; <= 4.5 s and
    3.56-3.60 s before revision 5).

  A copy of `auto_fsm.c` that starts the clock at the detected lift-off failed
  it in 100/100 sorties (A2a). The unit test `test_auto_fsm.c` (9.2 item 10)
  covers the same rule with a fake controller.
- **Learning chains** (`testLearnChain`, 4.5): 10 chained sorties each, at
  c = +0.05, +0.06, -0.05 and 0 m/s (true K 165000, starting there), and
  c = +0.03 with a stale prior (true K 165000, starting from A2a's default
  149500, ratio 1.104).
  - kPrior stays within 0.95-1.05 x true (0.90-1.05 for the stale prior) and
    ends >= 0.95 x true (the stale prior learns 149500 -> 160255, 0.971 x
    true, in 10/10).
  - The last sortie of each chain still reaches HOLD.
  - On the old `vert_ctrl.c` (learning the running Kfrz) it fails: kPrior
    0.878 x true at +0.05 and 0.864 at +0.06, and the last sortie never
    reaches HOLD.
- **Camera timing** (`testCameraTiming`, 4.11): residual +3.7 +/- 2 mg,
  K +/-5 %, c 0, the observer healthy 1.3 s after its frames start; the
  camera wakes 2.5 s after the first command (the manual take-off's standby,
  forced) or at once (the state machine's `camNoStandby`), both resting at
  4.05 V (A2a ran the standby row at 3.95 V, which G3 now refuses). Requires:
  no KILLED; awake: the wait ends `HOVER_TIMEOUT` in >= 95 %, `K_RANGE`
  <= 2 %, median true height at LAND >= 0.17 m and >= 0.04 m above the
  standby row (measured 0.20 vs 0.14 m).
- **Camera convergence** (`testCameraConvergence`, A2b, review R21): camera
  awake, realistic offsets, the observer's first healthy sample 1.3-5.4 s
  after the lift-off (the logged range), 200 sorties at `autoaccel` 2000 and
  3000 ms: never KILLED. Info (4.11): `HOVER_NO_CAM` cuts 103/200 waits at
  2000 ms, 59/200 at 3000 ms.
- **LAND from CLIMB** (tdThrustOk already set in the air) with a tilt trip
  at a true height of at least 0.09 m: KILLED `TILT` in 20/20. With a 2 g
  contact just before it: `TOUCHDOWN_TRIP`. A2b keeps A2a's premise with a
  drone 12 % lighter than its stored gain (K -12 %, the user range's edge),
  where T_ref <= tdThrust / landStart x kPrior / Vf from the first LAND step
  whatever the CLIMB average: a Mac LAND 0.70 s after the lift-off and the
  tilt on the first armed step give 20/20 armed at a true height >= 0.16 m.
  Every one of the 20 seeds must be a Mac LAND whose tilt starts, and the
  0.09 m premise is checked on each trip, not used to skip one (review
  WP4-3). (At 0.35 s, 2 of 20 lose tdThrustOk within the 50 ms tilt
  persistence and end KILLED, which is the rule.)
- **The phone 2 s into the wait:** LAND `APP_INPUT`, touchdown, never KILLED
  (impact max 0.52 m/s).
- **"Always slow land"** (`testSlowLandFromClimb`, A2b):
  - the phone stick 0.35/0.5/0.7/0.9 s after the detected lift-off, 20 seeds
    each, camera awake: LAND `APP_INPUT`, touchdown, never KILLED (80/80);
    impact max **<= 0.65 m/s** (0.34 / 0.35 / 0.42 / 0.52 m/s; A2a 0.55 /
    0.70 / 0.88 / 1.00); rise after the stick **<= 0.10 m**;
  - the phone stick in RAMP: -10/-15/-20 mg, K -12/0/+12 %, 8 stick times
    0.3-2.4 s into the RAMP, 4 seeds; the 175 sorties with no lift-off
    detected before the stick (67 already up) all end LAND `APP_INPUT`,
    never KILLED. Impact max 0.24 m/s (info; 0.46 m/s with A2a's
    plateau-only slow abort).
- **The Monte Carlo of section 4.9:** 200 uniform draws plus the 8 corners
  with 100 seeds each, per row.
  - No sortie KILLED. The air clock stays <= 18 s (10 s before revision 5);
    LAND and aborts end within 6 s. Each row has its expected endings; the
    12 s wait (the maximum) ends `HOVER_TIMEOUT`, never at the air clock.
  - True peak and impact are checked on the larger of the uniform and corner
    maxima:

    | Row | Peak limit | Impact limit |
    |---|---|---|
    | warm camera, wait 5 s | 0.85 m | 1.2 m/s |
    | warm camera, wait 12 s (revision 5; A2b's 8 s row: 0.85 m) | 1.2 m | 1.2 m/s |
    | no camera | 2.2 m | 2.3 m/s |
    | cold camera | 1.0 m | 1.5 m/s |
    | camera lost | 0.70 m | 1.2 m/s |
    | realistic | 0.70 m | 1.0 m/s |
    | stress 3.90 V (G3 overridden, `vmin` 3.70; A2b) | 0.85 m | 1.2 m/s |

- **0.60 m and the 10 s wait** (`testHigh60`, revision 5; numbers in 4.9):
  FLIGHT timing with the camera awake from the ARM, the Mac's default TAKEOFF
  (1 photo), `autowait` 10000, the default `autoair`; 40 and 60 cm on the same
  200 draws, realistic and user range. Requires: never KILLED, never
  `CLIMB_TIMEOUT`, never the air clock; the wait's LAND >= 2.5 s inside the
  18 s clock (max 15.08 s); realistic: every sortie waits, >= 1 s inside the
  6 s pre-wait limit (max 4.83 s), `HOVER_TIMEOUT` in >= 98 % (200/200 at
  0.60 m); the impact at 0.60 m within 0.10 / 0.15 m/s of 0.40 m's p50 / p95
  and <= 0.90 m/s (0.34 / 0.67 / 0.79 against 0.28 / 0.57 / 0.66); user
  range impact <= 1.30 m/s (1.16) and true peak <= 1.40 m (1.27). The 18 s
  clock still lands a wait that would end after it: 0.60 m, 3 photos 1 s
  apart, `autoair` 8000, `autowait` 12000: the wait starts at 7.2-7.5 s and
  `AIR_TIME` lands 50/50 at exactly 18.000 s. Without a camera at 0.60 m:
  never waits, never KILLED, true peak <= 2.8 m (2.56), impact <= 2.7 m/s
  (2.45) over the uniform draws; the user range's 8 corners (100 seeds each,
  the Monte Carlo's seeds) never KILLED, never wait, LAND by the 6 s
  pre-wait clock (at most 5.11 s), touchdown within 6 s (3.45 s), true
  peak <= 3.25 m (2.96, -20 mg, K -12 %), impact <= 2.9 m/s (2.63). The
  phone stick during the climb to 0.60 m at 0.35-1.60 s after the lift-off:
  LAND `APP_INPUT`, touchdown, never KILLED, impact <= 1.05 m/s (0.94 at
  1.60 s).

### 9.3 `tools/tests/test_motor_guard.c` (engineer 3)

`motorGuardCoreStep()` on a scripted clock (503 checks):
- steady progress never trips;
- frozen with the motors on trips at 80 ms (+0..5 ms), exactly once;
- frozen with the motors off never trips;
- a late gap (> 15 ms) adds nothing and keeps the stall observed before it.
  For example: 40 ms observed, then a 60 ms late gap (`MG_LATE`, `stallUs`
  still 40 ms), then a trip 40 ms later.
- a late call with progress, or with the motors off, restarts the window;
- a guard late every 40 ms still trips on a frozen loop, after exactly 80 ms
  of on-time observation;
- repeated 20 ms gaps never trip and count nothing;
- 50 flash erases (45 ms) with the motors on never trip;
- the worst legitimate stall, 61 ms around a 45 ms erase, counts 65 ms and
  does not trip;
- a 15 ms gap is on time, a 15.001 ms gap is late;
- a clock that goes backwards counts as late and restarts the window;
- disabled never trips and keeps no window;
- tripped stays `MG_TRIPPED`;
- `maxStallUs`, `lateCalls` and `maxCallGapUs` are correct.

### 9.4 Bench, props off (after A2b)

- Each `mguard test` fault with motors at gcap 15000: duty reads 0 within
  90 ms (hang, imu); panic: pins low before the handler's first line (scope).
- 10 min of app throttle at gcap 15000 (props off) with `mguard` on, with
  photos, STATUS and USB commands running: no trip; `maxStallUs`,
  `lateCalls` and `maxCallGapUs` recorded.
- DRY: a full sortie through every state on the sim; BENCH: phone stick ->
  LAND and motors 0 at the sim's touchdown; daemon stopped -> LAND at 1.0 s;
  hand tilt > 30 deg -> cut; Mac KILL -> duty 0 within 50 ms; `reboot`,
  `reboot rwdt`, a forced panic and the R button each give the reset-lock.
- Kill latency on the phone path: 240 fps video, thumb to motor stop, p99 <= 200 ms.

As adapted for the A2b build. The operator's checklist is
`docs/autonomy-bench.md` (Claude never flashes the drone or opens its serial
port, the user does); the glue's own bench targets (the loop-rate go/no-go
before anything spins, the stabilizer stack, the heap, the reset-lock
cases) are in `docs/autonomy-a2b.md` section 11. What the bench checks of
this document's rules:

- **Before flashing,** save `flights` and `crash` (8.6).
- **Battery.** DRY runs at any battery (G3 exempts it); BENCH needs a
  battery resting >= 4.00 V (`vmin`). Nobody lowers `vmin` for the bench.
- **Camera awake** in BENCH (`camNoStandby`, 4.11, 8.4): the camera never
  enters the take-off standby, the window ends about 1.5 s after the sim's
  detected lift-off, and the photos reach the Mac during the wait.
- **Mac KILL latency (user decision: the Mac KILL and the automatic trips cut
  at once).** Duty 0 within 50 ms of the KILL packet with a live or blocked
  stabilizer; 80-90 ms, by the motor watchdog, if the stabilizer busy-loops
  on the link task's core (section 5).
- **Phone (user decision: "always slow land").** A stick in any airborne
  state is LAND `APP_INPUT`, never KILLED; the phone never stops the motors,
  so its latency is measured from the thumb on the stick to LAND entry.
  Target p99 <= 200 ms end to end (app plus link).
- **Trips:** a hand tilt > 30 deg in BENCH is KILLED `TILT`, duty 0 at once;
  the daemon stopped is LAND `HB_LOST` 1.0 s after the last heartbeat.
- **Not testable props off** (stage-B prerequisites): the camera drift hold
  engaging in an autonomous HOVER (needs thrust > 30000); the observer's time
  to first healthy in an autonomous take-off with the camera awake (it decides
  how many waits `HOVER_NO_CAM` cuts short, 4.11); the supply-rail minimum of
  an autonomy-profile take-off with the camera awake. The first FLIGHT logs
  also check the LAND from CLIMB impacts and the RAMP-abort rule.

## 10. Open issues (need the user)

1. **Climb cap and LAND from CLIMB.** The user's first cap, 1.10 x lift-off
   thrust, cannot leave ground effect: in the 36 logged take-offs, hover
   thrust was 1.28-1.57 x lift-off thrust (median 1.41). The lead's default
   is now 1.65 (`vzcaplo` 1650). There `capKhat` (1.15 x Khat / V) is the
   ceiling that binds.

   A LAND during the climb (the phone's slow LAND, a Mac LAND,
   `CLIMB_CAPPED`, `K_RANGE`) starts from T_ref = T_lo, about 0.75 x hover:
   - it descends at up to ~0.3 g;
   - in the mid-climb sweep it touched down at up to 1.45 m/s;
   - that is not the "slow LAND" the user asked for.

   Fix to decide (a LAND rule change): take T_ref from the applied thrust in
   CLIMB (an average, as T_hold) instead of T_lo.

   **Resolved in A2b** (user decision "always slow land", including during
   the climb): T_ref from the CLIMB average within [T_lo, `landClimbRefRel`
   x kPrior / Vf], T_lo within 0.25 s of the lift-off or of a capped step
   (4.2). A phone or Mac LAND in the climb now touches down at 0.34-0.52 m/s
   (A2a 0.55-1.00). **The residual corner, reported to the user:** at -20 mg
   with a stored gain 12 % too high, without the camera, the worst impact
   barely moves (1.41 against 1.45 m/s) and the rise after the LAND reaches
   0.48 m (A2a 0.30 m); with the camera awake, 1.12-1.20 m/s and 0.16-0.22 m.
   `landClimbRefRel` 1.00 is proposed (0.90 fails two limits and buys little
   with the camera); the first FLIGHT logs check it.
2. **Without the camera, height runaway is bounded only by time.** In the
   user range (+/-20 mg, K +/-12 %):
   - the uniform draw's true peak reaches 1.68-1.73 m (p95 0.83-0.99 m);
   - its corner at -20 mg, K -12 % reaches 2.06-2.08 m at impacts up to
     2.18-2.20 m/s (4.9);
   - revision 5, the higher targets, the same corner: **2.82-2.96 m at up to
     2.63 m/s at 0.60 m** (uniform draw 2.56 m, 2.45 m/s), 2.18-2.29 m at
     2.32 m/s at 0.40 m (4.9, "Revision 5"). The accelerometer alone climbs
     longer to the higher target before `HOVER_NO_CAM`; with A2b's 4.5 s
     pre-wait clock the 0.60 m corner reached 2.49-2.61 m.

   The bounds are the pre-wait air clock (4.5 s; 6 s since revision 5, which
   raised the 0.30 m uniform no-camera peak to 1.84 m, corners unchanged, and
   the 0.60 m corner as above) and `HOVER_NO_CAM` 2 s into the hover; at
   0.30 m the true lift-off to LAND took at most 3.85 s, at 0.60 m the LAND
   comes at an air clock of 4.9-5.0 s. Only the operator's kill switch stops
   such a climb sooner (`docs/autonomy-bench.md` C2: the kill height).

   With a healthy camera the worst peak was 0.69-0.90 m, and 0.54 m in the
   realistic range (0.57 m including its corners). The camera's slow offset c moves the drone by c x the
   wait: a positive c sinks it toward the floor, a negative c lifts it.
   Measuring c on today's drone (the camera shadow test the user chose) comes
   before trusting the wait.
3. **Heap.** G9 is 12 KB because A1 left 15.7 KB free (13.0 KB minimum);
   the design's 18 KB would refuse every ARM. Trimming the link task comes
   before stage B. A2b moves the link task's stack and transmit buffer to
   PSRAM (`docs/autonomy-link.md`, On the drone); the glue's internal RAM is
   in `docs/autonomy-a2b.md` section 10, and the heap targets of the bench
   (measured against the M0 baseline) in its section 11.
4. **Nuisance LANDs.** With the lead's 500 ms app-silence and 2.55 V
   fast-battery thresholds, the logged gaps (up to 319 ms) and dips (down to
   2.62 V) no longer land. The dip margin is 70 mV.
5. **Cold sorties.** The hover lands `HOVER_NO_CAM` if the camera height
   estimate is not healthy 2 s into HOVER. In the sim a camera waking 3 s into
   the take-off was healthy in time (149/200 sorties waited), but the real
   observer's convergence after a cold wake is untested.

   A2b: the camera no longer sleeps in an autonomous take-off, cold or warm
   (4.11). What is left is the observer's time to its first healthy sample
   with the camera awake, unknown until the first FLIGHT logs. Drawn from
   the logged 1.3-5.4 s and counted from the lift-off (pessimistic),
   `HOVER_NO_CAM` ends 103/200 waits 0.12-0.20 s after they start at the
   default `autoaccel` 2000 ms, and 59/200 (after about 1.2 s) at 3000 ms,
   where the no-camera corners reach 2.20 m / 2.27 m/s, exactly at their
   limits. Options:
   - (a) keep 2000 and decide after the first FLIGHT logs (proposed);
   - (b) `auto set autoaccel 3000`, an NVS value in the validated range;
   - (c) a state machine rule change, for example "no healthy sample yet"
     waits up to X s while any later loss lands at once.

   Also offered, for A2b or stage B: a `HOLD_LOST` LAND when the camera drift
   hold has not been engaged for more than 1-2 s during the wait (today the
   drone then coasts on the gyro and the accelerometer).
6. **DRY and the reset-lock**: resolved. DRY is exempt (lead decision).
7. **Stored hover gain (`vzK`).** Today's drone hovers at 1.09-1.15 x the
   default 149.5k (flights 19-21), and lifted off at up to 0.91 x it.

   The RAMP plateau sits at 1.00 x kPrior / V. In the sim it still detects
   every lift-off up to 10 % above the prior. At 15 % above, with the logs'
   highest lift-off ratio, 26/40 sorties end `NO_LIFTOFF_TIME` (safe, but
   no flight). A prior 12 % too high lets a strong drone rise at the plateau
   before the detection.

   The user range's worst corners are all at K -12 %, a prior 12 % too
   high (4.9).

   Set `vzK` over USB to the drone's own gain before stage B (section 11).
   `vzLearnK()` is slow by design:
   - it learns only from sorties that reach `holdLoopS` and land by
     `CMD_LAND` or `HOLD_NO_CAM` (4.5);
   - it moves 0.3 of the way per sortie, at most 5 %;
   - in the sim it settles about 2.5 % low.

   The stale-prior chain (true 1.10 x 149.5k) went from 0.91 to 0.97 x true
   in 10 sorties.

   **Resolved in A2b** (user decision): `VZ_K_DEFAULT` is 165000, the logged
   hover K at 40 kHz PWM (163-167k), and `rampRate` scales with it
   (13250/s, 4.2). A `vzK` set over USB stays in NVS (`docs/autonomy-a2b.md`
   section 7); from a stale 149500 the chain learns 149500 -> 160255
   (0.906 -> 0.971 x true) in 10 sorties. Check the value against the first
   FLIGHT logs.
8. **Photos during the wait.** The wait only helps if Claude gets the photos
   during it. The A2b glue must push them in the air, after the take-off
   window's photo and Wi-Fi pause has ended (section 8.3). **Resolved in
   A2b:** the window ends 1.5 s after the detected lift-off (8.4), so the
   photos are captured in HOVER after it, and the link pushes them during
   the wait (`docs/autonomy-link.md`, Photos).
9. **Sign-off on the revision-2 ceiling and abort changes**: `rampTopRel`
   (the plateau), the abort that follows LAND's ceiling from the plateau, and
   `tLoMaxRel`. They only lower thrust. **A2b adds, for sign-off:**
   `rampRate` 13250 (4.2), `landClimbRefRel` 1.00 (LAND from CLIMB, 4.2),
   `abortSlowRel` 0.60 (the slow RAMP abort, 4.2) and the G3 DRY exemption
   (3.4).
10. **Large negative in-flight offsets fail closed.** At -15 to -20 mg the
    lift-off is often not detected and RAMP ends (`NO_LIFTOFF_TIME`, 8-10 %
    of user-range sorties). The camera study measured +1.4 +/- 2.2 mg against g0,
    so this should be rare. If it is not, the stored `bAir` and the per-sortie
    g0 need a look.
11. **Learning without the camera** (lead's call). A sortie without a healthy
    camera still learns, from Khat at `holdLoopS`, and the residual offset
    skews that sample. 40-sortie chains settle rather than ratchet:
    - 0.967 x true at +3.7 mg;
    - 0.96 at +8.1 mg;
    - 0.90-0.92 at +20 mg.

    Requiring the camera at the sample step would remove the skew, but then
    no sortie without a camera would learn. The code learns either way for
    now.

## 11. Open items for stage B

1. **Set the stale `vzK` from measured hover data.** Today's drone hovers at
   1.09-1.15 x the stored 149.5k (flights 19-21). Set `vzK` over USB from the
   logged hover thrust x battery voltage; do not wait for `vzLearnK()`
   (open issue 7). A2b: the default is now 165000 from those logs; `auto get`
   shows the value in use (`docs/autonomy-a2b.md` section 7), and the first
   FLIGHT logs re-check it.
2. **A LAND during the climb is not slow.** A phone or Mac LAND mid-climb
   starts from T_lo, about 0.75 x hover, and touched down at up to ~1.45 m/s
   in the mid-climb sweep (9.1 item 7b). It needs the LAND rule change of
   open issue 1. A2b: done (the CLIMB average, 4.2); the first FLIGHT logs
   check the CLIMB LAND impacts and the residual corner.
3. **The camera offset c moves the drone during the hover wait.** That is
   about 0.25 m over 5 s at 0.05 m/s. A positive c sinks the drone to the
   floor. Measure c with the shadow test first, before trusting the wait:
   - manual flights with the camera on (`cam on`), landed on the floor;
   - read the `drift` column of `tools/espfly_diag.py flights` (4.10).
4. **Photos must be sent in the air.** The A2b link pushes them during the
   wait, after the take-off window's photo and Wi-Fi pause has ended (8.3,
   8.4, open issue 8; `docs/autonomy-link.md`). A2b: done.
5. **What props-off cannot show** (9.4), before the first FLIGHT sortie, a
   separate decision: the camera drift hold engaging in an autonomous HOVER;
   the observer's time to first healthy with the camera awake (open issue 5:
   `autoaccel` and the offered `HOLD_LOST` rule); the supply-rail minimum of
   an autonomy-profile take-off with the camera awake (4.11).
6. **The take-off height limit is 0.60 m** (`VZ_H_TARGET_MAX`; user
   decision 2026-10-12 evening, revision 5; 0.40 m before). The Mac tool's
   default stays 40 cm (about 0.32 m true on the logged mission, 0.3-0.55 m
   in the sim); `--height 60` holds about 0.5-0.6 m in the sim (on the
   drone's 0.32 / 0.40 ratio, ~0.48 m). The Monte Carlo rows at 0.60 m are
   in 4.9 ("Revision 5") and 9.2b.

## 12. The 2026-10-13 interactive-hover failures

Two FLIGHT sorties on 6844898, 10:39-10:41, from the floor of a bright room
with large windows (`firmware-backups/cal/flight-2026-10-13-interactive.csv`,
`flights-2026-10-13-interactive.txt`, `auto-rtc-2026-10-13-interactive.txt`;
the photos `~/.espfly/photos/20261013-*`). Times below are seconds from the
HOVER entry on the recorder's tick, which runs ~6.5 % slow against the camera
clock and the FSM (the FSM's 2.0 s is 1.87 s of recorder tick).

- **Flight 1** (4.13 -> 3.26 V, air 38.5 s): hover at ~1.06-1.08 m by the
  estimate, scene distance D ~3.1 m. A person (a person) stood and walked about
  1-1.5 m in front of the camera from ~30 s on. At 34.45 s the drone hit
  something (what, the logs cannot tell); 0.77 s later the vertical
  controller landed it `VZ` (`K_RANGE`). The summary read h at touchdown
  +1.15 m, v +0.45 m/s, drift +0.028 m/s, tilt max 12.5 deg, gyro max
  622 dps.
- **Flight 2** (3.96 V, 60.0 s after flight 1's 41 s): HOVER at ~1.06 m, then
  LAND `HOVER_NO_CAM` exactly 2.0 s into it. Observer ok 60 %, 81 accepted /
  44 refused / 1 CUSUM alarm; ground baseline g0 - 1 g +12.5 mg (flight 1:
  -4.0 mg; logged flights usually +1.4 +/- 2.2 mg). A bright window filled
  the right half of the view.
- The trim learner moved the attitude trim (roll/pitch) 0.24/0.78 ->
  0.26/0.63 after flight 1 and -> 0.34/0.65 after flight 2.

### 12.1 Root causes

| | Cause | Confidence |
|---|---|---|
| RC1 | Flight 1: the camera drift hold followed the person. Its view test passes a fit residual up to 15 grey levels and has no moving-object test, while the height observer refuses the same frames (residual 8, `|dxRes|` 1.5 px, smoothed `|scale / dt|` 0.3 /s). The crossing (30.9-32.4 s) and the recession (33.8-34.43 s) were integrated as the drone's own drift: pitch reached its -2.5 deg limit at ~34.0 s, roll -1.04 deg, and the drone flew forward-left into the contact (impact ax -1.72 g, ay -1.57 g) | high |
| RC2 | Flight 1: no rule detects an in-air contact. The first (3.98 g, 492 dps) passed every trip (rate > 500 dps for 30 ms, tilt > 30 deg); the motors ran on for 0.86 s through five contacts (34.45, 34.70, 34.82, 35.11, 35.28 s). The vertical loop, still on a healthy observer, read the impulses as a climb (v +0.42, then +0.85 m/s), cut thrust to ~30k and ran Khat into its floor: `K_RANGE` at 35.22 s. The 12.5 deg and 622 dps of the summary are these contacts, not the touchdown | high |
| RC3 | Flight 2: a false CUSUM alarm 0.98 s into HOVER held the observer unhealthy for 3 s, so the FSM correctly landed `HOVER_NO_CAM` at HOVER + 2.0 s. The room's textured parallax came from the floor and table, the window showed a far scene: the effective gain fell ~3x with height (tracked 92 -> 67 -> 34 -> 27 px/m in the 1 m climb). A tracked G counts as accurate to 20 % and white (`sigGrel`), so the climb speed the camera under-read went into the offset state (+6-7 mg); when G caught up at the top, the corrected camera speed disagreed with the offset-inflated prediction on 8 frames (z +1.1..+2.4) and CUSUM+ reached 5.6 (threshold 5) | high |
| RC4 | Flight 2, contributing: the IMU was heat-soaked. The take-off came 60.0 s after a 41 s flight, inside the 10-80 s soak window the 20 s settle allows. g0 was right (+12.5 mg) but the offset moved 5.8 mg in the air (normal +0.3 +/- 1.8), which put the prior a few mg on the alarming side. Not the cause alone: priors of 3-8 mg sigma still alarm | medium |
| RC5 | Misleading diagnostics. Flight 1's 'h at touchdown +1.15 m, v +0.45' came from the third contact: the shadow opened a new impact record > 0.5 s after the previous record's first impact. Before the first contact the observer read v -0.004 m/s at h 0.83 m. Flight 2's '-1.36 m / -0.126 m/s' is the 3 s CUSUM coast plus a fast LAND, not a camera speed offset | high |
| RC6 | The trim learner learned from both failed sorties: no gate on how a sortie ended, on hold saturation or on the interactive motions (whose feedforward lean camHoldStep adds into the correction it averages). Pitch 0.78 -> 0.63 is the capped 0.15 deg step the clean part of flight 1 alone also gives; roll 0.26 -> 0.34 came from 6.2 s of the HOVER_NO_CAM sortie on a heat-soaked IMU | medium |

Side findings, not fixed here: the height at ~1 m is uncertain by about
+/-0.3-0.5 m (flight 2 probably overshot to 1.3-1.7 m while estimating
1.05-1.11 m; flight 1's photo suggests 0.6-0.9 m, low confidence); flight 2's
LAND, started with an unhealthy estimate, touched down at ~1.5-1.9 m/s with a
3.4 g impact, a bounce and a 44 deg yaw spin; in long clean hovers the hold's
forward integral winds slowly (iFwd ~-0.4 by 22-28 s, a scale/dt bias of
~-0.02 /s, also in mission60); at 12.71 s in flight 1 something brushed the
drone (77 dps, +0.22 g), below the new trip.

### 12.2 The fixes

Estimation, health and gating only: no gain, limit or LAND-law change.
`vert_ctrl.c`, the observer's 10 ms block path, `optical_flow.c` and the
LAND/touchdown rules are untouched. Each fix is a no-op on the clean
2026-10-12 autonomy sorties (12.4).

1. **Observer gain re-anchor** (RC3, RC4; `cam_vz_obs.h` `gainReanchorRel`,
   4.10): a tracked G that moved more than 30 % from its anchor re-inflates
   sigma_v once (as a re-lock) and becomes the new anchor; a yaw restart
   clears the anchor. Only in the take-off climb window (until 2 s after the
   speed estimate's climb ended, at most 6 s after the lift-off) and on a
   frame inside the gate, so a drag or a dead camera in the hover is still
   caught by the CUSUM. CUSUM, gates and P[2][2] unchanged. Counter
   `reanchors` (`camvz`).
2. **HOVER scene gate for the drift hold** (RC1; `camera_hold.h`
   `camHoldSceneOk`, 12.3).
3. **`COLLISION`** (RC2; 3.2 row 8b, 3.7): \|acc\| > 1.6 g or
   max(\|gx\|, \|gy\|) > 250 dps held 3 ms, armed in the controller's CLIMB and
   HOLD from 0.2 s after its lift-off, FLIGHT and DRY, never BENCH or LAND.
   98cd4ae cut the motors (KILLED: props must not keep running against a
   person); the user decided on 2026-10-13 to try a slow landing instead:
   LAND `COLLISION` with the hold blocked and a blind descent (12.8).
4. **Touchdown values from the first impact of a contact cluster** (RC5;
   `cam_vz_shadow.c`): a cluster runs on while its impacts come within
   0.5 s of each other. `espfly_diag flights` prints drift 'n/a (in-air
   contact)' for an in-air end (`COLLISION`, `TILT`, `RATE`, `FREEFALL`,
   `ATT_ERR`) and decodes the recorder's holdFlags bit 3 (frames the scene
   gate refused, per flight in `dump`/`analyze`).
5. **Trim-learner guards** (RC6; `att_trim_learn.h`): samples during a motion
   restart the settle; a correction at the hold's limit (\|corr\| >= 2.4 deg)
   is skipped; an autonomy flight is learned only when it ended LAND
   `LAND_CMD`, `SORTIE_DONE`, `HOVER_TIMEOUT` or `BATT_LOW` (manual flights
   as before). The stored trim is not changed by the firmware (12.7).
6. **Heat-soak settle** (RC4; G2, `AUTO_SOAK_*`, 4.11; **needs the user's
   sign-off**, its own commit): FLIGHT waits min(20 s + 1.5 x the latest
   flight's motor time, 90 s) on the ground.

### 12.3 The drift hold's HOVER scene gate

`camHoldSceneOk()` (camera_hold.c, pure) applies the observer's three frame
gates to the hold: residual < 8 (NaN refused), \|dxRes\| <= 1.5 px (dx minus
the rotation model `FLOW_KX_*`, the term already inside wLat), and the
EMA(0.2) of scale / dt within +/-0.3 /s. The EMA runs on every frame, armed
or not, as the observer's does. It is armed only in autonomy HOVER (the
recorder's latest `autoState`) after 2.0 s of it, counted in frame time from
the first frame seen in HOVER (a frame whose interval began before the entry
does not count; the camera clock is the FSM's, the recorder tick runs slow).
Manual flights, TAKEOFF, LAND and the first 2 s of every HOVER fly exactly
as before. `camera_flow.c` calls it from `holdUpdate()` on every frame and
resets it with the hold (init, camera restart); a refused frame is a lost
view (`viewOk` false), so the hold coasts on its learned integral. The
recorder's holdFlags bit 3 marks a refused frame; `hold` prints the refusals
by reason (`sceneRes`, `sceneDx`, `sceneExp`).

Interplay, no other change:
- A MOVE's own rates stay under the gates (`AUTO_MV_W_LAT_MAX` 0.8 px/frame,
  `AUTO_MV_W_FWD_MAX` 0.15 /s). A refused frame makes the hold coast: a
  running MOVE pauses and a new one is refused `NO_VIEW` (a2b 14.4); 2 s of
  it lands `MOVE_NO_CAM`.
- STABLE needs the hold applied and not coasting, but only before HOVER +
  2 s (the photos): the gate is not armed then.
- The observer refuses the same frames, so a view refused for 2 s is also 2 s
  without an accepted update: the existing `HOVER_NO_CAM` bounds the
  coasting. No integral decay is added.
- While a person moves in the view the drone holds on the integral alone:
  expect a slow drift of a few cm per second, not a lean toward the person.

### 12.4 Replays and tests (host)

Recorder replays use the recorder's own data throughout (the drone's a_up
and the recorder's differ by 3-5 mg, 4.11); long-hover replays of h drift,
so flight 1 is judged on v, decision counts and the summary rules.

- **Observer** (`test_cam_vz_replay_ih.c`; g0 from the recorder's ground
  rows, prior +1.37 mg): flight 2 with the re-anchor off reproduces the
  drone: 81 accepted / 36 quality / 44 CUSUM / 1 alarm. With it: 125 / 36 /
  0 / 0, 2 re-anchors (0.00 and 0.27 s after the climb end), healthy at
  HOVER + 2.0 s (h 1.07 m) and on every row from the first healthy one to the
  LAND entry, ok 86 % (59 %), h at the last airborne row -0.60 m (-1.44 m);
  no alarm for priors -6..+10 mg against g0. Flight 1: decisions unchanged
  (492 / 119 / 0 / 0), 2 re-anchors, |v| <= 0.081 m/s from HOVER + 2 s to
  the contact. The eight clean 2026-10-12 sorties (a2b-hops f1/f2, calib1,
  calib2, evening f4, hop5s f3, mission5s f1, mission60 f1): 0 re-anchors,
  every frame decision and row's h, v and offset bit-identical. A dead camera
  in the hover (the review's 49 cases: 7 sorties x a 0.1-0.4 m/s climb or a
  0.2-0.6 m/s excursion with the frames motionless from 1 s before it): no
  re-anchor in the fault; unhealthy no later than with the re-anchor off
  except in 2 (flight 1 at HOVER + 6 / + 15 s, where its own climb re-anchors
  left a different state: 1.16 s -> none, 0.80 -> 0.86 s; the review found 13
  of 49 before the guards); mission60 f1, frames dead from HOVER + 3 s and a
  0.3 m/s climb from + 4 s: unhealthy after 0.94 s (0.94 off; never before the
  guards). `test_cam_vz_obs.c` `testGainReanchor`: a 0.35 m/s climb 0.12 ->
  1.17 m with G = 6 + 15/h alarms in 22 of 30 noise seeds without the
  re-anchor, in none with it (healthy at hover + 2 s, |v err| <= 0.093 m/s
  there, 1-2 re-anchors, all within the window); a constant G is
  bit-identical. `testReanchorGuards`: testRelock's 4 px/frame drag with the
  CUSUM on alarms at 6.23 s and is identical to the re-anchor off (before: a
  re-anchor, no alarm, h 3 m off); 144 hover drags (G 50/120 px/m, -4..+4
  px/frame for 2.5 s from hover + 0.2..4 s, inside and after the window) keep
  the healthy speed error at the re-anchor-off level (worst 0.088 m/s both;
  before: up to 0.66 m/s). On the hand-flown study take-offs of 2026-10-10/11
  (`test_cam_vz_shadow` CSV mode) it fires in 7 of 50: decisions unchanged,
  offset at wake + 3 s within 3 mg in 26 of 28 (27 without the re-anchor).
- **Shadow** (`test_cam_vz_shadow.c`): impacts at 0 / 0.25 / 0.37 / 0.66 /
  0.83 s with the motors off at 0.86 s give the values before the 0 s impact
  (before: the 0.66 s one); a single impact and a bounce 1.2 s before the
  landing as before. Flight 1 replayed through the shadow: touchdown h
  +0.807 m, v -0.004 m/s, its values at the first contact (on board: 0.832 m,
  -0.004 m/s).
- **Hold** (`test_hold_replay.c`, the hold on the logged frames gated and
  ungated): the eight clean sorties and 2026-10-13 flight 2 bit-identical,
  never refused. Flight 1, 28-34.44 s: min pitch -2.50 -> -1.61 deg, min roll
  -1.07 -> -0.26 deg, coasting 17 -> 55 of 103 frames (95 refusals over the
  flight, all on the residual); 2-12 s: mean \|pitch difference\| 0.011 deg.
- **Synthetic scenes** (`test_flow_scenes.c`, the real `ofProcess` and the
  armed gate, 15 fps, background drifting 0.3 / -0.2 px a frame): an
  articulated, motion-blurred walker crossing at 2 px/frame (full frame
  height) is refused on 12 of 14 frames, at 3-4 px/frame (1.5-2 x) on 14,
  receding 2 %/frame on 13, standing on 0 (residual 1.6). Clean translation
  (up to 1.0 / 0.6 px) and expansion (+/-0.5 %, 1 %/frame): 0 refused and the
  hold's outputs identical. A saturated window (+60/+120/+200 grey: 1.3 /
  13.9 / 30.6 % of the pixels >= 250) biases the fit by <= 0.020 / 0.005 px,
  0.0003 a frame; exposure steps x0.95 / x0.90 with a window in a third of the
  view keep the residual at 4.4 / 6.7 (passed); a window filling half the view
  with x1.10 steps reaches 8.0 (borderline).
- **Contact trip** (`test_collision_replay.c`, rows held 10 x 1 ms): flight 1
  trips first at 34.453 s, never earlier; no trip in the CLIMB/HOLD of flight
  2 or the eight clean sorties; highest armed values elsewhere 1.16 g and
  77 dps. `test_auto_fsm.c` `testCollision` (3.2) and `testSoak` (4.11). The
  LAND `COLLISION` of 12.8 adds the replay through the state machine.
- **Learner** (`test_att_trim_learn.c` with the CSV directory): without the
  guards the two sorties reproduce the drone's learning (flight 1 -> 0.26 /
  0.63, flight 2 -> 0.34 / 0.65); with them neither changes the trim.

### 12.5 What the scene gate does not see (KNOWN_GAP)

A smooth-textured rigid object that moves as a whole drags the fit without
raising its residual. Pinned in `test_flow_scenes.c` (a change of the fit
moves these numbers and fails the test on purpose): rectangles of 15-75 % of
the width, textured or dark with mild texture, moving sideways 2 px/frame,
receding or approaching 2 %/frame, or vertically 1.5 px/frame: residuals up to
7.7 (textured) and 9.5 (dark), and on the frames the gate passes fit errors up
to 1.2-1.3 px a frame and 0.028 a frame in scale. The person of flight 1 was
caught on the residual; a person in plain, evenly lit clothes filling much of
the view may not be. The 2026-10-13 review measured the worst case for
following a person (the real `ofProcess` and the armed gate, background
drifting 0.3 / -0.2 px a frame): a light or dark smooth shirt filling 60-90 %
of the view and receding 1-2 % a frame is refused on 0 of 14 frames, and the
hold takes -0.07..-0.15 /s of expansion from it, i.e. the drone 'drifting
back', so it leans toward the person; a plaid shirt is refused on 10-14 of
14. The gate's 0.3 /s (the observer's) is ~5x the largest smoothed expansion
of a clean armed HOVER (<= 0.06 /s on the 2026-10-12 sorties; 0.44 /s with
flight 1's person). A HOVER-only 0.12 /s is the data-backed hardening, not
made here: it is tighter than the observer's gate, so a refused view would no
longer end in `HOVER_NO_CAM` and the hold could coast without a bound; it
needs its own (e.g. LAND after 2 s of coasting in HOVER), which would also
land the drone whenever a person slowly walks away in front of it, the
follow task's normal case (12.7). Until then: the operator rules of
docs/autonomy-bench.md phase D, and the `COLLISION` LAND (12.8) as the backstop.

The candidates for the fit itself (an IMU consistency gate on the hold's
rates, a tile-consensus robust fit) are deferred: a simple jump gate added
nothing on flight 1 (98 vs 96 coasts) and re-synced on a lasting bias, the
Kalman form needs D and re-acquisition, and a tile fit costs camera-task time
(20-25 ms of headroom); these cases are their baseline.

### 12.6 Deferred and rejected

- Substituting the estimate through a contact (vzStep, the observer's block
  path): deferred; the LAND `COLLISION` of 12.8 does not read the estimate
  at all (a blind descent), and both are validated IRAM paths.
- Mid-hover hysteresis for `HOVER_NO_CAM` / `HOLD_NO_CAM`: rejected; both
  reacted correctly, and hysteresis lengthens accelerometer-only flight above
  1 m.
- Freezing the offset at high camera speed: not chosen; it changes every
  flight (study set offset at +3 s within 3 mg 96.4 -> 92.9 %; mission60 b at
  touchdown 2.02 -> 1.92 mg), while the re-anchor never fires on the clean
  autonomy take-offs. (The re-anchor gives the same 92.9 % on the hand-flown
  study set, 12.4: its hand-flown climbs move G.)
- A 1/h gain model and coupling the LAND ramp to it: follow-ups that need the
  sim and the study re-validated.
- The per-sortie 1 kHz maxima of \|acc\| and gx/gy for the contact trip's
  margin: left out for the IRAM budget (+84 B); the recorder's 100 Hz values
  (12.4) are a lower bound.

### 12.7 Decisions for the user, and the next bench session

- **The heat soak** (fix 6) lengthens the signed 20 s settle: keep it or
  revert its commit ('Wait out the IMU heat soak before FLIGHT').
- **A contact KILLs**: the drone falls from its height. A slow LAND instead
  would be `enter(LAND, COLLISION)` with the hold blocked as the LAND contact
  latch does; it keeps the props turning near whatever was hit. *Decided
  2026-10-13: try a slow landing (12.8).*
- **The trim** now stored, 0.34 / 0.65 roll / pitch, includes the roll step
  learned from flight 2's HOVER_NO_CAM sortie on a heat-soaked IMU; the pitch
  step is what the clean part of flight 1 also gives. Restoring 0.26 / 0.63
  (`atrim set 0.26 0.63`, then `atrim save`) is the user's call.
- **Bench, before the next FLIGHT** (props off): a DRY sortie with `camvz air
  full` must keep >= 940 loops/s in TAKEOFF/HOVER/LAND (`auto prof`); a firm
  tap on the frame in DRY HOVER (\|acc\| > 1.6 g or a gx/gy rate > 250 dps
  for 3 ms) must give LAND `COLLISION` (12.8; KILLED before), the sim's
  blind descent, then IDLE `TOUCHDOWN`; `hold` shows the scene gate's
  counters.
- **People in the view**: docs/autonomy-bench.md phase D. A person in smooth
  clothes filling most of the view and walking slowly away from the camera is
  not refused by the scene gate (12.5). Hardening it means a HOVER-only
  expansion gate at ~0.12 /s plus a coasting bound (LAND after ~2 s of
  coasting in HOVER), which would also land the drone whenever the person
  walks away slowly in front of it: the user's call (land early, or keep the
  operator rule and the contact LAND).

### 12.8 LAND `COLLISION`: a blind slow landing after an in-air contact

**Superseded on 2026-10-15** (docs/autonomy-a2b.md 22.2): a contact in HOLD,
or in the climb with the camera height in use, no longer lands: `CONTACT`, a
0.5 s hush (the estimate frozen, the measured hover thrust), the motion
stopped or the hold re-anchored, then 1.5 s of open loop before the camera's
speed is used again, and the event with the contact's peaks and direction
for Claude. LAND `COLLISION` stays for a contact in the climb without the
camera height (the hover thrust not measured), and any LAND that begins
before the camera height is back after a contact is this blind one.

**User decision, 2026-10-13:** on an in-air contact, try a slow landing
instead of cutting the motors (98cd4ae's KILLED `COLLISION`). The trip itself
is unchanged (\|acc\| > 1.6 g or max(\|gx\|, \|gy\|) > 250 dps held 3 ms,
armed in CLIMB/HOLD from lift-off + 0.2 s, FLIGHT and DRY, never BENCH or
LAND); what follows it:

- **State machine** (3.2 row 8b): LAND `COLLISION` from TAKEOFF (CLIMB) or
  HOVER, after every cut row of the same step and before every other LAND
  row. `enter()` stops a running MOVE or YAW where it is (`LEFT_HOVER`: no
  rate, lean feedforward or vRef). `autoFsmOut_t.collided` is set from that
  step to the next ARM. Every KILL trip stays as it was and still cuts in
  the LAND (tilt > 30 deg for 50 ms, rate > 500 dps for 30 ms, free fall,
  tracking, the Mac KILL, NaN, the motor faults, the LAND backstop): a drone
  that flips or spins after the contact is still cut (a 600 dps spin: LAND
  `COLLISION` after 3 ms, KILLED `RATE` at 30 ms). A contact during LAND,
  this one or any other, does nothing new.
- **Camera hold blocked** for the rest of the sortie, from the contact's own
  stabilizer loop: the glue's `holdBlock` (autonomyHoldBlocked()) is set from
  `collided` right after the FSM step (the LAND contact path of the A2b
  hops: controller_pid.c zeroes the applied correction at once, camera_flow.c
  treats the drone as not airborne, which disengages the hold and clears its
  integrals). The drone stops leaning toward what it hit and flies on its
  trim alone.
- **Blind descent** (`vzInput_t.blind` = `collided`; 4.2 LAND): the contacts
  corrupt the vertical estimate (flight 1: +0.42 then +0.85 m/s of climb that
  was not there, and the loop cut the thrust to ~30k), so the controller's
  thrust is its ceiling alone: from HOLD `landTop` 0.98 x T_hold (measured
  in the hover just before the contact; T_hold from before the last vertical
  move or bob if that is lower, see "Contacts during vertical motions"
  below) for the flat part, with or without the camera estimate, then
  falling 10 % of T_hold per second; from CLIMB `landStart` 0.95 x the CLIMB
  average falling at once. No velocity loop, no stall rule, no
  bump-below-tdNearH touchdown (all read the estimate); the touchdown is the
  force or jolt rule once the ceiling reaches 0.75 x T_hov (2.3 s after the
  flat part); the LAND timeout and the FSM's LAND backstop are unchanged, and
  so is the flat part except after a move down (the reference height's, see
  below).

**The choice** (`tools/tests/test_vert_ctrl.c` `testContactLand`: the
controller on its sim plant with the warm camera model, hover at 1.00 or
1.20 m; at the contact the accelerometer reads a 10 ms pulse that puts an
error e of -1..+1 m/s on the speed estimate, and the observer's camVz carries
e too, decaying with tau 0.3 or 1.0 s, persistent, or the observer lost
50 ms later; one contact (+3 g, 3 ms) or flight 1's train at 0 / 0.25 / 0.37
/ 0.66 / 0.83 s; three drones (K -3/0/+3 %, -2..+2 mg), 5 or 20 s of hover;
240 sorties per height and option; LAND 3 ms after the contact):

| Option | 1.00 m: touchdown p50 / p95 / max | DONE in the air | 1.20 m: p50 / p95 / max | DONE in the air |
|---|---|---|---|---|
| (a) the normal LAND (landSlow from HOLD, the loop on the estimate) | 0.57 / 3.92 / 4.09 m/s | 102 | 0.57 / 4.26 / 4.41 m/s | 96 |
| (b) the ceiling alone (blind), **chosen** | 0.28 / 0.68 / 0.91 m/s | 0 | 0.36 / 0.74 / 0.99 m/s | 0 |
| (c) the normal LAND, loop off the first 0.5 s | 0.62 / 3.72 / 3.86 m/s | 23 | 0.60 / 4.04 / 4.22 m/s | 16 |
| (c) the normal LAND, loop off the first 1.0 s | 0.41 / 1.87 / 1.97 m/s | 0 | 0.45 / 2.05 / 2.13 m/s | 0 |

The KILL from the hover hits the floor at 4.43 m/s from 1.00 m (4.85 from
1.20 m) in free fall (the sim's true hover heights 0.95 / 1.14 m: 4.31 /
4.73 m/s; 3.88 / 4.22 with the plant's rotor damping). From a still hover
no option climbs (true climb after the contact <= 0.009 m in all 1920
sorties: every ceiling is below T_hold, which is the hover thrust there);
during a vertical motion 1c2a43f's blind LAND did climb (see "Contacts
during vertical motions" below). (a) and (c) fail the same way: an error that reads a
climb drives the loop to the thrust floor (0.65 x T_ref) and the drone
drops; one that reads a descent makes the estimated height reach tdNearH
while the drone is still up, so the train's next bump is a touchdown in the
air and cuts it; the loop off for 1 s only helps while the observer heals
within it. (b) does not depend on e (per error, persistent, the train: the
same speed within 0.10 m/s) and was never DONE in the air. Further:

- After 60 s of hover (the drift allowance lengthens the flat part by 5 s; it
  is min(2.0 m, 0.017 m/s x the HOLD) at 0.20 m/s since the 120 s air clock,
  docs/autonomy-a2b.md 18.1: 5.1 s at 60 s, up to 10 s; it was capped at 1.0 m):
  touchdown <= 0.17 m/s, but the drone then floats in ground effect at
  ~5-8 cm (thrust 0.98 x T_hold) for up to 6.9 s before the ceiling's ramp
  ends the LAND; timeout margin >= 1.25 s.
- Plant sensitivity (a still hover; rotor damping 0.035-0.14 g per m/s,
  ground effect 0.15-0.50): touchdown <= 0.84 / 0.96 m/s at 1.00 / 1.20 m,
  never in the air, no climb. The descent has no feedback: its speed is the ceiling's
  deficit against the damping, so a drone with much more damping than the
  sim reaches the ramp higher up and lands faster.
- A contact in CLIMB (true 0.60 m on the way to 1.20 m, 30 sorties): the
  drone first rises 0.1-0.16 m on its own climb speed; blind <= 1.19 m/s and
  never in the air, the normal LAND 3.45 m/s with 21 of 30 DONE in the air.
  The CLIMB reference: a contact read as a descent drives the climb loop into
  its cap within the 3 ms before the FSM lands, and A2a's rule then takes
  T_lo (~0.75 x hover, with `tdThrustOk` at once); a blind LAND keeps the
  CLIMB average (the cap is the contact's). Cost: a lift-off hidden at the
  cap that also meets a contact starts from <= 0.95 x kPrior / Vf.
- Closed loop (`test_interactive_sim.c` `testContactSim`: the real FSM,
  motions, controller and camera hold; 36 sorties at 1.00 / 1.20 m, e
  -1/0/+1 m/s persistent, one contact or the train, a 0.3 deg trim error):
  LAND `COLLISION` every time, the hold blocked on the contact's step and no
  correction applied after it, touchdown <= 0.86 m/s, climb <= 0.002 m,
  contact to IDLE <= 7.8 s, horizontal travel during the LAND <= 0.51 m (the
  trim error alone with the hold blocked); a contact 1.5 s into a forward
  MOVE of 60 cm stops the move on that step and lands as softly.

**Contacts during vertical motions** (the verification of 1c2a43f, the
first LAND `COLLISION` commit; fixed in the commit after it). The claims
above ("no option climbs", touchdown <= 1.0 m/s) held for still hovers and
one forward MOVE only. T_hold is a 0.25 s average of the applied thrust in
HOLD, so it carries the vertical motion the drone is flying: a MOVE up's
acceleration and climb, a MOVE down's braking at its end, the re-track bob
after a YAW and the re-view bob after a long lateral MOVE (0.20 m/s^2 legs).
At the worst contacts T_hold was 1.03-1.04 x the true hover thrust, so
0.98 x T_hold was 1-2 % above it, held open loop for the whole flat part
(3.3-5 s), and the drone kept
climbing (after a MOVE up, into whatever is most likely above it) and came
down from higher once the ramp passed the hover thrust. None of the KILL
trips sees a slow climb. Second, a take-off above the knee moved down to it
or below kept the take-off height's flat part (3.3 / 4.0 s from 1.00 / 1.20 m)
with the knee's 6.0 s timeout; the ceiling alone needs the flat part + ~2.9 s
to the touchdown condition (the normal LAND gets there earlier through its
loop and the stall rule, both off when blind), so those LANDs ended
`LAND_TIMEOUT` (a cut at <= 3 cm, but not `TOUCHDOWN`, and no gain learned).

The fix (vert_ctrl.c `holdQuiet()`, `blindLand()`): in HOLD the controller
keeps `tHoldQ`, T_hold as of the last step with no vertical reference
(`vRef` 0) in the `VZ_QUIET_S` (1.0 s) before it (from HOLD entry until the
first motion: T_hold itself); a blind LAND from HOLD takes T_ref = min(T_hold,
tHoldQ) (and K_ref from it). After a move down (`hOff` < 0) its flat part is
the reference height's (the timeout before landTimes()' last rule minus its
4 s: the knee's 2.0 s at or below it, 2.0 s + the extra height at 0.3 m/s
above it, plus the drift allowance), so the 4 s after the flat part that
every LAND above the knee keeps also holds there; the timeout itself is
unchanged. A low take-off moved up keeps its longer flat part (sim
0.20 + 0.50 m: 0.41-0.51 m/s at touchdown with it, 0.61-0.62 with the
reference height's). Not blind: bit-identical.

| Contact during (blind LAND) | 1c2a43f: climb max / touchdown max | now: climb max / touchdown max |
|---|---|---|
| MOVE up 0.5 m from 1.0 m, contact every 0.05 s, damping 0.035 / 0.07 / 0.14 g per m/s | 1.03 / 0.75 / 0.52 m; 2.35 / 1.85 / 1.43 m/s | 0.11 / 0.09 / 0.06 m; 1.10 / 1.26 / 1.10 m/s |
| the end of a MOVE down 0.5 m from 1.2 m (same) | 1.08 / 0.59 / 0.17 m; 2.14 / 1.49 / 0.92 m/s | 0.01 m; 0.64 / 0.65 / 0.78 m/s |
| the bob at 1.0 m (same) | 0.91 / 0.50 / 0.18 m; 2.23 / 1.63 / 1.11 m/s | 0.04 / 0.03 / 0.02 m; 1.12 / 1.04 / 0.92 m/s |
| a take-off at 0.8 / 1.0 / 1.2 m moved down to 0.3-0.6 m, contact in the still hover after (36 per case, damping 0.035-0.14) | 0 (0.46 s to spare) / 36 / 36 of 36 `LAND_TIMEOUT` | none; touchdown <= 0.66 m/s, LAND <= 4.9 s, >= 1.10 s before the timeout |
| a take-off at 0.2-0.4 m moved up 0.5-1.0 m, contact during the move | touchdown <= 1.37 m/s | <= 0.87 m/s (after the move: <= 0.92, unchanged) |
| closed loop (`test_interactive_sim.c`): MOVE up 50 from 0.70 m / MOVE down 50 from 1.00 m and the hover after it / the YAW 90 bob at 1.00 m | 0.54 m, 1.49 m/s / 8 of 30 `LAND_TIMEOUT` / 0.77 m, 1.84 m/s | 0.07 m, 1.03 m/s / 0.01 m, 0.37 m/s, all `TOUCHDOWN` / 0.04 m, 0.81 m/s |

(The first three rows: the verifier's sweep `sens.c` on the controller and
its plant, one contact or the train, e 0, the motion module's 0.20 m/s^2
and 0.15 m/s with its position correction. `test_vert_ctrl.c`
`testContactLand` runs the same three cases every 0.1 s with the motion
module's settle and the stop's measured `hOff`: now climb <= 0.113 / 0.006 /
0.079 m, touchdown <= 1.13 / 0.63 / 0.96 m/s; 1c2a43f 1.02 / 0.36 / 1.78 m
and 2.34 / 1.49 / 2.93 m/s.) What still climbs is the drone's own upward
speed at the contact (up to ~0.17 m/s in a bob, 0.15 m/s in a MOVE up): it
coasts up to ~0.11 m at the lowest damping, with the thrust already below
the hover, so it no longer pushes into what it hit. Corrected envelope:
touchdown <= ~1.0 m/s from a still hover at 1.0-1.2 m (0.91 / 0.99, the table
above), up to ~1.3 m/s after a contact during a vertical motion or with a
plant damped differently (a MOVE up reaches 1.5 m), still ~4x below the
KILL's free fall from those heights (4.4-5.4 m/s); climb <= 0.01 m from a
settled hover, <= ~0.05 m right at HOLD entry (still rising from the climb;
the verifier's `early.c`, unchanged), <= ~0.11 m during a motion.

**Tests:** `test_auto_fsm.c` `testCollision` (LAND, not KILLED; collided and
the glue's hold block on the step; the blind input on every LAND step; the
controller's touchdown ends it IDLE `TOUCHDOWN`; the next ARM clears it;
CLIMB at lift-off + 250 ms; RAMP, BENCH and LAND never), `testCollisionThenTrips`
(tilt, rate, free fall, tracking and the Mac KILL still cut in it; a spin:
LAND at 3 ms then KILLED `RATE` at 30 ms; further contacts change nothing),
`testCollisionMotion` (MOVE right/forward/up and YAW stopped on the step);
`test_collision_replay.c` (flight 1 through the real state machine: LAND
`COLLISION` at the predicate's tick 79023, 34.453 s from HOVER entry, the
controller landing and the hold blocked; the other nine sorties never
`COLLISION`); `test_autonomy_glue_host.c` `collision-dry` (a 3 g tap in DRY
HOVER: LAND `COLLISION`, autonomyHoldBlocked() on the same loop and through
the LAND, the RTC record and the recorder fields, a further tap changes
nothing, both barriers 0); `test_vert_ctrl.c` `testContactLand` (with
contacts every 0.1 s through a MOVE up, the end of a MOVE down and the bob,
damping 0.035-0.14, and after a move down to 0.50 m from 1.00 / 1.20 m:
all five checks fail on 1c2a43f); `test_interactive_sim.c` `testContactSim`
(also through a MOVE up 50 from 0.70 m, a MOVE down 50 from 1.00 m and the
hover after it, the YAW 90 bob: all three fail on 1c2a43f); `test_att_trim_learn.c` (a LAND
`COLLISION` flight is not learned). The Mac shows the reason by name as
before (`espfly_auto` 'state' events: to LAND, reason `COLLISION`;
`espfly_diag flights`: `AUTO(COLLISION)`, drift 'n/a (in-air contact)', its
h@td 'at or after the in-air contact').

**IRAM and heap** (eb9264d -> this change, same sdkconfig): the LAND
COLLISION costs `autoFsmStep` +28 B, `vzStep` +52, `startLand` +8, `stepFull`
+6, `vzRun` +4, `enter` +4 of IRAM; the ARM and TAKEOFF command handlers
(`armCmd`, `takeoffCmd`: once per command, motors off) moved out of line to
flash (`autoFsmStep` -428 B), as `motionCmd` and `landExtraMs` before them.
Net static IRAM 131054 -> 130726 B (-328): the text ends at 0x40393ea7
(+16 B prefetch pad), so `_iram_text_end` falls a 256-byte step, 0x40394000
-> 0x40393f00 (73 B of slack to the next step); .bss 87304 -> 87328 B (+24:
`fsm`, `fout`, `vin` +4, `vz` +8, alignment); flash .text +540 B;
`_heap_start` 0x3fcc0ff0 -> 0x3fcc0f08, i.e. 232 B more internal heap at
boot than eb9264d. `callgraph.py` from `autonomyArbitrate`: the same IRAM
functions; the only new flash targets are `armCmd` and `takeoffCmd` (an ARM
in IDLE, a TAKEOFF in ARMED); nothing new runs from flash per loop.
The follow-up (contacts during vertical motions, 1c2a43f -> its fix): IRAM
`holdQuiet` 46 B (new, out of line: the same two statements inlined cost
`vzStep` 84 B in register spills), `startLand` +8 (`|| blind` routes a blind
LAND through `landTimes`), `vzStep` -3; `blindLand` is flash, behind
`landTimes` (once per LAND). Static IRAM 130726 -> 130774 B (+48), the text
ends 25 B under `_iram_text_end` 0x40393f00 (unchanged); .bss +8 (`vz`:
`tHoldQ`, `vRefT`); flash .text +148 B; `_heap_start` 0x3fcc0f08 ->
0x3fcc0f10: 8 B less internal heap than 1c2a43f, still 224 B more than
eb9264d (0x3fcc0ff0). `callgraph.py` from `autonomyArbitrate`: one more IRAM
function (`holdQuiet`), the flash and ROM targets identical to 1c2a43f's.

**Risks and what to watch:**
- The blind LAND takes ~6.5-7.8 s from 1.0-1.2 m (up to ~12 s after a 60 s
  hover, the last seconds floating at ~5-8 cm), with the props turning and
  the hold blocked: the drone drifts on its trim (sim <= 0.5 m with a
  0.3 deg trim error). Keep people >= 2 m away (phase D).
- Its speed rests on T_ref being the hover thrust; 0.98 x T_ref leaves 2 %
  margin against a climb (the three logged autonomy hovers: 0.98 x T_hold
  against the next 3 s of mean thrust <= 0.999; none of them had a vertical
  move). A vertical move or bob no longer enters T_ref (`tHoldQ`, above), but
  a large gust while T_hold averages (tau 0.25 s) still can. `tHoldQ` is as
  old as the motion (up to ~11 s: a 0.5 m MOVE up, its settle and its
  re-view bob): a battery sagging meanwhile makes it read low, so the flat
  part sinks faster (never a climb). A contact during a MOVE up or a bob
  still rises on the drone's own speed, <= ~0.11 m on the sim.
- A real contact can push the drone (a true speed change); the sim only
  corrupts the estimate. The bench tap in DRY tests the logic, not the
  descent.
