# Autonomy stage A2b: the glue, as built

Stage A2b wires the host-tested state machine (`auto_fsm.h`) and vertical
controller (`vert_ctrl.h`) into the firmware, so the Mac can fly short
autonomous sorties: arm, take off straight up, hover STABLE for 1 s, push
photos to the Mac in the air, hover in place for at most `autowait` (5 s
by default, at most 60 s) waiting for LAND, another PHOTO or, since the
interactive hover (section 14), a MOVE, YAW or STOP, and land in place.

This document describes the firmware glue as built: `modules/src/autonomy.c`
(API `modules/interface/autonomy.h`), its pure part `utils/src/auto_glue_core.c`
(`utils/interface/auto_glue_core.h`, host-tested), and the changes to the
stabilizer, the phone hook, the NVS store, the USB console, the recorder and
the flight summary. The behaviour it runs is specified in `docs/autonomy-a2.md`
(state machine, controller, guards, thresholds) and `docs/autonomy-link.md`
(the Mac link). Where this document and those disagree on a value, the values
in the headers win.

Binding user decisions it implements:

- the camera stays awake through every FLIGHT and BENCH take-off, and its drift
  hold (camera flow + gyro) stabilizes the autonomous hover;
- any phone input lands slowly ("always slow land"); only the Mac KILL and the
  automatic trips cut the motors at once;
- the hover wait is 5 s (`autowait`); on 2026-10-12 (evening) the user
  raised the envelope to a take-off height of up to 0.60 m (`--height 60`)
  and a 10 s wait (`auto set autowait 10000`): `AUTO_HEIGHT_CM_MAX` 60,
  `AUTO_HOVER_WAIT_MAX_MS` 12000, `AUTO_AIR_HARD_MS` 18000, `autoair` 6000
  (2000..8000), docs/autonomy-a2.md revision 5;
- the interactive hover (2026-10-12, late; section 14): take off to up to
  1.20 m, one photo, then wait up to 60 s for each MOVE, YAW or STOP the
  Mac sends (every accepted one restarts the wait), 80 s of air in all
  (120 s since 2026-10-14, section 18.1),
  LAND `BATT_LOW` at the end of the battery (3.30 V for 1 s at first; since
  2026-10-14 a reading below 3.00 V or a hover thrust over 57000 for 0.5 s,
  section 16): `AUTO_HEIGHT_CM_MAX` 120,
  `AUTO_HOVER_WAIT_MAX_MS` 60000, `AUTO_AIR_HARD_MS` 80000;
- longer flights and the palm catch (2026-10-14, section 18): the air clock
  120 s (`AUTO_AIR_HARD_MS` 120000, the LAND's drift allowance 2.0 m); a
  hover wait of up to 120 s (`AUTO_HOVER_WAIT_MAX_MS` 120000; TAKEOFF byte 7
  carries the wait's bits 16-23; NVS `autowait` u32, its old u16 item still
  read);
- after any reset other than a battery power-on, FLIGHT and BENCH are locked
  until a deliberate physical act: a battery power-on or, since 2026-10-12
  (evening, user decision), `auto unlock` typed on the USB console (section 6);
- every autonomous flight is preceded by a props-off bench session (section 11);
- two power changes (2026-10-13, evening; section 15): above 50 % duty the
  motor pulses are placed two at the period start and two at its end, and
  the access point runs at 20 MHz (HT20). The quiet Wi-Fi option stays off.
- a hover without drift (user request 2026-10-14, afternoon; section 20): the
  camera drift hold's gains scale with the scene distance (`holdk`, kmax 2.5)
  in an autonomy sortie until its first MOVE, with new USB trims (facc 0.007,
  atrim -0.60 / 0.00).
- one-step long MOVEs and a diagonal straight from take-off (user decisions
  2026-10-14, evening; section 21): Claude judges the distance to a person and
  approaches in one MOVE of up to 4 m forward (sideways and back 1.5 m, the
  vector 4 m), the drone checks what its camera measures; a TAKEOFF may carry
  a planned move flown from the hover wait's start.

---

## 1. Files

| File | What |
|---|---|
| `modules/src/autonomy.c` | the glue (replaces the step-0 stub) |
| `utils/src/auto_glue_core.c` | reset-lock record, settings table, AUTO_STATUS fill, trip bits |
| `tools/tests/test_auto_glue_core.c` | host test of `auto_glue_core.{h,c}` (155 checks) |
| `tools/tests/test_autonomy_glue_host.c`, `tools/tests/autonomy_glue_host/stub/` | host regression test of `autonomy.c`'s per-loop logic (section 12) |
| `modules/src/cam_vz_glue.c`, `modules/interface/cam_vz_glue.h` | `camVzGluePaused()`; `camVzGlueVzInput()` unusable while paused (integrator-approved, review WP1-R1) |
| `tools/tests/test_cam_vz_glue_host.c` | host test of the pause and bench rules of `cam_vz_glue.c` |
| `modules/src/stabilizer.c` | the dispatcher call, barrier 2, applied thrust, lift-off note, e-stop getter |
| `modules/src/crtp_commander_rpyt.c` | the phone hook |
| `hal/src/imu_level_cal_store.c` | the flash interlock on every NVS write path; `autoSettingStore*` |
| `modules/src/usb_diag.c` | `auto ...` commands (`auto unlock`: the reset-lock's USB unlock, section 6), the busy gates, recorder columns, `flights` |
| `modules/src/flight_recorder.c`, `modules/interface/flight_recorder.h` | 7 autonomy columns (103-byte record) |
| `utils/interface/flight_summary.h`, `modules/src/flight_rtc_log.c` | `autoReason`, `FSUM_END_AUTO` |
| `modules/src/att_trim_auto.c` | no trim save while active; 2026-10-13: autonomy flights learned only after a normal LAND, no samples during a motion (`att_trim_learn.h`) |
| `modules/src/takeoff_aux.c` | `autonomyPoll()` every 10 ms (G9's heap sample, the vzK save); a failed task create is logged |
| `modules/src/controller_pid.c` | `controllerPidGetAttitudeDesired()` (IRAM) |
| `drivers/general/motors/motors.c` | `motorsReadyNoLock()` |
| `hal/src/ledseq.c` | `seq_autoPrespin` (red, 50/50 ms, right after `seq_restartLock`) |

---

## 2. The 1 kHz loop

### 2.1 Order inside `stabilizerTask`

| Step | Code | A2b |
|---|---|---|
| 1 | `sensorsWaitDataReady`, fault hang, `stateEstimator`, `cameraFlowGyroSample`, `commanderGetSetpoint` | unchanged |
| **2** | `autonomyArbitrate(&setpoint, &sensorData, &state, T2M(xTaskGetTickCount()), loop gap)` | new |
| 3 | restart lock, gcap, ground start / take-off window / soft start | unchanged; they only lower thrust; the autonomy SPIN thrust opens the take-off window |
| 4 | `sensorsMpu6050SetThrust`, `sitAwUpdateSetpoint`, `controller()` | unchanged |
| **5** | `control.thrust = autoClampThrust(control.thrust, autonomyThrustMax())` after `checkEmergencyStopTimeout()` | new: barrier 2 |
| 6 | `powerStop()` / `powerDistribution()` | unchanged (thrust 0 stops the motors) |
| 7 | `motorGuardStabilizerAlive()`, `camVzGlueStep()` | unchanged |
| **8** | `autonomyNoteApplied(emergencyStop \|\| !armed \|\| !anyDuty ? 0 : control.thrust)` | new |

`stabilizerInit` calls `autonomyInit()` right after `camVzGlueInit()`, before
the task is created.

### 2.2 `autonomyArbitrate()`

IRAM. Each call:

1. `loops++`; lock-free reads of the mailbox (`autoMbPending`), the KILL
   counter and the session counter.
2. **Fast path:** `autoIdleSkip()` is true in IDLE with action NONE, nothing
   pending, no new KILL or session, except on every 100th loop: return. This
   is manual flight, nearly all the time; it touches no flash code.
3. **Fast inputs** (IRAM, every full step): time, KILL count, session change,
   heartbeat age, the phone hook's ages and input count, attitude, tilt
   (`autoTiltDegFromGravZ(sensfusion6GetInvThrustCompensationForTilt())`, no
   libm `acosf`), attitude error against `controllerPidGetAttitudeDesired()`
   (trim and camera correction included; a NaN is kept), gyro, `|acc|`, loop
   gap. Every age is `autoAgeMs()` of a `T2M(xTaskGetTickCount())` stamp, so a
   stamp written on the other core after `nowMs` was read is 0 old.
4. **`stepFull()`** (IRAM since the bench fix, section 2.6):
   - the previous step's `vzOutput_t`;
   - **slow inputs** on every IDLE full step and every 10th loop while active
     (100 Hz): `pmGetBatteryVoltage()`, `flightRtcLogLiveVbatMin()`, camera
     state/fps, the age of the last frame with a valid fit (`autoAgeUsToMs`),
     `camViewMs`, the drift hold (enabled; applied = engaged and fresher than
     300 ms, as controller_pid.c; coasting), `stabilizerOnGround()` and its
     duration, the locks (`stabilizerRestartLocked`, reset-lock, e-stop,
     `motorsReadyNoLock() && !motorsIsInhibited()`, motor watchdog), gcap,
     the internal heap as the aux task last sampled it (`autonomyPoll()`,
     every 10 ms; a sample older than 200 ms, or none since boot, reads 0 B,
     so a stalled or missing aux task fails G9), the RAM settings;
   - **the command** when one is pending: `autoMbTake()` under the glue's
     spinlock. An NVS write or a console change in progress (`flashBusy`:
     the flash interlock is held, section 7.3) makes G1 fail for that step
     (`in.motorsReady = false`), so an ARM is refused GUARDS/LOCKS. On an
     ARM step the commanded mode picks the plant (G12 sees it), FLIGHT never
     the sim; for the modes whose controller reads the camera height
     (FLIGHT, BENCH-imu) a `camvz air` bench mode or a paused observer
     (`camvz off`, `camVzGluePaused()`) makes G1 fail; and
     `randToken = esp_random() | 1`;
   - `autoFsmStep()`;
   - **one-step actions:** a new sortie latches the plant, the commanded mode
     (the glue's own copy: `autoFsm_t` is private), `vzK`, `vzBair`,
     `vzcaplo`, the resting battery and `autowait`, and clears the capture
     state; `yawHold` follows the
     heading in ARMED and pre-spin and freezes at the motors' start;
     `vzReset` starts the controller (2.4); `captureStart` posts a capture
     request; `lockAppThrust`; set / clear e-stop; the pre-spin LED on its
     edges (retried if the LED queue is full); `cameraFlowSetTakeoffStandbySkip`
     on every change of `camNoStandby`; the command's result published on the
     step that took it (after the capture request, so a PHOTO's set exists
     before its ACK); on a transition the RTC record, and on reaching IDLE the
     vzK learning (FLIGHT, LAND -> IDLE), the controller stopped, the lift-off
     hint cleared and an untaken capture request withdrawn;
   - **the controller** (2.4) from `vzReset` until IDLE (also in LAND after
     touchdown and in KILLED);
   - **the status snapshot** (2.5) on every 100th loop (10 Hz), on events in
     IDLE, on a taken command and on every transition.
5. **Override** (IRAM) unless the action is NONE: `autoBuildSetpoint()` writes
   the whole setpoint, roll and pitch exactly `0.0f` in `modeAbs` (the drift
   hold engages only on exact zeros), x/y/z disabled, yaw `modeAbs` at
   `yawHold`, thrust `autoSelectThrust(action, vo.thrust, thrustMax)`.
6. `thrustMax = autoBarrierMax(action, out.thrustMax)` for barrier 2, the
   action, the state; the profile.

In the 600 ms release hold after a sortie (IDLE, action ZERO) the FSM steps
every loop and the slow inputs are read on each of those steps; the snapshot
stays at 10 Hz.

### 2.3 Barriers

| | Barrier 1 (override) | Barrier 2 (`control.thrust`) |
|---|---|---|
| NONE (manual) | setpoint untouched | 65535: untouched |
| ZERO, CUT (ARMED, pre-spin, release hold, landed, KILLED) | 0 | 0 in every mode |
| VERT_CTRL / VERT_LAND | `min(vo.thrust, mode max)`, NaN/negative 0 | mode max: FLIGHT `VZ_THRUST_ABS_MAX` (58000; 54000 until 2026-10-14), DRY 0, BENCH `min(gcap at ARM, 15000)` lowered by a lower gcap |

`autoClampThrust` fails closed: a NaN or out-of-range limit gives 0; below
65535 a NaN or negative thrust gives 0. In DRY both barriers give 0 in every
state, so a bug in the override cannot spin a motor. KILLED also sets the
stabilizer's emergency stop and holds thrust 0 itself (other code may clear
the flag).

### 2.4 Controller wiring

On `vzReset`: `vzDefaultConfig()`, then `hTarget` from TAKEOFF, `kPrior` =
`vzK`, `bAir` = `vzBair` x 1e-4 g, `capLo` = `vzcaplo` / 1000, all three as
latched on the ARM step; `vzInit()` (an
invalid config is CUT, the FSM kills); the output zeroed; the lift-off hint
cleared; the sim set up on DRY and BENCH-sim (section 9).

| Input | FLIGHT, BENCH-imu | DRY, BENCH-sim |
|---|---|---|
| dt | IMU interrupt timestamps (vz clamps 0.2..5 ms) | same |
| accZ | `sensfusion6GetAccZ(acc)` | `sim.accZ` |
| gravZ, tilt | gravZ, the FSM's tilt | 1, 0 |
| gyroMaxDps | max \|g\| (NaN kept) | 0 |
| vbat | the 100 Hz copy of the 10 Hz reading | the sim's, sampled every 100 ms |
| thrustApplied | what the motors got (step 8) | the previous `vo.thrust` (what the sim got) |
| camera | `camVzGlueVzInput(vo.g0, ...)` (unusable in a bench mode or while paused) | the sim's camera model |

After `vzStep`: on real inputs `camVzGlueSetLiftoffHint(vo.liftedOff)`, on
the sim `vzSimStep(vo.thrust)` (unclamped: BENCH motors get the bench cap);
`stabilizerNoteLiftoff()` on `vo.liftoffNow` in FLIGHT and BENCH (the mode
latched at ARM).

Hold block (hops of 2026-10-12: in hop 2's bounce the camera hold read the
motion as drift and slewed to its 2.5 deg limit, then the drone tipped to
28 deg). Every loop, `holdBlock = state == LAND && (holdBlock || (vzActive &&
(vo.contact || vo.touchdown)) || |acc| > AUTO_TD_CONTACT_G)`: latched at the
loop rate because an impact lasts 1-3 recorder samples and the hold steps
once per camera frame. `autonomyHoldBlocked()` (IRAM, one load) is read by
`controllerPid()`, which zeroes the applied camera correction at once, and
by `camera_flow.c`'s `holdUpdate()`, which then treats the drone as not
airborne (the hold disengages and clears its integrals). It clears when the
sortie leaves LAND.

**Take-off integrator hold** (user decision 2026-10-13, evening; the
take-off wind-up study of 2026-10-12). `controllerPid()` has always held the
four roll/pitch integrators (attitude and rate) at zero below 26000 thrust;
an autonomous RAMP (13250/s) spends ~0.4-0.8 s above it before
vert_ctrl's lift-off (the 2026-10-12/13 autonomous take-offs: 0.37-0.80 s
from 26000 to `vzPhase` 3; the glue sim: 758 loops), and a drone resting
tilted on the floor wound them there: on the manual floor take-offs of
2026-10-12 (rest roll error +3.1..+3.6 deg) the rate integrators reached
-765..-1843 by the lift-off, ~85-90 % of it through the attitude P path, and
kicked the drone 0.7-1.4 deg past level (the drone also starts to move at
34-41k there); on an autonomous floor take-off the model predicts 2.9 deg
past level and -20 cm sideways at 2 s with the camera hold (-38 cm if the
floor pins it to 36000). `autonomyTakeoffIntegratorHold()` is true from
`vzReset` until `vo.liftedOff` (latched) and false in IDLE, so every manual
flight is untouched; it reads the glue's own stabilizer-task state
(`vzActive && !vo.liftedOff && vo.phase <= VZ_RAMP && !vzRampAtTop(&vz)`,
in flash: a flag stored in `stepFull` cost 134 B of IRAM in register
spills), and `controllerPid()` (after `autonomyArbitrate()` in the same
loop) holds the four integrators while it is true, as below 26000.
*Bounded* (after the verification of e4f7686, same evening): it also ends
when the RAMP reaches its plateau (`vzRampAtTop()`: the ramp at
`rampTopRel` x kPrior / Vf, the hover thrust of the stored gain) and in
every phase after RAMP without a lift-off (the abort LAND, DONE, CUT; a
KILL), vert_ctrl's own "the drone may be up" cases. Without the bound a
lift-off vert_ctrl never latches (a residual in-flight accelerometer
offset; in the vert_ctrl sim 7 / 74 / 99 of 100 sorties at -10 / -15 /
-20 mg) flew P/D-only through the plateau's float and the abort LAND's
descent: up to 3.75 s airborne, up to 0.134 m high, with a steady lean of
(need - trim) / 1007, 1.3-2.5 deg. With it: at most 0.87 s airborne before
the release, up to 0.056 m high (`test_vert_ctrl` `testRampAtTop`: the real
vert_ctrl.c on its plant, 40 seeds per case, -10 to -30 mg; the plateau at
RAMP 1.86 s there); no latched lift-off came after the plateau (K x
1.00-1.12 and every offset), so a detected lift-off is never released
early. The logs keep the
protection: the 13 autonomous take-offs of 2026-10-12/13 latched at
0.76-0.88 x the plateau thrust (RAMP 0.82-1.20 s, recorder clock
corrected), 0.39-0.72 s before the plateau. A drone that reaches the
plateau still on the floor winds the integrators there as before this
change (it then lands `NO_LIFTOFF_TIME` without leaving the floor, or
lifts off late with that wind-up, as on e46bf90). P, D, yaw and the
torque-trim feed-forward act as before (the trim ramps in from 20000 to its
full value at 30000, before any lift-off). Released on the lift-off step, the
integrators start from zero: the hover need beyond the stored torque trim
(the 2026-10-13 autonomous hovers: pitch +880..+2550, roll -190..+1150)
then builds in the rate integrators (tau ~0.43 s) instead of being
pre-loaded by the ground wind-up (which today happens to pre-load pitch
+760..+2070 and roll the wrong way on a floor). Expected, on the build's own
controller closed around the plant validated on the logs (b 0.22, rotor lag
0.01 s; RAMP through 26000, release at 32000, lift-off at it, hover need
roll +750 / pitch +1300 above the trim): roll 2.88 -> 0.36 deg past level,
-20.3 -> +2.5 cm at 2 s (pinned to 36000: 4.95 -> 0.36 deg, -37.7 -> +2.6
cm); pitch, the torque shortfall at release: peak -0.53 -> -0.80 deg,
+2.3 -> +4.6 cm at 2 s (-1.09 deg, +5.7 cm if the lift-off is detected
0.3 s late); with the stored trim matching the need: roll 0.21 deg, pitch
-0.15 deg. On the logged take-offs (counterfactual replay): the floor
take-offs' kick past level 0.72-1.40 -> 0.34-0.89 deg and +5.8..+13.3 cm
at 2 s toward cancelling it; the stool hops (rest tilt the other way) get
slightly worse: ~-5 cm, pitch peak -0.2..-0.5 deg larger. The controller
model reproduces the logged I terms within 0.17 deg/s and 216 (rate) and
the logged attitude after the release within 0.04-0.26 deg RMS. Not in this
change (user decision: no other tuning): the torque trim; read the need from
the first flight (the rate integrators' mean over lift-off +1..2 s) and set
it as a separate step (`trim set` / `trim save`). The bench cannot exercise
it (the bench cap and DRY stay below 26000): the flight recorder's four I
terms exactly 0 above 26000 before `vzPhase` 3 and nonzero after it are
the first evidence. Tests: `test_controller_pid_ihold` (the real
controller: held, all four exactly 0 on every loop above 26000 before the
lift-off and growing after it, the trim applied, manual flight bit-identical
to e46bf90's controller over a 20 s sequence; 5 of its 12 checks fail on
e46bf90), `test_autonomy_glue_host` scenario `ihold` (the real glue: held
on every spinning TAKEOFF loop before the lift-off note and above 26000,
released from it on, never in manual flight or IDLE; a heavy plant that
never lifts off: held until its RAMP's plateau and never after it, not in
the float nor the NO_LIFTOFF LAND; a KILL in the ramp releases it at once;
24 checks, 3 of them fail on e4f7686's unbounded glue; it does not build on
e46bf90).

### 2.5 AUTO_STATUS snapshot and seq

`autoStatusFill()` and `autoTripBits()` (IRAM) fill a staging copy; under the
spinlock, in this order: on a transition `autoMbSetState()` (the flash
interlock's state), the 64-byte copy, then on a transition `statusSeq++`.
`autonomyGetStatus()` returns the snapshot and its seq under the same lock.
`hoverWaitMs` is the `autowait` latched at ARM (in IDLE the current setting);
the vz fields are 0 (`VZ_IDLE`) outside a sortie.

### 2.6 Cost, IRAM and stack (as built)

Clean build against step 0 (`idf.py size`, the same sdkconfig, ESP-IDF
5.0.7, -O2), no new compiler warnings (the same 13 deprecation and
unused-variable warnings as step 0):

| | Step 0 | WP1 | Change | Budget |
|---|---|---|---|---|
| Static IRAM | 104626 | 106234 | +1608 | |
| IRAM .text | 103599 | 105207 | +1608 | +2560 |
| D/IRAM .data | 31360 | 31440 | +80 | |
| D/IRAM .bss | 107528 | 109632 | +2104 | .data + .bss: +2184 of +2600 |
| Flash .text | 701231 | 731555 | +30324 | |

(After the review fixes: the settings claim flag and the sortie's latched
mode and vz settings add 24 B of .bss; IRAM is unchanged.)

**Per-loop code in IRAM (bench fix, 2026-10-12).** The first A2b build ran
`stepFull()`, `autoFsmStep()`, `vzStep()` and `vzSimStep()` from flash: on the
bench (Mac and phone linked, photo pushes) the stabilizer fell to 630-700
loops/s during sorties (`activeAvgUs` 337, max 938), the 16 KB instruction
cache being shared with the camera's flow fit. Now in IRAM (`AUTO_HOT` in
vert_ctrl.c, auto_fsm.c, auto_glue_core.c; `IRAM_ATTR` elsewhere): `stepFull`,
`vzRun`, `slowInputs`, `publishStatus`; `autoFsmStep` with the helpers GCC
keeps out of line (`inputsFinite`, `stopCmd`, `autoFsmGuards` on every ARMED
step, `groundExit`, `trip`, `enter`); `vzStep` with `startLand` and
`velocityLoop`; `vzSimStep`, `autoTripBits`, `autoStatusFill`,
`camVzGlueVzInput`, `camVzGlueSetLiftoffHint`, `sensfusion6GetAccZ`,
`sensfusion6GetInvThrustCompensationForTilt`; the getters `slowInputs` reads
(battery, `flightRtcLogLiveVbatMin` with `flightRtcLogLive`, camera
state/fps/hold, stabilizer locks/ground/gcap, motor ready/inhibit, motor
guard, `thrustSoftStartActive`); `expf` with `finitef` (linker_camvz.lf).
The internal heap for G9 is sampled by `autonomyPoll()` in the aux task,
with its time: `slowInputs` takes a sample older than 200 ms, or none, as
0 B, so a stalled or missing aux task refuses ARM and TAKEOFF (HEAP) instead
of passing an old value (`takeoffAuxInit()` logs a failed task create; a
failed Wi-Fi TX-power change holds the aux task 500 ms). About +16.7 KB of
IRAM, paid for by moving the camera's gyro ring (12 KB) and the two register
snapshots (4.2 KB) to PSRAM: the ring is allocated by `cameraFlowInit()` only
when the camera is enabled (PSRAM only; without it the camera stays
`failed`), the boot snapshot at boot and the other on the first `regs`
command. Free internal heap ~22-24 KB with the Mac and phone linked, as
before. Measured, DRY sorties with `camvz air full`: `activeAvgUs` 34-35, max
225-335; 948-973 loops/s in TAKEOFF/HOVER/LAND against 983-990 in IDLE. The
plan's go/no-go of 975 is replaced by 940: the user signed off 940 on
2026-10-12, before bench phases B and C, on these numbers and on manual
flights that flew well at ~957 loops/s (2026-10-11). Any lower gate needs
the user's sign-off again (section 11 step 2).

After the review of that build (0cf6fbf) the last per-loop flash calls
moved into IRAM too (the out-of-line FSM helpers above, the tilt getter,
`flightRtcLogLive`, `finitef`) and the heap sample got its time: +1200 B of
IRAM (static IRAM 124258 B; step 0: 104626), `_heap_start` +1288 B (IRAM
ends on a 256-byte boundary), so expect the free internal heap about 1.3 KB
below the figures above. Neither the heap nor the loop rate of this build is
measured on the drone yet. The WP1 IRAM budget in the table (+2560) predates
the bench fix, which exceeds it by design.

IRAM functions of the dispatcher: `autonomyArbitrate` (about 1.5 KB, with
the inlined override, barriers, fast path, tilt and ages),
`autonomyThrustMax`, `autonomyNoteApplied`, `controllerPidGetAttitudeDesired`.
The dispatcher calls no flash code on its fast path. The call graph of the
review build from `autonomyArbitrate` (disassembly and literal pools): no
flash code runs on every loop in any state, on the DRY/BENCH-sim path
(`vzSimStep`, `expf`) or on the FLIGHT/BENCH-imu path (`camVzGlueVzInput`,
`sensfusion6GetAccZ`, `camVzGlueSetLiftoffHint`); `sqrtf`,
`esp_timer_get_time`, `xTaskGetTickCount`, `memset`, `memcpy` and the
soft-float helpers are IRAM/ROM. What stays in flash runs on events only: a
command (`autoMbTake`, `autoMbPublish`; on an ARM `camVzGlueBenchAir`,
`camVzGluePaused`), a transition (`autoMbSetState`, `autoRtcNoteTransition`,
the e-stop set/clear, `crtpCommanderRpytLockThrust` and
`commanderDropSetpoint` on reaching IDLE, `cameraFlowSetTakeoffStandbySkip`,
`ledseqRun`/`ledseqStop` on the pre-spin LED's edges), once a sortie
(`vzStart`, the double `sqrt` at the end of SPIN, `stabilizerNoteLiftoff`,
`vzLearnK`), and `expf`'s `__math_oflowf`/`__math_uflowf` (a result beyond
the float range only). The stabilizer task's own loop, `cameraFlowGyroSample`
included, is flash code as before A2b.

Stack frames (windowed ABI `entry`) on the new chain under the stabilizer
task: `autonomyArbitrate` 96, `stepFull` 96, `slowInputs` 144, `vzRun` 32 (48 before the review fixes),
`vzStep` 144, `vzStart` 32 + `vzInit` 32 + `vzConfigValid` 224 (vzReset
only), `autoFsmStep` 80, `publishStatus` 80. Deepest: about 0.6 KB above the
stabilizer loop's own frame. Every FSM and controller struct is static. `auto
prof` reports the lifetime high-water mark (bench: >= 1024 B free).

**The 2026-10-13 fixes** (docs/autonomy-a2.md section 12; 910cee0 ->
the docs commit of that series, same sdkconfig, no new warnings):

| | 910cee0 | after | Change | Budget |
|---|---|---|---|---|
| Static IRAM | 130846 | 131054 | +208 | +256 |
| D/IRAM .bss | 87248 | 87304 | +56 | +64 |
| D/IRAM .data | 31464 | 31464 | 0 | |
| `_heap_start` | 0x3fcc0fb8 | 0x3fcc0ff0 | +56 | |
| Flash .text | 725475 | 723447 | -2028 (inlining) | |

- IRAM: `autoFsmStep` +156 (the contact trip: one `track()` per loop, the
  arming read only once it has held), `autoFsmGuards` +14 (the heat soak),
  `slowInputs` +12 (the last flight's motor time, a word read without a
  call), `camVzShadowStep` and `camVzTask` +4 each (the observer state 8 B
  longer, so the shadow's fields behind it moved), the rest unchanged; the
  observer's new fields sit in padding and after
  its counters, the shadow keeps its layout (`tImpact` now holds the
  cluster's latest impact), the FSM's new timer is the last field. The text
  ends 1 B below the 256-byte boundary the linker rounds IRAM up to (with its
  16 B prefetch pad): `_iram_text_end` stays 0x40394000, so the heap loses
  only the 56 B of .bss (one more byte of IRAM would cost 256 B of heap).
- .bss: `holdScene` 24 (the hold's scene gate, camera task), the glue's two
  status copies of the observer counters +8 each (`reanchors`), the FSM +8
  (the contact timer), `fin` +4 and the stabilizer's motor-time word +4 (the
  heat soak).
- **LAND `COLLISION`** (user decision 2026-10-13, docs/autonomy-a2.md 12.8;
  eb9264d -> its commit): IRAM `autoFsmStep` +28 B, `vzStep` +52, `startLand`
  +8, `stepFull` +6, `vzRun` +4, `enter` +4, paid by moving the ARM and
  TAKEOFF command handlers (`armCmd`, `takeoffCmd`; once per command, motors
  off) out of line to flash (-428 B). Static IRAM 131054 -> 130726 B
  (-328), `_iram_text_end` 0x40394000 -> 0x40393f00 (73 B of slack), .bss
  +24 B, `_heap_start` 0x3fcc0ff0 -> 0x3fcc0f08: 232 B more internal heap at
  boot (expect ~0.2 KB above eb9264d's free heap after a reset). The IRAM graph
  from `autonomyArbitrate` reaches the same functions; its only new flash
  targets are `armCmd` and `takeoffCmd`.
- **LAND `COLLISION` during vertical motions** (the verification of 1c2a43f;
  autonomy-a2.md 12.8 "Contacts during vertical motions"): the blind LAND's
  hover reference from before the last vertical move or bob, the reference
  height's flat part after a move down. IRAM `holdQuiet` 46 B (new; inlined
  it cost `vzStep` 84 B of spills), `startLand` +8, `vzStep` -3; `blindLand`
  in flash behind `landTimes` (once per LAND). Static IRAM 130726 -> 130774 B
  (+48), the text 25 B under `_iram_text_end` 0x40393f00 (unchanged); .bss
  +8 (`vz`); `_heap_start` 0x3fcc0f08 -> 0x3fcc0f10 (8 B less heap than
  1c2a43f, 224 B more than eb9264d). The IRAM graph from `autonomyArbitrate`
  adds `holdQuiet`; its flash and ROM targets are 1c2a43f's.
- Per frame, flash: `camHoldSceneOk()` (a few flops, camera task) and the
  observer's re-anchor test (camvz task); the flow fit is untouched, so the
  camera's cadence is unchanged. The re-anchor's climb window and gate guard
  (2026-10-13 review) add flash only (`camVzObsFrame` +240 B, a few compares
  per frame): their state (two flags and a 16-bit time) sits in the observer's
  padding, so IRAM, .bss, `_heap_start` and every IRAM symbol are unchanged
  (`callgraph.py` from `autonomyArbitrate`, `stabilizerTask`,
  `camVzShadowStep`, `camVzTask` and `flightRecorderSample`: the same graph).
  Nothing new runs per loop except the contact `track()` and its test (IRAM);
  `callgraph.py` from `autonomyArbitrate` finds no new per-loop flash target.
  The recorder's scene-gate getter is flash (the recorder sample is flash
  code).

`auto prof` times the quiescent path on every 16th loop (`fastMaxUs`), every
full IDLE step (`idleAvgUs`, `idleMaxUs`) and every 16th active step
(`activeAvgUs`, `activeMaxUs`), and counts active loops per second
(`activeLoops / activeMs`). The bench targets and the go/no-go are in
section 11.

---

## 3. Link side (what the glue offers the link task)

- **Mailbox** (`auto_mailbox.{h,c}`, step 0) under the glue's spinlock:
  `autonomyPostCommand` (stamps the post for the latency profile),
  `autonomyTakeResult`, `autonomyCancelCommand`. The stabilizer takes at most
  one command per step and publishes its result on the same step.
- **KILL:** `autonomyKillFromLink()` stops spinning motors at once
  (`motorsEmergencyStopNoWait()`, no mutex, never blocks) and increments the
  KILL counter; the FSM sees it on the next loop in any state (KILLED, or the
  2 s kill latch in IDLE).
- **Session / heartbeat:** `autonomyNoteSession()` (sessionValid, a session
  change on the next step), `autonomyNoteHeartbeat()` (stamped here).
- **Photo sets the FSM starts:** `autonomyTakeCaptureRequest()` once per
  request; `autonomyNoteCapture(reqId, BUSY|DONE|FAILED, photoId)`; reports for
  an older request are ignored. A request the link has not taken is withdrawn
  when the sortie ends.
- `autonomyCameraAwake()`: the take-off standby skip is applied (the capture
  need not wait for the window).
- Status: `autonomyActive`, `autonomyState`, `autonomyStatusSeq`,
  `autonomyGetStatus` (2.5).

---

## 4. Phone hook

`crtpCommanderRpytDecodeSetpoint()`, camera build, right after the link
statistics, on the bounded values: `autonomyNoteAppPacket(roll, pitch,
rawThrust)`. Invalid packets return earlier and are not input. The hook stamps
`T2M(xTaskGetTickCount())` itself and never changes the setpoint:

- input: raw thrust >= 1000 or \|roll\| or \|pitch\| >= 2 deg (yaw ignored),
  counted atomically;
- consent gesture: \|roll\| >= 10 deg, then released below 2 deg;
- thrust zero: the last raw thrust < 1000.

The FSM lands slowly on any input while airborne (`APP_INPUT`) and on app
silence over 500 ms with `appreq` (`APP_SILENT`); the phone never cuts.

---

## 5. Take-off window and the camera

- `stabilizerNoteLiftoff()` (stabilizer task) sets the window's lift-off for
  the current ground start if none is recorded, so the window ends at
  max(3 s after the ground start, lift-off + 1.5 s) and the camera's phase at
  lift-off + 0.5 s, without waiting for the 40000 thrust level. It is called on
  the controller's detected lift-off in FLIGHT and BENCH (the sim's in
  BENCH-sim); DRY has no ground start.
- `cameraFlowSetTakeoffStandbySkip(out.camNoStandby)` on every change.
  `camNoStandby` is true ARMED..HOVER of every FLIGHT and BENCH sortie (WP4's
  `auto_fsm.c`; with the A2a state machine only FLIGHT at >= 4.00 V), so the
  skip is set before the SPIN's ground start and the camera never sleeps in an
  autonomous take-off.
- The camera drift hold engages in autonomous flight without changes to
  camera_flow.c, camera_hold.c or controller_pid.c: the override writes
  roll/pitch `0.0f` with x/y disabled (the recorder reads 0, the controller
  applies the hold), yaw is held absolute (yaw rate ~0), and the hover thrust
  is above the hold's 30000 airborne level. FLIGHT STABLE needs the hold
  applied and not coasting with a frame <= 300 ms old (A2a: 200; the
  2026-10-12 hops' ages reach 196-197 ms), so photos are only taken while
  the hold is engaged on a live view. A STABLE streak survives a dropout
  shorter than 150 ms (a single weak frame makes the hold coast ~75 ms)
  while its dropouts stay below 300 ms in all (`AUTO_STABLE_DROP_BUDGET_MS`:
  at STABLE 1 s the hold was engaged on a live view >= 70 % of the streak;
  the 2026-10-12 hops that pass spend 0-150 ms of it in dropouts, while a
  hold that coasts 93 % of the time no longer passes), but never a camera
  that leaves RUNNING (docs/autonomy-a2.md section 3.5).
  `camViewMs` (the NOT_STABLE view) still counts frames <= 200 ms old
  (`AUTO_VIEW_FRAME_AGE_MS`).

---

## 6. Reset-lock RTC record

- `static RTC_NOINIT_ATTR autoRtcRecord_t rtc` (96 B; linked at 0x50000520,
  right after crash_log's record; the flight store moves, see below).
- `static volatile bool resetLocked = true` (.data): `autonomyInit()` clears
  it only when `autoRtcBootLocked(reset == POWERON, &rtc)` is false, i.e. a
  power-on with the magic absent; after init only `autonomyUsbUnlock()`
  (below) clears it. A skipped or reordered init leaves FLIGHT and BENCH
  locked. G1 exempts DRY.
- `autoRtcStartBoot()`: keeps the previous boot's last state, reason, mode,
  lock and transition count when the record was valid; `boots` +1 (1 after a
  power-on); a new CRC (`autoProtoCrc32` of bytes 8..95). Every transition
  appends to an 8-entry ring and updates the CRC. The record's `locked`
  (the next boot's `prevLocked`, `auto rtc`): 0 not locked at boot (a battery
  power-on), 1 locked, 2 locked at boot and then cleared by `auto unlock`.
- **Unlock, either of:**
  - a **battery power-on**: unplug the battery **and USB** for 5 s, then plug
    the battery in;
  - **`auto unlock` on the USB console** (user decision 2026-10-12, evening:
    the battery is hard to unplug). The rule's intent stays: after an
    unexpected reset nobody flies autonomously again without a deliberate
    physical act by the user, and a command typed on the USB console, which
    needs the cable in the drone, is that act. `usb_diag.c` is the only
    caller of `autonomyUsbUnlock()`; the console reads only the
    USB-Serial-JTAG port, so nothing on Wi-Fi, CRTP or the autonomy link
    reaches it, and the Mac's `espfly_auto.py` has no unlock
    (`test_espfly_diag` checks both). It clears the reset-lock exactly as a
    battery power-on does (`resetLocked` false: G1 and the AUTO_STATUS flag
    from the next step) and sets the record's `locked` to 2 (new CRC). It
    answers `ERR busy: autonomy active` outside IDLE, in the 600 ms release
    hold or with an ARM in the mailbox, and `ERR auto unlock: <why>` unless
    the motors are stopped (`motorsIdle()`, then every duty 0); nothing
    changes then. It runs inside the flash interlock: an ARM taken meanwhile
    is refused by G1, and the stabilizer, whose only record write in IDLE
    follows an accepted ARM, cannot write the record at the same time. It
    changes **no other lock**: the restart lock (`plock`), the motor driver's
    STOP latch after a KILL (a reset clears it), the motor watchdog trip and
    the emergency stop stay, and the reply shows them:
    `OK auto unlock resetLocked=0 was=1 rst=software (other locks unchanged:
    plock=off rlocked=0 motorsReady=1 estop=0 mguard=on mguardTripped=0)`
    (`was=0`: the boot was not locked, nothing to do). `auto status` then
    gives `resetLocked=0` with `rst=` still the real reset reason; the
    unlock is logged as a warning (the `crash` warning ring keeps it across
    resets). The next reset locks again: the boot rule reads the magic only.
  - **After a flash** (USB is plugged for flashing anyway): flash, wait for
    the boot (about 3 s), `python3 tools/espfly_diag.py auto unlock`, then
    unplug USB. After an unexpected reset (a brownout, panic or watchdog
    reset, `auto status` `rst=`), save `crash` and `flights` first and
    unlock only when the user says so.
- **Before flashing A2b:** save `python3 tools/espfly_diag.py flights` and
  `... crash`. The new RTC variable moves the flight store, whose magic check
  then clears it.

---

## 7. Settings, USB commands, NVS gating

### 7.1 NVS keys (namespace `imucal`)

| Key | Type | Default | Range | Unit |
|---|---|---|---|---|
| `autoon` | u8 | 0 | 0..1 | |
| `appreq` | u8 | 1 | 0..1 | |
| `autogest` | u8 | 1 | 0..1 | |
| `vmin` | u16 | `AUTO_VMIN_DEFAULT` x 1000 (4000 with WP4) | 3700..4200 | mV |
| `autoair` | u16 | 6000 (4500 until 2026-10-12 evening) | 2000..8000 (..6000) | ms |
| `autowait` | u32 (u16 until 2026-10-14: still read, section 18.2) | 5000 | 0..120000 (..60000 until 2026-10-14, ..12000 until the interactive hover, ..8000 until 2026-10-12 evening) | ms |
| `autoaccel` | u16 | 2000 | 0..3000 | ms |
| `vzK` | u32 | `VZ_K_DEFAULT` (165000 with WP4) | 120000..180000 | thrust x V |
| `vzBair` | i16 | -23 | -300..300 | 0.1 mg |
| `vzcaplo` | u16 | 1650 | 1000..2000 | x1000 |
| `photocol` | u8 | 0 (1 until the review of 2026-10-15, 24.7) | 0..1 (1: colour photos, always on for the boot: the split of section 28) | |
| `cvqlate` | u8 | 1 (off) | 1..5 (1..10 until the review, 25.9) | x |
| `cvqlates` | u8 | 40 | 20..120 | s |
| `cvt2k`, `cvt2r` | u8, i16 | 0 (off), 124 | 0..200, -350..350 | kR x 100, ug/s (section 29) |
| `cvhb` | u8 | 0 (off) | 0 or 10..80 (1..9 refused) | the healthy offset speed-up x 10 (section 37) |
| `mvspeed`, `mvacc`, `mvsettle` | u16, u8, u16 | 100, 20, 2500 | 100..300, 10..40, 1000..3000 | the motion speed step and its fast caps (sections 31, 35) |
| `mvfloor`, `mvfstop`, `mvprogm`, `mvprogr` | u8 | 30, 25, 20, 40 | 0 or 20..100, 0 or 15..80, 0 or 10..60, 0 or 10..100 (1..min - 1 refused; the plain ranges until 2026-10-18) | the MOVE guards in cm and %; **0 = off** (section 38; the margins section 32) |
| `mvlatm`, `mvnear`, `mvfix` | u8 | 40, 80, 59 | 15..100, 0..150 (0 off), 0..63 | the sideways margin's floor, the near brake, the move fixes (sections 32-34) |
| `mvklow` | u8 | 60 | 60..100 | the lowest expansion scale k x 100 the floor, the front stop, the near brake and the measured lower bound of D allow for; 100: none (section 38) |
| `mvlat` | u8 | 50 | 0..200 | a sideways MOVE at most this / 100 x D (TOO_WIDE); 0: no cap (section 38) |
| `mvwmax` | u8 | 15 | 10..40 | the expansion rate cap x 100 (/s): a forward move's speed at most this x its end distance at 100 % (section 38) |

`photocol`, `cvqlate`, `cvqlates`, `cvt2k`, `cvt2r` and `cvhb` are read once at boot (camera_flow.c `cameraFlowInit`,
cam_vz_glue.c `camVzGlueInit`; sections 24, 25, 28, 29, 37): a change needs a reboot. `autonomyInit()` loads every key; an absent, mistyped or out-of-range key uses
the default. The RAM copy is what the next ARM uses; the ARM step latches
`vzK`, `vzBair` and `vzcaplo` for the sortie's controller.

**Settings writers.** After init the RAM copy changes only inside a settings
claim (`settingClaim()` in autonomy.c): the flash interlock plus a
one-writer flag, both taken under the glue's spinlock before anything
changes. Inside it come the NVS write (the store brackets it again, nested),
its read-back and then the RAM copy; the claim is released last. So:

- an ARM that came first (taken, or waiting in the mailbox) makes the claim
  fail: `auto set|reset|on|off` answer `ERR busy: autonomy active` with
  nothing changed;
- an ARM taken while the claim is held is refused by G1 (LOCKS, counted in
  `armsBlocked`), so no sortie starts between the NVS write and the RAM
  copy (review WP1-R7);
- an ARM posted after the claim makes the store's own bracket refuse the
  write (`ESP_ERR_INVALID_STATE`): busy, nothing changed, and that ARM is
  refused too while the claim is held;
- the console and the aux task never write at the same time: while one holds
  the claim the other's claim fails (the console answers busy; the aux task
  retries 5 s later).

A learned `vzK` (FLIGHT, LAND -> IDLE, `vzLearnK`) is stored by
`autonomyPoll()` in the takeoff_aux task once in IDLE, out of the release
hold, every duty 0 and on the ground, inside a settings claim (a refused
claim, an interlock refusal or an NVS error retries 5 s later; a console
`auto set vzK` or `auto reset vzK` cancels it, checked again inside the
claim).

### 7.2 USB `auto` commands (never over Wi-Fi)

| Command | Reply |
|---|---|
| `auto` | the A1 link line, then ` state= resetLocked= linkStack=psram\|internal kills= autoCmds= cmdTimeouts= cmdBusy= cmdTakenUnknown= autoStatusTx= autoCaptures= photosPreempted= photoHeldBusy= killRateDrops=` (the link fills them; 0 with the A1 link) |
| `auto status` | `OK auto status state= mode= reason= vz= vzReason= vzLand= guards=0x..(NAMES) resetLocked= rtcValid= rst= prev=STATE/REASON/MODE transitions= kill= session= hbAgeMs= appAgeMs= camAwake= hold= camHeight= sim= waiting= waitLeftMs= airMs= stateMs= vbat= vmin= heap= heapMin= benchSrc= linkStack= mv=STATE/KIND why= end= mvSeq= prog=R/F/U tgt=R/F/U D= dFlags=0x.. hTarget= yawOff= battLow= airLeftDs=` (the interactive hover's fields: cm, deg, 0.1 s; section 14) |
| `auto get` | `OK auto get autoon=0(default) appreq=1(nvs) ...`; the source is read from NVS in IDLE only (`ram` while active) |
| `auto set <key> <value>` | `OK auto set <key>=<v> stored=nvs`, `ERR busy: autonomy active`, `ERR auto set: <key> must be <min>..<max> <unit>`, `ERR auto set: NVS write failed (<key> unchanged)` |
| `auto reset <key>` | `OK auto reset <key>=<default>(default) stored=erased` |
| `auto on` / `auto off` | as `auto set autoon 1/0` |
| `auto rtc` | `OK auto rtc locked= valid= rst= prev= prevLocked= prevTransitions= transitions=`, `T <i> ms= from= to= reason= vz=` lines, `END auto rtc n=`; `prevLocked=2`: the boot before was unlocked by `auto unlock` |
| `auto unlock` | clears the reset-lock (section 6): `OK auto unlock resetLocked=0 was=<0\|1> rst= (other locks unchanged: plock= rlocked= motorsReady= estop= mguard= mguardTripped=)`, `ERR busy: autonomy active`, `ERR auto unlock: <why>` (motors not stopped) |
| `auto bench sim\|imu` | BENCH input source, RAM only, IDLE only |
| `auto prof` / `auto prof reset` | `OK auto prof loops= full= fastMaxUs= idleAvgUs= idleMaxUs= activeAvgUs= activeMaxUs= activeLoops= activeMs= activeLoopsPerS= cmdLatMaxMs= stabStackFree= flashRefused= armsBlocked= upMs=`; the reset happens at the next IDLE step |

`flights` F lines end with ` auto=<reason>` when `end=4`.

### 7.3 Gating while active

- **Flash interlock.** Every NVS write, commit and erase in the firmware is in
  imu_level_cal_store.c (`saveBlob`, `eraseKey`, `saveFlag`,
  `thrustCapStoreSave`, `motorPwmStoreSave`, `yawLockStoreSave`,
  `autoSettingStoreSave`; `autoSettingStoreErase` uses `eraseKey`). Each opens
  with `autonomyFlashBegin()` (refused: `ESP_ERR_INVALID_STATE`, flash
  untouched) and calls `autonomyFlashEnd()` on every exit after it. Begin
  refuses unless the published state is IDLE and no ARM is in the mailbox; an
  ARM taken during a write is refused by G1. Both decisions are under one lock
  (`test_auto_mailbox`). The read-backs run after the bracket.
- **Console gates.** Every command that writes NVS, inhibits the motors or
  starts a bench mode answers `ERR busy: autonomy active`: cal, trim, facc and
  atrim set/clear/save/learn, cam, hold, yawlock, tkoff, pwm, soft, plock,
  autokey, mguard on/off/test, reboot, rail, measure, crash test, rec
  dump/clear, `camvz off`, `camvz batch`, `camvz air on|full|spin`, `auto
  set|reset|on|off|bench|unlock`. `motorsIdle()` also answers "autonomy active" first.
  Allowed while active: help, status, stream, live, net, regs, the read forms
  (`auto`, `auto status|get|rtc|prof`, `camvz`, `camvz prof`, `camvz air off`,
  crash, flights, `rec info`, `mguard status`, `cal|trim|atrim|facc show`,
  `gcap`, `tkoff`, ...). Read forms skip their NVS reads while active (an NVS
  read also stops the flash cache): `auto` prints `nvsFp=active`, `cam` prints
  `boot=?`, `tkoff` prints `wifiq=0`.
- **Console changes inside the interlock (review WP1-R2).** The gate above
  is checked before the change, so on its own it leaves a gap: an ARM taken
  between the check and the change would start a sortie that sees the
  change. So every console change a sortie could read is made inside the
  flash interlock, taken before anything changes (`changeBegin()` /
  `changeEnd()` in usb_diag.c, which call `autonomyFlashBegin()` /
  `autonomyFlashEnd()`; autonomy.h lists only imu_level_cal_store.c as their
  caller, and this is the one other): cal/trim/facc/atrim set and clear, atrim
  learn, hold, yawlock, tkoff, pwm, soft, plock, gcap, mguard on/off, `camvz
  off|batch|air on|full|spin` and `auto bench`. An ARM that came first (taken,
  or in the mailbox) makes the change answer `ERR busy: autonomy active` with
  nothing changed; an ARM taken while it runs is refused by G1 (LOCKS). Inside
  the interlock the NVS write comes first and the live value changes only
  after it succeeded, so a write refused by the store's own bracket (an ARM
  posted meanwhile: `ESP_ERR_INVALID_STATE`) also answers busy with nothing
  changed (`pwm` sets the driver first, because the driver may refuse, and
  puts the previous frequency back). Every store call maps
  `ESP_ERR_INVALID_STATE` to `ERR busy: autonomy active` (`storeBusy()`), also
  the save-only forms (cal, trim, facc, atrim save, cam, autokey). The `hold`,
  `yawlock` NVS-failure replies are now `NVS save failed (...); unchanged`
  (the live value is no longer changed first). An ARM sent while any of these
  runs is refused GUARDS/LOCKS: wait for the reply before arming.
- **`camvz off`** pauses the camera height observer: no blocks reach it and
  its snapshot freezes in its last phase (CVS_AIR if paused in the air,
  review WP1-R1). It now needs `motorsIdle()` as well (only on the ground),
  `camVzGlueVzInput()` reports the estimate unusable while paused, and an ARM
  for FLIGHT or BENCH-imu is refused GUARDS/LOCKS until `camvz on`, like a
  `camvz air` bench mode. DRY and BENCH-sim fly the sim's camera model, so
  neither affects them. `camvz on` is always allowed.
- **`autokey`**: `autokey <anything>` answers `ERR busy: autonomy active`
  while active before it parses the argument (review WP1-R4); the bare
  `autokey` read form still answers.
- **gcap while active:** in BENCH, `gcap N` with 0 < N < the current cap is
  applied in RAM (`OK gcap N stored=ram note=autonomy_active`; the FSM lowers
  its bench cap to it). Raising, `off`, `0`, and any gcap in FLIGHT or DRY:
  `ERR busy: autonomy active`.
- `att_trim_auto.c` does not save while active; a refused save keeps
  `savePending` and retries after the sortie.

---

## 8. Recorder and flight summary

- `flightRecord_t` (103 B) appends `autoState`, `autoReason`, `vzPhase` (u8),
  `vzHmm`, `vzVmms` (i16), `vzThrust` (u16), `autoFlags` (u8, `AUTO_STF_*`) from
  `autonomyRecordFields()`; all 0 in IDLE outside a sortie.
- Recording runs while thrust is requested **or autonomy is active**, so DRY
  sorties are recorded; `flightId` increments when recording starts.
- `RECORD_FIELDS` appends `autoState,autoReason,vzPhase,vzH_mm,vzV_mms,vzThrust,autoFlags`.
- The interactive hover (section 14) appends `mvState` (u8), `mvRefLat`,
  `mvRefFwd` (i16, the hold's reference x 1e4), `mvVRef` (i16, mm/s) and
  `mvYawOff` (i16, 0.01 deg): 112-byte records, 672 KB of PSRAM;
  `RECORD_FIELDS` appends `mvState,mvRefLat_e4,mvRefFwd_e4,mvVRef_mms,mvYawOff_cdeg`.
- `flightSummary_t.pad` is `autoReason` (same offset; old summaries read 0).
  A flight whose closing sample (or the one before it) was not IDLE ends
  `FSUM_END_AUTO` (4) with that sample's reason (the LAND or KILLED trigger);
  summaries still open on thrust only, so DRY makes none. `FLIGHT_RTC_MAGIC`
  is unchanged (same layout).

---

## 9. DRY and BENCH sim plant

- `simPlant = mode == DRY || (mode == BENCH && benchSrc == SIM)`, decided with
  the commanded mode on the ARM step and latched until IDLE; FLIGHT never.
- At `vzReset` on the sim: `vzSimDefaultConfig()`, `kTrue = kPrior`, `vRest` =
  the resting battery at ARM, camera model on (warm, `camWakeS` 1.5 s,
  `camConvS` 0.3 s, `c` = 0), seeded by `esp_random()`.
- The FSM's trips, battery rules and locks always use the real sensors.
- Thrust to the motors: DRY 0 at both barriers; BENCH `min(vo.thrust, bench
  cap <= 15000)`; the sim always gets `vo.thrust`.
- No learning on the sim (`fromSim`, and the glue learns in FLIGHT only).
- The real camera supplies the photos in DRY and BENCH.

---

## 10. Internal RAM

The glue's static state is internal RAM (the 1 kHz loop must not touch PSRAM):
FSM 176, inputs 236, outputs 92, controller 544 + config 252, sim 68 + 76,
mailbox 80, two status copies 128, app hook 24, profile 72, plus small
variables. The reset-lock record is in RTC slow memory. The recorder's +10 B
per record is PSRAM. The interactive hover adds the motion state (FSM 440,
outputs 208, hold references 32) and the IRAM of section 14.7.

---

## 11. Bench, WP1's part

These need the drone. The operator procedure is `docs/autonomy-bench.md`
(props on, on a mattress: phase A DRY only, phase B BENCH on the sim below
lift-off with hands clear, phase C short FLIGHT hops); this is what it checks
of the glue.

1. **Reset-lock:** a battery power-on (battery and USB out 5 s, then battery)
   gives `auto status` `resetLocked=0 rst=poweron`. `reboot`, `reboot rwdt`,
   `mguard test panic` and the R button (a power-on reset that keeps RTC
   memory) give `resetLocked=1`. A battery-only replug with USB connected is
   not a reset (USB powers the board): `resetLocked` stays as it was. While
   locked, a BENCH ARM is refused GUARDS (LOCKS) and a DRY ARM is accepted:
   DRY is exempt from both the reset-lock and the restart lock (`plock`,
   default on; off on the user's drone) in G1, since it cannot spin a motor
   (integrator decision after review WP1-R3; auto_fsm.c `locked()`).
   USB unlock (2026-10-12): after a `reboot`, `auto unlock` while a DRY sortie
   is ARMED answers `ERR busy: autonomy active`; in IDLE `OK auto unlock
   resetLocked=0 was=1 rst=software (...)`, then `auto status`
   `resetLocked=0 rst=software` and a BENCH ARM passes G1; the next `reboot`
   gives `resetLocked=1` again with `auto rtc` `prevLocked=2`
   (docs/autonomy-bench.md A3 step 6).
2. **Loop-rate go/no-go first:** `auto prof reset`, `camvz air full` (the
   observer on its in-flight path, which a DRY sortie alone never feeds), one
   DRY sortie, then `auto prof` `activeLoopsPerS` >= 940, cross-checked by two
   `camvz prof` reads 3 s apart in the hover wait. Only the loop rate gates
   (review RT-1): `activeAvgUs`, `activeMaxUs`, `fastMaxUs` and `idleMaxUs`
   are wall-clock times that include preemption, shown as information.
   `stabStackFree` >= 1024 is a check of its own (sampled in IDLE every 5 s
   and on the first IDLE step after a sortie).
   Below 940: stop at DRY; no BENCH or FLIGHT until a fallback (PLAN section
   2.6) is in, which needs the user's sign-off. The plan's gate was 975; the
   user signed off 940 on 2026-10-12, before phases B and C, on the
   measurements of that day (Mac and phone linked, the daemon running, photo
   pushes): 630-700 loops/s with the per-loop code in flash, 948-973 in
   TAKEOFF/HOVER/LAND with it in IRAM, idle 983-990; manual flights flew well
   at ~957 (2026-10-11). Lowering the gate below 940 needs the user's sign-off
   again. The measurement is a DRY sortie: its controller runs on the sim
   plant (`vzSimStep`, `expf`), not on the FLIGHT path's inputs
   (`camVzGlueVzInput`, `sensfusion6GetAccZ`, `camVzGlueSetLiftoffHint`),
   which are IRAM as well (section 2.6) but not timed here; the ARMED loops
   run the full guard check (`autoFsmGuards`) on every loop.
3. **Settings and gates:** `auto get` shows `vmin=3800 vzK=165000
   autowait=5000 autoaccel=2000 autoon=0`; an ARM before `auto on` is refused
   ENABLED; during an ARMED DRY sortie every gated command answers
   `ERR busy: autonomy active` and `auto prof` `flashRefused` stays 0. For the
   link key use exactly `autokey x`: while active it answers busy before
   parsing, and in IDLE (if the 10 s ARMED window has lapsed) it only answers
   the usage error, so it can never erase or replace the real key
   (`autokey clear` or a valid key could). The flash-interlock race (an ARM
   during `atrim learn`, an `auto set` or a vzK save is refused GUARDS/LOCKS
   and counted in `armsBlocked`) is proven by the host tests (section 12),
   not on the bench.
4. **Camera height observer paused:** in IDLE on the ground `camvz off`
   (`OK camvz off ...`); with `auto bench imu` a BENCH ARM is refused
   GUARDS (LOCKS) (so would a FLIGHT ARM be); a DRY ARM is accepted; after
   `camvz on` no LOCKS; `camvz air on` locks again; with `auto bench sim` a
   paused observer does not lock. The bench doc runs this in phase A with
   `gcap off`, so every BENCH ARM there is also refused GCAP and none is
   accepted. `camvz off` with the motors running answers `ERR busy: ...`.
5. **DRY sortie:** the recorder's motors and `thrust` 0 throughout; `rec dump`
   has the `autoState` column; `flights` has no new summary. The tilt, rate
   and attitude-error trips are checked here by hand: they read the real IMU,
   so a hand tilt in a DRY hover gives KILLED with the motors never spinning.
6. **BENCH** (sim, `gcap 15000`, props on, below lift-off, hands clear): the
   **collective** thrust stays <= 15000: `live` `thrust=` and the recorder's
   `thrust` column (the setpoint after the override, gcap and soft start).
   The per-motor values (`live` `m=`) can read above it while the clamp
   works (review WP1-R5): both barriers clamp the collective thrust, and the
   mixer adds the roll, pitch and yaw terms per motor. AUTO_STATUS `thrust`
   and the recorder's `vzThrust` are the controller's command before the mode
   clamp and also exceed 15000 on the sim. After the sortie `tkoff` shows
   `takeoffs` +1, `window=0` and `sbCount` unchanged with `camsb=1` (the
   camera stayed awake); when the window ended (1.5 s after the sim's
   lift-off) is not visible from the console and is covered by the host
   tests. BENCH on the real IMU (`auto bench imu`) needs a hand near
   spinning props and is not run.
7. **Stops and the motor watchdog:** a Mac KILL with the motors spinning
   stops them at once and latches the driver until a reset (a battery
   power-on then also clears the reset-lock). `mguard test hang|imu 200` with
   the props spinning from the phone throttle cuts 80-90 ms into the freeze:
   that watchdog cut is what the bench measures for a frozen stabilizer. A
   `k` pressed by hand cannot be timed against the hang (the watchdog always
   cuts first). A KILL during a frozen stabilizer would need a scripted
   trigger (one process writing the console command and sending the KILL
   burst within ~20 ms); it is open for stage B. By design the KILL calls
   the motor stop directly, without waiting for the stabilizer
   (`autonomyKillFromLink()`).
8. **Heap:** `auto status` `heap` >= 20 KB right after a power-on with the
   phone and the Mac joined; `heapMin` >= 14 KB read right after the BENCH
   photo sortie, before any reset (it is the minimum since boot).
9. **In flight only** (the phase C hops): the drift hold engaging in an
   autonomous HOVER, the observer's time to its first healthy sample with
   the camera awake, the supply-rail minimum of an autonomy take-off with the
   camera awake.

---

## 12. Verification done off the drone

- `test_autonomy_glue_host` (`tools/tests/test_autonomy_glue_host.c`, stubs
  in `tools/tests/autonomy_glue_host/stub/`; review WP1-R9): `autonomy.c`
  compiled with stubbed drone and IDF APIs and the real state machine,
  controller, mailbox and glue core. It emulates the stabilizer loop (barrier
  1: the override; barrier 2: `autoClampThrust` on `control.thrust`), the
  link (posts, heartbeat, photo sets), the phone, the USB console and the aux
  task; each scenario runs in its own forked process (fresh statics, one
  boot). On the integrated tree (a2b-int at 8cf3860): 193 checks, 0 failures
  (205 with the `heap` scenario, after the review fixes of 0cf6fbf; 221 with
  the 60 cm `margin-60` scenarios of 12.1; 248 with the USB unlock scenarios
  `usb-unlock` and `usb-unlock-poweron`: after a software reset FLIGHT is
  refused LOCKS only; the unlock is refused in ARMED, HOVER, the release
  hold, with an ARM in the mailbox and in manual flight (a motor running),
  each time with nothing changed; in IDLE it unlocks, a DRY ARM taken during
  it is refused LOCKS, `auto rtc` keeps `rst` software, the AUTO_STATUS flag
  clears, a FLIGHT ARM is accepted and DRY is accepted throughout).
  WP1 alone had 182, run both against the repo's (A2a) state machine and
  controller and against WP4's (`a2b-wp4-vert` at 619588a and at 3cdb035,
  its `auto_fsm.h` and `vert_ctrl.h` overlaid on a copy of `utils/interface`:
  a header includes its neighbours from its own directory first, so putting
  WP4's directory first on the include path alone mixes the two versions).
  Scenarios:
  - `idle`: exactly 40 full steps in 4000 IDLE loops, one extra step at once
    on a command, a KILL and a session, back to 10 Hz; the manual setpoint
    bitwise untouched on every IDLE loop; barrier 2 65535;
  - `dry`: three DRY sorties (photos, PHOTO while waiting, `HOVER_TIMEOUT`,
    TOUCHDOWN; a Mac KILL in HOVER; the phone stick in HOVER, LAND
    `APP_INPUT`): **both barriers 0 in ARMED, TAKEOFF, HOVER, LAND, KILLED
    and the release hold**, also for a controller that outputs 40000; the
    override exact (roll/pitch +0.0f, x/y/z disabled, roll/pitch/yaw
    `modeAbs`, yaw at the held heading) on every active loop; the real camera
    height never read; nothing learned; the RTC ring; and, at every spinlock
    release in the three sorties, the snapshot and the seq belong together:
    the snapshot's transition count moves with the seq and the lock-free seq
    equals the locked one (a seq++ in a critical section of its own, before or
    after the snapshot copy, fails it);
  - `bench`: BENCH-sim, <= 15000 at both barriers (also for a controller at
    40000), the sim's lift-off noted;
  - `plant`: DRY, FLIGHT, BENCH-sim, BENCH-imu, DRY in a row: each sortie's
    plant comes from its own ARM command (the sim flag, and whether the real
    camera height is read), FLIGHT is never refused G12 after a sim sortie;
  - `locks`: an ARM while a writer holds the flash interlock is refused
    GUARDS/LOCKS and counted in `armsBlocked`; `camvz air on`, `camvz air
    spin` and `camvz off` (paused) refuse FLIGHT and BENCH-imu LOCKS and leave
    DRY and BENCH-sim alone; without them FLIGHT and BENCH-imu are accepted;
  - `race`: a console change (gcap) that ends just before an ARM's take: the
    ARM reads the guard inputs after the take, so it is refused, never
    accepted on the values from before the change;
  - `handback` (review E2E-1): a phone setpoint with its throttle up still
    queued in the commander when autonomy hands back and the phone falls
    silent never reaches the motors (the queued setpoint is dropped);
  - `settings`: an ARM taken while the store writes, and one taken after the
    store's own bracket but before the RAM copy, are refused LOCKS (for a set
    and an erase); an ARM waiting in the mailbox makes the write answer busy
    with RAM and NVS unchanged, and then proceeds; an ARM taken the moment the
    claim is released arms with the new value; the sortie's kPrior is the
    `vzK` of its ARM;
  - `learn-console`, `learn-poll`: the console's `auto set vzK` run between
    the aux task's NVS write and its RAM copy answers busy, and the other way
    round the aux task's save is refused and cancelled; RAM and NVS end equal,
    with no later write;
  - `flightsim`: FLIGHT on a simulated plant through the real-input path
    (camera awake, plant gain 1.03 x the stored `vzK`): lift-off noted, HOVER
    with the camera height in use, the hover wait, landing, the learned `vzK`
    stored once by `autonomyPoll()` and kept in NVS; <= `VZ_THRUST_ABS_MAX`;
  - `resetlock`: after a software reset BENCH is refused LOCKS and DRY is
    accepted (the stubbed restart lock is off);
  - `heap` (review F4 of 0cf6fbf): G9 on the aux task's heap sample: no
    sample since boot refuses an ARM (and the IDLE status shows HEAP); a
    fresh sample below `AUTO_HEAP_MIN` refuses; a sample up to ~160 ms old
    still counts; once the aux task has stalled for longer, the last sample
    (plenty of heap before the drop) refuses, and so does a stall with the
    heap fine, until it samples again; a TAKEOFF on a stale sample is refused
    HEAP and the drone stays ARMED. On the code before the fix (the value
    seeded at init, no time) 6 of its 12 checks fail. `learn-*` stop the aux
    task only after the landing, `margin-*` keep it running: their ARMs need
    its heap sample;
  - `margin-*`: info only (12.1).
  Mutation check (scratch, not in the repo): each of these edits to
  `autonomy.c` makes it fail: no pause or no bench-mode test at ARM, the
  flash-busy flag ignored, the settings claim removed (the pre-fix code), the
  RAM copy moved after the claim's release, the aux task's save without the
  claim, the seq++ in its own critical section, barrier 2 not 0 in DRY, the
  override unclamped, the plant from the latch instead of the ARM, the bench
  source ignored, IDLE every 50th loop, a command not stepped at once, the
  sortie mode from the wrong field, `vzK` not latched from RAM at ARM, the
  yaw not held.
- `test_cam_vz_glue_host` (`tools/tests/test_cam_vz_glue_host.c`, includes
  `cam_vz_glue.c`): 16 checks, 0 failures (a stale snapshot is reported
  unusable since 8cf3860, review E2E-2). While paused no block reaches the
  observer and a snapshot frozen healthy in CVS_AIR is reported unusable
  (`camVzGlueVzInput`), `camVzGluePaused()` is true; `camvz on` hands blocks
  over and makes it usable again; a bench mode is unusable. Removing the
  pause test from `camVzGlueVzInput` or from `camVzGlueStep` makes it fail.
- `test_auto_glue_core`: 145 checks, 0 failures (A2a and WP4 headers).
- `test_flight_summary`: 37 checks, 0 failures.
- The other suites on the integrated tree (8cf3860): `test_auto_mailbox`
  43/0, `test_auto_proto` 194/0, `test_auto_link_host` 224/0,
  `test_auto_fsm` 920/0, `test_auto_sortie` 61/0, `test_vert_ctrl` 216/0,
  `test_cam_vz_shadow` 326/0.

### 12.1 The pre-wait air-clock margin (review WP1-R10)

The FSM lands `AIR_TIME` if the air clock reaches `autoair` (default 6000 ms
since 2026-10-12 evening, 4500 ms before) before the hover wait starts, i.e.
before the photos requested in TAKEOFF are done. With the Mac's first default (40 cm, 2 photos 300 ms apart) the margin
was thin. Integrator decision D3: the Mac tool's default sortie takes **1
photo**; more photos are requested with PHOTO during the hover wait, which the
pre-wait limit does not count (only `AUTO_AIR_HARD_MS`, 10 s, still applies;
18 s since 2026-10-12 evening).

`test_autonomy_glue_host`'s `margin-*` runs (FLIGHT on the plant, 40 cm,
photos 300 ms apart, a set done (n - 1) x gap + one frame after it starts,
the air clock as AUTO_STATUS reports it at 10 Hz), the same with the A2a and
WP4 state machine and controller:

| Request | Plant gain | Hover wait starts at air | Margin to 4500 ms |
|---|---|---|---|
| 1 photo (D3 default) | = stored `vzK` | 3896 ms | ~600 ms |
| 1 photo (D3 default) | 1.03 x stored | 4096 ms | ~400 ms |
| 2 photos | = stored `vzK` | 4196 ms | ~300 ms |
| 2 photos | 1.03 x stored | 4396 ms | ~100 ms |

With WP4's controller a plant 6 % heavier than its stored gain started the
wait at 4196 ms with one photo. Every run reached the hover wait and landed `HOVER_TIMEOUT`. A
second photo at take-off costs ~300 ms of the margin; `auto set autoair 6000`
(inside the validated range) would add 1.5 s if the user wants more photos in
the TAKEOFF request. Not modelled: a slower real camera start in the air.

**2026-10-12 (evening), 0.60 m and a 10 s wait** (user decision;
docs/autonomy-a2.md revision 5). The margin runs above now print the new
default (6000 ms: margins 1.6-2.1 s at 40 cm). Two more, `margin-60` and
`margin-60-heavy` (TAKEOFF 60 cm, 1 photo, `auto set autowait 10000` through
`autonomySettingSet`, which refuses 12001), are checks:

| Request | Plant gain | Hover wait starts at air | Margin to 6000 ms | The 10 s wait's LAND at air | True peak |
|---|---|---|---|---|---|
| 60 cm, 1 photo, wait 10 s | = stored `vzK` | 4396 ms | ~1600 ms | 14335 ms (`HOVER_TIMEOUT`) | 0.65 m |
| 60 cm, 1 photo, wait 10 s | 1.03 x stored | 4596 ms | ~1400 ms | 14498 ms (`HOVER_TIMEOUT`) | 0.59 m |

Required: the wait >= 1 s inside `autoair`, `hoverWaitMs` 10000 in
AUTO_STATUS, `HOVER_TIMEOUT` 10 s into the wait (+/- 150 ms: both air clocks
are 10 Hz AUTO_STATUS samples) and >= 2.5 s inside the 18 s clock, a true
peak of 0.45-0.80 m, back in IDLE. The glue's `vzStart()` sets the target with
`vzConfigSetHeight()` (a 2.5 s climb timeout, a 2.0 s LAND flat part and the
estimate guard `hMax` 1.2 m at 0.60 m; unchanged up to 0.40 m).

---

## 13. Frozen-interface gaps and their workarounds (review WP1-R8)

The step-0 headers stay frozen (integrator decision); these gaps are worked
around in WP1's own files:

- `autoStatusExtra_t` (auto_glue_core.h) has no `hoverWaitMs`, so
  `autoStatusFill()` cannot fill AUTO_STATUS offset 36 and leaves it 0; the
  glue writes `staging.hoverWaitMs` right after the fill (the latched
  `autowait`, the setting in IDLE). `test_auto_glue_core` checks that the fill
  leaves it 0.
- autonomy.h has no getters for the phone hook's ages: `auto status` prints
  `appAgeMs` (from the RPYT link statistics, the same tick stamp) instead of
  plan 7.2's `inputAgeMs=` and `gestureAgeMs=`; the consent gesture shows only
  through the GESTURE guard bit in `guards=`.
- autonomy.h has no per-setting source: `auto get` reads NVS itself, in IDLE
  only, and prints `(nvs)`, `(default)`, `(bad)` or, while active, `(ram)`
  (plan 7.2's `src=`).
- `autonomyRtcView_t` has no `boots`: `auto rtc` prints `prevLocked=` and
  `prevTransitions=` instead of `boots=`.
- `autonomySettingSet/Reset` report only `nvs` for an NVS error: the reply is
  `ERR auto set: NVS write failed (<key> unchanged)` without the esp_err name
  (the name is in the debug log).
- autonomy.h names imu_level_cal_store.c as the caller of
  `autonomyFlashBegin/End`; usb_diag.c now also calls them to hold the
  interlock across a console change (7.3).
- `autoFsm_t.mode` is private: the glue keeps its own copy of the sortie's
  mode from the ARM step (review WP1-R6).
- Additive getters after step 0, as `autonomyHoldBlocked()` before:
  `autonomyTakeoffIntegratorHold()` (the take-off integrator hold, 2.4; user
  decision 2026-10-13, evening) for controller_pid.c, which autonomy.h's
  callers list now names.

PLAN section 7.2 and WP3's docs/autonomy-bench.md should use these forms.

---

## 14. Interactive hover (user decision 2026-10-12, late)

(2026-10-15, section 22: the drone no longer lands at the wait's end or for the guards below that only meant "this is
unusual"; it holds and reports, and Claude decides.)

The user, the same night: "No multi-phase BS. It's time to try the real
thing." The drone is left on any flat spot (nobody points it anywhere). It
takes off, hovers at about 1 m and takes one photo. Then it waits, and the
user's instructions, given to Claude in the chat, are flown one at a time:
move left, right, forward or back by X m, up or down, turn, or move close
to an object Claude sees in the photo, up to X m from it. This supersedes
the multi-phase stage C plan (`docs/autonomy-c-plan.md`) for now.

### 14.1 Envelope

| What | Value | Where |
|---|---|---|
| take-off height | 0.20..1.20 m (TAKEOFF `heightCm` 20..120; 60 before) | `AUTO_HEIGHT_CM_MAX`, `VZ_H_TARGET_MAX` |
| climb above 0.60 m | the pre-wait air limit and the TAKEOFF backstop grow 25 ms per cm above 60 (1.5 s at 1.20 m); `hMax` keeps its rule | `AUTO_AIR_PER_CM_ABOVE_60_MS`, `vzConfigSetHeight` |
| hover wait | up to 120 s since 2026-10-14 (section 18.2; 60 s before): NVS `autowait` 0..120000, or TAKEOFF `actWaitMs` 1000..120000 for one sortie (`--wait-s`) | `AUTO_HOVER_WAIT_MAX_MS` |
| wait restarts | every accepted MOVE, YAW (and every STOP); it does not count down while a motion runs | auto_fsm.c HOVER |
| air limit | 120 s from the first RAMP step, wait and motions included (since 2026-10-14, section 18.1; 80 s before, 18 s before the interactive hover) | `AUTO_AIR_HARD_MS` |
| controller HOLD limit | 119 s (`holdMaxS`, at most 120; 79 / 80 before) | `VZ_HOLD_MAX_LIMIT_S` |
| battery | since 2026-10-14 (section 16): in HOVER the 10 Hz battery < 3.00 V, or T_hold > 57000, for 0.5 s: LAND `BATT_LOW` (39); MOVE / YAW refused once that has held 0.25 s (AUTO_STATUS `battLow` bit 0; bit 1 is the condition now) or while T_hold > 53400 (reply `LOW_BATT` only). Until then: < 3.30 V for 1 s, refused after 0.5 s | `AUTO_VBAT_LOW_LAND_V`, `_MS`, `_REFUSE_MS`, `AUTO_HEADROOM_LAND_T`, `AUTO_MV_REFUSE_T` |
| height offset | the moves' commanded offset from the take-off height: +/-1.00 m in both the motion module and the controller (0.20 m + 1.00 m = 1.20 m) | `AUTO_MV_HOFF_MAX` = `VZ_HOFF_MAX` |
| horizontal move speed | up to 0.30 m/s (0.20 until 2026-10-14, section 17.5); the camera's view rates cap it to 0.12 x D sideways and 0.15 x the end distance forward | `AUTO_MV_V_H_MAX`, `AUTO_MV_W_LAT_MAX`, `AUTO_MV_W_FWD_MAX` |

The 3.30 V threshold comes from the logs: the worst 1 s window of the
loaded 10 Hz battery was 3.48 V (60 s mission), 3.58 V (5 s missions) and
3.34 V (calibration and evening flights from 3.91-3.93 V at rest), and a
depleted pack read 3.27-3.34 V. Every other rule (heartbeat, phone, trips,
camera, the vertical controller's guards) is unchanged. (The user lowered it
to 3.10 V and then 2.65 V on 2026-10-13; since 2026-10-14 the end of the
battery is section 16.)

**LAND from up to 1.20 m.** Above the knee `VZ_H_KNEE` (0.60 m) LAND is
slow even without the camera estimate (`HOLD_NO_CAM`): falling at once from
up there reached ~1 m/s in the sim. Its flat part and timeout grow with the
reference height and a vertical move up (`landFlatEff`, `landMaxS` = 6 s +
(h - 0.60 m) / 0.3 m/s, and at least the flat part + 4 s), and after 18 s of
HOLD by the drift the height estimate may hide (min(1.0 m, 0.017 m/s x the
HOLD time) at the near-ground speed; 2.0 m since the 120 s air clock,
section 18.1). At or below 0.60 m and a HOLD under
18 s without a move up both are exactly as before (test_vert_ctrl). The
state machine's LAND backstop is 6.5 s plus an upper bound of the same
extra (9.8 s after 0.20 + 1.00 m). Sim (camera, c up to +/-0.03 m/s):
touchdown at most 0.41 m/s from 1.20 m, 0.37 m/s without the camera,
0.47 m/s after a 60 s hover; 0.20 m moved up to 1.20 m: <= 0.31 m/s
(test_vert_ctrl), <= 0.40 m/s (test_interactive_sim).

### 14.2 Commands

| Command | Limits | Profile |
|---|---|---|
| MOVE right, forward, up (cm, the drone's frame when accepted) | \|right\|, \|forward\| <= 1.50 m, \|up\| <= 0.50 m; sideways also <= 0.5 x D (`TOO_WIDE`); height after it 0.25..1.20 m (commanded and estimated; a move down: >= 0.25 m + 0.01 m/s x the air time); forward refused if it could leave < 0.50 m to the scene ahead at the expansion scale k = 0.6 (`AUTO_MOVE_MIN_FRONT_CM`, `AUTO_MV_K_LOW`; the Mac keeps 0.60 m) | horizontal: one straight line, <= 0.30 m/s (0.20 until 2026-10-14, section 17.5), 0.10 m/s^2, lateral image rate <= 0.12 /s, expansion <= 0.15 /s (the observer's frame gates); vertical <= 0.15 m/s, 0.20 m/s^2 |
| YAW deg (+ clockwise, right) | 1..180 either way | 30 deg/s, 60 deg/s^2 (15 and 30 before the user decision of 2026-10-13, evening; section 14.10) |
| STOP | never refused | a running MOVE brakes (0.15 m/s^2 lean) and holds where it is; a turn ends where it is; a re-track or re-view bob runs on (<= ~8 s) |

One motion at a time; HMAC, ACKed, in FLIGHT, DRY and BENCH (DRY and BENCH
do not fly: the references run as if tracked perfectly). The refusal
reasons and the ACK detail are in `docs/autonomy-link.md`.

### 14.3 How a motion is flown

The drone has no position sensor. A horizontal MOVE is a moving reference
fed into the camera drift hold, in the hold's own units: lateral x / D,
forward ln(D0 / D), where D is the scene distance the camera height
observer measures (G = f / D, f = 99.79 px). D is taken when the command is
accepted; without a valid D a horizontal MOVE is refused (`NO_DIST`). The
hold flies the drone to the reference with its own, unchanged gains, helped
by a feedforward lean (acceleration plus drag); the reference the ~120 ms
old frames are compared with is delayed by 120 ms. After the profile the
motion settles (<= 3 s, or within 0.04 m + 10 %); a settle that times out
short of the target along the move by more than that, or off its line by
more than 0.15 m, with at least half of it measured, ends `SHORT` (section
14.10): the measured end becomes the move's end and the drone holds there. A vertical move is a
speed reference for the controller's HOLD (with a position correction on
its height estimate); a turn is a ramp of the yaw-lock setpoint.

Every MOVE and YAW starts a new epoch of the hold's reference; a STOP, a
progress guard, the end of a turn or leaving HOVER mid-motion starts a snap
epoch: the hold's integral goes back to its value at the epoch start (the
lean it had learned), so the drone holds where it is. A MOVE that finished
keeps its end as the hold's reference, in HOVER and after it: a LAND then
lands where the move ended (591f176 dropped that reference outside HOVER,
and the hold, engaged in LAND until contact, flew the drone 0.2-0.6 m back
toward the move's start while it descended). A turn changes the scene: D is
stale until the tracker has a G of the new view; after a 0.5 s dwell, if it
has none, the drone bobs +/-4 cm (up to 4 cycles) to excite the tracker; the
camera height estimate may use the old G meanwhile (`healthyGuess`). Without
a G after the bob the turn ends `RETRACK` and, without a healthy height
estimate, the drone lands `HOVER_NO_CAM`. A STOP never ends the dwell or the
bob (it would end the guess and land `HOVER_NO_CAM` at once).

**Which D.** The hold integrates v / D_now, so every conversion rests on D:

- a forward or back move measures where the scene now is from its own
  expansion (`dTrack`, k = 1) and a lower bound (`dTrackLow`, k >= 0.6);
  the next forward move converts with `dTrack` (more accurate than the
  observer's G, which barely moves in a still hover); the status reports it;
- a sideways move converts with the larger of the observer's D and `dTrack`
  (a D too large makes a move short, never long);
- the forward floor uses the lower bound and the expansion scale k at 0.6:
  the hold drives the measured k ln(D0 / D) to the target, so the true end
  is D0 ((D0 - F) / D0)^(1/k) (k < 1 when the floor is in view; stage C
  found 0.27-2.5 in rendered arenas, 0.73 for a wall plus a plain floor at
  3 m). At D 2.0 m the drone takes at most 1.12 m forward (k 0.6 ends at
  0.50 m, k 1 at 0.88 m);
- a depth step under a sideways move (sliding off a near object onto a far
  background) makes it longer by up to D_far / D_near, since D is sampled at
  acceptance: sideways moves are at most 0.5 x D, and once the sideways shift
  since the last fresh view reaches 0.35 D (the observer's own view change,
  as a 20 deg turn; small turns count by their angle) the move ends with a
  re-view bob of 2 whole cycles (~3.6 s, D stale meanwhile): without a turn
  the tracker is not restarted and keeps the old view's G through a still
  hover; the bob makes it follow the new depth (test_cam_vz_obs
  testViewShiftBob: 118 -> 68 px/m against a true 66 in 3.6 s).

**Zero is bit-identical.** Without a motion every output is 0 and the epoch
never changes: the hold and the vertical controller compute exactly what
8f51285 computed. test_camera_hold and test_vert_ctrl hash their outputs
over long input sequences (FNV-1a 64; camera_hold 0x42DA004C5D79DB66,
vert_ctrl 0x169CEE4B871A74D9), the same at -O0 and -O2 on 8f51285's sources
and on these.

### 14.4 Guards while a motion runs (FLIGHT)

- the hold not applied, coasting, or a frame older than 300 ms: the
  reference pauses; 2 s of it: LAND `MOVE_NO_CAM` (40);
- tilt over 8 deg for 100 ms: LAND `MOVE_TILT` (41);
- while the profile runs (RUN), the measured progress off the reference by
  more than 0.15 m + 30 % of the move's part on that axis (vertical 0.15 m +
  50 %): STOP (`PROGRESS`), the drone holds (591f176 had 0.30 m + 50 %,
  which a move of 0.60 m or less could never trip);
- once the profile has ended (SETTLE; user decision 2026-10-13, evening,
  section 14.10): the same margins around the target, except that the
  lateral one is at least 0.25 m (the forward one, toward the scene, is
  unchanged): past the target or back from it by the margin stops it; an
  under-shoot within the margin is no fault (the settle ends it `SHORT` or
  DONE);
- the settle timed out with less than half of the move measured along it
  (a frozen view: frames that pass the checks but show no motion): STOP
  (`PROGRESS`) instead of DONE, so the unreachable target does not stay in
  the hold (sim: a 0.5 m move with a frozen view ends 0.50 m out instead of
  1.27-1.51 m);
- every existing rule keeps running (trips, heartbeat, phone, battery, the
  air limit, the controller's guards);
- 2026-10-13 (docs/autonomy-a2.md section 12): an in-air contact lands
  (`COLLISION`, 42; user decision 2026-10-13, a KILL in 98cd4ae..eb9264d),
  during a motion as anywhere in TAKEOFF after the lift-off and in HOVER: the
  motion stops on that step, the camera hold is blocked for the rest of the
  sortie and the descent is blind to the height estimate the contact
  corrupted, its thrust from the hover before any vertical move or bob
  (autonomy-a2.md 12.8); and from 2 s into HOVER the hold's scene gate
  treats frames of something moving in the view (a person) as a lost view,
  so the reference pauses (and a new MOVE is refused `NO_VIEW`) while it
  lasts.

### 14.5 The Mac

`tools/espfly_auto.py` (one JSON line per command; the daemon must run):

```
python3 tools/espfly_auto.py daemon --allow-flight            # its own terminal: k KILL, l LAND
python3 tools/espfly_auto.py sortie --mode flight --confirm-flight --height 100 --photos 1 --then wait --wait-s 60
python3 tools/espfly_auto.py move --right 0.5                  # --left/--forward/--back <= 1.5, --up/--down <= 0.5
python3 tools/espfly_auto.py yaw --deg 90                      # + right
python3 tools/espfly_auto.py approach --bearing 12 --to 1.0 --est 2.5 [--person]   # (removed 2026-10-15, 22.9)
python3 tools/espfly_auto.py photo --n 1 --gap 150
python3 tools/espfly_auto.py stop
python3 tools/espfly_auto.py state                             # D_m, yawOff_deg, hTarget_m, airLeft_s, mv
python3 tools/espfly_auto.py land
```

`move` and `yaw` return when the motion has ended (`end` DONE, SHORT,
STOP_CMD, PROGRESS, ...), with the measured travel (`travelled_m`: the
drone's end status, which keeps a MOVE's measured progress once it has
ended; from an older drone that restarted it at a STOP, the last progress
seen while it ran); `ok` is true for DONE and SHORT (SHORT adds a `note`:
it holds where it ended); `--no-wait` returns at the ACK.
The Mac refuses (nothing is sent) outside the hover wait, while a motion
runs, with the battery low (`battLow` bit 0: the drone's BATT_LOW condition,
below 3.00 V or T_hold over 57000, held 0.25 s, its own refusal; the
condition now is bit 1, `battLowNow`, and refuses nothing; until 2026-10-14
0.5 s below 3.30 V), without a valid D for a horizontal move, sideways more than
0.5 x D, a forward move that could leave < 0.60 m at k 0.6
(D ((D - F) / D)^(1/0.6)), a height outside 0.25..1.20 m, or a commanded
height below 0.25 m + 0.01 m/s x the air time (the estimate drifts, reading
high: late in a sortie only a move up). `sortie --wait-s` above 12 s needs
`--height` >= 0.25 + 0.01 x (wait + 8 s) m (0.93 m for 60 s). Plain `move
--forward` has no person rule: use `approach --person` toward people (since 21.5: `move --person-gap G --est E`;
`approach` was removed on 2026-10-15, 22.9).

`approach --bearing B --to M --est E [--person]` (removed 2026-10-15, 22.9) turned to a bearing read off
the latest photo's ticks, waits for a valid D (<= 8 s), then flies forward
one step of at most 1.0 m (then a photo and `approach` again). `--est`, the
object's distance Claude reads from the photo, is required: D is the
distance of the textured scene the tracker follows, mostly the background,
never the object's (591f176 planned on D alone, so a person 2.5 m away in
front of a 4 m wall got a 1.0 m step and would have ended ~1.5 m from them).
The plan allows the object to be 25 % nearer than min(D, est) and the step
to fly as far as k = 0.6 allows: F is the largest with
0.75 min(D, est) - (D - D ((D - F) / D)^(1/0.6)) >= to and the scene end
>= 0.60 m. Examples: D 3.0, est 3.5, to 1.0: 0.82 m; D 4.0, est 3.6, to 2.0
(`--person`): 0.43 m; D 2.0, est 2.5, to 1.0: 0.31 m; est 2.5 with `--person`: no step ("already within
reach"). It converges in steps; expect to stop well short of `--to` (the
margins are for the worst case). `--to` >= 0.60 m; with `--person` >= 1.0 m (user decision 2026-10-13; was 2.0 m).
It is refused unless the latest photo was taken from where the drone is
now: no MOVE or YAW since (and the same heading within 5 deg), taken while
no motion ran (591f176 turned by a bearing read 90 deg away after a `yaw`).

The photo JSON carries `yawOff_deg` (the heading from the take-off heading;
a bearing in the photo adds to it), `hTarget_m`, `D_m`, the motion state
and the air time left.

### 14.6 Verification (host)

- test_auto_motion (137 checks): profiles, acceptance and every refusal,
  epochs and snaps, the guards, the brake, the bob; the idle outputs keep a
  finished MOVE's end; which D converts what; the forward floor at k 0.6 for
  every accepted move over D 0.6..5 m and k 0.6..2.5; TOO_WIDE; the re-view
  bob; the drift floor; the frozen view; STOP never ends a bob; the +1.00 m
  offset.
- test_interactive_sim (35 checks): the real state machine, motion module,
  vertical controller and camera hold against a horizontal point mass with
  camera latency and noise and a room; the hold stays engaged in LAND until
  contact, as camera_flow.c does. The mission TAKEOFF 100, photo, MOVE
  right 0.5, back 0.5, YAW 90, up 0.2, forward 0.4, STOP, LAND over 30 seeds
  (D error +/-10 %, trim +/-0.3 deg): relative error p50 7 %, p90 15 %, p95
  18 %, max 23 % (118 of 120 within 20 %); D error +/-25 %: worst 25 %; no
  trip; touchdown <= 0.37 m/s. Guards: camera lost mid-move: LAND
  `MOVE_NO_CAM`; battery: LAND `BATT_LOW` after 1.1 s (since 2026-10-14:
  at 28.7 s, 0.48 m/s; section 16); repeated STOPs: LAND
  `AIR_TIME` at 80 s (120 s since section 18.1); a STOP mid-move drifts 0.01 m more. After 591f176's
  verification: LAND 2 s after a finished move: <= 0.04 m sideways, forward
  within the LAND's own creep (0.06-0.16 m without a move), <= 0.08 m/s at
  contact (591f176: 0.24-0.40 m back toward the move's start); k 1.0 / 0.73
  / 0.6 with the largest forward move the drone takes: never closer than
  0.50 m (k 0.5: 0.38 m, outside the bound); a box face 1.5 m ahead of a
  3 m wall: right 1.00 m refused `TOO_WIDE`, right 0.75 m flies 1.20 m
  (<= 0.5 x D_far); a frozen view: `PROGRESS`, 0.39-0.98 m out at the LAND
  command; 0.20 m + 0.50 + 0.50 m then LAND: <= 0.40 m/s; moves down late in
  a 60 s hover with the estimate drifting +0.01 m/s: never below 0.38 m true
  (591f176: 0.04 m); a STOP in the re-track bob: still hovering, the turn
  DONE.
- test_cam_vz_obs testViewShiftBob: the re-view bob makes the tracker follow
  a depth step without a restart.
- test_auto_fsm (1245), test_vert_ctrl (267), test_camera_hold (44),
  test_auto_sortie (79: 60 s waits at up to 0.60 m), test_autonomy_glue_host
  (306: DRY and FLIGHT interactive scenarios, battery, the 1.20 m margin),
  test_auto_proto (golden_ih), test_espfly_auto (176: the Mac commands
  against a fake drone, the battery bits, `--est` required, the photo view,
  the late low refusals).

### 14.7 IRAM and internal RAM

The motion step runs on every HOVER loop, so it is IRAM (`AUTO_HOT`) with
everything it calls per loop; the IRAM call graph from `autonomyArbitrate`
has no new per-loop flash target. What runs once per command or event stays
in flash: `autoMotionMove`, `autoMotionYaw`, `autoMvProfPlan` (a command),
`autoMotionStop` (a STOP, a guard, leaving HOVER), `settleEnd` with
`trackD` and `commitUp` (the end of a MOVE's settle), `autoMotionInit`
(ARM), `motionCmd` and `landExtraMs` (auto_fsm.c: a command, a LAND's
entry) and `landTimes` (vert_ctrl.c, a LAND from above the knee, after a
move up or a long HOLD); auto_motion.c is built with `-Os`. About +6 KB of
IRAM, paid with internal RAM moved to PSRAM, as 0cf6fbf did: the USB
console's output line (1 KB) and its crash-log copy (1 KB, now allocated
per command), the log subsystem's ops table (2 KB) and the log task's stack
(3 KB; it never writes flash). The fixes of 14.9 add the re-view bob
(`bobStep`, IRAM) and move the per-move settle end and the LAND backstop's
sum to flash: IRAM 50 B below 591f176 (text end 0x40394000), `.bss` 32 B
above. `_heap_start` 0x3fcc0fb8 is 632 B lower than 8f51285's (0x3fcc1230)
and 624 B lower than 4d35897's code (0x3fcc1228): expect the free internal
heap ~0.6 KB above 4d35897's readings (19279 B right after a reset,
22.7-22.9 KB settled on e66e223), i.e. ~19.9 KB right after a reset and
~23.3 KB settled. The changes of 2026-10-13 evening: section 14.10.

### 14.8 Limits and risks

- No position sensor: a move's size is only as good as D (+/-20-25 %), and
  the hover wanders +/-10-25 cm meanwhile. Expect +/-20-30 % over a scene of
  even depth. A sideways move past the edge of a near object onto a farther
  background goes up to D_far / D_near further (the 0.5 x D cap bounds it
  near 0.5 x D_far: 1.20 m for 0.75 m in the sim's box case); prefer short
  sideways moves when the photo shows a near object in front of a far
  background.
- Forward moves end shorter than D - F when the floor is in view (k < 1):
  the floor holds down to k 0.6; below it (stage C saw 0.27 in rendered
  arenas) a move can end nearer than 0.50 m. The Mac's 0.60 m and the
  approach margins are the reserve.
- A frozen view (frames that pass the checks but show no motion) is caught
  only by the progress guard and the settle check: the drone may end up to
  ~0.4 m beyond the move before it holds.
- While a move's profile runs the sideways margin of a straight forward or
  back move (and the forward one of a sideways move) is still 0.15 m: the
  hover's own wander can stop it `PROGRESS` (the tiger2 forward move of
  2026-10-13 reached 0.13 m sideways in its RUN; section 14.10).
- The height estimate drifts over a long hover (0.2-0.6 m in 60 s at the
  c measured lately, reading high); the LAND allowance covers up to 1.0 m.
  Late in a sortie a low hover can reach the floor without any motion: take
  off at about 1 m (the Mac refuses a long `--wait-s` from low take-offs, a
  move down below 0.25 m + 0.01 m/s x the air time, and any motion that keeps
  the commanded height below that).
- Every LAND creeps forward ~0.05-0.18 m: the descent is read partly as
  image expansion (stage C's "LAND forward creep").
- About a minute in the air in all: every command must come within the 60 s
  wait, and the whole flight ends at 80 s (LAND `AIR_TIME`). Since
  2026-10-14 (section 18): 120 s, and waits of up to 120 s.
- A turn can leave the tracker without a G (a blank wall): `RETRACK`, then
  `HOVER_NO_CAM` LAND.
- AUTO_STATUS `airMs` saturates at 65.5 s: use `airLeftDs`.

### 14.9 Fixes after the verification of 591f176 (2026-10-13)

| Finding | Fix |
|---|---|
| A LAND after a finished MOVE flew the drone back toward the move's start (the idle outputs dropped the end reference inside the same epoch) | `autoMotionIdleOut` keeps a DONE MOVE's end (test_auto_motion, test_auto_fsm, test_interactive_sim with the hold engaged until contact) |
| `approach --person` without `--est` planned on the background's D | `--est` required for every `approach`; the plan allows the object 25 % nearer than min(D, est) and k 0.6 |
| k < 1 made forward moves overshoot past the 0.50 m floor | the floor uses k 0.6 and the lower bound of D in the drone, the Mac and `approach` |
| A depth step under a sideways move overshot by D_far / D_near | sideways <= 0.5 x D (`TOO_WIDE`), the larger D for sideways conversions, a re-view bob after 0.35 D of shift |
| `VZ_HOFF_MAX` 0.60 m against the motion module's 1.00 m | `VZ_HOFF_MAX` = `AUTO_MV_HOFF_MAX` = 1.00 m; the LAND timeout >= the flat part + 4 s; the FSM backstop follows |
| A move down's floor ignored the estimate's drift | >= 0.25 m + 0.01 m/s x the air time; the Mac also refuses motions that keep the drone below it and long waits from low take-offs |
| The progress guard could not trip on moves <= 0.6 m; a frozen view kept its target | 0.15 m + 30 %; a settle timeout with < 50 % of the move measured stops it (`PROGRESS`) |
| A STOP during a turn's re-track bob landed `HOVER_NO_CAM` | a STOP never ends a re-track or re-view bob |
| The Mac refused on a single low battery reading | AUTO_STATUS `battLow` bit 0 = held 0.5 s (the drone's refusal), bit 1 = one reading; the Mac refuses on bit 0 |
| `approach` turned by a bearing from a photo taken elsewhere | refused unless the latest photo was taken with no MOVE / YAW since and the same heading |
| No per-command durations for planning | docs/autonomy-bench.md phase D: a time table and 2-3 instructions per battery |

### 14.10 User decision 2026-10-13, evening: the settle, faster turns

After the tiger2 flight of 2026-10-13 the user decided three changes and no
other tuning: the take-off integrator hold (section 2.4), the end-of-move
progress check (below) and faster turns (below).

**Why the tiger2 forward MOVE ended `PROGRESS`.** MOVE forward 1.00 m at D
2.51 m (reference ln-zoom 0.507): RUN 7.0 s, SETTLE 1.05 s, then STOPPED
`PROGRESS` with ~0.86 m measured forward. Rebuilt from the recorder's flow
columns as camera_hold.c integrates them (every engaged frame from the
first after the acceptance; `tools/tests/test_motion_replay.c`), the stop
came on the frame that took the measured **sideways** progress to 0.0602
(0.151 m) against a margin of 0.15 m / D = 0.0597: a move without a
sideways part had 0.15 m + 30 % x 0 sideways, in RUN and in SETTLE alike.
The forward progress (0.418 against the target 0.507) was well inside its
margin (0.25). The sideways drift was the hover's own wander (0.13 m to one
side in the RUN, 0.15 m to the other in the settle, the hold's position
gain being weak: 2 deg per unit of x / D); the settle could not end within
its tolerance either (0.04 m sideways). Replayed through d3b3566's
`autoMotionStep`, the logged inputs stop it `PROGRESS` on the logged record
(14.98 s recorder clock).

**The settle now** (`margLatS`, `settleEnd`): once the profile has ended,
the guard compares the measured progress with the target (the reference
has reached it) using the RUN margins (0.15 m + 30 % of the move's part on
that axis), except that the lateral margin is at least 0.25 m
(`AUTO_MV_SETTLE_LAT_M`; a forward or back move: 0.15 -> 0.25 m; a lateral
part of 0.33 m or more keeps its own). The forward margin, the axis toward
the scene, is unchanged, both ways: a forward drift, or a falling reading
the hold would answer by pushing toward the scene, stops a sideways move's
settle at 0.15 m as before. So an under-shoot within the margin is no
fault, while an overshoot or a fall back from the target by the margin
stops it `PROGRESS` at once (the RUN guard keeps the progress within that
margin of the reference, so a settle starts inside it: only a regress in
the settle can leave it, e.g. a receding person read as the drone moving
back). The settle ends as before within its tolerance (DONE, the target
kept) or with less than half of the move measured (`PROGRESS`, a frozen
view); otherwise, timed out short of the target along the move by more
than the tolerance or off the move's line by more than 0.15 m (the RUN
margin: between it and 0.25 m the settle no longer stops), it ends `SHORT`
(`autoMvEnd_t` 6, state DONE): the measured end becomes the move's end, so
the hold keeps the drone where it is (its integral as a snap would leave
it) instead of pushing on toward a target the camera may never confirm
(k < 1, D off); a re-view bob still runs and ends it `SHORT`. RUN keeps its
guard unchanged (a frozen or fooled camera during the profile). The status
keeps a MOVE's measured progress once it has ended, whatever ended it (the
snap epoch used to restart it at 0: 'fwd -0.01 m' after ~0.8 m flown); the
Mac reports it as `travelled_m`. On the tiger2 replay the move now ends
`SHORT` at the settle's end with 0.859 m forward and 0.151 m sideways
measured (from the logged stop on the camera keeps its last value: what the
log shows after it is the drone's answer to that stop). `endTurn` (once per
turn) runs from flash.

*Corrected after the verification of e4f7686* (same evening): e4f7686
widened the settle's bounds further, to back past the start (not the
target) on a commanded axis and 0.15 m + 30 % of the whole move (0.45 m for
1.0 m) across it, the forward axis of a sideways move included. That never
helped a real under-shoot (see above) but let a regress toward the start
run on to the 3 s timeout: a forward 1.00 m at D 2.5 m whose reading fell
from 0.75 to 0.55 / 0.30 / 0 of the target stopped 740 / 480 / 340 ms into
the settle on e46bf90 and only at +3000 ms on e4f7686; a sideways 0.60 m
with 0.18-0.30 m of forward drift ended DONE where it used to stop. Now: the
same falling reading stops `PROGRESS` as it passes the target minus the
margin (0.567 m of 1.00 m; `testSettleGuard`), 0.16 m forward or a false
0.16 m backward in a sideways move's settle stops it, and the tiger2 replay
still ends `SHORT` (0.859 m forward, 0.151 m sideways).

What to watch: a forward or back move whose RUN meets 0.15 m of sideways
wander still stops `PROGRESS` (the RUN margin is unchanged; tiger2 reached
0.13 m); so does a sideways move whose settle drifts 0.15 m toward or away
from the scene; a `SHORT` end means the camera measured less than the
command or the drone off its line, so take a photo before the next step.

**Faster turns** (user decision 2026-10-13, evening): YAW at 30 deg/s and
60 deg/s^2 (15 and 30 before; the spec's ceiling is 45 deg/s). A 180 deg
turn takes 6.5 s instead of 12.5 s, then the dwell and the re-track bob as
before. What had to change with it, and why:

- *The hold's coasting.* camera_flow.c refused every frame turning faster
  than 0.35 rad/s (20 deg/s); a coasting hold pauses the motion, so at
  30 deg/s the turn paused, the drone slowed below 20 deg/s, resumed, and
  so on: in the interactive sim a 30 deg/s YAW 90 then took 5.2 s and a
  YAW 180 10.0 s (~18 deg/s), never faster than before. While a commanded
  turn runs (the recorder's `mvState`: a YAW in RUN, PAUSED or SETTLE, i.e.
  its turn, dwell and bob) the gate is now `AUTO_MV_TURN_GATE_RPS` = 0.785
  rad/s (45 deg/s) (`camHoldYawOk`, `camHoldTurnActive`, camera_hold.c);
  manual flight and every other autonomy phase keep 0.35.
- *The camera height observer.* Its quality gate refused the same frames;
  with the hold's gate widened alone, a 30 deg/s turn of 90 deg or more has
  no accepted frame for over 2 s (the healthy window): the sim landed
  `VZ` (`HOLD_NO_CAM`) in the middle of YAW 90 and 180. In a commanded turn
  (the glue's `camGuessOk`, `camVzObsSetTurn`) its yaw test is 45 deg/s too;
  the yaw restart of its gain tracker (0.35 rad accumulated) and the
  re-track bob after the turn are unchanged. `sizeof(camVzObs_t)` is
  unchanged (the flag sits in padding) and with the flag false every
  logged replay (cam_vz_shadow, cam_vz_replay_ih, hold_replay) is
  bit-identical.
- *Rotation removal at 30 deg/s.* The rotation model (FLOW_KX_*, R^2 0.97)
  comes from a hand-held recording (flowcal1, 2026-10-10) that turned up to
  ~45 deg/s: binned by rate its |dxRes| (the derotated sideways flow, hand
  motion included) has median 0.13-0.31 px and p90 <= 0.9 px at 20-45
  deg/s, max 1.24 px, under the scene gate's 1.5 px, and the flow fit was
  ok on every frame up to 6-9 px of shift (the flow test goes to 3.5 px; a
  30 deg/s turn is 2.4 / 4.8 px per 45 / 90 ms frame). A gain error in the
  model costs the same per degree turned at any rate (error x angle x D).
- *The HOVER scene gate.* Its residual limit (8 grey levels) is the one
  that tightens with speed: on the hand-held scene (texture ~100) the fit
  residual's median grows from 4.8 at rest to 8.3-8.5 at 25-35 deg/s; the
  flight scenes of 2026-10-13 (texture ~35) ran at half of the hand-held
  residual (2.4 at rest, 4.0 at 15-20 deg/s), so expect ~5-6 at 30 deg/s
  and some frames above 8: each pauses the turn for a frame (a coasting
  hold). Not changed; 2 s of it lands `MOVE_NO_CAM` as before. The tiger2
  turn of 2026-10-13 was gated at 15 deg/s already (residual 8.6-13 and an
  expansion with a near object late in the turn, at a 3.0-3.2 V battery).
- *The yaw response.* The logged 15 deg/s turns ran 15.8 +/- 2.0 deg/s per
  frame (max 18.8) in their cruise; second-order fits to the logged turns
  (w 7-19 rad/s, zeta 0.65-0.9) give at most 31.3 deg/s and a 3-6 deg lag
  for the 30 deg/s profile, so frames stay under the 45 deg/s gate with
  ~10 deg/s to spare.
- *The re-track bob, the progress guard.* Unchanged: the bob runs after the
  turn's dwell as before (its own clock), and a YAW has no progress guard
  (a MOVE after it needs a G of the new view, as before). The air time a
  YAW needs (turn, dwell, the longest bob, a LAND) is 6 s shorter at 180 deg.

Interactive sim (`testFastTurn`, 12 sorties of YAW 90 / 180 / -45 at
30 deg/s): each turn takes exactly its profile (3.5 / 6.5 / 2.0 s), never
pauses, the hold is engaged on every frame of it, it ends DONE with the
heading exact and the drone lands on the Mac's LAND; the drift during the
turn and its dwell / bob <= 0.17 m (<= 0.24 m for the same sorties at
15 deg/s). The Mac's YAW timeouts follow `YAW_RATE_DPS` (25 s + 2 x the
turn's time; checked against the header) and its `yaw` help gives the rate.

What to watch on the first turns: the recorder's `holdFlags` should stay 3
(engaged) through the turn, with 7 / 15 (coasting / scene-gated) only on a
few frames; `mvState` 33 (RUN YAW) without 34 (PAUSED) runs; `cvFlags` still
showing accepted frames (upper nibble 9) during the turn; the heading
reaching the command 0.1-0.2 s after the setpoint; per-frame yaw rates
(`flowGz_e4` / `flowDt_100us`) under 40 deg/s.

**Totals of the three changes** (e46bf90 -> them, with the corrections
after the verification of e4f7686; both built here on the int-sdkconfig):
static IRAM -36 B (`.iram0.text` 0x1fad3 -> 0x1faaf: the settle guard's
lateral margin choice in `autoMotionStep`, `endTurn` moved to flash, -99 B,
the observer's turn flag +16 B in `camVzTask`; the take-off hold none: its
getter and `vzRampAtTop()` are flash), the text 61 B under
`_iram_text_end` 0x40393f00 (25 B on e46bf90), so `endTurn` stays in flash;
.bss +8 B (`margLatS`, `shortEnd`), `_heap_start` 0x3fcc0f10 -> 0x3fcc0f18:
8 B less internal heap at boot. `callgraph.py` from `autonomyArbitrate`
against e46bf90's build: the one difference is `endTurn` (once per turn,
from `autoMotionStep`), now a flash target; nothing new runs from flash per
loop. Host tests: 0 failures in every C run (30 runs of 28 programs;
test_auto_motion 189, test_motion_replay 6, test_interactive_sim 42,
test_vert_ctrl 307, test_autonomy_glue_host 338, test_controller_pid_ihold
12, test_cam_vz_obs 2863, test_camera_hold 66), the Python suite 185 tests
OK.

## 15. Power: motor pulse placement and Wi-Fi HT20 (user decision 2026-10-13, evening)

The power investigation of 2026-10-13 found that flights end on voltage
sag, so firmware can win only a few percent. From its list the user chose
two firmware changes, and not the quiet Wi-Fi option (`tkoff wifiq`, still
off by default). Neither change touches the control law, a limit, a guard
or a setting: the controller computes and commands the same motor duties.

### 15.1 Motor pulse placement (`utils/src/motor_pwm_place.c`, `motors.c`)

The four motor channels share one 8-bit LEDC timer (40 kHz, a 25 us
period of 256 counts). A channel is on from its hpoint for `duty` counts.
Since 2026-10-11 the pulse starts have been spread over the slack of the
period (hpoint = id x (255 - duty) / 3), so the spin-up current peaks do
not coincide (coinciding starts browned the drone out at 2-10 % duty). At
hover duty (65-85 %) the spread has 1, 2, 3, 4, 3, 2, 1 motors on in turn,
so the battery current swings by up to three motor currents every period,
and the cell and connector (~0.15-0.2 ohm) heat with its square.

Now a motor above 50 % duty uses the **aligned** layout: M1 and M3 (one
diagonal, the same spin direction) start at the period start (hpoint 0),
and M2 and M4 end at its end (hpoint 255 - duty). Two to four motors are
on, never one: the least swing possible without a pulse wrapping over the
period end. The ripple metric (`tools/tests/test_motor_pwm_place.c`) is the
variance over a period of the number of motors on:

| All four motors at | Spread | Aligned | Change |
|---|---|---|---|
| 65 % (duty 166) | 1.327 | 0.866 | -35 % |
| 75 % (duty 191) | 1.367 | 1.031 | -25 % |
| 85 % (duty 217) | 1.074 | 0.879 | -18 % |

The mean current is the same. Over 20000 unequal hover sets (each motor
within +/-15 counts of the others, 60-90 %) the variance is lower in every
one: at least 16 % and 34 % on average. The skeptic's estimate of the
gain is +0.5-2 % of air per charge. It is smallest at the 80-88 % duties
near the end of a charge.

The rules:

- **Per motor, with hysteresis:** aligned from duty 141 (55 %), the spread
  again at or below 128 (50 %), unchanged in between. The take-off ramp and
  a landing cross 50 % once, and 3-count loop noise does not toggle the layout.
- **At or below 50 % the spread is exactly as before**, whatever came
  before. The spin-up is unchanged.
- **M1 and M4 do not move:** they sit at 0 and 255 - duty in both
  layouts. Only M2 (to 255 - duty) and M3 (to 0) move, by up to 84 counts
  for one period at a layout change. The pulse length stays the same.
- Duty 0 is hpoint 0, as before. hpoint + duty <= 255 always, so no pulse
  wraps. The duty is at most 255, so at least one count per period is
  low: no channel is ever full-on.

**A side effect: every motor now sees the same supply.** In the spread,
the middle pulses (M2 and M3, the back motors) overlap more of the others.
During their pulses 0.1-0.4 more motors are on than during M1's and M4's
(2.15 against 1.73 at 61 % duty, 2.56 against 2.33 at 75 %). So the back
motors ran on a slightly lower voltage. In the aligned layout all four see
the same (1.73 at 61 %, 2.33 at 75 %). Expect the back-minus-front duty split
at hover, (m2 + m3) - (m1 + m4), to fall a little. How much it falls shows
how much ripple gets past the board's capacitors to the motors. The pitch
part of the trim moves by the same amount, and the integrators and the trim
learner absorb it.

This also overlaps the BL experiment (the battery cable re-routed away
from the back-left leg). For clean data, fly the cable change on the
current firmware first, then this build.

**One latch per update.** IDF 5.0.7's `ledc_set_duty_with_hpoint()` sets
LEDC_PARA_UP right after writing the hpoint. `ledc_update_duty()` then sets
it again with LEDC_DUTY_START, which loads the duty. Each call has its own
critical section. A timer overflow between the two runs one period with the
new hpoint and the old duty. When a right-aligned pulse shortens, that sum
passes 255, the pulse does not end in its period, and it stays on into the
next one. This has applied to M4 since 2026-10-11. For per-loop steps of up
to 20 counts it is 4890 of 10035 duty steps for M4 before, and 2264 for M2
now. `ledcWriteLocked()` (motors.c) now writes every field first and sets
LEDC_PARA_UP once, with this core's interrupts masked. The register values
are the same as the two IDF calls: hpoint, the duty's integer part, no fade,
output enabled, duty start. The hardware takes hpoint and duty at the same
overflow, as IDF 5.3's driver does (its `ledc_duty_config()` no longer sets
LEDC_PARA_UP). The disassembly shows that sequence and one PARA_UP store.
`#error` guards stop the build on a target other than the ESP32-S3 or an
IDF outside 5.0.x, so it is re-checked before an upgrade.

The stop paths are unchanged:
- `stopLocked()`, `motorsEmergencyStop()`, `motorsEmergencyStopNoWait()`:
  pins low through the GPIO matrix, then `ledc_stop()` at idle level 0.
- After a STOP, `readyLocked()` refuses every write.
- The inhibit still forces duty 0.

**USB.** `pwm` without an argument now also prints the live pulses:
`OK pwm 40000 hpoint=0,99,0,99 duty=156,156,156,156 aligned=0xf` (aligned:
bit n = motor n+1). At or below 50 % it reads, for example,
`hpoint=0,65,131,197 duty=58,...,58 aligned=0x0`. The hpoint is the value
written; the duty is the one running.

### 15.2 Wi-Fi access point at 20 MHz (`wifi_esp32.c`)

IDF 5.0.7 starts the soft-AP at HT40 (channel 1 with a secondary channel
above it). `wifiInit()` now calls `esp_wifi_set_bandwidth(WIFI_IF_AP,
WIFI_BW_HT20)` after `esp_wifi_set_mode`/`esp_wifi_set_config` and before
`esp_wifi_start`. It reads the bandwidth back after the start, sets it once
more if the start did not keep it, and logs:

`wifi: AP bandwidth HT20 (set: ESP_OK, read: ESP_OK, not saved), channel 1, secondary 0`

A failure keeps HT40, is logged, and never stops the boot. `net` prints
`bwMhz=20` (40 for HT40, 0 unknown). The expected saving is about 3 mA (the
receive current, 91 -> 88 mA, when a station would use 40 MHz), +0.2-0.4 %
of air. The link also shares a busy band better.

**HT20 belongs to this build only; it is never saved.** With its default
storage (`WIFI_STORAGE_FLASH`, `CONFIG_ESP32_WIFI_NVS_ENABLED=y`) the Wi-Fi
driver saves `esp_wifi_set_bandwidth()` to its own NVS namespace
(`nvs.net80211`: `ap.phybw`, and `ap.sndchan` set to none), and
`esp_wifi_init()` loads it on every later boot, older builds included.
Flashing back an earlier build would then have kept HT20, so an A/B against
the previous build would have run HT20 on both sides, and a rollback would
not have undone an HT20 link problem. Restoring HT40 would have needed an
NVS erase, which also loses the calibration, trim and settings. So
`wifiInit()` sets the bandwidth (both calls, before and after the start)
between `esp_wifi_set_storage(WIFI_STORAGE_RAM)` and
`esp_wifi_set_storage(WIFI_STORAGE_FLASH)`. In IDF 5.0.7's libnet80211,
`ieee80211_set_phy_bw()` passes "store = the storage is FLASH" to
`wifi_nvs_set()` for both keys, and `wifi_nvs_commit()` returns early under
RAM, so only the driver's RAM copy changes. The soft-AP start at HT20 on
channel 1 writes neither key. If RAM storage cannot be selected the
bandwidth is not touched and HT40 stays (the log shows the error). The
other settings `wifiInit()` makes (`esp_wifi_set_mode`/`set_config`) are
written to the Wi-Fi NVS with the same values as by every build before;
TX power and the 11b rates are not saved. The first power build,
`espfly-power-b1a745b.bin`, made the call without the RAM storage and so
saves HT20: do not flash it. It had not been flashed when this was fixed,
so the drone's Wi-Fi NVS still holds the HT40 default.

The following are unchanged: the SSID and password, channel 1, the 11 dBm
TX power, the take-off 2 dBm (`txlo`), the quiet option, the IP and DHCP
set-up, and the UDP ports for the phone app and the Mac.

### 15.3 Cost and tests

Built against e4f7686, with the int-sdkconfig build:

- **IRAM is unchanged:** `.iram0.text` 0x1fad3, ending 41 B under
  `_iram_text_end` 0x40393f00.
- **Internal RAM is unchanged:** .data and .bss are the same size, and
  `_heap_start` stays 0x3fcc0f20. The internal heap at boot is the same as
  on e4f7686. The hysteresis state is one byte that fits in the existing
  padding, and the interrupt mask needs no spinlock.
- **Flash grows by about 2.2 KB:** +1.8 KB of code (the Wi-Fi library's
  bandwidth, storage and channel calls, the placement, the `pwm` report) and
  +0.4 KB of read-only data.
- **Per loop:** `motorsSetRatio()` has run from flash since stock. It now
  calls `motorPwmPlaceHpoint()` (124 B) and writes the registers inline,
  instead of the two IDF driver calls with their argument checks, critical
  sections and fade-lock checks. Its stack frame is still 32 B.
- **Stacks:** the USB console's frames are unchanged (the `pwm` report is
  out of line). `wifiInit()`'s frame grows by 32 B, and it calls
  `apSetHt20ThisBoot()` (a 32 B frame), once, at boot.

Host tests:
- `test_motor_pwm_place` (48 checks) covers the limits for every id, duty
  0..300 and history, the spread kept at or below 50 %, the aligned pairs,
  the hysteresis (ramps, band jitter, noise at either threshold), a take-off
  and landing through 50 %, and the ripple table above.
- All 31 C runs (29 programs) report 0 failures.
- The Python suite: 185 tests OK.

Bench and flight checks: docs/autonomy-bench.md section E.

## 16. End of battery (2026-10-14)

User decision 2026-10-13, late: "lower the limits again, try to squeeze as
much time as possible, while it lands safely as well." Until then a sortie
that ran the battery down ended on the vertical controller's `HOLD_CAPPED`:
the hover thrust reached the 54000 ceiling for 0.5 s at about 3.06-3.12 V
loaded (the cell resting at 3.7-3.85 V), and the controller started a fast
LAND from a reference measured at the cap. Now the last flight of a charge
ends on the state machine's `BATT_LOW`, a normal slow LAND, deeper into the
battery. The earlier decisions stay: battery limits removed (`VBAT_LOW` 2.60
V for 1 s, the 1 kHz `VBAT_DIP` at 2.55 V), NVS `vmin` 3.30 V, a contact is a
slow LAND, no gain or tuning change, the person at least 1 m away.

### 16.1 The rules

| What | Value | Where |
|---|---|---|
| Thrust ceiling (RAMP, CLIMB, HOLD, the final clamp, FLIGHT `thrustMax`) | 58000 (was 54000) | `VZ_THRUST_ABS_MAX` |
| `BATT_LOW` trigger 1, the floor | every 10 Hz reading below 3.00 V | `AUTO_VBAT_LOW_LAND_V` |
| `BATT_LOW` trigger 2, the headroom | T_hold (`vzOutput_t.tHold`, the 0.25 s average of the applied thrust, 0 before HOLD) above 57000 | `AUTO_HEADROOM_LAND_T` = ceiling - 1000 |
| `BATT_LOW` persistence | either trigger on every step for 0.5 s (was 1 s), HOVER only, one tracker | `AUTO_VBAT_LOW_LAND_MS` |
| MOVE / YAW refused (`AUTO_MVW_BATT`, reply `LOW_BATT`) | T_hold above 53400 when the command arrives, or the `BATT_LOW` condition held 0.25 s (was 0.5 s) | `AUTO_MV_REFUSE_T` = ceiling - 4600, `AUTO_VBAT_LOW_REFUSE_MS` |
| HOLD's `capLo` ceiling | `capLo T_lo max(V_lo, Vf) / Vf` (CLIMB keeps `capLo T_lo`) | `climbCeiling()` |
| Motor stop in `powerDistribution()` | `control.thrust` > 60000 (unchanged), now a name with a compile-time check that the ceiling stays 2000 below it | `POWER_THRUST_STOP` |

Unchanged: `HOLD_CAPPED` (0.5 s at a ceiling) and `CLIMB_CAPPED` (0.7 s),
`K_RANGE` (0.88-1.12 x kPrior), `EST_RANGE`, `HOLD_NO_CAM`, `HOLD_TIMEOUT`,
`VBAT_LOW`, `VBAT_DIP`, `capKhat` (1.15 x Khat / Vf), the 80 s air clock,
the 60 s wait (both raised to 120 s later: section 18), `vmin` 3.30 V, the AUTO_STATUS layout and every reason code
(`BATT_LOW` is still 39). STOP, PHOTO and LAND are never refused; a motion
already running ends with the LAND itself.

In HOVER the levels come in this order as the battery runs down: MOVE / YAW
refused (T_hold 53400, about 3.05-3.1 V loaded), `BATT_LOW` (a loaded mean of
about 2.96 V, or T_hold 57000), `HOLD_CAPPED` at 58000, `VBAT_LOW` 2.60 V,
`VBAT_DIP` 2.55 V. `VBAT_LOW` remains the guard of TAKEOFF and CLIMB, where
`BATT_LOW` does not apply; in HOVER `BATT_LOW` always fires first.

The AUTO_STATUS `battLow` bit 0 is the `BATT_LOW` condition (either trigger)
held 0.25 s, bit 1 the condition now. A refusal for T_hold over 53400 alone
does not set bit 0 (that would have cost 199 B of IRAM in `autoFsmStep`): the
Mac learns it from the `LOW_BATT` reply, whose hint now says the drone lands
by itself and that hover, photos, STOP and LAND still work.

### 16.2 Why these numbers

**The floor, 3.00 V for 0.5 s.** `pmGetBatteryVoltage()` is a 30-sample ADC
mean every 100 ms; in hover the readings scatter with an sd of 29-39 mV
(measured), so 5-6 readings in a row below 3.00 V mean a loaded mean of
about 2.95-2.97 V.

**The headroom, 57000 for 0.5 s.** At the measured K of 165-171k (hover
thrust x Vbat), 57000 is 2.89-3.0 V loaded, so the two triggers meet at a
typical K: the floor ends the low-K flights, the headroom the high-K ones
before the ceiling. The end must be this slow LAND, never `HOLD_CAPPED`. A
race replay of the real hover thrust noise (5 flights, sd 170-334 counts)
on a hover need rising toward the 58000 cap:

| Margin, persistence | Slow LAND first, rise 1000 counts/s (~43 mV/s) | 2000 counts/s |
|---|---|---|
| 1000 counts, 0.5 s (chosen) | 100 % | 99 % |
| 1000 counts, 1.0 s | 82 % | |
| 600 counts, 0.5 s | | 70 % |

`HOLD_CAPPED` stays the backstop for a runaway or a voltage collapse faster
than about 1000 counts/s. A runaway capped from a normal hover lifts T_hold to
57000 only after about 0.8 s, so `HOLD_CAPPED` still catches real runaways
first. `capHitHoldS` is not lengthened: 1 s at `capKhat` would allow 0.74 m
of runaway climb instead of 0.18 m. Brief touches of 58000 are still ridden
through (the 0.5 s continuous rule).

**The ceiling, 58000.** 60000 is the hard limit: `powerDistribution()` stops
every motor above it, and with tilt compensation off `control.thrust` is the
vertical controller's thrust. The 8-bit duty reaches 65280. Per motor, with
the p99 steady roll/pitch/yaw correction of +3.0k:

| Balance | At the 57000 land level | At 58000 (brief touches) |
|---|---|---|
| Battery forward (battfwd2 f2, FR/BR/BL/FL 70/72/69/74 %): FL +3.5 % | 59.0k static, 62.0k with the correction (3.3k margin) | 60.0k, 63.0k (2.3k margin) |
| The old imbalance comes back (BL +5.3 %, e.g. the battery shifts) | 63.0k | 64.1k (1.2k margin) |

Motions are refused above 53400: the hardest motor is then 55.3k, plus the
manoeuvre p99 correction (5.7k) 61.0k (61.9k with the old imbalance). Below
53400 a MOVE up or a bob (+4 %), a lean and the settle's loop swing stay
under 57000; in the drain sim (16.5) a refusal level of 55100 let moves
started near the end hit `HOLD_CAPPED` (impact up to 1.4 m/s), 53400 removed
every case. 59000 would leave 0.2k at p99 with the old imbalance; 60000 cuts
the motors and puts FL at its p99 limit in a calm hover; above 58000 the
modelled gain is only +0.2-0.9 s.

**HOLD's `capLo` ceiling, battery-scaled.** `capLo T_lo` (1.65 x the thrust
at lift-off) is in thrust units, and thrust units grow as the battery drains:
on full-charge sorties it bound at 51.7-58.9k, so raising 54000 alone would
have gained nothing. In K units the end hovers of the logs need only 1.21-1.35
x K_lo (9 flights), so HOLD now bounds `capLo K_lo` (`capLo T_lo max(V_lo,
Vf) / Vf`, never tighter than before). CLIMB keeps thrust units: the same
scaling there failed the capLo 1.10 runaway tests (peak 1.82-1.98 m against
0.80). `capKhat` is unchanged.

**Side effects.** The RAMP's `NO_LIFTOFF_CEIL` moves from 2.97 V to 2.77 V
loaded (kPrior 160.4k); `CLIMB_CAPPED` take-offs from below about 3.80-3.85 V
at rest to below about 3.70 V (3.78 V for a drone 5k heavier than the pooled
fit; vzK 160.4k, 16.5). Take-offs from a weak cell are not short hops (16.5, 16.7): from
3.65-3.70 V at rest nearly all end `CLIMB_CAPPED` / `NO_LIFTOFF` (fast, up to
0.9 m/s from up to 0.21 m) with the loaded battery at 2.6-2.9 V in the air,
and the higher CLIMB ceiling lets Khat reach its `K_RANGE` clamp in the climb
of a drone whose K is over about 1.07 x vzK.

**Margin above the brown-outs.** The deepest 40 kHz in-flight survival is
2.838 V (10 Hz); the 15 kHz in-flight resets of 2026-10-10/11 were at
2.72-2.75 V; the expected 1 kHz minimum at the LAND start is about 2.75-2.86 V,
0.2-0.3 V above `VBAT_DIP` (2.55 V); the brown-out detector (level 7) is at
about 2.44 V.

### 16.3 How the LAND runs

`BATT_LOW` puts the state machine in LAND, which sends `VZ_CMD_LAND`; the
controller starts its LAND from HOLD with reason `CMD_LAND`. In FLIGHT HOVER
the camera estimate is in use (`HOVER_NO_CAM` guarantees it), or the height
is above 0.6 m, so it is the slow LAND: T_ref = T_hold, the measured hover
need, uncapped (the rule fires at least 1000 below the cap); the ceiling
starts at 0.98 x T_ref and holds for `landFlatS` (1.5 s at 0.3-0.4 m, 2.0 s
at 0.6 m, 4.0 s at 1.2 m, plus the drift allowance after holds longer than
18 s); the loop flies -0.30 m/s, -0.20 m/s below 0.20 m; then the ceiling
falls 0.10 x T_ref/s to 0.65 x T_ref, and the stall rule ends a float.
Thrust never exceeds 0.98 x T_ref (at most about 56.7k), so every motor has
more headroom than in the hover just before (the hardest at most about
61-62k at p99). The fast LAND of 17.4 (2026-10-14) is never this LAND: it
needs T_ref <= 53400 and a loaded Vf of 3.15 V or more, and `BATT_LOW` fires
at T_hold over 57000 or at a battery under 3.00 V (`testDrain` checks that no
`BATT_LOW` LAND is fast). Before, the end was `HOLD_CAPPED`: T_ref measured at the cap
(below the real need), a ceiling starting at 0.95 and falling at once, and a
drone accelerating downward.

The first step down (to 0.98) lowers the current about 3 % and raises the
voltage about 0.02 V; after that the descent holds about the hover thrust
while the cell keeps draining (16.5: the lowest loaded voltage in the LAND).

### 16.4 The 3.3 V rail meter is not an end rule

The rail estimate ('3.3V rail min' of `flights`, `flightRtcLogLive()`
`railNow` / `railMin`) is one ADC1 conversion (channel 8, 11 dB) of GPIO9,
the LED pad driven high, taken by the priority-1 `VBATMON` task between
battery readings, so at most once a millisecond (`analogReadRailOnce()`,
adc_esp32.c). It saturates at the 11 dB full scale (about 3.16-3.27 V on
this drone), reads tens of mV low (the LED current) and misses sub-ms dips.
The logged 40 kHz resets are 12 brown-outs and 1 panic (2026-10-11 i boots
15-20, n 2, o 2, q 1-2, s 5, t 8-9), all in the take-off window (0.16-2.84 s
of air) at a 10 Hz battery of 3.07-3.40 V, with rail minima of 2.74-2.85 V;
40 kHz flights that did not reset read as low as 2.82 V (10-11i boot 21,
10-12 evening f3, 10-13 tiger2 b3). The ranges overlap, and the resets came
with the battery far above the new end: sub-ms take-off transients the meter
does not see. In the long autonomy hovers the rail minimum sat 0.10-0.29
V under the 10 Hz battery minimum (night f2 / f3 / f6, tiger2 b2 / b3,
interactive f1, battfwd2 f2, pwmA1 f7), so at the new end (10 Hz about
2.90-2.95 V) it should read about 2.6-2.8 V, lower than any 40 kHz flight so
far. A rail threshold would either never fire or end normal flights at random,
and wiring it in would cost IRAM and four test rigs. It stays a logged
post-flight number that cannot clear a flight; the stop rule is any reset
(16.7).

### 16.5 Verification (host)

**Closed-loop drain sim** (`test_interactive_sim.c` `testDrain`, ported from
the designer's prototype): the real `auto_fsm`, `auto_motion`, `vert_ctrl`,
`camera_hold` and `cam_vz_obs` on the sim's plant with a draining battery: a
generic LiPo OCV(q) table, a sag of 1.15e-5 V per count (0.62 V at 54k)
growing 50 % below 20 % charge, capacity C in A s, K = 206.4k - 12.3k x V
(the pooled fit) + an offset, 35 mV of noise on the 10 Hz reading; a MOVE
every 6 s and STOPs to keep the wait alive. The grid: 3.82 V at rest (night
f6), C 150 / 250 / 400 / 600 A s (end slopes at 54k of about -12 to -3 mV/s;
C 400 matches night f6's 28.5 s to 54k and about 84 s per charge), K offset
-5k / 0 / +5k, 0.30 / 0.60 / 1.20 m, 3 seeds: 108 sorties.

| | This build | 6742d91 (the designer's run of the same sim) |
|---|---|---|
| Endings | 107 `BATT_LOW`, 0 `HOLD_CAPPED`, 0 `CLIMB_CAPPED`, 1 `K_RANGE` (a 44 s hover at 30 cm whose height estimate drifted down to the ground; 0.06 m/s) | 62 `HOLD_CAPPED`, 42 `CLIMB_CAPPED` (take-off at 3.82 V), 4 other |
| Touchdown | p50 0.32, p90 0.46, max 0.71 m/s (0.47 for C >= 250; 0.71 the steepest collapse, C 150 from 1.2 m) | p50 0.87, max 1.85 m/s; `HOLD_CAPPED` from 1.2 m 1.31-1.39 |
| Loaded V at the LAND start | 2.90-3.05 V | |
| Lowest loaded V in the LAND | 2.89 V (C 400), 2.85 V (C 250), 2.76 V (C 150 at 1.2 m) | |
| MOVE accepted above T_hold 53400 | none (235 `LOW_BATT` refusals) | |

The test checks every sortie: no capped ending and no KILLED, a touchdown,
<= 0.50 m/s for C >= 250 and <= 0.75 m/s for C 150, thrust <= 58000 always
and <= 0.98 x T_ref in the LAND, loaded >= 2.88 V when `BATT_LOW` starts the
LAND, no MOVE accepted above 53400, at least one `LOW_BATT` refusal.

Full hovers from 4.10 V (60 and 120 cm): C 400 lands `AIR_TIME` at 80 s
(3.13 V loaded, 0.23-0.31 m/s), as 6742d91 does (with the 120 s clock of
section 18.1: `BATT_LOW` at 91 s, 2.96 V, 0.35-0.43 m/s); C 300 `BATT_LOW` at 68 s
(2.97 V, 0.35-0.38 m/s; 6742d91: `HOLD_CAPPED` at 62-64 s, 0.69 / 1.44 m/s);
C 200 `BATT_LOW` at 45 s (2.96-2.97 V, 0.30 / 0.61 m/s).

The step-drop battery test of the same sim (the rest voltage falls 30 s
into the wait) is defined against the floor (0.40 V over it), so it moved
from a 3.05 V to a 3.40 V step with this change. Like for like: on the 3.40 V
step this build lands `BATT_LOW` at 28.7 s, 0.48 m/s, where 6742d91 lands
LAND VZ (`HOLD_CAPPED`) at 28.9 s, 1.15 m/s; on the old 3.05 V step (now a
second, pinned case: about 2.5 V loaded at the hover thrust, faster than the
floor can act) this build lands `VBAT_DIP` at 28.3 s, 1.69 m/s, where 6742d91
lands LAND VZ (`HOLD_CAPPED` at 54000) at 28.8 s, 1.53 m/s: the 58000 ceiling
pulls the loaded voltage under 2.55 V first, so this deep sudden collapse is
slightly harder (bounded as before: no KILLED, a touchdown, <= 2.0 m/s). The
glue host's `batt-low` scenario (also relative to the floor, 0.38 V over it:
now a 3.38 V step) lands `BATT_LOW` at 2.86 V loaded, 0.51 m/s; 6742d91 on the
same 3.38 V step lands LAND VZ at 0.71 m/s, and on its old 3.03 V step LAND VZ
at 1.10 m/s, where this build lands `BATT_LOW` at 0.91 m/s.

**At the drone's K prior** (`testDrainVzK`, after the verification of
cb09352). The grid above runs at `vzDefaultConfig`'s kPrior of 165000; the
drone's NVS vzK is about 160.4k. The plant's K is the pooled fit (+ the
offset) whatever the prior, and with the 58000 CLIMB ceiling Khat now reaches
its 1.12 x kPrior clamp in the take-off's climb (0.3 s) before the ceiling
freezes it:

| vzK | `BATT_LOW` | `K_RANGE` (in the take-off climb) | Touchdown max |
|---|---|---|---|
| 165000 | 107 | 1 (0) | 0.71 m/s |
| 160400 | 83 | 25 (24, all K +5k; the plant K >= 1.075 x vzK) | 0.71 `BATT_LOW`, 0.69 `K_RANGE` |
| 158000 | 76 | 32 (31, 7 of them K +0; >= 1.058 x vzK) | 0.71, 0.77 |

No capped ending, no KILLED, a touchdown in every sortie, every `BATT_LOW` as
in the grid at 165000; the take-off `K_RANGE` LAND is fast, from about 0.25 m
(at most 0.33 m). 6742d91 at 160400 ends the same grid 64 `HOLD_CAPPED` and
39 `CLIMB_CAPPED` (and 3 `NOT_STABLE`, 2 `K_RANGE`; up to 1.80 m/s). The test
checks all of that, `K_RANGE` touchdowns <= 0.80 m/s, and at 160400 no
take-off `K_RANGE` for a drone on the pooled fit (K offset <= 0).

**Weak-cell take-offs** (`testWeakTakeoff`): 3.60-3.85 V at rest, C 250 /
400, K -5k / 0 / +5k, 0.3 / 0.6 / 1.2 m, at vzK 160400 (158000 similar):

| Rest V | Fast take-off LAND (K <= 0 of them) | Touchdown max, from | Lowest loaded V in those | 6742d91 (fast LANDs, touchdown, loaded V) |
|---|---|---|---|---|
| 3.60 | 0 (18 `VBAT_DIP` in the RAMP, hops <= 2 cm) | 0.24 m/s | 2.55 V | 9 (8 `NO_LIFTOFF`) and 9 `VBAT_DIP`, 2.55 V |
| 3.65 | 18 (12): 12 `CLIMB_CAPPED`, 6 `NO_LIFTOFF` | 0.46 m/s, 0.08 m | 2.62 V | 18, 0.28 m/s, 2.69 V |
| 3.70 | 17 (11): 16 `CLIMB_CAPPED` | 0.90 m/s, 0.21 m | 2.83 V | 18, 0.59 m/s, 0.11 m, 2.90 V |
| 3.75 | 4 (0) `CLIMB_CAPPED` | 1.14 m/s, 0.30 m | 3.07 V | 11 (5), 1.18 m/s |
| 3.78 | 2 (0): 1 `CLIMB_CAPPED`, 1 `K_RANGE` | 1.22 m/s, 0.41 m | 3.10 V | 10 (4), 1.01 m/s |
| 3.80 | 4 (0), all `K_RANGE` | 0.74 m/s, 0.29 m | 3.13 V | 10 (4) `CLIMB_CAPPED`, 1.08 m/s |
| 3.85 | 4 (0), all `K_RANGE` | 0.70 m/s, 0.27 m | 3.19 V | 4 (0), 1.02 m/s |

Checked: no KILLED, a touchdown after every lift-off, never `HOLD_CAPPED`,
touchdown <= 1.30 m/s, and from 3.80 V up no take-off LAND for a drone on the
pooled fit (vzK 160400). The sortie that reaches HOVER ends `BATT_LOW` as in
the grid.

**Unit tests.** `test_auto_fsm` (1388 checks): the headroom trigger (57001
lands at 500 ms with vzCmd LAND and the slow thrust, exactly 57000 never, one
step at 56999 restarts the streak, TAKEOFF ignores it), the floor (2.99 V for
500 ms lands, 3.00 V never, 2.99 / 3.00 alternating at 10 Hz never), the
refusals (T_hold 53401 refuses MOVE and YAW with the `battLow` bits clear,
53400 accepts; a battery streak of 249 ms accepts, 250 ms refuses; STOP and
PHOTO accepted), `VBAT_LOW` in TAKEOFF and `BATT_LOW` before it in HOVER,
`VBAT_DIP` in TAKEOFF and HOVER, the controller's `HOLD_CAPPED` still LAND
VZ. `test_vert_ctrl` (313): the HOLD ceiling on the exact value (T_lo 33000
at 3.70 V, HOLD at Vf 3.00: min(67.2k, capKhat, 58000), over capLo x T_lo;
T_lo 28000: exactly capLo x T_lo x 3.70 / 3.00; the same in CLIMB: capLo x
T_lo; HOLD at Vf above V_lo: capLo x T_lo); every sortie's invariant is now
CLIMB <= capLo T_lo and HOLD <= capLo T_lo max(V_lo, Vf) / Vf. Its identity
hash moves from 0x169CEE4B871A74D9 to 0x6E54AEE6FC4E3868: compared step by
step with 6742d91 over the 30 identity sorties the reported ceiling changes
in 23 and the thrust in none; the thrust changes only where a ceiling binds.
In its Monte Carlo rows (plant K down to -12 %) the 1-5 runs a row that
ended `CLIMB_CAPPED` / `HOLD_CAPPED` at 54000, and 0-4 `CMD_LAND` runs a
row, now end `K_RANGE` (the gain estimate reaches its clamp before the
ceiling); no row's peak or impact maximum grows. `test_auto_sortie`, `test_vert_replay`,
`test_collision_replay`, `test_auto_motion` pass unchanged. All 31 C runs
(29 programs) report 0 failures; the Python suite (186 tests, the new BATT texts and a
drone refusal for the hover thrust) passes.

### 16.6 Cost

Built with the int-sdkconfig against a rebuild of 6742d91: `.iram0.text`
129711 -> 129759 B (+48: `auto_fsm` +45 in its objects, `vert_ctrl` -14 with
`climbCeiling()` out of line in IRAM and one division for both bounds),
still in the same 256 B step (`_iram_text_end` 0x40393f00, 13 B left before
the next step after the 16 B prefetch pad). `.dram0.data` 31464 and
`.dram0.bss` 87344 are unchanged, and `_heap_start` stays 0x3fcc0f18, so the
internal heap is the same as on 6742d91. Flash code and read-only data are
unchanged in size. No new state.

### 16.7 Expected gain, risks, what to check

**Gain.** The extra air comes on the last flight of a charge, the one that
reaches the end; the spread is wide because the cell's end curve is unknown:
the analyst's cell model fitted to the measured end slope (-2 to -4 mV/s,
411 scenarios) gives p10 +17.5 s, median +28.0 s, p90 +39.7 s; the drain
sim's generic curve (C 400) a median of +11.0 s after the first T_hold of
53800 (C 600: +25.6 s, C 150-250: +4-6 s). Per charge (today 82-88 s) about
+5 % to +45 %, most likely +15 % to +35 %. A full charge flown as one sortie
still lands `AIR_TIME` at 80 s (about 3.13 V loaded in the sim) before the
battery runs out: the gain shows in the second or last sortie of a charge,
or if the 80 s / 60 s limits go up (the user's decision; not changed here;
they went up to 120 s on 2026-10-14, section 18, and the C 400 hover then
ends `BATT_LOW` at ~91 s).

**Risks.**
- Deeper discharge with thin data: the LAND starts at about 2.95-3.0 V (10 Hz
  loaded); the only 40 kHz in-flight data that deep is one manual flight at
  2.838 V. The rail meter is expected to read about 2.6-2.8 V there (16.4),
  lower than any 40 kHz flight so far and inside or under the range of the
  logged resets (2.74-2.85 V); it cannot warn of a brown-out, and it cannot
  clear a flight either.
- A steep end of the cell: a long LAND from 1.2 m can drain to about
  2.76-2.85 V (sim) before the touchdown. `VBAT_DIP` does not act in LAND;
  brown-out level 7 is about 2.44 V.
- Take-offs from a weak cell (16.5): from 3.65-3.70 V at rest nearly every
  take-off is a `CLIMB_CAPPED` or `NO_LIFTOFF` fast LAND (up to 0.9 m/s from up
  to 0.21 m in the sim; up to 1.2 m/s from 0.41 m at 3.75-3.78 V for a drone
  5k heavier than the pooled fit), with the loaded battery at 2.6-2.9 V inside
  the take-off window, where every logged 40 kHz in-air reset happened (10 Hz
  battery 3.07-3.40 V; and a take-off reboot on 10-13 at 3.80 V at rest). They
  are not short hops. `vmin` 3.80 V (the firmware default) avoids the
  `CLIMB_CAPPED` / `NO_LIFTOFF` ones in the sim (the `K_RANGE` ones: next
  item); 3.65 V does not. `vmin` stays 3.30 V as the user set it. A FLIGHT
  TAKEOFF refusal below about 3.80 V at rest would be the firmware's version
  of the same rule (not built: the user's decision).
- `K_RANGE` in the take-off: with the 58000 CLIMB ceiling Khat reaches its
  clamp (1.12 x vzK) in the climb when the drone's K is over about 1.07 x vzK
  (16.5: with vzK 160.4k, a drone about 5k heavier than the pooled fit; 24 of
  the 36 grid sorties at K +5k, every one climbing to 0.6 or 1.2 m, a fast LAND
  at up to 0.69 m/s from about 0.25 m; at vzK 158k also 7 sorties on the
  pooled fit). 6742d91 ended those take-offs `CLIMB_CAPPED`, so this is no
  new failure, but it costs the sortie. In the hover the end comes first: at the new end the hover K is
  about 168-172k against the clamp of 179.6k (vzK 160.4k). vzK is learned
  after every slow LAND (`auto get` shows it).
- Motor headroom assumes the battery-forward balance; if the battery shifts
  back, the hardest motor at 58000 is 64.1k at p99 (1.2k margin).
- The cell is discharged deeper and runs warmer: it will wear faster.

**What to check (autonomy-bench.md F).** Fly the first battery-end flights
at 0.6 m or lower; after each, save `flights` (the 1 kHz minimum, the rail
minimum). Any reset is the stop rule: after one in the hover's end or its
LAND raise `AUTO_VBAT_LOW_LAND_V` to 3.05 V, after one in a take-off raise
`vmin`. The rail minimum is logged, not a pass / fail: below about 2.85 V it
is inside the logged reset range, and about 2.6-2.8 V is expected at the new
end (16.4).

---

## 17. More air per charge (user request 2026-10-14)

The user asked for four changes that buy air per charge with the same function
and safety: the camera asleep while the drone waits on the ground, the camera
sensor at the rate the flow loop uses, a faster climb and a faster LAND from
height, and faster moves. Not in scope: the LEDs, the PWM frequency (60 kHz was
tested on 2026-10-13 and rejected), gains and tuning, the 80 s air clock, the
60 s wait (both raised later: section 18) and the battery thresholds.

| Item | Built | Expected extra air per charge |
|---|---|---|
| 17.1 Camera ground standby | yes | ~3.5-5.6 s typical, 8-13 s on a long-ground charge (most of the gain) |
| 17.2 Camera at 14.8 fps (XCLK 13.33 MHz) | yes, behind an NVS switch and an automatic fallback | ~0.2-0.4 s |
| 17.3 Faster climb (0.60 m/s) | **no** (the take-off `K_RANGE` sims) | (0-0.9 s per sortie, cancelled by the failures) |
| 17.4 Faster LAND from 1.00 m and higher | yes (revised after the verification: 17.4) | ~1-2 s |
| 17.5 MOVE top speed 0.30 m/s (0.20 toward the scene ahead) | yes (revised: approaches unchanged) | 0-2 s |

The per-sortie energy shares of the 2026-10-12/13 logs (power investigation,
`firmware-backups/handoff-2026-10-12/flighttime-report.json`): the idle wait 60 %,
the motions 13 %, the climb 12 %, the LAND 8 %.

### 17.1 The camera asleep on the ground (`utils/src/cam_gnd.c`, `camera_flow.c`)

**Why.** On battery the board draws ~0.16-0.25 A while the drone waits on the
ground: setup, the IMU heat-soak wait (up to 90 s after a flight; the first
flight after unplugging USB waits too), and between flights. The camera and
its flow work are an estimated 40-110 mA of it: the sensor's wake was measured
as a ~0.07 V rail drop over ~1 ohm (60-90 mA), plus ~10-20 mA of core-1 flow
work. The logged ground waits on battery: ready -> ARM 8-119 s (median ~35 s,
n 18), between flights 21-149 s (median ~77 s, n 10); one full charge
(2026-10-13 night) spent 684 s on the ground against 86 s in the air, with 6
ARMs.

**What.** The camera task reuses the take-off standby's sensor write (OV3660
SYSTEM CTROL0 0x3008 = 0x42, the whole register, never read-modify-write) and
sleeps on the ground when all of these hold (`camGndAwakeStep()`):

- `tkoff camgnd` on (NVS `camgnd`, default on), a sensor with a standby (OV3660,
  OV5640), the camera streaming (`CAMERA_FLOW_RUNNING`);
- autonomy in IDLE, `stabilizerOnGround()` (zero thrust, at rest since the last
  ground stop), outside the camera's take-off window;
- no frame snapshot pending, no bench hold (`stream N > 0`, a `camvz air` mode,
  `tkoff camwake <s>`);
- the wake hold run out: every PHOTO in IDLE, every ARM and every TAKEOFF packet
  holds the camera awake for `camgndms` (NVS, 5000..65000 ms, default 45000: the
  logged planning-PHOTO -> ARM gaps were 15-39 s);
- the camera started at least 30 s ago, and its sensor clock check (17.2) has
  decided: it needs ~10-15 s of awake frames, so a slow boot falls back to
  20 MHz before the camera first sleeps;
- not within 10 s of a failed standby write (counted in `gndFails`).

It wakes, within one 20 ms poll (`camGndAsleepStep()`), on a wake request
(PHOTO, ARM, TAKEOFF, `tkoff camwake`), autonomy leaving IDLE, the drone off
the ground, the switch turned off, or a bench hold. A ground start while
asleep is handed over to the take-off standby when that takes it (`camsb` on,
its phase open, not skipped by autonomy): the sensor stays asleep and the
take-off standby wakes it at the end of its window, as it always did, with no
wake and sleep in between; otherwise (camsb off, or an autonomous take-off,
which keeps `camNoStandby`) it wakes at once. The wake is the take-off
standby's (three tries 10 ms apart; a write that never lands falls to the 4 s
frame timeout and the camera's re-initialisation), then the flow fit restarts
on fresh frames. While asleep the drift hold gets a frameless update every
100 ms (its clocks run; what it learned is kept, as in the take-off standby).

The state reads `CAMERA_FLOW_STANDBY` (`cam=standby` in `status`, `live`, and
on the Mac) for both standbys; `status` `camGnd=1` and the `tkoff` line
(`asleep`, `gndCount`, `gndFails`, `gndMs`, `gndHoldLeftMs`, `gndAwakeMs`) tell
the ground standby apart. `cameraFlowInStandby()`, `sbCount` / `sbMs` and the
flight summary's `sbDid` keep their take-off meaning. No state machine change:
leaving IDLE wakes the camera, and autonomous take-offs keep it awake as before.

**The frame rate after a wake.** G4 (FLIGHT ARM: `camFps` over 10) read an
average that survived the sleep. Every wake (ground or take-off standby) now
hides it until two fresh frames have come (~0.15-0.25 s after the wake), so a
FLIGHT ARM needs a streaming camera. The rate itself is the processed one,
1 / the average frame interval, kept across the sleep (17.2): one lucky
interval after a wake cannot pass G4. An ARM that arrives while the camera sleeps is therefore
refused `GUARDS` `CAMERA`; the ARM packet itself woke the camera, and the Mac
(`espfly_auto.py`, both `arm` and `sortie`) sends the same ARM again (new seq)
0.5 s later, at most 6 times (3 s; event `arm_cam_wake` with the count; any
other guard set is answered as before). An older Mac tool shows `GUARDS CAMERA`
after a long idle: a `photo` first wakes the camera.

**Photos.** A PHOTO in IDLE while the camera sleeps on the ground is accepted
(OK, its id) and wakes the camera; its frames wait for 6 fresh frames after the
wake (auto-exposure settled within frames 4-6, 0.23-0.47 s, in the 13 logged
take-off standby wakes, none failed), at most 2.5 s after the PHOTO, after
which the 1 s frame timeout reports a camera that never woke (`AUTO_CAM_WAKE_*`,
`auto_link.c`). Such a photo arrives ~0.4-0.6 s later than from an awake
camera. The take-off standby, a stopped or a restarting camera still answer
`CAM_UNAVAIL` (the Mac's hint now names the take-off standby only).

**The thermal question.** Camera load changes the board and IMU temperature,
and the accelerometer drifts with temperature (e7cc199: the soak rule, IMU
temperature in `status`, the camera wake option). Could waking the camera
before a take-off restart or lengthen the heat-soak wait, or add in-flight
drift?

- The soak cannot restart or lengthen. G2 STILL (`auto_fsm.c`) is the time on
  the ground since thrust 0 (20 s + 1.5 x the last motor time, 27.5-90 s) plus
  |acc| - 1 g < 50 mg, gyro < 3 dps and tilt < 8 deg. Nothing in it reads the
  camera or a temperature, and a thermal drift of a few mg is far inside 50 mg.
  The logged soaks never refused an ARM.
- The in-flight drift. Nothing was measured on battery or in standby; the rest
  is inference. The camera is 0.15-0.42 W of ~0.6-0.95 W of ground
  dissipation; at the board's ~16-29 C/W that is ~2-10 C at the IMU at
  equilibrium, with a time constant of minutes (on USB the IMU warmed 31 -> 52 C
  in 8 min). A wake L s before the take-off adds, over a 60 s flight,
  dT x tempco x (e^-L/tau - e^-(L+60)/tau): an upper-bound range of about
  0.1-7 mg, for L of 3-40 s alike. Compare the observer's in-flight bias drift
  of -1.6..-5.7 mg on 1 m flights, the motors' own heating (0.55 mg per motor
  second, saturating at +35..+41 mg), and the ~15-20 C cooling step after
  unplugging USB that every first flight after setup already flies through.
- The design keeps the camera asleep through the whole soak and the idle, and
  wakes it at the planning PHOTO (15-39 s before the ARM in the logs) or at the
  ARM (at least ~3 s before lift-off: the ARM retry ~0.5 s, the pre-spin ~1 s,
  the spin 0.8 s, the ramp ~1 s). The camera height observer only uses frames
  from lift-off, and auto-exposure settles in 0.23-0.47 s, so frames, exposure,
  the hold and the observer are as healthy at lift-off as in today's
  camera-awake autonomous take-offs.
- Verdict: **go**, with the measurement built in: the flight summary now keeps
  the IMU temperature at the first thrust and at the end (`imuT0`, `imuT1`, 0.5 C)
  and how long the camera had been awake at the first thrust (`camAwakeDs`, 0.1 s;
  -1: no ground standby this boot), printed by `flights` (`FLIGHT_RTC_MAGIC`
  0xF1A6C0E5, the summaries of the previous build are cleared once). The bench
  step test (`espfly_diag.py drift --log-acc --cam-step awake:300,sleep:600,awake:600`)
  measures dImuT, d(acc z), tau and the tempco and prints the predicted in-flight
  drift for L 3 / 20 / 60 s and, if it is over 1 mg at 3 s, the `tkoff camlead`
  that brings it to 1 mg. `camlead` (NVS, 0..60000 ms, default 0) hides the frame
  rate in IDLE for that long after a ground wake, so a FLIGHT ARM is refused
  `CAMERA` until the lead has passed (take a photo, wait the lead, then ARM; the
  Mac's 3 s of ARM retries cover leads up to ~2.5 s). `tkoff camgnd off` turns
  the whole feature off.

**Numbers.** Sleep covers the soak and the idle minus ~22 s of awake time per
sortie (photo -> ARM) and 45 s after any extra photo. A typical charge (setup
~35 s and 3 gaps of ~77 s, 4 ARMs): ~180 s asleep x 70-110 mA = 12-20 A s, about
3.5-5.6 s of hover per charge (+4-7 %). A long-ground charge like 2026-10-13
night: ~420 s asleep = 29-46 A s, about 8-13 s (+10-15 %). Only 1-4 s per charge
if USB stays plugged during the waits (USB then powers the rail).

**Costs and risks.** An ARM after a long idle without a photo waits one retry
(~0.5-1 s, ~0.1-0.2 A s); a photo after a sleep comes ~0.4-0.6 s later. A failed
wake write falls back to the 4 s frame timeout and a camera re-initialisation
(SCCB) on the ground, counted in `gndFails`. Bench tools that need frames hold
the camera awake (`stream`, the `camvz air` modes, `tkoff camwake`). Built
against 126b309: `.iram0.text` unchanged (129759 B; all of it is flash code in
the camera, link and console tasks), internal RAM +72 B (`.dram0.data` +16,
`.dram0.bss` +56; `_heap_start` 0x3fcc0f18 -> 0x3fcc0f60), flash +6.8 KB of
code, the RTC flight summary +4 B a flight (`.rtc_noinit` 7020 -> 7084 B of the
8 KB RTC slow memory).

**USB.** `tkoff camgnd on|off`, `tkoff camgndms <5000..65000>`,
`tkoff camlead <0..60000>` (stored, motors stopped, IDLE), `tkoff camwake <s>`
(0..3600 s, RAM, any time; 0 ends the hold). The `OK tkoff` line prints
`camgnd camgndms camlead asleep gndCount gndFails gndMs gndHoldLeftMs gndAwakeMs`.
The console's output line grew from 1024 to 1536 bytes (PSRAM): `status` was
~950 characters before the camera fields.

**Tests.** `tools/tests/test_cam_gnd.c` (92 checks): every sleep condition
blocks the sleep on its own (the undecided clock check of 17.2 included), each wake reason wakes within one step, a ground
start while asleep is HANDOVER with camsb on and WAKE with it off, a failed
standby write backs off 10 s and is counted, the lead hides the rate until it
has passed (never with autonomy active, never with lead 0), repeated wakes
extend the hold (at most 1 h), the 32-bit ms clock wraps, the counters and the
awake time. `test_auto_link_host` (241): a PHOTO in IDLE during the ground
standby is OK, wakes the camera and is captured only after 6 fresh frames
(within 1.5 s); the take-off standby still answers `CAM_UNAVAIL`; a camera that
never wakes reports the frame missing after 2.5 s + 1 s; ARM and TAKEOFF wake
it, MOVE does not. `test_flight_summary` (40): the new fields and the size (136
B; the magic bump). `test_espfly_auto.py` `ArmCameraWakeTest`: one `GUARDS CAMERA`
then OK arms after one retry (new seq, >= 0.5 s, event logged); seven give up
after six retries with a hint; `STILL|CAMERA` is not retried; the sortie uses
the same retry; the `CAM_UNAVAIL` hint. `test_espfly_diag.py`
`CamGroundStandbyTest`: the `flights` fields, the drift `--log-acc` row
(`measure` and `status`), the step schedule, and the step report on synthetic
data (tempco, tau, the predicted drift, the camlead it asks for).

### 17.2 The camera at the flow loop's rate (`utils/src/cam_clock.c`, `camera_flow.c`)

**Why.** The OV3660 at QQVGA grayscale (esp32-camera's PLL x8, PCLK divider
8) runs a frame of 2300 x 783 pixel clocks with PCLK = XCLK / 2, so
fps = 2 x XCLK / (2300 x 783): 22.21 fps (45.0 ms) at the old 20 MHz. The flow
loop on core 1 takes ~61 +/- 3 ms per frame, so it processed only every other
frame or so: the logged intervals were exactly 45 or 90 ms (3,795 x 45, 4,742 x
90, 1 x 135), 14.73-14.90 fps processed; first-seen frame ages p50 117-121 ms,
max 195-198 ms (against `AUTO_VIEW_FRAME_AGE_MS` 200). One slow boot ran
P ~88 ms and 11.3 fps. The sensor clocked a third of its frames for nothing.

**What.** XCLK = 160 MHz / 12 = 13.33 MHz (`CAM_CLOCK_SLOW_HZ`): 14.81 fps
(67.5 ms), the rate the loop already processes, with regular frames. Not 16 MHz
(esp32-camera's `cam_hal.c` switches to its PSRAM DMA mode at exactly 16 MHz),
not 160 / 11 = 14.55 MHz (16.15 fps: its 61.9 ms period is shorter than the
~61 ms of processing, and the processed rate would collapse to ~10.7-11.8 fps).

| XCLK | fps | Period | Readout (768 of 783 lines) | Gyro window offset |
|---|---|---|---|---|
| 20 MHz (160 / 8, before) | 22.21 | 45.0 ms | 44.2 ms | 15 ms (the 2026-10-10 fit) |
| 14.55 MHz (160 / 11) | 16.15 | 61.9 ms | | (not used) |
| **13.33 MHz (160 / 12)** | **14.81** | **67.5 ms** | **66.2 ms** | **26 ms** (model) |
| 12.31 MHz (160 / 13) | 13.67 | 73.2 ms | | (not used) |

- NVS `camclk` (`tkoff camclk 20|13`, read at boot; absent = 13.33 MHz) picks
  the clock; `cameraStart()` passes it to `esp_camera_init()`.
- The rolling shutter. The frame's exposure is centred later on a slower
  clock: the gyro window offset is 15 ms + half the extra readout, rounded to the
  gyro's 1 ms (`camClockGyroOffsetUs()`): 15 ms at 20 MHz, 26 ms at 13.33 MHz,
  to be re-fitted on the bench (`espfly_diag.py flow-cal --offset-scan`; if the
  best offset is outside 22-30 ms, set the model to it and rebuild). The flow
  fit and the camera height observer use the same value:
  `cameraFlowGyroOffsetUs()` goes into the observer's config at
  `camVzGlueInit()`, and `camVzGlueSetGyroOffsetUs()` follows a change (the boot
  value, the fallback). `cam_vz_obs.c`'s default stays 15000, so the replays of
  the 22 fps logs (`test_cam_vz_replay_ih`, `test_cam_vz_shadow` csv / glue,
  `test_hold_replay`) are unchanged.
- The loop's own statistics, per processed frame (flash, `camClockTrack_t` in
  `cam_clock.c`, so the host test runs the same code): the busy time from
  `frameGet()`'s return to the end of the iteration, as an EMA (alpha 0.05) and
  a maximum; the frames processed at this clock (`clockFrames`, kept across
  standby wakes); the frame intervals over 1.5 sensor periods (`camSkips`; at
  20 MHz every 90 ms interval counts); the processed rate over windows of 150
  contiguous frames (capture times, restarted at every wake; a full window
  stays valid); the average processed interval (EMA 0.1, kept across standby
  wakes). `status` prints `camClk`, `camClkFallback`, `camClkChk`
  (`wait`: the check has not decided, `ok`, `slow`: 13.33 MHz and a loop too
  slow for it), `camProcAvgUs`, `camProcMaxUs`, `camSkips`, `camWinFps` and
  `camGyroOffUs`; the `tkoff` line `camclk` (the stored choice), `camClkNow` and
  `camClkFallback`.
- The fallback (`camClockFallback()`, host-tested): on the ground, awake,
  autonomy idle and outside a take-off window, at 13.33 MHz a processing
  average over 62000 us after 100 frames (`CAM_CLOCK_DECIDE_FRAMES`, ~7 s at
  14.8 fps, ~10 s at 9.9 fps), or a processed rate under 14.0 fps over a full
  150-frame window, switches the sensor to 20 MHz for the rest of the boot:
  standby (0x42), `sensor->set_xclk(sensor, LEDC_TIMER_2, 20)` (the PLL
  multiplies the new XCLK), the gyro offset 15000 for the flow and the
  observer, wake (0x02), and a new flow fit from fresh frames. Logged once
  (`camClkFallback=1`), never in flight. The camera does not sleep on the
  ground (17.1) until the check has decided (`camClockPending()`: 100 frames and
  a full window at 13.33 MHz, or the fallback ran), so the decision is made in
  the first ~10-15 s after the camera starts, on every boot.
- The published rate (`camFps`, G4: a FLIGHT ARM needs over 10) is the
  processed rate, 1 / the average frame interval (`camClockPublishFps()`). It
  is 0 until the clock check has decided (an ARM in the first ~10 s after the
  camera starts is refused `CAMERA`; G2's 20 s on the ground after a boot comes
  later anyway), until the second fresh frame after a wake, during the thermal
  lead, and while the loop is too slow for 13.33 MHz (the fallback's switch
  failed: every FLIGHT ARM of that boot is refused `CAMERA`; `tkoff camclk 20`
  and a reboot fly at 20 MHz). A camera re-initialisation after a stall keeps
  the decision.

**Numbers.** Frame age with regular frames: 66 ms readout + 61 ms processing =
~127 ms at the publish, ~195 ms just before the next frame (as today's maximum).
At 13.33 MHz the frames stay regular at 14.8 fps while P < ~64-65 ms (margin
~4-6 ms); from P ~68 ms they alternate 67.5 / 135 ms: 9.88 fps processed, frame
ages 271-293 ms before the next frame (the verifier's model of the driver's
queue, `batch/verifier-safety/src/camsim.py`, which reproduces the logged
45 / 90 ms split). The fallback acts on that boot ~10 s after the camera starts
(P 88 ms: at the 100th frame, before the first ground sleep); the host
timeline then gives 11.2 fps at 20 MHz, as the logged slow boot's 11.3.

**Fix after the verification (2026-10-14).** The first build ran the fallback
only on a full 300-frame window (30.4 s at 9.9 fps), while the camera slept
30.0 s after its start and every sleep and wake restarted the window: a slow
boot never completed one, and could have flown at 9.88 fps with 135 ms gaps.
G4 did not stop it either: the EMA of 1 / interval reads 10.9-11.3 fps on the
67.5 / 135 ms alternation (and 14.81 after a wake). Now the processing rule
needs 100 frames counted across wakes, the window is 150 frames, the camera
stays awake until the check has decided, and the published rate is the
processed one and is 0 before the decision and while the loop is too slow for
the clock. Auto-exposure: the band step of 98 rows is 5.6 ms at
20 MHz and 8.45 ms at 13.33 MHz (close to the 8.33 ms of 60 Hz flicker), and the
longest exposure the frame allows grows 1.5x in time. The saving: the sensor's
digital clock runs 33 % slower, an estimated 5-15 mA (the analyst's 3-10 mA),
not measured; no CPU saving (the processed rate is unchanged).

**Expected gain.** 5-15 mA of the ~3-3.5 A hover current (0.2-0.4 %): about
0.2-0.4 s of air per 86 s charge, plus a few mA s on the ground while awake
(most of that is gone with 17.1). The smallest of the four items; it ships
behind the NVS switch and the fallback. Side benefit: regular 67.5 ms frames
instead of 45 / 90 ms.

**Risks.** The cadence cliff: processing above the 67.5 ms period drops the
processed rate to 9.9-12.6 fps; the margin is only ~4-6 ms at P 61 +/- 3, and in
flight P is not watched for a fallback (on the ground only; a loop that slows
down later in the boot falls back at the next ground idle, or, armed, has its
rate hidden so the TAKEOFF is refused). A jittery loop (P 62 +/- 3 ms) skips
~3 % of the frames (14.4 fps, above the 14.0 window rule) and its frame ages
pass 200 ms more often than at 20 MHz (the camsim model: ~20 % against ~8 % of
frames): watched on the bench (`camSkips`, `flowDt`, `camAgeMs`). The sensor's VCO
at 8 x 13.33 = 106.7 MHz is below the 128 MHz the driver tested: the PLL lock is
a bench check. The 26 ms offset is a model value until re-fitted; the observer
and the hold use the same one. Longer exposures in dim light (more blur); the
band filter step changes. The saving is unmeasured and may be at the low end.
The runtime `set_xclk` of the fallback may give a corrupted frame or two on the
ground (skipped as stale). Built against the 17.1 build: `.iram0.text`
unchanged (flash code in the camera task; the observer already read its
offset from its config every frame), `.dram0.bss` +40 B (`_heap_start`
0x3fcc0f60 -> 0x3fcc0f88).

**Tests.** `tools/tests/test_cam_clock.c` (69 checks): the fps for the dividers
8 / 11 / 12 / 13 (22.21 / 16.15 / 14.81 / 13.67), the periods and readouts, the
offset 15000 / 26000, 16 MHz and 160 / 11 refused, the skip test, the EMA; the
fallback rule (the processing rule only after 100 frames, the rate rule only on
a full window; never with autonomy active, off the ground, in a take-off
window, asleep, from 20 MHz, or twice); the bookkeeping (a wake keeps the frame
count and the averages and restarts the window; the decision; the published
rate: 9.70-10.05 fps on the 67.5 / 135 ms alternation where the old EMA read
10.91-11.30, nothing while pending or too slow, two fresh frames after a wake;
a re-initialisation keeps the decision); and the integrated timeline with a C
port of the driver-queue model (fb_count 2, GRAB_LATEST) and the ground standby
(`cam_gnd.c`): the slow boot (P 88 ms) falls back at 10.2 s, sleeps at 30.1 s,
the PHOTO at 60 s wakes it and the ARM 20 s later sees 20 MHz at 11.21 fps;
with a failed switch the ARM sees 0 fps (refused); an ARM 8 s into a slow or a
normal boot sees 0 (pending); an ARM while asleep sees 0 (the Mac resends it);
a normal boot (P 61 ms) keeps 13.33 MHz, sleeps at 30 s and arms at 14.81 fps;
for every P of 55-100 ms either 13.33 MHz at >= 14 fps or the fallback before
the first sleep. `test_cam_gnd` (92): the undecided clock check blocks the
sleep.
`test_cam_vz_glue_host` (29): the observer gets the camera's offset at init (the
rest of its config the default), a change applies (kept through the
observer's reset), an offset over 100 ms is ignored. `test_espfly_diag.py`
`FlowOffsetScanTest`: on synthetic 100 Hz recorder rows whose frames were made
with a 26 ms (15 ms) offset the scan picks 24-28 ms (13-17 ms), R^2 > 0.95,
dx vs gz ~102.4 px/rad. `test_interactive_sim`'s camera model (frames every
60-68 ms, ~120 ms late) already matches regular 14.8 fps frames.

### 17.3 The climb stays at 0.40 m/s (no-go)

**Asked.** `vClimbMax` 0.40 -> ~0.60 m/s (the climb timeouts scale with it),
possibly battery-aware, as the end-of-battery rules allow.

**Not built.** `vClimbMax` stays 0.40 m/s and `aClimb` 1.0 m/s^2. The
project's closed-loop sims (`test_interactive_sim` `testDrainVzK` /
`testWeakTakeoff` harness, the real `auto_fsm`, `vert_ctrl`, `cam_vz_obs` and
`camera_hold`; the designer's `batch/designer/src/climb_sweep.c`, run with
0.40 / 0.50 / 0.60) with only the climb changed: a faster climb makes Khat
absorb more acceleration and drag (about +2 % of kPrior per 0.1 m/s) and reach
its 1.12 x kPrior clamp (`K_RANGE`) in the take-off climb more often and higher
up, and a take-off `K_RANGE` is a fast LAND that costs the sortie.

| `vClimbMax` (vzK 160400, 108 sorties of the drain grid) | Take-off `K_RANGE` | ... with K offset <= 0 | From up to | Touchdown up to | Also | Checks failed |
|---|---|---|---|---|---|---|
| 0.40 (today) | 24 | 0 | 0.26 m | 0.69 m/s | 0 capped | 0 |
| 0.50 | 31 | 11 | 0.53 m | 1.04 m/s | | 6 |
| 0.60 | 15 | 6 | 0.86 m | 1.71 m/s | 2 `CLIMB_CAPPED` (C 150, 1.2 m, 1.22-1.23 m/s) | 10 |

A battery-aware rate does not help: at 0.60 m/s the take-off `K_RANGE` comes
at every rest voltage of 3.75-4.15 V (3-8 per cell of 18, up to 4 for a drone on
the pooled fit, touchdowns 0.76-1.26 m/s), because in the sim plant K rises as
the loaded voltage falls, so the mismatch is everywhere, not worst on a full
battery. A climb acceleration feedforward (the designer's prototype, + Khat /
Vf x aClimb / g while the ramp binds) brings the count back near 0.40's, but
those left start at 0.45-0.79 m and touch down at up to 1.43 m/s, still
failing the existing checks, and it is a controller change outside the
request. The climb would have saved 0 s at 0.3 m, 1.76 -> 1.42 s at 0.6 m and
3.25 -> 2.38 s at 1.2 m (0-0.9 s per sortie, 1-3 s per charge); one extra
take-off `K_RANGE` per 30-50 sorties cancels that, and the sims show many more.
Revisiting it needs a user decision on how a `K_RANGE` in CLIMB ends (for
example a slow LAND from the climb average instead of the fast one), and must
then pass `testDrainVzK` (take-off `K_RANGE` touchdowns <= 0.80 m/s, none for a
drone on the pooled fit at vzK 160400) and `testWeakTakeoff` unchanged.

### 17.4 A faster LAND from height (`vert_ctrl.c`)

**Why.** The LAND is about 8 % of a sortie's energy at ~0.9-1.0 x the hover
power, and from 1.0-1.2 m it takes 4.5-5.5 s at -0.30 m/s. It cannot brake: its
thrust never exceeds the ceiling of 0.98 x T_ref, so a faster vref alone only
reaches the floor faster (the mapper's probe: 0.30 -> 0.38 m/s). The real LANDs
from ~1 m sit at that ceiling and accelerate from -0.27 m/s (0.7 s) to
-0.45..-0.58 m/s at contact (night #3, tiger #1, pwmA1 #7); others settle at
-0.22..-0.28 m/s (propswap, mission60, pwmA1 #5). The sim's damping of 0.07 g
per m/s fits the slow ones, ~0.02 the fast ones, so both are simulated.

**What.** A fast descent above 0.85 m, back to -0.30 m/s by 0.60 m, with a
small, time-bounded brake above T_ref that only takes back the speed the fast
part added. New `vzConfig_t` fields (defaults, `vzConfigValid` ranges):
`landVfast` -0.50 m/s (-0.60..`landVref`; equal to `landVref` turns it off),
`landFastH` 0.60 m (0.30..1.00), `landFastBand` 0.25 m (0.05..0.50),
`landFastMinH` 1.00 m (`landFastH` + `landFastBand`..1.50), `landBrakeTop` 1.03
(1.00..1.05), `landBrakeH` 0.40 m (`landSlowH`..`landFastH`), `landBrakeMaxS`
0.5 s (0..1.0); `vzState_t` `landFast`, `landFastOk`, `landFastFlown`,
`brakeS`; `vzInput_t` `landFastOk`.

- The state machine says whether this LAND may be fast: `vzInput_t.landFastOk`,
  set by the glue on every step from the reason of the latest transition
  (`autoFsmLandFastReason()`, `auto_fsm.h`, a 32-bit mask): the Mac's LAND,
  `AIR_TIME`, `HOVER_TIMEOUT`, `SORTIE_DONE`. Never `VBAT_DIP` (a 1 kHz dip
  under 2.55 V that leaves the 0.5 s-filtered Vf above 3.15 V: good flights dip
  to 2.62 V), `VBAT_LOW`, `BATT_LOW`, `LOOP_GAP`, a lost link, session or app,
  `NOT_STABLE`, `CAPTURE_FAIL`, a motion's or the camera's guard, a contact.
  Before the verification every state machine LAND reached the controller as
  `CMD_LAND`, so a `VBAT_DIP` LAND from 1 m could brake at up to 1.03 x T_ref
  right after a brown-out-level dip.
- `startLand()` (IRAM): for a slow LAND (`landSlow`) with the camera estimate in
  use at a reference height (hTarget + hOff) of 1.00 m or more it calls
  `landFastStart()` (flash, once per LAND), which sets `landFast` for `CMD_LAND`
  only, with `landFastOk`, never blind, with T_ref <= `VZ_LAND_FAST_TREF_MAX`
  (53400 = `AUTO_MV_REFUSE_T`, a static assert in `auto_fsm.h`) and Vf >=
  `VZ_LAND_FAST_VMIN` (3.15 V loaded). So `K_RANGE`, `EST_RANGE`,
  `HOLD_CAPPED`, `HOLD_NO_CAM`, `CMD_MISSING`, `HOLD_TIMEOUT`, a LAND from
  CLIMB, blind (`COLLISION`), every state machine reason off the allow list and
  the end of the battery (`BATT_LOW`: T_hold over 57000 or a battery under
  3.00 V) are never fast and bit-identical to before.
- The LAND step: vref as before, plus `(landVfast - landVref) x blend`, blend =
  `clamp((h - landFastH) / landFastBand, 0, 1)`, on steps with the camera
  estimate (a dropout reverts to -0.30 at once). The brake: once the fast
  reference was flown (blend >= 0.5 on a camera step: `landFastFlown`), while
  the estimate reads between `landBrakeH` 0.40 m and `landFastH` +
  `landFastBand` 0.85 m (blend < 1: the slow-down band, never the cruise
  above), for at most `landBrakeMaxS` in all and only in the flat part, the
  thrust may reach `landBrakeTop` x T_ref instead of the ceiling; it does not
  need the camera (0.5 s of propagated velocity). The final clamp is
  `thrust <= min(max(ceiling, brakeTop), 58000)`; the ceiling itself stays the
  monotone 0.98 one (the stall rule, `landFlatEff`, the timeouts and the FSM's
  backstops, all sized for -0.30 m/s, unchanged and conservative).
- Brake thrust <= 1.03 x 53400 = 55000 (the hardest motor ~59.9k at p99 with the
  battery-forward balance).

**Why the band, after the verification (2026-10-14).** The first build
(-0.50 m/s above 0.65 m, back to -0.30 by 0.50 m, from 0.70 m, the brake
anywhere above 0.40 m) made the mean touchdown harder: on the low-drag plant,
holding -0.50 m/s needs ~0.99 x T_ref, above the 0.98 ceiling, so the 0.5 s
budget was spent in the cruise above 0.65 m and the drone reached 0.50 m at
-0.46 m/s (designed -0.30). Per (h, damping, e) cell of 60 runs (K -3 / 0 / +3 %
x 20 seeds, e -0.2 / 0 / +0.2 / +0.4 m) 22 of 36 cell means were harder (worst
+0.057 m/s, 17 more than 2 SE), 17 shares over 0.40 m/s higher. Variants were
swept on the same cells (`batch/fixer/src/landx.c`, 3 seed sets of 180 runs per
cell, with a before-against-before control on other seeds: no cell mean more
than 2 SE apart, the shares up to 14 of 180 apart): the brake only in the
slow-down band and only after the fast part (`mode 4`), with the band moved up
to 0.85-0.60 m and the fast LAND from 1.00 m, was the best (worst cell mean
+0.010..+0.014 m/s in all three seed sets; the remaining cells are within the
control's noise or better). Braking without flying fast first (the LAND before
plus the brake alone) made the mid-drag plant's mean +0.04 m/s harder: the
sim's touchdown is bimodal there (the ground effect either catches the drone,
which floats ~4 cm and then drops at ~0.4 m/s once the stall rule lowers the
ceiling to `landFloor`, or the drone touches at its approach speed), and a
slower approach floats more often. The fast LAND now reaches 0.50 m at about
the LAND before's speed there and 0.20 m at the same speed (30 runs per row,
e 0, true speed in m/s):

| From, damping | At 0.85 / 0.60 / 0.50 / 0.20 m, fast | ... before | LAND fast / before |
|---|---|---|---|
| 1.0 m, 0.07 | 0.32 / 0.32 / 0.29 / 0.28 | 0.22 / 0.26 / 0.27 / 0.27 | 3.96 / 4.49 s |
| 1.0 m, 0.035 | 0.31 / 0.36 / 0.34 / 0.39 | 0.22 / 0.32 / 0.34 / 0.38 | 3.30 / 3.68 s |
| 1.0 m, 0.02 | 0.30 / 0.37 / 0.36 / 0.44 | 0.21 / 0.34 / 0.37 / 0.44 | 2.66 / 2.97 s |
| 1.2 m, 0.07 | 0.41 / 0.33 / 0.30 / 0.27 | 0.26 / 0.26 / 0.26 / 0.26 | 4.48 / 5.33 s |
| 1.2 m, 0.035 | 0.44 / 0.38 / 0.37 / 0.40 | 0.30 / 0.34 / 0.35 / 0.38 | 3.61 / 4.32 s |
| 1.2 m, 0.02 | 0.46 / 0.40 / 0.41 / 0.46 | 0.32 / 0.38 / 0.40 / 0.46 | 2.92 / 3.46 s |

**Sims.** `test_vert_ctrl` `testFastLand` (the controller on its plant, 1 kHz,
kPrior 160400, 3.85 V at rest; the LAND 3 s after HOLD entry; the estimate
reading e high from the LAND on; 60 runs per cell: K -3 / 0 / +3 % x 20 seeds),
touchdown mean fast / before (m/s), runs over 0.40 m/s fast / before,
LAND-to-DONE mean fast / before:

| Cell | Damping 0.07 (default plant) | 0.035 | 0.02 (the accelerating real LANDs) |
|---|---|---|---|
| 1.0 m, e 0 | 0.259 / 0.315, 0 / 2, 4.10 / 4.64 s | 0.364 / 0.362, 30 / 27, 3.42 / 3.83 s | 0.225 / 0.244, 11 / 13, 2.70 / 3.03 s |
| 1.2 m, e 0 | 0.273 / 0.322, 0 / 4, 4.41 / 5.27 s | 0.335 / 0.364, 35 / 34, 3.63 / 4.32 s | 0.257 / 0.271, 3 / 6, 2.83 / 3.33 s |
| 1.0 m, e +0.2 | 0.265 / 0.280, 0 / 0, 3.73 / 4.57 s | 0.371 / 0.357, 9 / 7, 3.33 / 3.91 s | 0.408 / 0.405, 48 / 41, 3.13 / 3.61 s |
| 1.2 m, e +0.2 | 0.269 / 0.311, 0 / 2, 4.22 / 5.32 s | 0.373 / 0.374, 13 / 21, 3.72 / 4.41 s | 0.428 / 0.434, 53 / 51, 3.52 / 3.96 s |
| 1.0 m, e +0.4 | 0.279 / 0.285, 0 / 0, 3.50 / 4.56 s | 0.360 / 0.358, 2 / 9, 3.25 / 3.96 s | 0.409 / 0.430, 37 / 48, 3.15 / 3.70 s |
| 1.2 m, e +0.4 | 0.276 / 0.303, 0 / 2, 3.98 / 5.30 s | 0.359 / 0.368, 1 / 16, 3.69 / 4.49 s | 0.409 / 0.453, 37 / 57, 3.54 / 4.14 s |
| 1.0 m, e -0.2 | 0.282 / 0.304, 0 / 1, 4.44 / 4.58 s | 0.358 / 0.376, 33 / 33, 3.65 / 3.90 s | 0.250 / 0.263, 10 / 15, 2.81 / 3.07 s |
| 1.2 m, e -0.2 | 0.265 / 0.341, 0 / 4, 4.85 / 5.38 s | 0.365 / 0.361, 39 / 31, 3.91 / 4.35 s | 0.256 / 0.275, 4 / 9, 2.98 / 3.34 s |

From 0.8 m the LAND is the one before (bit-identical). The worst cell mean is
+0.014 m/s (1.0 m, e +0.2, damping 0.035: the float-and-drop of both LANDs,
0.36-0.37 m/s, the fast one's floor ceiling a hair lower at the drop); the
estimate reading high (e +0.2 / +0.4) is now softer or equal everywhere else
(it was +0.03..+0.045 m/s harder). The interactive hover's own plant
(`test_interactive_sim`, the real state machine, damping 0.07): the mission's
LAND from ~1.2 m 5.37 -> 4.55 s mean (30/30 fast; 4.42 in the first build), a
LAND after a move at 1.0 m 4.37 -> 3.85 s, 0.20 + 1.00 m moved up 5.59 -> 4.69 s,
touchdown max 0.29 m/s. `test_vert_ctrl`'s fixed sorties (before -> first build
-> now): 1.2 m camera LAND max 5.75 -> 4.94 -> 5.16 s, impact 0.41 -> 0.30 ->
0.31 m/s; with a camera offset +/-0.01 m/s 6.39 -> 5.57 -> 5.63 s, 0.61 -> 0.40 ->
0.40; 0.9 + 0.3 m moved up 5.78 -> 5.06 -> 5.08 s, 0.41 -> 0.31 -> 0.32; 0.2 +
1.0 m 5.85 -> 4.88 -> 4.91 s, 0.31 -> 0.33 -> 0.33; 1.0 m after a 60 s HOLD 10.31
-> 10.09 -> 10.15 s, 0.47 -> 0.39 -> 0.37. The camera lost 0.3-1.5 s into a fast
LAND (360 runs, 60 per height and plant): touchdown max 0.48 m/s, the worst mean
+0.033 m/s (the mid-drag plant; its share over 0.40 within 2 SE). A `VBAT_DIP`
LAND from 1.0 m with the camera (`test_interactive_sim`): not fast, LAND thrust
<= 0.98 x T_ref (with every reason allowed it was fast and braked to 0.9976).
The drain grid (108 sorties): every `BATT_LOW` LAND is not fast, touchdowns
unchanged (max 0.71 m/s).

**The contact case.** A contact the state machine did not see (its estimate
corrupted but still "healthy"; the real FSM lands such a contact blind, LAND
`COLLISION`, never fast) lifts the drone up to 0.083 m at 1.0 m and 0.071 m at
1.2 m in the LAND (first build 0.095 / 0.089; 0.008 m without the brake):
`testContactLand` (a) checks <= 0.10 m.

**Expected gain.** LAND-to-DONE 0.3-0.5 s shorter from 1.0 m and 0.5-0.9 s from
1.2 m on the three plants (the first build: 0.3-0.7 and 0.7-1.0 s); nothing
below 1.00 m. At ~0.9-1.0 x the hover power, about 1-2 s of air per charge when
sorties fly at 1.0-1.2 m (3-4 sorties); the logs suggest the low end.

**Risks.** The LAND's "never above T_ref" becomes "<= 1.03 x T_ref for <= 0.5 s
while the estimate reads 0.40-0.85 m, after the fast part": a corrupted
velocity estimate that still reads healthy can lift the drone ~0.08 m (above).
The sim's touchdown on the mid-drag plant is bimodal (a float and a drop), and
the fast LAND changes the odds of a float by a few per cent either way: within
+0.015 m/s per cell. A camera dropout in the fast part: up to +0.03 m/s on
average. The fast descent raises the vertical image motion (~0.45 v / D of
expansion) toward the observer's 0.3 /s gate at near scenes (D < ~0.75 m), which
reverts to the normal vref (the safe direction). The sims' default drag
overstates the gain.

**Cost.** Against the first build: `vzStep` +44 B of IRAM (the band, the flown
flag, the per-step `landFastOk`), the glue's `vzRun` +32 B (the allow list, a
32-bit mask); `.iram0.text` 129923 -> 129999 B, still inside the same 256 B step
(`_iram_text_end` 0x40394000). Against 126b309 +240 B, one 256 B step (the
internal heap starts 0x3fcc0f18 -> 0x3fcc10d0). The vert_ctrl identity hash
(heights 0.20-0.60 m) is unchanged.

**Tests.** `test_vert_ctrl` (357 checks): the defaults and the 7 fields'
limits; `testFastLand`: the gates (1.00 and 1.20 m fast; 0.99 m, 0.70 m,
`CMD_MISSING`, blind, `HOLD_TIMEOUT`, `EST_RANGE`, a LAND in CLIMB at 0.8 m,
`HOLD_NO_CAM`, a reason off the allow list at 1.00 and 1.20 m (`VBAT_DIP`,
`LOOP_GAP`), T_ref over 53400 at Vf 3.2 V and Vf under 3.15 V bit-identical to
the LAND before); the Monte Carlo above, per (h, damping, e) cell: the mean
touchdown <= the LAND before's + 0.015 m/s, the share over 0.40 m/s not higher
by more than 2 SE + 1 run, the hardest <= the LAND before's + 0.03 m/s, every
touchdown <= 0.55 m/s, LAND-to-DONE faster on average from 1.0-1.2 m, every LAND
from 0.8 m bit-identical; thrust <= 1.03 x T_ref, above the 0.98 ceiling only
while the estimate reads 0.40-0.85 m and for <= 0.5 s in all; no climb after the
LAND start (e 0, <= 0.02 m); the camera lost 0.3-1.5 s into the LAND (every
touchdown <= 0.55 m/s, the mean per height and plant <= + 0.04 m/s, the share
within 2 SE + 1); `testContactLand` (a) true climb <= 0.10 m. The test runners
that model the Mac's LAND set `landFastOk`. `test_auto_fsm` (1432): the allow
list is exactly the four reasons. `test_interactive_sim` (69): `testDrain`'s LAND
thrust check is "<= 0.98 x T_ref, or <= 1.03 x T_ref in a fast LAND (T_ref <=
53400)" and no `BATT_LOW` LAND is fast; a `VBAT_DIP` LAND from 1.0 m is not fast
and stays <= 0.98 x T_ref; the `AIR_TIME` LAND may be; the harness sets
`landFastOk` as the glue does. `test_auto_sortie` (the same wiring),
`test_autonomy_glue_host` and the identity hash pass.

### 17.5 Faster moves (`auto_motion.h`)

**What.** `AUTO_MV_V_H_MAX` 0.20 -> 0.30 m/s (the user approved 0.25-0.30; 0.30
passed the sims and the camera analysis), inside `#ifndef` so the replay of
the logs flown at 0.20 pins the old value. The component toward the scene ahead
(a move's forward part, `fwdM > 0`) stays at 0.20 m/s (`AUTO_MV_V_FWD_MAX`,
`vmax <= 0.20 x len / fwdM`): a person approach (`approach --person`) is a
series of forward MOVEs whose `W_FWD` cap uses the drone's scene D, mostly the
background behind the person (`espfly_auto.py` ~:3200), and ends >= 1.9 m from
it by that D, so the image-rate cap does not bind there; at 0.30 the approach
steps would have flown at 0.23-0.30 m/s and the STOP braking distance toward the
person would have grown from 0.13 to 0.18-0.30 m (the verifier's
`approach_speed.c`). With the forward cap the approaches, their steps and
their STOP distance are exactly as before. Lateral and backward moves (and the
sideways part of a diagonal one) fly up to 0.30 m/s. Unchanged: `AUTO_MV_A_H` 0.10 m/s^2,
`AUTO_MV_W_LAT_MAX` 0.12 /s and `AUTO_MV_W_FWD_MAX` 0.15 /s (they bound the
camera's image rates whatever the top speed), `AUTO_MV_BRAKE_ACC` 0.15, the
progress margins (0.15 m + 30 %), the settle (3 s, 10 % + 4 cm, lateral >= 0.25 /
D), `TOO_WIDE` (0.5 D), the re-view bob, `AUTO_MV_LAND_TIME_MS` 8500 and the air
check; the planning in `auto_motion.c` gains the one forward cap (0 B IRAM,
`AUTO_COLD`). The Mac (`espfly_auto.py`) mirrors the profile (`MOVE_V_H_MAX`,
`MOVE_V_FWD_MAX`, `MOVE_A_H`, `MOVE_W_LAT_MAX`, `MOVE_W_FWD_MAX`, checked against
the header); its
`move_timeout()` keeps its 0.20 m/s basis on purpose (10 s + 2 x horiz / v: longer
than the drone needs); `travelled_m` and the approach logic are distance-based,
unchanged.

**Where it matters.** The top speed binds only for sideways (or backward) moves
with the lateral D over V / 0.12 (2.5 m at 0.30). Forward moves, and with them
the approaches to a person, fly at most 0.20 m/s toward the scene as before;
near-scene moves are as before. The lateral image motion stays <= 0.12 x 102.4
px/rad x 0.0675 s = 0.83 px/frame with the regular 14.8 fps frames of 17.2 (1.1
px at the old 90 ms frames) against the observer's 1.5 px gate, whatever the top
speed.

**Sims.** `test_interactive_sim` `testFarMoves` (the real state machine, motion
module, hold and vertical controller; D ~3.7 m; 10 seeds each, the same seeds
also at 0.20 through the host-only `AUTO_MV_HOST_VH_OVERRIDE`), MOVE to its end:

| Move (D) | 0.20 m/s | 0.30 m/s | Error mean / max | Lean max |
|---|---|---|---|---|
| right 1.00 m (3.67 m) | 7.34 s | 6.86 s | 9 / 14 % | 1.2 deg |
| left 1.50 m (3.77 m) | 13.37 s | 11.09 s | 10 / 16 % | 1.3 deg |
| forward 1.00 m (3.81 m) | 7.23 s | 7.23 s (0.20 toward the scene) | 9 / 19 % | 1.0 deg |
| forward 1.50 m (3.72 m) | 9.89 s | 9.89 s (0.20 toward the scene) | 8 / 13 % | 1.0 deg |
| diagonal 0.70 / 0.70 m (3.92 m) | 7.00 s | 6.40 s (0.28 m/s: 0.20 forward) | 13 / 20 % | 0.9 deg |

Every move ends DONE and is never slower than the same seed at 0.20 (the
forward rows are the 0.20 runs; the first build flew them in 6.76 and 8.17 s). The
designer's sweep (`batch/designer/src/move_sweep.c`, 15 cells, D 2.7-4.6 m)
found the same outcomes (DONE / SHORT / PROGRESS) at 0.20 and 0.30 in every
cell. A STOP 2.5 s into a 1.0 m move: 0.55 m done at the STOP, 0.02 m more after
it (the check is now <= 0.10 m). The brake distance at 0.30 m/s is 0.30 m (0.13 at
0.20) plus ~0.04 m of camera lag, on far-scene moves only. The logged forward
moves at 0.20 were `W_FWD`-capped at D 2.4-2.5 m (cruise 71-78 % of the
reference, 0 of 4 DONE: PROGRESS / kill), so 0.30 changes them by <= 15 %.

**Expected gain.** Nothing for moves of 0.9 m or less, near scenes (lateral D
under 2.5 m) or toward the scene; 0.5-2.3 s per 1.0-1.5 m sideways or backward
move at D >= 2.7 m (a diagonal one ~0.6 s); 0-2 s per charge depending on how
many long far-scene sideways moves are flown.
`AUTO_MV_A_H` 0.15 (a 0.9 deg feedforward lean) would save another 0.5-1.6 s
per 1-1.5 m move and passes the move checks, but it turned one end-of-battery
sortie of `testWeakTakeoff` into `HOLD_CAPPED` (vzK 158000, 3.85 V, C 250,
60 cm, 0.71 m/s; 0 of 576 in a 4-seed drain sweep) and was not asked for: a
separate decision for the user, not built.

**Risks.** The real hold lags more than the sim's (cruise 71-78 % of the
reference at 0.20): at 0.30 the absolute lag grows ~1.5x, so more `PROGRESS`
ends are possible on real far-scene sideways moves (a stop, not unsafe; the move
is re-issued). The STOP braking distance doubles to ~0.34 m on far-scene sideways
and backward moves (toward the scene it stays 0.13 m). A
frozen view ends `PROGRESS` a little later in distance (sim: 0.79 -> 0.92 m
before the LAND for a 1.0 m frozen lateral move, 0.98 -> 1.07 m back; lean up
to 1.8 deg). Replays of old logs must pin 0.20.

**Tests.** `test_auto_motion` (197): the STOP brake lasts the planned peak
speed / `AUTO_MV_BRAKE_ACC` (0.212 m/s for (100, 100) at D 2.0, the forward
image-rate cap; it was `V_H_MAX`); `testTopSpeeds`: at D 5.0 m right, left, back
and right-back 0.30 m/s, forward 1.00 / 1.50 m 0.20 m/s, right-forward 0.283 m/s
(its forward component 0.20); the lean and rate checks pass. `test_motion_replay` is built
with `-DAUTO_MV_V_H_MAX=0.20f` (a static assert; at 0.30 its reference check fails
0.0112 > 0.0097). `test_interactive_sim` (69): `testFarMoves` above, the STOP
drift <= 0.10 m. `test_auto_fsm` passes. `test_espfly_auto.py`:
the profile constants against `auto_motion.h` (and its `#ifndef`), and
`move_timeout()` against the drone's profile at 0.30 (0.20 toward the scene) +
the 3 s settle + the 4 s re-view bob + 2 s for six moves (`MoveSpeedTest`,
forward 1.5 m at D 5.0 = 9.5 s, back 1.5 m 8.0 s).

### 17.6 Cost, verification and what to check

**Built against 126b309** (int-sdkconfig, ESP-IDF 5.0.7; with the fixes after
the verification): `.iram0.text` 129759 -> 129999 B (+240: the fast LAND in
`vzStep` / `startLand` and the allow list in the glue's `vzRun`; the camera and
move changes are flash code), `_iram_text_end` 0x40393f00 -> 0x40394000 (one
256 B step); `.dram0.data` 31464 -> 31480, `.dram0.bss` 87344 -> 87512;
`_heap_start` 0x3fcc0f18 -> 0x3fcc10d0 (the internal heap 440 B smaller: the
256 B IRAM step and 184 B of RAM); `.flash.text` 726719 -> 733227 (+6.4 KB),
`.flash.rodata` 184992 -> 185856 (+0.8 KB); `.rtc_noinit` 7020 -> 7084 B (the
flight summaries). The A2b builds read 21.3-22.8 KB of internal heap on the
bench (`heapInt`): expect ~20.9-22.4 KB, above the 20 KB this change must keep
(G9 refuses an ARM below 12 KB); bench checks G1 and G2 read `heapInt`.

**Bench checks** (autonomy-bench.md G1-G5) before flying: the ground standby
(cycles, wake times, the Mac's ARM retry, the thermal step test), the camera
clock (rate, regularity, processing time, the offset scan, the loop rate), the
LAND (DRY, BENCH-sim, then the first real 100 cm LANDs with a stop rule), the
moves (DRY, then far-scene moves with a stop rule).

## 18. Longer flights and the palm catch (user decisions 2026-10-14, ~01:40-01:55)

**2026-10-15 (section 22):** the 120 s air clock (18.1) and the palm catch (18.3) are gone; the wait (18.2) is
information only (no HOVER_TIMEOUT). The text below is the history of those builds.

Three decisions, one commit each on a2b-int after b176c26:

| Item | What changes | Section |
|---|---|---|
| A. The air clock 120 s | `AUTO_AIR_HARD_MS` 80000 -> 120000, the controller's HOLD backstop with it (119 s), the LAND's drift allowance 1.0 -> 2.0 m; the Mac's `AIR_HARD_MS` | 18.1 |
| B. A hover wait of up to 120 s | the wait is u32 ms in the firmware, u24 on the wire (TAKEOFF byte 7), u32 in NVS with a read of the old u16 item; the Mac's `--wait-s 1..120` | 18.2 |
| C. The palm catch | a palm under the hovering drone: LAND `PALM`, then a support probe stops the motors; the sortie ends in a normal IDLE | 18.3 |

Not changed: gains and tuning, the battery rules of section 16, the motion
height floor (`AUTO_MV_H_*`), the heat soak, the 60 s flight recorder.

### 18.1 The air clock 120 s (`auto_fsm.h`, `vert_ctrl.h`)

**What changed.**

- `AUTO_AIR_HARD_MS` 80000 -> 120000. A static assert ties it to the
  controller's limit: `AUTO_AIR_HARD_MS == VZ_HOLD_MAX_LIMIT_S x 1000`.
- `VZ_HOLD_MAX_LIMIT_S` 80 -> 120, so `holdMaxS` (`VZ_HOLD_MAX_LIMIT_S - 1`)
  is 119 s. The two must move together: `holdMaxS` 119 under the old 80 s
  limit fails `vzConfigValid()` (BAD_CONFIG -> CUT -> KILLED), and without the
  raise every hover would land `HOLD_TIMEOUT` at ~81-83 s of air. HOLD starts
  more than 1 s after the first RAMP step, so the air clock (`AIR_TIME`, on the
  fast LAND's allow list) always lands first. The controller's `tPhase` sums
  1 ms steps in float and reads ~0.05 s late at 119 s (the safe side).
- `VZ_LAND_DRIFT_MAX_M` 1.00 -> 2.00 m (static assert: `VZ_LAND_DRIFT_MPS` x
  118 s <= 2.0 m). A LAND after a HOLD longer than 18 s keeps its flat part
  and its timeout longer by min(2.0 m, 0.017 m/s x the HOLD) at 0.20 m/s:

  | HOLD | extra flat part and timeout | from 1.20 m: flat / timeout / FSM backstop | from 1.05 m |
  |---|---|---|---|
  | 58.8 s | +5.0 s (as before) | 9.0 / 13.0 / 13.5 s | 8.5 / 12.5 / 13.0 s |
  | 80 s | +6.8 s (was capped at 5.0) | 10.8 / 14.8 / 15.3 s | 10.3 / 14.3 / 14.8 s |
  | >= 117.6 s | +10.0 s | 14.0 / 18.0 / 18.5 s | 13.5 / 17.5 / 18.0 s |

  `landTimes()` (vert_ctrl.c) and the FSM's mirror `landExtraMs()` read the
  constant; no code changed. Why: an estimate reading low (the drone higher
  than it thinks) by the worst logged 0.011 m/s hides 1.3 m after 118 s,
  beyond the old 1.0 m.
- The Mac: `AIR_HARD_MS = 120000` (the test reads it against the `#define`);
  at the limit (`airLeftDs` 0, `airMs` saturated at 65535) `air_s` is 120.0,
  not 65.5. Since the verification of 2c69fc5 `air_s` is `airMs` while that
  is exact (below its 65.5 s saturation) and `AIR_HARD_MS` - `airLeftDs` only
  past it: a drone and a Mac of different versions (an 80 s drone, a 120 s
  Mac or the reverse) would otherwise shift `air_s` by 40 s with no sign, and
  `air_s` drives the Mac's height-floor (and, below, ceiling) rules. Unsaturated,
  `airMs` + 100 x `airLeftDs` is the drone's own limit; more than 0.2 s from
  `AIR_HARD_MS` is flagged (`airClockMismatch`, event `air_clock_mismatch`)
  and the Mac refuses MOVE / YAW for the rest of the sortie (STOP, photos and
  LAND still work).
- AUTO_STATUS `airLeftDs` is at most 1200 (u16; byte layout unchanged).

**Unchanged on purpose.** The motion height floor 0.25 m + 0.010 m/s x the
air time (`AUTO_MV_H_DRIFT_MPS`): 1.05 m at 80 s, 1.45 m at 120 s, so the Mac
refuses every MOVE / YAW from 80 s at 1.05 m (95 s at 1.20 m), and the
drone refuses moves down there; the extra time is hover, photos, STOP and
LAND only (the safe direction). `hMax` / `EST_RANGE`. The heat soak. The end
of battery (`BATT_LOW`, `VBAT_LOW`, `VBAT_DIP`, the MOVE refusal at 53400).
The 60 s flight recorder: a 120 s flight keeps only its last 60 s (raising
`FLIGHT_RECORDER_SAMPLES_PSRAM` 6000 -> 12000 would keep all of it in PSRAM,
not internal RAM: a later decision). The battery measured ~86 s of air per
charge, so `BATT_LOW` usually ends a flight before 120 s.

**What a longer flight does (no firmware height guard is possible without a
height sensor; the Mac's ceiling rule below bounds the time above 1.05 m).** The controller's own estimate (`vzH`, not the shadow observer's
`cvH`) at touchdown after the six logged hovers longer than 20 s: RMS
0.020 m/s; worst reading high +0.031 m/s, worst reading low -0.011 m/s
(analyst `drift_table.csv`: -0.25, -0.07, +0.15, +0.43, +0.65, +0.98 m after
21-32 s; the last two landed on unverified surfaces). The +1.44 m of
2026-10-13 night flight 6 was the shadow `cvH`; `vzH` stayed 1.00 m.
Extrapolated to 120 s: +/-1.1 m (a random walk) to +/-2.3 m (linear).

- Reading high (the drone sinks): at <= 0.03 m/s it reaches the floor 35-50 s
  after a 1.05 m take-off, inside the old 80 s too. It ends in the
  controller's `K_RANGE` (Khat walks to its clamp in ground effect), a
  `HOVER_NO_CAM` or `AIR_TIME` LAND on the floor (and with 18.3 a `PALM` cue
  and the probe). Slow and benign: sim touchdowns <= 0.29 m/s.
- Reading low (the drone climbs): the worst logged rate over 115 s climbs
  1.3 m, to 2.35 m from 1.05 m or 2.5 m from 1.20 m: an 8 ft (2.44 m) ceiling
  is in reach. A LAND begun in free air below a ceiling comes down at
  ~0.2-0.3 m/s (the 2.0 m allowance keeps its flat part long enough). But a
  drone sucked against the ceiling (the ceiling effect raises the thrust) is
  a fall: **safety finding, 18.3** - the LAND's force touchdown rule is not
  height-gated and stops the motors at the ceiling (sim: a 2.4 m fall at
  5.85 m/s, b176c26 included). With this 120 s clock a slower drift reaches
  the ceiling (from 1.20 m: c -0.011 m/s at 100 s of air; under the old 80 s
  clock only drifts >= ~0.016 m/s did). Hence the Mac's ceiling rule (below):
  above 1.05 m a hover keeps the old 80 s; at <= 1.05 m the worst logged rate
  climbs to ~2.32-2.35 m in 115 s (linear), a thin margin under 2.44 m that
  the measured long hover (bench H1) has to confirm.
- `K_RANGE`: the clamp is 1.12 x the drone's vzK 150873 = 169.0k, against
  end-of-hover K of 162.5-168.3k after 25-37 s. Long hovers may end `K_RANGE`,
  a LAND that is not slow, from height. It exists at 80 s too (sim below:
  14 of 15 such LANDs came before 80 s); a fix is a separate decision.

**Operating rule (until a measured long hover says otherwise).** Fly hovers
that may pass ~80 s at <= 1.05 m, in a room >= 2.4 m high, and watch the
gap; send `land` if the drone rises past ~2.0 m, before it touches the
ceiling (never `k` for that: a deliberate 2 m drop, and a `k` with the props
spinning latches the motor driver until a reboot). Validate with one measured
long hover (autonomy-bench.md H1) before routine use.

**The ceiling rule in the Mac (`tools/espfly_auto.py` `CEIL_AIR_S` 80,
`CEIL_H_M` 1.05; the verification of 2c69fc5).** The operating rule's height
was advice only: the Mac took `--height 120 --wait-s 120`, and STOP
keep-alives held any height to 120 s, though its own refusal text said "105
cm, no higher". Now:

- a take-off above 105 cm whose wait (`--wait-s`, or with `--wait-s 0` the
  drone's `autowait` as the last `espfly_diag.py auto get` read it) plus the
  8 s before the wait passes 80 s is refused ("the ceiling rule"): above
  105 cm at most a 72 s wait;
- a hover whose commanded height (`hTarget`: the take-off height plus the
  vertical moves) is above 1.05 m is LANDed by the Mac at 80 s of air,
  whatever kept it alive (a wait, STOP / MOVE / YAW keep-alives): event
  `ceiling_land`, then the normal `land` (LAND_CMD: the slow LAND, fast from
  height as any Mac LAND). STOP itself is never refused. A lost link lands the
  drone by itself (HB_LOST), so the rule does not rely on the link more than
  the flight does;
- a MOVE that would end above 1.05 m past 80 s of air is refused (the Mac
  would land it at once).

At <= 1.05 m the 120 s clock is unchanged. The firmware keeps the 120 s
clock at every height: a firmware limit (80 s above 1.05 m, `AUTO_AIR_*` in
the FSM, with the motion module's MOVE check and `airLeftDs` following it) is
still the user's decision, as is a force touchdown that needs a descent seen
before it (18.3 finding 1). The model's ceiling suction (a gain of 1.5
within 1 cm) is not measured, so the real margin is uncertain both ways.

**Verification (host).**

- `test_auto_fsm`: `AIR_TIME` at exactly 120000 ms after the first RAMP step
  (still HOVER at 119999), a 60 s wait restarted by STOP; at 100 s of air
  (20 s left) a YAW 90 (its turn, dwell, bob and a LAND: 19.7 s) is accepted
  and a YAW 180 (22.7 s) refused `AIR`; at 14 s left a short MOVE still fits.
- `test_vert_ctrl` (`testAirClock120`): the default config valid with
  `holdMaxS` 119, 120.5 BAD_CONFIG; the drift allowance's flat part and
  timeout at 17.9 / 58.8 / 80 / 118 / 119 s of HOLD, blind or not (the table
  above); a 119 s camera HOLD on the sim: `HOLD_TIMEOUT` at 119.05 s, then
  the slow LAND, touchdown <= 0.29 m/s. Every earlier LAND test unchanged.
- `test_auto_glue_core`: `airLeftDs` 1199 at 100 ms, 200 at 100 s, 1 at
  119.9 s, 0 from 119.901 s.
- `test_interactive_sim`:
  - the air limit (STOPs keep a 60 s wait alive): LAND `AIR_TIME` at
    120000 ms from a true 1.40 m, the fast LAND, touchdown 0.24 m/s, no
    `HOLD_TIMEOUT`;
  - full hovers from 4.10 V (the drain cell): C 200 / 300 / 400 end
    `BATT_LOW` at 45 / 68 / 91 s (slow, touchdown <= 0.61 m/s; with the 80 s
    clock C 400 landed `AIR_TIME`); C 600 lands `AIR_TIME` at 120 s
    (0.26-0.28 m/s);
  - the long-hover drift sweep (`testLongHover`; c -0.017 / -0.011 / -0.005 /
    0 / +0.010 / +0.020 / +0.031 m/s x 1.00 / 1.05 / 1.20 m x 10 seeds, 120 s
    with STOP keep-alive): never KILLED, never cut in the air by the LAND
    timeout, a touchdown in all 210;

    | c (m/s) | ends | true height at the LAND | touchdown max (2.0 m cap) | with the old 1.0 m cap |
    |---|---|---|---|---|
    | -0.017 | `AIR_TIME` 30 | 2.78-3.92 m | 1.23 m/s (one of 30 over 1.0) | 1.73 m/s (17 of 30 over 1.0) |
    | -0.011 | `AIR_TIME` 30 | 1.93-2.96 m | 0.32 m/s | 1.07 m/s |
    | -0.005 | `AIR_TIME` 30 | 0.97-2.19 m | 0.33 m/s | 0.33 m/s |
    | 0 (the sim's own +/-0.004) | `AIR_TIME` 30 | 0.09-2.13 m | 0.36 m/s | 0.36 m/s |
    | +0.010 | `AIR_TIME` 11, `K_RANGE` 19 | 0.02-0.44 m | 0.29 m/s | same |
    | +0.020, +0.031 | `K_RANGE` 60 (on the floor) | 0.02-0.03 m | 0.14 m/s | same |

    The sim's estimate drifts more than c alone: its observer model does not
    see the controller's g0 capture error (+/-1.3 mg through the 0.2 s blend,
    +/-0.3 m in 120 s; the drone's observer reports its bias against g0), so
    the sweep is pessimistic. At c -0.017 (the allowance's design rate, 55 %
    beyond the worst logged) plus that error the drone was up to 2.5 m above
    its estimate: the last of it is flown after the flat part, faster (checked
    <= 1.3 m/s there, <= 1.0 m/s over the logged range). In a real room the
    ceiling would end such a climb first, and that is a fall: a ceiling
    contact cues `PALM` (or `COLLISION`) and the LAND stops the motors at the
    ceiling (18.3 finding 1); the Mac's ceiling rule keeps a hover above
    1.05 m to 80 s. The comparison builds
    the sim with `-DVZ_LAND_DRIFT_MAX_M=1.0f -DVZ_HOST_DRIFT_CAP_OVERRIDDEN`
    (a host-only override; the drone's build refuses it);
  - K drift (`testKDrift`: the plant's K +1 %/10 s from 160k against vzK
    150873, 0.60 / 1.05 / 1.20 m, 5 seeds): every sortie ends `K_RANGE`, at
    51.7-112.1 s of air (14 of 15 before 80 s), from up to 1.45 m true,
    touchdown up to 1.61 m/s (from 1.2 m: the LAND after `K_RANGE` is not
    slow). Reported, not a change of this item.
- `test_auto_sortie`, `test_autonomy_glue_host`: their 60 s waits end
  `HOVER_TIMEOUT` >= 48 s inside the 120 s clock.
- `test_espfly_auto.py`: `AIR_HARD_MS` against the header; the golden
  AUTO_STATUS (an 80 s drone's: `airMs` 41200, `airLeftDs` 388) reads `air_s`
  41.2 with `airClockMismatch` (120 - 38.8 s = 81.2 before the fix); this
  version's 788 reads 41.2 without it; at the limit 120.0; at 100 s of air
  from 1.05 m every MOVE / YAW is refused "too low for 100 s"; an 80 s fake
  drone: the event, MOVE refused, STOP and LAND OK.

**Bench (USB, after flashing).** `auto status` in IDLE. A DRY sortie
`sortie --mode dry --height 105 --wait-s 100` (or a 60 s wait with a STOP
every ~50 s): `airLeftDs` ~1190 right after RAMP, the Mac's `air_s` counts
past 80 s, LAND `AIR_TIME` at 120 s, no `HOLD_TIMEOUT`, no `BAD_CONFIG`;
`heapInt` >= 20 KB. The first long flight: autonomy-bench.md H1.

### 18.2 A hover wait of up to 120 s (`auto_proto.c`, `auto_fsm.c`, `autonomy.c`, `auto_glue_core.c`)

**Representation.** Inside the firmware the wait is `uint32_t` ms
everywhere; on the wire u24 ms; in NVS u32 ms with a read of the old u16
item. Every `(uint16_t)` cast of the wait is gone (120000 would have wrapped
to 54464 ms).

**Wire: TAKEOFF stays 12 bytes.** Byte 7, until now a pad that had to be 0,
carries bits 16-23 of `actWaitMs`: `actWaitMs = u16 at 10 | byte 7 << 16`
(`autoProtoParseTakeoff` checks the length only; the state machine checks the
range, `RANGE`). `autoProtoPackTakeoff` writes byte 7. An older Mac sends 0
there, so every wait up to 65535 ms is bit for bit as before (the golden
`takeoff_wait`, 60000, is unchanged); an older drone given a wait over
65535 ms refuses it at the parser (`RANGE`, detail 0) and never misreads it.
Switching the u16 to 0.1 s units was rejected: an older drone would read
1200 as 1.2 s. 120000 is byte 7 0x01 and bytes 10-11 0xD4C0.

**State machine.** `AUTO_HOVER_WAIT_MAX_MS` 60000 -> 120000 (static assert
<= `AUTO_AIR_HARD_MS`); `AUTO_HOVER_WAIT_TAKEOFF_MIN_MS` stays 1000.
`autoCmd_t.actWaitMs`, `autoFsmIn_t.hoverWaitNvsMs` and the sortie's
`hoverWaitMs` are `uint32_t`. To keep every offset the IRAM code uses,
`autoCmd_t` was reordered to its old 28 bytes (`type` still first), and the
two other fields moved to the end of their structs (the old u16 slots stay
as `reservedWait16`). ARM forces the NVS value into 0..120000; TAKEOFF takes
1000..120000. Unchanged: the `HOVER_TIMEOUT` row, `waitLeftMs` (already
u32), and the wait restarting on every accepted MOVE / YAW, on STOP and
while a motion runs. The wait starts ~5-7 s after the first RAMP step
(photos), so a 120 s wait always ends by `AIR_TIME` at 120 s of air, 5-7 s
of it left.

**AUTO_STATUS (layout unchanged: older Macs take only 64 or 96 bytes).**
`waitLeftMs` @34 and `hoverWaitMs` @36 stay u16 ms, saturated at 65535 ("65.5
s or more", `autoSatMs16()`): a 120 s wait shows 65535 for its first 54.5 s.
The pilot only ever sees the wait left underestimated (the safe side);
`airLeftDs` @94 stays the exact bound.

**NVS `autowait`.** The table entry is `{"autowait", AUTO_NVS_U32, 0,
120000, 5000, "ms", legacyType AUTO_NVS_U16}` (a new member,
`autoSettingInfo_t.legacyType`, 0 for every other key).

- `autonomyInit()`: a key absent with its type is read once more with its
  legacy type. ESP-IDF 5.0.7 lookups are typed: `nvs_get_u32` on the stored
  u16 item returns `ESP_ERR_NVS_NOT_FOUND` (`Storage::findItem` swallows the
  page's `TYPE_MISMATCH`). So the drone's stored 10000 keeps meaning 10 s, with
  no write at boot; the console logs `setting autowait: legacy u16 10000 ms`,
  and `auto get` shows its source as `legacy`.
- `autonomySettingSet()` for a key with a legacy type erases every item of
  the key first, then writes the u32 (a write adds an item of its own type
  and would leave the u16 one behind). If the write fails after the erase,
  the RAM copy follows NVS (the default).
- `autoSettingStoreErase()` (`imu_level_cal_store.c`) loops `nvs_erase_key`
  until `NOT_FOUND`, at most 4 times, then one commit: `nvs_erase_key`
  erases one item of any type, so `auto reset autowait` would otherwise leave
  the other item behind.
- Power lost between the erase and the write leaves the key absent: the
  default 5 s (fail-safe).
- Downgrade: an older firmware reads a u32 `autowait` as absent: 5 s.
- The USB `auto get/set/reset autowait` works as before, in ms (0..120000).

**The Mac (`tools/espfly_auto.py`).** `ACT_WAIT_MS = (1000, 120000)` (a
test reads it against the header); `takeoff_payload()` packs byte 7; `--wait-s`
1..120. The long-wait height rule is capped at the old 80 s horizon
(`WAIT_DRIFT_CAP_S = 80`, the recommended user-decision default; `None`
reverts it): `--height >= 0.25 m + 0.01 m/s x min(wait + 8 s, 80 s)`, so
--wait-s 30 / 60 / 72 / 120 need 63 / 93 / 105 / 105 cm (without the cap a
120 s wait would need an impossible 153 cm, and the longest usable wait at
1.20 m would be 87 s). Rationale: past ~80 s the floor-side sink is slow and
ends in a LAND either way, and a higher take-off would raise the ceiling
exposure (18.1); the refusal points to 105 cm, and above 105 cm the ceiling
rule (18.1) allows at most a 72 s wait. With `--wait-s 0` (the drone's NVS
`autowait`, up to 120 s now) both rules use the autowait as the last
`espfly_diag.py auto get` read it (the verification of 73969ad: `auto set
autowait 120000` then `sortie --height 40` passed unchecked); without a read
they are not checked and the reply's `note` says so (the ceiling rule's
LAND at 80 s still applies). `espfly_diag.py auto set` / `auto reset`, when
the drone accepts them, read `auto get` again for that cache. A wait over 65.5 s shows as
`waiting >= 65.5 s` on the status line and `waitLeftSaturated: true` in the
JSON (the sortie / state replies, the AUTO_STATUS view, the photo JSON and
the `waiting` event).

**Risks.** A u16 cast anywhere would silently turn 120000 into 54464 ms
(the glue host test runs NVS 120000 through the whole glue). A change of type
under the same key needs the legacy read and the erase-all, or the stored
10000 silently becomes 5000, or a reset brings the other item back (both
tested). An older drone with a new Mac refuses waits over 65.5 s (`RANGE`
detail 0) and takes the rest. The Mac rule's cap is a policy relaxation
(flagged for the user). Firmware downgrades after `auto set autowait` read
the default 5 s.

**Verification (host).**

- Golden vectors (`gen_auto_proto_golden.py` -> `auto_proto_golden_ih.json`;
  every existing vector byte-identical): `takeoff_wait_65536`,
  `takeoff_wait_120000` and `auto_status_ih120` (airMs, waitLeftMs and
  hoverWaitMs 65535, airLeftDs 150: `air_s` 105.0), built and parsed by
  `test_auto_proto` and `test_espfly_auto.py`.
- `test_auto_proto`: u24 round trips (bits 24-31 not sent); byte 7 0x01 +
  0xD4C0 parses as 120000; ARM / PHOTO / MOVE / YAW pads still refused.
- `auto_link_host`: TAKEOFF with byte 7 0x02 is posted with actWaitMs
  131072 + the low half; the FSM's `RANGE` comes back in the ACK.
- `test_auto_fsm`: TAKEOFF waits 1000 / 60001 / 65535 / 65536 / 120000
  accepted, 999 / 120001 / 131072 / 0xFFFFFF refused `RANGE`, 0 = NVS; NVS
  130000 forced to 120000 (the wait then ends `AIR_TIME`); a 70000 ms wait
  shows 70000 (no wrap); a TAKEOFF wait of 66000 ms lands `HOVER_TIMEOUT`
  66 s in; STOP and a MOVE restart a 100 s wait, which does not count down
  while the MOVE runs.
- `test_auto_glue_core`: autowait u32 0..120000, legacy u16, the only key
  with one; 120000 in range, 120001 not; the status saturates 110000 at
  65535.
- `test_autonomy_glue_host` (its NVS stub now typed like IDF's): only the
  u16 10000 stored -> 10000 and nothing written at boot; u16 + u32 stored ->
  the u32; none -> 5000; `set` erases both and writes the u32; `reset`
  leaves no item of either type; NVS 120000 reaches the FSM whole (the DRY
  wait shows 65535 at its start and still 54 s later, then counts down; it
  ends `AIR_TIME`); a 110 s TAKEOFF wait at 1.20 m (FLIGHT on the plant)
  lands `HOVER_TIMEOUT` at 115.9 s of air, touchdown 0.29 m/s.
- `test_auto_sortie`: a 120 s NVS wait at 1.05 m (c +/-0.010 m/s, 50 draws):
  `AIR_TIME` at 120 s in 47, `K_RANGE` (sunk into ground effect) in 3, never
  KILLED, touchdown <= 0.31 m/s.
- `test_espfly_auto.py`: the regexes, `--wait-s 120` accepted at 105 cm and
  refused at 100 cm (`needs --height >= 105 cm`), 121 refused, 60 s at 93 cm
  accepted; the height rule 63 / 93 / 97 / 105 / 105 / 105 cm at 30 / 60 / 64 /
  72 / 100 / 120 s; `waitLeftSaturated`; an older drone (fake) refuses a 100 s
  wait and is never left waiting.

**Cost.** IRAM: no function grew (`publishStatus` +4 B with the saturating
copy, `stepFull` -4 B); `.iram0.text` 129999 -> 129991 B; `.dram0.bss` +8 B
(the two u32 fields); `.flash.text` +0.3 KB.

**Bench (USB).** Before flashing: `auto get` (expect `autowait=10000`).
After: autonomy-bench.md H2.


### 18.3 The palm catch (`auto_fsm.h/.c`, `vert_ctrl.h/.c`, `autonomy.c`)

**Principle.** A supported drone reads ~1 g whatever its thrust; in free air
the specific force follows the thrust (30 ms rotor lag). At hover thrust the
two look the same, so the drone declares support only after testing it: it
lowers its own thrust and watches the accelerometer (the "probe"). Lift that
scales with the thrust (ceiling or ground effect, an updraft) and friction
against a wall go away with the thrust and fail the test. Nothing here reads
the height estimate.

**The cue (FSM, `auto_fsm.h/.c`).** In HOVER, from 1 s into the controller's
HOLD (`AUTO_PALM_ARM_MS` 1000, `vzOutput_t.tPhase`), an upward push or lift of
the drone: the controller's `aRel` (20 Hz low-pass of world-up specific force
minus g0) above `AUTO_PALM_G` 0.12 g for `AUTO_PALM_MS` 20 ms (`autoHeld_t
palm`, appended after `collided`; NaN fails it) -> LAND `PALM`
(`AUTO_RS_PALM` = 43, appended). The row sits right after `COLLISION`, so the
trips (KILLED) and a hard contact (`COLLISION`, blind) come first. Not in
BENCH, not in CLIMB (the commanded climb is 0.10 g and the lift-off reaches
0.175 g; a catch in TAKEOFF is `COLLISION`, armed 0.2 s after lift-off). The
LAND is the normal slow LAND from HOLD (not on the fast-LAND allow list, not
blind, the stall rule on); `enter()` stops a running MOVE / YAW
(`LEFT_HOVER`). A hard contact during it (row 8b's condition: |acc| > 1.6 g
or a gx / gy rate > 250 dps held 3 ms) makes it blind from that step
(`stepLand` sets `collided`: `vzInput_t.blind` and the glue's hold block),
the blind `COLLISION` LAND it would have been in HOVER; the reason stays
`PALM` (fix after the verification of b4d850d: the slow LAND's loop read the
estimate the contacts corrupted, and with its thrust under 0.75 T_hov the jolt
rule stopped the motors in the air: a soft cue, then knocks, 8 of 45 cut at
0.5-1.4 m in the sim, none since; before C a soft first touch started no
LAND, and the hard contacts after it were `COLLISION`). The camera hold is not blocked by the cue (the existing
contact, touchdown and |acc| > 1.5 g blocks apply). Margin in the logs: the
highest armed `aRel` in free flight over the 27 deduplicated sorties of
2026-10-12/13 is 0.050 g (0.067 g near walls per the designer's 100 Hz
analysis): 1.8x below the threshold. Outside free flight one logged sortie
cues: hop5s:3, an accelerometer-only hover sinking into the floor's ground
effect from 5.60 s (armed `aRel` up to 0.149 g there), cues at 5.81 s: a
`PALM` LAND on the floor. A ground-effect onset low over furniture or the
floor can do the same (a slow LAND there, finding 6); the replay checks
exactly this one cue.

**The support probe (`vert_ctrl.h/.c`), only in contact LANDs.** The glue
(`autonomy.c vzRun`) sets the new `vzInput_t.probe` = `collided` or reason
`PALM` (the reason stays `PALM` until IDLE): every LAND `COLLISION` and every
LAND `PALM`; never a Mac LAND, `AIR_TIME`, `BATT_LOW` or any other. With
`probe` false every output is bit-identical (`vert_ctrl_identity.h`).
`landProbe()` (`AUTO_HOT`, noinline) runs in the LAND-after-lift-off branch
after the thrust and the brake, before the touchdown rules:

- Start: the first probe `VZ_PROBE_ARM_S` 0.10 s into the LAND; later ones
  after a bump (`aLp20` > `tdNearJoltG` 0.30 g) once `VZ_PROBE_GAP_S` 0.40 s
  have passed since the last, and in a blind LAND also every
  `VZ_PROBE_EVERY_S` 2.0 s; each only with f = aLp20 + 1 >= `VZ_PROBE_TRIG_G`
  0.97 and the thrust above the level.
- The test: the thrust cap falls from the current thrust at `VZ_PROBE_RATE`
  2.0 x T_ref/s to `VZ_PROBE_LEVEL` 0.55 x T_hov and holds there.
- Abort (free air): f < `VZ_PROBE_FREE_G` 0.86 g for `VZ_PROBE_FREE_S` 10 ms,
  or `VZ_PROBE_MAX_S` 0.50 s, or the support statistic failing after the hold;
  the next step flies the LAND's thrust again.
- Support: the level held `VZ_PROBE_HOLD_S` 0.15 s with the mean specific
  force, clipped at `VZ_SUP_CLIP_G` 1.10 g, over the steps with thrust <=
  `VZ_SUP_U` 0.80 x T_hov at least `VZ_SUP_G` 0.95 g -> `touchdown`, DONE
  `VZ_R_TOUCHDOWN_SUPPORT` (= 28, appended); the motors stop in that loop.
- The force rule (FORCE: f over 1.3 x the thrust's prediction for 0.2 s)
  and `tdThrustOk` see the capped thrust: `tdThrustOk` holds during a probe
  (0.55 < 0.75), so FORCE may end it on a support, and a TILT / RATE trip
  with a bump in the last 300 ms is `TOUCHDOWN_TRIP` (a normal IDLE). The
  jolt rule (JOLT: `aLp20` > 0.6 g with the thrust condition) reads the
  LAND's own thrust, from before the cap (fix after the verification of
  b4d850d): the probe lowers the thrust as a test, in free air too, and with
  the capped thrust a second contact ~0.21-0.22 s into a free-air probe (the
  cap just under 0.75 T_hov) ended blind LANDs DONE `TOUCHDOWN_JOLT` at
  0.3-1.2 m, a fall (verifier harness: 29 of 4503 swept spikes, 1.2 % of
  single contacts at a uniform time, 14 % of Poisson contact trains, 17 % of
  the logged interactive:1 train jittered +/-50 ms; 0 without the probe).
  With the probe off the two conditions are the same value (bit-identical).

**The end.** `stepLand` sees VZ_DONE: IDLE `TOUCHDOWN` (vz
`TOUCHDOWN_SUPPORT`, `TOUCHDOWN_FORCE` or `TOUCHDOWN_JOLT`) after 1 s at rest
or 3 s; no KILL. `learnK`: a `COLLISION` / `PALM` LAND is `CMD_LAND` for the
controller, so the K sample of HOLD + 0.5 s (before any contact) is learned
as after any LAND; it is stored once the drone is put down (the aux task
saves only with the drone at rest on the ground).

**The Mac and the diag.** `espfly_auto.py` `REASON_NAMES` + `PALM`,
`VZ_REASON_NAMES` + `TOUCHDOWN_SUPPORT` (a test reads both against the
enums). `espfly_diag.py` `IN_AIR_ENDS` + `PALM` (the height at a touchdown on
a hand is not drift). The status line, `flights` and the recorder show the
names through the name functions.

**IRAM.** Paid by moving once-a-sortie code to flash (a separate commit
before this one, 4daf503: `spinEnd()` and `liftoffSeed()` in vert_ctrl.c,
`photoCmd()` and `startCapture()` in auto_fsm.c; each runs once a sortie or
per command, < 0.5 ms). Against b176c26: `.iram0.text` 129999 -> 129767 B
(-232), `_iram_text_end` 0x40394000 -> 0x40393f00 (one 256 B step fewer),
`_heap_start` 0x3fcc10d0 -> 0x3fcc1000 (internal heap +208 B), `.dram0.bss`
87512 -> 87560 (+48: vzState_t, the FSM's `palm` and `hoverWaitMs`),
`.flash.text` +1.2 KB. `landProbe` is 501 B (the design guessed 180-230 B:
the support statistic and three abort paths), `vzStep` +270 B for its call
and the SUPPORT ending, `autoFsmStep` +120 B for the cue.

**Closed-loop results (`test_interactive_sim.c`, the harness-only hand of
the spec: a spring-damper palm k 200 / 500 / 2000 N/m with c 2 / 5 / 10 N s/m
rising 10 cm at 0.02-0.30 m/s, then its approach stops in 0.1 s while it
lifts 0 / 1 / 3 cm over 0.2 s, minimum jerk; hovers at 0.40-1.20 m).**

| | result |
|---|---|
| every run (144 palms, 35 variants, 9 in a MOVE, 6 Mac LANDs, 9 grabs, 14 releases) | a normal IDLE end (or KILLED `ATT_ERR` on the palm, below); never SUPPORT in the air; the motors never stopped with the drone free in the air; floor touchdowns <= 0.40 m/s after a hand lowered away |
| which LAND | the damper's step c x v at the contact is \|acc\| > 1.6 g for 3 ms from c x v ~ 0.17 N: approach >= 0.10 m/s is always `COLLISION`; `PALM` mostly at <= 0.05 m/s with a lift (`PALM` 34, `COLLISION` 83, none 27 of 144) |
| a LAND at all | approach >= 0.10 m/s or a 3 cm lift: 96/96; <= 0.05 m/s with a 1 cm lift: 15/24; <= 0.05 m/s without a lift: 6/24 (the drone hovers on against the palm; the operator lowers it, the Mac lands) |
| `PALM` path: contact to motors off | every cue (34): p50 0.73, p95 0.78, max 2.86 s; drop on the palm 0; thrust after the cue <= 0.98 x T_hold. 22 of them are made blind ~0.45 s after the contact by the drone landing back on the palm (the damper's step at ~0.1 m/s is over 1.6 g for 3 ms), then end 0.71-0.79 s after the contact by FORCE in the probe that bump arms, one (k 200 / c 2) at 2.86 s, the `COLLISION` path's next probe (before the blind switch of the verification fix: max 1.002 s); the 12 that stay non-blind: p50 0.50, max 0.52 s |
| `COLLISION` path: contact to motors off | p50 2.08, p95 4.25, max 4.61 s (the same contacts without the probe, b176c26's blind LAND: p50 6.00, p95 6.67 s; sooner in 79/83) |
| in a sideways MOVE | the motion stopped at the LAND 9/9; `PALM` off <= 0.79 s (3 of 3 made blind by the bounce), `COLLISION` <= 2.53 s |
| a `PALM` cue, then hard contacts (`testPalmHardContacts`: Poisson 4/s over 1.5 s, 1.7-4 g, 3-10 ms, up-kicks on half; the palm held still or pulled away 0.25 / 0.45 s after the contact; 144 sorties, 67 cue `PALM`) | made blind 67/67; motors off free in the air 0 (without the blind switch 8 of 45), SUPPORT in the air 0, KILLED 0; floor touchdowns <= 1.02 m/s (after a palm pulled away) |
| a Mac LAND onto a palm held still | no probe; the stall rule and FORCE end it on the palm 0.53-0.56 s after the contact |
| a grab in the TAKEOFF climb at 0.40 m (held still, yaw 300 dps, roll 600 dps from 0.6 s) | `COLLISION`, motors off 0.40-0.41 s after it (before the rotation), never KILLED |
| the palm pulled away 0.2 / 0.4 s after the contact | the LAND goes on to the floor, <= 1.01 m/s (without the probe 1.27) |
| the hand tilting the drone 15 deg | KILLED `ATT_ERR` in 2 of 7 (a `COLLISION`-path drone resting tilted on the palm, waiting for the next probe); the motors stop on the palm, KILLED clears to IDLE |
| the touchdown that ends it | FORCE in every palm run, SUPPORT in none (below) |

Every other sim (1311 sorties without a hand, a gust or a ceiling: the
mission, guards, moves, turns and bobs, the depth step, the drain grid, the
contacts, A's 120 s drift sweep and K drift) never cues `PALM` and never
declares SUPPORT in the air. The probe in the existing free-air `COLLISION`
LANDs (pairs with and without it): hover 0.41 m/s max (without 0.86), MOVE
0.30 (0.81), MOVE up 0.71 (1.03), MOVE down 0.50 (0.37; one pair 0.45 vs
0.11, +0.34, over the design's +0.30), the re-track bob 0.65 (0.95). False
positives: vertical gust steps +/-0.05 / 0.08 g for 0.2-2 s 0/36; turbulence
sd 0.02 g over ten 120 s hovers 0, sd 0.04 g `PALM` in 5 of 10; lateral gusts
0/6.

**Second contacts (`test_vert_ctrl.c testProbeSecondContact`, after the
fix).** Contact LANDs from 0.70 / 1.00 / 1.20 m with random contacts after
the first, nothing holding the drone (a DONE above 3 cm is a fall), each
with and without the probe:

| LANDs | DONE in the air with the probe (the unfixed jolt rule) | without the probe |
|---|---|---|
| blind, one more contact at 0.05-1.0 s, 1-4 g, 3-10 ms (600) | 0 (6) | 0 |
| blind, Poisson 4/s over 1.5 s, 0.5-4 g, 2-10 ms, up-kicks 0-0.10 m/s (600) | 0 (79) | 0 |
| blind, the drone 8 % weaker after the contact, 3/s, 1-4 g (300) | 0 (6) | 0 |
| blind, the logged interactive:1 train, each contact +/-50 ms (300) | 0 (54) | 0 |
| `PALM` (not blind), soft contacts 0.2-0.6 g (300) | 0 (0) | 0 |
| `PALM` (not blind), hard contacts 1.7-4 g, the controller alone (300) | 2 (69) | 6 |

The last row is the non-blind LAND's own jolt rule (its loop thrust falls
under 0.75 T_hov in the descent): in the drone a hard contact makes a
`PALM` LAND blind (the FSM, below), so it is the blind rows. Every closed-loop
result of this section (the hand sims, the replays, the glue and sortie
tests) is unchanged by the fix: on a palm FORCE ends it, never JOLT.

**Controller unit tests (`test_vert_ctrl.c`).** Free-air LANDs from 0.40-1.20
m, 30 seeds: blind 353 probes, each aborted <= 0.136 s, touchdown max 0.45
m/s (without 0.77; worst pair +0.19); `PALM` 120 probes <= 0.124 s, 0.38 m/s
(0.42; +0.09). With acc noise 0.03 / 0.06 g, K +/-10 % at the LAND and a
descent-inflow gain (x1.3 at -0.8 m/s): never SUPPORT in the air, the level
never held free; DONE in the air with K -10 % 26/64 (without the probe 31,
a pre-existing blind-LAND weakness: T_ref then exceeds the need by 11 %), with
inflow 0 (without 6). The palm grid (k x wobble x 0.70 / 1.00 m, the LAND 20
ms after the contact): a touchdown on the palm within 0.40 s of the first
probe in 18/18, contact to motors off <= 0.72 s. The existing contact LANDs
with the probe: hover 1.00 m 0.51 m/s (0.73), 1.20 m 0.53 (0.90), CLIMB 1.22
(1.22), MOVE up 0.60 (1.00), bob 0.68 (0.95), after 60 s 0.35 (0.17); the
final floor arrival to motors off p50 0.14, p95 0.23, max 0.38 s.

**Replays.** `test_collision_replay.c` (every 2026-10-12/13 CSV, 27
sorties, `aRel` recomputed from the world-up acc and the SPIN g0):
`COLLISION` still first on battfwd:2 and interactive:1; no `PALM` at the
preAB:4 brush; one cue: hop5s:3 at 5.81 s, sinking into the floor's ground
effect (armed `aRel` 0.149 g there): a `PALM` LAND on the floor. The
open-loop probe replays (`test_vert_replay.c`, valid only while supported):
the hand catch battfwd:2 ends FORCE at 7.62 s (the logged RATE kill came at
8.04 s); preAB:4 FORCE at 14.21 s (the bed from below at 13.78 s, the kill at
15.06 s); the floor RAMP windows: the support statistic >= 0.95 in 81/81
(lowest 0.996).

**Open deviations from the spec's bounds (for the user: accept them as the
limits, or ask for a change; the tests check them at the measured values with
a little room, `test_interactive_sim.c` `PALM_ALL_*`, `PROBE_PAIR_MPS`,
`RELEASE_TD_MPS`, `test_vert_ctrl.c` `PROBE_PAIR_OPEN`, so a regression still
fails).**

| bound | spec | measured | checked at |
|---|---|---|---|
| `PALM` cue: contact to motors off | p95 0.70 s, max 1.00 s | every cue p95 0.78, max 2.86 s (the non-blind ones p95 / max 0.52 s, checked at the spec) | p95 0.80 s, max 5.0 s |
| the probe's cost, each free-air `COLLISION` LAND against the same without it | +0.30 m/s | closed loop +0.34 (one MOVE down 50 pair: 0.45 vs 0.11 m/s); controller unit +0.41 / +0.40 (hover contacts); every touchdown <= 0.71 m/s | +0.35 / +0.45, every touchdown <= 1.0 m/s |
| floor touchdown after a palm pulled away | 1.0 m/s | 1.01 (released), 1.02 (released, then knocks); b176c26's blind LAND without the probe 1.27 | 1.10 m/s |
| the cue outside free flight | none | hop5s:3 in the floor's ground effect (above) | exactly that one |

**Findings (deviations from the design, for the user).**

1. **SAFETY, not from the palm catch: a LAND against a ceiling is a fall.**
   With the spec's ceiling model (thrust gain up to 1.5 within 1 cm), a
   drifting long hover sucked against a 2.44 m ceiling cues (b176c26:
   `COLLISION`; now `PALM` first, at the gain's onset); the LAND lowers the
   thrust to <= 0.75 T_hov while the ceiling still holds the drone at ~1 g,
   and `TOUCHDOWN_FORCE` (never height-gated) stops the motors up there: a
   2.4 m fall, 5.85 m/s in the sim, with or without the probe (6/6). The
   controller unit test: 13/20 such LANDs DONE at the ceiling with the probe,
   20/20 without. A's 120 s clock lets slower drifts get there (from 1.20 m,
   c -0.011 m/s at 100 s of air). Since the `PALM` blind switch (below the
   verification of b4d850d) 3 of the 6 sim cases are made blind by the
   ceiling's own contact and fall off it later instead (LAND_TIMEOUT near the
   floor, 3.0-3.3 m/s): still a fall. Mitigation now: the Mac's ceiling rule
   of 18.1 (above 1.05 m a hover ends at 80 s of air: take-offs with longer
   waits refused, the Mac LANDs at 80 s) and the operating rule (long hovers
   at <= 1.05 m, a room >= 2.4 m, watch the gap; `land` early if it rises
   past ~2.0 m, before it touches). Options (not built, the user's decision):
   the same limit in the firmware; a force touchdown that also needs a
   descent seen before it (a ceiling-stuck drone never descended; it would
   change the palm catch, which FORCE ends).
2. **SUPPORT never ends a LAND in the sims.** The probe's thrust drops below
   0.75 T_hov at ~0.12 s and the existing FORCE rule fires ~0.31 s into the
   probe, ~0.05 s before the support verdict. Both stop the motors on the
   support; the verdict stays as the path for supports FORCE misses (none
   found).
3. **Contact to motors off is slower than the design's 0.465 s.** `PALM`:
   p95 0.78 s, max 2.86 s (spec p95 0.7, max 1.0): the slow LAND's speed
   loop reads the palm's lift as a climb and lowers the thrust, the drone
   leaves the palm by ~1 cm and lands back on it before the first probe can
   start (f >= 0.97); that landing back is a hard contact in the spec's
   damper model, so since the verification fix the LAND turns blind there and
   ends in the probe the bump arms (0.71-0.79 s), once in the next 2 s probe
   (2.86 s on the soft underdamped k 200 / c 2 palm, whose probe aborts on the
   bounce). The `PALM` LANDs that stay non-blind meet the spec (p95 and max
   0.52 s). `COLLISION` from a palm: p50 2.1 s, up to 4.6 s: a firm
   hand knocks the drone up, the first probe finds free air, the drone settles
   back with < 0.30 g (no re-arm), and the next probe is the blind LAND's 2.0 s
   one. Still 3x faster than b176c26 (6.0 s). Options (not built): a lower
   re-arm threshold or `VZ_PROBE_EVERY_S` 1.0 s.
4. **Most palms are `COLLISION`, not `PALM`** in the spec's hand model (the
   damper's step at the contact). Both paths probe and end in IDLE; the user
   sees LAND `COLLISION` for a brisk palm.
5. **The cue needs a lift.** A palm raised at <= 0.05 m/s without a lift cues
   6/24; with a 1 cm lift 15/24; a 3 cm lift or >= 0.10 m/s always lands.
   Instruction: bring the palm up at ~10 cm/s, then lift ~3 cm.
6. **An updraft's false cue lands hard.** 0.15 g for 0.5 s: LAND `PALM` in
   6/6 (spec: expected), the probe aborts in the air, but the slow LAND
   starts inside the updraft, whose lift its stall rule answers by lowering
   the monotone ceiling to ~0.81 x T_ref; when the updraft ends the drone
   descends on that ceiling: touchdown <= 1.41 m/s (without the probe 1.56;
   b176c26 had no LAND: ~0.27 m/s). Indoors 0.15 g updrafts are not expected;
   a ground-effect onset over furniture is the same mechanism (the drone then
   lands on the furniture). Option (not built): a `PALM` LAND whose first
   probe aborts in free air returns to HOLD.
7. **A tilted hold can still KILL** (`ATT_ERR`, > 10 deg for 0.5 s) before a
   `COLLISION`-path probe: 2/7 at 15 deg. The motors stop on the palm; KILLED
   clears to IDLE (FSM trips were never reboot-latched; only a Mac `k` with a
   motor duty nonzero latches the driver). Hold the palm flat.
8. **Risk 5 stands:** the motors stop on the support; a palm pulled away at
   that moment drops the drone from the palm's height. Keep the palm still
   until the motors stop (<= 1 s on the `PALM` path, a few s after a brisk
   `COLLISION`).
9. **Turbulence of sd 0.04 g cues in 5 of 10 long hovers** (reported; 0.02 g:
   0/10).

**Bench and first flights.** autonomy-bench.md H3.

## 19. Thermal gyro drift, bright windows, take-off kicks (user request 2026-10-14 ~10:00)

User request: "calibrate things such that bright windows are not a problem"
and "trim if you need to, calibrate things so it's stable". This morning's
flights on 14bcfc6 (living room, daylight, big windows;
`cal/flight-2026-10-14-morning.csv`, `flights-` / `crash-2026-10-14-morning.txt`):
flight 1 LAND `PALM` 6 s after a YAW 90 toward a white sofa, then KILLED `RATE`;
flights 2-3 `SPIN_GYRO` on the ground; flight 4 (48.5 s air facing the window)
crept backward and right until the user caught it while the camera hold applied
100 %.

**What the forensics found** (the numbers are the investigation's, re-checked
here where a test replays them):

1. **Flight 4's drift is the gyro's thermal drift, not the window** (~90 %).
   The gyro was zeroed on the ground at 43 C; in flight the IMU cooled to 33 C
   under the prop wash. The pitch gyro's zero moved +0.060 dps per C of cooling
   (+0.65 dps still after landing; roll -0.018 dps per C). The attitude filter
   (`sensfusion6.c`, Kp 0.4 /s, twoKi 0.002: ~400 s to learn a bias) settles at
   an error of bias / Kp: 0.14 -> 1.54 deg nose-down. The controller held that
   estimate at trim + hold = -1.4 deg, so the drone flew +0.05..+0.33 deg
   nose-up, backward and to the right at 0.05-0.16 m/s; the camera measured it
   correctly (~1.8 m back, ~1.0 m right), and the hold's integral (KI 2 deg per
   unit, 1.5 deg cap) cannot follow a ramping error. Every autonomy hover of
   10 s or more since 10-12 shows the same signature: the "drifts backward since
   the battery move" pattern, and why the levelfit pitch fits were NOT USABLE.
2. **The window did not cause flight 4**: 0 of 758 frames refused, brightness
   114-130, exposure steps on 0.8 % of frames, texture 42-63 (gate 8),
   residual 2-3, 8-11 % saturated.
3. **The earlier bright-window failures** (10-12/13, HOVER_NO_CAM) were at XCLK
   20 MHz: the 45/90 ms frame alternation (gone at 13.33 MHz, b176c26) and
   auto-exposure steps on 9-35 % of frames with residuals >= 8. The fit had a
   brightness offset but no gain, so a multiplicative step on a centre-bright
   frame reads as expansion. (The OV3660 "50 Hz" band step, 98 lines, is 8.45 ms
   at 13.33 MHz, within 1.4 % of US 120 Hz flicker: the banding registers stay.)
4. **Flight 1** (no recorder rows: flight 4's 60 s ring overwrote them): the cue
   came 6.09 s after the YAW (3.5 s turn + 0.5 s dwell + ~1.9 s into the re-track
   bob's upward acceleration). Most likely (~60 %) a soft contact with, or
   near-surface lift from, the sofa (the cue worked); possibly (~30 %) a false
   cue in free air from the bob. The RATE kill came 6.2 s into the slow LAND, on
   the soft sloped sofa (MEDIUM).
5. **Flights 2-3**: one filtered 1 kHz gyro sample over 15 dps in the idle
   spin's last 0.5 s aborts; logged single-sample peaks reach 14.4 dps.
6. **Take-off kicks**: the stored torque trim (+1792 / -4435) still corrects the
   CoG before the battery move; the hover's rate integrators sit at roll -1500 /
   pitch +5500 (flight 4). Every take-off since the move pitches 2.5-3.1 deg
   nose-down at lift-off, then overshoots +1.6..+2.8 deg. (A USB trim, below.)
7. **The take-off climb reads as forward motion**: +0.06..+0.79 ln of expansion
   over TAKEOFF (floor geometry, exposure); the hold, engaged 1 s after
   lift-off, integrated it into a nose-up lean at the HOVER entry.

Side note (not changed): the recorder's `tickMs` runs ~6.9 % slow against the
camera and wall clocks (it counts ~935 Hz stabilizer loops); scale ring
durations by 1.069.

**What is built** (a2b-int, one commit per item after 14bcfc6):

| Item | Commit | What changes | Section |
|---|---|---|---|
| A. Gyro temperature compensation | b40a827 | the driver subtracts k (T - Tref) from the gyro; NVS `gyrotc`, USB `gtc` | 19.1 |
| B. In-flight bias learning | 9807560 | the attitude filter's x / y integral at twoKiAir 0.040 while airborne; NVS `mkiair`, USB `mki` | 19.2 |
| C. Recorder | 6f1ba59 | 124 B records (temperature, compensation, learned bias, exposure), 180 s ring, `flights` fields | 19.3 |
| D. Flow fit: exposure | dde2241 | the gain on step frames, a template saturation mask, an 8 % step gate; NVS `flowfit`, USB `flowfit` | 19.4 |
| E. No forward hold in TAKEOFF | 9040655 | the hold's forward axis off in the climb | 19.5 |
| F. PALM at 0.16 g in a vertical reference | 5728c23 | the cue's threshold during the bob / a vertical MOVE and 0.5 s after | 19.6 |
| G. SPIN_GYRO for 20 ms | 6031528 | one vibration spike no longer aborts the take-off | 19.7 |
| L. The loop rate (after A4 read 921) | 819a4ce, 6cf394c | A at 20 Hz with an inline per-sample apply, B and C without calls on the loop; RAM-only bisect toggles; review round: the recorder's `sat16` out of line, `usecTimestamp` in IRAM, `auto prof` `bypassSeen` | 19.11 |

Not built (19.8): a firmware auto exposure (H), a flow-vs-IMU innovation gate
(I), gain changes (J), the PALM LAND that returns to HOVER (K). No PID or
Mahony Kp change.

### 19.1 Gyro temperature compensation (`utils/src/gyro_tempco.c`, `sensors_mpu6050_hm5883L_ms5611.c`)

**Model.** `corr[i] = clamp(k[i] x (min(T, knee) - min(Tref, knee)), +-1.0 dps)`,
T the MPU-6050 die temperature low-passed with tau 0.5 s (seeded by the first
sample; NaN skipped), Tref latched where a gyro bias takes effect, in the same
sensor step: when the startup bias is first set and inside every accepted
ground re-zero. Defaults (body axes = the recorder's gx / gy / gz): kx +0.018,
ky -0.060, kz 0 dps/C, knee 44 C (the zero moved 0.02 dps between 43 and 47 C),
clamp 1.00 dps (2.5 deg of estimate error at Kp 0.4), on. The sign: the pitch
gyro reads b_y = -0.060 (T - Tref) after cooling (T < Tref: positive), so the
corrected gyro reads b - corr = 0; a sign error would double the drift (the
bench cooling test B2 and the closed-loop sim catch it).

**Driver** (flash, the sensors task, 0 IRAM): in `processAccGyroMeasurements`
after the gyro is converted to dps and before its low-pass, the correction is
subtracted (off: +0.0, bit-identical). Since the loop-rate fix (19.11) the
filter and the model run once per 50 samples (20 Hz, `gyroTcBlock` on the
block's mean temperature, out of line) and each sample only adds its raw
temperature and subtracts the cached correction, inline (no call, lock or
divide); 0ed68c9 ran `gyroTcStep` and `gyroTcCorr` under a critical section on
every sample. An accepted re-zero records (Tref, T, the
bias step = new minus old raw bias in body dps, the model's correction at that
moment, flags: thrust since the previous latch, thrust above lift-off (30000)
since it, a `gtc hold` since it, on): `r = step - corr` is the error the
compensation left since the previous latch, the post-flight still-gyro error.
The model also runs (and is recorded) when off. **The re-zero log**
(`gyroTcLogRezero()`, pure, host-tested): a still drone re-zeros about once a
second, so an 8-entry ring of every re-zero lost the first one after a flight
~10 s after the motors stopped, before USB was plugged in (fixer review). The
ring now takes only the re-zeros that carry news: after thrust (a flight or a
ground spin), after a hold (the bench cooling test's residual), or with a step
>= 0.05 dps on an axis; and the first re-zero after an airborne stretch is also
kept on its own (`A`), with its delay after the last airborne sample and the
stretch's length, until the next airborne one (ground spins wrapping the ring
do not touch it). `test_gyro_tempco.c`: the flight's entry and its r_y read
back after 600 quiet re-zeros and 9 ground spins; a hold's and a 50 mdps step's
re-zero are logged, a 49 mdps quiet one is not; the clock wrap. The flight
summary takes the same r (19.3).

**USB** (`gtc`; motors stopped, autonomy idle, live until `gtc save`):
`gtc show` prints its detail lines first and the `OK gtc` line last (a reader
that stops at OK, `espfly_diag raw`, prints them all; there is no END line):
the `A` line (the latest airborne re-zero: seq, ageS, landS, airS, Tref, T,
dTeff, step, corr, r, on; `A seq=0` before the boot's first flight), one `Z`
line per logged re-zero (the same fields, flight=1 after thrust, air=1 above
lift-off, hold=1 after a hold, on), then OK (config, source nvs / default /
live, T, Tlp, Tref, the latch's age, dTeff, the correction now and the model,
re-zero counts `rezeros` / `logged` / `n` / `airSeq`, `thrustSinceLatch`,
`airSinceLatch`, the hold). `python3 tools/espfly_diag.py gtc [...]` sends any
`gtc` command and, for show, prints the post-flight check under the lines
(r, dTeff, PASS / FAIL, `SIGN` when the model's sign is opposite to the
measured step on an axis with both over 0.10 dps, and the refinement's `gtc
set` command). `gtc set <kx> <ky> <kz> [<knee>]` (|k| <= 0.15, knee
30-60 C); `gtc on|off`; `gtc save` (NVS `gyrotc` in `imucal`: version / size /
magic "GCTC" / crc32 like the attitude trim; the clamp is not stored); `gtc
clear` (erase, defaults live); `gtc hold <0..300 s>` (bench only, RAM: no
ground re-zero, so Tref stays; any thrust or an active autonomy ends it).

**Refinement** after each flight of >= 40 s: the `A` line (or the flight's
`flights` entry). If |r_y| > 0.10 dps and |dTeff| >= 5 C (dTeff = min(T, knee) -
min(Tref, knee), negative when cooling): `gtc set 0.018 <ky + r_y / dTeff> 0
44`, `gtc save`; roll the same way if |r_x| > 0.08 (`espfly_diag gtc` prints
the command). A residual of the expected size with the opposite sign (`step`
and `corr` of opposite signs, r about twice the step): `gtc off` and no long
hovers.

**Numbers.** Flight 4 (43 -> 33 C): corr y +0.600, x -0.180 dps (removes ~1.5
deg of pitch and ~0.45 deg of roll estimate error); flight 1 (47 -> 38 C): y
+0.360. A +-20 % coefficient error leaves <= 0.12 dps (<= 0.3 deg); B halves
it. Replay (`test_gyro_tempco.c`): flight 4's steady hover windows (HOVER, no
motion running, 3-5 s, the gyro mean minus the window's own attitude change:
the hold's corrections are real rotations), T(t) = Tend + (Tstart - Tend)
e^(-t / tau) with tau fitted on y: tau 18 s, residual rms y 0.093 / x 0.064
dps, worst 0.149 / 0.122 (the uncompensated zero: rms 0.468). Only flight 4 has
both an `IMU C start->end` entry and recorder rows. Cost: 0 IRAM, DRAM +232 B
(config, state, ring), flash ~6.6 KB (most of it the `gtc` console).

**Risks.** The knee rests on two ground points; below 33 C the slope is
extrapolated; warming (0.057) vs cooling (0.065) differ ~12 %. gz is not
compensated (unknown): the lateral derotation can still see a yaw-gyro drift.
The die temperature may lag the bias by seconds under prop wash. The
compensation is relative to the last latch and assumes the linear model over
the whole gap. The clamp and `gtc off` bound the damage.

### 19.2 In-flight bias learning (`modules/src/sensfusion6.c`, `estimator_complementary.c`)

`twoKiAir` (default 0.040: Ki 0.02 /s^2 with Kp 0.4 /s, poles 0.059 and 0.34
/s, the slow one tau ~17 s) replaces twoKi 0.002 on x and y while airborne (the
driver's thrust > 30000, set before every 250 Hz update together with the
driver's re-zero count; since 19.11 by inline loads and stores, no calls) with | |a|^2 - 1 | <= 0.20 (|a| 0.894-1.095 g) and |gx|,
|gy| <= 30 dps; z always uses twoKi. The x / y integral is clamped at 1.5 dps
and zeroed when the driver's re-zero count changes (the bias it learned is in
the new ground bias). `sensfusion6RateCorr` keeps its meaning (P + I, ~-bias),
so the camera observer's gyro-bias path (1996b09) is unchanged.
`sensfusion6GetLearnedBiasDps()` (= -integral) feeds the recorder. twoKiAir 0
is the 14bcfc6 filter bit for bit (no gate, clamp or zeroing). Kept at 0.04
(not 0.1) so that an error of A still shows in the post-landing re-zero
residual. USB `mki show|set <0..0.2>|save|clear` (NVS `mkiair`, a float); the
boot value is loaded in `estimatorComplementaryInit`.

**Numbers** (`test_sensfusion6_ratecorr.c`, `test_mahony_replay.c`,
`test_drift_thermal.c`, `test_cam_vz_shadow.c`):

- 0.6 dps in the air: error -0.055 deg after 60 s (without: -1.30 deg); on the
  ground bit-identical to twoKiAir 0; a 0.015 dps/s ramp (flight 4's cooling)
  -0.68 deg at 50 s (r / Ki 0.75; without -1.59); |a| 1.2 g and 52 dps fall back
  to the slow Ki bit for bit; the clamp at 1.501 dps; twoKiAir 0 = 14bcfc6 on a
  synthetic sequence crossing every gate and on flight 4's log (hashes pinned
  from 14bcfc6's file).
- 36 logged flights of 10-11..10-14 replayed through the filter (the logged 100
  Hz gyro / accelerometer at the firmware's own integration time; the 14bcfc6
  filter tracks the logged attitude to 0.01-0.18 deg): 17 of 17 hover-end
  innovations over 0.3 deg drop (flight 4 pitch 1.18 -> 0.13 deg, roll 0.33 ->
  0.05); the worst 1 s window of TAKEOFF / a MOVE or YAW / LAND / contacts gets
  0.08 deg worse against the accelerometer (limit 0.2); the learned bias <= 0.46
  dps on the 21 flights that ended in a normal LAND (flight 4: +0.48 dps at its
  end).
- Closed loop (the real filter at 250 Hz, the real camHoldStep at 14.8 fps,
  b_y = 0.86 (1 - e^(-t/31)) dps, b_x = -0.3 b_y, drag 0.5 /s, D 2.65 m):
  14bcfc6 reproduces flight 4 (-1.71 m and hold pitch -1.47 deg at 45 s,
  -4.70 m at 120 s); B alone -0.64 m at 45 s; A exact + B max |drift| 0.29 m;
  A with +30 % / -30 % coefficients + B 0.44 / 0.24 m over 120 s, the hold's
  mean pitch within +-0.11 deg.
- The camera observer's replays with the filter's own rate correction, twoKiAir
  0 -> 0.040: drift 0.1152 -> 0.1148 m/s (files 6-15), 0.0480 -> 0.0481 (16-21),
  offsets within 3 mg 26/28 and 26/27 both, counters identical.

**Risks.** It can learn from a held or tilted drone, a contact or a catch:
gated by |a| and the rate, clamped, zeroed at the next ground re-zero. With A it
can partly mask an error of A (hence 0.04). The estimate still converges to the
accelerometer direction: its errors (facc 12 mg; ~0.44 mg per C of x/y drift,
~0.25 deg over 10 C) remain, <= 0.3 deg.

### 19.3 The recorder (`flight_recorder.h/.c`, `usb_diag.c`, `flight_rtc_log.c`, `tools/espfly_diag.py`)

`flightRecord_t` 112 -> 124 B (asserted), appended: `imuTemp` (0.01 C),
`gtcCorrX/Y` (A's correction, mdps), `mkiBiasX/Y` (B's learned bias, mdps),
`flowGain` ((a - 1) x 1000, +-127) and `flowSat` (saturated %, bit 7: the frame
was refused for its exposure step; both from the camera sample's padding, which
stays 64 B, asserted). The dump adds the CSV columns `imuTemp_cC`,
`gtcCorrX_mdps`, `gtcCorrY_mdps`, `mkiBiasX_mdps`, `mkiBiasY_mdps`,
`flowGain_e3`, `flowSat_pct`, `flowExpGate` (72 columns). The recorded gyro is
the compensated one. The ring: 6000 -> 18000 records (180 s, 2.23 MB PSRAM),
12000 if 180 s would leave PSRAM under 1 MB, the internal fallback unchanged
(1500); `rec info` shows `psramFree`. A full dump is ~3x longer. The flight
summary (RTC) adds the end-of-flight `gtcY` and `mki` x,y (mdps; `flights`;
RTC magic E6, summary 140 B, `.rtc_noinit` +64 B), and since the fixer review
the first ground re-zero after the flight (19.1: `gtcRf` set / on, `gtcR` r x,y
mdps, `gtcRdT` dTeff 0.01 C, `gtcRland` 0.1 s after the last airborne sample;
filled by `flight_rtc_log.c` when the driver's airborne re-zero count moves and
the newest summary is this boot's, closed and flew; magic E7, summary 148 B,
`.rtc_noinit` +128 B more). `espfly_diag` reads the new
columns by header (older dumps: empty), prints the IMU temperature, the
compensation and the learned bias at the end, the exposure steps corrected /
refused and the saturated %, and the flights table's `gtc y` / `learned x/y` /
`post-flight r x/y @dTeff`.
The C replays read up to 20000 rows. Rule kept: dump after any PALM, COLLISION
or KILLED flight before the next take-off.

### 19.4 The flow fit and bright windows (`utils/src/optical_flow.c`, `camera_flow.c`)

**Model.** `I_prev(x) = a I_cur(x + d(x)) + c`. The gain a = mean(I_prev) /
mean(I_cur) over the flat unsaturated coarse pixels (|gx| + |gy| <= 3 grey/px,
I_prev >= 8, < 250; >= 300 of them, else 1), clamped 0.80..1.25, exactly 1
inside a 0.6 % dead band. **Deviations from the design, measured** (warps of
the drone's own photos, `test_flow_exposure.c`; experiments in the implementer's
notes):

- The gain is measured in the coarse level's iteration 1, not 0: at
  iteration 0 the warp is still the initial one and the selection of flat
  pixels biased the ratio by up to 0.5 % at a steady exposure.
- A frame with a = 1 (no step) is fitted plain to the end, bit for bit the fit
  of 14bcfc6: a saturated window with a steady exposure moves rigidly, and
  leaving it out only lost accuracy.
- On a step frame the mask tests the template only: the pixel or a neighbour
  its gradient uses at or above 250, or the pixel itself at or above 250 a when
  the current frame is brighter (a < 1, where it clips). The design's test of
  the warped sample (jv >= 250) refuses pixels by the sign of their residual:
  on a synthetic 30 % window it shifted the fit -0.08 px (plain +0.02); the
  gradient-stencil mask +0.002.
- `satFrac` is the fine level's on step frames and the coarse level's (pixels
  >= 250) on the others (counted in the gain's iteration: no extra fine pass).

The texture gate keeps its denominator (in-bounds pixels, masked ones
included), the residual and the n < 100 test use the pixels used.
`camera_flow.c`: `s.ok` also needs `ofExposureStepTooLarge()` false: |a - 1|
<= 0.08, or without a measured gain (OF_FIT_PLAIN, or < 300 flat pixels) the
mean brightness within 8 % of the previous frame's. One flag gates the hold
(it coasts) and the camera height observer (it refuses the frame); the
recorder's `flowOk` includes it, `flowExpGate` tells it apart. The camera keeps
its own auto exposure and registers (H).

**Modes** (the CPU fallback, NVS `flowfit`, USB `flowfit robust|lite|plain`,
used from the next frame, saved): robust (default); lite (the mask on the
fine level only, the statistics on every other coarse row); plain (14bcfc6's
fit, the gate on the mean brightness only). The reply also shows the refused
and corrected step counts and `camProcAvgUs` / `camProcMaxUs`.

**Numbers.** Constant exposure (shift +-2 / 0.3 px, scale +-1 %, rotation 0.01
rad, five photos): the gain reads exactly 1 on every frame and the default fit
is the plain one bit for bit. Steps x0.95 / x1.05 / x0.90 on alternate frames
and +-2 %/frame ramps (with a 0.3 px drift, and receding 0.2 %/frame): every
frame seen as a step, the gain within 0.39 % of the truth, residual <= 1.5,
fake scale 0.00003-0.00031 a frame (plain 0.0003-0.0047), the rate within
+-0.0046 /s (plain up to -0.0134 /s); the white bed's texture unchanged. Steps
of x0.95 are corrected, x0.80 refused. OF_FIT_PLAIN = 14bcfc6 (hash).
`test_optical_flow` / `test_flow_scenes` pass; the walker frames of
`flow_scenes` (refused anyway) now read as steps (a person shifts the
flat-pixel mean: residual mean 18.4 -> 18.3, 21.0 -> 20.6). Replay proxy (no
image sequences were logged): frames whose logged brightness moved > 8 %
refused through the hold and the observer replays: the good flights (flight 4,
tiger f1/f2, tiger2, mission60, night f6, pwmA1 f7) 0-1 HOVER frames (<= 0.22 %),
no new coasting run and no new unhealthy stretch; the 20 MHz failures:
interactive f2 0, preAB f3 9 airborne frames (1 in HOVER).

**CPU.** One specialised pixel loop per pass kind; on the host a frame without
a step costs +0..2 % (the statistics' iteration), a step frame -2..-4 % (the
masked pixels are skipped). Estimate on the drone: ~+0.6 ms of 58.66 ms. The
budget is `camProcAvgUs` <= 60500 (the clock fallback's rule is 62000; measured
on 0ed68c9 at 13.33 MHz: plain 61.86, lite 62.1, robust 62.0 ms, over it: see
19.11 for the cause and the re-check); over
it, `flowfit lite`, then `flowfit plain` (no reflash). 0 IRAM, flash +4.9 KB.

**Risks.** A large moving object (a person) shifts the flat-pixel mean and
gives a wrong gain (clamped and gated; fit noise of ~0.0035 /s per 0.5 % of
gain error). Gamma and saturation leave part of a large step uncorrected (the
8 % gate). In window-dominated views a step frame uses fewer pixels (texture
26-50 % lower on the window photos, still >= 19 against the gate's 8).
Unproven at 13.33 MHz, where flight 4 was
already clean: robustness, not this morning's cause.

### 19.5 No forward hold in the take-off climb (`camera_hold.h/.c`, `camera_flow.c`)

`camHoldInput_t.fwdOff`: the forward axis neither filters nor integrates (fwdF
0; iFwd, pFwd and the forward reference bookkeeping unchanged) and outputs KI
iFwd + ffPitch; the lateral axis, engaged and coasting are unchanged.
`camera_flow.c` sets it while the recorder's autoState is TAKEOFF, so the
forward axis starts at the first HOVER frame from fwdF 0 and iFwd 0. fwdOff
false is bit-identical (`camera_hold_identity.h`). Fly it after the torque trim
(19.9): the climb no longer corrects a lift-off kick forward.

**Numbers.** The 25 logged autonomy take-offs of 10-12..10-14 (TAKEOFF
expansion positive in 20): the hold's pitch entering HOVER 0.00 deg (with the
forward hold in the climb: mean +0.71, up to +2.50), after the first HOVER
frame's own P update <= 0.42 deg (open loop: the logged frames carry the motion
the old kick caused, so convergence cannot be shown there). The interactive sim
(climb fake expansion 0.3 v / D; 20 seeds, `testTakeoffKick`): with no lean
error the mean displacement 2 s into HOVER -0.19 -> -0.01 m, the hold's pitch
at the HOVER entry 0.24 -> 0.00 deg; with a +-0.3 deg lean error (uncorrected in
the climb now) |displacement| 0.20 -> 0.16 m mean, max 0.33 m. Two sim checks
moved with the take-off position: `testDepthStep` flies without the lean error
(its seed drifted 0.24 m toward the box in the climb: D 1.32 m made +-75 cm
TOO_WIDE; at D 1.56 as designed both pass), and the weak-cell take-off grid
allows one `HOLD_CAPPED` (the end-of-battery race between BATT_LOW and
HOLD_CAPPED happens 1 in 324 sorties both before and after this change over 6
seed sets; this grid's seeds hit it once: touchdown 0.72 m/s, limit 1.30).

**Risks.** No forward position hold in the ~4 s climb: a wrong torque trim or
a gust is uncorrected there (~0.1-0.2 m per 0.2 deg). The fake expansion of the
hover's own vertical motion (the bob, a vertical MOVE) is not addressed
(symmetric in the bob).

### 19.6 The palm cue in a vertical reference (`auto_fsm.h/.c`)

(Removed with the palm cue on 2026-10-15, section 22.)

`AUTO_PALM_G_VREF` 0.16 g while a vertical reference runs (`out.mv.vRef` != 0:
the YAW's re-track bob, the lateral re-view bob, a vertical MOVE) and for
`AUTO_PALM_VREF_HOLD_MS` 500 ms after it; 0.12 g otherwise. `autoFsm_t.vRefMs`
(appended; the latest such step's time | 1, 0 = none this sortie, cleared at
ARM; an integer test of vRef's bits and a signed window test, for the IRAM
budget). The cue is otherwise unchanged (20 ms, HOVER, HOLD tPhase >= 1 s);
COLLISION and the probe are unchanged. Free-air margin in a bob: 0.12 - (0.058
+ 0.04) = 0.022 g before, 0.062 g now (the plain hover's).

**Numbers.** `test_auto_fsm.c`: a MOVE up: 0.13 / 0.159 g for 30 ms nothing,
0.17 g LAND PALM; 0.13 g cues 500 + 20 ms after the last vertical reference;
in the bob 0.13 g nothing, 0.17 g PALM; plain HOVER 0.13 g still cues.
`test_interactive_sim.c`: every existing run unchanged (the 144 palms: p50
0.73 / p95 0.78 s; the sorties without a hand never cue); new palms in the bob /
in a MOVE up 20 (24 + 24, the soft palm, every approach x lift): PALM 5 / 9,
COLLISION 17 / 11, none 2 / 4 (with 0.12 g: PALM 7 / 9, none 1 / 4), all a
normal end. `test_collision_replay` identical (hop5s:3 still cues). A slow palm
during a bob or a vertical move needs a firmer push (or ends COLLISION). It does
not prevent a cue from real furniture or near-surface lift in a plain hover;
flight 1's verdict is a judgment without recorder data.

### 19.7 SPIN_GYRO for 20 ms (`vert_ctrl.h/.c`)

`vzConfig_t.spinGyroHoldS` (default 0.020 s, valid 0..0.2, 0 = the one-sample
rule) and `vzState_t.spinGyroS` (both appended). In the SPIN window a sample
over 15 dps adds dt and aborts `SPIN_GYRO` once it reaches 20 ms; a sample under
resets it; a sample over stays out of g0. Logged idle spins (26): single 100
Hz samples 5.7-14.4 dps (90th percentile 13.0), three in a row <= 9.3, a 50 ms
mean <= 9.9. A rotation shorter than 20 ms passes (<= 0.3 deg). A damaged prop's
vibration no longer trips this check (SPIN_NOISY and the in-air checks remain):
inspect and seat the props after any crash. `test_vert_ctrl.c` (a 1 ms 40 dps
spike: no abort, not in g0; 19 ms at 16 dps none, 20 ms SPIN_GYRO; hold 0 =
the old rule); `test_vert_replay.c`: the 26 logged SPINs (100 Hz rows at the
real 1.07 ms loop): no abort with either rule, the same g0 in all 23 full
SPINs; the identity hash unchanged.

### 19.8 Not built

- **H. A firmware auto exposure / zone weights / AE freeze / banding change.**
  The OV3660 keeps its own AE (uniform 4x4 weights, target 50-59) and the
  manual "50 Hz" band (98 lines = 8.45 ms at 13.33 MHz, within 1.4 % of US 120
  Hz flicker; the "60 Hz" registers would give 7.05 ms). The effective step per
  frame is now measured for free (`flowGain_e3`). Reopen if more than 2 % of
  HOVER frames show |gain - 1| > 0.03 at 13.33 MHz.
- **I. A flow-vs-IMU innovation gate / "hold cannot hold" LAND.** In flight 4
  the camera was right and the attitude wrong: a gate built on the estimate
  would have refused the true drift. Watch the hold's mean pitch per flight; if
  it exceeds 1.0 deg after A + B, reopen.
- **J. Hold KI / limits or Mahony Kp.** The cause was a ramping estimate error
  that A + B remove (D x ramp / KI = 0.04 m/s).
- **K. A PALM LAND whose first probe finds free air returns to HOVER** (18.3
  finding 6): the user's decision; flight 1 gives no data to justify it.

### 19.9 Trims (USB, after flashing; not changed by the firmware)

1. Torque trim (independent of this build; direction HIGH, value +-600):
   `trim show` (expect 1792, -4435), `trim set 250 800`, `trim save`. Basis:
   flight 4's hover rate integrators roll -1500 / pitch +5500 -> the hover needs
   ~+290 / +1115; the trim analyst's 3-flight mean ~+200 / +280..+500. Check on
   the first take-off: the pitch excursion within +-1.5 deg in the first 0.4 s
   after lift-off; the hover's `pitchRateI` mean within +-1500, roll +-1000;
   else move the pitch trim by that mean.
2. Attitude trim, only with this build (LOW-MEDIUM): `atrim set -0.20 0.00`,
   `atrim save` (roll, then pitch). Without it: leave -0.20 / -0.10 and keep
   hovers to ~20 s.
3. facc unchanged (`0.0120 0.0000`); the trim learner stays off until 2-3 hovers
   of >= 40 s pass F1 (re-zero |r_y| < 0.15 dps, innovation < 0.4 deg, hold mean
   pitch within +-0.4 deg).
4. The new defaults need no command (`gtc`, `mki show`, `flowfit`).

### 19.10 Cost and verification

**Built against 14bcfc6** (int-sdkconfig, ESP-IDF 5.0.7): `.iram0.text` 129599
-> 129747 B (+148: F's `autoFsmStep` +100 and `enter` +4, G's `vzStep` +39;
A-E are flash code), 41 B to the next 256 B step (was 189: no step taken);
`.dram0.data` 31480 -> 31528, `.dram0.bss` 87560 -> 87800: the heap start
+288 B (expect `heapInt` ~22.1-22.5 KB with the Mac linked, from 22.4-22.8;
>= 20 KB required); `.flash.text` 734467 -> 749535 (+15.1 KB), `.flash.rodata`
186032 -> 187960; `.rtc_noinit` 7084 -> 7148; the app 1100192 B (28 % of the
partition free). The recorder takes 2.23 MB of PSRAM (was 0.67). **Fixer
review** (the post-flight re-zero kept, `gtc show`'s order, the summary's r):
`.iram0.text` 129743 (-4: `flightRtcLogLive`'s literal pool holds one constant
fewer with the RTC store's new field offsets; every IRAM function the same size
as abe885b's; 45 B to the next 256 B step), `.dram0.bss` +48 B (the re-zero log
and its flags), `.dram0.data` unchanged, `.flash.text` 751299 (+1.8 KB),
`.flash.rodata` 188224, `.rtc_noinit` 7276 (+128 of 8192), the app 1102224 B.

**Host tests.** New: `test_gyro_tempco.c`, `test_mahony_replay.c`,
`test_drift_thermal.c`, `test_flow_exposure.c` (with the photo fixtures in
`tools/tests/fixtures/photo-*.raw`). Extended: `test_sensfusion6_ratecorr.c`,
`test_cam_vz_shadow.c`, `test_cam_vz_replay_ih.c`, `test_hold_replay.c`,
`test_camera_hold.c`, `test_interactive_sim.c`, `test_auto_fsm.c`,
`test_vert_ctrl.c`, `test_vert_replay.c`, `test_flight_summary.c`,
`test_espfly_diag.py`. Every C program and the python suite: 0 failures.

**Bench and first flights:** autonomy-bench.md I.

### 19.11 The loop rate (user decision 2026-10-14 ~11:00: option 2, fix the speed first)

**Measured on the drone** (Claude, USB, the A4 procedure: `camvz air full`, a
DRY sortie, `auto prof` `activeLoopsPerS`; gate >= 940):

| Build | activeLoopsPerS | camProcAvgUs at 13.33 MHz |
|---|---|---|
| 14bcfc6 | 961.1, 960.6 | 59.98-60.8 ms |
| 0ed68c9, flowfit plain | 921.4; `gtc off` 931.6; `mki set 0` 923.0; both 932.1 | plain 61.86, lite 62.1, robust 62.0 ms |
| 0ed68c9, camera at 20 MHz | 921.9; `gtc off` 930.7 | |

The camera clock does not matter; internal heap ~19.4 KB (14bcfc6 ~20.0-22.8).
Robust and lite were over the clock's 62 ms rule (the camera fell back to 20
MHz). Goal: >= 955 (ideally ~960), A and B functionally identical, D's modes
kept, RAM-only toggles to price each addition on the drone.

**Cause** (two read-only analyses, scratchpad `lr/analyst`, `lr/contention`):
new flash code on the 1 kHz sensors / stabilizer path. Both cores share the 16
KB instruction cache (32 B lines) and core 1 runs the flow fit ~90 % of the
time, so every new line run once per ms misses: ~1.7-2 loops/s per line per ms
(the same mechanism as 2026-10-12, autonomy's per-loop code in flash 630-700
loops/s, in IRAM 948-973). 0ed68c9 ran on every sample, in the sensors task,
a critical section (interrupts masked) around `gyroTcStep` (a ROM divide),
`gyroTcCorr` and `gyroTcModel` (8 fminf / fmaxf calls), the temperature decode
(a divide) and a tail GCC placed after the loop: ~19 new lines a sample. `gtc
off` skipped only `gyroTcModel` (~5 lines: the measured +10.2); `mki set 0`
only the new integral branch (the 4 calls stayed: +1.6). Every 10 ms the
recorder and the flight summary added three getters (two critical sections,
four divides) and the post-flight re-zero poll (a critical section and a copy
of 34 B, for an event once a flight); every 4 ms the attitude update made 4
calls into 3 files and 4 fminf / fmaxf. Unchanged between the builds: task
priorities, core pinning, sdkconfig, IRAM placement (E, F, G add 144 B of IRAM
and no flash call from IRAM: < 0.5 loops/s). The plain fit is 14bcfc6's
arithmetic (127 against 130 instructions a pixel); its +1.4-1.9 ms is wall time
the 1 kHz tasks' misses take on core 1, not the camera's own work.

**What changed** (819a4ce, 0 IRAM bytes):

| | 0ed68c9, per loop | Now |
|---|---|---|
| A, sensors task, 1 kHz | lock, 2 divides, 3 calls, ~19 lines | inline: add the raw temperature to the block's sum, count, subtract the 3 cached floats (70 B, ~2 lines); `gyroTcBlockStep` once per 50 samples (20 Hz, out of line, cold: decode the block's mean, filter with dt 50 ms, the model, publish under the lock) |
| A, stabilizer, 1 kHz, thrust > 0 | 5 volatile stores | 14bcfc6's two stores, `lastAirTick` when above lift-off, the hold's end only when on; the re-zero log's thrust / air flags derived at 20 Hz from the ticks |
| A, re-zero (~1 Hz) | the entry built with interrupts masked | built outside the lock; the lock copies the config and publishes |
| B, stabilizer, 250 Hz | 4 calls (3 files), 4 fminf / fmaxf | inline loads and stores (45 B), a call only when the re-zero count moves; the clamp inline |
| C, stabilizer, 100 Hz | GetGyroTcCorr (lock), GetTempC, GetLearnedBiasDps (3 divides), GetGyroTcAir (lock, copy) | inline reads of the published values; GetGyroTcAir only when the driver's airborne re-zero count moved |
| D | the fit unchanged | unchanged |

**A at 20 Hz: what changes, and why it does not matter.** 0ed68c9, per sample:
T = raw / 340 + 36.53, tLp += (T - tLp) dt / (tau + dt) with dt 1 ms, the
model from tLp. Now: the mean of 50 raw samples, the same filter with dt 50 ms,
the correction held for the block; a latch (ground re-zero) recomputes it at
once from the latest block's tLp (Tref up to 50 ms old). `gyroTcBlock` with
one sample is the old per-sample step bit for bit (`test_gyro_tempco.c`). Both
pipelines on the same 1 kHz raw stream (+-3 LSB noise): flight 4's cooling (43
-> 33 C, tau 18 s, up to 0.56 C/s) with 118 re-zeros: the subtracted correction
differs by <= 2.18 mdps on y, 0.65 on x (the correction itself up to 565), the
logged model by <= 0.06 mdps, Tref by <= 0.001 C; the bench cooling test
(0.04 C/s) <= 0.20 mdps; an instant 1 C step (not physical) <= 5.6 mdps; off:
+0.0 on every sample, so the gyro is 14bcfc6's bit for bit. The error grows
~0.044 mdps per sample of block (10 samples: 0.41): 20 Hz keeps it under 1/28
of a gyro LSB (61 mdps) and 1/55 of the model's +-20 % coefficient error (120
mdps); B learns what is left. The re-zero log's airborne stretch starts at the
block that saw it (`airS` <= 0.05 s short); its end (`landS`) is exact.

**B bit for bit.** The setters keep `sensfusion6NoteGyroBiasEpoch`'s rule (the
first note sets it, a change zeroes the integral when twoKiAir > 0); the clamp
`!(v >= -M) ? -M : (v > M ? M : v)` is fminf(fmaxf(v, -M), M) for every v (NaN
-> -M, -0 kept). `test_sensfusion6_ratecorr.c` pins 0ed68c9's twoKiAir 0.040
hash (0x44EA30DC66EFEB2F) beside 14bcfc6's twoKiAir 0 hash: both match.

**C, the same values.** The recorder reads the latest sample's temperature
(the same expression), the correction the driver subtracts now, the learned
bias by the same expression; the 124 B record, the CSV columns and the flight
summary are unchanged.

**D.** The fit's code is unchanged. `test_flow_exposure.c` now hashes every
output of the robust and lite fits too, pinned from 0ed68c9 (robust
0xFF4EC5A8FBF0B54A, lite 0xF23F6588366CE079; plain = 14bcfc6 0x963CD6E7944A5815):
a later optimisation of the fit must keep all three. The budget: after this
build, `camProcAvgUs` at 13.33 MHz per mode (autonomy-bench.md A4b, step 4);
robust / lite stay only at <= 60500, otherwise fly plain at 13.33 MHz and
record it here. Not built (a decision of its own): unrolling the fit's
normal-equation accumulation (`#pragma GCC unroll 5` on its two inner loops)
is bit-identical (the three hashes on the host; on the xtensa each accumulator
stays one madd.s, or an exact add.s for the constant 1 column) and cuts the
plain pixel loop from ~295 to ~170 instructions (estimated -20..-35 % fit
time), but it also moves each frame's output ~10-20 ms earlier (the hold's
latency) and, at 20 MHz, would raise the processed frame rate: a timing change
that wants its own bench and flight check (scratchpad `lr/impl/unroll`).

**The bisect toggles** (`utils/interface/bench_bypass.h`; USB `bypass
[gtc|mki|rec|flow|all on|off]`, RAM only, 0 at every boot, motors stopped and
autonomy idle; shown as `bypass=0x..` in `status`, `auto prof`, and per bit in
`gtc show`, `mki show`, `flowfit`). `auto prof` also prints `bypassSeen=0x..`:
every bit that was on at any time since `auto prof reset` (set from the mask at
the reset, each change ORed in; the USB task only, nothing on the loop). The
loop counters run from reset to read, so that, not the live `bypass=`, is the
state they were measured under: a run read after `bypass ... off`, or sorties
with and without a bit and no reset between them, keep the bit there and
`espfly_diag.py` gives no go/no-go. Each bit puts one addition of this section
back to 14bcfc6's code; one load and a branch where it sits:

| Bit | Off | On (14bcfc6) |
|---|---|---|
| `gtc` 0x01 | the compensation | no accumulate, block step, subtraction, latch or log (the gyro is 14bcfc6's); off again: restarts unseeded, 0 until the next ground re-zero (~1 s at rest) |
| `mki` 0x02 | the in-flight learning | no airborne / epoch inputs, 14bcfc6's integral branch (twoKiAir untouched; a re-zero meanwhile zeroes the integral when it comes back) |
| `rec` 0x04 | the recorder's thermal / exposure fields | `imuTemp`, `gtcCorrX/Y`, `mkiBiasX/Y`, `flowGain`, `flowSat` written 0; no post-flight re-zero poll (a flight's residual is not filled meanwhile) |
| `flow` 0x08 | the NVS fit mode and the exposure gate | the plain fit without the gate or its bookkeeping (the camera loop of 14bcfc6), from the next frame; NVS untouched |

No toggle for E, F, G: IRAM or per frame, < 0.5 loops/s, and a flag test in
IRAM would spend part of the slack before the next 256 B heap step. None for
`vbatMonitorTask`'s growth either (layout, review round below, 3).
`espfly_diag.py bypass ...` sends the command; `auto prof` gives no go/no-go
while any bit is on or was on since `auto prof reset` (`bypassSeen`; it
prints the rate for the bisect). Every bit reverts to code that flew on
14bcfc6, so a bit left on is not a hazard, but the build as designed (and the
go/no-go) is `bypass=0x00` and `bypassSeen=0x00`.

**Review round (fixer, 2026-10-14).** Three findings on 819a4ce / 39e80e2:

1. *The expectation was too high.* 819a4ce's own gains (A +25..30, B +2..3, C
   +2..3) sum to +29..36: 950-957 from 921.4, not 952-960. By the line model
   (0ed68c9's ~23 line-equivalents cost 961 - 921.4 = 39.6 loops/s, 1.72 each)
   the 4-5 left cost 6.9-8.6: 952-954. And C saved ~0.6 line-equivalents (+1),
   not +2..3: `flightRecorderSample` grew 3177 -> 3325 B, GCC inlined `sat16`
   (~72 B with its `lrintf` call) at all 28 sites (14bcfc6: 23), and its
   thermal block alone (~486 B, ~15 lines) ran every 10 ms. 819a4ce alone:
   ~950-957, central ~953, most likely just under 955.
2. *`auto prof` labelled the rate with the mask at the read*, not the one the
   counters ran under: `bypassSeen` (above) and `espfly_diag.py` (no go/no-go
   when it is not 0; `test_prof_bisect_bit_cleared_before_the_read`).
3. *The flight summary's larger store grew `vbatMonitorTask`*, a flash task on
   core 0 every 1 ms: 395 B (14bcfc6) -> 423 B, the part inside its critical
   section (interrupts masked on core 0) 239 -> 267 B, ~8 -> ~9 lines. Same
   source: `flights[16]` at 148 B pushed `vCurMin` ... `rMin` past the narrow
   load / store offsets (extra `l32r`, wide `l32i` / `s32i`) and the 148 B
   stride (136 before) takes an extra `addx4`. No toggle can cover it (layout);
   it stays in `all on`'s residual. Not changed: moving the battery window
   before `flights[]` needs a `FLIGHT_RTC_MAGIC` bump (the RTC summaries are
   lost once) and leaves the stride; ~+1 line a ms in a priority-1 task, <= ~1.7
   loops/s by the model, probably less.

**What the review round changed** (6cf394c; no behaviour change, the records
and the gyro are byte for byte the same):

| | 819a4ce | Now |
|---|---|---|
| `sat16` in `flight_recorder.c`, stabilizer, 100 Hz | inlined at 28 sites: `flightRecorderSample` 3325 B (104 lines every 10 ms) | `noinline`: 1921 B + `sat16` 62 B (~62 lines): -42 lines every 10 ms, ~4.2 line-equivalents |
| `usecTimestamp`, 1 kHz: the IMU data-ready ISR (IRAM), the stabilizer 3x a loop, the camera hold in the controller | flash, 15 B across 2 lines | IRAM, 15 B with no literal (its call to `esp_timer_get_time` relaxes to a `call8`, and so does the ISR's call to it: the ISR -4 B, its literal -4 B): `.iram0.text` +8 B, inside the slack, 0 heap |

Each `sat16` site is still the caller's `mul.s` and then the same compare /
`lrintf` / `sext` (30 `mul.s`, 0 `madd.s` / `msub.s`, 7 `utrunc.s` in both
builds' `flightRecorderSample`), so the int16s cannot differ;
`test_flight_recorder_host.c` (flight_recorder.c on the host with stubs, 6000
drawn samples: NaN, +-inf, -0, the int16 limits, ties after the scale, the
toggle, flow present / absent, flights starting and ending) pins the records'
FNV-1a hash from 39e80e2 (0xE92337A1EB681F4C; the ring 0x68472DAEA6FCF5B5); a
mutant with `lroundf`, -32768, 1 for NaN or a plain cast changes it.
`systemIsArmed` (19 B + two literals, also 3x a loop in flash) did not fit
too: the IRAM's slack is the `.iram0.text_end` gap minus its 16 B prefetch pad
(39e80e2: 45 - 16 = 29 B; both together crossed the 256 B step, -256 B of
heap). Now 37 - 16 = 21 B are left.

**Expected** (the line model above, not measured): 819a4ce ~953 (950-957);
the recorder's `sat16` ~+6..8 (4.2 line-equivalents); `usecTimestamp` ~+2..3
(1-2 lines a ms). This build: **~958-964, central ~961**, so >= 955 likely but
not certain (the layout shift alone is +-3). Both levers also remove costs
14bcfc6 had (its 23 inlined `sat16`, `usecTimestamp` in flash), so the build
can read above 14bcfc6's 961, and `bypass all on` should read ~5-11 above 961
if the toggles cover the regression. `camProcAvgUs` plain back to
~60.0-60.8 ms, lite / robust within ~0.2 ms of it. What each toggle should
give back now: `gtc` ~3-5, `mki` ~1-2, `rec` ~0.5-1 (the 5 thermal `sat16`
calls are ~100 B now), `flow` ~0 in plain. Bisect if under 955 (bench A4b):
each bit's gain is that addition's cost now. Next levers, not built: the
fit's unroll (above; frees core 1); `systemIsArmed` inline in `system.h` or in
IRAM (the latter costs the 256 B step: heap ~19.1 KB); the I2C read's command
link built in a static buffer instead of ~9 heap calls a sample (a cost
14bcfc6 has too; I2C_LINK_RECOMMENDED_SIZE(3) = 408 B of internal RAM, so
heap ~19.0 KB unless it lives in PSRAM); the battery window before
`flights[]` (above, 3).

**Cost** (int-sdkconfig, ESP-IDF 5.0.7):

| | 14bcfc6 | 0ed68c9 | 819a4ce | review round |
|---|---|---|---|---|
| `.iram0.text` (`.iram0.text_end`: the gap to the 256 B step, 16 B of it the prefetch pad) | 129599 (189) | 129743 (45) | 129743 (45) | 129751 (37): `usecTimestamp` +15, the ISR's call and literal -8, alignment +1 |
| `.dram0.data` / `.dram0.bss` | 31480 / 87560 | 31528 / 87848 | 31528 / 87872 | 31528 / 87880 (+4: `benchBypassSeen`) |
| heap start (internal) | 0x3fcc1000 | +336 B | +360 B (+24: the block's sum and count, the air tick, the thrust tick seen, the published airborne re-zero count, the toggle mask) | +368 B (0x3fcc1170) |
| `.flash.text` / `.flash.rodata` | 734467 / 186032 | 751299 / 188224 | 752687 / 188560 (+1.4 KB: `bypass`, the latch and block step) | 751375 / 188576 |
| `sensorsTask` | 1703 B | 2285 B | 1891 B (the per-sample addition 70 B) | 1891 B |
| `estimatorComplementary` / `sensfusion6UpdateQ` | 132 / 726 B | 144 / 1032 B | 176 / 1006 B (no calls added) | the same |
| `flightRecorderSample` (100 Hz) | 2781 B | 3177 B | 3325 B | 1921 B + `sat16` 62 B |
| `vbatMonitorTask` (1 kHz, core 0; critical section) | 395 B (239 B) | 423 B (267 B) | 423 B (267 B) | 423 B (267 B), not changed |

`heapInt` expected ~19.4 KB with the Mac linked, as 0ed68c9 (>= 19 KB; the
preferred 20 KB would need ~600 B back: `gyroTcLog`, 184 B, could move to
PSRAM). Host tests: `test_gyro_tempco.c` (the block rate against the
per-sample pipeline, 91 checks), `test_sensfusion6_ratecorr.c` (0ed68c9's
twoKiAir hash), `test_flow_exposure.c` (robust and lite hashes),
`test_flight_recorder_host.c` (the records' hash, pinned from 39e80e2),
`test_espfly_diag.py` (`bypass` keys, `auto prof` with a toggle on,
`bypassSeen` with the bit cleared before the read); every C program and the
python suite: 0 failures.

## 20. The hold's gain scale with the scene distance (user request 2026-10-14, afternoon: no drift at take-off)

The drone drifted within seconds of every take-off on 2026-10-14 (afternoon f3-f5, the evening f6-f9) and the
user caught it each time (the PALM / COLLISION / K_RANGE / HOLD_NO_CAM / RATE / TILT / ATT_ERR endings were the
grabs). Target: < 0.3 m in the first 15 s and < 0.5 m over 45 s of a plain hover, then moves toward people.
Analysis (the designer, read-only, scratchpad `drift/designer`): a closed-loop replay of take-off and hover on the
logged frames through `camera_hold.c`, which reproduces the logged hold to 0.007 deg rms, with one addition the
earlier replays lacked: the controller holds the attitude *estimate*, and in the air the accelerometer reads only
the rotor drag (-25 mg per m/s) plus constant offsets, so the Mahony correction Kp (alpha - e) (Kp 0.384 at 960
loops/s) moves the estimate away from the true attitude. A trim error tilts the drone up to about twice its size in
the first seconds, and an error in the estimate's reference at lift-off never decays by itself: drift or the hold
has to burn it off (the hold's lean integrated over time = error / Kp, ~1 deg s per 0.4 deg).

### 20.1 The causes (today's 5 flights that reached a real hover: afternoon f3/f4/f5, p2b f7/f9)

| Cause | Share | Evidence |
|---|---|---|
| 1. The hold is too weak at today's scene distance | ~40 % | its gains do not depend on D; at D 2.55-3.46 m KI / D is 0.57-0.78 deg/m: 0.15 m of steady offset per 0.1 deg of error, ~3 s time constant; it never saturated (max 1.18 deg of 2.5) |
| 2. Roll trim error (afternoon only) | ~25 % | atrim roll -0.20 against a level of -0.60 +- 0.17: 0.4-0.5 m right by HOVER + 3 s; -0.65 (p2 f7) stayed within 0.2 m laterally |
| 3. The pitch reference jumps between the floor and the air at lift-off | ~25 % | facc x 0.012 g came from the 10-10 hover accelerometer reading, which holds the body's own hover tilt (~4.5 mg) besides the vibration offset (5-7 mg; inertial check 3-8): the in-air reference sits 0.3-0.5 deg nose-down of the lift-off one, uncorrected in the climb (the forward hold is off there, 19.5); the climb needs +0.38 deg more nose-up than the hover in 13 of 14 flights; roll (facc y 0 = its vibration offset) shows no shift (-0.06 +- 0.15) |
| 4. Level scatter between flights | ~10 % | the residual thermal gyro bias after gtc, before mki learns it (estimate minus accelerometer -0.03..-0.27 deg); f3's -0.47 was the knee-44 clip (knee 55 since) |
| 5. The lift-off kick | partly cancels 3 | torque trim pitch 800 against a need of ~357 (roll 250 against ~650): +0.2..+0.7 deg nose-up over the first 0.5 s |
| 6. f8 (a tilted take-off spot, rest roll +1.47 deg) | unresolved | no replayed fix brings it under ~0.6 m: take off from a flat, hard spot |

Not causes: 9040655 (the forward hold off in TAKEOFF; with the fixes, reverting it makes today worse, 0.32 -> 0.50
m: the climb's floor parallax changes sign with the scene, today's contraction -0.3..-0.7 m against the references'
expansion +0.3..+0.8 m), the hold's engage delay (0.18-0.78 s after lift-off; at lift-off 0.68 vs 0.65 m), its
saturation (none), gtc / mki at take-off (< 3 cm), and the analysis tool's 0.368/m climb correction (a reporting
error: f8's -105 cm forward is -67 cm raw, ~-26..-45 real).

### 20.2 What changed (F1; `camera_hold.h/.c`, `camera_flow.c`, `usb_diag.c`, `imu_level_cal_store.c`)

- `camHoldInput_t.gainScale` (appended): kp = KP s, ki = KI s; every integral clamp (the step, the reference
  take, the snap) is `CAM_HOLD_I_LIMIT_DEG / ki`, so the integral's contribution stays <= 1.5 deg; the output
  limit (2.5 deg) and both slews (6 deg/s) are unchanged. 0, NaN, +-inf and <= 1 mean 1; at most
  `CAM_HOLD_GAIN_SCALE_MAX` 3.0. A change of s first rescales `iFwd` and `iLat` by s_old / s_new (KI s i, the
  learned lean, is continuous) and the reference bookkeeping with them (`lost' = f lost + (1 - f)(p - r)`): a
  snap after the scale changed in its epoch (a YAW forgets G, below) restores the lean the epoch began with.
  `camHoldState_t.gainScale` (appended) is the scale applied last. s = 1 is bit-identical to c928b6d.
- `camHoldGainTarget(latch, autoState, mvState, cvFlags, cvGain, kmax)` (pure): autoState not TAKEOFF / HOVER / LAND
  clears the latch (a new sortie at ARM, a manual flight); an mvState of kind MOVE (any state, a vertical MOVE too)
  sets `translated` for the rest of the sortie; a turn (`camHoldTurnActive`: a YAW running, paused or settling)
  forgets the latched G; otherwise cvFlags bits 0 and 2 (the observer airborne, its G tracked) with cvGain 150..5000
  (15-500 px/m, the observer's own gMin..gMax) latch G = cvGain / 10. Target clamp(80 / G, 1, kmax) (80 px/m is the
  observer's gDefault, D 1.25 m: s = D / 1.25 m) in TAKEOFF and HOVER when kmax > 1, a G is latched and nothing
  translated; else 1. **LAND targets 1** (the latch kept; added in the review round, 20.8): the descent's floor
  parallax reads as forward motion, and the larger gain turned it into a larger creep. **Airborne** is not in the
  designer's text: on the ground the observer keeps the previous flight's last tracked G (afternoon f4 / f5: 18.5
  and 22.2 px/m, the landing's view) and flags it tracked through the next spin-up; latching it would have applied s
  2.5 from the spin-up on a stale view. `camHoldGainSlew` moves s at most 2.0 per second of frame time (dt clamped
  to 0.3 s).
- `camera_flow.c holdUpdate()` (the camera task, ~15 Hz, flash): the target from the recorder snapshot it already
  reads, the slew, `hin.gainScale`. Nothing on the 1 kHz path; no IRAM.
- NVS `imucal` / `holdkmax` (float, 1.0..3.0; absent: 2.5; 1.0 = off), read at boot.
- USB `holdk show | set <1..3> | save | clear` (`set` is live until `save`; `clear` erases the key and applies 2.5;
  set / save / clear refused while autonomy is active and with the motors running). `show`: `kmax src s target G
  D translated hold`. `espfly_diag.py holdk ...` passes it through.
- Not done (optional in the design): s in the recorder or the flight summary. The record (124 B) and the CSV are
  unchanged; read k from the dump as 80 / (cvGain_dpm / 10) on rows with cvFlags & 5 = 5, before the first MOVE and
  outside LAND (`tools/hold_band.py` rebuilds it, 20.8); `holdk show` gives the live s on the bench.

### 20.3 The USB settings (after flashing, or alone on c928b6d: they halve the drift, 0.96 -> 0.44 m)

| Command | Confidence | Why |
|---|---|---|
| `facc set 0.007 0.000`, `facc save` | MEDIUM | the in-flight x vibration offset is ~5-7 mg (inertial check 3-8); 12 mg held the hover tilt too. -5 mg = +0.29 deg on the in-flight pitch reference: the lift-off compensation of cause 3. The replay is flat for 6-8 mg (15 s max 0.24 m each) |
| `atrim set -0.60 0.00`, `atrim save` | roll HIGH, pitch MEDIUM | roll: estimate frame -0.62 (sd 0.17), accelerometer frame -0.57; -0.45..-0.75 stays at 0.25-0.28 m. Pitch 0.00 = today's estimate-frame hover level (-0.30) + 0.29 for the facc change: valid only with facc 0.007 and torque trim 250 / 800; +-0.1 deg costs +0.05 m |
| `atrim learn off` (confirm) | HIGH | |
| `trim show`: 250 800, kept | MEDIUM | today's kick is part of this calibration; set the trim to the measured need (~500 / 350) only together with atrim pitch +0.15 (or facc -0.002). After a battery reseat that moves the hover `pitchRateI` mean outside +-1000, re-tune the trim and the pitch compensation together |
| `gtc show`: 0.018 -0.060 0, knee 55, kept | HIGH | f4-f9 post-flight residuals PASS |
| `mki show`: 0.04, kept; vzK 160000, kept | HIGH | |
| `holdk show`: kmax 2.5 (new build only) | | `holdk set 1` (not saved) is c928b6d's hold exactly: the fallback if the drone rocks |

Expected with these and F1, replayed on today's 5 flights: mean |x| 0.13 m at HOVER + 3 s and 0.14 m at lift-off +
8 s, the 15 s maximum 0.24 m (per flight 0.24 / 0.25 / 0.15 / 0.19 / 0.37); a take-off like f8 (a tilted spot)
still ~0.95 m.

### 20.4 Verification (host)

- `test_camera_hold.c` (+49 checks): the identity hash (no references) unchanged; scale 0 / 1 / NaN / -1 / +-inf /
  0.5 / 1 - 1e-7 bit-identical to scale 0 over 20000 steps with references, snaps, feedforward, fwdOff, dropouts and
  landings (every state field); 7 clamped to 3.0; scale 2.5: the output 2.5 x scale 1's below the limits (worst
  1e-5), the integral clamped at 0.3 (1.5 deg), the output limit 2.5 and the slew 6 deg/s kept; a scale change 1 ->
  2.5 -> 1 -> 1.7 -> 3 -> 1.2 in a hover, engaged and coasting: |output change| <= 1e-6 deg; a snap after the scale
  went 2.5 -> 1 in its epoch restores the epoch start's lean; `camHoldGainTarget` (IDLE, ARMED, untracked, not
  airborne, G 32 / 40 / 50 / 100, out of range, LAND (1, the latch kept: 20.8), KILLED, MOVE until the next ARM, YAW
  until a fresh tracked G, kmax 1 / NaN / 0.5 / 9) and the slew (1 -> 2.5 in 12 frames of 67.5 ms). Mutants (no
  rescale, no bookkeeping, the old clamp, scale 0..1 accepted, no YAW forget, no MOVE latch, no clear) each fail it.
  A differential test against c928b6d's `camera_hold.c` (scratchpad `drift/implementer/diff`): 3,000,000 steps with
  every input, scale 0 / 1 / NaN, -O0 and -O2: 0 differing.
- `test_hold_replay.c` (f): the 2026-10-14 sorties (afternoon f3-f5, p2b f6-f9, morning f4) through
  `camHoldGainTarget` / `camHoldGainSlew` from the logged autoState / mvState / cvFlags / cvGain, as
  `camera_flow.c`: kmax 1 bit-identical; kmax 2.5: every output finite and within 2.5 deg; s 1 until the
  observer's first tracked G in the air, 1.21-1.42 s after lift-off (the designer's 0.7-1.9), then in HOVER before
  a motion s 1.62-2.50 (means 2.03-2.50; f5's view 2.03, the afternoon's far wall 2.49-2.50); s 1 within 1 s of
  every MOVE (4 of 4) and of every LAND entered at s > 1 (4 of 4: f5, f6, f8, f9; 20.8).
- `test_interactive_sim.c`: kmax from the command line (default 2.5; `run_c_tests.sh` runs both). At kmax 1 the
  output is byte-identical to c928b6d's sim. At 2.5 every existing check passes (the palm tests fly at kmax 1 in
  both runs, below), but `testFarMoves` passes there only for its one pinned seed set: over 16 seed sets the far
  moves end other than DONE 7.6 % of the time against 5.1 % at kmax 1 (`testFarMovesSeeds`, 20.8), and kmax 1.8 /
  2.0 fail the pinned set itself (seed 4112, MOVE -150 ends PROGRESS), 2.2 its 0.20 m/s comparison: **kmax 2.0 is
  not a drop-in alternative to 2.5**; the fallback is `holdk set 1`. New: the observer's G = f / (D (1 + eD)) with
  its tracked bit from 1.3 s after lift-off and only within 15..500 px/m; the attitude filter's coupling as an
  option (`scen_t.coupled`: d' = 0.384 (drag v / g
  - e) + I, I' = 0.02 (drag v / g - e), the lean error a setpoint offset); `testTakeoffGrid` (20 seeds x lean error
  +-0.3 deg per axis x D 1 / 2 / 3 / 4 m x eD -0.3 / 0 / +1.0; |x| = the distance from the take-off point):

| D, eD | HOVER + 3 s | lift-off + 15 s | lift-off + 45 s | s at + 15 s | hold 0.3-1.5 Hz p2p (rms), noise / no noise |
|---|---|---|---|---|---|
| 1 m, -0.3 / 0 / +1.0 | 0.34 -> 0.34 / 0.34 / 0.28 | 0.37 -> 0.37 / 0.36 / 0.30 | 0.35 -> 0.35 / 0.34 / 0.28 | 1.00 / 1.04 / 1.56 | 0.48 / 0.48 / 0.87 (0.098) / 0.22 |
| 2 m | 0.51 -> 0.48 / 0.40 / 0.32 | 0.56 -> 0.51 / 0.43 / 0.35 | 0.54 -> 0.49 / 0.40 / 0.31 | 1.16 / 1.57 / 2.50 | 0.44 / 0.64 / 1.12 (0.144) / 0.18 |
| 3 m | 0.63 -> 0.49 / 0.41 / 0.40 | 0.73 -> 0.54 / 0.45 / 0.44 | 0.74 -> 0.50 / 0.41 / 0.39 | 1.65 / 2.27 / 2.50 | 0.63 / 0.88 / 0.99 (0.130) / 0.11 |
| 4 m | 0.70 -> 0.50 / 0.48 / 0.70 | 0.88 -> 0.55 / 0.52 / 0.87 | 0.94 -> 0.51 / 0.47 / 0.90 | 2.19 / 2.50 / 1.52 | 0.82 / 0.91 / 0.89 (0.124) / 0.07 |

  (kmax 1 -> 2.5, the three eD after the arrow; the sim's +-0.3 deg per axis is a larger error than the USB trims
  leave.) Checked: better than kmax 1 on all three at D >= 2 m wherever the observer tracks (8 cells; D 4 m eD +1.0
  is D_obs 8 m, beyond its gMin: s stays 1 until the drone drifts in, never worse); never worse at D 1 m (<= +0.01
  m). **Deviation from the designer's "hold p2p < 1 deg"**: with the camera noise of the logged hovers (0.03 /s per
  frame; the logged k = 1 hold's 0.3-1.5 Hz band: rms 0.02-0.19 deg, median ~0.05, the sim 0.045-0.056) the band
  carries that noise times s: 0.44-0.91 deg peak-to-peak at the design gains, 0.87-1.12 where s is 1.6-2.5 x the
  scene's need (eD +1.0), raw peak-to-peak 0.6-1.5 deg. It is noise, not an oscillation: the same cells without the
  camera noise read 0.07-0.22 deg (checked < 0.3 in every cell, eD +1.0 at D 1 m included: 0.22); the band rms is
  checked < 0.16 deg (0.144 max) and its peak-to-peak < 1.25. These are the sim's numbers, whose camera noise (band
  rms 0.045-0.056 at s 1) is quieter than the logs (0.02-0.12 today, 0.19 on 2026-10-13 pwmA1 f7): the flight check
  uses the log replay instead (20.8). A real loss of damping in the sim is a limit cycle of the hold over its +-2.5
  deg (band peak-to-peak 4.0-5.0 deg, rms 0.39-1.4; 20.8). The position cost of the noise is ~1 mm. The mission at
  both kmax (16 seeds): moves p95 15 -> 15 %, max 23 -> 24 %, lean 1.23 -> 1.23 deg, every LAND by the Mac. A LAND
  without a move creeps forward 0.07-0.17 m with LAND at s 1 (kmax 1: 0.06-0.18; 0.13-0.23 when 5679a6b kept s in
  LAND: the descent's fake expansion through the larger gain, 20.8).
- The palm tests (`testPalmCatch`, `testPalmHardContacts`, `testPalmFalsePositives`) fly at kmax 1 in both runs.
  Their checks are pinned at one seed set's measured values; at kmax 2.5 one marginal soft palm (seed 12111: v
  0.02 m/s, lift 1 cm, k 200, aRel ~0.12 g) cues at 0.56 s instead of 0.03 s (non-blind p95 0.52 -> 0.93 s) and
  one pulled-away palm lands at 1.13 m/s (limit 1.10). Both are chaos, not the gain: kmax 1.001 (a 0.1 % gain
  change) flips the same cue; over 8 seed sets kmax 1 and 2.5 give the same palm statistics in 7, and kmax 1 alone
  misses these pinned bounds in 6 of the 8 (scratchpad `drift/implementer/dbg`).
- The designer's coupled replays re-run (`replay2.py` unchanged; `scan_main.py`, `run_refs.py` reproduce
  `scan_main.txt` / `refs.txt` exactly) and F1 as built (scratchpad `drift/implementer/replay_f1.py`: k from the
  tracked airborne G, latched, slewed 2 /s, rescaled; the window ends at the first MOVE / YAW): today's 5 flights,
  15 s maximum as flown 0.96, F1 alone 0.52, the USB settings alone 0.44, **both 0.23** (0.25 / 0.22 / 0.14 / 0.19
  / 0.36; HOVER + 3 s 0.14, lift-off + 8 s 0.15, lateral |R| at HOVER + 3 s 0.10), kmax 2.0 / 3.0: 0.26 / 0.23;
  f8 0.98. The 12 reference flights (c928b6d's configuration): F1 never worse by > 0.02 m (0 of 12), mean 0.91 ->
  0.75; with the facc / atrim-pitch compensation 0.63.
- `test_espfly_diag.py`: `holdk` (the reply's keys against `usb_diag.c`, the CLI passes `holdk`, `set 1`, `save`
  through; an ERR is exit 1).
- Every C program 0 failures but two that already failed at c928b6d with the 2026-10-14 CSVs in `cal/` (unchanged
  by this work, neither links `camera_hold.c`): `test_gyro_tempco` (22: its replay fits the raw-gyro model to the
  logged zero, and today's flights logged it after the compensation) and `test_mahony_replay` (3: afternoon / p2
  f4 roll innovation 0.41 -> 0.46 with the learning; p2b f8, flown with twoKiAir 0.04 from a tilted spot, against
  the twoKiAir 0 replay: 0.211 > 0.2 deg). The python suite: OK.

### 20.5 Cost (int-sdkconfig, ESP-IDF 5.0.7, against c928b6d)

`.iram0.text` 129751 -> 129751 (`.iram0.text_end` 37 -> 37; the 805 IRAM symbols and their sizes identical);
`.dram0.data` 31528 -> 31544 (+16: s, its target, kmax), `.dram0.bss` 87880 -> 87888 (+8: the latch): the heap
start +24 B (0x3fcc1170 -> 0x3fcc1188; `heapInt` ~20.04 KB from 20.06, >= 19.5 required); `.flash.text` 751375 ->
753831 (+2456: `camHoldStep` 1834 -> 2293 B, `camHoldGainTarget` 238, `camHoldGainSlew` 128, `holdUpdate` 525 ->
588, `cameraFlowInit` +40, `cmdHoldK` 754, the store's three functions), `.flash.rodata` 188576 -> 189016; the app
1105568 B (was 1102656). Per frame (~15 Hz, camera task): one target, one slew,
two multiplies. The 1 kHz flash functions moved (the link order): their cache-line footprint is 527 lines before
and after (some +1, some -1); the 19.11 line model expects <= +-3 loops/s from such a shift (A4 >= 940 on the bench,
c928b6d measured 964.6 / 959.3).

### 20.6 Not built (the designer's no-go items)

- **F2, a lift-off pitch feedforward** ff0 exp(-t / 2.6 s): facc 0.007 does the same job (15 s max 0.24 either
  way) without 1 kHz code; it would duplicate facc and depend on the kick.
- **F3, freezing the accelerometer correction from the RAMP to lift-off + 1 s**: 0.24 -> 0.28 m (the RAMP's
  correction pre-adapts the estimate toward the in-air reference); helps only f8. Rule instead: take off from a
  flat, hard spot, rest tilt within +-1 deg.
- **F4, reverting / modifying 9040655**: with the fixes 0.32 -> 0.50 m today (the references 0.75 -> 0.46): the
  climb's floor parallax changes sign with the scene.
- **F5, earlier or stronger engagement**: 0.68 vs 0.65 m; no saturation.
- **F6, removing the torque-trim kick in firmware**: 0.24 -> 0.28 m unless the pitch compensation grows ~0.2 deg;
  later, the flight summary could print the torque need (trim + hover rate I).
- **F7, the analysis tool's 0.368/m climb correction**: not flight code; read the next flights with the
  velocity-constrained climb or the raw camera.

### 20.7 Risks and what to check

- Over-gain when G is too small (D over-estimated) at a close real scene: the linear model has k 2.5 at a true
  0.5-0.8 m marginal (damping <= 0.04 at 0.25 s of latency). Bounded by kmax 2.5, a tracked G only, s 1 for good
  from the first MOVE (an approach shrinks D while G is stale: every move flies as today), a YAW forgetting G, and
  `holdk set 1` on USB.
- At the first MOVE s ramps to 1 at 2 /s (~0.75 s from 2.5) with the integral rescaled: the lean is continuous; the
  P term changes by (s - 1) KP |latF|, small in a hover.
- After a MOVE the hold is today's again (weak at far scenes); a later extension could rescale s by the expansion
  the move measured.
- The hold's 0.3-1.5 Hz jitter grows with s: from the camera's noise alone the log replay of today's 5 hovers at s
  2.0-2.5 reads band rms 0.08-0.27 deg (per axis), peak-to-peak 0.44-1.62 deg, rms / s 0.04-0.11 deg. Rocking is a
  visible oscillation, a band peak-to-peak > 2.2 deg or a band rms > 0.4 deg (20.8; `tools/hold_band.py`
  computes them with s from the dump; autonomy-bench.md J3 3).
- LAND flies s 1 (20.8): its creep is c928b6d's.
- Far-scene MOVEs (D >= ~3 m): s falling to 1 in the move's first 0.75 s raises the sim's PROGRESS / SHORT ends
  from 2.7 to 6.7 % of the accepted moves (20.8). At D <= 2.5 m (flight C) the sim shows no change. A slower fall
  (0.75 /s) would fix it; not built before flights A-C.
- Bench and flights: autonomy-bench.md J.

### 20.8 Review round (2026-10-14 evening: the verifier's three findings)

**1. The flight check for rocking came from the sim, whose camera is quieter than the logs.** Section J's "band
rms > 0.2 deg or peak-to-peak > 1.5 deg" would have called 3 of today's 5 hovers rocking with F1 and no
oscillation. The coupled log replay (it reproduces the logged hold's 0.3-1.5 Hz band at s 1, e.g. pm-f3 roll / pitch
rms 0.057 / 0.097 logged, 0.056 / 0.097 replayed), HOVER + 2 s to the window's end:

| Flight | s (F1) | band rms roll / pitch, logged s 1 -> F1 | band p2p roll / pitch, F1 | rms / s, F1 |
|---|---|---|---|---|
| pm-f3 | 2.50 | 0.057 / 0.097 -> 0.155 / 0.269 | 0.85 / 1.62 | 0.108 |
| pm-f4 | 2.50 | 0.048 / 0.020 -> 0.104 / 0.053 | 0.59 / 0.37 | 0.042 |
| pm-f5 | 2.04 | 0.095 / 0.080 -> 0.203 / 0.188 | 1.06 / 0.93 | 0.099 |
| w-f7 | 2.27 | 0.033 / 0.040 -> 0.082 / 0.065 | 0.44 / 0.31 | 0.036 |
| w-f9 | 2.24 | 0.094 / 0.112 -> 0.195 / 0.233 | 0.75 / 1.12 | 0.104 |

What a real loss of damping looks like (the sim, 1500 take-off-and-hover runs over D 0.7-4 m, kmax 1.4-3.0, the
observer's D exact or 3.5 m, the camera noise on): the band peak-to-peak is bimodal, 0-2.0 deg in 1438 runs and
2.0-3.0 in 9, then 4.0-5.0 deg in the 53 runs whose hold swings over its whole +-2.5 deg (a limit cycle; the
hold at ~3 x or more of the gain its scene needs). **Rocking = a visible oscillation, or band peak-to-peak > 2.2 deg,
or band rms > 0.4 deg**: it flags all 53 limit cycles and 5 other runs (the worst run without a limit cycle 2.58 /
0.42), and none of the replayed hovers above (max 1.62 / 0.27). A 45 s hover's ~40 s window raises the noise's
peak-to-peak over the replays' 6-16 s by ~20 % (peak-to-peak / rms of band-limited noise: median 4.8 over 12 s,
5.7 over 40 s, at most 7.1 in 200 synthetic windows): pm-f3 at s 2.5 would read ~1.6-1.9 deg, under 2.2. rms / s (the noise normalised by the gain) is printed but not acted on: a lightly damped
hold at 2.5-4 x its need reads 0.09-0.15 in the sim, today's hovers 0.04-0.11 and 2026-10-13 pwmA1 f7 (s 1, a
noisier camera) 0.19, so it cannot tell the two apart. A camera as noisy as pwmA1 f7 would read ~0.45 rms at s
2.5 and trip the rule; `holdk set 1` for the next flight is then the conservative answer. The check, with s
rebuilt from the dump as `camera_flow.c` computes it: `tools/hold_band.py <csv> <flightId> [--kmax 1]` (new;
standard library only; prints OK / NOISY / ROCKING, exit 1 on ROCKING; on the logs at s 1: 2026-10-14 afternoon
f3-f5, p2b f7 / f9, morning f4, 10-13 tiger f1, 10-12 mission60 f1 OK, 10-13 pwmA1 f7 NOISY;
`tools/tests/test_hold_band.py`: the filter, s against `test_camera_hold.c`'s cases, the verdicts, a synthetic
dump end to end).

**2. LAND at s 1** (`camHoldGainTarget`, the only firmware change of this round). 5679a6b kept s through LAND (the
forward axis is on there) and no replay covered the descent. The verifier's replay through touchdown on 7
reference LANDs without a MOVE (k 1 matches the logged hold in LAND to 0.03 deg rms on 12-m60 and 13-tig1): s
kept added +0.10..+0.26 m of forward creep on 5 of the 7 (13-tig1, D ~2.9 m like today: +0.26 m forward, +0.16
lateral, hold pitch peak -2.34 deg against -1.52; the other two entered LAND at s 1.24 and 1.00), the descent's
floor parallax through the larger gain, the same effect that
took the forward hold out of TAKEOFF (19.5). With LAND targeting 1 (the latch kept; s slews to 1 at 2 /s, ~0.75 s
from 2.5, with the integrals rescaled so the lean is continuous) the same replay gives +0.00..+0.02 m on 6 of the
7 (13-tig1 +0.26 -> +0.01) and 12-calib1 +0.11 -> +0.09 (a 1.7 s LAND entered 0.14 m apart from the k 1 run: the
state it entered with, not the gain in LAND; scratchpad `drift/fixer/landreplay_f.py --land1`); the sim's LAND creep
without a move 0.13-0.23 -> 0.07-0.17 m (kmax 1:
0.06-0.18). Take-off and hover are unchanged (every hover replay ends at LAND entry: 20.3's 0.24 m stands).
Tests: `test_camera_hold.c` LAND -> 1 with G kept, tracked G in LAND latched but 1, LAND at kmax 3 -> 1 (122
checks); `test_hold_replay.c` (f) s 1 within 1 s of LAND entry and for good on the 4 LANDs entered at s > 1 (59
checks); a mutant without the LAND rule fails 3 + 1 of them. No IRAM, nothing on the 1 kHz path.

**3. Far-scene MOVEs end PROGRESS / SHORT more often at s > 1.** `testFarMoves` pins one seed set; over 16 sets
(800 moves per kmax, the far scene D ~3.5 m, 0.30 m/s; `testFarMovesSeeds`):

| kmax | not DONE | PROGRESS | SHORT | refused | PROGRESS + SHORT of the accepted |
|---|---|---|---|---|---|
| 1.0 | 41 (5.1 %) | 19 | 2 | 20 | 2.7 % |
| 1.5 | 33 (4.1 %) | 20 | 1 | 12 | 2.7 % |
| 2.0 | 47 (5.9 %) | 34 | 4 | 9 | 4.8 % |
| 2.5 | 61 (7.6 %) | 44 | 9 | 8 | 6.7 % |
| 3.0 | 64 (8.0 %) | 48 | 9 | 7 | 7.2 % |

Every run still ends in a LAND by the Mac and a touchdown, never KILLED. Paired by seed, kmax 1 vs 2.5:
PROGRESS / SHORT only at 1: 10, only at 2.5: 42. The cause is not the start position (the verifier's reading of
one trace): with s taken to 1 2.5-5 s before the MOVE (the same start, observer D 3.48 m) the same seeds give
20-21 not DONE (PROGRESS 10-11, SHORT 1-2). It is s falling while the move starts: 2 /s (as built) 61, 6 /s 76,
instant 81, 0.75 /s 3.5 % of 480
moves (D 3.5 m sweep), 0.5 /s 17 of 800, s kept through the move's run 9. The hold at s 1 is weak for a far move
(kmax 1's own 19 PROGRESS), and the drop disturbs the move's start. Against the scene distance (8 seed sets,
240-480 moves each): at D 1.5 / 2.0 / 2.5 m no PROGRESS or SHORT at either kmax, the refusals within 3; at 3.5 m
PROGRESS + SHORT 13 -> 31 (kmax 1 -> 2.5).
So flight C (a person 2.5 m ahead) is unaffected; far-scene moves (D >= ~3 m) are not, and kmax 2.0 is no
remedy (it fails the pinned set). Candidate, not built before flights A-C: s falls at 0.75 /s while a MOVE runs
(PROGRESS + SHORT at 3.5 m 13 / 31 -> 12 of 480). Since a 0.30 m/s approach lowers the scene's need D / 1.25 by
at most 0.24 /s, s at 0.75 /s keeps the over-gain ratio s / need at or below its value at the move start (if that
is under 3.1, the hover's own stability edge) and never above c928b6d's once s is 1. Review it with the designer
before the first far-scene MOVE, together with `AUTO_MV_PROG_MARGIN_*`.

Scratchpad `drift/fixer/`: `holdband.py` (the pandas draft of `tools/hold_band.py`), `check_f1band.py` (its
verdict on the F1 replays), `simosc*.c` (the
over-gain sweeps), `simfar*.c` (the far-move rates, the transition variants, the D sweep), `landreplay_f.py`.

---

## 21. One-step long MOVEs and a diagonal straight from take-off (user decisions 2026-10-14, evening)

The user decided that Claude, not a tool planner, judges distances; that an approach is **one** continuous MOVE;
and that the drone can fly diagonally straight from take-off when Claude has planned the move from the floor photo.
The drone checks only what its camera measures (the scene ahead, the view, the battery, the air time); Claude
judges the distance to the person. Nothing here adds a landing: every refusal leaves the drone hovering in the wait,
with the reason. Two phases: **phase 1** is the Mac alone (it works on b6a54da today: at most 1.5 m per axis,
`MOVE_FWD_MAX_M` 1.50, the plan sent by the Mac); **phase 2** is one firmware build plus the Mac's constants
(forward 4 m, the 18-byte TAKEOFF), flashed only after the gates below and the user's OK. Not included (separate
items): the soak timer, the palm probe, LOW_BATT hysteresis, the post-move gain re-latch, the log sideways mapping.

### 21.1 The limits per MOVE (`auto_motion.h`, `autoMvRangeOk`)

| | b6a54da | now |
|---|---|---|
| forward (toward the scene ahead) | 150 cm | **400 cm** (`AUTO_MV_FWD_MAX_CM`) |
| right / left, back | 150 cm | 150 cm (the camera does not see behind); sideways also <= 0.5 x D (`TOO_WIDE`) |
| sqrt(right^2 + forward^2) | (212 cm) | **400 cm** (`AUTO_MV_LEN_MAX_CM`; (150, 371) is 401.2: `RANGE`) |
| up / down | 50 cm | 50 cm; **up != 0 with \|forward\| > 150 cm: `RANGE`** (the camera reads vertical motion partly as forward) |

`autoMvRangeOk(r, f, u)` (flash) holds them for a MOVE and for the take-off's plan (21.4). The speeds are unchanged
(0.20 m/s toward the scene, 0.30 m/s sideways and back, the image-rate caps 0.12 /s sideways and 0.15 /s expansion);
forward 1.5 / 2 / 2.5 / 3 / 3.5 / 4 m take 9.5 / 12 / 14.5 / 17 / 19.5 / 22 s. The AIR rule is unchanged (profile
+ 3 s settle + 8.5 s LAND within the air left): a 4 m move needs 33.5 s, refused with 33.0 s left and accepted with
34.0 s (after ~86 s of the 120 s clock). The scene floor is unchanged (the drone `TOO_CLOSE` at 0.50 m for k 0.6,
the Mac 0.60 m) and binds below D ~5.5 m. The largest forward move (whole cm, rounded down so the end keeps the
floor; the spec's table rounded to the nearest cm, 0.01 m higher):

| D (m) | 2.0 | 2.5 | 3.0 | 3.5 | 4.0 | 4.66 | 5.0 | >= 5.5 |
|---|---|---|---|---|---|---|---|---|
| the Mac (0.60 m); the take-off's plan (21.9) | 1.02 | 1.43 | 1.85 | 2.28 | 2.71 | 3.29 | 3.59 | 4.00 |
| the drone (0.50 m), a Mac MOVE | 1.12 | 1.54 | 1.97 | 2.41 | 2.85 | 3.43 | 3.74 | 4.00 |

The camera measures forward travel by the scene's expansion, so a move much longer than D cannot be measured: in a
furnished room (D 2.5-3 m) one move is ~1.4-1.9 m. A refusal states the largest allowed move; Claude decides.
`autoMotionMove` also refuses `TOO_CLOSE` a forward move whose end would be nearer than 0.30 m by the sideways
guard's distance (`AUTO_MV_LAT_GUARD_D_MIN_M`, 21.2); the scene floor already keeps >= 0.41 m, so it never binds.

### 21.2 The guards for long moves (`auto_motion.c`; every MOVE)

- **B1, the sideways margin in metres at the reference's distance now.** PROGRESS when |progLat - rLat| x (dLat0 -
  sFwdM) > margLatM, margLatM = max(0.25 m, 0.15 m + 0.30 x |lateral m|), the same in RUN and SETTLE; sFwdM = s x
  max(fwdM, 0) / len (`fwdUnitPos`, from `horizAt` at the lag; a back move keeps the start distance). It replaced
  `margLat` / `margLatS` (radians at the start distance) in their slot. The camera's lateral reading is the
  integral of v_right / D_now, so its error in metres is the radians times the distance now: 5 of the 7 PROGRESS
  stops of 2026-10-14 were 0.15 m of sideways hover wander on straight forward moves, and near the end of a long
  approach D_now / D0 is 0.3-0.5, where the radian margin was 2-3x tighter in metres. The spec's distance floor
  max(., 0.30 m) is enforced at acceptance (`TOO_CLOSE`, 21.1) instead of in the 1 kHz step (IRAM).
  `test_auto_motion.c`: a 3 m forward move from D 4.66 m, 0.20 m of sideways wander at its end passes and 0.30 m
  stops it PROGRESS; at its start 0.24 m passes, 0.26 m stops it.
- **B2, the live front guard.** At acceptance a forward move gets progFwdMax = 0.6 x ln(D_low / 0.40 m)
  (`AUTO_MV_FRONT_STOP_M`; others: never). In RUN and SETTLE (FLIGHT, the camera on this epoch) measured progress
  past it STOPs the move with the new end **`AUTO_MVE_FRONT` 7** (the step's one guard branch calls
  `autoMotionStop(PROGRESS)`, which names it FRONT, in flash). An accepted move's target is at most 0.6 x ln(D_low
  / 0.50), 0.134 below it, so an ideally tracked move never trips it (tested at the TOO_CLOSE boundary: DONE); at k
  0.6 it trips with the scene 0.400 m ahead, + ~0.13 m of braking, whatever the move's length (the PROGRESS margin's
  ahead side grows with 30 % of the move: ~0.48 log units beyond a 3.3 m move's target). Sideways and back moves,
  YAW and DRY never end FRONT. No move of the long-move grid (below) reached it. (That bound holds with D exact: the
  guard reads the same D the move was planned with, so a D 10 % low takes it nearer; on the drone's 0.50 m floor the
  largest moves came 0.34-0.40 m from the scene at k 0.6, on the Mac's 0.60 m 0.41-0.47 m, why the take-off's plan
  now flies the Mac's floor: 21.9.)
- **B3, the forward PROGRESS margin is unchanged**, and a **frozen-view rule** is added (not in the spec): a move
  whose lagged forward reference is past 0.10 (ln) while the measured forward progress is under 0.25 x it is STOPPED
  PROGRESS (`AUTO_MV_FROZEN_REF` / `_FRAC`). The margin's behind side grows with 30 % of the move: in the sim a
  frozen 3.3 m approach from D 4.7 m (and 4 m from 5.5 m) tripped it only ~10 s in, and the hold's push carried the
  drone into the scene ahead in 2 of 4 seeds (the front guard reads the same frozen camera). With the rule the
  frozen 2.5 / 3.3 m approaches stop 3.6 s in, the wall >= 4.60 m away (1.5 m: 3.0 s, >= 3.38 m). The hold drives
  the measured progress to the reference, so a tracking move reads near 1 x it: in the grid's completed moves the
  ratio's lowest was 0.25; 0.06 instead of 0.10 stopped 3.4 % of the grid's genuine moves (0.10: 1.1 %).
- **B4, the sideways image-rate cap at the forward end**: `AUTO_MV_W_LAT_MAX x max(D - max(F, 0), 0.05) x len /
  |R|` (the start D before). A diagonal of 1.5 m right and 3.0 m forward at D 4.66 m: the largest sideways image
  rate 0.053 /s; over 2449 accepted diagonals toward the scene 0.1183 /s at most (the cap 0.12; with the cap at the
  start D 0.162 /s, 1.35 x, past the observer's 1.5 px gate).
- **The SHORT end's "off its line"** (not in the spec): the settle's cross-track error is now in metres at the
  distance now, as B1 reads it (at the start distance a long forward move's 0.06 m read 0.17 m at its end, D0 /
  D_now 2.8). With it the grid's PROGRESS + SHORT at D <= 4.7 m fell 14.7 % -> 4.9 % (0.75 /s, before the
  escalation), and 21.3's escalation took it to 1.5 %.
- **B5, the battery: no firmware change.** The 53400 refusal at acceptance stays, BATT_LOW at 3.00 V lands in
  place; a move of <= 22 s + 3 s keeps the T_hold headroom (+65-91 counts/s) under 57000. A long move accepted late
  in a charge can end BATT_LOW partway (safe); the Mac's duration check (21.5) handles the usability.

### 21.3 The hold's gain during a move (`camera_hold.h/.c`, `camera_flow.c`; the camera task, flash)

- `CAM_HOLD_GAIN_SLEW_MOVE_PER_S` 0.75: `camHoldGainSlew(s, target, dt, rate)` takes the rate; `camHoldGainRate`
  gives 0.75 /s once the latch is translated (a MOVE ran) and the sortie is in HOVER, else 2 /s. LAND still goes to
  1 at 2 /s (20.8 #2's creep fix); the hover before the first MOVE is unchanged. Evidence: 20.8's sim at D 3.5 m,
  PROGRESS + SHORT 31 -> 12 of 480; every gym move (D 4.2-4.7 m) stopped in RUN while s fell 2.5 -> 1 within 0.75 s.
  Stability: an approach at 0.20 m/s lowers the scene's need (D / 1.25 m) by at most 0.16 /s, under 0.75 /s, so
  s / need never rises during a move.
- **The escalation was needed** (H5's trigger): with 0.75 /s alone the long-move grid ended 4.9 % of the accepted
  moves at D <= 4.7 m PROGRESS / SHORT (gate 3 %). `camHoldGainMove` (after `camHoldGainTarget`): in HOVER, while a
  MOVE with a part toward the scene runs, pauses or settles (`autoMvOut_t.dLowM` > 0 -> `autonomyHoldRef_t.dLowM`,
  one float each), the target is clamp(D_ref_now / 1.25 m, 1, s at the move's start), D_ref_now = D_low x
  exp(-refFwd / 0.6): the gain the reference's scene distance needs, never more than at the start (approaching only
  lowers it). Two restrictions the spec did not have, both from the frozen-view sims: it applies only **once the
  camera has seen the move go** (the reference past 0.01 and the measured forward progress >= 0.3 x it), and a move
  the camera does not see (`untracked`) keeps the plain target (1) at 2 /s: in a frozen view the hold's push on the
  reference grows with s (the frozen 1.5 m approach went 2.83 m with s kept, 1.91 m at 0.75 /s, 1.87 m with this
  rule; b6a54da's 2 /s 1.79 m). Sideways and back moves, kmax 1, no latched G: the target unchanged.
- **The grid's rates (21.6) are the first MOVE's** (normally the planned one): once a MOVE ran, the latch is
  translated, s returns to 1 in HOVER, and `camHoldGainMove` escalates only from s > 1 at a move's start, so every
  later MOVE flies at s = 1, as the kmax-1 column: PROGRESS + SHORT + FRONT 12.0 % at D <= 4.7 m (18.6 % at 4.7 m,
  35.6 % at 5.5 m) against 1.5 % for the first. A second long approach should expect an occasional SHORT or PROGRESS
  (both hold where they end); the post-move re-latch (not built) would lift it (2026-10-15 review, 21.9).
- `tools/hold_band.py` models the move rate and the move gain (from `mvRefFwd_e4`, D_low from the latched G, the
  camera assumed to see the move; `--moves` keeps the motions in the window, to the end of HOVER); on the 13
  long-move sim runs: 12 OK, 1 NOISY (D 4.0 F 1.5: rms / s 0.152), no ROCKING. `test_hold_replay.c` (f) on the
  2026-10-14 sorties: s at or below its start's in the first MOVE, 1 after it (4 of 4), 1 in LAND (4 of 4), kmax 1
  bit-identical.

### 21.4 TAKEOFF with a planned move (phase 2)

- **Wire** (`auto_proto.h/.c`): `AUTO_TAKEOFF_PLAN_LEN` 18 = the 12 bytes of before + i16 planRightCm, i16
  planFwdCm, u16 0 (reserved). `autoProtoParseTakeoff` takes 12 (plan 0, 0) or 18; any other length or a nonzero
  reserved field is malformed (`RANGE` detail 0 from the link: not processed, no take-off). `autoProtoPackTakeoff`
  writes 18 only with a plan, so plain sorties still fly an older drone. The packet: 16 + 18 + 16 = 50 bytes
  (`AUTO_RX_BUF_LEN` 64; `auto_link.c`'s static assert). The link puts the plan into `cmd.moveRightCm / moveFwdCm`
  (`autoCmd_t` unchanged, 28 B).
- **State machine** (`auto_fsm.h/.c`): `autoFsm_t` gains planRightCm, planFwdCm, planPending, a pad (+6 B, after
  `vRefMs`). `takeoffCmd` (flash): a nonzero plan must pass `autoMvRangeOk` and the sortie must have a hover wait
  (actWaitMs, or the NVS `autowait`, nonzero), else `RANGE`; every accepted TAKEOFF latches the plan (one without a
  plan clears it). Rows 25-26 moved into `waitStart` (flash, noinline: once per sortie): with no wait, LAND
  SORTIE_DONE; else the wait starts and a pending plan goes to `autoMotionMovePlan` with every acceptance check on
  the live D (NO_DIST, TOO_WIDE, TOO_CLOSE, NO_VIEW, BATT, AIR; TOO_CLOSE at the Mac's 0.60 m floor,
  `AUTO_PLAN_MIN_FRONT_CM`, since 21.9: the Mac cannot check the plan before the take-off). A refusal leaves `mv.why`
  (AUTO_STATUS mvWhy) and the
  drone hovering in the wait, as after a refused Mac MOVE; AIR cannot refuse it (at any wait's start at least the
  120 s clock minus the pre-wait limit is left, more than a 4 m move's 33.5 s). A STOP before the wait cancels it:
  in HOVER `motionCmd`'s STOP branch, in ARMED / TAKEOFF the unconsumed-command switch, now `cmdOutside` (flash,
  noinline; it was inline in `autoFsmStep`'s IRAM: moved to pay for B1/B2 and the frozen rule, 21.6). LAND, KILL,
  HB_LOST or any landing before HOVER never reach row 26. A Mac MOVE before the wait stays `NOT_WAITING`, during
  the plan `BUSY`; STOP ends it `STOP_CMD`. AUTO_STATUS is unchanged: running = mvSeq >= 1 and mvState RUN /
  PAUSED / SETTLE; refused = waiting, mvSeq 0, mvWhy != 0.
- **When it starts:** the wait's start, STABLE 1 s plus the take-off photos: with `--photos 1` HOVER + 1.80 s,
  TAKEOFF + 7.36-7.43 s (`test_auto_sortie.c`; 3 photos HOVER + 2.40 s); `test_autonomy_glue_host.c` (the firmware
  glue, DRY): the wait at HOVER + 1.64 s, the plan RUN on the wait's first AUTO_STATUS (mvSeq 1), DONE 15.50 s after
  TAKEOFF. By HOVER + 1.0 s the observer's G is within ~1 % of its settled value. The closed-loop sim with the G
  transient G x (1 + 0.25 e^(-t / 0.4 s)) from the HOVER entry (16 runs, forward 1.5 / 2.5 m at D 4.7 m): the plan
  ends -13..+8 % of its command (mean -2.1 %), within -4.2..+2.6 % of the same Mac MOVE sent 0.4 s later; started at
  the HOVER entry the same move ends +14..+32 % long (mean +22.0 %): the rejected start. The spec's +-10 % per run is
  checked as +-15 % per run with the mean within +-10 %: the sim's far 1.5 m moves themselves err up to 15-20 %
  (`testFarMoves`, from a settled hover), the plan-specific part is the +-5 % match. The plan's frame is the take-off
  heading (yaw lock); the start point carries the take-off drift (Claude's clearance rule, 21.7).

### 21.5 The Mac (`tools/espfly_auto.py`)

(2026-10-15, section 22.4: the Mac no longer LANDs on its own in the air, its height refusals are one-sided, and
the take-off's long-wait rules are advice.)

- Constants: `MOVE_FWD_MAX_M` 4.00 (1.50 in phase 1: the test cross-checks it against the header's
  `AUTO_MV_FWD_MAX_CM`, or 150 without it), `MOVE_HORIZ_MAX_M` 1.50, `MOVE_LEN_MAX_M` 4.00, `fwd_max(top=
  MOVE_FWD_MAX_M)`; `move --forward` 0..4.0, `--right/--left/--back` 0..1.5. `move_range_refusal` mirrors
  `autoMvRangeOk`.
- `move`: the floor (0.60 m at k 0.6) and TOO_WIDE as before; a refusal gives the largest forward move now
  (`fwd_max_m`, `D_m`). `--person-gap G` (Claude's estimate of the end distance to the nearest person): refused
  under 1.0 m (`PERSON_MIN_M`, the user rule of 2026-10-13); since 21.9 it needs `--est` (the distance to that
  person now) and is also refused when the drone's own travel at k down to 0.6 with the live D could end under
  1.0 m (`person_gap_low`; the reply gives the largest forward part that keeps it, `fwd_max_m`, and `person_low_m`);
  G and `--est` are logged (`person_gap` event, with D) for calibration. The battery's duration: with the AUTO_STATUS
  vbat (under load) below 3.25 V, seconds left =
  (vbat - 3.00) / 0.005; a move whose profile + 3 s does not fit is refused with the longest along the same line
  that does (`longest_m`, `battery_s`); `--batt-ok` overrides it. `move_timeout` = profile + 15 s (37 s for 4 m;
  moves up to 1.5 m keep the old, longer 0.20 m/s basis).
- Results: PROGRESS with 90-110 % measured along the move and at most 0.15 m + 10 % of it off its line is `ok:
  true`, `reached: true`, note "reached (stopped at the end by the guard)" (21.9: past 110 % "overshoot by X m",
  further off "off its line by Y m", `ok: false`, take a photo; `along`, `off_m`); FRONT: "stopped: the scene ahead
  got close; holding, take a photo" (`MV_END_NAMES`, the hint).
- `sortie --plan-right/--plan-left/--plan-forward/--plan-back M` (the 21.1 limits; range checks only before the
  take-off, D is unknown; a rest voltage under 3.78 V is warned on; needs `--then wait` and a wait; `--person-gap`
  >= 1.0 m). `--plan-via drone` (phase 2): the 18-byte TAKEOFF; the sortie replies at the wait's start with the
  take-off photo(s) and the plan: `running` (mvSeq, D) or `refused` (why, the hint, D, `fwd_max_m`: the plan flies
  the Mac's floor, 21.9), or `unknown` if AUTO_STATUS shows neither within 1 s. A plan with `--person-gap` needs
  `--est` and goes `--plan-via mac` (21.9; refused before the take-off otherwise). An older drone refuses the 18 bytes
  malformed: "this build has no planned take-off: --plan-via mac" (no take-off happened; the sortie disarms).
  `--plan-via mac` (the default until phase 2 flies; it works on b6a54da): the daemon sends the MOVE through
  `move`'s checks (the 0.60 m floor, the battery) on the first AUTO_STATUS in the wait with the photos on disk (the
  logs: 0.33-0.46 s after the wait's start), and replies at its ACK. The sortie's timeout adds `move_timeout`. A
  client gone during the plan: LAND.
- `wait-move [--photo] [--timeout S]`: blocks until the running motion ends (at once with none): end, `reached`,
  travelled and target (m), D, the margins (to the target, the scene ahead), and with `--photo` a photo.
- `approach` stays in the code but leaves the runbook (21.7). (Removed from the code on 2026-10-15: 22.9.)
- Tests (`test_espfly_auto.py`, `DaemonLongMoveTest`, parametric on `MOVE_FWD_MAX_M` so phase 1 passes too): the
  limits and profiles, the 18-byte payload (and 12 without a plan), 3.5 m at D 6, refusals with the largest move,
  the person gap and its event, the battery refusal and `--batt-ok`, PROGRESS at 95 % reached / 50 % not, FRONT,
  wait-move with a photo, the plan via the drone (running, refused at D 2.0), the old drone's message, the plan via
  the Mac on a b6a54da-like drone (12-byte TAKEOFF, the MOVE within one status of the wait), its floor refusal, the
  pre-take-off refusals, a client gone in the plan (LAND); the golden `takeoff_plan`.

### 21.6 Verification and cost

- Host: `test_auto_motion.c` 253 checks (RANGE, the floor sweep: 4021 forward moves over F 5..400 cm x D 0.5..8 m x
  k 0.6..2.5, the nearest true end 0.501 m; AIR, profiles, B1-B4, the frozen rule), `test_camera_hold.c` 148 (2.5
  -> 1 in 2.0 s translated in HOVER, 2 /s in LAND, unchanged before a move, `camHoldGainMove`), `test_auto_fsm.c`
  2144 (the plan: latched, RANGE, started on the row-26 step for photoN 0 and 3, the refusals, STOP in the pre-spin
  / climb / HOVER before the wait, never after LAND or KILL, cleared by a plain TAKEOFF, NOT_WAITING / BUSY),
  `test_auto_proto.c` 315 (12 / 18, 17 and 19 and a nonzero reserved field rejected, round trip, golden),
  `auto_link_host` 250, `test_autonomy_glue_host.c` 400 (`plan-dry`), `test_auto_sortie.c` 83, `test_hold_replay.c`
  59; the Python suite 258 tests. `run_c_tests.sh`: every test passes but `test_gyro_tempco` (33 of 139) and
  `test_mahony_replay` (4 of 443), which fail the same way at b6a54da (today's CSVs).
- **The long-move grid** (`test_interactive_sim.c`, `testLongMoves`): D {3.5, 4.0, 4.7, 5.5} m x F {1.5, 2.5,
  3.0, 4.0, capped by the Mac's floor: 2.28 / 2.71 / 3.33 / 4.00} x k {0.6, 1.0, 1.6} x drag {0.15, 0.30, 0.45} x
  16 seed sets, at kmax 2.5 and 1 (1872 runs each):

| D | kmax 2.5: PROGRESS + SHORT + FRONT of the accepted | kmax 1 |
|---|---|---|
| 3.5 m | 0.3 % (1 of 288) | 5.2 % (15 of 288) |
| 4.0 m | 0.9 % (4 of 432) | 7.9 % (34 of 432) |
| 4.7 m | 2.4 % (14 of 576) | 18.6 % (107 of 576) |
| 5.5 m | 1.6 % (8 of 498) | 35.4 % (168 of 475) |

  At D <= 4.7 m 1.5 % (19 of 1296; gate 3 %, and kmax 1 12.0 %). **These are the first MOVE's** (normally the
  planned one): every later MOVE starts at the hold's gain scale 1 and flies as the kmax-1 column (21.3, 21.9 #7):
  expect the occasional SHORT or PROGRESS on a second long approach. The weakest cell is the short move at the far
  scene, D 4.7 m F 1.5 m: 8.3 % (3 PROGRESS, 9 SHORT of 144; kmax 1 41.7 %). FRONT 0, never KILLED or another end;
  the nearest true end 0.41 m (gate 0.40); the controller's height error <= 0.084 m through the 17-22 s moves
  (gate 0.10), the true height's excursion <= 0.263 m against 0.249 m in a 25 s plain hover from the same point
  (the height estimate's own offset); refused 78 (kmax 1: 101), every one NO_DIST at D 5.5 m (no tracked scene
  distance there in those seeds). `testFarMovesSeeds` (D ~3.5 m, 800 moves): PROGRESS + SHORT 1.8 % of the accepted
  at kmax 2.5 (6.7 % in 20.8), 2.1 % at 1.
- **The gym's moves replayed** (`test_motion_replay.c`, 2026-10-14 gym f3 / f4 / f5, the logged progress through
  this build's guards; the gym's 0.20 m/s profile pinned): b6a54da's sideways margin stopped 2 of the 4 at the
  logged record (f3 -0.154 m, f5 -0.164 m at the start distance); this build does not stop them (-0.145 / -0.127 m
  at the distance now, under 0.25). f4 (right 62 forward 113 at D 4.96 m) stops PROGRESS at the same record in both:
  its measured forward 0.240 against the reference's 0.120, past the unchanged forward margin 0.117 (B3: the hold
  ran ahead, not the sideways wander). No move stops earlier than b6a54da's guards would.
- **Cost** (int-sdkconfig, ESP-IDF 5.0.7, against b6a54da; the review round's own in 21.9): `.iram0.text` 129751 ->
  129695 (-56: `autoFsmStep`
  4042 -> 3902 with rows 25-26 and the unconsumed-command switch moved to flash, `autoMotionStep` 2611 -> 2667 for
  B1 / B2 / the frozen rule, `autonomy.c` `stepFull` 1336 -> 1348 publishing `dLowM` into the hold reference (one
  4 B store a loop), `horizAt` +2, `autoMotionIdleOut` +1); `.iram0.text_end` 37 -> 93 B, the same 256 B step. `.dram0.bss` +40 B (the heap start 0x3fcc1188 -> 0x3fcc11b0: `heapInt` ~20.0 KB from 20.0-20.1, >= 19 KB).
  `.flash.text` +1232 (`camHoldGainMove` 327, `cmdOutside` 193, `autoProtoParseTakeoff` +171, `holdUpdate` +105,
  `autoMvRangeOk` 81, `autoMotionMove` +77), `.flash.rodata` +8. Nothing new in flash runs every loop (the per-move
  guards are in `autoMotionStep`'s IRAM; the gain is per camera frame; the plan once per sortie); the layout shift
  moved the 1 kHz flash functions' footprint by one 32 B line (the 19.11 line model: within its +-3 loops/s). Bench:
  A4 >= 955 loops/s, `heapInt` >= 19 KB.

### 21.7 Claude's procedure (runbook, phase 2; phase 1 the same with `--plan-via mac` and forward <= 1.5 m)

(2026-10-15: what to do when the drone reports something, and when to LAND, is section 22.5; the planning steps
below stand.)

1. The floor photo at the take-off spot, facing the target: the path within +-35 deg of the heading (half the field
   of view is 38 deg), else turn first (or take off and `yaw`).
2. Estimate the distance E and the bearing B (the photo ruler once it exists).
3. The end gap: to a person >= 1.0 m + 0.15 x E; + 0.3 m until the session's 20 s trim check has passed; to an
   object >= 0.6 m. **The k term (21.9):** toward a person the drone may fly F' = D (1 - ((D - F) / D)^(1/0.6))
   instead of F (k down to 0.6: 3.84 m for F 3.0 m from D 4.7 m); plan F so that E - F' >= 1.0 m too (E = D = 4.7 m:
   F <= 2.84 m, not 3.0); the Mac refuses it otherwise with the largest F (`fwd_max_m`).
4. The move: R = (E - gap) x sin B, F = (E - gap) x cos B, |R| <= 1.5 m, F <= 4 m. **In a room where no long move
   has yet measured k** (the end distance by photo: k = ln(D0 / D_planned_end) / ln(D0 / D_photo_end), 21.9), keep
   F <= 1.5-2 m; longer only after one long move there measured k >= 0.7 (below 0.6 a 4 m move reaches the wall).
5. At least 0.5 m clear of furniture on each side of the path (0.7 m before the trim check).
6. A diagonal's sideways part toward the scene comes out short by about D_avg / D0: 87 % at 1.13 m forward from D
   4.66 m, ~70 % at 2.5 m forward.
7. `sortie --mode flight --confirm-flight --height 100 --photos 1 --wait-s 60 --plan-right R --plan-forward F
   --plan-via drone`; **toward a person** `--person-gap <E - |move|> --est E --plan-via mac` (the Mac checks the gap
   against the drone's travel at k 0.6 with the live D; the drone cannot: 21.9).
8. Read the hover photo while the drone travels; `stop` if the path is wrong (it brakes in ~0.13 m).
9. `wait-move --photo`; judge the end photo.
10. A refusal that says D limits the move: fly the largest allowed (`move --forward <fwd_max_m>`), photo, judge
    again; or turn toward a deeper background. A second long move flies at the hold's gain scale 1 (21.3): expect the
    occasional SHORT or PROGRESS there (12 % at D <= 4.7 m in the sim against 1.5 % for the first).

Flight acceptance (a gym-like room, D 4-5 m, `stop` ready): a 20 s plain hover (the trim check); a planned 1.5 m
forward with no person (DONE, +-0.15 m by photo); a planned 3 m toward an empty wall (DONE, the end distance within
15 % by photo, `hold_band.py` OK; k from the end photo; and 21.9's height checks: `cvGain` before and after it, no
`vzV` / `cvV` oscillation in the move's last seconds and the hover after it); a diagonal of 0.8 m right and 2.5 m
forward (DONE, >= 0.55 m sideways); only then a person approach with the gap rule (and its k term). Follow-ups, not
built: the gain following the move's distance beyond the
escalation, the post-move re-latch, the log sideways mapping.

### 21.8 Risks and what to check

- The frozen-view rule and the escalation are sim-tuned (0.10 / 0.25, 0.01 / 0.3): a real slow-tracking camera
  (low texture, k well under 1) could stop a genuine move PROGRESS early. Check every PROGRESS in the dump (`mvProg*`
  against `mvRef*`) on the acceptance flights; a stop with the measured forward under 0.25 x the reference while the
  photo shows motion is this rule.
- The take-off drift carries into the plan's start point (no re-anchoring: the climb's forward drift cannot be
  measured); Claude's clearance rule covers it.
- A long move late in a charge can end BATT_LOW partway (safe, lands in place); the Mac's duration check refuses
  it below 3.25 V under load unless `--batt-ok`.
- The weakest grid cell (D 4.7 m, F 1.5 m: 8.3 %) is a short move toward a far scene: expect the occasional SHORT
  there; a SHORT holds where it ended.
- (21.9) Below k 0.6 nothing on the drone notices: a long move at a far scene reaches the wall (k 0.4: 0.15 m, k 0.3:
  through it, in the sim) where the 1.5 m cap left more than 1 m; the runbook keeps forward moves at 1.5-2 m in a
  room until a long move there has measured k >= 0.7 by photo.
- (21.9) The camera height observer's gain follows the measured forward progress: exact at k 1, (D0 / D)^(1 - k)
  off otherwise (the speed read up to 2.9 x at k 0.6, down to 0.58 x at k 1.6, in HOVER, the sim); the first long
  flights check `cvGain`, `vzV` and `cvV` around the move (bench K4.3).
- (21.9) Later MOVEs fly at the hold's gain scale 1: the kmax-1 rates (12 % PROGRESS + SHORT at D <= 4.7 m).

### 21.9 Review round (2026-10-15): the person gap's k term, the camera height gain through a move, the plan's floor

Seven findings of the safety review of 148af89 (`scratchpad/review/verifier-safety`), re-checked; all real, fixed or
documented here.

1. **The person gap left out the drone's own travel error (major).** `--person-gap` was checked against 1.0 m at
   the move's nominal end; the floor (and a2b's own 0.73 for a wall plus floor) allows k down to 0.6, where the drone
   flies D (1 - ((D - F) / D)^(1/k)): E = D = 4.7 m, the runbook's gap 1.70 m, F 3.0 m: 3.84 m, 0.86 m from the
   person (the review's sim: at k 0.6 19 of 22 runs under 1.0 m, the nearest 0.52 m; k 0.73 10 of 24). With chained
   1.5 m moves the error stayed on the last short step; one long move puts all of it at the person. Now (Mac,
   `espfly_auto.py`): `--person-gap` needs `--est`; `person_gap_low` = min(gap - overshoot, est - the most the move
   with the overshoot can bring the drone nearer a person in the photo, within +-38 deg of the heading: its length
   when it points there, a back move none), the overshoot `fwd_overshoot` = the travel at K_LOW from the live D minus
   F; refused under 1.0 m with the largest forward part that keeps it (`fwd_max_m`; 2.84 m for the review's case,
   0.91 m for 1.0 m at D 4.0 with a 1.4 m gap). A planned move toward a person goes `--plan-via mac` (the drone cannot
   check it: refused before
   the take-off with `--plan-via drone`, and without `--est`). Runbook 21.7 steps 3 and 7.
2. **The camera height observer's G did not follow a level move (major).** The observer turns vertical image
   motion into speed with the G = f / D it last took, and takes G only with vertical motion; the logged moves kept
   `cvGain` within 0.2 px/m while the reference reached 0.07-0.51. So through a long move toward the scene HOVER's
   vertical loop, closed on camVz, read the speed D0 / D_now too fast: 3.4-4.2 x at the end of the largest moves at
   k 1, 8-11 x at k 0.6 (flown: <= 1.66; b6a54da's own worst ~2.5 at k 1, ~4.2 at k 0.6). Confirmed in the real
   observer (`test_cam_vz_obs.c testForwardGain`: an approach G 24 -> 81.6 px/m with a 0.1 Hz bob under the
   tracker's excitation need: the speed's slope 2.99 against the true speed; 1.34 hovering at the end distance).
   **Fix (firmware):** the frames carry the interactive hover's measured forward progress ln(D0 / D) of the
   sortie's MOVEs (`autoMotionFwdLn`: `autoMotion_t.fwdLnBase` + the running move's; `autoFsmOut_t.fwdLn`; the glue
   `camVzGlueSetFwdLn` in `vzRun`, copied into every `camVzObsFrame_t.fwdLn`), and the observer's update uses G x
   exp(fwdLn - fwdLnAtG), `fwdLnAtG` the progress when the G in use was taken (a tracked G, or the yaw restart that
   made the guess, which keeps the scaled G). Exact at k 1 (slope 1.31 against 1.34), (D0 / D)^(1 - k) off
   otherwise (k 0.6: 1.99); a bob after the move re-takes G at the distance now and the scale starts again there
   (no double correction). The reported gain (D = f / G for the motions, `cvGain`) is the tracker's, unchanged;
   nothing changes while fwdLn stays (bit for bit outside moves: every replay unchanged).
   **The sim** (`test_interactive_sim.c`, `scen_t.gLatch`): camVz's part below ~1 s read x G_true / G_used, G taken
   after 2 s of >= 0.03 m/s rms vertical excitation (the climb, a bob, a vertical move, LAND) or, the review's
   pessimistic tracker, never in HOVER. `testLatchedG` (the Mac's largest move at D 3.5-5.5 m, k 0.6 / 1 / 1.6, 3
   drags, 8 seeds, 288 runs each):

   | | the gain following the move (this build) | without it (148af89) |
   |---|---|---|
   | G after 2 s of excitation | 0 not a clean Mac LAND; height error <= 0.084 m; speed read 0.59-2.91 x | 1 (k 0.6, PALM); 0.134 m; up to 11.5 x |
   | no G in HOVER (the review's) | 0; 0.084 m; 0.58-2.80 x | 25 (all k 0.6: 22 PALM, 3 VZ); 0.325 m; up to 11.1 x |

   The long-move grid (21.6) now runs with the latched G (gLatch 1): the same rates (D <= 4.7 m 1.5 %, 19 of
   1296; D 5.5 m 1.8 %; kmax 1 12.0 % / 35.6 %), the nearest true end 0.41 m, the height error <= 0.084 m, the true
   height's excursion <= 0.266 m against 0.249 m in a 25 s plain hover, the speed read <= 2.78 x. The H11 / bench
   K4.3 flight adds `cvGain` before and after the long move and no `vzV` / `cvV` oscillation through its end.
3. **Below K_LOW (minor; documented).** The frozen-view rule reads ~1 (the hold drives the measured progress to
   the reference) and the front guard reads the same camera, so nothing detects k < 0.6, and a2b 14.8 recorded k
   0.27 in rendered arenas. `testBelowKLow` (3 drags x 8 seeds, the Mac's floor, nearest the scene; runs under
   0.40 m):

   | D | k 0.5: <= 1.5 m / <= 4 m | k 0.4 | k 0.3 |
   |---|---|---|---|
   | 3.5 m | 0.62 (0 of 24) / 0.31 (17) | 0.25 (3) / 0.15 (20) | 0.04 (6) / through the wall (11) |
   | 4.7 m | 1.63 (0) / 0.29 (23) | 1.16 (0) / 0.15 (18) | 0.31 (1) / through it (10) |
   | 5.5 m | 2.25 (0) / 0.27 (13) | 1.59 (0) / 0.14 (20) | 1.07 (0) / at the wall (14) |

   The runbook (21.7 step 4): forward moves of 1.5-2 m in a room until one long move there has measured k >= 0.7
   (k = ln(D0 / D_planned_end) / ln(D0 / D_photo_end) from the end photo, the acceptance flight's step 3).
4. **The plan was checked against the drone's 0.50 m floor only (minor).** The Mac's 0.60 m cannot run before the
   take-off, and with the observer's D 10 % low the drone's largest moves came 0.34-0.40 m from the scene at k 0.6
   (D 2.5-5.5 m, the review's `drone_floor_sim`). **Fix (firmware, flash):** the plan goes to `autoMotionMovePlan`,
   the floor
   `AUTO_PLAN_MIN_FRONT_CM` 60 (a Mac MOVE keeps the drone's 50); `testPlanFloorSim`: the plan at its largest at the
   observer's D (+-10 %), k 0.6, D 2.5-5.5 m, 3 drags x 16 seeds: 237 accepted, the nearest true end 0.43 m (per D
   0.45 / 0.47 / 0.43 / 0.45 / 0.46), never KILLED, 2 cm more TOO_CLOSE (210 of 210 below 4 m). The Mac's plan
   refusal gives `fwd_max_m` at that floor (`fwd_max_drone_m` is gone).
5. **'reached' had no upper bound and no cross-track check (minor).** A PROGRESS stop is reached only with 90-110 %
   measured along the move and at most 0.15 m + 10 % of it off its line (`move_reached`, `move` and `wait-move`);
   the review's case (1.13 m from D 4.66 m stopped at 137 %: 1.55 m of travel) is now `ok: false`, "overshoot by
   0.42 m ... take a photo"; 0.30 m off a 1.0 m move: "off its line by 0.30 m".
6. **Bench K4 step 1 never hovered 20 s (minor).** `--then land` lands once waiting; the trim check is `sortie ...
   --wait-s 25` (the default `--then wait`), then `land` after 20 s of hover (or HOVER_TIMEOUT at 25 s).
7. **The grid's gate covers the first MOVE only (minor; documented).** 21.3, 21.7 step 10, 21.8, bench K.

**Cost** (int-sdkconfig, against 148af89): `.iram0.text` 129695 -> 129763 (+68 B: `autoFsmStep` +28 for
`autoMotionFwdLn`, `vzRun` +12 for the store, the camvz task's `camVzTask` +12 (the frame's fwdLn) and
`camVzShadowStep` +12 (the observer state's 8 B more move the shadow's fields behind it)); b6a54da's 129751 + 12,
`.iram0.text_end` 25 B, the same 256 B step. `.dram0.bss` +16 B (`fsm` +4 `fwdLnBase`, the FSM output +4, the
glue's `camVzGlueFwdLnIn` 4; heap start 0x3fcc11b0 -> 0x3fcc11c0: `heapInt` ~20.0 KB, >= 19 KB). Flash: the
observer's frame update +236 B (the camvz task, per frame, as before), `autoMotionMovePlan` 29 B. Nothing new
runs from flash on the 1 kHz path. The observer state 2024 -> 2032 B (PSRAM).

**Tests:** `test_cam_vz_obs.c` 2881 checks (`testForwardGain`), `test_auto_motion.c` 270 (`testFwdLn`,
`testPlanFloor`), `test_auto_fsm.c` 2150 (the plan's floor), `test_autonomy_glue_host.c` 401 (plan-dry: the
glue's fwdLn ln 2), `test_interactive_sim.c` 123 (`testLatchedG`, `testBelowKLow`, `testPlanFloorSim`, the grid with
the latched G; its no-hand palm count leaves out the runs outside the design on purpose: 0 PALM in 8835), the Python
suite 265 (the person gap's k term, the plan via the Mac toward a person, the plan's floor,
the overshoot and off-line PROGRESS ends). `run_c_tests.sh`: every test passes but `test_gyro_tempco` and
`test_mahony_replay`, which fail the same at b6a54da (the 2026-10-14 CSVs).

## 22. Claude decides: hold and report (user decisions 2026-10-14 ~20:30-20:40)

The user, after the evening's flights: "I want YOU (Claude) to make the decisions; I don't want this to be a tool."
"Sometimes you give up and land; I don't want this. Take a photo and find a solution based on what you see. If
something in the software stops you from doing this, remove it." "Keep only: landing at the battery limit (as
agreed), the motor cut after a crash or flip, landing on Mac link loss, the k kill." "Don't crash on things while
attempting to find a solution." The MOVE / YAW refusal near the end of the battery counts as the battery limit.

So every rule that landed, stopped or refused only because something was unusual now holds the drone as well as its
sensors allow and reports an event with its evidence; Claude (the Mac) decides from a photo. What cannot be held
physically keeps a fallback that is safe: it sinks, it never climbs without bound. Commits 8684a61 (the controller),
b9b93cb (the motions), f55e329 (the controller after a contact), a2ad5bd (the state machine, the glue, AUTO_STATUS),
f7594ca (the Mac); the verification of the finished state and its fixes: 22.8; the user's decisions B (every LAND after
a contact is the blind one) and P (no approach planner): 22.9.

### 22.1 The inventory

KEEP-MINIMUM: the user's list. KEEP-PHYSICAL: what no hold can replace (on the ground, a runaway, a backstop no
battery reaches). CONVERT: now a hold and an event. REMOVE: gone.

| Rule (where) | Until 2026-10-15 | Now | Class |
|---|---|---|---|
| VBAT_LOW, BATT_LOW (3.00 V or T_hold > 57000 for 0.5 s), VBAT_DIP (FSM) | LAND | LAND | KEEP-MINIMUM (battery) |
| MOVE / YAW refused LOW_BATT / BATT near the end (auto_motion, Mac duration check) | refusal | refusal | KEEP-MINIMUM (battery) |
| TILT, RATE, FREEFALL, ATT_ERR, VZ_CUT, MOTOR_GUARD, NAN (FSM) | KILLED (cut) | KILLED | KEEP-MINIMUM (crash, flip) |
| HB_LOST (1 s), SESSION (FSM) | LAND | LAND | KEEP-MINIMUM (Mac link) |
| KILL_CMD (k) | KILLED | KILLED | KEEP-MINIMUM |
| APP_INPUT, APP_SILENT (the phone) | LAND | LAND | KEEP-PHYSICAL (the user's own hand on the stick, as the Mac's link) |
| LAND_CMD, DISARM (Claude, the user) | LAND | LAND | (a command) |
| TAKEOFF_TIMEOUT (no HOLD in 8 s), the SPIN / RAMP / lift-off checks (vert_ctrl) | LAND | LAND | KEEP-PHYSICAL (not yet a hover) |
| LAND_BACKSTOP, LAND_TIMEOUT | KILLED | KILLED | KEEP-PHYSICAL (a LAND that does not end) |
| HOVER_TIMEOUT (the wait's end) | LAND | the wait is information (`waiting`, waitLeftMs 0) | REMOVE |
| AIR_TIME (the 120 s air clock) | LAND | no clock; the controller's HOLD_TIMEOUT backstop 599 s | REMOVE |
| SORTIE_DONE (no wait, no photo) | LAND | the wait starts | REMOVE |
| the pre-wait clock (airMax, + the climb's extra) | LAND AIR_TIME | TAKEOFF_SLOW event, the wait starts | CONVERT |
| NOT_STABLE (no STABLE 1 s by HOVER + 2 s) | LAND | event, the wait starts, holdBits NOT_STABLE | CONVERT |
| CAPTURE_FAIL | LAND | event, the set consumed, the wait starts | CONVERT |
| HOVER_NO_CAM (the camera height out 2 s) | LAND | CAM_LOST: open loop under capQ; SINK after 5 s (at once without capQ), RESUME | CONVERT (SINK: KEEP-PHYSICAL) |
| HOLD_NO_CAM (vert_ctrl, 3 s) | LAND | 7 s backstop, 2 s behind the SINK | KEEP-PHYSICAL |
| MOVE_NO_CAM, MOVE_TILT (auto_motion) | LAND | the motion stopped (NO_VIEW / TILT), MOVE_STOPPED event; a YAW turns back once | CONVERT |
| the view lost in a plain hover | (nothing) | HOLD_NO_VIEW / VIEW_BACK events | (new report) |
| COLLISION in HOLD, or in the climb with the camera height (FSM) | LAND, blind | CONTACT: hush, motion stopped (a MOVE's hold anchor 0.25 m back the way it came), event (22.2) | CONVERT |
| any LAND later in a sortie that had a contact (Claude's, the battery rules, a SINK, the link, the phone, a loop gap, the controller's own) | the camera's LAND; blind only within ~2.5 s of the contact | the blind LAND, whatever its reason; the camera hold kept to the touchdown outside the contact's window (22.9) | (user decision B) |
| the TAKEOFF's planned move after an event before the wait (a contact, NOT_STABLE, ...) | never flown (the event landed) | not started (mvWhy PLAN_CANCELLED), the drone hovers in the wait; the Mac's `--plan-via mac` MOVE not sent | (kept as it was, 22.8) |
| COLLISION in the climb without the camera height | LAND, blind | LAND, blind | KEEP-PHYSICAL (the hover thrust not measured) |
| PALM (the palm cue) and the controller's support probe | LAND | gone: Claude LANDs on a hand it sees | REMOVE |
| LOOP_GAP | LAND | one: event, the motion stopped; the third within 1 s: LAND | CONVERT (3 in 1 s: KEEP-PHYSICAL) |
| K_RANGE, HOLD_CAPPED in HOLD (vert_ctrl) | LAND at the clamp / cap | event; LAND only after 5 s at the upper thrust bound (leaky), at a battery ceiling, or Khat at its HOLD floor (0.75 x kPrior on the camera) | CONVERT (the bounds: KEEP-PHYSICAL) |
| EST_RANGE: the height out of range while the loop can climb | LAND | LAND | KEEP-PHYSICAL (the runaway guard) |
| EST_RANGE: the speed out of range | LAND | re-seeded, event | CONVERT |
| climb faults (K_RANGE, CLIMB_CAPPED, CLIMB_TIMEOUT) with the camera height | LAND | HOLD where it is, event | CONVERT |
| the same without it | LAND | LAND | KEEP-PHYSICAL (the estimate may read low) |
| AIR (auto_motion: the move and a LAND past the air clock) | refusal | never | REMOVE |
| the move-down height floor (0.25 m + 0.01 m/s x air) | refused any motion below it | one-sided (a move down only), the drift term at most 0.30 m | CONVERT |
| a YAW without a view | refused NO_VIEW | up to 90 deg (blind turn, toward texture) | CONVERT |
| Mac: a sortie abandoned in the air (client gone, timeout) | LAND | holds (`sortie_holding`) | REMOVE |
| Mac: a sortie's TAKEOFF that may have acted after the sortie ended (its answer late or lost) | LAND | a fresh AUTO_STATUS decides: ARMED, DISARM; flying, it flies on (`takeoff_late`); none in 2 s, nothing (22.8) | REMOVE (the LAND) |
| Mac: an ARM with no usable answer | DISARM, and LAND when ARMED or flying | DISARM unless the drone flies; never a LAND (22.8) | REMOVE (the LAND) |
| Mac: a drone reboot, or a session change while active | ARM / TAKEOFF refused until the daemon restarts | refused until the drone reports IDLE in this session; a HELLO once a BAD_SESSION shows it idle (22.8) | CONVERT |
| Mac: STOP | the client's (withdrawn if the client left before it went out) | the daemon's: sent until its ACK (22.8) | CONVERT |
| Mac: a LAND confirmed by a status in LAND SINK | confirmed (a lost LAND left a SINK that could resume) | not confirmed: sent until the drone takes it over (LAND_CMD) (22.8) | (a fix) |
| Mac: `approach` (its planner: the 25 % D error, the 1 m step, "already within reach", the photo's view) | refusals | removed: Claude judges distances and sends a YAW, then a plain MOVE (`--person-gap/--est` toward a person) (22.9) | REMOVE (user decision P) |
| Mac: the ceiling rule at 80 s above 1.05 m | LAND | `ceiling_warning` | CONVERT |
| Mac: the take-off's long-wait height rules | refusal | advice in `note` | CONVERT |
| Mac: the move refusals for height (low, ceiling) | any motion | one-sided (a move down below the floor, a climb above 1.05 m past 80 s) | CONVERT |
| Mac: a planned move needs a wait | refusal | the wait always starts | REMOVE |
| Mac: 'q' and SIGHUP in the daemon's terminal | LAND | LAND | (the user's command) |

### 22.2 What the drone does instead

**A contact** (|acc| > 1.6 g or max(|gx|, |gy|) > 250 dps held 3 ms, from lift-off + 0.2 s, CLIMB or HOLD, FLIGHT and
DRY): a hush of 0.5 s (`vzInput_t.hush`): the estimate is not integrated, the thrust is the camera-measured hover
less kfrzBias (braked by the speed before the contact, lower while the camera sees a rise: a push from below), Khat
and the timers frozen; a running motion stops (a snap and a brake) or the hold re-anchors where it is. A MOVE
stopped so puts the hold's anchor 0.25 m back along its horizontal path, the way it came (`AUTO_MV_CONTACT_BACK_M`;
a back move's anchor keeps 0.50 m to the scene ahead), so the hold does not walk the drone back onto what it hit; a
contact in HOVER with no motion re-anchors again at the hush's end, where the bounce left the drone (22.8). The speed
before the contact leaves out the 3 ms the trip integrated (aLp20 x its 8 ms tau). At the hush's end the speed is
that one moved toward the camera's by at most 0.3 m/s, and for 1.5 s the hold flies open loop under capQ (the
camera estimate unused: a jolt's frames may read a speed that is not there), then it closes on the camera again.
At most two hushes start in any 2 s (a prop rubbing a wall must not freeze the estimate for good): a contact the
bound refuses is reported only. In the climb with the camera height the climb ends in HOLD where it is
(CONTACT_HOLD). Every later LAND of the sortie is the blind one (22.9); one that begins before the camera height has
been in use 0.5 s after the window also blocks the camera hold (collided).
The event carries the peaks, the body-frame force at the peak and the heading then (the obstacle's bearing).

**The camera height lost** (in HOVER from hoverAccelMs, at once after a contact or a climb fault): CAM_LOST after
300 ms (the why bits), the hold open loop under capQ (it cannot climb); SINK (a LAND, CMD_LAND for the controller)
5 s after the camera was last in use, at once without a camera-measured hover; the camera back 0.5 s during the SINK:
RESUME (HOLD again, HOVER, still waiting). A LAND, DISARM, the link, the phone or the battery take a SINK over (never
resumed); a SINK begun in a contact's window never resumes; after an older contact its descent is blind and it
resumes as any (22.9). The controller's 1.5 s window after a contact is not a loss (`vzOutput_t camOpen`).

**The controller's bounds** (vert_ctrl.h "Hold and report"): an event instead of a LAND at the clamps and caps; it
lands only after 5 s at the upper thrust bound (leaky), at a battery ceiling for capHitHoldS, or with Khat at its
HOLD floor (0.75 x kPrior with the camera). Open loop it never passes capQ.

**Never climbs without bound:** the open loop is under capQ; the post-contact window is open loop; the runaway
guard (EST_RANGE while the loop can climb) and the climb faults without the camera still land; the upper thrust
bound lands after 5 s. What remains: a camera speed that is wrong but healthy moves the closed-loop hold at that
speed (22.7).

### 22.3 AUTO_STATUS bytes 96-135, the events

docs/autonomy-link.md has the table. The latest three events (code, detail, age), evCount (the Mac logs one
`drone_event` per change), holdBits (CAM_LOST, SINKING, HUSH, K_CLAMP, CAPPED, NO_VIEW, NOT_STABLE, OPEN_LOOP),
camBits (no frames, no G, CUSUM, fault, not running), the SINK countdown, the contact's peaks, force and heading,
the time since Claude's last command, the air time past 65.5 s (airDs), Khat / prior, capQ, the contact and loop-gap
counts. The status is pushed at once on every event, as on a transition. `airLeftDs` is 0 (no air clock).

### 22.4 The Mac

`hold` in every reply (`state`, `sortie`, `move`, `wait`...): the events decoded with a hint each (every detail:
NOT_STABLE's failed conditions, SINK's why, HOLD_NO_VIEW's why, LOOP_GAP's ms), the bits, `sinkIn_s`,
`contact.obstacle_bearing_deg` (atan2(Ay, -Ax) at the contact, 0 ahead, +90 right, minus the turn since). The daemon
prints a `drone_event` line per event; `wait --until event` returns at the next report or state change. The Mac never
LANDs on its own: not for a sortie abandoned in the air (`sortie_holding`), the ceiling (`ceiling_warning`), a late
or lost TAKEOFF answer (`takeoff_late`), nor an ARM without an answer (22.8); only 'l', 'q' / SIGHUP, the land op and
`--then land` do. The height refusals are one-sided; the long-wait rules are advice. A planned move is not sent after
an event (`plan.state` cancelled). A 96-byte (2026-10-14) or 64-byte drone is still read as before. The `approach`
planner is gone (22.9): a YAW to the photo's bearing, then a plain MOVE.

### 22.5 Claude's procedure (replaces the landing advice of 21.7; its planning steps stand)

1. **A photo before every MOVE and YAW**, facing the way the drone will go (the camera sees ahead only: turn first,
   then move forward; sideways and back moves are blind beyond the hold's view). Size the move to the clear space
   the photo shows, at least 0.5 m to anything on each side of the path (0.7 m before the trim check).
2. **After every `drone_event`, read `hold` first**, then a photo:
   - CONTACT: the drone stopped and holds; `obstacle_bearing_deg` says where it was hit from. After a MOVE the hold
     already backs off 0.25 m the way it came; with no motion running it holds where the bounce left it, and its own
     drift can bring it back (22.8): back away from it (0.3-0.5 m, the other way), never retry the same move. A hand
     under or at the drone, or the drone caught or snagged (a curtain, a plant): LAND at once (a held drone is not
     landed by the drone: 22.7); on a hand the blind LAND keeps the props near the hover thrust for ~5 s: k if they
     must stop now. From now on every LAND of this sortie is the blind one (22.9): ~1 m/s touchdowns, it creeps
     ~0.2 x the height forward like any LAND, so back away 0.3 m or more from an obstacle before landing near it.
   - A sortie's `plan.state` cancelled: the drone reported an event before the wait (`plan.events`); the planned
     move did not start: photo, then decide.
   - CAM_LOST: the photo shows why (dark, blank, blocked); textured and back (CAM_BACK): go on; otherwise LAND
     before `sinkIn_s` runs out, or let the SINK land it (it resumes if the camera comes back).
   - SINK: the drone is descending; LAND takes it over; RESUME means it hovers again.
   - HOLD_NO_VIEW: the hold has no view, the drone drifts: turn toward texture (a blind YAW up to 90 deg) or LAND.
   - MOVE_STOPPED, RETRACK_UNDO: photo, replan from where it is (`wait-move` reports where).
   - VZ (K_RANGE, HOLD_CAPPED, EST_RANGE, CONTACT_HOLD): the controller is at a bound: LAND soon.
   - NOT_STABLE, TAKEOFF_SLOW: photo; check the drift before moving.
   - LOOP_GAP: hold; a second soon after: LAND (the third within 1 s lands by itself).
3. **People:** never closer than 1.0 m + the k term (21.7, 21.9); back away from a person who comes closer. Toward a
   person: a photo, the bearing and the distance read off it, a YAW to the bearing, a photo, then one MOVE forward
   with `--person-gap` and `--est` (the `approach` planner was removed, 22.9).
4. **Height:** `ceiling_warning` or a ceiling near in a photo: move down or LAND; no long hover below 0.5 m.
5. **Battery:** the drone lands by itself at its limit; MOVE / YAW are refused near it (fly what fits).
6. **End:** Claude LANDs when the task is done. Nothing else ends a hover but the battery, the link and k.

### 22.6 Verification and cost

Host tests (`run_c_tests.sh`: every suite passes but `test_gyro_tempco` 33 and `test_mahony_replay` 4, failing as
before): `test_vert_ctrl` 394 (the hush, the window, the climb contact, the camera-lost hold, the bounds, the
backstop at 599 s, RESUME, the events; 404 with 22.9), `test_auto_fsm` 1597 (1641 with 22.8, 1774 with 22.9),
`test_auto_motion` 286 (317),
`test_auto_glue_core` 191,
`test_auto_proto` 327 (the 136-byte golden), `test_auto_sortie` 79 (the Mac's LAND at the wait's end emulated: the
sortie numbers of before but the reason), `test_autonomy_glue_host` 425 (camlost-flight SINK / RESUME, DRY taps,
the palm and tap scenarios), `test_collision_replay` 71 (2026-10-13 flight 1's first CONTACT at the trip's tick;
over the 27 logged sorties of 2026-10-12/13 no CONTACT in a clean sortie's free flight, none before a contact
sortie's contact), `test_interactive_sim` 102 / 86 (107 / 91 with 22.8's wall, 121 / 105 with 22.9),
`test_att_trim_learn` 62; the Python suite 266 (272 with 22.8, 270 with 22.9).

Measured in the closed-loop sim (test_interactive_sim, the Mac's LAND 3 s after a contact; the camera speed off by
-1 / 0 / +1 m/s for the jolt's 1 s; that LAND is the blind one since 22.9, whose numbers follow in 22.9): the true
height rose at most 0.041 m after a contact in the hover (0.41 m
when the camera's speed was used at once), 0.036 during a MOVE up, 0.128 during a MOVE down and the hover after it,
0.097 in a re-track bob; touchdowns <= 0.61 m/s; 3 of the 142 ended by the controller's EST_RANGE first (a train's
later pulses beyond the hush chain). A palm brought up under the hover (48 runs, approach to 0.30 m/s): at most 0.10 m
above the contact, never a cut, a Mac LAND after it touched down on the floor <= 0.40 m/s. A grab in the climb:
CONTACT, the Mac's LAND in the hand, never KILLED. The camera lost 3 s after lift-off (test_auto_sortie, the user
range's corners): true peak 0.73 m (0.61 with the old immediate LAND; the same corner with the camera in use to
the wait's end: 0.88 m, the camera's own speed offset). A 120 s hover: nothing lands it; the Mac's LAND then
(drain from 4.10 V: BATT_LOW first up to C 400, at 45-91 s of air).

**Cost** (int-sdkconfig, against 56bd936): at 46ae25c `.iram0.text` 129763 -> 130083 (+320 B, two more 256 B
steps; `vzStep` -22 net with the probe gone, `autoFsmStep` +156 after the event forwarding moved past the step
(-356 B), the entry rules, the contact and camera gates and the re-track report moved to flash), `.dram0.bss` 87944 ->
88240 (+296: the report 60 B and its rules in `fsm`, the status 96 -> 136 B twice, the controller's state +48),
`_heap_start` 0x3fcc11c0 -> 0x3fcc14e8: the internal heap about 0.8 KB smaller, ~19.2 KB expected. Since the fixes of
22.8 (`startLand()` to flash, -575 B): `.iram0.text` 129519 (text and its padding end at 129788, 56bd936's step: the
design's gate), `_heap_start` 0x3fcc12e8, 296 B above 56bd936's (the .bss): ~19.7 KB expected after a reset (bench L0:
a hard no-go below 19 KB). Nothing new runs from flash on every loop (`callgraph.py` from
`autonomyArbitrate`): the new rules call flash on an event or while a condition lasts (a hush, a blind period, the
camera out, the view lost, a SINK); in a normal hover only the status's report block (`autoStatusFillReport`, at
10 Hz with the status, as the mailbox's `autoMbSetState` already ran) and the controller's quiet-hover window end
(`qcamDone`, once a second) do.

### 22.7 Risks and what to check (and a question for the user)

- **A drone held still or snagged** (in a hand, on a curtain or a plant after a crash) reads v ~0: the upper bound's
  5 s count needs Khat at its clamp, which a held drone may never reach: the props keep about the hover thrust until
  Claude LANDs, the user's k, a trip or the battery. The minimum "crash cut" covers only TILT / RATE / FREEFALL /
  ATT_ERR (a flip), not a drone caught upright. The verifier's grab sim (a grab at HOLD + 5 s, 65 s): 7 of 12 runs
  never landed (34.7-52.3k thrust), 2 landed K_RANGE at 38-51 s, 1 at 23 s, 2 pulled down landed HOLD_CAPPED in
  ~1 s. A camera-measured "stuck" rule (|v| < 0.02 m/s with the thrust over 1.1 x capQ for 5 s) was considered and
  not added: the hover thrust rises over a flight as the battery sags (T_hold 45-53k), so it would land healthy late
  hovers. The runbook: LAND on a hand or a snag at once; k if the props must stop now.
- **A camera speed that is wrong but healthy** moves the closed-loop hold at that speed: the sim's camera offset
  c -0.05 m/s climbs 3.6-3.9 m in 70 s (the long hover, `ceiling_warning` at 80 s above 1.05 m). After a contact the
  1.5 s window covers a jolt's frames; a camera that stays 1 m/s wrong for good rises 0.56 m in 3 s and a LAND
  on it then touched down at 3.9 m/s in the sim (the blind LAND within ~2.5 s of the contact does not read the
  camera). **Question for the user:** after a contact, should every LAND of that sortie be the blind one (about
  1 m/s touchdowns whatever the camera says) instead of only those within ~2.5 s of it? **Answered yes (B): 22.9**
  (that case: 2.43 m/s; the hold still flies on the camera, so the drone was rising at ~1 m/s when the LAND came).
- **A long contact** (pressed against something for more than 1 s): the chain bound allows two hushes per 2 s;
  the rest of the impulses are integrated (the estimate can then leave its range: EST_RANGE lands).
- **No air clock:** a hover nobody commands lasts until BATT_LOW (64-68 s of air per flight measured).
- **The phone still lands** (APP_INPUT / APP_SILENT): the user's own control, kept on purpose.
- **A contact with no motion running** (the hold's own drift into something, or a push): the hold re-anchors where
  the bounce left the drone, and its drift can bring it back (the sim's side plane 0.15-0.25 m away, pushed 1.0-1.5
  m/s^2 for 0.5 s: 5 of 17 pressed against it again, 4 of 16 at kmax 1, 3 of those 5 a plane 0.10 m away hit in
  the take-off's drift; the same as at 29f8091).
  The force-direction back-off (the anchor along the contact's measured force) was not added: its sign on the drone
  is unconfirmed until bench L3, and a wrong sign would push into what was hit. Claude backs away from the photo.
- **The approach op stays** (22.8): an optional helper with its own refusals; Claude never needs it. (Removed: 22.9.)
- **Bench L** checks the heap (a hard no-go below 19 KB), the loop rate in a hover, a tap and a covered camera, the
  events on the drone (a tap, the camera covered, a hand, a wall).

### 22.8 The verification of the finished state (2026-10-15) and its fixes

Two verifiers read 56bd936..29f8091 as it was (the plan, a wall sim, the Mac's paths, the size). What they found and
what changed (each with a test; nothing flashed):

- **The planned move flew after an event** (major). A TAKEOFF's plan (up to 4 m forward) started at the wait's start
  even after a CONTACT in the climb or in HOVER before the wait, NOT_STABLE, TAKEOFF_SLOW, CAPTURE_FAIL, the camera
  height lost or a climb fault that ended in HOLD; every one of those landed at 56bd936, so the plan never flew after
  one. Now any event before the wait cancels it (`recordEvent()` -> `planCancel()`, and a HOVER entered from a climb
  fault's HOLD): mvWhy 12 `PLAN_CANCELLED`, mvSeq 0, the drone hovers in the wait. The Mac (`--plan-via mac`) does
  not send its MOVE when evCount is not 0 at the wait; both report `plan.state` cancelled with the events.
  `test_auto_fsm` testPlanTakeoff: a contact in the climb, a contact 0.3 s into HOVER, NOT_STABLE, a CLIMB_TIMEOUT
  HOLD, a VZ event, a loop gap, CAPTURE_FAIL, TAKEOFF_SLOW (each failed with the cancel removed); the Python
  `test_plan_via_{mac,drone}_cancelled_by_an_event`.
- **The hold walked the drone back onto what it hit** (major). The contact's snap anchored the hold at the contact
  point; after a MOVE into an obstacle the brake pushed the drone 0.1-0.3 m off and the hold's integral brought it
  back within ~0.01 m in 7-18 s, touching it until Claude acted (the verifier's wall sim: 18 of 25 runs). Now a MOVE
  stopped by a contact anchors the hold `AUTO_MV_CONTACT_BACK_M` 0.25 m back along its horizontal path (the way it
  came; a back move's anchor keeps 0.50 m to the scene; not capped at the move's travel: capped at the travel +
  0.05 m, two of 24 runs stayed within 0.05 m because the hover had drifted next to the obstacle before the MOVE), the
  reference until the next motion or snap (`autoMotion_t.backOff`, in `snap`'s padding: +8 B of IRAM in
  `autoMotionStep` / `autoMotionIdleOut`). A contact in HOVER with no motion re-anchors at the hush's end, where the
  bounce left the drone. `test_interactive_sim` testWallSim (a rigid plane the camera does not see, restitution
  0.2-0.4, no Mac LAND for 42 s after the contact; 27 MOVEs forward 60-100 cm into a plane 0.25-0.35 m ahead): of the
  24 contacts that stopped the MOVE (20 at kmax 1), none touched the plane again from 3 s after the contact, the
  lowest gap 0.138 m (0.092 at kmax 1), 20 s after it at least 0.20 m (0.16); 29f8091: 18 pressed against it again.
  The spec's measure (the gap never back by more than 0.05 m) is not met over 40 s: 11 runs (12 at kmax 1) come back
  by more than 0.05 m, at most 0.16 m (0.17), the hold's own wander at D 2 m around the anchor; at 29f8091 24 runs,
  0.29 m. The no-motion case is a residual risk (22.7).
- **The Mac still LANDed by itself** (minor, two reports). A sortie's TAKEOFF answered after the sortie ended, or
  never answered, was undone with a LAND; an ARM without a usable answer sent a LAND when the drone showed ARMED,
  TAKEOFF or HOVER. Now (spec 7(h)/(i)): such a TAKEOFF is decided from a fresh AUTO_STATUS (a STATUS_REQ every
  0.2 s, up to 2 s): ARMED, DISARM; TAKEOFF or HOVER, it flies on (`takeoff_late`); anything else or nothing, no
  command (a DISARM in the air is a LAND). An unanswered ARM is DISARMed when the drone shows IDLE, ARMED, KILLED
  or nothing, and gets nothing when it flies (an ARM cannot make it fly). The commit f7594ca's "no LAND by the Mac"
  is true from here.
- **Mac refusals the design converted** (minor). (j) After a reboot (a new bootNonce: `drone_rebooted` with the old
  and new nonce and the last state) and after a session change while active (`session_lost`), ARM and TAKEOFF are
  refused only until the drone reports IDLE in this daemon's session (`reboot_cleared`, `session_back`); the daemon
  says HELLO again once a BAD_SESSION ACK shows the drone IDLE (the ACK carries the state), never while it is active.
  (k) STOP is the daemon's: never withdrawn when its client leaves. (g) The `approach` op and its refusals stay on
  purpose: it is an optional helper (a bearing read in a photo turned into a YAW and a MOVE); Claude never needs it,
  the plain `yaw` and `move --person-gap/--est` do the same with the drone's and the Mac's MOVE checks only.
- **A LAND lost during a SINK** (minor). A status in LAND with reason SINK confirmed a LAND (and `wait --until land`),
  so a LAND and its retries lost before it left a SINK that could resume. Now that status confirms neither: the LAND
  is sent until the drone takes it over (its reason LAND_CMD). `test_land_in_a_sink_until_taken_over`.
- **Texts and decoding** (minor). The PHOTO error, the `--wait-s` help, MV_WHY HEIGHT, the ceiling comment no longer
  describe the removed landings; every event's detail is decoded (NOT_STABLE's `AUTO_NS_*`, SINK's why, HOLD_NO_VIEW's
  why, LOOP_GAP's ms), `AUTO_NS_*` checked against the header.
- **Abnormal flights learned** (minor). The trim learner took a flight with contacts or a camera loss that Claude
  landed (LAND_CMD) as normal. Now `attLearnEndOk()` also needs the sortie's evCount 0 (`autonomySortieEvents()`).
  espfly_diag drops a SINK's touchdown height from the drift samples; its 'F' line has no events (a fixed RTC
  record), so a flight with a contact that Claude landed still counts there: the daemon's `drone_event` lines tell.
- **A snag keeps the props spinning** (minor): the vert_ctrl.h comment no longer lists a held drone among those the
  5 s bound lands; 22.7 says it plainly. No stuck rule was added (22.7).
- **The heap margin** (minor): `startLand()` (575 B, once per LAND) moved to flash: +512 B of internal heap
  (~19.7 KB expected, 22.6). Bench L0 is a hard no-go below 19 KB.
- **The loop rate in abnormal states** (minor): the rules call flash every loop only while a condition lasts (the
  camera rule while out, the SINK, a contact's hush and blind period, the view rule, the controller's bounds); bench
  L3 and L5 now read `activeLoopsPerS` and the largest gap during a tap train and a covered camera (>= 955).

### 22.9 The user's decisions B and P (2026-10-15)

The user answered 22.7's question and 22.8's open decisions: "(B) after any CONTACT (or COLLISION) in a sortie, EVERY
later LAND in that sortie (Claude's LAND, BATT_LOW and the battery rules, SINK's descent, link-loss LAND, any other)
flies the blind slow LAND (the one that ignores the camera height estimate, ~1 m/s touchdowns) instead of only those
within ~2.5 s of the contact; the hold itself may keep using the camera. (P) Remove the Mac approach planner; Claude
judges distances and sends plain MOVEs." Everything else stays as at faa18db: the phone's stick still lands, no
"stuck" rule, the 0.25 m back-off after a MOVE's contact uncapped. Commits 65fbf85 (B), b7dacc2 and e15a48f (the
blind LAND's reference and its floor), 3c4168c (the FSM tests), 2fbb0ec (P). Nothing flashed.

**B, what the drone does** (`auto_fsm.c`, `vert_ctrl.c`, `autonomy.c`):

- `autoFsmOut_t.blind` is the sortie's contact count (`rep.contacts`: every CONTACT, one the chain bound did not hush,
  and COLLISION; cleared at ARM) not 0. It is set in every state and the glue gives it to the controller as
  `vzInput_t.blind` (it gave `collided`). So a LAND the controller starts itself (EST_RANGE, K_RANGE, HOLD_CAPPED,
  HOLD_NO_CAM, HOLD_TIMEOUT) is blind from its first step too. The controller reads it only in LAND: the hold keeps
  the camera, as the user said.
- `collided` keeps its meaning: LAND COLLISION, or a LAND begun in the contact's window (its hush, or before the
  camera height was back 0.5 s after it). That LAND blocks the camera hold, as before. A LAND long after the contact
  keeps the camera hold down to its touchdown, as any LAND (the controller's contact or touchdown latch blocks it
  then). Why: the hold keeps the drone at its anchor, off what it hit. With the hold blocked in these LANDs instead,
  the sim's late LANDs drifted 0.6-0.8 m (0.07-0.29 m with it; a SINK without frames 1.06 m either way), and the wall
  runs pressed into the wall 1283-1938 steps in the LAND.
- A SINK after an older contact descends blind and resumes as any: the controller's resume no longer refuses a blind
  LAND, and the state machine asks only outside a contact's window (`!collided`). A SINK begun in the window never
  resumes, as before. Why: never give up; the HOLD it resumes is seeded from the camera-measured hover, never from the
  descent.
- **The blind LAND's hover reference** from HOLD is now the lowest of T_hold, tHoldQ and the camera-measured quiet
  hover tQc scaled to the battery now (x Vf at its window / Vf now; `vfQc` appended to `vzState_t`). Until now a blind
  LAND always began in the contact's open loop, under capQ, which is below tQc. A late one begins in a closed-loop hold
  that may be pushing:
  - A hold braking the descent a 1 s camera error had flown (contact during a MOVE up, e +0.85 m/s): T_hold 47266
    against tQc 45338 (+4.3 %). The flat part at 0.98 x T_hold climbed from 0.15 to 1.46 m, touchdown 1.69 m/s.
    With tQc: 0.09 m/s, no climb.
  - A camera wrong by -1 m/s for good: T_hold 15 % over the hover. The drone rose to 8.5 m and touched down at
    8.83 m/s. With tQc: 2.43 m/s. faa18db's camera LAND: 3.86 m/s.
  - A step sag of the battery before BATT_LOW (rest 4.10 to 3.40 V) left tQc 25 % low in thrust units: 1.30-1.64 m/s.
    Scaled to Vf: 0.94 m/s.
- **The floor:** in a blind LAND the stall rule (the ceiling at stallRate and the contact latch) now acts only while
  the estimate reads the drone below tdNearH (0.15 m).
  - Without it, the flat part at 0.98 x the hover floated the drone in ground effect at the floor: up to 6.8 s from
    the first floor arrival to the motors off (BATT_LOW), 1.6-1.9 s for the other kinds; the camera's LAND takes
    0.01-0.19 s. The hold kept leaning meanwhile and crept the drone 0.19 m back into the obstacle it had backed off
    from (wall sim).
  - With it: 0.7-1.9 s, bounces 0.04-0.07 m. **Corrected in 22.10:** only for a LAND 15 s after the contact. After a
    longer hover the same drift reads the floor 0.2-0.9 m high, the gate never acts, and the drone floated 5-16 s;
    22.10 replaced the gate's role with a floor rule that does not read the height.
  - A stall read in the air (a contact's fake climb, or a camera that reads a climb) also reads the drone high, so it
    does not act there. Only a double fault (an estimate drifted low and a camera reading a climb) could end the flat
    part early.
  - Written as a bound on h (`!(h >= (blind ? tdNearH : 1e9))`) it costs vzStep 20 B; `!blind || h < tdNearH` cost
    200 B in register spills.
- The controller's guard LANDs (EST_RANGE, K_RANGE, HOLD_CAPPED, HOLD_NO_CAM below the knee) keep their ceiling from
  landStart, falling at once, blind or not. A held or pulled-down drone's thrust goes sooner, and the runaway guard
  stops a climb sooner. EST_RANGE in the white-box sim: 1.18-1.51 m/s, 1.40 without a contact. **Changed in 22.10:**
  the user's B covers "any other" LAND, and after 60-100 s of hover this EST_RANGE touched down at 1.64-1.84 m/s;
  they are the slow blind LAND now.
- No fast LAND after a contact (a blind LAND never is one).

**P, the Mac** (`espfly_auto.py`): removed `op_approach` with its 25 % D margin (`APPROACH_D_ERR`), its 1 m step
(`APPROACH_STEP_MAX_M`), its 8 s scene wait (`APPROACH_D_WAIT_S`) and "already within reach"; the photo-view check only
it used (`photo_view_refusal`, `note_photo_view`, the daemon's `photo_view`, `PHOTO_YAW_TOL_DEG`); `cmd_approach`, the
`approach` subcommand and its op. `approach` is now a usage error; the daemon answers "unknown op". Nothing else
changed: `yaw`, `move` with `--person-gap` / `--est` (the Mac's person gap at k down to 0.6, 21.7 / 21.9), `stop`, the
sortie's plan.

**Verification (host; `run_c_tests.sh`: all pass but `test_gyro_tempco` 33 and `test_mahony_replay` 4, as before):**

- `test_interactive_sim` 121 / 105 at kmax 1 (was 107 / 91). New: testLateBlindLands, 132 sorties: a CONTACT 5 s into
  the wait (e -0.85 / 0 / +0.85 m/s for 1 s), no LAND for it, then each kind of LAND 15 s later, from 1.00 and 1.20 m.
  Checked for each: the reason, every controller LAND blind from its first step, none with the hold blocked, never
  KILLED or LAND_TIMEOUT, touchdown <= 1.1 m/s (EST_RANGE <= 1.6), no climb (<= 0.05 m). The same LANDs without the
  contact are the camera's and never blind:

| LAND 15 s after a CONTACT | touchdown max (m/s) | rise in it (m) | travel in it (m) | after the floor: up to (m), to the motors off (s) | without the contact: touchdown, to the motors off |
|---|---|---|---|---|---|
| the Mac's LAND | 0.36 | 0.002 | 0.28 | 0.037, 0.71 | 0.33, 0.19 |
| BATT_LOW (a step sag) | 0.94 | 0.018 | 0.09 | 0.073, 1.73 | 0.53, 0.01 |
| VBAT_DIP | 0.39 | 0.001 | 0.29 | 0.044, 1.13 | 0.31, 0.18 |
| the link lost (HB_LOST) | 0.40 | 0.003 | 0.24 | 0.053, 1.40 | 0.28, 0.18 |
| the camera out for good (SINK) | 0.38 | 0.000 | 1.06 (no frames) | 0.046, 3.85 | 0.31, 0.19 |
| the camera back 1 s into the SINK: RESUME, the Mac's LAND 5 s later | 0.57 | 0.003 | 0.13 | 0.071, 0.87 | 0.26, 0.18 |
| the phone's stick (APP_INPUT) | 0.38 | 0.003 | 0.26 | 0.001, 1.26 | 0.30, 0.19 |
| three loop gaps (LOOP_GAP) | 0.48 | 0.002 | 0.26 | 0.043, 1.68 | 0.31, 0.19 |
| a session change (SESSION) | 0.35 | 0.001 | 0.25 | 0.038, 0.71 | 0.29, 0.18 |
| the controller's EST_RANGE (white-box) | 1.51 | 0.001 | 0.07 | 0.045, 1.91 | 1.40, 0.72 |
| the controller's HOLD_TIMEOUT (25 s) | 0.35 | 0.001 | 0.22 | 0.000, 1.29 | 0.28, 0.43 |

- The LANDs 3 s after a contact (testContactSim, now blind): touchdowns at most 0.53 / 0.65 / 0.68 / 0.66 m/s in the
  hover / a MOVE up / a MOVE down / the bob (faa18db: 0.46 / 0.55 / 0.51 / 0.61); rises unchanged (<= 0.128 m). A palm
  under the hover, then the Mac's LAND: up to 0.74 m/s (0.40).
- The wall (testWallSim): the Mac's LAND 35 s after the CONTACT with nothing done in between touched the plane in 1 of
  24 runs (3 of 20 at kmax 1), at the floor (<= 0.15 m up) at <= 0.09 m/s. Every LAND creeps forward about 0.2 x the
  height (the descent read as image expansion): faa18db's camera LAND came within 0.03 m of the plane in the same
  runs. With the runbook (back away 0.30 m 5 s after the contact, then LAND): 0 of 24 (0 of 20), the lowest gap 0.17 m.
- `test_vert_ctrl` 404 (394 at faa18db; +1 the resume of a blind SINK, +9 testBlindEveryLand). The reference
  (tQc under a pushing T_hold, T_hold under tQc, tQc scaled 3.90 to 3.60 V, tHoldQ without tQc, not blind unchanged,
  EST_RANGE blind from its first step), the stall gate at 0.50 / 0.05 m, not blind as before. Each check fails with
  its part reverted.
- `test_auto_fsm` 1774 (was 1641). testBlindEveryLand: 12 LAND kinds 3 s after a CONTACT (its window over) are blind
  on every step, not collided, the hold not blocked; without a contact never blind; the next ARM clears it.
  - out.blind from the contact's own step.
  - The window's LAND still collided; COLLISION blind; a contact in the climb makes the later LAND blind.
  - A SINK after an old contact resumes and the LAND after it is blind; a SINK begun in the window is never asked
    to resume.
  - The minimum rules after an old contact: the battery's MOVE refusal at 255 ms (LOW_BATT BATT) and its LAND, a
    flip's cut in the hover and in the blind LAND, k in the blind LAND.
  - A second contact is CONTACT again.
  - With `out.blind = collided` (faa18db's rule) 57 checks fail.
- `test_autonomy_glue_host` 425 (the real glue): a tap, then the Mac's LAND onto a palm 3 s later is blind now. Motors
  off 4.69 s after the palm's contact (0.72 s at faa18db), TOUCHDOWN_FORCE, the hold not blocked.
- Python 270 (272 - the 3 approach tests + `test_approach_is_gone`).

**Cost** (int-sdkconfig, against faa18db):
- `.iram0.text` 129519 -> 129539: `vzStep` +32 B, `autoFsmStep` +4 B. Text and padding still end at 129788,
  56bd936's 256 B step, with 249 B to the next.
- `.dram0.bss` 88240 and `_heap_start` 0x3fcc12e8 unchanged: ~19.7 KB of internal heap expected (bench L0).
- `callgraph.py` from `autonomyArbitrate` and `stabilizerTask`: the flash functions reachable from IRAM are the same
  set as at faa18db. `blindLand()` (once per LAND, inside `startLand()`), `landResume()` and `sinkStep()` are flash as
  before. Nothing new runs from flash in a normal hover.

**Risks and what to check** (bench L9, L10):

- **A blind LAND onto a hand or any support above the floor** keeps the props near the hover thrust until its flat
  part has run and its ceiling falls under the touchdown thrust: 4.7 s on a palm in the glue sim. The stall gate needs
  the estimate near the floor. k stops them at once.
- **At the floor** a blind LAND whose estimate reads high (drift) floats in ground effect until its flat part ends:
  up to ~2-4 s with the hold leaning; the camera's LAND sees the stall at once. **Wrong, corrected in 22.10:** that
  number held only 15 s after the contact; after 60-100 s of hover the float lasted up to 12-15 s (the flat part's
  drift allowance, +5-10 s). See 22.10 for the fix and the numbers now.
- **The forward creep** of every LAND (~0.2 x the height toward the scene) plus that float can bring the drone back to
  an obstacle it backed off from. The runbook backs away 0.3 m or more before landing near one.
- **A camera wrong for good** after a contact: the hold flies on it (the user's choice), so the drone may already be
  moving when the LAND comes. The blind LAND does not brake that: 2.43 m/s in the sim's -1 m/s case.
- **The controller's guard LANDs** stay the fast-falling ones (EST_RANGE 1.2-1.5 m/s in the white-box sim).
  (Changed in 22.10: they are the slow blind LAND too.)

### 22.10 The verification of B and its fixes: the blind LAND's floor, the guard LANDs (2026-10-15)

Two verifiers re-checked 29f8091..e51f4b7. They found two problems with B, both now fixed in commit 4a6253f and
these docs. Nothing was flashed.

**1. After a long hover, the blind LAND floated at the floor for 5-16 s (major).**
- A blind LAND flies landTop x the hover open loop for its flat part. That part is the reference height's time plus
  the drift allowance (`landTimes()`: +5.1 s after 60 s of HOLD, +8.5 s after 100 s, up to +10 s). The allowance is
  there for a drone truly higher than its estimate.
- The floor rule of e15a48f (the stall gate) acted only while the estimate read below tdNearH. The same drift made
  the estimate read 0.2-0.9 m at the floor, so the gate never acted.
- The drone then hung in ground effect ~8 cm up, with the hold engaged, until the flat part ended.
- Claude's sessions hover for minutes, so this was every late LAND: Claude's, HB_LOST, SESSION, BATT_LOW.
- 22.9's "0.7-1.9 s" and bench L9's "~2 s" held only for a LAND 15 s after the contact.
- testLateBlindLands measured only that case. It also timed the float from the first floor touch, and a float in
  ground effect never touches.
- The same float let a blind SINK resume into a HOVER 3-8 cm over the floor.

**2. The controller's guard LANDs were not the slow blind LAND (minor).**
- After a contact, EST_RANGE, K_RANGE, HOLD_CAPPED and HOLD_NO_CAM below the knee were blind, but they still started
  at landStart and fell at once.
- That was 22.9's open decision 3, shipped as the default. The user's B says "EVERY later LAND ... any other".
- After long hovers, EST_RANGE touched down at 1.64-1.84 m/s, over the test's own 1.6 bound.

**The fix** (`vert_ctrl.c`):
- **`blindFloor()`** (flash; vzStep calls it only when a blind LAND has not been descending for stallS, or after the
  contact latch). Once the flat part without the allowance (`landFlatBase`, appended to `vzState_t`) has run, either
  of these ends the flat part:
  - VZ_BLIND_FLOOR_S (0.5 s) without a descent: the stall's speed test, v > -stallV, whatever the height estimate
    reads;
  - the contact latch.

  The ceiling then falls at landRate, the same fall that ends every blind flat part. In the air that is the right
  answer too: landTop x the reference is not bringing the drone down.
- **Why not before the base.** A camera wrong for good that reads a descent too slowly could then take its error
  into the touchdown. With an earlier, camera-gated end, testContactLand's blind LANDs touched down at 1.26-1.70 m/s
  (bound 1.1). After the base, the worst it can do is take away the drift allowance. testContactLand's 60 s hovers
  with a camera +1 m/s wrong for good: 0.61 / 0.75 m/s at 1.00 / 1.20 m (0.37 / 0.36 before).
- **The blind stall rule** (the estimate below tdNearH: the ceiling at stallRate, the contact latch) moved into
  `blindFloor()`. It is armed from VZ_BLIND_STALL_ARM_S (1.0 s), not 0.3 s: landTop x the reference starts the
  descent slowly (-0.08 m/s after ~0.5 s). An estimate drifted low read that start as a stall in the air (a resumed
  SINK's LAND: the ceiling cut 7 %, 1.27 m/s).
- **A blind SINK resumes only in its flat part, and only while descending.** stallS at rest refuses it: at the floor
  no stall rule touches it down first. A camera LAND's stall touches down within ~0.5 s.
  - It can still resume in the last centimetres of a descent, as the camera's SINK can: the lowest resume 0.07 m
    (15 s set), 0.17 / 0.20 m (60 / 100 s).
  - A SINK that began within 0.25 m of the floor lands (3 / 4 / 6 of 12).
- **`startLand()`:** a blind LAND from HOLD (a measured hover) is the slow one whatever its reason, the controller's
  guards included.

**The numbers** (testLateBlindLands, now three sets of 132 sorties). Each set: every LAND kind, 1.00 and 1.20 m,
e -0.85 / 0 / +0.85 m/s for the contact's 1 s. The LAND comes 15 s after the contact, or 60 / 100 s into a 120 s wait
with the camera's speed offset -0.004 / +0.004 m/s.
- "Under 0.12 m" is the time with the motors on and the true height under 0.12 m, from the LAND's start. It includes
  the last ~0.5 s of the descent (the camera's LAND: ~1.1-1.5 s).
- The 6.6 s cases began at the floor: the contact's camera error had pushed the hold down there. The flat part's base
  (3.3 s at 1.00 m, 4.0 s at 1.20 m) plus the ~2.3 s fall to the touchdown thrust.

| LAND after a CONTACT | touchdown max, 15 / 60 / 100 s (m/s) | under 0.12 m max, 15 / 60 / 100 s (s) | the same from 0.25 m up or more (s) | e51f4b7: under 0.12 m (s) | without the contact at 100 s: touchdown, under 0.12 m |
|---|---|---|---|---|---|
| the Mac's LAND | 0.36 / 0.39 / 0.33 | 6.67 / 6.65 / 6.62 | 2.28 / 3.93 / 3.31 | 8.49 / 11.87 / 14.03 | 0.28, 1.48 |
| BATT_LOW (a step sag) | 0.94 / 0.96 / 1.13 | 6.15 / 6.28 / 6.62 | 1.59 / 5.02 / 5.10 | 8.10 / 11.58 / 14.78 | 0.51, 1.11 |
| VBAT_DIP | 0.39 / 0.38 / 0.36 | 6.66 / 6.65 / 6.62 | 2.54 / 3.89 / 3.78 | 8.49 / 11.88 / 14.96 | 0.31, 1.43 |
| the link lost (HB_LOST) | 0.40 / 0.35 / 0.34 | 6.66 / 6.67 / 6.65 | 3.85 / 3.49 / 3.87 | 8.56 / 11.97 / 15.35 | 0.28, 1.21 |
| the camera out for good (SINK) | 0.38 / 0.40 / 0.43 | 8.92 / 6.66 / 6.62 | 3.57 / 4.35 / 5.60 | 8.92 / 12.31 / 14.92 | 0.31, 8.97 |
| a SINK resumed, then the Mac's LAND | 0.36 / 1.01 / 0.58 | 6.67 / 6.66 / 6.67 | 4.03 / 1.20 / 1.19 | 6.66 / 6.66 / 6.67 | 0.31, 1.17 |
| the phone's stick (APP_INPUT) | 0.38 / 0.38 / 0.38 | 6.64 / 6.62 / 6.62 | 1.19 / 4.03 / 3.37 | 8.47 / 11.20 / 13.48 | 0.26, 1.20 |
| three loop gaps (LOOP_GAP) | 0.48 / 0.36 / 0.35 | 5.98 / 6.66 / 6.66 | 4.41 / 3.61 / 3.78 | 7.83 / 11.91 / 15.30 | 0.28, 1.66 |
| a session change (SESSION) | 0.35 / 0.36 / 0.40 | 6.43 / 6.62 / 6.64 | 2.28 / 3.65 / 3.42 | 8.26 / 11.85 / 15.26 | 0.32, 1.17 |
| the controller's EST_RANGE | 0.32 / 0.34 / 0.33 | 5.98 / 6.65 / 6.62 | 4.51 / 3.38 / 4.13 | 2.35 / 2.36 / 2.36 | 1.60, 1.23 |
| the controller's HOLD_TIMEOUT | 0.35 / 0.36 / 0.38 | 6.65 / 6.62 / 6.62 | 1.20 / 3.49 / 3.40 | 8.78 / 11.98 / 14.75 | 0.29, 1.19 |

- EST_RANGE after a contact, at e51f4b7: 1.51 / 1.65 / 1.73 m/s (15 / 60 / 100 s). Now: <= 0.34.
- A pulled-down drone (a sim harness: a -0.15 g pull 15 s after the contact, K_RANGE): touchdown 0.37-1.19 m/s
  before, now 0.28-0.39.
  - Its motors stop later: 2.2-4.4 s after the LAND starts (before: 1.5-2.5 s).
  - Pulled onto the floor before the LAND: 6.6 s (before: 2.3 s). That is the price of "any other" LAND being the
    slow one; k stops it at once.
- The test checks every sortie:
  - the reason (or the controller's K_RANGE first, with the hold already at the floor);
  - blind from the first step, the hold not blocked, no climb, never KILLED or LAND_TIMEOUT;
  - touchdown <= 1.1 m/s (BATT_LOW 100 s into the wait: <= 1.2);
  - under 0.12 m within the flat part's base + 3 s (the camera out for good: its whole flat part + 3 s).
- At e51f4b7 the same sets show touchdowns up to 1.73 m/s and 8.8 / 12.0 / 15.4 s under 0.12 m.

**Verification:**
- `run_c_tests.sh`: everything passes but `test_gyro_tempco` (33) and `test_mahony_replay` (4), as before.
- `test_vert_ctrl` 412 (404):
  - testBlindFloor: 6 kinds after a 60 s HOLD with the estimate reading 0.50 m. At rest: the flat part ends at its
    base, then landRate (not stallRate). Descending: the allowance runs in full. At rest from base + 1 s: it ends
    0.5 s later. Nothing ends it before the base. The contact latch ends it at the base. Not blind: unchanged.
  - The blind stall's arm.
  - EST_RANGE blind and slow; not blind, as before.
  - The blind resume refused past the flat part and at rest.
  - Each check fails with its part reverted (mutations in `optb/fixer/mut`).
- `test_interactive_sim` 145 / 129 at kmax 1 (121 / 105). A contact during a MOVE down: touchdown max 0.68 -> 0.51
  m/s. Other info lines unchanged.
- `test_autonomy_glue_host` 425: the palm LAND still stops 4.69 s after the palm's contact.
- `test_auto_fsm` 1774. Python 270.

**Cost** (int-sdkconfig, against e51f4b7):
- `.iram0.text` 129539 -> 129559: vzStep +15 B, the call placed right after the stall rule. Placed after the bump's
  latch it spilled registers, +244 B. Text and padding still end at 129788, 56bd936's step.
- `.dram0.bss` 88240 -> 88248: `vz` 0x2f0 -> 0x2f8 (`landFlatBase`, 8-byte aligned). `_heap_start` 0x3fcc12e8 ->
  0x3fcc12f0: ~19.7 KB expected (bench L0).
- `callgraph.py` from `autonomyArbitrate` and `stabilizerTask`: the flash set of e51f4b7 plus `blindFloor`.
  `blindFloor` is called from vzStep only in a blind LAND that is not descending or has the contact latch: never in
  a normal hover.

**Risks and what to check** (bench L9):
- **A LAND that begins at the floor or on a support** after a contact keeps the props near the hover thrust for the
  flat part's base plus the fall: up to ~6.7 s from 1.20 m, 4.2 s from the 0.30 m default (a harness run of 12 late
  LANDs from 0.30 m after a contact; e51f4b7: up to 9.5 s).
  - The hover can be at the floor when a contact's camera error pushed it down. The sim then hovered in ground
    effect.
  - A real leg touch is a bump. It ends the flat part at the base, as above.
- **The camera out for good** (SINK) gives no speed to see the floor by, and the sim's smooth ground effect gives no
  bump. The float runs to the flat part's end: up to 8.9 s here, 15.7 s at 100 s with e +0.85 in a sweep. The
  camera's own SINK floats too (9.0 s, 13.2 s in the sweep).
- **BATT_LOW long into a hover after a contact, with a step sag:** 1.11-1.13 m/s, as at e51f4b7. The blind reference
  (the camera-measured hover scaled to Vf at the LAND's start) is fixed while the battery keeps falling ~0.2 V in the
  first second. The camera LAND's T_hold was 8 % higher, 0.44-0.51 m/s.
- **A camera wrong for good** that reads a descent too slowly takes away at most the drift allowance: 0.61-0.75 m/s
  in testContactLand's 60 s cases.
- The forward creep and the camera-wrong-for-good climb of 22.9 are unchanged.

## 23. The height from photos: SET_HEIGHT and the level line (user decisions 2026-10-15)

The drone has no height sensor. Its height estimate (the camera and accelerometer observer through vert_ctrl) drifts
0.01-0.03 m/s, mostly reading high, so a long hover sinks: in the living room on 2026-10-15 vzH read 0.97-1.01 m for
105 s while the drone sank about 1 m to the floor, and the 12:44 flight's photos show the far wall's floor line rising
toward the image centre while the drone believed 1.0 m. The user ruled out hardware changes and hardcoded room
knowledge, accepted firmware and minimal Mac tooling, and approved (2026-10-15): (1) a firmware SET_HEIGHT, "you are at
X m", with generic bounds only; (2) minimal Mac tooling: a level line on every photo, a `set-height` command, logging;
(3) Claude's routine (this section, 23.5: the everyday heights live in Claude's judgement, never in software); (4) the
height limit 1.20 -> 2.00 m with the timings scaled from the height (23.9). Commits 7bc4d15 (the firmware), a7f5a49
(the Mac), this section, and the 2.0 m limit on top; the review fixes of 2026-10-15 (23.10): the take-off stays at
1.20 m (2.00 m by MOVEs up only), nothing flies near the end of the battery (a descending correction included), the
LAND timeout after a move down, and the Mac's guards.

### 23.1 Why correcting the estimate alone is not enough

HOLD has no height loop: it is a speed loop on v, which the camera's camVz pulls, with vRef 0 when no motion runs
(vert_ctrl.c HOLD, auto_motion.c). A sink is a camera speed offset c: the drone moves at -c while h stays flat. So
nothing is gained by changing h alone, or by slewing an offset into it: the difference has to be flown. SET_HEIGHT is
an instant shift of the estimate plus a correction flown as a vertical MOVE: the existing, tested path at 0.15 m/s.

### 23.2 SET_HEIGHT on the drone (`auto_fsm.c setHeightCmd`, `auto_motion.c autoMotionSetHeight`, `vert_ctrl.c vzShiftHeight`, `autonomy.c applyHeightShift`)

**The command** (0x33, docs/autonomy-link.md): i16 xCm, Claude's judged true height X; i16 atCm, the drone's own
estimate when the judged photo was captured (INT16_MIN: now). The offset form, the default: d = X - h_at; the absolute
form: d = X - h_now. The new estimate is h_now + d. The offset form exists because Claude takes 12-18 s from photo to
command (events.jsonl): moves made in between cancel out, and only the drift during that delay remains (0.12-0.24 m at
0.01-0.016 m/s).

**On acceptance**, all on the same 1 kHz step:
1. The controller's h += d (`vzShiftHeight`, flash, HOLD only, |d| <= 0.51). Nothing else in vzState_t moves: v, vHp,
   Khat, Kfrz, T_hold, tHoldQ, tQc / capQ, kLearn and the LAND fields stay. The next 2000 steps' thrust is
   bit-identical to an unshifted twin (test_vert_ctrl `testShiftHeight`).
2. The committed height offset hOff += d: the reference moves into the corrected frame, so hMax + hOff keeps exactly
   its margin over h and no EST_RANGE comes from the shift (without it a +0.30 m shift 0.10 m inside the guard lands
   EST_RANGE: the same test).
3. A vertical-only MOVE starts with upM = min(h_now, the cap) - h_new (normally -d) from hStart = h_new, mvFlags bit 6
   (0x40, `AUTO_MVF_SET_HEIGHT`: on a MOVE a correction, on a YAW the blind turn): the profile at 0.15 m/s and
   0.20 m/s^2, the position term 1/s, |vRef| <= 0.15 m/s, the settle (<= 3 s, 0.04 m + 10 %) and the progress guard
   (0.15 m + 0.5 |up|) of every vertical MOVE. This step's motion inputs see the shifted estimate (progress 0): with
   the old one the progress guard stopped every correction of 0.3 m or more on its first step.
4. DONE commits upM, so hOff returns exactly to its old value (`commitUp`: the offset before it + (d + upM), and the
   Mac's upM is exactly -d below the cap): the drone ends at the height it was holding in the old frame, truly where it
   believed it was. STOP, a contact, LEFT_HOVER (LAND, KILL) or a guard commit the measured travel instead: hOff = old
   + d + progUp, the reference where the drone stopped.

The estimate shift is kept in every case, so a LAND right after the ACK uses the corrected height. Durations: 0.10 m
1.41 s, 0.30 m 2.75 s, 0.50 m 4.08 s, plus the settle. No event: an event bumps evCount, and the trim learner skips
every sortie with one (att_trim_auto.c `autonomySortieEvents`); a routine correction must not disable it.

**The checks, in order** (the first failure answers; `mvWhy` keeps the latest refusal):

| # | Check | Refusal |
|---|---|---|
| 1 | HOVER, in the wait, the controller in HOLD (outside HOVER `cmdOutside`: detail before \| NOT_WAITING << 8) | NOT_WAITING (BAD_STATE) |
| 2 | no motion running (a MOVE, a YAW, a bob or re-view, a correction, the take-off's planned move) | BUSY (BAD_STATE) |
| 3 | no contact window (`contactBlind`) and no hush | SH_CONTACT 15 (CAM_UNAVAIL) |
| 4 | the camera height in use (`mi->camHeight`; in open loop vRef is ignored: the correction could not fly) | NO_CAM_HEIGHT (CAM_UNAVAIL) |
| 5 | the hold's view, as any vertical MOVE | NO_VIEW (CAM_UNAVAIL) |
| 6 | no accepted SET_HEIGHT in the last 2000 ms | SH_RATE 14 (BAD_STATE) |
| 7 | X 10..250 cm, d and upM finite, d not 0 cm | RANGE |
| 8 | \|d\| <= 50 cm (whole cm: the absolute form's X - h_now rounds), \|upM\| <= 0.55 m | SH_STEP 13 (RANGE) |
| 9 | h_new within 0.10 m..the cap + 0.50 m | HEIGHT (RANGE) |
| 10 | battLow (BATT_LOW held 0.25 s, or T_hold > 53400): no correction flies, up or down, as no MOVE (23.10: a descent's braking there ended HOLD_CAPPED in the drain sim) | BATT (LOW_BATT) |

moveFloor's height checks do not apply: the floor's drift term would refuse exactly the down-corrections needed late
in a sortie, and the end is the held height. The cap still applies to the corrected estimate for every later MOVE.
`autoMotionSetHeight` repeats checks 2, 4, 5 and 10 (its own unit tests): the state machine's BATT check is then an
equivalent duplicate.

**The ACK** detail = state | why << 8 | the old estimate in cm (i16) << 16: mvDetail's layout with the seq replaced.
This firmware never answers SET_HEIGHT with detail 0; an older drone (1e4184b) answers the unknown type BAD_STATE
detail 0, so the Mac tells the two apart. **AUTO_STATUS** (no layout change): lastCmdType 10, mvSeq + 1, mvKind MOVE
with mvFlags 0x40, tgtUpCm = upM, progUpCm, mvState RUN / SETTLE / DONE / STOPPED with mvEnd, hMm the corrected
estimate from the next status, hTargetCm the reference ramping from old + d back to old.

**Not touched:** the camera height observer (cam_vz_obs x[0] / the recorder's cvH: not measured, only logged, on the
camera task; after corrections cvH_mm differs from vzH_mm by their sum, and the daemon's events line them up); the LAND
drift-allowance clocks (HOLD tPhase, hoverDurMs: conservative, a correctly estimated drone still lands by the stall
rule); heightFloor's air term; Khat and vzLearnK. KILL, HB_LOST, the TILT / RATE / ATT_ERR cuts, BATT_LOW, EST_RANGE,
K_RANGE, SINK and the phone's stick keep their code paths.

**Behaviour:** STOP during a correction keeps the corrected estimate and holds where the drone is. A LAND during one
commits the measured travel (landExtraMs, landTimes): if the drone was higher than believed (d > 0) the flat part is
longer, which is safe. A second SET_HEIGHT while one runs: BUSY. The height offset's limit VZ_HOFF_MAX =
AUTO_MV_HOFF_MAX = (cap - 0.20) + 0.50 m (2.30 at the 2.00 m cap, 1.50 at 1.20 m; 1.00 before): during a correction
the reference may sit up to 0.50 m beyond the commanded range, and a smaller clamp would make hMax + hOff under-read
(a false EST_RANGE).

### 23.3 The level line (Mac only: `tools/espfly_auto.py`)

Claude judges the true height by what lies at camera height: the row of the true horizontal. On the upright photo
(after FLIP_VERTICAL), row 0 at the top, at 160x120:

    y(x) = 60 + FY tan(pitch + tilt) - (x - 80) tan(roll)            FY = 99.79 (LEVEL_FY_PX = AUTO_MV_FY_PX)

and on the x4 PNG (X = 4x + 1.5, Y = 4y + 1.5, as the bearings' x4_of):

    Y(X) = 241.5 + 399.16 tan(pitch + tilt) - (X - 321.5) tan(roll)   (about 7.0 px per degree near the centre)

+pitch is nose up, +tilt the camera pointing up, +roll makes the line rise to the right. pitch and roll are the photo
META's (the recorder's attitude when the copy is seen, 0.1-0.23 s after the exposure). There is no camera roll
constant (0 +/- 1 deg measured). The tilt absorbs the principal-point offset: one function draws and calibrates.

**The mount tilt is not a constant** (the tilt analysis: 24 ground photos, 21 hover vanishing points): in flight
-6.4..+3.9 deg (sd 2.7); jumps of 3-8 deg with the 10-13 kill fall (+3.5 -> -4), the 10-14 crash (-0.3 -> +2.8) and a
battery replug (~8.5 deg); over 5 gym landings it drifted -1.4 -> -5.0. Within one flight it is stable (0.0-0.6 deg
over 5-20 s), and the pre-take-off ground photo predicts the in-flight value: ground - flight = -0.29 +/- 0.71 deg
(n = 15, worst 1.19). A stale value 5 deg off puts the line 0.26 m off at 3 m and 0.44 m at 5 m. So the tilt is ONE Mac
setting, never per room, re-measured every flight from the ground photo (the camera sits 3.5 cm above the floor, so a
far floor-wall line D m away lies FY x 0.035 / D px below the level row: room-independent physics), and after any
crash, contact, catch or battery swap.

- **The setting** `~/.espfly/level.json` (atomic write, private dir): tilt_ground_deg, tilt_flight_deg (= ground +
  LEVEL_FLIGHT_ADJ_DEG 0.3), set_t / set_utc, method floor | horizon | manual, source {photo, row_x4, dist_m,
  pitch_deg, roll_deg}, boot_nonce (the calibration photo's), invalidated_t / utc / why; the history
  `~/.espfly/level-history.jsonl` (append-only). The daemon re-reads it (by mtime) before each photo. Without a
  setting the default 1.9 deg (ground) is used and always drawn stale.
- **Stale** (drawn dotted): set before its invalidated time, or set on another boot than the photo's. The daemon
  invalidates on a new bootNonce (a reboot or battery swap), every LAND -> IDLE (a landing), a CONTACT event, a LAND
  COLLISION and KILLED (`level_invalidated` events): mechanical facts, no decisions.
- **`level-cal`** (arithmetic only; through the daemon when it runs, else on the files): `--floor-row R [--dist D, 3.0]
  [--photo P]` on the latest ground photo (IDLE / ARMED): y = (R - 1.5) / 4 - FY x 0.035 / D, tilt_ground = atan((y -
  60) / FY) - pitch, flight = + 0.3; `--horizon-row R` on the latest flight photo (a vanishing point of horizontal
  room lines, or another true-horizon cue, at the centre column): tilt_flight = atan(((R - 1.5) / 4 - 60) / FY) -
  pitch, ground = - 0.3; `--set DEG` (the flight value by hand); `--show`. Each run prints the old and new values and
  the line's new row on that photo, writes the setting, the history line and a `level_cal` event, and draws that
  photo's _x4.png again; with --floor-row it reports `floor_gap_px` (the line should sit 3-7 px above a far
  floor-wall line 2-6 m away). The 10-15 ground photos 122352 and 124340 (far floor line at x4 row 240, pitch -2.61 /
  -2.73, D ~3.5 m) give +1.82 / +1.94 deg ground (the default 1.9; the analyst's +2.1 in its 240 / 409.6 convention);
  in flight +2.2. Do not calibrate from a take-off hover photo by assuming 1.0 m: the gym's true heights were 1.08-1.27
  m while the estimate read 0.89-1.11. The latest photo is the latest by when it was taken (`photo_time()`: the JSON's
  `link.received_t`, else `received_utc`, else the name's stamp; never the mtime, which `rerender_level` moves). A
  photo, given or the default, taken before the setting's latest invalidation (kept through later level-cals as
  `valid_after_t`), or through the daemon from another boot, is refused: it shows the camera before what may have
  moved it (23.10). The reply carries the photo's `photo_utc` and `photo_age_s`.
- **The drawing** (only the _x4.png; the 160x120 .png stays raw): 1 px along Y(X), rounded per column, dashed 8 on / 8
  off with the dashes alternating yellow (255, 230, 0) and magenta (255, 0, 255) (visible on dark and light); a stale
  line dotted 2 / 6. It covers at most 1 of the 4 rows of a source pixel's 4x4 block, on half the columns: no source
  pixel is hidden (>= 12 of 16 px kept, tested). Edge ticks at the level row (12 x 3, yellow, a 1 px black border, X
  0-11 and 628-639); right-edge angle ticks at +-1 and +-2 deg (6 px) and +-5 deg (12 px), rows Y_c + 4 FY (tan(p + t +
  k) - tan(p + t)), roll ignored; out of rows 0..479 only a 3 x 12 marker at the top or bottom of the centre column
  (the JSON says by how many px). No text. `ESPFLY_LEVEL_LINE=0` or `--no-level` (daemon, photo): not drawn, the JSON
  block written.
- **The photo JSON** gains `level` {row_x4_center, row_160_center, slope_x4 (= -tan roll), endpoints_x4, pitch_deg,
  roll_deg, tilt_deg_used, tilt_kind (ground for a photo captured in IDLE / ARMED), tilt_set_utc, tilt_method, stale,
  stale_why, fy_px, formula, ticks_deg, in_frame, drawn} and in `autonomy`: `h_est_capture_m` (the drone's estimate
  when the frame was captured: set-height's h_at), `h_src`, `state_capture`, `sortie_key`, `capture_mono`,
  `last_set_height` {utc, offset_m, new_m}. The photo reply and event add the row, the tilt, stale and the estimate.

**The estimate at capture.** META captureUpMs and STATUS upMs are the esp_timer clock; AUTO_STATUS upMs is the FreeRTOS
tick (a boot-constant offset: the tick starts with the scheduler). The daemon keeps a ring of this boot's AUTO_STATUS
(upMs, hMm, state) and a second minimum-offset estimator for the esp_timer clock (from the 1 Hz STATUS replies, as
`note_clock` does for the tick), maps the capture onto the ring through the Mac's clock and interpolates (`h_src`
capture); without a STATUS yet, the snapshot received nearest the META's arrival (`h_src` arrival: 0.1-1 s late).

### 23.4 The Mac: `set-height`, the logs

`set-height X [--photo P | --at M | --now] [--no-wait]` (X 0.10..2.50; daemon op `set-height`): by default h_at is the
latest saved photo's `h_est_capture_m` from this sortie. Refused locally only when no photo was saved since the last
accepted set-height ("take a new photo or use --now": one photo is never applied twice) or the photo is from another
sortie or boot. The reply: result, why with its hint, old_m (from the ACK), offset_m, new_m, fly_m, seq (from the next
status), then (by default) the correction's end, h_m, dur_s. BAD_STATE detail 0: "firmware has no SET_HEIGHT (fw
...)". A STOP withdraws a queued one (MOTION_TYPES); wait-move and the status name a correction SET_HEIGHT (mvFlags
0x40 on a MOVE), never Claude's MOVE. Events: `set_height` {x_m, at_m, at_photo, old_m, offset_m, new_m, fly_m, seq,
result, why}, `set_height_end` {end, h_m, progUp_m, dur_s}, `level_cal`, `level_invalidated`. The status gains
`since_set_height_s` and `since_photo_s` (information). The recorder CSVs are unchanged. A set-height whose outcome is
unknown (no ACK after the resends, a reconcile ending UNKNOWN: the drone may have applied it) answers `outcome`
"unknown" and marks the photos up to then as used, as an accepted one does: the next offset-form command needs a newer
photo (23.10; before, only an ACK OK did, so the same photo could be applied twice).

The ceiling rule (1.05 m, 80 s: the take-off note, `ceiling_warning`, the up-move refusal) is removed: Claude checks
the height and the ceiling from the photos (23.5). Its successor is a warning only (23.10): `height_check_due`, once
per lapse, when the sortie's latest height check (a saved photo or an accepted set-height) is older than the runbook's
cadence at the commanded height (`HEIGHT_CHECK_S` 15 s, `HEIGHT_CHECK_HIGH_S` 10 s above 1.20 m), with the drift that
may have built up; the status's `height_check_due` says the same. Nothing is refused or landed for it.

### 23.5 Claude's height routine (replaces the 1.05 m / 80 s ceiling advice)

The everyday heights below are Claude's judgement and live only in this document, never in software.

**Commissioning** (once after flashing; the user present with a tape mark): bench M (docs/autonomy-bench.md): DRY
first (accepted, the sim's estimate jumps by d, the correction at <= 0.15 m/s, a second command within 2 s BUSY or
SH_RATE, IDLE and LAND NOT_WAITING); a tape mark at 1.0 m on a wall 2-3 m away: hover at 1.0, `set-height 0.80 --now`
(the drone climbs ~0.2 m: check against the tape; the camera's scale is +-20-30 %), then the opposite; a 60 s hover at
1.0 m with checks every 10-15 s, the tape as ground truth.

**Every flight:**
1. **The level line, on the ground.** Place the drone where it will take off, facing a wall 2-6 m away whose
   floor-wall junction shows near the image centre. `photo`, read the far floor-wall row R at the centre column,
   `level-cal --floor-row R --dist D`, and check that the redrawn line sits 3-7 px above that floor line. Mandatory
   after every landing, battery swap or reboot, crash, contact, catch or handling (the daemon marks the line stale and
   draws it dotted). With no far floor-wall line in view: ask the user to turn the drone toward one; otherwise stay
   <= 1.2 m and use `level-cal --horizon-row` on the first hover photo with clear converging horizontal lines; failing
   both, treat the line as +-5 deg (+-0.25 m at 3 m).
2. **Take off to <= 1.0-1.2 m** (the drone refuses a higher take-off since 23.10). Go higher only by MOVE up in
   <= 0.5 m steps from a checked hover, a photo after each step, and only when the judged ceiling clearance at the
   target is >= 0.6 m, the battery reads >= ~3.25 V loaded and no person is within 2 m horizontally.
3. **A reference photo right after take-off** (the take-off set). Judge the true height; never assume the target (the
   gym: 1.08-1.27 m true while the estimate read 0.89-1.11).
4. **Judging the height.** The photo's `h_est_capture_m` is the drone's estimate. Look at what lies ON the dashed line:
   if a reference's top lies on the line, the true height is its height, whatever its distance. Otherwise h = H_ref +
   D (Y_ref - Y_line) / 400 (x4 px, Y downward; ~7 px per deg; 1 deg = 5 cm at 3 m), D from the scene (the photo's
   D_m, a person's size, the room's geometry). Everyday heights: door handles ~1.0 m; table and desk tops 0.72-0.76 m;
   kitchen counters ~0.9 m; sofa backs 0.8-0.9 m; office-chair backs ~1.0-1.1 m; light switches 1.05-1.2 m; window
   sills 0.8-1.0 m; door tops 2.0-2.1 m; ceilings 2.4-2.7 m; a bed or crib 0.5-0.6 m; an adult's waist ~1.0 m,
   shoulders 1.4-1.5 m, eyes ~1.6 m. Prefer references 2-5 m away near the image centre (little barrel distortion),
   and two that agree within 0.1 m.
5. **Correcting.** (Since 27, 2026-10-16: X is the true height when the photo was taken and the drone flies to the
   commanded height, so send the judged height as it is, never the est-rule's h_at + X - wanted; `--to H` for another
   height; the reply's `true_now_m`, `target_m`, `fly_m` say what it does.) `set-height X` (the offset form, the
   default) when |X - target| > ~0.10 m and the judgement
   is confident (two references within 0.1 m, or one clear reference on the line). References disagreeing by > 0.2 m
   or nothing near the line: no correction; another photo, or a YAW of 30-60 deg for a better view. More than 0.5 m
   off: send the 0.5 m part, a new photo, again. After a correction of >= 0.3 m, a new photo to confirm (the camera's
   scale is +-20-30 %).
6. **Cadence.** Every 10-15 s at <= 1.2 m, every 5-10 s above it; before every MOVE (vertical, forward, toward
   people); before a planned LAND when the last check is more than 10 s old and the battery allows (the shift applies
   at once; the LAND may follow the ACK). Keep photo-to-command under ~10 s: read the line first, the rest of the scene
   after. The error left is about |c| x that delay; at |c| 0.03 m/s with a 15 s delay one 0.5 m command a cycle cannot
   keep up (23.6).
7. **Never assume the drift's direction.** On 10-15 the drone rose 0.13-0.27 m in 23 s, then sank about 1.2 m.
8. **Above 1.2 m** (23.9): at every check the ceiling clearance = D_wall (Y_line - Y_ceiling-wall) / 400; keep it >=
   0.5 m (MOVE down if less). No hover above 1.5 m within 2 m of a person (a sink brings the props to face height).
   Come down to <= 1.2 m when T_hold >= ~52000: from 53400 on the drone refuses every MOVE and every set-height, a
   descent included (23.10), and its BATT_LOW LAND from 2.0 m is the one left. (A loaded reading of ~3.20 V is no rule:
   climbs are refused on thrust headroom only, UP_HEADROOM, 27.3; the tiger climb flew at 3.12-3.22 V.)
   After a contact (every later LAND is blind), MOVE down to <= 1.2 m before the LAND when possible.
9. **Refusals.** BUSY: wait for DONE or STOPPED (<= ~7 s). SH_RATE: wait 2 s. SH_STEP: send <= 0.5 m. HEIGHT: the
   judgement is implausible, take a new photo. NO_CAM_HEIGHT, NO_VIEW, SH_CONTACT: wait for CAM_BACK / VIEW_BACK, then
   a new photo. LOW_BATT: near the end of the battery no correction flies (up or down): LAND, or have come down from
   ~52000 (8.). LOW_BATT with why UP_HEADROOM (27.3): a climb without the motors' headroom (T_hold within 4600 of the
   ceiling): come down or LAND. A reply with `warnings` "the photo is N s old": take a new photo first. A reply with
   `clipped_to_m`: the drone flies 0.50 m toward the target; a new photo, then a plain `set-height X` flies the rest
   (the daemon remembers the clipped target, 27.9; a vertical `move` or `--to` replaces it). A `drone_event` HOLD_LIMIT
   (27.4): photo, watch the drift, land soon, `trimcheck`. A reply with `outcome` "unknown": take a new photo before
   the next set-height (the drone may have applied it). `height_check_due`: take a photo now. "no SET_HEIGHT in firmware": `move --up/--down` by the judged
   error.
10. **After any contact or catch in flight** the camera may have moved: check the line against two references or
    against converging room lines; off by more than ~2 deg (14 px): `level-cal --horizon-row` on a clean horizon cue,
    else stay <= 1.2 m, LAND soon and recalibrate on the ground.
11. Say in the conversation which references each judgement used. The daemon logs every set-height.

### 23.6 Verification (host)

- `test_vert_ctrl` `testShiftHeight`: only h changes (memcmp of vzState_t), 2000 steps of bit-identical thrust against
  an unshifted twin for d +-0.50, +0.23, -0.07, refused outside HOLD (IDLE, CLIMB, LAND, CUT), for NaN, inf and |d|
  0.52; the EST_RANGE margin unchanged with hOff + d and broken without it.
- `test_auto_motion` `testSetHeight`: hStart = vzH + d, hOff = old + d, upM = -d, the flag; the first step's vRef the
  profile's (no jump), no PROGRESS on the first step for +-0.5; |v| <= 0.15, |a| <= 0.20; DONE restores hOff exactly;
  STOP / CONTACT / LEFT_HOVER commit old + d + travel; a stuck estimate stops PROGRESS at 0.15 + 0.5 x 0.40 m; BUSY
  (MOVE, YAW, a correction), NO_VIEW, NO_CAM_HEIGHT in every mode, BATT climbing and descending (23.10).
- `test_auto_fsm` `testSetHeight`: NOT_WAITING in IDLE, ARMED, pre-spin, the climb, HOVER before the wait, LAND, LAND
  after the touchdown, KILLED and LAND SINK; the accepted command's ACK (old estimate 100 cm), hShift on that step
  only, progress 0 on it, the reference at old + d, no event, DONE with hOff 0 and the estimate back; the absolute
  form; the order (NO_VIEW and NO_CAM_HEIGHT before SH_RATE), the rate at 1999 / 2000 ms; X 9 / 251, d 0 cm, |d| 0.50
  / 0.51 either way, the absolute form's 0.504 / 0.506 either way (whole cm), h_new 0.10 / 0.09 and the cap + 0.50 / +
  0.51, |upM| 0.56 above the cap (and the flight to the cap); NO_VIEW, NO_CAM_HEIGHT, BATT climbing and descending;
  SH_CONTACT in the hush and in the window, accepted after it; LAND, k and BATT_LOW during a correction; the ARM
  clearing the rate bound.
- `test_auto_proto` and `gen_auto_proto_golden.py`: `set_height`, `set_height_now` and both ACKs, reproduced by C and
  Python; `auto_link_host`: 0x33 routed with X and h_at (INT16_MIN too), 3 bytes or X < 0 RANGE detail 0 (the unknown
  type of the older tests moved to 0x34); `test_autonomy_glue_host set-height-flight`: the ACK's old estimate, the
  status of the accepting step already shows the shifted estimate (applied before vzRun), the plant flies the -0.20 m
  correction (-0.21 m), BUSY while it runs, NOT_WAITING in IDLE with a nonzero detail.
- Python: the formula and its signs, the calibration on the living-room numbers, staleness (boot, invalidation and
  re-calibration), the 4x4 rule and the dash fraction, out of frame, the raw .png byte-identical, the JSON block, the
  capture mapping (a synthetic ring and offsets), set-height's offset form from the latest photo, the twice-used photo
  and another sortie refused, --now and --at, SH_STEP and SH_RATE hints, the old firmware, wait-move naming the
  correction, the invalidations on a landing, a KILL, a contact and a reboot; the ceiling rule's removal.
- Mutations: 39 of 40 firmware mutations caught (the state machine's BATT check is duplicated in
  `autoMotionSetHeight`: equivalent; 5 of them on the whole-cm step check), 15 of 15 in the Mac tooling.

**Closed loop** (`test_interactive_sim` `testSetHeightLoop`: the real state machine, motions, controller and hold on
the sim plant): a scripted Claude takes a 'photo' P = 12 s after the previous correction's acceptance or photo (and
1 s after a correction of >= 0.3 m ends, the runbook's confirming photo), judges X = the true height + N(0, 0.07)
clipped +-0.15 + a bias 0 / +-0.05, sends the offset form L = 8 or 15 s later when |X - h_at| > 0.10; 150 s of air;
the baseline flies the same seeds without commands. The error is against the height the drone held at the wait's
start (what a correction flies back to; the climb ends within holdBand, 0.05 m, of its target). At 1.00 m (5 seeds per
cell):

| c (m/s) | L 8 s: error p50 / p95 / max (m) | L 15 s | at 150 s: with / without (m) |
|---|---|---|---|
| -0.030 | 0.59 / 0.87 / 1.01 | 1.17 / 1.82 / 2.22 (cannot keep up) | 0.51-1.68 / 4.5 |
| -0.015 | 0.26 / 0.46 / 0.60 | 0.39 / 0.63 / 0.82 | 0.33-0.44 / 2.2 |
| -0.005 | 0.08 / 0.21 / 0.30 | 0.16 / 0.34 / 0.45 | 0.09-0.23 / 0.55-0.83 |
| +0.005 | 0.09 / 0.21 / 0.29 | 0.13 / 0.27 / 0.36 | 0.06-0.15 / 0.64-0.73 |
| +0.015 | 0.25 / 0.49 / 0.62 | 0.42 / 0.70 / 0.86 | 0.30-0.39 / 2.1 (the floor at 63-114 s) |
| +0.030 | 0.50 / 0.76 / 0.87 | 0.71 / 0.91 / 0.93 (the floor) | 0.44-0.90 / 4.2 (the floor at 39-54 s) |

- Passed: never KILLED or cut in the air; no shift refused; no PROGRESS stop; within |c| (P + 2 L) + 0.25 m wherever
  Claude keeps up; at 150 s at most 1/3 of the baseline for |c| >= 0.015 (the baseline extrapolated at |c| x 141 s
  where it reached the floor first); single +-0.50 m corrections DONE in 4.1 s, within 0.02 m of their target, the
  thrust within 3.4-3.6 % of its 2 s mean (3.2-3.4 % in a plain hover; in the loop 4.4-5.2 % against 4.0-5.0 %).
- **Honestly:** at |c| = 0.03 m/s with 15 s from photo to command one 0.5 m command a cycle (L + the ~4 s flight + the
  1 s confirming photo) cannot keep up with the drift (0.5 m / 20 s = 0.025 m/s): climbing, the error grows to 2.2 m in
  150 s; sinking from 1.0 m, the drone reaches the floor between corrections (4 of 5 runs, K_RANGE / EST_RANGE LANDs
  at the floor). With 8 s it keeps up (the floor not reached). Hence the runbook's < 10 s.
- The camera's vertical scale +-25 % (c +-0.015, report only): error p50 0.33 m, max 0.69-0.83 m, no PROGRESS; a
  single 0.50 m correction then ends 0.08-0.12 m off its target (the next photo catches it).

### 23.7 Cost (the build of this section and 23.9, against 1e4184b)

- `.iram0.text` 129559 -> 129591 (+32 B: `stepFull` +12, the call to `applyHeightShift` inside `if (took)`;
  `autoFsmStep` +6, the dispatch of the new command; `vzStep` +4; alignment). Text and its padding (229 -> 197 B) still
  end at 129788, 56bd936's 256 B step: the gate holds, no internal RAM lost to IRAM.
- `.dram0.bss` 88248 -> 88264 (+16 B: `setHeightMs`, `shD` / `shOff0`, `shiftRefused`), `_heap_start` 0x3fcc12f0 ->
  0x3fcc1300: 16 B less internal heap, ~19.7 KB expected (bench L0 / M0: >= 19.5 KB; below 19 KB NO-GO).
- Flash: `.flash.text` +1680 B, `.flash.rodata` +40 B, the image 1114464 -> 1116224 B. The new functions are flash,
  once per command: `setHeightCmd` 974 B, `autoMotionSetHeight` 382 B, `vzShiftHeight` 66 B, `applyHeightShift` 60 B.
- `callgraph.py` from `autonomyArbitrate` and `stabilizerTask`: the same 86 IRAM functions as 1e4184b; one new flash
  reference, `applyHeightShift` from `stepFull`, called only on a step that took a command (and it returns at once
  unless that command was an accepted SET_HEIGHT). The 2.00 m cap changes constants only (no code path).
- The review fixes (23.10), against ee10514: `.iram0.text` 129591 (unchanged; + 197 B padding = 129788, the gate),
  `.dram0.bss` 88264 and `_heap_start` 0x3fcc1300 unchanged (~19.7 KB), `.flash.text` 764123 -> 764119 B, the image
  1116224 B. Every change is flash code or a constant: `startLand`'s timeout (flash since 22.8), `precheck` and
  `autoMotionSetHeight` (cold), `setHeightCmd`, `vzConfigValid`, `CAM_VZ_REANCHOR_MAX_S` back to 6.0 (the observer's
  per-block path as flown).

### 23.8 Risks

- **A wrong judgement is flown.** The drone checks only generic bounds; a judgement 0.5 m off in the wrong direction
  is flown at 0.15 m/s. Bounded per command (0.50 m, one per 2 s, one at a time) and by the runbook (two references
  within 0.1 m, or one clear reference on the line; a confirming photo after >= 0.3 m). The camera's vertical scale
  (+-20-30 %) shows as 0.08-0.12 m left after a 0.50 m correction (the next photo catches it).
- **The delay.** The error left is about |c| x the photo-to-command time. At |c| 0.03 m/s and 15 s, one 0.5 m
  command a cycle cannot keep up: the drone reached the floor in the sim (4 of 5 runs at 1.0 m) or rose 2.2 m above
  the intended height. Hence < 10 s and the faster cadence above 1.2 m.
- **A stale level line** (a new boot, a landing, a contact, a KILL without a new `level-cal`) is drawn dotted; a
  mount tilt 5 deg off puts the line 0.26 m off at 3 m. The daemon marks it stale; it never blocks a command.
- **Logs:** after corrections the recorder's camera height (cvH) differs from the controller's (vzH) by their sum;
  the daemon's `set_height` events line them up. The LAND's drift allowance does not know the corrections
  (conservative: a correctly estimated drone still lands by the stall rule).
- **The 2.00 m cap (23.9):** in a room with a 2.4-2.7 m ceiling a hover at 2.0 m has 0.4-0.7 m above it, less than
  the error the sim leaves between photos (the loop at 2.0 m with 7 s photos: the highest true height 2.41 m at c
  -0.015 m/s, 2.81 m at -0.03 m/s with an 8 s delay); a LAND pressed against a ceiling stops the motors up there (18.3
  finding 1). A fall from 2.0 m (a KILL, the cut) reaches the floor at ~6.3 m/s (4.9 from 1.2 m). An end-of-battery
  LAND from 2.0 m with a weak battery touched down at 0.67-1.03 m/s in the sim (C 150-250 A s, 12-23 s of air from
  3.82 V; 0.47 at the measured ~400); with the runbook's descent from ~52000 / 3.20 V every LAND of the measured
  battery came from <= 1.31 m (23.10). The runbook's rules (>= 0.6 m judged ceiling clearance, nobody within 2 m above
  1.5 m, down to <= 1.2 m from ~3.20 V loaded) are Claude's, not the drone's: the drone enforces the 2.00 m hover cap,
  the 1.20 m take-off cap and the battery's refusals.
- **The camera lost for good during or right after a vertical move** (a known limit, as flown; 23.10): the open loop
  flies at most capQ, the latest quiet window, which a move's settle can bias; the drone drifts slowly while the
  accelerometer estimate drifts its own way, and the backstop LAND (the state machine's SINK at 5 s, the controller's
  HOLD_NO_CAM at 7 s) starts with an estimate off by up to ~0.7 m. In the controller sim: 0.60 + 0.60 m to 1.20 m (the
  firmware flown) touched down at up to 0.94 m/s, 1.20 + 0.80 m to 2.00 m at up to 0.98 m/s (> 0.6 in 40-46 of
  230-260 runs); with the camera lost in a settled hover <= 0.35. Not a regression; not fixed here.

### 23.9 The 2.00 m limit (user decision (4) 2026-10-15: the timings scaled from the height)

The hover goes to 2.00 m: the moves' cap `AUTO_MV_H_MAX_M` and the controller's `VZ_H_TARGET_MAX` (1.20 until now; the
height offset `VZ_HOFF_MAX` and SET_HEIGHT's range follow it). **The take-off stays at 1.20 m** (`VZ_H_TAKEOFF_MAX`,
`AUTO_HEIGHT_CM_MAX` 120, the Mac's `HEIGHT_CM` 20..120): 2.00 m is reached by MOVEs up from a checked hover, as the
runbook (23.5) does anyway. Why (the review of 2026-10-15, 23.10): a LAND during the take-off climb has no measured
hover yet, so its ceiling falls from landStart at once (`startLand`: landSlow needs hoverKnown); a climb to 2.00 m
lasts ~5.2 s, and every LAND reason that can come in it (the Mac's or the user's LAND, HB_LOST after 1 s, APP_INPUT,
SESSION, VBAT_LOW, VBAT_DIP, LOOP_GAP, TAKEOFF_TIMEOUT, CLIMB_TIMEOUT, a contact in the climb) touched down at 1.8-2.05
m/s from 1.9 m in the controller sim (1.06 from 0.6 m, 1.3-1.5 from the 1.20 m climb flown since A2a). A LAND from
HOLD after the moves is slow (<= 0.32 m/s, below). The direct 2.00 m take-off was offered by the plan; a slow LAND from
the climb would need a hover reference the climb does not measure (its thrust average carries the climb's drag and
acceleration: landTop x it flew the blind open loop up, not down), so it was left out rather than designed in a hurry.

The height-dependent timings were already formulas of the target (`vzConfigSetHeight`: climbMaxS 2.0 + (h - 0.40) /
0.40, landFlatS 2.0 + (h - 0.60) / 0.30 above the knee, hMax h + 0.6, the LAND timeout 6.0 + (h - 0.60) / 0.30; the
state machine's pre-wait limit + 25 ms per cm above 60, its LAND backstop 6.5 s + the same extra). A move up adds its
height to a LAND's own flat part, timeout and backstop (`landTimes`, `landExtraMs`), so a LAND from 1.20 + 0.80 m has
exactly the 2.00 m take-off's: flat 6.67 s, timeout 10.67 s, backstop 11.17 s, hMax + hOff 2.6 m. The config's hard
limits follow the take-off cap and stay as flown:

| | the take-off cap 1.20 m (both builds) | the hover cap 2.00 m |
|---|---|---|
| `VZ_CLIMB_MAX_LIMIT_S` = 2.0 + (take-off cap - 0.40) / 0.40 + 0.5 | 4.5 s (climbMaxS at 1.20 m: 4.0) | - |
| `VZ_LAND_FLAT_LIMIT_S` = 2.0 + (take-off cap - 0.60) / 0.30 + 0.5 | 4.5 s (landFlatS at 1.20 m: 4.0) | (a LAND's flat part after moves: 6.67 s) |
| `VZ_HOFF_MAX` = `AUTO_MV_HOFF_MAX` = (hover cap - 0.20) + 0.50 | - | 2.30 m (1.50 at a 1.20 m hover cap) |
| `AUTO_HEIGHT_CM_MAX` (= `VZ_H_TAKEOFF_MAX`), `AUTO_MV_H_MAX_M` (= `VZ_H_TARGET_MAX`), asserted | 120 cm | 2.00 m |
| hMax (+ hOff), the LAND timeout, the LAND backstop | 1.8 m, 8.0 s, 8.5 s | 2.6 m, 10.67 s, 11.17 s |
| the pre-wait limit (default / highest) | 7.5 / 9.5 s | - |
| `CAM_VZ_REANCHOR_MAX_S` = climbMaxS(take-off cap) + 2.0 (asserted in cam_vz_glue.c) | 6.0 s (as flown) | - |

The Mac: `HEIGHT_CM` 20..120, `MOVE_H_RANGE_M` 0.25..2.00; a drone before this build refuses a move above 1.20 m with
HEIGHT. The controller refuses a 2.00 m take-off config (vzInit CUT BAD_CONFIG, before any thrust).

**Verification (host):**
- `test_vert_ctrl`: the limits (4.5 / 4.5 s, |hOff| 2.30 m), vzConfigSetHeight 1.21 and 2.00 invalid, hTarget 1.21,
  climbMaxS 4.51, landFlatS 4.51 refused, 1.20 / 4.5 / 4.5 accepted. `testHigh200` (20 sorties per row; the 1.20 m
  take-off moved up at 0.15 m/s): at 1.50 / 2.00 m the camera's LAND touches down <= 0.33 / 0.29 m/s in <= 6.2 / 7.3 s
  (timeout 9.0 / 10.67, exactly the take-off's); a camera offset +-0.01 m/s <= 0.34 / 0.32; the camera lost in the
  settled hover (HOLD_NO_CAM) <= 0.34 / 0.35; a 0.60 m take-off moved up to the height <= 0.33 / 0.31 (the same
  timeout); after 60 s of HOLD with a drifting estimate <= 0.34 / 0.36 m/s; the blind LAND (5 and 60 s of hover, e
  -1..+1 m/s) <= 0.88 / 0.92 m/s, no climb, >= 1.16 s inside its timeout; the fast LAND (damping 0.02-0.07, e
  -0.2..+0.4 m) <= 0.47 / 0.49 m/s, the brake within its rules; a 2.00 m take-off CUT BAD_CONFIG.
- `test_auto_fsm`: TAKEOFF 121 and 200 RANGE, 120 accepted; from 1.20 m moves up 0.50 + 0.10 to 1.80 m, then 0.20 OK
  and 0.21 HEIGHT, 0.01 HEIGHT at 2.00; the LAND backstop 11167 ms after 1.20 + 0.50 + 0.30 m (8500 from 1.20 m).
  `test_autonomy_glue_host`: margin-120 (margin-200 removed: a 2.00 m TAKEOFF is refused).
- `test_interactive_sim` (the whole stack): TAKEOFF 1.20 m moved up 0.50 + 0.30 m (9 sorties, camera offsets 0 /
  +-0.01 m/s): no TAKEOFF_SLOW, both moves accepted, true peak <= 2.20 m, the Mac's LAND <= 0.31 m/s; 1.00 m moved up
  0.50 + 0.50 m: <= 0.32 m/s; the battery drained at 2.00 m (C 150-600 A s, K -5k..+5k): BATT_LOW in 12 of 12, never
  killed, capped or cut in the air; the measured battery (C >= 400) <= 0.47 m/s, the weak end up to 1.03 m/s
  (reported, bounded at 1.1; 23.8, 23.10). The SET_HEIGHT loop at 2.00 m with 7 s photos (reached by the moves):

| c (m/s) | L 8 s: error p50 / max (m), highest true height | L 15 s |
|---|---|---|
| -0.030 | 0.52 / 0.88, 2.81 m | 1.18 / 1.99, 3.92 m (cannot keep up) |
| -0.015 | 0.25 / 0.48, 2.41 m | 0.30 / 0.68, 2.61 m |
| -0.005 | 0.11 / 0.31, 2.24 m | 0.07 / 0.33, 2.25 m |
| +0.005 | 0.08 / 0.26 | 0.12 / 0.33 |
| +0.015 | 0.22 / 0.57 | 0.43 / 0.78 |
| +0.030 | 0.50 / 0.86 | 1.05 / 1.94 (cannot keep up; 1 run reached the floor) |

  (The same as with the 2.00 m take-off before, within the seeds' spread.) Single +-0.50 m corrections at 2.00 m: DONE
  in 4.07-4.11 s, within 0.017 m. Never KILLED or cut, no shift refused,
  no PROGRESS stop. The sim has no ceiling: the highest heights are what a 2.4-2.7 m ceiling would meet (23.8).
- Python: `HEIGHT_CM` against `AUTO_HEIGHT_CM_MAX` and `VZ_H_TAKEOFF_MAX`, `MOVE_H_RANGE_M` against `AUTO_MV_H_MAX_M`
  and `VZ_H_TARGET_MAX`; `--height 120` sent, 121 a usage error; the Mac's move refusal at 1.80 + 0.30 m.

**What the data and the sims say about 1.5-2 m, honestly:**
- **No flight data above ~1.27 m true height.** Every logged hover (the gym 1.08-1.27 m true, the living room ~1 m)
  is lower. The camera height observer's behaviour at 1.5-2 m is untested beyond synthetic scenes: G = f / D_eff is
  set mostly by the scene distance, and the floor's part (as 1 / h) halves from 1 m to 2 m, so the analyst expects the
  estimate no better there and probably 1.1-1.35x worse per second (2-3x in big halls, D 6-10 m): 0.012-0.025 m/s at
  2 m in rooms against the logged mean +0.004, RMS 0.011 m/s (worst sustained -0.016). Hence photos every 5-10 s above
  1.2 m. The floor-wall line leaves the view at 2 m for walls nearer than 3.4 m.
- **The climb.** Logged take-off climbs (cal/flight-2026-10-1[2-5], vzPhase 3 -> 4) that ended at 1.05 m took
  2.78-2.87 s of their 3.75 s limit, at 1.15 m 2.97-3.03 s of 4.0 s; the sim's nominal 1.20 m climb 3.2 s of 4.0. With
  the take-off cap at 1.20 m these are the climbs that fly; 2.00 m adds two MOVEs up (0.50 m in ~4.1 s, 0.30 m in
  ~2.75 s, each plus a settle of <= 3 s). (A 2.00 m climb would have taken ~5 s of 6.0, extrapolated and simulated.)
- **K, Khat and vzLearnK.** The hover gain is flat with height in the logs: 156-167k from 0.35 to 1.15 m; nothing is
  expected to change at 1.5-2 m unless the drone is within ~0.1-0.2 m of a ceiling (ceiling suction: less thrust needed,
  Khat falls; at khatMinRel the K_RANGE event, 22). vzLearnK's sample is taken once, at holdLoopS of the first HOLD
  (the take-off height, <= 1.20 m), so the moves to 2.00 m do not touch it; Khat adapts on the camera in HOLD through
  the moves as through any vertical MOVE. The sim's plant has no height-dependent K, so it cannot show more.
- **Energy and the battery.** The climb's energy is negligible (~0.27 J/m: 0.8 m of moves ~0.2 J); in the drain sim
  (3.82 V at rest, C 400 / 600 A s, K -5k..+5k) BATT_LOW came at the same air time at 0.60, 1.20 and 2.00 m (30.8-57.3
  s, mean 43.3 / 43.3 / 43.4 s); the BATT_LOW LAND took up to 3.9 / 5.4 / 6.4 s to DONE. A 2 m sortie costs the moves'
  ~10 s and the longer LAND (~1-3 s), not battery charge.
- **Landings from 2 m** (the controller sim, the moves' path above): every LAND kind slow with a correct estimate
  (<= 0.36 m/s; blind <= 0.92); with the estimate reading low after 20-60 s uncorrected the blind LAND reached 0.88-0.94
  m/s in the analyst's sim (0.66 from 1 m). None of it is flight-verified.


### 23.10 The review of 2026-10-15: what changed and why

1. **The take-off stays at 1.20 m; 2.00 m by MOVEs up only** (`VZ_H_TAKEOFF_MAX`, `AUTO_HEIGHT_CM_MAX` 120, the Mac's
   `HEIGHT_CM`; 23.9). A LAND during a climb to 2.00 m touched down at 1.8-2.05 m/s in the controller sim (a LAND 0.4 s
   before HOLD, K x0.94-1.06, offsets +-10 mg: 1.81-2.02 m/s against 1.30-1.49 from the 1.20 m climb; a contact in
   the climb at 1.9 m, blind: 2.05). The timing limits and the re-anchor window are back at their flown values (4.5 /
   4.5 s, 6.0 s); the 2.00 m tests reach the height by moves (testHigh200, testHigh200Sim, the SET_HEIGHT loop).
   **This narrows the approved plan's (4) "take-off height range 2.0": the user should confirm it**; the hover cap,
   the moves and SET_HEIGHT go to 2.00 m as approved.
2. **The LAND timeout after a move down** (`startLand`'s plain branch, flash): a take-off above the knee moved down to
   it or below and LANDed within 18 s of HOLD kept the take-off height's flat part (4.0 s from 1.20 m, 6.67 s from a
   2.00 m take-off) with the knee's 6.0 s timeout: the 2.00 m take-off's LANDs ended LAND_TIMEOUT up to 0.67 m in the
   air (26 of 54 runs at c -0.03..-0.05 m/s), the 1.20 m one's on the floor (a CUT where a touchdown was due). Now
   max(6.0, the flat part + 4.0 s), as `landTimes`: 8.0 s from 1.20 m; the state machine's backstop (8.5 s there)
   stays above it. `testLandAfterMoveDown`: 1.20 m moved down 0.60 / 0.70 m, LAND 15.5-17.5 s into HOLD, c 0 / -0.03 /
   -0.04 m/s: a touchdown in 108 of 108 (80 before the fix), <= 0.29 / 0.53 / 0.82 m/s.
3. **Nothing flies near the end of the battery, a descent included.** The review asked for a vertical MOVE down under
   battLow (as a descending SET_HEIGHT was accepted). In the drain sim (2.00 m reached by moves, one 0.50 m descent sent
   once T_hold passed a level, C 150-600, K -5k..+5k, 8 seeds) a descent accepted at T_hold 54600-56000 ended
   HOLD_CAPPED (the braking and the loop's swing at the 58000 ceiling for 0.5 s: a fast LAND, 1.22-1.53 m/s) in 1-2 of
   37-50 sorties, the descending SET_HEIGHT exactly as the MOVE; none at 53400-54000. So the refusal stays for MOVE
   and now covers every SET_HEIGHT correction too (`autoMotionSetHeight`, `setHeightCmd` check 10). The runbook comes
   down from T_hold ~52000 or a 3.20 V reading (23.5): with it every BATT_LOW LAND of the measured battery (C >= 400)
   came from <= 1.31 m (drainOk's bounds); the weak grid (C 150 / 250: 12-23 s of air from 3.82 V, where the climb to
   2.00 m itself breaks the runbook's 3.25 V rule) still touched down at 0.67-1.03 m/s from 1.6-1.9 m:
   **0.6-1.0 m/s for a nearly empty battery at 2 m is the risk the user has to accept**, or keep the drone at
   <= 1.2 m on a battery that reads below ~3.25 V loaded.
4. **The Mac** (`tools/espfly_auto.py`): a set-height whose outcome is unknown marks the photos up to then as used
   (23.4); `level-cal` refuses a photo taken before the setting's latest invalidation (kept as `valid_after_t`) or from
   another boot, chooses the latest photo by when it was taken (`photo_time`; new photos carry `link.received_t`), and
   reports the photo's age (23.3); `height_check_due`, a warning only, succeeds the removed ceiling warning (23.4:
   **the removal of the ceiling rule was not in plan items (1)-(4); the user should confirm it**, and this warning
   too); the Python suite checks the SET_HEIGHT constants against the headers (`AUTO_SH_X_CM_MIN/MAX`,
   `AUTO_SH_MAX_M`, `AUTO_SH_GAP_MS`, `AUTO_SH_AT_NOW`, `AUTO_MV_FY_PX`), and the fake drone uses the same numbers.
5. **Found on the way, not changed** (23.8): the camera lost for good during or within ~3 s after a vertical move
   gives the backstop LAND an estimate off by up to ~0.7 m (touchdown up to 0.94 m/s at 1.20 m as flown, 0.98 at
   2.00 m, bounded in testHigh200).

## 24. Colour photos (user request 2026-10-15: "allow colour")

**Replaced by section 28 (2026-10-16):** colour is now split out of the camera driver's own copy (the flow keeps
grey's frame and cost; no `lumaBuf`); this section stays as the record of the first colour build and its bench
findings.

Claude decides from photos, and colour tells a person, a wall and a floor apart where grey does not. With NVS
`photocol` 1 the photos are colour while the optical flow, the drift hold and the camera height observer keep the
8-bit grey frame they had: the same luma values, every frame. **Colour is opt-in** since the review of 2026-10-15
(24.7): the default `photocol` 0 runs the camera exactly as before (Y8 output, grey photos, the 40-byte META), so a
flash of this build changes nothing in the camera path; `auto set photocol 1` + reboot turns colour on after bench N0-N3
pass with the daemon restarted from this tree.

### 24.1 How the colour frame is captured (esp32-camera 2.1.0 in managed_components, not patched)

The camera ran `PIXFORMAT_GRAYSCALE` at QQVGA: the OV3660 sends Y8 (`ov3660_settings.h` `sensor_fmt_grayscale`:
FORMAT_CTRL 0x00, FORMAT_CTRL00 0x10), and `ll_cam.c ll_cam_set_sample_mode` takes 1 byte a pixel from it. The
options for a colour frame, from the driver code:

| Option | What the driver does | Hold interruption | Internal RAM |
|---|---|---|---|
| (a) `esp_camera_deinit` + init in YUV422 for one capture, then back | sensor reset and the whole register list (SCCB at 100 kHz, `ov3660.c` `reset` / `init_status` with their 100 ms delays), the auto exposure and white balance start over, `cam_config` frees and re-allocates the 15360-byte DMA buffer, the descriptors, the queues and the 6 KB `cam_task` stack | two re-initialisations, ~1-2 s without frames (the hold coasts, the observer refuses; past CAM_LOST's 5 s if a restart retries) | a 15 KB contiguous DMA block re-allocated in flight with ~18.4 KB free: a fragmented heap leaves the camera dead for the rest of the flight. **No.** |
| (b) `sensor->set_pixformat()` at run time | writes the two format registers only; `cam_hal.c` keeps the sizes of `cam_config` (`in_bytes_per_pixel`, `recv_size`, `fb_size`, the DMA node and half-buffer sizes from `ll_cam_calc_rgb_dma`). A 38400-byte frame into a 19200-byte layout overflows (`FB-OVF`, the DMA stopped); the other way round fails `FB-SIZE` (`len != fb_size`) and the frame is dropped. The fields are in a static `cam_obj`: changing them means patching the managed component (re-downloaded by the component manager) | not possible without a driver fork | - |
| **(c) YUV422 all the time, the flow gets the Y bytes** | `ll_cam_set_sample_mode`: 2 bytes a pixel; `ll_cam_calc_rgb_dma` for 320-byte lines: 3840-byte nodes, 2 half-buffers of 7680 (24 lines), a **15360-byte** DMA buffer, the size of the Y8 one (4 x 3840): both are capped by `CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX` 16384 | **none**: every frame is processed, a photo is the copy of a frame the flow uses anyway | **0** (the DMA buffer, descriptors and frame headers keep their sizes; the event queue is 1 deep instead of 3) |

(c) is taken. The sensor's timing does not change with the format: `ov3660.c set_framesize` calls the same
`set_pll(false, 8, 1, 0, false, 0, true, 8)` for every non-JPEG format at QQVGA, so the frame stays 2300 x 783 sensor
clocks (cam_clock.h): 14.81 fps at 13.33 MHz, the same rolling shutter and the same gyro window offset (26 ms). Y8 and
YUYV select the output of the same ISP YUV path (FORMAT_CTRL 0x00 in both, FORMAT_CTRL00 0x10 against 0x30), so the Y
bytes are the values the Y8 output gave (auto exposure and white balance run in both). The DVP carries 320 bytes a
line instead of 160: 48 us at the 6.67 MHz PCLK (XCLK / 2), inside one 86 us sensor line, with ~550 us between two of
the 120 output lines (768 readout lines / 120); the driver's `cam_task` (core 0, priority 23) copies 38400 instead of
19200 bytes a frame from the DMA buffer into the PSRAM frame buffer (2 copies of 7680 per 13 ms half-buffer instead of
4 of 3840). 16 MHz (the driver's PSRAM DMA mode) stays excluded (cam_clock.h).

### 24.2 The firmware

- `camera_flow.c`: `photoColour` from NVS `photocol` (`auto_glue_core.h AUTO_SET_PHOTOCOL`, u8 0/1, default 0), read
  once in `cameraFlowInit` (the frame buffers' size is the camera's for the boot), with its PSRAM luma buffer
  (`lumaBuf`, 19200 bytes; none: grey photos, a warning). `cameraStart` picks `PIXFORMAT_YUV422` or
  `PIXFORMAT_GRAYSCALE`; `frameGet` wants the whole YUYV frame (38400 bytes, word-aligned) in colour. Per frame, after
  the stale-frame check: `ofLumaFromYuyv(fb->buf, lumaBuf)` and the fit, the brightness sum and the exposure gate run on
  `lumaBuf` (the Y8 frame itself with `photocol 0`). `snapshotFill` copies the frame as captured (38400 bytes in
  colour) and names its format and length (`cameraFlowSnapshot_t.pixFmt`, `.bytes`: in the struct's padding, still
  24 bytes). `cameraFlowPhotoPixFmt()`; USB `status` prints `camPix=yuyv|gray`.
- `optical_flow.c ofLumaFromYuyv`: every even byte in order, two 32-bit words in, one out (flash code, ~10
  instructions a loop, once a frame). The fit itself is unchanged.
- `auto_link.c`: the PSRAM photo slots hold either frame (3 x 38400); a frame's size and chunk count come from its
  snapshot (`frameBytes`, `frameChunks`; a copy whose length is not its format's is sent as missing, META size 0);
  `CHUNKS` past a colour frame's 38 chunks, or a grey one's 19 once captured, is RANGE; the META is the A1 40 bytes
  for a grey or missing frame and 44 bytes with `pixFmt` for a colour one (`auto_proto.h AUTO_META_COLOR_LEN`,
  `AUTO_PIXFMT_*`, `autoMetaLen`, `autoPixfmtBytesPerPixel`; docs/autonomy-link.md "Photos").
- Nothing in the 1 kHz path, the state machine, the vertical controller or the landing rules changes. A photo cannot
  start CAM_LOST -> SINK or HOLD_NO_VIEW: no frame is skipped for it (the copy is the grey photo's memcpy at twice
  the bytes, in the same place: after the fit, before the buffer goes back).

### 24.3 The Mac (`tools/espfly_auto.py`)

`parse_meta` takes 40 bytes (pixFmt GRAY8) or 44 (`pixFmt`, `pixFmtName`, `color`); `Frame.assemble` checks the size
against width x height x bytes per pixel, and an unknown format is a final error ("update this tool, or set the
drone's photocol to 0"). `save_frame` decodes YUYV to RGB (`yuyv_to_rgb`: full-range BT.601, R = Y + 1.402 (V -
128), G = Y - 0.344136 (U - 128) - 0.714136 (V - 128), B = Y + 1.772 (U - 128), rounded and clamped; a pixel pair
shares its U and V; U = V = 128 gives the grey photo exactly) and writes RGB `.png` and `_x4.png`; the JSON has
`color` (true / false, also for grey photos), `pixFmt` and `pixFmtName`; the photo replies (`photo`, the daemon's
`photos`) and the `photo` event carry `color`. On a colour image the bearing ticks get a 1 px black edge and every
dash pixel of the level line a black pixel under it (two rows of a 4 x 4 block at most: no source pixel hidden), so
the yellow / magenta marks read on yellow, red or magenta scenery; a grey photo is drawn exactly as before.
`rerender_level` keeps the colour (the raw `.png` is RGB). An older Mac tool cannot read the 44-byte META (its
`parse_meta` wants exactly 40 bytes: the frame times out): set `photocol 0` for it.

### 24.4 Cost (this build against 7a78e1b, `idf.py size` / `size-components`)

| | 7a78e1b | colour | |
|---|---|---|---|
| static IRAM | 130618 B | 130618 B | 0 |
| internal .bss / .data | 88264 / 31544 B | 88272 / 31544 B | +8 B (`photoColour`, `lumaBuf`, the settings array's 11th value) |
| flash (.text + .rodata) | 953911 B | 954631 B | +720 B |
| camera DMA (internal, at start) | 15360 B | 15360 B (YUYV) | 0 |
| PSRAM | | +38400 (frame buffers 2 x 38400) +19200 (`lumaBuf`) +57600 (slots 3 x 38400) | +115.2 KB of 8 MB |

CPU, estimated (the host cannot measure it; autonomy-bench.md N): the luma pass streams 57.6 KB (38.4 KB read, 19.2
KB written with write-allocate) through the 32 KB data cache both cores share (octal PSRAM, 80 MHz), and evicts the
fit's own PSRAM state (`ofState_t`) on the way: **~1-2 ms a frame** on core 1 (0.3-1.0 ms was the first estimate,
too low by the camera review of 2026-10-15), inside `camProcAvgUs` (59.4-60.8 ms on the last builds with `flowfit
plain`; the bench gate 60500, the clock fallback 62000). A photo frame's `snapshotFill` copies 38.4 KB PSRAM to PSRAM
inside the flow task before the buffer goes back (~77 KB of traffic, ~1-2 ms, that frame only): the worst frame is
~60.5 + 1.5 + 2 ~ 64 ms against the 67.5 ms period, ~3.5 ms before a skipped frame (bench N1 reads `camProcMaxUs`
during a 3-photo push). The driver's copy of the second 19.2 KB: ~0.2-0.4 ms a frame on core 0 at priority 23 (the
stabilizer task has no core affinity, so it can share core 0). The link's CRC covers 38.4 KB (link task, priority 1)
and its push takes ~0.2 s a frame instead of ~0.1 s (38 chunks + META at ~200 KB/s; a set of 3 ~0.6 s instead of
~0.3 s: the Wi-Fi transmits twice as long per set, 24.6 item 6).

### 24.5 Verification (host)

`test_optical_flow` (+3): `ofLumaFromYuyv` takes the even bytes in order, and the fit of two YUYV frames' luma
equals the grey frames' fit bit for bit, with any chroma. `test_auto_link_host` (+10, `testColourPhoto`): a colour
set is two 44-byte METAs (YUYV, 38400, 38 chunks) and 76 CHUNKs, the last 512 bytes, the CRC over the whole YUYV
frame; CHUNKS 34..37 resent, past 37 RANGE, the META again 44 bytes; a grey set after it is the 40-byte META with 19
chunks and CHUNKS past 18 RANGE; a snapshot whose length does not match its format goes out as missing.
`test_auto_proto` (+11): the golden `meta_color` (auto_proto_golden_ih.json, its frame rule and CRC) and the format
helpers. `test_auto_glue_core` (+1): `photocol` u8 0..1, default 1 (0 since 24.7). `test_espfly_auto.py` (+9): the golden colour
META, the META lengths (40, 44, the rest refused; an unknown format kept as `unknown-N`), reassembly of 38 chunks
and the final errors, the YUV -> RGB values (8 cases computed from the formula, pairs sharing chroma), a chroma-free
YUYV frame saves the grey photo's pixels, the RGB `.png` and `_x4.png` pixels against the source, the ticks' and the
level line's black edges in colour only, `rerender_level` on a colour photo, an end-to-end `photo` from a colour
fake drone (a chunk past 19 re-requested, `color` true) and a daemon sortie with colour take-off photos (`photos`,
the events and the JSON). Every other suite unchanged (autonomy-bench.md N lists the numbers of this build).

### 24.6 Risks and what the bench checks (autonomy-bench.md N)

1. **The flow loop's time** (~1-2 ms a frame, estimated, 24.4). `camProcAvgUs` was 59.4-60.8 ms: in colour it may
   pass the 60500 gate. The 62000 rule acts on the ground only and switches the camera to 20 MHz for the boot
   (`camClockFallback`): the flow then processes frames 45 / 90 ms apart with a 15 ms gyro offset (~11 fps), not the
   operating point the height study, kyC (0.0294 px a frame is 0.33 px/s at 11.1 fps against 0.436 at 14.8) and the
   R1 replays assumed. N1 measures grey against colour on the same boot setup; over 60500 the user decides (colour
   photos or the margin); over 62000 or `camClkFallback=1`: `photocol 0`.
2. **The loop rate** (the A4 gate, >= 940): the driver's copy of twice the bytes on core 0 and more PSRAM traffic
   (the data cache is shared with flash rodata the 1 kHz path reads). N2 is the A4 run with `photocol 1`.
3. **A frame lost at the frame's end** (corrected by the camera review of 2026-10-15; the first text said a 13 ms
   window and one frame). In YUYV the driver's event queue is 1 deep (`cam_hal.c` sets it to `dma_half_buffer_cnt` -
   1: 1 for 2 half-buffers of 7680 B, 3 for Y8's 4 of 3840). Each frame sends 5 EOF events ~13.2 ms apart, then the
   VSYNC event. The tight window is the vertical blanking between the last EOF and the VSYNC: 15 of 783 lines x 86 us
   ~ **1.3 ms** (~0.9 ms at the 20 MHz fallback). If `cam_task` has not taken the last EOF by then, the VSYNC event
   cannot be queued (`cam_hal.c` EV-VSYNC-OVF; EV-EOF-OVF likewise), the driver stops and goes IDLE: frame N is never
   delivered and N+1 is not captured, **2 frames** (a ~202 ms `dt`). `camera_flow.c` numbers only the frames it
   processes (`s.seq = ++seq`), so the stream's `flowSeq` shows no gap: N3 reads the `flowDt_100us` maximum (<= 700)
   and the USB console's `cam_hal: EV-` lines. The risk is low: IDF FreeRTOS yields from an ISR to a task of equal
   priority, so the Wi-Fi task at `cam_task`'s priority 23 does not hold it off; interrupts-off sections or
   priority-24 work over ~1 ms would. A 2-frame gap stays under the 300 ms CAM_LOST report and the 5 s SINK.
4. **A Mac tool older than this build** cannot read a colour photo (its `parse_meta` wants exactly 40 bytes: the META
   counts as `bad_payload`, the frame never completes, every photo times out). The daemon running when this build is
   flashed is such a tool until restarted from this tree. Claude decides from photos: colour stays off (the default)
   until the daemon is restarted (bench N0).
5. **The Y bytes are the Y8 values only by the register settings** (`ov3660_settings.h`: 0x4300 0x10 against 0x30,
   FORMAT_CTRL 0x00 in both, no byte swap). The host tests prove that the even bytes give the grey frame's fit; no
   hardware check shows that the sensor's YUYV Y equals its Y8 output (a byte-phase error, a line starting on U,
   would feed the fit chroma: a nearly textureless plane, frames refused, CAM_LOST -> SINK in flight; a Y-range
   difference would move the texture, residual and exposure thresholds and kyG / kyC). Bench N1 compares lum,
   texture, the ok rate and the flow noise on one scene in both formats, a grey card's chroma and a red / blue
   object's U / V order before any colour flight.
6. **Wi-Fi transmits twice as long per photo set** (~0.6 s instead of ~0.3 s for 3 frames, the same 4 packets per
   20 ms): the peak current is unchanged, the time on the XIAO's marginal rail (2.5-2.8 V at a 2.95 V battery,
   earlier brownouts) doubles. Pushes are suppressed in the take-off window, not near the battery limit. Bench N4
   takes a set near the end of a battery in a DRY hover and compares `railMin` with grey; if colour dips lower, keep
   `photocol 0` (or grey near the battery limit, a later change).
7. Colour accuracy: the white balance is the sensor's automatic one, as it was for the grey frames' Y, and the Mac
   decodes full-range BT.601 (assumed, not measured: the grey card of N1 shows it). Neither touches the flow.

### 24.7 The review of 2026-10-15: colour opt-in

The code, camera and budget reviews of the colour build found the mechanism right (DMA sizes recomputed from
`ll_cam_calc_rgb_dma`, the luma pass, the snapshot, the 40 / 44-byte META, the Mac decode) and one major problem each
time: with `photocol` 1 the default, every flash changed the camera path the flow, the hold and the height observer
use on every flight, unmeasured (24.6 items 1-5), and the user asked to *allow* colour. Fixed: **`photocol` defaults
to 0**, so this build's camera is 7a78e1b's bit for bit until the user sets `photocol 1` (`auto_glue_core.c`;
`test_auto_glue_core` checks the default 0; `test_espfly_diag.py`'s fake `auto get` says `photocol=0(default)`). The
order to turn it on (autonomy-bench.md N): restart the daemon from this tree, then N0-N4 with `photocol 1`; any
FLIGHT in colour only after they pass. Corrected here: the frame-loss mechanism (24.6 item 3), the CPU estimate (24.4),
the luma equivalence, Wi-Fi and colour range risks (24.6 items 5-7).

## 25. The late offset noise: R1 of the height study (2026-10-15; built for host testing, default off)

The height study of 2026-10-15 (firmware-backups/handoff-2026-10-12/height-study-2026-10-15.md) found why a long
hover sinks after ~35-45 s: in HOLD the controller holds the camera observer's *speed* at 0 (vert_ctrl.c:908-935,
cam_vz_glue.c `camVzGlueVzInput`), and the observer's accelerometer-offset state (process noise `qBiasG` 0.0003
g/sqrt(s), cam_vz_obs.c `camVzObsDefaultConfig`) lags the real offset, which turns after ~20-40 s and rises 0.12-0.25
mg/s; the 1-2.4 mg lag reads as a climb and the drone answers with a real sink of 1-3 cm/s. Its section 4.2 proposed
R1: a faster offset late in the flight, with guards. The user approved building it for host tests (flashing is a
separate decision). This build has it, **off** by default (NVS `cvqlate` 1): with it off, and with it on in every
sortie shorter than its step, the observer and the controller run bit for bit as in 7a78e1b (25.5).

### 25.1 What the observer does (`cam_vz_obs.c`, header "Late offset noise")

- **The step.** From `qLateS` (NVS `cvqlates`, default 40 s) after the sortie's **first** lift-off the offset's random
  walk is `qBiasG` x `qLateMult` (NVS `cvqlate`, 1..5; 1..10 until the review, 25.9). The clock is `tFirstLift`: the lift-off of the first frame
  the filter takes (the camera wakes >= 1.5 s after the throttle-up), never restarted by a re-lift (`tLift` is,
  cam_vz_obs.c `sampleFlags`); a new sortie (`camVzObsReset` at every ground start, cam_vz_shadow.c `sortieStart`)
  restarts it. A bounce before the first frame only delays the step. The multiplier never runs on the ground (the
  filter keeps running after a touchdown: LANDED, a hop).
- **The guard.** The multiplier stops, and the offset's variance is held at its size at the step (`p22Norm`) plus
  what `qBiasG` alone adds since the previous frame (since 25.9; its covariances scaled by the same sigma ratio, which
  keeps P positive semi-definite), while any of these happened in the last 3 s (`CAM_VZ_QLATE_QUIET_US`):
  - a frame refused for what it shows: EXPANSION, LATERAL, NO_GAIN, the CUSUM hold, the innovation gate. Not QUALITY
    or NO_WINDOW: such a frame carries nothing into the filter, single ones come every few seconds in a good hover
    (3 % of the 10-15 p1 hover's frames), and a run of them is the next event;
  - no accepted update for 0.5 s (`CAM_VZ_QLATE_NOACC_US`: frames refused or stopped);
  - a CUSUM sum above half its alarm threshold (`CAM_VZ_QLATE_CS_REL`, the build-up) or its alarm;
  - a re-lock, a gain re-anchor or a yaw restart of the tracker; a G that is not tracked (a guess or the default);
  - a commanded turn (`camVzObsSetTurn`, as before) or motion (`camVzObsSetMotion`, new: a MOVE, a SET_HEIGHT's
    correction or a YAW to the end of its settle, auto_motion.h `holdWait`, or a contact's hush since 25.9, set by
    autonomy.c every controller step of a sortie and handed to the observer by cam_vz_glue.c before each block);
  - the offset moving more than 1 mg since its value 1-1.5 s before (`CAM_VZ_QLATE_RATE_G`, `CAM_VZ_QLATE_RATE_US`):
    the accelerometer's offset drifts <= 0.25 mg/s on every logged flight, so a faster move is the camera's. Added
    after the first fault runs (25.5): without it a small glitch the boosted offset swallowed (a 0.1 m/s false
    descent for 1-2 s, below the CUSUM build-up), followed by a lost camera, left x10 0.2-0.5 m worse than today.
- **The controller's offset.** vert_ctrl latches the offset of its last step with the camera in use and flies on it
  when the camera drops out (vert_ctrl.c `lastBias`, unchanged). The observer now gives it `biasHeldG`
  (`camVzObsOut_t`, used by `camVzShadowVzInput` and `camVzGlueVzInput`): `biasG` itself before the step and with
  the multiplier 1; past the step the mean of the offset 2.0-4.5 s ago (one history entry per 0.5 s, 10 entries), and
  **frozen while the guard holds**, for 4.5 s of one guard run at most (`CAM_VZ_QLATE_FREEZE_US`, since 25.9; then
  the delayed mean again, of an offset that moved at `qBiasG` alone). A camera lost after a glitch therefore leaves the controller the offset from at
  least 2 s before the glitch's first event (the frames stop: the no-update event 0.5 s after the last accepted frame,
  the health 2 s after it). The mean rather than one old value: the boosted offset is noisier (+-1 mg), and the open
  loop until the SINK (5-7 s) turns 1 mg into ~0.2 m.
- **Where it runs.** All of it in the frame path (flash, ~15 Hz): `latePre` before a frame's update (the step, the
  block-level events, the variance bound before the update can use it), `latePost` after it (the frame's events, the
  multiplier's flag for the blocks up to the next frame, the history, `biasHeldG`). The 10 ms block path (IRAM) only
  multiplies `qBiasG` when the flag is set and the drone is not on the ground; the flag sits in padding (`lateOn`
  after `onGround`), so no offset the block path reads moved. Between frames the multiplier grows P[2][2], and through
  it P[1][1] and P[0][0] (the first text said P[2][2] only: corrected by the review); the next frame's `latePre`
  bounds P[2][2] and its cross terms before its update, not P[1][1] or P[0][0], so the frames back after a gap weigh
  more on the speed (benign; the offset itself stays protected: 1 s after a 1 s gap x5 moved it 0.12 mg, x1 0.70 mg).
  A camera that stops entirely (no frame to run `latePre`) lets the variance grow until the next frame, where it is
  bounded before any update takes it.

What it does not change: the vertical controller (vert_ctrl.c untouched), SET_HEIGHT, MOVE, the camera gates, the
0.050 g limit on the controller's offset (`VZ_CAM_BIAS_MAX_G`), every landing: K_RANGE and CAM_LOST -> SINK stay as
they are (user decision 2026-10-15), and the allowed automatic landings. `kyC` is unchanged (25.4).

### 25.2 Settings and reports

- NVS `cvqlate` (u8 1..5 since 25.9, default 1 = off) and `cvqlates` (u8 s 20..120, default 40), `auto_glue_core.h
  AUTO_SET_CVQLATE`, `AUTO_SET_CVQLATES`: `auto set cvqlate 5` over USB (`python3 tools/espfly_diag.py auto set
  cvqlate 5`), like every autonomy setting (`auto get`, `auto reset`; never over Wi-Fi). **Read at boot**
  (cam_vz_glue.c `camVzGlueInit`): a change takes effect after a reboot; `auto get` shows the stored value, `camvz`
  the one running. Every value in range is a valid observer config (asserted; `test_auto_glue_core` checks all
  5 x 101).
- **Refused loudly.** A value out of range (`auto set cvqlate 6` answers `ERR auto set: cvqlate must be 1..5 x`), or
  a stored one that is out of range or cannot be read at boot: the boot log's warning, `camvz` `qlateCfg=bad`, the
  flight summary's `FSUM_CVL_CFG_BAD`, the diag tool's "STORED SETTING REFUSED", and the multiplier 1 (off). The
  rest of the config is never dropped for it: `camVzObsInit` falls back to the whole default on any invalid field (the
  camera's gyro window offset of 17.2 included), which the glue ignored; the glue now checks its result (a refusal,
  impossible with in-range settings, is reported the same way and keeps the camera's offset).
- **USB `camvz`**: `qlate=<x> qlateS=<s> qlateCfg=ok|bad late=off|on|guard lateMs=<ms on this sortie>
  lateGuards=<stops> biasHeldMg=<the controller's offset>` (the diag tool prints a line from them).
- **The RTC flight summary** (`flightSummary_t.camVzLate`, `fsumCvLate_t`, 6 bytes + 2 of alignment: 148 -> 156 B,
  `FLIGHT_RTC_MAGIC` 0xF1A6C0E8, the old summaries cleared once at the first boot of this build): the multiplier and
  step in use, `FSUM_CVL_RAN | STEP | CFG_BAD`, the guard stops, the time with the multiplier on (0.1 s). USB
  `flights`: `cvqf= cvq= cvqS= cvqG= cvqDs=` on each `F` line; `python3 tools/espfly_diag.py flights` adds "late noise
  x5 from 40 s: on 46.3 s, 1 guards" to the camera observer's notes.
- **Not in AUTO_STATUS.** Its 136 bytes are all assigned (bytes 94-95 `airLeftDs` are 0 since the air clock went, but
  the Mac parses them), and a longer message needs a Mac change in step: left out. The Mac reads which version flew
  from the summary after the flight, and the user sets it over USB anyway.

### 25.3 The firmware

`cam_vz_obs.h/.c` (the config's `qLateMult`, `qLateS`, appended; `camVzObsOut_t.biasHeldG`; the state appended, the
three flags the block path reads in padding; `camVzObsSetMotion`, `camVzObsLateState`); `cam_vz_shadow.h/.c`
(`camVzShadowVzInput` on `biasHeldG`; `camVzShadowLateStats`; `lateCfgBad`); `cam_vz_glue.h/.c` (the NVS read and its
checks, `camVzGlueSetMotion`, `camVzGlueLate` for USB, `camVzGlueFlightStats` with the late fields);
`flight_summary.h`, `flight_rtc_log.c`; `auto_glue_core.h/.c`; `usb_diag.c`; `autonomy.c` (one store per
controller step: `camVzGlueSetMotion(out->mv.holdWait || out->hush)` before the plant choice since 25.9, false at
`vzStart`);
`tools/espfly_diag.py`.

### 25.4 kyC: not changed

The check of the study's candidate (scratch r1-colour/kyc-check, and its skeptic) found that the per-frame constant
`kyC` (0.0294 px, 0.436 px/s at 14.8 fps) is most (~75 %) of the early false climb, and that a per-second kyC changes
nothing (the flow has processed 14.8 fps on every log, before and after the camera clock change; in colour too,
24.1). But `kyC` = 0 alone climbs +0.26 to +0.85 m in the first 40 s of the replays (today's kyC cancels the early
offset lag), and with R1 it ends +0.6 to +1.1 m high; the later camera error it also cancels is unexplained (an
offset or a scale error near the floor: the data cannot tell). So the evidence does not support a kyC change on its
own, and no kyC switch is built. The early climb of 0.1-0.45 m stays: R1 acts from 40 s (Claude's photos, 23.5).

### 25.5 Verification (host)

(The first build's numbers, with x5 and x10 and the floor counted as a pass; this build's are in 25.9.)

- `test_cam_vz_obs` (+162 checks): the replay fixture's FINAL outputs hash to the value 65dfcba's observer gives
  (pinned; `-DCVO_BASE` builds the test against an older `cam_vz_obs.c`); every row and decision of the 43 take-offs
  bit-identical with x10 from 20 s until its step (42 take-offs end before it; file 16 fid 3 reaches it 0.25 s before
  its end); a 90 s synthetic hover with the offset rising 0.2 mg/s from 25 s: bit-identical to x1 until the step (the
  first difference at 40.7 s, the lift-off at 0.6 s), the mean offset error 60-90 s 0.49 mg at x5 against 1.90 at x1,
  the controller's offset within the observer's of 2.0-4.6 s before, x1's `biasHeldG` = `biasG`; the clock: a ground
  flag for 1 s at 20 s restarts `tLift`, not the step (at first lift-off + 40 s), on the ground after the step the
  variance grows at x1.000 of the base noise, back in the air the guard, on again 3 s after the re-lift's first frame,
  a reset restarts the clock and keeps the config; every guard event at 60 s (the gate, lateral, expansion, a CUSUM
  build-up, a yaw restart, a commanded turn, a commanded motion, frames stopping for 1 s): the guard, the variance <=
  1.05 x its step size at every frame, the held offset frozen, on again 3.00-3.15 s after the last event; a single
  QUALITY refusal: no guard; a 0.1 m/s glitch for 1 s at 60 s and then the camera lost: the held offset is from before
  the glitch and closer to the truth than the one the glitch moved; the config's limits. The unit hover uses the 10-15
  hovers' accelerometer noise (5 mg per 10 ms) and 0.05 px of camera noise: the other tests' 20 mg and 0.15 px move a
  boosted offset by 1 mg in a second every few seconds, the rate guard's event.
- `test_cam_vz_late` (new, 695 checks; the section 6.4 fault tests): the real vert_ctrl HOLD on its sim plant, the
  real shadow observer fed as the glue feeds it, a synthetic camera, Claude's SET_HEIGHT before the fault, the FSM's
  SINK 5 s after the camera's last use. With x1 the run is bit for bit 65dfcba's (pinned hash). PASS: the error at
  the SINK's start (a camera lost) or 15 s after the fault at most today's + 0.10 m, or both versions on the floor.
  - 192 synthetic faults (G 25 / 56; +-0.1 / +-0.3 m/s for 0.5 / 1 / 2 s at 60 s; with and without the camera lost
    right after; two seeds; x5 and x10): **192 pass**; 44 of them end with both versions on the floor (a camera lost
    for 5-7 s over a 0.2 mg/s drift sinks today's drone 0.5-0.9 m); otherwise the worst excess +0.087 m. Without the
    rate guard 184 / 192 (the worst +0.47 m, x10 after a small negative glitch and a loss).
  - No fault (the same hovers): the true height change 60 -> 90 s x5 -0.22..-0.68 m, x10 -0.16..-0.54, today
    -0.52..-0.91; the guards keep the multiplier on 22-26 s (x5) / 13-17 s (x10) of the 50 s past the step in this
    sim (its camera noise trips the rate guard; on the real-data replays below x5 stops 1-3 times a flight).
  - The real camera of 10-13 F1 (the person and the in-air contact, 33 s) and F2 (the bright window, 9.5 s) spliced
    from 50 s into the real offsets of the 10-15 p1, p2 and tiger hovers (fixture `cam_vz_late.txt`,
    `gen_cam_vz_late_fixture.py`; every disagreement with the controller counted as camera error): **24 pass** (16
    with both on the floor: those segments end most of these hovers on the floor in both versions; otherwise the late
    noise is never worse).
  - Resting on the floor (flown down slowly from 60 s, no descent commanded on arrival): K_RANGE lands 1.2-1.7 s after
    the arrival, today and x5 and x10 alike, on the synthetic and the three real offsets. The study expected R1 to
    make K_RANGE rarer on a surface; this plant (ground effect, the touchdown jolt) does not show it, and neither
    model reproduces the real 3-8 s: printed, no pass mark (25.7).
- `test_cam_vz_glue_host` (+23): the NVS values taken (x5/40, the limits, an absent step), refused (x0, x11, 19 s,
  121 s, an unreadable item): the flag in `camVzGlueLate` and the flight summary, the multiplier 1, the camera's offset
  kept; `camBias` = `biasHeldG` + 1 - g0; a non-finite held offset unusable; the motion flag reaches the observer.
- `test_autonomy_glue_host` (+4): the motion flag set through a MOVE up and a YAW, clear in the wait and after DONE.
- `test_auto_glue_core` (+3), `test_flight_summary` (+1: 156 B, `camVzLate` at 148), `test_espfly_diag.py` (+1 and
  the `camvz` reply).
- Every other suite unchanged (autonomy-bench.md O lists the counts).

### 25.6 Replays (scratch `r1-colour/r1-impl/dsim_r1.c`: the study's dsim.c on this observer)

(The first build's; x10 is no longer allowed. 25.9 has x3 and x5 on this build: x5 identical in the table below.)

The study's closed-loop replay (each flight's real offset, accelerometer noise and every frame's real camera error)
with the implementation: `biasHeldG`, the guards, the motion flag in a SET_HEIGHT, and vert_ctrl's use of the camera
(only while healthy, the offset latched, Khat's clamps and K_RANGE). True height change from HOVER + 3 s (m):

| Flight | x | 30 s | 40 s | 60 s | lowest / highest to the real end | real end | 125 s (unattended) |
|---|---|---|---|---|---|---|---|
| p1 (end 68 s) | 1 | +0.05 | +0.01 | -0.44 | -0.77 / +0.18 | -0.78 | -2.92 |
| | 5 | +0.05 | +0.01 | -0.16 | -0.36 / +0.18 | -0.36 | -1.06 |
| | 10 | +0.05 | +0.01 | -0.14 | -0.25 / +0.18 | -0.25 | -0.96 |
| p2 (110 s) | 1 | +0.06 | +0.08 | -0.01 | -0.39 / +0.08 | -0.39 | -0.49 |
| | 5 | +0.06 | +0.08 | +0.07 | -0.00 / +0.10 | +0.09 | +0.11 |
| | 10 | +0.06 | +0.08 | +0.09 | -0.00 / +0.20 | +0.20 | +0.24 |
| tiger, study truth (101 s) | 1 | -0.17 | -0.19 | -0.45 | -1.21 / +0.00 | -1.21 | -1.64 |
| | 5 | -0.17 | -0.19 | -0.29 | -0.43 / +0.00 | -0.43 | -0.54 |
| | 10 | -0.17 | -0.19 | -0.24 | -0.37 / +0.00 | -0.37 | -0.52 |
| tiger, photo truth (102 s) | 1 | +0.11 | +0.05 | -0.35 | -1.44 / +0.11 | -1.45 | |
| | 5 | +0.11 | +0.05 | -0.19 | -0.70 / +0.11 | -0.71 | |
| | 10 | +0.11 | +0.05 | -0.14 | -0.60 / +0.11 | -0.61 | |
| fastcheck, held out (59 s) | 1 | +0.34 | -0.00 | | -0.65 / +0.44 | | |
| | 5 | +0.34 | -0.00 | | -0.35 / +0.44 | | |
| | 10 | +0.34 | -0.00 | | -0.35 / +0.44 | | |
| gym fid 1 (36 s) | 1, 5, 10 | -0.62 | | | -0.67 / +0.06 | -0.67 | |

The study's marks: p1 within +-0.15 m at 60 s: x10 -0.14 passes, x5 -0.16 just misses (today -0.44); p2 within
+-0.25 m to the end: x5 and x10 pass; tiger within +-0.25 m to the end: **fails** with both truths at x5 and x10
(-0.37..-0.71; today -1.21..-1.45; the study's own unguarded R1 x10 gave -0.25..-0.46 by 100 s). The guards cost a
little against that unguarded R1 (p1 x10 at 60 s -0.14 against -0.04); x10 stops on the guards 3-12 times a
flight, x5 1-3. The gym flight ends at 36 s, before the step: identical in all three (its "truth" is the controller's
own integral, the study's harm-test choice). The early climb (fastcheck +0.44 m) is untouched.

SET_HEIGHT +0.50 m at 60 s with the scene's G off (true / replayed), the true change over the 10 s after it (the same
run without the SET_HEIGHT subtracted):

| Flight | G x1.33 (today / x5 / x10) | G x1.00 | G x0.80 |
|---|---|---|---|
| p1 | +0.51 / +0.55 / +0.53 | +0.50 / +0.47 / +0.45 | +0.50 / +0.43 / +0.40 |
| p2 | +0.53 / +0.50 / +0.49 | +0.52 / +0.47 / +0.47 | +0.51 / +0.46 / +0.45 |
| tiger | +0.51 / +0.45 / +0.45 | +0.50 / +0.43 / +0.42 | +0.52 / +0.43 / +0.43 |

With the late noise the correction undershoots by up to 0.10 m (the frozen held offset through the move and the
guard's 3 s after it; today's lagging offset happens to fly it fully here). Within the study's advice to confirm any
SET_HEIGHT >= 0.3 m with a new photo.

Resting windows (the real frames and offsets of the drone sitting on a surface, a floor under the replayed drone from
the arrival): K_RANGE fires in none of the versions, today included, so this replay's controller model cannot answer
the question (the closed-loop test above has the opposite limit). Open: autonomy-bench.md O4 and the first flights.

### 25.7 Cost (this build against 65dfcba, `idf.py size` / `size-components`)

| | 65dfcba | late noise | |
|---|---|---|---|
| static IRAM | 130618 B | 130650 B | +32 B (`camVzObsImu` +28, `camVzObsGet` +20, `vzRun` +12, `camVzTask` +11, `camVzShadowStep` -28, `camVzShadowRecord` -28); `_iram_text_end` 0x40393f00 unchanged: the same 256 B step, 165 B left |
| internal .bss | 88272 B | 88296 B | +24 B (`_heap_start` 0x3fcc1308 -> 0x3fcc1320: the snapshot's late fields, the motion flag): the internal heap 24 B smaller |
| RTC no-init | | +128 B | 16 summaries x 8 B |
| flash | 954631 B | 957395 B | +2.8 KB (the frame-path code, the NVS read, USB) |
| PSRAM | | +120 B | the observer's state (2032 -> 2152 B on the host) |

CPU: not literally untouched (the budget review): one byte store per controller step in `vzRun` (the motion flag,
next to the guess flag's), a 4 B larger observer snapshot copied per step in `camVzGlueVzInput`, 8 B more copied at
100 Hz into the RTC summary (`flightRtcLogSample`, its stack frame +16 B); the stabilizer's own code is unchanged;
the camvz task's block path tests one flag; the frame path (flash, ~15 Hz) adds ~40 flops and two passes over a
10-entry history only with the multiplier > 1. The loop-rate gate (>= 940) is the bench's (O3).

### 25.8 Risks and what is open

1. **Tiger fails the +-0.25 m mark** with the late noise (-0.37 to -0.71 m by its end): R1 halves today's sink but
   does not remove it on a far, slow-drifting scene. Claude's photo rhythm (23.5) still decides the height.
2. **K_RANGE on a surface** is not answered: one model always fires it, the other never. The user keeps K_RANGE; a
   drone that rests on a table with the late noise on may sit at near-hover thrust until Claude lands it (study 4.3
   rule 6: LAND at once on a floor or table-top view).
3. **The guards trade benefit for safety.** In the closed-loop sim x10 runs only 13-17 s of 50 (x5 22-26 s); on the
   real-data replays x5 runs 56-81 s of the long flights and stops 1-3 times, x10 stops 3-12 times. x5 is the tested
   strength.
4. **A lost camera's open loop** (5 s to the SINK, 2 s of health window before it) sinks 0.5-0.9 m on a 0.2 mg/s drift
   in both versions; 44 of the 192 synthetic faults end on the floor in both. The late noise does not make it worse
   (25.5) but this is no fix for it.
5. **SET_HEIGHT undershoots** by up to 0.10 m with the late noise (25.6).
6. **The camera stopping entirely** lets the boosted variance grow until the next frame, which bounds it before its
   update; `camvz` `sigBiasMg` shows the growth meanwhile.
7. **The flight summary layout** changed (156 B, a new magic): the summaries of the previous build are cleared once.
   Save `python3 tools/espfly_diag.py flights` before flashing. RTC slow memory: 7404 of 8192 B used, 788 B left for
   later summary growth (RTC slow, not heap).
8. **Slow camera errors** (25.9): a camera that reads a false sink of 2-6 cm/s for 5-20 s, under every per-frame
   gate, is followed further by the faster offset than by today's: the drone climbs more, and after a loss keeps
   climbing on the offset it moved. Intrinsic to a faster offset (no guard tells it from the accelerometer's drift);
   it is why x10 was cut. At x5, 7 % of the slow-error cases climb more than today + 0.10 m, the worst +0.39 m (a
   6 cm/s false sink for 20 s, then the camera lost). The same size as the unexplained late camera error of the kyC
   check (0.24-0.67 px/s, 1-3 cm/s).
9. **Today's CUSUM alarm partly relies on the lag R1 removes**: a 1 s glitch the lagging offset made an alarm of may
   be accepted by the faster one, and the other way round (25.9: a cold start, -0.3 m/s then lost: today alarms, x5
   does not). Re-check the alarm rate on 1 s glitches of both signs when re-tuning.
10. **The 1 kHz path's rule**: the motion flag's store in `vzRun` is an exception the user should know of (one byte a
   step, measured +12 B of IRAM, +16 B with the hush; A4 and `stabStackFree` >= 1024 are re-checked in O3).

### 25.9 The review fixes (2026-10-15, late; still off by default)

Four reviews of the R1 build (code, safety, camera, budgets) agreed that the default `cvqlate` 1 is today's observer
bit for bit and found two major problems with it on. Both are fixed or bounded here; nothing changes with it off (the
pinned hashes below are the same). Scratch harnesses: `r1-colour/fixer/` (the safety review's `xfault.c` on this
observer, `dsim_r1.c` replays).

1. **The held offset no longer stays frozen indefinitely** (`CAM_VZ_QLATE_FREEZE_US` 4.5 s). The first build froze
   `biasHeldG` for as long as guard events kept coming, and the controller flies on it whenever the camera is in use,
   not only after a loss: through a long or repeated MOVE / YAW / SET_HEIGHT, or a drift that keeps the rate guard
   tripping, it flew on an ever older offset while the real one kept rising 0.12-0.25 mg/s (the reviews: a motion
   held 45-90 s sank x5 0.27-0.35 m more than today from 60 to 90 s at G 56 and put it on the floor at G 25; the
   controller's offset 8-9 mg off at 76 s against today's 1.5-2). Now a guard run freezes it for its first 4.5 s (the
   history's span: a glitch the boosted offset took before the guard tripped is still kept out of it), then the
   delayed mean resumes, of an offset that moves at `qBiasG` alone while the guard holds. Tried and not taken:
   `biasG` itself after the cap (no delay at all) made a glitch followed by a loss 0.10-0.11 m worse than today in 3
   of 192 synthetic cases.
2. **x10 is gone: `cvqlate` 1..5** (`CAM_VZ_QLATE_MULT_MAX` 5; a stored 6..10 is refused at boot like any bad value,
   `qlateCfg=bad`, x1 runs). The faster offset follows a slow camera error sooner than today's (25.8 item 8): x10
   climbed up to 0.62 m more than today, x5 0.39 m, x3 0.36 m. Tried and not taken: a slower detector, the offset
   moving more than 1.25 / 1.5 / 2.0 mg in 4.5-5 s as a guard event (the accelerometer drifts <= 0.25 mg/s, so 1.25 mg
   at most in 5 s). At 1.5 mg it cut the x5 slow-error failures from 20 to 13 of 288, but it fires on the real drift
   too: the p1 replay at 60 s went from -0.16 to -0.22 m (x5), its end from -0.36 to -0.46 m, fastcheck from -0.35
   to -0.53 m; at 2.0 mg nothing changed, at 1.25 mg the flat-offset cases got worse. No setting tells a slow camera
   error from the drift R1 is for; the risk is pinned by a test (section 6 of `test_cam_vz_late`) and stated in 25.8.
3. **The guard's variance bound regrows like today's.** It held P[2][2] at its size at the step even after a gap or
   a long guard, stiffer than today's filter (the docs said "keeps qBiasG"). Now the bound is that size plus what
   `qBiasG` alone adds since the previous frame: only the multiplier's share is taken back (a 3 s gap: 1.105e-4 =
   8.39e-5 + the base growth, where the multiplier had grown it to 1.19e-3).
4. **`lateUs` after a re-lift.** Frames are skipped on the ground, so the flag stayed as at the touchdown and the
   first frame after a re-lift added up to 0.5 s of ground time; now the ground skip clears it (and counts that as a
   guard stop, as the ground is a guard). With the flag off the blocks between the re-lift and its first frame run at
   `qBiasG` (they did at x`qLateMult` before).
5. **A contact's hush is a guard event, and a DRY sortie sets the flag.** autonomy.c sets
   `camVzGlueSetMotion(out->mv.holdWait || out->hush)` once per controller step before the plant choice (it sat in
   the real-input branch: a DRY sortie never set it, so bench O4 could not exercise it). The cost on the stabilizer
   task's autonomy step: the same one byte store as before plus an OR (`vzRun` 463 -> 467 B of IRAM; 451 B at
   65dfcba). The hard rule says the 1 kHz path gets no new work: this store is R1's one exception, for the user to
   know (25.8 item 10); moving it to the hold-reference snapshot (also built every loop) would only move it.

Verification (host, this build; `run_ctests.py`, each test's own header command):

- `test_cam_vz_obs` 3043 -> 3053 checks: the FINAL replay hash `e0ff64f3b4b2983d` unchanged (65dfcba's observer);
  every row before the step bit-identical at x5 from 20 s (x10 before; the header now says plainly that the replay
  covers only the path before the step: 42 of the 43 take-offs end before 20 s); a motion held 60-80 s: the guard
  throughout, the held offset frozen 4.54 s, then within the observer's offsets of 2.0-4.6 s before; a 3 s gap: the
  variance bound regrown at the base rate (the first frame back a QUALITY refusal, so P[2][2] after it is the bound);
  the ground's time not counted after a re-lift (at most 0.15 s from 59.95 s: the frames before the touchdown; the
  first build added up to 0.5 s more); the limits x5 valid,
  x5.01, x6 and x10 refused; the state 2160 B (+8: the freeze run's flag and start).
- `test_cam_vz_late` 695 -> 1385 checks, x3 and x5 (x5 and x10 before). The pass mark no longer counts "both on the
  floor" as a pass: there R1 must reach the floor no more than 2 s before today (at the 4-5 cm/s these runs sink at
  near the floor, the study's 0.10 m), and the floor cases are tallied apart.
  - Synthetic faults: 192 of 192 (148 not on the floor, the worst excess +0.055 m; 44 on the floor in both, R1 at
    most 1.8 s sooner: x3 after a +0.3 m/s 1 s glitch, G 25, which today took without an alarm and x3 with one, so
    the camera was out for 3 s with the glitch in the speed; the first build reached the floor 3.2 s sooner there).
  - Splices: 24 of 24 (8 not on the floor, worst +0.011 m; 16 on the floor in both, R1 never sooner).
  - New section 5, Claude's motions past the step (a MOVE held 45-90 s, and 6 s on / 4 s off; G 25 and 56, two
    seeds, x3 and x5): R1 within today + 0.05 m (G 56: -0.55..-0.62 m against today's -0.52..-0.60 from 60 to 90 s),
    on the floor at G 25 no more than 2 s sooner (0.2-1.4 s). The first build fails 11 of these 16 checks (G 56
    -0.87 m against -0.52..-0.60).
  - New section 6, slow camera errors (+-2 / 4 / 6 cm/s for 5 / 10 / 20 s from 60 s, a step or a ramp, with and
    without the camera lost, the drifting and a flat offset, G 25 and 56: 216 cases a strength): x3 212 of 216 within
    the mark (the worst +0.363 m), x5 203 of 216 (the worst +0.393 m). Pinned: at most 10 % over the mark and none
    over 0.45 m (25.8 item 8). The known residual risk, not a proof.
  - The no-fault hovers (60 -> 90 s): x3 -0.28..-0.70 m, x5 -0.21..-0.68 m, today -0.52..-0.91 m. Resting: K_RANGE
    1.3-1.8 s after the arrival in x1, x3 and x5 alike. The x1 run's hash `869d48cae62cef73` unchanged.
- `test_cam_vz_glue_host` 52 -> 54 (x3 and x5 taken, x6 and x10 refused), `test_auto_glue_core` 195 -> 196
  (`cvqlate` 1..5, x6 not a valid observer config, `photocol` default 0), `test_autonomy_glue_host` 443 -> 446 (a
  contact's hush in a DRY sortie raises the flag, and it clears after). Python: `test_espfly_diag` 79 (the fake `auto
  get` with `photocol=0`), `test_espfly_auto` 203, `test_hold_band` 13, all OK.
- The safety review's extended families (scratch `xfault`, this observer, x3 and x5 from 40 and 20 s; the floor
  still counted as a pass there): slow errors on the drifting offset x5 20 of 288 over (the worst +0.40 m), x3 9;
  on a flat offset x5 7 of 144 (+0.15 m), x3 2; plain losses x5 4 of 64, all a loss at 85 s where today's drone had
  already sunk to the floor's 0.29 m and R1's at 0.74 m fell further with a smaller offset error (-1.0 against -2.3
  mg at the loss); offset shapes x5 3 of 80 (16 in the first build: the cold start, a -0.3 m/s glitch then lost,
  which today's CUSUM alarms on and x5 accepts, 25.8 item 9; and a loss at 75 s on the same shape); SET_HEIGHT across
  the step and long motion 0 of 96 (the worst +0.05 m); the real person / window segments late 0 of 144. A long
  motion with the camera fine (10-30 s from 45 s): x5 at most +0.087 m worse than today, on average 0.021 m better
  (the first build: +0.217 m, 0.026 m worse).
- Replays (`dsim_r1`, this observer): x5 gives the 25.6 table's x5 rows to the cm (p1 60 s -0.16, its end -0.36;
  p2 -0.00..+0.10; tiger -0.43; tiger photo truth -0.71; fastcheck -0.35); x3: p1 -0.24 / -0.47, p2
  -0.04..+0.08, tiger -0.61, tiger photo truth -0.84, fastcheck -0.45. SET_HEIGHT +0.50 m: x5 0.43-0.56 m, x3
  0.44-0.55 m (today 0.50-0.53). The study's toy fault scripts (`verify-safety/fault*.py`) model the study's R1, not
  this code: re-run, their output is unchanged.

Cost against af7fe4d (`idf.py size` / `size-components`, the build in `r1-colour/build-fixer`): static IRAM 130650 ->
130646 B (-4: `vzRun` +4, `camVzShadowStep` -7, alignment), `_iram_text_end` 0x40393f00 unchanged (149 -> 153 B
before the next 256 B step), .bss 88296 and .data 31544 unchanged, `_heap_start` 0x3fcc1320 unchanged, flash 957395 ->
957555 B (+160), RTC no-init unchanged (7404 B). Against 7a78e1b: IRAM +28 B, .bss +32 B (the internal heap 32 B
smaller), flash +3644 B.

What a flight with `cvqlate` > 1 still needs: the user's OK, bench O0-O5 (O3's loop rate and `stabStackFree`), and
the risks of 25.8 (items 1, 2, 5, 8 and 9 above all).

## 26. The USB console's stack: `status` overflowed it (2026-10-16)

On the bench with 82bcefb, `status` crashed the drone: `crash` `exception=4 pc=0x40376d5d ... details=***ERROR*** A
stack overflow in task USBDIAG has been detected.` (the backtrace: `_frxt_dispatch` -> `vTaskSwitchContext` ->
`vApplicationStackOverflowHook` -> `panic_abort`, FreeRTOS's canary check at a context switch). With `photocol` 1
every `status` crashed; after `photocol 0` and a reboot too, twice in a row, 6-30 s after boot; in the grey boot right
after the flash it worked for 524 s (and failed twice in the first seconds). Every other console command worked.
7a78e1b's `status` never crashed. Nothing here was run on the drone: the numbers are from this tree's builds of
82bcefb and 7a78e1b (byte-identical code addresses to the flashed image: the crash's backtrace decodes to the lines
above), each function's `entry` frame from `objdump` and a call graph that follows the literal-pool `callx8` calls
(scratch `usbdiag-fix/fixer1/cg.py`); `-fstack-usage` gives the same frames for usb_diag.c.

### 26.1 What the stack holds

- The task's 5120 B (`USBDIAG_TASK_STACKSIZE`, static): the coprocessor save area (316 B) and the first context
  frame at the top, the canary (20 B of 0xa5, `CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY`, checked at every context
  switch of the task) at the bottom: **4768 B** from the task's first stack pointer to the canary. Below the array
  sits the task's own TCB (`osSys_usbDiagTaskTaskBuffer`).
- `out()` formats into the PSRAM line with `vsnprintf`, but newlib's full `_svfprintf_r` is deep: `vsnprintf` 64 +
  `_vsnprintf_r` 144 + `_svfprintf_r` 784 + `_dtoa_r` 144 + `__pow5mult` 32 + `__multiply` 64 + `_Balloc` 32 =
  1264 B for a `%f` that needs big integers (`_dtoa_r`'s quick path stops 128 B higher; a NaN never reaches
  `_dtoa_r`), and **+224 B** (`malloc` -> `heap_caps_malloc_base` -> `multi_heap_malloc` -> the spinlock) when the
  task's big-integer free list has no block of that size yet: newlib keeps the blocks per task, so the first such
  `%f` after a boot is the deepest.
- The windowed ABI: arguments past the sixth go in the caller's frame, so `status`'s one `out()` with 97 arguments (26
  doubles) carried a ~0.5 KB argument block in `cmdStatus`'s 1696 B frame. An interrupt taken on this core pushes
  its 192 B frame (`XT_STK_FRMSZ`) on the interrupted task's stack before it switches to the ISR stack, plus the
  window spill's 16 B.
- GCC inlines a static function called once: every command handler but a few was inlined into `usbDiagTask`, whose
  frame then holds all their locals and argument blocks, live under every command.

| build | `usbDiagTask` frame | `cmdStatus` frame | `status` peak, free list / malloc | left before the canary |
|---|---|---|---|---|
| 7a78e1b | 1456 B | 1696 B | 4496 / 4720 B | 272 / 48 B |
| 82bcefb | 1552 B | 1696 B | 4592 / 4816 B | 176 / -48 B |
| this build | 144 B | 416 B | 1920 / 2144 B | 2848 / 2624 B |

**Why 82bcefb and not 7a78e1b:** not `camPix=`: the one more argument fits `cmdStatus`'s frame as before (1696 B in
both). 401fbb9's `camvz` fields (the late-offset struct and nine arguments) and the `flights` line's five, both inlined
into `usbDiagTask`, grew the dispatcher's frame by 96 B, under every command. 7a78e1b's 272 B held an interrupt at
the deepest point (208 B); 82bcefb's 176 B does not, and its first big-integer `%f` after a boot overflows without one.

**Why it depended on the boot** (an explanation from the numbers, not measured on the drone): (1) the values: which
`%f` take `_dtoa_r`'s big-integer path (and of which size) depends on the numbers printed (battery, drift, trims,
temperature; `roll` / `pitch` / `yaw` print NaN without a flight record and stop higher); (2) the first such blocks
after a boot come from `malloc`, 224 B deeper, so the first `status` of a boot is the worst and a boot that survived
it has warm blocks; (3) an interrupt landing in the few microseconds at the deepest point (the 1 kHz tick, Wi-Fi with
the Mac linked, the camera's DMA, I2C, USB) adds 208 B. The canary is checked at the task's next context switch, which
always comes: `usbWrite` waits (`vTaskDelay(1)`) while the 64 B USB FIFO drains the ~1.1 KB line.

### 26.2 The fix (no internal RAM)

1. Every command handler is out of line (`OUT_OF_LINE`, noinline), and the stream's record moved to `streamTick()`:
   `usbDiagTask`'s frame 1552 -> 144 B, the base under every command.
2. `status` is built in eleven `outCat()` pieces (at most 16 arguments) into the same PSRAM line and sent with one
   write (`outEnd`): the same single line, the same fields in the same order; the getters' structs in block scopes.
   `cmdStatus`'s frame 1696 -> 416 B. `out()` is `outCatV()` + `outEnd()`, its output unchanged.
3. `diagStackFree=` appended last to `status`: `uxTaskGetStackHighWaterMark` of the console task, bytes, the least
   it has had free since boot (any command), read after the line's other pieces were formatted.
   `tools/espfly_diag.py status` prints a warning under 1024 B (`DIAG_STACK_MIN`).

Every command's peak in this build (the malloc path; an interrupt adds 208 B): `flights` 2784 B, `camvz` 2672,
`auto ...` 2656 (`auto status` 2608), `net` 2400, `gtc` 2384, `rec` 2304, `measure` 2288, `status` 2144; the least
margin is `flights`' 1984 B before an interrupt (82bcefb: `status` -48, `auto status` 336, `gtc` 992). No other
`out()` needed splitting: with the dispatcher's frame gone each big one (`camvz` 72 arguments, `flights` 90, `auto
status` 50) is within its own frame and leaves >= 1.7 KB. `diagStackFree` should read well above 1024 after `status`,
`flights`, `camvz` and `auto status` (the predicted ~2.0 KB is a bound: frames are not written to their bottom).

Cost against 82bcefb (`idf.py size`): static IRAM 130646 B, .data 31544 B, .bss 88296 B, `_heap_start` 0x3fcc1320
and `_iram_text_end` 0x40393f00 all unchanged; flash 957555 -> 951571 B (-5984: .text -6024, mostly the long
stack-offset sequences of the >1 KB frames; .rodata +40). The task's stack stays 5120 B.

Verification (host): `run_ctests.py` 37 suites, all pass but `test_gyro_tempco` (33) and `test_mahony_replay` (4), the
same failures as before (25.9); Python `test_espfly_diag` 79 -> 81 (the warning; the `status` format keeps its last
fields and ends with `diagStackFree=%u`), `test_espfly_auto` 203, `test_hold_band` 13, all OK. On the bench: bench
section P.

## 27. Set-height to the target, climbs on thrust headroom, the hold at its limit, the trim check (2026-10-16)

After the 10-16 flights (F1, F2, the A/B flight on 7a78e1b and the tiger mission; the left-drift study,
firmware-backups/handoff-2026-10-12/leftdrift-2026-10-16.md) the user asked: drift is fine now; the late sink is the
problem ("you could have used more thrust since you seemed to still have battery"). This section fixes what the study
found in the height commands and adds two reports. It does not touch the sink itself (the estimator's late offset:
the colour/sink plan's T2, colour-sink-plan-2026-10-16.md). On the stabilizer task it adds one byte load and compare
at the slow-input rate (every 10th loop) and, only when the camera's hold-limit bits change (a few times a sortie), one
call into flash code (27.4, 27.9); nothing lands on its own. The review round's changes are in 27.9.

### 27.1 The set-height flaw

Section 23's SET_HEIGHT shifted the estimate by d = X - h_at and flew -d: the drone ended where its old estimate was,
"truly where it believed it was". But HOLD's estimate is not the commanded height: the climb enters HOLD at
hTarget - holdBand and brakes, so a 1.00 m take-off holds an estimate of 0.88-0.93 m (every 10-15 / 10-16 log). Each
correction therefore ended ~0.1 m under the commanded height: the logged ends (events.jsonl `set_height_end`) were
0.87-0.96 m for a commanded 1.00 m. On 10-16 Claude worked around it by hand (X = est_at_photo + judged - wanted, "the
est-rule").

### 27.2 "You are at X, go to the target" (`auto_fsm.c setHeightCmd`, `auto_motion.c autoMotionSetHeightRef`)

**The command:** SET_HEIGHT gains a 6-byte form (`AUTO_SET_HEIGHT_TO_LEN`): i16 xCm, i16 atCm, then i16 toCm, the
height to fly to (`AUTO_SH_TO_REF` = INT16_MIN: the commanded height, take-off height + the committed moves; else
25..200 cm, the MOVE range). The link passes it as `cmd.mode = AUTO_SH_FORM_TO`, the target in `moveRightCm`
(autoCmd_t unchanged in size). The 4-byte form is the build before it but for the climb's refusal: UP_HEADROOM (27.3)
applies to both forms against the ceiling in force, which at the 58000 ceiling refuses exactly what 8277b68's BATT
(T_hold > 53400) refused (only the reported reason differs); under a lower ceiling (never seen in the logs) it refuses
more. A drone before this build answers the 6-byte form RANGE detail 0 (the link's length check: not processed).

**On acceptance** (the same step, the same checks as 23.2 plus two): the estimate shifts by d as before (h_new =
h_now + d = the true height now); the flight is upM = target - h_new; the committed height offset is set so the
reference starts at the corrected estimate (hOff = h_new - hTarget: refShift = h_new - hTarget - hOff_old) and DONE
commits hOff = target - hTarget (commitUp's old + refShift + upM: unchanged code). The drone ends truly at the target
and its estimate there too; an explicit target becomes the commanded height (`hTargetCm` in AUTO_STATUS) for the moves
and the LAND timings after it. STOP, a contact or a LAND commit the measured travel from the corrected estimate: the
reference where the drone is. d may round to 0 cm in this form (the estimate was right: the flight alone), not both
(RANGE).

| # | Check (added or changed) | Refusal |
|---|---|---|
| 7 | the target form: an explicit target outside 25..200 cm, or d and the flight both 0 cm | RANGE |
| 8 | \|d\| <= 50 cm and \|upM\| <= 0.55 m, as before (the flight now target - h_new: a 0.45 m estimate error plus the 0.10 m HOLD gap fits) | SH_STEP |
| 9b | a flight up (upM > 0, both forms) without the thrust a climb needs (27.3) | UP_HEADROOM 16 (LOW_BATT) |
| 10 | battLow, as before (a descent too) | BATT (LOW_BATT) |

**The Mac** (`set-height X [--photo P | --at M | --now] [--to H] [--no-wait]`): always the 6-byte form; the target the
status's `hTarget_m` or --to. A flight over 0.55 m is sent as `--to` 0.50 m from the true height toward the target
(`clipped_to_m`, `rest_m`: a new photo, then again: the daemon remembers the target, 27.9). The reply and the `set_height` event: `true_now_m`,
`target_m`, `target_from` (commanded | --to | remembered | legacy), `fly_m`, `offset_m`, `old_m`, `sh_form` (target |
legacy-target | legacy), and
`t_hold`, `flies_to_m` (the true end), `new_m` (the drone's estimate right after its shift: the true height now in the
target form, h_now + X' - h_at in the legacy one, 27.9); `warnings` when the photo is older than 10 s (`photo_age_s`: the estimate's drift since the photo is not in
the correction) or T_hold is over 53400 for a climb. A drone without the target form (the RANGE detail 0 answer) gets
the 4-byte form at once with X' = X + h_now - target (`legacy_x_m`, event `set_height_form`): it flies back to its old
estimate h_now and so ends at h_now + X - X' = the target; its estimate then reads h_now - target off (~-0.1 m, as
after a take-off). `set_height_plan()` and `set_height_legacy_x()` are the drone's arithmetic, tested on the replays.

**Replays** (Claude's judgements in RESUME.md, the estimates and times from events.jsonl; intended 1.00 m):

| correction | X judged, h_at, h_now | sent then | ended then (true, model) | target form: fly | model end | closed loop: DONE / 2 s later |
|---|---|---|---|---|---|---|
| 10-15 p2 18:18 | 1.05, 0.91, 0.90 | 1.05 | 0.90 | -0.04 | 1.000 | -0.016 / -0.033 |
| 10-15 p1 19:44 | 1.08, 0.90, 0.89 | 1.08 | 0.89 | -0.07 | 1.000 | -0.017 / -0.022 |
| 10-15 p1 19:45 | 0.83, 0.89, 1.03 | refused NO_CAM_HEIGHT | - | refused NO_CAM_HEIGHT | - | - |
| 10-15 fastcheck 20:05 | 1.28, 0.93, 0.93 | 1.28 | 0.93 | -0.28 | 1.000 | -0.017 / +0.004 |
| 10-15 fastcheck 20:06 | 0.82, 0.92, 0.93 | 0.82 | 0.93 | +0.17 | 1.000 | +0.048 / +0.013 |
| 10-16 F1 09:45 | 1.12, 0.90, 0.90 | 1.12 | 0.90 | -0.12 | 1.000 | -0.011 / +0.005 |
| 10-16 F2 09:52:16 | 1.15, 0.89, 0.89 | 1.04 (est-rule) | 1.00 | -0.15 | 1.000 | -0.042 / -0.029 |
| 10-16 F2 09:52:43 | 0.60, 0.89, 0.90 | 0.49 (est-rule) | 1.01 | +0.39 | 1.000 | -0.028 / -0.059 |
| 10-16 tiger 11:09 | 0.70, 0.85, 1.06 | 0.55 (est-rule) | 1.21 | +0.09 | 1.000 | -0.021 / -0.050 |

"The model" (test_auto_fsm.c `testSetHeightReplay`: the firmware's state machine and motion code, the photo's error
carried to the command, the plant tracking the vertical reference exactly): every correction ends within 0.000 m of
1.00 (the gate: 0.03 m); the 4-byte form as sent ended 0.07-0.11 m low where Claude sent the judgement and 0.00-0.21 m
off with the est-rule. The closed loop (test_interactive_sim.c `testSetHeightReplaySim`: the real vert_ctrl on its sim
plant and camera model, the logged geometry set white-box, a perfect judgement sent 0.5 s after the photo): DONE within
-0.042..+0.048 m, inside a vertical MOVE's settle tolerance (0.04 m + 10 % of the flight; checked), and 2 s later within
0.059 m (the sim's own drift; gate 0.07). So the arithmetic is exact and the dynamics add the settle tolerance every
vertical MOVE has; the logged 4-byte corrections ended 0.02-0.03 m past their own targets the same way. With the logged
photo-to-command times (10-29 s) the sim drifts -0.12..+0.06 m before the command: the delay, not the command, is the
larger error. The tiger correction shows the offset form's assumption: the estimate rose 0.21 m in the 29 s after the
photo while the drone sank (thrust 51k -> 41k: ground effect over the playpen); the target form trusts that rise as
motion and flies only +0.09 m. Hence the Mac's warning on photos older than 10 s and the runbook's "act within ~10 s".

### 27.3 Climbs on thrust headroom, never on the voltage (on the drone)

(On the drone. The Mac keeps its battery-duration MOVE rule below, and since the review round warns on a climb late in
the battery: 27.9.)

The study proposed refusing upward corrections below ~3.25 V loaded; the user's tiger flight climbed +0.3 m fine at
3.12-3.22 V loaded. The firmware never had a voltage rule for climbs: since 2026-10-14 every motion is refused once
T_hold > `AUTO_MV_REFUSE_T` (53400 = 58000 - 4600) or the BATT_LOW condition (3.00 V or T_hold > 57000) has held
0.25 s. The new rule names the climb: a MOVE up or a SET_HEIGHT whose flight goes up is refused `UP_HEADROOM` (reply
LOW_BATT) when the ceiling in force (`vzOutput_t.ceiling`: 58000, the battery bound capLo x K_lo, the model's capKhat x
Khat) is less than min(4600, 10 % of T_hold) above T_hold. Checked when the command arrives (flash, `upHeadroomOk`),
before BATT, after BUSY and RANGE.

Why 4600: the logged 0.15 m/s climbs needed at most +2.2k over T_hold (F2's +0.42 m at T_hold 51.7k peaked 53.9k;
fastcheck's +0.13 m +1.5k; descents' braking up to +3.0k), the hold loop's thrust noise sd 170-334 (3 sd ~1k): T_hold +
3.2k must stay under the 57000 BATT_LOW trigger, and 4600 keeps it. At the 58000 ceiling the rule is exactly the old
53400 level (a static assert), so nothing 8277b68 accepted is refused there; it adds a refusal only if the ceiling in
force is lower (never seen in the logs: F2's HOLD_CAPPED ran at 58000), and it reports the thrust instead of "BATT".
The 10 % floor keeps the soft model ceiling (1.15 x the hover: a slower climb, never a LAND) from refusing a climb off
a support (the tiger's T_hold 41k in ground effect, a hover at 30k on a cushion).

What it does on the two cases: **the tiger's +0.3 m** (T_hold 41.0k at the command, 3.30 V; peak 52.2k, then 53.3k):
allowed (headroom 17k). **F2's +0.42 m** (T_hold 51.7k at 3.17 V; the climb itself peaked 53.9k and ended DONE at
64.6 s of the log): allowed (headroom 6.3k), and the thrust evidence agrees: the HOLD_CAPPED came 2.3 s after the climb
had ended, in a -0.3 m/s sink at 3.05-3.10 V with the thrust pinned at 58000 (the stale-photo mistake's descent toward
the island, then the user's catch), not from the climb's demand. The Mac's own battery-duration rule (a MOVE refused
under 3.25 V loaded when its profile + settle would outlast the fall to 3.00 V at 5 mV/s; `--batt-ok`) stays: it is
about BATT_LOW's 3.00 V landing, not the climb; at 3.12 V it allows 24 s, any vertical move fits. set-height has no
voltage rule at all.

### 27.4 The hold at its limit: HOLD_LIMIT (information)

**Rule** (`camera_hold.c camHoldLimitStep`, camera task, flash): per axis, the hold's output at >= 1.3 deg (of the 1.5
deg its integral may lean, `CAM_HOLD_I_LIMIT_DEG`) for 8 s, in autonomy HOVER outside any motion (the recorder's
mvState RUN / PAUSED / SETTLE of any kind: a MOVE's lean, a turn's new view, a correction), on frames the output is
applied (engaged, or coasting on the learned integral: the lean stays applied). A run opens at 1.3 deg and, since the review round
(27.9), stays open down to 1.2 deg (`CAM_HOLD_LIMIT_KEEP_DEG`); it breaks on one frame under that or out of those
conditions. The camera task publishes the bits (`cameraFlowHoldLimitBits`, one byte; the axes and
signs, `CAM_HOLD_LIM_*` = `AUTO_HL_*`); the glue reads it with its slow inputs (every 10th loop, a byte load and a
compare) and only on a change calls `autoFsmHoldLimit()` (flash), which reports a rising axis as the event
`HOLD_LIMIT` (15, detail the rising axes and every sign) in HOVER in the wait with no motion running, FLIGHT only.
Nothing else: the drone holds and reports. It is pushed without recordEvent()'s plan cancel (it comes in the wait
only) and counted in `infoEvents`, which `autoFsmTrimEvents()` (the trim learner's `autonomySortieEvents`) leaves out,
so the learner runs exactly as before. The Mac: `drone_event` HOLD_LIMIT with `detailName` [{axis, sign, creeps}]
(+roll holds against a drift left, +pitch against one forward) and the hint (photo, watch the drift, land soon,
`trimcheck`, check the legs, props and battery seat; do not raise the hold's limit).

**The logs** (tools/tests/test_hold_limit_replay.c, every flight-2026-10-1[3-6] CSV, the frames' own clock):

| flights | longest run at >= 1.3 deg (roll / pitch) | HOLD_LIMIT |
|---|---|---|
| every 10-15 flight (fastcheck, p1, tiger, p2-height; 11 flights) | <= 0.9 / <= 1.7 s | never |
| the other 21 10-14 flights (afternoon, gym 2-6, morning, p2, p2b) | <= 0.9 / <= 2.5 s | never |
| 10-14 gym flight 1 (boot 12) | 12.8 / 1.1 s (roll 1.5-2.1 deg) | roll at 13.6 s |
| 10-16 F1 | 8.1 / 11.8 s | roll at 48.2 s, pitch at 56.2 s |
| 10-16 F2 | 7.6 / 1.1 s | never (just under 8 s) |
| 10-16 A/B on 7a78e1b, tiger | <= 0.3 / <= 2.0 s | never |

The requirement said "never on a 10-14 log". The gym flight 1 cannot be told from F1 by any threshold near 1.3 deg / 8
s (its run is longer and higher than F1's roll run; F1's pitch run is 11.8 s): it is the same condition, the hover that
"needed +1.58 deg with the same trim" (the left-drift study), so the warning fires there and the test pins it. F1's
roll run was 8.1 s at >= 1.3 deg (0.1 s to spare); with the 1.2 deg keep (27.9) it is 8.6 s, its pitch 13.8 s, and the
same flights fire at the same times.

**Cost:** camera_flow.c 12 B of state (`camHoldLimit_t`) + 1 B published, autonomy.c 1 B, autoFsm_t +3 B (appended);
IRAM: the glue's slow-input compare (27.7).

### 27.5 The post-flight trim check (`tools/espfly_diag.py`)

`trimcheck CSV [--flight N] [--trim ROLL PITCH | --flights TXT] [--ay-normal MG] [--json]` (no drone): per autonomy
flight, the first 10 s of HOVER outside motions with the hold applied: the mean hold correction per axis (flagged
above 0.8 deg) and the mean sideways accelerometer reading (flagged more than 15 mg from the drone's normal, -12 mg: the
median of the first 10 s of the good 10-14 / 10-15 hovers, -7.4..-16.5 mg; 1 deg of steady lean reads ~17.45 mg). A
flagged axis gets the suggested atrim with its arithmetic: the trim takes the hold's mean (the learner's own rule,
att_trim_learn.h: the trim is added to the setpoint as the hold is), the full value trim + mean and a first step trim +
0.5 x mean, rounded to 0.05 (the need moved ~1.5 deg between days and spots); the trim from --trim, or from a saved
`flights` output (its `trim p/r deg` column). It prints `atrim set R P` as a RAM-only try and never sends anything.
`analyze` and `dump` print the same check per autonomy flight (the trim unknown there), and `flights` adds a line for a
flight whose whole-flight hold mean is over 0.8 deg, with the arithmetic against that flight's trim.

On the logs: F1 roll +1.11, pitch +1.12, ay +9.6 mg (+21.6): all three flagged, atrim -0.60 + 1.11 = +0.51 (full),
first step -0.05; F2 roll +0.94, ay +7.0 (+19.0): flagged, first step -0.15; the A/B flight, 10-15 p1 and fastcheck:
nothing to change; the 10-14 gym flight 1: roll +1.85, ay +17.5 (+29.5).

### 27.6 Verification (host)

- `test_auto_fsm` 1885 -> 1962 checks: the target form (the flaw against the old form, --to, d 0, the ranges, SH_STEP
  with the flight, a LAND during it, UP_HEADROOM at a lower ceiling and at 3.12 V, the relative floor), the replays,
  HOLD_LIMIT (reported, repeated after a fall, not during a motion or in LAND or DRY, left out of the trim events,
  cleared at ARM); the fake controller now reports a ceiling (58000) in HOLD; two existing checks name UP_HEADROOM
  where a climb at T_hold 53401 said BATT.
- `test_interactive_sim` + the closed-loop replays (8 checks); `test_camera_hold` 148 -> 156 (camHoldLimitStep,
  camHoldMotionActive); `test_hold_limit_replay` (new, 4 checks over the 60 logged flights of 10-13..10-16); `test_auto_proto` 374 -> 394
  and the goldens `set_height_to`, `set_height_to_120` (gen_auto_proto_golden.py; every other golden byte-identical);
  `test_auto_link_host` 265 -> 268 (the 6-byte form posted with mode 1, a 7-byte one RANGE detail 0).
- Python: `test_espfly_auto` 203 -> 210 (the fake drone's target form, the older-drone fallback, UP_HEADROOM, the old
  photo warning, the arithmetic on the replays, the HOLD_LIMIT decode); `test_espfly_diag` 81 -> 87 (the trim check, on
  synthetic rows and on the logged flights; no port opened); `test_hold_band` 13.
- Unchanged failures: `test_gyro_tempco` 33, `test_mahony_replay` 4 (25.9).

### 27.7 Cost (`idf.py size` against 8277b68, the same sdkconfig, out of tree)

| | 8277b68 | this build | |
|---|---|---|---|
| static IRAM (.text) | 129619 B | 129659 B | +40: `slowInputs` +32 (the byte load, the compare, the call on a change), `autoFsmStep` -6 (the assembler's relaxation only: the same instructions, compared) and alignment |
| `_iram_text_end` | 0x40393f00 | 0x40393f00 | unchanged: inside the 256-byte step |
| .data | 31544 B | 31544 B | 0 |
| .bss | 88296 B | 88320 B | +24: the camera's run state and bits, the glue's last bits, autoFsm_t's three appended bytes |
| `_heap_start` | 0x3fcc1320 | 0x3fcc1338 | +24 B: the internal heap 24 B smaller (bench: `heapInt` ~18.4 KB with the Mac linked; the guard 12288) |
| flash | 951571 B | 952707 B | +1136 (.text +1096, .rodata +40): the target form, the headroom rule, autoFsmHoldLimit, camHoldLimitStep |

The 1 kHz path: `autoFsmStep` and every other IRAM function are unchanged but `slowInputs` (every 10th loop: the
byte compare). The camera task's per-frame tracker is flash and a few float compares. No new task, stack or NVS key.

### 27.8 Risks and what stays open

- The target form trusts the estimate's change between the photo and the command as motion (the offset form's
  assumption, 23.2). After a sink into ground effect (the tiger: the estimate rose 0.21 m while the drone sank) it
  under-corrects; the Mac warns on photos older than 10 s; the runbook: act within ~10 s, take a fresh photo after a
  yaw, a camera gap or a contact (the 10-16 F2 mistake).
- It ends at the commanded height within a vertical MOVE's settle tolerance (0.04 m + 10 %: up to ~0.08 m for a
  0.4 m flight), as every vertical MOVE; the closed loop's worst was 0.048 m at DONE.
- An explicit `--to` moves the commanded height: the LAND timings and the MOVE caps follow it, as after a MOVE up.
- A drone before this build gets the 4-byte form with X + h_now - target from the Mac: correct at the target, but its
  estimate stays ~0.1 m off, and a 0.5 m judgement may then exceed its |d| limit (SH_STEP).
- HOLD_LIMIT fires on the 10-14 gym flight 1 (the requirement said none on 10-14): the same condition as F1, not
  separable by the rule; F1's roll fires with 0.6 s to spare since the 1.2 deg keep (27.9; its pitch fires robustly).
  It is information only. The user decides whether the gym fire is acceptable.
- The trim check's normal sideways reading (-12 mg) is this drone's, from the 10-14 / 10-15 hovers after its IMU level
  calibration; a new level calibration or airframe moves it (`--ay-normal`).
- Not done (the user's or later): the late sink (the colour/sink plan's T2), raising the hold's 1.5 deg limit (it
  would hide this warning), any automatic trim change.

### 27.9 The review round (2026-10-16): what changed and why

Three reviews (code, flight safety, camera budget) of 8277b68..dfab760. With every switch at its default (`photocol`
0, `cvt2k` 0) none found a defect that changes how the drone flies; these are the fixes and the rebuttals.

**Firmware (flash only; the IRAM code is instruction for instruction dfab760's, see the cost below):**
- **A target-form descent keeps a MOVE down's floor** (`auto_fsm.c setHeightCmd`, `autoMotionHeightFloor`): a
  6-byte SET_HEIGHT whose flight goes down to a target under 0.25 m + min(0.30, 0.01 m/s x air) is refused HEIGHT
  (RANGE). Before, an explicit `--to` reached 0.25 m late in a flight, where a MOVE down is refused up to 0.55 m; an
  old photo can leave the drone lower than X says (the tiger correction). The 4-byte form keeps no floor (as 8277b68).
- **`upHeadroomOk` refuses a non-finite T_hold or ceiling** (its comment said so; the code accepted a NaN T_hold, as
  8277b68's battLow compare did). vert_ctrl never publishes one in HOLD; a refusal only.
- **HOLD_LIMIT keeps an open run down to 1.2 deg** (`CAM_HOLD_LIMIT_KEEP_DEG`): one frame just under 1.3 no longer
  restarts the 8 s. F1's roll run 8.1 -> 8.6 s (0.6 s to spare instead of 0.1), pitch 11.8 -> 13.8 s; every logged
  flight fires or not exactly as before, at the same times (1.15 would make F2 fire, so not lower).
- **A rise that cannot be reported is not taken as seen** (`autoFsmHoldLimit`): only falls are noted outside the
  wait or during a motion, so the camera's next change (the other axis, a sign) reports it. The glue still calls on a
  change only, so a rise followed by no change stays unreported until the run breaks and rises again (another 8 s):
  partial, by design (no call every 10th loop).

**The Mac (`tools/espfly_auto.py`):**
- **The clipped target is remembered** (`sh_goal`): a clipped set-height's target is the next set-height's default
  (`target_from` "remembered", `remembered_note`) until a set-height is not clipped (reached), a vertical MOVE
  (`remembered_target_cleared_m`), an explicit `--to` or a new sortie. Before, the clip's explicit target became the
  commanded height and a plain `set-height X` stopped there, `rest_m` short.
- **`new_m` in the legacy fallback**: the drone's estimate right after its shift, h_now + X' - h_at (1.05 in the test
  case where the true height is 1.15), no longer the true height; `flies_to_m` is now the true end (true_now + fly).
- **`set-height --legacy`**: the 4-byte form of 8277b68 (X, h_at; back to the old estimate; no clip, no target), for
  the bench's A/B (bench R1.3c). `--legacy --to` is an error.
- **The floor, locally**: the same target-form descent rule, with `why` HEIGHT and `floor_m`, before anything is sent.
- **Warnings (never refusals):** `events_since_photo` when the drone counted events (CONTACT, CAM_LOST, a catch's
  SINK, MOVE_STOPPED ...) after the photo's capture (the photo JSON's new `evCount_capture`, from the AUTO_STATUS
  snapshot at or before the capture; the F2 mistake); and **a climb late in the battery** (`late_climb_warning`:
  T_hold >= 51000 and the loaded battery < 3.20 V) on set-height and MOVE up.
- **Event hints**: CAM_LOST, CAM_BACK and HOLD_NO_VIEW say "with T2 on, a photo within ~5 s".

**F2's climb: a warning, not a refusal (rebuttal of the safety review's major finding).** leftdrift-2026-10-16.md
(step 7) asked that "a simulation of F2's situation must refuse"; the later plan (colour-sink-plan section 0) and the
user's 10-16 input ("you could have used more thrust") ask that climbs be refused only near the motors' limit. The logs
(every climb of 10-13..10-16 with its 2 s pre-climb mean thrust and battery, firmware-backups/handoff-2026-10-12/sink-replay/latebatt.py, `python3 latebatt.py 51000 3.20`): the
only climbs a rule "T_hold >= 51000 and V < 3.20 V" would have refused are **F2** (T_hold ~51989, 3.17 V: the climb
peaked 53.9k and ended DONE; 2.4 s later the thrust jumped from ~52k to the 58000 cap in a -0.37 m/s sink, HOLD_CAPPED
LAND) and **10-15 fastcheck** (T_hold ~52016, 3.12 V: peaked 53.9k, ended DONE, then held 52-53k for 16 s at 3.07-3.17 V
until its landing). The two are not separable by T_hold or voltage (fastcheck's were the worse of both), and F2's
thrust held ~52k for 2 s after its climb before the sudden sink: the climb did not visibly cause it. A refusal would
have stopped a good climb and kept F2 0.4 m lower before the same sink. So the Mac warns with both cases named, and
Claude decides from the photo; the drone's rule stays the thrust headroom (27.3). Making the warning a refusal is the
user's decision.

**`slowInputs` (rebuttal, wording fixed):** the hard rule says nothing new in the 1 kHz stabilizer path. On that task
the build adds one byte load and compare every 10th loop (+32 B IRAM, 27.7) and, when the camera's bits change (a few
times a sortie), a call into flash code (`holdLimitNote` -> `autoFsmHoldLimit`, a few hundred instructions, possibly
an instruction-cache miss). The same pattern was already there (`camVzGlueWhyBits()`, flash, from `slowInputs`
while the camera height is out). Moving the call to another task would race the state machine's report, which the
stabilizer task writes. The wording above (27, 27.7, bench R) now says so; bench R0 / A4 (>= 940 loops/s) checks it.

**Kept for the user:** HOLD_LIMIT fires on the 10-14 gym flight 1 (27.4); T2 stays off (29.6); the photo-latency mark
of bench N'1.3 (+0.30 s median against the plan's +250 ms); making `photocol` 1 the default after N'.

**Verification:** `test_auto_fsm` 1962 -> 1983 (the floor at 31 s of air: --to 0.54 HEIGHT, 0.56 OK, a climb and the
4-byte form unaffected; the ranges' --to 0.25 descent now HEIGHT and --to 0.60 OK; NaN / inf / negative T_hold and a
NaN ceiling refuse UP_HEADROOM, through new fake-controller fields that leave the thrust alone; HOLD_LIMIT's rise
during a motion reported at the next change); `test_camera_hold` 156 -> 159 (the keep: 1.21 stays, 1.19 breaks, 1.25
opens nothing, a 1.27-1.31 run fires at 8 s); `test_hold_limit_replay` 4 (the same fires); `test_espfly_auto` 210 ->
212 (the remembered target and its clearing by a MOVE, `new_m` / `flies_to_m` in the fallback, the floor, `--legacy`,
the event-since-photo and late-climb warnings). Every other suite unchanged; the known `test_gyro_tempco` (33) and
`test_mahony_replay` (4) failures only. The replay gates (sink-replay, `fwT2` byte-identical to this tree's
`cam_vz_obs.c/.h`) rerun identical to gates-70c537b.txt.

**Cost** (out of tree, ESP-IDF 5.0.7, against dfab760's build): static IRAM .text 129763 B, `_iram_text_end`
0x40393f00, .data 31544, .bss 88400, `_heap_start` 0x3fcc1388, RTC noinit end 0x50001d6c: all unchanged; every IRAM
function the same size and the same instructions (`autoFsmStep` differs only in the assembler's narrow/wide forms and
alignment padding, compared); flash +228 B (.text). Against 8277b68 cumulatively: IRAM .text +144 B, .bss +104 B (the
internal heap 104 B smaller), flash +6152 B.

## 28. Always-on colour: the camera driver's copy splits each frame (2026-10-16)

The user (2026-10-16): colour must be **always on** when enabled, with no grey/colour switching per photo, and
efficient. Section 24's build made every frame cost the flow ~1.9 ms more (bench N, 2026-10-15: `camProcAvgUs` 62.2
ms, past the 62.0 ms fallback, so the clock fell back to 20 MHz and half the frames were skipped): the flow's core
copied each YUYV frame once more through PSRAM (`ofLumaFromYuyv` into `lumaBuf`) just to get the Y bytes. This build
replaces section 24's path (the plan colour-sink-plan-2026-10-16.md section 2, option (b) with the reviewers'
corrections; the designs in colour-sink-design-2026-10-16.json). **`photocol` stays 0 by default**: grey, the camera
path's code as on 8277b68 (28.4); `photocol` 1 is always-on colour for the boot. Bench: autonomy-bench.md N (N').

### 28.1 How it works

- With `photocol` 1 the camera is initialised with `PIXFORMAT_YUV422`: the OV3660 streams YUYV (0x4300 = 0x30, Y0 U
  Y1 V) on **every** frame, at the same 160x120, clock, frame timing and rolling shutter as Y8.
- `cam_split.c` wraps three of esp32-camera 2.1.0's `ll_cam` functions at link time (`-Wl,--wrap`; `cam_hal.c` calls
  them from another object file, so the wrap applies; the component is pinned `==2.1.0` in idf_component.yml because
  the wrappers read its private `cam_obj_t`):
  - `__wrap_ll_cam_set_sample_mode`: after the real one, for YUV422 on the OV3660 outside the PSRAM DMA mode, sets
    `fb_bytes_per_pixel = 1`, the driver's own "YUV to grayscale" geometry: `cam_config` then computes fb 19200 and
    recv 38400. On **every** init (also the 4 s stall re-init) it resets the photo handshake (28.2).
  - `__wrap_ll_cam_dma_sizes`: grey's DMA geometry: half buffers of 3840 B (12 YUYV lines; node 3840, 120 % 12 == 0),
    4 of them in the same 15360 B ring, so the event queue stays 3 deep (the YUYV default is 2 x 7680 with a queue 1
    deep, the frame-drop risk the review named). It records the sizes for the boot check.
  - `__wrap_ll_cam_memcpy`: always calls the driver's own IRAM copy first: with fb 1 and in 2 it writes `out[k] =
    in[2k]`, the Y bytes, into the same 19200 B PSRAM frame buffers grey fills. **The flow gets exactly grey's frame
    layout from the driver's tested code, in the same buffers, and runs the same code** (`ofProcess(of, fb->buf)`).
    Only when a photo is armed does it also copy the raw 3840 B chunk from the internal DMA ring straight into the
    requester's PSRAM slot at `slot + 2 x off`.
- `camera_flow.c`: `cameraStart` enables the split and checks it after the init (`camSplitInitInfo`: active, fb
  19200, recv 38400, half 3840, cnt 4; the boot log prints `camera: colour split ok (fb 19200 recv 38400 half 3840 cnt
  4)`). A failed check deinitialises the camera, waits 200 ms and starts it in Y8 for the rest of the boot
  (`camSplit=bad`, grey photos). `frameGet` wants exactly 19200 B in colour. `lumaBuf`, its allocation and the
  `ofLumaFromYuyv` call are gone (the function stays in optical_flow.c as the host tests' reference; the linker drops
  it from the firmware).
- The wire format is unchanged: the 44-byte META, pixFmt 1 (YUYV), 38400 B, 38 chunks, the CRC over the whole frame.
  No chroma plane, no re-interleave, no Mac decoder change. The Mac keeps the raw bytes as `<stamp>_<id>_<idx>.yuyv`
  for the bench (`espfly_diag.py colourcheck`).

### 28.2 The photo handshake (`cam_split_core.c`, pure, host-tested)

```
IDLE --request--> REQUESTED --a frame starts (offset 0)--> CAPTURING --its last chunk copied--> CAPTURED
CAPTURED --the flow processes that frame (buffer and capture time match)--> COPYING (meta only) --> FILLED --take--> IDLE
```

- The photo is the first frame to **start** after the request (its chunks must all reach the slot).
- A new frame start while CAPTURING (the driver aborted the frame: EV-OVF, FB-SIZE) restarts the capture on it; a chunk
  out of sequence (never seen from `cam_hal.c`) re-arms.
- The flow sees a newer frame than the captured one while CAPTURED (GRAB_LATEST dropped the photo frame, or it was
  skipped as stale): CAPTURED -> REQUESTED, counted (`camSplitRearm`).
- **Cancel** (auto_link's 1000 ms frame timeout, `photoRelease`): REQUESTED, CAPTURING or CAPTURED -> IDLE with the
  generation counter incremented first; every chunk copies only while CAPTURING **and** with the generation it
  latched. The first line of defence is the cores: `cam_task` (CONFIG_CAMERA_CORE0, priority 23) and auto_link
  (core 0, priority 1; `_Static_assert(AUTO_LINK_TASK_CORE == 0)`, `cam_split.c` `#error`s without
  CONFIG_CAMERA_CORE0) share core 0, so a cancel never runs in the middle of a chunk. FILLED -> IDLE; COPYING refuses
  for the few microseconds the flow fills the meta (`photoRelease` spins as before).
- **Reset** on every camera init (the sample-mode wrapper, before the driver's copy task exists): CAPTURING or
  CAPTURED -> REQUESTED (the request stays armed, so the requester's 1 s timeout still applies), the generation
  incremented and the latched buffer, time and generation cleared: the stall re-init frees and re-allocates the frame
  buffers, likely at the same addresses.
- **Publish order:** the latched buffer and time are written only while the state is not CAPTURED; CAPTURED is stored
  with release after the last chunk's copy; the flow reads them after an acquire load of CAPTURED.
- The state words stay in internal RAM: the ESP32-S3's `s32c1i` compare-and-swap does not work on PSRAM.

### 28.3 Files

- `utils/interface/cam_split_core.h`, `utils/src/cam_split_core.c` (new): the handshake, the geometry check, the
  frame-buffer lookup (pass-through for an unknown pointer and for `out == in`, `cam_take`'s in-place call in the PSRAM
  DMA mode), `camSplitStep` (the wrapper's whole per-chunk step).
- `modules/interface/cam_split.h`, `modules/src/cam_split.c` (new, flash): the three wrappers, `camSplitSnap`,
  `camSplitEnable`, `camSplitInitInfo`; `#error` with the camera converter on (the size fields are then floats) or
  outside the ESP32-S3.
- `CMakeLists.txt`: the two sources, the three `--wrap` options on the component's INTERFACE link line, and
  esp32-camera's `target/private_include` (from `idf_component_get_property(... COMPONENT_DIR)`). `idf_component.yml`:
  `espressif/esp32-camera: "==2.1.0"` (the lock had 2.1.0 already; its manifest hash updated).
- `camera_flow.c` / `.h`: `splitWanted` (NVS photocol), `splitStat`, `photoColour` (the split running); the boot check
  and fallback in `cameraStart`; `snapshotFillSplit` (meta only); the request / take / cancel and the ground standby's
  pending check route to the core in colour; `cameraFlowSplitStats`, `cameraFlowSplitName`. The grey branch of
  `snapshotFill` is the same code (its two `photoColour ?` choices became their grey constants).
- `usb_diag.c` `status`: `camPix=gray|yuyv-split`, and a new piece `camSplit=off|ok|bad camSplitArm= camSplitDone=
  camSplitRearm= camSplitCancel= camSplitRestart=` (restarts + init re-arms), before `diagStackFree` (the line grows by
  ~95 characters to ~1220 of 1534). `cmdStatus`'s frame is unchanged (416 B; `usbDiagTask` 144 B, `outCat` 96 B, read
  from the disassembly), so `diagStackFree` is not affected; the handlers stay out of line (26.2).
- `auto_link.c`: comments and the core assert only (the wire format, the CRC and `photoRelease` are unchanged).
- Mac: `espfly_auto.py` keeps a colour frame's raw `.yuyv` (`files["raw"]`); `espfly_diag.py status` prints a
  `colour:` line (or a WARNING for `camSplit=bad`); `espfly_diag.py colourcheck GREY COLOUR [--white] [--orange]`
  (offline): the Y identity (mean |Y grey - Y colour| <= 2, min / max within 1, max >= 250 in both), the mean U / V
  and R / G / B, the white-sheet and orange checks.

### 28.4 Grey (`photocol` 0) on this build

Functionally identical to 8277b68, not bit-identical: the camera is initialised in Y8 as before; the wrappers call the
real functions and change nothing (`split.on` false): the driver's copy gains one flash call and two loads per chunk
(5 a frame, core 0, +32 B of `cam_task` stack), and every camera init runs `camSplitReset`. The flow's core: a byte
load and a branch in `snapshotFill`, `frameGet` and the ground standby's pending check. Every IRAM function is
instruction for instruction the same as 8277b68's (the disassembly of `.iram0.text` compared with the literal-pool
words removed: 0 differing lines; only literal words holding moved flash addresses differ). The observer, the hold,
the stabilizer and the camvz task are untouched.

### 28.5 Cost (`idf.py size` against the set-height build 0863660, the same sdkconfig, out of tree)

| | 0863660 | this build | |
|---|---|---|---|
| static IRAM (.text) | 129659 B | 129659 B | **0** (instructions identical; `__real_ll_cam_memcpy` is the driver's IRAM function, the wrappers are `.flash.text`) |
| `_iram_text_end` | 0x40393f00 | 0x40393f00 | unchanged |
| .data | 31544 B | 31544 B | 0 |
| .bss | 88320 B | 88384 B | +64: `camSplitSnap` 48 (atomics: internal RAM), the wrappers' flags and sizes 12 + 2, `splitStat`, `splitWanted`; `lumaBuf` (4) gone |
| `_heap_start` | 0x3fcc1338 | 0x3fcc1378 | +64 B: the internal heap 64 B smaller (bench ~18.4 KB with the Mac linked; the guard 12288) |
| flash | 952707 B | 954991 B | +2284 (.text +2052, .rodata +232) |
| .bin | 0x1103d0 | 0x110cb0 | 27 % of the app partition free |
| PSRAM in colour | (section 24: 2 x 38400 + 19200) | 2 x 19200 frame buffers, as grey | -57.6 KB against section 24's colour |

`nm`: `__wrap_ll_cam_memcpy`, `__wrap_ll_cam_set_sample_mode`, `__wrap_ll_cam_dma_sizes` defined. The literal-pool
search of the disassembly: `cam_task` (twice) and `cam_take` call `__wrap_ll_cam_memcpy`; the only code that loads
the real `ll_cam_memcpy` (0x40380188, IRAM) is the wrapper; `cam_config` calls the other two wrappers.

Stacks (the `entry` sizes in the disassembly): the copy task's copy path was `cam_task` 80 + `ll_cam_memcpy` 32 + its
`memcpy`; grey and unarmed colour add the wrapper's 32 B; a photo chunk goes through the out-of-line
`splitArmedChunk` 128 + `camSplitStep` 64 + `camSplitCopyChunk` 48 + `camSplitChunk` 32 (~+240 B, only while a photo is
captured), well under the depth the task's own log calls reach (`camStackFree` ~3980 of 6144 B on the 10-16 status). The
flow task's frame shrank 352 -> 336 B; `cameraStart` 32 -> 80 B.

**Per-frame cost, from the disassembly** (240 MHz; PSRAM line costs assumed 0.45-0.65 us per 32 B line as in the
colour-cost study; to be measured on the bench, N'1 and N'1.3):

| Path | Grey on 8277b68 | Colour on this build | Difference |
|---|---|---|---|
| flow core (1), every frame | the fit on the 19200 B Y8 frame | the same frame, buffers and code; `snapshotFillSplit`: a call and one load (~12 instructions) | ~0 |
| flow core (1), the driver's end-of-buffer interrupts | 5 a frame | 10 a frame (ISR `ll_cam_dma_isr` 31 + `ll_cam_send_event` 42 + `xQueueGenericSendFromISR` 89 instructions, the interrupt entry and exit; flash code, CONFIG_LCD_CAM_ISR_IRAM_SAFE off) | +5 x ~1.3-2 us warm, up to ~10 us with instruction-cache misses: **+6 to +50 us a frame** (the reviewers' 20-50 us is the cold end) |
| copy core (0), every frame | `memcpy` 5 x 3840 B (word copy, ~0.04 ms) | the driver's YUV-to-grey loop at 0x40380188: a zero-overhead loop of 10 instructions (4 `l8ui`, 4 `s8i`, 2 `addi.n`) per 8 input bytes, each store waiting on its load (~4 load-use bubbles): ~14 cycles x 4800 iterations (~0.28-0.30 ms; the review of 2026-10-16, first estimated 0.23); 10 copy-task wake-ups instead of 5 (+~10-25 us); the wrapper's two loads and `camSplitWants` (~30 instructions a chunk) | **+~0.25-0.3 ms of core 0** (~0.4-0.7 % of core 0 at 14.8-22 fps; priority 23; the PSRAM writes are the same 19200 B as grey) |
| copy core (0), the photo frame | - | 10 x `camSplitStep` (~100-120 instructions) + `memcpy` 3840 B to PSRAM: 1200 line write-allocates over the frame | +0.6-0.8 ms of core 0, spread over the frame's 67.5 ms |
| flow core (1), the photo frame | `snapshotFill`'s `memcpy` 19200 B PSRAM to PSRAM (~0.5-1 ms) | the meta only (~40 instructions); the copy core's 38.4 KB through the shared 32 KB data cache may evict some of the flow's state (+0.1-0.5 ms, the review's estimate) | **-0.9 to 0 ms**: the photo frame is no dearer than grey's |
| link task (0, priority 1) | CRC 19200 B, 19 chunks (~0.1 s) | CRC 38400 B, 38 chunks (~0.2 s a frame) | as section 24 |

So colour minus grey on the flow's `camProcAvgUs`: **~+6 to +50 us** (gate N'1: <= +150 us), against section 24's
+1.9 ms. Photo latency: **+70-190 ms** (0-67.5 ms to the next frame start, 67.5 ms of capture, the flow finishing its
frame and processing this one for the meta), far inside the 1000 ms frame timeout. Nothing is added to the 1 kHz
stabilizer path; the stabilizer task has no core affinity, so on core 0 it can wait behind one chunk copy (~22 us
instead of ~4 us): N'2's loop-rate gate checks it.

### 28.6 Verification (host)

- `test_cam_split` (new, 97 checks): a model of `cam_hal.c`'s copy sequence (2 frame buffers, the 4 x 3840 ring, 10
  chunks a frame, the driver's YUV-to-grey copy copied from `ll_cam.c:536-550`) through `camSplitStep`, the flow's
  side and the requester's: (1) the Y bytes equal `ofLumaFromYuyv`'s on unarmed and armed frames (also 0 / 255 / 16);
  (2) the armed slot is the raw YUYV frame byte for byte; (3) a request mid-frame latches the next frame; (4) a cancel
  mid-frame: no slot byte changes afterwards (poisoned slot), IDLE, the generation check alone stops a chunk; (5) an
  aborted frame restarts the capture; (6) a GRAB_LATEST drop re-arms, an older frame or another buffer never matches;
  (7) a re-init resets CAPTURING and CAPTURED, keeps IDLE, REQUESTED and FILLED; (8) pass-through for an unknown
  pointer, NULL and `out == in`; (9) the geometry check; (10) the latched buffer and time are never written under a
  published CAPTURED (a hook on every write over the soak); the refusals (one request at a time, COPYING refuses a
  cancel); a 20000-frame randomised soak (requests, cancels, aborts, drops, re-inits): every filled photo is its
  frame's YUYV with its buffer and time, every Y frame the luma. Mutations (no generation check, no re-arm, no restart,
  no reset re-arm, a write after CAPTURED) each fail it.
- `test_cam_split_wrap` (new, 29 checks): `cam_split.c` itself against host copies of the driver's
  `ll_cam_set_sample_mode`, `ll_cam_dma_sizes` / `ll_cam_calc_rgb_dma` and `ll_cam_memcpy` (2.1.0, esp32s3) and a model
  of `cam_hal.c`'s `cam_config` sizes and copy loop: grey keeps fb 19200, 4 x 3840, queue 3, 5 copies, also with the
  split enabled; the driver's own YUYV is fb 38400, 2 x 7680, queue 1; the split gives fb 19200, recv 38400, node
  3840, 4 x 3840, queue 3, 10 copies, and a whole frame passes the driver's FB-OVF and FB-SIZE checks with the Y bytes
  in the buffer; an armed frame lands raw in the slot with its buffer and capture time; every init (grey too) resets the
  core; the OV2640, 16 MHz and the split off stay the driver's.
- `test_auto_link_host` 268 -> 274: the camera stub runs the core with the driver's chunks: a 2-frame colour set (the
  44-byte METAs, each frame the raw YUYV of one whole frame, the CRC over it, 2 armed and 2 filled), and the 1000 ms
  frame timeout cancelling a capture stalled after 5 of 10 chunks: the frame is reported missing and its late chunks
  leave the slot untouched.
- Python: `test_espfly_diag` 87 -> 91 (the `colour:` status line, the firmware's status fields before
  `diagStackFree`, `colour_check` on synthetic frames incl. 16-235 clipping and a 3-level offset, `colourcheck` on
  saved photos and its refusals); `test_espfly_auto` 210 (the raw `.yuyv` of a colour photo, none for grey, in the
  existing PNG test).
- Every other suite unchanged; the known failures `test_gyro_tempco` 33 and `test_mahony_replay` 4 as before.

### 28.7 Risks and what stays open

- The OV3660's Y in YUYV is assumed identical to its Y8 output (the same ISP path, only 0x4300 differs). N'1.4 is a
  direct check (`colourcheck`: mean difference <= 2, the same min / max, max >= 250): a failure (for example 16-235
  clipping, which would break OF_SAT_LEVEL and the exposure gate) is a hard stop.
- The chroma matrix and range (white sheet, N'1.5) and the U / V order (the orange tiger reads R > G > B) are measured
  on the bench; the Mac decodes full-range BT.601 as before.
- The wrappers depend on esp32-camera 2.1.0's private `cam_obj_t` and `cam_hal.c`'s call pattern: the component is
  pinned and the boot check falls back to grey. A later driver version needs this code re-checked.
- The driver's YUYV `ll_cam_memcpy` loop is byte-wise (~0.23 ms a frame on core 0 at priority 23, the Wi-Fi task's
  priority); the core-0 budget is not measured directly (N'2 checks the loop rate, N'3 the frame regularity).
- The extra end-of-buffer interrupts land on the flow's core (`cameraStart` runs in the core-1 flow task, so the ISRs
  are allocated there): counted above; any later ISR work would land on the flow too.
- A photo set takes twice the Wi-Fi airtime (as section 24): N'4 near the battery end; the reserve is a 4:2:0 or 80x60
  chroma wire format (a new pixFmt, a separate build).
- Colour does not address the late sink (the plan's section 3, T2): that is a separate change.
- `photocol` 1 by default (the user's request) needs the user's OK after N' passes (plan section 7 item 4); this build
  keeps 0.

## 29. The late-sink fix T2: the offset drift rate (2026-10-16; built for host testing, default off)

The plan: firmware-backups/handoff-2026-10-12/colour-sink-plan-2026-10-16.md section 3 (S1-S4), its designs and
challengers in colour-sink-design-2026-10-16.json. The user's point (2026-10-16): "you could have used more thrust
since you seemed to still have battery". The study's answer stands: the tiger flight used at most 53.4k of ~58k; the
sink came from the height estimate (the accelerometer offset rising 0.12-0.28 mg/s from ~30-40 s, the camera
correcting it too slowly) and a 30 s old photo, not from thrust authority. Climbs are refused on thrust headroom only
(section 27, UP_HEADROOM), never on a voltage.

### 29.1 The law (cam_vz_obs.h "Offset drift rate")

- From `t2StartS` (30 s) after the sortie's FIRST lift-off (R1's clock: a bounce does not restart it) the offset
  estimate drifts: `x[2] += bRate * d` once per 10 ms block, applied in `closeBlock()` just before `predict()` (the
  replay's order; `predict()` itself is unchanged), and only in the air with an accepted update within 0.5 s
  (`CAM_VZ_T2_ACC_US`, the challenger's correction 2).
- `bRate` starts at the prior `t2PriorMgS` (0.124 mg/s, the pooled late slope of the 10-14/15 flights) and, at each
  ACCEPTED frame outside the guard hold, takes `t2kR` (0.5 /s) times that frame's offset correction (x[2] after the
  update minus before), clamped to +-`t2BoundMgS` (0.35 mg/s).
- The guard is R1's event set held 3 s: a refused frame (not ACCEPTED, QUALITY or NO_WINDOW), a CUSUM sum above 0.5
  x cusumH or an alarm, a re-lock, re-anchor or yaw restart, no tracked G, a commanded turn or motion, no accepted
  update for 0.5 s, and from the start the offset moving > 1 mg against its history entry >= 1 s old.
- Config `t2kR` (0 = off, 0..2), `t2StartS` (20..120), `t2PriorMgS`, `t2BoundMgS` ((0, 0.5]); `camVzObsConfigValid`
  refuses NaN/inf, |prior| > bound and R1 + T2 together. `camVzObsT2Get` (flash) reports it.

### 29.2 S1: the guard path split from R1 (bit-identical)

R1's event detection moved from `latePre`/`latePost` into `guardPre`/`guardPost` (they set only `hasGuard`/`tGuard`,
run the first-lift clock and keep the offset history, from R1's step or T2's start); R1's step, variance bound, held
offset and freezing stay in `latePre`/`latePost` and run only with `qLateMult > 1`. The entry runs the guard path for
`qLateMult > 1 || t2kR > 0`. Checked: the observer's every step (x, P, biasHeldG, h, v, healthy, lateOn) hashed over
the 26 replay inputs x 8 configs (R1 off, x3, x5 at 20-40 s, glitches, losses, gaps, the simulated Claude) is
740b6ad's, 208 of 208; `test_cam_vz_late` (x1 hash pin and every x3/x5 mark), `test_cam_vz_obs`, `_shadow`,
`_glue_host`, `_replay_ih` unchanged. With T2 on and R1 off, `biasHeldG == biasG` on every block and R1's step never
runs (test viii). The DUPLICATE question of the plan: neither R1 nor T2 ever sees a DUPLICATE or SKIPPED frame
(`camVzObsFrame` returns before the guard path); the replay's guard matches that (dsimg).

### 29.3 Settings and reports (S3)

NVS `cvt2k` (u8 0..200 = kR x 100, default 0) and `cvt2r` (i16 -350..350 ug/s, default 124), read once at boot by
cam_vz_glue.c; out of range or unreadable: refused loudly (boot log, `t2Cfg=bad`, `FSUM_CVL_T2_CFG_BAD`), T2 off;
`cvt2k > 0` with `cvqlate > 1`: T2 refused, R1 kept as stored. Reports: `camvz` `t2k t2r t2S t2Cfg t2
(off|wait|on|frozen|hold) t2OnS t2bR t2bRmin t2bRmax`; `status` `t2k t2r t2Cfg t2 t2OnS t2bR` (before
`diagStackFree`); the RTC flight summary (fsumCvLate_t +8 B: kR, prior, time on, end/min/max rate; magic E9;
`flights` `cvt2k cvt2r cvt2Ds cvt2bR=end,min,max`); `espfly_diag.py` prints all three ("T2 k0.50 r+0.124: on ..").

### 29.4 Cost (out of tree, ESP-IDF 5.0.7, against the colour build 740b6ad)

Static IRAM .text +104 B (129659 -> 129763; `camVzObsImu` +82 with the block close's T2 test, `camVzTask` +37,
`camVzObsGet` -12; `_iram_text_end` 0x40393f00 unchanged, the plan's < +200 B met), .data 0, .bss +16 B (two
settings, the snapshot's summary copy; `_heap_start` 0x3fcc1378 -> 0x3fcc1388), RTC noinit +128 B (7404 -> 7532 of
8192), flash +2504 B, .bin 0x1116e0 (27 % of the partition free). camVzObs_t 2160 -> 2192 B (PSRAM). Per block with
T2 off: one byte load and branch; on: one multiply-add and one add when drifting; per frame (flash, camvz task, core 0,
priority 1): the guard bookkeeping R1 already had plus ~6 flops. Nothing in the 1 kHz path. USB console frames:
`camvz` 944 -> 1088 B, `flights` 1056 -> 1104 B, `auto get` 416 -> 480 B, `status` 416 B unchanged (26.2's peaks
then ~2.8 KB for `camvz`/`flights`, >= 1.7 KB left before an interrupt).

### 29.5 Results: what passed and what did not

Host, tools/tests/test_cam_vz_t2.c (552 checks pass; the plan's marks are tallied and printed, "PLAN MARK"):
- 0: kR 0 is 740b6ad bit for bit (two closed-loop sorties' every-step hashes pinned from a build against 740b6ad's
  sources); kR 0.5 and 2 on a 29 s sortie identical to kR 0. **Met.**
- i (lag < 0.4 mg at 60 s, +0.2 mg/s from 30 s): **not met in the closed loop** (0/4: 0.44-0.82 mg, today 2.0-2.2;
  true height 30 -> 90 s -0.42..+0.06 m against today's -0.86..-1.33); met 6/6 on the observer alone (open loop,
  exact hover, clean camera, G 25 / 56).
- ii (|bRate| < 0.05 mg/s at 70 s, no drift): **not met** (0/4 closed loop: 0.10-0.35; 3/6 open loop). bRate swings
  within its bound, as the challenger found: it is a PI offset tracker, not a learned rate. The height with no drift
  is as today's within 0.10 m (gated).
- iii (reversed drift: bRate negative, within +-0.35): met 4/4 and 6/6.
- iv (glitches +-0.1/0.3 m/s 0.5-2 s at 50 and 60 s, +- the camera lost, induced change within today's + 0.1 m):
  **not met**: worst induced -1.19 / +1.00 m against today's -0.79 / +0.58; the late test's raw per-case mark missed in
  24 of 192 (negative glitches, mostly with the loss: a glitch the CUSUM took longer to see, absorbed by the offset,
  then flown on). Pinned so a change that makes it worse fails.
- v (the ground freezes bRate and the offset; a bounce keeps the clock; a motion, a turn and a yaw restart hold the
  adaptation 3 s, then it resumes), vi (the bound reached and kept at kR 2; NaN faults; 13 invalid configs refused,
  `camVzObsInit` false), vii (gaps of 3/5/10 s: the offset exactly frozen from 0.5 s after the last accept), viii:
  **met.** ix (the real 10-15 offsets, fixture): late drift 60 -> 90 s +0.03..+0.27 m against today's -0.20..-0.70.

Replay, firmware-backups/handoff-2026-10-12/sink-replay/ (gates.py, the firmware's own cam_vz_obs.c through dsimfw;
results in gates-70c537b.txt):

| Gate | Result |
|---|---|
| G1 firmware T2 vs dsim5's T2p (t2h 0.5, the firmware's guard set, dsimg) | **PASS**: lo and e60 (x60 for the two short flights) within 0.001 m on all 7; against dsim5's own smaller guard set 0.021 m |
| G2 lo, no Claude | **PASS**: p1 -0.09, p2 0.00, tiger -0.22, fastcheck 0.00, f2 -0.22, tiger16 -0.19 (>= -0.25); f1 -0.47 (>= -0.55); today -0.77/-0.39/-1.18/-0.63/-0.56/-1.00/-0.60 |
| G3 no drift / reversed after 30 s | **PASS**: worst hi +0.44 (today +0.44) / +0.52 (today +1.14) |
| G4 no drift + gaps from 36 s | **FAIL**: extra climb +0.35 / +0.52 / +1.04 m for 3 / 5 / 10 s gaps against today's +0.24 / +0.22 / +0.28 (+0.15 allowed: 3 s passes). The plan's fallbacks do not fix it: prior 0.06 +0.29/+0.44/+0.89, bound 0.25 +0.35/+0.52/+1.04, prior 0 +0.24/+0.36/+0.76, and each fails G2 (f1 -0.55, tiger16 -0.30, f1 -0.67 / tiger -0.26 / f2 -0.26). Kept: 0.124 / 0.35 as designed |
| G5 glitch at 50 s +- loss, induced 5 s after | **FAIL by 0.03 m**: -1.01 / +0.79 against today's -1.98 / +0.66 (+-0.10); the down side much better, the up side over |
| G6 camera speed bias from 30 s | reported: -2 cm/s worst hi +1.94 (today +1.25), -1 cm/s +1.12 (+0.44), +1 cm/s worst lo -0.84 (-1.85), +2 cm/s -1.46 (-2.52) |
| G7 harm set (16 flights; kh3, p2b_5, gym2 excluded: today's replay diverges on them) | **FAIL as worded** ("no change beyond a few cm"): ih1's lo -0.30 against -0.40, a 0.099 m improvement; the largest worsening of any flight 0.011 m |

The replay's G5 is a reconstruction (the study left no glitch script): it gives today -1.98/+0.66 where the design
quoted -1.77/+0.77, and the design's T2p -0.96/+0.86 against its -0.96/+0.82. The other study scripts are
reconstructed and checked against the study's pickles: stress.py and grid.py to 1e-4 m, claude.py exactly (840 runs).
The simulated Claude (claude.py, the firmware's T2): worst lo 30/30 -0.73, 20/10 -0.95, 15/10 -0.82, 10/5 -0.51 m
against today's -1.02/-0.99/-1.02/-0.67.

Caveats the plan requires, plainly: the prior (0.124 mg/s) and the "0.5p" choice were made with the 10-16 flights in
view (the 10-15-only grid had no prior; without it kR 0.5 gives -0.15/0.00/-0.26/0.00/-0.59/-0.27/-0.24); the late
"truth" after the last photo anchor is the camera itself (fastcheck, f1, f2, tiger16), which favours camera-trusting
fixes; the ranking survives +-1 cm/s of camera bias (worst lo today -0.64 to -1.85 against the firmware T2's -0.17
to -0.84, G6; the challenger's T2p -0.20 to -0.82); the
absolute numbers carry about +-0.3 m. The replay has no floor or support model, no CAM_LOST -> SINK and no capQ rule.

Verification (host, `run_ctests.py`, against 740b6ad): 40 C suites before, 41 after (`test_cam_vz_t2` new, 552
checks); changed: `test_cam_vz_glue_host` 54 -> 74, `test_auto_glue_core` 196 -> 200, `test_flight_summary` 49 -> 50
(`test_cam_vz_obs` 3053, its state-size pin moved 2160 -> 2192); every other count unchanged, all pass but
`test_gyro_tempco` (33) and `test_mahony_replay` (4), the known failures. Python: `test_espfly_diag` 91 -> 93,
`test_espfly_auto` 210, `test_hold_band` 13, all OK.

### 29.6 What it needs from the user

T2 is built and off (`cvt2k 0`): with it off the build flies as the colour build (and 8277b68's observer), bit for
bit. The plan asks for host tests, G1-G7 and the size checks to pass before the flash; they do not all pass (29.5),
so flashing T2 on, and every flight of bench Q, is the user's decision on these numbers. The reviews of 2026-10-16
(code and safety) both say T2 is **not fit to enable** as it stands: G4's +0.5..+1.0 m extra climb after a 5-10 s
camera gap and G6's +1.9 m under a -2 cm/s camera bias break the 2 m rule from a 1 m hover with nothing on board to
catch it (the estimate believes it is on target), and at a realistic 20/10 s photo pace the simulated Claude gains only
0.04 m on the worst low point (-0.95 against -0.99 m). The test suite gates regression bounds (better than today, the
glitch result pinned), not the plan's marks i, ii and iv, which it prints as "PLAN MARK ... not met" lines. The main new risk is a slow
climb when the camera is steadily wrong or briefly gone with no drift (G4, G6, iv): the photo cadence and the floor
LAND of bench Q are part of T2, not optional; so are (review 2026-10-16) a photo within ~5 s of CAM_LOST, CAM_BACK
or HOLD_NO_VIEW (the daemon's event hints say it), the abort on a photo > 0.5 m above or below the commanded height,
and never enabling T2 and colour for the first time on the same flight. Nothing adds an automatic landing; K_RANGE fires less on a surface.

## 30. The faster flow fit: room for always-on colour (2026-10-16)

The colour bench of 2026-10-16 (bench N' on 02cbbe0): grey `camProcAvgUs` 60.3 ms (max 63.1) against the 67.5 ms
frame at 13.33 MHz; always-on colour settled at 60.5-60.6 ms, and a DRY sortie with `camvz air full` and a photo took
it to avg 61.2 / max 65.3 ms: `camClkFallback=1` (20 MHz), 1012 skipped frames, colour off again. The user: "yes,
start on the faster image processing for colour". This build makes the flow fit itself cheaper, with the same
results bit for bit, so colour fits with a wide margin. `photocol` stays as it is (0 by default); grey and colour
run the same fit code as before (section 28's driver split is untouched).

### 30.1 Where the ~60 ms went

The fit's pixel loop (`refineKind`, `OFK_PLAIN` in the `flowfit plain` used since 19.11) ran 28256 pixel evaluations a
frame (coarse 3 iterations x 3744, fine 2 x 8512) at **274 instructions each** (the firmware's own object, emulated,
30.3): 172 of them the rolled normal-equation loop (`for i { b[i] += G[i] * e; for j <= i A[i][j] += G[i] * G[j] }`,
a load, a fused multiply-add and a store per accumulator, two loop branches), and ~35 taken branches. 7.73 M
instructions a frame; a rough in-order model of the LX7 (load-use and FP latencies, taken branches; cyc.py) puts its
CPU at **41-51 ms**. The rest of `camProcAvgUs` was the PSRAM copies (~1-2 ms) and core 1's preemption by the 1 kHz
tasks and interrupts (`busyUs` is wall time). The camera task's own work around the fit (brightness, snapshot, gyro
window, hold) is ~0.1-0.3 ms.

### 30.2 What changed (`optical_flow.c`)

- **The sums in registers.** The pixel loop computes each used pixel's terms (G[0..3] and e) into a 32-entry stack
  buffer (`ofTerm_t`, 640 B); `sumsFlush` (out of line, one copy for every kind) adds a full buffer in two short
  loops, `sumsHessian` (A[i][j], i, j < 4: ten accumulators and four terms in registers) and `sumsGradient` (b, sse,
  A[4][j]). Every sum sees the same pixels in the same order with the same operation as 02cbbe0's rolled loop: one
  fused multiply-add a pixel (written `fmaf()`: the firmware's `madd.s`, and no host vectoriser can split it), the
  G[4] = 1 terms an add (exactly the fma(1, x, s) the rolled loop did), A44 = n (1 x 1 summed n times). Only the order
  *between* different sums changes; no sum depends on another.
- **Hessian reuse.** A depends on the template's gradients and the set of used pixels, not on p. A pass after one
  that used every pixel in bounds takes that pass's A (`held`) and sums only b, sse and n; a pixel out of bounds in
  such a pass restarts it with the full sums (the same pixel set is then guaranteed by construction: the mask kind's
  saturation test reads the template only). In a hover 16000 of the 28256 pixel evaluations skip the Hessian.
- **One unlikely branch for the bounds** (the same four float compares, NaN still in bounds as before), the gradients
  with one conversion (`(float)(u - v)` of two bytes is the value `(float)u - (float)v`), the pixel count kept at
  the flushes.
- **The warp, rounded as 02cbbe0's firmware rounded it.** 02cbbe0's GCC formed the fused multiply-adds late, after
  moving loop invariants out of the pixel loop: in the plain, gain and mask kinds (nothing stored in the loop could
  change p) `p[3] * yr` left the loop as a rounded product and was then subtracted; in `OFK_STATS` (its statistics'
  stores through `stats` could alias p) it stayed in the loop, fused into the subtraction. A first draft of this
  build moved that product (both ways, in two drafts) and changed the firmware's results on ~1 frame pair in 3 while
  every host test passed. The warp is now spelled out with `fmaf()` and the row's product (`WARP_ROW_PRODUCT`: 1 on the
  ESP32-S3; 0 on the host, whose compilers fuse 02cbbe0's whole expression in every kind), so the code around it cannot
  move it again.
- **Stage times** (30.5): `ofProcess` stamps the core's cycle counter (CCOUNT, `rsr.ccount`; 0 on the host) at its
  stage boundaries into `ofState_t.stageCycles`.

### 30.3 The same results: how it was checked

**On the firmware's code** (`tools/flow_emu/flow_emu.py`, new): an interpreter of the ESP32-S3's Xtensa LX7 (windowed
ABI, float32 with fused `madd.s`/`msub.s`) runs `ofProcess` of `optical_flow.c` compiled with the firmware's own compile
command and linked alone (the firmware build's `optical_flow.c.obj` disassembles identical to the emulated object).
On 450 frame pairs (two seeds of `frames.py`: the drone's 159 photos and the five fixtures, each warped by one motion
kind: hover drift, rotations of 0.01-0.04 rad, shifts of 3.4-9.5 px past the margin (the reuse's restarts: 7 pairs),
exposure steps x0.9-x1.12, scale +-0.5-3 %, brightened and saturated; the consecutive real photo pairs; flat and
half-flat frames) in robust, lite and plain against 02cbbe0's object:
- every `ofResult` byte identical (450/450);
- every used pixel's terms identical and in the same order (each `b[i]` update's operands G[i] and e traced);
- every `ofSolve` input (A and b, 25 + 5 floats) identical.
The check is sensitive: the two drafts that moved the warp's product failed it on 28 and 75 of 225 pairs (the outputs
alone differed on fewer: one rounding of one pixel often vanishes in the sums). Instructions a plain frame: **7.73 M ->
3.18 M** (-59 %); the model's CPU **41-51 -> 14-16 ms** (robust 15-18 ms; the worst case seen, a robust frame with a
coarse restart, 17-20 ms). By stage (model): downsample 0.2, coarse 5.5-6.3, fine 8.6-9.8 ms; the copies' PSRAM time is
not modelled.

**On the host** (`tools/tests/test_flow_fast.c`, new): 02cbbe0's `optical_flow.c`, kept verbatim as
`tools/tests/optical_flow_ref_02cbbe0.c`, against this one: synthetic sequences (drift, fast motion, rotation 0.02
rad/frame, approach, an exposure ramp, motion past the margin) and the five fixture photos warped by the same kinds
(and saturated, flat, half-flat), every mode, state carried over frames: every output bit for bit, the previous-frame
copies too; and a sweep aimed at the Hessian reuse's restart (each fixture shifted 6-13 px and 3-5.5 px, with a little
rotation, scale and exposure, every mode), whose restarts the new fit counts in the host build only
(`-DOF_TEST_COUNT_RESTARTS`; the firmware's object is unchanged) and requires at least 100 of (219 today; the
sequences before the sweep reach 2). 68823 checks; with the restart kept off (the held Hessian used for a partial pass) 218 of them
fail, 216 in the sweep. It holds under the host's default contraction
and at -O0 and -O3; with `-ffp-contract=off` or `=fast` the reference itself rounds differently (unfused; or the
warp hoisted like the firmware's), which is expected. `test_flow_exposure`'s pinned hashes (14bcfc6's plain fit,
0ed68c9's robust and lite) pass unchanged; every other suite as before (the known `test_gyro_tempco` 33 and
`test_mahony_replay` 4); Python 212 + 93 + 13 OK.

### 30.4 Cost (`idf.py size` / `size-components` against 02cbbe0, the same sdkconfig, out of tree)

| | 02cbbe0 | this build | |
|---|---|---|---|
| static IRAM (.text) | 129763 B | 129763 B | **0**; `_iram_text_end` 0x40393f00 unchanged |
| .data / .bss | 31544 / 88400 B | 31560 / 88408 B | +24 B: `stageMux` 8, `stageAcc` 4, `stageResetReq` 1 (and alignment) |
| `_heap_start` | 0x3fcc1388 | 0x3fcc13a0 | the internal heap **24 B** smaller (~18.8 KB with the Mac linked; guard 12288) |
| flash | 957723 B | 961775 B | +4052 (.text +3852, .rodata +200); `optical_flow.o` 6439 -> 9369 B; libcrazyflie.a flash 215228 -> 219279 B |
| .bin | 0x1117d0 | 0x1127b0 | 27 % of the app partition free |
| PSRAM | | +116 B | the stage statistics (allocated with the fit state) |

Stacks (the `entry` sizes): `refineLevel` 336 -> 1104 B (the term buffer, `held`, the sums), `sumsFlush` 80 (below
`ofSolve`'s 112), `cameraFlowTask` 336 -> 352; the fit's path from the camera task 960 -> 1760 B of its 6144 B stack.
The task's measured worst was 2656 B (`flowStackFree` ~3488), deeper than the fit's path, so `flowStackFree` should not
move (at worst ~2.7 KB). `cmdFlowFit` 208 -> 256 B, far below `cmdStatus` (416) and `cmdCamVz` (1088): `diagStackFree`
unchanged. Nothing runs in the 1 kHz loop; no IRAM. The fit's hot code is ~0.5 KB as before (the pixel loop and the two
sum loops), so the shared instruction cache sees about the same footprint, and core 1 has ~30 ms a frame more idle.

### 30.5 Stage times: `flowfit prof [reset]`

`python3 tools/espfly_diag.py raw "flowfit prof"` prints

```
OK flowfit prof frames=N stages=ds/coarse/fine/copy/snap/post/total avgUs=a/b/c/d/e/f/g minUs=... maxUs=...
```

per processed frame with a fit since the boot or `raw "flowfit prof reset"`: `ds` the clear and the downsample,
`coarse` and `fine` the two levels' fits, `copy` the previous-frame copies and the result (ofProcess's stamps),
`snap` the mean brightness, the photo snapshot and the frame's return, `post` the gyro window, the drift hold and the
sample, `total` from the frame's arrival to the `camProcAvgUs` point (`total` avg ~ `camProcAvgUs`, which is an EMA).
Cycle counts of core 1 (the camera task is pinned there): wall time, so **min** is close to a stage's own CPU time and
avg / max include the preemption. Kept in PSRAM (116 B) under its own spinlock (not `sampleMux`, which the camera
height observer's IRAM path takes 100 times a second); `flowfit`'s plain reply is unchanged. Not in `status`, which is
near its 1536-character line.

### 30.6 The version string

02cbbe0 reported `fw=1`: IDF names the app by `PROJECT_VER`, else a `version.txt` in the project directory, else
`git describe` of it, else "1", and the drone's builds run from copies of the tree without `.git`. A copy now gets
`git -C <tree> describe --always --dirty > <copy>/version.txt` before its build (IDF makes the file a configure
dependency, so a reused build directory follows a new one); the image reads its commit (`esp_app_desc_t` version
checked in the `.bin`). `CMakeLists.txt` stops the configure when none of the three gives a version (a copy without
`version.txt` no longer builds an fw=1 image), and when a `version.txt` (git-ignored) sits in a git tree at another
commit. A `-D PROJECT_VER=<sha>` still works but is taken out of the CMake cache after its configure (the review of
e221e89: cached, it would have named an old commit in every later build of that directory, with no warning).

### 30.7 Expected on the bench, and what stays open

- `camProcAvgUs` in grey: **~20-35 ms** instead of 60.3 (the model's 14-16 ms of CPU, the copies' and the fine level's
  PSRAM misses, core 1's preemption; 02cbbe0's own model said 41-51 ms against the measured 60.3, and if those 10-19 ms
  of overhead add up unchanged the figure is ~25-35); `camProcMaxUs` well under 45 ms. Colour adds what it added
  before (+0.2-0.3 ms; the photo frame no dearer). The 62 ms fallback mark and its logic are unchanged and should never
  come near. Estimates: bench T (autonomy-bench.md) measures them.
- **The timing changes, not the values.** Each flow sample is published ~30-40 ms earlier in its frame; its values
  and its timestamps (the frame's) are 02cbbe0's. Downstream that is not "the same numbers sooner" everywhere:
  - the drift hold (`holdUpdate` in the camera task) acts on each sample ~30-40 ms sooner (less latency);
  - the camera height observer (`cam_vz_glue.c` `runBlock()` hands the newest sample to `camVzShadowFrame()` after the
    IMU block it follows) folds each sample in 3-4 blocks earlier. Its innovation uses tau = now - tMid
    (`cam_vz_obs.c` `frameUpdate`), so in theory this is latency-neutral (H = [0, 1, now - tMid]), and the age gate
    (`maxFrameAgeUs` 300 ms) rejects fewer frames, not more; but the observer's outputs are no longer bit-identical to
    02cbbe0's;
  - the fit's gyro window is unaffected (its samples, ~15-25 ms after the frame's vsync stamp, exist long before the
    frame has been read out);
  - at `camclk 20` (the 45 ms frame: `tkoff camclk 20`, or the fallback), 02cbbe0's ~60 ms fit skipped every other
    frame; this one processes every frame, so the hold and the observer see pairs half as far apart (each pair's
    result is 02cbbe0's). At the default 13.33 MHz in grey the processed frames are 02cbbe0's (it had `camSkips` 0);
    only colour's and the photos' skips go away, which is the point.
  The unroll note of 19 called such an output-timing change one that "wants its own bench and flight check": bench
  T's DRY sortie (T4) does not fly the hold, so T5 is a short hover-hold flight against the 2026-10-16 R flights'
  logs, with the user's OK.
- The Hessian reuse's restart (a pixel out of bounds in a reused pass) is rare in a still hover and likelier in real
  motion; each costs at most a redone pass (worst case ~25 ms of CPU a frame, still within the budget). `flowfit
  prof`'s `coarse` / `fine` max show it; T4 (DRY) and T5 (flight) read it.
- robust and lite (19.11: over budget at ~62 ms) should now fit too (model 15-18 ms of CPU): measured in T1 as
  information; the flown mode stays the user's choice (`flowfit plain` today).
- Not done (not needed for the target, or not exact): the lum sum fused into the downsample and a double-buffered
  previous frame (~0.3-0.8 ms, a change of `ofState_t`'s use); a 64-byte data-cache line (global); the fit in IRAM (no
  room); fewer iterations or sub-sampling (other results).

## 31. Fast moves: the mvspeed speed step (user request 2026-10-16, evening)

The user: "the movements are very slow ... forward, backward, left, right, turns: remove the limits for these, you
should be fast". A 1 m forward MOVE took 7.0 s of profile plus up to 3 s of settle, a 90-100 deg YAW 10-15 s with its
dwell and re-track bob, and a 3 m approach 17-20 s of a 60-75 s battery. The studies (scratchpad fastmove/: code
limits, the logged moves, physics and vision) found the binding limits in this order: Claude's own cycle between
commands (photo 0.8-0.9 s + 3.9-14.4 s of decision, median ~7.3 s), V_FWD 0.20 m/s, the 3 s settle (4 of the 5 logged
DONE moves ran it out: the sideways error never came inside 0.04 m), the bob after a turn (1.8-7.2 s), A_H 0.10 m/s^2
(2 s ramps), the 30 deg/s turn, W_FWD checked at the move's end distance. The flow fit itself tracks 5 px/frame and
1.5 /s of expansion; the camera's HOVER gates (1.5 px/frame sideways, 0.3 /s smoothed expansion, 45 deg/s in a turn)
and the hold's 2.5 deg / 6 deg/s lean bind first.

### 31.1 One setting

`mvspeed`, NVS u16 % 100..300, default 100 (`auto_glue_core.c` AUTO_SET_MVSPEED, after `cvt2r`). The drone reads it at
the next ARM (no reboot; `autonomy.c` sortieSpeedLatch, after the ARM step's autoMotionInit: autoFsmSetMvSpeed) and
keeps it for the sortie (settings writers are refused outside IDLE). AUTO_STATUS reports it (bytes 136-137,
mvSpeedPct: the NVS value, which in a sortie is the latched one), `auto status` prints `mvSpeed=`.

```
python3 tools/espfly_diag.py auto set mvspeed 150      # OK auto set mvspeed=150 stored=nvs (the drone in IDLE)
python3 tools/espfly_diag.py auto get                  # ... mvspeed=150(nvs); the Mac cache ~/.espfly/auto_get.json
python3 tools/espfly_diag.py auto reset mvspeed        # back to 100 (today's motions exactly)
```

At 100 every limit is the header constant and no new rule runs: the motions over a long script (MOVEs of every kind,
STOPs, turns with their bobs, view losses, a tilt, a frozen view, overshoots, SHORT, SET_HEIGHT, a contact, DRY,
refusals) hash bit-identical to 7ab74b9's (`tools/tests/auto_motion_identity.h`, test_auto_motion.c
testSpeedIdentity), the hold with its new fields 0 and the controller at the 6.0 deg/s slew are bit-identical too.

### 31.2 The limits of a step (auto_motion.h "fast moves", autoMvSpeedCfg)

s = pct / 100; each limit scales with s up to its own cap (the caps as changed by the review of cbccec5, 31.8; the
values before it in brackets):

| | 100 | 150 | 200 | 250 | 300 | rule |
|---|---|---|---|---|---|---|
| V_H (sideways, diagonal) m/s | 0.30 | 0.40 | 0.40 | 0.40 | 0.40 | min(0.30 s, 0.40) [0.60] |
| V_BACK (away from the scene) | 0.30 | 0.40 | 0.40 | 0.40 | 0.40 | min(0.30 s, 0.40) |
| V_FWD (toward the scene) | 0.20 | 0.30 | 0.40 | 0.50 | 0.50 | min(0.20 s, 0.50) |
| A_H m/s^2 | 0.10 | 0.15 | 0.20 | 0.25 | 0.30 | min(0.10 s, 0.30) |
| W_LAT /s | 0.12 | 0.14 | 0.14 | 0.14 | 0.14 | min(0.12 s, 0.14): 0.97 px/frame of the 1.5 gate |
| W_FWD /s | 0.15 | 0.17 | 0.17 | 0.17 | 0.17 | min(0.15 s, 0.17) [0.20], on the profile's true peak |
| brake m/s^2 | 0.15 | 0.225 | 0.30 | 0.375 | 0.40 | min(0.15 s, 0.40) |
| settle max ms | 3000 | 2000 | 1500 | 1200 | 1000 | max(1000, 3000 / s) |
| yaw deg/s, deg/s^2 | 30, 60 | 35, 70 | 35, 70 | 35, 70 | 35, 70 | min(30 s, 35), min(60 s, 70) [40, 120] |
| yaw dwell ms | 500 | 333 | 300 | 300 | 300 | max(300, 500 / s) |
| bob m/s^2 (cycle s) | 0.20 (1.79) | 0.20 | 0.20 | 0.20 | 0.20 | min(0.20 s, 0.20) [0.40]: as at 100 |

Above 100 only: W_FWD is checked on the profile's true peak of v / (D - s) instead of v / D_end (closed form
`autoMvFwdPeakCap`: none when sqrt(A / (2 De)) <= W, else v_pk <= (A / W)(1 - sqrt(1 - 2 W^2 De / A)); a grid of 330
moves stays <= W_FWD + 1e-4 along the whole profile and reaches it where the rule binds); the progress margins grow by
0.40 s x the peak speed (sideways x |ux| m, forward x uy / max(D - F+, 0.30)); the frozen-view reference is 0.10 +
0.40 x W_FWD = 0.168; a MOVE that loses its view pauses as at 100, and if the view is still lost 150 ms later
(AUTO_MV_FAST_VIEW_DEB_MS: three refused frames in a row, or a stall) with its reference faster than 0.20 m/s at the
pause or 0.40 s before it (the drone lags its reference), it is stopped with its brake from that speed (end VIEW_FAST =
13, the event MOVE_STOPPED VIEW_FAST; a turn, a vertical move or a slower reference pauses as before, NO_VIEW after
2 s). No landing is added. Unchanged at every step: TILT 8 deg / 100 ms, the view age
300 ms and NO_CAM 2 s, FRONT_STOP 0.40 m, the drone's 0.50 m and the Mac's 0.60 m floor at k 0.6 (0.25 / 0.30 / 0.35 m
and NVS settings since 2026-10-17, section 32), LAT_D_REL, the
length caps, the blind-turn 90 deg, D 0.30-6 m, the vertical limits, CAM_LAG 120 ms (95 ms would be ~1 cm at 0.5 m/s
and break the identity), the 45 deg/s turn gate, the camera's scene gates, the Mac's person minimum 1.0 m.

The hold (camera_hold.h camHoldMoveLimits, while a motion's reference or feedforward is active and mvspeed > 100): the
camera feedback (P + I, the learned trim included) is still clamped at 2.5 deg and the integral at 1.5; the
feedforward goes on top, the sum clamped at min(2.5 + (s - 1), 4.5) deg; coasting applies the feedforward too (a
stopped move's brake with the view lost); the output slews at min(6 s, 12) deg/s. controller_pid applies the
correction at the hold's slew of that frame (cameraFlowHoldSlew: 6.0 otherwise, the same float as the constant of
before) and, with the hold stale (no frame for 300 ms) and a fast MOVE's brake running, applies the brake lean
directly (autonomyBrakeLean; pilot input and a blocked hold keep priority). Both are flash code.

### 31.3 Times (profile; the settle and the bob run while Claude reasons, 31.5)

| mvspeed | 1 m forward, D 3 | 3 m forward, D 5 | 0.6 m sideways, D 2.5 | 1.5 m sideways, D 5 | 90 deg turn | 180 deg turn |
|---|---|---|---|---|---|---|
| 100 | 7.00 s (0.20 m/s) | 17.0 s (0.20) | 4.90 s | 8.00 s (0.30) | 3.50 s | 6.50 s |
| 150 | 5.33 s (0.30) | 12.0 s (0.30) | 4.00 s | 6.42 s (0.40) | 3.07 s | 5.64 s |
| 200 | 4.50 s (0.40) | 9.50 s (0.40) | 3.46 s | 5.75 s (0.40) | 3.07 s | 5.64 s |
| 250 | 4.12 s (0.39, the peak rule) | 9.22 s (0.39) | 3.11 s | 5.35 s (0.40) | 3.07 s | 5.64 s |
| 300 | 3.89 s (0.38, the peak rule) | 9.14 s (0.38) | 2.88 s | 5.08 s (0.40) | 3.07 s | 5.64 s |

(The table after the review of cbccec5, 31.8; before it 250 / 300 ran 1 m in 4.00 / 3.69 s, 3 m in 8.0 / 7.9 s, and
the turns 2.58 / 4.83 s from 133 %.) Turns stop improving above ~117 %: 35 deg/s keeps the logged per-frame overshoot
under the 45 deg/s camera turn gate. Near a scene the peak rule binds: 1 m forward at D 2 is 8.17 s at 100 (W_FWD x the
end distance, 0.15 m/s) and 6.40 s at 200 (DRY through the glue, test_autonomy_glue_host mvspeed-200); 2 m at D 3.24
(flown 10-16) 12.6 s at 100, 9.6-9.8 s at 150-300. With the early return and `--photo` a 1 m step is about 4.5 + 0.9 +
the decision at 200 (today ~15-18 s), a 90 deg turn about 3.4 + 0.9 + the decision. The battery fit: at 3.10 V under
load (20 s left) a 3 m move at D 5 is refused at 100 (17 + 3 s) and fits at 200 (9.5 + 1.5 s).

### 31.4 Why faster is still safe near things

- The hold drives the measured image rates to the reference's, so near anything the camera sees the speed is set by
  image rates, not metres: the true closing speed is at most lead x W x x_true / k (k 0.6-1), whatever the D
  estimate's error. The design took lead 1.35 (the logged 0.20 m/s moves); the sim measures up to 1.31 at k 1 but
  1.25-1.49 at k 0.6 (31.8; 1.75 at W_FWD 0.20), so the design's clearance table (0.27-0.31 m left after a front-stop
  trip at k 0.6) is not proven as written: what the sim shows directly is the Mac's longest approach at k 0.6 ending
  >= 0.54 m from the wall at every step, and bench U3 / flight F2 must measure the lead before 250-300 are trusted. The
  0.60 / 0.50 / 0.40 m guards stay as they are (they cover D and k errors, not reaction time).
- Stopping from top speed: 0.23 / 0.16 / 0.12 / 0.10 / 0.10 m from 0.2 m/s, 0.54 m from 0.5 m/s at 200, 0.44 m at
  300; 0.5 m/s is only reached with >= 1.1 m (k 0.6) of open space ahead. A STOP ends at most ~0.04 m past the plan.
- The person rule (Mac) adds 0.40 s x the forward peak to the k 0.6 overshoot above 100 (+0.12 m at 0.30 m/s, +0.16
  at 0.40, +0.20 at 0.50). A person approach is not slowed by the image rate (the background is D): the first person
  flights at <= 200 % with --person-gap 1.5.
- A lost view at speed: today's pause froze the reference and coasted on drag (0.67 m from 0.2 m/s, ~1.7 m from
  0.5 m/s); now a loss that lasts 150 ms stops it with its brake from the reference's speed then or 0.40 s before (so a
  loss on the profile's slowing part brakes too: in the sim it ends 0.09-0.28 m short of the plan where the review
  measured up to 0.45 m past it), the hold's coast path carries the brake lean, and a camera stall (no frames at all)
  gets it from controller_pid. One or two refused frames only pause the move, as at 100.
- Lean: <= 4.5 deg commanded plus 1-2 deg of logged wobble stays under the 8 deg TILT; thrust +0.3 % at 4.5 deg.
- The blind directions: sideways up to 0.40 m/s (W_LAT x D binds below D 2.9 m: 0.35 m/s at D 2.5; the design said
  "0.35 at D 2.5" for the ceiling, which was 0.45-0.60 m/s at 150-300 before the review), back up to 0.40 m/s (0.30
  today: 1.33x), forward up to ~0.40 m/s far from scenes (0.20 today: 2x). Thin obstacles Claude does not see in the
  photo are hit up to that much faster: hence the steps and the slow ramp-up (bench U).

### 31.5 The Mac (tools/espfly_auto.py)

- The 140-byte AUTO_STATUS (64 / 96 / 136 / 140 parsed; an older drone reads 100). An old Mac refuses the 140-byte
  status (ProtoError) and so refuses motions instead of misreading them: **the Mac tool and the firmware must come from
  the same commit**.
- `mv_limits(pct)` mirrors autoMvSpeedCfg, `move_profile()` the planning (tools/tests/mvspeed_golden.json, written by
  `test_auto_motion --write-golden`, pins 516 rows to 1e-4 s); the battery fit counts the step's settle, move_timeout
  its profile, the help strings print the cached mvspeed's speeds.
- **Early return** (the default above 100): `move` replies once the profile has ended (the drone in SETTLE holding the
  target), `yaw` once its re-track bob runs (`settling: true`, the travel so far, profile_s, mvspeed_pct). `--photo`
  takes the photo then and returns it in the same reply (one tool call less per step); `--full-wait` waits for the
  end as before. A `move` / `yaw` / `set-height` sent while that motion still settles or bobs waits for its end (on the
  AUTO_STATUS push), then every check runs again on the fresh status; it is refused if that motion ended other than
  DONE, SHORT or a reached PROGRESS ("the previous MOVE ended CONTACT after your photo: take a new photo"), after its
  settle + 4 bob cycles + 2 s, or by a STOP (withdrawn). `wait-move` still gives the real end.
- Claude's runbook above 100: `move ... --photo`, `yaw --deg N --photo`, judge the photo, send the next step.

### 31.6 Not done (with reasons)

- MOVE / YAW chaining inside the firmware during SETTLE: Claude's decisions (3.9-14.4 s) are longer than the new
  1.0-1.5 s settle; the Mac's early return plus the wait already overlaps the settle and the bob, and chaining during a
  bob would keep the camera height on its old G guess longer.
- Scene and turn gates relative to the commanded reference (sideways ~0.3 x D, forward 0.25-0.30 x D_now) and a
  gyro-seeded flow fit for 55-60 deg/s turns: phase 2, after the phase 1 flights.
- CAM_LAG 120 -> 95 ms: ~1 cm at 0.5 m/s, and it would break the identity at 100.

### 31.7 Tests and cost

Host: test_auto_motion (identity, the table and its clamping, the peak rule on 330 moves, the times, the settle and
dwell, VIEW_FAST, the margins and the frozen view at 200, the golden), test_camera_hold (the move limits, the coast
feedforward, the 12 deg/s slew, 0 bit-identical), test_controller_pid_ihold (6.0 bit-identical every loop, 12 deg/s,
the stale brake, pilot input and the blocked hold first; the manual hash of e46bf90 unchanged), test_auto_proto
(140 bytes, auto_status_ms golden, 136 / 96 / 64 still parsed), test_auto_glue_core, test_auto_fsm (the latch, the
VIEW_FAST event, the next ARM back at 100), test_autonomy_glue_host (mvspeed-100 / -200: the NVS value in IDLE, the
latch, the camera's step, the brake lean only while the brake runs, only above 100), test_cam_vz_obs (a turn's tracker
restart shows before the turn ends at 94 ms frames: the 300 ms dwell holds; the bobs at 0.40 m/s^2: G back in 0.57 s,
5 % at the end rule; the re-view bob 11 %; 12 deg/s pitch-rate steps with a 5 ms gyro window error: the height error
as at 6 deg/s), test_interactive_sim testFastMoves (150 / 200 / 300 against the same seeds at 100, kmax 0, 1 and 2.5:
the far moves DONE as often, as accurate, in their profile time, true image rates <= 0.21 / 0.15 /s, lead <= 1.35,
lean <= 3.2 deg; the mission; the Mac's longest k 0.6 approach >= 0.53 m from the wall; a STOP at 0.30-0.50 m/s
0.08-0.12 m further; the frames stopping at speed VIEW_FAST, 0.35 / 0.47 / 0.63 m further (100's pause: 0.55 m from
0.20 m/s); a frozen 2.5 m approach PROGRESS <= 1.3 m; YAW 90 / 180 / -45 at 40 deg/s (35 since 31.8) never paused, no
CAM_LOST);
Python (the status lengths, the limits against the header, the mirror, the person term; the fake drone's early
return with --photo, --full-wait, the wait behind and its refusals, STOP, the battery fit, a 136-byte drone as
before). Known failures unchanged: test_gyro_tempco 33, test_mahony_replay 4.

Cost against 7ab74b9 (idf.py size): static IRAM 130790 -> 130830 B (+40: the motion step's four field loads and its
RUN -> PAUSED call, +13 B; the state machine's and the glue's steps +8 / +4 B of alignment and a call; the status's
mvSpeedPct is filled on the readers' side, in flash), .bss +32 B (the motion memory +16, the setting, the status
copies, the latched step), flash +2.3 KB. 7ab74b9 had only 10 B of slack in its 256 B step of the IRAM / DRAM split
(memprot), so _iram_end moves 0x40393f00 -> 0x40394000 and the internal heap is 288 B smaller (256 + 32); the design
expected this step. Nothing new runs per loop on a fresh hold; the controller's stale branch and the camera task's
limits are flash code. Bench U measures the loop rate, the heap with the Mac and the USB stack.

### 31.8 The review of cbccec5 (2026-10-16, late) and what changed

Three reviews (code, flight safety, budgets) found no blocker; the flight-safety review found three major points. All
are fixed or answered here; at 100 the motions still hash bit-identical to 7ab74b9 (0x47E10B48658F1BE0).

**Major (fixed).**
- *Turns against the 45 deg/s turn gate.* The logged 30 deg/s turns (10-14 gym and p2, 10-16 tiger) ran 35.9-37.8
  deg/s over a 70 ms window (1.20-1.26 x the command; gyro peaks 43-49), and some already paused (39 rows in the gym
  180, 7 in the p2 45). 40 deg/s would put frames at ~48-50 deg/s. The cap is now 35 deg/s (x 1.26 = 44) and the
  acceleration cap 70 deg/s^2 (near today's 60, so the ramps' overshoot stays as logged): a 90 deg turn 3.07 s instead
  of 3.50 (2.58 at 40). The sim's heading has no overshoot, so the flights check it (bench U, F1: flowGz under 45 deg/s;
  the abort rule now compares PAUSED rows in turns with the 100 % flights instead of "any").
- *Sideways speed stated wrong.* "0.35 m/s at D 2.5" was the value at D 2.5, not the ceiling: 1.5 m sideways reached
  0.45 / 0.55 / 0.60 m/s at 150 / 200 / 300 with D >= 4-5 m. V_H is now capped at 0.40 m/s (back stays 0.40): the
  blind-side contact speed is at most 1.33 x today's 0.30 until the flights have shown more is wanted. 31.2-31.4 say
  so.
- *The hold's expansion gate.* The peak rule drives the reference expansion to W_FWD at the cruise's end, and the
  hold's HOVER scene gate refuses frames when the measured expansion's EMA passes 0.3 /s; the sim did not model that
  gate. It does now, in every scenario (camHoldSceneOk on each frame: the expansion EMA and |dxRes|; scen_t
  noSceneGate gives the sim before it); all 210 / 194 / 194 sim checks (kmax 2.5 / 1 / 0) pass with it. At W_FWD 0.20
  the Mac's longest k 0.6 approach put the EMA at 0.29 /s (0.18 at 100); W_FWD is now 0.17 and the EMA peaks at 0.24
  (k 0.6) and 0.20 (far scenes, k 1), checked (<= 0.27 / <= 0.25, no refused frame). Cost: 3 m at D 5 runs 9.1-9.5 s
  at 200-300 instead of 7.9-9.5, the forward peak ~0.40 m/s.

**Minor (fixed).**
- *A view lost on the profile's slowing part was not braked*: VIEW_FAST compared the reference speed at the pause,
  which comes 0.18-0.30 s after the last good frame, so a loss near the end coasted up to ~0.45 m past the plan. The
  check now takes the reference's speed then or 0.40 s before (the drone lags it) and brakes from that speed: in the
  sim (D 5.5 m, 1.3 m, the frames stopping with the reference at 0.25 m/s) every fast case ends VIEW_FAST 0.09-0.28 m
  short of the plan, the travel after the stall <= 0.16 m (100: NO_VIEW, up to 0.33 m after it, +0.07 m past).
- *One refused frame ended a fast move*: VIEW_FAST now needs the view lost for 150 ms (AUTO_MV_FAST_VIEW_DEB_MS: three
  refused frames in a row, or a stall); shorter losses pause and resume as at 100 (sim: 1-2 refused frames end DONE,
  4-5 end VIEW_FAST, at 150 / 200 / 300). The stall cases still stop within v (0.30 + 0.35) s + v^2 / (2 BRAKE) +
  0.05 m. Cost: a flag in a padding hole of autoMotion_t and one branch in the IRAM step (the check itself is flash).
- *The bob's thrust at the end of the battery*: BOB_ACC stays 0.20 m/s^2 at every step (the early return hides the
  bob; 0.40 doubled its vertical demand). The observer tests at 0.40 stay as extra coverage.
- *The lead behind the clearance table*: the design's 1.35 holds at k 1 in the sim (<= 1.31) but not at k 0.6
  (1.25-1.49 at kmax 2.5, up to 1.79 at kmax 0-1, reported). 31.4 no longer quotes the 0.27-0.31 m table as proven;
  what the sim checks directly is the Mac's longest k 0.6 approach >= 0.54 m from the wall, and F2 measures the lead.
- *The Mac*: the early-returned motion is forgotten at ARM and IDLE (the drone's mvSeq restarts at ARM), and a waiting
  command whose client went away leaves no entry (defer_fn on_gone). Test: test_waiting_client_gone_and_the_next_sortie.
- *Bench U*: U0 checks `fw=` and compares times and peaks with the host profile (bit identity is the host hash's; no
  7ab74b9 DRY dump exists and a real-time sortie is never bit-identical); U2 adds the stale-hold loop rate, the
  stacks of the stabilizer, camera and flow tasks, the camera budget with colour at 300, and a heap mark outside the
  old noise (>= 18.2 KB, heapMin >= 14 KB); the abort rules for PAUSED and VIEW_FAST say what counts.

**Answered, not changed.**
- *The heap is 288 B smaller* (the +40 B of IRAM of cbccec5 crossed a 256 B memprot step that 7ab74b9 had 9-10 B of
  room in). Getting it back means moving >= 31 B of IRAM out elsewhere or changing autoFsm_t's layout at the risk of
  the identity at 100; not done. **This needs the user's OK**: ~18.6 KB with the Mac expected against the 12288 B
  guard. This build adds 56 B more IRAM inside the same step (static IRAM 130886 B, _iram_end 0x40394000 as cbccec5,
  ~170 B left before the next step), .bss and .data unchanged.
- *autonomyBrakeLean on every stale loop* (manual flights, the ground, DRY): it returns after one load at 100 and
  outside a fast MOVE's brake; flash, well under 1 us. U2 measures the loop rate with the hold stale.
- *The brake lean in DRY or with the hold off*: a FLIGHT MOVE needs the hold applied to start (viewOk), so there is no
  fast MOVE brake without a hold; in DRY / BENCH the motors are off or clamped. Harmless.
- *VIEW_FAST and the stale brake are not shown on the bench*: DRY and BENCH treat the view as good (auto_fsm.c: `flight`
  only in FLIGHT), so a lens-cover bench step cannot trip it; the host tests carry it and F1's dump is the first real
  look.
- *An out-of-range stored mvspeed reads "(default)"* in `auto get`: the pattern every setting shares (it falls back to
  100, the slow side); `auto set` refuses out-of-range values.
- *History*: 70b8a1f alone fails four Mac tests (e7c304c fixes them) and cbccec5's "Docs:" subject hides its
  autonomy.c / auto_glue_core change (mvSpeedPct moved to the readers' side). The commits are kept as they are; bins
  are only built from a branch head.

## 32. Bolder ahead margins and a looser off-course guard (user decisions 2026-10-17)

The user, after the 300 % flights of 2026-10-17: "you are too scared to approach things. That's ok, you can take risks
... get close to humans about 0.5 meter, the drone is tiny and harmless, it can't hurt anybody." Approved: (1) people
0.5 m (the Mac's PERSON_MIN_M, 640989c); (2) the ahead margins lowered to about 0.35 / 0.30 / 0.25 m, the final values
from a braking analysis ("touching furniture occasionally is acceptable, hard crashes are not"); (3) the off-course
(progress) guard looser by default and adjustable over USB, because near a bright window it stopped two short approach
steps after 0.2-0.4 m. This section is (2) and (3).

### 32.1 What changed

| | before | now | where |
|---|---|---|---|
| the Mac's floor ahead (k 0.6) | 0.60 m | **0.35 m** = the drone's mvfloor + 0.05 | espfly_auto.py `mac_min_front()` |
| the TAKEOFF plan's floor (the Mac's, on the drone) | 0.60 m | **0.35 m** = mvfloor + 0.05 | AUTO_PLAN_MIN_FRONT_CM, AUTO_PLAN_FRONT_EXTRA_CM |
| the drone's MOVE floor (TOO_CLOSE) | 0.50 m | **0.30 m**, NVS `mvfloor` 20..100 cm | AUTO_MOVE_MIN_FRONT_CM |
| the live front stop (FRONT) | 0.40 m | **0.25 m**, NVS `mvfstop` 15..80 cm, <= mvfloor - 5 | AUTO_MV_FRONT_STOP_CM |
| a forward MOVE's view lost near the scene | the pause (coast) | **brakes, end FRONT** within 0.60 m (k 0.6; **0.80 m** and any horizontal MOVE since the review, 34), NVS `mvnear` 0..150 cm, 0 off | AUTO_MV_NEAR_SCENE_CM |
| the progress margin | 0.15 m + 30 % | **0.20 m + 40 %**, NVS `mvprogm` 10..60 cm, `mvprogr` 10..100 % | AUTO_MV_PROG_MARGIN_CM / _PCT |
| a back move's forward margin | 0.15 m + 30 % | the same (at most): its overshoot is blind | AUTO_MV_PROG_MARGIN_M / _REL |
| the sideways margin's floor | 0.25 m | **0.40 m**, NVS `mvlatm` 15..100 cm | AUTO_MV_LAT_MARGIN_CM |
| the SHORT end's off-the-line limit | 0.15 m | mvprogm (0.20 m) | settleEnd |
| sideways per MOVE | 0.5 x D | 0.5 x D (kept, 32.4) | AUTO_MV_LAT_D_REL |

The largest forward MOVE at D 1.5 / 2.0 / 3.0 / 4.66 m: 0.92 / 1.35 / 2.24 / 3.76 m (0.72 / 1.13 / 1.98 / 3.44 before);
the Mac's (and the plan's) 0.87 / 1.29 / 2.17 / 3.67 m (0.66 / 1.02 / 1.85 / 3.29 before). A forward MOVE's ideally
tracked end now sits 0.6 ln(0.30 / 0.25) = 0.109 below its front stop (0.134 before).

### 32.2 The braking analysis

**The stop itself.** The live front stop trips when the measured forward progress passes 0.6 ln(D_low / mvfstop): at
k 0.6 the scene is mvfstop ahead, at k 1 farther (D_low^0.4 mvfstop^0.6: 0.57 m from D 2 at 0.25). From there the
drone travels v t_d + v^2 / (2 BRAKE) with t_d ~0.55 s (the STOP's measured 0.35 s, the brake lean's onset at the 6-12
deg/s slew and the attitude, plus the frame's 0.12-0.19 s of age and interval) and BRAKE 0.15 / 0.225 / 0.30 / 0.40
m/s^2 at 100 / 150 / 200 / 300. How fast can it be there? The hold drives the measured image rates, so near anything
the camera sees the speed scales with the distance: tracked, v <= lead x W_FWD x x / k (lead <= 1.5 in the sims, W_FWD
0.15 / 0.17); above v = 0.30 x / k the hold's expansion gate (0.3 /s, smoothed) refuses the frames. At k 0.6 and the
trip distance x = mvfstop:

| mvfstop | v tracked (lead 1.5), 100 / 150-300 | stop, scene left | v at the gate | stop, scene left |
|---|---|---|---|---|
| 0.40 m (before) | 0.150 / 0.170 m/s | 0.13-0.16 m, 0.24-0.27 m | 0.200 m/s | 0.16-0.24 m, 0.16-0.24 m |
| 0.30 m | 0.112 / 0.128 | 0.09-0.11, 0.19-0.21 | 0.150 | 0.11-0.16, 0.14-0.19 |
| **0.25 m** | 0.094 / 0.106 | 0.07-0.08, **0.17-0.18** | 0.125 | 0.09-0.12, **0.13-0.16** |
| 0.20 m | 0.075 / 0.085 | 0.06, 0.14 | 0.100 | 0.07-0.09, 0.11-0.13 |

The stopping distance shrinks with the distance itself (v ~ x), so lowering the stop costs little clearance: at 0.25 m
the drone keeps >= 0.13 m to the scene in the worst case the camera can still see (>= 0.08 m beyond the sim's 0.05 m
guard radius), 0.17 m when tracked. 0.25 m was kept rather than 0.20 (0.11 m left at the gate).

**What the analysis found in the closed loop** (test_interactive_sim testFrontBraking): the drone's largest forward
MOVE from D 2 / 3 / 4.5 m at k 0.6 / 0.73 / 1.0, 100-300 %, with an overshoot forced by a forward push while the move
runs (0 / 0.10 / 0.137 / 0.20 m/s^2; 0.137 is the forensics' 0.8 deg of unlearned lean, overshoot300-verdict.md). With
the margins alone (0.30 / 0.25 m, no near-scene brake) a push of 0.10-0.137 m/s^2 put 2-6 of 36 fast moves into the
wall at up to 0.24 m/s, and 0.15-0.20 at 100 % 1-5 of 36 (the margins before: none at 100, 2-3 of 36 at 200-300
from 0.137). The stop never tripped in those: the camera's expansion gate refused the frames first (the approach was
faster than 0.5 x the distance), the MOVE paused, and a pause freezes the reference and coasts on drag (at 100, and above
100 below VIEW_FAST's 0.20 m/s reference speed, which the slowing end of an approach always is). Hence the near-scene
rule: **a forward MOVE whose view stays lost AUTO_MV_FAST_VIEW_DEB_MS (150 ms) while its last measured progress puts the
scene within mvnear (0.60 m) at k 0.6 is stopped with its brake from max(its reference's speed then or 0.4 s before,
0.20 m/s), end FRONT, at every speed step** (auto_motion.c nearScene, pausedFast; flash: the RUN -> PAUSED transition
arms it in the padding flag the fast moves use; only a RUN pauses, a view lost in the settle ends NO_VIEW after 2 s as
before). With it, at the defaults (kmax 2.5, 36 moves per cell):

| push m/s^2 | 100 % | 150 % | 200 % | 300 % | the margins before, 100 / 150 / 200 / 300 % |
|---|---|---|---|---|---|
| 0 | 0.28 m, no contact | 0.29 | 0.28 | 0.28 | 0.46 / 0.46 / 0.47 / 0.48 |
| 0.10 | 0.23 (FRONT 3) | 0.23 | 0.19 | 0.21 | 0.38-0.39, no contact |
| 0.137 | 0.20 (FRONT 7) | 0.18 | 0.16 | 0.19 | 0.36 / 0.36 / **2 contacts** / **3 contacts** (<= 0.29 m/s) |
| 0.20 | 1 contact at 0.08 m/s | 0.13 | 0.12 | 0.17 | 0.32 / **4** / **8** / **10 contacts** (<= 0.41 m/s) |

(the closest true approach of the camera to the wall; the sim's guard radius is 0.05 m). Every FRONT stop's travel
stayed within v 0.55 s + v^2 / (2 BRAKE) + 0.05 m. At kmax 1 and 0 (the hold before its gain scale) 0.20 m/s^2 still
reaches the wall in 1-2 of 36 at 200-300 (the view lost farther out, a slow reference): information, as is 0.20 at
100; the check is no contact up to 0.137 at every kmax and step. The undisturbed largest moves end 0.28-0.29 m from
the wall (k 0.6, the observer's D exact); with its -10 % error and three drags, the Mac's longest approaches in the
long-move grid and the plan end >= 0.24-0.25 m (0.40-0.43 before) and the Mac's longest k 0.6 approach of the fast-move
test >= 0.30 m (0.54 before).

So the new margins are bolder when nothing goes wrong (0.28 m instead of 0.46 m at the end of the longest move) and,
with the near-scene brake, safer than the old ones above 100 % under a disturbance; at 100 % a 0.9-1.2 deg unlearned
lean can now bring the drone within a few cm (one touch at 0.08 m/s in 36). Below the floor's model (k 0.4-0.5, which
nothing on the drone measures) the Mac's longest moves came within 0.13-0.15 m at k 0.5 (0.27-0.31 before) and reached
the wall at k 0.4 (testBelowKLow, information): the runbook's rule stands (forward moves of 1.5-2 m in a room until one
long move there has measured k >= 0.7 by photo).

### 32.3 The progress and sideways guards

The four PROGRESS stops of 2026-10-17 (the 300 % flights, a bright window ahead or behind the scene) were all the
sideways margin: forward 0.9 at D 2.85 stopped with 0.27 m sideways, forward 1.0 + right 0.35 at D 2.91 with 0.36 m
right after 0.06 m forward, back 0.8 / 0.7 with 0.25 / 0.26 m sideways (the reply's metres; the guard reads them at the
reference's distance now). The sideways margin was max(0.25, 0.15 + 0.30 |lateral|) m (+ 0.4 s x the peak speed above
100); it is now max(0.40, 0.20 + 0.40 |lateral|): the three forward steps would have gone on, a 1.5 m sideways move
has 0.80 m (0.60 before). The forward margin toward the scene is 0.20 m at the end distance + 40 % of the ln target
(its behind side catches a frozen view, AUTO_MV_FROZEN_* catch it earlier; its ahead side is the front stop's). A back
move keeps the forward margin of before (min of the setting and 0.15 m + 30 %): its overshoot is blind, and the 300 %
back move overshot 0.45 m uncaught with it (the OVERSHOOT end is a separate fix of the forensics). The SHORT end's
off-the-line limit follows mvprogm (0.20 m). What the looser guard costs: a sideways runaway (the camera does not see
the sides) is stopped 0.15 m later; a contact there stops it (CONTACT) and the hold holds; nothing lands.

### 32.4 Sideways per MOVE: 0.5 x D kept

AUTO_MV_LAT_D_REL bounds the overshoot of a sideways move off a near object onto a far background (the hold integrates
v / D_now: a depth step makes the move D_far / D_near too long). Near a person at 0.5-1 m (the new person rule) D is
the person: 0.5 x D is 0.25-0.5 m, and stepping off the person onto a 3 m wall would make it 1.5-3 m on the blind side.
A larger ratio makes that worse in exactly the new close-approach case; Claude splits a longer move (the drone re-checks
D in between) and a lateral shift over 0.35 x D re-views anyway. Not changed, not a setting.

### 32.5 Settings and the Mac

Six NVS settings (auto_glue_core.c, after `mvspeed`), read at the next ARM with mvspeed (autonomy.c sortieSpeedLatch:
autoMvMarginsFrom clamps them and autoMotionSetMargins latches them for the sortie; refused while active):

```
python3 tools/espfly_diag.py auto get                    # ... mvfloor=30(default) mvfstop=25(default) mvprogm=20(default)
                                                         #     mvprogr=40(default) mvlatm=40(default) mvnear=80(default)
                                                         #     mvfix=59(default)   (mvnear 60, mvfix 27 before the review, 34)
python3 tools/espfly_diag.py auto set mvfloor 50         # back to the drone's 0.50 m (the Mac then keeps 0.55 m)
python3 tools/espfly_diag.py auto set mvfstop 40         # the live stop of before (at most mvfloor - 5 at the latch)
python3 tools/espfly_diag.py auto set mvlatm 25          # the sideways margin of before (mvprogm 15, mvprogr 30 too)
python3 tools/espfly_diag.py auto set mvnear 0           # no near-scene brake (the motions before 2026-10-17)
python3 tools/espfly_diag.py auto reset mvfloor          # the default again
```

`autoMvMarginsOld` (50 / 40 / 15 / 30 / 25, the plan 60, mvnear 0) reproduces the motions of before bit for bit:
test_auto_motion's identity script hashes to 7ab74b9's 0x47E10B48658F1BE0 with it, at mvspeed 100 alone and with
autoMotionSetSpeed(100); the settings of before give the same but the plan's 55 (+5 instead of +10). The latched
margins live in auto_motion.c (7 B of .data), not in autoMotion_t: the motion memory's layout, the identity hash's bytes
and the IRAM code's offsets into autoFsm_t are unchanged; only the once-per-command code (flash) reads them, the step
keeps reading the per-move margins it already had. The FSM reports a near-scene stop as MOVE_STOPPED FRONT (the step's
AUTO_MVL_FAST report now carries the MOVE's end, VIEW_FAST or FRONT). AUTO_STATUS is unchanged (140 bytes).

The Mac (tools/espfly_auto.py): `mac_min_front()` is the drone's mvfloor as the last `auto get` cached it (else 0.30)
+ 0.05 m, used by the MOVE check, its refusal's "at most F m now", the plan's refusal and the move help; `cached_margins()`
reads the six. PLAN_MIN_FRONT_M and MAC_MIN_FRONT_M are 0.35 (the header's AUTO_PLAN_MIN_FRONT_CM, checked by the test
against the header). After an `auto set mvfloor`, `espfly_diag.py` re-reads `auto get` for the Mac's cache; a Mac
without the cache keeps 0.35 m (with a drone set higher the drone refuses TOO_CLOSE and the reply gives its largest).

### 32.6 Risks and what stays open

- **The height near the end of long approaches.** In the long-move grid the true height's excursion through the 17-22 s
  moves reached 0.335 m (0.266 m before, a 25 s plain hover 0.249 m): the longest approaches now end 0.24-0.30 m from
  the scene, where the camera height observer's G (scaled by the measured progress, which reads k x the true one at
  k 0.6) over-reads by (D0 / D_now)^0.4. The check is now the hover's + 0.10 m (+0.05 before). Flights: watch the height
  at the end of approaches longer than 2 m; a SET_HEIGHT from the photo corrects it.
- **At 100 % under a disturbance** (32.2): a 0.9-1.2 deg unlearned lean can bring the drone within a few cm of the
  scene, which the old 0.50 m did not; the forensics' fixes (feed-forward, the stillness gate) attack the cause.
- **The near-scene brake at 100 %** changes a behaviour at 100 (a pause near the scene used to coast): it only runs when a
  forward MOVE's view stays lost 150 ms within 0.60 m (k 0.6), brakes away from the scene, and ends the MOVE FRONT
  (STOPPED: the drone holds, Claude decides). A person or a glare crossing the view right at the end of an approach now
  ends the move there instead of resuming.
- **Below k 0.6** the floors are thinner (32.2): forward moves in a new room stay 1.5-2 m until a photo shows k >= 0.7.
- Not changed: the person rule (Claude's estimate, PERSON_MIN_M 0.5 m with the k 0.6 overshoot and the speed term),
  K_LOW 0.6, the D range 0.30-6 m, the tilt and view guards, the frozen-view rule, the fast-move margins' lead term.

### 32.7 Tests and cost

Host: test_auto_motion (the defaults, the clamps, the settings of before; the floor and the plan's at their TOO_CLOSE
boundaries at four D and a sweep of 4000+ accepted moves x k 0.6-2.5 (every true end >= 0.30 m); mvfloor 45; the front
stop's distance and gap; the sideways margins and the window steps of 10-17; the forward and back margins; the SHORT
limit; a back move's contact anchor at the floor; the near-scene brake at 100 / 200 / 300 and not farther, not on a back
move, not with mvnear 0; the identity hash with the margins of before; the tests written before 2026-10-17 run with
those; the golden profiles rewritten, 534 rows), test_auto_glue_core (the six settings, every mvfloor x mvfstop),
test_auto_fsm (the floors at their boundaries), test_autonomy_glue_host guard-margins (NVS in IDLE, latched at ARM,
clamped, refused while active, TOO_CLOSE under mvfloor 45, the defaults after a reset), test_interactive_sim (the Mac's
floor follows the drone's; the k-floor, plan, long-move checks at the new distances; testFrontBraking), Python (the
Mac's floor and its refusals, the auto get keys).

Known host failures unchanged: test_gyro_tempco 33, test_mahony_replay 4; test_interactive_sim 211 checks at kmax 2.5,
195 at 1 and 0, all pass; Python 334 tests.

Cost against 4dea542 (`idf.py size` / `size-components`, the same sdkconfig, out of tree): static IRAM 130886 ->
130894 B (+8: autoMotionStep +4 B of the assembler's branch-target alignment, its instructions unchanged; no new IRAM
code), `_iram_end` 0x40394000 as before (the same 256 B memprot step); .data +16 B (the latched margins, 7 B), .bss
+24 B (the six settings' RAM copies): `_heap_start` 0x3fcc14c0 -> 0x3fcc14e8, the internal heap **40 B smaller**
(~18.6 KB with the Mac expected, against the 12288 B guard); flash +0.9 KB (libcrazyflie .text +682, .rodata +269).
Nothing new on the 1 kHz path: the margins and the near-scene check are read by flash code only (moveFloor, settleEnd,
contactBackOff, runViewLost, pausedFast). The USB console's `auto get` frame grows 512 -> 672 B (its line buffer is
AUTO_SET_COUNT x 28 B): the console's `auto ...` peak ~2.8 KB of its 4768 B, `diagStackFree` well over 1024. The
camera task is not touched (its 4064 B free on the 4dea542 bench is unchanged by this build).

## 33. The move fixes of the 300 % forensics (2026-10-17)

The first 300 % flight of 2026-10-17 (flight 6: forward 1.00 m replied 0.69 / 0.30 right, back 1.00 m replied -1.38 m
for a true ~-1.66 m) was taken apart by a forensics workflow (firmware-backups/handoff-2026-10-12/overshoot300-verdict.md):
the camera tracked cleanly, the errors came from how the hold controls the drone and from logic gaps in the firmware.
The user approved the fixes with the margins of 32 ("you can take risks"). This section is the verdict's items (a)-(i).

### 33.1 What changed

One NVS setting, `mvfix` (a bit mask, after `mvnear`; read at the next ARM with the margins: autoMvMargins_t `fix`):

| bit | name | what | default |
|---|---|---|---|
| 1 | DRIFT | dTrack (the D the last forward / back move measured) follows the hold's measured drift since; MOVE, SET_HEIGHT and a contact's snap rebase it (autoMotionTrackNow) | on |
| 2 | LONG | a settle timed out past its target ends LONG (end 14, state DONE); a back move's forward margin at its end distance | on |
| 4 | STILL | a horizontal MOVE in FLIGHT holds its profile at its start until the hold's rates are still (0.05 /s for 0.3 s), at most 1.5 s | **off** (33.4) |
| 8 | GAIN | the hold's gain scale in a MOVE toward the scene from the measured progress; above 100 % no "untracked" cut for 1.0 s | on |
| 16 | BRAKE | a braking feedforward lean scaled down by the measured over the reference rate (0..1), once the camera saw the axis move | on |
| 32 | FRONT | (the review, 34.1) the live front stop and the near-scene brake on every horizontal MOVE, not only a forward one | on |

`mvfix 59` is the default (27 until the review added FRONT, 34), `mvfix 0` the motions of before: with it and the margins
of before the identity script hashes to 7ab74b9's 0x47E10B48658F1BE0 (test_auto_motion testSpeedIdentity). Without a
flash: `auto set mvfix 0` (none), `57` (no LONG), `63` (STILL too), `27` (no FRONT). The camera's two (GAIN, BRAKE) reach camera_flow.c through `autonomyMvFixBits()` (0 outside
a sortie: manual flights fly the hold as before).

Constants above 100 % (not a setting until 35: NVS mvacc / mvsettle, these values their defaults; mvspeed > 100 only):
the acceleration cap AUTO_MV_FAST_A_H_CAP 0.30 -> 0.20 m/s^2 (250 and 300 % now plan as 200 %: 1 m forward at D 3 4.50
s, 3.89 s at 300 before) and the settle floor AUTO_MV_FAST_SETTLE_MIN_MS 1000 -> 2500 ms (2.5 s at every step above 100;
3.0 s at 100 as before). The Mac mirrors both (MV_FAST_CAPS, MV_FAST_SETTLE_MIN_MS; mvspeed_golden.json rewritten).

### 33.2 DRIFT: the scene distance after a move

`trackD()` set dTrack at a move's end and `sceneD()` kept using it while the drone drifted. Flight 6's move 1 ended
SHORT with dTrack 1.669 m; between the moves the drone drifted 0.193 ln back (0.35 m) and move 2 was planned at 1.669 m
while the camera's own count put the drone at 2.02 m: the back 1.00 m was ln(1.669 / 2.669) = -0.4705, 1.21 m at the
true distance (and the Mac's metres were low by the same factor).

Now dTrack is the scene distance at the hold's measured forward progress dTrackRef of the current epoch (0 at a new
epoch; a DONE move's end: its measured end, the same epoch). A command first moves it by the progress since:
dTrack x e^-(p - ref) (k 1), its lower bound x e^(-(p - ref) / 0.6) toward the scene (x e^-(p - ref) away from it),
then plans with it (flash, autoMotionTrackNow; FLIGHT and the camera on the epoch only). A sideways or vertical move
keeps dTrack, moved by its own measured forward progress; a STOP's snap starts from D at the stop; a contact's snap
without a motion rebases first (auto_fsm.c contactStart), and so does the hush-end snap (hushSnapEnd, since the
review: before it the bounce's drift was lost). The
replay of flight 6 (tools/tests/test_speed300_replay.c): move 2 planned at **2.017 m**, its target -0.4026 ln (1.00 m
there). The status's sceneD shows the rebased D once a move is accepted (the Mac's `drift_since_last_end_m`, 33.8);
between moves it is the last end's, as before (correcting it every step would put an expf in the 1 kHz step).

### 33.3 LONG: an end past the target

`settleEnd` only knew SHORT: a drone 0.2-0.45 m past its target ended DONE, and the hold then walked it back toward
"old anchor + move" unreported (flight 6's move 2 drifted forward to -0.35 ln after its DONE). Now a settle that times
out past the target along the move by more than the settle tolerance (0.04 m + 10 %) ends **LONG**: as SHORT, the
measured end becomes the move's end (the hold's reference), the drone holds there, the state is DONE and the travel is
reported (LONG is tested before SHORT's off-the-line test: the overshoot is the news). In the replay flight 6's move 2
ends LONG at its old target (the measured -0.5705 ln the reference after it) and, planned at the corrected D, LONG at
32.05 s reporting -1.64 m (as flown the drone reported -1.28 m at its end and the Mac's early reply -1.38 m).

Since the review (34.3) a LONG move **with a part toward the scene** (fwdM > 0) keeps its target and only reports
LONG: held at the overshoot, an unlearned forward lean crept it ~0.1 m further toward the scene or a person; the hold
walks it back to the target instead, and with DRIFT its D follows it there. A back or sideways LONG re-anchors as above.

A back move's forward margin (its PROGRESS guard toward the start) is in metres at its end distance D0 + |F| (it was
at the start distance D0: about 0.9 m at the 300 % move's end); the base and the fast lead term both. This changes a
back move at 100 too (the bit off: as before).

### 33.4 STILL: implemented, off by default

The verdict asked to hold the profile at its start until the hold's rates are under 0.03 ln/s for 0.3 s (at most
1.5 s): move 2 was accepted while drifting back 0.15-0.17 m/s. Built so (the camera task's camHoldMoving on the filtered
measured rates, published as one byte, read with the glue's slow inputs; the step's only change: tRun held at 0 while
`stillWait`, released when still or after AUTO_MV_STILL_CAP_MS). But the replay of flights 6 and 7 shows the hold's
filtered rates at 0.04-0.07 /s (median of max(|fwdF|, |latF|)) in plain hover, 0.10-0.14 /s at the 90th percentile:

| threshold | flight 6: mean wait, hit the 1.5 s cap | flight 7 |
|---|---|---|
| 0.03 /s (the verdict's) | 1.15 s, 53 % | 1.42 s, 90 % |
| 0.05 /s | 0.70 s, 16 % | 1.09 s, 53 % |
| 0.07 /s | 0.52 s, 7 % | 0.72 s, 19 % |

At 0.03 /s a start would wait the cap most of the time and then go anyway; flight 6's move 2 itself read 0.070 /s at its
acceptance, inside flight 7's plain hover. In the closed-loop sim (33.9) STILL changed nothing once DRIFT measures the
drift before the acceptance. So the bit is off by default, its threshold CAM_HOLD_STILL_RATE 0.05 /s when on
(`auto set mvfix 31`), and the 1 kHz path carries what it costs anyway: one byte copied in mvIn and one test while a
MOVE runs.

### 33.5 Not done: clearing the integral at the acceptance (the split reference)

The verdict's alternative to the stillness gate: "clear the old epoch's integral at accept, or measure the guards and
settle against old anchor + move". The hold's integral is the position (rel units) and KI s x it is the lean the hover
needs: 0.8 deg of lean need is a 0.2-0.4 ln offset (KI s = 2-3.7 deg per unit). Clearing it at a new epoch (a snap at
every MOVE) would drop the learned lean and restart the very drift; measuring against the old anchor would count that
lean's offset as the move's error. Neither is built; DRIFT puts the drift into D, LONG / SHORT report where the drone
ended.

### 33.6 GAIN and BRAKE (the camera task)

**GAIN** (camHoldGainMoveFix, called before camHoldGainMove each frame): the gain scale during a MOVE toward the scene
followed the reference's distance (D_low e^(-refFwd / 0.6)), so it fell 1.84 -> 1.0 by 15.99 s of move 1 while the
drone lagged its reference by 0.21 ln: the hold was weakest when the drone lagged. Now the target is the measured
progress's distance, D_low e^(-max(pFwd, 0) / 0.6) (k at its low: still the lower bound, never above the move's start
scale), also before the tracking test applies (the plain 1 only while the camera does not see the move: untracked).
Above 100 % the tracking test waits CAM_HOLD_GAIN_GRACE_S 1.0 s from the move's start: the 300 % speed-up read under 0.3 x
its reference for ~0.7 s and the 'untracked' cut took the scale to 1.22. Replay: 1.15 at 15.99 s (1.00 as flown).

**BRAKE** (camHoldInput_t ffBrakeScale): a feedforward that opposes its axis's reference rate (a brake) is multiplied by
the measured over the reference rate through the same 0.25 s filter (fwdF / rateFwdF, latF / rateLatF), clamped 0..1,
on engaged frames, once the axis's measured progress is at least half its reference (a frozen view keeps the full
brake: in the sim a frozen sideways move went 0.72 m without that test), and not below a reference rate of 0.01 /s.
The planned 1.5-1.8 deg brake of move 1 hit a drone at 0.09 m/s (reference 0.24) and slid it back 0.04 ln; the replay
scales it to 0.35 x. Only down: 1.5 x up (the verdict's clamp) over-braked the sim's moves near their end, where the
reference rate is small and the lagging drone faster than it (the fast mission then had a move over 25 % at 150-300). The
speed-up and drag leans are not scaled: a drone behind its reference needs more of them, not less (the verdict's
"advance tRun only while tracking" would put a per-step governor into the 1 kHz step: not built).

### 33.7 The recorder (item h): not done

The 124-byte record has no spare byte; pFwd / pLat, iFwd / iLat, the gain scale and the feedforward split would grow it
by 12-16 B (the 100 Hz sample is in the stabilizer task, the dump format and its parsers change, PSRAM grows by 0.2 MB).
The replay reconstructs all of them from the existing fields (the hold within 0.024 / 0.011 deg rms of the logged
correction over flight 6's moves), so the test infrastructure covers what the fields would show.

### 33.8 The Mac (tools/espfly_auto.py, item c)

- LONG is a normal end (MVE_OK: ok, a note "ended LONG ... past the target": the drone holds there, or for a move with a
  part toward the scene "the drone's hold walks it back to the target", 34.3).
- An early reply above 100 % gives `travelled_pre_settle_m` instead of `travelled_m` (the 300 % "travelled" was taken at
  the first settle status), with its along / off.
- The next MOVE / YAW / SET_HEIGHT, which waits behind an early-returned motion, carries that motion's final end and
  travel under `previous` (also on a refusal); `wait-move` gives it as before. (Until the review, 34.4, a SET_HEIGHT
  neither reported nor consumed it: it came with the next MOVE / YAW, one motion late.)
- `move_reached` also judges a normal end and an early reply: a `warning` when the measured travel is more than 10 %
  past or short of the target, or off its line by more than 0.15 m + 10 %.
- `drift_since_last_end_m`: the drone's D at the acceptance (D_TRACK, rebased by DRIFT) against its D at the last
  move's end (+: toward the scene). Since the review (34.4): the end's D as the daemon first saw it at that end (a
  refused command meanwhile already moves the status D by part of the drift), and no field when the cached mvfix has
  DRIFT off (the drone then measures none: a 0.000 would read as "no drift").
- `sideways_m_at_mean_D`: a diagonal's sideways travel at the mean of its start and end distances (the drone converts
  at the start D; flight 6's 0.30 m right was 0.19-0.21 m).

### 33.9 Tests

- **Replay** (new tools/tests/test_speed300_replay.c, built with -DAUTO_MV_HOST_VH_OVERRIDE for the as-flown caps): the
  hold on the recorder's frames as camera_flow.c runs it, the motion module on every record. As flown (mvfix 0, the
  margins before): move 1 SHORT at 18.49 s with dTrack 1.6688, move 2 at 1.6688 m DONE at 32.46 s, the hold within
  0.024 / 0.011 deg rms; flight 7's PROGRESS stops within 0.01 s. The fixes: 33.2, 33.3, 33.6; flight 7's back moves run
  past their logged PROGRESS stops with the margins of 32. 11 checks.
- **Sim** (test_interactive_sim testDriftLean): forward / back 1.00 m from D 2.3 m at 100 / 200 / 300 %, 4 seeds:
  undisturbed, a 0.17 m/s kick 0.5 s before the send (0.11-0.14 m/s at it), a 0.8 deg nose-up step 3 s before (0.06-0.10
  m/s at it), 0.8 deg ramped in over 5 s during the move (either sign); the defaults against none and STILL:

| (kmax 2.5 / 1) | the defaults | without the fixes | with STILL |
|---|---|---|---|
| undisturbed: worst end off the command | 0.18 / 0.14 m | 0.18 / 0.15 m | 0.18 / 0.14 m |
| a kick at the send: worst normal end | 0.17 / 0.13 m | 0.21 / 0.11 m | 0.17 / 0.13 m |
| a lean step before: DONE hiding > 0.20 m | 2 / 0 of 24 | 11 / 8 | 2 / 0 |
| a lean in the move: DONE hiding > 0.20 m | 4 / 2 of 48 | 20 / 19 | 4 / 2 |
| the reported travel against the true one at the end | 0.22 / 0.20 m | 0.22 / 0.25 m | |

  **Not met: the verdict's "must still end within 0.15 m"** under a persistent unlearned 0.8 deg (a lean step before the
  send or a lean in the move): the true end is 0.3-1.1 m off with or without the fixes, and more moves end PROGRESS
  (18 / 12 of 24 for the lean step at kmax 1). That is the hold's position gain (33.5): 0.8 deg needs a 0.2-0.4 ln
  offset. What the fixes give there is honesty: LONG / SHORT / PROGRESS instead of a DONE that hides 0.2-1 m. A stiffer
  hold or a learned trim in flight is the fix for the offset itself (open, 33.11).
- **Units**: test_auto_motion 492 (DRIFT with flight 6's numbers, LONG, STILL), test_camera_hold 198 (the brake scale,
  stillness, the measured gain), test_auto_glue_core (mvfix), the fast-move sim checks (the profile time less a still
  wait; the stall bound + 0.15 s for the 2 s speed-up's lean reversal). C: every suite but test_gyro_tempco (33) and
  test_mahony_replay (4); sim 215 / 199 / 199 at kmax 2.5 / 1 / 0; Python 339.

### 33.10 Cost

Against 4dea542 (and 4f5d979), the same sdkconfig, out of tree: static IRAM 130886 (130894) -> **130922 B** (+36 / +28;
`_iram_text_end` 0x40394000 as before, 133 B left in the 256 B memprot step): auto_motion.c -3 B (the still test, and
the bob's end code now loaded), auto_fsm.c +7 B (mvIn copies holdMoving), autonomy.c +24 B (slowInputs, 100 Hz, reads
the camera's byte). `_heap_start` 0x3fcc14c0 (0x3fcc14e8) -> 0x3fcc1500: the internal heap **64 B smaller than 4dea542**
(24 B than 4f5d979; ~18.55 KB with the Mac expected, guard 12288). libcrazyflie: flash .text +1897 B, .rodata +316 B,
.bss +50 B. The 1 kHz path gains one byte copy (mvIn) and, while a MOVE runs, one test of `stillWait`; everything else is
flash code once per command (autoMotionTrackNow, settleEnd) or the camera task per frame.

Stacks: the camera flow task's holdUpdate frame 304 -> 320 B (-fstack-usage); its deepest known path (cameraFlowTask >
ofProcess > refineLevel > ofSolve, 1744 B) is unchanged since 7ab74b9. **camStackFree is the esp32-camera driver's
`cam_task` (6144 B), not the flow task** (that is flowStackFree: 3432 at 7ab74b9, 3428 at 4dea542). None of this repo's
code changed in cam_task between 7ab74b9 and 4dea542 (camHoldStep +16 B and three 32-48 B leaves are the flow task's).
The 5332 -> 4064 B is most likely what ran: 5332 was 7ab74b9's grey readback right after the flash, 4064 the 4dea542
bench with colour photos and camvz air, and the driver's `ESP_LOGE("FB-SIZE: %u != %u")` (in the 10-17 crash dumps
several times) runs esp_log / vprintf in cam_task, ~1.2 KB of newlib stack. Bench W2 checks it. The USB console's
`cmdAutoGet` frame 672 -> 704 B (the line buffer AUTO_SET_COUNT x 28), still under cmdFlights' 1104 B.

### 33.11 Risks and what stays open

- **The hold's softness** (33.9): an unlearned lean of 0.8 deg moves the end 0.3-1 m with or without these fixes. The
  fixes report it; trim (atrim, the trimcheck of the dump) and a ground-truth flight (the verdict's item 12, tape marks
  or a phone video) come before more speed.
- **More PROGRESS ends** under a persistent drift (the back move's margin at its end distance, the frozen-view rule on a
  forward move that starts while drifting back): the drone holds and Claude re-photos.
- **LONG re-anchors where the drone is** (a back or sideways move; a move toward the scene keeps its target since the
  review, 34.3). Correction (the review): only the live front stop runs in the settle, the near-scene brake only arms
  from a RUN's view loss (runViewLost), and neither runs after DONE / LONG.
- **GAIN keeps a higher gain while the drone lags** (bounded by the move's start scale and the lower-bound distance); a
  frozen view after the 1.0 s grace falls back to the plain target at 2 /s as before.
- **BRAKE trusts the camera's speed** once the axis's progress reached half its reference; a k < 1 scene reads both rates
  alike (the ratio is unaffected).
- Not flashed, not flown. The Mac tool must come from the same commit (LONG, the replies).

## 34. The review of the bolder margins and the move fixes (2026-10-17)

Three reviews of 640989c..70444c8 (correctness, flight safety, budget) found no blocker, three safety majors and a set
of minors. All three majors are fixed here, behind settings, with each fix checked in the reviewer's adversarial sim
harness (T1-T7) against the real code. Not flashed, not flown.

### 34.1 FRONT: the front stop and the near brake on every horizontal MOVE (major 1)

After a forward move the drone parks 0.30-0.42 m from the scene (32). There only a forward MOVE had the live front stop
(progFwdMax) and the near-scene braked view loss (nearScene). A sideways or back move had its forward PROGRESS margin,
mvprogm / D = 0.20 / 0.32 = 0.63 ln, about 0.11 m to the scene at k 0.6. New mvfix bit **32 FRONT** (on; mvfix default
**59**, range 0..63): progFwdMax = 0.6 ln(D_low / mvfstop) for every horizontal MOVE, and nearScene for any MOVE with
a horizontal part. A vertical move or a turn holds as a hover does (no reference to run past). A view loss in a
sideways move within mvnear now ends it FRONT with its brake along the move, not with a pause.

The sim (test_interactive_sim testSideNearScene, the reviewer's T1): the largest sideways MOVE (0.5 x D) from D0 0.32 /
0.36 / 0.42 m, 100-300 %, k 0.6 / 0.8 / 1, 3 seeds, both sides:

| push toward the scene | with FRONT: contacts | without FRONT |
|---|---|---|
| none | 0 (one FRONT in 90) | 0 |
| 0.10 m/s^2 | 0 | 3 (up to 0.10 m/s) |
| 0.137 m/s^2 (0.8 deg) | 0 | 5 (up to 0.18 m/s) |
| 0.20 m/s^2 | 1 at 0.14 m/s | 20 (up to 0.25 m/s) |
| a kept 0.8 deg lean (T1 mode 4) | 1 at 0.13 m/s (its view lost in the settle, NO_VIEW) | 8 (70444c8: mvnear 60) |

The one remaining lean case loses its view in the SETTLE, where only the live front stop runs. A near brake in the settle
removed it in the sim (the reviewer's vsP 2), but it is a new branch in the IRAM step (about 149 B left in the 256 B
step): not built (34.8).

### 34.2 mvnear 60 -> 80 cm (major 2)

nearScene reads the last measured progress, which lags the drone by about 0.12 s plus the filter. In the traced case
(T7: 300 %, k 0.6, a 0.137 push, a 0.6 s frame refusal) the expansion gate tripped at a true 0.536 m at 0.33 m/s; the
measured progress read 0.850 against the 0.861 of 0.60 m, so the move only paused, and it coasted 2 s into the wall. At
the default **mvnear 80** that run ends FRONT 0.34 m from the wall. T3 (frames refused or stalled 0.25 / 0.6 s at
1-9.25 s, the largest forward MOVE, k 0.6 / 1): contacts 3 + 1 in 96 + 96 at 300 % with mvnear 60, **0** in all 1536
runs at 80 (at kmax 2.5 and at 1). testFrontBraking and testDriftLean are unchanged at 80. The cost: more FRONT ends from a plain view
loss near the scene (T3 undisturbed at 100 %: 5 of 96 FRONT instead of a pause and resume); FRONT is a hold, and Claude
re-photos. The brake's own blind backward travel is at most 0.09 m at 100 % and 0.13 m at 200 % (T5).

### 34.3 LONG toward the scene keeps its target (major 3)

LONG re-anchored the hold at the overshoot. Under a persistent unlearned forward lean the drone then kept creeping
toward the scene or the person: 0.51-0.56 m past a 1.00 m command at 0.8 deg, against 0.41-0.46 m without LONG. Now a
LONG move with a part toward the scene (fwdM > 0) keeps its target and only reports LONG; the hold walks it back. dTrack
is the measured end, and DRIFT follows the hold back at the next command. A back or sideways LONG re-anchors as in 33.3.
T6 (1.00 m forward, k 1, 100-300 %, 0.5 / 0.8 / 1.2 deg): with LONG the overshoot is now the same as without it (0.41-0.46
m at 0.8 deg, 0.28-0.31 m at 0.5), and the LONG report stays. One effect remains as before LONG existed: the next
move's fwdLnBase (the height observer's gain) adds the measured end, not the target.

### 34.4 Minors

- **Mac, SET_HEIGHT and `previous`**: op_set_height now reports and consumes the waited-behind motion's final end (it
  went stale and came with the next MOVE / YAW). Test: early MOVE, then set-height, then MOVE.
- **Mac, `drift_since_last_end_m`**: no field when the cached mvfix has DRIFT off (drift_measured(); no cache = on). The
  D at the last end is the one the daemon first saw at that end (d_end), so a refused command in between no longer
  shrinks the reported drift. MVFIX_DRIFT is used; MV_MARGIN_KEYS includes mvnear and mvfix.
- **DRIFT at the hush-end snap**: auto_fsm.c hushSnapEnd (flash, 96 B frame like contactStart's) calls
  autoMotionTrackNow before the snap.
- **STILL's cap** counts from the acceptance (stillMs, a uint16 in started's padding: the struct is unchanged), not from
  the latest PAUSED -> RUN. STILL stays off by default.
- **Docs**: 33.11's sentence that mvnear runs in the settle was wrong (only the front stop does; corrected there). The
  USB `auto set` usage lists mvfix.
- **Bench**: W0 reads `status` after `auto get` (diagStackFree is a low-water mark); W2 has explicit floors
  (camStackFree >= 3500, flowStackFree >= 3330, diagStackFree >= 1024; U2's 5230 is a fresh-boot number); DRIFT, LONG and
  STILL run only in FLIGHT, so W-F1 is their first run on the drone. Bench X adds X0 and two flight checks.
- **Not changed (rebutted or accepted)**: the back move's forward margin at D0 + |F| with LONG (tighter in ln, as
  disclosed; it catches a back move sliding toward the start, a blind direction); the 1 kHz byte copy (mvIn) and the
  stillWait test (disclosed, a few cycles, no stack); the flaky DaemonFastMoveTest timing test (pre-existing, cbccec5).

### 34.5 The person rule at 0.5 m (information, the runbook)

The drone's front stop, near brake and image-rate cap all use the scene distance (the wall behind the person), not the
person's. person_gap_low models the k overshoot and the speed lead, not the soft hold's lean overshoot (0.3-0.7 m at
0.5-1.2 deg, T6 and testDriftLean). The runbook (bench X): approach a person only after a hover trim check, in two
steps, the last <= 0.5 m, the person seated.

### 34.6 Settings

| key | range | default | before the review |
|---|---|---|---|
| mvfloor | 20..100 cm | 30 | 30 |
| mvfstop | 15..80 cm (<= mvfloor - 5) | 25 | 25 |
| mvprogm | 10..60 cm | 20 | 20 |
| mvprogr | 10..100 % | 40 | 40 |
| mvlatm | 15..100 cm | 40 | 40 |
| mvnear | 0..150 cm (0 off) | **80** | 60 |
| mvfix | 0..63 bits: 1 DRIFT, 2 LONG, 4 STILL, 8 GAIN, 16 BRAKE, 32 FRONT | **59** | 27 (0..31) |

All u8, refused while autonomy is active, latched at the next ARM. Without a flash: `auto set mvfix 27` (no FRONT),
`auto set mvnear 60`, `auto set mvfix 0` with the V values (the motions before 2026-10-17; the identity hash holds).

### 34.7 Tests

- C (run_ctests): every suite passes but the known test_gyro_tempco (33) and test_mahony_replay (4). test_auto_motion
  515 (FRONT on a sideways / back-diagonal move, not on a vertical one, with and without the bit; the sideways near brake
  at 100 %; a forward LONG keeping its target and the next D following it back; a sideways LONG re-anchoring; STILL's
  cap through a flicker), the identity hash 0x47E10B48658F1BE0; test_auto_glue_core 206 (mvnear 80, mvfix 59, 0..63);
  test_auto_fsm 1991, test_autonomy_glue_host 501, test_camera_hold 198, test_speed300_replay 11, test_motion_replay 9.
- Sim: 216 / 200 / 200 at kmax 2.5 / 1 / 0 (testSideNearScene new).
- The reviewer's harness (its sim copy with the real auto_motion.c / auto_fsm.c): T1-T7 as in 34.1-34.3; T2 (an over-read
  D up to 30 %) 1 touch at 0.05 m/s at k 0.5 only, as before; T4 (a kept lean on the largest forward move) no contact up
  to 0.8 deg, 1-3 per 54 at 1.2 deg (0.32-0.37 m/s, as before the review).
- Python 342 (339 + set-height's `previous`, the drift field without DRIFT, a LONG toward the scene's note).

### 34.8 Cost and what stays open

Against 70444c8, the same sdkconfig, out of tree: static IRAM 130922 -> **130914 B** (-8: auto_fsm.c -12, the hush snap
now out of line in flash; auto_motion.c +4, STILL's 16-bit cap); `_iram_text_end` 0x40394000 as before; `_heap_start`
0x3fcc1500 unchanged (internal heap as 70444c8, 64 B under 4dea542). libcrazyflie flash .text +78 B, .rodata +4 B. Stack
frames: hushSnapEnd 96 B (contactStep 32 -> it, as contactStart's 96 B path), nearScene 48, settleEnd 64, autoFsmStep
176 and autoMotionStep 192 unchanged. Against 4dea542: IRAM +28 B, heap -64 B.

Open: the near brake in the settle (34.1; IRAM); the soft hold's lean offset (33.11), now the largest error near people;
the 1.2 deg lean case on the largest forward move (T4); flights per bench W and X.

## 35. Real 300 %: the fast steps' acceleration and settle as settings (user request 2026-10-17, evening)

The user, after the 300 % forensics (33) and its review (34): "no, I want real 300%". Section 33 lowered two constants
above 100 %: the acceleration cap AUTO_MV_FAST_A_H_CAP 0.30 -> 0.20 m/s^2 and the settle floor
AUTO_MV_FAST_SETTLE_MIN_MS 1000 -> 2500 ms, so 250 and 300 % planned as 200 %. Both are now NVS settings with those two
values as defaults; every other fix of d3bedee (DRIFT, LONG, GAIN, BRAKE, FRONT, the margins) is kept as it is.

### 35.1 Two settings

| key | type | range | default | what (above mvspeed 100 only) |
|---|---|---|---|---|
| mvacc | u8 cm/s^2 | 10..40 | 20 (d3bedee's 0.20 m/s^2) | A_H = min(0.10 s, mvacc / 100): the profile's speed-up and slowing, so their feedforward leans (0.30 m/s^2 is 1.75 deg), and the expansion cap's closed form |
| mvsettle | u16 ms | 1000..3000 | 2500 (d3bedee's) | the settle's floor: max(3000 / s, mvsettle) |

`auto_glue_core.c` AUTO_SET_MVACC, AUTO_SET_MVSETTLE (after `mvfix`, last). Read at the next ARM with mvspeed
(`autonomy.c` sortieSpeedLatch: `autoMotionSetFastCaps` before `autoFsmSetMvSpeed`, which copies the step's settle into
the motions), refused while autonomy is active, out of range refused loudly: `auto set mvacc 41` is `ERR auto set: mvacc
must be 10..40 cm/s^2`, a stored out-of-range or mistyped value is the default with the boot's warning `setting mvacc:
stored 41 out of range, default 20` (as cvqlate, mvfix); `autoMotionSetFastCaps` clamps too. At 100 neither is read. A_H
x s is at most 0.30 at 300 %, so mvacc 31..40 plan as 30 (the range is the one asked for; only a larger mvspeed range
would reach it). The latched pair lives in `auto_motion.c` (8 B of .data, as the margins; autoMotion_t is unchanged);
only the once-per-command code (`autoMvSpeedCfg`, flash) and the ARM's `autoMotionSetSpeed` read it.

Reported: `auto get` (`... mvfix=59(default) mvacc=20(default) mvsettle=2500(default)`), `auto status` (`... mvSpeed=300
mvAcc=30 mvSettle=1000`: as mvSpeed the NVS values, which in a sortie are the latched ones), the `auto set` usage list,
and the Mac's status and replies above mvspeed 100 (`mvacc_cm_s2`, `mvsettle_ms`, `fast_caps_src`: AUTO_STATUS does not
carry them, so the Mac shows what the last `auto get` read; `espfly_diag.py auto set` re-reads `auto get` after a set).

**Real 300 %** is the pair of 4dea542 (the 300 % flights of 2026-10-17 morning):

```
python3 tools/espfly_diag.py auto set mvspeed 300
python3 tools/espfly_diag.py auto set mvacc 30
python3 tools/espfly_diag.py auto set mvsettle 1000
python3 tools/espfly_diag.py auto get        # ... mvspeed=300(nvs) ... mvacc=30(nvs) mvsettle=1000(nvs)
```

Back to d3bedee's: `auto reset mvacc`, `auto reset mvsettle` (or `auto set mvacc 20`, `auto set mvsettle 2500`). The
pair applies at every step above 100, as 4dea542's caps did:

| mvspeed | A_H m/s^2 (default / real) | settle s (default / real) | 1 m forward at D 3 | 0.6 m sideways at D 2.5 |
|---|---|---|---|---|
| 150 | 0.15 / 0.15 | 2.5 / 2.0 | 5.33 s | 4.00 s |
| 200 | 0.20 / 0.20 | 2.5 / 1.5 | 4.50 s | 3.46 s |
| 250 | 0.20 / 0.25 | 2.5 / 1.2 | 4.50 / 4.12 s | 3.46 / 3.11 s |
| 300 | 0.20 / 0.30 | 2.5 / 1.0 | 4.50 / 3.89 s | 3.46 / 2.88 s |

Near a scene the expansion's peak rule binds, and the gain is small: forward 1.00 m at D 2.00 runs 6.40 s at the default
and 6.20 s at real 300 (DRY through the glue, test_autonomy_glue_host fast-caps). A 1 m step with the early return is
~3.9 s of profile, and its settle is 1.0 s instead of 2.5 s.

### 35.2 The Mac (tools/espfly_auto.py)

`mv_limits(pct, caps)`, `settle_s`, `move_profile(_s)`, the person rule (`person_speed_term`, `person_gap_low`,
`person_fwd_max`) and `move_timeout` take the drone's pair (`fast_caps()`: the cached `auto get` values, each in its
range, else the default; an older drone has none: d3bedee's). The daemon uses it for the profile, the battery fit (the
profile + the settle), the person rule's speed term, the move timeout, the early reply's "settles at most" and the wait
behind an early-returned motion. With no caps (or the defaults) every function is the Mac of d3bedee bit for bit.
`espfly_diag.py`'s help shows `auto set mvacc 30` and `auto set mvsettle 1000`.

### 35.3 Tests

- test_auto_motion (535; 515 before): testFastCapsIdentity: the identity script (auto_motion_identity.h) at 100 / 150 /
  200 / 250 / 300 with the default margins and mvfix hashes to the values taken on d3bedee's auto_motion.c
  (0x663B4B9687241D7F ... 0x7B6490612920859B), with the pair as initialised and set to the defaults; with mvacc 30 /
  mvsettle 1000 to d3bedee's motions with 4dea542's caps (d3bedee built with its host-only `autoMvFastCapsOld`, which
  this replaces), bit for bit. testFastCaps: 4dea542's limits table at 30 / 1000 (every other limit as at the defaults),
  100 never reads them, the clamps, every one of the 516 profiles of 4dea542's golden
  (`tools/tests/mvspeed_golden_4dea542.json`, its 86 rows at 300 % the flights') reproduced within 1e-5 (the defaults do
  not reproduce its 250 / 300 rows), the design table's 250 / 300 times, the settle timing out at 1.0 s at 300 (2.5 s at
  the default), a running motion keeping its latched settle.
- test_auto_glue_core (210; 206): mvacc u8 10..40 default 20 (= AUTO_MV_FAST_A_H_CAP x 100), mvsettle u16 1000..3000
  default 2500, appended after mvfix; the defaults latched give A_H exactly 0.20f and the settle 2500 at 300.
- test_autonomy_glue_host (539; 501): fast-caps (out of range refused, nothing stored; latched at ARM, not before;
  refused while active; the 300 % RUN 6.40 -> 6.20 s; `auto reset` gives the defaults at the next ARM), fast-caps-stored
  (a stored 41 / 999 refused at boot, the defaults latched).
- test_speed300_replay (11): the flights' caps now come from `autoMotionSetFastCaps(30, 1000)` (the hook is gone); the
  replay reproduces the logs as before.
- Python (347: 342 + 5): FastCapsMirrorTest (the constants and the table against the firmware's sources; the defaults
  the mirror of before, with the d3bedee golden; real 300 = 4dea542's limits and every row of its golden within 1e-4;
  `fast_caps()` from the cache, out of range the default; the status fields only above 100), DaemonFastMoveTest
  test_real_300_battery_fit_and_status (a voltage the defaults would refuse fits with the 1.0 s settle; the note
  "settles at most 1 s"; the status fields). test_espfly_diag's console replies carry the new keys.
- Sim (information: `test_interactive_sim 2.5 real300` runs testFastMoves, testFrontBraking, testSideNearScene and
  testDriftLean with the pair latched at every ARM; their 43 checks, calibrated at the defaults, all pass): no new
  contact with the default margins and mvfix; the sideways case at a 0.20 m/s^2 push toward the scene keeps its one
  touch (0.19 m/s against 0.14); far-scene moves lean up to 2.9 deg (2.2), the hold leads by up to 1.31 (1.19); a STOP 2
  s into a 2 m approach travels 0.01 m after it, the frames stopping 0.44 m (bound 0.61); the lean cases (48 runs, a 0.8
  deg lean in the move) end at most 0.75 m off (0.89), DONE more than 0.20 m off 6 of 48 (4): shorter moves give the
  lean less time, and a shorter settle catches less of the end. All other C suites as at d3bedee (the known
  test_gyro_tempco 33 and test_mahony_replay 4 failures); test_interactive_sim 200 at the defaults.

### 35.4 Cost

Against d3bedee, the same sdkconfig, out of tree: static IRAM 130914 -> **130918 B** (+4: autoMotionStep's assembly, the
same instructions with one narrow move widened and the branch-target padding moved; its source is unchanged);
`_iram_text_end` 0x40394000 as before. `_heap_start` 0x3fcc1500 -> 0x3fcc1508: the internal heap **8 B smaller** (~18.54
KB with the Mac, guard 12288): .data +8 (the latched pair), .bss +8 (two settings). Flash .text +172 B, .rodata +120 B.
The 1 kHz path: nothing new (autoMvSpeedCfg runs once per command; the step reads the settle it had). Stack frames
(-fstack-usage): cmdAutoGet 704 -> 752 B (the line buffer AUTO_SET_COUNT x 28), cmdAutoStatus 896 -> 944 B, both under
cmdFlights' 1104 and cmdCamVz's 1088 (the console's deepest), so `diagStackFree` keeps its margin.

### 35.5 Risks

- Real 300 % is what flew on 2026-10-17 morning for the profile and the settle: the speed-up and brake leans of 0.30
  m/s^2 ran ahead of a hold that did not follow them (33). BRAKE (the braking lean scaled by the measured speed) and
  GAIN now act on it; DRIFT and LONG report what the 1.0 s settle cuts short. Expect more SHORT / LONG ends and 0.1-0.4
  m of slide after an early reply; `wait-move` gives the final end.
- mvsettle 1000 also shortens 150 % (2.0 s) and 200 % (1.5 s), as 4dea542 did.
- Not flashed, not flown. The Mac tool must come from the same commit.

## 36. Accelerometer temperature compensation and the ground cooling spin (user request 2026-10-18 ~00:15: "do 3 and 2")

Built on 19fe168 (a2b-int), host-tested, **not flashed, not flown**. Bench: docs/autonomy-bench.md Z.

The 15:26 table flight of 2026-10-17 took off right after a 2.5 h USB charge with the IMU at 47.5 C; it cooled to
31.0 C in the air. The camera height observer read 2.46 m while the controller held 1.47 m, then the hover sank:
the accelerometer's zero had moved after the idle spin's g0. The gyro already has a temperature compensation
(19.1); this section gives the accelerometer the same one, a check that measures its coefficient on the drone,
and a way to cool the IMU on the ground before a take-off instead of flying the drift.

### 36.1 The fit (scratchpad `atc/atcfit.py`, the recorder logs of 2026-10-14..17)

Two independent estimates of k_z (mg per C of die temperature), both poor:

1. **The observer's offset state** (`cvBias_cmg`, the world-up accelerometer offset against 1 g the camera
   observer learns) against `imuTemp_cC` within each flight, from lift-off + 10 s with healthy frames, 2 s block
   means, one slope per flight: 33 flights with >= 3 C of cooling gave **-0.67..+0.67 mg/C, mean +0.12, sd 0.41,
   median +0.20** (sem 0.07 if the flights were independent draws of one k, which the spread says they are not:
   the state also tracks ground effect, the camera's scale and the hold). The 2026-10-17 flights: speed300 +0.59,
   speed300b#7 +0.40, table#8 -0.04, table2#1 +0.36, table2#2 (the 15:26 flight) -0.65, real300 -0.21 / +0.38,
   night +0.02. The `flights` summaries' b@3s -> b@td pairs give the same picture (+0.65, +0.28, +0.65, +1.0,
   +0.46, +0.13, -0.10 for boots 2-5 of 10-17; the touchdown moves the state).
2. **The still ground rows** before the first thrust (warm) and after the last thrust (cold, >= 1 s after the
   motors stopped), motors off, 1 s means, for the flights that rest on the same spot without an attitude change
   (|d roll|, |d pitch| < 0.5 deg; 27 of 31 pairs are contaminated by a tipped or caught landing, x / y steps of
   tens to hundreds of mg): **p1-height#4 +0.05 (50.4 -> 34.7 C, z -0.8 mg), table#8 -0.09 (45.4 -> 33.2,
   +1.1 mg), p2-height#3 -0.36 (47.7 -> 30.2, +6.3 mg), night#1 -0.04 (49.2 -> 33.8, +0.6 mg; roll moved 0.53
   deg)**: mean +0.07, sd 0.45 mg/C. The still reading's own scatter between the rows before and after is 1-6 mg,
   so a 15 C cooling changes the resting z by less than ~6 mg: |k_z| below ~0.4 mg/C, sign unknown. The warming
   ground-only stretches (33-57 s at 44-48 C, dT 0.6-0.9 C) gave kz -1.2 +/- 0.1 mg/C on z, the opposite sign
   over a span too short to trust (a 0.9 C change, 1 mg).

Neither method pins k_z to a factor of 2, so the compensation ships **off** with k_z +0.20 mg/C compiled (the
observer's median; enabling it corrects 3.3 mg over the 15:26 flight's 16.7 C; a wrong sign would add 3.3 mg,
within the +-20 mg take-off envelope of autonomy-a2.md 4.9 and far below the 15 mg clamp) and x / y at 0: the
clean pairs put |k_x|, |k_y| below ~0.5 mg/C too (p1-height#4 -0.25 / -0.12, table#8 +0.34 / +0.18, p2-height#3
+0.40 / -0.53), inconsistent in sign. The steady +1.2 deg pitch hold lean of 10-17 (21 mg if it were the
accelerometer) is not in the ground rows (the x step over 12-16 C of cooling is -4..+4 mg on the clean pairs):
the lean is not the accelerometer's heat drift, or not mainly. What does pin k: the check below (every flight
measures step / dTeff; average several) and the cooling spin (a bench sweep of 5-10 C at rest).

Note the vertical channel's sensitivity: a 0.2 mg bias for 30 s is 1 m of height (h = a t^2 / 2). The observer
corrects it with the camera; the controller's held offset lags. Even a well-fitted k leaves the first 10 s after a
hot take-off to the camera; the cooling spin attacks the cause.

### 36.2 The compensation (`utils/src/accel_tempco.c`, `sensors_mpu6050_hm5883L_ms5611.c`)

**Model.** `corr[i] = clamp(k[i] x (min(T, knee) - min(Tref, knee)), +-15 mg)`, T the die temperature low-passed
with tau 0.5 s on the gyro compensation's 50-sample block (20 Hz, 19.11), Tref latched where the accelerometer's
reference is taken: at ARM (autonomy.c, the drone rests with the motors off, ~1 s before the idle spin takes g0),
at a cooling spin's start, at `atc latch`, and once at the startup bias. Defaults: kx 0, ky 0, kz +0.20 mg/C,
knee 60 C (none), clamp 15 mg, **off**. The sign: a zero that follows b_z = k_z (T - Tref) reads below its reference
after cooling when k_z > 0; the controller reads that as sinking and climbs. The corrected reading is b - corr.

**Driver** (flash, the sensors task): the per-sample path adds the three raw samples (mapped body frame) to the
block's sums and subtracts the cached correction (g) from the scaled accelerometer before the alignment and the
low-pass, so `sensfusion6GetAccZ`, vert_ctrl, the observer, the attitude filter and the recorder's `az_mg` all see
it (+0.0 when off: bit-identical). The block step (shared with the gyro's, under its lock) pushes the block's raw
means into a 1 s window (20 block means, mg), runs the model, and serves a pending latch: the window's mean becomes
the reference reading (if the drone rested the whole second: `refValid`), Tref the temperature now, the correction
restarts at 0. **The check**: at every accepted gyro re-zero the window's mean minus the reference is the step a
still accelerometer took since the latch, against the model at that moment (`r = step - corr` what the
compensation left); entries after thrust (flight=1, air=1 above lift-off) or a cooling spin (cool=1) go to a
4-entry ring, the first after an airborne stretch also to its own slot (`A`). A sortie's g0 is taken in the idle
spin with the motors running, so `r` of a flight is the offset the controller flew with up to the spin's own
vibration offset (a few mg, the same every flight). A changed resting attitude moves x / y by 17 mg per degree and
z by (1 - cos): the reader flags x / y steps over 20 mg (TILT). The bisect toggle `bypass gtc` covers this path
too (the same block).

**USB** (`atc`; set / on / off / save / clear with the motors stopped and autonomy idle, live until `atc save`):
`atc show` prints the `A` line (seq, ageS, landS, Tref, T, dTeff, step, corr, r in mg, on), one `Z` line per logged
entry (flight / air / cool / on), then `OK atc` (config, src, T, Tlp, Tref, refSet / refValid, latchAgeMs, pending,
dTeff, corr and model now, the reference and the current 1 s readings, counts). `atc set <kx> <ky> <kz> [<knee>]`
(|k| <= 2 mg/C, knee 30-60), `atc on|off`, `atc save` (NVS `acctc` in `imucal`, magic "ACTC"), `atc clear`, `atc
latch` (bench: the reference now). `status`: `atc=0/1 atcZ=<mg subtracted now>`. `python3 tools/espfly_diag.py atc`
prints the post-flight check under the lines: the still z step, r_z, PASS (|r_z| < 3 mg) / FAIL, TILT, the
measured k_z = step_z / dTeff, and the refinement `atc set 0.00 0.00 <k_z> 60` when |dTeff| >= 5 C and |r_z| > 3 mg.

**Recorder and summary.** The record grows 124 -> 126 B (`atcCorrZ_cmg`, the z correction subtracted, 0.01 mg;
18000 x 126 = 2.27 MB of PSRAM, the ring's capacity unchanged); `espfly_diag.py` reads 72-field dumps of older
firmware with the column empty. `flightSummary_t` 164 -> 172 B (`atcStepZCmg`, `atcCorrZCmg`, `atcResDtCc`,
`atcResFlags`; FLIGHT_RTC_MAGIC 0xF1A6C0EA: the stored flights are cleared by the flash), the `flights` line
`atcRf= atcS= atcC= atcRdT= cool=`, the Mac's second table a column `acc z step/r mg`.

**Enabling it** (after the check has measured k_z on 3+ flights with |dTeff| >= 10 C and the same sign): `atc set
0 0 <mean k_z> 60`, `atc on`, `atc save`; or `atc on` alone for the compiled +0.20. `atc off` + `atc save` undoes
it without a flash. The correction only acts between a latch and the next; before the first latch it is 0.

### 36.3 The ground cooling spin (`utils/src/ground_cool.c`, `modules/src/ground_cool_glue.c`, `stabilizer.c`)

`cool [<s> [<pwm>]]` (USB; default 20 s, max 40; PWM default 12000 = 30 % of the ~40000 hover, 1000..20000, the idle
spin's cap: lift-off needs >= 30000) runs all four motors at one equal PWM from IDLE. The stabilizer replaces the
setpoint by the spin's PWM with a level attitude and zeroes the controller's attitude outputs after it, so
powerDistribution gives each motor exactly the PWM; the restart lock, `gcap`, the soft start, the driver's STOP
latch and the kill all stay in front. Guards at the start: autonomy IDLE, every motor duty 0 and no thrust
requested, on the ground and resting (gyro < 10 dps, |a| within 0.15 g, tilt < 8 deg), the motor driver ready (no
STOP latch, no restart lock), the resting battery >= `vmin`. Aborts at once: tilt > 8 deg, any gyro > 30 dps, |a|
off 1 g by > 0.15 (after the first 300 ms of spin-up), a kill (the link's `k` ends it through autonomyKillFromLink;
`cool stop` on USB), autonomy leaving IDLE (an ARM; it is then refused anyway, the motors spin), the driver no
longer ready. `cool` alone prints the state (`run` or the latest end DONE / STOP / KILL / TILT / GYRO / ACC / AUTO /
LOCKED), the PWM and seconds, the time left, the IMU temperature at the start and now, the tilt and gyro maxima.
The accelerometer compensation's reference is latched at the start, so the first ground re-zero ~2-3 s after the
spin logs the still accelerometer's step over the cooling (`atc show`, the `Z` line with cool=1: k = step / dTeff on
each axis), and the gyro's entry carries cool=1 too. The spin is a recorded "flight" with thrust <= 20000 and no
air time, labelled COOL in `flights` (FSUM_MOTOR_COOL), and it does not update the heat-soak motor time
(`stabilizerLastFlightMotorMs`): the STILL guard's base 20 s at zero thrust still applies after it (DRY / BENCH 3
s), so the sequence is cool, 20 s of rest, ARM. The 1 kHz loop pays one load and a branch when no spin runs.

`python3 tools/espfly_diag.py cool [S [PWM]]` starts it, prints the state line every 2 s, and after the end waits
4 s and prints the temperatures before / after with the accelerometer entry and the k it implies. **Only USB**: the
Mac link has no COOL message (its daemon's command path, the status layout and the golden protocol tests would all
change for one ground command); run it over USB before unplugging, or let the IMU cool with the motors off.

Expected: the prop wash at 12000 cools the IMU a few C in 20 s (the flights cool 0.3-0.6 C/s at hover thrust; the
spin moves less air). If 20 s at 12000 gives less than 2 C, try `cool 40 16000` with the props clear and a hand on
`k`. The spin runs the motors on a battery about to fly: a 20 s spin at 30 % of hover costs roughly 1-2 s of air.

### 36.4 Verification off the drone

- `test_accel_tempco` (657): the defaults, the sign and magnitude (47.7 -> 31.0 C: z -3.34 mg on, exactly +0.0
  off), the knee, the clamp, the latch and low-pass, NaN, the block step bit for bit against the per-sample one and
  50-sample blocks within 0.01 mg on a 0.6 C/s cooling, the window, the log, the NVS record; the replay of the check
  on the two clean ground pairs (p1-height#4: step z -0.77 mg over -15.7 C, r +2.37 with the default model, k_z
  measured +0.049; table#8: +1.08 over -12.2 C, r +3.52, -0.088): both within 10 mg.
- `test_ground_cool` (64): every start guard, a full 20 s run (the PWM for exactly 20000 loops, DONE, the
  temperatures and maxima), the clock wrap, every abort (tilt, gyro, acc after the spin-up and not during it,
  autonomy, lock, the kill counter, a NaN tilt, stop and kill requests, a stale kill request cleared by the start).
- `test_flight_recorder_host` (10557): the 124 B prefix of every record hashes to the 39e80e2 pins unchanged; the
  126 B record pinned. `test_flight_summary` (50): 172 B. `test_autonomy_glue_host` (539) with the latch and kill
  stubs. The rest of the C suite as at 19fe168 (the known test_gyro_tempco 33 and test_mahony_replay 4 failures).
- Python 357 (347 + 10): AccTcTest (the console prints A and Z before OK, the check's PASS / FAIL / TILT / k_z /
  refine, the flights line), GroundCoolTest (a run to DONE with the entry, a refusal, an abort, stop and status),
  the 73-column record and 72-column dumps of older firmware.

### 36.5 Cost

Against 19fe168, the same sdkconfig: `.iram0.text` 129891 -> 129903 B (+12; `.iram0.text_end` -12: the padding),
`.dram0.data` +32 B, `.dram0.bss` 88496 -> 88960 B (+464: the 1 s window of block means 240 B, the log, the spin's
state), `_heap_start` 0x3fcc1508 -> 0x3fcc16f8: the internal heap **496 B smaller** (~19.2 KB expected after a reset,
~18.0 KB with the Mac linked; bench L0's hard no-go is 19 KB without the Mac). `.rtc_noinit` 7532 -> 7660 B (+128,
the summary's 8 B x 16 flights; 8 KB RTC). Flash `.text` +8.6 KB, `.rodata` +1.8 KB (the two consoles), the bin
0x113bf0 -> 0x115a50 (26 % of the partition free). The 1 kHz path: the sensors task adds three int32 adds and three
float subtractions per sample (the gyro compensation's pattern), the stabilizer one load and a branch; no new
float-heavy work per sample. If the heap matters, the window's block means can shrink to int16 (120 B).

### 36.6 Risks

- The coefficient is not known: `atc` is off, and `atc on` applies a +0.20 mg/C guess whose sign the logs do not
  establish. Wrong, it adds up to 3-4 mg over a 16 C cooling to a controller that already sinks late; the clamp
  bounds it at 15 mg and `atc off` is immediate.
- The check's reference comes from the 1 s before ARM; a drone picked up and put down between ARM and the landing
  spot changes the attitude and the x / y (flagged) and, slightly, z. Compare flights that start and end on the same
  floor.
- The cooling spin runs the motors on the ground with the props on: props clear, a hand on `k`, the drone on a flat
  floor (a tilted or sliding drone aborts TILT / GYRO / ACC). Its PWM is far below lift-off, but a 20000 spin on a
  full battery is 50 % of hover: start at the default 12000.
- The spin resets the STILL clock (20 s) and wakes nothing else; the camera is untouched.
- The RTC summary store is cleared by the flash (its layout changed): `flights` from before the flash are lost;
  read them first.

## 37. The healthy offset speed-up "hb": the early climb (the height estimator study of 2026-10-18; built for host testing, default off)

`firmware-backups/handoff-2026-10-12/height-estimator-2026-10-18.md`. The study found why a hover climbs early and
sinks late: the accelerometer's vertical offset FALLS 4-8 mg in the first 15-40 s of every logged flight (the die
cooling after a hot start) and rises +0.15-0.25 mg/s afterwards; the observer's offset state follows at `qBiasG` with
a ~10 s time constant and lags it by 2-4.5 mg, and the HOLD law turns every mg of lag into 0.08-0.15 m of real height
change. Replaying the measured offsets through the real observer reproduces the 15:26 flight's +0.85 m uncommanded
climb from the lag alone. R1 (section 25) starts at 40 s and holds a 2-4.5 s old offset: it removes 8-17 % of the
climb at best. The study's pick, built here: a faster offset random walk while the camera is healthy, from the first
frame after lift-off, x3 first.

### 37.1 The law (`cam_vz_obs.h` "Healthy offset speed-up")

- On every 10 ms block in which the observer is **healthy in the air** (a tracked G, no CUSUM alarm, an update
  accepted within `healthyWindowUs` 2 s, not on the ground) the offset's random walk is `qBiasG x hbMult`; on every
  other block `qBiasG`, today's filter: before the first accepted frame, in a camera gap from 2 s after the last
  accepted frame, during a CUSUM alarm's hold, on the ground (a bounce too: off until the first accepted frame after
  the re-lift).
- Nothing else changes: the controller takes `biasG` as today (no held offset, no variance bound, no guard events,
  no step time). No landing, guard, SINK or K_RANGE logic is touched.
- The flag the block path reads (`hbOn`: a tracked G, no alarm, an update ever, no fault) is set at each frame the
  filter takes (flash); the block (IRAM) adds the ground and the 2 s window and counts the time on (`hbUs`). With
  `hbMult` 0 the block path tests one flag and the filter is today's bit for bit (the replay's pinned hash, 37.4).
- Refused with R1 (`qLateMult` > 1) or T2 (`t2kR` > 0): untested together (`camVzObsConfigValid`; the glue keeps R1
  / T2 as stored and leaves the speed-up off, `hbCfg=bad`).

### 37.2 Settings and reports

- NVS `cvhb` (u8, `hbMult` x 10: 0 = off, the default; 10..80; 1..9 refused at `auto set` and at boot), `auto_glue_core.h
  AUTO_SET_CVHB`, appended after `mvsettle`: `python3 tools/espfly_diag.py auto set cvhb 30` (x3, the study's first
  step), `auto set cvhb 0` or `auto reset cvhb` (off). **Read at boot** (cam_vz_glue.c `camVzGlueInit`): a change
  takes effect after a reboot; `auto get` shows the stored value, `camvz` the one running.
- **Refused loudly**, as `cvqlate`: a stored value out of range, 1..9, unreadable, or with `cvqlate` > 1 or `cvt2k` > 0:
  the boot log's warning, `camvz` `hbCfg=bad`, the flight summary's `FSUM_CVL_HB_CFG_BAD`, the diag tool's "STORED
  SETTING REFUSED AT BOOT", and the speed-up off. The rest of the config is never dropped for it.
- **USB `camvz`**: `hb=<x> hbCfg=ok|bad hbOn=0|1 hbOnS=<s on this sortie>` (the diag tool prints a line from them:
  "healthy offset speed-up x3.0 (on now), on 57.1 s this sortie").
- **The RTC flight summary**: two flag bits in `fsumCvLate_t.flags` (`FSUM_CVL_HB_ON` 0x20: `hbMult` > 0 in use,
  `FSUM_CVL_HB_CFG_BAD` 0x40), no new field: the store's layout is unchanged (USB `flights` shows them in `cvqf=`).
- The recorder's `cvFlags` has no spare bit (bits 0-3 airborne / healthy / gain / fault, 4-7 the frame decision):
  the state is in `camvz` and the summary only. Not in AUTO_STATUS (all 136 bytes assigned).

### 37.3 The firmware

`cam_vz_obs.h/.c` (the config's `hbMult`, appended; `camVzObsOut_t.hbOn`; the state's `hbOn`, `hbUs`, appended:
2192 -> 2208 B, PSRAM; `predict()` takes the flag, `closeBlock()` decides the block, `hbPost()` the frame's flag);
`cam_vz_glue.h/.c` (the NVS read and its checks, `camVzGlueHb` for USB); `cam_vz_shadow.h/.c` (`hbCfgBad`, the two
summary bits); `flight_summary.h`; `auto_glue_core.h/.c` (`cvhb`; `autoSettingInRange` refuses 1..9);
`usb_diag.c`; `tools/espfly_diag.py` (the `camvz` line).

### 37.4 Verification (host)

- `test_cam_vz_obs` (3340 checks, 3211 before): the FINAL replay's pinned hash (every row of the 43 logged take-offs
  bit for bit the observer of 65dfcba) stands with `cvhb` 0; a twin with x3 differs in 0 of 25218 rows before its
  first accepted frame and in 54236 of 62656 after it. A 60 s synthetic hover whose offset falls 8 mg over 5-25 s
  (the 15:26 fall): the offset lag over 8-40 s off 2.09 mg mean (max 3.33), x3 0.91 mg (max 1.76); nothing before the
  first accepted frame with a tracked G (2.87 s; the first difference 2.88 s); on in 100 % of the healthy airborne
  samples. A 6 s camera gap: on for 2 s after the last accepted frame (the offset's variance growing x9.00), then off
  (x1.000, not counted) until the first accepted frame after the gap; a CUSUM alarm: off for its hold; the ground
  mid-flight: off, and after the re-lift off until the first accepted frame; the config limits (0 or 1..8, refused
  with R1 or T2; an invalid value leaves the default off).
- `test_auto_glue_core` (213, 211 before): `cvhb` u8 0..80, 1..9 and 81 refused, appended last; every stored value
  in range is a valid observer config alone, not with `cvqlate` > 1 or `cvt2k` > 0. `test_cam_vz_glue_host` (96, 74
  before): NVS absent / 0 / 30 / 10 / 80 / 50 taken, 5 / 81 / with `cvqlate` 5 / with `cvt2k` 50 / unreadable refused
  (the others as stored), shown by `camvz` and the summary bits.
- The rest of the C suite and the python tests as at 7763729 (the known `test_gyro_tempco` 33 and
  `test_mahony_replay` 4 failures); `test_cam_vz_late`'s x1 closed-loop hash and `test_cam_vz_t2`'s unchanged (the
  speed-up off is bit for bit).

### 37.5 Replays (the study's harness, `energy-2026-10-17/height-estimator/harness`, rebuilt against this observer)

The study's closed-loop replay (sink-replay's plant, each flight's real offset, accelerometer noise and every frame's
real camera error, HOLD on the camera's speed, no SET_HEIGHT) over its 16 truth flights, with the FIRMWARE's `cvhb`
30 against today and the study's harness-side option (a) `qs=3`:

| | worst lo | worst hi (15:26) | mean climb | mean late | 5 s gap, no drift (G4) | 10 s gap |
|---|---|---|---|---|---|---|
| today | -1.18 | +1.13 | +0.47 | -2.45 cm/s | -0.44 | -1.13 |
| the study's (a) x3 (`qs=3 qsig=0.04`) | -0.88 | +0.66 | +0.23 | -1.17 | -0.71 | -1.92 |
| **this build, `cvhb 30`** | **-0.93** | **+0.58** | **+0.19** | **-1.15** | **-0.75** | **-2.06** |
| this build, `cvhb 50` | -0.91 | +0.42 | +0.11 | -0.82 | | |

Against the study's numbers the build is within 0.05 m on the mean climb (+0.19 vs +0.23), the worst low point (-0.93
vs -0.88), the late sink and the 5 s gap cost (-0.75 vs -0.71), and **0.08 m stronger on the 15:26 flight (+0.58 vs
+0.66)**, 0.14 m worse on the 10 s gap (-2.06 vs -1.92). The difference is where the speed-up starts: the study's
harness switched its noise on at the simulated hover's start (HOVER + 3 s), the firmware at the first accepted frame
with a tracked G (~3 s after lift-off), so the offset's first fall is caught 3-6 s earlier. Switching the firmware's
on at the hover's start instead (a harness option, `cvhbT=1`) gives the study's table back to 0.01 m (+0.66 / +0.23 /
-0.89 / -1.15); the study's sigma_v <= 40 mm/s gate changes nothing on these flights (`cvhbsig=0.04`: identical). The
5 s gap stress on 9 flights: worst lo -0.54 (today -0.30), worst hi +0.58 (+1.30); no K_RANGE event in any run.
Replay numbers carry about +-0.3 m (the study's caveats).

### 37.6 Cost

Against 7763729, the same sdkconfig (the hb hand-back in `firmware-backups/handoff-2026-10-12`): `.iram0.text`
129903 -> 129927 B (+24: the block close's flag test, the window compare and the time add; `.iram0.text_end` -24, the
padding: `_iram_text_end` unchanged), `.dram0.data` 0, `.dram0.bss` 88960 -> 88976 B (+16: the setting and the
`camvz` snapshot's fields), `_heap_start` 0x3fcc16f8 -> 0x3fcc1708: the internal heap **16 B smaller**.
`.rtc_noinit` 7660 B unchanged (two flag bits, no field: the summary store keeps its layout). Flash `.text` +876 B,
`.rodata` +336 B, the bin 0x1164e0 -> 0x1169b0 (26 % of the partition free). Per block with the speed-up off: one
byte load and branch (as R1 and T2); on: one compare of the accepted-update age, one multiply in `predict()`, one
add. Per frame (flash): four flag reads. Nothing in the 1 kHz path. `camVzObs_t` 2192 -> 2208 B (PSRAM). USB `camvz`
+~40 B, `auto get` +~16 B (the frames of 26.2 keep >= 1.6 KB).

### 37.7 Risks and what stays open

- The same risk every faster offset has (the study's F): a camera that is steadily wrong by -2 cm/s is followed a
  little further (+1.55 m against +1.44 m worst case, both too close to the 2 m rule from a 1.2 m hover), and a 5 s
  camera dropout with no drift sinks 0.75 m instead of 0.44 m on the worst logged flight (f1). The CAM_LOST rule caps
  thrust in a dropout, the floor rules and the photos catch a sink; a climb toward the ceiling has nothing on board to
  catch it, which is why x3 (not x5) and the photo cadence (bench Q) stay. x5 (`cvhb 50`) is the second step after two
  or three clean flights.
- It does not touch the cause: the ground cooling spin (36.3) attacks it; the first flights compare a hot start with
  one after `cool 20`.
- Not flown. The flight check (the study's G): 60-90 s hovers at 1.0 m, a photo every 10 s with the ceiling-line look,
  no upward set-height from a photo older than 5 s in the first 40 s; pass if the first-30 s climb stays under 0.3 m
  and the late sink under 1 cm/s.

## 38. The MOVE guards down to off (user decision 2026-10-18 ~13:30: "remove the guards or make them absolutely minimum")

The table approach of 2026-10-18 noon (the 46d7fe2 flights 12:50, 12:57 and 13:01, `cal/flight-2026-10-18-noon.csv`,
boot 4 flights 1 / 2 / 4): the drone approached the low table at 0.9 m and every long MOVE was stopped by the firmware
well short of its target: the reference reached forward 0.75 of 1.75 m, 0.85 of 1.8, 0.55 of 1.3 (the recorder's
mvState 17 RUN -> 21 STOPPED). The expansion-based front logic reads the near table top, expanding fast in the lower
half of the image, as the scene getting close, and the moves end FRONT or PROGRESS before the drone is over the
table. The camera odometry itself was not the problem. The user's decision: the drone is 28 g, a contact with the
furniture is accepted; the guards go, or to their minimum. So every MOVE distance guard is now adjustable down to OFF
through the NVS settings, no rebuild, the defaults the behaviour of before bit for bit (the identity hashes of
test_auto_motion stand: 7ab74b9's with the margins of before, d3bedee's at every speed step).

### 38.1 What changed

| key | range | default | what | 0 means |
|---|---|---|---|---|
| `mvklow` | 60..100 % | 60 (`AUTO_MV_K_LOW` x 100) | the lowest image expansion scale k the planning floor (TOO_CLOSE), the live front stop (FRONT), the near brake (mvnear) and the measured lower bound of D after a move allow for | (100: no safety factor, the measured D as it is: the true end of a forward move F from D is D - F, not D ((D - F) / D)^(1/k)) |
| `mvfloor` | **0** or 20..100 cm | 30 | the planning floor: a forward MOVE's true end at k mvklow must leave this to the scene; the TAKEOFF's plan + 5 cm | no floor: a MOVE and the plan may be planned up to D - 0.05 m (auto_motion.c D_FLOOR_M: the ln target needs D - F > 0); the sideways guard's 0.30 m distance floor at acceptance goes with it; a back move's contact anchor keeps nothing ahead |
| `mvfstop` | **0** or 15..80 cm | 25 | the live front stop: a MOVE is STOPPED FRONT when its measured forward progress says the scene is this close at k mvklow (every horizontal MOVE with mvfix FRONT) | off: progFwdMax 1e30 on every MOVE, never FRONT from it (the near brake is `mvnear`'s own, 0 off, as before) |
| `mvprogm` | **0** or 10..60 cm | 20 | the progress guard's base margin (and the SHORT end's off-the-line limit) | the progress guard off: the sideways and forward margins 1e30 (never PROGRESS from them in RUN or SETTLE), the frozen-view rule off (the step's frozenRef 1e30), no SHORT for an end off the line |
| `mvprogr` | **0** or 10..100 % | 40 | the progress guard's part of the move | the progress guard off, as mvprogm 0 (the off-the-line SHORT keeps mvprogm when that is set) |
| `mvlat` | 0..200 % | 50 (`AUTO_MV_LAT_D_REL` x 100) | a sideways MOVE at most this / 100 x D (TOO_WIDE: a depth step under a sideways move makes it D_far / D_near too long, 32.4) | no cap: the 1.50 m command limit is all that is left |
| `mvwmax` | 10..40 (x 0.01 /s) | 15 (`AUTO_MV_W_FWD_MAX` x 100) | the expansion rate cap: at 100 % a forward move's speed is at most this x its end distance (0.15 x 0.45 m = 0.07 m/s for a move ending 0.45 m from the scene: 1.75 m in 26 s); above 100 % min(this x s, max(0.17, this)) | (a cap over ~25 loses the view near the end of an approach: the hold's own expansion gate refuses frames over 0.3 /s smoothed; then NO_VIEW after 2 s, or the near brake) |

`mvfloor`, `mvfstop`, `mvprogm` and `mvprogr` take 0 as "off" and keep their ranges otherwise: 1..min - 1 is refused
(`autoSettingOnMin`, the console's `ERR auto set: mvfloor must be 0 (off) or 20..100 cm`; the same message form for
`cvhb`). `mvlatm` (the sideways margin's floor) has no off of its own: with the progress guard off it is unused;
`mvnear` 0 was already off (32). All seven are latched at the next ARM with the margins (autonomy.c
`sortieSpeedLatch`: `autoMvMarginsExtra` for mvklow and mvlat into `autoMvMargins_t`, `autoMotionSetFwdRate` for
mvwmax before the speed step as the fast caps), refused while autonomy is active. `auto status` ends
`mvGuards=floor/plan/stop/progm/progr/latm/near klow= lat= wfwd=` (the margins in force: in IDLE the last sortie's or
the defaults; 0 = off).

### 38.2 What each guard did, and what is left of it off

- **The planning floor (mvfloor, TOO_CLOSE)**: the drone refused a forward MOVE whose true end, at the lowest k and the
  lowest estimate of D, would leave less than 0.30 m to the scene (the TAKEOFF's plan 0.35 m). Off: the only limit is
  D - F >= 0.05 m (RANGE / NO_DIST as before), so "forward 1.9 at D 2.0" is accepted. The move then crawls: with the
  default mvwmax its speed is 0.15 x 0.06 m = 0.009 m/s (test_auto_motion testGuardsOff: 215 s for 1.94 m), which is
  what `mvwmax` is for. With the floor off and the live stop on, an ideally tracked long move ends FRONT at the stop
  (0.25 m at k 0.6): the two are separate.
- **The live front stop (mvfstop, FRONT)**: STOPPED when the measured progress put the scene at mvfstop at k mvklow.
  This is the guard that most likely ended the table moves (the table top's expansion reads as the scene at 0.25 m).
  Off: never FRONT from it; a wild forward reading is then the progress margin's, or nothing with that off too.
- **The progress guard (mvprogm, mvprogr; PROGRESS)**: STOPPED when the measured motion left the reference by more
  than the margin, sideways (in metres at the distance now, at least mvlatm) or forward, in RUN and SETTLE; with it
  the frozen-view rule (a reference past 0.10 ln with less than a quarter of it measured). Off: margins 1e30, no
  frozen-view rule. What remains of the settle: it still ends DONE within its tolerance, SHORT / LONG along the move,
  and STOP PROGRESS when it timed out with less than half of the move measured (`AUTO_MV_SETTLE_MIN_PROG`, a frozen
  view must not keep its target): these end a move that has stopped moving, at the settle's end; none stops one.
- **The expansion scale k (mvklow)**: every distance above was computed for a k down to 0.6 (a wall plus a plain
  floor in view reads k 0.73 at 3 m; a frontal wall ~1). At 100 the floor, the stop, the near brake and the measured
  lower bound of D use the measured distance as it is: the largest MOVE at D 2.0 m is 1.69 m instead of 1.35 m at the
  default floor, the stop trips at ln(D / 0.25) instead of 0.6 ln(D / 0.25).
- **The sideways cap (mvlat, TOO_WIDE)**: |lateral| <= 0.5 x D per MOVE. Off: the 1.50 m command limit only.
- **The expansion rate cap (mvwmax)**: not a stop but the speed limit that makes approaches near a scene slow (and the
  only one that binds for the table approach once the floor is off). It matters: 0.15 -> 0.30 halves the time of an
  approach ending near the scene (the 1.75 m move of the noon flights, D ~2.2 m: 26 s -> 13 s). Above ~0.25 the hold's
  own expansion gate (0.3 /s, smoothed EMA) refuses frames near the end: the move pauses, NO_VIEW after 2 s or the
  near brake FRONT (mvnear). It is the user's call; 30 is a sensible first step with the view gate in mind.

### 38.3 What still protects the drone with everything off

Untouched: the state machine's contact / tilt / rate / fall / attitude-error kills, its battery rules (BATT_LOW's LAND
and the MOVE refusal BATT, LOW_BATT, UP_HEADROOM), the camera-loss rule (CAM_LOST, its SINK), the hold's own limits
(HOLD_LIMIT), the vertical controller's floor and top (HEIGHT: 0.25 m + the drift term, 2.00 m), the command limits
(RANGE: 4.00 m forward, 1.50 m sideways and back, 0.50 m up), the view guards of a motion (the pause, NO_VIEW after
2 s, VIEW_FAST above 100 %, the near brake unless `mvnear 0`), the tilt stop of a motion (8 deg for 100 ms), the
vertical progress margin, the re-view bob, the Mac's person rule (PERSON_MIN_M 0.5 m with the k 0.6 overshoot, which
does not follow mvklow: a k under 1 is what the camera measures off a floor in view, not a setting) and its battery-
time refusal. With everything off the drone flies the move it was given until its profile and settle end, or one of
those stops it; a wall in the way is met at the profile's speed (0.20 m/s toward the scene at 100 %, less near it by
mvwmax), and the contact kill / hold decide from there.

### 38.4 Commands

```
python3 tools/espfly_diag.py auto set mvfloor 0      # no planning floor (the Mac's floor follows: 0)
python3 tools/espfly_diag.py auto set mvfstop 0      # the live front stop off
python3 tools/espfly_diag.py auto set mvprogm 0      # the progress guard off (mvprogr 0 does the same)
python3 tools/espfly_diag.py auto set mvlat 0        # no sideways cap
python3 tools/espfly_diag.py auto set mvklow 100     # no expansion safety factor
python3 tools/espfly_diag.py auto set mvnear 0       # (from 32) the near-scene brake off
python3 tools/espfly_diag.py auto set mvwmax 30      # approaches near the scene twice as fast (the view gate: ~25 is
                                                     # where the hold starts refusing frames near the end)
python3 tools/espfly_diag.py auto get                # ... mvfloor=0(nvs) mvfstop=0(nvs) ... mvklow=100(nvs) mvlat=0(nvs) mvwmax=30(nvs)
python3 tools/espfly_diag.py auto status             # ... mvGuards=0/0/0/0/40/40/0 klow=100 lat=0 wfwd=30 (after an ARM)
```
Back to the defaults: `auto reset mvfloor`, `auto reset mvfstop`, `auto reset mvprogm`, `auto reset mvprogr`, `auto
reset mvlat`, `auto reset mvklow`, `auto reset mvnear`, `auto reset mvwmax` (or `auto set` with 30 / 25 / 20 / 40 / 50 /
60 / 80 / 15). `auto set mvfloor 10`: `ERR auto set: mvfloor must be 0 (off) or 20..100 cm`. The minimum short of off:
`mvfloor 20`, `mvfstop 15`, `mvprogm 10`, `mvprogr 10`, `mvlat 10`, `mvklow 100`.

### 38.5 The Mac (tools/espfly_auto.py)

The Mac mirrors the settings from the `auto get` cache (`espfly_diag.py auto get`, re-read after every accepted `auto
set`): `mac_min_front()` is 0 with mvfloor 0 (no forward refusal for the scene ahead and no "at most F m now" clip;
`fwd_max` keeps the drone's 0.05 m, so a TOO_CLOSE from the drone is passed on with its hint), `cached_k_low()` gives
mvklow / 100 to the floor check and to the largest move it names (the person rule keeps K_LOW 0.6), `cached_lat_rel()`
lifts the sideways refusal with mvlat 0 (mvlat / 100 x D otherwise), `fast_caps()` carries mvwmax into `mv_limits` /
`move_profile` (a cap over 0.17 is itself the fast cap, as the drone's `autoMvSpeedCfg`). `state` adds `guards`: the
Mac's floor, k, the cap, the cached mvfstop / mvprogm / mvprogr, mvwmax and `off` (what is off). The move help and the
TOO_CLOSE / TOO_WIDE hints follow the settings. The person rule (PERSON_MIN_M 0.5 m, `--person-gap` with `--est`) and
the battery-time refusal stay as they are.

### 38.6 Tests

- test_auto_motion 600 (535): testGuardsOff: the clamps keep 0 as off (and the plan with the floor, the stop's own
  range without a floor), 1 clamps up to the ranges' lower ends as before; mvklow 100: the largest MOVE at D 2.0 1.69 m
  (1.70 ends 0.29999998 m in float), the stop ln(2 / 0.25), the measured lower bound equal to the measured D; mvfloor
  0: forward 1.94 at D 2.0 accepted (1.95 leaves 0.0499999 m in float: TOO_CLOSE), the plan too, the stop still ends
  the ideally tracked move FRONT, planned at 0.009 m/s; mvfstop 0: no front stop on a forward, sideways or back move
  (a reading of 5 is PROGRESS); mvprogm 0 and mvprogr 0 each: margins and frozenRef 1e30, readings of 5 / -5 never
  stop it, the front stop still does, a frozen camera runs the profile and the settle out and ends with the settle's
  STOP PROGRESS (at the defaults it stops in RUN), the frozen reference back to the speed step's once the guard is on
  again, no off-the-line SHORT; mvlat 0 / 100 / default at D 1.0; mvwmax 30: W_FWD 0.30 at 100 and at 300, the
  peak of forward 1.35 at D 2.0 0.0975 -> 0.195 m/s, the clamps, 0.17 at 150 back at 15; everything off: the command
  limits stay, forward 1.94 accepted with no stop and no margins, readings of 9 / -9 never stop it, ideally tracked
  DONE, the tilt stop and the 2 s view stop stay. The identity hashes (7ab74b9's, d3bedee's at 100..300) stand: the
  defaults are bit for bit the motions of 46d7fe2.
- test_auto_glue_core 219 (213): the four keys' 0 and 1..min - 1, `autoSettingOnMin`, the three new keys (ranges,
  defaults against the header, appended last), every mvfloor x mvfstop with 0.
- test_autonomy_glue_host 561 (501 + the scenarios since): the seven written in IDLE (out of range refused), latched
  at the ARM (`autoMotionGetMargins`, `autoMotionGetFwdRate`), refused while active, a MOVE forward 1.80 at D 2.0
  accepted and DONE with no floor (TOO_CLOSE at every setting of before), the defaults after `auto reset`.
- Python 362 (357 + 5): the floor off (the Mac accepts 1.9 at D 2.0, passes the drone's TOO_CLOSE at 1.97 on with
  its hint, `state` lists `off: ["floor"]`), mvklow 100 (1.60 accepted, 1.66 refused with "at most 1.65 m", 59
  falls back to 0.6), mvlat 0 and 100, everything off with the person rule standing, mvwmax (the limits, the profile,
  the cache's range, the default bit for bit); the constants against the header; the diag test's fake console replies
  with the new keys.
- The full runner (`int-tests/run_c_tests.sh`): every suite passes but the known test_gyro_tempco (33) and
  test_mahony_replay (4).

### 38.7 Risks and what stays open

- **With the guards off the firmware no longer stops a move for the scene ahead.** A move planned from a wrong D (a
  misjudged photo, a D off a near object with the target behind it) is flown to its end. The contact kill
  (AUTO_MVE_CONTACT, the hold's back-off anchor) and the tilt / rate kills are what is left; at 100 % the approach
  speed toward the scene is at most 0.20 m/s, less near it by mvwmax, so a contact is a push, as the user accepted.
- **mvklow 100 alone** keeps the floors but drops the k safety factor: with a floor in view (k 0.6-0.7) a forward
  move truly ends at D ((D - F) / D)^(1/k), nearer the scene than its planned D - F (the Mac's fwd_overshoot: 0.30 m
  beyond a 1.5 m move from D 2.0 at k 0.6), so a floor of 0.30 m planned at k 1 can come out as a touch. The Mac's
  person rule keeps modelling that overshoot at k 0.6.
- **mvwmax over ~25** fights the hold's own view gate near the end of an approach (38.2); a move that loses its view
  there ends NO_VIEW (2 s) or FRONT (the near brake, mvnear): it does not crash the camera, it ends the move early.
- **The progress guard off** also means a sideways runaway (the camera does not see the sides) is not stopped by the
  firmware; the contact stop and the hold's limit are what is left (32.3's cost, all of it).
- Not changed: the hold's gain scale during a move toward the scene (`dLowM`, the camera task, k 0.6 built in there:
  it is the hold's gain, not a guard), the D range 0.30-6 m, the view-shift re-view bob (0.35 x D), the vertical
  limits.
- Not flown. The noon log says which guard fired only by its STOPPED state; the first flight with `mvfstop 0` alone
  (the stop is the likely one) is the cheapest check before turning off more.

### 38.8 Cost

Against 46d7fe2 (`idf.py build`, the same 19fe168 sdkconfig copy, a fresh build directory, the same 14 warnings):
`.iram0.text` 129927 B unchanged (nothing new on the 1 kHz path: every setting is read by the once-per-command flash
code, moveFloor / trackD / nearScene / settleEnd / autoMvSpeedCfg, and the step keeps reading the per-move margins it
already had; `_iram_text_end` as before), `.dram0.data` 31608 -> 31624 (+16: the margins struct 8 -> 10 B, the fast
caps struct 8 -> 16 B with the rate cap's float and byte), `.dram0.bss` 88976 -> 88992 (+16: the three settings' RAM
copies), `_heap_start` 0x3fcc1708 -> 0x3fcc1728: the internal heap **32 B smaller** (the asked limit was ~100 B;
~19.2 KB expected after a reset, 17975 B measured with the Mac linked on 46d7fe2), `.rtc_noinit` 7660 unchanged (the
flight store's layout is not touched: the flights survive the flash), flash `.text` +828 B, `.rodata` +224 B,
`ESPDrone.bin` 1141168 -> 1142240 (+1072 B). `autoMotion_t` is unchanged (the identity hash's bytes, the IRAM code's
offsets). The USB console: `auto get` grows by three keys (its line buffer is AUTO_SET_COUNT x 28 B, on the stack,
+84 B of the console's 4768 B), `auto status` by ~60 characters (OUT_MAX_LEN 1536).
