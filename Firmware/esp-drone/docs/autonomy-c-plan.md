# Stage C plan (final): approach a person poster and stop at a safe distance

Final edit, 2026-10-12. This version applies the critic's review (R1-R17 and its 17
missing items) to the plan merged from the perception, control and safety plans.
Appendix A lists every review item and what was done with it.

- Merged plan before review: `scratchpad/stagec/editor/PLAN.merged-before-review.md`.
- Role notes and scripts: `scratchpad/stagec/{perception,control,safety,critic,editor}/`.
- Editor's first estimates, used for the provisional numbers below:
  - `editor/guard_sim2.py` and `editor/guard_sweep.py` (their output is in `guard_sweep.out.txt`);
  - `editor/feasibility.py` (its output is in `feasibility.out.txt`).
  They are rough Python estimates. Phase C2a replaces them with a closed-loop
  simulation built on the real C code.

**Firmware base.** Branch a2b-int at ff2c3e3.
- 6041dbd added the USB-only `auto unlock` for the reset-lock.
- ff2c3e3 changed docs only: unlock only after a reset done on purpose in this
  session (a flash, `reboot`, `mguard test panic`). After any other reset, save
  the crash log and the flights, and ask the user first.

**User constraint: no more tuning changes.** The user called tonight's flights
good. So this plan:
- leaves the hold gains, limits and slews unchanged (camera_hold.h:32-40);
- leaves the vertical controller, the height observer and the stored trims unchanged;
- adds only a moving-reference input to the hold. With no move running that
  input is 0, and a replay test proves the hover output is bit-identical to today's.

A trim refit and every behaviour change are separate user decisions (section 0).

**Paths.**
- Firmware paths are relative to `Firmware/esp-drone/components/core/crazyflie`.
- Tools paths are relative to `Firmware/esp-drone/tools`.

**Status.**
- C0 and C1 change no firmware, and their hover-only sorties stay far from the
  poster. They can start once the user has made the decisions below.
- No firmware is written before C1's measurements set the inputs for C2a: k_low_eff,
  b_max, LAND creep, hover creep and frame noise.

---

## 0. Decisions for the user

Each item gives a recommendation. Items 1-7 are needed before C0. The rest are
needed before the phase they name.

1. **Poster and range markers.**
   - An A0 portrait person print (84 x 119 cm) with a 4 cm black border, its
     top at or below 1.5 m, on a flat wall.
   - Two **30 cm** black squares on white, **1.40 m apart** centre to centre,
     one on each side of the poster at 0.4-0.5 m height.
   - Why 30 cm and 1.40 m: a 20 cm square is only 4.5-5 px at 4-4.5 m. 1.40 m
     keeps both markers clear of the poster and still gives 32 px of
     separation at 4.5 m, so 1 px is about 3 % of range.
2. **Distances: the 2 m rule applies to the poster too.** Recommended.

   | Layer | Value |
   |---|---|
   | Hard floor, compiled in (`AUTO_MOVE_FLOOR_MIN_CM`) | 2.0 m |
   | Minimum commanded stop, compiled in for this plan (`AUTO_MOVE_TARGET_MIN_CM`) | 2.8 m |
   | Commanded stop D_stop (NVS `autodtgt`, which can only raise it) | 2.3 m + measured margin M |
   | Red tape line | 2.0 m |
   | Kill line | 2.3 m (2.8 m in C3) |
   | C3: floor / minimum target | 2.5 m / 3.4 m |

   - The provisional margin M is 0.75 m (section 2), so expect the first stops
     near 3.0-3.3 m, not 2.5 m.
   - The stop comes down toward 2.8 m only as the measured terms of M shrink.
   - A stop below 2.8 m needs a firmware change and a new decision.
3. **Room and arena.**
   - Length: at least 5.5 m of clear floor in front of the wall. C3 starts at
     4.5 m, plus 1 m behind the start.
     - With only 5.0 m, C3 starts at 4.0 m and its steps are 0.1-0.45 m
       (section 3).
   - Width: at least 4.5 m. That is the corridor, 1.5 m clear on one side,
     and the operator 2.5 m away on the other side.
   - **The floor treatment is part of the spec, not a fallback.** At 40 cm
     the floor fills 32-41 % of the 60 deg vertical view, so "the wall fills
     the view" cannot be made true. Use a textured runner or patterned rug,
     at least 2 m wide, from the wall to 1 m behind the start.
   - Printed texture sheets on the wall around the poster.
   - Nothing in the 76 deg view from any start: no window, doorway, TV or
     moving object. Side furniture stays still.
   - Lights on and flicker-free: no dimmers.
4. **Operator position.**
   - Stand level with the kill line, at least 2.5 m to the side of the
     corridor, so you see the line side-on instead of along it.
   - Press **'k' for any forward crossing**, not the phone stick. The stick
     gives a LAND, and LAND creeps forward.
   - Use the phone stick for sideways drift.
   - Safety glasses. Nobody else in the room.
   - A phone on a tripod films the corridor from the side, as the video
     truth for distance and height.
5. **Trims and the hover's level point.** At 3.5 m, a 0.3 deg level error
   holds the drone about D·b/KI ≈ 0.5 m off its start point.
   - **Option A (recommended):** each session starts with an ordinary
     hover-only sortie from at least 3.5 m, and `levelfit.py` runs on its log.
     - Moving sorties fly only if |pitch level point - stored pitch trim| ≤
       0.15 deg.
     - If not, Claude reports the numbers and asks whether to refit with
       `atrim set`, the same calibration you did tonight. It is never
       automatic and never a gain change.
   - **Option B:** never refit. The larger offset goes into the margin
     budget, which pushes D_stop to 3.3 m or more.
6. **Coupling fallback, agreed in advance.** C0 may find that forward motion
   fakes vertical speed (|cvV| > 0.02 m/s at 0.2 m/s; earlier studies saw
   0.08-0.33 per m/s). If so:
   - the plan first lowers the wRef cap until the fake vertical speed is at
     most 0.02 m/s, and reruns C2a;
   - a move-only correction in the observer is a separate behaviour-change
     decision.

   Recommended: agree to the wRef fallback now.
7. **Your time.**
   - C0: about 90 minutes of bench work.
   - C1: at least 8 hover-only sorties with video, about 4-5 charges.
   - C3: at least 8 sorties.
   - C4: at least 6 approaches of 2-3 sorties each, so 12-18 sorties over
     6-12 charges.
8. **Who decides in the air.** Recommended:
   - Claude approves a policy for each sortie while the drone is on the
     ground, and the Mac's runner applies it in under 1 s. LAND is always
     available.
   - With the corrected time budget, a STEP has to be sent within about
     1-2 s of the STABLE photo (section 2.4). Claude's own 5-20 s cannot fit,
     so C5 (live in-air decisions) is expected not to happen.
9. **Firmware additions.** For your approval before C2b. Each is described
   in section 3.
   - approach_est on board;
   - push 0x85 APPROACH_STATUS;
   - about 10 B added to each recorder record;
   - STEP 0x31, with no maxMs field, and ABORT_MOVE 0x32, which bypasses the
     single pending-command slot the way KILL does;
   - the hold's moving reference: hover bit-identical, no gain change;
   - move_ref.c;
   - the move guards: progress, range floor with bias and coupling terms,
     time-to-contact, overshoot, view loss, sideways, travel;
   - a separate brake term in controller_pid. The setpoint attitude stays
     exactly 0;
   - the iFwd restore at every exit after a move;
   - the LAND reasons MOVE_GUARD and SETTLE_FAIL;
   - USB-only NVS keys, with moves off by default;
   - the two compiled floors.

   YAW 0x30 comes only in the optional C6.
10. **Motion limits.**
    - Forward only.
    - Each step at most 0.15 in ln-zoom (about 0.45-0.65 m).
    - wRef 0.06 /s, raised to at most 0.08 /s only after a clean C3. The
      compiled cap is 0.10 /s.
    - One step per sortie.
    - At most 0.8 m of travel per sortie.
    - No turning before C6.
11. **Height and battery.**
    - Fly at 40 cm (recommended).
    - The Mac refuses a moving sortie below 4.10 V at rest (recommended). A
      16 s sortie took the battery from 4.09 V to 3.37 V.
    - The firmware's vmin stays at 3.80 V.
12. **Sign-offs.**
    - After C0: the arena, f and cx, and k_low_eff.
    - After C1: b_max, LAND creep, hover creep, M and D_stop.
    - After C2a: the guard parameters, the false-abort rate and the L_fail table.
    - After C3: the updated M.
    - The C4 review gate after the first 3 approaches.
13. **Not in this plan.** Each would need your separate sign-off, and only if
    measured necessary:
    - a fix for LAND's forward creep (for example, coasting the forward hold
      on the restored integral during a LAND after a move);
    - a forward/vertical correction in the observer;
    - a move-only feedforward, if C2a finds the progress guard false-aborts
      too often;
    - a change to the 0.35 rad yaw reset;
    - raising the 18 s air limit, which two steps per sortie would need;
    - any stop below 2.8 m.
14. **A real person (C7) is not planned.** The flow measures the wall behind a
    person, not the person. That stage needs its own design, its own review
    and your explicit go.

---

## 1. Approach in one page

- **The camera hold does the moving, the firmware bounds it, and the Mac and
  Claude can only shorten it.**
  - A forward move is a moving reference wRef(t) fed into the existing drift
    hold. The hold integrates (wFwd - wRef), so its integral is the
    ln-position error.
  - The move never touches the setpoint attitude. autoBuildSetpoint keeps roll
    and pitch at exactly 0.0f (auto_glue_core.h:200-218). A nonzero pitch
    would:
    - zero the camera correction (controller_pid.c:266-269);
    - make camHoldStep see the sticks off-centre on both axes
      (camera_flow.c:325);
    - after 1 s, reset both integrals (camera_hold.c:63-70).
  - The profile is a gentle trapezoid in ln units: wMax 0.06 /s, 0.08 /s²,
    area dLn_cmd. No feedforward, unchanged gains, the same 2.5 deg limit.
- **The hold lags its reference badly, and the plan is built around that.**
  - Only 30-50 % of the travel is done when the reference ends; the rest
    comes during settling.
  - So every move guard stays on from STEP until touchdown, not just while
    the reference runs.
  - Settling is a stillness test on the summed expansion, not an
    attitude-only STABLE.
- **Progress is measured without a metric scale.** The summed image expansion
  is ln(D0/D).
- **The metric anchor comes from the Mac.**
  - The two markers in the photo give D = f·B/sep_px, with f and cx fitted
    against the tape in C0.
  - The poster-size match is an offline cross-check only. NCC is too slow
    for in-air decisions (R16).
  - The drone's own vertical-flow range D_G is **telemetry only**. It cannot
    flag the unsafe scene: in the plain-floor arena at 5 m, D_G under-reads
    exactly when k does (R6).
- **The expansion's scale factor k depends on the scene, and k < 1 is unsafe.**
  - k is measured per 0.5 m segment at the true hover camera height, and
    k_low = the minimum over segments and pushes.
  - k_low_eff = k_low·(1 - c_fv·c_vf) if the forward/vertical coupling loop
    is positive (R7, R8).
  - Every moving sortie checks k in flight from the marker photos before and
    after the move.
- **The expansion has an additive bias.** mission60's hover summed
  ln(1+s) = -0.19 in 10.9 s. The drone's range lower bound therefore
  includes a bias term b_max·t and a vertical-coupling term (R3).
- **A camera that reports 'ok' while seeing nothing is caught by a progress
  check,** not by dead reckoning.
  - The lean-model x_model wandered by up to +1.2 m in plain hover on
    mission60. It stays telemetry, computed offline, until it qualifies (R2).
  - The worst travel of each failure case comes from the C2a simulation and
    is checked against the floor before a STEP is accepted:
    - a frozen camera;
    - a camera biased by b_max;
    - view loss;
    - the camera task dying.
- **A written margin budget sets the stop.** D_stop = kill line 2.3 m + M,
  where M is the sum of the measured p99 terms, and D_stop is never below
  2.8 m (section 2).
- **Decisions are made between sorties.** It goes hop by hop: one step per
  sortie, land in place, measure, then the next sortie, with Claude
  reviewing the photos and logs in between.

---

## 2. Margin budget, acceptance rule and feasibility

### 2.1 Margin budget (R5)

Every term is a p99 in metres, at D ≈ 3 m, over the time from the anchor
photo to touchdown (about 8-10 s). The provisional values come from tonight's
logs and the role studies. Each is replaced by a measured value in the phase
named.

| # | Term | Provisional | Basis | Measured in |
|---|---|---|---|---|
| 1 | Move overshoot beyond the nominal end (k ≥ k_low_eff, latency 0.12-0.19 s, drag 0.2-0.5 /s, D error ±σ) | 0.25 | control sim, gentle move: p95 +0.17 to +0.22, worst +0.36 (control/numbers.txt) | C2a sim, C3/C4 video |
| 2 | True hover creep from the anchor to touchdown, including expansion bias | 0.30 | camera-only drift 9-25 cm/10 s at D 1.5-2.0 m; mission60's -0.19 ln is unexplained; no video truth yet | C0/C1 video at 3.0-4.5 m |
| 3 | LAND creep toward the poster | 0.15 | lean model: +0.06 (mission5s) and +0.14 m (mission60), still moving 0.12-0.14 m/s at touchdown (critic/land_lean.py) | C0/C1 video |
| 4 | Extra travel from the forward/vertical coupling loop | 0.05 | not measured yet | C0 (c_fv, c_vf), C3 video height |
| 5 | Range anchor error | 0 | already in D_cons = D_photo·(1 - 2σ), and the target-ratio form ends at or beyond the target | C0 |
| | **M (linear sum of p99s)** | **0.75** | | |

- **Rule.** D_stop = 2.3 m + M, rounded up to 5 cm, and never below 2.8 m.
  The provisional D_stop is 3.05 m.
- **Why a linear sum.** It is deliberately conservative: a root-sum-square of
  the same terms is about 0.45 m.
- **Operator reaction.** 'k' takes 0.3-0.5 s, about 0.10-0.15 m at 0.3 m/s.
  This is covered by the 0.3 m between the kill line and the 2.0 m floor,
  not by M.
- **Failure cases are checked against the floor, not against D_stop.**
  These are a frozen camera, a biased camera, view loss and a dead camera
  task. See L_fail in 2.2.

### 2.2 STEP acceptance rule (firmware)

All of the following must hold. ln() is computed in the link task, never on
the 1 kHz IRAM FSM path (R14).

1. **State.**
   - FLIGHT, HOVER and waiting; no photo set busy; STABLE reached.
   - At least 1 s of good view; the hold engaged and not coasting.
   - In DRY only, the sim plant stands in for the hold. A host test shows
     this path cannot be reached in FLIGHT.
2. **Enabled.** NVS `automove` = 1, at most one STEP per sortie, axis 0, and
   wMax ≤ `autowref`.
3. **Target.** dTarget ≥ max(`AUTO_MOVE_TARGET_MIN_CM` 280, `autodtgt`). The
   drone enforces this; it does not depend on the Mac's or Claude's policy.
4. **Freshness.** photoFrameSeq is at most 24 frames old, and D_low is valid.
5. **Anchor.**
   - D0_low = min(dCons carried forward from its frame by approach_est,
     every other anchor's D_low).
   - The Mac's dCons already holds -2σ, so the drone does not apply σ again (R14).
6. **Size.** dLn_cmd = min(1, k_low_eff)·ln(D0_low/dTarget) must be between
   0.02 and `autostep`.
7. **Failure bound.**
   - D0_low - L_fail(dLn_cmd, D0_low) ≥ `autodmin` + `autocreep`.
   - L_fail is the worst true travel to rest over the C2a failure cases,
     including detection, the brake, the glide and LAND. It is compiled in
     as a table.
   - Provisional envelope: 1.5·(D0_low - dTarget) + 0.20 m. The editor's
     sweep (`editor/guard_sweep.out.txt`) found frozen-camera travel up to
     0.78 m for a 0.5 m step.
8. **Time.** A STEP never extends the wait.
   - 18 s - air ≥ T_profile + 4 s (settle cap) + 0.5 s (photo) + 2.5 s (LAND)
     + 0.5 s (reserve).
   - The remaining wait ≥ T_profile + 4 s + 0.5 s.
   - T_profile is computed on board; the Mac sends no maxMs (R2b, R10d).

### 2.3 Feasibility (R10, provisional)

From `editor/feasibility.py`, with σ 0.02-0.04, k_low_eff 0.8-1.0 and air at
STEP 7.0 s. The nominal end is between the k_low and k = 1.5 cases. Every
accepted step has T ≤ 3.2 s, which fits the time rule.

| Phase sheet | Start (tape) | Accepted targets | Nominal end | Note |
|---|---|---|---|---|
| C3: autodmin 2.5, autodtgt 3.4, autostep 0.15 | 4.5 m | 3.6-4.0 m (3.8-4.0 when σ 0.02 and k_low 1.0) | 3.75-4.4 m (0.2-0.75 m of travel) | 3.4-3.5 m refused by autostep |
| same | 4.0 m | 3.4-3.7 m | 3.55-3.95 m (0.1-0.45 m) | too small to fit drag and overshoot well |
| C4: autodmin 2.0, autodtgt 2.8, autostep 0.15 | 3.5 m | 2.8-3.0 m (2.8 refused when σ 0.02 and k_low 1.0) | 2.9-3.4 m | |
| same | 3.3 m | 2.8-2.9 m | 2.9-3.2 m | |
| same | 4.0 m | ≥ 3.2 m only | 3.3-3.9 m | needs 2-3 hops |

- Because of the target-ratio form and D_cons, the drone ends beyond its
  target. With autodtgt 2.8, hop by hop from 3.5 m settles near 2.9-3.0 m
  after 2-3 sorties.
- With D_stop = 3.05 m the first approach ends near 3.1-3.3 m.
- Two steps in one sortie cannot fit the 18 s limit. The second STEP would
  come at 11-14 s of air, so that sub-phase (old C4b) is dropped.

### 2.4 Time line of a moving sortie

| Event | Air time |
|---|---|
| First thrust | 0 s |
| Hover wait begins | 5-6 s |
| STABLE photo at the Mac | about 6.5-7 s |

- The time rule (2.2, rule 8) needs air ≤ 10.5 s - T_profile at the STEP: about 7.5-8.5 s of air for a 2-3 s profile.
- So the Mac's runner applies the policy Claude approved on the ground, and
  that leaves about 1-2 s for a live decision.

---

## 3. On-board design

### 3.1 approach_est

A pure module, utils/src/approach_est.c with utils/interface/approach_est.h.
It has no ESP-IDF dependency.
- **C1:** it runs offline only, on recorder replays.
- **C2b:** on board, in the camera task right after holdUpdate
  (camera_flow.c:545), never on the 1 kHz path.

**Inputs.** All of them are already in the recorder:
- flowSeq, flowOk, flowTexture, flowResidual_dg, flowDt, flowScale_e4, flowGz;
- holdFlags, holdPitch, cvH, vzH, cvGain.

**What it computes on each frame.**
- **lnZoom**: Σ ln(1+scale) over frames that pass the hold's viewOk gate
  (camera_flow.c:540-541), counted from each anchor's frame.
- **fwdF**: the hold's 0.25 s filter. TTC = 1/fwdF.
- **D_est** = D_anchor·exp(-lnZoom).
- **D_low** = the minimum over the anchors of
  D_i·exp(-max(lnZoom_i, 0)/k_low_eff - b_max·t_i - c_vf·|Δh_i|/D_i).
  - D_i is the anchor's conservative value, so σ is applied once.
  - t_i is the time since the anchor frame, and Δh_i is the height change
    since then (vzH).
- **Gap fill.** A failed or missing frame counts as an approach at 0.25 /s.
  D_low is invalid after 300 ms without a good frame.
- **Move state.**
  - s, the lnZoom since STEP, and ∫wRef;
  - the stillness window: the change of lnZoom over the last 1 s.
- **Lateral.** latM = |∫wLat dt·D_est| since STABLE. It uses D_est because
  D_low would under-read the sideways travel.
- **Travel this sortie.** D_anchor0·(1 - exp(-max(lnZoom_total, 0)/k_low_eff))
  + b_max·t·D_anchor0.
- **D_G and its tracked flag:** telemetry only.
- **Not on board.** x_model is computed offline by range_review.py from
  holdPitch and pitch, with a 3 s baseline. It becomes a guard only if C1
  shows its false travel is below 0.15 m (p99, 6 s window) on at least 10
  hovers.

**Outputs.**
- **Recorder:** about 10 B appended:
  - lnZoom e4, D_low cm, D_est cm;
  - move phase and flags, the guard sub-code;
  - wRef e4, the brake in cdeg.

  The record grows from 103 to about 113 B (about +60 KB in PSRAM for the
  60 s ring). Update the assert in flight_recorder.c, RECORD_FIELDS in
  usb_diag.c, and espfly_diag.py.
- **Push 0x85 APPROACH_STATUS:** with every AUTO_STATUS and at each move
  transition, with an exact size and golden vectors.
  - AUTO_STATUS itself cannot grow: it is exactly 64 B and the Mac checks
    that size.
- **USB:** `auto approach`.

### 3.2 The hold's moving reference

- camHoldInput_t gets `wFwdRef`. The filter and integral act on
  (wFwd - wFwdRef), clamped as today.
- With wFwdRef = 0 the output must be bit-identical to today's over every
  logged flight.
- **iFwd snapshot.** The hold's iFwd is captured when a STEP is accepted.
  The FSM can then ask the camera task to restore it; the restore is applied
  at the next frame.
  - It is requested at **every exit after a move**:
    - ABORT_MOVE;
    - any MOVE_GUARD or SETTLE_FAIL;
    - LAND of any reason: HOVER_TIMEOUT, HB_LOST, phone, VBAT, AIR_TIME,
      NOT_STABLE, HOVER_NO_CAM;
    - view loss.
  - It is also requested **before any coast after a move**, so no wound-up
    push outlives a move (R4b, R9).
  - If the camera task is dead, its hold output goes stale and the
    controller drops it to 0 after 300 ms (controller_pid.c:259-261).
- **move_ref.c:** a trapezoid in ln units. wMax ≤ `autowref`, 0.08 /s², a
  triangle when the move is short, area dLn_cmd, and T_profile computed on
  board. The camera task evaluates wRef at each frame's time.

### 3.3 The brake (R1)

- **Where it goes.** A separate term in controller_pid.c, added after
  camCorrPitch:
  `attitudeDesired.pitch = setpoint pitch + trimP + camCorrPitch + brakeP`
  (today controller_pid.c:280).
  - The setpoint attitude stays exactly 0, so the hold keeps engaging or
    coasting on both axes, and no integral resets.
- **Who sets it.** The FSM (stabilizer task) publishes brakeP and its end
  time. The term is slewed at CAM_HOLD_APPLY_SLEW. It is zero:
  - when autonomyHoldBlocked() is set (after touchdown contact);
  - on KILL;
  - outside autonomy.
- **Size.** Nose-up at most 1.5 deg, for tb = min(1 s, v_hat/(g·tan 1.5°)),
  with v_hat = max(the last good fwdF, wRef now)·D_est.
  - It uses D_est, not D_low: D_low under-reads on purpose, which would
    undersize the brake.
  - It takes the max with wRef because a frozen 'ok' camera reports fwdF ≈ 0
    while the drone may still be moving at the reference speed.
- **When it is used.** View loss, and every guard abort while moving (fwdF
  or wRef above 0.02 /s).
- **Tests.** A host test shows that a brake leaves holdRoll, iLat and the
  restored iFwd untouched. The C2a sim bounds the backward overshoot to
  0.3 m.

### 3.4 STEP 0x31 and ABORT_MOVE 0x32

**STEP payload** (12 B):

| Field | Type |
|---|---|
| stepId | u16 |
| dConsCm | u16 |
| dTargetCm | u16 |
| photoFrameSeq | u16 |
| wMaxE4 | u16 |
| photoN | u8 |
| axis (must be 0) | u8 |

There is no maxMs field: the drone computes every time limit itself.

**How a STEP is handled.**
- The link task parses STEP. It reads the latest approach_est snapshot
  (published under the hold mutex) and computes dLn_cmd, D0_low, T_profile
  and L_fail.
- It posts a command that carries those numbers. The FSM only compares them
  and checks state and time (section 2.2).

**ABORT_MOVE** works like KILL. It is a direct atomic request
(autonomyAbortMoveFromLink) outside the single pending-command slot, so it is
never answered BUSY. Today LAND and every FSM command share that slot
(auto_link.c:944-948), while KILL bypasses it (auto_link.c:1058).
- LAND keeps its flown path. The Mac resends LAND every 100 ms until it gets
  an ACK (R13).

**Move phases.**
- **REF:** the reference runs.
- **SETTLE:** done when the drone is still, i.e. |the change of lnZoom over
  1 s| < 0.015 (provisional) and lnZoom ≤ dLn_cmd + 0.05. The thresholds are
  at least 3 sd of the C1-measured frame noise.
- **Caps:**
  - a progress check at T_profile + 1 s, or at the stillness test if that
    comes first;
  - a settle cap at T_profile + 4 s.
- **DONE:** then the post-move photo, then waiting, then LAND.

### 3.5 Guards: active from STEP accept until touchdown (R9)

| Trigger | Action |
|---|---|
| ABORT_MOVE from the Mac | restore iFwd, brake if moving, then hover (the wait continues) |
| Progress: lnZoom < A·dLn_cmd at the check (A provisional 0.3, set in C2a) | restore, brake, LAND MOVE_GUARD/PROGRESS |
| Settle cap reached without stillness | restore, brake, LAND SETTLE_FAIL |
| TTC: fwdF > min(0.25, 2·wMax + 0.05) /s for 300 ms | restore, brake, LAND MOVE_GUARD/TTC |
| Overshoot: lnZoom > dLn_cmd + 0.05 | restore, brake, LAND MOVE_GUARD/OVER |
| Range: D_est or D_low < `autodmin` + `autocreep` (R4a) | restore, brake, LAND MOVE_GUARD/RANGE |
| View lost: no good frame for 150 ms, or coasting | restore, brake; LAND MOVE_GUARD/VIEW if the view is not back within 1 s |
| Sideways: latM > 0.5 m (moving sorties only) | LAND MOVE_GUARD/LAT |
| Travel this sortie > `autotravel` | LAND MOVE_GUARD/TRAVEL |
| Any A2b exit | ends the move with an iFwd restore first, then the A2b action |

The A2b exits are KILL, trips, phone, heartbeat, battery, AIR_TIME,
HOVER_NO_CAM, NOT_STABLE and HOVER_TIMEOUT.

Any guard trip after a STEP lands the drone. Only the Mac's own ABORT_MOVE
returns it to hover.

### 3.6 NVS keys

USB console only, motors idle. Moves are off by default.

| Key | Meaning | Default | Range or floor |
|---|---|---|---|
| automove | moves enabled | 0 | 0/1 |
| autodmin | distance floor, cm | 250 | compiled floor 200, at most 400 |
| autodtgt | minimum commanded target, cm | 340 | compiled floor 280, and at least autodmin + 50 |
| autostep | max ln-zoom per step, e4 | 1500 | cap 2000 |
| autowref | max reference rate, e4 /s | 600 | compiled cap 1000 |
| autotravel | max travel per sortie, cm | 80 | cap 110 |
| autoklo | k_low_eff, e3 | from C0 | compiled floor 500 |
| autobmax | b_max, e4 /s | 200 | floor 100 |
| autocvf | c_vf, e3 | from C0 (800 until then) | floor 0 |
| autocreep | LAND creep p99, cm | 15 | floor 10 |
| autoprog | progress threshold A, e3 | from C2a (300) | 100-800 |

---

## 4. Phases

**Rules for every firmware phase.**
- It builds, and every host suite stays green: 20 C programs, test_auto_fsm
  (1014 checks) and 155 Python tests.
- `auto prof` shows at least 975 loops/s. At least 1 KB of stabilizer stack
  and 12 KB of internal heap stay free. IRAM growth is recorded.
- It is flashed app-only. The bin is copied to firmware-backups.
- After the flash, `auto unlock` (a reset on purpose). After any other
  reset, save the crash log and the flights, and ask the user.

**Before every moving sortie (C3 onward):**
- the session's level-point check (decision 5);
- the battery at least 4.10 V at rest;
- the pre-flight NVS readback matches the phase sheet;
- 20 s still on the ground before ARM.

### C0: arena and ground truth (no firmware change)

**Goal.** Before anything gates motion:
- measure every range source against the tape and against video, in the
  real arena;
- measure k per 0.5 m segment at the true hover camera height;
- measure the coupling loop with its signs, and hover drift and LAND creep
  at 3.0-4.5 m.

If the arena fails, fix it.

**Firmware.** None: the build flashed with the USB unlock (6041dbd; ff2c3e3
changed only docs). It uses only existing paths:
- `stream 100`, with every recorder field at 100 Hz with the motors idle
  (flight_recorder.c:198-201, usb_diag.c:109);
- `camvz air on` (IDLE, motors idle);
- PHOTO in IDLE;
- the ordinary A2b hover sortie (hop_wait_h.py).

**Mac, new tools:**
- **tools/poster_locate.py** (numpy/PIL, run with python3 -I):
  - the marker pair gives bearing = atan((x - cx)/f) and D_photo = f·B/sep_px;
  - it outputs D_cons = D·(1 - 2σ), texture, lum and an annotated x4 PNG;
  - its markers-only path is timed and is the only one used in the air;
  - the multi-scale NCC poster match (0.5-2x) is an offline cross-check
    with its own timing.
- **tools/range_bench.py**, from a stream log plus the mark times. It
  reports:
  - k per 0.5 m segment;
  - c_fv, the fake cvV per m/s of forward push, with its sign;
  - c_vf, the expansion per m of vertical motion, with its sign;
  - D_G/D_true;
  - the bearing residuals and the fitted f and cx.
- **tools/video_marks.py**: frame times plus the tape and height-scale
  readings from the side video, as CSV.

**Tests.**
- **test_poster_locate.py** on rendered markers and posters: scale 0.5-2x,
  bearing ±35 deg, noise, gamma, blur, ±1 px. It must give:
  - bearing within ±0.5 deg;
  - range within ±4 % up to 4.5 m;
  - D_cons ≤ truth in 100 % of cases;
  - a wrong target rejected.
- range_bench self-test on a synthetic fixture.

**User procedure** (about 90 min):
1. **Setup.**
   - Poster, markers, floor runner, wall texture, lights.
   - Floor tape every 0.5 m from 2.0 to 5.0 m, a red line at 2.0 m and kill
     lines at 2.3 and 2.8 m.
   - A vertical height scale (5 cm marks) beside the corridor.
   - A phone on a tripod 3 m or more to the side, filming 2-5 m of the
     corridor.
2. **True hover camera height.** First fly one ordinary hover sortie (40 cm,
   autowait 10000) from 4.0 m and read its height by video against the scale.
   - In mission5s, cvH (0.21 m) and vzH (0.32 m) disagreed by 11 cm.
3. **Ground photos.** On a box at the measured camera height, 3 PHOTOs at
   each mark from 2.0 to 4.5 m. Time poster_locate on each.
4. **Sign and bearing check** at 3 m on a protractor sheet: 0, ±10, ±20 and
   ±30 deg, with a landmark on the right. Confirm the yaw, dx and bearing
   signs, and that the photo is not mirrored.
5. **Cart pushes**, props off and motors never armed, `camvz air on`,
   `stream 100` logged:
   - three pushes from 4.5 to 2.0 m at 0.15-0.2 m/s at the hover camera
     height, plus one push each at +10 cm and -10 cm;
   - pause 2 s at each mark.
6. **Hand bobs** of ±5-10 cm at about 0.7 Hz for 5 s at 4.5, 3.5 and 2.5 m.
7. **One push with a hazard** (sofa or window) in view.
8. **Hover-only sorties:** 40 cm, autowait 10000, 2 photos, from 3.0, 3.5,
   4.0 and 4.5 m facing the poster.
   - The operator stands at the kill line, 2.5 m or more to the side, with
     'k' ready.
   - Afterwards: the recorder dump, the video, and the landing spot measured
     by tape.
   - These start C1's data set.

**Exit criteria.**
- **Bearing:** right is positive; error ≤ 1 deg within ±30 deg after fitting
  cx and f.
- **D_photo against the tape:** within ±5 % on the ground from 2.0 to 4.5 m;
  markers found in 100 % of photos; D_cons ≤ tape in every case.
- **Timing:** markers-only locate ≤ 0.2 s per photo on the Mac. NCC is
  offline if it takes more than 0.5 s.
- **k:** per 0.5 m segment, k_low = the minimum over segments, pushes and
  heights.
  - It must be at least 0.8 after k_low_eff = k_low·(1 - c_fv·c_vf) whenever
    that loop product is positive.
  - Otherwise fix the arena (runner, texture, start distance) and repeat.
- **Coupling:** c_fv and c_vf reported with their signs. If |cvV| > 0.02 m/s
  at 0.2 m/s, apply decision 6.
- **View quality:** texture ≥ 30 and lum 60-200 at every mark.
- **D_G:** D_G/D_true reported; it is telemetry only.
- **Hover:** the true hover height is known; each hover sortie ends
  HOVER_TIMEOUT with a soft touchdown; drift and LAND creep by video are
  recorded.
- **Sign-off:** the user signs off the arena, f and cx, and k_low_eff.

### C1: offline estimator against video (no firmware change, hover-only sorties)

**Goal.** Run approach_est offline on real hover flights against video truth.
From that, measure what C2a needs:
- b_max, the LAND creep p99 and the hover creep p99 at 3.0-4.5 m;
- the frame noise, and the level-point stability;
- whether x_model could ever qualify;
- the Mac's latency.

The result is the first measured margin M.

**Firmware.** None. The ordinary hover sortie.

**Mac.**
- approach_est.c compiled for the host, plus tools/range_review.py: replay a
  recorder dump with anchors (tape or marker photos) and compare frame by
  frame with the video marks.
- A DRY latency harness: photo saved → the Mac's policy command (target
  ≤ 1 s); Claude's own photo → decision time.
- levelfit.py on each sortie.

**Tests.** tools/tests/test_approach_est.c:
- **Synthetic scenes:** constant speed with k from k_low_eff to 2.5; an
  additive bias up to 2·b_max; descent and climb; gaps, frame jumps, NaN,
  viewOk false; anchors that disagree.
- **Invariant:** D_low ≤ truth for every k ≥ k_low_eff and bias ≤ b_max, over
  10^4 random sequences.
  - The test model includes the additive bias and the vertical term, so the
    invariant can fail.
- **Replays:**
  - mission60, where -0.19 in 10.9 s must be bounded by the bias term;
  - mission5s, calib1 and calib2, hop5s, and the C0 pushes and sorties.
- **Mutation tests that must fail the suite:**
  - min replaced by max;
  - the gap fill off;
  - k_low ignored;
  - the bias term off;
  - the vertical term off;
  - a stale anchor reused;
  - a frame gap ignored.

**Flights.**
- Hover-only sorties as in C0 step 8, two from each of 3.0, 3.5, 4.0 and
  4.5 m: at least 8 sorties, about 2 per charge.
- Video and tape for each.

**Exit criteria.**
- **D_low:** ≤ the true distance (video or tape) at every frame of every
  sortie; median |D_est/true - 1| ≤ 15 %.
- **b_max:** the p99 over 10 s windows of |expansion-implied - true| drift
  rate, and at least 0.01 /s.
  - If b_max > 0.03 /s, stop and escalate: the margins cannot bound it.
- **Creep:** LAND creep p99 (sets autocreep) and hover creep p99 from the
  anchor to touchdown at 3.0-4.5 m (margin term 2).
- **Frame noise:** the high-pass per-frame noise is measured; it is the
  input to the C2a stillness and progress thresholds.
- **Level point:** the spread of the level point per sortie is reported
  (decision 5).
- **x_model:** with a 3 s baseline, its false travel over a 6 s window is
  reported. It stays telemetry unless p99 < 0.15 m on 10 or more hovers.
- **Mac range:** in-air marker range within ±8 % of video or tape.
- **Latency:** the Mac's policy time ≤ 1 s; Claude's latency reported.
- **Sign-off:** M is computed from the measured terms, and the user signs
  off M and D_stop.

### C2a: pure modules and the closed-loop simulation (host only, no flash)

**Goal.** Write and prove the pure parts on the host with the real C code,
and derive from the simulation every number the guards need:
- the progress threshold A and its check time;
- the stillness thresholds;
- the L_fail table;
- the brake sizing;
- the overshoot p99, which is margin term 1.

**Firmware.** Pure code only, not flashed:
- approach_est.c;
- move_ref.c;
- camera_hold.c/.h: wFwdRef, plus the snapshot and restore.

**Mac.** None.

**Tests.**
- **test_camera_hold.c:** existing cases unchanged; with wFwdRef = 0,
  bit-identical over every logged flight's frame sequence; tracking; save
  and restore.
- **test_move_ref.c:** area within ±1e-4, the rate and acceleration limits,
  the triangle profile, T_profile.
- **test_move_closedloop.c:** the real hold, move_ref and approach_est on a
  1-D plant with vertical coupling and a LAND phase. Parameters:
  - latency 0.12-0.19 s, about 15 fps;
  - frame noise from C1;
  - drag 0.2-0.5 /s;
  - D error ±σ;
  - residual level error ±0.15 deg;
  - k from k_low_eff to 2.5;
  - bias 0 to b_max;
  - c_fv and c_vf from C0;
  - the LAND creep model.
- **Cases:**
  - normal;
  - a frozen 'ok' camera;
  - a camera biased by b_max;
  - view loss at peak speed;
  - the camera task dead;
  - ABORT_MOVE mid-move;
  - a regression of the critic's frozen_cam case (3.0 → 2.6 m ended at
    1.89 m under the old rule). The new rule must refuse that STEP or bound
    it above autodmin.

**Exit criteria.** For every pair the C3 and C4 phase sheets accept:
- the normal case's true end ≥ D_stop - margin term 1;
- every failure case's true end ≥ autodmin;
- the brake's backward overshoot ≤ 0.3 m;
- the progress check catches 100 % of frozen cases;
- the normal false-abort rate is ≤ 10 % over the grid.
  - The editor's rough sweep gave 10-45 % at A 0.3 with noise 0.0015-0.003
    per frame. A false abort lands short, which is safe but costs sorties.
  - If ≤ 10 % cannot be reached, the user decides: accept the rate; use
    smaller or slower steps; or a move-only feedforward (decision 13).
- L_fail and A are compiled in.
- An independent review of the simulation leaves no open item.

### C2b: firmware integration, DRY, and shadow flights

**Goal.** STEP and ABORT_MOVE work end to end with every guard, on the host
and in DRY with props off. On-board approach_est matches the offline replay,
and the hover is unchanged.

**Firmware.**
- **Rebase** onto ff2c3e3.
- **approach_est** called in the camera task.
- **Recorder fields** and **0x85**: auto_proto.h, auto_link.c, golden vectors.
- **STEP and ABORT_MOVE**: STEP through the link task, ABORT_MOVE outside the
  pending slot.
- **FSM**: the move sub-state inside HOVER, so no new state value goes on
  the wire; the guards; the iFwd restore at every exit.
- **The brake term** in controller_pid.c.
- **LAND reasons** MOVE_GUARD (with sub-codes) and SETTLE_FAIL, appended.
- **NVS keys** and the two compiled floors.
- **D_G** sampled in the aux task (the heapFree pattern, autonomy.c:983-988).

**Mac.** espfly_auto.py:
- decodes 0x85 with the exact-size check;
- the `step` and `abort-move` ops; daemon key 'a' = ABORT_MOVE;
- LAND resent until it is ACKed.

**Tests.** test_auto_fsm:
- the accept/refuse matrix: autodtgt, the 280 cm floor, caps, stale frame,
  budget, one STEP per sortie;
- every guard;
- every A2b exit ends a move and restores iFwd;
- ABORT_MOVE is never refused, including while a STEP or a PHOTO is pending;
- LAND while a STEP or PHOTO is pending gives BUSY_PENDING, and the Mac's
  retry gets it through;
- KILL during a move cuts the motors;
- the DRY bypass cannot be reached in FLIGHT.

Also:
- A controller host test: the brake leaves holdRoll, iLat and the restored
  iFwd untouched, and the setpoint attitude stays 0.
- test_auto_sortie with a STEP; golden vectors (0x85, a 12 B STEP);
  auto_link_host; test_espfly_auto.
- Budgets: the IRAM call graph unchanged, `auto prof` ≥ 975 loops/s, stack
  ≥ 1 KB, heap ≥ 12 KB.

**DRY bench** (props off):
- lens covered → STALE;
- the poster in view → accepted and runs to DONE;
- a frozen-view trick (a printed photo held still in front of the lens) →
  PROGRESS;
- 'a' stops a move;
- the daemon killed → HB_LOST LAND, with iFwd restored;
- phone stick → LAND;
- no motor turns.

**Flights.** Shadow flights: 2-3 ordinary hover-only sorties with
`automove` 0.

**Exit criteria.**
- All host tests pass, including the hover-identity replay.
- The DRY checklist passes.
- On the shadow flights:
  - the on-board values equal the offline replay within 1e-3;
  - the hover metrics (tilt rms, hold %, drift) are within tonight's range.
- An independent skeptic and safety review of the diff leaves no open item.

### C2c: Mac runner and end-to-end rehearsal (DRY, props off)

**Goal.** The Mac side is complete and fault-tested before any move flies.

**Firmware.** None (C2b).

**Mac.**
- **The `approach` runner.** It applies Claude's per-sortie policy JSON
  (target, max bearing, min texture) to the first STABLE photo, and sends
  STEP or LAND. It logs JSONL next to the photos.
- **The pre-send rule:**
  - a photo less than 2 s old with the markers found;
  - |bearing| ≤ 6 deg;
  - D_cons - target ≥ 0.3 m;
  - target ≥ D_stop.
- **k_flight and range consistency**, per sortie (R6):
  - k_flight = lnZoom(photo0 → photo1)/ln(D_photo0/D_photo1), with its 95 %
    interval from σ;
  - D_photo after the move against D_est.
  - The Mac stops the campaign if the upper bound of k_flight < k_low_eff,
    or if the running median over 3 or more sorties < k_low_eff, or if
    D_photo is below D_est by more than 2σ.
- **Pre-flight:** the `auto get` readback against the phase sheet.
- **The 4.10 V battery rule.**

**Tests.**
- Python tests for the runner, with fault injection: ±7 % range, a missed
  marker, a wrong bearing sign, a stale photo, a k_flight below k_low. In
  none of them may the runner send a STEP that the rules forbid.
- DRY rehearsals with the poster arena: the whole sortie, ABORT, LAND retry,
  latency.

**Flights.** None.

**Exit criteria.**
- The tests pass.
- Three DRY rehearsals run cleanly.
- The latency is within budget.

### C3: first moving flights far from the poster (floor 2.5 m, kill line 2.8 m)

**Goal.** One step per sortie, with a wide margin. Measure:
- true travel against the command, overshoot and settling;
- the height change during the move, by video;
- LAND creep after a move;
- drag μ;
- k_flight.

Refit the simulation and update M.

**Firmware.** C2b. Phase-sheet NVS:

| Key | Value |
|---|---|
| automove | 1 |
| autodmin | 250 |
| autodtgt | 340 |
| autostep | 1500 |
| autowref | 600 |
| autotravel | 80 |

autoklo, autobmax, autocvf and autocreep come from C0/C1; autoprog from C2a.

**Mac.** The approach runner in scripted mode, with targets from the
feasibility table.

**Tests.** After each flight:
- the recorder dump, range_review and video_marks;
- the landing spot measured by tape;
- a closed-loop replay with the measured latency and drag.

**Flights.**
- Start on the 4.5 m mark (4.0 m if the room is short), aimed by hand with
  the bearing within ±3 deg.
- Sortie: 40 cm, autowait 10000; the STABLE photo, then one STEP, then the
  stillness test, then a photo, then LAND (the Mac sends it 1 s after DONE).
- Targets of 4.2, 4.0 and 3.8 m, 2 sorties each.
- Two more sorties with 'a' pressed mid-move, to measure the abort-and-brake
  stop in flight.
- At least 8 sorties.
- Kill line 2.8 m. The operator is level with it, 2.5 m or more to the side.

**Exit criteria.** In every flight:
- closest true distance ≥ 3.0 m;
- true travel ≤ nominal + C2a overshoot p99 + 0.10 m;
- hold pitch ≤ 2.5 deg;
- height change ≤ 0.10 m by video against the scale (not cvH or vzH);
- stillness reached within T_profile + 4 s;
- k_flight consistent with k_low_eff;
- no unexplained guard trip, SETTLE_FAIL, HOVER_NO_CAM or trip.

Also:
- Simulation and flight overshoot agree within ±0.15 m; otherwise refit and
  repeat.
- The measured terms 1-4 update M and D_stop.
- If D_stop > 3.3 m: lower wRef or the step size, or escalate. The hold gains
  are not changed.

### C4: approach the poster and stop at D_stop (floor 2.0 m, kill line 2.3 m)

**Goal.** The deliverable. From 3.5-4.0 m, approach the poster hop by hop and
stop at D_stop = 2.3 m + M (2.8 m or more), repeatably, never crossing the
2.3 m kill line. Also measure every margin term again on approach data.

**Firmware.** C2b. Phase-sheet NVS:
- autodmin 200;
- autodtgt = D_stop from C3;
- autostep 1500;
- autowref 600 (800 only if C3 was clean and the simulation rerun agrees);
- autotravel 80.

**Mac.**
- Hop by hop, one step per sortie, landing in place.
- Between sorties, Claude reviews the ground photo, the in-air photos,
  k_flight and the log, then writes and approves the next policy.
- The approach is complete when the drone or the Mac refuses the next step
  (target ≥ D_stop).
- A mission log: one JSON per sortie.

**Tests.**
- Before each session: the full host suite, `auto prof`, the battery, the
  level-point check, and the floor lines visible on the video.
- After each flight: a row in the running table (true closest and final
  distance, D_photo and D_est against truth, overshoot, sideways drift,
  k_flight) and a closed-loop replay.

**Flights.**
- 3.5 m → about 3.1 m → about 2.9-3.0 m, or from 4.0 m in 3 hops.
- Re-aim by hand only if the bearing is off by more than 5 deg.
- At least 6 approaches (12-18 sorties).
- **Review gate after the first 3 approaches:** Claude and the user review
  the table. Continue only if every criterion holds.

**Exit criteria.**
- Every sortie: the closest true distance (video) ≥ 2.3 m; the kill line is
  never crossed.
- Every final stop is within [D_stop - margin term 1, D_stop + 0.5 m].
- Each margin term, re-measured on C4 data, is at most its budget value, and
  their sum ≤ D_stop - 2.3 m.
- D_photo at the stop is within ±10 % of the video.
- Every refusal, guard trip and Claude-versus-drone disagreement is logged
  and explained.
- The user agrees with M.

The count of approaches alone is weak evidence: 0 failures in 8 gives only
about 70 % confidence that the failure rate is below 14 %. The safety
argument rests on the per-term budget and the independent floors.

### C5: Claude decides live in the air (Mac only; expected not to fit)

**Goal.** Claude reads the in-air STABLE photo and sends STEP, no-go or LAND
itself. This happens only if the C1-measured photo-to-command latency fits
the window of about 1-2 s (section 2.4).

**Firmware.** Unchanged.

**Mac.**
- The daemon pushes the PNG plus the marker range and bearing; Claude sends
  `step` or `land`.
- No-go rules enforced by the Mac:
  - markers not found;
  - |bearing| > 6 deg;
  - texture < 30;
  - D_photo and D_est disagree by more than 15 %;
  - battery low;
  - too little air left for the time rule.

**Tests.** DRY:
- the latency fits;
- Claude silent → HOVER_TIMEOUT;
- a step beyond the caps → refused.

**Flights.** As in C4, but Claude decides. 5 sorties, one with the poster
15 deg off-centre (expect a no-go).

**Exit criteria.**
- All 5 decisions in time.
- The bad view gets a correct no-go.
- Stops as good as in C4.
- If the latency does not fit, C5 is skipped, and Claude's role stays the
  per-sortie policy.

### C6 (optional): YAW to aim at an off-axis poster

**Goal.** Turn toward a bearing Claude picks while keeping the hold, the
camera height estimate and STABLE. Then re-anchor the range before any step.

**Firmware.**
- YAW 0x30: i16 deltaCdeg (+ = right), photoN, photoGapMs.
- An IRAM ramp of yawHold in autonomy.c: ≤ 8 deg/s and ≤ 16 deg/s². Done when
  the error is < 1.5 deg and |gz| < 5 dps.
- At most ±12 deg net per sortie (NVS cyawmax ≤ 12), one turn per sortie,
  then SETTLE.
- approach_est re-anchors after any yaw: a new photo range is needed before
  a STEP.
- The sign rule is confirmed by the C0 check.
- The 0.35 rad gain-tracker reset (cam_vz_obs.c:490-503) is unchanged.
  Changing it would be decision 13.

**Mac.** A `yaw` op.

**Tests.**
- test_auto_fsm YAW rows: refused outside the wait, beyond the cap, or while
  BUSY; LAND, phone, heartbeat and KILL during a turn; SETTLE_FAIL.
- Ramp, wrap and abort in test_autonomy_glue_host.
- DRY with `auto prof`.
- A bench turntable.

**Flights.**
- 40 cm: photo, YAW ±5 deg, photo, LAND. Then ±10 deg.
- Then from 3.5 m: the poster at 10 deg; turn, re-anchor, step.

**Exit criteria.**
- Heading within ±2 deg of the command.
- A landmark's bearing in the photos shifts by the command ±2 deg.
- No HOVER_NO_CAM.
- The hold engaged on ≥ 90 % of the frames during the turn.
- STABLE again within 2 s.
- A re-anchor before every step.
- The C4 criteria still hold.

### C7 (not planned): a real person

Only after the C4 evidence and an explicit go from the user.

**Why it needs its own design.**
- The flow measures the wall behind a person, not the person.
- A person who moves drags the flow fit (cam_vz_obs.h:56-58).

**A candidate design.**
- The person stands with their heels 0.5 m in front of the marker wall, so
  the distance to the person = D_photo(wall) - 0.5 m.
- Commanded stop max(2.8 m, 2.3 m + M) from the person.
- Prop guards if available.
- The first flights stop at 3.0 m or more.

It needs its own plan, its own review and its own decision.

---

## 5. Safety case

**What the layers do.** They are independent and aim for this: no known
single failure, and nothing Claude or the Mac says, puts the drone inside
2.0 m.

**What they do not do.** The 2.0 m floor is enforced through estimates of
the range: the drone's own, carried forward with conservative terms, and the
Mac's. It is not a physical barrier. The physical backstop is the operator
at the 2.3 m kill line with 'k'.

**Residual risks that remain after these layers** (R17):
- a frozen 'ok' camera: up to about 0.8 m of travel per 0.5 m step before
  the progress check catches it (editor's estimate);
- an expansion bias larger than b_max;
- LAND creep beyond its p99;
- a scene with k below k_low_eff.

Each is bounded by a check below, and each is measured before it is relied on.

1. **Compiled limits.**
   - The floor is 2.0 m. The minimum commanded stop is 2.8 m.
   - NVS can only raise them, and no link command carries a distance below
     them.
   - Moves are off by default and enabled only over USB.
2. **Each move is bounded before it starts.**
   - A STEP is accepted only if:
     - the floor still holds after the worst failure-case travel
       (D0_low - L_fail ≥ autodmin + LAND creep);
     - the target is at least D_stop;
     - the time rule leaves room to settle, photograph and land.
   - The rate is capped at wRef ≤ 0.08 /s and the lean at the unchanged
     2.5 deg.
3. **The drone's own range is a lower bound with every known error term.**
   - The anchor is conservative, and σ is applied once.
   - It is carried forward with k_low_eff, the bias b_max·t and the
     vertical-coupling term.
   - Gaps are filled assuming a worst-case approach, and the range is invalid
     after 300 ms without a good frame.
   - Every LAND-triggering range threshold includes the LAND creep.
4. **Guards stay on from STEP until touchdown:**
   - progress (it catches a camera that sees nothing);
   - stillness, with a settle cap;
   - time-to-contact, overshoot, range, view loss, sideways, travel.
   - Any trip lands the drone.
5. **No wound-up push outlives a move.**
   - iFwd is restored at every exit after a move, and before any coast.
   - The brake is its own controller term (≤ 1.5 deg, ≤ 1 s). It leaves the
     hold on both axes and its integrals intact.
   - A dead camera task's correction drops to 0 after 300 ms.
6. **Claude and the Mac can only make the drone more cautious.**
   - Their range is min-combined with the drone's own.
   - The Mac can only refuse, abort (ABORT_MOVE never BUSY), land (resent
     until ACKed) or kill.
   - k_flight and range-consistency stop rules end the campaign if the scene
     model is wrong.
   - A lost link → LAND after 1 s; a silent Claude → HOVER_TIMEOUT. A STEP
     never extends the wait.
7. **The arena is made to fit the assumptions, and is measured.**
   - The floor runner and wall texture are in the spec.
   - k is measured per 0.5 m segment at the true hover height.
   - There are no hazards in view, and moves are forward only, aimed by hand.
   - The corridor has 1.5 m clear on each side, with a sideways guard
     on board.
8. **Every sensor and error term is measured before it gates motion.**
   - C0 and C1 against video and tape.
   - The C2a simulation sets the guards.
   - C3 measures moves far from the poster before C4 lowers the floor.
9. **Every A2b rule is unchanged and takes precedence:**
   - 'k' kill within 50 ms;
   - trips KILL;
   - phone stick → LAND;
   - heartbeat lost for 1 s → LAND;
   - NOT_STABLE, HOVER_NO_CAM, the 18 s AIR_TIME and the fast-sag battery
     LAND;
   - the reset-lock (cleared by a battery power-on, or by `auto unlock` only
     after a reset on purpose);
   - vmin 3.80 V.
10. **People.**
    - The operator stands level with the kill line, 2.5 m or more to the
      side, with safety glasses, and watches the line, not the screen.
    - 'k' for a forward crossing.
    - Nobody else in the room.
11. **No tuning change.**
    - With no STEP, the code path is today's hover; the wFwdRef = 0
      bit-identity replay and the C2b shadow flights show it.
    - A trim refit or any behaviour change needs the user's yes.
12. **Scope.** This method does not cover a free-standing person (C7).

---

## 6. Risks

1. **k depends on the scene, and k < 1 is unsafe.**
   - Rendered arenas gave 0.27-2.5. Per segment, wall plus a plain floor gave
     2.28 at 4 m and 0.73 at 3 m.
   - Mitigations: segment-wise k_low at the hover height, the coupling
     correction, the floor runner, the in-flight k check and the progress
     guard.
2. **Expansion bias.** mission60 summed -0.19 in 10.9 s, with no ground
   truth. The hold acts on a bias: about -0.38 deg nose-down, a push toward
   the scene.
   - Mitigations: b_max in D_low, measured by video in C1; margin term 2; the
     range-consistency stop rule; stop if b_max > 0.03 /s.
3. **LAND creeps toward the poster.**
   - During LAND the hold pitch moved nose-down from the hover mean:
     -0.16 → -0.58 deg on mission60 and +0.26 → -0.51 deg on mission5s.
   - That is +0.06 to +0.14 m by the lean model.
   - Mitigations: autocreep in every range threshold, margin term 3, and the
     iFwd restore. A real fix would be a behaviour change (decision 13).
4. **Forward/vertical coupling loop.**
   - Moving at 0.06-0.08 /s stays under the observer's 0.3 /s expansion gate
     (cam_vz_obs.c:40, 519). So the contaminated frames keep feeding the
     height estimate; the gate does not protect against this.
   - Mitigations: c_fv and c_vf measured in C0, k_low_eff, the wRef fallback
     (decision 6), and the height checked by video.
5. **Frozen 'ok' camera.**
   - The hold pushes from the reference alone, and the residual level error
     adds to it.
   - The editor's sweep: caught in 100 % of runs, but after 0.3-0.8 m of
     travel. The normal false-abort rate was 10-45 %.
   - Mitigations: L_fail in acceptance; C2a sets A from the real code and
     noise; the user decides if the false aborts stay high.
6. **The hold is a weak, lagging spring.**
   - Its gain falls as 1/D. A level error b gives an offset of D·b/KI (about
     0.5 m at 3.5 m for 0.3 deg).
   - Only 30-50 % of a move is done when the reference ends.
   - Mitigations: the level-point gate (decision 5), guards until touchdown,
     the stillness test and margin term 2.
7. **Hover drift at stage C distances is unmeasured.**
   - The 9-25 cm per 10 s is at 1.5-2.0 m and from the camera.
   - calib1 at 2.76 m drifted 45 cm left in about 4 s.
   - Mitigations: C0/C1 video at 3.0-4.5 m, the sideways guard, and 1.5 m
     clear on each side.
8. **Texture, light and patterns.**
   - A dim or blank view leads to NOT_STABLE or HOVER_NO_CAM.
   - Repeating patterns can fool the hold, and flicker can band the image.
   - Mitigations: the C0 texture and lum checks, and no dimmers.
9. **Time and battery.**
   - The hover wait starts at 5-6 s, the STEP must come by about 7.5-8.5 s of air,
     and the air limit is 18 s.
   - So one step per sortie and 1-2 sorties per charge.
   - Brown-out near vmin is a fall in place, hence the 4.10 V Mac rule.
10. **Claude's latency (5-20 s)** does not fit in flight. Decisions are made
    between sorties, and C5 is expected to be skipped.
11. **Yaw.**
    - A turn above about 20 deg resets the gain tracker (HOVER_NO_CAM), and
      the hold switches off above 0.35 rad/s.
    - Hence aiming by hand, and C6 capped at ±12 deg.
12. **Calibration of bearing and range.**
    - cx and the lens distortion are uncalibrated (1 px is 0.56 deg), and the
      sign is unverified in flight.
    - Markers may be cropped or poorly lit.
    - Mitigations: the C0 fit and sign check, 30 cm markers, and Claude's
      check of the annotated photo.
13. **Implementation budgets.**
    - The new code stays in the camera, link and aux tasks, and in the FSM's
      cheap comparisons. The instruction cache is shared with the stabilizer.
    - The recorder grows by about 60 KB in PSRAM.
    - The new packets need golden vectors and exact-size parsers.
    - Rebase onto ff2c3e3.
14. **Human factors.**
    - A reaction takes 0.3-0.5 s.
    - Judging depth along the line of sight is poor, hence the operator
      stands level with the kill line.
    - The phone stick gives LAND, which creeps, hence 'k' for a forward
      crossing.
15. **Small-sample evidence.** A few dozen sorties cannot show a low failure
    rate. The margin budget and the independent floors carry the argument.
16. **None of this transfers to a person (C7).**

---

## Appendix A: Changes from review

The critic's verdict was "revise before C1/C2". Each item below says what was
done; where the plan departs from the critic's wording, it gives the reason.

### Required changes

| Item | Status | What changed |
|---|---|---|
| R1, brake through the setpoint pitch | Applied, one deviation | Verified: setpoint roll and pitch are exactly 0 (auto_glue_core.h:200-218); sticksCentred reads pitchDes (camera_flow.c:325); a nonzero pitch zeroes camCorrPitch (controller_pid.c:266-269); the integrals reset after 1 s idle (camera_hold.c:63-70). The brake is now its own term after camCorrPitch, the setpoint stays 0, and a host test covers it (3.3). **Deviation:** sized from max(last good fwdF, wRef)·D_est, not fwdF·D_low. D_low under-reads on purpose, which would undersize the brake, and a frozen 'ok' camera reports fwdF ≈ 0 while the drone moves. The C2a sim bounds the backward overshoot. |
| R2, x_model and the 1 deg lean guard | Applied, one deviation | Reproduced: x_model reached -0.21 to +1.20 m in mission60's hover (1 s baseline); frozen camera 3.0 → 2.6 m ended at 1.89 m. (a) A progress check replaces x_model as the camera-failure guard (3.4, 3.5); the 1 deg lean guard is removed. (c) x_model is offline telemetry until it qualifies (C1). (d) L_fail includes detection, brake, glide and LAND (2.2). (b) **Deviation:** the time limits are computed on board, as a progress check at T_profile + 1 s and a settle cap at T_profile + 4 s, not a single T_profile + 1 s cap. The hold has done only 30-50 % of the move when the reference ends, so a T + 1 s total cap would abort every normal move. |
| R3, bias term in D_low | Applied | D_low = D_i·exp(-max(lnZoom,0)/k_low_eff - b_max·t - c_vf·\|Δh\|/D_i) (3.1). b_max starts at 0.02 /s (covers mission60's 0.017 /s), with a floor of 0.01 /s; C1 sets it from video; stop if > 0.03 /s. Tests include an additive bias, a descent and the mission60 replay, with mutations for both terms (C1). The old C1 creep criterion, which mission60 already failed, is replaced by this rule. |
| R4, LAND creep | Applied | (a) Every LAND-triggering range threshold is autodmin + autocreep, with creep measured by video in C0/C1. (b) iFwd is restored at every exit after a move, every LAND reason included, and before any coast (3.2). (c) Citation corrected: mission5s was +13.9 right / +6.8 forward at LAND start and +21 right / -16 forward at touchdown (RESUME.md). The plan now cites the lean-model creep instead (section 6, risk 3). |
| R5, margin budget | Applied | A written budget (2.1); D_stop = kill line 2.3 m + M (provisional 0.75 m, so 3.05 m), never below 2.8 m; compiled AUTO_MOVE_TARGET_MIN_CM 280 plus NVS autodtgt, enforced by the drone. |
| R6, far-scene flag | Applied, one deviation | D_G is telemetry only and the far-scene flag is gone. An in-flight k_flight check and a range-consistency check run on every moving sortie (C2c). **Deviation:** the stop rule uses k_flight's 95 % upper bound, or a running median over 3 or more sorties, not a single value. With ±3 % D_photo errors, one small step's k_flight is uncertain by ±30-40 %, so a single value would stop the campaign by chance. |
| R7, k per segment at hover height | Applied | k per 0.5 m segment; k_low = the minimum over segments, pushes and ±10 cm heights; true hover camera height measured by video against a height scale; the floor runner is part of the arena spec (C0, decision 3). |
| R8, coupling loop | Applied | c_fv and c_vf with signs; k_low_eff = k_low·(1 - c_fv·c_vf) if the loop is positive; C3's height check by video; the fallback agreed in advance (decision 6). The 0.3 /s gate is now listed as a risk, not a protection. |
| R9, guards until stopped | Applied, stricter | The guards stay on from STEP until **touchdown**, not just until the drone is still. SETTLE is a stillness test, and the post-move photo comes after it. **Deviation:** the stillness test is the 1 s change of lnZoom with thresholds of at least 3 sd of the measured noise, not \|fwdF\| < 0.01 /s for 1 s. With 0.002-0.004 per frame of noise, fwdF has an sd of about 0.02 /s, so that condition almost never holds. |
| R10, phases vs the acceptance rule | Applied | (a) C3 starts at 4.5 m with re-sized caps (autodmin 2.5, autodtgt 3.4, autostep 0.15). The feasibility table shows the accepted targets, and the room spec is a user decision. (b) The C4 exit is a window tied to D_stop. (c) C4b is dropped; raising the air limit is listed under decision 13 and not proposed. (d) The time rule is corrected (2.2, rule 8). |
| R11, hover drift at 3-4.5 m | Applied | (a) Drift by video at 3.0, 3.5, 4.0 and 4.5 m (C0/C1). (b) Session calibration with a level-point gate, versus fixed trims and a larger margin, is decision 5. (c) A sideways guard on board (latM > 0.5 m, using D_est) and 1.5 m clear on each side. |
| R12, split the phases | Applied | C1 is an offline replay against video with no flash. approach_est on board and 0x85 move to C2b. C2 splits into C2a (pure code plus sim), C2b (integration, DRY, shadow flights) and C2c (Mac runner, rehearsals). C4 is 2-3 sorties per approach, at least 6 approaches, with a review gate after 3. |
| R13, ABORT_MOVE and the pending slot | Applied | Verified: LAND gets BUSY_PENDING while a command is pending (auto_link.c:944-948), and KILL bypasses the slot (auto_link.c:1058). ABORT_MOVE now bypasses it like KILL. LAND keeps its flown path, and the Mac resends it every 100 ms until ACKed. Tests cover ABORT and LAND while a STEP or PHOTO is pending. |
| R14, σ once, ln off IRAM, travel defined | Applied | D_low does not re-apply σ to the Mac's dCons. ln() runs in the link task, and the FSM only compares. "Travel this sortie" is defined in 3.1. |
| R15, kill-line geometry | Applied | The operator stands level with the kill line, 2.5 m or more to the side, plus a side-view phone. 'k' for a forward crossing, the stick only for sideways drift (decision 4). |
| R16, C0 coverage | Applied | D_photo validated to 4.5 m; 30 cm markers (1.40 m apart); poster_locate timed; in-air decisions use the markers only, and NCC is offline. |
| R17, safety-case wording | Applied | Rewritten (section 5) to name the residual risks and their bounds, and to say the floor is estimate-based with 'k' as the physical backstop. The C4 exit ties to the budget, and the small-sample caveat is stated. |

### Missing items

| Missing item | Where it is now |
|---|---|
| Ground truth by video for drift, height, LAND creep and bias | C0 setup (side phone, height scale), C1 exit, C3 height |
| Additive bias and vertical terms in D_low, with tests that can fail | 3.1, C1 tests (R3) |
| A guard on whether the drone follows the reference | The progress check (3.4, 3.5), tuned in C2a |
| Glide after view loss or abort, and detection travel | L_fail in acceptance (2.2), the brake (3.3), C2a cases |
| iFwd restored on every exit and every coast after a move | 3.2 |
| A motion test in SETTLE; guards on until stopped | 3.4, 3.5 (until touchdown) |
| A firmware-enforced minimum target | AUTO_MOVE_TARGET_MIN_CM 280 plus autodtgt (2.2, 3.6) |
| A feasibility table and the room length per phase | 2.3, decision 3 |
| An in-flight k check with a stop rule | C2c (k_flight, range consistency) |
| k per 0.5 m segment at the true hover height | C0 |
| The coupling loop with signs, and the fallback | C0, decision 6 |
| Hover drift at 3-4.5 m, and a session calibration with a level-point gate | C0/C1, decision 5 |
| A sideways guard and corridor clearance | 3.5, decision 3 |
| ABORT_MOVE and LAND around the pending slot | 3.4, C2b tests |
| A corrected time formula, and a C4 without C4b | 2.2 rule 8, 2.4, C4 |
| Operator position and 'k' versus the stick | Decision 4, section 5 item 10 |
| A written margin budget that sets the stop | 2.1; the C4 exit |

### Other edits by the editor

- **Base updated** to ff2c3e3, with its unlock rule: `auto unlock` only after
  a reset done on purpose; after any other reset, save the logs and ask the
  user.
- **New finding from the editor's sweep.** A progress check that catches
  every frozen camera false-aborts 10-45 % of normal moves at today's
  frame-noise range. So C2a has an explicit false-abort criterion and a user
  decision if it fails.
  - Also: a stillness test alone would pass a frozen camera at once, since
    it is "still". So the stillness test also requires progress.
- **C5 marked as expected not to fit.** The corrected time rule leaves about
  1-2 s between the STABLE photo and the last moment a STEP can be sent.
- **STEP payload** shrinks to 12 B (no maxMs). NVS keys for D_G (autodvr,
  autodvok) are dropped, and autodtgt, autobmax, autocvf, autocreep and
  autoprog are added.
- **x_model** is no longer computed on board. It is offline telemetry from
  the recorder until it qualifies.
- **Every guard trip after a STEP now lands the drone.** Only the Mac's own
  ABORT_MOVE returns it to hover. That is simpler and more conservative than
  the merged plan's abort-to-hover.

## Appendix B: Conflicts between the three role plans (merge decisions, updated)

| Topic | Choice |
|---|---|
| Metric anchor | Markers (now 30 cm, 1.40 m apart); the poster-size match is an offline cross-check |
| D_G on board | Telemetry only (was: cross-check, far-scene flag, possibly an anchor) |
| Move law | Control's trapezoid with an error integral, safety's iFwd restore (now at every exit), no stop lead |
| Camera-independent travel bound | Replaced by the progress check plus the L_fail acceptance rule (was: lean-model x_model) |
| View lost during a move | Restore iFwd, then a separate brake term, then LAND if not back within 1 s |
| Status packet | One new push, 0x85 APPROACH_STATUS; AUTO_STATUS stays 64 B |
| Recording in C0 | `stream 100` plus `camvz air on`, no firmware change |
| YAW | Optional C6 |
| Hover wait | A STEP never extends it; corrected time rule |
| Step computation | Target-ratio form; σ once |
| Hover-only test distances | 3.0-4.5 m |
| Feedforward or stronger move gains | None (a move-only feedforward is only a fallback under decision 13) |

## Appendix C: Scripts behind the numbers

| Script | Shows |
|---|---|
| critic/xmodel_hover.py | x_model wander in hover (mission60: 1 s baseline -0.21 to +1.20 m; 3 s baseline sd 0.13-0.14 m) |
| critic/land_lean.py | hold pitch during LAND, and lean-model creep (+0.06 / +0.14 m) |
| critic/frozen_cam.py | the old rule's frozen-camera case (3.0 → 2.6 m ended at 1.89 m) |
| critic/accept_table.py | the old rule refused C3's 3.3 m target; C4 stuck at 2.60-2.72 m |
| critic/progress_guard.py | normal moves cover 30-50 % of the reference by its end |
| editor/guard_sim2.py and editor/guard_sweep.py | the revised guards: frozen camera caught 100 %, travel 0.3-0.8 m, normal false aborts 10-45 % (guard_sweep.out.txt) |
| editor/feasibility.py | the revised acceptance table (feasibility.out.txt) |
| control/move_sim.py and control/step_study.py | gentle-move overshoot (p95 +0.17 to +0.22 m, worst +0.36 m) |
| perception/results.txt | k and D_v by scene and distance |
