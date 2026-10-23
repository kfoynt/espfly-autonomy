# Autonomy A2b: bench tests and first hops, step by step

The acceptance procedure for stage A2b, for the two people in the session:

- **the user** handles the drone, the battery, the USB cable, the R button and the phone app, does the
  physical tests, says what they see and hear, and can say **stop** at any time;
- **Claude** runs every command on the Mac: the USB console (`tools/espfly_diag.py`), the Mac link tool
  (`tools/espfly_auto.py`) and the flashing.

The propellers stay **on** (they are hard to remove). The drone sits on a big soft mattress. Three phases, in
order, each only after the previous one passed:

- **Phase A: the motors cannot spin.** DRY sorties only: the vertical controller flies a simulated drone and
  the thrust is 0 at both barriers, so the props are harmless and the drone may be handled.
- **Phase B: the props spin below lift-off, hands clear.** BENCH sorties on the simulated drone with the
  thrust capped at 15000 (`gcap`). Lift-off needs about 32000-36000 on a full battery (hover about 40000:
  the hover gain `vzK` 165000 divided by the loaded battery volts), so the drone cannot lift. Nobody touches or
  holds the drone while any motor can spin; the user stands back with safety glasses.
- **Phase C: short autonomous hops over the mattress** (FLIGHT, 40 cm), only if every A and B check passed.
  Since 2026-10-12 (evening) the user also flies 60 cm with a 10 s wait (section C2).

Run every command from `Firmware/esp-drone`. Files go to `../../firmware-backups/cal/` (the backups folder
two levels up). `<date>` is the session date, e.g. `2026-10-12`. Short forms: a bare console command
(`auto status`, `gcap off`, `raw "camvz off"`, `status`, `flights`, `dump --out ...`) runs through
`python3 tools/espfly_diag.py`; `sortie`, `arm`, `disarm`, `land`, `photo`, `state`, `wait` and `events` run
through `python3 tools/espfly_auto.py`, and its `hello`, `status` and `kill` are always written in full.

---

## 0. How the session runs

### The rules that apply throughout

- **One console client at a time.** The USB port has no lock: two `espfly_diag.py` processes (or
  `espfly_auto.py install-key`, or the flasher) at once steal each other's replies. Claude never starts a
  console command while another runs, and runs `watch` only with `--seconds N` and nothing else on the console
  meanwhile. The Mac link tool (`espfly_auto.py`, over Wi-Fi) is separate and may run alongside.
- **Resets.** After any drone reset (`reboot`, R button, power-on, flashing, a panic) the console needs about
  3 s, and the Mac about 10-20 s to rejoin the drone's Wi-Fi. The old daemon has reported `drone_rebooted`
  and refuses every ARM: the user presses `q` in its tab, or Claude runs `kill <pid>` (the `pid` in
  `~/.espfly/auto.sock.json`). Claude waits for a one-shot HELLO, then starts the daemon again:

  ```
  for i in $(seq 15); do python3 tools/espfly_auto.py hello && break; sleep 3; done
  ```

  If it never succeeds, the user rejoins the drone's AP by hand (macOS may have joined another network).
- **Unlocking the reset-lock** (after any reset other than a battery power-on, BENCH and FLIGHT are refused
  `LOCKS`). Two ways, both a deliberate physical act by the user:
  - **`auto unlock` on the USB console** (user decision 2026-10-12, evening: the battery is hard to unplug).
    With USB in, the drone in IDLE and the motors stopped: `python3 tools/espfly_diag.py auto unlock` gives
    `OK auto unlock resetLocked=0 was=1 rst=software (other locks unchanged: plock=off rlocked=0 motorsReady=1
    estop=0 mguard=on mguardTripped=0)`, and `auto status` then `resetLocked=0` with `rst=` still the real
    reset reason. It clears only the reset-lock: a `1` among the other locks (or `plock=on`, `mguard=off`)
    still refuses BENCH and FLIGHT. Refused (`ERR busy: autonomy active`, `ERR auto unlock: ...`) while a
    sortie is active or a motor runs. It is a USB console command only: the Mac link tool has no unlock.
    **After a flash:** USB is plugged for flashing anyway; flash, wait for the boot (about 3 s),
    `auto unlock`, then unplug USB. After a `reboot` the same (`auto unlock` once it is back). After an
    **unexpected** reset (a brownout, panic or watchdog reset: `auto status` `rst=`), save `crash` and
    `flights` first and unlock only when the user says so.
  - **Battery power-on:** unplug the battery **and** USB for at least 5 s, plug the battery, then USB.
    `auto status` shows `resetLocked=0 rst=poweron`. With USB in, a battery swap is not a reset (USB powers
    the board).
- **Battery.** BENCH and FLIGHT need a resting battery of at least `vmin` (DRY is exempt): 3.80 V by
  default. The user lowered it from 4.00 V (3.85 V, then 3.80 V) on 2026-10-12, accepting the brown-out
  risk: hops from 3.93-3.96 V took the 3.3 V rail to 2.82-2.87 V. When `auto status` `vbat` is below,
  charge or swap the battery. Only the user changes `vmin`.
- **Never** `plock off`, `mguard off` or a lower `vmin` to get past a refusal.

### The Mac on the drone's Wi-Fi

The Mac's built-in Wi-Fi (`en0`) joins the drone's access point `ESP-DRONE_<MAC>` (password `12345678`). The
drone is 192.168.43.42; the Mac gets a 192.168.43.x address. Internet stays on the iPhone USB tether (iPhone
USB above Wi-Fi in the network service order; auto-join off for other known Wi-Fi networks during the
session). The phone app joins the same AP. Check: `python3 tools/espfly_auto.py up` gives `"ok": true` and
`"internet_separate": true`.

### The daemon (the Mac's link to the drone)

Claude starts it in a Terminal tab of the app, where the user can see it:

```
python3 tools/espfly_auto.py daemon                  # phases A and B
python3 tools/espfly_auto.py daemon --allow-flight   # phase C only
```

It says HELLO once and sends the heartbeat every 100 ms. It is never started in the background from
Claude's shell (it would have no keys); if that is ever unavoidable:
`nohup python3 tools/espfly_auto.py daemon --no-tty > daemon.log 2>&1 < /dev/null &` (closing a terminal then
stops nothing; `kill <pid>` lands and ends it). Its keys act only while **its tab has the keyboard focus**:

| Key | What it does |
|---|---|
| `k` | **KILL**: 10 KILL packets in 100 ms; the motors stop at once |
| `l` | LAND (slow, in place) |
| `s` | one status line |
| `q` | quit; if the drone is active, LAND first and wait up to 3 s for the touchdown |

Any `k`, `l` or `q` typed into that tab acts, so nothing else is typed there. Before every test in which a
motor can spin, the user clicks into the daemon tab and keeps the focus there. Claude's kill switch works
with any focus, and with the daemon dead or frozen: `python3 tools/espfly_auto.py kill`.

**What cuts the motors:** `k`, `kill`, and the drone's own trips (tilt > 30 deg, rate > 500 dps, free fall,
attitude error > 10 deg for 0.5 s, a motor fault). **What only lands, slowly:** `l`, `land`, `q`, closing
the daemon's tab, `kill <pid>`, the daemon dying or frozen (`HB_LOST` 1 s after its last heartbeat), any
phone stick input (`APP_INPUT`), and the phone app going silent for 0.5 s (`APP_SILENT`).

The daemon prints one JSON event per line (also in `~/.espfly/events.jsonl`): `state` (each transition with
its reason), `waiting`, `photo`, `kill`, `land_sent`, `sortie_abandoned`, `drone_rebooted`, `session_lost`,
... Photos go to `~/.espfly/photos/`.

### Claude's client commands (each prints one JSON line; the daemon must run)

| Command | What it does |
|---|---|
| `sortie --mode dry\|bench\|flight --height 40` | ARM + TAKEOFF; returns in the hover wait with the take-off photo on disk (`--photos 0..3`, default 1), or with `ok: false` and the reason if the sortie ended first. `--then land` lands once waiting and returns at IDLE. `--timeout S` (default 20) abandons it. FLIGHT also needs `--confirm-flight`. |
| `arm --mode ...`, `disarm`, `land` | one command each; `arm` shows the token, or the failed `guards` next to `vbat` and `vmin` |
| `photo --n 1 --gap 150` | in the hover wait: one more photo set (it does not extend the wait) |
| `state`, `wait --until waiting\|idle\|land`, `events --since N` | the latest AUTO_STATUS (and `daemon.pid`); wait for a state; the events |
| `kill` | KILL from a socket of its own, then asks the daemon to KILL too |
| `move --right/--left/--forward/--back M`, `--up/--down M` | the interactive hover (phase D): one MOVE in the hover wait (sideways and back <= 1.5 m, sideways also <= 0.5 x D; forward <= 4.0 m with the long-move build, 1.5 m before it, and only as far as ends >= 0.60 m from the scene ahead (the refusal gives the largest); the vector <= 4.0 m; <= 0.5 m vertical, and then forward <= 1.5 m); returns when it ended. `--person-gap G` (Claude's estimate of the end distance to the nearest person; refused under 1.0 m; logged with `--est`), `--batt-ok` (no battery-duration refusal), `--no-wait` (return once accepted; then `wait-move`). docs/autonomy-a2b.md section 21 |
| `wait-move [--photo]` | block until the running MOVE / YAW ends (at once if none): its end, `reached`, travel, target, D, the margins; `--photo`: a photo then |
| `sortie ... --plan-right/--plan-left/--plan-forward/--plan-back M [--plan-via drone\|mac]` | a move planned from the floor photo, flown from the hover wait's start (`--then wait`, a wait); the reply comes at the wait's start with the take-off photo and the plan (running, or refused with the reason and the largest forward move). `drone`: the 18-byte TAKEOFF (the long-move build only); `mac` (default): the daemon sends the MOVE then |
| `yaw --deg D` | phase D: turn (+ right / clockwise, as the photo bearings), <= 180 deg, at 30 deg/s (180 deg in 6.5 s, then a re-track bob) |
| `approach ...` | removed on 2026-10-15 (user decision P, a2b 22.9): a usage error now. Toward something seen in a photo: `yaw --deg` to its bearing, a photo, then one `move --forward F` (toward a person with `--person-gap G --est E`) |
| `stop` | phase D: end a running MOVE or YAW; the drone holds where it is (a re-track bob after a turn or a long sideways move runs on, <= ~8 s) |

`hello` and `status` also work without a daemon; while one runs they go through it.

### What an ARM needs (the guards)

Every ARM, DRY included:

- `autoon` 1 (`auto on`: an NVS write, IDLE only; default 0) and the daemon's heartbeat (`LINK`);
- the phone app connected and sending, with no stick input for the last 1 s (`APP`; setting `appreq`,
  default 1);
- the **consent gesture** (`GESTURE`; `autogest`, default 1): the user pushes the phone's roll stick past
  10 deg and releases it. Claude's ARM must come 1 to 15 s after the release; one gesture covers every ARM
  in those 15 s. In this document "gesture" before a command means: the user gestures, then Claude sends it;
- the drone on the ground 3 s, still and level (`STILL`; FLIGHT: 20 s since the last motor run, longer after
  a longer flight: 20 s + 1.5 x its motor time, at most 90 s, the accelerometer's heat soak; the `arm` reply
  then says 'IMU heat soak: wait N s'), no KILL in the last 2 s, enough heap (`HEAP`);
- the motor driver ready (not latched by a KILL), the motor watchdog on and not tripped, no emergency stop
  (`LOCKS`).

BENCH and FLIGHT also need: a battery power-on or an `auto unlock` since the last reset (the reset-lock, `LOCKS`; DRY is exempt;
the restart lock `plock` would add the same rule but is **off on this drone**) and the resting battery at or
above `vmin` 3.80 V (`BATT`). BENCH needs `gcap` 1..20000 (`GCAP`). FLIGHT needs `gcap off`, the camera
running with the drift hold on (`CAMERA`), the daemon started with `--allow-flight` and `--confirm-flight`.
FLIGHT and BENCH on the real IMU (`auto bench imu`) are also refused `LOCKS` while the camera height observer
is paused (`camvz off`) or in a bench mode (`camvz air`).

In IDLE, `auto status` and `state` list **FLIGHT's** failed guards; for a DRY or BENCH ARM read the `arm`
reply's `guards`.

### Latches and how to recover

| After | What stays | Recover |
|---|---|---|
| a KILL (`k`, `kill`) with any motor spinning | the motor driver's STOP: every ARM refused `LOCKS`, the phone throttle dead (`auto unlock` does not clear it) | dump the recorder first (a reset clears it), then `reboot` and `auto unlock`, or a battery power-on |
| a motor-watchdog trip (`mguard test hang\|imu`, or a real freeze) | `mguard status` `tripped=1`; ARMs refused `LOCKS`; more tests answer `ERR mguard test: guard already tripped (reboot)` (`auto unlock` does not clear it) | `reboot`, then `auto unlock` (with `plock` on: a battery power-on) |
| any reset other than a battery power-on | the reset-lock: BENCH and FLIGHT refused `LOCKS`, DRY allowed | `auto unlock` on the USB console (IDLE, motors stopped; user decision 2026-10-12), or a battery power-on. After an unexpected reset (not a flash, `reboot` or fault test done on purpose in this session; `auto status` `rst=`): save `crash` and `flights` and unlock only when the user says so |
| a KILL with the motors at 0 (DRY, IDLE), or a trip (KILLED) | nothing: KILLED goes to IDLE after at least 2 s, once the drone has rested 1 s | none |

`heapMin` and the recorder count from the last boot: read them before any reset.

---

## S. Setup (before phase A)

1. **Mac:** Wi-Fi as in section 0; `python3 tools/espfly_auto.py up`.
2. **Save the flight logs** (flashing can clear them), on the old firmware:

   ```
   python3 tools/espfly_diag.py flights > ../../firmware-backups/cal/flights-<date>-pre-a2b.txt
   python3 tools/espfly_diag.py crash > ../../firmware-backups/cal/crash-<date>-pre-a2b.txt
   ```

   Both files must be non-empty (`ls -l`); do not flash otherwise.
3. **Heap baseline** (skip if `status-<date>-m0.txt` exists): phone app and Mac connected; Claude:
   `python3 tools/espfly_auto.py photo`, then `python3 tools/espfly_auto.py status` (`status.heapMin`),
   `python3 tools/espfly_diag.py raw auto` and `python3 tools/espfly_diag.py status`; all three into
   `../../firmware-backups/cal/status-<date>-m0.txt`. If `heapMin` + 2.9 KB is under 14 KB, do not flash:
   the build needs the Wi-Fi RX IRAM change first (user sign-off).
4. **Flash** (Claude; app only, never the bootloader):

   ```
   python -m esptool --chip esp32s3 -p /dev/cu.usbmodemXXXX -b 921600 --before default_reset \
     --after hard_reset write_flash --flash_mode keep --flash_freq keep --flash_size keep \
     0x10000 ../../firmware-backups/espfly-a2b-<commit>.bin
   ```

   Rollback: the same with `../../firmware-backups/espfly-shadow2.bin`. Then `status`: `fw=` is the A2b build.
   A flash is a reset: every later flash of an A2b build ends, with USB still in, with `auto unlock` once
   the drone has booted (about 3 s; section 0), then USB out. (Here step 6's power-on clears it anyway.)
5. **Link key:** `python3 tools/espfly_diag.py raw autokey` gives `stored=` and `running=` with the same
   fingerprint, and after the power-on of step 6 `python3 tools/espfly_auto.py hello` gives `drone_fp` equal
   to `key_fp`. If not: `python3 tools/espfly_auto.py install-key` (a console client), then step 6 (the key is
   used after a reset). A wrong key shows as `no HELLO_ACK ... after 5 tries`.
6. **Power-on** (user), phone app connected, throttle at zero. Claude, after the Mac has rejoined:
   - `auto status`: `resetLocked=0`, `rst=poweron`, `heap=` at least 20 KB (record it: the heap at boot;
     expect about 1.3 KB less than 0cf6fbf gave, docs/autonomy-a2b.md section 2.6);
   - `raw auto`: `port=open`, `linkStack=psram`, `stackFree=` at least 2048;
   - `plock`: record it (expected `OK plock off rlocked=0`; if on, every reboot below also gives `rlocked=1`
     and the phone throttle stays at zero until a power-on);
   - `status`: `cam=running`, or (from 30 s after the power-on, autonomy idle) `cam=standby` with `camGnd=1`:
     the camera sleeps on the ground (docs/autonomy-a2b.md 17.1; `tkoff camwake 60` first if a step must see
     frames); `camClkChk=ok` (`wait` for the first ~10-15 s after the camera starts, with `camFps=0.0`);
     `hold`: `on`;
   - `gcap`: record the value, then `gcap off` (no BENCH ARM can pass in phase A).

---

## Phase A: the motors cannot spin (DRY only)

### A1. Link and daemon (Claude)

1. `auto status`: `guards=` includes `LINK` (no daemon yet).
2. `python3 tools/espfly_auto.py status`: its `auto` part shows `stateName` IDLE.
3. Claude starts the daemon in its tab. `python3 tools/espfly_auto.py status` now has `"via": "daemon"`;
   the daemon's events show one `hello`. `raw auto`: `kills=0`. `auto status`: no `LINK`.

### A2. Settings and guards

1. `auto get`: `autoon=0 appreq=1 autogest=1 vmin=3800 autoair=6000 autowait=5000 autoaccel=2000
   vzK=165000` (or the last learned value). This also saves them for the Mac tool (its ARM reply shows
   `vmin`, and it checks `--air-ms` against `autoair`).
2. Gesture; `arm --mode dry`: refused `GUARDS`, `ENABLED` among the guards.
3. `auto on`: `OK auto on stored=nvs`.
4. No gesture for 15 s; `arm --mode dry`: `GUARDS` `[GESTURE]`.
5. User: close the phone app. `arm --mode dry`: `APP` among the guards. User: reopen it, connect, throttle at
   zero.
6. Gesture; `arm --mode dry`: `OK` with a token; `disarm`: `OK`, IDLE `DISARM_CMD`.
7. Gesture; `arm --mode bench`: `GUARDS` with `GCAP`.
8. **Battery rule:** `auto set vmin 4200`; gesture; `arm --mode bench`: `BATT` among the guards (skip if the
   resting battery reads 4.20 V); `arm --mode dry`: `OK` (DRY is exempt), then `disarm`. `auto reset vmin`
   (`vmin=3800(default)`), then `auto get` again.
9. **Camera-observer lock** (needs `resetLocked=0`; with `gcap off` every BENCH ARM also shows `GCAP`, so
   none is accepted). Gesture before each `arm`:

   | Console | `arm --mode bench` | `arm --mode dry` |
   |---|---|---|
   | `auto bench imu`, `raw "camvz off"` | `LOCKS` among the guards | `OK`, then `disarm` |
   | `raw "camvz on"` | no `LOCKS` | |
   | `raw "camvz air on"` | `LOCKS` | |
   | `raw "camvz air off"`, `auto bench sim`, `raw "camvz off"` | no `LOCKS` (the sim does not use the observer) | |

   Finish with `raw "camvz on"`.

**A2.10. Busy gates.** Gesture; `arm --mode dry` (`OK`; ARMED lasts 10 s). At once Claude runs this (each
command is harmless if ARMED has lapsed):

```
python3 - <<'EOF'
import sys; sys.path.insert(0, "tools"); import espfly_diag as dg
d = dg.Drone(dg.find_port())
try:
    for c in ("cal save", "trim save", "facc save", "atrim save", "autokey x", "auto set autowait 5000",
              "auto on", "camvz air spin", "camvz batch 1", "gcap off", "mguard test hang 1"):
        r = d.command(c)[0]
        print(f"{c:24s} -> {r}")
        if r != "ERR busy: autonomy active":
            sys.exit("not busy: ARMED lapsed? arm again and repeat")
finally:
    d.close()
EOF
```

Every line answers `ERR busy: autonomy active`. Then `disarm`; `auto prof`: `flashRefused=0`.

### A3. Reset-lock and restart lock

1. `reboot`. After the reset: `auto status` `resetLocked=1`; `plock`: `rlocked=0` (1 if `plock` is on).
   Rejoin and restart the daemon (section 0). Gesture; `arm --mode dry`: `OK`, then `disarm`;
   `arm --mode bench`: `LOCKS` (and `GCAP`) among the guards.
2. `raw "reboot rwdt"`: `resetLocked=1`.
3. The R button: `resetLocked=1` (a power-on reset, but RTC memory keeps the lock's record).
4. Battery out and in, USB connected: no reset (`status` `up_ms=` keeps counting); `resetLocked` unchanged.
5. Battery power-on: `resetLocked=0 rst=poweron`. Rejoin, restart the daemon.
6. **USB unlock** (new 2026-10-12, user decision; once, on the first build that has it). `reboot`; rejoin,
   restart the daemon; `auto status` `resetLocked=1 rst=software`. Gesture; `arm --mode dry` (`OK`), and at
   once `auto unlock`: `ERR busy: autonomy active`; `auto status` `resetLocked=1`. `disarm`; then
   `auto unlock`: `OK auto unlock resetLocked=0 was=1 rst=software (other locks unchanged: plock=off rlocked=0
   motorsReady=1 estop=0 mguard=on mguardTripped=0)`; `auto status`: `resetLocked=0 rst=software`; `auto rtc`:
   `locked=0`. `auto unlock` again: `was=0`. Gesture; `arm --mode bench`: no `LOCKS` among the guards (`GCAP`
   with `gcap off`: expected). `reboot`: `resetLocked=1` again, `auto rtc` `prevLocked=2`. Finish with
   `auto unlock` (or a battery power-on).

If any check gives `resetLocked=0` where 1 is expected: **stop**, save `auto rtc` and `auto status`; no
phase B or C. (`mguard test panic`, another reset, is at the end of phase B: it needs `gcap`.)

### A4. GO / NO-GO: the loop rate

The camera height observer must run its in-flight path, which a DRY sortie alone never feeds.

1. `raw "camvz air full"` (IDLE; it does not lock DRY), then `auto prof reset`.
2. Gesture; then Claude, in one line (two loop-counter reads 3 s apart in the 5 s hover wait):

   ```
   python3 tools/espfly_auto.py sortie --mode dry --height 40 --photos 0 && python3 tools/espfly_diag.py raw "camvz prof" && sleep 3 && python3 tools/espfly_diag.py raw "camvz prof"
   ```

3. `python3 tools/espfly_auto.py wait --until idle`, `sleep 6`, then `auto prof`.
4. `raw "camvz air off"`.

| Check | Needs |
|---|---|
| `auto prof` `activeLoopsPerS` (the tool prints `go/no-go: GO`) | >= 940 |
| the two `camvz prof` reads: (`loops` difference) / (`upMs` difference) x 1000 | >= 940 |
| `auto prof` `stabStackFree` (sampled in IDLE every 5 s and on the first IDLE step after a sortie) | >= 1024 B |

Only the loop rate gates. `activeAvgUs` and `activeMaxUs` are wall-clock times that include preemption: the
tool shows them as information. **Below 940 loops/s: stop.** No BENCH, no FLIGHT until a fallback of the A2b
plan (section 2.6) is in, which needs the user's sign-off.

The gate was 975 in the plan. **The user signed off 940 on 2026-10-12, before phases B and C**, on that day's
measurements with the Mac link (daemon running), the phone link and photo pushes: the first A2b build ran
630-700 loops/s during sorties with its per-loop autonomy code in flash; with that code in IRAM, 948-973 loops/s
in TAKEOFF/HOVER/LAND against 983-990 in IDLE; manual flights flew well at ~957 (2026-10-11). **Any further
lowering of the gate needs the user's sign-off again.** (docs/autonomy-a2b.md sections 2.6 and 11.)

What the number measures: a DRY sortie, so the vertical controller runs on the sim plant (`vzSimStep`, `expf`),
and `camvz air full` adds the observer task's in-flight path. The FLIGHT path's per-loop calls
(`camVzGlueVzInput`, `sensfusion6GetAccZ`, `camVzGlueSetLiftoffHint`) are not timed here; they are in IRAM like
the DRY path's (checked in the disassembly, docs/autonomy-a2b.md section 2.6). The ARMED loops run the full guard
check on every loop.

### A4b. Bisect the loop rate (when A4 reads under the target)

docs/autonomy-a2b.md 19.11. The build since the loop-rate fix has four RAM-only toggles
(`utils/interface/bench_bypass.h`); each puts one per-loop addition of a2b section 19 back to 14bcfc6's code.
They are 0 at every boot; `status` and `auto prof` show `bypass=0x..`. Set them only in IDLE with the motors
stopped (the firmware refuses otherwise), one A4 run per state. `auto prof` also shows `bypassSeen=0x..`: every
bit on at any time since `auto prof reset`, the state its loop counters were measured under (a bit cleared before
the read, or sorties with and without it and no reset between them, stay in it); the tool's go/no-go needs it 0:

| Command (`python3 tools/espfly_diag.py raw "..."` or `espfly_diag.py bypass ...`) | Bit | What goes back to 14bcfc6 |
|---|---|---|
| `bypass gtc on` | 0x01 | the gyro temperature compensation's whole path (no accumulate, 20 Hz step, subtraction, latch or log) |
| `bypass mki on` | 0x02 | the attitude filter's in-flight learning (no inputs, the old integral branch) |
| `bypass rec on` | 0x04 | the recorder's 7 thermal / exposure fields (written 0) and the post-flight re-zero poll |
| `bypass flow on` | 0x08 | the flow fit forced plain, no exposure gate (NVS `flowfit` untouched) |
| `bypass all on` / `bypass all off` | 0x0F / 0 | all of them / none |

1. `bypass` (shows `OK bypass=0x00 ...`); `flowfit` (note the mode); `status` `camClk` 13.33.
2. The A4 run as written (steps 1-4) with `bypass=0x00`: the build's number and its go/no-go.
3. For each of `gtc`, `mki`, `rec`, `flow`: `bypass <name> on`, the A4 run (its `auto prof reset` comes after
   the bit is set), `bypass <name> off`. Then `bypass all on`, the A4 run, `bypass all off`. `auto prof` prints
   the rate with `bypass <names>` and no go/no-go while `bypassSeen` is not 0 (read `bypassSeen=` to be sure it
   names the bit of this run and no other). Each bit's cost is (its run) - (step 2's run). `all on` compares
   with 14bcfc6's 961 plus the review round's two levers (the recorder's `sat16` out of line, `usecTimestamp`
   in IRAM: ~+8..11 by the model, costs 14bcfc6 also had): ~966-972 if the toggles cover the regression; what
   it falls short of that is outside the toggles (flash layout; E/F/G; `vbatMonitorTask`'s larger critical
   section, a2b 19.11 review round) or the levers' model. Before step 4 or any go/no-go: `bypass all off`, then
   `auto prof reset` (a run after it reads `bypassSeen=0x00`).
4. Camera budget per fit mode, `bypass=0x00`, facing the room at 13.33 MHz, `camClkFallback=0`: `status`
   `camProcAvgUs` after >= 30 s awake with `flowfit` robust, then lite, then plain (each `flowfit` change is
   saved; the EMA needs ~20 s). Keep robust / lite only at <= 60500; otherwise `flowfit plain` and note it
   (a2b 19.11, D). If the clock already fell back (`camClk` 20.00), reboot for each mode (I3's steps).
5. `reboot` (or `bypass all off`) and `status` `bypass=0x00` before any BENCH or FLIGHT.

| Reading | Means |
|---|---|
| `gtc` on gives back >= 5 loops/s | the compensation's per-sample path still costs: report the number |
| `all on` ~966-972 (961 plus the review round's levers) | the toggles cover the regression; the rest is layout |
| `all on` at or under 961 | a cost outside the toggles (flash layout, E/F/G, `vbatMonitorTask`, the camera) or the levers gave less than modelled: report with `camProcAvgUs` |
| `bypassSeen` names a bit the run did not set | a bit from an earlier run (no `auto prof reset` after it): `bypass all off`, `auto prof reset`, repeat the run |
| robust / lite `camProcAvgUs` > 60500 | fly `flowfit plain` at 13.33 MHz (or 20 MHz with robust): report |

A toggle never adds code to a flight path: every bit reverts to code that flew on 14bcfc6. Clear them anyway
(the go/no-go is for `bypass=0x00`).

### A5. DRY: one full sortie

1. `flights`: note `n=`.
2. Gesture; `sortie --mode dry --height 40` (one take-off photo).
3. Expect in the events: `state` ARMED, TAKEOFF (the red LED for 1 s: the pre-spin), HOVER; `photo` and
   `waiting` (either order); LAND `HOVER_TIMEOUT` 5 s after `waiting`; IDLE `TOUCHDOWN`. The `photo` event's
   `t` is before the LAND event's, and the `sortie` reply lists the PNG. `state` in the wait shows `sim` in
   `auto.flagNames`.
4. After IDLE: `dump --out ../../firmware-backups/cal/flight-<date>-dry.csv`: "autonomy with no thrust
   (DRY)", the state timeline, "motors got up to 0". `flights`: `n=` unchanged (DRY makes no summary).

### A6. DRY: stops, commands and trips in the hover wait

Each row is a fresh `sortie --mode dry --height 40` (gesture first) that returns in the wait; then the test.

| Test | Who | Expect |
|---|---|---|
| `photo --n 1 --gap 150` | Claude | `OK` with an id, the PNG arrives; `waitLeftMs` keeps counting down |
| a second `photo` right after it | Claude | it waits for the first set's PHOTO_FREE, then works; `raw auto` `photoHeldBusy=0` |
| `land` | Claude | LAND `LAND_CMD`, IDLE `TOUCHDOWN` |
| throttle stick up (then back to zero) | user | LAND `APP_INPUT`, never KILLED. Then `dump --out ../../firmware-backups/cal/flight-<date>-dry-stick.csv`: `rawThrust` rises and `autoState` becomes 4 (LAND) within 2 rows (20 ms). A 240 fps video of thumb and drone LED gives the end-to-end time (target 200 ms). |
| close the phone app | user | LAND `APP_SILENT` about 0.5 s later; reopen the app |
| `python3 tools/espfly_auto.py status` | Claude | the sortie goes on (no LAND `SESSION`) |
| `q` in the daemon tab | user | LAND `LAND_CMD`; the daemon exits after the touchdown; Claude restarts it |
| close the daemon's tab | user | as `q` (SIGHUP), not `HB_LOST`; Claude restarts it in a new tab |
| `kill -9 <pid>` (`daemon.pid` in `state`) | Claude | LAND `HB_LOST` 1.0 s after the last heartbeat; restart the daemon |
| `kill -STOP <pid>`; `kill -CONT <pid>` 3 s later | Claude | LAND `HB_LOST` |
| instead of the plain sortie: `sortie --mode dry --timeout 0.6` | Claude | `sortie_abandoned`; no TAKEOFF after it; IDLE `LAND_CMD` from the pre-spin |
| instead: `sortie --mode dry --timeout 2.5` | Claude | `sortie_abandoned`; LAND `LAND_CMD` in the climb |
| `k` in the daemon tab | user | KILLED `KILL_CMD`; an `arm` before IDLE is refused `BAD_STATE`; IDLE `KILL_CLEARED` after at least 2 s, once it has rested 1 s |
| `python3 tools/espfly_auto.py kill` | Claude | the same; the reply has `ok: true` and `state` KILLED |
| in IDLE: `python3 tools/espfly_auto.py kill; python3 tools/espfly_auto.py arm --mode dry` | Claude | `KILL_LATCHED` (an ARM within 2 s of a KILL) |
| **hand tilt**: the user lifts one side of the drone and tilts it firmly past 30 deg within about 0.3 s, then puts it down and leaves it still | user | KILLED `TILT` (a slow tilt gives `ATT_ERR`, a flick `RATE`: all three are cuts). The trips read the real IMU; in DRY the motors never spin. IDLE once it rests. |

Only `k` and `kill` (and the trips) cut; everything else above lands. `HB_LOST` comes from the drone, so the
same rows prove the drone side of a dead Mac.

---

## Phase B: the props spin below lift-off, hands clear

Only after every phase A check passed. The drone sits on the mattress, props on. Nobody touches the drone
while a motor can spin; the user stands back with safety glasses, holds the phone, and keeps the focus in the
daemon tab. The red LED means the motors start within 1 s.

Setup (Claude): `gcap 15000` (`OK gcap 15000 stored=nvs`); `auto status`: `resetLocked=0` (else read
`rst=`: after a reset done on purpose in this session (a flash, `reboot`, `mguard test panic`),
`auto unlock`; after any other reset, save `crash` and `flights` and ask the user before `auto unlock`,
section 0), `benchSrc=sim`, `vbat` at least `vmin` (3.80 V); `raw "camvz air off"`.

A BENCH sortie on the mattress may rock: the attitude control pushes against the soft surface while the
collective thrust is capped. A tilt past 30 deg cuts it (KILLED `TILT`); that is safe, note it and go on.

### B1. A BENCH sortie with a photo

1. `tkoff`: note `camsb=` (should be 1), `sbCount=`, `takeoffs=`.
2. Gesture; then Claude (the console reads `live` 10 times a second while the sortie runs on Wi-Fi):

   ```
   python3 tools/espfly_diag.py watch --hz 10 --seconds 25 > ../../firmware-backups/cal/watch-<date>-bench.txt 2>&1 &
   sleep 2; python3 tools/espfly_auto.py sortie --mode bench --height 40 --then land; wait
   ```

3. Expect: the props spin up after the red LED, stay at the cap through the sim's climb and hover, and stop at
   the sim's touchdown (IDLE `TOUCHDOWN`); the photo arrives in the wait, before the LAND.
   - `watch` `thrust=` (the collective thrust after every clamp) never above 15000. The per-motor `motors=`
     percentages can read a little higher (the mixer adds the attitude terms); AUTO_STATUS `thrust` and the
     recorder's `vzThrust` are the controller's command before the clamp and read above 15000 by design.
   - `tkoff`: `takeoffs` +1, `window=0`, `sbCount` unchanged (the camera stayed awake). When the window
     ended (1.5 s after the sim's lift-off) is not visible on the console; the host tests cover it.
4. Right away, before any reset: `auto status`: `heapMin` at least 14 KB (the minimum since boot, with phone,
   Mac and photos). `sleep 6`, then `auto prof`: `stabStackFree` at least 1024.
5. `dump --out ../../firmware-backups/cal/flight-<date>-bench.csv`: `thrust` at most 15000, "sim plant".

### B2-B5. Stops with the props spinning

Each is a fresh `sortie --mode bench --height 40` (gesture first) that returns in the wait; then the stop.

| Test | Who | Expect |
|---|---|---|
| B2: roll or pitch stick (not the throttle) | user | LAND `APP_INPUT`, never KILLED; the props stay at the cap and stop at the sim's touchdown |
| B3: `kill -9 <pid>` | Claude | LAND `HB_LOST` 1 s later; the props stop at the sim's touchdown; restart the daemon |
| B4: `k` in the daemon tab | user | KILLED `KILL_CMD`; the props stop at once |
| B5: `python3 tools/espfly_auto.py kill` | Claude | the same; the reply has `state` KILLED |

After B3: `raw "mguard status"`: `tripped=0`; record `maxStallUs`, `lateCalls`, `maxCallGapUs` (no false trip
under the full load).

After B4 and after B5: once IDLE (`KILL_CLEARED`),
`dump --out ../../firmware-backups/cal/flight-<date>-kill.csv` (and `-kill2.csv`): the first row with
`autoState` 5 (KILLED), or the next one (rows are 10 ms apart), has `m1`..`m4` all 0 (duty 0 well within
50 ms). The motor driver is now latched: `reboot` and `auto unlock` (or a battery power-on), rejoin, restart
the daemon.

### B6. Motor watchdog fault injection (last: each one latches)

These run in manual mode (autonomy IDLE), where no tilt trip cuts: the user keeps the throttle just above
spin-up, and pulls it to zero at once if the drone rocks or tips.

1. User, hands clear: raise the phone throttle until the props spin (`gcap` holds it at 15000; the drone
   stays down). Claude at once: `raw "mguard test hang 200"`: `OK ... motorsOn=1`; the props stop.
   `raw "mguard status"`: `tripped=1`, `maxStallUs` 80000-90000 (the cut came 80-90 ms into the freeze).
   User: throttle to zero. Claude: `reboot`; after it, `raw "mguard status"`: `tripped=0`; the phone app
   reconnects.
2. The same with `raw "mguard test imu 200"` (no IMU data for 200 ms).
3. `raw "mguard test panic"` (motors off): the drone aborts and restarts. `crash` shows the abort;
   `auto status`: `resetLocked=1`.
4. `auto unlock` (or a battery power-on); rejoin; restart the daemon.

A `k` cannot be timed against a frozen stabilizer by hand: the watchdog always cuts first, and that 80-90 ms
cut is what this step measures.

---

## Phase C: short autonomous hops over the mattress

Only if every phase A and B check passed and the battery rests at `vmin` (3.80 V) or more.

**Setup.**

- The room is clear; the mattress lies flat, with a patterned sheet or blanket on it (the downward camera
  needs texture for the drift hold and the height estimate); the drone in the middle, level.
- The user stands 1.5-2 m away at the Mac, with safety glasses, the phone app in front and awake (a silent
  app lands the drone), throttle at zero, and the focus in the daemon tab.
- Claude: `gcap off` (FLIGHT is refused `GCAP` while it is set); `raw "camvz air off"`; `raw "camvz on"`;
  `hold`: `on`; `status`: `cam=running`, or `cam=standby` with `camGnd=1` (the ground standby: the ARM wakes
  it), and `camClkChk=ok`; `auto status`: `resetLocked=0`, `vbat` at least 4.00;
  `auto set autowait 0`.
- The plain daemon ends (`q` in its tab, or Claude's `kill <pid>`); Claude starts
  `python3 tools/espfly_auto.py daemon --allow-flight` in its tab.
- 20 s with no motor running (FLIGHT's `STILL` rule) and nobody touching the drone. Then `state`:
  `auto.guards` is `["GESTURE"]` only (or empty), or `["GESTURE","CAMERA"]` while the camera sleeps on the ground
  (`status` `camGnd=1`; in IDLE the guards are checked as for a FLIGHT ARM, and G4 needs a streaming camera):
  the ARM wakes it and the daemon resends the ARM (event `arm_cam_wake`).

**A hop.** Claude says "gesture now"; the user gestures; Claude:

```
python3 tools/espfly_auto.py sortie --mode flight --confirm-flight --height 40 --photos 1
```

| Hop | `autowait` before it | Expect |
|---|---|---|
| 1 | 0 | the drone lands right after the photo: the reply is `ok: false` with "ended before the hover wait: LAND (SORTIE_DONE)", which is the plan. `wait --until idle --timeout 15`. |
| 2 | `auto set autowait 2000` | the reply in the wait (`ok: true`); LAND `HOVER_TIMEOUT` 2 s later |
| 3 and on | `auto set autowait 5000` | in the wait Claude may read the PNG, ask for a `photo`, then `land` |

**What the user watches:** a straight climb to about 40 cm; a still hover (the camera drift hold engages);
the photo arrives (`photo` event; in hop 1 it can come during the descent); a slow descent (about 0.3 m/s,
slowing to 0.2 m/s near the take-off height) and the motors stopping at the first touch: no bounce under
power, no lean after contact. If the touchdown is detected late (a soft surface), the controller cuts after 6 s
of LAND (`LAND_TIMEOUT`), or the state machine kills at 6.5 s (`LAND_BACKSTOP`). Both stop the motors at the
surface; note which happened.

**Stop rules during a hop.**

- Any tilt, wobble or drift toward an edge: **phone stick** (slow LAND).
- Climbing past about 70 cm, fast drift, oscillation, a strange motor sound, anything unexpected: **`k`**
  (the drone drops onto the mattress). After a `k` with the motors on: dump first, then `reboot` and
  `auto unlock` (or a battery power-on).
- Stop hopping and report after any KILLED that was not `k`, a drone reset (`session_lost` or
  `drone_rebooted`: save `crash` and `flights`), or a LAND reason other than `SORTIE_DONE`, `HOVER_TIMEOUT`,
  `LAND_CMD`, `APP_INPUT` or `HOVER_NO_CAM` (the camera height not ready 2 s into the hover: a safe LAND,
  record it).

**After each hop** (Claude, one console command at a time; `N` is the hop number):

```
python3 tools/espfly_auto.py events > ../../firmware-backups/cal/hop<N>-<date>-events.json
python3 tools/espfly_diag.py dump --out ../../firmware-backups/cal/flight-<date>-hop<N>.csv
python3 tools/espfly_diag.py flights > ../../firmware-backups/cal/flights-<date>-hop<N>.txt
python3 tools/espfly_diag.py auto status > ../../firmware-backups/cal/auto-<date>-hop<N>.txt
python3 tools/espfly_diag.py auto rtc >> ../../firmware-backups/cal/auto-<date>-hop<N>.txt
python3 tools/espfly_diag.py camvz >> ../../firmware-backups/cal/auto-<date>-hop<N>.txt
python3 tools/espfly_diag.py auto get >> ../../firmware-backups/cal/auto-<date>-hop<N>.txt
```

The dump's analysis gives the state timeline, the highest height estimate and the flags seen (`hold`,
`camHeight` expected); `auto get` shows whether a learned `vzK` was stored. Before the next hop: the battery
still rests at `vmin` (3.80 V) or more, and 20 s without motors have passed.

These hops are also the first measurements of what the bench cannot show: the drift hold engaging in an
autonomous hover, the camera height observer's time to its first healthy sample with the camera awake (the
dump and `camvz`), and the supply-rail minimum of an autonomous take-off (`flights`).

**The take-off integrator hold** (user decision 2026-10-13, evening; a2b 2.4), on the first autonomous
take-off of a build that has it: in the dump, `rollI_cdps`, `pitchI_cdps`, `rollRateI` and `pitchRateI` are
exactly 0 on every record from the spin-up to the last one before `vzPhase` 3, including the ~35-75 records at
26000 thrust or more, and nonzero right after it (any nonzero value above 26000 before `vzPhase` 3: stop and
report). Roll should not swing past level after the lift-off (about 1 deg at most, against 2.5-3 deg from a
tilted floor before); expect a pitch dip of about 1 deg for ~0.4 s (the stored torque trim misses the hover
need; the rate integrators build it from 0). Their mean over lift-off +1..2 s is that miss: the user may set
the trim from it afterwards (`trim set`, `trim save`), as a separate step. A take-off that ends in a LAND
before `vzPhase` 3 (`NO_LIFTOFF_TIME`, `NO_LIFTOFF_CEIL` or another abort) flew without the integrators until
its RAMP reached the plateau (the hover thrust of the stored gain; every logged lift-off came 0.39 s or more
before it) and with them from there on: if the drone was airborne undetected (a negative in-flight
accelerometer offset), that was on P and D alone for up to ~0.9 s (sim: up to 5.6 cm high, a steady lean of
~1.3-2.5 deg; before the bound up to 3.75 s through the float and the abort LAND). In the dump the four I
terms are 0 up to the plateau (the thrust stops rising) and nonzero after it; report such a sortie.

### C2. 60 cm and a 10 s wait (user decision 2026-10-12, evening)

After the first full mission at 40 cm (~0.32 m true, a 5 s wait, `HOVER_TIMEOUT`, 9.4 s in the air, battery
4.15 -> 3.49 V in flight, 3.3 V rail down to 3.14 V) the user asked for one more sortie, higher (50-60 cm)
with a 10 s wait. It needs the firmware with the raised envelope (docs/autonomy-a2.md revision 5:
`--height` up to 60, `autowait` up to 12000, `autoair` default 6000 and up to 8000, the 18 s air clock).

**Settings** (Claude, over USB, in IDLE):

- `auto set autowait 10000` (the drone has 5000).
- `auto get`: `autoair=6000(default)`. A stored lower value (`(nvs)`, e.g. 4500 from before) leaves the
  0.60 m sortie less than 1 s of margin before its wait: `auto reset autoair` (or `auto set autoair 6000`).
- The battery fully charged: at least 4.10 V resting (`auto status` `vbat`). The sortie is ~15-17 s in the
  air, ~1.7 x the 40 cm mission.
- Everything else as for the hops above (`gcap off`, camera and hold on, 20 s still, the `--allow-flight`
  daemon, the gesture).

**The sortie:**

```
python3 tools/espfly_auto.py sortie --mode flight --confirm-flight --height 60 --photos 1
python3 tools/espfly_auto.py wait --until idle --timeout 30
```

**Expect** (sim and the 40 cm logs): the first thrust ~1 s after TAKEOFF (pre-spin), HOVER ~2.7-3.0 s after it,
the photo and the wait at an air clock of ~4.3-4.8 s (the limit is 6 s), a hover at about 0.5 m (sim
0.45-0.6 m; the drone flew 0.32 m at 40 cm), `HOVER_TIMEOUT` at an air clock of ~14.3-14.8 s (the hard limit
is 18 s), then the slow LAND, ~2-3 s from 0.5 m, motors off at the first touch. About 15.5-17 s in the air.
Battery: about 1.7 x the mission's use, so ~0.2 V of resting voltage (4.15 -> ~3.93-3.97 V after it
recovers), the in-flight 10 Hz reading down to ~3.40-3.45 V near the end of the wait, the 1 kHz minimum
~3.2-3.25 V, the 3.3 V rail ~3.0-3.05 V (estimates, scaled from the mission).

**Watch for** (the camera study's verdict was a ~5 s wait at ~0.5 m; this one is 10 s): the hover drifting
slowly up or down during the wait (the camera's speed offset: 0.03 m/s is 0.3 m over 10 s). The 70 cm `k`
rule above was set for 40 cm hops; for this sortie the user sets the height that calls for `k` before the
flight (the sim's hover peaks at 0.6-1.0 m with the drift; a slow steady drift is a phone-stick slow LAND, a
fast climb a `k`). For that height: if the camera height estimate never comes up (the observer not healthy;
`HOVER_NO_CAM` lands it 2 s into the hover), the accelerometer alone flies the climb, and in the sim's worst
corner (-20 mg offset, a stored gain 12 % high) the drone climbs on to **2.8-3.0 m true and comes down at up to
2.6 m/s** before that LAND, about 5 s after the first RAMP step (40 cm: 2.2-2.3 m; the uniform draw at 60 cm:
up to 2.56 m; the realistic range: up to 0.86 m; docs/autonomy-a2.md 4.9, revision 5). Nothing but `k` stops
it sooner: a climb past the chosen height without slowing is a `k`.

### D. The interactive hover (user decision 2026-10-12, late)

The user leaves the drone on a flat spot; it takes off to ~1 m, takes one photo and waits; the user gives
instructions in the chat ("move left 0.5 m", "turn right 90", "go close to the chair, 1 m from it") and Claude
flies them one at a time (docs/autonomy-a2b.md section 14). It needs the interactive-hover firmware (the user
flashes it; then `auto unlock` over USB as after any flash) and everything of the hops above: `gcap off`,
camera and hold on, 20 s still, the battery at or above `vmin`, the gesture.

**In-air contact (user decision 2026-10-13: "try a slow landing"):** a contact now lands (LAND `COLLISION`,
see "People in the view" below) instead of cutting the motors. Once per new firmware, props off: `sortie --mode
dry --height 100 --photos 1 --then wait --wait-s 30`, and in the hover wait a firm tap on the frame must give
event `state` to LAND, reason `COLLISION` (not KILLED), then the sim's slow descent and IDLE `TOUCHDOWN`
(~7 s; `test_autonomy_glue_host` `collision-dry`).

**The spot:** flat, ~2 m clear around and ahead, >= 1.6 m headroom, a textured scene ahead within ~1-4 m (the
camera measures its distance D; a blank wall or open space gives none, and every horizontal move is then
refused `NO_DIST`). Nobody within 2 m of the drone's path.

**The user:** starts nothing; clicks into the daemon tab and keeps the focus there. `k` cuts the motors at once
(any time something looks wrong: a fast climb, a lean, the drone heading for something), `l` lands, any phone
stick input lands.

**Claude:**

```
python3 tools/espfly_auto.py daemon --allow-flight            # in the user's Terminal tab
python3 tools/espfly_auto.py sortie --mode flight --confirm-flight --height 100 --photos 1 --then wait --wait-s 60
# (--wait-s 1..120 since 2026-10-14; 72 s and more need --height 105)
# then, per instruction (each within the wait of the last; the whole flight ends at 120 s of air, 80 s before
# 2026-10-14; MOVE / YAW only above 0.25 m + 0.01 m/s x the air time: from 80 s at 1.05 m none):
python3 tools/espfly_auto.py move --left 0.5
python3 tools/espfly_auto.py yaw --deg 90
python3 tools/espfly_auto.py photo --n 1 --gap 150             # after every MOVE / YAW, before the next one
python3 tools/espfly_auto.py yaw --deg 12                      # to a bearing read off the photo, then a photo
python3 tools/espfly_auto.py move --forward 1.0 --person-gap 1.5 --est 2.5   # ('approach' until 2026-10-15)
python3 tools/espfly_auto.py stop
python3 tools/espfly_auto.py land
python3 tools/espfly_auto.py wait --until idle --timeout 30
```

`state` shows the scene distance (`D_m`), the heading from take-off (`yawOff_deg`), the commanded height
(`hTarget_m`), the air time left (`airLeft_s`), the battery (`battLow`: the drone's `BATT_LOW` condition, the
10 Hz battery below 3.00 V or the hover thrust T_hold over 57000 of the 58000 ceiling, has held 0.25 s; the
drone then refuses MOVE / YAW and lands `BATT_LOW` by itself 0.25 s later; `battLowNow`: the condition on the
latest step, where one reading dips 60-110 mV under the loaded mean and means nothing alone) and the motion
(`mv`). A MOVE or YAW refused `LOW_BATT` with `battLow` clear means the hover needs over 53400: the end of the
battery is near, and Claude keeps to hover, photos, STOP and LAND. Claude lands when `airLeft_s` is below ~15 s
and never on a single low reading. On a held `battLow` it need not send LAND: the drone lands `BATT_LOW` by
itself (the same slow LAND; a LAND sent then is harmless and only changes the reason to `LAND_CMD`). Until
2026-10-14 `battLow` meant 0.5 s below 3.30 V and the drone landed after 1 s. Claude never sends a command it
is unsure of. `land` is always safe; `stop` ends a move where it is but lets a
re-track bob finish (ending it would cost the camera height estimate).

**Plan the session from these times** (the sim, 1 m hover; the battery gives ~60 s of hover per full charge,
the air limit 120 s since 2026-10-14, 80 s before):

| Step | Air time |
|---|---|
| `sortie` to the hover wait (climb to 1 m, photo) | ~6-7 s |
| a 0.5 m move | ~5-8 s |
| a 1 m move | ~7-11 s, plus ~3.6 s of re-view bob when it goes sideways 0.35 x D or more |
| up / down 0.2 m | ~2 s |
| `yaw` 15 / 45 / 90 / 180 deg | ~1.5 / ~6.5 / ~8 / ~11 s at 30 deg/s (from 45 deg on including the re-track bob; user decision 2026-10-13, evening, was 15 deg/s: ~2 / ~8 / ~11 / ~17 s) |
| `photo` | <1 s of the drone's time (Claude's reading of it is the long part) |
| a turn, a forward move and a photo toward something (the `approach` step until 2026-10-15) | ~10-20 s |
| `land` from 1 m to IDLE | ~5-6 s |
| LAND `COLLISION` (a contact) from 1.0 / 1.2 m to IDLE | ~6.5-7 / ~7-8 s (longer after a long hover) |

The user gives the instructions live, in plain words, in the chat (user decision 2026-10-13); Claude
translates them and answers fast. At chat pace (20-30 s between instructions) about 2-3 instructions fit in
one battery. Before a LAND with something within about 1 m ahead, move back 0.5 m first: every LAND creeps
forward, more when the floor fills the view (a2b 14.8). People: approach in small steps with a fresh photo
each time, judge the distance from their size in the photo (D is the background's distance), never closer
than 1.0 m; if they leave the view, turn to find them; if they cannot be found, say so and hover in place;
stop following after 2 minutes (user decisions 2026-10-13).

**People in the view** (2026-10-13: the hold followed a person walking 1-1.5 m in front of the camera, leaned
the drone toward him and it hit something; docs/autonomy-a2.md section 12):
- From 2 s into the hover the drone ignores camera frames of something moving in the view: it holds on what
  it has learned and drifts slowly (a few cm per second) instead of leaning toward the person. A MOVE pauses
  meanwhile and a new one is refused `NO_VIEW`; 2 s of it lands `MOVE_NO_CAM` or `HOVER_NO_CAM`. A person in
  plain, evenly lit clothes filling much of the view may still pass as background: do not rely on it.
- So the person stays >= 2 m from the drone, out of the space it may drift into, and does not walk across in
  front of the camera within ~2 m; they stand still while Claude photographs or approaches them, and move
  slowly between instructions (a move toward them ends >= 1.0 m away by the photo's estimate: `--person-gap`).
- Between instructions the person moves sideways (across the view, >= 2 m away) rather than straight away
  from the drone: a person in smooth clothes filling most of the view and slowly receding is not refused
  (autonomy-a2.md 12.5) and reads as the drone drifting back, so the hold leans toward them.
- An in-air contact (\|acc\| > 1.6 g or a gx/gy rate > 250 dps for 3 ms) lands the drone slowly: LAND
  `COLLISION` (user decision 2026-10-13: "try a slow landing"; it cut the motors before). Any move stops, the
  camera hold is off for the rest of the sortie (the drone no longer leans toward what it hit, it drifts on its
  trim) and the descent ignores the height estimate the contact upset: ~6.5-8 s from 1.0-1.2 m, the props
  turning, the last seconds floating a few cm above the floor; ~0.3-1 m/s at touchdown from a still hover (sim),
  up to ~1.3 m/s after a contact during a vertical move or a bob (it first rises up to ~0.1 m on its own speed),
  against ~4.4 m/s for the old cut from 1 m. The descent's thrust comes from the hover before any up/down move
  or turn's bob (the first LAND `COLLISION` build, 1c2a43f, took it from the move itself and climbed up to ~1 m
  on the sim after a contact during one: do not fly that build). A flip or spin after the contact
  (tilt > 30 deg, a rate > 500 dps for 30 ms) still cuts the motors. So: nobody within ~2 m and nothing fragile
  under or around it; `k` (KILL) if it heads for someone.
- After a flight longer than ~20 s, wait before the next one (the `arm` reply's 'IMU heat soak: wait N s').

**Expect:** about a minute in the air in all; a move off by +/-20-30 % over a scene of even depth (no position
sensor: the scene distance sets its size) plus the hover's own +/-10-25 cm wander; more (up to D_far / D_near)
for a sideways move past a near object onto a far background, and a forward move that goes further than commanded
(ends nearer the scene) when the floor fills the view; a turn, or a sideways move of 0.35 x D or more, followed by a short +4 cm bob while the
camera re-measures the scene; forward moves the drone refuses near the scene ahead (`TOO_CLOSE`: the floor holds
for an image expansion scale down to 0.6), long sideways ones (`TOO_WIDE`: split them); the slow LAND from 1 m
(~5-6 s), with
~0.05-0.18 m of forward creep.

### E. The power build: pulse placement and Wi-Fi HT20 (user decision 2026-10-13, evening)

This build changes two things (docs/autonomy-a2b.md section 15). Above 50 % duty, M1 and M3 pulse at the start of
each PWM period and M2 and M4 at its end. The access point runs at 20 MHz (HT20). Control, limits and settings
are unchanged. The user flashes it, then runs `auto unlock` as after any flash. Fly the re-routed battery cable on
the current firmware **first**, so the back-left data is not mixed with this change (a2b 15.1, "side effect").

**Wi-Fi, no motors.**

1. `net` should print `bwMhz=20`. Any other value means the AP kept HT40: the boot log line `wifi: AP bandwidth`
   says why. `txDbm` should be 11.00, as before. HT20 is set for each boot only and never saved to the Wi-Fi NVS
   (a2b 15.2), so flashing back an older build brings HT40 back with no NVS erase. To see it from the Mac with
   any build: `system_profiler SPAirPortDataType` lists nearby networks under "Other Local Wi-Fi Networks"; the
   drone's should show `Channel: 1 (2GHz, 20MHz)` on this build and `40MHz` on an older one. Never flash
   `espfly-power-b1a745b.bin`: that first power build saved HT20.
2. The phone app and the Mac join as before (`net` `joins`, `appAgeMs`). Run `python3 tools/espfly_auto.py hello`
   and check a `photo` transfer time, the heartbeat age and a KILL round trip in DRY against the last session.
   `tkoff` should show `txlo` as before.

**Pulse placement on the bench.** Anything over 50 % duty needs enough thrust to lift the drone, so this check
is held down by weight, never by hands, and only if the user decides to run it (as on 2026-10-11: coins taped on,
safety glasses, the user stands back).

1. `pwm` with the motors stopped prints `OK pwm 40000 hpoint=0,0,0,0 duty=0,0,0,0 aligned=0x0`. After a latched
   STOP it prints only `OK pwm 40000`.
2. Low duty (phase B, `gcap 15000`, throttle up): `pwm` shows the old spread, e.g. `hpoint=0,65,131,197
   duty=58,58,58,58 aligned=0x0`. This is the spin-up layout and is unchanged.
3. Held down at `gcap 40000` (duty ~156, 61 %): `pwm` shows M1 and M3 at 0 and M2 and M4 at 255 - duty, e.g.
   `hpoint=0,99,0,99 duty=156,... aligned=0xf`. The mixer spreads the duties a little.
4. Optional A/B against the previous build (it runs HT40 again, since HT20 is not saved), at matched resting
   voltage (ABAB): the `status` `vbatFastMin` and `railFastMin` should rise, the mean loaded `vbat` should not
   fall, and the recorder's gyro noise above 5 Hz should not grow. At each period start two channels switch on while two switch off, so a switching-noise
   change would show up there.
5. With a scope (GPIO 7/4/3/1 = M1..M4, or the motor pads), at hover duty: M1 and M3 rise together at the period
   start and M2 and M4 fall together at its end. Every pulse lasts at most one period, and no channel stays high.

**In flight.** Fly a normal sortie: the drone should fly as before.

- The recorder's back-minus-front duty split at hover, (m2 + m3) - (m1 + m4), may fall a little. In the old
  layout the back motors saw more of the others' current during their pulses, so the pitch trim may move by
  as much (the integrators absorb it). Note it as a measurement, not a fault.
- The landing step (resting minus loaded voltage) at matched resting voltage should be equal or a few mV
  smaller.
- Air per charge cannot show a gain this small: session-to-session noise is +/-10-20 %.

### F. The end-of-battery build (user decision 2026-10-13, late)

This build lets the last flight of a charge go deeper (docs/autonomy-a2b.md section 16): the thrust ceiling is
58000 (was 54000), and in the hover the drone lands `BATT_LOW` (slowly, by itself) once the battery reads below
3.00 V or the hover needs more than 57000 for 0.5 s, instead of the fast `HOLD_CAPPED` LAND at about 3.06-3.12 V
loaded. MOVE and YAW are refused (`LOW_BATT`) once the hover needs more than 53400; hover, photos, STOP and LAND
still work. Nothing else changes: gains, settings, `vmin` 3.30 V, the 80 s air clock and the 60 s wait (both 120 s
since section H).

Before the first flight: `auto get` and note `vzK` (160391 on 2026-10-12; it is learned after every slow LAND).
`K_RANGE` (a fast LAND, about 0.7 m/s from about 0.25 m in the sim) can end a take-off on a late charge when the
drone's K is more than about 1.07 x `vzK`: with `vzK` 160.4k, a drone about 5k heavier than the pooled fit (in the
sim at 3.82 V at rest: 24 of 108 sorties, all with K +5k; at `vzK` 158000, 31, 7 of them on the pooled fit
itself; `test_interactive_sim` `testDrainVzK`). It is a take-off ending, not an end-of-battery one: the hover's
end is `BATT_LOW` either way.

1. Fly the first battery-end flights at 0.6 m or lower, over the mattress, the person at least 1 m away.
2. Let it hover (STOPs keep the wait alive) until it lands `BATT_LOW` by itself; expect it about 10-30 s later
   than the old end on the same kind of charge. Moves refused `LOW_BATT` near the end are expected.
3. After each one save `flights` and check the 1 kHz battery minimum and the '3.3V rail min'. **Any reset is the
   stop rule** (a boot that ends a flight `RESET`, or the drone rebooting in the air): stop; after a reset in
   the hover's end or its LAND raise `AUTO_VBAT_LOW_LAND_V` to 3.05 V, after one in a take-off raise `vmin`
   (item 4). The rail minimum cannot clear a flight: the 13 logged 40 kHz resets (12 brown-outs, 1 panic, all
   in the take-off window) read 2.74-2.85 V on it, flights that did not reset as low as 2.82 V, so anything
   below about 2.85 V is inside the reset range, not "clear". At the new end expect about 2.6-2.8 V (the long
   hovers read 0.10-0.29 V under their 10 Hz battery minimum), lower than any 40 kHz flight so far: note it,
   it is no pass / fail (a2b 16.4).
4. Take-offs from a weak cell are not short hops. In the sim (`vzK` 160400, `test_interactive_sim`
   `testWeakTakeoff`), from 3.65-3.70 V at rest nearly every take-off ends `CLIMB_CAPPED` or `NO_LIFTOFF`, a fast
   LAND at up to 0.9 m/s from up to 0.21 m, with the loaded battery at 2.6-2.9 V in the air (6742d91: up to
   0.6 m/s from 0.11 m, 2.69-2.90 V); from 3.75-3.78 V some take-offs of a drone 5k heavier than the pooled fit
   end the same way (up to 1.2 m/s from up to 0.41 m). The take-off is where every logged 40 kHz in-air reset
   happened (at a 10 Hz battery of 3.07-3.40 V), and the drone rebooted in a take-off on 2026-10-13 at 3.80 V at
   rest. `vmin` 3.80 V (the firmware default: `auto set vmin 3800`) avoids the `CLIMB_CAPPED` / `NO_LIFTOFF`
   take-offs in the sim, 3650 does not; only the `K_RANGE` take-offs of a heavier drone remain (above). `vmin`
   stays 3.30 V unless the user changes it.
5. If the battery is moved back (the old back-left heavy balance), the per-motor margin at 58000 shrinks to
   1.2k at p99: keep the battery forward.

### G. The more-air build (user request 2026-10-14)

This build changes what the drone does while it waits and how fast it flies (docs/autonomy-a2b.md section 17).
Claude runs these checks over USB after the user has flashed it (and `auto unlock` as after any flash), props off
unless a step says otherwise, the drone level on the bench. Never flash from here.

**G1. The camera asleep on the ground (a2b 17.1).**

1. `tkoff`: `camgnd=1 camgndms=45000 camlead=0 gndCount=0`.
2. Leave it idle 50 s (USB, autonomy IDLE): `status` `cam=standby camGnd=1`; `tkoff` `asleep=1 gndCount=1`; `live`
   `cam=standby`.
3. `tkoff camwake 30`: within 0.5 s `status` `cam=running camGnd=0` and `camFps` over 10 (published from the
   second fresh frame); `camTimeouts`, `camRestarts` and `camBad` unchanged.
4. `tkoff camgndms 5000`, then 10 sleep/wake cycles (`tkoff camwake 1`, wait ~7 s each): `gndFails=0`,
   `camRestarts` unchanged. `tkoff camgndms 45000` afterwards.
5. Mac: `photo` while it sleeps: OK, the frame arrives within 1.5 s, `lum` within +/-15 % of a photo taken awake.
   `arm --mode flight` after a sleep (props off, then `disarm` at once): the event `arm_cam_wake` with
   `retries` 1-2, then ARM OK. `heapInt` >= 20 KB throughout.
6. The thermal step test, before relying on short leads: USB plugged at least 20 min, then
   `python3 tools/espfly_diag.py drift --minutes 25 --interval 15 --log-acc --cam-step awake:300,sleep:600,awake:600`.
   Report the per-step dImuT and d(acc z), tau, the tempco and the predicted in-flight drift for L = 3 / 20 /
   60 s. Over 1 mg at L = 3 s: `tkoff camlead <the ms it prints>`; if that is over 60000 keep the sleep to the
   soak only, or `tkoff camgnd off`.
7. The first battery sessions: per charge note the time on the ground on battery and `tkoff` `gndMs`. After the
   flights compare `flights` `imuT0` / `imuT1` / `camAwakeDs` and the observer's `cvB3` / `cvBtd` / `cvHtd` with
   earlier flights. Stop rule: flights after a short wake (`camAwakeDs` under 30 s) whose |biasTd - bias3| is
   more than 2 mg beyond the earlier range, or more `HOVER_NO_CAM` endings: `tkoff camlead 60000` or
   `tkoff camgnd off`, and report.

**G2. The camera at 14.8 fps (a2b 17.2).** USB, camera awake (`tkoff camwake 300`), a textured scene. Before
flashing, record the same checks on 126b309 as the baseline (or A/B in the new build: `tkoff camclk 20`, reboot,
measure, `tkoff camclk 13`, reboot).

1. Boot log: camera running, PID 0x3660, no init error; `status` `camClk=13.33 camClkFallback=0`; within ~15 s
   of the camera's start `camClkChk` goes from `wait` (`camFps=0.0`: no FLIGHT ARM yet) to `ok`, and only then
   does the camera sleep on the ground. A boot that shows `camClkFallback=1` (the loop too slow for 13.33 MHz:
   the boot log prints `camProcAvgUs`) flies at 20 MHz for that boot: note `camProcAvgUs` and report; `slow`
   (the switch failed) refuses every FLIGHT ARM `CAMERA` until `tkoff camclk 20` and a reboot.
2. `status` at t = 0 (after `camClkChk=ok`) and t = 60 s: `camFps` 14.6-14.9 (the processed rate); the `camFrames` delta over the `up_ms` delta >= 14.4 /s;
   `camTimeouts`, `camRestarts`, `camBad` unchanged (0); `camAgeMs` < 150; `camProcAvgUs` <= 62000 and
   `camProcMaxUs` <= 66000; `camSkips` <= 1 % of the frames; `heapInt` >= 20 KB; `maxGapUs`, `gaps1500`,
   `gaps2500` no worse than the baseline.
3. `python3 tools/espfly_diag.py flow-cal --seconds 60 --out cal13.csv --offset-scan`, hand-held (yaw, pitch,
   roll): flowDt median 67-68 ms, >= 95 % within 66-69 ms, <= 2 % at 130 ms or more; the first-sighting
   `flowAge` p50 <= 135 ms, max <= 200 ms; `flowOk` >= 95 %; texture and residual medians within +/-15 % of the
   baseline; `lum` median within +/-15 % and no frame-to-frame lum flicker over 10 %; dx vs gz within +/-5 % of
   102.4 px/rad, R^2 >= 0.95; the scan's best offset 22-30 ms (outside: set `CAM_CLOCK_OFFSET_REF_US` / the model
   to it, rebuild).
4. `camvz air full` and a hand-moved vertical motion: accept / reject ratios as on the baseline.
5. A DRY sortie with `camvz air full`: >= 940 loops/s (A4 gate).
6. Optional, on battery, same boot: `rail` with `camclk 20` against `13` (expect 5-15 mV).

If any check fails: `tkoff camclk 20`, reboot, and fly as before.

**G3. The climb is unchanged (a2b 17.3).** After flashing, the first sorties' climb times (TAKEOFF to HOVER in
`events` / the recorder) should match the previous build's: `vClimbMax` is still 0.40 m/s.

**G4. The faster LAND from 1.00 m and higher (a2b 17.4).**

1. Bench (USB): a DRY sortie at 100 cm with `camvz air full` ends DONE (the sim plant); a BENCH-sim sortie at 120 cm:
   LAND reason `CMD_LAND`; in the recorder `vzThrust` over 0.98 x T_ref only for <= 0.5 s and only while `vzH_mm`
   reads 400-850.
2. The first real flights (props on, the user present, the 2 m rule): 100 cm sorties, LAND by command with the camera
   estimate healthy. From the dump: LAND to contact about 0.3-0.5 s shorter than the earlier 100 cm flights
   (2.7-3.1 s); `vzV` between 0.85 and 0.6 m about -0.30..-0.40 m/s (sims from 1.0 m: -0.30..-0.37; from 1.2 m
   -0.40..-0.46 at 0.85 m), then at 0.5 m within ~0.05 m/s of the earlier flights' LANDs at 0.5 m (the sims: 0.29-0.41 m/s fast against 0.26-0.40 before, by drag) and at 0.2 m as before; the
   contact speed no faster than the earlier flights' 0.43-0.58 m/s from 1 m; no thrust over 1.03 x T_ref; no climb
   in the LAND (`vzV` never above +0.05 m/s after its first 0.3 s). A LAND below 100 cm, or for a reason other than
   the Mac's LAND, the air clock, the hover wait's end or the sortie done (a battery rule, a lost link, a guard),
   is the LAND of before (thrust <= 0.98 x T_ref).
3. Stop rule: a contact faster than 0.6 m/s or any upward motion in a LAND: rebuild with `landVfast` = `landVref`
   (no NVS switch) and report.

**G5. Faster moves (a2b 17.5).**

1. DRY: `move --right 1.0` at the sim scene D 2.0 m (capped by the lateral view rate at 0.24 m/s) ends DONE with
   the profile time printed; BENCH-sim the same.
2. The first real flights: one lateral 1.0 m move at D >= 3 m and one forward 1.0 m at D >= 3 m, at 1.0 m height.
   From the dump: the horizontal reference speed (`mvVRef_mms` is the vertical one and stays near 0 here): with the
   move's scene D from the Mac's move result, lateral D x the slope of `mvRefLat_e4` x 1e-4 per s, forward
   D x exp(-`mvRefFwd_e4` x 1e-4) x its slope (both over 0.5 s at the cruise): the lateral peak 0.30 m/s (0.12 x D
   below D 2.5 m), the forward one 0.20 m/s (toward the scene as before; less where 0.15 x the end distance caps
   it); the measured travel within 25 %, the end DONE or SHORT, the lean under 2 deg; a STOP on a far lateral move
   stops within ~0.4 m.
3. Stop rule: a `PROGRESS` end (the hold lags too much at 0.30 m/s): rebuild at 0.25 m/s (the user allowed
   0.25-0.30) and report.

### H. Longer flights and the palm catch (user decisions 2026-10-14)

docs/autonomy-a2b.md section 18. After flashing, before anything else: `auto status` (names) and `heapInt`
>= 20 KB (expect ~22.0-22.3 KB).

**H1. The 120 s air clock (a2b 18.1).**

1. Bench (USB): a DRY sortie `sortie --mode dry --height 105 --wait-s 100` (or `--wait-s 60` with a `stop`
   every ~50 s): `airLeftDs` ~1190 right after RAMP; the Mac's `air_s` counts past 80 s (and past 65.5 s, where
   `airMs` saturates); the sortie ends LAND `AIR_TIME` at 120 s; no `HOLD_TIMEOUT`, no `BAD_CONFIG`.
2. The first long flight, after the normal short checks. Setup: a full, rested battery; a room >= 2.4 m high
   with an open floor; a textured scene >= 1.5 m ahead; tape marks on the wall at 1.0 m and 2.0 m and a phone
   video from the side as the height reference; `--height 105 --wait-s 100` and a `stop` every ~50 s to keep it
   up. During: the operator watches the gap and sends `land` (the daemon's `l`) if the drone rises past
   ~2.0 m, before it reaches the ceiling (a LAND begun pressed against a ceiling stops the motors up there:
   a2b 18.3 finding 1); `k` only for a loss of control or a sink below ~0.3 m (a `k` with the props spinning
   latches the motor driver until a reboot). After: `dump` (the recorder keeps the last 60 s) and `flights`
   (`h` at touchdown is the drift); compare with the video.
3. Rule: hovers that may pass ~80 s only after this measured flight, at <= 1.05 m. If the estimate read low by
   more than 0.5 m at 80 s, ask the user for the height-dependent air limit in the firmware (80 s above
   1.05 m; the Mac has it already, step 4).
4. (Removed 2026-10-15, a2b 23.4: the Mac has no ceiling rule; Claude checks the height and the ceiling from the
   photos and corrects with `set-height`, M.) The ceiling rule (the Mac, a2b 18.1). Above 105 cm a hover ends at 80 s of air: `sortie --mode dry --height
   110 --wait-s 80` is refused ("the ceiling rule"); `sortie --mode dry --height 110 --wait-s 60` with a `stop`
   every ~30 s is landed by the Mac at 80 s (event `ceiling_land`, LAND `LAND_CMD`). Whatever the Mac does,
   `land` a drone that rises past ~2.0 m before it touches the ceiling (step 2).

**H2. Hover waits of up to 120 s (a2b 18.2).** USB, IDLE.

1. Before flashing: `auto get`: note `autowait` (expect 10000).
2. After flashing: `auto get`: `autowait=10000(legacy)` (the old u16 item, read through the fallback).
3. `auto set autowait 10000` (re-saves it as u32), `reboot`, `auto get`: 10000 (`nvs`).
4. `auto set autowait 120001`: refused (range); `auto set autowait 120000`: OK.
5. `auto reset autowait`: 5000; `reboot`; `auto get`: still 5000 (`default`): both items are gone.
6. `auto set autowait 10000` (the user's value).
7. Mac DRY: `sortie --mode dry --height 105 --wait-s 110`: accepted; the status shows `waiting >= 65.5 s`
   (`waitLeftSaturated: true`), then counts down from 65.5 s; a `stop` restarts it; the sortie ends LAND
   `AIR_TIME` at 120 s. `sortie --mode dry --height 100 --wait-s 110` is refused with `needs --height >= 105 cm`,
   `--height 110 --wait-s 110` with "the ceiling rule" (that refusal is gone since 2026-10-15: a2b 23.4). `--wait-s 0` is checked against `autowait` as the last
   `auto get` read it (`auto set` / `auto reset` read it again): with `autowait 120000` (as right after step 4)
   `sortie --mode dry --height 40` is refused (`the drone's autowait (120000 ms ...) needs --height >= 105 cm`);
   with step 6's 10000 it flies.
8. First flight: nothing extra; the long-flight check H1 with `--wait-s 100` instead of the STOP keep-alive.

**H3. The palm catch (a2b 18.3).**

1. Bench (USB): `auto status` names `PALM` and `TOUCHDOWN_SUPPORT` (the Mac's `state` too); `heapInt` >= 20 KB
   (expect b176c26's value + ~0.2 KB).
2. DRY, props off: `sortie --mode dry --height 105`. After the wait starts, tap the drone firmly with a finger
   (the real IMU: LAND `COLLISION`; the sim plant flies the blind LAND in free air). Expect: the probes abort
   (the recorder's `vzThrust` dips to ~0.8 x T_hold for ~0.1 s at 0.1 s and every 2 s), the sim lands, IDLE
   `TOUCHDOWN` (vz FORCE / JOLT), never KILLED.
3. The `PALM` cue cannot be tried in DRY: it reads the controller's `aRel`, which in DRY comes from the sim
   plant's accelerometer (the `COLLISION` of 2 reads the real IMU's \|acc\|). The first flights below are its
   test.
4. BENCH mode: neither `PALM` nor `COLLISION` (unchanged).
5. First palm-catch flights. Setup: low and soft, `--height 40 --wait-s 30`, one photo, over a bed or a sofa;
   a fresh battery, appreq 0; the daemon tab open with `k` ready, only for trouble while the props spin; eye
   protection advisable; kneel beside the drone, arm under it.
   1. Let it hover >= 2 s (the cue arms 1 s into the hold).
   2. Bring a flat palm (fingers together) straight up from below at ~10 cm/s until it touches the frame or the
      battery, then lift ~3 cm and hold still. Never from the side or above (the props are on top).
   3. Expect the motors to stop within ~1 s: LAND `PALM` (or `COLLISION` for a brisk touch: then up to a few
      seconds, the drone may bounce off the palm and settle back), then IDLE `TOUCHDOWN` (vz `TOUCHDOWN_FORCE`
      or `TOUCHDOWN_SUPPORT`).
   4. Nothing within ~2 s (too gentle: no cue): lower the hand away; the drone hovers on and lands by the wait
      or `land`.
   5. Keep the palm still and flat until the motors stop (pulled away then, the drone drops from the palm; a
      tilt over 10 deg for 0.5 s is KILLED `ATT_ERR`, which clears by itself). Then put the drone down: the
      learned vzK is stored only on the ground, and the next ARM needs >= 20 s still plus the heat soak.
   6. Three times at 40 cm, then at 70 cm. Optionally a firmer upward tap (the `COLLISION` path).
   7. After: `dump`: the probe dips in `vzThrust` (~0.55 x T_hold for ~0.15 s when supported), contact to
      thrust 0, and no `PALM` in the session's normal hovers. Do not use `k` after the motors stopped (not
      needed; a `k` while they spin latches the motor driver until a reboot).
6. Long hovers: a2b 18.3 finding 1 (a LAND pressed against a ceiling stops the motors up there). Until it is
   addressed, `land` a drone that rises past ~2.0 m before it reaches the ceiling, and keep long hovers at
   <= 1.05 m (the Mac's ceiling rule lands a hover above 1.05 m at 80 s: H1 step 4). Since 2026-10-15: the
   height routine of a2b 23.5 (photo checks and `set-height`) replaces the ceiling rule.

### I. The thermal drift, bright windows and take-off build (user request 2026-10-14 ~10:00)

docs/autonomy-a2b.md section 19. After flashing, before anything else: `auto status` (names), `status`
`heapInt` >= 20 KB with the Mac linked (expect ~22.1-22.5 KB), `rec info` (`cap=18000`, `psramFree` >= 1 MB).

**I0. Trims** (a2b 19.9), one USB session, read / set / save / read back: `trim show` (1792, -4435), `trim set
250 800`, `trim save`, `trim show`; `atrim show` (-0.20 / -0.10, learn off), `atrim set -0.20 0.00`, `atrim
save`; `facc show` (0.0120 0.0000, unchanged). pwm 40000, camclk 13, vzK and the hold switch unchanged.

**I1. The defaults** (`gtc`, `raw "mki show"`, `raw flowfit`). `gtc` is `python3 tools/espfly_diag.py gtc`
(`gtc show` on the drone): it prints the `A` line (the first re-zero after the latest flight), the logged `Z`
lines, then the `OK gtc` line, and the post-flight check; `raw "gtc show"` prints the same lines (the details
come before OK). `gtc set|on|off|save|clear|hold` go through it too.

1. `gtc`: on, k +0.018 / -0.060 / +0.000, knee 44.0, clamp 1.00, src default; T 38-47 C; `latchAgeMs` below
   ~1500 and `rezeros` counting up about once a second while the drone sits still (Tref re-latched), while
   `logged` does not (a quiet re-zero is not logged: only one after thrust, after a hold, or with a step >= 0.05
   dps); `A seq=0` until the first flight of the boot.
2. `mki`: `twoKiAir=0.0400`, `airborne=0`, learned x / y below 0.05 dps; still for 5 min: still below 0.05 and
   back to 0 at each re-zero.
3. `flowfit`: `mode=robust`.

**I2. The cooling test** (no flight; props may stay on, motors off). `gtc hold 240` with the drone still,
then let the IMU cool (the camera's ground standby, 45 s, does it; or a desk fan). Every 30 s: `measure 2000`
(its `gyro=` is the still reading minus the bias in use, uncompensated: the raw drift since the latch, positive
on y as the IMU cools) and `gtc` (the OK line's `model`, `dTeff`). The corrected still gyro is `gyro` minus
`model`. Pass: while T falls >= 5 C the corrected |gy| <= 0.15 dps and |gx| <= 0.10 dps, and `model` y grows
~0.06 dps per C of cooling (positive, the same sign as `gyro` y). Abort: a residual of about the expected size
with the opposite sign (`gyro` y and `model` y of opposite signs): `gtc off`, and stop (report). At the end
`gtc hold 0` (any thrust or an ARM also ends it); ~2 s later `gtc` lists a `Z` line with `hold=1`: the first
re-zero after the hold, whose `r` is the whole cooling's residual in one number (same pass limits; `step` and
`corr` of opposite signs is the abort).

**I3. The camera** (Claude reads over USB), facing the bright window, then the dark room: `status`
`camProcAvgUs` <= 60500, `camProcMaxUs` <= 63000, `camClkChk=ok`, `camFps` 14.6-14.9, `camSkips=0`, `camClk`
13.33, `camClkFallback=0`; `raw flowfit` `satPct` ~8-20 at the window, `gainE3` ~0. Over budget
(`camProcAvgUs` > 60500), or already `camClk` 20.00 / `camClkFallback=1`: the 62 ms clock fallback lasts for the
rest of the boot (cam_clock.h), so a fit mode alone cannot bring 13.33 MHz back: `raw "flowfit lite"` (saved),
`reboot`, `auto unlock` (the reset-lock, as after a flash), wait ~15 s for `camClkChk=ok`, then re-check
`camClk` 13.33, `camClkFallback=0`, `camClkChk=ok` and `camProcAvgUs` (facing the window); still over: the same
with `raw "flowfit plain"`; still over with plain: stop and report (that is 14bcfc6's fit: the cost is
elsewhere). Hand test: wave a hand over the lens, or turn the drone by hand from the room to the window:
`expGated` / `expCorrected` count up; a `stream 10` shows `flowOk` 0 on the gated frames for <= 0.5 s at a time,
never more than 1 s. A ground arm/disarm and `dump`: `imuTemp_cC` near `gtc`'s T, `flowGain_e3` ~0,
`flowSat_pct` 0-20.

**I3b. GO / NO-GO: the loop rate** (A4; before any BENCH or FLIGHT). This build grows the camera's fit (four
specialised copies of the pixel loop on core 1, which shares the 16 KB instruction cache) and adds flash code on
the 1 kHz sensors path (the temperature compensation), and the drone's cost was estimated, not measured. With
the `flowfit` mode I3 left: the A4 sequence as written (`raw "camvz air full"`, `auto prof reset`, gesture, the
DRY sortie with the two `camvz prof` reads 3 s apart, `wait --until idle`, `sleep 6`, `auto prof`, `raw "camvz
air off"`). Needs `activeLoopsPerS` >= 940 (the tool prints `go/no-go: GO`), (`loops` difference) / (`upMs`
difference) x 1000 of the two `camvz prof` reads >= 940, `stabStackFree` >= 1024 B; also `status`
`camProcAvgUs` <= 60500 and `camClk` 13.33 right after it. **Below 940: stop** (A4): no BENCH, no FLIGHT; try `flowfit lite` (I3's reboot steps)
and repeat once, then report. Measured on 0ed68c9 (plain, 13.33 MHz): 921.4, the gate missed; the loop-rate
build (a2b 19.11) targets >= 955 (14bcfc6: 961; expected ~958-964 with the review round, not measured): `status`
`bypass=0x00`, the A4 run (its `auto prof` with `bypassSeen=0x00`), then, if under 955, A4b's bisect (each
toggle's gain) and A4b step 4 (the fit modes' `camProcAvgUs`).

**I4. SPIN.** Inspect and seat the props. Three consecutive autonomy take-offs from the floor with no
`SPIN_GYRO`.

**I5. F1: the long hover** (the thermal fix and the windows). Open room, >= 1 m from furniture, facing the
bright window, `--height 100` with a wait of >= 45 s (a `stop` keeps it up), no moves. The user watches the
position against a floor mark. After the landing, once USB is in: `gtc`, `flights`, `dump`. The first ground
re-zero after the flight (~2-3 s after the motors stop, once the drone rests still) is kept on its own until the
next flight, so it can be read any time in that boot: `gtc`'s `A` line (`landS` its delay after the last airborne
sample, `airS` the airborne stretch, `ageS` its age) and the check `gtc` prints under it; `flights` shows the
same r and dTeff per flight (`post-flight r x/y @dTeff`, RTC: it also survives a brownout reset). `A seq=0`, or
`airSinceLatch=1` in the OK line, means it has not come yet: leave the drone still a few seconds and read again.
Pass:

1. the re-zero residual |r_y| < 0.15 dps (was +0.65) and |r_x| < 0.10; refine if |r_y| > 0.10 with |dTeff| >=
   5 C (|r_x| > 0.08 the same way on x): `gtc` prints the command, `gtc set <kx> <ky + r_y / dTeff> 0 44` (dTeff
   negative when cooling), then `gtc save`; a `SIGN` line (the model's sign opposite to the measured step):
   `gtc off`, no long hovers, report;
2. the hold's mean pitch within +-0.4 deg, the drift the user saw < 0.5 m;
3. in the dump: the learned bias settles within ~20 s and |`mkiBiasY_mdps`| < 200 for the whole hover; the
   innovation (the accelerometer tilt minus the estimate) < 0.4 deg from 40 s on; `holdPitch` at the HOVER
   entry within +-0.3 deg, the pitch excursion in the first 0.4 s after lift-off within +-1.5 deg (the new
   torque trim), no backward kick after the climb; refused exposure frames < 1 %, `flowSat_pct` logged.
4. `flights`: `gtcY` ~ +0.6 dps x (the cooling / 10 C), `mki` small.

After 2-3 such hovers pass: `atrim learn on` (else every long hover walks the pitch trim -0.15 toward its -0.8
limit).

**I6. F2: a YAW in the open.** At 1.0 m, >= 1 m from furniture: `yaw 90`. Pass: no `PALM` through the turn and
its re-track bob, the motion ends DONE. A palm brought up during a bob needs a firmer push (0.16 g) or ends
`COLLISION`.

| What you see | Why, and what to do |
|---|---|
| `gtc` prints `SIGN` (the `A` line's `step` and `corr` of opposite signs, r about twice the step) | the compensation's sign is wrong for this IMU: `gtc off` and no long hovers; report |
| `gtc` `A seq=0` or `airSinceLatch=1` after a flight | no ground re-zero since the motors stopped: the drone has not rested still for 2 s; leave it still, read again |
| `camProcAvgUs` > 60500 | `raw "flowfit lite"`, then `raw "flowfit plain"` (both saved; plain = 14bcfc6's fit with a brightness gate) |
| `expGated` climbing fast while hovering | the auto exposure steps > 8 %: the hold coasts on those frames; `dump` and report (the H item may reopen) |
| a backward / forward creep returns on long hovers | `gtc` / `flights` post-flight residuals and `mki` learned bias after landing; refine `gtc` as in I5 |

### J. The hold's gain scale and the take-off trims (user request 2026-10-14, afternoon: no drift at take-off)

docs/autonomy-a2b.md section 20. After flashing: `auto unlock`, then `status` (`heapInt` >= 19.5 KB with the Mac
linked: expect ~20.0 KB, 24 B under c928b6d's 20.06), `rec info`.

**J0. The settings** (one USB session; read, set, save, read back):

1. `facc show` (0.0120 0.0000), `facc set 0.007 0.000`, `facc save`, `facc show`.
2. `atrim show` (learn off; -0.42 / -0.18 if the interim value is still in), `atrim set -0.60 0.00`, `atrim save`,
   `atrim show`. `atrim learn off` if it reads on.
3. Unchanged, read only: `trim show` 250 800; `gtc show` k 0.018 -0.060 0, knee 55; `raw "mki show"` 0.0400;
   `auto get` vzK 160000.
4. `raw "holdk show"` (or `python3 tools/espfly_diag.py holdk`): `kmax=2.50 src=default s=1.00 target=1.00 G=0.0
   D=0.00 translated=0 hold=on`.

**J1. GO / NO-GO.** The A4 sequence (`activeLoopsPerS` >= 940, `bypassSeen=0x00`; this build adds no IRAM and no
code on the 1 kHz path: expect c928b6d's 959-965 +- 3); `status` `camClk` 13.33, `camProcAvgUs` <= 62000 (plain).
During A4's DRY sortie `holdk show` should read `s=1.00` (a still drone gives the observer no vertical motion
to track a G from, even with `camvz air full`); more is harmless there (DRY has no thrust), but note it.

**J2. Flight A**: TAKEOFF to 1.0 m, PHOTO, a plain HOVER of 45 s (`--wait-s 45`, no MOVE or YAW), LAND. A flat,
hard floor (not a rug or a tilted board), the drone idle >= 2 s before ARM, rest tilt within +-1 deg; facing a
textured wall or furniture 2-3 m ahead, nobody walking in view, steady light. The user 2 m away with the kill
switch; catch only beyond 0.6 m of drift or near an obstacle. **Flight B**: the same from the same spot, turned 90
deg to another textured view. **Flight C**, only if A and B pass: HOVER 15 s, then one forward MOVE toward a person
2.5 m ahead (s ramps to 1 at the MOVE; at D <= 2.5 m the sim shows no MOVE cost from the gain scale, while far
scenes, D >= ~3 m, end PROGRESS / SHORT more often: a2b 20.8, keep C at D <= 2.5 m).

**J3. After each flight**: `dump`, `flights`, `gtc`; then the replay scripts (scratchpad `drift/designer`,
`drift/implementer`). Read:

1. Drift: lateral from the camera; forward from the velocity-constrained climb plus the hover camera (not levelfit's
   0.368/m climb correction), at HOVER entry, HOVER + 3, lift-off + 8, + 15, + 30, + 45 s. **Pass**: |x| < 0.3 m at
   lift-off + 15 s and < 0.5 m at lift-off + 45 s. Replay expectation: ~0.13 m at HOVER + 3, ~0.14 at + 8, 15 s
   maximum ~0.24 m (worst flight 0.37).
2. The gain scale: s = min(2.5, max(1, 80 / (cvGain_dpm / 10))) on rows with `cvFlags & 5 == 5`, 1 before the first
   such row (slewed at 2 /s, 1 again from the first MOVE and in LAND): expect s 1 until ~1.2-1.4 s after lift-off,
   then 1.6-2.5 in the hover (2.0-2.5 at a 2.5-3.5 m wall), back to 1 within ~0.75 s of LAND entry. `holdk show`
   clears with the sortie (IDLE): read s from the dump, not after landing.
3. Oscillation: the hold roll and pitch in the 0.3-1.5 Hz band from HOVER + 2 s to LAND: `python3 tools/hold_band.py
   <dump.csv> <flightId>` (`--kmax 1` after `holdk set 1`; it rebuilds s from the dump; exit 1 on ROCKING). Expected
   from the camera noise alone at s 2-2.5 (the log replay of today's 5 hovers, a2b 20.8): band rms 0.05-0.27 deg per
   axis, peak-to-peak 0.3-1.6 deg (up to ~1.9 over a 40 s window), rms / s 0.04-0.11 deg (the logged s 1 hovers: rms
   0.02-0.12 today, 0.19 on 2026-10-13 pwmA1 f7). **Rocking**: a visible oscillation, band peak-to-peak > 2.2 deg,
   or band rms > 0.4 deg (the sim's loss of damping is a limit cycle at 4-5 deg peak-to-peak): `holdk set 1` (not
   saved: c928b6d's hold exactly) and report. rms / s > 0.15 alone (hold_band.py prints NOISY) is a noisier camera,
   not rocking: note it and compare with the next flight.
4. The hold's mean roll and pitch over lift-off + 10..40 s within +-0.3 deg; else `atrim` += 0.7 x the mean, `atrim
   save`.
5. The velocity-constrained forward displacement at HOVER entry: more than +0.12 m forward in both A and B: `facc
   set 0.005 0.000` and atrim pitch +0.11; less than -0.12 m: `facc set 0.009 0.000` and atrim pitch -0.11 (save
   both).
6. The lift-off kick (estimate minus setpoint over 0-0.5 s) and the hover's mean `pitchRateI`: outside +-1000 (a
   reseated battery) note the new torque need first; the trim and the pitch compensation change together.
7. `gtc`: post-flight r within +-0.15 dps.

| What you see | Why, and what to do |
|---|---|
| `holdk show` `s=1.00` all through a hover | no tracked G in the air (texture, a scene beyond ~6.6 m, a YAW / MOVE in the sortie): the hold as c928b6d; check `cvFlags` in the dump |
| the drone rocks visibly, or band peak-to-peak > 2.2 deg, or band rms > 0.4 deg (hold_band.py ROCKING) | `holdk set 1` (live, not saved), fly again, report |
| band rms 0.2-0.4 deg, peak-to-peak 1-2 deg at s 2-2.5, no visible rocking | expected: the camera noise times s (today's replay up to 0.27 / 1.62); not rocking |
| it drifts the same way in A and B | a trim: J3 4 / 5 |
| drift only from a tilted spot (f8: rest roll +1.5 deg) | the filter's lift-off reference: take off from a flat, hard spot |

### K. One-step long moves and the planned take-off (user decisions 2026-10-14, evening)

docs/autonomy-a2b.md section 21. **Phase 1** (the Mac alone, on b6a54da): nothing to flash; `sortie ...
--plan-forward F --plan-via mac` (F <= 1.5 m) and plain moves of up to 1.5 m with `--person-gap`, `--batt-ok` and
`wait-move`. **Phase 2** (the long-move build, flashed only after the user's OK):

**K0. After flashing:** `auto unlock`, `status`: `heapInt` >= 19 KB with the Mac linked and the camera on (expect
~20.0 KB, 56 B under b6a54da's 20.0-20.1); `auto status` names; `rec info`.

**K1. GO / NO-GO.** The A4 sequence: `activeLoopsPerS` >= 955 (b6a54da ~960; this build's IRAM is 12 B over
b6a54da's in the same 256 B step (a2b 21.9) and it adds no flash code on the 1 kHz path), `bypassSeen=0x00`;
`status` `camFps` 14.6-14.9 and `camClkChk=ok`, `camProcAvgUs` <= 62000.

**K2. DRY: the planned take-off.** Gesture; then

```
python3 tools/espfly_auto.py sortie --mode dry --height 100 --photos 1 --wait-s 30 --plan-forward 1.0 --plan-via drone
python3 tools/espfly_auto.py wait-move
python3 tools/espfly_auto.py land
```

The sortie returns in the wait with the photo and `plan.state` `running`, `plan.mvSeq` 1 (DRY flies the sim plant
with a nominal scene D 2.0 m, `AUTO_MV_SIM_D_M`: the plan flies the Mac's 0.60 m floor, up to 1.02 m forward there;
`--plan-forward 1.05` is refused `TOO_CLOSE` with `fwd_max_m` 1.02, a2b 21.9); `state` right after
it: `waiting` true, `mvSeq` 1, `mvState` RUN or SETTLE. Then
`wait-move`: `end` DONE, `travelled_m.fwd` ~1.0 (DRY: ideal). The same with `--plan-forward 4.5`: refused before
anything is sent (usage error, forward <= 4.0); with `--plan-right 1.5 --plan-forward 3.8`: refused by the Mac (the
length 4.085 m); with `--then land`: refused ("needs --then wait"). An old build (b6a54da) answers the 18-byte
TAKEOFF `RANGE` and the sortie says "this build has no planned take-off: --plan-via mac" (no take-off; DISARM).

**K3. DRY: the move refusals.** In a DRY wait (D 2.0 m): `move --forward 3.0`: refused by the Mac, "at most 1.02 m
now" (`fwd_max_m` 1.02); `move --forward 1.0`: DONE; `move --up 0.2 --forward 1.6`: refused (a move up or down goes
at most 1.50 m forward); `move --forward 1.0 --person-gap 0.8`: refused (never closer than 1.0 m); `move --forward
0.5 --person-gap 1.2`: refused (needs `--est`); `move --forward 0.5 --person-gap 1.2 --est 1.7`: refused (the k
term, a2b 21.9: the drone may fly ~0.27 m beyond 0.5 m at D 2.0); `move --back 1.0 --no-wait`, then at once `stop`:
`STOP_CMD`, it holds; a plan toward a person with `--plan-via drone`: refused before anything is sent ("goes
--plan-via mac").

**K4. Flight acceptance** (a gym-like room, D 4-5 m to an empty wall, the user 2 m away with the kill switch,
`stop` ready; `daemon --allow-flight`):

1. A 20 s plain hover (the trim check): `sortie --mode flight --confirm-flight --height 100 --photos 1 --wait-s 25`
   (the default `--then wait`: `--then land` lands as soon as the wait starts, a2b 21.9), then `land` after 20 s of
   hover (or let it land by itself, HOVER_TIMEOUT at 25 s). The trim check passes with no drift a photo shows.
2. A planned 1.5 m forward, no person: `sortie --mode flight --confirm-flight --height 100 --photos 1 --wait-s 40
   --plan-forward 1.5 --plan-via drone`, then `wait-move --photo`, `land`. Pass: DONE, the travel within +-0.15 m by
   the photo.
3. A planned 3 m toward the empty wall (D >= 4.6 m): `--plan-forward 3.0`. Pass: DONE; the end distance within 15 %
   of plan by the photo; `python3 tools/hold_band.py <dump.csv> <flightId> --moves` OK (not ROCKING). From the end
   photo the room's k = ln(D0 / D_planned_end) / ln(D0 / D_photo_end): >= 0.7 before longer moves in this room (a2b
   21.7 step 4). The camera height through it (a2b 21.9): in the dump `cvGain_dpm` before and after the move (the
   tracker's, it stays unless the drone bobbed), no growing `vzV_mms` / `cvV_mms` oscillation in the move's last 5 s
   and the hover after it, `hMm` steady; an oscillation or a PALM / VZ LAND there: report, keep moves <= 2 m.
4. A diagonal: `--plan-right 0.8 --plan-forward 2.5`. Pass: DONE, >= 0.55 m sideways (the sideways part comes out
   short by ~D_avg / D0).
5. Only then a person approach with the gap rule of a2b 21.7 (>= 1.0 m + 0.15 x E; + 0.3 m before the trim check
   passes; and its k term), `--person-gap` and `--est` given, `--plan-via mac` (or a plain `move`).

After each: `dump`, `flights`; in the dump `mvEnd` (7 = FRONT), `mvProgFwd_e4` against `mvRefFwd_e4`.

| What you see | Why, and what to do |
|---|---|
| `plan.state` `refused`, `why` TOO_CLOSE, `fwd_max_m` | the scene ahead limits the move at this D: `move --forward <fwd_max_m>`, photo, judge again; or turn toward a deeper background |
| `plan.state` `refused`, `why` NO_DIST | no tracked scene distance yet: `state` again, then a plain `move` |
| `end` PROGRESS with `reached` true | stopped at its end by the guard (90-110 % measured, on its line): treat as done |
| `end` PROGRESS, "overshoot by X m" or "off its line by Y m" | the guard caught it past its end or off its line: the drone holds there, nearer what is ahead or beside than planned; photo before the next move |
| a refusal "may fly X m beyond ... from the person" | the person gap's k term (a2b 21.9): fly `fwd_max_m` or less, photo, judge again |
| a second long move ends SHORT or PROGRESS | later MOVEs fly at the hold's gain scale 1 (a2b 21.3: ~12 % at D <= 4.7 m in the sim); it holds where it ended: photo, then the rest |
| `end` PROGRESS early, the measured forward under 0.25 x the reference while the photo shows motion | the frozen-view rule on a slow-tracking camera (a2b 21.8): note it, report; the drone holds |
| `end` FRONT | the scene ahead got close (0.40 m at k 0.6): holding; take a photo |
| a battery refusal ("the longest along this line that fits") | under 3.25 V loaded: fly the shorter move, or `--batt-ok` (a BATT_LOW partway lands in place) |
| rocking during or after a long move (`hold_band.py` ROCKING) | `holdk set 1` (live, not saved), fly again, report |

### L. Claude decides: hold and report (user decisions 2026-10-14 ~20:30-20:40)

docs/autonomy-a2b.md section 22. The drone lands for nothing but the battery limit, a crash or flip (the cut), a
lost Mac link and the k kill; everything else holds and reports (`drone_event` lines from the daemon, `hold` in
every reply). Claude must LAND: nothing ends a hover but the battery otherwise. Flashed only after the user's OK.
Since the user's decisions B and P (a2b 22.9, 22.10): after a contact every LAND of that sortie is the blind one,
the controller's own guards included, and the Mac has no `approach` (L9, L10).

**L0. After flashing:** `auto unlock`, `status`: `heapInt` with the Mac linked (the daemon running) and the camera on.
**Below 19 KB: NO-GO, nothing flies** (not DRY either: report it; the fix is more once-per-event code moved out of
IRAM, a2b 22.8). Expect ~19.7 KB: `_heap_start` 304 B above 56bd936's (the .bss; the IRAM is back in 56bd936's
256 B step since `startLand()` went to flash, a2b 22.6; B and P change neither, a2b 22.9; the blind LAND's floor
costs 8 B of .bss, a2b 22.10). Under ~19.3 KB: report it
before going on. `auto status`
prints its `OK auto report` line (events, bits); `rec info`. Since 2026-10-16 (a2b 26): `flights`, `camvz`,
`auto status`, then `status` again: **`diagStackFree` >= 1024** (the console task's least free stack since boot, bytes;
expect ~2 KB or more; `espfly_diag.py status` warns below 1024). Below 1024, or a `crash` naming `stack overflow in
task USBDIAG`: report it before going on.

**L1. GO / NO-GO.** The A4 sequence: `activeLoopsPerS` >= 955, `bypassSeen=0x00` (nothing new runs from flash on
every loop; at 10 Hz the status's report block and once a second the controller's quiet-hover window, a2b 22.6).

**L2. DRY: the wait no longer lands.** `sortie --mode dry --height 100 --photos 1 --wait-s 5`, then wait 15 s:
`state` still HOVER, `waiting` true, `hold.events` empty, `waitLeftMs` 0. `land`.

**L3. DRY: a tap.** In a DRY wait, a firm tap on the frame (the bench test of a2 12.7): no LAND; a `drone_event`
CONTACT (detail phase HOLD), `hold.contact.peakAcc_g` >= 1.6, `hold.contacts` 1; `hold.bits` `hush` for 0.5 s. Three
taps within 2 s: the third reported with `noHush`. The loop rate while the contact rules run (they call flash every
loop through the hush and until the camera height is back): `auto prof reset` before the taps, the two `camvz prof`
reads 3 s apart across the tap train (A4), and `auto prof` after: `activeLoopsPerS` >= 955, `maxGapUs` no worse than
L1's; below 955: NO-GO, report. A tap from the front, then from the left: `hold.contact.obstacle_bearing_deg` near 0,
then near -90 (the sign of the body-frame force, which nothing in the firmware acts on yet: a2b 22.7). `land`.

**L4. A YAW without a view** (FLIGHT over the mattress at 0.60 m; DRY flies an ideal view): cover the lens with a
card, `yaw --deg 120`: refused NO_VIEW (a blind turn at most 90 deg); `yaw --deg 60`: accepted (a blind turn,
`mvFlags` blindTurn), the turn runs, its dwell and bob wait for the view; uncover. `land`.

**L5. Hover over the mattress, the camera covered** (FLIGHT, 0.60 m, `stop` and the kill ready): cover the lens with
a card for 2 s, then uncover: CAM_LOST (`hold.sinkIn_s` counting from ~4.7), CAM_BACK, the height steady (by eye:
no climb). Cover it for good: SINK at 5 s (a slow descent), uncover during the descent: RESUME, HOVER again (if the
camera height comes back 0.5 s before the touchdown). The loop rate while covered (the camera rule, the view rule
and the controller's open loop call flash every loop while the condition lasts; the camera task keeps streaming a
blank view): `auto prof reset` before covering, two `camvz prof` reads 3 s apart while covered, `auto prof` after the
LAND: `activeLoopsPerS` >= 955, `maxGapUs` no worse than L1's; below 955: NO-GO, report. `land`.

**L6. A hand under the hover** (FLIGHT, 0.60 m): bring a flat hand up under the drone slowly: the drone holds above
it (a CONTACT if the touch is firm), it does not land by itself; `land` onto the hand: TOUCHDOWN on it. After a
CONTACT that LAND is the blind one: the props stay near the hover thrust on the hand for up to ~5 s before they stop
(the glue sim: 4.7 s; a2b 22.9); `k` if they must stop at once. Never grab it: a held drone pushes at its hover
thrust until Claude lands it or k (a2b 22.7).

**L7. The wall** (FLIGHT, 1.0 m, 1.0 m from a wall, facing it): `move --forward 0.8` (the Mac refuses beyond its
margins; to test the contact use `--no-wait` and a padded wall): a CONTACT with `obstacle_bearing_deg` near 0
(ahead), the motion stopped (`mvEnd` CONTACT), the drone backs off by itself to ~0.25 m from the contact point (the
hold's anchor, a2b 22.8) and holds there: watch 20 s that it does not drift back onto the wall (the sim's lowest gap
0.14 m over 42 s); then `move --back 0.5`, photo, `land` (blind after the contact, a2b 22.9: it creeps forward
about 0.2 x the height on the way down, so the back move comes first). Report the rise after the contact from the
dump (`hMm`; the sim's bound is 0.10 m).

**L8. DRY: the planned move after an event** (a2b 22.8): `sortie --mode dry --height 100 --photos 3 --gap 1000
--plan-forward 1.0 --plan-via drone`; once the daemon's line shows HOVER (the three take-off photos take ~3 s before
the wait), a firm tap: the reply's `plan.state` cancelled, `plan.events` CONTACT, `mvSeq` 0, the drone hovering in
the wait; the same without the tap: the plan runs. `land`.

**L9. The blind LAND after a contact** (user decision B, a2b 22.9 and 22.10). DRY first: a tap in the wait (CONTACT),
20 s later `land`: `state` reason LAND_CMD, then IDLE TOUCHDOWN; in the dump the LAND's thrust is flat (~0.98 x the
measured hover, no loop) for its flat part, then falls. The same without the tap: the camera's LAND (the thrust
follows the loop). Then FLIGHT over the mattress at 0.60 m (`stop` and `k` ready): a firm tap 10 s into the wait, 20 s
later `land`: a slow start, a touchdown by eye under ~1 m/s, no climb at the LAND's start (dump `hMm`). At the floor
the props may stay near the hover thrust a few seconds before they stop: the flat part runs at least its base (2.0 s
at 0.60 m), then the drone's rest ends it and the thrust falls to the touchdown in ~2.3 s. The sim, from 1.00-1.20 m:
up to 4.5-5.1 s with the motors on under 0.12 m (the last ~0.5 s of the descent included); up to ~6.7 s when the
hover was already at the floor. Report it if the props run longer than ~7 s at the floor, or the drone leans or
creeps on the floor. Once more with the battery: a tap, then hover until BATT_LOW lands it. That is the long-hover
case: the same slow descent, the same floor times (a2b 22.10: until then this float lasted up to 15 s after a long
hover). The sim, with a step sag at the end of the battery: <= 0.96 m/s after 60 s of hover, <= 1.13 m/s after 100 s.
Report anything that touches down harder than ~1 m/s. Once with the camera covered after the tap (a SINK): it may
float at the floor until its whole flat part has run (no speed to see the floor by without the camera; the sim up to
9 s, a real leg touch ends it sooner); `k` if it must stop.

**L10. No `approach`** (user decision P): `python3 tools/espfly_auto.py approach --bearing 0 --to 1 --est 2`: a usage
error, nothing sent. Toward something in a photo: `yaw --deg B`, a photo, `move --forward F` (with `--person-gap G
--est E` toward a person).

| What you see | Why, and what to do |
|---|---|
| `drone_event` CONTACT | photo first; `obstacle_bearing_deg` is where it was hit from (0 ahead, +90 right): back away from it, or LAND on a hand or if it is caught; every LAND of this sortie is now the blind one (slow start, up to ~1 m/s at touchdown; on a hand or at the floor the props run up to ~5-7 s): back away 0.3 m or more from an obstacle before a LAND near it |
| a LAND that starts slowly and touches down firmly after a contact | the blind LAND (a2b 22.9, 22.10): expected; it ignores the camera's height estimate whatever started it (Claude, the battery, the link, the phone, a SINK, the controller's own guards) |
| after a contact, the props keep running a few seconds on the floor before they stop | the blind LAND's flat part ends when the drone stops descending (22.10): up to ~7 s in the sim from 1.2 m, longer with the camera covered (a SINK). `k` stops them at once |
| `approach` is a usage error | removed (a2b 22.9): `yaw` to the bearing, a photo, then `move --forward` (with `--person-gap --est` toward a person) |
| `drone_event` CAM_LOST, `sinkIn_s` counting | photo; a textured scene and the camera back: go on; else LAND before the SINK (or let it sink) |
| `drone_event` SINK | the drone descends slowly; it resumes if the camera height comes back; LAND takes it over |
| `drone_event` VZ (K_RANGE, HOLD_CAPPED, EST_RANGE, CONTACT_HOLD) | the controller is at a bound and reported it: LAND soon |
| `drone_event` HOLD_NO_VIEW | the hold has no fresh view (the drone drifts): photo, turn toward texture, or LAND |
| `drone_event` MOVE_STOPPED / RETRACK_UNDO | the drone stopped the motion itself (view, tilt, loop gap) or turned a YAW back: photo, replan |
| `ceiling_warning` | removed 2026-10-15 (a2b 23.4): no longer sent; the height routine (M, a2b 23.5) checks the height and the ceiling from the photos. Its successor `height_check_due` (a warning only): the latest photo or set-height is older than the runbook's 15 s (10 s above 1.20 m commanded): take a photo |
| `sortie_holding` | the sortie's client went away in the air: the drone holds; send the next command or LAND |
| `takeoff_late` | a TAKEOFF took effect after its sortie ended (its answer late or lost): the drone flies on and holds; `state`, a photo, then decide |
| `plan.state` cancelled | the drone reported an event before the wait; the planned move did not start: photo, then decide |
| `drone_rebooted`, then `reboot_cleared` | the drone rebooted: ARM waits for its IDLE in the new boot (the reset lock after an in-air reset needs the user over USB) |
| `photo_sets_dropped` (with `drone_rebooted`) | the old boot's photo sets are forgotten (its ids start at 1 again): a set still being collected is answered as failed; the next `photo` of any id captures fresh (2026-10-22: before, an id saved in the old boot was answered with that boot's files) |

### M. The height from photos: `set-height` and the level line (user decisions 2026-10-15)

docs/autonomy-a2b.md section 23. The drone has no height sensor and its estimate drifts 0.01-0.03 m/s; Claude judges
the true height from photos (what lies on the level line) and sends `set-height X`: the drone shifts its estimate
by d = X - (its estimate when that photo was captured) and flies -d back at <= 0.15 m/s, so it ends at the height it
believed it held (~0.1 m under the commanded height: since section R's build it flies to the commanded height, a2b
27). Flashed only after the user's OK. Nothing here changes the loop: the new code runs from flash, once
per command.

**M0. After flashing:** L0 and L1 as before (`heapInt`, the loop rate; `_heap_start` 16 B above 1e4184b's: ~19.7 KB
expected, below 19 KB NO-GO; a2b 23.7). `auto prof`: the line ends with `shiftRefused=0`. `python3 tools/espfly_auto.py level-cal --show`: the setting (`setting` null before the first
calibration: `ground.tilt_deg` 1.9, the default, `stale` true).

**M1. DRY: the command.** `sortie --mode dry --height 100 --photos 1 --wait-s 5`; once the wait starts:
1. `set-height 0.80 --now`: OK; the reply `old_m` ~1.00, `offset_m` -0.20, `new_m` ~0.80, `fly_m` +0.20; the status
   right after shows hMm ~800, then the correction (wait-move names it `SET_HEIGHT`, `mvFlags` 0x40): DONE in ~2 s
   + the settle, `hMm` back near 1000 (the sim flies the plant up 0.20 m).
2. `set-height 1.20 --now --no-wait` twice in a row (the reply comes with the ACK): the first OK, the second BUSY
   (the correction running). `stop`, and within 2 s of the accepted one `set-height 1.10 --now`: SH_RATE ("wait
   2 s"); after 2 s: OK.
3. `set-height 1.80 --now` (d >= +0.6 from <= 1.2 m): SH_STEP ("send 0.50 m of it"). `set-height 0.05 --now`:
   refused locally (X 0.10..2.50).
4. `photo`, then `set-height 0.90` (the offset form, h_at from that photo): OK, `at_m` in the event; a second
   `set-height 0.90` without a new photo: refused by the Mac ("take a new photo or use --now").
5. `land`, then in IDLE `set-height 1.00 --now`: BAD_STATE NOT_WAITING (a nonzero detail).

**M2. The level line, on the ground.** The drone on the floor where it will take off, facing a wall 2-6 m away
whose floor-wall junction shows near the image centre. `photo`; open the `_x4.png`: a 1 px dashed yellow / magenta
line (dotted while the setting is stale), ticks at both edges and at +-1, 2, 5 deg on the right. Read the far
floor-wall row R at the centre column, measure D, `level-cal --floor-row R --dist D`: the reply's old and new tilt,
the redrawn line now solid and 3-7 px above that floor line. `photo` again: the same. `level-cal --show`. Then
`reboot` (or a battery swap) and `photo`: dotted again (`stale_why` boot), until the next `level-cal`.

**M3. The tape mark** (FLIGHT, over the mattress, `stop` and `k` ready; a tape mark at 1.0 m on a wall 2-3 m away,
a phone video from the side): take off to 1.0 m, `photo`: judge the height by the tape against the level line.
`set-height 0.80 --now`: the drone climbs ~0.20 m (check against the tape: the camera's scale is +-20-30 %, so
0.14-0.26 m), then `set-height 1.20 --now` (2 s later): it descends ~0.20 m. `land`. Report the climb and the
descent measured on the video.

**M4. 60 s at 1.0 m** (FLIGHT, the same setup): take off to 1.0 m; a `photo` every 10-15 s; judge the height
against the tape and the level line; `set-height X` (the offset form) when off by more than 0.10 m. Note each
photo's `h_est_capture_m`, the judgement, the reply and the time from the photo to the command (keep it under
~10 s). After 60 s, `land`. The video's height should stay within ~0.2 m of 1.0 m; the uncorrected drone drifted
0.6-1.8 m in 60 s (the 2026-10-15 living-room hover: about 1 m down in 105 s).

**M5. The 2.00 m cap** (a2b 23.9, 23.10; the hover by moves up: 2.00 m, 1.20 until this build; the take-off stays at
1.20 m). DRY: `sortie --mode dry --height 121`: a usage error, nothing sent. `sortie --mode dry --height 120 --photos 1
--wait-s 60`: the wait starts by itself (no `TAKEOFF_SLOW` event); `move --up 0.5`, `move --up 0.3` (hTarget 2.00),
then `move --up 0.1` refused by the Mac ("must stay 0.25..2.00 m"); `land`: the LAND takes up to ~8 s. FLIGHT above
1.2 m only with the user present, a full battery (>= ~3.25 V loaded), nobody within 2 m, and a ceiling judged >= 0.6 m
above the target (a 2.4 m ceiling: at most ~1.8 m): take off to 1.0 m, `move --up` in <= 0.5 m steps with a photo
after each (tape marks at 1.5 and 2.0 m), photo checks every 5-10 s up there (`height_check_due` says when one is
due), down to <= 1.2 m from ~3.20 V loaded or T_hold ~52000 (the drone refuses every MOVE and set-height from 53400),
`land` from the top: the LAND takes up to ~7 s (timeout 10.7 s). Report the touchdown and any CAM_LOST during a move
up.

| What you see | Why, and what to do |
|---|---|
| `set-height` BUSY | a MOVE, YAW or correction is running: wait for its end (<= ~7 s), then a new photo |
| `set-height` SH_RATE | another was accepted under 2 s ago: wait 2 s |
| `set-height` SH_STEP | more than 0.5 m (or, since R, a flight over 0.55 m: the Mac clips it itself): send the 0.5 m part, a new photo, again |
| `set-height` LOW_BATT, why UP_HEADROOM | since R: a climb without the motors' headroom (T_hold within 4600 of the ceiling, ~53400): come down or LAND; never a voltage rule |
| `set-height` HEIGHT | the corrected estimate would be under 0.10 m or over the cap + 0.5 m: the judgement is implausible, take a new photo |
| `set-height` NO_CAM_HEIGHT, NO_VIEW, SH_CONTACT | no camera height, no view, or a contact's hush or window (until the camera height is back 0.5 s): wait for CAM_BACK / VIEW_BACK or the window's end, then a new photo |
| `set-height` LOW_BATT | near the end of the battery (T_hold > 53400, or the BATT_LOW condition) no correction flies, up or down (a descent's braking there can end HOLD_CAPPED): LAND, or come down earlier (from ~52000) |
| `set-height` error with `outcome` "unknown" | no ACK: the drone may have applied it; take a new photo before the next set-height (the old one counts as used) |
| `level-cal` refuses the photo | it was taken before the landing / contact / KILL / reboot that made the line stale, or on another boot: take a new ground photo |
| "firmware has no SET_HEIGHT" | the drone runs an older build: `move --up/--down` by the judged error |
| the level line dotted | the tilt setting is stale (a new boot, a landing, a contact, a KILL): `level-cal` on the ground before the next take-off |
| the level line far from the room's converging horizontal lines in flight | the camera may have moved (a contact, a catch): stay <= 1.2 m, `level-cal --horizon-row` on a clean horizon cue, or LAND and recalibrate |

### N. Always-on colour (user requests 2026-10-15 "allow colour" and 2026-10-16 "always on, efficient")

docs/autonomy-a2b.md section 28 (section 24 was the first colour build, replaced). **Colour stays opt-in**: this build
boots with NVS `photocol` 0, the grey camera of before (Y8 output, grey photos, the 40-byte META, the camera path's
code as on 8277b68), so flashing it changes nothing in the camera path. With `photocol` 1 the sensor streams YUYV on
**every** frame and **every** photo is colour, for the whole boot: the driver's own copy hands the flow the Y bytes in
grey's frame buffers (same bytes, same code, same cost on the flow's core) and copies only the photo frame raw into
the photo slot. Nothing switches between grey and colour within a boot. The setting is read at boot only (`auto get`:
the stored value; `status` `camPix=`: the one running). Nothing below is measured on the host.

The daemon must be the one of this tree (`q` the running one, start `python3 tools/espfly_auto.py daemon` from this
checkout): it reads the 44-byte META and keeps each colour photo's raw `.yuyv` next to its PNGs, which N'1.4 needs.
**Order:** N'0, N'0g, N'1, N'1.3, N'1.4, N'1.5, N'2, N'3, the soak; N'4 and N'5 when the battery and a ground standby
come up. **Any hard fail** (marked): `python3 tools/espfly_diag.py auto set photocol 0`, `reboot`, report the numbers.
The default stays 0 until the user decides after N' (plan section 7 item 4).

**One format boot** (used below as "boot grey" / "boot colour"; the drone on the bench, props off, facing the same
lit, textured scene with a bright window or lamp in the view, the Mac linked, `flowfit` as flown):

```
python3 tools/espfly_diag.py auto set photocol 0      # 1 for colour
python3 tools/espfly_diag.py reboot                   # wait ~3 s
python3 tools/espfly_diag.py auto unlock
python3 tools/espfly_diag.py tkoff camwake 300        # the camera stays awake (no ground standby) for the bench
sleep 60                                              # auto-exposure and the clock check settle
```

**N'0. The colour boot.** Boot colour, then `python3 tools/espfly_diag.py status` (it prints a `colour:` line).

| Check | Pass |
|---|---|
| `camPix`, `camSplit` | `yuyv-split`, `ok` (`camSplit=bad`: **hard fail**; the tool prints a WARNING, the boot log has a `camera: colour split boot check failed (...)` line with the sizes) |
| the boot log (a plain serial monitor started right after the `reboot`, before anything else opens the port: `python3 -m serial.tools.miniterm /dev/cu.usbmodemXXXX 115200`; the camera starts ~1.5 s after boot) | `camera: colour split ok (fb 19200 recv 38400 half 3840 cnt 4)`, and the driver's `cam_hal: buffer_size: 15360, half_buffer_size: 3840, node_buffer_size: 3840, node_cnt: 4, total_cnt: 10`. If the monitor missed them: `camSplit=ok` already means the check saw exactly these sizes |
| `camFps` | 14.6-14.9, within 0.1 of the grey boot's (N'0g) |
| `camClk`, `camClkChk`, `camClkFallback`, `camSkips` | 13.33, `ok`, 0, 0 |
| `heapInt` (the Mac linked) | within +-200 B of the grey boot's (the build's internal RAM is the same for both formats) |
| `diagStackFree` | >= 1024 (L0) |
| `python3 tools/espfly_auto.py photo` | the reply's frame `color` true and a `raw` `.yuyv` path; the `_x4.png` a colour image with the ticks and level line |

**N'0g. Grey on this build** (the regression check against 8277b68): boot grey, then three `status` 10 s apart.
`camPix=gray`, `camSplit=off`, `camSkips` 0, `camClkFallback` 0, and a `photo` with `color` false. `camProcAvgUs`:
**information only** against 8277b68's earlier readings (the logged day-to-day spread of grey is 59.5-60.3 ms with
the scene and the warmth, wider than +-200 us); the hard marks are the absolute ones (<= 60500 us, a fail over 61000
or any `camClkFallback`) and N'1's same-session comparison.

**N'1. The flow's time, grey against colour.** Four boots in the order grey, colour, grey, colour; in each, three
`status` 10 s apart (after the 60 s settle). Note `camProcAvgUs`, `camProcMaxUs`, `camSkips`, `camFps`,
`camClkFallback`, `heapInt`. No photo during N'1 (the photo frame is N'1.3).

| Check | Pass |
|---|---|
| mean colour `camProcAvgUs` - mean grey `camProcAvgUs` (6 readings each) | <= +150 us (estimated +6 to +50 us: the driver's 5 extra end-of-buffer interrupts a frame land on the flow's core; a2b 28.5) |
| colour `camProcAvgUs` | <= 60500 us |
| colour `camProcMaxUs` | <= the grey boots' max + 500 us |
| colour `camClkFallback`, `camSkips` after 5 min (read again at the end of the second colour boot) | 0, 0 |

**Hard fail:** a colour `camProcAvgUs` over 61000, or any `camClkFallback=1`.

**Repeat rule (review 2026-10-16):** when only the +150 us difference fails (every absolute mark passes), run one more
grey / colour pair of boots (three readings each) and judge the mean of all the pairs; fail only if it still exceeds
+150 us. The EMA's readings 10 s apart in one boot are correlated, and the scene and warmth move grey by up to ~0.8 ms
between sessions, so one pair can fail on drift alone.

**N'1.3. The photo frame.** In each format (one boot each, after N'1's readings): three
`python3 tools/espfly_auto.py photo --n 3 --gap 150` 10 s apart, then `status`.

| Check | Pass |
|---|---|
| colour `camProcMaxUs` | <= the grey boot's `camProcMaxUs` after its photo sets (in colour the flow copies no photo: estimated 0.5-1 ms *cheaper* than a grey photo frame). One excess under 200 us: run the three sets once more in both formats and compare again |
| colour `camSplitDone` (the `colour:` line: "filled") | up by 9 (the photos taken) against the `status` before the sets; `camSplitRearm` up by <= 1; `camSplitCancel` and `camSplitRestart` unchanged |
| `camSkips` | unchanged by the sets |
| the photo replies | all `ok`, every frame `color` true |

**Photo latency.** Then five `python3 tools/espfly_auto.py photo --n 1` 5 s apart in each format; note each reply's
`elapsed_s`. Pass: median colour - median grey <= 0.30 s (**this mark needs the user's OK before the bench:** the plan said
"reply <= grey + 250 ms"; the implementer redefined it as a median over five single photos with 0.30 s, which adds
the ~0.1 s of the 38-chunk push; with the plan's wording keep +0.25 s on single photos) (estimated +0.07 to +0.19 s to the next frame start, its
capture and the flow's processing, plus ~0.1 s to push 38 chunks instead of 19); every colour reply <= 1.5 s. A set
of 3 (`--n 3 --gap 150`) may take up to ~0.55 s longer than grey's (3 x ~0.1 s of push, the frames' latencies overlap
the gaps): information only.

**N'1.4. The luma is grey's (direct, hard gate).** The drone still, the same static scene with the bright window
(auto-exposure settled: 60 s after the boot), back to back:
1. boot grey: `python3 tools/espfly_auto.py photo --n 1`: the grey photo `G` (its `.json`);
2. boot colour, the drone not moved: `python3 tools/espfly_auto.py photo --n 1`: the colour photo `C`;
3. `python3 tools/espfly_diag.py colourcheck G.json C.json`.

| Check | Pass |
|---|---|
| mean \|Y grey - Y colour\| (the colour photo's Y bytes from its raw `.yuyv`) | <= 2 levels |
| min and max Y | the same within 1 |
| max Y | >= 250 in both (a bright window in the view) |

**Hard fail** (the tool says FAIL and exit code 1): the ISP limits Y in YUV mode (for example to 16-235), which would
silently break the flow's saturation gate (OF_SAT_LEVEL 250) and the exposure checks, or a byte-phase error (a dark,
textureless Y). Grey stays (`photocol 0`). Then the flow statistics, as before, with the drone still:
`python3 tools/espfly_diag.py stream --hz 50 --seconds 20 --quiet --out /tmp/n1-gray.csv` in a grey boot and
`... --out /tmp/n1-yuyv.csv` in a colour boot, compared with:

```
python3 -c "import csv,statistics as S,sys; [print(f, 'frames', 1 + sum(r[i]['flowSeq'] != r[i-1]['flowSeq'] for i in range(1, len(r))), *('%s %.2f' % (k, S.mean(float(x[k]) for x in r)) for k in ('flowLum','flowTexture','flowResidual_dg','flowOk')), 'dx sd %.2f dy sd %.2f' % (S.pstdev(float(x['flowDx_cpx']) for x in r), S.pstdev(float(x['flowDy_cpx']) for x in r)), 'dt max', max(int(x['flowDt_100us']) for x in r)) for f in sys.argv[1:] for r in [[x for x in csv.DictReader(open(f)) if int(x['flowSeq'])]]]" /tmp/n1-gray.csv /tmp/n1-yuyv.csv
```

Pass: `flowLum` within 3 %, `flowTexture` within 5 %, `flowResidual_dg` within 10 %, the `flowOk` rate within 2
points, the dx / dy noise at rest within 20 %; `frames` (the frames processed: `flowSeq` changes) >= 285 in both
(grey gave 298 in 20 s, the first colour build 152); `dt max` <= 700.

**N'1.5. The colours (U/V order and range).** In a colour boot, each with one `photo --n 1` and `colourcheck` against
the grey photo `G` of N'1.4 (its Y check is for N'1.4 only; here read the colour lines):
1. a white or grey sheet filling the view: `colourcheck G.json W.json --white`: mean U and V 124-132 and R / G / B
   within 10 of each other: PASS (a fixed tint here is the sensor's white balance: report it, nothing to tune);
2. the orange tiger filling most of the view: `colourcheck G.json T.json --orange`: R > G > B: PASS. Swapped U and V
   show it blue (B > G > R): **hard fail**;
3. a red and a blue object side by side: open the raw `.png`: red reads red, blue reads blue;
4. `python3 tools/espfly_diag.py flow-cal --offset-scan` (hand-held, a2b 17.2) in colour: the gyro window offset ~26
   ms as in grey.

**N'2. The loop rate in colour:** A4 as written in a colour boot: `activeLoopsPerS` >= 940 and within 10 of the grey
build's (971.6), the two `camvz prof` reads ~972/s; `auto prof` `stabStackFree` and `status` `diagStackFree` within 64 B
of a grey boot's, `camStackFree` (the driver's copy task) within 128 B of grey's. Below 940: `photocol 0`, A4 again,
report both.

**N'3. No frame lost to a photo** (in a colour boot; the Mac link tool alongside the console):

```
python3 tools/espfly_diag.py stream --hz 50 --seconds 12 --quiet --out /tmp/n3-stream.csv &
sleep 2; python3 tools/espfly_auto.py photo --n 3 --gap 150; wait
python3 -c "import csv; r=[(int(x['flowSeq']),int(x['flowDt_100us'])) for x in csv.DictReader(open('/tmp/n3-stream.csv')) if int(x['flowSeq'])]; s=[a for i,a in enumerate(r) if i==0 or a!=r[i-1]]; print('frames',len(s),'seq gaps',[(b[0]-a[0])%255 for a,b in zip(s,s[1:]) if (b[0]-a[0])%255!=1],'dt_100us max',max(d for _,d in s))"
```

Pass: no `seq gaps`, `dt_100us max` <= 700, `camSkips` unchanged, the photo `ok` with three `color` frames. Then one
more set with the diag tool closed and a plain serial monitor on the drone's port: no `cam_hal: EV-` line (an event
queue overrun; the split keeps grey's 3-deep queue). Then the same in the hover wait of a DRY sortie (`sortie --mode
dry --height 40 --photos 3 --gap 150 --wait-s 20`, the stream started 2 s before the take-off photos): no long
interval, no `EV-` line.

**N'3b. A flash write under the stream** (review 2026-10-16: the split's chunk is 12 YUYV lines, ~6.6 ms, against
grey's 24 Y8 lines, so the driver's 4-half ring, 3-deep queue tears a frame without any `EV-` line after ~13-20 ms of
core-0 stall instead of grey's ~26-40 ms; an NVS write stalls the caches. In flight NVS is written only on the ground
with the motors off, so this is a bench check of the margin). In a grey boot and then a colour boot, the camera awake
(`tkoff camwake 300`): `status` (note `camSkips`, `camBad`), `auto get` (note the stored `vzK`), then three
`python3 tools/espfly_diag.py auto set vzK <that same value>` 2 s apart, then `status` again; then the same with a
plain serial monitor instead of the diag tool for the log lines (`FB-SIZE`, `FB-OVF`, `EV-`). Pass: colour's
`camSkips` / `camBad` and the log lines no worse than grey's (each format: at most 1 skip per write). Worse in colour:
report; it does not block the bench, NVS is never written in the air.

**The camera's stall restart in colour** (the 4 s stall re-init: `esp_camera_deinit` + `cameraStart`, the split's
reset and its geometry check run again) is covered on the host only (test_cam_split, test_cam_split_wrap): no console
command forces it. If a `status` ever shows `camRestarts` > 0 in a colour boot, also read `camSplit` (must stay `ok`),
`camSplitRestart` (up by one per restart) and take one `photo` (must be `ok`, `color` true).

**N'4. Near the end of the battery** (a colour set keeps the Wi-Fi transmitting ~0.6 s instead of ~0.3 s): on a battery
near its limit, a DRY sortie's hover wait with a set of 3 in a grey boot and one in a colour boot; `flights` / `live`
`railMin`: colour within 50 mV of grey. Lower: `photocol 0` for flights near the battery limit, report (the reserve: a
4:2:0 or 80x60 chroma wire format, a separate build with a new Mac pixFmt).

**N'5. Wake.** In a colour boot without `tkoff camwake`, let the camera sleep on the ground (`status` `camGnd=1`), then
`python3 tools/espfly_auto.py photo --n 3 --gap 150`: the reply `ok`, all `color` true, exposure normal (the frames
wait AUTO_CAM_WAKE_FRAMES = 6 fresh frames after the wake, as in grey).

**Soak.** A colour boot streaming 5 min (`stream --hz 10 --seconds 300 --quiet --out /tmp/soak.csv`) with a `photo --n
3` every 30 s from a second terminal: `status` at the end: `camRestarts`, `camBad`, `camTimeouts`, `camSkips` all 0;
`camSplitDone` = photos taken, `camSplitCancel` 0.

| What you see | Why, and what to do |
|---|---|
| `camSplit=bad` with `photocol` 1 | the driver's sizes are not the split's (another esp32-camera version, another sensor): grey this boot; report the boot log line |
| `camPix=gray`, `camSplit=off` with `photocol` 1 stored | the setting is read at boot: reboot |
| a photo times out on an older Mac tool | it cannot read the 44-byte META: restart the daemon from this tree, or `photocol 0` |
| `colourcheck` "without its raw .yuyv" | the photo was saved by a daemon before 2026-10-16: restart the daemon from this tree, take it again |
| `camSplitRearm` climbing with every photo | the photo frames are dropped before the flow takes them (GRAB_LATEST): report with `camSkips` |
| a colour cast | the sensor's auto white balance (as in grey): judge by shapes and colours; a per-room R/B gain for the display copy is a later Mac change |

### O. The late offset noise (R1 of the height study; built for host tests, flashing needs the user's OK)

docs/autonomy-a2b.md section 25 (25.9: the review fixes). NVS `cvqlate` (1..5 since the review, default 1 = off: the
observer of before, bit for bit) and
`cvqlates` (20..120 s, default 40), read at boot: `auto get` shows the stored values, `camvz` the ones running
(`qlate= qlateS= qlateCfg=`). Nothing here flies; O4 is a DRY sortie. Host counts of this build against 65dfcba (the
same host, `run_ctests.py`): 36 C suites before, 37 after (`test_cam_vz_late` new, 695 checks); changed:
`test_cam_vz_obs` 2881 -> 3043, `test_cam_vz_glue_host` 29 -> 52, `test_autonomy_glue_host` 439 -> 443,
`test_auto_glue_core` 192 -> 195, `test_flight_summary` 48 -> 49; every other suite's count unchanged, all pass but
`test_gyro_tempco` (33 failures) and `test_mahony_replay` (4), which fail the same checks on 65dfcba (the logs added
to the cal directory since they were pinned). Python: `test_espfly_diag` 78 -> 79 tests, `test_hold_band` 13,
`test_espfly_auto` 203, all OK. After the review fixes (a2b 25.9): `test_cam_vz_obs` 3053, `test_cam_vz_late` 1385,
`test_cam_vz_glue_host` 54, `test_autonomy_glue_host` 446, `test_auto_glue_core` 196; the rest as above.

**O0. After flashing:** L0 and L1 as before (`heapInt`: `_heap_start` 24 B above 65dfcba's; the IRAM in the same 256
B step). `flights`: the summaries of the previous build are gone once (the record grew, a new magic). `camvz`:
`qlate=1 qlateS=40 qlateCfg=ok late=off lateMs=0`.

**O1. The setting and its readback:**
1. `python3 tools/espfly_diag.py auto set cvqlate 6`: `ERR auto set: cvqlate must be 1..5 x`; `auto set cvqlates
   19`: `ERR ... 20..120 s`. Nothing stored (`auto get`: `cvqlate=1(default) cvqlates=40(default)`).
2. `auto set cvqlate 5`, `auto set cvqlates 20`, `reboot`, wait ~3 s, `auto unlock`: the boot log line "late offset
   noise x5 from 20 s"; `camvz`: `qlate=5 qlateS=20 qlateCfg=ok late=off`.

**O2. The bench sortie's path** (props off, the drone on the bench; `camvz air full` makes it look airborne, frames
pass the quality gate): with O1's x5 from 20 s, `raw "camvz air full"`, wait 30 s, `camvz`: `late=guard` (the step
came 20 s after the bench lift-off; a bench without vertical motion has no tracked G, a guard event, so the multiplier
stays off while its frame path runs every frame), `lateMs=0`, `biasHeldMg` the offset at the step (frozen while the
guard holds), close to `biasMg` on a still bench; `raw "camvz air off"`.

**O3. The loop rate with the frame path on:** A4 as written, with O1's x5 from 20 s and the DRY sortie lasting at
least 30 s after the take-off (`--wait-s 30`), so the bench observer runs its late path: `activeLoopsPerS` >= 940,
GO; `camvz prof` frame max/avg within ~10 % of the x1 run's (the frame path adds ~40 flops a frame); `stabStackFree`
>= 1024 (the summary's copy at 100 Hz grew 8 B; a2b 25.7).

**O4. DRY with x5 from 20 s:** `sortie --mode dry --height 40 --photos 1 --wait-s 40`; at the end `flights`: the last
`F` line `cvq=5 cvqS=20`; `python3 tools/espfly_diag.py flights` prints "late noise x5 from 20 s: ...". (DRY flies
the sim plant: the observer is not used by the controller there; the readback only. Since the review a DRY sortie also
sets the observer's motion flag; the observer only reaches its late path on the bench with `camvz air full` set
before the ARM, which DRY allows: then a MOVE in the wait shows `late=guard` in `camvz`.)

**O5. Back to off:** `auto set cvqlate 1`, `auto set cvqlates 40`, reboot: `camvz qlate=1 qlateS=40`. A FLIGHT with
the late noise on needs the user's OK (docs/autonomy-a2b.md 25.8; the study's F1-F4 flights, its section 6).

| What you see | Why, and what to do |
|---|---|
| `qlateCfg=bad` in `camvz`, "REFUSED" in `flights` | a stored `cvqlate`/`cvqlates` out of range or unreadable at boot: x1 runs; `auto reset cvqlate`, `auto reset cvqlates`, set again, reboot |
| `camvz` `qlate` differs from `auto get` | the setting is read at boot: reboot |
| `late=guard` all the time in a hover | a guard event keeps coming (the camera refuses frames, a G that is not tracked, the offset moving > 1 mg/s): note `camvz` `lastFrame`, `rejected`, `alarms`, `gainTracked`; report |

---

### P. The USB console's stack (2026-10-16: `status` crashed 82bcefb)

docs/autonomy-a2b.md section 26. The console's command handlers run out of line and `status` is formatted in pieces:
the same line, the same fields in the same order, `diagStackFree=` appended last. Nothing in the flight code changes;
static IRAM, .data, .bss and `_heap_start` are 82bcefb's to the byte (`heapInt` as there).

**P0. After flashing:** `auto unlock`; `status` five times in the first 30 s after the boot, then `crash` (no new
record), `flights`, `camvz`, `auto status`, `auto get`, `auto prof`, `net`, `gtc show`, `tkoff`, then `status`:
`diagStackFree` >= 1024 (expect ~2 KB or more). Then `auto set photocol 1`, reboot, `auto unlock`, the same with the
Mac linked (the daemon running) and the camera on; then `auto set photocol 0`, reboot. Any crash, or `diagStackFree`
below 1024: report the `crash` record and the last `status`.

### Q. The late-sink fix T2 (the offset drift rate; built for host tests, flashing and flights need the user's OK)

docs/autonomy-a2b.md section 29. NVS `cvt2k` (0..200 = kR x 100, default 0 = off: the observer of before, bit for
bit) and `cvt2r` (-350..350 ug/s, default 124: the prior 0.124 mg/s), read at boot; `cvt2k` > 0 together with
`cvqlate` > 1 is refused (T2 off, `t2Cfg=bad`). **Read 29.5 first: several of the plan's gates are not met** (host
marks i, ii and iv; replay G4, G5 and G7 as worded). Flying T2 is the user's decision on those numbers; Q0, Q1,
Q5 and Q6 are bench checks of the build and need no decision beyond the flash.

**Q0. After flashing:** P0 / R0 (`status` x5, `diagStackFree` >= 1024: `camvz`'s frame grew 944 -> 1088 B and
`flights`' 1056 -> 1104 B, so run `camvz` and `flights` before the last `status`), L0 (`heapInt`: 16 B less than the
build of 28), A4 >= 940 loops/s. `flights`: the summaries of the previous build are gone once (magic E9). `camvz` and
`status`: `t2k=0.00 t2r=+0.124 t2Cfg=ok t2=off`.

**Q1. The setting:** `auto set cvt2k 201` and `auto set cvt2r 351`: ERR, nothing stored. `auto set cvt2k 50`,
reboot, `auto unlock`: the boot log "T2 k0.50 r+0.124 mg/s"; `camvz` `t2k=0.50 t2=wait`. With `cvqlate 5` stored as
well: "cvt2k 50 with cvqlate 5 refused", `t2Cfg=bad`, `qlate=5` (set `cvqlate 1` and reboot to clear it).

**Q5 (the plan's). The load:** the camvz air load test (`raw "camvz air full"`) with `cvt2k 50` for >= 60 s, then A4:
>= 940 loops/s and within 10 of the `cvt2k 0` run; heap with the Mac linked >= 12288 B (bench ~18.4 KB).

**Q6 (the plan's). DRY:** `sortie --mode dry --height 100 --photos 1 --wait-s 50` with `cvt2k 0`, then with `cvt2k
50` (reboot between): `cvFlags` 151 in hover; `camvz` / `status` read back `t2k`, `t2r`, `t2` (`wait` before 30 s,
then `on` / `frozen` / `hold`), `t2OnS`, `t2bR`; `diagStackFree` ok; the last `F` line `cvt2k=50 cvt2r=124 cvt2Ds=..
cvt2bR=end,min,max`, and `espfly_diag.py flights` prints "T2 k0.50 r+0.124: on .. s". DRY flies the sim plant (the
controller does not use the observer there): this is the readback only; with `camvz air full` set before the ARM the
bench observer runs T2 (no vertical motion means no tracked G: `t2=hold` most of the time, as R1's O2).

**Q-flights (each needs the user's OK; the plan's section 6).** `cvqlate 1` first (NVS still holds `cvqlate 5`,
`cvqlates 40` from step 2). **Never turn T2 and colour on for the first time in the same flight** (review 2026-10-16:
colour adds core-0 load, which can make camera gaps likelier, and a camera gap is T2's worst case, G4): F0 checks
colour with `cvt2k 0`; if colour is not yet bench-passed (N'), fly F-T2a/b with `photocol 0`. F0: colour on, `cvt2k 0`, 90 s, no yaw (the A/B reference). F-T2a: `cvt2k 50`, 90-120 s,
no yaw, a photo every 10 s, measure only; correct in the first 30 s as usual, after 30 s only if > 0.35 m off. Pass:
the true height within +-0.15 m from 30 to 90 s, never > +0.25 m above. F-T2b: the same with a 90 deg yaw at ~45 s;
pass +-0.25 m. After each: `dump`, `flights`, anchor the photos (sink-replay/mkin.py style) and check the replay
predicted the late sink rate within ~30 %. **Abort (LAND at once)** on a photo judged > 0.5 m above or below the
commanded height (a climb counts as much as a sink: from a 1.0 m hover +0.5 m is half way to the 2 m rule, and G4 / G6
climbed +1.0..+1.9 m in the replays with nothing on board to see it), a contact, a floor view, or the user's call.

**The Claude procedure with T2 (required: T2's new risk is an unseen slow climb under a steady camera speed bias):**
1. First photo 5-8 s after HOVER, then one every 10-15 s, each judged and sent within 10 s.
2. A fresh photo after every yaw, camera gap or contact; never act on a photo older than ~10 s (the 79 s correction
   of the tiger flight came from a 30 s old photo). **After CAM_LOST, CAM_BACK or HOLD_NO_VIEW a photo within ~5 s**
   (the daemon's hints say so): after a gap T2's estimate can sit low and the drone climb (G4). The daemon now warns
   when a set-height's photo predates an event (`events_since_photo`).
3. SET_HEIGHT up to the 0.5 m limit, in two steps when farther off, with the target form of section R.
4. **LAND at once on a floor or table-top view.** T2 makes K_RANGE fire less on a surface (as R1 would): the drone can
   sit on the floor with the motors on longer than before.
5. Watch the photos for a climb as much as for a sink: in the replays a 1-2 cm/s downward camera bias took the p2
   flight 1.1-1.9 m above its start without a photo (29.5, G6).

| What you see | Why, and what to do |
|---|---|
| `t2Cfg=bad` | a stored `cvt2k`/`cvt2r` out of range or unreadable, or `cvt2k` > 0 with `cvqlate` > 1: T2 off; fix the setting, reboot |
| `t2=frozen` in a hover | no accepted frame in 0.5 s: the drift stops by design (correction 2); note `camvz` `lastFrame`, `rejected` |
| `t2=hold` all the time | a guard event keeps coming (a motion, no tracked G, the CUSUM building up, the offset moving > 1 mg/s): the rate keeps its value; report |
| `t2bR` at +-0.350 for long | the rate at its bound: expected now and then (bRate swings, 29.5); for most of a hover, report |

### R. Set-height to the target, climbs on thrust headroom, the hold-limit warning, the trim check (2026-10-16)

docs/autonomy-a2b.md section 27 (and 27.9, the review round). The build of 27 on top of 8277b68 (with the colour and
T2 switches off it flies as 8277b68: a 4-byte SET_HEIGHT as before but that a climb is refused UP_HEADROOM where
8277b68 refused it BATT, the same commands at the 58000 ceiling; on the stabilizer task one byte compare every 10th
loop and, when the camera's hold-limit bits change, one call into flash code). The
Mac tools of this tree send the 6-byte target form; against 7a78e1b / 8277b68 they fall back to the 4-byte form with
X + h_now - target (`sh_form` "legacy-target"), so they work on the drone as flashed too. Flashing needs the user's OK.

**R0. Before flashing:** `python3 tools/espfly_diag.py flights > flights-before-<sha>.txt` (the T2 build's RTC magic
E9 erases the stored flight summaries at its first boot). **After flashing:** the P0 checks (`status` x5, `diagStackFree` >= 1024), L0 (`heapInt`: 24 B less than
8277b68's 18.4-18.6 KB with the Mac linked; under 12288 a hard stop), A4 (>= 940 loops/s; 971.6 on 8277b68).

**R1. DRY: the target form.** `sortie --mode dry --height 100 --photos 1 --wait-s 60`; once waiting (`state`: hMm
~0.9x, `hTarget_m` 1.00):
1. `set-height 1.10 --now`: OK; `true_now_m` 1.10, `target_m` 1.00, `fly_m` -0.10 (not -(1.10 - h)); after DONE
   `state` hMm ~1.00 and hTarget 1.00.
2. `set-height 0.90 --now --to 1.20`: OK, `fly_m` +0.30, `target_from` "--to"; after DONE hTarget 1.20, hMm ~1.20.
3. `set-height 0.75 --now --to 1.40` (d -0.45, 0.65 to go): `clipped_to_m` 1.25, `rest_m` 0.15, `fly_m` +0.50;
   after DONE hTarget 1.25. The daemon remembers the 1.40 (review 2026-10-16): after a new photo, a plain `set-height
   X` reads `target_from` "remembered", `target_m` 1.40 and flies the rest; a vertical `move`, an explicit `--to` or a
   set-height that is not clipped ends the memory. (`move --up 0.15` does the same and clears it.)
3b. The floor: after 30 s of air `set-height 0.90 --now --to 0.50` is refused locally with `why` HEIGHT and `floor_m`
   0.55 (a target-form descent keeps a MOVE down's floor, 0.25 m + 0.01 m/s of air up to 0.55; the drone refuses it
   too); `--to 0.60` flies.
3c. The A/B (only when asked for): `set-height 1.10 --now --legacy` sends the 4-byte form of 8277b68 (`sh_form`
   "legacy", `fly_m` -d, `flies_to_m` the old estimate): the drone ends ~0.1 m under the commanded height as before.
4. `photo`, wait 12 s, `set-height 1.00`: `warnings` "the photo is 12 s old".
5. `land`.

**R2. DRY: the climb's headroom.** Not reachable on the sim's thrust (T_hold ~36-47k); the host tests cover the
ceilings (test_auto_fsm `testSetHeightTo`). On a flight near the end of the battery, a refused climb reads `LOW_BATT`
with `why` UP_HEADROOM and `t_hold` over ~53400.

**R3. The trim check on the saved logs (no drone):** `python3 tools/espfly_diag.py trimcheck ../../firmware-backups/cal/flight-2026-10-16-r1-f1.csv --flights ../../firmware-backups/cal/flights-2026-10-16-r1-f1.txt`: F1 flagged (roll +1.11, pitch +1.12, ay +21.6 mg from normal), `atrim set -0.05 0.55` suggested as a RAM-only try; the 10-15 p1 flight 5 and the A/B flight: "nothing to change". After every flight: `dump` prints it per autonomy flight; `flights` adds a line when a whole-flight hold mean is over 0.8 deg. Nothing is ever set by it.

**R4. The hold-limit warning in flight** (a normal hover, FLIGHT; information only): with the hold at its limit for
8 s outside motions the daemon prints `drone_event` HOLD_LIMIT with `detailName` [{axis, sign, creeps}] and the hint.
On a well-trimmed drone it should not appear (none of the 10-15 flights would have had one). If it does: photo, watch
the drift, land soon, `trimcheck`, check the legs, props and battery seat.

**R5. Flights (each needs the user's OK).** A 1.0 m hover with p1 ~2.5 m ahead, a photo every 10-15 s, each
correction sent as the judged height within ~10 s of its photo: after each correction the next photo should read the
commanded height within ~0.1 m (the 4-byte form left ~0.1 m low). Report each correction's `true_now_m`, `fly_m`, the
next photo's judgement and the photo-to-command time.

### T. The faster flow fit: room for always-on colour (2026-10-16; flashing needs the user's OK)

(Not the setup section S at the top, nor phase Q's late-sink fix T2: this section's steps are T0-T5.)

docs/autonomy-a2b.md section 30. The flow fit gives 02cbbe0's results bit for bit (host test, and the firmware's own
code in an emulator) in ~40 % of its instructions; the camera task reports its stages (`flowfit prof`). `photocol`,
`flowfit`, the clock logic and its 62 ms fallback mark are as on 02cbbe0. **What does change is the timing** (a2b 30.7):
each flow sample is ready ~30-40 ms earlier in its frame, so the drift hold acts on it sooner and the camera height
observer folds it in 3-4 IMU blocks earlier (its outputs are no longer bit-identical to 02cbbe0's, though the values
of each sample are); and at `camclk 20` (45 ms frames: `tkoff camclk 20` or a fallback), where 02cbbe0 skipped every
other frame, every frame is now processed (pairs half as far apart). T5 is the flight check of that change. Order:
T-pre, T0, T1, T2, T3, T4, then T5 (a flight: the user's OK). **Any hard fail** (marked):
`python3 tools/espfly_diag.py auto set photocol 0`, `reboot`, report the numbers.

**T-pre. Before flashing, on 02cbbe0** (grey, the drone still on USB facing the lit, textured scene it will face in
T1, nothing moving): the reference for T1's flow check:
`python3 tools/espfly_diag.py stream --hz 50 --seconds 20 --quiet --out /tmp/t-pre-02cbbe0.csv`.

**T0. After flashing** (the drone on USB, props off, facing the same lit, textured scene as N'): the P0 checks
(`status` x5). **`fw=` reads the commit (the sha in the `.bin`'s name), not `fw=1`** (hard fail: a wrong image);
`diagStackFree` >= 1024; `camStackFree`, `flowStackFree` >= 1024 (expected ~4000 and ~2700-3500); `heapInt` with the
Mac linked within -24 B of 02cbbe0's 18.8 KB (hard stop under 12288).

**T1. Grey** (`photocol` 0, `flowfit plain`; the format boot of N, then the 60 s settle):

```
python3 tools/espfly_diag.py raw "flowfit prof reset"
sleep 60
python3 tools/espfly_diag.py status        # three times, 10 s apart
python3 tools/espfly_diag.py raw "flowfit prof"
```

| Check | Pass |
|---|---|
| `camProcAvgUs` (02cbbe0: 60.3 ms) | <= 45000 (expected ~20000-32000) |
| `camProcMaxUs` | <= 50000 |
| `camClkFallback`, `camSkips`, `camFps` | 0, 0, 14.6-14.9 |
| `flowfit prof` `total` avg | within ~2 ms of `camProcAvgUs`; `coarse` + `fine` most of it (their min ~14-17 ms: the model's CPU) |
| `flowfit prof` `snap`, `post` avg | <= 1000 each (grey, no photo) |

Note every `flowfit prof` number (avg / min / max per stage) for the report: min against avg shows how much of a stage
is core 1's preemption. **Information:** the same readings with `raw "flowfit robust"` and with `raw "flowfit lite"`
(each saved; `flowfit prof reset`, 60 s, `status`, `flowfit prof`), then `raw "flowfit plain"` back (the flown mode
stays plain until the user decides).

**The flow's numbers on hardware** (a cheap guard; the proof is the emulator's, a2b 30.3): with the drone exactly as
in T-pre, `python3 tools/espfly_diag.py stream --hz 50 --seconds 20 --quiet --out /tmp/t1-gray.csv`, then N'1.4's
one-liner on both files. Pass: N'1.4's marks (`flowLum` within 3 %, `flowTexture` within 5 %, `flowResidual_dg`
within 10 %, the `flowOk` rate within 2 points, the dx / dy noise at rest within 20 %; `frames` >= 285 in both): the
scene and the sensor's noise differ between the two boots, so these are tolerances, not bit equality. A larger
difference with the same scene: report both files (not a hard fail by itself, but no T5 flight until the user has seen them).

**T2. The loop rate** (A4 as written, grey): `activeLoopsPerS` >= 940 (02cbbe0: 966); the two `camvz prof` reads
>= 940; `stabStackFree` >= 1024. **Below 940: stop** (a2b 2.6).

**T3. Colour** (`auto set photocol 1`, `reboot`, the 60 s settle): N'0's checks (`camPix=yuyv-split`, `camSplit=ok`),
then T1's readings: `camProcAvgUs` <= 45000 and within +500 us of T1's; `camProcMaxUs` <= 50000; `camClkFallback` 0,
`camSkips` 0. Then three `python3 tools/espfly_auto.py photo --n 3 --gap 150` 10 s apart, `status` and
`raw "flowfit prof"`: `camSkips` unchanged, `camProcMaxUs` <= 50000, the replies `ok` with `color` true.

**T4. The case that failed on 02cbbe0: a DRY sortie in colour with `camvz air full` and photos** (the colour boot of
T3):

```
python3 tools/espfly_diag.py raw "camvz air full"
python3 tools/espfly_diag.py raw "flowfit prof reset"
python3 tools/espfly_diag.py auto prof reset
# gesture, then:
python3 tools/espfly_auto.py sortie --mode dry --height 100 --photos 1 --wait-s 120
#   the DRY hover stays until 'land' (L2). For 5 min, from a second terminal:
#   python3 tools/espfly_auto.py photo --n 1 every 15 s, and python3 tools/espfly_diag.py status every 60 s
python3 tools/espfly_auto.py land
python3 tools/espfly_auto.py wait --until idle
python3 tools/espfly_diag.py status
python3 tools/espfly_diag.py raw "flowfit prof"
python3 tools/espfly_diag.py auto prof
python3 tools/espfly_diag.py raw "camvz air off"
```

| Check | Pass |
|---|---|
| `camClkFallback` (every `status`) | **0** (02cbbe0: 1 in this test; hard fail) |
| `camSkips` (every `status`, ~5 min) | not rising: at most +1 over the sortie (02cbbe0: 1012; hard fail if it climbs) |
| `camProcAvgUs` / `camProcMaxUs` | <= 45000 / <= 50000 (02cbbe0: 61.2 / 65.3 ms) |
| `flowfit prof` `total` max | <= 50000 |
| `auto prof` `activeLoopsPerS` | >= 940 (the A4 gate, now with colour, photos and the observer) |
| `diagStackFree`, `flowStackFree`, `camStackFree`; `heapInt` | each >= 1024; within T0's |
| `flowfit prof` `coarse` / `fine` max | information (the Hessian reuse's restart, a pass redone when motion carries pixels past the margin, shows here: a DRY hover has little motion, so T5 reads it again) |
| the photo replies | all `ok`, `color` true |

Then N''s soak (5 min streaming with a photo every 30 s) on this build: `camRestarts`, `camBad`, `camTimeouts`,
`camSkips` 0. If T1-T4 pass, always-on colour has its frame budget; making `photocol` 1 the default stays the user's
decision (a2b 28, the plan's section 7 item 4).

**T5. The timing change in flight (needs the user's OK; grey, `flowfit plain`, the default 13.33 MHz).** The fit's
results are 02cbbe0's, but its samples reach the drift hold and the camera height observer ~30-40 ms sooner (a2b
30.7; a2b 19's unroll note asked for exactly this check). Before it: `python3 tools/espfly_diag.py flights >
flights-before-<sha>.txt` (as R0). Two plain hovers, 1.0 m, 60 s each, no moves, the same place and light as the
2026-10-16 R flights, `raw "flowfit prof reset"` before each and `raw "flowfit prof"`, `status` after. Compare with
those flights' logs (`firmware-backups/cal/flight-2026-10-16-r1-f*.csv`, `trimcheck` as R3): the drift in the hover
(the photos' judged position and the hold's output), the height track (`set-height` judgements against the observer's
height) and any oscillation of the hold. Pass: no worse than 02cbbe0's flights (the hold's latency only shrank);
`flowfit prof` `total` max <= 50000 in flight (the restarts are likelier with real motion); `camSkips` 0,
`camClkFallback` 0. **An oscillation or a drift clearly worse than 02cbbe0's: land, report;** the flown image goes
back to 02cbbe0 until the user decides (it is in firmware-backups).

| What you see | Why, and what to do |
|---|---|
| `fw=1` | not this build's `.bin` (an older image; a copy built without a version now stops at the configure): check its sha256 |
| `flowfit prof frames=0` | no frame with a fit yet (the camera asleep on the ground: `tkoff camwake 300`), or just reset |
| `camProcAvgUs` near 60 ms | the old image (check `fw=`); else report `flowfit prof` (which stage, its min against its avg) |
| a stage's min far below its avg | core 1's preemption (the 1 kHz tasks, interrupts): information; the frame budget counts wall time |

### U. Fast moves: the mvspeed speed step (2026-10-16; flashing and flights need the user's OK)

docs/autonomy-a2b.md 31. The user flashes; agents never do. Every USB command below runs with the drone in IDLE.
**The Mac tool must come from the same commit** (AUTO_STATUS is 140 bytes: an older Mac refuses motions with a
protocol error). Order: U0 the version and today's motions at 100, U1 DRY at 200 and 300, U2 the bench, U3 a
hand-carried camera test, then the flights stepping up. **Flashing, the bench and every flight need the user's OK.**

(Revised after the review of cbccec5, a2b 31.8: the caps V_H 0.40, W_FWD 0.17, yaw 35 deg/s / 70 deg/s^2, the bob as at
100, VIEW_FAST after 150 ms of lost view; U0's identity mark, the fw check, the camera, stack and heap marks.)

**U0. Flash, version and today's motions at 100.**
- `python3 tools/espfly_diag.py status`: `fw=<the bin's short sha>` (the Mac tool from the same commit: `git -C
  Firmware/esp-drone log -1 --format=%h`). Fail: any other fw: stop, nothing flies.
- `python3 tools/espfly_diag.py auto get`: `mvspeed=100(default)`; `auto status`: `mvSpeed=100`.
- A DRY sortie with the Mac at D_sim 2: `move --forward 1.0`, `move --right 0.6`, `yaw --deg 90`, then a STOP partway
  through a `move --forward 1.0`. Replies at the end (no `settling`). From the recorder (mvState, mvVRef_mms,
  mvRefFwd_e4, mvRefLat_e4, mvYawOff_cdeg) per command: RUN 8.17 s with mvVRef peak 150 mm/s (1 m forward), 4.90 s
  peak 240 mm/s (0.6 m right), turn 3.50 s + the 0.5 s dwell, each within 0.1 s and 1 % of the Mac's `profile_s`; the
  end references equal the commands. Bit-identity at 100 is the host hash's (test_auto_motion testSpeedIdentity:
  0x47E10B48658F1BE0 for 7ab74b9 and this build); a real-time DRY sortie is never bit-identical, so it is not asked
  here. Fail: a time or peak off by more than that: stay on 7ab74b9.

**U1. DRY at 200, then 300.**
- `auto set mvspeed 200`, a new ARM; `state` shows `mvspeed_pct 200`.
- At D_sim 2: 1 m forward 6.40 s (the peak rule, W_FWD 0.17), 0.6 m sideways 3.54 s (W_LAT x 2 = 0.28 m/s), 90 deg
  3.07 s + 0.3 s dwell (DRY: no bob). The Mac's `profile_s` equals the drone's RUN time within 0.1 s. At 300: 6.18 s,
  3.08 s, 3.07 s.
- `move --forward 0.5 --photo` replies at SETTLE with the photo (`settling: true`); a `move` sent at once waits and
  goes after DONE (`motion_waits`, `motion_released` events); a STOP withdraws a waiting one.
- The battery-fit message names 1.5 s of settle (1.0 s at 300).
- VIEW_FAST, the stale-hold brake lean and the 9-12 deg/s apply slew cannot be shown here: DRY and BENCH treat the view
  as always good (auto_fsm.c: `flight` only in FLIGHT), and the hold needs real frames with a reference. The host tests
  carry them (test_auto_motion testViewFast, test_controller_pid_ihold, test_interactive_sim testFastMoves D/E/H/I);
  F1 is their first real run, read from its dump (below).

**U2. BENCH sim plant at 300, props off, colour on (photocol 1).**
- Loop rate >= 940 loops/s during moves and turns (A4 style), and again in a plain BENCH hover with the camera hold
  stale (controller_pid calls autonomyBrakeLean on every loop without a fresh hold: it returns at once at 100 and
  outside a fast MOVE's brake). No loop gaps.
- Heap with the Mac linked: the build's internal heap is 288 B smaller than 7ab74b9's (a2b 31.7: 7ab74b9 read 18.9 KB,
  82bcefb read 18.41-18.63 KB), so ~18.6 KB is expected and a single reading within the old noise is not a fail. Pass:
  >= 18.2 KB, and heapMin after the whole of U1-U2 >= 14 KB (the guard is 12288 B). Fail: below either: stop and report.
- `diagStackFree >= 1024` after `auto get`, `auto set mvspeed` and `auto status` (a high-water mark: read it last);
  stabStackFree >= 2800, the camera task >= 5230 and the flow task >= 3330 (T0 / A4 read 2896 / 5332 / 3432; the
  static depths moved by 32 B or less).
- The camera's budget at 300 with colour: camProcAvgUs <= 23 ms, the max <= 28 ms, camClkFallback 0, camSkips <= 1
  over a minute of moves (bench T's T1 / T3 marks).

**U3. Hand-carried camera test** (`camvz air` bench mode, motors off), held at 1 m over the usual scenes:
(a) toward a wall from 3.5 m to 1.0 m at a steady 0.3-0.4 m/s; (b) sideways at 0.3 m/s at D 2.5; (c) a turn by hand
at 30-35 deg/s. Pass: flowOk >= 98 %; expEma <= 0.22 /s while v / x <= 0.17; |dxRes| p95 <= 1.2 px at 0.14 /s; the
per-frame yaw rate under 0.75 rad/s; the observer accepts >= 95 % of the frames; no cvH jump over 0.05 m. A fail
lowers that axis's cap before any flight (W_FWD 0.15, W_LAT 0.13, yaw 32 deg/s).

**Flights.** Open room, >= 3 m clear ahead and to the sides of every move, nobody within 2 m of any path, rest
voltage >= 3.9 V, the user ready with 'k', the battery monitor as usual. Dump the recorder after every flight.
- F1, 150: take-off, `move --forward 1.0 --photo` (D >= 3), `--back 1.0`, `--right 0.6`, `--left 0.6`, `yaw --deg 90
  --photo`, `yaw --deg -90 --photo`, land. From the dump: the hold's output slew stays <= 9 deg/s, the lean <= 3.5 deg,
  the per-frame yaw rate (flowGz) under 45 deg/s in both turns.
- F2, 150 again: a 2-2.5 m approach toward the couch or a wall (D 4-5) ending at the Mac's guard. From the dump: the
  hold's lead (the measured expansion over the reference's, mvRefFwd's rate) and expEma <= 0.25 /s; the closest
  distance against a2b 31.4 before 250-300 are flown.
- F3, 200: F1 + F2. F4, 250, then F5, 300: the same. Two clean sorties at a step before going up.
- Missions afterwards at the highest step that passed: the tiger approach; a person approach at <= 200 % first with
  `--person-gap 1.5` for the first two sorties, then the 1.0 m rule; following the child only after two clean
  person-approach sorties.

**Abort and step down** (any one ends the step: land, 'k' if anything is wrong in the air, `auto set mvspeed
<previous step>` or `auto reset mvspeed`, report the dump): tilt over 6 deg; a hold axis within 0.2 deg of its limit
for over 0.3 s, or HOLD_LIMIT; a VIEW_FAST, NO_VIEW or PAUSED in a clean move with nothing crossing the view (a person
walking through the view or a refused-frame burst in the dump is not the step's fault: note it, fly on); PAUSED rows in
a turn more than the 100 % flights had for the same turn (the logged 30 deg/s turns already had some: 39 rows in the
10-14 gym 180, 7 in the 10-14 p2 45); a PROGRESS, FROZEN or FRONT stop on a clean view; CAM_LOST, a SINK or a
camera-height event within 5 s of a motion; a contact; an overshoot past the target over 0.15 m (0.10 m on a person
approach); a move more than 20 % off on a clean view; a loop gap, a brownout, or a battery-fit or timeout refusal that
disagrees with the drone.

Report after each flight, per motion: the RUN / SETTLE times, the end, the peak |hold|, the tilt, flowOk and expEma
against the table of a2b 31.3; a time off by more than 0.3 s is looked at before the next step.

### V. Bolder ahead margins and the looser off-course guard (2026-10-17; flashing and flights need the user's OK)

docs/autonomy-a2b.md 32. The user flashes; agents never do. Every USB command below runs with the drone in IDLE. **The
Mac tool must come from the same commit** (its floor follows the drone's cached `mvfloor`).

**V0. Flash and the settings.**
- `python3 tools/espfly_diag.py status`: `fw=<the bin's short sha>`, `diagStackFree >= 1024` after the steps below.
- `python3 tools/espfly_diag.py auto get`: ends `mvspeed=... mvfloor=30(default) mvfstop=25(default) mvprogm=20(default)
  mvprogr=40(default) mvlatm=40(default) mvnear=60(default)` (saved for the Mac). `auto set mvfloor 19` and `auto set
  mvnear 151`: ERR with the range. `auto set mvfloor 45`, then `auto get` shows `mvfloor=45(nvs)`; `auto reset mvfloor`.

**V1. DRY at D_sim 2.0 (mvspeed 100).**
- `move --forward 1.40`: refused by the Mac ("the Mac keeps 0.35 m: at most 1.29 m now"); `move --forward 1.29`: DONE
  (8-9 s). The drone's own boundary is 1.35 m (0.30 m at k 0.6): from a Mac-less test only, not needed here.
- `auto set mvfloor 45` (IDLE), a new ARM: `move --forward 1.25` refused by the Mac (0.50 m: at most 1.12 m); `auto reset
  mvfloor` after.
- DRY cannot show the guards (the view is always good there); the host tests carry them (a2b 32.7).

**V2. Bench (BENCH sim plant, props off, colour on).** A4 as in U2: loop rate >= 940 loops/s during moves, the heap
with the Mac linked >= 18.2 KB and heapMin >= 14 KB (this build adds 8 B of .data and 24 B of settings), stacks as U2
(camStackFree read and reported: 4064 B on the 4dea542 bench; this build does not touch the camera task).

**Flights** (open room, the user ready with 'k', rest voltage >= 3.9 V, mvspeed 100 first; dump after every flight).
- G1: toward a soft target (the couch, a cushion on a chair) from D 2.5-3 m: the Mac's longest forward move, then a
  photo. Expected: DONE at about D - F from it (k ~1 for a frontal target: 0.75-0.85 m from D 2.5-3 with F 1.73-2.17 m;
  the floor's 0.35 m is the end at k 0.6). From the dump: no FRONT, no PAUSED near the end, expEma under 0.25 /s; the
  end distance on the photo or a tape mark (it measures k).
- G2: the same toward a scene with a lot of floor in view (k ~0.6-0.7): the end 0.35-0.5 m from it. A FRONT stop there
  is the guard working (the drone holds): note the distance at the stop.
- G3: two short steps (0.5-0.9 m) toward a scene with a bright window behind it (the 10-17 table flight): both end DONE
  or SHORT, not PROGRESS (the sideways margin 0.40 m); the reply's sideways travel noted.
- Then the person approach with the 0.5 m rule (`--person-gap`, `--est`) at 100, a person sitting still.

**Abort** (land, 'k' if anything is wrong; `auto set mvfloor 50`, `auto set mvfstop 40` and `auto set mvlatm 25` put the
margins of before back without a flash; `auto set mvnear 0` removes the near-scene brake): a contact on an approach; an
end closer than 0.20 m to the scene by the photo or the tape; a FRONT stop whose drone keeps closing on the scene after
it; a sideways excursion over 0.5 m on a straight move; the height over 0.15 m off at the end of an approach (a2b 32.6:
correct it with set-height and report).

### W. The move fixes of the 300 % forensics (2026-10-17; flashing and flights need the user's OK)

docs/autonomy-a2b.md 33. The user flashes; agents never do. Every USB command below runs with the drone in IDLE. **The
Mac tool must come from the same commit** (the LONG end, the replies' fields).

**W0. Flash and the settings.** (With the review's build, X0 replaces this.)
- `python3 tools/espfly_diag.py auto get`: ends `... mvnear=60(default) mvfix=27(default)`. `auto set mvfix 32`: ERR with
  the range; `auto set mvfix 0`, `auto get` shows `mvfix=0(nvs)`; `auto reset mvfix`.
- Then (last: diagStackFree is a low-water mark, and `auto get` has the larger frame) `python3 tools/espfly_diag.py
  status`: `fw=<the bin's short sha>`, `diagStackFree >= 1024`.

**W1. DRY at D_sim 2.0.**
- mvspeed 100: `move --forward 1.0`, `move --back 1.0`: DONE, the replies as before (`travelled_m`); the second reply has
  no `drift_since_last_end_m` in DRY (no camera) or 0.
- mvspeed 300 (`auto set mvspeed 300`, a new ARM): `move --forward 1.0` replies early with `travelled_pre_settle_m` (no
  `travelled_m`), its profile 6.34 s as at 200 (6.18 s before: A_H 0.20 now); `move --right 0.3 --full-wait` waits the
  first one's settle (at most 2.5 s now) and its reply has `previous: {end: DONE, travelled_m: ...}`. `auto reset
  mvspeed`.

**W2. Bench (BENCH sim plant, props off, colour on), as U2 / V2.** The loop rate >= 940 loops/s during moves, the heap
with the Mac >= 18.2 KB, heapMin >= 14 KB (this build: 64 B less internal heap than 4dea542). Stacks: read `status` right
after the boot (grey or colour, no photo yet): `camStackFree` (the esp32-camera driver's cam_task, 6144 B) and
`flowStackFree` (the flow and hold task); then take a colour photo set and check the boot log (`crash`) for `cam_hal:
FB-SIZE`; read them again. Expected: flowStackFree ~3.4 KB both times; camStackFree ~5.3 KB before and ~4.0 KB once an
FB-SIZE line was logged (a2b 33.10: the driver's log path, not this build). Report both numbers either way. Floors
(the review; U2's camera >= 5230 B was a fresh-boot number and does not hold once an FB-SIZE line ran): **camStackFree
>= 3500, flowStackFree >= 3330, diagStackFree >= 1024**; below any of them: do not fly this build, report. DRIFT, LONG
and STILL only run in FLIGHT on the camera (`in->flight && !in->simPlant`): neither DRY nor this bench exercises them,
so W-F1 is their first run on the drone (the host tests and the 300 % replay carry them until then); GAIN and BRAKE do
run here.

**Flights** (open room, the user ready with 'k', rest voltage >= 3.9 V; dump after every flight). The fixes change
mvspeed 100 too (DRIFT, LONG, the back move's margin, BRAKE, GAIN), so W-F1 is at 100.
- W-F1 (100 %): forward 1.0 m toward a textured scene at 2-3 m, a photo, hover 5-10 s, back 1.0 m (flight 6's pair).
  From the replies: the back move's `drift_since_last_end_m` (the drift the drone now plans with) and its end (DONE,
  SHORT or LONG with `travelled_m`); `wait-move` after each. Against tape marks or a phone video (the verdict's item 12:
  the camera's scale): the true travel of both moves.
- W-F2 (150 %, only after W-F1 ended within 0.20 m of the marks or reported its LONG / SHORT): the same pair. The early
  reply's `travelled_pre_settle_m` against the `previous` final end in the next reply.
- From the dumps: the hold's pitch while braking (the brake now scales with the measured speed: no slide back at the end
  of a slow move), the gain scale through the moves (holdk show; no fall to 1.0 while the drone lags).

**Abort** (land, 'k' if anything is wrong; `auto set mvfix 0` puts the motions of before back without a flash, `auto set
mvfix 57` only LONG off; 25 on the build before the review): a move's true end more than 0.30 m from the marks with a DONE reply (the fixes should report it
LONG / SHORT); a slide back after a brake larger than 0.15 m; a sideways excursion over 0.5 m on a straight move; any
contact.

### X. The review of the margins and the move fixes (2026-10-17; flashing and flights need the user's OK)

docs/autonomy-a2b.md 34. The user flashes; agents never do. Every USB command below runs with the drone in IDLE. **The
Mac tool must come from the same commit.** This build replaces W's: W1, W2 (with its floors) and W-F1 / W-F2 stand, with
X0 for W0 and the checks below added.

**X0. Flash and the settings.**
- `python3 tools/espfly_diag.py auto get`: ends `... mvfstop=25(default) mvprogm=20(default) mvprogr=40(default)
  mvlatm=40(default) mvnear=80(default) mvfix=59(default)`. `auto set mvfix 64`: ERR with the range (0..63); `auto set`
  with no key lists `mvfix` among the keys; `auto set mvfix 27`, `auto get` shows `mvfix=27(nvs)`; `auto reset mvfix`.
- Then `python3 tools/espfly_diag.py status` last: `fw=<the bin's short sha>`, `diagStackFree >= 1024`.

**X1. DRY at D_sim 0.40 (mvspeed 100).** `move --right 0.19`: DONE as before (DRY shows no guard; the host tests carry
the front stop on sideways moves, a2b 34.7). A `set-height --now` sent while an early-returned move settles (mvspeed 200)
has `previous` in its reply, and the next MOVE has none.

**Flights** (after W-F1 at 100; open room, the user ready with 'k', rest voltage >= 3.9 V; dump after every flight).
- X-F1 (100 %): from a forward move's end near the floor (0.35-0.5 m from a soft target), `move --right` 0.15 m and
  back. Expected: DONE, or FRONT if the drone is pushed toward the target (the drone holds, Claude backs off); never a
  contact. A view loss there now ends the move FRONT (mvnear 0.80 m, any horizontal move) instead of pausing.
- **Before any person approach** (the 0.5 m rule: a2b 34.5): a 20-30 s hover trim check first (the dump's trimcheck, or
  `atrim`) with the drone's drift under ~0.05 m/s; the drone's guards measure the wall behind the person, not the
  person, and an unlearned 0.5-1.2 deg lean overshoots a planned end by 0.3-0.7 m (sim). Approach in two steps, the last
  one <= 0.5 m, the person seated, the user's hand on 'k'.

**Abort** as W, and: a contact on a sideways or back move near the scene; a FRONT stop whose drone keeps closing; a
LONG on an approach whose drone does not come back toward its target within ~3 s (`wait-move` and a photo). Without a
flash: `auto set mvfix 27` (no FRONT), `auto set mvnear 60` (the near brake of before), `auto set mvfix 0` and the V
values (the motions before 2026-10-17).

### Y. Real 300 %: mvacc and mvsettle (2026-10-17, evening; flashing and flights need the user's OK)

docs/autonomy-a2b.md 35. The user flashes; agents never do. Every USB command below runs with the drone in IDLE. **The
Mac tool must come from the same commit.** This build replaces X's; X0's checks stand with the two new keys.

**Y0. Flash and the settings.**
- `python3 tools/espfly_diag.py auto get`: ends `... mvnear=80(default) mvfix=59(default) mvacc=20(default)
  mvsettle=2500(default)`. `auto set mvacc 41` and `auto set mvsettle 999`: ERR with the ranges (10..40 cm/s^2,
  1000..3000 ms); `auto set` with no key lists `mvacc mvsettle`. `auto status` ends `mvSpeed=100 mvAcc=20 mvSettle=2500`.
- Then `python3 tools/espfly_diag.py status` last: `fw=<the bin's short sha>`, `diagStackFree >= 1024` (`auto get` and
  `auto status` have the larger frames: read `status` after them).

**Y1. Real 300 %** (the profile and settle of the morning's 300 % flights, with d3bedee's move fixes and margins):
```
python3 tools/espfly_diag.py auto set mvspeed 300
python3 tools/espfly_diag.py auto set mvacc 30
python3 tools/espfly_diag.py auto set mvsettle 1000
python3 tools/espfly_diag.py auto get        # mvspeed=300(nvs) ... mvacc=30(nvs) mvsettle=1000(nvs); the Mac's cache
```
`auto status`: `mvSpeed=300 mvAcc=30 mvSettle=1000`. DRY first (D_sim 2.0): `move --forward 1.0`: the early reply's note
"settles at most 1 s", `state` shows `mvacc_cm_s2: 30, mvsettle_ms: 1000, fast_caps_src: "auto get"`, `profile_s` 6.2
(6.3 at the defaults). Back to d3bedee's caps: `auto reset mvacc`, `auto reset mvsettle`; to 100 %: `auto reset mvspeed`.

**Flights** (after W-F1 / X-F1 at 100, open room, the user ready with 'k', rest voltage >= 3.9 V, dump after every
flight): Y-F1 at real 300: flight 6's moves again (1.0 m forward, then 1.0 m back, against tape marks), each with
`--full-wait` or a `wait-move` before judging the end. Expected: DONE, SHORT or LONG with the end reported (not a silent
0.2-0.45 m overshoot); 0.1-0.4 m of slide after an early reply.

**Abort** as W and X, and: a lean over ~3 deg in a move, a contact, a back move ending more than 0.3 m past its target.
Without a flash: `auto reset mvacc` and `auto reset mvsettle` (d3bedee's 0.20 m/s^2 and 2.5 s), or `auto set mvspeed 100`.

### Z. Accelerometer tempco and the ground cooling spin (2026-10-18; flashing and flights need the user's OK)

docs/autonomy-a2b.md 36. The user flashes; agents never do. Every USB command below runs with the drone in IDLE,
props clear for Z2. **The Mac tool must come from the same commit.** This build replaces Y's; Y0's checks stand.

**Z0. Flash and the settings.** The RTC flight store is cleared by this flash (its layout changed): `flights` first
if the old ones matter. Right after flashing, the NVS still holds mvacc 30 / mvsettle 1000 from the real-300 test,
which this build reads (d3bedee ignored them): `python3 tools/espfly_diag.py auto set mvacc 20` and `auto set
mvsettle 2500` (or `auto reset mvacc`, `auto reset mvsettle`) unless real 300 % is wanted. Then:
```
python3 tools/espfly_diag.py atc            # A seq=0, OK atc on=0 k=+0.000,+0.000,+0.200 knee=60.0 clamp=15.0 src=default refSet=1
python3 tools/espfly_diag.py raw "atc set 0 0 0.3"   # OK atc set on=0 k=...+0.300 stored=unchanged; then raw "atc clear"
python3 tools/espfly_diag.py raw "atc set 0 0 3"     # ERR atc set: |k| <= 2.0 mg/C
python3 tools/espfly_diag.py cool           # OK cool state=none pwm=0 s=0 ... runs=0
python3 tools/espfly_diag.py status         # ... atc=0 atcZ=+0.00 imuTempC=... fw=<sha> diagStackFree >= 1024
python3 tools/espfly_diag.py rec info       # cap=18000 (126 B records), psramFree >= 1 MB
```
`refValid=1` once the drone has rested 1 s after boot; `still=` near `ref=` (x / y within a few mg, z ~1000).

**Z1. The check without flying** (bench): `raw "atc latch"`, wait 5 s still, `atc`: `latchAgeMs` ~5000, `dTeff`
~0, `still` - `ref` within +-1 mg on every axis (the window's noise). Tilt the drone by hand 10 deg and back: no
entry (no thrust), `still` returns to `ref` within 1 mg.

**Z2. The cooling spin** (props on, clear, the drone on the floor, a hand on `k` in the daemon tab or `cool stop`):
```
python3 tools/espfly_diag.py cool 20        # OK cool start state=run pwm=12000 s=20 ...; a state line every 2 s;
                                            # then: IMU 49.x -> 4x.x C (-y.y) in 20.0 s, end DONE; still accelerometer
                                            # step x/y/z ... over dTeff ... -> k_z ... mg/C
```
Expect all four props spinning evenly and the drone not moving; `gyroMax` below 30 and `tiltMax` below 8 on the
state lines; a cooling of 1-4 C in 20 s (unknown: this is the first measurement). After it, `gtc`: a `Z` line with
`flight=1 air=0 cool=1` (the gyro's step over the cooling, r within +-0.15 dps); `flights`: the spin's row with
`COOL` in the motor-drive column and 0.0 s of air. Refusals to see once each: `raw "cool 20"` while tilted 10 deg
by hand (ERR cool: not resting on the ground); `raw "cool 50"` (ERR usage); `raw "cool 20 25000"` (ERR usage). An
ARM within 20 s after the spin is refused STILL (the settle); autonomy's heat-soak wait does not count the spin.
Repeat `cool 20` twice more from a hot IMU (right off the charger, 47-53 C) and note the three k_z values: their
sign and spread are the first real estimate of the coefficient. If they agree within a factor of 2 and |k_z| >=
0.1 mg/C: `raw "atc set 0 0 <mean> 60"`, `raw "atc on"`, `raw "atc save"`; else leave it off.

**Flights** (open room, the user ready with `k`, rest voltage >= 3.9 V, dump after every flight): Z-F1 a plain 60 s
hover at 1.0 m with `atc` off, taking off from the floor it lands on, no cool: afterwards `atc` prints the
post-flight check (step z, r_z, PASS / FAIL, TILT, `k_z measured`), `flights` the `acc z step/r mg` column. Z-F2
the same after `cool 20` on a hot IMU: the observer's early climb (cvH against vzH at 20-40 s in the dump) should
be smaller than the 15:26 flight's 1 m. `atc on` only after Z2's three spins and Z-F1 agree on the sign.

**Abort**: a spin that does not stop at its time (`cool stop`, `k`, the battery lead); any flight that climbs or
sinks visibly more than before with `atc on` (`raw "atc off"`, land). Without a flash: `atc off` + `atc save`; the
cooling spin is never automatic.

### AA. The healthy offset speed-up hb (2026-10-18; flashing and flights need the user's OK)

docs/autonomy-a2b.md 37 (the height estimator study: the early climb). NVS `cvhb` (u8 = the multiplier x 10: 0 = off,
the default and the filter of before bit for bit; 10..80; 1..9 refused), read at boot like `cvqlate`. The user
flashes; agents never do. This build replaces Z's; Z0's checks stand (the `mvacc 20` / `mvsettle 2500` musts included).

**AA0. The settings.** `python3 tools/espfly_diag.py auto get` ends `... mvsettle=2500(default) cvhb=0(default)`.
`auto set cvhb 5`, `auto set cvhb 81`: `ERR auto set: cvhb must be 0 (off) or 10..80 x0.1` (`0..80` until the guards-off
build of 2026-10-18, AB); `auto set` with no key lists `cvhb` (then `mvklow mvlat mvwmax`, AB). `camvz` (IDLE): `... hb=0.0 hbCfg=ok hbOn=0 hbOnS=0.0`; the tool prints "healthy offset speed-up x0.0 (off)".
```
python3 tools/espfly_diag.py auto set cvhb 30     # x3, the study's first step
python3 tools/espfly_diag.py reboot               # read at boot; ~3 s, then auto unlock
python3 tools/espfly_diag.py camvz                # hb=3.0 hbCfg=ok hbOn=0 hbOnS=0.0 (on the ground)
```
With `cvqlate` > 1 or `cvt2k` > 0 stored as well: the boot log "cvhb 30 with cvqlate ... refused", `camvz`
`hb=0.0 hbCfg=bad` (R1 / T2 as stored). Back to off: `auto set cvhb 0` (or `auto reset cvhb`), reboot, `camvz`
`hb=0.0 hbCfg=ok`.

**Flights** (open room, the user ready with `k`, rest voltage >= 3.9 V, dump after every flight; the study's flight
check): AA-F1 a plain 60-90 s hover at 1.0 m with `cvhb 30`, a photo every 10 s with the ceiling-line look (the
ceiling-wall junction on or below the level line = at the ceiling: come down), no upward set-height from a photo
older than 5 s in the first 40 s. Pass: the first-30 s climb under 0.3 m (today's flights: 0.4-1.1 m), the late sink
under 1 cm/s; `camvz` after it `hbOnS` close to the air time, `flights` `cvqf` with 0x20. AA-F2 the same from a hot
IMU (off the charger) and AA-F3 after `cool 20`: compare the early climbs. After two or three clean flights the
second step is `cvhb 50`.

**Abort** as Z, and: a climb the photos show past 0.5 m above the target in the first 40 s (land); a sink after a
camera dropout deeper than before (the expected cost: 0.75 m over a 5 s gap on the worst logged flight, 0.44 today;
the CAM_LOST rule and the SINK stand). Without a flash: `auto set cvhb 0`, reboot.

### AB. The MOVE guards down to off (2026-10-18 ~13:30; flashing and flights need the user's OK)

docs/autonomy-a2b.md 38 (the table approach stopped short by the front logic; user decision "remove the guards or
make them absolutely minimum"). Seven NVS settings, all read at the next ARM, no reboot: `mvfloor` / `mvfstop` /
`mvprogm` / `mvprogr` take **0 = off** (their ranges otherwise), `mvklow` (k x 100, 60..100), `mvlat` (the sideways
cap x 100, 0..200, 0 off), `mvwmax` (the expansion rate cap x 100, 10..40). Defaults: the flights of before bit for
bit. The user flashes; agents never do. Z0's and AA0's checks stand (`mvacc 20` / `mvsettle 2500` already in NVS,
`cvhb 30` and `atc off` as the user left them).

**AB0. The settings.** `python3 tools/espfly_diag.py auto get` ends `... cvhb=30(nvs) mvklow=60(default) mvlat=50(default)
mvwmax=15(default)`. `auto set mvfloor 10`: `ERR auto set: mvfloor must be 0 (off) or 20..100 cm` (the same for
`mvfstop 5`, `mvprogm 5`, `mvprogr 5`); `auto set mvklow 59`, `auto set mvlat 201`, `auto set mvwmax 41`: ERR with
the ranges. `auto status` ends `mvGuards=30/35/25/20/40/40/80 klow=60 lat=50 wfwd=15` (the margins in force: the
last ARM's, the defaults on a fresh boot).
```
python3 tools/espfly_diag.py auto set mvfstop 0      # the likely culprit of the noon stops: first
python3 tools/espfly_diag.py auto set mvfloor 0      # then, if needed: no planning floor (the Mac follows)
python3 tools/espfly_diag.py auto set mvprogm 0      # the progress guard off
python3 tools/espfly_diag.py auto set mvlat 0        # no sideways cap
python3 tools/espfly_diag.py auto set mvklow 100     # no expansion safety factor
python3 tools/espfly_diag.py auto set mvwmax 30      # approaches near the scene twice as fast
python3 tools/espfly_diag.py auto get                # saved for the Mac: espfly_auto.py 'state' shows guards.off
```
`auto set` is refused while autonomy is active; an ARM after it latches the values (`auto status` `mvGuards=0/0/0/0/...`).

**AB1. The Mac.** With `mvfloor 0` cached, `move --forward 1.9` at D 2.0 goes to the drone (before: "at most 1.29 m
now"); the drone's own `TOO_CLOSE` (D - F under 0.05 m) comes back with its hint. With `mvlat 0`, `move --right 1.4` at
D 2.0 goes to the drone. `state` -> `guards: {mac_floor_m: 0.0, k_low: 1.0, lat_rel: 0.0, mvfstop: 0, ..., off:
["floor", "lat", "mvfstop", "mvprogm", "mvprogr"]}`. The person rule (`--person-gap`, 0.5 m, k 0.6) refuses as before.

**Flights** (the user's table approach; the user ready with `k`, the drone's rest voltage >= 3.9 V, dump after every
flight): AB-F1 the approach of 2026-10-18 noon again with `mvfstop 0` alone: the MOVE should reach its target (the
reply's `along` ~1.0; before: 0.43-0.47 of it, STOPPED). If it still stops early (PROGRESS): `mvprogm 0`; if the Mac
or the drone refuses it TOO_CLOSE: `mvfloor 0` (or `mvklow 100` first). AB-F2 with `mvwmax 30`: the same approach in
about half the time, the view kept to the end (`hold` events: no NO_VIEW / FRONT at the end). Watch: the drone meets
the table edge at the profile's speed if the photo's distance was wrong; the contact kill and the hold decide.

**Abort**: `k`; land. Without a flash: `auto reset mvfstop` (and the others) and a new ARM put every guard back.

## After the session

- `auto set autowait 5000` (also after C2's 10000; phase D sets its wait per sortie and leaves `autowait`);
  `raw "camvz air off"`; `auto bench sim`; `auto get` `cvqlate=1` and `cvt2k=0` unless the user decided otherwise
  (O5, Q).
- `q` the `--allow-flight` daemon; start a plain one only when needed.
- `gcap` stays off (manual flights and FLIGHT need it off); set `gcap 15000` again only for a later BENCH
  session. `autoon` stays 1.
- Save `flights` and `crash` to `../../firmware-backups/cal/flights-<date>-post-a2b.txt` and
  `crash-<date>-post-a2b.txt`.

---

## When something goes wrong

| What you see | Why, and what to do |
|---|---|
| `arm` refused `GUARDS` | the reply names the guards (section 0). `LOCKS`: a reset since the last battery power-on or `auto unlock` (BENCH, FLIGHT), the motor driver latched by a KILL with motors on, the watchdog tripped or off, the emergency stop, a console change in progress, or for FLIGHT/BENCH-imu `camvz off` or a `camvz air` mode. `LINK`: no daemon heartbeat (`state` `daemon.rx_age_ms`; restart the daemon). `HEAP`: stop and report `auto status` `heap` (a stalled `takeoff_aux` task also gives HEAP: its heap sample is then older than 200 ms). `BATT`: charge or swap the battery. `STILL`: wait (FLIGHT 20 s, after a flight up to 90 s: the heat soak, the reply's hint) and level the drone. `CAMERA`: `status` `cam=`, `hold`; since 2026-10-14 the camera sleeps on the ground and the ARM wakes it: the daemon sends the ARM again for 3 s (event `arm_cam_wake`); still refused: a `camlead` longer than that (take a `photo`, wait the lead), or the camera did not wake (`status` `camGnd`, `tkoff` `gndFails`). |
| `arm` refused `KILL_LATCHED` or `BAD_STATE` | a KILL in the last 2 s; or the drone not yet back in IDLE (KILLED lasts at least 2 s) |
| `arm` refused `REFUSED` | the Mac tool's own gates: FLIGHT without `--allow-flight`/`--confirm-flight`, a rebooted drone (restart the daemon), a lost session |
| `air_ms must be 0 ... or 2000..6000` | `--air-ms` may only lower the drone's `autoair` (as the last `auto get` read it; 6000 without one, 4500 before 2026-10-12 evening); the drone refuses a higher value `RANGE` |
| `--height: must be 20..120` | the take-off height range (20..60 before the interactive hover, 20..40 before 2026-10-12 evening); a drone on older firmware refuses the rest `RANGE` |
| `move`/`yaw` `REFUSED` "not in the hover wait" / "is running" | phase D: only after the take-off photo, one motion at a time (`stop` ends one) |
| `move` refused `NO_DIST` or "no scene distance D yet" | the camera has no distance to the scene ahead (after a turn it bobs to get one): `state` again in a few seconds, or face a textured scene |
| `move` "the Mac keeps 0.60 m" | a forward move would leave less than 0.60 m to the scene ahead; use the distance the reply names |
| LAND `MOVE_NO_CAM`, `MOVE_TILT`, `BATT_LOW` | phase D: the camera lost the view for 2 s in a move, a lean over 8 deg in a move, the end of the battery (since 2026-10-14 below 3.00 V or a hover thrust over 57000 for 0.5 s; 3.30 V for 1 s before): the drone landed by itself |
| `move`/`yaw` refused `LOW_BATT` (BATT) | the hover needs over 53400 of the 58000 ceiling, or the drone's BATT_LOW condition has held 0.25 s: the end of the battery is near and the drone will land `BATT_LOW` by itself; hover, `photo`, `stop` and `land` still work |
| `move --up` / `set-height` refused `LOW_BATT`, why `UP_HEADROOM` | (section R) a climb the motors cannot sustain: T_hold within min(4600, 10 %) of the ceiling in force; level moves, descents, `stop`, `land` work; come down or land |
| `drone_event` `HOLD_LIMIT` | (section R) the camera hold's roll or pitch correction has sat at >= 1.3 of its 1.5 deg for 8 s outside motions: the drone needs that lean and may creep (`creeps`); photo, watch the drift, land soon; `espfly_diag.py trimcheck` on the dump; check legs, props, battery seat. Nothing landed |
| LAND `COLLISION` | an in-air contact (or a hard bump): the drone stopped any move and lands slowly with the camera hold off (KILLED `COLLISION` before 2026-10-13's change). Let it land (`k` only if it heads for someone); then check the drone, the props and whatever it touched; save `flights` and the recorder (`dump`) before anything else; the next ARM waits for the stillness and the heat soak |
| KILLED `TILT` or `RATE` right after LAND `COLLISION` | the contact flipped or spun the drone: the trips cut it as always; as above |
| LAND `PALM` | an upward push or lift of the hovering drone (a palm from below, from 1 s into the hold), or a strong updraft / a sudden ground effect over furniture: the normal slow LAND with the support probe. On a palm the motors stop within ~1 s (IDLE `TOUCHDOWN`, vz `TOUCHDOWN_FORCE` or `TOUCHDOWN_SUPPORT`); without a palm it lands on the floor (after an updraft possibly hard, a2b 18.3). Unexpected ones: `dump` and report |
| KILLED `ATT_ERR` with the drone on a palm | the hand held it tilted over 10 deg for 0.5 s before the motors stopped: the motors are off; it clears to IDLE by itself; hold the palm flat |
| `arm` refused `STILL` with 'IMU heat soak: wait N s' | FLIGHT after a flight: the accelerometer warms up for a minute or so after landing (20 s + 1.5 x the flight's motor time, at most 90 s); wait and arm again |
| `arm` or `sortie` reply with `stops` | the ARM may have acted without an answer: the daemon sent DISARM (and LAND if it saw ARMED or beyond). Check `state`, then arm again. |
| `drone link is not up`, `no HELLO_ACK ... after 5 tries`, `send ... failed: No route to host` | the Mac is not (yet) on the drone's AP: wait 10-20 s after a reset; rejoin by hand. With the AP joined: a wrong key (S5). |
| event `drone_rebooted` | the drone reset: the daemon refuses ARM; save `crash` and `flights`; restart the daemon |
| event `session_lost` | another session replaced the daemon's mid-sortie, most likely a drone reset (brownout). `auto status` `rst=`; save `crash` and `flights`; restart the daemon once IDLE |
| "daemon socket present but not answering" | a daemon is stopped or hung: end it (or remove `~/.espfly/auto.sock`); `--force-hello` lands any active sortie |
| `takeoff` reply `"reconciled": true`, or `UNKNOWN` | the drone took the command but the answer was lost; check `state` before anything else |
| `land` reply "not confirmed within 3 s" | the daemon keeps sending it for 10 s; check `state`; `k` if the drone must stop now |
| `ERR busy: autonomy active` | a console change while active (expected in A2.10), or in IDLE an ARM was being taken: repeat |
| `ERR mguard test: guard already tripped (reboot)` / `bench only; set gcap ...` | `reboot` (latches table); set `gcap 15000` first |
| `ERR reboot: thrust is requested` or `ERR rec dump: land and set throttle to zero first` | throttle to zero (and wait for IDLE) |
| a console reply that does not match its command | two console clients ran at once: stop one, repeat |
| a reset-lock check gives `resetLocked=0` | stop: save `auto rtc`, `auto status`; no phase B or C |
| below 940 loops/s | stop at DRY (A4); A4b prices each a2b section 19 addition |
| `bypass=0x..` not 0 in `status` / `auto prof` | a bisect toggle is on (RAM): `bypass all off` or reboot before BENCH / FLIGHT |
| `auto prof` `bypassSeen=0x..` not 0 with `bypass=0x00` | a toggle was on since `auto prof reset`: the reading is a bisect one, no go/no-go; `auto prof reset` and repeat A4 |
| `crash` `details=***ERROR*** A stack overflow in task USBDIAG` | the USB console's task ran out of stack (82bcefb's `status`, a2b 26). On this build or later: save `crash` and the commands typed before it, and report; `status` `diagStackFree` shows the margin |
