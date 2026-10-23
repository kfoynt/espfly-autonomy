#!/usr/bin/env python3
"""
ESP-FLY USB diagnostics helper (talks to components/.../usb_diag.c).

Requires pyserial (already installed with ESP-IDF / esptool).
Opening the port does not reset the drone: DTR/RTS are left asserted as
macOS/Linux set them on open, and the chip only resets on RTS without DTR.

Examples (run from Firmware/esp-drone):
  python3 tools/espfly_diag.py status
  python3 tools/espfly_diag.py measure --ms 5000 --out pos0.json
  python3 tools/espfly_diag.py measure --ms 5000 --out pos180.json   # drone turned 180 deg
  python3 tools/espfly_diag.py level-cal pos0.json pos180.json --apply --save
  python3 tools/espfly_diag.py drift --minutes 5 --out drift.csv
  python3 tools/espfly_diag.py dump --out flight.csv
  python3 tools/espfly_diag.py analyze flight.csv
  python3 tools/espfly_diag.py flights         # includes the camera height observer (shadow) per flight
  python3 tools/espfly_diag.py camvz           # the observer now: estimate, counters, stabilizer-task cost
  python3 tools/espfly_diag.py auto status     # autonomy (A2b): state, guards, reset-lock, link
  python3 tools/espfly_diag.py auto get        # settings; also saved for the Mac tool (~/.espfly/auto_get.json)
  python3 tools/espfly_diag.py auto set mvspeed 150   # fast moves: the motion speed step, 100..300 % (read at the
                                                      # next ARM; 'auto reset mvspeed' back to 100; docs/autonomy-a2b.md 31)
  python3 tools/espfly_diag.py auto set mvfloor 40    # the guard margins (cm; mvfloor mvfstop mvprogm mvprogr mvlatm
                                                      # mvnear), read at the next ARM; docs/autonomy-a2b.md 32
  python3 tools/espfly_diag.py auto set mvfix 59      # the move fixes (bits: 1 DRIFT 2 LONG 4 STILL 8 GAIN 16 BRAKE
                                                      # 32 FRONT; default 59 = all but STILL; 0 = none, as before
                                                      # 2026-10-17), read at the next ARM; docs/autonomy-a2b.md 33, 34
  python3 tools/espfly_diag.py auto set mvacc 30      # above mvspeed 100: the acceleration cap (cm/s^2, 10..40,
                                                      # default 20) ...
  python3 tools/espfly_diag.py auto set mvsettle 1000 # ... and the settle's floor (ms, 1000..3000, default 2500): 30
                                                      # and 1000 are 'real 300 %' (4dea542's profiles, the move fixes
                                                      # kept), read at the next ARM; docs/autonomy-a2b.md 35
  python3 tools/espfly_diag.py auto prof       # loop rate and cost while active, with the bench go/no-go
  python3 tools/espfly_diag.py auto rtc        # reset-lock record and the last transitions
  python3 tools/espfly_diag.py auto unlock     # clear the reset-lock after a flash/reboot (USB only; IDLE, motors stopped)
  python3 tools/espfly_diag.py gtc             # gyro temperature compensation: config, re-zeros, post-flight check
  python3 tools/espfly_diag.py bypass gtc on   # loop-rate bisect toggle (RAM, 0 at boot; docs/autonomy-bench.md A4b)
  python3 tools/espfly_diag.py colourcheck G.json C.json   # colour bench: Y identity of a grey and a colour photo
"""
import argparse
import bisect
import csv
import glob
import json
import math
import os
import re
import statistics
import sys
import time

RECORD_FIELDS = ("idx,tickMs,flightId,flags,thrust,rollDes_cdeg,pitchDes_cdeg,roll_cdeg,pitch_cdeg,yaw_cdeg,"
                 "gx_ddps,gy_ddps,gz_ddps,ax_mg,ay_mg,az_mg,m1,m2,m3,m4,rollI_cdps,pitchI_cdps,"
                 "rollRateI,pitchRateI,yawRateDes_ddps,vbat_mv,flowSeq,flowOk,flowLum,flowTexture,"
                 "flowResidual_dg,flowDt_100us,flowAge_ms,flowDx_cpx,flowDy_cpx,flowScale_e4,flowRot_e4,"
                 "flowGx_e4,flowGy_e4,flowGz_e4,holdFlags,holdRoll_cdeg,holdPitch_cdeg,rawThrust,linkAge_ms,"
                 "linkFlags,cvH_mm,cvV_mms,cvBias_cmg,cvGain_dpm,cvFlags,cvSigV_mms,"
                 "autoState,autoReason,vzPhase,vzH_mm,vzV_mms,vzThrust,autoFlags,"
                 "mvState,mvRefLat_e4,mvRefFwd_e4,mvVRef_mms,mvYawOff_cdeg,"
                 "imuTemp_cC,gtcCorrX_mdps,gtcCorrY_mdps,mkiBiasX_mdps,mkiBiasY_mdps,flowGain_e3,flowSat_pct,"
                 "flowExpGate,atcCorrZ_cmg").split(",")
# Firmware before the camera height observer (2026-10-11) sends the first 46 fields, firmware
# before autonomy A2b the first 52, A2b before the interactive hover the first 59, before the
# thermal-drift and exposure columns (docs/autonomy-a2b.md 19.3, 2026-10-14) the first 64; the
# missing columns (cv*, then auto*/vz*, then mv*, then imuTemp..flowExpGate) are empty (None); firmware
# before the accelerometer compensation's column (docs/autonomy-a2b.md 36, 2026-10-18) the first 72.
RECORD_FIELDS_V1 = RECORD_FIELDS[:RECORD_FIELDS.index("cvH_mm")]
RECORD_FIELDS_V2 = RECORD_FIELDS[:RECORD_FIELDS.index("autoState")]
RECORD_FIELDS_V3 = RECORD_FIELDS[:RECORD_FIELDS.index("mvState")]
RECORD_FIELDS_V4 = RECORD_FIELDS[:RECORD_FIELDS.index("imuTemp_cC")]
RECORD_FIELDS_V5 = RECORD_FIELDS[:RECORD_FIELDS.index("atcCorrZ_cmg")]
RECORD_LENGTHS = (len(RECORD_FIELDS), len(RECORD_FIELDS_V5), len(RECORD_FIELDS_V4), len(RECORD_FIELDS_V3),
                  len(RECORD_FIELDS_V2), len(RECORD_FIELDS_V1))

FLAG_ARMED, FLAG_INHIBITED, FLAG_ANGLE = 0x01, 0x02, 0x04
# holdFlags (flight_recorder.h): the camera drift hold. Bit 3: the latest frame was refused by the
# HOVER scene gate (camera_hold.h camHoldSceneOk; a person or another moving object in the view).
HOLD_ENABLED, HOLD_APPLIED, HOLD_COASTING, HOLD_SCENE_GATED = 0x01, 0x02, 0x04, 0x08
HOLD_FLAG_NAMES = ("enabled", "applied", "coasting", "sceneGated")


def hold_flag_names(bits):
    """Names of the set holdFlags bits (unknown bits as bitN)."""
    bits = int(bits or 0)
    return [HOLD_FLAG_NAMES[b] if b < len(HOLD_FLAG_NAMES) else f"bit{b}" for b in range(8) if bits & (1 << b)]


def hold_summary(rows):
    """The camera drift hold over recorder rows: percentages of the rows applied and coasting, and the
    camera frames (new flowSeq) the HOVER scene gate refused; None without hold columns."""
    rows = [r for r in rows if r.get("holdFlags") is not None]
    if not rows:
        return None
    n = len(rows)
    frames = gated = 0
    prev = None
    for r in rows:
        seq = r.get("flowSeq")
        if seq and seq != prev:
            frames += 1
            gated += bool(r["holdFlags"] & HOLD_SCENE_GATED)
        prev = seq
    return {"applied_pct": 100.0 * sum(1 for r in rows if r["holdFlags"] & HOLD_APPLIED) / n,
            "coasting_pct": 100.0 * sum(1 for r in rows if r["holdFlags"] & HOLD_COASTING) / n,
            "frames": frames, "scene_gated": gated}
MAX_OFFSET_G = 0.15


# ---------------------------------------------------------------- transport

def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No USB serial port found (looked for /dev/cu.usbmodem*, /dev/ttyACM*). Use --port.")
    return ports[0]


class Drone:
    def __init__(self, port):
        import serial  # imported lazily so 'analyze' works without pyserial
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        self.buf = b""

    def close(self):
        self.ser.close()

    def readline(self, deadline):
        while time.time() < deadline:
            nl = self.buf.find(b"\n")
            if nl >= 0:
                line, self.buf = self.buf[:nl], self.buf[nl + 1:]
                return line.decode("ascii", "replace").strip("\r")
            chunk = self.ser.read(4096)
            if chunk:
                self.buf += chunk
        return None

    def drain(self):
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.buf = b""

    def command(self, cmd, timeout=3.0, until=("OK", "ERR")):
        """Send a command; return (reply_line, other_lines). Ignores ESP_LOG noise."""
        self.drain()
        self.ser.write((cmd + "\n").encode())
        deadline = time.time() + timeout
        others = []
        while True:
            line = self.readline(deadline)
            if line is None:
                raise TimeoutError(f"no reply to '{cmd}' within {timeout:.0f} s "
                                   "(is the diagnostics firmware flashed?)")
            word = line.split(" ", 1)[0]
            if word in until:
                return line, others
            others.append(line)


def parse_kv(reply):
    out = {}
    for tok in reply.split()[2:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def floats(s):
    return [float(x) for x in s.split(",")]


def read_rows(f):
    """Recorder CSV rows as ints; empty cells (the cv* columns of an older firmware's dump) are left out."""
    return [{k: int(v) for k, v in r.items() if v not in ("", None)} for r in csv.DictReader(f)]


def parse_record(line):
    """'R,...*CS' or 'S,...*CS' -> dict, or None if malformed/corrupted."""
    if "*" not in line:
        return None
    body, cs = line.rsplit("*", 1)
    try:
        want = int(cs[:2], 16)
    except ValueError:
        return None
    got = 0
    for ch in body.encode():
        got ^= ch
    if got != want:
        return None
    parts = body.split(",")
    if len(parts) - 1 not in RECORD_LENGTHS:
        return None
    try:
        vals = [int(p) for p in parts[1:]]
    except ValueError:
        return None
    rec = dict.fromkeys(RECORD_FIELDS)
    rec.update(zip(RECORD_FIELDS, vals))
    return rec


# ---------------------------------------------------------------- commands

# The USB console task's stack: 'status' diagStackFree= is the least it has had free since boot, bytes
# (uxTaskGetStackHighWaterMark; 82bcefb's 'status' overflowed it, docs/autonomy-a2b.md 26). The bench's L0 floor.
DIAG_STACK_MIN = 1024


def diag_stack_note(reply):
    """A warning when a 'status' reply shows the console task's stack margin under DIAG_STACK_MIN; None when it
    is enough or the reply has no (readable) diagStackFree (a firmware before 2026-10-16)."""
    v = parse_kv(reply).get("diagStackFree")
    try:
        free = int(v)
    except (TypeError, ValueError):
        return None
    if free >= DIAG_STACK_MIN:
        return None
    return (f"WARNING: diagStackFree={free} B < {DIAG_STACK_MIN}: the USB console task's stack is near its end; "
            f"report it (docs/autonomy-bench.md L0)")


def cam_split_note(reply):
    """The colour split's line for a 'status' reply (docs/autonomy-a2b.md 28): a warning when its boot check failed
    (camSplit=bad: photocol 1 stored, grey photos this boot), its photo counters while it runs; None for grey
    (camSplit=off) and for a firmware before 2026-10-16 (no camSplit)."""
    kv = parse_kv(reply)
    split = kv.get("camSplit")
    if split == "bad":
        return ("WARNING: camSplit=bad: the colour split failed its boot check (photocol 1 stored): grey photos this "
                "boot; report the boot log's 'colour split' line (docs/autonomy-bench.md N'0)")
    if split == "ok":
        c = {k: kv.get("camSplit" + k, "?") for k in ("Arm", "Done", "Rearm", "Cancel", "Restart")}
        return (f"colour: always on (camPix={kv.get('camPix', '?')}): photos armed {c['Arm']}, filled {c['Done']}, "
                f"re-armed {c['Rearm']}, cancelled {c['Cancel']}, restarted {c['Restart']}")
    return None


def cmd_status(d, args):
    reply, _ = d.command("status")
    print(reply)
    for note in (diag_stack_note(reply), cam_split_note(reply), t2_status_line(parse_kv(reply))):
        if note:
            print(note)


# The colour bench checks (docs/autonomy-bench.md N'1.4 and N'1.5), offline on two saved photos of the same still
# scene: a grey one (photocol 0) and a colour one (photocol 1, its raw .yuyv written by espfly_auto.py).
COLOUR_Y_MAD_MAX = 2.0         # mean |Y grey - Y colour|, levels
COLOUR_Y_EXT_TOL = 1           # min and max Y the same within this
COLOUR_Y_BRIGHT_MIN = 250      # max Y in both (no 16-235 clipping): a bright window in view
COLOUR_WHITE_UV = (124, 132)   # a white sheet: mean U and V
COLOUR_WHITE_RGB_SPREAD = 10   # ... and R, G, B means within this of each other


def _photo_doc(path):
    """A saved photo's JSON (from its .json, .png, _x4.png or .yuyv path) and its base path."""
    for suffix in ("_x4.png", ".png", ".json", ".yuyv"):
        if path.endswith(suffix):
            base = path[:-len(suffix)]
            break
    else:
        base = path
    with open(base + ".json") as f:
        return json.load(f), base


def photo_luma(path):
    """(Y bytes in the order sent, the JSON, chroma (U bytes, V bytes) or None) of a saved photo: a grey photo's .png
    (rows put back in the sent order when it was saved flipped), a colour photo's raw .yuyv."""
    doc, base = _photo_doc(path)
    w, h = doc.get("width", 160), doc.get("height", 120)
    if doc.get("color"):
        raw_path = doc.get("files", {}).get("raw") or base + ".yuyv"
        if not os.path.exists(raw_path):
            raise SystemExit(f"{base}: a colour photo without its raw .yuyv (saved by a Mac tool before 2026-10-16)")
        with open(raw_path, "rb") as f:
            raw = f.read()
        if len(raw) != 2 * w * h:
            raise SystemExit(f"{raw_path}: {len(raw)} bytes, want {2 * w * h}")
        return raw[0::2], doc, (raw[1::4], raw[3::4])
    from PIL import Image
    with Image.open(base + ".png") as im:
        if im.mode != "L":
            raise SystemExit(f"{base}.png: mode {im.mode}, want a grey photo")
        y = im.tobytes()
    if doc.get("flippedVertical"):
        y = b"".join(y[r * w:(r + 1) * w] for r in range(h - 1, -1, -1))
    return y, doc, None


def colour_check(grey_y, colour_y, chroma=None, rgb_means=None, white=False, orange=False):
    """The N' checks on two Y frames of the same still scene (grey photo, colour photo) and the colour photo's chroma
    and RGB means: a dict with the numbers and 'pass' per check (None: not asked)."""
    n = min(len(grey_y), len(colour_y))
    mad = sum(abs(grey_y[i] - colour_y[i]) for i in range(n)) / n
    out = {"y_mad": round(mad, 3), "grey_min": min(grey_y), "grey_max": max(grey_y), "colour_min": min(colour_y),
           "colour_max": max(colour_y)}
    out["y_identity_pass"] = (mad <= COLOUR_Y_MAD_MAX and abs(out["grey_min"] - out["colour_min"]) <= COLOUR_Y_EXT_TOL
                              and abs(out["grey_max"] - out["colour_max"]) <= COLOUR_Y_EXT_TOL
                              and min(out["grey_max"], out["colour_max"]) >= COLOUR_Y_BRIGHT_MIN)
    out["white_pass"] = out["orange_pass"] = None
    if chroma is not None:
        u, v = chroma
        out["mean_u"], out["mean_v"] = round(sum(u) / len(u), 2), round(sum(v) / len(v), 2)
    if rgb_means is not None:
        out["mean_rgb"] = [round(c, 1) for c in rgb_means]
    if white and chroma is not None and rgb_means is not None:
        lo, hi = COLOUR_WHITE_UV
        out["white_pass"] = (lo <= out["mean_u"] <= hi and lo <= out["mean_v"] <= hi
                             and max(rgb_means) - min(rgb_means) <= COLOUR_WHITE_RGB_SPREAD)
    if orange and rgb_means is not None:
        r, g, b = rgb_means
        out["orange_pass"] = r > g > b
    return out


def cmd_colourcheck(_d, args):
    """Offline: the colour bench's Y identity (and the white-sheet / orange checks) on two saved photos."""
    gy, gdoc, _ = photo_luma(args.grey)
    cy, cdoc, chroma = photo_luma(args.colour)
    if gdoc.get("color") or not cdoc.get("color"):
        raise SystemExit("want a grey photo (photocol 0) first, then a colour one (photocol 1)")
    rgb = None
    from PIL import Image
    _, cbase = _photo_doc(args.colour)
    with Image.open(cbase + ".png") as im:
        px = im.convert("RGB").tobytes()
    rgb = tuple(sum(px[c::3]) / (len(px) // 3) for c in range(3))
    res = colour_check(gy, cy, chroma, rgb, white=args.white, orange=args.orange)
    if args.json:
        print(json.dumps(res, indent=1))
    else:
        print(f"Y identity: mean |grey - colour| {res['y_mad']:.2f} (<= {COLOUR_Y_MAD_MAX}), min {res['grey_min']} / "
              f"{res['colour_min']}, max {res['grey_max']} / {res['colour_max']} (the same within "
              f"{COLOUR_Y_EXT_TOL}, >= {COLOUR_Y_BRIGHT_MIN} in both): {'PASS' if res['y_identity_pass'] else 'FAIL'}")
        print(f"colour photo: mean U {res['mean_u']}, V {res['mean_v']}; mean R/G/B "
              f"{'/'.join(str(c) for c in res['mean_rgb'])}")
        if res["white_pass"] is not None:
            print(f"white sheet: U and V in {COLOUR_WHITE_UV[0]}-{COLOUR_WHITE_UV[1]}, R/G/B within "
                  f"{COLOUR_WHITE_RGB_SPREAD}: {'PASS' if res['white_pass'] else 'FAIL'}")
        if res["orange_pass"] is not None:
            print(f"orange object: R > G > B: {'PASS' if res['orange_pass'] else 'FAIL'}")
        if not res["y_identity_pass"]:
            print("A Y-identity failure is a hard stop: 'auto set photocol 0', reboot, report (docs/autonomy-bench.md "
                  "N'1.4)")
    return 0 if res["y_identity_pass"] and res["white_pass"] is not False and res["orange_pass"] is not False else 1


def cmd_raw(d, args):
    reply, others = d.command(" ".join(args.text), timeout=args.timeout)
    for o in others:
        print(o)
    print(reply)


def measure(d, ms):
    reply, _ = d.command(f"measure {ms}", timeout=ms / 1000.0 + 5)
    if reply.startswith("ERR"):
        sys.exit(reply)
    kv = parse_kv(reply)
    m = {
        "n": int(kv["n"]), "ms": int(kv["ms"]), "still": kv["still"] == "1",
        "motorsZero": kv["motorsZero"] == "1", "maxThrust": int(kv["maxThrust"]),
        "acc": floats(kv["acc"]), "accStd": floats(kv["accStd"]),
        "gyro": floats(kv["gyro"]), "gyroStd": floats(kv["gyroStd"]), "gyroBias": floats(kv["gyroBias"]),
        "calibrated": kv["calibrated"] == "1", "level": floats(kv["level"]),
        "accScale": float(kv["accScale"]),
        "accRoll": float(kv["accRoll"]), "accPitch": float(kv["accPitch"]),
        "estRoll": float(kv["estRoll"]), "estPitch": float(kv["estPitch"]),
        "estRollStd": float(kv["estRollStd"]), "estPitchStd": float(kv["estPitchStd"]),
        "time": time.time(), "raw": reply,
    }
    return m


def level_offset(acc_a, acc_b=None):
    """Level offset (g) from raw mean acceleration at heading A and, optionally,
    at A + 180 deg on the same surface. Turning the drone about the vertical
    reverses the surface tilt in the body frame but not the sensor offset, so
    the average is the offset and half the difference is the surface tilt.
    Returns ((ox, oy), (tilt_x, tilt_y)); tilt is (0, 0) for a single position."""
    if acc_b is None:
        return (acc_a[0], acc_a[1]), (0.0, 0.0)
    return (((acc_a[0] + acc_b[0]) / 2, (acc_a[1] + acc_b[1]) / 2),
            ((acc_a[0] - acc_b[0]) / 2, (acc_a[1] - acc_b[1]) / 2))


def tilt_deg(x, y, z):
    """Estimator conventions (sensfusion6GetEulerRPY): roll=atan2(y,z), pitch=asin(x/|a|)."""
    n = math.sqrt(x * x + y * y + z * z)
    return math.degrees(math.atan2(y, z)), math.degrees(math.asin(x / n))


def print_measure(m, label=""):
    ax, ay, az = m["acc"]
    r, p = tilt_deg(ax, ay, az)
    print(f"{label}samples={m['n']} still={m['still']} motorsZero={m['motorsZero']} gyroCalibrated={m['calibrated']}")
    print(f"  raw acc (g)       x={ax:+.4f} y={ay:+.4f} z={az:+.4f}  -> sensor-only tilt roll={r:+.2f} pitch={p:+.2f} deg")
    print(f"  acc std (g)       {m['accStd'][0]:.4f} {m['accStd'][1]:.4f} {m['accStd'][2]:.4f}")
    print(f"  level offset (g)  x={m['level'][0]:+.4f} y={m['level'][1]:+.4f}")
    print(f"  gyro mean (dps)   x={m['gyro'][0]:+.3f} y={m['gyro'][1]:+.3f} z={m['gyro'][2]:+.3f}  "
          f"(std {m['gyroStd'][0]:.3f} {m['gyroStd'][1]:.3f} {m['gyroStd'][2]:.3f})")
    print(f"  attitude (deg)    estimator roll={m['estRoll']:+.2f} pitch={m['estPitch']:+.2f} | "
          f"accel-implied roll={m['accRoll']:+.2f} pitch={m['accPitch']:+.2f}")
    if not m["still"]:
        print("  WARNING: the drone was moving or vibrating during the measurement; repeat it.")
    if m["maxThrust"] > 0:
        print("  WARNING: thrust was requested during the measurement (motors were held at zero).")


def cmd_measure(d, args):
    m = measure(d, args.ms)
    print_measure(m)
    if args.out:
        with open(args.out, "w") as f:
            json.dump(m, f, indent=1)
        print(f"saved {args.out}")


def cmd_level_cal(d, args):
    a = json.load(open(args.a))
    b = json.load(open(args.b)) if args.b else None
    problems = []
    for name, m in (("A", a), ("B", b)):
        if m is None:
            continue
        if not m["still"]:
            problems.append(f"measurement {name} was not still")
        if not m["calibrated"]:
            problems.append(f"measurement {name} was taken before gyro calibration finished")
        if not (0.85 < m["acc"][2] < 1.15):
            problems.append(f"measurement {name} z={m['acc'][2]:.3f} g: drone not upright?")

    (ox, oy), tilt = level_offset(a["acc"], b["acc"] if b else None)
    if b:
        print("Two-position level calibration (B = A turned 180 deg about vertical)")
        print(f"  A raw x={a['acc'][0]:+.4f} y={a['acc'][1]:+.4f} | B raw x={b['acc'][0]:+.4f} y={b['acc'][1]:+.4f} g")
        print(f"  surface tilt seen in A's frame: pitch {math.degrees(math.asin(max(-1, min(1, tilt[0])))):+.2f} deg, "
              f"roll {math.degrees(math.atan(tilt[1])):+.2f} deg (cancelled out)")
    else:
        print("Single-position level calibration: assumes the surface is exactly level.")
    pitch_eq = math.degrees(math.asin(max(-1, min(1, ox))))
    roll_eq = math.degrees(math.atan(oy))
    print(f"  level offset: x={ox:+.5f} g, y={oy:+.5f} g")
    print(f"  = the drone currently believes 'level' is pitch {pitch_eq:+.2f} deg, roll {roll_eq:+.2f} deg off")
    if abs(ox) > MAX_OFFSET_G or abs(oy) > MAX_OFFSET_G:
        problems.append(f"offset exceeds the +/-{MAX_OFFSET_G} g firmware limit")
    if problems:
        print("NOT APPLYING:\n  - " + "\n  - ".join(problems))
        return 1
    if not args.apply:
        print("(dry run: add --apply to set it live, and --save to persist it)")
        return 0
    reply, _ = d.command(f"cal set {ox:.5f} {oy:.5f}")
    print(reply)
    if reply.startswith("ERR"):
        return 1
    if args.save:
        reply, _ = d.command("cal save")
        print(reply)
        if reply.startswith("ERR"):
            return 1
    return 0


def cmd_cal(d, args):
    cmd = "cal " + " ".join(args.args) if args.args else "cal show"
    reply, _ = d.command(cmd)
    print(reply)
    return 1 if reply.startswith("ERR") else 0


def steady_torque(rows, settle_ms=2000):
    """Per-flight median roll/pitch actuator output in steady, hands-off hover,
    recovered from the motor commands by inverting the X mixer
    (m1 = T - r + p + y, m2 = T - r - p - y, m3 = T + r - p + y, m4 = T + r + p - y,
    with r = roll/2, p = pitch/2). Returns [(flightId, n, roll, pitch)]."""
    flights = {}
    for r in rows:
        flights.setdefault(r["flightId"], []).append(r)
    out = []
    for fid, fr in flights.items():
        t0 = next((r["tickMs"] for r in fr if r["thrust"] > 0), None)
        if t0 is None:
            continue
        s = [r for r in fr if r["thrust"] > 40000 and r["tickMs"] - t0 >= settle_ms
             and abs(r["roll_cdeg"]) < 500 and abs(r["pitch_cdeg"]) < 500
             and r["rollDes_cdeg"] == 0 and r["pitchDes_cdeg"] == 0
             and max(r["m1"], r["m2"], r["m3"], r["m4"]) < 65000]
        if len(s) < 50:  # need at least 0.5 s of steady hover
            continue
        roll = sorted(((r["m3"] + r["m4"]) - (r["m1"] + r["m2"])) / 2 for r in s)[len(s) // 2]
        pitch = sorted(((r["m1"] + r["m4"]) - (r["m2"] + r["m3"])) / 2 for r in s)[len(s) // 2]
        out.append((fid, len(s), roll, pitch))
    return out


def frames(rows):
    """One row per camera frame (the recorder repeats a sample until the next frame)."""
    out, last = [], None
    for r in rows:
        if r.get("flowSeq", 0) and r["flowSeq"] != last:
            out.append(r)
            last = r["flowSeq"]
    return out


def lstsq(X, y):
    """Least squares for a few columns via normal equations (no numpy needed)."""
    n = len(X[0])
    A = [[sum(row[i] * row[j] for row in X) for j in range(n)] for i in range(n)]
    b = [sum(row[i] * yy for row, yy in zip(X, y)) for i in range(n)]
    for c in range(n):  # Gauss-Jordan with partial pivoting
        p = max(range(c, n), key=lambda r: abs(A[r][c]))
        A[c], A[p], b[c], b[p] = A[p], A[c], b[p], b[c]
        if abs(A[c][c]) < 1e-12:
            return None
        for r in range(n):
            if r != c:
                f = A[r][c] / A[c][c]
                A[r] = [a - f * ac for a, ac in zip(A[r], A[c])]
                b[r] -= f * b[c]
    coef = [b[i] / A[i][i] for i in range(n)]
    pred = [sum(c * v for c, v in zip(coef, row)) for row in X]
    mean = sum(y) / len(y)
    ss_tot = sum((v - mean) ** 2 for v in y) or 1e-12
    ss_res = sum((v - p) ** 2 for v, p in zip(y, pred))
    return coef, 1 - ss_res / ss_tot, (ss_res / len(y)) ** 0.5


def flow_calibration(rows, min_texture=5, max_residual=None):
    """Fit image motion per frame against the body rotation over the same
    interval: dx, dy (px) and image rotation (rad) vs gyro angles about
    body x, y, z (rad), plus a constant."""
    fr = [r for r in frames(rows) if r["flowOk"] and r["flowTexture"] >= min_texture
          and (max_residual is None or (r.get("flowResidual_dg") is not None and r["flowResidual_dg"] < max_residual * 10))]
    if len(fr) < 30:
        return None
    X = [[r["flowGx_e4"] / 1e4, r["flowGy_e4"] / 1e4, r["flowGz_e4"] / 1e4, 1.0] for r in fr]
    fits = {}
    for name, key, scale in (("dx", "flowDx_cpx", 100.0), ("dy", "flowDy_cpx", 100.0), ("rot", "flowRot_e4", 1e4)):
        res = lstsq(X, [r[key] / scale for r in fr])
        if res is None:
            return None
        coef, r2, rms = res
        fits[name] = {"gx": coef[0], "gy": coef[1], "gz": coef[2], "c": coef[3], "r2": r2, "rms": rms}
    span = [max(abs(r["flowG%s_e4" % a]) for r in fr) / 1e4 for a in "xyz"]
    return {"frames": len(fr), "fits": fits, "gyroSpanRad": span}


def gyro_series(rows):
    """The recorder's 100 Hz body rates as (t_ms list, cumulative angle lists x/y/z in rad) for gyro_angle():
    trapezoids between the samples (linear rate between them)."""
    seen = {}
    for r in rows:
        seen[r["tickMs"]] = r
    ts = sorted(seen)
    k = math.pi / 180.0 / 10.0          # 0.1 deg/s -> rad/s
    rates = [[seen[t][f"g{a}_ddps"] * k for t in ts] for a in "xyz"]
    cum = [[0.0] * len(ts) for _ in range(3)]
    for i in range(1, len(ts)):
        h = (ts[i] - ts[i - 1]) / 1000.0
        for a in range(3):
            cum[a][i] = cum[a][i - 1] + 0.5 * (rates[a][i] + rates[a][i - 1]) * h
    return ts, rates, cum


def _angle_to(series, t):
    ts, rates, cum = series
    i = bisect.bisect_right(ts, t) - 1
    if i < 0 or i >= len(ts) - 1:
        return None
    h = (t - ts[i]) / 1000.0
    f = (t - ts[i]) / float(ts[i + 1] - ts[i])
    out = []
    for a in range(3):
        r0 = rates[a][i]
        rt = r0 + (rates[a][i + 1] - r0) * f
        out.append(cum[a][i] + 0.5 * (r0 + rt) * h)
    return out


def gyro_angle(series, t0, t1):
    """Body rotation (rad, x/y/z) over [t0, t1] ms from gyro_series(); None outside the samples."""
    a0, a1 = _angle_to(series, t0), _angle_to(series, t1)
    if a0 is None or a1 is None:
        return None
    return [b - a for a, b in zip(a0, a1)]


def flow_offset_scan(rows, offsets=range(0, 41), min_texture=5, max_residual=None):
    """The camera's gyro window offset (docs/autonomy-a2b.md 17.2): each frame's rotation recomputed from the
    100 Hz gyro samples over [t - dt + off, t + off] (t its capture time on the recorder clock, tickMs - flowAge
    at its first sighting; dt its interval) for every offset in ms, dx and dy fitted against it as the
    calibration does; the offset whose mean R^2 is highest. None without 30 usable frames."""
    fr = [r for r in frames(rows) if r["flowOk"] and r["flowTexture"] >= min_texture and r["flowDt_100us"] > 0
          and (max_residual is None or (r.get("flowResidual_dg") is not None and r["flowResidual_dg"] < max_residual * 10))]
    if len(fr) < 30:
        return None
    series = gyro_series(rows)
    scan = []
    for off in offsets:
        X, ydx, ydy = [], [], []
        for r in fr:
            t = r["tickMs"] - r["flowAge_ms"]
            a = gyro_angle(series, t - r["flowDt_100us"] / 10.0 + off, t + off)
            if a is None:
                continue
            X.append([a[0], a[1], a[2], 1.0])
            ydx.append(r["flowDx_cpx"] / 100.0)
            ydy.append(r["flowDy_cpx"] / 100.0)
        if len(X) < 30:
            continue
        fx, fy = lstsq(X, ydx), lstsq(X, ydy)
        if fx is None or fy is None:
            continue
        scan.append({"off_ms": off, "r2": 0.5 * (fx[1] + fy[1]), "r2_dx": fx[1], "r2_dy": fy[1],
                     "dx_gz": fx[0][2], "frames": len(X)})
    if not scan:
        return None
    best = max(scan, key=lambda e: e["r2"])
    return {"best_ms": best["off_ms"], "r2": best["r2"], "dx_gz": best["dx_gz"], "scan": scan}


def print_offset_scan(res, now_us=None, origin="legacy-start"):
    if res is None:
        print("offset scan: not enough usable frames")
        return
    label = "residual correction to published capture-centre timestamps" if origin in ("native-centre", "vga-centre") else "offset from legacy frame-start timestamps"
    print(f"gyro window offset scan ({label}; mean R^2 of dx/dy fits):")
    print("  " + "  ".join(f"{e['off_ms']}:{e['r2']:.3f}" for e in res["scan"] if e["off_ms"] % 4 == 0))
    print(f"best offset {res['best_ms']} ms (R^2 {res['r2']:.3f}, dx vs gz {res['dx_gz']:.1f} px/rad)"
          + ("" if now_us is None else f"; the firmware uses {now_us / 1000:.0f} ms ('status' camGyroOffUs)"))


def cmd_flow_cal(d, args):
    if args.csv:
        with open(args.csv) as f:
            rows = read_rows(f)
    else:
        print(f"Recording {args.seconds:.0f} s at 100 Hz. Hold the drone facing a textured scene and slowly")
        print("turn it left/right (yaw), then tilt the nose up/down (pitch), then roll it side to side.")
        reply, _ = d.command("stream 100")
        rows, t_end = [], time.time() + args.seconds
        try:
            while time.time() < t_end:
                line = d.readline(t_end)
                if line and line.startswith("S,"):
                    r = parse_record(line)
                    if r:
                        rows.append(r)
        finally:
            d.ser.write(b"stream 0\n")
        if args.out:
            with open(args.out, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(RECORD_FIELDS)
                for r in rows:
                    w.writerow([r[k] for k in RECORD_FIELDS])
            print(f"saved {args.out}")
    fr = frames(rows)
    if fr:
        dts = sorted(r["flowDt_100us"] / 10 for r in fr if r["flowDt_100us"])
        okp = 100 * sum(r["flowOk"] for r in fr) / len(fr)
        print(f"frames: {len(fr)}  median interval {dts[len(dts) // 2] if dts else 0:.1f} ms  ok {okp:.0f}%  "
              f"lum median {sorted(r['flowLum'] for r in fr)[len(fr) // 2]}  "
              f"texture median {sorted(r['flowTexture'] for r in fr)[len(fr) // 2]}")
    origin = getattr(args, "timestamp_origin", "native-centre")
    # Calibrate only measurements that navigation can use; a converged fit
    # with a large image residual can still report flowOk and bias the scan.
    quality = {"min_texture": 8, "max_residual": 8.0} if origin in ("native-centre", "vga-centre") else {}
    cal = flow_calibration(rows, **quality)
    if not cal:
        print("Not enough good frames to fit (need >= 30 with texture). Is the camera running and the scene textured?")
        return 1
    print(f"fit over {cal['frames']} navigation-qualified frames; gyro span (rad) x {cal['gyroSpanRad'][0]:.3f} "
          f"y {cal['gyroSpanRad'][1]:.3f} z {cal['gyroSpanRad'][2]:.3f}")
    for name, fit in cal["fits"].items():
        unit = "px/rad" if name != "rot" else "rad/rad"
        print(f"  {name:3s} = {fit['gx']:+8.1f}*gx {fit['gy']:+8.1f}*gy {fit['gz']:+8.1f}*gz {fit['c']:+.3f}"
              f"   ({unit}; R^2 {fit['r2']:.2f}, residual {fit['rms']:.3f})")
    print("Expect dx dominated by gz and dy by gy with |coefficient| ~ focal length (100-150 px/rad at 160x120),")
    print("and rot dominated by gx with |coefficient| ~ 1.")
    if getattr(args, "offset_scan", False):
        origin = getattr(args, "timestamp_origin", "native-centre")
        res = flow_offset_scan(rows, offsets=range(-40, 41) if origin in ("native-centre", "vga-centre") else range(0, 41), **quality)
        print_offset_scan(res, origin=origin)
        cal["timestampOrigin"] = origin
        if origin in ("native-centre", "vga-centre"):
            cal["captureStartToCentreMs"] = getattr(args, "capture_offset_ms", 37)
            print(f"Native timestamps already include {cal['captureStartToCentreMs']} ms start-to-centre alignment; scan results are additional corrections, ideally near zero.")
        if res is not None:
            cal["offsetScan"] = {"best_ms": res["best_ms"], "r2": res["r2"]}
    if args.save:
        with open(args.save, "w") as f:
            json.dump(cal, f, indent=1)
        print(f"saved calibration {args.save}")
    return 0


def cmd_crash(d, args):
    cmd = "crash " + " ".join(args.args) if args.args else "crash"
    if args.args and args.args[0] in ("clear", "test"):
        reply, _ = d.command(cmd)
        print(reply)
        return 1 if reply.startswith("ERR") else 0
    end, lines = d.command(cmd, until=("END", "ERR"))
    for l in lines:
        print(l)
    print(end)
    return 0


RESET_REASONS = {0: "unknown", 1: "poweron", 2: "ext", 3: "sw", 4: "panic", 5: "int_wdt",
                 6: "task_wdt", 7: "wdt", 8: "deepsleep", 9: "brownout", 10: "sdio"}
FLIGHT_ENDS = {0: "open", 1: "stick released", 2: "link lost", 3: "RESET", 4: "AUTO"}
# Mixer order (power_distribution): m1 front-right, m2 back-right, m3 back-left, m4 front-left.
MOTOR_NAMES = ("FR", "BR", "BL", "FL")


def line_kv(line):
    """key=value tokens after the line tag ("F", "C", ...)."""
    out = {}
    for tok in line.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


# Camera height observer in shadow mode (cam_vz_shadow.h, fsumCamVz_t): flags and the
# "not reached" value of its int16 fields.
CVZ_RAN, CVZ_LIFT, CVZ_LIFT_ACC, CVZ_TD, CVZ_IMPACT, CVZ_GAIN, CVZ_PRIOR, CVZ_FAULT = (
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80)
CVZ_NONE = -32768


def camvz_summary(f):
    """The observer's per-flight fields of an 'F' line (or an 'OK camvz' reply), None if absent.

    Offsets are reported against the flight's ground baseline g0 (bias - g0, the camera
    height study's b_g0: +1.4 +/- 2.2 mg on the logged flights). h_td is the height estimate
    when the drone was back on the floor, i.e. the drift; drift_ms = h_td over the airborne
    time after the camera wake estimates the camera's slow speed offset c (study: ~0.03 m/s).
    """
    if "cvf" not in f:
        return None

    def i(k):
        return int(f.get(k, 0))

    def opt(k, scale):
        v = i(k)
        return None if v == CVZ_NONE else v * scale

    flags = i("cvf")
    g0 = opt("cvG0", 0.1)

    def rel(k):
        b = opt(k, 0.01)
        return None if b is None or g0 is None else b - g0

    cam_air = i("cvCamAir") / 100.0
    h_td = opt("cvHtd", 0.001)
    return {
        "flags": flags, "ran": bool(flags & CVZ_RAN), "lifted": bool(flags & CVZ_LIFT),
        "lift": "acc" if flags & CVZ_LIFT_ACC else "thrust", "touchdown": bool(flags & CVZ_TD),
        "impact": bool(flags & CVZ_IMPACT), "gain_tracked": bool(flags & CVZ_GAIN),
        "prior": bool(flags & CVZ_PRIOR), "fault": bool(flags & CVZ_FAULT),
        "g0_mg": g0, "bias3_mg": opt("cvB3", 0.01), "bias5_mg": opt("cvB5", 0.01), "bias_td_mg": opt("cvBtd", 0.01),
        "b3_g0_mg": rel("cvB3"), "b5_g0_mg": rel("cvB5"), "btd_g0_mg": rel("cvBtd"),
        "sig5_mg": i("cvSb5") / 100.0 if i("cvSb5") else None,
        "gain": i("cvGain") / 10.0 if i("cvGain") else None,
        "h_td": h_td, "v_td": opt("cvVtd", 0.001), "h_max": opt("cvHmax", 0.001),
        "air_s": i("cvAir") / 100.0, "cam_air_s": cam_air,
        "ok_pct": 100.0 * i("cvOk") / i("cvCamAir") if i("cvCamAir") else None,
        "first_ok_s": None if i("cvFirstDs") == 255 else i("cvFirstDs") / 10.0,
        "gap_s": i("cvGap") / 100.0, "accepted": i("cvAcc"), "rejected": i("cvRej"),
        "alarms": i("cvAlarm"), "yaw_resets": i("cvYaw"),
        "drift_ms": h_td / cam_air if h_td is not None and cam_air >= 1.0 else None,
        "late": late_summary(f),
        "t2": t2_summary(f),
    }


# The observer's late offset noise (R1 of the height study, docs/autonomy-a2b.md 25; 'F' line cvq*, firmware of
# 2026-10-15 on): FSUM_CVL_* of flight_summary.h.
CVL_RAN, CVL_STEP, CVL_CFG_BAD = 0x01, 0x02, 0x04


def late_summary(f):
    """Which late offset noise flew ('F' line cvqf/cvq/cvqS/cvqG/cvqDs), None for older firmware or no sortie."""
    if "cvqf" not in f or not int(f["cvqf"]) & CVL_RAN:
        return None
    flags = int(f["cvqf"])
    return {"mult": int(f["cvq"]), "step_s": int(f["cvqS"]), "guards": int(f["cvqG"]), "on_s": int(f["cvqDs"]) / 10.0,
            "step_reached": bool(flags & CVL_STEP), "cfg_bad": bool(flags & CVL_CFG_BAD)}


def late_text(late):
    """'x5 from 40 s: on 46.3 s, 1 guard' or 'off (x1)'; a refused stored setting is flagged."""
    if late is None:
        return "-"
    bad = " (stored setting REFUSED at boot)" if late["cfg_bad"] else ""
    if late["mult"] <= 1:
        return "off (x1)" + bad
    if not late["step_reached"]:
        return f"x{late['mult']} from {late['step_s']} s: not reached{bad}"
    return f"x{late['mult']} from {late['step_s']} s: on {late['on_s']:.1f} s, {late['guards']} guards{bad}"


# The observer's offset drift rate T2 (the late-sink fix, docs/autonomy-a2b.md 29; 'F' line cvt2*, firmware of
# 2026-10-16 on): FSUM_CVL_T2_* of flight_summary.h; rates in 0.01 mg/s, the prior in ug/s.
CVL_T2_ON, CVL_T2_CFG_BAD = 0x08, 0x10


def t2_summary(f):
    """Which T2 flew ('F' line cvqf flags, cvt2k/cvt2r/cvt2Ds/cvt2bR=end,min,max), None for older firmware or no
    sortie."""
    if "cvt2k" not in f or "cvqf" not in f or not int(f["cvqf"]) & CVL_RAN:
        return None
    flags = int(f["cvqf"])
    end, lo, hi = (int(x) / 100.0 for x in f["cvt2bR"].split(","))
    return {"kR": int(f["cvt2k"]) / 100.0, "prior_mgs": int(f["cvt2r"]) / 1000.0, "on_s": int(f["cvt2Ds"]) / 10.0,
            "rate_mgs": end, "min_mgs": lo, "max_mgs": hi, "started": bool(flags & CVL_T2_ON),
            "cfg_bad": bool(flags & CVL_T2_CFG_BAD)}


def t2_text(t2):
    """'k0.50 r+0.124: on 41.2 s, bR +0.152 (-0.076..+0.350) mg/s' or 'off'; a refused stored setting is flagged."""
    if t2 is None:
        return "-"
    bad = " (stored setting REFUSED at boot)" if t2["cfg_bad"] else ""
    if t2["kR"] <= 0:
        return "off" + bad
    head = f"k{t2['kR']:.2f} r{t2['prior_mgs']:+.3f}"
    if not t2["started"]:
        return head + ": not started (shorter than its 30 s)" + bad
    return (f"{head}: on {t2['on_s']:.1f} s, bR {t2['rate_mgs']:+.2f} ({t2['min_mgs']:+.2f}..{t2['max_mgs']:+.2f}) "
            f"mg/s{bad}")


def t2_status_line(kv):
    """T2's line for 'camvz' / 'status' (t2k/t2r/t2Cfg/t2/t2OnS/t2bR; the plan's 't2=k0.50 r+0.124 on=Ns bR=..'),
    a warning when the stored setting was refused at boot; None for a firmware without T2."""
    if "t2k" not in kv:
        return None
    line = (f"offset drift rate T2: t2=k{float(kv['t2k']):.2f} r{float(kv['t2r']):+.3f} {kv.get('t2', '?')} "
            f"on={float(kv.get('t2OnS', 0)):.1f}s bR={float(kv.get('t2bR', 0)):+.3f} mg/s")
    if "t2bRmin" in kv:
        line += f" ({float(kv['t2bRmin']):+.3f}..{float(kv['t2bRmax']):+.3f})"
    if kv.get("t2Cfg") == "bad":
        line += ("  WARNING: the stored cvt2k / cvt2r was REFUSED at boot (out of range, unreadable, or with "
                 "cvqlate > 1): T2 is off")
    return line


# Autonomy ends with an in-air contact, so h at touchdown is no drift measurement: the KILLED trips (the
# touchdown values of the camera height observer are the ones before the first contact of the impact cluster,
# cam_vz_shadow.c, not a touchdown on the floor) and COLLISION. COLLISION is a LAND reason since the user's
# decision of 2026-10-13 (a KILLED one in 98cd4ae..eb9264d): the motors run on to the floor in a blind LAND, so
# the observer's floor values carry the contact's impulses. PALM (the palm catch, 2026-10-14, docs/autonomy-a2b.md
# 18.3): the drone usually ends on a hand in the air, so its h at touchdown is the palm's height, not a drift.
IN_AIR_ENDS = ("COLLISION", "PALM", "TILT", "RATE", "FREEFALL", "ATT_ERR")
# Since 2026-10-15 ("Claude decides", docs/autonomy-a2b.md 22) a contact, the camera height lost, NOT_STABLE or a
# stopped motion no longer ends a sortie: the drone holds and reports, and Claude's LAND ends it LAND_CMD. The 'F'
# line carries no events (its RTC record is fixed), so such a flight's h at touchdown still counts as a drift sample
# here; the daemon's 'drone_event' lines (the events log) tell those flights apart. A SINK (the camera height out
# 5 s, then the slow descent) is no drift measurement either: the observer flew the end without the camera.
NO_DRIFT_ENDS = ("SINK",)


def in_air_end(end):
    """True for 'AUTO(<reason>)' ends that are an in-air trip (IN_AIR_ENDS)."""
    m = re.fullmatch(r"AUTO\((\w+)\)", end or "")
    return bool(m) and m.group(1) in IN_AIR_ENDS


def flight_summary(f):
    """Derived numbers for one 'F' line of the flights command."""
    def i(k):
        return int(f[k])
    air, steady, hold = i("airCs"), i("steadyCs"), i("holdCs")
    motors = [int(x) for x in f["m"].split(",")]
    end = FLIGHT_ENDS.get(i("end"), "?")
    if i("end") == 3:
        end = "RESET: " + RESET_REASONS.get(i("rst"), str(i("rst")))
    elif i("end") == 4:     # autonomy flew it: why its thrust went to 0 (the LAND or KILLED trigger)
        end = f"AUTO({auto_name('REASON_NAMES', f.get('auto', f.get('autoReason')))})"
    camvz = camvz_summary(f)
    if camvz is not None and in_air_end(end):
        camvz["in_air_contact"] = True
        camvz["contact_land"] = end == "AUTO(COLLISION)"   # LAND COLLISION: the floor values after the contact
        camvz["drift_ms"] = None      # h at touchdown is the height at the contact, not a drift
    elif camvz is not None and re.fullmatch(r"AUTO\((%s)\)" % "|".join(NO_DRIFT_ENDS), end or ""):
        camvz["drift_ms"] = None      # the camera height was out at the end (SINK): not the estimate's drift
    return {
        "boot": i("boot"), "id": i("id"), "end": end,
        "dur_s": i("thrCs") / 100.0, "air_s": air / 100.0,
        "thrust_max": i("thrMax"), "vbat_start": i("vStart") / 1000.0, "vbat_min": i("vMin") / 1000.0,
        "cam_pct": 100.0 * i("camCs") / air if air else 0.0,
        "hold_pct": 100.0 * hold / air if air else 0.0,
        "coast_pct": 100.0 * int(f.get("coastCs", 0)) / air if air else 0.0,
        "gyro_drift": [int(x) / 100.0 for x in f.get("gDrift", "0,0").split(",")],
        "hold_roll": i("holdR") / hold / 100.0 if hold else 0.0,
        "hold_pitch": i("holdP") / hold / 100.0 if hold else 0.0,
        "tilt_rms": math.sqrt(i("tiltSq") / steady) / 10.0 if steady else None,
        "tilt_max": i("tiltMax") / 10.0 if steady else None,
        "gyro_rms": math.sqrt(i("gyroSq") / steady) if steady else None,
        "gyro_max": i("gyroMax") if steady else None,
        "motor_pct": [100.0 * m / air / 255.0 for m in motors] if air else None,
        "roll_rate_i": i("rI") / air if air else 0.0,
        "pitch_rate_i": i("pI") / air if air else 0.0,
        # Diagnostic-build fields (absent from older firmware):
        "up_s": opt_int(f, "upS"),
        "since_join_s": age_s(f.get("joinDs")),
        "since_app_s": age_s(f.get("appDs")),
        "rail_min": opt_mv(f, "railMin"),
        "vbat_fast_min": opt_mv(f, "vFastMin"),
        "tx": opt_int(f, "tx"),
        "rx": opt_int(f, "rx"),
        "gap_max_ms": None if "gapMaxUs" not in f else int(f["gapMaxUs"]) / 1000.0,
        "gaps_2500": opt_int(f, "gaps2500"),
        "pwm_khz": opt_int(f, "pwmKhz"),
        "soft": opt_int(f, "soft"),
        "camsb": opt_int(f, "camsb"),
        "txlo": opt_int(f, "txlo"),
        "wifiq": opt_int(f, "wifiq"),
        "sb_did": opt_int(f, "sbDid"),
        "tl_did": opt_int(f, "tlDid"),
        "cold": opt_int(f, "cold"),
        "atrim": None if "atrim" not in f else [int(x) / 100.0 for x in f["atrim"].split(",")],
        # The camera's ground standby and the IMU heat drift (docs/autonomy-a2b.md 17.1; absent before it):
        "imu_t0": opt_temp(f, "imuT0"),
        "imu_t1": opt_temp(f, "imuT1"),
        # The gyro temperature compensation and the learned bias at the end of the flight (19.3; absent before):
        "gtc_y_end": None if "gtcY" not in f else int(f["gtcY"]) / 1000.0,
        "mki_end": None if "mki" not in f else [int(x) / 1000.0 for x in f["mki"].split(",")],
        # The first ground re-zero after the flight: what the compensation left (19.1; absent before, or not yet):
        "gtc_res": gtc_flight_residual(f),
        # The accelerometer compensation's check at that re-zero (docs/autonomy-a2b.md 36.2; absent before, or not yet)
        # and the ground cooling spin's label (36.3):
        "atc_res": atc_flight_residual(f),
        "cool": opt_int(f, "cool"),
        "cam_awake_s": None if "camAwakeDs" not in f or int(f["camAwakeDs"]) < 0 else int(f["camAwakeDs"]) / 10.0,
        "cam_gnd_seen": "camAwakeDs" in f and int(f["camAwakeDs"]) >= 0,
        "camvz": camvz,
    }


def atc_flight_residual(f):
    """The flights line's accelerometer check (atcRf / atcS / atcC / atcRdT): the still z step over the flight, the
    model's correction and r = step - corr (mg), dTeff (C); None if absent or not come yet."""
    flags = int(f.get("atcRf", 0))
    if not flags & 1 or "atcS" not in f:
        return None
    step, corr = int(f["atcS"]) / 100.0, int(f.get("atcC", 0)) / 100.0
    return {"step": step, "corr": corr, "r": step - corr, "dteff": int(f.get("atcRdT", 0)) / 100.0,
            "on": bool(flags & 2)}


def gtc_flight_residual(f):
    """The flights line's post-flight re-zero (gtcRf / gtcR / gtcRdT / gtcRland): None if absent or not come yet."""
    flags = int(f.get("gtcRf", 0))
    if not flags & 1 or "gtcR" not in f:
        return None
    return {"r": [int(x) / 1000.0 for x in f["gtcR"].split(",")], "dteff": int(f.get("gtcRdT", 0)) / 100.0,
            "land_s": int(f.get("gtcRland", 0)) / 10.0, "on": bool(flags & 2)}


RAIL_SATURATED_MV = 65535   # the rail meter's "at or above this chip's full scale" (~2.85-3.2 V)
AGE_NONE = 0xFFFF


def opt_int(f, key):
    return int(f[key]) if key in f else None


def opt_temp(f, key):
    """deg C from a 'flights' temperature field; None if absent or not read (nan)."""
    v = f.get(key)
    if v is None:
        return None
    try:
        x = float(v)
    except ValueError:
        return None
    return None if math.isnan(x) else x


def opt_mv(f, key):
    """Volts from an mV field; None if absent or 0 (no reading)."""
    v = int(f.get(key, 0))
    return v / 1000.0 if v else None


def age_s(ds):
    """Seconds from a 0.1 s age field; None if absent or never stamped."""
    if ds is None or int(ds) == AGE_NONE:
        return None
    return int(ds) / 10.0


def rail_text(volts):
    if volts is None:
        return "-"
    return ">=FS" if volts * 1000 >= RAIL_SATURATED_MV - 1 else f"{volts:.2f}"


def print_flights(lines, end_line):
    rows = [flight_summary(line_kv(l)) for l in lines if l.startswith("F ")]
    trace = [line_kv(l) for l in lines if l.startswith("C ")]
    vwin = next((line_kv(l) for l in lines if l.startswith("V ")), None)
    print(f"{'boot':>4} {'id':>3} {'time':>5} {'air':>5} {'battery':>11} {'cam':>4} {'hold':>4} {'coast':>5} "
          f"{'tilt rms/max':>12} {'gyro rms/max':>12} {'motors FR/BR/BL/FL %':>21} {'hold p/r deg':>12} "
          f"{'trim p/r deg':>12} {'gyro drift':>11}  end")
    for r in rows:
        tilt = f"{r['tilt_rms']:.1f}/{r['tilt_max']:.1f}" if r["tilt_rms"] is not None else "-"
        gyro = f"{r['gyro_rms']:.0f}/{r['gyro_max']}" if r["gyro_rms"] is not None else "-"
        mot = "/".join(f"{m:.0f}" for m in r["motor_pct"]) if r["motor_pct"] else "-"
        hold = f"{r['hold_pitch']:+.1f}/{r['hold_roll']:+.1f}" if r["hold_pct"] else "-"
        drift = "{:+.2f},{:+.2f}".format(*r["gyro_drift"])
        trim = "-" if r.get("atrim") is None else f"{r['atrim'][1]:+.2f}/{r['atrim'][0]:+.2f}"
        print(f"{r['boot']:>4} {r['id']:>3} {r['dur_s']:>4.1f}s {r['air_s']:>4.1f}s "
              f"{r['vbat_start']:.2f}->{r['vbat_min']:.2f} {r['cam_pct']:>3.0f}% {r['hold_pct']:>3.0f}% "
              f"{r['coast_pct']:>4.0f}% {tilt:>12} {gyro:>12} {mot:>21} {hold:>12} {trim:>12} {drift:>11}  {r['end']}")
    for r in rows:
        note = flights_trim_note(r)
        if note:
            print(note)
    if any(r["up_s"] is not None for r in rows):
        def s(x):
            return "-" if x is None else f"{x:.1f}"
        print(f"\n{'boot':>4} {'id':>3} {'uptime':>7} {'since phone joined':>18} {'since app connected':>19} "
              f"{'battery min':>11} {'3.3V rail min':>13} {'tx':>5} {'rx':>5} {'loop pause':>10} {'motor drive':>23} "
              f"{'IMU C start->end':>16} {'cam awake':>9} {'gtc y':>6} {'learned x/y':>11} "
              f"{'post-flight r x/y @dTeff':>24} {'acc z step/r mg':>15}  end")
        for r in rows:
            if r["up_s"] is None:
                continue
            bat = "-" if r["vbat_fast_min"] is None else f"{r['vbat_fast_min']:.2f}"
            gap = "-" if r["gap_max_ms"] is None else f"{r['gap_max_ms']:.1f}ms/{r['gaps_2500']}"
            drive = "-" if not r["pwm_khz"] else (f"{r['pwm_khz']}kHz{'+soft' if r['soft'] else ''}"
                                                  f"{'+SB' if r.get('sb_did') else '+sb' if r.get('camsb') else ''}"
                                                  f"{'+TL' if r.get('tl_did') else '+tl' if r.get('txlo') else ''}"
                                                  f"{'+q' if r.get('wifiq') else ''}{' cold' if r.get('cold') else ''}")
            temp = "-" if r.get("imu_t0") is None else (f"{r['imu_t0']:.1f}->" + (
                "-" if r.get("imu_t1") is None else f"{r['imu_t1']:.1f}"))
            awake = "-" if r.get("cam_awake_s") is None else f"{r['cam_awake_s']:.1f}s"
            gtc = "-" if r.get("gtc_y_end") is None else f"{r['gtc_y_end']:+.2f}"
            mki = "-" if r.get("mki_end") is None else "{:+.2f}/{:+.2f}".format(*r["mki_end"])
            gr = r.get("gtc_res")
            res = "-" if gr is None else ("{:+.3f}/{:+.3f} @{:+.1f}C".format(gr["r"][0], gr["r"][1], gr["dteff"]) +
                                          ("" if gr["on"] else " off"))
            ar = r.get("atc_res")
            acc = "-" if ar is None else ("{:+.1f}/{:+.1f}".format(ar["step"], ar["r"]) + ("" if ar["on"] else " off"))
            if r.get("cool"):
                drive += " COOL"
            print(f"{r['boot']:>4} {r['id']:>3} {r['up_s']:>6}s {s(r['since_join_s']):>17}s "
                  f"{s(r['since_app_s']):>18}s {bat:>11} {rail_text(r['rail_min']):>13} {r['tx']:>5} {r['rx']:>5}"
                  f" {gap:>10} {drive:>23} {temp:>16} {awake:>9} {gtc:>6} {mki:>11} {res:>24} {acc:>15}  {r['end']}")
    cv_rows = [r for r in rows if r.get("camvz") and r["camvz"]["ran"]]
    if cv_rows:
        print_camvz(cv_rows)
    if trace:
        print(f"\nlast {len(trace) / 25:.1f} s before the reset ({end_line.split('traceReset=')[-1]}), 25 Hz:")
        print(f"{'t s':>6} {'thrust':>6} {'vbat':>5} {'rail':>6} {'motors FR/BR/BL/FL':>19} {'roll':>5} {'pitch':>5} "
              f"{'gyro':>4} {'tx':>3} {'rx':>3} flags")
        for c in trace:
            m = "/".join(f"{100 * int(x) / 255:.0f}" for x in c["m"].split(","))
            fl = int(c["f"])
            flags = " ".join(n for b, n in ((8, "air"), (1, "hold"), (2, "cam"), (4, "LOCK"), (16, "camSB"),
                                             (32, "txLow")) if fl & b)
            rail = rail_text(opt_mv(c, "rail"))
            print(f"{int(c['i']) / 25:>6.2f} {int(c['thr']):>6} {int(c['v']) / 1000:>5.2f} {rail:>6} {m:>19} "
                  f"{int(c['r']) / 2:>+5.1f} {int(c['p']) / 2:>+5.1f} {int(c['g']):>4} {c.get('tx', '-'):>3} "
                  f"{c.get('rx', '-'):>3} {flags}")
    if vwin:
        mv = [int(x) for x in vwin["mv"].split(",")]
        print(f"\nlowest battery voltage per 10 ms, last {len(mv) * 10} ms before the reset (1 kHz samples):")
        print("  " + " ".join(f"{v / 1000:.2f}" for v in mv))
        drop = max(mv[i] - mv[i + 1] for i in range(len(mv) - 1)) if len(mv) > 1 else 0
        print(f"  lowest {min(mv) / 1000:.2f} V, largest drop between windows {drop / 1000:.2f} V")
    rwin = next((line_kv(l) for l in lines if l.startswith("R ")), None)
    if rwin:
        mv = [int(x) for x in rwin["mv"].split(",")]
        seen = [v for v in mv if v]
        print(f"\nlowest XIAO 3.3 V rail reading per 10 ms, same windows (>=FS = at or above the meter's "
              f"full scale, ~2.85-3.2 V; see 'rail'):")
        print("  " + " ".join(rail_text(v / 1000 if v else None) for v in mv))
        if seen:
            low = min(seen)
            print(f"  lowest {rail_text(low / 1000)} V; windows below 2.70 V: {sum(1 for v in seen if v < 2700)}"
                  f", below 2.50 V: {sum(1 for v in seen if v < 2500)}")
    return rows


def flights_trim_note(r):
    """The flights table's trim check (whole-flight hold means and the trim, from the RTC summary; the first-10-s
    check needs the dump: 'trimcheck'). None when nothing is flagged."""
    if not r.get("hold_pct") or r.get("atrim") is None:
        return None
    parts = []
    for axis, m, t in (("roll", r["hold_roll"], r["atrim"][0]), ("pitch", r["hold_pitch"], r["atrim"][1])):
        if abs(m) > TRIM_HOLD_FLAG_DEG:
            parts.append(f"{axis} hold {m:+.1f} deg over the flight (> {TRIM_HOLD_FLAG_DEG}): atrim {axis} {t:+.2f} "
                         f"{'+' if m >= 0 else '-'} {TRIM_STEP_FRAC:g} x {abs(m):.1f} = "
                         f"{round_to(t + TRIM_STEP_FRAC * m):+.2f} as a first step ({round_to(t + m):+.2f} full)")
    if not parts:
        return None
    return (f"  trim check boot {r['boot']} flight {r['id']}: " + "; ".join(parts) +
            ". Suggested only (never set here): check the legs, props and battery seat, then 'trimcheck' on the dump")


def print_camvz(rows):
    """The camera height observer's table of the flights command."""
    def f(x, fmt):
        return "-" if x is None else format(x, fmt)
    print("\ncamera height observer (shadow mode): offsets in mg against the flight's ground baseline g0 "
          "(logged flights: +1.4 +/- 2.2);\nh at touchdown = the height estimate back on the floor (the drift); "
          "drift = h at touchdown / time with the camera (~ the camera's offset c)")
    print(f"{'boot':>4} {'id':>3} {'air':>5} {'lift':>6} {'g0-1g':>6} {'b@3s':>6} {'b@5s':>6} {'b@td':>6} {'sig5':>5} "
          f"{'G px/m':>6} {'ok':>4} {'1st ok':>6} {'gap':>5} {'h@td':>6} {'v@td':>6} {'h max':>5} {'drift':>7} "
          f"{'acc/rej/cusum/yaw':>17}  notes")
    for r in rows:
        c = r["camvz"]
        if not c["lifted"]:
            print(f"{r['boot']:>4} {r['id']:>3}  no lift-off")
            continue
        contact = c.get("in_air_contact", False)
        where = "at or after the in-air contact (COLLISION)" if c.get("contact_land") else \
            "before the first in-air contact"
        notes = " ".join(n for n, on in (("impact", c["impact"]), ("FAULT", c["fault"]), ("no-prior", not c["prior"]),
                                         ("no-touchdown", not c["touchdown"]), ("no-gain", not c["gain_tracked"]),
                                         (f"h@td/v@td {where}; drift n/a (in-air contact)", contact))
                         if on)
        if c.get("late") is not None:
            notes = (notes + " " if notes else "") + "late noise " + late_text(c["late"])
        if c.get("t2") is not None:
            notes = (notes + " " if notes else "") + "T2 " + t2_text(c["t2"])
        cnt = f"{c['accepted']}/{c['rejected']}/{c['alarms']}/{c['yaw_resets']}"
        drift = "n/a" if contact else f(c['drift_ms'], '+7.3f')
        print(f"{r['boot']:>4} {r['id']:>3} {c['air_s']:>4.1f}s {c['lift']:>6} {f(c['g0_mg'], '+6.1f'):>6} "
              f"{f(c['b3_g0_mg'], '+6.1f'):>6} {f(c['b5_g0_mg'], '+6.1f'):>6} {f(c['btd_g0_mg'], '+6.1f'):>6} "
              f"{f(c['sig5_mg'], '5.1f'):>5} {f(c['gain'], '6.0f'):>6} {f(c['ok_pct'], '3.0f') + '%' if c['ok_pct'] is not None else '-':>4} "
              f"{f(c['first_ok_s'], '5.1f') + 's' if c['first_ok_s'] is not None else '-':>6} {c['gap_s']:>4.1f}s "
              f"{f(c['h_td'], '+6.2f'):>6} {f(c['v_td'], '+6.2f'):>6} {f(c['h_max'], '5.2f'):>5} "
              f"{drift:>7} {cnt:>17}  {notes}")


def format_live(kv):
    """One compact line from an 'OK live' reply."""
    def v(x):
        x = int(x)
        return f"{x / 1000:.2f}" if x else "-"
    motors = "/".join(f"{100 * int(m) / 65535:.0f}" for m in kv["m"].split(","))
    rail = rail_text(int(kv["rail"]) / 1000 if int(kv["rail"]) else None)
    rail_min = rail_text(int(kv["railMin"]) / 1000 if int(kv["railMin"]) else None)
    join = int(kv["joinAgeMs"])
    app = int(kv["appAgeMs"])
    return (f"t={int(kv['up_ms']) / 1000:7.1f}s thrust={int(kv['thrust']):5d} (app {int(kv['raw']):5d}) "
            f"motors={motors:>11}% battery={v(kv['vbat'])} (min {v(kv['vbatMin'])}) "
            f"rail={rail} (min {rail_min}) tx={kv['tx']} rx={kv['rx']} "
            f"phone={'-' if join < 0 else f'{join / 1000:.0f}s'} app={'-' if app < 0 else f'{app / 1000:.0f}s'} "
            f"gcap={kv['gcap']}")


def restarted(last_up_ms, last_wall_s, up_ms, now_s):
    """True if the drone's uptime advanced clearly less than wall time did."""
    if last_up_ms is None:
        return False
    return up_ms - last_up_ms < (now_s - last_wall_s) * 1000.0 - 500.0


def cmd_watch(d, args):
    """Poll 'live' and print it; survive a drone restart (the port re-enumerates)."""
    import serial
    import termios
    dr = d
    port = args.port or d.ser.port   # reconnect to the same device, never a new guess
    period = 1.0 / max(1, min(args.hz, 20))
    t_end = time.time() + args.seconds if args.seconds else None
    last_up = last_wall = None
    worst_rail = worst_bat = None
    restarts = 0
    lost = (TimeoutError, serial.SerialException, OSError, termios.error)
    try:
        while t_end is None or time.time() < t_end:
            t0 = time.time()
            try:
                reply, _ = dr.command("live", timeout=1.0)
            except lost as e:
                print(f"-- no reply ({e.__class__.__name__}); drone restarting or unplugged, retrying")
                try:
                    dr.close()
                except Exception:
                    pass
                time.sleep(1.0)
                try:
                    dr = Drone(port)
                except lost:
                    pass
                continue
            if not reply.startswith("OK live"):
                print(reply)
                time.sleep(period)
                continue
            try:
                kv = parse_kv(reply)
                up = int(kv["up_ms"])
                r, b = int(kv["railMin"]), int(kv["vbatMin"])
                line = format_live(kv)
            except (KeyError, ValueError):
                print(f"-- garbled reply: {reply}")
                time.sleep(period)
                continue
            now = time.time()
            if restarted(last_up, last_wall, up, now):
                restarts += 1
                print("*** DRONE RESTARTED (run 'crash' and 'flights' to see why) ***")
            last_up, last_wall = up, now
            if r and (worst_rail is None or r < worst_rail):
                worst_rail = r
            if b and (worst_bat is None or b < worst_bat):
                worst_bat = b
            print(line)
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if dr is not d:
            dr.close()
    print(f"watch: lowest battery {'-' if worst_bat is None else f'{worst_bat / 1000:.2f}'} V, lowest rail "
          f"{rail_text(None if worst_rail is None else worst_rail / 1000)} V, restarts seen {restarts}")
    return 0


def cmd_simple(name):
    def fn(d, args):
        words = getattr(args, "args", None) or []
        reply, _ = d.command(" ".join([name] + list(words)))
        print(reply)
        return 1 if reply.startswith("ERR") else 0
    return fn


def cmd_camvz(d, args):
    reply, _ = d.command("camvz")
    print(reply)
    if not reply.startswith("OK camvz on=1"):
        return 1 if reply.startswith("ERR") else 0
    kv = parse_kv(reply)
    c = camvz_summary(kv)
    print(f"phase {kv.get('phase')}  h {float(kv['h']):+.3f} m  v {float(kv['v']):+.3f} m/s  offset "
          f"{float(kv['biasMg']):+.2f} mg vs 1 g (g0 {float(kv['g0Mg']):+.2f})  G {float(kv['gain']):.0f} px/m  "
          f"healthy {kv['healthy']}  cost: step max {kv['stepUsMax']} us avg {kv['stepUsAvg']} us, frame max "
          f"{kv['frameUsMax']} us avg {kv['frameUsAvg']} us; stabilizer stack free {kv.get('stabStackFree', '?')} B")
    if "qlate" in kv:
        # the late offset noise in use this boot (NVS cvqlate / cvqlates, read at boot), docs/autonomy-a2b.md 25
        bad = "  STORED SETTING REFUSED AT BOOT (x1 runs)" if kv.get("qlateCfg") == "bad" else ""
        print(f"late offset noise x{kv['qlate']} from {kv['qlateS']} s: {kv['late']}, on {int(kv['lateMs']) / 1000:.1f} s, "
              f"{kv['lateGuards']} guards this sortie; the controller's held offset {float(kv['biasHeldMg']):+.2f} mg{bad}")
    t2 = t2_status_line(kv)
    if t2:
        print(t2)
    if "hb" in kv:
        # the healthy offset speed-up in use this boot (NVS cvhb, read at boot), docs/autonomy-a2b.md 37
        bad = "  STORED SETTING REFUSED AT BOOT (off runs)" if kv.get("hbCfg") == "bad" else ""
        print(f"healthy offset speed-up x{kv['hb']} ({'off' if float(kv['hb']) == 0 else ('on now' if kv.get('hbOn') == '1' else 'not now')}), "
              f"on {float(kv['hbOnS']):.1f} s this sortie{bad}")
    if c and c["ran"]:
        print_camvz([{"boot": "-", "id": "now", "camvz": c}])
    return 0


# ---------------------------------------------------------------- autonomy (stage A2b)

AUTO_GET_CACHE = os.path.join(os.path.expanduser("~"), ".espfly", "auto_get.json")   # read by espfly_auto.py
# Bench go/no-go (docs/autonomy-bench.md step A4): only the stabilizer loop rate while autonomy is active gates
# (review RT-1). activeAvgUs/activeMaxUs are wall-clock times around autonomyArbitrate on every 16th active loop,
# preemption and interrupts included: information only. stabStackFree is a bench check of its own (sampled in
# IDLE every 5 s and on the first IDLE step after a sortie, never while active; -1 before the first sample).
# 940 (the plan's 975 lowered with the user's sign-off, 2026-10-12; any lower gate needs it again): with the
# per-loop autonomy code in IRAM the bench measured 948-973 loops/s in TAKEOFF/HOVER/LAND during DRY sorties with
# the Mac link, the phone link and photo pushes (idle 983-990); manual flights flew well at ~957 (2026-10-11).
AUTO_PROF_GO = ("activeLoopsPerS", ">=", 940)
# The loop-rate bisect toggles (bench_bypass.h, 'bypass' on the console; docs/autonomy-bench.md A4b): a reply's
# bypass=0x.. names them. With any on, the number prices a toggle, it is not the build's go/no-go. 'auto prof'
# also gives bypassSeen=0x..: every toggle on at any time since 'auto prof reset' (the mask its counters were
# measured under; bypass= is only the mask at the read, 0 again once 'bypass ... off' came before it).
BYPASS_BITS = (("gtc", 0x01), ("mki", 0x02), ("rec", 0x04), ("flow", 0x08))
AUTO_PROF_STACK = ("stabStackFree", ">=", 1024)
AUTO_PROF_INFO = ("activeAvgUs", "activeMaxUs")
AUTO_STF_SIM = 0x40


def auto_name(table, v):
    """Name of an autonomy enum value from tools/espfly_auto.py's tables (REASON_NAMES, STATE_NAMES,
    VZ_PHASE_NAMES, ...); a name is kept as it is, None gives '?'."""
    if v is None or v == "":
        return "?"
    try:
        n = int(v)
    except (TypeError, ValueError):
        return str(v)
    try:
        import espfly_auto  # same directory; standard library only at import
    except ImportError:
        return str(n)
    names = getattr(espfly_auto, table)
    return names[n] if 0 <= n < len(names) else str(n)


def auto_flag_names(bits):
    import espfly_auto
    return espfly_auto.bit_names(bits, espfly_auto.STF_NAMES)


def parse_auto_get(reply):
    """'OK auto get autoon=0(default) vmin=4000(nvs) ...' -> {key: {"value": "4000", "src": "nvs"}}."""
    out = {}
    for k, v in parse_kv(reply).items():
        m = re.fullmatch(r"(-?\d+)\((\w+)\)", v)
        out[k] = {"value": m.group(1), "src": m.group(2)} if m else {"value": v, "src": None}
    return out


def save_auto_get(reply, path=None):
    """Keeps the last 'auto get' for the Mac tool (its ARM reply shows vmin next to vbat)."""
    path = path or AUTO_GET_CACHE
    doc = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "reply": reply,
           "values": parse_auto_get(reply)}
    try:
        os.makedirs(os.path.dirname(path), mode=0o700, exist_ok=True)
        with open(path + ".tmp", "w") as f:
            json.dump(doc, f, indent=1)
        os.replace(path + ".tmp", path)
        return path
    except OSError as e:
        print(f"(could not save {path}: {e})")
        return None


def prof_check(kv, rule):
    """(key, value, op, limit, ok) for one 'auto prof' field, or None if the reply lacks it."""
    key, op, lim = rule
    if key not in kv:
        return None
    v = float(kv[key])
    return key, v, op, lim, v >= lim if op == ">=" else v <= lim


def bypass_names(v):
    """The bisect toggles a bypass=0x.. value names ('' for none or a missing / unreadable value)."""
    try:
        m = int(str(v), 0)
    except (TypeError, ValueError):
        return ""
    return " ".join(name for name, bit in BYPASS_BITS if m & bit)


def prof_bypass(kv):
    """(key, value, names) of the bisect toggles an 'auto prof' reading was measured under: bypassSeen (every
    toggle on since 'auto prof reset') when the firmware gives it, else the live bypass; names '' for none."""
    for key in ("bypassSeen", "bypass"):
        if key in kv:
            names = bypass_names(kv[key])
            if names or key == "bypass":
                return key, kv[key], names
    return None, None, ""


def print_prof_verdict(kv):
    """The go/no-go (loops/s only), the stack check, and the timings as information. Returns 1 if either
    check fails. With a bisect toggle on while the counters ran (bypassSeen=0x.., or bypass=0x.. from a firmware
    without it), the loop rate is printed for the bisect, no go/no-go."""
    go = prof_check(kv, AUTO_PROF_GO)
    if go is None:
        return 0
    bkey, bval, bisect_on = prof_bypass(kv)
    if bisect_on and go[1] != 0:
        print(f"  {go[0]:16s} {go[1]:8g} with bypass {bisect_on} (bisect toggles on)")
        now = ""
        if bkey == "bypassSeen" and not bypass_names(kv.get("bypass")):
            now = " (cleared before this read: still measured with them)"
        print(f"go/no-go: none: a bisect measurement ({bkey}={bval}: {bisect_on}{now}); 'bypass all off' "
              "(or a reboot), 'auto prof reset' and repeat A4 for the go/no-go (docs/autonomy-bench.md A4b)")
        return 0
    if go[1] == 0:
        print("go/no-go: no active loops measured yet ('auto prof reset' in IDLE, then a DRY sortie; read it after)")
        return 0
    key, v, op, lim, ok = go
    print(f"  {key:16s} {v:8g} {op} {lim:<6g} {'ok' if ok else 'FAIL'}")
    stack = prof_check(kv, AUTO_PROF_STACK)
    stack_bad = False
    if stack is not None:
        skey, sv, sop, slim, sok = stack
        if sv < 0:
            print(f"  {skey:16s} not sampled yet (every 5 s in IDLE)")
        else:
            stack_bad = not sok
            print(f"  {skey:16s} {sv:8g} {sop} {slim:<6g} {'ok' if sok else 'FAIL'} (its own check, not the go/no-go)")
    for k in AUTO_PROF_INFO:
        if k in kv:
            print(f"  {k:16s} {float(kv[k]):8g} info only (wall-clock, preemption included)")
    print("go/no-go: " + ("GO" if ok else f"NO-GO ({key} < {lim:g}): no further than DRY; "
                                           "see docs/autonomy-bench.md step A4"))
    return 1 if not ok or stack_bad else 0


def cmd_auto(d, args):
    """The USB 'auto' commands (usb_diag.c), printed as they come: status, get, set KEY V, reset KEY,
    on, off, rtc, bench sim|imu, prof [reset], unlock. 'get' is also saved for espfly_auto.py; 'prof'
    adds the bench go/no-go. 'unlock' (user decision 2026-10-12, evening) clears the reset-lock as a
    battery power-on does, over this USB cable only (espfly_auto.py, the Wi-Fi link, has no unlock);
    the drone refuses it unless IDLE with the motors stopped, and changes no other lock. A 'set' or 'reset'
    that the drone accepted reads 'auto get' again for the Mac tool's cache."""
    words = list(args.args)
    cmd = " ".join(["auto"] + words)
    sub = words[0] if words else ""
    if sub == "rtc":
        end, lines = d.command(cmd, until=("END", "ERR"), timeout=5.0)
        for line in lines:
            print(line)
        print(end)
        return 1 if end.startswith("ERR") else 0
    reply, _ = d.command(cmd, timeout=5.0)
    print(reply)
    if reply.startswith("ERR"):
        return 1
    if sub == "get":
        path = save_auto_get(reply)
        if path:
            print(f"(saved for espfly_auto.py: {path})")
    elif sub in ("set", "reset"):
        # the Mac tool checks a sortie's --wait-s 0 against the cached autowait (and ARM air_ms against autoair):
        # keep the cache what the drone now has
        got, _ = d.command("auto get", timeout=5.0)
        path = save_auto_get(got) if got.startswith("OK") else None
        if path:
            print(f"(auto get saved again for espfly_auto.py: {path})")
    elif sub == "prof" and words[1:2] != ["reset"]:
        return print_prof_verdict(parse_kv(reply))
    return 0


def auto_summary(fr):
    """Autonomy in one recorded flight (A2b columns): the state changes with their reasons and the
    vertical controller's highest height estimate; None if autonomy was not active."""
    rows = [r for r in fr if r.get("autoState") is not None]
    active = [r for r in rows if r["autoState"]]
    if not active:
        return None
    steps, last = [], None
    for r in rows:
        key = (r["autoState"], r["autoReason"])
        if key != last:
            steps.append({"t_ms": r["tickMs"], "state": auto_name("STATE_NAMES", r["autoState"]),
                          "reason": auto_name("REASON_NAMES", r["autoReason"]),
                          "vz": auto_name("VZ_PHASE_NAMES", r["vzPhase"])})
            last = key
    flags = 0
    for r in active:
        flags |= r["autoFlags"]
    return {"steps": steps, "h_max": max(r["vzH_mm"] for r in active) / 1000.0,
            "vz_thrust_max": max(r["vzThrust"] for r in active), "thrust_max": max(r["thrust"] for r in active),
            "flags": auto_flag_names(flags), "sim": bool(flags & AUTO_STF_SIM)}


def print_auto(a):
    t0 = a["steps"][0]["t_ms"]
    print("  autonomy: " + " -> ".join(f"{s['state']}({s['reason']}, vz {s['vz']}) +{(s['t_ms'] - t0) / 1000:.2f}s"
                                       for s in a["steps"]))
    print(f"  vertical controller: highest height estimate {a['h_max']:.2f} m, highest command {a['vz_thrust_max']}"
          f" (motors got up to {a['thrust_max']}){'; sim plant' if a['sim'] else ''}; flags seen: "
          f"{' '.join(a['flags']) or '-'}")


def cmd_regs(d, args):
    """Chip register snapshot (boot copy + now); saved raw for diffing between boots."""
    end, lines = d.command("regs", until=("END", "ERR"), timeout=10.0)
    lines = [l for l in lines if l[:2] in ("R ", "A ", "E ")]
    if end.startswith("ERR"):
        print(end)
        return 1
    text = "\n".join(lines) + "\n"
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
        print(f"regs: {len(lines)} lines saved to {args.out}")
    else:
        sys.stdout.write(text)
    head = next((l for l in lines if l.startswith("E ")), "")
    if head:
        print(head)
    return 0


def cmd_flights(d, args):
    if args.args and args.args[0] == "clear":
        reply, _ = d.command("flights clear")
        print(reply)
        return 1 if reply.startswith("ERR") else 0
    end, lines = d.command("flights", until=("END", "ERR"), timeout=5.0)
    if end.startswith("ERR"):
        print(end)
        return 1
    head = next((l for l in lines if l.startswith("OK flights")), "")
    print(head)
    print_flights(lines, end)
    return 0


# The post-flight check of the gyro temperature compensation (docs/autonomy-bench.md I5, autonomy-a2b.md 19.1).
GTC_PASS_R = (0.10, 0.15)      # |r| x / y below these: pass, dps
GTC_REFINE_R = (0.08, 0.10)    # |r| x / y above these, with |dTeff| >= GTC_REFINE_DT: refine k
GTC_REFINE_DT = 5.0            # C
GTC_SIGN_STEP = 0.10           # dps: a step and a model this large with opposite signs: the sign is wrong
GTC_K_MAX = 0.15               # 'gtc set' accepts |k| up to this


def gtc_check(a, ok):
    """From 'gtc show': its A line (the first ground re-zero after the latest flight) and its OK line (the
    config). r = step - corr is what the compensation left: with corr = k x dTeff a wrong k leaves
    r = (k_true - k) x dTeff, so k_true = k + r / dTeff = step / dTeff (the form used: it stays right if k was
    changed since the flight, or the correction hit its clamp). Returns None without a post-flight re-zero, else
    a dict: r, step, corr, dteff, on, pass, sign (axes whose model has the opposite sign to the measured step:
    'gtc off'), refine ('gtc set ...' or None)."""
    if a is None or int(a.get("seq", "0")) == 0 or "r" not in a:
        return None
    r, step, corr = floats(a["r"]), floats(a["step"]), floats(a["corr"])
    dt = float(a["dTeff"])
    k, knee = floats(ok["k"]), float(ok["knee"])
    sign = [ax for ax in (0, 1) if abs(step[ax]) > GTC_SIGN_STEP and abs(corr[ax]) > GTC_SIGN_STEP and
            step[ax] * corr[ax] < 0]
    new_k = list(k)
    for ax in (0, 1):
        if not sign and abs(dt) >= GTC_REFINE_DT and abs(r[ax]) > GTC_REFINE_R[ax]:
            new_k[ax] = step[ax] / dt
    changed = [round(x, 3) for x in new_k] != [round(x, 3) for x in k]
    in_range = all(abs(x) <= GTC_K_MAX for x in new_k)
    refine = None
    if changed and in_range:
        refine = f"gtc set {new_k[0]:.3f} {new_k[1]:.3f} {new_k[2]:.3f} {knee:.0f}"
    return {"r": r, "step": step, "corr": corr, "dteff": dt, "on": a.get("on") == "1",
            "pass": abs(r[0]) < GTC_PASS_R[0] and abs(r[1]) < GTC_PASS_R[1], "sign": sign, "refine": refine,
            "k": k, "new_k": new_k, "refine_out_of_range": changed and not in_range,
            "land_s": float(a.get("landS", "nan")), "air_s": float(a.get("airS", "nan")),
            "age_s": float(a.get("ageS", "nan"))}


def print_gtc_check(c):
    if c is None:
        print("post-flight check: no ground re-zero after a flight yet this boot (it comes ~2-3 s after the motors "
              "stop, once the drone rests)")
        return
    print(f"post-flight check: the first re-zero {c['land_s']:.1f} s after the flight's last airborne sample "
          f"({c['air_s']:.1f} s airborne, {c['age_s']:.0f} s ago): r x/y {c['r'][0]:+.3f}/{c['r'][1]:+.3f} dps at "
          f"dTeff {c['dteff']:+.2f} C, compensation {'on' if c['on'] else 'OFF'}")
    if c["sign"]:
        print("  SIGN: the model has the opposite sign to the measured step on " +
              "/".join("xy"[ax] for ax in c["sign"]) + ": 'gtc off', no long hovers, report")
        return
    print(f"  {'PASS' if c['pass'] else 'FAIL'} (|r_x| < {GTC_PASS_R[0]}, |r_y| < {GTC_PASS_R[1]} dps)")
    if c["refine"]:
        print(f"  refine: {c['refine']}, then gtc save")
    elif c["refine_out_of_range"]:
        print("  refine: the new k would be outside +-0.15 dps/C ({:+.3f}/{:+.3f}): report, do not set".format(
            *c["new_k"][:2]))
    elif abs(c["dteff"]) < GTC_REFINE_DT:
        print(f"  no refinement: |dTeff| < {GTC_REFINE_DT:.0f} C")


def cmd_gtc(d, args):
    """gtc [show|set KX KY KZ [KNEE]|on|off|save|clear|hold S]: every line of the reply (show prints its A and Z
    lines before the OK line), and for show the post-flight check."""
    words = list(args.args) or ["show"]
    reply, others = d.command("gtc " + " ".join(words))
    for o in others:
        print(o)
    print(reply)
    if reply.startswith("ERR"):
        return 1
    if words[0] == "show":
        a = next((line_kv(o) for o in others if o.startswith("A ")), None)
        kv = parse_kv(reply)
        if kv.get("bypass") == "1":
            print("note: the compensation is bypassed (bisect toggle 'bypass gtc on', RAM): nothing is subtracted, "
                  "no re-zero is logged; 'bypass gtc off' restarts it (0 until the next ground re-zero)")
        print_gtc_check(gtc_check(a, kv))
    return 0


ATC_PASS_R = 3.0        # |r_z| below this: pass, mg (0.2 mg for 30 s is 1 m of height, but the still reading itself
                        # scatters by 1-3 mg between the ground rows before and after a flight)
ATC_REFINE_DT = 5.0     # C: refine k_z only over a cooling this large
ATC_TILT_STEP = 20.0    # mg: an x / y step this large is a changed resting attitude (1.1 deg), not the zero
ATC_K_MAX = 2.0         # 'atc set' accepts |k| up to this, mg/C


def atc_check(a, ok):
    """From 'atc show': its A line (the first ground re-zero after the latest flight: the still accelerometer after
    the flight minus its reference before, against the model) and its OK line (the config). r = step - corr is
    what the compensation left on z; k_z measured = step_z / dTeff. Returns None without a post-flight entry, else
    a dict: step, corr, r, dteff, on, pass, tilt (the x / y step says the drone rests at another attitude: the z
    step is contaminated by (1 - cos) x g, flagged), k_meas, refine ('atc set ...' when |dTeff| >= 5 C and the
    measured k_z differs from the configured one by more than the still scatter allows)."""
    if a is None or int(a.get("seq", "0")) == 0 or "r" not in a:
        return None
    r, step, corr = floats(a["r"]), floats(a["step"]), floats(a["corr"])
    dt = float(a["dTeff"])
    k, knee = floats(ok["k"]), float(ok["knee"])
    tilt = abs(step[0]) > ATC_TILT_STEP or abs(step[1]) > ATC_TILT_STEP
    k_meas = step[2] / dt if abs(dt) >= 0.5 else None
    refine = None
    new_k = list(k)
    if k_meas is not None and abs(dt) >= ATC_REFINE_DT and not tilt and abs(r[2]) > ATC_PASS_R:
        new_k[2] = k_meas
        if abs(new_k[2]) <= ATC_K_MAX:
            refine = f"atc set {new_k[0]:.2f} {new_k[1]:.2f} {new_k[2]:.2f} {knee:.0f}"
    return {"step": step, "corr": corr, "r": r, "dteff": dt, "on": a.get("on") == "1", "pass": abs(r[2]) < ATC_PASS_R,
            "tilt": tilt, "k_meas": k_meas, "k": k, "new_k": new_k, "refine": refine,
            "land_s": float(a.get("landS", "nan")), "age_s": float(a.get("ageS", "nan"))}


def print_atc_check(c):
    if c is None:
        print("post-flight check: no ground re-zero after a flight yet this boot (it comes ~2-3 s after the motors "
              "stop, once the drone rests; the reference needs 1 s of rest before the ARM)")
        return
    print(f"post-flight check: the first re-zero {c['land_s']:.1f} s after the flight's last airborne sample "
          f"({c['age_s']:.0f} s ago): still z step {c['step'][2]:+.2f} mg, model {c['corr'][2]:+.2f}, r_z "
          f"{c['r'][2]:+.2f} mg at dTeff {c['dteff']:+.2f} C, compensation {'on' if c['on'] else 'OFF'}")
    if c["tilt"]:
        print(f"  TILT: the x / y step {c['step'][0]:+.1f}/{c['step'][1]:+.1f} mg says the drone rests at another "
              f"attitude (> {ATC_TILT_STEP:.0f} mg = 1.1 deg): the z step is not the zero's drift; no refinement")
    print(f"  {'PASS' if c['pass'] else 'FAIL'} (|r_z| < {ATC_PASS_R:.0f} mg)")
    if c["k_meas"] is not None:
        print(f"  k_z measured this flight: {c['k_meas']:+.3f} mg/C (configured {c['k'][2]:+.3f}; average several "
              f"flights before setting it)")
    if c["refine"]:
        print(f"  refine: {c['refine']}, then atc save (and 'atc on' if it is off)")
    elif not c["tilt"] and abs(c["dteff"]) < ATC_REFINE_DT:
        print(f"  no refinement: |dTeff| < {ATC_REFINE_DT:.0f} C")


def cmd_atc(d, args):
    """atc [show|set KX KY KZ [KNEE]|on|off|save|clear|latch]: every line of the reply (show prints its A and Z lines
    before the OK line), and for show the post-flight check."""
    words = list(args.args) or ["show"]
    reply, others = d.command("atc " + " ".join(words))
    for o in others:
        print(o)
    print(reply)
    if reply.startswith("ERR"):
        return 1
    if words[0] == "show":
        a = next((line_kv(o) for o in others if o.startswith("A ")), None)
        kv = parse_kv(reply)
        if kv.get("bypass") == "1":
            print("note: the compensation is bypassed ('bypass gtc on' covers it, RAM): nothing is subtracted, no "
                  "entry is logged")
        print_atc_check(atc_check(a, kv))
    return 0


COOL_POLL_S = 2.0        # the state line while the spin runs
COOL_REZERO_WAIT_S = 4.0   # after the spin: the driver's first ground re-zero comes >= 2 s after the motors stop


def cool_summary(start, end, entry):
    """One line from the 'OK cool start' and the final 'OK cool' key-values and the 'atc show' Z line with cool=1
    (None if it has not come): the temperatures, the still accelerometer step over the spin and the k it says."""
    t0, t1 = float(start.get("T0", "nan")), float(end.get("T", "nan"))
    s = f"IMU {t0:.1f} -> {t1:.1f} C ({t1 - t0:+.1f}) in {float(end.get('elapsedS', 'nan')):.1f} s, end {end.get('state')}"
    if entry is not None:
        step, dt = floats(entry["step"]), float(entry["dTeff"])
        s += f"; still accelerometer step x/y/z {step[0]:+.2f}/{step[1]:+.2f}/{step[2]:+.2f} mg over dTeff {dt:+.2f} C"
        if abs(dt) >= 1.0:
            s += f" -> k_z {step[2] / dt:+.3f} mg/C (k_x {step[0] / dt:+.3f}, k_y {step[1] / dt:+.3f})"
        else:
            s += " (dTeff below 1 C: no k)"
    else:
        s += "; no re-zero entry yet (the drone must rest still for ~2 s after the spin: 'atc show' later)"
    return s


def cmd_cool(d, args):
    """cool [S [PWM]] | cool stop | cool: starts the ground cooling spin (ground_cool.h; USB only), prints its state
    line every 2 s until it ends, then the IMU temperature before / after and the accelerometer compensation's entry
    for the spin ('atc show', cool=1): the still z step over the cooling and the k_z it implies."""
    words = list(args.args)
    if not words or words[0] == "stop":
        reply, _ = d.command("cool" + (" stop" if words else ""))
        print(reply)
        return 1 if reply.startswith("ERR") else 0
    reply, _ = d.command("cool " + " ".join(words))
    print(reply)
    if reply.startswith("ERR"):
        return 1
    start = parse_kv(reply)
    deadline = time.monotonic() + float(words[0]) + 10.0
    end = start
    while time.monotonic() < deadline:
        time.sleep(COOL_POLL_S)
        reply, _ = d.command("cool")
        print(reply)
        if reply.startswith("ERR"):
            return 1
        end = parse_kv(reply)
        if end.get("state") != "run":
            break
    if end.get("state") == "run":
        print("the spin is still running past its time: 'cool stop'")
        return 1
    time.sleep(COOL_REZERO_WAIT_S)
    reply, others = d.command("atc show")
    entry = next((line_kv(o) for o in reversed(others) if o.startswith("Z ") and line_kv(o).get("cool") == "1"), None)
    print(cool_summary(start, end, entry))
    if end.get("state") != "DONE":
        print(f"the spin ended {end.get('state')}, not DONE: check the drone rests level and still, then try again")
        return 1
    return 0


def cmd_cam(d, args):
    reply, _ = d.command("cam " + " ".join(args.args) if args.args else "cam")
    print(reply)
    return 1 if reply.startswith("ERR") else 0


def cmd_hold(d, args):
    reply, _ = d.command("hold " + " ".join(args.args) if args.args else "hold")
    print(reply)
    return 1 if reply.startswith("ERR") else 0


def cmd_atrim(d, args):
    reply, _ = d.command("atrim " + " ".join(args.args) if args.args else "atrim show")
    print(reply)
    return 1 if reply.startswith("ERR") else 0


def cmd_facc(d, args):
    reply, _ = d.command("facc " + " ".join(args.args) if args.args else "facc show")
    print(reply)
    return 1 if reply.startswith("ERR") else 0


def cmd_yawlock(d, args):
    reply, _ = d.command("yawlock " + " ".join(args.args) if args.args else "yawlock")
    print(reply)
    return 1 if reply.startswith("ERR") else 0


def cmd_trim(d, args):
    cmd = "trim " + " ".join(args.args) if args.args else "trim show"
    reply, _ = d.command(cmd)
    print(reply)
    return 1 if reply.startswith("ERR") else 0


def cmd_trim_learn(d, args):
    flights = []
    for path in args.csv:
        with open(path) as f:
            rows = read_rows(f)
        for fid, n, roll, pitch in steady_torque(rows):
            flights.append((path, fid, n, roll, pitch))
            print(f"  {path} flight {fid}: {n} steady samples, roll {roll:+.0f}, pitch {pitch:+.0f}")
    if len(flights) < 2:
        print("Need at least two flights with >= 0.5 s of steady hands-off hover.")
        return 1
    rolls = sorted(f[3] for f in flights)
    pitches = sorted(f[4] for f in flights)
    med_r, med_p = statistics.median(rolls), statistics.median(pitches)
    trim_r, trim_p = round(med_r * args.fraction), round(med_p * args.fraction)
    print(f"median steady correction: roll {med_r:+.0f}, pitch {med_p:+.0f} "
          f"(range roll {rolls[0]:+.0f}..{rolls[-1]:+.0f}, pitch {pitches[0]:+.0f}..{pitches[-1]:+.0f})")
    print(f"suggested trim ({args.fraction:.0%} of median, integrators handle the rest): "
          f"roll {trim_r:+d}, pitch {trim_p:+d}")
    if not args.apply:
        print("(dry run: add --apply to set it live, and --save to persist it)")
        return 0
    reply, _ = d.command(f"trim set {trim_r} {trim_p}")
    print(reply)
    if reply.startswith("ERR"):
        return 1
    if args.save:
        reply, _ = d.command("trim save")
        print(reply)
        return 1 if reply.startswith("ERR") else 0
    return 0


def parse_cam_steps(text):
    """'awake:300,sleep:600,awake:600' -> [("awake", 300.0), ("sleep", 600.0), ("awake", 600.0)]."""
    steps = []
    for part in (text or "").split(","):
        if not part.strip():
            continue
        kind, _, sec = part.strip().partition(":")
        if kind not in ("awake", "sleep") or not sec:
            raise ValueError(f"--cam-step: '{part}' is not awake:SECONDS or sleep:SECONDS")
        steps.append((kind, float(sec)))
    return steps


def cam_step_at(steps, t):
    """The step index and kind in force at t s after the start (the last one after the schedule)."""
    acc = 0.0
    for i, (kind, sec) in enumerate(steps):
        acc += sec
        if t < acc:
            return i, kind
    return (len(steps) - 1, steps[-1][0]) if steps else (None, None)


def acc_log_fields(m, status_reply):
    """The --log-acc fields of one drift window: the accelerometer means (mg) from 'measure', the IMU
    temperature and the camera state from 'status'."""
    kv = parse_kv(status_reply or "")
    t = kv.get("imuTempC")
    return {"ax_mg": m["acc"][0] * 1000.0, "ay_mg": m["acc"][1] * 1000.0, "az_mg": m["acc"][2] * 1000.0,
            "imu_t": float(t) if t not in (None, "nan") else None, "cam": kv.get("cam"),
            "cam_gnd": kv.get("camGnd")}


def exp_fit(ts, ys):
    """y = a + b * exp(-t / tau) by a grid over tau (20 s..2 h) and least squares for a and b; returns
    (tau_s, a, b, rms) or None with fewer than 4 points."""
    if len(ts) < 4:
        return None
    t0 = ts[0]
    best = None
    tau = 20.0
    while tau <= 7200.0:
        x = [math.exp(-(t - t0) / tau) for t in ts]
        n = len(x)
        mx, my = sum(x) / n, sum(ys) / n
        sxx = sum((xi - mx) ** 2 for xi in x)
        if sxx > 1e-12:
            b = sum((xi - mx) * (yi - my) for xi, yi in zip(x, ys)) / sxx
            a = my - b * mx
            rms = math.sqrt(sum((a + b * xi - yi) ** 2 for xi, yi in zip(x, ys)) / n)
            if best is None or rms < best[3]:
                best = (tau, a, b, rms)
        tau *= 1.05
    return best


def cam_step_report(rows, steps, leads=(3.0, 20.0, 60.0), flight_s=60.0, limit_mg=1.0):
    """The camera step test (docs/autonomy-a2b.md 17.1): per step the change of the IMU temperature and of
    acc z (start -> end, mean of 2 windows each) and the temperature's time constant; the temperature
    coefficient (acc z against the IMU temperature, all rows); and the in-flight drift predicted for a wake
    L s before the take-off, dT_wake x tempco x (exp(-L/tau) - exp(-(L + flight)/tau)), with dT_wake the
    awake steps' fitted equilibrium change. rows: dicts with t_s, imu_t, az_mg (and step). Returns a dict."""
    per = []
    for i, (kind, _) in enumerate(steps):
        rs = [r for r in rows if r.get("step") == i and r.get("imu_t") is not None]
        if len(rs) < 2:
            per.append({"step": i, "kind": kind, "n": len(rs)})
            continue
        k = min(2, len(rs) // 2)
        t_a = sum(r["imu_t"] for r in rs[:k]) / k
        t_b = sum(r["imu_t"] for r in rs[-k:]) / k
        z_a = sum(r["az_mg"] for r in rs[:k]) / k
        z_b = sum(r["az_mg"] for r in rs[-k:]) / k
        fit = exp_fit([r["t_s"] for r in rs], [r["imu_t"] for r in rs])
        per.append({"step": i, "kind": kind, "n": len(rs), "dT": t_b - t_a, "dAz_mg": z_b - z_a,
                    "tau_s": fit[0] if fit else None, "dT_eq": (fit[1] - rs[0]["imu_t"]) if fit else None})
    pts = [(r["imu_t"], r["az_mg"]) for r in rows if r.get("imu_t") is not None]
    tempco = None
    if len(pts) >= 3:
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        sxx = sum((p[0] - mx) ** 2 for p in pts)
        if sxx > 1e-9:
            tempco = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx
    wakes = [p for i, p in enumerate(per) if p["kind"] == "awake" and i > 0 and p.get("tau_s")]
    tau = sum(p["tau_s"] for p in wakes) / len(wakes) if wakes else None
    d_t = sum(p["dT_eq"] for p in wakes) / len(wakes) if wakes else None
    pred = {}
    lead = None
    if tau and d_t is not None and tempco is not None:
        def drift(L):
            return abs(d_t * tempco * (math.exp(-L / tau) - math.exp(-(L + flight_s) / tau)))
        pred = {L: drift(L) for L in leads}
        if drift(leads[0]) > limit_mg:
            lead = next((L for L in range(1, 3601) if drift(float(L)) <= limit_mg), None)
    return {"steps": per, "tempco_mg_per_c": tempco, "tau_s": tau, "dT_wake_c": d_t, "predicted_mg": pred,
            "camlead_ms": None if lead is None else int(lead * 1000)}


def print_cam_step_report(rep):
    print("\ncamera step test (docs/autonomy-a2b.md 17.1): per step, start -> end")
    print(f"{'step':>4} {'camera':>6} {'n':>3} {'dIMU C':>7} {'d acc z mg':>10} {'tau s':>6}")
    for p in rep["steps"]:
        if "dT" not in p:
            print(f"{p['step']:>4} {p['kind']:>6} {p['n']:>3}  (too few windows)")
            continue
        tau = "-" if p["tau_s"] is None else f"{p['tau_s']:.0f}"
        print(f"{p['step']:>4} {p['kind']:>6} {p['n']:>3} {p['dT']:>+7.2f} {p['dAz_mg']:>+10.2f} {tau:>6}")
    tc, dt, tau = rep["tempco_mg_per_c"], rep["dT_wake_c"], rep["tau_s"]
    tc_text = "-" if tc is None else f"{tc:+.3f} mg/C"
    dt_text = "-" if dt is None else f"{dt:+.2f} C"
    tau_text = "-" if tau is None else f"{tau:.0f} s"
    print(f"temperature coefficient of acc z: {tc_text}; wake step {dt_text}, tau {tau_text}")
    for L, mg in sorted(rep["predicted_mg"].items()):
        print(f"  predicted in-flight drift for a wake {L:.0f} s before the take-off (60 s flight): {mg:.2f} mg")
    if rep["camlead_ms"] is not None:
        print(f"  over 1 mg at the shortest lead: set 'tkoff camlead {rep['camlead_ms']}' (<= 1 mg); above "
              "60000 keep the sleep to the soak only, or 'tkoff camgnd off'")
    elif rep["predicted_mg"]:
        print("  <= 1 mg at the shortest lead: camlead 0 is fine")


def cmd_drift(d, args):
    reply, _ = d.command("status")
    up_ms = int(parse_kv(reply).get("up_ms", "0"))
    print(f"drone uptime at start: {up_ms / 1000:.0f} s (gyro bias was calibrated shortly after boot)")
    steps = parse_cam_steps(getattr(args, "cam_step", None))
    log_acc = getattr(args, "log_acc", False) or bool(steps)
    rows = []
    acc_rows = []
    t_end = time.time() + args.minutes * 60
    t0 = time.time()
    w = None
    f = open(args.out, "w", newline="") if args.out else None
    if f:
        w = csv.writer(f)
        w.writerow(["t_s", "uptime_s", "gx_dps", "gy_dps", "gz_dps", "estRoll", "estPitch",
                    "accRoll", "accPitch", "still"] +
                   (["ax_mg", "ay_mg", "az_mg", "imuTempC", "cam", "camGnd", "step", "camStep"] if log_acc else []))
    print(f"{'t(s)':>6} {'gx':>7} {'gy':>7} {'gz':>7}  {'estR':>6} {'estP':>6}  {'accR':>6} {'accP':>6}  "
          f"{'errR':>6} {'errP':>6}" + (f"  {'az mg':>8} {'IMU C':>6} {'cam':>8}" if log_acc else ""))
    step_now = None
    try:
        while time.time() < t_end:
            if steps:
                i, kind = cam_step_at(steps, time.time() - t0)
                if i != step_now:
                    step_now = i
                    left = sum(sec for _, sec in steps[i:]) + 60
                    cmd = f"tkoff camwake {int(min(3600, left))}" if kind == "awake" else "tkoff camwake 0"
                    r, _ = d.command(cmd)
                    print(f"-- step {i}: camera {kind} ({cmd}: {r[:60]})")
            m = measure(d, args.window_ms)
            t = m["time"] - t0
            row = [t, up_ms / 1000 + t, *m["gyro"], m["estRoll"], m["estPitch"], m["accRoll"], m["accPitch"],
                   m["still"]]
            extra = ""
            if log_acc:
                st, _ = d.command("status")
                a = acc_log_fields(m, st)
                a.update({"t_s": t, "step": step_now, "kind": None if step_now is None else steps[step_now][0]})
                acc_rows.append(a)
                row += [a["ax_mg"], a["ay_mg"], a["az_mg"], a["imu_t"], a["cam"], a["cam_gnd"], step_now, a["kind"]]
                extra = (f"  {a['az_mg']:8.2f} {'-' if a['imu_t'] is None else format(a['imu_t'], '6.2f'):>6} "
                         f"{(a['cam'] or '-'):>8}")
            rows.append(row)
            if w:
                w.writerow([f"{v:.4f}" if isinstance(v, float) else v for v in row])
                f.flush()
            print(f"{t:6.0f} {m['gyro'][0]:+7.3f} {m['gyro'][1]:+7.3f} {m['gyro'][2]:+7.3f}  "
                  f"{m['estRoll']:+6.2f} {m['estPitch']:+6.2f}  {m['accRoll']:+6.2f} {m['accPitch']:+6.2f}  "
                  f"{m['estRoll'] - m['accRoll']:+6.2f} {m['estPitch'] - m['accPitch']:+6.2f}"
                  f"{'' if m['still'] else '  (moving)'}{extra}")
            time.sleep(max(0.0, args.interval - args.window_ms / 1000.0))
    finally:
        if steps:
            d.command("tkoff camwake 0")
        if f:
            f.close()
    if steps:
        print_cam_step_report(cam_step_report(acc_rows, steps))
    still = [r for r in rows if r[9]]     # the still flag (the --log-acc columns follow it)
    if len(still) >= 2:
        first, last = still[0], still[-1]
        print("\nSummary (still windows only):")
        for i, ax in enumerate("xyz"):
            print(f"  gyro {ax}: {first[2 + i]:+.3f} -> {last[2 + i]:+.3f} dps "
                  f"(residual bias change {last[2 + i] - first[2 + i]:+.3f})")
        err_r = [r[5] - r[7] for r in still]
        err_p = [r[6] - r[8] for r in still]
        print(f"  estimator minus accel-implied attitude: roll {min(err_r):+.2f}..{max(err_r):+.2f}, "
              f"pitch {min(err_p):+.2f}..{max(err_p):+.2f} deg")
        print("  (a residual gyro bias b shifts the estimate by about b/0.4 rad, i.e. 0.1 dps -> 0.25 deg)")


def cmd_stream(d, args):
    reply, _ = d.command(f"stream {args.hz}")
    print(reply)
    t_end = time.time() + args.seconds
    f = open(args.out, "w", newline="") if args.out else None
    w = csv.writer(f) if f else None
    if w:
        w.writerow(RECORD_FIELDS)
    n = bad = 0
    try:
        while time.time() < t_end:
            line = d.readline(t_end)
            if not line or not line.startswith("S,"):
                continue
            r = parse_record(line)
            if r is None:
                bad += 1
                continue
            n += 1
            if w:
                w.writerow([r[k] for k in RECORD_FIELDS])
            if not args.quiet:
                print(f"thr={r['thrust']:5d} roll={r['roll_cdeg'] / 100:+6.2f} pitch={r['pitch_cdeg'] / 100:+6.2f} "
                      f"yaw={r['yaw_cdeg'] / 100:+7.2f} gyro=({r['gx_ddps'] / 10:+6.1f},{r['gy_ddps'] / 10:+6.1f},"
                      f"{r['gz_ddps'] / 10:+6.1f}) acc=({r['ax_mg'] / 1000:+.3f},{r['ay_mg'] / 1000:+.3f},"
                      f"{r['az_mg'] / 1000:+.3f}) m=({r['m1']},{r['m2']},{r['m3']},{r['m4']})"
                      f" | cam seq={r['flowSeq']:3d} ok={r['flowOk']} lum={r['flowLum']:3d} tex={r['flowTexture']:3d}"
                      f" dx={r['flowDx_cpx'] / 100:+5.2f} dy={r['flowDy_cpx'] / 100:+5.2f}")
    finally:
        d.ser.write(b"stream 0\n")
        if f:
            f.close()
    print(f"{n} records, {bad} corrupted")


def cmd_dump(d, args):
    info, _ = d.command("rec info")
    capacity = int(parse_kv(info).get("cap", "0")) or None
    d.drain()
    d.ser.write(b"rec dump\n")
    # A full ring (6000 records) takes longer than a fixed deadline over USB:
    # wait 30 s for the first line, then 15 s after each line received.
    deadline = time.time() + 30
    header = None
    rows, bad = [], 0
    while True:
        line = d.readline(deadline)
        if line is None:
            sys.exit("timed out waiting for the recorder dump")
        deadline = time.time() + 15
        if line.startswith("ERR"):
            sys.exit(line)
        if line.startswith("OK rec dump"):
            header = line
            continue
        if line.startswith("END rec"):
            end = line
            break
        if header and line.startswith("R,"):
            r = parse_record(line)
            if r is None:
                bad += 1
            else:
                rows.append(r)
    print(header.split(" fields=")[0])
    print(end, f"received={len(rows)} corrupted={bad}")
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(RECORD_FIELDS)
        for r in rows:
            w.writerow([r[k] for k in RECORD_FIELDS])
    print(f"saved {args.out}")
    if not args.keep and bad == 0:
        # Start the next round with an empty ring so flights never mix.
        reply, _ = d.command("rec clear")
        print(reply)
    analyze(rows, capacity)


def cmd_analyze(_d, args):
    with open(args.csv) as f:
        rows = read_rows(f)
    analyze(rows)


# ---------------------------------------------------------------- analysis

def mean(v):
    return sum(v) / len(v) if v else float("nan")


def flight_end_cause(fr):
    """Why thrust went to zero at the end of a flight (needs the link fields)."""
    idx = max(i for i, r in enumerate(fr) if r["thrust"] > 0)
    last = fr[idx]
    after = fr[idx + 1] if idx + 1 < len(fr) else None
    closing = after if after is not None and after.get("autoState") else last
    if closing.get("autoState"):
        return (f"AUTONOMY: {auto_name('STATE_NAMES', closing['autoState'])} "
                f"({auto_name('REASON_NAMES', closing['autoReason'])})")
    if "rawThrust" not in last:
        return "unknown (recorded before link logging)"
    if last["thrust"] < 20000:
        return "throttle lowered by the pilot (landing)"
    if after is None:
        return "recording ended while thrust was applied"
    if after["linkAge_ms"] > 1900:
        return f"LINK LOST: no packets for {after['linkAge_ms']} ms (2 s failsafe cut)"
    if after["linkFlags"] & 1 and after["rawThrust"] > 0:
        return f"THRUST LOCK after a link gap: app still sent {after['rawThrust']} (packet age {after['linkAge_ms']} ms)"
    if after["rawThrust"] == 0:
        return f"APP SENT ZERO THRUST suddenly (from {last['thrust']}; packet age {after['linkAge_ms']} ms)"
    return f"other (raw {after['rawThrust']}, locked {after['linkFlags'] & 1}, age {after['linkAge_ms']} ms)"


def analyze(rows, capacity=None):
    if not rows:
        print("no records (the recorder only stores data while thrust is applied or autonomy is active)")
        return
    flights = {}
    for r in rows:
        flights.setdefault(r["flightId"], []).append(r)
    for fid, fr in flights.items():
        thr = [r for r in fr if r["thrust"] > 0]
        auto = auto_summary(fr)
        if not thr:
            if auto:
                print(f"\n=== flight {fid}: autonomy with no thrust (DRY) ===")
                print_auto(auto)
            continue
        t0 = thr[0]["tickMs"]
        dur = (thr[-1]["tickMs"] - t0) / 1000
        maybe_truncated = capacity is not None and fr[0] is rows[0] and len(rows) >= capacity
        print(f"\n=== flight {fid}: {dur:.1f} s with thrust"
              f"{' (start may have been overwritten in the ring)' if maybe_truncated else ''} ===")
        angle = all(r["flags"] & FLAG_ANGLE for r in thr)
        print(f"  roll/pitch mode: {'angle (self-level)' if angle else 'NOT angle mode for some samples'}")
        rd = [r["rollDes_cdeg"] / 100 for r in thr]
        pd = [r["pitchDes_cdeg"] / 100 for r in thr]
        print(f"  commanded roll  {min(rd):+.2f}..{max(rd):+.2f} deg (mean {mean(rd):+.2f})")
        print(f"  commanded pitch {min(pd):+.2f}..{max(pd):+.2f} deg (mean {mean(pd):+.2f})")
        if max(abs(x) for x in rd + pd) > 0.5:
            print("  NOTE: nonzero roll/pitch commands -> not a pure-throttle flight (stick, tilt control or trim)")
        max_thr = max(r["thrust"] for r in thr)
        t_ramp = next((r["tickMs"] for r in thr if r["thrust"] >= 0.9 * max_thr), t0)
        print(f"  thrust: max {max_thr}, reached 90% of max {(t_ramp - t0) / 1000:.2f} s after first thrust")

        windows = [(0.0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 1e9)]
        print("  window      roll err  pitch err (deg, est-cmd mean/max|.|)   rateI roll/pitch   attI roll/pitch   motors m1..m4 mean")
        for a, b in windows:
            seg = [r for r in thr if a <= (r["tickMs"] - t0) / 1000 < b]
            if not seg:
                continue
            er = [(r["roll_cdeg"] - r["rollDes_cdeg"]) / 100 for r in seg]
            ep = [(r["pitch_cdeg"] - r["pitchDes_cdeg"]) / 100 for r in seg]
            label = f"{a:.1f}-{'end' if b > 1e8 else f'{b:.1f}'}s"
            print(f"  {label:10s} {mean(er):+6.2f}/{max(abs(x) for x in er):5.2f}  {mean(ep):+6.2f}/{max(abs(x) for x in ep):5.2f}"
                  f"        {mean([r['rollRateI'] for r in seg]):+7.0f}/{mean([r['pitchRateI'] for r in seg]):+7.0f}"
                  f"      {mean([r['rollI_cdps'] / 100 for r in seg]):+5.1f}/{mean([r['pitchI_cdps'] / 100 for r in seg]):+5.1f}"
                  f"     {mean([r['m1'] for r in seg]):6.0f} {mean([r['m2'] for r in seg]):6.0f}"
                  f" {mean([r['m3'] for r in seg]):6.0f} {mean([r['m4'] for r in seg]):6.0f}")
        sat = sum(1 for r in thr if max(r["m1"], r["m2"], r["m3"], r["m4"]) >= 65000)
        if sat:
            print(f"  NOTE: a motor was at full output in {sat} of {len(thr)} samples (saturation)")
        yaw_rate = mean([r["gz_ddps"] / 10 for r in thr])
        print(f"  mean yaw rate {yaw_rate:+.1f} dps")
        hs = hold_summary(thr)
        if hs:
            print(f"  camera hold: applied {hs['applied_pct']:.0f} %, coasting {hs['coasting_pct']:.0f} % of the samples;"
                  f" HOVER scene gate refused {hs['scene_gated']} of {hs['frames']} frames"
                  f"{' (a moving object in the view)' if hs['scene_gated'] else ''}")
        th = thermal_summary(thr)
        if th:
            print_thermal(th)
        if auto:
            print_auto(auto)
            print_trim_check(fid, trim_check(fr))
        print(f"  ended: {flight_end_cause(fr)}")


# The post-flight trim check (2026-10-16, docs/autonomy-a2b.md 27.5): on 10-16 the drone needed ~1.4 deg more right
# lean than on 10-15 (a re-seated leg; the hold sat at its 1.5 deg limit and the drone crept left). Information only:
# it suggests an atrim value with its arithmetic and never sets one ('atrim set' / 'atrim save' are the user's).
TRIM_EARLY_S = 10.0           # the first seconds of HOVER (outside any motion, the hold applied)
TRIM_HOLD_FLAG_DEG = 0.8      # |mean hold correction| above this on an axis: flagged
TRIM_AY_FLAG_MG = 15.0        # |mean sideways accel - normal| above this: flagged
TRIM_AY_NORMAL_MG = -12.0     # the drone's normal hover ay: the median of the first 10 s of the good 10-14 / 10-15
                              # hovers (-7.4..-16.5 mg; 10-16 F1 +9.6, F2 +7.0, the A/B flight -7.4)
TRIM_MG_PER_DEG = 17.45       # 1000 sin(1 deg): a steady lean of 1 deg reads ~17 mg sideways
TRIM_STEP_FRAC = 0.5          # the first step: half the mean (the need moved ~1.5 deg between days and spots)
REC_LOOPS_PER_S = 955.0       # the recorder's tickMs counts stabilizer loops (~945-960 a second)


def round_to(x, q=0.05):
    return round(round(x / q) * q, 2)


def trim_check(rows, trim=None, ay_normal=TRIM_AY_NORMAL_MG, early_s=TRIM_EARLY_S):
    """The trim check of one flight's recorder rows (an autonomy sortie): the mean camera-hold correction and
    sideways accelerometer reading over the first early_s of HOVER outside motions with the hold applied; flags and
    the suggested atrim (trim = (roll, pitch) now, None: unknown). None without a HOVER."""
    hov = [r for r in rows if r.get("autoState") == 3]
    if not hov or "holdRoll_cdeg" not in hov[0]:
        return None
    t0 = hov[0]["tickMs"]
    sel = [r for r in hov if (r["tickMs"] - t0) / REC_LOOPS_PER_S < early_s and (r.get("mvState", 0) & 15) not in
           (1, 2, 3) and r.get("holdFlags", 0) & HOLD_APPLIED]
    if len(sel) < 100:      # ~1 s
        return {"n": len(sel), "flags": [], "note": "under 1 s of steady HOVER with the hold applied: no check"}
    hr = mean([r["holdRoll_cdeg"] / 100.0 for r in sel])
    hp = mean([r["holdPitch_cdeg"] / 100.0 for r in sel])
    ay = mean([r["ay_mg"] for r in sel])
    ax = mean([r["ax_mg"] for r in sel])
    out = {"n": len(sel), "seconds": round((sel[-1]["tickMs"] - sel[0]["tickMs"]) / REC_LOOPS_PER_S, 1),
           "hold_roll": round(hr, 2), "hold_pitch": round(hp, 2), "ay_mg": round(ay, 1), "ax_mg": round(ax, 1),
           "ay_normal_mg": ay_normal, "ay_off_mg": round(ay - ay_normal, 1),
           "ay_lean_deg": round((ay - ay_normal) / TRIM_MG_PER_DEG, 2), "trim": trim, "flags": [], "suggest": {}}
    for axis, m, k in (("roll", hr, 0), ("pitch", hp, 1)):
        if abs(m) > TRIM_HOLD_FLAG_DEG:
            out["flags"].append(f"{axis} hold {m:+.2f} deg (> {TRIM_HOLD_FLAG_DEG} deg)")
    if abs(ay - ay_normal) > TRIM_AY_FLAG_MG:
        out["flags"].append(f"sideways accel {ay:+.1f} mg, {ay - ay_normal:+.1f} from normal {ay_normal:+.1f} "
                            f"(> {TRIM_AY_FLAG_MG:g} mg; ~{(ay - ay_normal) / TRIM_MG_PER_DEG:+.2f} deg of lean)")
    if out["flags"]:
        for axis, m, k in (("roll", hr, 0), ("pitch", hp, 1)):
            if abs(m) <= TRIM_HOLD_FLAG_DEG and not (axis == "roll" and abs(ay - ay_normal) > TRIM_AY_FLAG_MG):
                continue
            if trim is None:
                out["suggest"][axis] = {"full": f"atrim {axis} now {m:+.2f}",
                                        "step": f"atrim {axis} now {TRIM_STEP_FRAC * m:+.2f}"}
            else:
                t = trim[k]
                out["suggest"][axis] = {
                    "now": t, "full": round_to(t + m), "step": round_to(t + TRIM_STEP_FRAC * m),
                    "arithmetic": f"{t:+.2f} {'+' if m >= 0 else '-'} {abs(m):.2f} = {t + m:+.2f} (full); first "
                                  f"step {t:+.2f} {'+' if m >= 0 else '-'} {TRIM_STEP_FRAC:g} x {abs(m):.2f} = "
                                  f"{t + TRIM_STEP_FRAC * m:+.2f} -> {round_to(t + TRIM_STEP_FRAC * m):+.2f}"}
    return out


def print_trim_check(fid, c):
    if c is None:
        return
    if not c.get("flags") and c.get("note"):
        print(f"  trim check (flight {fid}): {c['note']}")
        return
    if "hold_roll" not in c:
        return
    head = (f"  trim check (flight {fid}, the first {c['seconds']:.1f} s of HOVER, {c['n']} samples): hold roll "
            f"{c['hold_roll']:+.2f} pitch {c['hold_pitch']:+.2f} deg, ay {c['ay_mg']:+.1f} mg ({c['ay_off_mg']:+.1f} "
            f"from normal), ax {c['ax_mg']:+.1f} mg")
    print(head)
    if not c["flags"]:
        print("    OK: nothing to change")
        return
    for f in c["flags"]:
        print(f"    FLAG: {f}")
    for axis, sg in c["suggest"].items():
        if "arithmetic" in sg:
            print(f"    suggested atrim {axis}: {sg['arithmetic']}")
        else:
            print(f"    suggested atrim {axis}: {sg['full']} (full), {sg['step']} (first step); the trim now: "
                  "'atrim show' (or --trim)")
    t = c.get("trim")
    if t is not None and c["suggest"]:
        r = c["suggest"].get("roll", {}).get("step", t[0])
        p = c["suggest"].get("pitch", {}).get("step", t[1])
        if isinstance(r, float) and isinstance(p, float):
            print(f"    to try (RAM only, a 20-30 s hover, never set by this tool): espfly_diag.py atrim set {r:.2f} "
                  f"{p:.2f}; 'atrim save' only after a better hover; check the legs, props and battery seat first")
    else:
        print("    never set by this tool: 'atrim set ROLL PITCH' (RAM) after a check of the legs, props and battery "
              "seat; 'atrim save' only after a better hover")


def flights_txt_trims(path):
    """{flight id: (roll, pitch)} from a saved 'flights' output (its first table, 'trim p/r deg' = pitch/roll; the
    last boot's row for an id seen twice)."""
    trims = {}
    try:
        with open(path) as f:
            for line in f:
                tok = line.split()
                if len(tok) >= 14 and tok[0].isdigit() and tok[1].isdigit():
                    m = re.fullmatch(r"([+-]\d+\.\d+)/([+-]\d+\.\d+)", tok[12])
                    if m:
                        trims[int(tok[1])] = (float(m.group(2)), float(m.group(1)))
    except OSError:
        return {}
    return trims


def cmd_trimcheck(_d, args):
    """The post-flight trim check on a saved recorder CSV (no drone needed)."""
    with open(args.csv) as f:
        rows = read_rows(f)
    trims = flights_txt_trims(args.flights) if args.flights else {}
    flights = {}
    for r in rows:
        flights.setdefault(r["flightId"], []).append(r)
    results = {}
    for fid, fr in flights.items():
        if args.flight is not None and fid != args.flight:
            continue
        trim = tuple(args.trim) if args.trim else trims.get(fid)
        c = trim_check(fr, trim=trim, ay_normal=args.ay_normal, early_s=args.early_s)
        if c is None:
            continue
        results[fid] = c
        if not args.json:
            print_trim_check(fid, c)
    if args.json:
        print(json.dumps(results, indent=1, default=str))
    return 0


def thermal_summary(rows):
    """The IMU temperature, the gyro temperature compensation and the learned bias at the end of the flight, the
    flow fit's exposure steps (docs/autonomy-a2b.md 19.3); None for a dump without those columns."""
    rows = [r for r in rows if r.get("imuTemp_cC") is not None]
    if not rows:
        return None
    air = [r for r in rows if r["thrust"] > 30000] or rows
    temps = [r["imuTemp_cC"] / 100 for r in air]
    last = air[-1]
    frames, prev = [], None
    for r in air:
        if r["flowSeq"] and r["flowSeq"] != prev:
            frames.append(r)
        prev = r["flowSeq"]
    gated = sum(1 for r in frames if r.get("flowExpGate"))
    steps = sum(1 for r in frames if r.get("flowGain_e3"))
    big = sum(1 for r in frames if abs(r.get("flowGain_e3") or 0) > 30)
    return {
        "t_min": min(temps), "t_max": max(temps), "t_start": temps[0], "t_end": temps[-1],
        "gtc_end": (last["gtcCorrX_mdps"] / 1000, last["gtcCorrY_mdps"] / 1000),
        "mki_end": (last["mkiBiasX_mdps"] / 1000, last["mkiBiasY_mdps"] / 1000),
        "frames": len(frames),
        "gated_pct": 100.0 * gated / len(frames) if frames else 0.0,
        "steps_pct": 100.0 * steps / len(frames) if frames else 0.0,
        "big_steps_pct": 100.0 * big / len(frames) if frames else 0.0,
        "sat_max": max((r.get("flowSat_pct") or 0) for r in frames) if frames else 0,
        "sat_mean": mean([r.get("flowSat_pct") or 0 for r in frames]) if frames else 0.0,
    }


def print_thermal(th):
    print(f"  IMU {th['t_start']:.1f} -> {th['t_end']:.1f} C (min {th['t_min']:.1f}, max {th['t_max']:.1f});"
          f" at the end: gyro tempco x/y {th['gtc_end'][0]:+.3f}/{th['gtc_end'][1]:+.3f} dps,"
          f" learned bias x/y {th['mki_end'][0]:+.3f}/{th['mki_end'][1]:+.3f} dps")
    if th["frames"]:
        print(f"  camera exposure: {th['steps_pct']:.1f} % of {th['frames']} frames corrected for a step"
              f" ({th['big_steps_pct']:.1f} % over 3 %), {th['gated_pct']:.1f} % refused (step > 8 %);"
              f" saturated mean {th['sat_mean']:.0f} %, max {th['sat_max']} %")


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    p = sub.add_parser("raw", help="send one console command")
    p.add_argument("text", nargs="+")
    p.add_argument("--timeout", type=float, default=5.0)
    p = sub.add_parser("measure", help="stationary IMU statistics (motors held at zero)")
    p.add_argument("--ms", type=int, default=5000)
    p.add_argument("--out")
    p = sub.add_parser("level-cal", help="compute (and optionally apply/save) the level offset")
    p.add_argument("a", help="measure JSON at heading A")
    p.add_argument("b", nargs="?", help="measure JSON at heading A+180 deg (recommended)")
    p.add_argument("--apply", action="store_true")
    p.add_argument("--save", action="store_true")
    p = sub.add_parser("cal", help="cal show | set X Y | save | clear")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("cam", help="cam [on|off]: start the camera at boot (default off; needs reboot)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("hold", help="hold [on|off]: camera drift hold (only with sticks centred)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("holdk", help="holdk [show|set K|save|clear]: the camera hold's largest gain scale with the "
                       "scene distance (docs/autonomy-a2b.md 20; default 2.5, 1 = the hold before it; 'set' is live "
                       "until 'save'; show: the live scale, its target, the latched G)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("atrim", help="atrim show | set ROLL_DEG PITCH_DEG | save | clear | learn on|off (level-attitude trim; learned from the camera hold when on)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("facc", help="facc show | set X_G Y_G | save | clear (accelerometer offset applied only in flight)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("flights", help="flights [clear]: per-flight summaries and the pre-reset trace (survive brownouts)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("watch", help="live battery / 3.3 V rail / motors / Wi-Fi view (Ctrl-C to stop)")
    p.add_argument("--hz", type=int, default=5)
    p.add_argument("--seconds", type=float, default=0, help="0 = until Ctrl-C")
    p = sub.add_parser("gcap", help="gcap [THRUST|off]: ground-test thrust cap, saved on the drone; only lowers thrust")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("reboot", help="reboot [sleep|rwdt|swd]: restart the drone (software reset, or deep sleep / "
                       "RTC watchdog / super watchdog); motors must be stopped")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("tkoff", help="tkoff [camsb|camgnd|txlo|wifiq on|off] | tkoff camms|camcold|camgndms|camlead MS | "
                       "tkoff camwake S: take-off load shedding (camera standby and its warm/cold length, low Wi-Fi "
                       "TX power, quiet Wi-Fi at next boot) and the camera's ground standby (switch, hold after a "
                       "wake, thermal lead; camwake holds it awake S s, 0 ends the hold)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("regs", help="chip register snapshot at boot and now (for diffing boot types)")
    p.add_argument("--out", help="save the raw lines to this file")
    sub.add_parser("rail", help="bench check of the GPIO9 3.3 V rail meter")
    sub.add_parser("net", help="phone join / app timing, UDP counters, packets per CRTP port")
    sub.add_parser("camvz", help="camera height observer (shadow mode): live estimate, counters, the current "
                   "flight's summary fields, stabilizer-task cost")
    p = sub.add_parser("auto", help="auto [status|get|set KEY V|reset KEY|on|off|rtc|bench sim|imu|prof [reset]|"
                       "unlock]: autonomy (A2b) over USB; 'get' is saved for espfly_auto.py, 'prof' gives the bench "
                       "go/no-go, 'unlock' clears the reset-lock after a flash or reboot (IDLE, motors stopped; "
                       "other locks unchanged)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("pwm", help="pwm [HZ|stock]: motor PWM frequency (stock 15000), saved; motors stopped")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("soft", help="soft [on|off]: take-off thrust ramp after a ground stop, saved; motors stopped")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("plock", help="plock [on|off]: no thrust after a non-power-on reset until the battery is replugged")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("crash", help="crash [clear|test]: last panic record and recent warnings")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("yawlock", help="yawlock [on|off]: ignore the yaw stick and hold heading")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("trim", help="trim show | set ROLL PITCH | save | clear")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("gtc", help="gtc [show|set KX KY KZ [KNEE]|on|off|save|clear|hold S]: gyro temperature "
                       "compensation; show prints the logged re-zeros and the post-flight check (docs/autonomy-bench.md "
                       "I5): the first re-zero after the latest flight, its residual and the refinement")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("atc", help="atc [show|set KX KY KZ [KNEE]|on|off|save|clear|latch]: accelerometer zero "
                       "temperature compensation (mg per C; docs/autonomy-a2b.md 36); show prints the logged entries "
                       "and the post-flight check: the still accelerometer after the latest flight against its "
                       "reference before it, the measured k_z")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("cool", help="cool [S [PWM]] | cool stop | cool: ground prop-wash cooling spin before a take-off "
                       "(docs/autonomy-a2b.md 36.3): all four motors at PWM (default 12000, well below lift-off) for S s "
                       "(default 20, max 40), IDLE and resting only; prints the IMU temperature before / after and the "
                       "accelerometer step over the cooling")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("bypass", help="bypass [gtc|mki|rec|flow|all on|off]: loop-rate bisect toggles (RAM only, 0 at "
                       "boot; motors stopped): each puts one per-loop addition of a2b section 19 back to 14bcfc6's code "
                       "(docs/autonomy-bench.md A4b)")
    p.add_argument("args", nargs="*")
    p = sub.add_parser("trim-learn", help="derive the torque trim from recorded hover (dump CSVs)")
    p.add_argument("csv", nargs="+")
    p.add_argument("--fraction", type=float, default=0.7,
                   help="fraction of the median steady correction to feed forward (default 0.7)")
    p.add_argument("--apply", action="store_true")
    p.add_argument("--save", action="store_true")
    p = sub.add_parser("flow-cal", help="fit camera image motion against gyro rotation (hand-held)")
    p.add_argument("--seconds", type=float, default=25)
    p.add_argument("--out", help="save the recorded stream CSV")
    p.add_argument("--csv", help="fit a previously saved stream/dump CSV instead of recording")
    p.add_argument("--save", help="save the fitted calibration JSON")
    p.add_argument("--offset-scan", action="store_true", help="scan gyro alignment relative to the recorded timestamps")
    p.add_argument("--timestamp-origin", choices=("native-centre", "vga-centre", "legacy-start"), default="native-centre",
                   help="Native: scan residual -40..40 ms around published timestamps. Legacy: scan start offset 0..40 ms")
    p.add_argument("--capture-offset-ms", type=int, default=37, help="QVGA start-to-centre shift used when recording (37 ms in qvga32); metadata only")
    p = sub.add_parser("drift", help="repeat measurements to see warm-up gyro drift (and with --log-acc / --cam-step "
                       "the accelerometer's heat drift against the camera's ground standby, docs/autonomy-a2b.md 17.1)")
    p.add_argument("--minutes", type=float, default=5)
    p.add_argument("--interval", type=float, default=15)
    p.add_argument("--window-ms", type=int, default=5000)
    p.add_argument("--out")
    p.add_argument("--log-acc", action="store_true", help="also log the accelerometer means, the IMU temperature "
                   "and the camera state of every window")
    p.add_argument("--cam-step", help="camera schedule, e.g. awake:300,sleep:600,awake:600 (seconds; 'tkoff "
                   "camwake'), then a report of the steps, the time constant and the predicted in-flight drift")
    p = sub.add_parser("stream", help="live telemetry")
    p.add_argument("--hz", type=int, default=20)
    p.add_argument("--seconds", type=float, default=10)
    p.add_argument("--out")
    p.add_argument("--quiet", action="store_true")
    p = sub.add_parser("dump", help="download and analyze the RAM flight recorder (then clear it)")
    p.add_argument("--out", default="flight.csv")
    p.add_argument("--keep", action="store_true", help="do not clear the recorder after a clean dump")
    p = sub.add_parser("analyze", help="analyze a saved flight CSV")
    p.add_argument("csv")
    p = sub.add_parser("trimcheck", help="post-flight trim check on a saved recorder CSV (no drone): the first 10 s of "
                                         "HOVER's mean camera-hold correction (> 0.8 deg flagged) and sideways accel "
                                         "(> 15 mg from normal flagged), with a suggested atrim and its arithmetic; "
                                         "never sets anything")
    p.add_argument("csv")
    p.add_argument("--flight", type=int, help="only this flight id")
    p.add_argument("--trim", type=float, nargs=2, metavar=("ROLL", "PITCH"), help="the attitude trim the flight had "
                                                                                  "(deg; 'atrim show')")
    p.add_argument("--flights", help="a saved 'flights' output: the trim of each flight id from it")
    p.add_argument("--ay-normal", type=float, default=TRIM_AY_NORMAL_MG, help="the drone's normal hover ay, mg")
    p.add_argument("--early-s", type=float, default=TRIM_EARLY_S, help="the seconds of HOVER checked")
    p.add_argument("--json", action="store_true")
    p = sub.add_parser("colourcheck", help="the colour bench checks on two saved photos of the same still scene (no "
                                           "drone): a grey one (photocol 0) and a colour one (photocol 1): Y identity "
                                           "(mean difference, min / max, a bright window >= 250), U / V and R / G / B "
                                           "means; docs/autonomy-bench.md N'1.4-N'1.5")
    p.add_argument("grey", help="the grey photo (.json / .png / _x4.png)")
    p.add_argument("colour", help="the colour photo (.json / .png / _x4.png / .yuyv)")
    p.add_argument("--white", action="store_true", help="the view is a white sheet: U and V 124-132, R/G/B within 10")
    p.add_argument("--orange", action="store_true", help="the view is an orange object: R > G > B")
    p.add_argument("--json", action="store_true")
    args = ap.parse_args()

    if args.cmd == "analyze":
        return cmd_analyze(None, args)
    if args.cmd == "trimcheck":
        return cmd_trimcheck(None, args)
    if args.cmd == "colourcheck":
        return cmd_colourcheck(None, args)
    if args.cmd in ("level-cal", "trim-learn") and not args.apply:
        return {"level-cal": cmd_level_cal, "trim-learn": cmd_trim_learn}[args.cmd](None, args)
    if args.cmd == "flow-cal" and args.csv:
        return cmd_flow_cal(None, args)

    d = Drone(args.port or find_port())
    try:
        fn = {"status": cmd_status, "raw": cmd_raw, "measure": cmd_measure, "level-cal": cmd_level_cal,
              "cal": cmd_cal, "trim": cmd_trim, "yawlock": cmd_yawlock, "crash": cmd_crash, "flights": cmd_flights, "watch": cmd_watch,
              "gcap": cmd_simple("gcap"), "reboot": cmd_simple("reboot"), "rail": cmd_simple("rail"),
              "net": cmd_simple("net"), "camvz": cmd_camvz, "auto": cmd_auto, "regs": cmd_regs, "tkoff": cmd_simple("tkoff"), "pwm": cmd_simple("pwm"), "soft": cmd_simple("soft"), "plock": cmd_simple("plock"), "atrim": cmd_atrim, "facc": cmd_facc, "gtc": cmd_gtc, "atc": cmd_atc, "cool": cmd_cool, "bypass": cmd_simple("bypass"), "hold": cmd_hold, "holdk": cmd_simple("holdk"), "cam": cmd_cam, "trim-learn": cmd_trim_learn, "drift": cmd_drift,
              "stream": cmd_stream, "dump": cmd_dump, "flow-cal": cmd_flow_cal}[args.cmd]
        return fn(d, args)
    finally:
        d.close()


if __name__ == "__main__":
    sys.exit(main() or 0)
