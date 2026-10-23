#!/usr/bin/env python3
"""
ESP-FLY autonomy link, Mac side (docs/autonomy-link.md, docs/autonomy-bench.md).

Stage A1: status and photos (HELLO, STATUS_REQ, PHOTO, CHUNKS, PHOTO_FREE).
Stage A2b adds the flight commands (ARM, TAKEOFF, LAND, KILL, DISARM) and
AUTO_STATUS. They go through one long-running process, the daemon: it holds
the session, sends the heartbeat every 100 ms from its single select() loop
(the drone lands when it stops for 1 s), collects the photos the drone pushes
during the hover wait, and serves the client commands over a Unix socket
(~/.espfly/auto.sock). The daemon never says HELLO again during a sortie (a new
session lands the drone), and the one-shot hello/status/photo go through it
while it runs. 'k' in the daemon's terminal, or the 'kill' command (its own
socket, works with a dead daemon), cuts the motors at once; 'l' or 'land' (and
the phone stick) land slowly. ARM in FLIGHT mode needs 'daemon --allow-flight'
and 'arm --confirm-flight'; DRY and BENCH need neither. A sortie whose client
goes away (Ctrl-C) or times out before the take-off is stopped (DISARM); once
it took off the drone holds (no LAND for it since 2026-10-15: Claude decides);
a client that only shuts down its write side after the request (nc -N) is not
gone and gets its reply. The daemon's terminal closed (SIGHUP) quits like 'q'
(LAND first); under nohup SIGHUP stays ignored.

The Mac joins the drone's Wi-Fi AP (drone 192.168.43.42) on one interface and
stays online through another; packets go out on the interface that holds the
192.168.43.x address (IP_BOUND_IF), whatever the default route says.

Every command prints exactly one JSON line on stdout (messages go to stderr)
and exits non-zero on failure; the daemon prints one JSON event per line.

Examples (run from Firmware/esp-drone):
  python3 tools/espfly_auto.py keygen                  # ~/.espfly/auto.key (0600)
  python3 tools/espfly_auto.py install-key             # over USB: 'autokey <hex>'
  python3 tools/espfly_auto.py up                      # drone AP joined and routed?
  python3 tools/espfly_auto.py hello
  python3 tools/espfly_auto.py status
  python3 tools/espfly_auto.py photo --n 3 --gap 300   # PNG + JSON in ~/.espfly/photos (grey by default; a
                                                       # drone with USB 'auto set photocol 1' + reboot sends colour:
                                                       # RGB PNGs, JSON 'color' true; docs/autonomy-a2b.md 24)
  python3 tools/espfly_auto.py daemon                  # own terminal: k KILL, l LAND, s status, q quit
  python3 tools/espfly_auto.py sortie --mode dry --height 40   # 1 take-off photo (--photos 0..3)
  python3 tools/espfly_auto.py photo --n 1 --gap 150   # in the hover wait: through the daemon
  python3 tools/espfly_auto.py land
  python3 tools/espfly_auto.py kill

The interactive hover (docs/autonomy-a2b.md sections 14, 18 and 22): take off to up to 1.20 m, one photo, then the
drone holds for Claude; higher, up to 2.00 m (a2b 23.9, 2026-10-15), only by 'move --up' from the hover (a LAND in the
take-off climb falls at once, a LAND from the hover is slow). Since 2026-10-15 ("Claude decides") nothing lands it but the
battery limit, a crash or flip (the cut), a lost Mac link and the k kill: the wait (--wait-s) is information, there
is no air clock, and what the drone does on its own (a contact, the camera height lost, a motion stopped) it reports:
'drone_event' lines from the daemon, and 'hold' in every reply (the latest three events, the SINK countdown, a
contact's obstacle bearing); the long-wait height rules are advice in the take-off's 'note'.

The height (docs/autonomy-a2b.md 23, 2026-10-15): the estimate drifts 0.01-0.03 m/s with no height sensor. Every
saved photo's _x4.png carries the level line (the row of the true horizontal: what lies on it is at camera height;
dashed, dotted when its tilt setting is stale) and its JSON the drone's estimate when it was captured
(autonomy.h_est_capture_m). Claude judges the true height from what lies on the line and says it with set-height; the
drone corrects its estimate to it and flies to the commanded height (2026-10-16, a2b 27: "you are at X, go to the
target"; until then it flew back to the old estimate and ended ~0.1 m low). The mount tilt is one setting,
re-measured every flight on the ground with level-cal.
  python3 tools/espfly_auto.py photo                   # on the ground, facing a wall 2-6 m away
  python3 tools/espfly_auto.py level-cal --floor-row 240 --dist 3.5   # the far floor-wall line's x4 row
  python3 tools/espfly_auto.py set-height 1.10         # "you are at 1.10 m" (when the latest photo was taken): to the
                                                       # commanded height (true_now_m, target_m, fly_m in the reply)
  python3 tools/espfly_auto.py set-height 0.95 --now   # "you are at 0.95 m now"
  python3 tools/espfly_auto.py set-height 1.10 --to 1.30   # ... and go to 1.30 m (the new commanded height)
  python3 tools/espfly_auto.py daemon --allow-flight
  python3 tools/espfly_auto.py sortie --mode flight --confirm-flight --height 100 --photos 1 --then wait --wait-s 60
  python3 tools/espfly_auto.py move --right 0.5        # also --left/--back (<= 1.5), --forward (<= 4.0; 1.5 before
                                                       # the long-move build), --up/--down (<= 0.5); sideways <=
                                                       # 0.5 x D; forward ends >= 0.35 m at k 0.6 (the drone's
                                                       # NVS mvfloor + 0.05 m; 0.60 m until 2026-10-17)
  python3 tools/espfly_auto.py yaw --deg 90            # + right (clockwise), as the photo bearings
  python3 tools/espfly_auto.py stop                    # ends a running MOVE / YAW: holds where it is
  python3 tools/espfly_auto.py state                   # D_m, yawOff_deg, hTarget_m, mv, hold (events)

Long moves and a move planned from take-off (2026-10-14, docs/autonomy-a2b.md 21): one MOVE per leg toward a person,
Claude judges the distance to them from the photo (the Mac's approach planner was removed on 2026-10-15: a YAW to the
bearing, then a plain MOVE); the drone checks what its camera measures (the scene ahead, the
view, the battery, the air time). Forward up to 4.00 m with the long-move build (1.50 m before it), sideways and back
1.50 m, the vector 4.00 m; a refusal gives the largest forward move the scene allows now. Toward a person,
--person-gap with --est: the Mac also refuses a move whose travel at an image expansion scale k down to 0.6 could end
under 1.0 m from the person (the live D), so a planned move toward a person goes --plan-via mac.
  python3 tools/espfly_auto.py sortie --mode flight --confirm-flight --height 100 --photos 1 --wait-s 60 \
      --plan-right 0.6 --plan-forward 2.5 [--plan-via drone|mac]
                                                       # returns at the wait's start: the photo and the plan
  python3 tools/espfly_auto.py sortie ... --plan-forward 2.2 --person-gap 2.4 --est 4.6 --plan-via mac
  python3 tools/espfly_auto.py wait-move --photo       # its end, travel, D, a photo
  python3 tools/espfly_auto.py move --forward 2.0 --person-gap 1.5 --est 3.8   # one step; --batt-ok late in a charge

The move replies after the 300 % forensics (2026-10-17, docs/autonomy-a2b.md 33): an early reply above 100 % gives
travelled_pre_settle_m (before the settle: not the end); the next move / yaw / set-height reports that motion's final
end under 'previous' (or wait-move); a normal end (DONE, SHORT, LONG: past its target; it holds there, but a move
with a part toward the scene keeps its target and the hold walks it back: the review of 2026-10-17) gets a
'warning' when the measured travel is more than 10 % off the target or off its line; drift_since_last_end_m is the
drone's D at the acceptance against its D at the last motion's end (+ toward the scene; only with NVS mvfix DRIFT,
default on, as the last 'auto get' cached it: without it the drone measures no drift);
sideways_m_at_mean_D the sideways travel of a diagonal at the mean of its start and end distances.
"""
import argparse
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import calendar
import hashlib
import hmac
import json
import math
import os
import queue
import re
import secrets
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import zlib
from collections import deque, namedtuple

# ---------------------------------------------------------------- protocol

PORT = 2391
DRONE_IP = "192.168.43.42"
SUBNET_PREFIX = "192.168.43."
MAGIC, VERSION = 0xE5, 1
HDR_LEN, TAG_LEN, MAX_PAYLOAD, KEY_LEN = 16, 16, 1100, 16
FLAG_ACKREQ, FLAG_RETX = 0x01, 0x02

# Mac -> drone
T_HELLO, T_HB, T_STATUS_REQ = 0x01, 0x02, 0x03
T_ARM, T_TAKEOFF, T_LAND, T_KILL, T_DISARM = 0x10, 0x11, 0x12, 0x13, 0x14   # stage A2b
T_PHOTO, T_CHUNKS, T_PHOTO_FREE = 0x20, 0x21, 0x22
T_YAW, T_MOVE, T_STOP = 0x30, 0x31, 0x32                                     # interactive hover
T_SET_HEIGHT = 0x33                                                          # "you are at X m" (2026-10-15)
# drone -> Mac
T_HELLO_ACK, T_ACK, T_STATUS, T_META, T_CHUNK = 0x81, 0x82, 0x83, 0xA0, 0xA1
T_AUTO_STATUS = 0x84                                                         # stage A2b

TYPE_NAMES = {T_HELLO: "HELLO", T_HB: "HB", T_STATUS_REQ: "STATUS_REQ", T_PHOTO: "PHOTO", T_CHUNKS: "CHUNKS",
              T_PHOTO_FREE: "PHOTO_FREE", T_HELLO_ACK: "HELLO_ACK", T_ACK: "ACK", T_STATUS: "STATUS",
              T_META: "META", T_CHUNK: "CHUNK", T_ARM: "ARM", T_TAKEOFF: "TAKEOFF", T_LAND: "LAND", T_KILL: "KILL",
              T_DISARM: "DISARM", T_AUTO_STATUS: "AUTO_STATUS", T_YAW: "YAW", T_MOVE: "MOVE", T_STOP: "STOP",
              T_SET_HEIGHT: "SET_HEIGHT"}

RESULT_NAMES = ("OK", "BAD_STATE", "BUSY", "RANGE", "TAKEOFF_WINDOW", "CAM_UNAVAIL", "LOW_BATT", "APP_ACTIVE",
                "KILL_LATCHED", "POWER_DEFER", "NO_PHOTO", "MEM", "GUARDS", "STALE_FRAME", "BAD_SESSION")
R_OK, R_BAD_STATE, R_BUSY, R_RANGE, R_NO_PHOTO, R_GUARDS, R_BAD_SESSION = 0, 1, 2, 3, 10, 12, 14
RESULT_HINTS = {
    "BAD_STATE": "not allowed in this stage or state",
    "BUSY": "the drone is busy: another command is pending, or an earlier photo set is still held (until "
            "PHOTO_FREE or 30 s)",
    "RANGE": "a parameter is out of range",
    "TAKEOFF_WINDOW": "refused during a take-off window",
    "CAM_UNAVAIL": "the camera is not running ('cam on' and reboot), or it is asleep for a take-off (its take-off "
                   "standby: wait for the window to end); asleep on the ground it wakes for a PHOTO or an ARM by itself",
    "NO_PHOTO": "no such photo set (freed or expired)",
    "MEM": "the drone could not allocate the photo buffers",
    "KILL_LATCHED": "a Mac KILL less than 2 s ago",
    "GUARDS": "an arming guard failed (see 'guards')",
    "BAD_SESSION": "the drone has a different session (it rebooted or another HELLO replaced ours)",
}

CAM_STATES = ("off", "starting", "running", "failed", "restarting", "standby")   # camera_flow.h

HDR_FMT = "<BBBBIIHH"
HELLO_ACK_FMT = "<III4s24s"
ACK_FMT = "<IBBBBI"
STATUS_FMT = "<IHBBHHIhhhBBBBHIIIII"
META_FMT = "<HBBHHIIIIhhhHBBHHH"
CHUNK_HDR_FMT = "<HBBHH"
CHUNK_DATA_MAX = 1024
FW_LEN = 24
MAX_N = 3

STATUS_FIELDS = ("upMs", "vbatMv", "camStateCode", "state", "fps_x10", "thrust", "frameSeq", "roll_cdeg",
                 "pitch_cdeg", "yaw_cdeg", "lum", "texture", "flags", "photoSlotsFree", "reserved", "authFails",
                 "heapFree", "heapMin", "packetsAccepted", "bootNonce")
STATUS_FLAGS = ("motorsIdle", "takeoffWindow", "onGround", "photoBusy")   # bit0..bit3
META_FIELDS = ("id", "idx", "n", "width", "height", "size", "crc32", "frameSeq", "captureUpMs", "roll_cdeg",
               "pitch_cdeg", "yaw_cdeg", "thrust", "lum", "texture", "chunkSize", "chunkCount", "vbatMv")
# Colour photos (2026-10-15, docs/autonomy-link.md "Photos"): a colour frame's META is 44 bytes, + u8 pixFmt and 3
# zero bytes; a grey frame's stays the 40 bytes of A1 (pixFmt GRAY8). auto_proto.h AUTO_PIXFMT_*.
META_COLOR_LEN = 44
PIXFMT_GRAY8, PIXFMT_YUYV, PIXFMT_JPEG = 0, 1, 2
PIXFMT_NAMES = {PIXFMT_GRAY8: "gray8", PIXFMT_YUYV: "yuyv422", PIXFMT_JPEG: "jpeg"}
PIXFMT_BPP = {PIXFMT_GRAY8: 1, PIXFMT_YUYV: 2}

assert struct.calcsize(HDR_FMT) == HDR_LEN
assert struct.calcsize(HELLO_ACK_FMT) == 40 and struct.calcsize(ACK_FMT) == 12
assert struct.calcsize(STATUS_FMT) == 48 and struct.calcsize(META_FMT) == 40
assert struct.calcsize(CHUNK_HDR_FMT) == 8

Packet = namedtuple("Packet", "type flags session seq payload")


class ProtoError(Exception):
    """A packet that fails the framing, tag or payload checks; reason is one word
    (short, magic, version, length, tag, payload)."""

    def __init__(self, reason, msg=""):
        super().__init__(msg or reason)
        self.reason = reason


def tag(key, data):
    return hmac.new(key, data, hashlib.sha256).digest()[:TAG_LEN]


def encode(key, ptype, flags, session, seq, payload=b""):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload of {len(payload)} bytes exceeds {MAX_PAYLOAD}")
    hdr = struct.pack(HDR_FMT, MAGIC, VERSION, ptype, flags, session, seq, len(payload), 0)
    return hdr + payload + tag(key, hdr + payload)


def decode(key, buf):
    """Framing and tag check (constant-time compare), as autoProtoDecode(); returns a Packet."""
    if len(buf) < HDR_LEN + TAG_LEN:
        raise ProtoError("short")
    if buf[0] != MAGIC:
        raise ProtoError("magic")
    if buf[1] != VERSION:
        raise ProtoError("version")
    plen = struct.unpack_from("<H", buf, 12)[0]
    if plen > MAX_PAYLOAD or len(buf) != HDR_LEN + plen + TAG_LEN:
        raise ProtoError("length")
    if not hmac.compare_digest(tag(key, bytes(buf[:HDR_LEN + plen])), bytes(buf[HDR_LEN + plen:])):
        raise ProtoError("tag")
    _, _, ptype, flags, session, seq, _, _ = struct.unpack_from(HDR_FMT, buf)
    return Packet(ptype, flags, session, seq, bytes(buf[HDR_LEN:HDR_LEN + plen]))


def key_fp(key):
    """Key fingerprint: SHA-256(key)[0:4] as hex, shown by both sides."""
    return hashlib.sha256(key).digest()[:4].hex()


def result_name(code):
    return RESULT_NAMES[code] if code < len(RESULT_NAMES) else f"RESULT_{code}"


# Mac -> drone payloads
def hello_payload(nonce):
    return struct.pack("<I", nonce)


def hb_payload(mac_ms):
    return struct.pack("<I", mac_ms & 0xFFFFFFFF)


def photo_payload(n, gap_ms):
    return struct.pack("<BBH", n, 0, gap_ms)


def chunks_payload(photo_id, idx, count, first):
    return struct.pack("<HBBH", photo_id, idx, count, first)


def photo_free_payload(photo_id):
    return struct.pack("<H", photo_id)


# drone -> Mac payloads (built only by the tests' fake drone; the Mac parses them)
def hello_ack_payload(session, boot_nonce, echo_nonce, fp, fw):
    return struct.pack(HELLO_ACK_FMT, session, boot_nonce, echo_nonce, fp, fw.encode("ascii")[:FW_LEN])


def ack_payload(ack_seq, ack_type, result, state=0, detail=0):
    return struct.pack(ACK_FMT, ack_seq, ack_type, result, state, 0, detail)


def status_payload(fields):
    return struct.pack(STATUS_FMT, *(fields.get(k, 0) for k in STATUS_FIELDS))


def meta_payload(fields):
    """40 bytes (grey, pixFmt absent or GRAY8), else 44 with the pixel format."""
    p = struct.pack(META_FMT, *(fields[k] for k in META_FIELDS))
    fmt = fields.get("pixFmt", PIXFMT_GRAY8)
    return p if fmt == PIXFMT_GRAY8 else p + struct.pack("<B3x", fmt)


def chunk_payload(photo_id, idx, chunk_idx, data):
    return struct.pack(CHUNK_HDR_FMT, photo_id, idx, 0, chunk_idx, len(data)) + data


def _need(payload, size, what):
    if len(payload) != size:
        raise ProtoError("payload", f"{what} payload is {len(payload)} bytes, want {size}")


def parse_hello_ack(p):
    _need(p, struct.calcsize(HELLO_ACK_FMT), "HELLO_ACK")
    session, boot, echo, fp, fw = struct.unpack(HELLO_ACK_FMT, p)
    return {"session": session, "bootNonce": boot, "echoNonce": echo, "keyFp": fp.hex(),
            "fw": fw.split(b"\0", 1)[0].decode("ascii", "replace")}


def parse_ack(p):
    _need(p, struct.calcsize(ACK_FMT), "ACK")
    ack_seq, ack_type, result, state, _, detail = struct.unpack(ACK_FMT, p)
    a = {"ackSeq": ack_seq, "ackType": ack_type, "result": result, "resultName": result_name(result),
         "state": state, "stateName": name_of(STATE_NAMES, state), "detail": detail}
    if result == R_BUSY:
        a["busyKind"], a["busyValue"] = busy_kind(detail)
    return a


def _angles(d):
    for a in ("roll", "pitch", "yaw"):
        d[a + "_deg"] = d[a + "_cdeg"] / 100.0


def parse_status(p):
    _need(p, struct.calcsize(STATUS_FMT), "STATUS")
    s = dict(zip(STATUS_FIELDS, struct.unpack(STATUS_FMT, p)))
    del s["reserved"]
    code = s["camStateCode"]
    s["camState"] = CAM_STATES[code] if code < len(CAM_STATES) else f"state{code}"
    s["vbat_v"] = s["vbatMv"] / 1000.0
    s["fps"] = s["fps_x10"] / 10.0
    s["stateName"] = name_of(STATE_NAMES, s["state"])     # byte 7: autonomy state (A2b)
    _angles(s)
    for bit, name in enumerate(STATUS_FLAGS):
        s[name] = bool((s["flags"] >> bit) & 1)
    return s


def parse_meta(p):
    """A grey drone's 40-byte META (pixFmt GRAY8) or a colour one's 44 bytes; 'color' in the photo JSON."""
    if len(p) not in (struct.calcsize(META_FMT), META_COLOR_LEN):
        raise ProtoError("payload", f"META payload is {len(p)} bytes, want 40 or {META_COLOR_LEN}")
    m = dict(zip(META_FIELDS, struct.unpack_from(META_FMT, p)))
    m["pixFmt"] = p[40] if len(p) == META_COLOR_LEN else PIXFMT_GRAY8
    m["pixFmtName"] = PIXFMT_NAMES.get(m["pixFmt"], f"unknown-{m['pixFmt']}")
    m["color"] = m["pixFmt"] in (PIXFMT_YUYV, PIXFMT_JPEG)
    _angles(m)
    m["vbat_v"] = m["vbatMv"] / 1000.0
    return m


def parse_chunk(p):
    """CHUNK payload -> (photo id, frame idx, chunk idx, data)."""
    if len(p) < struct.calcsize(CHUNK_HDR_FMT):
        raise ProtoError("payload", "CHUNK payload shorter than its header")
    photo_id, idx, _, chunk_idx, n = struct.unpack_from(CHUNK_HDR_FMT, p)
    if n > CHUNK_DATA_MAX or n != len(p) - 8:
        raise ProtoError("payload", f"CHUNK dataLen {n} does not match {len(p) - 8} bytes")
    return photo_id, idx, chunk_idx, p[8:]


# ---------------------------------------------------------------- stage A2b: flight commands, AUTO_STATUS

# Names in enum order, copied from auto_fsm.h, vert_ctrl.h and auto_proto.h (append-only there;
# tools/tests/test_espfly_auto.py checks them against the headers).
STATE_NAMES = ("IDLE", "ARMED", "TAKEOFF", "HOVER", "LAND", "KILLED")                    # autoState_t
S_IDLE, S_ARMED, S_TAKEOFF, S_HOVER, S_LAND, S_KILLED = range(6)
MODE_NAMES = ("FLIGHT", "DRY", "BENCH")                                                  # autoMode_t
MODES = {"flight": 0, "dry": 1, "bench": 2}
MODE_FLIGHT = 0
REASON_NAMES = ("NONE", "ARM", "TAKEOFF", "HOLD", "DISARM_CMD", "ARM_TIMEOUT", "MOVED", "LOCKED", "LAND_CMD",
                "HB_LOST", "APP_INPUT", "APP_SILENT", "SESSION", "AIR_TIME", "VBAT_LOW", "VBAT_DIP", "LOOP_GAP", "VZ",
                "NOT_STABLE", "SORTIE_DONE", "CAPTURE_FAIL", "TAKEOFF_TIMEOUT", "TOUCHDOWN", "TOUCHDOWN_TRIP",
                "LAND_TIMEOUT", "KILL_CMD", "TILT", "RATE", "FREEFALL", "ATT_ERR", "NAN", "VZ_CUT", "MOTOR_GUARD",
                "MOTOR_STOP", "ESTOP_EXT", "LAND_BACKSTOP", "KILL_CLEARED", "HOVER_TIMEOUT",
                "HOVER_NO_CAM", "BATT_LOW", "MOVE_NO_CAM", "MOVE_TILT", "COLLISION", "PALM", "SINK",
                "RESUME")                                                                # autoReason_t
VZ_PHASE_NAMES = ("IDLE", "SPIN", "RAMP", "CLIMB", "HOLD", "LAND", "DONE", "CUT")       # vzPhase_t
VZ_DONE, VZ_CUT = 6, 7
VZ_REASON_NAMES = ("NONE", "TAKEOFF", "SPIN_DONE", "LIFTOFF", "HOLD_REACHED", "CMD_LAND", "CMD_MISSING",
                   "SPIN_NOISY", "SPIN_TILT", "SPIN_GYRO", "SPIN_SAG", "NO_LIFTOFF_TIME", "NO_LIFTOFF_CEIL",
                   "LIFTOFF_ATTITUDE", "CLIMB_CAPPED", "CLIMB_TIMEOUT", "HOLD_CAPPED", "HOLD_TIMEOUT", "K_RANGE",
                   "EST_RANGE", "ABORTED", "TOUCHDOWN_FORCE", "TOUCHDOWN_JOLT", "LAND_TIMEOUT", "CMD_CUT",
                   "NAN_INPUT", "BAD_CONFIG", "HOLD_NO_CAM", "TOUCHDOWN_SUPPORT", "RESUME",
                   "CONTACT_HOLD")                                                       # vzReason_t
# What the drone did on its own and holds reporting ("Claude decides", 2026-10-15, docs/autonomy-a2b.md 22):
# AUTO_STATUS bytes 96-135 (auto_fsm.h autoEvent_t, AUTO_HB_*, AUTO_CB_*).
EVENT_NAMES = ("NONE", "CAM_LOST", "CAM_BACK", "SINK", "RESUME", "CONTACT", "VZ", "NOT_STABLE", "TAKEOFF_SLOW",
               "CAPTURE_FAIL", "MOVE_STOPPED", "LOOP_GAP", "RETRACK_UNDO", "HOLD_NO_VIEW", "VIEW_BACK",
               "HOLD_LIMIT")                                                             # autoEvent_t
EV_CAM_LOST, EV_SINK, EV_CONTACT, EV_VZ, EV_MOVE_STOPPED, EV_RETRACK_UNDO = 1, 3, 5, 6, 10, 12
HB_NAMES = ("camLost", "sinking", "hush", "kClamp", "capped", "noView", "notStable", "openLoop")   # AUTO_HB_*
CB_NAMES = ("noFrames", "noG", "cusum", "fault", "notRunning")                                   # AUTO_CB_*
NS_NAMES = ("tilt", "gyro", "hold", "vz")          # AUTO_NS_*: NOT_STABLE's detail, the STABLE conditions that failed
NS_HINTS = {"tilt": "roll or pitch over the STABLE limit", "gyro": "a rate over the STABLE limit",
            "hold": "the camera hold not applied on a fresh frame", "vz": "the vertical controller not steady"}
SINK_WHY = ("budget", "noCapQ", "afterContactOrClimbFault")   # SINK's detail (auto_fsm.h AUTO_EV_SINK)
SINK_WHY_HINTS = ("the camera height was out 5 s (the open-loop hold's budget)",
                  "the camera height went out before the hover thrust was measured on it (no open-loop bound): at "
                  "once",
                  "the camera height went out after a contact or a climb fault (no measured hover): at once")
VIEW_WHY = (None, "coasting", "notApplied", "stale")   # HOLD_NO_VIEW's detail: 1 coasting, 2 not applied, 3 no fresh
                                                       # frame
EV_SINK_CODE, EV_NOT_STABLE, EV_LOOP_GAP, EV_HOLD_NO_VIEW = 3, 7, 11, 13
EV_HOLD_LIMIT = 15            # the camera hold at its limit (2026-10-16, a2b 27.4): information, the drone holds
HL_ROLL, HL_PITCH, HL_ROLL_NEG, HL_PITCH_NEG = 0x01, 0x02, 0x04, 0x08   # its detail (auto_fsm.h AUTO_HL_*)
CONTACT_NO_HUSH, CONTACT_MOTION = 0x40, 0x80   # CONTACT detail bits (the low 6: the controller's phase)
EVENT_HINTS = {
    "CAM_LOST": "the camera height is out: the drone holds open loop under its measured hover (it cannot climb) and "
                "sinks to the ground at the countdown (sinkIn_s) unless it comes back; take a photo. With T2 on "
                "(cvt2k > 0, bench Q): take a photo within ~5 s of CAM_BACK too (after a camera gap T2's estimate "
                "can sit 0.3-1 m low in the replays: the drone climbs)",
    "CAM_BACK": ("the camera height is back: the hold is closed again (with T2 on, cvt2k > 0: a photo within ~5 s, the "
                 "drone may have climbed during the gap)"),
    "SINK": "the drone is sinking to the ground (LAND SINK): it resumes the hover if the camera height comes back; "
            "a LAND takes it over",
    "RESUME": "the camera height came back in the SINK: the drone hovers again (HOVER RESUME)",
    "CONTACT": "an in-air contact: the drone held (the estimate frozen 0.5 s, then 1.5 s open loop) and stopped any "
               "motion (a MOVE it stopped: its hold anchor goes 0.25 m back the way it came; no motion: the hold "
               "re-anchors where the bounce left it); obstacle_bearing_deg is where it was hit from (body frame now: "
               "0 ahead, +90 right); take a photo before any move, back away from it or LAND on a hand",
    "VZ": "the vertical controller held and reported (its reason in detailName)",
    "NOT_STABLE": "the hover never held STABLE 1 s: the wait started anyway (detailName: the conditions that failed); "
                  "wait 2-3 s and check a photo before moving",
    "TAKEOFF_SLOW": "the take-off photos were not done by the pre-wait time: the wait started anyway",
    "CAPTURE_FAIL": "a photo set failed: take another PHOTO",
    "MOVE_STOPPED": "the drone stopped the motion (detailName: NO_VIEW, TILT, LOOP_GAP): it holds there",
    "LOOP_GAP": "a stabilizer loop gap (gap_ms; 255: longer): a running MOVE stopped; three within 1 s land",
    "RETRACK_UNDO": "the YAW found no scene distance after its turn (or lost its view): it turns back to its start "
                    "heading",
    "HOLD_NO_VIEW": "the camera hold has no fresh view (detailName: coasting, notApplied, stale): the drone drifts; "
                    "photo, then turn toward texture (a blind YAW up to 90 deg) or LAND (with T2 on, cvt2k > 0: that "
                    "photo within ~5 s, and check the height on it)",
    "VIEW_BACK": "the camera hold has its view again",
    "HOLD_LIMIT": ("the camera hold's correction has sat at >= 1.3 of its 1.5 deg for 8 s (outside any MOVE/YAW) on "
                   "the axes in detailName: the drone needs that lean (the trim, a bent leg, a prop, the battery's "
                   "seat, the air there) and cannot hold more, so it may creep the way 'creeps' says; take a photo and "
                   "watch the drift, land soon, then run 'espfly_diag.py trimcheck' on the dump and check the legs, "
                   "props and battery seat (do not raise the hold's limit); the drone only reports it"),
}
GUARD_NAMES = ("LINK", "LOCKS", "STILL", "BATT", "CAMERA", "APP", "GCAP", "ENABLED", "VZPARAMS", "HEAP", "GESTURE",
               "KILL", "MODE")                                                           # AUTO_G_* bits
CMD_NAMES = ("NONE", "ARM", "TAKEOFF", "LAND", "KILL", "DISARM", "PHOTO", "MOVE", "YAW", "STOP",
             "SET_HEIGHT")                                                               # autoCmdType_t
CAPTURE_NAMES = ("NONE", "BUSY", "DONE", "FAILED")                                       # autoCapture_t
CAP_NONE, CAP_BUSY, CAP_DONE, CAP_FAILED = range(4)
STF_NAMES = ("waiting", "stable", "camHeight", "camAwake", "resetLocked", "prespin", "sim", "hold")  # AUTO_STF_*
TRF_NAMES = ("tilt", "rate", "fall", "attErr", "vbatLow", "moved", "rest", "vzCap")      # AUTO_TRF_*
STF_WAITING = 0x01
CMD_OF_TYPE = {T_ARM: 1, T_TAKEOFF: 2, T_LAND: 3, T_KILL: 4, T_DISARM: 5, T_PHOTO: 6, T_MOVE: 7, T_YAW: 8, T_STOP: 9,
               T_SET_HEIGHT: 10}
DETAIL_TAKEN_UNKNOWN = 0xFFFFFFFF     # auto_mailbox.h: the state machine took it; its result did not come
# ACK BUSY detail kinds (auto_proto.h AUTO_DETAIL_BUSY_*): a BUSY the link itself answers carries its kind in
# bits 24-31 and a value in the low bits. A BUSY from the state machine keeps its documented detail (the
# state: kind 0), and a CHUNKS answered BUSY keeps detail 0. A detail is a photo id only when its kind is
# HELD_SET.
DETAIL_KIND_MASK, DETAIL_VALUE_MASK = 0xFF000000, 0x00FFFFFF
DETAIL_BUSY_PENDING = 0x01000000      # another command is pending (value: its seq & 0xFFFFFF); not processed
DETAIL_BUSY_UNTAKEN = 0x02000000      # not processed, guaranteed: slot held or withdrawn untaken (value: state)
DETAIL_BUSY_HELD_SET = 0x03000000     # PHOTO refused while a photo set is held (value: its id, 16 bits)
BUSY_KIND_NAMES = {DETAIL_BUSY_PENDING: "pending", DETAIL_BUSY_UNTAKEN: "untaken",
                   DETAIL_BUSY_HELD_SET: "held_set"}

AUTO_STATUS_FMT = "<IIII10B8H2h6H2BI"
AUTO_STATUS_FIELDS = ("upMs", "bootNonce", "transitions", "failedGuards", "state", "mode", "reason", "vzLandReason",
                      "landTrip", "vzPhase", "vzReason", "flags", "trips", "capture", "photoId", "stateMs", "airMs",
                      "stableMs", "waitLeftMs", "hoverWaitMs", "vbatMv", "vbatFastMinMv", "hMm", "vzMms", "thrust",
                      "tLo", "tHold", "tRef", "kPriorDiv10", "khatDiv10", "lastCmdType", "lastCmdResult", "killCount")
assert struct.calcsize(AUTO_STATUS_FMT) == 64 and len(AUTO_STATUS_FIELDS) == 33
# The interactive hover appends 32 bytes (AUTO_STATUS is 96 bytes; an A2b drone sends the first 64 only).
AUTO_STATUS_EXT_FMT = "<6BH6hHhhhHH"
AUTO_STATUS_EXT_FIELDS = ("mvState", "mvKind", "mvWhy", "mvEnd", "mvFlags", "battLow", "mvSeq", "progRightCm",
                          "progFwdCm", "progUpCm", "tgtRightCm", "tgtFwdCm", "tgtUpCm", "sceneDCm", "hTargetCm",
                          "yawOffCdeg", "yawTgtDeg", "mvPhaseMs", "airLeftDs")
AUTO_STATUS_LEN_A2B, AUTO_STATUS_LEN_IH = 64, 96
assert struct.calcsize(AUTO_STATUS_EXT_FMT) == 32 and len(AUTO_STATUS_EXT_FIELDS) == 19
# Hold and report ("Claude decides", 2026-10-15) appends 40 bytes (136; an interactive-hover drone sends 96).
AUTO_STATUS_HR_FMT = "<BBHHBBHHHHhhHHHHhBBBBHBBH"
AUTO_STATUS_HR_FIELDS = ("evCode", "evDetail", "evCount", "evAgeMs", "holdBits", "camBits", "camOffMs", "sinkInMs",
                         "peakAccCg", "peakRateDps", "contactAxCg", "contactAyCg", "sinceCmdDs", "airDs", "khatPm",
                         "capQ", "contactYawCdeg", "contacts", "loopGaps", "ev1Code", "ev1Detail", "ev1AgeMs",
                         "ev2Code", "ev2Detail", "ev2AgeMs")
AUTO_STATUS_LEN_HR = 136
assert struct.calcsize(AUTO_STATUS_HR_FMT) == 40 and len(AUTO_STATUS_HR_FIELDS) == 25
# The speed step (fast moves, 2026-10-16, docs/autonomy-a2b.md 31) appends 4 bytes (140; a hold-and-report drone sends
# 136 and flies 100 %).
AUTO_STATUS_MS_FMT = "<HH"
AUTO_STATUS_MS_FIELDS = ("mvSpeedPct", "reservedMs")
AUTO_STATUS_LEN = 140
assert struct.calcsize(AUTO_STATUS_MS_FMT) == 4

# The interactive hover (auto_motion.h; user decision 2026-10-12, late): names in enum order.
MV_STATE_NAMES = ("IDLE", "RUN", "PAUSED", "SETTLE", "DONE", "STOPPED")                  # autoMvState_t
MV_IDLE, MV_RUN, MV_PAUSED, MV_SETTLE, MV_DONE, MV_STOPPED = range(6)
MV_ACTIVE = (MV_RUN, MV_PAUSED, MV_SETTLE)
MV_KIND_NAMES = ("NONE", "MOVE", "YAW")                                                  # autoMvKind_t
MV_WHY_NAMES = ("NONE", "NOT_WAITING", "BUSY", "RANGE", "HEIGHT", "NO_DIST", "TOO_CLOSE", "NO_VIEW",
                "NO_CAM_HEIGHT", "BATT", "AIR", "TOO_WIDE", "PLAN_CANCELLED", "SH_STEP", "SH_RATE",
                "SH_CONTACT", "UP_HEADROOM")                                             # autoMvWhy_t
MV_END_NAMES = ("NONE", "DONE", "STOP_CMD", "PROGRESS", "LEFT_HOVER", "RETRACK", "SHORT", "FRONT", "CONTACT",
                "NO_VIEW", "TILT", "UNDONE", "LOOP_GAP", "VIEW_FAST", "LONG")              # autoMvEnd_t
MVE_DONE, MVE_PROGRESS, MVE_SHORT, MVE_FRONT, MVE_LONG = 1, 3, 6, 7, 14
MVE_OK = (MVE_DONE, MVE_SHORT, MVE_LONG)   # ended normally (SHORT / LONG: its settle timed out short of its target or
                                           # off its line / past it; the measured end is its end, it holds there)
MVF_NAMES = ("dValid", "dStale", "dSim", "camGuess", "bob", "dTrack", "blindTurn", "undo")  # AUTO_MVF_*
MVF_D_VALID, MVF_D_STALE, MVF_BOB, MVF_D_TRACK = 0x01, 0x02, 0x10, 0x20
MVF_SET_HEIGHT = 0x40         # with mvKind MOVE: a SET_HEIGHT's correction (AUTO_MVF_SET_HEIGHT; BLIND_TURN on a YAW)
BATTLOW_HELD, BATTLOW_NOW = 0x01, 0x02    # AUTO_STATUS battLow bits (AUTO_BATTLOW_*): the drone's BATT_LOW condition
                                          # (battery < 3.00 V or hover thrust > 57,000) held 0.25 s / holds now
SH_GAP_S = 2.0                # SET_HEIGHT: at most one accepted per this (the drone's AUTO_SH_GAP_MS)
MV_WHY_HINTS = {
    "NOT_WAITING": "the drone is not in the hover wait (MOVE and YAW only after the take-off photo)",
    "BUSY": "a motion is running: STOP it or wait for it to end",
    "RANGE": ("out of range: |right| <= 1.50 m, forward <= 4.00 m, back <= 1.50 m, the vector <= 4.00 m, |up| <= "
              "0.50 m (with up, |forward| <= 1.50 m), |yaw| <= 180 deg (a drone before the long-move build, "
              "b6a54da and older: 1.50 m on every axis)"),
    "HEIGHT": ("a move down would end below 0.25 m + min(0.30 m, 0.01 m/s x the air time) (the height estimate "
               "drifts, reading high), or a move up above 2.00 m (the cap: 1.20 m on a drone before 2026-10-15); a "
               "level move or a YAW is never refused for the height"),
    "NO_DIST": "no scene distance D yet (the camera tracker has no G: after a YAW or a long sideways move the "
               "drone bobs to re-track)",
    "TOO_CLOSE": "a forward move could leave less than the drone's floor to the scene ahead (NVS mvfloor, default "
                 "0.30 m, 0 = no floor but 0.05 m, for an expansion scale k down to NVS mvklow / 100, default 0.6; the "
                 "take-off's planned move 5 cm more, the Mac's)",
    "NO_VIEW": "the camera hold is not applied on a fresh view",
    "NO_CAM_HEIGHT": "a vertical move needs the camera height estimate in use",
    "BATT": ("hover only: the hover needs over 53,400 of the 58,000 ceiling, or the battery reads below 3.00 V; "
             "the drone lands BATT_LOW by itself at 57,000 or below 3.00 V for 0.5 s. A move down is refused too (its "
             "braking near the ceiling can end HOLD_CAPPED): from up high, come down from ~52,000 (the runbook)"),
    "AIR": ("the motion and a LAND would not fit in the 120 s air limit (a drone before 2026-10-15; the drone has no "
            "air clock now)"),
    "TOO_WIDE": "sideways at most NVS mvlat / 100 x D per move (default 0.5; a nearer object under the move would make "
                "it longer): split it (or mvlat 0: no cap)",
    "PLAN_CANCELLED": ("the TAKEOFF's planned move was not started: the drone reported an event before the hover wait "
                       "(hold.events: a contact, NOT_STABLE, the camera height lost, ...); it hovers in the wait: take "
                       "a photo, then decide (a plain 'move')"),
    "SH_STEP": ("SET_HEIGHT: the correction is over 0.50 m (or the flight over 0.55 m): send 0.50 m of it, take a new "
                "photo, then the rest"),
    "SH_RATE": f"SET_HEIGHT: one accepted per {SH_GAP_S:g} s: wait {SH_GAP_S:g} s",
    "SH_CONTACT": ("SET_HEIGHT: a contact's hush or its window (the camera height not back 0.5 s since): wait for it, "
                   "then take a new photo"),
    "UP_HEADROOM": ("a climb needs thrust headroom the motors no longer have: the hover thrust T_hold is within 4,600 "
                    "(or 10 %) of the ceiling in force (58,000), i.e. above ~53,400 (the thrust, not the voltage: the "
                    "tiger climb ran fine at 3.12-3.22 V with T_hold 41-52k); level moves, descents and STOP/LAND still "
                    "work: come down or land, the battery's end is near"),
}
# SET_HEIGHT's refusals in its own words (docs/autonomy-a2b.md 23, the runbook's step 9)
SH_WHY_HINTS = {
    "NOT_WAITING": "not hovering in the wait with the controller in HOLD (TAKEOFF, LAND, a SINK, IDLE ...)",
    "BUSY": "a motion is running (a MOVE, a YAW, a bob, a correction): wait for its DONE or STOPPED (<= ~7 s)",
    "RANGE": "X outside 0.10..2.50 m, or the correction rounds to 0 cm",
    "HEIGHT": ("the corrected estimate would leave 0.10 m..the cap + 0.50 m (the judgement is implausible, take a new "
               "photo), or the target form's descent would end under a MOVE down's floor (0.25 m + 0.01 m/s of air, at "
               "most 0.55 m; 2026-10-16 review): pick a higher --to"),
    "NO_CAM_HEIGHT": "the camera height estimate is out (open loop ignores the correction): wait for CAM_BACK, then a new "
                     "photo",
    "NO_VIEW": "the camera hold has no fresh view: wait for VIEW_BACK, then a new photo",
    "BATT": ("near the end of the battery (the hover over 53,400, or the BATT_LOW condition) no correction flies, "
             "up or down (a descent's braking there can end HOLD_CAPPED, a fast LAND): LAND, or come down earlier "
             "(the runbook: from T_hold ~52,000)"),
    "SH_STEP": ("|X - h_at| over 0.50 m, or the flight (target - true now) over 0.55 m: send 0.50 m of it (the Mac clips "
                "a long flight to 0.50 m by itself and remembers the target), take a new photo, then set-height again"),
    **{k: MV_WHY_HINTS[k] for k in ("SH_RATE", "SH_CONTACT", "UP_HEADROOM")},
}
SH_X_M = (0.10, 2.50)         # SET_HEIGHT X, m (the drone's AUTO_SH_X_CM_MIN..MAX)
SH_AT_M = (-3.0, 3.0)         # set-height --at M: a plausible estimate
SH_MAX_M = 0.50               # the drone's AUTO_SH_MAX_M (|d| per command)
SH_FLY_MAX_M = 0.55           # the drone's AUTO_SH_FLY_MAX_M (|the flight|)
SH_FLY_CLIP_M = 0.50          # a longer flight to the target: the Mac sends --to this far from the true height now
SH_TO_M = (0.25, 2.00)        # set-height --to (the drone's AUTO_SH_TO_CM_MIN..MAX: the MOVE range)
SH_TO_REF = -32768            # toCm: the commanded height (auto_fsm.h AUTO_SH_TO_REF, INT16_MIN)
SH_PHOTO_OLD_S = 10.0         # a photo older than this: a warning (the estimate's drift since it is not corrected)
UP_HEADROOM_T = 4600          # the drone's AUTO_MV_UP_HEADROOM_T (a climb refused within it of the ceiling)
AUTO_MV_REFUSE_T_MAC = 53400  # 58000 - UP_HEADROOM_T: T_hold above it, a climb is refused (a warning here)
# A climb late in the battery (2026-10-16 review, leftdrift F2): a warning only, never a refusal (the user's 10-16 input:
# "you could have used more thrust"). F2 climbed +0.42 m at T_hold ~52.0k / 3.17 V and sank at the 58000 cap 2.4 s
# after it settled; fastcheck (10-15) climbed +0.13 m at ~52.0k / 3.12 V and held 16 s after it: no T_hold / voltage
# threshold separates the two, so the Mac names the condition and Claude decides from the photo.
LATE_CLIMB_T, LATE_CLIMB_V = 51000, 3.20
SH_WAIT_S = 15.0              # a correction's wait: 0.50 m 4.1 s + the settle <= 3 s, generous
STATUS_RING_N = 1200          # AUTO_STATUS snapshots kept (~2 min at 10 Hz): the height estimate at a photo's capture
CAPTURE_MATCH_MS = 400        # a capture time farther than this from every snapshot: the one nearest the META's arrival

# How a motion ended, for the replies (the end names above).
MV_END_HINTS = {
    "STOP_CMD": " (a STOP): the drone holds where it is",
    "PROGRESS": " (the camera saw the drone off its path): the drone holds where it is",
    "RETRACK": " (no scene distance after the turn, and its turn back found none either): the drone holds; photo, "
               "then turn toward a textured scene",
    "CONTACT": " (an in-air contact): the drone stopped and holds; photo, then back away from what it hit",
    "NO_VIEW": " (the camera hold lost its view while moving): the drone holds where it is; photo",
    "TILT": " (tilted over the limit while moving): the drone holds; photo, then LAND if it is caught",
    "UNDONE": " (it turned back to its start heading and re-tracked there)",
    "LOOP_GAP": " (a stabilizer loop gap): the drone holds where it is",
    "FRONT": ": stopped: the scene ahead got close; holding, take a photo",
    "LONG": (" (its settle timed out past the target by the travel's excess; the move fixes of 2026-10-17, mvfix "
             "LONG: a move with a part toward the scene keeps its target, the hold walks the drone back to it; any "
             "other holds where it ended): take a photo before the next step"),
    "VIEW_FAST": " (a fast MOVE lost its view for longer than its VIEW_FAST timer, 'auto status' mvView ms, and was "
                 "stopped with its brake): the drone holds where it is; "
                 "take a photo",
}

HEIGHT_CM = (20, 120)         # TAKEOFF heightCm (AUTO_HEIGHT_CM_MIN..MAX; 20..60 until the interactive hover). The
                              # hover goes to MOVE_H_RANGE_M[1] by moves up only (vert_ctrl.h VZ_H_TAKEOFF_MAX)
ACT_WAIT_MS = (1000, 120000)  # TAKEOFF actWaitMs when not 0 (AUTO_HOVER_WAIT_TAKEOFF_MIN_MS..AUTO_HOVER_WAIT_MAX_MS;
                              # 60000 until 2026-10-14: an older drone refuses a wait over 65.5 s, RANGE detail 0)
WAIT_SAT_MS = 65535           # AUTO_STATUS waitLeftMs / hoverWaitMs saturate here: "65.5 s or more"
AIR_HARD_MS = 120000          # AUTO_AIR_HARD_MS: never longer in the air (the whole interactive hover; 80000 until
                              # the user decision of 2026-10-14)
AIR_CLOCK_TOL_MS = 200        # AUTO_STATUS airMs + 100 x airLeftDs is the drone's air limit (airLeftDs rounds down
                              # to 0.1 s): further than this from AIR_HARD_MS, the drone is another version
# MOVE / YAW limits (auto_motion.h): the drone refuses beyond them; the Mac checks first. Long moves (2026-10-14,
# docs/autonomy-a2b.md 21): forward (toward the camera's scene) up to MOVE_FWD_MAX_M, sideways and back
# MOVE_HORIZ_MAX_M (the camera does not see behind), the vector MOVE_LEN_MAX_M, a move with up != 0 at most
# MOVE_HORIZ_MAX_M forward. MOVE_FWD_MAX_M is the drone's AUTO_MV_FWD_MAX_CM: 1.50 until the long-move build flies
# (phase 1: b6a54da caps every axis at 1.50 m), 4.00 with it.
MOVE_HORIZ_MAX_M, MOVE_UP_MAX_M, YAW_MAX_DEG = 1.50, 0.50, 180
MOVE_FWD_MAX_M = 4.00
MOVE_LEN_MAX_M = 4.00
YAW_RATE_DPS = 30.0      # auto_motion.h AUTO_MV_YAW_RATE_DPS (user decision 2026-10-13, evening: 15 -> 30)
# The drone's horizontal profile (auto_motion.h): top speed AUTO_MV_V_H_MAX (user request 2026-10-14: 0.20 -> 0.30),
# capped by the camera's view rates, AUTO_MV_W_LAT_MAX x D sideways and AUTO_MV_W_FWD_MAX x the end distance forward,
# so 0.30 m/s only for long sideways (or backward) moves with the scene 2.5 m away or more. The component toward the
# scene ahead stays at AUTO_MV_V_FWD_MAX 0.20 m/s (approaches to a person as before). move_timeout() keeps its
# 0.20 m/s basis on purpose: more generous than the drone's profile.
MOVE_V_H_MAX, MOVE_A_H, MOVE_W_LAT_MAX, MOVE_W_FWD_MAX = 0.30, 0.10, 0.12, 0.15
MOVE_V_FWD_MAX = 0.20
# Fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31): one NVS setting, the speed step in % (the drone's
# AUTO_MV_SPEED_*_PCT), latched at ARM and reported in AUTO_STATUS mvSpeedPct; mv_limits() mirrors auto_motion.c
# autoMvSpeedCfg (each limit x s = pct / 100 up to its own cap: AUTO_MV_FAST_*), move_profile() its profile planning
# (tools/tests/mvspeed_golden.json pins the two together).
MV_SPEED_PCT = (100, 300)
MV_SPEED_DEFAULT = 100
# (after the review of cbccec5: v_h 0.60 -> 0.40, w_fwd 0.20 -> 0.17, yaw 40 / 120 -> 35 / 70, bob 0.40 -> 0.20)
# (after the 300 % forensics of 2026-10-17: a_h 0.30 -> 0.20, the settle at least 2.5 s, 1.0 before)
MV_FAST_CAPS = {"v_h": 0.40, "v_back": 0.40, "v_fwd": 0.50, "a_h": 0.20, "w_lat": 0.14, "w_fwd": 0.17, "brake": 0.40,
                "yaw_rate": 35.0, "yaw_acc": 70.0, "bob_acc": 0.20}
MV_FAST_SETTLE_MIN_MS, MV_FAST_DWELL_MIN_MS = 2500, 300
# 'real 300 %' (user request 2026-10-17 "no, I want real 300%", docs/autonomy-a2b.md 35): the A_H cap and the settle
# floor above 100 are the drone's NVS mvacc (cm/s^2) and mvsettle (ms), latched at its ARM; their defaults are the two
# above (MV_FAST_CAPS["a_h"] x 100, MV_FAST_SETTLE_MIN_MS); mvacc 30 / mvsettle 1000 give 4dea542's profiles
# (tools/tests/mvspeed_golden_4dea542.json). fast_caps() reads them from the last 'auto get'.
MV_ACC_CM = (10, 40)          # AUTO_MV_ACC_MIN_CM..AUTO_MV_ACC_MAX_CM
MV_ACC_DEFAULT_CM = 20        # AUTO_MV_ACC_DEFAULT_CM
MV_SETTLE_FLOOR_MS = (1000, 3000)   # AUTO_MV_SETTLE_FLOOR_MIN_MS..AUTO_MV_SETTLE_FLOOR_MAX_MS
# The guards down to off (user decision 2026-10-18 "remove the guards or make them absolutely minimum",
# docs/autonomy-a2b.md 38): the drone's NVS mvwmax (the expansion rate cap AUTO_MV_W_FWD_MAX x 100, 10..40) rides with
# the fast caps in fast_caps() / mv_limits(); mvklow, mvlat and mvfloor 0 are mirrored by cached_k_low(),
# cached_lat_rel() and mac_min_front() (below, with the ahead margins).
MV_WFWD_PCT = (10, 40)        # AUTO_MV_MG_WFWD_MIN_PCT..AUTO_MV_MG_WFWD_MAX_PCT
MV_WFWD_DEFAULT_PCT = 15      # AUTO_MV_W_FWD_PCT (MOVE_W_FWD_MAX x 100)
MV_FAST_DEFAULT = {"mvacc": MV_ACC_DEFAULT_CM, "mvsettle": MV_FAST_SETTLE_MIN_MS, "mvwmax": MV_WFWD_DEFAULT_PCT,
                   "src": "default"}
MV_FAST_T_TRACK_S = 0.40      # AUTO_MV_FAST_T_TRACK_S: the hold's lead at speed (margins; the person rule's speed term)
MV_FAST_PAUSE_V = 0.20        # AUTO_MV_FAST_PAUSE_V: a fast MOVE losing its view above this is stopped (VIEW_FAST)
MOVE_BRAKE_ACC = 0.15         # AUTO_MV_BRAKE_ACC
MOVE_SETTLE_MS = 3000         # AUTO_MV_SETTLE_MAX_MS
YAW_ACC_DPS2 = 60.0           # AUTO_MV_YAW_ACC_DPS2
YAW_DWELL_MS = 500            # AUTO_MV_YAW_DWELL_MS
BOB_ACC, BOB_A_M, BOB_CYCLES = 0.20, 0.04, 4   # the re-track bob (AUTO_MV_BOB_ACC, _A_M, _CYCLES)
FROZEN_REF = 0.10             # AUTO_MV_FROZEN_REF
PERSON_SPEED_T = 0.40         # s: the person rule's speed term above 100 % (the hold's lead x the forward peak)
BEHIND_EXTRA_S = 2.0          # a command waiting behind an early-returned motion: its settle + 4 bob cycles + this
MOVE_H_RANGE_M = (0.25, 2.00)   # a move's end height (AUTO_MV_H_MIN_M..AUTO_MV_H_MAX_M; 1.20 until 2026-10-15)
# The ahead margins (user decisions 2026-10-17, "you are too scared to approach things", docs/autonomy-a2b.md 32): the
# drone's floor is NVS mvfloor (default DRONE_MIN_FRONT_M, AUTO_MOVE_MIN_FRONT_CM; 0.50 until then), its live front stop
# mvfstop (0.25; 0.40); the Mac keeps MAC_FRONT_EXTRA_M more (mac_min_front(): the cached mvfloor + it), and so does
# the TAKEOFF's planned move on the drone (AUTO_PLAN_FRONT_EXTRA_CM).
DRONE_MIN_FRONT_M = 0.30      # AUTO_MOVE_MIN_FRONT_CM: the default mvfloor
MAC_FRONT_EXTRA_M = 0.05      # AUTO_PLAN_FRONT_EXTRA_CM
MAC_MIN_FRONT_M = 0.35        # the Mac leaves this much to the scene ahead at the default mvfloor (0.60 until
                              # 2026-10-17, with the drone's 0.50) ...
K_LOW = 0.60                  # ... for an image expansion scale k down to this (AUTO_MV_K_LOW): the true end of a
                              # forward move F from D is D ((D - F) / D)^(1/k), shorter than D - F when k < 1. The
                              # default of the drone's NVS mvklow (x 100; cached_k_low(), 2026-10-18)
LAT_D_REL = 0.50              # sideways at most this x D per MOVE (AUTO_MV_LAT_D_REL); the default of the drone's NVS
                              # mvlat (x 100, 0 off; cached_lat_rel(), 2026-10-18)
MV_KLOW_PCT = (60, 100)       # AUTO_MV_MG_KLOW_MIN_PCT..AUTO_MV_MG_KLOW_MAX_PCT
MV_LAT_PCT = (0, 200)         # 0..AUTO_MV_MG_LAT_REL_MAX_PCT
MOVE_D_FLOOR_M = 0.05         # auto_motion.c D_FLOOR_M: a forward move never plans nearer than this to the scene, even
                              # with the drone's floor off (mvfloor 0)
H_FLOOR_M, H_DRIFT_MPS = 0.25, 0.010   # a move down: >= 0.25 m + 0.01 m/s x the air time (AUTO_MV_H_*), one-sided:
                              # a motion that does not go down is never refused for it (2026-10-15)
H_DRIFT_CAP_M = 0.30          # ... the drift term at most this (AUTO_MV_H_DRIFT_CAP_M)
WAIT_LONG_S, WAIT_PRE_S = 12, 8   # sortie --wait-s above 12 s: --height >= 0.25 + 0.01 x min(wait + 8 s, 80 s) m
WAIT_DRIFT_CAP_S = 80             # ... the drift term capped at the old 80 s horizon (user-decision default 2026-10-14:
                                  # waits >= 72 s need 1.05 m, not the impossible 1.53 m of a 120 s wait; past ~80 s
                                  # the floor sink is slow and ends in a LAND either way, and a higher take-off would
                                  # raise the ceiling exposure). None: no cap (the rule of before)
# The ceiling rule (CEIL_H_M 1.05 m, CEIL_AIR_S 80 s: advice, a warning and an up-move refusal) was removed on
# 2026-10-15 (docs/autonomy-a2b.md 23): Claude checks the height from the photos' level line and corrects it with
# set-height; the ceiling clearance is Claude's judgement (the runbook), not a rule here. Its successor is a warning
# only (never a refusal, no room knowledge): height_check_due, once per lapse, when the latest height check of the
# sortie (a saved photo or an accepted set-height) is older than the runbook's cadence at the commanded height: the
# estimate drifts 0.01-0.03 m/s either way, toward a ceiling or a face as easily as toward the floor (2026-10-15 review:
# the 80 s warning would fire on every hover above 1.05 m with the 2.00 m cap).
HEIGHT_CHECK_S = 15.0         # the runbook: a photo every 10-15 s at <= HEIGHT_CHECK_HIGH_M (commanded) ...
HEIGHT_CHECK_HIGH_S = 10.0    # ... every 5-10 s above it
HEIGHT_CHECK_HIGH_M = 1.20
AUTOWAIT_MS = (0, ACT_WAIT_MS[1])   # NVS autowait (auto_glue_core.c: 0..AUTO_HOVER_WAIT_MAX_MS)
PERSON_MIN_M = 0.5            # move / sortie --person-gap: never closer than this (user decision
                              # 2026-10-17: 'you can get close to humans about 0.5 meter'; 1.0 m from
                              # 2026-10-13, 2 m before)
# The battery's duration (2026-10-14, docs/autonomy-a2b.md 21.5): under load below BATT_DUR_FROM_V the battery falls
# ~BATT_DUR_VPS (the gym's end-of-charge flights): a move that would not end (its profile + its settle) before
# BATT_END_V is refused unless --batt-ok (a BATT_LOW partway lands in place: safe, but the move is lost)
BATT_DUR_FROM_V, BATT_END_V, BATT_DUR_VPS = 3.25, 3.00, 0.005
MOVE_TIMEOUT_MARGIN_S = 15.0  # a MOVE's wait: its profile + this (37 s for 4 m)
MOVE_REACHED_FRAC = 0.90      # a PROGRESS stop with this much measured along the move: reached (the guard at its end)
MOVE_REACHED_MAX_FRAC = 1.10  # ... and at most this much (more: an overshoot the forward margin's ahead side caught) ...
MOVE_REACHED_OFF_M, MOVE_REACHED_OFF_REL = 0.15, 0.10   # ... and off its line by at most 0.15 m + 10 % of the move
                              # (a sideways-guard stop is up to 0.25-0.6 m off it: 2026-10-15 review)
PLAN_MIN_FRONT_M = 0.35       # AUTO_PLAN_MIN_FRONT_CM: the drone flies the TAKEOFF's plan with the Mac's own floor
                              # (2026-10-15 review: the Mac cannot check it before the take-off; 0.60 until 2026-10-17)
MV_MARGIN_KEYS = ("mvfloor", "mvfstop", "mvprogm", "mvprogr", "mvlatm", "mvnear", "mvfix",   # the guard margins' NVS
                  "mvklow", "mvlat", "mvwmax")   # keys and the move fixes' (2026-10-17); the guards-off keys (2026-10-18)
MVFIX_DRIFT = 0x01            # the drone's NVS mvfix bit DRIFT (auto_motion.h "move fixes", 2026-10-17): its measured D
                              # follows the drift since the last move's end (the reply's drift_since_last_end_m)
MVFIX_DEFAULT = 0x3B          # AUTO_MVFIX_DEFAULT (0x1B until the review of 2026-10-17: FRONT, 0x20, added)
PLAN_REST_WARN_V = 3.78       # sortie --plan-*: a rest voltage under this is warned on (the long move may not fit)
PLAN_STATUS_S = 1.0           # --plan-via drone: the plan's outcome shows in AUTO_STATUS within this of the wait
TAKEOFF_PLAN_LEN = 18         # AUTO_TAKEOFF_PLAN_LEN: TAKEOFF with a planned move (12 without)
PLAN_VIA_DEFAULT = "mac"      # 'drone' once the long-move build flies (an older drone refuses the 18-byte TAKEOFF)
MOTION_STATUS_MAX_AGE_S = 1.0  # a MOVE / YAW is checked against an AUTO_STATUS at most this old
PHOTO_GAP_AIR_MS = (150, 1000)
AIR_MS = (2000, 8000)         # ARM airMaxMs when not 0 (AUTO_AIR_MAX_MIN_MS..AUTO_AIR_MAX_HI_MS) ...
AIR_MS_DEFAULT = 6000         # ... and at most the drone's NVS autoair (default AUTO_AIR_MAX_DEFAULT_MS)


def name_of(table, v):
    return table[v] if 0 <= v < len(table) else f"?{v}"


def bit_names(bits, table):
    return [table[i] if i < len(table) else f"bit{i}" for i in range(32) if bits >> i & 1]


def guard_names(bits):
    return bit_names(bits, GUARD_NAMES)


G_CAMERA = 1 << GUARD_NAMES.index("CAMERA")


# G2 STILL in FLIGHT includes the accelerometer's heat soak (auto_fsm.h AUTO_SOAK_*): on the ground
# min(20 s + 1.5 x the last flight's motor time, 90 s) after a flight, still and level.
SOAK_SETTLE_S, SOAK_PER_MOTOR, SOAK_MAX_S = 20.0, 1.5, 90.0


def soak_hint(air_s, since_s):
    """The hint of an ARM refused with STILL: about how long the heat soak still needs, from the last FLIGHT
    sortie's air time (the motor time's stand-in, a second or two longer) and the time since it ended, both as
    the daemon saw them; None when the soak is over (STILL then means: not still or level) or unknown."""
    if air_s is None or since_s is None:
        return None
    need = min(SOAK_SETTLE_S + SOAK_PER_MOTOR * air_s, SOAK_MAX_S)
    left = math.ceil(need - since_s)
    return f"IMU heat soak: wait {left} s" if left > 0 else None


def busy_detail(kind, value):
    """As autoDetailBusy() (auto_proto.h): the detail of a BUSY the link answers. Built by the tests' fake drone."""
    return kind | (value & (0xFFFF if kind == DETAIL_BUSY_HELD_SET else DETAIL_VALUE_MASK))


def busy_kind(detail):
    """(kind, value) of an ACK BUSY's detail. kind: 'taken_unknown' (it may have been acted on), 'pending'
    (value: the pending command's seq, low 24 bits), 'untaken' (value: the state), 'held_set' (value: the
    held photo set's id), 'fsm' (kind 0, the state machine's own BUSY or an A1 one: value = the detail) or
    'other' (a kind this tool does not know: not processed, like every BUSY but taken-unknown)."""
    if detail == DETAIL_TAKEN_UNKNOWN:
        return "taken_unknown", None
    kind = detail & DETAIL_KIND_MASK
    if kind == 0:
        return "fsm", detail
    value = detail & (0xFFFF if kind == DETAIL_BUSY_HELD_SET else DETAIL_VALUE_MASK)
    return BUSY_KIND_NAMES.get(kind, "other"), value


def arm_payload(mode, air_ms=0):
    return struct.pack("<BBH", mode, 0, air_ms)


def takeoff_payload(token, height_cm, photo_n, gap_ms, act_wait_ms=0, plan_right_cm=0, plan_fwd_cm=0):
    """TAKEOFF (0x11). actWaitMs is u24: byte 7 (a pad until 2026-10-14) carries its bits 16-23, bytes 10-11 its
    low 16 bits; a wait <= 65535 ms is byte for byte what an older Mac sent. A planned move (2026-10-14, docs/
    autonomy-a2b.md 21.4): 18 bytes, + i16 planRightCm, i16 planFwdCm, u16 0; without one the 12 of before (an
    older drone flies it; it refuses the 18, RANGE detail 0)."""
    p = struct.pack("<IHBBHH", token, height_cm, photo_n, (act_wait_ms >> 16) & 0xFF, gap_ms, act_wait_ms & 0xFFFF)
    if plan_right_cm or plan_fwd_cm:
        p += struct.pack("<hhH", plan_right_cm, plan_fwd_cm, 0)
    return p


def wait_height_floor(wait_s):
    """The --height a sortie --wait-s is advised to take off at (advice since 2026-10-15): the estimate's drift over
    the wait and the take-off, capped at WAIT_DRIFT_CAP_S of it (not at the move-down floor's H_DRIFT_CAP_M: the
    drift itself goes on)."""
    t = wait_s + WAIT_PRE_S
    return H_FLOOR_M + H_DRIFT_MPS * max(0.0, t if WAIT_DRIFT_CAP_S is None else min(t, WAIT_DRIFT_CAP_S))


def move_payload(right_cm, fwd_cm, up_cm):
    """MOVE (0x31): + right, + forward, + up, cm (the drone's own frame at the moment it accepts it)."""
    return struct.pack("<hhhH", right_cm, fwd_cm, up_cm, 0)


def yaw_payload(deg):
    """YAW (0x30): + clockwise (to the right, as the photo bearings)."""
    return struct.pack("<hH", deg, 0)


SH_AT_NOW = -32768   # SET_HEIGHT atCm: the drone's estimate now (auto_fsm.h AUTO_SH_AT_NOW, INT16_MIN)


def set_height_payload(x_cm, at_cm=SH_AT_NOW, to_cm=None):
    """SET_HEIGHT (0x33, 2026-10-15, docs/autonomy-a2b.md 23): i16 xCm, Claude's judged true height; i16 atCm, the
    drone's own estimate when the judged photo was captured (SH_AT_NOW: the estimate now). to_cm (2026-10-16, a2b 27):
    the 6-byte form, i16 toCm, the height to fly to (SH_TO_REF: the commanded height); None: the 4-byte form (the drone
    flies back to its old estimate)."""
    if to_cm is None:
        return struct.pack("<hh", x_cm, at_cm)
    return struct.pack("<hhh", x_cm, at_cm, to_cm)


def set_height_plan(x_m, at_m, h_now_m, target_m):
    """The drone's arithmetic of the target form (auto_fsm.c setHeightCmd): the offset d = X - h_at (at_m None: X -
    h_now), the true height now h_now + d, the flight target - (h_now + d). Rounded to mm."""
    d = x_m - (h_now_m if at_m is None else at_m)
    true_now = h_now_m + d
    return {"offset_m": round(d, 3), "true_now_m": round(true_now, 3), "target_m": round(target_m, 3),
            "fly_m": round(target_m - true_now, 3)}


def set_height_legacy_x(x_m, at_m, h_now_m, target_m):
    """The 4-byte form's X that lands a drone without the target form (7a78e1b, 8277b68) at the target: it flies back
    to its old estimate h_now, so its true end is h_now + X - x (at h_at); x = X + h_now - target (the est-rule of
    2026-10-16 with the estimate now). Its estimate then reads h_now - target off (the take-off's ~-0.1 m)."""
    return x_m + h_now_m - target_m


def late_climb_warning(a, up_m):
    """A climb late in the battery (LATE_CLIMB_*): a warning text, or None (never a refusal)."""
    t, v = (a or {}).get("tHold"), (a or {}).get("vbat_v")
    if not (up_m > 0 and t and v and t >= LATE_CLIMB_T and v < LATE_CLIMB_V):
        return None
    return (f"late in the battery: T_hold {t} >= {LATE_CLIMB_T} and {v:.2f} V < {LATE_CLIMB_V:.2f} V under load. The "
            "drone allows the climb (thrust headroom left), but F2 (10-16) sank at the 58000 cap 2.4 s after a climb "
            "like this, while fastcheck (10-15) held 16 s after one: climb only if the photo needs it, then plan the "
            "landing")


def events_since(a, ev_capture):
    """The events the drone counted since a photo's capture (its evCount then), newest first: [] if none or unknown."""
    if not a or not a.get("hr") or ev_capture is None:
        return []
    n = (a["evCount"] - int(ev_capture)) & 0xFFFF
    if n == 0 or n > 0x8000:
        return []
    names = [name_of(EVENT_NAMES, a[c]) for c in ("evCode", "ev1Code", "ev2Code") if a.get(c)]
    return names[:min(n, 3)] + ([f"+{n - 3} more"] if n > 3 else [])


def auto_status_payload(fields, length=AUTO_STATUS_LEN):
    """Built only by the tests' fake drone; the Mac parses it. length AUTO_STATUS_LEN_A2B: an A2b drone's;
    AUTO_STATUS_LEN_IH: an interactive-hover drone's (before 2026-10-15); AUTO_STATUS_LEN_HR: a hold-and-report drone's
    (before the speed step, 2026-10-16); mvSpeedPct defaults to 100 in the 140 bytes."""
    p = struct.pack(AUTO_STATUS_FMT, *(fields.get(k, 0) for k in AUTO_STATUS_FIELDS))
    if length >= AUTO_STATUS_LEN_IH:
        p += struct.pack(AUTO_STATUS_EXT_FMT, *(fields.get(k, 0) for k in AUTO_STATUS_EXT_FIELDS))
    if length >= AUTO_STATUS_LEN_HR:
        p += struct.pack(AUTO_STATUS_HR_FMT, *(fields.get(k, 0) for k in AUTO_STATUS_HR_FIELDS))
    if length >= AUTO_STATUS_LEN:
        p += struct.pack(AUTO_STATUS_MS_FMT, fields.get("mvSpeedPct", MV_SPEED_DEFAULT), fields.get("reservedMs", 0))
    return p


def mv_detail(detail):
    """An ACK detail of MOVE / YAW / STOP (auto_fsm.c mvDetail): (state, code, motion seq); code is the
    refusal (autoMvWhy_t) of a MOVE or YAW, the motion state a STOP found."""
    return detail & 0xFF, (detail >> 8) & 0xFF, detail >> 16


def fwd_end_low(d, f, k=K_LOW):
    """Where a forward move of f m from a scene d m ahead truly ends at the image expansion scale k: the hold
    drives the measured k ln(D0 / D) to ln(d / (d - f)), so the end is d ((d - f) / d)^(1/k) (k < 1, a floor in
    view: shorter than d - f). A back move (f <= 0) ends no closer than d."""
    if f <= 0.0:
        return d
    if f >= d:
        return 0.0
    return d * ((d - f) / d) ** (1.0 / k)


def fwd_max(d, floor, k=K_LOW, top=None):
    """The longest forward move (m, whole cm, <= top: MOVE_FWD_MAX_M) that ends at least floor m from the scene d m
    ahead at k."""
    top = MOVE_FWD_MAX_M if top is None else top
    for fc in range(int(round(top * 100)), 0, -1):
        if d - fc / 100.0 >= MOVE_D_FLOOR_M - 1e-9 and fwd_end_low(d, fc / 100.0, k) >= floor:
            return fc / 100.0   # (the drone's own 0.05 m: what is left of the floor at mvfloor 0)
    return 0.0


def fwd_overshoot(d, f, k=K_LOW):
    """How much further than f m a forward move from a scene d m ahead may truly fly at the image expansion scale k
    (the drone's own model, fwd_end_low): the travel d - d ((d - f) / d)^(1/k) minus f; 0 for f <= 0."""
    if f <= 0.0 or d is None:
        return 0.0
    return max(0.0, d - fwd_end_low(d, f, k) - f)


PERSON_FOV_HALF_DEG = 38.0    # the person Claude judges is in the photo: within half the camera's field of view


def toward_person_max(r, f, half_deg=PERSON_FOV_HALF_DEG):
    """The most a displacement (r right, f forward, m) can bring the drone nearer a person seen within +-half_deg of
    the heading: the largest r sin b + f cos b over those bearings b (its length when it points inside them)."""
    h = math.radians(half_deg)
    phi = math.atan2(r, f)
    if -h <= phi <= h:
        return math.hypot(r, f)
    return max(r * math.sin(b) + f * math.cos(b) for b in (-h, h))


def person_speed_term(d, r, f, pct=MV_SPEED_DEFAULT, caps=None):
    """Above 100 % (fast moves): PERSON_SPEED_T x the move's forward peak speed (the hold leads the reference by up to
    ~0.4 s of it at speed), m; 0 at 100 (its 0.20 m/s lead is inside the k overshoot's margin, as before)."""
    if f <= 0.0 or pct is None or pct <= MV_SPEED_DEFAULT:
        return 0.0
    length = math.hypot(r, f)
    return PERSON_SPEED_T * move_profile(r, f, 0.0, d, pct, caps)[1] * f / length


def person_gap_low(d, r, f, gap, est, pct=MV_SPEED_DEFAULT, caps=None):
    """The end distance to the person at its lowest (2026-10-15 review): Claude's planned end gap less the forward
    overshoot at K_LOW, and its distance to the person now (est) less the most the move with that overshoot can bring
    the drone nearer a person in the photo (toward_person_max; the sideways part never comes out long toward the
    scene). Above 100 % the overshoot also has the speed term (person_speed_term). (lowest, overshoot)."""
    over = fwd_overshoot(d, f) + person_speed_term(d, r, f, pct, caps)
    return min(gap - over, est - max(0.0, toward_person_max(r, f + over))), over


def person_fwd_max(d, r, f, gap, est, pct=MV_SPEED_DEFAULT, caps=None):
    """The longest forward part (m, whole cm, <= f) whose person_gap_low keeps PERSON_MIN_M, Claude's gap growing by
    what the move gives up (the person ahead along it)."""
    for fc in range(int(round(f * 100)), -1, -1):
        if person_gap_low(d, r, fc / 100.0, gap + (f - fc / 100.0), est, pct, caps)[0] >= PERSON_MIN_M - 1e-9:
            return fc / 100.0
    return 0.0


def move_reached(a_end, travelled, target):
    """A MOVE's end against its target (2026-10-15 review): reached when it ended DONE, SHORT or LONG, or PROGRESS with
    MOVE_REACHED_FRAC..MOVE_REACHED_MAX_FRAC of it measured along it and off its line by at most MOVE_REACHED_OFF_M +
    MOVE_REACHED_OFF_REL x its length (the guard tripped at its end). The fields for the reply: along, off_m, and for a
    PROGRESS stop reached (with a note) or the error (an overshoot, off the line, or short); for a normal end, or an
    early reply before the settle (a_end None), a warning when the measured travel is outside those bounds (2026-10-17:
    the 300 % flight's DONE 0.2-0.45 m past its target read as reached)."""
    out = {}
    if not target:
        return out
    along = move_along(travelled, target)
    if along is None:
        return out
    out["along"] = round(along, 3)
    n = math.sqrt(sum(v * v for v in target.values()))
    off = math.sqrt(sum((travelled.get(k, 0.0) - along * target.get(k, 0.0)) ** 2 for k in ("right", "fwd", "up")))
    out["off_m"] = round(off, 3)
    if a_end in MVE_OK or a_end is None:
        # a normal end (or an early reply, a_end None: before the settle): the measured travel against the target
        # (2026-10-17, the 300 % forensics: a DONE 0.2-0.45 m past it was reported as reached)
        warn = []
        if along > MOVE_REACHED_MAX_FRAC:
            warn.append(f"past the target by {(along - 1.0) * n:.2f} m along the move ({100 * along:.0f} %)")
        elif along < MOVE_REACHED_FRAC:
            warn.append(f"short of the target by {(1.0 - along) * n:.2f} m along the move ({100 * along:.0f} %)")
        if off > MOVE_REACHED_OFF_M + MOVE_REACHED_OFF_REL * n:
            warn.append(f"{off:.2f} m off its line")
        if warn:
            out["warning"] = ("as the drone measured it" + (" before its settle" if a_end is None else "") + ": " +
                              "; ".join(warn))
        return out
    if a_end != MVE_PROGRESS or along < MOVE_REACHED_FRAC:
        return out
    if along > MOVE_REACHED_MAX_FRAC:
        out["error"] = (f"the MOVE ended PROGRESS past its end: overshoot by {(along - 1.0) * n:.2f} m ({100 * along:.0f} "
                        "% of it measured along it): the drone holds there, nearer what is ahead than planned; take a "
                        "photo")
    elif off > MOVE_REACHED_OFF_M + MOVE_REACHED_OFF_REL * n:
        out["error"] = (f"the MOVE ended PROGRESS off its line by {off:.2f} m ({100 * along:.0f} % along it): the "
                        "drone holds there; take a photo")
    else:
        out.update(reached=True, note=f"reached (stopped at the end by the guard: {100 * along:.0f} % of the move "
                                      f"measured along it, {off:.2f} m off its line)")
    return out


def move_range_refusal(rc, fc, uc, fwd_top=None):
    """Why a MOVE of (rc, fc, uc) cm is out of the drone's command limits (autoMvRangeOk), or None."""
    fwd_top = MOVE_FWD_MAX_M if fwd_top is None else fwd_top
    if abs(rc) > MOVE_HORIZ_MAX_M * 100 or fc > fwd_top * 100 + 1e-6 or fc < -MOVE_HORIZ_MAX_M * 100 or \
            abs(uc) > MOVE_UP_MAX_M * 100:
        return (f"out of range: |right| <= {MOVE_HORIZ_MAX_M:.2f} m, forward <= {fwd_top:.2f} m, back <= "
                f"{MOVE_HORIZ_MAX_M:.2f} m, |up| <= {MOVE_UP_MAX_M:.2f} m per MOVE")
    if rc * rc + fc * fc > (MOVE_LEN_MAX_M * 100) ** 2 + 1e-6:
        return (f"out of range: the move's length {math.hypot(rc, fc) / 100.0:.3f} m is over {MOVE_LEN_MAX_M:.2f} m "
                "(sqrt(right^2 + forward^2))")
    if uc and fc > MOVE_HORIZ_MAX_M * 100:
        return (f"out of range: a move up or down goes at most {MOVE_HORIZ_MAX_M:.2f} m forward (the camera reads "
                "vertical motion partly as forward): split it")
    return None


def _f32(x):
    """x rounded to a C float (the drone's arithmetic for the step's integer limits)."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def mv_limits(pct=MV_SPEED_DEFAULT, caps=None):
    """The limits of the speed step pct (auto_motion.c autoMvSpeedCfg): at 100 (and below) the header constants, above
    each one x s = pct / 100 up to its cap (MV_FAST_CAPS; A_H's the drone's mvacc), the settle and the yaw dwell / s
    down to their floors (the settle's the drone's mvsettle), the progress margins' and the person rule's tracking time,
    the frozen-view reference. pct above 300: 300. caps: {"mvacc": cm/s^2, "mvsettle": ms} (fast_caps(); None: the
    defaults, MV_FAST_DEFAULT; each clamped to its range as the drone's autoMotionSetFastCaps)."""
    acc, settle_floor, w_fwd = fast_caps_values(caps)
    pct = MV_SPEED_DEFAULT if pct is None else min(int(pct), MV_SPEED_PCT[1])
    lim = {"pct": MV_SPEED_DEFAULT, "fast": False, "v_h": MOVE_V_H_MAX, "v_back": MOVE_V_H_MAX, "v_fwd": MOVE_V_FWD_MAX,
           "a_h": MOVE_A_H, "w_lat": MOVE_W_LAT_MAX, "w_fwd": w_fwd, "brake": MOVE_BRAKE_ACC,
           "settle_ms": MOVE_SETTLE_MS, "dwell_ms": YAW_DWELL_MS, "yaw_rate": YAW_RATE_DPS, "yaw_acc": YAW_ACC_DPS2,
           "bob_acc": BOB_ACC, "t_track": 0.0, "frozen_ref": FROZEN_REF}
    if pct <= MV_SPEED_DEFAULT:
        return lim
    s = _f32(pct * _f32(0.01))
    base = {"v_h": MOVE_V_H_MAX, "v_back": MOVE_V_H_MAX, "v_fwd": MOVE_V_FWD_MAX, "a_h": MOVE_A_H,
            "w_lat": MOVE_W_LAT_MAX, "w_fwd": w_fwd, "brake": MOVE_BRAKE_ACC, "yaw_rate": YAW_RATE_DPS,
            "yaw_acc": YAW_ACC_DPS2, "bob_acc": BOB_ACC}
    # (mvwmax over the fast cap 0.17 is itself the cap, as the drone's autoMvSpeedCfg)
    fcaps = dict(MV_FAST_CAPS, a_h=acc / 100.0, w_fwd=max(MV_FAST_CAPS["w_fwd"], w_fwd))
    lim.update({k: min(v * s, fcaps[k]) for k, v in base.items()})
    lim.update(pct=pct, fast=True, t_track=MV_FAST_T_TRACK_S, frozen_ref=FROZEN_REF + MV_FAST_T_TRACK_S * lim["w_fwd"],
               settle_ms=max(settle_floor, int(_f32(_f32(MOVE_SETTLE_MS / s) + 0.5))),
               dwell_ms=max(MV_FAST_DWELL_MIN_MS, int(_f32(_f32(YAW_DWELL_MS / s) + 0.5))))
    return lim


def fast_caps_values(caps):
    """(mvacc cm/s^2, mvsettle ms, the expansion rate cap /s of mvwmax) of caps (None: the defaults), each clamped to
    its range (the drone's autoMotionSetFastCaps, autoMotionSetFwdRate)."""
    caps = caps or MV_FAST_DEFAULT
    acc = int(caps.get("mvacc", MV_ACC_DEFAULT_CM))
    settle = int(caps.get("mvsettle", MV_FAST_SETTLE_MIN_MS))
    w = int(caps.get("mvwmax", MV_WFWD_DEFAULT_PCT))
    return (min(max(acc, MV_ACC_CM[0]), MV_ACC_CM[1]),
            min(max(settle, MV_SETTLE_FLOOR_MS[0]), MV_SETTLE_FLOOR_MS[1]),
            min(max(w, MV_WFWD_PCT[0]), MV_WFWD_PCT[1]) / 100.0)


def settle_s(pct=MV_SPEED_DEFAULT, caps=None):
    """A MOVE's settle at most (s): AUTO_MV_SETTLE_MAX_MS at 100, above 3.0 / s down to the drone's mvsettle (2.5 s by
    default; 1.0 s at 'real 300 %')."""
    return mv_limits(pct, caps)["settle_ms"] / 1000.0


def _trap(dist, v, a):
    """auto_motion.c autoMvProfPlan: (T, peak speed) of a trapezoid (a triangle when short)."""
    if dist <= 0 or v <= 0 or a <= 0:
        return 0.0, 0.0
    t_a = v / a
    if a * t_a * t_a >= dist:
        t_a = math.sqrt(dist / a)
        return 2.0 * t_a, a * t_a
    return 2.0 * t_a + (dist - v * t_a) / v, v


def fwd_peak_cap(d, f, a, w):
    """auto_motion.c autoMvFwdPeakCap: the largest peak speed of a straight approach of f m (acceleration a) toward a
    scene d m ahead whose expansion v / (d - s) stays <= w along the whole trapezoid (inf: none reaches w)."""
    de = max(d - max(f, 0.0), 0.05)
    if a <= 0 or w <= 0:
        return math.inf
    disc = 1.0 - 2.0 * w * w * de / a
    if disc <= 0:
        return math.inf
    return (a / w) * (1.0 - math.sqrt(disc))


def move_profile(right, fwd, up, d, pct=MV_SPEED_DEFAULT, caps=None):
    """The drone's profile of a MOVE (auto_motion.c moveFloor at the speed step pct): (time s, horizontal peak m/s).
    One trapezoid along the horizontal vector at A_H, its speed the least of V_H, the sideways image rate at the
    forward end (W_LAT x max(D - F+, 0.05) x len / |R|), V_FWD for the part toward the scene, and the expansion rate:
    at 100 W_FWD x the end distance (back moves: the start D), above 100 the true peak of v / (D - s) along the
    profile (fwd_peak_cap) and V_BACK for the part away from it; the vertical one at 0.15 m/s, 0.20 m/s^2. d None
    (before take-off): no image-rate cap (the scene far). caps: the drone's mvacc / mvsettle (mv_limits)."""
    lim = mv_limits(pct, caps)
    length = math.hypot(right, fwd)
    v = lim["v_h"]
    if length > 0:
        d_end = max(d - max(fwd, 0.0), 0.05) if d else 99.0
        if right:
            v = min(v, lim["w_lat"] * d_end * length / abs(right))
        if fwd and not (lim["fast"] and fwd > 0):
            v = min(v, lim["w_fwd"] * d_end * length / abs(fwd))
        if fwd > 0:
            v = min(v, lim["v_fwd"] * length / fwd)
        if lim["fast"] and fwd > 0 and d:
            v = min(v, fwd_peak_cap(d, fwd, lim["a_h"] * fwd / length, lim["w_fwd"]) * length / fwd)
        if lim["fast"] and fwd < 0:
            v = min(v, lim["v_back"] * length / -fwd)
    t_h, v_pk = _trap(length, v, lim["a_h"])
    return max(t_h, _trap(abs(up), 0.15, 0.20)[0]), v_pk


def move_profile_s(right, fwd, up, d, pct=MV_SPEED_DEFAULT, caps=None):
    """move_profile's time (s)."""
    return move_profile(right, fwd, up, d, pct, caps)[0]


def yaw_profile_s(deg, pct=MV_SPEED_DEFAULT):
    """A YAW's turn (s) at the step's rate and acceleration (its dwell and re-track bob come after it)."""
    lim = mv_limits(pct)
    return _trap(abs(deg), lim["yaw_rate"], lim["yaw_acc"])[0]


def bob_cycle_s(pct=MV_SPEED_DEFAULT):
    """One re-track (or re-view) bob cycle (s): two triangle legs of BOB_A_M at the step's bob acceleration."""
    return 2.0 * _trap(BOB_A_M, 0.15, mv_limits(pct)["bob_acc"])[0]


def plan_cm(req):
    """The sortie's planned move (right, forward) in cm from its request (plan_right / plan_fwd, metres), range-checked
    against the drone's command limits (ValueError: nothing is sent)."""
    pr, pf = (int(round(float(req.get(k) or 0.0) * 100)) for k in ("plan_right", "plan_fwd"))
    if pr or pf:
        why = move_range_refusal(pr, pf, 0)
        if why:
            raise ValueError("the planned move: " + why)
        if req.get("plan_via", PLAN_VIA_DEFAULT) not in ("drone", "mac"):
            raise ValueError("plan_via must be drone or mac")
    return pr, pf


def move_along(travelled, target):
    """The fraction of a move measured along it (travelled . target / |target|^2), or None without a horizontal
    target."""
    if not travelled or not target:
        return None
    n2 = target.get("right", 0.0) ** 2 + target.get("fwd", 0.0) ** 2 + target.get("up", 0.0) ** 2
    if n2 <= 0:
        return None
    return (travelled.get("right", 0.0) * target.get("right", 0.0) + travelled.get("fwd", 0.0) * target.get("fwd", 0.0)
            + travelled.get("up", 0.0) * target.get("up", 0.0)) / n2


def batt_seconds_left(vbat):
    """Seconds of hover the battery has left by its fall under load (BATT_DUR_*), or None above BATT_DUR_FROM_V."""
    if vbat is None or vbat >= BATT_DUR_FROM_V:
        return None
    return max(0.0, (vbat - BATT_END_V) / BATT_DUR_VPS)


def height_floor(air_s):
    """The lowest commanded height a move down may leave the drone at: the height estimate drifts (logged up to
    0.01 m/s, reading high), so late in a sortie the drone may be lower than it thinks; the drift term at most
    H_DRIFT_CAP_M (the drone's AUTO_MV_H_DRIFT_CAP_M, 2026-10-15: uncapped it reached 1.20 m at 95 s)."""
    return H_FLOOR_M + min(H_DRIFT_CAP_M, H_DRIFT_MPS * max(0.0, air_s or 0.0))


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def parse_auto_status(p):
    """AUTO_STATUS (0x84) -> its fields plus decoded names. 140 bytes (the speed step, 2026-10-16), a hold-and-report
    drone's 136 (mvSpeedPct then 100: it flies the limits of before), an interactive-hover drone's 96 (the report
    fields then 0 and 'hr' False) or an A2b drone's 64 (the interactive fields too, 'interactive' False)."""
    if len(p) not in (AUTO_STATUS_LEN_A2B, AUTO_STATUS_LEN_IH, AUTO_STATUS_LEN_HR, AUTO_STATUS_LEN):
        raise ProtoError("payload", f"AUTO_STATUS payload is {len(p)} bytes, want {AUTO_STATUS_LEN} (or an older "
                                    f"drone's {AUTO_STATUS_LEN_HR}, {AUTO_STATUS_LEN_IH} or {AUTO_STATUS_LEN_A2B})")
    a = dict(zip(AUTO_STATUS_FIELDS, struct.unpack(AUTO_STATUS_FMT, p[:AUTO_STATUS_LEN_A2B])))
    ext = len(p) >= AUTO_STATUS_LEN_IH
    a.update(zip(AUTO_STATUS_EXT_FIELDS, struct.unpack(AUTO_STATUS_EXT_FMT, p[AUTO_STATUS_LEN_A2B:AUTO_STATUS_LEN_IH])
                 if ext else (0,) * len(AUTO_STATUS_EXT_FIELDS)))
    hr = len(p) >= AUTO_STATUS_LEN_HR
    a.update(zip(AUTO_STATUS_HR_FIELDS, struct.unpack(AUTO_STATUS_HR_FMT, p[AUTO_STATUS_LEN_IH:AUTO_STATUS_LEN_HR])
                 if hr else (0,) * len(AUTO_STATUS_HR_FIELDS)))
    ms = len(p) >= AUTO_STATUS_LEN
    a.update(zip(AUTO_STATUS_MS_FIELDS, struct.unpack(AUTO_STATUS_MS_FMT, p[AUTO_STATUS_LEN_HR:AUTO_STATUS_LEN])
                 if ms else (0, 0)))
    # an older drone (no speed step) flies 100; a 140-byte drone reports 100..300 (0 would be a bad status: 100)
    a["mvSpeedPct"] = min(max(a["mvSpeedPct"], MV_SPEED_DEFAULT), MV_SPEED_PCT[1]) if ms else MV_SPEED_DEFAULT
    a["interactive"] = ext
    a["hr"] = hr
    a["speedStep"] = ms
    a["stateName"] = name_of(STATE_NAMES, a["state"])
    a["modeName"] = name_of(MODE_NAMES, a["mode"])
    for k in ("reason", "landTrip"):
        a[k + "Name"] = name_of(REASON_NAMES, a[k])
    for k in ("vzLandReason", "vzReason"):
        a[k + "Name"] = name_of(VZ_REASON_NAMES, a[k])
    a["vzPhaseName"] = name_of(VZ_PHASE_NAMES, a["vzPhase"])
    a["captureName"] = name_of(CAPTURE_NAMES, a["capture"])
    a["lastCmdName"] = name_of(CMD_NAMES, a["lastCmdType"])
    a["lastCmdResultName"] = result_name(a["lastCmdResult"])
    a["guards"] = guard_names(a["failedGuards"])
    a["flagNames"] = bit_names(a["flags"], STF_NAMES)
    a["tripNames"] = bit_names(a["trips"], TRF_NAMES)
    a["waiting"] = bool(a["flags"] & STF_WAITING)
    a["waitLeftSaturated"] = a["waiting"] and a["waitLeftMs"] >= WAIT_SAT_MS   # more than 65.5 s left
    a["vbat_v"] = a["vbatMv"] / 1000.0
    a["h_m"] = a["hMm"] / 1000.0
    a["vz_ms"] = a["vzMms"] / 1000.0
    a["mvStateName"] = name_of(MV_STATE_NAMES, a["mvState"])
    a["mvKindName"] = name_of(MV_KIND_NAMES, a["mvKind"])
    a["mvWhyName"] = name_of(MV_WHY_NAMES, a["mvWhy"])
    a["mvEndName"] = name_of(MV_END_NAMES, a["mvEnd"])
    # mvFlags 0x40 means a blind turn on a YAW, a SET_HEIGHT's correction on a MOVE (auto_motion.h, 2026-10-15)
    a["mvCorrection"] = a["mvKind"] == MV_KIND_NAMES.index("MOVE") and bool(a["mvFlags"] & MVF_SET_HEIGHT)
    a["mvFlagNames"] = [("setHeight" if n == "blindTurn" and a["mvCorrection"] else n)
                        for n in bit_names(a["mvFlags"], MVF_NAMES)]
    a["mvActive"] = a["mvState"] in MV_ACTIVE
    a["battLowNow"] = bool(a["battLow"] & BATTLOW_NOW)     # BATT_LOW condition: reading < 3.00 V or T_hold > 57,000
    a["battLow"] = bool(a["battLow"] & BATTLOW_HELD)       # 0.25 s of it: MOVE / YAW refused (0.5 s: it lands)
    a["sceneD_m"] = a["sceneDCm"] / 100.0 if a["mvFlags"] & MVF_D_VALID else None
    a["hTarget_m"] = a["hTargetCm"] / 100.0
    a["yawOff_deg"] = a["yawOffCdeg"] / 100.0
    a["prog_m"] = {k: a["prog" + k.capitalize() + "Cm"] / 100.0 for k in ("right", "fwd", "up")}
    a["tgt_m"] = {k: a["tgt" + k.capitalize() + "Cm"] / 100.0 for k in ("right", "fwd", "up")}
    a["airLeft_s"] = a["airLeftDs"] / 10.0
    # The air time: airMs while it is exact (u16 ms; it saturates at 65.5 s), then airDs (0.1 s; the 136-byte
    # drone has no air clock and airLeftDs 0). A 96-byte drone (2026-10-14) had the 120 s clock: AIR_HARD_MS minus
    # its airLeftDs, which counts down to its limit; unsaturated, airMs + 100 x airLeftDs is that limit, and a
    # drone of another version (another limit) is flagged (airClockMismatch).
    sat = a["airMs"] >= 65535
    if a["airMs"] and not sat:
        a["air_s"] = a["airMs"] / 1000.0
    elif hr:
        a["air_s"] = a["airDs"] / 10.0
    elif a["airLeftDs"]:
        a["air_s"] = (AIR_HARD_MS - 100 * a["airLeftDs"]) / 1000.0
    elif sat:
        a["air_s"] = AIR_HARD_MS / 1000.0
    else:
        a["air_s"] = 0.0
    a["airClockMismatch"] = not hr and bool(a["airLeftDs"]) and 0 < a["airMs"] < 65535 and \
        abs(a["airMs"] + 100 * a["airLeftDs"] - AIR_HARD_MS) > AIR_CLOCK_TOL_MS
    if hr:
        a["hold"] = hold_view(a)
    return a


def event_view(code, detail, age_ms):
    """One reported event of AUTO_STATUS bytes 96-135, decoded (detailName: the cause's own name)."""
    name = name_of(EVENT_NAMES, code)
    e = {"code": code, "name": name, "detail": detail, "age_ms": age_ms}
    if code == EV_VZ:
        e["detailName"] = name_of(VZ_REASON_NAMES, detail)
    elif code in (EV_MOVE_STOPPED, EV_RETRACK_UNDO):
        e["detailName"] = name_of(MV_END_NAMES, detail)
    elif code == EV_CAM_LOST:
        e["detailName"] = bit_names(detail, CB_NAMES)
    elif code == EV_CONTACT:
        e["detailName"] = {"phase": name_of(VZ_PHASE_NAMES, detail & 0x3F), "noHush": bool(detail & CONTACT_NO_HUSH),
                           "motionStopped": bool(detail & CONTACT_MOTION)}
    elif code == EV_NOT_STABLE:
        e["detailName"] = bit_names(detail, NS_NAMES)
        e["failed"] = [NS_HINTS[n] for n in e["detailName"] if n in NS_HINTS]
    elif code == EV_SINK_CODE:
        e["detailName"] = name_of(SINK_WHY, detail)
        if detail < len(SINK_WHY_HINTS):
            e["why"] = SINK_WHY_HINTS[detail]
    elif code == EV_HOLD_NO_VIEW:
        e["detailName"] = name_of(VIEW_WHY, detail)
    elif code == EV_HOLD_LIMIT:
        # +roll (right side down) holds against a drift to the left, +pitch (nose up) against one forward
        axes = []
        if detail & HL_ROLL:
            neg = bool(detail & HL_ROLL_NEG)
            axes.append({"axis": "roll", "sign": "-" if neg else "+", "creeps": "right" if neg else "left"})
        if detail & HL_PITCH:
            neg = bool(detail & HL_PITCH_NEG)
            axes.append({"axis": "pitch", "sign": "-" if neg else "+", "creeps": "back" if neg else "forward"})
        e["detailName"] = axes
    elif code == EV_LOOP_GAP:
        e["gap_ms"] = detail
        e["detailName"] = f"{detail} ms" if detail < 255 else ">= 255 ms"
    hint = EVENT_HINTS.get(name)
    if hint:
        e["hint"] = hint
    return e


def hold_view(a):
    """AUTO_STATUS bytes 96-135 (a 136-byte drone): what the drone did on its own and what it holds, for Claude.
    The obstacle's bearing: the contact's body-frame force points away from what was hit; its bearing at the
    contact (0 ahead, +90 right) is atan2(Ay, -Ax), and now, the heading having turned by yawOff - contactYaw
    (clockwise positive), it is that minus the turn."""
    events = [event_view(a[c], a[d], a[g]) for c, d, g in (("evCode", "evDetail", "evAgeMs"),
                                                          ("ev1Code", "ev1Detail", "ev1AgeMs"),
                                                          ("ev2Code", "ev2Detail", "ev2AgeMs")) if a[c]]
    h = {"events": events, "evCount": a["evCount"], "bits": bit_names(a["holdBits"], HB_NAMES),
         "camBits": bit_names(a["camBits"], CB_NAMES), "contacts": a["contacts"], "loopGaps": a["loopGaps"],
         "khatRel": a["khatPm"] / 1000.0, "capQ": a["capQ"],
         "sinceCmd_s": None if a["sinceCmdDs"] == 65535 else a["sinceCmdDs"] / 10.0}
    if a["holdBits"] & 0x01:   # AUTO_HB_CAM_LOST
        h["camOff_s"] = a["camOffMs"] / 1000.0
        h["sinkIn_s"] = a["sinkInMs"] / 1000.0
    if a["contacts"]:
        ax, ay = a["contactAxCg"] / 100.0, a["contactAyCg"] / 100.0
        at = math.degrees(math.atan2(ay, -ax)) if (ax or ay) else None
        turned = (a["yawOffCdeg"] - a["contactYawCdeg"]) / 100.0
        h["contact"] = {"peakAcc_g": a["peakAccCg"] / 100.0, "peakRate_dps": a["peakRateDps"], "force_g": [ax, ay],
                        "bearing_at_contact_deg": None if at is None else round(at, 1),
                        "obstacle_bearing_deg": None if at is None else round(wrap180(at - turned), 1),
                        "turned_since_deg": round(turned, 1)}
    return h


# ---------------------------------------------------------------- frames

FRAME_W, FRAME_H = 160, 120
FRAME_SIZE = FRAME_W * FRAME_H
CHUNKS_MAX = 8          # CHUNKS count limit


def chunk_count(size, chunk_size=CHUNK_DATA_MAX):
    return (size + chunk_size - 1) // chunk_size


def missing_runs(missing, max_count=CHUNKS_MAX):
    """Group missing chunk indices into (first, count) requests of at most max_count
    chunks; a request may span chunks already held when that saves a request."""
    m = sorted(set(missing))
    runs = []
    i = 0
    while i < len(m):
        first = m[i]
        j = i
        while j + 1 < len(m) and m[j + 1] < first + max_count:
            j += 1
        runs.append((first, m[j] - first + 1))
        i = j + 1
    return runs


class FrameError(Exception):
    """A frame that cannot be used; final when fetching it again cannot help."""

    def __init__(self, msg, final=False):
        super().__init__(msg)
        self.final = final


class Frame:
    """One frame of a photo set being reassembled: its META and the chunks seen so
    far, in any order (repeats ignored)."""

    def __init__(self):
        self.meta = None
        self.chunks = {}
        self.repeats = 0

    def add_meta(self, meta):
        if self.meta is None:
            self.meta = meta
        elif meta["crc32"] != self.meta["crc32"] or meta["size"] != self.meta["size"]:
            log(f"frame {meta['idx']}: a second META disagrees with the first; keeping the first")

    def add_chunk(self, chunk_idx, data):
        if self.meta is not None and chunk_idx >= self.meta["chunkCount"]:
            return False
        if chunk_idx in self.chunks:
            self.repeats += 1
            return False
        self.chunks[chunk_idx] = bytes(data)
        return True

    @property
    def count(self):
        """Chunk count (from META, or the 160x120 default until it arrives)."""
        return self.meta["chunkCount"] if self.meta is not None else chunk_count(FRAME_SIZE)

    def missing(self):
        return [i for i in range(self.count) if i not in self.chunks]

    def seen(self):
        return self.meta is not None or bool(self.chunks)

    def complete(self):
        """META and every chunk it announces (a META of size 0 announces none)."""
        return self.meta is not None and not self.missing()

    def assemble(self):
        """The whole frame, checked against META (geometry, chunk sizes, size, CRC-32)."""
        m = self.meta
        if m is None:
            raise FrameError("no META")
        size, cs, cc = m["size"], m["chunkSize"], m["chunkCount"]
        if size == 0:
            raise FrameError("the drone captured no frame (META size 0: camera stalled, restarting or asleep)",
                             final=True)
        bpp = PIXFMT_BPP.get(m.get("pixFmt", PIXFMT_GRAY8))
        jpeg = m.get("pixFmt") == PIXFMT_JPEG
        if bpp is None and not jpeg:
            raise FrameError(f"unknown pixel format {m['pixFmt']} (a newer drone: update this tool, or set the "
                             f"drone's photocol to 0)", final=True)
        geometry_bad = ((m["width"], m["height"]) not in ((320, 240), (640, 480)) or size > 2 * m["width"] * m["height"]) if jpeg else m["width"] * m["height"] * bpp != size
        if geometry_bad or not 0 < cs <= CHUNK_DATA_MAX or cc != chunk_count(size, cs):
            raise FrameError(f"inconsistent META: {m['width']}x{m['height']} {m.get('pixFmtName', 'gray8')} "
                             f"size={size} chunkSize={cs} chunkCount={cc}", final=True)
        parts = []
        for i in range(cc):
            want = min(cs, size - i * cs)
            d = self.chunks.get(i)
            if d is None:
                raise FrameError(f"chunk {i} missing")
            if len(d) != want:
                raise FrameError(f"chunk {i} is {len(d)} bytes, want {want}")
            parts.append(d)
        data = b"".join(parts)
        crc = zlib.crc32(data)
        if crc != m["crc32"]:
            raise FrameError(f"CRC-32 {crc:08x} does not match META {m['crc32']:08x}")
        return data


# ---------------------------------------------------------------- bearings and images

FOCAL_PX = 102.4        # camera_flow.c FLOW_KX_GZ, horizontal, at 160x120
CENTER_X = 80.0
TICK_DEG = (-30, -20, -10, 0, 10, 20, 30)
X4_SCALE = 4
TICK_LEN = 16           # x4 pixels, at the top and bottom edges
TICK_W = 2
TICK_RGB = (255, 210, 0)
TICK_CENTER_RGB = (255, 60, 60)
FLIP_VERTICAL = True    # the camera's frames arrive upside down (not mirrored); saved images are flipped back
OUTLINE_RGB = (0, 0, 0)  # colour photos: the ticks' and the level line's dark edge (readable on any colour)


def bearing_x(deg):
    """Pixel column (160x120 image) at a bearing; positive is right."""
    return CENTER_X + FOCAL_PX * math.tan(math.radians(deg))


def bearing_deg(x):
    return math.degrees(math.atan((x - CENTER_X) / FOCAL_PX))


def x4_of(x, scale=X4_SCALE):
    """Position in the upscaled image of column x: the centre of its scale columns."""
    return scale * x + (scale - 1) / 2.0


def tick_left(x, scale=X4_SCALE):
    """Left column of the TICK_W-wide tick centred on column x."""
    return int(math.floor(x4_of(x, scale) - (TICK_W - 1) / 2.0))


def tick_label(deg):
    return "0" if deg == 0 else f"{deg:+d}"


def bearings_info(scale=X4_SCALE):
    return {
        "model": "bearing_deg = degrees(atan((x - 80) / 102.4)); x = pixel column of the 160x120 image, "
                 "positive = right of the camera axis",
        "focal_px": FOCAL_PX, "center_x": CENTER_X,
        "x4_scale": scale, "x4_of_x": f"{scale}*x + {(scale - 1) / 2:g} (centre of the upscaled columns)",
        "edge_deg": [round(bearing_deg(0), 2), round(bearing_deg(FRAME_W - 1), 2)],
        "ticks": [{"deg": d, "x": round(bearing_x(d), 2), "x4": round(x4_of(bearing_x(d), scale), 2)}
                  for d in TICK_DEG],
    }


def render_x4(img, scale=X4_SCALE, level=None):
    """Nearest-neighbour upscale (RGB) with bearing ticks at the top and bottom edges,
    labelled below the top ticks; 0 deg is red. level (level_info()): the level line on top when its 'drawn'.
    A colour image (any mode but L) gets a 1 px black edge around each tick and under the level line, so both read on
    yellow, red or magenta scenery; a grey one is drawn exactly as before."""
    from PIL import Image, ImageDraw, ImageFont
    colour = img.mode != "L"
    w, h = img.size
    if (w, h) in ((320, 240), (640, 480)):
        scale = w // FRAME_W
        big = img.convert("RGB").copy()
    else:
        big = img.resize((FRAME_W * scale, FRAME_H * scale), Image.NEAREST).convert("RGB")
    W, H = big.size
    draw = ImageDraw.Draw(big)
    try:
        font = ImageFont.load_default()
    except Exception:   # labels are optional
        font = None
    for deg in TICK_DEG:
        left = tick_left(bearing_x(deg), scale)
        color = TICK_CENTER_RGB if deg == 0 else TICK_RGB
        if colour:
            draw.rectangle((left - 1, 0, left + TICK_W, TICK_LEN), fill=OUTLINE_RGB)
            draw.rectangle((left - 1, H - TICK_LEN - 1, left + TICK_W, H - 1), fill=OUTLINE_RGB)
        draw.rectangle((left, 0, left + TICK_W - 1, TICK_LEN - 1), fill=color)
        draw.rectangle((left, H - TICK_LEN, left + TICK_W - 1, H - 1), fill=color)
        if font is None:
            continue
        try:
            text = tick_label(deg)
            x0, y0, x1, y1 = draw.textbbox((0, 0), text, font=font)
            tw = x1 - x0
            tx = min(max(1, round(x4_of(bearing_x(deg), scale) - tw / 2.0)), W - tw - 1)
            ty = TICK_LEN + 2
            draw.rectangle((tx - 1, ty + y0 - 1, tx + tw, ty + y1), fill=(0, 0, 0))
            draw.text((tx - x0, ty), text, fill=color, font=font)
        except Exception:
            font = None
    if level is not None and level.get("drawn"):
        draw_level(big, level, outline=colour)
    return big


def flip_rows(data, width, height):
    """The frame upside down: rows in reverse order, each row unchanged (so columns keep their bearings). width: the
    row's length in bytes (3 x the pixels of an RGB image)."""
    return b"".join(bytes(data[r * width:(r + 1) * width]) for r in range(height - 1, -1, -1))


# YUV 4:2:2 -> RGB: full-range BT.601 (JFIF), as the OV3660 sends it; Y is the grey photo's value, so a frame without
# chroma (U = V = 128) decodes to the grey photo exactly. Each pixel pair Y0 U Y1 V shares its U and V.
_RV = [1.402 * (v - 128) for v in range(256)]
_GU = [-0.344136 * (u - 128) for u in range(256)]
_GV = [-0.714136 * (v - 128) for v in range(256)]
_BU = [1.772 * (u - 128) for u in range(256)]


def _c8(x):
    return 0 if x < 0.0 else (255 if x >= 254.5 else int(x + 0.5))


def yuv_rgb(y, u, v):
    """One pixel: (r, g, b), each round(Y + chroma term) clamped to 0..255."""
    return _c8(y + _RV[v]), _c8(y + _GU[u] + _GV[v]), _c8(y + _BU[u])


def yuyv_to_rgb(data, width, height):
    """A YUYV frame (2 x width bytes a row) -> RGB bytes (3 x width a row), rows in the same order."""
    n = width * height
    if len(data) != 2 * n or width % 2:
        raise ValueError(f"YUYV frame of {len(data)} bytes, want {2 * n} (even width)")
    if n > 160 * 120:
        # C-backed array operations keep the worker's Python GIL occupancy short.
        pairs = np.frombuffer(data, dtype=np.uint8).reshape(-1, 4)
        y = pairs[:, (0, 2)].reshape(-1).astype(np.int16)
        u = np.repeat(pairs[:, 1], 2)
        v = np.repeat(pairs[:, 3], 2)
        rgb = np.empty((n, 3), dtype=np.uint8)
        rgb[:, 0] = np.clip(y + np.asarray(_RV, dtype=np.int16)[v], 0, 255)
        rgb[:, 1] = np.clip(y + np.asarray(_GU, dtype=np.int16)[u] + np.asarray(_GV, dtype=np.int16)[v], 0, 255)
        rgb[:, 2] = np.clip(y + np.asarray(_BU, dtype=np.int16)[u], 0, 255)
        return rgb.tobytes()
    out = bytearray(3 * n)
    o = 0
    for i in range(0, 2 * n, 4):
        y0, u, y1, v = data[i], data[i + 1], data[i + 2], data[i + 3]
        rv, gc, bu = _RV[v], _GU[u] + _GV[v], _BU[u]
        out[o:o + 6] = bytes((_c8(y0 + rv), _c8(y0 + gc), _c8(y0 + bu), _c8(y1 + rv), _c8(y1 + gc), _c8(y1 + bu)))
        o += 6
    return bytes(out)


def frame_image(meta, data, flip=FLIP_VERTICAL):
    """The frame as a PIL image the right way up (flip): 'L' for grey, 'RGB' for a colour (YUYV) frame."""
    from PIL import Image
    w, h = meta["width"], meta["height"]
    if meta.get("pixFmt") == PIXFMT_JPEG:
        import io
        with Image.open(io.BytesIO(data)) as captured:
            if captured.size != (w, h): raise ValueError("JPEG dimensions disagree with META")
            img = captured.convert("RGB")
        return img.transpose(Image.Transpose.FLIP_TOP_BOTTOM) if flip else img
    if meta.get("pixFmt", PIXFMT_GRAY8) == PIXFMT_YUYV:
        rgb = yuyv_to_rgb(data, w, h)
        return Image.frombytes("RGB", (w, h), flip_rows(rgb, 3 * w, h) if flip else rgb)
    return Image.frombytes("L", (w, h), flip_rows(data, w, h) if flip else data)


def save_frame(out_dir, stamp, meta, data, extra=None, flip=FLIP_VERTICAL, level=None):
    """Writes <stamp>_<id>_<idx>.png, ..._x4.png and ....json; returns their paths.
    With flip the images are saved the right way up (row order reversed); crc32 stays that of
    the frame as sent. level (level_info()): the JSON's 'level' block, and the line on the _x4.png only (the
    160x120 .png stays raw). A colour frame (META pixFmt YUYV) is saved as RGB PNGs, a grey one as before; the JSON
    has 'color' (and pixFmt / pixFmtName) from the META. A colour frame's bytes as sent (YUYV, rows not flipped) are
    also kept as <base>.yuyv ('raw'): the bench's Y-identity and U/V checks read them (espfly_diag.py colourcheck,
    docs/autonomy-bench.md N')."""
    base = os.path.join(out_dir, f"{stamp}_{meta['id']}_{meta['idx']}")
    files = {"png": base + ".png", "png_x4": base + "_x4.png", "json": base + ".json"}
    img = frame_image(meta, data, flip)
    if meta.get("pixFmt", PIXFMT_GRAY8) == PIXFMT_YUYV:
        files["raw"] = base + ".yuyv"
        with open(files["raw"], "wb") as f:
            f.write(bytes(data))
    if meta.get("pixFmt") == PIXFMT_JPEG:
        files["raw"] = base + ".jpg"
        with open(files["raw"], "wb") as f: f.write(bytes(data))
    # Lossless PNG, lower CPU cost on the image worker; original JPEG bytes
    # and decoded pixel values remain unchanged.
    img.save(files["png"], compress_level=1)
    render_x4(img, level=level).save(files["png_x4"], compress_level=1)
    doc = dict(meta)
    doc["color"] = bool(meta.get("color", False))
    doc["crc32_hex"] = f"{meta['crc32']:08x}"
    doc["flippedVertical"] = bool(flip)
    doc["files"] = files
    doc["bearings"] = bearings_info(meta["width"] // FRAME_W if meta["width"] >= 320 else X4_SCALE)
    if level is not None:
        doc["level"] = level
    if extra:
        doc.update(extra)
    with open(files["json"], "w") as f:
        json.dump(doc, f, indent=1)
        f.write("\n")
    return files


# ---------------------------------------------------------------- the level line (2026-10-15, docs/autonomy-a2b.md 23)
#
# Claude judges the true height from a photo by what lies at camera height: the row of the true horizontal. On the
# upright image (after FLIP_VERTICAL), row 0 at the top, at 160x120:
#     y(x) = 60 + FY tan(pitch + tilt) - (x - 80) tan(roll)
# +pitch nose up, +tilt the camera pointing up (the mount's tilt against the body), +roll makes the line rise to the
# right. FY is the vertical focal length (LEVEL_FY_PX = auto_motion.h AUTO_MV_FY_PX, cam_vz_obs kyGy), not the
# horizontal 102.4. On the x4 PNG (X = 4x + 1.5, Y = 4y + 1.5, as the bearings' x4_of):
#     Y(X) = 241.5 + 399.16 tan(pitch + tilt) - (X - 321.5) tan(roll)        (about 7.0 px per degree)
# pitch and roll are the photo META's (the recorder's attitude when the copy is seen, 0.1-0.23 s after the exposure).
# The tilt absorbs the principal-point offset: one function computes the row for the drawing and the calibration.
# The mount tilt is NOT a constant (2026-10-15 tilt analysis: -6.4..+3.9 deg in flight, jumps of 3-8 deg with a
# crash, the kill fall or a battery replug; stable within one flight, 0.0-0.6 deg): ONE Mac setting (never per room)
# in ~/.espfly/level.json, re-measured every flight from the pre-take-off ground photo ('level-cal'), marked stale by
# the daemon on a new boot, a landing, a contact or a KILL (mechanical facts, no decisions). Ground and flight differ
# by LEVEL_FLIGHT_ADJ_DEG (ground - flight = -0.29 +/- 0.71 deg, n = 15). The physics is room-independent: the camera
# sits LEVEL_CAM_H_GROUND_M above the floor, so on the ground a far floor-wall line D m away lies FY x 0.035 / D px
# (160 scale) below the level row.

LEVEL_FY_PX = 99.79           # vertical focal length at 160x120 (AUTO_MV_FY_PX)
LEVEL_CY = 60.0               # the centre row at 160x120
LEVEL_FLIGHT_ADJ_DEG = 0.3    # tilt in flight = tilt on the ground + this
LEVEL_CAM_H_GROUND_M = 0.035  # the camera above the floor, the drone on the ground
LEVEL_DEFAULT_TILT_DEG = 1.9  # the ground value without a setting (always drawn stale): the 10-15 living room's
LEVEL_FLOOR_DIST_M = 3.0      # level-cal --floor-row: the floor-wall line's distance by default
LEVEL_TICKS_DEG = (-5, -2, -1, 1, 2, 5)   # the right-edge angle ticks (12 px at +-5, 6 px at the others)
LEVEL_RGB = ((255, 230, 0), (255, 0, 255))   # the dashes alternate: visible on dark and light
LEVEL_DASH = (8, 8)           # on, off (x4 px)
LEVEL_DOT = (2, 6)            # a stale line
LEVEL_EDGE_TICK = (12, 3)     # the level row's ticks at both edges, w x h, with a 1 px black border
LEVEL_MARKER = (3, 12)        # the line out of the frame: a marker at the top or bottom edge of the centre column
LEVEL_GROUND_STATES = ("IDLE", "ARMED")   # a photo captured in these is a ground photo (tilt_ground)
LEVEL_ENV = "ESPFLY_LEVEL_LINE"           # 0: no drawing (the JSON block is still written)


def level_path():
    return os.path.join(DEFAULT_DIR, "level.json")


def level_history_path():
    return os.path.join(DEFAULT_DIR, "level-history.jsonl")


def utc_str(t=None):
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(time.time() if t is None else t))


def level_row_160(x, pitch_deg, roll_deg, tilt_deg):
    """The level row at column x of the 160x120 upright image."""
    return LEVEL_CY + LEVEL_FY_PX * math.tan(math.radians(pitch_deg + tilt_deg)) - (x - CENTER_X) * \
        math.tan(math.radians(roll_deg))


def level_row_x4(X, pitch_deg, roll_deg, tilt_deg, scale=X4_SCALE):
    """The level row at column X of the upscaled image (the centre of the source pixels: x4_of)."""
    c = (scale - 1) / 2.0
    return scale * level_row_160((X - c) / scale, pitch_deg, roll_deg, tilt_deg) + c


def level_tilt_from_row(y160, pitch_deg):
    """The tilt that puts the level row at y160 (160 scale, the centre column) for this pitch."""
    return math.degrees(math.atan((y160 - LEVEL_CY) / LEVEL_FY_PX)) - pitch_deg


def load_level(path=None):
    """The level setting (level-cal), or None (none, or unreadable)."""
    try:
        with open(path or level_path()) as f:
            doc = json.load(f)
        float(doc["tilt_ground_deg"])
        float(doc["tilt_flight_deg"])
        return doc
    except (OSError, ValueError, KeyError, TypeError):
        return None


def write_json_atomic(path, doc):
    """path written whole or not at all (a temporary file in its directory, then rename); the directory private."""
    d = os.path.dirname(os.path.abspath(path))
    private_dir(d)
    tmp = path + f".tmp{os.getpid()}"
    with open(tmp, "w") as f:
        json.dump(doc, f, indent=1)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)


def append_jsonl(path, entry):
    private_dir(os.path.dirname(os.path.abspath(path)))
    with open(path, "a") as f:
        f.write(json.dumps(entry) + "\n")


def level_choice(setting, kind, boot=None):
    """The tilt for a photo: (tilt_deg, set_utc, method, stale, stale_why). kind 'ground' or 'flight'. Stale (drawn
    dotted): no setting (the default), invalidated since it was set (a new boot, a landing, a contact, a KILL), or set
    on another boot than the photo's (boot: the photo's bootNonce, hex)."""
    if setting is None:
        t = LEVEL_DEFAULT_TILT_DEG + (LEVEL_FLIGHT_ADJ_DEG if kind == "flight" else 0.0)
        return t, None, "default", True, f"no level-cal yet (the default {LEVEL_DEFAULT_TILT_DEG:g} deg ground)"
    t = float(setting["tilt_flight_deg"] if kind == "flight" else setting["tilt_ground_deg"])
    stale, why = False, None
    inv_t, set_t = setting.get("invalidated_t"), setting.get("set_t")
    if inv_t is not None and (set_t is None or set_t < inv_t):
        stale, why = True, setting.get("invalidated_why") or "invalidated"
    elif boot and setting.get("boot_nonce") and setting["boot_nonce"] != boot:
        stale, why = True, "the drone rebooted (or the battery was swapped) since the level-cal"
    return t, setting.get("set_utc"), setting.get("method"), stale, why


def level_info(pitch_deg, roll_deg, kind, setting, boot=None, scale=X4_SCALE, draw=True):
    """The photo JSON's 'level' block (and what draw_level draws)."""
    tilt, set_utc, method, stale, why = level_choice(setting, kind, boot)
    W, H = FRAME_W * scale, FRAME_H * scale
    cx = x4_of(CENTER_X, scale)
    yc = level_row_160(CENTER_X, pitch_deg, roll_deg, tilt)
    Yc = level_row_x4(cx, pitch_deg, roll_deg, tilt, scale)
    y0, y1 = level_row_x4(0, pitch_deg, roll_deg, tilt, scale), level_row_x4(W - 1, pitch_deg, roll_deg, tilt, scale)
    info = {"row_x4_center": round(Yc, 2), "row_160_center": round(yc, 3),
            "slope_x4": round(-math.tan(math.radians(roll_deg)), 5),
            "endpoints_x4": [[0, round(y0, 2)], [W - 1, round(y1, 2)]],
            "pitch_deg": pitch_deg, "roll_deg": roll_deg, "tilt_deg_used": round(tilt, 3), "tilt_kind": kind,
            "tilt_set_utc": set_utc, "tilt_method": method, "stale": stale, "stale_why": why, "fy_px": LEVEL_FY_PX,
            "formula": f"Y(X) = {x4_of(LEVEL_CY, scale):g} + {scale * LEVEL_FY_PX:.2f} tan(pitch + tilt) - (X - "
                       f"{cx:g}) tan(roll) (x4 px, row 0 at the top; +pitch nose up, +tilt camera up, +roll: the line "
                       "rises to the right); about 7 px per deg",
            "ticks_deg": list(LEVEL_TICKS_DEG), "in_frame": not (max(y0, y1) < 0 or min(y0, y1) > H - 1),
            "drawn": bool(draw)}
    if not info["in_frame"]:
        # how far the nearer end is beyond the frame: negative above it, positive below it
        info["out_of_frame_px"] = round(max(y0, y1), 1) if max(y0, y1) < 0 else round(min(y0, y1) - (H - 1), 1)
    return info


def level_drawing_on():
    return os.environ.get(LEVEL_ENV, "1") != "0"


def draw_level(img, info, outline=False):
    """The level line on the upscaled RGB image: 1 px along Y(X), dashed 8 on / 8 off (stale: dotted 2 / 6), the dashes
    alternating yellow and magenta; on each 4x4 source block it covers at most one row, on half the columns, so no
    source pixel is hidden. Ticks at the level row on both edges (12 x 3, yellow, a black border), the angle ticks at
    the right edge (+-1, +-2 deg 6 px, +-5 deg 12 px; roll ignored), and out of the frame only a marker at the top or
    bottom edge of the centre column. No text. outline (a colour photo): a black pixel under every dash pixel (two
    rows of a 4x4 block at most, the line's row unchanged) and a black column on both sides of the marker, so the line
    reads on yellow or magenta scenery too."""
    W, H = img.size
    px = img.load()
    p, r, t = info["pitch_deg"], info["roll_deg"], info["tilt_deg_used"]
    scale = W // FRAME_W
    black = (0, 0, 0)

    def put(x, y, c):
        if 0 <= x < W and 0 <= y < H:
            px[x, y] = c
    if not info["in_frame"]:
        mw, mh = LEVEL_MARKER
        x0 = int(round(x4_of(CENTER_X, scale) - (mw - 1) / 2.0))
        top = info["out_of_frame_px"] < 0
        for y in range(mh):
            for x in range(x0, x0 + mw):
                put(x, y if top else H - 1 - y, LEVEL_RGB[1] if (y // 3) % 2 else LEVEL_RGB[0])
            if outline:
                put(x0 - 1, y if top else H - 1 - y, black)
                put(x0 + mw, y if top else H - 1 - y, black)
        return
    on, off = LEVEL_DOT if info["stale"] else LEVEL_DASH
    for X in range(W):
        k = X % (on + off)
        if k >= on:
            continue
        Y = int(math.floor(level_row_x4(X, p, r, t, scale) + 0.5))
        put(X, Y, LEVEL_RGB[(X // (on + off)) % 2])
        if outline:
            put(X, Y + 1, black)
    # the level row at both edges
    tw, th = LEVEL_EDGE_TICK
    for xs in (range(0, tw), range(W - tw, W)):
        xm = (xs[0] + xs[-1]) / 2.0
        Ye = int(math.floor(level_row_x4(xm, p, r, t, scale) + 0.5))
        for x in range(xs[0] - 1, xs[-1] + 2):
            for y in range(Ye - th // 2 - 1, Ye + th // 2 + 2):
                inside = xs[0] <= x <= xs[-1] and abs(y - Ye) <= th // 2
                put(x, y, LEVEL_RGB[0] if inside else black)
    # the angle ticks at the right edge, from the centre column's row (roll ignored)
    Yc = level_row_x4(x4_of(CENTER_X, scale), p, r, t, scale)
    for k in LEVEL_TICKS_DEG:
        Yk = int(math.floor(Yc + scale * LEVEL_FY_PX * (math.tan(math.radians(p + t + k)) -
                                                         math.tan(math.radians(p + t))) + 0.5))
        n = 12 if abs(k) == 5 else 6
        for x in range(W - n, W):
            put(x, Yk, LEVEL_RGB[0])
            put(x, Yk + 1, black)


def photo_json_path(ref):
    """A photo's JSON path from its .json, .png or _x4.png path."""
    ref = os.path.abspath(os.path.expanduser(ref))
    for suffix in ("_x4.png", ".png", ".json"):
        if ref.endswith(suffix):
            return ref[:-len(suffix)] + ".json"
    return ref + ".json"


def photo_kind(doc):
    """'ground' (captured in IDLE / ARMED, or a one-shot photo without the daemon: on the ground) or 'flight'."""
    lv = doc.get("level") or {}
    if lv.get("tilt_kind") in ("ground", "flight"):
        return lv["tilt_kind"]
    au = doc.get("autonomy")
    if not au:
        return "ground"
    return "ground" if (au.get("state_capture") or au.get("state")) in LEVEL_GROUND_STATES else "flight"


_PHOTO_STAMP_RE = re.compile(r"^(\d{8}-\d{6})_(\d+)_(\d+)\.json$")


def photo_time(doc, path=None):
    """When a photo was taken (wall clock, s; None: unknown): its link block's received_t (the daemon's save, since
    2026-10-15), else its received_utc (1 s), else its file name's stamp (local time, the set's start). Never the file's
    mtime: rerender_level() writes old JSONs again."""
    link = doc.get("link") or {}
    t = link.get("received_t")
    if isinstance(t, (int, float)) and math.isfinite(t):
        return float(t)
    utc = link.get("received_utc")
    if isinstance(utc, str):
        try:
            return float(calendar.timegm(time.strptime(utc, "%Y-%m-%dT%H:%M:%SZ")))
        except ValueError:
            pass
    m = _PHOTO_STAMP_RE.match(os.path.basename(path or ""))
    if m:
        try:
            return time.mktime(time.strptime(m.group(1), "%Y%m%d-%H%M%S"))
        except (ValueError, OverflowError):
            pass
    return None


def latest_photo_json(photo_dir, kind=None):
    """The latest photo JSON in photo_dir (of that kind: ground or flight) by when it was taken (photo_time(), then the
    name's stamp, id and index; not the mtime, which rerender_level() moves), or None."""
    try:
        names = [n for n in os.listdir(photo_dir) if n.endswith(".json")]
    except OSError:
        return None
    found = []
    for n in names:
        path = os.path.join(photo_dir, n)
        try:
            with open(path) as f:
                doc = json.load(f)
        except (OSError, ValueError):
            continue
        if "pitch_deg" in doc and (kind is None or photo_kind(doc) == kind):
            t = photo_time(doc, path)
            m = _PHOTO_STAMP_RE.match(n)
            found.append((t if t is not None else float("-inf"),
                          (m.group(1), int(m.group(2)), int(m.group(3))) if m else (n, 0, 0), path))
    return max(found)[2] if found else None


def rerender_level(json_path, setting, draw=True):
    """Draws a saved photo's _x4.png again from its raw .png with the level line of this setting, and writes the JSON's
    new 'level' block; returns that block."""
    from PIL import Image
    with open(json_path) as f:
        doc = json.load(f)
    base = json_path[:-len(".json")]
    lv = level_info(doc["pitch_deg"], doc["roll_deg"], photo_kind(doc), setting, (doc.get("link") or {}).get("bootNonce"),
                    scale=doc["width"] // FRAME_W if doc["width"] >= 320 else X4_SCALE, draw=draw)
    with Image.open(base + ".png") as img:
        render_x4(img, level=lv).save(base + "_x4.png")
    doc["level"] = lv
    write_json_atomic(json_path, doc)
    return lv


def level_cal_run(req, photo_dir, event, boot=None):
    """level-cal (arithmetic only, docs/autonomy-a2b.md 23): the mount tilt from a photo, written as THE level
    setting (~/.espfly/level.json: one setting, never per room), its history line and an event; the photo's _x4.png
    drawn again. floor_row R (x4 row of a far floor-wall junction 2-6 m away near the centre column, on a ground
    photo; dist D, default 3.0): y = (R - 1.5) / 4 - FY x 0.035 / D, tilt_ground = atan((y - 60) / FY) - pitch,
    tilt_flight = + 0.3. horizon_row R (a hover photo's vanishing point of horizontal room lines, or another true
    horizon cue, at the centre column): tilt_flight = atan(((R - 1.5) / 4 - 60) / FY) - pitch, ground = - 0.3. set
    DEG: the flight value by hand (ground DEG - 0.3). show: the setting as it is.
    A row is read off a photo of the camera as it is now: a photo (given or the latest) taken before the latest
    invalidation of the setting (a landing, a contact, a KILL, a new boot: they may move the camera), or from another
    boot than boot (the daemon's), is refused (2026-10-15 review); the setting keeps that time (valid_after_t) through
    later level-cals."""
    old = load_level()
    if req.get("show"):
        out = {"ok": True, "setting": old, "path": level_path()}
        for kind in ("ground", "flight"):
            t, set_utc, method, stale, why = level_choice(old, kind)
            out[kind] = {"tilt_deg": round(t, 3), "stale": stale, "stale_why": why}
        return out
    methods = [m for m in ("floor_row", "horizon_row", "set") if req.get(m) is not None]
    if len(methods) != 1:
        raise ValueError("give exactly one of floor_row, horizon_row, set (or show)")
    method = methods[0]
    ref = req.get("photo")
    if ref:
        path = photo_json_path(ref)
    else:
        path = latest_photo_json(photo_dir, {"floor_row": "ground", "horizon_row": "flight", "set": None}[method])
        if path is None and method != "set":
            want = "ground photo (one taken before the take-off, in IDLE)" if method == "floor_row" else "flight photo"
            return {"ok": False, "error": f"no {want} in {photo_dir}: take one ('photo') or give --photo"}
    doc = None
    if path is not None:
        try:
            with open(path) as f:
                doc = json.load(f)
            float(doc["pitch_deg"])
        except (OSError, ValueError, KeyError, TypeError) as e:
            return {"ok": False, "error": f"cannot read the photo {path}: {e}"}
    t = time.time()
    # the latest event that may have moved the camera: the setting's invalidation, kept through later level-cals
    after = [v for v in ((old or {}).get("invalidated_t"), (old or {}).get("valid_after_t"))
             if isinstance(v, (int, float))]
    valid_after = max(after) if after else None
    t_ph = photo_time(doc, path) if doc is not None else None
    ph_info = {} if doc is None else {"photo": path, "photo_utc": None if t_ph is None else utc_str(t_ph),
                                      "photo_age_s": None if t_ph is None else round(t - t_ph, 1)}
    if doc is not None and method != "set":
        ph_boot = (doc.get("link") or {}).get("bootNonce")
        if boot and ph_boot and ph_boot != boot:
            return {"ok": False, **ph_info,
                    "error": f"the photo {os.path.basename(path)} is from another boot ({ph_boot}, the drone now "
                             f"{boot}: rebooted or a battery swap, which may move the camera): take a new ground photo "
                             "('photo') and level-cal again"}
        if valid_after is not None and (t_ph is None or t_ph < valid_after):
            why = (old or {}).get("invalidated_why") or "an event that may have moved the camera"
            return {"ok": False, **ph_info,
                    "error": f"the photo {os.path.basename(path)} was taken "
                             f"{'at an unknown time' if t_ph is None else 'at ' + utc_str(t_ph)}, before {why} "
                             f"({utc_str(valid_after)}) made the level line stale: take a new photo ('photo') and "
                             "level-cal again"}
    src = {"photo": path}
    if method == "set":
        tf = float(req["set"])
        if not (math.isfinite(tf) and -30.0 <= tf <= 30.0):
            raise ValueError("set: -30..30 deg")
        tg, how = tf - LEVEL_FLIGHT_ADJ_DEG, "manual"
    else:
        R = float(req[method])
        if not (math.isfinite(R) and -200.0 <= R <= 680.0):
            raise ValueError(f"{method}: an x4 row, -200..680")
        pitch = float(doc["pitch_deg"])
        src.update(row_x4=R, pitch_deg=pitch, roll_deg=doc.get("roll_deg"))
        if method == "floor_row":
            dist = float(req.get("dist") or LEVEL_FLOOR_DIST_M)
            if not (math.isfinite(dist) and 0.5 <= dist <= 20.0):
                raise ValueError("dist: 0.5..20 m")
            y = (R - 1.5) / 4.0 - LEVEL_FY_PX * LEVEL_CAM_H_GROUND_M / dist
            tg = level_tilt_from_row(y, pitch)
            tf, how = tg + LEVEL_FLIGHT_ADJ_DEG, "floor"
            src["dist_m"] = dist
        else:
            tf = level_tilt_from_row((R - 1.5) / 4.0, pitch)
            tg, how = tf - LEVEL_FLIGHT_ADJ_DEG, "horizon"
    new = {"tilt_ground_deg": round(tg, 3), "tilt_flight_deg": round(tf, 3), "set_t": t, "set_utc": utc_str(t),
           "method": how, "source": src,
           "boot_nonce": None if doc is None else (doc.get("link") or {}).get("bootNonce"),
           "invalidated_t": None, "invalidated_utc": None, "invalidated_why": None, "valid_after_t": valid_after}
    write_json_atomic(level_path(), new)
    append_jsonl(level_history_path(), {"utc": utc_str(t), "t": t, "kind": "set", "method": how, "old": old,
                                        "new": {k: new[k] for k in ("tilt_ground_deg", "tilt_flight_deg", "source")}})
    out = {"ok": True, "method": how, "path": level_path(),
           "old": None if old is None else {k: old.get(k) for k in ("tilt_ground_deg", "tilt_flight_deg", "set_utc",
                                                                  "method")},
           "new": {k: new[k] for k in ("tilt_ground_deg", "tilt_flight_deg", "set_utc")}, **ph_info}
    if path is not None:
        try:
            lv = rerender_level(path, new, draw=level_drawing_on() and not req.get("no_level"))
        except (OSError, ValueError, KeyError) as e:
            out["warning"] = f"the photo's _x4.png was not drawn again: {type(e).__name__}: {e}"
        else:
            out.update(photo=path, png_x4=path[:-len(".json")] + "_x4.png", row_x4_center=lv["row_x4_center"],
                       tilt_kind=lv["tilt_kind"])
            if method == "floor_row":
                gap = req["floor_row"] - lv["row_x4_center"]
                out["floor_gap_px"] = round(gap, 1)
                out["check"] = (f"the line now sits {gap:.1f} px above the floor-wall line (a far line 2-6 m away: "
                                "3-7 px)")
    event("level_cal", **{k: out.get(k) for k in ("method", "old", "new", "photo", "row_x4_center", "floor_gap_px")},
          src=src)
    return out


# ---------------------------------------------------------------- key

DEFAULT_DIR = os.path.expanduser("~/.espfly")
DEFAULT_KEY_FILE = os.path.join(DEFAULT_DIR, "auto.key")
DEFAULT_PHOTO_DIR = os.path.join(DEFAULT_DIR, "photos")


class LinkError(Exception):
    """A command failure; info goes into the JSON line."""

    def __init__(self, msg, **info):
        super().__init__(msg)
        self.info = info


_log_sink = None    # the daemon routes its messages through a LineWriter: its loop never blocks on stderr


def log(msg):
    if _log_sink is not None:
        _log_sink(msg)
        return
    print(msg, file=sys.stderr, flush=True)


class LineWriter:
    """Lines to a stream from a thread of its own, so the daemon's loop (heartbeat, 'k') never blocks on its
    terminal or pipe: a reader that stopped, a full pipe, a terminal in Ctrl-S. At most max_lines wait;
    further lines are dropped and counted."""

    def __init__(self, stream, max_lines=1000):
        self.stream = stream
        self.q = queue.Queue(max_lines)
        self.dropped = 0
        self.thread = threading.Thread(target=self._run, name="espfly-out", daemon=True)
        self.thread.start()

    def write_line(self, line):
        try:
            self.q.put_nowait(line)
        except queue.Full:
            self.dropped += 1

    def _run(self):
        while True:
            line = self.q.get()
            if line is None:
                return
            try:
                self.stream.write(line + "\n")
                self.stream.flush()
            except (OSError, ValueError):
                pass

    def close(self, timeout=0.5):
        """Lets the queued lines out for up to timeout s (a stream that blocks keeps the thread, which is a
        daemon thread)."""
        try:
            self.q.put(None, timeout=timeout)
        except queue.Full:
            return
        self.thread.join(timeout)


def load_key(path):
    try:
        with open(path) as f:
            text = f.read().strip()
    except FileNotFoundError:
        raise LinkError(f"no key file {path}: run 'keygen', then 'install-key' with the drone on USB",
                        key_file=path) from None
    if not re.fullmatch(r"[0-9a-fA-F]{32}", text):
        raise LinkError(f"{path} must hold 32 hex digits", key_file=path)
    if os.stat(path).st_mode & 0o077:
        log(f"warning: {path} is readable by other users; chmod 600 it")
    return bytes.fromhex(text)


def private_dir(path):
    """Creates path (0700) if needed; the default ~/.espfly is always kept at 0700."""
    if not os.path.isdir(path):
        os.makedirs(path, mode=0o700)
        os.chmod(path, 0o700)
    elif os.path.abspath(path) == os.path.abspath(DEFAULT_DIR):
        os.chmod(path, 0o700)


def write_key(path, key):
    private_dir(os.path.dirname(os.path.abspath(path)))
    tmp = path + ".tmp"
    try:
        os.unlink(tmp)
    except FileNotFoundError:
        pass
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(key.hex())
        f.flush()
        os.fsync(f.fileno())
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)


def parse_fp_reply(reply):
    """Key fingerprint in the drone's 'autokey' reply ('fp=xxxxxxxx', or a lone 8-hex word)."""
    m = re.search(r"\b(?:key_?fp|keyFp|fp)=([0-9a-fA-F]{8})\b", reply)
    if m:
        return m.group(1).lower()
    words = re.findall(r"\b[0-9a-fA-F]{8}\b", reply)
    return words[0].lower() if len(words) == 1 else None


# ---------------------------------------------------------------- interfaces

IP_BOUND_IF = 25        # <netinet/in.h> on macOS; Python's socket module does not export it


def _tool(name):
    for d in ("/sbin", "/usr/sbin"):
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    return name


def run_cmd(argv):
    """stdout of a local command, or None if it fails."""
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout if r.returncode == 0 else None


def parse_ifconfig(text):
    """[(interface, IPv4 address), ...] from ifconfig output."""
    out = []
    iface = None
    for line in text.splitlines():
        if line and not line[0].isspace():
            iface = line.split(":", 1)[0]
            continue
        parts = line.split()
        if iface and len(parts) >= 2 and parts[0] == "inet":
            out.append((iface, parts[1]))
    return out


def parse_route_iface(text):
    """Interface from 'route -n get' output."""
    for line in (text or "").splitlines():
        k, _, v = line.strip().partition(":")
        if k == "interface" and v.strip():
            return v.strip()
    return None


def link_check(run=None, drone_ip=DRONE_IP):
    """Which interface reaches the drone, and does the default route use another one?
    Local commands only; nothing is sent to the drone."""
    run = run or run_cmd
    cands = [(i, ip) for i, ip in parse_ifconfig(run([_tool("ifconfig")]) or "")
             if ip.startswith(SUBNET_PREFIX) and ip != drone_ip]
    route_if = parse_route_iface(run([_tool("route"), "-n", "get", drone_ip]))
    default_if = parse_route_iface(run([_tool("route"), "-n", "get", "default"]))
    iface, local_ip = next(((i, ip) for i, ip in cands if i == route_if), cands[0] if cands else (None, None))
    link_up = iface is not None and route_if == iface
    problems, warnings = [], []
    if iface is None:
        problems.append(f"no interface has a {SUBNET_PREFIX}x address: join the drone's Wi-Fi AP")
    elif route_if != iface:
        problems.append(f"the route to {drone_ip} uses {route_if}, not {iface}")
    if len(cands) > 1:
        warnings.append(f"several interfaces are on {SUBNET_PREFIX}x: {', '.join(i for i, _ in cands)}")
    if default_if is None:
        warnings.append("no default route: the Mac is not online through another interface")
    elif default_if == iface:
        warnings.append(f"the default route uses the drone Wi-Fi ({iface}): internet traffic goes to the drone")
    return {"link_up": link_up, "iface": iface, "local_ip": local_ip, "drone": drone_ip, "route_iface": route_if,
            "default_iface": default_if, "internet_separate": default_if is not None and default_if != iface,
            "candidates": [i for i, _ in cands], "problems": problems, "warnings": warnings}


# ---------------------------------------------------------------- link

class Link:
    """One UDP socket to the drone and this process's session: seq numbers from 1,
    ACKs by seq, STATUS replies and photo frames collected as they arrive."""

    RETRY_S = 0.15
    RETRIES = 4         # after the first send

    def __init__(self, key, host, port=PORT, iface=None):
        self.key = key
        self.fp = key_fp(key)
        try:
            self.addr = (socket.gethostbyname(host), port)
        except OSError as e:
            raise LinkError(f"cannot resolve {host}: {e}") from None
        self.iface = iface
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        if iface:
            try:
                index = socket.if_nametoindex(iface)
            except OSError:
                self.sock.close()
                raise LinkError(f"no interface {iface}") from None
            if sys.platform == "darwin":
                self.sock.setsockopt(socket.IPPROTO_IP, getattr(socket, "IP_BOUND_IF", IP_BOUND_IF), index)
            else:
                log(f"warning: not binding to {iface} (IP_BOUND_IF is macOS only)")
        self.sock.bind(("", 0))
        self.session = 0
        self.seq = 0
        self.nonce = None
        self.hello_ack = None
        self.acks = {}
        self.statuses = []
        self.auto_statuses = []
        self.photos = {}
        self.stats = {"sent": 0, "received": 0, "bad_tag": 0, "bad_framing": 0, "bad_payload": 0,
                      "other_session": 0, "foreign": 0}

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def next_seq(self):
        self.seq += 1
        return self.seq

    def send(self, ptype, flags, seq, payload=b"", session=None):
        pkt = encode(self.key, ptype, flags, self.session if session is None else session, seq, payload)
        try:
            self.sock.sendto(pkt, self.addr)
        except OSError as e:
            raise LinkError(f"send to {self.addr[0]}:{self.addr[1]} failed: {e}", iface=self.iface) from None
        self.stats["sent"] += 1

    def poll(self, timeout):
        """Receives and dispatches at most one datagram; returns its Packet or None."""
        self.sock.settimeout(max(timeout, 0.001))
        try:
            data, src = self.sock.recvfrom(4096)
        except socket.timeout:
            return None
        except OSError as e:
            log(f"receive: {e}")
            time.sleep(min(timeout, 0.01))
            return None
        return self.handle(data, src)

    def poll_nowait(self):
        """As poll, for a select() loop: None at once when nothing is waiting."""
        self.sock.setblocking(False)
        try:
            data, src = self.sock.recvfrom(4096)
        except (BlockingIOError, InterruptedError):
            return None
        except OSError as e:
            log(f"receive: {e}")
            return None
        return self.handle(data, src)

    def handle(self, data, src):
        if src[0] != self.addr[0]:
            self.stats["foreign"] += 1
            return None
        try:
            pkt = decode(self.key, data)
        except ProtoError as e:
            self.stats["bad_tag" if e.reason == "tag" else "bad_framing"] += 1
            return None
        self.stats["received"] += 1
        try:
            self.dispatch(pkt)
        except ProtoError:
            self.stats["bad_payload"] += 1
            return None
        return pkt

    def dispatch(self, pkt):
        if pkt.type == T_HELLO_ACK:
            h = parse_hello_ack(pkt.payload)
            if self.nonce is not None and h["echoNonce"] == self.nonce and h["session"] == pkt.session:
                if self.hello_ack is not None:
                    if pkt.session == self.session:
                        return  # the drone's answer to a resent HELLO: same session
                    log(f"later HELLO_ACK: session {pkt.session:08x} replaces {self.session:08x}")
                    h["rtt_ms"], h["tries"] = self.hello_ack.get("rtt_ms"), self.hello_ack.get("tries")
                self.hello_ack = h
                self.session = pkt.session
            else:
                self.stats["other_session"] += 1
            return
        if pkt.type == T_ACK:
            a = parse_ack(pkt.payload)
            # BAD_SESSION comes in the session we sent (our stale one), not the drone's current one
            if self.session and (pkt.session == self.session or a["result"] == R_BAD_SESSION):
                self.acks[a["ackSeq"]] = a
            else:
                self.stats["other_session"] += 1
            return
        if self.session == 0 or pkt.session != self.session:
            self.stats["other_session"] += 1
            return
        if pkt.type == T_STATUS:
            self.statuses.append(parse_status(pkt.payload))
        elif pkt.type == T_AUTO_STATUS:
            self.auto_statuses.append(parse_auto_status(pkt.payload))
        elif pkt.type == T_META:
            m = parse_meta(pkt.payload)
            if m["idx"] < MAX_N:
                self.frame(m["id"], m["idx"]).add_meta(m)
        elif pkt.type == T_CHUNK:
            photo_id, idx, chunk_idx, data = parse_chunk(pkt.payload)
            if idx < MAX_N:
                self.frame(photo_id, idx).add_chunk(chunk_idx, data)

    def frame(self, photo_id, idx):
        return self.photos.setdefault(photo_id, {}).setdefault(idx, Frame())

    def wait(self, pred, timeout):
        deadline = time.monotonic() + timeout
        while not pred():
            left = deadline - time.monotonic()
            if left <= 0:
                return False
            self.poll(left)
        return True

    def hello(self):
        """HELLO with a fresh nonce; returns the HELLO_ACK fields (session, bootNonce, fw...)."""
        self.nonce = secrets.randbits(32)
        self.session = 0
        self.hello_ack = None
        seq = self.next_seq()
        t0 = time.monotonic()
        for attempt in range(1 + self.RETRIES):
            self.send(T_HELLO, FLAG_RETX if attempt else 0, seq, hello_payload(self.nonce), session=0)
            if self.wait(lambda: self.hello_ack is not None, self.RETRY_S):
                break
        else:
            raise LinkError(f"no HELLO_ACK from {self.addr[0]}:{self.addr[1]} after {1 + self.RETRIES} tries "
                            "(drone off or AP not joined, no key installed, or a different key: "
                            "compare the keygen and install-key fingerprints)",
                            key_fp=self.fp, link=dict(self.stats))
        h = self.hello_ack
        h["rtt_ms"] = round((time.monotonic() - t0) * 1000.0, 1)
        h["tries"] = attempt + 1
        if not hmac.compare_digest(h["keyFp"], self.fp):
            raise LinkError("the drone reports a different key fingerprint", drone_fp=h["keyFp"], key_fp=self.fp)
        return h

    def command(self, ptype, payload, rehello=True):
        """Sends with ACKREQ, retrying with RETX and the same seq; returns the ACK fields."""
        seq = self.next_seq()
        for attempt in range(1 + self.RETRIES):
            self.send(ptype, FLAG_ACKREQ | (FLAG_RETX if attempt else 0), seq, payload)
            if self.wait(lambda: seq in self.acks, self.RETRY_S):
                break
        else:
            raise LinkError(f"no ACK for {TYPE_NAMES[ptype]} (seq {seq}) after {1 + self.RETRIES} tries",
                            link=dict(self.stats))
        ack = self.acks.pop(seq)
        if ack["result"] == R_BAD_SESSION and rehello:
            log(f"{TYPE_NAMES[ptype]}: BAD_SESSION; saying HELLO again")
            self.hello()
            return self.command(ptype, payload, rehello=False)
        if ack["ackType"] != ptype:
            log(f"warning: ACK for seq {seq} names type 0x{ack['ackType']:02x}, sent 0x{ptype:02x}")
        return ack

    def status(self, rehello=True):
        """STATUS_REQ (a query: no ACKREQ, a new seq per try); returns the STATUS fields."""
        for _ in range(1 + self.RETRIES):
            seq = self.next_seq()
            n0 = len(self.statuses)
            self.send(T_STATUS_REQ, 0, seq)
            if self.wait(lambda: len(self.statuses) > n0 or seq in self.acks, self.RETRY_S):
                break
        else:
            raise LinkError(f"no STATUS after {1 + self.RETRIES} tries", link=dict(self.stats))
        if len(self.statuses) > n0:
            return self.statuses[-1]
        ack = self.acks.pop(seq)
        if ack["result"] == R_BAD_SESSION and rehello:
            log("STATUS_REQ: BAD_SESSION; saying HELLO again")
            self.hello()
            return self.status(rehello=False)
        raise LinkError(f"STATUS_REQ answered with ACK {ack['resultName']}", result=ack["resultName"])


def ack_error(what, ack):
    name = ack["resultName"]
    hint = RESULT_HINTS.get(name)
    return LinkError(f"{what} refused: {name}" + (f" ({hint})" if hint else ""), result=name,
                     detail=ack["detail"], state=ack["state"])


# ---------------------------------------------------------------- photos

QUIET_S = 0.25          # no photo packet for this long: re-request what is missing
DUE_S = 1.5             # a frame never seen is due this long after its capture time
MAX_ROUNDS = 4          # re-request rounds per frame


def receive_photo(link, photo_id, n, gap_ms, deadline, t0=None, quiet_s=QUIET_S, due_s=DUE_S,
                  max_rounds=MAX_ROUNDS):
    """Collects the n frames of a photo set, re-requesting missing chunks (CHUNKS, at most
    8 per request) when the link goes quiet. A CHUNKS answered NO_PHOTO ends that frame;
    BUSY (not captured yet, or the drone's resend queue is full) ends the round without
    using it up. Returns ({idx: data}, {idx: reason}, rounds)."""
    t0 = time.monotonic() if t0 is None else t0
    frames = [link.frame(photo_id, i) for i in range(n)]
    done, failed = {}, {}
    rounds = [0] * n
    refetched = set()
    last_rx = time.monotonic()
    while True:
        for i, f in enumerate(frames):
            if i in done or i in failed or not f.complete():
                continue
            try:
                done[i] = f.assemble()
                log(f"photo {photo_id} frame {i}: {f.count} chunks, CRC ok")
            except FrameError as e:
                if e.final or i in refetched:
                    failed[i] = str(e)
                else:
                    log(f"photo {photo_id} frame {i}: {e}; fetching it again")
                    refetched.add(i)
                    f.chunks.clear()
        pending = [i for i in range(n) if i not in done and i not in failed]
        if not pending:
            break
        now = time.monotonic()
        if now >= deadline:
            for i in pending:
                f = frames[i]
                failed[i] = "timed out: " + ("no META, " if f.meta is None else "") + \
                    f"{len(f.missing())} of {f.count} chunks missing"
            break
        pkt = link.poll(min(quiet_s, deadline - now))
        if pkt is not None and pkt.type in (T_META, T_CHUNK) and len(pkt.payload) >= 2 \
                and struct.unpack_from("<H", pkt.payload)[0] == photo_id:
            last_rx = time.monotonic()
            continue
        now = time.monotonic()
        if now - last_rx < quiet_s:
            continue
        latest_seen = max((i for i in range(n) if frames[i].seen()), default=-1)
        for i in pending:
            f = frames[i]
            if i > latest_seen and now - t0 < i * gap_ms / 1000.0 + due_s:
                continue        # not captured yet
            if rounds[i] >= max_rounds:
                failed[i] = ("no META (cannot check the CRC), " if f.meta is None else "") + \
                    f"{len(f.missing())} of {f.count} chunks missing after {rounds[i]} re-request rounds"
                continue
            rounds[i] += 1
            # Without its META the frame's chunk count is unknown (and a frame never captured
            # has none): ask for the META first (CHUNKS count 0), the chunks in a later round.
            requests = [(0, 0)] if f.meta is None else missing_runs(f.missing())
            for first, count in requests:
                try:
                    ack = link.command(T_CHUNKS, chunks_payload(photo_id, i, count, first))
                except LinkError as e:
                    log(f"photo {photo_id} frame {i}: {e}")
                    continue
                if ack["result"] == R_OK:
                    continue
                log(f"photo {photo_id} frame {i}: CHUNKS {first}+{count}: {ack['resultName']}")
                if ack["result"] == R_NO_PHOTO:
                    failed[i] = "the drone has no data for this frame (NO_PHOTO: not captured, or the set " \
                                "was freed)"
                elif ack["result"] == R_BUSY:
                    rounds[i] -= 1
                break
        last_rx = time.monotonic()
    return done, failed, rounds


# ---------------------------------------------------------------- daemon (stage A2b)
#
# One process owns the session for a whole sortie: a single-threaded select() loop over the UDP
# socket, the client socket, its connections and the terminal. The heartbeat is sent from that
# loop and nowhere else, so the drone's 1 s heartbeat rule tells it whether this process is
# healthy; nothing in the loop blocks (client replies are buffered and sent as the socket takes
# them, terminal output goes through a LineWriter thread). Commands are asynchronous: one of
# ARM, TAKEOFF, PHOTO at a time, as the drone's link serves one, plus any number of CHUNKS and
# PHOTO_FREE. LAND and DISARM (the stop commands) never wait behind them: a stop withdraws every
# ARM, TAKEOFF and PHOTO and is sent again until the drone confirms it.

HB_S = 0.1                 # heartbeat period; the drone lands after 1 s without one
HB_LATE_S = 0.3            # a heartbeat later than this is logged
STATUS_REQ_S = 1.0
LINK_LOST_S = 2.0          # no packet from the drone for this long: link_lost event
BUSY_RESENDS = 3           # ACK BUSY (another command pending, or withdrawn unprocessed): sent again this often
BUSY_RETRY_S = 0.05
ARM_CAM_WAKE_S = 0.5       # ARM refused GUARDS for CAMERA alone: the camera wakes from its ground standby on the ARM
ARM_CAM_WAKE_RESENDS = 6   # itself (docs/autonomy-a2b.md 17.1, ~0.15-0.25 s to two fresh frames); sent again (new
                           # seq) after ARM_CAM_WAKE_S, at most this often (3 s); every other guard set is final
BUSY_PHOTO_MAX = 10        # PHOTO refused BUSY while a set is held: waits for its PHOTO_FREE this often
PENDING_WAIT_S = 0.3       # BUSY naming an earlier copy of the same command as pending: its ACK comes within this
RECONCILE_S = 1.0          # ACK BUSY taken-unknown: an AUTO_STATUS newer than the command must tell within this
RECONCILE_STATUS_S = 0.2   # STATUS_REQ this often while reconciling (the drone answers with AUTO_STATUS)
FRESH_MARGIN_MS = 100      # an AUTO_STATUS is newer than a command when its snapshot time (upMs) is this much past
                           # the command's send time on the drone's clock (taken-unknown: the result is not
                           # published within 200 ms, so every snapshot with the result but one is this new)
CLOCK_LEAK = 1e-4          # the Mac-to-drone clock offset estimate rises 100 ppm while no closer sample comes
STOP_RETRY_S = 0.1         # LAND / DISARM: sent again (new seq) this often until confirmed
STOP_GIVE_UP_S = 10.0      # ... and given up after this (the drone's own air-time and heartbeat rules remain)
SETTLE_S, SETTLE_REQ_S = 2.0, 0.2   # a TAKEOFF that may have acted after its sortie ended: decided from a fresh
                                    # AUTO_STATUS (a STATUS_REQ this often, up to this long; Daemon.settle_takeoff)
STOP_REPLY_S = 3.0         # the land/disarm client is answered by then, confirmed or not
WITHDRAW_WAIT_S = 0.3      # a withdrawn command already sent: its ACK may still come this long
QUIT_LAND_S = 3.0          # 'q' while active: LAND, then wait this long for touchdown
SET_TIMEOUT_S = 20.0       # native VGA: includes asynchronous compression and transfer (heartbeat independent)


def photo_set_timeout_s(n):
    """Sequential VGA JPEGs: one frame's receive budget plus 10 s per extra encode.

    Unknown pushed sets reserve all three slots until META names their count.
    Explicit client deadlines still take precedence.
    """
    return SET_TIMEOUT_S + 10.0 * (max(1, MAX_N if n is None else n) - 1)
KILL_BURST_N, KILL_BURST_GAP_S = 10, 0.010
KILL_ACK_WAIT_S = 0.2
CLIENT_PROBE_S = 1.0       # a daemon socket that does not answer within this: the one-shots refuse
CLIENT_SEND_S = 2.0        # a client that does not take its reply within this is dropped
EVENTS_KEPT = 5000
SOCK_MAX_LEN = 100         # sockaddr_un.sun_path holds 104 bytes on macOS
FLIGHT_TYPES = (T_ARM, T_TAKEOFF, T_PHOTO, T_MOVE, T_YAW, T_STOP, T_SET_HEIGHT)  # one at a time, in the flight slot
MOTION_TYPES = (T_MOVE, T_YAW, T_SET_HEIGHT)                                      # (STOP first); a STOP withdraws these
STOP_TYPES = (T_LAND, T_DISARM)                    # the stop slot: never refused, harmless twice
EFFECT_STATES = {T_ARM: (S_ARMED,), T_TAKEOFF: (S_TAKEOFF, S_HOVER)}
STOP_STATES = {T_LAND: (S_LAND, S_IDLE, S_KILLED), T_DISARM: (S_IDLE, S_LAND, S_KILLED)}
RS_SINK = REASON_NAMES.index("SINK")   # a LAND SINK (the camera height out 5 s) may resume: it confirms no LAND
KEY_HELP = "keys: k KILL (motors off at once), l LAND, s status line, q quit (LANDs first if active)"


def default_sock():
    return os.path.join(DEFAULT_DIR, "auto.sock")


def default_events():
    return os.path.join(DEFAULT_DIR, "events.jsonl")


def vmin_cache_path():
    """Written by 'espfly_diag.py auto get' (USB): the drone's autonomy settings as last read."""
    return os.path.join(DEFAULT_DIR, "auto_get.json")


def cached_setting(name):
    """One autonomy setting from the last 'espfly_diag.py auto get': {"value": int, "src", "read_utc"}, or None."""
    try:
        with open(vmin_cache_path()) as f:
            doc = json.load(f)
        v = doc["values"][name]
        return {"value": int(v["value"]), "src": v.get("src"), "read_utc": doc.get("utc")}
    except (OSError, ValueError, KeyError, TypeError):
        return None


def speed_help(pct=None):
    """The help strings' speeds (fast moves): the drone's mvspeed (and mvacc / mvsettle) as the last 'auto get' read it
    (else 100)."""
    if pct is None:
        v = cached_setting("mvspeed")
        pct = v["value"] if v is not None else MV_SPEED_DEFAULT
    lim = mv_limits(pct, fast_caps())
    return lim, f"mvspeed {lim['pct']} %"


def fast_caps():
    """The drone's fast steps' acceleration cap and settle floor (NVS mvacc cm/s^2, mvsettle ms; 2026-10-17 "no, I
    want real 300%", docs/autonomy-a2b.md 35) as the last 'auto get' read them: {"mvacc", "mvsettle", "src"}. A key
    missing (an older drone: its caps are the defaults) or out of its range (the drone refuses it, loudly) is the
    default; src "auto get" when both came from the cache, else "default" (or "partly default"). The drone latched
    them at its ARM (no setting changes during a sortie), so a cache read after the last 'auto set' mirrors them."""
    out = dict(MV_FAST_DEFAULT)
    got = 0
    for k, (lo, hi) in (("mvacc", MV_ACC_CM), ("mvsettle", MV_SETTLE_FLOOR_MS)):
        v = cached_setting(k)
        if v is not None and lo <= v["value"] <= hi:
            out[k] = v["value"]
            got += 1
    out["src"] = "auto get" if got == 2 else ("partly default" if got else "default")
    # with them the expansion rate cap mvwmax (2026-10-18; at every speed step): the default without it, src untouched
    v = cached_setting("mvwmax")
    if v is not None and MV_WFWD_PCT[0] <= v["value"] <= MV_WFWD_PCT[1]:
        out["mvwmax"] = v["value"]
    return out


def mac_min_front():
    """The Mac's floor ahead (m): the drone's NVS mvfloor as the last 'auto get' read it (else DRONE_MIN_FRONT_M) +
    MAC_FRONT_EXTRA_M, as the drone's TAKEOFF plan keeps (2026-10-17: 0.35 m at the default; a drone set to a larger
    floor makes the Mac's larger too)."""
    v = cached_setting("mvfloor")
    if v is not None and v["value"] == 0:
        return 0.0   # no floor (2026-10-18 "guards off"): the Mac refuses nothing for the scene ahead, the drone plans
                     # up to MOVE_D_FLOOR_M from it (its TAKEOFF plan too)
    floor_m = v["value"] / 100.0 if v is not None and 20 <= v["value"] <= 100 else DRONE_MIN_FRONT_M
    return round(floor_m + MAC_FRONT_EXTRA_M, 2)


def cached_k_low():
    """The lowest image expansion scale k the drone plans for (NVS mvklow / 100, 2026-10-18 "guards off", docs/autonomy-
    a2b.md 38) as the last 'auto get' read it (else K_LOW): the Mac's floor check and its "at most F m now" use it. The
    person rule keeps K_LOW: a k under 1 is what the camera measures off a floor in view, not a setting."""
    v = cached_setting("mvklow")
    return v["value"] / 100.0 if v is not None and MV_KLOW_PCT[0] <= v["value"] <= MV_KLOW_PCT[1] else K_LOW


def cached_lat_rel():
    """The drone's sideways cap x D (NVS mvlat / 100; 2026-10-18) as the last 'auto get' read it: LAT_D_REL without
    one, None for 0 (no cap: the Mac refuses nothing sideways but the command limit)."""
    v = cached_setting("mvlat")
    if v is None or not MV_LAT_PCT[0] <= v["value"] <= MV_LAT_PCT[1]:
        return LAT_D_REL
    return None if v["value"] == 0 else v["value"] / 100.0


def move_guards_help():
    """The move help's sideways and ahead rules as the drone's cached settings have them (2026-10-18)."""
    lat = cached_lat_rel()
    side = (f"sideways at most {lat:g} x D per move" if lat is not None else
            "no sideways cap but the 1.50 m limit (the drone's mvlat 0)")
    mfront = mac_min_front()
    front = (f"forward only as far as ends {mfront:.2f} m from the scene ahead for an image expansion scale down to "
             f"{cached_k_low():g} (the drone's mvfloor + 0.05 m; the refusal gives the largest)" if mfront > 0.0 else
             f"no floor ahead but the drone's {MOVE_D_FLOOR_M:.2f} m (its mvfloor 0: a move may be planned up to the scene)")
    return side + "; " + front


def guards_view():
    """The MOVE guards the Mac mirrors from the last 'auto get' (2026-10-18, docs/autonomy-a2b.md 38), for the
    replies: its floor ahead (m, 0 off), the expansion scale k, the sideways cap x D (0 off), the drone's live front
    stop and progress margins as cached (0 off; absent without a cache: the drone's defaults), mvwmax, and the list
    of what is off."""
    mg = cached_margins()
    lat = cached_lat_rel()
    out = {"mac_floor_m": mac_min_front(), "k_low": cached_k_low(), "lat_rel": 0.0 if lat is None else lat,
           "mvwmax": fast_caps()["mvwmax"]}
    for k in ("mvfstop", "mvprogm", "mvprogr"):
        if k in mg:
            out[k] = mg[k]
    off = [name for name, v in (("floor", out["mac_floor_m"]), ("lat", out["lat_rel"]), ("mvfstop", mg.get("mvfstop", 1)),
                                ("mvprogm", mg.get("mvprogm", 1)), ("mvprogr", mg.get("mvprogr", 1))) if v == 0]
    if off:
        out["off"] = off
    return out


def cached_margins():
    """The drone's guard margins and its move fixes (2026-10-17, docs/autonomy-a2b.md 32-34) as the last 'auto get'
    read them: {key: value} for the keys it had (an older drone: none)."""
    out = {}
    for k in MV_MARGIN_KEYS:
        v = cached_setting(k)
        if v is not None:
            out[k] = v["value"]
    return out


def drift_measured():
    """The drone measures the drift since a move's end (NVS mvfix DRIFT) as the last 'auto get' cached mvfix (no cache:
    the default, on). Without it the drone's D at an acceptance is the last end's: a 'drift' of 0 would be no news."""
    fix = cached_margins().get("mvfix", MVFIX_DEFAULT)
    return bool(fix & MVFIX_DRIFT)


def cached_vmin():
    v = cached_setting("vmin")
    return None if v is None else {"v": v["value"] / 1000.0, "src": v["src"], "read_utc": v["read_utc"]}


def air_ms_limit():
    """The highest nonzero ARM airMaxMs the drone accepts: its NVS autoair (auto_fsm.c armCmd answers RANGE
    above it), as the last 'auto get' read it, or the default AIR_MS_DEFAULT ms without one."""
    v = cached_setting("autoair")
    if v is not None and AIR_MS[0] <= v["value"] <= AIR_MS[1]:
        return v["value"], f"autoair {v['value']} ms as 'auto get' read it at {v['read_utc']}"
    return AIR_MS_DEFAULT, (f"the default autoair {AIR_MS_DEFAULT} ms (no 'auto get' read: run "
                            "'python3 tools/espfly_diag.py auto get' over USB)")


def check_air_ms(air):
    """ARM airMaxMs: 0 (the drone's NVS autoair) or AIR_MS[0]..autoair; it may only lower the NVS value."""
    if air:
        top, src = air_ms_limit()
        if not AIR_MS[0] <= air <= top:
            raise ValueError(f"air_ms must be 0 (the drone's autoair) or {AIR_MS[0]}..{top}: a nonzero value may "
                             f"only lower the drone's autoair ({src}); the drone refuses a higher one RANGE")
    return air


def info_path(sock_path):
    return sock_path + ".json"


def read_daemon_info(sock_path):
    try:
        with open(info_path(sock_path)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def status_view(s):
    return None if s is None else dict(s, bootNonce=hex32(s["bootNonce"]))


def auto_view(a):
    return None if a is None else dict(a, bootNonce=hex32(a["bootNonce"]))


def send_kill_burst(link, session, seqs, gap_s=KILL_BURST_GAP_S):
    """One KILL with ACKREQ per seq, gap_s apart, and nothing else in between; returns how many went out."""
    sent = 0
    for i, seq in enumerate(seqs):
        if i:
            time.sleep(gap_s)
        try:
            link.send(T_KILL, FLAG_ACKREQ, seq, b"", session=session)
            sent += 1
        except LinkError as e:
            log(f"KILL: {e}")
    return sent


def kill_burst_state(states):
    """The state a KILL burst's ACKs report, given in burst order: KILLED if any shows it, else the last one's
    (None without any). A KILL ACK carries the state when it is sent; the state machine enters KILLED on its
    next 1 ms step, so the first ACK of a burst usually still shows the state before the KILL."""
    states = [s for s in states if s is not None]
    if not states:
        return None
    return "KILLED" if "KILLED" in states else states[-1]


class Cmd:
    """A command waiting for its ACK. done(result) is called once with the ACK's fields plus 'ok'
    (or 'error'), and 'sent' (whether it ever went out).

    Retries: the drone's sequence filter drops a seq at or below the newest it accepted unless it
    is one of the last 8 (a duplicate: the cached ACK is replayed). Heartbeats take a new seq every
    100 ms, so a retry under the same seq only helps when the ACK was lost. ARM, TAKEOFF and PHOTO
    are repeated under their seq twice (RETX), then sent under a new seq; a new seq sent while an
    earlier one has no answer is 'blind', and its answer is read knowing the first may have acted
    (ARM BAD_STATE in ARMED, TAKEOFF BAD_STATE in TAKEOFF/HOVER, PHOTO BUSY held_set with a new
    set's id). LAND, DISARM, CHUNKS and PHOTO_FREE are harmless twice and take a new seq on every
    retry.

    owner: the Client or Sortie the command is for. While it is still queued (never sent, or sent
    again after a BUSY), a command whose owner has ended (the client was answered or went away,
    the sortie ended) is withdrawn instead of sent."""

    def __init__(self, ptype, payload, done, owner=None):
        self.ptype, self.payload, self.done = ptype, payload, done
        self.owner = owner
        self.seqs = []
        self.sent_at = {}         # seq -> Mac time of its first send
        self.unanswered = set()   # seqs sent without any answer yet
        self.blind = set()        # seqs sent while an earlier one had no answer: the earlier may have acted
        self.retx_seq = None      # the seq a RETX repeats
        self.attempt = 0
        self.busy = 0
        self.not_before = 0.0
        self.next_retry = None
        self.recon = None         # reconciling through AUTO_STATUS (Daemon.start_reconcile)
        self.known_ids = None     # photo sets known when a PHOTO was first sent
        self.mv_seq0 = None       # MOVE / YAW: the drone's motion seq when it was queued (AUTO_STATUS mvSeq)
        self.cam_wake = 0         # ARM: sent again after GUARDS CAMERA alone (the camera waking, ARM_CAM_WAKE_*)
        self.withdrawn = None     # why it was withdrawn: never sent again; an ACK already on its way still counts
        self.finished = False
        self.result = None        # what done() got
        self.waiters = []         # more callbacks (the stop commands)

    @property
    def idempotent(self):
        return self.ptype in (T_LAND, T_DISARM, T_CHUNKS, T_PHOTO_FREE, T_STOP)

    @property
    def sent(self):
        return bool(self.seqs)

    def abandoned(self):
        return self.owner is not None and self.owner.ended()


class PhotoSet:
    """A photo set being collected: one the Mac asked for, or one the drone pushed on its own (the
    take-off photos). Frames are saved as they complete; PHOTO_FREE follows the last one."""

    def __init__(self, pid, n, gap_ms, source, now, out_dir):
        self.id, self.n, self.gap_ms, self.source = pid, n, gap_ms, source
        self.t0 = self.last_rx = now
        self.deadline = now + photo_set_timeout_s(n) + MAX_N * gap_ms / 1000.0
        self.out_dir = out_dir
        self.stamp = time.strftime("%Y%m%d-%H%M%S")
        self.saved, self.failed, self.rounds = {}, {}, {}
        self.refetched = set()
        self.max_seen = -1
        self.requests = 0         # CHUNKS without their ACK yet
        self.free = None          # "sent", then the PHOTO_FREE result
        self.waiters = []
        self.meta_rx = {}         # idx -> Mac time its first META arrived (the height at capture's fallback)

    def resolved(self):
        return self.n is not None and all(i in self.saved or i in self.failed for i in range(self.n))

    def photos(self):
        return [{"idx": i, "id": self.id, **self.saved[i]} for i in sorted(self.saved)]

    def failures(self):
        return [{"idx": i, "error": self.failed[i]} for i in sorted(self.failed)]


class Client:
    """One connection on the daemon socket: one JSON request line in, one JSON reply line out. The reply
    is buffered and sent as the socket takes it, never blocking the daemon's loop; a client that does not
    take it within CLIENT_SEND_S is dropped.

    A read EOF alone does not mean the client is gone: a client may shut down its write side after its
    request (shutdown(SHUT_WR), as 'nc -N' does) and still read the reply. It is gone once a write to it
    fails: when the peer has closed its socket, even a zero-length send fails (EPIPE), and that is the
    probe check() uses."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b""
        self.request = None
        self.out = bytearray()     # reply bytes not sent yet
        self.replied = False       # its one reply is queued
        self.closed = False
        self.eof = False           # the peer shut down its write side (or closed): it is no longer read
        self.gone = False          # the peer went away before its reply
        self.deadline = None       # the reply must be out by then

    @property
    def done(self):
        """No reply can be sent any more (one is queued, or the connection is closed)."""
        return self.replied or self.closed

    def ended(self):
        return self.done or self.check()

    def check(self):
        """True once the peer has gone, learnt now and without blocking: a decision that depends on the
        client being there (sending a TAKEOFF for it) looks first. Only after a read EOF can the peer be gone;
        then a zero-length send tells a half-closed client (it succeeds) from a closed one (it fails)."""
        if self.closed:
            return self.gone
        if not self.eof:
            try:
                data = self.sock.recv(1, socket.MSG_PEEK)
            except (BlockingIOError, InterruptedError):
                return False
            except OSError:
                self.lost()         # reset
                return self.gone
            if data:
                return False
            self.eof = True
        try:
            self.sock.send(b"")
        except (BlockingIOError, InterruptedError):
            return False            # its buffer is full: still there
        except OSError:
            self.lost()             # EPIPE: the peer has closed its socket
        return self.gone

    def reply(self, obj):
        if self.done:
            return
        self.replied = True
        self.out += (json.dumps(obj) + "\n").encode()
        self.deadline = time.monotonic() + CLIENT_SEND_S
        self.flush()

    def flush(self):
        """Sends what the socket takes now (it is non-blocking); closes once the reply is out."""
        try:
            while self.out:
                n = self.sock.send(self.out)
                if n <= 0:
                    break
                del self.out[:n]
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            self.out.clear()
        if not self.out:
            self.close()

    def lost(self):
        """The peer closed or reset the connection (a write to it failed, or a read did)."""
        if not self.replied:
            self.gone = True
        self.out.clear()
        self.close()

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                self.sock.close()
            except OSError:
                pass


class Daemon:
    def __init__(self, link, sock_path, photo_dir, allow_flight=False, keys_fd=None, events_path=None, out=None,
                 host=None, no_level=False):
        self.link = link
        self.sock_path = sock_path
        self.photo_dir = photo_dir
        self.allow_flight = allow_flight
        self.keys_fd = keys_fd
        self.events_path = events_path
        self.events_file = None
        self.out = out if out is None or isinstance(out, LineWriter) else LineWriter(out)
        self.host = host
        self.server = None
        self.bound = False
        self.clients, self.waiters, self.events = [], [], []
        self.clients_dropped = 0
        self.events_base = 0
        # the drone
        self.boot = None
        self.rebooted = self.session_lost = self.link_lost = False
        self.hello_req = None
        self.last_rehello = -1e9
        self.status = self.auto = None
        self.status_count = self.auto_count = 0
        self.sortie_n = 0           # ARMs seen (a sortie's photos and motions belong to it)
        self.sortie_air_s = 0.0     # the current sortie's air clock as last seen (FLIGHT)
        self.air_clock_mismatch = None   # this sortie's drone air limit (ms) when it is not AIR_HARD_MS
        self.soak_air_s = self.soak_from = None   # the last FLIGHT sortie's air time and when it ended (SOAK hint)
        self.auto_rx = None
        self.last_state = S_IDLE
        self.last_rx = 0.0
        self.clock_off = self.clock_t = None    # Mac monotonic time minus the drone's snapshot time (AUTO_STATUS upMs)
        # commands
        self.flight = None          # the ARM, TAKEOFF or PHOTO in flight
        self.flight_free_at = 0.0   # when the flight slot last became free
        self.stop = None            # the LAND or DISARM being sent until confirmed
        self.queue, self.aux = [], []
        self.sorties = []
        self.settles = []           # TAKEOFFs that may have acted after their sortie ended (settle_takeoff)
        self.seq_owner = {}
        self.kill_seqs, self.kill_acks = {}, {}
        self.arm_token = None
        self.takeoff_gap = PHOTO_GAP_AIR_MS[1]
        # photos
        self.sets = {}
        self.image_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='photo-save')
        self.image_jobs = {}
        self.done_sets = deque(maxlen=16)
        self.freed_ids = set()
        # timers
        self.next_hb = self.next_status = 0.0
        self.hb_sent = self.hb_late = self.send_errors = 0
        self.quit_deadline = None
        self.quit_signal = None
        self.stop_now = False
        # SET_HEIGHT and the level line (2026-10-15, docs/autonomy-a2b.md 23)
        self.no_level = no_level    # --no-level: no line drawn (the JSON's level block is still written)
        self.ring = deque(maxlen=STATUS_RING_N)   # (upMs, hMm, state name, Mac rx time, evCount or None): this boot's
                                                  # AUTO_STATUSes
        self.esp_off = self.esp_t = None          # Mac time minus the drone's esp_timer (STATUS upMs, META captureUpMs)
        self.level, self.level_mtime = None, None   # the level setting, re-read by its mtime before each photo
        self.photo_log = deque(maxlen=64)         # the saved photos: JSON, sortie, capture time, the estimate then
        self.last_sh = None                       # this sortie's latest accepted SET_HEIGHT
        self.sh_goal = None                       # a clipped set-height's target (this sortie): the default target of
                                                  # the next set-height until reached, a vertical MOVE or --to
        self.sh_used = None                       # (Mac time, why): photos up to then are used: the latest set-height
                                                  # accepted, or whose outcome is unknown (it may have been applied)
        self.check_warned = None                  # the height check (Mac time) height_check_due last warned about
        self.wait_mono = None                     # Mac time the hover wait started (no check yet: counted from it)
        self.prev_final = None                    # the early-returned motion's final end, for the next reply (2026-10-17)
        self.d_end = None                         # {mvSeq, D}: the drone's D when the latest motion's end was first seen
        self.early_seq = None                     # fast moves: the motion (mvSeq) a reply returned early from (its
                                                  # settle or bob still running): a new motion waits behind it
        self.behind = []                          # those waiting commands (STOP withdraws them)
        self.sortie_key = None                    # bootNonce-pid-sortie: a photo of another sortie or boot is refused

    # ------------------------------------------------ life cycle

    def start(self):
        """Binds the client socket, then says this daemon's one HELLO."""
        self.bind()
        try:
            h = self.link.hello()
        except BaseException:
            self.close(quiet=True)
            raise
        now = time.monotonic()
        self.last_rx = self.next_hb = self.next_status = now
        self.boot = h["bootNonce"]
        self.write_info()
        self.event("hello", session=hex32(h["session"]), bootNonce=hex32(h["bootNonce"]), fw=h["fw"],
                   rtt_ms=h["rtt_ms"], allow_flight=self.allow_flight, sock=self.sock_path)
        return h

    def bind(self):
        path = self.sock_path
        if len(path.encode()) > SOCK_MAX_LEN:
            raise LinkError(f"socket path {path} is too long for a Unix socket ({len(path)} bytes); use --sock")
        private_dir(os.path.dirname(os.path.abspath(path)))
        if os.path.exists(path):
            try:
                other = daemon_request(path, {"op": "ping"}, CLIENT_PROBE_S)
            except DaemonError as e:
                if e.kind != "refused":
                    raise LinkError(f"daemon socket {path} present but not answering ({e}): another daemon is "
                                    "stopped or hung; end it first", sock=path) from None
                os.unlink(path)     # stale: nothing listens behind it
            else:
                raise LinkError(f"a daemon already runs on {path} (pid {other.get('pid')})", sock=path)
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        old = os.umask(0o177)
        try:
            srv.bind(path)
        finally:
            os.umask(old)
        os.chmod(path, 0o600)
        srv.listen(16)
        srv.setblocking(False)
        self.server, self.bound = srv, True

    def write_info(self):
        info = {"pid": os.getpid(), "host": self.host, "drone": self.link.addr[0], "port": self.link.addr[1],
                "iface": self.link.iface, "session": hex32(self.link.session),
                "bootNonce": hex32(self.boot or 0), "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
        try:
            with open(info_path(self.sock_path), "w") as f:
                json.dump(info, f)
        except OSError as e:
            log(f"daemon: cannot write {info_path(self.sock_path)}: {e}")

    def run(self):
        try:
            while not self.stop_now:
                self.step()
        finally:
            self.close()

    def request_stop(self):
        self.stop_now = True

    def close(self, quiet=False):
        for c in self.clients:
            c.reply({"ok": False, "error": "the daemon is exiting"})
            c.close()               # what the socket did not take at once is dropped
        self.clients = []
        if self.server is not None:
            self.server.close()
            self.server = None
        if self.bound:
            for p in (self.sock_path, info_path(self.sock_path)):
                try:
                    os.unlink(p)
                except OSError:
                    pass
            self.bound = False
        if not quiet:
            self.event("daemon_exit", hb_sent=self.hb_sent, hb_late=self.hb_late, send_errors=self.send_errors,
                       clients_dropped=self.clients_dropped,
                       out_dropped=self.out.dropped if self.out is not None else 0)
        if self.events_file is not None:
            self.events_file.close()
            self.events_file = None
        if self.out is not None:
            self.out.close()
        self.link.close()
        self.image_worker.shutdown(wait=True, cancel_futures=True)

    # ------------------------------------------------ loop

    def step(self, max_wait=0.05):
        """One pass of the loop. Nothing in it waits on a client or on the terminal: replies go out as
        the client sockets take them (select's write set), output through LineWriter threads."""
        now = time.monotonic()
        timeout = max(0.0, min(max_wait, self.next_hb - now))
        self.clients = [c for c in self.clients if not c.closed]
        rl = [self.link.sock, self.server] + [c.sock for c in self.clients if not c.eof]
        wl = [c.sock for c in self.clients if c.out]
        if self.keys_fd is not None:
            rl.append(self.keys_fd)
        try:
            ready, writable, _ = select.select(rl, wl, [], timeout)
        except (OSError, ValueError) as e:
            log(f"daemon: select: {e}")
            ready = writable = []
        now = time.monotonic()
        if self.keys_fd is not None and self.keys_fd in ready:
            self.read_keys(now)            # first: a KILL goes out before any other packet
        if self.quit_signal is not None:
            sig, self.quit_signal = self.quit_signal, None
            self.request_quit(sig, now)
        self.heartbeat(now)
        if self.link.sock in ready:
            self.receive(now)
        if self.server in ready:
            self.accept()
        for c in list(self.clients):
            if c.closed:
                continue
            if c.sock in ready:
                self.read_client(c, now)
            if not c.closed and c.out and c.sock in writable:
                c.flush()
            if not c.closed and c.out and now >= c.deadline:
                self.clients_dropped += 1
                log(f"daemon: a client did not take its reply ({len(c.out)} bytes left) within "
                    f"{CLIENT_SEND_S:g} s; dropped")
                c.out.clear()
                c.close()
        self.service(now)

    def heartbeat(self, now):
        if now < self.next_hb or not self.link.session:
            return
        late = now - self.next_hb
        if late > HB_LATE_S:
            self.hb_late += 1
            log(f"daemon: heartbeat {late * 1000:.0f} ms late")
        if self.send(T_HB, 0, self.link.next_seq(), hb_payload(int(time.time() * 1000))):
            self.hb_sent += 1
        self.next_hb += HB_S
        if self.next_hb <= now:
            self.next_hb = now + HB_S

    def send(self, ptype, flags, seq, payload=b"", session=None):
        try:
            self.link.send(ptype, flags, seq, payload, session=session)
            return True
        except LinkError as e:
            self.send_errors += 1
            if self.send_errors in (1, 10, 100) or self.send_errors % 1000 == 0:
                log(f"daemon: send ({self.send_errors} failed): {e}")
            return False

    def send_status_req(self, now):
        self.send(T_STATUS_REQ, 0, self.link.next_seq())
        self.next_status = now + STATUS_REQ_S

    def receive(self, now):
        for _ in range(64):
            pkt = self.link.poll_nowait()
            if pkt is None:
                break
            self.last_rx = now
            if self.link_lost:
                self.link_lost = False
                self.event("link_ok")
            if pkt.type in (T_META, T_CHUNK) and self.link.session and pkt.session == self.link.session \
                    and len(pkt.payload) >= 4:
                self.note_photo_packet(pkt, now)
        for s in self.link.statuses:
            self.on_status(s, now)
        self.link.statuses.clear()
        for a in self.link.auto_statuses:
            self.on_auto_status(a, now)
        self.link.auto_statuses.clear()
        self.service_acks(now)

    def service(self, now):
        self.finish_image_jobs(now)
        self.timers(now)
        self.pump(now)
        for s in list(self.sets.values()):
            self.set_step(s, now)
        self.pump(now)
        if now >= self.next_status and self.link.session:
            self.send_status_req(now)
        if not self.link_lost and now - self.last_rx > LINK_LOST_S:
            self.link_lost = True
            self.event("link_lost", silent_s=round(now - self.last_rx, 1), state=self.state_name())
        waiters, self.waiters = self.waiters, []
        self.waiters = [w for w in waiters if not w(now)] + self.waiters
        if self.quit_deadline is not None and (self.touched_down() or now >= self.quit_deadline):
            self.stop_now = True

    def timers(self, now):
        for st in list(self.settles):
            if now >= st["deadline"]:
                self.settles.remove(st)
                self.settle_decide(st, None)
            elif now >= st["next"]:
                st["next"] = now + SETTLE_REQ_S
                self.send_status_req(now)
        c = self.flight
        if c is not None:
            if c.recon is not None:
                self.recon_step(c, now)
            elif now >= c.next_retry:
                self.retry(c, now)
        s = self.stop
        if s is not None and not s.finished and now >= s.next_retry:
            self.retry_stop(s, now)
        for c in list(self.aux):
            if now >= c.next_retry:
                self.retry(c, now)
        h = self.hello_req
        if h is not None:
            if self.link.session != h["session"]:
                self.hello_req = None
                hk = self.link.hello_ack
                self.event("hello", session=hex32(self.link.session), bootNonce=hex32(hk["bootNonce"]), fw=hk["fw"],
                           rehello=True)
                self.check_boot(hk["bootNonce"], "HELLO_ACK")
                self.write_info()
            elif now >= h["next"]:
                h["attempt"] += 1
                if h["attempt"] > Link.RETRIES:
                    log("daemon: no HELLO_ACK to the new HELLO; trying again on the next BAD_SESSION")
                    self.hello_req = None
                else:
                    self.send(T_HELLO, FLAG_RETX, h["seq"], hello_payload(h["nonce"]), session=0)
                    h["next"] = now + Link.RETRY_S

    # ------------------------------------------------ what the drone says

    def state_name(self):
        return name_of(STATE_NAMES, self.last_state)

    def active(self):
        return self.last_state != S_IDLE

    def touched_down(self):
        a = self.auto
        return a is None or a["state"] in (S_IDLE, S_KILLED) or a["vzPhase"] in (VZ_DONE, VZ_CUT)

    def wait_left(self, now):
        """The hover wait left, ms: a lower bound (a saturated status, 65535, means 65.5 s or more)."""
        a = self.auto
        if a is None or not a["waiting"]:
            return 0
        return max(0, a["waitLeftMs"] - int((now - self.auto_rx) * 1000))

    def wait_saturated(self):
        """The drone's waitLeftMs is saturated: more than 65.5 s are left (a wait of up to 120 s)."""
        a = self.auto
        return a is not None and a["waiting"] and a["waitLeftMs"] >= WAIT_SAT_MS

    def check_boot(self, nonce, via):
        """A new bootNonce: the drone rebooted (mission aborted). Reported with the old and new nonce and the last
        state and reason seen; ARM and TAKEOFF are refused until the drone reports IDLE in this boot (clear_latches;
        until 2026-10-15 until the daemon was restarted). The drone's own guards decide after that (an in-air reset:
        the reset lock keeps the motors off until the user clears it over USB)."""
        if self.boot is None:
            self.boot = nonce
        elif nonce != self.boot:
            old, self.boot = self.boot, nonce
            self.rebooted = True
            self.clock_off = None           # a new boot: a new snapshot clock
            self.esp_off = None             # ... and a new esp_timer
            self.ring.clear()
            self.level_invalidate("a new bootNonce: the drone rebooted (or the battery was swapped)")
            a = self.auto
            self.event("drone_rebooted", old=hex32(old), new=hex32(nonce), via=via,
                       last_state=None if a is None else a["stateName"], last_reason=None if a is None else
                       a["reasonName"])
            self.drop_photo_state(old)
            log("daemon: the drone rebooted (mission aborted); ARM and TAKEOFF wait until it reports IDLE")

    def clear_latches(self, state, nonce):
        """The reboot and session latches end once the drone reports IDLE in this daemon's session and boot."""
        if state != S_IDLE or not self.link.session or nonce != self.boot:
            return
        if self.rebooted:
            self.rebooted = False
            self.event("reboot_cleared", bootNonce=hex32(nonce),
                       note="the drone is idle in its new boot: ARM goes to its guards again (after an in-air reset "
                            "its reset lock holds until the user clears it over USB)")
        if self.session_lost:
            self.session_lost = False
            self.event("session_back", note="the drone is idle in this daemon's session again")

    def on_status(self, s, now=None):
        self.status = s
        self.status_count += 1
        self.last_state = s["state"]
        self.check_boot(s["bootNonce"], "STATUS")
        if s["bootNonce"] == self.boot:
            self.note_esp_clock(s, time.monotonic() if now is None else now)
        self.clear_latches(s["state"], s["bootNonce"])

    def on_auto_status(self, a, now):
        prev = self.auto
        self.auto, self.auto_rx = a, now
        self.auto_count += 1
        if not a["mvActive"] and a["mvSeq"] and (prev is None or prev["mvActive"] or prev["mvSeq"] != a["mvSeq"]):
            # a motion's end, first seen: its D (a refused command later moves the drone's D by the drift so far,
            # mvfix DRIFT; the review of 2026-10-17: drift_since_last_end_m then held only the rest of it)
            self.d_end = {"mvSeq": a["mvSeq"], "D": a["sceneD_m"] if (a["mvFlags"] & MVF_D_TRACK and
                                                                       a["mvKindName"] == "MOVE") else None}
        self.last_state = a["state"]
        self.check_boot(a["bootNonce"], "AUTO_STATUS")
        if a["bootNonce"] == self.boot:
            self.note_clock(a, now)
            if not self.ring or a["upMs"] > self.ring[-1][0]:
                self.ring.append((a["upMs"], a["hMm"], a["stateName"], now, a["evCount"] if a.get("hr") else None))
        if prev is None or a["state"] != prev["state"] or a["transitions"] != prev["transitions"]:
            self.event("state", **{"from": None if prev is None else prev["stateName"]}, to=a["stateName"],
                       reason=a["reasonName"], vzLandReason=a["vzLandReasonName"], landTrip=a["landTripName"],
                       mode=a["modeName"], transitions=a["transitions"])
            if a["state"] in (S_IDLE, S_ARMED):
                self.early_seq = None   # the drone's mvSeq restarts at ARM: an old early return is not this sortie's
                self.prev_final = None
                self.d_end = None
            if a["state"] == S_IDLE:
                self.arm_token = None
                if prev is not None and prev["state"] != S_ARMED and self.sortie_air_s:
                    self.soak_air_s, self.soak_from = self.sortie_air_s, now
                self.sortie_air_s = 0.0
            if a["state"] == S_ARMED:
                self.sortie_n += 1
                self.sortie_air_s = 0.0
                self.air_clock_mismatch = None
                self.sortie_key = f"{hex32(self.boot or 0)}-{os.getpid()}-{self.sortie_n}"
                self.last_sh = self.sh_used = self.sh_goal = None
            # the level setting goes stale on what may move the camera (mechanical facts; level-cal again)
            if prev is not None and prev["state"] == S_LAND and a["state"] == S_IDLE:
                self.level_invalidate("a landing")
            elif a["state"] == S_KILLED and (prev is None or prev["state"] != S_KILLED):
                self.level_invalidate("KILLED (the motors cut in the air)")
            elif a["state"] == S_LAND and a["reasonName"] == "COLLISION" and \
                    (prev is None or prev["reasonName"] != "COLLISION"):
                self.level_invalidate("a contact in the climb (LAND COLLISION)")
        if a["mode"] == MODE_FLIGHT and a["state"] in (S_TAKEOFF, S_HOVER, S_LAND, S_KILLED):
            self.sortie_air_s = max(self.sortie_air_s, a["air_s"])
        if a["airClockMismatch"] and self.air_clock_mismatch is None:
            self.air_clock_mismatch = a["airMs"] + 100 * a["airLeftDs"]
            self.event("air_clock_mismatch", drone_ms=self.air_clock_mismatch, mac_ms=AIR_HARD_MS,
                       airMs=a["airMs"], airLeftDs=a["airLeftDs"])
        if a["hr"] and (prev is None or not prev.get("hr") or a["evCount"] != prev["evCount"]):
            # what the drone did on its own (hold and report): one event per new report, oldest first
            n_new = 3 if prev is None or not prev.get("hr") else (a["evCount"] - prev["evCount"]) & 0xFFFF
            for e in reversed(a["hold"]["events"][:min(n_new, 3)]):
                self.event("drone_event", kind=e["name"], **{k: v for k, v in e.items() if k != "name"})
                if e["code"] == EV_CONTACT and prev is not None:
                    self.level_invalidate("an in-air contact")
        if a["waiting"] and not (prev is not None and prev["waiting"]):
            self.wait_mono = now
            self.event("waiting", waitLeftMs=a["waitLeftMs"], waitLeftSaturated=a["waitLeftMs"] >= WAIT_SAT_MS,
                       photoId=a["photoId"], capture=a["captureName"])
        if a["interactive"] and a["state"] == S_HOVER and a["waiting"] and a["bootNonce"] == self.boot:
            self.height_check(a, now)
        pid = a["photoId"]
        if pid and a["capture"] != CAP_NONE and a["state"] in (S_TAKEOFF, S_HOVER, S_LAND) \
                and pid not in self.sets and pid not in self.freed_ids:
            self.new_set(pid, None, self.takeoff_gap, "drone", now)
        c = self.flight
        if c is not None and c.recon is not None and self.fresh(a, c.recon["after_ms"]):
            self.settle(c, a, now)
        s = self.stop
        # (a LAND SINK confirms nothing: it may resume; the drone turns a LAND or DISARM it receives into LAND_CMD /
        # DISARM_CMD, so until that shows, the stop is sent again: a lost LAND must not leave a SINK that resumes)
        if s is not None and not s.finished and self.flight is None and s.seqs and a["state"] in STOP_STATES[s.ptype] \
                and not (a["state"] == S_LAND and a["reason"] == RS_SINK) \
                and self.fresh(a, self.drone_ms(max(s.sent_at[s.seqs[0]], self.flight_free_at))):
            self.finish(s, {"result": R_OK, "resultName": "OK", "state": a["state"], "stateName": a["stateName"],
                            "detail": None}, confirmed="AUTO_STATUS")
        for st in list(self.settles):
            if self.fresh(a, st["after_ms"]):
                self.settles.remove(st)
                self.settle_decide(st, a)
        self.clear_latches(a["state"], a["bootNonce"])

    def note_clock(self, a, now):
        """The drone's snapshot clock (AUTO_STATUS upMs, the stabilizer's tick) against the Mac's: now - upMs is
        never below the true offset (delay and snapshot age only add), so its minimum is the estimate; it
        rises by CLOCK_LEAK while no closer sample comes, which follows a drift either way."""
        off = now - a["upMs"] / 1000.0
        if self.clock_off is None:
            self.clock_off = off
        else:
            self.clock_off = min(off, self.clock_off + (now - self.clock_t) * CLOCK_LEAK)
        self.clock_t = now

    def drone_ms(self, t):
        """Mac monotonic time t on the drone's snapshot clock (ms), or None before any AUTO_STATUS."""
        return None if self.clock_off is None else (t - self.clock_off) * 1000.0

    def note_esp_clock(self, s, now):
        """The drone's esp_timer clock (STATUS upMs; META captureUpMs is on it too) against the Mac's, as note_clock does
        for the AUTO_STATUS snapshots' tick clock. The two drone clocks are not the same (the tick starts with the
        scheduler, a boot-constant offset): a photo's capture time reaches the snapshots through the Mac's clock."""
        off = now - s["upMs"] / 1000.0
        if self.esp_off is None:
            self.esp_off = off
        else:
            self.esp_off = min(off, self.esp_off + (now - self.esp_t) * CLOCK_LEAK)
        self.esp_t = now

    def capture_status(self, meta, rx):
        """(the height estimate m, the state name, the source) when a frame was captured: 'capture' (META captureUpMs
        on the esp_timer clock, through the Mac's clock onto the AUTO_STATUS snapshots, interpolated), else
        'arrival' (the snapshot received nearest the META's arrival: 0.1-1 s late), else (None, None, None)."""
        ring = self.ring
        if not ring:
            return None, None, None
        if self.esp_off is not None and self.clock_off is not None:
            t = meta["captureUpMs"] + (self.esp_off - self.clock_off) * 1000.0
            prev = None
            for e in ring:
                if e[0] >= t:
                    if prev is not None and e[0] - prev[0] <= 2 * CAPTURE_MATCH_MS:
                        f = (t - prev[0]) / (e[0] - prev[0]) if e[0] > prev[0] else 0.0
                        return (prev[1] + f * (e[1] - prev[1])) / 1000.0, (prev if f < 0.5 else e)[2], "capture"
                    if e[0] - t <= CAPTURE_MATCH_MS:
                        return e[1] / 1000.0, e[2], "capture"
                    break
                prev = e
            else:
                if t - prev[0] <= CAPTURE_MATCH_MS:
                    return prev[1] / 1000.0, prev[2], "capture"
        if rx is None:
            return None, None, None
        e = min(ring, key=lambda e: abs(e[3] - rx))
        return e[1] / 1000.0, e[2], "arrival"

    def capture_events(self, meta, rx):
        """The drone's event count (evCount) at a frame's capture: the latest snapshot at or before it (else the one
        nearest the META's arrival); None when unknown (an older drone, no snapshot)."""
        ring = self.ring
        if not ring:
            return None
        e = None
        if self.esp_off is not None and self.clock_off is not None:
            t = meta["captureUpMs"] + (self.esp_off - self.clock_off) * 1000.0
            before = [x for x in ring if x[0] <= t]
            e = before[-1] if before else None
        if e is None and rx is not None:
            e = min(ring, key=lambda x: abs(x[3] - rx))
        return e[4] if e is not None and len(e) > 4 else None

    def capture_mono(self, meta, rx):
        """A frame's capture on the Mac's monotonic clock (the esp_timer offset), else its META's arrival."""
        if self.esp_off is not None:
            return meta["captureUpMs"] / 1000.0 + self.esp_off
        return rx

    def level_setting(self):
        """The level setting (level-cal), re-read when its file changed."""
        try:
            m = os.stat(level_path()).st_mtime_ns
        except OSError:
            self.level, self.level_mtime = None, None
            return None
        if m != self.level_mtime:
            self.level, self.level_mtime = load_level(), m
        return self.level

    def level_invalidate(self, why):
        """Something that may have moved the camera happened (a new boot, a landing, a contact, a KILL): the level
        setting is stale until the next level-cal (the line drawn dotted). A mechanical fact, no decision; nothing
        without a setting (the default is always stale)."""
        doc = load_level()
        if doc is None:
            return
        t = time.time()
        doc.update(invalidated_t=t, invalidated_utc=utc_str(t), invalidated_why=why)
        try:
            write_json_atomic(level_path(), doc)
            append_jsonl(level_history_path(), {"utc": utc_str(t), "t": t, "kind": "invalidated", "why": why})
        except OSError as e:
            log(f"daemon: level setting {level_path()}: {e}")
        self.event("level_invalidated", why=why)

    def fresh(self, a, after_ms):
        """a is a snapshot the drone took after after_ms (its clock) + FRESH_MARGIN_MS: newer than the command
        sent then. A stalled stabilizer publishes nothing, so the link keeps pushing its last snapshot: that
        one is never fresh."""
        return after_ms is not None and a["upMs"] > after_ms + FRESH_MARGIN_MS

    def service_acks(self, now):
        if not self.link.acks:
            return
        acks, self.link.acks = self.link.acks, {}
        for seq, ack in acks.items():
            c = self.seq_owner.get(seq)
            if c is not None:
                self.on_ack(c, seq, ack, now)
            elif seq in self.kill_seqs:
                del self.kill_seqs[seq]
                self.kill_acks[seq] = ack["stateName"]
            elif ack["result"] == R_BAD_SESSION:
                self.on_bad_session(now, ack.get("state"))

    def on_bad_session(self, now, state=None):
        """BAD_SESSION: our session is gone (the drone rebooted, or another HELLO replaced ours). The ACK carries the
        drone's state now (state; None: none known, our last status decides). IDLE: a new HELLO (a reboot then shows
        as a new bootNonce: drone_rebooted). Active: never a HELLO (it would end a flight with SESSION): reported once
        (session_lost; the drone lands on HB_LOST if it was ours), and a HELLO follows once a BAD_SESSION shows the
        drone IDLE (until 2026-10-15: restart the daemon)."""
        if self.hello_req is not None:
            return
        idle = state == S_IDLE if state is not None else not self.active()
        if idle:
            if now - self.last_rehello >= 1.0:
                self.start_rehello(now)
        elif not self.session_lost:
            self.session_lost = True
            self.event("session_lost", state=self.state_name() if state is None else name_of(STATE_NAMES, state),
                       note="the drone has another session; no new HELLO while it is active (a flight of ours lands "
                            "on HB_LOST); the daemon says HELLO again once the drone is idle")
            log("daemon: session lost while the drone is not idle; heartbeat continues; no new HELLO")

    def start_rehello(self, now):
        """BAD_SESSION while idle (a reboot, or another HELLO): a new session. Never while active."""
        self.last_rehello = now
        nonce = secrets.randbits(32)
        self.link.nonce = nonce
        seq = self.link.next_seq()
        self.hello_req = {"seq": seq, "nonce": nonce, "attempt": 0, "next": now + Link.RETRY_S,
                          "session": self.link.session}
        self.event("rehello", reason="BAD_SESSION while idle")
        self.send(T_HELLO, 0, seq, hello_payload(nonce), session=0)

    # ------------------------------------------------ commands

    def queue_flight(self, c, now, priority=False):
        if priority:
            self.queue.insert(0, c)
        else:
            self.queue.append(c)
        self.pump(now)

    def aux_send(self, ptype, payload, done, now):
        c = Cmd(ptype, payload, done)
        self.aux.append(c)
        self.transmit(c, now)
        return c

    def stopping(self):
        return self.stop is not None and not self.stop.finished

    def pump(self, now):
        """Sends the next queued flight command once the flight slot is free. Whether its owner has ended
        (its client answered or gone, its sortie over: abandoned(), which also ends a sortie past its
        deadline) is checked at the moment of sending, so a sortie that has ended never sends a TAKEOFF."""
        while True:
            for c in [c for c in list(self.queue) if c.abandoned()]:       # abandoned() may end a sortie
                self.finish(c, error="withdrawn: " + ("its sortie ended" if isinstance(c.owner, Sortie) else
                                                      "its client was answered or went away"), withdrawn=True)
                if c in self.queue:
                    self.queue.remove(c)        # finished before (never expected): never sent
            if self.flight is not None or self.stopping():
                return      # one command at a time; nothing new while a LAND or DISARM is being confirmed
            # a PHOTO goes only once every earlier set is freed (PHOTO_FREE acknowledged)
            c = next((c for c in self.queue if c.not_before <= now and not (c.ptype == T_PHOTO and self.sets)),
                     None)
            if c is None:
                return
            if c.abandoned():
                continue        # its owner ended since (a callback above): withdrawn by the sweep
            self.queue.remove(c)
            if c.known_ids is None:
                c.known_ids = set(self.sets) | self.freed_ids
            self.flight = c
            self.transmit(c, now)
            return

    def transmit(self, c, now, retx=False):
        if retx:
            seq = c.retx_seq
        else:
            seq = self.link.next_seq()
            if c.unanswered:
                c.blind.add(seq)        # an earlier copy has no answer: it may have acted
            c.seqs.append(seq)
            c.sent_at[seq] = now
            c.unanswered.add(seq)
            c.retx_seq = seq
            self.seq_owner[seq] = c
        self.send(c.ptype, FLAG_ACKREQ | (FLAG_RETX if retx else 0), seq, c.payload)
        c.next_retry = now + Link.RETRY_S

    def retry(self, c, now):
        if isinstance(c.owner, Sortie) and not c.withdrawn:
            # A sortie's ARM or TAKEOFF is never sent again once the sortie has ended (here: its deadline
            # passed since the last pass); its answer may still come until WITHDRAW_WAIT_S after its last send.
            c.owner.ended()
            if c.finished or (c.withdrawn and now < c.next_retry):
                return
        if c.withdrawn:
            return self.finish(c, error=c.withdrawn, withdrawn=True)
        c.attempt += 1
        if c.attempt > Link.RETRIES:
            return self.finish(c, error=f"no ACK for {TYPE_NAMES[c.ptype]} after {1 + Link.RETRIES} tries")
        self.transmit(c, now, retx=not c.idempotent and c.attempt <= 2 and c.retx_seq is not None)

    def requeue(self, c, now, delay=BUSY_RETRY_S):
        if self.flight is c:
            self.flight = None
            self.flight_free_at = now
        if c.withdrawn or c.abandoned():
            return self.finish(c, error=c.withdrawn or "withdrawn: its client or sortie ended", withdrawn=True)
        c.attempt, c.not_before = 0, now + delay
        self.queue.insert(0, c)

    def finish(self, c, ack=None, error=None, **extra):
        if c.finished:
            return
        c.finished = True
        for s in c.seqs:
            self.seq_owner.pop(s, None)
        if self.flight is c:
            self.flight = None
            self.flight_free_at = time.monotonic()
        for pending in (self.queue, self.aux):
            if c in pending:
                pending.remove(c)
        r = dict(ack) if ack else {"result": None, "resultName": None, "state": None, "stateName": None,
                                   "detail": None}
        r.update(extra)
        r["type"] = TYPE_NAMES[c.ptype]
        r["ok"] = error is None and r["result"] == R_OK
        r["sent"] = c.sent
        if error:
            r["error"] = error
        c.result = r
        c.done(r)
        for w in c.waiters:
            w(r)

    def on_ack(self, c, seq, ack, now):
        if c.finished:
            return
        c.unanswered.discard(seq)
        res, state = ack["result"], ack["state"]
        if res == R_BAD_SESSION:
            self.on_bad_session(now, state)
            return self.finish(c, ack, error="BAD_SESSION: " + RESULT_HINTS["BAD_SESSION"])
        if c is self.stop:
            return self.on_stop_ack(c, seq, ack, now)
        if c.ptype not in FLIGHT_TYPES:
            return self.finish(c, ack)
        if res == R_BUSY:
            self.seq_owner.pop(seq, None)       # answered; a BUSY from the link is not cached, a RETX gets nothing
            return self.on_busy(c, seq, ack, now)
        if seq in c.blind and res == R_BAD_STATE:
            if c.ptype == T_ARM and state == S_ARMED:
                self.finish(c, ack, error="ARM took effect but its ACK (with the token) was lost; DISARM sent: "
                                          "arm again")
                return self.disarm_quietly(now, "ARM acted on but its ACK was lost")
            if c.ptype == T_TAKEOFF and state in (S_TAKEOFF, S_HOVER):
                return self.finish(c, dict(ack, result=R_OK, resultName="OK"), resent=True)
            if c.ptype == T_SET_HEIGHT and c.mv_seq0 is not None:
                # a copy sent again found the first one's correction running (BUSY) or its 2 s bound (SH_RATE): the
                # first was accepted when the latest status shows a correction one motion up (the detail's high bits
                # are the estimate here, not the motion seq)
                why = (ack["detail"] >> 8) & 0xFF
                a = self.auto
                if why in (MV_WHY_NAMES.index("BUSY"), MV_WHY_NAMES.index("SH_RATE")) and a is not None and \
                        a.get("mvCorrection") and a["mvSeq"] == ((c.mv_seq0 + 1) & 0xFFFF):
                    return self.finish(c, dict(ack, result=R_OK, resultName="OK", detail=ack["state"]), resent=True,
                                       old_unknown=True)
            elif c.ptype in MOTION_TYPES and c.mv_seq0 is not None:
                st, why, mseq = mv_detail(ack["detail"])
                if why == MV_WHY_NAMES.index("BUSY") and mseq == ((c.mv_seq0 + 1) & 0xFFFF):
                    # an earlier copy was accepted (motion seq one up): this one found it running
                    return self.finish(c, dict(ack, result=R_OK, resultName="OK", detail=st | mseq << 16),
                                       resent=True)
        if c.ptype == T_ARM and res == R_GUARDS and ack["detail"] == G_CAMERA and c.cam_wake < ARM_CAM_WAKE_RESENDS:
            # The camera slept on the ground and this ARM woke it (auto_link.c): the same ARM again, new seq.
            c.cam_wake += 1
            self.event("arm_cam_wake", retries=c.cam_wake)
            return self.requeue(c, now, delay=ARM_CAM_WAKE_S)
        self.finish(c, ack, **({"cam_wake_retries": c.cam_wake} if c.cam_wake else {}))

    def on_busy(self, c, seq, ack, now):
        """ACK BUSY for an ARM, TAKEOFF or PHOTO, by kind (busy_kind). Only taken-unknown may have acted:
        it is reconciled, never sent again blindly. Every other BUSY was not processed: sent again under
        a new seq, at most BUSY_RESENDS times; a PHOTO refused because a photo set is held waits for that
        set's PHOTO_FREE."""
        kind, val = busy_kind(ack["detail"])
        if c.recon is not None:
            return              # already reconciling: an answer to another copy changes nothing
        if c.withdrawn:
            return self.finish(c, ack, error=c.withdrawn, withdrawn=True, busy=True)
        if kind == "taken_unknown":
            c.unanswered.add(seq)           # its outcome is unknown
            self.event("busy_taken_unknown", type=TYPE_NAMES[c.ptype], seq=seq)
            return self.start_reconcile(c, seq, now, "taken_unknown")
        if c.ptype == T_PHOTO and kind == "held_set" and val:
            if seq in c.blind and val not in c.known_ids and ack["state"] != S_IDLE:
                # A set newer than this PHOTO is held: its first copy may have started it. AUTO_STATUS tells.
                return self.start_reconcile(c, seq, now, "held_set", pid=val)
            return self.photo_held(c, val, ack["state"], now, ack)
        c.busy += 1
        if c.busy > BUSY_RESENDS:
            return self.finish(c, ack, error=f"{TYPE_NAMES[c.ptype]}: the drone answered BUSY {c.busy} times "
                                             f"({kind}, detail 0x{ack['detail']:08x})", busy=True)
        if kind == "pending":
            own = [s for s in c.seqs if s != seq and s in c.unanswered and (s & DETAIL_VALUE_MASK) == val]
            if own:
                # An earlier copy of this very command is the one pending on the drone: its ACK follows
                # (a RETX of its seq gets the cached result). A new seq would only be answered BUSY again.
                c.retx_seq, c.attempt = own[-1], 0
                c.next_retry = now + PENDING_WAIT_S
                return
        self.requeue(c, now)

    def photo_held(self, c, pid, state, now, ack=None):
        """PHOTO refused BUSY held_set: photo set pid is held on the drone; a PHOTO goes only once it is freed.
        A set this daemon collects: wait for its PHOTO_FREE. One freed here but still held there (its
        PHOTO_FREE got lost), or in IDLE one an earlier run left: PHOTO_FREE it again, in any state. One not
        seen yet: its frames are on the way."""
        c.busy += 1
        if c.busy > BUSY_PHOTO_MAX:
            return self.finish(c, ack, error=f"PHOTO: photo set {pid} is still held on the drone (BUSY {c.busy} "
                                             "times)", busy=True)
        if pid in self.sets:
            return self.requeue(c, now)     # pump() holds it until that set's PHOTO_FREE is acknowledged
        if pid in self.freed_ids or state == S_IDLE:
            log(f"daemon: photo set {pid} is still held on the drone; freeing it")
            self.freed_ids.add(pid)
            self.aux_send(T_PHOTO_FREE, photo_free_payload(pid), lambda r: None, now)
        self.requeue(c, now, delay=0.1)

    def start_reconcile(self, c, seq, now, why, pid=None):
        """BUSY taken-unknown (the command may have acted), or a PHOTO sent again that found a new set held
        (its first copy may have started it): an AUTO_STATUS newer than the send decides (fresh()), never a
        resend. The command keeps the flight slot meanwhile; a STATUS_REQ every RECONCILE_STATUS_S brings an
        AUTO_STATUS."""
        c.recon = {"why": why, "seq": seq, "pid": pid, "after_ms": self.drone_ms(c.sent_at[seq]),
                   "until": now + RECONCILE_S, "next_req": now}
        self.recon_step(c, now)

    def recon_step(self, c, now):
        r = c.recon
        if c.withdrawn:
            return self.finish(c, error=c.withdrawn, withdrawn=True)
        if now >= r["until"]:
            return self.settle(c, None, now)
        if now >= r["next_req"]:
            r["next_req"] = now + RECONCILE_STATUS_S
            self.send_status_req(now)

    def settle(self, c, a, now):
        """Decides a reconciling command from a fresh AUTO_STATUS a, or None when none came in time. Done only
        on evidence that this command was processed: lastCmdType its type with OK, or the state it leads to
        (ARM, TAKEOFF), or for a PHOTO a photo set newer than every set known when it was first sent. A
        stale snapshot (a stalled stabilizer) is never read: fresh() rejects it."""
        r, t = c.recon, c.ptype
        if a is not None:
            mine = a["lastCmdType"] == CMD_OF_TYPE[t]
            ok = mine and a["lastCmdResult"] == R_OK
            st = {"state": a["state"], "stateName": a["stateName"]}
            done = dict(st, result=R_OK, resultName="OK")
            if t == T_PHOTO:
                pid = a["photoId"]
                new = bool(pid) and pid not in c.known_ids and a["capture"] != CAP_NONE
                if r["why"] == "held_set":
                    c.recon = None
                    if ok and new and pid == r["pid"]:
                        return self.finish(c, dict(done, detail=pid), reconciled=True, resent=True)
                    return self.photo_held(c, r["pid"], a["state"], now)    # not ours: wait until it is freed
                if ok and new:
                    c.recon = None
                    return self.finish(c, dict(done, detail=pid), reconciled=True)
                if ok and a["capture"] in (CAP_NONE, CAP_FAILED):
                    c.recon = None      # accepted, but no photo set was started (detail 0)
                    return self.finish(c, dict(done, detail=0), reconciled=True)
                if ok:
                    return              # accepted, its set not reported yet: the next fresh AUTO_STATUS tells
            elif t in FLIGHT_TYPES and t not in (T_ARM, T_TAKEOFF):     # MOVE, YAW, STOP, SET_HEIGHT
                if ok:
                    c.recon = None
                    if t == T_SET_HEIGHT:   # (its ACK's high bits are the old estimate, which the status does not keep)
                        return self.finish(c, dict(done, detail=a["state"]), reconciled=True, old_unknown=True)
                    code = a["mvState"] if t == T_STOP else 0
                    return self.finish(c, dict(done, detail=a["state"] | code << 8 | a["mvSeq"] << 16),
                                       reconciled=True)
            elif ok or a["state"] in EFFECT_STATES.get(t, ()):
                c.recon = None
                return self.finish(c, dict(done, detail=None if t == T_ARM else a["state"]), reconciled=True)
            if mine:
                c.recon = None
                res = a["lastCmdResult"]
                detail = a["failedGuards"] if res == R_GUARDS else a["state"]
                if t == T_SET_HEIGHT:
                    detail = a["state"] | a["mvWhy"] << 8
                elif t in MOTION_TYPES:
                    detail = a["state"] | a["mvWhy"] << 8 | a["mvSeq"] << 16
                return self.finish(c, dict(st, result=res, resultName=result_name(res), detail=detail),
                                   reconciled=True)
        elif r["why"] == "held_set":
            c.recon = None
            return self.photo_held(c, r["pid"], self.last_state, now)
        c.recon = None
        self.finish(c, error=f"{TYPE_NAMES[t]} was taken by the drone but its result was lost (BUSY taken-unknown), "
                             "and " + (f"no AUTO_STATUS newer than it came within {RECONCILE_S:g} s" if a is None else
                                       "the AUTO_STATUS newer than it does not show it done") +
                             ": check 'state' before sending anything again",
                    resultName="UNKNOWN", auto=auto_view(a if a is not None else self.auto))

    # -------- the stop commands (LAND, DISARM)

    def stop_request(self, ptype, why, now, done=None):
        """LAND or DISARM. The drone never refuses either and both are harmless twice, so the request is sent
        at once, whatever else is in flight (it never waits behind another command), then again under a new
        seq every STOP_RETRY_S until its ACK OK, or until an AUTO_STATUS newer than it (and than the last
        flight command's end) shows the state it leads to; a BUSY of any kind only means 'again'. Every
        running sortie ends (except one landing itself), and every ARM, TAKEOFF and PHOTO is withdrawn:
        queued, dropped; in flight, never sent again. A LAND asked for while a DISARM is being sent turns it
        into a LAND; a DISARM joins a LAND."""
        s = self.stop
        new = s is None or s.finished
        if new:
            s = self.stop = Cmd(ptype, b"", None)
            s.done = lambda r, s=s: self.stop_done(s, r)
        elif ptype == T_LAND and s.ptype != T_LAND:
            s.ptype = T_LAND
            new = True
        if done is not None:
            s.waiters.append(done)
        if not new:
            return s
        s.why, s.give_up = why, now + STOP_GIVE_UP_S
        what = f"{TYPE_NAMES[s.ptype]} requested ({why})"
        self.event("land_sent" if s.ptype == T_LAND else "disarm_sent", why=why, state=self.state_name())
        for so in list(self.sorties):
            if so.phase != "landing":
                so.end(False, f"the sortie ended: {what}")
        self.withdraw_all("withdrawn: " + what, now)
        if not s.finished:
            self.transmit(s, now)
            s.next_retry = now + STOP_RETRY_S
        return s

    def on_stop_ack(self, s, seq, ack, now):
        if ack["result"] == R_BUSY:
            self.seq_owner.pop(seq, None)
            s.busy += 1
            s.next_retry = min(s.next_retry, now + BUSY_RETRY_S)
            return
        self.finish(s, ack)

    def retry_stop(self, s, now):
        if now >= s.give_up:
            return self.finish(s, error=f"{TYPE_NAMES[s.ptype]} not confirmed within {STOP_GIVE_UP_S:g} s (no ACK, "
                                        "no AUTO_STATUS showing it): the drone's heartbeat and air-time rules "
                                        "remain")
        s.attempt += 1
        self.transmit(s, now)
        s.next_retry = now + STOP_RETRY_S

    def stop_done(self, s, r):
        self.event("land_ack" if s.ptype == T_LAND else "disarm_ack", result=r.get("resultName"),
                   error=r.get("error"), confirmed=r.get("confirmed", "ACK" if r["ok"] else None),
                   tries=len(s.seqs), busy=s.busy)

    def withdraw(self, c, why, now):
        """No more sends for c. Queued, or reconciling: finished now (its callback learns whether it was
        ever sent). In flight: finished when its ACK comes, or WITHDRAW_WAIT_S after its last send."""
        if c.finished:
            return
        if c is not self.flight or c.recon is not None or not c.sent:
            return self.finish(c, error=why, withdrawn=True)
        c.withdrawn = why
        c.next_retry = max(c.next_retry, max(c.sent_at.values()) + WITHDRAW_WAIT_S)

    def withdraw_all(self, why, now):
        for c in [c for c in self.queue if c.ptype in FLIGHT_TYPES]:
            self.withdraw(c, why, now)
        if self.flight is not None:
            self.withdraw(self.flight, why, now)

    def undo(self, ptype, r, why, now):
        """A sortie's ARM or TAKEOFF that did not lead on (the sortie ended, or the TAKEOFF failed): nothing
        more is sent for it but what stops it on the ground. An ARM answered OK is DISARMed; one with no clear
        answer as arm_unknown_stops says (never a LAND). A TAKEOFF that may have acted is decided from a fresh
        AUTO_STATUS (settle_takeoff: ARMED, DISARM; flying, it flies on: takeoff_late; never a LAND since
        2026-10-15, "Claude decides"); one never sent or refused leaves the drone ARMED by the sortie's ARM:
        DISARM."""
        res = r.get("result")
        if res == R_BAD_SESSION:
            return          # the session is gone: the drone lands or disarms on its own (SESSION, HB_LOST)
        maybe = r.get("sent") and res in (None, R_OK, R_BUSY)
        if ptype == T_TAKEOFF:
            if maybe:
                self.settle_takeoff(why, now)
            else:
                self.stop_request(T_DISARM, why, now)
        elif res == R_OK and r.get("sent"):
            self.stop_request(T_DISARM, why, now)
        else:
            for t in self.arm_unknown_stops(r):
                self.stop_request(t, why, now)

    def arm_unknown_stops(self, r):
        """The stops for an ARM that was sent and may have acted but got no usable answer: no ACK after the
        retries (or withdrawn before one came), BUSY beyond BUSY_RESENDS, or a taken-unknown reconcile ending
        UNKNOWN (result None or BUSY). The drone may then be ARMED with a token nobody knows. An ARM cannot make it
        fly, so the latest AUTO_STATUS decides: none yet, IDLE, ARMED or KILLED: DISARM (never refused; ARMED ->
        IDLE, harmless on the ground); TAKEOFF, HOVER or LAND: nothing (something else flies, or a SINK that may
        resume: a DISARM in the air is a LAND). Never a LAND for an ARM (it sent one with the DISARM until
        2026-10-15). [] when the ARM cannot have acted, or the drone flies."""
        if not r.get("sent") or r.get("result") not in (None, R_BUSY):
            return []
        a = self.auto
        return [] if a is not None and a["state"] in (S_TAKEOFF, S_HOVER, S_LAND) else [T_DISARM]

    def arm_unknown_note(self, stops):
        """The reply's words for arm_unknown_stops."""
        if stops:
            return f"; the ARM may have acted: {' and '.join(TYPE_NAMES[t] for t in stops)} sent"
        return (f"; the ARM may have acted, but the drone shows {self.state_name()}: nothing sent (never a LAND for "
                "an ARM)")

    def settle_takeoff(self, why, now):
        """A TAKEOFF that may have acted after its sortie ended (its client gone, its timeout, its answer lost):
        decided from a fresh AUTO_STATUS (a STATUS_REQ every SETTLE_REQ_S, up to SETTLE_S; settle_decide)."""
        self.settles.append({"why": why, "after_ms": self.drone_ms(now), "deadline": now + SETTLE_S, "next": now})
        self.event("takeoff_settle", why=why, state=self.state_name())

    def settle_decide(self, st, a):
        """ARMED (the TAKEOFF was not processed): DISARM. TAKEOFF or HOVER: it flies on and holds, no LAND
        (takeoff_late; the battery, a lost link and k still land it; Claude decides from 'state'). Anything else,
        or no fresh AUTO_STATUS within SETTLE_S: nothing sent (a DISARM in the air is a LAND; an ARMED drone
        goes back to IDLE by itself after its ARM timeout)."""
        now = time.monotonic()
        state = None if a is None else a["state"]
        if state == S_ARMED:
            self.stop_request(T_DISARM, st["why"], now)
        elif state in (S_TAKEOFF, S_HOVER):
            self.event("takeoff_late", why=st["why"], state=a["stateName"],
                       note="the TAKEOFF took effect after its sortie ended: the drone flies on and holds (no LAND by "
                            "the Mac since 2026-10-15): 'state', a photo, then decide")
        else:
            self.event("takeoff_settled" if a is not None else "takeoff_unknown", why=st["why"],
                       state=None if a is None else a["stateName"],
                       note=None if a is not None else f"no fresh AUTO_STATUS within {SETTLE_S:g} s: nothing sent; "
                                                       "check 'state'")

    def disarm_quietly(self, now, why):
        self.stop_request(T_DISARM, why, now)

    def land(self, why, now, done=None):
        return self.stop_request(T_LAND, why, now, done)

    def kill(self, source):
        seqs = [self.link.next_seq() for _ in range(KILL_BURST_N)]
        for s in seqs:
            self.kill_seqs[s] = source
        sent = send_kill_burst(self.link, self.link.session, seqs)
        now = time.monotonic()
        why = f"KILL ({source})"
        for so in list(self.sorties):
            so.end(False, f"the sortie ended: {why}")
        self.withdraw_all("cancelled by " + why, now)
        self.event("kill", source=source, sent=sent, state=self.state_name())
        if len(self.kill_seqs) > 200:
            for s in sorted(self.kill_seqs)[:-100]:
                del self.kill_seqs[s]
        return seqs

    def refusal(self, mode=None, confirm=False):
        if self.rebooted:
            return ("the drone rebooted (mission aborted): ARM and TAKEOFF wait until it reports IDLE in its new boot "
                    "(the daemon says HELLO again by itself); check 'state'")
        if self.session_lost:
            return ("the drone's session changed while it was active (another program, or a reboot): the daemon says "
                    "HELLO again once the drone is idle; check 'state'")
        if not self.link.session:
            return "no session with the drone"
        if mode == MODE_FLIGHT and not self.allow_flight:
            return "FLIGHT refused: the daemon was started without --allow-flight"
        if mode == MODE_FLIGHT and not confirm:
            return "FLIGHT refused: --confirm-flight not given"
        return None

    # ------------------------------------------------ photos

    def new_set(self, pid, n, gap_ms, source, now, out_dir=None):
        s = PhotoSet(pid, n, gap_ms, source, now, out_dir or self.photo_dir)
        self.sets[pid] = s
        self.event("photo_set", id=pid, frames=n, source=source, state=self.state_name())
        return s

    def note_photo_packet(self, pkt, now):
        pid, idx = struct.unpack_from("<HB", pkt.payload)
        if pid in self.freed_ids:
            self.link.photos.pop(pid, None)     # a late packet of a freed set
            return
        s = self.sets.get(pid)
        if s is None:
            s = self.new_set(pid, None, self.takeoff_gap, "drone", now)
        if pkt.type == T_META and s.n is None and 1 <= pkt.payload[3] <= MAX_N:
            s.n = pkt.payload[3]
        if pkt.type == T_META:
            s.meta_rx.setdefault(idx, now)
            frame = self.link.photos.get(s.id, {}).get(idx)
            if frame is not None and frame.meta and frame.meta.get("size", 0) > 2 * FRAME_SIZE:
                # VGA has 16x QQVGA's pixels; keep the heartbeat independent while the bulk transfer drains.
                s.deadline = max(s.deadline, now + 20.0)
        s.last_rx = now
        if idx < MAX_N:
            s.max_seen = max(s.max_seen, idx)

    def set_step(self, s, now):
        if s.free is not None:
            return
        frames = self.link.photos.get(s.id, {})
        for i in range(s.n or 0):
            if i in s.saved or i in s.failed or (s.id, i) in self.image_jobs:
                continue
            f = frames.get(i)
            if f is None or not f.complete():
                continue
            try:
                data = f.assemble()
            except FrameError as e:
                if e.final or i in s.refetched:
                    self.frame_failed(s, i, str(e))
                else:
                    s.refetched.add(i)
                    f.chunks.clear()
                continue
            self.save(s, i, f.meta, data, now)
        if s.resolved():
            return self.resolve(s, now)
        if now >= s.deadline:
            if s.n is None:
                s.n = max(1, s.max_seen + 1)
            for i in range(s.n):
                if i not in s.saved and i not in s.failed:
                    f = frames.get(i)
                    self.frame_failed(s, i, "timed out: " + ("no META, " if f is None or f.meta is None else "")
                                      + (f"{len(f.missing())} of {f.count} chunks missing" if f else "nothing received"))
            return self.resolve(s, now)
        if not s.requests and now - s.last_rx >= QUIET_S:
            self.rerequest(s, frames, now)

    def rerequest(self, s, frames, now):
        """As receive_photo: CHUNKS for what is missing once the set has been quiet for QUIET_S;
        a frame without its META first asks for the META (CHUNKS count 0)."""
        n = s.n if s.n is not None else max(1, s.max_seen + 1)
        latest = max((i for i in range(n) if i in frames and frames[i].seen()), default=-1)
        for i in range(n):
            if i in s.saved or i in s.failed or (s.id, i) in self.image_jobs:
                continue
            f = frames.get(i)
            if i > latest and now - s.t0 < i * s.gap_ms / 1000.0 + DUE_S:
                continue        # not captured yet
            rnd = s.rounds.get(i, 0)
            if rnd >= MAX_ROUNDS:
                self.frame_failed(s, i, ("no META (cannot check the CRC), " if f is None or f.meta is None else "")
                                  + f"{len(f.missing()) if f else '?'} chunks missing after {rnd} re-request rounds")
                continue
            s.rounds[i] = rnd + 1
            for first, count in [(0, 0)] if f is None or f.meta is None else missing_runs(f.missing()):
                s.requests += 1
                self.aux_send(T_CHUNKS, chunks_payload(s.id, i, count, first),
                              lambda r, i=i, rnd=rnd + 1: self.chunks_done(s, i, rnd, r), now)
        s.last_rx = now

    def chunks_done(self, s, i, rnd, r):
        s.requests -= 1
        s.last_rx = time.monotonic()
        if i in s.saved or i in s.failed or r.get("result") is None:
            return
        if r["result"] == R_NO_PHOTO:
            self.frame_failed(s, i, "the drone has no data for this frame (NO_PHOTO: not captured, or the set was "
                                    "freed)")
        elif r["result"] == R_BUSY and s.rounds.get(i) == rnd:
            s.rounds[i] = rnd - 1       # not captured yet, or its resend queue is full: the round is not used
        elif r["result"] != R_OK:
            self.frame_failed(s, i, f"CHUNKS refused: {r['resultName']}")

    def frame_failed(self, s, i, why):
        s.failed[i] = why
        self.event("photo_failed", id=s.id, idx=i, error=why)

    def save(self, s, i, meta, data, now):
        a = self.auto
        rx = s.meta_rx.get(i)
        h_cap, st_cap, h_src = self.capture_status(meta, rx)
        cap_mono = self.capture_mono(meta, rx)
        ev_cap = self.capture_events(meta, rx)
        sh = self.last_sh
        extra = {"link": {"session": hex32(self.link.session), "bootNonce": hex32(self.boot or 0),
                          "fw": (self.link.hello_ack or {}).get("fw"), "key_fp": self.link.fp,
                          "rerequest_rounds": s.rounds.get(i, 0),
                          "received_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                          "received_t": round(time.time(), 3)},
                 "autonomy": {"source": s.source, "state": self.state_name(), "waitLeftMs": self.wait_left(now),
                              "waitLeftSaturated": self.wait_saturated(),
                              "mode": a["modeName"] if a else None, "airMs": a["airMs"] if a else None,
                              "h_m": a["h_m"] if a else None,
                              # SET_HEIGHT (docs/autonomy-a2b.md 23): the drone's estimate when this frame was
                              # captured (set-height's h_at), the state then, this sortie
                              "h_est_capture_m": None if h_cap is None else round(h_cap, 3), "h_src": h_src,
                              "state_capture": st_cap, "sortie_key": self.sortie_key,
                              "capture_mono": None if cap_mono is None else round(cap_mono, 3),
                              "evCount_capture": ev_cap,   # set-height warns of events since (2026-10-16 review)
                              "last_set_height": None if sh is None else
                              {k: sh[k] for k in ("utc", "offset_m", "new_m")}}}
        if a is not None and a["interactive"]:
            # where the drone looked from: heading from the take-off heading (+ clockwise; a bearing in this
            # image adds to it), the commanded height, the scene distance the next MOVE would use
            extra["autonomy"].update({"yawOff_deg": a["yawOff_deg"], "hTarget_m": a["hTarget_m"],
                                      "D_m": a["sceneD_m"], "mvFlags": a["mvFlagNames"], "mvSeq": a["mvSeq"],
                                      "mvState": a["mvStateName"], "air_s": a["air_s"],
                                      "airLeft_s": a["airLeft_s"]})
        t0 = time.monotonic()
        try:
            # the level line: the setting as level-cal last wrote it; a ground photo (captured in IDLE / ARMED) uses
            # the ground tilt, every other the flight one
            kind = "ground" if (st_cap or self.state_name()) in LEVEL_GROUND_STATES else "flight"
            level = level_info(meta["pitch_deg"], meta["roll_deg"], kind, self.level_setting(), hex32(self.boot or 0),
                               scale=meta["width"] // FRAME_W if meta["width"] >= 320 else X4_SCALE,
                               draw=level_drawing_on() and not self.no_level)
            if os.path.abspath(s.out_dir).startswith(os.path.abspath(DEFAULT_DIR) + os.sep):
                private_dir(DEFAULT_DIR)
            os.makedirs(s.out_dir, exist_ok=True)
            future = self.image_worker.submit(save_frame, s.out_dir, s.stamp, meta, data, extra, level=level)
            self.image_jobs[(s.id, i)] = (future, s, meta, extra, level, kind, h_cap, h_src, cap_mono, t0,
                                        self.boot, self.sortie_key)
            s.deadline = max(s.deadline, now + 5.0)
            return
        except Exception as e:      # disk, Pillow
            return self.frame_failed(s, i, f"saving failed: {type(e).__name__}: {e}")
    def finish_image_jobs(self, now):
        for key, job in list(self.image_jobs.items()):
            future, s, meta, extra, level, kind, h_cap, h_src, cap_mono, t0, boot, sortie = job
            if not future.done():
                continue
            del self.image_jobs[key]
            if boot != self.boot or sortie != self.sortie_key or self.sets.get(s.id) is not s:
                continue  # A completed old image never becomes a new-sortie observation.
            i = key[1]
            try:
                files = future.result()
            except Exception as e:
                self.frame_failed(s, i, f"saving failed: {type(e).__name__}: {e}")
                continue
            self.publish_saved(s, i, files, meta, extra, level, kind, h_cap, h_src, cap_mono, t0, now)

    def publish_saved(self, s, i, files, meta, extra, level, kind, h_cap, h_src, cap_mono, t0, now):
        publish_started = time.monotonic()
        # (the photo replies carry what Claude reads off the line: its row, the tilt, stale, the estimate then)
        s.saved[i] = dict(files, color=bool(meta.get("color")), level_row_x4=level["row_x4_center"],
                          tilt_deg=level["tilt_deg_used"], tilt_kind=kind, level_stale=level["stale"],
                          h_est_capture_m=extra["autonomy"]["h_est_capture_m"], h_src=h_src)
        self.photo_log.append({"json": files["json"], "sortie_key": self.sortie_key, "boot": hex32(self.boot or 0),
                               "capture_mono": cap_mono, "saved_mono": now, "h_est_capture_m": h_cap,
                               "h_src": h_src})
        self.event("photo", id=s.id, idx=i, png=files["png"], png_x4=files["png_x4"], json=files["json"],
                   color=bool(meta.get("color")), state=self.state_name(), waitLeftMs=self.wait_left(now), source=s.source,
                   save_ms=round((time.monotonic() - publish_started) * 1000.0, 1),
                   image_save_wall_ms=round((time.monotonic() - t0) * 1000.0, 1), level_row_x4=level["row_x4_center"],
                   tilt_deg=level["tilt_deg_used"], tilt_kind=kind, level_stale=level["stale"],
                   h_est_capture_m=extra["autonomy"]["h_est_capture_m"], h_src=h_src)

    def resolve(self, s, now):
        self.event("photo_set_done", id=s.id, saved=sorted(s.saved), failed={str(k): v for k, v in s.failed.items()},
                   source=s.source)
        waiters, s.waiters = s.waiters, []
        for w in waiters:
            w(s)
        s.free = "sent"
        self.aux_send(T_PHOTO_FREE, photo_free_payload(s.id), lambda r: self.freed(s, r), now)

    def freed(self, s, r):
        s.free = r.get("resultName") or "no ACK"
        if self.sets.get(s.id) is not s:
            return      # dropped with its boot meanwhile (drop_photo_state): its id must not come back as freed
        self.sets.pop(s.id, None)
        self.done_sets.append(s)
        self.freed_ids.add(s.id)
        self.link.photos.pop(s.id, None)
        self.event("photo_freed", id=s.id, result=s.free)
        self.pump(time.monotonic())

    def find_set(self, pid):
        return self.sets.get(pid) or next((s for s in self.done_sets if s.id == pid), None)

    def drop_photo_state(self, old_boot):
        """A new bootNonce (check_boot): the drone's photo set ids start at 1 again, so every set of the old boot
        is forgotten. Until 2026-10-22 done_sets and freed_ids outlived a reboot: a PHOTO whose new id matched a
        set saved in the old boot was answered with that boot's files (find_set), and the new frames were dropped
        as 'a late packet of a freed set'. A set still being collected cannot complete (its frames are gone
        with the boot): its missing frames fail and its waiters are answered now; its PHOTO_FREE is not sent
        (the new boot holds no such set)."""
        held = list(self.sets.values())
        dropped = {"held": sorted(self.sets), "done": sorted({s.id for s in self.done_sets}),
                   "freed": sorted(self.freed_ids), "saving": sorted({k[0] for k in self.image_jobs})}
        self.sets.clear()
        self.done_sets.clear()
        self.freed_ids.clear()
        self.image_jobs.clear()         # finished later or not, their files never become this boot's photos
        self.link.photos.clear()
        for c in self.queue:
            if c.ptype == T_PHOTO:
                c.known_ids = None      # the ids known when it is sent belong to this boot
        for s in held:
            if s.free is not None:
                continue                # resolved; its PHOTO_FREE was in flight
            if s.n is None:
                s.n = max(1, s.max_seen + 1)
            for i in range(s.n):
                if i not in s.saved and i not in s.failed:
                    self.frame_failed(s, i, f"the drone rebooted (bootNonce {hex32(old_boot)} -> "
                                            f"{hex32(self.boot)}): photo set {s.id} is gone with its boot")
            self.event("photo_set_done", id=s.id, saved=sorted(s.saved),
                       failed={str(k): v for k, v in s.failed.items()}, source=s.source, rebooted=True)
            waiters, s.waiters = s.waiters, []
            for w in waiters:
                w(s)
            s.free = "dropped"
        if any(dropped.values()):
            self.event("photo_sets_dropped", old=hex32(old_boot), new=hex32(self.boot), **dropped)

    def fsm_set_ready(self, a):
        """The state machine's latest photo set (AUTO_STATUS photoId) is saved, or there is none."""
        pid = a["photoId"]
        if not pid or a["capture"] == CAP_NONE:
            return True
        s = self.find_set(pid)
        return s.resolved() if s is not None else pid in self.freed_ids

    # ------------------------------------------------ terminal

    def read_keys(self, now):
        try:
            data = os.read(self.keys_fd, 64)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            data = b""
        if not data:
            self.keys_fd = None     # end of input
            return
        if b"k" in data or b"K" in data:
            self.kill("key")
        for ch in data.decode("ascii", "replace"):
            if ch in "kK\r\n ":
                continue
            if ch in "lL":
                self.land("key", now)
            elif ch in "sS":
                log(self.status_line(now))
            elif ch in "qQ\x03":
                self.request_quit("key", now)
            else:
                log(KEY_HELP)

    def request_quit(self, why, now):
        if self.quit_deadline is not None or not self.active():
            self.stop_now = True
            return
        self.quit_deadline = now + QUIT_LAND_S
        self.event("quit", why=why, state=self.state_name())
        log("daemon: quitting: LAND sent, waiting up to 3 s for the touchdown (q again: quit now)")
        self.land("quit", now)

    def status_line(self, now):
        a = self.auto
        s = f"session {self.link.session:08x} {self.state_name()}"
        if a is not None:
            s += (f" ({a['reasonName']}) {a['modeName']} vz {a['vzPhaseName']} h {a['h_m']:+.2f} m "
                  f"vbat {a['vbat_v']:.2f} V air {a['airMs']} ms")
            if a["waiting"]:
                s += " waiting >= 65.5 s" if self.wait_saturated() else f" waiting {self.wait_left(now)} ms"
            if a["interactive"] and (a["mvState"] or a["state"] == S_HOVER):
                d = "-" if a["sceneD_m"] is None else f"{a['sceneD_m']:.2f}"
                kind = "SET_HEIGHT" if a.get("mvCorrection") else a["mvKindName"]
                s += (f" mv {kind}#{a['mvSeq']} {a['mvStateName']}/{a['mvEndName']} D {d} m "
                      f"yaw {a['yawOff_deg']:+.0f} hT {a['hTarget_m']:.2f} air left {a['airLeft_s']:.0f} s")
            if a["flagNames"]:
                s += " [" + " ".join(a["flagNames"]) + "]"
        s += f" | link rx {now - self.last_rx:.1f} s ago, hb {self.hb_sent} sent {self.hb_late} late"
        if self.sets:
            s += f", photo sets held {sorted(self.sets)}"
        if self.rebooted or self.session_lost:
            s += " | " + ("REBOOTED " if self.rebooted else "") + ("SESSION LOST" if self.session_lost else "")
        return s

    # ------------------------------------------------ clients

    def event(self, name, **fields):
        ev = {"event": name, "n": self.events_base + len(self.events), "t": round(time.time(), 3), **fields}
        self.events.append(ev)
        if len(self.events) > EVENTS_KEPT:
            drop = len(self.events) - EVENTS_KEPT
            del self.events[:drop]
            self.events_base += drop
        line = json.dumps(ev)
        if self.out is not None:
            self.out.write_line(line)       # never blocks: a LineWriter thread writes it
        if self.events_path:
            try:
                if self.events_file is None:
                    private_dir(os.path.dirname(os.path.abspath(self.events_path)))
                    self.events_file = open(self.events_path, "a")
                self.events_file.write(line + "\n")
                self.events_file.flush()
            except OSError as e:
                log(f"daemon: events file {self.events_path}: {e}")
                self.events_path = None
        return ev

    def accept(self):
        for _ in range(16):
            try:
                sock, _ = self.server.accept()
            except (BlockingIOError, InterruptedError):
                return
            except OSError as e:
                log(f"daemon: accept: {e}")
                return
            sock.setblocking(False)
            self.clients.append(Client(sock))

    def read_client(self, c, now):
        try:
            data = c.sock.recv(65536)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            return c.lost()         # reset
        if not data:
            # EOF: the client shut down its write side after its request (shutdown(SHUT_WR), 'nc -N') or
            # closed the connection. It is no longer read; a zero-length send tells which (Client.check).
            # Half-closed, it still gets its reply, and what it started goes on; closed, it is gone.
            c.eof = True
            if c.check() or c.request is not None or c.replied:
                return
            if not c.buf.strip():
                return c.lost()     # nothing asked
            data = b"\n"            # its last request line came without the newline
        if c.request is not None or c.replied:
            return
        c.buf += data
        if b"\n" not in c.buf:
            if len(c.buf) > 65536:
                c.reply({"ok": False, "error": "bad request: longer than 64 KB"})
            return
        try:
            req = json.loads(c.buf.split(b"\n", 1)[0])
            if not isinstance(req, dict):
                raise ValueError("not a JSON object")
        except ValueError as e:
            return c.reply({"ok": False, "error": f"bad request: {e}"})
        c.request = req
        fn = self.OPS.get(req.get("op"))
        if fn is None:
            return c.reply({"ok": False, "error": f"unknown op {req.get('op')!r}", "ops": sorted(self.OPS)})
        try:
            fn(self, c, req, now)
        except (ValueError, TypeError, KeyError) as e:
            c.reply({"ok": False, "error": f"bad request: {type(e).__name__}: {e}"})

    def defer(self, c, check, deadline, on_timeout):
        """Replies when check(now) returns a reply, or on_timeout() at the deadline; dropped once the client
        is gone (closed, not merely half-closed: Client.check)."""
        def w(now):
            if c.ended():
                return True
            r = check(now)
            if r is None and now >= deadline:
                r = on_timeout()
            if r is None:
                return False
            c.reply(r)
            return True
        if not w(time.monotonic()):
            self.waiters.append(w)

    def last_height_check(self):
        """(Mac time, what) of this sortie's latest height check: the latest photo saved or set-height accepted."""
        lp = self.photo_log[-1] if self.photo_log and self.photo_log[-1]["sortie_key"] == self.sortie_key else None
        sh = self.last_sh
        checks = ([(lp["saved_mono"], "photo")] if lp is not None else []) + \
                 ([(sh["mono"], "set-height")] if sh is not None else [])
        return max(checks) if checks else None

    def height_check(self, a, now):
        """height_check_due (a warning, once per lapse; the successor of the ceiling rule's warning): the latest height
        check older than the runbook's cadence at the commanded height (information only: Claude decides)."""
        h = a["hTarget_m"]
        due_s = HEIGHT_CHECK_HIGH_S if h > HEIGHT_CHECK_HIGH_M + 1e-9 else HEIGHT_CHECK_S
        last = self.last_height_check()
        t0 = last[0] if last is not None else self.wait_mono
        if t0 is None or now - t0 < due_s or self.check_warned == t0:
            return
        self.check_warned = t0
        age = now - t0
        self.event("height_check_due", since_s=round(age, 1), last=None if last is None else last[1],
                   hTarget_m=h, due_s=due_s,
                   note=f"{age:.0f} s since the last height check at {h:.2f} m commanded (the runbook: every "
                        f"{due_s:g} s here): the estimate drifts 0.01-0.03 m/s either way (up to "
                        f"{0.03 * age:.2f} m by now); take a photo and judge the height and the ceiling clearance "
                        "(docs/autonomy-a2b.md 23.5)")

    def brief(self, now):
        a = self.auto
        if a is None:
            return {"state": self.state_name()}
        b = {"state": a["stateName"], "reason": a["reasonName"], "vzLandReason": a["vzLandReasonName"],
             "mode": a["modeName"], "waiting": a["waiting"], "waitLeftMs": self.wait_left(now),
             "waitLeftSaturated": self.wait_saturated(), "airMs": a["airMs"], "h_m": a["h_m"], "vbat": a["vbat_v"], "photoId": a["photoId"],
             "capture": a["captureName"], "status_age_ms": int((now - self.auto_rx) * 1000)}
        if a["interactive"]:
            b.update(self.motion_view(a))
        if a["hr"]:
            b["hold"] = a["hold"]   # what the drone did on its own and holds (events, bits, the SINK countdown, contact)
        # information only (SET_HEIGHT, docs/autonomy-a2b.md 23): this sortie's latest accepted correction and photo
        sh = self.last_sh
        lp = self.photo_log[-1] if self.photo_log and self.photo_log[-1]["sortie_key"] == self.sortie_key else None
        b["since_set_height_s"] = None if sh is None else round(now - sh["mono"], 1)
        b["since_photo_s"] = None if lp is None else round(now - lp["saved_mono"], 1)
        if a["interactive"] and a["state"] == S_HOVER:
            last = self.last_height_check()
            due_s = HEIGHT_CHECK_HIGH_S if a["hTarget_m"] > HEIGHT_CHECK_HIGH_M + 1e-9 else HEIGHT_CHECK_S
            b["height_check_due"] = last is None or now - last[0] >= due_s
        return b

    @staticmethod
    def motion_view(a):
        """The interactive hover's fields of an AUTO_STATUS, for the replies. Above mvspeed 100 also the drone's
        mvacc / mvsettle (2026-10-17 'real 300 %': not in AUTO_STATUS; as the last 'auto get' read them, fast_caps)."""
        pct = a.get("mvSpeedPct", MV_SPEED_DEFAULT)
        fast = {}
        if pct > MV_SPEED_DEFAULT:
            caps = fast_caps()
            fast = {"mvacc_cm_s2": caps["mvacc"], "mvsettle_ms": caps["mvsettle"], "fast_caps_src": caps["src"]}
        return {"air_s": a["air_s"], "airLeft_s": a["airLeft_s"], "battLow": a["battLow"],
                "battLowNow": a["battLowNow"],
                "hTarget_m": a["hTarget_m"], "yawOff_deg": a["yawOff_deg"], "D_m": a["sceneD_m"],
                "mvspeed_pct": pct, **fast, "guards": guards_view(),
                "mv": {"state": a["mvStateName"], "kind": "SET_HEIGHT" if a.get("mvCorrection") else a["mvKindName"],
                       "seq": a["mvSeq"], "why": a["mvWhyName"],
                       "end": a["mvEndName"], "flags": a["mvFlagNames"], "prog_m": a["prog_m"], "tgt_m": a["tgt_m"],
                       "yawTgt_deg": a["yawTgtDeg"], "phaseMs": a["mvPhaseMs"]}}

    def daemon_view(self, now):
        return {"pid": os.getpid(), "session": hex32(self.link.session), "bootNonce": hex32(self.boot or 0),
                "rebooted": self.rebooted, "session_lost": self.session_lost, "link_lost": self.link_lost,
                "rx_age_ms": int((now - self.last_rx) * 1000), "allow_flight": self.allow_flight,
                "hb_sent": self.hb_sent, "hb_late": self.hb_late, "send_errors": self.send_errors,
                "flight": None if self.flight is None else TYPE_NAMES[self.flight.ptype],
                "stop": TYPE_NAMES[self.stop.ptype] if self.stopping() else None,
                "queued": [TYPE_NAMES[c.ptype] for c in self.queue], "sets_held": sorted(self.sets),
                "sorties": len(self.sorties), "armed_token": self.arm_token is not None,
                "clients_dropped": self.clients_dropped,
                "out_dropped": self.out.dropped if self.out is not None else 0,
                "events_next": self.events_base + len(self.events)}

    def op_ping(self, c, req, now):
        c.reply({"ok": True, "pid": os.getpid(), "session": hex32(self.link.session)})

    def op_hello(self, c, req, now):
        c.reply({"ok": True, "via": "daemon", **hello_info(self.link), "rebooted": self.rebooted,
                 "session_lost": self.session_lost})

    def op_state(self, c, req, now):
        c.reply({"ok": True, **self.brief(now), "status": status_view(self.status), "auto": auto_view(self.auto),
                 "daemon": self.daemon_view(now)})

    def op_status(self, c, req, now):
        """The one-shot 'status' through the daemon: a fresh STATUS (and AUTO_STATUS), no HELLO."""
        n0 = self.status_count
        self.send_status_req(now)

        def reply(stale):
            if self.status is None:
                return {"ok": False, "error": "no STATUS from the drone", "via": "daemon"}
            return {"ok": True, "via": "daemon", **hello_info(self.link), "status": status_view(self.status),
                    "auto": auto_view(self.auto), "stale": stale, "daemon": self.daemon_view(time.monotonic())}
        self.defer(c, lambda now: reply(False) if self.status_count > n0 else None, now + 1.5, lambda: reply(True))

    def arm_result(self, r, mode):
        """The arm reply, and whether a DISARM must follow (the ARM acted but its token was lost)."""
        token = r["detail"] if r["ok"] and not r.get("reconciled") else None
        a = self.auto
        out = {"ok": r["ok"] and token is not None, "result": r.get("resultName"), "token": token,
               "mode": MODE_NAMES[mode], "state": r.get("stateName"),
               "vbat": a["vbat_v"] if a else (self.status or {}).get("vbat_v"), "vmin": None}
        v = cached_vmin()
        if v:
            out["vmin"], out["vmin_src"] = v["v"], f"auto get {v['read_utc']} ({v['src']})"
        if r.get("cam_wake_retries"):
            out["cam_wake_retries"] = r["cam_wake_retries"]
        if r.get("result") == R_GUARDS:
            out["guards"], out["failedGuards"] = guard_names(r["detail"]), r["detail"]
            hint = soak_hint(self.soak_air_s, None if self.soak_from is None else time.monotonic() - self.soak_from)
            if "STILL" in out["guards"] and mode == MODE_FLIGHT and hint:
                out["hint"] = hint
            elif r["detail"] == G_CAMERA and r.get("cam_wake_retries"):
                out["hint"] = (f"the camera did not come up within {ARM_CAM_WAKE_RESENDS * ARM_CAM_WAKE_S:g} s of "
                               "the ARM (ground standby, or a thermal lead 'tkoff camlead'): see 'status' (cam, fps)")
        disarm = False
        if r.get("error"):
            out["error"] = r["error"]
        elif not r["ok"]:
            out["error"] = str(ack_error("ARM", r))
        elif token is None:
            out["error"] = "ARM took effect but its token was lost (BUSY taken-unknown); DISARM sent: arm again"
            disarm = True
        if token is not None:
            self.arm_token = token
        return out, disarm

    def arm_done(self, c, r, mode):
        """The plain 'arm': as a sortie's ARM, one that may have acted without a usable answer is stopped on the
        ground (DISARM unless the drone flies: arm_unknown_stops; never a LAND) and the reply says so."""
        out, disarm = self.arm_result(r, mode)
        stops = [T_DISARM] if disarm else [] if out["ok"] else self.arm_unknown_stops(r)
        if stops:
            out["stops"] = [TYPE_NAMES[t] for t in stops]
        if not disarm and not out["ok"] and r.get("sent") and r.get("result") in (None, R_BUSY):
            out["error"] += self.arm_unknown_note(stops)
        c.reply(out)
        now = time.monotonic()
        for t in stops:
            self.stop_request(t, "ARM token lost" if disarm else "the ARM may have acted without an answer", now)

    def op_arm(self, c, req, now):
        mode = MODES[req["mode"]]
        air = check_air_ms(int(req.get("air_ms") or 0))
        why = self.refusal(mode, bool(req.get("confirm_flight")))
        if why:
            return c.reply({"ok": False, "result": "REFUSED", "error": why})
        self.queue_flight(Cmd(T_ARM, arm_payload(mode, air), lambda r: self.arm_done(c, r, mode), owner=c), now)

    def takeoff_cmd(self, token, req, done, owner=None):
        """The TAKEOFF, range-checked. The long-wait rules on the wait the drone will fly (--wait-s, or with 0 the
        drone's NVS autowait as the last 'auto get' read it) are advice since 2026-10-15 ("Claude decides": they
        refused the take-off until then): cmd.note carries them, and the reply's 'note'."""
        height, n, gap = int(req.get("height", 40)), int(req.get("photos", 1)), int(req.get("gap", 300))
        wait_ms = int(round(float(req.get("wait_s") or 0) * 1000))
        if not HEIGHT_CM[0] <= height <= HEIGHT_CM[1] or not 0 <= n <= MAX_N \
                or not PHOTO_GAP_AIR_MS[0] <= gap <= PHOTO_GAP_AIR_MS[1] \
                or not (wait_ms == 0 or ACT_WAIT_MS[0] <= wait_ms <= ACT_WAIT_MS[1]):
            raise ValueError(f"height {HEIGHT_CM[0]}..{HEIGHT_CM[1]} cm, photos 0..{MAX_N}, gap "
                             f"{PHOTO_GAP_AIR_MS[0]}..{PHOTO_GAP_AIR_MS[1]} ms, wait_s 0 (the drone's autowait) or "
                             f"{ACT_WAIT_MS[0] // 1000}..{ACT_WAIT_MS[1] // 1000}")
        note = None
        wait_eff, src = wait_ms, f"a {wait_ms / 1000.0:g} s wait"
        if not wait_ms:
            v = cached_setting("autowait")
            if v is not None and AUTOWAIT_MS[0] <= v["value"] <= AUTOWAIT_MS[1]:
                wait_eff = v["value"]
                src = f"the drone's autowait ({wait_eff} ms as 'auto get' read it at {v['read_utc']}; --wait-s 0)"
            else:
                wait_eff = None
                note = ("--wait-s 0: the drone's NVS autowait is unknown here (no 'auto get' read: run "
                        "'python3 tools/espfly_diag.py auto get' over USB), so its long-wait height advice was not "
                        "checked")
        notes = [note] if note else []
        if wait_eff is not None and wait_eff > WAIT_LONG_S * 1000 and \
                height / 100.0 < wait_height_floor(wait_eff / 1000.0) - 1e-9:
            notes.append(f"{src} at {height} cm: the height estimate drifts up to {H_DRIFT_MPS:g} m/s (reading high), "
                         f"so a long low hover can reach the floor; "
                         f"{math.ceil(wait_height_floor(wait_eff / 1000.0) * 100 - 1e-6):.0f} cm or more is safer "
                         f"(advice: watch the photos for the floor)")
        note = "; ".join(notes) if notes else None

        pr, pf = plan_cm(req)   # (a planned move flies with any wait: the wait always starts, 2026-10-15)
        via_drone = (pr or pf) and req.get("plan_via", PLAN_VIA_DEFAULT) == "drone"

        def acked(r):
            if r["ok"]:
                self.takeoff_gap = gap
            done(r)
        cmd = Cmd(T_TAKEOFF, takeoff_payload(token, height, n, gap, wait_ms, pr if via_drone else 0,
                                             pf if via_drone else 0), acked, owner=owner)
        cmd.note = note
        return cmd

    def op_takeoff(self, c, req, now):
        why = self.refusal()
        if why is None and self.arm_token is None:
            why = "not armed through this daemon (no ARM token): arm first"
        if why:
            return c.reply({"ok": False, "result": "REFUSED", "error": why})

        cmd = None

        def done(r):
            out = {"ok": r["ok"], "result": r.get("resultName"), "state": r.get("stateName")}
            for k in ("reconciled", "resent", "auto"):
                if k in r:
                    out[k] = r[k]
            if cmd.note:
                out["note"] = cmd.note
            if not r["ok"]:
                out["error"] = r.get("error") or str(ack_error("TAKEOFF", r))
            c.reply(out)
        cmd = self.takeoff_cmd(self.arm_token, req, done, owner=c)
        self.queue_flight(cmd, now)

    def stop_op(self, c, ptype, now):
        """land / disarm: the reply comes when the drone confirms it, or after STOP_REPLY_S (the daemon keeps
        sending it until STOP_GIVE_UP_S)."""
        s = self.stop_request(ptype, "client", now)
        what = TYPE_NAMES[ptype]

        def check(now):
            if not s.finished:
                return None
            r = s.result
            out = {"ok": r["ok"], "type": r["type"], "result": r.get("resultName"), "state": r.get("stateName"),
                   "confirmed": r.get("confirmed", "ACK" if r["ok"] else None)}
            if not r["ok"]:
                out["error"] = r.get("error") or str(ack_error(what, r))
            return out
        self.defer(c, check, now + STOP_REPLY_S,
                   lambda: {"ok": False, "type": TYPE_NAMES[s.ptype], "result": None, "state": self.state_name(),
                            "error": f"{TYPE_NAMES[s.ptype]} not confirmed within {STOP_REPLY_S:g} s; the daemon "
                                     f"keeps sending it (up to {STOP_GIVE_UP_S:g} s): check 'state'"})

    def op_land(self, c, req, now):
        self.stop_op(c, T_LAND, now)

    def op_disarm(self, c, req, now):
        self.stop_op(c, T_DISARM, now)

    def op_kill(self, c, req, now):
        """The reply's state: from an AUTO_STATUS that counts this KILL (killCount above the count before it,
        so taken after the step that acted), else the burst's ACKs (kill_burst_state), else the last known."""
        kills0 = self.auto["killCount"] if self.auto is not None else None
        n0 = self.auto_count
        seqs = self.kill("client")

        def reply():
            acked = [s for s in seqs if s in self.kill_acks]
            out = {"ok": True, "sent": len(seqs), "acked": len(acked),
                   "state": kill_burst_state([self.kill_acks[s] for s in acked]) or self.state_name()}
            a = self.auto
            if self.auto_count > n0 and a is not None and kills0 is not None and a["killCount"] > kills0:
                out["state"], out["killCount"] = a["stateName"], a["killCount"]
            return out
        self.defer(c, lambda now: None, now + KILL_ACK_WAIT_S, reply)

    def op_photo(self, c, req, now):
        """In the hover wait the state machine's PHOTO, in IDLE the A1 photo; replies with the frames."""
        n, gap = int(req.get("n", 1)), int(req.get("gap", 250))
        if not 1 <= n <= MAX_N or not 0 <= gap <= 1000:
            raise ValueError(f"n 1..{MAX_N}, gap 0..1000 ms")
        timeout = float(req["timeout"]) if req.get("timeout") is not None else photo_set_timeout_s(n)
        out_dir = os.path.abspath(os.path.expanduser(req["out"])) if req.get("out") else None
        self.photo_request(c, n, gap, timeout, out_dir, now, c.reply)

    def photo_request(self, c, n, gap, timeout, out_dir, now, done):
        """The PHOTO of op_photo for client c; done(reply) with the frames (or the error)."""
        held = {}

        def reply(s, error=None):
            r = {"ok": error is None and not s.failed, "id": s.id, "n": s.n, "photos": s.photos(),
                 "failed": s.failures(), "dir": s.out_dir, **self.brief(time.monotonic())}
            if error or s.failed:
                r["error"] = error or f"{len(s.failed)} of {s.n} frame(s) not received"
            return r

        def acked(r):
            if not r["ok"]:
                return done({"ok": False, "result": r.get("resultName"), "detail": r.get("detail"),
                             "error": r.get("error") or str(ack_error("PHOTO", r)), **self.brief(time.monotonic())})
            pid = (r["detail"] or 0) & 0xFFFF
            if not pid:
                return done({"ok": False, "result": r.get("resultName"), "id": None, "photos": [], "failed": [],
                             "error": "PHOTO accepted, but no photo set was started (the camera is not running "
                                      "or has no photo slots); the drone keeps hovering (a failed set is a "
                                      "CAPTURE_FAIL event since 2026-10-15): try 'photo' again",
                             **self.brief(time.monotonic())})
            s = self.sets.get(pid) or self.find_set(pid)
            if s is None:
                s = self.new_set(pid, n, gap, "mac", time.monotonic(), out_dir)
            elif s.free is None:
                s.n, s.gap_ms, s.source = s.n or n, gap, "mac"
                if out_dir and not s.saved:
                    s.out_dir = out_dir
            held["set"] = s
            if s.resolved():
                done(reply(s))
            else:
                s.waiters.append(lambda s: done(reply(s)))

        self.queue_flight(Cmd(T_PHOTO, photo_payload(n, gap), acked, owner=c), now)
        self.defer_fn(c, lambda now: None, now + timeout,
                      lambda: reply(held["set"], "timed out") if "set" in held else
                      {"ok": False, "error": f"PHOTO not answered within {timeout:g} s",
                       **self.brief(time.monotonic())}, done)

    def op_wait(self, c, req, now):
        until = req["until"]
        ev0 = None if self.auto is None else (self.auto["evCount"], self.auto["state"])
        conds = {"waiting": lambda a: a["state"] == S_HOVER and a["waiting"] and self.fsm_set_ready(a),
                 "idle": lambda a: a["state"] == S_IDLE,
                 # (a LAND SINK may resume: not a landing until the drone's reason says it is taken over)
                 "land": lambda a: a["state"] in (S_IDLE, S_KILLED) or (a["state"] == S_LAND and
                                                                        a["reason"] != RS_SINK),
                 "photos": lambda a: not self.sets and self.fsm_set_ready(a),
                 # hold and report (2026-10-15): the drone reported something new, or changed state (a 136-byte
                 # drone; an older one: a state change only)
                 "event": lambda a: ev0 is None or (a["evCount"], a["state"]) != ev0}
        cond = conds[until]
        n0 = self.auto_count
        self.send_status_req(now)       # a fresh AUTO_STATUS within a round trip, even in IDLE

        def check(now):
            a = self.auto
            if self.auto_count > n0 and cond(a):
                return {"ok": True, "until": until, **self.brief(now)}
            return None
        self.defer(c, check, now + float(req.get("timeout", 30.0)),
                   lambda: {"ok": False, "until": until, "error": f"timeout waiting for {until}",
                            **self.brief(time.monotonic())})

    def op_events(self, c, req, now):
        since = int(req.get("since", 0))
        c.reply({"ok": True, "events": [e for e in self.events if e["n"] >= since],
                 "next": self.events_base + len(self.events)})

    def op_sortie(self, c, req, now):
        mode = MODES[req["mode"]]
        why = self.refusal(mode, bool(req.get("confirm_flight")))
        if why:
            return c.reply({"ok": False, "result": "REFUSED", "error": why})
        Sortie(self, c, req, mode, now)


    # -------- the interactive hover: MOVE, YAW, STOP (docs/autonomy-a2b.md section 14)

    def motion_refusal(self, now):
        """Why a MOVE or YAW is not sent now, or None. The drone checks it all again (and answers with
        its own reason); the Mac refuses first so that nothing reaches a drone that is not waiting."""
        why = self.refusal()
        if why:
            return why
        a = self.auto
        if a is None or self.auto_rx is None or now - self.auto_rx > MOTION_STATUS_MAX_AGE_S:
            return "no fresh AUTO_STATUS from the drone"
        if not a["interactive"]:
            return ("the drone's firmware has no interactive hover (its AUTO_STATUS is the A2b 64-byte one): "
                    "flash the interactive-hover build")
        if a["state"] != S_HOVER or not a["waiting"]:
            return (f"not in the hover wait (state {a['stateName']}, waiting {a['waiting']}): MOVE and YAW only "
                    "after the take-off photo, while the drone waits for a command")
        if self.air_clock_mismatch is not None:
            return (f"the drone's air limit is {self.air_clock_mismatch / 1000.0:g} s (AUTO_STATUS airMs + airLeftDs), "
                    f"not this Mac tool's {AIR_HARD_MS / 1000.0:g} s: the two are different versions, so the Mac's air "
                    "time (its height-floor rule) would be wrong past 65.5 s. Update the firmware or the "
                    "Mac tool; hover, photos, STOP and LAND still work")
        if a["mvActive"]:
            what = "SET_HEIGHT correction" if a.get("mvCorrection") else a["mvKindName"]
            return f"a {what} is running ({a['mvStateName']}): STOP it, or wait for it to end"
        busy = [c for c in self.queue + ([self.flight] if self.flight is not None else [])
                if c.ptype in MOTION_TYPES and not c.finished]
        if busy:
            return f"a {TYPE_NAMES[busy[0].ptype]} is being sent"
        if a["battLow"]:
            return ("the drone's BATT_LOW condition has held 0.25 s (the battery below 3.00 V, or the hover needs over "
                    "57,000 of the 58,000 ceiling): the drone refuses MOVE and YAW and lands BATT_LOW by itself after "
                    "0.5 s; hover, photos, STOP and LAND still work")
        return None

    @staticmethod
    def low_refusal(a, h_after):
        """Why a motion that lowers the drone to the commanded height h_after is refused for the height
        estimate's drift, or None. One-sided (2026-10-15, as the drone's AUTO_MV_H_*): a motion that does not go
        down (a YAW, a level or upward move) is never refused for it, wherever the drone is."""
        lo = height_floor(a["air_s"])
        if h_after < lo - 1e-9 and h_after < a["hTarget_m"] - 1e-9:
            return (f"too low for {a['air_s']:.0f} s of air: the commanded height would be {h_after:.2f} m and the "
                    f"height estimate may have drifted {lo - H_FLOOR_M:.2f} m (reading high), so the drone may be "
                    f"near the floor; at least {lo:.2f} m: move up first, or land")
        return None

    def behind_needed(self):
        """A new MOVE / YAW / SET_HEIGHT must wait (fast moves, 2026-10-16): the motion this daemon returned early from
        (its profile done, its settle or re-track bob still running) is still active on the drone."""
        a = self.auto
        return (self.early_seq is not None and a is not None and a["interactive"] and a["state"] == S_HOVER and
                a["mvActive"] and a["mvSeq"] == self.early_seq and self.refusal() is None)

    def wait_behind(self, owner, now, done, retry, what):
        """Holds a MOVE / YAW / SET_HEIGHT behind the early-returned motion until its AUTO_STATUS shows it ended, then
        runs every check again on that status (retry(now)): it goes out only after a normal end (DONE, SHORT, a reached
        PROGRESS); any other end refuses it (the photo Claude judged is older than that end). At most the motion's
        settle + 4 bob cycles + BEHIND_EXTRA_S; a STOP withdraws it; leaving HOVER refuses it."""
        seq = self.early_seq
        pct = self.auto.get("mvSpeedPct", MV_SPEED_DEFAULT)
        limit_s = settle_s(pct, fast_caps()) + BOB_CYCLES * bob_cycle_s(pct) + BEHIND_EXTRA_S
        entry = {"withdrawn": None, "what": what}
        self.behind.append(entry)
        self.event("motion_waits", type=what, behind_mvSeq=seq, limit_s=round(limit_s, 2))

        def drop():
            if entry in self.behind:
                self.behind.remove(entry)

        def check(now):
            if entry["withdrawn"]:
                drop()
                return {"ok": False, "result": "REFUSED", "type": what, "error": entry["withdrawn"], **self.brief(now)}
            a = self.auto
            if a is None:
                return None
            if a["state"] != S_HOVER:
                drop()
                return {"ok": False, "result": "REFUSED", "type": what,
                        "error": f"the drone left HOVER while the {what} waited for the previous motion: "
                                 f"{a['stateName']} ({a['reasonName']})", **self.brief(now)}
            if a["mvSeq"] == seq and a["mvActive"]:
                return None
            drop()
            if self.early_seq == seq:
                self.early_seq = None
            if a["mvSeq"] == seq:
                rch = move_reached(a["mvEnd"], dict(a["prog_m"]), dict(a["tgt_m"])) \
                    if a["mvKindName"] == "MOVE" and not a.get("mvCorrection") else {}
                ok = a["mvEnd"] in MVE_OK or bool(rch.get("reached"))
                # the early-returned motion's final end (2026-10-17): in the next reply as 'previous'
                self.prev_final = {"mvSeq": seq, "kind": a["mvKindName"], "end": a["mvEndName"],
                                   "travelled_m": dict(a["prog_m"]),
                                   **{k: rch[k] for k in ("along", "off_m", "warning") if k in rch}}
                if not ok:
                    prev = "SET_HEIGHT correction" if a.get("mvCorrection") else a["mvKindName"]
                    final, self.prev_final = self.prev_final, None
                    return {"ok": False, "result": "REFUSED", "type": what, "previous_end": a["mvEndName"],
                            "previous": final,
                            "error": f"the previous {prev} ended {a['mvEndName']} after your photo"
                                     f"{MV_END_HINTS.get(a['mvEndName'], '')}: take a new photo", **self.brief(now)}
            self.event("motion_released", type=what, behind_mvSeq=seq, end=a["mvEndName"])
            try:
                retry(now)
            except (ValueError, TypeError, KeyError) as e:
                return {"ok": False, "result": "REFUSED", "type": what, "error": f"bad request: {type(e).__name__}: {e}"}
            return False

        def timed_out():
            drop()
            return {"ok": False, "result": "REFUSED", "type": what,
                    "error": f"the previous motion (mvSeq {seq}) has not ended within {limit_s:.1f} s of its settle "
                             "or bob: take a photo, then decide", **self.brief(time.monotonic())}
        self.defer_fn(owner, check, now + limit_s, timed_out, done, on_gone=drop)

    def refuse_motion(self, done, why, now, **extra):
        out = {"ok": False, "result": "REFUSED", "error": why, **extra, **self.brief(now)}
        if self.prev_final is not None:
            out["previous"], self.prev_final = self.prev_final, None   # (the motion it waited behind: its end)
        done(out)

    def motion_cmd(self, c, ptype, payload, req, now, timeout_s, info, done):
        """Sends a MOVE or YAW; done(reply) once the drone refused it, or (wait, the default) once the
        motion it started has ended (DONE or STOPPED: its end, progress and the latest status), the
        drone left HOVER, or timeout_s passed (the motion keeps running on the drone).
        Fast moves (2026-10-16, docs/autonomy-a2b.md 31): with the drone's mvspeed above 100 (or early) the reply
        comes as soon as the motion's profile has ended (a MOVE in SETTLE holding its target, a YAW in its re-track
        bob; settling: true) unless full_wait; the next MOVE / YAW / SET_HEIGHT then waits for its end (wait_behind).
        photo: a photo is taken when it replies (the drone in HOVER) and goes in the same reply."""
        what = TYPE_NAMES[ptype]
        wait = req.get("wait", True)
        pct = self.auto.get("mvSpeedPct", MV_SPEED_DEFAULT)
        early = bool(wait and not req.get("full_wait") and (req.get("early") or pct > MV_SPEED_DEFAULT))
        photo = bool(wait and req.get("photo"))
        info = {**info, "mvspeed_pct": pct}
        caps = fast_caps()   # the drone's mvacc / mvsettle (the early reply's settle; the status shows them)
        cmd = Cmd(ptype, payload, None, owner=c)
        cmd.mv_seq0 = self.auto["mvSeq"]
        a0 = self.auto
        # the D the last forward / back move measured (its end), for the drift since then (drift_since_last_end_m):
        # as first seen at that end (a refused command since moved the status D by the drift so far), else the status D
        if self.d_end is not None and self.d_end["mvSeq"] == a0["mvSeq"] and not a0["mvActive"]:
            d_last = self.d_end["D"]
        else:
            d_last = a0["sceneD_m"] if (a0["mvFlags"] & MVF_D_TRACK and not a0["mvActive"] and
                                        a0["mvKindName"] == "MOVE") else None
        if not drift_measured():
            d_last = None   # (mvfix without DRIFT: the drone measures no drift; no field rather than a 0.000)

        def acked(r):
            now = time.monotonic()
            out = {"ok": r["ok"], "type": what, "result": r.get("resultName"), **info}
            for k in ("reconciled", "resent"):
                if k in r:
                    out[k] = r[k]
            seq = None
            if r.get("detail") is not None and r.get("result") is not None:
                _, code, seq = mv_detail(r["detail"])
                if not r["ok"]:
                    out["why"] = name_of(MV_WHY_NAMES, code)
            if self.prev_final is not None:
                out["previous"] = self.prev_final   # the early-returned motion's final end (it waited behind it)
                self.prev_final = None
            if not r["ok"]:
                hint = MV_WHY_HINTS.get(out.get("why"))
                out["error"] = r.get("error") or f"{what} refused {r.get('resultName')}" + \
                    (f" ({out['why']}: {hint})" if hint else "")
                return done({**out, **self.brief(now)})
            out["mvSeq"] = seq
            if self.early_seq is not None:
                self.early_seq = None   # a new motion: the early-returned one has ended
            self.event("motion", type=what, mvSeq=seq, **info)
            if not wait:
                return done({**out, "running": True, **self.brief(now)})
            n0 = self.auto_count
            # What it travelled: the drone keeps a MOVE's measured progress in its status once the move has
            # ended (since the user decision of 2026-10-13, evening). An older drone restarted it with the
            # hold's new epoch at a STOP (the tiger flight of 2026-10-13 read 'fwd -0.01 m' for ~0.8 m flown):
            # then, and when the drone left HOVER, the last progress seen while it ran.
            ran = {"prog_m": None, "d_acc": None}

            def travelled(a):
                if ran["prog_m"] is not None and not any(a["prog_m"].values()):
                    return ran["prog_m"]
                return dict(a["prog_m"])

            def extras(t):
                """The drift since the last move's end (2026-10-17, mvfix DRIFT: the drone's D at the acceptance
                follows the hold's measured drift; + toward the scene) and, for a move with a forward part, its sideways
                travel at the mean of its start and end distances (the drone converts at the start D)."""
                x = {}
                if d_last is not None and ran["d_acc"] is not None and ptype == T_MOVE:
                    x["drift_since_last_end_m"] = {"fwd": round(d_last - ran["d_acc"], 3), "D_last_end_m": d_last,
                                                   "D_at_accept_m": ran["d_acc"]}
                d0 = ran["d_acc"] or info.get("D_m")
                if ptype == T_MOVE and t and d0 and t.get("right") and t.get("fwd"):
                    r = max(0.05, 1.0 - t["fwd"] / d0)
                    x["sideways_m_at_mean_D"] = round(t["right"] * 0.5 * (1.0 + r), 3)
                return x

            def with_photo(res):
                """--photo: the photo now, in the same reply (the drone takes it during a settle or bob too)."""
                if not photo:
                    return res
                self.photo_request(c, 1, int(req.get("gap", 150)), float(req.get("photo_timeout", 10.0)), None,
                                   time.monotonic(),
                                   lambda p: done({**res, "photo": {k: p.get(k) for k in ("ok", "id", "photos",
                                                                                        "failed", "dir", "error")
                                                                    if k in p}}))
                return False

            def check(now):
                a = self.auto
                if self.auto_count <= n0 or a is None:
                    return None
                if seq is not None and a["mvSeq"] == seq and a["mvState"] not in (MV_DONE, MV_STOPPED):
                    ran["prog_m"] = dict(a["prog_m"])
                if seq is not None and a["mvSeq"] == seq and ran["d_acc"] is None:
                    ran["d_acc"] = a["sceneD_m"] if a["mvFlags"] & MVF_D_TRACK else 0.0
                    ran["d_acc"] = ran["d_acc"] or None
                if a["state"] != S_HOVER:
                    return {**out, "ok": False, "end": a["mvEndName"], "travelled_m": ran["prog_m"],
                            "error": f"the drone left HOVER during the {what}: {a['stateName']} ({a['reasonName']})",
                            **self.brief(now)}
                if early and seq is not None and a["mvSeq"] == seq and a["mvState"] == MV_SETTLE and \
                        (ptype == T_MOVE or a["mvFlags"] & MVF_BOB):
                    # the profile has ended: the drone holds the target (a MOVE's settle, a YAW's re-track bob)
                    self.early_seq = seq
                    rest = (f"settles at most {settle_s(pct, caps):g} s" if ptype == T_MOVE else
                            f"re-tracks with its bob (cycles of {bob_cycle_s(pct):.2f} s, at most {BOB_CYCLES})")
                    self.event("motion_early", type=what, mvSeq=seq, mvspeed_pct=pct)
                    pre = dict(a["prog_m"])
                    res = {**out, "ok": True, "settling": True, "end": None, "mvState": a["mvStateName"],
                           "travelled_pre_settle_m": pre, **extras(pre),
                           "note": f"the {what}'s profile has ended (mvspeed {pct} %): the drone holds there and "
                                   f"{rest}; travelled_pre_settle_m is before its settle (not its end: the drone "
                                   "can still slide 0.1-0.4 m); wait-move gives the final end, and the next move / "
                                   "yaw / set-height waits for it and reports it under 'previous'", **self.brief(now)}
                    if ptype == T_MOVE:
                        rch = move_reached(None, pre, out.get("target_m"))
                        res.update({k: rch[k] for k in ("along", "off_m", "warning") if k in rch})
                    return with_photo(res)
                if seq is None or a["mvSeq"] != seq or a["mvState"] not in (MV_DONE, MV_STOPPED):
                    return None
                res = {**out, "ok": a["mvEnd"] in MVE_OK, "end": a["mvEndName"], "travelled_m": travelled(a),
                       **self.brief(now)}
                res.update(extras(res["travelled_m"]))
                rch = move_reached(a["mvEnd"], res["travelled_m"], out.get("target_m"))
                res.update({k: rch[k] for k in ("along", "off_m", "warning") if k in rch})
                if rch.get("reached"):
                    res.update(ok=True, reached=True, note=rch["note"])
                elif "error" in rch:
                    res["error"] = rch["error"]
                elif a["mvEnd"] in (MVE_SHORT, MVE_LONG):
                    t = res["travelled_m"]
                    how = "short of the target or off its line" if a["mvEnd"] == MVE_SHORT else "past the target"
                    # a LONG move toward the scene keeps its target (the review of 2026-10-17): the hold walks it back
                    back = a["mvEnd"] == MVE_LONG and (out.get("target_m") or {}).get("fwd", 0.0) > 0.0
                    hold = ("the drone's hold walks it back to the target (a move toward the scene keeps its target)"
                            if back else "the drone holds where it ended")
                    res["note"] = (f"the {what} ended {a['mvEndName']}: its settle timed out {how} (travelled right "
                                   f"{t['right']:+.2f} m, forward {t['fwd']:+.2f} m); {hold}. "
                                   "Take a photo before planning the next step")
                elif a["mvEnd"] != MVE_DONE:
                    res["error"] = f"the {what} ended {a['mvEndName']}" + MV_END_HINTS.get(a["mvEndName"], "")
                return with_photo(res)
            self.defer_fn(c, check, now + timeout_s,
                          lambda: {**out, "ok": False, "error": f"the {what} has not ended within {timeout_s:.0f} s; "
                                                                "the drone keeps flying it (STOP ends it)",
                                   **self.brief(time.monotonic())}, done)
        cmd.done = acked
        self.queue_flight(cmd, now)

    @staticmethod
    def move_timeout(right, fwd, up, d, pct=MV_SPEED_DEFAULT, caps=None):
        """The drone's profile at its speed step (move_profile_s) + MOVE_TIMEOUT_MARGIN_S: 37 s for 4 m forward at
        D 8 m at 100; a move of up to MOVE_HORIZ_MAX_M (the old limit) never waits less than the generous 0.20 m/s
        basis of before 2026-10-14 (that basis would give a 4 m move 50 s: 28 s of nothing after its 22 s profile)."""
        horiz = math.hypot(right, fwd)
        new = move_profile_s(right, fwd, up, d, pct, caps) + MOVE_TIMEOUT_MARGIN_S
        if horiz > MOVE_HORIZ_MAX_M + 1e-9:
            return new
        v = max(0.03, min(0.20, MOVE_W_LAT_MAX * (d or 1.0)))
        return max(new, min(75.0, 10.0 + 2.0 * horiz / v + 2.0 * abs(up) / 0.15))

    def op_move(self, c, req, now):
        """MOVE right / forward / up (metres; the drone's own frame when it accepts it)."""
        self.move_request(c, req, now, c.reply)

    def move_request(self, owner, req, now, done):
        """The checks of a MOVE, then the MOVE for owner (a client or a sortie); done(reply). person_gap: Claude's
        estimate of the end distance to the nearest person (>= PERSON_MIN_M), with est (its estimate of the distance
        to that person now, required with it): the drone's own travel error at the image expansion scale k down to
        K_LOW may not take it under PERSON_MIN_M either (person_gap_low; 2026-10-15 review), both logged for
        calibration; batt_ok: no battery-duration refusal. Behind an early-returned motion (fast moves) it waits for that
        motion's end, then every check runs on the fresh status (wait_behind)."""
        vals = [float(req.get(k) or 0.0) for k in ("right", "fwd", "up")]
        if not all(math.isfinite(v) for v in vals):
            raise ValueError("right, fwd, up must be finite")
        if self.behind_needed():
            return self.wait_behind(owner, now, done, lambda n: self.move_request(owner, req, n, done), "MOVE")
        rc, fc, uc = (int(round(v * 100)) for v in vals)
        info = {"target_m": {"right": rc / 100.0, "fwd": fc / 100.0, "up": uc / 100.0}}
        gap = req.get("person_gap")
        est = req.get("est")
        if gap is not None:
            gap = float(gap)
            info["person_gap_m"] = gap
        if est is not None:
            est = float(est)
            info["est_m"] = est
        why = move_range_refusal(rc, fc, uc)
        if why:
            return self.refuse_motion(done, why + " (split a longer move)", now, **info)
        if rc == fc == uc == 0:
            return self.refuse_motion(done, "nothing to move (every component rounds to 0 cm)", now, **info)
        if gap is not None and not (math.isfinite(gap) and gap >= PERSON_MIN_M - 1e-9):
            return self.refuse_motion(done, f"--person-gap {gap:.2f} m: a move never ends closer than "
                                            f"{PERSON_MIN_M:.1f} m to a person (user rule 2026-10-13): plan it to "
                                            "end farther", now, **info)
        if gap is not None and not (est is not None and math.isfinite(est) and est > 0.0):
            return self.refuse_motion(done, "--person-gap needs --est (your distance to that person now): the Mac "
                                            "checks the gap against the drone's own travel error with the live D",
                                      now, **info)
        why = self.motion_refusal(now)
        if why:
            return self.refuse_motion(done, why, now, **info)
        a = self.auto
        d = a["sceneD_m"]
        pct = a.get("mvSpeedPct", MV_SPEED_DEFAULT)
        caps = fast_caps()   # the drone's mvacc / mvsettle (above 100 %): its profile, the battery fit, the person rule
        if (rc or fc) and d is None:
            return self.refuse_motion(done, "no scene distance D yet (mvFlags " + str(a["mvFlagNames"]) + "): "
                                      "the camera's tracker needs some vertical motion; after a YAW the drone bobs "
                                      "to re-track. Ask 'state' again", now, **info)
        # the drone's sideways cap and floor ahead as the last 'auto get' cached them (2026-10-18 "guards off": mvlat 0
        # and mvfloor 0 lift the two refusals; mvklow the k of the floor check)
        lat_rel = cached_lat_rel()
        if rc and lat_rel is not None and abs(rc) / 100.0 > lat_rel * d + 1e-4:
            return self.refuse_motion(done, f"sideways {abs(rc) / 100.0:.2f} m is more than {lat_rel:g} x D = "
                                            f"{lat_rel * d:.2f} m (D {d:.2f} m; the drone's mvlat): a nearer object "
                                            "under a long sideways move makes it go further. Split it (the drone "
                                            "re-checks D in between)", now, **info)
        mfront = mac_min_front()
        k_low = cached_k_low()
        if fc > 0 and mfront > 0.0 and fwd_end_low(d, fc / 100.0, k_low) < mfront:
            fmax = fwd_max(d, mfront, k_low)
            return self.refuse_motion(done, f"forward {fc / 100.0:.2f} m could leave only "
                                            f"{fwd_end_low(d, fc / 100.0, k_low):.2f} m to the scene ahead (D {d:.2f} m, "
                                            f"an image expansion scale k down to {k_low:g}); the Mac keeps "
                                            f"{mfront:.2f} m: at most {fmax:.2f} m now (fly that, take a "
                                            "photo and judge again, or turn toward a deeper background)", now,
                                      fwd_max_m=fmax, D_m=d, **info)
        if gap is not None:
            # the person: the drone's travel error toward the scene at k down to K_LOW (its floor's model) would
            # land the whole overshoot of one long move at the person (2026-10-15 review: E = D = 4.7 m, the
            # runbook's 1.70 m gap, F 3.0 m: 0.86 m at k 0.6)
            low, over = person_gap_low(d, rc / 100.0, fc / 100.0, gap, est, pct, caps)
            speed = person_speed_term(d, rc / 100.0, fc / 100.0, pct, caps)
            info["person_low_m"] = round(low, 2)
            if speed > 0.0:
                info["person_speed_term_m"] = round(speed, 3)
            if low < PERSON_MIN_M - 1e-9:
                fmax = person_fwd_max(d, rc / 100.0, fc / 100.0, gap, est, pct, caps)
                sp = (f" (of it {speed:.2f} m the speed term at mvspeed {pct} %: {PERSON_SPEED_T:g} s x the forward "
                      "peak)") if speed > 0.0 else ""
                return self.refuse_motion(done, f"--person-gap {gap:.2f} m (--est {est:.2f} m): at an image "
                                                f"expansion scale k down to {K_LOW:g} the drone may fly {over:.2f} m"
                                                f"{sp} beyond {fc / 100.0:.2f} m forward (D {d:.2f} m) and end "
                                                f"{low:.2f} m from the person; {PERSON_MIN_M:.1f} m keeps at most "
                                                f"{fmax:.2f} m forward now", now, fwd_max_m=fmax, D_m=d, **info)
        h = a["hTarget_m"] + uc / 100.0
        if uc and not MOVE_H_RANGE_M[0] <= h <= MOVE_H_RANGE_M[1]:
            return self.refuse_motion(done, f"the height would be {h:.2f} m (commanded): it must stay "
                                            f"{MOVE_H_RANGE_M[0]:.2f}..{MOVE_H_RANGE_M[1]:.2f} m", now, **info)
        why = self.low_refusal(a, h)
        if why:
            return self.refuse_motion(done, why, now, **info)
        prof = move_profile_s(rc / 100.0, fc / 100.0, uc / 100.0, d, pct, caps)
        settle = settle_s(pct, caps)
        secs = batt_seconds_left(a["vbat_v"])
        if secs is not None and prof + settle > secs and not req.get("batt_ok"):
            lo, hi = 0.0, 1.0
            for _ in range(30):     # the longest move along the same line that ends before the battery's end
                mid = 0.5 * (lo + hi)
                if move_profile_s(mid * rc / 100.0, mid * fc / 100.0, mid * uc / 100.0, d, pct, caps) + settle <= \
                        secs:
                    lo = mid
                else:
                    hi = mid
            longest = lo * math.hypot(math.hypot(rc, fc), uc) / 100.0
            return self.refuse_motion(done, f"the battery reads {a['vbat_v']:.2f} V under load: ~{secs:.0f} s to "
                                            f"{BATT_END_V:.2f} V at {BATT_DUR_VPS * 1000:.0f} mV/s, and this move "
                                            f"needs {prof:.1f} s + {settle:g} s settle: the longest along "
                                            f"this line that fits is {longest:.2f} m (or --batt-ok: a BATT_LOW "
                                            "partway lands in place)", now, battery_s=round(secs, 1),
                                      longest_m=round(longest, 2), **info)
        info["D_m"] = d
        info["profile_s"] = round(prof, 1)
        late = late_climb_warning(a, uc / 100.0)
        if late:
            info["warnings"] = [late]
        if uc and self.sh_goal is not None:
            info["remembered_target_cleared_m"] = self.sh_goal["target_m"]
            self.sh_goal = None             # a vertical MOVE sets the commanded height itself
        if gap is not None or est is not None:
            self.event("person_gap", gap_m=gap, est_m=est, target_m=info["target_m"], D_m=d)
        self.motion_cmd(owner, T_MOVE, move_payload(rc, fc, uc), req, now,
                        self.move_timeout(rc / 100.0, fc / 100.0, uc / 100.0, d, pct, caps), info, done)

    def op_yaw(self, c, req, now):
        """YAW deg: + clockwise, to the right (as the photo bearings). Behind an early-returned motion it waits for
        that motion's end (wait_behind)."""
        deg = int(round(float(req["deg"])))
        info = {"target_deg": deg}
        if deg == 0 or abs(deg) > YAW_MAX_DEG:
            return self.refuse_motion(c.reply, f"deg must be -{YAW_MAX_DEG}..{YAW_MAX_DEG}, not 0", now, **info)
        if self.behind_needed():
            return self.wait_behind(c, now, c.reply, lambda n: self.op_yaw(c, req, n), "YAW")
        why = self.motion_refusal(now) or self.low_refusal(self.auto, self.auto["hTarget_m"])
        if why:
            return self.refuse_motion(c.reply, why, now, **info)
        pct = self.auto.get("mvSpeedPct", MV_SPEED_DEFAULT)
        info["profile_s"] = round(yaw_profile_s(deg, pct), 2)
        info["dwell_s"] = mv_limits(pct)["dwell_ms"] / 1000.0
        self.motion_cmd(c, T_YAW, yaw_payload(deg), req, now, self.yaw_timeout(deg), info, c.reply)

    @staticmethod
    def yaw_timeout(deg):
        """Generous: the turn at YAW_RATE_DPS, its dwell, a re-track bob of up to ~8 s, margin."""
        return min(75.0, 25.0 + 2.0 * abs(deg) / YAW_RATE_DPS)

    def op_stop(self, c, req, now):
        """STOP: ends a running MOVE or YAW where it is (the drone holds there); never refused by the drone.
        A MOVE or YAW still queued here is withdrawn. The STOP is the daemon's: sent even if its client goes."""
        why = self.refusal()
        if why:
            return c.reply({"ok": False, "result": "REFUSED", "error": why})
        for q in [q for q in self.queue if q.ptype in MOTION_TYPES]:
            self.withdraw(q, "withdrawn: STOP", now)
        for e in self.behind:   # a command waiting behind an early-returned motion (fast moves)
            e["withdrawn"] = f"withdrawn: STOP (the {e['what']} waited for the previous motion's end)"

        def acked(r):
            out = {"ok": r["ok"], "type": "STOP", "result": r.get("resultName")}
            if r.get("detail") is not None and r.get("result") is not None:
                _, found, seq = mv_detail(r["detail"])
                out["found"], out["mvSeq"] = name_of(MV_STATE_NAMES, found), seq
            if not r["ok"]:
                out["error"] = r.get("error") or str(ack_error("STOP", r))
            c.reply({**out, **self.brief(time.monotonic())})
        # daemon-owned (2026-10-15, docs/autonomy-a2b.md 22): never withdrawn when its client goes (a Bash timeout
        # must not cancel a collision-avoidance STOP); sent again until its ACK (a reply to a gone client is dropped)
        self.queue_flight(Cmd(T_STOP, b"", acked), now, priority=True)

    def op_wait_move(self, c, req, now):
        """Blocks until the motion running ends (at once when none runs), or the drone leaves HOVER; replies with
        its end, the travel (right / forward m, as the drone measured it), the target, D, the margins (what is left
        to the target along the move, the scene ahead) and, with photo, a photo taken then."""
        timeout = float(req.get("timeout", 60.0))
        n0 = self.auto_count
        self.send_status_req(now)

        def result(a, now):
            t, g = dict(a["prog_m"]), dict(a["tgt_m"])
            if a.get("mvCorrection"):
                # the latest motion is a SET_HEIGHT correction (mvFlags 0x40 on a MOVE): not Claude's MOVE
                return {"ok": a["mvEnd"] in MVE_OK, "end": a["mvEndName"], "kind": "SET_HEIGHT", "mvSeq": a["mvSeq"],
                        "travelled_m": t, "target_m": g, "D_m": a["sceneD_m"],
                        "note": "the latest motion was a SET_HEIGHT correction, not a MOVE", **self.brief(now)}
            along = move_along(t, g)
            rch = move_reached(a["mvEnd"], t, g)
            out = {"ok": a["mvEnd"] in MVE_OK or bool(rch.get("reached")),
                   "end": a["mvEndName"], "kind": a["mvKindName"], "mvSeq": a["mvSeq"], "travelled_m": t,
                   "target_m": g, "D_m": a["sceneD_m"],
                   "margins": {"to_target_m": None if along is None else
                               round((1.0 - along) * math.sqrt(sum(v * v for v in g.values())), 3),
                               "scene_ahead_m": a["sceneD_m"]}, **self.brief(now)}
            out.update({k: rch[k] for k in ("along", "off_m") if k in rch})
            if rch.get("reached"):
                out["reached"] = True
                out["note"] = rch["note"]
            elif "error" in rch:
                out["error"] = rch["error"]
            elif a["mvEnd"] not in MVE_OK and a["mvEnd"]:
                out["error"] = f"the {a['mvKindName']} ended {a['mvEndName']}" + MV_END_HINTS.get(a["mvEndName"], "")
            return out

        def check(now):
            a = self.auto
            if self.auto_count <= n0 or a is None:
                return None
            if not a["interactive"]:
                return {"ok": False, "error": "the drone's firmware has no interactive hover", **self.brief(now)}
            if a["state"] != S_HOVER:
                return {"ok": False, "end": a["mvEndName"], "error": f"the drone is not in HOVER: {a['stateName']} "
                                                                     f"({a['reasonName']})", **self.brief(now)}
            if a["mvActive"]:
                return None
            out = result(a, now)
            if not req.get("photo"):
                return out

            def photo_done(p):
                c.reply({**out, "photo": {k: p.get(k) for k in ("ok", "id", "photos", "failed", "dir", "error")
                                          if k in p}})
            self.photo_request(c, 1, int(req.get("gap", 150)), float(req.get("photo_timeout", 10.0)), None, now,
                               photo_done)
            return False
        self.defer_fn(c, check, now + timeout,
                      lambda: {"ok": False, "error": f"the motion has not ended within {timeout:g} s (it keeps "
                                                     "running; STOP ends it)", **self.brief(time.monotonic())},
                      c.reply)

    # -------- SET_HEIGHT and the level line (2026-10-15, docs/autonomy-a2b.md 23)

    def sh_photo(self, ref):
        """The photo whose estimate at capture is the h_at of a set-height in the offset form: ref (a path), else the
        latest saved in this sortie. (doc, path) or (None, why): another sortie or boot, or not newer than the latest
        accepted SET_HEIGHT (one photo is never applied twice)."""
        if ref:
            path = photo_json_path(ref)
            try:
                with open(path) as f:
                    doc = json.load(f)
            except (OSError, ValueError) as e:
                return None, f"cannot read the photo {path}: {e}"
        else:
            if not self.photo_log:
                return None, "no photo saved by this daemon: take a photo, or use --now / --at"
            path = self.photo_log[-1]["json"]
            try:
                with open(path) as f:
                    doc = json.load(f)
            except (OSError, ValueError) as e:
                return None, f"cannot read the latest photo {path}: {e}"
        au = doc.get("autonomy") or {}
        boot = (doc.get("link") or {}).get("bootNonce")
        if boot != hex32(self.boot or 0) or au.get("sortie_key") is None or au.get("sortie_key") != self.sortie_key:
            return None, (f"the photo {os.path.basename(path)} is from another sortie or boot: take a new photo, or use "
                          "--now")
        if au.get("h_est_capture_m") is None:
            return None, f"the photo {os.path.basename(path)} has no estimate at capture: use --now or --at"
        used = self.sh_used
        cap = au.get("capture_mono")
        if used is not None and (cap is None or cap <= used[0]):
            if used[1] == "unknown":
                return None, ("the last set-height's outcome is unknown (it may have been applied): no photo since it is "
                              "used again (one photo is never applied twice): take a new photo, or check 'state' and "
                              "use --now")
            return None, ("no photo since the last accepted set-height (one photo is never applied twice): take a new "
                          "photo, or use --now")
        return doc, path

    def op_set_height(self, c, req, now):
        """SET_HEIGHT (0x33): Claude's judged true height X when the judged photo was captured (the offset form, h_at its
        estimate then; d = X - h_at: moves since the photo cancel out), or now (--now: d = X - the estimate now), or
        against an estimate given (--at). Since 2026-10-16 (docs/autonomy-a2b.md 27) the drone takes X as its true
        height and flies to the target: the commanded height (take-off height + the moves; hTarget_m), or --to H (which
        becomes the commanded height); a flight over 0.55 m is clipped here to 0.50 m toward the target (the rest is
        named). A drone without that form (7a78e1b, 8277b68: the link answers the 6-byte SET_HEIGHT RANGE, detail 0)
        gets the 4-byte form with X + h_now - target (it flies back to its old estimate: the same end; its estimate
        stays ~0.1 m off as after the take-off). No decisions here: the drone checks the state, the step, the rate, the
        ranges and the climb's thrust headroom. The reply: the result, the old estimate (from the ACK), offset_m (d),
        true_now_m, new_m (the drone's estimate right after its shift), target_m, fly_m, flies_to_m (the true end), the
        motion seq; by default once the correction has ended (its end, h, duration).
        2026-10-16 review: a clipped flight's target is remembered (sh_goal) and is the next set-height's default
        target until it is reached (a set-height not clipped), a vertical MOVE, --to or a new sortie; a target-form
        descent under a MOVE down's floor is refused here (the drone refuses it HEIGHT); legacy: the 4-byte form as
        before 2026-10-16 (X, h_at; it flies back to its old estimate), for the bench's A/B; warnings for a photo with
        events since it and for a climb late in the battery (LATE_CLIMB_*)."""
        x = float(req["x_m"])
        if not (math.isfinite(x) and SH_X_M[0] - 1e-9 <= x <= SH_X_M[1] + 1e-9):
            raise ValueError(f"x_m {SH_X_M[0]:g}..{SH_X_M[1]:g}")
        x_cm = int(round(x * 100))
        to_req = req.get("to_m")
        if to_req is not None:
            to_req = float(to_req)
            if not (math.isfinite(to_req) and SH_TO_M[0] - 1e-9 <= to_req <= SH_TO_M[1] + 1e-9):
                raise ValueError(f"to_m {SH_TO_M[0]:g}..{SH_TO_M[1]:g}")
            to_req = int(round(to_req * 100)) / 100.0
        legacy = bool(req.get("legacy"))
        if legacy and to_req is not None:
            raise ValueError("--legacy (the 4-byte form) has no target: it flies back to the old estimate")
        info = {"x_m": x_cm / 100.0}
        if self.behind_needed():   # fast moves: behind an early-returned motion, wait for its end (wait_behind)
            return self.wait_behind(c, now, c.reply, lambda n: self.op_set_height(c, req, n), "SET_HEIGHT")
        if self.prev_final is not None:
            # the motion it waited behind: its final end, in this reply (refused or not; the review of 2026-10-17: it
            # went stale and came with the next MOVE / YAW instead)
            info["previous"], self.prev_final = self.prev_final, None
        wait = req.get("wait", True)
        why = self.refusal()
        a = self.auto
        if why is None and (a is None or self.auto_rx is None or now - self.auto_rx > MOTION_STATUS_MAX_AGE_S):
            why = "no fresh AUTO_STATUS from the drone"
        busy = [q for q in self.queue + ([self.flight] if self.flight is not None else [])
                if q.ptype in MOTION_TYPES and not q.finished]
        if why is None and busy:
            why = f"a {TYPE_NAMES[busy[0].ptype]} is being sent"
        at_cm, at_photo, photo_age, ev_since = None, None, None, []
        if why is None and not req.get("now"):
            if req.get("at_m") is not None:
                at = float(req["at_m"])
                if not (math.isfinite(at) and SH_AT_M[0] <= at <= SH_AT_M[1]):
                    raise ValueError(f"at_m {SH_AT_M[0]:g}..{SH_AT_M[1]:g}")
                at_cm = int(round(at * 100))
            else:
                doc, path = self.sh_photo(req.get("photo"))
                if doc is None:
                    why = path
                else:
                    at_cm = int(round(doc["autonomy"]["h_est_capture_m"] * 100))
                    at_photo = path
                    cap = doc["autonomy"].get("capture_mono")
                    photo_age = round(now - cap, 1) if isinstance(cap, (int, float)) else None
                    ev_since = events_since(a, doc["autonomy"].get("evCount_capture"))
        info.update(at_m=None if at_cm is None else at_cm / 100.0, at_photo=at_photo,
                    form="now" if req.get("now") else "at" if req.get("at_m") is not None else "photo")
        if photo_age is not None:
            info["photo_age_s"] = photo_age
        if why:
            return c.reply({"ok": False, "result": "REFUSED", "type": "SET_HEIGHT", "error": why, **info,
                            **self.brief(now)})
        h0 = a["h_m"]                    # the estimate when it was sent (an old estimate the ACK may not carry)
        at_m = None if at_cm is None else at_cm / 100.0
        goal = self.sh_goal if (to_req is None and not legacy and self.sh_goal is not None and
                                self.sh_goal.get("sortie_key") == self.sortie_key) else None
        target = to_req if to_req is not None else goal["target_m"] if goal is not None else a["hTarget_m"]
        plan = set_height_plan(x_cm / 100.0, at_m, h0, target)
        to_cm = SH_TO_REF if (to_req is None and goal is None) else int(round(target * 100))
        clip = None
        if legacy:
            # the 4-byte form as before 2026-10-16 (the bench's A/B, bench R): back to the old estimate (its true end)
            target = h0
            plan = set_height_plan(x_cm / 100.0, at_m, h0, h0)
            to_cm = None
        elif abs(plan["fly_m"]) > SH_FLY_MAX_M + 1e-9:
            # one command flies at most 0.55 m: 0.50 m of it toward the target now, the rest named
            to_c = plan["true_now_m"] + math.copysign(SH_FLY_CLIP_M, plan["fly_m"])
            to_c = min(max(to_c, SH_TO_M[0]), SH_TO_M[1])
            to_cm = int(round(to_c * 100))
            clip = {"clipped_to_m": to_cm / 100.0, "rest_m": round(target - to_cm / 100.0, 3)}
        info.update(target_m=round(target, 3), target_from="legacy: the old estimate" if legacy else "--to" if
                    to_req is not None else "remembered" if goal is not None else "commanded", t_hold=a.get("tHold"))
        if goal is not None:
            info["remembered_note"] = (f"the target {goal['target_m']:.2f} m of the clipped set-height at "
                                       f"{goal['utc']} (pass --to to fly elsewhere)")
        if clip:
            info.update(clip)
        if not legacy and plan["fly_m"] < 0:
            end = to_cm / 100.0 if to_cm != SH_TO_REF else target
            lo = height_floor(a.get("air_s"))
            if end < lo - 1e-9:
                return c.reply({"ok": False, "result": "REFUSED", "type": "SET_HEIGHT", "why": "HEIGHT",
                                "error": f"the correction would end at {end:.2f} m, under a MOVE down's floor "
                                         f"{lo:.2f} m after {a.get('air_s') or 0:.0f} s of air (the estimate may read "
                                         "high: an old photo can leave the drone lower than X says): pick a higher "
                                         "--to, or land", "floor_m": round(lo, 2), **info, **self.brief(now)})
        warnings = []
        if photo_age is not None and photo_age > SH_PHOTO_OLD_S:
            warnings.append(f"the photo is {photo_age:.0f} s old: the estimate's drift since it (and any sink) is not "
                            "in this correction; act within ~10 s of a photo")
        if ev_since:
            warnings.append(f"the drone reported {', '.join(ev_since)} since the photo was captured: the photo may not "
                            "show where it is now (a contact or a catch moves it, a CAM_LOST lets it sink); take a new "
                            "photo")
            info["events_since_photo"] = ev_since
        if a.get("tHold") and a["tHold"] > AUTO_MV_REFUSE_T_MAC and plan["fly_m"] > 0:
            warnings.append(f"T_hold {a['tHold']} is above {AUTO_MV_REFUSE_T_MAC}: the drone refuses a climb "
                            "(UP_HEADROOM): come down or land")
        else:
            late = late_climb_warning(a, plan["fly_m"])
            if late:
                warnings.append(late)
        if warnings:
            info["warnings"] = warnings
        cmd_box = {}

        def send(form, payload):
            cmd = Cmd(T_SET_HEIGHT, payload, None, owner=c)
            cmd.mv_seq0 = self.auto["mvSeq"] if self.auto else 0
            cmd.done = lambda r: acked(r, form)
            cmd_box["cmd"] = cmd
            self.queue_flight(cmd, time.monotonic())

        def acked(r, form):
            now = time.monotonic()
            res, detail = r.get("result"), r.get("detail")
            out = {"ok": r["ok"], "type": "SET_HEIGHT", "result": r.get("resultName"), **info, "sh_form": form}
            for k in ("reconciled", "resent"):
                if k in r:
                    out[k] = r[k]
            if res == R_BAD_STATE and detail == 0 and not r.get("reconciled"):
                fw = (self.link.hello_ack or {}).get("fw")
                out.update(ok=False, why="UNSUPPORTED",
                           error=f"firmware has no SET_HEIGHT (fw {fw}): the drone answered BAD_STATE detail 0 (an "
                                 "unknown type); correct with 'move --up/--down' by the judged error")
                self.event("set_height", **info, result="UNSUPPORTED", fw=fw)
                return c.reply({**out, **self.brief(now)})
            if form == "target" and res == R_RANGE and detail == 0 and not r.get("reconciled"):
                # the link's length check: a drone without the target form (not processed): the 4-byte form that ends
                # at the same height (it flies back to its old estimate)
                h_now = self.auto["h_m"] if self.auto else h0
                tgt = to_cm / 100.0 if to_cm != SH_TO_REF else target
                xl = set_height_legacy_x(x_cm / 100.0, at_m, h_now, tgt)
                xl_cm = int(round(xl * 100))
                info.update(legacy_x_m=xl_cm / 100.0, legacy_note=(
                    "this drone has no target form (fw before 2026-10-16): sent X + h_now - target in the 4-byte form, "
                    "which flies back to its old estimate; its estimate then reads h_now - target off"))
                self.event("set_height_form", form="legacy-target", x_m=info["x_m"], legacy_x_m=xl_cm / 100.0,
                           h_now_m=h_now, target_m=round(tgt, 3))
                if not (round(SH_X_M[0] * 100) <= xl_cm <= round(SH_X_M[1] * 100)):
                    out.update(ok=False, result="REFUSED", error=f"the 4-byte form's X {xl_cm / 100.0:.2f} m is outside "
                               f"{SH_X_M[0]:g}..{SH_X_M[1]:g} m: correct with 'move --up/--down'")
                    return c.reply({**out, **self.brief(now)})
                return send("legacy-target", set_height_payload(xl_cm, SH_AT_NOW if at_cm is None else at_cm))
            if r.get("error") or res is None:
                # the outcome unknown (no ACK after the resends, a reconcile ending UNKNOWN): it may have been applied,
                # so the photos up to now count as used (the default photo is never applied twice)
                self.sh_used = (now, "unknown")
                out.update(ok=False, outcome="unknown", error=(r.get("error") or "SET_HEIGHT: no answer") +
                           " (the outcome is unknown: it may have been applied; take a new photo before another "
                           "set-height)")
                self.event("set_height", **info, result=None, outcome="unknown", error=out["error"])
                return c.reply({**out, **self.brief(now)})
            known = detail is not None and not r.get("reconciled") and not r.get("old_unknown")
            old = (int(struct.unpack("<h", struct.pack("<H", (detail >> 16) & 0xFFFF))[0]) / 100.0) if known else h0
            tgt = old if to_cm is None else to_cm / 100.0 if to_cm != SH_TO_REF else target
            pl = set_height_plan(x_cm / 100.0, at_m, old, tgt)
            if form in ("legacy-target", "legacy"):
                # the 4-byte form: the drone shifts its estimate by d_sent and flies back to its old estimate; its
                # estimate right after the shift is old + d_sent (not the true height when X was rewritten)
                xs = info["legacy_x_m"] if form == "legacy-target" else x_cm / 100.0
                d_sent = xs - (at_m if at_m is not None else old)
                fly = round(-d_sent if old <= MOVE_H_RANGE_M[1] else MOVE_H_RANGE_M[1] - (old + d_sent), 3)
                new_m = round(old + d_sent, 3)
            else:
                fly = pl["fly_m"]
                new_m = pl["true_now_m"]
            out.update(old_m=round(old, 3), old_from="ACK" if known else "status", offset_m=pl["offset_m"],
                       true_now_m=pl["true_now_m"], new_m=new_m, fly_m=fly,
                       flies_to_m=round(pl["true_now_m"] + fly, 3))
            keys = ("x_m", "at_m", "at_photo", "old_m", "offset_m", "true_now_m", "target_m", "fly_m", "new_m",
                    "sh_form", "clipped_to_m", "rest_m", "photo_age_s")
            if not r["ok"]:
                code = (detail >> 8) & 0xFF if detail is not None else 0
                out["why"] = name_of(MV_WHY_NAMES, code)
                hint = SH_WHY_HINTS.get(out["why"])
                out["error"] = r.get("error") or f"SET_HEIGHT refused {r.get('resultName')}" + \
                    (f" ({out['why']}: {hint})" if hint else "")
                self.event("set_height", **{k: out.get(k) for k in keys if k in out},
                           result=out["result"], why=out["why"])
                return c.reply({**out, **self.brief(now)})
            self.last_sh = {"utc": utc_str(), "mono": now, "offset_m": pl["offset_m"], "new_m": new_m,
                            "x_m": info["x_m"], "at_m": info["at_m"], "target_m": round(tgt, 3), "fly_m": fly}
            if form in ("target", "legacy-target"):
                # a clipped flight: its target stays the default until reached (2026-10-16 review); else reached
                self.sh_goal = {"target_m": round(target, 3), "sortie_key": self.sortie_key, "utc": utc_str()} \
                    if clip else None
            self.sh_used = (now, "accepted")
            sent["t"], sent["n0"] = now, self.auto_count
            mv_seq0 = cmd_box["cmd"].mv_seq0

            def seq_of(a):
                return a["mvSeq"] if a is not None and a.get("mvCorrection") and \
                    a["mvSeq"] == ((mv_seq0 + 1) & 0xFFFF) else None
            self.event("set_height", **{k: out.get(k) for k in keys if k in out},
                       seq=seq_of(self.auto), result="OK", why="NONE")

            def check(now):
                a = self.auto
                if self.auto_count <= sent["n0"] or a is None:
                    return None
                seq = seq_of(a)
                if seq is not None:
                    out["seq"] = seq
                if a["state"] != S_HOVER:
                    end = {"end": a["mvEndName"], "h_m": a["h_m"], "progUp_m": a["prog_m"]["up"],
                           "dur_s": round(now - sent["t"], 1)}
                    self.event("set_height_end", **end, left=a["stateName"])
                    return {**out, "ok": False, **end, "error": f"the drone left HOVER during the correction: "
                                                                f"{a['stateName']} ({a['reasonName']})",
                            **self.brief(now)}
                if not wait:
                    return {**out, "running": seq is not None and a["mvActive"], **self.brief(now)} \
                        if seq is not None or now - sent["t"] >= 0.5 else None
                if seq is None or a["mvState"] not in (MV_DONE, MV_STOPPED):
                    return None
                end = {"end": a["mvEndName"], "h_m": a["h_m"], "progUp_m": a["prog_m"]["up"],
                       "dur_s": round(now - sent["t"], 1)}
                self.event("set_height_end", seq=seq, **end)
                res = {**out, "ok": a["mvEnd"] in MVE_OK, **end, **self.brief(now)}
                if a["mvEnd"] not in MVE_OK:
                    res["error"] = "the correction ended " + a["mvEndName"] + MV_END_HINTS.get(a["mvEndName"], "")
                return res
            self.defer_fn(c, check, now + (SH_WAIT_S if wait else 1.0),
                          lambda: {**out, "ok": not wait, **({} if not wait else
                                                             {"error": f"the correction has not ended within "
                                                                       f"{SH_WAIT_S:g} s (it keeps running; STOP "
                                                                       "ends it)"}),
                                   **self.brief(time.monotonic())}, c.reply)
        sent = {}
        if legacy:
            self.event("set_height_form", form="legacy", x_m=info["x_m"])
            return send("legacy", set_height_payload(x_cm, SH_AT_NOW if at_cm is None else at_cm))
        send("target", set_height_payload(x_cm, SH_AT_NOW if at_cm is None else at_cm, to_cm))

    def op_level_cal(self, c, req, now):
        """level-cal through the daemon (the setting it reads before each photo; its event in this log)."""
        out = level_cal_run(req, self.photo_dir, self.event, hex32(self.boot) if self.boot else None)
        self.level_mtime = None          # re-read at the next photo
        c.reply(out)

    def defer_fn(self, c, check, deadline, on_timeout, done, on_gone=None):
        """As defer(), with done(reply) instead of c.reply; check returning False ends the wait without a
        reply (it has passed the work on); on_gone() runs when the client has gone (its cleanup)."""
        def w(now):
            if c.ended():
                if on_gone is not None:
                    on_gone()
                return True
            r = check(now)
            if r is False:
                return True
            if r is None and now >= deadline:
                r = on_timeout()
            if r is None:
                return False
            done(r)
            return True
        if not w(time.monotonic()):
            self.waiters.append(w)

    OPS = {"ping": op_ping, "hello": op_hello, "state": op_state, "status": op_status, "arm": op_arm,
           "takeoff": op_takeoff, "land": op_land, "disarm": op_disarm, "kill": op_kill, "photo": op_photo,
           "wait": op_wait, "events": op_events, "sortie": op_sortie, "move": op_move, "yaw": op_yaw,
           "stop": op_stop, "wait-move": op_wait_move, "set-height": op_set_height, "level-cal": op_level_cal}


class Sortie:
    """ARM, TAKEOFF, then wait until the drone hovers waiting with the take-off photos on disk (or
    the sortie ends); with then=land, LAND and wait for IDLE.

    Once a sortie has ended (its reply sent, its timeout, its client gone, a KILL, a LAND or DISARM
    from anyone), nothing more is sent for it, never a TAKEOFF: its queued command is withdrawn, and
    the ARM or TAKEOFF already sent is undone on the ground when its answer comes (Daemon.undo: DISARM;
    a TAKEOFF that may have acted is decided from a fresh AUTO_STATUS: DISARM if still ARMED, else it
    flies on, takeoff_late). A sortie its client abandoned (the connection closed before the reply, e.g.
    Ctrl-C; a client that only shut down its write side is still there) or that timed out after its
    TAKEOFF holds in the air (sortie_holding). Its own ARM failing without a usable answer (it may have
    acted) is stopped too: DISARM unless the drone flies. Since 2026-10-15 ("Claude decides") the Mac
    never LANDs on its own: only 'l', 'q', the land op and --then land do."""

    def __init__(self, d, c, req, mode, now):
        self.d, self.c, self.mode = d, c, mode
        self.then = req.get("then", "wait")
        if self.then not in ("wait", "land"):
            raise ValueError("then must be wait or land")
        self.air = check_air_ms(int(req.get("air_ms") or 0))
        self.req = req
        # the planned move (docs/autonomy-a2b.md 21.4): range checks only (D is not known before the take-off)
        self.plan = plan_cm(req)
        self.via = req.get("plan_via", PLAN_VIA_DEFAULT)
        if any(self.plan) and self.then != "wait":
            raise ValueError("a planned move needs --then wait")
        gap = req.get("person_gap")
        if any(self.plan) and gap is not None and not (math.isfinite(float(gap)) and float(gap) >= PERSON_MIN_M):
            raise ValueError(f"--person-gap {float(gap):.2f} m: a move never ends closer than {PERSON_MIN_M:.1f} m to "
                             "a person (user rule 2026-10-13)")
        if any(self.plan) and gap is not None:
            # a person approach needs the live D (the drone's travel error at k down to K_LOW, move_request): the
            # Mac sends it in the wait (2026-10-15 review, until the flight acceptance has measured k in the room)
            if req.get("est") is None:
                raise ValueError("--person-gap needs --est (your distance to that person now)")
            if self.via == "drone":
                raise ValueError("a planned move toward a person goes --plan-via mac: the Mac checks your gap "
                                 "against the drone's travel at an image expansion scale k down to "
                                 f"{K_LOW:g} with the live D; the drone sees only the scene")
        note = d.takeoff_cmd(0, req, None).note   # range checks before anything is sent
        self.deadline = now + (float(req["timeout"]) if req.get("timeout") is not None else
                               photo_set_timeout_s(req.get("photos", 1)))
        self.phase = "arm"
        self.count0 = None
        self.photo_id = None
        self.out = {"note": note} if note else {}
        self.plan_wait_t = None   # the first status of the wait (--plan-via drone: its outcome within PLAN_STATUS_S)
        if any(self.plan):
            r, f = self.plan[0] / 100.0, self.plan[1] / 100.0
            self.deadline += Daemon.move_timeout(r, f, 0.0, None)
            d.event("plan", via=self.via, target_m={"right": r, "fwd": f}, gap_m=gap, est_m=req.get("est"))
            self.out["plan"] = {"via": self.via, "target_m": {"right": r, "fwd": f}, "state": "pending"}
            rest = d.status["vbat_v"] if d.status and d.status.get("vbat_v") else None
            if d.auto is not None and d.auto["state"] == S_IDLE:
                rest = d.auto["vbat_v"]
            if rest is not None and rest < PLAN_REST_WARN_V:
                self.out["warning"] = (f"the battery rests at {rest:.2f} V (< {PLAN_REST_WARN_V:.2f}): a long planned "
                                       "move may not fit the charge (the drone lands BATT_LOW at 3.00 V under load, "
                                       "in place)")
        d.sorties.append(self)
        self.cmd = Cmd(T_ARM, arm_payload(mode, self.air), self.armed, owner=self)   # the ARM or TAKEOFF in progress
        d.queue_flight(self.cmd, now)
        d.waiters.append(self.step)

    def ended(self):
        """True once the sortie is over. Its deadline passing, or its client going away (closed, not merely
        half-closed: Client.check), ends it (abandoned) here, so every decision that asks (the ARM's answer,
        a TAKEOFF about to be queued, sent or sent again) sees the timeout at once, not only at step()."""
        if self.phase != "done":
            if time.monotonic() >= self.deadline:
                self.end(False, f"timeout in phase {self.phase}", safe=True)
            elif self.c.check():
                self.end(False, "the sortie's client went away", safe=True)
        return self.phase == "done"

    def armed(self, r):
        self.cmd = None
        now = time.monotonic()
        if self.ended():
            return self.d.undo(T_ARM, r, "the sortie had ended when its ARM was answered", now)
        a, disarm = self.d.arm_result(r, self.mode)
        self.out["arm"] = {k: a.get(k) for k in ("result", "token", "guards", "vbat", "vmin", "cam_wake_retries")}
        if not a["ok"]:
            # An ARM that may have acted without a usable answer (its token lost, no ACK after the retries,
            # BUSY beyond BUSY_RESENDS, a taken-unknown reconcile ending UNKNOWN) never ends the sortie
            # silently: DISARM unless the drone flies (never a LAND), told in the reply.
            stops = [T_DISARM] if disarm else self.d.arm_unknown_stops(r)
            extra = {"stops": [TYPE_NAMES[t] for t in stops]} if stops else {}
            error = a.get("error")
            if not disarm and r.get("sent") and r.get("result") in (None, R_BUSY):
                error += self.d.arm_unknown_note(stops)
            self.end(False, error, result=a.get("result"), **extra)
            for t in stops:
                self.d.stop_request(t, "sortie: " + ("the ARM token was lost" if disarm else
                                                     "the ARM may have acted without an answer"), now)
            return
        if self.ended():
            return self.d.undo(T_ARM, r, "the sortie had ended when its TAKEOFF was due", now)
        self.phase = "takeoff"
        self.cmd = self.d.takeoff_cmd(a["token"], self.req, self.took_off, owner=self)
        self.d.queue_flight(self.cmd, now)

    def took_off(self, r):
        self.cmd = None
        now = time.monotonic()
        if self.ended():
            return self.d.undo(T_TAKEOFF, r, "the sortie had ended when its TAKEOFF was answered", now)
        self.out["takeoff"] = {"result": r.get("resultName"), "reconciled": r.get("reconciled", False)}
        if not r["ok"]:
            err = r.get("error") or str(ack_error("TAKEOFF", r))
            if any(self.plan) and self.via == "drone" and r.get("result") == R_RANGE and r.get("detail") == 0:
                # the link's length check: an older drone does not parse the 18-byte TAKEOFF (not processed)
                err = ("this build has no planned take-off: --plan-via mac (the drone answered the 18-byte TAKEOFF "
                       "RANGE, detail 0: not processed, no take-off)")
                self.out["plan"]["state"] = "unsupported"
            self.end(False, err, result=r.get("resultName"))
            return self.d.undo(T_TAKEOFF, r, "sortie: TAKEOFF not done", now)
        self.phase, self.count0 = "flying", self.d.auto_count
        self.ended()                        # a client gone by now: the drone holds (sortie_holding)

    def step(self, now):
        if self.ended():
            return True
        d = self.d
        a = d.auto if self.count0 is not None and d.auto_count > self.count0 else None
        if a is not None and a["photoId"] and a["capture"] != CAP_NONE:
            self.photo_id = a["photoId"]
        if self.phase == "flying" and a is not None:
            if a["state"] == S_HOVER and a["waiting"] and d.fsm_set_ready(a):
                if any(self.plan):
                    return self.plan_step(a, now)
                if self.then == "wait":
                    return self.end(True, None)
                self.out["waiting"] = d.brief(now)
                self.out["photos"] = self.photos()
                self.phase = "landing"
                d.land("sortie --then land", now)
            elif a["state"] in (S_LAND, S_KILLED, S_IDLE):
                return self.end(False, f"the sortie ended before the hover wait: {a['stateName']} ({a['reasonName']}"
                                       + (f", vz {a['vzLandReasonName']}" if a["vzLandReason"] else "") + ")")
        elif self.phase == "plan" and a is not None and a["state"] in (S_LAND, S_KILLED, S_IDLE):
            return self.end(False, f"the sortie ended before the plan's answer: {a['stateName']} ({a['reasonName']})")
        elif self.phase == "landing" and a is not None and a["state"] in (S_IDLE, S_KILLED):
            return self.end(True, None, final={"state": a["stateName"], "reason": a["reasonName"],
                                               "vzLandReason": a["vzLandReasonName"]})
        if now >= self.deadline:
            return self.end(False, f"timeout in phase {self.phase}", safe=True)
        return False

    def plan_step(self, a, now):
        """In the wait with the take-off photos on disk: the plan's outcome. --plan-via drone: the drone started (or
        refused) it on the wait's first step, the next AUTO_STATUS shows it; mac: the MOVE goes now, through
        op_move's checks, and the reply comes at its ACK. Not after an event (2026-10-15 verification): the drone
        reported something before the wait (a contact, NOT_STABLE, the camera height lost, a controller event ...):
        each of those landed until "Claude decides", so the plan never flew after one; now the drone cancels its own
        (mvWhy PLAN_CANCELLED) and the Mac does not send its MOVE: plan.state 'cancelled' with the events, the drone
        hovers in the wait for Claude to judge a photo."""
        p = self.out["plan"]
        if self.via == "mac":
            if self.phase != "flying":
                return False
            if a.get("hr") and a["evCount"]:
                return self.plan_cancelled(a)   # (evCount counts this sortie's events: 0 at its ARM)
            self.phase = "plan"
            req = {"right": self.plan[0] / 100.0, "fwd": self.plan[1] / 100.0, "wait": False,
                   "person_gap": self.req.get("person_gap"), "est": self.req.get("est"),
                   "batt_ok": self.req.get("batt_ok")}
            self.d.event("plan_send", via="mac", target_m=p["target_m"], status_age_ms=int((now - self.d.auto_rx) * 1000))
            self.d.move_request(self, req, now, self.plan_sent)
            return False
        if a["mvSeq"] >= 1:
            p.update(state="running" if a["mvActive"] else "ended", mvSeq=a["mvSeq"], end=a["mvEndName"],
                     D_m=a["sceneD_m"])
            return self.end(True, None)
        if a["mvWhyName"] == "PLAN_CANCELLED":
            return self.plan_cancelled(a)
        if a["mvWhy"]:
            self.plan_refused(a, a["mvWhyName"], MV_WHY_HINTS.get(a["mvWhyName"]))
            return self.end(True, None)
        if self.plan_wait_t is None:
            self.plan_wait_t = now
        if now - self.plan_wait_t >= PLAN_STATUS_S:
            p.update(state="unknown", error=f"the drone showed neither the plan's motion nor a refusal within "
                                            f"{PLAN_STATUS_S:g} s of the wait: 'state', then a plain 'move'")
            return self.end(True, None)
        return False

    def plan_cancelled(self, a):
        """The plan not flown: the drone reported events before the hover wait (hold.events)."""
        h = a.get("hold") or {}
        names = [e["name"] for e in h.get("events", [])]
        self.out["plan"].update(state="cancelled", why="PLAN_CANCELLED", hint=MV_WHY_HINTS["PLAN_CANCELLED"],
                                events=names, evCount=a.get("evCount"))
        self.d.event("plan_cancelled", via=self.via, events=names, evCount=a.get("evCount"))
        return self.end(True, None)

    def plan_refused(self, a, why, hint):
        d = a["sceneD_m"]
        self.out["plan"].update(state="refused", why=why, hint=hint, D_m=d,
                                fwd_max_m=None if d is None else fwd_max(d, mac_min_front(), cached_k_low()),
                                note="the drone hovers in the wait: judge the photo, then a plain 'move' (or turn "
                                     "toward a deeper background)")

    def plan_sent(self, r):
        """--plan-via mac: the MOVE's answer (running, or refused by the Mac or the drone)."""
        if self.phase == "done":
            return
        a = self.d.auto
        p = self.out["plan"]
        if r.get("ok"):
            p.update(state="running", mvSeq=r.get("mvSeq"), D_m=r.get("D_m"), profile_s=r.get("profile_s"))
        else:
            why = r.get("why") or "MAC"
            self.plan_refused(a, why, MV_WHY_HINTS.get(why))
            p["error"] = r.get("error")
            for k in ("fwd_max_m", "longest_m", "battery_s"):
                if r.get(k) is not None:
                    p[k] = r[k]
        self.end(True, None)

    def photos(self, failed=False):
        s = self.d.find_set(self.photo_id) if self.photo_id else None
        return [] if s is None else s.failures() if failed else s.photos()

    def end(self, ok, error, safe=False, **extra):
        """Replies (if the client is still there) and ends the sortie. safe: it was abandoned (client gone,
        timeout), so stop what it started: its ARM or TAKEOFF is withdrawn (its answer, when it comes, is
        undone); once the TAKEOFF went through the drone holds (no LAND since 2026-10-15: Claude decides)."""
        if self.phase == "done":
            return True
        phase, self.phase = self.phase, "done"
        if self in self.d.sorties:
            self.d.sorties.remove(self)
        r = {"ok": ok, **self.d.brief(time.monotonic()), "photos": self.out.pop("photos", None) or self.photos(),
             "photos_failed": self.photos(failed=True), "photo_id": self.photo_id, **self.out, **extra}
        if error:
            r["error"] = error
        self.c.reply(r)
        if safe:
            now = time.monotonic()
            self.d.event("sortie_abandoned", phase=phase, why=error, state=self.d.state_name())
            if self.cmd is not None and not self.cmd.finished:
                self.d.withdraw(self.cmd, f"withdrawn: {error}", now)     # armed()/took_off() undo its answer
            elif phase in ("flying", "plan"):
                # in the air: no LAND for it since 2026-10-15 ("Claude decides"): the drone holds; the next command
                # (or the battery, or a lost link) decides
                self.d.event("sortie_holding", phase=phase, why=error, state=self.d.state_name())
        return True


class DaemonError(Exception):
    """Nothing answers on the daemon socket; kind: absent, refused (stale), timeout, closed."""

    def __init__(self, msg, kind):
        super().__init__(msg)
        self.kind = kind


def daemon_request(path, req, timeout):
    """One request line to the daemon, one reply line back (a dict)."""
    if not os.path.exists(path):
        raise DaemonError(f"no daemon socket {path}", "absent")
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        try:
            s.connect(path)
        except socket.timeout:
            raise DaemonError(f"no answer within {timeout:g} s", "timeout") from None
        except FileNotFoundError:
            raise DaemonError(f"no daemon socket {path}", "absent") from None
        except OSError as e:
            raise DaemonError(f"nothing listens on it ({e.strerror or e})", "refused") from None
        buf = b""
        try:
            s.sendall((json.dumps(req) + "\n").encode())
            while b"\n" not in buf:
                chunk = s.recv(65536)
                if not chunk:
                    raise DaemonError("the daemon closed the connection without a reply", "closed")
                buf += chunk
        except socket.timeout:
            raise DaemonError(f"no answer within {timeout:g} s", "timeout") from None
        except OSError as e:
            raise DaemonError(f"connection failed ({e})", "closed") from None
    finally:
        s.close()
    try:
        return json.loads(buf.split(b"\n", 1)[0])
    except ValueError as e:
        raise DaemonError(f"unreadable reply ({e})", "closed") from None


# ---------------------------------------------------------------- commands

def open_link(args, key):
    """Link on the drone Wi-Fi interface, or to --host (tests, fake drones; bound only with --iface)."""
    if args.host:
        return Link(key, args.host, args.port, iface=args.iface)
    iface = args.iface
    if not iface:
        info = link_check()
        if not info["link_up"]:
            raise LinkError("drone link is not up: " + "; ".join(info["problems"]),
                            iface=info["iface"], route_iface=info["route_iface"])
        iface = info["iface"]
    return Link(key, DRONE_IP, args.port, iface=iface)


def hex32(v):
    return f"{v:08x}"


def hello_info(link):
    """The latest HELLO_ACK (a BAD_SESSION retry may have replaced the first)."""
    h = link.hello_ack
    return {"host": link.addr[0], "port": link.addr[1], "iface": link.iface, "session": hex32(h["session"]),
            "bootNonce": hex32(h["bootNonce"]), "fw": h["fw"], "drone_fp": h["keyFp"], "key_fp": link.fp,
            "rtt_ms": h["rtt_ms"], "tries": h["tries"]}


def cmd_keygen(args):
    path = args.key_file
    old_fp = None
    if os.path.exists(path):
        try:
            old_fp = key_fp(load_key(path))
        except LinkError:
            pass
        if not args.force:
            return {"ok": False, "error": f"{path} exists; --force replaces it (then run install-key again)",
                    "key_file": path, "key_fp": old_fp}, 1
    key = secrets.token_bytes(KEY_LEN)
    write_key(path, key)
    fp = key_fp(key)
    log(f"new key in {path}, fingerprint {fp}; install it with the drone on USB: "
        "python3 tools/espfly_auto.py install-key")
    return {"ok": True, "key_file": path, "key_fp": fp, "replaced_fp": old_fp}, 0


def cmd_install_key(args):
    key = load_key(args.key_file)
    fp = key_fp(key)
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    import espfly_diag  # USB console (pyserial); only this command opens a serial port
    try:
        port = args.serial_port or espfly_diag.find_port()
    except SystemExit as e:
        raise LinkError(str(e.code)) from None
    try:
        d = espfly_diag.Drone(port)
    except Exception as e:
        raise LinkError(f"cannot open {port}: {e}", port=port) from None
    secret = (key.hex(), key.hex().upper())

    def redact(s):
        for form in secret:
            s = s.replace(form, "<key>")
        return s

    try:
        reply, _ = d.command(f"autokey {key.hex()}", timeout=5.0)
    except TimeoutError as e:
        raise LinkError(redact(str(e)), port=port) from None
    except Exception as e:  # serial errors etc.: never let the command text escape
        raise LinkError(redact(f"{type(e).__name__}: {e}"), port=port) from None
    finally:
        d.close()
    reply = redact(reply)
    drone_fp = parse_fp_reply(reply)
    ok = reply.startswith("OK") and drone_fp == fp
    out = {"ok": ok, "port": port, "reply": reply, "drone_fp": drone_fp, "key_fp": fp, "match": drone_fp == fp,
           "reboot_needed": "reboot" in reply}
    if not ok:
        out["error"] = "the drone refused the key" if not reply.startswith("OK") else \
            "the drone's fingerprint does not match the local key"
    else:
        log(f"key {fp} installed on the drone" + ("; it is used after the next reboot" if out["reboot_needed"]
                                                    else ""))
    return out, 0 if ok else 1


def cmd_up(args):
    info = link_check()
    for w in info["warnings"]:
        log("warning: " + w)
    out = {"ok": info["link_up"], **info}
    if not info["link_up"]:
        out["error"] = "drone link is not up: " + "; ".join(info["problems"])
    return out, 0 if info["link_up"] else 1


def via_daemon(args):
    """The one-shots (hello, status, photo) and a running daemon: a HELLO of their own would open a
    new session and land an active sortie. True: a daemon answers, go through it. False: no daemon
    socket, keep the one-shot's own session. Refused: the socket is there but nothing answers."""
    if not os.path.exists(args.sock):
        return False
    try:
        daemon_request(args.sock, {"op": "ping"}, CLIENT_PROBE_S)
        return True
    except DaemonError as e:
        if args.force_hello:
            log(f"daemon socket {args.sock}: {e}; --force-hello: opening a session of our own")
            return False
        raise LinkError(f"daemon socket present but not answering; refusing to open a new session ({e}; a HELLO "
                        f"would land an active sortie). End the daemon (or remove a stale {args.sock}), or use "
                        "--force-hello", sock=args.sock) from None


def ask_daemon(args, req, timeout):
    """One request to the running daemon; its reply and exit code."""
    try:
        r = daemon_request(args.sock, req, timeout)
    except DaemonError as e:
        hint = ("start it in its own terminal: python3 tools/espfly_auto.py daemon" if e.kind == "absent" else
                "it is stopped, hung or gone")
        raise LinkError(f"daemon: {e} ({hint})", sock=args.sock) from None
    return r, 0 if r.get("ok") else 1


def cmd_hello(args):
    if via_daemon(args):
        return ask_daemon(args, {"op": "hello"}, 5.0)
    key = load_key(args.key_file)
    with open_link(args, key) as link:
        h = link.hello()
        log(f"session {h['session']:08x}, fw {h['fw']}, key {h['keyFp']}, {h['rtt_ms']} ms")
        return {"ok": True, **hello_info(link)}, 0


def cmd_status(args):
    if via_daemon(args):
        return ask_daemon(args, {"op": "status"}, 5.0)
    key = load_key(args.key_file)
    with open_link(args, key) as link:
        link.hello()
        s = link.status()
        if s["bootNonce"] != link.hello_ack["bootNonce"]:
            log("warning: STATUS bootNonce differs from HELLO_ACK: the drone restarted")
        s["bootNonce"] = hex32(s["bootNonce"])
        out = {"ok": True, **hello_info(link), "status": s}
        # Stage A2b answers STATUS_REQ with STATUS and then AUTO_STATUS (A1 firmware: STATUS only).
        if link.wait(lambda: link.auto_statuses, Link.RETRY_S):
            out["auto"] = auto_view(link.auto_statuses[-1])
        return out, 0


def cmd_photo(args):
    if args.timeout is None:
        args.timeout = photo_set_timeout_s(args.n)
    if via_daemon(args):
        return ask_daemon(args, {"op": "photo", "n": args.n, "gap": args.gap, "timeout": args.timeout,
                                 "out": args.out}, args.timeout + 10.0)
    key = load_key(args.key_file)
    out_dir = os.path.abspath(os.path.expanduser(args.out or DEFAULT_PHOTO_DIR))
    if out_dir.startswith(os.path.abspath(DEFAULT_DIR) + os.sep):
        private_dir(DEFAULT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    started = time.monotonic()
    with open_link(args, key) as link:
        link.hello()
        ack = link.command(T_PHOTO, photo_payload(args.n, args.gap))
        kind, held = busy_kind(ack["detail"]) if ack["result"] == R_BUSY else (None, None)
        if kind == "held_set" and held:     # only a held_set BUSY names a photo id
            log(f"photo {held} from an earlier run is still held; freeing it")
            link.command(T_PHOTO_FREE, photo_free_payload(held))
            ack = link.command(T_PHOTO, photo_payload(args.n, args.gap))
        if ack["result"] != R_OK:
            raise ack_error("PHOTO", ack)
        photo_id = ack["detail"] & 0xFFFF
        if not photo_id:
            raise LinkError("PHOTO accepted, but no photo set was started (the camera is not running or has no "
                            "photo slots)", result="OK", detail=0)
        accepted = time.monotonic()
        log(f"photo {photo_id}: {args.n} frame(s), {args.gap} ms apart")
        free = None
        saved, failed = [], []
        try:
            done, bad, rounds = receive_photo(link, photo_id, args.n, args.gap, accepted + args.timeout,
                                              t0=accepted)
            for i in range(args.n):
                if i not in done:
                    failed.append({"idx": i, "error": bad.get(i, "not received")})
                    log(f"photo {photo_id} frame {i}: {failed[-1]['error']}")
                    continue
                meta = link.frame(photo_id, i).meta
                h = link.hello_ack
                extra = {"link": {"session": hex32(link.session), "bootNonce": hex32(h["bootNonce"]),
                                  "fw": h["fw"], "key_fp": link.fp, "rerequest_rounds": rounds[i],
                                  "received_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}}
                # without the daemon nothing flies: a ground photo's level line (the ground tilt)
                level = level_info(meta["pitch_deg"], meta["roll_deg"], "ground", load_level(),
                                   hex32(h["bootNonce"]), scale=meta["width"] // FRAME_W if meta["width"] >= 320 else X4_SCALE,
                                   draw=level_drawing_on() and not args.no_level)
                files = save_frame(out_dir, stamp, meta, done[i], extra, level=level)
                saved.append({"idx": i, **files, "color": bool(meta.get("color")), "frameSeq": meta["frameSeq"],
                              "captureUpMs": meta["captureUpMs"],
                              "roll_deg": meta["roll_deg"], "pitch_deg": meta["pitch_deg"],
                              "yaw_deg": meta["yaw_deg"], "thrust": meta["thrust"], "lum": meta["lum"],
                              "texture": meta["texture"], "vbat_v": meta["vbat_v"],
                              "crc32": f"{meta['crc32']:08x}", "rerequest_rounds": rounds[i],
                              "level_row_x4": level["row_x4_center"], "tilt_deg": level["tilt_deg_used"],
                              "tilt_kind": "ground", "level_stale": level["stale"]})
        finally:
            try:
                free = link.command(T_PHOTO_FREE, photo_free_payload(photo_id))["resultName"]
            except LinkError as e:
                log(f"PHOTO_FREE: {e}")
        ok = not failed
        out = {"ok": ok, "id": photo_id, "n": args.n, "gap_ms": args.gap, "dir": out_dir, "stamp": stamp,
               "frames": saved, "failed": failed, "free": free, "session": hex32(link.session),
               "bootNonce": hex32(link.hello_ack["bootNonce"]), "fw": link.hello_ack["fw"],
               "key_fp": link.fp, "elapsed_s": round(time.monotonic() - started, 2), "link": dict(link.stats)}
        if not ok:
            out["error"] = f"{len(failed)} of {args.n} frame(s) not received"
        return out, 0 if ok else 1


def keys_tty(fd):
    """The daemon's terminal for its keys: no Enter needed, no echo, and no flow control. Ctrl-S would
    otherwise stop the terminal's output, and a writer to it (the daemon's own, before LineWriter)
    would block the loop that sends the heartbeat and reads 'k'. Ctrl-C stays a signal (ISIG).
    Returns the function that restores the terminal."""
    import termios
    import tty
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    attrs = termios.tcgetattr(fd)
    attrs[0] &= ~(termios.IXON | termios.IXOFF | getattr(termios, "IXANY", 0))
    attrs[3] |= termios.ISIG
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    def restore():
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return restore


def cmd_daemon(args):
    global _log_sink
    key = load_key(args.key_file)
    link = open_link(args, key)
    keys_fd = restore = None
    if not args.no_tty and sys.stdin.isatty():
        keys_fd = sys.stdin.fileno()
        restore = keys_tty(keys_fd)
    out, err = LineWriter(sys.stdout), LineWriter(sys.stderr)
    _log_sink = err.write_line
    d = Daemon(link, args.sock, os.path.abspath(os.path.expanduser(args.photos)), allow_flight=args.allow_flight,
               keys_fd=keys_fd, events_path=args.events, out=out, host=args.host, no_level=args.no_level)

    def on_signal(signum, frame):
        if d.quit_signal is not None or d.quit_deadline is not None:
            d.stop_now = True                   # the second one: now
        else:
            d.quit_signal = signal.Signals(signum).name
    # SIGHUP (the terminal closed) is a quit like 'q': LAND first if active, then exit after the touchdown.
    # Started under nohup (SIGHUP already ignored), it stays ignored: the daemon outlives the session that
    # started it, as nohup asks; stop it with SIGTERM (kill <pid>), which is the same quit.
    hup_ignored = signal.getsignal(signal.SIGHUP) == signal.SIG_IGN
    for sig in (signal.SIGINT, signal.SIGTERM) + (() if hup_ignored else (signal.SIGHUP,)):
        signal.signal(sig, on_signal)
    try:
        h = d.start()
        log(f"daemon: session {h['session']:08x}, fw {h['fw']}, heartbeat every {HB_S * 1000:.0f} ms, socket "
            f"{args.sock}" + (", FLIGHT allowed" if args.allow_flight else ", FLIGHT refused (no --allow-flight)"))
        log("daemon: " + (KEY_HELP if keys_fd is not None else "no terminal keys (--no-tty or not a tty)"))
        log("daemon: " + ("SIGHUP ignored (started under nohup): a hangup does not stop it; 'kill <pid>' "
                          "(SIGTERM) lands first, then quits" if hup_ignored else
                          "closing its terminal (SIGHUP) lands first, then quits, like 'q'"))
        d.run()
    finally:
        _log_sink = None
        err.close()
        if restore:
            restore()
    return {"ok": True, "session": hex32(link.session), "hb_sent": d.hb_sent, "hb_late": d.hb_late,
            "events": d.events_base + len(d.events), "rebooted": d.rebooted, "session_lost": d.session_lost}, 0


def daemon_alive(info, sock_path):
    """Why the daemon record cannot be trusted, or None: its process must exist and its socket must still be
    there (a daemon that exits cleanly removes both the socket and the record; a SIGKILLed one leaves them)."""
    pid = info.get("pid")
    if not isinstance(pid, int) or pid <= 0:
        return "no pid in the record"
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return f"its daemon (pid {pid}) is gone"
    except PermissionError:
        pass                    # it exists (another user's process: the record is suspect, but alive)
    except OSError as e:
        return f"pid {pid}: {e}"
    if not os.path.exists(sock_path):
        return f"its socket {sock_path} is gone"
    return None


def kill_bindings(args):
    """Where the kill client sends its KILLs: [(Namespace(host, iface, port), why)].

    An explicit --host/--iface wins. Otherwise the binding the daemon recorded in <sock>.json, but only
    while that daemon is alive (daemon_alive) and, for an interface binding, while that interface still
    holds a 192.168.43.x address (link_check). A record that fails either is ignored: the KILLs go to
    the drone's known address (192.168.43.42) on the interface the normal lookup finds, with no HELLO.
    When the daemon's interface and the lookup's differ, both get the burst."""
    if args.host or args.iface:
        return [(args, "--host/--iface")]
    info = read_daemon_info(args.sock)
    if not info:
        return [(args, "no daemon record: the drone interface lookup")]
    problem = daemon_alive(info, args.sock)
    if problem:
        log(f"kill: ignoring the daemon record {info_path(args.sock)}: {problem}")
        return [(args, f"daemon record ignored ({problem}): the drone interface lookup")]
    rec = argparse.Namespace(host=info.get("host"), iface=info.get("iface"), port=info.get("port") or args.port)
    if rec.host or not rec.iface:
        return [(rec, "the live daemon's binding")]
    chk = link_check()
    if rec.iface in chk.get("candidates", []):
        if chk["link_up"] and chk["iface"] != rec.iface:
            return [(rec, "the live daemon's interface"), (args, f"the drone route ({chk['iface']})")]
        return [(rec, "the live daemon's interface")]
    log(f"kill: the daemon's interface {rec.iface} has no {SUBNET_PREFIX}x address now"
        + (f"; using {chk['iface']}" if chk["link_up"] else ""))
    if chk["link_up"]:
        return [(args, f"the daemon's interface {rec.iface} is off the drone network: the drone interface lookup")]
    return [(rec, "the live daemon's interface (no interface holds a drone-network address)")]


def cmd_kill(args):
    """KILL from a socket of its own, so it works with the daemon dead or hung, bound like the daemon's
    link (kill_bindings), session 0 and a random seq (the drone acts on a KILL in any session and answers
    it at its sender); then the daemon is asked too."""
    key = load_key(args.key_file)
    sent_total, acked_all, states, used = 0, set(), [], []
    for binding, why in kill_bindings(args):
        try:
            link = open_link(binding, key)
        except LinkError as e:          # a stale record (interface gone): find the drone's interface as usual
            log(f"kill: {why}: {e}")
            if binding is args:
                if used:
                    continue
                raise
            try:
                link = open_link(args, key)
            except LinkError as e2:
                log(f"kill: the drone interface lookup: {e2}")
                continue
            why = "the drone interface lookup (the daemon's binding failed)"
        with link:
            base = secrets.randbits(30) + 1
            seqs = [base + i for i in range(KILL_BURST_N)]
            sent = send_kill_burst(link, 0, seqs)
            sent_total += sent
            acked = {}
            deadline = time.monotonic() + KILL_ACK_WAIT_S
            while len(acked) < sent and time.monotonic() < deadline:
                pkt = link.poll(deadline - time.monotonic())
                if pkt is None or pkt.type != T_ACK:
                    continue
                try:
                    a = parse_ack(pkt.payload)
                except ProtoError:
                    continue
                if a["ackType"] == T_KILL and a["ackSeq"] in seqs:
                    acked[a["ackSeq"]] = a["stateName"]
            states += [acked[s] for s in seqs if s in acked]      # burst order
            acked_all |= set(acked)
            used.append({"iface": link.iface, "drone": link.addr[0], "sent": sent, "acked": len(acked), "why": why})
    if not used:
        raise LinkError("kill: no binding to the drone could be opened")
    # 'state': KILLED if any ACK of the burst shows it (the first usually still carries the state before the
    # KILL), else the last ACK's; a running daemon's AUTO_STATUS that counts the KILL replaces it.
    out = {"sent": sent_total, "acked": len(acked_all), "state": kill_burst_state(states), "ack_states": states,
           "iface": used[0]["iface"], "drone": used[0]["drone"], "session": 0, "bindings": used}
    try:
        r = daemon_request(args.sock, {"op": "kill"}, CLIENT_PROBE_S)
        out["daemon"] = {k: r.get(k) for k in ("ok", "sent", "acked", "state", "killCount", "error")}
        if r.get("killCount") is not None and r.get("state"):
            out["state"] = r["state"]
    except DaemonError as e:
        out["daemon"] = {"ok": False, "error": str(e)}
    out["ok"] = bool(acked_all) or bool(out["daemon"].get("acked"))
    if not out["ok"]:
        out["error"] = "no KILL was acknowledged (drone off, out of range, or a different key)"
    return out, 0 if out["ok"] else 1


def cmd_arm(args):
    if args.mode == "flight" and not args.confirm_flight:
        return {"ok": False, "result": "REFUSED", "error": "FLIGHT refused: --confirm-flight not given"}, 1
    return ask_daemon(args, {"op": "arm", "mode": args.mode, "air_ms": args.air_ms,
                             "confirm_flight": args.confirm_flight}, 5.0)


def cmd_takeoff(args):
    return ask_daemon(args, {"op": "takeoff", "height": args.height, "photos": args.photos, "gap": args.gap,
                             "wait_s": args.wait_s}, 5.0)


def cmd_sortie(args):
    if args.mode == "flight" and not args.confirm_flight:
        return {"ok": False, "result": "REFUSED", "error": "FLIGHT refused: --confirm-flight not given"}, 1
    if args.timeout is None:
        args.timeout = photo_set_timeout_s(args.photos)
    pr = (args.plan_right or 0.0) - (args.plan_left or 0.0)
    pf = (args.plan_forward or 0.0) - (args.plan_back or 0.0)
    req = {"op": "sortie", "mode": args.mode, "height": args.height, "photos": args.photos, "gap": args.gap,
           "then": args.then, "timeout": args.timeout, "air_ms": args.air_ms, "wait_s": args.wait_s,
           "confirm_flight": args.confirm_flight}
    extra = 0.0
    if pr or pf:
        req.update(plan_right=pr, plan_fwd=pf, plan_via=args.plan_via, person_gap=args.person_gap, est=args.est,
                   batt_ok=args.batt_ok)
        extra = Daemon.move_timeout(pr, pf, 0.0, None)
    return ask_daemon(args, req, args.timeout + extra + 10.0)


def cmd_move(args):
    right = (args.right or 0.0) - (args.left or 0.0)
    fwd = (args.forward or 0.0) - (args.back or 0.0)
    up = (args.up or 0.0) - (args.down or 0.0)
    if right == fwd == up == 0.0:
        return {"ok": False, "result": "REFUSED", "error": "give at least one of --right/--left/--forward/--back/"
                                                          "--up/--down (metres)"}, 1
    req = {"op": "move", "right": right, "fwd": fwd, "up": up, "wait": not args.no_wait, "batt_ok": args.batt_ok,
           "photo": args.photo, "full_wait": args.full_wait}
    if args.person_gap is not None:
        req["person_gap"] = args.person_gap
    if args.est is not None:
        req["est"] = args.est
    return ask_daemon(args, req, Daemon.move_timeout(right, fwd, up, None) + 30.0)


def cmd_wait_move(args):
    return ask_daemon(args, {"op": "wait-move", "photo": args.photo, "timeout": args.timeout}, args.timeout + 20.0)


def cmd_yaw(args):
    return ask_daemon(args, {"op": "yaw", "deg": args.deg, "wait": not args.no_wait, "photo": args.photo,
                             "full_wait": args.full_wait}, 90.0)


def cmd_set_height(args):
    """set-height X: "you are at X m" (docs/autonomy-a2b.md 23); the daemon's latest photo's estimate at capture is
    h_at unless --photo, --at or --now."""
    req = {"op": "set-height", "x_m": args.x_m, "wait": not args.no_wait}
    if args.to is not None:
        req["to_m"] = args.to
    if args.photo:
        req["photo"] = os.path.abspath(os.path.expanduser(args.photo))
    if args.at is not None:
        req["at_m"] = args.at
    if args.now:
        req["now"] = True
    if args.legacy:
        req["legacy"] = True
    return ask_daemon(args, req, SH_WAIT_S + 15.0)


def cmd_level_cal(args):
    """level-cal: through a running daemon (its setting, its event log), else here (the files only)."""
    req = {"op": "level-cal", "floor_row": args.floor_row, "dist": args.dist, "horizon_row": args.horizon_row,
           "set": args.set, "show": args.show, "no_level": args.no_level}
    if args.photo:
        req["photo"] = os.path.abspath(os.path.expanduser(args.photo))
    req = {k: v for k, v in req.items() if v not in (None, False)}
    req["op"] = "level-cal"
    if os.path.exists(args.sock):
        try:
            daemon_request(args.sock, {"op": "ping"}, CLIENT_PROBE_S)
        except DaemonError:
            pass
        else:
            return ask_daemon(args, req, 10.0)
    events = default_events() if args.events is None else args.events

    def event(name, **fields):
        try:
            append_jsonl(events, {"event": name, "t": round(time.time(), 3), "via": "level-cal", **fields})
        except OSError as e:
            log(f"level-cal: events file {events}: {e}")
    photo_dir = os.path.abspath(os.path.expanduser(args.photos or DEFAULT_PHOTO_DIR))
    try:
        out = level_cal_run(req, photo_dir, event)
    except ValueError as e:
        out = {"ok": False, "error": str(e)}
    return out, 0 if out.get("ok") else 1


def cmd_client(op):
    def fn(args):
        return ask_daemon(args, {"op": op}, 5.0)
    return fn


def cmd_wait(args):
    return ask_daemon(args, {"op": "wait", "until": args.until, "timeout": args.timeout}, args.timeout + 5.0)


def cmd_events(args):
    return ask_daemon(args, {"op": "events", "since": args.since}, 5.0)


# ---------------------------------------------------------------- main

class Parser(argparse.ArgumentParser):
    """Usage errors also come out as one JSON line."""
    cmd = None  # the subcommand word from argv; main() sets it before parsing

    def error(self, message):
        self.print_usage(sys.stderr)
        log(f"{self.prog}: error: {message}")
        print(json.dumps({"cmd": Parser.cmd, "ok": False, "error": message}), flush=True)
        sys.exit(2)


def _interrupted(signum, frame):
    raise KeyboardInterrupt


def int_range(lo, hi):
    def parse(s):
        v = int(s)
        if not lo <= v <= hi:
            raise argparse.ArgumentTypeError(f"must be {lo}..{hi}")
        return v
    return parse


def float_range(lo, hi):
    def parse(s):
        v = float(s)
        if not (math.isfinite(v) and lo <= v <= hi):
            raise argparse.ArgumentTypeError(f"must be {lo:g}..{hi:g}")
        return v
    return parse


def wait_s_arg(s):
    v = float(s)
    if not (v == 0 or ACT_WAIT_MS[0] / 1000 <= v <= ACT_WAIT_MS[1] / 1000):
        raise argparse.ArgumentTypeError(f"must be 0 or {ACT_WAIT_MS[0] // 1000}..{ACT_WAIT_MS[1] // 1000}")
    return v


def personopt(p):
    p.add_argument("--person-gap", type=float_range(0.0, 20.0), metavar="M",
                   help=f"your estimate of the move's end distance to the nearest person, m: refused under "
                        f"{PERSON_MIN_M:.1f} (the user rule), or when the drone's travel at an image expansion scale "
                        f"k down to {K_LOW:g} could take it there (the live D; needs --est); logged with --est")
    p.add_argument("--est", type=float_range(0.1, 20.0), metavar="M",
                   help="your estimate of the distance to the person (or object) now, from the photo (logged; "
                        "needed with --person-gap)")
    p.add_argument("--batt-ok", action="store_true",
                   help="no battery-duration refusal (under 3.25 V a move that may not end before 3.00 V: a "
                        "BATT_LOW partway lands in place)")


def build_parser():
    ap = Parser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    keyopt = Parser(add_help=False)
    keyopt.add_argument("--key-file", default=DEFAULT_KEY_FILE, help=f"(default {DEFAULT_KEY_FILE})")
    net = Parser(add_help=False)
    net.add_argument("--host", help=f"drone address (default {DRONE_IP} on the interface holding a "
                                    f"{SUBNET_PREFIX}x address; giving --host skips the interface lookup, "
                                    "and binds only with --iface)")
    net.add_argument("--port", type=int, default=PORT, help=f"UDP port (default {PORT})")
    net.add_argument("--iface", help="send on this interface instead of finding the drone's")
    sockopt = Parser(add_help=False)
    sockopt.add_argument("--sock", default=default_sock(), help=f"daemon socket (default {default_sock()})")
    oneshot = Parser(add_help=False)
    oneshot.add_argument("--force-hello", action="store_true",
                         help="open a session of our own even though a daemon socket exists and does not answer "
                              "(lands an active sortie)")
    flightopt = Parser(add_help=False)
    flightopt.add_argument("--height", type=int_range(*HEIGHT_CM), default=40,
                           help=f"take-off height, cm ({HEIGHT_CM[0]}..{HEIGHT_CM[1]}; 40 flies ~0.32 m; higher, up to "
                                f"{MOVE_H_RANGE_M[1]:.2f} m, by 'move --up' from the hover)")
    flightopt.add_argument("--photos", type=int_range(0, MAX_N), default=1,
                           help="photos at STABLE + 1 s (0..3, default 1: more in the wait with 'photo')")
    flightopt.add_argument("--gap", type=int_range(*PHOTO_GAP_AIR_MS), default=300, help="ms between photos")
    flightopt.add_argument("--wait-s", type=wait_s_arg, default=0,
                           help="this sortie's hover wait, s: information only since 2026-10-15 (the drone has no "
                                "wait timeout and never lands for it); 0 = the drone's autowait (NVS), else "
                                f"{ACT_WAIT_MS[0] // 1000}..{ACT_WAIT_MS[1] // 1000}. Advice only (the take-off's "
                                "'note'): above 12 s --height >= 0.25 m + 0.01 m/s x min(wait + 8 s, 80 s), 105 cm "
                                "from 72 s; 0 checks the drone's autowait as the last 'auto get' read it. The height "
                                "itself: photo checks and set-height (docs/autonomy-a2b.md 23)")
    modeopt = Parser(add_help=False)
    modeopt.add_argument("--mode", required=True, choices=sorted(MODES),
                         help="dry: no thrust, the sim flies; bench: real thrust <= min(gcap, 15000), below lift-off; "
                              "flight: real (needs daemon --allow-flight and --confirm-flight)")
    modeopt.add_argument("--air-ms", type=int, default=0,
                         help=f"ARM airMaxMs: 0 = the drone's NVS autoair, else {AIR_MS[0]}..autoair (may only lower "
                              f"it; checked against the last 'espfly_diag.py auto get', {AIR_MS_DEFAULT} without one)")
    modeopt.add_argument("--confirm-flight", action="store_true", help="required with --mode flight")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("keygen", parents=[keyopt], help="make the link key (16 random bytes, 0600)")
    p.add_argument("--force", action="store_true", help="replace an existing key")
    p.set_defaults(fn=cmd_keygen)
    p = sub.add_parser("install-key", parents=[keyopt], help="send the key to the drone over USB ('autokey')")
    p.add_argument("--port", dest="serial_port", help="serial port (default: first /dev/cu.usbmodem*)")
    p.set_defaults(fn=cmd_install_key)
    p = sub.add_parser("up", help="is the drone AP joined, routed, and the default route elsewhere?")
    p.set_defaults(fn=cmd_up)
    p = sub.add_parser("hello", parents=[keyopt, net, sockopt, oneshot],
                       help="start a session; check the key fingerprint (through a running daemon: its session)")
    p.set_defaults(fn=cmd_hello)
    p = sub.add_parser("status", parents=[keyopt, net, sockopt, oneshot],
                       help="HELLO + STATUS_REQ, decoded, with AUTO_STATUS (through a running daemon: no HELLO)")
    p.set_defaults(fn=cmd_status)
    p = sub.add_parser("photo", parents=[keyopt, net, sockopt, oneshot],
                       help="1..3 camera frames as PNG + JSON (in the hover wait: through the daemon)")
    p.add_argument("--n", type=int_range(1, MAX_N), default=1, help="frames (1..3)")
    p.add_argument("--gap", type=int_range(0, 1000), default=250,
                   help="ms between frames (0..1000; 150..1000 in the hover wait)")
    p.add_argument("--out", help=f"(default {DEFAULT_PHOTO_DIR}, or the daemon's --photos)")
    p.add_argument("--timeout", type=float, default=None, help="seconds for the whole set after PHOTO is "
                         "accepted (default 20/30/40 for 1/2/3 VGA photos)")
    p.add_argument("--no-level", action="store_true", help=f"no level line on the _x4.png (or {LEVEL_ENV}=0; the JSON "
                                                           "keeps its level block)")
    p.set_defaults(fn=cmd_photo)
    p = sub.add_parser("daemon", parents=[keyopt, net, sockopt],
                       help="hold the session: heartbeat, AUTO_STATUS, photos, client socket; keys k/l/s/q")
    p.add_argument("--allow-flight", action="store_true", help="let 'arm --mode flight --confirm-flight' through")
    p.add_argument("--photos", default=DEFAULT_PHOTO_DIR, help=f"where frames go (default {DEFAULT_PHOTO_DIR})")
    p.add_argument("--events", default=default_events(), help=f"JSON lines log (default {default_events()})")
    p.add_argument("--no-tty", action="store_true", help="no terminal keys (for nohup)")
    p.add_argument("--no-level", action="store_true", help=f"no level line on the photos (or {LEVEL_ENV}=0; their "
                                                           "JSON keeps the level block)")
    p.set_defaults(fn=cmd_daemon)
    p = sub.add_parser("arm", parents=[sockopt, modeopt], help="ARM (prints the token, or the failed guards)")
    p.set_defaults(fn=cmd_arm)
    p = sub.add_parser("takeoff", parents=[sockopt, flightopt], help="TAKEOFF with the daemon's ARM token")
    p.set_defaults(fn=cmd_takeoff)
    p = sub.add_parser("sortie", parents=[sockopt, modeopt, flightopt],
                       help="ARM + TAKEOFF; returns in the hover wait with the photos on disk (or at the sortie's end)")
    p.add_argument("--then", choices=("wait", "land"), default="wait", help="land: LAND once waiting, return at IDLE")
    p.add_argument("--timeout", type=float, default=None,
                   help="seconds (default 20/30/40 for 1/2/3 VGA photos; a planned move adds its own)")
    horiz, vert = float_range(0.0, MOVE_HORIZ_MAX_M), float_range(0.0, MOVE_UP_MAX_M)
    fwdr = float_range(0.0, MOVE_FWD_MAX_M)
    for name, t, top in (("--plan-right", horiz, MOVE_HORIZ_MAX_M), ("--plan-left", horiz, MOVE_HORIZ_MAX_M),
                         ("--plan-forward", fwdr, MOVE_FWD_MAX_M), ("--plan-back", horiz, MOVE_HORIZ_MAX_M)):
        p.add_argument(name, type=t, metavar="M", help=f"a move from take-off, {name[7:]}, m (0..{top:.2f}; the "
                                                       "vector <= 4.00): flown from the hover wait's start, in the "
                                                       "take-off heading's frame (plan it from the floor photo)")
    p.add_argument("--plan-via", choices=("drone", "mac"), default=PLAN_VIA_DEFAULT,
                   help="drone: the 18-byte TAKEOFF (the long-move build); mac: this daemon sends the MOVE at the "
                        f"wait's start through 'move''s checks (default {PLAN_VIA_DEFAULT})")
    personopt(p)
    p.set_defaults(fn=cmd_sortie)
    slim, sname = speed_help()
    p = sub.add_parser("move", parents=[sockopt],
                       help="in the hover wait: MOVE the drone (metres, in its own frame now); returns when the "
                            f"move has ended (above mvspeed 100: when its profile has ended, the settle running). At "
                            f"{sname} (the drone's NVS mvspeed as the last 'auto get' read it; the reply says the "
                            f"drone's) up to {slim['v_h']:.2f} m/s ({slim['v_fwd']:.2f} toward the scene ahead, "
                            f"{slim['v_back']:.2f} back), {slim['a_h']:.2f} m/s^2, slower near the scene "
                            f"({slim['w_lat']:g} x D sideways, {slim['w_fwd']:g} x the distance forward), a settle of "
                            f"at most {slim['settle_ms'] / 1000.0:g} s. Forward <= {MOVE_FWD_MAX_M:.2f} m, sideways and "
                            "back <= "
                            f"{MOVE_HORIZ_MAX_M:.2f} m, the vector <= {MOVE_LEN_MAX_M:.2f} m; {move_guards_help()}; "
                            f"Claude judges a person's distance: --person-gap (>= {PERSON_MIN_M:.1f} m)")
    for name, t, top in (("--right", horiz, MOVE_HORIZ_MAX_M), ("--left", horiz, MOVE_HORIZ_MAX_M),
                         ("--forward", fwdr, MOVE_FWD_MAX_M), ("--back", horiz, MOVE_HORIZ_MAX_M),
                         ("--up", vert, MOVE_UP_MAX_M), ("--down", vert, MOVE_UP_MAX_M)):
        p.add_argument(name, type=t, metavar="M", help=f"{name[2:]}, m (0..{top:.2f})")
    p.add_argument("--no-wait", action="store_true", help="return once the drone accepted it")
    p.add_argument("--photo", action="store_true", help="take a photo when it replies (in the same reply)")
    p.add_argument("--full-wait", action="store_true", help="above mvspeed 100: reply at the move's end (its settle "
                                                            "done), not when its profile ends")
    personopt(p)
    p.set_defaults(fn=cmd_move)
    p = sub.add_parser("wait-move", parents=[sockopt],
                       help="block until the running motion ends (at once if none); its end, travel, target, D, the "
                            "margins; --photo: a photo then")
    p.add_argument("--photo", action="store_true", help="take a photo once it has ended (in the reply)")
    p.add_argument("--timeout", type=float, default=60.0, help="seconds (default 60)")
    p.set_defaults(fn=cmd_wait_move)
    p = sub.add_parser("yaw", parents=[sockopt], help="in the hover wait: turn by --deg (+ right / clockwise, as "
                                                      f"the photo bearings) at {slim['yaw_rate']:.0f} deg/s ({sname}; "
                                                      f"180 deg: {yaw_profile_s(180, slim['pct']):.1f} s, then a "
                                                      f"{slim['dwell_ms']} ms dwell and a re-track bob of up to "
                                                      f"{BOB_CYCLES * bob_cycle_s(slim['pct']):.1f} s); returns when "
                                                      "the turn has ended (above mvspeed 100: when its bob starts)")
    p.add_argument("--deg", type=int_range(-YAW_MAX_DEG, YAW_MAX_DEG), required=True)
    p.add_argument("--no-wait", action="store_true", help="return once the drone accepted it")
    p.add_argument("--photo", action="store_true", help="take a photo when it replies (in the same reply)")
    p.add_argument("--full-wait", action="store_true", help="above mvspeed 100: reply at the turn's end (its re-track "
                                                            "bob done)")
    p.set_defaults(fn=cmd_yaw)
    p = sub.add_parser("stop", parents=[sockopt], help="STOP a running MOVE or YAW: the drone holds where it is (a "
                                                       "re-track bob after a turn or a long sideways move runs on, "
                                                       "<= ~8 s)")
    p.set_defaults(fn=cmd_client("stop"))
    p = sub.add_parser("set-height", parents=[sockopt],
                       help="in the hover wait: 'you are at X m' (docs/autonomy-a2b.md 23, 27): X judged from a "
                            "photo's level line; the drone takes X as its true height and flies to the commanded "
                            "height (or --to H) at 0.15 m/s (|X - h_at| <= 0.50 m, the flight <= 0.55 m: a longer one "
                            "is clipped to 0.50 m and the rest named; one per 2 s). By default X is the height when the "
                            "latest photo was captured (moves since then cancel out; the estimate's drift since "
                            "does not: act within ~10 s). The reply: true_now_m, target_m, fly_m")
    p.add_argument("x_m", type=float_range(*SH_X_M), metavar="X", help=f"the judged true height, m "
                                                                       f"({SH_X_M[0]:g}..{SH_X_M[1]:g})")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--photo", help="the judged photo (its .json / .png / _x4.png): h_at its estimate at capture "
                                   "(default: the latest photo of this sortie)")
    g.add_argument("--at", type=float_range(*SH_AT_M), metavar="M", help="h_at given: the estimate the judgement is "
                                                                         "against, m")
    g.add_argument("--now", action="store_true", help="against the estimate now (no photo: moves since the judgement "
                                                      "count as error)")
    p.add_argument("--to", type=float_range(*SH_TO_M), metavar="H",
                   help=f"fly to this height, m ({SH_TO_M[0]:g}..{SH_TO_M[1]:g}; it becomes the commanded height), "
                        "not to the commanded one (default: the commanded height, or the target a clipped "
                        "set-height left to fly, until reached)")
    p.add_argument("--legacy", action="store_true",
                   help="the 4-byte form as before 2026-10-16 (it flies back to the old estimate; no target, no clip): "
                        "the bench's A/B only (bench R)")
    p.add_argument("--no-wait", action="store_true", help="return once the drone accepted it")
    p.set_defaults(fn=cmd_set_height)
    p = sub.add_parser("level-cal", parents=[sockopt],
                       help="the camera mount's tilt for the level line, ONE setting (never per room), re-measured "
                            "every flight from the ground photo (and after any crash, contact, catch or battery swap)")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--floor-row", type=float, metavar="R", help="x4 row of a far floor-wall junction 2-6 m away near "
                                                                "the centre column, on a ground photo")
    g.add_argument("--horizon-row", type=float, metavar="R", help="x4 row of a true horizon cue (the vanishing point "
                                                                  "of horizontal room lines) at the centre column, on a "
                                                                  "hover photo")
    g.add_argument("--set", type=float_range(-30.0, 30.0), metavar="DEG", help="the flight tilt by hand (ground: DEG "
                                                                               f"- {LEVEL_FLIGHT_ADJ_DEG:g})")
    g.add_argument("--show", action="store_true", help="the setting as it is")
    p.add_argument("--dist", type=float_range(0.5, 20.0), metavar="D",
                   help=f"--floor-row: the floor-wall line's distance, m (default {LEVEL_FLOOR_DIST_M:g})")
    p.add_argument("--photo", help="the photo (default: the latest ground photo; --horizon-row the latest flight one)")
    p.add_argument("--photos", help=f"without a daemon: the photo directory (default {DEFAULT_PHOTO_DIR})")
    p.add_argument("--events", help="without a daemon: the events log (default the daemon's)")
    p.add_argument("--no-level", action="store_true", help="do not draw the line again")
    p.set_defaults(fn=cmd_level_cal)
    p = sub.add_parser("land", parents=[sockopt], help="LAND (never refused; slow, in place)")
    p.set_defaults(fn=cmd_client("land"))
    p = sub.add_parser("disarm", parents=[sockopt], help="DISARM (ARMED: back to IDLE; in the air: LAND)")
    p.set_defaults(fn=cmd_client("disarm"))
    p = sub.add_parser("kill", parents=[keyopt, net, sockopt],
                       help="KILL: motors off at once (own socket: works without the daemon), then the daemon too")
    p.set_defaults(fn=cmd_kill)
    p = sub.add_parser("state", parents=[sockopt], help="latest STATUS and AUTO_STATUS, decoded, and the daemon's own")
    p.set_defaults(fn=cmd_client("state"))
    p = sub.add_parser("wait", parents=[sockopt], help="return when the drone is waiting / idle / landing, the "
                                                       "photos are in, or it reports an event or changes state")
    p.add_argument("--until", required=True, choices=("waiting", "idle", "photos", "land", "event"))
    p.add_argument("--timeout", type=float, default=30.0)
    p.set_defaults(fn=cmd_wait)
    p = sub.add_parser("events", parents=[sockopt], help="the daemon's events from number --since on")
    p.add_argument("--since", type=int, default=0)
    p.set_defaults(fn=cmd_events)
    return ap


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    # The top-level parser has only -h, so its first non-option word is the subcommand.
    Parser.cmd = next((a for a in argv if not a.startswith("-")), None)
    args = build_parser().parse_args(argv)
    # A killed run still reaches the finally blocks (PHOTO_FREE). A signal already ignored stays ignored
    # (nohup: SIGHUP; the daemon reads it again, cmd_daemon).
    for sig in (signal.SIGTERM, signal.SIGHUP):
        if signal.getsignal(sig) != signal.SIG_IGN:
            signal.signal(sig, _interrupted)
    try:
        out, code = args.fn(args)
    except LinkError as e:
        out, code = {"ok": False, "error": str(e), **e.info}, 1
    except KeyboardInterrupt:
        out, code = {"ok": False, "error": "interrupted"}, 130
    except Exception as e:
        out, code = {"ok": False, "error": f"{type(e).__name__}: {e}"}, 1
    if not out.get("ok", False) and "error" in out:
        log(f"{args.cmd}: {out['error']}")
    print(json.dumps({"cmd": args.cmd, **out}), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
