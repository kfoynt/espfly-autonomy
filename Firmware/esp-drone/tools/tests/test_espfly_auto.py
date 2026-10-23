"""Host tests for tools/espfly_auto.py (autonomy link, Mac side; docs/autonomy-link.md).

Run from Firmware/esp-drone:  python3 tools/tests/test_espfly_auto.py
Only 127.0.0.1 is used (in-process fake drones, stage A1 and A2b); no serial
port, no real network, nothing under ~/.espfly. The A2b daemon runs in a thread
(or, for SIGKILL/SIGSTOP, as its own process) with its socket in a temp dir.
"""
import collections
import contextlib
import io
import json
import math
import os
import random
import re
import select
import shutil
import signal
import socket
import stat
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
import zlib
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import espfly_auto as auto  # noqa: E402

GOLDEN_PATH = os.path.join(HERE, "auto_proto_golden.json")
with open(GOLDEN_PATH) as _f:
    GOLDEN = json.load(_f)
KEY = bytes.fromhex(GOLDEN["key"])
SESSION = 0xA1B2C3D4


def golden_frame():
    return bytes((x * 7 + y * 3) & 0xFF for y in range(120) for x in range(160))


def other_frame(salt):
    return bytes((x * 3 + y * 5 + salt * 41) & 0xFF for y in range(120) for x in range(160))


def colour_frame(salt=0):
    """A YUYV frame (gen_auto_proto_golden.py's colour rule for salt 0): Y the grey golden frame's, U (even x) 64 + y,
    V (odd x) 192 - y, both moved by salt."""
    return bytes(b for y in range(120) for x in range(160)
                 for b in ((x * 7 + y * 3) & 0xFF, ((64 + y) if x % 2 == 0 else (192 - y)) ^ salt))


def meta_for(frame, photo_id=5, idx=0, n=2, **over):
    m = {"id": photo_id, "idx": idx, "n": n, "width": 160, "height": 120, "size": len(frame),
         "crc32": zlib.crc32(frame), "frameSeq": 4321, "captureUpMs": 98765, "roll_cdeg": -150,
         "pitch_cdeg": 220, "yaw_cdeg": 9000, "thrust": 0, "lum": 97, "texture": 31, "chunkSize": 1024,
         "chunkCount": auto.chunk_count(len(frame)), "vbatMv": 3870}
    m.update(over)
    return m


class SandboxedTest(unittest.TestCase):
    """Default paths point into a temporary directory and local commands
    (ifconfig/route) are refused unless a test supplies them."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        home = os.path.join(self.tmp.name, "home", ".espfly")
        for name, val in (("DEFAULT_DIR", home), ("DEFAULT_KEY_FILE", os.path.join(home, "auto.key")),
                          ("DEFAULT_PHOTO_DIR", os.path.join(home, "photos"))):
            p = mock.patch.object(auto, name, val)
            p.start()
            self.addCleanup(p.stop)
        p = mock.patch.object(auto, "run_cmd", side_effect=AssertionError("local command run in a test"))
        p.start()
        self.addCleanup(p.stop)

    def run_main(self, argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                code = auto.main(argv)
            except SystemExit as e:
                code = e.code
        lines = out.getvalue().splitlines()
        self.assertEqual(len(lines), 1, f"want exactly one JSON line, got {out.getvalue()!r}")
        return code, json.loads(lines[0]), out.getvalue(), err.getvalue()

    def key_file(self, key=KEY):
        path = os.path.join(self.tmp.name, "auto.key")
        auto.write_key(path, key)
        return path


# ---------------------------------------------------------------- codec

class GoldenTest(unittest.TestCase):
    def test_key_fingerprint(self):
        self.assertEqual(auto.key_fp(KEY), GOLDEN["key_fp"])

    def test_frame_crc(self):
        self.assertEqual(zlib.crc32(golden_frame()), GOLDEN["frame_crc32"])

    def test_packets_from_fields(self):
        frame = golden_frame()
        fp = bytes.fromhex(GOLDEN["key_fp"])
        built = {
            "hello": auto.encode(KEY, auto.T_HELLO, 0, 0, 1, auto.hello_payload(0x11223344)),
            "hello_ack": auto.encode(KEY, auto.T_HELLO_ACK, 0, SESSION, 1,
                                     auto.hello_ack_payload(SESSION, 0x0BADF00D, 0x11223344, fp, "f7e67fb")),
            "photo": auto.encode(KEY, auto.T_PHOTO, auto.FLAG_ACKREQ, SESSION, 2, auto.photo_payload(2, 300)),
            "ack": auto.encode(KEY, auto.T_ACK, 0, SESSION, 7, auto.ack_payload(2, auto.T_PHOTO, auto.R_OK, 0, 5)),
            "meta": auto.encode(KEY, auto.T_META, 0, SESSION, 8, auto.meta_payload(meta_for(frame))),
            "chunk": auto.encode(KEY, auto.T_CHUNK, 0, SESSION, 9, auto.chunk_payload(5, 0, 0, frame[:1024])),
            "chunks": auto.encode(KEY, auto.T_CHUNKS, auto.FLAG_ACKREQ | auto.FLAG_RETX, SESSION, 3,
                                  auto.chunks_payload(5, 0, 4, 12)),
            "hb": auto.encode(KEY, auto.T_HB, 0, SESSION, 4, auto.hb_payload(123456)),
        }
        self.assertEqual(set(built), set(GOLDEN["packets"]))
        for name, pkt in built.items():
            with self.subTest(name=name):
                self.assertEqual(pkt.hex(), GOLDEN["packets"][name])

    def test_decode_and_reencode(self):
        for name, hx in GOLDEN["packets"].items():
            with self.subTest(name=name):
                buf = bytes.fromhex(hx)
                p = auto.decode(KEY, buf)
                self.assertEqual(auto.encode(KEY, p.type, p.flags, p.session, p.seq, p.payload), buf)

    def test_parsed_fields(self):
        g = {k: auto.decode(KEY, bytes.fromhex(v)) for k, v in GOLDEN["packets"].items()}
        h = auto.parse_hello_ack(g["hello_ack"].payload)
        self.assertEqual((h["session"], h["bootNonce"], h["echoNonce"], h["keyFp"], h["fw"]),
                         (SESSION, 0x0BADF00D, 0x11223344, GOLDEN["key_fp"], "f7e67fb"))
        self.assertEqual(g["hello_ack"].session, SESSION)
        a = auto.parse_ack(g["ack"].payload)
        self.assertEqual((a["ackSeq"], a["ackType"], a["resultName"], a["detail"]), (2, auto.T_PHOTO, "OK", 5))
        m = auto.parse_meta(g["meta"].payload)
        self.assertEqual(m["roll_cdeg"], -150)
        self.assertAlmostEqual(m["roll_deg"], -1.5)
        self.assertEqual((m["width"], m["height"], m["size"], m["chunkCount"], m["crc32"]),
                         (160, 120, 19200, 19, GOLDEN["frame_crc32"]))
        pid, idx, ci, data = auto.parse_chunk(g["chunk"].payload)
        self.assertEqual((pid, idx, ci, data), (5, 0, 0, golden_frame()[:1024]))
        p = g["photo"]
        self.assertEqual((p.type, p.flags, p.session, p.seq), (auto.T_PHOTO, auto.FLAG_ACKREQ, SESSION, 2))
        self.assertEqual(struct.unpack("<BBH", p.payload), (2, 0, 300))
        self.assertEqual(struct.unpack("<HBBH", g["chunks"].payload), (5, 0, 4, 12))

    def test_status_layout(self):
        fields = {"upMs": 123456, "vbatMv": 3870, "camStateCode": 2, "fps_x10": 158, "thrust": 1000,
                  "frameSeq": 77, "roll_cdeg": -150, "pitch_cdeg": 220, "yaw_cdeg": -9000, "lum": 97,
                  "texture": 255, "flags": 0b1101, "photoSlotsFree": 1, "authFails": 3, "heapFree": 100000,
                  "heapMin": 90000, "packetsAccepted": 42, "bootNonce": 0x0BADF00D}
        p = auto.status_payload(fields)
        self.assertEqual(len(p), 48)
        self.assertEqual(struct.unpack_from("<I", p, 44)[0], 0x0BADF00D)
        self.assertEqual(struct.unpack_from("<h", p, 20)[0], -9000)
        self.assertEqual(p[24], 0b1101)
        s = auto.parse_status(p)
        self.assertEqual(s["camState"], "running")
        self.assertEqual((s["motorsIdle"], s["takeoffWindow"], s["onGround"], s["photoBusy"]),
                         (True, False, True, True))
        self.assertAlmostEqual(s["fps"], 15.8)
        self.assertAlmostEqual(s["yaw_deg"], -90.0)
        self.assertNotIn("reserved", s)


class RejectTest(unittest.TestCase):
    def reason(self, buf, key=KEY):
        with self.assertRaises(auto.ProtoError) as cm:
            auto.decode(key, buf)
        return cm.exception.reason

    def test_tampered_wrong_key_truncated(self):
        other = bytearray(KEY)
        other[0] ^= 1
        for name, hx in GOLDEN["packets"].items():
            with self.subTest(name=name):
                g = bytes.fromhex(hx)
                for pos in (2, 3, 4, 8, 14, 15, len(g) - 17, len(g) - 1):
                    bad = bytearray(g)
                    bad[pos] ^= 0x01
                    self.assertEqual(self.reason(bytes(bad)), "tag", f"byte {pos}")
                self.assertEqual(self.reason(g, bytes(other)), "tag")
                self.assertEqual(self.reason(g[:-1]), "length")
                self.assertEqual(self.reason(g + b"\0"), "length")
                self.assertEqual(self.reason(g[:31]), "short")
                self.assertEqual(self.reason(b"\xe4" + g[1:]), "magic")
                self.assertEqual(self.reason(g[:1] + b"\x02" + g[2:]), "version")

    def test_length_limits(self):
        hdr = struct.pack(auto.HDR_FMT, auto.MAGIC, auto.VERSION, auto.T_CHUNK, 0, 1, 1, 1101, 0)
        self.assertEqual(self.reason(hdr + bytes(1101) + bytes(16)), "length")
        self.assertEqual(len(auto.encode(KEY, auto.T_CHUNK, 0, 1, 1, bytes(1100))), 1132)
        with self.assertRaises(ValueError):
            auto.encode(KEY, auto.T_CHUNK, 0, 1, 1, bytes(1101))

    def test_payload_sizes(self):
        for fn, size in ((auto.parse_status, 48), (auto.parse_meta, 40), (auto.parse_ack, 12),
                         (auto.parse_hello_ack, 40)):
            with self.subTest(fn=fn.__name__):
                with self.assertRaises(auto.ProtoError):
                    fn(bytes(size - 1))
        with self.assertRaises(auto.ProtoError):
            auto.parse_chunk(struct.pack(auto.CHUNK_HDR_FMT, 5, 0, 0, 0, 10) + bytes(9))
        with self.assertRaises(auto.ProtoError):
            auto.parse_chunk(struct.pack(auto.CHUNK_HDR_FMT, 5, 0, 0, 0, 1025) + bytes(1025))


# ---------------------------------------------------------------- reassembly

def chunks_of(frame, size=1024):
    return [frame[i:i + size] for i in range(0, len(frame), size)]


class ReassemblyTest(unittest.TestCase):
    def test_missing_and_out_of_order(self):
        frame = golden_frame()
        parts = chunks_of(frame)
        f = auto.Frame()
        f.add_meta(meta_for(frame))
        order = [i for i in range(19) if i not in (3, 7, 18)]
        random.Random(1).shuffle(order)
        for i in order:
            self.assertTrue(f.add_chunk(i, parts[i]))
        self.assertEqual(f.missing(), [3, 7, 18])
        self.assertFalse(f.complete())
        self.assertEqual(auto.missing_runs(f.missing()), [(3, 5), (18, 1)])
        self.assertFalse(f.add_chunk(order[0], parts[order[0]]))
        self.assertEqual(f.repeats, 1)
        self.assertFalse(f.add_chunk(19, b"x"))        # beyond chunkCount
        for i in (18, 3, 7):
            f.add_chunk(i, parts[i])
        self.assertTrue(f.complete())
        self.assertEqual(f.assemble(), frame)

    def test_chunks_before_meta(self):
        frame = other_frame(1)
        parts = chunks_of(frame)
        f = auto.Frame()
        for i in reversed(range(19)):
            f.add_chunk(i, parts[i])
        self.assertEqual(f.missing(), [])
        self.assertFalse(f.complete())
        with self.assertRaises(auto.FrameError):
            f.assemble()
        f.add_meta(meta_for(frame))
        self.assertTrue(f.complete())
        self.assertEqual(f.assemble(), frame)

    def test_crc_and_length_checked(self):
        frame = golden_frame()
        parts = chunks_of(frame)
        f = auto.Frame()
        f.add_meta(meta_for(frame, crc32=zlib.crc32(frame) ^ 1))
        for i, p in enumerate(parts):
            f.add_chunk(i, p)
        with self.assertRaisesRegex(auto.FrameError, "CRC"):
            f.assemble()
        f = auto.Frame()
        f.add_meta(meta_for(frame))
        for i, p in enumerate(parts):
            f.add_chunk(i, p[:-1] if i == 18 else p)
        with self.assertRaisesRegex(auto.FrameError, "chunk 18"):
            f.assemble()
        f = auto.Frame()
        f.add_meta(meta_for(frame, chunkCount=18))
        with self.assertRaisesRegex(auto.FrameError, "inconsistent"):
            f.assemble()

    def test_missing_runs(self):
        self.assertEqual(auto.missing_runs([]), [])
        self.assertEqual(auto.missing_runs(range(19)), [(0, 8), (8, 8), (16, 3)])
        self.assertEqual(auto.missing_runs([0, 7]), [(0, 8)])
        self.assertEqual(auto.missing_runs([8, 0]), [(0, 1), (8, 1)])
        for first, count in auto.missing_runs([1, 2, 5, 9, 10, 11, 12, 13, 14, 15, 16, 17]):
            self.assertTrue(1 <= count <= 8)


# ---------------------------------------------------------------- images and bearings

class BearingTest(unittest.TestCase):
    def test_positions(self):
        self.assertEqual(auto.bearing_x(0), 80.0)
        self.assertAlmostEqual(auto.bearing_x(30), 80 + 102.4 * math.tan(math.radians(30)))
        self.assertAlmostEqual(auto.bearing_x(30), 139.12, places=2)
        self.assertAlmostEqual(auto.bearing_x(-30), 20.88, places=2)
        self.assertAlmostEqual(auto.bearing_x(10), 98.06, places=2)
        for d in auto.TICK_DEG:
            self.assertAlmostEqual(auto.bearing_deg(auto.bearing_x(d)), d)
            self.assertAlmostEqual(auto.bearing_x(-d) - 80, 80 - auto.bearing_x(d))
        self.assertEqual(auto.TICK_DEG, (-30, -20, -10, 0, 10, 20, 30))
        self.assertEqual(auto.x4_of(80), 321.5)
        self.assertEqual(auto.tick_left(80), 321)       # columns 321-322, centred on 321.5
        info = auto.bearings_info()
        self.assertEqual([t["deg"] for t in info["ticks"]], list(auto.TICK_DEG))
        self.assertEqual(info["ticks"][3]["x4"], 321.5)
        self.assertAlmostEqual(info["edge_deg"][0], -38.0, places=1)


class ImageTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_png_outputs(self):
        from PIL import Image
        frame = golden_frame()
        meta = auto.parse_meta(auto.meta_payload(meta_for(frame)))
        files = auto.save_frame(self.tmp.name, "20261011-120000", meta, frame, {"link": {"session": "x"}})
        self.assertEqual(os.path.basename(files["png"]), "20261011-120000_5_0.png")
        self.assertEqual(os.path.basename(files["png_x4"]), "20261011-120000_5_0_x4.png")
        self.assertEqual(os.path.basename(files["json"]), "20261011-120000_5_0.json")
        with Image.open(files["png"]) as im:
            self.assertEqual((im.size, im.mode), ((160, 120), "L"))
            self.assertEqual(im.tobytes(), auto.flip_rows(frame, 160, 120))   # the camera is upside down
            self.assertEqual(im.getpixel((0, 0)), frame[119 * 160])
            self.assertEqual(im.getpixel((159, 119)), frame[159])
            self.assertEqual(im.getpixel((37, 100)), frame[(119 - 100) * 160 + 37])
        with Image.open(files["png_x4"]) as im:
            self.assertEqual((im.size, im.mode), ((640, 480), "RGB"))
            px = im.load()
            for x, y in ((0, 40), (79, 60), (80, 60), (159, 80), (37, 100)):
                v = frame[(119 - y) * 160 + x]
                for dx, dy in ((0, 0), (3, 3), (1, 2)):
                    self.assertEqual(px[4 * x + dx, 4 * y + dy], (v, v, v))
            for d in auto.TICK_DEG:
                left = auto.tick_left(auto.bearing_x(d))
                color = auto.TICK_CENTER_RGB if d == 0 else auto.TICK_RGB
                for y in (0, auto.TICK_LEN - 1, 479 - auto.TICK_LEN + 1, 479):
                    self.assertEqual(px[left, y], color, f"tick {d} at row {y}")
                    self.assertEqual(px[left + 1, y], color)
                    self.assertNotEqual(px[left - 1, y], color)
                    self.assertNotEqual(px[left + 2, y], color)
            self.assertEqual((px[321, 5], px[322, 5]), (auto.TICK_CENTER_RGB,) * 2)
            self.assertEqual(px[393, 5], auto.TICK_RGB)          # +10 deg: x = 98.06 -> 393.7
        with open(files["json"]) as f:
            doc = json.load(f)
        for k in auto.META_FIELDS:
            self.assertEqual(doc[k], meta[k], k)
        self.assertEqual(doc["crc32_hex"], f"{GOLDEN['frame_crc32']:08x}")   # of the frame as sent
        self.assertIs(doc["flippedVertical"], True)
        self.assertEqual(len(doc["bearings"]["ticks"]), 7)
        self.assertEqual(doc["link"], {"session": "x"})

    def test_png_unflipped(self):
        from PIL import Image
        frame = golden_frame()
        meta = auto.parse_meta(auto.meta_payload(meta_for(frame)))
        files = auto.save_frame(self.tmp.name, "20261011-120000", meta, frame, flip=False)
        with Image.open(files["png"]) as im:
            self.assertEqual(im.tobytes(), frame)
        with open(files["json"]) as f:
            self.assertIs(json.load(f)["flippedVertical"], False)

    def test_flip_rows(self):
        self.assertEqual(auto.flip_rows(b"abcdefghijkl", 4, 3), b"ijklefghabcd")
        self.assertEqual(auto.flip_rows(bytearray(b"ab"), 2, 1), b"ab")
        frame = golden_frame()
        self.assertEqual(auto.flip_rows(auto.flip_rows(frame, 160, 120), 160, 120), frame)


# ---------------------------------------------------------------- colour photos (2026-10-15)

class ColourPhotoTest(unittest.TestCase):
    """The colour photo (docs/autonomy-link.md "Photos"): a 44-byte META with pixFmt YUYV, the 38400-byte frame
    decoded to RGB PNGs (full-range BT.601), 'color' in the JSON, the overlays readable; grey photos unchanged."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def sub(self, name):
        path = os.path.join(self.tmp.name, name)
        os.makedirs(path, exist_ok=True)
        return path

    def test_golden_meta_color(self):
        g = auto.decode(KEY, bytes.fromhex(GOLDEN_IH["packets"]["meta_color"]))
        m = auto.parse_meta(g.payload)
        self.assertEqual(len(g.payload), auto.META_COLOR_LEN)
        self.assertEqual((m["pixFmt"], m["pixFmtName"], m["color"], m["size"], m["chunkCount"], m["crc32"]),
                         (auto.PIXFMT_YUYV, "yuyv422", True, 38400, 38, GOLDEN_IH["frame_color_crc32"]))
        self.assertEqual(zlib.crc32(colour_frame()), GOLDEN_IH["frame_color_crc32"])
        grey = auto.parse_meta(auto.decode(KEY, bytes.fromhex(GOLDEN["packets"]["meta"])).payload)
        self.assertEqual((grey["pixFmt"], grey["pixFmtName"], grey["color"]), (auto.PIXFMT_GRAY8, "gray8", False))

    def test_meta_lengths(self):
        frame = colour_frame()
        p = auto.meta_payload(meta_for(frame, pixFmt=auto.PIXFMT_YUYV))
        self.assertEqual((len(p), p[40:]), (44, b"\x01\x00\x00\x00"))
        self.assertEqual(len(auto.meta_payload(meta_for(golden_frame()))), 40)            # grey: the A1 META
        self.assertEqual(len(auto.meta_payload(meta_for(golden_frame(), pixFmt=0))), 40)
        for n in (39, 41, 42, 43, 45):
            with self.subTest(n=n), self.assertRaises(auto.ProtoError):
                auto.parse_meta(bytes(n))
        m = auto.parse_meta(p[:40] + b"\x00\x00\x00\x00")                               # 44 bytes, grey
        self.assertEqual((m["pixFmt"], m["color"]), (0, False))
        m = auto.parse_meta(p[:40] + b"\x07\x00\x00\x00")                               # a format this tool lacks
        self.assertEqual((m["pixFmt"], m["pixFmtName"], m["color"]), (7, "unknown-7", False))

    def test_reassembly(self):
        frame = colour_frame(3)
        f = auto.Frame()
        f.add_meta(auto.parse_meta(auto.meta_payload(meta_for(frame, pixFmt=auto.PIXFMT_YUYV))))
        parts = chunks_of(frame)
        self.assertEqual((len(parts), f.count, len(parts[-1])), (38, 38, 512))
        for i, part in enumerate(parts):
            if i != 30:
                f.add_chunk(i, part)
        self.assertEqual(f.missing(), [30])                      # past the grey frame's 19 chunks
        f.add_chunk(30, parts[30])
        self.assertEqual(f.assemble(), frame)
        f = auto.Frame()                                         # a YUYV META with a grey frame's size
        f.add_meta(meta_for(golden_frame(), pixFmt=auto.PIXFMT_YUYV))
        for i, part in enumerate(chunks_of(golden_frame())):
            f.add_chunk(i, part)
        with self.assertRaisesRegex(auto.FrameError, "inconsistent") as cm:
            f.assemble()
        self.assertTrue(cm.exception.final)
        f = auto.Frame()                                         # an unknown format: final, no refetch
        f.add_meta(meta_for(frame, pixFmt=7))
        for i, part in enumerate(parts):
            f.add_chunk(i, part)
        with self.assertRaisesRegex(auto.FrameError, "unknown pixel format") as cm:
            f.assemble()
        self.assertTrue(cm.exception.final)

    def test_yuv_to_rgb(self):
        # full-range BT.601: R = Y + 1.402 (V - 128), G = Y - 0.344136 (U - 128) - 0.714136 (V - 128),
        # B = Y + 1.772 (U - 128), each rounded and clamped
        cases = {(128, 128, 128): (128, 128, 128), (0, 128, 128): (0, 0, 0), (255, 128, 128): (255, 255, 255),
                 (76, 85, 255): (254, 0, 0), (150, 44, 21): (0, 255, 1), (29, 255, 107): (0, 0, 254),
                 (100, 100, 180): (173, 73, 50), (200, 255, 0): (21, 248, 255)}
        for (y, u, v), rgb in cases.items():
            with self.subTest(yuv=(y, u, v)):
                self.assertEqual(auto.yuv_rgb(y, u, v), rgb)
        # a pixel pair shares its U and V; rows stay in order
        data = bytes((76, 85, 150, 255, 128, 128, 0, 128))       # 2 x 2: (76, 150 | U 85 V 255), (128, 0 | grey)
        self.assertEqual(auto.yuyv_to_rgb(data, 2, 2),
                         bytes((254, 0, 0)) + bytes(auto.yuv_rgb(150, 85, 255)) + bytes((128, 128, 128, 0, 0, 0)))
        with self.assertRaises(ValueError):
            auto.yuyv_to_rgb(bytes(7), 2, 2)

    def test_grey_chroma_decodes_to_the_grey_photo(self):
        from PIL import Image
        grey = golden_frame()
        yuyv = bytes(b for y in grey for b in (y, 128))
        a = auto.save_frame(self.sub("g"), "s", auto.parse_meta(auto.meta_payload(meta_for(grey))),
                            grey)
        b = auto.save_frame(self.sub("c"), "s", auto.parse_meta(auto.meta_payload(meta_for(
            yuyv, pixFmt=auto.PIXFMT_YUYV))), yuyv)
        with Image.open(a["png"]) as ia, Image.open(b["png"]) as ib:
            self.assertEqual((ia.mode, ib.mode), ("L", "RGB"))
            self.assertEqual(ib.tobytes(), bytes(v for g in ia.tobytes() for v in (g, g, g)))

    def test_png_outputs(self):
        from PIL import Image
        frame = colour_frame()
        meta = auto.parse_meta(auto.meta_payload(meta_for(frame, pixFmt=auto.PIXFMT_YUYV)))
        files = auto.save_frame(self.tmp.name, "20261015-120000", meta, frame, {"link": {"session": "x"}})

        def want(x, y):          # the source pixel shown at (x, y) of the upright image
            r = 119 - y
            i = 2 * (r * 160 + x)
            pair = i - (i % 4)
            return auto.yuv_rgb(frame[i], frame[pair + 1], frame[pair + 3])
        with Image.open(files["png"]) as im:
            self.assertEqual((im.size, im.mode), ((160, 120), "RGB"))
            for x, y in ((0, 0), (1, 0), (159, 119), (37, 100), (80, 60), (81, 61)):
                self.assertEqual(im.getpixel((x, y)), want(x, y), (x, y))
            self.assertEqual(im.tobytes(), auto.flip_rows(auto.yuyv_to_rgb(frame, 160, 120), 480, 120))
        with Image.open(files["png_x4"]) as im:
            self.assertEqual((im.size, im.mode), ((640, 480), "RGB"))
            px = im.load()
            for x, y in ((0, 40), (79, 60), (80, 60), (159, 80), (37, 100)):
                for dx, dy in ((0, 0), (3, 3), (1, 2)):
                    self.assertEqual(px[4 * x + dx, 4 * y + dy], want(x, y))
            for d in auto.TICK_DEG:          # the ticks as on a grey photo, inside a 1 px black edge
                left = auto.tick_left(auto.bearing_x(d))
                color = auto.TICK_CENTER_RGB if d == 0 else auto.TICK_RGB
                for y in (0, auto.TICK_LEN - 1, 479 - auto.TICK_LEN + 1, 479):
                    self.assertEqual((px[left, y], px[left + 1, y]), (color, color), f"tick {d} at row {y}")
                    self.assertEqual((px[left - 1, y], px[left + 2, y]), (auto.OUTLINE_RGB,) * 2)
                self.assertEqual(px[left, auto.TICK_LEN], auto.OUTLINE_RGB)
                self.assertEqual(px[left, 479 - auto.TICK_LEN], auto.OUTLINE_RGB)
        with open(files["json"]) as f:
            doc = json.load(f)
        self.assertEqual((doc["color"], doc["pixFmt"], doc["pixFmtName"], doc["size"]), (True, 1, "yuyv422", 38400))
        self.assertEqual(doc["crc32_hex"], f"{zlib.crc32(frame):08x}")            # of the frame as sent
        grey = auto.save_frame(self.sub("grey"), "s",
                               auto.parse_meta(auto.meta_payload(meta_for(golden_frame()))), golden_frame())
        with open(grey["json"]) as f:
            self.assertIs(json.load(f)["color"], False)
        # the colour frame's bytes as sent are kept (.yuyv, rows not flipped: the bench's colourcheck); none for grey
        self.assertEqual(os.path.basename(files["raw"]), "20261015-120000_5_0.yuyv")
        with open(files["raw"], "rb") as f:
            self.assertEqual(f.read(), bytes(frame))
        self.assertEqual(doc["files"]["raw"], files["raw"])
        self.assertNotIn("raw", grey)
        self.assertFalse(os.path.exists(grey["png"][:-4] + ".yuyv"))

    def test_level_line_outlined_in_colour_only(self):
        from PIL import Image
        info = auto.level_info(1.0, 2.0, "flight", {"tilt_flight_deg": 0.5, "tilt_ground_deg": 0.2})
        self.assertTrue(info["drawn"] and info["in_frame"], info)
        frame = colour_frame()
        meta = auto.parse_meta(auto.meta_payload(meta_for(frame, pixFmt=auto.PIXFMT_YUYV, pitch_cdeg=100,
                                                          roll_cdeg=200)))
        c = auto.save_frame(self.sub("c"), "s", meta, frame, level=info)
        g = auto.save_frame(self.sub("g"), "s",
                            auto.parse_meta(auto.meta_payload(meta_for(golden_frame(), pitch_cdeg=100,
                                                                       roll_cdeg=200))), golden_frame(), level=info)
        p, r, t = info["pitch_deg"], info["roll_deg"], info["tilt_deg_used"]
        with Image.open(c["png_x4"]) as ic, Image.open(g["png_x4"]) as ig:
            pc, pg = ic.load(), ig.load()
            dashes = shadows = grey_shadows = 0
            for X in range(40, 600):          # away from the edge ticks
                if X % 16 >= 8:
                    continue
                Y = int(math.floor(auto.level_row_x4(X, p, r, t) + 0.5))
                if not 0 <= Y < 479:
                    continue
                dashes += pc[X, Y] in auto.LEVEL_RGB and pg[X, Y] in auto.LEVEL_RGB
                shadows += pc[X, Y + 1] == auto.OUTLINE_RGB
                grey_shadows += pg[X, Y + 1] == auto.OUTLINE_RGB
            self.assertGreater(dashes, 200)
            self.assertEqual(shadows, dashes)        # every colour dash pixel has its black pixel below
            self.assertLess(grey_shadows, dashes // 4)   # the grey photo is drawn as before (black only where the scene is)
        # rerender_level keeps the colour (the raw .png is RGB)
        lv = auto.rerender_level(c["json"], {"tilt_flight_deg": 0.5, "tilt_ground_deg": 0.2})
        self.assertTrue(lv["drawn"])
        with Image.open(c["png_x4"]) as ic:
            self.assertEqual(ic.mode, "RGB")
            X = 100
            Y = int(math.floor(auto.level_row_x4(X, p, r, lv["tilt_deg_used"]) + 0.5))
            self.assertEqual(ic.load()[X, Y + 1], auto.OUTLINE_RGB)


# ---------------------------------------------------------------- local link check

IFCONFIG = """lo0: flags=8049<UP,LOOPBACK,RUNNING,MULTICAST> mtu 16384
\tinet 127.0.0.1 netmask 0xff000000
en0: flags=8863<UP,BROADCAST,SMART,RUNNING,SIMPLEX,MULTICAST> mtu 1500
\tether 02:00:00:aa:bb:cc
\tinet6 fe80::1%en0 prefixlen 64 secured scopeid 0xe
\tinet 192.168.43.44 netmask 0xffffff00 broadcast 192.168.43.255
\tstatus: active
en18: flags=8863<UP,BROADCAST,SMART,RUNNING,SIMPLEX,MULTICAST> mtu 1500
\tinet 192.0.0.2 netmask 0xffffffff
"""
ROUTE_DRONE = """   route to: 192.168.43.42
destination: 192.168.43.42
  interface: en0
      flags: <UP,HOST,DONE,LLINFO,WASCLONED,IFSCOPE,IFREF,ROUTER>
"""
ROUTE_DEFAULT = """   route to: default
destination: default
       mask: default
    gateway: 192.0.0.1
  interface: en18
"""


def fake_runner(ifconfig=IFCONFIG, drone=ROUTE_DRONE, default=ROUTE_DEFAULT):
    def run(argv):
        name = os.path.basename(argv[0])
        if name == "ifconfig":
            return ifconfig
        if name == "route" and argv[-1] == "default":
            return default
        if name == "route":
            return drone
        raise AssertionError(argv)
    return run


class LinkCheckTest(SandboxedTest):
    def test_parsers(self):
        self.assertEqual(auto.parse_ifconfig(IFCONFIG),
                         [("lo0", "127.0.0.1"), ("en0", "192.168.43.44"), ("en18", "192.0.0.2")])
        self.assertEqual(auto.parse_route_iface(ROUTE_DRONE), "en0")
        self.assertEqual(auto.parse_route_iface(ROUTE_DEFAULT), "en18")
        self.assertIsNone(auto.parse_route_iface(None))

    def test_up(self):
        info = auto.link_check(fake_runner())
        self.assertTrue(info["link_up"])
        self.assertEqual((info["iface"], info["local_ip"], info["default_iface"]), ("en0", "192.168.43.44", "en18"))
        self.assertTrue(info["internet_separate"])
        self.assertEqual(info["warnings"], [])

    def test_default_via_drone_is_a_warning(self):
        info = auto.link_check(fake_runner(default=ROUTE_DEFAULT.replace("en18", "en0")))
        self.assertTrue(info["link_up"])
        self.assertFalse(info["internet_separate"])
        self.assertEqual(len(info["warnings"]), 1)
        info = auto.link_check(fake_runner(default=None))
        self.assertTrue(info["link_up"])
        self.assertIsNone(info["default_iface"])

    def test_down(self):
        info = auto.link_check(fake_runner(ifconfig=IFCONFIG.replace("192.168.43.44", "10.0.0.5"),
                                           drone=ROUTE_DRONE.replace("en0", "en18")))
        self.assertFalse(info["link_up"])
        self.assertIsNone(info["iface"])
        info = auto.link_check(fake_runner(drone=ROUTE_DRONE.replace("en0", "en18")))
        self.assertFalse(info["link_up"])
        self.assertIn("en18", info["problems"][0])

    def test_up_command(self):
        with mock.patch.object(auto, "run_cmd", side_effect=fake_runner()):
            code, out, _, _ = self.run_main(["up"])
        self.assertEqual((code, out["ok"], out["cmd"], out["iface"]), (0, True, "up", "en0"))
        with mock.patch.object(auto, "run_cmd", side_effect=fake_runner(ifconfig="")):
            code, out, _, _ = self.run_main(["up"])
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertIn("error", out)


# ---------------------------------------------------------------- key handling

class KeyTest(SandboxedTest):
    def test_keygen(self):
        path = os.path.join(self.tmp.name, "newdir", "auto.key")
        code, out, raw, err = self.run_main(["keygen", "--key-file", path])
        self.assertEqual((code, out["ok"]), (0, True))
        with open(path) as f:
            text = f.read()
        self.assertRegex(text, r"^[0-9a-f]{32}$")
        key = bytes.fromhex(text)
        self.assertEqual(out["key_fp"], auto.key_fp(key))
        self.assertNotIn(text, raw + err)
        self.assertEqual(stat.S_IMODE(os.stat(path).st_mode), 0o600)
        self.assertEqual(stat.S_IMODE(os.stat(os.path.dirname(path)).st_mode), 0o700)
        code, out, _, _ = self.run_main(["keygen", "--key-file", path])
        self.assertEqual((code, out["ok"], out["key_fp"]), (1, False, auto.key_fp(key)))
        with open(path) as f:
            self.assertEqual(f.read(), text)
        code, out, _, _ = self.run_main(["keygen", "--key-file", path, "--force"])
        self.assertEqual((code, out["replaced_fp"]), (0, auto.key_fp(key)))
        with open(path) as f:
            self.assertNotEqual(f.read(), text)
        self.assertEqual(stat.S_IMODE(os.stat(path).st_mode), 0o600)

    def test_keygen_default_dir_is_private(self):
        code, out, _, _ = self.run_main(["keygen"])
        self.assertEqual(code, 0)
        self.assertEqual(out["key_file"], auto.DEFAULT_KEY_FILE)
        self.assertEqual(stat.S_IMODE(os.stat(auto.DEFAULT_DIR).st_mode), 0o700)

    def install(self, reply_fn):
        sent = []

        class FakeConsole:
            def __init__(self, port):
                self.port = port

            def command(self, cmd, timeout=3.0, until=("OK", "ERR")):
                sent.append(cmd)
                return reply_fn(cmd), []

            def close(self):
                pass

        fake = types.ModuleType("espfly_diag")
        fake.Drone = FakeConsole
        fake.find_port = lambda: "/dev/cu.usbmodemFAKE"
        path = self.key_file()
        with mock.patch.dict(sys.modules, {"espfly_diag": fake}):
            result = self.run_main(["install-key", "--key-file", path])
        return result, sent

    def test_install_key(self):
        (code, out, raw, err), sent = self.install(
            lambda cmd: f"OK autokey stored=nvs fp={auto.key_fp(bytes.fromhex(cmd.split()[1]))} "
                        "note=takes_effect_after_reboot")
        self.assertEqual(sent, [f"autokey {KEY.hex()}"])
        self.assertEqual((code, out["ok"], out["match"], out["drone_fp"], out["key_fp"], out["port"]),
                         (0, True, True, GOLDEN["key_fp"], GOLDEN["key_fp"], "/dev/cu.usbmodemFAKE"))
        self.assertTrue(out["reboot_needed"])
        self.assertNotIn(KEY.hex(), raw + err)

    def test_install_key_mismatch_and_echo_redacted(self):
        (code, out, raw, _), _ = self.install(lambda cmd: f"OK autokey fp=deadbeef got {cmd.split()[1]}")
        self.assertEqual((code, out["ok"], out["match"], out["drone_fp"]), (1, False, False, "deadbeef"))
        self.assertNotIn(KEY.hex(), raw)
        (code, out, _, _), _ = self.install(lambda cmd: "ERR autokey: usage autokey <32 hex digits>")
        self.assertEqual((code, out["ok"]), (1, False))

    def test_missing_key(self):
        code, out, _, _ = self.run_main(["hello", "--host", "127.0.0.1", "--key-file",
                                         os.path.join(self.tmp.name, "none.key")])
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertIn("keygen", out["error"])


# ---------------------------------------------------------------- fake drone

class FakeDrone(threading.Thread):
    """Drone side of the A1 link on 127.0.0.1: HELLO, STATUS_REQ, PHOTO, CHUNKS,
    PHOTO_FREE, with the session, seq/duplicate and ACK rules of the spec."""

    def __init__(self, key=KEY, frames=None, drop=(), lose_first_ack=(), photo_result=auto.R_OK,
                 fp=None, echo_xor=0, rotate_before=None, tamper_first_chunk=False, missing_frames=(),
                 busy_chunks=0, pixfmt=auto.PIXFMT_GRAY8):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.key = key
        self.pixfmt = pixfmt                # YUYV: a photocol 1 drone (44-byte METAs, 38400-byte frames)
        self.frames = frames or ([colour_frame(0), colour_frame(1), colour_frame(2)] if pixfmt == auto.PIXFMT_YUYV
                                 else [golden_frame(), other_frame(1), other_frame(2)])
        self.drop = set(drop)               # (idx, chunkIdx or "meta") left out of the first push
        self.lose_first_ack = set(lose_first_ack)
        self.photo_result = photo_result
        self.fp = fp if fp is not None else bytes.fromhex(auto.key_fp(key))
        self.echo_xor = echo_xor
        self.rotate_before = rotate_before  # drone "reboots" before handling this type once
        self.tamper = tamper_first_chunk
        self.missing_frames = set(missing_frames)   # not captured: META size 0, no chunks
        self.busy_chunks = busy_chunks              # first CHUNKS requests answered BUSY
        self.boot = 0x0BADF00D
        self.rng = random.Random(7)
        self.session = 0
        self.seq = 0
        self.last = None
        self.recent = collections.deque(maxlen=8)
        self.cache = {}
        self.peer = None
        self.photo = None
        self.next_id = 5
        self.freed = []
        self.received = []
        self.auth_fails = 0
        self.stop_evt = threading.Event()
        self.start()

    def stop(self):
        self.stop_evt.set()
        self.join(2)
        self.sock.close()

    def run(self):
        while not self.stop_evt.is_set():
            try:
                data, src = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            self.handle(data, src)

    def packet(self, ptype, payload):
        self.seq += 1
        return auto.encode(self.key, ptype, 0, self.session, self.seq, payload)

    def send(self, ptype, payload):
        self.sock.sendto(self.packet(ptype, payload), self.peer)

    def new_session(self):
        self.session = self.rng.getrandbits(32) | 1
        self.seq = 0
        self.last = None
        self.recent.clear()
        self.cache.clear()

    def push(self, photo_id, n, gap_ms, apply_drop):
        for idx in range(n):
            if idx:
                threading.Event().wait(gap_ms / 1000.0)
            frame = self.frames[idx]
            if idx in self.missing_frames:
                if not (apply_drop and (idx, "meta") in self.drop):
                    self.send(auto.T_META, auto.meta_payload(meta_for(frame, photo_id, idx, n, size=0, crc32=0,
                                                                      chunkCount=0)))
                continue
            if not (apply_drop and (idx, "meta") in self.drop):
                self.send(auto.T_META, auto.meta_payload(meta_for(frame, photo_id, idx, n, pixFmt=self.pixfmt)))
            for ci, part in enumerate(chunks_of(frame)):
                if apply_drop and (idx, ci) in self.drop:
                    continue
                if self.tamper:
                    self.tamper = False
                    bad = bytearray(self.packet(auto.T_CHUNK, auto.chunk_payload(photo_id, idx, ci, part)))
                    bad[20] ^= 0xFF
                    self.sock.sendto(bytes(bad), self.peer)
                self.send(auto.T_CHUNK, auto.chunk_payload(photo_id, idx, ci, part))

    def handle(self, data, src):
        try:
            pkt = auto.decode(self.key, data)
        except auto.ProtoError:
            self.auth_fails += 1
            return
        self.peer = src
        self.received.append(pkt)
        if pkt.type == auto.T_HELLO:
            (nonce,) = struct.unpack("<I", pkt.payload)
            self.new_session()
            self.send(auto.T_HELLO_ACK,
                      auto.hello_ack_payload(self.session, self.boot, nonce ^ self.echo_xor, self.fp, "fake-a1"))
            return
        if self.rotate_before == pkt.type:
            self.rotate_before = None
            self.boot ^= 0x1111
            self.new_session()
        if pkt.session != self.session:
            self.send(auto.T_ACK, auto.ack_payload(pkt.seq, pkt.type, auto.R_BAD_SESSION))
            return
        if pkt.seq in self.recent:
            if pkt.seq in self.cache:
                self.sock.sendto(self.cache[pkt.seq], self.peer)
            return
        if self.last is not None and pkt.seq <= self.last:
            return
        self.last = pkt.seq
        self.recent.append(pkt.seq)
        result, detail, after = auto.R_OK, 0, None
        if pkt.type == auto.T_STATUS_REQ:
            self.send(auto.T_STATUS, auto.status_payload(STATUS_FIELDS | {"bootNonce": self.boot}))
        elif pkt.type == auto.T_PHOTO:
            n, _, gap = struct.unpack("<BBH", pkt.payload)
            if self.photo is not None:
                result = auto.RESULT_NAMES.index("BUSY")
            elif self.photo_result != auto.R_OK:
                result = self.photo_result
            elif not 1 <= n <= 3 or gap > 1000:
                result = auto.RESULT_NAMES.index("RANGE")
            else:
                self.photo = (self.next_id, n)
                detail = self.next_id
                self.next_id += 1
                after = lambda: self.push(detail, n, gap, True)  # noqa: E731
        elif pkt.type == auto.T_CHUNKS:
            pid, idx, count, first = struct.unpack("<HBBH", pkt.payload)
            if self.photo is None or self.photo[0] != pid:
                result = auto.R_NO_PHOTO
            elif idx >= self.photo[1] or count > 8 or \
                    (first != 0 if count == 0 else first + count > auto.chunk_count(len(self.frames[idx]))):
                result = auto.RESULT_NAMES.index("RANGE")
            elif count == 0:
                # count 0: re-send the frame's META (size 0 for a frame never captured), as the firmware
                frame = self.frames[idx]
                meta = meta_for(frame, pid, idx, self.photo[1], size=0, crc32=0, chunkCount=0) \
                    if idx in self.missing_frames else meta_for(frame, pid, idx, self.photo[1], pixFmt=self.pixfmt)

                def after():
                    self.send(auto.T_META, auto.meta_payload(meta))
            elif idx in self.missing_frames:
                result = auto.R_NO_PHOTO
            elif self.busy_chunks:
                self.busy_chunks -= 1
                result = auto.R_BUSY
            else:
                parts = chunks_of(self.frames[idx])

                def after():
                    for ci in range(first, first + count):
                        self.send(auto.T_CHUNK, auto.chunk_payload(pid, idx, ci, parts[ci]))
        elif pkt.type == auto.T_PHOTO_FREE:
            (pid,) = struct.unpack("<H", pkt.payload)
            if self.photo is not None and self.photo[0] == pid:
                self.photo = None
                self.freed.append(pid)
            else:
                result = auto.R_NO_PHOTO
        elif pkt.type != auto.T_HB:
            result = auto.RESULT_NAMES.index("BAD_STATE")
        if pkt.flags & auto.FLAG_ACKREQ:
            ack = self.packet(auto.T_ACK, auto.ack_payload(pkt.seq, pkt.type, result, 0, detail))
            self.cache[pkt.seq] = ack
            if pkt.type in self.lose_first_ack:
                self.lose_first_ack.discard(pkt.type)
            else:
                self.sock.sendto(ack, self.peer)
        if after:
            after()


STATUS_FIELDS = {"upMs": 123456, "vbatMv": 3870, "camStateCode": 2, "state": 0, "fps_x10": 158, "thrust": 0,
                 "frameSeq": 4321, "roll_cdeg": -150, "pitch_cdeg": 220, "yaw_cdeg": 9000, "lum": 97,
                 "texture": 31, "flags": 0b0101, "photoSlotsFree": 1, "authFails": 3, "heapFree": 100000,
                 "heapMin": 90000, "packetsAccepted": 42}


class EndToEndTest(SandboxedTest):
    def drone(self, **kw):
        d = FakeDrone(**kw)
        self.addCleanup(d.stop)
        return d

    def args(self, drone, cmd, *extra, key_file=None):
        return [cmd, "--host", "127.0.0.1", "--port", str(drone.port), "--key-file", key_file or self.key_file(),
                *extra]

    def assert_seq_rule(self, pkts):
        self.assertEqual(pkts[0].seq, 1)
        for prev, cur in zip(pkts, pkts[1:]):
            if cur.seq == prev.seq:
                self.assertTrue(cur.flags & auto.FLAG_RETX, f"repeat of seq {cur.seq} without RETX")
            else:
                self.assertGreater(cur.seq, prev.seq)

    def test_hello(self):
        d = self.drone()
        code, out, _, _ = self.run_main(self.args(d, "hello"))
        self.assertEqual((code, out["ok"], out["cmd"]), (0, True, "hello"))
        self.assertEqual(out["session"], f"{d.session:08x}")
        self.assertEqual((out["bootNonce"], out["fw"], out["drone_fp"], out["key_fp"]),
                         ("0badf00d", "fake-a1", GOLDEN["key_fp"], GOLDEN["key_fp"]))
        self.assertEqual([(p.type, p.session, p.seq) for p in d.received], [(auto.T_HELLO, 0, 1)])

    def test_status(self):
        d = self.drone()
        code, out, _, _ = self.run_main(self.args(d, "status"))
        self.assertEqual((code, out["ok"]), (0, True))
        s = out["status"]
        self.assertEqual((s["camState"], s["vbat_v"], s["fps"], s["roll_deg"], s["pitch_deg"], s["yaw_deg"]),
                         ("running", 3.87, 15.8, -1.5, 2.2, 90.0))
        self.assertEqual((s["motorsIdle"], s["takeoffWindow"], s["onGround"], s["photoBusy"]),
                         (True, False, True, False))
        self.assertEqual((s["frameSeq"], s["authFails"], s["packetsAccepted"], s["bootNonce"]),
                         (4321, 3, 42, "0badf00d"))
        types_ = [p.type for p in d.received]
        self.assertEqual(types_, [auto.T_HELLO, auto.T_STATUS_REQ])
        self.assertEqual(d.received[1].session, d.session)
        self.assertEqual(d.received[1].flags & auto.FLAG_ACKREQ, 0)
        self.assert_seq_rule(d.received)

    def test_status_after_drone_reset_rehellos(self):
        d = self.drone(rotate_before=auto.T_STATUS_REQ)
        code, out, _, err = self.run_main(self.args(d, "status"))
        self.assertEqual((code, out["ok"]), (0, True), err)
        self.assertEqual([p.type for p in d.received],
                         [auto.T_HELLO, auto.T_STATUS_REQ, auto.T_HELLO, auto.T_STATUS_REQ])
        self.assertEqual(out["session"], f"{d.session:08x}")
        self.assert_seq_rule(d.received)

    def test_photo_with_lost_ack_dropped_chunk_and_bad_tag(self):
        from PIL import Image
        d = self.drone(drop={(0, 7)}, lose_first_ack={auto.T_PHOTO}, tamper_first_chunk=True)
        out_dir = os.path.join(self.tmp.name, "photos")
        code, out, _, err = self.run_main(self.args(d, "photo", "--n", "2", "--gap", "50", "--out", out_dir))
        self.assertEqual((code, out["ok"]), (0, True), err)
        self.assertEqual((out["id"], out["n"], out["free"], out["failed"]), (5, 2, "OK", []))
        self.assertGreaterEqual(out["link"]["bad_tag"], 1)
        self.assertEqual(d.freed, [5])
        sent = [(p.type, p.flags, p.seq, p.payload) for p in d.received]
        photos = [s for s in sent if s[0] == auto.T_PHOTO]
        self.assertEqual(len(photos), 2)                                 # first ACK lost
        self.assertEqual(photos[0][2], photos[1][2])
        self.assertEqual((photos[0][1], photos[1][1]), (auto.FLAG_ACKREQ, auto.FLAG_ACKREQ | auto.FLAG_RETX))
        self.assertEqual(photos[0][3], auto.photo_payload(2, 50))
        chunks = [s for s in sent if s[0] == auto.T_CHUNKS]
        self.assertEqual([c[3] for c in chunks], [auto.chunks_payload(5, 0, 1, 7)])
        self.assertTrue(chunks[0][1] & auto.FLAG_ACKREQ)
        self.assertEqual(sent[-1][0], auto.T_PHOTO_FREE)
        self.assertEqual(sent[-1][3], auto.photo_free_payload(5))
        self.assertTrue(all(p.session == d.session for p in d.received[1:]))
        self.assert_seq_rule(d.received)
        self.assertEqual([f["idx"] for f in out["frames"]], [0, 1])
        self.assertGreaterEqual(out["frames"][0]["rerequest_rounds"], 1)
        for f in out["frames"]:
            frame = d.frames[f["idx"]]
            for k in ("png", "png_x4", "json"):
                self.assertTrue(f[k].startswith(out_dir + os.sep) and os.path.exists(f[k]), f[k])
            self.assertRegex(os.path.basename(f["png"]), r"^\d{8}-\d{6}_5_%d\.png$" % f["idx"])
            with Image.open(f["png"]) as im:
                self.assertEqual((im.size, im.mode, im.tobytes()), ((160, 120), "L", auto.flip_rows(frame, 160, 120)))
            with Image.open(f["png_x4"]) as im:
                self.assertEqual(im.size, (640, 480))
            with open(f["json"]) as fh:
                doc = json.load(fh)
            self.assertEqual((doc["id"], doc["idx"], doc["n"], doc["crc32"]), (5, f["idx"], 2, zlib.crc32(frame)))
            self.assertEqual(f["crc32"], f"{zlib.crc32(frame):08x}")
        self.assertFalse(os.path.exists(auto.DEFAULT_PHOTO_DIR))

    def test_photo_colour(self):
        """A photocol 1 drone: 44-byte METAs, 38 chunks a frame; a chunk past the grey frame's 19 is re-requested;
        RGB PNGs and 'color' true in the reply and the JSON."""
        from PIL import Image
        d = self.drone(pixfmt=auto.PIXFMT_YUYV, drop={(1, 30)})
        out_dir = os.path.join(self.tmp.name, "photos")
        code, out, _, err = self.run_main(self.args(d, "photo", "--n", "2", "--gap", "50", "--out", out_dir))
        self.assertEqual((code, out["ok"]), (0, True), err)
        chunks = [struct.unpack("<HBBH", p.payload) for p in d.received if p.type == auto.T_CHUNKS]
        self.assertEqual(chunks, [(5, 1, 1, 30)])
        for f in out["frames"]:
            frame = d.frames[f["idx"]]
            self.assertIs(f["color"], True)
            with Image.open(f["png"]) as im:
                self.assertEqual((im.size, im.mode), ((160, 120), "RGB"))
                self.assertEqual(im.tobytes(), auto.flip_rows(auto.yuyv_to_rgb(frame, 160, 120), 480, 120))
            with open(f["json"]) as fh:
                doc = json.load(fh)
            self.assertEqual((doc["color"], doc["size"], doc["chunkCount"], doc["crc32"]),
                             (True, 38400, 38, zlib.crc32(frame)))
        self.assertEqual(d.freed, [5])

    def test_photo_lost_meta_is_re_requested(self):
        d = self.drone(drop={(1, "meta")})
        out_dir = os.path.join(self.tmp.name, "photos")
        code, out, _, _ = self.run_main(self.args(d, "photo", "--n", "2", "--gap", "0", "--out", out_dir,
                                                  "--timeout", "5"))
        self.assertEqual((code, out["ok"]), (0, True))
        self.assertEqual([f["idx"] for f in out["frames"]], [0, 1])
        chunks = [struct.unpack("<HBBH", p.payload) for p in d.received if p.type == auto.T_CHUNKS]
        self.assertIn((5, 1, 0, 0), chunks)                      # CHUNKS count 0: META again
        self.assertEqual(d.freed, [5])

    def test_photo_uncaptured_frame_and_no_photo_chunks(self):
        # Frame 1: META of size 0 (not captured) -> fails at once, never re-requested.
        # Frame 2: not captured and its META lost -> CHUNKS count 0 brings its size-0 META back,
        # which ends only that frame.
        d = self.drone(missing_frames={1, 2}, drop={(2, "meta")}, busy_chunks=0)
        out_dir = os.path.join(self.tmp.name, "photos")
        code, out, _, _ = self.run_main(self.args(d, "photo", "--n", "3", "--gap", "0", "--out", out_dir,
                                                  "--timeout", "8"))
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertEqual([f["idx"] for f in out["frames"]], [0])
        failed = {f["idx"]: f["error"] for f in out["failed"]}
        self.assertIn("captured no frame", failed[1])
        self.assertIn("captured no frame", failed[2])
        chunks = [struct.unpack("<HBBH", p.payload) for p in d.received if p.type == auto.T_CHUNKS]
        self.assertTrue(chunks and all(c[1] == 2 for c in chunks), chunks)
        self.assertEqual(chunks, [(chunks[0][0], 2, 0, 0)])     # only the META was asked for
        self.assertEqual(d.freed, [5])

    def test_busy_chunks_do_not_use_up_rounds(self):
        d = self.drone(drop={(0, 3), (0, 12)}, busy_chunks=6)
        out_dir = os.path.join(self.tmp.name, "photos")
        code, out, _, err = self.run_main(self.args(d, "photo", "--out", out_dir))
        self.assertEqual((code, out["ok"]), (0, True), err)
        busy_and_ok = [p for p in d.received if p.type == auto.T_CHUNKS]
        self.assertGreater(len(busy_and_ok), auto.MAX_ROUNDS)
        with open(out["frames"][0]["json"]) as fh:
            self.assertEqual(json.load(fh)["crc32"], zlib.crc32(d.frames[0]))

    def test_photo_refused_by_name(self):
        d = self.drone(photo_result=auto.RESULT_NAMES.index("CAM_UNAVAIL"))
        out_dir = os.path.join(self.tmp.name, "photos")
        code, out, _, _ = self.run_main(self.args(d, "photo", "--out", out_dir))
        self.assertEqual((code, out["ok"], out["result"]), (1, False, "CAM_UNAVAIL"))
        self.assertIn("camera", out["error"])
        self.assertEqual(os.listdir(out_dir), [])

    def test_wrong_key(self):
        d = self.drone(key=bytes(16))
        code, out, _, _ = self.run_main(self.args(d, "hello"))
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertIn("no HELLO_ACK", out["error"])
        self.assertEqual(d.auth_fails, 1 + auto.Link.RETRIES)

    def test_fingerprint_and_nonce_checked(self):
        d = self.drone(fp=b"\0\0\0\0")
        code, out, _, _ = self.run_main(self.args(d, "hello"))
        self.assertEqual((code, out["ok"], out["drone_fp"]), (1, False, "00000000"))
        d = self.drone(echo_xor=1)
        code, out, _, _ = self.run_main(self.args(d, "hello"))
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertEqual(len(d.received), 1 + auto.Link.RETRIES)
        self.assertTrue(all(p.flags & auto.FLAG_RETX for p in d.received[1:]))

    def test_usage_error_is_json(self):
        code, out, _, _ = self.run_main(["photo", "--n", "4"])
        self.assertEqual((code, out["ok"]), (2, False))


# ================================================================ stage A2b (docs/autonomy-a2b.md section 12.4)

GOLDEN_A2_PATH = os.path.join(HERE, "auto_proto_golden_a2.json")
with open(GOLDEN_A2_PATH) as _f:
    GOLDEN_A2 = json.load(_f)
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
TOOL = os.path.join(REPO, "tools", "espfly_auto.py")
GOLDEN_AUTO_STATUS = dict(
    upMs=123456, bootNonce=0x0BADF00D, transitions=7, failedGuards=0x4, state=3, mode=0, reason=3, vzLandReason=0,
    landTrip=0, vzPhase=4, vzReason=4, flags=0x8D, trips=0x40, capture=2, photoId=5, stateMs=2100, airMs=4300,
    stableMs=1050, waitLeftMs=3950, hoverWaitMs=5000, vbatMv=3800, vbatFastMinMv=3420, hMm=305, vzMms=-12,
    thrust=47100, tLo=35400, tHold=46800, tRef=0, kPriorDiv10=16500, khatDiv10=16620, lastCmdType=6, lastCmdResult=0,
    killCount=1)


def rs(name):
    return auto.REASON_NAMES.index(name)


def wait_for(pred, timeout, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(step)
    return bool(pred())


class GoldenA2Test(unittest.TestCase):
    def test_packets_from_fields(self):
        S, A = SESSION, auto.FLAG_ACKREQ
        built = {
            "arm": auto.encode(KEY, auto.T_ARM, A, S, 10, auto.arm_payload(auto.MODES["dry"])),
            "arm_air": auto.encode(KEY, auto.T_ARM, A, S, 11, auto.arm_payload(auto.MODES["flight"], 3000)),
            "takeoff": auto.encode(KEY, auto.T_TAKEOFF, A, S, 12, auto.takeoff_payload(0x5EED0001, 40, 2, 300)),
            "land": auto.encode(KEY, auto.T_LAND, A, S, 13),
            "disarm": auto.encode(KEY, auto.T_DISARM, A, S, 14),
            "kill": auto.encode(KEY, auto.T_KILL, A, S, 15),
            "kill_any_session": auto.encode(KEY, auto.T_KILL, A, 0, 0x7FFFFFFF),
            "photo_hover": auto.encode(KEY, auto.T_PHOTO, A, S, 16, auto.photo_payload(1, 150)),
            "ack_arm": auto.encode(KEY, auto.T_ACK, 0, S, 20, auto.ack_payload(10, auto.T_ARM, auto.R_OK, 1, 0x5EED0001)),
            "ack_guards": auto.encode(KEY, auto.T_ACK, 0, S, 21, auto.ack_payload(11, auto.T_ARM, auto.R_GUARDS, 0,
                                                                                0x40C)),
            "auto_status": auto.encode(KEY, auto.T_AUTO_STATUS, 0, S, 22,
                                       auto.auto_status_payload(GOLDEN_AUTO_STATUS, auto.AUTO_STATUS_LEN_A2B)),
        }
        self.assertEqual(GOLDEN_A2["key"], KEY.hex())
        self.assertEqual(GOLDEN_A2["auto_status_fmt"], auto.AUTO_STATUS_FMT)
        self.assertEqual(set(built), set(GOLDEN_A2["packets"]))
        for name, pkt in built.items():
            with self.subTest(name=name):
                self.assertEqual(pkt.hex(), GOLDEN_A2["packets"][name])

    def test_parse_back(self):
        g = {k: auto.decode(KEY, bytes.fromhex(v)) for k, v in GOLDEN_A2["packets"].items()}
        for name, p in g.items():
            with self.subTest(name=name):
                self.assertEqual(auto.encode(KEY, p.type, p.flags, p.session, p.seq, p.payload).hex(),
                                 GOLDEN_A2["packets"][name])
        self.assertEqual(struct.unpack("<BBH", g["arm"].payload), (1, 0, 0))
        self.assertEqual(struct.unpack("<BBH", g["arm_air"].payload), (0, 0, 3000))
        self.assertEqual(struct.unpack("<IHBBHH", g["takeoff"].payload), (0x5EED0001, 40, 2, 0, 300, 0))
        for name in ("land", "disarm", "kill", "kill_any_session"):
            self.assertEqual(g[name].payload, b"")
        self.assertEqual((g["kill_any_session"].session, g["kill_any_session"].seq), (0, 0x7FFFFFFF))
        a = auto.parse_ack(g["ack_arm"].payload)
        self.assertEqual((a["ackSeq"], a["ackType"], a["resultName"], a["stateName"], a["detail"]),
                         (10, auto.T_ARM, "OK", "ARMED", 0x5EED0001))
        a = auto.parse_ack(g["ack_guards"].payload)
        self.assertEqual((a["resultName"], auto.guard_names(a["detail"])), ("GUARDS", ["STILL", "BATT", "GESTURE"]))
        s = auto.parse_auto_status(g["auto_status"].payload)
        for k, v in GOLDEN_AUTO_STATUS.items():
            self.assertEqual(s[k], v, k)
        self.assertEqual((s["stateName"], s["modeName"], s["reasonName"], s["vzPhaseName"], s["vzReasonName"],
                          s["captureName"], s["lastCmdName"], s["lastCmdResultName"]),
                         ("HOVER", "FLIGHT", "HOLD", "HOLD", "HOLD_REACHED", "DONE", "PHOTO", "OK"))
        self.assertEqual(s["guards"], ["STILL"])
        self.assertEqual(s["flagNames"], ["waiting", "camHeight", "camAwake", "hold"])
        self.assertEqual(s["tripNames"], ["rest"])
        self.assertTrue(s["waiting"])
        self.assertAlmostEqual(s["h_m"], 0.305)
        self.assertEqual(auto.auto_status_payload(s, auto.AUTO_STATUS_LEN_A2B), g["auto_status"].payload)
        # an A2b drone's 64-byte AUTO_STATUS still parses: no interactive fields
        self.assertFalse(s["interactive"])
        self.assertEqual((s["mvSeq"], s["sceneD_m"], s["mvStateName"]), (0, None, "IDLE"))
        for n in (63, 65, 95, 97):
            with self.assertRaises(auto.ProtoError):
                auto.parse_auto_status(bytes(n))


def c_enum(text, type_name, prefix):
    """Names of a C 'typedef enum {...} type_name;' in value order, without the prefix."""
    m = re.search(r"typedef enum \{([^{}]*)\}\s*" + type_name + r"\s*;", text)
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    out, v = {}, -1
    for item in body.split(","):
        name, _, val = item.strip().partition("=")
        if not name.strip():
            continue
        v = int(val.strip().rstrip("u"), 0) if val.strip() else v + 1
        out[v] = name.strip()[len(prefix):]
    return tuple(out[i] for i in range(len(out)))


class NameTablesTest(unittest.TestCase):
    """The Python name tables against the firmware headers (enums are append-only there)."""

    def header(self, rel):
        with open(os.path.join(REPO, "components", "core", "crazyflie", rel)) as f:
            return f.read()

    def test_names(self):
        fsm = self.header("utils/interface/auto_fsm.h")
        vz = self.header("utils/interface/vert_ctrl.h")
        proto = self.header("utils/interface/auto_proto.h")
        self.assertEqual(c_enum(fsm, "autoState_t", "AUTO_"), auto.STATE_NAMES)
        self.assertEqual(c_enum(fsm, "autoMode_t", "AUTO_MODE_"), auto.MODE_NAMES)
        self.assertEqual(c_enum(fsm, "autoReason_t", "AUTO_RS_"), auto.REASON_NAMES)
        self.assertEqual(c_enum(fsm, "autoCmdType_t", "AUTO_CMD_"), auto.CMD_NAMES)
        self.assertEqual(c_enum(fsm, "autoCapture_t", "AUTO_CAP_"), auto.CAPTURE_NAMES)
        self.assertEqual(c_enum(vz, "vzPhase_t", "VZ_"), auto.VZ_PHASE_NAMES)
        self.assertEqual(c_enum(vz, "vzReason_t", "VZ_R_"), auto.VZ_REASON_NAMES)
        self.assertEqual(c_enum(proto, "autoResult_t", "AUTO_R_"), auto.RESULT_NAMES)
        guards = {int(b): n for n, b in re.findall(r"AUTO_G_(\w+)\s*=\s*1u\s*<<\s*(\d+)", fsm)}
        self.assertEqual(tuple(guards[i] for i in range(len(guards))), auto.GUARD_NAMES)
        for prefix, table in (("AUTO_STF_", auto.STF_NAMES), ("AUTO_TRF_", auto.TRF_NAMES)):
            bits = {int(v, 16): n for n, v in re.findall(r"#define " + prefix + r"(\w+)\s+(0x[0-9A-Fa-f]+)", proto)}
            self.assertEqual([bits[1 << i].replace("_", "").lower() for i in range(8)], [n.lower() for n in table])
        self.assertIn('"' + auto.AUTO_STATUS_FMT + '"', proto)
        self.assertIn('"' + auto.AUTO_STATUS_EXT_FMT + '"', proto)
        self.assertIn('"' + auto.AUTO_STATUS_HR_FMT + '"', proto)
        self.assertRegex(proto, r"#define AUTO_AUTO_STATUS_LEN\s+%d\b" % auto.AUTO_STATUS_LEN)
        self.assertRegex(proto, r"#define AUTO_AUTO_STATUS_IH_LEN\s+%d\b" % auto.AUTO_STATUS_LEN_IH)
        # hold and report (2026-10-15): the events and the report's bits
        self.assertEqual(c_enum(fsm, "autoEvent_t", "AUTO_EV_"), auto.EVENT_NAMES)
        for prefix, table in (("AUTO_HB_", auto.HB_NAMES), ("AUTO_CB_", auto.CB_NAMES)):
            bits = {int(v, 16): n for n, v in re.findall(r"#define " + prefix + r"(\w+)\s+(0x[0-9A-Fa-f]+)", fsm)}
            self.assertEqual([bits[1 << i].replace("_", "").lower() for i in range(len(table))],
                             [n.lower() for n in table])
        for name, v in (("NO_HUSH", auto.CONTACT_NO_HUSH), ("MOTION", auto.CONTACT_MOTION)):
            self.assertRegex(fsm, r"#define AUTO_CONTACT_%s\s+0x%02X\b" % (name, v))
        ns = {int(v, 16): n for n, v in re.findall(r"#define AUTO_NS_(\w+)\s+(0x[0-9A-Fa-f]+)", fsm)}
        self.assertEqual([ns[1 << i].lower() for i in range(len(ns))], [n.lower() for n in auto.NS_NAMES])
        for code, name in ((auto.EV_SINK_CODE, "SINK"), (auto.EV_NOT_STABLE, "NOT_STABLE"),
                           (auto.EV_LOOP_GAP, "LOOP_GAP"), (auto.EV_HOLD_NO_VIEW, "HOLD_NO_VIEW")):
            self.assertEqual(auto.EVENT_NAMES[code], name)
        # the details' meanings as auto_fsm.c sets them (camRule's SINK, viewRule's HOLD_NO_VIEW)
        src = open(os.path.join(REPO, "components", "core", "crazyflie", "utils", "src", "auto_fsm.c")).read()
        self.assertIn("bound ? 0u : ((f->contactBlind || f->faultHover) ? 2u : 1u)", src)
        self.assertIn("in->holdCoasting ? 1u : (!in->holdApplied ? 2u : 3u)", src)

        motion = self.header("utils/interface/auto_motion.h")
        self.assertEqual(c_enum(motion, "autoMvState_t", "AUTO_MV_"), auto.MV_STATE_NAMES)
        self.assertEqual(c_enum(motion, "autoMvKind_t", "AUTO_MVK_"), auto.MV_KIND_NAMES)
        self.assertEqual(c_enum(motion, "autoMvWhy_t", "AUTO_MVW_"), auto.MV_WHY_NAMES)
        ends = c_enum(motion, "autoMvEnd_t", "AUTO_MVE_")
        self.assertEqual(ends, auto.MV_END_NAMES[:len(ends)])   # append-only: the Mac may know a newer end (FRONT)
        self.assertGreaterEqual(len(auto.MV_END_NAMES), len(ends))
        bits = {int(v, 16): n for n, v in re.findall(r"#define AUTO_MVF_(\w+)\s+(0x[0-9A-Fa-f]+)", motion)}
        self.assertEqual([bits[1 << i].replace("_", "").lower() for i in range(len(bits))],
                         [n.lower() for n in auto.MVF_NAMES])
        for t, name in ((auto.T_YAW, "YAW"), (auto.T_MOVE, "MOVE"), (auto.T_STOP, "STOP"),
                        (auto.T_SET_HEIGHT, "SET_HEIGHT")):
            self.assertRegex(proto, r"#define AUTO_T_%s\s+0x%02X\b" % (name, t))
        self.assertRegex(motion, r"#define AUTO_MV_YAW_RATE_DPS\s+%.1ff\b" % auto.YAW_RATE_DPS)
        # the horizontal profile (2026-10-14: AUTO_MV_V_H_MAX 0.30, inside #ifndef so the log replays can pin 0.20)
        for name, v in (("V_H_MAX", auto.MOVE_V_H_MAX), ("A_H", auto.MOVE_A_H), ("W_LAT_MAX", auto.MOVE_W_LAT_MAX),
                        ("W_FWD_MAX", auto.MOVE_W_FWD_MAX), ("V_FWD_MAX", auto.MOVE_V_FWD_MAX)):
            m = re.search(r"#define AUTO_MV_%s\s+([0-9.]+)f" % name, motion)
            self.assertIsNotNone(m, name)
            self.assertAlmostEqual(float(m.group(1)), v, places=6, msg=name)
        self.assertRegex(motion, r"#ifndef AUTO_MV_V_H_MAX\s+#define AUTO_MV_V_H_MAX\s+0\.30f\s+#endif")
        self.assertRegex(fsm, r"#define AUTO_HEIGHT_CM_MAX\s+%d\b" % auto.HEIGHT_CM[1])
        # the caps (2026-10-15, asserted in C too): the take-off's is the controller's VZ_H_TAKEOFF_MAX (1.20 m), the
        # hover's (2.00 m, by moves up) the moves' AUTO_MV_H_MAX_M and the controller's VZ_H_TARGET_MAX
        self.assertRegex(vz, r"#define VZ_H_TAKEOFF_MAX\s+%.2ff" % (auto.HEIGHT_CM[1] / 100.0))
        self.assertRegex(vz, r"#define VZ_H_TARGET_MAX\s+%.2ff" % auto.MOVE_H_RANGE_M[1])
        self.assertRegex(motion, r"#define AUTO_MV_H_MAX_M\s+%.2ff" % auto.MOVE_H_RANGE_M[1])
        self.assertRegex(motion, r"#define AUTO_MV_H_MIN_M\s+%.2ff" % auto.MOVE_H_RANGE_M[0])
        self.assertLessEqual(auto.HEIGHT_CM[1], auto.MOVE_H_RANGE_M[1] * 100)
        # SET_HEIGHT (2026-10-15, docs/autonomy-a2b.md 23): the Mac's limits and the level line's focal length are the
        # drone's (the fake drone below uses these same numbers)
        self.assertRegex(fsm, r"#define AUTO_SH_X_CM_MIN\s+%d\b" % round(auto.SH_X_M[0] * 100))
        self.assertRegex(fsm, r"#define AUTO_SH_X_CM_MAX\s+%d\b" % round(auto.SH_X_M[1] * 100))
        self.assertRegex(fsm, r"#define AUTO_SH_MAX_M\s+%.2ff" % auto.SH_MAX_M)
        self.assertRegex(fsm, r"#define AUTO_SH_GAP_MS\s+%d\b" % round(auto.SH_GAP_S * 1000))
        self.assertRegex(fsm, r"#define AUTO_SH_AT_NOW\s+INT16_MIN\b")
        self.assertEqual(auto.SH_AT_NOW, -32768)
        self.assertRegex(motion, r"#define AUTO_MV_FY_PX\s+%.2ff" % auto.LEVEL_FY_PX)
        self.assertIn("wait %g s" % auto.SH_GAP_S, auto.MV_WHY_HINTS["SH_RATE"])
        self.assertRegex(fsm, r"#define AUTO_HOVER_WAIT_MAX_MS\s+%d\b" % auto.ACT_WAIT_MS[1])
        self.assertRegex(fsm, r"#define AUTO_HOVER_WAIT_TAKEOFF_MIN_MS\s+%d\b" % auto.ACT_WAIT_MS[0])
        # no air clock since 2026-10-15 (AIR_HARD_MS is the 96-byte drone's, for its airLeftDs)
        self.assertNotRegex(fsm, r"#define AUTO_AIR_HARD_MS\b")
        for name, v in (("HORIZ_MAX_CM", auto.MOVE_HORIZ_MAX_M * 100), ("UP_MAX_CM", auto.MOVE_UP_MAX_M * 100),
                        ("YAW_MAX_DEG", auto.YAW_MAX_DEG)):
            self.assertRegex(motion, r"#define AUTO_MV_%s\s+%d\b" % (name, round(v)))
        # long moves (2026-10-14): the Mac's forward limit is the drone's (1.50 before the long-move build)
        fwd = re.search(r"#define AUTO_MV_FWD_MAX_CM\s+(\d+)", motion)
        self.assertEqual(round(auto.MOVE_FWD_MAX_M * 100), int(fwd.group(1)) if fwd else round(auto.MOVE_HORIZ_MAX_M * 100))
        length = re.search(r"#define AUTO_MV_LEN_MAX_CM\s+(\d+)", motion)
        if length:
            self.assertEqual(round(auto.MOVE_LEN_MAX_M * 100), int(length.group(1)))
            self.assertRegex(proto, r"#define AUTO_TAKEOFF_PLAN_LEN\s+%d\b" % auto.TAKEOFF_PLAN_LEN)
            self.assertRegex(motion, r"AUTO_MVE_FRONT = %d," % auto.MVE_FRONT)
        self.assertRegex(motion, r"#define AUTO_MOVE_MIN_FRONT_CM\s+(\d+)")
        drone_front = int(re.search(r"#define AUTO_MOVE_MIN_FRONT_CM\s+(\d+)", motion).group(1)) / 100.0
        self.assertGreaterEqual(auto.MAC_MIN_FRONT_M, drone_front)
        # the TAKEOFF's plan flies the Mac's floor (2026-10-15 review): the header's and the Mac's are one number
        plan_front = re.search(r"#define AUTO_PLAN_MIN_FRONT_CM\s+(\d+)", motion)
        if plan_front:
            self.assertEqual(int(plan_front.group(1)), round(auto.PLAN_MIN_FRONT_M * 100))
            self.assertEqual(auto.PLAN_MIN_FRONT_M, auto.MAC_MIN_FRONT_M)
        # the fixes after 591f176's verification: the same numbers on both sides
        for name, v in (("K_LOW", auto.K_LOW), ("LAT_D_REL", auto.LAT_D_REL), ("H_MIN_M", auto.H_FLOOR_M),
                        ("H_DRIFT_MPS", auto.H_DRIFT_MPS), ("H_DRIFT_CAP_M", auto.H_DRIFT_CAP_M)):
            m = re.search(r"#define AUTO_MV_%s\s+([0-9.]+)f" % name, motion)
            self.assertIsNotNone(m, name)
            self.assertAlmostEqual(float(m.group(1)), v, places=6, msg=name)
        # the guards down to off (2026-10-18): the settings' ranges and defaults as the header has them
        for name, v in (("MG_KLOW_MIN_PCT", auto.MV_KLOW_PCT[0]), ("MG_KLOW_MAX_PCT", auto.MV_KLOW_PCT[1]),
                        ("MG_LAT_REL_MAX_PCT", auto.MV_LAT_PCT[1]), ("MG_WFWD_MIN_PCT", auto.MV_WFWD_PCT[0]),
                        ("MG_WFWD_MAX_PCT", auto.MV_WFWD_PCT[1]), ("W_FWD_PCT", auto.MV_WFWD_DEFAULT_PCT),
                        ("K_LOW_PCT", round(auto.K_LOW * 100)), ("LAT_D_REL_PCT", round(auto.LAT_D_REL * 100))):
            m = re.search(r"#define AUTO_MV_%s\s+(\d+)\b" % name, motion)
            self.assertIsNotNone(m, name)
            self.assertEqual(int(m.group(1)), v, name)
        self.assertRegex(motion, r"#define D_FLOOR_M\s+0\.05f") if "D_FLOOR_M" in motion else None
        for name, v in (("HELD", auto.BATTLOW_HELD), ("NOW", auto.BATTLOW_NOW)):
            self.assertRegex(proto, r"#define AUTO_BATTLOW_%s\s+0x%02X\b" % (name, v))
        mailbox = self.header("utils/interface/auto_mailbox.h")
        self.assertRegex(mailbox, r"#define AUTO_DETAIL_TAKEN_UNKNOWN 0xFFFFFFFFu")
        self.assertEqual(auto.DETAIL_TAKEN_UNKNOWN, 0xFFFFFFFF)
        self.assertEqual({auto.TYPE_NAMES[t]: c for t, c in auto.CMD_OF_TYPE.items()},
                         {n: i for i, n in enumerate(auto.CMD_NAMES) if n != "NONE"})

    def test_event_details(self):
        """event_view decodes every event's detail (2026-10-15 verification: NOT_STABLE, SINK, HOLD_NO_VIEW and
        LOOP_GAP came without one)."""
        e = auto.event_view(auto.EV_NOT_STABLE, 0x09, 10)
        self.assertEqual((e["detailName"], e["failed"][1]), (["tilt", "vz"], "the vertical controller not steady"))
        self.assertEqual(auto.event_view(auto.EV_SINK_CODE, 2, 0)["detailName"], "afterContactOrClimbFault")
        self.assertIn("5 s", auto.event_view(auto.EV_SINK_CODE, 0, 0)["why"])
        self.assertEqual(auto.event_view(auto.EV_HOLD_NO_VIEW, 3, 0)["detailName"], "stale")
        g = auto.event_view(auto.EV_LOOP_GAP, 27, 0)
        self.assertEqual((g["gap_ms"], g["detailName"]), (27, "27 ms"))
        self.assertEqual(auto.event_view(auto.EV_LOOP_GAP, 255, 0)["detailName"], ">= 255 ms")

    def test_busy_detail_kinds(self):
        """D1: the BUSY detail kinds against auto_proto.h AUTO_DETAIL_* (added there by the link package; the
        test checks whichever of them the header defines, and all of them once it has the BUSY kinds)."""
        proto = self.header("utils/interface/auto_proto.h")
        found = {k: int(v, 16) for k, v in re.findall(
            r"#define AUTO_DETAIL_(BUSY_PENDING|BUSY_UNTAKEN|BUSY_HELD_SET|KIND_MASK|VALUE_MASK)\s+(0x[0-9A-Fa-f]+)u?",
            proto)}
        want = {"BUSY_PENDING": auto.DETAIL_BUSY_PENDING, "BUSY_UNTAKEN": auto.DETAIL_BUSY_UNTAKEN,
                "BUSY_HELD_SET": auto.DETAIL_BUSY_HELD_SET, "KIND_MASK": auto.DETAIL_KIND_MASK,
                "VALUE_MASK": auto.DETAIL_VALUE_MASK}
        if "AUTO_DETAIL_BUSY_" not in proto:
            self.skipTest("auto_proto.h does not define the BUSY kinds yet (the link package adds them)")
        self.assertEqual(found, want)


# ---------------------------------------------------------------- fake drone with FakeAutonomy

IDLE, ARMED, TAKEOFF, HOVER, LAND, KILLED = range(6)


ESP_AHEAD_MS = 300    # the drone's esp_timer (STATUS upMs, META captureUpMs) runs this far ahead of its FreeRTOS tick


class FakeAutoDrone(FakeDrone):
    """The A2b drone on 127.0.0.1: FakeDrone's link plus FakeAutonomy.

    ARM (configurable guards) gives a token; TAKEOFF checks it; a pre-spin, HOVER at hover_at s;
    capture_after s into HOVER (STABLE + 1 s) it pushes the take-off set unsolicited (id
    fsm_photo_id, set_drop leaves chunks out of the first push), then waits with a waitLeftMs
    countdown; HOVER_TIMEOUT lands, TOUCHDOWN after land_s, IDLE rest_s later. LAND, DISARM, KILL
    (any session, any seq, answered at its sender, the peer unchanged), PHOTO in the wait (BUSY
    with the held id while a set it started is held). No heartbeat for 1 s while active: LAND
    HB_LOST. AUTO_STATUS pushed on transitions, at 10 Hz while active, 2 Hz for 2 s after IDLE
    and after each STATUS. An ARM within 2 s of a KILL is refused KILL_LATCHED. Every BUSY the link
    answers carries its kind (D1: auto_proto.h AUTO_DETAIL_BUSY_*). Injections: busy_unknown (act,
    answer BUSY taken-unknown), busy_state (answer BUSY untaken, do not act), lose_cmd (the first
    packet of a type is lost), bad_session_in_hover (another session replaces ours at HOVER),
    free_delay (PHOTO_FREE processed late), slow ({type: s}: the command stays pending s, every
    other state-machine command meanwhile answered BUSY pending), stall ({type: (act, s)}: answered
    BUSY taken-unknown, the AUTO_STATUS snapshot frozen for s as with a stalled stabilizer, acted on
    if act; used once), land_busy (the first LANDs answered BUSY untaken), mute_acks (types whose
    ACKs are never sent), mute_first (types whose first command's ACK, and its replays, are never
    sent), held_reply (ids: the next PHOTOs answered BUSY held_set with them, not acted on),
    photo_no_set (PHOTO in the wait accepted, but no set started: OK detail 0), drop_free
    (PHOTO_FREE packets lost), kill_ack_lag (as auto_link.c: the ACK of the KILL that cuts carries the
    state before it, the state machine steps after the ACK), kill_ack_keep (only this many KILL ACKs
    are sent), reboot(). arm_air lists every ARM's airMaxMs.

    The interactive hover (ih, the default; ih=False: an A2b drone, 64-byte AUTO_STATUS, MOVE/YAW/STOP
    answered BAD_STATE): TAKEOFF heights 20..to_max_cm (120) cm and actWaitMs (0 = wait_ms, else 1000..120000, the
    sortie's wait; u24: byte 7 carries bits 16-23 since 2026-10-14; long_wait=False: a drone before that, whose
    parser refuses a nonzero byte 7 with RANGE detail 0, and whose range is 1000..60000); MOVE and YAW in the wait run for their profile time divided by mv_speed (the wait held
    while they run, restarted at their end), then DONE with the progress at the target; refusals as
    auto_motion.c (BUSY, BATT, RANGE, NO_DIST without scene_d, TOO_CLOSE, HEIGHT; NOT_WAITING), or the
    reason in refuse_mv (once each); STOP ends a running one STOPPED (progress where it was). The scene
    distance is scene_d (m, None: not valid)."""

    def __init__(self, guards=0, prespin=1.0, hover_at=2.5, capture_after=1.0, wait_ms=5000, land_s=1.0, rest_s=0.5,
                 fsm_photo_id=7, set_drop=(), busy_unknown=(), busy_state=None, bad_session_in_hover=False,
                 free_delay=0.0, lose_cmd=(), slow=None, stall=None, land_busy=0, mute_acks=(), mute_first=(),
                 held_reply=(), photo_no_set=False, drop_free=0, arm_timeout=10.0, kill_ack_lag=False,
                 kill_ack_keep=None, ih=True, scene_d=2.0, mv_speed=8.0, refuse_mv=(), batt_low=False,
                 mv_short=None, old_fw_zero=False, arm_guards=(), long_wait=True, air_hard_ms=None, long_moves=None,
                 plan_takeoff=None, vbat_mv=4050, mv_end=None, hr=False, drop_land=0, event_at_hover=None, sh=True,
                 h_max_cm=200, to_max_cm=120, sh_to=True, t_hold=46800, mv_pct=None, old136=False, mv_settle=0.0,
                 yaw_end=None, mv_floor=None, mv_klow=None, mv_lat=None, **kw):
        self.guards, self.prespin, self.hover_at, self.capture_after = guards, prespin, hover_at, capture_after
        self.arm_guards = list(arm_guards)     # the failed guards of the next ARMs, one each (then guards)
        self.wait_ms, self.land_s, self.rest_s, self.fsm_photo_id = wait_ms, land_s, rest_s, fsm_photo_id
        self.set_drop, self.busy_unknown = set(set_drop), set(busy_unknown)
        self.busy_state = dict(busy_state or {})
        self.bad_session_in_hover, self.free_delay, self.lose_cmd = bad_session_in_hover, free_delay, set(lose_cmd)
        self.slow, self.stall = dict(slow or {}), dict(stall or {})
        self.land_busy, self.mute_acks, self.drop_free = land_busy, set(mute_acks), drop_free
        self.mute_first, self.mute_seqs = set(mute_first), set()
        self.held_reply, self.photo_no_set, self.arm_timeout = list(held_reply), photo_no_set, arm_timeout
        self.kill_ack_lag, self.kill_ack_keep, self.kill_acks_sent = kill_ack_lag, kill_ack_keep, 0
        self.arm_air = []
        # the interactive hover
        self.ih, self.scene_d, self.mv_speed, self.refuse_mv = ih, scene_d, mv_speed, list(refuse_mv)
        # the guards down to off (2026-10-18, auto_motion.h "guards off"): the drone's mvfloor (m; 0 none; None: the
        # default DRONE_MIN_FRONT_M), its k (mvklow / 100; None: K_LOW) and its sideways cap x D (mvlat / 100; 0 none;
        # None: LAT_D_REL), as its ARM latched them
        self.mv_floor = auto.DRONE_MIN_FRONT_M if mv_floor is None else mv_floor
        self.mv_klow = auto.K_LOW if mv_klow is None else mv_klow
        self.mv_lat = auto.LAT_D_REL if mv_lat is None else mv_lat
        # mv_short: a MOVE's settle times out at this fraction of it (end SHORT, auto_motion.c); old_fw_zero: a
        # drone before 2026-10-13 evening (the progress shows while a MOVE runs and restarts at 0 after a STOP)
        self.mv_short, self.old_fw_zero = mv_short, old_fw_zero
        self.long_wait = long_wait
        # the height caps: a MOVE's end (AUTO_MV_H_MAX_M) 200 since 2026-10-15, 120 on a drone before; the TAKEOFF's
        # heightCm (AUTO_HEIGHT_CM_MAX) 120 (2.00 m by moves up only: vert_ctrl.h VZ_H_TAKEOFF_MAX)
        self.h_max_cm, self.to_max_cm = h_max_cm, to_max_cm
        # long moves (2026-10-14): the drone's MOVE limits (forward 400, the vector 400; False: 150 every axis, b6a54da)
        # and the 18-byte TAKEOFF with a planned move (False: the link refuses it RANGE detail 0). Default: the Mac's
        # build (MOVE_FWD_MAX_M 4.00: the long-move drone)
        self.long_moves = auto.MOVE_FWD_MAX_M > 1.5 if long_moves is None else long_moves
        self.plan_takeoff = self.long_moves if plan_takeoff is None else plan_takeoff
        self.plan, self.plan_pending, self.vbat_mv = (0, 0), False, vbat_mv
        self.mv_end_at = mv_end     # (end name, fraction): a MOVE ends so at this fraction of its target
        # AUTO_STATUS battLow bits (bit 0: the BATT_LOW condition held 0.25 s, the drone refuses MOVE / YAW; bit 1: now)
        self.batt_low_bits = auto.BATTLOW_HELD | auto.BATTLOW_NOW if batt_low else 0
        self.air_ms_offset = 0      # added to the air clock (late-sortie tests)
        self.air_hard_ms = auto.AIR_HARD_MS if air_hard_ms is None else air_hard_ms   # its AUTO_AIR_HARD_MS
        # hold and report (hr, 2026-10-15): the 136-byte AUTO_STATUS (airDs, no air clock: airLeftDs 0), no
        # HOVER_TIMEOUT (the wait is information); report() injects what the drone reports (its events, bits)
        self.hr, self.rep = hr, {}
        # drop_land: this many LAND packets lost (never processed, never answered); event_at_hover: (code, detail)
        # reported at the HOVER entry, before the wait (a contact in the climb ...: the plan is cancelled)
        self.drop_land, self.event_at_hover = drop_land, event_at_hover
        # SET_HEIGHT (2026-10-15, auto_fsm.c setHeightCmd): sh False: an older drone (the link answers the unknown
        # type BAD_STATE detail 0); h_mm the estimate in HOVER (AUTO_STATUS hMm); mv_corr: the latest motion is a
        # correction (mvFlags 0x40 on a MOVE)
        self.sh, self.h_mm, self.sh_ms, self.mv_corr = sh, 320, None, False
        # the target form (2026-10-16, a2b 27): sh_to False, a drone before it (7a78e1b, 8277b68): the link's length
        # check answers the 6-byte SET_HEIGHT RANGE detail 0 (not processed); t_hold the hover thrust (AUTO_STATUS
        # tHold; a climb refused UP_HEADROOM above 53400, the 58000 ceiling)
        self.sh_to, self.t_hold = sh_to, t_hold
        # fast moves (2026-10-16, a2b 31): mv_pct the drone's mvspeed in its 140-byte AUTO_STATUS (None: 100, or with
        # old136 a hold-and-report drone's 136 bytes, no speed step); mv_settle s: a MOVE settles (SETTLE) and a YAW
        # bobs (SETTLE with MVF_BOB) this long after its profile; yaw_end: a YAW's end after it (e.g. "UNDONE")
        self.mv_pct, self.old136, self.mv_settle, self.yaw_end = mv_pct, old136, mv_settle, yaw_end
        if mv_pct is not None:
            self.hr = True
        self.sh_lens = []           # the payload length of every SET_HEIGHT the FSM saw
        self.sh_cmds = []           # (xCm, atCm, d m) of every accepted SET_HEIGHT
        self.height_cm, self.h_off_cm, self.yaw_off_cdeg = 0, 0, 0
        self.mv_state = self.mv_kind = self.mv_why = self.mv_end = self.mv_seq = 0
        self.mv_t0 = self.mv_dur = 0.0
        self.mv_tgt, self.mv_prog, self.mv_yaw = (0, 0, 0), (0, 0, 0), 0
        self.t_air = None
        self.wait_ms0 = wait_ms
        self.pend = None            # (packet, until): a slow command pending
        self.frozen = None          # (until, fields): the snapshot a stalled stabilizer left
        self.last_kill = None
        self.lock = threading.RLock()
        self.t_boot = time.monotonic()
        self.state, self.mode, self.reason, self.vz_land = IDLE, 1, 0, 0
        self.t_state = self.t_boot
        self.t_idle = None
        self.transitions = 0
        self.token = None
        self.n, self.gap = 0, 300
        self.t_hover = self.t_wait = None
        self.waiting = False
        self.capture, self.cur_photo = 0, 0
        self.sets = {}
        self.pending_free = []
        self.kill_count = 0
        self.last_cmd = (0, 0)
        self.last_hb = None
        self.hb_lost = None
        self.next_push = 0.0
        self.log, self.kill_rx, self.timeline, self.lost = [], [], [], []
        self.hellos = self.photo_with_held = self.bad_sessions = 0
        self.accepted = collections.Counter()
        self.plan_moves = 0
        self.status_pushes = []     # (time, waiting, mvSeq) of every AUTO_STATUS pushed
        super().__init__(**kw)
        self.next_id = 20

    def run(self):
        self.sock.settimeout(0.002)
        while not self.stop_evt.is_set():
            try:
                data, src = self.sock.recvfrom(4096)
            except socket.timeout:
                data = None
            except OSError:
                break
            with self.lock:
                now = time.monotonic()
                if data is not None:
                    self.handle(data, src, now)
                self.tick(now)

    # -------- link

    def ack_packet(self, seq, ptype, result, detail, session, state=None):
        self.seq += 1
        return auto.encode(self.key, auto.T_ACK, 0, session, self.seq,
                           auto.ack_payload(seq, ptype, result, self.state if state is None else state, detail))

    def send(self, ptype, payload):
        if self.peer is not None and self.session:
            self.sock.sendto(self.packet(ptype, payload), self.peer)

    def handle(self, data, src, now=None):
        now = now or time.monotonic()
        try:
            pkt = auto.decode(self.key, data)
        except auto.ProtoError:
            self.auth_fails += 1
            return
        self.received.append(pkt)
        self.log.append((now, pkt, src))
        if pkt.type == auto.T_KILL:
            self.kill_rx.append((now, pkt, src))
            self.kill_count += 1
            self.last_kill = now
            before = self.state
            if self.state not in (IDLE, KILLED):
                self.to(KILLED, rs("KILL_CMD"), now)
            if pkt.flags & auto.FLAG_ACKREQ and (self.kill_ack_keep is None or self.kill_acks_sent < self.kill_ack_keep):
                self.kill_acks_sent += 1
                self.sock.sendto(self.ack_packet(pkt.seq, pkt.type, auto.R_OK, 0, pkt.session,
                                                 state=before if self.kill_ack_lag else None), src)
            return
        if pkt.type == auto.T_HELLO:
            self.hellos += 1
            self.peer = src
            (nonce,) = struct.unpack("<I", pkt.payload)
            self.new_session()
            self.send(auto.T_HELLO_ACK, auto.hello_ack_payload(self.session, self.boot, nonce, self.fp, "fake-a2b"))
            if self.state in (TAKEOFF, HOVER):
                self.to(LAND, rs("SESSION"), now)
            elif self.state == ARMED:
                self.to(IDLE, rs("SESSION"), now)
            return
        if not self.session or pkt.session != self.session:
            self.bad_sessions += 1
            self.sock.sendto(self.ack_packet(pkt.seq, pkt.type, auto.R_BAD_SESSION, 0, pkt.session), src)
            return
        self.peer = src
        if pkt.seq in self.recent:
            if pkt.seq in self.cache and pkt.seq not in self.mute_seqs and pkt.type not in self.mute_acks:
                self.sock.sendto(self.cache[pkt.seq], self.peer)
            return
        if self.last is not None and pkt.seq <= self.last:
            return
        if pkt.type == auto.T_LAND and self.drop_land:
            self.drop_land -= 1
            self.lost.append(pkt)
            return
        if pkt.type in self.lose_cmd:
            self.lose_cmd.discard(pkt.type)
            self.lost.append(pkt)
            return
        if pkt.type == auto.T_PHOTO_FREE and self.drop_free:
            self.drop_free -= 1
            self.lost.append(pkt)
            return
        self.last = pkt.seq
        self.recent.append(pkt.seq)
        self.accepted[pkt.type] += 1
        fsm_cmd = pkt.type in (auto.T_ARM, auto.T_TAKEOFF, auto.T_LAND, auto.T_DISARM, auto.T_MOVE, auto.T_YAW,
                               auto.T_STOP) or (pkt.type == auto.T_PHOTO and self.state != IDLE) or \
            (pkt.type == auto.T_SET_HEIGHT and self.sh)
        if fsm_cmd and self.pend is not None:      # one command at a time: BUSY pending, not cached
            self.answer(pkt, auto.R_BUSY, auto.busy_detail(auto.DETAIL_BUSY_PENDING, self.pend[0].seq), False)
            return
        if fsm_cmd and pkt.type in self.slow:
            self.pend = (pkt, now + self.slow.pop(pkt.type))
            return
        result, detail, cache, after = self.respond(pkt, now)
        self.answer(pkt, result, detail, cache)
        if after:
            after()

    def answer(self, pkt, result, detail, cache):
        if result is not None and pkt.flags & auto.FLAG_ACKREQ:
            ack = self.ack_packet(pkt.seq, pkt.type, result, detail, self.session)
            if cache:
                self.cache[pkt.seq] = ack
            if pkt.type in self.mute_first:
                self.mute_first.discard(pkt.type)
                self.mute_seqs.add(pkt.seq)
            if pkt.type in self.lose_first_ack:
                self.lose_first_ack.discard(pkt.type)
            elif pkt.type not in self.mute_acks and pkt.seq not in self.mute_seqs:
                self.sock.sendto(ack, self.peer)

    def respond(self, pkt, now):
        """(result, detail, cache the ACK, then) for a packet in our session with a new seq."""
        t, p = pkt.type, pkt.payload
        if t == auto.T_HB:
            self.last_hb = now
            return auto.R_OK, 0, True, None
        if t == auto.T_STATUS_REQ:
            def after():
                # STATUS upMs: the esp_timer clock, ESP_AHEAD_MS ahead of the AUTO_STATUS snapshots' tick (as the drone)
                self.send(auto.T_STATUS, auto.status_payload(STATUS_FIELDS | {
                    "bootNonce": self.boot, "state": self.state,
                    "upMs": (int((now - self.t_boot) * 1000) + ESP_AHEAD_MS) & 0xFFFFFFFF}))
                self.push_auto(now)
            return auto.R_OK, 0, True, after
        if t in self.stall:
            act, secs = self.stall.pop(t)
            self.frozen = (now + secs, self.snapshot(now))      # the stabilizer stalls in the step that took it
            if act:
                self.respond(pkt, now)
            return auto.R_BUSY, auto.DETAIL_TAKEN_UNKNOWN, False, None
        if self.busy_state.get(t, 0):
            self.busy_state[t] -= 1             # withdrawn untaken: never processed
            return auto.R_BUSY, auto.busy_detail(auto.DETAIL_BUSY_UNTAKEN, self.state), False, None
        if t == auto.T_PHOTO and self.held_reply:
            return auto.R_BUSY, auto.busy_detail(auto.DETAIL_BUSY_HELD_SET, self.held_reply.pop(0)), False, None
        if t == auto.T_ARM:
            mode, _, air = struct.unpack("<BBH", p)
            self.arm_air.append(air)
            if self.state == IDLE and self.last_kill is not None and now - self.last_kill < 2.0:
                r, d = auto.RESULT_NAMES.index("KILL_LATCHED"), self.state     # auto_fsm.c armCmd
            elif self.state != IDLE:
                r, d = auto.R_BAD_STATE, self.state
            elif mode > 2:
                r, d = 3, self.state
            elif (self.arm_guards[0] if self.arm_guards else self.guards):
                r, d = auto.R_GUARDS, self.arm_guards.pop(0) if self.arm_guards else self.guards
            else:
                if self.arm_guards:
                    self.arm_guards.pop(0)
                self.token = self.rng.getrandbits(32) | 1
                self.mode = mode
                self.to(ARMED, rs("ARM"), now)
                r, d = auto.R_OK, self.token
            self.last_cmd = (1, r)
            return r, d, True, None
        if t == auto.T_TAKEOFF:
            if len(p) == 18 and self.plan_takeoff:
                token, h, n, hi, gap, act, pr, pf, res = struct.unpack("<IHBBHHhhH", p)
                if res:
                    return 3, 0, True, None    # the reserved field: malformed (the link), not processed
            elif len(p) == 12:
                token, h, n, hi, gap, act = struct.unpack("<IHBBHH", p)
                pr = pf = 0
            else:
                return 3, 0, True, None        # an older drone and 18 bytes: the link's length check, RANGE detail 0
            if hi and not self.long_wait:
                self.last_cmd = (2, 3)
                return 3, 0, True, None        # an older drone: the link's parser refuses the pad, RANGE detail 0
            act |= hi << 16
            if self.state != ARMED:
                r, d = auto.R_BAD_STATE, self.state
            elif token != self.token:
                r, d = auto.R_BAD_STATE, 0
            elif not (20 <= h <= (self.to_max_cm if self.ih else 60) and 0 <= n <= 3 and 150 <= gap <= 1000 and
                      (act == 0 or (self.ih and 1000 <= act <= (120000 if self.long_wait else 60000)))):
                r, d = 3, self.state
            elif (pr or pf) and (self.mv_range_why(pr, pf, 0) or (act == 0 and self.wait_ms0 == 0)):
                r, d = 3, self.state           # auto_fsm.c takeoffCmd: the plan's limits, a wait to fly it in
            else:
                self.plan, self.plan_pending = (pr, pf), bool(pr or pf)
                self.n, self.gap = n, gap
                self.height_cm, self.h_off_cm, self.yaw_off_cdeg = h, 0, 0
                self.mv_state = self.mv_kind = self.mv_why = self.mv_end = self.mv_seq = 0
                self.wait_ms = act or self.wait_ms0
                self.to(TAKEOFF, rs("TAKEOFF"), now)
                r, d = auto.R_OK, self.state
            self.last_cmd = (2, r)
            if t in self.busy_unknown:
                self.busy_unknown.discard(t)
                return auto.R_BUSY, auto.DETAIL_TAKEN_UNKNOWN, False, None   # acted on, result lost
            return r, d, True, None
        if t == auto.T_LAND and self.land_busy:
            self.land_busy -= 1
            return auto.R_BUSY, auto.busy_detail(auto.DETAIL_BUSY_UNTAKEN, self.state), False, None
        if t in (auto.T_LAND, auto.T_DISARM):
            before = self.state
            why = rs("LAND_CMD" if t == auto.T_LAND else "DISARM_CMD")
            if before == ARMED or (before == TAKEOFF and now - self.t_state < self.prespin):
                self.to(IDLE, why, now)
            elif before in (TAKEOFF, HOVER):
                self.to(LAND, why, now)
            elif before == LAND and self.reason == rs("SINK"):
                self.reason = why           # auto_fsm.c sinkStep: the same LAND, never resumed
            self.last_cmd = (3 if t == auto.T_LAND else 5, auto.R_OK)
            return auto.R_OK, before, True, None
        if t == auto.T_PHOTO:
            n, _, gap = struct.unpack("<BBH", p)
            if self.state == IDLE:
                if self.sets:
                    return auto.R_BUSY, auto.busy_detail(auto.DETAIL_BUSY_HELD_SET, min(self.sets)), False, None
                sid = self.next_id
                self.next_id += 1
                self.start_set(sid, n, gap, now, fsm=False)
                return auto.R_OK, sid, True, None
            held = [sid for sid, s in self.sets.items() if s["fsm"]]
            if held:
                self.photo_with_held += 1
                return auto.R_BUSY, auto.busy_detail(auto.DETAIL_BUSY_HELD_SET, held[0]), False, None   # never posted
            if self.state == HOVER and self.waiting and 1 <= n <= 3 and 150 <= gap <= 1000 and self.photo_no_set:
                self.capture, self.last_cmd = 3, (6, auto.R_OK)     # the camera could not start it
                return auto.R_OK, 0, True, None
            if self.state == HOVER and self.waiting and 1 <= n <= 3 and 150 <= gap <= 1000:
                sid = self.next_id
                self.next_id += 1
                self.start_set(sid, n, gap, now, fsm=True)
                self.last_cmd = (6, auto.R_OK)
                return auto.R_OK, sid, True, None
            self.last_cmd = (6, auto.R_BAD_STATE)
            return auto.R_BAD_STATE, self.state, True, None
        if t == auto.T_CHUNKS:
            pid, idx, count, first = struct.unpack("<HBBH", p)
            s = self.sets.get(pid)
            if s is None:
                return auto.R_NO_PHOTO, 0, True, None
            if idx >= s["n"] or count > 8 or \
                    (first != 0 if count == 0 else first + count > auto.chunk_count(len(self.frames[idx]))):
                return 3, 0, True, None
            if idx not in s["pushed"]:
                return auto.R_BUSY, 0, True, None
            frame = self.frames[idx]
            if count == 0:
                return auto.R_OK, 0, True, lambda: self.send(auto.T_META, auto.meta_payload(
                    meta_for(frame, pid, idx, s["n"], captureUpMs=self.cap_ms(s, idx), pixFmt=self.pixfmt)))
            parts = chunks_of(frame)

            def after():
                for ci in range(first, first + count):
                    self.send(auto.T_CHUNK, auto.chunk_payload(pid, idx, ci, parts[ci]))
            return auto.R_OK, 0, True, after
        if t == auto.T_PHOTO_FREE:
            (pid,) = struct.unpack("<H", p)
            if self.free_delay:
                self.pending_free.append((now + self.free_delay, pid, pkt))
                return None, 0, False, None
            return self.free(pid), pid, True, None
        if self.ih and t in (auto.T_MOVE, auto.T_YAW, auto.T_STOP):
            return self.motion(t, p, now) + (True, None)
        if t == auto.T_SET_HEIGHT:
            if not self.sh:
                return auto.R_BAD_STATE, 0, True, None     # an older drone: the link's unknown type, detail 0
            if len(p) not in (4, 6) or (len(p) == 6 and not self.sh_to):
                return auto.R_RANGE, 0, True, None         # the link's length check: not processed
            return self.set_height(p, now) + (True, None)
        return auto.R_BAD_STATE, self.state, True, None

    def set_height(self, p, now):
        """(result, detail) of a SET_HEIGHT, as auto_fsm.c setHeightCmd (the checks in its order, the detail state |
        why << 8 | the old estimate cm << 16); accepted: the estimate shifted, the reference + d, a correction of -d."""
        W = auto.MV_WHY_NAMES.index
        self.sh_lens.append(len(p))
        x, at = struct.unpack("<hh", p[:4])
        to = struct.unpack("<h", p[4:6])[0] if len(p) == 6 else None
        old = self.h_mm / 1000.0
        d = (x - at) / 100.0 if at != auto.SH_AT_NOW else x / 100.0 - old
        new = old + d
        if to is None:
            up = -d
        else:
            tgt = (self.height_cm + self.h_off_cm) / 100.0 if to == auto.SH_TO_REF else to / 100.0
            up = tgt - new
        why = 0
        if not (self.state == HOVER and self.waiting):
            why = W("NOT_WAITING")
        elif self.mv_active():
            why = W("BUSY")
        elif self.sh_ms is not None and now - self.sh_ms < auto.SH_GAP_S:
            why = W("SH_RATE")
        elif not round(auto.SH_X_M[0] * 100) <= x <= round(auto.SH_X_M[1] * 100) or \
                (round(d * 100) == 0 and (to is None or round(up * 100) == 0)) or \
                (to is not None and to != auto.SH_TO_REF and not 25 <= to <= 200):
            why = W("RANGE")
        elif abs(d) > auto.SH_MAX_M + 1e-4 or abs(up) > auto.SH_FLY_MAX_M + 1e-4:
            why = W("SH_STEP")
        elif up > 0 and self.t_hold > 53400:
            why = W("UP_HEADROOM")
        elif self.refuse_mv:
            why = W(self.refuse_mv.pop(0))
        detail = self.state | why << 8 | (int(round(old * 100)) & 0xFFFF) << 16
        if why:
            self.mv_why = why
            res = {W("RANGE"): 3, W("SH_STEP"): 3, W("HEIGHT"): 3, W("NO_CAM_HEIGHT"): 5, W("NO_VIEW"): 5,
                   W("SH_CONTACT"): 5, W("BATT"): 6, W("UP_HEADROOM"): 6}.get(why, auto.R_BAD_STATE)
            self.last_cmd = (10, res)
            return res, detail
        self.sh_cmds.append((x, at, d) if to is None else (x, at, d, to))
        self.sh_ms = now
        self.h_mm += int(round(d * 1000))
        if to is None:
            self.h_off_cm += int(round(d * 100))
        else:
            self.h_off_cm = int(round(new * 100)) - self.height_cm   # the reference at the corrected estimate
        self.mv_seq += 1
        self.mv_kind, self.mv_tgt, self.mv_prog, self.mv_yaw, self.mv_corr = 1, (0, 0, int(round(up * 100))), \
            (0, 0, 0), 0, True
        self.mv_dur = abs(up) / 0.15 / self.mv_speed + 0.1
        self.mv_why, self.mv_end, self.mv_state, self.mv_t0 = 0, 0, auto.MV_RUN, now
        self.t_wait = now
        self.last_cmd = (10, auto.R_OK)
        self.push_auto(now)
        return auto.R_OK, detail

    # -------- the interactive hover (auto_motion.c, auto_fsm.c motionCmd)

    def mv_active(self):
        return self.mv_state in auto.MV_ACTIVE

    def motion(self, t, p, now, floor=None):
        """(result, detail) of a MOVE, YAW or STOP, as auto_fsm.c motionCmd / auto_motion.c (floor: the forward floor,
        the drone's mvfloor DRONE_MIN_FRONT_M by default (0.50 until 2026-10-17); 0.35 for the TAKEOFF's plan:
        AUTO_PLAN_MIN_FRONT_CM)."""
        floor = self.mv_floor if floor is None else floor
        W = auto.MV_WHY_NAMES.index
        detail = lambda code: self.state | code << 8 | self.mv_seq << 16     # noqa: E731 (auto_fsm.c mvDetail)
        if t == auto.T_STOP:
            found = self.mv_state
            if self.mv_active():
                frac = min(1.0, (now - self.mv_t0) / self.mv_dur) if self.mv_dur else 1.0
                self.mv_prog = tuple(int(v * frac) for v in self.mv_tgt)
                self.mv_end_now(auto.MV_STOPPED, 2, now)
                if self.old_fw_zero:
                    self.mv_prog = (0, 0, 0)        # the snap epoch's progress
                    self.push_auto(now)
            if self.waiting:
                self.t_wait = now
            self.last_cmd = (9, auto.R_OK)
            return auto.R_OK, detail(found)
        cmd = 7 if t == auto.T_MOVE else 8
        if not (self.state == HOVER and self.waiting):
            why = W("NOT_WAITING")
        elif self.mv_active():
            why = W("BUSY")
        elif self.batt_low_bits & auto.BATTLOW_HELD:
            why = W("BATT")
        elif self.refuse_mv:
            why = W(self.refuse_mv.pop(0))
        elif t == auto.T_MOVE:
            r, f, u, _ = struct.unpack("<hhhH", p)
            why = 0
            if self.mv_range_why(r, f, u):
                why = W("RANGE")
            elif (r or f) and self.scene_d is None:
                why = W("NO_DIST")
            elif r and self.mv_lat and abs(r) > self.mv_lat * 100 * self.scene_d + 0.01:
                why = W("TOO_WIDE")
            elif f > 0 and (self.scene_d - f / 100.0 < auto.MOVE_D_FLOOR_M - 1e-9 or
                            auto.fwd_end_low(self.scene_d, f / 100.0, self.mv_klow) < floor - 1e-9):
                why = W("TOO_CLOSE")
            elif u and not 25 <= self.height_cm + self.h_off_cm + u <= self.h_max_cm:
                why = W("HEIGHT")
            if not why:
                if getattr(self, "scene_d_accept", None) is not None:   # the drone's D moved by the drift (mvfix DRIFT)
                    self.scene_d, self.scene_d_accept = self.scene_d_accept, None
                self.mv_corr = False
                self.mv_kind, self.mv_tgt, self.mv_prog, self.mv_yaw = 1, (r, f, u), (0, 0, 0), 0
                self.mv_dur = (math.hypot(r, f) / (auto.MOVE_V_H_MAX * 100.0) + abs(u) / 15.0) / self.mv_speed + 0.1
        else:
            (deg, _) = struct.unpack("<hH", p)
            why = W("RANGE") if deg == 0 or abs(deg) > 180 else 0
            if not why:
                self.mv_corr = False
                self.mv_kind, self.mv_tgt, self.mv_prog, self.mv_yaw = 2, (0, 0, 0), (0, 0, 0), deg
                self.mv_dur = abs(deg) / auto.YAW_RATE_DPS / self.mv_speed + 0.1
        if why:
            self.mv_why = why
            self.last_cmd = (cmd, 1)
            res = {W("RANGE"): 3, W("HEIGHT"): 3, W("TOO_CLOSE"): 3, W("AIR"): 3, W("NO_DIST"): 5, W("NO_VIEW"): 5,
                   W("NO_CAM_HEIGHT"): 5, W("BATT"): 6}.get(why, auto.R_BAD_STATE)
            self.last_cmd = (cmd, res)
            return res, detail(why)
        self.mv_seq += 1
        self.mv_why, self.mv_end, self.mv_state, self.mv_t0 = 0, 0, auto.MV_RUN, now
        self.t_wait = now
        self.last_cmd = (cmd, auto.R_OK)
        self.push_auto(now)
        return auto.R_OK, detail(0)

    def mv_range_why(self, r, f, u):
        """auto_motion.c autoMvRangeOk (the long-move limits; b6a54da: 150 cm on every axis)."""
        if not self.long_moves:
            return max(abs(r), abs(f)) > 150 or abs(u) > 50 or r == f == u == 0
        return (abs(r) > 150 or f > 400 or f < -150 or abs(u) > 50 or r == f == u == 0 or r * r + f * f > 160000 or
                (u != 0 and f > 150))

    def mv_end_now(self, state, end, now):
        self.mv_state, self.mv_end, self.mv_t0 = state, end, now
        if self.mv_kind == 1:
            self.h_off_cm += self.mv_prog[2]
            if self.mv_corr:
                self.h_mm += self.mv_prog[2] * 10   # the correction flown: the estimate back where it was held
        elif self.mv_kind == 2:
            frac = 1.0 if state == auto.MV_DONE else 0.5
            self.yaw_off_cdeg = (self.yaw_off_cdeg + int(self.mv_yaw * 100 * frac) + 18000) % 36000 - 18000
        self.push_auto(now)

    def start_plan(self, now):
        """auto_fsm.c waitStart: the TAKEOFF's planned move as a MOVE at the wait's first step (refused: mvWhy)."""
        if self.plan_pending:
            self.plan_pending = False
            if self.hr and self.rep.get("evCount"):
                self.mv_why = auto.MV_WHY_NAMES.index("PLAN_CANCELLED")   # auto_fsm.c planCancel: an event before
                return                                                    # the wait
            self.plan_moves += 1
            self.motion(auto.T_MOVE, struct.pack("<hhhH", self.plan[0], self.plan[1], 0, 0), now,
                        floor=auto.PLAN_MIN_FRONT_M if self.mv_floor > 0 else 0.0)

    def free(self, pid):
        if self.sets.pop(pid, None) is None:
            return auto.R_NO_PHOTO
        self.freed.append(pid)
        return auto.R_OK

    # -------- FakeAutonomy

    def to(self, state, reason, now):
        if self.state == HOVER and state != HOVER and self.mv_active():
            self.mv_state, self.mv_end = auto.MV_STOPPED, 4         # LEFT_HOVER
        if state == TAKEOFF:
            self.t_air = now
        elif state in (IDLE, KILLED):
            self.t_air = None
        self.state, self.reason = state, reason
        self.t_state = now
        self.transitions += 1
        self.timeline.append((now, auto.STATE_NAMES[state], auto.REASON_NAMES[reason]))
        if state == HOVER:
            self.t_hover = now
        if state != HOVER:
            self.waiting = False
        if state == IDLE:
            self.t_idle = now
        if state == ARMED:
            self.capture, self.cur_photo = 0, 0
            self.sh_ms, self.mv_corr, self.h_mm = None, False, 320
            if self.hr:
                self.rep = {}               # the report is this sortie's (auto_fsm.c armReset)
        if state == HOVER and self.event_at_hover:
            self.report(*self.event_at_hover)
        self.push_auto(now)
        if state == LAND and reason == rs("HB_LOST"):
            self.hb_lost = (self.last_hb, now)
        if state == HOVER and self.bad_session_in_hover:
            self.new_session()          # another HELLO replaced ours
            self.to(LAND, rs("SESSION"), now)

    def start_set(self, sid, n, gap, now, fsm):
        self.sets[sid] = {"n": n, "gap": gap, "t0": now, "pushed": set(), "fsm": fsm}
        if fsm:
            self.capture, self.cur_photo = 1, sid

    def cap_ms(self, s, idx):
        """A frame's captureUpMs: its capture on the esp_timer clock (ESP_AHEAD_MS ahead of the snapshots' tick)."""
        return (int((s["t0"] + idx * s["gap"] / 1000.0 - self.t_boot) * 1000) + ESP_AHEAD_MS) & 0xFFFFFFFF

    def push_frame(self, sid, s, idx):
        frame = self.frames[idx]
        if (idx, "meta") not in self.set_drop:
            self.send(auto.T_META, auto.meta_payload(meta_for(frame, sid, idx, s["n"], captureUpMs=self.cap_ms(s, idx),
                                                              pixFmt=self.pixfmt)))
        for ci, part in enumerate(chunks_of(frame)):
            if (idx, ci) not in self.set_drop:
                self.send(auto.T_CHUNK, auto.chunk_payload(sid, idx, ci, part))

    def tick(self, now):
        st, dt = self.state, now - self.t_state
        if st in (ARMED, TAKEOFF, HOVER) and self.last_hb is not None and now - self.last_hb > 1.0:
            if st == ARMED or (st == TAKEOFF and dt < self.prespin):
                self.to(IDLE, rs("HB_LOST"), now)
            else:
                self.to(LAND, rs("HB_LOST"), now)
        st, dt = self.state, now - self.t_state
        if st == ARMED and dt >= self.arm_timeout:
            self.to(IDLE, rs("ARM_TIMEOUT"), now)
        elif st == TAKEOFF and dt >= self.hover_at:
            self.to(HOVER, rs("HOLD"), now)
        elif st == HOVER:
            if self.mv_active():
                self.t_wait = now                   # the wait does not count down while a motion runs
                if self.old_fw_zero and self.mv_kind == 1 and self.mv_dur:
                    frac = min(1.0, (now - self.mv_t0) / self.mv_dur)
                    self.mv_prog = tuple(int(v * frac) for v in self.mv_tgt)
                if now - self.mv_t0 >= self.mv_dur and self.mv_state == auto.MV_RUN and self.mv_settle > 0:
                    # the profile has ended: a MOVE settles at its target, a YAW bobs (fast moves' early return)
                    self.mv_state, self.mv_t0, self.mv_dur = auto.MV_SETTLE, now, self.mv_settle
                    if self.mv_kind == 1:
                        self.mv_prog = self.mv_tgt
                    self.push_auto(now)
                elif now - self.mv_t0 >= self.mv_dur:
                    if self.mv_kind == 2 and self.yaw_end is not None:
                        self.mv_end_now(auto.MV_DONE, auto.MV_END_NAMES.index(self.yaw_end), now)
                    elif self.mv_end_at is not None and self.mv_kind == 1:
                        name, frac = self.mv_end_at[:2]
                        off = self.mv_end_at[2] if len(self.mv_end_at) > 2 else (0, 0, 0)   # cm off its line
                        self.mv_prog = tuple(int(round(v * frac)) + o for v, o in zip(self.mv_tgt, off))
                        self.mv_end_now(auto.MV_STOPPED if name not in ("SHORT", "LONG") else auto.MV_DONE,
                                        auto.MV_END_NAMES.index(name), now)
                    elif self.mv_short is not None and self.mv_kind == 1:
                        self.mv_prog = tuple(int(round(v * self.mv_short)) for v in self.mv_tgt)
                        self.mv_end_now(auto.MV_DONE, auto.MVE_SHORT, now)
                    else:
                        self.mv_prog = self.mv_tgt
                        self.mv_end_now(auto.MV_DONE, auto.MVE_DONE, now)
            if not self.cur_photo and not self.waiting and now - self.t_hover >= self.capture_after:
                if self.n:
                    self.start_set(self.fsm_photo_id, self.n, self.gap, now, fsm=True)
                else:
                    self.waiting, self.t_wait = True, now
                    self.start_plan(now)
            if self.waiting and (now - self.t_wait) * 1000.0 >= self.wait_ms and not self.hr:
                self.to(LAND, rs("HOVER_TIMEOUT"), now)
        elif st == LAND and dt >= self.land_s + self.rest_s and self.reason != rs("SINK"):   # (a SINK: until a LAND)
            self.to(IDLE, rs("TOUCHDOWN"), now)
        elif st == KILLED and dt >= 2.0:
            self.to(IDLE, rs("KILL_CLEARED"), now)
        for sid, s in list(self.sets.items()):
            for idx in range(s["n"]):
                if idx not in s["pushed"] and now >= s["t0"] + idx * s["gap"] / 1000.0:
                    s["pushed"].add(idx)
                    self.push_frame(sid, s, idx)
            if s["fsm"] and sid == self.cur_photo and self.capture == 1 and len(s["pushed"]) == s["n"]:
                self.capture = 2
                if self.state == HOVER and not self.waiting:
                    self.waiting, self.t_wait = True, now
                    self.start_plan(now)
        if self.pend is not None and now >= self.pend[1]:
            pkt, self.pend = self.pend[0], None
            result, detail, cache, after = self.respond(pkt, now)
            self.answer(pkt, result, detail, cache)
            if after:
                after()
        for item in [i for i in self.pending_free if now >= i[0]]:
            self.pending_free.remove(item)
            _, pid, pkt = item
            ack = self.ack_packet(pkt.seq, pkt.type, self.free(pid), pid, self.session)
            self.cache[pkt.seq] = ack
            if self.peer is not None:
                self.sock.sendto(ack, self.peer)
        if self.session and self.peer is not None and now >= self.next_push:
            if self.state != IDLE:
                self.push_auto(now)
            elif self.t_idle is not None and now - self.t_idle <= 2.0:
                self.push_auto(now, period=0.5)

    def push_auto(self, now, period=0.1):
        self.next_push = now + period
        f = self.snapshot(now)
        if self.frozen is not None:
            if now < self.frozen[0]:
                f = self.frozen[1]          # a stalled stabilizer publishes nothing: the link pushes its last
            else:
                self.frozen = None
        self.status_pushes.append((now, bool(f["flags"] & 1), f.get("mvSeq", 0)))
        self.send(auto.T_AUTO_STATUS, auto.auto_status_payload(f, auto.AUTO_STATUS_LEN_A2B if not self.ih else
                                                                auto.AUTO_STATUS_LEN_HR if self.hr and self.old136
                                                                else auto.AUTO_STATUS_LEN if self.hr else
                                                                auto.AUTO_STATUS_LEN_IH))

    def snapshot(self, now):
        st = self.state
        prespin = st == TAKEOFF and now - self.t_state < self.prespin
        phase = {TAKEOFF: 0 if prespin else 2, HOVER: 4, KILLED: 7}.get(st, 0)
        if st == LAND:
            phase = 6 if now - self.t_state >= self.land_s else 5
        f = {"upMs": int((now - self.t_boot) * 1000) & 0xFFFFFFFF, "bootNonce": self.boot,
             "transitions": self.transitions, "failedGuards": self.guards, "state": st, "mode": self.mode,
             "reason": self.reason, "vzLandReason": 0, "vzPhase": phase,
             "flags": (1 if self.waiting else 0) | (2 if st == HOVER else 0) | (0x20 if prespin else 0) |
                      (0x40 if self.mode != 0 else 0),
             "trips": 0x40 if st in (IDLE, ARMED) else 0, "capture": self.capture, "photoId": self.cur_photo,
             "stateMs": min(65535, int((now - self.t_state) * 1000)),
             "waitLeftMs": min(65535, max(0, int(self.wait_ms - (now - self.t_wait) * 1000))) if self.waiting else 0,
             "hoverWaitMs": min(65535, self.wait_ms), "vbatMv": self.vbat_mv, "vbatFastMinMv": self.vbat_mv - 40,
             "hMm": self.h_mm if st == HOVER else 0, "tHold": self.t_hold if st == HOVER else 0,
             "thrust": 41000 if st in (TAKEOFF, HOVER) and not prespin else 0,
             "kPriorDiv10": 16500, "khatDiv10": 16500, "lastCmdType": self.last_cmd[0],
             "lastCmdResult": self.last_cmd[1], "killCount": self.kill_count}
        air = int((now - self.t_air) * 1000) + self.air_ms_offset if self.t_air is not None else 0
        f["airMs"] = min(65535, air)
        if self.ih:
            d = self.scene_d
            f.update({"mvState": self.mv_state, "mvKind": self.mv_kind, "mvWhy": self.mv_why, "mvEnd": self.mv_end,
                      "mvFlags": (0 if d is None else auto.MVF_D_VALID) |
                                 (auto.MVF_SET_HEIGHT if self.mv_corr and self.mv_kind == 1 else 0) |
                                 (auto.MVF_BOB if self.mv_kind == 2 and self.mv_state == auto.MV_SETTLE else 0) |
                                 (auto.MVF_D_TRACK if getattr(self, "d_track", False) and d is not None else 0),
                      "battLow": self.batt_low_bits,
                      "mvSeq": self.mv_seq, "progRightCm": self.mv_prog[0], "progFwdCm": self.mv_prog[1],
                      "progUpCm": self.mv_prog[2], "tgtRightCm": self.mv_tgt[0], "tgtFwdCm": self.mv_tgt[1],
                      "tgtUpCm": self.mv_tgt[2], "sceneDCm": 0 if d is None else int(round(d * 100)),
                      "hTargetCm": self.height_cm + self.h_off_cm if self.t_air is not None else 0,
                      "yawOffCdeg": self.yaw_off_cdeg, "yawTgtDeg": self.mv_yaw,
                      "mvPhaseMs": min(65535, int((now - self.mv_t0) * 1000)) if self.mv_state else 0,
                      "airLeftDs": max(0, self.air_hard_ms - air) // 100 if self.t_air is not None else 0})
        if self.mv_pct is not None:
            f["mvSpeedPct"] = self.mv_pct
        if self.hr:
            f["airLeftDs"] = 0
            f["airDs"] = min(65535, air // 100)
            f["sinceCmdDs"] = 65535
            f.update(self.rep)
        return f

    def report(self, code, detail=0, **fields):
        """hr: the drone reports an event (the newest; the two before move down) with these report fields."""
        with self.lock:
            r = self.rep
            r["ev2Code"], r["ev2Detail"] = r.get("ev1Code", 0), r.get("ev1Detail", 0)
            r["ev1Code"], r["ev1Detail"] = r.get("evCode", 0), r.get("evDetail", 0)
            r["evCode"], r["evDetail"] = code, detail
            r["evCount"] = (r.get("evCount", 0) + 1) & 0xFFFF
            r.update(fields)

    def reboot(self):
        """Call with self.lock held: a new boot, no session, IDLE."""
        self.boot ^= 0x22222222
        self.session, self.peer, self.last = 0, None, None
        self.recent.clear()
        self.cache.clear()
        self.state, self.sets, self.last_hb, self.t_idle = IDLE, {}, None, None

    def got(self, ptype, since=0.0):
        return [(t, p) for t, p, _ in list(self.log) if p.type == ptype and t >= since]


class DaemonCase(SandboxedTest):
    """An in-process daemon (its loop in a thread) on a FakeAutoDrone; clients through main()."""

    def drone(self, **kw):
        d = FakeAutoDrone(**kw)
        self.addCleanup(d.stop)
        return d

    def start_daemon(self, d, allow_flight=False, keys=False, name="d", out=None, pre=None):
        self.keyf = self.key_file()
        self.sock = os.path.join(self.tmp.name, name + ".sock")
        self.photos = os.path.join(self.tmp.name, "photos")
        r = w = None
        if keys:
            r, w = os.pipe()
            self.addCleanup(os.close, w)
            self.addCleanup(os.close, r)
        dm = auto.Daemon(auto.Link(KEY, "127.0.0.1", d.port), self.sock, self.photos, allow_flight=allow_flight,
                         keys_fd=r, events_path=os.path.join(self.tmp.name, name + "-events.jsonl"),
                         out=io.StringIO() if out is None else out, host="127.0.0.1")
        dm.start()
        if pre is not None:
            pre(dm)
        th = threading.Thread(target=dm.run, daemon=True)
        th.start()

        def stop():
            dm.request_stop()
            th.join(3)
        self.addCleanup(stop)
        self.dm, self.thread = dm, th
        return dm, w

    def cli(self, *argv):
        return self.run_main([*argv, "--sock", self.sock])

    def oneshot(self, d, *argv):
        return self.run_main([*argv, "--host", "127.0.0.1", "--port", str(d.port), "--key-file", self.keyf,
                              "--sock", self.sock])

    def events(self, name):
        return [e for e in list(self.dm.events) if e["event"] == name]

    def sortie_to_wait(self, *extra):
        code, out, _, err = self.cli("sortie", "--mode", "dry", *extra)
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        self.assertEqual((out["state"], out["waiting"]), ("HOVER", True))
        return out


FAST = dict(prespin=0.3, hover_at=0.7, capture_after=0.3, land_s=0.3, rest_s=0.2)


class DaemonSortieTest(DaemonCase):
    def test_sortie_dry_photos_in_the_wait_then_land(self):
        """12.4 #4: the take-off photos are on disk while waiting (before LAND); land; IDLE TOUCHDOWN."""
        d = self.drone(**dict(FAST, wait_ms=3000), set_drop={(0, 4), (1, "meta")})
        self.start_daemon(d)
        out = self.sortie_to_wait("--height", "40", "--photos", "2", "--gap", "300")
        self.assertEqual((out["photo_id"], out["reason"], out["mode"]), (7, "HOLD", "DRY"))
        self.assertTrue(0 < out["waitLeftMs"] <= 3000, out["waitLeftMs"])
        self.assertEqual([p["idx"] for p in out["photos"]], [0, 1])
        for p in out["photos"]:
            for k in ("png", "png_x4", "json"):
                self.assertTrue(os.path.exists(p[k]), p[k])
            with open(p["json"]) as f:
                doc = json.load(f)
            self.assertEqual((doc["id"], doc["autonomy"]["source"], doc["autonomy"]["state"]), (7, "drone", "HOVER"))
            self.assertEqual(doc["crc32"], zlib.crc32(d.frames[p["idx"]]))
        self.assertTrue(d.got(auto.T_CHUNKS), "the dropped chunk and META were asked for again")
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))
        code, out, _, _ = self.cli("wait", "--until", "idle", "--timeout", "5")
        self.assertEqual((code, out["state"], out["reason"]), (0, "IDLE", "TOUCHDOWN"))
        land_t = next(e["t"] for e in self.events("state") if e["to"] == "LAND")
        photo_t = [e["t"] for e in self.events("photo")]
        self.assertEqual(len(photo_t), 2)
        self.assertLess(max(photo_t), land_t)
        self.assertIn((LAND, "LAND_CMD"), [(auto.STATE_NAMES.index(s), r) for _, s, r in d.timeline])
        self.assertTrue(wait_for(lambda: 7 in d.freed, 2))
        self.assertEqual(d.hellos, 1)
        code, out, _, _ = self.cli("events")        # every event keeps its number (photo_set's frame count is 'frames')
        self.assertEqual(code, 0, out)
        self.assertEqual([e["n"] for e in out["events"]], list(range(len(out["events"]))))
        self.assertEqual([e["frames"] for e in out["events"] if e["event"] == "photo_set"], [None])

    def test_sortie_colour_photos(self):
        """A photocol 1 drone's take-off photos: saved as RGB PNGs, 'color' true in the photo reply, event and JSON;
        a dropped chunk past the grey frame's 19 is asked for again."""
        from PIL import Image
        d = self.drone(**dict(FAST, wait_ms=3000), pixfmt=auto.PIXFMT_YUYV, set_drop={(0, 33)})
        self.start_daemon(d)
        out = self.sortie_to_wait("--height", "40", "--photos", "2", "--gap", "300")
        self.assertEqual([p["idx"] for p in out["photos"]], [0, 1])
        for p in out["photos"]:
            self.assertIs(p["color"], True)
            with Image.open(p["png"]) as im:
                self.assertEqual(im.mode, "RGB")
                self.assertEqual(im.tobytes(), auto.flip_rows(auto.yuyv_to_rgb(d.frames[p["idx"]], 160, 120), 480,
                                                              120))
            with open(p["json"]) as f:
                doc = json.load(f)
            self.assertEqual((doc["color"], doc["pixFmtName"], doc["crc32"]),
                             (True, "yuyv422", zlib.crc32(d.frames[p["idx"]])))
        self.assertEqual([e["color"] for e in self.events("photo")], [True, True])
        self.assertIn((7, 0, 1, 33), [struct.unpack("<HBBH", pk.payload) for _, pk in d.got(auto.T_CHUNKS)])
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))

    def test_sortie_then_land(self):
        d = self.drone(**dict(FAST, wait_ms=5000))
        self.start_daemon(d)
        code, out, _, err = self.cli("sortie", "--mode", "dry", "--photos", "1", "--gap", "150", "--then", "land")
        self.assertEqual((code, out["ok"]), (0, True), err)
        self.assertEqual(out["final"], {"state": "IDLE", "reason": "TOUCHDOWN", "vzLandReason": "NONE"})
        self.assertEqual(len(out["photos"]), 1)
        self.assertEqual(out["waiting"]["state"], "HOVER")
        self.assertEqual([r for _, s, r in d.timeline if s == "LAND"], ["LAND_CMD"])

    def test_sortie_refused_arm_shows_guards_and_vmin(self):
        d = self.drone(guards=(1 << 3) | (1 << 10))
        dm, _ = self.start_daemon(d)
        self.assertTrue(wait_for(lambda: dm.auto is not None, 2))
        os.makedirs(auto.DEFAULT_DIR, exist_ok=True)
        with open(auto.vmin_cache_path(), "w") as f:
            json.dump({"utc": "2026-10-12T10:00:00Z", "values": {"vmin": {"value": "4000", "src": "nvs"}}}, f)
        code, out, _, _ = self.cli("arm", "--mode", "bench")
        self.assertEqual((code, out["result"], out["guards"]), (1, "GUARDS", ["BATT", "GESTURE"]))
        self.assertEqual((out["vmin"], out["vbat"]), (4.0, 4.05))
        code, out, _, _ = self.cli("sortie", "--mode", "dry")
        self.assertEqual((code, out["result"], out["arm"]["guards"]), (1, "GUARDS", ["BATT", "GESTURE"]))
        self.assertFalse(d.got(auto.T_TAKEOFF))

    def test_photo_while_waiting_waits_for_the_free(self):
        """12.4 #8: PHOTO in the wait is OK with the id; a second one waits for PHOTO_FREE of the first."""
        d = self.drone(**dict(FAST, wait_ms=8000), free_delay=0.15)
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        replies = []

        def ask():
            replies.append(auto.daemon_request(self.sock, {"op": "photo", "n": 1, "gap": 150, "timeout": 8}, 12))
        threads = [threading.Thread(target=ask) for _ in range(2)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(15)
        self.assertEqual(len(replies), 2)
        for r in replies:
            self.assertTrue(r["ok"], r)
            self.assertEqual((len(r["photos"]), r["state"]), (1, "HOVER"))
            self.assertTrue(os.path.exists(r["photos"][0]["png"]))
        ids = sorted(r["id"] for r in replies)
        self.assertEqual(len(set(ids)), 2)
        payloads = {p.payload for _, p in d.got(auto.T_PHOTO)}
        self.assertEqual(d.photo_with_held, 0, "a PHOTO reached the drone while a set was held")
        self.assertTrue(wait_for(lambda: set(ids) | {7} <= set(d.freed), 2), d.freed)
        self.assertEqual(payloads, {auto.photo_payload(1, 150)})
        self.assertEqual(d.state, HOVER)

    def test_photo_ids_of_the_old_boot_are_forgotten_after_a_reboot(self):
        """2026-10-22: the drone's photo set ids start at 1 again after a reboot. A set id saved in the old boot
        (the take-off photo 7, the 'photo' request's 20) asked for again in the new boot captures and saves a new
        frame (its JSON carries the new bootNonce); until then done_sets / freed_ids outlived the reboot and the
        old boot's files were returned (events.jsonl 11:40-11:52: ids 2 and 3)."""
        d = self.drone(**dict(FAST, wait_ms=8000))
        dm, _ = self.start_daemon(d)
        boot_a = f"{d.boot:08x}"
        out_a = self.sortie_to_wait("--photos", "1", "--gap", "150")
        self.assertEqual([p["id"] for p in out_a["photos"]], [7])
        r_a = auto.daemon_request(self.sock, {"op": "photo", "n": 1, "gap": 150, "timeout": 8}, 12)
        self.assertEqual((r_a["ok"], r_a["id"], len(r_a["photos"])), (True, 20, 1), r_a)
        for p in (out_a["photos"][0], r_a["photos"][0]):
            with open(p["json"]) as f:
                self.assertEqual(json.load(f)["link"]["bootNonce"], boot_a)
        self.assertEqual(self.cli("land")[0], 0)
        code, out, _, _ = self.cli("wait", "--until", "idle", "--timeout", "5")
        self.assertEqual((code, out["state"]), (0, "IDLE"))
        self.assertTrue(wait_for(lambda: {7, 20} <= set(d.freed), 2), d.freed)
        with d.lock:
            d.reboot()
            d.next_id = 20              # the new boot's ids start over: the next 'photo' is 20 again
        boot_b = f"{d.boot:08x}"
        self.assertNotEqual(boot_a, boot_b)
        self.assertTrue(wait_for(lambda: self.events("reboot_cleared") and not dm.rebooted, 3))
        ev = self.events("photo_sets_dropped")
        self.assertEqual(len(ev), 1, ev)
        self.assertEqual((ev[0]["old"], ev[0]["new"], ev[0]["done"], ev[0]["freed"]), (boot_a, boot_b, [7, 20], [7, 20]))
        self.assertEqual((dm.sets, list(dm.done_sets), dm.freed_ids, dm.image_jobs), ({}, [], set(), {}))
        out_b = self.sortie_to_wait("--photos", "1", "--gap", "150")
        self.assertEqual([p["id"] for p in out_b["photos"]], [7])
        r_b = auto.daemon_request(self.sock, {"op": "photo", "n": 1, "gap": 150, "timeout": 8}, 12)
        self.assertEqual((r_b["ok"], r_b["id"], len(r_b["photos"])), (True, 20, 1), r_b)
        for old, new in ((out_a["photos"][0], out_b["photos"][0]), (r_a["photos"][0], r_b["photos"][0])):
            self.assertNotEqual(old["json"], new["json"], "the old boot's file was returned")
            with open(new["json"]) as f:
                doc = json.load(f)
            self.assertEqual((doc["id"], doc["link"]["bootNonce"]), (new["id"], boot_b))
        self.assertEqual([e["id"] for e in self.events("photo_set")], [7, 20, 7, 20])
        self.assertEqual([e["id"] for e in self.events("photo")], [7, 20, 7, 20])
        self.assertEqual(len(d.got(auto.T_PHOTO)), 2, "each 'photo' request reached the drone")
        self.assertEqual(self.cli("land")[0], 0)


class DaemonLinkTest(DaemonCase):
    def test_heartbeat_cadence_while_retrying_and_reassembling(self):
        """12.4 #2: HB gaps <= 150 ms over 3 s (<= 250 ms only across a PNG save, each < 100 ms) while
        a command retries (lost ACK) and a photo set is re-requested."""
        saves = []
        real_save = auto.save_frame

        def timed_save(*a, **kw):
            t0 = time.monotonic()
            try:
                return real_save(*a, **kw)
            finally:
                saves.append((t0, time.monotonic()))
        d = self.drone(lose_first_ack={auto.T_PHOTO}, set_drop={(0, 3), (0, 11), (1, "meta"), (1, 18)})
        with mock.patch.object(auto, "save_frame", timed_save):
            self.start_daemon(d)
            t0 = time.monotonic()
            code, out, _, err = self.oneshot(d, "photo", "--n", "2", "--gap", "150", "--out",
                                             os.path.join(self.tmp.name, "p"))
            self.assertEqual((code, out["ok"], len(out["photos"])), (0, True, 2), err)
            time.sleep(max(0.0, t0 + 3.0 - time.monotonic()))
        hb = [t for t, _ in d.got(auto.T_HB, since=t0) if t <= t0 + 3.0]
        self.assertGreaterEqual(len(hb), 28)
        photos = d.got(auto.T_PHOTO)
        self.assertEqual(len(photos), 2)
        self.assertEqual(photos[0][1].seq, photos[1][1].seq)
        self.assertTrue(photos[1][1].flags & auto.FLAG_RETX)
        self.assertTrue(d.got(auto.T_CHUNKS))
        self.assertEqual(len(saves), 2)
        for a, b in saves:
            self.assertLess(b - a, 0.1)
        for prev, cur in zip(hb, hb[1:]):
            gap = cur - prev
            during_save = any(a < cur and b > prev for a, b in saves)
            self.assertLessEqual(gap, 0.25 if during_save else 0.15, f"heartbeat gap {gap * 1000:.0f} ms")

    def test_k_key_kills_first(self):
        """12.4 #3: 'k': 10 KILL within 150 ms, the first before any other packet."""
        d = self.drone()
        dm, keys = self.start_daemon(d, keys=True)
        n = len(d.got(auto.T_HB))
        self.assertTrue(wait_for(lambda: len(d.got(auto.T_HB)) > n + 1, 1))
        time.sleep(0.02)            # everything sent with that heartbeat has arrived; the next is ~80 ms away
        t_key = time.monotonic()
        os.write(keys, b"k")
        self.assertTrue(wait_for(lambda: len(d.kill_rx) >= 10, 1))
        after = [(t, p) for t, p, _ in list(d.log) if t >= t_key]
        self.assertEqual(after[0][1].type, auto.T_KILL)
        kills = [(t, p) for t, p in after if p.type == auto.T_KILL]
        self.assertEqual(len(kills), 10)
        self.assertLessEqual(kills[-1][0] - t_key, 0.15)
        self.assertEqual({p.session for _, p in kills}, {d.session})
        self.assertTrue(all(p.flags & auto.FLAG_ACKREQ for _, p in kills))
        self.assertTrue(wait_for(lambda: len(dm.kill_acks) == 10, 1))
        self.assertEqual(self.events("kill")[0]["source"], "key")

    def test_kill_client_without_daemon(self):
        """12.4 #3: 'kill' with no daemon: 10 KILL, session 0, valid tag."""
        d = self.drone()
        keyf = self.key_file()
        code, out, _, _ = self.run_main(["kill", "--host", "127.0.0.1", "--port", str(d.port), "--key-file", keyf,
                                         "--sock", os.path.join(self.tmp.name, "none.sock")])
        self.assertEqual((code, out["ok"], out["sent"], out["acked"], out["session"]), (0, True, 10, 10, 0))
        self.assertFalse(out["daemon"]["ok"])
        self.assertEqual(len(d.kill_rx), 10)
        self.assertEqual({p.session for _, p, _ in d.kill_rx}, {0})
        self.assertEqual(len({p.seq for _, p, _ in d.kill_rx}), 10)
        self.assertLessEqual(d.kill_rx[-1][0] - d.kill_rx[0][0], 0.15)
        self.assertEqual(d.auth_fails, 0)
        self.assertEqual(d.kill_count, 10)

    @unittest.skipUnless(sys.platform == "darwin", "IP_BOUND_IF is macOS only")
    def test_kill_client_binding(self):
        """12.4 #13: the kill client's socket is built through Link: IP_BOUND_IF = the --iface index."""
        d = self.drone()
        bound = []

        class RecLink(auto.Link):
            def send(self, *a, **kw):
                bound.append(self.sock.getsockopt(socket.IPPROTO_IP, auto.IP_BOUND_IF))
                return super().send(*a, **kw)
        with mock.patch.object(auto, "Link", RecLink):
            code, out, _, _ = self.run_main(["kill", "--host", "127.0.0.1", "--iface", "lo0", "--port", str(d.port),
                                             "--key-file", self.key_file(),
                                             "--sock", os.path.join(self.tmp.name, "none.sock")])
        self.assertEqual((code, out["iface"], out["acked"]), (0, "lo0", 10))
        self.assertEqual(bound, [socket.if_nametoindex("lo0")] * 10)
        self.assertEqual({(p.type, p.session) for _, p, _ in d.kill_rx}, {(auto.T_KILL, 0)})
        self.assertEqual(d.auth_fails, 0)

    def test_kill_client_uses_the_daemons_binding_and_asks_it(self):
        d = self.drone()
        self.start_daemon(d)
        code, out, _, _ = self.run_main(["kill", "--key-file", self.keyf, "--sock", self.sock])
        self.assertEqual((code, out["drone"], out["acked"]), (0, "127.0.0.1", 10))
        self.assertEqual((out["daemon"]["ok"], out["daemon"]["sent"], out["daemon"]["acked"]), (True, 10, 10))
        self.assertEqual(sorted({p.session for _, p, _ in d.kill_rx}), sorted({0, d.session}))

    def test_kill_state_from_the_burst_and_auto_status(self):
        """P2: the first KILL ACK of a burst still carries the state before the KILL (auto_link.c reads it before
        the state machine's next step). With only that ACK arriving, the kill client's 'ack_states' show it, and
        its 'state' comes from the daemon's AUTO_STATUS that counts the KILL."""
        self.assertEqual(auto.kill_burst_state(["HOVER", "KILLED", "KILLED"]), "KILLED")
        self.assertEqual(auto.kill_burst_state(["HOVER", None]), "HOVER")
        self.assertEqual(auto.kill_burst_state(["IDLE", "IDLE"]), "IDLE")
        self.assertIsNone(auto.kill_burst_state([]))
        d = self.drone(**dict(FAST, wait_ms=8000), kill_ack_lag=True, kill_ack_keep=1)
        self.start_daemon(d)
        self.sortie_to_wait()
        code, out, _, _ = self.run_main(["kill", "--key-file", self.keyf, "--sock", self.sock])
        self.assertEqual((code, out["ok"], out["acked"], out["ack_states"]), (0, True, 1, ["HOVER"]))
        self.assertEqual((out["daemon"]["acked"], out["daemon"]["state"], out["state"]), (0, "KILLED", "KILLED"))
        self.assertGreater(out["daemon"]["killCount"], 10)

    def test_sortie_at_120_cm(self):
        """The take-off height goes up to 120 cm (the interactive hover, user decision 2026-10-12 late; 20..60
        before); 2.00 m (2026-10-15, docs/autonomy-a2b.md 23.9) only by moves up from the hover: --height 120 reaches
        the drone in TAKEOFF; 121 is a usage error and sends nothing."""
        self.assertEqual(auto.HEIGHT_CM, (20, 120))
        self.assertEqual(auto.MOVE_H_RANGE_M, (0.25, 2.00))
        d = self.drone(**dict(FAST, wait_ms=3000))
        self.start_daemon(d)
        code, out, _, _ = self.cli("sortie", "--mode", "dry", "--height", "121")
        self.assertEqual((code, out["ok"]), (2, False))
        self.assertIn("must be 20..120", out["error"])
        self.assertFalse(d.got(auto.T_ARM))
        self.sortie_to_wait("--height", "120", "--photos", "1", "--gap", "300")
        self.assertEqual([struct.unpack("<IHBBHH", p.payload)[1] for _, p in d.got(auto.T_TAKEOFF)], [120])
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))
        code, out, _, _ = self.cli("wait", "--until", "idle", "--timeout", "5")
        self.assertEqual((code, out["state"], out["reason"]), (0, "IDLE", "TOUCHDOWN"))

    def test_air_ms_may_only_lower_the_drones_autoair(self):
        """ARM airMaxMs, when not 0, must be <= the drone's NVS autoair (auto_fsm.c armCmd answers RANGE above
        it): checked against the last 'auto get', the default 6000 ms without one; a refused value sends nothing."""
        self.assertEqual((auto.AIR_MS, auto.AIR_MS_DEFAULT), ((2000, 8000), 6000))   # auto_fsm.h, 2026-10-12 evening
        d = self.drone()
        self.start_daemon(d)
        for argv in (("arm", "--mode", "dry", "--air-ms", "6500"), ("sortie", "--mode", "dry", "--air-ms", "8000")):
            code, out, _, _ = self.cli(*argv)
            self.assertEqual(code, 1, out)
            self.assertIn("2000..6000", out["error"])
            self.assertIn("default autoair", out["error"])
        self.assertFalse(d.got(auto.T_ARM))
        os.makedirs(auto.DEFAULT_DIR, exist_ok=True)

        def cache(autoair):
            with open(auto.vmin_cache_path(), "w") as f:
                json.dump({"utc": "2026-10-12T10:00:00Z", "values": {"autoair": {"value": autoair, "src": "nvs"}}}, f)
        cache("8000")
        self.assertEqual(auto.air_ms_limit()[0], 8000)
        code, out, _, _ = self.cli("arm", "--mode", "dry", "--air-ms", "8000")
        self.assertEqual((code, out["result"]), (0, "OK"), out)
        self.assertEqual(d.arm_air, [8000])
        code, out, _, _ = self.cli("disarm")
        self.assertEqual(code, 0, out)
        cache("3000")
        code, out, _, _ = self.cli("arm", "--mode", "dry", "--air-ms", "4000")
        self.assertEqual(code, 1, out)
        self.assertIn("2000..3000", out["error"])
        self.assertEqual(d.arm_air, [8000])
        cache("9999")       # out of the drone's range: not trusted, the default applies
        self.assertEqual(auto.air_ms_limit()[0], 6000)

    def kill_with_record(self, d, record, sock_exists=True, check=None):
        """The kill client with a daemon record and no --host/--iface; returns (open_link calls, reply).
        open_link is faked: an interface named gone* does not exist, anything else reaches the fake drone."""
        sock = os.path.join(self.tmp.name, "rec.sock")
        if sock_exists:
            open(sock, "w").close()
        elif os.path.exists(sock):
            os.unlink(sock)
        with open(auto.info_path(sock), "w") as f:
            json.dump(dict({"port": d.port}, **record), f)
        calls = []

        def fake_open(args, key):
            calls.append((args.host, args.iface))
            if (args.iface or "").startswith("gone"):
                raise auto.LinkError(f"no interface {args.iface}")
            return auto.Link(key, "127.0.0.1", d.port)
        chk = mock.Mock(return_value=check or {"link_up": False, "iface": None, "candidates": []})
        with mock.patch.object(auto, "open_link", fake_open), mock.patch.object(auto, "link_check", chk):
            code, out, _, _ = self.run_main(["kill", "--key-file", self.key_file(), "--sock", sock])
        self.assertEqual((code, out["acked"]), (0, out["sent"]), out)
        self.assertGreaterEqual(out["sent"], 10)
        return calls, out

    def test_kill_client_falls_back_when_the_daemon_binding_is_stale(self):
        """The daemon is alive but its interface is gone (a replug): the drone interface lookup."""
        d = self.drone()
        calls, _ = self.kill_with_record(d, {"pid": os.getpid(), "host": None, "iface": "gone0"},
                                         check={"link_up": False, "iface": None, "candidates": ["gone0"]})
        self.assertEqual(calls, [(None, "gone0"), (None, None)])

    def test_kill_client_ignores_a_dead_daemons_record(self):
        """WP3-5: a record left by a SIGKILLed daemon (its pid gone, or its socket gone) is never used, even
        when it still resolves (here a --host record pointing elsewhere): the fresh, HELLO-free lookup."""
        d = self.drone()
        p = subprocess.Popen([sys.executable, "-c", "pass"])
        p.wait()
        for record, sock_exists in (({"pid": p.pid, "host": "127.0.0.9", "iface": None}, True),
                                    ({"pid": os.getpid(), "host": "127.0.0.9", "iface": None}, False),
                                    ({"host": "127.0.0.9", "iface": None}, True)):
            with self.subTest(record=record, sock_exists=sock_exists):
                calls, out = self.kill_with_record(d, record, sock_exists)
                self.assertEqual(calls, [(None, None)])
                self.assertEqual(len(out["bindings"]), 1)
                self.assertIn("ignored", out["bindings"][0]["why"])

    def test_kill_client_checks_the_daemons_interface(self):
        """WP3-5: a live daemon's interface is used only while it holds a drone-network address; otherwise the
        interface the lookup finds; when both hold one but the route differs, both get the burst."""
        d = self.drone()
        rec = {"pid": os.getpid(), "host": None, "iface": "en7"}
        calls, _ = self.kill_with_record(d, rec, check={"link_up": True, "iface": "en7", "candidates": ["en7"]})
        self.assertEqual(calls, [(None, "en7")])
        calls, _ = self.kill_with_record(d, rec, check={"link_up": True, "iface": "en0", "candidates": ["en0"]})
        self.assertEqual(calls, [(None, None)])
        calls, out = self.kill_with_record(d, rec, check={"link_up": True, "iface": "en0",
                                                          "candidates": ["en0", "en7"]})
        self.assertEqual(calls, [(None, "en7"), (None, None)])
        self.assertEqual((out["sent"], len(out["bindings"])), (20, 2))

    def test_flight_gate(self):
        """12.4 #5: FLIGHT needs daemon --allow-flight and --confirm-flight; nothing is sent otherwise."""
        d = self.drone()
        self.start_daemon(d)
        for argv in (("arm", "--mode", "flight", "--confirm-flight"), ("arm", "--mode", "flight"),
                     ("sortie", "--mode", "flight", "--confirm-flight"), ("sortie", "--mode", "flight")):
            code, out, _, _ = self.cli(*argv)
            self.assertEqual((code, out["ok"], out["result"]), (1, False, "REFUSED"), argv)
            self.assertIn("--confirm-flight" if "--confirm-flight" not in argv else "--allow-flight", out["error"])
        self.assertFalse(d.got(auto.T_ARM))
        d2 = self.drone()
        self.start_daemon(d2, allow_flight=True, name="f")
        code, out, _, _ = self.cli("arm", "--mode", "flight")
        self.assertEqual((code, out["result"]), (1, "REFUSED"))
        self.assertFalse(d2.got(auto.T_ARM))
        code, out, _, _ = self.cli("arm", "--mode", "flight", "--confirm-flight")
        self.assertEqual((code, out["result"], out["mode"], out["state"]), (0, "OK", "FLIGHT", "ARMED"))
        self.assertEqual(out["token"], d2.token)
        code, out, _, _ = self.cli("disarm")
        self.assertEqual((code, out["result"]), (0, "OK"))

    def test_reboot_refuses_arm(self):
        """12.4 #6: a bootNonce change gives drone_rebooted (the old and new nonce, the last state); ARM is refused
        while the latch holds; since 2026-10-15 (docs/autonomy-a2b.md 22) it clears once the drone reports IDLE in
        its new boot (reboot_cleared), and ARM goes to the drone's own guards again (until then: refused until the
        daemon was restarted)."""
        d = self.drone()
        dm, _ = self.start_daemon(d)
        with d.lock:
            d.reboot()
        self.assertTrue(wait_for(lambda: self.events("drone_rebooted"), 3))
        ev = self.events("drone_rebooted")[0]
        self.assertEqual((ev["old"], ev["new"]), ("0badf00d", f"{0x0BADF00D ^ 0x22222222:08x}"))
        self.assertIn("last_state", ev)
        self.assertEqual(d.hellos, 2)           # idle: one new HELLO after BAD_SESSION
        self.assertTrue(wait_for(lambda: self.events("reboot_cleared") and not dm.rebooted, 3))
        dm.rebooted = True                      # the latch (between the reboot and the first IDLE status)
        self.assertIn("rebooted", dm.refusal())
        self.assertTrue(wait_for(lambda: not dm.rebooted, 3))   # the next IDLE status clears it again
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["result"]), (0, "OK"), out)
        self.assertEqual(len(d.got(auto.T_ARM)), 1)
        self.assertEqual(self.cli("disarm")[0], 0)

    def test_bad_json_keeps_the_daemon(self):
        """12.4 #9: bad JSON on the socket: an error reply, the daemon and its heartbeat go on."""
        d = self.drone()
        self.start_daemon(d)
        for raw, want in ((b"not json\n", "bad request"), (b'["op"]\n', "bad request"), (b'{"op": "fly"}\n', "unknown op"),
                          (b'{"op": "arm", "mode": "upside-down"}\n', "bad request")):
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(2)
            s.connect(self.sock)
            s.sendall(raw)
            reply = json.loads(s.makefile().readline())
            s.close()
            self.assertFalse(reply["ok"])
            self.assertIn(want, reply["error"])
        n = len(d.got(auto.T_HB))
        time.sleep(0.5)
        self.assertGreaterEqual(len(d.got(auto.T_HB)) - n, 4)
        self.assertTrue(auto.daemon_request(self.sock, {"op": "ping"}, 1)["ok"])
        self.assertTrue(self.thread.is_alive())

    def test_one_shots_route_through_the_daemon(self):
        """12.4 #10: with a sortie running, status/hello/photo one-shots send no HELLO."""
        d = self.drone(**dict(FAST, wait_ms=8000))
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        code, out, _, err = self.oneshot(d, "status")
        self.assertEqual((code, out["via"], out["auto"]["stateName"], out["status"]["stateName"]),
                         (0, "daemon", "HOVER", "HOVER"), err)
        code, out, _, _ = self.oneshot(d, "hello")
        self.assertEqual((code, out["via"], out["session"]), (0, "daemon", f"{d.session:08x}"))
        code, out, _, _ = self.oneshot(d, "photo", "--n", "1", "--gap", "150", "--out", os.path.join(self.tmp.name, "o"))
        self.assertEqual((code, len(out["photos"])), (0, 1))
        self.assertTrue(out["photos"][0]["png"].startswith(os.path.join(self.tmp.name, "o")))
        self.assertEqual(d.hellos, 1)
        self.assertEqual(d.state, HOVER)
        self.assertNotIn("SESSION", [r for _, _, r in d.timeline])

    def test_never_rehello_while_active(self):
        """12.4 #11: BAD_SESSION while the drone is active (another HELLO replaced ours in HOVER; the drone lands
        SESSION): session_lost, no HELLO while it is active (the heartbeat goes on in the old session), ARM refused;
        since 2026-10-15 (docs/autonomy-a2b.md 22) a HELLO once a BAD_SESSION shows the drone IDLE (session_back),
        and ARM goes to the drone again (until then: refused until the daemon was restarted)."""
        d = self.drone(**dict(FAST, wait_ms=8000, land_s=2.0), bad_session_in_hover=True)
        dm, _ = self.start_daemon(d)
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual(code, 0)
        code, out, _, _ = self.cli("takeoff", "--photos", "0")
        self.assertEqual(code, 0)
        self.assertTrue(wait_for(lambda: dm.session_lost, 3))
        old = dm.link.session
        self.assertNotEqual(old, d.session)
        t = time.monotonic()
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["result"]), (1, "REFUSED"))
        self.assertIn("HELLO again once the drone is idle", out["error"])
        time.sleep(0.8)
        self.assertEqual(d.hellos, 1)           # still landing (LAND SESSION, 2 s): no HELLO
        hb_old = [p for _, p in d.got(auto.T_HB, since=t) if p.session == old]
        self.assertGreaterEqual(len(hb_old), 6)
        self.assertIn(self.events("session_lost")[0]["state"], ("HOVER", "LAND"))
        self.assertTrue(wait_for(lambda: self.events("session_back"), 6), d.timeline)
        t_idle = [tl[0] for tl in d.timeline if tl[1] == "IDLE"][-1]
        hellos = [tt for tt, _ in d.got(auto.T_HELLO)]
        self.assertEqual(len(hellos), 2)
        self.assertGreaterEqual(hellos[1], t_idle)   # the new HELLO only once the drone was idle
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["result"]), (0, "OK"), out)
        self.assertEqual(self.cli("disarm")[0], 0)

    def test_busy_taken_unknown_is_reconciled_not_resent(self):
        """12.4 #12: TAKEOFF answered BUSY taken-unknown but acted on: no re-send, done from AUTO_STATUS."""
        d = self.drone(**FAST, busy_unknown={auto.T_TAKEOFF})
        self.start_daemon(d)
        self.assertEqual(self.cli("arm", "--mode", "dry")[0], 0)
        code, out, _, err = self.cli("takeoff", "--photos", "0")
        self.assertEqual((code, out["ok"], out["result"], out["reconciled"]), (0, True, "OK", True), err)
        self.assertEqual(len(d.got(auto.T_TAKEOFF)), 1)
        self.assertTrue(self.events("busy_taken_unknown"))
        self.assertEqual(self.cli("land")[0], 0)

    def test_busy_with_state_is_resent_with_a_new_seq(self):
        d = self.drone(busy_state={auto.T_ARM: 1})
        self.start_daemon(d)
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["result"]), (0, "OK"))
        arms = [p for _, p in d.got(auto.T_ARM)]
        self.assertEqual(len(arms), 2)
        self.assertGreater(arms[1].seq, arms[0].seq)
        self.assertEqual(d.accepted[auto.T_ARM], 2)
        self.assertEqual(self.cli("disarm")[0], 0)

    def test_lost_command_is_sent_again_under_a_new_seq(self):
        """A lost TAKEOFF: its RETX under the same seq is dropped as old once a heartbeat with a newer
        seq got through, so after two RETX the daemon sends it under a new seq; it acts once."""
        d = self.drone(**FAST, lose_cmd={auto.T_TAKEOFF})
        self.start_daemon(d)
        self.assertEqual(self.cli("arm", "--mode", "dry")[0], 0)
        code, out, _, err = self.cli("takeoff", "--photos", "0")
        self.assertEqual((code, out["result"]), (0, "OK"), err)
        sent = [p for _, p in d.got(auto.T_TAKEOFF)]
        self.assertGreaterEqual(len({p.seq for p in sent}), 2)
        self.assertEqual(d.accepted[auto.T_TAKEOFF], 1)
        self.assertEqual(self.cli("land")[0], 0)

    def test_q_lands_first_then_quits(self):
        d = self.drone(**dict(FAST, wait_ms=8000))
        dm, keys = self.start_daemon(d, keys=True)
        self.sortie_to_wait("--photos", "0")
        os.write(keys, b"q")
        self.thread.join(4)
        self.assertFalse(self.thread.is_alive())
        self.assertIn("LAND_CMD", [r for _, s, r in d.timeline if s == "LAND"])
        self.assertFalse(os.path.exists(self.sock))
        self.assertEqual(self.events("daemon_exit")[0]["hb_late"], 0)

    def test_second_daemon_refused_stale_socket_replaced(self):
        d = self.drone()
        self.start_daemon(d)
        other = auto.Daemon(auto.Link(KEY, "127.0.0.1", d.port), self.sock, self.tmp.name)
        with self.assertRaisesRegex(auto.LinkError, "already runs"):
            other.start()
        other.link.close()
        stale = os.path.join(self.tmp.name, "stale.sock")
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.bind(stale)
        s.close()                   # the file stays, nothing listens
        d2 = self.drone()
        dm = auto.Daemon(auto.Link(KEY, "127.0.0.1", d2.port), stale, self.tmp.name)
        dm.start()
        self.assertTrue(os.path.exists(stale))
        dm.close()
        self.assertFalse(os.path.exists(stale))


def raw_client(path, req):
    """A client that sends one request and leaves the connection to the test (to close, or never read)."""
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect(path)
    s.sendall((json.dumps(req) + "\n").encode())
    return s


class DaemonAbandonTest(DaemonCase):
    """WP3-1: once a sortie has ended (timeout, client gone, KILL), nothing more is sent for it, never a
    TAKEOFF; what it may have armed is DISARMed, and once it may have taken off it is LANDed."""

    def assert_never_took_off(self, d):
        self.assertFalse(d.got(auto.T_TAKEOFF), "a TAKEOFF went out for an ended sortie")
        self.assertNotIn("TAKEOFF", [s for _, s, _ in d.timeline])

    def assert_disarmed(self, d, since):
        self.assertTrue(wait_for(lambda: d.got(auto.T_DISARM, since), 2), "the ARMed sortie was not DISARMed")
        want = [("ARMED", "ARM"), ("IDLE", "DISARM_CMD")]
        self.assertTrue(wait_for(lambda: [tl[1:] for tl in d.timeline][-2:] == want, 2), d.timeline)
        self.assertEqual(d.state, IDLE)

    def test_timeout_in_phase_arm_disarms_and_never_takes_off(self):
        """The ARM pending on the drone (0.4 s) when the sortie times out (0.2 s): the ARM, answered later,
        is DISARMed; never a TAKEOFF."""
        d = self.drone(**FAST, slow={auto.T_ARM: 0.4})
        self.start_daemon(d)
        t0 = time.monotonic()
        code, out, _, _ = self.cli("sortie", "--mode", "dry", "--timeout", "0.2")
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertIn("timeout in phase arm", out["error"])
        self.assert_disarmed(d, t0)
        time.sleep(0.3)
        self.assert_never_took_off(d)
        self.assertEqual(self.events("sortie_abandoned")[0]["phase"], "arm")

    def test_timeout_zero_sends_nothing(self):
        """A sortie past its deadline before its ARM could go out: the ARM is withdrawn at the moment of
        sending (Daemon.pump asks Sortie.ended, which checks the deadline): no packet at all for it."""
        d = self.drone(**FAST)
        self.start_daemon(d)
        code, out, _, _ = self.cli("sortie", "--mode", "dry", "--timeout", "0")
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertIn("timeout in phase arm", out["error"])
        time.sleep(0.3)
        for t in (auto.T_ARM, auto.T_TAKEOFF, auto.T_DISARM, auto.T_LAND):
            self.assertFalse(d.got(t), auto.TYPE_NAMES[t])
        self.assertEqual(d.state, IDLE)

    # -------- the deadline seen at once (WP3-1 gap a): the loop stepped by hand, so that the deadline
    # passes exactly between two passes

    def manual_daemon(self, d):
        """A daemon whose loop the test steps (no thread)."""
        self.keyf = self.key_file()
        self.sock = os.path.join(self.tmp.name, "m.sock")
        dm = auto.Daemon(auto.Link(KEY, "127.0.0.1", d.port), self.sock, os.path.join(self.tmp.name, "photos"),
                         events_path=None, out=io.StringIO(), host="127.0.0.1")
        dm.start()
        self.addCleanup(dm.close, True)
        self.dm = dm
        return dm

    @staticmethod
    def step_until(dm, pred, timeout=3.0):
        t_end = time.monotonic() + timeout
        while not pred() and time.monotonic() < t_end:
            dm.step(0.01)
        return bool(pred())

    def manual_sortie(self, d, **req):
        dm = self.manual_daemon(d)
        s = raw_client(self.sock, dict({"op": "sortie", "mode": "dry", "timeout": 5}, **req))
        self.addCleanup(s.close)
        return dm, s

    def reply_of(self, dm, s):
        f = s.makefile()
        self.assertTrue(self.step_until(dm, lambda: select.select([s], [], [], 0)[0]), "no reply")
        return json.loads(f.readline())

    def test_deadline_passed_before_the_arm_answer_is_read(self):
        """The deadline passes while the ARM's ACK (OK, with the token) waits in the daemon's socket. The
        pass that reads it sees the timeout first (Sortie.ended checks the deadline, not only step()): no
        TAKEOFF, the ARM is DISARMed, and the reply is the timeout."""
        d = self.drone(**FAST, slow={auto.T_ARM: 0.3})
        dm, s = self.manual_sortie(d)
        self.assertTrue(self.step_until(dm, lambda: d.got(auto.T_ARM)))
        time.sleep(0.45)                # the ARM is processed (0.3 s); its ACK waits, nobody reads it
        self.assertEqual(d.state, ARMED)
        dm.sorties[0].deadline = time.monotonic() - 0.001
        r = self.reply_of(dm, s)
        self.assertFalse(r["ok"])
        self.assertIn("timeout in phase arm", r["error"])
        self.assertTrue(self.step_until(dm, lambda: d.state == IDLE and not dm.stopping()), d.timeline)
        self.assert_never_took_off(d)
        self.assertTrue(d.got(auto.T_DISARM))
        self.assertEqual([tl[1:] for tl in d.timeline][-2:], [("ARMED", "ARM"), ("IDLE", "DISARM_CMD")])

    def test_deadline_passed_while_the_takeoff_is_unanswered(self):
        """The TAKEOFF went out but its ACKs are lost; the deadline passes just before the retry that would
        send it again (a new seq). The retry sees the timeout: never sent again, not even as a RETX; once its
        answer is given up, a fresh AUTO_STATUS decides: it took off, so it flies on (takeoff_late), no LAND
        (2026-10-15, "Claude decides": a LAND until then)."""
        d = self.drone(**dict(FAST, hover_at=3.0), mute_acks={auto.T_TAKEOFF})
        dm, s = self.manual_sortie(d, photos=0)
        self.assertTrue(self.step_until(dm, lambda: d.got(auto.T_TAKEOFF)))
        c = dm.flight
        self.assertEqual(c.ptype, auto.T_TAKEOFF)
        c.attempt, c.next_retry = 2, time.monotonic()      # the next retry: a copy under a new seq
        dm.sorties[0].deadline = time.monotonic() - 0.001
        r = self.reply_of(dm, s)
        self.assertIn("timeout in phase takeoff", r["error"])
        self.assertTrue(self.step_until(dm, lambda: self.events("takeoff_late"), 6), d.timeline)
        self.assertEqual(self.events("takeoff_late")[0]["why"], "the sortie had ended when its TAKEOFF was answered")
        self.step_until(dm, lambda: False, 0.5)
        self.assertNotIn("LAND_CMD", [x for _, _, x in d.timeline], d.timeline)
        self.assertFalse(d.got(auto.T_LAND) or d.got(auto.T_DISARM))
        self.assertIn(d.state, (TAKEOFF, HOVER))
        self.assertEqual(len(d.got(auto.T_TAKEOFF)), 1, "the TAKEOFF was sent again after the deadline")

    def test_deadline_passed_before_a_busy_takeoff_is_sent_again(self):
        """The TAKEOFF answered BUSY untaken (not processed) waits BUSY_RETRY_S to go again; the deadline
        passes meanwhile. The copy is withdrawn at the moment of sending (pump asks the sortie): one TAKEOFF
        only, and the drone (ARMED) is stopped."""
        d = self.drone(**dict(FAST, hover_at=3.0), busy_state={auto.T_TAKEOFF: 1})
        dm, s = self.manual_sortie(d, photos=0)
        self.assertTrue(self.step_until(dm, lambda: any(c.ptype == auto.T_TAKEOFF for c in dm.queue)))
        c = next(c for c in dm.queue if c.ptype == auto.T_TAKEOFF)
        c.not_before = dm.sorties[0].deadline = time.monotonic() - 0.001     # the copy due as the deadline passes
        r = self.reply_of(dm, s)
        self.assertIn("timeout in phase takeoff", r["error"])
        self.assertTrue(self.step_until(dm, lambda: d.state == IDLE and not dm.stopping()), d.timeline)
        self.assertEqual(len(d.got(auto.T_TAKEOFF)), 1, "the TAKEOFF was sent again after the deadline")
        self.assertNotIn("TAKEOFF", [st for _, st, _ in d.timeline])

    def test_client_gone_before_the_arm_answer(self):
        """The sortie's client goes away (Ctrl-C) while its ARM is pending on the drone: the ARM, answered
        OK later, is undone by a DISARM."""
        d = self.drone(**FAST, slow={auto.T_ARM: 0.2})
        self.start_daemon(d)
        t0 = time.monotonic()
        s = raw_client(self.sock, {"op": "sortie", "mode": "dry"})
        self.assertTrue(wait_for(lambda: d.got(auto.T_ARM), 2, 0.001))
        s.close()
        self.assert_disarmed(d, t0)
        time.sleep(0.3)
        self.assert_never_took_off(d)
        ev = self.events("sortie_abandoned")
        self.assertEqual((ev[0]["phase"], ev[0]["why"]), ("arm", "the sortie's client went away"))

    def test_client_gone_at_once(self):
        """The client closes right after its request: the ARM is either never sent (the daemon saw the close
        first) or DISARMed after; never a TAKEOFF."""
        for _ in range(3):
            d = self.drone(**FAST)
            self.start_daemon(d)
            t0 = time.monotonic()
            raw_client(self.sock, {"op": "sortie", "mode": "dry"}).close()
            self.assertTrue(wait_for(lambda: self.events("sortie_abandoned"), 2))
            time.sleep(0.5)
            self.assert_never_took_off(d)
            if d.got(auto.T_ARM):
                self.assert_disarmed(d, t0)
            self.assertEqual(d.state, IDLE)
            self.dm.request_stop()
            self.thread.join(3)

    def test_kill_during_the_arm_never_takes_off(self):
        """'k' while the sortie's ARM is pending on the drone: the sortie ends with the KILL."""
        d = self.drone(**FAST, slow={auto.T_ARM: 0.25})
        _, keys = self.start_daemon(d, keys=True)
        res = {}
        th = threading.Thread(target=lambda: res.update(r=auto.daemon_request(
            self.sock, {"op": "sortie", "mode": "dry", "timeout": 5}, 10)))
        th.start()
        self.assertTrue(wait_for(lambda: d.got(auto.T_ARM), 2, 0.001))
        os.write(keys, b"k")
        th.join(10)
        self.assertFalse(res["r"]["ok"])
        self.assertIn("KILL", res["r"]["error"])
        time.sleep(0.6)
        self.assert_never_took_off(d)
        self.assertEqual(len(d.kill_rx), 10)

    def test_client_gone_in_the_air_holds(self):
        """Since 2026-10-15 ("Claude decides") a sortie abandoned in the air (its client gone) does not LAND: the
        drone holds (sortie_holding); the next command, the battery or a lost link decide (a LAND until then)."""
        d = self.drone(**dict(FAST, hover_at=1.5, wait_ms=8000))
        dm, _ = self.start_daemon(d)
        s = raw_client(self.sock, {"op": "sortie", "mode": "dry", "photos": 0})
        self.assertTrue(wait_for(lambda: dm.sorties and dm.sorties[0].phase == "flying", 3))
        s.close()
        self.assertTrue(wait_for(lambda: self.events("sortie_holding"), 2))
        time.sleep(0.5)
        self.assertNotIn("LAND_CMD", [r for _, _, r in d.timeline], d.timeline)
        self.assertFalse(self.events("land_sent"))
        self.assertEqual(self.events("sortie_abandoned")[0]["phase"], "flying")
        self.assertEqual(self.events("sortie_holding")[0]["why"], "the sortie's client went away")
        self.assertEqual(len(d.got(auto.T_TAKEOFF)), 1)

    def test_timeout_after_takeoff_holds(self):
        d = self.drone(**dict(FAST, hover_at=3.0))
        self.start_daemon(d)
        code, out, _, _ = self.cli("sortie", "--mode", "dry", "--timeout", "0.6", "--photos", "0")
        self.assertEqual(code, 1)
        self.assertIn("timeout in phase flying", out["error"])
        self.assertTrue(wait_for(lambda: self.events("sortie_holding"), 2))
        time.sleep(0.5)
        self.assertNotIn("LAND_CMD", [r for _, _, r in d.timeline], d.timeline)

    def test_undo_rules(self):
        """Daemon.undo: the stop that follows an ended sortie's ARM or TAKEOFF, by its answer."""
        dm = auto.Daemon(auto.Link(KEY, "127.0.0.1", 9), os.path.join(self.tmp.name, "u.sock"), self.tmp.name)
        self.addCleanup(dm.link.close)
        dm.link.session = 1
        A, T = auto.T_ARM, auto.T_TAKEOFF
        for ptype, r, want in ((A, {"sent": False, "result": None}, None),
                               (A, {"sent": True, "result": auto.R_OK}, "DISARM"),
                               (A, {"sent": True, "result": None}, "DISARM"),          # no answer, UNKNOWN
                               (A, {"sent": True, "result": auto.R_BUSY}, "DISARM"),
                               (A, {"sent": True, "result": auto.R_GUARDS}, None),
                               (T, {"sent": False, "result": None}, "DISARM"),         # never sent: still ARMED
                               (T, {"sent": True, "result": auto.R_BAD_STATE}, "DISARM"),
                               (T, {"sent": True, "result": auto.R_OK}, "settle"),     # may have acted: a fresh
                               (T, {"sent": True, "result": None}, "settle"),          # AUTO_STATUS decides
                               (T, {"sent": True, "result": auto.R_BAD_SESSION}, None)):
            with self.subTest(type=auto.TYPE_NAMES[ptype], r=r):
                dm.stop, dm.settles = None, []
                dm.undo(ptype, r, "test", time.monotonic())
                got = "settle" if dm.settles else None if dm.stop is None else auto.TYPE_NAMES[dm.stop.ptype]
                self.assertEqual(got, want)
                self.assertTrue(dm.stop is None or dm.stop.ptype != auto.T_LAND)
        # An ARM with no usable answer: never a LAND (2026-10-15). The latest AUTO_STATUS ARMED (or none, IDLE,
        # KILLED): DISARM; flying (TAKEOFF, HOVER, LAND: something else flies, or a SINK): nothing.
        for st, r, want, sends in ((auto.S_ARMED, {"sent": True, "result": None}, "DISARM", 1),
                                   (auto.S_HOVER, {"sent": True, "result": auto.R_BUSY}, None, 0),
                                   (auto.S_ARMED, {"sent": True, "result": auto.R_OK}, "DISARM", 1),
                                   (auto.S_LAND, {"sent": True, "result": None}, None, 0)):
            with self.subTest(state=auto.STATE_NAMES[st], r=r):
                dm.stop, dm.auto = None, {"state": st}
                dm.undo(A, r, "test", time.monotonic())
                self.assertEqual((None, 0) if dm.stop is None else (auto.TYPE_NAMES[dm.stop.ptype],
                                                                     len(dm.stop.seqs)), (want, sends))

    def test_settle_decide(self):
        """A TAKEOFF that may have acted after its sortie ended, by the fresh AUTO_STATUS: ARMED DISARM; TAKEOFF or
        HOVER: it flies on (takeoff_late); none in SETTLE_S: nothing sent (takeoff_unknown). Never a LAND."""
        dm = auto.Daemon(auto.Link(KEY, "127.0.0.1", 9), os.path.join(self.tmp.name, "u.sock"), self.tmp.name)
        self.addCleanup(dm.link.close)
        dm.link.session = 1
        for st, want_stop, want_ev in ((auto.S_ARMED, "DISARM", "takeoff_settled"),
                                       (auto.S_TAKEOFF, None, "takeoff_late"), (auto.S_HOVER, None, "takeoff_late"),
                                       (auto.S_IDLE, None, "takeoff_settled"), (None, None, "takeoff_unknown")):
            with self.subTest(state=st):
                dm.stop, dm.events = None, []
                a = None if st is None else {"state": st, "stateName": auto.STATE_NAMES[st]}
                dm.settle_decide({"why": "test"}, a)
                self.assertEqual(None if dm.stop is None else auto.TYPE_NAMES[dm.stop.ptype], want_stop)
                names = [e["event"] for e in dm.events]
                if want_stop is None:
                    self.assertIn(want_ev, names)
                self.assertNotIn("land_sent", names)

    def test_arm_unknown_stops(self):
        dm = auto.Daemon(auto.Link(KEY, "127.0.0.1", 9), os.path.join(self.tmp.name, "u.sock"), self.tmp.name)
        self.addCleanup(dm.link.close)
        D = auto.T_DISARM
        for st, r, want in ((None, {"sent": False, "result": None}, []),        # never sent: cannot have acted
                            (None, {"sent": True, "result": auto.R_GUARDS}, []),  # refused
                            (None, {"sent": True, "result": auto.R_OK}, []),      # answered (the token is known)
                            (None, {"sent": True, "result": None}, [D]),           # no AUTO_STATUS yet
                            (auto.S_IDLE, {"sent": True, "result": None}, [D]),
                            (auto.S_IDLE, {"sent": True, "result": auto.R_BUSY}, [D]),
                            (auto.S_ARMED, {"sent": True, "result": None}, [D]),        # never a LAND for an ARM
                            (auto.S_TAKEOFF, {"sent": True, "result": None}, []),       # it flies: nothing
                            (auto.S_HOVER, {"sent": True, "result": auto.R_BUSY}, []),
                            (auto.S_LAND, {"sent": True, "result": None}, []),          # (a SINK may resume)
                            (auto.S_KILLED, {"sent": True, "result": None}, [D])):
            with self.subTest(state=st, r=r):
                dm.auto = None if st is None else {"state": st}
                self.assertEqual(dm.arm_unknown_stops(r), want)


class SoakHintTest(DaemonCase):
    """G2 STILL in FLIGHT includes the heat soak after a flight (auto_fsm.h AUTO_SOAK_*): the ARM refusal says
    about how long to wait, from the last FLIGHT sortie's air time and the time since it ended."""

    def test_hint_numbers(self):
        self.assertEqual(auto.soak_hint(41.0, 60.0), "IMU heat soak: wait 22 s")    # 20 + 61.5 = 81.5 s
        self.assertEqual(auto.soak_hint(5.0, 20.0), "IMU heat soak: wait 8 s")      # 27.5 s
        self.assertEqual(auto.soak_hint(200.0, 30.0), "IMU heat soak: wait 60 s")   # at most 90 s
        self.assertIsNone(auto.soak_hint(41.0, 100.0))                               # over: STILL is stillness
        self.assertIsNone(auto.soak_hint(None, None))

    def test_daemon_tracks_the_last_sortie(self):
        dm = auto.Daemon(auto.Link(KEY, "127.0.0.1", 9), os.path.join(self.tmp.name, "s.sock"), self.tmp.name)
        self.addCleanup(dm.link.close)
        now = time.monotonic()

        def status(state, air_ms, mode=auto.MODE_FLIGHT, transitions=1):
            return auto.parse_auto_status(auto.auto_status_payload(
                {"state": state, "mode": mode, "airMs": air_ms, "transitions": transitions, "bootNonce": 7}))

        def refused(bits, mode=auto.MODE_FLIGHT):
            out, _ = dm.arm_result({"ok": False, "result": auto.R_GUARDS, "resultName": "GUARDS", "detail": bits,
                                    "state": auto.S_IDLE, "stateName": "IDLE"}, mode)
            return out

        for k, (st, air) in enumerate(((auto.S_IDLE, 0), (auto.S_ARMED, 0), (auto.S_TAKEOFF, 1500),
                                       (auto.S_HOVER, 30000), (auto.S_LAND, 41000), (auto.S_IDLE, 0))):
            dm.on_auto_status(status(st, air, transitions=k + 1), now + k)
        self.assertEqual((dm.soak_air_s, dm.soak_from), (41.0, now + 5))
        dm.soak_from = time.monotonic() - 60.0
        out = refused(1 << 2)
        self.assertEqual(out["guards"], ["STILL"])
        self.assertIn(out["hint"], ("IMU heat soak: wait 22 s", "IMU heat soak: wait 21 s"))
        self.assertNotIn("hint", refused(1 << 3))                         # not STILL
        self.assertNotIn("hint", refused(1 << 2, mode=auto.MODES["dry"]))  # DRY: 3 s, no soak
        # a DRY sortie does not count
        for k, (st, air) in enumerate(((auto.S_ARMED, 0), (auto.S_TAKEOFF, 2000), (auto.S_IDLE, 0))):
            dm.on_auto_status(status(st, air, mode=auto.MODES["dry"], transitions=10 + k), now + 10 + k)
        self.assertEqual(dm.soak_air_s, 41.0)
        dm.soak_from = time.monotonic() - 100.0
        self.assertNotIn("hint", refused(1 << 2))                         # the soak is over

    def test_collision_is_a_land_reason_by_name(self):
        """LAND COLLISION (user decision 2026-10-13; a KILLED reason in 98cd4ae..eb9264d): the daemon's 'state'
        event names it for the LAND, and the sortie still counts for the heat soak (the motors ran to the floor)."""
        dm = auto.Daemon(auto.Link(KEY, "127.0.0.1", 9), os.path.join(self.tmp.name, "s.sock"), self.tmp.name)
        self.addCleanup(dm.link.close)
        now = time.monotonic()
        seq = ((auto.S_ARMED, 1, 0), (auto.S_TAKEOFF, 2, 1500), (auto.S_HOVER, 3, 30000), (auto.S_LAND, 42, 34500),
               (auto.S_IDLE, 22, 0))
        for k, (st, why, air) in enumerate(seq):
            dm.on_auto_status(auto.parse_auto_status(auto.auto_status_payload(
                {"state": st, "reason": why, "mode": auto.MODE_FLIGHT, "airMs": air, "transitions": k + 1,
                 "bootNonce": 7})), now + k)
        got = [(e["to"], e["reason"]) for e in dm.events if e["event"] == "state"]
        self.assertEqual(got, [("ARMED", "ARM"), ("TAKEOFF", "TAKEOFF"), ("HOVER", "HOLD"), ("LAND", "COLLISION"),
                               ("IDLE", "TOUCHDOWN")])
        self.assertEqual((auto.REASON_NAMES[42], dm.soak_air_s), ("COLLISION", 34.5))


class MoveSpeedTest(unittest.TestCase):
    """docs/autonomy-a2b.md 17.5: the drone flies MOVEs at up to MOVE_V_H_MAX (0.30 m/s); the Mac's wait for a
    move keeps its 0.20 m/s basis, so it is still longer than the drone's profile, the settle and the re-view bob."""

    @staticmethod
    def profile_s(dist, v, a):
        """The drone's trapezoid (auto_motion.c autoMvProfPlan): time for dist at top speed v, acceleration a."""
        if dist <= v * v / a:
            return 2.0 * math.sqrt(dist / a)
        return dist / v + v / a

    def drone_move_s(self, right, fwd, d):
        horiz = math.hypot(right, fwd)
        v = auto.MOVE_V_H_MAX
        if right:
            v = min(v, auto.MOVE_W_LAT_MAX * d * horiz / abs(right))
        if fwd:
            v = min(v, auto.MOVE_W_FWD_MAX * max(d - max(fwd, 0.0), 0.3) * horiz / abs(fwd))
        if fwd > 0:
            v = min(v, auto.MOVE_V_FWD_MAX * horiz / fwd)   # toward the scene: as before (approaches)
        return self.profile_s(horiz, v, auto.MOVE_A_H)

    def test_timeout_covers_the_faster_profile(self):
        settle_s, bob_s, margin_s = 3.0, 4.0, 2.0
        for right, fwd, d in ((1.5, 0.0, 3.7), (0.0, 1.5, 3.7), (1.0, 1.0, 4.6), (1.5, 0.0, 5.0), (0.3, 0.0, 1.0),
                              (0.0, 1.0, 2.9)):
            need = self.drone_move_s(right, fwd, d) + settle_s + bob_s + margin_s
            self.assertGreaterEqual(auto.Daemon.move_timeout(right, fwd, 0.0, d), need, (right, fwd, d))
        self.assertAlmostEqual(self.drone_move_s(1.5, 0.0, 3.7), 8.0, places=3)       # 0.30 m/s from D 2.5 m
        self.assertAlmostEqual(self.drone_move_s(1.0, 0.0, 2.0), 1.0 / 0.24 + 2.4, places=3)   # 0.12 x D caps it
        self.assertAlmostEqual(self.drone_move_s(0.0, 1.5, 5.0), 1.5 / 0.20 + 2.0, places=3)   # forward: 0.20 m/s
        self.assertAlmostEqual(self.drone_move_s(0.0, -1.5, 5.0), 8.0, places=3)       # backward: 0.30 m/s


class ArmCameraWakeTest(DaemonCase):
    """docs/autonomy-a2b.md 17.1: the drone's camera sleeps on the ground and the ARM wakes it, so an ARM refused
    GUARDS for CAMERA alone is sent again (new seq) after ARM_CAM_WAKE_S, at most ARM_CAM_WAKE_RESENDS times, in
    the plain 'arm' and in a sortie; any other guard set is answered as before."""
    CAM = auto.G_CAMERA

    def test_constants(self):
        self.assertEqual(self.CAM, 1 << 4)
        self.assertEqual((auto.ARM_CAM_WAKE_S, auto.ARM_CAM_WAKE_RESENDS), (0.5, 6))

    def arms(self, d):
        return [p for _, p in d.got(auto.T_ARM)]

    def test_camera_wakes_after_one_retry(self):
        d = self.drone(arm_guards=[self.CAM])
        self.start_daemon(d)
        t0 = time.monotonic()
        code, out, _, err = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["ok"], out["result"], out.get("cam_wake_retries")), (0, True, "OK", 1), (out, err))
        arms = self.arms(d)
        self.assertEqual(len(arms), 2)
        self.assertNotEqual(arms[0].seq, arms[1].seq, "sent again under a new seq")
        self.assertGreaterEqual(time.monotonic() - t0, auto.ARM_CAM_WAKE_S - 0.05)
        self.assertEqual([e["retries"] for e in self.events("arm_cam_wake")], [1])
        self.assertEqual(d.state, ARMED)

    def test_gives_up_after_six(self):
        d = self.drone(arm_guards=[self.CAM] * 7)
        self.start_daemon(d)
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["ok"], out["result"], out["guards"]), (1, False, "GUARDS", ["CAMERA"]))
        self.assertEqual((out["cam_wake_retries"], len(self.arms(d))), (6, 7))
        self.assertIn("camera did not come up", out["hint"])
        self.assertEqual([e["retries"] for e in self.events("arm_cam_wake")], [1, 2, 3, 4, 5, 6])

    def test_other_guards_are_final(self):
        still = 1 << auto.GUARD_NAMES.index("STILL")
        d = self.drone(arm_guards=[still | self.CAM])
        self.start_daemon(d)
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["result"], out["guards"]), (1, "GUARDS", ["STILL", "CAMERA"]))
        self.assertEqual(len(self.arms(d)), 1)
        self.assertNotIn("cam_wake_retries", out)
        self.assertEqual(self.events("arm_cam_wake"), [])

    def test_sortie_uses_the_same_retry(self):
        d = self.drone(**dict(FAST, wait_ms=3000), arm_guards=[self.CAM, self.CAM])
        self.start_daemon(d)
        out = self.sortie_to_wait("--photos", "1", "--gap", "150")
        self.assertEqual(out["arm"]["cam_wake_retries"], 2)
        self.assertEqual(len(self.arms(d)), 3)
        self.assertEqual([e["retries"] for e in self.events("arm_cam_wake")], [1, 2])
        self.assertTrue(d.got(auto.T_TAKEOFF))

    def test_cam_unavail_hint_names_the_take_off_standby(self):
        hint = auto.RESULT_HINTS["CAM_UNAVAIL"]
        self.assertIn("take-off", hint)
        self.assertIn("wakes for a PHOTO", hint)


class DaemonArmUnknownTest(DaemonCase):
    """WP3-1 gap b: an ARM that may have acted but got no usable answer (every ACK lost, BUSY beyond
    BUSY_RESENDS, a taken-unknown reconcile ending UNKNOWN) never ends silently: DISARM unless the drone flies;
    the reply says so (stops). Never a LAND for an ARM (2026-10-15; a DISARM and a LAND until then)."""

    def assert_stopped(self, d, out, stops):
        self.assertEqual(out.get("stops"), stops, out)
        self.assertIn("the ARM may have acted: " + " and ".join(stops) + " sent", out["error"])
        self.assertTrue(wait_for(lambda: d.got(auto.T_DISARM), 2), "no DISARM")
        if "LAND" in stops:
            self.assertTrue(wait_for(lambda: d.got(auto.T_LAND), 2), "no LAND")
        self.assertTrue(wait_for(lambda: d.state == IDLE and not self.dm.stopping(), 3), d.timeline)
        time.sleep(0.2)
        self.assertFalse(d.got(auto.T_TAKEOFF))
        self.assertEqual(bool(d.got(auto.T_LAND)), "LAND" in stops)
        self.assertIsNone(self.dm.arm_token)

    def test_sortie_arm_acks_all_lost(self):
        """Every ARM ACK lost; the drone ARMED (its AUTO_STATUS push shows it): DISARM (no LAND), IDLE."""
        d = self.drone(**FAST, mute_acks={auto.T_ARM})
        self.start_daemon(d)
        code, out, _, _ = self.cli("sortie", "--mode", "dry")
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertIn("no ACK for ARM after 5 tries", out["error"])
        self.assert_stopped(d, out, ["DISARM"])
        self.assertEqual([tl[1:] for tl in d.timeline][-2:], [("ARMED", "ARM"), ("IDLE", "DISARM_CMD")])
        self.assertEqual([e["why"] for e in self.events("disarm_sent")],
                         ["sortie: the ARM may have acted without an answer"])

    def test_sortie_arm_busy_beyond_resends(self):
        """Every ARM answered BUSY untaken (never processed): after BUSY_RESENDS copies the sortie fails; the
        DISARM still goes (harmless in IDLE); AUTO_STATUS shows IDLE, so no LAND."""
        d = self.drone(**FAST, busy_state={auto.T_ARM: 10})
        self.start_daemon(d)
        code, out, _, _ = self.cli("sortie", "--mode", "dry")
        self.assertEqual((code, out["ok"], out["result"]), (1, False, "BUSY"))
        self.assertEqual(d.accepted[auto.T_ARM], 1 + auto.BUSY_RESENDS)
        self.assert_stopped(d, out, ["DISARM"])
        self.assertEqual(d.timeline, [])

    def test_sortie_arm_reconcile_unknown(self):
        """The ARM acted, answered BUSY taken-unknown with the stabilizer stalled past RECONCILE_S (its
        snapshot frozen at IDLE): UNKNOWN, then DISARM; the drone goes from ARMED to IDLE."""
        d = self.drone(**FAST, stall={auto.T_ARM: (True, 1.5)})
        self.start_daemon(d)
        code, out, _, _ = self.cli("sortie", "--mode", "dry")
        self.assertEqual((code, out["ok"], out["result"]), (1, False, "UNKNOWN"))
        self.assert_stopped(d, out, ["DISARM"])
        self.assertEqual([tl[1:] for tl in d.timeline][-2:], [("ARMED", "ARM"), ("IDLE", "DISARM_CMD")])
        self.assertEqual(len(d.got(auto.T_ARM)), 1)

    def test_plain_arm_acks_all_lost(self):
        """The plain 'arm' the same way: no token, DISARM (no LAND), told in the reply."""
        d = self.drone(**FAST, mute_acks={auto.T_ARM})
        self.start_daemon(d)
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["ok"], out["token"]), (1, False, None))
        self.assert_stopped(d, out, ["DISARM"])
        self.assertEqual([tl[1:] for tl in d.timeline][-2:], [("ARMED", "ARM"), ("IDLE", "DISARM_CMD")])


class DaemonClientTest(DaemonCase):
    """A client that shuts down its write side after its request (shutdown(SHUT_WR), as 'nc -N' does) is
    not gone: it gets its reply, and a sortie it started goes on. It is gone once a write to it fails."""

    def half_closed(self, req):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.addCleanup(s.close)
        s.settimeout(8)
        s.connect(self.sock)
        s.sendall(req if isinstance(req, bytes) else (json.dumps(req) + "\n").encode())
        try:
            s.shutdown(socket.SHUT_WR)
        except OSError:
            pass        # the daemon answered a complete line and closed already (the reply is in our buffer)
        return s

    def test_client_check_tells_half_closed_from_closed(self):
        a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        self.addCleanup(a.close)
        self.addCleanup(b.close)
        a.setblocking(False)
        c = auto.Client(a)
        self.assertFalse(c.check())
        b.shutdown(socket.SHUT_WR)
        self.assertFalse(c.check())
        self.assertTrue(c.eof)
        self.assertFalse(c.closed or c.gone)
        c.reply({"ok": True})
        self.assertEqual(b.recv(100), b'{"ok": true}\n')       # the reply still reaches it
        a2, b2 = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        self.addCleanup(b2.close)
        a2.setblocking(False)
        c2 = auto.Client(a2)
        b2.shutdown(socket.SHUT_WR)
        self.assertFalse(c2.check())
        b2.close()
        self.assertTrue(c2.check())
        self.assertTrue(c2.gone and c2.closed)

    def test_half_closed_requests_get_their_replies(self):
        d = self.drone()
        self.start_daemon(d)
        for req in ({"op": "ping"}, b'{"op": "state"}', {"op": "events", "since": 0}, b"not json\n"):
            with self.subTest(req=req):
                line = self.half_closed(req).makefile().readline()
                r = json.loads(line)
                self.assertEqual(r["ok"], req != b"not json\n", r)
        s = self.half_closed(b"")                   # nothing asked: closed without a reply
        self.assertEqual(s.makefile().readline(), "")
        self.assertTrue(auto.daemon_request(self.sock, {"op": "ping"}, 1)["ok"])

    def test_half_closed_client_is_not_read_again(self):
        """A half-closed client waiting for a deferred reply is no longer selected for reading (its EOF would
        make every select return at once: a busy loop); it gets the reply at its time."""
        d = self.drone()
        passes = []

        def count(dm):
            step = dm.step
            dm.step = lambda *a: (passes.append(1), step(*a))
        self.start_daemon(d, pre=count)
        s = self.half_closed({"op": "wait", "until": "waiting", "timeout": 1.0})
        time.sleep(0.1)
        n0 = len(passes)
        time.sleep(0.6)
        self.assertLess(len(passes) - n0, 100, "the daemon's loop spins on the half-closed client")
        r = json.loads(s.makefile().readline())
        self.assertEqual((r["ok"], r["error"]), (False, "timeout waiting for waiting"))

    def test_half_closed_sortie_client_gets_its_reply(self):
        """The sortie's client half-closes at once ('nc -N'): the sortie is not abandoned; it returns in the
        hover wait as usual."""
        d = self.drone(**dict(FAST, wait_ms=6000))
        self.start_daemon(d)
        s = self.half_closed({"op": "sortie", "mode": "dry", "photos": 0})
        r = json.loads(s.makefile().readline())
        self.assertTrue(r["ok"], r)
        self.assertEqual((r["state"], r["waiting"]), ("HOVER", True))
        self.assertFalse(self.events("sortie_abandoned"))
        self.assertEqual(len(d.got(auto.T_TAKEOFF)), 1)
        self.assertNotIn("LAND_CMD", [x for _, _, x in d.timeline])
        self.assertEqual(self.cli("land")[0], 0)

    def test_half_closed_then_closed_abandons(self):
        """Half-closed, the sortie goes on; once the client closes (a write to it fails), it is gone: the
        sortie in the air is abandoned and the drone holds (no LAND since 2026-10-15)."""
        d = self.drone(**dict(FAST, hover_at=1.5, wait_ms=8000))
        dm, _ = self.start_daemon(d)
        s = self.half_closed({"op": "sortie", "mode": "dry", "photos": 0})
        self.assertTrue(wait_for(lambda: dm.sorties and dm.sorties[0].phase == "flying", 3))
        time.sleep(0.2)
        self.assertFalse(self.events("sortie_abandoned"))
        s.close()
        self.assertTrue(wait_for(lambda: self.events("sortie_holding"), 2))
        ev = self.events("sortie_abandoned")
        self.assertEqual((ev[0]["phase"], ev[0]["why"]), ("flying", "the sortie's client went away"))
        self.assertNotIn("LAND_CMD", [x for _, _, x in d.timeline], d.timeline)


class DaemonOutputTest(DaemonCase):
    """WP3-2: nothing the daemon writes blocks its loop."""

    def test_stalled_client_and_blocked_output_never_stall_the_loop(self):
        """A client asks for a large reply (1500 events, about 0.5 MB) and never reads it, and nobody reads
        the daemon's stdout (a full pipe, as a terminal held by Ctrl-S): the heartbeat gaps stay <= 150 ms,
        'k' sends its 10 KILLs within 150 ms, other clients are served, and the stalled client is dropped
        after CLIENT_SEND_S."""
        d = self.drone()
        r, w = os.pipe()
        out = os.fdopen(w, "w")
        def close_out():
            with contextlib.suppress(OSError):
                out.close()
        self.addCleanup(close_out)
        self.addCleanup(os.close, r)        # after the daemon stops: the blocked writer thread gets EPIPE

        def fill(dm):
            for i in range(1500):
                dm.event("filler", i=i, pad="x" * 300)
        dm, keys = self.start_daemon(d, keys=True, out=out, pre=fill)
        t0 = time.monotonic()
        s = raw_client(self.sock, {"op": "events"})            # never read
        self.addCleanup(s.close)
        time.sleep(1.0)
        self.assertTrue(auto.daemon_request(self.sock, {"op": "ping"}, 1)["ok"])
        t_key = time.monotonic()
        os.write(keys, b"k")
        self.assertTrue(wait_for(lambda: len(d.kill_rx) >= 10, 1, 0.05))
        kills = [round((t - t_key) * 1000) for t, _, _ in d.kill_rx]
        self.assertLessEqual(kills[0], 50, kills)
        self.assertLessEqual(kills[9], 150, kills)
        self.assertTrue(wait_for(lambda: dm.clients_dropped == 1, 3, 0.05))
        time.sleep(max(0.0, t0 + 3.0 - time.monotonic()))
        hb = [t for t, _ in d.got(auto.T_HB, since=t0) if t <= t0 + 3.0]
        self.assertGreaterEqual(len(hb), 28)
        gaps = [round((b - a) * 1000) for a, b in zip(hb, hb[1:])]
        self.assertLessEqual(max(gaps), 150, f"heartbeat gaps (ms): {gaps}")
        self.assertGreater(dm.out.dropped, 0)

    def test_line_writer_never_blocks(self):
        r, w = os.pipe()
        f = os.fdopen(w, "w")
        lw = auto.LineWriter(f, max_lines=100)
        t0 = time.monotonic()
        for _ in range(5000):
            lw.write_line("y" * 200)        # ~1 MB into a 64 KB pipe nobody reads
        self.assertLess(time.monotonic() - t0, 0.5)
        self.assertGreater(lw.dropped, 0)
        os.close(r)
        lw.close(0.2)
        with contextlib.suppress(OSError):
            f.close()

    def test_log_goes_through_the_sink(self):
        got = []
        with mock.patch.object(auto, "_log_sink", got.append):
            auto.log("hello")
        self.assertEqual(got, ["hello"])

    @unittest.skipUnless(hasattr(os, "openpty"), "needs a pty")
    def test_keys_tty_clears_flow_control(self):
        """The daemon's terminal: no IXON/IXOFF (Ctrl-S cannot stop its output), ISIG kept (Ctrl-C), no
        ICANON/ECHO; restored on exit. On the pty, Ctrl-S then does not stop the output."""
        import termios
        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        a = termios.tcgetattr(slave)
        a[0] |= termios.IXON | termios.IXOFF
        termios.tcsetattr(slave, termios.TCSANOW, a)
        before = termios.tcgetattr(slave)
        restore = auto.keys_tty(slave)
        a = termios.tcgetattr(slave)
        self.assertFalse(a[0] & (termios.IXON | termios.IXOFF))
        self.assertTrue(a[3] & termios.ISIG)
        self.assertFalse(a[3] & (termios.ICANON | termios.ECHO))
        os.write(master, b"\x13")           # Ctrl-S typed in the daemon's terminal
        time.sleep(0.05)
        os.write(slave, b"still flowing\n")
        ready, _, _ = select.select([master], [], [], 1.0)
        self.assertTrue(ready, "Ctrl-S stopped the output")
        self.assertIn(b"still flowing", os.read(master, 1024))
        restore()
        after = termios.tcgetattr(slave)
        pendin = getattr(termios, "PENDIN", 0)      # the kernel's "input pending" mark, not a setting
        after[3] &= ~pendin
        before[3] &= ~pendin
        self.assertEqual(after, before)


class DaemonBusyTest(DaemonCase):
    """WP3-3, WP3-4, D1, D2, D3: BUSY kinds, reconciling from a fresh AUTO_STATUS only, PHOTO without a set."""

    def photo(self, **kw):
        return auto.daemon_request(self.sock, dict({"op": "photo", "n": 1, "gap": 150, "timeout": 6}, **kw), 12)

    def test_busy_kinds(self):
        self.assertEqual(auto.busy_kind(auto.DETAIL_TAKEN_UNKNOWN), ("taken_unknown", None))
        self.assertEqual(auto.busy_kind(auto.busy_detail(auto.DETAIL_BUSY_PENDING, 0x12345678)), ("pending", 0x345678))
        self.assertEqual(auto.busy_kind(auto.busy_detail(auto.DETAIL_BUSY_UNTAKEN, 3)), ("untaken", 3))
        self.assertEqual(auto.busy_kind(auto.busy_detail(auto.DETAIL_BUSY_HELD_SET, 0x10007)), ("held_set", 7))
        self.assertEqual(auto.busy_kind(3), ("fsm", 3))
        self.assertEqual(auto.busy_kind(0x07000001)[0], "other")
        a = auto.parse_ack(auto.ack_payload(5, auto.T_PHOTO, auto.R_BUSY, 3, 0x03000014))
        self.assertEqual((a["busyKind"], a["busyValue"]), ("held_set", 20))

    def test_busy_pending_naming_its_own_earlier_copy_waits_for_it(self):
        """D1: an ARM pending on the drone for 0.8 s; its new-seq copy is answered BUSY pending with the first
        copy's seq. That one's answer follows, so no other copy is sent and the ARM gets its token (without
        the kind, a third copy would find the drone ARMED and the token would be lost)."""
        d = self.drone(**FAST, slow={auto.T_ARM: 0.8})
        self.start_daemon(d)
        code, out, _, err = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["result"], out["token"]), (0, "OK", d.token), (out, err))
        arms = [p for _, p in d.got(auto.T_ARM)]
        self.assertEqual(len({p.seq for p in arms}), 2)
        self.assertEqual(d.accepted[auto.T_ARM], 2)
        self.assertFalse(d.got(auto.T_DISARM))
        self.assertEqual(self.cli("disarm")[0], 0)

    def test_resent_photo_answered_busy_untaken_is_sent_again(self):
        """WP3-3: the first PHOTO lost, its new-seq copy answered BUSY untaken (detail 0x02000003, state
        HOVER): never processed, so it is sent again; the 3 is never read as a photo id."""
        d = self.drone(**dict(FAST, wait_ms=8000), lose_cmd={auto.T_PHOTO}, busy_state={auto.T_PHOTO: 1})
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        r = self.photo()
        self.assertTrue(r["ok"], r)
        self.assertEqual((r["id"], len(r["photos"])), (20, 1))
        self.assertEqual(d.accepted[auto.T_PHOTO], 2)

    def test_resent_photo_finding_another_set_held_is_not_taken_for_its_own(self):
        """WP3-3: a new-seq PHOTO answered BUSY held_set with a set newer than it, which AUTO_STATUS does not
        show as this PHOTO's (lastCmd is not PHOTO OK with that id): waited on, then sent again."""
        d = self.drone(**dict(FAST, wait_ms=8000), lose_cmd={auto.T_PHOTO}, held_reply=[99])
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        r = self.photo()
        self.assertTrue(r["ok"], r)
        self.assertEqual((r["id"], len(r["photos"])), (20, 1))

    def test_resent_photo_finding_its_own_set_held(self):
        """WP3-3: the first PHOTO acted on but every answer to it lost; its new-seq copy is refused BUSY
        held_set with the set the first started, and a fresh AUTO_STATUS (lastCmd PHOTO OK, photoId that
        id) confirms it is this PHOTO's: no second set."""
        d = self.drone(**dict(FAST, wait_ms=8000), mute_first={auto.T_PHOTO}, free_delay=1.0)
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        r = self.photo()
        self.assertTrue(r["ok"], r)
        self.assertEqual((r["id"], len(r["photos"])), (20, 1))
        self.assertEqual(d.next_id, 21, "a second set was started")
        self.assertEqual(d.photo_with_held, 1)

    def test_taken_unknown_photo_with_a_stale_snapshot_is_unknown(self):
        """WP3-4: PHOTO answered BUSY taken-unknown with the stabilizer stalled: every AUTO_STATUS carries the
        snapshot from before it (lastCmd PHOTO OK, photoId the previous set). Not newer than the PHOTO, it is
        never read as done: UNKNOWN after RECONCILE_S, never the previous id, and no resend."""
        d = self.drone(**dict(FAST, wait_ms=9000))
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        first = self.photo()
        self.assertTrue(first["ok"], first)
        with d.lock:
            d.stall[auto.T_PHOTO] = (False, 1.8)
        r = self.photo()
        self.assertFalse(r["ok"], r)
        self.assertEqual(r["result"], "UNKNOWN")
        self.assertNotEqual(r.get("id"), first["id"])
        self.assertEqual(len(d.got(auto.T_PHOTO)), 2)
        self.assertTrue(self.events("busy_taken_unknown"))

    def test_taken_unknown_photo_reconciled_from_a_fresh_snapshot(self):
        """WP3-4: the same PHOTO acted on, the stabilizer back after 0.4 s: the first AUTO_STATUS newer than
        the PHOTO shows the new set, so it is done with that id."""
        d = self.drone(**dict(FAST, wait_ms=9000))
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        self.assertTrue(self.photo()["ok"])
        with d.lock:
            d.stall[auto.T_PHOTO] = (True, 0.4)
        r = self.photo()
        self.assertTrue(r["ok"], r)
        self.assertEqual((r["id"], len(r["photos"])), (21, 1))
        self.assertEqual(len(d.got(auto.T_PHOTO)), 2)

    def test_taken_unknown_photo_not_acted_on_is_unknown(self):
        """WP3-4: the stabilizer back after 0.2 s but this PHOTO not processed: the AUTO_STATUS newer than it
        shows lastCmd PHOTO OK and a set, but both are the previous PHOTO's (its id was known before): never
        done with that id."""
        d = self.drone(**dict(FAST, wait_ms=9000))
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        first = self.photo()
        self.assertTrue(first["ok"], first)
        with d.lock:
            d.stall[auto.T_PHOTO] = (False, 0.2)
        r = self.photo()
        self.assertEqual((r["ok"], r["result"]), (False, "UNKNOWN"), r)

    def test_taken_unknown_arm_with_a_stale_snapshot_is_unknown(self):
        """WP3-4: an earlier ARM timed out (lastCmd ARM OK, back in IDLE); the next ARM is answered BUSY
        taken-unknown with the stabilizer stalled. Its pushes carry that old snapshot (lastCmd ARM OK): not
        newer than the ARM, so never read as done."""
        d = self.drone(**FAST, arm_timeout=0.3)
        dm, _ = self.start_daemon(d)
        self.assertEqual(self.cli("arm", "--mode", "dry")[0], 0)
        self.assertTrue(wait_for(lambda: d.state == IDLE and dm.last_state == auto.S_IDLE, 3))
        with d.lock:
            d.stall[auto.T_ARM] = (False, 1.6)
        code, out, _, _ = self.cli("arm", "--mode", "dry")
        self.assertEqual((code, out["ok"], out["result"]), (1, False, "UNKNOWN"), out)
        self.assertEqual(len(d.got(auto.T_ARM)), 2)

    def test_taken_unknown_takeoff_with_a_stale_snapshot_is_unknown(self):
        d = self.drone(**FAST, stall={auto.T_TAKEOFF: (False, 1.5)})
        self.start_daemon(d)
        self.assertEqual(self.cli("arm", "--mode", "dry")[0], 0)
        code, out, _, _ = self.cli("takeoff", "--photos", "0")
        self.assertEqual((code, out["ok"], out["result"]), (1, False, "UNKNOWN"))
        self.assertEqual(out["auto"]["stateName"], "ARMED")
        self.assertEqual(len(d.got(auto.T_TAKEOFF)), 1)
        self.assertEqual(self.cli("disarm")[0], 0)

    def test_photo_accepted_without_a_set(self):
        """D2: PHOTO OK with detail 0 (the camera could not start a set): no set, answered at once."""
        d = self.drone(**dict(FAST, wait_ms=8000), photo_no_set=True)
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        t0 = time.monotonic()
        r = self.photo()
        self.assertLess(time.monotonic() - t0, 1.5)
        self.assertFalse(r["ok"])
        self.assertIsNone(r["id"])
        self.assertIn("no photo set", r["error"])

    def test_default_sortie_takes_one_photo(self):
        """D3: the default sortie asks for 1 take-off photo, from the command line and from the daemon."""
        d = self.drone(**dict(FAST, wait_ms=5000))
        self.start_daemon(d)
        out = self.sortie_to_wait()
        _, takeoff = d.got(auto.T_TAKEOFF)[0]
        self.assertEqual(struct.unpack("<IHBBHH", takeoff.payload)[2], 1)
        self.assertEqual(len(out["photos"]), 1)
        self.assertEqual(self.cli("land")[0], 0)
        self.assertEqual(self.cli("wait", "--until", "idle", "--timeout", "5")[0], 0)
        n = len(d.got(auto.T_TAKEOFF))
        r = auto.daemon_request(self.sock, {"op": "sortie", "mode": "dry"}, 15)     # no "photos" key
        self.assertTrue(r["ok"], r)
        _, takeoff = d.got(auto.T_TAKEOFF)[n]
        self.assertEqual(struct.unpack("<IHBBHH", takeoff.payload)[2], 1)

    def test_one_shot_photo_frees_only_a_held_set(self):
        """D1 without a daemon: a BUSY held_set names the set to free (A1 behaviour)."""
        d = self.drone()
        with d.lock:
            d.start_set(30, 1, 150, time.monotonic(), fsm=False)
        keyf = self.key_file()
        code, out, _, err = self.run_main(["photo", "--host", "127.0.0.1", "--port", str(d.port), "--key-file", keyf,
                                           "--sock", os.path.join(self.tmp.name, "none.sock"),
                                           "--out", os.path.join(self.tmp.name, "o")])
        self.assertEqual((code, out["ok"], out["id"]), (0, True, 20), err)
        self.assertIn(30, d.freed)


class DaemonStopTest(DaemonCase):
    """WP3-6: LAND never waits behind another command and is sent until the drone confirms it."""

    def test_land_preempts_a_pending_command(self):
        """LAND goes out at once while a TAKEOFF is pending on the drone (answered BUSY pending), is sent again
        until ACKed OK, and the TAKEOFF is never sent again after it."""
        d = self.drone(**dict(FAST, hover_at=2.0), slow={auto.T_TAKEOFF: 0.25})
        self.start_daemon(d)
        self.assertEqual(self.cli("arm", "--mode", "dry")[0], 0)
        res = {}
        th = threading.Thread(target=lambda: res.update(r=auto.daemon_request(
            self.sock, {"op": "takeoff", "photos": 0}, 5)))
        th.start()
        self.assertTrue(wait_for(lambda: d.got(auto.T_TAKEOFF), 2, 0.001))
        t_tk = d.got(auto.T_TAKEOFF)[0][0]
        t_land = time.monotonic()
        code, out, _, err = self.cli("land")
        th.join(5)
        self.assertEqual((code, out["ok"], out["result"]), (0, True, "OK"), (out, err))
        lands = d.got(auto.T_LAND)
        self.assertLess(lands[0][0], t_tk + 0.25, "LAND waited for the TAKEOFF's answer")
        self.assertGreaterEqual(len(lands), 2)
        self.assertFalse([p for t, p in d.got(auto.T_TAKEOFF) if t > t_land], "TAKEOFF sent after the LAND")
        self.assertTrue(res["r"]["ok"], res)
        self.assertIn("LAND_CMD", [r for _, _, r in d.timeline])

    def test_land_is_sent_until_acked_through_busy(self):
        """Answered BUSY six times (more than any other command may be): still sent until ACKed OK."""
        d = self.drone(**dict(FAST, wait_ms=8000), land_busy=6)
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "0")
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))
        self.assertIn(out["confirmed"], ("ACK", "AUTO_STATUS"))
        self.assertGreaterEqual(len(d.got(auto.T_LAND)), 7)
        self.assertEqual([r for _, s, r in d.timeline if s == "LAND"], ["LAND_CMD"])

    def test_land_confirmed_by_auto_status_when_its_acks_are_lost(self):
        d = self.drone(**dict(FAST, wait_ms=8000, land_s=1.0), mute_acks={auto.T_LAND})
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "0")
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["ok"], out["confirmed"], out["state"]), (0, True, "AUTO_STATUS", "LAND"))
        self.assertEqual([r for _, s, r in d.timeline if s == "LAND"], ["LAND_CMD"])

    def test_stop_is_sent_after_its_client_went_away(self):
        """STOP is the daemon's (2026-10-15, docs/autonomy-a2b.md 22): queued behind a MOVE pending on the drone and
        its client gone at once, it still goes out (a client's command would be withdrawn)."""
        d = self.drone(**dict(IHFAST, hr=True, mv_speed=0.5), slow={auto.T_MOVE: 0.5})
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "0")
        th = threading.Thread(target=lambda: auto.daemon_request(self.sock, {"op": "move", "fwd": 1.0, "wait": False},
                                                                 10))
        th.start()
        self.assertTrue(wait_for(lambda: d.got(auto.T_MOVE), 2, 0.001))
        raw_client(self.sock, {"op": "stop"}).close()
        self.assertTrue(wait_for(lambda: d.got(auto.T_STOP), 3), "the STOP was withdrawn with its client")
        th.join(10)
        self.assertTrue(wait_for(lambda: d.mv_state == auto.MV_STOPPED, 3), d.mv_state)
        self.cli("land")

    def test_land_in_a_sink_until_taken_over(self):
        """A LAND SINK (the camera height out 5 s) may resume: an AUTO_STATUS in LAND SINK confirms no LAND (2026-10-15
        verification). The first four LANDs lost: the daemon sends it again until the drone takes it over (reason
        LAND_CMD, never resumed); 'wait --until land' does not return on the SINK itself."""
        d = self.drone(**dict(IHFAST, hr=True, land_s=1.0))
        dm, _ = self.start_daemon(d)
        self.sortie_to_wait("--photos", "0")
        with d.lock:
            d.to(LAND, rs("SINK"), time.monotonic())
        self.assertTrue(wait_for(lambda: dm.auto["state"] == auto.S_LAND and dm.auto["reason"] == auto.RS_SINK, 2))
        code, out, _, _ = self.cli("wait", "--until", "land", "--timeout", "0.6")
        self.assertEqual((code, out["ok"]), (1, False), out)    # a SINK is not a landing (it may resume)
        d.drop_land = 4
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.assertGreaterEqual(len(d.got(auto.T_LAND)), 5)      # sent until one went through
        self.assertEqual((d.state, d.reason), (LAND, rs("LAND_CMD")))
        code, out, _, _ = self.cli("wait", "--until", "land", "--timeout", "2")
        self.assertEqual((code, out["ok"]), (0, True), out)

    def test_land_after_taken_unknown_is_sent_again(self):
        d = self.drone(**dict(FAST, wait_ms=8000), stall={auto.T_LAND: (False, 0.2)})
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "0")
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))
        self.assertGreaterEqual(len(d.got(auto.T_LAND)), 2)
        self.assertEqual([r for _, s, r in d.timeline if s == "LAND"], ["LAND_CMD"])

    def test_lost_photo_free_is_sent_again_when_the_set_is_still_held(self):
        """Every PHOTO_FREE of the take-off set lost: the daemon dropped the set. A PHOTO then refused BUSY
        held_set with that id, in HOVER, frees it again and then works."""
        d = self.drone(**dict(FAST, wait_ms=9000), drop_free=5)
        self.start_daemon(d)
        self.sortie_to_wait("--photos", "1", "--gap", "150")
        r = auto.daemon_request(self.sock, {"op": "photo", "n": 1, "gap": 150, "timeout": 6}, 12)
        self.assertTrue(r["ok"], r)
        self.assertEqual(r["id"], 20)
        frees = [p for _, p in d.got(auto.T_PHOTO_FREE) if struct.unpack("<H", p.payload)[0] == 7]
        self.assertGreaterEqual(len(frees), 6)
        self.assertIn(7, d.freed)


class SubprocessDaemonTest(DaemonCase):
    """The daemon as its own process, for SIGKILL and SIGSTOP."""

    def spawn(self, d, nohup=False):
        self.keyf = self.key_file()
        self.sock = os.path.join(self.tmp.name, "p.sock")
        env = dict(os.environ, HOME=os.path.join(self.tmp.name, "home"))
        self.log_path = os.path.join(self.tmp.name, "daemon.log")
        log = open(self.log_path, "w")
        self.addCleanup(log.close)
        p = subprocess.Popen((["nohup"] if nohup else []) +
                             [sys.executable, TOOL, "daemon", "--host", "127.0.0.1", "--port", str(d.port),
                              "--key-file", self.keyf, "--sock", self.sock, "--photos",
                              os.path.join(self.tmp.name, "photos"), "--events",
                              os.path.join(self.tmp.name, "events.jsonl"), "--no-tty"],
                             stdin=subprocess.DEVNULL, stdout=log, stderr=log, env=env)

        def end():
            try:
                os.kill(p.pid, signal.SIGCONT)
            except OSError:
                pass
            p.kill()
            p.wait(5)
        self.addCleanup(end)

        def ready():
            try:
                return auto.daemon_request(self.sock, {"op": "ping"}, 0.5)["ok"]
            except auto.DaemonError:
                return False
        self.assertTrue(wait_for(ready, 10, 0.05), "the daemon did not start")
        return p

    def test_sigkill_lands_hb_lost(self):
        """12.4 #7: the daemon killed: the drone sees the heartbeat stop and lands HB_LOST 1.0-1.2 s later."""
        d = self.drone(**dict(FAST, wait_ms=8000))
        p = self.spawn(d)
        code, out, _, err = self.cli("sortie", "--mode", "dry", "--photos", "1", "--gap", "150")
        self.assertEqual((code, out["state"]), (0, "HOVER"), err)
        os.kill(p.pid, signal.SIGKILL)
        self.assertTrue(wait_for(lambda: d.hb_lost is not None, 3))
        last_hb, landed = d.hb_lost
        self.assertTrue(1.0 <= landed - last_hb <= 1.2, landed - last_hb)
        self.assertIn(("LAND", "HB_LOST"), [tl[1:] for tl in d.timeline])

    def test_one_shots_refuse_a_stopped_daemon(self):
        """12.4 #10: the daemon socket is there but the daemon is stopped: the one-shots refuse and send nothing."""
        d = self.drone()
        p = self.spawn(d)
        os.kill(p.pid, signal.SIGSTOP)
        time.sleep(0.2)
        n = len(d.log)
        for argv in (("status",), ("hello",), ("photo", "--out", os.path.join(self.tmp.name, "o"))):
            code, out, _, _ = self.oneshot(d, *argv)
            self.assertEqual((code, out["ok"]), (1, False), argv)
            self.assertIn("daemon socket present but not answering; refusing to open a new session", out["error"])
        self.assertEqual(len(d.log), n)
        self.assertEqual(d.hellos, 1)
        os.kill(p.pid, signal.SIGCONT)
        self.assertTrue(wait_for(lambda: len(d.log) > n, 2))      # the heartbeat resumes
        self.assertEqual(d.hellos, 1)

    def test_sighup_lands_like_q(self):
        """WP3-7: the daemon's terminal closed (SIGHUP) is the q path: LAND (LAND_CMD) at once, then the
        daemon exits after the touchdown; not HB_LOST."""
        d = self.drone(**dict(FAST, wait_ms=8000))
        p = self.spawn(d)
        code, out, _, err = self.cli("sortie", "--mode", "dry", "--photos", "0")
        self.assertEqual((code, out["state"]), (0, "HOVER"), err)
        os.kill(p.pid, signal.SIGHUP)
        self.assertEqual(p.wait(5), 0)
        self.assertEqual([r for _, s, r in d.timeline if s == "LAND"], ["LAND_CMD"])
        self.assertNotIn("HB_LOST", [r for _, _, r in d.timeline])
        with open(self.log_path) as f:
            self.assertIn("closing its terminal (SIGHUP) lands first", f.read())

    @unittest.skipUnless(shutil.which("nohup"), "needs nohup")
    def test_sighup_ignored_under_nohup(self):
        """Started under nohup (SIGHUP already ignored), the daemon keeps ignoring it: the sortie goes on and
        the heartbeat continues; SIGTERM is then the q path (LAND first, then exit)."""
        d = self.drone(**dict(FAST, wait_ms=8000))
        p = self.spawn(d, nohup=True)
        code, out, _, err = self.cli("sortie", "--mode", "dry", "--photos", "0")
        self.assertEqual((code, out["state"]), (0, "HOVER"), err)
        t_hup = time.monotonic()
        os.kill(p.pid, signal.SIGHUP)
        time.sleep(1.5)
        self.assertIsNone(p.poll(), "the daemon quit on SIGHUP under nohup")
        self.assertNotIn("LAND", [s for _, s, _ in d.timeline])
        self.assertGreaterEqual(len(d.got(auto.T_HB, since=t_hup + 1.0)), 3)
        os.kill(p.pid, signal.SIGTERM)
        self.assertEqual(p.wait(5), 0)
        self.assertEqual([r for _, s, r in d.timeline if s == "LAND"], ["LAND_CMD"])
        with open(self.log_path) as f:
            self.assertIn("SIGHUP ignored (started under nohup)", f.read())



# ================================================================ the interactive hover (docs/autonomy-a2b.md section 14)

GOLDEN_IH_PATH = os.path.join(HERE, "auto_proto_golden_ih.json")
with open(GOLDEN_IH_PATH) as _f:
    GOLDEN_IH = json.load(_f)


class GoldenIHTest(unittest.TestCase):
    """MOVE, YAW, STOP, TAKEOFF with the sortie's wait and the 96-byte AUTO_STATUS, against the vectors
    test_auto_proto.c checks the firmware with (tools/tests/auto_proto_golden_ih.json)."""

    def test_packets_from_fields(self):
        S, A = SESSION, auto.FLAG_ACKREQ
        g = {k: auto.decode(KEY, bytes.fromhex(v)) for k, v in GOLDEN_IH["packets"].items()}
        built = {
            "move": auto.encode(KEY, auto.T_MOVE, A, S, 30, auto.move_payload(50, -120, 20)),
            "move_left_down": auto.encode(KEY, auto.T_MOVE, A, S, 31, auto.move_payload(-150, 0, -50)),
            "yaw": auto.encode(KEY, auto.T_YAW, A, S, 32, auto.yaw_payload(90)),
            "yaw_left": auto.encode(KEY, auto.T_YAW, A, S, 33, auto.yaw_payload(-180)),
            "stop": auto.encode(KEY, auto.T_STOP, A, S, 34),
            "takeoff_wait": auto.encode(KEY, auto.T_TAKEOFF, A, S, 35, auto.takeoff_payload(0x5EED0002, 100, 1, 300,
                                                                                            60000)),
            "ack_move_refused": auto.encode(KEY, auto.T_ACK, 0, S, 36, auto.ack_payload(30, auto.T_MOVE, 5, 3,
                                                                                       3 | 5 << 8 | 3 << 16)),
            "auto_status_ih": auto.encode(KEY, auto.T_AUTO_STATUS, 0, S, 37, auto.auto_status_payload(
                auto.parse_auto_status(g["auto_status_ih"].payload), auto.AUTO_STATUS_LEN_IH)),
            # waits over 65.5 s (2026-10-14): TAKEOFF byte 7 = actWaitMs bits 16-23
            "takeoff_wait_65536": auto.encode(KEY, auto.T_TAKEOFF, A, S, 38,
                                              auto.takeoff_payload(0x5EED0003, 105, 1, 300, 65536)),
            "takeoff_wait_120000": auto.encode(KEY, auto.T_TAKEOFF, A, S, 39,
                                               auto.takeoff_payload(0x5EED0004, 105, 1, 300, 120000)),
            "auto_status_ih120": auto.encode(KEY, auto.T_AUTO_STATUS, 0, S, 40, auto.auto_status_payload(
                auto.parse_auto_status(g["auto_status_ih120"].payload), auto.AUTO_STATUS_LEN_IH)),
            # hold and report (2026-10-15): 136 bytes
            "auto_status_hr": auto.encode(KEY, auto.T_AUTO_STATUS, 0, S, 42, auto.auto_status_payload(
                auto.parse_auto_status(g["auto_status_hr"].payload), auto.AUTO_STATUS_LEN_HR)),
            # the speed step (2026-10-16): 140 bytes, mvspeed 200
            "auto_status_ms": auto.encode(KEY, auto.T_AUTO_STATUS, 0, S, 50, auto.auto_status_payload(
                auto.parse_auto_status(g["auto_status_ms"].payload))),
            # the take-off's planned move (2026-10-14): 18 bytes, left 80 forward 250, a 90 s wait
            "takeoff_plan": auto.encode(KEY, auto.T_TAKEOFF, A, S, 41, auto.takeoff_payload(0x5EED0005, 100, 1, 300,
                                                                                            90000, -80, 250)),
            # SET_HEIGHT (0x33, 2026-10-15): judged 1.10 m on a photo at 0.95 m; the absolute form 0.80 m; the ACKs
            # (detail: state | why << 8 | the old estimate cm << 16)
            "set_height": auto.encode(KEY, auto.T_SET_HEIGHT, A, S, 43, auto.set_height_payload(110, 95)),
            "set_height_now": auto.encode(KEY, auto.T_SET_HEIGHT, A, S, 44, auto.set_height_payload(80)),
            "ack_set_height": auto.encode(KEY, auto.T_ACK, 0, S, 45, auto.ack_payload(43, auto.T_SET_HEIGHT, 0, 3,
                                                                                      3 | 100 << 16)),
            "ack_set_height_refused": auto.encode(KEY, auto.T_ACK, 0, S, 46, auto.ack_payload(
                44, auto.T_SET_HEIGHT, 3, 3, 3 | auto.MV_WHY_NAMES.index("SH_STEP") << 8 | 98 << 16)),
            # SET_HEIGHT's target form (2026-10-16): 6 bytes, toCm -32768 (the commanded height) or 120
            "set_height_to": auto.encode(KEY, auto.T_SET_HEIGHT, A, S, 48, auto.set_height_payload(110, 90,
                                                                                                  auto.SH_TO_REF)),
            "set_height_to_120": auto.encode(KEY, auto.T_SET_HEIGHT, A, S, 49, auto.set_height_payload(90, auto.SH_AT_NOW,
                                                                                                      120)),
            # a colour photo's META (2026-10-15): 44 bytes, pixFmt YUYV
            "meta_color": auto.encode(KEY, auto.T_META, 0, S, 47, auto.meta_payload(meta_for(
                colour_frame(), 6, 0, 1, frameSeq=4400, captureUpMs=99000, pixFmt=auto.PIXFMT_YUYV))),
        }
        self.assertEqual(GOLDEN_IH["auto_status_fmt"], auto.AUTO_STATUS_FMT + auto.AUTO_STATUS_EXT_FMT[1:])
        self.assertEqual(GOLDEN_IH["auto_status_fmt_hr"],
                         auto.AUTO_STATUS_FMT + auto.AUTO_STATUS_EXT_FMT[1:] + auto.AUTO_STATUS_HR_FMT[1:])
        self.assertEqual(set(built), set(GOLDEN_IH["packets"]))
        for name, pkt in built.items():
            with self.subTest(name=name):
                self.assertEqual(pkt.hex(), GOLDEN_IH["packets"][name])

    def test_parse(self):
        g = {k: auto.decode(KEY, bytes.fromhex(v)) for k, v in GOLDEN_IH["packets"].items()}
        a = auto.parse_ack(g["ack_move_refused"].payload)
        self.assertEqual((a["resultName"], a["stateName"], auto.mv_detail(a["detail"])), ("CAM_UNAVAIL", "HOVER",
                                                                                          (3, 5, 3)))
        self.assertEqual(auto.MV_WHY_NAMES[5], "NO_DIST")
        s = auto.parse_auto_status(g["auto_status_ih"].payload)
        self.assertTrue(s["interactive"])
        self.assertEqual((s["stateName"], s["lastCmdName"], s["mvStateName"], s["mvKindName"], s["mvWhyName"],
                          s["mvEndName"], s["mvFlagNames"], s["mvSeq"], s["mvActive"], s["battLow"]),
                         ("HOVER", "MOVE", "RUN", "MOVE", "NONE", "NONE", ["dValid"], 3, True, False))
        self.assertEqual(s["prog_m"], {"right": 0.21, "fwd": -0.48, "up": 0.0})
        self.assertEqual(s["tgt_m"], {"right": 0.5, "fwd": -1.2, "up": 0.0})
        self.assertEqual((s["sceneD_m"], s["hTarget_m"], s["yawOff_deg"], s["yawTgtDeg"], s["airLeft_s"]),
                         (2.12, 1.0, -90.0, -90, 38.8))
        # the air clock: airMs while it is exact (below its 65.5 s saturation). The golden bytes are an 80 s clock's
        # (airMs 41200 + airLeftDs 388 x 100 = 80000): a drone of another version, flagged; 120 s - airLeftDs would
        # read 81.2 s (the verification of 2c69fc5)
        self.assertAlmostEqual(s["air_s"], 41.2)
        self.assertEqual(s["airMs"], 41200)
        self.assertTrue(s["airClockMismatch"])
        p = bytearray(g["auto_status_ih"].payload)
        struct.pack_into("<H", p, 94, (auto.AIR_HARD_MS - 41200) // 100)   # this version's airLeftDs then: 788
        s = auto.parse_auto_status(bytes(p))
        self.assertEqual((s["air_s"], s["airLeft_s"], s["airClockMismatch"]), (41.2, 78.8, False))
        for ms in (100, 5099, 65534):    # airLeftDs rounds down to 0.1 s: never a mismatch
            struct.pack_into("<H", p, 30, ms)
            struct.pack_into("<H", p, 94, (auto.AIR_HARD_MS - ms) // 100)
            s = auto.parse_auto_status(bytes(p))
            self.assertEqual((s["air_s"], s["airClockMismatch"]), (ms / 1000.0, False), ms)
        # at the 120 s limit airLeftDs is 0 and airMs saturated: the air clock is the limit, not 65.5 s
        p = bytearray(g["auto_status_ih"].payload)
        struct.pack_into("<H", p, 30, 65535)
        struct.pack_into("<H", p, 94, 0)
        s = auto.parse_auto_status(bytes(p))
        self.assertEqual((s["air_s"], s["airLeft_s"]), (auto.AIR_HARD_MS / 1000.0, 0.0))
        self.assertEqual(auto.AIR_HARD_MS, 120000)
        struct.pack_into("<H", p, 30, 0)
        self.assertEqual(auto.parse_auto_status(bytes(p))["air_s"], 0.0)   # outside the air
        # a long sortie (2026-10-14): the u16 ms fields saturated, the air clock from airLeftDs
        s = auto.parse_auto_status(g["auto_status_ih120"].payload)
        self.assertEqual((s["airMs"], s["waitLeftMs"], s["hoverWaitMs"], s["airLeftDs"]), (65535, 65535, 65535, 150))
        self.assertAlmostEqual(s["air_s"], 105.0)
        self.assertFalse(s["airClockMismatch"])   # saturated: not decidable
        self.assertTrue(s["waiting"] and s["waitLeftSaturated"])
        self.assertEqual((s["mvStateName"], s["mvEndName"], s["lastCmdName"], s["hTarget_m"]), ("DONE", "STOP_CMD",
                                                                                            "STOP", 1.05))
        self.assertFalse(auto.parse_auto_status(g["auto_status_ih"].payload)["waitLeftSaturated"])
        self.assertFalse(auto.parse_auto_status(g["auto_status_ih"].payload)["hr"])
        # hold and report (2026-10-15, 136 bytes): a drone holding after a contact, the camera height lost
        s = auto.parse_auto_status(g["auto_status_hr"].payload)
        self.assertTrue(s["interactive"] and s["hr"])
        self.assertEqual((s["mvStateName"], s["mvEndName"], s["airLeftDs"], s["airClockMismatch"]),
                         ("STOPPED", "CONTACT", 0, False))
        self.assertAlmostEqual(s["air_s"], 41.2)   # airMs exact (airDs 412 agrees)
        h = s["hold"]
        self.assertEqual([e["name"] for e in h["events"]], ["CAM_LOST", "CONTACT", "MOVE_STOPPED"])
        self.assertEqual(h["events"][0]["detailName"], ["noG"])
        self.assertEqual(h["events"][1]["detailName"], {"phase": "HOLD", "noHush": False, "motionStopped": True})
        self.assertEqual(h["events"][2]["detailName"], "CONTACT")
        self.assertEqual((h["evCount"], h["bits"], h["camBits"], h["camOff_s"], h["sinkIn_s"], h["contacts"]),
                         (7, ["camLost", "openLoop"], ["noG"], 1.8, 3.2, 1))
        self.assertEqual((h["khatRel"], h["capQ"], h["sinceCmd_s"]), (1.03, 47120, 12.4))
        c = h["contact"]
        self.assertEqual((c["peakAcc_g"], c["peakRate_dps"], c["force_g"]), (2.31, 310, [-2.05, 0.98]))
        # pushed back (-x) and to the left (+y): hit from ahead and to the right of the heading then, atan2(0.98,
        # 2.05) = +25.6 deg; the drone has not turned since (yawOff -90 = contactYaw -90)
        self.assertEqual((c["bearing_at_contact_deg"], c["turned_since_deg"], c["obstacle_bearing_deg"]),
                         (25.6, 0.0, 25.6))
        p = bytearray(g["auto_status_hr"].payload)
        struct.pack_into("<h", p, 88, 0)   # yawOffCdeg 0 now: the drone turned 90 deg clockwise since the contact
        self.assertEqual(auto.parse_auto_status(bytes(p))["hold"]["contact"]["obstacle_bearing_deg"], -64.4)
        struct.pack_into("<H", p, 30, 65535)   # airMs saturated: the air time from airDs, no air clock
        self.assertAlmostEqual(auto.parse_auto_status(bytes(p))["air_s"], 41.2)
        t = g["takeoff_wait_120000"].payload
        self.assertEqual(struct.unpack("<IHBBHH", t), (0x5EED0004, 105, 1, 1, 300, 0xD4C0))


IHFAST = dict(FAST, wait_ms=4000)


class DaemonLongMoveTest(DaemonCase):
    """Long moves, the person gap, the battery's duration, the move planned from take-off and wait-move (2026-10-14,
    docs/autonomy-a2b.md 21). The fake drone is this tool's build (forward <= MOVE_FWD_MAX_M; the 18-byte TAKEOFF
    with the long-move build) unless a test says otherwise. Its person-gap numbers were worked out at a 1.0 m floor;
    the tool's floor is 0.5 m since 2026-10-17 (the user), so the class runs at 1.0 and test_limits reads the real one."""

    def setUp(self):
        super().setUp()
        self.real_person_min = auto.PERSON_MIN_M
        p = mock.patch.object(auto, "PERSON_MIN_M", 1.0)
        p.start()
        self.addCleanup(p.stop)

    def up(self, *extra, **kw):
        d = self.drone(**dict(IHFAST, **kw))
        self.start_daemon(d)
        self.sortie_to_wait("--height", "100", "--photos", "1", "--wait-s", "60", *extra)
        return d

    def test_limits(self):
        self.assertEqual((auto.MOVE_HORIZ_MAX_M, auto.MOVE_LEN_MAX_M, self.real_person_min), (1.5, 4.0, 0.5))
        self.assertIn(auto.MOVE_FWD_MAX_M, (1.5, 4.0))
        R = auto.move_range_refusal
        self.assertIsNone(R(150, 0, 0))
        self.assertIsNotNone(R(151, 0, 0))
        self.assertIsNotNone(R(0, -151, 0))
        self.assertIsNone(R(0, -150, 50))
        self.assertIsNone(R(0, int(auto.MOVE_FWD_MAX_M * 100), 0))
        self.assertIsNotNone(R(0, int(auto.MOVE_FWD_MAX_M * 100) + 1, 0))
        self.assertIsNotNone(R(150, 371, 0, fwd_top=4.0))       # 401.2 cm
        self.assertIsNone(R(150, 370, 0, fwd_top=4.0))
        self.assertIsNotNone(R(0, 151, 10, fwd_top=4.0))        # up with forward > 1.5
        # the profile times toward the scene: 1.5 / 2 / 2.5 / 3 / 3.5 / 4 m: 9.5 / 12 / 14.5 / 17 / 19.5 / 22 s
        for f, t in ((1.5, 9.5), (2.0, 12.0), (2.5, 14.5), (3.0, 17.0), (3.5, 19.5), (4.0, 22.0)):
            self.assertAlmostEqual(auto.move_profile_s(0.0, f, 0.0, 8.0), t, places=6)
        self.assertAlmostEqual(auto.Daemon.move_timeout(0.0, 4.0, 0.0, 8.0), 37.0, places=6)
        self.assertAlmostEqual(auto.batt_seconds_left(3.10), 20.0)
        self.assertIsNone(auto.batt_seconds_left(3.25))
        self.assertAlmostEqual(auto.fwd_max(3.0, auto.MAC_MIN_FRONT_M), min(auto.MOVE_FWD_MAX_M, 2.17))
        self.assertAlmostEqual(auto.Daemon.move_timeout(0.0, 1.5, 0.0, 8.0), 25.0, places=6)   # the old basis
        if auto.MOVE_FWD_MAX_M >= 4.0:
            # whole cm, k 0.6: the Mac's floor 0.35 m, the drone's 0.30 m (D ((D - F) / D)^(1/0.6) >= floor; 0.60 /
            # 0.50 until 2026-10-17: [1.02, 1.43, 1.85, 2.28, 2.71, 3.29, 3.59, 4.00] / [1.12, 1.54, 1.97, ...])
            ds = (2.0, 2.5, 3.0, 3.5, 4.0, 4.66, 5.0, 5.5)
            self.assertEqual([auto.fwd_max(d, auto.MAC_MIN_FRONT_M) for d in ds],
                             [1.29, 1.73, 2.17, 2.62, 3.07, 3.67, 3.98, 4.00])
            self.assertEqual([auto.fwd_max(d, auto.DRONE_MIN_FRONT_M) for d in ds],
                             [1.35, 1.79, 2.24, 2.69, 3.15, 3.76, 4.00, 4.00])

    def test_mac_floor_follows_the_drones(self):
        """The Mac's floor ahead is the drone's NVS mvfloor + 0.05 m as the last 'auto get' cached it (2026-10-17,
        docs/autonomy-a2b.md 32.5): 0.35 m without a cache or with an out-of-range value; mvfloor 45 gives 0.50 m."""
        self.assertEqual((auto.mac_min_front(), auto.cached_margins()), (0.35, {}))
        self.assertEqual(auto.MAC_MIN_FRONT_M, round(auto.DRONE_MIN_FRONT_M + auto.MAC_FRONT_EXTRA_M, 2))
        os.makedirs(auto.DEFAULT_DIR, exist_ok=True)

        def cache(**vals):
            with open(auto.vmin_cache_path(), "w") as f:
                json.dump({"utc": "2026-10-17T09:00:00Z",
                           "values": {k: {"value": str(v), "src": "nvs"} for k, v in vals.items()}}, f)

        cache(mvfloor=45, mvfstop=40, mvlatm=25)
        self.assertEqual(auto.mac_min_front(), 0.50)
        self.assertEqual(auto.cached_margins(), {"mvfloor": 45, "mvfstop": 40, "mvlatm": 25})
        d = self.up(scene_d=2.0)
        code, out, _, _ = self.cli("move", "--forward", "1.2")
        self.assertEqual((code, out["ok"], out["fwd_max_m"]), (1, False, 1.12), out)
        self.assertIn("the Mac keeps 0.50 m: at most 1.12 m now", out["error"])
        self.assertEqual(d.got(auto.T_MOVE), [])
        cache(mvfloor=5)
        self.assertEqual(auto.mac_min_front(), 0.35)
        cache(mvfloor=30)
        code, out, _, _ = self.cli("move", "--forward", "1.29")
        self.assertEqual((code, out["ok"]), (0, True), out)

    def test_guards_off_floor(self):
        """The guards down to off (user decision 2026-10-18, docs/autonomy-a2b.md 38): with mvfloor 0 cached the Mac
        refuses no forward move for the scene ahead (its "at most F m" clip is off; the drone's 0.05 m is all that is
        left, its TOO_CLOSE passed on), and 'state' lists what is off."""
        self.assertEqual((auto.mac_min_front(), auto.cached_k_low(), auto.cached_lat_rel()), (0.35, 0.6, 0.5))
        self.assertEqual(auto.guards_view(), {"mac_floor_m": 0.35, "k_low": 0.6, "lat_rel": 0.5, "mvwmax": 15})
        write_auto_get({"mvfloor": 0})
        self.assertEqual((auto.mac_min_front(), auto.fwd_max(2.0, auto.mac_min_front())), (0.0, 1.95))
        self.assertEqual(auto.guards_view()["off"], ["floor"])
        d = self.up(scene_d=2.0, mv_floor=0.0)
        code, out, _, _ = self.cli("move", "--forward", "1.9")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.assertEqual(len(d.got(auto.T_MOVE)), 1)
        code, out, _, _ = self.cli("move", "--forward", "1.97")
        self.assertEqual((code, out["ok"], out["why"]), (1, False, "TOO_CLOSE"), out)   # the drone's 0.05 m
        self.assertIn("0 = no floor but 0.05 m", out["error"])   # (the hint rides in the error text)
        code, out, _, _ = self.cli("state")
        self.assertEqual((out["guards"]["off"], out["guards"]["mac_floor_m"]), (["floor"], 0.0), out["guards"])

    def test_guards_off_klow(self):
        """mvklow gives the k of the Mac's floor check and its "at most F m now" (100: 1.60 m at D 2.0 ends 0.40 m; at
        k 0.6 it would end 0.14 m); out of range: the default 0.6."""
        write_auto_get({"mvklow": 100})
        self.assertEqual(auto.cached_k_low(), 1.0)
        d = self.up(scene_d=2.0, mv_klow=1.0)
        code, out, _, _ = self.cli("move", "--forward", "1.6")
        self.assertEqual((code, out["ok"]), (0, True), out)
        code, out, _, _ = self.cli("move", "--forward", "1.66")
        self.assertEqual((code, out["ok"], out["fwd_max_m"]), (1, False, 1.65), out)
        self.assertIn("k down to 1", out["error"])
        self.assertEqual(len(d.got(auto.T_MOVE)), 1)
        write_auto_get({"mvklow": 59})
        self.assertEqual(auto.cached_k_low(), 0.6)
        code, out, _, _ = self.cli("move", "--forward", "1.6")
        self.assertEqual((code, out["ok"], out["fwd_max_m"]), (1, False, 1.29), out)   # (the default k 0.6)

    def test_guards_off_lat(self):
        """mvlat 0 lifts the sideways 0.5 x D refusal (the 1.50 m limit stays); mvlat 100 is 1 x D."""
        write_auto_get({"mvlat": 0})
        self.assertIsNone(auto.cached_lat_rel())
        self.assertEqual(auto.guards_view()["off"], ["lat"])
        d = self.up(scene_d=1.0, mv_lat=0.0)
        code, out, _, _ = self.cli("move", "--right", "1.4")
        self.assertEqual((code, out["ok"]), (0, True), out)
        code, out, _, _ = self.cli("move", "--right", "1.6")
        self.assertEqual((code, out["ok"]), (2, False), out)   # (the command limit: the argument's range)
        self.assertIn("must be 0..1.5", out["error"])
        self.assertEqual(len(d.got(auto.T_MOVE)), 1)
        write_auto_get({"mvlat": 100})
        d.mv_lat = 1.0
        code, out, _, _ = self.cli("move", "--right", "1.0")
        self.assertEqual((code, out["ok"]), (0, True), out)
        code, out, _, _ = self.cli("move", "--right", "1.1")
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("more than 1 x D = 1.00 m", out["error"])
        write_auto_get({})
        code, out, _, _ = self.cli("move", "--right", "0.6")
        self.assertIn("more than 0.5 x D = 0.50 m", out["error"])

    def test_guards_off_everything_keeps_the_person_rule(self):
        """Everything off on the drone: the Mac's person rule stays, with its k 0.6 overshoot (1.5 m from D 2.0 may fly
        1.80 m: a 1.1 m gap at 2.6 m comes out 0.80 m, under this class's 1.0 m)."""
        write_auto_get({"mvfloor": 0, "mvfstop": 0, "mvprogm": 0, "mvprogr": 0, "mvklow": 100, "mvlat": 0})
        self.assertEqual(auto.guards_view()["off"], ["floor", "lat", "mvfstop", "mvprogm", "mvprogr"])
        d = self.up(scene_d=2.0, mv_floor=0.0, mv_klow=1.0, mv_lat=0.0)
        code, out, _, _ = self.cli("move", "--forward", "1.5", "--person-gap", "1.1", "--est", "2.6")
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("k down to 0.6", out["error"])
        self.assertEqual(d.got(auto.T_MOVE), [])
        code, out, _, _ = self.cli("move", "--forward", "1.5")
        self.assertEqual((code, out["ok"]), (0, True), out)

    def test_mvwmax_mirrors_the_drones(self):
        """The expansion rate cap mvwmax (2026-10-18): the drone's NVS value from the cache in mv_limits / move_profile
        (0.15 by default; at 300 % a cap over 0.17 is itself the cap), the default bit for bit as before."""
        self.assertEqual((auto.MV_WFWD_PCT, auto.MV_WFWD_DEFAULT_PCT / 100.0), ((10, 40), auto.MOVE_W_FWD_MAX))
        self.assertEqual(auto.fast_caps()["mvwmax"], 15)
        self.assertEqual([auto.mv_limits(p)["w_fwd"] for p in (100, 150, 300)], [0.15, 0.17, 0.17])
        t15, v15 = auto.move_profile(0.0, 1.35, 0.0, 2.0)
        self.assertAlmostEqual(v15, 0.0975, places=6)
        write_auto_get({"mvwmax": 30})
        caps = auto.fast_caps()
        self.assertEqual((caps["mvwmax"], caps["src"]), (30, "default"))   # (src: the two of 'real 300 %')
        self.assertEqual([auto.mv_limits(p, caps)["w_fwd"] for p in (100, 150, 300)], [0.30, 0.30, 0.30])
        t30, v30 = auto.move_profile(0.0, 1.35, 0.0, 2.0, 100, caps)
        self.assertAlmostEqual(v30, 0.195, places=6)
        self.assertLess(t30, t15)
        write_auto_get({"mvwmax": 41, "mvacc": 30, "mvsettle": 1000})
        caps = auto.fast_caps()
        self.assertEqual((caps["mvwmax"], caps["src"]), (15, "auto get"))   # out of range: the default
        self.assertEqual(auto.fast_caps_values({"mvwmax": 9}), (20, 2500, 0.10))

    def test_takeoff_plan_payload(self):
        p = auto.takeoff_payload(0x5EED0005, 100, 1, 300, 90000, -80, 250)
        self.assertEqual(len(p), auto.TAKEOFF_PLAN_LEN)
        self.assertEqual(struct.unpack("<IHBBHHhhH", p), (0x5EED0005, 100, 1, 1, 300, 90000 & 0xFFFF, -80, 250, 0))
        self.assertEqual(auto.takeoff_payload(0x5EED0005, 100, 1, 300, 90000), p[:12])   # no plan: the 12 of before

    def test_long_forward_move(self):
        d = self.up(scene_d=6.0)
        code, out, _, err = self.cli("move", "--forward", "3.5")
        if auto.MOVE_FWD_MAX_M < 3.5:
            self.assertEqual(code, 2, out)     # phase 1: the usage's range is the drone's 1.50
            return
        self.assertEqual((code, out["ok"], out["end"], out["profile_s"]), (0, True, "DONE", 19.5), (out, err))
        self.assertEqual([struct.unpack("<hhhH", p.payload)[:3] for _, p in d.got(auto.T_MOVE)], [(0, 350, 0)])

    def test_refusals_report_the_largest(self):
        d = self.up(scene_d=2.0)
        code, out, _, _ = self.cli("move", "--forward", str(min(2.5, auto.MOVE_FWD_MAX_M)))
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("at most 1.29 m now", out["error"])          # D 2.0: 0.35 m left at k 0.6 (1.2971 m, floored)
        self.assertEqual((out["fwd_max_m"], out["D_m"]), (1.29, 2.0))
        if auto.MOVE_FWD_MAX_M >= 3.8:
            code, out, _, _ = self.cli("move", "--right", "1.5", "--forward", "3.8")
            self.assertEqual((code, out["ok"]), (1, False))
            self.assertIn("the move's length 4.085 m is over 4.00 m", out["error"])
            code, out, _, _ = self.cli("move", "--up", "0.2", "--forward", "1.6")
            self.assertIn("a move up or down goes at most 1.50 m forward", out["error"])
        self.assertEqual(d.got(auto.T_MOVE), [])

    def test_person_gap(self):
        d = self.up(scene_d=4.0)
        code, out, _, _ = self.cli("move", "--forward", "1.0", "--person-gap", "0.9", "--est", "2.0")
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertIn("never ends closer than 1.0 m to a person", out["error"])
        code, out, _, _ = self.cli("move", "--forward", "1.0", "--person-gap", "1.6")
        self.assertEqual((code, out["ok"]), (1, False))
        self.assertIn("--person-gap needs --est", out["error"])
        # the drone's own travel at k 0.6 (2026-10-15 review): forward 1.0 m from D 4.0 may fly 1.52 m, so a 1.4 m
        # gap could end 0.88 m from the person (0.91 m forward keeps 1.0 m); 1.6 m keeps 1.08 m
        code, out, _, _ = self.cli("move", "--forward", "1.0", "--person-gap", "1.4", "--est", "2.6")
        self.assertEqual((code, out["ok"], out["person_low_m"]), (1, False, 0.88), out)
        self.assertIn("may fly 0.52 m beyond 1.00 m forward", out["error"])
        self.assertIn("keeps at most 0.91 m forward now", out["error"])
        self.assertEqual((out["fwd_max_m"], out["D_m"]), (0.91, 4.0))
        self.assertEqual(d.got(auto.T_MOVE), [])
        code, out, _, err = self.cli("move", "--forward", "1.0", "--person-gap", "1.6", "--est", "2.6")
        self.assertEqual((code, out["ok"], out["person_gap_m"], out["est_m"], out["person_low_m"]),
                         (0, True, 1.6, 2.6, 1.08), (out, err))
        ev = self.events("person_gap")
        self.assertEqual((len(ev), ev[0]["gap_m"], ev[0]["est_m"], ev[0]["D_m"]), (1, 1.6, 2.6, 4.0))

    def test_person_gap_long_move(self):
        # the review's case: E = D = 4.7 m, the runbook's gap 1.0 + 0.15 E = 1.70 m, F 3.0 m: 3.84 m at k 0.6, 0.86 m
        # from the person; the largest forward part that keeps 1.0 m is 2.84 m
        self.assertAlmostEqual(auto.fwd_overshoot(4.7, 3.0), 0.837, places=3)
        self.assertEqual(auto.person_fwd_max(4.7, 0.0, 3.0, 1.7, 4.7), 2.84)
        low, _ = auto.person_gap_low(4.7, 0.0, 2.84, 1.86, 4.7)
        self.assertGreaterEqual(low, 1.0)
        self.assertLess(auto.person_gap_low(4.7, 0.0, 2.85, 1.85, 4.7)[0], 1.0)
        # a diagonal: the sideways part adds to the farthest the drone may get (the triangle bound)
        self.assertLess(auto.person_gap_low(4.7, 1.0, 2.84, 1.86, 4.7)[0], low)
        # a back or sideways move has no forward overshoot
        low, over = auto.person_gap_low(4.7, 1.0, -0.5, 2.0, 3.0)
        self.assertEqual((low, over), (2.0, 0.0))                     # back and sideways: away from a person ahead
        low, over = auto.person_gap_low(4.7, 1.0, 0.0, 2.0, 2.5)        # sideways 1.0: at most sin 38 deg nearer
        self.assertAlmostEqual(low, 2.5 - math.sin(math.radians(38.0)))
        self.assertEqual(auto.toward_person_max(0.5, 1.0), math.hypot(0.5, 1.0))   # 26.6 deg: inside the view
        if auto.MOVE_FWD_MAX_M < 3.0:
            return
        d = self.up(scene_d=4.7)
        code, out, _, _ = self.cli("move", "--forward", "3.0", "--person-gap", "1.7", "--est", "4.7")
        self.assertEqual((code, out["ok"], out["fwd_max_m"], out["person_low_m"]), (1, False, 2.84, 0.86), out)
        code, out, _, err = self.cli("move", "--forward", "2.84", "--person-gap", "1.86", "--est", "4.7")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        self.assertEqual([struct.unpack("<hhhH", p.payload)[:3] for _, p in d.got(auto.T_MOVE)], [(0, 284, 0)])

    def test_battery_duration(self):
        f = min(3.0, auto.MOVE_FWD_MAX_M)
        need = auto.move_profile_s(0.0, f, 0.0, 6.0) + auto.settle_s()
        mv = int(round((auto.BATT_END_V + auto.BATT_DUR_VPS * (need - 4.0)) * 1000))   # 4 s short of it
        d = self.up(scene_d=6.0, vbat_mv=mv)
        code, out, _, _ = self.cli("move", "--forward", str(f))
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("the longest along this line that fits is", out["error"])
        self.assertIn("--batt-ok", out["error"])
        self.assertLess(out["longest_m"], f)
        self.assertAlmostEqual(auto.move_profile_s(0.0, out["longest_m"], 0.0, 6.0) + auto.settle_s(),
                               out["battery_s"], delta=0.15)
        self.assertEqual(d.got(auto.T_MOVE), [])
        code, out, _, err = self.cli("move", "--forward", str(f), "--batt-ok")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))

    def test_progress_at_the_end_is_reached(self):
        self.up(scene_d=4.0, mv_end=("PROGRESS", 0.95))
        code, out, _, err = self.cli("move", "--forward", "1.0")
        self.assertEqual((code, out["ok"], out["end"], out["reached"]), (0, True, "PROGRESS", True), (out, err))
        self.assertIn("reached (stopped at the end by the guard", out["note"])
        self.assertNotIn("error", out)

    def test_progress_past_its_end_is_not_reached(self):
        # the review's case: a 1.13 m move from D 4.66 m stopped in its settle by the forward margin's ahead side at
        # 137 % along it (1.55 m of travel): an overshoot, not 'reached'
        self.up(scene_d=4.66, mv_end=("PROGRESS", 1.37))
        code, out, _, _ = self.cli("move", "--forward", "1.13")
        self.assertEqual((code, out["ok"], out["end"], out["along"]), (1, False, "PROGRESS", 1.372), out)
        self.assertNotIn("reached", out)
        self.assertIn("overshoot by 0.42 m", out["error"])
        self.assertIn("take a photo", out["error"])
        code, out, _, _ = self.cli("wait-move")
        self.assertEqual((code, out["ok"], out["end"]), (1, False, "PROGRESS"), out)
        self.assertIn("overshoot by 0.42 m", out["error"])

    def test_progress_off_its_line_is_not_reached(self):
        # 95 % along a 1.0 m forward move but 0.30 m off its line (a sideways-guard stop): over 0.15 + 10 % of it
        self.up(scene_d=4.0, mv_end=("PROGRESS", 0.95, (30, 0, 0)))
        code, out, _, _ = self.cli("move", "--forward", "1.0")
        self.assertEqual((code, out["ok"], out["end"], out["off_m"]), (1, False, "PROGRESS", 0.3), out)
        self.assertIn("off its line by 0.30 m", out["error"])
        self.assertNotIn("reached", out)

    def test_progress_short_of_it_is_not(self):
        self.up(scene_d=4.0, mv_end=("PROGRESS", 0.5))
        code, out, _, _ = self.cli("move", "--forward", "1.0")
        self.assertEqual((code, out["ok"], out["end"]), (1, False, "PROGRESS"))
        self.assertNotIn("reached", out)

    def test_front(self):
        self.up(scene_d=4.0, mv_end=("FRONT", 0.7))
        code, out, _, _ = self.cli("move", "--forward", "1.0")
        self.assertEqual((code, out["ok"], out["end"]), (1, False, "FRONT"))
        self.assertIn("stopped: the scene ahead got close; holding, take a photo", out["error"])

    def test_wait_move(self):
        d = self.up(scene_d=4.0, mv_speed=0.5)
        code, out, _, _ = self.cli("move", "--forward", "1.0", "--no-wait")
        self.assertEqual((code, out["running"]), (0, True))
        code, out, _, err = self.cli("wait-move", "--photo")
        self.assertEqual((code, out["ok"], out["end"], out["kind"]), (0, True, "DONE", "MOVE"), (out, err))
        self.assertEqual((out["travelled_m"]["fwd"], out["target_m"]["fwd"], out["D_m"]), (1.0, 1.0, 4.0))
        self.assertEqual(out["margins"], {"to_target_m": 0.0, "scene_ahead_m": 4.0})
        self.assertTrue(out["photo"]["ok"] and len(out["photo"]["photos"]) == 1, out["photo"])
        self.assertTrue(os.path.exists(out["photo"]["photos"][0]["png"]), out["photo"])
        t0 = time.monotonic()
        code, out, _, _ = self.cli("wait-move")      # nothing running: at once, the latest end
        self.assertEqual((code, out["end"]), (0, "DONE"))
        self.assertLess(time.monotonic() - t0, 3.0)
        self.assertEqual(d.state, HOVER)

    # ---- the move planned from take-off

    def plan_sortie(self, d, *extra):
        self.start_daemon(d)
        return self.cli("sortie", "--mode", "dry", "--height", "100", "--photos", "1", "--wait-s", "60", *extra)

    def test_plan_via_drone_running(self):
        if auto.MOVE_FWD_MAX_M < 4.0:
            self.skipTest("the long-move build's TAKEOFF (phase 2)")
        d = self.drone(**dict(IHFAST, scene_d=4.0))
        code, out, _, err = self.plan_sortie(d, "--plan-right", "0.5", "--plan-forward", "1.5", "--plan-via", "drone",
                                             "--est", "3.8")
        self.assertEqual((code, out["ok"], out["state"], out["waiting"]), (0, True, "HOVER", True), (out, err))
        p = out["plan"]
        self.assertEqual((p["via"], p["state"], p["mvSeq"], p["target_m"]), ("drone", "running", 1,
                                                                          {"right": 0.5, "fwd": 1.5}), p)
        (pl,) = [x.payload for _, x in d.got(auto.T_TAKEOFF)]
        self.assertEqual(struct.unpack("<IHBBHHhhH", pl)[6:], (50, 150, 0))
        self.assertEqual((d.plan_moves, d.got(auto.T_MOVE)), (1, []))     # the drone's own MOVE, none from the Mac
        self.assertEqual(len(out["photos"]), 1)
        ev = self.events("plan")
        self.assertEqual((ev[0]["via"], ev[0]["gap_m"], ev[0]["est_m"]), ("drone", None, 3.8))
        code, out, _, _ = self.cli("wait-move")
        self.assertEqual((code, out["end"], out["travelled_m"]["fwd"]), (0, "DONE", 1.5))

    def test_plan_via_drone_refused(self):
        if auto.MOVE_FWD_MAX_M < 4.0:
            self.skipTest("the long-move build's TAKEOFF (phase 2)")
        d = self.drone(**dict(IHFAST, scene_d=2.0))
        code, out, _, err = self.plan_sortie(d, "--plan-forward", "1.5", "--plan-via", "drone")
        self.assertEqual((code, out["ok"], out["waiting"]), (0, True, True), (out, err))
        p = out["plan"]
        self.assertEqual((p["state"], p["why"], p["D_m"], p["fwd_max_m"]), ("refused", "TOO_CLOSE", 2.0, 1.29), p)
        self.assertNotIn("fwd_max_drone_m", p)             # the plan flies the Mac's floor (AUTO_PLAN_MIN_FRONT_CM)
        self.assertIn("the take-off's planned move 5 cm more", p["hint"])

    def test_plan_via_drone_flies_the_macs_floor(self):
        # 1.30 m at D 2.0: inside the drone's own 0.30 m (1.35 m) but not the plan's 0.35 m (1.29 m): refused
        if auto.MOVE_FWD_MAX_M < 4.0:
            self.skipTest("the long-move build's TAKEOFF (phase 2)")
        d = self.drone(**dict(IHFAST, scene_d=2.0))
        code, out, _, err = self.plan_sortie(d, "--plan-forward", "1.30", "--plan-via", "drone")
        self.assertEqual((code, out["plan"]["state"], out["plan"]["why"]), (0, "refused", "TOO_CLOSE"), (out, err))

    def test_plan_via_drone_at_the_macs_floor(self):
        if auto.MOVE_FWD_MAX_M < 4.0:
            self.skipTest("the long-move build's TAKEOFF (phase 2)")
        d = self.drone(**dict(IHFAST, scene_d=2.0))
        code, out, _, err = self.plan_sortie(d, "--plan-forward", "1.29", "--plan-via", "drone")
        self.assertEqual((code, out["plan"]["state"]), (0, "running"), (out, err))
        self.assertEqual(d.state, HOVER)
        self.assertTrue(d.waiting)

    def test_plan_old_drone(self):
        d = self.drone(**dict(IHFAST, plan_takeoff=False))
        code, out, _, err = self.plan_sortie(d, "--plan-forward", "1.0", "--plan-via", "drone")
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("this build has no planned take-off: --plan-via mac", out["error"])
        self.assertEqual(out["plan"]["state"], "unsupported")
        self.assertTrue(wait_for(lambda: d.state == IDLE, 2), d.timeline)
        self.assertNotIn("TAKEOFF", [s for _, s, _ in d.timeline])   # no take-off happened
        self.assertTrue(d.got(auto.T_DISARM))

    def test_plan_via_mac(self):
        d = self.drone(**dict(IHFAST, scene_d=4.0, long_moves=False, plan_takeoff=False))   # works on b6a54da
        code, out, _, err = self.plan_sortie(d, "--plan-right", "0.4", "--plan-forward", "1.2", "--plan-via", "mac")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        p = out["plan"]
        self.assertEqual((p["via"], p["state"], p["mvSeq"], p["D_m"]), ("mac", "running", 1, 4.0), p)
        (pl,) = [x.payload for _, x in d.got(auto.T_TAKEOFF)]
        self.assertEqual(len(pl), 12)                                   # the TAKEOFF of before
        ((t_move, mvp),) = d.got(auto.T_MOVE)
        self.assertEqual(struct.unpack("<hhhH", mvp.payload)[:3], (40, 120, 0))
        first_wait = next(t for t, w, _ in d.status_pushes if w)
        between = [t for t, w, _ in d.status_pushes if w and first_wait <= t < t_move]
        self.assertLessEqual(len(between), 2, (first_wait, t_move, between))   # the MOVE within one status of the wait

    def plan_cancelled(self, via):
        d = self.drone(**dict(IHFAST, scene_d=4.0, hr=True, event_at_hover=(auto.EV_CONTACT, 0x04)))
        code, out, _, err = self.plan_sortie(d, "--plan-forward", "1.2", "--plan-via", via)
        self.assertEqual((code, out["ok"], out["state"], out["waiting"]), (0, True, "HOVER", True), (out, err))
        p = out["plan"]
        self.assertEqual((p["state"], p["why"], p["events"]), ("cancelled", "PLAN_CANCELLED", ["CONTACT"]), p)
        self.assertIn("take a photo", p["hint"])
        self.assertEqual((d.got(auto.T_MOVE), d.plan_moves, d.mv_seq), ([], 0, 0))
        self.assertTrue(self.events("plan_cancelled"))
        code, out, _, _ = self.cli("move", "--forward", "0.5")     # Claude's own move after a photo
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.cli("land")

    def test_plan_via_mac_cancelled_by_an_event(self):
        """An event before the hover wait (here a CONTACT at the HOVER entry, a 136-byte drone): the planned move is
        not flown (2026-10-15 verification): the Mac does not send its MOVE; plan.state 'cancelled' with the events,
        the drone hovers in the wait for Claude to judge a photo."""
        self.plan_cancelled("mac")

    def test_plan_via_drone_cancelled_by_an_event(self):
        """The same with the plan in the TAKEOFF: the drone cancels it (mvWhy PLAN_CANCELLED)."""
        if auto.MOVE_FWD_MAX_M < 4.0:
            self.skipTest("the long-move build's TAKEOFF (phase 2)")
        self.plan_cancelled("drone")

    def test_plan_via_mac_person(self):
        # a person approach from take-off goes via the Mac, which checks the gap with the live D (2026-10-15 review)
        d = self.drone(**dict(IHFAST, scene_d=4.0, long_moves=False, plan_takeoff=False))
        code, out, _, err = self.plan_sortie(d, "--plan-forward", "1.0", "--plan-via", "mac", "--person-gap", "1.4",
                                             "--est", "2.6")
        self.assertEqual((code, out["ok"], out["waiting"]), (0, True, True), (out, err))
        p = out["plan"]
        self.assertEqual((p["state"], p["why"], p["fwd_max_m"]), ("refused", "MAC", 0.91), p)
        self.assertIn("end 0.88 m from the person", p["error"])
        self.assertEqual(d.got(auto.T_MOVE), [])

    def test_plan_via_mac_person_ok(self):
        d = self.drone(**dict(IHFAST, scene_d=4.0, long_moves=False, plan_takeoff=False))
        code, out, _, err = self.plan_sortie(d, "--plan-forward", "1.0", "--plan-via", "mac", "--person-gap", "1.6",
                                             "--est", "2.6")
        self.assertEqual((code, out["plan"]["state"]), (0, "running"), (out, err))
        self.assertEqual(len(d.got(auto.T_MOVE)), 1)

    def test_plan_via_mac_refused_by_the_floor(self):
        d = self.drone(**dict(IHFAST, scene_d=2.0))
        code, out, _, err = self.plan_sortie(d, "--plan-forward", "1.5", "--plan-via", "mac")
        self.assertEqual((code, out["ok"], out["waiting"]), (0, True, True), (out, err))
        p = out["plan"]
        self.assertEqual((p["state"], p["fwd_max_m"]), ("refused", 1.29), p)
        self.assertIn("at most 1.29 m now", p["error"])
        self.assertEqual(d.got(auto.T_MOVE), [])

    def test_plan_ranges_before_take_off(self):
        d = self.drone(**IHFAST)
        self.start_daemon(d)
        for argv in (("--plan-forward", str(auto.MOVE_FWD_MAX_M + 0.1)), ("--plan-right", "1.6"),
                     ("--plan-back", "1.6")):
            code, out, _, _ = self.cli("sortie", "--mode", "dry", "--wait-s", "60", *argv)
            self.assertEqual(code, 2, argv)                                 # usage errors
        cases = [(("--plan-forward", "1.0", "--then", "land"), "needs --then wait"),
                 (("--plan-forward", "1.0", "--person-gap", "0.8"), "never ends closer than 1.0 m"),
                 (("--plan-forward", "1.0", "--person-gap", "1.5"), "--person-gap needs --est"),
                 (("--plan-forward", "1.0", "--person-gap", "1.5", "--est", "3.0", "--plan-via", "drone"),
                  "a planned move toward a person goes --plan-via mac")]
        if auto.MOVE_FWD_MAX_M >= 3.8:
            cases.append((("--plan-right", "1.5", "--plan-forward", "3.8"), "the move's length 4.085 m"))
        for argv, msg in cases:
            code, out, _, _ = self.cli("sortie", "--mode", "dry", "--wait-s", "60", *argv)
            self.assertEqual((code, out["ok"]), (1, False), argv)
            self.assertIn(msg, out["error"])
        os.makedirs(auto.DEFAULT_DIR, exist_ok=True)
        with open(auto.vmin_cache_path(), "w") as f:   # the drone's autowait 0: the wait starts anyway (2026-10-15)
            json.dump({"utc": "t", "values": {"autowait": {"value": 0, "src": "nvs"}}}, f)
        code, out, _, _ = self.cli("sortie", "--mode", "dry", "--plan-forward", "1.0")
        self.assertEqual((code, out["ok"]), (0, True), out)   # refused until 2026-10-15 ("needs a hover wait")
        self.assertEqual(len(d.got(auto.T_ARM)), 1)
        self.cli("land")

    def test_plan_client_gone_holds(self):
        d = self.drone(**dict(FAST, hover_at=1.5, wait_ms=8000, scene_d=4.0))
        dm, _ = self.start_daemon(d)
        s = raw_client(self.sock, {"op": "sortie", "mode": "dry", "photos": 0, "plan_fwd": 1.0, "plan_via": "mac"})
        self.assertTrue(wait_for(lambda: dm.sorties and dm.sorties[0].phase == "flying", 3))
        s.close()
        self.assertTrue(wait_for(lambda: self.events("sortie_holding"), 3))
        time.sleep(0.5)
        self.assertNotIn("LAND_CMD", [r for _, _, r in d.timeline], d.timeline)
        self.assertEqual(self.events("sortie_abandoned")[0]["phase"], "flying")
        self.assertEqual(d.got(auto.T_MOVE), [])


class DaemonHoldReportTest(DaemonCase):
    """A 136-byte drone ("Claude decides", 2026-10-15, docs/autonomy-a2b.md 22): it holds past its wait (no
    HOVER_TIMEOUT, no air clock) and what it does on its own reaches Claude: a 'drone_event' line per reported
    event, and 'hold' in the replies (the latest three events, the bits, the SINK countdown, the contact's
    obstacle bearing)."""

    def test_events_and_hold(self):
        d = self.drone(**dict(IHFAST, hr=True))
        self.start_daemon(d)
        out = self.sortie_to_wait("--height", "100", "--wait-s", "1")
        self.assertEqual(out["hold"]["events"], [])
        time.sleep(1.5)                                   # past the 1 s wait: the drone holds
        self.assertEqual(d.state, HOVER)
        d.report(auto.EV_CONTACT, 0x84, contacts=1, peakAccCg=231, peakRateDps=310, contactAxCg=-205,
                 contactAyCg=98, contactYawCdeg=0)
        self.assertTrue(wait_for(lambda: self.events("drone_event"), 2))
        ev = self.events("drone_event")[0]
        self.assertEqual((ev["kind"], ev["detailName"]["phase"], ev["detailName"]["motionStopped"]),
                         ("CONTACT", "HOLD", True))
        self.assertIn("back away", ev["hint"])
        d.report(auto.EV_CAM_LOST, 0x02, holdBits=0x81, camBits=0x02, camOffMs=500, sinkInMs=4500)
        self.assertTrue(wait_for(lambda: len(self.events("drone_event")) == 2, 2))
        self.assertEqual(self.events("drone_event")[1]["kind"], "CAM_LOST")
        code, out, _, _ = self.cli("state")
        h = out["hold"]
        self.assertEqual([e["name"] for e in h["events"]], ["CAM_LOST", "CONTACT"])
        self.assertEqual((h["bits"], h["camBits"], h["sinkIn_s"], h["contacts"]), (["camLost", "openLoop"], ["noG"],
                                                                                    4.5, 1))
        self.assertEqual(h["contact"]["obstacle_bearing_deg"], 25.6)   # ahead and to the right
        # the air time past 65.5 s from airDs (no air clock: never a mismatch)
        d.air_ms_offset = 100000
        self.assertTrue(wait_for(lambda: self.dm.auto["air_s"] > 100, 3))
        self.assertFalse(self.events("air_clock_mismatch"))
        self.assertEqual((self.dm.auto["airLeftDs"], self.dm.auto["airMs"]), (0, 65535))
        code, out, _, _ = self.cli("move", "--right", "0.2")   # no air clock: the Mac does not refuse for it
        self.assertEqual((code, out["ok"]), (0, True), out)
        # 'wait --until event': returns at the drone's next report
        threading.Timer(0.5, lambda: d.report(auto.EV_VZ, 18)).start()   # VZ: the controller's K_RANGE event
        code, out, _, _ = self.cli("wait", "--until", "event", "--timeout", "5")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.assertEqual((out["hold"]["events"][0]["name"], out["hold"]["events"][0]["detailName"]), ("VZ", "K_RANGE"))
        self.cli("land")


class DaemonInteractiveTest(DaemonCase):
    """The user's flow (2026-10-12, late): the drone is left on a flat spot, takes off to ~1 m, takes one
    photo, then waits for one command at a time from the Mac: move, yaw, stop, land (the approach planner was removed
    on 2026-10-15: Claude judges the distance and sends a YAW, then a plain MOVE)."""

    def up(self, d=None, *extra, **kw):
        d = d or self.drone(**dict(IHFAST, **kw))
        self.start_daemon(d)
        self.sortie_to_wait("--height", "100", "--photos", "1", "--wait-s", "60", *extra)
        return d

    def sent(self, d, ptype):
        return [p.payload for _, p in d.got(ptype)]

    def test_sortie_carries_the_wait_and_height(self):
        d = self.up()
        (p,) = self.sent(d, auto.T_TAKEOFF)
        self.assertEqual(struct.unpack("<IHBBHH", p)[1:], (100, 1, 0, 300, 60000))
        self.assertEqual(d.wait_ms, 60000)
        code, out, _, _ = self.cli("state")
        self.assertEqual(code, 0)
        self.assertEqual((out["hTarget_m"], out["D_m"], out["mv"]["state"], out["auto"]["interactive"]),
                         (1.0, 2.0, "IDLE", True))
        self.assertGreater(out["airLeft_s"], 70)

    def test_wait_s_range(self):
        d = self.drone(**IHFAST)
        self.start_daemon(d)
        for bad in ("0.5", "121", "120.5"):
            code, out, _, _ = self.cli("sortie", "--mode", "dry", "--wait-s", bad)
            self.assertEqual((code, out["ok"]), (2, False), bad)
        self.assertFalse(d.got(auto.T_ARM))

    def test_long_wait(self):
        """Waits up to 120 s (2026-10-14, docs/autonomy-a2b.md 18.2): TAKEOFF byte 7 carries bits 16-23; the
        height rule's drift term is capped at 80 s (72 s and more need 105 cm); the status saturates at 65535,
        shown as waitLeftSaturated."""
        d = self.drone(**IHFAST)
        self.start_daemon(d)
        # the height rule is advice since 2026-10-15 (it refused the take-off until then)
        out = self.sortie_to_wait("--height", "100", "--wait-s", "120")
        self.assertIn("105 cm or more is safer", out["note"])
        self.cli("land")
        self.assertTrue(wait_for(lambda: d.state == IDLE, 5))
        out = self.sortie_to_wait("--height", "105", "--wait-s", "120")
        self.assertNotIn("note", out)
        p = self.sent(d, auto.T_TAKEOFF)[-1]
        self.assertEqual(p[7], 0x01)
        self.assertEqual(struct.unpack("<H", p[10:12])[0], 0xD4C0)
        self.assertEqual(d.wait_ms, 120000)
        self.assertTrue(out["waitLeftSaturated"], out)
        self.assertLessEqual(out["waitLeftMs"], 65535)
        code, out, _, _ = self.cli("status")
        self.assertEqual(code, 0)
        self.assertTrue(out["auto"]["waitLeftSaturated"], out)
        self.assertEqual(out["auto"]["waitLeftMs"], 65535)
        code, out, _, _ = self.cli("state")
        self.assertTrue(out["waitLeftSaturated"], out)
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))

    def test_wait_height_rule(self):
        """The take-off height a --wait-s needs: 0.25 m + 0.01 m/s x min(wait + 8 s, 80 s)."""
        for w, cm in ((30, 63), (60, 93), (64, 97), (72, 105), (100, 105), (120, 105)):
            with self.subTest(wait=w):
                self.assertEqual(math.ceil(auto.wait_height_floor(w) * 100 - 1e-6), cm)
        d = self.drone(**IHFAST)
        self.start_daemon(d)
        self.sortie_to_wait("--height", "93", "--wait-s", "60")
        self.assertEqual(struct.unpack("<IHBBHH", self.sent(d, auto.T_TAKEOFF)[0])[1:], (93, 1, 0, 300, 60000))
        self.assertFalse(self.cli("status")[1]["auto"]["waitLeftSaturated"])

    def test_old_drone_refuses_a_long_wait(self):
        """A drone before 2026-10-14 refuses a nonzero TAKEOFF byte 7 (RANGE, detail 0): never a misread wait. A
        wait <= 65.5 s still works there (byte 7 is 0)."""
        d = self.drone(**dict(IHFAST, long_wait=False))
        self.start_daemon(d)
        code, out, _, _ = self.cli("sortie", "--mode", "dry", "--height", "105", "--wait-s", "100")
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertEqual(d.wait_ms, d.wait_ms0)
        self.assertNotEqual(d.state, HOVER)

    def test_move_right_back_up_and_yaw(self):
        d = self.up()
        code, out, _, err = self.cli("move", "--right", "0.5")
        self.assertEqual((code, out["ok"], out["end"], out["result"]), (0, True, "DONE", "OK"), (out, err))
        self.assertEqual(out["target_m"], {"right": 0.5, "fwd": 0.0, "up": 0.0})
        self.assertEqual((out["mvSeq"], out["mv"]["prog_m"]["right"], out["D_m"]), (1, 0.5, 2.0))
        code, out, _, _ = self.cli("move", "--left", "0.25", "--back", "0.3", "--up", "0.2")
        self.assertEqual((code, out["ok"], out["mvSeq"], out["hTarget_m"]), (0, True, 2, 1.2))
        code, out, _, _ = self.cli("yaw", "--deg", "90")
        self.assertEqual((code, out["ok"], out["end"], out["yawOff_deg"]), (0, True, "DONE", 90.0))
        code, out, _, _ = self.cli("yaw", "--deg", "-45")
        self.assertEqual((code, out["yawOff_deg"]), (0, 45.0))
        self.assertEqual([struct.unpack("<hhhH", p)[:3] for p in self.sent(d, auto.T_MOVE)],
                         [(50, 0, 0), (-25, -30, 20)])
        self.assertEqual([struct.unpack("<hH", p)[0] for p in self.sent(d, auto.T_YAW)], [90, -45])
        self.assertEqual(d.state, HOVER)
        self.assertTrue(d.waiting)
        self.assertEqual([e["type"] for e in self.events("motion")], ["MOVE", "MOVE", "YAW", "YAW"])
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))

    def test_mac_refusals_send_nothing(self):
        d = self.drone(**IHFAST)
        self.start_daemon(d)
        self.assertTrue(wait_for(lambda: self.dm.auto is not None, 2))
        code, out, _, _ = self.cli("move", "--right", "0.3")     # IDLE: not in the hover wait
        self.assertEqual((code, out["result"]), (1, "REFUSED"))
        self.assertIn("not in the hover wait", out["error"])
        self.sortie_to_wait("--height", "120", "--wait-s", "60")
        with d.lock:
            d.h_off_cm = 60                     # moved up 0.60 m since (the take-off stays <= 1.20 m)
        self.assertTrue(wait_for(lambda: self.dm.auto["hTarget_m"] == 1.8, 2))
        cases = [
            (("move", "--forward", "1.5"), "the Mac keeps 0.35 m"),     # D 2.0: 0.5 m left
            (("move", "--forward", "1.3"), "at most 1.29 m"),           # 2.0 (0.7 / 2.0)^(1/0.6) = 0.348 m at k 0.6
            (("move", "--right", "1.1"), "Split it"),                   # > 0.5 x D 2.0
            (("move", "--up", "0.3"), "must stay 0.25..2.00 m"),       # 1.8 + 0.3
        ]
        for argv, msg in cases:
            with self.subTest(argv=argv):
                code, out, _, _ = self.cli(*argv)
                self.assertEqual((code, out["ok"]), (1, False), out)
                self.assertIn(msg, out["error"])
        for argv in (("move", "--right", "1.6"), ("move", "--up", "0.6"), ("yaw", "--deg", "181")):
            code, out, _, _ = self.cli(*argv)
            self.assertEqual((code, out["ok"]), (2, False), argv)     # usage errors
        self.assertEqual((d.got(auto.T_MOVE), d.got(auto.T_YAW)), ([], []))
        d.scene_d = None
        self.assertTrue(wait_for(lambda: self.dm.auto["sceneD_m"] is None, 2))
        code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual(code, 1)
        self.assertIn("no scene distance D", out["error"])
        code, out, _, _ = self.cli("move", "--down", "0.2")      # vertical: no D needed
        self.assertEqual((code, out["ok"], out["hTarget_m"]), (0, True, 1.6), out)

    def test_drone_refusal_reason(self):
        d = self.up(refuse_mv=["NO_VIEW"])
        code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual((code, out["ok"], out["result"], out["why"]), (1, False, "CAM_UNAVAIL", "NO_VIEW"))
        self.assertIn("camera hold is not applied", out["error"])
        code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual((code, out["ok"]), (0, True))

    def test_drone_refusal_hover_thrust(self):
        """The drone refuses a MOVE / YAW (LOW_BATT, BATT) once the hover needs over 53,400 of the 58,000 ceiling,
        with the battLow bits clear (2026-10-14): the Mac sends it, and its hint says the drone lands by itself."""
        d = self.up(refuse_mv=["BATT"])
        code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual((code, out["ok"], out["result"], out["why"]), (1, False, "LOW_BATT", "BATT"))
        self.assertIn("53,400 of the 58,000 ceiling", out["error"])
        self.assertIn("lands BATT_LOW by itself", out["error"])
        self.assertEqual(len(self.sent(d, auto.T_MOVE)), 1)

    def test_stop_ends_a_running_move(self):
        d = self.up(mv_speed=0.5)                 # 1.0 m at 0.2 m/s / 0.5: ~10 s
        code, out, _, _ = self.cli("move", "--forward", "1.0", "--no-wait")
        self.assertEqual((code, out["ok"], out["running"]), (0, True, True))
        self.assertTrue(wait_for(lambda: d.mv_state == auto.MV_RUN, 1))
        code, out, _, _ = self.cli("move", "--right", "0.2")          # one at a time
        self.assertEqual((code, out["result"]), (1, "REFUSED"))
        self.assertIn("is running", out["error"])
        time.sleep(0.5)
        code, out, _, _ = self.cli("stop")
        self.assertEqual((code, out["ok"], out["found"]), (0, True, "RUN"))
        self.assertTrue(wait_for(lambda: self.dm.auto["mvState"] == auto.MV_STOPPED, 1))
        self.assertEqual(self.dm.auto["mvEndName"], "STOP_CMD")
        self.assertLess(self.dm.auto["progFwdCm"], 100)
        code, out, _, _ = self.cli("stop")                            # nothing running: still OK
        self.assertEqual((code, out["ok"], out["found"]), (0, True, "STOPPED"))

    def test_waiting_move_reports_a_stop(self):
        d = self.up(mv_speed=0.5)
        res = {}
        th = threading.Thread(target=lambda: res.update(r=auto.daemon_request(
            self.sock, {"op": "move", "fwd": 1.0, "wait": True}, 30.0)))
        th.start()
        self.assertTrue(wait_for(lambda: d.mv_state == auto.MV_RUN, 2))
        time.sleep(0.6)                           # ~6 cm of the 1.0 m
        code, out, _, _ = self.cli("stop")
        self.assertEqual(code, 0)
        th.join(10)
        out = res["r"]
        self.assertEqual((out["ok"], out["end"]), (False, "STOP_CMD"))
        self.assertIn("holds where it is", out["error"])
        # the drone keeps the move's travel in its end status: that is what the Mac reports
        self.assertGreater(out["travelled_m"]["fwd"], 0.0)
        self.assertEqual(out["travelled_m"], self.dm.auto["prog_m"])

    def test_short_move_is_ok_and_reports_its_travel(self):
        d = self.up(mv_short=0.85)
        code, out, _, err = self.cli("move", "--forward", "1.0")
        self.assertEqual((code, out["ok"], out["end"]), (0, True, "SHORT"), (out, err))
        self.assertEqual(out["travelled_m"], {"right": 0.0, "fwd": 0.85, "up": 0.0})
        self.assertIn("ended SHORT", out["note"])
        self.assertIn("holds where it ended", out["note"])
        self.assertNotIn("error", out)
        self.assertEqual(d.state, HOVER)

    def test_long_move_is_ok_and_reports_its_overshoot(self):
        """mvfix LONG (2026-10-17): a settle that timed out past the target ends LONG (DONE): ok, the travel, a note
        and the overshoot as a warning."""
        d = self.up(mv_end=("LONG", 1.3))
        code, out, _, err = self.cli("move", "--forward", "1.0")
        self.assertEqual((code, out["ok"], out["end"]), (0, True, "LONG"), (out, err))
        self.assertEqual(out["travelled_m"], {"right": 0.0, "fwd": 1.3, "up": 0.0})
        self.assertIn("ended LONG", out["note"])
        self.assertIn("past the target", out["note"])
        self.assertIn("past the target by 0.30 m", out["warning"])
        self.assertNotIn("error", out)
        self.assertEqual(d.state, HOVER)

    def test_done_past_its_target_warns(self):
        """A normal end whose measured travel is past the target by more than 10 % (the fake's SHORT path at 125 %; a
        DONE of a drone without LONG the same): a warning in the reply, still ok."""
        self.up(mv_short=1.25)
        code, out, _, err = self.cli("move", "--forward", "1.0")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        self.assertIn("past the target by 0.25 m", out["warning"])

    def test_drift_since_the_last_end(self):
        """mvfix DRIFT (2026-10-17): the drone's D at the next MOVE's acceptance follows the drift since the last
        move's end; the reply names the difference (+ toward the scene)."""
        d = self.up(scene_d=3.0)
        d.d_track = True
        code, out, _, err = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        self.assertNotIn("drift_since_last_end_m", out)   # (no D of a previous move's end yet)
        d.scene_d_accept = 2.8
        code, out, _, err = self.cli("move", "--forward", "0.3")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        self.assertEqual(out["drift_since_last_end_m"], {"fwd": 0.2, "D_last_end_m": 3.0, "D_at_accept_m": 2.8})

    def test_no_drift_field_without_mvfix_drift(self):
        """The review of 2026-10-17: with the cached mvfix's DRIFT off the drone measures no drift (its D at the
        acceptance is the last end's): no drift_since_last_end_m rather than a 0.000 that reads as 'no drift'."""
        os.makedirs(auto.DEFAULT_DIR, exist_ok=True)
        with open(auto.vmin_cache_path(), "w") as f:
            json.dump({"utc": "2026-10-17T09:00:00Z", "values": {"mvfix": {"value": "58", "src": "nvs"}}}, f)
        self.assertFalse(auto.drift_measured())
        d = self.up(scene_d=3.0)
        d.d_track = True
        code, out, _, err = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        code, out, _, err = self.cli("move", "--forward", "0.3")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        self.assertNotIn("drift_since_last_end_m", out)
        os.remove(auto.vmin_cache_path())
        self.assertTrue(auto.drift_measured())   # (no cache: the default, DRIFT on)

    def test_long_toward_the_scene_walks_back_and_sideways_holds(self):
        """The review of 2026-10-17: a LONG move with a part toward the scene keeps its target (the hold walks it back);
        a sideways LONG holds where it ended."""
        self.up(mv_end=("LONG", 1.3), scene_d=3.0)
        code, out, _, err = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["end"]), (0, "LONG"), (out, err))
        self.assertIn("walks it back to the target", out["note"])
        code, out, _, err = self.cli("move", "--right", "0.4")
        self.assertEqual((code, out["end"]), (0, "LONG"), (out, err))
        self.assertIn("holds where it ended", out["note"])

    def test_sideways_at_the_mean_distance(self):
        """A diagonal toward the scene: the drone converts its sideways travel at the start distance; the reply adds it
        at the mean of the start and end distances (k 1)."""
        self.up(scene_d=2.0)
        code, out, _, err = self.cli("move", "--right", "0.4", "--forward", "1.0")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        self.assertAlmostEqual(out["sideways_m_at_mean_D"], 0.4 * 0.5 * (1.0 + (1.0 - 1.0 / 2.0)), places=3)

    def test_stop_travel_from_an_older_drone(self):
        """A drone before the 2026-10-13 evening change restarts the progress at a STOP: the last running one."""
        d = self.up(mv_speed=0.5, old_fw_zero=True)
        res = {}
        th = threading.Thread(target=lambda: res.update(r=auto.daemon_request(
            self.sock, {"op": "move", "fwd": 1.0, "wait": True}, 30.0)))
        th.start()
        self.assertTrue(wait_for(lambda: d.mv_state == auto.MV_RUN, 2))
        self.assertTrue(wait_for(lambda: self.dm.auto["prog_m"]["fwd"] >= 0.2, 5))
        code, out, _, _ = self.cli("stop")
        self.assertEqual(code, 0)
        th.join(10)
        out = res["r"]
        self.assertEqual(out["end"], "STOP_CMD")
        self.assertEqual(self.dm.auto["prog_m"]["fwd"], 0.0)
        self.assertGreaterEqual(out["travelled_m"]["fwd"], 0.2)

    def test_approach_is_gone(self):
        """The Mac's approach planner was removed (user decision P, 2026-10-15): no 'approach' command, the daemon
        answers 'unknown op' and sends nothing; Claude judges the distance from the photo and sends a YAW, then a plain
        MOVE (with --person-gap / --est toward a person), which the drone and the Mac check as any."""
        d = self.up(scene_d=3.0)
        code, out, _, err = self.cli("approach", "--bearing", "12", "--to", "1.0", "--est", "2.5")
        self.assertEqual(code, 2, (out, err))
        r = auto.daemon_request(self.sock, {"op": "approach", "bearing": 12, "to": 1.0, "est": 2.5}, 5.0)
        self.assertFalse(r["ok"])
        self.assertIn("unknown op", r["error"])
        self.assertNotIn("approach", r["ops"])
        self.assertEqual((d.got(auto.T_MOVE), d.got(auto.T_YAW)), ([], []))
        self.assertFalse(any(hasattr(auto, n) for n in ("APPROACH_D_ERR", "APPROACH_STEP_MAX_M", "APPROACH_D_WAIT_S",
                                                        "PHOTO_YAW_TOL_DEG", "cmd_approach")))
        self.assertFalse(any(hasattr(auto.Daemon, n) for n in ("op_approach", "photo_view_refusal", "note_photo_view")))
        # the same intent, the plain way: a YAW to the bearing, then a MOVE toward a person
        code, out, _, _ = self.cli("yaw", "--deg", "12")
        self.assertEqual((code, out["ok"]), (0, True), out)
        code, out, _, _ = self.cli("move", "--forward", "0.5", "--person-gap", "2.5", "--est", "3.0")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.assertEqual([struct.unpack("<hH", p)[0] for p in self.sent(d, auto.T_YAW)], [12])
        self.assertEqual([struct.unpack("<hhhH", p)[:3] for p in self.sent(d, auto.T_MOVE)], [(0, 50, 0)])

    def test_batt_low_held_not_one_reading(self):
        """AUTO_STATUS battLow: bit 0 is the drone's own refusal (its BATT_LOW condition, a reading below 3.00 V or the
        hover thrust over 57,000, held 0.25 s), bit 1 the condition now. The
        Mac refuses on bit 0 only (591f176 refused on any single reading: 3 of 8 moves in a rehearsal)."""
        d = self.up()
        d.batt_low_bits = auto.BATTLOW_NOW
        self.assertTrue(wait_for(lambda: self.dm.auto["battLowNow"], 2))
        code, out, _, _ = self.cli("move", "--right", "0.2")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.assertTrue(out["battLowNow"])
        self.assertFalse(out["battLow"])
        d.batt_low_bits = auto.BATTLOW_HELD | auto.BATTLOW_NOW
        self.assertTrue(wait_for(lambda: self.dm.auto["battLow"], 2))
        code, out, _, _ = self.cli("move", "--right", "0.2")
        self.assertEqual((code, out["result"]), (1, "REFUSED"))
        self.assertIn("0.25 s", out["error"])
        self.assertIn("lands BATT_LOW by itself", out["error"])
        self.assertIn("STOP and LAND still work", out["error"])
        self.assertNotIn("): LAND", out["error"])            # no LAND advice: the drone lands by itself
        self.assertEqual(len(self.sent(d, auto.T_MOVE)), 1)

    def test_low_late_and_long_wait(self):
        """The height estimate drifts (up to 0.01 m/s, reading high): a long wait at a low take-off gets advice, and
        late in a sortie the Mac refuses a move DOWN below 0.25 m + the drift (at most 0.30 m, the drone's
        AUTO_MV_H_DRIFT_CAP_M); one-sided since 2026-10-15: a YAW, a level or an upward move is never refused for it."""
        d = self.drone(**IHFAST)
        self.start_daemon(d)
        out = self.sortie_to_wait("--height", "60", "--wait-s", "60")
        self.assertIn("93 cm or more is safer", out["note"])      # advice (a refusal until 2026-10-15)
        self.cli("land")
        self.assertTrue(wait_for(lambda: d.state == IDLE, 5))
        self.sortie_to_wait("--height", "100", "--wait-s", "60")
        d.air_ms_offset = 76000            # 76 s of air: the floor 0.25 + min(0.30, 0.76) = 0.55 m
        self.assertTrue(wait_for(lambda: self.dm.auto["air_s"] > 76, 2))
        self.assertAlmostEqual(auto.height_floor(76.0), 0.55)
        code, out, _, _ = self.cli("move", "--down", "0.5")      # 0.50 m: under the floor, and down
        self.assertEqual((code, out["result"]), (1, "REFUSED"), out)
        self.assertIn("too low for", out["error"])
        for argv in (("move", "--down", "0.4"), ("move", "--right", "0.2"), ("yaw", "--deg", "30")):
            code, out, _, _ = self.cli(*argv)
            self.assertEqual((code, out["ok"]), (0, True), (argv, out))
            self.assertTrue(wait_for(lambda: not self.dm.auto["mvActive"], 5))
        # at 0.60 m, past 100 s of air (the drift capped: 0.55 m): level moves, YAW and up all go
        d.air_ms_offset = 100000 - 1000
        self.assertTrue(wait_for(lambda: self.dm.auto["air_s"] > 100, 3))
        for argv in (("move", "--right", "0.2"), ("yaw", "--deg", "30"), ("move", "--up", "0.05")):
            code, out, _, _ = self.cli(*argv)
            self.assertEqual((code, out["ok"]), (0, True), (argv, out))
            self.assertTrue(wait_for(lambda: not self.dm.auto["mvActive"], 5))
        self.assertFalse(self.events("ceiling_warning"))   # 0.65 m: not above the ceiling rule's height
        self.assertEqual(d.state, HOVER)
        self.cli("land")

    def test_no_ceiling_rule(self):
        """The ceiling rule (1.05 m, 80 s: a take-off note, ceiling_warning, an up-move refusal) was removed on
        2026-10-15 (docs/autonomy-a2b.md 23): Claude checks the height from the photos' level line and corrects it with
        set-height. A 1.10 m take-off with an 80 s wait gets no advice, no warning at 80 s of air, and every move
        goes."""
        d = self.drone(**IHFAST)
        self.start_daemon(d)
        out = self.sortie_to_wait("--height", "110", "--wait-s", "80")
        self.assertNotIn("note", out)
        self.assertEqual(struct.unpack("<IHBBHH", d.got(auto.T_TAKEOFF)[0][1].payload)[1], 110)
        d.air_ms_offset = 78000
        self.assertTrue(wait_for(lambda: self.dm.auto["air_s"] >= 80.0, 3))
        time.sleep(0.3)
        self.assertFalse(self.events("ceiling_warning"))
        self.assertFalse(d.got(auto.T_LAND))
        for argv in (("move", "--up", "0.05"), ("move", "--right", "0.2"), ("move", "--down", "0.1")):
            code, out, _, _ = self.cli(*argv)
            self.assertEqual((code, out["ok"]), (0, True), (argv, out))
            self.assertTrue(wait_for(lambda: not self.dm.auto["mvActive"], 5))
        self.assertFalse(hasattr(auto, "CEIL_H_M") or hasattr(auto, "CEIL_AIR_S"))
        self.assertFalse(hasattr(auto.Daemon, "ceiling_refusal"))
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))

    def test_height_check_due(self):
        """The ceiling rule's successor (a warning only, 2026-10-15 review): height_check_due once per lapse when the
        sortie's latest height check (a saved photo or an accepted set-height) is older than the runbook's cadence at
        the commanded height (15 s, 10 s above 1.20 m; shortened here); a new photo re-arms it; nothing is refused or
        landed for it."""
        with mock.patch.object(auto, "HEIGHT_CHECK_S", 1.0), mock.patch.object(auto, "HEIGHT_CHECK_HIGH_S", 0.5):
            d = self.drone(**IHFAST)
            self.start_daemon(d)
            self.sortie_to_wait("--height", "100", "--photos", "1", "--wait-s", "60")
            self.assertTrue(wait_for(lambda: self.events("height_check_due"), 4))
            self.assertTrue(wait_for(lambda: self.dm.photo_log, 4))       # the take-off photo
            self.assertTrue(wait_for(lambda: self.events("height_check_due")[-1]["last"] == "photo", 4))
            ev = self.events("height_check_due")
            self.assertEqual((ev[-1]["due_s"], ev[-1]["hTarget_m"]), (1.0, 1.0))
            self.assertGreaterEqual(ev[-1]["since_s"], 1.0)
            self.assertIn("take a photo", ev[-1]["note"])
            n = len(ev)
            time.sleep(0.6)
            self.assertEqual(len(self.events("height_check_due")), n)       # once per lapse
            code, out, _, _ = self.cli("state")
            self.assertTrue(out["height_check_due"], out)
            code, out, _, _ = self.cli("photo")                          # a new check re-arms it
            self.assertEqual((code, out["ok"]), (0, True), out)
            code, out, _, _ = self.cli("state")
            self.assertFalse(out["height_check_due"], out)
            self.assertTrue(wait_for(lambda: len(self.events("height_check_due")) == n + 1, 4))
            with d.lock:
                d.h_off_cm = 40                                          # 1.40 m commanded: the higher cadence
            self.assertTrue(wait_for(lambda: self.dm.auto["hTarget_m"] == 1.4, 2))
            code, out, _, _ = self.cli("photo")
            self.assertTrue(wait_for(lambda: len(self.events("height_check_due")) == n + 2, 4))
            self.assertEqual(self.events("height_check_due")[-1]["due_s"], 0.5)
            code, out, _, _ = self.cli("move", "--right", "0.2")         # information only: nothing refused
            self.assertEqual((code, out["ok"]), (0, True), out)
            self.assertFalse(d.got(auto.T_LAND))
            self.cli("land")

    def write_autowait(self, ms):
        os.makedirs(auto.DEFAULT_DIR, exist_ok=True)
        with open(auto.vmin_cache_path(), "w") as f:
            json.dump({"utc": "2026-10-14T09:00:00Z", "values": {"autowait": {"value": str(ms), "src": "nvs"}}}, f)

    def test_nvs_autowait_rules(self):
        """--wait-s 0 flies the drone's NVS autowait (up to 120 s since 2026-10-14): the long-wait advice uses it as
        the last 'auto get' read it (the verification of 73969ad: 'auto set autowait 120000' then 'sortie --height
        40' passed unchecked). Without a read the advice is not checked, and the reply says so."""
        d = self.drone(**IHFAST)
        self.start_daemon(d)
        self.write_autowait(100000)
        out = self.sortie_to_wait("--height", "40")           # advice since 2026-10-15 (refused until then)
        self.assertIn("the drone's autowait (100000 ms", out["note"])
        self.assertIn("105 cm or more is safer", out["note"])
        self.cli("land")
        self.assertTrue(wait_for(lambda: d.state == IDLE, 5))
        out = self.sortie_to_wait("--height", "110")
        self.assertNotIn("note", out)        # no ceiling advice since 2026-10-15 (set-height, docs/autonomy-a2b.md 23)
        self.cli("land")
        self.assertTrue(wait_for(lambda: d.state == IDLE, 5))
        code, out, _, _ = self.cli("sortie", "--mode", "dry", "--height", "40", "--wait-s", "10")
        self.assertEqual((code, out["ok"]), (0, True), out)   # an explicit --wait-s wins
        self.assertNotIn("note", out)
        self.cli("land")
        self.assertTrue(wait_for(lambda: d.state == IDLE, 5))
        self.write_autowait(10000)
        out = self.sortie_to_wait("--height", "40")
        self.assertNotIn("note", out)
        self.cli("land")
        self.assertTrue(wait_for(lambda: d.state == IDLE, 5))
        os.remove(auto.vmin_cache_path())
        out = self.sortie_to_wait("--height", "40")
        self.assertIn("autowait is unknown", out["note"])
        self.cli("land")

    def test_air_clock_of_another_version(self):
        """A drone whose air limit is not this Mac's AIR_HARD_MS (an 80 s drone of b176c26 with this 120 s Mac): the
        Mac reads airMs while it is exact, flags the difference (event air_clock_mismatch) and refuses MOVE / YAW,
        whose height rules need the air time past 65.5 s; STOP and LAND still work."""
        d = self.drone(**dict(IHFAST, air_hard_ms=80000))
        self.start_daemon(d)
        self.sortie_to_wait("--height", "100", "--wait-s", "60")
        self.assertTrue(wait_for(lambda: self.events("air_clock_mismatch"), 2))
        self.assertTrue(79900 < self.events("air_clock_mismatch")[0]["drone_ms"] <= 80000)
        a = self.dm.auto
        self.assertLess(abs(a["air_s"] - a["airMs"] / 1000.0), 1e-9)
        code, out, _, _ = self.cli("move", "--right", "0.2")
        self.assertEqual((code, out["result"]), (1, "REFUSED"))
        self.assertIn("different versions", out["error"])
        self.assertFalse(d.got(auto.T_MOVE))
        code, out, _, _ = self.cli("stop")
        self.assertEqual((code, out["ok"]), (0, True), out)
        code, out, _, _ = self.cli("land")
        self.assertEqual((code, out["result"]), (0, "OK"))

    def test_a2b_drone_refused(self):
        """A drone without the interactive hover (64-byte AUTO_STATUS): MOVE refused by the Mac."""
        d = self.drone(**dict(IHFAST, ih=False))
        self.start_daemon(d)
        self.sortie_to_wait("--height", "60")
        code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual((code, out["result"]), (1, "REFUSED"))
        self.assertIn("no interactive hover", out["error"])
        self.assertFalse(d.got(auto.T_MOVE))

    def test_photo_json_has_heading(self):
        d = self.up()
        self.cli("yaw", "--deg", "30")
        code, out, _, _ = self.cli("photo", "--n", "1", "--gap", "150")
        self.assertEqual(code, 0, out)
        with open(out["photos"][0]["json"]) as f:
            doc = json.load(f)
        self.assertEqual((doc["autonomy"]["yawOff_deg"], doc["autonomy"]["hTarget_m"], doc["autonomy"]["D_m"]),
                         (30.0, 1.0, 2.0))

    def test_motion_lost_ack_is_reconciled_not_repeated(self):
        """A MOVE whose result was lost (BUSY taken-unknown) is decided by a fresh AUTO_STATUS (lastCmdType
        MOVE, OK), never sent again blindly."""
        d = self.up(busy_unknown=())
        d.stall = {auto.T_MOVE: (True, 0.0)}
        code, out, _, err = self.cli("move", "--right", "0.4")
        self.assertEqual((code, out["ok"], out.get("reconciled")), (0, True, True), (out, err))
        self.assertEqual(len(self.sent(d, auto.T_MOVE)), 1)
        self.assertEqual(d.mv_seq, 1)


# ---------------------------------------------------------------- the level line and set-height (2026-10-15, a2b 23)

class LevelLineTest(SandboxedTest):
    """The level line (docs/autonomy-a2b.md 23): the formula, its calibration from a ground photo, the drawing."""

    def test_formula(self):
        fy4 = 4 * auto.LEVEL_FY_PX
        self.assertAlmostEqual(fy4, 399.16, places=2)
        for p, t in ((0.0, 0.0), (-2.61, 1.9), (3.0, -1.0)):
            yc = auto.level_row_x4(321.5, p, 0.0, t)
            self.assertAlmostEqual(yc, 241.5 + fy4 * math.tan(math.radians(p + t)), places=6)
            self.assertAlmostEqual(auto.level_row_160(80, p, 0.0, t), (yc - 1.5) / 4, places=6)
        self.assertAlmostEqual(auto.level_row_x4(321.5, 0, 0, 0), 241.5)
        # +pitch nose up (or +tilt camera up): the horizon lower in the image; about 7 px per degree
        self.assertAlmostEqual(auto.level_row_x4(321.5, 1.0, 0, 0) - 241.5, 6.97, places=2)
        self.assertAlmostEqual(auto.level_row_x4(321.5, 0, 0, 1.0), auto.level_row_x4(321.5, 1.0, 0, 0))
        # +roll: the line rises to the right; the slope -tan(roll) per x4 px
        y0, y1 = auto.level_row_x4(0, 0, 5.0, 0), auto.level_row_x4(639, 0, 5.0, 0)
        self.assertLess(y1, y0)
        self.assertAlmostEqual((y1 - y0) / 639, -math.tan(math.radians(5.0)), places=9)
        info = auto.level_info(0.4, 5.0, "flight", None)
        self.assertEqual(info["slope_x4"], round(-math.tan(math.radians(5.0)), 5))
        self.assertEqual([e[0] for e in info["endpoints_x4"]], [0, 639])
        # ground vs flight: the flight tilt is the ground's + 0.3
        st = {"tilt_ground_deg": 1.0, "tilt_flight_deg": 1.3, "set_t": 10.0, "set_utc": "u", "method": "floor"}
        g, f = auto.level_info(0.0, 0.0, "ground", st), auto.level_info(0.0, 0.0, "flight", st)
        self.assertEqual((g["tilt_deg_used"], f["tilt_deg_used"], g["tilt_kind"], f["tilt_kind"]),
                         (1.0, 1.3, "ground", "flight"))
        self.assertFalse(g["stale"])
        d = auto.level_info(0.0, 0.0, "ground", None)
        self.assertEqual((d["tilt_deg_used"], d["stale"], d["tilt_method"]), (auto.LEVEL_DEFAULT_TILT_DEG, True,
                                                                              "default"))
        self.assertAlmostEqual(auto.level_info(0.0, 0.0, "flight", None)["tilt_deg_used"],
                               auto.LEVEL_DEFAULT_TILT_DEG + 0.3, places=9)

    def write_photo(self, stamp, pitch, roll=0.0, state="IDLE", boot="c771102a", frame=None):
        frame = frame or golden_frame()
        meta = auto.parse_meta(auto.meta_payload(meta_for(frame, pitch_cdeg=int(round(pitch * 100)),
                                                          roll_cdeg=int(round(roll * 100)))))
        d = os.path.join(auto.DEFAULT_DIR, "photos")
        os.makedirs(d, exist_ok=True)
        files = auto.save_frame(d, stamp, meta, frame, {"link": {"bootNonce": boot},
                                                        "autonomy": {"state": state, "state_capture": state}})
        return files

    def test_level_cal_floor_reproduces_the_living_room(self):
        """The 10-15 ground photos 122352 / 124340: far floor line at x4 row 240, pitch -2.61 / -2.73, D ~3.5 m: the
        ground tilt +1.82 / +1.94 (about +1.9, the default), the line 4 px above the floor line (FY x 0.035 / 3.5 at
        x4); the flight tilt +0.3."""
        events = []
        tilts = []
        for stamp, pitch in (("20261015-122352", -2.61), ("20261015-124340", -2.73)):
            files = self.write_photo(stamp, pitch)
            out = auto.level_cal_run({"floor_row": 240, "dist": 3.5, "photo": files["png_x4"]},
                                     os.path.dirname(files["png"]), lambda n, **f: events.append((n, f)))
            self.assertTrue(out["ok"], out)
            tilts.append(out["new"]["tilt_ground_deg"])
            self.assertAlmostEqual(out["new"]["tilt_flight_deg"] - out["new"]["tilt_ground_deg"], 0.3, places=6)
            self.assertAlmostEqual(out["floor_gap_px"], 4 * auto.LEVEL_FY_PX * 0.035 / 3.5, places=1)
            self.assertEqual(out["row_x4_center"], round(240 - 4 * auto.LEVEL_FY_PX * 0.035 / 3.5, 2))
            with open(files["json"]) as f:
                doc = json.load(f)
            self.assertEqual((doc["level"]["tilt_kind"], doc["level"]["stale"]), ("ground", False))
            self.assertEqual(doc["level"]["row_x4_center"], out["row_x4_center"])
        self.assertAlmostEqual(tilts[0], 1.82, places=2)
        self.assertAlmostEqual(tilts[1], 1.94, places=2)
        self.assertAlmostEqual(sum(tilts) / 2, auto.LEVEL_DEFAULT_TILT_DEG, delta=0.05)
        self.assertEqual([n for n, _ in events], ["level_cal", "level_cal"])
        doc = auto.load_level()
        self.assertEqual((doc["method"], doc["source"]["row_x4"], doc["source"]["dist_m"], doc["boot_nonce"]),
                         ("floor", 240, 3.5, "c771102a"))
        with open(auto.level_history_path()) as f:
            hist = [json.loads(line) for line in f]
        self.assertEqual([h["kind"] for h in hist], ["set", "set"])
        self.assertIsNone(hist[0]["old"])
        self.assertEqual(hist[1]["old"]["tilt_ground_deg"], tilts[0])
        if os.name == "posix":
            self.assertEqual(os.stat(auto.level_path()).st_mode & 0o777, 0o600)

    def test_level_cal_horizon_set_show_and_defaults(self):
        g = self.write_photo("20261015-100000", -2.0, state="IDLE")
        time.sleep(0.02)
        h = self.write_photo("20261015-100100", 0.5, state="HOVER")
        d = os.path.dirname(g["png"])
        ev = []
        # --floor-row without --photo: the latest ground photo (not the newer hover one)
        out = auto.level_cal_run({"floor_row": 236}, d, lambda n, **f: ev.append(n))
        self.assertEqual(out["photo"], g["json"])
        self.assertAlmostEqual(out["new"]["tilt_ground_deg"],
                               auto.level_tilt_from_row((236 - 1.5) / 4 - auto.LEVEL_FY_PX * 0.035 / 3.0, -2.0), places=3)
        # --horizon-row: the latest flight photo; tilt_flight from the row, ground 0.3 below
        out = auto.level_cal_run({"horizon_row": 249.5}, d, lambda n, **f: ev.append(n))
        self.assertEqual(out["photo"], h["json"])
        self.assertAlmostEqual(out["new"]["tilt_flight_deg"], auto.level_tilt_from_row((249.5 - 1.5) / 4, 0.5), places=3)
        self.assertAlmostEqual(out["new"]["tilt_ground_deg"], out["new"]["tilt_flight_deg"] - 0.3, places=6)
        self.assertEqual(out["row_x4_center"], 249.5)       # the row it was set from (a flight photo)
        out = auto.level_cal_run({"set": 2.2}, d, lambda n, **f: ev.append(n))
        self.assertEqual((out["new"]["tilt_flight_deg"], out["new"]["tilt_ground_deg"], out["method"]), (2.2, 1.9,
                                                                                                         "manual"))
        out = auto.level_cal_run({"show": True}, d, lambda n, **f: ev.append(n))
        self.assertEqual((out["ground"]["tilt_deg"], out["flight"]["tilt_deg"], out["ground"]["stale"]), (1.9, 2.2,
                                                                                                         False))
        self.assertEqual(ev, ["level_cal"] * 3)
        with self.assertRaises(ValueError):
            auto.level_cal_run({"floor_row": 1, "set": 2}, d, lambda n, **f: None)
        # through main(), no daemon: the files only, the event to the events log
        code, out, _, _ = self.run_main(["level-cal", "--floor-row", "240", "--dist", "3.5", "--sock",
                                         os.path.join(self.tmp.name, "none.sock")])
        self.assertEqual((code, out["ok"], out["method"]), (0, True, "floor"))
        with open(auto.default_events()) as f:
            self.assertEqual(json.loads(f.read().splitlines()[-1])["event"], "level_cal")

    def test_level_cal_refuses_a_photo_before_the_invalidation(self):
        """level-cal reads a row off the camera as it is now (2026-10-15 review): a photo taken before the setting's
        latest invalidation (a landing, a contact, a KILL, a new boot), given or the latest by default, is refused, and
        the time is kept through later level-cals; a photo from another boot than the daemon's too; the latest photo is
        chosen by when it was taken, not by the JSON's mtime (rerender_level writes old JSONs again)."""
        stamp = lambda t: time.strftime("%Y%m%d-%H%M%S", time.localtime(t))      # noqa: E731
        now = time.time()
        p1 = self.write_photo(stamp(now - 120), -2.6)
        d = os.path.dirname(p1["png"])
        out = auto.level_cal_run({"floor_row": 240, "dist": 3.5}, d, lambda n, **f: None)
        self.assertTrue(out["ok"], out)
        self.assertEqual(out["photo"], p1["json"])
        self.assertAlmostEqual(out["photo_age_s"], 120, delta=3)
        # a landing 60 s ago: the line stale; the same photo again (the default, the latest ground photo): refused
        st = auto.load_level()
        st.update(invalidated_t=now - 60, invalidated_utc=auto.utc_str(now - 60), invalidated_why="a landing")
        auto.write_json_atomic(auto.level_path(), st)
        out = auto.level_cal_run({"floor_row": 240, "dist": 3.5}, d, lambda n, **f: None)
        self.assertFalse(out["ok"])
        self.assertIn("before a landing", out["error"])
        self.assertEqual(out["photo"], p1["json"])
        out = auto.level_cal_run({"horizon_row": 240, "photo": p1["json"]}, d, lambda n, **f: None)
        self.assertFalse(out["ok"])                                    # given explicitly: refused too
        # a newer photo: accepted; the invalidation's time is kept, so the old photo stays refused
        p2 = self.write_photo(stamp(now - 5), -2.7)
        out = auto.level_cal_run({"floor_row": 240, "dist": 3.5}, d, lambda n, **f: None)
        self.assertTrue(out["ok"], out)
        self.assertEqual(out["photo"], p2["json"])
        self.assertAlmostEqual(auto.load_level()["valid_after_t"], now - 60, places=3)
        self.assertIsNone(auto.load_level()["invalidated_t"])
        out = auto.level_cal_run({"floor_row": 240, "photo": p1["json"]}, d, lambda n, **f: None)
        self.assertFalse(out["ok"])
        # the latest by capture time: the old photo's JSON rewritten (its mtime newest) does not make it the latest
        auto.rerender_level(p1["json"], auto.load_level())
        os.utime(p1["json"], (now + 10, now + 10))
        self.assertEqual(auto.latest_photo_json(d, "ground"), p2["json"])
        # another boot than the daemon's: refused (set by hand is not a row off a photo: accepted)
        out = auto.level_cal_run({"floor_row": 240}, d, lambda n, **f: None, boot="bbbbbbbb")
        self.assertFalse(out["ok"])
        self.assertIn("another boot", out["error"])
        self.assertTrue(auto.level_cal_run({"floor_row": 240}, d, lambda n, **f: None, boot="c771102a")["ok"])
        self.assertTrue(auto.level_cal_run({"set": 2.0}, d, lambda n, **f: None, boot="bbbbbbbb")["ok"])
        # the time a photo was taken: received_t, else received_utc, else the name's stamp
        self.assertEqual(auto.photo_time({"link": {"received_t": 12.5}}), 12.5)
        self.assertEqual(auto.photo_time({"link": {"received_utc": "1970-01-01T00:01:40Z"}}), 100.0)
        self.assertAlmostEqual(auto.photo_time({}, p2["json"]), now - 5, delta=1.01)
        self.assertIsNone(auto.photo_time({}, "x.json"))

    def test_stale_by_boot(self):
        st = {"tilt_ground_deg": 1.0, "tilt_flight_deg": 1.3, "set_t": 10.0, "set_utc": "u", "method": "floor",
              "boot_nonce": "aaaaaaaa"}
        self.assertFalse(auto.level_choice(st, "ground", "aaaaaaaa")[3])
        t, _, _, stale, why = auto.level_choice(st, "ground", "bbbbbbbb")
        self.assertTrue(stale)
        self.assertIn("rebooted", why)
        st.update(invalidated_t=11.0, invalidated_why="a landing")
        self.assertEqual(auto.level_choice(st, "flight", "aaaaaaaa")[3:], (True, "a landing"))
        st["set_t"] = 12.0                                   # set again after it
        self.assertFalse(auto.level_choice(st, "flight", "aaaaaaaa")[3])

    def blocks_kept(self, a, b):
        """min over the 4x4 source blocks away from the edge ticks (X 13..626) of the pixels left unchanged."""
        pa, pb = a.load(), b.load()
        worst = 16
        for bx in range(4, 156):
            for by in range(0, 120):
                same = sum(pa[4 * bx + i, 4 * by + j] == pb[4 * bx + i, 4 * by + j] for i in range(4) for j in range(4))
                worst = min(worst, same)
        return worst

    def test_drawing_hides_no_source_pixel(self):
        from PIL import Image
        frame = other_frame(3)
        img = Image.frombytes("L", (160, 120), auto.flip_rows(frame, 160, 120))
        plain = auto.render_x4(img)
        for pitch, roll, stale in ((0.0, 0.0, False), (-2.6, -0.1, False), (1.0, 7.0, False), (-3.0, -12.0, True),
                                   (2.0, 0.3, True)):
            st = None if stale else {"tilt_ground_deg": 1.9, "tilt_flight_deg": 2.2, "set_t": 1.0, "set_utc": "u",
                                     "method": "floor"}
            info = auto.level_info(pitch, roll, "flight", st)
            drawn = auto.render_x4(img, level=info)
            self.assertGreaterEqual(self.blocks_kept(plain, drawn), 12, (pitch, roll))
            px = drawn.load()
            on = [X for X in range(13, 627) if px[X, int(math.floor(auto.level_row_x4(X, pitch, roll,
                                                                                      info["tilt_deg_used"]) + 0.5))]
                  in auto.LEVEL_RGB]
            frac = len(on) / (627 - 13)
            self.assertAlmostEqual(frac, 0.25 if stale else 0.5, delta=0.04, msg=(pitch, roll, stale))
            # both colours, the edge ticks at the level row, the angle ticks at the right edge
            self.assertTrue(any(px[X, int(round(auto.level_row_x4(X, pitch, roll, info["tilt_deg_used"])))] ==
                                auto.LEVEL_RGB[1] for X in range(13, 627)))
            ye = int(math.floor(auto.level_row_x4(5.5, pitch, roll, info["tilt_deg_used"]) + 0.5))
            self.assertEqual((px[0, ye], px[11, ye], px[0, ye - 2], px[12, ye]), (auto.LEVEL_RGB[0],) * 2 + ((0, 0, 0),) * 2)
            yc = info["row_x4_center"]
            y5 = int(math.floor(yc + 4 * auto.LEVEL_FY_PX * (math.tan(math.radians(pitch + info["tilt_deg_used"] + 5)) -
                                                             math.tan(math.radians(pitch + info["tilt_deg_used"]))) + 0.5))
            self.assertEqual((px[628, y5], px[639, y5], px[627, y5] == auto.LEVEL_RGB[0]), (auto.LEVEL_RGB[0],) * 2 +
                             (False,))

    def test_out_of_frame_marker(self):
        from PIL import Image
        img = Image.frombytes("L", (160, 120), bytes(160 * 120))
        for pitch, top in ((-40.0, True), (40.0, False)):
            info = auto.level_info(pitch, 0.0, "ground", None)
            self.assertFalse(info["in_frame"])
            self.assertLess(info["out_of_frame_px"], 0) if top else self.assertGreater(info["out_of_frame_px"], 0)
            px = auto.render_x4(img, level=info).load()
            y = 0 if top else 479
            self.assertIn(px[321, y], auto.LEVEL_RGB)
            self.assertNotIn(px[321, 240], auto.LEVEL_RGB)
        self.assertTrue(auto.level_info(0.0, 0.0, "ground", None)["in_frame"])

    def test_save_frame_level_block_and_raw_png(self):
        from PIL import Image
        frame = golden_frame()
        meta = auto.parse_meta(auto.meta_payload(meta_for(frame)))
        for sub in ("a", "b", "c"):
            os.makedirs(os.path.join(self.tmp.name, sub))
        a = auto.save_frame(os.path.join(self.tmp.name, "a"), "s", meta, frame, {})
        info = auto.level_info(meta["pitch_deg"], meta["roll_deg"], "ground", None)
        b = auto.save_frame(os.path.join(self.tmp.name, "b"), "s", meta, frame, {}, level=info)
        with open(a["png"], "rb") as fa, open(b["png"], "rb") as fb:
            self.assertEqual(fa.read(), fb.read())           # the 160x120 png stays raw
        with Image.open(a["png_x4"]) as ia, Image.open(b["png_x4"]) as ib:
            self.assertNotEqual(ia.tobytes(), ib.tobytes())
        with open(b["json"]) as f:
            doc = json.load(f)
        for k in ("row_x4_center", "row_160_center", "slope_x4", "endpoints_x4", "pitch_deg", "roll_deg",
                  "tilt_deg_used", "tilt_kind", "tilt_set_utc", "tilt_method", "stale", "stale_why", "fy_px", "formula",
                  "ticks_deg", "in_frame"):
            self.assertIn(k, doc["level"])
        # ESPFLY_LEVEL_LINE=0 / --no-level: nothing drawn, the block written
        off = auto.level_info(meta["pitch_deg"], meta["roll_deg"], "ground", None, draw=False)
        c = auto.save_frame(os.path.join(self.tmp.name, "c"), "s", meta, frame, {}, level=off)
        with Image.open(a["png_x4"]) as ia, Image.open(c["png_x4"]) as ic:
            self.assertEqual(ia.tobytes(), ic.tobytes())
        with open(c["json"]) as f:
            self.assertFalse(json.load(f)["level"]["drawn"])
        with mock.patch.dict(os.environ, {auto.LEVEL_ENV: "0"}):
            self.assertFalse(auto.level_drawing_on())
        self.assertTrue(auto.level_drawing_on())


class CaptureStatusTest(SandboxedTest):
    """The height estimate when a frame was captured: META captureUpMs is on the esp_timer clock (STATUS upMs), the
    AUTO_STATUS snapshots on the FreeRTOS tick (a boot-constant offset): mapped through the Mac's clock."""

    def test_capture_and_arrival(self):
        link = auto.Link(KEY, "127.0.0.1", 9)
        self.addCleanup(link.close)
        dm = auto.Daemon(link, os.path.join(self.tmp.name, "x.sock"), self.tmp.name)
        # snapshots every 100 ms on the tick clock, the estimate rising 1 mm per ms; Mac time = tick / 1000 + 50
        for k in range(50):
            dm.ring.append((10000 + 100 * k, 1000 + 100 * k, "HOVER" if k > 5 else "TAKEOFF", 50.0 + 10.0 + 0.1 * k))
        dm.clock_off = 50.0                         # Mac - tick
        dm.esp_off = 49.7                           # Mac - esp_timer: the esp clock 300 ms ahead of the tick
        h, st, src = dm.capture_status({"captureUpMs": 12350 + 300}, 99.0)   # tick 12350
        self.assertEqual((src, st), ("capture", "HOVER"))
        self.assertAlmostEqual(h, 3.35, places=6)
        h, st, src = dm.capture_status({"captureUpMs": 10050 + 300}, None)
        self.assertEqual((src, st, round(h, 3)), ("capture", "TAKEOFF", 1.05))
        # far outside the ring: the snapshot nearest the META's arrival
        h, st, src = dm.capture_status({"captureUpMs": 900000}, 60.0 + 2.05)
        self.assertEqual((src, round(h, 3)), ("arrival", 3.0))
        dm.esp_off = None                           # no STATUS yet in this boot
        h, st, src = dm.capture_status({"captureUpMs": 12650}, 60.0 + 0.42)
        self.assertEqual((src, round(h, 3)), ("arrival", 1.4))
        self.assertEqual(dm.capture_status({"captureUpMs": 1}, None), (None, None, None))
        dm.ring.clear()
        self.assertEqual(dm.capture_status({"captureUpMs": 1}, 1.0), (None, None, None))
        # the esp clock's offset: the lowest Mac - upMs, leaking upward slowly (as note_clock)
        dm.note_esp_clock({"upMs": 1000}, 101.0)
        dm.note_esp_clock({"upMs": 2000}, 102.05)
        self.assertAlmostEqual(dm.esp_off, 100.0, places=3)
        dm.note_esp_clock({"upMs": 3000}, 102.9)
        self.assertAlmostEqual(dm.esp_off, 99.9, places=6)


SHFAST = dict(IHFAST, wait_ms=60000, mv_speed=40.0)


class SetHeightArithmeticTest(unittest.TestCase):
    """The Mac's copy of the drone's target-form arithmetic (auto_fsm.c setHeightCmd) on the logged corrections of
    2026-10-15 / 10-16 (docs/autonomy-a2b.md 27.2, Claude's judgements in RESUME.md): true now, the flight, the end at
    the intended 1.00 m; the 4-byte fallback's X lands there too (h_now + X - x)."""

    CASES = [  # what, X judged, h_at, h_now (the ACK's old estimate)
        ("10-15 p2", 1.05, 0.91, 0.90), ("10-15 p1", 1.08, 0.90, 0.89), ("10-15 fastcheck a", 1.28, 0.93, 0.93),
        ("10-15 fastcheck b", 0.82, 0.92, 0.93), ("10-16 F1", 1.12, 0.90, 0.90), ("10-16 F2 a", 1.15, 0.89, 0.89),
        ("10-16 F2 b", 0.60, 0.89, 0.90), ("10-16 tiger", 0.70, 0.85, 1.06)]

    def test_replays(self):
        for what, x, at, h_now in self.CASES:
            with self.subTest(what=what):
                p = auto.set_height_plan(x, at, h_now, 1.00)
                self.assertAlmostEqual(p["true_now_m"] + p["fly_m"], 1.00, places=3)
                self.assertAlmostEqual(p["true_now_m"], h_now + x - at, places=3)
                self.assertLessEqual(abs(p["fly_m"]), auto.SH_FLY_MAX_M)
                xl = auto.set_height_legacy_x(x, at, h_now, 1.00)
                self.assertAlmostEqual(h_now + x - xl, 1.00, places=6)   # the 4-byte form's true end
        p = auto.set_height_plan(0.90, None, 0.90, 1.20)
        self.assertEqual((p["offset_m"], p["true_now_m"], p["fly_m"]), (0.0, 0.9, 0.3))

    def test_payload(self):
        self.assertEqual(auto.set_height_payload(110, 90), struct.pack("<hh", 110, 90))
        self.assertEqual(auto.set_height_payload(110, 90, auto.SH_TO_REF), struct.pack("<hhh", 110, 90, -32768))
        self.assertEqual(auto.SH_TO_M, (0.25, 2.00))

    def test_hold_limit_event(self):
        e = auto.event_view(auto.EV_HOLD_LIMIT, auto.HL_ROLL, 120)
        self.assertEqual((e["name"], e["detailName"]), ("HOLD_LIMIT", [{"axis": "roll", "sign": "+", "creeps": "left"}]))
        self.assertIn("trimcheck", e["hint"])
        e = auto.event_view(auto.EV_HOLD_LIMIT, auto.HL_ROLL | auto.HL_PITCH | auto.HL_PITCH_NEG | auto.HL_ROLL_NEG, 0)
        self.assertEqual(e["detailName"], [{"axis": "roll", "sign": "-", "creeps": "right"},
                                           {"axis": "pitch", "sign": "-", "creeps": "back"}])
        e = auto.event_view(auto.EV_HOLD_LIMIT, auto.HL_PITCH | auto.HL_ROLL_NEG, 0)   # only the rising axis listed
        self.assertEqual(e["detailName"], [{"axis": "pitch", "sign": "+", "creeps": "forward"}])
        self.assertEqual(auto.MV_WHY_NAMES[16], "UP_HEADROOM")
        self.assertIn("UP_HEADROOM", auto.SH_WHY_HINTS)


class DaemonSetHeightTest(DaemonCase):
    """set-height (docs/autonomy-a2b.md 23): the offset form against the latest photo's estimate at capture, --now,
    the local refusals, the old firmware, the events, wait-move and the status."""

    def up(self, photos="1", **kw):
        d = self.drone(**dict(SHFAST, **kw))
        self.start_daemon(d)
        self.sortie_to_wait("--height", "100", "--photos", photos, "--wait-s", "60")
        return d

    def photo(self):
        code, out, _, _ = self.cli("photo")
        self.assertEqual((code, out["ok"]), (0, True), out)
        return out["photos"][-1]

    def test_offset_form_from_the_latest_photo(self):
        d = self.up()
        d.h_mm = 980
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 980, 2))
        ph = self.photo()
        self.assertEqual((ph["h_est_capture_m"], ph["tilt_kind"], ph["level_stale"]), (0.98, "flight", True))
        self.assertIn(ph["h_src"], ("capture", "arrival"))
        with open(ph["json"]) as f:
            doc = json.load(f)
        self.assertEqual((doc["autonomy"]["h_est_capture_m"], doc["autonomy"]["sortie_key"]),
                         (0.98, self.dm.sortie_key))
        self.assertIn("level", doc)
        d.h_mm = 1010                          # the estimate moved since the photo: the offset form ignores it
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 1010, 2))
        code, out, _, _ = self.cli("set-height", "1.10")
        self.assertEqual((code, out["ok"], out["result"], out["end"]), (0, True, "OK", "DONE"), out)
        self.assertEqual(d.sh_cmds, [(110, 98, 0.12, auto.SH_TO_REF)])     # the 6-byte target form (2026-10-16)
        self.assertEqual((out["old_m"], out["old_from"], out["offset_m"], out["new_m"], out["true_now_m"],
                          out["target_m"], out["target_from"], out["fly_m"], out["form"], out["sh_form"], out["at_m"],
                          out["at_photo"]),
                         (1.01, "ACK", 0.12, 1.13, 1.13, 1.0, "commanded", -0.13, "photo", "target", 0.98, ph["json"]))
        self.assertEqual(out["seq"], 1)
        self.assertEqual(out["mv"]["kind"], "SET_HEIGHT")
        self.assertIsNotNone(out["since_set_height_s"])
        self.assertIsNotNone(out["since_photo_s"])
        ev = self.events("set_height")
        self.assertEqual((ev[0]["result"], ev[0]["offset_m"], ev[0]["new_m"], ev[0]["x_m"], ev[0]["at_m"],
                          ev[0]["true_now_m"], ev[0]["target_m"], ev[0]["fly_m"]),
                         ("OK", 0.12, 1.13, 1.1, 0.98, 1.13, 1.0, -0.13))
        end = self.events("set_height_end")
        self.assertEqual((end[0]["end"], end[0]["seq"]), ("DONE", 1))
        # one photo is never applied twice
        code, out, _, _ = self.cli("set-height", "1.00")
        self.assertEqual((code, out["result"]), (1, "REFUSED"), out)
        self.assertIn("no photo since the last accepted set-height", out["error"])
        self.assertEqual(len(d.sh_cmds), 1)
        # wait-move: the latest motion is a correction, not Claude's MOVE
        code, out, _, _ = self.cli("wait-move")
        self.assertEqual((code, out["kind"], out["end"]), (0, "SET_HEIGHT", "DONE"), out)
        self.assertIn("SET_HEIGHT correction", out["note"])
        # --now needs no photo; the drone's 2 s bound answers SH_RATE
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "0.90", "--now")
        self.assertEqual((code, out["ok"], out["form"]), (0, True, "now"), out)
        self.assertEqual(d.sh_cmds[-1][1], auto.SH_AT_NOW)
        self.assertAlmostEqual(out["offset_m"], 0.90 - out["old_m"], places=6)
        code, out, _, _ = self.cli("set-height", "0.80", "--now")
        self.assertEqual((code, out["result"], out["why"]), (1, "BAD_STATE", "SH_RATE"), out)
        self.assertIn("wait 2 s", out["error"])
        # a new photo: the offset form again (2 s after the accepted --now)
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        ph2 = self.photo()
        self.assertEqual(ph2["h_src"], "capture")          # the fake's clocks: esp_timer 300 ms ahead of the tick
        code, out, _, _ = self.cli("set-height", "1.40", "--no-wait")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.cli("land")

    def test_refusals_and_hints(self):
        d = self.up(photos="0")
        code, out, _, _ = self.cli("set-height", "1.00")
        self.assertEqual((code, out["result"]), (1, "REFUSED"))
        self.assertIn("no photo saved", out["error"])
        self.photo()
        code, out, _, _ = self.cli("set-height", "1.00")      # 0.32 -> 1.00: d 0.68
        self.assertEqual((code, out["result"], out["why"]), (1, "RANGE", "SH_STEP"), out)
        self.assertIn("send 0.50 m of it", out["error"])
        code, out, _, _ = self.cli("set-height", "0.62", "--at", "0.32")
        self.assertEqual((code, out["ok"], out["form"]), (0, True, "at"), out)
        self.assertEqual(d.sh_cmds[-1][:2], (62, 32))
        code, out, _, _ = self.cli("set-height", "3.0")        # the client's range
        self.assertEqual(code, 2)
        self.assertEqual(len(d.sh_cmds), 1)
        # another sortie: its photo refused
        self.cli("land")
        self.assertTrue(wait_for(lambda: d.state == IDLE, 5))
        old = self.dm.photo_log[-1]["json"]
        self.sortie_to_wait("--height", "100", "--photos", "1", "--wait-s", "60")
        code, out, _, _ = self.cli("set-height", "0.50", "--photo", old)
        self.assertEqual((code, out["result"]), (1, "REFUSED"), out)
        self.assertIn("another sortie or boot", out["error"])
        self.cli("land")

    def test_unknown_outcome_uses_the_photo(self):
        """A set-height whose outcome is unknown (no ACK after the resends: the drone may have applied it) marks the
        photos up to then as used: the next one in the offset form needs a newer photo, so one photo is never applied
        twice (2026-10-15 review: the guard was set on an ACK OK only)."""
        d = self.up(mute_acks={auto.T_SET_HEIGHT})
        d.h_mm = 980
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 980, 2))
        self.photo()
        code, out, _, _ = self.cli("set-height", "1.10", "--no-wait")
        self.assertEqual((code, out["ok"], out.get("outcome")), (1, False, "unknown"), out)
        self.assertIn("outcome is unknown", out["error"])
        self.assertEqual(len(d.sh_cmds), 1)                      # the drone took it
        self.assertEqual(self.events("set_height")[-1]["outcome"], "unknown")
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.sh_used[0] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "1.10")          # the same (default) photo: refused locally
        self.assertEqual((code, out["result"]), (1, "REFUSED"), out)
        self.assertIn("outcome is unknown", out["error"])
        self.assertEqual(len(d.sh_cmds), 1)
        with d.lock:
            d.mute_acks.discard(auto.T_SET_HEIGHT)
        self.photo()                                              # a newer photo: sent again
        code, out, _, _ = self.cli("set-height", "1.10", "--no-wait")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.assertEqual(len(d.sh_cmds), 2)
        self.cli("land")

    def test_old_firmware(self):
        d = self.up(sh=False)
        code, out, _, _ = self.cli("set-height", "0.40", "--now")
        self.assertEqual((code, out["ok"], out["why"]), (1, False, "UNSUPPORTED"), out)
        self.assertIn("firmware has no SET_HEIGHT", out["error"])
        self.assertEqual(self.events("set_height")[-1]["result"], "UNSUPPORTED")
        self.cli("land")

    def test_target_form(self):
        """"You are at X, go to the target" (2026-10-16, docs/autonomy-a2b.md 27): the commanded height or --to, the
        fields true_now_m / target_m / fly_m, the clip of a long flight, the estimate at the target afterwards."""
        d = self.up()
        d.h_mm = 900                        # HOLD's estimate 0.10 m under the commanded 1.00 m
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 900, 2))
        self.photo()
        code, out, _, _ = self.cli("set-height", "1.10")
        self.assertEqual((code, out["ok"], out["end"]), (0, True, "DONE"), out)
        self.assertEqual(d.sh_cmds[-1], (110, 90, 0.2, auto.SH_TO_REF))
        self.assertEqual((out["true_now_m"], out["target_m"], out["fly_m"], out["offset_m"]), (1.1, 1.0, -0.1, 0.2))
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 1000 and self.dm.auto["hTargetCm"] == 100, 2))
        # --to: the new commanded height; d 0 is fine (the flight alone)
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "1.00", "--now", "--to", "1.20")
        self.assertEqual((code, out["ok"], out["fly_m"], out["target_from"], out["target_m"]),
                         (0, True, 0.2, "--to", 1.2), out)
        self.assertEqual(d.sh_cmds[-1][3], 120)
        self.assertTrue(wait_for(lambda: self.dm.auto["hTargetCm"] == 120 and self.dm.auto["hMm"] == 1200, 2))
        # a flight over 0.55 m: 0.50 m of it, the rest named
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "0.75", "--now")     # d -0.45, true now 0.75: 0.45 to go -> fine
        self.assertEqual((code, out["ok"], out["fly_m"]), (0, True, 0.45), out)
        self.assertNotIn("clipped_to_m", out)
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "0.75", "--now", "--to", "1.40")   # 0.65 to go
        self.assertEqual((code, out["ok"], out["clipped_to_m"], out["rest_m"], out["fly_m"]), (0, True, 1.25, 0.15, 0.5),
                         out)
        self.assertEqual(d.sh_cmds[-1][3], 125)
        # the clipped flight's target is remembered (2026-10-16 review): the next plain set-height flies the rest to
        # 1.40, not to the clip point (the commanded height 1.25 now)
        self.assertTrue(wait_for(lambda: self.dm.auto["hTargetCm"] == 125, 2))
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "1.25", "--now")
        self.assertEqual((code, out["ok"], out["target_from"], out["target_m"], out["fly_m"], out["flies_to_m"]),
                         (0, True, "remembered", 1.4, 0.15, 1.4), out)
        self.assertIn("clipped set-height", out["remembered_note"])
        self.assertEqual(d.sh_cmds[-1][3], 140)
        self.assertIsNone(self.dm.sh_goal)                  # reached: the commanded height again
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "1.30", "--now")
        self.assertEqual((code, out["ok"], out["target_from"], out["target_m"]), (0, True, "commanded", 1.4), out)
        self.assertEqual(d.sh_cmds[-1][3], auto.SH_TO_REF)
        # a vertical MOVE after a clip drops the remembered target
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "0.90", "--now", "--to", "1.60")   # 0.70 to go: clipped at 1.40
        self.assertEqual((code, out["ok"], out["clipped_to_m"], out["rest_m"]), (0, True, 1.4, 0.2), out)
        self.assertEqual(self.dm.sh_goal["target_m"], 1.6)
        code, out, _, _ = self.cli("move", "--up", "0.05")
        self.assertEqual((code, out["ok"], out["remembered_target_cleared_m"]), (0, True, 1.6), out)
        self.assertIsNone(self.dm.sh_goal)
        self.cli("land")

    def test_floor_legacy_and_warnings(self):
        """2026-10-16 review: a target-form descent under a MOVE down's floor is refused here; --legacy sends the 4-byte
        form (the A/B); a photo with events since it and a climb late in the battery are warnings."""
        d = self.up(hr=True)
        d.h_mm = 900
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 900, 2))
        d.air_ms_offset = 40000                                   # 40 s more of air: the floor 0.55
        self.assertTrue(wait_for(lambda: self.dm.auto["air_s"] > 40.0, 2))
        n = len(d.sh_cmds)
        code, out, _, _ = self.cli("set-height", "0.90", "--now", "--to", "0.50")
        self.assertEqual((code, out["ok"], out["why"], out["floor_m"]), (1, False, "HEIGHT", 0.55), out)
        self.assertIn("floor", out["error"])
        self.assertEqual(len(d.sh_cmds), n)                       # nothing sent
        code, out, _, _ = self.cli("set-height", "0.90", "--now", "--to", "0.60")
        self.assertEqual((code, out["ok"], out["fly_m"]), (0, True, -0.3), out)
        # --legacy: the 4-byte form, back to the old estimate (true end = the old estimate)
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 600, 2))
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "0.70", "--now", "--legacy")
        self.assertEqual((code, out["ok"], out["sh_form"], out["fly_m"], out["new_m"], out["flies_to_m"]),
                         (0, True, "legacy", -0.1, 0.7, 0.6), out)
        self.assertEqual(d.sh_lens[-1], 4)
        code, out, _, _ = self.cli("set-height", "0.70", "--now", "--legacy", "--to", "1.0")
        self.assertEqual((code, out["ok"]), (1, False), out)
        # a photo, then a contact: the set-height from that photo warns
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        self.photo()
        d.report(auto.EV_CONTACT, 0x84, contacts=1)
        self.assertTrue(wait_for(lambda: self.dm.auto["evCode"] == auto.EV_CONTACT, 2))
        code, out, _, _ = self.cli("set-height", "0.62", "--no-wait")
        self.assertEqual((code, out["ok"], out["events_since_photo"]), (0, True, ["CONTACT"]), out)
        self.assertIn("since the photo was captured", " ".join(out["warnings"]))
        self.cli("land")

    def test_late_climb_warning(self):
        """A climb late in the battery: a warning, never a refusal (F2 vs fastcheck, 2026-10-16 review)."""
        self.assertIsNone(auto.late_climb_warning({"tHold": 52000, "vbat_v": 3.15}, -0.1))     # a descent
        self.assertIsNone(auto.late_climb_warning({"tHold": 41000, "vbat_v": 3.12}, 0.3))      # the tiger climb
        self.assertIsNone(auto.late_climb_warning({"tHold": 52000, "vbat_v": 3.21}, 0.3))
        self.assertIn("F2", auto.late_climb_warning({"tHold": 51989, "vbat_v": 3.172}, 0.42))  # F2's climb
        d = self.up(t_hold=52000, vbat_mv=3150)
        d.h_mm = 900
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 900, 2))
        code, out, _, _ = self.cli("set-height", "0.80", "--now")
        self.assertEqual((code, out["ok"], out["fly_m"]), (0, True, 0.2), out)
        self.assertIn("late in the battery", " ".join(out["warnings"]))
        code, out, _, _ = self.cli("move", "--up", "0.05")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.assertIn("late in the battery", " ".join(out["warnings"]))
        self.cli("land")

    def test_target_form_on_an_older_drone(self):
        """A drone without the target form (7a78e1b, 8277b68) answers the 6-byte SET_HEIGHT RANGE detail 0 (the link's
        length check, not processed): the 4-byte form follows with X + h_now - target, which ends at the target."""
        d = self.up(sh_to=False)
        d.h_mm = 900
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 900, 2))
        self.photo()
        code, out, _, _ = self.cli("set-height", "1.15")
        self.assertEqual((code, out["ok"], out["sh_form"], out["legacy_x_m"]), (0, True, "legacy-target", 1.05), out)
        self.assertEqual(d.sh_cmds, [(105, 90, 0.15)])       # the 4-byte form (the 6-byte one was not processed)
        self.assertEqual(d.sh_lens, [4])
        self.assertEqual((out["true_now_m"], out["target_m"], out["fly_m"]), (1.15, 1.0, -0.15))
        # the drone's estimate right after its shift is h_now + (X' - h_at) = 1.05, not the true 1.15 (review)
        self.assertEqual((out["new_m"], out["flies_to_m"]), (1.05, 1.0))
        self.assertIn("no target form", out["legacy_note"])
        self.assertEqual(self.events("set_height_form")[-1]["form"], "legacy-target")
        self.cli("land")

    def test_up_headroom(self):
        """A climb is refused on thrust headroom (UP_HEADROOM, LOW_BATT), never on the voltage; a descent flies."""
        d = self.up(t_hold=54000, vbat_mv=3150)
        d.h_mm = 900
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 900, 2))
        code, out, _, _ = self.cli("set-height", "0.70", "--now")
        self.assertEqual((code, out["ok"], out["why"]), (1, False, "UP_HEADROOM"), out)
        self.assertIn("the thrust, not the voltage", out["error"])
        self.assertIn("UP_HEADROOM", " ".join(out["warnings"]))
        code, out, _, _ = self.cli("set-height", "1.20", "--now")
        self.assertEqual((code, out["ok"], out["fly_m"]), (0, True, -0.2), out)
        # the tiger flight's climb (3.12 V loaded, T_hold 41k): accepted, no voltage rule
        with d.lock:
            d.t_hold, d.vbat_mv = 41000, 3120
        self.assertTrue(wait_for(lambda: time.monotonic() - self.dm.last_sh["mono"] > 2.1, 3))
        code, out, _, _ = self.cli("set-height", "0.70", "--now")
        self.assertEqual((code, out["ok"], out["fly_m"]), (0, True, 0.3), out)
        self.assertNotIn("warnings", out)
        self.cli("land")

    def test_old_photo_warning(self):
        d = self.up()
        d.h_mm = 900
        self.assertTrue(wait_for(lambda: self.dm.auto["hMm"] == 900, 2))
        ph = self.photo()
        with open(ph["json"]) as f:
            doc = json.load(f)
        doc["autonomy"]["capture_mono"] -= 30.0          # taken 30 s ago (the tiger flight's correction)
        with open(ph["json"], "w") as f:
            json.dump(doc, f)
        code, out, _, _ = self.cli("set-height", "0.80", "--no-wait")
        self.assertEqual((code, out["ok"]), (0, True), out)
        self.assertGreaterEqual(out["photo_age_s"], 30.0)
        self.assertIn("30 s old", " ".join(out["warnings"]))
        self.cli("land")

    def test_level_invalidation(self):
        """The daemon marks the level setting stale on a landing, a KILL, a contact and a new boot (mechanical facts);
        a level-cal after it makes it fresh again."""
        d = self.drone(**dict(SHFAST, hr=True))
        self.start_daemon(d)
        auto.write_json_atomic(auto.level_path(), {"tilt_ground_deg": 1.0, "tilt_flight_deg": 1.3, "set_t": 1.0,
                                                   "set_utc": "u", "method": "manual"})
        self.sortie_to_wait("--height", "100", "--photos", "1", "--wait-s", "60")
        self.assertFalse(auto.load_level().get("invalidated_why"))
        d.report(auto.EV_CONTACT, 0x84, contacts=1)
        self.assertTrue(wait_for(lambda: (auto.load_level() or {}).get("invalidated_why") == "an in-air contact", 3))
        ph = self.photo()
        self.assertTrue(ph["level_stale"])
        out = auto.level_cal_run({"set": 1.5}, self.photos, lambda n, **f: None)
        self.assertTrue(out["ok"])
        self.assertFalse(self.photo()["level_stale"])
        self.cli("land")
        self.assertTrue(wait_for(lambda: auto.load_level().get("invalidated_why") == "a landing", 5))
        auto.level_cal_run({"set": 1.5}, self.photos, lambda n, **f: None)
        self.sortie_to_wait("--height", "100", "--photos", "1", "--wait-s", "60")
        self.oneshot(d, "kill")
        self.assertTrue(wait_for(lambda: (auto.load_level().get("invalidated_why") or "").startswith("KILLED"), 5))
        auto.level_cal_run({"set": 1.5}, self.photos, lambda n, **f: None)
        with d.lock:
            d.reboot()
        self.assertTrue(wait_for(lambda: "bootNonce" in (auto.load_level().get("invalidated_why") or ""), 5))
        whys = [e["why"] for e in self.events("level_invalidated")]
        self.assertEqual(whys[:3], ["an in-air contact", "a landing", "KILLED (the motors cut in the air)"])
        with open(auto.level_history_path()) as f:
            kinds = [json.loads(line)["kind"] for line in f]
        self.assertEqual(kinds.count("invalidated"), 4)


MVSPEED_GOLDEN_PATH = os.path.join(HERE, "mvspeed_golden.json")


class MvSpeedMirrorTest(unittest.TestCase):
    """Fast moves (mvspeed, 2026-10-16, docs/autonomy-a2b.md 31): the 140-byte AUTO_STATUS, the limits and the profile
    mirror against auto_motion.c (mvspeed_golden.json, written by test_auto_motion.c --write-golden), the settle in the
    battery fit, the person rule's speed term."""

    def test_status_lengths(self):
        f = {"state": auto.S_HOVER, "mvSpeedPct": 250}
        a = auto.parse_auto_status(auto.auto_status_payload(f))
        self.assertEqual((len(auto.auto_status_payload(f)), a["mvSpeedPct"], a["speedStep"], a["hr"]), (140, 250, True,
                                                                                                       True))
        for n, hr in ((auto.AUTO_STATUS_LEN_HR, True), (auto.AUTO_STATUS_LEN_IH, False), (auto.AUTO_STATUS_LEN_A2B,
                                                                                          False)):
            a = auto.parse_auto_status(auto.auto_status_payload(f, n))
            self.assertEqual((a["mvSpeedPct"], a["speedStep"], a["hr"]), (100, False, hr), n)
        a = auto.parse_auto_status(auto.auto_status_payload({"mvSpeedPct": 0}))
        self.assertEqual(a["mvSpeedPct"], 100)   # a bad 0: 100
        for bad in (135, 137, 139, 141):
            with self.assertRaises(auto.ProtoError):
                auto.parse_auto_status(bytes(bad))
        self.assertEqual(auto.Daemon.motion_view(auto.parse_auto_status(auto.auto_status_payload(f)))["mvspeed_pct"],
                         250)
        with open(os.path.join(REPO, "components", "core", "crazyflie", "utils", "interface", "auto_proto.h")) as fh:
            src = fh.read()
        self.assertRegex(src, r"#define AUTO_AUTO_STATUS_HR_LEN\s+%d\b" % auto.AUTO_STATUS_LEN_HR)

    def test_limits_match_the_header(self):
        """mv_limits at the steps the C test pins (test_auto_motion.c testSpeedCfg) and the header's caps."""
        with open(os.path.join(REPO, "components", "core", "crazyflie", "utils", "interface", "auto_motion.h")) as fh:
            src = fh.read()

        def hdr(name):
            return float(re.search(r"#define %s\s+([0-9.]+)f?\b" % name, src).group(1))
        caps = {"v_h": "V_H", "v_back": "V_BACK", "v_fwd": "V_FWD", "a_h": "A_H", "w_lat": "W_LAT", "w_fwd": "W_FWD",
                "brake": "BRAKE", "yaw_rate": "YAW_RATE", "yaw_acc": "YAW_ACC", "bob_acc": "BOB_ACC"}
        for k, n in caps.items():
            self.assertEqual(auto.MV_FAST_CAPS[k], hdr(f"AUTO_MV_FAST_{n}_CAP"), k)
        self.assertEqual((auto.MV_FAST_SETTLE_MIN_MS, auto.MV_FAST_DWELL_MIN_MS, auto.MV_FAST_T_TRACK_S,
                          auto.MV_FAST_PAUSE_V, auto.MOVE_BRAKE_ACC, auto.MOVE_SETTLE_MS, auto.YAW_DWELL_MS,
                          auto.BOB_ACC, auto.FROZEN_REF, auto.MV_SPEED_PCT[0], auto.MV_SPEED_PCT[1],
                          auto.MV_SPEED_DEFAULT),
                         (hdr("AUTO_MV_FAST_SETTLE_MIN_MS"), hdr("AUTO_MV_FAST_DWELL_MIN_MS"),
                          hdr("AUTO_MV_FAST_T_TRACK_S"), hdr("AUTO_MV_FAST_PAUSE_V"), hdr("AUTO_MV_BRAKE_ACC"),
                          hdr("AUTO_MV_SETTLE_MAX_MS"), hdr("AUTO_MV_YAW_DWELL_MS"), hdr("AUTO_MV_BOB_ACC"),
                          hdr("AUTO_MV_FROZEN_REF"), hdr("AUTO_MV_SPEED_MIN_PCT"), hdr("AUTO_MV_SPEED_MAX_PCT"),
                          hdr("AUTO_MV_SPEED_DEFAULT_PCT")))
        # (after the 300 % forensics of 2026-10-17: A_H at most 0.20, the settle at least 2500 ms)
        table = {100: (0.30, 0.30, 0.20, 0.10, 0.12, 0.15, 0.15, 3000, 500, 30, 60, 0.20, 0.10),
                 150: (0.40, 0.40, 0.30, 0.15, 0.14, 0.17, 0.225, 2500, 333, 35, 70, 0.20, 0.168),
                 200: (0.40, 0.40, 0.40, 0.20, 0.14, 0.17, 0.30, 2500, 300, 35, 70, 0.20, 0.168),
                 250: (0.40, 0.40, 0.50, 0.20, 0.14, 0.17, 0.375, 2500, 300, 35, 70, 0.20, 0.168),
                 300: (0.40, 0.40, 0.50, 0.20, 0.14, 0.17, 0.40, 2500, 300, 35, 70, 0.20, 0.168)}
        keys = ("v_h", "v_back", "v_fwd", "a_h", "w_lat", "w_fwd", "brake", "settle_ms", "dwell_ms", "yaw_rate",
                "yaw_acc", "bob_acc", "frozen_ref")
        for pct, want in table.items():
            lim = auto.mv_limits(pct)
            for k, w in zip(keys, want):
                self.assertAlmostEqual(lim[k], w, places=5, msg=(pct, k))
            self.assertEqual(lim["fast"], pct > 100)
        self.assertEqual(auto.mv_limits(400), auto.mv_limits(300))
        self.assertEqual(auto.mv_limits(50), auto.mv_limits(100))
        self.assertEqual((auto.settle_s(), auto.settle_s(200), auto.settle_s(300)), (3.0, 2.5, 2.5))
        self.assertAlmostEqual(auto.bob_cycle_s(), 1.789, places=3)
        self.assertAlmostEqual(auto.bob_cycle_s(200), 1.789, places=3)   # the bob as at 100 (review of cbccec5)
        self.assertAlmostEqual(auto.yaw_profile_s(90), 3.5, places=3)
        self.assertAlmostEqual(auto.yaw_profile_s(90, 200), 3.071, places=3)   # 35 deg/s, 70 deg/s^2
        lim, name = auto.speed_help(200)
        self.assertEqual((lim["v_h"], name), (0.40, "mvspeed 200 %"))

    def test_profile_mirror_golden(self):
        """move_profile against every row the drone's autoMotionMove planned (1e-4 s, 1e-4 m/s)."""
        with open(MVSPEED_GOLDEN_PATH) as f:
            rows = json.load(f)["rows"]
        self.assertGreater(len(rows), 400)
        worst = 0.0
        for pct, r, fw, u, d, t, vpk in rows:
            tt, vv = auto.move_profile(r, fw, u, d, pct)
            worst = max(worst, abs(tt - t), abs(vv - vpk))
            self.assertAlmostEqual(tt, t, delta=1e-4, msg=(pct, r, fw, u, d))
            self.assertAlmostEqual(vv, vpk, delta=1e-4, msg=(pct, r, fw, u, d))
        self.assertLess(worst, 1e-4)
        # the design's table (docs/autonomy-a2b.md 31): 1 m forward at D 3, 3 m at D 5, 0.6 m sideways at D 2.5
        # (250 and 300 as 200 since the 300 % forensics: A_H at most 0.20; 4.12 / 9.22 / 3.11, 3.89 / 9.14 / 2.88)
        for pct, f1, f3, side in ((100, 7.0, 17.0, 4.90), (150, 5.33, 12.0, 4.00), (200, 4.50, 9.5, 3.46),
                                  (250, 4.50, 9.5, 3.46), (300, 4.50, 9.5, 3.46)):   # W_FWD 0.17
            self.assertAlmostEqual(auto.move_profile_s(0.0, 1.0, 0.0, 3.0, pct), f1, delta=0.01)
            self.assertAlmostEqual(auto.move_profile_s(0.0, 3.0, 0.0, 5.0, pct), f3, delta=0.06)
            self.assertAlmostEqual(auto.move_profile_s(0.6, 0.0, 0.0, 2.5, pct), side, delta=0.01)
        # at 100 the old formula (a direct copy of the function before the speed step)
        def old(right, fwd, up, d):
            def trap(dist, v, a):
                if dist <= 0 or v <= 0:
                    return 0.0
                t_a = v / a
                if a * t_a * t_a >= dist:
                    return 2.0 * math.sqrt(dist / a)
                return 2.0 * t_a + (dist - v * t_a) / v
            length = math.hypot(right, fwd)
            v = 0.30
            if length > 0:
                d_end = max(d - max(fwd, 0.0), 0.05) if d else 99.0
                if right:
                    v = min(v, 0.12 * d_end * length / abs(right))
                if fwd:
                    v = min(v, 0.15 * d_end * length / abs(fwd))
                if fwd > 0:
                    v = min(v, 0.20 * length / fwd)
            return max(trap(length, v, 0.10), trap(abs(up), 0.15, 0.20))
        for r, fw, u, d in ((0.5, 1.2, 0.0, 3.0), (-1.5, 0.0, 0.0, 4.0), (0.0, -1.0, 0.2, 2.0), (0.7, 0.7, 0.0, None),
                            (0.0, 3.0, 0.0, 6.0), (0.3, 0.0, -0.3, 1.2)):
            self.assertEqual(auto.move_profile_s(r, fw, u, d), old(r, fw, u, d))

    @mock.patch.object(auto, "PERSON_MIN_M", 1.0)   # numbers worked out at the 1.0 m floor (0.5 m since 2026-10-17)
    def test_person_speed_term(self):
        """Above 100 % the person rule adds PERSON_SPEED_T x the forward peak to the overshoot (0 at 100)."""
        self.assertEqual(auto.person_speed_term(4.0, 0.0, 1.0, 100), 0.0)
        t200 = auto.person_speed_term(4.0, 0.0, 1.5, 200)
        self.assertAlmostEqual(t200, 0.40 * auto.move_profile(0.0, 1.5, 0.0, 4.0, 200)[1], places=9)
        self.assertGreater(t200, 0.10)
        low100, over100 = auto.person_gap_low(4.0, 0.0, 1.5, 1.6, 4.0)
        low200, over200 = auto.person_gap_low(4.0, 0.0, 1.5, 1.6, 4.0, 200)
        self.assertAlmostEqual(over200 - over100, t200, places=9)
        self.assertAlmostEqual(low100 - low200, t200, places=9)
        self.assertLess(auto.person_fwd_max(4.0, 0.0, 1.5, 1.6, 4.0, 200), auto.person_fwd_max(4.0, 0.0, 1.5, 1.6, 4.0))


MVSPEED_GOLDEN_4DEA542_PATH = os.path.join(HERE, "mvspeed_golden_4dea542.json")
REAL300 = {"mvacc": 30, "mvsettle": 1000}


def write_auto_get(values):
    """The 'espfly_diag.py auto get' cache the Mac reads (the sandbox's DEFAULT_DIR): {key: int}."""
    os.makedirs(auto.DEFAULT_DIR, exist_ok=True)
    with open(auto.vmin_cache_path(), "w") as f:
        json.dump({"utc": "2026-10-17T18:00:00Z",
                   "values": {k: {"value": str(v), "src": "nvs"} for k, v in values.items()}}, f)


class FastCapsMirrorTest(SandboxedTest):
    """'Real 300 %' (user request 2026-10-17, docs/autonomy-a2b.md 35): the drone's NVS mvacc / mvsettle (the A_H cap
    and the settle floor above 100) mirrored in mv_limits / move_profile / settle_s / the person rule; their defaults
    the Mac of d3bedee bit for bit, mvacc 30 / mvsettle 1000 4dea542's profiles (its golden, written by its
    test_auto_motion.c), the ranges and the table against the firmware's sources, fast_caps() from the cache."""

    def test_constants_match_the_firmware(self):
        with open(os.path.join(REPO, "components", "core", "crazyflie", "utils", "interface", "auto_motion.h")) as fh:
            hdr_src = fh.read()
        with open(os.path.join(REPO, "components", "core", "crazyflie", "utils", "src", "auto_glue_core.c")) as fh:
            glue = fh.read()

        def hdr(name):
            return int(re.search(r"#define %s\s+([0-9]+)\b" % name, hdr_src).group(1))
        self.assertEqual((auto.MV_ACC_CM, auto.MV_ACC_DEFAULT_CM, auto.MV_SETTLE_FLOOR_MS),
                         ((hdr("AUTO_MV_ACC_MIN_CM"), hdr("AUTO_MV_ACC_MAX_CM")), hdr("AUTO_MV_ACC_DEFAULT_CM"),
                          (hdr("AUTO_MV_SETTLE_FLOOR_MIN_MS"), hdr("AUTO_MV_SETTLE_FLOOR_MAX_MS"))))
        self.assertEqual(auto.MV_ACC_DEFAULT_CM / 100.0, auto.MV_FAST_CAPS["a_h"])
        self.assertEqual(auto.MV_FAST_DEFAULT["mvsettle"], auto.MV_FAST_SETTLE_MIN_MS)
        self.assertRegex(glue, r'\[AUTO_SET_MVACC\] = \{"mvacc", AUTO_NVS_U8, AUTO_MV_ACC_MIN_CM, AUTO_MV_ACC_MAX_CM, '
                               r'AUTO_MV_ACC_DEFAULT_CM,')
        self.assertRegex(glue, r'\[AUTO_SET_MVSETTLE\] = \{"mvsettle", AUTO_NVS_U16, AUTO_MV_SETTLE_FLOOR_MIN_MS, '
                               r'AUTO_MV_SETTLE_FLOOR_MAX_MS,\s+AUTO_MV_FAST_SETTLE_MIN_MS,')

    def test_defaults_are_the_mirror_of_before(self):
        """No caps, the defaults given, or the cache's defaults: the same limits at every step (the golden of d3bedee
        is test_profile_mirror_golden's), and 100 never reads them."""
        for pct in (100, 120, 150, 175, 200, 250, 300):
            base = auto.mv_limits(pct)
            self.assertEqual(auto.mv_limits(pct, auto.MV_FAST_DEFAULT), base)
            self.assertEqual(auto.mv_limits(pct, {"mvacc": 20, "mvsettle": 2500}), base)
        self.assertEqual(auto.mv_limits(100, REAL300), auto.mv_limits(100))
        self.assertEqual(auto.mv_limits(100, {"mvacc": 40, "mvsettle": 3000}), auto.mv_limits(100))
        with open(MVSPEED_GOLDEN_PATH) as f:
            rows = json.load(f)["rows"]
        for pct, r, fw, u, d, t, vpk in rows:
            tt, vv = auto.move_profile(r, fw, u, d, pct, auto.MV_FAST_DEFAULT)
            self.assertAlmostEqual(tt, t, delta=1e-4, msg=(pct, r, fw, u, d))
            self.assertAlmostEqual(vv, vpk, delta=1e-4, msg=(pct, r, fw, u, d))

    def test_real_300_is_4dea542(self):
        """mvacc 30 / mvsettle 1000: 4dea542's limits (A_H 0.15 / 0.20 / 0.25 / 0.30, the settle 2000 / 1500 / 1200 /
        1000 ms) and every profile of its golden (its 300 % rows the flights' of 2026-10-17 morning)."""
        for pct, a_h, settle in ((150, 0.15, 2000), (200, 0.20, 1500), (250, 0.25, 1200), (300, 0.30, 1000)):
            lim, base = auto.mv_limits(pct, REAL300), auto.mv_limits(pct)
            self.assertAlmostEqual(lim["a_h"], a_h, places=6)
            self.assertEqual(lim["settle_ms"], settle)
            self.assertEqual({k: v for k, v in lim.items() if k not in ("a_h", "settle_ms")},
                             {k: v for k, v in base.items() if k not in ("a_h", "settle_ms")})
        self.assertEqual((auto.settle_s(300, REAL300), auto.settle_s(300)), (1.0, 2.5))
        with open(MVSPEED_GOLDEN_4DEA542_PATH) as f:
            rows = json.load(f)["rows"]
        self.assertGreater(len(rows), 400)
        n300 = differ = 0
        for pct, r, fw, u, d, t, vpk in rows:
            tt, vv = auto.move_profile(r, fw, u, d, pct, REAL300)
            self.assertAlmostEqual(tt, t, delta=1e-4, msg=(pct, r, fw, u, d))
            self.assertAlmostEqual(vv, vpk, delta=1e-4, msg=(pct, r, fw, u, d))
            n300 += pct == 300
            differ += abs(auto.move_profile(r, fw, u, d, pct)[0] - t) > 1e-4
        self.assertGreater(n300, 70)
        self.assertGreater(differ, 50)   # the defaults plan 250 / 300 as 200
        # the design table's 300 % times (docs/autonomy-a2b.md 31) as flown on 2026-10-17 morning
        self.assertAlmostEqual(auto.move_profile_s(0.0, 1.0, 0.0, 3.0, 300, REAL300), 3.894, delta=0.01)
        self.assertAlmostEqual(auto.move_profile_s(0.6, 0.0, 0.0, 2.5, 300, REAL300), 2.881, delta=0.01)
        # above 30 nothing changes (A_H x s is 0.30 at 300); out of range clamped as the drone's setter
        l40, l30 = auto.mv_limits(300, {"mvacc": 40, "mvsettle": 1000}), auto.mv_limits(300, REAL300)
        self.assertAlmostEqual(l40.pop("a_h"), l30.pop("a_h"), places=12)   # (0.1 x s against 0.30: a double's ulp)
        self.assertEqual(l40, l30)
        self.assertEqual(auto.mv_limits(300, {"mvacc": 5, "mvsettle": 9000}),
                         auto.mv_limits(300, {"mvacc": 10, "mvsettle": 3000}))
        # the person rule's speed term follows the faster peak (a short approach: the acceleration sets the peak)
        self.assertGreater(auto.person_speed_term(4.0, 0.0, 0.3, 300, REAL300),
                           auto.person_speed_term(4.0, 0.0, 0.3, 300) + 0.02)

    def test_fast_caps_from_the_cache(self):
        self.assertEqual(auto.fast_caps(), {"mvacc": 20, "mvsettle": 2500, "mvwmax": 15, "src": "default"})
        write_auto_get({"mvspeed": 300, "mvacc": 30, "mvsettle": 1000})
        self.assertEqual(auto.fast_caps(), {"mvacc": 30, "mvsettle": 1000, "mvwmax": 15, "src": "auto get"})
        lim, name = auto.speed_help()
        self.assertEqual((name, lim["settle_ms"]), ("mvspeed 300 %", 1000))
        self.assertAlmostEqual(lim["a_h"], 0.30, places=6)
        write_auto_get({"mvacc": 41, "mvsettle": 1000})       # out of range (the drone refuses it): the default
        self.assertEqual(auto.fast_caps(), {"mvacc": 20, "mvsettle": 1000, "mvwmax": 15, "src": "partly default"})
        write_auto_get({"mvfix": 59})                          # an older drone: the caps of d3bedee
        self.assertEqual(auto.fast_caps()["src"], "default")
        a = auto.parse_auto_status(auto.auto_status_payload({"state": auto.S_HOVER, "mvSpeedPct": 300}))
        write_auto_get(REAL300)
        v = auto.Daemon.motion_view(a)
        self.assertEqual((v["mvspeed_pct"], v["mvacc_cm_s2"], v["mvsettle_ms"], v["fast_caps_src"]),
                         (300, 30, 1000, "auto get"))
        a100 = auto.parse_auto_status(auto.auto_status_payload({"state": auto.S_HOVER, "mvSpeedPct": 100}))
        self.assertNotIn("mvacc_cm_s2", auto.Daemon.motion_view(a100))   # not read at 100


class DaemonFastMoveTest(DaemonCase):
    """Fast moves on the Mac (2026-10-16, docs/autonomy-a2b.md 31): with the drone at mvspeed > 100 a MOVE / YAW
    replies when its profile ends (settling: true; --photo in the same reply), a command sent meanwhile waits for the
    end and is checked again on the fresh status (refused after an end other than DONE / SHORT / a reached PROGRESS,
    after the limit, or by a STOP), --full-wait waits as before; the battery fit counts the step's settle; an older
    136-byte drone is unchanged."""

    def up(self, *extra, **kw):
        d = self.drone(**dict(IHFAST, **kw))
        self.start_daemon(d)
        self.sortie_to_wait("--height", "100", "--photos", "1", "--wait-s", "60", *extra)
        return d

    def test_early_return_with_photo(self):
        d = self.up(mv_pct=200, mv_settle=1.0, scene_d=4.0)
        code, out, _, err = self.cli("move", "--forward", "0.5", "--photo")
        self.assertEqual((code, out["ok"], out.get("settling"), out["mvspeed_pct"]), (0, True, True, 200), (out, err))
        self.assertEqual((out["mvState"], out["mv"]["state"]), ("SETTLE", "SETTLE"))
        self.assertTrue(out["photo"]["ok"], out)
        self.assertAlmostEqual(out["profile_s"], round(auto.move_profile_s(0.0, 0.5, 0.0, 4.0, 200), 1))
        self.assertEqual(d.mv_state, auto.MV_SETTLE)
        self.assertEqual(len(self.events("motion_early")), 1)
        # a YAW: replies in its re-track bob (SETTLE with the bob flag), its profile and dwell at 200
        self.assertTrue(wait_for(lambda: d.mv_state == auto.MV_DONE, 3))
        code, out, _, err = self.cli("yaw", "--deg", "90", "--photo")
        self.assertEqual((code, out["ok"], out.get("settling")), (0, True, True), (out, err))
        self.assertIn("bob", out["mv"]["flags"])
        self.assertEqual((out["profile_s"], out["dwell_s"]), (round(auto.yaw_profile_s(90, 200), 2), 0.3))
        self.assertTrue(out["photo"]["ok"], out)

    def test_early_reply_is_pre_settle_and_the_next_reports_the_end(self):
        """2026-10-17 (the 300 % forensics): the early reply's travel is labelled pre-settle (no travelled_m), with its
        along / off; the next command, which waits behind it, reports its final end under 'previous'."""
        self.up(mv_pct=200, mv_settle=1.0, scene_d=4.0)
        code, out, _, err = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["ok"], out["settling"]), (0, True, True), (out, err))
        self.assertNotIn("travelled_m", out)
        self.assertEqual(out["travelled_pre_settle_m"]["fwd"], 0.5)
        self.assertIn("before its settle", out["note"])
        self.assertAlmostEqual(out["along"], 1.0, places=3)
        seq = out["mvSeq"]
        code, out2, _, err = self.cli("move", "--right", "0.3", "--full-wait")
        self.assertEqual((code, out2["ok"], out2["end"]), (0, True, "DONE"), (out2, err))
        self.assertEqual((out2["previous"]["mvSeq"], out2["previous"]["end"], out2["previous"]["travelled_m"]["fwd"]),
                         (seq, "DONE", 0.5), out2)
        code, out3, _, err = self.cli("move", "--right", "0.1", "--full-wait")
        self.assertNotIn("previous", out3)   # (once)

    def test_set_height_behind_an_early_move_reports_its_end(self):
        """The review of 2026-10-17: a set-height that waited behind an early-returned MOVE reports that move's final
        end under 'previous' (it went stale and came with the next MOVE, two commands late); the next MOVE has none."""
        self.up(mv_pct=200, mv_settle=1.0, scene_d=4.0)
        code, out, _, err = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["settling"]), (0, True), (out, err))
        seq = out["mvSeq"]
        code, out2, _, err = self.cli("set-height", "1.00", "--now")
        self.assertEqual((out2["previous"]["mvSeq"], out2["previous"]["end"]), (seq, "DONE"), (out2, err))
        code, out3, _, err = self.cli("move", "--right", "0.1", "--full-wait")
        self.assertNotIn("previous", out3, out3)

    def test_full_wait(self):
        d = self.up(mv_pct=200, mv_settle=0.6, scene_d=4.0)
        code, out, _, err = self.cli("move", "--forward", "0.5", "--full-wait")
        self.assertEqual((code, out["ok"], out["end"], "settling" in out), (0, True, "DONE", False), (out, err))
        self.assertEqual(d.mv_state, auto.MV_DONE)

    def test_waits_behind_then_goes(self):
        d = self.up(mv_pct=200, mv_settle=1.0, scene_d=4.0)
        code, out, _, _ = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["settling"]), (0, True), out)
        t_reply = time.monotonic()
        code, out2, _, err = self.cli("move", "--right", "0.3", "--full-wait")
        self.assertEqual((code, out2["ok"], out2["end"]), (0, True, "DONE"), (out2, err))
        moves = d.got(auto.T_MOVE)
        self.assertEqual(len(moves), 2)
        self.assertGreater(moves[1][0] - t_reply, 0.6)   # sent only after the first one's settle (1.0 s) ended
        self.assertEqual([e["type"] for e in self.events("motion_waits")], ["MOVE"])
        self.assertEqual(self.events("motion_released")[0]["end"], "DONE")
        # a SET_HEIGHT and a YAW wait the same way
        code, out, _, _ = self.cli("move", "--forward", "0.3")
        self.assertTrue(out["settling"], out)
        code, out, _, err = self.cli("yaw", "--deg", "-30", "--full-wait")
        self.assertEqual((code, out["ok"]), (0, True), (out, err))
        self.assertEqual(len(self.events("motion_waits")), 2)

    def test_refused_after_a_bad_end(self):
        d = self.up(mv_pct=200, mv_settle=0.8, scene_d=4.0, mv_end=("CONTACT", 1.0))
        code, out, _, _ = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["settling"]), (0, True), out)
        code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual((code, out["ok"], out["previous_end"]), (1, False, "CONTACT"), out)
        self.assertIn("ended CONTACT after your photo", out["error"])
        self.assertIn("take a new photo", out["error"])
        self.assertEqual(len(d.got(auto.T_MOVE)), 1)

    def test_refused_after_an_undone_turn(self):
        d = self.up(mv_pct=200, mv_settle=0.8, scene_d=4.0, yaw_end="UNDONE")
        code, out, _, _ = self.cli("yaw", "--deg", "60")
        self.assertEqual((code, out["settling"]), (0, True), out)
        code, out, _, _ = self.cli("move", "--forward", "0.3")
        self.assertEqual((code, out["ok"], out["previous_end"]), (1, False, "UNDONE"), out)
        self.assertEqual(d.got(auto.T_MOVE), [])

    def test_refused_on_timeout(self):
        d = self.up(mv_pct=200, mv_settle=20.0, scene_d=4.0)
        code, out, _, _ = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["settling"]), (0, True), out)
        with mock.patch.object(auto, "BEHIND_EXTRA_S", -8.0):   # the limit 1.5 + 4 x 1.789 - 8 = 0.66 s
            t0 = time.monotonic()
            code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("has not ended within", out["error"])
        self.assertLess(time.monotonic() - t0, 3.0)
        self.assertEqual(len(d.got(auto.T_MOVE)), 1)

    def test_stop_withdraws_a_waiting_command(self):
        d = self.up(mv_pct=200, mv_settle=20.0, scene_d=4.0)
        code, out, _, _ = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["settling"]), (0, True), out)
        got = {}

        def waiting():
            got["r"] = auto.daemon_request(self.sock, {"op": "move", "right": 0.3}, 30.0)
        th = threading.Thread(target=waiting)
        th.start()
        self.assertTrue(wait_for(lambda: len(self.events("motion_waits")) == 1, 3))
        code, out, _, _ = self.cli("stop")
        self.assertEqual(code, 0, out)
        th.join(5)
        self.assertEqual((got["r"]["ok"], got["r"]["result"]), (False, "REFUSED"), got)
        self.assertIn("withdrawn: STOP", got["r"]["error"])
        self.assertEqual(len(d.got(auto.T_MOVE)), 1)

    def test_waiting_client_gone_and_the_next_sortie(self):
        """Review of cbccec5: a waiting command whose client went away leaves no entry behind; the early-returned
        motion is forgotten when the sortie ends (the drone's mvSeq restarts at ARM)."""
        d = self.up(mv_pct=200, mv_settle=20.0, scene_d=4.0)
        code, out, _, _ = self.cli("move", "--forward", "0.5")
        self.assertEqual((code, out["settling"]), (0, True), out)
        self.assertIsNotNone(self.dm.early_seq)
        with self.assertRaises(auto.DaemonError):
            auto.daemon_request(self.sock, {"op": "move", "right": 0.3}, 0.5)   # the client gives up after 0.5 s
        self.assertTrue(wait_for(lambda: self.dm.behind == [], 3), self.dm.behind)
        self.assertEqual(len(d.got(auto.T_MOVE)), 1)
        code, out, _, _ = self.cli("land")
        self.assertEqual(code, 0, out)
        self.assertTrue(wait_for(lambda: self.dm.auto is not None and self.dm.auto["state"] == auto.S_IDLE, 10))
        self.assertIsNone(self.dm.early_seq)

    def test_battery_fit_uses_the_step_settle(self):
        f = 3.0
        need = auto.move_profile_s(0.0, f, 0.0, 6.0, 200) + auto.settle_s(200)
        mv = int(round((auto.BATT_END_V + auto.BATT_DUR_VPS * (need - 1.0)) * 1000))   # 1 s short of it
        d = self.up(mv_pct=200, mv_settle=0.5, scene_d=6.0, vbat_mv=mv)
        code, out, _, _ = self.cli("move", "--forward", str(f))
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("2.5 s settle", out["error"])
        self.assertEqual(d.got(auto.T_MOVE), [])
        # at 100 the same voltage leaves less: still refused, with the 3 s settle in its message
        self.assertGreater(auto.move_profile_s(0.0, f, 0.0, 6.0) + auto.settle_s(), need)

    def test_real_300_battery_fit_and_status(self):
        """'Real 300 %' (2026-10-17): with the cached mvacc 30 / mvsettle 1000 the battery fit counts 4dea542's profile
        and the 1.0 s settle (a voltage the defaults' 2.5 s settle and slower profile would refuse it at), the status
        and the early reply's note show them."""
        f = 3.0
        need = auto.move_profile_s(0.0, f, 0.0, 6.0, 300, REAL300) + auto.settle_s(300, REAL300)
        need_def = auto.move_profile_s(0.0, f, 0.0, 6.0, 300) + auto.settle_s(300)
        self.assertGreater(need_def, need + 1.5)
        mv = int(round((auto.BATT_END_V + auto.BATT_DUR_VPS * (need + 0.8)) * 1000))   # 0.8 s to spare
        write_auto_get({"mvspeed": 300, **REAL300})
        self.up(mv_pct=300, mv_settle=0.5, scene_d=6.0, vbat_mv=mv)
        code, out, _, err = self.cli("move", "--forward", str(f))
        self.assertEqual((code, out["ok"], out["settling"]), (0, True, True), (out, err))
        self.assertIn("settles at most 1 s", out["note"])
        self.assertAlmostEqual(out["profile_s"], round(auto.move_profile_s(0.0, f, 0.0, 6.0, 300, REAL300), 1))
        self.assertEqual((out["mvacc_cm_s2"], out["mvsettle_ms"], out["fast_caps_src"]), (30, 1000, "auto get"))

    @mock.patch.object(auto, "PERSON_MIN_M", 1.0)   # numbers worked out at the 1.0 m floor (0.5 m since 2026-10-17)
    def test_person_rule_speed_term(self):
        d = self.up(mv_pct=200, mv_settle=0.5, scene_d=4.0)
        code, out, _, _ = self.cli("move", "--forward", "1.5", "--person-gap", "1.6", "--est", "3.1")
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("the speed term at mvspeed 200", out["error"])
        self.assertEqual(out["fwd_max_m"], auto.person_fwd_max(4.0, 0.0, 1.5, 1.6, 3.1, 200))
        self.assertEqual(d.got(auto.T_MOVE), [])

    def test_old_drone_unchanged(self):
        """A 136-byte drone (no speed step): 100 %, no early return, a second command while one runs refused as
        before."""
        d = self.up(hr=True, old136=True, mv_settle=1.5, scene_d=4.0)
        code, out, _, _ = self.cli("move", "--forward", "0.5", "--no-wait")
        self.assertEqual((code, out["ok"], out["mvspeed_pct"]), (0, True, 100), out)
        self.assertTrue(wait_for(lambda: self.dm.auto["mvActive"], 2))
        code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual((code, out["ok"]), (1, False), out)
        self.assertIn("is running", out["error"])
        self.assertEqual(self.events("motion_waits"), [])
        self.assertTrue(wait_for(lambda: d.mv_state == auto.MV_DONE, 5))
        code, out, _, _ = self.cli("move", "--right", "0.3")
        self.assertEqual((code, out["ok"], out["end"], "settling" in out), (0, True, "DONE", False), out)

if __name__ == "__main__":
    unittest.main()
