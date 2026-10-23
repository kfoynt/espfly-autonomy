#!/usr/bin/env python3
"""Golden packets for the autonomy link (docs/autonomy-link.md): stage A1 in
auto_proto_golden.json, stage A2b (flight commands, AUTO_STATUS) in
auto_proto_golden_a2.json, the interactive hover (MOVE, YAW, STOP, the 96-byte
AUTO_STATUS, the TAKEOFF with a planned move), hold and report (the 136-byte
AUTO_STATUS, 2026-10-15), the speed step (the 140-byte AUTO_STATUS, 2026-10-16), SET_HEIGHT (0x33, 2026-10-15; its 6-byte form 2026-10-16) and a colour photo's
44-byte META (2026-10-15) in auto_proto_golden_ih.json.

Built with Python's hmac/struct only, independently of the C codec and the Mac
tool; tools/tests/test_auto_proto.c and tools/tests/test_espfly_auto.py must
both reproduce them byte for byte.
"""
import hashlib
import hmac
import json
import os
import struct
import zlib

KEY = bytes(range(16))


def packet(ptype, flags, session, seq, payload):
    hdr = struct.pack("<BBBBIIHH", 0xE5, 1, ptype, flags, session, seq, len(payload), 0)
    tag = hmac.new(KEY, hdr + payload, hashlib.sha256).digest()[:16]
    return hdr + payload + tag


def main():
    frame = bytes((x * 7 + y * 3) & 0xFF for y in range(120) for x in range(160))
    chunk0 = frame[:1024]
    cases = [
        ("hello", packet(0x01, 0, 0, 1, struct.pack("<I", 0x11223344))),
        ("hello_ack", packet(0x81, 0, 0xA1B2C3D4, 1,
                             struct.pack("<III4s24s", 0xA1B2C3D4, 0x0BADF00D, 0x11223344,
                                         hashlib.sha256(KEY).digest()[:4], b"f7e67fb"))),
        ("photo", packet(0x20, 0x01, 0xA1B2C3D4, 2, struct.pack("<BBH", 2, 0, 300))),
        ("ack", packet(0x82, 0, 0xA1B2C3D4, 7, struct.pack("<IBBBBI", 2, 0x20, 0, 0, 0, 5))),
        ("meta", packet(0xA0, 0, 0xA1B2C3D4, 8,
                        struct.pack("<HBBHHIIIIhhhHBBHHH", 5, 0, 2, 160, 120, len(frame), zlib.crc32(frame),
                                    4321, 98765, -150, 220, 9000, 0, 97, 31, 1024, 19, 3870))),
        ("chunk", packet(0xA1, 0, 0xA1B2C3D4, 9, struct.pack("<HBBHH", 5, 0, 0, 0, len(chunk0)) + chunk0)),
        ("chunks", packet(0x21, 0x03, 0xA1B2C3D4, 3, struct.pack("<HBBH", 5, 0, 4, 12))),
        ("hb", packet(0x02, 0, 0xA1B2C3D4, 4, struct.pack("<I", 123456))),
    ]
    out = {
        "key": KEY.hex(),
        "key_fp": hashlib.sha256(KEY).digest()[:4].hex(),
        "frame_crc32": zlib.crc32(frame),
        "frame_rule": "frame[y*160+x] = (x*7 + y*3) & 0xFF, 160x120",
        "packets": {name: pkt.hex() for name, pkt in cases},
    }
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "auto_proto_golden.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    print(f"wrote {path}")

    # Stage A2b (docs/autonomy-a2b.md section 3): a separate file, so the A1
    # file and the tests that list its cases exactly stay unchanged.
    a2 = [
        ("arm", packet(0x10, 0x01, 0xA1B2C3D4, 10, struct.pack("<BBH", 1, 0, 0))),
        ("arm_air", packet(0x10, 0x01, 0xA1B2C3D4, 11, struct.pack("<BBH", 0, 0, 3000))),
        ("takeoff", packet(0x11, 0x01, 0xA1B2C3D4, 12, struct.pack("<IHBBHH", 0x5EED0001, 40, 2, 0, 300, 0))),
        ("land", packet(0x12, 0x01, 0xA1B2C3D4, 13, b"")),
        ("disarm", packet(0x14, 0x01, 0xA1B2C3D4, 14, b"")),
        ("kill", packet(0x13, 0x01, 0xA1B2C3D4, 15, b"")),
        ("kill_any_session", packet(0x13, 0x01, 0, 0x7FFFFFFF, b"")),
        ("photo_hover", packet(0x20, 0x01, 0xA1B2C3D4, 16, struct.pack("<BBH", 1, 0, 150))),
        ("ack_arm", packet(0x82, 0, 0xA1B2C3D4, 20, struct.pack("<IBBBBI", 10, 0x10, 0, 1, 0, 0x5EED0001))),
        ("ack_guards", packet(0x82, 0, 0xA1B2C3D4, 21, struct.pack("<IBBBBI", 11, 0x10, 12, 0, 0, 0x0000040C))),
        ("auto_status", packet(0x84, 0, 0xA1B2C3D4, 22, struct.pack(
            "<IIII10B8H2h6H2BI",
            123456, 0x0BADF00D, 7, 0x00000004,          # upMs, bootNonce, transitions, failedGuards
            3, 0, 3, 0, 0, 4, 4, 0x8D, 0x40, 2,          # state HOVER, mode FLIGHT, reason HOLD, vzLand, landTrip,
                                                         # vzPhase HOLD, vzReason HOLD_REACHED, flags, trips REST, capture DONE
            5, 2100, 4300, 1050, 3950, 5000, 3800, 3420, # photoId, stateMs, airMs, stableMs, waitLeftMs, hoverWaitMs,
                                                         # vbatMv, vbatFastMinMv
            305, -12,                                    # hMm, vzMms
            47100, 35400, 46800, 0, 16500, 16620,        # thrust, tLo, tHold, tRef, kPrior/10, khat/10
            6, 0, 1))),                                  # lastCmdType PHOTO, lastCmdResult OK, killCount
    ]
    out2 = {
        "key": KEY.hex(),
        "auto_status_fmt": "<IIII10B8H2h6H2BI",
        "packets": {name: pkt.hex() for name, pkt in a2},
    }
    path2 = os.path.join(here, "auto_proto_golden_a2.json")
    with open(path2, "w") as f:
        json.dump(out2, f, indent=1)
        f.write("\n")
    print(f"wrote {path2}")

    # The interactive hover (docs/autonomy-a2b.md section 14): MOVE, YAW, STOP,
    # TAKEOFF with this sortie's wait, the 96-byte AUTO_STATUS. Its own file, so
    # the A1 and A2b files stay as they were (an A2b drone's 64-byte AUTO_STATUS
    # is still parsed).
    ih_fmt = "<IIII10B8H2h6H2BI6BH6hHhhhHH"
    ih = [
        ("move", packet(0x31, 0x01, 0xA1B2C3D4, 30, struct.pack("<hhhH", 50, -120, 20, 0))),
        ("move_left_down", packet(0x31, 0x01, 0xA1B2C3D4, 31, struct.pack("<hhhH", -150, 0, -50, 0))),
        ("yaw", packet(0x30, 0x01, 0xA1B2C3D4, 32, struct.pack("<hH", 90, 0))),
        ("yaw_left", packet(0x30, 0x01, 0xA1B2C3D4, 33, struct.pack("<hH", -180, 0))),
        ("stop", packet(0x32, 0x01, 0xA1B2C3D4, 34, b"")),
        ("takeoff_wait", packet(0x11, 0x01, 0xA1B2C3D4, 35, struct.pack("<IHBBHH", 0x5EED0002, 100, 1, 0, 300,
                                                                        60000))),
        # MOVE refused: CAM_UNAVAIL (5), state HOVER, detail = HOVER | NO_DIST << 8 | seq 3 << 16
        ("ack_move_refused", packet(0x82, 0, 0xA1B2C3D4, 36, struct.pack("<IBBBBI", 30, 0x31, 5, 3, 0,
                                                                          3 | 5 << 8 | 3 << 16))),
        ("auto_status_ih", packet(0x84, 0, 0xA1B2C3D4, 37, struct.pack(
            ih_fmt,
            654321, 0x0BADF00D, 9, 0,                     # upMs, bootNonce, transitions, failedGuards
            3, 0, 3, 0, 0, 4, 4, 0x8D, 0x40, 2,          # HOVER, FLIGHT, HOLD, .., vzPhase HOLD, flags, trips, capture
            6, 30000, 41200, 20000, 52000, 60000, 3460, 3390,   # photoId, stateMs, airMs, stableMs, waitLeftMs,
                                                                # hoverWaitMs, vbatMv, vbatFastMinMv
            1012, 8,                                     # hMm, vzMms
            46900, 35100, 46700, 0, 16500, 16710,        # thrust, tLo, tHold, tRef, kPrior/10, khat/10
            7, 0, 0,                                     # lastCmdType MOVE, OK, killCount
            1, 1, 0, 0, 0x01, 0,                         # mvState RUN, kind MOVE, why, end, flags D_VALID, battLow
            3,                                           # mvSeq
            21, -48, 0, 50, -120, 0,                     # progress right/fwd/up cm, target right/fwd/up cm
            212,                                         # sceneDCm
            100, -9000, -90,                             # hTargetCm, yawOffCdeg (-90 deg), yawTgtDeg
            2400, 388))),                                # mvPhaseMs, airLeftDs (38.8 s)
        # Waits over 65.5 s (2026-10-14, docs/autonomy-a2b.md 18.2): TAKEOFF byte 7 (a pad until then) carries
        # actWaitMs bits 16-23, bytes 10-11 bits 0-15. 65536 = 0x01 / 0x0000, 120000 = 0x01 / 0xD4C0.
        ("takeoff_wait_65536", packet(0x11, 0x01, 0xA1B2C3D4, 38, struct.pack("<IHBBHH", 0x5EED0003, 105, 1,
                                                                              65536 >> 16, 300, 65536 & 0xFFFF))),
        ("takeoff_wait_120000", packet(0x11, 0x01, 0xA1B2C3D4, 39, struct.pack("<IHBBHH", 0x5EED0004, 105, 1,
                                                                               120000 >> 16, 300,
                                                                               120000 & 0xFFFF))),
        # A long sortie: airMs and the u16 waits saturated (65535 = 65.5 s or more), the air clock from airLeftDs
        # (150: 15.0 s left of 120 s, air_s 105.0)
        ("auto_status_ih120", packet(0x84, 0, 0xA1B2C3D4, 40, struct.pack(
            ih_fmt,
            765432, 0x0BADF00D, 11, 0,                    # upMs, bootNonce, transitions, failedGuards
            3, 0, 3, 0, 0, 4, 4, 0x8D, 0x40, 2,          # HOVER, FLIGHT, HOLD, .., vzPhase HOLD, flags, trips, capture
            7, 65535, 65535, 65535, 65535, 65535, 3410, 3350,   # photoId, stateMs, airMs, stableMs, waitLeftMs,
                                                                # hoverWaitMs (all saturated), vbatMv, vbatFastMinMv
            1050, -3,                                    # hMm, vzMms
            47300, 35100, 47200, 0, 16500, 16630,        # thrust, tLo, tHold, tRef, kPrior/10, khat/10
            9, 0, 0,                                     # lastCmdType STOP, OK, killCount
            4, 1, 0, 2, 0x01, 0,                         # mvState DONE, kind MOVE, why, end STOP_CMD, flags, battLow
            2,                                           # mvSeq
            0, 0, 0, 0, 0, 0,                            # progress, target
            230,                                         # sceneDCm
            105, 0, 0,                                   # hTargetCm, yawOffCdeg, yawTgtDeg
            65535, 150))),                               # mvPhaseMs (saturated), airLeftDs (15.0 s)
        # The take-off's planned move (2026-10-14, docs/autonomy-a2b.md 21.4): 18 bytes, the 12 of before then i16
        # planRightCm, i16 planFwdCm, u16 0 (reserved). Left 80, forward 250, a 90 s wait (byte 7 0x01, 0x5F90).
        ("takeoff_plan", packet(0x11, 0x01, 0xA1B2C3D4, 41, struct.pack("<IHBBHHhhH", 0x5EED0005, 100, 1,
                                                                        90000 >> 16, 300, 90000 & 0xFFFF,
                                                                        -80, 250, 0))),
    ]
    # Hold and report ("Claude decides", 2026-10-15, docs/autonomy-a2b.md 22): 40 bytes appended (136), the first
    # 96 as before. A drone holding after a contact, the camera height lost: CAM_LOST (detail no_G), before it
    # CONTACT (detail HOLD | motion stopped), VZ CONTACT... the SINK in 3.2 s.
    hr_fmt = ih_fmt + "BBHHBBHHHHhhHHHHhBBBBHBBH"
    hr_block = (1, 0x02, 7, 1800,                     # evCode CAM_LOST, detail no_G, evCount 7, evAgeMs
                0x01 | 0x80, 0x02, 1800, 3200,          # holdBits CAM_LOST | OPEN_LOOP, camBits no_G, camOffMs, sinkInMs
                231, 310, -205, 98,                     # peakAccCg 2.31 g, peakRateDps, contactAxCg, contactAyCg
                124, 412, 1030, 47120,                  # sinceCmdDs 12.4 s, airDs 41.2 s, khatPm 1.030, capQ
                -9000, 1, 0,                            # contactYawCdeg -90.00, contacts, loopGaps
                5, 0x84, 2300,                          # ev1: CONTACT, detail HOLD (4) | motion (0x80), age
                10, 8, 2300)                            # ev2: MOVE_STOPPED, detail CONTACT (8), age
    ih.append(("auto_status_hr", packet(0x84, 0, 0xA1B2C3D4, 42, struct.pack(
        hr_fmt,
        654321, 0x0BADF00D, 9, 0,
        3, 0, 3, 0, 0, 4, 4, 0x8D, 0x40, 2,
        6, 30000, 41200, 20000, 0, 60000, 3460, 3390,
        1012, 8,
        46900, 35100, 46700, 0, 16500, 16710,
        7, 0, 0,
        5, 1, 0, 8, 0x01, 0,                          # mvState STOPPED, MOVE, end CONTACT (8)
        3,
        21, -48, 0, 50, -120, 0,
        212,
        100, -9000, -90,
        2400, 0,                                      # mvPhaseMs, airLeftDs 0 (no air clock)
        *hr_block))))
    # The speed step (2026-10-16, docs/autonomy-a2b.md 31): 4 bytes appended (140), the first 136 as before: u16
    # mvSpeedPct, u16 reserved 0. The same drone as auto_status_hr, flying at mvspeed 200.
    ms_fmt = hr_fmt + "HH"
    ih.append(("auto_status_ms", packet(0x84, 0, 0xA1B2C3D4, 50, struct.pack(
        ms_fmt,
        654321, 0x0BADF00D, 9, 0,
        3, 0, 3, 0, 0, 4, 4, 0x8D, 0x40, 2,
        6, 30000, 41200, 20000, 0, 60000, 3460, 3390,
        1012, 8,
        46900, 35100, 46700, 0, 16500, 16710,
        7, 0, 0,
        5, 1, 0, 8, 0x01, 0,
        3,
        21, -48, 0, 50, -120, 0,
        212,
        100, -9000, -90,
        2400, 0,
        *hr_block,
        200, 0))))
    # SET_HEIGHT (0x33, 2026-10-15, docs/autonomy-a2b.md 23): i16 xCm (the judged true height), i16 atCm (the drone's
    # estimate at the judged photo; -32768: now). Judged 1.10 m on a photo taken at 0.95 m (d +0.15); the absolute form
    # 0.80 m. The ACK: mvDetail's layout with the motion seq replaced by the old estimate (cm, i16): accepted at 1.00 m
    # (HOVER | 0 << 8 | 100 << 16); refused RANGE SH_STEP (13) at 0.98 m.
    ih.append(("set_height", packet(0x33, 0x01, 0xA1B2C3D4, 43, struct.pack("<hh", 110, 95))))
    ih.append(("set_height_now", packet(0x33, 0x01, 0xA1B2C3D4, 44, struct.pack("<hh", 80, -32768))))
    ih.append(("ack_set_height", packet(0x82, 0, 0xA1B2C3D4, 45, struct.pack("<IBBBBI", 43, 0x33, 0, 3, 0,
                                                                             3 | 0 << 8 | 100 << 16))))
    ih.append(("ack_set_height_refused", packet(0x82, 0, 0xA1B2C3D4, 46, struct.pack("<IBBBBI", 44, 0x33, 3, 3, 0,
                                                                                     3 | 13 << 8 | 98 << 16))))
    # SET_HEIGHT "go to the target" (2026-10-16, docs/autonomy-a2b.md 27): the 6-byte form, i16 toCm appended
    # (-32768: the commanded height). Judged 1.10 m on a photo at 0.90 m, to the commanded height; judged 0.90 m now,
    # to 1.20 m.
    ih.append(("set_height_to", packet(0x33, 0x01, 0xA1B2C3D4, 48, struct.pack("<hhh", 110, 90, -32768))))
    ih.append(("set_height_to_120", packet(0x33, 0x01, 0xA1B2C3D4, 49, struct.pack("<hhh", 90, -32768, 120))))
    # Colour photos (2026-10-15, docs/autonomy-link.md "Photos"): a colour frame's META is 44 bytes, the 40 of A1 then
    # u8 pixFmt (1: YUYV) and 3 zero bytes. The frame: Y the A1 frame's rule, U (even x) 64 + y, V (odd x) 192 - y,
    # Y0 U Y1 V per pixel pair: 38400 bytes, 38 chunks.
    cframe = bytes(b for y in range(120) for x in range(160)
                   for b in ((x * 7 + y * 3) & 0xFF, 64 + y if x % 2 == 0 else 192 - y))
    ih.append(("meta_color", packet(0xA0, 0, 0xA1B2C3D4, 47,
                                    struct.pack("<HBBHHIIIIhhhHBBHHHB3x", 6, 0, 1, 160, 120, len(cframe),
                                                zlib.crc32(cframe), 4400, 99000, -150, 220, 9000, 0, 97, 31, 1024,
                                                38, 3870, 1))))
    out3 = {
        "key": KEY.hex(),
        "auto_status_fmt": ih_fmt,
        "auto_status_fmt_hr": hr_fmt,
        "auto_status_fmt_ms": ms_fmt,
        "frame_color_crc32": zlib.crc32(cframe),
        "frame_color_rule": "YUYV 160x120: byte[2*(y*160+x)] = (x*7 + y*3) & 0xFF, byte[2*(y*160+x)+1] = 64 + y "
                            "(x even, U) or 192 - y (x odd, V)",
        "packets": {name: pkt.hex() for name, pkt in ih},
    }
    path3 = os.path.join(here, "auto_proto_golden_ih.json")
    with open(path3, "w") as f:
        json.dump(out3, f, indent=1)
        f.write("\n")
    print(f"wrote {path3}")


if __name__ == "__main__":
    main()
