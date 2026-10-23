"""Host tests for tools/espfly_diag.py. Run: python3 -m unittest discover -s tools/tests
(or python3 tools/tests/test_espfly_diag.py). No serial port: the console is faked."""
import contextlib
import io
import json
import math
import os
import re
import sys
import tempfile
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import espfly_diag as diag  # noqa: E402


def make_line(tag, vals):
    body = tag + "," + ",".join(str(v) for v in vals)
    cs = 0
    for ch in body.encode():
        cs ^= ch
    return f"{body}*{cs:02X}"


class LevelOffsetTest(unittest.TestCase):
    def test_two_position_cancels_surface_tilt(self):
        offset = (0.031, -0.018)
        for tilt in [(0.0, 0.0), (0.012, -0.007), (-0.02, 0.015)]:
            # Turning 180 deg about the vertical negates the surface tilt in x and y.
            a = (offset[0] + tilt[0], offset[1] + tilt[1], 0.999)
            b = (offset[0] - tilt[0], offset[1] - tilt[1], 0.999)
            (ox, oy), (tx, ty) = diag.level_offset(a, b)
            self.assertAlmostEqual(ox, offset[0], places=9)
            self.assertAlmostEqual(oy, offset[1], places=9)
            self.assertAlmostEqual(tx, tilt[0], places=9)
            self.assertAlmostEqual(ty, tilt[1], places=9)

    def test_single_position(self):
        (ox, oy), tilt = diag.level_offset((0.02, -0.01, 1.0))
        self.assertEqual((ox, oy), (0.02, -0.01))
        self.assertEqual(tilt, (0.0, 0.0))

    def test_tilt_conventions_match_estimator(self):
        r, p = diag.tilt_deg(math.sin(math.radians(2.0)), 0.0, math.cos(math.radians(2.0)))
        self.assertAlmostEqual(p, 2.0, places=6)
        self.assertAlmostEqual(r, 0.0, places=6)
        r, p = diag.tilt_deg(0.0, math.sin(math.radians(-1.5)), math.cos(math.radians(-1.5)))
        self.assertAlmostEqual(r, -1.5, places=6)


class RecordParseTest(unittest.TestCase):
    VALS = [7, 123456, 3, 5, 30000, 0, -25, 110, -40, 9000, 12, -3, 0, 15, -8, 1001, 30000, 31000, 29000, 30500,
            25, -10, 400, -200,
            15, 3850, 42, 1, 120, 37, 25, 333, 12, -123, 45, 2, -1, 10, -20, 30, 3, -45, 120, 30000, 25, 0,
            312, -45, 1372, 803, 0x93, 21,
            3, 3, 4, 318, -12, 41500, 0x8B,
            0x11, 2500, -1823, 150, -9000,
            3912, -180, 600, -95, 410, -21, 14, 1, -334]

    def test_valid_record(self):
        r = diag.parse_record(make_line("R", self.VALS))
        self.assertIsNotNone(r)
        self.assertEqual(r["tickMs"], 123456)
        self.assertEqual(r["pitchRateI"], -200)
        self.assertEqual(r["flowGz_e4"], 30)
        self.assertEqual(len(self.VALS), len(diag.RECORD_FIELDS))
        self.assertEqual((r["cvH_mm"], r["cvV_mms"], r["cvBias_cmg"], r["cvGain_dpm"], r["cvFlags"], r["cvSigV_mms"]),
                         (312, -45, 1372, 803, 0x93, 21))
        self.assertEqual((r["autoState"], r["autoReason"], r["vzPhase"], r["vzH_mm"], r["vzV_mms"], r["vzThrust"],
                          r["autoFlags"]), (3, 3, 4, 318, -12, 41500, 0x8B))
        self.assertEqual((r["mvState"], r["mvRefLat_e4"], r["mvRefFwd_e4"], r["mvVRef_mms"], r["mvYawOff_cdeg"]),
                         (0x11, 2500, -1823, 150, -9000))
        self.assertEqual((r["imuTemp_cC"], r["gtcCorrX_mdps"], r["gtcCorrY_mdps"], r["mkiBiasX_mdps"],
                          r["mkiBiasY_mdps"], r["flowGain_e3"], r["flowSat_pct"], r["flowExpGate"]),
                         (3912, -180, 600, -95, 410, -21, 14, 1))
        self.assertEqual(r["atcCorrZ_cmg"], -334)

    def test_columns_match_the_firmware(self):
        """The A2b record layout (plan section 8 appends the autonomy columns), then the interactive hover's, then
        the thermal drift's and the exposure's (docs/autonomy-a2b.md 19.3)."""
        self.assertEqual(len(diag.RECORD_FIELDS), 73)
        self.assertEqual(diag.RECORD_FIELDS[-21:-14], ["autoState", "autoReason", "vzPhase", "vzH_mm", "vzV_mms",
                                                       "vzThrust", "autoFlags"])
        self.assertEqual(diag.RECORD_FIELDS[-14:-9], ["mvState", "mvRefLat_e4", "mvRefFwd_e4", "mvVRef_mms",
                                                      "mvYawOff_cdeg"])
        self.assertEqual(diag.RECORD_FIELDS[-9:], ["imuTemp_cC", "gtcCorrX_mdps", "gtcCorrY_mdps", "mkiBiasX_mdps",
                                                   "mkiBiasY_mdps", "flowGain_e3", "flowSat_pct", "flowExpGate",
                                                   "atcCorrZ_cmg"])
        self.assertEqual((len(diag.RECORD_FIELDS_V5), len(diag.RECORD_FIELDS_V4), len(diag.RECORD_FIELDS_V3),
                          len(diag.RECORD_FIELDS_V2), len(diag.RECORD_FIELDS_V1)), (72, 64, 59, 52, 46))

    def test_record_from_14bcfc6_firmware(self):
        """64 fields (the interactive hover, before the thermal-drift columns): those columns are None."""
        r = diag.parse_record(make_line("R", self.VALS[:64]))
        self.assertIsNotNone(r)
        self.assertEqual(r["mvYawOff_cdeg"], -9000)
        for k in diag.RECORD_FIELDS[64:]:
            self.assertIsNone(r[k], k)

    def test_record_from_19fe168_firmware(self):
        """72 fields (before the accelerometer compensation's column): atcCorrZ_cmg is None."""
        r = diag.parse_record(make_line("R", self.VALS[:72]))
        self.assertIsNotNone(r)
        self.assertEqual(r["flowExpGate"], 1)
        self.assertIsNone(r["atcCorrZ_cmg"])

    def test_old_and_new_csv(self):
        """A 112-byte firmware's CSV (64 columns) and a new one (72) read by header; the thermal summary needs the
        new columns."""
        for n in (64, 72):
            r = diag.parse_record(make_line("R", self.VALS[:n]))
            buf = io.StringIO()
            w = diag.csv.writer(buf)
            w.writerow(diag.RECORD_FIELDS[:n])
            w.writerow([r[k] for k in diag.RECORD_FIELDS[:n]])
            buf.seek(0)
            rows = diag.read_rows(buf)
            self.assertEqual(rows[0]["mvYawOff_cdeg"], -9000)
            if n == 64:
                self.assertNotIn("imuTemp_cC", rows[0])
                self.assertIsNone(diag.thermal_summary(rows))
            else:
                self.assertEqual(rows[0]["flowExpGate"], 1)

    def test_record_from_a2b_firmware(self):
        """59 fields (A2b before the interactive hover): the mv* columns are None."""
        r = diag.parse_record(make_line("R", self.VALS[:59]))
        self.assertIsNotNone(r)
        self.assertEqual(r["autoFlags"], 0x8B)
        for k in ("mvState", "mvRefLat_e4", "mvRefFwd_e4", "mvVRef_mms", "mvYawOff_cdeg"):
            self.assertIsNone(r[k], k)

    def test_columns_match_usb_diag_c(self):
        """espfly_diag parses records by position: its columns must be usb_diag.c's RECORD_FIELDS, in order.
        A firmware tree before A2b (52 columns) must match the 52-column layout; with the A2b recorder
        (59) the whole list."""
        path = os.path.join(os.path.dirname(__file__), "..", "..", "components", "core", "crazyflie", "modules",
                            "src", "usb_diag.c")
        with open(path) as f:
            text = f.read()
        m = re.search(r'#define RECORD_FIELDS((?:\s*"[^"]*"\s*\\?)+)', text)
        self.assertIsNotNone(m, "no RECORD_FIELDS in usb_diag.c")
        fields = "".join(re.findall(r'"([^"]*)"', m.group(1))).split(",")
        self.assertIn(len(fields), diag.RECORD_LENGTHS)
        self.assertEqual(fields, list(diag.RECORD_FIELDS[:len(fields)]))

    def test_record_from_older_firmware(self):
        """Firmware without the camera height observer sends 46 fields: the cv* columns are None."""
        old = self.VALS[:len(diag.RECORD_FIELDS_V1)]
        self.assertEqual(len(old), 46)
        r = diag.parse_record(make_line("R", old))
        self.assertIsNotNone(r)
        self.assertEqual(r["linkAge_ms"], 25)
        self.assertIsNone(r["cvH_mm"])
        self.assertIsNone(r["cvSigV_mms"])
        self.assertIsNone(r["autoState"])
        self.assertEqual(set(r), set(diag.RECORD_FIELDS))

    def test_record_from_firmware_before_a2b(self):
        """52 fields (camera observer, no autonomy): the auto*/vz* columns are None."""
        r = diag.parse_record(make_line("R", self.VALS[:52]))
        self.assertIsNotNone(r)
        self.assertEqual(r["cvSigV_mms"], 21)
        for k in ("autoState", "autoReason", "vzPhase", "vzH_mm", "vzV_mms", "vzThrust", "autoFlags"):
            self.assertIsNone(r[k], k)
        for n in (45, 47, 51, 53, 58, 60, 63, 65, 71, 74):
            self.assertIsNone(diag.parse_record(make_line("R", (self.VALS * 2)[:n])), n)

    def test_csv_with_empty_observer_columns(self):
        r = diag.parse_record(make_line("R", self.VALS[:46]))
        buf = io.StringIO()
        w = diag.csv.writer(buf)
        w.writerow(diag.RECORD_FIELDS)
        w.writerow([r[k] for k in diag.RECORD_FIELDS])
        buf.seek(0)
        rows = diag.read_rows(buf)
        self.assertEqual(rows[0]["thrust"], 30000)
        self.assertNotIn("cvH_mm", rows[0])

    def test_corrupted_record_rejected(self):
        line = make_line("R", self.VALS)
        self.assertIsNone(diag.parse_record(line.replace("30000", "30001", 1)))
        self.assertIsNone(diag.parse_record(line.split("*")[0]))
        self.assertIsNone(diag.parse_record(make_line("R", self.VALS[:-2])))


class SteadyTorqueTest(unittest.TestCase):
    def test_mixer_inversion_recovers_roll_and_pitch(self):
        # Forward X mixer from power_distribution_stock.c with known outputs.
        T, roll, pitch, yaw = 45000, 3000, -5600, 300
        r, p = roll / 2, pitch / 2
        m = (T - r + p + yaw, T - r - p - yaw, T + r - p + yaw, T + r + p - yaw)
        rows = []
        for i in range(400):
            vals = [i, 1000 + 10 * i, 4, diag.FLAG_ANGLE | diag.FLAG_ARMED, 45000, 0, 0, 10, -20, 0,
                    0, 0, 0, 0, 0, 1000, *[int(x) for x in m], 0, 0, 0, 0]
            rows.append(dict(zip(diag.RECORD_FIELDS, vals)))
        res = diag.steady_torque(rows)
        self.assertEqual(len(res), 1)
        fid, n, got_roll, got_pitch = res[0]
        self.assertEqual(fid, 4)
        self.assertEqual(n, 200)  # samples after the 2 s settle time
        self.assertAlmostEqual(got_roll, roll, delta=1)
        self.assertAlmostEqual(got_pitch, pitch, delta=1)

    def test_stick_input_and_saturation_excluded(self):
        rows = []
        for i in range(400):
            sat = 65535 if i % 2 else 45000
            vals = [i, 10 * i, 1, diag.FLAG_ANGLE, 45000, 0, 500, 0, 0, 0,
                    0, 0, 0, 0, 0, 1000, sat, 45000, 45000, 45000, 0, 0, 0, 0]
            rows.append(dict(zip(diag.RECORD_FIELDS, vals)))
        self.assertEqual(diag.steady_torque(rows), [])


class FlowCalibrationTest(unittest.TestCase):
    def test_recovers_rotation_coefficients(self):
        import random
        random.seed(3)
        rows = []
        for i in range(300):
            gx, gy, gz = (random.uniform(-0.02, 0.02) for _ in range(3))
            r = dict.fromkeys(diag.RECORD_FIELDS, 0)
            r.update(flowSeq=i % 255 + 1, flowOk=1, flowTexture=40,
                     flowGx_e4=round(gx * 1e4), flowGy_e4=round(gy * 1e4), flowGz_e4=round(gz * 1e4),
                     flowDx_cpx=round((131.0 * gz + 0.02 + random.gauss(0, 0.02)) * 100),
                     flowDy_cpx=round((-127.0 * gy + random.gauss(0, 0.02)) * 100),
                     flowRot_e4=round((-1.0 * gx + random.gauss(0, 1e-4)) * 1e4))
            rows.append(r)
            rows.append(dict(r))  # the recorder repeats a frame until the next one
        cal = diag.flow_calibration(rows)
        self.assertIsNotNone(cal)
        self.assertEqual(cal["frames"], 300)
        self.assertAlmostEqual(cal["fits"]["dx"]["gz"], 131.0, delta=2.0)
        self.assertAlmostEqual(cal["fits"]["dy"]["gy"], -127.0, delta=2.0)
        self.assertAlmostEqual(cal["fits"]["rot"]["gx"], -1.0, delta=0.05)
        self.assertLess(abs(cal["fits"]["dx"]["gy"]), 5.0)
        self.assertGreater(cal["fits"]["dx"]["r2"], 0.95)

    def test_needs_texture(self):
        rows = [dict(dict.fromkeys(diag.RECORD_FIELDS, 0), flowSeq=i % 255 + 1, flowOk=1, flowTexture=0)
                for i in range(100)]
        self.assertIsNone(diag.flow_calibration(rows))


class FlowOffsetScanTest(unittest.TestCase):
    """flow-cal --offset-scan (docs/autonomy-a2b.md 17.2): the gyro window offset recovered from 100 Hz recorder
    rows whose frames (67.5 ms apart, first seen ~120 ms after capture) were made with a known offset."""

    @staticmethod
    def synth(true_off_ms, seconds=40.0, seed=5):
        import random
        rnd = random.Random(seed)
        comps = [[(rnd.uniform(0.2, 1.6), rnd.uniform(0.2, 0.9), rnd.uniform(0, 6.28)) for _ in range(4)]
                 for _ in range(3)]

        def rate(a, t_ms):      # rad/s
            t = t_ms / 1000.0
            return sum(amp * math.sin(2 * math.pi * f * t + ph) for f, amp, ph in comps[a])

        def angle(a, t0, t1, n=40):
            h = (t1 - t0) / n
            return sum(rate(a, t0 + (k + 0.5) * h) for k in range(n)) * h / 1000.0

        frames_ = []
        t, seq = 200.0, 0
        while t < seconds * 1000 - 300:
            seq += 1
            frames_.append((seq, int(round(t)), 67.5))
            t += 67.5
        rows = []
        k = math.pi / 180.0 / 10.0
        for tick in range(0, int(seconds * 1000), 10):
            pub = [f for f in frames_ if f[1] + 120 <= tick]
            r = dict.fromkeys(diag.RECORD_FIELDS, 0)
            r.update(tickMs=tick, gx_ddps=round(rate(0, tick) / k), gy_ddps=round(rate(1, tick) / k),
                     gz_ddps=round(rate(2, tick) / k))
            if pub:
                seq, tf, dt = pub[-1]
                ax, ay, az = (angle(a, tf - dt + true_off_ms, tf + true_off_ms) for a in range(3))
                r.update(flowSeq=seq % 65535 + 1, flowOk=1, flowTexture=40, flowDt_100us=int(dt * 10),
                         flowAge_ms=tick - tf,
                         flowDx_cpx=round((-1.0 * ax - 4.9 * ay + 102.4 * az + rnd.gauss(0, 0.03)) * 100),
                         flowDy_cpx=round((1.28 * ax + 99.8 * ay - 2.76 * az + rnd.gauss(0, 0.03)) * 100))
            rows.append(r)
        return rows

    def test_finds_26_ms(self):
        res = diag.flow_offset_scan(self.synth(26.0))
        self.assertIsNotNone(res)
        self.assertTrue(24 <= res["best_ms"] <= 28, res["best_ms"])
        self.assertGreater(res["r2"], 0.95)
        self.assertAlmostEqual(res["dx_gz"], 102.4, delta=5.0)

    def test_finds_15_ms(self):
        res = diag.flow_offset_scan(self.synth(15.0, seconds=30.0, seed=9))
        self.assertTrue(13 <= res["best_ms"] <= 17, res["best_ms"])

    def test_needs_frames(self):
        self.assertIsNone(diag.flow_offset_scan(self.synth(26.0, seconds=1.0)))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_offset_scan(None)
        self.assertIn("not enough", buf.getvalue())

    def test_vga_centre_negative_residual(self):
        res = diag.flow_offset_scan(self.synth(-12.0), offsets=range(-40, 41))
        self.assertIsNotNone(res)
        self.assertLessEqual(abs(res["best_ms"] + 12), 2)

    def test_vga_centre_zero_residual(self):
        res = diag.flow_offset_scan(self.synth(0.0), offsets=range(-40, 41))
        self.assertIsNotNone(res)
        self.assertLessEqual(abs(res["best_ms"]), 2)


class FlightEndCauseTest(unittest.TestCase):
    def flight(self, end):
        base = dict.fromkeys(diag.RECORD_FIELDS, 0)
        rows = [dict(base, thrust=48000, rawThrust=48000, linkAge_ms=20) for _ in range(10)]
        rows.append(dict(base, **end))
        return rows

    def test_app_zero(self):
        self.assertIn("APP SENT ZERO", diag.flight_end_cause(self.flight(dict(thrust=0, rawThrust=0, linkAge_ms=15))))

    def test_lock_after_gap(self):
        self.assertIn("THRUST LOCK", diag.flight_end_cause(self.flight(dict(thrust=0, rawThrust=47000, linkAge_ms=10, linkFlags=1))))

    def test_link_lost(self):
        self.assertIn("LINK LOST", diag.flight_end_cause(self.flight(dict(thrust=0, rawThrust=48000, linkAge_ms=2001))))

    def test_landing(self):
        rows = self.flight(dict(thrust=0, rawThrust=0, linkAge_ms=15))
        rows[-2]["thrust"] = 9000
        self.assertIn("landing", diag.flight_end_cause(rows))


class AnalyzeTest(unittest.TestCase):
    def test_analyze_synthetic_flight(self):
        rows = []
        for i in range(300):
            thrust = min(40000, i * 400) if i < 250 else 0
            vals = [i, 1000 + 10 * i, 1, diag.FLAG_ANGLE | diag.FLAG_ARMED, thrust, 0, 0, 50, -30, 0,
                    0, 0, 0, 0, 0, 1000, thrust, thrust, thrust, thrust, 10, -5, 300, -150]
            rows.append(dict(zip(diag.RECORD_FIELDS, vals)))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.analyze(rows)
        out = buf.getvalue()
        self.assertIn("flight 1", out)
        self.assertIn("angle (self-level)", out)
        self.assertNotIn("not a pure-throttle", out)



class HoldFlagsTest(unittest.TestCase):
    def test_decode_scene_gate_bit(self):
        self.assertEqual(diag.hold_flag_names(0x0B), ["enabled", "applied", "sceneGated"])
        self.assertEqual(diag.hold_flag_names(0x07), ["enabled", "applied", "coasting"])
        self.assertEqual(diag.hold_flag_names(0x08), ["sceneGated"])
        self.assertEqual(diag.hold_flag_names(0x10), ["bit4"])
        self.assertEqual(diag.hold_flag_names(None), [])
        self.assertEqual(diag.HOLD_SCENE_GATED, 8)

    def test_hold_summary_counts_gated_frames(self):
        rows = []
        for i in range(100):
            gated = 40 <= i < 70
            flags = diag.HOLD_ENABLED | (diag.HOLD_APPLIED | diag.HOLD_COASTING if gated else diag.HOLD_APPLIED)
            rows.append({"flowSeq": 1 + i // 6, "holdFlags": flags | (diag.HOLD_SCENE_GATED if gated else 0)})
        s = diag.hold_summary(rows)
        self.assertEqual(s["frames"], 17)
        self.assertEqual(s["scene_gated"], 5)     # frames 7..11 start inside rows 40..69 (42, 48, 54, 60, 66)
        self.assertAlmostEqual(s["applied_pct"], 100.0)
        self.assertAlmostEqual(s["coasting_pct"], 30.0)
        self.assertIsNone(diag.hold_summary([{"flowSeq": 1}]))

    def test_analyze_prints_the_scene_gate(self):
        rows = []
        for i in range(300):
            thrust = min(40000, i * 400) if i < 250 else 0
            vals = [i, 1000 + 10 * i, 1, diag.FLAG_ANGLE | diag.FLAG_ARMED, thrust, 0, 0, 50, -30, 0,
                    0, 0, 0, 0, 0, 1000, thrust, thrust, thrust, thrust, 10, -5, 300, -150]
            r = dict(zip(diag.RECORD_FIELDS, vals))
            r.update(flowSeq=1 + i // 6, holdFlags=0x0B if 120 <= i < 180 else 0x03)
            rows.append(r)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.analyze(rows)
        self.assertIn("HOVER scene gate refused 10 of", buf.getvalue())


class FlightSummaryTest(unittest.TestCase):
    LINE = ("F boot=21 id=3 end=3 rst=9 thrCs=520 airCs=400 steadyCs=300 holdCs=200 coastCs=80 camCs=380 "
            "thrMax=52428 vStart=3720 vMin=3010 tiltMax=25 gyroMax=40 tiltSq=4800 gyroSq=7500 "
            "m=40800,45900,51000,43350 holdR=-6000 holdP=16000 rI=4000 pI=-8000 gDrift=-12,55")

    def test_derived_numbers(self):
        r = diag.flight_summary(diag.line_kv(self.LINE))
        self.assertEqual(r["end"], "RESET: brownout")
        self.assertAlmostEqual(r["dur_s"], 5.2)
        self.assertAlmostEqual(r["air_s"], 4.0)
        self.assertAlmostEqual(r["cam_pct"], 95.0)
        self.assertAlmostEqual(r["hold_pct"], 50.0)
        self.assertAlmostEqual(r["hold_pitch"], 0.8)
        self.assertAlmostEqual(r["hold_roll"], -0.3)
        self.assertAlmostEqual(r["tilt_rms"], 0.4)      # sqrt(4800/300)/10
        self.assertAlmostEqual(r["gyro_rms"], 5.0)      # sqrt(7500/300)
        self.assertAlmostEqual(r["motor_pct"][0], 40.0)  # 40800/400/255
        self.assertAlmostEqual(r["motor_pct"][2], 50.0)
        self.assertAlmostEqual(r["roll_rate_i"], 10.0)
        self.assertAlmostEqual(r["coast_pct"], 20.0)
        self.assertEqual(r["gyro_drift"], [-0.12, 0.55])

    def test_diagnostic_fields(self):
        line = self.LINE + " upS=37 joinDs=152 appDs=65535 railMin=2612 vFastMin=2930 tx=410 rx=880"
        r = diag.flight_summary(diag.line_kv(line))
        self.assertEqual(r["up_s"], 37)
        self.assertAlmostEqual(r["since_join_s"], 15.2)
        self.assertIsNone(r["since_app_s"])
        self.assertAlmostEqual(r["rail_min"], 2.612)
        self.assertAlmostEqual(r["vbat_fast_min"], 2.93)
        self.assertEqual((r["tx"], r["rx"]), (410, 880))
        self.assertEqual(diag.rail_text(r["rail_min"]), "2.61")
        self.assertEqual(diag.rail_text(3.08), "3.08")
        self.assertEqual(diag.rail_text(65.535), ">=FS")
        self.assertEqual(diag.rail_text(None), "-")

    def test_older_line_without_new_fields(self):
        line = self.LINE.replace(" coastCs=80", "").replace(" gDrift=-12,55", "")
        r = diag.flight_summary(diag.line_kv(line))
        self.assertEqual(r["coast_pct"], 0.0)
        self.assertEqual(r["gyro_drift"], [0.0, 0.0])

    def test_no_airborne_time(self):
        line = self.LINE.replace("airCs=400", "airCs=0").replace("steadyCs=300", "steadyCs=0").replace("end=3", "end=1")
        r = diag.flight_summary(diag.line_kv(line))
        self.assertEqual(r["end"], "stick released")
        self.assertIsNone(r["tilt_rms"])
        self.assertIsNone(r["motor_pct"])
        self.assertEqual(r["cam_pct"], 0.0)

    def test_print_with_trace(self):
        lines = [self.LINE, "C i=-2 thr=52428 v=3100 m=200,210,230,205 r=-3 p=4 g=12 f=11 la=0",
                 "C i=-1 thr=52428 v=2900 m=255,255,255,255 r=-1 p=2 g=30 f=9 la=1",
                 "V win_ms=10 mv=3350,3340,3320,1200"]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rows = diag.print_flights(lines, "END flights trace=2 traceBoot=21 traceReset=brownout(9)")
        self.assertEqual(len(rows), 1)
        out = buf.getvalue()
        self.assertIn("RESET: brownout", out)
        self.assertIn("before the reset (brownout(9))", out)
        self.assertIn("air hold cam", out)
        self.assertIn("lowest 1.20 V, largest drop between windows 2.12 V", out)

    def test_print_rail_windows_and_timing(self):
        line = (self.LINE + " upS=37 joinDs=152 appDs=98 railMin=2612 vFastMin=2930 tx=410 rx=880 gapMaxUs=55274 "
                "gaps2500=3 pwmKhz=40 soft=1")
        lines = [line, "C i=-1 thr=52428 v=2900 m=255,255,255,255 r=-1 p=2 g=30 f=9 la=1 rail=2610 tx=3 rx=4",
                 "V win_ms=10 mv=3350,3340,3320,2900", "R win_ms=10 mv=3100,3060,2650,2480"]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_flights(lines, "END flights trace=1 traceBoot=21 traceReset=brownout(9)")
        out = buf.getvalue()
        self.assertIn("since phone joined", out)
        self.assertIn("15.2s", out)
        self.assertIn("2.61", out)
        self.assertIn("lowest XIAO 3.3 V rail reading per 10 ms", out)
        self.assertIn("lowest 2.48 V; windows below 2.70 V: 2, below 2.50 V: 1", out)
        self.assertIn("55.3ms/3", out)
        self.assertIn("40kHz+soft", out)

    CVZ = (" cvf=123 cvFirstDs=11 cvG0=-297 cvB3=-2715 cvB5=-2887 cvBtd=-2905 cvSb5=105 cvGain=1259 cvHtd=102 "
           "cvVtd=-3 cvHmax=310 cvAir=1132 cvCamAir=950 cvOk=836 cvGap=12 cvAcc=148 cvRej=2 cvAlarm=1 cvYaw=0")

    def test_camera_observer_fields(self):
        r = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ))
        c = r["camvz"]
        self.assertTrue(c["ran"] and c["lifted"] and c["touchdown"] and c["impact"] and c["gain_tracked"])
        self.assertTrue(c["prior"])
        self.assertFalse(c["fault"])
        self.assertEqual(c["lift"], "thrust")
        self.assertAlmostEqual(c["g0_mg"], -29.7)
        self.assertAlmostEqual(c["bias3_mg"], -27.15)
        self.assertAlmostEqual(c["b3_g0_mg"], 2.55)          # against the ground baseline
        self.assertAlmostEqual(c["b5_g0_mg"], 0.83)
        self.assertAlmostEqual(c["sig5_mg"], 1.05)
        self.assertAlmostEqual(c["gain"], 125.9)
        self.assertAlmostEqual(c["h_td"], 0.102)
        self.assertAlmostEqual(c["v_td"], -0.003)
        self.assertAlmostEqual(c["h_max"], 0.31)
        self.assertAlmostEqual(c["air_s"], 11.32)
        self.assertAlmostEqual(c["ok_pct"], 88.0)
        self.assertAlmostEqual(c["first_ok_s"], 1.1)
        self.assertAlmostEqual(c["gap_s"], 0.12)
        self.assertAlmostEqual(c["drift_ms"], 0.102 / 9.5)
        self.assertEqual((c["accepted"], c["rejected"], c["alarms"], c["yaw_resets"]), (148, 2, 1, 0))
        self.assertIsNone(c["late"])                                      # firmware before the late offset noise

    def test_late_offset_noise_fields(self):
        """The 'F' line's late offset noise (cvq*, docs/autonomy-a2b.md 25): which version flew."""
        c = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ + " cvqf=3 cvq=5 cvqS=40 cvqG=1 cvqDs=463"))["camvz"]
        self.assertEqual(c["late"], {"mult": 5, "step_s": 40, "guards": 1, "on_s": 46.3, "step_reached": True,
                                     "cfg_bad": False})
        self.assertEqual(diag.late_text(c["late"]), "x5 from 40 s: on 46.3 s, 1 guards")
        off = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ + " cvqf=5 cvq=1 cvqS=40 cvqG=0 cvqDs=0"))["camvz"]
        self.assertEqual(diag.late_text(off["late"]), "off (x1) (stored setting REFUSED at boot)")
        short = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ + " cvqf=1 cvq=3 cvqS=40 cvqG=0 cvqDs=0"))
        self.assertEqual(diag.late_text(short["camvz"]["late"]), "x3 from 40 s: not reached")
        none = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ + " cvqf=0 cvq=0 cvqS=0 cvqG=0 cvqDs=0"))
        self.assertIsNone(none["camvz"]["late"])                         # no sortie in this flight

    def test_t2_fields(self):
        """The 'F' line's offset drift rate T2 (cvt2*, docs/autonomy-a2b.md 29): which version flew."""
        on = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ + " cvqf=9 cvq=1 cvqS=40 cvqG=0 cvqDs=0 cvt2k=50 "
                                              "cvt2r=124 cvt2Ds=623 cvt2bR=15,-8,35"))["camvz"]
        self.assertEqual(on["t2"], {"kR": 0.5, "prior_mgs": 0.124, "on_s": 62.3, "rate_mgs": 0.15, "min_mgs": -0.08,
                                    "max_mgs": 0.35, "started": True, "cfg_bad": False})
        self.assertEqual(diag.t2_text(on["t2"]), "k0.50 r+0.124: on 62.3 s, bR +0.15 (-0.08..+0.35) mg/s")
        self.assertEqual(diag.late_text(on["late"]), "off (x1)")
        short = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ + " cvqf=1 cvq=1 cvqS=40 cvqG=0 cvqDs=0 "
                                                 "cvt2k=50 cvt2r=60 cvt2Ds=0 cvt2bR=0,0,0"))["camvz"]
        self.assertEqual(diag.t2_text(short["t2"]), "k0.50 r+0.060: not started (shorter than its 30 s)")
        bad = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ + " cvqf=17 cvq=5 cvqS=40 cvqG=0 cvqDs=0 "
                                               "cvt2k=0 cvt2r=124 cvt2Ds=0 cvt2bR=0,0,0"))["camvz"]
        self.assertEqual(diag.t2_text(bad["t2"]), "off (stored setting REFUSED at boot)")
        older = diag.flight_summary(diag.line_kv(self.LINE + self.CVZ + " cvqf=3 cvq=5 cvqS=40 cvqG=1 cvqDs=463"))
        self.assertIsNone(older["camvz"]["t2"])                           # firmware before T2

    def test_t2_status_line(self):
        kv = diag.parse_kv("OK status fw=x t2k=0.50 t2r=+0.124 t2Cfg=ok t2=on t2OnS=41.2 t2bR=+0.152 imuTempC=40.25 "
                           "bypass=0x00 diagStackFree=2000")
        self.assertEqual(diag.t2_status_line(kv),
                         "offset drift rate T2: t2=k0.50 r+0.124 on on=41.2s bR=+0.152 mg/s")
        bad = diag.t2_status_line(diag.parse_kv("OK status t2k=0.00 t2r=+0.124 t2Cfg=bad t2=off t2OnS=0.0 t2bR=+0.000"))
        self.assertIn("REFUSED at boot", bad)
        self.assertIsNone(diag.t2_status_line(diag.parse_kv("OK status fw=x imuTempC=40.25")))   # before T2
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            text = f.read()
        i = text.index('"OK status fw=')
        body = text[i:text.index("\n}\n", i)]
        for k in ("t2k=%.2f", "t2r=%+.3f", "t2Cfg=%s", "t2=%s", "t2OnS=%.1f", "t2bR=%+.3f"):
            self.assertIn(k, body)
        self.assertLess(body.index("t2k=%.2f"), body.index("diagStackFree=%u"))
        j = text.index('out("OK camvz on=1')
        camvz = text[j:text.index("\n}\n", j)]
        for k in ("t2k=%.2f", "t2r=%+.3f", "t2S=%.0f", "t2Cfg=%s", "t2=%s", "t2OnS=%.1f", "t2bR=%+.3f",
                  "t2bRmin=%+.3f", "t2bRmax=%+.3f"):
            self.assertIn(k, camvz)
        k = text.index('"F boot=%u')
        flights = text[k:text.index("\n}\n", k)]
        self.assertIn("cvt2k=%u cvt2r=%d cvt2Ds=%u cvt2bR=%d,%d,%d", flights)

    def test_camera_observer_not_reached(self):
        line = self.LINE + (" cvf=3 cvFirstDs=255 cvG0=-32768 cvB3=-32768 cvB5=-32768 cvBtd=125 cvSb5=0 cvGain=0 "
                            "cvHtd=-32768 cvVtd=-32768 cvHmax=88 cvAir=150 cvCamAir=0 cvOk=0 cvGap=0 cvAcc=0 cvRej=0 "
                            "cvAlarm=0 cvYaw=0")
        c = diag.flight_summary(diag.line_kv(line))["camvz"]
        self.assertFalse(c["prior"] or c["touchdown"])
        self.assertIsNone(c["g0_mg"])
        self.assertIsNone(c["bias3_mg"])
        self.assertIsNone(c["b3_g0_mg"])
        self.assertIsNone(c["btd_g0_mg"])            # no g0 to refer it to
        self.assertIsNone(c["h_td"])
        self.assertIsNone(c["ok_pct"])
        self.assertIsNone(c["first_ok_s"])
        self.assertIsNone(c["gain"])
        self.assertIsNone(c["drift_ms"])
        self.assertIsNone(diag.flight_summary(diag.line_kv(self.LINE))["camvz"])   # older firmware

    def test_print_camera_observer_table(self):
        lines = [self.LINE + self.CVZ,
                 self.LINE.replace("id=3", "id=4") + " cvf=1 cvFirstDs=255 cvG0=-32768 cvB3=-32768 cvB5=-32768 "
                 "cvBtd=-32768 cvSb5=0 cvGain=0 cvHtd=-32768 cvVtd=-32768 cvHmax=-32768 cvAir=0 cvCamAir=0 cvOk=0 "
                 "cvGap=0 cvAcc=0 cvRej=0 cvAlarm=0 cvYaw=0",
                 self.LINE.replace("id=3", "id=5") + " cvf=0 cvFirstDs=0 cvG0=0 cvB3=0 cvB5=0 cvBtd=0 cvSb5=0 "
                 "cvGain=0 cvHtd=0 cvVtd=0 cvHmax=0 cvAir=0 cvCamAir=0 cvOk=0 cvGap=0 cvAcc=0 cvRej=0 cvAlarm=0 cvYaw=0"]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_flights(lines, "END flights trace=0 traceBoot=0 traceReset=none(0)")
        out = buf.getvalue()
        self.assertIn("camera height observer (shadow mode)", out)
        self.assertIn("  +2.6", out)        # b@3s against g0 (2.55)
        self.assertIn(" +0.10", out)        # h at touchdown
        self.assertIn("148/2/1/0", out)
        self.assertIn("impact", out)
        self.assertIn("no lift-off", out)   # id 4
        table = out.split("camera height observer (shadow mode)")[1]
        self.assertIn("\n  21   3 ", table)
        self.assertNotIn("\n  21   5 ", table)   # id 5: no sortie (no ground start), no row

    def test_camvz_reply(self):
        reply = ("OK camvz on=1 mem=psram bytes=2944 phase=air sortie=3 lift=thrust g0Mg=-29.70 h=0.212 v=-0.010 "
                 "biasMg=-27.150 sigH=0.020 sigV=0.015 sigBiasMg=1.200 gain=125.9 gainTracked=1 healthy=1 "
                 "airborne=1 fault=0 sinceAcceptMs=40 innov=0.010 innovSig=0.030 lastFrame=ACCEPTED frames=300 "
                 "skipped=40 quality=5 exp=0 lat=0 noWindow=0 noGain=3 accepted=148 rejected=2 relocks=0 alarms=1 "
                 "yawResets=0 seqGaps=0 imuIgnored=0 stepUsMax=6 stepUsAvg=1.20 frameUsMax=14 frameUsAvg=7.5 "
                 "frameCalls=300 stabStackFree=2380 gyroBiasDps=0.10,-0.20,0.00 bench=0 qlate=5 qlateS=40 qlateCfg=ok "
                 "late=on lateMs=46300 lateGuards=1 biasHeldMg=-27.300 t2k=0.00 t2r=+0.124 t2S=30 t2Cfg=ok t2=off "
                 "t2OnS=0.0 t2bR=+0.000 t2bRmin=+0.000 t2bRmax=+0.000" + self.CVZ)
        kv = diag.parse_kv(reply)
        self.assertEqual(kv["phase"], "air")
        c = diag.camvz_summary(kv)
        self.assertAlmostEqual(c["b5_g0_mg"], 0.83)
        self.assertEqual(kv["stabStackFree"], "2380")

        class FakeDrone:
            def command(self, cmd, **kw):
                return (reply if cmd == "camvz" else "ERR"), []
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_camvz(FakeDrone(), None)
        out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn("phase air  h +0.212 m", out)
        self.assertIn("stabilizer stack free 2380 B", out)
        self.assertIn("148/2/1/0", out)
        self.assertIn("offset drift rate T2: t2=k0.00 r+0.124 off on=0.0s bR=+0.000 mg/s (+0.000..+0.000)", out)
        self.assertIn("late offset noise x5 from 40 s: on, on 46.3 s, 1 guards this sortie; the controller's held "
                      "offset -27.30 mg", out)

        class OffDrone:
            def command(self, cmd, **kw):
                return "OK camvz on=0 (no observer: no memory, or the stabilizer has not run yet)", []
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(diag.cmd_camvz(OffDrone(), None), 0)

    def test_format_live(self):
        kv = diag.parse_kv("OK live up_ms=12345 thrust=30000 raw=30000 m=32768,0,65535,16384 vbat=3712 "
                           "vbatMin=3650 rail=65535 railMin=2890 tx=12 rx=340 joinAgeMs=8000 appAgeMs=-1 "
                           "gcap=45000 cam=running")
        s = diag.format_live(kv)
        self.assertIn("t=   12.3s", s)
        self.assertIn("motors=50/0/100/25%", s)
        self.assertIn("rail=>=FS (min 2.89)", s)
        self.assertIn("phone=8s app=-", s)
        self.assertIn("gcap=45000", s)

    def test_restart_detection(self):
        self.assertFalse(diag.restarted(None, None, 1000, 0.0))
        self.assertFalse(diag.restarted(10000, 100.0, 10200, 100.2))   # normal poll
        self.assertTrue(diag.restarted(10000, 100.0, 2500, 102.0))     # uptime fell
        self.assertTrue(diag.restarted(1500, 100.0, 1800, 103.0))      # early-boot reset: up grew 0.3 s in 3 s

    def test_in_air_contact_end_has_no_drift(self):
        """An in-air trip (KILLED TILT, ...) or an in-air contact (LAND COLLISION since 2026-10-13, KILLED in
        98cd4ae..eb9264d): h at touchdown is no drift, so the drift column says n/a (2026-10-13 flight 1)."""
        line = self.LINE.replace("end=3", "end=4") + self.CVZ
        r = diag.flight_summary(diag.line_kv(line + " auto=COLLISION"))
        self.assertEqual(r["end"], "AUTO(COLLISION)")
        self.assertTrue(r["camvz"]["in_air_contact"])
        self.assertIsNone(r["camvz"]["drift_ms"])
        self.assertAlmostEqual(r["camvz"]["h_td"], 0.102)    # still printed: the contact value
        self.assertEqual(diag.flight_summary(diag.line_kv(line + " auto=42"))["end"], "AUTO(COLLISION)")
        self.assertTrue(diag.in_air_end("AUTO(TILT)") and diag.in_air_end("AUTO(FREEFALL)"))
        self.assertFalse(diag.in_air_end("AUTO(HOVER_NO_CAM)") or diag.in_air_end("RESET: brownout"))
        # the palm catch (2026-10-14): a LAND PALM usually ends on a hand in the air, so h at touchdown is no drift
        self.assertTrue(diag.in_air_end("AUTO(PALM)"))
        palm = diag.flight_summary(diag.line_kv(line + " auto=43"))
        self.assertEqual(palm["end"], "AUTO(PALM)")
        self.assertTrue(palm["camvz"]["in_air_contact"])
        self.assertIsNone(palm["camvz"]["drift_ms"])
        self.assertFalse(palm["camvz"]["contact_land"])
        landed = diag.flight_summary(diag.line_kv(line + " auto=HOVER_TIMEOUT"))
        self.assertFalse(landed["camvz"].get("in_air_contact", False))
        self.assertAlmostEqual(landed["camvz"]["drift_ms"], 0.102 / 9.5)
        # a SINK (2026-10-15: the camera height out 5 s, then the slow descent): no drift sample either
        sink = diag.flight_summary(diag.line_kv(line + " auto=SINK"))
        self.assertEqual(sink["end"], "AUTO(SINK)")
        self.assertIsNone(sink["camvz"]["drift_ms"])
        self.assertFalse(sink["camvz"].get("in_air_contact", False))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_flights([line + " auto=COLLISION"], "END flights trace=0 traceBoot=0 traceReset=none(0)")
        out = buf.getvalue()
        self.assertIn("AUTO(COLLISION)", out)
        self.assertIn("drift n/a (in-air contact)", out)
        # LAND COLLISION (2026-10-13): the motors ran to the floor, the values carry the contact
        self.assertIn("h@td/v@td at or after the in-air contact (COLLISION)", out)
        self.assertTrue(r["camvz"]["contact_land"])
        self.assertFalse(diag.flight_summary(diag.line_kv(line + " auto=TILT"))["camvz"]["contact_land"])
        self.assertIn(" +0.10", out)                          # h at touchdown, the contact value
        self.assertIn("    n/a ", out)                         # the drift column

    def test_autonomy_end(self):
        """FSUM_END_AUTO (4): 'end=AUTO(<reason>)', from the firmware's auto=<name> (or a number)."""
        line = self.LINE.replace("end=3", "end=4")
        r = diag.flight_summary(diag.line_kv(line + " auto=HOVER_TIMEOUT"))
        self.assertEqual(r["end"], "AUTO(HOVER_TIMEOUT)")
        self.assertEqual(diag.flight_summary(diag.line_kv(line + " auto=10"))["end"], "AUTO(APP_INPUT)")
        self.assertEqual(diag.flight_summary(diag.line_kv(line))["end"], "AUTO(?)")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_flights([line + " auto=TILT"], "END flights trace=0 traceBoot=0 traceReset=none(0)")
        self.assertIn("AUTO(TILT)", buf.getvalue())


def auto_rows(states, thrust=0, fid=9):
    """Recorder rows (59 fields) of one sortie: (n rows, autoState, autoReason, vzPhase, vzH_mm) segments."""
    rows, i = [], 0
    for n, st, why, phase, h in states:
        for _ in range(n):
            r = dict.fromkeys(diag.RECORD_FIELDS, 0)
            r.update(idx=i, tickMs=1000 + 10 * i, flightId=fid, thrust=thrust if st in (2, 3, 4) else 0,
                     autoState=st, autoReason=why, vzPhase=phase, vzH_mm=h, vzThrust=41000 if st in (2, 3) else 0,
                     autoFlags=0x40 | (0x01 if st == 3 else 0), rawThrust=0, linkAge_ms=20)
            rows.append(r)
            i += 1
    return rows


class CamGroundStandbyTest(unittest.TestCase):
    """docs/autonomy-a2b.md 17.1: the flights fields of the camera's ground standby and the IMU temperature, and
    the drift command's --log-acc / --cam-step (the accelerometer's heat drift against the camera's wake)."""
    F = FlightSummaryTest.LINE

    def test_flight_fields(self):
        r = diag.flight_summary(diag.line_kv(self.F + " imuT0=41.5 imuT1=44.0 camAwakeDs=223"))
        self.assertEqual((r["imu_t0"], r["imu_t1"], r["cam_awake_s"], r["cam_gnd_seen"]), (41.5, 44.0, 22.3, True))
        r = diag.flight_summary(diag.line_kv(self.F + " imuT0=nan imuT1=nan camAwakeDs=-1"))
        self.assertEqual((r["imu_t0"], r["imu_t1"], r["cam_awake_s"], r["cam_gnd_seen"]), (None, None, None, False))
        r = diag.flight_summary(diag.line_kv(self.F))     # an older firmware
        self.assertIsNone(r["imu_t0"])

    def test_flights_print_shows_them(self):
        line = (self.F + " upS=37 joinDs=152 appDs=65535 railMin=2612 vFastMin=2930 tx=410 rx=880 gapMaxUs=1200 "
                "gaps2500=0 pwmKhz=40 soft=1 camsb=1 txlo=1 wifiq=0 sbDid=0 tlDid=0 cold=0 atrim=10,-20 "
                "imuT0=41.5 imuT1=44.0 camAwakeDs=223")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_flights([line], "END flights")
        self.assertIn("41.5->44.0", buf.getvalue())
        self.assertIn("22.3s", buf.getvalue())

    def test_usb_diag_prints_the_fields(self):
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            text = f.read()
        self.assertIn("imuT0=%.1f imuT1=%.1f camAwakeDs=%d", text)
        self.assertIn("camGnd=%d", text)
        self.assertIn("gtcY=%d mki=%d,%d", text)

    def test_thermal_flight_fields(self):
        """docs/autonomy-a2b.md 19.3: the end-of-flight gyro temperature compensation and learned bias."""
        r = diag.flight_summary(diag.line_kv(self.F + " imuT0=43.0 imuT1=33.0 camAwakeDs=15 gtcY=600 mki=-95,410"))
        self.assertEqual((r["gtc_y_end"], r["mki_end"]), (0.6, [-0.095, 0.41]))
        r = diag.flight_summary(diag.line_kv(self.F))     # an older firmware
        self.assertIsNone(r["gtc_y_end"])
        self.assertIsNone(r["mki_end"])
        line = (self.F + " upS=37 joinDs=152 appDs=65535 railMin=2612 vFastMin=2930 tx=410 rx=880 gapMaxUs=1200 "
                "gaps2500=0 pwmKhz=40 soft=1 camsb=1 txlo=1 wifiq=0 sbDid=0 tlDid=0 cold=0 atrim=10,-20 "
                "imuT0=43.0 imuT1=33.0 camAwakeDs=15 gtcY=600 mki=-95,410")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_flights([line], "END flights")
        self.assertIn("+0.60", buf.getvalue())
        self.assertIn("-0.10/+0.41", buf.getvalue())

    def test_thermal_summary(self):
        rows = []
        for i in range(200):
            rows.append(dict(thrust=45000, imuTemp_cC=4300 - 5 * i, gtcCorrX_mdps=-i, gtcCorrY_mdps=3 * i,
                             mkiBiasX_mdps=0, mkiBiasY_mdps=i, flowSeq=i // 7 + 1,
                             flowGain_e3=(40 if i // 7 == 3 else (-100 if i // 7 == 5 else 0)),
                             flowSat_pct=12, flowExpGate=1 if i // 7 == 5 else 0))
        th = diag.thermal_summary(rows)
        self.assertAlmostEqual(th["t_start"], 43.0)
        self.assertAlmostEqual(th["t_end"], 33.05)
        self.assertEqual(th["gtc_end"], (-0.199, 0.597))
        self.assertEqual(th["frames"], 29)
        self.assertAlmostEqual(th["gated_pct"], 100 / 29)
        self.assertAlmostEqual(th["steps_pct"], 200 / 29)
        self.assertEqual(th["sat_max"], 12)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_thermal(th)
        self.assertIn("IMU 43.0 -> 33.0 C", buf.getvalue())

    def test_cam_steps(self):
        self.assertEqual(diag.parse_cam_steps("awake:300,sleep:600,awake:600"),
                         [("awake", 300.0), ("sleep", 600.0), ("awake", 600.0)])
        with self.assertRaises(ValueError):
            diag.parse_cam_steps("on:30")
        steps = diag.parse_cam_steps("awake:10,sleep:20")
        self.assertEqual(diag.cam_step_at(steps, 0.0), (0, "awake"))
        self.assertEqual(diag.cam_step_at(steps, 10.0), (1, "sleep"))
        self.assertEqual(diag.cam_step_at(steps, 99.0), (1, "sleep"))

    def test_log_acc_fields(self):
        m = {"acc": [0.0123, -0.004, 1.0021]}
        a = diag.acc_log_fields(m, "OK status fw=x up_ms=5 cam=standby camGnd=1 camPid=0x3660 imuTempC=40.25")
        self.assertAlmostEqual(a["ax_mg"], 12.3)
        self.assertAlmostEqual(a["az_mg"], 1002.1)
        self.assertEqual((a["imu_t"], a["cam"], a["cam_gnd"]), (40.25, "standby", "1"))
        self.assertIsNone(diag.acc_log_fields(m, "OK status cam=running")["imu_t"])

    def test_diag_stack_note(self):
        self.assertIsNone(diag.diag_stack_note("OK status fw=x bypass=0x00 diagStackFree=2480"))
        self.assertIsNone(diag.diag_stack_note("OK status fw=x bypass=0x00 diagStackFree=1024"))
        self.assertIn("diagStackFree=1023 B < 1024", diag.diag_stack_note("OK status fw=x diagStackFree=1023"))
        self.assertIsNone(diag.diag_stack_note("OK status fw=x imuTempC=40.25 bypass=0x00"))   # before 2026-10-16
        self.assertIsNone(diag.diag_stack_note("OK status fw=x diagStackFree=?"))

    def test_status_ends_with_diag_stack_free(self):
        """usb_diag.c's 'status' keeps its last fields and prints diagStackFree after them, last."""
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            text = f.read()
        if "diagStackFree=" not in text:
            self.skipTest("usb_diag.c before 2026-10-16")
        i = text.index('"OK status fw=')
        body = text[i:text.index("\n}\n", i)]
        self.assertIn('"imuTempC=%.2f bypass=0x%02x diagStackFree=%u"', body)
        self.assertIn("uxTaskGetStackHighWaterMark(NULL)", body)

    def synth(self, d_t, tempco, tau, interval=15.0):
        """awake 300 s (at equilibrium), sleep 600 s (cools by d_t), awake 600 s (warms by d_t), tau each."""
        steps = [("awake", 300.0), ("sleep", 600.0), ("awake", 600.0)]
        rows, t, temp0 = [], 0.0, 45.0
        while t < 1500.0:
            i, kind = diag.cam_step_at(steps, t)
            if i == 0:
                temp = temp0
            elif i == 1:
                temp = temp0 - d_t * (1 - math.exp(-(t - 300.0) / tau))
            else:
                t_sleep_end = temp0 - d_t * (1 - math.exp(-600.0 / tau))
                temp = t_sleep_end + (temp0 - t_sleep_end) * (1 - math.exp(-(t - 900.0) / tau))
            rows.append({"t_s": t, "imu_t": temp, "az_mg": 1000.0 + tempco * (temp - temp0), "step": i})
            t += interval
        return rows, steps

    def test_step_report_small_drift(self):
        rows, steps = self.synth(d_t=4.0, tempco=0.5, tau=200.0)
        rep = diag.cam_step_report(rows, steps)
        self.assertAlmostEqual(rep["tempco_mg_per_c"], 0.5, places=3)
        self.assertAlmostEqual(rep["tau_s"], 200.0, delta=20.0)
        self.assertAlmostEqual(rep["steps"][1]["dT"], -4.0 * (1 - math.exp(-585.0 / 200.0)), delta=0.3)
        self.assertLess(rep["predicted_mg"][3.0], 1.0)
        self.assertIsNone(rep["camlead_ms"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_cam_step_report(rep)
        self.assertIn("camlead 0 is fine", buf.getvalue())

    def test_step_report_asks_for_a_lead(self):
        rows, steps = self.synth(d_t=10.0, tempco=1.0, tau=120.0)
        rep = diag.cam_step_report(rows, steps)
        self.assertGreater(rep["predicted_mg"][3.0], 1.0)
        self.assertIsNotNone(rep["camlead_ms"])
        lead = rep["camlead_ms"] / 1000.0
        tau, d_t, tc = rep["tau_s"], rep["dT_wake_c"], rep["tempco_mg_per_c"]
        self.assertLessEqual(abs(d_t * tc * (math.exp(-lead / tau) - math.exp(-(lead + 60) / tau))), 1.0)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_cam_step_report(rep)
        self.assertIn(f"tkoff camlead {rep['camlead_ms']}", buf.getvalue())

    def test_drift_with_camera_steps(self):
        class Console:
            def __init__(self):
                self.sent = []

            def command(self, cmd, timeout=3.0, until=("OK", "ERR")):
                self.sent.append(cmd)
                if cmd.startswith("measure"):
                    return ("OK measure n=100 ms=50 still=1 motorsZero=1 maxThrust=0 acc=0.01000,-0.00200,1.00100 "
                            "accStd=0.001,0.001,0.001 gyro=0.1,0.2,0.3 gyroStd=0.01,0.01,0.01 gyroBias=0,0,0 "
                            "calibrated=1 level=0,0 accScale=1.0 accRoll=0.1 accPitch=0.2 estRoll=0.1 estPitch=0.2 "
                            "estRollStd=0.01 estPitchStd=0.01"), []
                if cmd == "status":
                    return "OK status fw=x up_ms=1000 cam=running camGnd=0 imuTempC=41.00", []
                return "OK tkoff camgnd=1", []
        c = Console()
        out = os.path.join(tempfile.mkdtemp(), "drift.csv")
        args = types.SimpleNamespace(minutes=0.002, interval=0.0, window_ms=50, out=out, log_acc=True,
                                     cam_step="awake:1")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.cmd_drift(c, args)
        self.assertTrue(c.sent[1].startswith("tkoff camwake "), c.sent[:3])
        self.assertEqual(c.sent[-1], "tkoff camwake 0")
        with open(out) as f:
            rows = list(diag.csv.DictReader(f))
        self.assertTrue(rows)
        self.assertEqual((float(rows[0]["az_mg"]), float(rows[0]["imuTempC"]), rows[0]["cam"], rows[0]["camStep"]),
                         (1001.0, 41.0, "running", "awake"))
        self.assertIn("camera step test", buf.getvalue())


class FakeSerial:
    """A serial port that returns a canned byte stream (for Drone.command)."""
    def __init__(self, data):
        self.data = data
        self.sent = b""

    def read(self, n):
        d, self.data = self.data[:n], self.data[n:]
        return d

    def write(self, b):
        self.sent += b

    def reset_input_buffer(self):
        pass


def fake_drone(stream):
    d = diag.Drone.__new__(diag.Drone)
    d.buf = b""
    d.ser = FakeSerial(stream)
    return d


class FakeSerialReplies(FakeSerial):
    """A serial port that answers each command with the next canned reply (Drone.command drains the input before
    every command, so a multi-command exchange cannot come from one stream)."""
    def __init__(self, replies):
        super().__init__(b"")
        self.replies = list(replies)

    def write(self, b):
        super().write(b)
        self.data += self.replies.pop(0).encode() if self.replies else b"ERR no canned reply\n"


def fake_drone_replies(replies):
    d = diag.Drone.__new__(diag.Drone)
    d.buf = b""
    d.ser = FakeSerialReplies(replies)
    return d


class GyroTcTest(unittest.TestCase):
    """docs/autonomy-a2b.md 19.1 / autonomy-bench.md I5: 'gtc show' prints its post-flight re-zero (A) and the
    logged re-zeros (Z) before its OK line, so that a reader stopping at OK (raw) has them all, and 'gtc'
    checks the post-flight residual."""
    OK = ("OK gtc on=1 k=+0.018,-0.060,+0.000 knee=44.0 clamp=1.00 src=default T=38.10 Tlp=38.09 Tref=38.05 "
          "refSet=1 latchAgeMs=420 dTeff=+0.04 corr=+0.001,-0.002,+0.000 model=+0.001,-0.002,+0.000 rezeros=812 "
          "logged=1 n=1 airSeq=1 thrustSinceLatch=0 airSinceLatch=0 hold=0 holdLeftS=0 bypass=0")
    A = ("A seq=1 ageS=95 landS=2.6 airS=48.3 Tref=43.00 T=33.40 dTeff=-9.60 step=-0.150,+0.650,+0.004 "
         "corr=-0.173,+0.576,+0.000 r=+0.023,+0.074,+0.004 on=1")
    Z = ("Z i=-1 Tref=43.00 T=33.40 dTeff=-9.60 step=-0.150,+0.650,+0.004 corr=-0.173,+0.576,+0.000 "
         "r=+0.023,+0.074,+0.004 flight=1 air=1 hold=0 on=1")

    def usb_diag(self):
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            return f.read()

    def test_firmware_prints_the_details_before_ok(self):
        text = self.usb_diag()
        body = text[text.index("#define GTC_RZ_FMT"):text.index("static void cmdGyroTc(")]
        a, z, ok = body.index('out("A seq='), body.index('out("Z i='), body.index('out("OK gtc on=')
        self.assertLess(a, ok)
        self.assertLess(z, ok)
        self.assertNotIn("END gtc", text)
        for key in ("seq=", "ageS=", "landS=", "airS=", "Tref=", "dTeff=", "step=", "corr=", "r=", "air=", "hold="):
            self.assertIn(key, body)

    def test_raw_prints_every_line(self):
        stream = (self.A + "\n" + self.Z + "\n" + self.OK + "\n").encode()
        d = fake_drone(stream)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.cmd_raw(d, types.SimpleNamespace(text=["gtc show"], timeout=1.0))
        out = buf.getvalue()
        self.assertIn("A seq=1", out)
        self.assertIn("Z i=-1", out)
        self.assertIn("OK gtc on=1", out)
        self.assertEqual(d.buf + d.ser.data, b"")    # nothing left unread
        self.assertEqual(d.ser.sent, b"gtc show\n")

    def test_gtc_command_pass(self):
        stream = ("I (123) some log line\n" + self.A + "\n" + self.Z + "\n" + self.OK + "\n").encode()
        d = fake_drone(stream)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_gtc(d, types.SimpleNamespace(args=[]))
        out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertEqual(d.ser.sent, b"gtc show\n")
        self.assertIn(self.Z, out)
        self.assertIn("r x/y +0.023/+0.074 dps at dTeff -9.60 C", out)
        self.assertIn("PASS", out)
        self.assertNotIn("refine: gtc set", out)

    def check(self, r, step=None, corr=None, dteff="-9.60", k=(0.018, -0.060, 0.0)):
        """An A line as the firmware would print it: corr = k x dTeff (clamped at 1 dps), step = r + corr."""
        rr = [float(x) for x in r.split(",")]
        if corr is None:
            corr = ",".join(f"{max(-1.0, min(1.0, kk * float(dteff))):+.3f}" for kk in k)
        if step is None:
            step = ",".join(f"{a + c:+.3f}" for a, c in zip(rr, (float(x) for x in corr.split(","))))
        a = diag.line_kv(f"A seq=2 ageS=5 landS=2.4 airS=40.0 Tref=43.00 T=33.40 dTeff={dteff} "
                         f"step={step} corr={corr} r={r} on=1")
        ks = ",".join(f"{kk:+.3f}" for kk in k)
        return diag.gtc_check(a, diag.parse_kv(self.OK.replace("k=+0.018,-0.060,+0.000", "k=" + ks)))

    def test_refine_rule(self):
        c = self.check("+0.023,+0.074,+0.000")
        self.assertTrue(c["pass"])
        self.assertIsNone(c["refine"])
        # |r_y| > 0.10 with |dTeff| >= 5 C: ky + r_y / dTeff (flight 4 uncompensated would be +0.65 at -10 C);
        # a pass (< 0.15) can still be refined
        c = self.check("+0.000,+0.120,+0.000", dteff="-10.00")
        self.assertTrue(c["pass"])
        self.assertEqual(c["refine"], "gtc set 0.018 -0.072 0.000 44")
        c = self.check("+0.000,+0.160,+0.000", dteff="-10.00")
        self.assertFalse(c["pass"])
        self.assertEqual(c["refine"], "gtc set 0.018 -0.076 0.000 44")
        c = self.check("+0.000,-0.120,+0.000", dteff="-10.00")
        self.assertEqual(c["refine"], "gtc set 0.018 -0.048 0.000 44")
        # roll too, over 0.08
        c = self.check("+0.090,+0.000,+0.000", dteff="-10.00")
        self.assertEqual(c["refine"], "gtc set 0.009 -0.060 0.000 44")
        # a small cool-down: no refinement
        c = self.check("+0.000,+0.120,+0.000", dteff="-3.00")
        self.assertIsNone(c["refine"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_gtc_check(c)
        self.assertIn("no refinement: |dTeff| < 5 C", buf.getvalue())
        # out of 'gtc set' range: reported, not offered
        c = self.check("+0.000,+1.000,+0.000", dteff="-10.00")
        self.assertIsNone(c["refine"])
        self.assertTrue(c["refine_out_of_range"])
        # k changed since the flight (the A line's corr is the old model's): step / dTeff, not k_now + r / dTeff
        c = self.check("+0.000,+0.120,+0.000", dteff="-10.00")
        self.assertEqual(c["refine"], "gtc set 0.018 -0.072 0.000 44")
        a_old = "A seq=2 ageS=5 landS=2.4 airS=40.0 Tref=43.00 T=33.00 dTeff=-10.00 step=-0.180,+0.720,+0.000 " \
                "corr=-0.180,+0.600,+0.000 r=+0.000,+0.120,+0.000 on=1"
        c = diag.gtc_check(diag.line_kv(a_old), diag.parse_kv(self.OK.replace("-0.060", "-0.072")))
        self.assertIsNone(c["refine"])     # already refined: -0.072 = 0.720 / -10
        # the correction clamped at 1 dps (dTeff -20 C with k -0.060): r is the clamp, not a wrong k
        c = self.check("+0.000,+0.200,+0.000", dteff="-20.00")
        self.assertIsNone(c["refine"])
        self.assertFalse(c["refine_out_of_range"])

    def test_sign_error(self):
        # the model has the opposite sign to the measured drift: r = 2 x step, 'gtc off'
        c = self.check("+0.000,+1.250,+0.000", step="+0.000,+0.650,+0.000", corr="+0.000,-0.600,+0.000",
                       dteff="-10.00")
        self.assertEqual(c["sign"], [1])
        self.assertIsNone(c["refine"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_gtc_check(c)
        self.assertIn("SIGN", buf.getvalue())
        self.assertIn("gtc off", buf.getvalue())

    def test_no_flight_yet(self):
        stream = ("A seq=0 (no re-zero after a flight yet this boot)\n" + self.OK.replace("airSeq=1", "airSeq=0") +
                  "\n").encode()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_gtc(fake_drone(stream), types.SimpleNamespace(args=["show"]))
        self.assertEqual(rc, 0)
        self.assertIn("no ground re-zero after a flight yet", buf.getvalue())

    def test_other_subcommands(self):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_gtc(fake_drone(b"ERR gtc set: |k| <= 0.15 dps/C, knee 30..60 C\n"),
                              types.SimpleNamespace(args=["set", "0.2", "0", "0"]))
        self.assertEqual(rc, 1)
        self.assertNotIn("post-flight", buf.getvalue())

    def test_flights_residual(self):
        f = FlightSummaryTest.LINE
        r = diag.flight_summary(diag.line_kv(f + " gtcRf=3 gtcR=23,74 gtcRdT=-960 gtcRland=26"))
        self.assertEqual(r["gtc_res"], {"r": [0.023, 0.074], "dteff": -9.6, "land_s": 2.6, "on": True})
        self.assertIsNone(diag.flight_summary(diag.line_kv(f + " gtcRf=0 gtcR=0,0 gtcRdT=0 gtcRland=0"))["gtc_res"])
        self.assertIsNone(diag.flight_summary(diag.line_kv(f))["gtc_res"])      # an older firmware
        line = (f + " upS=37 joinDs=152 appDs=65535 railMin=2612 vFastMin=2930 tx=410 rx=880 gapMaxUs=1200 "
                "gaps2500=0 pwmKhz=40 soft=1 camsb=1 txlo=1 wifiq=0 sbDid=0 tlDid=0 cold=0 atrim=10,-20 "
                "imuT0=43.0 imuT1=33.0 camAwakeDs=15 gtcY=600 mki=-95,410 gtcRf=1 gtcR=23,74 gtcRdT=-960 gtcRland=26")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_flights([line], "END flights")
        self.assertIn("+0.023/+0.074 @-9.6C off", buf.getvalue())
        self.assertIn("gtcRf=%u gtcR=%d,%d gtcRdT=%d gtcRland=%u", self.usb_diag())


class AccTcTest(unittest.TestCase):
    """docs/autonomy-a2b.md 36: 'atc show' prints its post-flight entry (A) and the logged entries (Z) before its
    OK line, and 'atc' checks the still z step against the model and reports the measured k_z."""
    OK = ("OK atc on=0 k=+0.000,+0.000,+0.200 knee=60.0 clamp=15.0 src=default T=38.10 Tlp=38.09 Tref=47.70 "
          "refSet=1 refValid=1 latchAgeMs=90000 pending=0 dTeff=-9.61 corr=+0.00,+0.00,+0.00 "
          "model=+0.00,+0.00,-1.92 ref=+12.1,-3.4,+1001.2 still=+13.0,-4.1,+1000.4 stillValid=1 logged=1 n=1 "
          "airSeq=1 thrustSinceLatch=0 airSinceLatch=0 coolSince=0 bypass=0")
    A = ("A seq=1 ageS=95 landS=2.6 Tref=47.70 T=31.00 dTeff=-16.70 step=+0.90,-0.70,-0.77 corr=+0.00,+0.00,+0.00 "
         "r=+0.90,-0.70,-0.77 on=0")
    Z = ("Z i=-1 Tref=47.70 T=31.00 dTeff=-16.70 step=+0.90,-0.70,-0.77 corr=+0.00,+0.00,+0.00 r=+0.90,-0.70,-0.77 "
         "flight=1 air=1 cool=0 on=0")

    def test_firmware_prints_the_details_before_ok(self):
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            text = f.read()
        body = text[text.index("#define ATC_RZ_FMT"):text.index("static void cmdAccTc(")]
        a, z, ok = body.index('out("A seq='), body.index('out("Z i='), body.index('out("OK atc on=')
        self.assertLess(a, ok)
        self.assertLess(z, ok)
        for key in ("seq=", "ageS=", "landS=", "Tref=", "dTeff=", "step=", "corr=", "r=", "air=", "cool="):
            self.assertIn(key, body)

    def test_atc_command_pass(self):
        stream = (self.A + "\n" + self.Z + "\n" + self.OK + "\n").encode()
        d = fake_drone(stream)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_atc(d, types.SimpleNamespace(args=[]))
        out = buf.getvalue()
        self.assertEqual(rc, 0)
        self.assertEqual(d.ser.sent, b"atc show\n")
        self.assertIn(self.Z, out)
        self.assertIn("still z step -0.77 mg, model +0.00, r_z -0.77 mg at dTeff -16.70 C, compensation OFF", out)
        self.assertIn("PASS", out)
        self.assertIn("k_z measured this flight: +0.046 mg/C", out)
        self.assertNotIn("refine:", out)

    def check(self, step, corr="+0.00,+0.00,+0.00", dteff="-16.70", k=(0.0, 0.0, 0.2), on="0"):
        st, co = [float(x) for x in step.split(",")], [float(x) for x in corr.split(",")]
        r = ",".join(f"{a - c:+.2f}" for a, c in zip(st, co))
        a = diag.line_kv(f"A seq=2 ageS=5 landS=2.4 Tref=47.70 T=31.00 dTeff={dteff} step={step} corr={corr} r={r} "
                         f"on={on}")
        ks = ",".join(f"{kk:+.3f}" for kk in k)
        return diag.atc_check(a, diag.parse_kv(self.OK.replace("k=+0.000,+0.000,+0.200", "k=" + ks)))

    def test_refine_rule(self):
        # a 3 mg step over -16.7 C with the compensation off: r = step, FAIL, k_z measured -0.18, offered
        c = self.check("+0.50,-0.20,+3.50")
        self.assertFalse(c["pass"])
        self.assertAlmostEqual(c["k_meas"], 3.5 / -16.7, places=3)
        self.assertEqual(c["refine"], "atc set 0.00 0.00 -0.21 60")
        # on with k_z 0.20 (corr -3.34) and a true -3.34 step: r 0, PASS, nothing to refine
        c = self.check("+0.00,+0.00,-3.34", corr="+0.00,+0.00,-3.34", on="1")
        self.assertTrue(c["pass"])
        self.assertIsNone(c["refine"])
        # a changed resting attitude (x step 25 mg): flagged, no refinement
        c = self.check("+25.00,+1.00,+4.00")
        self.assertTrue(c["tilt"])
        self.assertIsNone(c["refine"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_atc_check(c)
        self.assertIn("TILT", buf.getvalue())
        # a small cool-down: no refinement
        c = self.check("+0.00,+0.00,+4.00", dteff="-3.00")
        self.assertIsNone(c["refine"])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_atc_check(c)
        self.assertIn("no refinement: |dTeff| < 5 C", buf.getvalue())
        # out of range: not offered
        c = self.check("+0.00,+0.00,+40.00")
        self.assertIsNone(c["refine"])

    def test_no_flight_yet(self):
        stream = ("A seq=0 (no re-zero after a flight yet this boot)\n" + self.OK.replace("airSeq=1", "airSeq=0") +
                  "\n").encode()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_atc(fake_drone(stream), types.SimpleNamespace(args=["show"]))
        self.assertEqual(rc, 0)
        self.assertIn("no ground re-zero after a flight yet", buf.getvalue())

    def test_flights_line_carries_the_check(self):
        f = diag.line_kv("F boot=5 id=2 atcRf=3 atcS=-77 atcC=-334 atcRdT=-1670 cool=0")
        a = diag.atc_flight_residual(f)
        self.assertEqual((a["step"], a["corr"], a["dteff"], a["on"]), (-0.77, -3.34, -16.7, True))
        self.assertAlmostEqual(a["r"], 2.57)
        self.assertIsNone(diag.atc_flight_residual(diag.line_kv("F boot=5 id=2 atcRf=0 atcS=0")))
        self.assertIsNone(diag.atc_flight_residual(diag.line_kv("F boot=5 id=2")))


class GroundCoolTest(unittest.TestCase):
    """docs/autonomy-a2b.md 36.3: 'cool S [PWM]' starts the spin, polls its state line until it ends and reports the
    IMU temperature before / after with the accelerometer entry of the spin."""
    START = ("OK cool start state=run pwm=12000 s=20 leftS=20.0 T0=49.50 T=49.50 dT=+0.00 elapsedS=0.0 tiltMax=0.0 "
             "gyroMax=0 runs=1")
    RUN = ("OK cool state=run pwm=12000 s=20 leftS=12.0 T0=49.50 T=47.10 dT=-2.40 elapsedS=8.0 tiltMax=2.1 gyroMax=6 "
           "runs=1")
    DONE = ("OK cool state=DONE pwm=12000 s=20 leftS=0.0 T0=49.50 T=43.80 dT=-5.70 elapsedS=20.0 tiltMax=2.3 "
            "gyroMax=7 runs=1")
    ATC = ("Z i=-1 Tref=49.50 T=43.90 dTeff=-5.60 step=+0.40,-0.30,-1.12 corr=+0.00,+0.00,+0.00 r=+0.40,-0.30,-1.12 "
           "flight=1 air=0 cool=1 on=0\nOK atc on=0 k=+0.000,+0.000,+0.200 knee=60.0 clamp=15.0 src=default T=43.90 "
           "Tlp=43.90 Tref=49.50 refSet=1 refValid=1 latchAgeMs=26000 pending=0 dTeff=-5.60 corr=+0.00,+0.00,+0.00 "
           "model=+0.00,+0.00,-1.12 ref=+12.1,-3.4,+1001.2 still=+12.5,-3.7,+1000.1 stillValid=1 logged=1 n=1 "
           "airSeq=0 thrustSinceLatch=0 airSinceLatch=0 coolSince=0 bypass=0")

    def test_cool_runs_to_done(self):
        d = fake_drone_replies([self.START + "\n", self.RUN + "\n", self.DONE + "\n", self.ATC + "\n"])
        buf = io.StringIO()
        with mock.patch.object(diag.time, "sleep", lambda s: None), contextlib.redirect_stdout(buf):
            rc = diag.cmd_cool(d, types.SimpleNamespace(args=["20"]))
        out = buf.getvalue()
        self.assertEqual(rc, 0, out)
        self.assertEqual(d.ser.sent, b"cool 20\ncool\ncool\natc show\n")
        self.assertIn("IMU 49.5 -> 43.8 C (-5.7) in 20.0 s, end DONE", out)
        self.assertIn("step x/y/z +0.40/-0.30/-1.12 mg over dTeff -5.60 C -> k_z +0.200 mg/C", out)

    def test_cool_refused(self):
        d = fake_drone(b"ERR cool: not resting on the ground\n")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_cool(d, types.SimpleNamespace(args=["20", "12000"]))
        self.assertEqual(rc, 1)
        self.assertEqual(d.ser.sent, b"cool 20 12000\n")

    def test_cool_aborted(self):
        aborted = self.DONE.replace("state=DONE", "state=TILT").replace("elapsedS=20.0", "elapsedS=4.0")
        d = fake_drone_replies([self.START + "\n", aborted + "\n",
                                "OK atc on=0 k=+0.000,+0.000,+0.200 knee=60.0 clamp=15.0 src=default T=47.0 Tlp=47.0 "
                                "Tref=49.50 refSet=1 refValid=1 latchAgeMs=9000 pending=0 dTeff=-2.50 "
                                "corr=+0.00,+0.00,+0.00 model=+0.00,+0.00,-0.50 ref=+12.1,-3.4,+1001.2 "
                                "still=+12.5,-3.7,+1000.1 stillValid=1 logged=0 n=0 airSeq=0 thrustSinceLatch=1 "
                                "airSinceLatch=0 coolSince=1 bypass=0\n"])
        buf = io.StringIO()
        with mock.patch.object(diag.time, "sleep", lambda s: None), contextlib.redirect_stdout(buf):
            rc = diag.cmd_cool(d, types.SimpleNamespace(args=["20"]))
        out = buf.getvalue()
        self.assertEqual(rc, 1)
        self.assertIn("end TILT", out)
        self.assertIn("no re-zero entry yet", out)
        self.assertIn("ended TILT, not DONE", out)

    def test_stop_and_status(self):
        d = fake_drone(b"OK cool stop (ends at the next loop)\n")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            self.assertEqual(diag.cmd_cool(d, types.SimpleNamespace(args=["stop"])), 0)
        self.assertEqual(d.ser.sent, b"cool stop\n")
        d = fake_drone(self.DONE.encode() + b"\n")
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(diag.cmd_cool(d, types.SimpleNamespace(args=[])), 0)
        self.assertEqual(d.ser.sent, b"cool\n")


class AutonomyAnalyzeTest(unittest.TestCase):
    SORTIE = [(20, 1, 1, 0, 0), (150, 2, 2, 2, 120), (300, 3, 3, 4, 330), (80, 4, 37, 5, 150), (30, 0, 22, 6, 0)]

    def test_dry_sortie_has_no_thrust_but_is_shown(self):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.analyze(auto_rows(self.SORTIE))
        out = buf.getvalue()
        self.assertIn("flight 9: autonomy with no thrust (DRY)", out)
        self.assertIn("ARMED(ARM, vz IDLE) +0.00s -> TAKEOFF(TAKEOFF, vz RAMP) +0.20s -> HOVER(HOLD, vz HOLD) +1.70s"
                      " -> LAND(HOVER_TIMEOUT, vz LAND) +4.70s -> IDLE(TOUCHDOWN, vz DONE) +5.50s", out)
        self.assertIn("highest height estimate 0.33 m", out)
        self.assertIn("sim plant", out)
        self.assertIn("flags seen: waiting sim", out)

    def test_bench_sortie_end_cause(self):
        rows = auto_rows(self.SORTIE, thrust=15000)
        a = diag.auto_summary(rows)
        self.assertEqual([s["state"] for s in a["steps"]], ["ARMED", "TAKEOFF", "HOVER", "LAND", "IDLE"])
        self.assertEqual((a["thrust_max"], a["vz_thrust_max"]), (15000, 41000))
        # The thrust went to 0 in LAND: the trigger is the one the flight summary keeps (autoReason).
        self.assertEqual(diag.flight_end_cause(rows), "AUTONOMY: LAND (HOVER_TIMEOUT)")
        self.assertEqual((rows[549]["thrust"], rows[550]["thrust"], rows[550]["autoState"]), (15000, 0, 0))
        rows[550].update(autoState=4, autoReason=37)                  # motors off at touchdown, still in LAND
        self.assertEqual(diag.flight_end_cause(rows), "AUTONOMY: LAND (HOVER_TIMEOUT)")
        rows[550].update(autoState=5, autoReason=26)                  # a cut: the closing sample says why
        self.assertEqual(diag.flight_end_cause(rows), "AUTONOMY: KILLED (TILT)")
        rows[550].update(autoState=4, autoReason=37)
        self.assertEqual(diag.flight_end_cause(rows), "AUTONOMY: LAND (HOVER_TIMEOUT)")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.analyze(rows)
        self.assertIn("ended: AUTONOMY: LAND (HOVER_TIMEOUT)", buf.getvalue())
        self.assertIsNone(diag.auto_summary([dict(r, autoState=0) for r in rows]))
        self.assertIsNone(diag.auto_summary([dict.fromkeys(diag.RECORD_FIELDS)]))


class FakeConsole:
    """usb_diag.c's replies to the 'auto' commands, in the forms WP1 prints (docs/autonomy-a2b.md sections 7.2
    and 13: appAgeMs, no inputAgeMs/gestureAgeMs; prevLocked/prevTransitions, no boots; guard names joined by
    '|'). AutoRepliesMatchFirmwareTest checks the keys against usb_diag.c when the tree has the A2b console."""

    REPLIES = {
        "auto status": "OK auto status state=IDLE mode=DRY reason=NONE vz=IDLE vzReason=NONE vzLand=NONE "
                       "guards=0x404(STILL|GESTURE) resetLocked=0 rtcValid=1 rst=poweron prev=IDLE/NONE/DRY "
                       "transitions=0 kill=0 session=0x00000001 hbAgeMs=40 appAgeMs=30 camAwake=0 hold=0 "
                       "camHeight=0 sim=0 waiting=0 waitLeftMs=0 airMs=0 stateMs=5000 vbat=4.050 vmin=4.000 "
                       "heap=21500 heapMin=15900 benchSrc=sim linkStack=psram mv=IDLE/- why=NONE end=NONE "
                       "mvSeq=0 prog=0/0/0 tgt=0/0/0 D=0 dFlags=0x00 hTarget=0 yawOff=0.00 battLow=0 airLeftDs=0 "
                       "mvSpeed=100 mvAcc=20 mvSettle=2500 mvGuards=30/35/25/20/40/40/80 klow=60 lat=50 wfwd=15 "
                       "mvView=323 holdRes=160 mvLatK=100 mvFfr=0",
        "auto get": "OK auto get autoon=1(nvs) appreq=1(default) autogest=1(default) vmin=4000(default) "
                    "autoair=6000(default) autowait=5000(default) autoaccel=2000(default) vzK=165000(default) "
                    "vzBair=-23(default) vzcaplo=1650(default) photocol=0(default) cvqlate=1(default) "
                    "cvqlates=40(default) cvt2k=0(default) cvt2r=124(default) mvspeed=100(default) "
                    "mvfloor=30(default) mvfstop=25(default) mvprogm=20(default) mvprogr=40(default) mvlatm=40(default) "
                    "mvnear=80(default) mvfix=59(default) mvacc=20(default) mvsettle=2500(default) cvhb=0(default) "
                    "mvklow=60(default) mvlat=50(default) mvwmax=15(default) mvview=0(default) vzest=1(default) "
                    "vzestms=0(default) holdres=160(default) mvlatk=100(default) mvffr=0(default)",
        "auto set autowait 4000": "ERR busy: autonomy active",
        "auto prof": "OK auto prof loops=120000 full=900 fastMaxUs=2 idleAvgUs=18.0 idleMaxUs=36 activeAvgUs=24.3 "
                     "activeMaxUs=95 activeLoops=19808 activeMs=20000 activeLoopsPerS=990.4 cmdLatMaxMs=2 "
                     "stabStackFree=1400 flashRefused=0 armsBlocked=0 upMs=120000 bypass=0x00 bypassSeen=0x00 "
                     "shiftRefused=0",
        "auto prof reset": "OK auto prof reset (at the next IDLE step)",
        "auto unlock": "OK auto unlock resetLocked=0 was=1 rst=software (other locks unchanged: plock=off rlocked=0 "
                       "motorsReady=1 estop=0 mguard=on mguardTripped=0)",
    }
    RTC = ["OK auto rtc locked=1 valid=1 rst=software(3) prev=IDLE/TOUCHDOWN/DRY prevLocked=0 prevTransitions=6 "
           "transitions=0",
           "T 0 ms=1000 from=IDLE to=ARMED reason=ARM vz=NONE",
           "T 1 ms=1800 from=ARMED to=TAKEOFF reason=TAKEOFF vz=NONE"]

    def __init__(self, replies=None):
        self.replies = dict(self.REPLIES, **(replies or {}))
        self.sent = []

    def command(self, cmd, timeout=3.0, until=("OK", "ERR")):
        self.sent.append(cmd)
        if cmd == "auto rtc":
            assert "END" in until
            return "END auto rtc n=2", list(self.RTC)
        return self.replies.get(cmd, "ERR unknown"), []

    def close(self):
        pass


class AutoPassthroughTest(unittest.TestCase):
    def run_auto(self, words, console=None):
        console = console or FakeConsole()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_auto(console, types.SimpleNamespace(args=words))
        return rc, buf.getvalue(), console.sent

    def test_status_set_and_rtc(self):
        rc, out, sent = self.run_auto(["status"])
        self.assertEqual((rc, sent), (0, ["auto status"]))
        self.assertIn("guards=0x404(STILL|GESTURE)", out)
        rc, out, _ = self.run_auto(["set", "autowait", "4000"])
        self.assertEqual(rc, 1)
        self.assertIn("ERR busy: autonomy active", out)
        rc, out, _ = self.run_auto(["rtc"])
        self.assertEqual(rc, 0)
        self.assertEqual(out.splitlines(), FakeConsole.RTC + ["END auto rtc n=2"])

    def test_get_is_saved_for_the_mac_tool(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "espfly", "auto_get.json")
            with mock.patch.object(diag, "AUTO_GET_CACHE", path):
                rc, out, _ = self.run_auto(["get"])
            self.assertEqual(rc, 0)
            self.assertIn("saved for espfly_auto.py", out)
            with open(path) as f:
                doc = json.load(f)
            self.assertEqual(doc["values"]["vmin"], {"value": "4000", "src": "default"})
            self.assertEqual(doc["values"]["autoon"], {"value": "1", "src": "nvs"})
            self.assertEqual(doc["values"]["vzBair"]["value"], "-23")
            import espfly_auto
            with mock.patch.object(espfly_auto, "DEFAULT_DIR", os.path.join(tmp, "espfly")):
                self.assertEqual(espfly_auto.cached_vmin()["v"], 4.0)
                self.assertEqual(espfly_auto.air_ms_limit()[0], 6000)     # autoair=6000(default)

    def test_set_refreshes_the_mac_tools_cache(self):
        """An accepted 'set' / 'reset' reads 'auto get' again: the Mac checks --wait-s 0 against the cached
        autowait (the verification of 73969ad)."""
        got = FakeConsole.REPLIES["auto get"].replace("autowait=5000(default)", "autowait=100000(nvs)")
        console = FakeConsole({"auto set autowait 100000": "OK auto set autowait=100000", "auto get": got,
                               "auto reset autowait": "OK auto reset autowait=5000"})
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "espfly", "auto_get.json")
            with mock.patch.object(diag, "AUTO_GET_CACHE", path):
                rc, out, sent = self.run_auto(["set", "autowait", "100000"], console)
                self.assertEqual((rc, sent), (0, ["auto set autowait 100000", "auto get"]))
                self.assertIn("saved again", out)
                with open(path) as f:
                    self.assertEqual(json.load(f)["values"]["autowait"], {"value": "100000", "src": "nvs"})
                import espfly_auto
                with mock.patch.object(espfly_auto, "DEFAULT_DIR", os.path.join(tmp, "espfly")):
                    self.assertEqual(espfly_auto.cached_setting("autowait")["value"], 100000)
                console.replies["auto get"] = FakeConsole.REPLIES["auto get"]
                rc, _, sent = self.run_auto(["reset", "autowait"], console)
                self.assertEqual((rc, sent[-2:]), (0, ["auto reset autowait", "auto get"]))
                with open(path) as f:
                    self.assertEqual(json.load(f)["values"]["autowait"]["value"], "5000")
            # refused (ERR): nothing read again
            rc, _, sent = self.run_auto(["set", "autowait", "4000"])
            self.assertEqual((rc, sent), (1, ["auto set autowait 4000"]))

    def prof(self, **fields):
        reply = FakeConsole.REPLIES["auto prof"]
        for k, v in fields.items():
            reply = re.sub(rf"\b{k}=\S+", f"{k}={v}", reply)
        return self.run_auto(["prof"], FakeConsole({"auto prof": reply}))

    def test_prof_go_no_go(self):
        rc, out, _ = self.run_auto(["prof"])
        self.assertEqual(rc, 0)
        self.assertIn("go/no-go: GO", out)
        rc, out, _ = self.prof(activeLoopsPerS="962.0")
        self.assertEqual(rc, 0)
        self.assertIn("go/no-go: GO", out)
        rc, out, _ = self.prof(activeLoopsPerS="935.0")
        self.assertEqual(rc, 1)
        self.assertIn("NO-GO (activeLoopsPerS < 940)", out)
        rc, out, _ = self.prof(activeLoopsPerS="0.0")
        self.assertEqual(rc, 0)
        self.assertIn("no active loops measured yet", out)
        rc, out, _ = self.run_auto(["prof", "reset"])
        self.assertEqual(rc, 0)
        self.assertNotIn("go/no-go", out)

    def test_prof_only_the_loop_rate_gates(self):
        """Review RT-1: activeAvgUs/activeMaxUs are wall-clock (preemption included): shown, never a NO-GO."""
        rc, out, _ = self.prof(activeAvgUs="48.7", activeMaxUs="2900")
        self.assertEqual(rc, 0)
        self.assertIn("go/no-go: GO", out)
        self.assertRegex(out, r"activeMaxUs\s+2900 info only")
        self.assertNotIn("FAIL", out)

    def test_prof_bisect_is_no_go_no_go(self):
        """docs/autonomy-bench.md A4b: with a bisect toggle on, the loop rate prices the toggle; no GO / NO-GO."""
        rc, out, _ = self.prof(activeLoopsPerS="931.6", bypass="0x05", bypassSeen="0x05")
        self.assertEqual(rc, 0)
        self.assertIn("with bypass gtc rec", out)
        self.assertIn("go/no-go: none: a bisect measurement (bypassSeen=0x05: gtc rec)", out)
        self.assertNotIn("NO-GO", out)
        self.assertNotIn("go/no-go: GO", out)
        rc, out, _ = self.prof(activeLoopsPerS="935.0", bypass="0x00")
        self.assertEqual(rc, 1)
        self.assertIn("NO-GO (activeLoopsPerS < 940)", out)
        # a firmware before bypassSeen: the live bypass decides
        reply = FakeConsole.REPLIES["auto prof"].replace(" bypassSeen=0x00", "").replace("bypass=0x00", "bypass=0x02")
        rc, out, _ = self.run_auto(["prof"], FakeConsole({"auto prof": reply}))
        self.assertEqual(rc, 0)
        self.assertIn("go/no-go: none: a bisect measurement (bypass=0x02: mki)", out)

    def test_prof_bisect_bit_cleared_before_the_read(self):
        """Review (fixer, 2026-10-14): a run measured with 'bypass gtc on', read after 'bypass gtc off' (live mask
        0), or a reading that mixes a bypassed sortie with normal ones (no 'auto prof reset' between them):
        bypassSeen keeps the toggle, so no GO for a number that is not the build's."""
        for live in ("0x00", "0x04"):
            with self.subTest(live=live):
                rc, out, _ = self.prof(activeLoopsPerS="958.0", bypass=live, bypassSeen="0x05")
                self.assertEqual(rc, 0)
                self.assertNotIn("go/no-go: GO", out)
                self.assertNotIn("NO-GO", out)
                self.assertIn("go/no-go: none: a bisect measurement (bypassSeen=0x05: gtc rec", out)
                self.assertEqual("cleared before this read" in out, live == "0x00")
        rc, out, _ = self.prof(activeLoopsPerS="958.0", bypass="0x00", bypassSeen="0x00")
        self.assertEqual(rc, 0)
        self.assertIn("go/no-go: GO", out)

    def test_prof_bypass_seen_firmware(self):
        """usb_diag.c: bypassSeen is set from the live mask on 'auto prof reset' and every toggle change ORs into
        it (the USB task only; nothing on the loop)."""
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            text = f.read()
        body = text[text.index("static void cmdAutoProf("):]
        body = body[:body.index("\n}\n")]
        reset = body[:body.index("autonomyProfileReset();")]
        self.assertIn("benchBypassSeen = benchBypassMask;", reset)
        self.assertIn("bypassSeen=0x%02x", body)
        apply_ = text[text.index("static void bypassApply("):]
        apply_ = apply_[:apply_.index("\n}\n")]
        self.assertIn("benchBypassSeen |= benchBypassMask;", apply_)
        self.assertEqual(len(re.findall(r"\bbenchBypassSeen\s*(?:\|)?=", text)), 2)
        self.assertEqual(diag.bypass_names("0x0f"), "gtc mki rec flow")
        self.assertEqual(diag.bypass_names(None), "")
        self.assertEqual(diag.bypass_names("x"), "")

    def test_bypass_command_keys(self):
        """The 'bypass' console command (usb_diag.c cmdBypass) and its CLI subcommand."""
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            text = f.read()
        fmt = c_format(text, "OK bypass=")
        self.assertIsNotNone(fmt)
        self.assertEqual(reply_keys(fmt), reply_keys("OK bypass=0x00 gtc=0 mki=0 rec=0 flow=0 (RAM only)"))
        for name, bit in diag.BYPASS_BITS:
            self.assertIn(f'{{ "{name}", BENCH_BYPASS_{name.upper()} }}', text)
        with open(os.path.join(FIRMWARE_SRC, "utils", "interface", "bench_bypass.h")) as f:
            hdr = f.read()
        for name, bit in diag.BYPASS_BITS:
            self.assertRegex(hdr, rf"#define BENCH_BYPASS_{name.upper()}\s+0x{bit:02X}u")
        rc, out, sent = self.run_auto_like_bypass(["gtc", "on"])
        self.assertEqual((rc, sent), (0, ["bypass gtc on"]))

    def test_holdk_command_keys(self):
        """The 'holdk' console command (usb_diag.c cmdHoldK, docs/autonomy-a2b.md 20) and its CLI subcommand."""
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            text = f.read()
        fmt = c_format(text, "OK holdk kmax=")
        self.assertIsNotNone(fmt)
        reply = "OK holdk kmax=2.50 src=default s=2.31 target=2.50 G=31.2 D=3.20 translated=0 hold=on"
        self.assertEqual(reply_keys(fmt), reply_keys(reply))
        self.assertIn('strcmp(argv[0], "holdk") == 0', text)
        for words, cmd in (([], "holdk"), (["set", "1"], "holdk set 1"), (["save"], "holdk save")):
            with self.subTest(cmd):
                con = FakeConsole({cmd: reply})
                buf = io.StringIO()
                with contextlib.redirect_stdout(buf):
                    rc = diag.cmd_simple("holdk")(con, types.SimpleNamespace(args=words))
                self.assertEqual((rc, con.sent), (0, [cmd]))
        con = FakeConsole({"holdk set 4": "ERR holdk set: usage holdk set <1..3>"})
        with contextlib.redirect_stdout(io.StringIO()):
            rc = diag.cmd_simple("holdk")(con, types.SimpleNamespace(args=["set", "4"]))
        self.assertEqual(rc, 1)
        with open(diag.__file__) as f:
            cli = f.read()
        self.assertIn('sub.add_parser("holdk"', cli)
        self.assertIn('"holdk": cmd_simple("holdk")', cli)

    def run_auto_like_bypass(self, words):
        con = FakeConsole({"bypass gtc on": "OK bypass=0x01 gtc=1 mki=0 rec=0 flow=0 (RAM only, 0 at boot; a bisect "
                                            "build state: not a go/no-go)"})
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = diag.cmd_simple("bypass")(con, types.SimpleNamespace(args=words))
        return rc, buf.getvalue(), con.sent

    def test_prof_stack_is_its_own_check(self):
        rc, out, _ = self.prof(stabStackFree="900")
        self.assertEqual(rc, 1)
        self.assertIn("go/no-go: GO", out)
        self.assertRegex(out, r"stabStackFree\s+900 >= 1024\s+FAIL")
        rc, out, _ = self.prof(stabStackFree="-1")
        self.assertEqual(rc, 0)
        self.assertRegex(out, r"stabStackFree\s+not sampled yet")

    def test_unlock_passes_through(self):
        """'auto unlock' (user decision 2026-10-12, evening) goes to the USB console as typed; an ERR (not IDLE,
        a motor running) is a failure."""
        rc, out, sent = self.run_auto(["unlock"])
        self.assertEqual((rc, sent), (0, ["auto unlock"]))
        self.assertIn("resetLocked=0 was=1 rst=software (other locks unchanged:", out)
        for reply in ("ERR busy: autonomy active", "ERR auto unlock: a motor output is nonzero"):
            with self.subTest(reply):
                rc, out, sent = self.run_auto(["unlock"], FakeConsole({"auto unlock": reply}))
                self.assertEqual((rc, sent, out.strip()), (1, ["auto unlock"], reply))
        console = FakeConsole()
        with mock.patch.object(diag, "Drone", lambda port: console), \
                mock.patch.object(sys, "argv", ["espfly_diag.py", "--port", "/dev/null", "auto", "unlock"]), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(diag.main(), 0)
        self.assertEqual(console.sent, ["auto unlock"])

    def test_main_routes_auto(self):
        console = FakeConsole()
        with mock.patch.object(diag, "Drone", lambda port: console), \
                mock.patch.object(sys, "argv", ["espfly_diag.py", "--port", "/dev/null", "auto", "status"]), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(diag.main(), 0)
        self.assertEqual(console.sent, ["auto status"])


FIRMWARE_SRC = os.path.join(os.path.dirname(__file__), "..", "..", "components", "core", "crazyflie")
C_LITERAL = re.compile(r'\s*"((?:[^"\\]|\\.)*)"')


def c_format(text, prefix):
    """The format string of usb_diag.c's out("<prefix>...") call (its adjacent literals joined), or None."""
    i = text.find('out("' + prefix)
    if i < 0:
        return None
    lits, j = [], i + len("out(")
    while True:
        m = C_LITERAL.match(text, j)
        if m is None:
            return "".join(lits)
        lits.append(m.group(1))
        j = m.end()


def reply_keys(line):
    return re.findall(r"(\w+)=", line)


class AutoRepliesMatchFirmwareTest(unittest.TestCase):
    """The fake console's 'auto' replies carry the keys the firmware prints, in its order: usb_diag.c's
    formats and auto_glue_core.c's settings table. Checked when this tree has the A2b console (WP1); a tree
    before it is skipped."""

    def setUp(self):
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            self.text = f.read()
        if 'out("OK auto status' not in self.text:
            self.skipTest("usb_diag.c has no A2b 'auto status' (a tree before WP1)")

    def test_status_prof_rtc_keys(self):
        for prefix, reply in (("OK auto status", FakeConsole.REPLIES["auto status"]),
                              ("OK auto prof loops", FakeConsole.REPLIES["auto prof"]),
                              ("OK auto rtc", FakeConsole.RTC[0]),
                              ("OK auto unlock", FakeConsole.REPLIES["auto unlock"]),
                              ("T %u ms=", FakeConsole.RTC[1])):
            with self.subTest(prefix):
                fmt = c_format(self.text, prefix)
                self.assertIsNotNone(fmt, f"no out(\"{prefix}...\") in usb_diag.c")
                self.assertEqual(reply_keys(reply), reply_keys(fmt))

    def test_unlock_is_usb_only(self):
        """The reset-lock's unlock (autonomyUsbUnlock) has one caller, the USB console (usb_diag.c, fed only by
        the USB-Serial-JTAG port), and the Mac's Wi-Fi tool has no unlock of any kind."""
        callers = set()
        tree = os.path.join(FIRMWARE_SRC, "..", "..", "..")
        for root, _, files in (w for top in ("components", "main") for w in os.walk(os.path.join(tree, top))):
            for name in files:
                if name.endswith((".c", ".h")):
                    with open(os.path.join(root, name), errors="replace") as f:
                        if re.search(r"\bautonomyUsbUnlock\s*\(", f.read()):   # a call, declaration or definition
                            callers.add(name)
        if not callers:
            self.skipTest("no autonomyUsbUnlock in this tree")
        self.assertEqual(callers, {"autonomy.h", "autonomy.c", "usb_diag.c"})
        self.assertIn('"unlock"', self.text)
        with open(os.path.join(os.path.dirname(__file__), "..", "espfly_auto.py")) as f:
            self.assertNotRegex(f.read(), r"(?i)unlock")

    def test_get_keys(self):
        path = os.path.join(FIRMWARE_SRC, "utils", "src", "auto_glue_core.c")
        if not os.path.exists(path):
            self.skipTest("no auto_glue_core.c")
        with open(path) as f:
            names = re.findall(r'\[AUTO_SET_\w+\]\s*=\s*\{"(\w+)"', f.read())
        self.assertTrue(names)
        self.assertEqual(list(diag.parse_auto_get(FakeConsole.REPLIES["auto get"])), names)

    def test_get_keys_2026_10_22(self):
        """The VIEW_FAST timer, the EST_RANGE guard and the hold's scene gate residual limit as settings (2026-10-22):
        parsed like any other key, the drone's defaults as the native profile reports them."""
        got = diag.parse_auto_get(FakeConsole.REPLIES["auto get"])
        self.assertEqual({k: got[k] for k in ("mvview", "vzest", "vzestms", "holdres")},
                         {"mvview": {"value": "0", "src": "default"}, "vzest": {"value": "1", "src": "default"},
                          "vzestms": {"value": "0", "src": "default"}, "holdres": {"value": "160", "src": "default"}})
        self.assertIn("mvView=323 holdRes=160", FakeConsole.REPLIES["auto status"])

    def test_get_keys_slide(self):
        """The left slide's two settings (2026-10-22): mvlatk (the hold's lateral position gain on a MOVE's frames,
        %) and mvffr (a feedforward roll per m/s of forward speed, x0.1 deg): parsed like any other key, defaults =
        the hold and the motions as before; auto status ends with the values latched at the last ARM."""
        got = diag.parse_auto_get(FakeConsole.REPLIES["auto get"])
        self.assertEqual({k: got[k] for k in ("mvlatk", "mvffr")},
                         {"mvlatk": {"value": "100", "src": "default"}, "mvffr": {"value": "0", "src": "default"}})
        self.assertTrue(FakeConsole.REPLIES["auto status"].endswith("mvLatK=100 mvFfr=0"))


CAL_DIR = "../../firmware-backups/cal"


class TrimCheckTest(unittest.TestCase):
    """The post-flight trim check (2026-10-16, docs/autonomy-a2b.md 27.5): flags a mean hold > 0.8 deg in the first
    10 s of HOVER or a sideways accel > 15 mg from normal, suggests an atrim with its arithmetic, never sets one."""

    @staticmethod
    def rows(hold_r, hold_p, ay, n=1500, auto_state=3, mv=0, flags=0x03):
        # the recorder's 100 Hz rows: tickMs counts loops, 10 a row (~955 loops a second)
        return [{"flightId": 1, "tickMs": 1000 + 10 * i, "autoState": auto_state, "mvState": mv, "holdFlags": flags,
                 "holdRoll_cdeg": int(hold_r * 100), "holdPitch_cdeg": int(hold_p * 100), "ay_mg": ay, "ax_mg": 3}
                for i in range(n)]

    def test_flags_and_arithmetic(self):
        c = diag.trim_check(self.rows(1.11, 0.20, 9.6), trim=(-0.60, 0.00))
        self.assertAlmostEqual(c["seconds"], 10.0, delta=0.1)
        self.assertEqual(len(c["flags"]), 2, c["flags"])           # roll hold and the sideways accel
        self.assertIn("roll hold +1.11", c["flags"][0])
        self.assertIn("+21.6 from normal", c["flags"][1])
        sr = c["suggest"]["roll"]
        self.assertEqual((sr["now"], sr["full"], sr["step"]), (-0.60, 0.5, -0.05))
        self.assertIn("-0.60 + 1.11 = +0.51 (full)", sr["arithmetic"])
        self.assertNotIn("pitch", c["suggest"])
        self.assertAlmostEqual(c["ay_lean_deg"], 21.6 / 17.45, places=2)

    def test_quiet_flight(self):
        c = diag.trim_check(self.rows(-0.33, 0.11, -14.5), trim=(-0.60, 0.00))
        self.assertEqual((c["flags"], c["suggest"]), ([], {}))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            diag.print_trim_check(1, c)
        self.assertIn("OK: nothing to change", buf.getvalue())

    def test_window_and_selection(self):
        # only HOVER, outside motions, the hold applied; the first 10 s: a late lean does not count
        rows = self.rows(0.1, 0.1, -12.0, n=955) + [dict(r, tickMs=r["tickMs"] + 9550, holdRoll_cdeg=200)
                                                    for r in self.rows(2.0, 0.1, -12.0, n=955)]
        self.assertEqual(diag.trim_check(rows)["flags"], [])
        rows = [dict(r, mvState=1) for r in self.rows(2.0, 0.0, -12.0)]     # a MOVE running: nothing counted
        self.assertIn("no check", diag.trim_check(rows)["note"])
        self.assertIsNone(diag.trim_check(self.rows(2.0, 0.0, 0.0, auto_state=4)))   # no HOVER
        c = diag.trim_check(self.rows(0.0, -0.9, -12.0))                  # pitch, the trim unknown
        self.assertIn("now -0.90", c["suggest"]["pitch"]["full"])

    def test_flights_note_and_txt(self):
        r = {"boot": 5, "id": 1, "hold_pct": 98.0, "hold_roll": 1.3, "hold_pitch": 1.1, "atrim": [-0.60, 0.00]}
        note = diag.flights_trim_note(r)
        self.assertIn("roll hold +1.3", note)
        self.assertIn("-0.60 + 0.5 x 1.3 = +0.05", note)
        self.assertIn("pitch hold +1.1", note)
        self.assertIn("never set here", note)
        self.assertIsNone(diag.flights_trim_note(dict(r, hold_roll=0.3, hold_pitch=-0.5)))
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
            f.write("boot  id  time   air     battery  cam hold coast tilt rms/max gyro rms/max  motors FR/BR/BL/FL % "
                    "hold p/r deg trim p/r deg  gyro drift  end\n"
                    "   5   1 63.6s 62.1s 4.15->3.29 100%  98%   16%     2.3/33.7       32/786           67/73/70/72"
                    "    +1.1/+1.3  +0.00/-0.60 -0.03,+0.09  AUTO(TILT)\n")
        try:
            self.assertEqual(diag.flights_txt_trims(f.name), {1: (-0.60, 0.00)})
        finally:
            os.unlink(f.name)

    def test_no_drone_needed(self):
        rows = self.rows(1.2, 0.0, 10)
        with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
            keys = list(rows[0])
            f.write(",".join(keys) + "\n")
            for r in rows:
                f.write(",".join(str(r[k]) for k in keys) + "\n")
        try:
            buf = io.StringIO()
            with mock.patch.object(diag, "find_port", side_effect=AssertionError("no port")), \
                    mock.patch.object(sys, "argv", ["espfly_diag.py", "trimcheck", f.name, "--trim", "-0.6", "0",
                                                    "--json"]), contextlib.redirect_stdout(buf):
                self.assertEqual(diag.main(), 0)
            out = json.loads(buf.getvalue())
            self.assertEqual(out["1"]["suggest"]["roll"]["step"], 0.0)
        finally:
            os.unlink(f.name)

    @unittest.skipUnless(os.path.isdir(CAL_DIR), "no flight logs")
    def test_logged_flights(self):
        def check(name, fid):
            with open(os.path.join(CAL_DIR, name)) as f:
                fr = [r for r in diag.read_rows(f) if r["flightId"] == fid]
            return diag.trim_check(fr, trim=(-0.60, 0.00))
        f1 = check("flight-2026-10-16-r1-f1.csv", 1)
        self.assertEqual(len(f1["flags"]), 3, f1)                         # roll, pitch, ay
        self.assertEqual(f1["suggest"]["roll"]["step"], -0.05)
        for name, fid in (("flight-2026-10-15-p1-height.csv", 5), ("flight-2026-10-15-fastcheck.csv", 6),
                          ("flight-2026-10-16-ab-7a78e1b.csv", 1)):
            with self.subTest(name=name):
                self.assertEqual(check(name, fid)["flags"], [])



class ColourSplitTest(unittest.TestCase):
    """Always-on colour (docs/autonomy-a2b.md 28): the status note, and the offline bench check of a grey and a
    colour photo (docs/autonomy-bench.md N'1.4-N'1.5)."""

    def test_cam_split_note(self):
        ok = ("OK status fw=x camPix=yuyv-split loops=1 camSplit=ok camSplitArm=6 camSplitDone=6 camSplitRearm=1 "
              "camSplitCancel=0 camSplitRestart=0 imuTempC=40.25 bypass=0x00 diagStackFree=2000")
        n = diag.cam_split_note(ok)
        self.assertIn("colour: always on (camPix=yuyv-split)", n)
        self.assertIn("armed 6, filled 6, re-armed 1, cancelled 0, restarted 0", n)
        self.assertIn("WARNING: camSplit=bad", diag.cam_split_note("OK status camPix=gray camSplit=bad camSplitArm=0"))
        self.assertIsNone(diag.cam_split_note("OK status camPix=gray camSplit=off camSplitArm=0"))
        self.assertIsNone(diag.cam_split_note("OK status camPix=gray loops=3"))   # before 2026-10-16

    def test_status_prints_the_split(self):
        """usb_diag.c's 'status': camPix=yuyv-split|gray and the camSplit fields, before diagStackFree."""
        with open(os.path.join(FIRMWARE_SRC, "modules", "src", "usb_diag.c")) as f:
            text = f.read()
        i = text.index('"OK status fw=')
        body = text[i:text.index("\n}\n", i)]
        self.assertIn('"yuyv-split" : "gray"', body)
        for k in ("camSplit=%s", "camSplitArm=%u", "camSplitDone=%u", "camSplitRearm=%u", "camSplitCancel=%u",
                  "camSplitRestart=%u"):
            self.assertIn(k, body)
        self.assertLess(body.index("camSplit=%s"), body.index("diagStackFree=%u"))

    def frames(self, shift=0, clip=False):
        grey = bytes((x * 3 + y * 2) % 251 for y in range(120) for x in range(160))
        ys = bytes(max(16, min(235, v)) if clip else min(255, v + shift) for v in grey)
        u, v = 126, 130
        yuyv = bytearray()
        for i in range(0, len(ys), 2):
            yuyv += bytes((ys[i], u, ys[i + 1], v))
        return grey, bytes(yuyv)

    def test_colour_check_numbers(self):
        grey, yuyv = self.frames()
        r = diag.colour_check(grey, yuyv[0::2], (yuyv[1::4], yuyv[3::4]), (128.0, 127.0, 126.0), white=True)
        self.assertEqual((r["y_mad"], r["grey_max"], r["colour_max"]), (0.0, 250, 250))   # >= 250: passes
        self.assertTrue(r["y_identity_pass"])
        self.assertEqual((r["mean_u"], r["mean_v"]), (126.0, 130.0))
        self.assertTrue(r["white_pass"])
        self.assertIsNone(r["orange_pass"])
        # 16-235 clipping in YUV mode (the reviewers' case): max 235 < 250, the min moves: a hard fail
        _, clipped = self.frames(clip=True)
        r = diag.colour_check(grey, clipped[0::2])
        self.assertFalse(r["y_identity_pass"])
        self.assertEqual(r["colour_max"], 235)
        # a 3-level offset: over the 2-level mean
        _, off = self.frames(shift=3)
        self.assertFalse(diag.colour_check(grey, off[0::2])["y_identity_pass"])
        # a cast: R/G/B spread 30
        r = diag.colour_check(grey, yuyv[0::2], (yuyv[1::4], yuyv[3::4]), (150.0, 128.0, 120.0), white=True,
                              orange=True)
        self.assertFalse(r["white_pass"])
        self.assertTrue(r["orange_pass"])

    def test_colourcheck_on_saved_photos(self):
        from PIL import Image
        grey, yuyv = self.frames()
        with tempfile.TemporaryDirectory() as tmp:
            def save(name, color, data):
                base = os.path.join(tmp, name)
                doc = {"width": 160, "height": 120, "color": color, "flippedVertical": True,
                       "files": {"png": base + ".png", "json": base + ".json"}}
                if color:
                    with open(base + ".yuyv", "wb") as f:
                        f.write(data)
                    doc["files"]["raw"] = base + ".yuyv"
                    Image.new("RGB", (160, 120), (128, 128, 126)).save(base + ".png")
                else:   # saved upright: the rows reversed
                    flipped = b"".join(data[r * 160:(r + 1) * 160] for r in range(119, -1, -1))
                    Image.frombytes("L", (160, 120), flipped).save(base + ".png")
                with open(base + ".json", "w") as f:
                    json.dump(doc, f)
                return base
            g, c = save("g", False, grey), save("c", True, yuyv)
            y, _, ch = diag.photo_luma(g + "_x4.png")
            self.assertEqual(y, grey)          # the grey photo's Y in the order sent
            self.assertIsNone(ch)
            y, _, ch = diag.photo_luma(c + ".json")
            self.assertEqual(y, yuyv[0::2])
            args = types.SimpleNamespace(grey=g + ".png", colour=c + ".yuyv", white=True, orange=False, json=False)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rc = diag.cmd_colourcheck(None, args)
            self.assertEqual(rc, 0, buf.getvalue())
            self.assertIn("Y identity: mean |grey - colour| 0.00", buf.getvalue())
            self.assertIn("white sheet: U and V in 124-132, R/G/B within 10: PASS", buf.getvalue())
            # the photos the wrong way round: refused
            with self.assertRaises(SystemExit):
                diag.cmd_colourcheck(None, types.SimpleNamespace(grey=c + ".png", colour=g + ".png", white=False,
                                                                 orange=False, json=False))
            os.remove(c + ".yuyv")
            with self.assertRaises(SystemExit):
                diag.photo_luma(c + ".png")   # an older Mac tool's colour photo: no raw bytes


if __name__ == "__main__":
    unittest.main()
