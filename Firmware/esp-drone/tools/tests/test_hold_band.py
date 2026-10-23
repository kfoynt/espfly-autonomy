"""Host tests for tools/hold_band.py (docs/autonomy-bench.md J3 3). Run: python3 -m unittest discover -s tools/tests"""
import contextlib
import csv
import io
import math
import os
import random
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import hold_band as hb  # noqa: E402

TRK = 0x07      # cvFlags: airborne, healthy, gain tracked
UNTRK = 0x03
MOVE_RUN = (hb.MVK_MOVE << 4) | hb.MV_RUN
YAW_RUN = (hb.MVK_YAW << 4) | hb.MV_RUN
YAW_DONE = (hb.MVK_YAW << 4) | 4


def sine(f, amp, n, dt=0.01):
    return [amp * math.sin(2 * math.pi * f * k * dt) for k in range(n)]


class BandTest(unittest.TestCase):
    def test_passband_and_stopbands(self):
        n = 6000
        for f, lo, hi in ((0.8, 0.78, 0.87), (0.05, 0.0, 0.2), (8.0, 0.0, 0.2)):
            out = hb.band(sine(f, 1.0, n))[3000:]
            gain = (max(out) - min(out)) / 2.0
            self.assertGreaterEqual(gain, lo, f)
            self.assertLessEqual(gain, hi, f)

    def test_constant_is_zero(self):
        self.assertTrue(all(abs(v) < 1e-12 for v in hb.band([1.7] * 500)))


class GainScaleTest(unittest.TestCase):
    """The same cases as test_camera_hold.c testGainTarget (camHoldGainTarget)."""

    def test_target(self):
        g = hb.GainScale(2.5)
        self.assertEqual(g.target(0, 0, TRK, 320), 1.0)
        self.assertEqual(g.target(2, 0, UNTRK, 800), 1.0)
        self.assertEqual(g.target(2, 0, 0x14, 185), 1.0)          # on the ground: the last flight's G ignored
        self.assertEqual(g.g, 0.0)
        self.assertEqual(g.target(2, 0, TRK, 320), 2.5)
        self.assertAlmostEqual(g.target(3, 0, TRK, 400), 2.0)
        self.assertEqual(g.target(3, 0, TRK, 1000), 1.0)
        self.assertEqual(g.target(3, 0, TRK, 100), 1.0)           # out of range: G 100 kept
        self.assertAlmostEqual(g.target(3, 0, TRK, 500), 1.6)
        self.assertAlmostEqual(g.target(3, 0, UNTRK, 300), 1.6)
        self.assertEqual(g.target(4, 0, UNTRK, 0), 1.0)           # LAND: 1, the latch kept
        self.assertEqual(g.g, 50.0)
        self.assertEqual(g.target(5, 0, TRK, 500), 1.0)           # KILLED: cleared
        self.assertEqual(g.g, 0.0)

    def test_move_and_yaw(self):
        g = hb.GainScale(2.5)
        self.assertEqual(g.target(3, 0, TRK, 320), 2.5)
        self.assertEqual(g.target(3, MOVE_RUN, TRK, 320), 1.0)
        self.assertEqual(g.target(3, YAW_DONE, TRK, 320), 1.0)    # 1 for the rest of the sortie
        self.assertEqual(g.target(1, 0, TRK, 320), 1.0)           # ARM clears
        self.assertEqual(g.target(2, 0, TRK, 320), 2.5)
        self.assertEqual(g.target(3, YAW_RUN, TRK, 320), 1.0)     # a turn forgets G
        self.assertEqual(g.target(3, YAW_DONE, UNTRK, 320), 1.0)
        self.assertAlmostEqual(g.target(3, YAW_DONE, TRK, 400), 2.0)

    def test_kmax(self):
        self.assertEqual(hb.GainScale(1.0).target(3, 0, TRK, 200), 1.0)
        self.assertEqual(hb.GainScale(float('nan')).target(3, 0, TRK, 200), 1.0)
        self.assertEqual(hb.GainScale(9.0).target(3, 0, TRK, 200), hb.SCALE_MAX)

    def test_slew(self):
        g = hb.GainScale(2.5)
        n = 0
        while g.s < 2.5 and n < 200:
            g.step(3, 0, TRK, 320, 0.01)
            n += 1
        self.assertEqual(n, 75)                                   # 1.5 at 2 /s: 0.75 s
        g.step(3, 0, TRK, 320, 5.0)
        self.assertEqual(g.s, 2.5)
        g.step(4, 0, TRK, 320, 5.0)                               # a gap counts at most 0.3 s
        self.assertAlmostEqual(g.s, 1.9)


class MoveGainTest(unittest.TestCase):
    """As test_camera_hold.c testGainRate / testGainMove: 0.75 /s once a MOVE ran in HOVER; during a MOVE toward the
    scene the gain its reference's scene distance needs, at most the start's."""

    def test_rate(self):
        g = hb.GainScale(2.5)
        self.assertEqual(g.rate(3), 2.0)
        for _ in range(100):
            g.step(3, 0, TRK, 320, 0.01)
        self.assertEqual(g.s, 2.5)
        n = 0
        while g.s > 1.0 and n < 400:
            g.step(3, MOVE_RUN, TRK, 320, 0.01)          # no reference: the plain target (1) at 0.75 /s
            n += 1
        self.assertIn(n, (200, 201))                     # 1.5 at 0.75 /s: 2.0 s
        self.assertEqual(g.rate(4), 2.0)                 # LAND: 2 /s

    def test_move_toward_the_scene(self):
        g = hb.GainScale(2.5)
        for _ in range(100):
            g.step(3, 0, TRK, 200, 0.01)                 # G 20: D 4.99 m, s 2.5
        self.assertEqual(g.s, 2.5)
        g.step(3, MOVE_RUN, TRK, 200, 0.01, 0.0)
        self.assertAlmostEqual(g.d_low, hb.FY_PX / 20.0)
        self.assertAlmostEqual(g.s, 2.4925)              # not yet along (TRACK_MIN): the plain 1 at 0.75 /s
        g.step(3, MOVE_RUN, TRK, 200, 0.01, 0.02)
        self.assertAlmostEqual(g.s, 2.5)                 # along: D_ref ~4.8 m needs 3.9 > 2.5: the start's 2.5
        tgt = g.move_target(1.0, 3, MOVE_RUN, 0.6 * math.log(4.99 / 2.0))   # D_ref 2.0 m: s 1.6
        self.assertAlmostEqual(tgt, 2.0 * hb.G_REF / hb.FY_PX, places=3)
        self.assertEqual(g.move_target(1.0, 4, MOVE_RUN, 0.2), 1.0)        # LAND: no
        g.move_target(1.0, 3, (hb.MVK_MOVE << 4) | 4, 0.0)                 # the move DONE
        self.assertEqual(g.move_target(1.0, 3, MOVE_RUN, -0.1), 1.0)       # a back move: never along


class VerdictTest(unittest.TestCase):
    def setUp(self):
        random.seed(7)

    def noise(self, rms, n=4000):
        # the logged hold's character: a random walk of 15 Hz steps, band-limited by the filter
        x, v, out = 0.0, 0.0, []
        for k in range(n):
            if k % 7 == 0:
                v = random.gauss(0.0, 1.0)
            x = 0.97 * x + 0.03 * v
            out.append(x)
        b = hb.band(out)
        scale = rms / hb.std(b[500:])
        return [scale * v for v in b[500:]]

    def test_noise_times_s_is_ok(self):
        for rms_s1 in (0.03, 0.06, 0.10):
            for s in (1.0, 2.0, 2.5):
                b = self.noise(rms_s1 * s)
                rms, p2p, rms_s, word = hb.verdict(b, b, s)
                self.assertEqual(word, 'OK', (rms_s1, s, rms, p2p))

    def test_limit_cycle_is_rocking(self):
        b = hb.band(sine(0.7, 2.4, 4000))[1000:]
        self.assertEqual(hb.verdict(b, b, 2.5)[3], 'ROCKING')
        b = hb.band(sine(0.7, 0.75, 4000))[1000:]                # rms ~0.43 deg
        self.assertEqual(hb.verdict(b, b, 2.5)[3], 'ROCKING')

    def test_noisy_camera(self):
        b = self.noise(0.19)
        self.assertEqual(hb.verdict(b, b, 1.0)[3], 'NOISY')


def write_csv(path, roll, extra_pitch=None, move_at=None):
    """ARMED 1 s, TAKEOFF 3 s (G 32 tracked in the air from 1.5 s), HOVER 30 s, LAND 2 s; 100 Hz. move_at: a MOVE
    forward runs from this row on (5 s, the reference to ln 0.3, then DONE)."""
    cols = ['idx', 'tickMs', 'flightId', 'autoState', 'mvState', 'cvFlags', 'cvGain_dpm', 'holdRoll_cdeg',
            'holdPitch_cdeg', 'mvRefFwd_e4']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        k = 0
        for st, dur in ((1, 1.0), (2, 3.0), (3, 30.0), (4, 2.0)):
            for i in range(int(dur * 100)):
                air = st >= 2 and (st > 2 or i >= 50)
                flags = (TRK if (st > 2 or i >= 150) else 0x01) if air else 0x10
                r = roll[k] if k < len(roll) else 0.0
                p = (extra_pitch[k] if extra_pitch and k < len(extra_pitch) else 0.0)
                mv, ref = 0, 0
                if move_at is not None and st == 3 and k >= move_at:
                    run = k - move_at < 500
                    mv = (hb.MVK_MOVE << 4) | (hb.MV_RUN if run else 4)
                    ref = int(round(3000 * min(1.0, (k - move_at) / 500.0)))
                row = [k, 1000 + 10 * k, 3, st, mv, flags, 320, int(round(r * 100)), int(round(p * 100)), ref]
                w.writerow(row)
                w.writerow(row)
                k += 1
            # a duplicate tick per row (the dump's overlap) is dropped by load()
    return path


class AnalyseTest(unittest.TestCase):
    def test_end_to_end(self):
        random.seed(3)
        n = 3600
        roll = [0.25 * random.gauss(0.0, 1.0) for _ in range(n)]
        with tempfile.TemporaryDirectory() as d:
            p = write_csv(os.path.join(d, 'f.csv'), roll)
            r = hb.analyse(hb.load(p, 3))
            self.assertAlmostEqual(r['s_mean'], 2.5, places=6)
            self.assertAlmostEqual(r['t0'], 2.0, places=2)
            self.assertGreater(r['t1'], 29.5)
            self.assertEqual(r['why'], 'the end of HOVER')
            self.assertEqual(r['verdict'], 'OK')
            self.assertEqual(hb.analyse(hb.load(p, 3), kmax=1.0)['s_mean'], 1.0)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(hb.main([p, '3']), 0)
            rock = [v + 2.0 * math.sin(2 * math.pi * 0.6 * k * 0.01) for k, v in enumerate(roll)]
            p2 = write_csv(os.path.join(d, 'g.csv'), rock)
            self.assertEqual(hb.analyse(hb.load(p2, 3))['verdict'], 'ROCKING')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(hb.main([p2, '3']), 1)
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(hb.main([p2, '9']), 2)

    def test_moves_window(self):
        """--moves: the window runs through the motions to the end of HOVER; without it, to the first one."""
        random.seed(5)
        roll = [0.2 * random.gauss(0.0, 1.0) for _ in range(3600)]
        with tempfile.TemporaryDirectory() as d:
            p = write_csv(os.path.join(d, 'm.csv'), roll, move_at=1400)
            r = hb.analyse(hb.load(p, 3))
            self.assertEqual(r['why'], 'the first motion')
            self.assertLess(r['t1'], 10.5)
            r = hb.analyse(hb.load(p, 3), moves=True)
            self.assertEqual(r['why'], 'the end of HOVER')
            self.assertGreater(r['t1'], 29.5)
            self.assertLess(r['s_min'], 2.5)             # s falls once the MOVE ran
            self.assertEqual(r['verdict'], 'OK')


if __name__ == '__main__':
    unittest.main()
