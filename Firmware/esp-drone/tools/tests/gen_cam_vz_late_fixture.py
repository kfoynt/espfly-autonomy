#!/usr/bin/env python3
"""Fixture for the late offset noise's closed-loop tests (tools/tests/test_cam_vz_late.c).

The fault tests of the height study 2026-10-15 (section 6.4) splice the real
camera of two 2026-10-13 interactive sorties into the 10-15 hovers. This
writes what those tests need from the recorder logs (not in the repository)
to tools/tests/fixtures/cam_vz_late.txt, so the C test runs without them:

    python3 tools/tests/gen_cam_vz_late_fixture.py [CSV_DIR]

CSV_DIR: the flight-*.csv recorder files (default ../../firmware-backups/cal).

1. The accelerometer's in-flight offset of the three 10-15 hovers the study
   reconstructed (p1 fid 5, p2 fid 3, tiger fid 2): beta(t) = the 10 s
   centred mean of the world-up specific force over the hover (HOVER + 3 s to
   the hover's end or its arrival on a surface, the window clipped to it) minus
   the take-off's ground baseline g0 (the median of the recorder's ground rows
   lift - 128 .. lift - 5, as test_cam_vz_replay_ih.c), every 0.1 s from the
   lift-off; before HOVER + 3 s its first value, after the hover's end the
   linear trend of its last 25 s (the study's prep.py), to 125 s.
2. The camera's error of two 2026-10-13 interactive sorties, per frame: the
   vertical image motion with the rotation removed (flowcal1 constants, kyC
   included) minus what the controller's own speed (vzV) says it should have
   been, dyRes + G (h(t1) - h(t0)), G the observer's logged gain, h the
   integral of vzV, the window [t1 - dt, t1] with t1 = capture + 15 ms (the
   20 MHz camera clock of that day). Every disagreement with the controller
   counts as camera error, the study's conservative choice for harm tests.
     F1 (fid 1, the person, then the in-air contact): HOVER + 3 s to the hover's
        end - 2 s (the study's window: before the contact);
     F2 (fid 2, the bright window filling half the view): every frame from the
        lift-off + 1.5 s (the camera wake) to the touchdown.
   Frames with flowOk 0 are kept as such.

Format ('#' comment lines):
  B name G_px_per_m n    then n offsets in 1e-5 g, 0.1 s apart from the lift-off
  S name n               then n lines: t_s dt_us ok err_cpx (t from the segment's first frame)
"""
import csv
import os
import sys

import numpy as np

KY = (1.2836, 99.790, -2.7643, 0.0294)   # cam_vz_obs.c kyGx, kyGy, kyGz, kyC (flowcal1.json)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fixtures', 'cam_vz_late.txt')
BETA = [  # name, file, fid, G (the study's table, px/m)
    ('p1', 'flight-2026-10-15-p1-height.csv', 5, 25.0),
    ('p2', 'flight-2026-10-15-p2-height.csv', 3, 56.0),
    ('tiger', 'flight-2026-10-15-tiger.csv', 2, 33.0),
]
SEG = [('F1', 'flight-2026-10-13-interactive.csv', 1), ('F2', 'flight-2026-10-13-interactive.csv', 2)]


def load(path, fid):
    with open(path) as f:
        r = csv.reader(f)
        hdr = next(r)
        rows = [list(map(int, x)) for x in r if len(x) == len(hdr)]
    a = np.array(rows, dtype=np.int64)
    d = {h: a[:, i] for i, h in enumerate(hdr)}
    idx = np.where(d['flightId'] == fid)[0]
    s = {k: v[idx] for k, v in d.items()}
    tick = s['tickMs'].astype(float)
    seq = s['flowSeq']
    new = np.r_[False, (seq[1:] != seq[:-1]) & (seq[1:] > 0)]
    fi = np.where(new)[0]
    # the recorder's tick against the camera's frame clock (test_cam_vz_replay_ih.c)
    sr = st = 0.0
    for a_, b_ in zip(fi[:-1], fi[1:]):
        if ((seq[b_] - seq[a_]) % 255 + 255) % 255 != 1:
            continue
        dtr = s['flowDt_100us'][b_] / 1e4
        dtt = ((tick[b_] - s['flowAge_ms'][b_]) - (tick[a_] - s['flowAge_ms'][a_])) / 1000.0
        if 0.02 < dtr < 0.2 and 0.01 < dtt < 0.3:
            sr += dtr
            st += dtt
    K = sr / st if st > 0 else 1.0
    t = (tick - tick[0]) * K / 1000.0
    rr = np.radians(s['roll_cdeg'] / 100.0)
    pp = np.radians(s['pitch_cdeg'] / 100.0)
    aup = (s['ax_mg'] / 1000.0 * np.sin(pp) + s['ay_mg'] / 1000.0 * np.cos(pp) * np.sin(rr) +
           s['az_mg'] / 1000.0 * np.cos(pp) * np.cos(rr))
    ground = (s['cvFlags'] & 1) == 0
    lift = int(np.argmax(~ground))
    hov = np.where(s['autoState'] == 3)[0]
    iH = int(hov[0])
    br = np.where(np.diff(hov) > 1)[0]
    iHend = int(hov[br[0]] if len(br) else hov[-1])
    return s, t, aup, ground, lift, iH, iHend, new


def beta(path, fid):
    s, t, aup, ground, lift, iH, iHend, _ = load(path, fid)
    g0 = float(np.median(aup[max(0, lift - 128):max(1, lift - 5)]))
    iS = int(np.searchsorted(t, t[iH] + 3.0))
    # the arrival on a surface ends the hover: the touchdown flag of the controller (vzPhase) is not logged at it;
    # use the first row after which the thrust stays below 85 % of the hover's median for 1 s, or the hover's end
    thr = s['thrust'].astype(float)
    med = float(np.median(thr[iS:iHend]))
    iE = iHend
    for i in range(iS + 500, iHend):
        if np.all(thr[i:i + 100] < 0.85 * med):
            iE = i
            break
    cs = np.r_[0.0, np.cumsum(aup)]
    b = {}
    for i in range(iS, iE):
        lo, hi = max(iS, i - 500), min(iE, i + 501)
        b[i] = (cs[hi] - cs[lo]) / (hi - lo) - g0
    ti = np.array(sorted(b))
    tt = t[ti] - t[lift]
    bb = np.array([b[i] for i in ti])
    fit = tt > tt[-1] - 25.0
    sl, ic = np.polyfit(tt[fit], bb[fit], 1)
    grid = np.arange(0.0, 125.0 + 1e-9, 0.1)
    out = np.interp(grid, tt, bb)
    after = grid > tt[-1]
    out[after] = bb[-1] + sl * (grid[after] - tt[-1])
    return out, tt[0], tt[-1], sl * 1e3


def segment(path, fid, name):
    s, t, aup, ground, lift, iH, iHend, new = load(path, fid)
    dt = s['flowDt_100us'] * 1e-4
    g = np.vstack([s['flowGx_e4'], s['flowGy_e4'], s['flowGz_e4']]).T * 1e-4
    dyres = s['flowDy_cpx'] / 100.0 - (KY[0] * g[:, 0] + KY[1] * g[:, 1] + KY[2] * g[:, 2] + KY[3])
    G = s['cvGain_dpm'] / 10.0
    Gf = G.astype(float).copy()
    last = 0.0
    for i in range(len(Gf)):
        if Gf[i] > 0:
            last = Gf[i]
        else:
            Gf[i] = last
    if (Gf > 0).any():
        Gf[Gf == 0] = Gf[Gf > 0][0]
    vz = s['vzV_mms'] / 1000.0
    h = np.r_[0.0, np.cumsum(0.5 * (vz[1:] + vz[:-1]) * np.diff(t))]
    if name == 'F1':
        t0, t1 = t[iH] + 3.0, t[iHend] - 2.0
    else:
        td = lift + int(np.argmax(ground[lift:])) if ground[lift:].any() else len(t) - 1
        t0, t1 = t[lift] + 1.5, t[td]
    rows = []
    for i in np.where(new)[0]:
        tc = t[i] - s['flowAge_ms'][i] / 1000.0
        if tc < t0 or tc > t1 or not 0.02 < dt[i] < 0.2:
            continue
        te = tc + 0.015
        ts = te - dt[i]
        err = dyres[i] + Gf[i] * (np.interp(te, t, h) - np.interp(ts, t, h))
        rows.append((tc, int(round(dt[i] * 1e6)), int(s['flowOk'][i] != 0), int(round(err * 100.0))))
    tb = rows[0][0]
    return [(r[0] - tb, r[1], r[2], r[3]) for r in rows], float(np.mean(Gf[iH:iHend]))


def main():
    cal = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                                                '..', '..', 'firmware-backups', 'cal')
    lines = ['# Late offset noise closed-loop fixture (gen_cam_vz_late_fixture.py); see its docstring']
    for name, fn, fid, G in BETA:
        b, ts, te, sl = beta(os.path.join(cal, fn), fid)
        lines.append('# %s: %s fid %d, hover %.1f-%.1f s after the lift-off, then %+.3f mg/s' % (name, fn, fid, ts, te,
                                                                                                 sl))
        lines.append('B %s %.1f %d' % (name, G, len(b)))
        vals = ['%d' % round(x * 1e5) for x in b]
        for k in range(0, len(vals), 20):
            lines.append(' '.join(vals[k:k + 20]))
    for name, fn, fid in SEG:
        rows, gm = segment(os.path.join(cal, fn), fid, name)
        lines.append('# %s: %s fid %d, %d frames over %.1f s, the logged G %.1f px/m on average' % (
            name, fn, fid, len(rows), rows[-1][0], gm))
        lines.append('S %s %d' % (name, len(rows)))
        for r in rows:
            lines.append('%.4f %d %d %d' % r)
    with open(OUT, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('wrote %s (%d lines)' % (OUT, len(lines)))


if __name__ == '__main__':
    main()
