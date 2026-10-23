#!/usr/bin/env python3
"""
Flight check for the camera drift hold's gain scale (docs/autonomy-bench.md J3 3, docs/autonomy-a2b.md 20.8):
is the hold rocking, or is its 0.3-1.5 Hz band only the camera's noise times the gain scale s?

  python3 tools/hold_band.py <recorder dump .csv> <flightId> [--kmax 2.5] [--t0 S] [--t1 S] [--moves]

The window runs from HOVER + 2 s to the first motion (mvState kind != 0) or the end of HOVER (--t0 / --t1:
seconds from the HOVER entry instead; --moves: through the motions, to the end of HOVER, for the long moves of
docs/autonomy-a2b.md 21). The recorder has no s column: s is rebuilt from autoState, mvState, cvFlags and
cvGain_dpm as camera_flow.c computes it (camHoldGainTarget, camHoldGainMove, camHoldGainRate, camHoldGainSlew in
camera_hold.c); during a MOVE toward the scene from mvRefFwd_e4 with the scene distance's lower bound taken as the
latched G's (99.79 / G: the observer's D; the drone's own bound can be lower after a forward move) and the camera
assumed to see the move (the recorder has no measured progress). pass --kmax 1 for a flight flown with 'holdk set 1'.
Standard library only.

Verdict (from the log replay of the 2026-10-14 hovers with the gain scale and the sim's over-gain sweep):
  OK       the noise times s: band rms 0.05-0.27 deg, peak-to-peak 0.3-1.6 deg at s 2-2.5, rms / s 0.04-0.11
  ROCKING  band peak-to-peak > 2.2 deg or band rms > 0.4 deg (the sim's loss of damping is a limit cycle of the
           hold over its +-2.5 deg: 4-5 deg peak-to-peak): 'holdk set 1' (not saved) and report
  NOISY    only rms / s > 0.15 deg: a noisier camera than 2026-10-14's (s 1: 0.02-0.12), not rocking
"""
import argparse
import csv
import math
import sys

AUTO_TAKEOFF, AUTO_HOVER, AUTO_LAND = 2, 3, 4
MVK_MOVE, MVK_YAW = 1, 2
MV_RUN, MV_PAUSED, MV_SETTLE = 1, 2, 3
G_REF = 80.0            # camera_hold.h CAM_HOLD_GAIN_G_REF
DPM_MIN, DPM_MAX = 150, 5000
CVF_LATCH = 0x05
SCALE_MAX = 3.0
SLEW_PER_S = 2.0
SLEW_MOVE_PER_S = 0.75  # CAM_HOLD_GAIN_SLEW_MOVE_PER_S: once a MOVE ran, in HOVER (2026-10-14)
FY_PX = 99.79           # AUTO_MV_FY_PX
K_LOW = 0.6             # AUTO_MV_K_LOW
TRACK_MIN = 0.01        # CAM_HOLD_GAIN_TRACK_MIN: the reference this far along before the move's gain applies
MAX_DT_S = 0.3          # CAM_HOLD_MAX_DT_S
LIM_P2P, LIM_RMS, LIM_RMS_S = 2.2, 0.4, 0.15


def band(x, dt=0.01):
    """The 0.3-1.5 Hz band: one-pole high-pass at 0.3 Hz, then one-pole low-pass at 1.5 Hz (the sim's filter)."""
    a_h = 0.5305 / (0.5305 + dt)
    a_l = dt / (0.1061 + dt)
    hp = bp = 0.0
    out = []
    prev = x[0] if x else 0.0
    for v in x:
        hp = a_h * (hp + v - prev)
        prev = v
        bp += a_l * (hp - bp)
        out.append(bp)
    return out


class GainScale:
    """camHoldGainTarget + camHoldGainMove + camHoldGainRate + camHoldGainSlew (camera_hold.c), one recorder row at
    a time."""

    def __init__(self, kmax):
        self.kmax = kmax
        self.g = 0.0
        self.translated = False
        self.move_active = False
        self.confirmed = False
        self.s_move = 1.0
        self.d_low = 0.0
        self.s = 1.0

    def target(self, auto_state, mv_state, cv_flags, cv_gain_dpm):
        if auto_state not in (AUTO_TAKEOFF, AUTO_HOVER, AUTO_LAND):
            self.g, self.translated = 0.0, False
            return 1.0
        kind, st = mv_state >> 4, mv_state & 0x0F
        if kind == MVK_MOVE:
            self.translated = True
        if kind == MVK_YAW and st in (MV_RUN, MV_PAUSED, MV_SETTLE):
            self.g = 0.0
        elif (cv_flags & CVF_LATCH) == CVF_LATCH and DPM_MIN <= cv_gain_dpm <= DPM_MAX:
            self.g = cv_gain_dpm * 0.1
        if not self.kmax > 1.0 or self.translated or not self.g > 0.0 or auto_state == AUTO_LAND:
            return 1.0
        return min(max(G_REF / self.g, 1.0), min(self.kmax, SCALE_MAX))

    def move_target(self, tgt, auto_state, mv_state, ref_fwd):
        """camHoldGainMove: during a MOVE toward the scene, once its reference is past TRACK_MIN (the camera assumed to
        see it: confirmed), clamp(D_ref_now / 1.25, 1, s at its start), D_ref_now = D_low exp(-ref_fwd / 0.6); D_low
        the latched G's D at its start (approximate)."""
        kind, st = mv_state >> 4, mv_state & 0x0F
        active = kind == MVK_MOVE and st in (MV_RUN, MV_PAUSED, MV_SETTLE)
        if active and not self.move_active:
            self.s_move = self.s if self.s > 1.0 else 1.0
            self.d_low = FY_PX / self.g if self.g > 0.0 else 0.0
            self.confirmed = False
        self.move_active = active
        if ref_fwd is not None and ref_fwd > TRACK_MIN:
            self.confirmed = True
        if ref_fwd is None or not active or auto_state != AUTO_HOVER or not self.g > 0.0 or not self.d_low > 0.0 \
                or not self.s_move > 1.0 or not self.confirmed:
            return tgt
        d_now = self.d_low * math.exp(-ref_fwd / K_LOW)
        return min(max(d_now * G_REF / FY_PX, 1.0), self.s_move)

    def rate(self, auto_state):
        """camHoldGainRate: 0.75 /s once a MOVE ran, in HOVER; else 2 /s."""
        return SLEW_MOVE_PER_S if self.translated and auto_state == AUTO_HOVER else SLEW_PER_S

    def step(self, auto_state, mv_state, cv_flags, cv_gain_dpm, dt, ref_fwd=None):
        tgt = self.target(auto_state, mv_state, cv_flags, cv_gain_dpm)
        tgt = self.move_target(tgt, auto_state, mv_state, ref_fwd)
        d = self.rate(auto_state) * min(dt, MAX_DT_S) if dt > 0 else 0.0
        self.s += max(-d, min(d, tgt - self.s))
        return self.s


def std(x):
    m = sum(x) / len(x)
    return math.sqrt(sum((v - m) ** 2 for v in x) / len(x))


def verdict(band_roll, band_pitch, s_mean):
    """(rms, p2p, rms / s, word): the larger axis of each."""
    rms = max(std(band_roll), std(band_pitch))
    p2p = max(max(band_roll) - min(band_roll), max(band_pitch) - min(band_pitch))
    rms_s = rms / max(s_mean, 1.0)
    if p2p > LIM_P2P or rms > LIM_RMS:
        word = 'ROCKING'
    elif rms_s > LIM_RMS_S:
        word = 'NOISY'
    else:
        word = 'OK'
    return rms, p2p, rms_s, word


def load(path, fid):
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            if int(r['flightId']) == fid:
                rows.append(r)
    rows.sort(key=lambda r: int(r['tickMs']))
    out, last = [], None
    for r in rows:
        t = int(r['tickMs'])
        if t == last:
            continue
        last = t
        out.append(r)
    return out


def analyse(rows, kmax=2.5, t0=None, t1=None, moves=False):
    if not rows:
        raise ValueError('no rows')
    tick0 = int(rows[0]['tickMs'])
    t = [(int(r['tickMs']) - tick0) / 1000.0 for r in rows]
    st = [int(r['autoState']) for r in rows]
    mv = [int(r.get('mvState') or 0) for r in rows]
    gs = GainScale(kmax)
    s = []
    for i, r in enumerate(rows):
        dt = t[i] - t[i - 1] if i else 0.0
        ref_fwd = int(r['mvRefFwd_e4']) * 1e-4 if r.get('mvRefFwd_e4') not in (None, '') else None
        s.append(gs.step(st[i], mv[i], int(r['cvFlags']), int(r['cvGain_dpm']), dt, ref_fwd))
    if AUTO_HOVER not in st:
        raise ValueError('no HOVER')
    ih = st.index(AUTO_HOVER)
    e = len(rows)
    why = 'the end of the log'
    for i in range(ih + 1, len(rows)):
        if mv[i] >> 4 != 0 and not moves:
            e, why = i, 'the first motion'
            break
        if st[i] != AUTO_HOVER:
            e, why = i, 'the end of HOVER'
            break
    lo = t[ih] + (2.0 if t0 is None else t0)
    hi = t[e - 1] if t1 is None else t[ih] + t1
    roll = [int(r['holdRoll_cdeg']) / 100.0 for r in rows[ih:e]]
    pitch = [int(r['holdPitch_cdeg']) / 100.0 for r in rows[ih:e]]
    br, bp = band(roll), band(pitch)
    idx = [k for k in range(e - ih) if lo <= t[ih + k] <= hi]
    if len(idx) < 300:
        raise ValueError('window %.1f s (< 3 s) from HOVER to %s' % (len(idx) / 100.0, why))
    br = [br[k] for k in idx]
    bp = [bp[k] for k in idx]
    sw = [s[ih + k] for k in idx]
    rms, p2p, rms_s, word = verdict(br, bp, sum(sw) / len(sw))
    return dict(t0=lo - t[ih], t1=hi - t[ih], why=why, s_min=min(sw), s_max=max(sw), s_mean=sum(sw) / len(sw),
                rms_roll=std(br), rms_pitch=std(bp), p2p_roll=max(br) - min(br), p2p_pitch=max(bp) - min(bp),
                rms=rms, p2p=p2p, rms_s=rms_s, verdict=word)


ADVICE = {'OK': 'the camera noise times s', 'ROCKING': "holdk set 1 (not saved) and report",
          'NOISY': 'a noisier camera, not rocking: note it, compare with the next flight'}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('csv')
    ap.add_argument('fid', type=int)
    ap.add_argument('--kmax', type=float, default=2.5)
    ap.add_argument('--t0', type=float, default=None)
    ap.add_argument('--t1', type=float, default=None)
    ap.add_argument('--moves', action='store_true', help='through the motions, to the end of HOVER')
    a = ap.parse_args(argv)
    try:
        r = analyse(load(a.csv, a.fid), a.kmax, a.t0, a.t1, a.moves)
    except ValueError as exc:
        print('flight %d: %s' % (a.fid, exc), file=sys.stderr)
        return 2
    print('flight %d: HOVER+%.1f..+%.1f s (to %s); s %.2f..%.2f (mean %.2f, kmax %.2f); band rms roll %.3f pitch '
          '%.3f, peak-to-peak roll %.2f pitch %.2f deg; rms / s %.3f -> %s: %s'
          % (a.fid, r['t0'], r['t1'], r['why'], r['s_min'], r['s_max'], r['s_mean'], a.kmax, r['rms_roll'],
             r['rms_pitch'], r['p2p_roll'], r['p2p_pitch'], r['rms_s'], r['verdict'], ADVICE[r['verdict']]))
    return 1 if r['verdict'] == 'ROCKING' else 0


if __name__ == '__main__':
    sys.exit(main())
