#!/usr/bin/env python3
"""Replay fixture for the camera vertical observer (cam_vz_obs.c).

Replays every logged camera take-off through the Python reference observer
(observer.py of the camera height study, which builds on flowmodel.py) and
writes its inputs and outputs to tools/tests/fixtures/cam_vz_replay.txt, so
tools/tests/test_cam_vz_obs.c can check the C port against it without the
flight CSVs (they are not in the repository) or Python.

    python3 tools/tests/gen_cam_vz_fixture.py STUDY_DIR [--data CSV_DIR]

STUDY_DIR holds observer.py, flowmodel.py and bias_truth.csv (the camera
height study, 2026-10-11; not in the repository); CSV_DIR the flight*.csv
recorder files (default: flowmodel.DATA_DIR, firmware-backups/cal).

Take-offs: every take-off of the 13 camera files with status ok and grade A,
B or C in bias_truth.csv (observer.takeoffs), i.e. the study's 28 A/B ones
plus 15 grade C ones, so that all 13 files are covered. The observer runs as
in observer.run_takeoff: wake W = 1.5 s, prior g0 + leave-one-file-out
offset (sigma 2.2 mg), once with the recommended FINAL gates and once with the
plain filter (DEFAULT only), which exercises the disabled-gate paths.

File format (whitespace separated, '#' starts a comment line):
  V 1
  T file fid grade b0_mg s0_mg t_cam_us t_lift_s t_touch_s nrows
      b0/s0: prior bias and sigma (mg); t_cam_us: frames whose motion window
      starts earlier are ignored (camera asleep); a row is on the ground while
      its time (ms / 1000.0, as a double) < t_lift_s; the take-off ends at its
      last row (the first with t >= t_touch_s), whose frame is not used.
  nrows lines, one per 100 Hz recorder row from the take-off's first row:
    A[/D]                       a_up - 1 g in units of 1e-7 g; /D: the row is
                                D ms after the previous one (default 10)
    A[/D] seq age dt ok tex res dx dy sc gx gy gz
                                the row shows a new camera frame: seq (frame
                                counter, unwrapped from the logged seq % 255),
                                flowAge_ms, flowDt_100us, flowOk, flowTexture,
                                flowResidual_dg, flowDx_cpx, flowDy_cpx,
                                flowScale_e4, flowGx_e4, flowGy_e4, flowGz_e4.
                                Capture time = row time - age + 1 ms (the
                                study's clock offset for the ms truncation).
  D cfg string                  one character per frame the observer was given
                                (the frames run_takeoff did not skip), see
                                DECISION below
  K cfg row z v b_mg sig_z sig_v sig_b_mg G n_upd n_rej
                                state after row `row` (0-based) and its frame,
                                every 100 rows and at the last row; G = the
                                gain tracker's G (nan before the first valid)
  E cfg relocks n_alarm n_exp_rej n_yaw_reset g_first_s
                                counters at the end; g_first_s = window centre
                                of the first valid gain (s), nan if none
cfg: 0 = FINAL, 1 = plain.

How the C test replays it (same semantics as run_takeoff): one
camVzObsImu() per row (time = row ms, ground while ms / 1000.0 < t_lift_s,
thrust > 0: the recorder logs from throttle-up, asserted below), then
camVzObsSetLiftoffTime(t_lift) at the first airborne row, so that the C
skip rule max(throttle-up + 1.5 s, lift-off) equals t_cam_us; then
camVzObsFrame() for the row's frame unless it is the last row. Python's
GainTracker / KF times are seconds since the take-off's first row, the C
module's microseconds since its first sample: the same origin.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

# Decision codes, one per frame given to VertObserver.frame():
#   q quality gate (ok, texture, residual, dt, yaw rate, consecutive seq)
#   e expansion-rate gate     x lateral-flow gate
#   w window not in the IMU history
#   g no gain (tracker invalid and no default/guess)
#   c refused while the CUSUM alarm holds
#   r innovation gate         A accepted
CHECK_EVERY = 100


def parse():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("study", help="camera height study directory (observer.py, flowmodel.py, bias_truth.csv)")
    p.add_argument("--data", help="directory of flight*.csv (default flowmodel.DATA_DIR)")
    p.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures",
                                                 "cam_vz_replay.txt"))
    return p.parse_args()


def main():
    a = parse()
    sys.path.insert(0, os.path.abspath(a.study))
    import flowmodel as fm    # noqa: E402  (study modules)
    import observer as ob     # noqa: E402
    if a.data:
        fm.DATA_DIR = os.path.abspath(a.data)

    class Instrumented(ob.VertObserver):
        """VertObserver that records why each frame was used or not."""
        def frame(self, t0, t1, dy_res, good, scale=None, dx_res=None, yaw=None):
            n_exp, n_rej, n_upd, n_g = self.n_exp_rej, self.n_rej, self.n_upd, len(self.g_log)
            rec = super().frame(t0, t1, dy_res, good, scale=scale, dx_res=dx_res, yaw=yaw)
            if self.n_upd > n_upd:
                code = "A"
            elif self.n_rej > n_rej:
                code = "c" if self.alarm_t is not None else "r"
            elif self.n_exp_rej > n_exp:
                c = self.c
                code = "e" if (c["max_exp_rate"] is not None and abs(self.exp_ema) > c["max_exp_rate"]) else "x"
            elif not good or t1 - t0 <= 0:
                code = "q"
            elif len(self.g_log) > n_g:
                code = "g"
            else:
                code = "w"
            self.codes.append(code)
            return rec

    def run(to, cfg, off):
        """observer.run_takeoff(to, 1.5, 'g0off', cfg, off=off), recording decisions and checkpoints."""
        c = dict(ob.DEFAULT)
        c.update(cfg)
        o, _ = off
        b0, s0 = (to["g0"] - 1000.0) + o, 2.2
        obs = Instrumented(b0, s0, c)
        obs.codes = []
        t, acc = to["t"], to["a_up_g"]
        t_lift, t_touch = to["t_lift"], to["t_touch"]
        t_cam = max(1.5, t_lift)
        fi = ob.frame_inputs(to, c)
        order = np.argsort(fi["t_avail"], kind="stable")
        k, nf = 0, len(order)
        i_end = int(np.searchsorted(t, t_touch))
        n = min(i_end + 1, len(t))
        ck, Z = [], []
        for i in range(n):
            obs.imu(t[i], acc[i], on_ground=t[i] < t_lift)
            while k < nf and fi["t_avail"][order[k]] <= t[i] + 1e-9:
                j = order[k]
                k += 1
                if fi["t0"][j] < t_cam or t[i] < t_lift or t[i] >= t_touch:
                    continue
                obs.frame(fi["t0"][j], fi["t1"][j], fi["dy_res"][j], bool(fi["good"][j]),
                          scale=fi["scale"][j], dx_res=fi["dx_res"][j], yaw=fi["fgz"][j])
            Z.append((obs.x[0], obs.x[1], obs.b_mg))
            if i % CHECK_EVERY == 0 or i == n - 1:
                P = obs.P
                ck.append((i, obs.x[0], obs.x[1], obs.b_mg, math.sqrt(max(P[0, 0], 0)), math.sqrt(max(P[1, 1], 0)),
                           obs.sig_b_mg, obs.gt.G, obs.n_upd, obs.n_rej))
        # the loop above must be observer.run_takeoff itself
        ref = ob.run_takeoff(to, 1.5, "g0off", cfg=cfg, off=off)
        Z = np.array(Z)
        assert np.array_equal(Z[:, 0], ref["z"]) and np.array_equal(Z[:, 1], ref["v"]) and \
            np.array_equal(Z[:, 2], ref["b"]), "instrumented loop differs from run_takeoff"
        assert obs.n_upd == ref["n_upd"] and obs.n_rej == ref["n_rej"]
        g_first = obs.gt.first_valid_t
        end = (obs.relocks, obs.n_alarm, obs.n_exp_rej, obs.n_yaw_reset,
               float("nan") if g_first is None else g_first)
        return dict(b0=b0, s0=s0, t_cam=t_cam, n=n, codes="".join(obs.codes), ck=ck, end=end, fi=fi)

    truth = ob.load_truth()
    tos = ob.takeoffs(grades=("A", "B", "C"))
    cfgs = [("FINAL", dict(ob.FINAL)), ("plain", dict())]
    lines = ["# cam_vz_obs replay fixture, generated by tools/tests/gen_cam_vz_fixture.py (format: see there)",
             "# reference: observer.py run_takeoff(W=1.5 s, prior g0off); cfg 0 = FINAL " + str(ob.FINAL) +
             ", cfg 1 = plain (DEFAULT)",
             "V 1"]
    n_frames = 0
    edge = 0
    for to in tos:
        off = ob.offset_prior(truth, exclude_file=to["file"])
        res = [run(to, cfg, off) for _, cfg in cfgs]
        r0 = res[0]
        ds = to["ds"]
        assert ds["thrust"][0] > 0, "take-off does not start at throttle-up"
        tick = ds["tickMs"].astype(np.int64)
        tick0 = int(tick[0])
        t_cam_us = int(round(r0["t_cam"] * 1e6))
        assert abs(t_cam_us - r0["t_cam"] * 1e6) < 1e-3
        # integer-microsecond skip rule == the study's double comparison?
        F = to["F"]
        t1_us = (tick[F["row"]] - ds["flowAge_ms"][F["row"]].astype(np.int64) + 1 + 15 - tick0) * 1000
        t0_us = t1_us - (ds["flowDt_100us"][F["row"]].astype(np.int64) * 100)
        edge += int(np.sum((t0_us < t_cam_us) != (r0["fi"]["t0"] < r0["t_cam"])))
        lines.append(f"T {to['file']} {to['fid']} {to['grade']} {r0['b0']:.17g} {r0['s0']:.17g} {t_cam_us} "
                     f"{to['t_lift']:.17g} {to['t_touch']:.17g} {r0['n']}")
        # frames by row; unwrap seq with the study's gap rule
        frame_at = {}
        seq = None
        prev = None
        for j, row in enumerate(F["row"]):
            s = int(F["seq"][j])
            if prev is None:
                seq = s
            else:
                seq += int((s - prev - 1) % 255) + 1
            prev = s
            frame_at[int(row)] = (j, seq)
        aup = to["a_up_g"]
        for i in range(r0["n"]):
            q = int(round((aup[i] - 1.0) * 1e7))
            dt = int(tick[i] - tick[i - 1]) if i > 0 else 10
            s = f"{q}" if dt == 10 else f"{q}/{dt}"
            if i in frame_at:
                j, sq = frame_at[i]
                r = int(F["row"][j])
                cols = ("flowAge_ms", "flowDt_100us", "flowOk", "flowTexture", "flowResidual_dg", "flowDx_cpx",
                        "flowDy_cpx", "flowScale_e4", "flowGx_e4", "flowGy_e4", "flowGz_e4")
                s += f" {sq} " + " ".join(str(int(ds[c][r])) for c in cols)
                n_frames += 1
            lines.append(s)
        for ci, r in enumerate(res):
            lines.append(f"D {ci} {r['codes'] if r['codes'] else '-'}")
            for (i, z, v, b, sz, sv, sb, G, nu, nr) in r["ck"]:
                lines.append(f"K {ci} {i} {z:.9g} {v:.9g} {b:.9g} {sz:.9g} {sv:.9g} {sb:.9g} {G:.9g} {nu} {nr}")
            rl, na, ne, ny, gf = r["end"]
            lines.append(f"E {ci} {rl} {na} {ne} {ny} {gf:.9g}")
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {a.out}: {len(tos)} take-offs, files {sorted(set(t['file'] for t in tos))}, "
          f"{sum(1 for x in lines if x[:1] not in 'TDKEV#')} rows, {n_frames} frames, "
          f"{os.path.getsize(a.out) / 1024:.0f} KiB; integer-us skip-rule mismatches: {edge}")
    if edge:
        sys.exit("the integer-microsecond frame skip rule differs from the study's; fix the fixture first")


if __name__ == "__main__":
    main()
