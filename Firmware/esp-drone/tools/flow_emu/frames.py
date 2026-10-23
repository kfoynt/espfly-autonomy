"""The frame pairs of the emulator check (flow_emu.py): the drone's own photos (ESPFLY_PHOTOS, default
~/.espfly/photos, 160x120 PNGs) and tools/tests/fixtures, warped (hover drift, rotation, large motion past the
margin, exposure steps, scale, brightened and saturated), consecutive real photo pairs, flat and half-flat frames.
Each photo gets one motion kind (index % 8) and a fit mode (index % 3), plain mode too on every other one."""
import glob, os
import numpy as np
from PIL import Image

FIX = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tests', 'fixtures')
PHOTOS = os.environ.get('ESPFLY_PHOTOS', os.path.expanduser('~/.espfly/photos'))


def load(p):
    if p.endswith('.raw'):
        return np.frombuffer(open(p, 'rb').read(), np.uint8).reshape(120, 160)
    return np.array(Image.open(p).convert('L'), dtype=np.uint8)


def photos():
    ps = sorted(p for p in glob.glob(os.path.join(PHOTOS, '*.png')) if not p.endswith('_x4.png'))
    ps = [p for p in ps if load(p).shape == (120, 160)]
    return ps + sorted(glob.glob(FIX + '/*.raw'))


def warp(img, dx, dy, s=0.0, rot=0.0, g=1.0, seed=0, off=0.0, noise=1.0):
    h, w = img.shape
    y, x = np.mgrid[0:h, 0:w].astype(np.float64)
    cx, cy = (w - 1) / 2, (h - 1) / 2
    X, Y = x - cx, y - cy
    c, sn = np.cos(rot), np.sin(rot)
    xs = cx + (1 + s) * (c * X - sn * Y) + dx
    ys = cy + (1 + s) * (sn * X + c * Y) + dy
    xs = np.clip(xs, 0, w - 1.001); ys = np.clip(ys, 0, h - 1.001)
    x0 = xs.astype(int); y0 = ys.astype(int); fx = xs - x0; fy = ys - y0
    f = img.astype(np.float64)
    v = (f[y0, x0] * (1 - fx) * (1 - fy) + f[y0, x0 + 1] * fx * (1 - fy) + f[y0 + 1, x0] * (1 - fx) * fy +
         f[y0 + 1, x0 + 1] * fx * fy)
    v = v * g + off + np.random.default_rng(seed).normal(0, noise, v.shape)
    return np.clip(np.rint(v), 0, 255).astype(np.uint8)


def pairs(n=None, seed=7):
    """[(name, f0, f1, mode)]"""
    rng = np.random.default_rng(seed)
    ps = photos()
    out = []
    for i, p in enumerate(ps):
        a = load(p)
        nm = os.path.basename(p)[:20]
        kind = i % 8
        if kind == 0:      # hover: sub-pixel drift
            prm = dict(dx=rng.normal(0, 0.5), dy=rng.normal(0, 0.5), s=rng.normal(0, 0.002), rot=rng.normal(0, 0.003))
        elif kind == 1:    # rotation-heavy (the warp's p[3] large)
            prm = dict(dx=rng.normal(0, 1), dy=rng.normal(0, 1), rot=rng.choice([-1, 1]) * rng.uniform(0.01, 0.04))
        elif kind == 2:    # large motion: pixels out of bounds (restarts)
            prm = dict(dx=rng.uniform(-6, 6), dy=rng.uniform(-5, 5), s=rng.normal(0, 0.01))
        elif kind == 3:    # exposure step
            prm = dict(dx=rng.normal(0, 1), dy=rng.normal(0, 1), g=rng.choice([0.9, 0.95, 1.05, 1.12]))
        elif kind == 4:    # scale (approach / recede)
            prm = dict(dx=rng.normal(0, 0.7), dy=rng.normal(0, 0.7), s=rng.choice([-1, 1]) * rng.uniform(0.005, 0.03))
        elif kind == 5:    # brighter + offset, saturation
            prm = dict(dx=rng.normal(0, 0.8), dy=rng.normal(0, 0.8), g=1.08, off=rng.uniform(0, 40))
        elif kind == 6:    # coarse shift past the margin after iteration 0 (Hessian reuse restarts, coarse)
            prm = dict(dx=rng.choice([-1, 1]) * rng.uniform(8.0, 9.5), dy=rng.normal(0, 0.5))
        else:              # fine shift near the margin (restarts at the fine level)
            prm = dict(dx=rng.choice([-1, 1]) * rng.uniform(3.4, 4.6), dy=rng.choice([-1, 1]) * rng.uniform(3.4, 4.6))
        b = warp(a, seed=i, **prm)
        mode = [2, 0, 1][i % 3]
        out.append((f'{nm} k{kind}', a.ravel(), b.ravel(), mode))
        if i % 2 == 0:    # the same pair in plain mode too (the mode in use)
            out.append((f'{nm} k{kind}', a.ravel(), b.ravel(), 2)) if mode != 2 else None
    # consecutive real pairs (same stem _0, _1)
    for p in ps:
        q = p.replace('_0.png', '_1.png')
        if p.endswith('_0.png') and q in ps:
            for mode in (2, 0):
                out.append((os.path.basename(p)[:20] + ' real', load(p).ravel(), load(q).ravel(), mode))
    # flat (fails), a half-flat frame, a saturated frame
    out.append(('flat', np.full(19200, 128, np.uint8), np.full(19200, 128, np.uint8), 2))
    a = load(ps[3])
    hf = a.copy(); hf[:, :80] = 90
    out.append(('halfflat', hf.ravel(), warp(hf, 0.4, 0.2, seed=99).ravel(), 2))
    out.append(('saturated', np.clip(a.astype(int) + 90, 0, 255).astype(np.uint8).ravel(),
                np.clip(warp(a, 0.3, -0.2, g=1.06, seed=98).astype(int) + 90, 0, 255).astype(np.uint8).ravel(), 0))
    return out[:n] if n else out
