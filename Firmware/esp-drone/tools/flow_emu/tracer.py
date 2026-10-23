"""Per-pixel equivalence on the target code (flow_emu.py): every b[i] update's operands (G[i], e) in execution order and every
ofSolve() input (A, b), traced in the emulator. b-update instructions are found by their source line."""
import re, subprocess
from xemu import Emu, TOOL

B_LINE = re.compile(r'(\bb\[i\] \+= G\[i\] \* e)|(\bb[0-3] \+= g[0-3] \* e|\bb[0-3] = fmaf\(g[0-3], e, b[0-3]\))|(\bb4 \+= e\b)')


def hooks_for(emu, src):
    lines = open(src).read().split('\n')
    pcs = [a for a, (mn, ops, s) in emu.ins.items() if mn in ('madd.s', 'msub.s', 'add.s')]
    out = subprocess.run([TOOL + '-addr2line', '-e', emu.elf] + ['%x' % a for a in pcs], capture_output=True,
                         text=True).stdout.split('\n')
    h = {}
    for a, l in zip(pcs, out):
        m = re.search(r':(\d+)', l)
        if not m:
            continue
        txt = lines[int(m.group(1)) - 1]
        mb = B_LINE.search(txt)
        if mb:
            if mb.group(3):   # b4 += e: an add
                if emu.ins[a][0] != 'add.s':
                    continue
            idx = 'b'
            h[a] = idx
    h[emu.syms['ofSolve']] = 'solve'
    return h


def segments(trace):
    """[(b-records before a solve, solve record)], the last with solve None."""
    seg, cur = [], []
    for r in trace:
        if r[0] == 'solve':
            seg.append((cur, r)); cur = []
        else:
            cur.append(r)
    seg.append((cur, None))
    return seg


def compare(tr_ref, tr_new):
    """True if every segment's solve input is equal and the new segment's b-records end with the reference's
    (a restarted pass's partial records come first)."""
    a, b = segments(tr_ref), segments(tr_new)
    if len(a) != len(b):
        return False, f'segments {len(a)} vs {len(b)}'
    restarts = 0
    for i, ((ra, sa), (rb, sb)) in enumerate(zip(a, b)):
        if sa != sb:
            return False, f'solve input {i} differs'
        if len(rb) < len(ra) or rb[len(rb) - len(ra):] != ra:
            return False, f'b-records of segment {i} differ ({len(ra)} vs {len(rb)})'
        restarts += len(rb) != len(ra)
    return True, f'{len(a)} segments, {sum(len(x[0]) for x in a)} b-records, {restarts} restarted'
