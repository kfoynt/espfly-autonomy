#!/usr/bin/env python3
"""The flow fit's firmware code against a reference, per pixel, in an Xtensa emulator (docs/autonomy-a2b.md 30).

GCC for the ESP32-S3 forms fused multiply-adds late, after moving loop invariants, so a change of optical_flow.c
can change the firmware's rounding where the host tests (another compiler) see nothing. This compiles
optical_flow.c with the firmware's own compile command (from a build directory's compile_commands.json), links it
alone (link.ld, libc/libm calls stubbed with float32 semantics) and runs ofProcess() in xemu.py on frame pairs
(frames.py: the drone's photos and the fixtures, warped), against the reference's object:
  - every ofResult byte,
  - every b[i] update's operands (G[i], e) in order: each used pixel's terms (tracer.py),
  - every ofSolve() input (A, b): the sums.
A restarted pass's records are allowed in front of the reference's. Instruction counts and a rough pipeline model
(cyc.py: load-use and FP latencies, taken branches; estimates, not the chip) give the cost.

  python3 tools/flow_emu/flow_emu.py compile OUT.elf SRC.c BUILD_DIR      # SRC's ../interface first on -I
  python3 tools/flow_emu/flow_emu.py equiv REF.elf:REF.c NEW.elf:NEW.c [N] # all pairs, 15 processes
  python3 tools/flow_emu/flow_emu.py cycles A.elf,B.elf [MODE] [PAIR]

02cbbe0's optical_flow.c is tools/tests/optical_flow_ref_02cbbe0.c (compile it from a copy named optical_flow.c
next to an interface/ directory with optical_flow.h, or from 02cbbe0's tree). Needs numpy and PIL; ~3 min for
225 pairs on 16 cores.
"""
import json, os, shlex, subprocess, sys, time
from collections import Counter
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frames
from tracer import compare, hooks_for
from xemu import TOOL, Emu, decode_res, run_pair

H = os.path.dirname(os.path.abspath(__file__))
STUBS = ['--defsym=memset=0x50000000', '--defsym=memcpy=0x50000010', '--defsym=sqrtf=0x50000020',
         '--defsym=__divsf3=0x50000030', '--defsym=fminf=0x50000040', '--defsym=fmaxf=0x50000050']
MODELS = {'opt': dict(load=2, fp=2, mul=2, fpcmp=1, taken=2, call=3),
          'pes': dict(load=2, fp=4, mul=2, fpcmp=2, taken=3, call=5)}


def compile_(out, src, build):
    src = os.path.abspath(src)
    for e in json.load(open(os.path.join(build, 'compile_commands.json'))):
        if e['file'].endswith('optical_flow.c'):
            args, d = shlex.split(e['command']), e['directory']
            break
    args = args[:args.index('-o')]
    args.insert(1, '-I' + os.path.join(os.path.dirname(os.path.dirname(src)), 'interface'))
    obj = out[:-4] + '.o'
    subprocess.run(args + ['-o', obj, '-c', src], cwd=d, check=True)
    subprocess.run([TOOL + '-ld', '-T', os.path.join(H, 'link.ld'), obj, '-o', out] + STUBS, check=True)
    print('ok', out)


P = None
E = {}


def _job(arg):
    global P
    i, ref, new = arg
    if P is None:
        P = frames.pairs()
    name, f0, f1, mode = P[i]
    for spec in (ref, new):
        if spec[0] not in E:
            e = Emu(spec[0])
            e.hooks = hooks_for(e, spec[1])
            E[spec[0]] = e
    er, en = E[ref[0]], E[new[0]]
    er.trace, en.trace = [], []
    r1, _, c1 = run_pair(er, f0, f1, mode)
    r2, _, c2 = run_pair(en, f0, f1, mode)
    ok, why = compare(er.trace, en.trace)
    return i, name, mode, r1 == r2, ok, why, sum(c1.values()), sum(c2.values()), decode_res(r1)['ok']


def equiv(ref, new, n=None):
    t = time.time()
    count = len(frames.pairs(n))
    with Pool(15) as pool:
        res = sorted(pool.map(_job, [(i, ref, new) for i in range(count)], chunksize=1))
    bad = [r for r in res if not (r[3] and r[4])]
    for r in bad:
        print('DIFF', r)
    restarts = sum(1 for r in res if not r[5].endswith(' 0 restarted'))
    print(f'{len(res)} pairs (modes {dict(Counter(r[2] for r in res))}): output and per-pixel trace identical on '
          f'{len(res) - len(bad)}; with Hessian-reuse restarts {restarts}; fit ok on {sum(1 for r in res if r[8])}; '
          f'instructions a pair ref {sum(r[6] for r in res) / len(res):.0f} new {sum(r[7] for r in res) / len(res):.0f}; '
          f'{time.time() - t:.0f} s')
    return not bad


def cycles(elfs, mode=2, pair=1):
    name, f0, f1, _ = frames.pairs()[pair]
    for elf in elfs:
        out = []
        for mk, m in MODELS.items():
            e = Emu(elf)
            e.model = m
            res, tot, last = run_pair(e, f0, f1, mode)
            out.append(f'{mk} {e.lastCycles / 240e3:6.2f} ms')
        print(f'{elf}: mode {mode} {name}: {sum(last.values())} instructions a frame, CPU at 240 MHz '
              f'{"  ".join(out)}')


if __name__ == '__main__':
    a = sys.argv[1:]
    if a and a[0] == 'compile' and len(a) == 4:
        compile_(a[1], a[2], a[3])
    elif a and a[0] == 'equiv' and len(a) in (3, 4):
        sys.exit(0 if equiv(a[1].split(':'), a[2].split(':'), int(a[3]) if len(a) > 3 else None) else 1)
    elif a and a[0] == 'cycles' and len(a) >= 2:
        cycles(a[1].split(','), int(a[2]) if len(a) > 2 else 2, int(a[3]) if len(a) > 3 else 1)
    else:
        print(__doc__)
        sys.exit(2)
