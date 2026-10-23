#!/usr/bin/env python3
"""Minimal ESP32-S3 (Xtensa LX7, windowed ABI) interpreter for optical_flow.c's linked code (flow_emu.py).

Runs ofProcess() of a standalone link (link.ld; memset/memcpy/sqrtf/__divsf3/fminf/fmaxf at stub addresses,
emulated with correctly rounded float32 semantics) and counts every executed instruction by pc, so variants of
the same source (compiler flags) can be compared for bit-identical outputs and dynamic instruction counts.
FP: add/sub/mul rounded once to float32 (via float64, innocuous double rounding); madd.s / msub.s fused (one
rounding, exact tie handling). Only the opcodes optical_flow.c's code uses are implemented.
"""
import math, os, re, struct, subprocess, sys
from collections import Counter
import numpy as np

TOOL = os.environ.get('XTENSA_TOOL', os.path.expanduser(
    '~/.espressif/tools/xtensa-esp32s3-elf/esp-2022r1-11.2.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf'))
STUBS = {0x50000000: 'memset', 0x50000010: 'memcpy', 0x50000020: 'sqrtf', 0x50000030: '__divsf3',
         0x50000040: 'fminf', 0x50000050: 'fmaxf'}
RET_SENTINEL = 0x60000000
TARGET_OPS = {'j', 'l32r', 'loop', 'loopnez', 'loopgtz', 'call8', 'beq', 'bne', 'blt', 'bge', 'bltu', 'bgeu',
              'beqi', 'bnei', 'blti', 'bgei', 'bltui', 'bgeui', 'beqz', 'bnez', 'beqz.n', 'bnez.n', 'bltz', 'bgez',
              'bbci', 'bbsi', 'bt', 'bf'}
M32 = 0xFFFFFFFF


def f2b(f):
    return struct.unpack('<I', struct.pack('<f', f))[0]


def b2f(b):
    return struct.unpack('<f', struct.pack('<I', b & M32))[0]


def r32(x):
    """float64 -> nearest float32 (as python float)."""
    return float(np.float32(x))


def fused(a, b, c, sub=False):
    """round32(c + a*b) (or c - a*b) with a single rounding."""
    p = a * b   # exact in float64 (24 x 24 bits)
    if sub:
        p = -p
    if math.isinf(p) or math.isnan(p) or math.isinf(c) or math.isnan(c):
        return r32(c + p)
    s = c + p
    # TwoSum: c + p == s + e exactly
    bp = s - c
    e = (c - (s - bp)) + (p - bp)
    r = np.float32(s)
    if e != 0.0 and float(r) != s:
        lo = np.float32(s) if float(np.float32(s)) < s else np.nextafter(np.float32(s), np.float32(-np.inf))
        hi = np.nextafter(lo, np.float32(np.inf))
        mid = (float(lo) + float(hi)) / 2.0
        if s == mid:   # a float32 tie decided by the rounding error of the double sum
            r = hi if e > 0 else lo
    return float(r)


class Emu:
    def __init__(self, elf):
        self.elf = elf
        dis = subprocess.run([TOOL + '-objdump', '-d', '--no-show-raw-insn', elf], capture_output=True,
                             text=True).stdout
        self.syms = {}
        self.ins = {}
        sym = None
        for line in dis.splitlines():
            m = re.match(r'^([0-9a-f]{8}) <([^>]+)>:', line)
            if m:
                sym = m.group(2)
                self.syms[sym] = int(m.group(1), 16)
                continue
            m = re.match(r'^([0-9a-f]+):\t(\S+)\t?(.*)$', line)
            if m:
                addr = int(m.group(1), 16)
                mn = m.group(2)
                ops = m.group(3)
                ops = re.sub(r'\s*<[^>]*>', '', ops)
                ops = [o.strip() for o in ops.split(',')] if ops.strip() else []
                self.ins[addr] = (mn, ops, sym)
        addrs = sorted(self.ins)
        self.next = {a: b for a, b in zip(addrs, addrs[1:])}
        # text image for l32r
        tb = '%s.%d.bin' % (elf, os.getpid())
        subprocess.run([TOOL + '-objcopy', '-O', 'binary', '-j', '.text', elf, tb], check=True)
        self.text = open(tb, 'rb').read()
        os.remove(tb)
        self.textBase = 0x40000000
        self.decode()

    def decode(self):
        """Pre-decode operands into ints."""
        def reg(o):
            return int(o[1:])
        self.dec = {}
        for a, (mn, ops, sym) in self.ins.items():
            d = []
            for o in ops:
                if re.fullmatch(r'a\d+', o) or re.fullmatch(r'f\d+', o) or re.fullmatch(r'b\d+', o):
                    d.append(reg(o))
                elif re.fullmatch(r'-?0x[0-9a-f]+', o):
                    d.append(int(o, 16))
                elif re.fullmatch(r'-?\d+', o):
                    d.append(int(o))
                elif re.fullmatch(r'[0-9a-f]+', o):
                    d.append(int(o, 16))
                else:
                    d.append(o)
            if mn in TARGET_OPS:
                d[-1] = int(ops[-1], 16)
            self.dec[a] = (mn, d)

    # memory: data space as a dict of bytearrays
    def setup_mem(self):
        self.mem = bytearray(0x100000)   # 0x3fc00000 .. +1 MB
        self.memBase = 0x3FC00000

    def ld(self, addr, n):
        o = addr - self.memBase
        if 0 <= o < len(self.mem):
            return int.from_bytes(self.mem[o:o + n], 'little')
        o = addr - self.textBase
        if 0 <= o < len(self.text):
            return int.from_bytes(self.text[o:o + n], 'little')
        raise RuntimeError('load from %08x' % addr)

    def st(self, addr, n, v):
        o = addr - self.memBase
        if not (0 <= o < len(self.mem)):
            raise RuntimeError('store to %08x' % addr)
        self.mem[o:o + n] = (v & ((1 << (8 * n)) - 1)).to_bytes(n, 'little')

    def call(self, fn, args, sp):
        """Calls fn(args...) (int args) as if from a callx8 in a frame with stack pointer sp. Returns a10."""
        AR = [0] * 4096
        self.base = 0
        AR[1] = sp
        for i, v in enumerate(args):
            AR[10 + i] = v & M32
        FR = [0.0] * 16
        BR = [False] * 16
        counts = Counter()
        pc = self.syms[fn]
        AR[8] = RET_SENTINEL
        pendingInc = 8
        loops = []   # (lbeg, lend, count) zero-overhead loop state (only one active in hardware: LBEG/LEND/LCOUNT)
        lbeg = lend = None
        lcount = 0
        base = 0
        ins = self.dec
        nxt = self.next
        ld, st = self.ld, self.st
        steps = 0
        model = getattr(self, 'model', None)
        hooks = getattr(self, 'hooks', None)
        trace = getattr(self, 'trace', None)
        if model is not None:
            from cyc import defuse, latency
            if not hasattr(self, 'du'):
                self.du = {a: defuse(mn_, ops_) for a, (mn_, ops_, s_) in self.ins.items()}
                self.lat = {a: latency(self.ins[a][0], model) for a in self.ins}
            du, latt = self.du, self.lat
            ready = {}
            cyc = self.cycles
        while True:
            if pc == RET_SENTINEL:
                if model is not None:
                    self.cycles = cyc
                return self.retval, counts
            mn, d = ins[pc]
            counts[pc] += 1
            if hooks is not None and pc in hooks:
                kind = hooks[pc]
                if kind == 'solve':
                    a_ = AR[base + 10]; b_ = AR[base + 11]
                    trace.append(('solve', bytes(self.mem[a_ - self.memBase:a_ - self.memBase + 100]),
                                  bytes(self.mem[b_ - self.memBase:b_ - self.memBase + 20])))
                elif mn in ('madd.s', 'msub.s'):
                    x_, y_ = f2b(FR[d[1]]), f2b(FR[d[2]])
                    trace.append((kind, min(x_, y_), max(x_, y_)))
                elif mn == 'add.s':
                    x_ = f2b(FR[d[2]]) if d[1] == d[0] else f2b(FR[d[1]])
                    one = 0x3F800000
                    trace.append((kind, min(x_, one), max(x_, one)))
                else:
                    raise RuntimeError('hook on %s' % mn)
            if model is not None:
                dd, ss = du[pc]
                t = cyc
                for k, r in ss:
                    key = (k, base + r) if k == 'a' else (k, r)
                    rt = ready.get(key, 0)
                    if rt > t:
                        t = rt
                cyc = t + 1
                lt = latt[pc]
                for k, r in dd:
                    ready[(k, base + r) if k == 'a' else (k, r)] = t + lt
            steps += 1
            npc = nxt.get(pc)
            B = base
            if mn in ('l32i', 'l32i.n'):
                AR[B + d[0]] = ld((AR[B + d[1]] + d[2]) & M32, 4)
            elif mn in ('s32i', 's32i.n'):
                st((AR[B + d[1]] + d[2]) & M32, 4, AR[B + d[0]])
            elif mn == 'lsi':
                FR[d[0]] = b2f(ld((AR[B + d[1]] + d[2]) & M32, 4))
            elif mn == 'ssi':
                st((AR[B + d[1]] + d[2]) & M32, 4, f2b(FR[d[0]]))
            elif mn == 'lsip':
                FR[d[0]] = b2f(ld(AR[B + d[1]], 4)); AR[B + d[1]] = (AR[B + d[1]] + d[2]) & M32
            elif mn == 'ssip':
                st(AR[B + d[1]], 4, f2b(FR[d[0]])); AR[B + d[1]] = (AR[B + d[1]] + d[2]) & M32
            elif mn == 'lsx':
                FR[d[0]] = b2f(ld((AR[B + d[1]] + AR[B + d[2]]) & M32, 4))
            elif mn == 'ssx':
                st((AR[B + d[1]] + AR[B + d[2]]) & M32, 4, f2b(FR[d[0]]))
            elif mn == 'l8ui':
                AR[B + d[0]] = ld((AR[B + d[1]] + d[2]) & M32, 1)
            elif mn == 'l16ui':
                AR[B + d[0]] = ld((AR[B + d[1]] + d[2]) & M32, 2)
            elif mn == 'l16si':
                v = ld((AR[B + d[1]] + d[2]) & M32, 2); AR[B + d[0]] = (v - 0x10000 if v & 0x8000 else v) & M32
            elif mn == 's8i':
                st((AR[B + d[1]] + d[2]) & M32, 1, AR[B + d[0]])
            elif mn == 's16i':
                st((AR[B + d[1]] + d[2]) & M32, 2, AR[B + d[0]])
            elif mn == 'l32r':
                AR[B + d[0]] = ld(d[1], 4)
            elif mn in ('add', 'add.n'):
                AR[B + d[0]] = (AR[B + d[1]] + AR[B + d[2]]) & M32
            elif mn in ('addi', 'addi.n', 'addmi'):
                AR[B + d[0]] = (AR[B + d[1]] + d[2]) & M32
            elif mn == 'addx2':
                AR[B + d[0]] = ((AR[B + d[1]] << 1) + AR[B + d[2]]) & M32
            elif mn == 'addx4':
                AR[B + d[0]] = ((AR[B + d[1]] << 2) + AR[B + d[2]]) & M32
            elif mn == 'addx8':
                AR[B + d[0]] = ((AR[B + d[1]] << 3) + AR[B + d[2]]) & M32
            elif mn == 'subx2':
                AR[B + d[0]] = ((AR[B + d[1]] << 1) - AR[B + d[2]]) & M32
            elif mn == 'subx4':
                AR[B + d[0]] = ((AR[B + d[1]] << 2) - AR[B + d[2]]) & M32
            elif mn == 'subx8':
                AR[B + d[0]] = ((AR[B + d[1]] << 3) - AR[B + d[2]]) & M32
            elif mn == 'sub':
                AR[B + d[0]] = (AR[B + d[1]] - AR[B + d[2]]) & M32
            elif mn == 'and':
                AR[B + d[0]] = AR[B + d[1]] & AR[B + d[2]]
            elif mn == 'or':
                AR[B + d[0]] = AR[B + d[1]] | AR[B + d[2]]
            elif mn == 'xor':
                AR[B + d[0]] = AR[B + d[1]] ^ AR[B + d[2]]
            elif mn == 'mov.n' or mn == 'mov':
                AR[B + d[0]] = AR[B + d[1]]
            elif mn in ('movi', 'movi.n'):
                AR[B + d[0]] = d[1] & M32
            elif mn == 'slli':
                AR[B + d[0]] = (AR[B + d[1]] << d[2]) & M32
            elif mn == 'srli':
                AR[B + d[0]] = AR[B + d[1]] >> d[2]
            elif mn == 'srai':
                v = AR[B + d[1]]; v = v - (1 << 32) if v & 0x80000000 else v; AR[B + d[0]] = (v >> d[2]) & M32
            elif mn == 'extui':
                AR[B + d[0]] = (AR[B + d[1]] >> d[2]) & ((1 << d[3]) - 1)
            elif mn == 'mull':
                AR[B + d[0]] = (AR[B + d[1]] * AR[B + d[2]]) & M32
            elif mn in ('rems', 'quos'):
                x = AR[B + d[1]]; y = AR[B + d[2]]
                x = x - (1 << 32) if x & 0x80000000 else x
                y = y - (1 << 32) if y & 0x80000000 else y
                q = abs(x) // abs(y) * (1 if (x >= 0) == (y >= 0) else -1)
                AR[B + d[0]] = ((x - q * y) if mn == 'rems' else q) & M32
            elif mn == 'neg':
                AR[B + d[0]] = (-AR[B + d[1]]) & M32
            elif mn in ('max', 'min'):
                x = AR[B + d[1]]; y = AR[B + d[2]]
                xs = x - (1 << 32) if x & 0x80000000 else x
                ys = y - (1 << 32) if y & 0x80000000 else y
                AR[B + d[0]] = x if ((xs >= ys) if mn == 'max' else (xs <= ys)) else y
            elif mn in ('movnez', 'moveqz', 'movltz', 'movgez'):
                t = AR[B + d[2]]; ts = t - (1 << 32) if t & 0x80000000 else t
                c = {'movnez': t != 0, 'moveqz': t == 0, 'movltz': ts < 0, 'movgez': ts >= 0}[mn]
                if c:
                    AR[B + d[0]] = AR[B + d[1]]
            elif mn in ('movt', 'movf'):
                if BR[d[2]] == (mn == 'movt'):
                    AR[B + d[0]] = AR[B + d[1]]
            elif mn in ('movt.s', 'movf.s'):
                if BR[d[2]] == (mn == 'movt.s'):
                    FR[d[0]] = FR[d[1]]
            elif mn in ('moveqz.s', 'movnez.s', 'movltz.s', 'movgez.s'):
                t = AR[B + d[2]]; ts = t - (1 << 32) if t & 0x80000000 else t
                c = {'movnez.s': t != 0, 'moveqz.s': t == 0, 'movltz.s': ts < 0, 'movgez.s': ts >= 0}[mn]
                if c:
                    FR[d[0]] = FR[d[1]]
            # ---- FP
            elif mn == 'wfr':
                FR[d[0]] = b2f(AR[B + d[1]])
            elif mn == 'rfr':
                AR[B + d[0]] = f2b(FR[d[1]])
            elif mn == 'mov.s':
                FR[d[0]] = FR[d[1]]
            elif mn == 'neg.s':
                FR[d[0]] = b2f(f2b(FR[d[1]]) ^ 0x80000000)
            elif mn == 'abs.s':
                FR[d[0]] = b2f(f2b(FR[d[1]]) & 0x7FFFFFFF)
            elif mn == 'add.s':
                FR[d[0]] = r32(FR[d[1]] + FR[d[2]])
            elif mn == 'sub.s':
                FR[d[0]] = r32(FR[d[1]] - FR[d[2]])
            elif mn == 'mul.s':
                FR[d[0]] = r32(FR[d[1]] * FR[d[2]])
            elif mn == 'madd.s':
                FR[d[0]] = fused(FR[d[1]], FR[d[2]], FR[d[0]])
            elif mn == 'msub.s':
                FR[d[0]] = fused(FR[d[1]], FR[d[2]], FR[d[0]], sub=True)
            elif mn == 'float.s':
                v = AR[B + d[1]]; v = v - (1 << 32) if v & 0x80000000 else v
                FR[d[0]] = r32(v / (1 << d[2]))
            elif mn == 'ufloat.s':
                FR[d[0]] = r32(AR[B + d[1]] / (1 << d[2]))
            elif mn in ('trunc.s', 'utrunc.s'):
                x = FR[d[1]] * (1 << d[2])
                if math.isnan(x):
                    v = 0x7FFFFFFF
                else:
                    v = int(x)   # toward zero
                    if mn == 'trunc.s':
                        v = max(min(v, 0x7FFFFFFF), -0x80000000)
                    else:
                        v = max(min(v, 0xFFFFFFFF), 0)
                AR[B + d[0]] = v & M32
            elif mn in ('olt.s', 'ole.s', 'oeq.s', 'ult.s', 'ule.s', 'ueq.s', 'un.s'):
                a, b = FR[d[1]], FR[d[2]]
                un = math.isnan(a) or math.isnan(b)
                BR[d[0]] = {'olt.s': (not un) and a < b, 'ole.s': (not un) and a <= b,
                            'oeq.s': (not un) and a == b, 'ult.s': un or a < b, 'ule.s': un or a <= b,
                            'ueq.s': un or a == b, 'un.s': un}[mn]
            # ---- control
            elif mn in ('j', 'j.l'):
                npc = d[0]
            elif mn in ('beq', 'bne', 'blt', 'bge', 'bltu', 'bgeu'):
                x = AR[B + d[0]]; y = AR[B + d[1]]
                if mn in ('blt', 'bge'):
                    x = x - (1 << 32) if x & 0x80000000 else x
                    y = y - (1 << 32) if y & 0x80000000 else y
                c = {'beq': x == y, 'bne': x != y, 'blt': x < y, 'bge': x >= y, 'bltu': x < y, 'bgeu': x >= y}[mn]
                if c:
                    npc = d[2]
            elif mn in ('beqi', 'bnei', 'blti', 'bgei', 'bltui', 'bgeui'):
                x = AR[B + d[0]]; y = d[1]
                if mn in ('blti', 'bgei', 'beqi', 'bnei'):
                    x = x - (1 << 32) if x & 0x80000000 else x
                else:
                    y &= M32
                c = {'beqi': x == y, 'bnei': x != y, 'blti': x < y, 'bgei': x >= y, 'bltui': x < y,
                     'bgeui': x >= y}[mn]
                if c:
                    npc = d[2]
            elif mn in ('beqz', 'bnez', 'beqz.n', 'bnez.n', 'bltz', 'bgez'):
                x = AR[B + d[0]]; xs = x - (1 << 32) if x & 0x80000000 else x
                c = {'beqz': x == 0, 'beqz.n': x == 0, 'bnez': x != 0, 'bnez.n': x != 0, 'bltz': xs < 0,
                     'bgez': xs >= 0}[mn]
                if c:
                    npc = d[1]
            elif mn in ('bbci', 'bbsi'):
                bit = (AR[B + d[0]] >> d[1]) & 1
                if bit == (mn == 'bbsi'):
                    npc = d[2]
            elif mn in ('bt', 'bf'):
                if BR[d[0]] == (mn == 'bt'):
                    npc = d[1]
            elif mn in ('loop', 'loopnez', 'loopgtz'):
                n = AR[B + d[0]]; ns = n - (1 << 32) if n & 0x80000000 else n
                lbeg, lend = npc, d[1]
                if mn == 'loop':
                    lcount = (n - 1) & M32
                elif (mn == 'loopnez' and n == 0) or (mn == 'loopgtz' and ns <= 0):
                    npc = lend
                    lcount = 0
                else:
                    lcount = n - 1
            elif mn in ('callx8', 'call8'):
                tgt = AR[B + d[0]] if mn == 'callx8' else d[0]
                ret = npc
                if tgt in STUBS:
                    self.stub(STUBS[tgt], AR, B + 8)
                else:
                    AR[B + 8] = ret
                    self.pendingInc = 8
                    npc = tgt
            elif mn == 'entry':
                nb = base + 8
                AR[nb + d[0]] = (AR[base + d[0]] - d[1]) & M32
                base = nb
            elif mn in ('retw', 'retw.n'):
                npc = AR[base + 0]
                if npc == RET_SENTINEL:
                    self.retval = AR[base + 2]
                base -= 8
            elif mn in ('nop', 'nop.n', 'memw'):
                pass
            elif mn in ('rsr', 'rsr.ccount'):
                AR[B + d[0]] = (cyc if model is not None else 0) & M32   # CCOUNT: the model's cycles
            else:
                raise RuntimeError('unimplemented %s at %08x (%s)' % (mn, pc, self.ins[pc][2]))
            if model is not None and npc != nxt.get(pc) and npc is not None:
                cyc += model['taken'] if mn not in ('callx8', 'call8', 'retw', 'retw.n') else model['call']
            # zero-overhead loop
            if lcount > 0 and npc == lend and npc == nxt.get(pc) and mn not in ('loop', 'loopnez', 'loopgtz'):
                lcount -= 1
                npc = lbeg
            pc = npc

    def stub(self, name, AR, a):
        """a: index of the callee's a2 (caller's a10) in AR."""
        x = [AR[a + 2 + i] for i in range(4)]
        if name == 'memset':
            for i in range(x[2]):
                self.st(x[0] + i, 1, x[1])
            r = x[0]
        elif name == 'memcpy':
            for i in range(x[2]):
                self.st(x[0] + i, 1, self.ld(x[1] + i, 1))
            r = x[0]
        elif name == 'sqrtf':
            v = b2f(x[0])
            r = f2b(r32(math.sqrt(v)) if v >= 0 else float('nan'))
        elif name == '__divsf3':
            a_, b_ = b2f(x[0]), b2f(x[1])
            try:
                r = f2b(r32(a_ / b_))
            except ZeroDivisionError:
                r = f2b(float('nan') if a_ == 0 or math.isnan(a_) else math.copysign(float('inf'), a_) *
                        (1 if math.copysign(1, b_) > 0 else -1))
        elif name in ('fminf', 'fmaxf'):
            a_, b_ = b2f(x[0]), b2f(x[1])
            if math.isnan(a_):
                r = x[1]
            elif math.isnan(b_):
                r = x[0]
            else:
                r = x[0] if ((a_ <= b_) if name == 'fminf' else (a_ >= b_)) else x[1]
        AR[a + 2] = r & M32
        self.stubCalls[name] += 1


ST = 0x3FC10000      # ofState_t
FRAME = 0x3FC20000
OUT = 0x3FC30000
SP = 0x3FCF0000


def run_pair(emu, f0, f1, mode):
    emu.setup_mem()
    emu.stubCalls = Counter()
    emu.st(ST + 28801, 1, mode)
    emu.st(ST + 28800, 1, 0)
    total = Counter()
    res = None
    emu.cycles = 0
    for f in (f0, f1):
        c0 = emu.cycles
        emu.mem[FRAME - emu.memBase:FRAME - emu.memBase + 19200] = bytes(f)
        r, c = emu.call('ofProcess', [ST, FRAME, 0, OUT], SP)
        total += c
        emu.lastCycles = emu.cycles - c0
        res = bytes(emu.mem[OUT - emu.memBase:OUT - emu.memBase + 48])
    return res, total, c


def decode_res(b):
    f = struct.unpack('<7fi2fi?3x', b)
    return dict(zip(['tx', 'ty', 'scale', 'rot', 'bias', 'texture', 'residual', 'pixels', 'gain', 'satFrac',
                     'flatPixels', 'ok'], f))
