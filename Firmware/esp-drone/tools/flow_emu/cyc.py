"""Rough in-order LX7 cycle model over an executed trace (xemu): 1 cycle an instruction, a stall until each source
register is ready (load-use, FP latency), and a penalty per taken branch/jump. Estimates only: the ESP32-S3's
pipeline latencies are not documented; run with two parameter sets to bracket."""
import re
from collections import Counter
LOADS = {'l32i', 'l32i.n', 'lsi', 'l8ui', 'l16ui', 'l16si', 'l32r', 'lsx', 'lsip'}
STORES = {'s32i', 's32i.n', 'ssi', 's8i', 's16i', 'ssx', 'ssip'}
FPA = {'add.s', 'sub.s', 'mul.s', 'madd.s', 'msub.s', 'float.s', 'ufloat.s', 'trunc.s', 'utrunc.s', 'wfr', 'rfr'}
ACCUM = {'madd.s', 'msub.s', 'movt', 'movf', 'movt.s', 'movf.s', 'moveqz', 'movnez', 'movltz', 'movgez',
         'moveqz.s', 'movnez.s', 'movltz.s', 'movgez.s'}
SRCONLY = STORES | {'beq', 'bne', 'blt', 'bge', 'bltu', 'bgeu', 'beqi', 'bnei', 'blti', 'bgei', 'bltui', 'bgeui',
                    'beqz', 'bnez', 'beqz.n', 'bnez.n', 'bltz', 'bgez', 'bbci', 'bbsi', 'bt', 'bf', 'loop',
                    'loopnez', 'loopgtz', 'j', 'callx8', 'call8', 'entry', 'retw', 'retw.n', 'nop', 'nop.n', 'memw'}


def regs(ops):
    out = []
    for o in ops:
        m = re.fullmatch(r'([afb])(\d+)', o)
        if m:
            out.append((m.group(1), int(m.group(2))))
    return out


def defuse(mn, ops):
    r = regs(ops)
    if mn in SRCONLY:
        return [], r
    if mn in ('lsip', 'ssip'):
        return ([r[0], r[1]] if mn == 'lsip' else [r[1]]), r[1:] if mn == 'lsip' else r
    if not r:
        return [], []
    d, s = [r[0]], r[1:]
    if mn in ACCUM:
        s = r
    return d, s


def latency(mn, P):
    if mn in LOADS:
        return P['load']
    if mn in FPA:
        return P['fp']
    if mn == 'mull':
        return P['mul']
    if mn.endswith('.s'):   # compares, mov.s, neg.s, abs.s
        return P['fpcmp'] if mn[:2] in ('ol', 'oe', 'ul', 'ue', 'un') else 1
    return 1
