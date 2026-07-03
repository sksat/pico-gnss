#!/usr/bin/env python3
"""Decisive paired test: production i_den=512 vs alternatives (128, 1024), FF on.
Adjacent-in-time pairs (drift-cancelling), sign test, paired bootstrap, and
block-reproducibility (does the 512-worse pattern hold in BOTH ABBA super-blocks?).
"""
import re, sys, statistics, random
LOG = sys.argv[1]; WARMUP = 150; random.seed(3)
cfg = {}; rows = []
for l in open(LOG, errors="ignore"):
    m = re.search(r"CTRLSWEEP idx=(\d+).*i_den=(\d+)", l)
    if m: cfg[int(m.group(1))] = int(m.group(2)); continue
    if "PPSGEN" not in l or "hwphase_ns" not in l: continue
    t = re.match(r"\s*([\d.]+)", l); hw = re.search(r"hwphase_ns=(-?\d+)", l)
    cidx = re.search(r"\bcidx=(\d+)", l); lk = re.search(r"\blk=(-?\d+)", l)
    if not (t and hw and cidx and lk): continue
    ci = int(cidx.group(1)); rows.append((float(t.group(1)), int(hw.group(1)), ci, int(lk.group(1)), cfg.get(ci,0)))
segs = []; i = 0
while i < len(rows):
    j = i
    while j < len(rows) and rows[j][2] == rows[i][2]: j += 1
    segs.append((i,j)); i = j
order = []  # (iden, sd, t0)
for (lo,hi) in segs:
    hw = [rows[k][1] for k in range(lo+WARMUP,hi) if rows[k][3]==1]
    if (hi-lo)>=900 and len(hw)>2:
        order.append((rows[lo][4], statistics.pstdev(hw), rows[lo][0]))
print("# segments in time order (iden:sd):", " ".join(f"{a}:{b:.0f}" for a,b,_ in order))
# split into 2 super-blocks by index (first 6, rest) for reproducibility check
def adj_pairs(seq, A, B):
    """signed diff sd[A]-sd[B] for consecutive segments where the two are {A,B}."""
    out = []
    for k in range(len(seq)-1):
        i0,s0,_ = seq[k]; i1,s1,_ = seq[k+1]
        if {i0,i1} == {A,B}:
            dA = s0 if i0==A else s1; dB = s0 if i0==B else s1
            out.append(dA-dB)
    return out
for (A,B,label) in [(512,1024,"prod512 vs 1024"),(512,128,"prod512 vs 128"),(128,1024,"128 vs 1024")]:
    d = adj_pairs(order, A, B)
    if not d: print(f"\n{label}: no adjacent pairs"); continue
    pos = sum(1 for x in d if x>0); n=len(d)
    boot=[]
    for _ in range(20000):
        boot.append(statistics.mean([random.choice(d) for _ in d]))
    boot.sort(); lo=boot[500]; hi=boot[19500]
    sig = "SIG(excl 0)" if (lo>0 or hi<0) else "ns(incl 0)"
    print(f"\n{label}: adj pairs (sd_{A}-sd_{B}) = {['%+.0f'%x for x in d]}")
    print(f"   n={n} mean={statistics.mean(d):+.1f} sign:{pos}/{n}>0 (one-sided p={0.5**n if pos==n or pos==0 else '>'.join([''])+'%.3f'%(sum(__import__('math').comb(n,k) for k in range(pos,n+1))/2**n)})  bootCI95[{lo:+.0f},{hi:+.0f}] {sig}")
    # block reproducibility
    half = len(order)//2
    b1 = adj_pairs(order[:half+1], A, B); b2 = adj_pairs(order[half:], A, B)
    print(f"   block1 pairs={['%+.0f'%x for x in b1]}  block2 pairs={['%+.0f'%x for x in b2]}")
