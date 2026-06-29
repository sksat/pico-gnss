#!/usr/bin/env python3
"""Test the session-start / time-drift confound on hwphase wander.

(1) Sliding-window SD across the WHOLE session ignoring config: is there a global
    time trend (receiver noise drifting over the session) that rivals config effect?
(2) Neighbor-differenced paired test: pair each segment's SD with the mean of its
    immediate neighbors (other configs) to cancel slow drift, then test if a config
    sits below local baseline.
"""
import re, sys, statistics, math, random
LOG = sys.argv[1]
WARMUP = int(sys.argv[2]) if len(sys.argv) > 2 else 150
random.seed(2)

cfg = {}; rows = []
for l in open(LOG, errors="ignore"):
    m = re.search(r"CTRLSWEEP idx=(\d+).*i_den=(\d+)", l)
    if m:
        cfg[int(m.group(1))] = int(m.group(2)); continue
    if "PPSGEN" not in l or "hwphase_ns" not in l: continue
    t = re.match(r"\s*([\d.]+)", l)
    hw = re.search(r"\bhwphase_ns=(-?\d+)\b", l)
    cidx = re.search(r"\bcidx=(\d+)\b", l)
    lk = re.search(r"\blk=(-?\d+)\b", l)
    if not (t and hw and cidx and lk): continue
    ci = int(cidx.group(1))
    rows.append((float(t.group(1)), int(hw.group(1)), ci, int(lk.group(1)), cfg.get(ci,0)))

# (1) global sliding-window SD over locked edges, ignoring config
locked = [(r[0], r[1], r[4]) for r in rows if r[3] == 1]
W = 400; STEP = 400
print(f"# (1) global sliding-window SD (W={W} locked edges), ignoring config:")
print(f"#   t_start  iden_majority   sd_window")
i = 0
while i + W <= len(locked):
    win = locked[i:i+W]
    sd = statistics.pstdev([x[1] for x in win])
    idens = [x[2] for x in win]
    maj = max(set(idens), key=idens.count)
    print(f"   {win[0][0]:8.0f}  iden={maj:<5}      {sd:6.1f}")
    i += STEP

# segment SDs in time order
segs = []; i = 0
while i < len(rows):
    j = i
    while j < len(rows) and rows[j][2] == rows[i][2]: j += 1
    segs.append((i, j)); i = j
segrecs = []
for (lo, hi) in segs:
    hw = [rows[k][1] for k in range(lo+WARMUP, hi) if rows[k][3]==1]
    if (hi-lo) >= 900 and len(hw) > 2:
        segrecs.append((rows[lo][4], statistics.pstdev(hw)))  # (iden, sd) in order

print(f"\n# (2) segment SDs in time order: " + " ".join(f"{i}:{s:.0f}" for i,s in segrecs))
# neighbor-differenced: sd[k] - mean(sd[k-1], sd[k+1]) for interior segments
print(f"# neighbor-differenced residual (sd - local mean of adjacent segs), by iden:")
resid = {}
for k in range(1, len(segrecs)-1):
    iden = segrecs[k][0]
    base = (segrecs[k-1][1] + segrecs[k+1][1]) / 2
    r = segrecs[k][1] - base
    resid.setdefault(iden, []).append(r)
for iden in sorted(resid):
    rs = resid[iden]
    m = statistics.mean(rs)
    print(f"   iden={iden:<5} n={len(rs)} residuals={['%+.0f'%x for x in rs]} mean={m:+.1f}")
