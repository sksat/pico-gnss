#!/usr/bin/env python3
"""Per-segment hwphase wander (SD) by controller config, ABBA-aware.

Parses PPSGEN rows, splits into contiguous-cidx segments, drops warmup + unlocked,
computes per-segment SD/mean of hwphase. Groups by cidx. Reports per-segment so
ABBA paired analysis can be done downstream. Coordinates (NMEA) never read.
"""
import re, sys, statistics, math

LOG = sys.argv[1]
WARMUP = int(sys.argv[2]) if len(sys.argv) > 2 else 200  # edges to drop after switch (settle)
MIN_N = int(sys.argv[3]) if len(sys.argv) > 3 else 1     # min steady edges to keep a seg

rows = []  # (t, hw, cidx, lk, inj, kick, iden, kp, se)
# track latest CTRLSWEEP config keyed by idx
cfg = {}  # idx -> (iden, kp, se)
for l in open(LOG, errors="ignore"):
    m = re.search(r"CTRLSWEEP idx=(\d+).*i_den=(\d+).*kp_inv=(\d+).*smith_edges=(\d+)", l)
    if m:
        cfg[int(m.group(1))] = (int(m.group(2)), int(m.group(3)), int(m.group(4)))
        continue
    if "PPSGEN" not in l or "hwphase_ns" not in l:
        continue
    t = re.match(r"\s*([\d.]+)", l)
    hw = re.search(r"\bhwphase_ns=(-?\d+)\b", l)
    cidx = re.search(r"\bcidx=(\d+)\b", l)
    lk = re.search(r"\blk=(-?\d+)\b", l)
    inj = re.search(r"\binj_ns=(-?\d+)\b", l)
    kk = re.search(r"\bkick_ns=(-?\d+)\b", l)
    if not (t and hw and cidx and lk):
        continue
    ci = int(cidx.group(1))
    iden, kp, se = cfg.get(ci, (0, 0, 0))
    rows.append((float(t.group(1)), int(hw.group(1)), ci, int(lk.group(1)),
                 int(inj.group(1)) if inj else 0, int(kk.group(1)) if kk else 0,
                 iden, kp, se))

# split into contiguous-cidx segments
segs = []
i = 0
while i < len(rows):
    j = i
    while j < len(rows) and rows[j][2] == rows[i][2]:
        j += 1
    segs.append((i, j))
    i = j

def stats_seg(lo, hi):
    # steady hwphase: lk==1, after warmup. exclude PRBS-injected edges if any inj!=0
    hw = []
    for k in range(lo, hi):
        if k - lo < WARMUP:
            continue
        if rows[k][3] != 1:
            continue
        hw.append(rows[k][1])
    return hw

print(f"# {LOG}  warmup={WARMUP} min_n={MIN_N}")
print(f"{'seg':>3} {'cidx':>4} {'i_den':>5} {'kp':>3} {'se':>3} {'n':>6} "
      f"{'mean':>8} {'sd':>8} {'p2p':>8} {'dur_s':>7}")
seg_recs = []
for si, (lo, hi) in enumerate(segs):
    iden, kp, se = rows[lo][6], rows[lo][7], rows[lo][8]
    cidx = rows[lo][2]
    hw = stats_seg(lo, hi)
    dur = rows[hi-1][0] - rows[lo][0]
    if len(hw) < 2:
        print(f"{si:>3} {cidx:>4} {iden:>5} {kp:>3} {se:>3} {len(hw):>6} {'--':>8} {'--':>8} {'--':>8} {dur:>7.0f}")
        continue
    mean = statistics.mean(hw); sd = statistics.pstdev(hw)
    p2p = max(hw) - min(hw)
    seg_recs.append((si, cidx, iden, kp, se, len(hw), mean, sd, p2p))
    print(f"{si:>3} {cidx:>4} {iden:>5} {kp:>3} {se:>3} {len(hw):>6} "
          f"{mean:>8.0f} {sd:>8.1f} {p2p:>8.0f} {dur:>7.0f}")

# group by cidx, only segs with n>=MIN_N
print(f"\n# per-cidx pooled (segments with steady n>={MIN_N}):")
print(f"{'cidx':>4} {'i_den':>5} {'kp':>3} {'se':>3} {'Nseg':>5} {'sds':>30} {'mean_sd':>8} {'med_sd':>8}")
by_cidx = {}
for r in seg_recs:
    if r[5] >= MIN_N:
        by_cidx.setdefault(r[1], []).append(r)
for cidx in sorted(by_cidx):
    rs = by_cidx[cidx]
    sds = [r[7] for r in rs]
    iden, kp, se = rs[0][2], rs[0][3], rs[0][4]
    sstr = " ".join(f"{s:.0f}" for s in sds)
    msd = statistics.mean(sds)
    medsd = statistics.median(sds)
    print(f"{cidx:>4} {iden:>5} {kp:>3} {se:>3} {len(rs):>5} {sstr:>30} {msd:>8.1f} {medsd:>8.1f}")
