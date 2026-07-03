#!/usr/bin/env python3
"""ABBA paired / bootstrap stats on per-segment hwphase wander (robust + raw SD).

Reports, per segment: raw n, steady n, pstdev, robust sigma (1.4826*MAD), p2p.
Then per-cidx pooling, paired adjacent comparisons, sign test, and a paired
bootstrap CI on the difference of mean-SD between two cidx. Also a power calc:
n segments needed to detect a given effect at the observed segment-noise.
"""
import re, sys, statistics, math, random

LOG = sys.argv[1]
WARMUP = int(sys.argv[2]) if len(sys.argv) > 2 else 150
MIN_RAW = int(sys.argv[3]) if len(sys.argv) > 3 else 900
random.seed(1)

cfg = {}
rows = []
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
    if not (t and hw and cidx and lk):
        continue
    ci = int(cidx.group(1))
    iden = cfg.get(ci, (0, 0, 0))[0]
    rows.append((float(t.group(1)), int(hw.group(1)), ci, int(lk.group(1)), iden))

segs = []
i = 0
while i < len(rows):
    j = i
    while j < len(rows) and rows[j][2] == rows[i][2]:
        j += 1
    segs.append((i, j))
    i = j

def robust_sigma(xs):
    med = statistics.median(xs)
    mad = statistics.median([abs(x - med) for x in xs])
    return 1.4826 * mad

recs = []  # (order, cidx, iden, raw_n, steady_n, sd, rsig, p2p, tmid)
print(f"# {LOG} warmup={WARMUP} min_raw={MIN_RAW}")
print(f"{'#':>2} {'cidx':>4} {'iden':>5} {'rawn':>5} {'n':>5} {'sd':>7} {'rsig':>7} {'p2p':>6}")
order = 0
for (lo, hi) in segs:
    raw_n = hi - lo
    hw = [rows[k][1] for k in range(lo + WARMUP, hi) if rows[k][3] == 1]
    cidx = rows[lo][2]; iden = rows[lo][4]
    if len(hw) < 2:
        continue
    sd = statistics.pstdev(hw); rsig = robust_sigma(hw); p2p = max(hw) - min(hw)
    tmid = (rows[lo][0] + rows[hi-1][0]) / 2
    keep = raw_n >= MIN_RAW
    recs.append((order, cidx, iden, raw_n, len(hw), sd, rsig, p2p, tmid, keep))
    flag = "" if keep else "  <DROP raw<%d>" % MIN_RAW
    print(f"{order:>2} {cidx:>4} {iden:>5} {raw_n:>5} {len(hw):>5} {sd:>7.1f} {rsig:>7.1f} {p2p:>6.0f}{flag}")
    order += 1

kept = [r for r in recs if r[9]]
by = {}
for r in kept:
    by.setdefault(r[1], []).append(r)

print(f"\n# per-cidx (kept segments), SD and robust-sigma:")
print(f"{'cidx':>4} {'iden':>5} {'N':>3} {'mean_sd':>8} {'sd_of_sd':>9} {'SE':>6} {'mean_rsig':>9}")
means = {}
for cidx in sorted(by):
    sds = [r[5] for r in by[cidx]]
    rsigs = [r[6] for r in by[cidx]]
    m = statistics.mean(sds)
    sdsd = statistics.pstdev(sds) if len(sds) > 1 else 0.0
    se = sdsd / math.sqrt(len(sds)) if len(sds) > 0 else 0.0
    means[cidx] = (m, sds, rsigs)
    print(f"{cidx:>4} {by[cidx][0][2]:>5} {len(sds):>3} {m:>8.1f} {sdsd:>9.1f} {se:>6.1f} {statistics.mean(rsigs):>9.1f}")

def boot_diff(a, b, B=20000):
    """bootstrap CI for mean(a)-mean(b), unpaired (per-cidx segment SD samples)."""
    diffs = []
    for _ in range(B):
        ra = statistics.mean([random.choice(a) for _ in a])
        rb = statistics.mean([random.choice(b) for _ in b])
        diffs.append(ra - rb)
    diffs.sort()
    lo = diffs[int(0.025 * B)]; hi = diffs[int(0.975 * B)]
    p_ge0 = sum(1 for d in diffs if d >= 0) / B
    return statistics.mean(diffs), lo, hi, p_ge0

print(f"\n# unpaired bootstrap diff of mean-SD between cidx pairs (neg => first is lower/better):")
cidxs = sorted(means)
for i in range(len(cidxs)):
    for j in range(i+1, len(cidxs)):
        a = means[cidxs[i]][1]; b = means[cidxs[j]][1]
        d, lo, hi, p = boot_diff(a, b)
        sig = "SIG" if (lo > 0 or hi < 0) else "ns"
        print(f"  cidx{cidxs[i]}(iden{by[cidxs[i]][0][2]}) - cidx{cidxs[j]}(iden{by[cidxs[j]][0][2]}): "
              f"d={d:+6.1f}  95%CI[{lo:+6.1f},{hi:+6.1f}]  {sig}")

# power: n segments needed to detect effect E at observed pooled segment-noise
all_sds = [r[5] for r in kept]
sigma = statistics.pstdev(all_sds)
print(f"\n# pooled segment-level SD noise (sd of per-seg SD) = {sigma:.1f} ns")
for E in (10, 15, 20, 30):
    # two-sample, want |d|>~2*SE_diff; SE_diff = sigma*sqrt(2/n); need n ~ 2*(2*sigma/E)^2*... use n=2*(z*sigma*sqrt2/E)^2
    z = 1.96
    n = 2 * (z * sigma * math.sqrt(2) / E) ** 2  # rough per-group n for 50% power-ish (alpha .05)
    print(f"  effect {E:>2}ns: ~{n:5.0f} segments/config for 95% CI to exclude 0 (alpha .05, naive)")
