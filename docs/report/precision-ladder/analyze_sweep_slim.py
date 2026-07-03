#!/usr/bin/env python3
"""I_DEN_SWEEP(d_den=4 固定)の per-iden 振幅解析 — scipy 不要版。

同一 boot・同一受信で iden を 128→256→512 と巡回したログから、iden 別の hwphase 振幅(clean sd)と
共振ディップ(autocorr 最深負)を出し、受信交絡を避けるため **1 cycle(128,256,512 が時間的に隣接)内の
比** も併記する。座標を含む NMEA 行は読み捨て(PPSGEN のみ解析、座標は一切出力しない)。

  python3 docs/report/analyze_sweep_slim.py logs/pps-sweep-dden4.log
"""
import re, sys, math, statistics

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/pps-sweep-dden4.log"
SETTLE_PERIODS = 1.2  # iden 変更直後 ~1.2 周期(2π√iden edge)を整定として捨てる

rows = []  # (t, hw, lk, iden, slope)
for line in open(LOG, errors="ignore"):
    if "PPSGEN" not in line:
        continue  # NMEA(座標)等は読み捨て
    hw = re.search(r"\bhwphase_ns=(-?\d+)\b", line)
    lk = re.search(r"\blk=(-?\d+)\b", line)
    idn = re.search(r"\biden=(\d+)\b", line)
    sl = re.search(r"\bslope_mppb=(-?\d+)\b", line)
    t = re.match(r"\s*([\d.]+)", line)
    if hw and lk and idn and t:
        rows.append((float(t.group(1)), int(hw.group(1)), int(lk.group(1)),
                     int(idn.group(1)), int(sl.group(1)) if sl else 0))
if not rows:
    sys.exit("no PPSGEN iden= rows yet")

span = (rows[-1][0] - rows[0][0]) / 60
print(f"# {LOG}: {len(rows)} PPSGEN rows, span {span:.1f} min")

# contiguous segments by iden
segs = []
i = 0
while i < len(rows):
    j = i
    while j < len(rows) and rows[j][3] == rows[i][3]:
        j += 1
    segs.append((rows[i][3], i, j))
    i = j
print("# segments: " + " ".join(f"{v}({h-l}e)" for v, l, h in segs))


def autocorr_dip(x):
    n = len(x)
    if n < 60:
        return float("nan"), 0
    m = statistics.mean(x); xc = [v - m for v in x]
    den = sum(v * v for v in xc) or 1.0
    best, bl = 1.0, 0
    for lag in range(8, min(160, n // 2)):
        ac = sum(xc[k] * xc[k + lag] for k in range(n - lag)) / den
        if ac < best:
            best, bl = ac, lag
    return best, bl


def clean_seg(lo, hi, iden):
    drop = int(SETTLE_PERIODS * 2 * math.pi * math.sqrt(iden))
    vals = [(rows[k][1], rows[k][4]) for k in range(lo + drop, hi)
            if rows[k][2] == 1 and abs(rows[k][1]) <= 5000]
    return vals


# per-iden aggregate
by_iden = {}
seg_sds = {}
for iden, lo, hi in segs:
    vals = clean_seg(lo, hi, iden)
    if len(vals) < 30:
        continue
    hwv = [v[0] for v in vals]
    by_iden.setdefault(iden, []).extend(hwv)
    seg_sds.setdefault(iden, []).append(statistics.pstdev(hwv))

print(f"\n{'iden':>6} {'n':>6} {'sd_ns':>7} {'seg_sd中央':>10} {'dip':>7} {'@lag':>5} {'slope中央':>9}")
for iden in sorted(by_iden):
    hwv = by_iden[iden]
    dip, lag = autocorr_dip(hwv)
    ssd = statistics.median(seg_sds[iden])
    sl = statistics.median([v for k in range(len(rows)) for v in [rows[k][4]] if rows[k][3] == iden])
    dips = f"{dip:.2f}" if dip == dip else "  -  "
    print(f"{iden:>6} {len(hwv):>6} {statistics.pstdev(hwv):>7.0f} {ssd:>10.0f} {dips:>7} {lag:>5} {sl:>9.0f}")

# matched-cycle: 各 cycle(連続する 128,256,512)の sd を並べ、受信を揃えた比を見る
print("\n# matched cycle (時間的に隣接=受信ほぼ同一の per-cycle sd[ns]):")
print(f"{'cycle':>5}  " + "  ".join(f"{v:>7}" for v in (128, 256, 512)))
cyc = {}
ci = 0
seen = set()
for iden, lo, hi in segs:
    vals = clean_seg(lo, hi, iden)
    if len(vals) < 30:
        continue
    sd = statistics.pstdev([v[0] for v in vals])
    if iden in seen:
        ci += 1; seen = set()
    seen.add(iden)
    cyc.setdefault(ci, {})[iden] = sd
for c in sorted(cyc):
    r = cyc[c]
    print(f"{c:>5}  " + "  ".join(f"{r.get(v, float('nan')):>7.0f}" if v in r else f"{'-':>7}" for v in (128, 256, 512)))
