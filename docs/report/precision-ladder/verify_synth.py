#!/usr/bin/env python3
"""敵対的統合検証: hwphase wander のセグメント別 detrended-std と、
oscillator-vs-reception の切り分け補助。座標は読まない。"""
import re, sys, statistics, math

LOG = sys.argv[1]
WARMUP = int(sys.argv[2]) if len(sys.argv) > 2 else 150
MINRAW = int(sys.argv[3]) if len(sys.argv) > 3 else 900

IDEN_MAP = None  # cidx->i_den は呼び出し側で固定して比較

rows = []
for l in open(LOG, errors="ignore"):
    if "PPSGEN" not in l or "count=" not in l:
        continue
    def g(k):
        m = re.search(rf"\b{k}=(-?\d+)\b", l)
        return int(m.group(1)) if m else None
    hw = g("hwphase_ns"); cidx = g("cidx"); lk = g("lk"); raw = g("count")
    if hw is None or cidx is None or lk is None:
        continue
    rows.append(dict(hw=hw, cidx=cidx, lk=lk, raw=raw,
                     interval=g("interval_ns"), slope=g("slope_mppb"),
                     trim=g("trim_mppb"), temp=g("temp_raw"), inj=g("inj_ns")))

# cidx でセグメント分割
segs = []
i = 0
while i < len(rows):
    j = i
    while j < len(rows) and rows[j]["cidx"] == rows[i]["cidx"]:
        j += 1
    segs.append((rows[i]["cidx"], i, j))
    i = j

def detrend_std(vals):
    n = len(vals)
    if n < 10: return None
    xs = list(range(n)); mx = (n-1)/2; my = statistics.mean(vals)
    sxx = sum((x-mx)**2 for x in xs)
    sxy = sum((x-mx)*(v-my) for x,v in zip(xs,vals))
    b = sxy/sxx if sxx else 0
    res = [v-(my+b*(x-mx)) for x,v in zip(xs,vals)]
    return statistics.pstdev(res)

def diffstd(vals):
    if len(vals) < 3: return None
    d = [vals[k+1]-vals[k] for k in range(len(vals)-1)]
    return statistics.pstdev(d)

print(f"# {LOG} warmup={WARMUP} minraw={MINRAW}")
print(f"{'seg#':>4} {'cidx':>4} {'n_all':>6} {'n_use':>6} {'hw_dtstd':>9} {'hw_diffstd':>10} {'slope_std':>9} {'trim_std':>9} {'temp_rng':>8}")
by_cidx = {}
for si,(c,lo,hi) in enumerate(segs):
    use = [rows[k] for k in range(lo,hi) if (k-lo)>=WARMUP and rows[k]["lk"]==1 and (rows[k]["raw"] or 0)>=MINRAW]
    if len(use) < 30:
        print(f"{si:>4} {c:>4} {hi-lo:>6} {len(use):>6}   (skip)")
        continue
    hw=[r["hw"] for r in use]
    slope=[r["slope"] for r in use if r["slope"] is not None]
    trim=[r["trim"] for r in use if r["trim"] is not None]
    temp=[r["temp"] for r in use if r["temp"] is not None]
    dts=detrend_std(hw); dfs=diffstd(hw)
    ss=statistics.pstdev(slope) if len(slope)>2 else float('nan')
    ts=statistics.pstdev(trim) if len(trim)>2 else float('nan')
    trng=(max(temp)-min(temp)) if temp else 0
    print(f"{si:>4} {c:>4} {hi-lo:>6} {len(use):>6} {dts:>9.1f} {dfs:>10.1f} {ss:>9.1f} {ts:>9.1f} {trng:>8}")
    by_cidx.setdefault(c,[]).append(dict(hw_dtstd=dts, hw_diffstd=dfs, slope_std=ss, trim_std=ts))

print("\n# cidx 別 pooled (detrended-std):")
for c in sorted(by_cidx):
    v=[x["hw_dtstd"] for x in by_cidx[c]]
    sl=[x["slope_std"] for x in by_cidx[c] if not math.isnan(x["slope_std"])]
    tr=[x["trim_std"] for x in by_cidx[c] if not math.isnan(x["trim_std"])]
    df=[x["hw_diffstd"] for x in by_cidx[c]]
    print(f"  cidx={c} n={len(v)} hw_dtstd mean={statistics.mean(v):.1f} sd={statistics.pstdev(v):.1f} vals={[round(x) for x in v]}")
    print(f"           hw_diffstd(HF floor) mean={statistics.mean(df):.1f}  slope_std mean={statistics.mean(sl):.1f} trim_std mean={statistics.mean(tr):.1f}")

# 全 use 行プールで相関: hwphase detrended-std(セグメント) vs slope_std, trim_std
allsegs=[x for v in by_cidx.values() for x in v]
def corr(a,b):
    n=len(a); ma=statistics.mean(a); mb=statistics.mean(b)
    num=sum((x-ma)*(y-mb) for x,y in zip(a,b))
    da=math.sqrt(sum((x-ma)**2 for x in a)); db=math.sqrt(sum((y-mb)**2 for y in b))
    return num/(da*db) if da*db else float('nan')
A=[x["hw_dtstd"] for x in allsegs]
S=[x["slope_std"] for x in allsegs]
T=[x["trim_std"] for x in allsegs]
print(f"\n# corr(hw_dtstd, slope_std)={corr(A,S):+.2f}  corr(hw_dtstd, trim_std)={corr(A,T):+.2f}  (n={len(A)} segs)")
