# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""出力位相 wander (hwphase) の素性を密に解析する。

目的: 「数百ns の遅い振動」が (a) どの周期帯にあるか、(b) clean-lock/平温でも残るか (受信律速 vs
relock 過渡 vs 熱)、(c) 受信プロキシ (interval jitter / HDOP) とどれだけ相関するか、(d) slope/trim
(α-β FF / PLL I) と相関するか を数値で出す。位相ループ帯域 (loop corner ~35s @ i_den=32) に対して
wander がどの帯にあるかが「ループを緩めれば削れるか」の判定材料になる。

NMEA の座標は一切読まない (PPS/PPSGEN/TIME/GSA の数値フィールドのみ)。GSA は DOP のみで座標を含まない。

  uv run docs/report/analyze_wander.py logs/pps-review.log [out_prefix]
"""
import re
import sys

import numpy as np
from scipy.signal import lombscargle

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/pps-review.log"
PREFIX = sys.argv[2] if len(sys.argv) > 2 else "docs/report/wander"

TS = re.compile(r"^(\d+\.\d+)\s")


def kv(line, key, typ=int):
    m = re.search(key + r"=(-?\d+)", line)
    return typ(m.group(1)) if m else None


gen = []   # (t, hw, lk, trim_mppb, slope_mppb, p, d, raw_lag)
pps = []   # (t, dev_ns, locked_state)
time = []  # (t, ppb)
gsa = []   # (t, pdop, hdop)

for line in open(LOG, errors="ignore"):
    m = TS.match(line)
    if not m:
        continue
    t = float(m.group(1))
    if "PPSGEN count=" in line:
        hw = kv(line, "hwphase_ns")
        lk = kv(line, "lk")
        if hw is None or lk is None:
            continue
        gen.append((t, hw, lk, kv(line, "trim_mppb") or 0, kv(line, "slope_mppb") or 0,
                    kv(line, "p_ns") or 0, kv(line, "d_ns") or 0, kv(line, "raw_lag") or 0))
    elif "] PPS count=" in line:
        iv = kv(line, "interval_ns")
        if iv is not None:
            st = "Locked" if "state=Locked" in line else "other"
            pps.append((t, iv - 1_000_000_000, st))
    elif "TIME unix_ns=" in line:
        p = kv(line, "ppb")
        if p is not None:
            time.append((t, p))
    elif "GNGSA" in line or "GPGSA" in line:
        # $..GSA,A,3,<12 sat slots>,PDOP,HDOP,VDOP*cs  — DOP は末尾3数値。座標なし。
        body = line.split("GSA,", 1)[1].split("*")[0] if "GSA," in line else ""
        nums = re.findall(r"\d+\.\d+", body)
        if len(nums) >= 2:
            gsa.append((t, float(nums[-3]) if len(nums) >= 3 else float("nan"), float(nums[-2])))

gen = np.array(gen, float)
if len(gen) == 0:
    sys.exit("no PPSGEN data")
print(f"# log={LOG}  PPSGEN={len(gen)} PPS={len(pps)} TIME={len(time)} GSA={len(gsa)}")
print(f"# span = {(gen[-1,0]-gen[0,0])/60:.1f} min")

t = gen[:, 0]
hw = gen[:, 1]
lk = gen[:, 2].astype(int)
trim = gen[:, 3]
slope = gen[:, 4]

# --- clean locked series: lk=1, drop startup (<90s) and gross spikes (|hw|>5µs = recal/holdover) ---
t0 = t[0]
good = (lk == 1) & (t - t0 > 90) & (np.abs(hw) < 5000)
tg, hwg = t[good], hw[good]
print(f"\n## locked-clean samples n={good.sum()} ({100*good.sum()/len(gen):.0f}% of edges)")
print(f"## hwphase: mean={hwg.mean():.0f}ns  sd={hwg.std():.0f}ns  p2p={hwg.max()-hwg.min():.0f}ns")
print(f"## lk=1 fraction overall = {100*(lk==1).mean():.0f}%  (unlock rate = {100*(lk==0).mean():.0f}%)")

# --- variance vs averaging window (does sd keep growing with window => slow wander?) ---
print("\n## rolling-sd vs window (locked-clean) -- how slow is the wander")
for w in (5, 15, 30, 60, 120, 300, 600):
    sds = []
    for i in range(len(tg)):
        seg = hwg[(tg >= tg[i] - w / 2) & (tg <= tg[i] + w / 2)]
        if len(seg) >= max(4, w // 4):
            sds.append(seg.std())
    if sds:
        print(f"   win={w:4d}s  median rolling-sd = {np.median(sds):4.0f}ns   (n_win={len(sds)})")

# --- Lomb-Scargle spectrum (irregular sampling) over periods 4..2000s ---
y = hwg - hwg.mean()
periods = np.geomspace(4, 2000, 400)
ang = 2 * np.pi / periods
power = lombscargle(tg, y, ang, normalize=True)
pk = periods[np.argmax(power)]
print(f"\n## Lomb-Scargle peak period = {pk:.0f}s")
order = np.argsort(power)[::-1]
seen = []
for idx in order:
    P = periods[idx]
    if all(abs(np.log(P / s)) > 0.35 for s in seen):
        seen.append(P)
    if len(seen) >= 4:
        break
print("## top periods (s): " + ", ".join(f"{P:.0f}(pwr={power[periods.tolist().index(P)]:.2f})" for P in seen))

# variance fraction by period band (via band-limited LS power integral approximation)
bands = [(4, 16), (16, 64), (64, 256), (256, 1024), (1024, 2000)]
tot = power.sum()
print("## power fraction by period band:")
for lo, hi in bands:
    sel = (periods >= lo) & (periods < hi)
    print(f"   {lo:5d}-{hi:<5d}s : {100*power[sel].sum()/tot:4.0f}%")

# --- clean-lock continuous runs (no unlock) vs relock transients ---
print("\n## relock transient vs steady (edge-ordered runs of lk=1)")
runs = []
i = 0
while i < len(lk):
    if lk[i] == 1:
        j = i
        while j < len(lk) and lk[j] == 1:
            j += 1
        runs.append((i, j))
        i = j
    else:
        i += 1
long_runs = [(a, b) for a, b in runs if b - a >= 120 and t[a] - t0 > 90]
steady, transient = [], []
for a, b in long_runs:
    seg_hw = hw[a:b]
    seg_ok = np.abs(seg_hw) < 5000
    transient.append(seg_hw[:20][np.abs(seg_hw[:20]) < 5000])
    steady.append(seg_hw[20:][np.abs(seg_hw[20:]) < 5000])
steady = np.concatenate(steady) if steady else np.array([])
transient = np.concatenate(transient) if transient else np.array([])
print(f"   long clean runs (>=120 edges): {len(long_runs)}")
if len(steady):
    print(f"   STEADY (>=20 edges into a clean run): n={len(steady)} sd={steady.std():.0f}ns mean={steady.mean():.0f}ns")
if len(transient):
    print(f"   first-20-edges-after-relock:          n={len(transient)} sd={transient.std():.0f}ns mean={transient.mean():.0f}ns")

# --- flat-temp subset: |d ppb/dt| small ---
if time:
    ta = np.array([x[0] for x in time]); pa = np.array([x[1] for x in time])
    ppb_at = np.interp(tg, ta, pa)
    dppb = np.gradient(ppb_at, tg)  # ppb/s
    flat = np.abs(dppb) < 0.3
    print(f"\n## flat-temp (|dppb/dt|<0.3 ppb/s) within locked-clean: {100*flat.mean():.0f}% of samples")
    if flat.sum() > 30:
        print(f"   wander sd in flat-temp&clean = {hwg[flat].std():.0f}ns   vs ramp {hwg[~flat].std():.0f}ns")

# --- reception proxies & control-signal correlations vs rolling |hwphase| sd ---
print("\n## correlations with rolling hwphase sd(30s)  (does reception/control predict the wander?)")
# rolling hwphase sd at each locked-clean sample
roll = np.array([hwg[(tg >= tg[i]-15) & (tg <= tg[i]+15)].std()
                 if ((tg >= tg[i]-15) & (tg <= tg[i]+15)).sum() >= 5 else np.nan for i in range(len(tg))])
def corr(a, b):
    ok = np.isfinite(a) & np.isfinite(b)
    return np.corrcoef(a[ok], b[ok])[0, 1] if ok.sum() > 10 else float("nan")
# interval jitter proxy from PPS dev
if pps:
    tp = np.array([x[0] for x in pps]); dv = np.array([x[1] for x in pps], float)
    # exclude gross interval outliers (gaps) for the jitter proxy
    dv = np.where(np.abs(dv) < 100000, dv, np.nan)
    ijit = np.array([np.nanstd(dv[(tp >= tp[i]-15) & (tp <= tp[i]+15)]) for i in range(len(tp))])
    ijit_g = np.interp(tg, tp, ijit)
    print(f"   corr(rolling hwphase-sd , PPS interval-jitter-15s) = {corr(roll, ijit_g):+.2f}")
    print(f"   median PPS interval jitter (15s) = {np.nanmedian(ijit):.0f}ns")
if gsa:
    tgsa = np.array([x[0] for x in gsa]); hdop = np.array([x[2] for x in gsa])
    hdop_g = np.interp(tg, tgsa, hdop)
    print(f"   corr(rolling hwphase-sd , HDOP)                    = {corr(roll, hdop_g):+.2f}")
    print(f"   HDOP range = {np.nanmin(hdop):.1f}..{np.nanmax(hdop):.1f} (median {np.nanmedian(hdop):.1f})")
print(f"   corr(hwphase , slope_mppb)  = {corr(hwg, slope[good]):+.2f}   (α-β FF re-injection?)")
print(f"   corr(hwphase , trim_mppb)   = {corr(hwg, trim[good]):+.2f}   (PLL I)")

# --- figures ---
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(11, 7))
    ax[0].plot(periods, power, lw=1)
    ax[0].axvline(35, color="r", ls="--", lw=1, label="loop corner ~35s (i_den=32)")
    ax[0].axvline(pk, color="g", ls=":", lw=1, label=f"peak {pk:.0f}s")
    ax[0].set_xscale("log"); ax[0].set_xlabel("period [s]"); ax[0].set_ylabel("LS power")
    ax[0].set_title(f"hwphase wander spectrum  ({LOG}, sd={hwg.std():.0f}ns)"); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].scatter((tg-t0)/60, hwg, s=4, alpha=.4)
    ax[1].set_xlabel("t [min]"); ax[1].set_ylabel("hwphase [ns]"); ax[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(PREFIX + "-spectrum.png", dpi=110)
    print(f"\n# saved {PREFIX}-spectrum.png")
except Exception as e:
    print("fig skipped:", e)
