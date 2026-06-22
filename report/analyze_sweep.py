# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy"]
# ///
"""I_DEN_SWEEP の per-i_den 振幅解析: 1 boot・固定 shift=0・同一受信で i_den を掃引したログから、
各 i_den セグメントの hwphase 振幅と卓越周期を出し、振幅 vs i_den の真の法則を confound 抜きで測る。

各 PPSGEN 行の iden= でセグメントを切り、i_den 変更直後の整定 (~1 周期 = 2π√i_den edge) を捨て、
lk=1 かつ |hw|<5µs の clean サンプルで集計する。同一 i_den の全セグメントを束ねて sd と周期を出し、
セグメント毎 sd も併記して受信ゆらぎの効きを見る。

  uv run report/analyze_sweep.py logs/pps-idensweep.log
"""
import re, sys, math
import numpy as np
from scipy.signal import lombscargle

LOG = sys.argv[1] if len(sys.argv) > 1 else "logs/pps-idensweep.log"

rows = []  # (t, hw, lk, iden)
for line in open(LOG, errors="ignore"):
    m = re.match(r"(\d+\.\d+).*PPSGEN count.*hwphase_ns=(-?\d+).*lk=(\d) iden=(\d+)", line)
    if m:
        rows.append((float(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))))
if not rows:
    sys.exit("no PPSGEN iden= rows (need the sweep firmware)")
t = np.array([r[0] for r in rows]); hw = np.array([r[1] for r in rows], float)
lk = np.array([r[2] for r in rows]); iden = np.array([r[3] for r in rows])
print(f"# {LOG}: {len(rows)} PPSGEN rows, span {(t[-1]-t[0])/60:.1f} min")

# split into contiguous segments by iden value (edge-ordered)
segs = []  # (iden, lo, hi)
i = 0
while i < len(iden):
    j = i
    while j < len(iden) and iden[j] == iden[i]:
        j += 1
    segs.append((int(iden[i]), i, j))
    i = j
print(f"# {len(segs)} segments: " + " ".join(f"{v}({h-l}e)" for v, l, h in segs))

def period(ts, ys):
    if len(ys) < 20:
        return float("nan")
    per = np.geomspace(8, 400, 300)
    p = lombscargle(ts, ys - ys.mean(), 2 * np.pi / per, normalize=True)
    return per[np.argmax(p)]

by_iden = {}
print(f"\n{'i_den':>6} {'settle':>6} {'n':>5} {'sd_ns':>7} {'mean':>6} {'period_s':>9}   per-segment sd")
for v, lo, hi in segs:
    settle = min(int(2 * math.pi * math.sqrt(v)), (hi - lo) // 2)  # drop ~1 natural period
    s, e = lo + settle, hi
    sel = np.arange(s, e)
    sel = sel[(lk[sel] == 1) & (np.abs(hw[sel]) < 5000)]
    if len(sel) < 10:
        print(f"{v:>6} {settle:>6} {len(sel):>5}   (too few clean samples)")
        continue
    sd = hw[sel].std()
    by_iden.setdefault(v, []).append((hw[sel], t[sel]))
    print(f"{v:>6} {settle:>6} {len(sel):>5} {sd:>7.0f} {hw[sel].mean():>6.0f} {period(t[sel],hw[sel]):>9.0f}")

print(f"\n=== aggregated per i_den (all segments of each value pooled, settling removed) ===")
print(f"{'i_den':>6} {'n_seg':>6} {'n':>6} {'sd_ns':>7} {'period_s':>9}   2pi√i_den")
for v in sorted(by_iden):
    parts = by_iden[v]
    allhw = np.concatenate([p[0] for p in parts])
    # pooled sd uses per-segment de-meaning so cross-segment offset steps don't inflate it
    dm = np.concatenate([p[0] - p[0].mean() for p in parts])
    allt = np.concatenate([p[1] for p in parts])
    sd = dm.std()
    per = period(parts[0][1], parts[0][0]) if len(parts) else float("nan")
    print(f"{v:>6} {len(parts):>6} {len(allhw):>6} {sd:>7.0f} {per:>9.0f}   {2*math.pi*math.sqrt(v):.0f}")
print("\nread: sd ~flat across i_den => the 6x was the adaptive switch / reception (loosening doesn't cut amplitude).")
print("      sd drops steeply with i_den => i_den itself matters (loosening is the lever).")
