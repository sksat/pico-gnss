# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""定期 K 再校正の効果を可視化する。

offset_wander のログ (elapsed_s, offset2_ns, hwphase_ns, raw_lag, ppb) を 2 本読み、
gap = offset2 - hwphase を時間軸で並べる。gap は firmware に見えない実効 K 誤差で、
再校正が無いと温度で creep し、再校正が入ると pin される (=ドリフト停止)。
gap の絶対値 ~50ns 分は ×1/×10 スコーププローブのスキュー等の計測セットアップ固有オフセットで、
firmware の値でも実出力オフセットでもない (実際の出力 vs GPS は hwphase=~0 中心)。ここで見るのは傾き。

  uv run docs/report/plot_recal.py <no-recal offset.log> <recal offset.log> [recal_times_csv] [out.png]
"""
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

NORECAL = sys.argv[1] if len(sys.argv) > 1 else "/tmp/offset-gate5.log"
RECAL = sys.argv[2] if len(sys.argv) > 2 else "/tmp/offset-slew.log"
RECAL_TS = [float(x) for x in sys.argv[3].split(",")] if len(sys.argv) > 3 and sys.argv[3] else []
OUT = sys.argv[4] if len(sys.argv) > 4 else "docs/report/offset-recal.png"


def load(path):
    t, gap = [], []
    for line in open(path):
        if line.startswith("#") or not line.strip():
            continue
        p = line.split("\t")
        try:
            el = float(p[0])
            off = None if p[1] == "None" else int(p[1])
            hw = None if p[2] == "None" else int(p[2])
        except (ValueError, IndexError):
            continue
        if off is None or hw is None:
            continue
        t.append(el / 60.0)  # minutes
        gap.append(off - hw)
    return np.array(t), np.array(gap)


def fit_slope(t, g):
    if len(t) < 2:
        return 0.0, np.mean(g) if len(g) else 0.0
    a, b = np.polyfit(t, g, 1)
    return a, b  # ns/min, intercept


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)

for ax, path, title, ts in [
    (ax1, NORECAL, "no periodic re-cal (#5 only)", []),
    (ax2, RECAL, "periodic K re-cal + slew", RECAL_TS),
]:
    t, g = load(path)
    if len(t) == 0:
        ax.set_title(title + " (no data)")
        continue
    a, b = fit_slope(t, g)
    ax.plot(t, g, ".", color="tab:purple", ms=4, alpha=0.6)
    xs = np.linspace(t.min(), t.max(), 50)
    ax.plot(xs, a * xs + b, "k--", lw=1.5, label=f"trend {a:+.1f} ns/min")
    for i, tt in enumerate(ts):
        ax.axvline(tt / 60.0, color="tab:green", ls=":", alpha=0.7,
                   label="K re-cal" if i == 0 else None)
    ax.set_xlabel("elapsed time [min]")
    ax.set_title(f"{title}\nsd={np.std(g):.0f}ns, trend {a:+.1f} ns/min")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(alpha=0.3)

ax1.set_ylabel("gap = offset② − hwphase [ns]\n(effective-K error, invisible to firmware)")
fig.suptitle("effective-K drift pinned by periodic re-cal "
             "(~50ns offset = probe skew; the slope is what matters)",
             fontsize=11)
fig.tight_layout()
fig.savefig(OUT, dpi=110)
print(f"wrote {OUT}")
