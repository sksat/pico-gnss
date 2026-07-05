#!/usr/bin/env python3
"""fig10b: リミッタあり/なし、それぞれの最悪加熱ステップの過渡を時系列で並べる。
なし = s5/stage5-heat.log の c268 イベント (ΔT 1.22℃、peak +8640/−4656 ns)。
あり = s5/clamped-heat.log の c442 イベント (ΔT 0.78℃、peak 592 ns)。
左は同一軸で桁の違いを、右はあり側だけを拡大して形を見せる。
usage: uv run --with matplotlib python3 logs/precision-rework/fig10b_clamp_transient.py
"""
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try:
            font_manager.fontManager.addfont(fp)
        except Exception:
            pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(os.path.dirname(HERE)),
                   "docs", "report", "precision-ladder", "precision-figs")
PAT = re.compile(r"count=(\d+) .*hwphase_ns=(-?\d+)")


def series(path, c_base, c_lo, c_hi):
    out = []
    for ln in open(os.path.join(HERE, path), errors="replace"):
        m = PAT.search(ln)
        if m:
            c = int(m.group(1))
            if c_lo <= c <= c_hi:
                out.append((c - c_base, int(m.group(2))))
    return out


unc = series("s5/stage5-heat.log", 268, 268 - 15, 268 + 75)
# あり側は加熱を連打した中の 1 発 (c442)。直前ステップの残りが入らないよう窓は −5s から
cl = series("s5/clamped-heat.log", 442, 442 - 5, 442 + 75)

fig, (ax, axz) = plt.subplots(1, 2, figsize=(11.4, 4.6),
                              gridspec_kw={"width_ratios": [1.25, 1], "wspace": 0.22})
ax.axhline(0, color="gray", lw=0.6)
ax.plot([t for t, _ in unc], [h for _, h in unc], lw=1.6, color="#c0392b",
        label="リミッタなし (ΔT≈1.2℃)")
ax.plot([t for t, _ in cl], [h for _, h in cl], lw=1.6, color="#2a7d3a",
        label="リミッタあり ±100 ppb (ΔT≈0.8℃)")
ax.set_xlabel("加熱開始からの時間 [s]")
ax.set_ylabel("loopback 位相 [ns]")
ax.set_title("それぞれの最悪ステップを同一軸で", fontsize=11)
ax.legend(loc="upper right", fontsize=9)
ax.annotate("+8640 ns", (13, 8640), xytext=(-12, 8200), fontsize=9, color="#c0392b")
ax.annotate("−4656 ns", (26, -4656), xytext=(33, -4900), fontsize=9, color="#c0392b")

# 右: あり側だけを拡大して形を見せる
axz.axhline(0, color="gray", lw=0.5)
axz.axhspan(-100, 100, color="#888888", alpha=0.15, lw=0)
axz.plot([t for t, _ in cl], [h for _, h in cl], lw=1.4, color="#2a7d3a")
axz.set_ylim(-700, 700)
axz.set_xlim(-5, 75)
axz.set_xlabel("加熱開始からの時間 [s]")
axz.set_title("リミッタあり側の拡大 (灰帯 = ±100 ns)", fontsize=11)
axz.annotate("peak 592 ns", (6, 592), xytext=(14, 560), fontsize=9, color="#2a7d3a")
fig.suptitle("最悪の加熱ステップの過渡: リミッタなし対あり", y=1.02)

fig.savefig(os.path.join(OUT, "fig10b-clamp-transient.png"), dpi=130,
            bbox_inches="tight")
print("saved fig10b-clamp-transient.png",
      f"unc n={len(unc)} peak={max(abs(h) for _, h in unc)}",
      f"cl n={len(cl)} peak={max(abs(h) for _, h in cl)}")
