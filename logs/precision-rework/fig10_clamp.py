#!/usr/bin/env python3
"""fig10: リミッタ (±100 ppb) の加熱 A/B。過渡 peak/℃ が中央値で約 2 倍、worst で約 9 倍縮む。
per-event 値は analyze.py の出力 (速い加熱ステップのみ):
  リミッタなし = s5/stage5-heat.log の 3 イベント {474, 1241, 7079}
  リミッタあり = s5/clamped-heat.log の 4 イベント {218, 447, 754, 764}
log 軸で比率を直感的に。
usage: uv run --with matplotlib python3 logs/precision-rework/fig10_clamp.py
"""
import os
import statistics as st

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
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                   "docs", "report", "precision-ladder", "precision-figs")

unc = [474.0, 1241.0, 7079.0]        # リミッタなし (出口制限なし)
cl = [218.0, 447.0, 754.0, 764.0]    # リミッタあり (±100 ppb)
unc_med, cl_med = st.median(unc), st.median(cl)

fig, ax = plt.subplots(figsize=(7.6, 4.4))
ax.set_yscale("log")
ax.bar([0], [unc_med], width=0.5, color="#c33", alpha=0.85)
ax.bar([1], [cl_med], width=0.5, color="#4a7", alpha=0.85)
ax.scatter([0] * len(unc), unc, color="#822", zorder=3, s=28)
ax.annotate("worst 7079", (0, max(unc)), xytext=(8, 0), textcoords="offset points",
            va="center", fontsize=9, color="#822")
ax.annotate(f"中央値 {unc_med:.0f}", (0, unc_med), xytext=(8, -4),
            textcoords="offset points", va="center", fontsize=9, color="#822")
ax.scatter([1] * len(cl), cl, color="#272", zorder=3, s=28)
ax.annotate("worst 764", (1, max(cl)), xytext=(8, 4), textcoords="offset points",
            va="center", fontsize=9, color="#272")
ax.annotate(f"中央値 {cl_med:.0f}", (1, cl_med), xytext=(8, -8),
            textcoords="offset points", va="center", fontsize=9, color="#272")
ax.annotate("", xy=(0.5, cl_med), xytext=(0.5, unc_med),
            arrowprops=dict(arrowstyle="<->", color="#444", lw=1.4))
ax.text(0.56, (unc_med * cl_med) ** 0.5, "中央値で約 2 倍\n(worst では約 9 倍)",
        fontsize=11, va="center",
        bbox=dict(boxstyle="round", fc="#fffbe6", ec="#caa"))
ax.set_xticks([0, 1])
ax.set_xticklabels(["リミッタなし", "リミッタあり"])
ax.set_ylabel("加熱時の過渡 peak / ℃  [ns/℃]  (対数軸)")
ax.set_ylim(150, 10000)
ax.set_title("±100 ppb リミッタによる加熱過渡の変化")
ax.grid(ls=":", alpha=0.4, axis="y")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig10-clamp-ab.png"), dpi=130)
print("wrote fig10-clamp-ab.png med %.0f->%.0f (%.1fx) worst %.0f->%.0f (%.1fx)"
      % (unc_med, cl_med, unc_med / cl_med, max(unc), max(cl), max(unc) / max(cl)))
