#!/usr/bin/env python3
"""dither の模式図 (パルス列版)。抽象軸をやめ、1PPS のパルス列を描いて、パルス間の長さ = 周期を
毎回どちらかから選ぶことを見せる。9 回 1000003 µs + 1 回 1000002 µs -> 平均 1000002.9 µs。"""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "precision-figs")

NP = 11  # パルス 11 本 = 周期 10 個
BLUE, ORANGE = "#4477cc", "#d97b00"

fig, ax = plt.subplots(figsize=(9.8, 4.6))
ax.axhline(0, color="#555", lw=1.0, zorder=1)
for i in range(NP):
    ax.plot([i, i], [0, 1.0], color="#333", lw=2.6, zorder=3)  # 1PPS のパルス (立ち上がり)

for g in range(NP - 1):
    last = (g == NP - 2)
    col = ORANGE if last else BLUE
    lab = "1000002" if last else "1000003"
    ax.annotate("", xy=(g + 0.96, 0.52), xytext=(g + 0.04, 0.52),
                arrowprops=dict(arrowstyle="<->", color=col, lw=1.4), zorder=2)
    ax.text(g + 0.5, 0.34, lab, ha="center", fontsize=9.5, color=col)

ax.text(0.0, 1.45, "出したい周期は、GPS-R の 1 秒をタイマーで数えた 1000002.9 µs。でも周期は 1 µs 単位でしか作れない。", fontsize=12.5, color="#333")
ax.text(0.0, 1.18, "そこで端数 0.9 µs のぶん、10 回のうち 9 回を長い方にする (矢印 = パルスからパルスまでの長さ = 周期、数字は µs):", fontsize=11, color="#555")

# 各パルスの、理想 (毎回ちょうど 1000002.9 µs) と比べた時刻のずれ。+0.1 ずつ溜まり、最後に 0 へ戻る。
offs = ["0"] + [f"+0.{k}" for k in range(1, 10)] + ["0"]
for i, o in enumerate(offs):
    last = (i == NP - 1)
    ax.text(i, -0.22, o, ha="center", fontsize=9.5,
            color="#cc3333" if last else "#777",
            fontweight="bold" if last else "normal")
ax.text(4.5, -0.50, "↑ 理想 (毎回ちょうど 1000002.9 µs で打つ) パルスと比べた時刻のずれ (µs)。", ha="center", fontsize=10.5, color="#555")
ax.text(4.5, -0.74, "ずれは 1 µs 未満のまま、10 周期でちょうど 0 に戻る = 平均の周期は出したい値に揃う", ha="center", fontsize=10.5, color="#555")
ax.text(4.5, -1.10, "周期 10 個の平均 = (9 × 1000003 + 1 × 1000002) ÷ 10 = 1000002.9 µs = 出したい周期", ha="center", fontsize=12,
        bbox=dict(boxstyle="round,pad=0.45", fc="#f7f7f7", ec="#999"))

ax.set_xlim(-0.4, NP - 0.6)
ax.set_ylim(-1.35, 1.7)
ax.axis("off")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig-dither-concept.png"), dpi=110); plt.close(fig)
print("wrote fig-dither-concept.png (pulse-train version)")
