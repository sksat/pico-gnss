#!/usr/bin/env python3
"""loopback 接続の説明図。配線 (GP3 の分岐を GP4 へ戻す) と、それで何が測れるか
(GPS-R と出力の両エッジを PIO が時刻捕捉 -> 差 = loopback 位相) を 1 枚で。
カウンタの同一性は主張しない (実際は 2 つのカウンタ + 実効 K。後の節で扱う)。"""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "precision-figs")

fig, ax = plt.subplots(figsize=(9.2, 4.4))
ax.set_xlim(0, 10); ax.set_ylim(0, 5.2); ax.axis("off")

# GPS-R
ax.add_patch(FancyBboxPatch((0.3, 2.9), 1.7, 1.2, boxstyle="round,pad=0.06", fc="#fff6e0", ec="#b8860b", lw=1.4))
ax.text(1.15, 3.5, "GPS-R", ha="center", va="center", fontsize=12)

# Pico
ax.add_patch(FancyBboxPatch((4.0, 0.9), 4.3, 3.5, boxstyle="round,pad=0.06", fc="#eef3fa", ec="#446", lw=1.4))
ax.text(4.35, 4.15, "Pico", ha="left", va="center", fontsize=12)

# ピン (Pico の縁の小さな四角)。GP3 はラベルをピンの上に置く (出力生成の箱と重ねない)
for (px, py, name) in ((4.0, 3.5, "GP2"), (4.0, 1.6, "GP4")):
    ax.add_patch(Rectangle((px - 0.12, py - 0.12), 0.24, 0.24, fc="#fff", ec="#446", lw=1.2, zorder=4))
    ax.text(px + 0.22, py, name, ha="left", va="center", fontsize=10, color="#446")
ax.add_patch(Rectangle((8.3 - 0.12, 3.5 - 0.12), 0.24, 0.24, fc="#fff", ec="#446", lw=1.2, zorder=4))
ax.text(8.3, 3.26, "GP3", ha="center", va="top", fontsize=10, color="#446")

# PIO ブロック: 独立した 3 つの小さなプログラム (SM) を走らせる
ax.add_patch(FancyBboxPatch((5.0, 1.25), 3.0, 2.75, boxstyle="round,pad=0.05", fc="#f0f7f0", ec="#2a7d2a", lw=1.3))
ax.text(6.5, 3.78, "PIO", ha="center", va="center", fontsize=9.5, color="#2a7d2a")
for (bx0, by0, bw, lab) in ((5.2, 2.75, 1.45, "エッジ捕捉\n(GPS-R 用)"), (5.2, 1.45, 1.45, "エッジ捕捉\n(loopback 用)"), (6.83, 2.75, 1.12, "GPSDO PPS\n出力生成")):
    ax.add_patch(FancyBboxPatch((bx0, by0), bw, 0.85, boxstyle="round,pad=0.04", fc="#e4f2e4", ec="#2a7d2a", lw=1.1))
    ax.text(bx0 + bw / 2, by0 + 0.42, lab, ha="center", va="center", fontsize=9)

# 出力生成 -> GP3
ax.annotate("", xy=(8.16, 3.45), xytext=(7.9, 3.3), arrowprops=dict(arrowstyle="->", color="#2a7d2a", lw=1.3))

# GPS-R 1PPS -> GP2
ax.annotate("", xy=(3.86, 3.5), xytext=(2.05, 3.5), arrowprops=dict(arrowstyle="->", color="#b8860b", lw=1.8))
ax.text(2.95, 3.62, "1PPS", ha="center", fontsize=10.5, color="#b8860b")
# GP2 -> 捕捉 SM / GP4 -> 捕捉 SM
ax.annotate("", xy=(5.2, 3.25), xytext=(4.14, 3.45), arrowprops=dict(arrowstyle="->", color="#446", lw=1.3))
ax.annotate("", xy=(5.2, 1.85), xytext=(4.14, 1.62), arrowprops=dict(arrowstyle="->", color="#446", lw=1.3))

# GP3 -> 外部出力 (途中に分岐点)
ax.annotate("", xy=(9.75, 3.5), xytext=(8.44, 3.5), arrowprops=dict(arrowstyle="->", color="#2a5db0", lw=1.8))
ax.text(9.75, 3.72, "GPSDO PPS 出力", ha="right", fontsize=10.5, color="#2a5db0")
bx = 9.1
ax.scatter([bx], [3.5], s=42, color="#2a5db0", zorder=5)  # 分岐点

# loopback 配線: 分岐点 -> 下 -> 左 -> 上 -> GP4
ax.plot([bx, bx, 3.2, 3.2], [3.5, 0.45, 0.45, 1.6], color="#2a5db0", lw=1.8)
ax.annotate("", xy=(3.86, 1.6), xytext=(3.2, 1.6), arrowprops=dict(arrowstyle="->", color="#2a5db0", lw=1.8))
ax.text(6.15, 0.28, "loopback: 出力を GP4 へ戻す", ha="center", fontsize=10.5, color="#2a5db0")

# 何が測れるか
ax.text(5.0, 4.7, "2 つのエッジ時刻の差 = loopback 位相 (出力が GPS-R からどれだけずれているか)", ha="center", fontsize=11.5)

fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig-loopback.png"), dpi=110); plt.close(fig)
print("wrote fig-loopback.png")
