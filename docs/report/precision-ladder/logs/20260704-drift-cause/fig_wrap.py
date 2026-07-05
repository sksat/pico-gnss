#!/usr/bin/env python3
"""fig-wrap-cost: 残った実ドリフトの機構図。
上段: カウンタの一周 (68.7s 毎) の 1 回転コストが、low 待ちループ (+1 cycle) と
      high 待ちループ (0) で違うこと。
下段: 2 つのピンの波形 (duty が逆) と、一周がどちらの区間に落ちるかで払いが決まること。
usage: uv run --with matplotlib python3 logs/20260704-drift-cause/fig_wrap.py
"""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "precision-figs")

fig = plt.figure(figsize=(10.6, 10.4))
gs = fig.add_gridspec(2, 1, height_ratios=[7.3, 4.6], hspace=0.12)

# ================= 上段: 待ちループのフローチャート =================
ax = fig.add_subplot(gs[0]); ax.set_xlim(0, 12); ax.set_ylim(0.9, 8.2); ax.axis("off")
from matplotlib.patches import Polygon
ax.text(0.1, 7.95, "カウンタは 32 bit なので、16 ns × 2 の 32 乗 ≈ 68.7 秒ごとに 0 を跨いで一周する",
        fontsize=11, ha="left", va="center")

def fbox(x, y, w, h, label, sub=None, ec="#446", fc="#eef2f8", fs=9.2):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.04", fc=fc, ec=ec, lw=1.2, zorder=4))
    if sub:
        ax.text(x + w / 2, y + h * 0.62, label, ha="center", va="center", fontsize=fs, zorder=5)
        ax.text(x + w / 2, y + h * 0.22, sub, ha="center", va="center", fontsize=7.2, color="#666", zorder=5)
    else:
        ax.text(x + w / 2, y + h / 2, label, ha="center", va="center", fontsize=fs, zorder=5)

def diamond(cx, cy, hw, hh, label, ec="#446", fs=8.8):
    ax.add_patch(Polygon([(cx - hw, cy), (cx, cy + hh), (cx + hw, cy), (cx, cy - hh)],
                         closed=True, fc="#fff", ec=ec, lw=1.2, zorder=4))
    ax.text(cx, cy, label, ha="center", va="center", fontsize=fs, zorder=5)

def flow(pts, color="#446", lw=1.4):
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=lw, zorder=3)
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="->", color=color, lw=lw,
                                 zorder=3, shrinkA=0, shrinkB=2, mutation_scale=11))

# --- 左: 立ち上がり待ち (low) ---
ax.text(0.4, 7.45, "ピンが low の間 (立ち上がり待ち)", fontsize=10, weight="bold", color="#8a2222")
fbox(1.8, 6.5, 2.4, 0.6, "ピンを見る", "1 cycle")
flow([(3.0, 6.5), (3.0, 6.14)])
diamond(3.0, 5.66, 1.3, 0.46, "立ち上がった?")
flow([(4.3, 5.66), (5.2, 5.66)])
ax.text(4.75, 5.86, "立ち上がった", fontsize=7.6, color="#446", ha="center")
ax.text(5.3, 5.66, "捕捉", fontsize=8.8, color="#446", ha="left", va="center")
flow([(3.0, 5.2), (3.0, 4.88)])
ax.text(3.15, 5.04, "low のまま", fontsize=7.6, color="#446", ha="left")
fbox(1.8, 4.26, 2.4, 0.6, "カウンタを 1 減らす", "1 cycle")
ax.text(4.35, 4.56, "跳び先はループの先頭。\n0 のときだけ跳ばず下の行へ", fontsize=7.2, color="#666", ha="left", va="center")
flow([(3.0, 4.26), (3.0, 3.92)])
diamond(3.0, 3.44, 1.3, 0.46, "0 を跨いだ?")
# 0 以外 (ほぼ毎回): 先頭へ
flow([(1.7, 3.44), (0.7, 3.44), (0.7, 6.8), (1.75, 6.8)])
ax.text(1.3, 3.62, "0 以外 (ほぼ毎回)", fontsize=7.4, color="#446", ha="center")
ax.text(0.78, 5.7, "2 cycle で\n1 目盛り", fontsize=7.4, color="#446", ha="left")
# 0 を跨いだ: jmp top を経由して先頭へ
flow([(3.0, 2.98), (3.0, 2.66)], color="#cc3333")
ax.text(3.15, 2.82, "0 を跨いだ (68.7 秒に 1 回)", fontsize=7.6, color="#cc3333", ha="left")
fbox(1.9, 2.04, 2.2, 0.6, "jmp low", "+1 cycle", ec="#cc3333", fc="#fdeaea")
flow([(1.9, 2.34), (0.26, 2.34), (0.26, 7.0), (1.78, 7.0)], color="#cc3333")
ax.text(4.65, 2.1, "この命令が無いと、すぐ下の\n捕捉のコードへ流れ込み、エッジが\n無いのに偽の捕捉をしてしまう",
        fontsize=7.4, color="#996666", ha="left")
ax.text(3.0, 1.3, "通常は 2 cycle、0 を跨ぐ回だけ 3 cycle\n= カウンタの時計が 8 ns 止まる", fontsize=9.4,
        color="#cc3333", ha="center", weight="bold")

# --- 右: 立ち下がり待ち (high) ---
ax.text(7.1, 7.45, "ピンが high の間 (立ち下がり待ち)", fontsize=10, weight="bold", color="#22661f")
fbox(7.9, 6.5, 2.4, 0.6, "カウンタを 1 減らす", "1 cycle")
ax.text(10.45, 6.8, "跳び先がすぐ次の行\nなので、0 で跳ばなく\nても行き先は同じ", fontsize=7.2, color="#22661f", ha="left", va="center")
flow([(9.1, 6.5), (9.1, 6.14)])
fbox(7.9, 5.52, 2.4, 0.6, "ピンを見る", "1 cycle")
flow([(9.1, 5.52), (9.1, 5.16)])
diamond(9.1, 4.68, 1.2, 0.46, "まだ high?")
flow([(7.9, 4.68), (7.05, 4.68), (7.05, 6.8), (7.85, 6.8)])
ax.text(7.42, 4.88, "まだ high", fontsize=7.4, color="#446", ha="center")
ax.text(7.12, 5.7, "2 cycle で\n1 目盛り", fontsize=7.4, color="#446", ha="left")
flow([(9.1, 4.22), (9.1, 3.9)])
ax.text(9.25, 4.06, "立ち下がった", fontsize=7.6, color="#446", ha="left")
ax.text(9.1, 3.68, "立ち上がり待ちへ", fontsize=8.6, color="#446", ha="center")
ax.text(9.1, 1.3, "0 を跨いでも流れが変わらないので\n2 cycle のまま", fontsize=9.4,
        color="#22661f", ha="center", weight="bold")

# ================= 下段: 2 つのピンの duty と、一周が落ちる場所 =================
ax2 = fig.add_subplot(gs[1]); ax2.set_xlim(0, 12); ax2.set_ylim(0, 4.6); ax2.axis("off")
ax2.text(0.1, 4.35, "一周がどちらの待ちループ中に来るかは、そのときピンが high か low かで決まる (波形は実測)",
         fontsize=11, ha="left", va="center")

def pps(ax, y, high_frac, color):
    """3 秒ぶんの 1PPS 波形。high_frac = high の割合。"""
    x0, w = 0.7, 3.0  # 1 秒 = 3.0
    for s in range(3):
        hx = x0 + s * w
        ax.plot([hx, hx], [y, y + 0.55], color=color, lw=1.6)
        ax.plot([hx, hx + w * high_frac], [y + 0.55, y + 0.55], color=color, lw=1.6)
        ax.plot([hx + w * high_frac, hx + w * high_frac], [y + 0.55, y], color=color, lw=1.6)
        ax.plot([hx + w * high_frac, hx + w], [y, y], color=color, lw=1.6)

# GPS-R PPS: high 900ms / low 100ms
pps(ax2, 2.9, 0.9, "#b8860b")
ax2.text(0.55, 3.2, "GPS-R の\nPPS ピン", fontsize=9, ha="right", va="center", color="#b8860b")
ax2.text(9.95, 3.35, "high が 900 ms / low は 100 ms\n→ 一周の 9 割は high 中 = 遅れない",
         fontsize=8.8, va="center", color="#22661f")
# 出力: high 100ms / low 900ms
pps(ax2, 1.15, 0.1, "#2a5db0")
ax2.text(0.55, 1.45, "GPSDO\n出力", fontsize=9, ha="right", va="center", color="#2a5db0")
ax2.text(9.95, 1.6, "high は 100 ms / low が 900 ms\n→ 一周の 9 割は low 中 = 8 ns 遅れる",
         fontsize=8.8, va="center", color="#cc3333")
# 一周の到来 (矢印)。green=high 中 (払わない)、red=low 中 (8ns 払う)
for x, y, pays in ((2.2, 2.9, False), (5.9, 2.9, False), (9.55, 2.9, True),   # GPS: 9割 high
                   (2.2, 1.15, True), (5.9, 1.15, True), (3.85, 1.15, False)):  # 出力: 9割 low
    c = "#cc3333" if pays else "#22661f"
    ax2.add_patch(FancyArrowPatch((x, y + 1.18), (x, y + 0.70), arrowstyle="->", color=c,
                                  lw=1.6, mutation_scale=13))
ax2.text(0.7, 4.02, "↓ = カウンタが 0 を跨いだ瞬間の例 (68.7 秒ごと、来る位相はまちまち):", fontsize=8.4,
         color="#555", ha="left")
ax2.text(6.75, 4.02, "high 中に来たら遅れない", fontsize=8.4, color="#22661f", ha="left")
ax2.text(9.35, 4.02, "low 中に来たら 8 ns 遅れる", fontsize=8.4, color="#cc3333", ha="left")
# 時間軸 (1 秒 = 3.0 単位、x0 = 0.7)
ax2.plot([0.7, 9.7], [0.95, 0.95], color="#888", lw=1.0)
for i in range(4):
    x = 0.7 + i * 3.0
    ax2.plot([x, x], [0.95, 1.03], color="#888", lw=1.0)
    ax2.text(x, 0.72, f"{i}", ha="center", fontsize=8, color="#666")
ax2.text(10.0, 0.72, "時間 [秒]", ha="left", fontsize=8, color="#666")
ax2.text(6.0, 0.22, "差し引き: 出力側のカウンタのほうが 0.87 回/分 × 0.8 × 8 ns ≈ 5.6 ns/min 余計に遅れていく",
         fontsize=10, ha="center", color="#cc3333", weight="bold")

fig.savefig(os.path.join(OUT, "fig-wrap-cost.png"), dpi=110, bbox_inches="tight")
print("wrote fig-wrap-cost.png")
