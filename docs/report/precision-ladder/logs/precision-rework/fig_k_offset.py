#!/usr/bin/env python3
"""カウンタのズレの図。上段: 2 つの自走カウンタは数え始めの瞬間が違うので、読みが常にズレる。
下段: 同じエッジを両方に見せると、読みの差 C0-C2 = K が測れる (校正時は loopback 捕捉も GP2 を見る)。
読みの値は説明用だが、差 5500 tick は実測の K の桁 (≈88µs) に合わせてある。"""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "precision-figs")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.4, 6.2))

# ---------- 上段: ズレが生まれる理由 ----------
ax1.set_xlim(0, 11.5); ax1.set_ylim(-0.7, 2.3); ax1.axis("off")
ax1.set_title("ズレが生まれる理由: 2 つのカウンタは数え始めの瞬間が違う", fontsize=12, loc="left")
rows = [(1.5, 1.2, "GPS-R PPS 用カウンタ", "起動時に数え始め"),
        (0.0, 2.9, "loopback 用カウンタ", "少し遅れて数え始め")]
READ_X = 8.2
for y, x0, name, startlab in rows:
    ax1.annotate("", xy=(10.2, y), xytext=(x0, y), arrowprops=dict(arrowstyle="->", color="#446", lw=1.6))
    for t in [x0 + 0.35 * k for k in range(1, int((10.0 - x0) / 0.35))]:
        ax1.plot([t, t], [y - 0.06, y + 0.06], color="#446", lw=0.7, alpha=0.5)
    ax1.scatter([x0], [y], s=48, color="#446", zorder=4)
    ax1.text(x0 - 0.15, y + 0.22, startlab, ha="left", fontsize=9.5, color="#446")
    ax1.text(0.0, y - 0.38, name, ha="left", fontsize=10.5)
# 数え始めの間隔 = K の由来
ax1.plot([1.2, 1.2], [1.5, 0.78], color="#888", ls=":", lw=1.0)
ax1.plot([2.9, 2.9], [0.0, 0.78], color="#888", ls=":", lw=1.0)
ax1.annotate("", xy=(2.9, 0.78), xytext=(1.2, 0.78), arrowprops=dict(arrowstyle="<->", color="#888", lw=1.2))
ax1.text(3.15, 0.68, "この間隔ぶんズレる。大きさは 2 つを有効化するコードの間隔で決まり、\n実測 ≈ 5500 tick (88 µs ぶん) = 目標 100 ns の 1000 倍", fontsize=9, color="#555", va="center")
ax1.plot([READ_X, READ_X], [-0.45, 2.05], color="#cc3333", ls="--", lw=1.4)
ax1.text(READ_X, 2.12, "同じ瞬間に読むと", ha="center", fontsize=10, color="#cc3333")
ax1.text(READ_X + 0.18, 1.5 + 0.2, "読み C0 = 71500", fontsize=10, color="#cc3333")
ax1.text(READ_X + 0.18, 0.0 + 0.2, "読み C2 = 66000", fontsize=10, color="#cc3333")
ax1.text(10.7, 0.75, "差 = 5500\nいつ読んでも一定", ha="center", va="center", fontsize=10,
         bbox=dict(boxstyle="round,pad=0.35", fc="#fdf2f2", ec="#cc3333"))

# ---------- 下段: 測り方 ----------
ax2.set_xlim(0, 11.5); ax2.set_ylim(-0.9, 2.6); ax2.axis("off")
ax2.set_title("測り方: 同じエッジを 2 つのカウンタに見せる", fontsize=12, loc="left")
# GPS-R 1PPS のパルス波形 (立ち上がって、下りる)
ax2.plot([0.3, 1.5, 1.5, 2.7, 2.7, 3.9], [0.5, 0.5, 1.55, 1.55, 0.5, 0.5], color="#b8860b", lw=2.0)
ax2.text(0.3, 1.82, "GPS-R 1PPS のパルス", fontsize=10.5, color="#b8860b")
# 読む瞬間 = 立ち上がり (上段の「同じ瞬間」と同じモチーフ)
ax2.plot([1.5, 1.5], [0.3, 2.15], color="#cc3333", ls="--", lw=1.4)
ax2.text(1.5, 2.22, "立ち上がりの瞬間に両方で読む", ha="left", fontsize=10, color="#cc3333")
# 2 つのカウンタ箱
for (by, name, read) in ((1.45, "GPS-R PPS 用カウンタ", "読み C0 = 71500"), (0.0, "loopback 用カウンタ", "読み C2 = 66000")):
    ax2.add_patch(FancyBboxPatch((5.2, by), 2.6, 0.95, boxstyle="round,pad=0.05", fc="#e4f2e4", ec="#2a7d2a", lw=1.2))
    ax2.text(6.5, by + 0.66, name, ha="center", fontsize=10)
    ax2.text(6.5, by + 0.26, read, ha="center", fontsize=10, color="#cc3333")
    ax2.annotate("", xy=(5.15, by + 0.48), xytext=(1.6, 1.0), arrowprops=dict(arrowstyle="->", color="#888", lw=1.5))
ax2.text(9.7, 1.2, "読みの差\nC0 − C2 = 5500", ha="center", va="center", fontsize=11,
         bbox=dict(boxstyle="round,pad=0.4", fc="#fdf2f2", ec="#cc3333"))
ax2.text(0.4, -0.65, "同じ瞬間の読みどうしなので、差はカウンタのズレそのもの。\n(校正のあいだだけ、loopback 用の捕捉も GPS-R のピンを見るように切り替える)",
         fontsize=10, color="#555")

fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig-k-offset.png"), dpi=110); plt.close(fig)
print("wrote fig-k-offset.png")
