#!/usr/bin/env python3
"""カウンタのズレの原因を切り分ける観測方法図。
上段: 3 本のカウンタ (GPS-R 用 c0=GP2、観測用 c3=GP2 常時、loopback 用 c2=GP4)。
  「校正での切り替え」= 定期校正で c2 を一瞬 GP2 へ向けてズレを測り直す動作、を右の吹き出しで説明。
下段: 表ではなく理由の連鎖。c3 は同じ GP2・切替なしなので c0-c3 には波形も切替も効かず、
  残るのはカウンタごとの癖だけ。だから c0-c3 がその単独テストになる。"""
import os
import re
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "precision-figs")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10.2, 7.4),
                               gridspec_kw={"height_ratios": [1.05, 0.95]})

# ================= 上段: 3 本のカウンタと「校正での切り替え」 =================
ax1.set_xlim(0, 11.4); ax1.set_ylim(0, 3.75); ax1.axis("off")
ax1.text(0.1, 3.58, "カウンタのズレ = 2 本のカウンタの読みの差。動かしているうちにこれがスリップする。原因を 3 本目 c3 で切り分ける",
         ha="left", va="center", fontsize=11)

# 入力源 → ピン
ax1.text(0.1, 2.62, "GPS-R の 1PPS", ha="left", va="center", fontsize=9.3, color="#b8860b")
ax1.annotate("", xy=(1.97, 2.62), xytext=(1.5, 2.62), arrowprops=dict(arrowstyle="->", color="#b8860b", lw=1.6))
ax1.text(0.1, 0.62, "出力の loopback", ha="left", va="center", fontsize=9.3, color="#2a5db0")
ax1.annotate("", xy=(1.97, 0.62), xytext=(1.5, 0.62), arrowprops=dict(arrowstyle="->", color="#2a5db0", lw=1.6))
for (py, name) in ((2.62, "GP2"), (0.62, "GP4")):
    ax1.add_patch(Rectangle((2.0, py - 0.13), 0.26, 0.26, fc="#fff", ec="#446", lw=1.2, zorder=5))
    ax1.text(2.13, py - 0.31, name, ha="center", va="top", fontsize=9.3, color="#446")

# 3 本のカウンタ箱
boxes = [
    (2.72, "c0   GPS-R 用", "GP2 を見る", "#2a7d2a", "#e4f2e4"),
    (1.78, "c3   観測用 (追加)", "常に GP2。切り替えない", "#8a5a00", "#faf0dc"),
    (0.28, "c2   loopback 用", "ふだん GP4。校正のとき一瞬 GP2 へ", "#2a7d2a", "#e4f2e4"),
]
BX, BW = 3.25, 3.55
for (by, name, sub, ec, fcc) in boxes:
    ax1.add_patch(FancyBboxPatch((BX, by), BW, 0.74, boxstyle="round,pad=0.04", fc=fcc, ec=ec, lw=1.3))
    ax1.text(BX + 0.2, by + 0.5, name, ha="left", va="center", fontsize=9.6, weight="bold")
    ax1.text(BX + 0.2, by + 0.2, sub, ha="left", va="center", fontsize=8.4, color="#555")

# GP2 -> c0, c3 / GP4 -> c2
jx = 2.7
ax1.scatter([jx], [2.62], s=28, color="#446", zorder=6)
ax1.annotate("", xy=(BX - 0.02, 3.05), xytext=(jx, 2.62), arrowprops=dict(arrowstyle="->", color="#446", lw=1.3))
ax1.annotate("", xy=(BX - 0.02, 2.12), xytext=(jx, 2.62), arrowprops=dict(arrowstyle="->", color="#8a5a00", lw=1.4))
ax1.text(2.34, 2.02, "同じ\nGP2", ha="center", va="center", fontsize=7.8, color="#8a5a00")
ax1.annotate("", xy=(BX - 0.02, 0.62), xytext=(2.26, 0.62), arrowprops=dict(arrowstyle="->", color="#446", lw=1.3))
# c2 が校正で一瞬 GP2 を見る (破線)
ax1.add_patch(FancyArrowPatch((BX + 0.3, 0.28), (2.13, 2.49), connectionstyle="arc3,rad=0.35",
                              arrowstyle="->", color="#cc3333", lw=1.2, ls=(0, (4, 2)), zorder=3))
ax1.text(2.5, 1.15, "校正のとき\n一瞬こちら", ha="center", va="center", fontsize=7.6, color="#cc3333")

# 右: 「校正での切り替え」の吹き出し (定義に絞る)
ax1.add_patch(FancyBboxPatch((7.35, 0.72), 3.85, 1.95, boxstyle="round,pad=0.08", fc="#fff4f4", ec="#cc3333", lw=1.2))
ax1.text(7.55, 2.36, "「校正での切り替え」とは", ha="left", va="center", fontsize=9.8, color="#cc3333", weight="bold")
ax1.text(7.55, 1.92, "定期校正 (2.5 分ごと) で", ha="left", va="center", fontsize=9.0, color="#333")
ax1.text(7.55, 1.55, "c2 を GP4→GP2→GP4 と動かし、", ha="left", va="center", fontsize=9.0, color="#333")
ax1.text(7.55, 1.18, "c0 と同じエッジでズレを測り直す", ha="left", va="center", fontsize=9.0, color="#333")

# ================= 下段: 見張った 2 つの差の時間変化 (実データ) =================
ax2.text(0.02, 1.14, "2 つの差の変化 (実測、2.4 時間)", transform=ax2.transAxes,
         ha="left", va="bottom", fontsize=10.5)
ax2.text(0.02, 1.03, "tick = カウンタが 1 つ進む幅 = 16 ns", transform=ax2.transAxes,
         ha="left", va="bottom", fontsize=8.6, color="#888")

# KEXP ログ (2.4 時間) から k (=カウンタのズレ、校正が測り直す値) と c0-c3 を読む
_KEXP = re.compile(r"KEXP count=(\d+) gen=\d+ c0=(\d+) c2=\d+ c3=(\d+) c3n=(\d+) k=(\d+) kt=\d+")
def _f32(u):
    u &= 0xFFFFFFFF
    return u - (1 << 32) if u >= (1 << 31) else u
_KLOG = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))), "logs",
                     "20260703-kexp", "kexp-run.log")
tm, kz, s3, _c0 = [], [], [], None
for _line in open(_KLOG, errors="replace"):
    _m = _KEXP.search(_line)
    if not _m:
        continue
    cnt, c0, c3, c3n, kk = (int(_m.group(i)) for i in (1, 2, 3, 4, 5))
    if c3n < 1:
        continue
    if _c0 is None:
        _c0 = cnt
    tm.append((cnt - _c0) / 60.0); kz.append(_f32(kk)); s3.append(_f32(c0 - c3))
kz = [v - kz[0] for v in kz]; s3 = [v - s3[0] for v in s3]

ax2.axhline(0, color="#ccc", lw=0.8, zorder=0)
ax2.plot(tm, s3, color="#8a5a00", lw=1.9, label="c0 − c3 (同じ GP2、切替なし)")
ax2.plot(tm, kz, color="#cc3333", lw=1.6, label="c0 − c2 (カウンタのズレ、c2 は校正で切替)")

ax2.annotate("平坦 → 同じものを見る 2 本は離れない (カウンタごとの癖なし)", xy=(tm[-1] * 0.75, s3[len(s3) * 3 // 4]),
             xytext=(tm[-1] * 0.20, 20), fontsize=8.8, color="#8a5a00",
             arrowprops=dict(arrowstyle="->", color="#8a5a00", lw=0.9))
ax2.annotate("校正のたびに −4 tick 下がる\n2.4h で −228 tick (−3.6 µs)", xy=(tm[-1], kz[-1]),
             xytext=(tm[-1] * 0.30, -170), fontsize=8.8, color="#cc3333",
             arrowprops=dict(arrowstyle="->", color="#cc3333", lw=0.9))

ax2.set_xlim(0, tm[-1] * 1.02); ax2.set_ylim(-250, 42)
ax2.set_xlabel("時間 [min]", fontsize=9.5)
ax2.set_ylabel("開始からの変化 [tick]", fontsize=9.5)
ax2.tick_params(labelsize=8)
for sp in ("top", "right"):
    ax2.spines[sp].set_visible(False)
ax2.legend(loc="lower left", fontsize=8.4, framealpha=0.9)
ax2.text(0.02, -0.24, "この −228 tick はピン (オシロ) には出ない → 内部だけの見かけ",
         transform=ax2.transAxes, ha="left", va="top", fontsize=8.8, color="#666")

fig.tight_layout(h_pad=1.2)
fig.savefig(os.path.join(OUT, "fig-kexp-setup.png"), dpi=110); plt.close(fig)
print("wrote fig-kexp-setup.png")
