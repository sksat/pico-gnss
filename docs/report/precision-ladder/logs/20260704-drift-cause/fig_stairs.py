#!/usr/bin/env python3
"""fig-wrap-fold: 一周イベント (8 ns の段) が実データに写っていることの可視化。
同じ GP4 を見る 2 本 (c2, c3) の読み差は、片方が一周を踏むと 0.5 tick (8 ns) ずれ、
もう片方が踏むと戻る。毎秒のディザに埋もれて直接は見えないが、一周の理論周期
(2^32 tick = 68.7195 s) で折り畳むと、この「行って戻る」窓が浮き出る。
左: 周回 × 位相のラスタ。右: 位相ごとの平均 (無関係な周期の対照つき)。
usage: uv run --with matplotlib python3 logs/20260704-drift-cause/fig_stairs.py
"""
import os
import re
import statistics as st
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(os.path.dirname(HERE)),
                   "docs", "report", "precision-ladder", "precision-figs")
KEXP = re.compile(r"KEXP count=(\d+) gen=\d+ c0=(\d+) c2=(\d+) c3=(\d+) c3n=(\d+)")
P = 4294967296 * 16e-9  # 一周の理論周期 [s]


def f32(u):
    u &= 0xFFFFFFFF
    return u - (1 << 32) if u >= (1 << 31) else u


rows = []
for ln in open(os.path.join(HERE, "c3gp4-rtt.log"), errors="replace"):
    m = KEXP.search(ln)
    if m:
        cnt, c0, c2, c3, c3n = (int(m.group(i)) for i in range(1, 6))
        if c3n == 1 and c0 != 0 and c3 != 0:
            rows.append((cnt, f32(c2 - c3)))
med = st.median(d for _, d in rows)
base = min(d for _, d in rows if abs(d - med) < 2)
data = [(c, d - base) for c, d in rows if abs(d - med) < 2]  # 0/1


def fold(period, nb=34):
    bins = [[] for _ in range(nb)]
    for c, v in data:
        ph = (c % period) / period
        bins[int(ph * nb) % nb].append(v)
    return [st.mean(b) * 16 if b else float("nan") for b in bins]


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.6, 4.4), gridspec_kw={"width_ratios": [1.0, 1.0]})

# ---- 左: 折り畳みで見えた「1 段の往復」(実データ) ----
NB = 34
xs = [(i + 0.5) * P / NB for i in range(NB)]
ys = fold(P, NB)
ax1.step(xs, ys, where="mid", color="#1a7a1a", lw=1.8, label=f"一周の周期 {P:.2f} s で折り畳んだ平均")
ax1.plot([(i + 0.5) * 61.0 / NB for i in range(NB)], fold(61.0, NB), "-", color="#c8c8c8", lw=1.2,
         label="無関係な周期 (61 s) だと平坦 (対照)")
ax1.axhline(6.4, color="#9c9", lw=0.8, ls=":")
ax1.axhline(13.5, color="#9c9", lw=0.8, ls=":")
ax1.annotate("片方の一周 (+8 ns)", xy=(60.8, 12.0), xytext=(37, 14.4), fontsize=8.8, color="#1a7a1a",
             arrowprops=dict(arrowstyle="->", color="#1a7a1a", lw=0.9))
ax1.annotate("もう片方の一周で戻る (−8 ns)\n(継ぎ目 = 一周の境界を跨ぐ)", xy=(1.5, 6.4), xytext=(11, 1.6),
             fontsize=8.8, color="#1a7a1a", arrowprops=dict(arrowstyle="->", color="#1a7a1a", lw=0.9))
ax1.set_ylim(-1, 17)
ax1.set_xlabel("一周の中の位相 [s]")
ax1.set_ylabel("2 本の読み差の平均 [ns]")
ax1.set_title("実データ: 同じピンの 2 本では、段は往復する (47 分の折り畳み)")
ax1.grid(ls=":", alpha=0.35)
ax1.legend(loc="upper left", fontsize=8)

# ---- 右: 積もり方の模式 (同じピン vs ピン違い) ----
ax2.set_xlim(0, 10); ax2.set_ylim(0, 10); ax2.axis("off")
ax2.set_title("模式: 段が往復すれば積もらず、片道なら階段に積もる")


def steps(ax, x0, y0, updown, dx=0.55, dy=0.75, color="#446"):
    """updown: '+', '-' の列。階段の折れ線を描いて終端座標を返す。"""
    x, y = x0, y0
    xs, ys = [x], [y]
    for ch in updown:
        x += dx
        xs.append(x); ys.append(y)
        y += dy if ch == "+" else -dy
        xs.append(x); ys.append(y)
    x += dx
    xs.append(x); ys.append(y)
    ax.plot(xs, ys, color=color, lw=1.8)
    return x, y


ax2.text(0.3, 8.9, "同じピンを見る 2 本 (行って戻る)", fontsize=9.4, color="#22661f")
steps(ax2, 0.5, 7.3, "+-+-+-+-", color="#22661f")
ax2.text(5.6, 7.5, "→ 離れない (実測 ±1 tick)", fontsize=8.6, color="#22661f")

ax2.text(0.3, 4.7, "ピンが違う 2 本 (戻りが 9 割来ない)", fontsize=9.4, color="#cc3333")
steps(ax2, 0.5, 1.2, "++-++++", color="#cc3333")
ax2.text(5.9, 3.6, "→ 8 ns ずつ積もる\n   (16 ns の測定目盛りごしには\n    2 段ごとの階段に見える)", fontsize=8.6, color="#cc3333")
ax2.text(0.3, 0.35, "1 段 = 8 ns、段の間隔 = 一周 68.7 秒 (ずれの速さ 5.6 ns/min)", fontsize=8.8, color="#555")

fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig-wrap-fold.png"), dpi=110)
print("wrote fig-wrap-fold.png")
