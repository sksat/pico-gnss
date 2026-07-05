#!/usr/bin/env python3
"""fig11-tempff-abab: 温度フィードフォワードだけを 30 分ごとに on/off 交互切替した
夜通し運転の可視化。上: firmware の loopback 位相 + 基板温度、中: オシロ実測
(GPS-R PPS vs 出力エッジ差)、下: 区間 (30 分の切替のひとまとまり) ごとの σ。
各セグメント冒頭 5 分は切替の整定として統計から除外する。温度は RP2040 内蔵センサを ℃ に換算。
usage: uv run --with matplotlib python3 logs/20260705-tempff-abab/fig_abab.py
"""
import os
import re
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

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.dirname(os.path.dirname(HERE))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(REPORT)))
DATA = os.path.join(ROOT, "logs", "20260705-tempff-abab")  # 生データ (gitignore、ローカルのみ)
OUT = os.path.join(REPORT, "precision-figs")
SETTLE = 300  # 切替後の整定として捨てる秒数

PPS = re.compile(r"count=(\d+) .*hwphase_ns=(-?\d+).*lk=(\d).*temp_raw=(\d+)")
TFF = re.compile(r"TFFAB count=(\d+) temp_ff=(\d)")

rows = []     # (count, hwphase, temp_raw)
bounds = []   # (count, new_state)
for line in open(os.path.join(DATA, "rtt.log"), errors="replace"):
    m = TFF.search(line)
    if m:
        bounds.append((int(m.group(1)), int(m.group(2))))
        continue
    m = PPS.search(line)
    if m and m.group(3) == "1":
        rows.append((int(m.group(1)), int(m.group(2)), int(m.group(4))))

# セグメント列: (state, c0, c1)。最初のセグメントの state は最初の切替の逆。
c_first, c_last = rows[0][0], rows[-1][0]
edges = [(c_first, 1 - bounds[0][1])] + bounds
segs = [(s, c0, (edges[i + 1][0] if i + 1 < len(edges) else c_last + 1))
        for i, (c0, s) in enumerate(edges)]

# オシロ shots: epoch → count へ (rtt.log の mtime と最終 count を錨にする)
anchor_epoch = os.path.getmtime(os.path.join(DATA, "rtt.log"))
shots = []    # (count 相当, ns)
for line in open(os.path.join(DATA, "abab.shots")):
    t, v = line.split()
    shots.append((float(t) - anchor_epoch + c_last, float(v)))

# 序盤にオシロ復旧の取得の穴があるので、穴のあとの最初のセグメント境界から先だけを
# 図と統計に使う (両計器を同じ窓でそろえ、部分区間も作らない)
CUT_AFTER_GAP = True
if CUT_AFTER_GAP:
    gap_ends = [b for (a, _), (b, _) in zip(shots, shots[1:]) if b - a > 240]
    if gap_ends:
        cut = min(c0 for _, c0, _ in segs if c0 >= max(gap_ends))
        segs = [(s, c0, c1) for s, c0, c1 in segs if c0 >= cut]
        rows = [r for r in rows if r[0] >= cut]
        shots = [p for p in shots if p[0] >= cut]
        c_first = cut


def seg_stats(data):
    """data: (count, value) の列 → セグメントごとの (state, c0, c1, vals)"""
    out = []
    for s, c0, c1 in segs:
        vals = [v for c, v in data if c0 + SETTLE <= c < c1]
        if len(vals) >= 100:
            out.append((s, c0, c1, vals))
    return out

hw = [(c, h) for c, h, _ in rows]
hw_segs = seg_stats(hw)
sc_segs = seg_stats(shots)

def temp_c(raw):
    return 27 - ((raw / 256) * 3.3 / 4096 - 0.706) / 0.001721

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 8.2), sharex=True,
                                    gridspec_kw={"hspace": 0.15,
                                                 "height_ratios": [3, 3, 1.9]})
h0 = c_first
YLIM = 560  # あり/なし ラベルの行のぶん上に余白を取る

def shade(ax):
    for s, c0, c1 in segs:
        ax.axvspan((c0 - h0) / 3600, (c1 - h0) / 3600,
                   color=("#2a9d3a" if s else "#c0392b"), alpha=0.07, lw=0)

def annotate(ax, seg_list):
    for s, c0, c1, vals in seg_list:
        x = ((c0 + c1) / 2 - h0) / 3600
        ax.text(x, 528, "あり" if s else "なし",
                ha="center", va="top", fontsize=8.5, fontweight="bold",
                color=("#1e7a2e" if s else "#a83226"))

# 上段: loopback 位相 + 温度
xs = [(c - h0) / 3600 for c, _ in hw]
ax1.plot(xs, [h for _, h in hw], lw=0.4, color="#1f77b4", alpha=0.8)
shade(ax1)
ax1.axhline(0, color="gray", lw=0.5)
ax1.set_ylabel("loopback 位相 [ns]")
ax1.set_ylim(-YLIM, YLIM)
ax1.set_yticks(range(-400, 401, 200))
annotate(ax1, hw_segs)  # あり/なし の帯ラベルは最上段だけに置く
axt = ax1.twinx()
t_step = max(1, len(rows) // 2000)
axt.plot(xs[::t_step], [temp_c(t) for _, _, t in rows][::t_step],
         lw=1.2, color="#8a6d3b", alpha=0.55)
axt.set_ylabel("基板温度 [℃]", color="#8a6d3b")
axt.tick_params(axis="y", labelcolor="#8a6d3b")
ax1.set_title("温度フィードフォワードを 30 分ごとに あり/なし 交互切替した夜通し運転", pad=14)

# 下段: オシロ実測
xs2 = [(c - h0) / 3600 for c, _ in shots]
ax2.plot(xs2, [v for _, v in shots], ".", ms=1.2, color="#444444", alpha=0.45)
shade(ax2)
ax2.axhline(0, color="gray", lw=0.5)
ax2.set_ylabel("オシロ実測 出力−GPS-R [ns]")
ax2.set_ylim(-YLIM, YLIM)
ax2.set_yticks(range(-400, 401, 200))

# 下段: 区間ごとの σ。切替のたびに下がる/戻るが一目で見えるように棒で並べる
shade(ax3)
for s3, c0, c1, vals in sc_segs:
    x0, x1 = (c0 - h0) / 3600, (c1 - h0) / 3600
    ax3.bar((x0 + x1) / 2, st.pstdev(vals), width=(x1 - x0) * 0.86,
            color=("#2a9d3a" if s3 else "#c0392b"), alpha=0.65, lw=0,
            label="_")
hw_pts = [(((c0 + c1) / 2 - h0) / 3600, st.pstdev(vals)) for _, c0, c1, vals in hw_segs]
ax3.plot([x for x, _ in hw_pts], [y for _, y in hw_pts], "o-", ms=4.5, lw=0.9,
         color="#222222", alpha=0.8)
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
ax3.legend(handles=[Patch(color="#2a9d3a", alpha=0.65, label="オシロ実測 (あり)"),
                    Patch(color="#c0392b", alpha=0.65, label="オシロ実測 (なし)"),
                    Line2D([], [], marker="o", ms=4.5, lw=0.9, color="#222222",
                           label="loopback 位相")],
           loc="upper right", fontsize=8, ncols=3)
ax3.set_ylabel("区間ごとの σ [ns]")
ax3.set_xlabel("経過時間 [h]")
ax3.set_ylim(0, 200)
# 取得の穴 (scope 復旧作業) を正直に注記する。近接する穴は 1 つに束ねる
raw_gaps = [(a, b) for (a, _), (b, _) in zip(shots, shots[1:]) if b - a > 240]
gaps = []
for a, b in raw_gaps:
    if gaps and a - gaps[-1][1] < 600:
        gaps[-1] = (gaps[-1][0], b)
    else:
        gaps.append((a, b))
for a, b in gaps:
    if b - a < 600:
        continue
    xm = ((a + b) / 2 - h0) / 3600
    ax2.text(xm, -560, "取得の穴\n(オシロ復旧)", ha="center", va="center",
             fontsize=8, color="#666666")

fig.savefig(os.path.join(OUT, "fig11-tempff-abab.png"), dpi=130,
            bbox_inches="tight")
print("saved fig11-tempff-abab.png")

# 温度外乱の公平性: 区間内の温度振れ幅 (p-p) が両群で揃っているか
tt = [(c, temp_c(t)) for c, _, t in rows]
for want, name in ((0, "off"), (1, "on")):
    pps = []
    for s, c0, c1 in segs:
        if s != want:
            continue
        seg_t = [v for c, v in tt if c0 + SETTLE <= c < c1]
        if len(seg_t) >= 100:
            pps.append(max(seg_t) - min(seg_t))
    if pps:
        print(f"[temp] {name}: 区間内温度 p-p 中央値 {st.median(pps):.2f}℃ "
              f"(範囲 {min(pps):.2f}〜{max(pps):.2f}, n={len(pps)})")

# 統計サマリ
for name, seg_list in (("hwphase", hw_segs), ("scope", sc_segs)):
    on = [v for s, _, _, vals in seg_list if s for v in vals]
    off = [v for s, _, _, vals in seg_list if not s for v in vals]
    print(f"[{name}] セグメント:")
    for s, c0, c1, vals in seg_list:
        a = sorted(abs(x) for x in vals)
        print(f"  {'on ' if s else 'off'} n={len(vals):5d} mean={st.mean(vals):+7.1f} "
              f"σ={st.pstdev(vals):6.1f} p95|x|={a[int(0.95 * len(a))]:5.0f} "
              f"≤100ns={100 * sum(1 for x in a if x <= 100) / len(a):.0f}%")
    if on and off:
        aon = sorted(abs(x) for x in on)
        aoff = sorted(abs(x) for x in off)
        print(f"  pool on : n={len(on):5d} σ={st.pstdev(on):6.1f} "
              f"≤100ns={100 * sum(1 for x in aon if x <= 100) / len(aon):.0f}%")
        print(f"  pool off: n={len(off):5d} σ={st.pstdev(off):6.1f} "
              f"≤100ns={100 * sum(1 for x in aoff if x <= 100) / len(aoff):.0f}%")
