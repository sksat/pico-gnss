#!/usr/bin/env python3
"""ソフト計測の揺れの図 (fig-naive-alone の置き換え)。8 boot の GPS-R 1PPS 間隔の読み値から
boot ごとの平均を引いてまとめ、ヒストグラムにする。本体は ±2 µs、まれに数十 µs 跳ぶ heavy tail
(= エッジが critical section に当たった回) を log-y で見せる。
usage: uv run --with matplotlib python3 logs/20260703-naive-boots/fig_naive_jitter.py
"""
import re, os, glob, statistics as st
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except Exception: pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(ROOT, "docs", "report", "precision-ladder", "precision-figs")

pooled = []
for p in sorted(glob.glob(os.path.join(HERE, "boot*.log"))):
    dv = []
    for ln in open(p, errors="replace"):
        if "PPSGEN count=" not in ln:
            continue
        d = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", ln)}
        if d.get("count", 0) > 5:
            dv.append(d["dev_ns"])
    m = st.mean(dv)
    pooled += [(v - m) / 1e3 for v in dv]

sig = st.pstdev(pooled)
within2 = 100 * sum(1 for v in pooled if abs(v) <= 2) / len(pooled)
outliers = [v for v in pooled if abs(v) > 10]
print(f"n={len(pooled)} σ={sig:.2f} µs  ±2µs 内 {within2:.0f}%  >10µs {len(outliers)} 点 (最大 {max(abs(v) for v in pooled):.0f} µs)")

fig, ax = plt.subplots(figsize=(8.0, 3.8))
clip = [max(-14.5, min(14.5, v)) for v in pooled]  # 枠外は端のビンへ寄せ、注記で示す
ax.hist(clip, bins=[x * 0.5 for x in range(-30, 31)], color="#4477cc", log=True)
ax.axvline(0, color="#333", lw=0.9)
ax.set_xlabel("読み値のずれ (µs、その boot の平均との差)")
ax.set_ylabel("回数 (log)")
ax.set_title(f"同じ 1 秒を測っても読み値は µs 単位でばらつく (8 回の boot、σ ≈ {sig:.0f} µs)")
ax.annotate(f"ほとんど (約 {within2:.0f}%) は ±2 µs に収まる", xy=(0.03, 0.88), xycoords="axes fraction", fontsize=10)
ax.annotate(f"まれに大きく跳ぶ:\n>10 µs が {len(outliers)} 点 (最大 {max(abs(v) for v in pooled):.0f} µs)\n→ 端のビンにまとめて表示",
            xy=(0.70, 0.66), xycoords="axes fraction", fontsize=9, color="#a33")
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig-naive-jitter.png"), dpi=110)
print("wrote fig-naive-jitter.png")
