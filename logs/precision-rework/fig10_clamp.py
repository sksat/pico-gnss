#!/usr/bin/env python3
"""fig10: clamp の加熱 A/B。過渡 peak/℃ が中央値で約8倍・worst で約14倍縮むのを一目で。
検証済みの per-event 値から作る (unclamped 中央値3085/worst5573、clamped 397/400/558)。log 軸で比率を直感的に。"""
import os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except: pass
plt.rcParams["font.family"]="Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"]=False
OUT=os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),"docs","report","precision-ladder","precision-figs")

unc_med, unc_worst = 3085.0, 5573.0          # clamp なし (温度FF が steering に出口制限なし)
cl = [397.0, 400.0, 558.0]                    # clamp あり (±100 ppb 制限)
cl_med = 400.0

fig, ax = plt.subplots(figsize=(7.6,4.4))
ax.set_yscale("log")
# 棒: 中央値
ax.bar([0],[unc_med],width=0.5,color="#c33",alpha=0.85,label="clamp なし")
ax.bar([1],[cl_med],width=0.5,color="#4a7",alpha=0.85,label="clamp あり (±100 ppb)")
# 個別点
ax.plot([0,0],[unc_med,unc_worst],"-",color="#822",lw=1)
ax.scatter([0,0],[unc_med,unc_worst],color="#822",zorder=3,s=28)
ax.annotate("worst 5573",(0,unc_worst),xytext=(8,0),textcoords="offset points",va="center",fontsize=9,color="#822")
ax.annotate("中央値 3085",(0,unc_med),xytext=(8,-2),textcoords="offset points",va="center",fontsize=9,color="#822")
ax.scatter([1]*len(cl),cl,color="#272",zorder=3,s=28)
ax.annotate("397 / 400 / 558",(1,cl_med),xytext=(8,0),textcoords="offset points",va="center",fontsize=9,color="#272")
# 比率の矢印注記
ax.annotate("", xy=(0.5,cl_med), xytext=(0.5,unc_med),
            arrowprops=dict(arrowstyle="<->",color="#444",lw=1.4))
ax.text(0.56,(unc_med*cl_med)**0.5,"中央値で約 8 倍\n(worst では約 14 倍)",fontsize=11,va="center",
        bbox=dict(boxstyle="round",fc="#fffbe6",ec="#caa"))
ax.set_xticks([0,1]); ax.set_xticklabels(["clamp なし","clamp あり"])
ax.set_ylabel("加熱時の過渡 peak / ℃  [ns/℃]  (対数軸)")
ax.set_ylim(200,8000)
ax.set_title("出力 steering に ±100 ppb の制限を入れると、加熱の過渡が中央値で約 8 倍縮む")
ax.grid(ls=":",alpha=0.4,axis="y")
fig.tight_layout(); fig.savefig(os.path.join(OUT,"fig10-clamp-ab.png"),dpi=110); plt.close(fig)
print("wrote fig10-clamp-ab.png (median %.0f vs %.0f = %.1fx; worst %.0f = %.1fx)"%(unc_med,cl_med,unc_med/cl_med,unc_worst,unc_worst/cl_med))
