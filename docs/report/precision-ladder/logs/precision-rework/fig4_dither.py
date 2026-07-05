#!/usr/bin/env python3
"""fig4: dither の定常の様子。warm(周波数が落ち着いた)区間で、毎周期ほぼ 1000003 を選びつつ
たまに 1000002 を混ぜ、平均を整数の間 (~1000002.9 tick) に置く。cold boot の引き込み混入は除く。"""
import re, os, statistics as st
from collections import Counter
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except: pass
plt.rcParams["font.family"]="Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"]=False
HERE=os.path.dirname(os.path.abspath(__file__))
OUT=os.path.join(os.path.dirname(os.path.dirname(HERE)),"docs","report","precision-ladder","precision-figs")
LOG=os.path.join(os.path.dirname(HERE),"20260630-precision-recap","S1-warm-dither.log")

d=[];c=[]
for ln in open(LOG,errors="replace"):
    m=re.search(r"PPSGEN count=(\d+).*?dith_ticks=(\d+)",ln)
    if m and int(m.group(2))>0: c.append(int(m.group(1))); d.append(int(m.group(2)))
# 定常 = 周波数推定が落ち着いた後 (count>=250)
st_idx=[i for i,x in enumerate(c) if x>=250]
ds=[d[i] for i in st_idx]; cs=[c[i] for i in st_idx]
mean=st.mean(ds)

fig,(ax,bx)=plt.subplots(1,2,figsize=(10,4.0),gridspec_kw={"width_ratios":[3,2]})
base=min(ds)
seg=ds[:80]
ax.step(range(len(seg)),[v-base for v in seg],where="mid",color="#36a",lw=1.0)
ax.axhline(mean-base,color="#c33",ls="--",lw=1.2,label=f"平均 {mean:.2f}")
ax.set_xlabel("周期の番号 (定常区間)")
ax.set_ylabel(f"選ばれた周期  [µs]  (+{base})")
ax.set_title("毎周期は 1 µs 単位のどれかを選ぶ")
ax.legend(loc="center right",fontsize=9); ax.grid(ls=":",alpha=0.4)
cnt=Counter(ds); keys=sorted(cnt)
bx.barh([str(k) for k in keys],[cnt[k] for k in keys],color="#88b")
bx.axhline(mean-keys[0],color="#c33",ls="--",lw=1.2)
bx.set_xlabel("出現回数"); bx.set_ylabel("選ばれた周期  [µs]")
bx.set_title("混ぜると平均は 1 µs 単位の間に来る")
bx.grid(ls=":",alpha=0.4,axis="x")
fig.suptitle(f"出力周期は 1 µs 単位だけ。ほぼ 1000003 µs を選びつつ稀に 1000002 µs を混ぜ、平均を狙い ({mean:.2f} µs) に置く",fontsize=11.5)
fig.tight_layout(); fig.savefig(os.path.join(OUT,"fig4-dither.png"),dpi=110); plt.close(fig)
print(f"wrote fig4 (steady n={len(ds)} mean={mean:.3f} dist={dict(sorted(cnt.items()))})")
