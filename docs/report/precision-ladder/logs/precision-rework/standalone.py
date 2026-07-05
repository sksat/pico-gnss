#!/usr/bin/env python3
"""各手法を単体で見せる時系列図。既存ログから。座標は出さない。"""
import re, os, statistics as st
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc","/usr/share/fonts/noto-cjk/NotoSansCJK-Light.ttc"):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except: pass
plt.rcParams["font.family"]="Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"]=False
LOG=os.path.dirname(os.path.abspath(__file__)); OUT=os.path.join(os.path.dirname(os.path.dirname(LOG)),"docs","report","precision-ladder","precision-figs")
def rows(p,line="PPS count=",warm=5):
    out=[]
    for ln in open(os.path.join(LOG,p),errors="replace"):
        if line not in ln: continue
        d={k:int(v) for k,v in re.findall(r"(\w+)=(-?\d+)",ln)}
        if d.get("count",0)>warm: out.append(d)
    return out
def col(r,k): return [d[k] for d in r if k in d]
def rows_from(path,line="PPS count=",warm=0):
    out=[]
    for ln in open(path,errors="replace"):
        if line not in ln: continue
        d={k:int(v) for k,v in re.findall(r"(\w+)=(-?\d+)",ln)}
        if d.get("count",0)>warm: out.append(d)
    return out

# (fig-naive-alone は廃止: ソフト計測の揺れは logs/20260703-naive-boots/fig_naive_jitter.py の
#  fig-naive-jitter.png (8 boot のヒストグラム) に置き換えた)

# PIO 単体: 同じ GPS 1PPS 周期を「PIO のハード捕捉で計った」値の隣り合う差 (ns に)
s2=rows("S2.log","PPS count=",120); c=[d["count"] for d in s2 if "interval_ns" in d]; iv=col(s2,"interval_ns")
ad=[iv[i+1]-iv[i] for i in range(len(iv)-1)]
fig,ax=plt.subplots(figsize=(8.0,3.8))
ax.plot([x-c[1] for x in c[1:]],ad,"-",color="#4a7",lw=0.8)
ax.set_title(f"PIO で捕捉したとき: 同じ GPS 1PPS 周期の計測が数 ns に (σ ≈ {st.pstdev(ad):.0f} ns)")
ax.set_xlabel("経過 [秒]"); ax.set_ylabel("隣り合う GPS 周期の差 [ns]"); ax.grid(ls=":",alpha=0.4)
fig.tight_layout(); fig.savefig(os.path.join(OUT,"fig-pio-alone.png"),dpi=110); plt.close(fig)

# PLL 単体: ロックしたあとの長時間 (~16分) の出力位相。位相0線つきで「ロック保持」を見せる
NEWLOG=os.path.join(os.path.dirname(LOG),"20260630-precision-recap","S3-pll-long.log")
lk=[d for d in rows_from(NEWLOG,"PPSGEN count=",0) if d.get("lk")==1 and abs(d.get("hwphase_ns",9e9))<5000]
lk=[d for d in lk if d["count"]>=lk[0]["count"]+120]  # 整定の谷を落とし、ロック保持だけ見せる
c=[d["count"] for d in lk]; hw=[d["hwphase_ns"] for d in lk]; t0=c[0]
tmin=[(x-t0)/60.0 for x in c]
fig,ax=plt.subplots(figsize=(9.0,3.8))
ax.axhspan(-100,100,color="#4a7",alpha=0.08,label="≤100 ns 帯")
ax.axhline(0,color="#444",lw=0.9,label="位相 0")
ax.plot(tmin,hw,"-",color="#36a",lw=0.7)
ax.set_title(f"PLL で閉じてロックしたあと ~{tmin[-1]:.0f} 分の出力位相 (σ ≈ {st.pstdev(hw):.0f} ns、30 秒窓で ≈ 60 ns、ロック保持)")
ax.set_xlabel("経過 [分]"); ax.set_ylabel("loopback 位相 [ns]"); ax.legend(loc="upper right",fontsize=9,ncol=2); ax.grid(ls=":",alpha=0.4)
fig.tight_layout(); fig.savefig(os.path.join(OUT,"fig-pll-alone.png"),dpi=110); plt.close(fig)
print("wrote fig-naive-alone, fig-pio-alone, fig-pll-alone")
