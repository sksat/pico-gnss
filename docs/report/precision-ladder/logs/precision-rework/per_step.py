#!/usr/bin/env python3
"""手法を変えた前後を時系列で見せる per-step 図。既存ログから生成。座標は出さない。"""
import re, os, statistics as st
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc","/usr/share/fonts/noto-cjk/NotoSansCJK-Light.ttc"):
    if os.path.exists(fp):
        try: font_manager.fontManager.addfont(fp)
        except: pass
plt.rcParams["font.family"]="Noto Sans CJK JP"; plt.rcParams["axes.unicode_minus"]=False
HERE=os.path.dirname(os.path.abspath(__file__))
REPORT=os.path.dirname(os.path.dirname(HERE))
ROOT=os.path.dirname(os.path.dirname(os.path.dirname(REPORT)))
LOG=os.path.join(ROOT,"logs","precision-rework")  # 生データ (gitignore、ローカルのみ)
OUT=os.path.join(REPORT,"precision-figs")

def rows(p,line,warm=5):
    out=[]
    for ln in open(os.path.join(LOG,p),errors="replace"):
        if line not in ln: continue
        d={k:int(v) for k,v in re.findall(r"(\w+)=(-?\d+)",ln)}
        if d.get("count",0)>warm: out.append(d)
    return out
def col(r,k): return [d[k] for d in r if k in d]
def adjdiff_gps(p,warm):
    # GPS 1PPS 周期 (PPS 行の interval_ns) の隣り合う差。ソフトも PIO も同じ「GPS 周期を firmware が計った値」
    r=rows(p,"PPS count=",warm); iv=col(r,"interval_ns"); c=[d["count"] for d in r if "interval_ns" in d]
    return c[1:],[iv[i+1]-iv[i] for i in range(len(iv)-1)]

# ---- 図: ソフト計測 vs PIO 計測 (firmware が GPS 1PPS 周期を計った隣り合う差・縦軸統一) ----
c1,a1=adjdiff_gps("S1.log",120); c2,a2=adjdiff_gps("S2.log",120)
s1=st.pstdev(a1)/1000; s2ns=st.pstdev(a2)
fig,(ax,bx)=plt.subplots(1,2,figsize=(10,4.0),sharey=True)
ax.plot([x-c1[0] for x in c1],[v/1000 for v in a1],"-",color="#c33",lw=0.8)
ax.set_title("ソフトで計る (GPIO 割り込み + µs 時計)"); ax.set_xlabel("経過 [秒]"); ax.set_ylabel("隣り合う GPS 周期の差 [µs]")
ax.text(0.97,0.95,f"σ ≈ {s1:.1f} µs",transform=ax.transAxes,ha="right",va="top",fontsize=11,color="#a22")
ax.grid(ls=":",alpha=0.4)
bx.plot([x-c2[0] for x in c2],[v/1000 for v in a2],"-",color="#4a7",lw=0.9)
bx.set_title("PIO で捕捉 (ハードで時刻決定)"); bx.set_xlabel("経過 [秒]")
bx.text(0.97,0.95,f"σ ≈ {s2ns:.0f} ns\n(≈ ほぼ 0 に潰れる)",transform=bx.transAxes,ha="right",va="top",fontsize=11,color="#272")
bx.grid(ls=":",alpha=0.4)
fig.suptitle("PIO に替えると、計測の揺れが µs から ns に潰れる (縦軸は両側そろえて µs)",fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(OUT,"fig-pio-meas.png"),dpi=110); plt.close(fig)

# ---- 図: 開ループ vs PLL (出力位相 hwphase・縦軸統一) ----
o=rows("S2.log","PPSGEN count=",120); l=rows("S3_locked.log","PPSGEN count=",0)
co=[d["count"] for d in o if "hwphase_ns" in d]; ho=[d["hwphase_ns"]-st.mean(col(o,"hwphase_ns")) for d in o if "hwphase_ns" in d]
cl=[d["count"] for d in l if "hwphase_ns" in d]; hl=[d-st.mean(col(l,"hwphase_ns")) for d in col(l,"hwphase_ns")]
so=st.pstdev(col(o,"hwphase_ns"))/1000; sl=st.pstdev(col(l,"hwphase_ns"))
fig,(ax,bx)=plt.subplots(1,2,figsize=(10,4.0),sharey=True)
ax.plot([x-co[0] for x in co],[v/1000 for v in ho],"-",color="#c33",lw=0.9)
ax.axhline(0,color="#888",lw=0.6)
ax.set_title("PLL なし (開ループ)"); ax.set_xlabel("経過 [秒]"); ax.set_ylabel("出力位相 (中心からの差) [µs]")
ax.text(0.97,0.95,f"σ ≈ {so:.1f} µs",transform=ax.transAxes,ha="right",va="top",fontsize=11,color="#a22")
ax.grid(ls=":",alpha=0.4)
bx.plot([x-cl[0] for x in cl],[v/1000 for v in hl],"-",color="#4a7",lw=0.9)
bx.axhline(0,color="#888",lw=0.6)
bx.set_title("PLL で閉じる (ロック中)"); bx.set_xlabel("経過 [秒]")
bx.text(0.97,0.95,f"σ ≈ {sl:.0f} ns\n(≈ ほぼ 0 に潰れる)",transform=bx.transAxes,ha="right",va="top",fontsize=11,color="#272")
bx.grid(ls=":",alpha=0.4)
fig.suptitle("PLL で閉じると、出力位相の揺れが µs から ns に潰れる (縦軸は両側そろえて µs)",fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(OUT,"fig-pll-lock.png"),dpi=110); plt.close(fig)
print("wrote fig-pio-meas.png, fig-pll-lock.png")
print(f"GPS-interval adj σ: soft={s1:.1f}µs PIO={s2ns:.0f}ns | hwphase σ: openloop={so:.1f}µs PLL={sl:.0f}ns")
