#!/usr/bin/env python3
"""precision.md 用の図を実ログから生成する。座標は一切出さない。
出力: docs/report/precision-figs/*.png
usage: uv run --with matplotlib python3 logs/precision-rework/plot_figs.py
"""
import re, os, statistics as st
from collections import Counter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
for _fp in ("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/noto-cjk/NotoSansCJK-Light.ttc"):
    if os.path.exists(_fp):
        try:
            font_manager.fontManager.addfont(_fp)
        except Exception:
            pass
plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LOG = os.path.join(ROOT, "logs", "precision-rework")
OUT = os.path.join(ROOT, "docs", "report", "precision-ladder", "precision-figs")
os.makedirs(OUT, exist_ok=True)

def rows(path, warm=0):
    out = []
    for ln in open(path, errors="replace"):
        if "PPSGEN count=" not in ln:
            continue
        d = {k: int(v) for k, v in re.findall(r"(\w+)=(-?\d+)", ln)}
        m = re.match(r"^(\d+\.\d+)\s", ln)
        d["ts"] = float(m.group(1)) if m else None
        if d.get("count", 0) > warm:
            out.append(d)
    return out

def pstdev(xs):
    return st.pstdev(xs) if len(xs) > 1 else 0.0

# --- metrics from logs ---
s0 = rows(os.path.join(LOG, "S0.log"), 5)
s1 = rows(os.path.join(LOG, "S1.log"), 120)
s2 = rows(os.path.join(LOG, "S2.log"), 120)
s3l = rows(os.path.join(LOG, "S3_locked.log"), 0)

def adjdiff_sd(rs):
    iv = [d["interval_ns"] for d in rs if "interval_ns" in d]
    return pstdev([iv[i+1]-iv[i] for i in range(len(iv)-1)])

adj = {"S0": adjdiff_sd(s0), "S1": adjdiff_sd(s1), "S2": adjdiff_sd(s2)}
hwsd_s2 = pstdev([d["hwphase_ns"] for d in s2])
hwsd_s3 = pstdev([d["hwphase_ns"] for d in s3l])

# ソフト計測の読み値ばらつき: 20260703 の 8 boot をプール (boot ごとに平均を引く)。
# S0/S1 の bar を分けない: ソフト補正は周波数を直す仕組みで、計測ジッタは縮めない
# (旧 9834 vs 3083 の差は capture 時の負荷差で、因果ではない)。
import glob as _glob
_soft = []
for _p in sorted(_glob.glob(os.path.join(ROOT, "logs", "20260703-naive-boots", "boot*.log"))):
    _dv = [d["dev_ns"] for d in rows(_p, 5) if "dev_ns" in d]
    _m = sum(_dv) / len(_dv)
    _soft += [v - _m for v in _dv]
softsd = pstdev(_soft)
s2devsd = pstdev([d["dev_ns"] for d in s2 if "dev_ns" in d])

# ============ Figure 1: 1 本の対数の物差しの上で揺れの現在地を見る ============
def _fmt_sigma(v):
    return f"{v/1e3:.1f} µs" if v >= 1000 else f"{v:.0f} ns"

fig, ax = plt.subplots(figsize=(10, 3.9))
ax.set_xscale("log")
ax.set_xlim(3, 1.6e4)
ax.set_ylim(-0.75, 2.05)
ax.axvline(100, color="#2a7d2a", ls="--", lw=1.6, zorder=2)
ax.text(100, 1.95, " 目標 100 ns", color="#2a7d2a", fontsize=11, va="top")
ax.axvspan(3, 100, color="#2a7d2a", alpha=0.05, zorder=1)

ladder_rows = [
    (1.2, "計測の揺れ (GPS-R 1PPS 間隔の読み値)", "タイマー+割り込み", softsd, "PIO", s2devsd),
    (0.0, "位相差の揺れ (出力 vs GPS-R)", "周波数合わせのみ", hwsd_s2, "PLL で閉じる", hwsd_s3),
]
for y, rowname, n_big, v_big, n_small, v_small in ladder_rows:
    ax.text(1.4e4, y + 0.42, rowname, fontsize=11, ha="right", color="#333")
    ax.annotate("", xy=(v_small * 1.25, y), xytext=(v_big / 1.25, y),
                arrowprops=dict(arrowstyle="->", color="#888", lw=1.6), zorder=2)
    ratio = v_big / v_small
    nice = round(ratio / 50) * 50 if ratio >= 100 else round(ratio)
    ax.text((v_big * v_small) ** 0.5, y + 0.09, f"約 1/{nice:,}",
            ha="center", fontsize=10, color="#666")
    for name, v, col in ((n_big, v_big, "#b77"), (n_small, v_small, "#4a7")):
        ax.scatter([v], [y], s=140, color=col, zorder=4)
        ax.text(v, y - 0.18, f"{name}\nσ ≈ {_fmt_sigma(v)}", ha="center", va="top", fontsize=10)

ax.set_xticks([10, 100, 1000, 10000])
ax.set_xticklabels(["10 ns", "100 ns", "1 µs", "10 µs"], fontsize=11)
ax.set_yticks([])
ax.set_xlabel("揺れの大きさ σ (対数目盛。左へ行くほど小さい = よい)")
ax.set_title("計測の揺れと位相差の揺れの現在地")
ax.grid(axis="x", which="major", ls=":", alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig1-ladder.png"), dpi=110)
plt.close(fig)

# ============ Figure 2: S4 before/after (scope) ============
fig, ax = plt.subplots(figsize=(7.5, 4.2))
# stage3 と production 3窓 (scope mean ± std)
xs = [0, 1.5]
means = [131.5, 33.3]
stds = [85.5, 20.9]
cols = ["#c96", "#4a7"]
lab = ["PLL のみ\n(定期校正なし)", "全部入り構成\n(定期校正あり)"]
ax.axhspan(-100, 100, color="#4a7", alpha=0.10, label="≤100 ns 帯")
ax.axhline(0, color="#888", lw=0.8)
ax.errorbar(xs, means, yerr=stds, fmt="o", ms=8, capsize=5, color="none",
            ecolor="#555", elinewidth=1.3)
for x, m, c in zip(xs, means, cols):
    ax.plot(x, m, "o", ms=10, color=c)
    ax.text(x, m+8, f"{m:+.0f}", ha="center", fontsize=9)
ax.set_xticks(xs); ax.set_xticklabels(lab, fontsize=9)
ax.set_ylabel("ピンの上の位相差: 出力 − GPS-R  [ns]")
ax.set_title("定期校正なし/あり (オシロ実測、平均 ± std)")
ax.legend(loc="upper right", fontsize=9)
ax.grid(axis="y", ls=":", alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig2-s4-beforeafter.png"), dpi=110)
plt.close(fig)

# ============ Figure 3: production 引き込みと整定 (loopback 位相 vs count) ============
prod = rows(os.path.join(LOG, "s4", "prod-s5.log"), 0)
# 収束窓に限定 (ロック前の巨大 pre-lock transient を除く)
prodc = [d for d in prod if "hwphase_ns" in d and 50 <= d["count"] <= 230]
cnt = [d["count"] for d in prodc]
hw = [d["hwphase_ns"] for d in prodc]
fig, ax = plt.subplots(figsize=(8.5, 4.0))
ax.plot(cnt, hw, "-o", color="#36a", lw=1.2, ms=2.5)
ax.axhspan(-100, 100, color="#4a7", alpha=0.10, label="≤100 ns 帯")
ax.axhline(0, color="#888", lw=0.8)
ax.set_ylim(-3000, 800)
# overshoot 区間と整定区間を注記
ax.axvspan(85, 115, color="#c33", alpha=0.12)
ax.text(100, 430, "オーバーシュート\n(+300 ns 前後)", ha="center", fontsize=10, color="#a22")
ax.axvspan(180, 230, color="#4a7", alpha=0.10)
ax.text(205, 430, "収束\n(+60 ns 前後、≤100 ns 帯)", ha="center", fontsize=10, color="#272")
ax.set_xlabel("起動からの経過  [秒]"); ax.set_ylabel("loopback 位相  [ns]")
ax.set_title("起動からロックが落ち着くまでの loopback 位相")
ax.grid(ls=":", alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig3-convergence.png"), dpi=110)
plt.close(fig)

# ============ Figure 4: 整数 tick の選び方をばらつかせて平均を非整数に ============
dith = []
for ln in open(os.path.join(LOG, "cold", "S1-b1.log"), errors="replace"):
    m = re.search(r"dith_ticks=(\d+)", ln)
    if m:
        v = int(m.group(1))
        if v > 0:
            dith.append(v)
mean = st.mean(dith)
fig, (ax, bx) = plt.subplots(1, 2, figsize=(10, 4.0), gridspec_kw={"width_ratios": [3, 2]})
# 左: 毎周期に選ばれた整数 tick の時系列 (交互する様子)。base を引いて読みやすく
base = min(dith)
seg = dith[:48]
ax.step(range(len(seg)), [v - base for v in seg], where="mid", color="#36a", lw=1.0)
ax.axhline(mean - base, color="#c33", ls="--", lw=1.2, label=f"平均 {mean:.2f}")
ax.set_xlabel("周期の番号")
ax.set_ylabel(f"選ばれた周期  [tick]  (+{base})")
ax.set_title("毎周期は整数 tick のどれかを選ぶ")
ax.legend(loc="upper right", fontsize=9)
ax.grid(ls=":", alpha=0.4)
# 右: その混合の分布。平均が整数の間 (1000001.75) に来る
c = Counter(dith)
keys = sorted(c)
bx.barh([str(k) for k in keys], [c[k] for k in keys], color="#88b")
bx.axhline((mean - keys[0]), color="#c33", ls="--", lw=1.2)
bx.set_xlabel("出現回数")
bx.set_ylabel("選ばれた周期  [tick]")
bx.set_title("混ぜると平均は整数の間に来る")
bx.grid(ls=":", alpha=0.4, axis="x")
fig.suptitle(f"出せるのは整数 tick だけ。選び方をばらつかせ、平均を整数の間 ({mean:.2f} tick) に置く", fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig4-dither.png"), dpi=110)
plt.close(fig)

# ============ Figure 5: boot 間再現性 (production cold-boot x3) ============
fig, ax = plt.subplots(figsize=(7.0, 4.2))
boots = [1, 2, 3]
sc_mean = [84.2, 72.9, 83.3]
sc_std = [16.3, 11.6, 19.5]
hw_mean = [66.1, 72.5, 80.0]
ax.axhspan(-100, 100, color="#4a7", alpha=0.10, label="≤100 ns 帯")
ax.axhline(0, color="#888", lw=0.8)
ax.errorbar(boots, sc_mean, yerr=sc_std, fmt="o", ms=10, capsize=6,
            color="#4a7", ecolor="#555", elinewidth=1.3, label="scope mean ± std")
ax.plot(boots, hw_mean, "s", ms=7, color="#36a", label="loopback 位相 mean")
for x, m in zip(boots, sc_mean):
    ax.text(x+0.05, m+6, f"{m:+.0f}", fontsize=9, color="#272")
ax.set_xticks(boots); ax.set_xticklabels([f"boot {b}" for b in boots])
ax.set_xlim(0.6, 3.5); ax.set_ylim(-120, 160)
ax.set_ylabel("出力 − GPS / loopback 位相  [ns]")
ax.set_title("再起動 3 回の再現性 (いずれも落ち着いた直後、起動から約 160 秒)")
ax.legend(loc="lower right", fontsize=9)
ax.grid(axis="y", ls=":", alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig5-bootrepro.png"), dpi=110)
plt.close(fig)

# ============ Figure 6: wander の出所 (σ vs 温度 / σ vs 受信) ============
def temp_c(raw): return 27 - ((raw/256)*3.3/4096 - 0.706)/0.001721
pg = []   # (ts, loopback 位相, temp_raw)
gg = []   # (ts, sats)
for ln in open(os.path.join(LOG, "s4", "prod-s5.log"), errors="replace"):
    m = re.match(r"^(\d+\.\d+)\s", ln)
    if not m: continue
    ts = float(m.group(1))
    if "PPSGEN count=" in ln:
        c = re.search(r"count=(\d+)", ln); hw = re.search(r"hwphase_ns=(-?\d+)", ln); tr = re.search(r"temp_raw=(\d+)", ln)
        if c and hw and tr and int(c.group(1)) > 180:
            pg.append((ts, int(hw.group(1)), int(tr.group(1))))
    elif "GGA" in ln and "NMEA" in ln:
        f = ln.split("NMEA ",1)[1].split(",")
        if len(f) > 7:
            try: gg.append((ts, int(f[7])))
            except ValueError: pass
WIN = 120.0; t0 = pg[0][0]
def winmap(rows):
    d = {}
    for r in rows: d.setdefault(int((r[0]-t0)//WIN), []).append(r)
    return d
pw = winmap(pg); gw = winmap(gg)
W_sd=[]; W_tc=[]; W_sat=[]; W_t=[]
for w in sorted(pw):
    hw = [r[1] for r in pw[w]]
    if len(hw) < 5: continue
    W_sd.append(st.pstdev(hw)); W_tc.append(temp_c(st.mean([r[2] for r in pw[w]])))
    W_t.append(w*WIN/60.0)  # 窓の時刻 [分]
    g = gw.get(w, []); W_sat.append(st.mean([x[1] for x in g]) if g else float("nan"))
def corr(xs, ys):
    pts=[(x,y) for x,y in zip(xs,ys) if x==x and y==y]
    if len(pts)<3: return float("nan")
    xs2=[p[0] for p in pts]; ys2=[p[1] for p in pts]; mx=st.mean(xs2); my=st.mean(ys2)
    sxx=sum((x-mx)**2 for x in xs2); syy=sum((y-my)**2 for y in ys2); sxy=sum((x-mx)*(y-my) for x,y in zip(xs2,ys2))
    return sxy/(sxx*syy)**0.5 if sxx>0 and syy>0 else float("nan")
def fitline(xs, ys):
    pts=[(x,y) for x,y in zip(xs,ys) if x==x and y==y]
    xs2=[p[0] for p in pts]; ys2=[p[1] for p in pts]; mx=st.mean(xs2); my=st.mean(ys2)
    sxx=sum((x-mx)**2 for x in xs2); sxy=sum((x-mx)*(y-my) for x,y in zip(xs2,ys2))
    b=sxy/sxx if sxx>0 else 0; a=my-b*mx
    lo,hi=min(xs2),max(xs2); return [lo,hi],[a+b*lo,a+b*hi]
r_t = corr(W_tc, W_sd); r_s = corr(W_sat, W_sd)
fig, (axa, axb) = plt.subplots(1, 2, figsize=(11, 4.3))
# 左: 時系列の twin 軸。σ は 120s 窓だと速い揺れが乗ってギザギザなので、
# 6 分平滑 (3 窓移動平均) の「ゆっくりした動き」を主役にして温度と重ねる。
W_sm = [st.mean(W_sd[max(0, i - 1):i + 2]) for i in range(len(W_sd))]
r_sm = corr(W_tc, W_sm)
axa.plot(W_t, W_sd, "-", color="#e8b0b0", lw=0.9, label="σ (120s 窓ごと)")
axa.plot(W_t, W_sm, "-o", color="#c33", ms=3.5, lw=2.0, label="σ の 6 分平滑")
axa.set_xlabel("経過 [分]"); axa.set_ylabel("loopback 位相 σ [ns]", color="#c33")
axa.tick_params(axis="y", labelcolor="#c33"); axa.grid(ls=":", alpha=0.4)
axa.legend(loc="upper left", fontsize=8)
axt = axa.twinx()
axt.plot(W_t, W_tc, "-", color="#39a", lw=2.0, label="ダイ温度")
axt.set_ylabel("RP2040 ダイ温度 [℃]\n(水晶温度の代理)", color="#39a"); axt.tick_params(axis="y", labelcolor="#39a")
axa.set_title("σ のゆっくりした動きと温度の推移")
# 右: 相関の頑健性チェック。トレンド (単調な上昇) を除くと温度の優位はほぼ消える
def detrend(ts, ys):
    pts=[(x,y) for x,y in zip(ts,ys) if y==y]
    xs2=[p[0] for p in pts]; ys2=[p[1] for p in pts]
    mx=st.mean(xs2); my=st.mean(ys2)
    sxx=sum((x-mx)**2 for x in xs2); sxy=sum((x-mx)*(y-my) for x,y in zip(xs2,ys2))
    b=sxy/sxx if sxx>0 else 0; a=my-b*mx
    return [y-(a+b*x) if y==y else y for x,y in zip(ts,ys)]
W_hd=[]
for w in sorted(pw):
    g = gw.get(w, [])
    if len([r for r in pw[w]]) < 5: continue
    hd=[x[2] for x in g if len(x)>2]
    W_hd.append(st.mean(hd) if hd else float("nan"))
# HDOP は gg に入れていないので NMEA から取り直す
gg2=[]
for ln in open(os.path.join(LOG, "s4", "prod-s5.log"), errors="replace"):
    m = re.match(r"^(\d+\.\d+)\s", ln)
    if not m or "GGA" not in ln or "NMEA" not in ln: continue
    f = ln.split("NMEA ",1)[1].split(",")
    if len(f) > 8:
        try: gg2.append((float(m.group(1)), float(f[8])))
        except ValueError: pass
gw2 = winmap(gg2)
W_hd=[]
for w in sorted(pw):
    if len(pw[w]) < 5: continue
    q = gw2.get(w, [])
    W_hd.append(st.mean([x[1] for x in q]) if q else float("nan"))
r_h = corr(W_hd, W_sd)
sd_d = detrend(W_t, W_sd); tc_d = detrend(W_t, W_tc)
sat_d = detrend(W_t, W_sat); hd_d = detrend(W_t, W_hd)
rd_t = corr(tc_d, sd_d); rd_s = corr(sat_d, sd_d); rd_h = corr(hd_d, sd_d)
import numpy as _np
xpos = _np.arange(3)
axb.bar(xpos - 0.19, [abs(r_t), abs(r_s), abs(r_h)], width=0.38, color="#c9a0a0", label="そのまま")
axb.bar(xpos + 0.19, [abs(rd_t), abs(rd_s), abs(rd_h)], width=0.38, color="#8a3030", label="上昇トレンドを除いた残差")
for x, (r1, r2) in zip(xpos, [(r_t, rd_t), (r_s, rd_s), (r_h, rd_h)]):
    axb.text(x - 0.19, abs(r1) + 0.015, f"{r1:+.2f}", ha="center", fontsize=8.4)
    axb.text(x + 0.19, abs(r2) + 0.015, f"{r2:+.2f}", ha="center", fontsize=8.4)
axb.set_xticks(xpos); axb.set_xticklabels(["温度", "sats (衛星数)", "HDOP"])
axb.set_ylabel("σ との相関の強さ |r| (120s 窓)")
axb.set_ylim(0, 0.62)
axb.set_title("トレンドを除くと、温度の優位はほぼ消える")
axb.legend(fontsize=8.4)
axb.grid(axis="y", ls=":", alpha=0.4)
fig.suptitle("1 時間の連続運転: 単一ログの相関では、遅いふらつきの出所は切り分けられない", fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig6-wander-source.png"), dpi=110)
plt.close(fig)
print(f"fig6: corr(σ,temp)={r_t:+.2f} corr(σ,sats)={r_s:+.2f} windows={len(W_sd)}")

print("metrics: adj-diff σ", {k: round(v,1) for k,v in adj.items()},
      "| loopback 位相 σ S2", round(hwsd_s2,1), "S3", round(hwsd_s3,1),
      "| dith mean", round(st.mean(dith),3), "n", len(dith))
print("wrote 4 figs to", OUT)
