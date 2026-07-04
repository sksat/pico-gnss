#!/usr/bin/env python3
"""4 条件の実効 K 誤差 (= オシロ実測 − 内部 loopback 位相) の時間変化。fig-recal-walk と同じ量。
内部位相を引くと遅いふらつき (両方に共通) が消え、K のズレだけが残る。
左: 定期校正なし。修正前 (stage-3 20260703) と修正後 (stage-3 fix 20260704) を 0 起点で重ねる。
    どちらも歩く = 修正は歩きを止めない。
右: 定期校正あり。修正前 (s5-dense) +40ns と修正後 (fix-dense) +10ns。修正で中心が下がる。
オシロ (絶対時刻) と内部位相 (相対時刻) の整合は gap の分散最小で自動決定。"""
import os
import re
import time
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
LOGS = os.path.dirname(HERE)
OUT = os.path.join(LOGS, "..", "docs", "report", "precision-ladder", "precision-figs")
RECAL = os.path.join(LOGS, "20260703-recal-scope")
NOREC = os.path.join(LOGS, "20260704-fix-norecal")
OLDNOREC = os.path.join(LOGS, "20260704-old-norecal")


def wall(hms):
    return time.mktime(time.strptime(f"2026-07-03 {hms}", "%Y-%m-%d %H:%M:%S"))


def load_rtt(path, lim=500):
    out = []
    for ln in open(path, errors="replace"):
        if "PPSGEN count=" not in ln:
            continue
        m = re.match(r"^(\d+\.\d+)\s", ln)
        if not m:
            continue
        d = dict(re.findall(r"(\w+)=(-?\d+)", ln))
        if "hwphase_ns" not in d:
            continue
        h = int(d["hwphase_ns"])
        if abs(h) < lim:
            out.append((float(m.group(1)), h))
    return out


def shots(path):
    return [(float(a), float(b)) for a, b in (ln.split() for ln in open(path))]


def gap_dense(scope, rtt_w, tol=2.5):
    gx, gy, j = [], [], 0
    for t, v in scope:
        while j + 1 < len(rtt_w) and abs(rtt_w[j + 1][0] - t) <= abs(rtt_w[j][0] - t):
            j += 1
        if abs(rtt_w[j][0] - t) <= tol:
            gx.append(t); gy.append(v - rtt_w[j][1])
    return gx, gy


def gap_auto(scope, rtt_rel):
    """オシロ (絶対時刻) と内部位相 (相対時刻) の整合を gap の分散最小で決める。"""
    best = None
    for k in range(0, 900, 2):
        bw = scope[0][0] - k
        gx, gy = gap_dense(scope, [(bw + t, h) for t, h in rtt_rel])
        if len(gy) < 400:
            continue
        v = st.pvariance(gy)
        if best is None or v < best[0]:
            best = (v, gx, gy)
    return best[1], best[2]


def slope(xs, ys):
    n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.3))

# ================= 左: 定期校正なし。修正前 (旧 fw) と修正後 (fix)、両方 dense =================
def walk(dirpath, densefile):
    gx0, gy0 = gap_auto(shots(os.path.join(dirpath, densefile)), load_rtt(os.path.join(dirpath, "rtt.log")))
    med = st.median(gy0)
    kp = [i for i in range(len(gy0)) if abs(gy0[i] - med) < 300]  # 不良 shot 除外
    gx = [gx0[i] for i in kp]; gy = [gy0[i] for i in kp]
    x = [(t - gx[0]) / 60 for t in gx]
    y = [g - sum(gy[:60]) / 60 for g in gy]  # 0 起点
    return x, y


def runmean(y, w=90):
    return [sum(y[max(0, i - w + 1):i + 1]) / (i + 1 - max(0, i - w + 1)) for i in range(len(y))]


xb, yb = walk(OLDNOREC, "oldnorecal-dense.shots")  # 修正前 (旧 fw, stage-3)
xa, ya = walk(NOREC, "fixnorecal-dense.shots")     # 修正後 (fix, stage-3)
slb, sla = slope(xb, yb), slope(xa, ya)
ax1.plot(xb, yb, ".", color="#c05050", ms=1.5, alpha=0.2)
ax1.plot(xa, ya, ".", color="#1a7a1a", ms=1.5, alpha=0.2)
ax1.plot(xb, runmean(yb), "-", color="#c05050", lw=2.0, label=f"修正前 (約 {slb:+.0f} ns/min)")
ax1.plot(xa, runmean(ya), "-", color="#1a7a1a", lw=2.0, label=f"修正後 (約 {sla:+.0f} ns/min)")
ax1.set_title("定期校正なし (修正前も修正後も同じ速さで歩く)")
ax1.set_xlabel("計測開始からの経過 [min]")
ax1.set_ylabel("ピンの上のずれ − 内部位相  [開始からの変化, ns]")
ax1.set_ylim(-30, 200)
ax1.grid(ls=":", alpha=0.4)
ax1.legend(loc="upper left", fontsize=9)

# ================= 右: 定期校正あり。修正前 (s5) と修正後 (fix) =================
BOOT5 = wall("10:57:30")
rtt5_w = [(BOOT5 + t, h) for t, h in load_rtt(os.path.join(RECAL, "stage5-rtt.log"))]
gx5, gy5 = gap_dense(shots(os.path.join(RECAL, "s5-dense.shots")), rtt5_w)
x5 = [(t - gx5[0]) / 60 for t in gx5]
gxf, gyf = gap_auto(shots(os.path.join(HERE, "fix-dense.shots")), load_rtt(os.path.join(HERE, "rtt.log")))
xf = [(t - gxf[0]) / 60 for t in gxf]
m5, mf = st.mean(gy5), st.mean(gyf)
ax2.axhline(0, color="#ccc", lw=0.8)
ax2.plot(x5, gy5, ".", color="#888", ms=2, alpha=0.5)
ax2.plot(xf, gyf, ".", color="#2ca02c", ms=2, alpha=0.5)
ax2.axhline(m5, color="#555", lw=1.6, label=f"修正前 (中心 {m5:+.0f} ns)")
ax2.axhline(mf, color="#1a7a1a", lw=1.6, label=f"修正後 (中心 {mf:+.0f} ns)")
xa = max(x5[-1], xf[-1]) + 2.5
ax2.annotate("", xy=(xa, mf), xytext=(xa, m5), arrowprops=dict(arrowstyle="->", color="#1a7a1a", lw=1.6))
ax2.text(xa + 1.2, (m5 + mf) / 2, f"{mf - m5:+.0f} ns\n取りこぼしを直した分", color="#1a7a1a", fontsize=9.5, va="center")
ax2.set_title("定期校正あり (平坦。修正で中心が下がる)")
ax2.set_xlabel("経過 [min]")
ax2.set_ylabel("ピンの上のずれ − 内部位相  [ns]")
ax2.set_ylim(-40, 90)
ax2.grid(ls=":", alpha=0.4)
ax2.legend(loc="upper right", fontsize=9)

fig.tight_layout()
fig.savefig(os.path.join(OUT, "fig-fix-result.png"), dpi=110)
print(f"校正なし 修正前 {slb:+.1f} / 修正後 {sla:+.1f} ns/min ; 校正あり 修正前 {m5:+.0f} / 修正後 {mf:+.0f} ns")
