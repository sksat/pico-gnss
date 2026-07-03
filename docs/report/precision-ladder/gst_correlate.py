# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "matplotlib"]
# ///
"""GST(擬似距離の誤差統計)と出力位相の追従誤差 hwphase の相関を見る。

狙い: 受信機が毎秒出す測定品質(GST の擬似距離残差 RMS・位置 σ)が、出力位相のふらつき
を予測できるか。できるなら、その品質に応じてループのゲイン/採否を変える(GST 適応)余地がある。

defmt ログ(probe-rs run の出力)から時刻(行頭の boot 秒)で対応付ける:
  PPSGEN ... hwphase_ns=<x> ... lk=<l>          毎秒、出力 vs GPS の位相誤差
  NMEA $G?GST,utc,rms,smjr,smnr,orient,latstd,lonstd,altstd   毎秒、測定品質

usage: uv run gst_correlate.py <log> [out.png]
"""
import re
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
for _f in ("Noto Sans CJK JP", "IPAGothic", "TakaoGothic"):
    try:
        matplotlib.rcParams["font.family"] = _f
        break
    except Exception:
        pass
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pps-flash.log"
out = sys.argv[2] if len(sys.argv) > 2 else "gst-correlation.png"

re_pps = re.compile(r"^(\d+\.\d+).*PPSGEN .*hwphase_ns=(-?\d+).*lk=(\d)")
re_gst = re.compile(
    r"^(\d+\.\d+).*\$G.GST,[\d.]*,([\d.]+),[\d.]+,[\d.]+,[\d.]+,([\d.]+),([\d.]+),([\d.]+)"
)

pt, hw, lk = [], [], []
gt, rms, latstd, lonstd = [], [], [], []
for ln in open(path, encoding="utf-8", errors="replace"):
    if (m := re_pps.search(ln)):
        pt.append(float(m.group(1)))
        hw.append(int(m.group(2)))
        lk.append(int(m.group(3)))
    elif (m := re_gst.search(ln)):
        gt.append(float(m.group(1)))
        rms.append(float(m.group(2)))
        latstd.append(float(m.group(3)))
        lonstd.append(float(m.group(4)))

pt = np.array(pt); hw = np.array(hw, float); lk = np.array(lk)
gt = np.array(gt); rms = np.array(rms)
hpos = np.hypot(np.array(latstd), np.array(lonstd))  # 水平位置 σ [m]
print(f"parsed {len(pt)} PPSGEN, {len(gt)} GST")

# rolling std of hwphase = local 揺れの大きさ (15s 窓)
W = 15
hwstd = np.array([hw[max(0, i - W):i + 1].std() for i in range(len(hw))])

# 各 PPSGEN に最も近い GST を対応付け (両者 ~1Hz)
idx = np.clip(np.searchsorted(gt, pt), 1, len(gt) - 1)
pick = np.where(np.abs(pt - gt[idx - 1]) <= np.abs(pt - gt[idx]), idx - 1, idx)
# 引き込み(pull-in)直後は rolling-σ が過渡で巨大になるので、最初のロックから SETTLE 秒は除外して
# 定常だけを相関に使う。
SETTLE = 180.0
t_lock0 = pt[lk == 1][0] if (lk == 1).any() else pt[0]
ok = (np.abs(pt - gt[pick]) < 0.6) & (lk == 1) & (pt - t_lock0 > SETTLE)
t = pt[ok] - pt[ok][0]
HW = hw[ok]; HWSTD = hwstd[ok]; RMS = rms[pick][ok]; HPOS = hpos[pick][ok]
print(f"paired {ok.sum()} locked epochs (dt<0.6s)")


def pearson(a, b):
    if len(a) < 3 or a.std() == 0 or b.std() == 0:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


r_abs = pearson(RMS, np.abs(HW))
r_std = pearson(RMS, HWSTD)
r_pos = pearson(HPOS, HWSTD)
print(f"RMS vs |hwphase|     : r = {r_abs:+.2f}")
print(f"RMS vs hwphase 15s-σ : r = {r_std:+.2f}")
print(f"水平位置σ vs hwphase 15s-σ : r = {r_pos:+.2f}")
print(f"GST RMS: mean {RMS.mean():.1f} m, range {RMS.min():.1f}–{RMS.max():.1f} m")
print(f"hwphase 15s-σ: mean {HWSTD.mean():.0f} ns, range {HWSTD.min():.0f}–{HWSTD.max():.0f} ns")

fig, ax = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle("GST(測定品質) と出力位相追従誤差 hwphase の相関", fontsize=13, fontweight="bold")

a = ax[0]
a.plot(t, HW, lw=0.7, color="#2563eb", label="hwphase [ns] (出力 vs GPS)")
a.plot(t, HWSTD, lw=1.2, color="#1e3a8a", label=f"hwphase 15s-σ [ns]")
a.set_xlabel("経過 [s]"); a.set_ylabel("位相 [ns]", color="#1e3a8a")
a.grid(True, alpha=0.2); a.axhline(0, color="#888", lw=0.5)
b = a.twinx()
b.plot(t, RMS, lw=1.0, color="#dc2626", alpha=0.8, label="GST 擬似距離残差 RMS [m]")
b.set_ylabel("GST RMS [m]", color="#dc2626")
a.set_title("時系列: 追従誤差の揺れと測定品質", fontsize=10)
h1, l1 = a.get_legend_handles_labels(); h2, l2 = b.get_legend_handles_labels()
a.legend(h1 + h2, l1 + l2, fontsize=8, loc="upper left")

a = ax[1]
a.scatter(RMS, HWSTD, s=10, color="#2563eb", alpha=0.4)
a.set_xlabel("GST 擬似距離残差 RMS [m]"); a.set_ylabel("hwphase 15s-σ [ns]")
a.set_title(f"測定品質 vs 追従誤差の揺れ (r={r_std:+.2f})", fontsize=10)
a.grid(True, alpha=0.2)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(out, dpi=110)
print(f"saved {out}")
