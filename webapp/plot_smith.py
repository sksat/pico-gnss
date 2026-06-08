#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""位相ロックの before/after: 旧(リミットサイクル σ~300ns) vs 新(Smith 予測子 σ~35ns)。

使い方: uv run plot_smith.py <old.log> <new.log> [out.png]
旧=Smith 前の PID (trim が ±60ppb 振動)、新=Smith 予測子+ζ0.71+外れ値3µs (trim 滑らか)。
"""
import re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

for _f in ("Noto Sans CJK JP", "IPAGothic", "TakaoGothic"):
    try:
        matplotlib.rcParams["font.family"] = _f; break
    except Exception:
        pass
matplotlib.rcParams["axes.unicode_minus"] = False

old_p = sys.argv[1] if len(sys.argv) > 1 else "../report/pid-capture.log"
new_p = sys.argv[2] if len(sys.argv) > 2 else "../report/smith-capture.log"
out = sys.argv[3] if len(sys.argv) > 3 else "smith.png"
RE = re.compile(r"hwphase_ns=(-?\d+) trim_ppb=(-?\d+)")


def load(p):
    hw, tr = [], []
    for ln in open(p, encoding="utf-8", errors="replace"):
        if (m := RE.search(ln)):
            hw.append(int(m.group(1))); tr.append(int(m.group(2)))
    return np.array(hw, float), np.array(tr, float)


ohw, otr = load(old_p)
nhw, ntr = load(new_p)
# 収束後 (後半) の定常部を切り出し、グリッチ除外で σ。
def steady(hw):
    s = hw[len(hw) // 2:]
    return s, np.std(s[np.abs(s) < 50000])


os_, osig = steady(ohw); ns_, nsig = steady(nhw)
N = min(220, len(os_), len(ns_))

fig, ax = plt.subplots(2, 1, figsize=(13, 7.5), height_ratios=[2, 1])
fig.suptitle("位相ロックの追い込み: 旧(リミットサイクル) → Smith 予測子で遅延補償 → σ 300ns→35ns",
             fontsize=14, fontweight="bold")

a = ax[0]
a.plot(np.arange(N), os_[-N:], color="#ef4444", lw=0.9, label=f"旧 PID (遅延でリミットサイクル, σ≈{osig:.0f}ns)")
a.plot(np.arange(N), ns_[-N:], color="#10b981", lw=1.0, label=f"新 Smith予測子+ζ0.71 (σ≈{nsig:.0f}ns)")
for v in (300, -300, 50, -50):
    a.axhline(v, color="#ddd", lw=0.5, ls="--")
a.axhline(0, color="#888", lw=0.7)
a.set_ylim(-900, 900)
a.set_title(f"A. 出力 PPS の UTC 秒位相: 旧は ±500ns 振動 / 新は ±50ns に貼り付き ({osig:.0f}→{nsig:.0f}ns = {osig/nsig:.0f}×)", fontsize=10.5)
a.set_ylabel("UTC 秒からのズレ [ns]"); a.legend(fontsize=9.5, loc="upper right"); a.grid(True, alpha=0.2)

a = ax[1]
a.plot(np.arange(N), otr[len(otr) // 2:][-N:] if len(otr) > N else otr[-N:], color="#ef4444", lw=0.9, label="旧: trim が ±60ppb 振動 (I が hunting)")
a.plot(np.arange(N), ntr[len(ntr) // 2:][-N:] if len(ntr) > N else ntr[-N:], color="#10b981", lw=1.0, label="新: trim 滑らかに整定 (減衰)")
a.axhline(0, color="#888", lw=0.6)
a.set_title("B. 周波数トリム trim_ppb: 旧は I 項が暴れる / 新は静かに正解周波数へ整定", fontsize=10.5)
a.set_xlabel("エッジ (定常部)"); a.set_ylabel("trim [ppb]"); a.legend(fontsize=9.5); a.grid(True, alpha=0.2)

fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(out, dpi=110)
print(f"saved {out}  (old σ={osig:.0f}ns, new σ={nsig:.0f}ns)")
