#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""位相ロックの制御項 (P / PI / PID) の効果を比較する。

PHASE_EXPERIMENT=true の firmware は cfg を 0=P,1=PI,2=PID と ~120 エッジ毎に巡回し、
PPSGEN 行に cfg/hwphase/trim/p_ns/d_ns を出す。それを cfg 区間で分けて:
A: 位相 (hwphase) の時系列を cfg で背景色分け — 各項で挙動がどう変わるか。
B: trim_ppb (I 項の周波数トリム) — P では 0、PI/PID で残差周波数を吸収。
C: cfg 別の整定統計 (mean=オフセット, σ=ばらつき) を棒で比較。

使い方: uv run plot_terms.py <exp.log> [out.png]
"""
import re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

for _f in ("Noto Sans CJK JP", "IPAGothic", "TakaoGothic"):
    try:
        matplotlib.rcParams["font.family"] = _f
        break
    except Exception:
        pass
matplotlib.rcParams["axes.unicode_minus"] = False

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/exp.log"
out = sys.argv[2] if len(sys.argv) > 2 else "terms.png"

RE = re.compile(
    r"PPSGEN count=(\d+) interval_ns=\d+ dev_ns=(-?\d+) phase_ns=-?\d+ "
    r"hwphase_ns=(-?\d+) trim_ppb=(-?\d+) cfg=(\d+) p_ns=(-?\d+) d_ns=(-?\d+)"
)
NAMES = {0: "P のみ", 1: "PI", 2: "PID"}
COLORS = {0: "#ef4444", 1: "#3b82f6", 2: "#10b981"}

cnt, dev, hw, trim, cfg, pn, dn = [], [], [], [], [], [], []
for ln in open(path, encoding="utf-8", errors="replace"):
    if (m := RE.search(ln)):
        g = m.groups()
        cnt.append(int(g[0])); dev.append(int(g[1])); hw.append(int(g[2]))
        trim.append(int(g[3])); cfg.append(int(g[4])); pn.append(int(g[5])); dn.append(int(g[6]))
hw = np.array(hw, float); trim = np.array(trim, float); cfg = np.array(cfg, int)
x = np.arange(len(hw))
print(f"parsed {len(hw)} PPSGEN, cfg blocks: {np.unique(cfg)}")

# cfg が変わる境界でブロック分割
bnd = [0] + [i for i in range(1, len(cfg)) if cfg[i] != cfg[i - 1]] + [len(cfg)]
blocks = [(bnd[i], bnd[i + 1], cfg[bnd[i]]) for i in range(len(bnd) - 1)]

fig, ax = plt.subplots(3, 1, figsize=(13, 10), height_ratios=[3, 1.5, 2])
fig.suptitle("位相ロックの制御項ごとの効果: P → PI → PID (巡回実験)", fontsize=14, fontweight="bold")

# A: 位相時系列 + cfg 背景。グリッチ (|hw|>50µs) はクリップ。
a = ax[0]
for s, e, c in blocks:
    a.axvspan(s, e, color=COLORS[c], alpha=0.08)
    a.text((s + e) / 2, 0.96, NAMES[c], transform=a.get_xaxis_transform(),
           ha="center", va="top", fontsize=8, color=COLORS[c], fontweight="bold")
hwc = np.clip(hw, -50000, 50000)
a.plot(x, hwc, "-", color="#334155", lw=0.6)
a.axhline(0, color="#888", lw=0.8)
for v in (500, -500):
    a.axhline(v, color="#bbb", lw=0.5, ls="--")
a.set_ylim(-8000, 8000)
a.set_title("A. 出力位相 (UTC 秒からのズレ) — P=オフセット残る / PI=0中心だが振動 / PID=振動も減衰", fontsize=10)
a.set_ylabel("位相 [ns]"); a.grid(True, alpha=0.15)

# B: trim_ppb
a = ax[1]
for s, e, c in blocks:
    a.axvspan(s, e, color=COLORS[c], alpha=0.08)
a.plot(x, trim, "-", color="#a855f7", lw=0.8)
a.axhline(0, color="#888", lw=0.6)
a.set_title("B. I 項の周波数トリム trim_ppb — P では 0 (→ドループ)、PI/PID で残差を吸収", fontsize=10)
a.set_ylabel("trim [ppb]"); a.set_xlabel("PPS パルス番号"); a.grid(True, alpha=0.15)

# C: cfg 別の整定統計 (各ブロックの後半 1/2 で集計、|hw|<50µs)
a = ax[2]
stats = {}
for s, e, c in blocks:
    seg = hw[s + (e - s) // 2:e]
    seg = seg[np.abs(seg) < 50000]
    if len(seg) > 3:
        stats.setdefault(c, []).append((np.mean(seg), np.std(seg)))
labels, means, sigmas = [], [], []
for c in (0, 1, 2):
    if c in stats:
        mm = np.mean([s[0] for s in stats[c]]); ss = np.mean([s[1] for s in stats[c]])
        labels.append(NAMES[c]); means.append(mm); sigmas.append(ss)
xp = np.arange(len(labels)); w = 0.35
a.bar(xp - w / 2, np.abs(means), w, color="#f59e0b", label="|平均オフセット| [ns]")
a.bar(xp + w / 2, sigmas, w, color="#06b6d4", label="σ ばらつき [ns]")
for i, (mm, ss) in enumerate(zip(means, sigmas)):
    a.text(xp[i] - w / 2, abs(mm), f"{abs(mm):.0f}", ha="center", va="bottom", fontsize=9)
    a.text(xp[i] + w / 2, ss, f"{ss:.0f}", ha="center", va="bottom", fontsize=9)
a.set_xticks(xp); a.set_xticklabels(labels, fontsize=11)
a.set_title("C. 整定品質 (各ブロック後半で集計): オフセット と σ", fontsize=11)
a.set_ylabel("[ns]"); a.legend(fontsize=9); a.grid(True, axis="y", alpha=0.2)

fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(out, dpi=110)
print(f"saved {out}")
for c in (0, 1, 2):
    if c in stats:
        mm = np.mean([s[0] for s in stats[c]]); ss = np.mean([s[1] for s in stats[c]])
        print(f"  {NAMES[c]:6s}: offset={mm:+.0f}ns  σ={ss:.0f}ns  ({len(stats[c])} blocks)")
