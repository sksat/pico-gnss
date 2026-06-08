#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""制御構成 P / PD / PI / PID / PID+Smith を**実機で**比較する (同じ信号・1 回の走行)。

PHASE_EXPERIMENT=true の firmware は cfg を 0=P,1=PD,2=PI,3=PID,4=PID+Smith と ~130 エッジ毎に
巡回し、PPSGEN 行に cfg/hwphase/trim を出す。それを cfg 区間で分けて挙動と整定品質を見る。

使い方: uv run plot_ctrl.py <ctrl5.log> [out.png]
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

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ctrl5.log"
out = sys.argv[2] if len(sys.argv) > 2 else "ctrl.png"
RE = re.compile(r"hwphase_ns=(-?\d+) trim_ppb=(-?\d+) cfg=(\d+)")
NAMES = {0: "P", 1: "PD", 2: "PI", 3: "PID", 4: "PID+Smith"}
COLS = {0: "#ef4444", 1: "#f59e0b", 2: "#a855f7", 3: "#3b82f6", 4: "#10b981"}

hw, tr, cfg = [], [], []
for ln in open(path, encoding="utf-8", errors="replace"):
    if (m := RE.search(ln)):
        hw.append(int(m.group(1))); tr.append(int(m.group(2))); cfg.append(int(m.group(3)))
hw = np.array(hw, float); tr = np.array(tr, float); cfg = np.array(cfg, int)
# 第1サイクル (PID+Smith→P→PD→PI→PID, ~660 エッジ) に限定。2 周目以降は弱信号スパイク期に当たり汚染。
LIM = int(sys.argv[3]) if len(sys.argv) > 3 else 660
hw, tr, cfg = hw[:LIM], tr[:LIM], cfg[:LIM]
x = np.arange(len(hw))
print(f"parsed {len(hw)} (limit {LIM}), cfg: {sorted(set(cfg.tolist()))}")

# cfg が変わる境界でブロック分割
bnd = [0] + [i for i in range(1, len(cfg)) if cfg[i] != cfg[i - 1]] + [len(cfg)]
blocks = [(bnd[i], bnd[i + 1], int(cfg[bnd[i]])) for i in range(len(bnd) - 1)]

fig, ax = plt.subplots(1, 1, figsize=(13, 5.5))
fig.suptitle("制御構成の実機比較: P / PD / PI / PID / PID+Smith (同じ信号・1 回の走行)",
             fontsize=13, fontweight="bold")

# 位相時系列 + cfg 背景。グリッチ(|hw|>50µs)はクリップ。
a = ax
for s, e, c in blocks:
    a.axvspan(s, e, color=COLS[c], alpha=0.08)
    if e - s > 30:
        a.text((s + e) / 2, 0.97, NAMES[c], transform=a.get_xaxis_transform(),
               ha="center", va="top", fontsize=8.5, color=COLS[c], fontweight="bold")
a.plot(x, np.clip(hw, -3000, 3000), "-", color="#334155", lw=0.6)
a.axhline(0, color="#888", lw=0.8)
for v in (500, -500):
    a.axhline(v, color="#ccc", lw=0.5, ls="--")
a.set_ylim(-2500, 2500)
a.set_title("出力位相 (UTC秒からのズレ): P/PD=ドループ残(I無し) / PI・PID=0中心(I が消す) / PID+Smith=±50ns。"
            "スパイクは弱信号の PPS 欠落 (outlier reject で保持→復帰)。σ の精密比較は tune.png/smith.png 参照",
            fontsize=8.8)
a.set_ylabel("位相 [ns]"); a.set_xlabel("PPS パルス番号"); a.grid(True, alpha=0.15)

fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(out, dpi=110)
print(f"saved {out}")
