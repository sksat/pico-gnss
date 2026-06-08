#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""制御構成 P / PD / PI / PID / PID+Smith を**実機で公平に**比較する。

各構成を**別々の走行でコールドスタート(~2.4ms)から**撮り、起動からのエッジで重ねる (compare と同じ流儀。
巡回だと前構成の終状態を引き継いで初期条件が揃わないため)。CTRL_SEL=0..4 でビルドした 5 ログを渡す。

使い方: uv run plot_ctrl.py [ctrl_0.log ctrl_1.log ... ctrl_4.log] [out.png]
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

NAMES = ["P", "PD", "PI", "PID", "PID+Smith"]
COLS = ["#ef4444", "#f59e0b", "#a855f7", "#3b82f6", "#10b981"]
args = sys.argv[1:]
out = "ctrl.png"
if args and args[-1].endswith(".png"):
    out = args[-1]; args = args[:-1]
logs = args if len(args) == 5 else [f"/tmp/ctrl_{i}.log" for i in range(5)]
RE = re.compile(r"hwphase_ns=(-?\d+)")


def load(p):
    hw = []
    try:
        for ln in open(p, encoding="utf-8", errors="replace"):
            if (m := RE.search(ln)):
                hw.append(int(m.group(1)))
    except FileNotFoundError:
        return np.array([])
    return np.array(hw, float)


def locked_sigma(hw):  # 後半 1/3 で |hw|<50µs (グリッチ除外) の σ
    s = hw[len(hw) * 2 // 3:]
    s = s[np.abs(s) < 50000]
    return np.std(s) if len(s) > 5 else float("nan")


fig, a = plt.subplots(1, 1, figsize=(13, 6))
fig.suptitle("制御構成の実機比較 (各構成をコールドスタートから個別に撮り重ねた・公平な初期条件)",
             fontsize=13, fontweight="bold")
for i, p in enumerate(logs):
    hw = load(p)
    if not len(hw):
        print(f"  {NAMES[i]}: no data ({p})"); continue
    hwc = np.where(np.abs(hw) < 3_000_000, hw, np.nan)  # グリッチ(>3ms)は線を切る
    sg = locked_sigma(hw)
    lw = 1.4 if i == 4 else 0.9
    a.plot(np.arange(len(hwc)), hwc, "-", color=COLS[i], lw=lw,
           label=f"{NAMES[i]} (整定 σ≈{sg:.0f}ns)")
    print(f"  {NAMES[i]:10s}: n={len(hw)} σ≈{sg:.0f}ns")
a.set_yscale("symlog", linthresh=100)
for v in (1e6, 1e3, 1e2, -1e2, -1e3, -1e6):
    a.axhline(v, color="#eee", lw=0.5)
a.axhline(0, color="#888", lw=0.8)
a.set_title("出力位相 (symlog): 全構成 2.4ms から → PID+Smith(Kp1/8) が最速で σ~38ns にロック。"
            "他は Kp1/16(安定上限,1/8 は PI が発散)で緩慢=Smith が高ゲインを可能に。定常の構造比較は tune.png",
            fontsize=8.6)
a.set_xlabel("起動からのエッジ (≈秒)"); a.set_ylabel("UTC 秒境界からのズレ [ns]")
a.legend(fontsize=9.5, loc="upper right"); a.grid(True, which="both", alpha=0.12)
fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(out, dpi=110)
print(f"saved {out}")
