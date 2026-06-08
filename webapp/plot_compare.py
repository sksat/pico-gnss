#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""stage② の効果: 旧(Instant)制御 と 新(PIOハード)制御 のログを比較プロットする。

使い方 (uv で隔離環境を自動構築):
    uv run plot_compare.py <old.log> <new.log> [out.png]

PPSGEN 行の phase_ns(旧=Instant 測定) と hwphase_ns(新=PIO ハード測定) を使う。
A: 出力 PPS の UTC 位相 (旧制御=±ms vs 新制御=±ns) — 制御結果。
B: 位相の「測定」精度 (Instant vs PIO、同じ出力を両方で測った) — 改善の源泉。
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

old_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/old.log"
new_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/new.log"
out = sys.argv[3] if len(sys.argv) > 3 else "compare.png"
import os
_here = os.path.dirname(os.path.abspath(__file__))
exp_path = sys.argv[4] if len(sys.argv) > 4 else os.path.join(_here, "../report/exp-capture.log")

RE = re.compile(r"PPSGEN count=\d+ interval_ns=\d+ dev_ns=-?\d+ phase_ns=(-?\d+) hwphase_ns=(-?\d+)")
TS = re.compile(r"(?:^|\s)(\d+\.\d+) \[")


def parse(path):
    t, ph, hw = [], [], []
    base = None
    for ln in open(path, encoding="utf-8", errors="replace"):
        m = RE.search(ln)
        if not m:
            continue
        tm = TS.search(ln)
        # server ログには defmt timestamp が無い場合があるので index を時間代わりに
        ti = float(tm.group(1)) if tm else len(t)
        if base is None:
            base = ti
        t.append(ti - base)
        ph.append(int(m.group(1)))
        hw.append(int(m.group(2)))
    return np.array(t, float), np.array(ph, float), np.array(hw, float)


ot, oph, ohw = parse(old_path)
nt, nph, nhw = parse(new_path)

fig, ax = plt.subplots(1, 2, figsize=(15, 5.5))
fig.suptitle("stage②: 規律 PPS 出力の UTC 位相同期 — 制御方式・制御項(A) と 測定精度(B) の比較",
             fontsize=13, fontweight="bold")


# 制御項 P/PI/PID のオフライン sim (実機実験ログ exp から外乱を取り A に重ねる)。
def sim_terms(exp):
    ER = re.compile(r"hwphase_ns=(-?\d+) trim_ppb=(-?\d+) cfg=\d+ p_ns=(-?\d+) d_ns=(-?\d+)")
    hw, tr, pn, dn = [], [], [], []
    for ln in open(exp, encoding="utf-8", errors="replace"):
        if (mm := ER.search(ln)):
            hw.append(int(mm.group(1))); tr.append(int(mm.group(2)))
            pn.append(int(mm.group(3))); dn.append(int(mm.group(4)))
    if len(hw) < 50:
        return None
    hw = np.array(hw, float); ce = np.array(tr, float) - np.array(pn, float) - np.array(dn, float)
    dphi = np.diff(hw); Dl = 2
    dist0 = dphi[Dl:] - ce[:-1 - Dl]
    drift = np.median(dist0[np.abs(dist0) < 5000])
    floor = np.std(dist0[np.abs(dist0 - drift) < 150])
    rng = np.random.default_rng(0)
    M = 400
    w = np.zeros(M)  # AR1 相関ノイズ (白色だと実機より過大に揺れるため; 実機は隣接16ns/edge と滑らか)
    for i in range(1, M):
        w[i] = 0.9 * w[i - 1] + rng.standard_normal() * floor * 0.44
    dist = drift + w

    def run(kp, ki, kd, db=500, init=2_400_000.0):  # 実測の取得開始 (~2.4ms) に初期値を合わせる
        p = np.full(M, init); ts = 0.0; lk = 0; last = init; eff = np.zeros(M)
        for n in range(M - 1):
            c = p[n]; locked = lk >= 5
            pp = (c / kp) if abs(c) > db else 0.0
            if ki and locked:
                ts = max(-3000, min(3000, ts - c / ki))
            elif not ki:
                ts = 0.0
            dd = ((c - last) / kd) if (kd and locked) else 0.0
            last = c; eff[n] = ts - pp - dd
            lk = min(lk + 1, 5) if abs(c) < 5000 else 0
            p[n + 1] = p[n] + (eff[n - Dl] if n >= Dl else 0.0) + dist[n]
        return p
    return {"P のみ": run(16, 0, 0), "PI": run(16, 128, 0), "PID": run(16, 128, 4)}

GLITCH = 3_000_000  # |hw|>3ms = PIO 周回グリッチ等の外れ値 → 表示・σ から除外


def locked_sigma(hw, bound):  # ロック確立後 (末尾 1/4) で |hw|<bound の σ
    s = hw[len(hw) * 3 // 4:]
    s = s[np.abs(s) < bound]
    return np.std(s) if len(s) > 2 else float("nan")


def clip(t, hw):  # グリッチ (>3ms) を表示から除外
    m = np.abs(hw) < GLITCH
    return t[m], hw[m]


# A: 出力位相 (PIO 真値 hwphase) 旧 vs 新。symlog で ms〜ns を 1 枚に。
a = ax[0]
if len(ohw):
    ct, ch = clip(ot, ohw)
    a.plot(ct, ch, ".-", color="#a78bfa", ms=2, lw=0.6, label=f"旧: Instant 制御 (ロック σ≈{locked_sigma(ohw, GLITCH)/1000:.0f}µs)")
if len(nhw):
    ct, ch = clip(nt, nhw)
    a.plot(ct, ch, ".-", color="#10b981", ms=2.5, lw=0.7, label=f"新: PIO ハード制御=PID (ロック後 σ≈{locked_sigma(nhw, 50_000):.0f}ns = sub-µs)")
# 制御項の効果を同じ軸に重ねる (オフライン sim, 初期2µsオフセットから): P=ドループ / PI=振動 / PID=減衰。
terms = None
try:
    terms = sim_terms(exp_path)
except Exception as ex:
    print(f"terms skip: {ex}")
if terms:
    # 新(実測)=PID なので、sim は参照として P(I無し=ドループ) と PI(D無し=振動) を重ねる。
    styles = {"P のみ": ("#ef4444", ":", "sim P のみ(I無し→ドループ)"),
              "PI": ("#f59e0b", "--", "sim PI(D無し→振動)")}
    for name, (col, ls, lab) in styles.items():
        p = terms[name]; s = p[len(p) // 2:]; sg = np.std(s[np.abs(s) < 50000])
        a.plot(np.arange(len(p)), p, ls, color=col, lw=1.1, alpha=0.9, label=f"{lab} σ{sg:.0f}ns")
a.set_yscale("symlog", linthresh=1000)
for v in (1e6, -1e6, 1e3, -1e3):
    a.axhline(v, color="#ccc", lw=0.5, ls="--")
a.axhline(0, color="#888", lw=0.7)
a.set_title("A. 出力位相 (UTC秒からのズレ, symlog): 旧Instant=±ms / 新PIO=sub-µs + 制御項 P/PI/PID の効果", fontsize=9.5)
a.set_xlabel("経過時間[s] (実測) / エッジ (sim)"); a.set_ylabel("UTC 秒境界からのズレ [ns]")
a.legend(fontsize=7.5, loc="upper right"); a.grid(True, which="both", alpha=0.15)

# B: 測定精度. 旧 run で同じ出力を Instant と PIO で測った差 = Instant の測定ノイズ。
a = ax[1]
if len(oph):
    diff = oph - ohw
    diff = diff[np.abs(diff) < 1_000_000]  # 秒境界跨ぎの偽値を除外
    a.plot(diff, ".", color="#ef4444", ms=3, alpha=0.6, label=f"Instant 測定 − PIO 真値 (σ={np.std(diff)/1000:.0f}µs)")
    a.axhline(0, color="#10b981", lw=1.4, label="PIO ハード測定 (真値=0基準, 16ns 刻み)")
    lim = max(2000, np.percentile(np.abs(diff), 98) * 1.3)
    a.set_ylim(-lim, lim)
a.set_title("B. 位相の『測定』精度: Instant は ±数百µs ばらつく / PIO は 16ns", fontsize=10.5)
a.set_xlabel("PPS パルス番号 (旧 run)"); a.set_ylabel("測定誤差 [ns] (PIO 真値との差)")
a.legend(fontsize=9); a.grid(True, alpha=0.2)


fig.tight_layout(rect=[0, 0, 1, 0.95])
fig.savefig(out, dpi=110)
print(f"saved {out}  (old {len(ohw)} pts, new {len(nhw)} pts)")
