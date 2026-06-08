#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""pico-gnss のログから評価レポート用の図を生成する (matplotlib)。

使い方 (uv で隔離環境を自動構築):
    uv run plot_report.py <logfile> [out.png]

各「手法が効いている様子」を、横軸=起動からの実時間 [s]、タイトル=結論、で 4 枚に。
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

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/report.log"
out = sys.argv[2] if len(sys.argv) > 2 else "report.png"
log = open(path, encoding="utf-8", errors="replace").read().splitlines()


def snap(raw):
    secs = (raw + (1 if raw >= 0 else -1) * 500_000_000) // 1_000_000_000
    return raw - secs * 1_000_000_000


def tof(ln):  # 行頭の defmt タイムスタンプ = 起動からの秒
    m = re.match(r"\s*([0-9]+\.[0-9]+)", ln)
    return float(m.group(1)) if m else None


# (t, value) で集める
pps_dev, ppb_t, ppb, err_t, errs, gen_dev, ph_t, phase = [], [], [], [], [], [], [], []
for ln in log:
    t = tof(ln)
    if (m := re.search(r"PPS count=\d+ interval_us=\d+ interval_ns=(\d+) state=(\w+) missed=\d+", ln)):
        if m.group(2) == "Locked" and abs(int(m.group(1)) - 1_000_000_000) < 1_000_000:
            pps_dev.append(int(m.group(1)) - 1_000_000_000)
    elif (m := re.search(r"TIME unix_ns=\d+ ppb=(-?\d+)", ln)):
        ppb_t.append(t); ppb.append(int(m.group(1)))
    elif (m := re.search(r"SYNC .*err_ns=(-?\d+)", ln)):
        err_t.append(t); errs.append(snap(int(m.group(1))))
    elif (m := re.search(r"PPSGEN count=\d+ interval_ns=\d+ dev_ns=(-?\d+) phase_ns=(-?\d+)", ln)):
        gen_dev.append(int(m.group(1))); ph_t.append(t); phase.append(int(m.group(2)))

fig, ax = plt.subplots(2, 2, figsize=(13, 8))
fig.suptitle("pico-gnss: GPSDO 時刻同期・規律 PPS 出力 の実機評価", fontsize=14, fontweight="bold")

# A: GPSDO 周波数そのものを時間軸で直接 (0→+2.4ppm にランプ→保持 が一目で分かる)
a = ax[0][0]
if len(ppb) > 5:
    p = np.array(ppb, float); tp = np.array(ppb_t, float)
    lock = np.median(p[len(p) * 2 // 3:]); ss = np.std(p[len(p) // 3:])
    a.plot(tp, p, color="#38bdf8", lw=1.4)
    a.set_ylim(min(0, p.min()), max(p) * 1.12 + 1)
    a.set_title(f"A. GPSDO: 起動で水晶 +{lock/1000:.2f}ppm を学習しロック→保持 (定常 σ≈{ss:.0f}ppb)", fontsize=11)
    a.set_xlabel("起動からの時間 [s]"); a.set_ylabel("推定周波数オフセット [ppb] (水晶 vs GPS)")
    a.grid(True, alpha=0.2)

# B: 時刻同期 ns 級 (±10ns spec 帯)
a = ax[0][1]
if len(errs) > 3:
    e = np.array(errs, float); te = np.array(err_t, float)
    sig = np.std(e[np.abs(e) < 1e6])
    a.axhspan(-10, 10, color="#10b981", alpha=0.18, label="MT3333 1PPS 仕様 ±10ns")
    a.plot(te, e, ".-", color="#34d399", ms=4, lw=0.5)
    a.axhline(0, color="#888", lw=0.6)
    lim = max(60, np.percentile(np.abs(e), 95) * 1.4)
    a.set_ylim(-lim, lim)
    a.set_title(f"B. 時刻補正残差 σ={sig:.0f}ns — 受信機 1PPS 仕様 ±10ns の内側", fontsize=11)
    a.set_xlabel("起動からの時間 [s]"); a.set_ylabel("補正後 UTC 残差 [ns]")
    a.legend(fontsize=9); a.grid(True, alpha=0.2)

# C: PPS / 規律出力 ジッタ — 各平均を引いて 0 中心に重ね、16ns 量子化を見せる
a = ax[1][0]
hp = np.array(pps_dev, float); hp -= hp.mean() if len(hp) else 0
ho = np.array([x for x in gen_dev if 1000 < abs(x) < 50000], float)
ho -= ho.mean() if len(ho) else 0
bins = np.arange(-72, 73, 8)  # 8ns 刻み (PIO tick=16ns)
if len(hp) > 4:
    a.hist(hp, bins=bins, color="#36d399", alpha=0.6, label=f"PPS 入力 σ{hp.std():.0f}ns (n={len(hp)})")
if len(ho) > 4:
    a.hist(ho, bins=bins, color="#fbbf24", alpha=0.6, label=f"規律出力 σ{ho.std():.0f}ns (n={len(ho)})")
a.set_title("C. PPS 入力 & 規律 PPS 出力 のジッタ — どちらも PIO 16ns が下限", fontsize=11)
a.set_xlabel("各平均からのズレ [ns] (16ns=PIO 1tick)"); a.set_ylabel("count")
a.legend(fontsize=9); a.grid(True, alpha=0.2)

# D: 位相同期の収束 (symlog: ms〜µs を 1 枚に)
a = ax[1][1]
if len(phase) > 2:
    ph = np.array(phase, float); tph = np.array(ph_t, float)
    a.plot(tph, ph, ".-", color="#7c3aed", ms=3, lw=0.6)
    a.set_yscale("symlog", linthresh=10000)  # ±10µs まで線形, 外は log
    for v in (1e6, -1e6, 1e5, -1e5):
        a.axhline(v, color="#ccc", lw=0.5, ls="--")
    a.axhline(0, color="#888", lw=0.7)
    settle = np.std(ph[len(ph) // 2:][np.abs(ph[len(ph) // 2:]) < 3e6])
    a.set_title(f"D. 規律出力の UTC 位相: 引き込むがソフトは ±{settle/1e6:.1f}ms 止まり", fontsize=11)
    a.set_xlabel("起動からの時間 [s]"); a.set_ylabel("UTC 秒境界からのズレ [ns] (symlog)")
    a.grid(True, which="both", alpha=0.15)
    a.text(0.98, 0.04, "ns 位相同期は測定の HW 化 (PIO) が必要", transform=a.transAxes,
           ha="right", fontsize=8, color="#888")

fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(out, dpi=110)
print(f"saved {out}  (pps {len(pps_dev)}, ppb {len(ppb)}, err {len(errs)}, gen {len(gen_dev)}, phase {len(phase)})")
