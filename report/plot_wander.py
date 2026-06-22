# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""出力位相 (hwphase) の揺れが温度 (ppb) とどう連動するかを時系列で可視化する。

firmware ログ (PPSGEN の hwphase_ns / lk, TIME の ppb) を読み、上段に hwphase の点 (locked=青、unlock=赤) と
移動標準偏差 (rolling sd)、下段に ppb (温度 proxy) を時間軸で並べる。温度が平坦な区間で hwphase が 2桁ns へ
締まり、ランプ区間で揺れが増える様子と、落ち着き具合を見る用。hwphase は実出力 vs GPS の真値 (scope の
プローブスキューは乗らない)。

  uv run report/plot_wander.py [pps-review.log] [out.png] [rolling_window_s]
"""
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

LOG = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pps-review.log"
OUT = sys.argv[2] if len(sys.argv) > 2 else "report/offset-wander-settle.png"
WIN_S = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0

gen_t, gen_hw, gen_lk, ppb_t, ppb_v = [], [], [], [], []
for line in open(LOG, errors="ignore"):
    m = re.match(r"(\d+\.\d+)", line)
    if not m:
        continue
    t = float(m.group(1))
    if "PPSGEN count" in line:
        h = re.search(r"hwphase_ns=(-?\d+)", line)
        lk = re.search(r"lk=(\d)", line)
        if h and lk:
            gen_t.append(t)
            gen_hw.append(int(h.group(1)))
            gen_lk.append(int(lk.group(1)))
    elif "TIME " in line:
        p = re.search(r"ppb=(-?\d+)", line)
        if p:
            ppb_t.append(t)
            ppb_v.append(int(p.group(1)))

if not gen_t:
    sys.exit("no PPSGEN hwphase in " + LOG)

gt = np.array(gen_t)
gh = np.array(gen_hw, float)
glk = np.array(gen_lk)
t0 = gt[0]
gm = (gt - t0) / 60.0  # minutes

# rolling sd over WIN_S (locked samples only), centered
roll_t, roll_sd = [], []
locked = glk == 1
for i in range(len(gt)):
    lo = gt[i] - WIN_S / 2
    hi = gt[i] + WIN_S / 2
    win = gh[(gt >= lo) & (gt <= hi) & locked]
    if len(win) >= 5:
        roll_t.append(gm[i])
        roll_sd.append(np.std(win))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

lk1 = glk == 1
ax1.scatter(gm[lk1], gh[lk1], s=6, color="tab:blue", alpha=0.4, label="hwphase (locked)")
if (~lk1).any():
    ax1.scatter(gm[~lk1], gh[~lk1], s=6, color="tab:red", alpha=0.4, label="hwphase (unlocked)")
ax1.axhline(0, color="k", lw=0.7, alpha=0.5)
# y レンジは起動後 (>0.5min) の locked 点で決める (t=0 の未初期化外れ値で潰れないように)。
settled = gh[lk1 & (gm > 0.5)]
if len(settled):
    lim = max(300.0, np.percentile(np.abs(settled), 99) * 1.15)
    ax1.set_ylim(-lim, lim)
ax1.set_ylabel("hwphase = output PPS − GPS [ns]\n(real offset②, no probe skew)")
ax1.legend(loc="upper left", fontsize=9)
ax1.grid(alpha=0.3)
axr = ax1.twinx()
axr.plot(roll_t, roll_sd, color="tab:orange", lw=1.5, label=f"rolling sd ({WIN_S:.0f}s)")
axr.set_ylabel(f"rolling sd [ns]", color="tab:orange")
axr.tick_params(axis="y", labelcolor="tab:orange")
axr.set_ylim(bottom=0)
axr.legend(loc="upper right", fontsize=9)
ax1.set_title("output phase wander vs temperature: settle-watch (hwphase + rolling sd)")

if ppb_t:
    pm = (np.array(ppb_t) - t0) / 60.0
    ax2.plot(pm, ppb_v, color="tab:green", lw=1.0)
    ax2.set_ylabel("crystal ppb (temp proxy)", color="tab:green")
    ax2.tick_params(axis="y", labelcolor="tab:green")
ax2.set_xlabel("elapsed time [min]")
ax2.grid(alpha=0.3)

fig.tight_layout()
fig.savefig(OUT, dpi=110)
loc_sd = np.std(gh[lk1]) if lk1.any() else 0
print(f"wrote {OUT}: {len(gt)} edges, {gm[-1]:.1f} min, "
      f"locked hwphase sd={loc_sd:.0f}ns, ppb {min(ppb_v) if ppb_v else 0}..{max(ppb_v) if ppb_v else 0}")
