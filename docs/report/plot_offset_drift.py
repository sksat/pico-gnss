# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""長期監視 (longmon) が記録した絶対オフセット②の温度ドリフトを可視化する。

longmon-result.log の各 "measure #N" 行から (firmware時刻, GP3-GPS offset, ppb) を取り、
ppb が無い旧形式行は firmware ログ (pps-flash.log の TIME 行) から近傍 ppb を補完する。
左: offset vs 経過時間、右: offset vs ppb(temp proxy) + 線形 fit (ns/ppb)。

  uv run docs/report/plot_offset_drift.py [longmon-result.log] [pps-flash.log] [out.png]
"""
import bisect
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RESULT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/longmon-result.log"
FWLOG = sys.argv[2] if len(sys.argv) > 2 else "/tmp/pps-flash.log"
OUT = sys.argv[3] if len(sys.argv) > 3 else "docs/report/offset-drift.png"

MRE = re.compile(r"\[t=(\d+)\] measure #\d+:.*?offset=(-?\d+)ns")
PRE = re.compile(r"ppb=(-?\d+)")

meas = []  # [t, offset_ns, ppb_or_None]
for line in open(RESULT):
    m = MRE.search(line)
    if not m:
        continue
    pm = PRE.search(line)
    meas.append([int(m.group(1)), int(m.group(2)), int(pm.group(1)) if pm else None])

if not meas:
    sys.exit("no 'measure #' lines in " + RESULT)

# backfill ppb from firmware TIME lines for old-format rows
if any(m[2] is None for m in meas):
    tre = re.compile(r"^(\d+\.\d+) .*ppb=(-?\d+) holdover")
    times, ppbs = [], []
    for line in open(FWLOG):
        mm = tre.match(line)
        if mm:
            times.append(float(mm.group(1)))
            ppbs.append(int(mm.group(2)))
    for m in meas:
        if m[2] is None and times:
            i = min(max(bisect.bisect_left(times, m[0]), 0), len(times) - 1)
            m[2] = ppbs[i]

t = np.array([m[0] for m in meas], float) / 3600.0  # hours
off = np.array([m[1] for m in meas], float)
ppb = np.array([m[2] for m in meas], float)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))

ax1.plot(t, off, "o-", color="tab:red", label="GP3-GPS offset")
ax1.set_xlabel("firmware time [h]")
ax1.set_ylabel("scope GP3-GPS offset [ns]", color="tab:red")
ax1.tick_params(axis="y", labelcolor="tab:red")
ax1.set_title(f"absolute offset② drift ({len(meas)} measurements)")
axb = ax1.twinx()
axb.plot(t, ppb, "s--", color="tab:blue", alpha=0.6, label="ppb")
axb.set_ylabel("crystal ppb (temp proxy)", color="tab:blue")
axb.tick_params(axis="y", labelcolor="tab:blue")

ax2.scatter(ppb, off, c=t, cmap="viridis")
if len(meas) >= 2 and np.ptp(ppb) > 0:
    a, b = np.polyfit(ppb, off, 1)
    xs = np.linspace(ppb.min(), ppb.max(), 50)
    ax2.plot(xs, a * xs + b, "k--", alpha=0.7, label=f"fit {a:.2f} ns/ppb")
    ax2.legend()
ax2.set_xlabel("crystal ppb (temp proxy)")
ax2.set_ylabel("scope GP3-GPS offset [ns]")
ax2.set_title("offset vs ppb (color=time)")

fig.tight_layout()
fig.savefig(OUT, dpi=110)
print(f"wrote {OUT} ({len(meas)} pts); offset {off.min():.0f}..{off.max():.0f} ns, "
      f"ppb {ppb.min():.0f}..{ppb.max():.0f}")
