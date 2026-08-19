#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""Visualize lock-acquisition convergence of the disciplined 1PPS, two ways:

  left  — self-reported: firmware loopback phase `hwphase_ns` per output edge (from the defmt log)
  right — external: the oscilloscope-measured GPS->gen offset vs elapsed time (scope_pps.py converge)

Both show the output edge being pulled onto the GPS second after boot. The scope only sees the
gen edge once it is inside its window (~+-100 us), so it captures the final pull-in; the firmware
self-report covers the whole range (hundreds of us -> 0), shown on a symlog axis.

  uv run plot_converge.py <defmt.log> <scope-converge.log> [out.png]
"""
import os
import re
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

for _f in ("Noto Sans CJK JP", "IPAGothic", "TakaoGothic"):
    try:
        matplotlib.rcParams["font.family"] = _f
        break
    except Exception:
        pass
matplotlib.rcParams["axes.unicode_minus"] = False

EN = os.environ.get("PLOT_LANG", "").lower() == "en"


def L(ja, en):
    return en if EN else ja


PPSGEN = re.compile(r"PPSGEN count=(\d+).*?hwphase_ns=(-?\d+)")


def main():
    defmt = sys.argv[1]
    scope = sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else "converge.png"

    # self-reported: (edge#, hwphase ns)
    cnt, hw = [], []
    with open(defmt) as f:
        for line in f:
            m = PPSGEN.search(line)
            if m:
                cnt.append(int(m.group(1)))
                hw.append(int(m.group(2)))
    cnt = np.array(cnt)
    hw = np.array(hw, dtype=float)

    # external: (elapsed s, offset ns), nan = out of window
    el, off = [], []
    with open(scope) as f:
        for line in f:
            if line.startswith("#"):
                continue
            a, b = line.split()
            el.append(float(a))
            off.append(float("nan") if b == "nan" else float(b))
    el = np.array(el)
    off = np.array(off)

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11, 4.0))
    fig.suptitle(
        L("GPSDO 1PPS のロック収束 (自己申告 + オシロ外部観測)",
          "Disciplined 1PPS lock-in convergence (self-reported + external scope)"),
        fontsize=12, fontweight="bold",
    )

    # self-reported (symlog: hundreds of us -> ~0)
    ax0.plot(cnt, hw, ".-", ms=3, lw=0.6, color="#36d399")
    ax0.axhline(0, color="#888", lw=0.8)
    ax0.set_yscale("symlog", linthresh=100)
    ax0.set_xlabel(L("出力エッジ番号 (≈秒)", "output edge # (≈ s)"))
    ax0.set_ylabel(L("自己申告 hwphase [ns] (symlog)", "self-reported hwphase [ns] (symlog)"))
    ax0.set_title(L("自己申告 (loopback): 全域 µs→0", "self-report (loopback): full range µs→0"))
    ax0.grid(True, which="both", alpha=0.2)

    # external (the in-window pull-in)
    ok = np.isfinite(off)
    ax1.plot(el[ok], off[ok], ".", ms=4, color="#60a5fa")
    ax1.axhline(0, color="#888", lw=0.8)
    ax1.set_xlabel(L("経過 [s]", "elapsed [s]"))
    ax1.set_ylabel(L("オシロ GPS→生成 [ns]", "scope GPS→gen [ns]"))
    ax1.set_title(L(f"オシロ外部: 窓内の最終引き込み ({ok.sum()}/{len(off)} 点)",
                    f"external scope: final pull-in ({ok.sum()}/{len(off)} pts)"))
    ax1.grid(True, alpha=0.2)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=120)
    print(f"wrote {out}  (self N={len(cnt)}, scope ok={int(np.isfinite(off).sum())}/{len(off)})")


if __name__ == "__main__":
    main()
