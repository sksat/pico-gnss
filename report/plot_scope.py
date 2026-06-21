#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""Visualize the oscilloscope GPS->gen 1PPS phase log (matplotlib).

Independent cross-check data from scope_pps.py: one GPS->gen rising-edge delta
(ns) per single-shot. Two panels:
  - distribution (histogram) with mean/sigma
  - per-shot sequence, to show the offset is a stable systematic, not drift

Usage (uv builds an isolated env automatically):
    uv run plot_scope.py scope-phase.log [scope-phase.png]

Labels: Japanese by default, English when PLOT_LANG=en.
"""
import os
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

EN = os.environ.get("PLOT_LANG", "").lower() == "en"


def L(ja, en):
    return en if EN else ja


for _f in ("Noto Sans CJK JP", "IPAGothic", "TakaoGothic"):
    try:
        matplotlib.rcParams["font.family"] = _f
        break
    except Exception:
        pass
matplotlib.rcParams["axes.unicode_minus"] = False


def main():
    log = sys.argv[1] if len(sys.argv) > 1 else "scope-phase.log"
    out = sys.argv[2] if len(sys.argv) > 2 else "scope-phase.png"
    d = np.loadtxt(log, comments="#")
    d = np.atleast_1d(d)
    n = d.size
    mean, sd = float(np.mean(d)), float(np.std(d))
    us = mean / 1000.0

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.0), width_ratios=[1, 1.25])
    fig.suptitle(
        L(
            f"オシロ独立検証: 生成 1PPS は GPS PPS に σ≈{sd:.0f}ns で追従 "
            f"(固定オフセット {us:+.2f}µs, N={n})",
            f"Scope cross-check: generated 1PPS tracks GPS PPS at σ≈{sd:.0f}ns "
            f"(fixed offset {us:+.2f}µs, N={n})",
        ),
        fontsize=12, fontweight="bold",
    )

    # --- distribution ---
    ax1.hist(d - mean, bins=24, color="#36d399", edgecolor="#0b0f14", alpha=0.85)
    ax1.axvline(0, color="#f87272", lw=1.5)
    ax1.axvline(-sd, color="#fbbd23", lw=1, ls="--")
    ax1.axvline(+sd, color="#fbbd23", lw=1, ls="--")
    ax1.set_xlabel(L("平均からの偏差 [ns]", "deviation from mean [ns]"))
    ax1.set_ylabel(L("出現回数", "count"))
    ax1.set_title(L(f"分布  σ={sd:.0f}ns  pp={np.ptp(d):.0f}ns", f"distribution  σ={sd:.0f}ns  pp={np.ptp(d):.0f}ns"))
    ax1.text(
        0.03, 0.97,
        L(
            "σ = GPS PPS sawtooth +\nサーボの緩慢な位相ふらつき",
            "σ = GPS PPS sawtooth +\nslow servo phase wander",
        ),
        transform=ax1.transAxes, va="top", fontsize=9, color="#555",
    )

    # --- sequence ---
    ax2.plot(np.arange(n), d, ".", ms=4, color="#60a5fa")
    ax2.axhline(mean, color="#f87272", lw=1.2, label=L(f"平均 {mean:.0f}ns", f"mean {mean:.0f}ns"))
    ax2.fill_between([0, n - 1], mean - sd, mean + sd, color="#fbbd23", alpha=0.15, label="±1σ")
    ax2.set_xlabel(L("ショット番号", "shot #"))
    ax2.set_ylabel(L("GPS→生成 位相 [ns]", "GPS->gen phase [ns]"))
    ax2.set_title(L("時系列 (ドリフトなし = 固定オフセット)", "sequence (no drift = fixed offset)"))
    ax2.legend(loc="upper right", fontsize=9)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=120)
    print(f"wrote {out}  (N={n}, mean={mean:.0f}ns, sigma={sd:.0f}ns)")


if __name__ == "__main__":
    main()
