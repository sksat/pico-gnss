#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["matplotlib", "numpy"]
# ///
"""Overlapping Allan deviation / TDEV of the disciplined 1PPS output (from a firmware log).

External-reference-free: it uses the firmware's *own* GP3->GP4 loopback phase error
(`hwphase_ns`, logged once per second in the PPSGEN line), so it characterizes how stably the
disciplined output tracks the GPS reference — no external instrument involved.

  uv run allan.py <logfile> [allan.png]

`hwphase_ns` is the output-vs-GPS time error x(t) at 1 Hz. From it:
  - overlapping Allan deviation  sigma_y(tau)  (fractional-frequency stability vs averaging time)
  - time deviation  TDEV(tau)                  (time-domain stability, ns)

Labels: Japanese by default, English when PLOT_LANG=en.
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


HWPHASE = re.compile(r"hwphase_ns=(-?\d+)")


def adev_octave(x, tau0):
    """Overlapping Allan deviation at octave-spaced tau (x = time error [s], evenly @ tau0)."""
    n = len(x)
    taus, devs = [], []
    m = 1
    while 2 * m < n:
        d = x[2 * m:] - 2 * x[m:-m] + x[: -2 * m]
        var = np.sum(d * d) / (2 * (n - 2 * m) * (m * tau0) ** 2)
        taus.append(m * tau0)
        devs.append(np.sqrt(var))
        m *= 2
    return np.array(taus), np.array(devs)


def tdev_octave(x, tau0):
    """Overlapping TDEV at octave-spaced tau. TDEV = tau/sqrt(3) * MDEV."""
    n = len(x)
    taus, devs = [], []
    m = 1
    while 3 * m < n:
        # modified Allan: average the second difference over a window of m, then variance.
        acc = np.zeros(n - 3 * m + 1)
        for j in range(m):
            acc += x[2 * m + j: 2 * m + j + len(acc)] - 2 * x[m + j: m + j + len(acc)] + x[j: j + len(acc)]
        mvar = np.sum(acc * acc) / (2 * m**2 * (m * tau0) ** 2 * len(acc))
        mdev = np.sqrt(mvar)
        taus.append(m * tau0)
        devs.append((m * tau0) / np.sqrt(3) * mdev)
        m *= 2
    return np.array(taus), np.array(devs)


def main():
    log = sys.argv[1] if len(sys.argv) > 1 else "capture.log"
    out = sys.argv[2] if len(sys.argv) > 2 else "allan.png"
    with open(log) as f:
        ph_ns = [int(m.group(1)) for line in f for m in [HWPHASE.search(line)] if m]
    if len(ph_ns) < 16:
        sys.exit(f"need >=16 hwphase samples, got {len(ph_ns)}")
    x = np.array(ph_ns, dtype=float) * 1e-9  # ns -> s (time error)
    n = len(x)
    tau0 = 1.0  # PPSGEN is logged at 1 Hz
    taus, adev = adev_octave(x, tau0)
    ttaus, tdev = tdev_octave(x, tau0)

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11, 4.0))
    fig.suptitle(
        L(
            f"GPSDO の 1PPS 出力の安定度 (loopback hwphase, 外部基準なし, N={n}s)",
            f"Disciplined 1PPS output stability (loopback hwphase, no external ref, N={n}s)",
        ),
        fontsize=12, fontweight="bold",
    )

    # phase time series
    ax0.plot(np.arange(n), x * 1e9, ".", ms=3, color="#60a5fa")
    ax0.axhline(np.mean(x) * 1e9, color="#f87272", lw=1, label=L(f"平均 {np.mean(x) * 1e9:.0f}ns", f"mean {np.mean(x) * 1e9:.0f}ns"))
    ax0.set_xlabel(L("時刻 [s]", "time [s]"))
    ax0.set_ylabel(L("出力位相誤差 hwphase [ns]", "output phase error hwphase [ns]"))
    ax0.set_title(L(f"位相時系列  σ={np.std(x) * 1e9:.0f}ns", f"phase series  σ={np.std(x) * 1e9:.0f}ns"))
    ax0.legend(fontsize=9)
    ax0.grid(True, alpha=0.2)

    # Allan deviation (log-log)
    ax1.loglog(taus, adev, "o-", color="#36d399", label="overlapping ADEV σ_y(τ)")
    ax1.loglog(ttaus, np.array(tdev) / np.array(ttaus), "s--", color="#fbbd23", alpha=0.6,
               label=L("TDEV(τ)/τ (参考)", "TDEV(τ)/τ (ref)"))
    ax1.set_xlabel(L("平均時間 τ [s]", "averaging time τ [s]"))
    ax1.set_ylabel(L("Allan 偏差 σ_y(τ)", "Allan deviation σ_y(τ)"))
    ax1.set_title(L("Allan 偏差 (小さいほど安定)", "Allan deviation (lower = more stable)"))
    ax1.legend(fontsize=9)
    ax1.grid(True, which="both", alpha=0.2)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=120)
    print(f"wrote {out}  (N={n}s, ADEV@1s={adev[0]:.2e}, TDEV@{int(ttaus[-1])}s={tdev[-1] * 1e9:.1f}ns)")


if __name__ == "__main__":
    main()
