#!/usr/bin/env python3
# /// script
# dependencies = ["matplotlib", "numpy"]
# ///
"""レポート `docs/report/ptp-sync/` の図と数値を作る。

    uv run docs/report/ptp-sync/logs/plot_ptp.py

生データは repo top の `logs/20260822-ptp-sync/` から読む (gitignore 配下なのでコミットしない)。
本文の数値はここが出したものだけを使う。図の注釈も同じ計算から取る。
"""

import os
import re
import statistics as st
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

matplotlib.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
RAW = os.path.join(HERE, "..", "..", "..", "..", "logs", "20260822-ptp-sync")

RESULT = re.compile(r"PTP n=(\d+) seq=(\d+) offset_ns=(-?\d+) path_ns=(-?\d+)")
MOMENTS = re.compile(r"PTPRAW n=(\d+) to_slave=(-?\d+) to_master=(-?\d+) gap=(-?\d+)")

# The counter decrements once per two system cycles at 125 MHz.
TICK_NS = 16.0
# Four timestamps, each quantised to one tick, halved by the mean: the floor a measurement of this
# shape cannot go below, whatever the link does.
QUANTISATION_FLOOR_NS = (4 * TICK_NS**2 / 12) ** 0.5 / 2

# One run per message gap. The gap is a build-time constant on the master (`PTP_GAP_MS`).
RUNS = [(2, "model2"), (20, "v3-ptp"), (60, "model60")]


def read(tag):
    """`path_ns` と、4 つの moment から作った折り返し時間・slave の周波数誤差を読む。"""
    path, moments = [], []
    with open(os.path.join(RAW, f"rtt-client-{tag}.log"), errors="replace") as f:
        for line in f:
            m = RESULT.search(line)
            if m:
                path.append(int(m.group(4)))
            m = MOMENTS.search(line)
            if m:
                moments.append(tuple(int(g) for g in m.groups()))
    # The first exchange is taken before the clock has been set at all.
    return path[1:], moments[1:]


def inliers(values):
    """中央絶対偏差で外れ値を切る。除いた数は返す — 隠して分布を綺麗に見せるためではない。"""
    med = st.median(values)
    mad = st.median([abs(v - med) for v in values])
    keep = [v for v in values if abs(v - med) <= 10 * mad + 1]
    return keep, len(values) - len(keep)


def summarise(tag):
    path, moments = read(tag)
    keep, dropped = inliers(path)
    turnaround_ms = st.median([m[3] for m in moments]) / 1e6
    to_slave = [m[1] for m in moments]
    # One exchange a second, so the drift of t2 - t1 per exchange is ns per second.
    drift_ppm = (to_slave[-1] - to_slave[0]) / (len(to_slave) - 1) / 1e9 * 1e6
    return {
        "path": keep,
        "dropped": dropped,
        "median": st.median(keep),
        "sd": st.pstdev(keep),
        "turnaround_ms": turnaround_ms,
        "drift_ppm": drift_ppm,
    }


def fig_quantisation(runs):
    """1 回の測定がどこまで細かく読めるか。

    答えは「8 ns 格子」で、図はそれをそのまま見せる。4 つのタイムスタンプがそれぞれ 16 ns 格子に
    乗り、その半和が報告値になるので、値は 8 ns おきの櫛にしか立たない。
    """
    fig, axes = plt.subplots(1, len(runs), figsize=(11, 3.4), sharey=True)
    for ax, (gap, tag) in zip(axes, runs):
        s = runs_data[tag]
        v = np.array(s["path"], dtype=float)
        lo, hi = v.min() - 6, v.max() + 6
        # 1 ns の bin。格子より細かく刻んで、値が乗っていないところが空くのを見せる。
        ax.hist(v, bins=np.arange(lo, hi + 1, 1.0), color="tab:blue")
        for k in np.arange(np.floor(lo / 8) * 8, hi + 8, 8):
            ax.axvline(k, color="0.75", lw=0.8, zorder=0)
        ax.set_title(
            f"gap {gap} ms\n中央値 {s['median']:+.1f} ns   sd {s['sd']:.2f} ns   n={len(v)}",
            fontsize=9.5,
        )
        ax.set_xlabel("報告された経路遅延 (ns)")
    axes[0].set_ylabel("count")
    fig.suptitle(
        "縦の細線が 8 ns 格子。1 回の測定はこの櫛の上にしか立たない "
        f"(量子化だけで σ = {QUANTISATION_FLOOR_NS:.2f} ns)",
        fontsize=10,
    )
    fig.tight_layout()
    path = os.path.join(OUT, "fig-quantisation.png")
    fig.savefig(path, dpi=140)
    print(f"saved {path}")


def fig_gap(runs):
    """報告値が gap で動くこと、そしてそれが何であるか。

    `path = d - δ·T/2`。傾きから出る δ と、t2 - t1 のドリフトから出る δ が一致すれば、動いている
    のは経路ではなく slave のカウンタの速さである。
    """
    xs = np.array([runs_data[t]["turnaround_ms"] for _, t in runs])
    ys = np.array([runs_data[t]["median"] for _, t in runs])
    drifts = np.array([runs_data[t]["drift_ppm"] for _, t in runs])

    slope, intercept = np.polyfit(xs, ys, 1)
    # path = d - δ·T/2 なので、傾き = -δ/2 (ns/ms = 1e-6 の比)。
    delta_from_slope_ppm = -2.0 * slope
    residual = ys - slope * xs

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(11, 3.8), gridspec_kw={"width_ratios": [1.3, 1]})
    ax.plot(xs, ys, "o", ms=7, color="tab:blue", label="実測 (中央値)")
    fine = np.linspace(0, xs.max() * 1.1, 50)
    ax.plot(fine, slope * fine + intercept, "-", lw=1.2, color="tab:red",
            label=f"あてはめ  {slope:+.3f} ns/ms")
    ax.axhline(intercept, color="0.6", lw=0.8, ls="--")
    ax.annotate(f"T→0 の外挿 {intercept:+.1f} ns", xy=(0, intercept),
                xytext=(6, 10), textcoords="offset points", fontsize=8.5, color="0.35")
    for x, y, (gap, _) in zip(xs, ys, runs):
        ax.annotate(f"gap {gap} ms", xy=(x, y), xytext=(0, -16),
                    textcoords="offset points", ha="center", fontsize=8.5)
    ax.set_xlabel("slave の折り返し時間 T = t3 − t2 (ms)")
    ax.set_ylabel("報告された経路遅延の中央値 (ns)")
    ax.set_title("報告値は経路ではなく、slave の速さ × 折り返し時間で動く", fontsize=10)
    ax.legend(fontsize=8.5, loc="upper right")
    ax.grid(alpha=0.3)

    labels = [f"gap {g} ms" for g, _ in runs]
    idx = np.arange(len(runs))
    bx.bar(idx - 0.18, drifts, width=0.34, color="tab:orange", label="t2 − t1 のドリフトから")
    bx.bar(idx + 0.18, [delta_from_slope_ppm] * len(runs), width=0.34, color="tab:green",
           label="上の傾きから")
    bx.set_xticks(idx)
    bx.set_xticklabels(labels, fontsize=8.5)
    bx.set_ylabel("slave のカウンタの周波数誤差 (ppm)")
    bx.set_title("2 通りの独立な推定が一致する", fontsize=10)
    bx.legend(fontsize=8)
    bx.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    path = os.path.join(OUT, "fig-gap.png")
    fig.savefig(path, dpi=140)
    print(f"saved {path}")
    print(f"  slope {slope:+.3f} ns/ms -> delta {delta_from_slope_ppm:.3f} ppm")
    print(f"  drift estimates {', '.join(f'{d:.3f}' for d in drifts)} ppm "
          f"(mean {drifts.mean():.3f})")
    print(f"  residual d at each gap: {', '.join(f'{r:+.1f}' for r in residual)} ns")


def main():
    if not os.path.isdir(RAW):
        print(f"生データが見つからない: {RAW}", file=sys.stderr)
        raise SystemExit(1)
    global runs_data
    runs_data = {}
    for gap, tag in RUNS:
        if not os.path.exists(os.path.join(RAW, f"rtt-client-{tag}.log")):
            print(f"{tag} が無いので飛ばす", file=sys.stderr)
            continue
        runs_data[tag] = summarise(tag)
        s = runs_data[tag]
        print(
            f"gap {gap:>2} ms  n={len(s['path']):>4} (除外 {s['dropped']})  "
            f"median {s['median']:+7.1f} ns  sd {s['sd']:5.2f}  "
            f"T {s['turnaround_ms']:7.3f} ms  drift {s['drift_ppm']:+.3f} ppm"
        )
    present = [(g, t) for g, t in RUNS if t in runs_data]
    if len(present) >= 2:
        fig_quantisation(present)
        fig_gap(present)
    print(f"quantisation floor: sigma = {QUANTISATION_FLOOR_NS:.3f} ns (tick {TICK_NS:g} ns)")


if __name__ == "__main__":
    main()
