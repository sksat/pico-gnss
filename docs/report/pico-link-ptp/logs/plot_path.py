#!/usr/bin/env python3
# /// script
# dependencies = ["matplotlib", "numpy"]
# ///
"""パッドで測った経路の分布を、捕捉回数を数え損ねていた前後で並べる。

Pico client は「質問の先頭ビットが出てから応答の先頭ビットが着くまで」を、Pico server は
「質問の先頭ビットが着いてから応答の先頭ビットが出るまで」を、それぞれ自分の
state machine で測る。差が経路で、この配線ではジャンパ 2 本ぶんしかない。

2 つのログは別の板から出るので、順番でスライドさせて対応づける。
誤対応が数個あっても決まるように、標準偏差ではなく中央絶対偏差で合わせる。

生データは repo top の `logs/20260820-pico-link/` から読む。

  uv run docs/report/pico-link-ptp/logs/plot_path.py
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
RAW = os.path.join(HERE, "..", "..", "..", "..", "logs", "20260820-pico-link")

# 板は ticks で出す回と ns で出す回がある。ticks を ns に直すのは 2 system clock 分。
NS_PER_TICK = 2 * 1_000_000_000 // 125_000_000

CLIENT = re.compile(r"HWRTT n=\d+ rtt_(ns|ticks)=(-?\d+)")
SERVER = re.compile(r"hw_turnaround_(ns|ticks)=(-?\d+)")


def read(name, pattern):
    out = []
    with open(os.path.join(RAW, name), errors="replace") as f:
        for line in f:
            m = pattern.search(line)
            if not m:
                continue
            value = int(m.group(2))
            # i64::MIN は「ハードウェアの時刻が取れなかった」印。
            if value <= -(2**62):
                continue
            out.append(value * NS_PER_TICK if m.group(1) == "ticks" else value)
    return out


def pair(client_log, server_log):
    client = read(client_log, CLIENT)
    server = read(server_log, SERVER)
    best = None
    for shift in range(0, max(1, len(server) - len(client) + 1)):
        window = server[shift : shift + len(client)]
        if len(window) < len(client):
            break
        diffs = [c - s for c, s in zip(client, window)]
        med = st.median(diffs)
        spread = st.median([abs(d - med) for d in diffs])
        if best is None or spread < best[0]:
            best = (spread, diffs)
    return np.array(best[1]) / 1000.0  # µs


def main():
    if not os.path.isdir(RAW):
        print(f"生データが見つからない: {RAW}", file=sys.stderr)
        raise SystemExit(1)

    before = pair("rtt-client-pair2.log", "rtt-server-pair2.log")
    after = pair("rtt-client-fix.log", "rtt-server-fix.log")

    # 対応づけに失敗した回が混じるので、中央絶対偏差で内挿分を切り出す。
    # 除いた数は図に出す。隠して分布を綺麗に見せるためのものではない。
    def inliers(data):
        med = np.median(data)
        mad = np.median(np.abs(data - med))
        keep = np.abs(data - med) <= 10 * mad + 1.0
        return data[keep], int((~keep).sum())

    before_in, before_out = inliers(before)
    after_in, after_out = inliers(after)
    span = (-6.0, 12.0)

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharex=True)
    for ax, whole, kept, dropped, title in (
        (axes[0], before, before_in, before_out, "捕捉回数を数え損ねていたとき"),
        (axes[1], after, after_in, after_out, "カウンタ自身に数えさせたあと"),
    ):
        ax.hist(kept, bins=np.linspace(*span, 60), color="tab:blue", alpha=0.75)
        ax.axvline(0, color="0.5", lw=0.8)
        ax.axvline(np.median(kept), color="tab:red", lw=1.2, label="中央値")
        ax.set_xlim(*span)
        ax.set_xlabel("経路 = client の往復 − server の折り返し (µs)")
        ax.set_ylabel("count")
        ax.set_title(
            f"{title}\n"
            f"中央値 {np.median(kept):+.3f} µs   sd {kept.std():.3f} µs   "
            f"n={len(kept)}/{len(whole)}",
            fontsize=10,
        )
        if dropped:
            ax.text(
                0.98,
                0.82,
                f"対応づかず {dropped} 個除外",
                transform=ax.transAxes,
                ha="right",
                fontsize=8,
                color="0.35",
            )
        ax.legend(fontsize=8, loc="upper left")
        ax.grid(alpha=0.3)

    fig.suptitle(
        "ジャンパ 2 本の配線なので、経路は ns しかないはずである", fontsize=11
    )
    fig.tight_layout()
    path = os.path.join(OUT, "fig-path.png")
    fig.savefig(path, dpi=140)
    print(f"saved {path}")
    print(
        f"before: median={np.median(before_in):+.3f} us sd={before_in.std():.3f} "
        f"n={len(before_in)}/{len(before)}"
    )
    print(
        f"after:  median={np.median(after_in):+.3f} us sd={after_in.std():.3f} "
        f"n={len(after_in)}/{len(after)}"
    )


if __name__ == "__main__":
    main()
