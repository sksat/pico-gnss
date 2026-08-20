#!/usr/bin/env python3
# /// script
# dependencies = ["matplotlib", "numpy"]
# ///
"""Figures for the Pico 間リンクの NTP 時刻同期。

生データは repo top の `logs/20260820-pico-link/` から読む (gitignore 配下なのでコミットしない)。
出力はこのスクリプトの 1 つ上、`docs/report/pico-link-ntp/` に書く。

  uv run docs/report/pico-link-ntp/logs/plot_link.py
"""

import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# 図中に日本語を出すので、CJK を持つフォントを明示する。無いと豆腐になる。
matplotlib.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
RAW = os.path.join(HERE, "..", "..", "..", "..", "logs", "20260820-pico-link")


def read_pairs(name):
    """`scope_pair.py` の CSV から server と client の 1PPS 位置を µs で読む。"""
    server, client = [], []
    with open(os.path.join(RAW, name)) as f:
        for row in csv.DictReader(f):
            if not row["server_ns"] or not row["client_ns"]:
                continue
            server.append(float(row["server_ns"]) / 1000.0)
            client.append(float(row["client_ns"]) / 1000.0)
    return np.array(server), np.array(client)


def fig_broadcast_series():
    """broadcast で同期している間の、両方の 1PPS の位置。"""
    server, client = read_pairs("pair-series.csv")
    shots = np.arange(len(server))

    fig, (ax, hx) = plt.subplots(
        1, 2, figsize=(11, 4), gridspec_kw={"width_ratios": [3, 1]}
    )
    ax.axhline(0, color="0.6", lw=0.8)
    ax.plot(shots, server, ".-", ms=3, lw=0.8, label="server GP6 (GNSS 直結)")
    ax.plot(shots, client, ".-", ms=3, lw=0.8, label="client GP6 (リンク越しの NTP のみ)")
    ax.set_xlabel("shot (約 1 shot / 5 秒)")
    ax.set_ylabel("GPS-R の秒からのずれ (µs)")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(alpha=0.3)

    bins = np.linspace(
        min(server.min(), client.min()), max(server.max(), client.max()), 24
    )
    hx.hist(server, bins=bins, orientation="horizontal", alpha=0.6)
    hx.hist(client, bins=bins, orientation="horizontal", alpha=0.6)
    hx.axhline(0, color="0.6", lw=0.8)
    hx.set_xlabel("count")
    hx.tick_params(labelleft=False)
    hx.grid(alpha=0.3)

    fig.suptitle(
        f"1PPS の位置 (n={len(server)})   "
        f"server {server.mean():+.2f} µs (sd {server.std():.2f})   "
        f"client {client.mean():+.2f} µs (sd {client.std():.2f})",
        fontsize=10,
    )
    fig.tight_layout()
    path = os.path.join(OUT, "fig-broadcast-series.png")
    fig.savefig(path, dpi=140)
    print(f"saved {path}")


def main():
    if not os.path.isdir(RAW):
        print(f"生データが見つからない: {RAW}", file=sys.stderr)
        raise SystemExit(1)
    fig_broadcast_series()


if __name__ == "__main__":
    main()
