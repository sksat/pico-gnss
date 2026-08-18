# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib"]
# ///
"""レポート `docs/report/pps-nmea-pairing/` の図を作る。

    uv run docs/report/pps-nmea-pairing/logs/pairing/plot_pairing.py

生データは repo top の `logs/20260818-ntp-bringup/` から読む (コミットしない)。
出力は `docs/report/pps-nmea-pairing/` 直下の PNG。

図はどちらも実測から起こす。模式図に見えるほうも、バーストの開始時刻と各センテンスの
位置は実際のログから取った中央値で描いている。

図中のラベルは英語にしてある。日本語フォントが入っていない環境で豆腐になるのを避けるため。
"""

import re
import statistics
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

REPO = Path(__file__).resolve().parents[5]
RAW = REPO / "logs" / "20260818-ntp-bringup"
OUT = Path(__file__).resolve().parents[2]

TIME_RE = re.compile(r"^(\d+\.\d+)\s")
PPS_RE = re.compile(r"PPS count=\d+ ")
NMEA_RE = re.compile(r"NMEA \$([A-Z]{2})([A-Z]{3})")

# 見やすさのため、図の色は 2 つの条件で固定する。
SLOW_C, FAST_C = "#c1442e", "#2e7d5b"


def load(path: Path):
    """(PPS エッジ時刻, [(センテンス時刻, 種別)]) を返す。"""
    edges, sentences = [], []
    for line in path.read_text(errors="replace").splitlines():
        m = TIME_RE.match(line)
        if not m:
            continue
        t = float(m.group(1))
        if PPS_RE.search(line):
            edges.append(t)
            continue
        n = NMEA_RE.search(line)
        if n:
            sentences.append((t, n.group(2)))
    return edges, sentences


def margins(edges, sentences, kind):
    """種別 `kind` のセンテンスについて (直前エッジからの経過, 次エッジまでの残り)。"""
    out = []
    for t, k in sentences:
        if k != kind:
            continue
        prev = [e for e in edges if e <= t]
        nxt = [e for e in edges if e > t]
        if prev and nxt:
            out.append((t - prev[-1], nxt[0] - t))
    return out


def one_second(edges, sentences, want=("RMC", "ZDA")):
    """代表的な 1 秒 (エッジからエッジ) を選び、その区間のセンテンス到着を実測のまま返す。

    模式図ではなくログそのものを描くために使う。選ぶのは「その区間に載ったセンテンス数が
    中央値の区間」で、たまたま短い/長い秒を代表にしないようにする。
    """
    per_edge: dict[int, list[tuple[float, str]]] = {}
    for t, k in sentences:
        prev = [i for i, e in enumerate(edges) if e <= t]
        if prev and prev[-1] + 1 < len(edges):
            per_edge.setdefault(prev[-1], []).append((t - edges[prev[-1]], k))
    if not per_edge:
        return []
    counts = sorted(len(v) for v in per_edge.values())
    target = counts[len(counts) // 2]
    for _i, items in sorted(per_edge.items()):
        if len(items) == target:
            return sorted(items)
    return sorted(next(iter(per_edge.values())))


def occupancy(sentences, span, baud):
    """NMEA が UART をどれだけ占有しているか (0..1)。1 文字 10 bit で数える。"""
    chars = sum(len(k) + 8 for _, k in sentences)  # talker+type+本体のおおよそ
    return chars * 10 / span / baud


def fig_timeline(slow, fast, path: Path):
    """代表的な 1 秒に、実測のセンテンス到着をそのまま並べる。"""
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.0), sharex=True)
    for ax, (label, items, colour, note) in zip(
        axes,
        [
            ("9600 baud", slow, SLOW_C, "82% of the link is NMEA"),
            ("115200 baud", fast, FAST_C, "7% of the link is NMEA"),
        ],
    ):
        for x in (0.0, 1.0):
            ax.axvline(x, color="#333", lw=2)
        ax.text(0.0, 1.30, "PPS edge", ha="center", fontsize=9, color="#333")
        ax.text(1.0, 1.30, "next PPS edge", ha="center", fontsize=9, color="#333")

        for off, kind in items:
            hot = kind in ("RMC", "ZDA")
            ax.plot(
                [off, off],
                [0.35, 0.75] if hot else [0.45, 0.65],
                color=colour if hot else "#999",
                lw=2.4 if hot else 1.2,
            )
        # 時刻センテンスだけ、次エッジまでの残りを添える
        for kind, dy in (("RMC", -26), ("ZDA", -44)):
            hit = [o for o, k in items if k == kind]
            if not hit:
                continue
            off = hit[0]
            ax.annotate(
                f"{kind}: {(1.0 - off) * 1000:.0f} ms to the next edge",
                (off, 0.35),
                textcoords="offset points",
                xytext=(4, dy),
                ha="left",
                fontsize=8,
                color=colour,
                arrowprops=dict(arrowstyle="-", color=colour, lw=0.8),
            )
        ax.text(0.5, 1.02, note, ha="center", fontsize=9, color=colour)
        ax.set_ylim(0, 1.5)
        ax.set_yticks([])
        ax.set_ylabel(label, rotation=0, ha="right", va="center", fontsize=10)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)

    axes[-1].set_xlim(-0.04, 1.12)
    axes[-1].set_xlabel("seconds after a PPS edge")
    fig.suptitle(
        "One second of NMEA, as logged: every sentence arrival between two PPS edges",
        fontsize=12,
        y=0.99,
    )
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path, dpi=150)
    print(f"wrote {path}")


def fig_margin(slow_m, fast_m, path: Path):
    """次エッジまでの残りの分布。9600 は二峰、115200 は一峰。"""
    fig, ax = plt.subplots(figsize=(9, 3.6))
    bins = [i / 50 for i in range(51)]
    ax.hist(
        [m * 1000 for _, m in slow_m],
        bins=[b * 1000 for b in bins],
        color=SLOW_C,
        alpha=0.75,
        label=f"9600 baud (n={len(slow_m)})",
    )
    ax.hist(
        [m * 1000 for _, m in fast_m],
        bins=[b * 1000 for b in bins],
        color=FAST_C,
        alpha=0.75,
        label=f"115200 baud (n={len(fast_m)})",
    )
    ax.axvline(0, color="#333", lw=2)
    ax.annotate(
        "the next PPS edge\n(crossing it shifts the epoch by one second)",
        (0, ax.get_ylim()[1] * 0.95),
        textcoords="offset points",
        xytext=(8, -6),
        va="top",
        fontsize=9,
        color="#333",
    )
    ax.set_xlabel("margin from the time sentence to the next PPS edge (ms)")
    ax.set_ylabel("sentences")
    ax.legend(loc="upper right")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"wrote {path}")


def stats(ms):
    v = [m * 1000 for _, m in ms]
    return f"n={len(v)} mean={statistics.mean(v):.0f}ms sd={statistics.pstdev(v):.0f}ms min={min(v):.0f}ms"


def main() -> int:
    slow_log = RAW / "rtt-9600-nmea.log"
    fast_log = RAW / "rtt-picognss-fast.log"
    for p in (slow_log, fast_log):
        if not p.exists():
            print(f"missing raw log: {p}", file=sys.stderr)
            return 1

    slow_e, slow_s = load(slow_log)
    fast_e, fast_s = load(fast_log)

    slow_rmc, fast_rmc = margins(slow_e, slow_s, "RMC"), margins(fast_e, fast_s, "RMC")
    slow_zda = margins(slow_e, slow_s, "ZDA")
    print(f"9600   RMC {stats(slow_rmc)}")
    print(f"9600   ZDA {stats(slow_zda)}")
    print(f"115200 RMC {stats(fast_rmc)}")

    fig_timeline(
        one_second(slow_e, slow_s),
        one_second(fast_e, fast_s),
        OUT / "fig-burst-timing.png",
    )
    fig_margin(slow_rmc, fast_rmc, OUT / "fig-margin.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
