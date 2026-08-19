# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib"]
# ///
"""レポート `docs/report/ntp-stratum1/` の図を作る。

    uv run docs/report/ntp-stratum1/logs/unicast/plot_unicast.py

生データは repo top の `logs/20260819-ntp-unicast/` から読む (コミットしない)。
出力は `docs/report/ntp-stratum1/` 直下の PNG。

図中のラベルは英語にしてある。日本語フォントが入っていない環境で豆腐になるのを避けるため。
"""

import statistics
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[5]
RAW = REPO / "logs" / "20260819-ntp-unicast"
OUT = Path(__file__).resolve().parents[2]

BEFORE_C, AFTER_C = "#c1442e", "#2e7d5b"


def load(path: Path):
    """(経過分, offset ms, delay ms) を返す。1 行 1 交換。"""
    rows = []
    for line in path.read_text().splitlines()[1:]:
        if not line or line.startswith("#"):
            continue
        f = line.split(",")
        rows.append((float(f[0]), float(f[1]), float(f[2])))
    t0 = rows[0][0]
    return [((t - t0) / 60, o, d) for t, o, d in rows]


def summary(name, rows):
    offs = [r[1] for r in rows]
    delays = [r[2] for r in rows]
    return (
        f"{name}: n={len(rows)} offset mean={statistics.mean(offs):+.3f}ms "
        f"sd={statistics.pstdev(offs):.3f}ms  delay median={statistics.median(delays):.2f}ms"
    )


def fig_edge(before, after, path: Path):
    """捕捉するエッジを変える前と後の offset。

    同じ軸に置くと、後の 1 ms 未満の分布が 100 ms のずれに潰されて何も読めない。段を分けて
    それぞれの縦軸を独立させ、代わりに各段へ平均と標準偏差を書く。
    """
    fig, axes = plt.subplots(2, 1, figsize=(9, 5.4))
    for ax, (label, rows, colour) in zip(
        axes,
        [
            ("capturing the rising edge", before, BEFORE_C),
            ("capturing the falling edge", after, AFTER_C),
        ],
    ):
        mins = [r[0] for r in rows]
        offs = [r[1] for r in rows]
        ax.plot(mins, offs, ".", ms=3, color=colour)
        ax.axhline(statistics.mean(offs), color=colour, lw=1, alpha=0.6)
        ax.axhline(0, color="#333", lw=1, ls=":")
        ax.set_ylabel(f"{label}\noffset (ms)", fontsize=9.5)
        ax.text(
            0.01,
            0.08,
            f"mean {statistics.mean(offs):+.2f} ms   sd {statistics.pstdev(offs):.2f} ms   n={len(offs)}",
            transform=ax.transAxes,
            fontsize=9.5,
            color=colour,
        )
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    axes[-1].set_xlabel("minutes")
    fig.suptitle(
        "The client's offset, before and after moving the capture to the marking edge",
        fontsize=12,
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"wrote {path}")


def fig_unicast(rows, path: Path):
    """クライアントから見た offset と往復時間。

    2 段に分けるのは、片方が時計のずれで、もう片方が経路の揺らぎだから。重ねると
    「ずれているのか揺れているのか」が読めなくなる。

    往復時間には注釈を付ける。これは probe の経路であって Ethernet ではないので、注釈が
    ないと「10BASE-T が 7 ms かかる」と読まれる。
    """
    mins = [r[0] for r in rows]
    offset = [r[1] for r in rows]
    delay = [r[2] for r in rows]

    fig, (ax_o, ax_d) = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)

    ax_o.plot(mins, offset, ".", ms=3, color=AFTER_C)
    ax_o.axhline(statistics.mean(offset), color="#1d5a3f", lw=1)
    ax_o.set_ylabel("offset (ms)")
    ax_o.text(
        0.01,
        0.06,
        f"mean {statistics.mean(offset):+.2f} ms   sd {statistics.pstdev(offset):.2f} ms   n={len(offset)}",
        transform=ax_o.transAxes,
        fontsize=9.5,
        color="#1d5a3f",
    )

    ax_d.plot(mins, delay, ".", ms=3, color="#2f6fb5")
    ax_d.set_ylabel("round trip (ms)")
    ax_d.set_xlabel("minutes")
    ax_d.text(
        0.01,
        0.06,
        f"median {statistics.median(delay):.2f} ms — the debug probe, not the network",
        transform=ax_d.transAxes,
        fontsize=9.5,
        color="#2f6fb5",
    )

    for ax in (ax_o, ax_d):
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    fig.suptitle("A client's view of pico-ntp over the SWD unicast path", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"wrote {path}")


def main() -> int:
    before_p, after_p = RAW / "unicast.csv", RAW / "unicast-inverted.csv"
    for p in (before_p, after_p):
        if not p.exists():
            print(f"missing raw log: {p}", file=sys.stderr)
            return 1
    before, after = load(before_p), load(after_p)
    print(summary("rising edge (before)", before))
    print(summary("falling edge (after)", after))

    for name in ("reference.csv", "reference-after.csv"):
        p = RAW / name
        if p.exists():
            r = load(p)
            best = min(r, key=lambda x: x[2])
            print(
                f"{name} (host vs public stratum 1): n={len(r)} "
                f"mean={statistics.mean([x[1] for x in r]):+.3f}ms "
                f"least-biased={best[1]:+.3f}ms at {best[2]:.1f}ms delay"
            )

    fig_edge(before, after, OUT / "fig-edge.png")
    fig_unicast(after, OUT / "fig-unicast.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
