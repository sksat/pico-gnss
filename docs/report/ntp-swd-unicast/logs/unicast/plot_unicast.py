# /// script
# requires-python = ">=3.11"
# dependencies = ["matplotlib"]
# ///
"""レポート `docs/report/ntp-swd-unicast/` の図を作る。

    uv run docs/report/ntp-swd-unicast/logs/unicast/plot_unicast.py

生データは repo top の `logs/20260819-ntp-unicast/` から読む (コミットしない)。
出力は `docs/report/ntp-swd-unicast/` 直下の PNG。

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


def fig_unicast(rows, path: Path):
    """クライアントから見た offset と往復時間。

    2 段に分けるのは、片方が系統的なずれで、もう片方が経路の揺らぎだから。重ねると
    「ずれているのか揺れているのか」が読めなくなる。

    往復時間には注釈を付ける。これは probe の経路であって Ethernet ではないので、注釈が
    ないと「10BASE-T が 7 ms かかる」と読まれる。
    """
    mins = [r[0] for r in rows]
    offset = [r[1] for r in rows]
    delay = [r[2] for r in rows]

    fig, (ax_o, ax_d) = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)

    ax_o.plot(mins, offset, ".", ms=3, color="#c1442e")
    ax_o.axhline(statistics.mean(offset), color="#7e2a1c", lw=1)
    ax_o.set_ylabel("offset (ms)")
    ax_o.text(
        0.01,
        0.06,
        f"mean {statistics.mean(offset):+.2f} ms   sd {statistics.pstdev(offset):.2f} ms   n={len(offset)}",
        transform=ax_o.transAxes,
        fontsize=9.5,
        color="#7e2a1c",
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
    src = RAW / "unicast.csv"
    if not src.exists():
        print(f"missing raw log: {src}", file=sys.stderr)
        return 1
    rows = load(src)
    offs = [r[1] for r in rows]
    print(
        f"unicast: n={len(rows)} mean={statistics.mean(offs):+.3f}ms "
        f"sd={statistics.pstdev(offs):.3f}ms min={min(offs):+.3f} max={max(offs):+.3f}"
    )

    ref = RAW / "reference.csv"
    if ref.exists():
        r = load(ref)
        ro = [x[1] for x in r]
        best = min(r, key=lambda x: x[2])
        print(
            f"reference (host vs public stratum 1): n={len(r)} mean={statistics.mean(ro):+.3f}ms "
            f"sd={statistics.pstdev(ro):.3f}ms  least-biased={best[1]:+.3f}ms at {best[2]:.1f}ms delay"
        )

    fig_unicast(rows, OUT / "fig-unicast.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
